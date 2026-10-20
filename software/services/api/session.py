"""Session d'un onglet du poste opérateur : un robot, un cerveau et le WebSocket.

Le robot est soit un rover simulé dans ce processus, soit un robot distant (rover réel ou
simulateur séparé) connecté sur /ws/robot. C'est ici, et seulement ici, que le simulateur est
branché au cerveau (racine de composition). Le cerveau ne voit que la `LiaisonRobot` : il ignore
s'il pilote un robot simulé ou réel.
"""

from __future__ import annotations

import asyncio
import base64
import binascii
import contextlib
import json
import logging
import time
import uuid
from dataclasses import asdict, dataclass
from typing import Any

from fastapi import WebSocket, WebSocketDisconnect
from pydantic import ValidationError

from services.api.config import Reglages
from services.api.journal import JournalJsonl
from services.cerveau.cerveau import Cerveau
from services.cerveau.evenements import ChatReponse
from services.mission_planner.consignes import (
    Interpretation,
    InterpreteConsignes,
    QuotaJournalier,
)
from services.mission_planner.intentions import Observer, Refus, traduire
from services.perception.vision import (
    DemandeVision,
    FournisseurVision,
    ResultatVision,
    executer_analyse,
)
from services.protocole.fiche_rover import FicheRover
from services.protocole.liaison import LiaisonRobot, MessageRobot
from services.protocole.messages import (
    Chat,
    CommandeVision,
    ImageStereo,
    Imu,
    Objectif,
    Odometrie,
    Reinitialiser,
    Reprendre,
    Stop,
    Teleoperation,
    lire_message_operateur,
)
from services.protocole.scenario import ScenarioPublic
from simulation.monde import Monde
from simulation.robot_simule import RobotSimule
from simulation.temps_reel import SimulationTempsReel
from simulation.vision_oracle import DetecteurVerite

journal = logging.getLogger("fieldmind.session")

FREQUENCE_VUE_HZ = 30.0
TYPES_JOURNALISES = {"journal", "statut", "mesures", "chat_reponse"}
TAILLE_MAX_MESSAGE = 4096
DEBIT_OPERATEUR_PAR_S = 60.0
"""Messages de l'opérateur acceptés par seconde (la téléopération en envoie 20)."""
DECIMATION = {"imu": 2, "odometrie": 2}
"""Messages retransmis au poste opérateur : un sur N (les autres capteurs le sont tous)."""


def _json(objet: dict[str, Any]) -> str:
    return json.dumps(objet, separators=(",", ":"), ensure_ascii=False)


@dataclass(slots=True)
class VisionPartagee:
    """Vision commune aux sessions : modèle (OpenRouter ou vLLM) et quota journalier.

    Sans modèle, `simulation` autorise le détecteur parfait dans les sessions simulées.
    """

    fournisseur: FournisseurVision | None
    simulation: bool
    quota: QuotaJournalier

    @property
    def nom(self) -> str:
        if self.fournisseur is not None:
            return self.fournisseur.nom
        return DetecteurVerite.nom if self.simulation else "aucune"


class Session:
    """Durée de vie : de l'ouverture du WebSocket à sa fermeture ou à l'inactivité."""

    def __init__(
        self,
        ws: WebSocket,
        reglages: Reglages,
        fiche: FicheRover,
        scenario: ScenarioPublic,
        monde: Monde | None,
        consignes: InterpreteConsignes,
        journal_jsonl: JournalJsonl | None = None,
        robot_distant: LiaisonRobot | None = None,
        identifiant: str | None = None,
        vision: VisionPartagee | None = None,
        robot_simule: RobotSimule | None = None,
    ) -> None:
        self.id = identifiant or uuid.uuid4().hex[:8]
        self.ws = ws
        self.reglages = reglages
        self.fiche = fiche
        self.scenario = scenario
        self.monde = monde
        self.simulation: SimulationTempsReel | None = None
        if robot_distant is not None:
            self.liaison: LiaisonRobot = robot_distant
        elif monde is not None:
            self.simulation = SimulationTempsReel(
                robot_simule or RobotSimule(fiche, monde, graine_bruit=monde.graine)
            )
            self.liaison = self.simulation
        else:
            raise ValueError("il faut un monde simulé ou un robot distant")
        self.vision = vision
        self.fournisseur_vision: FournisseurVision | None = None
        if vision is not None and vision.fournisseur is not None:
            self.fournisseur_vision = vision.fournisseur
        elif vision is not None and vision.simulation and self.simulation is not None:
            simulation = self.simulation
            self.fournisseur_vision = DetecteurVerite(lambda: simulation.robot)
        self._vision_dispo = asyncio.Event()
        self._analyses_vision = 0
        self.cerveau = self._nouveau_cerveau()
        self._envoi: asyncio.Queue[str] = asyncio.Queue(maxsize=400)
        self._travail_dispo = asyncio.Event()
        self._duree_cerveau = 0.0
        self._compteurs: dict[str, int] = {}
        self._derniere_activite = time.monotonic()
        self.telemetrie_perdue = 0
        self.consignes = consignes
        self.journal_jsonl = journal_jsonl
        self._messages_chat = 0
        self._chat_en_cours: asyncio.Task[None] | None = None

    def _nouveau_cerveau(self) -> Cerveau:
        cerveau = Cerveau(self.fiche, self.scenario)
        cerveau.vision_disponible = self.fournisseur_vision is not None
        cerveau.periode_vigilance = self.reglages.vision_periode_s
        if cerveau.vision_disponible and self.reglages.vision_vigilance:
            cerveau.definir_vigilance(True)
        return cerveau

    # ------------------------------------------------------------------ cycle de vie
    async def executer(self) -> None:
        # Envoyés directement : ces deux messages doivent précéder toute télémétrie.
        await self.ws.send_text(_json(self._bienvenue()))
        await self.ws.send_text(
            _json(self.cerveau.evenement_carte(0.0, complete=True).model_dump())
        )
        if self.simulation is not None:
            self.simulation.demarrer()
        taches = [
            asyncio.create_task(self._boucle_robot(), name=f"robot-{self.id}"),
            asyncio.create_task(self._boucle_perception(), name=f"perception-{self.id}"),
            asyncio.create_task(self._mesurer_charge(), name=f"charge-{self.id}"),
            asyncio.create_task(self._boucle_envoi(), name=f"envoi-{self.id}"),
            asyncio.create_task(self._boucle_reception(), name=f"reception-{self.id}"),
            asyncio.create_task(self._surveiller_inactivite(), name=f"inactivite-{self.id}"),
            asyncio.create_task(self._boucle_vision(), name=f"vision-{self.id}"),
        ]
        if self.simulation is not None:
            taches.append(asyncio.create_task(self._boucle_vue(), name=f"vue-{self.id}"))
        try:
            fini, _ = await asyncio.wait(taches, return_when=asyncio.FIRST_COMPLETED)
            for tache in fini:
                erreur = None if tache.cancelled() else tache.exception()
                if erreur is not None and not isinstance(erreur, WebSocketDisconnect):
                    journal.exception("session %s interrompue", self.id, exc_info=erreur)
        finally:
            for tache in taches:
                tache.cancel()
            for tache in taches:
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await tache
            await self.liaison.fermer()

    def _bienvenue(self) -> dict[str, Any]:
        return {
            "type": "bienvenue",
            "session": self.id,
            "scenario": self.scenario.model_dump(),
            "rover": self.fiche.model_dump(
                include={"nom", "geometrie", "limites", "securite", "capteurs"}
            ),
            "mode": "simulation" if self.simulation is not None else "robot_reel",
            "monde": self.monde.en_json() if self.simulation is not None and self.monde else None,
            "chat": {
                "fournisseur": self.consignes.nom,
                "max_messages": self.reglages.chat_max_par_session,
            },
            "vision": {
                "fournisseur": None
                if self.fournisseur_vision is None
                else self.fournisseur_vision.nom,
                "max_analyses": self.reglages.vision_max_par_session,
                "periode_s": self.reglages.vision_periode_s,
            },
            "inactivite_s": self.reglages.session_inactivite_s,
        }

    # ------------------------------------------------------------------ robot → cerveau
    async def _boucle_robot(self) -> None:
        while True:
            message = await self.liaison.recevoir()
            debut = time.perf_counter()
            commande = self.cerveau.sur_message_robot(message)
            self._duree_cerveau += time.perf_counter() - debut
            if commande is not None:
                await self.liaison.envoyer(commande)
                self._envoyer_evenement(commande.model_dump())
            if message.type == "scan":
                self._travail_dispo.set()
            elif message.type == "image":
                self._vision_dispo.set()
            self._retransmettre(message)
            self._vider_cerveau()

    async def _boucle_perception(self) -> None:
        """Flux lent du cerveau dans un fil : un seul balayage à la fois, le plus récent."""
        while True:
            await self._travail_dispo.wait()
            self._travail_dispo.clear()
            cerveau = self.cerveau  # inchangé pendant le calcul, même en cas de réinitialisation
            travail = cerveau.prendre_travail()
            if travail is None:
                continue
            debut = time.perf_counter()
            resultat = await asyncio.to_thread(cerveau.traiter_perception, travail)
            self._duree_cerveau += time.perf_counter() - debut
            if cerveau is self.cerveau:
                cerveau.appliquer(resultat)
                self._vider_cerveau()

    async def _mesurer_charge(self) -> None:
        while True:
            await asyncio.sleep(1.0)
            charge, self._duree_cerveau = self._duree_cerveau, 0.0
            precedente = self.cerveau.charge_cpu
            self.cerveau.charge_cpu = (
                charge if precedente is None else 0.7 * precedente + 0.3 * charge
            )

    def _retransmettre(self, message: MessageRobot) -> None:
        """Télémétrie brute vers le poste opérateur, éventuellement décimée. La paire stéréo
        n'est pas retransmise : le poste reçoit la profondeur calculée par le cerveau."""
        if isinstance(message, ImageStereo):
            return
        if isinstance(message, Imu | Odometrie):
            n = self._compteurs.get(message.type, 0) + 1
            self._compteurs[message.type] = n
            if n % DECIMATION[message.type]:
                return
        self._envoyer_evenement(message.model_dump())

    def _vider_cerveau(self) -> None:
        for evenement in self.cerveau.vider_sortie():
            donnees = evenement.model_dump()
            self._envoyer_evenement(donnees, prioritaire=True)
            if self.journal_jsonl is not None and evenement.type in TYPES_JOURNALISES:
                self.journal_jsonl.ecrire(self.id, donnees)

    # ------------------------------------------------------------------ vue 3D
    async def _boucle_vue(self) -> None:
        periode = 1.0 / FREQUENCE_VUE_HZ
        simulation = self.simulation
        assert simulation is not None
        while True:
            await asyncio.sleep(periode)
            verite = asdict(simulation.verite())
            self._envoyer_evenement(
                {
                    "type": "vue",
                    **verite,
                    "cpu_simulation": round(simulation.charge_cpu, 4),
                    "cpu_camera": round(simulation.charge_camera, 4),
                    "erreur_camera": simulation.erreur_camera,
                    "telemetrie_perdue": self.telemetrie_perdue,
                }
            )

    # ------------------------------------------------------------------ WebSocket
    def _envoyer_evenement(self, evenement: dict[str, Any], prioritaire: bool = False) -> None:
        texte = _json(evenement)
        try:
            self._envoi.put_nowait(texte)
        except asyncio.QueueFull:
            if not prioritaire:
                self.telemetrie_perdue += 1
                return
            # Un événement du cerveau ne se perd pas : on sacrifie la plus ancienne télémétrie.
            with contextlib.suppress(asyncio.QueueEmpty):
                self._envoi.get_nowait()
            self._envoi.put_nowait(texte)

    async def _boucle_envoi(self) -> None:
        while True:
            texte = await self._envoi.get()
            await self.ws.send_text(texte)

    async def _boucle_reception(self) -> None:
        jetons, t_jetons = DEBIT_OPERATEUR_PAR_S, time.monotonic()
        while True:
            texte = await self.ws.receive_text()
            # Seau à jetons : au-delà du débit autorisé, le message est ignoré.
            maintenant = time.monotonic()
            jetons = min(
                DEBIT_OPERATEUR_PAR_S, jetons + (maintenant - t_jetons) * DEBIT_OPERATEUR_PAR_S
            )
            t_jetons = maintenant
            if jetons < 1:
                continue
            jetons -= 1
            if len(texte) > TAILLE_MAX_MESSAGE:
                self._erreur("message trop long")
                continue
            try:
                message = lire_message_operateur.validate_json(texte)
            except ValidationError as erreur:
                self._erreur(f"message invalide : {erreur.errors()[0]['msg']}")
                continue
            if not isinstance(message, Teleoperation) or message.avance or message.direction:
                self._derniere_activite = time.monotonic()
            if isinstance(message, Teleoperation):
                self.cerveau.sur_teleoperation(message)
            elif isinstance(message, Stop):
                self.cerveau.sur_stop(message)
            elif isinstance(message, Objectif):
                refus = self.cerveau.definir_objectif(message.x, message.y, "carte")
                if refus:
                    self.cerveau.journal(f"Objectif refusé : {refus}.", "alerte")
            elif isinstance(message, Reprendre):
                refus = self.cerveau.reprendre()
                if refus:
                    self.cerveau.journal(f"Reprise impossible : {refus}.", "alerte")
            elif isinstance(message, Reinitialiser):
                self._reinitialiser()
            elif isinstance(message, Chat):
                self._recevoir_chat(message.texte)
            elif isinstance(message, CommandeVision):
                self._commande_vision(message)
            self._vider_cerveau()

    # ------------------------------------------------------------------ vision
    def _commande_vision(self, message: CommandeVision) -> None:
        if message.action == "analyser":
            refus = self.cerveau.demander_analyse()
            if refus:
                self.cerveau.journal(f"Analyse impossible : {refus}.", "alerte")
            self._vision_dispo.set()
        else:
            refus = self.cerveau.definir_vigilance(message.action == "vigilance_on")
            if refus:
                self.cerveau.journal(f"Vigilance impossible : {refus}.", "alerte")

    async def _boucle_vision(self) -> None:
        """Analyses de la caméra demandées par le cerveau, une à la fois."""
        while True:
            await self._vision_dispo.wait()
            self._vision_dispo.clear()
            cerveau = self.cerveau
            demande = cerveau.prendre_demande_vision()
            if demande is None:
                continue
            resultat = await self._analyser(demande)
            if cerveau is not self.cerveau:
                continue
            texte = cerveau.sur_resultat_vision(demande, resultat)
            if demande.raison == "operateur":
                self._repondre_chat(
                    texte + (f" « {resultat.analyse.resume} »" if resultat.analyse else ""),
                    refus=resultat.erreur,
                    fournisseur=resultat.fournisseur,
                    duree_ms=resultat.duree_ms,
                )
            self._enregistrer_preuve(demande, resultat, texte)
            self._vider_cerveau()

    async def _analyser(self, demande: DemandeVision) -> ResultatVision:
        fournisseur = self.fournisseur_vision
        if fournisseur is None:
            return ResultatVision(None, "serveur", 0.0, "aucun modèle de vision")
        maximum = self.reglages.vision_max_par_session
        if self._analyses_vision >= maximum:
            self.cerveau.definir_vigilance(False)
            return ResultatVision(None, "serveur", 0.0, f"limite de {maximum} analyses par session")
        simule = isinstance(fournisseur, DetecteurVerite)
        if not simule and self.vision is not None and not self.vision.quota.prendre():
            self.cerveau.definir_vigilance(False)
            return ResultatVision(None, "serveur", 0.0, "quota journalier d'analyses atteint")
        self._analyses_vision += 1
        return await executer_analyse(fournisseur, demande)

    def _enregistrer_preuve(
        self, demande: DemandeVision, resultat: ResultatVision, texte: str
    ) -> None:
        if self.journal_jsonl is None or not demande.image_jpeg_b64:
            return
        try:
            jpeg = base64.b64decode(demande.image_jpeg_b64, validate=True)
        except (binascii.Error, ValueError):
            return
        self.journal_jsonl.ecrire_preuve(
            self.id,
            demande.id,
            jpeg,
            {
                "t_image": demande.t,
                "raison": demande.raison,
                "pose_estimee": {"x": demande.x, "y": demande.y, "cap": demande.cap},
                "fournisseur": resultat.fournisseur,
                "duree_ms": round(resultat.duree_ms, 1),
                "erreur": resultat.erreur,
                "analyse": resultat.analyse.model_dump() if resultat.analyse else None,
                "reponse": texte,
            },
        )

    def _recevoir_chat(self, texte: str) -> None:
        maximum = self.reglages.chat_max_par_session
        if self._messages_chat >= maximum:
            self._repondre_chat(
                f"Limite de {maximum} consignes atteinte pour cette session : utilise la carte.",
                refus="quota de session",
                fournisseur="serveur",
            )
            return
        if self._chat_en_cours is not None and not self._chat_en_cours.done():
            self._repondre_chat(
                "Une consigne est déjà en cours d'analyse.", refus="occupé", fournisseur="serveur"
            )
            return
        self._messages_chat += 1
        self._chat_en_cours = asyncio.create_task(self._traiter_chat(texte))

    async def _traiter_chat(self, texte: str) -> None:
        interpretation = await self.consignes.interpreter(texte)
        cerveau = self.cerveau
        if interpretation.intention is None:
            echec = interpretation.echec or "modèle indisponible"
            self._repondre_chat(
                f"Consigne non exécutée : {echec}. Réessaie dans un instant, ou clique sur la "
                "carte.",
                refus=echec,
                fournisseur=interpretation.fournisseur,
                duree_ms=interpretation.duree_ms,
            )
            self._journaliser_consigne(texte, interpretation, echec)
            return
        action, reponse = traduire(interpretation.intention, self.scenario, cerveau.pose_courante())
        refus = action.raison if isinstance(action, Refus) else None
        if not isinstance(action, Refus):
            refus = cerveau.executer(action)
            if refus:
                reponse = f"Consigne refusée : {refus}."
            elif isinstance(action, Observer):
                self._vision_dispo.set()
        self._repondre_chat(
            reponse,
            refus=refus,
            fournisseur=interpretation.fournisseur,
            intention=interpretation.intention.model_dump(),
            duree_ms=interpretation.duree_ms,
        )
        self._journaliser_consigne(texte, interpretation, refus)
        self._vider_cerveau()

    def _journaliser_consigne(
        self, texte: str, interpretation: Interpretation, refus: str | None
    ) -> None:
        if self.journal_jsonl is None:
            return
        intention = interpretation.intention
        self.journal_jsonl.ecrire(
            self.id,
            {
                "type": "consigne",
                "texte": texte,
                "intention": intention.model_dump() if intention else None,
                "fournisseur": interpretation.fournisseur,
                "duree_ms": round(interpretation.duree_ms, 1),
                "refus": refus,
            },
        )

    def _repondre_chat(
        self,
        texte: str,
        refus: str | None,
        fournisseur: str,
        intention: dict[str, object] | None = None,
        duree_ms: float | None = None,
    ) -> None:
        reponse = ChatReponse(
            t=self.cerveau.t,
            texte=texte,
            intention=intention,
            refus=refus,
            fournisseur=fournisseur,
            duree_ms=None if duree_ms is None else round(duree_ms, 1),
        )
        self._envoyer_evenement(reponse.model_dump(), prioritaire=True)

    def _reinitialiser(self) -> None:
        """Rover simulé : remis au départ. Dans tous les cas : cerveau neuf (carte vide)."""
        if self.simulation is not None and self.monde is not None:
            self.simulation.remplacer_robot(
                RobotSimule(self.fiche, self.monde, graine_bruit=self.monde.graine)
            )
        self.cerveau = self._nouveau_cerveau()
        self._envoyer_evenement({"type": "reinitialisation"}, prioritaire=True)
        self._envoyer_evenement(
            self.cerveau.evenement_carte(0.0, complete=True).model_dump(), prioritaire=True
        )

    def _erreur(self, texte: str) -> None:
        self._envoyer_evenement({"type": "erreur", "texte": texte}, prioritaire=True)

    async def _surveiller_inactivite(self) -> None:
        delai = self.reglages.session_inactivite_s
        while True:
            await asyncio.sleep(min(delai, 5.0))
            if time.monotonic() - self._derniere_activite > delai:
                self._envoyer_evenement(
                    {"type": "fin_session", "raison": "inactivite"}, prioritaire=True
                )
                await asyncio.sleep(0.5)
                return
