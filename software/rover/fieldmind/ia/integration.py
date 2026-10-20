"""Façade de l'IA pour le reste du rover (serveur, exécuteur de mission).

    ia = Ia(bus, profil, outils)      # outils : fieldmind.app.OutilsRover
    await ia.demarrer()               # dans la boucle asyncio du serveur
    await ia.consigne("avance de 2 m", "texte")
    await ia.vision("chercher", "gauche", "chaise")
    ia.voix_active(True)              # le micro est démarré par le serveur
    await ia.pour_mission("chercher", "chaise")   # appelé par l'action de mission « chercher »

Aucune de ces méthodes ne commande les moteurs : l'agent dépose des actions dans la file de
mission, qui passent ensuite par la sécurité.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import math
import time
from typing import Any

from fieldmind.bus import Bus
from fieldmind.ia.agent import Agent
from fieldmind.ia.config import ConfigIA, ErreurConfigIA
from fieldmind.ia.distant import DetectionContinue, MemoireLieux, VoixRover
from fieldmind.ia.fournisseurs import ErreurIA
from fieldmind.ia.inference import ClientInference
from fieldmind.ia.messages import publier_chat
from fieldmind.ia.outils import ActionRefusee as RefusIa
from fieldmind.ia.transcription import Transcripteur
from fieldmind.ia.vision import Vision
from fieldmind.ia.voix import Voix
from fieldmind.mission.actions import ActionRefusee as RefusMission
from fieldmind.types import Action, ImageCamera, Pose2D

journal = logging.getLogger(__name__)


class _Outils:
    """Adapte les outils du rover au protocole de l'IA (exception de refus de l'IA)."""

    def __init__(self, outils: Any) -> None:
        self._o = outils

    def etat_resume(self) -> dict[str, Any]:
        return self._o.etat_resume()

    def ajouter_actions(self, actions: list[Action], remplacer: bool) -> str:
        try:
            return self._o.ajouter_actions(actions, remplacer)
        except RefusMission as refus:
            raise RefusIa(refus.raison) from refus

    def annuler(self) -> None:
        self._o.annuler()

    def arret_immediat(self, raison: str) -> None:
        try:
            self._o.arret_immediat(raison)
        except Exception:  # l'arrêt ne doit jamais lever
            journal.exception("arret_immediat")

    def derniere_image(self, camera: str) -> ImageCamera | None:
        return self._o.derniere_image(camera)

    def pixel_vers_sol(self, camera: str, u: float, v: float) -> tuple[float, float] | None:
        return self._o.pixel_vers_sol(camera, u, v)

    def mesurer_objet(self, camera: str, u: float, v: float,
                      boite: tuple[float, float, float, float] | None = None,
                      categorie: str | None = None) -> dict[str, Any] | None:
        mesurer = getattr(self._o, "mesurer_objet", None)
        return None if mesurer is None else mesurer(camera, u, v, boite, categorie)

    def detecter_camera(self, camera: str = "gauche") -> list[dict[str, Any]]:
        detecter = getattr(self._o, "detecter_camera", None)
        return [] if detecter is None else detecter(camera)

    def pose_odom(self) -> Pose2D:
        return self._o.pose_odom()


class Ia:
    def __init__(self, bus: Bus, profil: Any, outils: Any, config: ConfigIA | None = None) -> None:
        self.bus, self.profil = bus, profil
        self._config_explicite = config is not None
        self.config = config if config is not None else ConfigIA.charger()
        self.outils = _Outils(outils)
        self.inference = ClientInference(self.config)
        self.vision_ia = Vision(bus, profil, self.config, self.outils, self.inference)
        self.detection = DetectionContinue(bus, profil, self.inference, self.outils)
        self.voix_rover = VoixRover(bus, profil, self.inference)
        self.memoire = MemoireLieux(self.inference)
        self._taches: list[asyncio.Task[None]] = []
        self.agent = Agent(bus, profil, self.config, self.outils, vision=self.vision_ia)
        self.transcripteur = Transcripteur(self.config)
        self.voix_ia: Voix | None = None
        self._tache_vigilance: asyncio.Task[None] | None = None
        self._pose_vigilance: Pose2D | None = None

    async def demarrer(self) -> None:
        self.voix_ia = Voix(self.bus, self.profil, self.config, self.transcripteur,
                            lambda texte: self.agent.consigne(texte, origine="voix"),
                            asyncio.get_running_loop())
        disponible, raison = self.config.disponibilite("agent")
        if disponible:
            self.bus.journaliser("ia", f"profil « {self.config.nom_actif} » prêt", "succes")
        else:
            self.bus.journaliser("ia", f"profil « {self.config.nom_actif} » : {raison}", "alerte")
        if not self._config_explicite:
            self._tache_vigilance = asyncio.create_task(self._boucle_vigilance())
            # Outils du GX10 (profil gx10) : détection continue, voix du rover
            self._taches = [asyncio.create_task(self.detection.boucle()),
                            asyncio.create_task(self.voix_rover.boucle())]

    async def fermer(self) -> None:
        for tache in self._taches:
            tache.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await tache
        self._taches = []
        await self.inference.fermer()
        if self._tache_vigilance is not None:
            self._tache_vigilance.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._tache_vigilance
            self._tache_vigilance = None
        if self.voix_ia is not None:
            await self.voix_ia.fermer()
        await self.config.passerelle().fermer()

    async def _boucle_vigilance(self) -> None:
        """Analyse 3D autonome en arrière-plan (Vision + Lidar + Stéréo) au démarrage et à chaque
        déplacement ou rotation significatif du rover."""
        await asyncio.sleep(1.2)
        dernier_t = 0.0
        while True:
            try:
                maintenant = time.monotonic()
                pose = self.outils.pose_odom()
                deplace = (
                    self._pose_vigilance is None
                    or math.hypot(pose.x - self._pose_vigilance.x,
                                  pose.y - self._pose_vigilance.y) >= 0.35
                    or abs(math.atan2(math.sin(pose.cap - self._pose_vigilance.cap),
                                      math.cos(pose.cap - self._pose_vigilance.cap))) >= 0.22
                )
                # Analyse automatique : payante (une requête Gemini par image), donc seulement si
                # l'opérateur l'a activée (Réglages > IA), et pas plus souvent que le réglage.
                if not bool(self.profil.valeur("ia.vigilance", False)):
                    await asyncio.sleep(1.0)
                    continue
                periode = float(self.profil.valeur("ia.vigilance_periode_s", 12.0))
                if not self.occupee() and self.outils.derniere_image("gauche") is not None and (
                        (deplace and maintenant - dernier_t >= periode / 3)
                        or maintenant - dernier_t >= periode):
                    self._pose_vigilance = pose
                    dernier_t = maintenant
                    with contextlib.suppress(ErreurIA):
                        await self.vision_ia.analyser("gauche", silencieux=True)
            except asyncio.CancelledError:
                raise
            except Exception:
                journal.debug("vigilance autonome en attente", exc_info=True)
            await asyncio.sleep(1.0)

    # ------------------------------------------------------------------ demandes du poste

    async def consigne(self, texte: str, origine: str = "texte") -> None:
        try:
            await self.agent.consigne(texte, origine)
        except Exception as erreur:  # l'agent gère ses erreurs ; filet de sécurité
            journal.exception("consigne")
            publier_chat(self.bus, "systeme", f"IA en erreur : {erreur}")

    async def vision(self, demande: str, camera: str = "gauche", objet: str | None = None) -> None:
        try:
            if demande == "chercher":
                points = await self.vision_ia.chercher(str(objet or ""), camera)
                if not points:
                    publier_chat(self.bus, "rover", f"Je ne vois pas « {objet} ».")
            elif demande == "analyser":
                await self.vision_ia.analyser(camera)
            else:
                message = await self.vision_ia.decrire(camera)
                await self._lieu_reconnu(message)
        except ErreurIA as erreur:
            publier_chat(self.bus, "systeme", f"Vision indisponible : {erreur}")

    async def _lieu_reconnu(self, message: dict[str, Any]) -> None:
        """Après « que vois-tu ? » : à quel lieu mémorisé la vue ressemble (empreintes)."""
        if not self.memoire.souvenirs or not message.get("image"):
            return
        import base64

        try:
            trouve = await self.memoire.reconnaitre(base64.b64decode(message["image"]))
        except ErreurIA:
            return
        if trouve is not None:
            nom, score = trouve
            publier_chat(self.bus, "rover", f"Cette vue ressemble au lieu « {nom} » "
                                            f"({round(100 * score)} %).")

    async def memoriser_vue(self, nom: str, camera: str = "gauche") -> None:
        """Garde l'empreinte de la vue actuelle avec un lieu mémorisé (sans effet sans GX10)."""
        try:
            if await self.memoire.memoriser(nom, self.outils.derniere_image(camera)):
                self.bus.journaliser("ia", f"vue du lieu « {nom} » mémorisée")
        except ErreurIA as erreur:
            journal.info("vue du lieu non mémorisée : %s", erreur)

    def voix_active(self, active: bool) -> None:
        if self.voix_ia is not None:
            self.voix_ia.activer(active)

    async def changer_profil(self, nom: str) -> dict[str, Any]:
        try:
            self.config.changer_profil(nom)
        except ErreurConfigIA as erreur:
            raise RefusMission(str(erreur)) from erreur
        self._pose_vigilance = None
        self.bus.journaliser("ia", f"profil IA : {nom}", "succes")
        return self.resume()

    def definir_cle(self, cle: str, nom: str | None = None) -> dict[str, Any]:
        self.config.definir_cle(cle, nom)
        self._pose_vigilance = None
        disponible, _ = self.config.disponibilite("agent")
        self.bus.journaliser("ia", "clé API Gemini enregistrée" if disponible
                             else "clé API Gemini retirée", "succes" if disponible else "alerte")
        return self.resume()

    # ------------------------------------------------------------------ état

    def resume(self) -> dict[str, Any]:
        return self.config.resume()

    def etat_voix(self) -> dict[str, Any]:
        return self.voix_ia.etat() if self.voix_ia is not None else {"active": False}

    def occupee(self) -> bool:
        voix = self.voix_ia.occupee if self.voix_ia is not None else False
        return bool(self.agent.occupee or self.vision_ia.occupee or voix)

    # ------------------------------------------------------------------ mission

    async def pour_mission(self, demande: str, argument: Any) -> Any:
        """Appelé par l'exécuteur (actions « chercher » et « decrire »)."""
        if demande == "chercher":
            objet, camera = (argument if isinstance(argument, tuple) else (argument, "gauche"))
            return await self.vision_ia.chercher(str(objet), camera or "gauche")
        if demande == "decrire":
            return await self.vision_ia.decrire(argument or "gauche")
        raise ErreurIA(f"demande inconnue : {demande}")
