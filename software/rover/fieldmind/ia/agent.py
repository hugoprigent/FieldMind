"""Agent : consigne écrite ou dite → appel de fonctions du modèle → plan validé → file de mission.

    agent = Agent(bus, profil, config_ia, outils, vision=vision)
    resultat = await agent.consigne("avance de 5 m puis tourne à gauche", origine="voix")

- Les ordres d'arrêt (« stop », « arrête », « halte »…) sont reconnus sans modèle et appliqués
  tout de suite (`outils.arret_immediat`). Un plan encore en cours d'analyse est alors abandonné.
- Le modèle ne fait que proposer : `outils.valider_plan` relit chaque appel ; le texte affiché à
  l'opérateur pour un plan est composé par le code, pas par le modèle.
- Sans modèle (profil « aucun », clé absente, quota, panne) : message clair, rien n'est exécuté.
- Une phrase dite qui n'est pas une consigne (bruit, conversation à côté) est ignorée : événement
  `voix` `ignore`, pas de réponse dans le chat.
"""

from __future__ import annotations

import asyncio
import collections
import json
import logging
import re
import unicodedata
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from fieldmind.bus import Bus
from fieldmind.ia.config import ConfigIA
from fieldmind.ia.fournisseurs import ErreurIA, IAIndisponible
from fieldmind.ia.messages import plan_pour_chat, publier_chat, publier_voix
from fieldmind.ia.outils import (
    ActionInvalide,
    ActionRefusee,
    ContexteValidation,
    OutilsRover,
    decrire_plan,
    gps_depuis_etat,
    lieux_depuis_etat,
    schemas_fonctions,
    valider_plan,
)
from fieldmind.types import Action, Pose2D

if TYPE_CHECKING:
    from fieldmind.ia.vision import Vision

journal = logging.getLogger("fieldmind.ia.agent")

# ------------------------------------------------------------------ arrêt sans modèle

MOTS_ARRET = frozenset({
    "stop", "stoppe", "stoppes", "stopper", "stoppez", "arrete", "arretes", "arretez",
    "arreter", "arret", "halte", "freine", "freinez", "immobilise", "urgence",
})
MOTS_AVANT = frozenset({"rover", "robot", "fieldmind", "eh", "he", "hey", "allez", "ok", "bon",
                        "alors", "oh", "mais", "non", "attention", "vite", "s", "on", "t",
                        "il", "te", "plait", "roveur", "robert"})
MOTS_COMPLEMENT = frozenset({"toi", "vous", "tout", "tous", "de", "suite", "maintenant",
                             "immediatement", "vite", "la", "ca", "le", "rover", "s", "il",
                             "te", "plait", "on", "t", "net", "et", "puis", "urgence", "d"})
NEGATIONS = frozenset({"pas", "jamais", "ne", "n", "plus"})
REMPLISSAGE = frozenset({"euh", "heu", "hum", "hmm", "mmh", "mh", "ah", "oh", "bah", "ben",
                         "hein", "bof", "pff", "ha", "hé", "he"})
HALLUCINATIONS = ("sous-titres", "sous titres", "amara.org", "merci d'avoir regardé",
                  "abonnez-vous", "sous-titrage")
"""Phrases que les modèles de transcription produisent parfois sur du bruit."""


def _normaliser(mot: str) -> str:
    decompose = unicodedata.normalize("NFD", mot.lower())
    return "".join(c for c in decompose if not unicodedata.combining(c))


def _mots(texte: str) -> list[tuple[str, int]]:
    """Mots normalisés (minuscules, sans accents) et leur position dans le texte d'origine."""
    return [(_normaliser(m.group()), m.start()) for m in re.finditer(r"\w+", texte)]


def detecter_arret(texte: str) -> tuple[bool, str]:
    """(ordre d'arrêt ?, reste de la phrase à traiter comme une consigne ou "").

    Arrêt si la phrase commence par un mot d'arrêt (« stop, recule de 1 m » → reste « recule de
    1 m ») ou n'est faite que d'un ordre d'arrêt (« on s'arrête tout de suite »). Pas d'arrêt
    sur « ne t'arrête pas » ni sur « va jusqu'à l'arrêt de bus ».
    """
    mots = _mots(texte)
    i = 0
    while i < len(mots) and mots[i][0] in MOTS_AVANT:
        i += 1
    if i >= len(mots):
        return False, ""
    if mots[i][0] not in MOTS_ARRET:
        normalises = {m for m, _ in mots}
        if not (normalises & MOTS_ARRET) or normalises & NEGATIONS:
            return False, ""
        autorises = MOTS_ARRET | MOTS_AVANT | MOTS_COMPLEMENT
        return (all(m in autorises for m, _ in mots), "")
    if i + 1 < len(mots) and mots[i + 1][0] in NEGATIONS:
        return False, ""
    j = i + 1
    while j < len(mots) and mots[j][0] in MOTS_COMPLEMENT | MOTS_ARRET:
        j += 1
    reste = texte[mots[j][1]:].strip() if j < len(mots) else ""
    if len(_mots(reste)) < 2:
        reste = ""
    return True, reste


def est_bruit(texte: str) -> str | None:
    """Raison d'ignorer une transcription sans la donner au modèle (None si elle passe)."""
    mots = [m for m, _ in _mots(texte)]
    if not mots:
        return "rien d'intelligible"
    if all(m in REMPLISSAGE for m in mots):
        return "hésitation"
    bas = texte.lower()
    if any(h in bas for h in HALLUCINATIONS):
        return "bruit (transcription parasite)"
    return None


# ------------------------------------------------------------------ consigne du modèle

SYSTEME = """Tu es l'agent de pilotage de FieldMind, un petit rover terrestre à quatre roues \
(direction avant de type voiture : il tourne sur un arc et ne pivote pas sur place). Tu \
transformes la consigne de l'opérateur, écrite ou dite, en appels de fonctions : une fonction par \
action, dans l'ordre d'exécution. Le programme du rover valide chaque action, la navigation \
évite seule les obstacles et la sécurité peut ralentir ou refuser.

Règles :
- Une consigne composée (« avance de 5 m puis tourne à gauche et cherche la chaise ») donne \
plusieurs appels, dans l'ordre.
- Distances en mètres (50 cm = 0.5), angles en degrés, positifs vers la gauche : « à gauche » = \
90, « à droite » = -90, « demi-tour » = 180. Reculer : avancer avec une distance négative.
- N'invente pas de valeur. « Tourne à gauche » = 90°. Si une distance manque (« avance »), \
demande-la en une phrase courte, sans appeler de fonction.
- Par défaut une consigne remplace la mission en cours ; appelle d'abord mettre_a_la_suite si \
l'opérateur dit « ensuite », « après ça », « ajoute ».
- Pour regarder (« que vois-tu ? », « y a-t-il quelqu'un ? ») appelle decrire ; pour trouver ou \
rejoindre un objet appelle chercher (aller=false s'il demande seulement où il est). Ne prétends \
jamais avoir vu quelque chose.
- Une question sur le rover (position, état, mission) : réponds en une phrase courte en français \
à partir de l'état ci-dessous, sans appeler de fonction.
- « Mémorise cet endroit comme le garage » : memoriser_lieu(nom="garage"). Pour aller à un \
lieu mémorisé (« retourne au garage ») : aller_lieu, seulement vers un lieu de la liste `lieux` \
de l'état ; sinon dis qu'il est inconnu.
- Des coordonnées GPS données par l'opérateur (« va au point 43.7105, -1.0531 ») : aller_gps \
(latitude puis longitude, degrés décimaux). L'état `gps` dit si une position GPS existe (fix) \
et si elle est alignée sur la carte (aligne).
- Respecte les bornes des paramètres ; au-delà, explique en une phrase que c'est hors limites.
- Si la phrase n'est manifestement pas adressée au rover (bruit, conversation à côté, phrase \
coupée ou sans rapport), appelle ignorer."""

VOIX = """La consigne vient de la reconnaissance vocale du micro du rover : elle peut contenir \
des erreurs de transcription (« avant de deux mètres » pour « avance de deux mètres ») ou des \
paroles qui ne lui sont pas adressées."""


@dataclass(slots=True)
class ResultatConsigne:
    """statut : plan | reponse | arret | annule | ignore | refus | indisponible | erreur |
    abandonne | vision."""

    statut: str
    texte: str = ""
    plan_id: str | None = None
    actions: list[Action] = field(default_factory=list)


@dataclass(slots=True)
class _Echange:
    """Une consigne en cours de traitement. Dite, elle n'apparaît dans le chat qu'avec la
    première réponse : une phrase ignorée (conversation à côté) n'encombre pas la conversation
    (la transcription reste visible par l'événement `voix`)."""

    texte: str
    origine: str
    publie: bool = False

    def publier(self, bus: Bus) -> None:
        if not self.publie:
            self.publie = True
            publier_chat(bus, "operateur", self.texte, self.origine)


def _valeur_profil(profil: Any, chemin: str, defaut: float) -> float:
    try:
        valeur = profil.valeur(chemin, defaut)
        return float(valeur) if valeur is not None else defaut
    except (AttributeError, TypeError, ValueError):
        return defaut


class Agent:
    def __init__(self, bus: Bus, profil: Any, config_ia: ConfigIA, outils: OutilsRover,
                 vision: Vision | None = None) -> None:
        self.bus = bus
        self.profil = profil
        self.config_ia = config_ia
        self.outils = outils
        self.vision = vision
        """Si fournie : un plan fait seulement de « decrire » ou « chercher sans y aller »
        s'exécute tout de suite, même rover désarmé (aucun mouvement)."""
        n = max(0, config_ia.agent.historique_echanges)
        self._historique: collections.deque[dict[str, str]] = collections.deque(maxlen=2 * n)
        self._verrou = asyncio.Lock()
        self._generation = 0  # incrémentée à chaque arrêt : les plans en vol sont abandonnés

    @property
    def occupee(self) -> bool:
        return self._verrou.locked()

    def effacer_historique(self) -> None:
        self._historique.clear()

    # ------------------------------------------------------------------ consigne

    async def consigne(self, texte: str, origine: str = "texte") -> ResultatConsigne:
        texte = " ".join(texte.split())
        if origine == "voix":
            raison = est_bruit(texte)
            if raison is not None:
                publier_voix(self.bus, "ignore", texte=texte, raison=raison)
                return ResultatConsigne("ignore", raison)
        if not texte:
            return ResultatConsigne("ignore", "consigne vide")
        echange = _Echange(texte, origine)
        if origine != "voix":
            echange.publier(self.bus)

        arret, reste = detecter_arret(texte)
        if arret:
            self._generation += 1
            raison = "consigne vocale" if origine == "voix" else "consigne écrite"
            try:
                self.outils.arret_immediat(raison)
            except Exception:  # l'arrêt doit être tenté même si l'intégration a un souci
                journal.exception("arret_immediat en erreur")
            self._repondre(echange, "Je m'arrête.")
            self._memoriser(texte, "Je m'arrête.")
            if reste:
                return await self._interpreter(_Echange(reste, origine, publie=True))
            return ResultatConsigne("arret", "Je m'arrête.")
        return await self._interpreter(echange)

    async def _interpreter(self, echange: _Echange) -> ResultatConsigne:
        try:
            self.config_ia.passerelle().acces("agent")
        except IAIndisponible as erreur:
            return self._systeme(echange, f"Consigne non exécutée : {erreur}.", "indisponible")
        async with self._verrou:
            decision = await self._decider(echange)
        if isinstance(decision, ResultatConsigne):
            return decision
        # Plan sans mouvement (decrire, chercher sans y aller), hors du verrou : la vision peut
        # prendre quelques secondes et ne doit pas retenir la consigne suivante.
        echange.publier(self.bus)
        return await self._regarder(echange.texte, decision)

    async def _decider(self, echange: _Echange) -> ResultatConsigne | list[Action]:
        texte, origine = echange.texte, echange.origine
        generation = self._generation
        try:
            etat = self.outils.etat_resume()
        except Exception:
            journal.exception("etat_resume en erreur")
            etat = {}
        lieux = lieux_depuis_etat(etat)
        vitesse_max = _valeur_profil(self.profil, "vitesses.max_auto_m_s", 0.5)
        try:
            reponse = await self.config_ia.passerelle().chat(
                "agent", self._messages(texte, origine, etat),
                outils=schemas_fonctions(vitesse_max, lieux))
        except ErreurIA as erreur:
            return self._systeme(echange, f"Consigne non exécutée : {erreur}.",
                                 "indisponible" if isinstance(erreur, IAIndisponible)
                                 else "erreur")
        if generation != self._generation:
            return ResultatConsigne("abandonne", "arrêt demandé pendant l'analyse")
        try:
            pose = self.outils.pose_odom()
        except Exception:
            pose = Pose2D()
        contexte = ContexteValidation(
            vitesse_max_m_s=vitesse_max, pose=pose, lieux=lieux, gps_fix=gps_depuis_etat(etat),
            geofence_m=_valeur_profil(self.profil, "securite.geofence_m", 60.0),
            actions_max=self.config_ia.agent.actions_max)
        try:
            plan = valider_plan(reponse.appels, contexte)
        except ActionInvalide as erreur:
            return self._refuser(echange, f"Je ne peux pas : {erreur}.")

        if plan.ignorer and not plan.actions:
            if origine == "voix":
                publier_voix(self.bus, "ignore", texte=texte,
                             raison=plan.raison_ignorer or "pas une consigne")
                return ResultatConsigne("ignore", plan.raison_ignorer)
            return self._refuser(echange, reponse.texte[:400] or
                                 "Je n'ai pas compris la consigne.", "reponse")
        if plan.annuler and not plan.actions:
            self.outils.annuler()
            return self._refuser(echange, "J'annule la mission en cours.", "annule")
        if not plan.actions:
            return self._refuser(echange, reponse.texte[:400] or
                                 "Je n'ai pas compris la consigne.", "reponse")
        if [a["type"] for a in plan.actions] == ["stop"]:
            self._generation += 1
            self.outils.arret_immediat("demande de l'opérateur")
            return self._refuser(echange, "Je m'arrête.", "arret")
        if self.vision is not None and all(_sans_mouvement(a) for a in plan.actions):
            return plan.actions
        try:
            plan_id = self.outils.ajouter_actions(plan.actions, plan.remplacer)
        except ActionRefusee as erreur:
            return self._refuser(echange, f"Refusé : {erreur.raison}.")
        except Exception as erreur:  # exception de la mission non convertie : refus aussi
            journal.warning("ajouter_actions : %r", erreur)
            raison = getattr(erreur, "raison", None) or str(erreur) or type(erreur).__name__
            return self._refuser(echange, f"Refusé : {raison}.")
        texte_plan = decrire_plan(plan.actions)
        if not plan.remplacer:
            texte_plan = "Ensuite : " + texte_plan[:1].lower() + texte_plan[1:]
        self._repondre(echange, texte_plan, plan_pour_chat(plan_id, plan.actions))
        self._memoriser(texte, texte_plan)
        return ResultatConsigne("plan", texte_plan, plan_id, plan.actions)

    async def _regarder(self, texte: str, actions: list[Action]) -> ResultatConsigne:
        assert self.vision is not None
        resumes = []
        for action in actions:
            camera = str(action.get("camera", "gauche"))
            try:
                if action["type"] == "decrire":
                    resultat = await self.vision.decrire(camera, action.get("question"))
                    resumes.append(str(resultat.get("description", "")))
                else:
                    await self.vision.chercher(str(action["objet"]), camera)
                    resumes.append(str(self.vision.dernier.get("description", "")
                                       if self.vision.dernier else ""))
            except ErreurIA as erreur:  # la vision a déjà prévenu l'opérateur
                resumes.append(str(erreur))
        resume = " ".join(r for r in resumes if r)[:400]
        self._memoriser(texte, resume or "J'ai regardé.")
        return ResultatConsigne("vision", resume, None, actions)

    # ------------------------------------------------------------------ outils internes

    def _messages(self, texte: str, origine: str, etat: dict[str, Any]) -> list[dict[str, Any]]:
        systeme = SYSTEME
        if origine == "voix":
            systeme += "\n\n" + VOIX
        etat_json = json.dumps(etat, ensure_ascii=False, default=str, separators=(",", ":"))
        systeme += f"\n\nÉtat du rover (repère odom, mètres, degrés) : {etat_json}"
        return [{"role": "system", "content": systeme}, *self._historique,
                {"role": "user", "content": texte}]

    def _memoriser(self, texte: str, reponse: str) -> None:
        if self._historique.maxlen:
            self._historique.append({"role": "user", "content": texte})
            self._historique.append({"role": "assistant", "content": reponse})

    def _repondre(self, echange: _Echange, texte: str, plan: dict[str, Any] | None = None
                  ) -> None:
        echange.publier(self.bus)
        publier_chat(self.bus, "rover", texte, echange.origine, plan)

    def _systeme(self, echange: _Echange, texte: str, statut: str) -> ResultatConsigne:
        echange.publier(self.bus)
        publier_chat(self.bus, "systeme", texte, echange.origine)
        return ResultatConsigne(statut, texte)

    def _refuser(self, echange: _Echange, texte: str, statut: str = "refus"
                 ) -> ResultatConsigne:
        self._repondre(echange, texte)
        self._memoriser(echange.texte, texte)
        return ResultatConsigne(statut, texte)


def _sans_mouvement(action: Action) -> bool:
    return action["type"] == "decrire" or (action["type"] == "chercher"
                                          and action.get("aller") is False)
