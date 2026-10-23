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
from fieldmind.ia.fournisseurs import AppelFonction, ErreurIA, IAIndisponible, ReponseModele
from fieldmind.ia.messages import plan_pour_chat, publier_chat, publier_voix
from fieldmind.ia.outils import (
    FONCTIONS_CONTROLE,
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

APPELS_SUCCESSIFS_MAX = 5
"""Relances au plus pour un modèle qui appelle une fonction à la fois."""

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
- Pour « explore la pièce », « fais le tour », « cartographie les environs », appelle explorer. \
Ne réponds pas seulement par une promesse et n'invente pas de points de passage : l'exécuteur \
les choisit dans la carte réelle, observe chaque étape et poursuit jusqu'aux limites demandées. \
Pour explorer afin de trouver quelque chose, renseigne l'objet d'explorer.
- L'état `monde` contient les objets et murs déjà observés, leurs coordonnées odom, leur source \
et leur âge. Utilise cette mémoire pour comprendre la scène et les missions. Une projection \
au sol est une estimation, pas une mesure 3D complète. Pour rejoindre un objet nommé, appelle \
chercher ; n'invente jamais sa position ni une pièce ou un passage qui n'a pas été observé.
- Une pièce (cuisine, salon, chambre...) est une zone, pas un objet : appelle rejoindre_piece \
avec son nom. L'exécuteur la reconnaît grâce aux objets réellement observés (par exemple four \
et micro-ondes pour la cuisine), vérifie les preuves et les passages. Ne remplace jamais cet \
outil par aller_a, chercher("cuisine") ou memoriser_lieu. Si l'opérateur a explicitement \
mémorisé ce nom dans `lieux`, aller_lieu reste possible. Une question « où est la cuisine ? » \
ne donne pas l'autorisation d'y aller. Une négation n'est pas un ordre de déplacement.
- Une consigne composée (« avance de 5 m puis tourne à gauche et cherche la chaise ») donne \
plusieurs appels, dans l'ordre.
- explorer, chercher et rejoindre_piece sont des missions autonomes complètes. N'ajoute \
aucune avance, rotation, coordonnée ou autre étape pour les réaliser : l'exécuteur s'en charge. \
Une étape supplémentaire n'est autorisée que si l'opérateur l'a demandée explicitement. \
Ne mémorise un lieu que sur demande explicite de mémorisation.
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


# L'ancrage est une politique de permissions sur la consigne originale. Les sorties des
# modèles (y compris les relances « action suivante ») ne sont jamais une nouvelle consigne.
_PIECES = {
    "cuisine": "cuisine", "salon": "salon", "sejour": "salon", "chambre": "chambre",
    "salle de bain": "salle de bain", "salle de bains": "salle de bain",
    "salle a manger": "salle à manger", "entree": "entrée", "bureau": "bureau",
    "garage": "garage", "toilettes": "toilettes", "wc": "toilettes",
    "couloir": "couloir", "buanderie": "buanderie",
}
_NOM_PIECE = "(?:" + "|".join(sorted(_PIECES, key=len, reverse=True)) + ")"
_AUTONOMES = frozenset({"explorer", "chercher", "rejoindre_piece"})
_VERBES_ETAPE = (r"va\b|vas\b|aller\b|rejoins\b|rejoindre\b|retourne\b|rends\b|"
                 r"avance\b|recule\b|tourne\b|cherche\b|trouve\b|explore\b|"
                 r"cartographie\b|prends\b|photographie\b|fais\b|reviens\b|"
                 r"retourne\b|decris\b|attends\b|memorise\b")


def _clauses_consigne(texte: str) -> list[str]:
    normalise = _normaliser(texte).replace("’", "'").replace("-", " ")
    return [c.strip(" ,;.!?") for c in re.split(
        rf"\bpuis\b|\bensuite\b|[;]|,(?=\s+(?:{_VERBES_ETAPE}))|"
        rf"\bet\b(?=\s+(?:{_VERBES_ETAPE}))", normalise)
        if c.strip(" ,;.!?")]


def _piece_demandee(clause: str) -> str | None:
    cible = re.search(
        rf"\b(?:va|vas|aller|rejoins|rejoindre|retourne|rends toi|cherche|trouve)\s+"
        rf"(?:(?:a|au|aux|dans|vers|jusqu|la|le|les|l|une|un)\b[' ]*)*"
        rf"(?P<piece>{_NOM_PIECE})\b\s*(?:s il te plait|s'il te plait|stp|maintenant)?$",
        clause)
    return _PIECES[cible["piece"]] if cible else None


def _cle_destination(nom: str) -> str:
    nom = " ".join(_normaliser(nom).replace("’", " ").replace("'", " ").split())
    return re.sub(r"^(?:le|la|les|l|du|au|aux)\s+", "", nom)


def _lieu_demande(clause: str, lieux: tuple[str, ...]) -> str | None:
    for lieu in lieux:
        cible = re.escape(_cle_destination(lieu))
        if re.search(
                rf"\b(?:va|vas|aller|rejoins|rejoindre|retourne|reviens|rends toi)\s+"
                rf"(?:(?:a|au|aux|dans|vers|jusqu|la|le|les|l)\b[' ]*)*{cible}\b\s*"
                rf"(?:s il te plait|s'il te plait|stp|maintenant)?$", clause):
            return lieu
    return None


def _piece_sans_ordre(texte: str, lieux: tuple[str, ...] = ()) -> bool:
    """Une question ou négation de destination ne donne pas une permission de mouvement."""
    for clause in _clauses_consigne(texte):
        if not re.search(rf"\b{_NOM_PIECE}\b", clause) and not any(
                re.search(rf"\b{re.escape(_cle_destination(n))}\b", clause) for n in lieux):
            continue
        if re.search(r"\b(?:pas|jamais|ni)\b", clause):
            return True
        if re.search(r"\b(?:ou est|ou se trouve|ou sont|sais tu|tu sais|savez vous|dis moi ou|"
                     r"indique moi ou|je me demande|comment savoir|pourquoi|si|quand|lorsque)\b",
                     clause):
            return True
    return False


def _type_etape(clause: str) -> str | None:
    # L'ordre compte : « explore pour trouver » désigne une seule mission explorer.
    motifs = (
        ("memoriser_lieu",
         r"\b(?:memorise|memoriser|enregistre cet endroit|retiens cet endroit)\b"),
        ("explorer", r"\b(?:explore|explorer|cartographie|fais le tour)\b"),
        ("retour_depart", r"\b(?:reviens|retourne|retourner)\b.*\bdepart\b"),
        ("chercher", r"\b(?:cherche|chercher|trouve|trouver)\b"),
        ("photo", r"\b(?:photo|photographie)\b"),
        ("decrire", r"\b(?:decris|decrire|regarde|regarder)\b"),
        ("avancer", r"\b(?:avance|avancer|recule|reculer)\b"),
        ("tourner", r"\b(?:tourne|tourner|demi tour)\b"),
        ("attendre", r"\b(?:attends|attendre)\b"),
        ("chercher", r"\b(?:va|vas|aller|rejoins|rejoindre)\b"),
    )
    return next((nom for nom, motif in motifs if re.search(motif, clause)), None)


def _verifier_mesure(clause: str, appel: AppelFonction) -> None:
    """Une mission composée n'autorise une manœuvre que dans la quantité demandée."""
    nombres = {"un": 1, "une": 1, "deux": 2, "trois": 3, "quatre": 4, "cinq": 5,
               "six": 6, "sept": 7, "huit": 8, "neuf": 9, "dix": 10}
    motif = r"(?P<n>\d+(?:[.,]\d+)?|" + "|".join(nombres) + ")"
    args = appel.arguments or {}
    attendu = None
    if appel.nom == "avancer":
        mesure = re.search(rf"\b{motif}\s*(?P<u>m|metres?|cm|centimetres?)\b", clause)
        if mesure:
            n = mesure["n"]
            attendu = float(nombres[n]) if n in nombres else float(n.replace(",", "."))
            if mesure["u"] in {"cm", "centimetre", "centimetres"}:
                attendu /= 100
            if re.search(r"\brecule", clause):
                attendu *= -1
        valeur = args.get("distance_m")
    else:
        mesure = re.search(rf"\b{motif}\s*(?:degres?|°)", clause)
        if mesure:
            n = mesure["n"]
            attendu = float(nombres[n]) if n in nombres else float(n.replace(",", "."))
            if "droite" in clause:
                attendu *= -1
        elif "demi tour" in clause:
            attendu = 180
        elif "gauche" in clause:
            attendu = 90
        elif "droite" in clause:
            attendu = -90
        valeur = args.get("angle_deg")
    if attendu is None or isinstance(valeur, bool) or not isinstance(valeur, int | float) or (
            abs(valeur - attendu) > 0.001):
        raise ActionInvalide("manœuvre supplémentaire sans mesure conforme à la consigne")


def _ancrer_appels(texte: str, appels: list[AppelFonction], lieux: tuple[str, ...]
                  ) -> list[AppelFonction]:
    """Permet les étapes réellement demandées ; les pièces sont résolues par la mission.

    Les ordres de pièce simples sont ancrés même si le modèle renvoie l'ancienne chaîne
    chercher → aller_a → avancer. Pour un composé, chaque autre étape doit avoir son appel
    explicite. Une consigne ambiguë est refusée en entier, sans exécution partielle.
    """
    def lecture(appel: AppelFonction) -> bool:
        return appel.nom == "decrire" or (
            appel.nom == "chercher" and (appel.arguments or {}).get("aller") is False)
    if _piece_sans_ordre(texte, lieux):
        if any(not lecture(a) and a.nom != "ignorer" for a in appels):
            raise ActionInvalide("une question ou une négation de pièce n'autorise pas à bouger")
        return appels
    if not appels:
        return appels  # une réponse/refus du modèle reste une réponse sans mouvement
    clauses = _clauses_consigne(texte)
    destinations = [_piece_demandee(c) for c in clauses]
    lieux_cibles = [_lieu_demande(c, lieux) for c in clauses]
    if not any(destinations) and all(lecture(a) for a in appels):
        return appels
    if any(a.nom == "memoriser_lieu" for a in appels) and not any(
            _type_etape(c) == "memoriser_lieu" for c in clauses):
        raise ActionInvalide("mémoriser un lieu exige une demande explicite")
    if not any(destinations) and not any(lieux_cibles) and not any(
            a.nom in _AUTONOMES for a in appels) and not re.search(
                r"\b(?:explore|explorer|cartographie|cherche|chercher|trouve|trouver)\b",
                _normaliser(texte)):
        return appels
    resultat = []
    if re.search(r"\b(?:ensuite|apres ca|ajoute)\b", _normaliser(texte)):
        resultat.append(AppelFonction("mettre_a_la_suite", {}))
    restants = [a for a in appels if a.nom not in FONCTIONS_CONTROLE and a.nom != "stop"]
    for clause, piece, lieu in zip(clauses, destinations, lieux_cibles, strict=True):
        if piece or lieu:
            memorise = lieu or next((n for n in lieux if _cle_destination(n) ==
                                    _cle_destination(piece or "")), None)
            resultat.append(AppelFonction("aller_lieu" if memorise else "rejoindre_piece",
                                          {"nom": memorise or piece}))
            # Une proposition chercher/aller_lieu pour la même pièce est remplacée, jamais
            # interprétée comme une demande supplémentaire de l'opérateur.
            restants = [a for a in restants if not (
                a.nom in {"chercher", "aller_lieu", "rejoindre_piece"} and
                _normaliser(str((a.arguments or {}).get("nom", (a.arguments or {}).get(
                    "objet", "")))).removeprefix("la ").removeprefix("le ") ==
                _normaliser(piece or lieu or ""))]
            continue
        attendu = _type_etape(clause)
        indice = next((i for i, a in enumerate(restants) if a.nom == attendu), None)
        if attendu is None or indice is None:
            raise ActionInvalide("étape de la consigne non comprise ; aucun déplacement lancé")
        appel = restants.pop(indice)
        if attendu in {"avancer", "tourner"}:
            _verifier_mesure(clause, appel)
        resultat.append(appel)
    if not any(destinations) and not any(lieux_cibles) and restants:
        raise ActionInvalide("le modèle a ajouté des étapes absentes de la consigne")
    return resultat


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
        identite_mission = (etat.get("mission") or {}).get("generation")
        profil_actif = self.config_ia.actif
        lieux = lieux_depuis_etat(etat)
        vitesse_max = _valeur_profil(self.profil, "vitesses.max_auto_m_s", 0.5)
        try:
            messages = self._messages(texte, origine, etat)
            outils = schemas_fonctions(vitesse_max, lieux)
            reponse = await self.config_ia.passerelle().chat("agent", messages, outils=outils)
            if self.config_ia.actif.appels_successifs and reponse.appels:
                reponse = await self._appels_suivants(messages, outils, reponse, texte)
        except ErreurIA as erreur:
            return self._systeme(echange, f"Consigne non exécutée : {erreur}.",
                                 "indisponible" if isinstance(erreur, IAIndisponible)
                                 else "erreur")
        if generation != self._generation:
            return ResultatConsigne("abandonne", "arrêt demandé pendant l'analyse")
        actuel = self.outils.etat_resume()
        if (identite_mission != (actuel.get("mission") or {}).get("generation")
                or profil_actif is not self.config_ia.actif
                or actuel.get("arret_urgence")
                or actuel.get("arme") != etat.get("arme")
                or actuel.get("mode") != etat.get("mode")):
            return self._systeme(echange, "Consigne abandonnée : la mission ou le mode a changé "
                                 "pendant l'analyse.", "abandonne")
        try:
            pose = self.outils.pose_odom()
        except Exception:
            pose = Pose2D()
        contexte = ContexteValidation(
            vitesse_max_m_s=vitesse_max, pose=pose, lieux=lieux, gps_fix=gps_depuis_etat(etat),
            geofence_m=_valeur_profil(self.profil, "securite.geofence_m", 60.0),
            actions_max=self.config_ia.agent.actions_max)
        try:
            appels_ancres = _ancrer_appels(texte, sans_stop_final(reponse.appels), lieux)
            plan = valider_plan(appels_ancres, contexte)
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

    async def _appels_suivants(self, messages: list[dict[str, Any]], outils: list[Any],
                               reponse: ReponseModele, texte: str) -> ReponseModele:
        """Modèles qui appellent une fonction à la fois (Qwen) : boucle agentique habituelle,
        on rend « noté » et le modèle donne la suivante, jusqu'à ce qu'il réponde sans appel.
        « avance puis tourne » donne ainsi ses deux actions."""
        tous = list(reponse.appels)
        dernier = reponse
        messages = list(messages)
        # Une mission autonome réalise elle-même ses déplacements et ses observations.
        # « Noté » n'est pas un résultat d'exécution autorisant le modèle à inventer la suite.
        # Les relances ne servent qu'aux étapes supplémentaires écrites par l'opérateur.
        etapes_max = max(1, len(_clauses_consigne(texte)))
        for tour in range(APPELS_SUCCESSIFS_MAX):
            if not dernier.appels or any(a.nom in FONCTIONS_CONTROLE for a in dernier.appels):
                break
            if len([a for a in tous if a.nom not in FONCTIONS_CONTROLE]) >= etapes_max:
                break
            ids = [a.id or f"appel{tour}_{k}" for k, a in enumerate(dernier.appels)]
            messages.append({"role": "assistant", "content": dernier.texte or None, "tool_calls": [
                {"id": i, "type": "function", "function": {
                    "name": a.nom, "arguments": a.brut or json.dumps(a.arguments or {})}}
                for i, a in zip(ids, dernier.appels, strict=True)]})
            messages.extend({"role": "tool", "tool_call_id": i,
                             "content": "Proposition enregistrée, pas encore exécutée. Donne "
                                        "uniquement l'étape suivante explicitement demandée "
                                        "par l'opérateur. Une mission autonome est une étape "
                                        "complète : n'invente aucun mouvement pour la réaliser. "
                                        "Réponds sans appel si la consigne est complète."}
                            for i in ids)
            suivante = await self.config_ia.passerelle().chat("agent", messages, outils=outils)
            deja = {(x.nom, json.dumps(x.arguments, sort_keys=True)) for x in tous[-2:]}
            nouveaux = [x for x in suivante.appels
                        if (x.nom, json.dumps(x.arguments, sort_keys=True)) not in deja]
            if not nouveaux:
                return ReponseModele(texte=suivante.texte or reponse.texte, appels=tous,
                                     modele=reponse.modele, fin=suivante.fin)
            tous.extend(nouveaux)
            dernier = ReponseModele(texte=suivante.texte, appels=nouveaux, modele=suivante.modele)
        return ReponseModele(texte=dernier.texte or reponse.texte, appels=tous,
                             modele=reponse.modele, fin=dernier.fin)

    def _messages(self, texte: str, origine: str, etat: dict[str, Any]) -> list[dict[str, Any]]:
        systeme = SYSTEME
        if origine == "voix":
            systeme += "\n\n" + VOIX
        etat_json = json.dumps(etat, ensure_ascii=False, default=str, separators=(",", ":"))
        systeme += f"\n\nÉtat du rover (repère odom, mètres, degrés) : {etat_json}"
        if etat.get("situation"):
            systeme += ("\n\n« situation » est ce que l'agent de veille a vu il y a « age_s » "
                        "secondes (« vus » : détecteur continu). Pour une question sur ce qui se "
                        "passe, réponds d'abord avec elle ; appelle « decrire » seulement si "
                        "elle est vieille ou si on te demande de regarder. Un ordre (suivre, "
                        "chercher, aller…) s'exécute même si la situation ne cite pas ce qui est "
                        "demandé : la vérification se fait sur l'image du moment.")
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


def sans_stop_final(appels: list[Any]) -> list[Any]:
    """« avance puis tourne, puis arrête-toi » : le modèle ajoute parfois `stop` à la fin d'un
    plan. Le rover s'arrête de lui-même au bout du plan, et un `stop` dans la file l'annulerait
    tout entier : il n'est gardé que seul."""
    if len(appels) > 1 and any(a.nom != "stop" for a in appels):
        return [a for a in appels if a.nom != "stop"]
    return appels


def _sans_mouvement(action: Action) -> bool:
    return action["type"] == "decrire" or (action["type"] == "chercher"
                                          and action.get("aller") is False)
