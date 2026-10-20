"""Ce que l'IA peut demander au rover : protocole `OutilsRover`, fonctions proposées au modèle et
validation stricte de ses propositions.

Règle : le modèle propose, le code décide. Chaque appel de fonction du modèle est relu ici
(nom connu, paramètres connus et typés, bornes, lieu mémorisé, portée, position GPS disponible)
et traduit en `Action`
(types.py). Un seul appel invalide fait refuser tout le plan : rien n'est exécuté à moitié.
La file de mission (`OutilsRover.ajouter_actions`) et le superviseur de sécurité vérifient encore
derrière (rover armé, obstacles, geofence).
"""

from __future__ import annotations

import math
import unicodedata
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from fieldmind.ia.fournisseurs import AppelFonction
from fieldmind.types import Action, ImageCamera, Pose2D

# ------------------------------------------------------------------ bornes

DISTANCE_MAX_M = 20.0
"""avancer : |distance| au plus (reculer = distance négative)."""
DISTANCE_MIN_M = 0.05
ANGLE_MAX_DEG = 360.0
ANGLE_MIN_DEG = 1.0
PORTEE_ALLER_A_M = 30.0
"""aller_a : point à moins de cette distance du rover."""
ATTENTE_MAX_S = 120.0
NOM_LIEU_LONGUEUR_MAX = 40
TOLERANCE_GPS_M = (0.5, 20.0)
"""aller_gps : précision demandée à l'arrivée (m), bornes de la mission."""
VITESSE_MIN_M_S = 0.05
OBJET_LONGUEUR_MAX = 80
QUESTION_LONGUEUR_MAX = 200
CAMERAS = ("gauche", "droite")
ACTIONS_MAX = 12

TYPES_ACTIONS = ("avancer", "tourner", "aller_a", "cap", "attendre", "retour_depart",
                 "aller_gps", "memoriser_lieu", "aller_lieu", "chercher", "decrire", "photo",
                 "stop")
"""Actions de mission (types.Action) que le modèle peut proposer."""
FONCTIONS_CONTROLE = ("mettre_a_la_suite", "annuler", "ignorer")
"""Fonctions qui ne sont pas des actions : elles règlent le traitement de la consigne."""


class ActionRefusee(Exception):
    """Levée par `OutilsRover.ajouter_actions` : action invalide, rover non armé, file pleine…"""

    def __init__(self, raison: str) -> None:
        super().__init__(raison)
        self.raison = raison


class ActionInvalide(ValueError):
    """Proposition du modèle rejetée par la validation (message lisible)."""


@runtime_checkable
class OutilsRover(Protocol):
    """Interface que la mission (l'intégrateur) fournit à l'agent et à la vision.

    Toutes les méthodes sont appelées depuis la boucle asyncio et doivent rendre la main vite.
    """

    def etat_resume(self) -> dict[str, Any]:
        """État court pour le modèle, par exemple :
        {"pose": {"x": 1.2, "y": -0.3, "cap_deg": 12}, "vitesse_m_s": 0.0, "mode": "auto",
         "arme": True, "securite": ["stereo_absente"], "mission": {"etat": "route",
         "action": {...}, "file": [...]}, "capteurs": {"lidar": True, "gauche": True,
         "droite": True, "stereo": True, "gnss": True}, "lieux": [{"nom": "garage",
         "gps": True}], "gps": {"aligne": True, "fix": "autonome"}}
        (lieux : ceux qu'`aller_lieu` peut rejoindre ; une liste de noms est aussi acceptée)."""
        ...

    def ajouter_actions(self, actions: list[Action], remplacer: bool) -> str:
        """Ajoute les actions à la file (après l'avoir vidée si `remplacer`) et passe en mode
        auto ; renvoie l'identifiant du plan. Lève ActionRefusee(raison) si une action est
        invalide ou si le rover n'est pas armé."""
        ...

    def annuler(self) -> None:
        """Vide la file et arrête l'action en cours (message `annuler`)."""
        ...

    def arret_immediat(self, raison: str) -> None:
        """Arrêt tout de suite : file vidée, consigne nulle (sans verrou d'arrêt d'urgence)."""
        ...

    def derniere_image(self, camera: str) -> ImageCamera | None:
        """Dernière image de la caméra (« gauche », « droite »), None si absente ou périmée."""
        ...

    def pixel_vers_sol(self, camera: str, u: float, v: float) -> tuple[float, float] | None:
        """Point du sol visé par le pixel (u, v normalisés 0..1) dans base_link : profondeur
        stéréo si elle existe, sinon sol plan. None si le pixel est au-dessus de l'horizon."""
        ...

    def pose_odom(self) -> Pose2D:
        """Pose courante dans le repère odom."""
        ...


# ------------------------------------------------------------------ fonctions pour le modèle


def _fonction(nom: str, description: str, proprietes: dict[str, Any] | None = None,
              requis: Iterable[str] = ()) -> dict[str, Any]:
    fonction: dict[str, Any] = {"name": nom, "description": description}
    if proprietes:  # Gemini refuse un objet sans propriétés : pas de `parameters` du tout
        fonction["parameters"] = {"type": "object", "properties": proprietes,
                                  "required": list(requis)}
    return {"type": "function", "function": fonction}


def schemas_fonctions(vitesse_max_m_s: float = 0.5,
                      lieux: Iterable[str] = ()) -> list[dict[str, Any]]:
    """Fonctions (format `tools` OpenAI) : une par type d'action, plus les fonctions de contrôle.
    `lieux` : noms des lieux mémorisés ; aller_lieu n'est proposée que s'il y en a, avec leurs
    noms en énumération."""
    lieux = sorted(set(lieux))
    camera = {"type": "string", "enum": list(CAMERAS),
              "description": "caméra avant gauche ou droite (gauche par défaut)"}
    return [
        _fonction("avancer", "Avancer en ligne droite d'une distance mesurée (cap tenu). "
                  "Distance négative pour reculer.",
                  {"distance_m": {"type": "number", "minimum": -DISTANCE_MAX_M,
                                  "maximum": DISTANCE_MAX_M,
                                  "description": "mètres, négatif = reculer"},
                   "vitesse_m_s": {"type": "number", "minimum": VITESSE_MIN_M_S,
                                   "maximum": round(vitesse_max_m_s, 2),
                                   "description": "facultatif, seulement si l'opérateur la "
                                   "précise"}},
                  ["distance_m"]),
        _fonction("tourner", "Tourner sur un arc (direction de voiture : le rover ne pivote pas "
                  "sur place) d'un angle relatif.",
                  {"angle_deg": {"type": "number", "minimum": -ANGLE_MAX_DEG,
                                 "maximum": ANGLE_MAX_DEG,
                                 "description": "degrés, positif à gauche, négatif à droite "
                                 "(« à gauche » = 90, « à droite » = -90, demi-tour = 180)"}},
                  ["angle_deg"]),
        _fonction("aller_a", "Aller à un point en évitant les obstacles (planificateur) : point "
                  "relatif au rover (repere robot : x devant, y à gauche) ou point de la carte "
                  "(repere odom).",
                  {"x": {"type": "number", "description": "mètres"},
                   "y": {"type": "number", "description": "mètres, positif à gauche"},
                   "repere": {"type": "string", "enum": ["robot", "odom"]}},
                  ["x", "y"]),
        _fonction("cap", "S'orienter vers un cap absolu de la carte (0 = direction du rover au "
                  "départ, positif vers la gauche).",
                  {"cap_deg": {"type": "number", "minimum": -180, "maximum": 360}},
                  ["cap_deg"]),
        _fonction("attendre", "Attendre sans bouger.",
                  {"duree_s": {"type": "number", "minimum": 0.1, "maximum": ATTENTE_MAX_S}},
                  ["duree_s"]),
        _fonction("retour_depart", "Revenir au point de départ de la carte."),
        _fonction("aller_gps", "Aller à des coordonnées GPS données par l'opérateur (degrés "
                  "décimaux WGS 84).",
                  {"lat": {"type": "number", "minimum": -90, "maximum": 90,
                           "description": "latitude, degrés décimaux"},
                   "lon": {"type": "number", "minimum": -180, "maximum": 180,
                           "description": "longitude, degrés décimaux (négative à l'ouest)"},
                   "tolerance_m": {"type": "number", "minimum": TOLERANCE_GPS_M[0],
                                   "maximum": TOLERANCE_GPS_M[1],
                                   "description": "facultatif : précision voulue à l'arrivée"}},
                  ["lat", "lon"]),
        _fonction("memoriser_lieu", "Mémoriser la position actuelle du rover sous un nom (« "
                  "mémorise cet endroit comme le garage »).",
                  {"nom": {"type": "string", "description": "nom court du lieu, sans article "
                           "(« garage »)"}},
                  ["nom"]),
        *([_fonction("aller_lieu", "Aller à un lieu mémorisé.",
                     {"nom": {"type": "string", "enum": lieux}}, ["nom"])] if lieux else []),
        _fonction("chercher", "Chercher un objet avec la caméra (vision), puis aller jusqu'à lui "
                  "sauf si aller vaut false (simple question « où est… »).",
                  {"objet": {"type": "string", "description": "objet décrit en français, "
                             "court (« chaise rouge »)"},
                   "aller": {"type": "boolean", "description": "aller jusqu'à l'objet (vrai par "
                             "défaut)"},
                   "camera": camera},
                  ["objet"]),
        _fonction("decrire", "Regarder avec la caméra et décrire la scène à l'opérateur ; "
                  "répondre à une question sur ce qui est visible.",
                  {"camera": camera,
                   "question": {"type": "string", "description": "facultatif : question de "
                                "l'opérateur sur l'image"}}),
        _fonction("photo", "Prendre une photo et l'enregistrer sur le rover.", {"camera": camera}),
        _fonction("stop", "S'arrêter (à la fin d'une séquence, ou seul pour arrêter tout)."),
        _fonction("mettre_a_la_suite", "À appeler en premier quand l'opérateur veut ajouter ces "
                  "actions après la mission en cours (« ensuite », « après ça », « ajoute ») au "
                  "lieu de la remplacer."),
        _fonction("annuler", "Annuler la mission en cours et vider la file, sans autre action."),
        _fonction("ignorer", "La phrase n'est pas une consigne adressée au rover (bruit, "
                  "conversation à côté, phrase coupée ou sans rapport) : ne rien faire.",
                  {"raison": {"type": "string"}}),
    ]


# ------------------------------------------------------------------ validation


@dataclass(slots=True)
class ContexteValidation:
    vitesse_max_m_s: float = 0.5
    pose: Pose2D = field(default_factory=Pose2D)
    lieux: tuple[str, ...] = ()
    """Noms des lieux mémorisés (aller_lieu)."""
    gps_fix: str | None = None
    """Fix GNSS d'`etat_resume()["gps"]` (« aucun » : aller_gps refusé) ; None si inconnu."""
    geofence_m: float | None = None
    actions_max: int = ACTIONS_MAX


@dataclass(slots=True)
class PlanPropose:
    """Lecture validée de la réponse du modèle."""

    actions: list[Action] = field(default_factory=list)
    remplacer: bool = True
    annuler: bool = False
    ignorer: bool = False
    raison_ignorer: str = ""


def lieux_depuis_etat(etat: Mapping[str, Any]) -> tuple[str, ...]:
    """Noms des lieux mémorisés d'`etat_resume()` : liste de {"nom": ..., "gps": ...} ou de
    noms, ou dict {nom: ...}."""
    brut = etat.get("lieux") or ()
    if isinstance(brut, Mapping):
        noms: Iterable[Any] = brut.keys()
    elif isinstance(brut, list | tuple):
        noms = (e.get("nom") if isinstance(e, Mapping) else e for e in brut)
    else:
        return ()
    propres = (" ".join(n.split()) for n in noms if isinstance(n, str))
    return tuple(dict.fromkeys(n for n in propres if n))


def gps_depuis_etat(etat: Mapping[str, Any]) -> str | None:
    """Fix GNSS de `etat_resume()["gps"]` (« aucun », « autonome »…), None si absent."""
    gps = etat.get("gps")
    fix = gps.get("fix") if isinstance(gps, Mapping) else None
    return fix if isinstance(fix, str) else None


def _cle_lieu(nom: str) -> str:
    """Nom comparable : minuscules, sans accents ni article (« Le Garage » → « garage »)."""
    decompose = unicodedata.normalize("NFD", nom.lower())
    texte = "".join(c for c in decompose if not unicodedata.combining(c))
    texte = " ".join(texte.replace("'", " ").replace("’", " ").split())
    for article in ("le ", "la ", "les ", "l ", "du ", "au ", "aux "):
        if texte.startswith(article):
            return texte[len(article):]
    return texte


def _nombre(args: Mapping[str, Any], cle: str, bas: float, haut: float, *,
            requis: bool = True) -> float | None:
    if cle not in args or args[cle] is None:
        if requis:
            raise ActionInvalide(f"paramètre {cle} manquant")
        return None
    valeur = args[cle]
    if isinstance(valeur, bool) or not isinstance(valeur, int | float):
        raise ActionInvalide(f"{cle} doit être un nombre")
    valeur = float(valeur)
    if not math.isfinite(valeur):
        raise ActionInvalide(f"{cle} invalide")
    if not bas <= valeur <= haut:
        raise ActionInvalide(f"{cle} = {_fr(valeur)} hors limites ({_fr(bas)} à {_fr(haut)})")
    return valeur


def _texte(args: Mapping[str, Any], cle: str, longueur_max: int, *, requis: bool = True
           ) -> str | None:
    valeur = args.get(cle)
    if valeur is None:
        if requis:
            raise ActionInvalide(f"paramètre {cle} manquant")
        return None
    if not isinstance(valeur, str) or not valeur.strip():
        raise ActionInvalide(f"{cle} doit être un texte")
    valeur = " ".join(valeur.split())
    if len(valeur) > longueur_max:
        raise ActionInvalide(f"{cle} trop long ({longueur_max} caractères au plus)")
    return valeur


def _camera(args: Mapping[str, Any]) -> str | None:
    camera = args.get("camera")
    if camera is None:
        return None
    if camera not in CAMERAS:
        raise ActionInvalide(f"caméra {camera!r} inconnue")
    return str(camera)


PARAMETRES: dict[str, frozenset[str]] = {
    "avancer": frozenset({"distance_m", "vitesse_m_s"}),
    "tourner": frozenset({"angle_deg"}),
    "aller_a": frozenset({"x", "y", "repere"}),
    "cap": frozenset({"cap_deg"}),
    "attendre": frozenset({"duree_s"}),
    "retour_depart": frozenset(),
    "aller_gps": frozenset({"lat", "lon", "tolerance_m"}),
    "memoriser_lieu": frozenset({"nom"}),
    "aller_lieu": frozenset({"nom"}),
    "chercher": frozenset({"objet", "aller", "camera"}),
    "decrire": frozenset({"camera", "question"}),
    "photo": frozenset({"camera"}),
    "stop": frozenset(),
    "mettre_a_la_suite": frozenset(),
    "annuler": frozenset(),
    "ignorer": frozenset({"raison"}),
}


def valider_action(nom: str, arguments: Mapping[str, Any] | None,
                   contexte: ContexteValidation) -> Action:
    """Traduit un appel de fonction d'action en `Action` ; lève ActionInvalide."""
    if nom not in TYPES_ACTIONS:
        raise ActionInvalide(f"action inconnue : {nom!r}")
    if arguments is None:
        raise ActionInvalide(f"{nom} : paramètres illisibles")
    args = {k: v for k, v in arguments.items() if v is not None}
    inconnus = set(args) - PARAMETRES[nom]
    if inconnus:
        raise ActionInvalide(f"{nom} : paramètre inconnu {', '.join(sorted(inconnus))}")
    match nom:
        case "avancer":
            distance = _nombre(args, "distance_m", -DISTANCE_MAX_M, DISTANCE_MAX_M)
            assert distance is not None
            if abs(distance) < DISTANCE_MIN_M:
                raise ActionInvalide("distance trop petite")
            action: Action = {"type": "avancer", "distance_m": round(distance, 3)}
            vitesse = _nombre(args, "vitesse_m_s", VITESSE_MIN_M_S, contexte.vitesse_max_m_s,
                              requis=False)
            if vitesse is not None:
                action["vitesse_m_s"] = round(vitesse, 3)
            return action
        case "tourner":
            angle = _nombre(args, "angle_deg", -ANGLE_MAX_DEG, ANGLE_MAX_DEG)
            assert angle is not None
            if abs(angle) < ANGLE_MIN_DEG:
                raise ActionInvalide("angle trop petit")
            return {"type": "tourner", "angle_deg": round(angle, 1)}
        case "aller_a":
            return _valider_aller_a(args, contexte)
        case "cap":
            cap = _nombre(args, "cap_deg", -180.0, 360.0)
            return {"type": "cap", "cap_deg": round(float(cap or 0.0), 1)}
        case "attendre":
            duree = _nombre(args, "duree_s", 0.1, ATTENTE_MAX_S)
            return {"type": "attendre", "duree_s": round(float(duree or 0.0), 2)}
        case "aller_gps":
            if contexte.gps_fix == "aucun":
                raise ActionInvalide("pas de position GPS (aucun fix)")
            lat = _nombre(args, "lat", -90.0, 90.0)
            lon = _nombre(args, "lon", -180.0, 180.0)
            assert lat is not None and lon is not None
            if lat == 0.0 and lon == 0.0:
                raise ActionInvalide("coordonnées GPS (0, 0) invalides")
            action = {"type": "aller_gps", "lat": round(lat, 7), "lon": round(lon, 7)}
            tolerance = _nombre(args, "tolerance_m", *TOLERANCE_GPS_M, requis=False)
            if tolerance is not None:
                action["tolerance_m"] = round(tolerance, 2)
            return action
        case "memoriser_lieu":
            nom_lieu = _texte(args, "nom", NOM_LIEU_LONGUEUR_MAX)
            return {"type": "memoriser_lieu", "nom": nom_lieu}
        case "aller_lieu":
            demande = _texte(args, "nom", NOM_LIEU_LONGUEUR_MAX)
            assert demande is not None
            connus = {_cle_lieu(n): n for n in contexte.lieux}
            if _cle_lieu(demande) not in connus:
                liste = ", ".join(f"« {n} »" for n in contexte.lieux) or "aucun"
                raise ActionInvalide(f"lieu inconnu : « {demande} » (lieux mémorisés : {liste})")
            return {"type": "aller_lieu", "nom": connus[_cle_lieu(demande)]}
        case "chercher":
            objet = _texte(args, "objet", OBJET_LONGUEUR_MAX)
            action = {"type": "chercher", "objet": objet}
            aller = args.get("aller", True)
            if not isinstance(aller, bool):
                raise ActionInvalide("aller doit valoir vrai ou faux")
            if not aller:
                action["aller"] = False
            if (camera := _camera(args)) is not None:
                action["camera"] = camera
            return action
        case "decrire":
            action = {"type": "decrire"}
            if (camera := _camera(args)) is not None:
                action["camera"] = camera
            question = _texte(args, "question", QUESTION_LONGUEUR_MAX, requis=False)
            if question:
                action["question"] = question
            return action
        case "photo":
            return {"type": "photo", "camera": _camera(args) or "gauche"}
        case _:  # retour_depart, stop
            return {"type": nom}


def _valider_aller_a(args: Mapping[str, Any], contexte: ContexteValidation) -> Action:
    borne = PORTEE_ALLER_A_M + abs(contexte.pose.x) + abs(contexte.pose.y)
    x = _nombre(args, "x", -borne, borne)
    y = _nombre(args, "y", -borne, borne)
    assert x is not None and y is not None
    repere = args.get("repere", "robot")
    if repere not in ("robot", "odom"):
        raise ActionInvalide(f"repère {repere!r} inconnu")
    if repere == "robot":
        distance = math.hypot(x, y)
        xo, yo = contexte.pose.vers_parent(x, y)
    else:
        distance = math.hypot(x - contexte.pose.x, y - contexte.pose.y)
        xo, yo = x, y
    if distance > PORTEE_ALLER_A_M:
        raise ActionInvalide(f"point à {_fr(distance, 1)} m : trop loin "
                             f"({_fr(PORTEE_ALLER_A_M)} m au plus)")
    if distance < 0.1:
        raise ActionInvalide("le rover est déjà à ce point")
    if contexte.geofence_m is not None and math.hypot(xo, yo) > contexte.geofence_m:
        raise ActionInvalide(f"point hors de la zone autorisée ({_fr(contexte.geofence_m)} m "
                             "autour du départ)")
    return {"type": "aller_a", "x": round(x, 3), "y": round(y, 3), "repere": repere}


def valider_plan(appels: Iterable[AppelFonction], contexte: ContexteValidation) -> PlanPropose:
    """Lit tous les appels du modèle ; lève ActionInvalide au premier appel invalide."""
    plan = PlanPropose()
    for appel in appels:
        if appel.nom in FONCTIONS_CONTROLE:
            arguments = appel.arguments or {}
            if set(arguments) - PARAMETRES[appel.nom]:
                raise ActionInvalide(f"{appel.nom} : paramètre inconnu")
            if appel.nom == "mettre_a_la_suite":
                plan.remplacer = False
            elif appel.nom == "annuler":
                plan.annuler = True
            else:
                plan.ignorer = True
                raison = arguments.get("raison")
                plan.raison_ignorer = raison[:200] if isinstance(raison, str) else ""
            continue
        plan.actions.append(valider_action(appel.nom, appel.arguments, contexte))
        if len(plan.actions) > contexte.actions_max:
            raise ActionInvalide(f"plan trop long ({contexte.actions_max} actions au plus)")
    return plan


# ------------------------------------------------------------------ texte des actions


def _fr(valeur: float, decimales: int = 2) -> str:
    """Nombre à la française : 0,5 ; 12 ; 2,25."""
    texte = f"{valeur:.{decimales}f}"
    if "." in texte:
        texte = texte.rstrip("0").rstrip(".")
    return (texte if texte not in ("", "-0") else "0").replace(".", ",")


def decrire_action(action: Action) -> str:
    """Phrase courte (sans majuscule) décrivant une action validée."""
    match action.get("type"):
        case "avancer":
            d = float(action["distance_m"])
            texte = f"j'avance de {_fr(d)} m" if d > 0 else f"je recule de {_fr(-d)} m"
            if "vitesse_m_s" in action:
                texte += f" à {_fr(float(action['vitesse_m_s']))} m/s"
            return texte
        case "tourner":
            a = float(action["angle_deg"])
            return f"je tourne de {_fr(abs(a), 0)}° à {'gauche' if a > 0 else 'droite'}"
        case "aller_a":
            x, y = float(action["x"]), float(action["y"])
            if action.get("repere") == "robot":
                cote = f", {_fr(abs(y), 1)} m à {'gauche' if y > 0 else 'droite'}" if abs(
                    y) >= 0.05 else ""
                sens = "devant" if x >= 0 else "derrière"
                return f"je vais au point {_fr(abs(x), 1)} m {sens}{cote}"
            return f"je vais au point ({_fr(x, 1)} ; {_fr(y, 1)})"
        case "cap":
            return f"je m'oriente au cap {_fr(float(action['cap_deg']), 0)}°"
        case "attendre":
            return f"j'attends {_fr(float(action['duree_s']), 1)} s"
        case "retour_depart":
            return "je reviens au départ"
        case "aller_gps":
            texte = f"je vais au point GPS {float(action['lat']):.5f}, {float(action['lon']):.5f}"
            if "tolerance_m" in action:
                texte += f" (à {_fr(float(action['tolerance_m']), 1)} m près)"
            return texte
        case "memoriser_lieu":
            return f"je mémorise cet endroit comme « {action['nom']} »"
        case "aller_lieu":
            return f"je vais à « {action['nom']} »"
        case "chercher":
            texte = f"je cherche « {action['objet']} »"
            return texte if action.get("aller", True) else texte + " sans y aller"
        case "decrire":
            return "je regarde et je décris ce que je vois"
        case "photo":
            return f"je prends une photo (caméra {action.get('camera', 'gauche')})"
        case "stop":
            return "je m'arrête"
    return str(action.get("type"))


def decrire_plan(actions: list[Action]) -> str:
    """« J'avance de 5 m, puis je tourne de 90° à gauche, puis je cherche « chaise ». »"""
    texte = ", puis ".join(decrire_action(action) for action in actions)
    return (texte[:1].upper() + texte[1:] + ".") if texte else ""
