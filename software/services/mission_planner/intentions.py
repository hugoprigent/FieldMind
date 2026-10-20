"""Intentions de l'opérateur : ce que le modèle de langage peut proposer, et sa validation.

Règle : l'IA propose, le code décide. Le modèle renvoie une intention JSON conforme au schéma
ci-dessous ; ce module la valide (action permise, lieu connu, point dans la zone, distance
raisonnable) et la traduit en action pour le cerveau. Le texte affiché à l'opérateur est composé
ici, jamais par le modèle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

from services.protocole.scenario import ScenarioPublic

ActionIntention = Literal[
    "aller_a",
    "parcours",
    "faire_le_tour",
    "trajet",
    "chercher",
    "deplacement_relatif",
    "s_arreter",
    "reprendre",
    "revenir_depart",
    "observer",
    "vigilance_activer",
    "vigilance_desactiver",
    "inconnu",
]
DISTANCE_RELATIVE_MAX_M = 30.0
MARGE_LIMITES_M = 0.5


class Intention(BaseModel):
    """Intention structurée. Les champs inutiles pour l'action sont à null."""

    model_config = ConfigDict(extra="forbid")

    action: ActionIntention
    lieu: str | None = Field(default=None, description="Identifiant d'un lieu nommé du scénario")
    lieux: list[str] | None = Field(
        default=None, description="Parcours : identifiants des lieux, dans l'ordre de visite"
    )
    cible: str | None = Field(
        default=None, description="Chercher : catégorie d'objet à trouver (tronc_couche, rocher…)"
    )
    nombre: int | None = Field(default=None, description="Chercher : combien en trouver (1 à 10)")
    reponse: str | None = Field(
        default=None, description="Action inconnu : courte réponse à une question ou remarque"
    )
    points: list[list[float]] | None = Field(
        default=None,
        description="Trajet : points [avant_m, gauche_m] relatifs au rover, dans l'ordre",
    )
    x: float | None = Field(default=None, description="Coordonnée est (m), si donnée explicitement")
    y: float | None = Field(
        default=None, description="Coordonnée nord (m), si donnée explicitement"
    )
    distance_m: float | None = Field(
        default=None, description="Déplacement relatif : distance (m), positive"
    )
    angle_deg: float | None = Field(
        default=None,
        description="Déplacement relatif : direction par rapport à l'avant du rover, en degrés, "
        "positive vers la gauche (0 = tout droit, 90 = à gauche, -90 = à droite, 180 = derrière)",
    )


def schema_json(scenario: ScenarioPublic) -> dict[str, Any]:
    """Schéma JSON strict de l'intention (sortie structurée du modèle)."""
    nullable_nombre = {"anyOf": [{"type": "number"}, {"type": "null"}]}
    return {
        "type": "object",
        "properties": {
            "action": {"type": "string", "enum": list(ActionIntention.__args__)},  # type: ignore[attr-defined]
            "lieu": {
                "anyOf": [
                    {"type": "string", "enum": [lieu.id for lieu in scenario.lieux]},
                    {"type": "null"},
                ]
            },
            "lieux": {
                "anyOf": [
                    {
                        "type": "array",
                        "items": {"type": "string", "enum": [lieu.id for lieu in scenario.lieux]},
                    },
                    {"type": "null"},
                ]
            },
            "cible": {"anyOf": [{"type": "string", "enum": list(CIBLES)}, {"type": "null"}]},
            "nombre": {"anyOf": [{"type": "integer"}, {"type": "null"}]},
            "reponse": {"anyOf": [{"type": "string"}, {"type": "null"}]},
            "points": {
                "anyOf": [
                    {
                        "type": "array",
                        "items": {"type": "array", "items": {"type": "number"}},
                    },
                    {"type": "null"},
                ]
            },
            "x": nullable_nombre,
            "y": nullable_nombre,
            "distance_m": nullable_nombre,
            "angle_deg": nullable_nombre,
        },
        "required": [
            "action",
            "lieu",
            "lieux",
            "cible",
            "nombre",
            "reponse",
            "points",
            "x",
            "y",
            "distance_m",
            "angle_deg",
        ],
        "additionalProperties": False,
    }


# ------------------------------------------------------------------ actions validées


@dataclass(frozen=True, slots=True)
class AllerA:
    x: float
    y: float
    libelle: str


@dataclass(frozen=True, slots=True)
class Parcours:
    """Étapes visitées dans l'ordre ; une étape impossible est sautée."""

    etapes: tuple[AllerA, ...]


@dataclass(frozen=True, slots=True)
class Chercher:
    """Exploration de la zone jusqu'à ce que la caméra voie la cible."""

    cible: str
    nom: str
    etapes: tuple[AllerA, ...]
    nombre: int = 1


@dataclass(frozen=True, slots=True)
class Repondre:
    """Pas une consigne (question, remarque) : seulement une réponse, aucune action."""


@dataclass(frozen=True, slots=True)
class Arreter:
    pass


@dataclass(frozen=True, slots=True)
class Reprendre:
    pass


@dataclass(frozen=True, slots=True)
class Observer:
    """Analyse immédiate de la caméra ; la réponse arrive avec le résultat de l'analyse."""


@dataclass(frozen=True, slots=True)
class Vigilance:
    active: bool


@dataclass(frozen=True, slots=True)
class Refus:
    raison: str


Action = AllerA | Parcours | Chercher | Repondre | Arreter | Reprendre | Observer | Vigilance
CIBLES: dict[str, str] = {
    "tronc_couche": "un tronc couché",
    "branche": "une branche au sol",
    "rocher": "un rocher",
    "trou": "un trou ou un fossé",
    "eau": "de l'eau",
    "personne": "une personne",
    "animal": "un animal",
    "vehicule": "un véhicule",
}
ETAPES_MAX = 8
POINTS_TRAJET_MAX = 40
PORTEE_TRAJET_M = 25.0


@dataclass(frozen=True, slots=True)
class PoseCourante:
    x: float
    y: float
    cap: float


def a_lieu(nom: str) -> str:
    """« à » contracté devant le nom d'un lieu : au grand rocher, à la clairière, aux arbres."""
    if nom.startswith("le "):
        return "au " + nom[3:]
    if nom.startswith("les "):
        return "aux " + nom[4:]
    return "à " + nom


def _tour(scenario: ScenarioPublic, pose: PoseCourante) -> tuple[AllerA, ...]:
    """Tous les lieux nommés, du plus proche au suivant le plus proche, puis retour au départ."""
    restants = [lieu for lieu in scenario.lieux if lieu.id != "depart"]
    x, y = pose.x, pose.y
    etapes: list[AllerA] = []
    while restants:
        lieu = min(restants, key=lambda lieu: math.hypot(lieu.x - x, lieu.y - y))
        restants.remove(lieu)
        etapes.append(AllerA(lieu.x, lieu.y, lieu.nom))
        x, y = lieu.x, lieu.y
    depart = scenario.lieu("depart")
    dx, dy = (depart.x, depart.y) if depart else (scenario.depart.x, scenario.depart.y)
    etapes.append(AllerA(dx, dy, "le départ"))
    return tuple(etapes)


def _exploration(scenario: ScenarioPublic, pose: PoseCourante) -> tuple[AllerA, ...]:
    """Balayage de la zone : grille 3 × 3 de points, du plus proche au suivant le plus proche."""
    lim = scenario.limites
    restants = [
        (lim.xmin + (lim.xmax - lim.xmin) * fx, lim.ymin + (lim.ymax - lim.ymin) * fy)
        for fx in (1 / 6, 1 / 2, 5 / 6)
        for fy in (1 / 6, 1 / 2, 5 / 6)
    ]
    x, y = pose.x, pose.y
    etapes: list[AllerA] = []
    while restants:
        p = min(restants, key=lambda q: math.hypot(q[0] - x, q[1] - y))
        restants.remove(p)
        etapes.append(AllerA(p[0], p[1], f"zone {len(etapes) + 1}/9"))
        x, y = p
    return tuple(etapes)


def _trajet(
    intention: Intention, scenario: ScenarioPublic, pose: PoseCourante
) -> tuple[Parcours | Refus, str]:
    """Forme libre (dessin, figure) : points relatifs au rover → étapes dans la zone."""
    bruts = intention.points or []
    c, s = math.cos(pose.cap), math.sin(pose.cap)
    etapes: list[AllerA] = []
    hors_zone = 0
    for point in bruts[:POINTS_TRAJET_MAX]:
        if len(point) != 2 or not all(math.isfinite(v) for v in point):
            continue
        avant, gauche = point
        if math.hypot(avant, gauche) > PORTEE_TRAJET_M:
            hors_zone += 1
            continue
        x, y = pose.x + c * avant - s * gauche, pose.y + s * avant + c * gauche
        if not scenario.limites.contient(x, y, MARGE_LIMITES_M):
            hors_zone += 1
            continue
        etapes.append(AllerA(x, y, f"point {len(etapes) + 1}"))
    if len(etapes) < 2:
        refus = "trajet impossible : pas assez de points dans la zone de mission"
        return Refus(refus), f"Consigne refusée : {refus}."
    total = len(etapes)
    etapes = [AllerA(e.x, e.y, f"point {i + 1}/{total}") for i, e in enumerate(etapes)]
    longueur = sum(
        math.hypot(b.x - a.x, b.y - a.y)
        for a, b in zip([AllerA(pose.x, pose.y, ""), *etapes[:-1]], etapes, strict=True)
    )
    note = f" ({hors_zone} point(s) hors zone ignorés)" if hors_zone else ""
    return Parcours(tuple(etapes)), f"Je suis le trajet : {total} points, {longueur:.0f} m{note}."


def traduire(
    intention: Intention, scenario: ScenarioPublic, pose: PoseCourante
) -> tuple[Action | Refus, str]:
    """Valide l'intention et renvoie (action ou refus, texte de réponse composé par le code)."""
    lim = scenario.limites
    match intention.action:
        case "s_arreter":
            return Arreter(), "Je m'arrête."
        case "reprendre":
            return Reprendre(), "Je reprends la mission interrompue."
        case "observer":
            return Observer(), "Je regarde avec la caméra…"
        case "vigilance_activer":
            return Vigilance(True), (
                "Vigilance activée : j'analyse la caméra pendant mes déplacements, à vitesse "
                "réduite."
            )
        case "vigilance_desactiver":
            return Vigilance(False), "Vigilance coupée : la caméra n'est plus analysée en route."
        case "revenir_depart":
            depart = scenario.lieu("depart")
            x, y = (depart.x, depart.y) if depart else (scenario.depart.x, scenario.depart.y)
            distance = math.hypot(x - pose.x, y - pose.y)
            return AllerA(
                x, y, "le départ"
            ), f"Je reviens au départ ({distance:.0f} m à vol d'oiseau)."
        case "aller_a":
            if intention.lieu is not None:
                lieu = scenario.lieu(intention.lieu)
                if lieu is None:
                    refus = "je ne connais pas ce lieu"
                    return Refus(refus), f"Consigne refusée : {refus}. Clique sur la carte."
                distance = math.hypot(lieu.x - pose.x, lieu.y - pose.y)
                return AllerA(lieu.x, lieu.y, lieu.nom), (
                    f"Je vais {a_lieu(lieu.nom)} ({distance:.0f} m à vol d'oiseau)."
                )
            if intention.x is not None and intention.y is not None:
                if not lim.contient(intention.x, intention.y, MARGE_LIMITES_M):
                    return Refus(
                        "point hors de la zone"
                    ), "Consigne refusée : ce point est hors de la zone de mission."
                return AllerA(
                    intention.x, intention.y, f"({intention.x:.1f} ; {intention.y:.1f})"
                ), (f"Je vais au point ({intention.x:.1f} ; {intention.y:.1f}).")
            refus = "destination inconnue"
            return Refus(
                refus
            ), f"Consigne refusée : {refus}. Nomme un lieu de la carte ou clique dessus."
        case "parcours":
            noms = [n for n in (intention.lieux or []) if scenario.lieu(n) is not None]
            if len(noms) < 2:
                refus = "il faut au moins deux lieux connus pour un parcours"
                return Refus(refus), f"Consigne refusée : {refus}."
            etapes = tuple(
                AllerA(lieu.x, lieu.y, lieu.nom)
                for n in noms[:ETAPES_MAX]
                if (lieu := scenario.lieu(n)) is not None
            )
            return Parcours(etapes), "Parcours : " + ", puis ".join(e.libelle for e in etapes) + "."
        case "faire_le_tour":
            etapes_tour = _tour(scenario, pose)
            return Parcours(etapes_tour), (
                "Je fais le tour : " + ", ".join(e.libelle for e in etapes_tour) + "."
            )
        case "trajet":
            return _trajet(intention, scenario, pose)
        case "chercher":
            if intention.cible not in CIBLES:
                refus = "je ne sais pas reconnaître cela avec la caméra"
                return Refus(refus), f"Consigne refusée : {refus}."
            nom = CIBLES[intention.cible]
            nombre = min(max(intention.nombre or 1, 1), 10)
            etapes_exploration = _exploration(scenario, pose)
            quoi = nom if nombre == 1 else f"{nombre} × {nom.split(' ', 1)[1]}"
            return Chercher(intention.cible, nom, etapes_exploration, nombre), (
                f"Je cherche {quoi} : j'explore la zone ({len(etapes_exploration)} points) et je "
                "m'arrête dès que la caméra les a vus."
            )
        case "deplacement_relatif":
            d, angle = intention.distance_m, intention.angle_deg or 0.0
            if d is None or not 0 < d <= DISTANCE_RELATIVE_MAX_M or not -360 <= angle <= 360:
                refus = f"distance invalide (entre 0 et {DISTANCE_RELATIVE_MAX_M:.0f} m)"
                return Refus(refus), f"Consigne refusée : {refus}."
            direction = pose.cap + math.radians(angle)
            x, y = pose.x + d * math.cos(direction), pose.y + d * math.sin(direction)
            if not lim.contient(x, y, MARGE_LIMITES_M):
                return Refus(
                    "point hors de la zone"
                ), "Consigne refusée : ce déplacement sortirait de la zone de mission."
            sens = "tout droit" if abs(angle) < 10 else ("à gauche" if angle > 0 else "à droite")
            if abs(angle) > 150:
                sens = "derrière moi"
            return AllerA(x, y, f"{d:.0f} m {sens}"), f"J'avance de {d:.0f} m {sens}."
        case "inconnu" if intention.reponse:
            return Repondre(), intention.reponse.strip()[:300]
        case _:
            refus = "consigne non comprise"
            return Refus(refus), (
                "Je ne sais pas faire ça. Je peux aller à un lieu, enchaîner plusieurs lieux, "
                "faire le tour de la carte, suivre une forme (« fais un carré de 6 m »), avancer "
                "ou reculer, m'arrêter et reprendre, ou décrire ce que je vois."
            )
