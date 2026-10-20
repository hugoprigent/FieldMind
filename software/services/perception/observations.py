"""Observations de la caméra placées au sol, et couche « vision » de la carte.

Un objet vu (boîte englobante) devient une observation datée, avec une **emprise au sol** : le
bas de la boîte (contact avec le sol) projeté sur le sol plan donne le bord proche ; l'emprise
couvre la zone où il peut se trouver (± l'erreur de ligne, jusqu'à 3 m de profondeur pour une
vue lointaine) et s'étend de `EPAISSEUR_M` au-delà (l'épaisseur d'un tronc). Le haut de la
boîte ne donne pas la profondeur : la caméra est à 27 cm du sol, le haut d'un objet bas est
toujours près de l'horizon. Un tronc en biais est donc couvert par l'accumulation des vues
prises en approchant, chacune montrant son bord le plus proche. L'incertitude vient de la
géométrie de la caméra (un pixel près de l'horizon vaut des mètres).

Seuls les obstacles bas, invisibles pour le lidar (plan de mesure à 0,32 m), entrent dans la
carte : tronc couché, branche, trou, eau, et « autre » s'il bloque le passage. Les rochers et les
arbres sont déjà cartographiés par le lidar, plus précisément.

La couche vision est distincte de la grille du lidar : les rayons du lidar passent au-dessus d'un
tronc couché et l'effaceraient. Les vues s'accumulent (un doublon exact est ignoré) ; une vue
nettement plus précise remplace les vues lointaines qu'elle recoupe. Une vue
fausse ou mal placée (de loin) est retirée par preuve négative : trois analyses où sa zone est
nettement visible (proche, dans le champ, non masquée par un obstacle du lidar) sans que rien n'y
soit revu.
"""

from __future__ import annotations

import itertools
import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from services.perception.camera import ERREUR_V, ModeleCamera
from services.perception.vision import (
    NOMS_CATEGORIES,
    PLURIELS_CATEGORIES,
    AnalyseVision,
    DemandeVision,
)
from services.protocole.scenario import Limites

Point = tuple[float, float]

CATEGORIES_CARTE = frozenset({"tronc_couche", "branche", "trou", "eau"})
PORTEE_CARTE_M = 12.0
"""Au-delà, la position au sol est trop incertaine pour la carte (listée seulement)."""
CONFIANCE_MIN = 0.35
EPAISSEUR_M = 0.4
PROFONDEUR_INCERTAINE_MAX_M = 3.0
"""Au plus, l'emprise couvre 3 m de profondeur d'incertitude (vue lointaine)."""
"""Profondeur de l'emprise au-delà du bord proche : l'épaisseur d'un tronc couché."""
LARGEUR_MAX_M = 6.0
MARGE_MIN = 0.1
MARGE_MAX = 0.5
OBSERVATIONS_MAX = 60
FUSION_MAX_M = 3.0
ABSENCES_POUR_RETRAIT = 3
DOUBLON_M = 0.15
VISIBILITE_M = (1.0, 5.0)
"""Distances où l'absence d'un objet dans l'image est une preuve (détection fiable)."""


@dataclass(frozen=True, slots=True)
class Observation:
    id: int
    t: float
    categorie: str
    x: float
    """Centre de l'emprise au sol."""
    y: float
    contour: tuple[Point, Point, Point, Point]
    """Emprise au sol : proche gauche, proche droite, lointain droite, lointain gauche."""
    distance: float
    """Distance du bord proche à la caméra (m)."""
    incertitude: float
    bloque: bool
    confiance: float
    boite: tuple[float, float, float, float]
    """Boîte dans l'image analysée (0–1000)."""
    dans_carte: bool
    """Vrai si l'observation est un obstacle de la couche vision."""
    demande: int

    @property
    def marge(self) -> float:
        """Marge autour de l'emprise, selon l'incertitude latérale."""
        return min(max(0.25 * self.incertitude, MARGE_MIN), MARGE_MAX)

    @property
    def bord_proche(self) -> tuple[Point, Point]:
        return self.contour[0], self.contour[1]

    @property
    def nom(self) -> str:
        return NOMS_CATEGORIES.get(self.categorie, self.categorie)


def _borner_largeur(a: Point, b: Point) -> tuple[Point, Point]:
    longueur = math.dist(a, b)
    if longueur <= LARGEUR_MAX_M:
        return a, b
    f = LARGEUR_MAX_M / longueur
    cx, cy = 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])
    return (cx + (a[0] - cx) * f, cy + (a[1] - cy) * f), (
        cx + (b[0] - cx) * f,
        cy + (b[1] - cy) * f,
    )


def placer_objets(
    analyse: AnalyseVision, demande: DemandeVision, camera: ModeleCamera, premier_id: int
) -> list[Observation]:
    """Place au sol les objets de l'analyse ; ceux dont le bas est au-dessus de l'horizon sont
    ignorés."""
    observations: list[Observation] = []
    pose = (demande.x, demande.y, demande.cap)
    for k, objet in enumerate(analyse.objets):
        x0, _, x1, y1 = (v / 1000.0 for v in objet.boite)
        gauche = camera.point_au_sol(x0, y1, *pose)
        droite = camera.point_au_sol(x1, y1, *pose)
        milieu = camera.point_au_sol(0.5 * (x0 + x1), y1, *pose)
        if gauche is None or droite is None or milieu is None:
            continue
        # Emprise : toute la zone où peut se trouver le bord proche (± l'erreur de ligne), plus
        # l'épaisseur de l'objet. Près du rover, quelques décimètres ; de loin, jusqu'à 3 m.
        avant_min = camera.avant_pour_ligne(min(y1 + ERREUR_V, 1.0), 0.0)
        avant_max = camera.avant_pour_ligne(max(y1 - ERREUR_V, 0.0), 0.0)
        avant_max = min(avant_max, avant_min + PROFONDEUR_INCERTAINE_MAX_M) + EPAISSEUR_M
        bords = []
        for avant in (avant_min, avant_max):
            v = camera.ligne(avant, 0.0)
            g = camera.point_au_sol(x0, v, *pose)
            d = camera.point_au_sol(x1, v, *pose)
            if g is None or d is None:
                g, d = gauche, droite
            bords.append(_borner_largeur((g.x, g.y), (d.x, d.y)))
        proche, lointain = bords
        contour = (proche[0], proche[1], lointain[1], lointain[0])
        obstacle_bas = objet.categorie in CATEGORIES_CARTE or (
            objet.categorie == "autre" and objet.bloque_passage
        )
        dans_carte = (
            obstacle_bas
            and objet.bloque_passage
            and objet.confiance >= CONFIANCE_MIN
            and milieu.distance <= PORTEE_CARTE_M
        )
        observations.append(
            Observation(
                id=premier_id + k,
                t=demande.t,
                categorie=objet.categorie,
                x=sum(p[0] for p in contour) / 4,
                y=sum(p[1] for p in contour) / 4,
                contour=contour,
                distance=milieu.distance,
                incertitude=max(gauche.incertitude, droite.incertitude, milieu.incertitude),
                bloque=objet.bloque_passage,
                confiance=objet.confiance,
                boite=objet.boite,
                dans_carte=dans_carte,
                demande=demande.id,
            )
        )
    return observations


# ------------------------------------------------------------------ géométrie plane


def distance_point_segment(p: Point, s: tuple[Point, Point]) -> float:
    (ax, ay), (bx, by) = s
    dx, dy = bx - ax, by - ay
    n2 = dx * dx + dy * dy
    t = 0.0 if n2 < 1e-12 else min(max(((p[0] - ax) * dx + (p[1] - ay) * dy) / n2, 0.0), 1.0)
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def _aretes(poly: tuple[Point, ...]) -> list[tuple[Point, Point]]:
    return [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]


def _contient(poly: tuple[Point, ...], p: Point) -> bool:
    signes = {
        (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) > 0 for a, b in _aretes(poly)
    }
    return len(signes) == 1


def distance_point_polygone(p: Point, poly: tuple[Point, ...]) -> float:
    """Distance d'un point à un polygone convexe (0 à l'intérieur)."""
    if _contient(poly, p):
        return 0.0
    return min(distance_point_segment(p, arete) for arete in _aretes(poly))


def distance_polygones(a: tuple[Point, ...], b: tuple[Point, ...]) -> float:
    """Distance entre deux polygones convexes (0 s'ils se chevauchent)."""
    if any(_contient(a, p) for p in b) or any(_contient(b, p) for p in a):
        return 0.0
    return min(
        min(distance_point_polygone(p, b) for p in a),
        min(distance_point_polygone(p, a) for p in b),
    )


def _rayon_fusion(vue: Observation) -> float:
    """Zone où se trouve l'objet vu, compte tenu de l'incertitude de la vue."""
    return min(vue.incertitude, FUSION_MAX_M) + 0.3


# ------------------------------------------------------------------ couche vision


class CoucheVision:
    """Obstacles bas vus par la caméra, rastérisés à la résolution de la grille du lidar."""

    def __init__(self, limites: Limites, resolution: float) -> None:
        self.resolution = resolution
        self.origine = (limites.xmin, limites.ymin)
        self.largeur = math.ceil((limites.xmax - limites.xmin) / resolution)
        self.hauteur = math.ceil((limites.ymax - limites.ymin) / resolution)
        self.obstacles: list[Observation] = []
        self._absences: dict[int, int] = {}
        self.masque = np.zeros((self.hauteur, self.largeur), dtype=bool)
        """Remplacé (jamais modifié sur place) : le flux lent peut le lire sans verrou."""

    def fusionner(self, nouvelles: list[Observation]) -> list[Observation]:
        """Ajoute les obstacles de la carte (hors doublons exacts) ; renvoie les ajouts. Les
        anciennes vues revues dans cette analyse redeviennent « confirmées »."""
        ajouts: list[Observation] = []
        for obs in (o for o in nouvelles if o.dans_carte):
            meme_categorie = [a for a in self.obstacles if a.categorie == obs.categorie]
            for a in meme_categorie:
                if distance_polygones(a.contour, obs.contour) < _rayon_fusion(obs):
                    self._absences.pop(a.id, None)
            if any(
                distance_polygones(a.contour, obs.contour) < DOUBLON_M
                and a.incertitude <= 1.5 * obs.incertitude
                for a in meme_categorie
            ):
                continue
            # Une vue nettement plus précise remplace les vues lointaines qu'elle recoupe.
            imprecises = [
                a
                for a in meme_categorie
                if a.incertitude > 2 * obs.incertitude
                and distance_polygones(a.contour, obs.contour) == 0.0
            ]
            self.obstacles = [o for o in self.obstacles if o not in imprecises]
            self.obstacles.append(obs)
            ajouts.append(obs)
        self.obstacles = self.obstacles[-OBSERVATIONS_MAX:]
        if ajouts:
            self._rasteriser()
        return ajouts

    def constater_absences(
        self,
        demande: DemandeVision,
        camera: ModeleCamera,
        nouvelles: list[Observation],
        masque_par: Callable[[Point, Point], bool],
    ) -> list[Observation]:
        """Preuve négative : les obstacles nettement visibles dans l'image analysée mais qui n'y
        ont pas été revus. Retirés après `ABSENCES_POUR_RETRAIT` analyses ; renvoie les retirés.

        `masque_par(a, b)` : vrai si un obstacle connu (lidar) cache b vu depuis a.
        """
        pose = (demande.x, demande.y, demande.cap)
        cx, cy, _ = camera.camera.position
        c, s = math.cos(demande.cap), math.sin(demande.cap)
        oeil = (demande.x + c * cx - s * cy, demande.y + s * cx + c * cy)
        retires: list[Observation] = []
        for obs in list(self.obstacles):
            if any(
                n.categorie == obs.categorie
                and distance_polygones(n.contour, obs.contour) < _rayon_fusion(n)
                for n in nouvelles
            ):
                continue
            (ax, ay), (bx, by) = obs.bord_proche
            cible = (0.5 * (ax + bx), 0.5 * (ay + by))
            image = camera.projeter(cible[0], cible[1], 0.0, *pose)
            distance = math.dist(oeil, cible)
            if (
                image is None
                or not (0.1 <= image[0] <= 0.9 and image[1] <= 0.98)
                or not VISIBILITE_M[0] <= distance <= VISIBILITE_M[1]
                or masque_par(oeil, cible)
            ):
                continue
            self._absences[obs.id] = self._absences.get(obs.id, 0) + 1
            if self._absences[obs.id] >= ABSENCES_POUR_RETRAIT:
                retires.append(obs)
        if retires:
            self.obstacles = [o for o in self.obstacles if o not in retires]
            for obs in retires:
                self._absences.pop(obs.id, None)
            self._rasteriser()
        return retires

    def _rasteriser(self) -> None:
        masque = np.zeros((self.hauteur, self.largeur), dtype=bool)
        r = self.resolution
        for obs in self.obstacles:
            e = obs.marge
            xs = [p[0] for p in obs.contour]
            ys = [p[1] for p in obs.contour]
            c0 = max(0, math.floor((min(xs) - e - self.origine[0]) / r))
            c1 = min(self.largeur, math.ceil((max(xs) + e - self.origine[0]) / r) + 1)
            l0 = max(0, math.floor((min(ys) - e - self.origine[1]) / r))
            l1 = min(self.hauteur, math.ceil((max(ys) + e - self.origine[1]) / r) + 1)
            if c0 >= c1 or l0 >= l1:
                continue
            px, py = np.meshgrid(
                self.origine[0] + (np.arange(c0, c1) + 0.5) * r,
                self.origine[1] + (np.arange(l0, l1) + 0.5) * r,
            )
            positifs = np.ones_like(px, dtype=bool)
            negatifs = np.ones_like(px, dtype=bool)
            proche = np.zeros_like(px, dtype=bool)
            for (ax, ay), (bx, by) in _aretes(obs.contour):
                produit = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
                positifs &= produit >= 0
                negatifs &= produit <= 0
                dx, dy = bx - ax, by - ay
                n2 = max(dx * dx + dy * dy, 1e-12)
                t = np.clip(((px - ax) * dx + (py - ay) * dy) / n2, 0.0, 1.0)
                proche |= np.hypot(px - ax - t * dx, py - ay - t * dy) <= e
            masque[l0:l1, c0:c1] |= positifs | negatifs | proche
        self.masque = masque


def decrire(analyse: AnalyseVision, observations: list[Observation]) -> str:
    """Texte de réponse composé par le code à partir des objets placés."""
    if not analyse.objets:
        return "Je ne vois aucun obstacle notable devant moi."
    places = sorted(observations, key=lambda o: o.distance)
    morceaux = []
    for categorie, groupe in itertools.groupby(
        sorted(places, key=lambda o: o.categorie), key=lambda o: o.categorie
    ):
        liste = list(groupe)
        plus_proche = min(o.distance for o in liste)
        noms = PLURIELS_CATEGORIES if len(liste) > 1 else NOMS_CATEGORIES
        texte = f"{len(liste)} {noms.get(categorie, categorie)}"
        morceaux.append((plus_proche, f"{texte} (le plus proche à {plus_proche:.0f} m)"))
    morceaux.sort()
    lointains = len(analyse.objets) - len(observations)
    phrase = "Je vois " + ", ".join(m for _, m in morceaux) if morceaux else "Je vois"
    if lointains > 0:
        phrase += (", " if morceaux else " ") + f"{lointains} objet(s) trop loin pour être placés"
    phrase += "."
    carte = [o for o in observations if o.dans_carte]
    if carte:
        phrase += f" {len(carte)} obstacle(s) bas ajouté(s) à la carte : je les contournerai."
    return phrase
