"""Calcul du chemin : carte de coûts, A* et lissage.

- La grille d'occupation (0,10 m) est ramenée à 0,20 m (une cellule est occupée si l'une de ses
  quatre sous-cellules l'est), puis gonflée : le centre du rover doit rester à plus de
  `rayon_gonflement` de tout obstacle (demi-largeur + marge).
- Coût d'une cellule : 1, plus une pénalité qui croît près des obstacles (le rover préfère le
  milieu des passages) et une pénalité pour l'inconnu (il y va quand c'est plus court, en
  recalculant au fur et à mesure qu'il découvre).
- A* en 8-connexité, heuristique octile × `ponderation_heuristique` (1 = optimal ; au-dessus,
  le chemin coûte au plus ce facteur fois l'optimum). La boucle d'exploration est compilée par
  numba (x86-64 et ARM64, donc aussi sur le GX10).
- Lissage par raccourcis en ligne droite, sans jamais réduire la distance aux obstacles sous celle
  du chemin d'origine, puis rééchantillonnage tous les 0,25 m.
"""

from __future__ import annotations

import heapq
import itertools
import math
import time
from dataclasses import dataclass

import numpy as np
from numba import njit
from scipy import ndimage

from services.navigation.grille import GrilleOccupation

RACINE_2 = math.sqrt(2.0)


@dataclass(frozen=True, slots=True)
class ParametresPlanification:
    resolution: float = 0.20
    rayon_gonflement: float = 0.50
    """Rayon du cercle qui contient l'emprise (0,44 m) plus 6 cm de marge : les coins comptent."""
    distance_confort: float = 1.3
    penalite_proximite: float = 4.0
    penalite_inconnu: float = 0.6
    ponderation_heuristique: float = 1.1
    pas_reechantillonnage: float = 0.25
    rayon_liberation_depart: float = 0.8
    rayon_rover_strict: float = 0.30
    """Distance minimale absolue du centre du rover à un obstacle (demi-largeur)."""


@dataclass(frozen=True, slots=True)
class CarteCouts:
    resolution: float
    origine: tuple[float, float]
    distance: np.ndarray
    """Distance (m) de chaque cellule à l'obstacle le plus proche."""
    bloquee: np.ndarray
    cout: np.ndarray

    @property
    def forme(self) -> tuple[int, int]:
        h, w = self.bloquee.shape
        return int(h), int(w)

    def cellule(self, x: float, y: float) -> tuple[int, int]:
        """(ligne, colonne) de la cellule contenant le point (x, y), bornée à la carte."""
        h, w = self.forme
        c = math.floor((x - self.origine[0]) / self.resolution)
        lig = math.floor((y - self.origine[1]) / self.resolution)
        return min(max(lig, 0), h - 1), min(max(c, 0), w - 1)

    def centre(self, lig: int, c: int) -> tuple[float, float]:
        return (
            self.origine[0] + (c + 0.5) * self.resolution,
            self.origine[1] + (lig + 0.5) * self.resolution,
        )

    def distance_en(self, x: float, y: float) -> float:
        lig, c = self.cellule(x, y)
        return float(self.distance[lig, c])


@dataclass(frozen=True, slots=True)
class Plan:
    points: list[tuple[float, float]]
    objectif: tuple[float, float]
    """Objectif effectivement visé (déplacé hors d'un obstacle si nécessaire)."""
    objectif_deplace: float
    cout: float
    longueur: float
    duree_ms: float
    noeuds_explores: int


class EchecPlanification(Exception):
    """Aucun chemin : le rover ou l'objectif est enfermé, ou hors de la carte."""


class DepartBloque(EchecPlanification):
    """Le rover touche presque un obstacle : il doit d'abord s'en écarter."""


def construire_carte_couts(
    grille: GrilleOccupation,
    p: ParametresPlanification,
    obstacles_supplementaires: np.ndarray | None = None,
    sol_verifie: np.ndarray | None = None,
) -> CarteCouts:
    """Carte de coûts du lidar, plus d'éventuels obstacles d'une autre source (caméra), donnés
    comme un masque booléen à la résolution de la grille. `sol_verifie` : cellules où la caméra
    a vu un sol nu de près ; un écho du lidar y est le sol d'une montée, pas un obstacle."""
    facteur = max(1, round(p.resolution / grille.resolution))
    occupee = grille.occupee()
    if sol_verifie is not None:
        occupee = occupee & ~sol_verifie
    if obstacles_supplementaires is not None:
        occupee = occupee | obstacles_supplementaires
    observee = grille.observee
    h, w = occupee.shape
    h2, w2 = math.ceil(h / facteur), math.ceil(w / facteur)
    occ = np.zeros((h2 * facteur, w2 * facteur), dtype=bool)
    occ[:h, :w] = occupee
    obs = np.zeros_like(occ)
    obs[:h, :w] = observee
    occ_r = occ.reshape(h2, facteur, w2, facteur).any(axis=(1, 3))
    obs_r = obs.reshape(h2, facteur, w2, facteur).any(axis=(1, 3))
    resolution = grille.resolution * facteur

    # Distance au plus proche obstacle ; les bords de la zone comptent comme des obstacles.
    libre = ~occ_r
    libre[0, :] = libre[-1, :] = libre[:, 0] = libre[:, -1] = False
    distance = ndimage.distance_transform_edt(libre) * resolution
    bloquee = distance < p.rayon_gonflement
    proximite = np.clip(
        (p.distance_confort - distance) / (p.distance_confort - p.rayon_gonflement), 0.0, 1.0
    )
    cout = 1.0 + p.penalite_proximite * proximite**2 + p.penalite_inconnu * (~obs_r)
    return CarteCouts(resolution, grille.origine, distance, bloquee, cout)


def _cellule_libre_la_plus_proche(bloquee: np.ndarray, lig: int, c: int) -> tuple[int, int]:
    if not bloquee[lig, c]:
        return lig, c
    _, indices = ndimage.distance_transform_edt(bloquee, return_indices=True)
    return int(indices[0][lig, c]), int(indices[1][lig, c])


@njit(cache=True)
def _explorer(
    cout: np.ndarray,
    interdit: np.ndarray,
    h: int,
    w: int,
    source: int,
    cible: int,
    ponderation: float,
) -> tuple[np.ndarray, float, int]:  # pragma: no cover - compilé par numba
    """Boucle A* compilée : renvoie le tableau des parents, le coût à la cible et les nœuds vus."""
    n_cellules = h * w
    g = np.full(n_cellules, np.inf)
    parent = np.full(n_cellules, -1, dtype=np.int64)
    ferme = np.zeros(n_cellules, dtype=np.bool_)
    li, ci = cible // w, cible % w
    dl_v = np.array([-1, 1, 0, 0, -1, -1, 1, 1])
    dc_v = np.array([0, 0, -1, 1, -1, 1, -1, 1])
    longueur_v = np.array([1.0, 1.0, 1.0, 1.0, RACINE_2, RACINE_2, RACINE_2, RACINE_2])
    g[source] = 0.0
    tas = [(0.0, 0.0, source)]
    explores = 0
    while len(tas) > 0:
        _, g_n, n = heapq.heappop(tas)
        if ferme[n]:
            continue
        if n == cible:
            return parent, g_n, explores
        ferme[n] = True
        explores += 1
        lig, c = n // w, n % w
        for k in range(8):
            ll, cc = lig + dl_v[k], c + dc_v[k]
            if ll < 0 or ll >= h or cc < 0 or cc >= w:
                continue
            m = ll * w + cc
            if interdit[m] or ferme[m]:
                continue
            # Diagonale : interdite si elle coupe le coin d'une cellule bloquée.
            if k >= 4 and (interdit[lig * w + cc] or interdit[ll * w + c]):
                continue
            g_m = g_n + longueur_v[k] * 0.5 * (cout[n] + cout[m])
            if g_m < g[m]:
                g[m] = g_m
                parent[m] = n
                dl, dc = abs(ll - li), abs(cc - ci)
                heur = ponderation * ((dl + dc) + (RACINE_2 - 2.0) * min(dl, dc))
                heapq.heappush(tas, (g_m + heur, g_m, m))
    return parent, np.inf, explores


def a_etoile(
    carte: CarteCouts,
    depart: tuple[int, int],
    arrivee: tuple[int, int],
    bloquee: np.ndarray,
    ponderation: float = 1.0,
) -> tuple[list[tuple[int, int]], float, int]:
    """A* 8-connexe sur la carte ; renvoie (cellules, coût, nœuds explorés)."""
    h, w = carte.forme
    source = depart[0] * w + depart[1]
    cible = arrivee[0] * w + arrivee[1]
    parent, cout, explores = _explorer(
        np.ascontiguousarray(carte.cout, dtype=np.float64).ravel(),
        np.ascontiguousarray(bloquee).ravel(),
        h,
        w,
        source,
        cible,
        ponderation,
    )
    if not math.isfinite(cout):
        raise EchecPlanification("aucun chemin entre le rover et l'objectif")
    chemin = [cible]
    while chemin[-1] != source:
        chemin.append(int(parent[chemin[-1]]))
    chemin.reverse()
    return [(k // w, k % w) for k in chemin], float(cout), int(explores)


def prechauffer() -> None:
    """Compile la boucle A* (1 à 2 s la première fois, puis mise en cache sur disque)."""
    cout = np.ones(16)
    _explorer(cout, np.zeros(16, dtype=np.bool_), 4, 4, 0, 15, 1.0)


def _segment_sur(
    carte: CarteCouts, a: tuple[float, float], b: tuple[float, float], seuil: float
) -> bool:
    """Vrai si tout le segment [a, b] reste à au moins `seuil` des obstacles (échantillonné)."""
    longueur = math.dist(a, b)
    n = max(2, math.ceil(longueur / (carte.resolution * 0.5)))
    f = np.linspace(0.0, 1.0, n + 1)
    xs = a[0] + f * (b[0] - a[0])
    ys = a[1] + f * (b[1] - a[1])
    h, w = carte.forme
    c = np.clip(np.floor((xs - carte.origine[0]) / carte.resolution).astype(np.int64), 0, w - 1)
    lig = np.clip(np.floor((ys - carte.origine[1]) / carte.resolution).astype(np.int64), 0, h - 1)
    return bool(carte.distance[lig, c].min() >= seuil)


def lisser(
    carte: CarteCouts, points: list[tuple[float, float]], seuil_min: float
) -> list[tuple[float, float]]:
    """Raccourcis gloutons : on relie i au point j le plus lointain sans perdre de marge."""
    if len(points) <= 2:
        return points
    marges = [carte.distance_en(x, y) for x, y in points]
    resultat = [points[0]]
    i = 0
    while i < len(points) - 1:
        j_retenu = i + 1
        marge_sous_chemin = marges[i + 1]
        for j in range(i + 2, len(points)):
            marge_sous_chemin = min(marge_sous_chemin, marges[j - 1], marges[j])
            seuil = max(seuil_min, min(marge_sous_chemin, 1.0))
            if _segment_sur(carte, points[i], points[j], seuil):
                j_retenu = j
            elif j - j_retenu > 6:
                break
        resultat.append(points[j_retenu])
        i = j_retenu
    return resultat


def reechantillonner(points: list[tuple[float, float]], pas: float) -> list[tuple[float, float]]:
    if len(points) < 2:
        return points
    tableau = np.asarray(points)
    segments = np.linalg.norm(np.diff(tableau, axis=0), axis=1)
    abscisse = np.concatenate([[0.0], np.cumsum(segments)])
    cibles = np.append(np.arange(0.0, abscisse[-1], pas), abscisse[-1])
    xs = np.interp(cibles, abscisse, tableau[:, 0])
    ys = np.interp(cibles, abscisse, tableau[:, 1])
    return [(round(float(x), 3), round(float(y), 3)) for x, y in zip(xs, ys, strict=True)]


def planifier(
    carte: CarteCouts,
    depart: tuple[float, float],
    objectif: tuple[float, float],
    p: ParametresPlanification,
) -> Plan:
    debut = time.perf_counter()
    bloquee = carte.bloquee.copy()
    ld, cd = carte.cellule(*depart)
    # Rover déjà dans la zone gonflée (près d'un tronc) : on l'autorise à en sortir, tant qu'il
    # reste à plus de sa demi-largeur de l'obstacle.
    if bloquee[ld, cd]:
        r = math.ceil(p.rayon_liberation_depart / carte.resolution)
        h, w = carte.forme
        l0, l1, c0, c1 = max(ld - r, 0), min(ld + r + 1, h), max(cd - r, 0), min(cd + r + 1, w)
        zone = carte.distance[l0:l1, c0:c1] >= p.rayon_rover_strict
        bloquee[l0:l1, c0:c1] &= ~zone
        if bloquee[ld, cd]:
            raise DepartBloque("le rover touche presque un obstacle")
    lo, co = carte.cellule(*objectif)
    lo2, co2 = _cellule_libre_la_plus_proche(bloquee, lo, co)
    objectif_effectif = carte.centre(lo2, co2) if (lo2, co2) != (lo, co) else objectif
    deplace = math.dist(objectif_effectif, objectif)
    cellules, cout, explores = a_etoile(
        carte, (ld, cd), (lo2, co2), bloquee, p.ponderation_heuristique
    )
    brut = [depart] + [carte.centre(lig, c) for lig, c in cellules[1:-1]] + [objectif_effectif]
    lisse = lisser(carte, brut, p.rayon_gonflement)
    points = reechantillonner(lisse, p.pas_reechantillonnage)
    longueur = float(sum(math.dist(a, b) for a, b in itertools.pairwise(points)))
    return Plan(
        points=points,
        objectif=(round(objectif_effectif[0], 3), round(objectif_effectif[1], 3)),
        objectif_deplace=deplace,
        cout=cout * carte.resolution,
        longueur=longueur,
        duree_ms=(time.perf_counter() - debut) * 1000,
        noeuds_explores=explores,
    )


def chemin_bloque(
    carte: CarteCouts, points: list[tuple[float, float]], depuis: int, horizon_m: float
) -> bool:
    """Vrai si le chemin traverse une zone bloquée sur `horizon_m` après l'indice `depuis`."""
    parcouru = 0.0
    for k in range(max(depuis, 1), len(points)):
        parcouru += math.dist(points[k - 1], points[k])
        if parcouru > horizon_m:
            break
        lig, c = carte.cellule(*points[k])
        if carte.bloquee[lig, c]:
            return True
    return False
