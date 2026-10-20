"""Planification de chemin sur la carte d'occupation (repère odom).

1. **Recadrage** sur le départ et le but (plus `navigation.marge_recadrage_m`), puis grille de
   planification à `navigation.resolution_m` (0,10 m), grossie si elle dépasse
   `navigation.cellules_max` (40 × 40 m passent alors à 0,20 m).
2. **Distance aux obstacles** (transformée en distance d'OpenCV sur la grille fine de la carte,
   cellules occupées ≥ `navigation.seuil_occupe`), lue au centre de chaque cellule de
   planification. Une cellule est interdite si l'essieu arrière y passerait à moins de la
   demi-largeur du rover + `navigation.marge_m` d'un obstacle (carte gonflée).
3. **Coût** d'une cellule : 1, plus une pénalité de proximité qui croît sous
   `navigation.distance_confort_m` (le rover préfère le milieu des passages), plus une pénalité
   pour l'inconnu (traversable : on y va quand c'est plus court, et on replanifie en découvrant).
4. **Recherche** 8-connexe (pas de diagonale qui coupe le coin d'une cellule interdite) : A*
   compilé par numba s'il est installé ; sinon Dijkstra de scipy (C), qui rend le même chemin
   optimal (A* en Python pur serait 10 fois trop lent sur 40 × 40 m).
5. **Lissage** par raccourcis en ligne droite qui ne passent jamais plus près des obstacles que le
   chemin d'origine (marge plafonnée à la distance de confort), puis points tous les
   `navigation.pas_chemin_m`.

Départ dans la zone gonflée (rover garé près d'un mur) : on l'autorise à en sortir tant qu'il
reste à plus de sa demi-largeur de l'obstacle. But dans un obstacle : déplacé vers la cellule
libre la plus proche (au plus `navigation.deplacement_but_max_m`), sinon pas de chemin.
"""

from __future__ import annotations

import heapq
import math
import time
from typing import Any

import cv2
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from fieldmind.config import Profil
from fieldmind.types import CarteOccupation, Chemin, Pose2D

RACINE_2 = math.sqrt(2.0)
VOISINS = ((0, 1, 1.0), (1, 0, 1.0), (1, 1, RACINE_2), (1, -1, RACINE_2))
"""(dl, dc, longueur) : la moitié des 8 voisins, chaque arête est ajoutée dans les deux sens."""

try:  # numba est facultatif (paquet python3-numba sur le Pi)
    from numba import njit as _njit
except ImportError:  # pragma: no cover - dépend de l'installation
    _njit = None


def _a_etoile_noyau(cout: np.ndarray, interdit: np.ndarray, h: int, w: int, source: int,
                    cible: int, ponderation: float) -> np.ndarray:  # pragma: no cover
    """A* 8-connexe sur une grille aplatie ; renvoie le tableau des parents (-1 : non atteint,
    parent[cible] = -1 si pas de chemin). Écrit pour numba (tableaux et tas de tuples)."""
    n = h * w
    g = np.full(n, np.inf)
    parent = np.full(n, -1, dtype=np.int64)
    ferme = np.zeros(n, dtype=np.bool_)
    lc, cc = cible // w, cible % w
    dl_v = np.array([-1, 1, 0, 0, -1, -1, 1, 1])
    dc_v = np.array([0, 0, -1, 1, -1, 1, -1, 1])
    longueur_v = np.array([1.0, 1.0, 1.0, 1.0, RACINE_2, RACINE_2, RACINE_2, RACINE_2])
    g[source] = 0.0
    tas = [(0.0, source)]
    while len(tas) > 0:
        _, m = heapq.heappop(tas)
        if ferme[m]:
            continue
        if m == cible:
            break
        ferme[m] = True
        lig, col = m // w, m % w
        for k in range(8):
            ll, c2 = lig + dl_v[k], col + dc_v[k]
            if ll < 0 or ll >= h or c2 < 0 or c2 >= w:
                continue
            q = ll * w + c2
            if interdit[q] or ferme[q]:
                continue
            if k >= 4 and (interdit[lig * w + c2] or interdit[ll * w + col]):
                continue
            g_q = g[m] + longueur_v[k] * 0.5 * (cout[m] + cout[q])
            if g_q < g[q]:
                g[q] = g_q
                parent[q] = m
                dl, dc = abs(ll - lc), abs(c2 - cc)
                heur = ponderation * ((dl + dc) + (RACINE_2 - 2.0) * min(dl, dc))
                heapq.heappush(tas, (g_q + heur, q))
    return parent


_A_ETOILE = _njit(cache=True)(_a_etoile_noyau) if _njit is not None else None


class Planificateur:
    def __init__(self, profil: Profil) -> None:
        v = profil.valeur
        self.demi_largeur = float(v("geometrie.emprise_demi_largeur_m", 0.225))
        self.marge = float(v("navigation.marge_m", 0.10))
        self.rayon_gonflement = self.demi_largeur + self.marge
        self.distance_confort = float(v("navigation.distance_confort_m", 0.8))
        self.penalite_proximite = float(v("navigation.penalite_proximite", 2.0))
        self.penalite_inconnu = float(v("navigation.penalite_inconnu", 0.5))
        self.seuil_occupe = int(v("navigation.seuil_occupe", 65))
        self.resolution = float(v("navigation.resolution_m", 0.10))
        self.cellules_max = int(v("navigation.cellules_max", 40000))
        self.marge_recadrage = float(v("navigation.marge_recadrage_m", 5.0))
        self.deplacement_but_max = float(v("navigation.deplacement_but_max_m", 1.0))
        self.pas_chemin = float(v("navigation.pas_chemin_m", 0.10))
        self.ponderation = float(v("navigation.ponderation_heuristique", 1.0))
        self.utiliser_numba = bool(v("navigation.numba", True)) and _A_ETOILE is not None
        self.dernier_bilan: dict[str, Any] = {}

    # ------------------------------------------------------------------ planification
    def planifier(self, carte: CarteOccupation, depart: Pose2D | tuple[float, float],
                  but: Pose2D | tuple[float, float]) -> Chemin | None:
        """Chemin (odom) de l'essieu arrière du départ au but, ou None s'il n'y en a pas."""
        debut = time.perf_counter()
        dx, dy = (depart.x, depart.y) if isinstance(depart, Pose2D) else map(float, depart[:2])
        bx, by = (but.x, but.y) if isinstance(but, Pose2D) else map(float, but[:2])
        res = float(carte.resolution)
        h_c, w_c = carte.cellules.shape
        # Recadrage (indices de la carte fine).
        x0 = min(dx, bx) - self.marge_recadrage
        x1 = max(dx, bx) + self.marge_recadrage
        y0 = min(dy, by) - self.marge_recadrage
        y1 = max(dy, by) + self.marge_recadrage
        c0 = max(int(math.floor((x0 - carte.origine_x) / res)), 0)
        c1 = min(int(math.ceil((x1 - carte.origine_x) / res)), w_c)
        l0 = max(int(math.floor((y0 - carte.origine_y) / res)), 0)
        l1 = min(int(math.ceil((y1 - carte.origine_y) / res)), h_c)
        if c0 >= c1 or l0 >= l1:
            return None
        fine = carte.cellules[l0:l1, c0:c1]
        occupe = fine >= self.seuil_occupe
        libre = np.where(occupe, 0, 255).astype(np.uint8)
        distance_fine = cv2.distanceTransform(libre, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        distance_fine = distance_fine.astype(np.float32) * res
        if not occupe.any():
            distance_fine[:] = 1e3
        # Grille de planification.
        facteur = max(1, int(round(self.resolution / res)))
        while math.ceil(fine.shape[0] / facteur) * math.ceil(fine.shape[1] / facteur) \
                > self.cellules_max:
            facteur += 1
        pas = facteur * res
        h, w = math.ceil(fine.shape[0] / facteur), math.ceil(fine.shape[1] / facteur)
        centres_l = np.minimum(np.arange(h) * facteur + facteur // 2, fine.shape[0] - 1)
        centres_c = np.minimum(np.arange(w) * facteur + facteur // 2, fine.shape[1] - 1)
        distance = distance_fine[np.ix_(centres_l, centres_c)]
        inconnu = fine[np.ix_(centres_l, centres_c)] < 0
        interdit = distance < self.rayon_gonflement
        proximite = np.clip((self.distance_confort - distance)
                            / max(self.distance_confort - self.rayon_gonflement, 1e-3), 0.0, 1.0)
        cout = 1.0 + self.penalite_proximite * proximite**2 + self.penalite_inconnu * inconnu
        ox = carte.origine_x + c0 * res
        oy = carte.origine_y + l0 * res

        def cellule(x: float, y: float) -> tuple[int, int]:
            return (min(max(int((y - oy) / pas), 0), h - 1), min(max(int((x - ox) / pas), 0),
                                                                  w - 1))

        def centre(lig: int, col: int) -> tuple[float, float]:
            # Centre de la cellule fine échantillonnée (là où la distance a été lue).
            return (ox + (centres_c[col] + 0.5) * res, oy + (centres_l[lig] + 0.5) * res)

        ld, cd = cellule(dx, dy)
        if interdit[ld, cd]:
            r = int(math.ceil(1.0 / pas))
            zone = (slice(max(ld - r, 0), ld + r + 1), slice(max(cd - r, 0), cd + r + 1))
            interdit[zone] &= distance[zone] < self.demi_largeur * 0.9
            if interdit[ld, cd]:
                self._bilan(debut, "depart_bloque", facteur * res, 0)
                return None
        lb, cb = cellule(bx, by)
        but_effectif = (bx, by)
        hors_carte = not (0 <= (by - oy) / pas < h and 0 <= (bx - ox) / pas < w)
        if interdit[lb, cb] or hors_carte:
            libre_l, libre_c = np.nonzero(~interdit)
            if not len(libre_l):
                self._bilan(debut, "aucune_cellule_libre", pas, 0)
                return None
            px = ox + (centres_c[libre_c] + 0.5) * res
            py = oy + (centres_l[libre_l] + 0.5) * res
            ecarts = np.hypot(px - bx, py - by)
            k = int(np.argmin(ecarts))
            if ecarts[k] > self.deplacement_but_max:
                self._bilan(debut, "but_inaccessible", pas, 0)
                return None
            lb, cb = int(libre_l[k]), int(libre_c[k])
            but_effectif = (float(px[k]), float(py[k]))

        source, cible = ld * w + cd, lb * w + cb
        if self.utiliser_numba and _A_ETOILE is not None:
            parents = _A_ETOILE(np.ascontiguousarray(cout, np.float64).ravel(),
                                np.ascontiguousarray(interdit).ravel(), h, w, source, cible,
                                self.ponderation)
            methode = "a_etoile_numba"
        else:
            parents = _dijkstra(cout, interdit, source)
            methode = "dijkstra_scipy"
        if source != cible and parents[cible] < 0:
            self._bilan(debut, "aucun_chemin", pas, 0, methode)
            return None
        indices = [cible]
        while indices[-1] != source:
            indices.append(int(parents[indices[-1]]))
        indices.reverse()
        brut = [(dx, dy)] + [centre(i // w, i % w) for i in indices[1:-1]] + [but_effectif]

        def marge(x: float, y: float) -> float:
            lig = min(max(int((y - carte.origine_y) / res) - l0, 0), fine.shape[0] - 1)
            col = min(max(int((x - carte.origine_x) / res) - c0, 0), fine.shape[1] - 1)
            return float(distance_fine[lig, col])

        lisse = _lisser(brut, marge, self.rayon_gonflement, self.distance_confort, res)
        points = _reechantillonner(lisse, self.pas_chemin)
        longueur = float(np.sum(np.hypot(*np.diff(points, axis=0).T))) if len(points) > 1 else 0.0
        self._bilan(debut, "ok", pas, len(indices), methode)
        return Chemin(t=float(carte.t), points=points, but=but_effectif, longueur_m=longueur)

    def _bilan(self, debut: float, etat: str, pas: float, cellules: int,
               methode: str = "") -> None:
        self.dernier_bilan = {"etat": etat, "duree_ms": round(1000 * (time.perf_counter() - debut),
                                                              2),
                              "resolution_m": round(pas, 3), "cellules_chemin": cellules,
                              "methode": methode}


def _dijkstra(cout: np.ndarray, interdit: np.ndarray, source: int) -> np.ndarray:
    """Parents du plus court chemin depuis `source` (graphe 8-connexe des cellules permises)."""
    h, w = cout.shape
    permis = ~interdit
    cout_plat = cout.ravel()
    lignes, colonnes, valeurs = [], [], []
    indices = np.arange(h * w).reshape(h, w)
    for dl, dc, longueur in VOISINS:
        # Paires (a, b) avec b = a + (dl, dc), toutes deux dans la grille.
        la = slice(0, h - dl)
        ca = slice(max(-dc, 0), w - max(dc, 0))
        lb = slice(dl, h)
        cb = slice(max(dc, 0), w - max(-dc, 0))
        ok = permis[la, ca] & permis[lb, cb]
        if dl and dc:  # diagonale : les deux cellules du coin doivent être permises
            ok &= permis[la, cb] & permis[lb, ca]
        a = indices[la, ca][ok]
        b = indices[lb, cb][ok]
        poids = longueur * 0.5 * (cout_plat[a] + cout_plat[b])
        lignes += [a, b]
        colonnes += [b, a]
        valeurs += [poids, poids]
    graphe = csr_matrix((np.concatenate(valeurs), (np.concatenate(lignes),
                                                   np.concatenate(colonnes))), shape=(h * w,) * 2)
    _, parents = dijkstra(graphe, indices=source, return_predecessors=True)
    parents = np.asarray(parents, dtype=np.int64)
    parents[parents < 0] = -1
    return parents


def _lisser(points: list[tuple[float, float]], marge: Any, seuil_min: float, plafond: float,
            pas_controle: float) -> list[tuple[float, float]]:
    """Raccourcis gloutons : de chaque point, on va au plus lointain atteignable en ligne droite
    sans passer plus près des obstacles que le chemin d'origine (plafonné à `plafond`)."""
    if len(points) <= 2:
        return points
    marges = [marge(x, y) for x, y in points]

    def segment_sur(a: tuple[float, float], b: tuple[float, float], seuil: float) -> bool:
        n = max(2, int(math.ceil(math.dist(a, b) / pas_controle)))
        for k in range(n + 1):
            f = k / n
            if marge(a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])) < seuil:
                return False
        return True

    resultat = [points[0]]
    i = 0
    while i < len(points) - 1:
        retenu = i + 1
        sous_chemin = marges[i + 1]
        for j in range(i + 2, len(points)):
            sous_chemin = min(sous_chemin, marges[j])
            seuil = max(seuil_min, min(sous_chemin, plafond)) - 1e-6
            if segment_sur(points[i], points[j], seuil):
                retenu = j
            elif j - retenu > 8:
                break
        resultat.append(points[retenu])
        i = retenu
    return resultat


def _reechantillonner(points: list[tuple[float, float]], pas: float) -> np.ndarray:
    tableau = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if len(tableau) < 2:
        return tableau
    segments = np.hypot(*np.diff(tableau, axis=0).T)
    abscisse = np.concatenate([[0.0], np.cumsum(segments)])
    if abscisse[-1] < 1e-9:
        return tableau[:1]
    cibles = np.append(np.arange(0.0, abscisse[-1], pas), abscisse[-1])
    return np.column_stack((np.interp(cibles, abscisse, tableau[:, 0]),
                            np.interp(cibles, abscisse, tableau[:, 1])))
