"""Carte d'élévation locale et traversabilité, sans dépendance à ROS (testable seule).

La carte est une grille glissante centrée sur le rover, alignée sur la gravité, dans le repère
`odom` (x, y) ; les hauteurs sont relatives à une altitude intégrée par le nœud. Chaque cellule
garde, pour la dernière observation de la caméra de profondeur, la hauteur la plus basse (le sol)
et la plus haute sous `hauteur_max` au-dessus de ce sol (branches plus hautes ignorées : le rover
passe dessous). Les impacts du lidar 2D sont gardés à part.

Une cellule est un obstacle si :
- une marche dépasse `pas_max` par rapport au sol voisin (rocher, tronc, mur, souche) ;
- la pente du sol dépasse `pente_max_deg` (flanc de butte, berge de ruisseau) ;
- le lidar la touche au-dessus du sol vu par la caméra, ou sans que la caméra y ait vu le sol
  (prudence : un lidar 2D ne distingue pas une pente d'un mur).

Un trou (obstacle « négatif » : fossé, berge qui plonge) ne se voit pas : on le déduit d'une
zone masquée entre deux zones de sol vues, près du rover, là où l'échantillonnage est dense.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

INCONNU, LIBRE, OBSTACLE = -1, 0, 1
MARCHE, PENTE, LIDAR = 1, 2, 3


@dataclass
class Classement:
    etat: np.ndarray
    """INCONNU, LIBRE ou OBSTACLE par cellule (lignes = y, colonnes = x), tous capteurs."""
    profondeur: np.ndarray
    """Même chose sans les obstacles vus par le seul lidar (ceux-là ont leur propre couche dans
    les costmaps, effacée par les rayons du lidar)."""
    sol: np.ndarray
    """Hauteur de référence du sol (NaN si inconnue)."""
    raison: np.ndarray
    """Pourquoi une cellule est un obstacle : 0 aucune, MARCHE, PENTE ou LIDAR."""
    pente_deg: np.ndarray
    x0: float
    y0: float


class CarteElevation:
    def __init__(
        self,
        resolution: float = 0.1,
        taille: float = 24.0,
        pas_max: float = 0.15,
        pente_max_deg: float = 22.0,
        hauteur_max: float = 1.2,
        memoire_s: float = 60.0,
        memoire_lidar_s: float = 4.0,
        points_min: int = 2,
        recalage_min: int = 40,
        recalage_max: float = 0.5,
        portee_sol_lidar: float = 1.5,
        lidar_proche: float = 1.2,
    ) -> None:
        self.res = resolution
        self.n = round(taille / resolution)
        self.pas_max = pas_max
        self.pente_max = pente_max_deg
        self.hauteur_max = hauteur_max
        self.memoire = memoire_s
        self.memoire_lidar = memoire_lidar_s
        self.points_min = points_min
        self.recalage_min = recalage_min
        self.recalage_max = recalage_max
        self.portee_sol_lidar = portee_sol_lidar
        self.lidar_proche = lidar_proche
        self.x0 = self.y0 = -self.n * resolution / 2
        forme = (self.n, self.n)
        self.zmin = np.full(forme, np.nan)
        self.zmax = np.full(forme, np.nan)
        self.t_vu = np.full(forme, -np.inf)
        self.zlidar = np.full(forme, np.nan)
        self.t_lidar = np.full(forme, -np.inf)

    # ------------------------------------------------------------------ grille glissante
    def centre(self) -> tuple[float, float]:
        m = self.n * self.res / 2
        return self.x0 + m, self.y0 + m

    def recentrer(self, x: float, y: float) -> None:
        """Décale la grille (par cellules entières) si le rover s'éloigne du centre."""
        cx, cy = self.centre()
        if abs(x - cx) < self.n * self.res / 6 and abs(y - cy) < self.n * self.res / 6:
            return
        di = round((x - cx) / self.res)
        dj = round((y - cy) / self.res)
        for nom, vide in (
            ("zmin", np.nan),
            ("zmax", np.nan),
            ("t_vu", -np.inf),
            ("zlidar", np.nan),
            ("t_lidar", -np.inf),
        ):
            setattr(self, nom, _decaler(getattr(self, nom), dj, di, vide))
        self.x0 += di * self.res
        self.y0 += dj * self.res

    def indices(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        ix = np.floor((np.asarray(x) - self.x0) / self.res).astype(np.int64)
        iy = np.floor((np.asarray(y) - self.y0) / self.res).astype(np.int64)
        dedans = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        return iy, ix, dedans

    # ------------------------------------------------------------------ observations
    def ajouter_nuage(self, points: np.ndarray, t: float, recaler: bool = True) -> float:
        """Points (N, 3) du capteur de profondeur, dans le repère de la carte. Une cellule revue
        est remplacée (la dernière vue fait foi).

        L'altitude du rover est intégrée et dérive : avant d'insérer, le nuage est recalé en
        hauteur sur le sol déjà cartographié (médiane des écarts sur les cellules communes).
        Renvoie ce décalage (m), à retrancher de l'altitude du rover."""
        iy, ix, ok = self.indices(points[:, 0], points[:, 1])
        if not ok.any():
            return 0.0
        z = points[ok, 2]
        k = iy[ok] * self.n + ix[ok]
        taille = self.n * self.n
        fmin = np.full(taille, np.inf)
        np.minimum.at(fmin, k, z)
        nombre = np.bincount(k, minlength=taille)
        decalage = 0.0
        if recaler:
            commun = (nombre >= self.points_min) & (self.t_vu.ravel() > t - self.memoire)
            if commun.sum() >= self.recalage_min:
                decalage = float(np.median(fmin[commun] - self.zmin.ravel()[commun]))
                decalage = max(-self.recalage_max, min(self.recalage_max, decalage))
                z = z - decalage
                fmin = fmin - decalage
        bas = z < fmin[k] + self.hauteur_max
        fmax = np.full(taille, -np.inf)
        np.maximum.at(fmax, k[bas], z[bas])
        vu = (nombre >= self.points_min).reshape(self.n, self.n)
        self.zmin[vu] = fmin.reshape(self.n, self.n)[vu]
        self.zmax[vu] = fmax.reshape(self.n, self.n)[vu]
        self.t_vu[vu] = t
        return decalage

    def ajouter_lidar(self, points: np.ndarray, t: float) -> None:
        iy, ix, ok = self.indices(points[:, 0], points[:, 1])
        if not ok.any():
            return
        k = iy[ok] * self.n + ix[ok]
        fmax = np.full(self.n * self.n, -np.inf)
        np.maximum.at(fmax, k, points[ok, 2])
        touche = np.isfinite(fmax).reshape(self.n, self.n)
        self.zlidar[touche] = fmax.reshape(self.n, self.n)[touche]
        self.t_lidar[touche] = t

    # ------------------------------------------------------------------ traversabilité
    def classer(self, t: float, robot: tuple[float, float] | None = None) -> Classement:
        vu = self.t_vu > t - self.memoire
        sol = np.where(vu, self.zmin, np.nan)
        # Sol de référence : le plus bas des cellules voisines (3 × 3).
        ref = ndimage.minimum_filter(
            np.where(vu, sol, np.inf), size=3, mode="constant", cval=np.inf
        )
        ref = np.where(np.isfinite(ref), ref, np.nan)
        with np.errstate(invalid="ignore"):
            marche = vu & (self.zmax - ref > self.pas_max)
        pente = _pente_deg(sol, self.res)
        with np.errstate(invalid="ignore"):
            raide = pente > self.pente_max
        lidar_obstacle = self._obstacles_lidar(t, sol, ref, pente, robot)
        etat = np.full(sol.shape, INCONNU, np.int8)
        etat[vu] = LIBRE
        etat[marche | raide] = OBSTACLE
        profondeur = etat.copy()
        etat[lidar_obstacle] = OBSTACLE
        raison = np.zeros(sol.shape, np.int8)
        raison[lidar_obstacle] = LIDAR
        raison[raide] = PENTE
        raison[marche] = MARCHE
        return Classement(etat, profondeur, ref, raison, pente, self.x0, self.y0)

    def _obstacles_lidar(self, t, sol, ref, pente, robot) -> np.ndarray:
        """Impacts du lidar 2D retenus comme obstacles.

        Un lidar 2D ne distingue pas un mur d'une montée : son faisceau (0,32 m au-dessus du sol,
        parallèle au rover) touche le sol dès que le terrain se relève. On compare donc l'impact
        au sol vu par la caméra :
        - sol vu dans la cellule : obstacle s'il dépasse le sol de plus de `pas_max` ;
        - sol vu à moins de `portee_sol_lidar` : même test, en prolongeant ce sol avec sa pente
          mesurée (plus une marge) sur la distance qui les sépare ;
        - rien de connu autour : obstacle seulement tout près du rover (moins de
          `lidar_proche` m), où une montée assez forte pour être touchée n'est pas plausible.
        """
        lidar = self.t_lidar > t - self.memoire_lidar
        if not lidar.any():
            return lidar
        with np.errstate(invalid="ignore"):
            confirme = lidar & (self.zlidar - ref > self.pas_max)
        sans_ref = lidar & np.isnan(ref)
        if sans_ref.any():
            connu = np.isfinite(sol)
            if connu.any():
                dist, (iy, ix) = ndimage.distance_transform_edt(~connu, return_indices=True)
                dist = dist * self.res
                sol_proche = sol[iy, ix]
                pente_proche = np.nan_to_num(pente[iy, ix], nan=self.pente_max)
                pres = sans_ref & (dist <= self.portee_sol_lidar)
                montee = np.radians(np.minimum(pente_proche + 8.0, self.pente_max))
                tolerance = self.pas_max + dist * np.tan(montee)
                with np.errstate(invalid="ignore"):
                    confirme |= pres & (self.zlidar - sol_proche > tolerance)
                sans_ref &= dist > self.portee_sol_lidar
            if robot is not None and sans_ref.any():
                iy, ix = np.nonzero(sans_ref)
                cx = self.x0 + (ix + 0.5) * self.res
                cy = self.y0 + (iy + 0.5) * self.res
                proche = np.hypot(cx - robot[0], cy - robot[1]) < self.lidar_proche
                confirme[iy[proche], ix[proche]] = True
        return confirme

    def rayons(
        self,
        c: Classement,
        x: float,
        y: float,
        nombre: int = 360,
        portee: float = 8.0,
        debut: float = 0.5,
        trou_min: float = 0.4,
        portee_trous: float = 3.0,
        tolerance_trou: float = 0.1,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Lance `nombre` rayons depuis (x, y). Renvoie, par rayon, l'angle (repère de la carte),
        la distance libre (jusqu'au dernier sol vu avant un obstacle ou une zone masquée) et la
        distance d'un trou (ou NaN).

        Une zone masquée entre deux sols vus est un trou (fossé, berge) si le sol qui suit est au
        moins aussi haut que le bord, à `tolerance_trou` près : un sol plan ne cache rien. Si le
        sol qui suit est plus bas, c'est une crête suivie d'une descente : la zone reste inconnue,
        la caméra la verra en arrivant sur la crête."""
        angles = np.linspace(-math.pi, math.pi, nombre, endpoint=False)
        pas = self.res / 2
        d = np.arange(debut, portee, pas)
        px = x + np.cos(angles)[:, None] * d
        py = y + np.sin(angles)[:, None] * d
        iy, ix, ok = self.indices(px, py)
        e = np.full(px.shape, INCONNU, np.int8)
        e[ok] = c.profondeur[iy[ok], ix[ok]]
        z = np.full(px.shape, np.nan)
        z[ok] = c.sol[iy[ok], ix[ok]]
        libre = np.zeros(nombre)
        trou = np.full(nombre, np.nan)
        n_trou = round(trou_min / pas)
        for r in range(nombre):
            ligne = e[r]
            obst = np.flatnonzero(ligne == OBSTACLE)
            fin = obst[0] if obst.size else ligne.size
            vus = np.flatnonzero(ligne[:fin] == LIBRE)
            if not vus.size:
                continue
            # Trou : sol vu, puis au moins trou_min de masqué, puis de nouveau du sol vu.
            ecarts = np.diff(vus) - 1
            grands = np.flatnonzero((ecarts >= n_trou) & (d[vus[:-1]] < portee_trous))
            if grands.size:
                k = vus[grands[0]]
                suivant = vus[grands[0] + 1]
                libre[r] = d[k]
                if not z[r, suivant] < z[r, k] - tolerance_trou:
                    trou[r] = d[k + 1]
            else:
                libre[r] = d[vus[-1]]
        return angles, libre, trou

    def est_sol(self, c: Classement, points: np.ndarray) -> np.ndarray:
        """Vrai pour les impacts (lidar) posés sur le sol vu par la caméra (à filtrer du scan)."""
        iy, ix, ok = self.indices(points[:, 0], points[:, 1])
        sol = np.zeros(len(points), bool)
        ref = np.full(len(points), np.nan)
        ref[ok] = c.sol[iy[ok], ix[ok]]
        libre = np.zeros(len(points), bool)
        libre[ok] = c.etat[iy[ok], ix[ok]] != OBSTACLE
        with np.errstate(invalid="ignore"):
            sol = libre & (points[:, 2] - ref <= self.pas_max)
        return sol


def _decaler(a: np.ndarray, dj: int, di: int, vide: float) -> np.ndarray:
    """Décale le contenu de (dj lignes, di colonnes) vers l'origine ; remplit avec `vide`."""
    b = np.full_like(a, vide)
    n = a.shape[0]
    if abs(dj) >= n or abs(di) >= n:
        return b
    sj = slice(max(dj, 0), n + min(dj, 0))
    si = slice(max(di, 0), n + min(di, 0))
    tj = slice(max(-dj, 0), n + min(-dj, 0))
    ti = slice(max(-di, 0), n + min(-di, 0))
    b[tj, ti] = a[sj, si]
    return b


def _pente_deg(sol: np.ndarray, res: float) -> np.ndarray:
    """Pente du sol (degrés) sur une base de 0,2 m, après lissage 3 × 3 ; NaN si inconnue."""
    connu = np.isfinite(sol)
    somme = ndimage.uniform_filter(np.where(connu, sol, 0.0), size=3, mode="constant")
    poids = ndimage.uniform_filter(connu.astype(float), size=3, mode="constant")
    with np.errstate(invalid="ignore", divide="ignore"):
        lisse = np.where(poids > 0.3, somme / poids, np.nan)
    gx = np.full(sol.shape, np.nan)
    gy = np.full(sol.shape, np.nan)
    gx[:, 1:-1] = (lisse[:, 2:] - lisse[:, :-2]) / (2 * res)
    gy[1:-1, :] = (lisse[2:, :] - lisse[:-2, :]) / (2 * res)
    pente = np.degrees(np.arctan(np.hypot(gx, gy)))
    return np.where(connu, pente, np.nan)
