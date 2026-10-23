"""Carte d'occupation locale dans le repère odom : log-odds, lancer de rayons, fenêtre glissante.

Deux couches de même géométrie :

- **lidar** : chaque faisceau libère les cellules qu'il traverse (jusqu'à `portee_libre_m`) et
  marque occupée celle de son écho. Le plan laser est à 0,322 m : il passe au-dessus des
  obstacles bas, qu'il ne voit donc ni ne doit effacer.
- **bas** : obstacles de la stéréo (marches, poutres, pots, sous le plan du lidar), marqués par
  `integrer_obstacles`. Les rayons du lidar ne les effacent pas ; seule la stéréo le fait, quand
  elle voit le sol nu à leur place (cellules du champ de la caméra plus proches que le premier
  obstacle dans leur direction).

La grille couvre `cartographie.taille_m` (40 m) à `resolution_m` (5 cm) ; quand le rover
s'éloigne de plus d'un quart de la taille du centre, elle glisse d'un nombre entier de cellules
pour le recentrer (ce qui sort est oublié). Les cellules sous le rover sont libres.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from fieldmind.config import Profil
from fieldmind.types import CarteOccupation, Obstacles, Odometrie, Pose2D, Scan

L_OCCUPE = 0.85
L_LIBRE = -0.40
L_MIN, L_MAX = -2.0, 3.5
L_BAS_OCCUPE = 0.9
L_BAS_LIBRE = -0.3
L_BAS_MAX = 3.0
SEUIL_BAS = 0.5
"""Log-odds de la couche basse au-delà duquel la cellule est un obstacle (2 vues suffisent)."""
VALEUR_INDICATIVE = 55
"""Valeur dans la carte d'un obstacle bas non bloquant : sous le seuil d'occupation du
planificateur (65), au-dessus de « libre » ; le planificateur l'évite par son coût."""


def _pose(pose: Pose2D | Odometrie) -> Pose2D:
    return pose.pose if isinstance(pose, Odometrie) else pose


class Cartographe:
    def __init__(self, profil: Profil) -> None:
        v = profil.valeur
        self.resolution = float(v("cartographie.resolution_m", 0.05))
        self.taille_m = float(v("cartographie.taille_m", 40.0))
        self.portee_libre = float(v("cartographie.portee_libre_m", 8.0))
        self.portee_max = float(v("lidar.portee_max_m", 12.0))
        self.lidar_xy = (float(v("lidar.x_m", 0.0)), float(v("lidar.y_m", 0.0)))
        g = profil.geometrie
        self.emprise = (float(g.emprise_arriere_m), float(g.emprise_avant_m),
                        float(g.emprise_demi_largeur_m))
        self.camera_x = float(v("cameras.x_m", 0.415))
        self.camera_z = float(v("cameras.z_m", 0.265))
        self.hfov = math.radians(float(v("cameras.hfov_deg", 78.0)))
        self.inclinaison = math.radians(float(v("cameras.inclinaison_deg", 10.0)))
        # Couche basse oubliée avec le temps : une détection de la stéréo qui n'est pas revue
        # (fausse, ou sortie du champ) s'efface en quelques secondes au lieu de rester
        self.oubli_bas_s = float(v("cartographie.oubli_bas_s", 4.0))
        self.profil = profil
        self._t_bas: float | None = None
        self.n = int(round(self.taille_m / self.resolution))
        self._libres = np.zeros(self.n * self.n, dtype=bool)
        self.effacer()

    # ------------------------------------------------------------------ état
    def effacer(self) -> None:
        n = self.n
        self.lidar = np.zeros((n, n), dtype=np.float32)
        self.bas = np.zeros((n, n), dtype=np.float32)
        self.vue = np.zeros((n, n), dtype=bool)
        self.origine_x = -self.taille_m / 2
        self.origine_y = -self.taille_m / 2
        self.t = 0.0
        self.scans = 0

    def _indices(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Indices plats des points dans la grille, et masque des points dedans."""
        c = np.floor((x - self.origine_x) / self.resolution).astype(np.int64)
        lig = np.floor((y - self.origine_y) / self.resolution).astype(np.int64)
        dedans = (c >= 0) & (c < self.n) & (lig >= 0) & (lig < self.n)
        return lig * self.n + c, dedans

    def _recentrer(self, x: float, y: float) -> None:
        demi = self.taille_m / 2
        cx, cy = self.origine_x + demi, self.origine_y + demi
        if abs(x - cx) <= self.taille_m / 4 and abs(y - cy) <= self.taille_m / 4:
            return
        dc = int(round((x - cx) / self.resolution))
        dl = int(round((y - cy) / self.resolution))
        for nom in ("lidar", "bas", "vue"):
            setattr(self, nom, _decaler(getattr(self, nom), dl, dc))
        self.origine_x += dc * self.resolution
        self.origine_y += dl * self.resolution

    # ------------------------------------------------------------------ intégration
    def integrer_scan(self, scan: Scan, pose: Pose2D | Odometrie) -> None:
        """Tour du lidar (base_link) vu depuis `pose` (odom, à l'instant du tour)."""
        p = _pose(pose)
        self._recentrer(p.x, p.y)
        c, s = math.cos(p.cap), math.sin(p.cap)
        distances = np.asarray(scan.distances, dtype=np.float64)
        angles = np.asarray(scan.angles, dtype=np.float64)
        garde = (distances > 0) & (distances <= self.portee_max)
        distances, angles = distances[garde], angles[garde]
        bx = distances * np.cos(angles)
        by = distances * np.sin(angles)
        ex, ey = p.x + c * bx - s * by, p.y + s * bx + c * by  # échos dans odom
        lx, ly = p.vers_parent(*self.lidar_xy)
        # Faisceaux : du lidar jusqu'à une cellule avant l'écho, au plus `portee_libre`.
        dx, dy = ex - lx, ey - ly
        longueur = np.hypot(dx, dy)
        limite = np.minimum(longueur - self.resolution, self.portee_libre)
        pas = self.resolution * 0.7
        k = np.arange(0.0, max(float(limite.max(initial=0.0)), 0.0), pas)
        libres = self._libres
        libres[:] = False
        if k.size and len(longueur):
            dedans_rayon = k[None, :] < limite[:, None]
            ux, uy = dx / np.maximum(longueur, 1e-9), dy / np.maximum(longueur, 1e-9)
            xs = (lx + ux[:, None] * k[None, :])[dedans_rayon]
            ys = (ly + uy[:, None] * k[None, :])[dedans_rayon]
            indices, dedans = self._indices(xs, ys)
            libres[indices[dedans]] = True
        echos, dedans = self._indices(ex, ey)
        echos = echos[dedans]
        libres[echos] = False
        plat = self.lidar.reshape(-1)
        idx_libres = np.flatnonzero(libres)
        plat[idx_libres] = np.maximum(plat[idx_libres] + L_LIBRE, L_MIN)
        plat[echos] = np.minimum(plat[echos] + L_OCCUPE, L_MAX)
        vue = self.vue.reshape(-1)
        vue[idx_libres] = True
        vue[echos] = True
        self._liberer_emprise(p)
        self.t = max(self.t, float(scan.t))
        self.scans += 1

    def integrer_obstacles(self, obstacles: Obstacles, pose: Pose2D | Odometrie,
                           liberer: bool = True) -> None:
        """Obstacles (bas) de la stéréo, points base_link à l'instant `obstacles.t`.

        `liberer` : les cellules du champ de la caméra, plus proches que le premier obstacle
        dans leur direction et à portée, ont été vues comme du sol nu : leur marque basse
        diminue (un objet enlevé finit par disparaître de la carte).
        """
        p = _pose(pose)
        self._recentrer(p.x, p.y)
        if self.oubli_bas_s > 0:
            t = float(obstacles.t)
            if self._t_bas is not None and t > self._t_bas:
                self.bas *= math.exp(-(t - self._t_bas) / self.oubli_bas_s)
            self._t_bas = t if self._t_bas is None else max(self._t_bas, t)
        points = np.asarray(obstacles.points, dtype=np.float64).reshape(-1, 2)
        c, s = math.cos(p.cap), math.sin(p.cap)
        if liberer:
            self._liberer_champ(p, points, float(obstacles.portee_m))
        if len(points):
            x = p.x + c * points[:, 0] - s * points[:, 1]
            y = p.y + s * points[:, 0] + c * points[:, 1]
            indices, dedans = self._indices(x, y)
            indices = np.unique(indices[dedans])
            plat = self.bas.reshape(-1)
            plat[indices] = np.minimum(plat[indices] + L_BAS_OCCUPE, L_BAS_MAX)
            self.vue.reshape(-1)[indices] = True
        self._liberer_emprise(p)
        self.t = max(self.t, float(obstacles.t))

    def _liberer_champ(self, p: Pose2D, points: np.ndarray, portee: float) -> None:
        # Direction de chaque obstacle vue de la caméra, par tranches de 1°.
        demi = self.hfov / 2
        tranches = int(math.ceil(math.degrees(self.hfov)))
        premier = np.full(tranches, portee)
        if len(points):
            rx, ry = points[:, 0] - self.camera_x, points[:, 1]
            angle = np.arctan2(ry, rx)
            dans = np.abs(angle) < demi
            i = ((angle[dans] + demi) / self.hfov * tranches).astype(np.int64).clip(0,
                                                                                   tranches - 1)
            np.minimum.at(premier, i, np.hypot(rx[dans], ry[dans]))
        # Sol visible : au-delà du bas de l'image, jusqu'à un peu avant le premier obstacle.
        vfov_demi = math.atan(math.tan(demi) * 9 / 16)
        proche = self.camera_z / math.tan(min(self.inclinaison + vfov_demi, 1.4))
        r = np.arange(proche + 0.1, portee, self.resolution)
        a = -demi + (np.arange(tranches) + 0.5) * self.hfov / tranches
        rr, aa = np.meshgrid(r, a)
        garde = rr < (premier[:, None] - 0.15)
        if not garde.any():
            return
        bx = self.camera_x + rr[garde] * np.cos(aa[garde])
        by = rr[garde] * np.sin(aa[garde])
        c, s = math.cos(p.cap), math.sin(p.cap)
        indices, dedans = self._indices(p.x + c * bx - s * by, p.y + s * bx + c * by)
        indices = np.unique(indices[dedans])
        plat = self.bas.reshape(-1)
        plat[indices] = np.maximum(plat[indices] + L_BAS_LIBRE, 0.0)
        self.vue.reshape(-1)[indices] = True

    def _liberer_emprise(self, p: Pose2D) -> None:
        arriere, avant, demi = self.emprise
        xs = np.arange(arriere, avant + 1e-9, self.resolution / 2)
        ys = np.arange(-demi, demi + 1e-9, self.resolution / 2)
        gx, gy = np.meshgrid(xs, ys)
        c, s = math.cos(p.cap), math.sin(p.cap)
        indices, dedans = self._indices(p.x + c * gx.ravel() - s * gy.ravel(),
                                        p.y + s * gx.ravel() + c * gy.ravel())
        indices = indices[dedans]
        self.lidar.reshape(-1)[indices] = np.minimum(self.lidar.reshape(-1)[indices], L_LIBRE)
        self.bas.reshape(-1)[indices] = 0.0
        self.vue.reshape(-1)[indices] = True

    # ------------------------------------------------------------------ lecture
    def carte(self, t: float | None = None) -> CarteOccupation:
        """Grille int8 : -1 inconnu, 0 libre … 100 occupé (lidar, ou obstacle bas confirmé)."""
        probabilite = 1.0 / (1.0 + np.exp(-self.lidar))
        cellules = np.round(100 * probabilite).astype(np.int8)
        bas = self.bas > SEUIL_BAS
        if bool(self.profil.valeur("securite.stereo_bloquante", True)):
            cellules[bas] = 100
        else:
            # Indicatif, et seulement en amas (une cellule isolée est du bruit de la stéréo)
            voisins = cv2.boxFilter(bas.astype(np.uint8), cv2.CV_16U, (3, 3), normalize=False,
                                    borderType=cv2.BORDER_CONSTANT)
            amas = bas & (voisins >= 3)
            cellules[amas] = np.maximum(cellules[amas], VALEUR_INDICATIVE)
        cellules[~self.vue] = -1
        return CarteOccupation(t=self.t if t is None else t, resolution=self.resolution,
                               origine_x=self.origine_x, origine_y=self.origine_y,
                               cellules=cellules)

    def occupees(self) -> np.ndarray:
        """Masque des cellules occupées (lidar p > 0,65 ou obstacle bas)."""
        masque: np.ndarray = self.lidar > 0.62
        if bool(self.profil.valeur("securite.stereo_bloquante", True)):
            masque |= self.bas > SEUIL_BAS
        return masque

    def obstacles_proches(self, pose: Pose2D | Odometrie, rayon: float = 3.0,
                          bas_seulement: bool = False) -> Obstacles:
        """Cellules occupées à moins de `rayon` du rover, en points base_link : la mémoire des
        obstacles bas sortis du champ de la caméra (pour le pilote local et la sécurité)."""
        p = _pose(pose)
        r = int(math.ceil(rayon / self.resolution))
        c0 = int((p.x - self.origine_x) / self.resolution)
        l0 = int((p.y - self.origine_y) / self.resolution)
        l1, l2 = max(l0 - r, 0), min(l0 + r + 1, self.n)
        c1, c2 = max(c0 - r, 0), min(c0 + r + 1, self.n)
        if l1 >= l2 or c1 >= c2:
            return Obstacles(self.t, "carte", np.zeros((0, 2), np.float32), portee_m=rayon)
        bas = self.bas[l1:l2, c1:c2] > SEUIL_BAS
        masque = bas if bas_seulement else bas | (self.lidar[l1:l2, c1:c2] > 0.62)
        lig, col = np.nonzero(masque)
        x = self.origine_x + (c1 + col + 0.5) * self.resolution
        y = self.origine_y + (l1 + lig + 0.5) * self.resolution
        dx, dy = x - p.x, y - p.y
        proches = dx * dx + dy * dy <= rayon * rayon
        c, s = math.cos(p.cap), math.sin(p.cap)
        bx = c * dx[proches] + s * dy[proches]
        by = -s * dx[proches] + c * dy[proches]
        return Obstacles(self.t, "carte", np.column_stack((bx, by)).astype(np.float32),
                         portee_m=rayon)


def _decaler(tableau: np.ndarray, dl: int, dc: int) -> np.ndarray:
    """Contenu décalé de (dl, dc) cellules (la nouvelle origine est l'ancienne + décalage) ;
    ce qui entre est vide."""
    sortie = np.zeros_like(tableau)
    h, w = tableau.shape
    if abs(dl) >= h or abs(dc) >= w:
        return sortie
    src_l = slice(max(dl, 0), h + min(dl, 0))
    dst_l = slice(max(-dl, 0), h + min(-dl, 0))
    src_c = slice(max(dc, 0), w + min(dc, 0))
    dst_c = slice(max(-dc, 0), w + min(-dc, 0))
    sortie[dst_l, dst_c] = tableau[src_l, src_c]
    return sortie
