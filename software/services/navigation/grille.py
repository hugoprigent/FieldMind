"""Grille d'occupation construite à partir des balayages lidar (log-odds, numpy vectorisé).

Chaque faisceau libère les cellules qu'il traverse et marque occupée la cellule de son écho. Un
faisceau sans écho libère seulement les premiers mètres : il peut s'agir d'une mesure perdue.
Le poids d'un balayage diminue quand la pose est incertaine, jusqu'à être ignoré.
"""

from __future__ import annotations

import base64
import math
import zlib
from dataclasses import dataclass

import numpy as np

from services.protocole.messages import Scan
from services.protocole.scenario import Limites

L_OCCUPE = 0.85
L_LIBRE = -0.40
L_MIN, L_MAX = -2.5, 3.5
SEUIL_OCCUPE = 0.6
"""Log-odds au-delà duquel une cellule est considérée occupée (p ≈ 0,65)."""
PORTEE_SANS_ECHO = 3.0
SIGMA_POSE_PLEINE = 0.10
SIGMA_POSE_MAX = 0.50
INCONNU = 255


@dataclass(frozen=True, slots=True)
class PoseCapteur:
    x: float
    y: float
    cap: float


class GrilleOccupation:
    def __init__(self, limites: Limites, resolution: float = 0.1) -> None:
        self.resolution = resolution
        self.origine = (limites.xmin, limites.ymin)
        self.largeur = math.ceil((limites.xmax - limites.xmin) / resolution)
        self.hauteur = math.ceil((limites.ymax - limites.ymin) / resolution)
        self.log_odds = np.zeros((self.hauteur, self.largeur), dtype=np.float32)
        self.observee = np.zeros((self.hauteur, self.largeur), dtype=bool)
        self.version = 0
        self._sale: list[int] | None = None  # [c0, l0, c1, l1] modifié depuis le dernier envoi

    # ------------------------------------------------------------------ coordonnées
    def cellule(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        c = np.floor((x - self.origine[0]) / self.resolution).astype(np.int64)
        lig = np.floor((y - self.origine[1]) / self.resolution).astype(np.int64)
        return c, lig

    def centre(self, c: int, lig: int) -> tuple[float, float]:
        return (
            self.origine[0] + (c + 0.5) * self.resolution,
            self.origine[1] + (lig + 0.5) * self.resolution,
        )

    def _dans(self, c: np.ndarray, lig: np.ndarray) -> np.ndarray:
        masque: np.ndarray = (c >= 0) & (c < self.largeur) & (lig >= 0) & (lig < self.hauteur)
        return masque

    # ------------------------------------------------------------------ intégration
    @staticmethod
    def poids_pour(sigma_xy: float) -> float:
        if sigma_xy <= SIGMA_POSE_PLEINE:
            return 1.0
        if sigma_xy >= SIGMA_POSE_MAX:
            return 0.0
        return 1.0 - (sigma_xy - SIGMA_POSE_PLEINE) / (SIGMA_POSE_MAX - SIGMA_POSE_PLEINE)

    def integrer(
        self, pose: PoseCapteur, scan: Scan, poids: float = 1.0, sol: np.ndarray | None = None
    ) -> int:
        """Intègre un balayage vu depuis `pose` (pose du lidar). Renvoie le nombre d'échos.

        `sol` : faisceaux arrêtés par le sol (montée), libres jusqu'à l'impact mais sans obstacle.
        """
        if poids <= 0:
            return 0
        n = len(scan.distances)
        mesures = np.array([np.nan if d is None else d for d in scan.distances], dtype=np.float64)
        if sol is not None and sol.any():
            limite_sol = np.where(sol, mesures - self.resolution, np.nan)
        else:
            limite_sol = None
        echo = ~np.isnan(mesures)
        if limite_sol is not None and sol is not None:
            echo &= ~sol
        angles = pose.cap + scan.angle_min + scan.pas * np.arange(n)
        cos_a, sin_a = np.cos(angles), np.sin(angles)

        # Espace libre le long des faisceaux, jusqu'à une demi-cellule avant l'écho.
        limite = np.where(echo, np.nan_to_num(mesures) - self.resolution, PORTEE_SANS_ECHO)
        if limite_sol is not None:
            limite = np.where(np.isnan(limite_sol), limite, limite_sol)
        pas = self.resolution * 0.5
        t = np.arange(0.0, max(float(limite.max()), 0.0), pas)
        if t.size:
            dedans = t[None, :] < limite[:, None]
            xs = pose.x + cos_a[:, None] * t[None, :]
            ys = pose.y + sin_a[:, None] * t[None, :]
            c, lig = self.cellule(xs[dedans], ys[dedans])
            ok = self._dans(c, lig)
            libres = np.unique(lig[ok] * self.largeur + c[ok])
        else:
            libres = np.zeros(0, dtype=np.int64)

        # Cellules des échos.
        d = mesures[echo]
        c, lig = self.cellule(pose.x + cos_a[echo] * d, pose.y + sin_a[echo] * d)
        ok = self._dans(c, lig)
        occupees = np.unique(lig[ok] * self.largeur + c[ok])
        libres = np.setdiff1d(libres, occupees, assume_unique=True)

        plat = self.log_odds.reshape(-1)
        vues = self.observee.reshape(-1)
        plat[libres] = np.maximum(plat[libres] + L_LIBRE * poids, L_MIN)
        plat[occupees] = np.minimum(plat[occupees] + L_OCCUPE * poids, L_MAX)
        vues[libres] = True
        vues[occupees] = True
        self._marquer_sale(np.concatenate([libres, occupees]))
        self.version += 1
        return int(occupees.size)

    def liberer_disque(self, x: float, y: float, rayon: float) -> None:
        """Le rover occupe cet espace : il est donc libre d'obstacle."""
        r = math.ceil(rayon / self.resolution)
        c0, l0 = (int(v) for v in self.cellule(np.array(x), np.array(y)))
        cs = np.arange(max(c0 - r, 0), min(c0 + r + 1, self.largeur))
        ls = np.arange(max(l0 - r, 0), min(l0 + r + 1, self.hauteur))
        if cs.size == 0 or ls.size == 0:
            return
        cc, ll = np.meshgrid(cs, ls)
        dedans = (cc - c0) ** 2 + (ll - l0) ** 2 <= r * r
        self.log_odds[ll[dedans], cc[dedans]] = np.minimum(
            self.log_odds[ll[dedans], cc[dedans]], L_LIBRE
        )
        self.observee[ll[dedans], cc[dedans]] = True
        self._marquer_sale(ll[dedans] * self.largeur + cc[dedans])

    def _marquer_sale(self, indices: np.ndarray) -> None:
        if indices.size == 0:
            return
        lig, c = np.divmod(indices, self.largeur)
        rect = [int(c.min()), int(lig.min()), int(c.max()) + 1, int(lig.max()) + 1]
        if self._sale is None:
            self._sale = rect
        else:
            s = self._sale
            self._sale = [
                min(s[0], rect[0]),
                min(s[1], rect[1]),
                max(s[2], rect[2]),
                max(s[3], rect[3]),
            ]

    # ------------------------------------------------------------------ lecture
    def occupee(self) -> np.ndarray:
        masque: np.ndarray = self.log_odds > SEUIL_OCCUPE
        return masque

    def obstacle_entre(
        self, a: tuple[float, float], b: tuple[float, float], sauf_fin: float = 0.4
    ) -> bool:
        """Vrai si une cellule occupée se trouve sur le segment a → b (hors des `sauf_fin`
        derniers mètres) : b est caché depuis a par un obstacle vu au lidar."""
        longueur = math.dist(a, b) - sauf_fin
        if longueur <= 0:
            return False
        t = np.linspace(0.0, longueur / math.dist(a, b), max(2, math.ceil(longueur / 0.05)))
        c, lig = self.cellule(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        dedans = self._dans(c, lig)
        return bool(np.any(self.log_odds[lig[dedans], c[dedans]] > SEUIL_OCCUPE))

    def probabilites_octets(self, c0: int, l0: int, c1: int, l1: int) -> np.ndarray:
        """Probabilités × 254 sur un rectangle ; 255 pour les cellules jamais observées."""
        bloc = self.log_odds[l0:l1, c0:c1]
        octets = np.round(254.0 / (1.0 + np.exp(-bloc))).astype(np.uint8)
        octets[~self.observee[l0:l1, c0:c1]] = INCONNU
        return octets

    def extraire_modifications(
        self, complet: bool = False
    ) -> tuple[tuple[int, int, int, int], str] | None:
        """Rectangle modifié depuis le dernier appel, compressé (zlib) et encodé en base64."""
        if complet:
            rect = [0, 0, self.largeur, self.hauteur]
        elif self._sale is None:
            return None
        else:
            rect = self._sale
        self._sale = None
        c0, l0, c1, l1 = rect
        donnees = zlib.compress(self.probabilites_octets(c0, l0, c1, l1).tobytes(), level=6)
        return (c0, l0, c1 - c0, l1 - l0), base64.b64encode(donnees).decode("ascii")
