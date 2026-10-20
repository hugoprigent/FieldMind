"""Suivi de trajectoire : poursuite pure sur l'essieu arrière et profil de vitesse.

La poursuite pure vise un point du chemin situé à une distance `Ld` devant l'essieu arrière
(le point dont la trajectoire suit le modèle bicyclette) : δ = atan(2 L sin α / Ld).

La vitesse est la plus faible des limites suivantes :
- vitesse maximale de la mission ;
- accélération latérale admissible dans les virages à venir (courbure du chemin) ;
- arrêt en douceur sur l'objectif ;
- distance libre mesurée devant le rover par le dernier balayage lidar ;
- écart de cap important (on ralentit pour rejoindre le chemin).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from services.protocole.fiche_rover import FicheRover


@dataclass(frozen=True, slots=True)
class ParametresSuivi:
    vitesse_max: float = 1.2
    vitesse_min: float = 0.25
    acceleration_laterale: float = 0.8
    deceleration_objectif: float = 0.6
    visee_min: float = 0.8
    visee_max: float = 1.4
    visee_par_vitesse: float = 0.45
    marge_frein_obstacle: float = 0.55
    """Distance libre (depuis le pare-chocs) sous laquelle on s'arrête, au-delà du firmware."""
    horizon_courbure: float = 2.5


@dataclass(frozen=True, slots=True)
class ConsigneSuivi:
    vitesse: float
    braquage: float
    indice: int
    """Indice du point du chemin le plus proche de l'essieu arrière."""
    distance_restante: float
    ecart_lateral: float
    angle_cible: float
    """Angle entre l'axe du rover et le point visé (rad) ; au-delà de 90°, la cible est derrière."""


class SuiviTrajectoire:
    def __init__(self, fiche: FicheRover, parametres: ParametresSuivi | None = None) -> None:
        self.f = fiche
        self.p = parametres or ParametresSuivi()
        self.points = np.zeros((0, 2))
        self._abscisse = np.zeros(0)
        self._courbure = np.zeros(0)
        self.indice = 0

    def definir_chemin(self, points: list[tuple[float, float]]) -> None:
        self.points = np.asarray(points, dtype=float).reshape(-1, 2)
        n = len(self.points)
        segments = np.linalg.norm(np.diff(self.points, axis=0), axis=1) if n > 1 else np.zeros(0)
        self._abscisse = np.concatenate([[0.0], np.cumsum(segments)])
        self._courbure = np.zeros(n)
        if n >= 3:
            # Courbure discrète (formule de Menger) sur des points distants de ~0,5 m.
            k = 2
            for i in range(k, n - k):
                a, b, c = self.points[i - k], self.points[i], self.points[i + k]
                ab, bc, ca = np.linalg.norm(b - a), np.linalg.norm(c - b), np.linalg.norm(a - c)
                aire2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
                if ab * bc * ca > 1e-9:
                    self._courbure[i] = 2 * aire2 / (ab * bc * ca)
        self.indice = 0

    @property
    def actif(self) -> bool:
        return len(self.points) >= 2

    def _projeter(self, x: float, y: float) -> tuple[int, float, float]:
        """Point du chemin le plus proche (recherche vers l'avant) : indice, abscisse, écart."""
        fin = min(len(self.points), self.indice + 60)
        debut = max(self.indice - 2, 0)
        candidats = self.points[debut:fin]
        distances = np.hypot(candidats[:, 0] - x, candidats[:, 1] - y)
        i = debut + int(np.argmin(distances))
        # Projection sur le segment [i, i+1] pour une abscisse continue.
        s, ecart = float(self._abscisse[i]), float(distances[i - debut])
        if i + 1 < len(self.points):
            a, b = self.points[i], self.points[i + 1]
            ab = b - a
            longueur2 = float(ab @ ab)
            if longueur2 > 1e-12:
                t = min(max(float((np.array([x, y]) - a) @ ab) / longueur2, 0.0), 1.0)
                s += t * math.sqrt(longueur2)
                projete = a + t * ab
                ecart = math.hypot(x - projete[0], y - projete[1])
        return i, s, ecart

    def _point_a(self, s: float) -> tuple[float, float]:
        s = min(max(s, 0.0), float(self._abscisse[-1]))
        return (
            float(np.interp(s, self._abscisse, self.points[:, 0])),
            float(np.interp(s, self._abscisse, self.points[:, 1])),
        )

    def consigne(
        self, x: float, y: float, cap: float, vitesse: float, distance_libre_avant: float
    ) -> ConsigneSuivi:
        """Consigne pour la pose (centre du rover) et la vitesse mesurées."""
        g, lim, p = self.f.geometrie, self.f.limites, self.p
        xr = x + g.essieu_arriere_x * math.cos(cap)
        yr = y + g.essieu_arriere_x * math.sin(cap)
        i, s, _ = self._projeter(xr, yr)
        self.indice = i
        # Écart latéral mesuré au centre du rover (le point qui doit suivre le chemin).
        _, _, ecart_centre = self._projeter(x, y)
        longueur_totale = float(self._abscisse[-1])
        # Distance restante mesurée depuis le centre du rover (l'objectif concerne le centre).
        restante = max(
            math.hypot(*(self.points[-1] - (x, y))), longueur_totale - s + g.essieu_arriere_x
        )

        visee = min(max(p.visee_min + p.visee_par_vitesse * abs(vitesse), p.visee_min), p.visee_max)
        cx, cy = self._point_a(s + visee)
        alpha = math.atan2(cy - yr, cx - xr) - cap
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        distance_visee = max(math.hypot(cx - xr, cy - yr), 0.3)
        if abs(alpha) > math.pi / 2:
            # Cible derrière : la loi de poursuite pure dégénère (sin α → 0 à 180°). On fait
            # demi-tour en braquant au maximum du côté de la cible.
            braquage = math.copysign(lim.braquage_max, alpha)
        else:
            braquage = math.atan2(2 * g.empattement * math.sin(alpha), distance_visee)
        braquage = min(max(braquage, -lim.braquage_max), lim.braquage_max)

        # Profil de vitesse.
        v = min(p.vitesse_max, lim.vitesse_max)
        devant = (self._abscisse >= s) & (self._abscisse <= s + p.horizon_courbure)
        courbure = float(self._courbure[devant].max()) if devant.any() else 0.0
        courbure = max(courbure, abs(math.tan(braquage)) / g.empattement * 0.5)
        if courbure > 1e-3:
            v = min(v, math.sqrt(p.acceleration_laterale / courbure))
        v = min(v, math.sqrt(2 * p.deceleration_objectif * max(restante, 0.0)) + 0.1)
        libre = max(distance_libre_avant - p.marge_frein_obstacle, 0.0)
        v = min(v, math.sqrt(2 * lim.deceleration_max * 0.5 * libre))
        if abs(alpha) > math.radians(50):
            v = min(v, p.vitesse_min)
        v = max(v, p.vitesse_min) if libre > 0.05 else 0.0
        return ConsigneSuivi(v, braquage, i, restante, ecart_centre, alpha)
