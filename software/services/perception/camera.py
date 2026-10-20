"""Géométrie de la caméra avant : passage d'un point de l'image à un point du sol, et inverse.

Hypothèse : sol plan et horizontal (celui de la zone de démonstration). Un point de l'image au
contact du sol (bas d'une boîte englobante) donne alors une position au sol ; son incertitude
croît avec le carré de la distance (une erreur d'un pixel près de l'horizon représente des
mètres). Au-dessus de l'horizon, pas de position.

Repère caméra : u vers la droite et v vers le bas de l'image (normalisés entre 0 et 1), axe
optique vers l'avant du rover, incliné de `inclinaison_deg` (négatif vers le sol).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from services.protocole.fiche_rover import Camera

ERREUR_V = 0.015
"""Erreur type sur la ligne de contact avec le sol, en fraction de la hauteur de l'image."""
INCERTITUDE_MAX = 5.0
"""Plafond de l'incertitude (m) : près de l'horizon, la position au sol n'a plus de sens."""


@dataclass(frozen=True, slots=True)
class PointSol:
    x: float
    y: float
    distance: float
    """Distance horizontale depuis la caméra (m)."""
    incertitude: float
    """Écart type de la position (m), dû à l'erreur sur la ligne de contact."""


class ModeleCamera:
    def __init__(self, camera: Camera) -> None:
        self.camera = camera
        self.f = camera.focale_px
        self.cx = camera.largeur / 2
        self.cy = camera.hauteur / 2
        plongee = -math.radians(camera.inclinaison_deg)
        self._s, self._c = math.sin(plongee), math.cos(plongee)

    def _rayon_rover(self, u: float, v: float) -> tuple[float, float, float]:
        """Direction (repère du rover : x avant, y gauche, z haut) du pixel normalisé (u, v)."""
        a = (u * self.camera.largeur - self.cx) / self.f
        b = (v * self.camera.hauteur - self.cy) / self.f
        s, c = self._s, self._c
        # avant optique (c, 0, -s) ; droite (0, -1, 0) ; bas de l'image (-s, 0, -c)
        return c - b * s, -a, -s - b * c

    def point_au_sol(self, u: float, v: float, x: float, y: float, cap: float) -> PointSol | None:
        """Point du sol vu au pixel (u, v) quand le rover est en (x, y, cap) ; None au-dessus
        de l'horizon ou trop loin."""
        sol = self._intersection(u, v, x, y, cap)
        if sol is None:
            return None
        plus_haut = self._intersection(u, v - ERREUR_V, x, y, cap)
        incertitude = INCERTITUDE_MAX if plus_haut is None else math.dist(sol[:2], plus_haut[:2])
        return PointSol(sol[0], sol[1], sol[2], min(incertitude, INCERTITUDE_MAX))

    def _intersection(
        self, u: float, v: float, x: float, y: float, cap: float
    ) -> tuple[float, float, float] | None:
        dx, dy, dz = self._rayon_rover(u, v)
        if dz > -1e-3:
            return None
        px, py, pz = self.camera.position
        k = pz / -dz
        fx, fy = px + k * dx, py + k * dy
        if math.hypot(k * dx, k * dy) > 60.0:
            return None
        c, s = math.cos(cap), math.sin(cap)
        return x + c * fx - s * fy, y + s * fx + c * fy, math.hypot(k * dx, k * dy)

    def ligne(self, avant: float, hauteur: float) -> float:
        """Ligne v de l'image où apparaît un point à `avant` mètres devant la caméra (repère du
        rover) et à `hauteur` du sol. Sur un capteur sans roulis, elle ne dépend pas du côté."""
        rz = hauteur - self.camera.position[2]
        profondeur = self._c * avant - self._s * rz
        bas = -self._s * avant - self._c * rz
        return (self.cy + self.f * bas / max(profondeur, 1e-6)) / self.camera.hauteur

    def avant_pour_ligne(self, v: float, hauteur: float, maximum: float = 60.0) -> float:
        """Distance devant la caméra d'un point à `hauteur` du sol vu à la ligne v (inverse de
        `ligne`, par dichotomie) ; bornée à [0,05 ; maximum]."""
        bas, haut = 0.05, maximum
        if v >= self.ligne(bas, hauteur):
            return bas
        if v <= self.ligne(haut, hauteur):
            return haut
        for _ in range(50):
            milieu = 0.5 * (bas + haut)
            if self.ligne(milieu, hauteur) > v:
                bas = milieu
            else:
                haut = milieu
        return 0.5 * (bas + haut)

    def projeter(
        self, px: float, py: float, pz: float, x: float, y: float, cap: float
    ) -> tuple[float, float, float] | None:
        """Point du monde → (u, v, profondeur) dans l'image ; None s'il est derrière la caméra."""
        c, s = math.cos(cap), math.sin(cap)
        ox, oy, oz = self.camera.position
        # Point dans le repère du rover, relatif à la caméra.
        rx = c * (px - x) + s * (py - y) - ox
        ry = -s * (px - x) + c * (py - y) - oy
        rz = pz - oz
        sp, cp = self._s, self._c
        profondeur = cp * rx - sp * rz
        if profondeur < 0.05:
            return None
        droite = -ry
        bas = -sp * rx - cp * rz
        u = (self.cx + self.f * droite / profondeur) / self.camera.largeur
        v = (self.cy + self.f * bas / profondeur) / self.camera.hauteur
        return u, v, profondeur
