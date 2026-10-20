"""Relief du terrain : ondulations douces, une butte à flanc raide et un fossé franchissable à gué.

Le relief n'existe que dans le simulateur (physique, capteurs) et dans l'affichage du poste
opérateur ; le cerveau le découvre avec sa caméra stéréo, comme le reste du monde.

- Ondulations : somme de quelques sinusoïdes de 18 à 40 m de longueur d'onde, tirées d'après la
  graine. Pente de quelques degrés au plus : le rover passe, et le plan du lidar (32 cm) ne touche
  pas le sol.
- Butte : plateau entouré d'un flanc à ~30°, trop raide pour le rover (seuil de 22°).
- Fossé : tranchée de 70 cm le long d'une polyligne, à bords raides ; une chaussée peu profonde
  le traverse au gué, là où passe la piste.

Grille régulière (pas de 25 cm) sur l'emprise du sol ; lignes du sud au nord, colonnes d'ouest en
est (ordre des données d'une « heightfield » MuJoCo).
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import numpy as np
from pydantic import BaseModel, ConfigDict, Field
from scipy import ndimage


class _Modele(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class ParamsButte(_Modele):
    x: float
    y: float
    rayon_sommet: float = Field(gt=0)
    rayon_base: float = Field(gt=0)
    hauteur: float = Field(gt=0)


class ParamsFosse(_Modele):
    points: list[tuple[float, float]] = Field(min_length=2)
    profondeur: float = Field(gt=0)
    demi_largeur: float = Field(gt=0)
    """Demi-largeur en haut des berges (m)."""
    gue: tuple[float, float]
    rayon_gue: float = Field(gt=0)
    """Demi-largeur de la chaussée du gué (m)."""
    profondeur_gue: float = Field(ge=0)


class ParamsRelief(_Modele):
    amplitude: float = Field(ge=0)
    """Amplitude des ondulations (m, crête à creux / 2)."""
    longueur_onde_min: float = Field(gt=0)
    longueur_onde_max: float = Field(gt=0)
    pas: float = Field(gt=0)
    buttes: list[ParamsButte] = Field(default_factory=list)
    fosses: list[ParamsFosse] = Field(default_factory=list)


@dataclass(frozen=True)
class Relief:
    """Hauteurs du sol sur une grille régulière, interpolées linéairement entre les nœuds."""

    origine: tuple[float, float]
    """Coin sud-ouest de la grille (premier nœud)."""
    pas: float
    hauteurs: np.ndarray
    """(lignes, colonnes) : lignes du sud au nord, colonnes d'ouest en est."""
    params: ParamsRelief

    @property
    def forme(self) -> tuple[int, int]:
        n, m = self.hauteurs.shape
        return int(n), int(m)

    def hauteur(self, x: np.ndarray | float, y: np.ndarray | float) -> np.ndarray:
        """Hauteur du sol en (x, y), bilinéaire ; bornée aux bords de la grille."""
        n, m = self.forme
        fx = np.clip((np.asarray(x, dtype=float) - self.origine[0]) / self.pas, 0, m - 1.000001)
        fy = np.clip((np.asarray(y, dtype=float) - self.origine[1]) / self.pas, 0, n - 1.000001)
        c, lig = np.floor(fx).astype(int), np.floor(fy).astype(int)
        tx, ty = fx - c, fy - lig
        h = self.hauteurs
        bas = h[lig, c] * (1 - tx) + h[lig, c + 1] * tx
        haut = h[lig + 1, c] * (1 - tx) + h[lig + 1, c + 1] * tx
        resultat: np.ndarray = bas * (1 - ty) + haut * ty
        return resultat

    def h(self, x: float, y: float) -> float:
        return float(self.hauteur(x, y))

    def pente_deg(self, x: np.ndarray | float, y: np.ndarray | float) -> np.ndarray:
        e = self.pas
        gx = (self.hauteur(np.asarray(x) + e, y) - self.hauteur(np.asarray(x) - e, y)) / (2 * e)
        gy = (self.hauteur(x, np.asarray(y) + e) - self.hauteur(x, np.asarray(y) - e)) / (2 * e)
        pente: np.ndarray = np.degrees(np.arctan(np.hypot(gx, gy)))
        return pente

    def distance_fosse(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Distance (m) à l'axe du fossé le plus proche ; `inf` sans fossé."""
        points = np.column_stack([np.ravel(x), np.ravel(y)])
        d = np.full(len(points), np.inf)
        for fosse in self.params.fosses:
            d = np.minimum(d, _distance_polyligne(points, np.asarray(fosse.points, dtype=float)))
        return d.reshape(np.shape(x))

    def dans_butte(self, x: np.ndarray, y: np.ndarray, marge: float = 0.0) -> np.ndarray:
        masque = np.zeros(np.shape(x), dtype=bool)
        for b in self.params.buttes:
            masque |= np.hypot(np.asarray(x) - b.x, np.asarray(y) - b.y) < b.rayon_base + marge
        return masque


def _distance_polyligne(points: np.ndarray, ligne: np.ndarray) -> np.ndarray:
    """Distance de chaque point (N, 2) à la polyligne (M, 2), par segments (mémoire bornée)."""
    d = np.full(len(points), np.inf)
    for a, b in itertools.pairwise(ligne):
        ab = b - a
        t = np.clip(((points - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
        d = np.minimum(d, np.linalg.norm(points - (a + t[:, None] * ab), axis=1))
    return d


def generer_relief(
    params: ParamsRelief, centre: tuple[float, float], demi: float, graine: int
) -> Relief:
    """Relief sur le carré de centre `centre` et de demi-côté `demi`."""
    rng = np.random.default_rng(graine + 404)
    n = math.ceil(2 * demi / params.pas) + 1
    x0, y0 = centre[0] - demi, centre[1] - demi
    xs = x0 + params.pas * np.arange(n)
    ys = y0 + params.pas * np.arange(n)
    x, y = np.meshgrid(xs, ys)  # lignes : y croissant (sud → nord)

    # Ondulations : sinusoïdes de directions et phases aléatoires.
    h = np.zeros_like(x)
    composantes = 6
    for _ in range(composantes):
        lam = rng.uniform(params.longueur_onde_min, params.longueur_onde_max)
        a = rng.uniform(0, 2 * math.pi)
        phase = rng.uniform(0, 2 * math.pi)
        h += np.sin(2 * math.pi * (x * math.cos(a) + y * math.sin(a)) / lam + phase)
    h *= params.amplitude / math.sqrt(composantes / 2)

    for b in params.buttes:
        d = np.hypot(x - b.x, y - b.y)
        flanc = np.clip((b.rayon_base - d) / (b.rayon_base - b.rayon_sommet), 0.0, 1.0)
        # Profil arrondi en haut et en bas, raide au milieu.
        h += b.hauteur * (3 * flanc**2 - 2 * flanc**3)

    points = np.column_stack([x.ravel(), y.ravel()])
    for f in params.fosses:
        d = _distance_polyligne(points, np.asarray(f.points, dtype=float)).reshape(x.shape)
        d_gue = np.hypot(x - f.gue[0], y - f.gue[1])
        # Gué : une chaussée peu profonde à travers la tranchée, bordée de talus raides ; on ne
        # descend pas du gué au fond du fossé.
        v = np.clip((d_gue - f.rayon_gue) / 0.6, 0.0, 1.0)
        plein = 3 * v**2 - 2 * v**3
        profondeur = f.profondeur_gue + (f.profondeur - f.profondeur_gue) * plein
        u = np.clip(1 - d / f.demi_largeur, 0.0, 1.0)
        h -= profondeur * (3 * u**2 - 2 * u**3)

    # Léger lissage : pas de facette plus raide que ce que la grille peut représenter.
    h = ndimage.uniform_filter(h, size=3, mode="nearest")
    return Relief((x0, y0), params.pas, h.astype(np.float64), params)
