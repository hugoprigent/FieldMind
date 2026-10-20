"""Modèle des caméras : passage d'un pixel à un point du sol (base_link) et inverse.

Modèle sténopé. Avec la calibration stéréo (`cameras.calibration`, fichier npz produit par
`calibration_stereo.py`), on prend les focales, le centre optique et la distorsion mesurés, et la
position relative des deux caméras. Sans elle, champ horizontal par défaut (`cameras.hfov_deg`),
centre optique au milieu de l'image, sans distorsion, caméras parallèles séparées de `base_m`.

La pose de montage (hauteur, avance, inclinaison vers le sol) vient du profil dans les deux cas :
la calibration de la paire ne dit rien de sa position sur le rover.

Conventions :
- (u, v) normalisés : u = 0 bord gauche de l'image, 1 bord droit ; v = 0 haut, 1 bas. Le centre
  du pixel de colonne i est en u = (i + 0,5) / largeur (centre de pixel entier d'OpenCV).
- Repère caméra d'OpenCV : x à droite, y en bas, z le long de l'axe optique.
- Roulis et tangage du rover (centrale) : rotation de base_link par rapport à l'horizontale,
  R = Ry(tangage) · Rx(roulis) ; tangage > 0 nez vers le bas (REP 103), roulis > 0 côté gauche
  levé. Les points au sol rendus sont alors dans le repère horizontal attaché au rover (même
  origine et même cap que base_link, z vertical), ce qui sert à la carte.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

import numpy as np

from fieldmind.config import Profil

PORTEE_SOL_MAX_M = 60.0
"""Au-delà, un point « au sol » vu près de l'horizon n'a plus de sens."""


def lire_calibration(chemin: str | os.PathLike[str] | None) -> dict[str, Any] | None:
    """Calibration stéréo enregistrée (npz) ; None si absente ou illisible."""
    if not chemin:
        return None
    fichier = Path(chemin).expanduser()
    if not fichier.is_file():
        return None
    try:
        with np.load(fichier, allow_pickle=False) as donnees:
            calibration = {cle: np.array(donnees[cle]) for cle in donnees.files}
    except (OSError, ValueError):
        return None
    for cle in ("k1", "d1", "k2", "d2", "r", "t", "taille"):
        if cle not in calibration:
            return None
    return calibration


def rotation_montage(inclinaison: float, roulis: float = 0.0, lacet: float = 0.0) -> np.ndarray:
    """Matrice caméra (OpenCV) → base_link d'une caméra regardant vers l'avant, inclinée de
    `inclinaison` vers le sol (rad), puis tournée de `roulis` autour de son axe optique et de
    `lacet` autour de la verticale (défauts de montage)."""
    c, s = math.cos(inclinaison), math.sin(inclinaison)
    # Colonnes : x caméra (droite), y caméra (bas), z caméra (axe optique), dans base_link.
    base = np.array([[0.0, -s, c],
                     [-1.0, 0.0, 0.0],
                     [0.0, -c, -s]])
    cr, sr = math.cos(roulis), math.sin(roulis)
    autour_axe = np.array([[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]])
    cl, sl = math.cos(lacet), math.sin(lacet)
    vertical = np.array([[cl, -sl, 0.0], [sl, cl, 0.0], [0.0, 0.0, 1.0]])
    rotation: np.ndarray = vertical @ base @ autour_axe
    return rotation


def rotation_assiette(roulis: float, tangage: float) -> np.ndarray:
    """base_link → repère horizontal : Ry(tangage) · Rx(roulis)."""
    cr, sr = math.cos(roulis), math.sin(roulis)
    cp, sp = math.cos(tangage), math.sin(tangage)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    rotation: np.ndarray = ry @ rx
    return rotation


class ModeleCamera:
    """Géométrie d'une caméra de la paire (« gauche » ou « droite »).

    `calibration` : True pour lire le fichier du profil, False pour l'ignorer (modèle par défaut,
    celui du monde simulé), ou un chemin / un dict de calibration.
    """

    def __init__(self, profil: Profil, camera: str = "gauche",
                 calibration: bool | str | os.PathLike[str] | dict[str, Any] = True) -> None:
        if camera not in ("gauche", "droite"):
            raise ValueError(f"caméra inconnue : {camera!r}")
        self.camera = camera
        self.largeur = int(profil.valeur("cameras.largeur", 1280))
        self.hauteur = int(profil.valeur("cameras.hauteur", 720))
        hfov = math.radians(float(profil.valeur("cameras.hfov_deg", 78.0)))
        base = float(profil.valeur("cameras.base_m", 0.12))
        milieu = np.array([float(profil.valeur("cameras.x_m", 0.415)),
                           float(profil.valeur("cameras.y_m", 0.0)),
                           float(profil.valeur("cameras.z_m", 0.265))])
        self.inclinaison = math.radians(float(profil.valeur("cameras.inclinaison_deg", 10.0)))
        r_montage = rotation_montage(
            self.inclinaison,
            math.radians(float(profil.valeur("cameras.roulis_deg", 0.0))),
            math.radians(float(profil.valeur("cameras.lacet_deg", 0.0))))

        donnees: dict[str, Any] | None
        if isinstance(calibration, dict):
            donnees = calibration
        elif calibration is True:
            donnees = lire_calibration(profil.valeur("cameras.calibration", ""))
        elif calibration is False:
            donnees = None
        else:
            donnees = lire_calibration(calibration)
        self.calibration = donnees

        if donnees is None:
            f = (self.largeur / 2) / math.tan(hfov / 2)
            self._k = np.array([[f, 0.0, self.largeur / 2 - 0.5],
                                [0.0, f, self.hauteur / 2 - 0.5],
                                [0.0, 0.0, 1.0]])
            self.distorsion: np.ndarray | None = None
            decalage = np.array([0.0, base / 2, 0.0])
            self.base = base
            self.rotation = r_montage
            self.centre = milieu + (decalage if camera == "gauche" else -decalage)
        else:
            w_c, h_c = (float(v) for v in np.asarray(donnees["taille"]).reshape(-1)[:2])
            k = np.asarray(donnees["k1" if camera == "gauche" else "k2"], dtype=np.float64)
            self._k = _mettre_a_l_echelle(k, self.largeur / w_c, self.hauteur / h_c)
            d = np.asarray(donnees["d1" if camera == "gauche" else "d2"], dtype=np.float64)
            self.distorsion = d.reshape(-1) if np.any(np.abs(d) > 0) else None
            r = np.asarray(donnees["r"], dtype=np.float64)
            t = np.asarray(donnees["t"], dtype=np.float64).reshape(3)
            # Centre de la caméra droite dans le repère de la gauche : −Rᵀ T.
            droite_dans_gauche = r_montage @ (-r.T @ t)
            self.base = float(np.linalg.norm(t))
            if camera == "gauche":
                self.rotation = r_montage
                self.centre = milieu - 0.5 * droite_dans_gauche
            else:
                self.rotation = r_montage @ r.T
                self.centre = milieu + 0.5 * droite_dans_gauche

    # ------------------------------------------------------------------ intrinsèques
    @property
    def calibree(self) -> bool:
        return self.calibration is not None

    def matrice(self, largeur: int | None = None, hauteur: int | None = None) -> np.ndarray:
        """Matrice intrinsèque K pour une image de cette taille (défaut : taille du profil)."""
        largeur = self.largeur if largeur is None else largeur
        hauteur = self.hauteur if hauteur is None else hauteur
        return _mettre_a_l_echelle(self._k, largeur / self.largeur, hauteur / self.hauteur)

    @property
    def focale_px(self) -> float:
        return float(self._k[0, 0])

    # ------------------------------------------------------------------ rayons
    def directions_camera(self, u: np.ndarray, v: np.ndarray) -> np.ndarray:
        """(N, 3) directions dans le repère caméra (z = 1) des pixels normalisés (u, v)."""
        u = np.asarray(u, dtype=np.float64).reshape(-1)
        v = np.asarray(v, dtype=np.float64).reshape(-1)
        px = u * self.largeur - 0.5
        py = v * self.hauteur - 0.5
        if self.distorsion is None:
            xn = (px - self._k[0, 2]) / self._k[0, 0]
            yn = (py - self._k[1, 2]) / self._k[1, 1]
        else:
            import cv2

            brut = np.column_stack((px, py)).reshape(-1, 1, 2)
            normalises = cv2.undistortPoints(brut, self._k, self.distorsion).reshape(-1, 2)
            xn, yn = normalises[:, 0], normalises[:, 1]
        return np.column_stack((xn, yn, np.ones_like(xn)))

    def rayons(self, u: np.ndarray, v: np.ndarray, roulis: float = 0.0,
               tangage: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
        """Origine (3,) et directions (N, 3) des rayons, dans le repère horizontal du rover."""
        assiette = rotation_assiette(roulis, tangage)
        directions = self.directions_camera(u, v) @ (assiette @ self.rotation).T
        return assiette @ self.centre, directions

    def pixel_vers_sol(self, u: Any, v: Any, roulis: float = 0.0, tangage: float = 0.0) -> Any:
        """Point du sol (x, y) vu au pixel (u, v), sol plan à z = 0.

        Scalaires : (x, y) ou None au-dessus de l'horizon (ou au-delà de 60 m). Tableaux :
        (N, 2), NaN là où il n'y a pas de sol.
        """
        scalaire = np.ndim(u) == 0 and np.ndim(v) == 0
        origine, directions = self.rayons(np.atleast_1d(u), np.atleast_1d(v), roulis, tangage)
        dz = directions[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            s = np.where(dz < -1e-9, -origine[2] / dz, np.nan)
        points = origine[:2] + s[:, None] * directions[:, :2]
        loin = np.hypot(points[:, 0] - origine[0], points[:, 1] - origine[1]) > PORTEE_SOL_MAX_M
        points[loin] = np.nan
        if scalaire:
            if not np.isfinite(points[0, 0]):
                return None
            return float(points[0, 0]), float(points[0, 1])
        return points

    # ------------------------------------------------------------------ projection
    def projeter(self, points: np.ndarray, roulis: float = 0.0, tangage: float = 0.0
                 ) -> tuple[np.ndarray, np.ndarray]:
        """Points (N, 3) du repère horizontal du rover → (u, v) normalisés (N, 2) et profondeur
        le long de l'axe optique (N,). Profondeur ≤ 0 : point derrière la caméra (u, v NaN)."""
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        assiette = rotation_assiette(roulis, tangage)
        r_cam = assiette @ self.rotation  # caméra → horizontal
        relatifs = (points - assiette @ self.centre) @ r_cam  # = r_camᵀ (p − c)
        profondeur = relatifs[:, 2]
        uv = np.full((len(points), 2), np.nan)
        devant = profondeur > 1e-6
        if np.any(devant):
            if self.distorsion is None:
                px = self._k[0, 0] * relatifs[devant, 0] / profondeur[devant] + self._k[0, 2]
                py = self._k[1, 1] * relatifs[devant, 1] / profondeur[devant] + self._k[1, 2]
            else:
                import cv2

                zero = np.zeros(3)
                projetes, _ = cv2.projectPoints(relatifs[devant].reshape(-1, 1, 3), zero, zero,
                                                self._k, self.distorsion)
                px, py = projetes[:, 0, 0], projetes[:, 0, 1]
            uv[devant, 0] = (px + 0.5) / self.largeur
            uv[devant, 1] = (py + 0.5) / self.hauteur
        return uv, profondeur

    def sol_vers_pixel(self, x: Any, y: Any, z: Any = 0.0, roulis: float = 0.0,
                       tangage: float = 0.0) -> Any:
        """Point (x, y, z) du repère horizontal du rover (sol par défaut) → (u, v) normalisés.

        Scalaires : (u, v) ou None si le point est derrière la caméra (u, v peuvent sortir de
        0..1 : point hors champ). Tableaux : (N, 2) avec NaN derrière la caméra.
        """
        scalaire = np.ndim(x) == 0 and np.ndim(y) == 0
        xs, ys = np.atleast_1d(np.asarray(x, dtype=np.float64)), np.atleast_1d(
            np.asarray(y, dtype=np.float64))
        zs = np.broadcast_to(np.asarray(z, dtype=np.float64), xs.shape)
        uv, _ = self.projeter(np.column_stack((xs, ys, zs)), roulis, tangage)
        if scalaire:
            if not np.isfinite(uv[0, 0]):
                return None
            return float(uv[0, 0]), float(uv[0, 1])
        return uv

    def dans_image(self, u: Any, v: Any) -> Any:
        return (np.asarray(u) >= 0) & (np.asarray(u) <= 1) & (np.asarray(v) >= 0) & (
            np.asarray(v) <= 1)


def _mettre_a_l_echelle(k: np.ndarray, sx: float, sy: float) -> np.ndarray:
    """K d'une image redimensionnée (centres de pixel entiers d'OpenCV)."""
    sortie = np.array(k, dtype=np.float64, copy=True)
    sortie[0, 0] *= sx
    sortie[0, 1] *= sx
    sortie[1, 1] *= sy
    sortie[0, 2] = (k[0, 2] + 0.5) * sx - 0.5
    sortie[1, 2] = (k[1, 2] + 0.5) * sy - 0.5
    return sortie
