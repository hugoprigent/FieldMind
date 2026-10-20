"""Modèle des caméras : pixel ↔ sol."""

from __future__ import annotations

import math
import tomllib

import numpy as np
import pytest

from fieldmind.config import FICHIER_PROFIL, Profil
from fieldmind.perception.projection import ModeleCamera


@pytest.fixture(scope="module")
def profil() -> Profil:
    with open(FICHIER_PROFIL, "rb") as f:
        base = tomllib.load(f)
    base["cameras"]["calibration"] = "/inexistant/stereo.npz"
    return Profil(base, {}, None)


def test_centre_image_au_sol(profil: Profil) -> None:
    """Axe optique incliné de 10° depuis 0,265 m : il touche le sol 1,503 m devant la caméra."""
    gauche = ModeleCamera(profil, "gauche")
    assert not gauche.calibree
    x, y = gauche.pixel_vers_sol(0.5, 0.5)
    assert x == pytest.approx(0.415 + 0.265 / math.tan(math.radians(10)), abs=1e-3)
    assert y == pytest.approx(0.06, abs=1e-6)
    x_d, y_d = ModeleCamera(profil, "droite").pixel_vers_sol(0.5, 0.5)
    assert x_d == pytest.approx(x) and y_d == pytest.approx(-0.06)
    # Au-dessus de l'horizon (10° au-dessus de l'axe, v ≈ 0,307) : pas de sol.
    assert gauche.pixel_vers_sol(0.5, 0.25) is None
    assert gauche.pixel_vers_sol(0.5, 0.35) is not None
    # Côté gauche de l'image → sol à gauche du rover (y > 0).
    assert gauche.pixel_vers_sol(0.1, 0.8)[1] > 0.06


def test_aller_retour(profil: Profil) -> None:
    modele = ModeleCamera(profil, "gauche")
    u, v = np.meshgrid(np.linspace(0.02, 0.98, 9), np.linspace(0.4, 0.98, 7))
    sol = modele.pixel_vers_sol(u.ravel(), v.ravel())
    assert sol.shape == (63, 2) and np.all(np.isfinite(sol))
    uv = modele.sol_vers_pixel(sol[:, 0], sol[:, 1])
    assert np.allclose(uv[:, 0], u.ravel(), atol=1e-9)
    assert np.allclose(uv[:, 1], v.ravel(), atol=1e-9)
    # Tableaux avec points au-dessus de l'horizon : NaN.
    melange = modele.pixel_vers_sol(np.array([0.5, 0.5]), np.array([0.1, 0.9]))
    assert np.isnan(melange[0, 0]) and np.isfinite(melange[1, 0])
    # Point derrière la caméra : None.
    assert modele.sol_vers_pixel(-1.0, 0.0) is None


def test_assiette(profil: Profil) -> None:
    """Nez vers le bas (tangage > 0) : l'axe optique touche le sol plus près."""
    modele = ModeleCamera(profil, "gauche")
    x0, _ = modele.pixel_vers_sol(0.5, 0.5)
    x1, _ = modele.pixel_vers_sol(0.5, 0.5, tangage=math.radians(5))
    x2, _ = modele.pixel_vers_sol(0.5, 0.5, tangage=math.radians(-3))
    assert x1 < x0 < x2
    # Roulis et tangage : aller-retour cohérent.
    u, v = modele.sol_vers_pixel(2.0, 0.3, roulis=0.05, tangage=0.03)
    x4, y4 = modele.pixel_vers_sol(u, v, roulis=0.05, tangage=0.03)
    assert (x4, y4) == pytest.approx((2.0, 0.3), abs=1e-6)


def test_calibration_ideale_equivaut_au_modele_par_defaut(profil: Profil) -> None:
    defaut = ModeleCamera(profil, "droite", calibration=False)
    k = defaut.matrice(1280, 720)
    calibration = {"k1": k, "d1": np.zeros(5), "k2": k, "d2": np.zeros(5), "r": np.eye(3),
                   "t": np.array([-0.12, 0.0, 0.0]), "taille": np.array([1280, 720])}
    calibre = ModeleCamera(profil, "droite", calibration=calibration)
    assert calibre.calibree and calibre.base == pytest.approx(0.12)
    assert np.allclose(calibre.centre, defaut.centre)
    for u, v in ((0.5, 0.5), (0.1, 0.9), (0.8, 0.6)):
        assert calibre.pixel_vers_sol(u, v) == pytest.approx(defaut.pixel_vers_sol(u, v))
    # Matrice à l'échelle d'une image réduite (centres de pixel entiers d'OpenCV).
    k640 = defaut.matrice(640, 360)
    assert k640[0, 0] == pytest.approx(k[0, 0] / 2)
    assert k640[0, 2] == pytest.approx(319.5)
