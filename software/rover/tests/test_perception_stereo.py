"""Stéréo sur des paires rendues par le monde simulé : obstacles bas à la bonne place."""

from __future__ import annotations

import math
import tomllib

import cv2
import numpy as np
import pytest
from budget import dans_le_budget

from fieldmind.config import FICHIER_PROFIL, Profil
from fieldmind.perception.projection import ModeleCamera
from fieldmind.perception.stereo import ResultatStereo, Stereo
from fieldmind.simulation.monde import Boite, Cylindre, Monde, Obstacle
from fieldmind.types import ImageCamera, Pose2D


def profil_test(calibration: str = "/inexistant/stereo.npz") -> Profil:
    with open(FICHIER_PROFIL, "rb") as f:
        base = tomllib.load(f)
    base["cameras"]["calibration"] = calibration
    return Profil(base, {}, None)


def paire(monde: Monde, pose: Pose2D, profil: Profil, t: float,
          largeur: int = 640, hauteur: int = 360) -> tuple[ImageCamera, ImageCamera]:
    images = []
    for camera in ("gauche", "droite"):
        bgr = monde.rendre_camera(pose, camera, largeur, hauteur, profil)
        jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()
        images.append(ImageCamera(t, camera, jpeg, largeur, hauteur))
    return images[0], images[1]


def distance_obstacle(o: Obstacle, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Distance (m) de points du monde à l'emprise au sol d'un obstacle."""
    if isinstance(o, Cylindre):
        return np.maximum(np.hypot(x - o.x, y - o.y) - o.rayon, 0.0)
    assert isinstance(o, Boite)
    c, s = math.cos(o.cap), math.sin(o.cap)
    lx = c * (x - o.x) + s * (y - o.y)
    ly = -s * (x - o.x) + c * (y - o.y)
    return np.hypot(np.maximum(np.abs(lx) - o.longueur / 2, 0),
                    np.maximum(np.abs(ly) - o.largeur / 2, 0))


def vers_monde(pose: Pose2D, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    c, s = math.cos(pose.cap), math.sin(pose.cap)
    p = points.astype(np.float64)
    return pose.x + c * p[:, 0] - s * p[:, 1], pose.y + s * p[:, 0] + c * p[:, 1]


def ecart_au_monde(monde: Monde, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    ecart = np.full(len(x), np.inf)
    for o in monde.obstacles:
        ecart = np.minimum(ecart, distance_obstacle(o, x, y))
    if monde.piece is not None:
        p = monde.piece
        ecart = np.minimum.reduce([ecart, np.abs(x - p.xmin), np.abs(x - p.xmax),
                                   np.abs(y - p.ymin), np.abs(y - p.ymax)])
    return ecart


def traiter_plusieurs(stereo: Stereo, monde: Monde, pose: Pose2D, profil: Profil,
                      n: int = 2) -> list[ResultatStereo]:
    resultats = []
    for k in range(n):
        r = stereo.traiter(*paire(monde, pose, profil, 0.25 * k))
        assert r is not None
        resultats.append(r)
    return resultats


@pytest.mark.parametrize(("nom", "pose"), [
    ("marche", Pose2D(5.6, 2.4, 0.0)),  # face avant à 2,35 m de l'essieu (1,94 m des caméras)
    ("pot", Pose2D(4.2, 1.0, 0.0)),  # 2,04 m
    ("poutre", Pose2D(1.2, 1.0, 0.6)),  # en biais, ≈ 2 m
    ("carton_bas", Pose2D(2.6, 2.3, 0.35)),
])
def test_obstacle_bas_a_la_bonne_place(nom: str, pose: Pose2D) -> None:
    profil = profil_test()
    monde = Monde(0, "salle")
    stereo = Stereo(profil)
    assert not stereo.calibree
    premiere, seconde = traiter_plusieurs(stereo, monde, pose, profil)
    assert len(premiere.obstacles.points) == 0  # rien n'est confirmé sur une seule image
    assert premiere.sol_estime and not premiere.calibree
    obstacles = seconde.obstacles
    assert obstacles.source == "stereo" and obstacles.hauteurs is not None
    cible = next(o for o in monde.obstacles if o.nom == nom)
    assert cible.haut < 0.322  # invisible pour le lidar
    x, y = vers_monde(pose, obstacles.points)
    sur_cible = distance_obstacle(cible, x, y) <= 0.10
    assert sur_cible.sum() >= 5, f"{nom} non trouvé"
    # Hauteur mesurée : celle de l'objet, à quelques centimètres près.
    assert obstacles.hauteurs[sur_cible].max() == pytest.approx(cible.haut, abs=0.05)
    # Tout ce qui est vu à moins de 2,5 m des caméras est à moins de 10 cm d'un vrai obstacle.
    proches = np.hypot(obstacles.points[:, 0] - 0.415, obstacles.points[:, 1]) < 2.5
    assert np.all(ecart_au_monde(monde, x[proches], y[proches]) <= 0.10)
    # Au-delà (jusqu'à 4 m), l'erreur de profondeur croît comme Z² : 20 cm tolérés.
    assert np.all(ecart_au_monde(monde, x, y) <= 0.20)
    # Plan du sol retrouvé (sol plat à z = 0).
    a, b, c = seconde.sol
    assert abs(c) < 0.02 and math.hypot(a, b) < 0.02


def test_sol_plat_sans_faux_obstacle_et_temps() -> None:
    profil = profil_test()
    stereo = Stereo(profil)
    resultats = traiter_plusieurs(stereo, Monde(0, "champ"), Pose2D(), profil, n=4)
    for r in resultats:
        assert len(r.obstacles.points) == 0
        assert r.couverture > 0.5
        assert r.profondeur_jpeg[:2] == b"\xff\xd8"
    durees = [r.duree_s for r in resultats[1:]]
    print(f"\nstéréo 640 × 360 : {1000 * np.median(durees):.1f} ms par paire (médiane)")
    assert dans_le_budget(float(np.median(durees)), 0.06)


def test_point_3d() -> None:
    profil = profil_test()
    monde = Monde(0, "salle")
    stereo = Stereo(profil)
    pose = Pose2D(5.6, 2.4, 0.0)
    assert stereo.point_3d("gauche", 0.5, 0.5) is None  # pas encore de profondeur
    traiter_plusieurs(stereo, monde, pose, profil, n=1)
    # Milieu de la face avant de la marche : x = 7,95 − 5,6 = 2,35 m, à mi-hauteur.
    for camera in ("gauche", "droite"):
        u, v = ModeleCamera(profil, camera).sol_vers_pixel(2.35, 0.0, 0.08)
        point = stereo.point_3d(camera, u, v)
        assert point is not None
        assert point == pytest.approx((2.35, 0.0, 0.08), abs=0.05)
    # Sol devant le rover.
    u, v = ModeleCamera(profil, "gauche").sol_vers_pixel(1.5, 0.2)
    assert stereo.point_3d("gauche", u, v) == pytest.approx((1.5, 0.2, 0.0), abs=0.04)


def test_confirmation_avec_pose_odom() -> None:
    """Le rover avance de 15 cm entre deux images : confirmé dans le repère odom."""
    profil = profil_test()
    monde = Monde(0, "salle")
    stereo = Stereo(profil)
    pose1, pose2 = Pose2D(5.45, 2.4, 0.0), Pose2D(5.6, 2.4, 0.0)
    stereo.traiter(*paire(monde, pose1, profil, 0.0), pose=pose1)
    r = stereo.traiter(*paire(monde, pose2, profil, 0.25), pose=pose2)
    assert r is not None
    x, y = vers_monde(pose2, r.obstacles.points)
    marche = next(o for o in monde.obstacles if o.nom == "marche")
    assert (distance_obstacle(marche, x, y) <= 0.10).sum() >= 5
