"""Monde simulé : scénarios, lidar, rendu des caméras, collisions."""

from __future__ import annotations

import json
import math
import time
import tomllib

import cv2
import numpy as np
import pytest
from budget import dans_le_budget

from fieldmind.config import FICHIER_PROFIL, Profil
from fieldmind.perception.projection import ModeleCamera
from fieldmind.simulation.monde import Monde, deplacer
from fieldmind.types import Pose2D


@pytest.fixture(scope="module")
def profil() -> Profil:
    with open(FICHIER_PROFIL, "rb") as f:
        base = tomllib.load(f)
    base["cameras"]["calibration"] = "/inexistant/stereo.npz"
    return Profil(base, {}, None)


@pytest.mark.parametrize("scenario", ["salle", "jardin", "couloir", "champ"])
def test_scenarios(scenario: str, profil: Profil) -> None:
    monde = Monde(graine=0, scenario=scenario)
    description = monde.description()
    json.dumps(description)  # sérialisable pour le poste
    assert description["scenario"] == scenario
    assert isinstance(monde.depart, Pose2D)
    assert not monde.collision(monde.depart, profil)
    if scenario in ("salle", "jardin"):
        bas = monde.obstacles_bas(0.322)
        assert len(bas) >= 4  # marches, poutres, pots, rochers… sous le plan du lidar
        assert all(o.haut < 0.322 for o in bas)
    with pytest.raises(ValueError, match="scénario inconnu"):
        Monde(0, "lune")


def test_graine_fait_varier_le_monde() -> None:
    a, b = Monde(0, "salle"), Monde(3, "salle")
    assert a.obstacles[0].x != b.obstacles[0].x
    assert Monde(3, "salle").obstacles[0].x == b.obstacles[0].x


def test_lidar_distances_et_obstacles_bas_invisibles(profil: Profil) -> None:
    monde = Monde(0, "salle")
    rng = np.random.default_rng(0)
    # Face au mur sud (y = 0), à 3 m.
    angles, distances = monde.lancer_lidar(Pose2D(5.0, 3.0, -math.pi / 2), profil, rng=rng)
    assert angles.dtype == np.float32 and distances.dtype == np.float32
    assert np.all(np.diff(angles) >= 0)
    assert 350 <= len(angles) <= 400
    devant = np.abs(angles) < 0.02
    assert np.median(distances[devant]) == pytest.approx(3.0, abs=0.02)
    # La marche (0,16 m de haut) est sous le plan laser : le faisceau va jusqu'au mur est.
    angles, distances = monde.lancer_lidar(Pose2D(5.0, 2.4, 0.0), profil, rng=rng)
    devant = np.abs(angles) < 0.02
    assert np.median(distances[devant]) == pytest.approx(5.0, abs=0.03)
    # Champ ouvert : rien à moins de 12 m.
    angles, _ = Monde(0, "champ").lancer_lidar(Pose2D(), profil)
    assert len(angles) == 0


def test_lidar_tour_deforme_par_le_mouvement(profil: Profil) -> None:
    """Avec la pose de début de tour, chaque faisceau part de la pose de son instant : le C1
    tourne en sens horaire depuis l'avant, la droite (−90°) est mesurée au quart du tour, la
    gauche (+90°) aux trois quarts. Le rover glisse ici de 10 cm vers la gauche pendant le tour."""
    monde = Monde(0, "salle")
    debut, fin = Pose2D(5.0, 2.9, 0.0), Pose2D(5.0, 3.0, 0.0)
    angles, distances = monde.lancer_lidar(fin, profil, pose_debut=debut,
                                           rng=np.random.default_rng(1))
    droite = np.abs(angles + math.pi / 2) < 0.03
    gauche = np.abs(angles - math.pi / 2) < 0.03
    assert np.median(distances[droite]) == pytest.approx(2.925, abs=0.012)  # mur sud
    assert np.median(distances[gauche]) == pytest.approx(4.025, abs=0.012)  # mur nord


def test_rendu_camera(profil: Profil) -> None:
    monde = Monde(0, "salle")
    pose = monde.depart
    gauche = monde.rendre_camera(pose, "gauche", 640, 360, profil)
    droite = monde.rendre_camera(pose, "droite", 640, 360, profil)
    assert gauche.shape == (360, 640, 3) and gauche.dtype == np.uint8
    assert not np.array_equal(gauche, droite)
    assert gauche.std() > 15  # texturé
    durees = []
    for _ in range(5):
        debut = time.perf_counter()
        monde.rendre_camera(pose, "gauche", 640, 360, profil)
        durees.append(time.perf_counter() - debut)
    print(f"\nrendu 640 × 360 (salle) : {1000 * min(durees):.1f} ms (min), "
          f"{1000 * np.median(durees):.1f} ms (médiane)")
    assert dans_le_budget(min(durees), 0.06)  # cible 40 ms


def test_rendu_perspective_exacte(profil: Profil) -> None:
    """Les coins d'une mire rendue tombent là où le modèle de projection.py les place."""
    monde = Monde(0, "champ")
    mire = monde.placer_mire((1.2, 0.05, 0.12), 0.1, 0.2, -0.15)
    image = monde.rendre_camera(Pose2D(), "gauche", 1280, 720, profil, surechantillonnage=2,
                                bruit=0.0)
    gris = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    trouve, coins = cv2.findChessboardCornersSB(gris, (9, 6), flags=cv2.CALIB_CB_ACCURACY)
    assert trouve
    # Coins intérieurs théoriques dans le repère de la mire, puis du monde (= base_link ici).
    i, j = np.meshgrid(np.arange(1, 10), np.arange(1, 7))
    a = (i.ravel() - 5) * mire.case_m
    b = (j.ravel() - 3.5) * mire.case_m
    points = mire.centre + np.outer(a, mire.rotation[:, 0]) + np.outer(b, mire.rotation[:, 1])
    uv, _ = ModeleCamera(profil, "gauche", calibration=False).projeter(points)
    attendus = np.column_stack((uv[:, 0] * 1280 - 0.5, uv[:, 1] * 720 - 0.5))
    trouves = coins.reshape(-1, 2)
    ecarts = np.linalg.norm(trouves[:, None, :] - attendus[None, :, :], axis=2).min(axis=1)
    assert np.max(ecarts) < 0.5  # pixels


def test_collisions(profil: Profil) -> None:
    monde = Monde(0, "salle")
    assert monde.collision(Pose2D(2.0, 6.4, 0.0), profil)  # dans le canapé
    assert monde.collision(Pose2D(6.2, 1.0, 0.0), profil)  # sur le pot (obstacle bas)
    assert monde.collision(Pose2D(9.8, 3.0, 0.0), profil)  # dans le mur est
    assert not monde.collision(Pose2D(6.8, 5.75, 0.0), profil)  # sous la table, entre les pieds
    assert monde.distance_obstacles(Pose2D(6.0, 2.5, 0.0), profil) > 0.3


def test_deplacer() -> None:
    droit = deplacer(Pose2D(), 0.5, 0.0, 2.0)
    assert droit.x == pytest.approx(1.0) and droit.y == pytest.approx(0.0)
    # Braquage constant : cercle de rayon L / tan δ, retour au point de départ.
    rayon = 0.38 / math.tan(0.3)
    pose = Pose2D()
    for _ in range(100):
        pose = deplacer(pose, 2 * math.pi * rayon / 100, 0.3, 1.0)
    assert math.hypot(pose.x, pose.y) < 1e-6
