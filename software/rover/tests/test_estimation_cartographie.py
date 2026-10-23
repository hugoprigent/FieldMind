"""Carte d'occupation : lancer de rayons du lidar, obstacles bas de la stéréo, fenêtre glissante."""

from __future__ import annotations

import math
import time
import tomllib

import numpy as np
from budget import dans_le_budget

from fieldmind.config import FICHIER_PROFIL, Profil
from fieldmind.estimation.cartographie import VALEUR_INDICATIVE, Cartographe
from fieldmind.simulation.monde import Monde, deplacer
from fieldmind.types import Obstacles, Pose2D, Scan


def profil_test() -> Profil:
    with open(FICHIER_PROFIL, "rb") as f:
        base = tomllib.load(f)
    base["cameras"]["calibration"] = "/inexistant/stereo.npz"
    return Profil(base, {}, None)


def valeur(carte, x: float, y: float) -> int:  # type: ignore[no-untyped-def]
    lig, col = carte.indice(x, y)
    return int(carte.cellules[lig, col])


def balayer(cartographe: Cartographe, monde: Monde, profil: Profil, pose: Pose2D,
            pas: int = 30) -> Pose2D:
    """Le rover avance en arc dans la salle ; poses vraies, repère odom = repère du monde."""
    for k in range(pas):
        angles, distances = monde.lancer_lidar(pose, profil)
        cartographe.integrer_scan(Scan(0.1 * k, angles, distances), pose)
        pose = deplacer(pose, 0.4, 0.1, 0.1)
    return pose


def test_murs_occupes_et_centre_libre() -> None:
    profil = profil_test()
    monde = Monde(0, "salle")
    cartographe = Cartographe(profil)
    durees = []
    pose = monde.depart
    for k in range(40):
        angles, distances = monde.lancer_lidar(pose, profil)
        debut = time.perf_counter()
        cartographe.integrer_scan(Scan(0.1 * k, angles, distances), pose)
        durees.append(time.perf_counter() - debut)
        pose = deplacer(pose, 0.4, 0.1, 0.1)
    debut = time.perf_counter()
    carte = cartographe.carte()
    duree_carte = time.perf_counter() - debut
    assert carte.cellules.shape == (800, 800)
    assert carte.resolution == 0.05
    # Mur sud (y = 0) devant le départ, armoire, intérieur de la pièce.
    assert valeur(carte, 3.0, 0.02) > 60
    assert valeur(carte, 9.36, 1.2) > 60
    assert 0 <= valeur(carte, 5.0, 2.5) < 30
    assert valeur(carte, 15.0, 15.0) == -1  # jamais vu (hors de la pièce)
    # Les obstacles bas sont sous le plan du lidar : invisibles dans sa couche.
    assert valeur(carte, 6.4, 1.0) < 50  # pot
    print(f"\nintégration d'un tour : {1000 * np.median(durees):.2f} ms ; "
          f"carte : {1000 * duree_carte:.2f} ms")
    assert dans_le_budget(float(np.median(durees)), 0.01)


def test_obstacles_bas_non_effaces_par_le_lidar() -> None:
    profil = profil_test()
    profil.regler("securite.stereo_bloquante", True)
    monde = Monde(0, "salle")
    cartographe = Cartographe(profil)
    pose = Pose2D(5.0, 1.0, 0.0)
    # La stéréo voit le pot (6,4 ; 1,0) : 1,4 m devant le rover (essieu), un peu moins de face.
    points = np.array([[1.4 - 0.16, dy] for dy in np.linspace(-0.12, 0.12, 6)], np.float32)
    bas = Obstacles(0.0, "stereo", points, np.full(6, 0.28, np.float32))
    for _ in range(2):
        cartographe.integrer_obstacles(bas, pose)
    for k in range(10):
        angles, distances = monde.lancer_lidar(pose, profil)
        cartographe.integrer_scan(Scan(0.1 * k, angles, distances), pose)
    carte = cartographe.carte()
    assert valeur(carte, 6.24, 1.0) == 100
    proches = cartographe.obstacles_proches(pose, 3.0, bas_seulement=True)
    assert len(proches.points) >= 3
    assert np.all(np.abs(proches.points[:, 0] - 1.24) < 0.08)
    # Le pot est enlevé : la stéréo voit le sol nu à sa place, la marque disparaît.
    for _ in range(4):
        cartographe.integrer_obstacles(Obstacles(1.0, "stereo", np.zeros((0, 2), np.float32)),
                                       pose)
    assert valeur(cartographe.carte(), 6.24, 1.0) < 50


def test_stereo_indicative_filtre_isoles_et_garde_les_murs() -> None:
    profil = profil_test()
    profil.regler("securite.stereo_bloquante", False)
    cartographe = Cartographe(profil)
    points = np.array([[1.02, 0.02], [1.02, 0.07], [1.07, 0.02], [2.02, 0.02]])
    for _ in range(2):
        cartographe.integrer_obstacles(Obstacles(0.0, "stereo", points), Pose2D(), liberer=False)
    # Un écho lidar au milieu de l'amas garde sa valeur occupée, même sous la stéréo.
    lig, col = cartographe.carte().indice(1.02, 0.07)
    cartographe.lidar[lig, col] = 3.0
    carte = cartographe.carte()
    assert valeur(carte, 1.02, 0.07) > 65
    assert valeur(carte, 1.02, 0.02) == VALEUR_INDICATIVE
    assert valeur(carte, 2.02, 0.02) == 50  # point isolé écarté
    assert cartographe.occupees().sum() == 1  # seul le lidar est bloquant
    # Le réglage agit aussi sur une carte déjà construite.
    profil.regler("securite.stereo_bloquante", True)
    assert valeur(cartographe.carte(), 2.02, 0.02) == 100
    assert cartographe.occupees().sum() == 4


def test_fenetre_glissante() -> None:
    profil = profil_test()
    monde = Monde(0, "salle")
    cartographe = Cartographe(profil)
    balayer(cartographe, monde, profil, monde.depart, 10)
    avant = cartographe.carte()
    mur = valeur(avant, 3.0, 0.02)
    # Le rover est maintenant à 12 m de l'origine : la grille glisse pour le recentrer.
    loin = Pose2D(12.0, 1.0, 0.0)
    cartographe.integrer_scan(Scan(5.0, np.zeros(0, np.float32), np.zeros(0, np.float32)), loin)
    apres = cartographe.carte()
    assert apres.origine_x > avant.origine_x
    assert abs(apres.origine_x + 20 - 12.0) < 0.1
    assert valeur(apres, 3.0, 0.02) == mur  # le contenu a suivi
    assert math.isclose(apres.resolution, 0.05)
    cartographe.effacer()
    assert np.all(cartographe.carte().cellules == -1)


def test_obstacle_bas_oublie_s_il_n_est_pas_revu() -> None:
    """Une détection de la stéréo qui n'est plus confirmée (fausse, ou hors du champ) s'efface
    d'elle-même en quelques secondes."""
    cartographe = Cartographe(profil_test())
    pose = Pose2D(5.0, 1.0, 0.0)
    # Point sur le côté, hors du champ de la caméra : jamais « vu libre » ensuite
    point = Obstacles(0.0, "stereo", np.array([[0.2, 0.6]], np.float32))
    for _ in range(3):
        cartographe.integrer_obstacles(point, pose, liberer=False)
    assert len(cartographe.obstacles_proches(pose, 3.0, bas_seulement=True).points) == 1
    rien = Obstacles(12.0, "stereo", np.zeros((0, 2), np.float32))
    cartographe.integrer_obstacles(rien, pose, liberer=False)
    assert len(cartographe.obstacles_proches(pose, 3.0, bas_seulement=True).points) == 0
