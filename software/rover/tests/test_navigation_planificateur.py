"""Planificateur : A* / Dijkstra sur la carte gonflée, inconnu pénalisé, lissage sans perte de
marge, temps de calcul."""

from __future__ import annotations

import math
import time
import tomllib

import cv2
import numpy as np
import pytest
from budget import dans_le_budget

from fieldmind.config import FICHIER_PROFIL, Profil
from fieldmind.navigation.planificateur import Planificateur
from fieldmind.types import CarteOccupation, Pose2D


@pytest.fixture(scope="module")
def profil() -> Profil:
    with open(FICHIER_PROFIL, "rb") as f:
        base = tomllib.load(f)
    base["cameras"]["calibration"] = "/inexistant/stereo.npz"
    return Profil(base, {}, None)


def carte_vide(taille: float = 12.0, resolution: float = 0.05, valeur: int = 0
               ) -> CarteOccupation:
    n = int(round(taille / resolution))
    return CarteOccupation(0.0, resolution, -1.0, -1.0, np.full((n, n), valeur, np.int8))


def remplir(carte: CarteOccupation, x0: float, y0: float, x1: float, y1: float,
            valeur: int = 100) -> None:
    l0, c0 = carte.indice(x0, y0)
    l1, c1 = carte.indice(x1, y1)
    carte.cellules[l0:l1 + 1, c0:c1 + 1] = valeur


def marges(carte: CarteOccupation, points: np.ndarray) -> np.ndarray:
    """Distance (m) de chaque point du chemin à la cellule occupée la plus proche."""
    libre = np.where(carte.cellules >= 65, 0, 255).astype(np.uint8)
    distance = cv2.distanceTransform(libre, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * carte.resolution
    lignes = ((points[:, 1] - carte.origine_y) / carte.resolution).astype(int)
    colonnes = ((points[:, 0] - carte.origine_x) / carte.resolution).astype(int)
    return distance[lignes, colonnes]


def test_contourne_un_mur(profil: Profil) -> None:
    carte = carte_vide()
    remplir(carte, 4.0, -1.0, 4.2, 6.0)  # mur de y = −1 à 6 ; passage au-dessus (y > 6)
    planificateur = Planificateur(profil)
    chemin = planificateur.planifier(carte, Pose2D(1.0, 2.0, 0.0), (8.0, 2.0))
    assert chemin is not None
    assert chemin.but == (8.0, 2.0)
    assert chemin.points[0] == pytest.approx((1.0, 2.0))
    assert chemin.points[-1] == pytest.approx((8.0, 2.0))
    assert np.all(np.hypot(*np.diff(chemin.points, axis=0).T) <= 0.1001)
    assert chemin.points[:, 1].max() > 6.0
    # Carte gonflée : l'essieu arrière reste à plus d'une demi-largeur + marge des obstacles.
    assert marges(carte, chemin.points).min() >= 0.225 + 0.10 - 0.05
    assert chemin.longueur_m == pytest.approx(
        float(np.sum(np.hypot(*np.diff(chemin.points, axis=0).T))))


def test_lissage_sans_perte_de_marge(profil: Profil) -> None:
    """Dans un couloir de 1,2 m, le chemin lissé reste au milieu (marge ≥ celle du brut)."""
    carte = carte_vide()
    remplir(carte, 0.0, 1.0, 9.0, 1.1)
    remplir(carte, 0.0, 2.3, 9.0, 2.4)
    chemin = Planificateur(profil).planifier(carte, Pose2D(0.5, 1.7, 0.0), (8.5, 1.7))
    assert chemin is not None
    assert np.all(np.abs(chemin.points[:, 1] - 1.7) < 0.08)
    assert marges(carte, chemin.points).min() > 0.5
    # Ligne droite : un seul segment après lissage, 81 points à 10 cm.
    assert len(chemin.points) == pytest.approx(81, abs=1)


def test_inconnu_traversable_mais_penalise(profil: Profil) -> None:
    carte = carte_vide()
    remplir(carte, -1.0, 0.0, 11.0, 4.0, valeur=-1)  # bande inconnue au milieu
    planificateur = Planificateur(profil)
    chemin = planificateur.planifier(carte, Pose2D(1.0, -0.5, 0.0), (9.0, -0.5))
    assert chemin is not None
    assert chemin.points[:, 1].max() < 0.1  # le long, dans le connu
    traversee = planificateur.planifier(carte, Pose2D(1.0, -0.5, 0.0), (1.0, 5.0))
    assert traversee is not None  # l'inconnu ne bloque pas
    assert traversee.longueur_m < 7.0


def test_but_dans_un_obstacle_et_sans_issue(profil: Profil) -> None:
    carte = carte_vide()
    remplir(carte, 5.0, 2.0, 5.4, 2.4)
    planificateur = Planificateur(profil)
    chemin = planificateur.planifier(carte, Pose2D(1.0, 2.2, 0.0), (5.2, 2.2))
    assert chemin is not None  # but déplacé vers la cellule libre la plus proche
    assert math.dist(chemin.but, (5.2, 2.2)) < 1.0
    assert marges(carte, chemin.points[-1:])[0] >= 0.3
    # But enfermé dans une boîte : pas de chemin.
    remplir(carte, 7.0, 7.0, 9.0, 7.1)
    remplir(carte, 7.0, 9.0, 9.0, 9.1)
    remplir(carte, 7.0, 7.0, 7.1, 9.1)
    remplir(carte, 8.9, 7.0, 9.0, 9.1)
    assert planificateur.planifier(carte, Pose2D(1.0, 2.2, 0.0), (8.0, 8.0)) is None
    assert planificateur.dernier_bilan["etat"] == "aucun_chemin"


def test_depart_colle_a_un_mur(profil: Profil) -> None:
    """Rover garé à 25 cm d'un mur (dans la zone gonflée) : il peut en sortir."""
    carte = carte_vide()
    remplir(carte, -1.0, 0.0, 11.0, 0.1)
    chemin = Planificateur(profil).planifier(carte, Pose2D(2.0, 0.37, 0.0), (6.0, 3.0))
    assert chemin is not None


def test_temps_40_m(profil: Profil) -> None:
    """40 × 40 m à 5 cm, obstacles épars et un long mur : < 50 ms (grille passée à 20 cm)."""
    rng = np.random.default_rng(0)
    cellules = np.zeros((800, 800), np.int8)
    for _ in range(300):
        x, y = rng.integers(0, 800, 2)
        cellules[max(0, y - 6):y + 6, max(0, x - 6):x + 6] = 100
    cellules[400, 100:700] = 100
    cellules[:, :5] = -1
    carte = CarteOccupation(0.0, 0.05, -20.0, -20.0, cellules)
    planificateur = Planificateur(profil)
    durees = []
    for _ in range(3):
        debut = time.perf_counter()
        chemin = planificateur.planifier(carte, Pose2D(-19.0, -19.0, 0.0), (19.0, 19.0))
        durees.append(time.perf_counter() - debut)
    assert chemin is not None
    assert marges(carte, chemin.points).min() >= 0.25
    print(f"\nplanification 40 × 40 m : {1000 * min(durees):.1f} ms "
          f"({planificateur.dernier_bilan})")
    assert dans_le_budget(min(durees), 0.05)
