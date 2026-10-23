"""Le dessin des volumes attend une mesure stable ; les dangers bruts restent visibles."""

from __future__ import annotations

import numpy as np
import pytest

from fieldmind.config import Profil
from fieldmind.perception.scene import ReconstructeurScene
from fieldmind.types import EtatSecurite, Obstacles, Pose2D, Scan


def echo(t: float) -> Scan:
    return Scan(t=t, angles=np.linspace(-0.02, 0.02, 5, dtype=np.float32),
                distances=np.full(5, 1.0, np.float32))


def scene() -> ReconstructeurScene:
    return ReconstructeurScene(Profil.charger(reglages=None))


def test_echo_lidar_isole_ne_devient_pas_arbre_ou_gros_bloc() -> None:
    r, pose = scene(), Pose2D()
    scan = echo(0)
    distances = scan.distances.copy()
    r.integrer_scan(scan, pose, 0)
    assert r.message(pose, None, None, maintenant=0.01)["objets"] == []
    np.testing.assert_array_equal(scan.distances, distances)
    # Plusieurs points du même scan ne sont pas plusieurs observations temporelles.
    for _ in range(4):
        r.integrer_scan(scan, pose, 0)
    assert not r.message(pose, None, None, maintenant=0.02)["objets"]
    for t in (0.1, 0.2):
        r.integrer_scan(echo(t), pose, t)
    objet, = r.message(pose, None, None, maintenant=0.21)["objets"]
    assert objet["observations_distinctes"] == 3
    assert objet["hauteur"] == 0.3 and objet["hauteur_estimee"]


def test_arbuste_mesure_stereo_garde_sa_hauteur() -> None:
    r, pose = scene(), Pose2D()
    r.mesurer_stereo = lambda *_args: {"hauteur": 0.62, "etendue": 0.55}
    for t in (0, 0.1, 0.2):
        r.integrer_scan(echo(t), pose, t)
    objet, = r.message(pose, None, None, maintenant=0.21)["objets"]
    assert objet["type"] == "bloc"
    assert objet["hauteur"] == pytest.approx(0.62)
    assert not objet["hauteur_estimee"]


def test_mur_mesure_reste_immediat() -> None:
    r, pose = scene(), Pose2D()
    y = np.linspace(-1.5, 1.5, 60)
    scan = Scan(t=0, angles=np.arctan2(y, 2).astype(np.float32),
                distances=np.hypot(2, y).astype(np.float32))
    r.integrer_scan(scan, pose, 0)
    assert any(o["type"] == "mur" for o in r.message(pose, None, None, maintenant=0.01)["objets"])


def test_pelouse_basse_pas_de_bloc_mais_danger_conserve() -> None:
    r, pose = scene(), Pose2D()
    points = np.array([[1 + 0.03 * i, 0.02 * i] for i in range(8)], np.float32)
    hauteurs = np.full(8, 0.04, np.float32)
    obstacles = Obstacles(t=0, source="stereo", points=points, hauteurs=hauteurs)
    r.integrer_stereo(obstacles, pose, 0)
    assert not r.message(pose, None, None, maintenant=0.01)["objets"]
    securite = EtatSecurite(t=0, arret=True, raisons=["obstacle_bas"], libre_avant_m=0.8)
    objet, = r.message(pose, None, securite, maintenant=0.02)["objets"]
    assert objet["danger"] and objet["hauteur"] == pytest.approx(0.04)
    np.testing.assert_array_equal(obstacles.points, points)
    np.testing.assert_array_equal(obstacles.hauteurs, hauteurs)


def test_echo_provisoire_dangereux_affiche_immediatement() -> None:
    r, pose = scene(), Pose2D()
    r.integrer_scan(echo(0), pose, 0)
    securite = EtatSecurite(t=0, arret=True, raisons=["obstacle_avant"], libre_avant_m=0.8)
    objet, = r.message(pose, None, securite, maintenant=0.01)["objets"]
    assert objet["danger"] and objet["observations_distinctes"] == 1
