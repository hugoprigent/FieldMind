"""Tests du simulateur : monde, physique MuJoCo, capteurs et firmware du contrôleur embarqué."""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np
import pytest

from services.protocole.fiche_rover import FicheRover
from services.protocole.messages import Commande, Firmware, Gnss, Odometrie, Scan
from simulation.monde import Monde, charger_monde, distance_a_polyligne
from simulation.robot_simule import RobotSimule

RACINE = Path(__file__).resolve().parents[1]
SCENARIO = RACINE / "simulation" / "scenarios" / "foret_demo.json"


@pytest.fixture(scope="module")
def fiche() -> FicheRover:
    return FicheRover.charger(RACINE / "config" / "rover_fieldmind.json")


@pytest.fixture(scope="module")
def monde() -> Monde:
    return charger_monde(SCENARIO)


def arbre_degage(monde: Monde, recul: float = 5.5) -> int:
    """Indice d'un arbre qu'on peut approcher en ligne droite vers l'est sur `recul` mètres."""
    obstacles = [(a.x, a.y, a.rayon) for a in monde.arbres] + [
        (r.x, r.y, r.rayon) for r in monde.rochers
    ]
    lim = monde.scenario.limites
    for i, a in enumerate(monde.arbres):
        if a.x - recul - 1 < lim.xmin:
            continue
        gene = any(
            (ox, oy) != (a.x, a.y)
            and a.x - recul - 1.0 < ox + r
            and ox - r < a.x + 0.5
            and abs(oy - a.y) < r + 0.9
            for ox, oy, r in obstacles
        )
        if not gene:
            return i
    raise AssertionError("aucun arbre dégagé dans ce monde")


def conduire(robot: RobotSimule, duree: float, vitesse: float, braquage: float = 0.0) -> list:
    """Envoie une consigne constante à 20 Hz pendant `duree` ; renvoie les messages publiés."""
    messages = []
    for _ in range(round(duree / 0.05)):
        robot.recevoir(Commande(t=robot.t, vitesse=vitesse, braquage=braquage))
        messages += robot.avancer(0.05)
    return messages


def placer(robot: RobotSimule, x: float, y: float, cap: float) -> None:
    robot.donnees.qpos[0:3] = [x, y, robot.fiche.geometrie.rayon_roue + 0.002]
    robot.donnees.qpos[3:7] = [math.cos(cap / 2), 0, 0, math.sin(cap / 2)]
    robot.donnees.qvel[:] = 0
    mujoco.mj_forward(robot.modele, robot.donnees)


# ------------------------------------------------------------------ monde


def test_monde_reproductible_et_dependant_de_la_graine() -> None:
    a, b, c = charger_monde(SCENARIO, 7), charger_monde(SCENARIO, 7), charger_monde(SCENARIO, 8)
    assert a.arbres == b.arbres
    assert a.arbres != c.arbres


def test_piste_et_lieux_degages(monde: Monde) -> None:
    assert len(monde.arbres) == 400
    arbres = np.array([[a.x, a.y] for a in monde.arbres])
    zone = monde.params.piste.demi_largeur + monde.params.piste.degagement
    assert distance_a_polyligne(arbres, monde.piste).min() > zone
    for lieu in monde.scenario.lieux:
        assert np.hypot(*(arbres - [lieu.x, lieu.y]).T).min() > monde.params.degagement_lieux


# ------------------------------------------------------------------ physique


def test_rover_stable_au_repos(fiche: FicheRover, monde: Monde) -> None:
    robot = RobotSimule(fiche, monde)
    conduire(robot, 2.0, 0.0)
    v = robot.verite()
    depart = monde.scenario.depart
    sol = monde.hauteur_sol(depart.x, depart.y)
    assert v.z == pytest.approx(sol + fiche.geometrie.rayon_roue, abs=0.02)
    assert math.hypot(v.x - monde.scenario.depart.x, v.y - monde.scenario.depart.y) < 0.01
    assert v.collisions == 0


def test_regulation_de_vitesse(fiche: FicheRover, monde: Monde) -> None:
    robot = RobotSimule(fiche, monde)
    conduire(robot, 4.0, 1.0)
    assert robot.verite().vitesse == pytest.approx(1.0, abs=0.05)


def test_direction_ackermann(fiche: FicheRover, monde: Monde) -> None:
    robot = RobotSimule(fiche, monde)
    placer(robot, 19.5, 26.0, 0.0)  # clairière
    conduire(robot, 2.0, 0.5, math.radians(20))
    v = robot.verite()
    assert v.braquage_gauche > v.braquage_droit > 0, (
        "roue intérieure plus braquée en virage à gauche"
    )
    attendu_g, attendu_d = robot.firmware.ackermann(math.radians(20))
    assert v.braquage_gauche == pytest.approx(attendu_g, abs=math.radians(1.5))
    assert v.braquage_droit == pytest.approx(attendu_d, abs=math.radians(1.5))


# ------------------------------------------------------------------ firmware


def test_chien_de_garde_arrete_le_rover(fiche: FicheRover, monde: Monde) -> None:
    robot = RobotSimule(fiche, monde)
    conduire(robot, 3.0, 1.0)
    messages = robot.avancer(2.0)  # plus aucune consigne
    assert robot.verite().vitesse == pytest.approx(0.0, abs=0.02)
    assert any(isinstance(m, Firmware) and m.arret == "chien_de_garde" for m in messages)


@pytest.mark.parametrize("vitesse", [0.6, 1.0, 1.5])
def test_arret_urgence_avant_un_arbre(fiche: FicheRover, monde: Monde, vitesse: float) -> None:
    arbre = monde.arbres[arbre_degage(monde)]
    robot = RobotSimule(fiche, monde)
    placer(robot, arbre.x - 5.0, arbre.y, 0.0)
    messages = conduire(robot, 8.0, vitesse)
    v = robot.verite()
    assert v.collisions == 0
    assert abs(v.vitesse) < 0.02
    assert any(isinstance(m, Firmware) and m.arret == "obstacle_avant" for m in messages)
    pare_chocs = v.x + fiche.geometrie.longueur / 2
    assert arbre.x - arbre.rayon - pare_chocs > 0.05


def test_marche_arriere_autorisee_apres_arret(fiche: FicheRover, monde: Monde) -> None:
    arbre = monde.arbres[arbre_degage(monde)]
    robot = RobotSimule(fiche, monde)
    placer(robot, arbre.x - 5.0, arbre.y, 0.0)
    conduire(robot, 8.0, 1.0)
    x_arret = robot.verite().x
    conduire(robot, 2.0, -0.4)
    assert robot.verite().x < x_arret - 0.3


# ------------------------------------------------------------------ capteurs


def test_frequences_des_capteurs(fiche: FicheRover, monde: Monde) -> None:
    robot = RobotSimule(fiche, monde)
    messages = conduire(robot, 10.0, 0.5)
    compte: dict[str, int] = {}
    for m in messages:
        compte[m.type] = compte.get(m.type, 0) + 1
    assert compte["imu"] == pytest.approx(500, abs=3)
    assert compte["odometrie"] == pytest.approx(500, abs=3)
    assert compte["scan"] == pytest.approx(60, abs=2)
    assert compte["gnss"] == pytest.approx(100, abs=2)
    assert compte["firmware"] == pytest.approx(100, abs=2)
    temps = [m.t for m in messages if isinstance(m, Odometrie)]
    assert temps == sorted(temps)


def test_gnss_publie_avec_latence(fiche: FicheRover, monde: Monde) -> None:
    robot = RobotSimule(fiche, monde)
    for _ in range(40):
        for m in robot.avancer(0.05):
            if isinstance(m, Gnss):
                assert robot.t - m.t >= fiche.capteurs.gnss.latence_s - 1e-6


def test_lidar_mesure_un_tronc(fiche: FicheRover, monde: Monde) -> None:
    arbre = monde.arbres[arbre_degage(monde)]
    robot = RobotSimule(fiche, monde)
    placer(robot, arbre.x - 3.0, arbre.y, 0.0)
    scans = [m for m in robot.avancer(0.4) if isinstance(m, Scan)]
    scan = scans[-1]
    lx = fiche.capteurs.lidar.position[0]
    # Faisceau qui vise le centre du tronc depuis le lidar.
    dx, dy = arbre.x - (arbre.x - 3.0 + lx), 0.0
    angle = math.atan2(dy, dx)
    i = round((angle - scan.angle_min) / scan.pas) % len(scan.distances)
    voisins = [d for d in scan.distances[i - 1 : i + 2] if d is not None]
    assert min(voisins) == pytest.approx(math.hypot(dx, dy) - arbre.rayon, abs=0.04)
