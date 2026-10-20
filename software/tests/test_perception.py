"""Tests de la grille d'occupation et de la localisation (EKF)."""

from __future__ import annotations

import base64
import math
import zlib
from pathlib import Path

import numpy as np
import pytest

from services.navigation.grille import INCONNU, GrilleOccupation, PoseCapteur
from services.navigation.localisation import Localisation
from services.protocole.fiche_rover import FicheRover
from services.protocole.messages import Gnss, Imu, Odometrie, Scan, Teleoperation
from services.protocole.scenario import Limites, PoseDepart
from simulation.banc_essai import BancEssai
from simulation.monde import charger_monde

RACINE = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def fiche() -> FicheRover:
    return FicheRover.charger(RACINE / "config" / "rover_fieldmind.json")


def scan_mur(t: float, distance: float) -> Scan:
    """Mur perpendiculaire à l'axe x, vu entre -20° et +20° ; rien ailleurs."""
    n = 360
    pas = 2 * math.pi / n
    distances: list[float | None] = []
    for i in range(n):
        a = -math.pi + i * pas
        distances.append(distance / math.cos(a) if abs(a) < math.radians(20) else None)
    return Scan(
        t=t, angle_min=-math.pi, pas=pas, portee_min=0.1, portee_max=8.0, distances=distances
    )


# ------------------------------------------------------------------ grille


def test_grille_marque_le_mur_et_libere_le_chemin() -> None:
    grille = GrilleOccupation(Limites(xmin=-5, xmax=5, ymin=-5, ymax=5), 0.1)
    for k in range(3):
        grille.integrer(PoseCapteur(0.0, 0.0, 0.0), scan_mur(k * 0.2, 3.0))
    occupee = grille.occupee()
    c_mur, l0 = (int(v) for v in grille.cellule(np.array(3.0), np.array(0.0)))
    assert occupee[l0, c_mur], "le mur à 3 m doit être occupé"
    c_libre, _ = (int(v) for v in grille.cellule(np.array(1.5), np.array(0.0)))
    assert grille.log_odds[l0, c_libre] < 0, "l'espace avant le mur doit être libre"
    c_derriere, _ = (int(v) for v in grille.cellule(np.array(4.0), np.array(0.0)))
    assert not grille.observee[l0, c_derriere], "derrière le mur : inconnu"


def test_grille_ignore_un_scan_si_la_pose_est_trop_incertaine() -> None:
    grille = GrilleOccupation(Limites(xmin=-5, xmax=5, ymin=-5, ymax=5), 0.1)
    poids = GrilleOccupation.poids_pour(0.8)
    assert poids == 0.0
    assert grille.integrer(PoseCapteur(0, 0, 0), scan_mur(0.0, 3.0), poids) == 0
    assert not grille.observee.any()


def test_extraction_compressee_des_modifications() -> None:
    grille = GrilleOccupation(Limites(xmin=-5, xmax=5, ymin=-5, ymax=5), 0.1)
    grille.integrer(PoseCapteur(0.0, 0.0, 0.0), scan_mur(0.0, 3.0))
    extrait = grille.extraire_modifications()
    assert extrait is not None
    (_, _, largeur, hauteur), donnees = extrait
    octets = np.frombuffer(zlib.decompress(base64.b64decode(donnees)), dtype=np.uint8)
    assert octets.size == largeur * hauteur
    assert (octets == INCONNU).any()
    assert (octets < 127).any()
    assert grille.extraire_modifications() is None, "rien de neuf depuis le dernier envoi"


# ------------------------------------------------------------------ localisation


def test_localisation_immobile_converge_sur_le_gnss(fiche: FicheRover) -> None:
    loc = Localisation(fiche, PoseDepart(x=1.0, y=1.0, cap=0.0))
    gx = fiche.capteurs.gnss.position[0]
    for k in range(200):
        t = k * 0.02
        loc.sur_imu(
            Imu(
                t=t,
                orientation=(1, 0, 0, 0),
                vitesse_angulaire=(0, 0, 0),
                acceleration=(0, 0, 9.81),
            )
        )
        loc.sur_odometrie(
            Odometrie(t=t, roue_av_g=0, roue_av_d=0, roue_ar_g=0, roue_ar_d=0, braquage=0)
        )
        if k % 5 == 0:
            loc.sur_gnss(Gnss(t=t, fix="rtk_fixe", x=gx, y=0.0, sigma=0.02, satellites=20))
    p = loc.pose()
    assert p.x == pytest.approx(0.0, abs=0.02)
    assert p.y == pytest.approx(0.0, abs=0.02)
    assert p.sigma_xy < 0.03


def test_localisation_rejette_un_saut_gnss(fiche: FicheRover) -> None:
    loc = Localisation(fiche, PoseDepart(x=0.0, y=0.0, cap=0.0))
    gx = fiche.capteurs.gnss.position[0]
    for k in range(100):
        t = k * 0.02
        loc.sur_imu(
            Imu(
                t=t,
                orientation=(1, 0, 0, 0),
                vitesse_angulaire=(0, 0, 0),
                acceleration=(0, 0, 9.81),
            )
        )
        if k % 5 == 0:
            loc.sur_gnss(Gnss(t=t, fix="rtk_fixe", x=gx, y=0.0, sigma=0.02, satellites=20))
    assert not loc.sur_gnss(
        Gnss(t=2.0, fix="rtk_fixe", x=gx + 3.0, y=0.0, sigma=0.02, satellites=20)
    )
    assert loc.pose().x == pytest.approx(0.0, abs=0.03)


def test_localisation_en_roulant_dans_la_foret(fiche: FicheRover) -> None:
    banc = BancEssai(fiche, charger_monde(RACINE / "simulation" / "scenarios" / "foret_demo.json"))
    programme = [(4, 1.0, 0.0), (3, 0.8, 0.25), (4, 1.0, -0.1), (3, 0.6, 0.6)]
    for duree, avance, direction in programme:
        teleop = Teleoperation(avance=avance, direction=direction)
        banc.avancer(duree, lambda b, m=teleop: b.cerveau.sur_teleoperation(m) or False)
    erreurs = np.array(banc.releve.erreurs_position[20:])  # après convergence initiale
    # Sous le couvert, le GNSS passe en RTK flottant (σ 12 cm) : quelques pointes à ~10 cm.
    assert np.median(erreurs) < 0.03
    assert np.percentile(erreurs, 95) < 0.08
    assert banc.cerveau.grille.occupee().sum() > 50
    # Conduite manuelle à l'aveugle : une collision latérale avec un arbre reste possible (le
    # contrôleur embarqué ne protège que l'avant et l'arrière) ; ce test porte sur la localisation.
