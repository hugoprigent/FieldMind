"""Différentiel électronique : géométrie, angle réel du servo, boucle de lacet, capteurs de roue."""

from __future__ import annotations

import math

import pytest

from fieldmind.config import Profil
from fieldmind.controle.differentiel import Differentiel
from fieldmind.simulation.materiel_simule import PhysiqueRover
from fieldmind.types import EtatEsp, Imu


@pytest.fixture
def profil(tmp_path) -> Profil:
    return Profil.charger(reglages=tmp_path / "reglages.json")


def esp(servo_deg: float, t: float, **champs) -> EtatEsp:
    return EtatEsp(t=t, connecte=True, servo_actif=True, servo_deg=servo_deg, **champs)


def test_geometrie_ackermann(profil: Profil) -> None:
    d = Differentiel(profil)
    delta = math.radians(20)
    r = d.repartir(0.4, delta, esp(20.0, 0.0), None, 0.0, 0.05)
    kappa = math.tan(delta) / 0.38
    assert r.v_d / r.v_g == pytest.approx((1 + kappa * 0.195) / (1 - kappa * 0.195), rel=1e-6)
    assert r.mode == "geometrie"  # pas de centrale : pas de boucle de lacet


def test_suit_la_position_reelle_du_servo(profil: Profil) -> None:
    d = Differentiel(profil)
    r = d.repartir(0.4, math.radians(25), esp(5.0, 0.0), None, 0.0, 0.05)
    assert r.braquage_reel == pytest.approx(math.radians(5))  # le servo n'y est pas encore
    r = d.repartir(0.4, math.radians(25), None, None, 0.0, 0.05)
    assert r.braquage_reel == pytest.approx(math.radians(25))  # sans télémétrie : la consigne


def test_centrale_lente_pas_de_boucle(profil: Profil) -> None:
    d = Differentiel(profil)
    for k in range(10):  # ROVER_2 : une mesure toutes les 500 ms
        d.imu(Imu(t=k * 0.5, accel_g=(0, 0, 1), gyro_rad_s=(0, 0, 0.0)))
    r = d.repartir(0.4, math.radians(20), esp(20.0, 4.5), None, 4.5, 0.05)
    assert r.mode == "geometrie" and r.correction == 0


def rouler(profil: Profil, boucle: bool, duree: float = 6.0, braquage_deg: float = 20.0
           ) -> tuple[float, float, PhysiqueRover]:
    """Boucle fermée à 50 Hz : différentiel → PWM → physique (sous-virage, moteur droit faible)
    → gyroscope. Renvoie le lacet attendu et le lacet obtenu en régime établi."""
    profil.regler("traction.boucle_lacet", boucle)
    d = Differentiel(profil)
    physique = PhysiqueRover(profil)
    physique.servo_neutre_reel = 0.0
    dt, t, v = 0.02, 0.0, 0.35
    mesures = []
    while t < duree:
        d.imu(Imu(t=t, accel_g=(0, 0, 1), gyro_rad_s=(0, 0, physique.vitesse_lacet)))
        r = d.repartir(v, math.radians(braquage_deg), esp(braquage_deg, t), None, t, dt)
        pwm_g, pwm_d = d.pwm(r, None, 0.0, dt)
        physique.avancer(dt, pwm_g, pwm_d, braquage_deg)
        if t > duree - 2:
            mesures.append((r.lacet_attendu, physique.vitesse_lacet))
        t += dt
    attendu = sum(m[0] for m in mesures) / len(mesures)
    obtenu = sum(m[1] for m in mesures) / len(mesures)
    return attendu, obtenu, physique


@pytest.mark.parametrize("braquage", [20.0, -25.0, 0.0])
def test_boucle_de_lacet_compense_sous_virage_et_moteurs(profil: Profil, braquage: float) -> None:
    attendu, sans, _ = rouler(profil, False, braquage_deg=braquage)
    attendu2, avec, _ = rouler(profil, True, braquage_deg=braquage)
    erreur_sans, erreur_avec = abs(attendu - sans), abs(attendu2 - avec)
    assert erreur_avec < 0.5 * erreur_sans + 0.01, (erreur_sans, erreur_avec)


def test_capteurs_de_roue_regulent_chaque_roue(profil: Profil) -> None:
    profil.regler("roues.impulsions_par_metre", 1000.0)
    d = Differentiel(profil)
    r = d.repartir(0.3, 0.0, None, None, 0.0, 0.05)
    lent = esp(0.0, 0.0, hz_g=300.0, hz_d=200.0)  # roue droite à 0,2 m/s au lieu de 0,3
    for _ in range(20):
        pwm_g, pwm_d = d.pwm(r, lent, 0.0, 0.05)
    assert pwm_d > pwm_g and r.mode == "roues"
