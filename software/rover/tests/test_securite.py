"""Superviseur de sécurité et conduite : arrêts, plafonds, différentiel, rampes."""

from __future__ import annotations

import math
import time

import numpy as np
import pytest

from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.controle.conduite import Conduite
from fieldmind.securite.superviseur import Superviseur, distance_libre, vitesse_admissible
from fieldmind.types import Consigne, EtatEsp, Imu, Obstacles, Odometrie, Pose2D, Scan


@pytest.fixture
def profil() -> Profil:
    return Profil.charger(reglages=None)


def scan_de(points: list[tuple[float, float]], t: float | None = None) -> Scan:
    p = np.array(points, np.float32)
    return Scan(t=t or time.monotonic(), angles=np.arctan2(p[:, 1], p[:, 0]).astype(np.float32),
                distances=np.hypot(p[:, 0], p[:, 1]).astype(np.float32))


def bus_sain(points: list[tuple[float, float]] | None = None, stereo: bool = True) -> Bus:
    bus = Bus()
    t = time.monotonic()
    bus.publier(S.ESP, EtatEsp(t=t, connecte=True, firmware="ROVER_3"))
    bus.publier(S.SCAN, scan_de(points or [(8.0, 3.0), (-8.0, -3.0)]))
    bus.publier(S.IMU, Imu(t=t, accel_g=(0.0, 0.0, 1.0), gyro_rad_s=(0.0, 0.0, 0.0)))
    bus.publier(S.ODOM, Odometrie(t=t, pose=Pose2D(), vitesse=0.3))
    if stereo:
        bus.publier(S.OBSTACLES_STEREO, Obstacles(t=t, source="stereo",
                                                  points=np.zeros((0, 2), np.float32)))
    return bus


def evaluer(bus: Bus, profil: Profil, vitesse: float, braquage: float = 0.0, mode: str = "manuel",
            arme: bool = True) -> tuple[Consigne, object]:
    sup = Superviseur(bus, profil)
    sup.operateurs = 1
    return sup.evaluer(Consigne(t=time.monotonic(), vitesse=vitesse, braquage=braquage,
                                source="manuel"), mode, arme)  # type: ignore[arg-type]


def test_voie_libre_vitesse_du_mode(profil: Profil) -> None:
    sortie, etat = evaluer(bus_sain(), profil, 0.6)
    assert sortie.vitesse == pytest.approx(0.6)
    assert not etat.raisons


def test_obstacle_devant_arrete_mais_laisse_reculer(profil: Profil) -> None:
    bus = bus_sain([(0.75, 0.0), (-6.0, 0.0)])
    sortie, etat = evaluer(bus, profil, 0.5)
    assert sortie.vitesse == 0.0 and "obstacle_avant" in etat.raisons
    sortie, _ = evaluer(bus, profil, -0.2)
    assert sortie.vitesse == pytest.approx(-0.2)


def test_ralentit_avant_un_obstacle_lointain(profil: Profil) -> None:
    bus = bus_sain([(1.2, 0.0)])
    sortie, etat = evaluer(bus, profil, 0.8)
    assert 0.1 < sortie.vitesse < 0.8 and "ralenti_obstacle" in etat.raisons
    libre = 1.2 - profil.geometrie.emprise_avant_m
    d = (sortie.vitesse * (profil.securite.latence_s + 0.0)
         + sortie.vitesse**2 / (2 * profil.securite.deceleration_m_s2))
    assert d + profil.securite.marge_avant_m <= libre + 0.06


def test_obstacle_sur_le_cote_compte_seulement_en_virage(profil: Profil) -> None:
    bus = bus_sain([(0.7, 0.6)])
    droit, _ = evaluer(bus, profil, 0.6, 0.0)
    gauche, _etat = evaluer(bus, profil, 0.6, math.radians(31))
    assert droit.vitesse == pytest.approx(0.6)
    assert gauche.vitesse < 0.5


def test_obstacle_bas_vu_par_la_stereo(profil: Profil) -> None:
    bus = bus_sain()
    bus.publier(S.OBSTACLES_STEREO, Obstacles(t=time.monotonic(), source="stereo",
                                              points=np.array([[0.7, 0.0]], np.float32)))
    sortie, etat = evaluer(bus, profil, 0.4)
    assert sortie.vitesse == 0.0 and "obstacle_bas" in etat.raisons


def test_sans_stereo_vitesse_plafonnee(profil: Profil) -> None:
    sortie, etat = evaluer(bus_sain(stereo=False), profil, 0.8)
    assert sortie.vitesse == pytest.approx(profil.securite.vitesse_sans_stereo_m_s)
    assert "stereo_absente" in etat.raisons


def test_lidar_perime_ou_esp_absent_arrete(profil: Profil) -> None:
    bus = bus_sain()
    bus.publier(S.SCAN, scan_de([(5.0, 0.0)]))
    bus._dates[S.SCAN] -= 2.0
    sortie, etat = evaluer(bus, profil, 0.3)
    assert sortie.vitesse == 0.0 and "lidar_absent" in etat.raisons
    bus = bus_sain()
    bus.publier(S.ESP, EtatEsp(t=time.monotonic(), connecte=False))
    sortie, etat = evaluer(bus, profil, 0.3)
    assert sortie.vitesse == 0.0 and "esp_absent" in etat.raisons


def test_non_arme_inclinaison_consigne_perimee(profil: Profil) -> None:
    sortie, etat = evaluer(bus_sain(), profil, 0.3, arme=False)
    assert sortie.vitesse == 0.0 and "non_arme" in etat.raisons
    bus = bus_sain()
    bus.publier(S.IMU, Imu(t=time.monotonic(), accel_g=(0.0, 0.6, 0.8),
                           gyro_rad_s=(0.0, 0.0, 0.0)))
    sortie, etat = evaluer(bus, profil, 0.3)
    assert sortie.vitesse == 0.0 and "inclinaison" in etat.raisons
    sup = Superviseur(bus_sain(), profil)
    sup.operateurs = 1
    sortie, etat = sup.evaluer(Consigne(t=time.monotonic() - 1.0, vitesse=0.4, source="manuel"),
                               "manuel", True)
    assert sortie.vitesse == 0.0 and "consigne_perimee" in etat.raisons


def test_blocage_detecte(profil: Profil) -> None:
    bus = bus_sain()
    bus.publier(S.ODOM, Odometrie(t=time.monotonic(), pose=Pose2D(), vitesse=0.0, qualite=1.0))
    sup = Superviseur(bus, profil)
    sup.operateurs = 1
    t0 = time.monotonic()
    for k in range(70):  # 3,5 s de consigne sans déplacement
        bus.publier(S.ESP, EtatEsp(t=time.monotonic(), connecte=True))
        sortie, etat = sup.evaluer(Consigne(t=t0 + 0.05 * k, vitesse=0.3, source="manuel"),
                                   "manuel", True, t0 + 0.05 * k)
    assert sortie.vitesse == 0.0 and "blocage" in etat.raisons


def test_vitesse_admissible() -> None:
    assert vitesse_admissible(0.2, 0.25, 0.8, 0.25) == 0.0
    v = vitesse_admissible(2.0, 0.25, 0.8, 0.25)
    assert v * 0.25 + v * v / 1.6 + 0.25 == pytest.approx(2.0)


def test_distance_libre_arriere(profil: Profil) -> None:
    libre, _ = distance_libre(np.array([[-1.0, 0.0]], np.float32), 0.0, -1, profil)
    assert libre == pytest.approx(1.0 + profil.geometrie.emprise_arriere_m, abs=0.05)


class FauxPilote:
    def __init__(self) -> None:
        self.commandes: list = []

    def connecte(self) -> bool:
        return True

    def appliquer(self, commande) -> None:
        self.commandes.append(commande)

    def arret_traction(self) -> None:
        pass


def test_conduite_differentiel_et_servo(profil: Profil) -> None:
    bus = bus_sain()
    pilote = FauxPilote()
    conduite = Conduite(bus, profil, lambda: pilote, Superviseur(bus, profil))  # type: ignore[arg-type]
    conduite.superviseur.operateurs = 1
    assert conduite.changer_mode("manuel") is None
    assert conduite.armer() is None
    braquage = math.radians(20)
    for _ in range(40):  # 2 s : la rampe atteint la vitesse
        bus.publier(S.ESP, EtatEsp(t=time.monotonic(), connecte=True))
        conduite.proposer(Consigne(t=time.monotonic(), vitesse=0.4, braquage=braquage,
                                   source="manuel"))
        conduite._cycle(time.monotonic())
    c = pilote.commandes[-1]
    assert c.vitesse == pytest.approx(0.4)
    assert c.pwm_d > c.pwm_g > 0  # virage à gauche : roue droite (extérieure) plus rapide
    assert c.servo_deg == pytest.approx(20.0, abs=0.2)
    # Désarmement : traction à zéro au cycle suivant
    conduite.desarmer()
    conduite._cycle(time.monotonic())
    assert pilote.commandes[-1].pwm_g == 0 and pilote.commandes[-1].pwm_d == 0


def test_conduite_refuse_armer_en_mode_arret(profil: Profil) -> None:
    bus = bus_sain()
    conduite = Conduite(bus, profil, lambda: FauxPilote(), Superviseur(bus, profil))  # type: ignore[arg-type]
    assert conduite.armer() is not None
    assert conduite.changer_mode("banc") is not None  # confirmation exigée
    assert conduite.changer_mode("banc", confirmer=True) is None


def test_personne_detectee_devant_ralentit_seulement(profil: Profil) -> None:
    bus = bus_sain()
    sans, _ = evaluer(bus, profil, 0.6)
    assert sans.vitesse == pytest.approx(0.6)  # sans détecteur : rien ne change
    bus.publier(S.DETECTIONS, {"type": "detections", "objets": [
        {"label": "personne", "score": 0.9, "sol": [2.5, 0.3]}]})
    avec, etat = evaluer(bus, profil, 0.6)
    assert avec.vitesse == pytest.approx(profil.securite.vitesse_pres_personne_m_s)
    assert "personne_proche" in etat.raisons
    bus.publier(S.DETECTIONS, {"type": "detections", "objets": [
        {"label": "personne", "score": 0.9, "sol": [2.5, 3.5]}]})  # loin sur le côté
    loin, _ = evaluer(bus, profil, 0.6)
    assert loin.vitesse == pytest.approx(0.6)
