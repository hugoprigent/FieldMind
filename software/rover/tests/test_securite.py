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
    bus = bus_sain([(0.62, 0.0), (-6.0, 0.0)])  # nez à 0,46 m + 0,20 m minimum
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
    bus = bus_sain([(0.7, 0.45)])
    droit, _ = evaluer(bus, profil, 0.6, 0.0)
    gauche, _etat = evaluer(bus, profil, 0.6, math.radians(31))
    assert droit.vitesse == pytest.approx(0.6)
    assert gauche.vitesse < 0.5


def test_obstacle_bas_vu_par_la_stereo(profil: Profil) -> None:
    profil.regler("securite.stereo_bloquante", True)
    bus = bus_sain()
    bus.publier(S.OBSTACLES_STEREO, Obstacles(t=time.monotonic(), source="stereo",
                                              points=np.array([[0.7, 0.0]], np.float32)))
    sortie, etat = evaluer(bus, profil, 0.4)
    assert sortie.vitesse == 0.0 and "obstacle_bas" in etat.raisons


def test_stereo_non_bloquante_ralentit_seulement(profil: Profil) -> None:
    """Réglage « seul le lidar arrête » : l'obstacle bas de la stéréo plafonne la vitesse, un
    obstacle du lidar arrête toujours."""
    profil.regler("securite.stereo_bloquante", False)
    bus = bus_sain()
    bus.publier(S.OBSTACLES_STEREO, Obstacles(t=time.monotonic(), source="stereo",
                                              points=np.array([[0.7, 0.0]], np.float32)))
    sortie, etat = evaluer(bus, profil, 0.6)
    assert sortie.vitesse == pytest.approx(profil.valeur("securite.vitesse_obstacle_bas_m_s"))
    assert "ralenti_obstacle_bas" in etat.raisons
    sortie, etat = evaluer(bus_sain([(0.62, 0.0)]), profil, 0.5)
    assert sortie.vitesse == 0.0 and "obstacle_avant" in etat.raisons


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


def test_direction_par_l_arriere(profil: Profil) -> None:
    """Servo hors service : le virage vient de la différence de vitesse des roues arrière,
    servo au neutre ; à l'arrêt, joystick sur le côté, rotation sur place, refusée si le
    lidar voit un obstacle dans le cercle balayé."""
    profil.regler("direction.par_l_arriere", True)
    bus = bus_sain()
    pilote = FauxPilote()
    conduite = Conduite(bus, profil, lambda: pilote, Superviseur(bus, profil))  # type: ignore[arg-type]
    conduite.superviseur.operateurs = 1
    assert conduite.changer_mode("manuel") is None and conduite.armer() is None

    def cycles(vitesse: float, braquage: float, n: int = 40) -> object:
        for _ in range(n):
            bus.publier(S.ESP, EtatEsp(t=time.monotonic(), connecte=True))
            conduite.proposer(Consigne(t=time.monotonic(), vitesse=vitesse, braquage=braquage,
                                       source="manuel"))
            conduite._cycle(time.monotonic())
        return pilote.commandes[-1]

    c = cycles(0.4, math.radians(20))
    assert c.pwm_d > c.pwm_g > 0 and c.servo_deg == pytest.approx(0.0)
    c = cycles(0.0, math.radians(25))
    assert c.pwm_g < 0 < c.pwm_d  # rotation sur place vers la gauche
    c = cycles(0.0, -math.radians(25))
    assert c.pwm_d < 0 < c.pwm_g
    bus.publier(S.SCAN, Scan(t=time.monotonic(), angles=np.array([1.5]),
                             distances=np.array([0.3])))  # obstacle sur le côté, à 30 cm
    c = cycles(0.0, math.radians(25), n=3)
    assert c.pwm_g == 0 and c.pwm_d == 0


def test_conduite_refuse_armer_en_mode_arret(profil: Profil) -> None:
    bus = bus_sain()
    conduite = Conduite(bus, profil, lambda: FauxPilote(), Superviseur(bus, profil))  # type: ignore[arg-type]
    assert conduite.armer() is not None
    assert conduite.changer_mode("banc") is not None  # confirmation exigée
    assert conduite.changer_mode("banc", confirmer=True) is None


@pytest.mark.parametrize("mode", ["manuel", "auto"])
def test_armement_auto_au_changement_de_mode_et_apres_urgence(profil: Profil, mode: str) -> None:
    profil.regler("conduite.armer_auto", True)
    bus = bus_sain()
    conduite = Conduite(bus, profil, lambda: FauxPilote(), Superviseur(bus, profil))
    assert conduite.mode == "arret" and not conduite.arme
    assert conduite.changer_mode(mode) is None
    assert conduite.arme
    conduite.arret_urgence("essai")
    assert not conduite.arme
    # Même si une source propose encore du mouvement pendant l'urgence, rien n'est repris.
    conduite.proposer(Consigne(t=time.monotonic(), vitesse=0.5, source=mode))
    assert conduite.armer() is not None
    conduite.rearmer()
    assert conduite.arme and not conduite.superviseur.arret_urgence
    assert conduite.consigne_courante().vitesse == 0
    conduite.desarmer()
    conduite._cycle(time.monotonic())
    assert not conduite.arme  # le désarmement explicite reste respecté
    conduite.changer_mode("arret")
    assert not conduite.arme


def test_armement_auto_respecte_esp_absent_urgence_et_banc(profil: Profil) -> None:
    profil.regler("conduite.armer_auto", True)
    bus = bus_sain()
    conduite = Conduite(bus, profil, lambda: None, Superviseur(bus, profil))
    conduite.changer_mode("manuel")
    assert not conduite.arme
    conduite = Conduite(bus, profil, lambda: FauxPilote(), Superviseur(bus, profil))
    conduite.arret_urgence("essai")
    conduite.changer_mode("auto")
    assert not conduite.arme and conduite.superviseur.arret_urgence
    conduite.changer_mode("banc", confirmer=True)
    conduite.rearmer()
    assert not conduite.arme


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


def test_manuel_sans_anticollision(profil: Profil) -> None:
    """Bouton du poste : en manuel seulement, obstacles ignorés mais vitesse plafonnée ; l'arrêt
    d'urgence et le désarmement s'appliquent toujours."""
    bus = bus_sain([(0.5, 0.0)])  # collé au nez
    sup = Superviseur(bus, profil)
    sup.operateurs = 1
    consigne = Consigne(t=time.monotonic(), vitesse=0.6, source="manuel")
    sortie, _ = sup.evaluer(consigne, "manuel", True)
    assert sortie.vitesse == 0.0
    sup.anticollision_coupee = True
    sortie, etat = sup.evaluer(Consigne(t=time.monotonic(), vitesse=0.6, source="manuel"),
                               "manuel", True)
    assert sortie.vitesse == pytest.approx(0.3) and "anticollision_coupee" in etat.raisons
    sortie, _ = sup.evaluer(Consigne(t=time.monotonic(), vitesse=0.6, source="auto"), "auto", True)
    assert sortie.vitesse == 0.0  # jamais en automatique
    sup.declencher("essai")
    sortie, _ = sup.evaluer(Consigne(t=time.monotonic(), vitesse=0.6, source="manuel"),
                            "manuel", True)
    assert sortie.vitesse == 0.0  # l'arrêt d'urgence reste maître
