"""Remorque attelée : angle d'attelage (modèle, estimation), emprise et règles de sécurité."""

from __future__ import annotations

import math
import time

import numpy as np
import pytest
from test_securite import bus_sain

from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.controle.remorque import (
    GeometrieRemorque,
    SuiviRemorque,
    angle_equilibre,
    angles_le_long,
    derivee_angle,
    distance_libre_remorque,
    emprise_monde,
)
from fieldmind.mission.actions import ActionRefusee
from fieldmind.securite.superviseur import Superviseur
from fieldmind.types import Consigne, Odometrie, Pose2D


@pytest.fixture
def profil() -> Profil:
    return Profil.charger(reglages=None)


@pytest.fixture
def attele(profil: Profil) -> Profil:
    profil.regler("remorque.attelee", True)
    return profil


def geometrie(profil: Profil) -> GeometrieRemorque:
    return GeometrieRemorque.depuis(profil)


def test_cotes_de_la_cao(profil: Profil) -> None:
    g = geometrie(profil)
    # Vis d'attelage à Y = 540 mm, essieu arrière du rover à Y = 380, essieu de la remorque à 850
    assert g.attelage_m == pytest.approx(0.16) and g.timon_m == pytest.approx(0.31)
    assert g.arriere_m - g.avant_m == pytest.approx(0.38)  # planche de 380 mm
    profil.regler("remorque.volets_deplies", True)
    assert geometrie(profil).demi_largeur_m > g.demi_largeur_m


def test_virage_etabli_sous_l_angle_de_contact(profil: Profil) -> None:
    """Braquage maximal en marche avant : l'angle se stabilise loin du contact (52°)."""
    g = geometrie(profil)
    kappa = math.tan(math.radians(25.8)) / profil.geometrie.empattement_m
    phi = 0.0
    for _ in range(4000):  # 20 s à 0,3 m/s
        phi += 0.005 * derivee_angle(phi, 0.3, 0.3 * kappa, g)
    assert phi == pytest.approx(angle_equilibre(kappa, g), abs=1e-3)
    assert math.radians(25) < phi < g.angle_contact - math.radians(10)


def test_marche_avant_stable_marche_arriere_instable(profil: Profil) -> None:
    g = geometrie(profil)
    s_av = np.arange(0.04, 1.0, 0.04)
    assert abs(angles_le_long(math.radians(10), 0.0, s_av, g)[-1]) < math.radians(0.5)
    phis_ar = angles_le_long(math.radians(5), 0.0, -s_av, g)
    assert abs(phis_ar[-1]) > math.radians(5) * 10  # tout droit en reculant : elle part


def test_estimation_converge_en_avancant(attele: Profil) -> None:
    bus = Bus()
    suivi = SuiviRemorque(bus, attele)
    suivi.etat.angle = math.radians(30)  # faux départ : la remorque est en fait dans l'axe
    t = 100.0
    for _ in range(60):  # 1,2 m à 0,3 m/s, tout droit
        t += 0.05
        bus.publier(S.ODOM, Odometrie(t=t, pose=Pose2D(), vitesse=0.3, vitesse_lacet=0.0))
    assert abs(math.degrees(suivi.etat.angle)) < 2.0  # 30° → < 2° en 1,2 m
    assert math.degrees(suivi.etat.incertitude) < 8.0
    incertitude = suivi.etat.incertitude
    for _ in range(20):  # 0,3 m en reculant : l'incertitude grandit
        t += 0.05
        bus.publier(S.ODOM, Odometrie(t=t, pose=Pose2D(), vitesse=-0.3, vitesse_lacet=0.0))
    assert suivi.etat.incertitude > incertitude * 2


def superviseur(bus: Bus, profil: Profil, angle_deg: float = 0.0,
                incertitude_deg: float = 2.0) -> Superviseur:
    sup = Superviseur(bus, profil)
    sup.operateurs = 1
    suivi = SuiviRemorque(bus, profil)
    suivi.etat.angle = math.radians(angle_deg)
    suivi.etat.incertitude = math.radians(incertitude_deg)
    sup.remorque = suivi
    return sup


def evaluer(sup: Superviseur, vitesse: float, braquage: float = 0.0,
            mode: str = "manuel") -> tuple[Consigne, object]:
    return sup.evaluer(Consigne(t=time.monotonic(), vitesse=vitesse, braquage=braquage,
                                source="manuel"), mode, True)  # type: ignore[arg-type]


def test_sans_remorque_rien_ne_change(profil: Profil) -> None:
    sup = superviseur(bus_sain([(8.0, 3.0), (-0.95, 0.0)]), profil)
    sortie, etat = evaluer(sup, -0.12)
    assert sortie.vitesse == pytest.approx(-0.12) and "remorque" not in etat.raisons


def test_obstacle_derriere_la_remorque(attele: Profil) -> None:
    """À 0,3 m derrière la planche : le rover seul pourrait reculer, pas avec la remorque."""
    g = geometrie(attele)
    derriere = -(g.attelage_m + g.arriere_m + 0.3)
    sup = superviseur(bus_sain([(8.0, 3.0), (derriere, 0.0)]), attele)
    sortie, etat = evaluer(sup, -0.12)
    assert sortie.vitesse == 0.0 and "obstacle_arriere" in etat.raisons
    sup = superviseur(bus_sain([(8.0, 3.0), (derriere - 1.5, 0.0)]), attele)
    sortie, _ = evaluer(sup, -0.12)
    assert sortie.vitesse == pytest.approx(-0.12)


def test_vitesses_plafonnees(attele: Profil) -> None:
    sortie, etat = evaluer(superviseur(bus_sain(), attele), 0.6)
    assert sortie.vitesse == pytest.approx(0.4) and "remorque" in etat.raisons
    sortie, _ = evaluer(superviseur(bus_sain(), attele), -0.3)
    assert sortie.vitesse == pytest.approx(-0.12)


def test_recul_refuse_si_angle_incertain_puis_permis_dans_l_axe(attele: Profil) -> None:
    bus = bus_sain()
    sup = superviseur(bus, attele, incertitude_deg=25.0)
    sortie, etat = evaluer(sup, -0.1)
    assert sortie.vitesse == 0.0 and "remorque_incertaine" in etat.raisons
    sup.remorque.aligner()
    sortie, _ = evaluer(sup, -0.1)
    assert sortie.vitesse == pytest.approx(-0.1)


def test_mise_en_portefeuille_evitee(attele: Profil) -> None:
    """φ = 17° (remorque à gauche) : reculer en braquant à droite l'ouvre encore, refusé ;
    braquer à gauche la referme, permis."""
    bus = bus_sain()
    sortie, etat = evaluer(superviseur(bus, attele, angle_deg=17), -0.1, math.radians(-20))
    assert sortie.vitesse == 0.0 and "remorque_portefeuille" in etat.raisons
    sortie, _ = evaluer(superviseur(bus, attele, angle_deg=17), -0.1, math.radians(20))
    assert sortie.vitesse == pytest.approx(-0.1)


def test_pas_de_recul_en_automatique(attele: Profil) -> None:
    sortie, etat = evaluer(superviseur(bus_sain(), attele), -0.1, mode="auto")
    assert sortie.vitesse == 0.0 and "remorque_recul_auto" in etat.raisons


def test_points_sur_la_remorque_ignores(attele: Profil) -> None:
    """Un point dans l'emprise actuelle (la remorque elle-même, son drone) ne bloque rien."""
    g = geometrie(attele)
    sur = np.array([[-(g.attelage_m + 0.3), 0.05]], np.float32)
    libre, indice = distance_libre_remorque(sur, 0.0, +1, 0.0, attele, g)
    assert indice == -1 and libre == pytest.approx(1.5)


def test_emprise_affichee(attele: Profil) -> None:
    g = geometrie(attele)
    coins = emprise_monde(0.0, 0.0, 0.0, 0.0, g)
    xs = [c[0] for c in coins]
    assert min(xs) == pytest.approx(-(g.attelage_m + g.arriere_m))
    # φ > 0 : remorque partie à gauche derrière
    assert np.mean([c[1] for c in emprise_monde(0.0, 0.0, 0.0, math.radians(30), g)]) > 0


def test_executeur_refuse_le_recul_automatique(attele: Profil) -> None:
    from fieldmind.app import Rover

    rover = Rover(attele, simulation=True)
    rover.conduite.arme = True
    with pytest.raises(ActionRefusee, match="remorque"):
        rover.executeur.ajouter([{"type": "avancer", "distance_m": -0.5}])
