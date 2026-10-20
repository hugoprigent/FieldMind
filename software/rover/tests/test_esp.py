"""Pilote ESP contre le faux ESP de la simulation (même protocole texte que le firmware)."""

from __future__ import annotations

import threading
import time

import pytest

from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.materiel.esp import PWM_MAX_SANS_CHIEN, PiloteEsp
from fieldmind.simulation.materiel_simule import FauxEsp
from fieldmind.types import CommandeRoues


class Horloge:
    """Fait avancer le faux firmware à 100 Hz dans un fil, comme la simulation."""

    def __init__(self, esp: FauxEsp) -> None:
        self.esp = esp
        self.actif = True
        self.fil = threading.Thread(target=self._tourner, daemon=True)
        self.fil.start()

    def _tourner(self) -> None:
        while self.actif:
            self.esp.tick(time.monotonic(), 0.01)
            time.sleep(0.01)

    def arreter(self) -> None:
        self.actif = False


@pytest.fixture
def profil() -> Profil:
    return Profil.charger(reglages=None)


def demarrer(version: str, profil: Profil) -> tuple[Bus, FauxEsp, PiloteEsp, Horloge]:
    bus = Bus()
    faux = FauxEsp(version)
    pilote = PiloteEsp(bus, profil, faux, drone_reel=False)
    horloge = Horloge(faux)
    pilote.demarrer()
    attendre(lambda: pilote.version == version)
    return bus, faux, pilote, horloge


def attendre(condition, delai: float = 2.0) -> None:
    fin = time.monotonic() + delai
    while time.monotonic() < fin:
        if condition():
            return
        time.sleep(0.01)
    raise AssertionError("condition non atteinte")


def test_rover3_configure_chien_et_telemetrie(profil: Profil) -> None:
    bus, faux, pilote, horloge = demarrer("ROVER_3", profil)
    try:
        pilote.appliquer(CommandeRoues(t=time.monotonic(), pwm_g=12, pwm_d=-8, servo_deg=5.5))
        assert faux.chien_ms == 400 and faux.tel_hz == 20 and faux.imu_hz == 50
        assert faux.pwm[0] == 12
        assert faux.pwm[1] == 0 and faux.attente_inversion[1] is not None  # passe par zéro
        fin = time.monotonic() + 1.2  # la conduite renvoie la consigne à 20 Hz
        while time.monotonic() < fin and faux.pwm[1] != -8:
            pilote.appliquer(CommandeRoues(t=time.monotonic(), pwm_g=12, pwm_d=-8, servo_deg=5.5))
            time.sleep(0.05)
        assert faux.pwm[1] == -8  # inversé après inv ms à zéro
        attendre(lambda: bus.dernier(S.ESP) is not None and bus.dernier(S.ESP).pwm_d == 8)
        etat = bus.dernier(S.ESP)
        assert etat.sens_d == 1 and etat.connecte
        attendre(lambda: getattr(bus.dernier(S.IMU), "lacet_rad", None) is not None)
    finally:
        horloge.arreter()
        pilote.arreter()


def test_chien_de_garde_coupe_la_traction(profil: Profil) -> None:
    _bus, faux, pilote, horloge = demarrer("ROVER_3", profil)
    try:
        pilote.appliquer(CommandeRoues(t=time.monotonic(), pwm_g=10, pwm_d=10))
        assert faux.pwm == [10, 10]
        time.sleep(0.6)  # le Pi se tait plus que 400 ms
        assert faux.pwm == [0, 0] and faux.chien_expire
    finally:
        horloge.arreter()
        pilote.arreter()


def test_rover2_sequence_inversion_et_plafond(profil: Profil) -> None:
    _bus, faux, pilote, horloge = demarrer("ROVER_2", profil)
    try:
        assert not pilote.rover3
        pilote.appliquer(CommandeRoues(t=time.monotonic(), pwm_g=35, pwm_d=10))
        assert faux.demande == [PWM_MAX_SANS_CHIEN, 10]
        # Inversion de la roue gauche : d'abord zéro, puis sens après le délai, puis la valeur
        pilote.appliquer(CommandeRoues(t=time.monotonic(), pwm_g=-10, pwm_d=10))
        assert faux.demande[0] == 0 and faux.sens[0] == 0
        pilote.appliquer(CommandeRoues(t=time.monotonic(), pwm_g=-10, pwm_d=10))
        assert faux.sens[0] == 0  # délai d'inversion pas encore écoulé
        time.sleep(float(profil.valeur("traction.delai_inversion_s")) + 0.05)
        pilote.appliquer(CommandeRoues(t=time.monotonic(), pwm_g=-10, pwm_d=10))
        assert faux.sens[0] == 1 and faux.demande[0] == 10 and faux.pwm[0] == -10
    finally:
        horloge.arreter()
        pilote.arreter()


def test_arret_traction_immediat(profil: Profil) -> None:
    _bus, faux, pilote, horloge = demarrer("ROVER_3", profil)
    try:
        pilote.appliquer(CommandeRoues(t=time.monotonic(), pwm_g=15, pwm_d=15))
        pilote.arret_traction()
        assert faux.pwm == [0, 0]
    finally:
        horloge.arreter()
        pilote.arreter()


def test_centrale_absente_ignoree(profil: Profil) -> None:
    bus = Bus()
    pilote = PiloteEsp(bus, profil, FauxEsp("ROVER_2"), drone_reel=False)
    pilote._traiter("IMU ABSENT  a=[+0.00 +0.00 +0.00] g  w=[   +0.0    +0.0    +0.0] deg/s   0.0 C"
                    " | BARO DOUTEUX  1013.20 hPa  22.0 C | KNOB ABSENT pos    +0        | Accueil")
    assert bus.dernier(S.IMU) is None
    pilote._traiter("IMU OK      a=[+0.01 -0.02 +0.99] g  w=[   +0.1    +0.0    +1.5] deg/s  31.0 C"
                    " | BARO OK      1013.20 hPa  22.0 C | KNOB ABSENT pos    +0        | Accueil")
    assert bus.dernier(S.IMU) is not None
