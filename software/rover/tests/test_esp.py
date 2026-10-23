"""Pilote ESP contre le faux ESP de la simulation (même protocole texte que le firmware)."""

from __future__ import annotations

import threading
import time

import numpy as np
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


def test_moteurs_permutes(profil: Profil) -> None:
    """Moteurs branchés à l'envers : la commande gauche part sur la sortie droite, et l'état
    publié parle toujours de la vraie roue gauche."""
    profil.regler("esp.moteurs_permutes", True)
    bus, faux, pilote, horloge = demarrer("ROVER_3", profil)
    try:
        fin = time.monotonic() + 1.5
        while time.monotonic() < fin and faux.pwm[1] != 12:
            pilote.appliquer(CommandeRoues(t=time.monotonic(), pwm_g=12, pwm_d=5, servo_deg=0))
            time.sleep(0.05)
        assert faux.pwm[1] == 12 and faux.pwm[0] == 5  # sorties échangées
        attendre(lambda: bus.dernier(S.ESP) is not None and bus.dernier(S.ESP).pwm_g == 12)
        assert bus.dernier(S.ESP).pwm_d == 5  # mesures remises dans l'ordre des vraies roues
    finally:
        horloge.arreter()
        pilote.arreter()


def test_sens_inverse(profil: Profil) -> None:
    """Avancer faisait reculer : les sorties partent retournées, l'état publié parle du vrai
    sens des roues."""
    profil.regler("esp.sens_inverse", True)
    bus, faux, pilote, horloge = demarrer("ROVER_3", profil)
    try:
        fin = time.monotonic() + 1.5
        while time.monotonic() < fin and faux.pwm[0] != -12:
            pilote.appliquer(CommandeRoues(t=time.monotonic(), pwm_g=12, pwm_d=-5, servo_deg=0))
            time.sleep(0.05)
        assert faux.pwm[0] == -12 and faux.pwm[1] == 5  # sorties retournées
        attendre(lambda: bus.dernier(S.ESP) is not None and bus.dernier(S.ESP).pwm_g == 12)
        etat = bus.dernier(S.ESP)
        assert etat.sens_g == 0 and etat.sens_d == 1  # vrai sens : gauche avant, droite arrière
    finally:
        horloge.arreter()
        pilote.arreter()


def test_preparation_rover3_laissee_passer(profil: Profil, monkeypatch: pytest.MonkeyPatch) -> None:
    """ROVER_3 relance la préparation (100 ms) d'une roue qui démarre à chaque nouvelle ligne p :
    pendant ce temps, seulement des battements ; un arrêt part toujours tout de suite."""
    _bus, _faux, pilote, horloge = demarrer("ROVER_3", profil)
    horloge.arreter()
    t = [100.0]
    monkeypatch.setattr(time, "monotonic", lambda: t[0])
    try:
        assert pilote._ligne_rover3(20, 20, 0.0).startswith("p 20 20")  # départ
        t[0] += 0.05
        assert pilote._ligne_rover3(22, 22, 0.0) == "hb"  # préparation en cours
        t[0] += 0.12
        assert pilote._ligne_rover3(24, 24, 0.0).startswith("p 24 24")  # roues lancées
        t[0] += 0.05
        assert pilote._ligne_rover3(26, 26, 0.0).startswith("p 26 26")
        assert pilote._ligne_rover3(-20, -20, 0.0).startswith("p -20 -20")  # inversion
        t[0] += 0.5
        assert pilote._ligne_rover3(-20, -20, 0.0) == "hb"  # inv ms à zéro + préparation
        assert pilote._ligne_rover3(0, 0, 0.0).startswith("p 0 0")  # arrêt immédiat
    finally:
        pilote.arreter()


def test_volets_rotation_continue(profil: Profil) -> None:
    """Servos à rotation continue : rien au démarrage ; déplier = tourner (droit en miroir)
    pendant la durée réglée puis couper le signal ; même état demandé deux fois : rien."""
    profil.regler("remorque.volets_vitesse", 50)
    profil.regler("remorque.volets_tours_min", 150)  # 450 °/s à 50 % : 90° en 0,2 s
    _bus, faux, pilote, horloge = demarrer("ROVER_4", profil)
    lignes: list[str] = []
    ecrire = faux.write

    def espion(donnees: bytes) -> int:
        lignes.extend(lg for lg in donnees.decode().splitlines() if lg.startswith("vol"))
        return ecrire(donnees)

    faux.write = espion  # type: ignore[method-assign]
    try:
        pilote.appliquer(CommandeRoues(t=time.monotonic(), pwm_g=0, pwm_d=0, servo_deg=0))
        time.sleep(0.1)
        assert lignes == []  # aucune position imposée au démarrage
        debut = time.monotonic()
        assert pilote.mouvoir_volets(True)
        assert time.monotonic() - debut >= 0.2
        assert lignes == ["volg 135", "vold 45", "volg -1", "vold -1"]
        assert faux.volets == [None, None]  # arrêtés
        assert not pilote.mouvoir_volets(True)  # déjà dépliés : pas de rotation en butée
        assert pilote.mouvoir_volets(False) and lignes[-4:-2] == ["volg 45", "vold 135"]
    finally:
        horloge.arreter()
        pilote.arreter()
    _bus, faux3, pilote3, horloge3 = demarrer("ROVER_3", profil)
    try:
        assert not pilote3.mouvoir_volets(True) and faux3.volets == [None, None]
    finally:
        horloge3.arreter()
        pilote3.arreter()


def test_mise_a_niveau_de_la_centrale(profil: Profil) -> None:
    """GY-521 monté de biais : la gravité mesurée au repos ramenée sur +z (roulis et tangage
    nuls à plat), et le lacet du firmware (autour de l'axe penché) n'est plus utilisé."""
    from fieldmind.materiel.esp import rotation_niveau

    g = (-0.79, 0.416, -0.475)
    r = rotation_niveau(g)
    assert np.allclose(r @ (np.array(g) / np.linalg.norm(g)), [0, 0, 1], atol=1e-9)
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-9)
    profil.regler("imu.niveau", " ".join(str(x) for x in g))
    bus, _faux, pilote, horloge = demarrer("ROVER_3", profil)
    try:
        pilote._publier_imu(time.monotonic(), g, (0.0, 0.0, 0.0), 12.0, None, "OK")
        imu = bus.dernier(S.IMU)
        roulis, tangage = imu.roulis_tangage()
        assert abs(roulis) < 0.01 and abs(tangage) < 0.01 and imu.lacet_rad is None
    finally:
        horloge.arreter()
        pilote.arreter()
