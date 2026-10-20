"""Conversions du pont ROS 2 (software/rover/ros2), sans ROS installé."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "ros2" / "src" / "fieldmind_rover_ros"))

from fieldmind_rover_ros import conversions as c

# ---------------------------------------------------------------- adresse et temps


@pytest.mark.parametrize(("adresse", "jeton", "attendu"), [
    ("rover.local:8080", "", "ws://rover.local:8080/ws/ros"),
    ("rover.local", "", "ws://rover.local:8080/ws/ros"),
    ("rover.local:9000", "abc", "ws://rover.local:9000/ws/ros?jeton=abc"),
    ("rover.local:8080", "a b&c", "ws://rover.local:8080/ws/ros?jeton=a%20b%26c"),
    ("ws://h:1", "", "ws://h:1/ws/ros"),
    ("http://h:1/ws/ros", "", "ws://h:1/ws/ros"),
    ("https://h/autre", "j", "wss://h/autre?jeton=j"),
    ("[::1]", "", "ws://[::1]:8080/ws/ros"),
    ("[::1]:81", "", "ws://[::1]:81/ws/ros"),
])
def test_construire_url(adresse: str, jeton: str, attendu: str) -> None:
    assert c.construire_url(adresse, jeton) == attendu


def test_url_invalide_et_jeton_masque() -> None:
    with pytest.raises(ValueError, match="schéma"):
        c.construire_url("ftp://h")
    with pytest.raises(ValueError, match="vide"):
        c.construire_url("  ")
    url = c.construire_url("h:1", "secret")
    assert "secret" not in c.masquer_jeton(url)
    assert c.masquer_jeton("ws://h:1/ws/ros") == "ws://h:1/ws/ros"


def test_secondes_vers_stamp() -> None:
    assert c.secondes_vers_stamp(1.5) == (1, 500_000_000)
    assert c.secondes_vers_stamp(2.9999999999) == (3, 0)
    assert c.secondes_vers_stamp(1_760_000_000.123456789)[1] == pytest.approx(123_456_789, abs=300)
    with pytest.raises(ValueError, match="date"):
        c.secondes_vers_stamp(-1.0)
    with pytest.raises(ValueError, match="date"):
        c.secondes_vers_stamp(math.nan)


# ---------------------------------------------------------------- recalage d'horloge


def test_recalage_latence_minimale_et_retards_ignores() -> None:
    rng = np.random.default_rng(1)
    horloge = c.RecaleurHorloge()
    decalage = 1_760_000_000.0  # horloge ROS (murale) − monotone du Pi
    for i in range(300):  # 30 s à 10 Hz
        t_envoi = 5000.0 + 0.1 * i
        latence = 0.003 + rng.exponential(0.01)
        if i % 37 == 0:
            latence += 0.8  # paquet bloqué par le Wi-Fi
        horloge.observer(t_envoi, t_envoi + decalage + latence)
    assert horloge.decalage == pytest.approx(decalage + 0.003, abs=0.003)
    # Date ROS d'une mesure du Pi, jamais dans le futur.
    maintenant = 5030.0 + decalage
    assert horloge.vers_ros(5029.9, maintenant) == pytest.approx(5029.9 + decalage, abs=0.01)
    assert horloge.vers_ros(5031.0, maintenant) == maintenant


def test_recalage_sans_estimation_et_remise_a_zero() -> None:
    horloge = c.RecaleurHorloge()
    assert horloge.decalage is None
    assert horloge.vers_ros(12.0, 99.0) == 99.0
    horloge.observer(100.0, 1100.01)
    assert horloge.decalage == pytest.approx(1000.01)
    # Le Pi redémarre : son horloge monotone repart de presque zéro.
    horloge.observer(3.0, 1200.0)
    assert horloge.remises_a_zero == 1
    assert horloge.decalage == pytest.approx(1197.0)


def test_recalage_saut_d_horloge_locale_et_fenetre() -> None:
    horloge = c.RecaleurHorloge(fenetre_s=20.0, saut_s=5.0)
    for i in range(100):
        horloge.observer(10.0 + 0.1 * i, 1010.0 + 0.1 * i + 0.01)
    assert horloge.decalage == pytest.approx(1000.01)
    # NTP avance l'horloge de cette machine de 30 s : nouvelle estimation tout de suite.
    horloge.observer(20.1, 1050.11)
    assert horloge.decalage == pytest.approx(1030.01)
    # Petite dérive vers le haut (+50 ms) : adoptée une fois la fenêtre écoulée.
    for i in range(300):
        horloge.observer(20.2 + 0.1 * i, 1050.2 + 0.1 * i + 0.06)
    assert horloge.decalage == pytest.approx(1030.06)


# ---------------------------------------------------------------- lidar


def test_float32_aller_retour_et_erreurs() -> None:
    valeurs = np.array([0.0, -1.5, 3.25, 1e-3], dtype=np.float32)
    assert np.array_equal(c.decoder_float32(c.encoder_float32(valeurs)), valeurs)
    with pytest.raises(c.MessageInvalide):
        c.decoder_float32("pas du base64 !")
    with pytest.raises(c.MessageInvalide, match="multiple"):
        c.decoder_float32("AAAA")  # 3 octets


def test_scan_uniforme_secteurs_et_bouclage() -> None:
    n = 360
    angles = np.array([-math.pi, -math.pi / 2, 0.0, 0.004, math.pi / 2, math.pi - 1e-4])
    distances = np.array([4.0, 1.0, 2.0, 1.5, 3.0, 3.5])
    angle_min, pas, rayons = c.scan_uniforme(angles, distances, n)
    assert angle_min == -math.pi
    assert pas == pytest.approx(2 * math.pi / n)
    assert rayons.dtype == np.float32
    assert rayons.shape == (n,)
    assert rayons[90] == pytest.approx(1.0)  # −90°
    assert rayons[180] == pytest.approx(1.5)  # 0° et 0,23° : le plus proche l'emporte
    assert rayons[270] == pytest.approx(3.0)
    assert rayons[0] == pytest.approx(3.5)  # −180° et +179,99° : même secteur
    assert np.isfinite(rayons).sum() == 4
    assert np.isinf(rayons[1:90]).all()  # secteurs vides : pas d'écho


def test_scan_uniforme_ignore_les_mesures_invalides() -> None:
    angles = np.array([0.0, 0.5, 1.0, math.nan])
    distances = np.array([math.nan, 0.0, -2.0, 1.0])
    _, _, rayons = c.scan_uniforme(angles, distances, 100)
    assert np.isinf(rayons).all()
    with pytest.raises(c.MessageInvalide):
        c.scan_uniforme(np.zeros(3), np.zeros(4), 100)
    with pytest.raises(ValueError, match="nb_rayons"):
        c.scan_uniforme(np.zeros(3), np.ones(3), 4)


def test_scan_uniforme_lidar_decale_reprojete_les_points() -> None:
    rng = np.random.default_rng(3)
    x = rng.uniform(-4, 4, 200)
    y = rng.uniform(-4, 4, 200)
    garde = np.hypot(x, y) > 1.0
    x, y = x[garde], y[garde]
    angles, distances = np.arctan2(y, x), np.hypot(x, y)  # autour de l'origine de base_link
    lidar = (0.10, -0.05)
    angle_min, pas, rayons = c.scan_uniforme(angles, distances, 3600, lidar)
    indices = np.flatnonzero(np.isfinite(rayons))
    a = angle_min + indices * pas
    px = rayons[indices] * np.cos(a) + lidar[0]
    py = rayons[indices] * np.sin(a) + lidar[1]
    # Chaque rayon reconstitué retombe sur un des points d'origine (à l'arrondi du secteur près).
    ecarts = np.hypot(px[:, None] - x[None, :], py[:, None] - y[None, :]).min(axis=1)
    assert ecarts.max() < 6.0 * pas
    assert len(indices) > 150


# ---------------------------------------------------------------- géométrie et consignes


@pytest.mark.parametrize("cap", [0.0, 0.3, -1.2, math.pi / 2, 3.1, -3.1])
def test_quaternion_du_cap(cap: float) -> None:
    q = c.quaternion_cap(cap)
    assert math.fsum(v * v for v in q) == pytest.approx(1.0)
    assert c.cap_quaternion(*q) == pytest.approx(cap)


def test_cap_d_un_quaternion_incline() -> None:
    assert c.quaternion_cap(math.pi / 2) == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)))
    # lacet 0,7 composé avec un tangage de 0,2 (q = q_lacet · q_tangage)
    lacet, tangage = 0.7, 0.2
    cz, sz = math.cos(lacet / 2), math.sin(lacet / 2)
    cy, sy = math.cos(tangage / 2), math.sin(tangage / 2)
    q = (-sz * sy, cz * sy, sz * cy, cz * cy)
    assert c.cap_quaternion(*q) == pytest.approx(lacet)


def test_twist_vers_cmd_vel() -> None:
    assert c.twist_vers_cmd_vel(0.3, 0.2) == {"type": "cmd_vel", "vitesse": 0.3,
                                             "vitesse_lacet": 0.2}
    # Trop vite : v et ω réduits du même facteur, la courbure ω/v (le braquage) est gardée.
    m = c.twist_vers_cmd_vel(1.0, 1.0)
    assert (m["vitesse"], m["vitesse_lacet"]) == (0.5, 0.5)
    m = c.twist_vers_cmd_vel(-0.6, 0.3)
    assert (m["vitesse"], m["vitesse_lacet"]) == (-0.3, 0.15)
    m = c.twist_vers_cmd_vel(0.2, 3.0)
    assert m["vitesse_lacet"] == 1.0
    assert m["vitesse"] == pytest.approx(0.0667, abs=1e-4)
    limites = c.LimitesConsigne(vitesse_max=0.2, vitesse_arriere_max=0.1, vitesse_lacet_max=0.5)
    assert c.twist_vers_cmd_vel(0.4, 0.0, limites)["vitesse"] == 0.2
    for invalide in (math.nan, math.inf, "x", None):
        assert c.twist_vers_cmd_vel(invalide, 0.0) is None
        assert c.twist_vers_cmd_vel(0.1, invalide) is None
    arret = c.twist_vers_cmd_vel(-0.0, -0.0)
    assert json.dumps(arret) == '{"type": "cmd_vel", "vitesse": 0.0, "vitesse_lacet": 0.0}'


def test_but_dans_odom() -> None:
    assert c.but_vers_message(2.0, -1.0) == {"type": "but", "x": 2.0, "y": -1.0}
    # But donné dans map ; pose de map dans odom : (1, 2), tournée de 90°.
    assert c.but_vers_message(1.0, 0.0, (1.0, 2.0, math.pi / 2)) == {"type": "but", "x": 1.0,
                                                                     "y": 3.0}
    with pytest.raises(ValueError, match="fini"):
        c.but_vers_message(math.nan, 0.0)


# ---------------------------------------------------------------- covariances, GNSS, caméra


def test_covariance_pose() -> None:
    cov9 = [0.04, 0.01, 0.002, 0.01, 0.09, 0.003, 0.002, 0.003, 0.0004]
    cov = c.covariance_pose(cov9, (1, 1, 1), 1e-4)
    assert len(cov) == 36
    assert (cov[0], cov[1], cov[5], cov[7], cov[11], cov[35]) == (0.04, 0.01, 0.002, 0.09,
                                                                  0.003, 0.0004)
    assert cov[30] == 0.002  # symétrique : lacet / x
    assert cov[14] == cov[21] == cov[28] == 1e-4  # z, roulis, tangage
    defaut = c.covariance_pose(None, (0.01, 0.02, 0.003), 1e-4)
    assert (defaut[0], defaut[7], defaut[35], defaut[1]) == (0.01, 0.02, 0.003, 0.0)
    assert c.covariance_pose([1.0] * 8, (0.5, 0.5, 0.5), 0.0)[0] == 0.5
    assert c.covariance_pose([-1.0] + [0.0] * 8, (0.5, 0.5, 0.5), 0.0)[0] == 0.5


def test_navsat() -> None:
    r = c.Reglages()
    statut, cov, genre = c.navsat("autonome", 1.2, r)
    assert (statut, genre) == (c.STATUT_FIX, c.COVARIANCE_APPROCHEE)
    assert cov[0] == pytest.approx(9.0)  # σ = 1,2 × 2,5 m
    assert cov[8] == pytest.approx(36.0)
    assert c.navsat("autonome", 0.4, r)[1][0] == pytest.approx(1.5 ** 2)  # plancher
    assert c.navsat("autonome", None, r)[1][0] == pytest.approx(25.0)
    assert c.navsat("dgnss", 1.0, r)[0] == c.STATUT_SBAS
    assert c.navsat("aucun", 1.0, r) == (c.STATUT_SANS_FIX, [0.0] * 9, c.COVARIANCE_INCONNUE)
    assert c.navsat("inconnu", 1.0, r)[0] == c.STATUT_SANS_FIX


def jpeg_minimal(largeur: int, hauteur: int) -> bytes:
    """En-têtes d'un JPEG (SOI, APP0, SOF0) : assez pour lire ses dimensions."""
    app0 = b"\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00"
    sof0 = (b"\xff\xc0\x00\x11\x08" + hauteur.to_bytes(2, "big") + largeur.to_bytes(2, "big")
            + b"\x03\x01\x22\x00\x02\x11\x01\x03\x11\x01")
    return b"\xff\xd8" + app0 + sof0 + b"\xff\xda\x00\x08" + bytes(20) + b"\xff\xd9"


def test_dimensions_jpeg_et_camera() -> None:
    assert c.dimensions_jpeg(jpeg_minimal(1280, 720)) == (1280, 720)
    assert c.dimensions_jpeg(b"\xff\xd8\xff\xff" + jpeg_minimal(64, 48)[2:]) == (64, 48)
    assert c.dimensions_jpeg(b"\x89PNG....") is None
    assert c.dimensions_jpeg(b"\xff\xd8\xff\xda\x00\x02") is None
    info = c.info_camera(640, 480, 90.0)
    assert info.k[0] == pytest.approx(320.0)
    assert info.k[2] == pytest.approx(319.5)
    assert info.p[5] == info.k[4]
    assert c.nom_camera("Gauche") == "gauche"
    with pytest.raises(c.MessageInvalide):
        c.nom_camera("../")


ETAT = {
    "type": "etat", "t": 1234.5, "mode": "auto", "arme": True, "arret_urgence": False,
    "consigne": {"vitesse": 0.3, "braquage": 0.1, "source": "ros2"},
    "securite": {"arret": False, "vitesse_max_avant": 0.45, "raisons": ["stereo_absente"],
                 "libre_avant_m": 1.8},
    "mission": {"etat": "attente"},
    "capteurs": {"lidar": {"ok": True, "hz": 10.2}, "stereo": {"ok": False}, "micro": None},
}


def test_diagnostics_etat() -> None:
    diagnostics = c.diagnostics_etat(ETAT)
    rover = diagnostics[0]
    assert rover.nom == "fieldmind/rover"
    assert rover.niveau == c.ALERTE
    assert "stereo_absente" in rover.message
    assert ("source_consigne", "ros2") in rover.valeurs
    assert ("arme", "oui") in rover.valeurs
    capteurs = {d.nom: d.niveau for d in diagnostics[1:]}
    assert capteurs == {"fieldmind/capteurs/lidar": c.OK, "fieldmind/capteurs/stereo": c.ALERTE}
    assert c.diagnostics_etat({"arret_urgence": True})[0].niveau == c.ERREUR
    assert c.diagnostics_etat({"mode": "manuel"})[0].niveau == c.OK
    assert c.diagnostics_etat({"securite": {"arret": True}})[0].niveau == c.ALERTE


# ---------------------------------------------------------------- décodage des messages


DECALAGE = 1_760_000_000.0


def test_decodeur_scan_date_recalee() -> None:
    d = c.Decodeur(c.Reglages(nb_rayons=360))
    message = {"type": "scan", "t": 100.0, "t_envoi": 100.02,
               "angles_b64": c.encoder_float32([0.0, 1.0]),
               "distances_b64": c.encoder_float32([2.0, 3.0])}
    s = d.decoder(message, 100.025 + DECALAGE)
    assert isinstance(s, c.ScanRos)
    assert s.stamp == pytest.approx(100.0 + DECALAGE + 0.005, abs=1e-6)
    assert s.angle_max == pytest.approx(math.pi - 2 * math.pi / 360)
    assert (s.portee_min, s.portee_max, s.duree) == (0.12, 12.0, 0.1)
    assert s.distances[180] == pytest.approx(2.0)
    assert d.decoder({**message, "duree": 0.125}, 100.1 + DECALAGE).duree == 0.125


def test_decodeur_imu_odom_gnss_image_etat() -> None:
    d = c.Decodeur()
    t = 50.0 + DECALAGE
    imu = d.decoder({"type": "imu", "t": 50.0, "accel_g": [0.0, 0.1, 1.0],
                     "gyro_rad_s": [0.0, 0.0, 0.5]}, t)
    assert isinstance(imu, c.ImuRos)
    assert imu.acceleration[2] == pytest.approx(c.G)
    assert imu.vitesse_angulaire == (0.0, 0.0, 0.5)
    assert imu.covariance_vitesse_angulaire[0] == 1e-4

    odom = d.decoder({"type": "odom", "t": 50.0, "x": 1.0, "y": 2.0, "cap": math.pi / 2,
                      "vitesse": 0.3, "vitesse_lacet": 0.1, "covariance": None}, t)
    assert isinstance(odom, c.OdomRos)
    assert odom.quaternion[2] == pytest.approx(math.sqrt(0.5))
    assert (odom.vitesse, odom.vitesse_lacet) == (0.3, 0.1)
    assert odom.covariance_pose[0] == 0.01
    assert odom.covariance_vitesse[0] == 0.0025

    gnss = d.decoder({"type": "gnss", "t": 49.5, "fix": "autonome", "latitude": 43.71,
                      "longitude": -1.05, "altitude_m": 12.0, "hdop": 1.0}, t)
    assert isinstance(gnss, c.GnssRos)
    assert (gnss.statut, gnss.latitude, gnss.altitude) == (c.STATUT_FIX, 43.71, 12.0)
    assert gnss.service == c.SERVICE_GPS | c.SERVICE_COMPASS
    sans = d.decoder({"type": "gnss", "t": 49.5, "fix": "autonome", "latitude": None,
                      "longitude": None}, t)
    assert sans.statut == c.STATUT_SANS_FIX
    assert math.isnan(sans.latitude)

    import base64

    image = d.decoder({"type": "image", "t": 49.9, "camera": "droite",
                       "jpeg_b64": base64.b64encode(jpeg_minimal(640, 360)).decode()}, t)
    assert isinstance(image, c.ImageRos)
    assert (image.camera, image.largeur, image.hauteur) == ("droite", 640, 360)

    etat = d.decoder(ETAT, t)
    assert isinstance(etat, c.EtatRos)
    assert json.loads(etat.json)["mode"] == "auto"
    assert d.decoder({"type": "journal", "t": 1.0}, t) is None


def test_decodeur_messages_invalides() -> None:
    d = c.Decodeur()
    for message in (
        {"type": "scan", "t": 1.0, "angles_b64": c.encoder_float32([0.0])},
        {"type": "scan", "t": 1.0, "angles_b64": c.encoder_float32([0.0, 1.0]),
         "distances_b64": c.encoder_float32([1.0])},
        {"type": "imu", "t": 1.0, "accel_g": [0, 0], "gyro_rad_s": [0, 0, 0]},
        {"type": "odom", "t": 1.0, "x": "?", "y": 0, "cap": 0},
        {"type": "odom", "t": 1.0, "x": math.inf, "y": 0, "cap": 0},
        {"type": "image", "t": 1.0, "camera": "gauche", "jpeg_b64": "AAAA"},
    ):
        with pytest.raises(c.MessageInvalide):
            d.decoder(message, 10.0)
    with pytest.raises(c.MessageInvalide):
        d.decoder([1, 2], 10.0)


def test_decodeur_horloge() -> None:
    d = c.Decodeur()
    # Sans t_envoi, le `t` des mesures sert au recalage…
    d.decoder({"type": "imu", "t": 10.0, "accel_g": [0, 0, 1], "gyro_rad_s": [0, 0, 0]},
              10.01 + DECALAGE)
    assert d.horloge.decalage == pytest.approx(DECALAGE + 0.01)
    # … mais pas celui d'un message qui n'est pas une mesure (il peut être en heure murale).
    d.decoder({"type": "etat", "t": DECALAGE + 9.0}, 10.1 + DECALAGE)
    assert d.horloge.decalage == pytest.approx(DECALAGE + 0.01)
    d.reinitialiser()
    assert d.horloge.decalage is None
    # Sans recalage, tout est daté à la réception.
    brut = c.Decodeur(c.Reglages(recaler_horloge=False))
    odom = brut.decoder({"type": "odom", "t": 3.0, "t_envoi": 3.0, "x": 0, "y": 0, "cap": 0},
                        777.0)
    assert odom.stamp == 777.0
