"""Conversions du pont ROS 2 : chaque message produit doit passer le validateur du protocole."""

from __future__ import annotations

import base64
import importlib.util
import io
import math
import sys
from pathlib import Path

import pytest
from PIL import Image as ImagePil

from services.protocole.messages import Commande, lire_message_robot

CHEMIN = (
    Path(__file__).resolve().parents[1]
    / "ros2/packages/fieldmind_pont/fieldmind_pont/conversions.py"
)
_spec = importlib.util.spec_from_file_location("conversions_pont", CHEMIN)
assert _spec is not None
assert _spec.loader is not None
conv = importlib.util.module_from_spec(_spec)
sys.modules["conversions_pont"] = conv  # requis par les dataclasses du module
_spec.loader.exec_module(conv)


def test_laserscan() -> None:
    message = conv.scan(
        12.5, -math.pi, math.radians(1), 0.1, 8.0, [1.0, math.inf, float("nan"), 0.05, 9.0]
    )
    scan = lire_message_robot.validate_python(message)
    assert scan.distances == [1.0, None, None, None, None]


def test_imu_quaternion_reordonne() -> None:
    message = conv.imu(1.0, (0.0, 0.0, 0.7071, 0.7071), (0.0, 0.0, 0.1), (0.0, 0.0, 9.81))
    imu = lire_message_robot.validate_python(message)
    assert imu.orientation == (0.7071, 0.0, 0.0, 0.7071)


def test_odometrie_essieu_arriere_seul() -> None:
    odo = lire_message_robot.validate_python(conv.odometrie(2.0, 1.2, 0.1, 0.2))
    assert odo.roue_ar_g == pytest.approx(12.0)
    assert odo.roue_av_g is None


def test_gnss_repere_local() -> None:
    origine = conv.OrigineLocale(48.0, 2.0)
    covariance = [0.0004, 0, 0, 0, 0.0004, 0, 0, 0, 0.001]
    gnss = lire_message_robot.validate_python(
        conv.gnss(3.0, 48.0 + 1e-4, 2.0 + 1e-4, 2, covariance, origine, 22)
    )
    assert gnss.fix == "rtk_fixe"
    assert gnss.y == pytest.approx(11.12, abs=0.02)  # 1e-4° de latitude ≈ 11,1 m
    assert gnss.x == pytest.approx(7.46, abs=0.02)  # cos(48°) fois plus court en longitude
    flottant = conv.gnss(3.0, 48.0, 2.0, 2, [0.04, 0, 0, 0, 0.04], origine)
    assert flottant["fix"] == "rtk_flottant"
    assert conv.gnss(3.0, 48.0, 2.0, -1, [], origine)["fix"] == "aucun"


def test_firmware() -> None:
    message = conv.firmware(
        4.0, {"arret": "chien_de_garde", "tension_batterie": "39.8", "etat_charge": "1.4"}
    )
    firmware = lire_message_robot.validate_python(message)
    assert firmware.arret == "chien_de_garde"
    assert firmware.etat_charge == 1.0


def test_commande() -> None:
    texte = Commande(t=1.0, vitesse=0.8, braquage=-0.2).model_dump()
    assert conv.commande(texte) == (0.8, -0.2)
    assert conv.commande({"type": "autre"}) is None


def test_image_jpeg() -> None:
    tampon = io.BytesIO()
    ImagePil.new("RGB", (640, 360), (90, 120, 60)).save(tampon, format="JPEG", quality=80)
    message = conv.image(12.5, "rgb8; jpeg compressed bgr8", tampon.getvalue())
    assert message is not None
    image = lire_message_robot.validate_python(message)
    assert (image.largeur, image.hauteur) == (640, 360)
    assert base64.b64decode(image.donnees) == tampon.getvalue()
    assert conv.image(1.0, "png", tampon.getvalue()) is None
    assert conv.image(1.0, "jpeg", b"pas un jpeg") is None
