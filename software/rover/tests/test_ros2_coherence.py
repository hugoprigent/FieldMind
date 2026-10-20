"""La pile ROS 2 (URDF, Nav2, pont) reste d'accord avec le profil du rover (config/rover.toml)."""

from __future__ import annotations

import math
import re
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

RACINE = Path(__file__).resolve().parents[1]
ROS2 = RACINE / "ros2" / "src"
XACRO = ROS2 / "fieldmind_rover_description" / "urdf" / "rover.urdf.xacro"
CONFIG = ROS2 / "fieldmind_rover_bringup" / "config"
ESPACE = "{http://www.ros.org/wiki/xacro}"


@pytest.fixture(scope="module")
def profil() -> dict:
    with open(RACINE / "config" / "rover.toml", "rb") as f:
        return tomllib.load(f)


@pytest.fixture(scope="module")
def proprietes() -> dict[str, float]:
    racine = ET.parse(XACRO).getroot()
    resultat = {}
    for p in racine.iter(f"{ESPACE}property"):
        try:
            resultat[p.get("name")] = float(p.get("value"))
        except (TypeError, ValueError):
            continue  # expression ${…}
    return resultat


def valeur_yaml(fichier: str, cle: str) -> str:
    """Première valeur `cle: valeur` d'un YAML de réglages (sans dépendre de PyYAML)."""
    texte = (CONFIG / fichier).read_text(encoding="utf-8")
    trouve = re.search(rf"^\s*{re.escape(cle)}:\s*(.+?)\s*(#.*)?$", texte, re.MULTILINE)
    assert trouve, f"{cle} absent de {fichier}"
    return trouve.group(1)


def test_urdf_suit_le_profil(profil: dict, proprietes: dict[str, float]) -> None:
    g, lidar, cam, gnss = profil["geometrie"], profil["lidar"], profil["cameras"], profil["gnss"]
    attendu = {
        "empattement": g["empattement_m"], "voie_arriere": g["voie_arriere_m"],
        "voie_avant": g["voie_avant_m"], "diametre_roue_arriere": g["diametre_roue_arriere_m"],
        "diametre_roue_avant": g["diametre_roue_avant_m"],
        "emprise_arriere": g["emprise_arriere_m"], "emprise_avant": g["emprise_avant_m"],
        "emprise_demi_largeur": g["emprise_demi_largeur_m"], "hauteur_max": g["hauteur_max_m"],
        "lidar_x": lidar["x_m"], "lidar_y": lidar["y_m"], "lidar_z": lidar["z_m"],
        "camera_x": cam["x_m"], "camera_base": cam["base_m"], "camera_z": cam["z_m"],
        "camera_inclinaison_deg": cam["inclinaison_deg"],
        "gnss_x": gnss["x_m"], "gnss_y": gnss["y_m"],
    }
    for nom, valeur in attendu.items():
        assert proprietes[nom] == pytest.approx(valeur), nom


def test_urdf_reperes_des_capteurs() -> None:
    racine = ET.parse(XACRO).getroot()
    liens = {lien.get("name") for lien in racine.iter("link")}
    assert {"base_link", "base_footprint", "lidar_link", "imu_link", "gnss_link"} <= liens
    # Les repères des caméras sont produits par la macro (nom = gauche, droite).
    appels = {e.get("nom") for e in racine.iter(f"{ESPACE}camera")}
    assert appels == {"gauche", "droite"}
    pont = (CONFIG / "pont.yaml").read_text(encoding="utf-8")
    assert '"camera_{camera}_optical_frame"' in pont
    assert "camera_${nom}_optical_frame" in XACRO.read_text(encoding="utf-8")


def test_emprise_de_nav2(profil: dict) -> None:
    g = profil["geometrie"]
    av, ar, demi = g["emprise_avant_m"], g["emprise_arriere_m"], g["emprise_demi_largeur_m"]
    attendu = f'"[[{av}, {demi}], [{av}, -{demi}], [{ar}, -{demi}], [{ar}, {demi}]]"'
    texte = (CONFIG / "nav2.yaml").read_text(encoding="utf-8")
    emprises = re.findall(r"^\s*footprint:\s*(.+)$", texte, re.MULTILINE)
    assert emprises == [attendu, attendu]  # costmaps locale et globale
    rayon_circonscrit = math.hypot(max(av, -ar), demi)
    locale = re.search(r"local_costmap:.*?inflation_radius:\s*([\d.]+)", texte, re.DOTALL)
    assert float(locale.group(1)) >= rayon_circonscrit


def test_rayon_de_braquage_et_vitesses(profil: dict) -> None:
    g, v = profil["geometrie"], profil["vitesses"]
    # Butée de 31° sur la roue intérieure : rayon au milieu de l'essieu arrière.
    rayon = g["empattement_m"] / math.tan(math.radians(g["braquage_max_deg"])) \
        + g["voie_avant_m"] / 2
    assert float(valeur_yaml("nav2.yaml", "minimum_turning_radius")) >= rayon
    assert float(valeur_yaml("nav2.yaml", "min_turning_r")) >= rayon
    assert float(valeur_yaml("nav2.yaml", "vx_max")) <= v["max_auto_m_s"]
    assert -float(valeur_yaml("nav2.yaml", "vx_min")) <= v["max_arriere_m_s"]
    assert float(valeur_yaml("pont.yaml", "vitesse_max")) <= v["max_auto_m_s"]
    assert float(valeur_yaml("pont.yaml", "vitesse_arriere_max")) <= v["max_arriere_m_s"]
    assert float(valeur_yaml("pont.yaml", "portee_min")) == profil["lidar"]["portee_min_m"]
    assert float(valeur_yaml("pont.yaml", "portee_max")) == profil["lidar"]["portee_max_m"]
    assert float(valeur_yaml("pont.yaml", "lidar_x")) == profil["lidar"]["x_m"]
    assert float(valeur_yaml("pont.yaml", "hfov_deg")) == profil["cameras"]["hfov_deg"]


def test_pas_de_rotation_sur_place() -> None:
    """Un Ackermann ne tourne pas sur place : ni Spin dans les arbres, ni dans les greffons."""
    for arbre in CONFIG.glob("*.xml"):
        texte = arbre.read_text(encoding="utf-8")
        assert "<Spin" not in texte, arbre.name
        assert "<BackUp" in texte, arbre.name
    assert "Spin" not in valeur_yaml("nav2.yaml", "behavior_plugins")
    assert valeur_yaml("nav2_rpp.yaml", "use_rotate_to_heading") == "false"
