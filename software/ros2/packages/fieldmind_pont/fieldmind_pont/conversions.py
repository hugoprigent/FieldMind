"""Conversions entre les messages ROS 2 du rover et le protocole JSON du cerveau FieldMind.

Fonctions pures (aucune dépendance à ROS) : elles sont testées dans la suite de tests du logiciel
(`software/tests/test_pont_ros2.py`) contre le validateur du protocole.
"""

from __future__ import annotations

import base64
import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import Any

# Rayons de l'ellipsoïde WGS84 (m).
_A = 6_378_137.0
_E2 = 6.694379990141317e-3

# sensor_msgs/NavSatStatus
_STATUS_NO_FIX, _STATUS_FIX, _STATUS_SBAS_FIX, _STATUS_GBAS_FIX = -1, 0, 1, 2
SIGMA_RTK_FIXE_MAX = 0.05
"""Au-delà de 5 cm (1 σ), une solution RTK est considérée comme flottante."""


@dataclass(frozen=True, slots=True)
class OrigineLocale:
    """Origine du repère local (x vers l'est, y vers le nord), en degrés."""

    latitude: float
    longitude: float

    def vers_local(self, latitude: float, longitude: float) -> tuple[float, float]:
        """Plan tangent local : précis au centimètre sur quelques kilomètres."""
        phi = math.radians(self.latitude)
        s = math.sin(phi)
        rayon_meridien = _A * (1 - _E2) / (1 - _E2 * s * s) ** 1.5
        rayon_normal = _A / math.sqrt(1 - _E2 * s * s)
        x = math.radians(longitude - self.longitude) * rayon_normal * math.cos(phi)
        y = math.radians(latitude - self.latitude) * rayon_meridien
        return x, y


def temps(sec: int, nanosec: int) -> float:
    return sec + nanosec * 1e-9


def scan(
    t: float,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    ranges: Iterable[float],
) -> dict[str, Any]:
    """sensor_msgs/LaserScan → `scan`. Mesures hors bornes, inf ou NaN → pas d'écho (null)."""
    distances: list[float | None] = []
    for r in ranges:
        valide = math.isfinite(r) and range_min <= r <= range_max
        distances.append(round(float(r), 4) if valide else None)
    return {
        "type": "scan",
        "t": t,
        "angle_min": angle_min,
        "pas": angle_increment,
        "portee_min": range_min,
        "portee_max": range_max,
        "distances": distances,
    }


def imu(
    t: float,
    orientation_xyzw: tuple[float, float, float, float],
    vitesse_angulaire: tuple[float, float, float],
    acceleration: tuple[float, float, float],
) -> dict[str, Any]:
    """sensor_msgs/Imu → `imu` (quaternion ROS x, y, z, w → w, x, y, z)."""
    x, y, z, w = orientation_xyzw
    return {
        "type": "imu",
        "t": t,
        "orientation": [w, x, y, z],
        "vitesse_angulaire": list(vitesse_angulaire),
        "acceleration": list(acceleration),
    }


def odometrie(t: float, vitesse: float, rayon_roue: float, braquage: float) -> dict[str, Any]:
    """Vitesse longitudinale de l'essieu arrière (nav_msgs/Odometry du VESC) → `odometrie`.

    Essieu rigide : les deux roues arrière tournent à la même vitesse. Les roues avant n'ont pas
    de codeur. Le braquage est la dernière consigne appliquée au servo (pas de retour de position).
    """
    omega = vitesse / rayon_roue
    return {
        "type": "odometrie",
        "t": t,
        "roue_ar_g": omega,
        "roue_ar_d": omega,
        "roue_av_g": None,
        "roue_av_d": None,
        "braquage": braquage,
    }


def gnss(
    t: float,
    latitude: float,
    longitude: float,
    statut: int,
    covariance: Iterable[float],
    origine: OrigineLocale,
    satellites: int = 0,
) -> dict[str, Any]:
    """sensor_msgs/NavSatFix → `gnss`, en coordonnées locales (m)."""
    cov = list(covariance)
    variance = max((cov[0] + cov[4]) / 2, 1e-6) if len(cov) >= 5 else 25.0
    sigma = math.sqrt(variance)
    if statut <= _STATUS_NO_FIX:
        fix = "aucun"
    elif statut == _STATUS_GBAS_FIX:
        fix = "rtk_fixe" if sigma <= SIGMA_RTK_FIXE_MAX else "rtk_flottant"
    elif statut == _STATUS_SBAS_FIX:
        fix = "dgnss"
    else:
        fix = "autonome"
    x, y = origine.vers_local(latitude, longitude)
    return {
        "type": "gnss",
        "t": t,
        "fix": fix,
        "x": round(x, 4),
        "y": round(y, 4),
        "sigma": round(sigma, 4),
        "satellites": satellites,
    }


RAISONS_ARRET = {"obstacle_avant", "obstacle_arriere", "chien_de_garde", "arret_operateur"}


def firmware(t: float, valeurs: Mapping[str, str]) -> dict[str, Any]:
    """diagnostic_msgs/DiagnosticStatus (clés/valeurs du contrôleur embarqué) → `firmware`."""

    def nombre(cle: str, defaut: float = 0.0) -> float:
        try:
            return float(valeurs.get(cle, defaut))
        except ValueError:
            return defaut

    arret = valeurs.get("arret") or None
    return {
        "type": "firmware",
        "t": t,
        "arret": arret if arret in RAISONS_ARRET else None,
        "vitesse_consigne": nombre("vitesse_consigne"),
        "braquage_consigne": nombre("braquage_consigne"),
        "couple_moteur": nombre("couple_moteur"),
        "regime_moteur": nombre("regime_moteur"),
        "tension_batterie": nombre("tension_batterie"),
        "courant_batterie": nombre("courant_batterie"),
        "etat_charge": min(max(nombre("etat_charge"), 0.0), 1.0),
    }


def dimensions_jpeg(donnees: bytes) -> tuple[int, int] | None:
    """(largeur, hauteur) lues dans l'en-tête SOF d'un JPEG, sans le décoder ; None sinon."""
    if donnees[:2] != b"\xff\xd8":
        return None
    i = 2
    while i + 9 < len(donnees):
        if donnees[i] != 0xFF:
            return None
        marqueur = donnees[i + 1]
        longueur = int.from_bytes(donnees[i + 2 : i + 4], "big")
        if marqueur in (0xC0, 0xC1, 0xC2):
            hauteur = int.from_bytes(donnees[i + 5 : i + 7], "big")
            largeur = int.from_bytes(donnees[i + 7 : i + 9], "big")
            return largeur, hauteur
        i += 2 + longueur
    return None


TAILLE_MAX_IMAGE_B64 = 400_000


def image(t: float, format_ros: str, donnees: bytes) -> dict[str, Any] | None:
    """sensor_msgs/CompressedImage (JPEG) → `image` ; None si ce n'est pas un JPEG lisible ou
    s'il est trop gros pour le protocole (réduire la résolution côté caméra)."""
    if "jpeg" not in format_ros.lower() and "jpg" not in format_ros.lower():
        return None
    dimensions = dimensions_jpeg(donnees)
    if dimensions is None:
        return None
    encodee = base64.b64encode(donnees).decode("ascii")
    if len(encodee) > TAILLE_MAX_IMAGE_B64:
        return None
    return {
        "type": "image",
        "t": t,
        "largeur": dimensions[0],
        "hauteur": dimensions[1],
        "format": "jpeg",
        "donnees": encodee,
    }


def commande(message: Mapping[str, Any]) -> tuple[float, float] | None:
    """`commande` JSON du cerveau → (vitesse m/s, braquage rad) pour ackermann_msgs, ou None."""
    if message.get("type") != "commande":
        return None
    vitesse, braquage = float(message["vitesse"]), float(message["braquage"])
    if not (math.isfinite(vitesse) and math.isfinite(braquage)):
        return None
    return vitesse, braquage
