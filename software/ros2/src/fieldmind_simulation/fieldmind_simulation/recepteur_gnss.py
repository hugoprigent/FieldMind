"""Modèle du récepteur GNSS RTK (u-blox ZED-F9P) : erreur réaliste selon le ciel visible.

Gazebo donne la position exacte de l'antenne (/sim/gnss_exact). Ce nœud publie ce que le
récepteur donnerait (/gnss/fix), avec la covariance qu'il annonce (hAcc) :
- ciel dégagé : RTK fixe, ≈ 1,5 cm ;
- sous un couvert partiel ou près d'un bâtiment : RTK flottant, quelques décimètres, avec un biais
  qui dérive lentement ;
- sous un couvert dense : solution autonome, 1 à 3 m, biais lent ;
- retour au RTK fixe seulement après une dizaine de secondes de ciel dégagé (reconvergence).

Le couvert est estimé à partir des arbres et bâtiments du monde généré (description.json), à la
position vraie du rover : c'est un modèle de capteur, pas une information donnée au rover.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

# Mode : (écart-type du bruit blanc, écart-type du biais, constante de temps du biais) en m, m, s
MODES = {
    "rtk_fixe": (0.012, 0.005, 60.0),
    "rtk_flottant": (0.08, 0.35, 40.0),
    "autonome": (0.6, 1.8, 90.0),
}
RAYON_TERRE = 6378137.0


class RecepteurGnss(Node):
    def __init__(self) -> None:
        super().__init__("recepteur_gnss")
        chemin = self.declare_parameter("description", "").value
        self.reconvergence = self.declare_parameter("reconvergence_s", 12.0).value
        self.rng = np.random.default_rng(self.declare_parameter("graine", 1).value)
        self.arbres = np.zeros((0, 2))
        self.batiments = np.zeros((0, 2))
        if chemin and Path(chemin).exists():
            objets = json.loads(Path(chemin).read_text())["objets"]
            self.arbres = np.array(
                [(o["x"], o["y"]) for o in objets if o["type"] in ("pin", "chene")]
            ).reshape(-1, 2)
            self.batiments = np.array(
                [
                    (o["extra"].get("ex", o["x"]), o["extra"].get("ey", o["y"]))
                    for o in objets
                    if o["type"].startswith("maison") or o["type"] == "ruine"
                ]
            ).reshape(-1, 2)
        self.position: tuple[float, float] | None = None
        self.biais = np.zeros(2)
        self.mode = "rtk_fixe"
        self.t_precedent: float | None = None
        self.t_ciel = 0.0
        self.create_subscription(PoseStamped, "/sim/verite", self._sur_verite, 10)
        self.create_subscription(
            NavSatFix, "/sim/gnss_exact", self._sur_fix, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(NavSatFix, "/gnss/fix", qos_profile_sensor_data)
        self.pub_etat = self.create_publisher(DiagnosticStatus, "/gnss/etat", 5)

    def _sur_verite(self, m: PoseStamped) -> None:
        self.position = (m.pose.position.x, m.pose.position.y)

    def couvert(self) -> float:
        """Indice de masquage du ciel : somme pondérée des houppiers et bâtiments proches."""
        if self.position is None:
            return 0.0
        p = np.array(self.position)
        c = 0.0
        if len(self.arbres):
            d2 = np.sum((self.arbres - p) ** 2, axis=1)
            c += float(np.sum(np.exp(-d2 / (2 * 3.5**2))))
        if len(self.batiments):
            d2 = np.sum((self.batiments - p) ** 2, axis=1)
            c += 1.5 * float(np.sum(np.exp(-d2 / (2 * 7.0**2))))
        return c

    def _choisir_mode(self, couvert: float, dt: float) -> str:
        if couvert < 0.6:
            self.t_ciel += dt
        else:
            self.t_ciel = 0.0
        if couvert >= 2.0:
            return "autonome"
        if couvert >= 0.6:
            return "rtk_flottant"
        if self.mode != "rtk_fixe" and self.t_ciel < self.reconvergence:
            return "rtk_flottant"
        return "rtk_fixe"

    def _sur_fix(self, m: NavSatFix) -> None:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        dt = 0.1 if self.t_precedent is None else min(max(t - self.t_precedent, 0.0), 1.0)
        self.t_precedent = t
        couvert = self.couvert()
        self.mode = self._choisir_mode(couvert, dt)
        blanc, sigma_biais, tau = MODES[self.mode]
        # Biais de Gauss-Markov (multitrajet, résidus atmosphériques), puis bruit blanc.
        if dt > 0:
            self.biais += -self.biais * dt / tau + sigma_biais * math.sqrt(2 * dt / tau) * (
                self.rng.standard_normal(2)
            )
        erreur = self.biais + blanc * self.rng.standard_normal(2)
        sortie = NavSatFix()
        sortie.header = m.header
        sortie.status.status = (
            NavSatStatus.STATUS_GBAS_FIX if self.mode.startswith("rtk") else NavSatStatus.STATUS_FIX
        )
        sortie.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GALILEO
        lat = math.radians(m.latitude)
        sortie.latitude = m.latitude + math.degrees(erreur[1] / RAYON_TERRE)
        sortie.longitude = m.longitude + math.degrees(erreur[0] / (RAYON_TERRE * math.cos(lat)))
        sortie.altitude = m.altitude + 2.0 * erreur[0]
        # Précision annoncée par le récepteur (hAcc) : un peu pessimiste, comme le vrai.
        precision = 1.5 * math.hypot(blanc, sigma_biais)
        sortie.position_covariance = [
            precision**2, 0.0, 0.0,
            0.0, precision**2, 0.0,
            0.0, 0.0, (2 * precision) ** 2,
        ]  # fmt: skip
        sortie.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(sortie)
        etat = DiagnosticStatus(name="gnss", hardware_id="zed-f9p", message=self.mode)
        etat.level = DiagnosticStatus.OK if self.mode == "rtk_fixe" else DiagnosticStatus.WARN
        etat.values = [
            KeyValue(key="mode", value=self.mode),
            KeyValue(key="precision_m", value=f"{precision:.3f}"),
            KeyValue(key="couvert", value=f"{couvert:.2f}"),
        ]
        self.pub_etat.publish(etat)


def main() -> None:
    rclpy.init()
    noeud = RecepteurGnss()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()
