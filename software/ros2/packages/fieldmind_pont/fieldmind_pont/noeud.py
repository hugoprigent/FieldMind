"""Nœud ROS 2 « pont FieldMind » : relie les topics du rover réel au cerveau FieldMind.

Topics lus (types standard) → messages JSON envoyés au serveur sur /ws/robot :
    /scan (sensor_msgs/LaserScan)                → scan
    /imu/data (sensor_msgs/Imu)                  → imu
    /odom (nav_msgs/Odometry, du VESC)           → odometrie (avec la dernière consigne de braquage)
    /fix (sensor_msgs/NavSatFix, du ZED-F9P)     → gnss (repère local depuis l'origine configurée)
    /firmware/etat (diagnostic_msgs/DiagnosticStatus) → firmware

Consignes reçues du serveur → /ackermann_cmd (ackermann_msgs/AckermannDriveStamped).

Si la liaison tombe, le pont ne publie plus rien : le chien de garde du contrôleur embarqué arrête
le rover, exactement comme en simulation.

Non testé sur ROS dans ce dépôt (pas de ROS dans l'environnement de développement) : seules les
conversions (`conversions.py`) sont couvertes par les tests. À valider sur le rover.
"""

from __future__ import annotations

import asyncio
import json
import threading
from typing import Any

import rclpy
import websockets
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu, LaserScan, NavSatFix

from fieldmind_pont import conversions


class PontFieldMind(Node):
    def __init__(self) -> None:
        super().__init__("pont_fieldmind")
        self.declare_parameter("url", "ws://gx10.local:8000/ws/robot")
        self.declare_parameter("jeton", "")
        self.declare_parameter("rayon_roue", 0.100)
        self.declare_parameter("origine_latitude", 0.0)
        self.declare_parameter("origine_longitude", 0.0)
        self.declare_parameter("frequence_image_hz", 2.0)
        p = self.get_parameter
        self._rayon = float(p("rayon_roue").value)
        self._origine = conversions.OrigineLocale(
            float(p("origine_latitude").value), float(p("origine_longitude").value)
        )
        self._braquage = 0.0
        self._periode_image = 1.0 / max(float(p("frequence_image_hz").value), 0.1)
        self._t_image = -1e9
        self._boucle = asyncio.new_event_loop()
        self._sortie: asyncio.Queue[str] = asyncio.Queue(maxsize=2000)

        self._consignes = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        capteurs = qos_profile_sensor_data
        self.create_subscription(LaserScan, "/scan", self._sur_scan, capteurs)
        self.create_subscription(Imu, "/imu/data", self._sur_imu, capteurs)
        self.create_subscription(Odometry, "/odom", self._sur_odom, capteurs)
        self.create_subscription(NavSatFix, "/fix", self._sur_fix, capteurs)
        self.create_subscription(DiagnosticStatus, "/firmware/etat", self._sur_firmware, 10)
        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed", self._sur_image, capteurs
        )

        url, jeton = str(p("url").value), str(p("jeton").value)
        self._fil = threading.Thread(target=self._executer_liaison, args=(url, jeton), daemon=True)
        self._fil.start()

    # ------------------------------------------------------------------ ROS → serveur
    def _envoyer(self, message: dict[str, Any]) -> None:
        texte = json.dumps(message, separators=(",", ":"))

        def deposer() -> None:
            if not self._sortie.full():
                self._sortie.put_nowait(texte)

        self._boucle.call_soon_threadsafe(deposer)

    def _sur_scan(self, m: LaserScan) -> None:
        t = conversions.temps(m.header.stamp.sec, m.header.stamp.nanosec)
        self._envoyer(
            conversions.scan(t, m.angle_min, m.angle_increment, m.range_min, m.range_max, m.ranges)
        )

    def _sur_imu(self, m: Imu) -> None:
        t = conversions.temps(m.header.stamp.sec, m.header.stamp.nanosec)
        o, w, a = m.orientation, m.angular_velocity, m.linear_acceleration
        self._envoyer(conversions.imu(t, (o.x, o.y, o.z, o.w), (w.x, w.y, w.z), (a.x, a.y, a.z)))

    def _sur_odom(self, m: Odometry) -> None:
        t = conversions.temps(m.header.stamp.sec, m.header.stamp.nanosec)
        self._envoyer(conversions.odometrie(t, m.twist.twist.linear.x, self._rayon, self._braquage))

    def _sur_fix(self, m: NavSatFix) -> None:
        t = conversions.temps(m.header.stamp.sec, m.header.stamp.nanosec)
        self._envoyer(
            conversions.gnss(
                t, m.latitude, m.longitude, m.status.status, m.position_covariance, self._origine
            )
        )

    def _sur_image(self, m: CompressedImage) -> None:
        t = conversions.temps(m.header.stamp.sec, m.header.stamp.nanosec)
        if t - self._t_image < self._periode_image:
            return  # le cerveau n'analyse que quelques images par seconde
        message = conversions.image(t, m.format, bytes(m.data))
        if message is None:
            self.get_logger().warning("image ignorée : JPEG attendu, 640 × 360 conseillé")
            return
        self._t_image = t
        self._envoyer(message)

    def _sur_firmware(self, m: DiagnosticStatus) -> None:
        maintenant = self.get_clock().now().nanoseconds * 1e-9
        self._envoyer(conversions.firmware(maintenant, {kv.key: kv.value for kv in m.values}))

    # ------------------------------------------------------------------ serveur → ROS
    def _publier_consigne(self, vitesse: float, braquage: float) -> None:
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.drive.speed = vitesse
        message.drive.steering_angle = braquage
        self._braquage = braquage
        self._consignes.publish(message)

    def _executer_liaison(self, url: str, jeton: str) -> None:
        asyncio.set_event_loop(self._boucle)
        self._boucle.run_until_complete(self._liaison(url, jeton))

    async def _liaison(self, url: str, jeton: str) -> None:
        while rclpy.ok():
            try:
                async with websockets.connect(f"{url}?jeton={jeton}", max_size=2**20) as ws:
                    self.get_logger().info(f"connecté au cerveau : {url}")
                    await asyncio.gather(self._emettre(ws), self._recevoir(ws))
            except (OSError, websockets.WebSocketException) as erreur:
                self.get_logger().warning(f"liaison au cerveau perdue ({erreur}), nouvel essai")
            while not self._sortie.empty():  # mesures périmées : jetées
                self._sortie.get_nowait()
            await asyncio.sleep(1.0)

    async def _emettre(self, ws: websockets.ClientConnection) -> None:
        while True:
            await ws.send(await self._sortie.get())

    async def _recevoir(self, ws: websockets.ClientConnection) -> None:
        async for texte in ws:
            consigne = conversions.commande(json.loads(texte))
            if consigne is not None:
                self._publier_consigne(*consigne)


def main() -> None:
    rclpy.init()
    noeud = PontFieldMind()
    try:
        rclpy.spin(noeud)
    finally:
        noeud.destroy_node()
        rclpy.shutdown()
