"""Nœud ROS 2 `pont` : client de la WebSocket `/ws/ros` du rover FieldMind.

    ros2 run fieldmind_rover_ros pont --ros-args -p rover:=rover.local:8080 -p jeton:=…

Rover → ROS 2 :
    scan   → /scan (sensor_msgs/LaserScan, lidar_link, angles uniformes reconstitués)
    imu    → /imu/data_raw (sensor_msgs/Imu, sans orientation : MPU-6050 sans magnétomètre)
    gnss   → /gps/fix (sensor_msgs/NavSatFix, gnss_link)
    odom   → /odom (nav_msgs/Odometry) + TF odom → base_link (recalage lidar + gyroscope du rover)
    image  → /camera/<gauche|droite>/image_raw/compressed (+ camera_info approchée)
    etat   → /fieldmind/etat (std_msgs/String, JSON) et /diagnostics
ROS 2 → rover :
    /cmd_vel (geometry_msgs/Twist)        → cmd_vel {vitesse, vitesse_lacet}
    /goal_pose (geometry_msgs/PoseStamped) → but {x, y} dans odom (paramètre `sujet_but`)

Le rover garde sa propre sécurité : il ne suit `cmd_vel` qu'en mode `auto`, armé, sans mission
en cours, et limite ou coupe chaque consigne. Le pont ne répète jamais une consigne et n'en garde
aucune pendant une coupure.

Toute la logique est dans `conversions.py` (sans ROS) et la partie réseau dans `client.py`.
"""

from __future__ import annotations

import array
import asyncio
import logging
import os
import threading
import time
from typing import Any

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, LaserScan, NavSatFix
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .client import ClientPont
from .conversions import (
    ERREUR,
    OK,
    PERIME,
    Decodeur,
    Diagnostic,
    EtatRos,
    GnssRos,
    ImageRos,
    ImuRos,
    LimitesConsigne,
    MessageInvalide,
    OdomRos,
    Reglages,
    ScanRos,
    but_vers_message,
    cap_quaternion,
    construire_url,
    info_camera,
    masquer_jeton,
    secondes_vers_stamp,
    twist_vers_cmd_vel,
)

CAMERAS = ("gauche", "droite")


class _VersJournalRos(logging.Handler):
    """Messages du module `client` (logging) → journal ROS du nœud."""

    def __init__(self, noeud: Node) -> None:
        super().__init__()
        self._journal = noeud.get_logger()

    def emit(self, enregistrement: logging.LogRecord) -> None:
        texte = self.format(enregistrement)
        if enregistrement.levelno >= logging.ERROR:
            self._journal.error(texte)
        elif enregistrement.levelno >= logging.WARNING:
            self._journal.warning(texte)
        else:
            self._journal.info(texte)


class Pont(Node):
    def __init__(self) -> None:
        super().__init__("pont_fieldmind")
        p = self._parametre
        adresse = p("rover", os.environ.get("FIELDMIND_ROVER", "rover.local:8080"))
        jeton = p("jeton", os.environ.get("FIELDMIND_JETON", ""))
        images = p("images", True)
        self._reglages = Reglages(
            nb_rayons=p("nb_rayons", 400),
            portee_min=p("portee_min", 0.12),
            portee_max=p("portee_max", 12.0),
            lidar_x=p("lidar_x", 0.0),
            lidar_y=p("lidar_y", 0.0),
            duree_scan=p("duree_scan", 0.1),
            recaler_horloge=p("recaler_horloge", True),
            hfov_deg=p("hfov_deg", 78.0),
            variance_acceleration=p("variance_acceleration", 0.1),
            variance_vitesse_angulaire=p("variance_vitesse_angulaire", 1e-4),
            uere_m=p("uere_m", 2.5),
        )
        self._limites = LimitesConsigne(
            vitesse_max=p("vitesse_max", 0.5),
            vitesse_arriere_max=p("vitesse_arriere_max", 0.3),
            vitesse_lacet_max=p("vitesse_lacet_max", 1.0),
        )
        self._repere_odom = p("repere_odom", "odom")
        self._repere_base = p("repere_base", "base_link")
        self._repere_lidar = p("repere_lidar", "lidar_link")
        self._repere_imu = p("repere_imu", "imu_link")
        self._repere_gnss = p("repere_gnss", "gnss_link")
        self._modele_repere_camera = p("repere_camera", "camera_{camera}_optical_frame")
        self._publier_tf = p("publier_tf", True)
        sujet_but = p("sujet_but", "/goal_pose")
        horodate = p("cmd_vel_horodate", False)
        silence_max_s = p("silence_max_s", 3.0)

        self._decodeur = Decodeur(self._reglages)
        self._invalides = 0
        self._diagnostics_rover: list[Diagnostic] = []
        self._date_etat = -1.0

        qos = QoSProfile(depth=10)
        verrouille = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_scan = self.create_publisher(LaserScan, "scan", qos)
        self._pub_imu = self.create_publisher(Imu, "imu/data_raw", qos)
        self._pub_gnss = self.create_publisher(NavSatFix, "gps/fix", qos)
        self._pub_odom = self.create_publisher(Odometry, "odom", qos)
        self._pub_etat = self.create_publisher(String, "fieldmind/etat", verrouille)
        self._pub_connecte = self.create_publisher(Bool, "fieldmind/connecte", verrouille)
        self._pub_diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", qos)
        self._pub_images: dict[str, tuple[object, object]] = {}
        self._verrou_images = threading.Lock()
        for camera in CAMERAS:
            self._publieurs_camera(camera)
        self._tf = TransformBroadcaster(self)
        self._tampon = Buffer()
        self._ecoute_tf = TransformListener(self._tampon, self, spin_thread=True)

        if horodate:
            self.create_subscription(TwistStamped, "cmd_vel",
                                     lambda m: self._sur_cmd_vel(m.twist), qos)
        else:
            self.create_subscription(Twist, "cmd_vel", self._sur_cmd_vel, qos)
        self.create_subscription(PoseStamped, sujet_but, self._sur_but, qos)

        journal_client = logging.getLogger("fieldmind_rover_ros")
        journal_client.setLevel(logging.INFO)
        journal_client.addHandler(_VersJournalRos(self))
        self._url = construire_url(adresse, jeton)
        self._client = ClientPont(
            self._url,
            images=images,
            sur_message=self._sur_message,
            sur_connexion=self._sur_connexion,
            horloge=self._maintenant,
            silence_max_s=silence_max_s,
        )
        self._publier_connecte(False)
        self._fil = threading.Thread(target=self._executer_reseau, name="ws-ros", daemon=True)
        self._fil.start()
        self.create_timer(1.0, self._publier_diagnostics)
        self.get_logger().info(
            f"rover {masquer_jeton(self._url)}, images {'oui' if images else 'non'}, "
            f"buts sur {sujet_but}")

    # ------------------------------------------------------------------ outils

    def _parametre(self, nom: str, defaut: Any) -> Any:
        return self.declare_parameter(nom, defaut).value

    def _maintenant(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp(t: float) -> TimeMsg:
        sec, nanosec = secondes_vers_stamp(t)
        return TimeMsg(sec=sec, nanosec=nanosec)

    def _publieurs_camera(self, camera: str) -> tuple[object, object]:
        with self._verrou_images:
            if camera not in self._pub_images:
                qos = QoSProfile(depth=5)
                self._pub_images[camera] = (
                    self.create_publisher(CompressedImage,
                                          f"camera/{camera}/image_raw/compressed", qos),
                    self.create_publisher(CameraInfo, f"camera/{camera}/camera_info", qos),
                )
            return self._pub_images[camera]

    # ------------------------------------------------------------------ réseau (fil ws-ros)

    def _executer_reseau(self) -> None:
        try:
            asyncio.run(self._client.executer())
        except Exception as erreur:  # le nœud reste en vie : diagnostics « déconnecté »
            self.get_logger().error(f"boucle réseau arrêtée : {erreur!r}")

    def arreter(self) -> None:
        self._client.arreter()
        self._fil.join(timeout=3.0)

    def _sur_connexion(self, connecte: bool, detail: str) -> None:
        self._decodeur.reinitialiser()
        self._publier_connecte(connecte)
        if connecte:
            self.get_logger().info(f"connecté au rover : {detail}")
        else:
            self.get_logger().warning(f"liaison avec le rover perdue : {detail}")

    def _publier_connecte(self, connecte: bool) -> None:
        self._pub_connecte.publish(Bool(data=connecte))

    def _sur_message(self, message: dict, t_reception: float) -> None:
        try:
            mesure = self._decodeur.decoder(message, t_reception)
        except MessageInvalide as erreur:
            self._invalides += 1
            self.get_logger().warning(f"message ignoré : {erreur}", throttle_duration_sec=5.0)
            return
        if isinstance(mesure, ScanRos):
            self._publier_scan(mesure)
        elif isinstance(mesure, OdomRos):
            self._publier_odom(mesure)
        elif isinstance(mesure, ImuRos):
            self._publier_imu(mesure)
        elif isinstance(mesure, GnssRos):
            self._publier_gnss(mesure)
        elif isinstance(mesure, ImageRos):
            self._publier_image(mesure)
        elif isinstance(mesure, EtatRos):
            self._pub_etat.publish(String(data=mesure.json))
            self._diagnostics_rover = mesure.diagnostics
            self._date_etat = time.monotonic()

    def _publier_scan(self, s: ScanRos) -> None:
        m = LaserScan()
        m.header.stamp = self._stamp(s.stamp)
        m.header.frame_id = self._repere_lidar
        m.angle_min = s.angle_min
        m.angle_max = s.angle_max
        m.angle_increment = s.increment
        m.time_increment = 0.0  # rayons réordonnés par angle : pas d'ordre temporel
        m.scan_time = s.duree
        m.range_min = s.portee_min
        m.range_max = s.portee_max
        m.ranges = array.array("f", s.distances.tobytes())
        self._pub_scan.publish(m)

    def _publier_odom(self, o: OdomRos) -> None:
        stamp = self._stamp(o.stamp)
        qx, qy, qz, qw = o.quaternion
        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id = self._repere_odom
        m.child_frame_id = self._repere_base
        m.pose.pose.position.x = o.x
        m.pose.pose.position.y = o.y
        m.pose.pose.orientation.x, m.pose.pose.orientation.y = qx, qy
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = qz, qw
        m.pose.covariance = o.covariance_pose
        m.twist.twist.linear.x = o.vitesse
        m.twist.twist.angular.z = o.vitesse_lacet
        m.twist.covariance = o.covariance_vitesse
        self._pub_odom.publish(m)
        if self._publier_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._repere_odom
            t.child_frame_id = self._repere_base
            t.transform.translation.x = o.x
            t.transform.translation.y = o.y
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self._tf.sendTransform(t)

    def _publier_imu(self, i: ImuRos) -> None:
        m = Imu()
        m.header.stamp = self._stamp(i.stamp)
        m.header.frame_id = self._repere_imu
        m.orientation_covariance = [-1.0] + [0.0] * 8  # REP 145 : orientation non fournie
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = i.vitesse_angulaire
        m.angular_velocity_covariance = i.covariance_vitesse_angulaire
        a = m.linear_acceleration
        a.x, a.y, a.z = i.acceleration
        m.linear_acceleration_covariance = i.covariance_acceleration
        self._pub_imu.publish(m)

    def _publier_gnss(self, g: GnssRos) -> None:
        m = NavSatFix()
        m.header.stamp = self._stamp(g.stamp)
        m.header.frame_id = self._repere_gnss
        m.status.status = g.statut
        m.status.service = g.service
        m.latitude, m.longitude, m.altitude = g.latitude, g.longitude, g.altitude
        m.position_covariance = g.covariance
        m.position_covariance_type = g.type_covariance
        self._pub_gnss.publish(m)

    def _publier_image(self, i: ImageRos) -> None:
        pub_image, pub_info = self._publieurs_camera(i.camera)
        repere = self._modele_repere_camera.format(camera=i.camera)
        stamp = self._stamp(i.stamp)
        m = CompressedImage()
        m.header.stamp = stamp
        m.header.frame_id = repere
        m.format = "jpeg"
        m.data = array.array("B", i.jpeg)
        pub_image.publish(m)
        if i.largeur and i.hauteur:
            info = info_camera(i.largeur, i.hauteur, self._reglages.hfov_deg)
            c = CameraInfo()
            c.header.stamp = stamp
            c.header.frame_id = repere
            c.width, c.height = info.largeur, info.hauteur
            c.distortion_model = info.modele
            c.d, c.k, c.r, c.p = info.d, info.k, info.r, info.p
            pub_info.publish(c)

    # ------------------------------------------------------------------ ROS → rover

    def _sur_cmd_vel(self, twist: Twist) -> None:
        message = twist_vers_cmd_vel(twist.linear.x, twist.angular.z, self._limites)
        if message is None:
            self.get_logger().warning("cmd_vel non finie ignorée", throttle_duration_sec=5.0)
            return
        if not self._client.envoyer(message):
            self.get_logger().warning("cmd_vel perdue : rover non connecté",
                                      throttle_duration_sec=5.0)

    def _sur_but(self, m: PoseStamped) -> None:
        repere = m.header.frame_id.lstrip("/") or self._repere_odom
        transformation = None
        if repere != self._repere_odom:
            try:
                tf = self._tampon.lookup_transform(self._repere_odom, repere, Time())
            except TransformException as erreur:
                self.get_logger().error(f"but ignoré : pas de TF {repere} → "
                                        f"{self._repere_odom} ({erreur})")
                return
            r = tf.transform.rotation
            transformation = (tf.transform.translation.x, tf.transform.translation.y,
                              cap_quaternion(r.x, r.y, r.z, r.w))
        try:
            message = but_vers_message(m.pose.position.x, m.pose.position.y, transformation)
        except ValueError:
            self.get_logger().error("but non fini ignoré")
            return
        if self._client.envoyer(message):
            self.get_logger().info(f"but envoyé au rover : x = {message['x']:.2f} m, "
                                   f"y = {message['y']:.2f} m ({self._repere_odom})")
        else:
            self.get_logger().error("but perdu : rover non connecté")

    # ------------------------------------------------------------------ diagnostics (1 Hz)

    def _publier_diagnostics(self) -> None:
        client = self._client
        decalage = self._decodeur.horloge.decalage
        pont = Diagnostic(
            "fieldmind/pont", OK if client.connecte else ERREUR,
            "connecté" if client.connecte else f"déconnecté : {client.derniere_erreur or '…'}",
            [("url", masquer_jeton(self._url)), ("connexions", str(client.connexions)),
             ("recus", ", ".join(f"{k} {v}" for k, v in sorted(client.recus.items()))),
             ("envoyes", str(client.envoyes)), ("perdus", str(client.perdus)),
             ("invalides", str(self._invalides + client.invalides)),
             ("decalage_horloge_s", "inconnu" if decalage is None else f"{decalage:.4f}")],
        )
        liste = [pont]
        perime = time.monotonic() - self._date_etat > 3.0
        for d in self._diagnostics_rover:
            if perime:
                liste.append(Diagnostic(d.nom, PERIME, "état du rover périmé", d.valeurs))
            else:
                liste.append(d)
        m = DiagnosticArray()
        m.header.stamp = self.get_clock().now().to_msg()
        for d in liste:
            m.status.append(DiagnosticStatus(
                level=bytes([d.niveau]), name=d.nom, message=d.message,
                hardware_id="fieldmind_rover",
                values=[KeyValue(key=k, value=v) for k, v in d.valeurs]))
        self._pub_diagnostics.publish(m)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    noeud = Pont()
    try:
        rclpy.spin(noeud)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        noeud.arreter()
        noeud.destroy_node()
        rclpy.try_shutdown()

