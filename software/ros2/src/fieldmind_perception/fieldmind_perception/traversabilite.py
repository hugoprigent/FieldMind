"""Nœud de traversabilité : carte d'élévation locale → obstacles pour Nav2 et scan filtré.

Entrées :
- /profondeur/points (PointCloud2, caméra de profondeur) : sol, marches, pentes ;
- /scan (LaserScan, lidar 2D) : obstacles à la hauteur du lidar, tout autour ;
- /imu/data (roulis et tangage) et TF odom → base_footprint (x, y, lacet).

Sorties :
- /traversabilite/obstacles (PointCloud2, base_footprint) : cellules non franchissables vues
  par la caméra (marches, pentes) et bords de trous, source « marquage » des costmaps ;
- /traversabilite/libre (PointCloud2, base_footprint) : fin du sol vu sur 360 rayons, source
  « effacement » des costmaps (on n'efface que ce qu'on a vu) ;
- /scan_filtre (LaserScan) : scan du lidar sans les impacts sur le sol vu par la caméra
  (montées), pour le SLAM et pour la couche « lidar » des costmaps ;
- /traversabilite/carte (OccupancyGrid, odom, tous capteurs, oubli compris) : costmap locale,
  interface et débogage.

L'odométrie de Nav2 reste plane (x, y, lacet). L'altitude du rover est intégrée ici à partir du
tangage et du déplacement, puis recalée sur le sol déjà cartographié à chaque image de
profondeur : la carte d'élévation n'a besoin que de hauteurs relatives cohérentes.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from fieldmind_perception.elevation import OBSTACLE, CarteElevation, Classement


def _rotation(roulis: float, tangage: float, lacet: float) -> np.ndarray:
    cr, sr = math.cos(roulis), math.sin(roulis)
    cp, sp = math.cos(tangage), math.sin(tangage)
    cy, sy = math.cos(lacet), math.sin(lacet)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _quaternion_matrice(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _lacet(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Traversabilite(Node):
    def __init__(self) -> None:
        super().__init__("traversabilite")
        p = self.declare_parameter
        self.cadre_odom = p("cadre_odom", "odom").value
        self.cadre_robot = p("cadre_robot", "base_footprint").value
        self.cadre_corps = p("cadre_corps", "base_link").value
        self.portee = p("portee", 8.0).value
        self.sous_echantillon = int(p("sous_echantillon_colonnes", 2).value)
        # Débogage : si non vide, la carte d'élévation y est enregistrée (npz) toutes les 5 s.
        self.dossier_debug = p("dossier_debug", "").value
        self.t_debug = 0.0
        self.carte = CarteElevation(
            resolution=p("resolution", 0.1).value,
            taille=p("taille", 24.0).value,
            pas_max=p("pas_max", 0.15).value,
            pente_max_deg=p("pente_max_deg", 22.0).value,
            hauteur_max=p("hauteur_max", 1.2).value,
            memoire_s=p("memoire_s", 60.0).value,
            memoire_lidar_s=p("memoire_lidar_s", 4.0).value,
        )
        self.tf = Buffer()
        # Fil propre : les recherches datées attendent la transformée sans bloquer ses messages.
        TransformListener(self.tf, self, spin_thread=True)
        self.rejets = 0
        self.derniere_erreur = ""
        self.roulis = self.tangage = 0.0
        # Historiques (t, valeurs) pour dater l'attitude et l'altitude à l'instant de la mesure.
        self.attitudes: deque[tuple[float, float, float]] = deque(maxlen=400)
        self.altitudes: deque[tuple[float, float]] = deque(maxlen=200)
        self.z = 0.0
        self.precedent: tuple[float, float] | None = None
        self.capteurs: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.classement: Classement | None = None
        self.t_classement = -math.inf

        self.create_subscription(Imu, "/imu/data", self._sur_imu, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, "/profondeur/points", self._sur_nuage, qos_profile_sensor_data
        )
        self.create_subscription(LaserScan, "/scan", self._sur_scan, qos_profile_sensor_data)
        self.pub_obstacles = self.create_publisher(PointCloud2, "/traversabilite/obstacles", 5)
        self.pub_libre = self.create_publisher(PointCloud2, "/traversabilite/libre", 5)
        self.pub_scan = self.create_publisher(LaserScan, "/scan_filtre", qos_profile_sensor_data)
        # Carte complète, remplacée à chaque publication : lue par la costmap locale (couche
        # « statique » glissante), l'interface et le débogage.
        latche = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_carte = self.create_publisher(OccupancyGrid, "/traversabilite/carte", latche)
        # Raison de chaque obstacle (33 marche, 66 pente, 100 lidar seul), pour l'interface.
        self.pub_raison = self.create_publisher(OccupancyGrid, "/traversabilite/raison", latche)
        self.create_timer(0.05, self._integrer_altitude)
        self.create_timer(0.25, self._publier)

    # ------------------------------------------------------------------ pose 3D
    def _sur_imu(self, m: Imu) -> None:
        q = m.orientation
        self.roulis = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        self.tangage = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.attitudes.append((self._t(m.header.stamp), self.roulis, self.tangage))

    @staticmethod
    def _a_l_instant(historique, t: float, defaut: tuple) -> tuple:
        """Valeurs interpolées linéairement à l'instant t (bornées aux extrémités)."""
        if not historique:
            return defaut
        if t <= historique[0][0]:
            return historique[0][1:]
        if t >= historique[-1][0]:
            return historique[-1][1:]
        for a, b in zip(reversed(historique), list(reversed(historique))[1:], strict=False):
            if b[0] <= t <= a[0]:
                f = (t - b[0]) / max(a[0] - b[0], 1e-9)
                return tuple(vb + f * (va - vb) for va, vb in zip(a[1:], b[1:], strict=True))
        return historique[-1][1:]

    def _pose_plane(self, stamp=None) -> tuple[float, float, float, Time] | None:
        try:
            # Sans attente : sous horloge simulée, une attente dans un rappel peut ne jamais finir.
            t = self.tf.lookup_transform(self.cadre_odom, self.cadre_robot, stamp or Time())
        except TransformException as e:
            self.derniere_erreur = str(e)
            return None
        tr = t.transform
        return (
            tr.translation.x,
            tr.translation.y,
            _lacet(tr.rotation),
            Time.from_msg(t.header.stamp),
        )

    def _integrer_altitude(self) -> None:
        """z += avance × tan(montée) ; en ROS, un tangage positif pique du nez."""
        pose = self._pose_plane()
        if pose is None:
            return
        x, y, lacet, stamp = pose
        if self.precedent is not None:
            dx, dy = x - self.precedent[0], y - self.precedent[1]
            avance = dx * math.cos(lacet) + dy * math.sin(lacet)
            self.z += -avance * math.tan(self.tangage)
        self.precedent = (x, y)
        self.altitudes.append((stamp.nanoseconds * 1e-9, self.z))
        self.carte.recentrer(x, y)

    def _capteur(self, cadre: str) -> tuple[np.ndarray, np.ndarray] | None:
        """Transformation fixe corps → capteur (issue de l'URDF)."""
        if cadre not in self.capteurs:
            try:
                t = self.tf.lookup_transform(self.cadre_corps, cadre, Time())
            except TransformException:
                return None
            tr = t.transform
            self.capteurs[cadre] = (
                _quaternion_matrice(tr.rotation),
                np.array([tr.translation.x, tr.translation.y, tr.translation.z]),
            )
        return self.capteurs[cadre]

    def _vers_carte(self, points: np.ndarray, cadre: str, stamp) -> np.ndarray | None:
        capteur = self._capteur(cadre)
        # Pose à l'instant de la mesure, jamais la dernière connue : en roulant ou en tournant, une
        # pose décalée de quelques dixièmes de seconde déplace les points lointains de plusieurs
        # décimètres (fausses marches et pentes). Sans pose datée, la mesure est ignorée.
        pose = self._pose_plane(stamp)
        if pose is None:
            self.rejets += 1
            self.get_logger().warn(
                f"pose indisponible à l'instant de la mesure ({self.rejets} mesures ignorées) : "
                f"{self.derniere_erreur}",
                throttle_duration_sec=10.0,
            )
            return None
        if capteur is None:
            return None
        rc, tc = capteur
        x, y, lacet, _ = pose
        t = stamp.nanoseconds * 1e-9
        roulis, tangage = self._a_l_instant(self.attitudes, t, (self.roulis, self.tangage))
        (z,) = self._a_l_instant(self.altitudes, t, (self.z,))
        corps = points @ rc.T + tc
        # base_link est à 0,1 m au-dessus de base_footprint, sur l'axe du corps.
        r = _rotation(roulis, tangage, lacet)
        origine = np.array([x, y, z]) + r @ np.array([0.0, 0.0, self._hauteur_corps()])
        return corps @ r.T + origine

    def _hauteur_corps(self) -> float:
        if "_corps" not in self.capteurs:
            try:
                t = self.tf.lookup_transform(self.cadre_robot, self.cadre_corps, Time())
                self.capteurs["_corps"] = (
                    np.eye(3),
                    np.array([0.0, 0.0, t.transform.translation.z]),
                )
            except TransformException:
                return 0.1
        return float(self.capteurs["_corps"][1][2])

    # ------------------------------------------------------------------ capteurs
    def _sur_nuage(self, m: PointCloud2) -> None:
        brut = np.frombuffer(m.data, np.float32).reshape(m.height, m.width, m.point_step // 4)
        xyz = brut[:, :: self.sous_echantillon, :3].reshape(-1, 3)
        distance = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(distance) & (distance > 0.25) & (distance < self.portee + 2)
        points = self._vers_carte(
            xyz[ok].astype(np.float64), m.header.frame_id, Time.from_msg(m.header.stamp)
        )
        if points is not None:
            decalage = self.carte.ajouter_nuage(points, self._t(m.header.stamp))
            # Recalage : l'altitude intégrée dérive, le sol déjà vu fait référence.
            self.z -= decalage
            self.altitudes = deque(((t, z - decalage) for t, z in self.altitudes), maxlen=200)

    def _sur_scan(self, m: LaserScan) -> None:
        r = np.asarray(m.ranges, dtype=np.float64)
        a = m.angle_min + np.arange(r.size) * m.angle_increment
        ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
        locaux = np.column_stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)])
        points = self._vers_carte(locaux[ok], m.header.frame_id, Time.from_msg(m.header.stamp))
        if points is None:
            return
        self.carte.ajouter_lidar(points, self._t(m.header.stamp))
        filtre = LaserScan()
        filtre.header = m.header
        for champ in (
            "angle_min",
            "angle_max",
            "angle_increment",
            "time_increment",
            "scan_time",
            "range_min",
            "range_max",
        ):
            setattr(filtre, champ, getattr(m, champ))
        sortie = r.copy()
        if self.classement is not None:
            sol = self.carte.est_sol(self.classement, points)
            indices = np.flatnonzero(ok)
            sortie[indices[sol]] = math.inf
        filtre.ranges = sortie.astype(np.float32).tolist()
        filtre.intensities = list(m.intensities)
        self.pub_scan.publish(filtre)

    @staticmethod
    def _t(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    # ------------------------------------------------------------------ sorties
    def _publier(self) -> None:
        pose = self._pose_plane()
        if pose is None:
            return
        x, y, lacet, stamp = pose
        t = stamp.nanoseconds * 1e-9
        c = self.carte.classer(t, (x, y))
        self.classement = c
        # Obstacles dans la portée, ramenés dans base_footprint.
        # Obstacles vus par la caméra de profondeur ; ceux du seul lidar passent par /scan_filtre.
        iy, ix = np.nonzero(c.profondeur == OBSTACLE)
        ox = self.carte.x0 + (ix + 0.5) * self.carte.res
        oy = self.carte.y0 + (iy + 0.5) * self.carte.res
        proche = np.hypot(ox - x, oy - y) < self.portee + 1.0
        angles, libre, trou = self.carte.rayons(c, x, y, portee=self.portee)
        avec_trou = np.isfinite(trou)
        ox = np.concatenate([ox[proche], x + trou[avec_trou] * np.cos(angles[avec_trou])])
        oy = np.concatenate([oy[proche], y + trou[avec_trou] * np.sin(angles[avec_trou])])
        vus = libre > 0
        lx = x + libre[vus] * np.cos(angles[vus])
        ly = y + libre[vus] * np.sin(angles[vus])
        entete = Header(frame_id=self.cadre_robot, stamp=stamp.to_msg())
        self.pub_obstacles.publish(self._nuage(entete, ox, oy, x, y, lacet))
        self.pub_libre.publish(self._nuage(entete, lx, ly, x, y, lacet))
        # Carte : état de chaque cellule, bords de trous compris.
        etat = c.etat.copy()
        ty, tx, dedans = self.carte.indices(
            x + trou[avec_trou] * np.cos(angles[avec_trou]),
            y + trou[avec_trou] * np.sin(angles[avec_trou]),
        )
        etat[ty[dedans], tx[dedans]] = OBSTACLE
        self._publier_carte(etat, c, stamp)
        if self.dossier_debug and t - self.t_debug > 5.0:
            self.t_debug = t
            k = self.carte
            np.savez_compressed(
                f"{self.dossier_debug}/elevation.npz",
                zmin=k.zmin,
                zmax=k.zmax,
                t_vu=k.t_vu,
                zlidar=k.zlidar,
                t_lidar=k.t_lidar,
                raison=c.raison,
                x0=k.x0,
                y0=k.y0,
                res=k.res,
                t=t,
                robot=(x, y, lacet, self.z),
            )

    @staticmethod
    def _nuage(entete: Header, px, py, x: float, y: float, lacet: float) -> PointCloud2:
        c, s = math.cos(lacet), math.sin(lacet)
        dx, dy = np.asarray(px) - x, np.asarray(py) - y
        locaux = np.column_stack([c * dx + s * dy, -s * dx + c * dy, np.full(len(dx), 0.2)])
        champs = [
            PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
            for k, n in enumerate("xyz")
        ]
        return point_cloud2.create_cloud(entete, champs, locaux.astype(np.float32))

    def _publier_carte(self, etat: np.ndarray, c: Classement, stamp: Time) -> None:
        g = OccupancyGrid()
        g.header.frame_id = self.cadre_odom
        g.header.stamp = stamp.to_msg()
        g.info.resolution = self.carte.res
        g.info.width = g.info.height = self.carte.n
        g.info.origin.position.x = c.x0
        g.info.origin.position.y = c.y0
        g.info.origin.orientation.w = 1.0
        valeurs = np.where(etat == OBSTACLE, 100, np.where(etat < 0, -1, 0)).astype(np.int8)
        g.data = valeurs.ravel().tolist()
        self.pub_carte.publish(g)
        if self.pub_raison.get_subscription_count():
            raison = np.where(c.raison > 0, c.raison.astype(np.int16) * 33 + (c.raison == 3), -1)
            g.data = raison.astype(np.int8).ravel().tolist()
            self.pub_raison.publish(g)


def main() -> None:
    rclpy.init()
    noeud = Traversabilite()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()
