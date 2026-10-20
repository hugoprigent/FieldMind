"""Contrôleur embarqué simulé (MCU du rover) : ce que fera le microcontrôleur du rover réel.

Sur le rover, ce code devient le firmware du MCU (micro-ROS) : mêmes topics, mêmes seuils. Il
est indépendant du cerveau (GX10) et le protège de ses erreurs.

Entrées : /cmd_vel (geometry_msgs/Twist, sortie de Nav2 ou de la téléopération), /scan (lidar,
pour l'arrêt d'urgence), /mcu/arret (arrêt opérateur).
Sorties : consignes des actionneurs (vitesse des roues arrière pour le VESC, angle du servo),
/odom_roues (odométrie : compte-tours de l'essieu arrière et braquage *commandé* — le servo n'a
pas de retour de position), /mcu/etat (diagnostic).

Protections, prioritaires sur toute consigne :
- chien de garde : sans consigne pendant 0,5 s, freinage jusqu'à l'arrêt ;
- arrêt d'urgence : obstacle dans le couloir du rover plus proche que la distance de freinage
  d'urgence + 0,40 m (le chemin parcouru depuis le dernier balayage est retranché) ;
- rampes d'accélération et de décélération, vitesse et braquage bornés.

Servo de direction (modèle d'un servo de modélisme 35 kg·cm, sans retour de position) :
consigne rafraîchie à 50 Hz (période PWM), vitesse limitée (250°/s en charge), zone morte
(±0,5°), jeu mécanique de la tringlerie (±0,7°) et défaut de neutre (0,8°, à calibrer sur le
rover). La géométrie Ackermann des biellettes donne l'angle de chaque roue avant.
"""

from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Float64

FREQUENCE_HZ = 100.0
PERIODE_PWM_S = 0.02
PERIODE_TELEMETRIE_S = 0.02


class ServoDirection:
    """Servo de modélisme : il vise la dernière consigne PWM, à vitesse limitée, avec une zone
    morte ; la tringlerie ajoute un jeu. Il ne renvoie aucune mesure."""

    def __init__(
        self, vitesse_deg_s: float, zone_morte_deg: float, jeu_deg: float, neutre_deg: float
    ) -> None:
        self.vitesse = math.radians(vitesse_deg_s)
        self.zone_morte = math.radians(zone_morte_deg)
        self.jeu = math.radians(jeu_deg)
        self.neutre = math.radians(neutre_deg)
        self.consigne_pwm = 0.0
        self.palonnier = 0.0
        self.sortie = 0.0
        self._t_pwm = 0.0

    def cycle(self, t: float, consigne: float, dt: float) -> float:
        if t - self._t_pwm >= PERIODE_PWM_S:
            self._t_pwm = t
            self.consigne_pwm = consigne
        ecart = self.consigne_pwm + self.neutre - self.palonnier
        if abs(ecart) > self.zone_morte:
            pas = self.vitesse * dt
            self.palonnier += max(-pas, min(pas, ecart))
        # Jeu : la roue ne suit le palonnier qu'une fois le jeu rattrapé.
        if self.palonnier - self.sortie > self.jeu:
            self.sortie = self.palonnier - self.jeu
        elif self.sortie - self.palonnier > self.jeu:
            self.sortie = self.palonnier + self.jeu
        return self.sortie


class ControleurEmbarque(Node):
    def __init__(self) -> None:
        super().__init__("controleur_embarque")
        p = self.declare_parameters(
            "",
            [
                ("empattement", 0.400),
                ("voie_avant", 0.446),
                ("rayon_roue", 0.100),
                ("longueur", 0.70),
                ("largeur", 0.53),
                ("lidar_x_depuis_centre", 0.18),
                ("vitesse_max", 1.5),
                ("vitesse_arriere_max", 0.6),
                ("braquage_max_deg", 30.0),
                ("vitesse_braquage_deg_s", 250.0),
                ("zone_morte_deg", 0.5),
                ("jeu_deg", 0.7),
                ("neutre_deg", 0.8),
                ("acceleration_max", 1.0),
                ("deceleration_max", 2.0),
                ("deceleration_urgence", 4.0),
                ("distance_arret", 0.40),
                ("secteur_avant_deg", 30.0),
                ("chien_de_garde_s", 0.5),
                ("impulsions_par_tour", 2048),
            ],
        )
        del p
        g = self.get_parameter
        self.L = g("empattement").value
        self.voie = g("voie_avant").value
        self.r = g("rayon_roue").value
        longueur, largeur = g("longueur").value, g("largeur").value
        lx = g("lidar_x_depuis_centre").value
        self.avant_depuis_lidar = longueur / 2 - lx
        self.arriere_depuis_lidar = longueur / 2 + lx
        self.demi_couloir = largeur / 2 + 0.05
        self.vmax, self.vrev = g("vitesse_max").value, g("vitesse_arriere_max").value
        self.bmax = math.radians(g("braquage_max_deg").value)
        self.acc, self.dec = g("acceleration_max").value, g("deceleration_max").value
        self.dec_urg = g("deceleration_urgence").value
        self.marge = g("distance_arret").value
        self.secteur = math.radians(g("secteur_avant_deg").value)
        self.delai_garde = g("chien_de_garde_s").value
        self.impulsions = g("impulsions_par_tour").value
        self.servo = ServoDirection(
            g("vitesse_braquage_deg_s").value,
            g("zone_morte_deg").value,
            g("jeu_deg").value,
            g("neutre_deg").value,
        )

        self.v_demandee = 0.0
        self.b_demande = 0.0
        self.t_consigne: float | None = None
        self.v_consigne = 0.0
        self.b_consigne = 0.0
        self.arret: str | None = None
        self.arret_operateur = False
        self.libre_avant = math.inf
        self.libre_arriere = math.inf
        self.parcouru = 0.0
        self.angle_roues: float | None = None
        self.t_roues: float | None = None
        self.v_mesuree = 0.0
        self.t_precedent: float | None = None

        self.create_subscription(Twist, "/cmd_vel", self._sur_cmd_vel, 10)
        self.create_subscription(LaserScan, "/scan", self._sur_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, "/mcu/arret", self._sur_arret, 10)
        self.create_subscription(JointState, "/sim/joint_states", self._sur_roues, 20)
        self.pub_g = self.create_publisher(Float64, "/sim/roue_ar_g/cmd_vel", 10)
        self.pub_d = self.create_publisher(Float64, "/sim/roue_ar_d/cmd_vel", 10)
        self.pub_bg = self.create_publisher(Float64, "/sim/braquage_g/cmd_pos", 10)
        self.pub_bd = self.create_publisher(Float64, "/sim/braquage_d/cmd_pos", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom_roues", 20)
        self.pub_etat = self.create_publisher(DiagnosticStatus, "/mcu/etat", 10)
        self.create_timer(1.0 / FREQUENCE_HZ, self._cycle)
        self.create_timer(0.1, self._publier_etat)

    # ------------------------------------------------------------------ entrées
    def _maintenant(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _sur_cmd_vel(self, m: Twist) -> None:
        v = min(max(m.linear.x, -self.vrev), self.vmax)
        # À l'arrêt, un Ackermann ne tourne pas sur place : on garde le braquage demandé.
        b = math.atan(self.L * m.angular.z / v) if abs(v) > 0.05 else self.b_demande
        self.v_demandee = v
        self.b_demande = min(max(b, -self.bmax), self.bmax)
        self.t_consigne = self._maintenant()

    def _sur_arret(self, m: Bool) -> None:
        self.arret_operateur = m.data

    def _sur_scan(self, m: LaserScan) -> None:
        avant = arriere = math.inf
        for i, d in enumerate(m.ranges):
            if not (m.range_min <= d <= m.range_max) or not math.isfinite(d):
                continue
            a = m.angle_min + i * m.angle_increment
            x, y = d * math.cos(a), d * math.sin(a)
            if abs(y) > self.demi_couloir:
                continue
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) <= self.secteur:
                avant = min(avant, x - self.avant_depuis_lidar)
            elif abs(a) >= math.pi - self.secteur:
                arriere = min(arriere, -x - self.arriere_depuis_lidar)
        self.libre_avant, self.libre_arriere = avant, arriere
        self.parcouru = 0.0

    def _sur_roues(self, m: JointState) -> None:
        """Compte-tours de l'essieu arrière (VESC), quantifié comme un codeur."""
        try:
            i = m.name.index("roue_ar_g_joint")
            j = m.name.index("roue_ar_d_joint")
        except ValueError:
            return
        angle = 0.5 * (m.position[i] + m.position[j])
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pas = 2 * math.pi / self.impulsions
        angle = math.floor(angle / pas) * pas
        if self.angle_roues is None or self.t_roues is None:
            self.angle_roues, self.t_roues = angle, t
            return
        # Télémétrie du VESC à 50 Hz : vitesse moyenne sur la fenêtre écoulée.
        if t - self.t_roues < PERIODE_TELEMETRIE_S:
            return
        omega = (angle - self.angle_roues) / (t - self.t_roues)
        self.v_mesuree = omega * self.r
        self._publier_odometrie(m, self.v_mesuree)
        self.angle_roues, self.t_roues = angle, t

    # ------------------------------------------------------------------ boucle 100 Hz
    def distance_freinage(self, v: float) -> float:
        return self.marge + abs(v) * 2 / FREQUENCE_HZ + v * v / (2 * self.dec_urg)

    def _cycle(self) -> None:
        t = self._maintenant()
        dt = 1.0 / FREQUENCE_HZ if self.t_precedent is None else max(t - self.t_precedent, 0.0)
        self.t_precedent = t
        if dt <= 0:
            return
        self.parcouru += self.v_mesuree * dt
        libre_avant = self.libre_avant - max(self.parcouru, 0.0)
        libre_arriere = self.libre_arriere + min(self.parcouru, 0.0)
        freinage = self.distance_freinage(max(self.v_mesuree, self.v_consigne, key=abs))

        cible = self.v_demandee
        self.arret = None
        if self.arret_operateur:
            self.arret = "arret_operateur"
        elif self.t_consigne is None or t - self.t_consigne > self.delai_garde:
            self.arret = "chien_de_garde" if self.t_consigne is not None else None
            cible = 0.0
        elif cible > 0 and libre_avant < freinage:
            self.arret = "obstacle_avant"
        elif cible < 0 and libre_arriere < freinage:
            self.arret = "obstacle_arriere"
        if self.arret is not None:
            cible = 0.0

        ecart = cible - self.v_consigne
        accelere = abs(cible) > abs(self.v_consigne) and cible * self.v_consigne >= 0
        pas = (self.acc if accelere else self.dec_urg if self.arret else self.dec) * dt
        self.v_consigne += max(-pas, min(pas, ecart))

        # Braquage : consigne « bicyclette » → servo → biellettes (Ackermann).
        self.b_consigne = self.b_demande
        roue = self.servo.cycle(t, self.b_consigne, dt)
        gauche, droite = self.ackermann(roue)
        omega = self.v_consigne / self.r
        self.pub_g.publish(Float64(data=omega))
        self.pub_d.publish(Float64(data=omega))
        self.pub_bg.publish(Float64(data=gauche))
        self.pub_bd.publish(Float64(data=droite))

    def ackermann(self, braquage: float) -> tuple[float, float]:
        if abs(braquage) < 1e-6:
            return 0.0, 0.0
        rayon = self.L / math.tan(braquage)
        demi = self.voie / 2
        return math.atan(self.L / (rayon - demi)), math.atan(self.L / (rayon + demi))

    # ------------------------------------------------------------------ sorties
    def _publier_odometrie(self, m: JointState, v: float) -> None:
        odom = Odometry()
        odom.header.stamp = m.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = v
        # Le servo n'a pas de retour : l'odométrie utilise le braquage commandé.
        odom.twist.twist.angular.z = v * math.tan(self.b_consigne) / self.L
        cov = [0.0] * 36
        cov[0] = 0.02**2 + (0.03 * v) ** 2
        cov[7] = 0.01**2
        cov[35] = 0.05**2 + (0.1 * odom.twist.twist.angular.z) ** 2
        for k in (14, 21, 28):
            cov[k] = 1e6
        odom.twist.covariance = cov
        self.pub_odom.publish(odom)

    def _publier_etat(self) -> None:
        etat = DiagnosticStatus()
        etat.name = "controleur_embarque"
        etat.hardware_id = "mcu"
        etat.level = DiagnosticStatus.OK if self.arret is None else DiagnosticStatus.WARN
        etat.message = self.arret or "ok"
        valeurs = {
            "arret": self.arret or "",
            "vitesse_consigne": f"{self.v_consigne:.3f}",
            "braquage_consigne": f"{self.b_consigne:.4f}",
            "vitesse_mesuree": f"{self.v_mesuree:.3f}",
            "libre_avant": f"{self.libre_avant:.2f}",
        }
        etat.values = [KeyValue(key=k, value=v) for k, v in valeurs.items()]
        self.pub_etat.publish(etat)


def main() -> None:
    rclpy.init()
    noeud = ControleurEmbarque()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        noeud.destroy_node()
        rclpy.try_shutdown()
