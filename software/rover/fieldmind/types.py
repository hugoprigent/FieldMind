"""Types de données échangés entre les modules du rover.

Conventions, partout dans le logiciel :
- **Temps** : `t` en secondes de `time.monotonic()` (horloge du Pi), jamais l'heure murale.
  L'heure murale (`time.time()`) ne sert qu'aux journaux et aux fichiers.
- **Repère du robot `base_link`** (REP 103/105 de ROS) : origine au sol, au milieu de l'essieu
  arrière (axe des moteurs, sous le lidar) ; x vers l'avant, y vers la gauche, z vers le haut.
  Angles en radians, positifs dans le sens trigonométrique (vers la gauche).
- **Repère `odom`** : fixe, posé là où le rover a démarré (ou été remis à zéro). Continu, il dérive
  lentement. La carte locale et les buts de navigation sont dans ce repère.
- **Braquage** : angle de la roue avant virtuelle centrale (modèle bicyclette), + vers la gauche.
- **Vitesse** : vitesse longitudinale de l'essieu arrière, + vers l'avant, en m/s.
- Distances en mètres, sauf mention contraire.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Literal

import numpy as np

Mode = Literal["arret", "manuel", "auto", "banc"]
"""- arret : aucune traction (état au démarrage) ;
- manuel : téléopération depuis le poste, sous le contrôle de la sécurité ;
- auto : les actions de la file de mission conduisent (consignes écrites, vocales, IA, carte) ;
- banc : essais roues levées, pourcentages PWM bruts plafonnés, sécurité obstacles ignorée."""

SourceConsigne = Literal["manuel", "auto", "ros2", "banc", "aucune"]


# ---------------------------------------------------------------- géométrie


@dataclass(slots=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    cap: float = 0.0  # rad, trigonométrique, 0 = axe x du repère

    def composer(self, autre: Pose2D) -> Pose2D:
        """self ⊕ autre : `autre` exprimée dans le repère de self, ramenée dans le repère parent."""
        c, s = math.cos(self.cap), math.sin(self.cap)
        return Pose2D(
            self.x + c * autre.x - s * autre.y,
            self.y + s * autre.x + c * autre.y,
            normaliser_angle(self.cap + autre.cap),
        )

    def inverse(self) -> Pose2D:
        c, s = math.cos(self.cap), math.sin(self.cap)
        return Pose2D(-c * self.x - s * self.y, s * self.x - c * self.y, -self.cap)

    def vers_local(self, x: float, y: float) -> tuple[float, float]:
        """Point du repère parent exprimé dans le repère de la pose."""
        dx, dy = x - self.x, y - self.y
        c, s = math.cos(self.cap), math.sin(self.cap)
        return c * dx + s * dy, -s * dx + c * dy

    def vers_parent(self, x: float, y: float) -> tuple[float, float]:
        c, s = math.cos(self.cap), math.sin(self.cap)
        return self.x + c * x - s * y, self.y + s * x + c * y


def normaliser_angle(a: float) -> float:
    """Angle ramené dans ]-π, π]."""
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a <= 0:
        a += 2 * math.pi
    return a - math.pi


# ---------------------------------------------------------------- capteurs


@dataclass(slots=True)
class Scan:
    """Un tour complet du lidar, déjà converti dans le repère `base_link`.

    Les angles du RPLIDAR tournent dans le sens horaire vu de dessus : le pilote les convertit
    (angle_base_link = décalage − angle_lidar). Les points sur le rover lui-même sont retirés.
    """

    t: float  # fin du tour
    angles: np.ndarray  # rad (float32), base_link, triés croissants dans [-π, π[
    distances: np.ndarray  # m (float32), > 0 ; les mesures invalides sont absentes
    duree: float = 0.1  # durée du tour (s). Points triés par angle, pas dans l'ordre de mesure :
    # un point date de t - duree·(1 - φ/2π), φ = angle parcouru par le C1 (horaire) depuis son 0°
    qualites: np.ndarray | None = None

    def points(self) -> np.ndarray:
        """(N, 2) points x, y dans base_link (le lidar est sur l'axe arrière, cf. profil)."""
        return np.column_stack((self.distances * np.cos(self.angles),
                                self.distances * np.sin(self.angles))).astype(np.float32)


@dataclass(slots=True)
class ImageCamera:
    t: float  # instant de capture (réception de la trame)
    camera: str  # "gauche" | "droite" (attribution par port USB, réglable)
    jpeg: bytes  # MJPEG de la webcam, transmis tel quel
    largeur: int
    hauteur: int
    numero: int = 0

    def bgr(self, reduction: int = 1) -> np.ndarray:
        """Image décodée (BGR). `reduction` = 2, 4 ou 8 décode directement en taille réduite."""
        import cv2  # import local : les modules qui ne décodent pas n'en dépendent pas

        drapeau = {1: cv2.IMREAD_COLOR, 2: cv2.IMREAD_REDUCED_COLOR_2,
                   4: cv2.IMREAD_REDUCED_COLOR_4, 8: cv2.IMREAD_REDUCED_COLOR_8}[reduction]
        image = cv2.imdecode(np.frombuffer(self.jpeg, np.uint8), drapeau)
        if image is None:
            raise ValueError("JPEG illisible")
        return image


@dataclass(slots=True)
class Imu:
    """Centrale MPU-6050 de la carte capteurs, lue par l'ESP. Axes déjà ramenés dans base_link."""

    t: float
    accel_g: tuple[float, float, float]  # accélération spécifique (g) ; ≈ (0, 0, 1) à plat
    gyro_rad_s: tuple[float, float, float]
    lacet_rad: float | None = None  # cap intégré par l'ESP (gyroscope seul, dérive), si fourni
    temperature_c: float | None = None
    etat: str = "OK"  # OK | DOUTEUX | ABSENT

    def roulis_tangage(self) -> tuple[float, float]:
        ax, ay, az = self.accel_g
        return math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az))


@dataclass(slots=True)
class Gnss:
    t: float
    fix: str = "aucun"  # aucun | autonome | dgnss | rtk_fixe | rtk_flottant | estime
    latitude: float | None = None
    longitude: float | None = None
    altitude_m: float | None = None
    hdop: float | None = None
    satellites: int = 0
    vitesse_m_s: float | None = None
    cap_deg: float | None = None  # cap de déplacement (vrai), valable seulement en roulant

    @property
    def valide(self) -> bool:
        return self.fix != "aucun" and self.latitude is not None and self.longitude is not None


@dataclass(slots=True)
class EtatEsp:
    """Ce que le firmware ESP (ROVER_2 et suivants) rapporte des actionneurs."""

    t: float
    connecte: bool = False
    firmware: str | None = None  # "ROVER_2", "ROVER_3"…
    defaut: bool = False
    raison_defaut: str | None = None
    pwm_g: int = 0  # % appliqué (non signé)
    pwm_d: int = 0
    sens_g: int = 0  # 0 avant logique, 1 arrière
    sens_d: int = 0
    inverse_g: bool | None = None
    inverse_d: bool | None = None
    frein: bool = False
    servo_actif: bool = False
    servo_deg: float = 0.0  # consigne en cours (angle indicatif du servo, pas une mesure)
    servo_cible_deg: float = 0.0
    chien_de_garde_ms: int | None = None  # None : firmware sans chien de garde
    hz_g: float | None = None  # impulsions/s des sorties SC des drivers (firmware « roues 1 »)
    hz_d: float | None = None
    age_derniere_ligne_s: float = math.inf


@dataclass(slots=True)
class EtatDrone:
    """État de la liaison Wi-Fi, du flux vidéo et des commandes du mini-drone (via ESP32-C5)."""

    t: float
    etat: str = "absent"  # absent | scan | connexion | connecte | inactif
    connecte: bool = False
    ssid: str | None = None
    bssid: str | None = None
    canal: int = 0
    rssi_dbm: int | None = None
    ip_locale: str | None = None
    passerelle: str | None = None
    protocole: str | None = None
    port_video: int = 0
    port_cmd: int = 8888
    paquets_rx: int = 0
    ko_rx: int = 0
    paquets_tx: int = 0
    ips: float = 0.0
    images_rx: int = 0
    batterie_pct: int | None = None
    commande_vol: str = "aucune"
    candidat_scan: str | None = None
    reseaux_scan: list[str] = field(default_factory=list)
    largeur: int = 640
    hauteur: int = 360
    mode_recon: str = "inactif"
    secteur_360: int = 0
    croix_trouvee: bool = False
    croix_verrouillee: bool = False
    croix_x: float = 0.0
    croix_y: float = 0.0
    croix_taille_pct: float = 0.0
    consigne_vol: str = "attente"
    pilotage_valide: bool = False
    analyse_ms: float = 0.0
    montage_recon: str | None = None
    age_s: float = math.inf


@dataclass(slots=True)
class EchantillonAudio:
    """Bloc audio du micro d'une webcam : PCM 16 bits mono."""

    t: float  # instant du dernier échantillon
    pcm: bytes
    frequence_hz: int = 16000


# ---------------------------------------------------------------- estimation et perception


@dataclass(slots=True)
class Odometrie:
    t: float
    pose: Pose2D  # repère odom
    vitesse: float = 0.0  # m/s mesurée (signée)
    vitesse_lacet: float = 0.0  # rad/s
    covariance: np.ndarray | None = None  # (3, 3) sur x, y, cap
    qualite: float = 1.0  # 0..1 : confiance de l'appariement lidar (1 = bon)
    source: str = "icp+gyro"


@dataclass(slots=True)
class Obstacles:
    """Points d'obstacle dans base_link à l'instant t (lidar filtré, stéréo, vision…)."""

    t: float
    source: str  # "lidar" | "stereo" | "vision"
    points: np.ndarray  # (N, 2) float32, base_link
    hauteurs: np.ndarray | None = None  # (N,) hauteur au-dessus du sol si connue
    portee_m: float = 4.0  # au-delà, la source ne garantit rien


@dataclass(slots=True)
class CarteOccupation:
    """Grille d'occupation dans le repère odom."""

    t: float
    resolution: float  # m par cellule
    origine_x: float  # coin (0, 0) de la grille dans odom
    origine_y: float
    cellules: np.ndarray  # (H, W) int8 : -1 inconnu, 0 libre … 100 occupé ; ligne = y, colonne = x

    def indice(self, x: float, y: float) -> tuple[int, int]:
        return (int((y - self.origine_y) / self.resolution),
                int((x - self.origine_x) / self.resolution))


@dataclass(slots=True)
class Chemin:
    t: float
    points: np.ndarray  # (N, 2) odom
    but: tuple[float, float]
    longueur_m: float = 0.0


# ---------------------------------------------------------------- commande et sécurité


@dataclass(slots=True)
class Consigne:
    """Demande de mouvement, avant passage par la sécurité."""

    t: float
    vitesse: float = 0.0  # m/s
    braquage: float = 0.0  # rad
    source: SourceConsigne = "aucune"
    pwm_brut: tuple[int, int] | None = None  # mode banc : pourcentages signés gauche/droite


@dataclass(slots=True)
class CommandeRoues:
    """Ce qui est envoyé à l'ESP après sécurité, rampes et différentiel."""

    t: float
    pwm_g: int = 0  # % signé, + = avant logique
    pwm_d: int = 0
    servo_deg: float = 0.0  # angle indicatif du servo (échelle du firmware)
    vitesse: float = 0.0  # vitesse visée après limitation (m/s)
    braquage: float = 0.0


@dataclass(slots=True)
class EtatSecurite:
    t: float
    arret: bool = True  # aucune traction autorisée
    vitesse_max_avant: float = 0.0  # m/s autorisés maintenant
    vitesse_max_arriere: float = 0.0  # m/s (valeur positive)
    raisons: list[str] = field(default_factory=list)  # codes : "obstacle_avant", "lidar_absent"…
    libre_avant_m: float | None = None  # espace libre le long de l'arc prévu, vers l'avant
    libre_arriere_m: float | None = None
    arret_urgence: bool = False  # verrou logiciel : réarmement explicite nécessaire


# ---------------------------------------------------------------- missions


Action = dict[str, Any]
"""Une action de mission, en JSON. Types reconnus (voir mission/actions.py) :
{"type": "avancer", "distance_m": 5.0, "vitesse_m_s": 0.4}   (distance négative = reculer)
{"type": "tourner", "angle_deg": 90}                           (+ vers la gauche)
{"type": "aller_a", "x": 2.0, "y": 1.0, "repere": "robot"}     ("robot" ou "odom")
{"type": "aller_pixel", "camera": "gauche", "u": 0.52, "v": 0.71}  (u, v normalisés 0..1)
{"type": "cap", "cap_deg": 0}                                  (orientation absolue odom)
{"type": "attendre", "duree_s": 2}
{"type": "retour_depart"}
{"type": "aller_gps", "lat": 43.7105, "lon": -1.0531, "tolerance_m": 2.5}
{"type": "memoriser_lieu", "nom": "garage"}
{"type": "aller_lieu", "nom": "garage"}
{"type": "chercher", "objet": "chaise rouge"}   (vision IA, puis aller_a)
{"type": "decrire"}                             (vision IA, réponse dans le chat)
{"type": "photo", "camera": "gauche"}
{"type": "stop"}
"""
