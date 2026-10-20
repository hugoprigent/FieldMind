"""Conversions entre la WebSocket `/ws/ros` du rover (JSON) et les messages ROS 2, sans ROS.

Le nœud `pont` (rclpy) ne fait que recopier dans les messages ROS les champs des objets produits
ici : toute la logique (angles uniformes du lidar, recalage d'horloge, covariances, bornes des
consignes, repères des buts) est dans ce module, testée sans ROS
(`software/rover/tests/test_ros2_conversions.py`).

Repères (`fieldmind/types.py`) : `base_link` au sol, au milieu de l'essieu arrière, x avant,
y gauche ; `odom` fixe au départ. Temps du rover : `time.monotonic()` du Pi.
"""

from __future__ import annotations

import base64
import binascii
import json
import math
from collections import deque
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any
from urllib.parse import quote, urlsplit, urlunsplit

import numpy as np

G = 9.80665
"""Accélération de la pesanteur (m/s²) : la centrale du rover donne l'accélération en g."""

PORT_ROVER = 8080
CHEMIN_WS = "/ws/ros"


class MessageInvalide(ValueError):
    """Message de /ws/ros incomplet ou mal formé : il est ignoré (et compté)."""


# ---------------------------------------------------------------- adresse


def construire_url(adresse: str, jeton: str = "", chemin: str = CHEMIN_WS) -> str:
    """`rover.local:8080` ou `ws://…` → URL de la WebSocket, jeton facultatif en requête.

    Sans schéma, `ws://` ; sans port, 8080 (port du rover) ; sans chemin, `/ws/ros`.
    """
    adresse = adresse.strip()
    if not adresse:
        raise ValueError("adresse du rover vide")
    if "://" not in adresse:
        hote = adresse.split("/", 1)[0]
        avec_port = "]:" in hote if hote.startswith("[") else ":" in hote  # IPv6 : [::1]:8080
        if not avec_port:
            adresse = adresse.replace(hote, f"{hote}:{PORT_ROVER}", 1)
        adresse = "ws://" + adresse
    morceaux = urlsplit(adresse)
    schema = {"http": "ws", "https": "wss"}.get(morceaux.scheme, morceaux.scheme)
    if schema not in ("ws", "wss"):
        raise ValueError(f"schéma non pris en charge : {morceaux.scheme}")
    voie = morceaux.path if morceaux.path not in ("", "/") else chemin
    requete = morceaux.query
    if jeton:
        requete = (requete + "&" if requete else "") + "jeton=" + quote(jeton, safe="")
    return urlunsplit((schema, morceaux.netloc, voie, requete, ""))


def masquer_jeton(url: str) -> str:
    """URL affichable dans les journaux (jeton remplacé par ***)."""
    morceaux = urlsplit(url)
    if "jeton=" not in morceaux.query:
        return url
    parties = ["jeton=***" if p.startswith("jeton=") else p for p in morceaux.query.split("&")]
    return urlunsplit(morceaux._replace(query="&".join(parties)))


# ---------------------------------------------------------------- temps


def secondes_vers_stamp(t: float) -> tuple[int, int]:
    """Secondes (float) → (sec, nanosec) de builtin_interfaces/Time."""
    if not math.isfinite(t) or t < 0:
        raise ValueError(f"date invalide : {t}")
    nanosecondes = round(t * 1e9)
    return nanosecondes // 1_000_000_000, nanosecondes % 1_000_000_000


class RecaleurHorloge:
    """Décalage entre l'horloge monotone du Pi et l'horloge ROS de la machine du pont.

    Le rover date chaque message à l'envoi (`t_envoi`, monotone du Pi). À la réception,
    `t_reception − t_envoi` = décalage + latence du réseau, donc toujours ≥ décalage. Le minimum
    de ces échantillons sur une fenêtre glissante donne le décalage à la latence minimale près
    (quelques millisecondes en Wi-Fi local) : les retards ponctuels (paquets bloqués, rafales
    après une coupure) sont ignorés. Une dérive lente des deux horloges est suivie en une fenêtre.

    L'estimation repart de zéro à chaque connexion (`remettre_a_zero`), quand l'horloge du Pi
    recule (redémarrage du Pi) et quand un échantillon dépasse l'estimation de plus de `saut_s`
    (horloge de cette machine avancée d'un coup, par NTP par exemple).
    """

    def __init__(self, fenetre_s: float = 20.0, saut_s: float = 5.0) -> None:
        self.fenetre_s = fenetre_s
        self.saut_s = saut_s
        # (date de réception, échantillon), échantillons strictement croissants : minimum en tête.
        self._file: deque[tuple[float, float]] = deque()
        self._dernier_envoi: float | None = None
        self.remises_a_zero = 0

    def remettre_a_zero(self) -> None:
        self._file.clear()
        self._dernier_envoi = None
        self.remises_a_zero += 1

    @property
    def decalage(self) -> float | None:
        """Secondes à ajouter à une date du Pi pour obtenir une date ROS (None : inconnu)."""
        return self._file[0][1] if self._file else None

    def observer(self, t_envoi: float, t_reception: float, *, ordonne: bool = True) -> None:
        """Un message daté `t_envoi` (Pi) reçu à `t_reception` (ROS).

        `ordonne` : `t_envoi` croît d'un message à l'autre (vrai pour le champ `t_envoi`, faux
        pour le `t` d'une mesure, qui peut être un peu plus ancien que la précédente).
        """
        if not (math.isfinite(t_envoi) and math.isfinite(t_reception)):
            return
        if ordonne:
            if self._dernier_envoi is not None and t_envoi < self._dernier_envoi - 1.0:
                self.remettre_a_zero()  # le Pi a redémarré : son horloge monotone est repartie
            self._dernier_envoi = t_envoi
        echantillon = t_reception - t_envoi
        actuel = self.decalage
        if actuel is not None and echantillon - actuel > self.saut_s:
            dernier_envoi = self._dernier_envoi
            self.remettre_a_zero()
            self._dernier_envoi = dernier_envoi
        while self._file and self._file[-1][1] >= echantillon:
            self._file.pop()
        self._file.append((t_reception, echantillon))
        while self._file[0][0] < t_reception - self.fenetre_s:
            self._file.popleft()

    def vers_ros(self, t: float, maintenant: float) -> float:
        """Date ROS d'une mesure datée `t` sur le Pi ; `maintenant` tant que rien n'est estimé.

        Jamais dans le futur : tf2 refuse d'extrapoler en avant.
        """
        decalage = self.decalage
        if decalage is None or not math.isfinite(t):
            return maintenant
        return min(t + decalage, maintenant)


# ---------------------------------------------------------------- géométrie


def quaternion_cap(cap: float) -> tuple[float, float, float, float]:
    """Cap (lacet, rad) → quaternion (x, y, z, w) d'une rotation autour de z."""
    return 0.0, 0.0, math.sin(cap / 2.0), math.cos(cap / 2.0)


def cap_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Lacet (rad, dans ]-π, π]) d'un quaternion (x, y, z, w) quelconque."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def transformer_point_2d(transformation: tuple[float, float, float], x: float,
                         y: float) -> tuple[float, float]:
    """Point (x, y) d'un repère B ramené dans A, `transformation` = pose de B dans A."""
    tx, ty, cap = transformation
    c, s = math.cos(cap), math.sin(cap)
    return tx + c * x - s * y, ty + s * x + c * y


# ---------------------------------------------------------------- réglages


@dataclass(slots=True)
class Reglages:
    """Réglages de conversion (paramètres ROS du nœud `pont`)."""

    nb_rayons: int = 400  # rayons du LaserScan uniforme sur 360° (C1 : ≈ 0,72° par point à 10 Hz)
    portee_min: float = 0.12  # m (RPLIDAR C1)
    portee_max: float = 12.0
    lidar_x: float = 0.0  # position du lidar dans base_link (m) : centre des rayons du LaserScan
    lidar_y: float = 0.0
    duree_scan: float = 0.1  # s, si le message `scan` ne porte pas `duree`
    recaler_horloge: bool = True  # False : mesures datées à leur réception par le pont
    hfov_deg: float = 78.0  # champ horizontal des webcams (CameraInfo approchée)
    variance_acceleration: float = 0.1  # (m/s²)² : vibrations du châssis comprises
    variance_vitesse_angulaire: float = 1e-4  # (rad/s)²
    variance_pose: tuple[float, float, float] = (0.01, 0.01, 0.0025)  # x, y (m²), cap (rad²)
    variance_vitesse: tuple[float, float] = (0.0025, 4e-4)  # (m/s)², (rad/s)²
    variance_plan: float = 1e-4  # z, roulis, tangage, vitesse latérale : fixés par le modèle 2D
    uere_m: float = 2.5  # erreur de distance équivalente d'un GPS grand public (σ = HDOP × UERE)
    sigma_gnss_min_m: float = 1.5
    sigma_gnss_defaut_m: float = 5.0  # sans HDOP


# ---------------------------------------------------------------- mesures prêtes pour ROS


@dataclass(slots=True)
class ScanRos:
    stamp: float
    angle_min: float
    angle_max: float
    increment: float
    duree: float
    portee_min: float
    portee_max: float
    distances: np.ndarray  # float32 (nb_rayons,), +inf : pas d'écho dans ce secteur


@dataclass(slots=True)
class ImuRos:
    stamp: float
    acceleration: tuple[float, float, float]  # m/s², base_link
    vitesse_angulaire: tuple[float, float, float]  # rad/s
    covariance_acceleration: list[float]  # 9
    covariance_vitesse_angulaire: list[float]  # 9


@dataclass(slots=True)
class OdomRos:
    stamp: float
    x: float
    y: float
    cap: float
    quaternion: tuple[float, float, float, float]  # x, y, z, w
    vitesse: float
    vitesse_lacet: float
    covariance_pose: list[float]  # 36
    covariance_vitesse: list[float]  # 36


@dataclass(slots=True)
class GnssRos:
    stamp: float
    statut: int  # sensor_msgs/NavSatStatus
    service: int
    latitude: float  # NaN si inconnue
    longitude: float
    altitude: float
    covariance: list[float]  # 9, ENU (m²)
    type_covariance: int  # sensor_msgs/NavSatFix.COVARIANCE_TYPE_*


@dataclass(slots=True)
class ImageRos:
    stamp: float
    camera: str
    jpeg: bytes
    largeur: int | None
    hauteur: int | None


@dataclass(slots=True)
class Diagnostic:
    """Un diagnostic_msgs/DiagnosticStatus."""

    nom: str
    niveau: int  # 0 OK, 1 WARN, 2 ERROR, 3 STALE
    message: str
    valeurs: list[tuple[str, str]] = field(default_factory=list)


@dataclass(slots=True)
class EtatRos:
    stamp: float
    json: str  # message `etat` tel que reçu
    diagnostics: list[Diagnostic]


Mesure = ScanRos | ImuRos | OdomRos | GnssRos | ImageRos | EtatRos

OK, ALERTE, ERREUR, PERIME = 0, 1, 2, 3


# ---------------------------------------------------------------- lidar


def decoder_float32(texte: str) -> np.ndarray:
    """Base64 de float32 little-endian → tableau float32."""
    try:
        brut = base64.b64decode(texte, validate=True)
    except (binascii.Error, TypeError, ValueError) as erreur:
        raise MessageInvalide(f"base64 illisible : {erreur}") from None
    if len(brut) % 4:
        raise MessageInvalide("longueur non multiple de 4 octets (float32)")
    return np.frombuffer(brut, dtype="<f4").astype(np.float32)


def encoder_float32(valeurs: Sequence[float] | np.ndarray) -> str:
    """Inverse de `decoder_float32` (serveur de test, outils)."""
    return base64.b64encode(np.asarray(valeurs, dtype="<f4").tobytes()).decode("ascii")


def scan_uniforme(angles: np.ndarray, distances: np.ndarray, nb_rayons: int,
                  lidar_xy: tuple[float, float] = (0.0, 0.0)) -> tuple[float, float, np.ndarray]:
    """Points (angle, distance) de base_link → LaserScan à pas angulaire constant.

    Le rover envoie seulement les mesures valides, déjà tournées dans base_link (angles croissants
    dans [-π, π[, sens trigonométrique), points du rover retirés. Le LaserScan couvre 360° en
    `nb_rayons` secteurs centrés sur `angle_min + i·pas`, `angle_min = -π` : chaque point va au
    secteur le plus proche, le plus proche des points d'un secteur l'emporte (prudent pour les
    obstacles), un secteur vide vaut +inf (« pas d'écho », REP 117).

    Si le lidar n'est pas à l'origine de base_link (`lidar_xy`), les points sont ré-exprimés en
    coordonnées polaires autour de lui : le LaserScan est publié dans `lidar_link`.

    Renvoie (angle_min, pas, distances float32).
    """
    if nb_rayons < 8:
        raise ValueError("nb_rayons trop petit")
    angles = np.asarray(angles, dtype=np.float64)
    distances = np.asarray(distances, dtype=np.float64)
    if angles.shape != distances.shape or angles.ndim != 1:
        raise MessageInvalide(f"angles {angles.shape} et distances {distances.shape} différents")
    valides = np.isfinite(angles) & np.isfinite(distances) & (distances > 0.0)
    angles, distances = angles[valides], distances[valides]
    lx, ly = lidar_xy
    if lx or ly:
        x = distances * np.cos(angles) - lx
        y = distances * np.sin(angles) - ly
        angles, distances = np.arctan2(y, x), np.hypot(x, y)
    angle_min = -math.pi
    pas = 2.0 * math.pi / nb_rayons
    sortie = np.full(nb_rayons, np.inf, dtype=np.float64)
    if angles.size:
        indices = np.floor((angles - angle_min) / pas + 0.5).astype(np.int64) % nb_rayons
        np.minimum.at(sortie, indices, distances)
    return angle_min, pas, sortie.astype(np.float32)


# ---------------------------------------------------------------- covariances


def covariance_pose(cov9: Sequence[float] | None, defaut: tuple[float, float, float],
                    plan: float) -> list[float]:
    """Covariance 3 × 3 (x, y, cap) du rover → 6 × 6 de ROS (x, y, z, roulis, tangage, lacet)."""
    c = [0.0] * 36
    indices = (0, 1, 5)
    valeurs = _nombres(cov9, 9)
    if valeurs is not None and all(valeurs[i * 4] >= 0 for i in range(3)):
        for i, a in enumerate(indices):
            for j, b in enumerate(indices):
                c[a * 6 + b] = valeurs[i * 3 + j]
    else:
        for i, a in enumerate(indices):
            c[a * 6 + a] = defaut[i]
    for a in (2, 3, 4):
        c[a * 6 + a] = plan
    return c


def covariance_vitesse(variances: tuple[float, float], plan: float) -> list[float]:
    """Vitesse longitudinale et de lacet ; les autres composantes sont nulles par construction."""
    c = [0.0] * 36
    for a in range(6):
        c[a * 6 + a] = plan
    c[0], c[35] = variances
    return c


def diagonale(variance: float) -> list[float]:
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def _nombres(valeurs: Any, n: int) -> list[float] | None:
    if not isinstance(valeurs, Sequence) or isinstance(valeurs, str) or len(valeurs) != n:
        return None
    try:
        nombres = [float(v) for v in valeurs]
    except (TypeError, ValueError):
        return None
    return nombres if all(math.isfinite(v) for v in nombres) else None


# ---------------------------------------------------------------- GNSS

# sensor_msgs/NavSatStatus et NavSatFix
STATUT_SANS_FIX, STATUT_FIX, STATUT_SBAS, STATUT_GBAS = -1, 0, 1, 2
SERVICE_GPS, SERVICE_GLONASS, SERVICE_COMPASS, SERVICE_GALILEO = 1, 2, 4, 8
COVARIANCE_INCONNUE, COVARIANCE_APPROCHEE = 0, 1

STATUTS_FIX = {
    "aucun": STATUT_SANS_FIX,
    "estime": STATUT_SANS_FIX,  # navigation à l'estime du module, sans centrale : inutilisable
    "autonome": STATUT_FIX,
    "dgnss": STATUT_SBAS,
    "rtk_flottant": STATUT_GBAS,
    "rtk_fixe": STATUT_GBAS,
}
SIGMA_RTK_M = {"rtk_fixe": 0.03, "rtk_flottant": 0.3}


def navsat(fix: str, hdop: float | None, reglages: Reglages) -> tuple[int, list[float], int]:
    """(statut NavSatStatus, covariance ENU 9, type de covariance) d'un fix du rover.

    ATGM336H (GPS + BeiDou, 2 à 3 m) : σ horizontal = HDOP × UERE, au moins `sigma_gnss_min_m` ;
    σ vertical = 2 σ horizontal.
    """
    statut = STATUTS_FIX.get(fix, STATUT_SANS_FIX)
    if statut == STATUT_SANS_FIX:
        return statut, [0.0] * 9, COVARIANCE_INCONNUE
    if fix in SIGMA_RTK_M:
        sigma = SIGMA_RTK_M[fix]
    elif hdop is not None and math.isfinite(hdop) and hdop > 0:
        sigma = max(hdop * reglages.uere_m, reglages.sigma_gnss_min_m)
    else:
        sigma = reglages.sigma_gnss_defaut_m
    h, v = sigma * sigma, 4.0 * sigma * sigma
    return statut, [h, 0.0, 0.0, 0.0, h, 0.0, 0.0, 0.0, v], COVARIANCE_APPROCHEE


# ---------------------------------------------------------------- caméras


@dataclass(slots=True)
class InfoCamera:
    """sensor_msgs/CameraInfo approchée (sténopé sans distorsion) d'après le champ horizontal."""

    largeur: int
    hauteur: int
    k: list[float]
    p: list[float]
    r: list[float] = field(default_factory=lambda: [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0])
    d: list[float] = field(default_factory=lambda: [0.0] * 5)
    modele: str = "plumb_bob"


def info_camera(largeur: int, hauteur: int, hfov_deg: float) -> InfoCamera:
    fx = (largeur / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    cx, cy = (largeur - 1) / 2.0, (hauteur - 1) / 2.0
    return InfoCamera(
        largeur, hauteur,
        k=[fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0],
        p=[fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    )


_SOF = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}
_SANS_LONGUEUR = {0x01, *range(0xD0, 0xDA)}


def dimensions_jpeg(donnees: bytes) -> tuple[int, int] | None:
    """(largeur, hauteur) lues dans l'en-tête SOF d'un JPEG, sans le décoder ; None sinon."""
    if donnees[:2] != b"\xff\xd8":
        return None
    i, n = 2, len(donnees)
    while i + 3 < n:
        if donnees[i] != 0xFF:
            return None
        marqueur = donnees[i + 1]
        if marqueur == 0xFF:  # octet de bourrage
            i += 1
            continue
        if marqueur in _SANS_LONGUEUR:
            i += 2
            continue
        longueur = int.from_bytes(donnees[i + 2:i + 4], "big")
        if marqueur in _SOF:
            if i + 9 > n:
                return None
            hauteur = int.from_bytes(donnees[i + 5:i + 7], "big")
            largeur = int.from_bytes(donnees[i + 7:i + 9], "big")
            return (largeur, hauteur) if largeur and hauteur else None
        if marqueur == 0xDA:  # début des données sans SOF avant : pas un JPEG valide
            return None
        i += 2 + longueur
    return None


def nom_camera(nom: Any) -> str:
    """Nom de caméra utilisable dans un nom de sujet ROS (`gauche`, `droite`…)."""
    texte = "".join(c if c.isascii() and (c.isalnum() or c == "_") else "_" for c in str(nom))
    texte = texte.strip("_").lower()
    if not texte or texte[0].isdigit():
        raise MessageInvalide(f"nom de caméra invalide : {nom!r}")
    return texte


# ---------------------------------------------------------------- état


def diagnostics_etat(etat: Mapping[str, Any]) -> list[Diagnostic]:
    """Extrait de `etat` du rover → diagnostics (mode, sécurité, capteurs).

    Les champs absents sont ignorés.
    """
    securite = _dict(etat.get("securite"))
    consigne = _dict(etat.get("consigne"))
    mission = _dict(etat.get("mission"))
    raisons = [str(r) for r in securite.get("raisons") or []]
    if etat.get("arret_urgence"):
        niveau, message = ERREUR, "arrêt d'urgence verrouillé"
    elif securite.get("arret"):
        niveau, message = ALERTE, "traction coupée : " + (", ".join(raisons) or "sécurité")
    elif raisons:
        niveau, message = ALERTE, "limité : " + ", ".join(raisons)
    else:
        niveau, message = OK, f"mode {etat.get('mode', '?')}"
    valeurs = [
        ("mode", etat.get("mode")),
        ("arme", etat.get("arme")),
        ("arret_urgence", etat.get("arret_urgence")),
        ("source_consigne", consigne.get("source")),
        ("vitesse_max_avant", securite.get("vitesse_max_avant")),
        ("vitesse_max_arriere", securite.get("vitesse_max_arriere")),
        ("libre_avant_m", securite.get("libre_avant_m")),
        ("raisons", ", ".join(raisons) if raisons else None),
        ("mission", mission.get("etat")),
    ]
    resultat = [Diagnostic("fieldmind/rover", niveau, message,
                           [(cle, _texte(v)) for cle, v in valeurs if v is not None])]
    for nom, capteur in _dict(etat.get("capteurs")).items():
        if not isinstance(capteur, Mapping):
            continue
        ok = bool(capteur.get("ok"))
        resultat.append(Diagnostic(
            f"fieldmind/capteurs/{nom}", OK if ok else ALERTE, "ok" if ok else "absent ou périmé",
            [(str(cle), _texte(v)) for cle, v in capteur.items()],
        ))
    return resultat


def _dict(valeur: Any) -> Mapping[str, Any]:
    return valeur if isinstance(valeur, Mapping) else {}


def _texte(valeur: Any) -> str:
    if isinstance(valeur, bool):
        return "oui" if valeur else "non"
    if isinstance(valeur, float):
        return f"{valeur:.3g}"
    if isinstance(valeur, list | dict):
        return json.dumps(valeur, ensure_ascii=False)
    return str(valeur)


# ---------------------------------------------------------------- décodage des messages


TYPES_MESURES = ("scan", "imu", "odom", "gnss", "image")
"""Messages dont `t` est une date monotone du Pi (utilisable pour le recalage)."""


class Decodeur:
    """Messages JSON de `/ws/ros` → mesures datées en temps ROS (`Mesure`), ou None si le type
    n'est pas publié en ROS. `MessageInvalide` si le message est mal formé."""

    def __init__(self, reglages: Reglages | None = None,
                 horloge: RecaleurHorloge | None = None) -> None:
        self.reglages = reglages or Reglages()
        self.horloge = horloge or RecaleurHorloge()

    def reinitialiser(self) -> None:
        """À chaque (re)connexion : le Pi a peut-être redémarré entre-temps."""
        self.horloge.remettre_a_zero()

    def decoder(self, message: Mapping[str, Any], t_reception: float) -> Mesure | None:
        if not isinstance(message, Mapping):
            raise MessageInvalide("objet JSON attendu")
        genre = message.get("type")
        t = _nombre_ou_none(message.get("t"))
        if self.reglages.recaler_horloge:
            t_envoi = _nombre_ou_none(message.get("t_envoi"))
            if t_envoi is not None:
                self.horloge.observer(t_envoi, t_reception)
            elif t is not None and genre in TYPES_MESURES:
                self.horloge.observer(t, t_reception, ordonne=False)
            stamp = (self.horloge.vers_ros(t, t_reception) if t is not None else t_reception)
        else:
            stamp = t_reception
        try:
            if genre == "scan":
                return self._scan(message, stamp)
            if genre == "imu":
                return self._imu(message, stamp)
            if genre == "odom":
                return self._odom(message, stamp)
            if genre == "gnss":
                return self._gnss(message, stamp)
            if genre == "image":
                return self._image(message, stamp)
            if genre == "etat":
                return EtatRos(stamp, json.dumps(message, ensure_ascii=False),
                               diagnostics_etat(message))
        except (KeyError, TypeError, ValueError, IndexError) as erreur:
            if isinstance(erreur, MessageInvalide):
                raise
            raise MessageInvalide(f"{genre} : {type(erreur).__name__} {erreur}") from None
        return None

    def _scan(self, m: Mapping[str, Any], stamp: float) -> ScanRos:
        r = self.reglages
        angles = decoder_float32(m["angles_b64"])
        distances = decoder_float32(m["distances_b64"])
        angle_min, pas, rayons = scan_uniforme(angles, distances, r.nb_rayons,
                                               (r.lidar_x, r.lidar_y))
        duree = _nombre_ou_none(m.get("duree"))
        duree = duree if duree is not None and 0.0 < duree < 1.0 else r.duree_scan
        return ScanRos(stamp, angle_min, angle_min + pas * (r.nb_rayons - 1), pas, duree,
                       r.portee_min, r.portee_max, rayons)

    def _imu(self, m: Mapping[str, Any], stamp: float) -> ImuRos:
        accel = _nombres(m["accel_g"], 3)
        gyro = _nombres(m["gyro_rad_s"], 3)
        if accel is None or gyro is None:
            raise MessageInvalide("imu : accel_g et gyro_rad_s attendus (3 nombres)")
        r = self.reglages
        return ImuRos(
            stamp,
            (accel[0] * G, accel[1] * G, accel[2] * G),
            (gyro[0], gyro[1], gyro[2]),
            diagonale(r.variance_acceleration),
            diagonale(r.variance_vitesse_angulaire),
        )

    def _odom(self, m: Mapping[str, Any], stamp: float) -> OdomRos:
        x, y, cap = float(m["x"]), float(m["y"]), float(m["cap"])
        vitesse = float(m.get("vitesse") or 0.0)
        vitesse_lacet = float(m.get("vitesse_lacet") or 0.0)
        if not all(math.isfinite(v) for v in (x, y, cap, vitesse, vitesse_lacet)):
            raise MessageInvalide("odom : valeur non finie")
        r = self.reglages
        return OdomRos(
            stamp, x, y, cap, quaternion_cap(cap), vitesse, vitesse_lacet,
            covariance_pose(m.get("covariance"), r.variance_pose, r.variance_plan),
            covariance_vitesse(r.variance_vitesse, r.variance_plan),
        )

    def _gnss(self, m: Mapping[str, Any], stamp: float) -> GnssRos:
        fix = str(m.get("fix") or "aucun")
        latitude = _nombre_ou_none(m.get("latitude"))
        longitude = _nombre_ou_none(m.get("longitude"))
        altitude = _nombre_ou_none(m.get("altitude_m"))
        statut, covariance, genre = navsat(fix, _nombre_ou_none(m.get("hdop")), self.reglages)
        if latitude is None or longitude is None:
            statut, covariance, genre = STATUT_SANS_FIX, [0.0] * 9, COVARIANCE_INCONNUE
        nan = math.nan
        return GnssRos(
            stamp, statut, SERVICE_GPS | SERVICE_COMPASS,  # ATGM336H : GPS + BeiDou
            latitude if latitude is not None else nan,
            longitude if longitude is not None else nan,
            altitude if altitude is not None else nan,
            covariance, genre,
        )

    def _image(self, m: Mapping[str, Any], stamp: float) -> ImageRos:
        camera = nom_camera(m["camera"])
        try:
            jpeg = base64.b64decode(m["jpeg_b64"], validate=True)
        except (binascii.Error, TypeError) as erreur:
            raise MessageInvalide(f"image : base64 illisible ({erreur})") from None
        dimensions = dimensions_jpeg(jpeg)
        if dimensions is None:
            raise MessageInvalide("image : JPEG illisible")
        return ImageRos(stamp, camera, jpeg, dimensions[0], dimensions[1])


def _nombre_ou_none(valeur: Any) -> float | None:
    if isinstance(valeur, bool) or not isinstance(valeur, int | float):
        return None
    valeur = float(valeur)
    return valeur if math.isfinite(valeur) else None


# ---------------------------------------------------------------- consignes vers le rover


@dataclass(slots=True)
class LimitesConsigne:
    """Bornes appliquées par le pont avant l'envoi (le rover applique les siennes ensuite)."""

    vitesse_max: float = 0.5  # m/s, vitesses.max_auto_m_s du rover
    vitesse_arriere_max: float = 0.3  # m/s, vitesses.max_arriere_m_s
    vitesse_lacet_max: float = 1.0  # rad/s


def twist_vers_cmd_vel(vitesse: float, vitesse_lacet: float,
                       limites: LimitesConsigne | None = None) -> dict[str, Any] | None:
    """geometry_msgs/Twist (linear.x, angular.z) → message `cmd_vel` du rover, ou None si invalide.

    Les deux composantes sont réduites du même facteur : la courbure ω/v (donc le braquage que
    le rover en déduit) est conservée, seule l'allure baisse.
    """
    limites = limites or LimitesConsigne()
    try:
        v, w = float(vitesse), float(vitesse_lacet)
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(v) and math.isfinite(w)):
        return None
    facteur = 1.0
    plafond_v = limites.vitesse_max if v >= 0 else limites.vitesse_arriere_max
    if abs(v) > plafond_v:
        facteur = min(facteur, plafond_v / abs(v))
    if abs(w) > limites.vitesse_lacet_max:
        facteur = min(facteur, limites.vitesse_lacet_max / abs(w))
    v, w = v * facteur, w * facteur
    return {"type": "cmd_vel", "vitesse": round(v, 4) + 0.0, "vitesse_lacet": round(w, 4) + 0.0}


def but_vers_message(x: float, y: float,
                     transformation: tuple[float, float, float] | None = None) -> dict[str, Any]:
    """But (x, y) dans le repère de la demande → message `but` (repère odom du rover).

    `transformation` : pose (x, y, cap) du repère de la demande dans odom (None : déjà odom).
    """
    if transformation is not None:
        x, y = transformer_point_2d(transformation, x, y)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("but non fini")
    return {"type": "but", "x": round(x, 3) + 0.0, "y": round(y, 3) + 0.0}


def abonnement(images: bool) -> dict[str, Any]:
    return {"type": "abonner", "images": bool(images)}
