"""Contrat des messages échangés entre le robot (réel ou simulé), le cerveau et le poste opérateur.

Unités SI. Repère au sol : x vers l'est, y vers le nord, cap en radians (sens trigonométrique,
0 = est). Repère du robot : x vers l'avant, y vers la gauche, z vers le haut.

Les messages du robot reprennent les messages standard de ROS 2 pour qu'un pont vers le vrai rover
soit une simple traduction :

| Message    | Équivalent ROS 2                              |
|------------|-----------------------------------------------|
| `scan`     | sensor_msgs/LaserScan                         |
| `imu`      | sensor_msgs/Imu                               |
| `odometrie`| sensor_msgs/JointState (roues et direction)   |
| `gnss`     | sensor_msgs/NavSatFix, converti en ENU local  |
| `firmware` | diagnostic_msgs/DiagnosticStatus              |
| `image`    | sensor_msgs/CompressedImage (JPEG)            |
| `stereo`   | 2 × sensor_msgs/CompressedImage (rectifiées)  |
| `commande` | ackermann_msgs/AckermannDrive                 |

Chaque message porte l'horodatage `t` de la mesure (horloge du robot, en secondes). Le cerveau
date tout avec cet horodatage et jamais avec son heure de réception.
"""

from __future__ import annotations

import math
from typing import Annotated, Literal

from pydantic import AfterValidator, BaseModel, ConfigDict, Field, TypeAdapter


def _fini(v: float) -> float:
    if not math.isfinite(v):
        raise ValueError("valeur non finie")
    return v


Reel = Annotated[float, AfterValidator(_fini)]
Temps = Annotated[float, Field(ge=0), AfterValidator(_fini)]


class _Message(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class Pose2D(_Message):
    x: Reel
    y: Reel
    cap: Reel


# ============================================================ robot → cerveau (capteurs)


class Scan(_Message):
    """Balayage lidar 2D dans le repère du capteur. `null` : pas d'écho (ou mesure perdue)."""

    type: Literal["scan"] = "scan"
    t: Temps
    angle_min: Reel
    pas: Annotated[float, Field(gt=0)]
    portee_min: Annotated[float, Field(ge=0)]
    portee_max: Annotated[float, Field(gt=0)]
    distances: list[Annotated[float, Field(ge=0)] | None] = Field(max_length=4096)


class Imu(_Message):
    """Centrale inertielle : orientation (quaternion w, x, y, z), gyroscope et accéléromètre."""

    type: Literal["imu"] = "imu"
    t: Temps
    orientation: tuple[Reel, Reel, Reel, Reel]
    vitesse_angulaire: tuple[Reel, Reel, Reel]
    acceleration: tuple[Reel, Reel, Reel]


class Odometrie(_Message):
    """Vitesse de rotation des roues (rad/s) et angle de braquage (rad).

    Sur le rover réel, la vitesse des roues arrière vient du compte-tours du VESC (essieu rigide :
    même valeur à gauche et à droite) et les roues avant n'ont pas de codeur (`null`). Le braquage
    est celui du servo : mesuré s'il a un retour de position, sinon la dernière consigne.
    """

    type: Literal["odometrie"] = "odometrie"
    t: Temps
    roue_ar_g: Reel
    roue_ar_d: Reel
    braquage: Reel
    roue_av_g: Reel | None = None
    roue_av_d: Reel | None = None


TypeFix = Literal["aucun", "autonome", "dgnss", "rtk_flottant", "rtk_fixe"]


class Gnss(_Message):
    """Position de l'antenne GNSS en coordonnées locales (m), avec son incertitude (1 σ).

    `z` : altitude de l'antenne au-dessus de l'origine locale (m), `sigma_z` son incertitude ;
    absente si le récepteur ne la fournit pas."""

    type: Literal["gnss"] = "gnss"
    t: Temps
    fix: TypeFix
    x: Reel
    y: Reel
    sigma: Annotated[float, Field(gt=0)]
    satellites: int = Field(ge=0)
    z: Reel | None = None
    sigma_z: Annotated[float, Field(gt=0)] | None = None


RaisonArret = Literal["obstacle_avant", "obstacle_arriere", "chien_de_garde", "arret_operateur"]


class Firmware(_Message):
    """État du contrôleur embarqué : protections, consignes appliquées, moteur et batterie."""

    type: Literal["firmware"] = "firmware"
    t: Temps
    arret: RaisonArret | None
    vitesse_consigne: Reel
    braquage_consigne: Reel
    couple_moteur: Reel
    regime_moteur: Reel
    """tr/min"""
    tension_batterie: Reel
    courant_batterie: Reel
    etat_charge: Annotated[float, Field(ge=0, le=1)]


TAILLE_MAX_IMAGE = 400_000
"""Longueur maximale du JPEG encodé en base64 (640 × 360 en qualité 80 : environ 50 000)."""


class Image(_Message):
    """Image de la caméra avant, compressée en JPEG et encodée en base64.

    `donnees` vide : image non transmise (banc d'essai sans rendu) ; seul l'horodatage compte.
    """

    type: Literal["image"] = "image"
    t: Temps
    largeur: int = Field(gt=0, le=4096)
    hauteur: int = Field(gt=0, le=4096)
    format: Literal["jpeg"] = "jpeg"
    donnees: str = Field(max_length=TAILLE_MAX_IMAGE)


class ImageStereo(_Message):
    """Paire stéréo synchronisée et rectifiée, en niveaux de gris (JPEG, base64). Même instant,
    même taille ; l'œil droit est décalé de la base vers la droite."""

    type: Literal["stereo"] = "stereo"
    t: Temps
    largeur: int = Field(gt=0, le=4096)
    hauteur: int = Field(gt=0, le=4096)
    format: Literal["jpeg"] = "jpeg"
    gauche: str = Field(max_length=TAILLE_MAX_IMAGE)
    droite: str = Field(max_length=TAILLE_MAX_IMAGE)


MessageRobot = Annotated[
    Scan | Imu | Odometrie | Gnss | Firmware | Image | ImageStereo, Field(discriminator="type")
]
lire_message_robot: TypeAdapter[Scan | Imu | Odometrie | Gnss | Firmware | Image | ImageStereo] = (
    TypeAdapter(MessageRobot)
)


# ============================================================ cerveau → robot


class Commande(_Message):
    """Consigne Ackermann : vitesse longitudinale (m/s, négative en marche arrière) et braquage."""

    type: Literal["commande"] = "commande"
    t: Temps
    vitesse: Reel
    braquage: Reel


MessageVersRobot = Commande


# ============================================================ poste opérateur → cerveau


class Objectif(_Message):
    type: Literal["objectif"] = "objectif"
    x: Reel
    y: Reel


class Chat(_Message):
    type: Literal["chat"] = "chat"
    texte: str = Field(min_length=1, max_length=200)


class Stop(_Message):
    type: Literal["stop"] = "stop"


class Reprendre(_Message):
    type: Literal["reprendre"] = "reprendre"


class Reinitialiser(_Message):
    type: Literal["reset"] = "reset"


class Teleoperation(_Message):
    """Conduite manuelle : fractions de la vitesse et du braquage maximaux, entre -1 et 1."""

    type: Literal["teleop"] = "teleop"
    avance: Annotated[float, Field(ge=-1, le=1)]
    direction: Annotated[float, Field(ge=-1, le=1)]


class CommandeVision(_Message):
    """Analyse immédiate de la caméra, ou marche/arrêt de la vigilance."""

    type: Literal["vision"] = "vision"
    action: Literal["analyser", "vigilance_on", "vigilance_off"]


MessageOperateur = Annotated[
    Objectif | Chat | Stop | Reprendre | Reinitialiser | Teleoperation | CommandeVision,
    Field(discriminator="type"),
]
lire_message_operateur: TypeAdapter[
    Objectif | Chat | Stop | Reprendre | Reinitialiser | Teleoperation | CommandeVision
] = TypeAdapter(MessageOperateur)
