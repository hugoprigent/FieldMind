"""Liaison entre le cerveau et un robot, réel ou simulé.

Le cerveau ne connaît le robot qu'à travers cette interface. Brancher le rover réel revient à
fournir une autre implémentation (pont ROS 2, liaison série, WebSocket) qui produit les mêmes
messages ; le cerveau ne change pas.
"""

from __future__ import annotations

from typing import Protocol

from services.protocole.messages import (
    Commande,
    Firmware,
    Gnss,
    Image,
    ImageStereo,
    Imu,
    Odometrie,
    Scan,
)

MessageRobot = Scan | Imu | Odometrie | Gnss | Firmware | Image | ImageStereo


class LiaisonRobot(Protocol):
    """Canal bidirectionnel vers un robot."""

    async def recevoir(self) -> MessageRobot:
        """Prochain message de capteur, dans l'ordre de publication. Bloque s'il n'y en a pas."""
        ...

    async def envoyer(self, commande: Commande) -> None:
        """Transmet une consigne Ackermann au contrôleur embarqué."""
        ...

    async def fermer(self) -> None:
        """Arrête la liaison (le robot freine de lui-même grâce à son chien de garde)."""
        ...
