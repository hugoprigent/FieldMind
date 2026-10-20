"""Robot distant : un pont externe (rover réel via ROS 2 ou liaison série, ou simulateur lancé à
part) se connecte en WebSocket sur /ws/robot et échange les messages du protocole.

Pont → serveur : messages de capteurs JSON (`scan`, `imu`, `odometrie`, `gnss`, `firmware`).
Serveur → pont : consignes JSON (`commande`).

Un seul robot à la fois, piloté par une seule session du poste opérateur (`/ws?robot=externe`).
Le pont s'authentifie avec le jeton FIELDMIND_JETON_ROBOT ; sans jeton configuré, l'accès est fermé.
"""

from __future__ import annotations

import asyncio
import contextlib
import hmac

from fastapi import WebSocket
from pydantic import ValidationError

from services.protocole.liaison import MessageRobot
from services.protocole.messages import TAILLE_MAX_IMAGE, Commande, lire_message_robot

TAILLE_MAX_MESSAGE = 2 * TAILLE_MAX_IMAGE + 1000
"""Le plus gros message est une paire stéréo (deux images JPEG)."""


class LiaisonRobotDistante:
    """`LiaisonRobot` alimentée par le WebSocket du pont."""

    def __init__(self, ws: WebSocket, taille_file: int = 2000) -> None:
        self.ws = ws
        self._file: asyncio.Queue[MessageRobot] = asyncio.Queue(maxsize=taille_file)
        self.messages_invalides = 0
        self.messages_perdus = 0
        self.connecte = True

    async def pomper(self) -> None:
        """Lit le WebSocket du pont jusqu'à sa fermeture (tâche du point d'entrée /ws/robot)."""
        try:
            while True:
                texte = await self.ws.receive_text()
                if len(texte) > TAILLE_MAX_MESSAGE:
                    self.messages_invalides += 1
                    continue
                try:
                    message = lire_message_robot.validate_json(texte)
                except ValidationError:
                    self.messages_invalides += 1
                    continue
                try:
                    self._file.put_nowait(message)
                except asyncio.QueueFull:
                    self.messages_perdus += 1
        finally:
            self.connecte = False

    def vider(self) -> int:
        """Jette les messages reçus quand personne ne pilotait ; renvoie leur nombre."""
        jetes = 0
        while not self._file.empty():
            self._file.get_nowait()
            jetes += 1
        return jetes

    async def recevoir(self) -> MessageRobot:
        return await self._file.get()

    async def envoyer(self, commande: Commande) -> None:
        if self.connecte:
            with contextlib.suppress(Exception):
                await self.ws.send_text(commande.model_dump_json())

    async def fermer(self) -> None:
        """La session opérateur se termine : le pont reste connecté (et son chien de garde
        arrête le robot faute de consignes)."""


class PosteRobot:
    """Registre du robot distant et de la session qui le pilote."""

    def __init__(self, jeton: str) -> None:
        self._jeton = jeton
        self.liaison: LiaisonRobotDistante | None = None
        self.session_pilote: str | None = None

    def jeton_valide(self, jeton: str | None) -> bool:
        if not self._jeton or not jeton:
            return False
        return hmac.compare_digest(self._jeton.encode(), jeton.encode())

    def prendre(self, session_id: str) -> LiaisonRobotDistante | None:
        """Réserve le robot connecté pour une session opérateur, s'il est libre."""
        if self.liaison is None or not self.liaison.connecte or self.session_pilote is not None:
            return None
        self.session_pilote = session_id
        # Le cerveau ne doit jamais traiter des mesures périmées : on repart du temps présent.
        self.liaison.vider()
        return self.liaison

    def rendre(self, session_id: str) -> None:
        if self.session_pilote == session_id:
            self.session_pilote = None
