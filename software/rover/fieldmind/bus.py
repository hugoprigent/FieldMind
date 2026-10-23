"""Bus de messages interne : publication / abonnement entre modules, sans dépendance.

Chaque sujet garde sa dernière valeur. Un abonné reçoit les valeurs dans le fil de l'émetteur
(rappel court, sans blocage) ou par une file qu'il lit à son rythme (fil séparé, ou asyncio).
Les noms de sujets sont dans `S` ; ils correspondent aux sujets ROS 2 du pont (fieldmind/ros2).
"""

from __future__ import annotations

import asyncio
import logging
import queue
import threading
import time
from collections import defaultdict
from collections.abc import Callable
from typing import Any

journal = logging.getLogger(__name__)


class S:
    """Sujets du bus. Type de la valeur entre parenthèses (fieldmind.types)."""

    SCAN = "scan"  # Scan, ≈ 10 Hz
    IMAGE_GAUCHE = "image.gauche"  # ImageCamera, ≈ 15 Hz
    IMAGE_DROITE = "image.droite"  # ImageCamera
    IMAGE_DRONE = "image.drone"  # ImageCamera (flux du drone ou mire de télémétrie)
    IMAGE_DRONE_SIMULEE = "image.drone_simule"  # ImageCamera, jamais le flux matériel
    DRONE_ATELIER = "drone.atelier"  # dict : mission et pilotage de l'atelier simulé
    IMU = "imu"  # Imu
    GNSS = "gnss"  # Gnss, 1 Hz
    ESP = "esp"  # EtatEsp
    DRONE = "drone"  # EtatDrone
    AUDIO = "audio"  # EchantillonAudio, blocs de 20 ms
    ODOM = "odom"  # Odometrie, 10–20 Hz
    REMORQUE = "remorque"  # controle.remorque.EtatRemorque, au rythme de l'odométrie
    OBSTACLES_LIDAR = "obstacles.lidar"  # Obstacles
    OBSTACLES_STEREO = "obstacles.stereo"  # Obstacles
    PROFONDEUR = "profondeur"  # bytes : JPEG de la carte de profondeur en fausses couleurs
    CARTE = "carte"  # CarteOccupation, 1 Hz
    SCENE = "scene"  # dict : message `scene` du poste (vue 3D), 4 Hz
    CHEMIN = "chemin"  # Chemin | None
    CONSIGNE = "consigne"  # Consigne (demande des producteurs : poste, missions, ROS 2)
    COMMANDE = "commande"  # CommandeRoues (appliquée, après sécurité)
    SECURITE = "securite"  # EtatSecurite
    MODE = "mode"  # dict : {"mode", "arme", "arret_urgence"}
    MISSION = "mission"  # dict : état de l'exécuteur
    JOURNAL = "journal"  # dict : {"t", "niveau", "source", "message"}
    CHAT = "chat"  # dict : {"role", "texte", "t", ...}
    SITUATION = "situation"  # dict : résumé de l'agent de veille (ia.distant.VeilleSituation)
    CIBLE = "cible"  # dict : personne suivie (perception.suivi), ≈ 10 Hz
    PAROLE = "parole"  # dict : voix du rover (WAV en base64) pour une réponse du chat
    VOIX = "voix"  # dict : état de l'écoute et transcriptions
    VOIX_NIVEAU = "voix.niveau"  # float : niveau sonore du micro (10 Hz, mode vocal)
    VISION = "vision"  # dict : résultat d'une analyse d'image par l'IA
    DETECTIONS = "detections"  # dict : objets détectés en continu (personnes, animaux…), conseil
    SYSTEME = "systeme"  # dict : processeur, température, alimentation, Wi-Fi
    MATERIEL = "materiel"  # dict : appareils détectés (USB, ports série, caméras, I2C)
    CONSOLE_ESP = "console.esp"  # str : une ligne reçue de l'ESP (ou envoyée, préfixée « → »)


class Bus:
    def __init__(self) -> None:
        self._derniers: dict[str, Any] = {}
        self._dates: dict[str, float] = {}
        self._rappels: dict[str, list[Callable[[Any], None]]] = defaultdict(list)
        self._verrou = threading.Lock()

    def publier(self, sujet: str, valeur: Any) -> None:
        with self._verrou:
            self._derniers[sujet] = valeur
            self._dates[sujet] = time.monotonic()
            rappels = list(self._rappels.get(sujet, ()))
        for rappel in rappels:
            try:
                rappel(valeur)
            except Exception:  # un abonné en panne ne doit pas casser l'émetteur
                journal.exception("abonné du sujet %s en erreur", sujet)

    def dernier(self, sujet: str, defaut: Any = None) -> Any:
        with self._verrou:
            return self._derniers.get(sujet, defaut)

    def age(self, sujet: str) -> float:
        """Secondes depuis la dernière publication (infini si jamais publié)."""
        with self._verrou:
            date = self._dates.get(sujet)
        return float("inf") if date is None else time.monotonic() - date

    def abonner(self, sujet: str, rappel: Callable[[Any], None]) -> Callable[[], None]:
        """Rappel appelé dans le fil de l'émetteur : il doit rendre la main tout de suite.
        Renvoie la fonction de désabonnement."""
        with self._verrou:
            self._rappels[sujet].append(rappel)

        def desabonner() -> None:
            with self._verrou:
                if rappel in self._rappels[sujet]:
                    self._rappels[sujet].remove(rappel)

        return desabonner

    def file(self, sujet: str, taille: int = 4) -> FileAbonnement:
        """File bornée pour un fil de travail : les plus anciennes valeurs sont jetées."""
        f = FileAbonnement(taille)
        f.desabonner = self.abonner(sujet, f.deposer)
        return f

    def file_async(
        self, sujet: str, boucle: asyncio.AbstractEventLoop, taille: int = 8
    ) -> tuple[asyncio.Queue[Any], Callable[[], None]]:
        """File asyncio (serveur web, agent IA). Valeurs les plus anciennes jetées si pleine."""
        q: asyncio.Queue[Any] = asyncio.Queue(taille)

        def deposer(valeur: Any) -> None:
            if q.full():
                try:
                    q.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            q.put_nowait(valeur)

        def rappel(valeur: Any) -> None:
            try:
                boucle.call_soon_threadsafe(deposer, valeur)
            except RuntimeError:  # boucle fermée à l'arrêt du programme
                pass

        return q, self.abonner(sujet, rappel)

    def journaliser(self, source: str, message: str, niveau: str = "info") -> None:
        """niveau : info | succes | alerte | erreur."""
        self.publier(S.JOURNAL, {"t": time.time(), "niveau": niveau, "source": source,
                                 "message": message})
        getattr(journal, {"alerte": "warning", "erreur": "error"}.get(niveau, "info"))(
            "[%s] %s", source, message)


class FileAbonnement:
    def __init__(self, taille: int) -> None:
        self._q: queue.Queue[Any] = queue.Queue(taille)
        self.desabonner: Callable[[], None] = lambda: None

    def deposer(self, valeur: Any) -> None:
        while True:
            try:
                self._q.put_nowait(valeur)
                return
            except queue.Full:
                try:
                    self._q.get_nowait()
                except queue.Empty:
                    pass

    def prendre(self, delai: float | None = None) -> Any:
        """Prochaine valeur ; lève queue.Empty après `delai` secondes."""
        return self._q.get(timeout=delai)

    def vider(self) -> list[Any]:
        valeurs = []
        while True:
            try:
                valeurs.append(self._q.get_nowait())
            except queue.Empty:
                return valeurs
