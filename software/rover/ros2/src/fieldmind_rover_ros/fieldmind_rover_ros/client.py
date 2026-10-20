"""Client de la WebSocket `/ws/ros` du rover : connexion, réception, envoi, reconnexion.

Partie réseau du nœud `pont`, sans rclpy (testée contre un faux serveur aiohttp,
`software/rover/tests/test_ros2_client.py`). Elle tourne dans sa propre boucle asyncio, dans un
fil à part ; `envoyer()` et `arreter()` s'appellent depuis n'importe quel fil.

Règle de sûreté : **rien n'est mis en attente hors connexion**. Une consigne émise pendant une
coupure est perdue (`envoyer` renvoie False) et une consigne restée trop longtemps dans la file
d'envoi est jetée : le rover ne doit jamais recevoir un ordre en retard. Le pont ne répète pas
non plus une consigne : si Nav2 se tait, le rover s'arrête de lui-même (consigne périmée).
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import time
from collections import Counter
from collections.abc import Callable
from typing import Any

import aiohttp

from .conversions import abonnement, masquer_jeton

journal = logging.getLogger("fieldmind_rover_ros.client")

TAILLE_MAX_MESSAGE = 16 * 1024 * 1024  # images JPEG en base64 comprises
TAILLE_FILE_ENVOI = 32


class ClientPont:
    """Client de `/ws/ros`.

    `sur_message(message, t_reception)` : chaque objet JSON reçu, daté par `horloge()` dès sa
    réception (avant décodage). `sur_connexion(connecte, detail)` : changements d'état de la
    liaison. Les deux sont appelés dans le fil de la boucle asyncio et doivent rester courts.
    """

    def __init__(
        self,
        url: str,
        *,
        sur_message: Callable[[dict[str, Any], float], None],
        sur_connexion: Callable[[bool, str], None] | None = None,
        images: bool = False,
        horloge: Callable[[], float] = time.time,
        silence_max_s: float = 3.0,
        battement_s: float = 2.0,
        delai_connexion_s: float = 5.0,
        attente_min_s: float = 0.5,
        attente_max_s: float = 5.0,
        peremption_envoi_s: float = 0.3,
    ) -> None:
        self.url = url
        self.images = images
        self._sur_message = sur_message
        self._sur_connexion = sur_connexion
        self._horloge = horloge
        self.silence_max_s = silence_max_s
        self.battement_s = battement_s
        self.delai_connexion_s = delai_connexion_s
        self.attente_min_s = attente_min_s
        self.attente_max_s = attente_max_s
        self.peremption_envoi_s = peremption_envoi_s
        self._boucle: asyncio.AbstractEventLoop | None = None
        self._arret: asyncio.Event | None = None
        self._ws: aiohttp.ClientWebSocketResponse | None = None
        self._file: asyncio.Queue[tuple[float, str]] | None = None
        self._taches: set[asyncio.Future[Any]] = set()
        self.connexions = 0
        self.recus: Counter[str] = Counter()
        self.envoyes = 0
        self.perdus = 0  # consignes non envoyées (hors connexion, file pleine, périmées)
        self.invalides = 0  # textes reçus qui ne sont pas un objet JSON
        self.derniere_erreur = ""

    # ------------------------------------------------------------------ depuis n'importe quel fil

    @property
    def connecte(self) -> bool:
        ws = self._ws
        return ws is not None and not ws.closed

    def envoyer(self, message: dict[str, Any]) -> bool:
        """Envoie un message au rover. False (message perdu) si la liaison est coupée."""
        boucle, file = self._boucle, self._file
        if boucle is None or file is None or not self.connecte or boucle.is_closed():
            self.perdus += 1
            return False
        texte = json.dumps(message, separators=(",", ":"), ensure_ascii=False)
        depot = (time.monotonic(), texte)
        try:
            boucle.call_soon_threadsafe(self._deposer, file, depot)
        except RuntimeError:  # boucle arrêtée entre-temps
            self.perdus += 1
            return False
        return True

    def arreter(self) -> None:
        """Ferme la liaison et termine `executer()`."""
        boucle = self._boucle
        if boucle is None or boucle.is_closed():
            return
        with contextlib.suppress(RuntimeError):
            boucle.call_soon_threadsafe(self._demander_arret)

    # ------------------------------------------------------------------ dans la boucle

    def _deposer(self, file: asyncio.Queue[tuple[float, str]], depot: tuple[float, str]) -> None:
        if file is not self._file:  # connexion remplacée entre-temps : message périmé
            self.perdus += 1
            return
        if file.full():
            file.get_nowait()  # le plus ancien cède la place
            self.perdus += 1
        file.put_nowait(depot)

    def _demander_arret(self) -> None:
        if self._arret is not None:
            self._arret.set()
        ws = self._ws
        if ws is not None and not ws.closed:
            tache = asyncio.ensure_future(ws.close())
            self._taches.add(tache)
            tache.add_done_callback(self._taches.discard)

    async def executer(self) -> None:
        """Boucle de connexion : jusqu'à `arreter()` (ou annulation de la tâche)."""
        self._boucle = asyncio.get_running_loop()
        self._arret = asyncio.Event()
        attente = self.attente_min_s
        delais = aiohttp.ClientTimeout(total=None, sock_connect=self.delai_connexion_s,
                                       connect=self.delai_connexion_s)
        async with aiohttp.ClientSession(timeout=delais) as session:
            while not self._arret.is_set():
                etait_connecte = False
                try:
                    etait_connecte, raison = await self._session(session)
                except (aiohttp.ClientError, OSError, TimeoutError) as erreur:
                    raison = f"{type(erreur).__name__} : {erreur}"
                self.derniere_erreur = raison
                if etait_connecte:
                    attente = self.attente_min_s
                    self._signaler(False, raison)
                if self._arret.is_set():
                    break
                journal.info("liaison %s : %s ; nouvel essai dans %.1f s",
                             masquer_jeton(self.url), raison, attente)
                with contextlib.suppress(TimeoutError):
                    await asyncio.wait_for(self._arret.wait(), attente)
                attente = min(attente * 2.0, self.attente_max_s)

    async def _session(self, session: aiohttp.ClientSession) -> tuple[bool, str]:
        """Une connexion, jusqu'à sa fin. Renvoie (connexion établie, raison de la fin)."""
        async with session.ws_connect(self.url, heartbeat=self.battement_s, autoping=True,
                                      max_msg_size=TAILLE_MAX_MESSAGE) as ws:
            file: asyncio.Queue[tuple[float, str]] = asyncio.Queue(maxsize=TAILLE_FILE_ENVOI)
            self._ws, self._file = ws, file
            self.connexions += 1
            emetteur = asyncio.ensure_future(self._emettre(ws, file))
            try:
                await ws.send_str(json.dumps(abonnement(self.images)))
                self._signaler(True, masquer_jeton(self.url))
                try:
                    return True, await self._recevoir(ws)
                except (aiohttp.ClientError, OSError) as erreur:
                    return True, f"{type(erreur).__name__} : {erreur}"
            finally:
                self._ws, self._file = None, None
                emetteur.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await emetteur
                self.perdus += file.qsize()

    async def _recevoir(self, ws: aiohttp.ClientWebSocketResponse) -> str:
        while True:
            try:
                message = await asyncio.wait_for(ws.receive(), self.silence_max_s)
            except TimeoutError:
                await ws.close()
                return f"rien reçu depuis {self.silence_max_s:.1f} s"
            if message.type == aiohttp.WSMsgType.TEXT:
                t_reception = self._horloge()
                try:
                    objet = json.loads(message.data)
                except ValueError:
                    self.invalides += 1
                    continue
                if not isinstance(objet, dict):
                    self.invalides += 1
                    continue
                self.recus[str(objet.get("type"))] += 1
                try:
                    self._sur_message(objet, t_reception)
                except Exception:  # un message qui casse la conversion ne coupe pas la liaison
                    journal.exception("traitement d'un message %s", objet.get("type"))
            elif message.type == aiohttp.WSMsgType.BINARY:
                self.invalides += 1
            elif message.type == aiohttp.WSMsgType.ERROR:
                return f"erreur : {ws.exception()}"
            else:  # CLOSE, CLOSING, CLOSED
                code = ws.close_code
                return "fermée par le rover" + (f" (code {code})" if code else "")

    async def _emettre(self, ws: aiohttp.ClientWebSocketResponse,
                       file: asyncio.Queue[tuple[float, str]]) -> None:
        while True:
            depose, texte = await file.get()
            if time.monotonic() - depose > self.peremption_envoi_s:
                self.perdus += 1  # resté trop longtemps : une consigne en retard est dangereuse
                continue
            await ws.send_str(texte)
            self.envoyes += 1

    def _signaler(self, connecte: bool, detail: str) -> None:
        if self._sur_connexion is None:
            return
        try:
            self._sur_connexion(connecte, detail)
        except Exception:
            journal.exception("rappel de connexion")
