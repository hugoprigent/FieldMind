"""Boucle réseau du pont ROS 2 (client de /ws/ros) contre un faux rover aiohttp, sans ROS."""

from __future__ import annotations

import asyncio
import contextlib
import json
import socket
import sys
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

from aiohttp import WSMsgType, web

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "ros2" / "src" / "fieldmind_rover_ros"))

from fieldmind_rover_ros import conversions as c
from fieldmind_rover_ros.client import ClientPont


class FauxRover:
    """Serveur /ws/ros minimal : enregistre ce que le pont envoie, envoie des mesures."""

    def __init__(self, jeton: str | None = None) -> None:
        self.jeton = jeton
        self.recus: list[dict[str, Any]] = []
        self.connexions = 0
        self.refus = 0
        self.ws: web.WebSocketResponse | None = None
        self._runner: web.AppRunner | None = None

    async def demarrer(self, port: int = 0) -> int:
        app = web.Application()
        app.router.add_get("/ws/ros", self._ws_ros)
        self._runner = web.AppRunner(app)
        await self._runner.setup()
        prise = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        prise.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        prise.bind(("127.0.0.1", port))
        site = web.SockSite(self._runner, prise)
        await site.start()
        return prise.getsockname()[1]

    async def arreter(self) -> None:
        if self.ws is not None:
            await self.ws.close()
        if self._runner is not None:
            await self._runner.cleanup()
            self._runner = None

    async def _ws_ros(self, requete: web.Request) -> web.StreamResponse:
        if self.jeton is not None and requete.query.get("jeton") != self.jeton:
            self.refus += 1
            raise web.HTTPUnauthorized()
        ws = web.WebSocketResponse()
        await ws.prepare(requete)
        self.ws = ws
        self.connexions += 1
        async for message in ws:
            if message.type == WSMsgType.TEXT:
                self.recus.append(json.loads(message.data))
        return ws

    async def envoyer(self, message: dict[str, Any]) -> None:
        assert self.ws is not None
        await self.ws.send_str(json.dumps(message))

    def types_recus(self) -> list[str]:
        return [m["type"] for m in self.recus]


async def attendre(condition: Callable[[], bool], delai: float = 3.0) -> None:
    limite = time.monotonic() + delai
    while not condition():
        if time.monotonic() > limite:
            raise AssertionError("condition non atteinte à temps")
        await asyncio.sleep(0.01)


def port_libre() -> int:
    with socket.socket() as prise:
        prise.bind(("127.0.0.1", 0))
        return prise.getsockname()[1]


class Pont:
    """Le client tel que l'utilise le nœud : messages reçus et changements de connexion notés."""

    def __init__(self, url: str, **options: Any) -> None:
        self.messages: list[tuple[dict[str, Any], float]] = []
        self.etats: list[bool] = []
        options.setdefault("attente_min_s", 0.05)
        options.setdefault("attente_max_s", 0.2)
        options.setdefault("battement_s", 1.0)
        self.client = ClientPont(
            url, sur_message=lambda m, t: self.messages.append((m, t)),
            sur_connexion=lambda ok, _detail: self.etats.append(ok), **options)
        self.tache: asyncio.Task[None] | None = None

    def __enter__(self) -> Pont:
        self.tache = asyncio.ensure_future(self.client.executer())
        return self

    def __exit__(self, *_: object) -> None:
        self.client.arreter()

    async def fin(self) -> None:
        self.client.arreter()
        assert self.tache is not None
        await asyncio.wait_for(self.tache, 2.0)


def executer(coroutine: Any) -> None:
    asyncio.run(asyncio.wait_for(coroutine, 15.0))


def test_reception_envoi_et_abonnement() -> None:
    async def scenario() -> None:
        rover = FauxRover()
        port = await rover.demarrer()
        pont = Pont(c.construire_url(f"127.0.0.1:{port}"), images=True)
        with pont:
            await attendre(lambda: pont.client.connecte and rover.recus)
            assert rover.recus[0] == {"type": "abonner", "images": True}
            assert pont.etats == [True]

            scan = {"type": "scan", "t": 10.0, "t_envoi": 10.01,
                    "angles_b64": c.encoder_float32([0.0, 1.5]),
                    "distances_b64": c.encoder_float32([2.0, 1.0])}
            avant = time.time()
            await rover.envoyer(scan)
            await rover.envoyer({"type": "odom", "t": 10.0, "x": 0.5, "y": 0.0, "cap": 0.1})
            await attendre(lambda: len(pont.messages) == 2)
            message, t_reception = pont.messages[0]
            assert message == scan
            assert avant <= t_reception <= time.time()
            assert pont.client.recus == {"scan": 1, "odom": 1}
            mesure = c.Decodeur().decoder(message, t_reception)
            assert isinstance(mesure, c.ScanRos)

            # Le nœud envoie depuis le fil de ROS, pas depuis la boucle du client.
            consigne = c.twist_vers_cmd_vel(0.3, 0.1)
            envoye = await asyncio.to_thread(pont.client.envoyer, consigne)
            assert envoye
            fil = threading.Thread(target=pont.client.envoyer,
                                   args=(c.but_vers_message(2.0, 1.0),))
            fil.start()
            fil.join()
            await attendre(lambda: len(rover.recus) == 3)
            assert rover.recus[1] == {"type": "cmd_vel", "vitesse": 0.3, "vitesse_lacet": 0.1}
            assert rover.recus[2] == {"type": "but", "x": 2.0, "y": 1.0}
            assert pont.client.envoyes == 2

            # Un texte qui n'est pas un objet JSON est compté, sans couper la liaison.
            assert rover.ws is not None
            await rover.ws.send_str("[1, 2]")
            await rover.ws.send_str("{pas du json")
            await attendre(lambda: pont.client.invalides == 2)
            assert pont.client.connecte
        await pont.fin()
        assert pont.etats == [True, False]
        await rover.arreter()

    executer(scenario())


def test_jeton() -> None:
    async def scenario() -> None:
        rover = FauxRover(jeton="secret")
        port = await rover.demarrer()
        sans = Pont(c.construire_url(f"127.0.0.1:{port}"))
        with sans:
            await attendre(lambda: rover.refus >= 2)
            assert not sans.client.connecte
            assert "401" in sans.client.derniere_erreur
        await sans.fin()
        assert sans.etats == []
        avec = Pont(c.construire_url(f"127.0.0.1:{port}", "secret"))
        with avec:
            await attendre(lambda: avec.client.connecte)
            assert rover.connexions == 1
        await avec.fin()
        await rover.arreter()

    executer(scenario())


def test_reconnexion_apres_fermeture_par_le_rover() -> None:
    async def scenario() -> None:
        rover = FauxRover()
        port = await rover.demarrer()
        pont = Pont(c.construire_url(f"127.0.0.1:{port}"))
        with pont:
            await attendre(lambda: pont.client.connecte)
            assert rover.ws is not None
            await rover.ws.close()
            await attendre(lambda: rover.connexions == 2 and pont.client.connecte)
            assert pont.etats == [True, False, True]
            assert rover.types_recus() == ["abonner", "abonner"]
            assert "fermée" in pont.client.derniere_erreur
        await pont.fin()
        await rover.arreter()

    executer(scenario())


def test_silence_du_rover_provoque_une_reconnexion() -> None:
    async def scenario() -> None:
        rover = FauxRover()
        port = await rover.demarrer()
        pont = Pont(c.construire_url(f"127.0.0.1:{port}"), silence_max_s=0.3)
        with pont:
            await attendre(lambda: rover.connexions >= 2, delai=3.0)
            assert "rien reçu" in pont.client.derniere_erreur
        await pont.fin()
        await rover.arreter()

    executer(scenario())


def test_rien_n_est_garde_pendant_une_coupure() -> None:
    async def scenario() -> None:
        rover = FauxRover()
        port = await rover.demarrer()
        pont = Pont(c.construire_url(f"127.0.0.1:{port}"))
        with pont:
            await attendre(lambda: pont.client.connecte)
            await rover.arreter()  # le rover disparaît (redémarrage du logiciel, Wi-Fi…)
            await attendre(lambda: not pont.client.connecte)
            perdus = pont.client.perdus
            assert not pont.client.envoyer(c.twist_vers_cmd_vel(0.4, 0.0))
            assert pont.client.perdus == perdus + 1

            rover2 = FauxRover()
            await rover2.demarrer(port)
            await attendre(lambda: pont.client.connecte and rover2.recus, delai=3.0)
            await asyncio.sleep(0.2)
            # La consigne émise pendant la coupure n'est jamais arrivée.
            assert rover2.types_recus() == ["abonner"]
        await pont.fin()
        await rover2.arreter()

    executer(scenario())


def test_rover_absent_au_demarrage() -> None:
    async def scenario() -> None:
        port = port_libre()
        pont = Pont(c.construire_url(f"127.0.0.1:{port}"))
        with pont:
            await asyncio.sleep(0.4)
            assert not pont.client.connecte
            assert pont.client.derniere_erreur
            rover = FauxRover()
            await rover.demarrer(port)
            await attendre(lambda: pont.client.connecte, delai=3.0)
        await pont.fin()
        await rover.arreter()

    executer(scenario())


def test_arret_pendant_l_attente_et_avant_le_demarrage() -> None:
    client = ClientPont("ws://127.0.0.1:1/ws/ros", sur_message=lambda m, t: None)
    client.arreter()  # sans boucle : sans effet
    assert not client.envoyer({"type": "cmd_vel"})

    async def scenario() -> None:
        pont = Pont(f"ws://127.0.0.1:{port_libre()}/ws/ros", attente_min_s=5.0)
        with pont:
            await asyncio.sleep(0.2)  # en attente avant le nouvel essai
            debut = time.monotonic()
        await pont.fin()
        assert time.monotonic() - debut < 1.0

    executer(scenario())


def test_consigne_restee_trop_longtemps_en_file_jetee() -> None:
    class FausseWs:
        def __init__(self) -> None:
            self.envoyes: list[str] = []

        async def send_str(self, texte: str) -> None:
            self.envoyes.append(texte)

    async def scenario() -> None:
        client = ClientPont("ws://h/ws/ros", sur_message=lambda m, t: None,
                            peremption_envoi_s=0.3)
        ws = FausseWs()
        file: asyncio.Queue[tuple[float, str]] = asyncio.Queue()
        file.put_nowait((time.monotonic() - 1.0, "ancienne"))
        file.put_nowait((time.monotonic(), "fraiche"))
        tache = asyncio.ensure_future(client._emettre(ws, file))  # type: ignore[arg-type]
        await attendre(lambda: ws.envoyes == ["fraiche"])
        tache.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await tache
        assert client.perdus == 1

    executer(scenario())


def test_file_pleine_le_plus_ancien_cede() -> None:
    async def scenario() -> None:
        client = ClientPont("ws://h/ws/ros", sur_message=lambda m, t: None)
        file: asyncio.Queue[tuple[float, str]] = asyncio.Queue(maxsize=2)
        client._file = file
        for i in range(3):
            client._deposer(file, (time.monotonic(), str(i)))
        assert [file.get_nowait()[1] for _ in range(2)] == ["1", "2"]
        assert client.perdus == 1
        autre: asyncio.Queue[tuple[float, str]] = asyncio.Queue()
        client._deposer(autre, (0.0, "connexion remplacée"))
        assert autre.empty()
        assert client.perdus == 2

    executer(scenario())
