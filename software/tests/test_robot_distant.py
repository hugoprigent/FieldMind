"""Intégration « robot distant » : le simulateur, lancé à part, se branche sur /ws/robot comme le
rover réel ; une session opérateur le pilote avec exactement le même cerveau."""

from __future__ import annotations

import asyncio
import json
import socket
import threading
import time
from collections.abc import Iterator

import pytest
import uvicorn
import websockets

from services.api import config
from simulation.pont_ws import executer as executer_pont

JETON = "jeton-de-test"


def _port_libre() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


@pytest.fixture
def serveur(monkeypatch: pytest.MonkeyPatch) -> Iterator[int]:
    monkeypatch.setenv("FIELDMIND_JETON_ROBOT", JETON)
    config.reglages.cache_clear()
    from services.api.serveur import creer_application
    from tests.conftest import FauxModele

    application = creer_application()
    application.state.consignes.fournisseur = FauxModele()
    port = _port_libre()
    serveur = uvicorn.Server(
        uvicorn.Config(application, host="127.0.0.1", port=port, log_level="warning")
    )
    fil = threading.Thread(target=serveur.run, daemon=True)
    fil.start()
    while not serveur.started:
        time.sleep(0.05)
    yield port
    serveur.should_exit = True
    fil.join(timeout=5)
    config.reglages.cache_clear()


def test_jeton_obligatoire(serveur: int) -> None:
    async def essai() -> None:
        with pytest.raises(websockets.InvalidStatus):
            async with websockets.connect(f"ws://127.0.0.1:{serveur}/ws/robot?jeton=faux"):
                pass

    asyncio.run(essai())


@pytest.mark.lent
def test_le_cerveau_pilote_un_robot_distant(serveur: int) -> None:
    async def essai() -> None:
        pont = asyncio.create_task(
            executer_pont(f"ws://127.0.0.1:{serveur}/ws/robot", JETON, None, duree_max_s=20)
        )
        await asyncio.sleep(1.0)
        async with websockets.connect(f"ws://127.0.0.1:{serveur}/ws?robot=externe") as ws:
            bienvenue = json.loads(await ws.recv())
            assert bienvenue["mode"] == "robot_reel"
            assert bienvenue["monde"] is None, "aucune vérité terrain en mode robot réel"
            await ws.send(json.dumps({"type": "chat", "texte": "avance de 3 m"}))
            premiere_pose = None
            etats = set()
            fin = time.monotonic() + 15
            while time.monotonic() < fin and "atteint" not in etats:
                message = json.loads(await asyncio.wait_for(ws.recv(), 5))
                if message["type"] == "pose_estimee" and premiere_pose is None:
                    premiere_pose = message
                if message["type"] == "statut":
                    etats.add(message["etat"])
                assert message["type"] != "vue", "le robot distant ne publie pas de vérité"
            assert "atteint" in etats
        pont.cancel()
        with pytest.raises(asyncio.CancelledError):
            await pont

    asyncio.run(essai())
