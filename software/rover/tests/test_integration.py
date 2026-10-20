"""Bout en bout : poste (WebSocket) → serveur → conduite → sécurité → faux ESP → physique."""

from __future__ import annotations

import asyncio
import json
import math
import time
from pathlib import Path

import pytest
from aiohttp.test_utils import TestClient, TestServer

from fieldmind.api.serveur import Serveur
from fieldmind.app import Rover
from fieldmind.config import Profil
from fieldmind.types import Pose2D


@pytest.fixture
async def client(tmp_path: Path):
    profil = Profil.charger(reglages=tmp_path / "reglages.json")
    rover = Rover(profil, simulation=True, scenario="salle", graine=0)
    # Au milieu de la pièce, vers un mur lointain : assez de place pour rouler
    rover.materiel.physique.pose = Pose2D(2.0, 3.5, 0.0)
    rover.demarrer()
    serveur = Serveur(rover, jeton="")
    async with TestClient(TestServer(serveur.app)) as c:
        c.rover = rover  # type: ignore[attr-defined]
        yield c
    rover.arreter()


async def recevoir(ws, type_: str, delai: float = 3.0) -> dict:
    fin = time.monotonic() + delai
    while time.monotonic() < fin:
        message = await ws.receive(timeout=fin - time.monotonic())
        donnees = json.loads(message.data)
        if donnees["type"] == type_:
            return donnees
    raise AssertionError(f"pas de message {type_}")


async def demander(ws, message: dict, ident: int) -> dict:
    await ws.send_json({**message, "id": ident})
    fin = time.monotonic() + 3
    while time.monotonic() < fin:
        donnees = json.loads((await ws.receive(timeout=3)).data)
        if donnees["type"] == "reponse" and donnees["id"] == ident:
            return donnees
    raise AssertionError("pas de réponse")


async def test_pilotage_manuel_homme_mort_et_arret_urgence(client) -> None:
    rover = client.rover
    ws = await client.ws_connect("/ws")
    bienvenue = await recevoir(ws, "bienvenue")
    assert bienvenue["materiel"] == "simulation"
    await asyncio.sleep(1.0)  # premiers tours du lidar
    assert (await demander(ws, {"type": "armer"}, 1))["ok"] is False  # mode arrêt
    assert (await demander(ws, {"type": "mode", "mode": "manuel"}, 2))["ok"]
    assert (await demander(ws, {"type": "armer"}, 3))["ok"]
    depart = rover.materiel.verite()
    for _ in range(25):  # 2,5 s de marche avant
        await ws.send_json({"type": "pilotage", "vitesse": 0.5, "braquage": 0.0})
        await asyncio.sleep(0.1)
    milieu = rover.materiel.verite()
    parcouru = math.hypot(milieu["x"] - depart["x"], milieu["y"] - depart["y"])
    assert parcouru > 0.3, parcouru
    # Homme mort : plus de messages → arrêt en moins d'une seconde
    for _ in range(30):  # arrêt en moins de 0,5 s visé ; jusqu'à 3 s sur une machine chargée
        await asyncio.sleep(0.1)
        if abs(rover.materiel.verite()["vitesse"]) < 0.02:
            break
    assert abs(rover.materiel.verite()["vitesse"]) < 0.02
    assert rover.conduite.consigne_courante().vitesse == 0
    # Arrêt d'urgence : verrouillé, désarme
    assert (await demander(ws, {"type": "arret_urgence"}, 4))["ok"]
    assert rover.conduite.arme is False and rover.superviseur.arret_urgence
    assert (await demander(ws, {"type": "armer"}, 5))["ok"] is False
    assert (await demander(ws, {"type": "rearmer"}, 6))["ok"]
    assert (await demander(ws, {"type": "armer"}, 7))["ok"]
    await ws.close()


async def test_action_refusee_si_desarme_et_flux_video(client) -> None:
    ws = await client.ws_connect("/ws")
    await recevoir(ws, "bienvenue")
    reponse = await demander(ws, {"type": "action", "action": {"type": "avancer",
                                                                "distance_m": 1}}, 1)
    assert reponse["ok"] is False and "désarmé" in reponse["erreur"]
    reponse = await demander(ws, {"type": "action", "action": {"type": "avancer",
                                                                "distance_m": 99}}, 2)
    assert reponse["ok"] is False
    for _ in range(50):  # première image de la caméra simulée
        await asyncio.sleep(0.1)
        if client.rover.bus.dernier("image.gauche") is not None:
            break
    photo = await client.get("/photo/gauche")
    assert photo.status == 200 and (await photo.read())[:2] == b"\xff\xd8"
    await ws.close()
