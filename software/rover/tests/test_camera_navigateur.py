"""La stéréo WebGL garde sa géométrie et reste isolée du matériel réel."""

import asyncio
import json
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from aiohttp.test_utils import TestClient, TestServer

from fieldmind.api import camera_simulee
from fieldmind.api.serveur import Serveur
from fieldmind.app import Rover
from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.simulation.camera_navigateur import CamerasNavigateur, dimensions_jpeg
from fieldmind.simulation.monde import Monde
from fieldmind.types import Pose2D


@pytest.fixture
def cameras(tmp_path):
    return CamerasNavigateur(
        Bus(), Profil.charger(reglages=tmp_path / "reglages.json"), Monde(scenario="jardin")
    )


def jpeg(w=640, h=360):
    return cv2.imencode(".jpg", np.zeros((h, w, 3), np.uint8))[1].tobytes()


def test_paire_synchronisee_et_geometrie(cameras):
    demande = cameras.nouvelle_demande(Pose2D(1, 2, 0.3), {"gauche": "gauche", "droite": "droite"})
    assert demande["cameras"]["gauche"]["centre"][1] > demande["cameras"]["droite"]["centre"][1]
    for role in ("gauche", "droite"):
        assert cameras.recevoir(demande["id"], role, jpeg()) is None
    g, d = cameras.bus.dernier(S.IMAGE_GAUCHE), cameras.bus.dernier(S.IMAGE_DROITE)
    assert g.t == d.t and g.numero != d.numero
    assert cameras.complete() and cameras.actif()


@pytest.mark.parametrize("erreur", ["ident", "doublon", "format", "taille", "camera"])
def test_refus_images_non_demandees(cameras, erreur):
    demande = cameras.nouvelle_demande(Pose2D(), {"gauche": "gauche"})
    ident, role, image = demande["id"], "gauche", jpeg()
    if erreur == "ident":
        ident += 1
    elif erreur == "doublon":
        assert cameras.recevoir(ident, role, image) is None
    elif erreur == "format":
        image = jpeg(320, 180)
    elif erreur == "taille":
        image = b"x" * 400001
    else:
        role = "drone"
    assert cameras.recevoir(ident, role, image) is not None


def test_transport_proprietaire_et_materiel_reel(cameras):
    poste = object()
    demande = cameras.nouvelle_demande(Pose2D(), {"gauche": "gauche"})
    paquet = json.dumps({"id": demande["id"], "camera": "gauche"}).encode() + b"\n" + jpeg()
    serveur = SimpleNamespace(
        rover=SimpleNamespace(materiel=SimpleNamespace(cameras_navigateur=cameras)),
        _poste_camera=poste,
    )
    camera_simulee.recevoir(serveur, object(), paquet)
    assert cameras.bus.dernier(S.IMAGE_GAUCHE) is None
    camera_simulee.recevoir(serveur, poste, paquet)
    assert cameras.bus.dernier(S.IMAGE_GAUCHE) is not None
    camera_simulee.recevoir(
        SimpleNamespace(rover=SimpleNamespace(materiel=object())), poste, paquet
    )


def test_retour_au_repli_et_entetes(cameras):
    assert dimensions_jpeg(jpeg()) == (640, 360)
    assert dimensions_jpeg(b"not a jpeg") is None
    cameras.derniere_image = time.monotonic() - 3
    assert not cameras.actif()
    demande = cameras.nouvelle_demande(Pose2D(), {"gauche": "gauche"})
    cameras.abandonner()
    assert cameras.recevoir(demande["id"], "gauche", jpeg()) == "non_demandee"


async def test_websocket_paire_binaire_et_mode_manuel(tmp_path):
    rover = Rover(
        Profil.charger(reglages=tmp_path / "reglages.json"),
        simulation=True,
        scenario="jardin",
        enregistrer=False,
    )
    serveur = Serveur(rover, jeton="")
    serveur.app.on_startup.clear()
    async with TestClient(TestServer(serveur.app)) as client:
        ws = await client.ws_connect("/ws")
        assert (await ws.receive_json())["cameras_navigateur"] is True
        await ws.send_json({"type": "camera_simulee_prete", "prete": True})
        messages = [await ws.receive_json(timeout=3) for _ in range(2)]
        assert messages[0]["type"] == "monde_simule"
        demande = messages[1]
        assert demande["type"] == "camera_simulee"
        for role in demande["vues"]:
            await ws.send_bytes(
                json.dumps({"id": demande["id"], "camera": role}).encode() + b"\n" + jpeg()
            )
        for _ in range(30):
            if rover.materiel.cameras_navigateur.complete():
                break
            await asyncio.sleep(0.01)
        assert rover.materiel.cameras_navigateur.complete()
        await ws.send_json({"type": "mode", "mode": "manuel", "id": 999})
        while True:
            reponse = await ws.receive_json(timeout=3)
            if reponse.get("id") == 999:
                assert reponse["ok"]
                break
        assert rover.conduite.mode == "manuel"
        await ws.close()
