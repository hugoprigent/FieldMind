"""Outils du GX10 côté rover : détection continue, voix, mémoire des lieux, pointage RoboBrain."""

from __future__ import annotations

import asyncio
import base64
from pathlib import Path
from typing import Any

from aiohttp import web
from aiohttp.test_utils import TestServer
from test_ia_faux_serveur import FauxOutils, config_essai
from test_ia_vision import profil_rover

from fieldmind.bus import Bus, S
from fieldmind.ia.distant import DetectionContinue, MemoireLieux, VoixRover, position_au_sol
from fieldmind.ia.inference import ClientInference
from fieldmind.ia.vision import consigne_pointage, lire_points_tuples
from fieldmind.types import ImageCamera


def outils_avec_image() -> FauxOutils:
    """Faux rover avec un vrai JPEG 640 × 360 (il est redimensionné avant l'envoi)."""
    import cv2
    import numpy as np

    outils = FauxOutils()
    ok, jpeg = cv2.imencode(".jpg", np.full((360, 640, 3), 128, np.uint8))
    assert ok
    outils.images["gauche"] = ImageCamera(t=1.0, camera="gauche", jpeg=jpeg.tobytes(),
                                          largeur=640, hauteur=360)
    return outils


class FauxOutilsGx10:
    def __init__(self) -> None:
        self.requetes: list[tuple[str, dict[str, Any]]] = []
        self.app = web.Application()
        self.app.router.add_post("/v1/analyse", self._analyse)
        self.app.router.add_post("/v1/empreintes", self._empreintes)
        self.app.router.add_post("/v1/audio/speech", self._parole)

    async def _analyse(self, requete: web.Request) -> web.Response:
        corps = await requete.json()
        self.requetes.append(("analyse", corps))
        return web.json_response({
            "image_id": "abc", "durees_ms": {"detection": 12.0},
            "detections": [{"label": "person", "score": 0.91, "boite": [0.4, 0.2, 0.6, 0.9],
                            "distance_m": 3.2},
                           {"label": "chair", "score": 0.6, "boite": [0.1, 0.5, 0.2, 0.7]}],
            "profondeur": {"points": [2.5 for _ in corps.get("points", [])]},
            "segmentation": [{"point": p, "contour": [[0.1, 0.1], [0.2, 0.1], [0.2, 0.2]]}
                             for p in corps.get("points", [])]})

    async def _empreintes(self, requete: web.Request) -> web.Response:
        corps = await requete.json()
        self.requetes.append(("empreintes", corps))
        return web.json_response({"vecteur": [1.0, 0.0], "dimension": 2})

    async def _parole(self, requete: web.Request) -> web.Response:
        self.requetes.append(("parole", await requete.json()))
        return web.Response(body=b"RIFF....WAVE", content_type="audio/wav")


async def _serveur() -> tuple[TestServer, FauxOutilsGx10]:
    faux = FauxOutilsGx10()
    serveur = TestServer(faux.app)
    await serveur.start_server()
    return serveur, faux


def _config(serveur: TestServer) -> Any:
    return config_essai(str(serveur.make_url("")).rstrip("/"), env={}, profil="gx10")


def test_points_robobrain() -> None:
    assert "[(x, y)]" in consigne_pointage("la porte", "robobrain")
    assert lire_points_tuples("[(500, 250), (1200, 3), (10.5, 990)]", "porte") == [
        (0.5, 0.25, "porte"), (0.0105, 0.99, "porte")]
    assert lire_points_tuples("rien", "porte") == []


def test_position_au_sol_par_la_profondeur() -> None:
    class SansSol:
        def pixel_vers_sol(self, *_: Any) -> None:
            return None

    sol = position_au_sol(SansSol(), profil_rover(), "gauche", [0.45, 0.2, 0.55, 0.9], 3.0)
    assert sol is not None and 3.0 < sol[0] < 3.6 and abs(sol[1]) < 0.05


async def test_detection_continue_publie_personnes_en_francais() -> None:
    serveur, faux = await _serveur()
    bus = Bus()
    recus: list[dict[str, Any]] = []
    bus.abonner(S.DETECTIONS, recus.append)
    client = ClientInference(_config(serveur))
    detection = DetectionContinue(bus, profil_rover(), client, outils_avec_image())
    assert detection.active()
    message = await detection.cycle()
    await client.fermer()
    await serveur.close()
    assert message is not None and recus == [message]
    personne = message["objets"][0]
    assert personne["label"] == "personne" and personne["vivant"] and personne["sol"]
    assert message["objets"][1]["label"] == "chaise" and not message["objets"][1]["vivant"]
    corps = faux.requetes[0][1]
    assert corps["taches"] == ["detection", "profondeur"] and corps["image"]


async def test_voix_du_rover() -> None:
    serveur, faux = await _serveur()
    bus = Bus()
    paroles: list[dict[str, Any]] = []
    bus.abonner(S.PAROLE, paroles.append)
    client = ClientInference(_config(serveur))
    voix = VoixRover(bus, profil_rover(), client)
    tache = asyncio.create_task(voix.boucle())
    await asyncio.sleep(0.05)
    bus.publier(S.CHAT, {"role": "rover", "texte": "J'avance de deux mètres.", "id": "m1"})
    bus.publier(S.CHAT, {"role": "operateur", "texte": "avance", "id": "m2"})  # pas dit
    for _ in range(50):
        if paroles:
            break
        await asyncio.sleep(0.02)
    tache.cancel()
    await client.fermer()
    await serveur.close()
    assert len(paroles) == 1 and paroles[0]["id"] == "m1"
    assert base64.b64decode(paroles[0]["audio"]).startswith(b"RIFF")
    assert [r for r, _ in faux.requetes] == ["parole"]


async def test_memoire_des_lieux(tmp_path: Path) -> None:
    serveur, _ = await _serveur()
    client = ClientInference(_config(serveur))
    memoire = MemoireLieux(client, tmp_path / "souvenirs.json")
    outils = outils_avec_image()
    assert await memoire.memoriser("garage", outils.derniere_image("gauche"))
    assert MemoireLieux(client, tmp_path / "souvenirs.json").souvenirs[0]["nom"] == "garage"
    trouve = await memoire.reconnaitre(b"\xff\xd8 jpeg")
    await client.fermer()
    await serveur.close()
    assert trouve == ("garage", 1.0)
