"""Service GX10 et client Pi : séquences, délai, calibration, limites des images."""

from __future__ import annotations

import asyncio
import threading
import time

import cv2
import numpy as np
import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from fieldmind.drone.simulation import DroneSimule
from fieldmind.drone.vision import CalibrationCroix, VisionCroix
from fieldmind.drone.vision_distante import VisionDistante
from fieldmind.drone.vision_service import creer_app, dimensions_jpeg


def jpeg_croix():
    d = DroneSimule()
    d.camera, d.h, d.x, d.y = "bas", 0.7, 0.02, -0.03
    _, buf = cv2.imencode(".jpg", d._rendre())
    return bytes(buf)


def headers(sequence=1):
    return {
        "X-Fieldmind-Jeton": "fixture",
        "X-Calibration": "simulation-320x240",
        "X-Image-Sequence": str(sequence),
    }


async def test_service_et_client_restituent_pose_avec_horloge_capture_du_pi():
    app = creer_app(CalibrationCroix(), "fixture")
    async with TestClient(TestServer(app)) as client:
        v = VisionDistante(
            str(client.make_url("/")), "fixture", "simulation-320x240", horloge=lambda: 100.1
        )
        cible = await v.estimer(jpeg_croix(), 7, 100)
        assert cible is not None and cible.t == 100 and cible.sequence == 7
        assert cible.hauteur_m == pytest.approx(0.7, abs=0.04)
        assert cible.source == "gx10_camera_bas"
        assert await v.estimer(jpeg_croix(), 7, 100) is None
        assert await v.estimer(jpeg_croix(), 6, 100) is None
        assert await v.estimer(jpeg_croix(), 8, 98) is None


async def test_jeton_calibration_jpeg_et_taille():
    app = creer_app(CalibrationCroix(), "fixture")
    async with TestClient(TestServer(app)) as client:
        assert (await client.post("/analyse", data=jpeg_croix())).status == 401
        mauvais = {**headers(), "X-Calibration": "autre"}
        assert (await client.post("/analyse", data=jpeg_croix(), headers=mauvais)).status == 409
        assert (await client.post("/analyse", data=b"invalide", headers=headers())).status == 400
        assert (
            await client.post("/analyse", data=b"x" * (1024**2 + 1), headers=headers())
        ).status == 413
        _, buf = cv2.imencode(".jpg", np.zeros((480, 640, 3), np.uint8))
        assert (await client.post("/analyse", data=bytes(buf), headers=headers())).status == 400
        d = await (await client.get("/sante")).json()
        assert not d["commandes_vol"]
    with pytest.raises(ValueError):
        creer_app(CalibrationCroix(), "")


@pytest.mark.parametrize(
    "decalage,scenario", [(1.0, "ancien"), (0.1, "retard"), (0.1, "autre_image"), (0.1, "nan")]
)
async def test_resultat_inutilisable_ne_devient_pas_une_consigne(decalage, scenario):
    async def analyser(_r):
        if scenario == "retard":
            await asyncio.sleep(0.4)
        return web.json_response(
            {
                "sequence": 9 if scenario == "autre_image" else 1,
                "calibration": "fixture",
                "cible": {
                    "avant_m": 0,
                    "droite_m": float("nan") if scenario == "nan" else 0,
                    "hauteur_m": 1,
                    "qualite": 1,
                },
            }
        )

    app = web.Application()
    app.router.add_post("/analyse", analyser)
    async with TestClient(TestServer(app)) as client:
        v = VisionDistante(str(client.make_url("/")), "fixture", "fixture")
        assert await v.estimer(jpeg_croix(), 1, time.monotonic() - decalage) is None


async def test_service_ne_met_pas_images_en_file_si_occupe(monkeypatch):
    pret, finir = threading.Event(), threading.Event()
    original = VisionCroix.estimer

    def lent(self, *a):
        pret.set()
        finir.wait(1)
        return original(self, *a)

    monkeypatch.setattr(VisionCroix, "estimer", lent)
    async with TestClient(TestServer(creer_app(CalibrationCroix(), "fixture"))) as client:
        premiere = asyncio.create_task(
            client.post("/analyse", data=jpeg_croix(), headers=headers())
        )
        assert await asyncio.to_thread(pret.wait, 1)
        try:
            deuxieme = await client.post("/analyse", data=jpeg_croix(), headers=headers(2))
            assert deuxieme.status == 503
        finally:
            finir.set()
        assert (await premiere).status == 200


def test_dimensions_sans_decoder_un_jpeg_geant():
    assert dimensions_jpeg(jpeg_croix()) == (320, 240)
    assert dimensions_jpeg(b"\xff\xd8\xff\xc0\xff\xff") is None
    assert dimensions_jpeg(b"\xff\xd8\xff\xda\x00\x02") is None
