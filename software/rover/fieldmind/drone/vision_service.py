"""Service géométrique facultatif pour le GX10 : images entrantes, poses sortantes.

Aucun outil de vol, aucune trame RC, aucun LLM. La calibration doit être celle du
capteur réel avant un usage matériel ; la calibration fournie est celle du banc.
"""

from __future__ import annotations

import argparse
import asyncio
import hmac
import json
import os
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np
from aiohttp import web

from .vision import CalibrationCroix, VisionCroix


def creer_app(calibration: CalibrationCroix, jeton: str) -> web.Application:
    if not jeton:
        raise ValueError("jeton de vision requis (FIELDMIND_DRONE_VISION_JETON)")
    vision = VisionCroix(calibration)
    traitement = asyncio.Lock()

    async def sante(_r: web.Request) -> web.Response:
        return web.json_response(
            {"service": "vision_croix", "calibration": calibration.ident, "commandes_vol": False}
        )

    async def analyser(r: web.Request) -> web.Response:
        if not hmac.compare_digest(r.headers.get("X-Fieldmind-Jeton", ""), jeton):
            raise web.HTTPUnauthorized()
        if r.headers.get("X-Calibration") != calibration.ident:
            raise web.HTTPConflict(text="identifiant de calibration différent")
        try:
            seq = int(r.headers["X-Image-Sequence"])
            if not 0 <= seq <= 2**53:
                raise ValueError("séquence invalide")
        except (KeyError, ValueError) as e:
            raise web.HTTPBadRequest(text="séquence d'image manquante ou invalide") from e
        if traitement.locked():
            raise web.HTTPServiceUnavailable(text="vision occupée : ignorer cette image")
        brut = await r.read()
        if traitement.locked():
            raise web.HTTPServiceUnavailable(text="vision occupée : ignorer cette image")
        async with traitement:

            def calculer() -> dict | None:
                # Vérifier SOF et dimensions avant tout décodage coûteux.
                if dimensions_jpeg(brut) != (calibration.largeur_px, calibration.hauteur_px):
                    raise ValueError("dimensions JPEG différentes de la calibration")
                im = cv2.imdecode(np.frombuffer(brut, np.uint8), cv2.IMREAD_COLOR)
                if im is None:
                    raise ValueError("JPEG invalide")
                cible = vision.estimer(im, 0, seq)  # horloge de capture conservée sur le Pi
                return asdict(cible) if cible is not None else None

            try:
                cible = await asyncio.to_thread(calculer)
            except ValueError as e:
                raise web.HTTPBadRequest(text=str(e)) from e
        return web.json_response(
            {"sequence": seq, "calibration": calibration.ident, "cible": cible}
        )

    app = web.Application(client_max_size=1024 * 1024)
    app.add_routes([web.get("/sante", sante), web.post("/analyse", analyser)])
    return app


def dimensions_jpeg(brut: bytes) -> tuple[int, int] | None:
    if not brut.startswith(b"\xff\xd8"):
        return None
    i = 2
    while i + 4 <= len(brut):
        if brut[i] != 255:
            return None
        while i < len(brut) and brut[i] == 255:
            i += 1
        if i >= len(brut):
            return None
        marqueur = brut[i]
        i += 1
        if marqueur in (0xD9, 0xDA):
            return None
        if marqueur == 0x01 or 0xD0 <= marqueur <= 0xD7:
            continue
        if i + 2 > len(brut):
            return None
        longueur = int.from_bytes(brut[i : i + 2], "big")
        if longueur < 2 or i + longueur > len(brut):
            return None
        if marqueur in (0xC0, 0xC1, 0xC2):
            if longueur < 8:
                return None
            return int.from_bytes(brut[i + 5 : i + 7], "big"), int.from_bytes(
                brut[i + 3 : i + 5], "big"
            )
        i += longueur
    return None


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--hote", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8092)
    p.add_argument("--calibration", type=Path, required=True)
    args = p.parse_args()
    calibration = CalibrationCroix(**json.loads(args.calibration.read_text()))
    app = creer_app(calibration, os.environ.get("FIELDMIND_DRONE_VISION_JETON", ""))
    web.run_app(app, host=args.hote, port=args.port, access_log=None)


if __name__ == "__main__":
    main()
