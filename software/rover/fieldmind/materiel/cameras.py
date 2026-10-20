"""Webcams USB (UVC) : capture continue, JPEG transmis sans ré-encodage.

Les webcams fournissent déjà du MJPEG : on récupère le JPEG tel quel (`CONVERT_RGB` désactivé),
ce qui laisse le processeur du Pi libre. Les caméras sont identifiées par leur port USB physique
(/dev/v4l/by-path) : deux webcams identiques restent distinctes. L'attribution gauche / droite
se fait par port (profil, ou réglage du poste).
"""

from __future__ import annotations

import os
import re
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from fieldmind.bus import Bus, S
from fieldmind.types import ImageCamera

PAR_PORT = Path("/dev/v4l/by-path")


def cameras_presentes() -> dict[str, tuple[str, str]]:
    """{port USB: (périphérique /dev/videoN, nom)} des webcams branchées."""
    cameras = {}
    for lien in sorted(PAR_PORT.glob("*-video-index0")):
        port = re.search(r"usb(?:v\d)?-\d+:([\d.]+)", lien.name)
        if not port:
            continue  # encodeurs et ISP internes du Pi, pas des caméras
        peripherique = os.path.realpath(lien)
        try:
            nom = Path(f"/sys/class/video4linux/{Path(peripherique).name}/name").read_text().strip()
        except OSError:
            nom = "caméra"
        cameras[port.group(1)] = (peripherique, nom)
    return cameras


class Camera:
    def __init__(self, bus: Bus, role: str, peripherique: str, nom: str, port: str,
                 largeur: int, hauteur: int, ips: int) -> None:
        self.bus, self.role = bus, role
        self.peripherique, self.nom, self.port = peripherique, nom, port
        self._demande = (largeur, hauteur, ips)
        self.actif = True
        self.erreur: str | None = None
        self.resolution = (0, 0)
        self.ips_mesure = 0.0
        self.images = 0
        self._fil = threading.Thread(target=self._boucle, name=f"cam-{role}", daemon=True)

    @property
    def sujet(self) -> str:
        return S.IMAGE_GAUCHE if self.role == "gauche" else S.IMAGE_DROITE

    def demarrer(self) -> None:
        self._fil.start()

    def arreter(self) -> None:
        self.actif = False

    def resume(self) -> dict[str, Any]:
        return {"ok": self.actif and self.bus.age(self.sujet) < 1.0,
                "ips": round(self.ips_mesure, 1),
                "age_s": round(min(self.bus.age(self.sujet), 99), 2),
                "resolution": list(self.resolution), "port": self.port, "nom": self.nom,
                "erreur": self.erreur}

    def _boucle(self) -> None:
        capture = cv2.VideoCapture(self.peripherique, cv2.CAP_V4L2)
        if not capture.isOpened():
            self._finir("ouverture impossible (déjà utilisée ?)")
            return
        largeur, hauteur, ips = self._demande
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, largeur)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, hauteur)
        capture.set(cv2.CAP_PROP_FPS, ips)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # la dernière image, pas une image en retard
        capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.resolution = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                           int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        self.bus.journaliser(f"camera-{self.role}", f"{self.nom} sur USB {self.port} : "
                             f"{self.resolution[0]}×{self.resolution[1]}", "succes")
        echecs, compte, debut = 0, 0, time.monotonic()
        try:
            while self.actif:
                ok, trame = capture.read()
                jpeg = _en_jpeg(trame) if ok else None
                if jpeg is None:
                    echecs += 1
                    if echecs >= 25:
                        self._finir("plus d'images (débranchée ?)")
                        return
                    time.sleep(0.04)
                    continue
                echecs = 0
                self.images += 1
                self.bus.publier(self.sujet, ImageCamera(
                    t=time.monotonic(), camera=self.role, jpeg=jpeg, largeur=self.resolution[0],
                    hauteur=self.resolution[1], numero=self.images))
                compte += 1
                if (duree := time.monotonic() - debut) >= 2:
                    self.ips_mesure, compte, debut = compte / duree, 0, time.monotonic()
        finally:
            capture.release()
            self.actif = False

    def _finir(self, erreur: str) -> None:
        self.erreur, self.actif = erreur, False
        self.bus.journaliser(f"camera-{self.role}", erreur, "alerte")


def _en_jpeg(trame: np.ndarray | None) -> bytes | None:
    if trame is None or trame.size == 0:
        return None
    if trame.ndim == 1 or (trame.ndim == 2 and trame.shape[0] == 1):
        donnees = trame.tobytes()  # MJPEG brut de la webcam
        return donnees if donnees[:2] == b"\xff\xd8" else None
    ok, tampon = cv2.imencode(".jpg", trame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return tampon.tobytes() if ok else None
