"""Pose relative d'une croix plane de dimensions connues et caméra calibrée.

Approximation caméra perpendiculaire au plateau : la surface du + est invariante
par rotation dans son plan. Un retour d'inclinaison significative doit invalider
cette approximation avant emploi sur un drone réel.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from fieldmind.materiel.reconnaissance import detecter_croix

from .modele import Cible


@dataclass(frozen=True)
class CalibrationCroix:
    fx: float = 160.0
    fy: float = 160.0
    cx: float = 159.5
    cy: float = 119.5
    largeur_px: int = 320
    hauteur_px: int = 240
    bras_m: float = 0.20
    epaisseur_m: float = 0.04
    ident: str = "simulation-320x240"

    def __post_init__(self) -> None:
        vs = (self.fx, self.fy, self.cx, self.cy, self.bras_m, self.epaisseur_m)
        if not all(math.isfinite(v) for v in vs):
            raise ValueError("calibration non finie")
        if (
            not 10 < self.fx < 5000
            or not 10 < self.fy < 5000
            or not 0 <= self.cx < self.largeur_px
            or not 0 <= self.cy < self.hauteur_px
            or not 0.05 <= self.bras_m <= 1
            or not 0.1 <= self.epaisseur_m / self.bras_m <= 0.4
            or not 32 <= self.largeur_px <= 1920
            or not 32 <= self.hauteur_px <= 1080
        ):
            raise ValueError("calibration de caméra/croix hors limites")


class VisionCroix:
    def __init__(self, calibration: CalibrationCroix | None = None) -> None:
        self.calibration = calibration or CalibrationCroix()

    def estimer(self, image: np.ndarray, t: float, sequence: int) -> Cible | None:
        c = self.calibration
        if image.shape[:2] != (c.hauteur_px, c.largeur_px):
            raise ValueError("résolution différente de la calibration")
        d = detecter_croix(image)
        if not d["trouvee"]:
            return None
        # Une aire image évite de confondre rotation du + et changement d'altitude.
        # Le masque adaptatif de détection est flouté : son contour élargi biaise
        # la hauteur. Mesurer le contour sur les pixels d'origine dans la zone
        # candidate, sur le plateau clair prévu pour cette estimation métrique.
        bx, by, bw, bh = d["boite"]
        gris = cv2.cvtColor(image[by : by + bh, bx : bx + bw], cv2.COLOR_BGR2GRAY)
        _, masque = cv2.threshold(gris, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centre = (float(d["centre"][0] - bx), float(d["centre"][1] - by))
        valides = [p for p in contours if cv2.pointPolygonTest(p, centre, False) >= 0]
        if not valides:
            return None
        aire = cv2.contourArea(max(valides, key=cv2.contourArea))
        aire_reelle = 2 * c.bras_m * c.epaisseur_m - c.epaisseur_m**2
        hauteur = math.sqrt(c.fx * c.fy * aire_reelle / max(1, aire))
        x, y = d["centre"]
        return Cible(
            t,
            sequence,
            -(y - c.cy) * hauteur / c.fy,
            (x - c.cx) * hauteur / c.fx,
            hauteur,
            qualite=min(1.0, max(0.0, float(d["score"]))),
        )
