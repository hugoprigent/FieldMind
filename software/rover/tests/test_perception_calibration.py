"""Calibration stéréo sur des mires rendues : base de 0,12 m retrouvée, fichier relu."""

from __future__ import annotations

import math
import tomllib
from pathlib import Path

import cv2
import numpy as np
import pytest

from fieldmind.config import FICHIER_PROFIL, Profil
from fieldmind.perception.calibration_stereo import CalibrationStereo
from fieldmind.perception.projection import ModeleCamera, lire_calibration
from fieldmind.perception.stereo import Stereo
from fieldmind.simulation.monde import Boite, Monde
from fieldmind.types import ImageCamera, Pose2D


def paire(monde: Monde, pose: Pose2D, profil: Profil, t: float
          ) -> tuple[ImageCamera, ImageCamera]:
    images = []
    for camera in ("gauche", "droite"):
        bgr = monde.rendre_camera(pose, camera, 640, 360, profil)
        jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()
        images.append(ImageCamera(t, camera, jpeg, 640, 360))
    return images[0], images[1]


def distance_boite(b: Boite, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    c, s = math.cos(b.cap), math.sin(b.cap)
    lx = c * (x - b.x) + s * (y - b.y)
    ly = -s * (x - b.x) + c * (y - b.y)
    return np.hypot(np.maximum(np.abs(lx) - b.longueur / 2, 0),
                    np.maximum(np.abs(ly) - b.largeur / 2, 0))


def profil_test(calibration: Path) -> Profil:
    with open(FICHIER_PROFIL, "rb") as f:
        base = tomllib.load(f)
    base["cameras"]["calibration"] = str(calibration)
    return Profil(base, {}, None)


def capturer(calibration: CalibrationStereo, profil: Profil, paires: int = 10) -> int:
    """Mire tenue devant les caméras à 0,45–0,9 m, inclinée au hasard (jusqu'à ±23°)."""
    monde = Monde(0, "champ")
    rng = np.random.default_rng(3)
    essais = 0
    while calibration.nombre < paires and essais < 3 * paires:
        essais += 1
        d = rng.uniform(0.45, 0.9)
        hauteur = 0.265 - math.tan(math.radians(10)) * d + rng.uniform(-0.1, 0.1) * d
        monde.placer_mire((0.415 + d, rng.uniform(-0.15, 0.15) * d, hauteur),
                          rng.uniform(-0.3, 0.3), rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4))
        gauche = monde.rendre_camera(Pose2D(), "gauche", 1280, 720, profil, bruit=1.0)
        droite = monde.rendre_camera(Pose2D(), "droite", 1280, 720, profil, bruit=1.0)
        calibration.ajouter_paire(gauche, droite)
    return essais


def test_calibration_retrouve_la_base(tmp_path: Path) -> None:
    fichier = tmp_path / "stereo.npz"
    profil = profil_test(fichier)
    calibration = CalibrationStereo(profil)
    assert calibration.nombre == 0
    with pytest.raises(ValueError, match="paires"):
        calibration.calculer()
    # Sans mire dans l'image : paire refusée.
    vide = Monde(0, "champ").rendre_camera(Pose2D(), "gauche", 1280, 720, profil)
    assert not calibration.ajouter_paire(vide, vide)

    capturer(calibration, profil)
    assert calibration.nombre == 10
    bilan = calibration.calculer()
    print(f"\ncalibration : base {1000 * bilan['base_m']:.2f} mm, RMS {bilan['erreur_rms_px']} px,"
          f" épipolaire {bilan['erreur_epipolaire_px']} px, champ {bilan['hfov_deg']}°")
    assert bilan["base_m"] == pytest.approx(0.12, abs=0.002)
    assert bilan["erreur_rms_px"] < 0.5
    assert bilan["erreur_epipolaire_px"] < 0.5
    assert bilan["hfov_deg"] == pytest.approx(78.0, abs=0.5)
    assert bilan["ok"] and not bilan["avertissements"]

    assert calibration.enregistrer() == fichier
    donnees = lire_calibration(fichier)
    assert donnees is not None
    assert float(np.linalg.norm(donnees["t"])) == pytest.approx(bilan["base_m"], abs=1e-5)
    modele = ModeleCamera(profil, "droite")
    assert modele.calibree and modele.base == pytest.approx(0.12, abs=0.002)
    assert modele.centre[1] == pytest.approx(-0.06, abs=0.002)

    # La stéréo charge la calibration, rectifie, et trouve toujours la marche à sa place.
    stereo = Stereo(profil)
    assert stereo.calibree
    monde = Monde(0, "salle")
    pose = Pose2D(5.6, 2.4, 0.0)
    stereo.traiter(*paire(monde, pose, profil, 0.0))
    resultat = stereo.traiter(*paire(monde, pose, profil, 0.25))
    assert resultat is not None and resultat.calibree
    points = resultat.obstacles.points.astype(np.float64)
    x, y = pose.x + points[:, 0], pose.y + points[:, 1]  # cap nul
    marche = next(o for o in monde.obstacles if o.nom == "marche")
    assert isinstance(marche, Boite)
    assert (distance_boite(marche, x, y) <= 0.10).sum() >= 5
