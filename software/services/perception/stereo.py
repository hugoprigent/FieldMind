"""Profondeur par stéréo (SGBM) et nuage de points dans le repère du rover.

La paire est synchronisée et rectifiée : un point du monde se trouve sur la même ligne dans les
deux images, décalé de la disparité d (pixels). Sa profondeur le long de l'axe optique est
Z = f · B / d (f : focale en pixels, B : base). La mise en correspondance est celle de
Hirschmüller (Semi-Global Block Matching, OpenCV) : coût par blocs de 5 × 5 pixels, lissé le long
de plusieurs directions, puis contrôle gauche-droite, unicité et filtre des taches.

Précision : une erreur de disparité δd donne δZ ≈ Z² · δd / (f · B). Avec la base de 6 cm et
320 px de focale, 1/4 de pixel vaut 3 cm à 2 m et 20 cm à 5 m. La distance compte peu pour la
traversabilité : la caméra étant basse, c'est surtout la hauteur des points qui importe, et elle
reste précise (quelques centimètres).
"""

from __future__ import annotations

import base64
import math
import time
from dataclasses import dataclass

import cv2
import numpy as np

from services.protocole.fiche_rover import Camera, Stereo
from services.protocole.messages import ImageStereo

TAILLE_BLOC = 5
PAS_NUAGE = 2
"""Un pixel sur deux en ligne et en colonne dans le nuage : 57 600 points au plus en 640 × 360."""


@dataclass(frozen=True, slots=True)
class Profondeur:
    t: float
    z: np.ndarray
    """Profondeur le long de l'axe optique (m) par pixel, NaN si inconnue ou hors portée."""
    duree_ms: float


def _decoder(donnees: str) -> np.ndarray:
    brut = np.frombuffer(base64.b64decode(donnees), dtype=np.uint8)
    image = cv2.imdecode(brut, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError("image JPEG illisible")
    return image


class ProfondeurStereo:
    """Calcule la profondeur d'une paire et la convertit en points 3D (repère du rover)."""

    def __init__(self, camera: Camera, stereo: Stereo) -> None:
        self.camera = camera
        self.stereo = stereo
        self.focale = camera.focale_px
        self.fb = self.focale * stereo.base_m
        disparites = max(16, 16 * math.ceil(stereo.disparites / 16))
        self._sgbm = cv2.StereoSGBM.create(
            minDisparity=0,
            numDisparities=disparites,
            blockSize=TAILLE_BLOC,
            P1=8 * TAILLE_BLOC**2,
            P2=32 * TAILLE_BLOC**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            preFilterCap=31,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        self._disparites = disparites
        # Rayons des pixels du nuage (sous-échantillonnés), dans le repère du rover :
        # x vers l'avant, y vers la gauche, z vers le haut, pour Z = 1 le long de l'axe optique.
        h, w = camera.hauteur, camera.largeur
        v, u = np.mgrid[0:h:PAS_NUAGE, 0:w:PAS_NUAGE]
        droite = (u + 0.5 - w / 2) / self.focale
        bas = (v + 0.5 - h / 2) / self.focale
        avant, gauche, haut = np.ones_like(droite), -droite, -bas
        tangage = math.radians(camera.inclinaison_deg)
        c, s = math.cos(tangage), math.sin(tangage)
        self._rayons = np.stack(
            [avant * c - haut * s, gauche, avant * s + haut * c], axis=-1
        ).reshape(-1, 3)
        self._origine = np.array(camera.position)
        # Rayons de toutes les colonnes paires, pour les bords d'occultation (pleine hauteur).
        v2, u2 = np.mgrid[0:h, 0:w:PAS_NUAGE]
        droite2 = (u2 + 0.5 - w / 2) / self.focale
        bas2 = (v2 + 0.5 - h / 2) / self.focale
        self._rayons_colonnes = np.stack([c - (-bas2) * s, -droite2, s + (-bas2) * c], axis=-1)

    def calculer(self, message: ImageStereo) -> Profondeur:
        debut = time.perf_counter()
        gauche = _decoder(message.gauche)
        droite = _decoder(message.droite)
        if gauche.shape != droite.shape:
            raise ValueError("images gauche et droite de tailles différentes")
        disparite = self._sgbm.compute(gauche, droite).astype(np.float32) / 16.0
        # Pixels « fantômes » : au bord d'un objet, le bloc mélange l'objet et le fond et donne
        # une profondeur intermédiaire, posée dans le vide. On écarte les pixels dont la
        # disparité varie trop dans leur voisinage 3 × 3.
        noyau = np.ones((3, 3), np.uint8)
        etendue = cv2.dilate(disparite, noyau) - cv2.erode(disparite, noyau)
        bord = etendue > np.maximum(1.0, 0.08 * disparite)
        with np.errstate(divide="ignore", invalid="ignore"):
            z = np.where((disparite > 0.5) & ~bord, self.fb / disparite, np.nan)
        z[z > self.stereo.portee_max] = np.nan
        # Bande gauche sans correspondant possible dans l'image droite.
        z[:, : self._disparites] = np.nan
        return Profondeur(message.t, z, (time.perf_counter() - debut) * 1000)

    def nuage(self, profondeur: Profondeur) -> tuple[np.ndarray, np.ndarray]:
        """Points (N, 3) dans le repère du rover (origine au sol sous son centre), et leur
        profondeur (N,) : elle fixe leur précision."""
        z = profondeur.z[::PAS_NUAGE, ::PAS_NUAGE].reshape(-1)
        ok = np.isfinite(z)
        points: np.ndarray = self._rayons[ok] * z[ok, None] + self._origine
        return points, z[ok]

    def bords_occultation(
        self,
        profondeur: Profondeur,
        portee: float = 3.0,
        saut: float = 0.45,
        saut_max: float = 2.5,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Bords de sol au-delà desquels la vue plonge : dans une même colonne, deux pixels
        mesurés séparés d'au plus 4 rangées sans mesure, dont la profondeur saute bien plus que
        ne le ferait un sol continu. Le pixel proche doit être du sol (pas le haut d'un objet).
        Renvoie les points proches et lointains (M, 3), repère du rover."""
        z = profondeur.z[:, ::PAS_NUAGE]
        valide = np.isfinite(z)
        hauteur_cam = self._origine[2]
        proches: list[np.ndarray] = []
        lointains: list[np.ndarray] = []
        for k in range(1, 5):
            # Rangée r (proche, plus bas dans l'image) et rangée r − k (plus loin).
            zp, zl = z[k:], z[:-k]
            ok = valide[k:] & valide[:-k]
            for j in range(1, k):
                ok &= ~valide[k - j : z.shape[0] - j]
            with np.errstate(invalid="ignore"):
                attendu = k * zp**2 / (self.focale * hauteur_cam)
                ecart = zl - zp
                bord = ok & (zp < portee) & (ecart > saut + 1.5 * attendu) & (ecart < saut_max)
            lig, col = np.nonzero(bord)
            if lig.size == 0:
                continue
            lig_p = lig + k
            # Sol : le pixel proche et celui 3 rangées plus bas sont à peu près à la même hauteur.
            dessous = np.minimum(lig_p + 3, z.shape[0] - 1)
            ray_p = self._rayons_colonnes[lig_p, col]
            ray_d = self._rayons_colonnes[dessous, col]
            pp = ray_p * z[lig_p, col, None] + self._origine
            pd = ray_d * z[dessous, col, None] + self._origine
            with np.errstate(invalid="ignore"):
                sol = np.abs(pp[:, 2] - pd[:, 2]) < 0.06 + 0.2 * np.abs(
                    np.hypot(pp[:, 0], pp[:, 1]) - np.hypot(pd[:, 0], pd[:, 1])
                )
            pl = self._rayons_colonnes[lig, col] * z[lig, col, None] + self._origine
            # Au-delà du bord, du sol (la profondeur croît d'une rangée à l'autre) et non une
            # paroi verticale, comme le pied d'un tronc lointain derrière du sol non mesuré.
            dessus = np.maximum(lig - 3, 0)
            with np.errstate(invalid="ignore"):
                croissance = z[dessus, col] - z[lig, col]
                sol &= croissance > 0.25 * 3 * z[lig, col] ** 2 / (self.focale * hauteur_cam)
            proches.append(pp[sol])
            lointains.append(pl[sol])
        if not proches:
            vide = np.zeros((0, 3))
            return vide, vide
        return np.vstack(proches), np.vstack(lointains)


def rotation(roulis: float, tangage: float, lacet: float) -> np.ndarray:
    """Matrice du repère du rover vers le repère du monde (convention ZYX)."""
    cr, sr = math.cos(roulis), math.sin(roulis)
    cp, sp = math.cos(tangage), math.sin(tangage)
    cy, sy = math.cos(lacet), math.sin(lacet)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def image_profondeur(profondeur: Profondeur, portee: float, largeur: int = 320) -> bytes:
    """Carte de profondeur en couleurs (proche : rouge, loin : bleu, inconnu : noir), JPEG."""
    z = profondeur.z
    hauteur = round(z.shape[0] * largeur / z.shape[1])
    petite = cv2.resize(z, (largeur, hauteur), interpolation=cv2.INTER_NEAREST)
    niveau = np.nan_to_num(np.clip(petite / portee, 0, 1), nan=1.0)
    couleurs = cv2.applyColorMap((255 * (1 - niveau)).astype(np.uint8), cv2.COLORMAP_TURBO)
    couleurs[~np.isfinite(petite)] = 0
    ok, jpeg = cv2.imencode(".jpg", couleurs, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return jpeg.tobytes() if ok else b""
