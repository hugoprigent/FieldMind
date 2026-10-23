"""Suivi d'une cible (une personne) dans l'image de la caméra gauche, sur le Pi, à ≈ 10 Hz.

La désignation est lente et se fait une fois (clic sur l'image, ou « suis la personne au t-shirt
rouge » par le modèle incarné du GX10) ; le suivi image par image, lui, doit être rapide et ne
dépendre d'aucun réseau : traqueur VitTrack (OpenCV, réseau ONNX de 700 ko), repli sur KCF.

- **Distance** : points du lidar dans la direction de la boîte (vus depuis la caméra), sinon taille
  de la boîte (personne de 1,70 m, pieds et tête visibles).
- **Perte** : score du traqueur trop bas plusieurs images de suite, ou boîte sortie de l'image.
- **Reprise** : parmi les personnes détectées par le GX10 (`S.DETECTIONS`), celle dont les
  couleurs du haut du corps ressemblent le plus à la cible (histogramme HSV).

Publie `S.CIBLE` (dict, voir `resume`). Conseil seulement : la sécurité ne s'en sert que pour ne
pas confondre la personne suivie avec une autre personne (voir superviseur).
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.types import ImageCamera, Scan

journal = logging.getLogger("fieldmind.suivi")

MODELE_VITTRACK = Path("~/fieldmind/modeles/vittrack.onnx").expanduser()
LARGEUR_TRAITEMENT = 640
TAILLE_PERSONNE_M = 1.70
SEUIL_SCORE = 0.30
PERTES_AVANT_PERDU = 5
SEUIL_RESSEMBLANCE = 0.45  # distance de Bhattacharyya entre histogrammes (0 = identiques)
GARDE_S = 1.0  # sans `garder()` depuis plus longtemps, le suivi s'arrête (mission finie)


def _decoder(image: ImageCamera) -> np.ndarray | None:
    import cv2

    drapeau = cv2.IMREAD_REDUCED_COLOR_2 if image.largeur >= 2 * LARGEUR_TRAITEMENT else (
        cv2.IMREAD_COLOR)
    return cv2.imdecode(np.frombuffer(image.jpeg, np.uint8), drapeau)


def histogramme(bgr: np.ndarray, boite_px: tuple[int, int, int, int]) -> np.ndarray | None:
    """Couleurs du haut du corps (tiers supérieur à moitié de la boîte) : le vêtement."""
    import cv2

    x, y, lb, hb = boite_px
    zone = bgr[max(0, y + hb // 6): max(0, y + hb // 2),
               max(0, x + lb // 5): max(0, x + 4 * lb // 5)]
    if zone.size == 0:
        return None
    hsv = cv2.cvtColor(zone, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [18, 8], [0, 180, 0, 256])
    return cv2.normalize(hist, hist).flatten()


def creer_traqueur() -> tuple[Any, str]:
    import cv2

    if MODELE_VITTRACK.exists() and hasattr(cv2, "TrackerVit_create"):
        parametres = cv2.TrackerVit_Params()
        parametres.net = str(MODELE_VITTRACK)
        return cv2.TrackerVit_create(parametres), "VitTrack"
    if hasattr(cv2, "TrackerKCF_create"):  # OpenCV « contrib » (paquet apt du Pi)
        return cv2.TrackerKCF_create(), "KCF"
    return cv2.TrackerMIL_create(), "MIL"  # toujours présent, plus lent


@dataclass
class EtatCible:
    etat: str = "inactif"  # inactif | suivi | perdu
    description: str = ""
    camera: str = "gauche"
    boite: tuple[float, float, float, float] | None = None  # u0, v0, u1, v1 normalisés
    score: float = 0.0
    azimut: float | None = None  # rad, depuis la caméra, > 0 à gauche
    distance_m: float | None = None  # depuis la caméra
    position: tuple[float, float] | None = None  # base_link
    source_distance: str | None = None  # lidar | taille
    traqueur: str = ""
    t: float = 0.0
    perdu_depuis: float | None = None
    hz: float = 0.0
    historique: list[float] = field(default_factory=list)


class SuiviCible:
    def __init__(self, bus: Bus, profil: Profil) -> None:
        self.bus, self.profil = bus, profil
        self.etat = EtatCible()
        self._verrou = threading.Lock()
        self._traqueur: Any = None
        self._modele_couleur: np.ndarray | None = None
        self._pertes = 0
        self._garde = 0.0
        self._image_traitee = -1
        self._actif = False
        self._fil = threading.Thread(target=self._boucle, name="suivi", daemon=True)
        bus.abonner(S.DETECTIONS, self._detections)

    # ------------------------------------------------------------------ commande

    def demarrer_fil(self) -> None:
        self._actif = True
        self._fil.start()

    def arreter_fil(self) -> None:
        self._actif = False

    def demarrer(self, image: ImageCamera, boite: tuple[float, float, float, float],
                 description: str = "") -> bool:
        """Initialise le traqueur sur l'image où la cible a été désignée."""
        bgr = _decoder(image)
        if bgr is None:
            return False
        h, lg = bgr.shape[:2]
        u0, v0, u1, v1 = (min(max(c, 0.0), 1.0) for c in boite)
        px = (int(u0 * lg), int(v0 * h), max(8, int((u1 - u0) * lg)), max(8, int((v1 - v0) * h)))
        traqueur, nom = creer_traqueur()
        traqueur.init(bgr, px)
        with self._verrou:
            self._traqueur = traqueur
            self._modele_couleur = histogramme(bgr, px)
            self._pertes = 0
            self._garde = time.monotonic()
            self.etat = EtatCible(etat="suivi", description=description, camera=image.camera,
                                  boite=(u0, v0, u1, v1), score=1.0, traqueur=nom,
                                  t=time.monotonic())
        self.bus.journaliser("suivi", f"cible désignée ({nom}) : {description or 'clic'}",
                             "succes")
        return True

    def garder(self) -> None:
        """Appelé à chaque pas de la mission « suivre » : sans lui, le suivi s'arrête seul."""
        self._garde = time.monotonic()

    def arreter(self) -> None:
        with self._verrou:
            self._traqueur = None
            if self.etat.etat != "inactif":
                self.etat = EtatCible()
        self.bus.publier(S.CIBLE, self.resume())

    def cible(self) -> EtatCible:
        with self._verrou:
            return self.etat

    def pour_securite(self) -> tuple[float, float] | None:
        """(azimut depuis base_link, distance) de la personne suivie, si le suivi est en cours :
        le superviseur ne la confond pas avec une autre personne."""
        e = self.cible()
        if e.etat != "suivi" or e.position is None or time.monotonic() - e.t > 0.6:
            return None
        x, y = e.position
        return math.atan2(y, x), math.hypot(x, y)

    def resume(self) -> dict[str, Any] | None:
        e = self.cible()
        if e.etat == "inactif":
            return None
        return {"etat": e.etat, "description": e.description, "camera": e.camera,
                "boite": [round(c, 4) for c in e.boite] if e.boite else None,
                "score": round(e.score, 2),
                "distance_m": None if e.distance_m is None else round(e.distance_m, 2),
                "source_distance": e.source_distance, "traqueur": e.traqueur,
                "hz": round(e.hz, 1)}

    # ------------------------------------------------------------------ boucle

    def _boucle(self) -> None:
        while self._actif:
            debut = time.monotonic()
            try:
                if self._traqueur is not None:
                    if debut - self._garde > GARDE_S:
                        self.arreter()
                    else:
                        self._pas(debut)
            except Exception:
                journal.exception("suivi")
            time.sleep(max(0.01, 0.1 - (time.monotonic() - debut)))

    def _pas(self, maintenant: float) -> None:
        sujet = S.IMAGE_GAUCHE if self.etat.camera == "gauche" else S.IMAGE_DROITE
        image: ImageCamera | None = self.bus.dernier(sujet)
        if image is None or image.numero == self._image_traitee:
            return
        self._image_traitee = image.numero
        bgr = _decoder(image)
        if bgr is None:
            return
        h, lg = bgr.shape[:2]
        with self._verrou:
            traqueur = self._traqueur
        if traqueur is None:
            return
        ok, px = traqueur.update(bgr)
        # Seul VitTrack donne un score ; les autres disent seulement s'ils ont trouvé
        score = float(traqueur.getTrackingScore()) if self.etat.traqueur == "VitTrack" else (
            1.0 if ok else 0.0)
        x, y, lb, hb = (int(c) for c in px) if ok else (0, 0, 0, 0)
        dans_image = ok and lb > 4 and hb > 4 and x + lb > 0 and x < lg and y + hb > 0 and y < h
        e = self.etat
        precedent = e.t
        if dans_image and score >= SEUIL_SCORE:
            self._pertes = 0
            boite = (max(0.0, x / lg), max(0.0, y / h), min(1.0, (x + lb) / lg),
                     min(1.0, (y + hb) / h))
            azimut, distance, source = self._mesurer(boite)
            position = None
            if distance is not None and azimut is not None:
                cx, cy = self._position_camera()
                position = (cx + distance * math.cos(azimut), cy + distance * math.sin(azimut))
            nouveau = EtatCible(etat="suivi", description=e.description, camera=e.camera,
                                boite=boite, score=score, azimut=azimut, distance_m=distance,
                                position=position, source_distance=source,
                                traqueur=e.traqueur, t=maintenant)
        else:
            self._pertes += 1
            if self._pertes < PERTES_AVANT_PERDU:
                return
            nouveau = EtatCible(etat="perdu", description=e.description, camera=e.camera,
                                boite=e.boite, score=score, traqueur=e.traqueur, t=maintenant,
                                perdu_depuis=e.perdu_depuis if e.etat == "perdu" else maintenant)
        periode = maintenant - precedent
        nouveau.hz = 0.8 * e.hz + 0.2 * (1.0 / periode) if 0 < periode < 2 else e.hz
        with self._verrou:
            if self._traqueur is traqueur:
                self.etat = nouveau
        self.bus.publier(S.CIBLE, self.resume())

    # ------------------------------------------------------------------ mesures

    def _position_camera(self) -> tuple[float, float]:
        p = self.profil
        base = float(p.valeur("cameras.base_m", 0.12))
        cote = 1.0 if self.etat.camera == "gauche" else -1.0
        return float(p.valeur("cameras.x_m", 0.415)), cote * base / 2

    def _champs(self) -> tuple[float, float]:
        hfov = math.radians(float(self.profil.valeur("cameras.hfov_deg", 70.0)))
        vfov = 2 * math.atan(math.tan(hfov / 2) * 9 / 16)
        return hfov, vfov

    def _mesurer(self, boite: tuple[float, float, float, float]
                 ) -> tuple[float | None, float | None, str | None]:
        u0, v0, u1, v1 = boite
        hfov, vfov = self._champs()
        t = math.tan(hfov / 2)
        azimut = math.atan((0.5 - (u0 + u1) / 2) * 2 * t)
        demi_largeur = max(math.radians(2.0), 0.35 * abs(math.atan((0.5 - u0) * 2 * t)
                                                         - math.atan((0.5 - u1) * 2 * t)))
        distance = self._distance_lidar(azimut, demi_largeur)
        if distance is not None:
            return azimut, distance, "lidar"
        hauteur = v1 - v0
        if hauteur > 0.15 and v0 > 0.02 and v1 < 0.98:  # tête et pieds visibles
            return azimut, TAILLE_PERSONNE_M / (hauteur * 2 * math.tan(vfov / 2)), "taille"
        return azimut, None, None

    def _distance_lidar(self, azimut: float, demi_largeur: float) -> float | None:
        scan: Scan | None = self.bus.dernier(S.SCAN)
        if scan is None or self.bus.age(S.SCAN) > 0.5:
            return None
        points = scan.points()
        if len(points) == 0:
            return None
        cx, cy = self._position_camera()
        dx, dy = points[:, 0] - cx, points[:, 1] - cy
        portee = np.hypot(dx, dy)
        ecart = np.abs(np.arctan2(np.sin(np.arctan2(dy, dx) - azimut),
                                  np.cos(np.arctan2(dy, dx) - azimut)))
        dans = (ecart < demi_largeur) & (portee > 0.25) & (portee < 8.0) & (dx > 0)
        if dans.sum() < 2:
            return None
        proches = np.sort(portee[dans])
        # Le groupe le plus proche (les jambes), pas le mur derrière : quart inférieur
        return float(np.percentile(proches[proches < proches[0] + 0.5], 50))

    # ------------------------------------------------------------------ reprise

    def _detections(self, message: Any) -> None:
        """Cible perdue : reprendre la personne détectée qui lui ressemble le plus."""
        e = self.etat
        if e.etat != "perdu" or not isinstance(message, dict) or self._modele_couleur is None:
            return
        sujet = S.IMAGE_GAUCHE if e.camera == "gauche" else S.IMAGE_DROITE
        image: ImageCamera | None = self.bus.dernier(sujet)
        if image is None:
            return
        bgr = _decoder(image)
        if bgr is None:
            return
        import cv2

        h, lg = bgr.shape[:2]
        meilleure: tuple[float, tuple[int, int, int, int]] | None = None
        for objet in message.get("objets") or []:
            if objet.get("label") not in ("personne", "person") or not objet.get("boite"):
                continue
            u0, v0, u1, v1 = objet["boite"]
            px = (int(u0 * lg), int(v0 * h), max(8, int((u1 - u0) * lg)),
                  max(8, int((v1 - v0) * h)))
            hist = histogramme(bgr, px)
            if hist is None:
                continue
            ecart = float(cv2.compareHist(self._modele_couleur, hist,
                                          cv2.HISTCMP_BHATTACHARYYA))
            if ecart < SEUIL_RESSEMBLANCE and (meilleure is None or ecart < meilleure[0]):
                meilleure = (ecart, px)
        if meilleure is None:
            return
        traqueur, nom = creer_traqueur()
        traqueur.init(bgr, meilleure[1])
        with self._verrou:
            self._traqueur = traqueur
            self._pertes = 0
            self.etat = EtatCible(etat="suivi", description=e.description, camera=e.camera,
                                  boite=None, score=1.0, traqueur=nom, t=time.monotonic())
        self.bus.journaliser("suivi", f"cible retrouvée (ressemblance {1 - meilleure[0]:.0%})",
                             "succes")
