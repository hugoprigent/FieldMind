"""Caméras simulées rendues par le navigateur du poste (WebGL), renvoyées au rover simulé.

En simulation, le poste (`web/js/camera_simulee.js`) reconstruit le monde simulé en three.js
et rend les deux caméras depuis la pose vraie du rover, à la demande du serveur. Les images
JPEG reviennent par la WebSocket du poste (trames binaires, `api/camera_simulee.py`) et sont
publiées sur le bus comme celles des vraies webcams : stéréo, vision, photos de mission et
envoi au GX10 les utilisent sans le savoir. Le lancer de rayons de `Monde.rendre_camera`
reste le repli (`MaterielSimule`), quand aucun navigateur ne rend.

Synchronisation : une demande porte un numéro, l'instant où la pose a été lue et cette pose ;
chaque image reçue est datée de cet instant (pas de sa réception), et les deux images d'une
demande partagent la même date : la paire stéréo est synchrone. Une seule demande est en cours
à la fois, la cadence s'adapte donc au navigateur et au réseau sans jamais dépasser `ips`.

Garde-fous : seules les images demandées sont acceptées (demande en cours, une fois par
caméra), taille bornée, JPEG aux dimensions demandées, lues dans l'en-tête sans décoder.
"""

from __future__ import annotations

import asyncio
import itertools
import math
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.perception.projection import ModeleCamera
from fieldmind.types import ImageCamera, Pose2D

LARGEUR, HAUTEUR = 640, 360
IPS_MAX = 10.0
TAILLE_MAX_JPEG = 400_000
"""Octets : un 640 × 360 du rendu pèse 40 à 120 ko selon la qualité."""
DELAI_ACTIF_S = 2.0
"""Sans image du navigateur depuis ce délai, le matériel simulé repasse au lancer de rayons."""

_SOF = frozenset({0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF})


def dimensions_jpeg(donnees: bytes) -> tuple[int, int] | None:
    """(largeur, hauteur) lues dans le segment SOF d'un JPEG, sans le décoder ; None si ce
    n'est pas un JPEG lisible."""
    n = len(donnees)
    if n < 4 or donnees[0] != 0xFF or donnees[1] != 0xD8:
        return None
    i = 2
    while i + 4 <= n:
        if donnees[i] != 0xFF:
            return None
        marqueur = donnees[i + 1]
        if marqueur == 0xFF:  # octets de remplissage
            i += 1
            continue
        if marqueur == 0x01 or 0xD0 <= marqueur <= 0xD8:  # marqueurs sans longueur
            i += 2
            continue
        if marqueur in (0xD9, 0xDA):  # fin d'image ou données avant tout SOF
            return None
        longueur = (donnees[i + 2] << 8) | donnees[i + 3]
        if longueur < 2:
            return None
        if marqueur in _SOF:
            if longueur < 7 or i + 9 > n:
                return None
            hauteur = (donnees[i + 5] << 8) | donnees[i + 6]
            largeur = (donnees[i + 7] << 8) | donnees[i + 8]
            return largeur, hauteur
        i += 2 + longueur
    return None


def geometrie_cameras(
    profil: Profil, largeur: int = LARGEUR, hauteur: int = HAUTEUR
) -> dict[str, dict[str, Any]]:
    """Géométrie exacte des deux caméras simulées (celle du lancer de rayons : modèle par
    défaut, sans calibration) : centre et rotation caméra (OpenCV) → base_link, et intrinsèques
    (fx, fy, cx, cy) pour l'image demandée, centres de pixel entiers d'OpenCV."""
    sortie: dict[str, dict[str, Any]] = {}
    for nom in ("gauche", "droite"):
        modele = ModeleCamera(profil, nom, calibration=False)
        k = modele.matrice(largeur, hauteur)
        sortie[nom] = {
            "centre": [round(float(v), 6) for v in modele.centre],
            "rotation": [[round(float(v), 9) for v in ligne] for ligne in modele.rotation],
            "k": [
                round(float(k[0, 0]), 6),
                round(float(k[1, 1]), 6),
                round(float(k[0, 2]), 6),
                round(float(k[1, 2]), 6),
            ],
        }
    return sortie


def description_rendu(monde: Any) -> dict[str, Any]:
    """Tout ce qu'il faut au navigateur pour reconstruire la scène vue par les caméras :
    `Monde.description()` (obstacles, pièce, limites…) complétée des couleurs du sol, du ciel
    et de la pièce, et de la mire de calibration si elle est posée."""
    description: dict[str, Any] = dict(monde.description())
    description["couleurs"] = {"sol": list(monde.couleur_sol), "ciel": list(monde.couleur_ciel)}
    if monde.piece is not None and description.get("piece") is not None:
        description["piece"] = {**description["piece"], "couleur": list(monde.piece.couleur)}
    mire = getattr(monde, "mire", None)
    description["mire"] = (
        None
        if mire is None
        else {
            "centre": [float(v) for v in mire.centre],
            "rotation": [[float(v) for v in ligne] for ligne in mire.rotation],
            "colonnes": int(mire.colonnes),
            "lignes": int(mire.lignes),
            "case_m": float(mire.case_m),
            "bord_cases": float(mire.bord_cases),
        }
    )
    return description


@dataclass(slots=True)
class Demande:
    ident: int
    t: float  # instant de lecture de la pose (date des images)
    pose: Pose2D
    vues: dict[str, str]  # rôle publié → géométrie rendue (« cameras_inverser »)
    recues: set[str] = field(default_factory=set)

    @property
    def complete(self) -> bool:
        return self.recues >= set(self.vues)


class CamerasNavigateur:
    """Côté rover : demandes de rendu, réception et publication des images du navigateur."""

    def __init__(
        self,
        bus: Bus,
        profil: Profil,
        monde: Any,
        numeroter: Callable[[], int] | None = None,
        ips: float = 6.0,
    ) -> None:
        self.bus, self.profil, self.monde = bus, profil, monde
        compteur = itertools.count(1)
        self._numeroter = numeroter or (lambda: next(compteur))
        self._ips_defaut = ips
        self._ids = itertools.count(1)
        self.demande: Demande | None = None
        self.paire_recue = asyncio.Event()
        self.derniere_image = -math.inf  # monotone de la dernière image acceptée
        self._paires: deque[float] = deque(maxlen=40)
        self.refus: dict[str, int] = {}
        self._signature: tuple[Any, ...] | None = None
        self.version_monde = 0

    # ------------------------------------------------------------------ réglages
    @property
    def ips(self) -> float:
        valeur = float(self.profil.valeur("simulation.camera_navigateur_ips", self._ips_defaut))
        return max(0.5, min(IPS_MAX, valeur))

    @property
    def qualite(self) -> float:
        valeur = float(self.profil.valeur("simulation.camera_navigateur_qualite", 0.82))
        return max(0.5, min(0.95, valeur))

    # ------------------------------------------------------------------ état
    def actif(self, delai: float = DELAI_ACTIF_S) -> bool:
        """Vrai si une image du navigateur est arrivée depuis moins de `delai` secondes."""
        return time.monotonic() - self.derniere_image < delai

    def ips_recues(self, fenetre: float = 3.0) -> float:
        """Paires complètes reçues par seconde, sur les `fenetre` dernières secondes."""
        maintenant = time.monotonic()
        recentes = [t for t in self._paires if maintenant - t <= fenetre]
        if len(recentes) < 2:
            return 0.0 if not recentes or maintenant - recentes[-1] > fenetre else 1 / fenetre
        return (len(recentes) - 1) / max(recentes[-1] - recentes[0], 1e-3)

    def description(self) -> dict[str, Any]:
        self._actualiser_version()
        return {"version": self.version_monde, "monde": description_rendu(self.monde)}

    def _actualiser_version(self) -> int:
        mire = getattr(self.monde, "mire", None)
        signature = (
            getattr(self.monde, "scenario", None),
            len(self.monde.obstacles),
            id(mire),
            None if mire is None else tuple(float(v) for v in mire.centre),
        )
        if signature != self._signature:
            self._signature = signature
            self.version_monde += 1
        return self.version_monde

    # ------------------------------------------------------------------ demandes
    def nouvelle_demande(self, pose: Pose2D, vues: dict[str, str]) -> dict[str, Any] | None:
        """Message `camera_simulee` pour le poste qui rend (None si aucune caméra active)."""
        if not vues:
            self.demande = None
            return None
        demande = Demande(
            next(self._ids), time.monotonic(), Pose2D(pose.x, pose.y, pose.cap), dict(vues)
        )
        self.demande = demande
        self.paire_recue.clear()
        return {
            "type": "camera_simulee",
            "id": demande.ident,
            "t": round(demande.t, 4),
            "pose": [round(pose.x, 5), round(pose.y, 5), round(pose.cap, 6)],
            "vues": demande.vues,
            "largeur": LARGEUR,
            "hauteur": HAUTEUR,
            "qualite": round(self.qualite, 2),
            "monde": self._actualiser_version(),
            "cameras": geometrie_cameras(self.profil),
        }

    def complete(self) -> bool:
        return self.demande is not None and self.demande.complete

    def abandonner(self) -> None:
        """Demande expirée : ses images éventuelles seront refusées."""
        self.demande = None

    def recevoir(self, ident: Any, role: Any, jpeg: bytes) -> str | None:
        """Image d'une caméra pour la demande `ident`. Publie et renvoie None si elle est
        acceptée, sinon la raison du refus (et rien n'est publié)."""
        raison = self._verifier(ident, role, jpeg)
        if raison is not None:
            self.refus[raison] = self.refus.get(raison, 0) + 1
            return raison
        demande = self.demande
        assert demande is not None
        demande.recues.add(role)
        maintenant = time.monotonic()
        self.derniere_image = maintenant
        if demande.complete:
            self._paires.append(maintenant)
            self.paire_recue.set()
        self.bus.publier(
            S.IMAGE_GAUCHE if role == "gauche" else S.IMAGE_DROITE,
            ImageCamera(
                t=demande.t,
                camera=role,
                jpeg=bytes(jpeg),
                largeur=LARGEUR,
                hauteur=HAUTEUR,
                numero=self._numeroter(),
            ),
        )
        return None

    def _verifier(self, ident: Any, role: Any, jpeg: bytes) -> str | None:
        demande = self.demande
        if demande is None or type(ident) is not int or ident != demande.ident:
            return "non_demandee"
        if role not in demande.vues:
            return "camera"
        if role in demande.recues:
            return "doublon"
        if not 100 <= len(jpeg) <= TAILLE_MAX_JPEG:
            return "taille"
        if dimensions_jpeg(jpeg) != (LARGEUR, HAUTEUR):
            return "format"
        return None
