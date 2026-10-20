"""Fiche technique du rover (config/rover_fieldmind.json), partagée par le cerveau et le simulateur.

Le cerveau n'en lit que la géométrie, les limites, la sécurité et la position des capteurs : ce
qu'un intégrateur connaît du robot réel. La section « physique » ne sert qu'au simulateur.
"""

from __future__ import annotations

import math
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field


class _Modele(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class Geometrie(_Modele):
    empattement: float = Field(gt=0)
    voie_avant: float = Field(gt=0)
    voie_arriere: float = Field(gt=0)
    rayon_roue: float = Field(gt=0)
    largeur_roue: float = Field(gt=0)
    longueur: float = Field(gt=0)
    largeur: float = Field(gt=0)
    essieu_arriere_x: float
    """Position de l'essieu arrière le long de l'axe avant, depuis le centre de l'emprise (m)."""
    garde_au_sol: float = Field(gt=0)

    @property
    def demi_largeur(self) -> float:
        return self.largeur / 2

    @property
    def rayon_englobant(self) -> float:
        """Rayon du cercle qui contient toute l'emprise, centré sur le centre du rover."""
        return math.hypot(self.longueur / 2, self.largeur / 2)


class Limites(_Modele):
    braquage_max_deg: float = Field(gt=0, lt=60)
    vitesse_braquage_deg_s: float = Field(gt=0)
    vitesse_max: float = Field(gt=0)
    vitesse_arriere_max: float = Field(gt=0)
    acceleration_max: float = Field(gt=0)
    deceleration_max: float = Field(gt=0)
    deceleration_urgence: float = Field(gt=0)

    @property
    def braquage_max(self) -> float:
        return math.radians(self.braquage_max_deg)

    @property
    def vitesse_braquage(self) -> float:
        return math.radians(self.vitesse_braquage_deg_s)


class Securite(_Modele):
    distance_arret_avant: float = Field(gt=0)
    secteur_avant_deg: float = Field(gt=0, le=90)
    chien_de_garde_s: float = Field(gt=0)


class Lidar(_Modele):
    modele: str
    position: tuple[float, float, float]
    frequence_hz: float = Field(gt=0)
    mesures_par_tour: int = Field(gt=0)
    portee_min: float = Field(ge=0)
    portee_max: float = Field(gt=0)
    bruit_sigma: float = Field(ge=0)
    taux_pertes: float = Field(ge=0, lt=1)


class Imu(_Modele):
    modele: str
    frequence_hz: float = Field(gt=0)
    bruit_gyro: float = Field(ge=0)
    biais_gyro_z: float
    bruit_accel: float = Field(ge=0)
    bruit_cap_absolu_deg: float = Field(ge=0)


class Gnss(_Modele):
    modele: str
    position: tuple[float, float, float]
    frequence_hz: float = Field(gt=0)
    sigma_rtk_fixe: float = Field(gt=0)
    latence_s: float = Field(ge=0)


class Camera(_Modele):
    """Caméra avant (modèle sténopé). Les coordonnées image sont normalisées : u vers la droite,
    v vers le bas, entre 0 et 1."""

    modele: str
    position: tuple[float, float, float]
    inclinaison_deg: float = Field(ge=-60, le=30)
    """Angle de l'axe optique par rapport à l'horizontale : négatif vers le sol."""
    champ_horizontal_deg: float = Field(gt=10, lt=170)
    largeur: int = Field(gt=0, le=1920)
    hauteur: int = Field(gt=0, le=1080)
    frequence_hz: float = Field(gt=0)
    qualite_jpeg: int = Field(ge=30, le=95)

    @property
    def focale_px(self) -> float:
        return self.largeur / 2 / math.tan(math.radians(self.champ_horizontal_deg) / 2)

    @property
    def champ_vertical_deg(self) -> float:
        return math.degrees(2 * math.atan(self.hauteur / 2 / self.focale_px))


class Stereo(_Modele):
    """Paire stéréo synchronisée : la caméra avant (`Capteurs.camera`) est l'œil gauche, l'œil
    droit est décalé de `base_m` vers la droite, mêmes optique, résolution et orientation.
    Images rectifiées (étalonnage fait une fois pour toutes sur le vrai module)."""

    modele: str
    base_m: float = Field(gt=0, le=0.5)
    disparites: int = Field(gt=0, le=256)
    """Nombre de disparités cherchées (multiple de 16) : fixe la distance minimale mesurée."""
    portee_max: float = Field(gt=0)
    """Au-delà, la profondeur est trop imprécise pour la carte des hauteurs (m)."""


class Codeurs(_Modele):
    frequence_hz: float = Field(gt=0)
    impulsions_par_tour: int = Field(gt=0)


class Capteurs(_Modele):
    lidar: Lidar
    imu: Imu
    gnss: Gnss
    codeurs: Codeurs
    camera: Camera | None = None
    stereo: Stereo | None = None


class Moteur(_Modele):
    rapport_transmission: float = Field(gt=0)
    couple_max: float = Field(gt=0)
    regime_max_tr_min: float = Field(gt=0)
    rendement: float = Field(gt=0, le=1)


class Servo(_Modele):
    couple_max: float = Field(gt=0)


class Batterie(_Modele):
    cellules_serie: int = Field(gt=0)
    capacite_ah: float = Field(gt=0)
    resistance_interne: float = Field(ge=0)
    consommation_repos_w: float = Field(ge=0)


class Physique(_Modele):
    masse_totale: float = Field(gt=0)
    masse_roue: float = Field(gt=0)
    frottement_pneu: float = Field(gt=0)
    moteur: Moteur
    servo: Servo
    batterie: Batterie


class FicheRover(_Modele):
    nom: str
    source: str = ""
    hypotheses: list[str] = Field(default_factory=list)
    geometrie: Geometrie
    limites: Limites
    securite: Securite
    capteurs: Capteurs
    physique: Physique

    @classmethod
    def charger(cls, chemin: Path) -> FicheRover:
        return cls.model_validate_json(chemin.read_text(encoding="utf-8"))
