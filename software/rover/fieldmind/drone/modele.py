"""Grandeurs physiques : avant/droite du drone, hauteur au-dessus du plateau, SI."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Cible:
    t: float
    sequence: int
    avant_m: float
    droite_m: float
    hauteur_m: float
    qualite: float = 1.0
    source: str = "camera_bas"

    def valide(self, maintenant: float, age_max: float = 0.5) -> bool:
        valeurs = (self.t, self.avant_m, self.droite_m, self.hauteur_m, self.qualite)
        return (
            all(math.isfinite(v) for v in valeurs)
            and 0 <= maintenant - self.t <= age_max
            and 0.08 <= self.hauteur_m <= 4
            and 0.7 <= self.qualite <= 1
            and self.sequence >= 0
        )


@dataclass(frozen=True)
class Observation:
    t: float
    sequence: int
    hauteur_m: float | None
    cap_rad: float | None
    vitesse_z: float = 0.0
    vitesse_lacet: float = 0.0
    camera: str = "inconnue"
    au_sol: bool | None = None
    cible: Cible | None = None
    image_t: float | None = None
    batterie_pct: float | None = None
    source: str = "inconnue"

    def valide(self, maintenant: float) -> bool:
        nombres = (self.t, self.hauteur_m, self.cap_rad, self.vitesse_z, self.vitesse_lacet)
        return (
            all(v is not None and math.isfinite(v) for v in nombres)
            and 0 <= maintenant - self.t <= 0.4
            and self.hauteur_m is not None
            and 0 <= self.hauteur_m <= 4
            and self.sequence >= 0
        )


@dataclass(frozen=True)
class Ordre:
    montee_m_s: float = 0.0
    lacet_rad_s: float = 0.0
    avant_m_s: float = 0.0
    droite_m_s: float = 0.0
    action: str | None = None
    photo: int | None = None

    def __post_init__(self) -> None:
        for valeur, limite in (
            (self.montee_m_s, 0.3),
            (self.lacet_rad_s, 0.8),
            (self.avant_m_s, 0.15),
            (self.droite_m_s, 0.15),
        ):
            if not math.isfinite(valeur) or abs(valeur) > limite + 1e-9:
                raise ValueError("consigne drone hors des limites physiques du banc")
        if self.action not in (None, "decoller", "atterrir", "camera_bas", "stop"):
            raise ValueError("action drone inconnue")


def borner(v: float, maximum: float) -> float:
    return max(-maximum, min(maximum, v))


def angle(v: float) -> float:
    return (v + math.pi) % (2 * math.pi) - math.pi
