"""État d'une mission de navigation vers un objectif."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Literal

OrigineObjectif = Literal["carte", "chat", "reprise"]


@dataclass(slots=True)
class Mission:
    id: int
    objectif: tuple[float, float]
    origine: OrigineObjectif
    t_debut: float
    libelle: str = ""
    objectif_effectif: tuple[float, float] | None = None
    chemin_id: int = 0
    nb_calculs: int = 0
    echecs_calcul: int = 0
    recuperations: int = 0
    meilleure_restante: float = math.inf
    t_progres: float = 0.0
    t_arret_obstacle: float | None = None
    demi_tours_bloques: int = 0
    """Manœuvres de demi-tour arrêtées aussitôt par un obstacle (boucle à éviter)."""
    distance_depart: float = 0.0
    duree_calculs_ms: list[float] = field(default_factory=list)


@dataclass(slots=True)
class Recul:
    """Manœuvre de dégagement : recul en ligne droite sur une courte distance."""

    t_debut: float
    x0: float
    y0: float
    distance: float = 1.0
    vitesse: float = -0.35
    duree_max: float = 5.0
    braquage: float = 0.0
    volontaire: bool = False
    """Manœuvre de demi-tour prévue (et non dégagement après un blocage)."""
