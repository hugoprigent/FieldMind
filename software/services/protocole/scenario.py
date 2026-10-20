"""Scénario de mission : partie publique, connue du cerveau.

Le cerveau connaît les limites de la zone, la pose de départ et les lieux nommés, comme un
opérateur qui les marquerait sur une carte. Il ne connaît jamais les arbres ni les rochers : la
section « monde » du fichier est réservée au simulateur et n'est pas lue ici.
"""

from __future__ import annotations

import json
import unicodedata
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, model_validator


class _Modele(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class Limites(_Modele):
    xmin: float
    xmax: float
    ymin: float
    ymax: float

    @model_validator(mode="after")
    def _ordonnees(self) -> Limites:
        if self.xmin >= self.xmax or self.ymin >= self.ymax:
            raise ValueError("limites vides")
        return self

    def contient(self, x: float, y: float, marge: float = 0.0) -> bool:
        return (
            self.xmin + marge <= x <= self.xmax - marge
            and self.ymin + marge <= y <= self.ymax - marge
        )


class PoseDepart(_Modele):
    x: float
    y: float
    cap: float


class Lieu(_Modele):
    id: str = Field(pattern=r"^[a-z0-9_]+$")
    nom: str
    alias: list[str] = Field(default_factory=list)
    x: float
    y: float


def normaliser(texte: str) -> str:
    """Minuscules sans accents ni espaces superflus, pour comparer des noms de lieux."""
    sans_accents = unicodedata.normalize("NFKD", texte).encode("ascii", "ignore").decode("ascii")
    return " ".join(sans_accents.lower().replace("'", " ").replace("’", " ").split())


class ScenarioPublic(_Modele):
    nom: str
    graine: int
    limites: Limites
    depart: PoseDepart
    lieux: list[Lieu]

    @model_validator(mode="after")
    def _lieux_valides(self) -> ScenarioPublic:
        ids = [lieu.id for lieu in self.lieux]
        if len(ids) != len(set(ids)):
            raise ValueError("identifiants de lieux en double")
        for lieu in self.lieux:
            if not self.limites.contient(lieu.x, lieu.y):
                raise ValueError(f"lieu hors limites : {lieu.id}")
        return self

    def lieu(self, identifiant: str) -> Lieu | None:
        return next((lieu for lieu in self.lieux if lieu.id == identifiant), None)

    def chercher_lieu(self, nom: str) -> Lieu | None:
        """Retrouve un lieu par son identifiant, son nom ou un alias (casse et accents ignorés)."""
        cle = normaliser(nom)
        for lieu in self.lieux:
            noms = [lieu.id.replace("_", " "), lieu.nom, *lieu.alias]
            if any(normaliser(n) == cle for n in noms):
                return lieu
        return None

    @classmethod
    def charger(cls, chemin: Path) -> ScenarioPublic:
        """Lit le fichier de scénario en ignorant la section « monde » du simulateur."""
        donnees = json.loads(chemin.read_text(encoding="utf-8"))
        donnees.pop("monde", None)
        return cls.model_validate(donnees)
