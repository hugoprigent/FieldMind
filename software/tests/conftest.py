"""Configuration commune des tests."""

import os
from collections.abc import Iterator

import pytest

import simulation  # noqa: F401 — choisit le rendu hors écran avant tout `import mujoco`

# Les tests n'appellent jamais de vrai modèle, même si une clé est définie sur la machine :
# consignes par un faux modèle (ci-dessous), vision par le détecteur parfait de la simulation.
os.environ["FIELDMIND_LLM_PROVIDER"] = "aucun"
os.environ["FIELDMIND_VISION_PROVIDER"] = "simulation"

from services.mission_planner.intentions import Intention

REPONSES_DU_FAUX_MODELE = {
    "va à la clairière": Intention(action="aller_a", lieu="clairiere"),
    "avance de 3 m": Intention(action="deplacement_relatif", distance_m=3.0, angle_deg=0.0),
    "que vois-tu ?": Intention(action="observer"),
}


class FauxModele:
    """Modèle de langage des tests : réponses fixées d'avance."""

    nom = "faux modèle (tests)"

    async def interpreter(self, texte: str, contexte: object = None) -> Intention:
        return REPONSES_DU_FAUX_MODELE.get(texte, Intention(action="inconnu"))


@pytest.fixture(autouse=True)
def faux_modele_de_langage() -> Iterator[None]:
    """Le serveur de démo interprète les consignes avec le faux modèle pendant chaque test."""
    from services.api.serveur import app

    consignes = app.state.consignes
    precedent = consignes.fournisseur
    consignes.fournisseur = FauxModele()
    yield
    consignes.fournisseur = precedent
