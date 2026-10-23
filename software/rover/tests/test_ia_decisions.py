"""Le GX10 choisit parmi les candidats ; une décision périmée ne relance pas une mission."""

from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace
from typing import Any

import pytest

from fieldmind.ia.decisions import ChoixExploration


class FauxContexte:
    def __init__(self) -> None:
        self.generation = 4
        self.urgence = False

    def etat_resume(self) -> dict[str, Any]:
        return {"mode": "auto", "arme": True, "arret_urgence": self.urgence,
                "mission": {"generation": self.generation, "plan_id": "plan-1",
                            "etat": "en_cours"},
                "monde": {"objets": [{"nom": "porte"}]}}


class FauxGx10:
    def __init__(self, reponse: Any = None) -> None:
        self.actif = object()
        self.reponse = reponse if reponse is not None else {"index": 1, "raison": "la porte"}
        self.recu: list[dict[str, Any]] = []
        self.demarre = asyncio.Event()
        self.fin = asyncio.Event()
        self.fin.set()

    def disponibilite(self, _role: str) -> tuple[bool, str]:
        return True, ""

    def passerelle(self) -> FauxGx10:
        return self

    async def chat(self, role: str, messages: list[dict[str, Any]]) -> Any:
        assert role == "agent"
        self.recu = messages
        self.demarre.set()
        await self.fin.wait()
        return SimpleNamespace(texte=json.dumps(self.reponse))


ARGUMENT = {"candidats": [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}],
            "objectif": "explorer la pièce"}


async def test_choix_gx10_recoit_monde_et_candidats() -> None:
    config = FauxGx10()
    decision = await ChoixExploration(config, FauxContexte()).choisir(ARGUMENT)
    assert decision is not None and decision["index"] == 1
    assert decision["generation_executeur"] == 4
    texte = config.recu[0]["content"]
    assert "porte" in texte and "candidats" in texte and "plan-1" in texte


@pytest.mark.parametrize("reponse", [{"index": True}, {"index": -1}, {"index": 2},
                                     {"index": 0.0}, {"index": None}, [], {"x": 2, "y": 3}])
async def test_choix_invente_ou_malforme_refuse(reponse: Any) -> None:
    assert await ChoixExploration(FauxGx10(reponse), FauxContexte()).choisir(ARGUMENT) is None


@pytest.mark.parametrize("changement", ["annulation", "urgence", "profil"])
async def test_changement_pendant_inference_invalide_le_choix(changement: str) -> None:
    config, outils = FauxGx10(), FauxContexte()
    config.fin.clear()
    cerveau = ChoixExploration(config, outils)
    tache = asyncio.create_task(cerveau.choisir(ARGUMENT))
    await config.demarre.wait()
    if changement == "annulation":
        outils.generation += 1
    elif changement == "urgence":
        outils.urgence = True
    else:
        config.actif = object()
    # Le deuxième appel ne s'accumule pas derrière le premier.
    assert await cerveau.choisir(ARGUMENT) is None
    config.fin.set()
    assert await tache is None


async def test_aucune_requete_si_candidats_invalides_ou_urgence() -> None:
    config, outils = FauxGx10(), FauxContexte()
    cerveau = ChoixExploration(config, outils)
    assert await cerveau.choisir({"candidats": [{"x": float("nan"), "y": 0}]}) is None
    outils.urgence = True
    assert await cerveau.choisir(ARGUMENT) is None
    assert not config.recu
