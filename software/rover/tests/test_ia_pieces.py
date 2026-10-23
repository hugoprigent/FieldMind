"""Compréhension des lieux : preuves réelles, résultats périmés et cache GX10."""

from __future__ import annotations

import asyncio
import json
import time
from types import SimpleNamespace
from typing import Any

import pytest
from test_ia_faux_serveur import config_essai, faux_serveur

from fieldmind.bus import Bus
from fieldmind.estimation.semantique import MemoireSemantique
from fieldmind.ia.integration import Ia, _Outils
from fieldmind.ia.pieces import IdentificationPieces, valider_propositions
from fieldmind.types import Pose2D

OBJETS = [{"id": 1, "label": "four", "x": 2.0, "y": 1.0, "age_s": 1.0, "vues": 1},
          {"id": 2, "label": "micro-ondes", "x": 2.2, "y": 1.0, "age_s": 1.0, "vues": 1}]
PIECE = {"nom": "cuisine", "objets": [1, 2], "raison": "four et micro-ondes observés",
         "confiance": "forte"}


class Contexte:
    def __init__(self) -> None:
        self.monde = {"repere": "odom", "generation": 0, "objets": [dict(o) for o in OBJETS],
                      "pieces": []}

    def etat_resume(self) -> dict[str, Any]:
        return {"monde": self.monde}


class Gx10:
    def __init__(self) -> None:
        self.actif = object()
        self.reponse: Any = {"pieces": [PIECE]}
        self.demarre, self.fin = asyncio.Event(), asyncio.Event()
        self.fin.set()
        self.appels = 0

    def disponibilite(self, _role: str) -> tuple[bool, str]:
        return True, ""

    def passerelle(self) -> Gx10:
        return self

    async def chat(self, _role: str, _messages: Any) -> Any:
        self.appels += 1
        self.demarre.set()
        await self.fin.wait()
        return SimpleNamespace(texte=json.dumps(self.reponse))


async def test_vrai_client_gx10_identifie_uniquement_les_objets_fournis() -> None:
    async with faux_serveur() as serveur:
        serveur.chat = lambda _corps: {"content": json.dumps({"pieces": [PIECE]})}
        config = config_essai(serveur.base, env={}, profil="gx10")
        try:
            resultat = await IdentificationPieces(config, Contexte()).identifier("cuisine")
            assert resultat == [PIECE]
            prompt = serveur.requetes[0][1]["messages"][0]["content"]
            assert "micro-ondes" in prompt and "aucune coordonnée" in prompt
        finally:
            await config.passerelle().fermer()


@pytest.mark.parametrize("ajouts", [
    {"objets": [1, 99]}, {"objets": [True, 2]}, {"objets": [1.0, 2]},
    {"objets": ["1", 2]}, {"objets": [1, 1]}, {"objets": [1]},
    {"x": 4, "y": 8}, {"confiance": "certaine"}, {"nom": "inconnue"},
])
def test_proposition_inventee_ou_malformee_refusee(ajouts: dict[str, Any]) -> None:
    assert valider_propositions([PIECE | ajouts], OBJETS) == []


def test_table_et_chaise_ne_prouvent_pas_une_cuisine() -> None:
    generiques = [OBJETS[0] | {"label": "table"}, OBJETS[1] | {"label": "chaise"}]
    assert valider_propositions([PIECE], generiques)[0]["confiance"] == "faible"


def test_lieu_general_pas_limite_a_un_catalogue_de_pieces() -> None:
    preuves = [OBJETS[0] | {"label": "arbre"}, OBJETS[1] | {"label": "arbuste"}]
    assert valider_propositions([PIECE | {"nom": "jardin"}], preuves)[0] == PIECE | {
        "nom": "jardin"}


@pytest.mark.parametrize("ajouts", [{"x": 10}, {"mobile": True}, {"age_s": 3601},
                                     {"x": float("nan")}, {"age_s": -1}])
async def test_objets_eloignes_mobiles_ou_perimes_ne_confirment_pas(ajouts: dict[str, Any]) -> None:
    outils, config = Contexte(), Gx10()
    outils.monde["objets"][1].update(ajouts)
    assert await IdentificationPieces(config, outils).identifier() == []


async def test_cache_ignore_age_et_vues_mais_pas_nouvelle_classe() -> None:
    outils, config = Contexte(), Gx10()
    cerveau = IdentificationPieces(config, outils)
    assert await cerveau.identifier() == [PIECE]
    outils.monde["objets"][0].update(age_s=10, vues=8, x=2.1)
    assert await cerveau.identifier() == [PIECE]
    assert config.appels == 1
    outils.monde["objets"][0]["label"] = "four électrique"
    assert await cerveau.identifier() == [PIECE]
    assert config.appels == 2


@pytest.mark.parametrize("changement", ["generation", "repere", "profil", "id", "classe"])
async def test_resultat_apres_reset_ou_changement_objet_rejete(changement: str) -> None:
    outils, config = Contexte(), Gx10()
    config.fin.clear()
    cerveau = IdentificationPieces(config, outils)
    tache = asyncio.create_task(cerveau.identifier())
    await config.demarre.wait()
    assert await cerveau.identifier() == []  # aucune file d'attente
    assert config.appels == 1
    if changement == "generation":
        outils.monde["generation"] += 1
    elif changement == "repere":
        outils.monde["repere"] = "nouveau"
    elif changement == "profil":
        config.actif = object()
    elif changement == "id":
        outils.monde["objets"].pop()
    else:
        outils.monde["objets"][1]["label"] = "chaise"
    config.fin.set()
    assert await tache == []


async def test_observer_mission_retourne_piece_memorisee_et_sans_moteurs() -> None:
    bus, config = Bus(), Gx10()
    memoire = MemoireSemantique(bus)
    maintenant = time.monotonic()
    observation = {"pose_image": {"x": 0.0, "y": 0.0, "cap": 0.0},
                   "points": [{"label": "four", "sol": [2.0, 1.0]},
                              {"label": "micro-ondes", "sol": [2.2, 1.0]}]}
    memoire.integrer(observation | {"image_t": maintenant - 1.0})
    memoire.integrer(observation | {"image_t": maintenant})

    class Outils:
        def etat_resume(self) -> dict[str, Any]:
            return {"monde": memoire.resume()}

        def memoriser_pieces(self, propositions: Any, *, generation: int | None = None) -> Any:
            return memoire.integrer_pieces(propositions, generation=generation)

        def pose_odom(self) -> Pose2D:
            return Pose2D(0.0, 0.0, 0.0)

    class Vision:
        async def analyser(self, _camera: str, silencieux: bool) -> dict[str, Any]:
            return {"description": "Four et micro-ondes"}

    ia = Ia.__new__(Ia)
    ia.outils = _Outils(Outils())
    ia.pieces = IdentificationPieces(config, ia.outils)
    ia.vision_ia = Vision()
    resultat = await ia.pour_mission("observer", {"piece": "cuisine"})
    assert resultat["piece"]["statut"] == "confirmee"
    assert resultat["piece"]["nom"] == "cuisine"
    assert await ia.pour_mission("identifier_piece", {"nom": "la cuisine"}) == resultat["piece"]
    config.reponse = {"pieces": []}
    assert await ia.pour_mission("identifier_piece", {"nom": "chambre"}) is None


async def test_objets_generiques_restent_une_hypothese_faible() -> None:
    outils, config = Contexte(), Gx10()
    outils.monde["objets"][0]["label"] = "table"
    outils.monde["objets"][1]["label"] = "chaise"
    resultat = await IdentificationPieces(config, outils).identifier()
    assert resultat and resultat[0]["confiance"] == "faible"
    assert config.appels == 1


def test_plusieurs_pieces_confirmees_meme_nom_restent_ambigues() -> None:
    outils = Contexte()
    outils.monde["pieces"] = [{"id": 10, "nom": "cuisine", "statut": "confirmee"},
                              {"id": 20, "nom": "cuisine", "statut": "confirmee"}]
    ia = Ia.__new__(Ia)
    ia.outils = _Outils(outils)
    assert ia._piece_confirmee("la cuisine") is None


async def test_observer_cible_ne_cumule_pas_analyse_globale_et_classification() -> None:
    appels = []

    class Vision:
        async def analyser(self, *_args: Any, **_kwargs: Any) -> None:
            raise AssertionError("une recherche ne doit pas lancer l'analyse globale")

        async def chercher(self, objet: str, camera: str) -> list[dict[str, Any]]:
            appels.append((objet, camera))
            return [{"label": "caisse rouge", "sol": [1.0, 0.2]}]

    ia = Ia.__new__(Ia)
    ia.vision_ia = Vision()
    resultat = await ia.pour_mission("observer", {"objet": "caisse rouge", "camera": "droite"})
    assert resultat == {"points": [{"label": "caisse rouge", "sol": [1.0, 0.2]}]}
    assert appels == [("caisse rouge", "droite")]
