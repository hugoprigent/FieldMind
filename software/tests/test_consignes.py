"""Tests des consignes : validation des intentions, parcours, fournisseur Claude simulé, échecs."""

from __future__ import annotations

import asyncio
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from fastapi.testclient import TestClient

from services.api.serveur import app
from services.mission_planner.consignes import (
    EchecFournisseur,
    FournisseurAnthropic,
    InterpreteConsignes,
    QuotaJournalier,
)
from services.mission_planner.intentions import (
    AllerA,
    Arreter,
    Intention,
    Parcours,
    PoseCourante,
    Refus,
    Reprendre,
    traduire,
)
from services.protocole.scenario import ScenarioPublic

RACINE = Path(__file__).resolve().parents[1]
SCENARIO = ScenarioPublic.charger(RACINE / "simulation" / "scenarios" / "foret_demo.json")
POSE = PoseCourante(x=-29.0, y=-29.2, cap=0.0)


# ------------------------------------------------------------------ validation par le code


def test_traduction_lieu_et_texte_compose_par_le_code() -> None:
    action, texte = traduire(Intention(action="aller_a", lieu="clairiere"), SCENARIO, POSE)
    assert isinstance(action, AllerA)
    assert (action.x, action.y) == (19.5, 26.0)
    assert texte.startswith("Je vais à la clairière")


def test_deplacement_relatif_depuis_la_pose() -> None:
    pose = PoseCourante(x=0.0, y=0.0, cap=math.pi / 2)  # face au nord
    action, _ = traduire(
        Intention(action="deplacement_relatif", distance_m=5, angle_deg=90), SCENARIO, pose
    )
    assert isinstance(action, AllerA)
    assert action.x == pytest.approx(-5.0)  # à gauche en regardant le nord : l'ouest
    assert action.y == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize(
    "intention",
    [
        Intention(action="aller_a", lieu="lune"),
        Intention(action="aller_a", x=500.0, y=0.0),
        Intention(action="aller_a"),
        Intention(action="deplacement_relatif", distance_m=250.0, angle_deg=0.0),
        Intention(action="deplacement_relatif", distance_m=-3.0, angle_deg=0.0),
        Intention(action="inconnu"),
    ],
)
def test_intentions_invalides_refusees(intention: Intention) -> None:
    action, texte = traduire(intention, SCENARIO, POSE)
    assert isinstance(action, Refus)
    assert "refusée" in texte or "Je ne sais pas faire" in texte


def test_parcours_et_tour_de_la_carte() -> None:
    action, texte = traduire(
        Intention(action="parcours", lieux=["clairiere", "grand_rocher"]), SCENARIO, POSE
    )
    assert isinstance(action, Parcours)
    assert [e.libelle for e in action.etapes] == ["la clairière", "le grand rocher"]
    assert "puis" in texte
    action, _ = traduire(Intention(action="faire_le_tour"), SCENARIO, POSE)
    assert isinstance(action, Parcours)
    noms = [e.libelle for e in action.etapes]
    assert noms[-1] == "le départ"
    assert {lieu.nom for lieu in SCENARIO.lieux if lieu.id != "depart"} <= set(noms)
    refus, _ = traduire(Intention(action="parcours", lieux=["clairiere"]), SCENARIO, POSE)
    assert isinstance(refus, Refus)


def test_arret_et_reprise() -> None:
    assert isinstance(traduire(Intention(action="s_arreter"), SCENARIO, POSE)[0], Arreter)
    assert isinstance(traduire(Intention(action="reprendre"), SCENARIO, POSE)[0], Reprendre)


# ------------------------------------------------------------------ fournisseur Claude (simulé)


class _FauxMessages:
    def __init__(self, reponse: Any) -> None:
        self.reponse = reponse
        self.appels: list[dict[str, Any]] = []

    async def create(self, **kwargs: Any) -> Any:
        self.appels.append(kwargs)
        return self.reponse


def _fournisseur(reponse: Any) -> tuple[FournisseurAnthropic, _FauxMessages]:
    fournisseur = FournisseurAnthropic(SCENARIO, "claude-opus-5-5", "cle-de-test", 10.0, 1024)
    faux = _FauxMessages(reponse)
    fournisseur._client = SimpleNamespace(beta=SimpleNamespace(messages=faux))  # type: ignore[assignment]
    return fournisseur, faux


def _reponse(texte: str, stop: str = "end_turn") -> Any:
    return SimpleNamespace(stop_reason=stop, content=[SimpleNamespace(type="text", text=texte)])


def test_fournisseur_anthropic_requete_et_reponse() -> None:
    json_intention = json.dumps(
        {"action": "aller_a", "lieu": "clairiere", "x": None, "y": None,
         "distance_m": None, "angle_deg": None}
    )  # fmt: skip
    fournisseur, faux = _fournisseur(_reponse(json_intention))
    intention = asyncio.run(fournisseur.interpreter("file à la clairière"))
    assert intention == Intention(action="aller_a", lieu="clairiere")
    appel = faux.appels[0]
    assert appel["model"] == "claude-opus-5-5"
    assert appel["max_tokens"] == 1024
    assert appel["fallbacks"] == "default"
    assert appel["betas"] == ["server-side-fallback-2026-07-01"]
    assert appel["output_config"]["effort"] == "low"
    schema = appel["output_config"]["format"]["schema"]
    assert schema["additionalProperties"] is False
    assert "clairiere" in json.dumps(schema)
    assert "19.5" not in appel["system"], "le modèle ne reçoit jamais les coordonnées des lieux"


@pytest.mark.parametrize(
    ("reponse", "message"),
    [
        (_reponse("", stop="refusal"), "refusée"),
        (_reponse('{"action": "aller_a"', stop="max_tokens"), "tronquée"),
        (_reponse('{"action": "decoller"}'), "schéma"),
    ],
)
def test_fournisseur_anthropic_echecs(reponse: Any, message: str) -> None:
    fournisseur, _ = _fournisseur(reponse)
    with pytest.raises(EchecFournisseur, match=message):
        asyncio.run(fournisseur.interpreter("…"))


def test_sans_reponse_du_modele_la_consigne_n_est_pas_executee() -> None:
    fournisseur, _ = _fournisseur(_reponse("", stop="refusal"))
    interprete = InterpreteConsignes(SCENARIO, fournisseur, QuotaJournalier(10))
    resultat = asyncio.run(interprete.interpreter("va à la clairière"))
    assert resultat.intention is None
    assert resultat.echec is not None


def test_quota_journalier() -> None:
    fournisseur, faux = _fournisseur(_reponse("{}"))
    interprete = InterpreteConsignes(SCENARIO, fournisseur, QuotaJournalier(0))
    resultat = asyncio.run(interprete.interpreter("arrête-toi"))
    assert not faux.appels
    assert resultat.intention is None
    assert resultat.echec is not None
    assert "quota" in resultat.echec


def test_sans_modele_rien_n_est_interprete() -> None:
    interprete = InterpreteConsignes(SCENARIO, None, QuotaJournalier(10))
    resultat = asyncio.run(interprete.interpreter("va à la clairière"))
    assert resultat.intention is None


# ------------------------------------------------------------------ de bout en bout


def test_chat_par_websocket_lance_la_mission() -> None:
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        ws.receive_text()
        ws.send_text(json.dumps({"type": "chat", "texte": "va à la clairière"}))
        reponse = statut = None
        fin = time.monotonic() + 5
        while time.monotonic() < fin and (reponse is None or statut is None):
            message = json.loads(ws.receive_text())
            if message["type"] == "chat_reponse":
                reponse = message
            elif message["type"] == "statut" and message["etat"] in ("calcul", "route"):
                statut = message
        assert reponse is not None
        assert reponse["refus"] is None
        assert "clairière" in reponse["texte"]
        assert statut is not None


def test_trajet_relatif_au_rover() -> None:
    pose = PoseCourante(x=0.0, y=0.0, cap=math.pi / 2)  # face au nord
    carre = [[0, 0], [5, 0], [5, 5], [0, 5], [0, 0]]
    action, texte = traduire(Intention(action="trajet", points=carre), SCENARIO, pose)
    assert isinstance(action, Parcours)
    assert len(action.etapes) == 5
    assert action.etapes[1].x == pytest.approx(0.0, abs=1e-9)
    assert action.etapes[1].y == pytest.approx(5.0)  # 5 m devant = 5 m au nord
    assert action.etapes[2].x == pytest.approx(-5.0)  # puis 5 m à gauche = à l'ouest
    assert "20 m" in texte
    loin = traduire(Intention(action="trajet", points=[[500, 0], [600, 0]]), SCENARIO, pose)[0]
    assert isinstance(loin, Refus)
