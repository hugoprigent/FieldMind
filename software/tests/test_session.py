"""Test d'intégration : un onglet se connecte, conduit le rover simulé et reçoit la télémétrie."""

from __future__ import annotations

import json
import time

from fastapi.testclient import TestClient

from services.api.serveur import app


def _recevoir_jusqu_a(ws, condition, delai_s: float = 5.0) -> dict:  # type: ignore[no-untyped-def]
    fin = time.monotonic() + delai_s
    while time.monotonic() < fin:
        message = json.loads(ws.receive_text())
        if condition(message):
            return message
    raise AssertionError("message attendu non reçu")


def test_session_teleoperation_et_telemetrie() -> None:
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        bienvenue = json.loads(ws.receive_text())
        assert bienvenue["type"] == "bienvenue"
        assert len(bienvenue["monde"]["arbres"]) == 400
        assert "physique" not in bienvenue["rover"], "les paramètres de simulation restent privés"

        types_recus: set[str] = set()
        fin = time.monotonic() + 1.5
        while time.monotonic() < fin:
            types_recus.add(json.loads(ws.receive_text())["type"])
        assert {"vue", "imu", "odometrie", "scan", "firmware", "commande"} <= types_recus

        for _ in range(10):
            ws.send_text(json.dumps({"type": "teleop", "avance": 1.0, "direction": 0.0}))
        commande = _recevoir_jusqu_a(ws, lambda m: m["type"] == "commande" and m["vitesse"] > 1.0)
        assert commande["braquage"] == 0.0


def test_message_invalide_refuse_sans_couper_la_session() -> None:
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        ws.receive_text()
        ws.send_text(json.dumps({"type": "objectif", "x": "nord"}))
        erreur = _recevoir_jusqu_a(ws, lambda m: m["type"] == "erreur")
        assert "invalide" in erreur["texte"]
        ws.send_text(json.dumps({"type": "stop"}))
        _recevoir_jusqu_a(ws, lambda m: m["type"] == "statut")
