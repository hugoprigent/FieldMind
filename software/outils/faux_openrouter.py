"""Faux serveur OpenRouter, pour essayer la chaîne complète (chat et vision) sans clé ni réseau.

Il sert le catalogue des modèles (/api/v1/models) et /api/v1/chat/completions, et vérifie que
chaque requête respecte les règles du projet : clé présente, prix plafonné à zéro, modèle
gratuit, image jointe pour la vision. Les réponses sont fixes et marquées comme factices.

    python -m outils.faux_openrouter --port 8100
    OPENROUTER_API_KEY=factice FIELDMIND_OPENROUTER_URL=http://127.0.0.1:8100/api/v1 \\
        uvicorn services.api.serveur:app
"""

from __future__ import annotations

import argparse
import json
from typing import Any

import uvicorn
from fastapi import FastAPI, HTTPException, Request

MODELES = {
    "stealth/space-bunny-alpha": {"structured": False},
    "google/gemma-4-31b-it:free": {"structured": False},
    "qwen/qwen3.8-27b:free": {"structured": True},
}

app = FastAPI()
requetes: list[dict[str, Any]] = []


@app.get("/api/v1/models")
async def modeles() -> dict[str, Any]:
    return {
        "data": [
            {
                "id": nom,
                "pricing": {"prompt": "0", "completion": "0"},
                "architecture": {"input_modalities": ["text", "image"]},
                "supported_parameters": ["response_format", "reasoning"]
                + (["structured_outputs"] if info["structured"] else []),
            }
            for nom, info in MODELES.items()
        ]
        + [
            {
                "id": "exemple/modele-payant",
                "pricing": {"prompt": "0.000001", "completion": "0.000002"},
                "architecture": {"input_modalities": ["text", "image"]},
                "supported_parameters": ["response_format"],
            }
        ]
    }


@app.post("/api/v1/chat/completions")
async def completions(requete: Request) -> dict[str, Any]:
    if not requete.headers.get("authorization", "").startswith("Bearer "):
        raise HTTPException(401, "clé absente")
    corps = await requete.json()
    requetes.append(corps)
    prix = corps.get("provider", {}).get("max_price", {})
    if prix.get("prompt") != 0 or prix.get("completion") != 0:
        raise HTTPException(402, "prix non plafonné à zéro")
    if corps["model"] not in MODELES:
        raise HTTPException(400, "modèle inconnu ou payant")
    contenu_utilisateur = corps["messages"][-1]["content"]
    if isinstance(contenu_utilisateur, list):
        images = [c for c in contenu_utilisateur if c.get("type") == "image_url"]
        if not images or not images[0]["image_url"]["url"].startswith("data:image/jpeg;base64,"):
            raise HTTPException(400, "image absente")
        reponse: dict[str, Any] = {
            "resume": "(réponse factice) Piste en terre bordée de conifères ; un tronc couché "
            "barre le chemin.",
            "objets": [
                {
                    "categorie": "tronc_couche",
                    "boite": [243, 142, 410, 149],
                    "bloque_passage": True,
                    "confiance": 0.8,
                },
                {
                    "categorie": "arbre",
                    "boite": [26, 0, 90, 151],
                    "bloque_passage": True,
                    "confiance": 0.9,
                },
                {
                    "categorie": "inventee",
                    "boite": [0, 0, 1, 1],
                    "bloque_passage": False,
                    "confiance": 1,
                },
            ],
        }
        texte = "```json\n" + json.dumps(reponse, ensure_ascii=False) + "\n```"
    else:
        consigne = str(contenu_utilisateur).lower()
        action = "observer" if "vois" in consigne else "aller_a"
        texte = json.dumps(
            {
                "action": action,
                "lieu": "clairiere" if action == "aller_a" else None,
                "x": None,
                "y": None,
                "distance_m": None,
                "angle_deg": None,
            }
        )
    return {
        "model": corps["model"],
        "choices": [{"message": {"role": "assistant", "content": texte}, "finish_reason": "stop"}],
    }


def main() -> None:
    parseur = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parseur.add_argument("--port", type=int, default=8100)
    args = parseur.parse_args()
    uvicorn.run(app, host="127.0.0.1", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
