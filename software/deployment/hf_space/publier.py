"""Publie la démo sur un Space Hugging Face (Docker, processeur gratuit) : une adresse publique
pour essayer la démo sans serveur à soi.

Lancé par le workflow « Publier sur Hugging Face » (.github/workflows/publier-hf-space.yml),
ou à la main depuis software/ :
    HF_TOKEN=... HF_SPACE=utilisateur/fieldmind python deployment/hf_space/publier.py

La clé OpenRouter (facultative) devient un secret du Space : elle n'est jamais écrite dans les
fichiers publiés.
"""

from __future__ import annotations

import os
import shutil
import sys
import tempfile
from pathlib import Path

from huggingface_hub import HfApi

RACINE = Path(__file__).resolve().parents[2]
CONTENU = [
    "Dockerfile",
    ".dockerignore",
    "requirements-demo.txt",
    "config",
    "services",
    "simulation",
    "interface",
]
ENTETE = """---
title: FieldMind
emoji: 🌲
colorFrom: green
colorTo: gray
sdk: docker
app_port: 8000
pinned: false
short_description: Rover autonome en forêt, simulation physique, lidar et vision
---

# FieldMind — démo de navigation autonome

Cerveau de navigation d'un rover (localisation, carte au lidar, calcul du chemin, vision par un
modèle gratuit d'OpenRouter), piloté en simulation physique MuJoCo. Code source :
https://github.com/hugoprigent/FieldMind (dossier `software/`).
"""
REGLAGES = {
    # 2 processeurs gratuits : peu de sessions simultanées (physique + rendu de la caméra).
    "FIELDMIND_SESSIONS_MAX": "3",
    "FIELDMIND_VISION_MAX_PAR_JOUR": "900",
}


def main() -> int:
    jeton, space = os.environ.get("HF_TOKEN", ""), os.environ.get("HF_SPACE", "")
    if not jeton or "/" not in space:
        print("HF_TOKEN et HF_SPACE (utilisateur/nom) sont requis.", file=sys.stderr)
        return 2
    api = HfApi(token=jeton)
    api.create_repo(space, repo_type="space", space_sdk="docker", exist_ok=True)
    for cle, valeur in REGLAGES.items():
        api.add_space_variable(space, cle, valeur)
    cle_openrouter = os.environ.get("OPENROUTER_API_KEY", "")
    if cle_openrouter:
        api.add_space_secret(
            space, "OPENROUTER_API_KEY", cle_openrouter, description="Modèles gratuits seulement"
        )
    with tempfile.TemporaryDirectory() as dossier:
        cible = Path(dossier)
        for nom in CONTENU:
            source = RACINE / nom
            if source.is_dir():
                shutil.copytree(
                    source, cible / nom, ignore=shutil.ignore_patterns("__pycache__", "*.pyc")
                )
            else:
                shutil.copy2(source, cible / nom)
        (cible / "README.md").write_text(ENTETE, encoding="utf-8")
        api.upload_folder(
            repo_id=space,
            repo_type="space",
            folder_path=cible,
            commit_message="Publication de la démo FieldMind",
            delete_patterns=["*"],
        )
    print(f"Publié : https://huggingface.co/spaces/{space}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
