"""Règle d'architecture : le cerveau ne dépend jamais du simulateur.

Seul `services/api` (racine de composition de la démo) branche le simulateur au cerveau. Le même
cerveau doit pouvoir piloter le rover réel à travers un autre pont, sans modification.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

RACINE = Path(__file__).resolve().parents[1]
PAQUETS_DU_CERVEAU = [
    "services/cerveau",
    "services/navigation",
    "services/mission_planner",
    "services/protocole",
    "services/perception",
]


def _imports(fichier: Path) -> set[str]:
    arbre = ast.parse(fichier.read_text(encoding="utf-8"))
    modules: set[str] = set()
    for noeud in ast.walk(arbre):
        if isinstance(noeud, ast.Import):
            modules.update(alias.name for alias in noeud.names)
        elif isinstance(noeud, ast.ImportFrom) and noeud.module:
            modules.add(noeud.module)
    return modules


@pytest.mark.parametrize("paquet", PAQUETS_DU_CERVEAU)
def test_le_cerveau_n_importe_pas_le_simulateur(paquet: str) -> None:
    for fichier in (RACINE / paquet).rglob("*.py"):
        interdits = {
            m
            for m in _imports(fichier)
            if m.split(".")[0] in {"simulation", "mujoco"} or m.startswith("services.api")
        }
        assert not interdits, f"{fichier.relative_to(RACINE)} importe {sorted(interdits)}"
