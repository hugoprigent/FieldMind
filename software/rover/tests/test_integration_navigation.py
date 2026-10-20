"""Bout en bout : « aller à » à travers la salle simulée (mission → sécurité → ESP → physique)."""

from __future__ import annotations

import asyncio
import math
from pathlib import Path

import pytest

from fieldmind.app import Rover
from fieldmind.config import Profil


@pytest.mark.lent
async def test_aller_a_traverse_la_salle_sans_collision(tmp_path: Path) -> None:
    profil = Profil.charger(reglages=tmp_path / "reglages.json")
    rover = Rover(profil, simulation=True, scenario="salle", graine=0)
    monde, physique = rover.materiel.monde, rover.materiel.physique
    depart = physique.pose
    but_monde = monde.but  # (x, y) dans le repère du monde
    # odom = repère posé sur le rover au démarrage : on y exprime le but
    bx, by = depart.vers_local(but_monde[0], but_monde[1])
    rover.superviseur.operateurs = 1
    rover.demarrer()
    collisions = 0
    try:
        await asyncio.sleep(2.0)
        assert rover.conduite.changer_mode("auto") is None
        assert rover.conduite.armer() is None
        rover.executeur.ajouter([{"type": "aller_a", "x": bx, "y": by, "repere": "odom"}])
        for _ in range(900):  # 90 s au plus
            await asyncio.sleep(0.1)
            collisions += int(physique.bloque)
            if rover.executeur.etat in ("termine", "echec"):
                break
        etat = rover.executeur.etat
        reste = math.hypot(physique.pose.x - but_monde[0], physique.pose.y - but_monde[1])
    finally:
        rover.arreter()
    assert etat == "termine", (etat, reste)
    assert reste < 0.45, reste
    assert collisions == 0
