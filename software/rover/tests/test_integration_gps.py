"""Bout en bout : but GPS dans le jardin simulé (orientation sur le GPS, puis navigation locale)."""

from __future__ import annotations

import asyncio
import math
from pathlib import Path

import pytest

from fieldmind.app import Rover
from fieldmind.config import Profil
from fieldmind.simulation.materiel_simule import METRES_PAR_DEGRE_LAT, REFERENCE_GPS


def latlon(x: float, y: float) -> tuple[float, float]:
    """Point du monde simulé en GPS, comme le faux récepteur (x vers l'est, y vers le nord)."""
    lon = REFERENCE_GPS[1] + x / (METRES_PAR_DEGRE_LAT * math.cos(math.radians(REFERENCE_GPS[0])))
    return REFERENCE_GPS[0] + y / METRES_PAR_DEGRE_LAT, lon


@pytest.mark.lent
async def test_but_gps_dans_le_jardin(tmp_path: Path) -> None:
    profil = Profil.charger(reglages=tmp_path / "reglages.json")
    rover = Rover(profil, simulation=True, scenario="jardin", graine=0)
    rover.gps.dossier = tmp_path
    physique = rover.materiel.physique
    cible = (12.0, 9.0)
    lat, lon = latlon(*cible)
    rover.superviseur.operateurs = 1
    rover.demarrer()
    collisions = 0
    try:
        await asyncio.sleep(2.0)
        assert rover.conduite.changer_mode("auto") is None
        assert rover.conduite.armer() is None
        rover.executeur.ajouter([{"type": "aller_gps", "lat": lat, "lon": lon}])
        for _ in range(1200):  # 120 s au plus
            await asyncio.sleep(0.1)
            collisions += int(physique.bloque)
            if rover.executeur.etat in ("termine", "echec"):
                break
        etat, mission = rover.executeur.etat, rover.executeur.resume()
        reste = math.hypot(physique.pose.x - cible[0], physique.pose.y - cible[1])
        alignement = rover.gps.alignement
    finally:
        rover.arreter()
    assert etat == "termine", (etat, mission, reste)
    assert alignement.ok
    assert reste < 4.0, reste  # tolérance 2,5 m + erreur du GPS simulé (~1,3 m)
    assert collisions == 0
