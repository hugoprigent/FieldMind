"""Bout en bout avec l'IA : consigne écrite → modèle (faux serveur) → plan → mission → roues."""

from __future__ import annotations

import asyncio
import math
from pathlib import Path

from test_ia_faux_serveur import appels, config_essai, faux_serveur

from fieldmind.app import Rover
from fieldmind.config import Profil
from fieldmind.types import Pose2D


async def test_consigne_ecrite_fait_avancer_le_rover(tmp_path: Path) -> None:
    profil = Profil.charger(reglages=tmp_path / "reglages.json")
    rover = Rover(profil, simulation=True, scenario="salle", graine=0)
    rover.materiel.physique.pose = Pose2D(2.0, 3.5, 0.0)  # place devant
    rover.superviseur.operateurs = 1
    rover.demarrer()
    try:
        async with faux_serveur() as fs:
            fs.chat = lambda corps: appels(("avancer", {"distance_m": 1.0}))
            await rover.demarrer_ia(asyncio.get_running_loop(), config_essai(fs.base))
            await asyncio.sleep(1.5)  # lidar, odométrie
            assert rover.conduite.changer_mode("auto") is None
            assert rover.conduite.armer() is None
            depart = rover.materiel.verite()
            await rover.ia.consigne("avance d'un mètre", "texte")
            for _ in range(150):
                await asyncio.sleep(0.1)
                if rover.executeur.etat in ("termine", "echec"):
                    break
            assert rover.executeur.etat == "termine", rover.executeur.resume()
            fin = rover.materiel.verite()
            parcouru = math.hypot(fin["x"] - depart["x"], fin["y"] - depart["y"])
            assert abs(parcouru - 1.0) < 0.15, parcouru
            await rover.ia.fermer()
    finally:
        rover.arreter()
