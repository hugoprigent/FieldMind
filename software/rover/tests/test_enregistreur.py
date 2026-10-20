"""Boîte noire : enregistrement d'une courte session simulée puis relecture."""

from __future__ import annotations

import time
from pathlib import Path

from fieldmind.app import Rover
from fieldmind.config import Profil
from fieldmind.enregistreur import Enregistreur, csv, lire, resume
from fieldmind.types import Consigne, Pose2D


def test_enregistrer_et_relire(tmp_path: Path) -> None:
    profil = Profil.charger(reglages=tmp_path / "reglages.json")
    rover = Rover(profil, simulation=True, enregistrer=False)
    rover.materiel.physique.pose = Pose2D(2.0, 3.5, 0.0)
    rover.superviseur.operateurs = 1
    boite = Enregistreur(rover.bus, tmp_path / "enregistrements")
    boite.demarrer()
    rover.demarrer()
    try:
        time.sleep(1.0)
        rover.conduite.changer_mode("manuel")
        assert rover.conduite.armer() is None
        fin = time.monotonic() + 2.0
        while time.monotonic() < fin:
            rover.conduite.proposer(Consigne(t=time.monotonic(), vitesse=0.3, source="manuel"))
            time.sleep(0.1)
    finally:
        rover.arreter()
        boite.arreter()
        time.sleep(1.0)
    lignes = lire(boite.chemin)
    r = resume(lignes)
    assert r["sujets"]["scan"] > 15 and r["sujets"]["commande"] > 30
    assert r["distance_m"] > 0.2
    assert "non_arme" in r["apparitions_des_raisons"]
    assert csv(lignes).count("\n") > 15
