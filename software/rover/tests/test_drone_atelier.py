"""Le poste peut piloter un drone simulé sans ouvrir le pilote physique."""

from __future__ import annotations

import json
from types import SimpleNamespace

import pytest

from fieldmind.api.serveur import Serveur
from fieldmind.bus import Bus, S
from fieldmind.drone.atelier import AtelierDrone
from fieldmind.drone.simulation import DroneSimule


def test_archivage_et_sources_separees(tmp_path):
    bus = Bus()
    sentinelle = object()
    bus.publier(S.DRONE, sentinelle)
    a = AtelierDrone(bus, tmp_path)
    a.drone, a.mode = DroneSimule(), "mission"
    a._ordre = a.pilote.demarrer(a.drone.observer(), 0)
    for _ in range(2000):
        a._pas()
        if a._termine_archive:
            break
    etat = bus.dernier(S.DRONE_ATELIER)
    assert etat["phase"] == "termine" and etat["source"] == "simulation"
    assert bus.dernier(S.DRONE) is sentinelle
    assert bus.dernier(S.IMAGE_DRONE) is None
    assert bus.dernier(S.IMAGE_DRONE_SIMULEE) is not None
    rapport = json.loads(a.fichier("rapport.json").read_text())
    assert rapport["source"] == "simulation" and rapport["photos"] == 6
    assert a.fichier("montage.jpg").is_file()
    assert a.fichier("../../rover.toml") is None


def test_absence_homme_mort_manuel_declenche_un_seul_land(tmp_path, monkeypatch):
    a = AtelierDrone(Bus(), tmp_path)
    a.drone, a.mode = DroneSimule(), "manuel"
    a.drone.au_sol, a.drone.h = False, 0.7
    a._hauteur_manuelle = 0.7
    a._dernier_poste = 1
    monkeypatch.setattr("fieldmind.drone.atelier.time.monotonic", lambda: 2)
    for _ in range(300):
        a._pas()
        if a._termine_archive:
            break
    assert a.drone.au_sol
    assert [x[1] for x in a.drone.actions] == ["atterrir"]


async def test_demande_atelier_ne_consulte_pas_esp(tmp_path):
    def interdit():
        raise AssertionError("pilote réel consulté depuis la simulation")

    rover = SimpleNamespace(
        bus=Bus(), conduite=None, profil=None, materiel=SimpleNamespace(esp=interdit), ia=None
    )
    serveur = Serveur(rover)
    serveur.atelier_drone.dossier = tmp_path
    try:
        await serveur._traiter(None, {"type": "drone_atelier", "action": "manuel"})
        assert serveur.atelier_drone.mode == "manuel"
        await serveur._traiter(None, {"type": "drone_atelier", "action": "axes", "montee": 1})
        assert serveur.atelier_drone._manuel.montee_m_s > 0
        with pytest.raises(Exception, match="axes du banc invalides"):
            await serveur._traiter(None, {"type": "drone_atelier", "action": "axes", "montee": 3})
    finally:
        serveur.atelier_drone.arreter()


def test_mission_en_cours_refuse_axes_et_redemarrage(tmp_path):
    a = AtelierDrone(Bus(), tmp_path)
    a.drone, a.mode = DroneSimule(), "mission"
    a.drone.au_sol = False
    with pytest.raises(ValueError, match="pendant la mission"):
        a.commander("axes", {"montee": 1})
    with pytest.raises(ValueError, match="terminer ou interrompre"):
        a.demarrer("manuel")


def test_mode_simulation_nouvre_pas_le_ble_reel(monkeypatch):
    from fieldmind.config import Profil
    from fieldmind.materiel.esp import PiloteEsp
    from fieldmind.simulation.materiel_simule import FauxEsp

    p = PiloteEsp(Bus(), Profil.charger(reglages=None), FauxEsp("ROVER_3"), drone_reel=False)

    def interdit():
        raise AssertionError("le mode simulation a démarré une liaison BLE réelle")

    monkeypatch.setattr(p._fil_ble, "start", interdit)
    try:
        p.demarrer()
        assert not p._fil_ble.is_alive()
    finally:
        p.arreter()
