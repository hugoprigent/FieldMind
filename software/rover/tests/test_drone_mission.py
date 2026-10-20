"""Mission fermée sur mesures : images synthétiques, aléas et replis sans matériel."""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest

from fieldmind.drone.autopilote import Autopilote, ReglagesMission
from fieldmind.drone.modele import Cible, Observation, Ordre
from fieldmind.drone.simulation import DroneSimule, executer
from fieldmind.drone.vision import CalibrationCroix, VisionCroix


@pytest.mark.parametrize("scenario", ["normal", "derive"])
@pytest.mark.parametrize("graine", range(12))
def test_mission_six_photos_tour_complet_et_pose_par_paliers(scenario, graine):
    d, photos = executer(graine, scenario)
    assert d["phase"] == "termine" and d["au_sol"]
    assert d["photos"] == len(photos) == 6
    assert d["paliers"] >= 5
    assert 357 < d["rotation_deg"] < 363
    assert d["distance_pose_m"] < 0.045
    assert [a for _, a in d["actions"]] == ["decoller", "camera_bas", "atterrir"]
    assert len({p.tobytes() for p in photos}) == 6  # six angles, pas six copies
    phases = [e["phase"] for e in d["journal"]]
    assert phases.index("camera_bas") > phases.index("rotation")
    assert phases.count("descente") >= 5
    assert phases.count("recentrage") >= phases.count("descente")


@pytest.mark.parametrize(
    "scenario", ["croix_perdue", "video_perdue", "camera_bloquee", "cap_perdu", "gx10_retard"]
)
def test_incident_ne_produit_pas_fausse_mission_reussie(scenario):
    d, _ = executer(3, scenario)
    assert d["phase"] == ("erreur" if scenario == "cap_perdu" else "interrompu")
    assert d["au_sol"]
    actions = [a for _, a in d["actions"]]
    assert actions.count("decoller") == 1
    assert actions.count("atterrir") == 1
    assert "stop" not in actions  # aucun arrêt moteur en altitude pour faire un repli
    if scenario == "gx10_retard":
        assert d["paliers"] == 0


def test_perte_croix_interrompt_immediatement_descente():
    d, p = DroneSimule(), Autopilote()
    ordre = p.demarrer(d.observer(), 0)
    for _ in range(2000):
        d.avancer(ordre)
        ordre = p.pas(d.observer(), d.t)
        if p.phase == "descente":
            break
    o = replace(d.observer(), cible=None)
    sortie = p.pas(o, d.t)
    assert sortie.montee_m_s == 0
    assert p.phase == "recentrage"
    assert p.hauteur_cible == pytest.approx(d.h)


def test_altitude_ne_peut_pas_etre_validee_par_temps_seul():
    d, p = DroneSimule(), Autopilote()
    p.demarrer(d.observer(), 0)
    for i in range(1, 326):
        t = i * 0.04
        o = Observation(t, i, 0.0, 0.0, au_sol=True, camera="avant", image_t=t)
        p.pas(o, t)
    assert p.phase == "interrompu" and p.photos == 0


def test_rotation_ne_peut_pas_etre_validee_par_temps_seul():
    d, p = DroneSimule(), Autopilote()
    p.demarrer(d.observer(), 0)
    p._phase("rotation", 0, "test")
    for i in range(1, 1700):
        t = i * 0.04
        o = Observation(t, i, 1.0, 0.0, au_sol=False, camera="avant", image_t=t)
        p.pas(o, t)
    assert p.phase == "repli"
    assert p.photos == 1  # seulement l'angle initial réellement observé
    assert p.rotation_rad == 0


@pytest.mark.parametrize(
    "changement",
    [
        {"hauteur_m": None},
        {"cap_rad": None},
        {"cap_rad": math.nan},
        {"hauteur_m": math.inf},
        {"au_sol": None},
        {"camera": "bas"},
        {"batterie_pct": 10},
        {"t": -1},
    ],
)
def test_prevol_exige_des_mesures_exploitables(changement):
    d = DroneSimule()
    with pytest.raises(ValueError):
        Autopilote().demarrer(replace(d.observer(), **changement), 0)


def test_annulation_demande_land_une_seule_fois():
    d, p = DroneSimule(), Autopilote()
    ordre = p.demarrer(d.observer(), 0)
    for _ in range(50):
        d.avancer(ordre)
        ordre = p.pas(d.observer(), d.t)
    p.annuler()
    assert p.pas(d.observer(), d.t).action == "atterrir"
    for _ in range(10):
        d.avancer(Ordre())
        assert p.pas(d.observer(), d.t).action is None


def test_un_saut_de_cap_ne_devient_pas_un_tour_mesure():
    d, p = DroneSimule(), Autopilote()
    p.demarrer(d.observer(), 0)
    o = replace(
        d.observer(), t=0.04, sequence=1, cap_rad=2.0, image_t=0.04, hauteur_m=0.5, au_sol=False
    )
    assert p.pas(o, 0.04).action == "atterrir"
    assert p.phase == "repli"


@pytest.mark.parametrize("angle_deg", [0, 15, 45, 90, 140, 200])
def test_pose_visuelle_issue_des_pixels(angle_deg):
    d = DroneSimule()
    d.camera, d.h, d.x, d.y, d.cap = "bas", 0.8, 0.04, -0.03, math.radians(angle_deg)
    cible = VisionCroix().estimer(d._rendre(), 1, 4)
    assert cible is not None
    assert cible.hauteur_m == pytest.approx(d.h, abs=0.035)
    assert cible.avant_m == pytest.approx(-d.x * math.cos(d.cap) - d.y * math.sin(d.cap), abs=0.009)
    assert cible.droite_m == pytest.approx(d.x * math.sin(d.cap) - d.y * math.cos(d.cap), abs=0.009)


def test_pose_indisponible_si_pas_de_croix():
    assert VisionCroix().estimer(np.full((240, 320, 3), 245, np.uint8), 0, 1) is None
    with pytest.raises(ValueError):
        VisionCroix().estimer(np.zeros((480, 640, 3), np.uint8), 0, 1)


@pytest.mark.parametrize("valeur", [math.nan, math.inf, -3, 2])
def test_refuse_consigne_physique_invalide(valeur):
    with pytest.raises(ValueError):
        Ordre(montee_m_s=valeur)


def test_calibration_invalide_et_pose_perimee():
    with pytest.raises(ValueError):
        CalibrationCroix(fx=math.nan)
    with pytest.raises(ValueError):
        ReglagesMission(hauteur_scan_m=10)
    assert not Cible(1, 1, 0, 0, 1).valide(2)
    assert not Cible(1, 1, 0, 0, math.nan).valide(1)
    assert not Cible(2, 1, 0, 0, 1).valide(1)


def test_simulation_ne_consulte_aucun_socket_ni_port_reel(monkeypatch):
    import socket

    def interdit(*a, **k):
        raise AssertionError("tentative d'ouverture réseau depuis le simulateur")

    monkeypatch.setattr(socket, "socket", interdit)
    d, _ = executer(1)
    assert d["phase"] == "termine"
