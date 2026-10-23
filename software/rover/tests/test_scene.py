"""Scène 3D : murs, arbres et blocs retrouvés dans le monde simulé, format du message."""

from __future__ import annotations

import math
import time

import numpy as np
import pytest
from budget import dans_le_budget

from fieldmind.config import Profil
from fieldmind.perception.scene import ReconstructeurScene, decouper_segments
from fieldmind.simulation.monde import Monde
from fieldmind.types import EtatSecurite, Obstacles, Pose2D, Scan


@pytest.fixture
def profil() -> Profil:
    return Profil.charger(reglages=None)


def parcourir(monde: Monde, profil: Profil, tours: int = 60) -> tuple[ReconstructeurScene, Pose2D,
                                                                       float]:
    r = ReconstructeurScene(profil)
    pose = monde.depart
    debut = time.perf_counter()
    for k in range(tours):
        pose = Pose2D(pose.x + 0.05 * math.cos(pose.cap), pose.y + 0.05 * math.sin(pose.cap),
                      pose.cap + 0.01)
        angles, distances = monde.lancer_lidar(pose, profil)
        garde = distances > 0.12
        r.integrer_scan(Scan(t=k * 0.1, angles=np.asarray(angles, np.float32)[garde],
                             distances=np.asarray(distances, np.float32)[garde]), pose, k * 0.1)
        if k % 10 == 0:
            r.message(pose, None, None, maintenant=k * 0.1)
    return r, pose, (time.perf_counter() - debut) / tours


def test_murs_de_la_salle(profil: Profil) -> None:
    r, pose, duree = parcourir(Monde(0, "salle"), profil)
    message = r.message(pose, None, None, maintenant=6.0)
    murs = [o for o in message["objets"] if o["type"] == "mur" and not o["extrapole"]]
    assert dans_le_budget(duree, 0.01)  # par tour du lidar, sur le Mac
    # Chaque grand mur de la pièce (x = 0, x = 10, y = 0, y = 7) est retrouvé au moins en partie
    for axe, valeur in (("x", 0.0), ("x", 10.0), ("y", 0.0), ("y", 7.0)):
        assert any(abs(m[axe] - valeur) < 0.1 for m in murs), (axe, valeur)
    # Le mur du bas, vu en entier, n'est pas en morceaux superposés
    bas = [m for m in murs if abs(m["y"]) < 0.1]
    assert len(bas) == 1 and bas[0]["longueur"] > 8
    types = {o["type"] for o in message["objets"]}
    assert {"mur", "arbre", "bloc"} <= types
    assert len(message["libre"]) == 90 and message["repere"] == "odom"


def test_obstacles_bas_danger_et_oubli(profil: Profil) -> None:
    r = ReconstructeurScene(profil)
    pose = Pose2D()
    points = np.array([[1.0 + 0.05 * i, 0.02 * i] for i in range(8)], np.float32)
    r.integrer_stereo(Obstacles(t=0.0, source="stereo", points=points,
                                hauteurs=np.full(8, 0.12, np.float32)), pose, 0.0)
    securite = EtatSecurite(t=0.0, arret=True, raisons=["obstacle_bas"], libre_avant_m=0.5)
    message = r.message(pose, None, securite, maintenant=0.1)
    bas = [o for o in message["objets"] if o["type"] == "bas"]
    assert len(bas) == 1 and bas[0]["hauteur"] == pytest.approx(0.12) and bas[0]["danger"]
    assert not r.message(pose, None, None, maintenant=40.0)["objets"]  # oublié après 30 s


def test_decoupage_en_segments() -> None:
    coin = np.array([[x, 0.0] for x in np.linspace(0, 2, 21)]
                    + [[2.0, y] for y in np.linspace(0.1, 1.5, 15)])
    segments = decouper_segments(coin)
    assert len(segments) == 2


def test_integrer_vision_fusion_3d(profil: Profil) -> None:
    r = ReconstructeurScene(profil)
    pose = Pose2D(1.0, 2.0, 0.0)
    r.integrer_vision({
        "points": [{
            "label": "caisse bleue",
            "sol": [1.5, -0.4],
            "hauteur": 0.52,
            "largeur": 0.65,
            "longueur": 0.45,
            "cap": 0.2,
            "type_3d": "bloc",
            "source": "lidar+stereo",
        }],
    }, pose, 1.0)
    message = r.message(pose, None, None, maintenant=1.1)
    etiquettes = [o for o in message["objets"] if o["type"] == "etiquette"]
    blocs = [o for o in message["objets"] if o["type"] == "bloc"]
    assert len(etiquettes) == 1 and etiquettes[0]["texte"] == "caisse bleue"
    assert len(blocs) == 1
    assert blocs[0]["x"] == pytest.approx(2.5) and blocs[0]["y"] == pytest.approx(1.6)
    assert blocs[0]["hauteur"] == pytest.approx(0.52)
    assert blocs[0]["largeur"] == pytest.approx(0.65)



def test_vision_invalide_et_etiquettes_voisines(profil: Profil) -> None:
    r = ReconstructeurScene(profil)
    pose = Pose2D()
    for message in (None, {"points": None}, {"points": 1}):
        r.integrer_vision(message, pose, 0)
        r.integrer_detections(message, pose, 0)
    r.integrer_vision({"points": [None, {"label": "porte", "sol": [float("nan"), 0]},
                                   {"label": "porte", "sol": "??"},
                                   {"label": "porte blanche", "sol": [1, 0]},
                                   {"label": "interrupteur", "sol": [1, 0.2]}]}, pose, 0)
    objets = r.message(pose, None, None, maintenant=0.1)["objets"]
    assert {o["texte"] for o in objets} == {"porte blanche", "interrupteur"}
    r.integrer_vision({"geometrie_valide": False,
                       "points": [{"label": "fantôme", "sol": [2, 0]}]}, pose, 0)
    assert len(r.objets) == 2


def test_dimensions_visuelles_et_hauteurs_mesurees_identifiees(profil: Profil) -> None:
    r = ReconstructeurScene(profil)
    pose = Pose2D()
    r.integrer_vision({"points": [{"label": "caisse", "sol": [1, 0], "type_3d": "bloc",
                                  "hauteur": float("nan"), "largeur": -1,
                                  "cap": float("inf")} ]}, pose, 0)
    bloc, = [o for o in r.message(pose, None, None, maintenant=0.1)["objets"]
             if o["type"] == "bloc"]
    assert bloc["hauteur_estimee"] and math.isfinite(bloc["hauteur"])
    assert bloc["largeur"] > 0 and math.isfinite(bloc["cap"])
    points = np.array([[2.0 + 0.03 * i, 0.02 * i] for i in range(5)], np.float32)
    r.integrer_stereo(Obstacles(t=0, source="stereo", points=points,
                                hauteurs=np.full(5, 0.12, np.float32)), pose, 0)
    bas, = [o for o in r.message(pose, None, None, maintenant=0.1)["objets"]
            if o["type"] == "bas"]
    assert not bas["hauteur_estimee"]
