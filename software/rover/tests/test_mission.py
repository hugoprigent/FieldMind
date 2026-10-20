"""Primitives de mission sur un rover cinématique idéal, et décodeur du lidar."""

from __future__ import annotations

import math

import numpy as np
import pytest

from fieldmind.config import Profil
from fieldmind.materiel.lidar import Decodeur, FiltreRover
from fieldmind.mission import actions as A
from fieldmind.types import EtatSecurite, Odometrie, Pose2D


@pytest.fixture
def profil() -> Profil:
    return Profil.charger(reglages=None)


def rouler(primitive: A.Primitive, profil: Profil, duree: float = 60.0,
           securite: EtatSecurite | None = None, dt: float = 0.1
           ) -> tuple[A.Pas, Pose2D, list[A.Pas]]:
    """Joue la primitive : la vitesse suit la consigne (premier ordre), pose intégrée."""
    pose, v = Pose2D(), 0.0
    empattement = profil.geometrie.empattement_m
    historique = []
    t = 0.0
    while t < duree:
        odom = Odometrie(t=t, pose=pose, vitesse=v)
        ctx = A.Contexte(t=t, odom=odom, securite=securite or EtatSecurite(
            t=t, arret=False, vitesse_max_avant=1.0, vitesse_max_arriere=0.3),
            carte=None, obstacles=np.zeros((0, 2), np.float32), vitesse_max=0.5,
            vitesse_defaut=0.35, braquage_max=math.radians(31), empattement=empattement)
        pas = primitive.pas(ctx)
        historique.append(pas)
        if pas.etat in ("termine", "echec"):
            return pas, pose, historique
        s = ctx.securite
        cible = max(-s.vitesse_max_arriere, min(s.vitesse_max_avant, pas.vitesse))  # conduite
        v += (cible - v) * min(1.0, dt / 0.3)
        w = v * math.tan(pas.braquage) / empattement
        pose = Pose2D(pose.x + v * math.cos(pose.cap) * dt, pose.y + v * math.sin(pose.cap) * dt,
                      A.normaliser_angle(pose.cap + w * dt))
        t += dt
    raise AssertionError("primitive non terminée")


def test_avancer_cinq_metres(profil: Profil) -> None:
    pas, pose, _ = rouler(A.Avancer(A.valider({"type": "avancer", "distance_m": 5})), profil)
    assert pas.etat == "termine"
    assert pose.x == pytest.approx(5.0, abs=0.08) and abs(pose.y) < 0.03


def test_reculer(profil: Profil) -> None:
    pas, pose, _ = rouler(A.Avancer(A.valider({"type": "avancer", "distance_m": -1.0})), profil)
    assert pas.etat == "termine" and pose.x == pytest.approx(-1.0, abs=0.06)


@pytest.mark.parametrize("angle", [90, -90, 180])
def test_tourner(profil: Profil, angle: float) -> None:
    pas, pose, _ = rouler(A.Tourner(A.valider({"type": "tourner", "angle_deg": angle})), profil)
    assert pas.etat == "termine"
    attendu = A.normaliser_angle(math.radians(angle))
    assert abs(A.normaliser_angle(pose.cap - attendu)) < math.radians(5)


def test_tourner_manoeuvre_si_bloque_devant(profil: Profil) -> None:
    """Devant bloqué au départ : le rover recule en contre-braquant, puis repart."""
    primitive = A.Tourner(A.valider({"type": "tourner", "angle_deg": 90}))
    bloque = EtatSecurite(t=0, arret=False, vitesse_max_avant=0.0, vitesse_max_arriere=0.3)
    pose, empattement = Pose2D(), profil.geometrie.empattement_m
    v = 0.0
    for k in range(15):
        ctx = A.Contexte(t=k * 0.1, odom=Odometrie(t=k * 0.1, pose=pose), securite=bloque,
                         carte=None, obstacles=np.zeros((0, 2), np.float32), vitesse_max=0.5,
                         vitesse_defaut=0.35, braquage_max=math.radians(31),
                         empattement=empattement)
        pas = primitive.pas(ctx)
        v = pas.vitesse
        w = v * math.tan(pas.braquage) / empattement
        pose = Pose2D(pose.x + v * math.cos(pose.cap) * 0.1, pose.y + v * math.sin(pose.cap) * 0.1,
                      pose.cap + w * 0.1)
    assert primitive.phase == -1 and pose.cap > 0  # recule et le cap tourne déjà du bon côté


def test_avancer_bloque_echoue_apres_attente(profil: Profil) -> None:
    bloque = EtatSecurite(t=0, arret=True, vitesse_max_avant=0.0, vitesse_max_arriere=0.0)
    pas, _, historique = rouler(A.Avancer(A.valider({"type": "avancer", "distance_m": 2})),
                                profil, duree=20, securite=bloque)
    assert pas.etat == "echec" and "bloquée" in (pas.raison or "")
    assert historique[5].etat == "bloque"


@pytest.mark.parametrize("action", [
    {"type": "avancer", "distance_m": 50}, {"type": "avancer"}, {"type": "tourner",
                                                                 "angle_deg": "x"},
    {"type": "voler"}, {"type": "chercher", "objet": ""}, {"type": "aller_a", "x": 40, "y": 0},
])
def test_validation_refuse(action: dict) -> None:
    with pytest.raises(A.ActionRefusee):
        A.valider(action)


def paquet(angle_deg: float, distance_mm: float, debut: bool) -> bytes:
    a = round(angle_deg * 64)
    d = round(distance_mm * 4)
    b0 = (40 << 2) | (1 if debut else 2)
    return bytes([b0, ((a & 0x7F) << 1) | 1, a >> 7, d & 0xFF, d >> 8])


def test_decodeur_lidar_et_repere(profil: Profil) -> None:
    flux = b"\x00\x13"  # octets parasites : resynchronisation
    for _tour in range(2):
        for i in range(360):
            flux += paquet(i, 2000 if i != 90 else 500, i == 0)
    flux += paquet(0, 2000, True)  # début du tour suivant : clôt le second
    decodeur = Decodeur()
    tours = decodeur.recevoir(flux[:700]) + decodeur.recevoir(flux[700:])
    assert len(tours) == 2 and len(tours[1][0]) == 360
    angles, distances = FiltreRover(profil).convertir(tours[1][0], tours[1][1])
    # 90° du lidar (horaire) = à droite du rover : y négatif dans base_link
    i = int(np.argmin(distances))
    assert distances[i] == pytest.approx(0.5, abs=1e-3)
    assert math.degrees(angles[i]) == pytest.approx(-90, abs=0.6)
