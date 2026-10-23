"""Suivre une personne : désignation, traqueur sur le Pi, distance lidar, commande, sécurité."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
import pytest

from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.ia.outils import ActionInvalide, ContexteValidation, valider_action
from fieldmind.ia.vision import consigne_encadrement, lire_boite
from fieldmind.mission import actions as A
from fieldmind.mission.executeur import Suivre
from fieldmind.perception.suivi import EtatCible, SuiviCible
from fieldmind.types import ImageCamera, Odometrie, Pose2D, Scan

cv2 = pytest.importorskip("cv2")


@pytest.fixture
def profil() -> Profil:
    return Profil.charger(reglages=None)


def image_avec_personne(u: float, numero: int) -> ImageCamera:
    """Fond texturé et une « personne » (t-shirt rouge, pantalon bleu) centrée en u."""
    rng = np.random.default_rng(3)
    bgr = cv2.GaussianBlur(rng.integers(60, 200, (360, 640, 3), dtype=np.uint8), (9, 9), 0)
    x = int(u * 640)
    cv2.rectangle(bgr, (x - 30, 80), (x + 30, 200), (30, 30, 220), -1)
    cv2.rectangle(bgr, (x - 25, 200), (x + 25, 320), (180, 60, 40), -1)
    cv2.circle(bgr, (x, 60), 18, (120, 160, 200), -1)
    ok, jpeg = cv2.imencode(".jpg", bgr)
    assert ok
    return ImageCamera(t=time.monotonic(), camera="gauche", jpeg=jpeg.tobytes(), largeur=640,
                       hauteur=360, numero=numero)


def scan_devant(distance: float) -> Scan:
    """Jambes à `distance` devant la caméra, mur loin derrière."""
    angles = np.radians(np.arange(-180, 180, 1.0)).astype(np.float32)
    distances = np.full_like(angles, 6.0)
    distances[np.abs(angles) < math.radians(3)] = distance + 0.415
    return Scan(t=time.monotonic(), angles=angles, distances=distances.astype(np.float32))


def test_traqueur_suit_et_mesure_au_lidar(profil: Profil) -> None:
    bus = Bus()
    suivi = SuiviCible(bus, profil)
    image = image_avec_personne(0.5, 1)
    assert suivi.demarrer(image, (0.45, 0.12, 0.55, 0.9), "la personne en rouge")
    bus.publier(S.SCAN, scan_devant(2.0))
    for i, u in enumerate([0.5, 0.51, 0.52, 0.53], start=2):
        bus.publier(S.IMAGE_GAUCHE, image_avec_personne(u, i))
        suivi._pas(time.monotonic())
    cible = suivi.cible()
    assert cible.etat == "suivi" and cible.boite is not None
    assert 0.45 < (cible.boite[0] + cible.boite[2]) / 2 < 0.6
    assert cible.source_distance == "lidar" and abs(cible.distance_m - 2.0) < 0.25
    assert suivi.pour_securite() is not None
    assert suivi.resume()["etat"] == "suivi"


def test_boites_robobrain_et_json() -> None:
    assert "bounding box" in consigne_encadrement("la personne en rouge", "robobrain")
    assert lire_boite("[100, 200, 300, 900]", "robobrain") == (0.1, 0.2, 0.3, 0.9)
    assert lire_boite('[{"box_2d": [200, 100, 900, 300]}]', "yx_1000") == (0.1, 0.2, 0.3, 0.9)
    assert lire_boite("[]", "yx_1000") is None


def test_action_et_outil_de_l_agent() -> None:
    propre = A.valider({"type": "suivre", "description": "la personne en t-shirt rouge"})
    assert propre["distance_m"] == 1.5 and propre["camera"] == "gauche"
    assert A.valider({"type": "suivre", "u": 0.4, "v": 0.6})["u"] == 0.4
    with pytest.raises(A.ActionRefusee):
        A.valider({"type": "suivre"})
    assert "suivre" in A.decrire_action(propre)
    action = valider_action("suivre", {"description": "moi", "distance_m": 2},
                            ContexteValidation())
    assert action == {"type": "suivre", "description": "moi", "distance_m": 2.0}
    with pytest.raises(ActionInvalide):
        valider_action("suivre", {"description": "moi", "distance_m": 20}, ContexteValidation())


@dataclass
class FauxSuivi:
    cible_courante: EtatCible
    demarre: bool = False

    def demarrer(self, *_: Any) -> bool:
        self.demarre = True
        return True

    def garder(self) -> None:
        pass

    def arreter(self) -> None:
        pass

    def cible(self) -> EtatCible:
        return self.cible_courante


class FauxFutur:
    def __init__(self, valeur: Any) -> None:
        self.valeur = valeur

    def done(self) -> bool:
        return True

    def result(self) -> Any:
        return self.valeur


class FauxExecuteur:
    def __init__(self, profil: Profil, suivi: FauxSuivi, trouve: Any) -> None:
        self.profil, self.suivi, self.trouve = profil, suivi, trouve

    def lancer_ia(self, demande: str, argument: Any) -> FauxFutur:
        assert demande == "designer"
        return FauxFutur(self.trouve)


def contexte(t: float) -> A.Contexte:
    return A.Contexte(t=t, odom=Odometrie(t=t, pose=Pose2D()), securite=None, carte=None,
                      obstacles=np.zeros((0, 2)), vitesse_max=0.5, vitesse_defaut=0.35,
                      braquage_max=math.radians(25), empattement=0.38)


def test_mission_suivre(profil: Profil) -> None:
    cible = EtatCible(etat="suivi", boite=(0.4, 0.1, 0.6, 0.9), distance_m=3.0,
                      position=(3.4, 0.4), source_distance="lidar", t=time.monotonic())
    suivi = FauxSuivi(cible)
    trouve = {"image": image_avec_personne(0.5, 1), "boite": (0.4, 0.1, 0.6, 0.9)}
    mission = Suivre(A.valider({"type": "suivre", "description": "toi"}),
                     FauxExecuteur(profil, suivi, trouve))  # type: ignore[arg-type]
    pas = mission.pas(contexte(1.0))
    assert suivi.demarre and pas.vitesse > 0.3 and pas.braquage > 0  # personne à gauche
    suivi.cible_courante = EtatCible(etat="suivi", distance_m=1.4, position=(1.8, 0.0),
                                     source_distance="lidar", t=time.monotonic())
    assert mission.pas(contexte(1.1)).vitesse == 0.0  # assez près : on attend
    suivi.cible_courante = EtatCible(etat="perdu", perdu_depuis=1.2)
    pas = mission.pas(contexte(2.0))
    assert pas.vitesse == 0.0 and pas.etat == "bloque"
    assert mission.pas(contexte(30.0)).etat == "echec"


def test_designation_ratee(profil: Profil) -> None:
    mission = Suivre(A.valider({"type": "suivre", "description": "personne"}),
                     FauxExecuteur(profil, FauxSuivi(EtatCible()), None))  # type: ignore[arg-type]
    assert mission.pas(contexte(1.0)).etat == "echec"


def test_securite_ne_confond_pas_la_personne_suivie(profil: Profil) -> None:
    from test_securite import bus_sain

    from fieldmind.securite.superviseur import Superviseur
    from fieldmind.types import Consigne

    bus = bus_sain()
    sup = Superviseur(bus, profil)
    sup.operateurs = 1
    bus.publier(S.DETECTIONS, {"t": time.time(), "objets": [
        {"label": "personne", "sol": [2.5, 0.0]}]})
    consigne = Consigne(t=time.monotonic(), vitesse=0.5, source="auto")
    sortie, etat = sup.evaluer(consigne, "auto", True)
    assert "personne_proche" in etat.raisons and sortie.vitesse <= 0.25
    sup.cible_suivie = lambda: (0.0, 2.5)  # c'est elle qu'on suit
    sortie, etat = sup.evaluer(Consigne(t=time.monotonic(), vitesse=0.5, source="auto"),
                               "auto", True)
    assert "personne_proche" not in etat.raisons and sortie.vitesse > 0.25
