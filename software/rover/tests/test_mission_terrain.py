"""Retour à une base mémorisée et photographie réellement enregistrée, sans matériel."""

from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from test_exploration import contexte

import fieldmind.mission.executeur as M
from fieldmind.bus import Bus, S
from fieldmind.mission.actions import AllerA
from fieldmind.types import ImageCamera, Pose2D


class Gps:
    session = "session-actuelle"

    def __init__(self, session):
        self.enregistre = {"nom": "base", "session": session, "x": 2.0, "y": 1.0,
                           "lat": 48.0, "lon": 2.0}
        self.conversions_gps = []

    def lieu(self, nom):
        assert nom == "base"
        return self.enregistre

    def latlon_vers_odom(self, lat, lon):
        self.conversions_gps.append((lat, lon))
        return 8.0, 0.0  # volontairement différent des coordonnées odom mémorisées

    def lieu_vers_odom(self, _lieu):
        raise AssertionError("conversion de secours inutile : un emplacement GPS existe")


def executeur(gps):
    profil = SimpleNamespace(valeur=lambda _chemin, defaut: defaut)
    return SimpleNamespace(gps=gps, profil=profil)


def test_retour_base_meme_session_prefere_odom_precis_malgre_gps():
    gps = Gps("session-actuelle")
    retour = M._AllerLieu({"type": "aller_lieu", "nom": "base"}, executeur(gps))
    resultat = retour.pas(contexte())
    assert isinstance(retour._sous, AllerA)
    assert retour._sous.action == {"type": "aller_a", "x": 2.0, "y": 1.0, "repere": "odom"}
    assert gps.conversions_gps == []
    assert resultat.etat == "route"
    arrivee = retour.pas(contexte(t=1, pose=Pose2D(2.0, 1.0)))
    assert arrivee.etat == "termine" and arrivee.vitesse == 0


def test_retour_base_autre_session_ignore_ancien_odom_et_garde_gps():
    gps = Gps("ancienne-session")
    retour = M._AllerLieu({"type": "aller_lieu", "nom": "base"}, executeur(gps))
    resultat = retour.pas(contexte())
    assert isinstance(retour._sous, M.AllerGps)
    assert retour._sous.action == {"type": "aller_gps", "lat": 48.0, "lon": 2.0}
    assert gps.conversions_gps == [(48.0, 2.0)]
    assert retour._sous._sous.but == (8.0, 0.0)
    assert resultat.etat == "route"


def test_photo_sans_image_echoue_et_ne_cree_pas_de_fichier(tmp_path, monkeypatch):
    monkeypatch.setattr(M, "DOSSIER_PHOTOS", tmp_path / "photos")
    resultat = M._Photo({"type": "photo", "camera": "gauche"}, Bus()).pas(contexte())
    assert resultat.etat == "echec" and "aucune image" in resultat.raison
    assert resultat.vitesse == 0
    assert not list(tmp_path.rglob("*.jpg"))


@pytest.mark.parametrize("camera,sujet", [("gauche", S.IMAGE_GAUCHE), ("droite", S.IMAGE_DROITE)])
def test_photo_reussit_uniquement_avec_fichier_jpeg_reel(camera, sujet, tmp_path, monkeypatch):
    dossier = tmp_path / "photos"
    monkeypatch.setattr(M, "DOSSIER_PHOTOS", dossier)
    image = np.full((8, 12, 3), 120, dtype=np.uint8)
    reussi, encode = cv2.imencode(".jpg", image)
    assert reussi
    octets = encode.tobytes()
    bus = Bus()
    bus.publier(sujet, ImageCamera(t=0, camera=camera, jpeg=octets, largeur=12, hauteur=8))
    resultat = M._Photo({"type": "photo", "camera": camera}, bus).pas(contexte())
    assert resultat.etat == "termine" and resultat.vitesse == 0
    fichiers = list(dossier.glob(f"*_{camera}.jpg"))
    assert len(fichiers) == 1 and fichiers[0].read_bytes() == octets
    assert cv2.imread(str(fichiers[0])).shape == (8, 12, 3)
