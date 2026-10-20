"""Essais visuels sans matériel : aucune commande moteur permise par la reconnaissance."""

from __future__ import annotations

import base64
import json
import threading
import time
from dataclasses import replace
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from aiohttp.test_utils import TestClient, TestServer

from fieldmind.api.serveur import Serveur
from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.materiel.esp import PiloteEsp
from fieldmind.materiel.reconnaissance import detecter_croix
from fieldmind.mission.actions import ActionRefusee
from fieldmind.simulation.materiel_simule import FauxEsp
from fieldmind.types import ImageCamera


def dessin(forme="plus", couleur=(0, 0, 255), dx=0):
    im = np.full((320, 240, 3), 245, np.uint8)
    if forme == "plus":
        cv2.rectangle(im, (110 + dx, 110), (130 + dx, 210), couleur, -1)
        cv2.rectangle(im, (70 + dx, 150), (170 + dx, 170), couleur, -1)
    elif forme == "x":
        cv2.line(im, (75 + dx, 115), (165 + dx, 205), couleur, 20)
        cv2.line(im, (75 + dx, 205), (165 + dx, 115), couleur, 20)
    elif forme == "rectangle":
        cv2.rectangle(im, (70, 110), (170, 210), couleur, -1)
    elif forme == "cercle":
        cv2.circle(im, (120, 160), 50, couleur, -1)
    return im


def image(t, forme="plus", dx=0):
    _, jpeg = cv2.imencode(".jpg", dessin(forme, dx=dx))
    return ImageCamera(
        t=t, camera="drone", jpeg=bytes(jpeg), largeur=240, hauteur=320, numero=int(t * 100)
    )


@pytest.fixture
def pilote(tmp_path):
    p = PiloteEsp(Bus(), Profil.charger(reglages=None), FauxEsp("ROVER_3"), drone_reel=False)
    p.recon.dossier = tmp_path
    return p


@pytest.mark.parametrize("couleur", [(0, 0, 255), (0, 0, 0)])
@pytest.mark.parametrize("forme", ["plus", "x"])
def test_croix_centree_et_decentree(forme, couleur):
    assert detecter_croix(dessin(forme, couleur))["verrouillee"]
    d = detecter_croix(dessin(forme, couleur, dx=35))
    assert d["trouvee"] and not d["verrouillee"] and d["x"] > 0.25


@pytest.mark.parametrize("forme", ["vide", "rectangle", "cercle"])
def test_rejette_faux_positifs_simples(forme):
    assert not detecter_croix(dessin(forme))["trouvee"]


@pytest.mark.parametrize(
    "commande", ["cmd decoller", "cmd recon_vol", "cmd calibrer", "manette 255 128 128 128"]
)
def test_vol_refuse_meme_si_connecte(pilote, commande):
    pilote._ble_connecte = True
    pilote.etat_drone = replace(pilote.etat_drone, connecte=True)
    assert not pilote.commander_drone(commande)
    assert "non validé" in pilote.drone_erreur
    assert not pilote._ble_cmd_file
    assert pilote.recon.mode == "inactif"


def test_console_ne_contourne_pas_validation(pilote):
    assert not pilote.envoyer_ligne("drone cmd decoller")
    assert not pilote.envoyer_ligne("DRONE manette 255 128 128 128")
    assert not pilote._ble_cmd_file


def test_camera_acceptee_sans_trame_de_vol(pilote):
    pilote._ble_connecte = True
    pilote.etat_drone = replace(pilote.etat_drone, connecte=True)
    assert pilote.commander_drone("cmd camera")
    assert list(pilote._ble_cmd_file) == [bytes([0x43, 2, 0, 0, 0, 0]), bytes([6, 2])]
    assert all(p[0] != 0x66 for p in pilote._ble_cmd_file)
    for _ in range(20):
        pilote.commander_drone("cmd camera")
    assert len(pilote._ble_cmd_file) <= 8


def test_aucune_commande_differree_si_deconnecte(pilote):
    assert not pilote.commander_drone("cmd camera")
    assert not pilote._ble_cmd_file
    assert "déconnecté" in pilote.drone_erreur


def test_mire_ne_permet_pas_de_demarrer(pilote):
    pilote._publier_mire_drone(time.monotonic())
    assert pilote.bus.dernier(S.IMAGE_DRONE) is not None
    assert not pilote.commander_drone("cmd recon_test")
    assert "image réelle" in pilote.drone_erreur


def test_centrage_exige_images_distinctes_et_continues(pilote):
    r = pilote.recon
    r.deposer(image(10))
    r.commander("cibler_croix", 10)
    r.traiter(10)
    for t in [10.2, 10.4, 10.6, 10.8]:
        r.traiter(t)
    assert not pilote.etat_drone.croix_verrouillee
    for t in [10.2, 10.4, 10.6, 10.9]:
        r.deposer(image(t))
        r.traiter(t)
    assert pilote.etat_drone.croix_verrouillee
    r.deposer(image(11.1, dx=35))
    r.traiter(11.1)
    assert not pilote.etat_drone.croix_verrouillee
    assert not pilote._ble_cmd_file


def test_perte_image_annule_sans_reprise_automatique(pilote):
    r = pilote.recon
    r.deposer(image(10))
    r.commander("cibler_croix", 10)
    r.traiter(10)
    r.traiter(11.6)
    assert r.mode == "erreur"
    assert not pilote.etat_drone.croix_verrouillee
    r.deposer(image(12))
    r.traiter(12)
    assert r.mode == "erreur"
    assert not pilote._ble_cmd_file


def test_capture_six_vues_persistantes_et_rapport_honnete(pilote):
    r = pilote.recon
    r.deposer(image(10))
    r.commander("recon_test", 10)
    for i in range(6):
        t = 11 + i
        r.commander("recon_capture", t)
        r.traiter(t)  # l'image précédente ne doit pas être capturée
        assert len(r._vues) == i
        r.deposer(image(t + 0.1))
        r.traiter(t + 0.1)
        assert len(r._vues) == i + 1
    assert r.mode == "recherche_croix"
    assert r.fichier("montage.jpg").is_file()
    rapport = json.loads(r.fichier("rapport.json").read_text())
    assert rapport["mode"] == "sans_vol" and len(rapport["vues"]) == 6
    assert rapport["couverture_360_verifiee"] is False
    assert rapport["atterrissage_commande"] is False
    assert r.fichier("../rover.toml") is None
    assert r.fichier("vue-7.jpg") is None
    assert not pilote._ble_cmd_file
    assert pilote.etat_drone.montage_recon
    for t in [16.3, 16.5, 16.7, 16.9, 17.2]:
        r.deposer(image(t))
        r.traiter(t)
    assert r.mode == "termine"
    assert "atterrissage commandé" in pilote.etat_drone.consigne_vol
    r.commander("cibler_croix", 17.2)
    assert r.fichier("montage.jpg") is not None  # captures conservées après bascule caméra


def test_perte_croix_ne_declare_jamais_un_pose(pilote):
    r = pilote.recon
    r.deposer(image(10, "vide"))
    r.commander("recon_test", 10)
    r.mode = "recherche_croix"
    for t in [11, 20, 30.1]:
        r.deposer(image(t, "vide"))
        r.traiter(t)
    assert r.mode == "erreur"
    assert not pilote.etat_drone.croix_verrouillee
    assert not pilote._ble_cmd_file


def test_jpeg_wifi_passe_aussi_par_la_vision(pilote):
    im = image(10)
    pilote._traiter_drone_jpg(1, 1, 1, base64.b64encode(im.jpeg).decode(), 10)
    pilote.recon.commander("cibler_croix", 10)
    pilote.recon.traiter(10)
    assert pilote.etat_drone.croix_trouvee


def test_erreur_disque_interrompt_essai_sans_moteur(pilote, monkeypatch):
    r = pilote.recon
    r.deposer(image(10))
    r.commander("recon_test", 10)
    r.commander("recon_capture", 10)
    r.deposer(image(10.1))
    monkeypatch.setattr(cv2, "imwrite", lambda *args: False)
    with pytest.raises(OSError):
        r.traiter(10.1)
    # La boucle du fil transforme cette exception en état d'erreur.
    r.interrompre("échec stockage")
    assert r.mode == "erreur" and not pilote._ble_cmd_file


def test_depot_ble_ne_bloque_pas_sur_traitement(pilote):
    verrou_pris = threading.Event()
    fini = threading.Event()

    def tenir_verrou():
        with pilote.recon._verrou:
            verrou_pris.set()
            fini.wait(2)

    fil = threading.Thread(target=tenir_verrou)
    fil.start()
    assert verrou_pris.wait(1)
    try:
        debut = time.monotonic()
        pilote.recon.deposer(image(10))
        assert time.monotonic() - debut < 0.1
    finally:
        fini.set()
        fil.join()


@pytest.mark.parametrize("cmd", ["camera", "recon_vol", "recon_test", "inconnue"])
async def test_api_ne_retourne_pas_faux_succes(pilote, cmd):
    rover = SimpleNamespace(
        bus=pilote.bus,
        conduite=None,
        profil=None,
        materiel=SimpleNamespace(esp=lambda: pilote),
        ia=None,
    )
    serveur = Serveur(rover)
    with pytest.raises(ActionRefusee):
        await serveur._traiter(None, {"type": "drone", "action": "cmd", "cmd": cmd})


async def test_export_protege_par_jeton_et_sans_traversee(pilote, tmp_path):
    dossier = tmp_path / "essai"
    dossier.mkdir()
    (dossier / "rapport.json").write_text('{"mode":"sans_vol"}')
    pilote.recon._session = dossier
    rover = SimpleNamespace(
        bus=pilote.bus,
        conduite=None,
        profil=None,
        materiel=SimpleNamespace(esp=lambda: pilote),
        ia=None,
    )
    serveur = Serveur(rover, jeton="essai")
    serveur.app.on_startup.clear()
    serveur.app.on_shutdown.clear()
    async with TestClient(TestServer(serveur.app)) as client:
        assert (await client.get("/api/drone/recon/rapport.json")).status == 401
        reponse = await client.get("/api/drone/recon/rapport.json?jeton=essai")
        assert reponse.status == 200 and (await reponse.json())["mode"] == "sans_vol"
        assert (await client.get("/api/drone/recon/inconnu?jeton=essai")).status == 404
