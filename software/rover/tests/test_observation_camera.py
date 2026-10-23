"""Géométrie liée à l'image : aucun capteur courant après la réponse réseau."""

from __future__ import annotations

import base64
import json
import math
import time

import numpy as np
import pytest
from test_ia_faux_serveur import FauxOutils, config_essai, faux_serveur

from fieldmind.app import Rover
from fieldmind.bus import S
from fieldmind.config import Profil
from fieldmind.ia.fournisseurs import ErreurIA
from fieldmind.ia.vision import Vision, consigne_analyse_scene, lire_detections
from fieldmind.types import ImageCamera, Imu, Obstacles, Odometrie, Pose2D, Scan


@pytest.fixture
def rover(tmp_path, monkeypatch):
    profil = Profil.charger(reglages=tmp_path / "reglages.json")
    r = Rover(profil, simulation=True, enregistrer=False)
    # Les tests fournissent seulement des observations synthétiques, sans oracle du monde.
    monkeypatch.setattr(r.outils, "detecter_camera", lambda camera: [])
    return r


def publier_image(rover, t=None, jpeg=b"image capturee"):
    t = time.monotonic() if t is None else t
    image = ImageCamera(t=t, camera="gauche", jpeg=jpeg, largeur=640, hauteur=360)
    rover.bus.publier(S.IMAGE_GAUCHE, image)
    return image


def publier_pose(rover, t, pose=None):
    odom = Odometrie(t=t, pose=pose or Pose2D(1, 2, math.pi / 2))
    rover.bus.publier(S.ODOM, odom)
    return odom


def publier_mur(rover, t, distance=2.0):
    y = np.linspace(-0.8, 0.8, 41, dtype=np.float32)
    x = np.full_like(y, distance)
    scan = Scan(t=t, angles=np.arctan2(y, x), distances=np.hypot(x, y))
    rover.bus.publier(S.SCAN, scan)
    return scan


def test_snapshot_pose_de_la_frame_et_projection_imu_figees(rover):
    image = publier_image(rover)
    ancienne = publier_pose(rover, image.t)
    rover.bus.publier(S.IMU, Imu(t=image.t, accel_g=(0, 0, 1), gyro_rad_s=(0, 0, 0)))
    avant = rover.pixel_vers_sol("gauche", 0.5, 0.7)
    publier_pose(rover, image.t + 0.2, Pose2D(8, 9, 0))
    snapshot = rover.outils.snapshot_image()
    assert snapshot is not None and snapshot.pose == ancienne.pose
    ancienne.pose.x = 99  # une référence de pose mutable ne doit pas déplacer la capture
    rover.bus.publier(S.IMU, Imu(t=image.t + 0.3, accel_g=(-0.3, 0, 1),
                                 gyro_rad_s=(0, 0, 0)))
    assert snapshot.image_t == image.t and snapshot.image.jpeg == image.jpeg
    assert snapshot.pose.x == 1
    assert snapshot.pixel_vers_sol("gauche", 0.5, 0.7) == pytest.approx(avant)
    assert rover.pixel_vers_sol("gauche", 0.5, 0.7) != pytest.approx(avant)


@pytest.mark.parametrize("decalage", [-2.0, 2.0])
def test_image_hors_fenetre_temporelle_rejetee(rover, decalage):
    publier_image(rover, time.monotonic() + decalage)
    assert rover.outils.snapshot_image() is None
    assert rover.outils.derniere_image("inconnue") is None


def test_capture_sans_pose_ou_pose_trop_eloignee_de_la_frame(rover):
    image = publier_image(rover)
    assert rover.outils.snapshot_image().pose is None
    publier_pose(rover, image.t - 1)
    assert rover.outils.snapshot_image().pose is None


def test_capteurs_non_synchronises_ignores(rover):
    image = publier_image(rover)
    publier_pose(rover, image.t)
    publier_mur(rover, image.t - 1)
    rover.bus.publier(S.IMU, Imu(t=image.t - 1, accel_g=(-0.3, 0, 1),
                                 gyro_rad_s=(0, 0, 0)))
    snapshot = rover.outils.snapshot_image()
    mesure = snapshot.mesurer_objet("gauche", 0.5, 0.7, categorie="porte")
    assert mesure is not None and "lidar" not in mesure["source"]
    # Le tangage d'une ancienne mesure IMU n'est pas appliqué à cette frame.
    from fieldmind.perception.projection import ModeleCamera

    attendu = ModeleCamera(rover.profil, "gauche").pixel_vers_sol(0.5, 0.7)
    assert snapshot.pixel_vers_sol("gauche", 0.5, 0.7) == pytest.approx(attendu)


def test_profondeur_et_plan_du_sol_copies(rover):
    assert rover.perception is not None
    image = publier_image(rover)
    publier_pose(rover, image.t)
    stereo = rover.perception.stereo
    geo = stereo._preparer(640, 360)
    profondeur = np.full((geo.hauteur, geo.largeur), 2.0, dtype=np.float32)
    plan = np.array([0.0, 0.0, 0.0])
    with stereo._verrou:
        stereo._derniere = (geo, profondeur)
        stereo._sol_lisse = plan
    rover.bus.publier(S.OBSTACLES_STEREO, Obstacles(
        t=image.t, source="stereo", points=np.empty((0, 2), dtype=np.float32)))
    snapshot = rover.outils.snapshot_image()
    point = snapshot.pixel_vers_sol("gauche", 0.5, 0.6)
    mesure = snapshot.mesurer_objet("gauche", 0.5, 0.6, (0.4, 0.4, 0.6, 0.7), "bloc")
    with stereo._verrou:
        profondeur[:] = 8.0
        plan[:] = [0, 0, 1]
    assert snapshot.pixel_vers_sol("gauche", 0.5, 0.6) == pytest.approx(point)
    assert snapshot.mesurer_objet("gauche", 0.5, 0.6, (0.4, 0.4, 0.6, 0.7), "bloc") == mesure
    assert rover.pixel_vers_sol("gauche", 0.5, 0.6) != pytest.approx(point)


async def test_vision_reseau_lent_garde_lidar_pose_et_image_capturees(rover):
    image = publier_image(rover)
    publier_pose(rover, image.t)
    publier_mur(rover, image.t)
    async with faux_serveur() as fs:
        # La réponse arrive après le délai de fraîcheur normal du lidar (0,8 s).
        fs.delai_s = 1.05

        def repondre(_corps):
            publier_pose(rover, time.monotonic(), Pose2D(8, 9, 0))
            publier_mur(rover, time.monotonic(), distance=4)
            publier_image(rover, jpeg=b"image suivante")
            return {"content": json.dumps([
                {"point": [490, 523], "box_2d": [100, 50, 950, 950],
                 "label": "mur blanc", "categorie": "mur"},
                {"point": [750, 800], "label": "porte blanche", "categorie": "porte"},
            ])}

        fs.chat = repondre
        config = config_essai(fs.base)
        vision = Vision(rover.bus, rover.profil, config, rover.outils)
        try:
            message = await vision.analyser(silencieux=True)
        finally:
            await vision.inference.fermer()
            await config.passerelle().fermer()
    assert message["geometrie_valide"] is True
    assert message["pose_image"] == {"x": 1, "y": 2, "cap": math.pi / 2}
    assert message["image_t"] == image.t
    assert base64.b64decode(message["image"]) == image.jpeg
    mur, porte = message["points"]
    assert mur["source"] == "lidar" and mur["sol"][0] < 2.4
    assert mur["categorie"] == "mur" and porte["categorie"] == "porte"
    assert mur["pose_odom"] == [1, 2, math.pi / 2]
    objets = rover.monde_semantique.resume()["objets"]
    mur_monde = next(o for o in objets if o["label"] == "mur blanc")
    assert mur_monde["x"] == pytest.approx(1 - mur["sol"][1], abs=0.001)
    assert mur_monde["y"] == pytest.approx(2 + mur["sol"][0], abs=0.001)
    assert all(o["x"] < 4 for o in objets)


async def test_vision_sans_pose_garde_label_sans_placement(rover):
    publier_image(rover)
    async with faux_serveur() as fs:
        fs.chat = lambda corps: {"content": '[{"point":[700,500],"label":"porte"}]'}
        config = config_essai(fs.base)
        vision = Vision(rover.bus, rover.profil, config, rover.outils)
        try:
            points = await vision.chercher("porte")
        finally:
            await vision.inference.fermer()
            await config.passerelle().fermer()
    assert points[0]["label"] == "porte" and points[0]["sol"] is None
    assert vision.dernier["pose_image"] is None
    assert vision.dernier["geometrie_valide"] is False
    assert not rover.monde_semantique.resume()["objets"]


async def test_resultat_arrive_apres_reset_ne_repeuple_pas_la_carte(rover):
    image = publier_image(rover)
    publier_pose(rover, image.t)
    async with faux_serveur() as fs:
        def repondre(_corps):
            rover.remise_a_zero_odom()
            publier_pose(rover, time.monotonic(), Pose2D())
            return {"content": '[{"point":[700,500],"label":"porte"}]'}

        fs.chat = repondre
        config = config_essai(fs.base)
        vision = Vision(rover.bus, rover.profil, config, rover.outils)
        try:
            await vision.chercher("porte")
        finally:
            await vision.inference.fermer()
            await config.passerelle().fermer()
    assert not rover.monde_semantique.resume()["objets"]


async def test_image_perimee_ne_declenche_pas_inference(rover):
    publier_image(rover, time.monotonic() - 2)
    async with faux_serveur() as fs:
        config = config_essai(fs.base)
        vision = Vision(rover.bus, rover.profil, config, rover.outils)
        try:
            with pytest.raises(ErreurIA, match="pas d'image"):
                await vision.analyser(silencieux=True)
        finally:
            await vision.inference.fermer()
            await config.passerelle().fermer()
        assert not fs.requetes


async def test_outils_legacy_conservent_schema_points_exact():
    # Les intégrations sans snapshots continuent de recevoir les quatre champs historiques.
    outils = FauxOutils()
    async with faux_serveur() as fs:
        fs.chat = lambda corps: {"content": '[{"point":[700,500],"label":"porte"}]'}
        config = config_essai(fs.base)
        from fieldmind.bus import Bus

        vision = Vision(Bus(), Profil.charger(reglages=None), config, outils)
        try:
            points = await vision.chercher("porte")
        finally:
            await vision.inference.fermer()
            await config.passerelle().fermer()
    assert points == [{"u": 0.5, "v": 0.7, "label": "porte", "sol": [2.1, -0.6]}]


def test_schema_perception_architecture_ne_promet_pas_des_pieces_cachees():
    detections = lire_detections([
        {"point": [700, 500], "label": "porte", "categorie": "porte"},
        {"point": [700, 600], "label": "mur", "categorie": "mur"},
        {"point": [700, 700], "label": "maison", "categorie": "batiment"},
    ])
    assert [o["categorie"] for o in detections] == ["porte", "mur", "batiment"]
    consigne = consigne_analyse_scene()
    assert "hidden passages" in consigne and "distances" in consigne


@pytest.mark.parametrize("pose", [Pose2D(float("nan"), 0, 0),
                                  Pose2D(0, float("inf"), 0),
                                  Pose2D(0, 0, float("nan"))])
def test_pose_non_finie_ne_permet_pas_placement(rover, pose):
    image = publier_image(rover)
    publier_pose(rover, image.t, pose)
    assert rover.outils.snapshot_image().pose is None


def test_reset_attend_une_pose_du_nouveau_repere(rover):
    image = publier_image(rover)
    publier_pose(rover, image.t, Pose2D(7, 9, 0))
    rover.remise_a_zero_odom()
    nouvelle = publier_image(rover)
    # Le bus garde la dernière valeur ODOM ; elle précède la remise à zéro et doit être ignorée.
    assert rover.outils.snapshot_image().pose is None
    publier_pose(rover, nouvelle.t, Pose2D())
    assert rover.outils.snapshot_image().pose == Pose2D()
