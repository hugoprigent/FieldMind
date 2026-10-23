"""Vision terrain : descriptions précises et reprise JSON sur la même capture."""

from __future__ import annotations

import base64
from typing import Any

import pytest
from test_ia_faux_serveur import Ecoute, FauxOutils, config_essai, faux_serveur

from fieldmind.bus import Bus
from fieldmind.config import Profil
from fieldmind.ia.fournisseurs import ErreurIA
from fieldmind.ia.vision import (
    Vision,
    consigne_analyse_scene,
    lire_cible_confirmee,
    lire_reponse_scene,
)
from fieldmind.types import ImageCamera

POINT = '{"point":[700,500],"label":"caisse bleue","categorie":"bloc"}'


@pytest.mark.parametrize("texte", [
    '```json\n{"points":[' + POINT + ']}\n```',
    'Voici [une annotation] puis {"points":[' + POINT + ']} fin.',
    '{"result":{"points":[' + POINT + ']}}',
    '{"points":' + POINT + '}',
    '{"points":{"objet_1":' + POINT + '}}',
    '{"detections":[' + POINT + ']}',
    POINT,
])
def test_formats_recuperables_sans_inventer_de_geometrie(texte: str) -> None:
    _donnees, detections = lire_reponse_scene(texte)
    assert len(detections) == 1
    assert detections[0]["label"] == "caisse bleue"
    assert detections[0]["u"] == 0.5 and detections[0]["v"] == 0.7


@pytest.mark.parametrize("texte", ['{"points":[]}', '[]'])
def test_absence_explicite_objets_valide(texte: str) -> None:
    assert lire_reponse_scene(texte)[1] == []


@pytest.mark.parametrize("texte", ['{"resume":"rien"}', '{"points":[{"label":"caisse"}]}',
                                  'Je ne sais pas', '{"points":['])
def test_malformation_declenche_reprise_pas_faux_resultat_vide(texte: str) -> None:
    with pytest.raises(ValueError):
        lire_reponse_scene(texte)


def test_prompt_distingue_couverture_sol_et_objets_mission() -> None:
    prompt = consigne_analyse_scene()
    assert "grass blades" in prompt and "continuous lawn" in prompt
    assert "real bushes" in prompt and "colored boxes" in prompt
    assert "Preserve object colors" in prompt


async def test_une_reprise_json_meme_image_pose_et_publication_unique() -> None:
    async with faux_serveur() as fs:
        bus, outils = Bus(), FauxOutils()
        original = outils.images["gauche"]
        ecoute = Ecoute(bus)

        def repondre(_corps: dict[str, Any]) -> dict[str, str]:
            if len(fs.requetes) == 1:
                outils.images["gauche"] = ImageCamera(
                    t=2.0, camera="gauche", jpeg=b"nouvelle capture", largeur=640, hauteur=360)
                return {"content": "Le résultat est : {points:"}
            return {"content": '{"points":[' + POINT + ']}'}

        fs.chat = repondre
        config = config_essai(fs.base)
        vision = Vision(bus, Profil.charger(reglages=None), config, outils)
        try:
            message = await vision.analyser(silencieux=True)
        finally:
            await config.passerelle().fermer()
    assert len(fs.requetes) == 2
    contenus = [r[1]["messages"][0]["content"] for r in fs.requetes]
    assert contenus[0][0]["image_url"] == contenus[1][0]["image_url"]
    assert "previous response had invalid JSON" in contenus[1][1]["text"]
    assert base64.b64decode(message["image"]) == original.jpeg
    assert message["image_t"] == original.t
    assert len(ecoute.vision) == 1 and not outils.plans


async def test_deux_reponses_invalides_erreur_sans_boucle_ni_point() -> None:
    async with faux_serveur() as fs:
        fs.chat = lambda _corps: {"content": "JSON illisible"}
        bus, outils = Bus(), FauxOutils()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        try:
            with pytest.raises(ErreurIA, match="illisible après une reprise"):
                await Vision(bus, Profil.charger(reglages=None), config, outils).analyser(
                    silencieux=True)
        finally:
            await config.passerelle().fermer()
    assert len(fs.requetes) == 2 and ecoute.vision == [] and outils.plans == []


@pytest.mark.parametrize("point", ['{"point":[true,500],"label":"caisse"}',
                                   '{"box_2d":[false,200,700,800],"label":"caisse"}'])
def test_boolean_ne_devient_pas_coordonnees(point: str) -> None:
    with pytest.raises(ValueError):
        lire_reponse_scene('[' + point + ']')


@pytest.mark.parametrize("confirmation", [
    {"present": False, "correspondance": False},
    {"present": True, "correspondance": False},
    {"present": False, "correspondance": True},
])
def test_grounding_sans_confirmation_presence_refuse(confirmation: dict[str, bool]) -> None:
    donnees = confirmation | {"points": [{"point": [500, 700], "label": "caisse rouge"}]}
    assert lire_cible_confirmee(donnees, "caisse rouge", "xy_1000") == []


@pytest.mark.parametrize("label", ["caisse bleue", "caisse", "boîte verte"])
def test_couleur_cible_non_vue_ne_devient_pas_label_objectif(label: str) -> None:
    donnees = {"present": True, "correspondance": True,
               "points": [{"point": [500, 700], "label": label}]}
    assert lire_cible_confirmee(donnees, "caisse rouge", "xy_1000") == []


@pytest.mark.parametrize("donnees", [[(500, 700)], {"present": "true", "correspondance": True},
                                     {"points": [{"point": [500, 700]}]}])
def test_boolean_strict_obligatoire_recherche_gx10(donnees: Any) -> None:
    with pytest.raises(ValueError):
        lire_cible_confirmee(donnees, "caisse rouge", "xy_1000")


async def test_gx10_cible_absente_ou_autre_couleur_jamais_memorisee() -> None:
    async with faux_serveur() as fs:
        bus, outils = Bus(), FauxOutils()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base, env={}, profil="gx10")
        vision = Vision(bus, Profil.charger(reglages=None), config, outils)
        try:
            for reponse in (
                '{"present":false,"correspondance":false,"points":[]}',
                '{"present":true,"correspondance":true,"points":['
                '{"point":[500,700],"label":"caisse bleue"}]}',
            ):
                fs.chat = lambda _corps, texte=reponse: {"content": texte}
                assert await vision.chercher("caisse rouge") == []
        finally:
            await vision.inference.fermer()
            await config.passerelle().fermer()
    assert all(v["points"] == [] for v in ecoute.vision)
    assert all(r[1]["model"] == "nvidia/Qwen3.6-35B-A3B-NVFP4" for r in fs.requetes)
    assert not outils.projections and not outils.plans


async def test_gx10_recherche_garde_capture_et_label_reel() -> None:
    async with faux_serveur() as fs:
        bus, outils = Bus(), FauxOutils()
        original = outils.images["gauche"]

        def repondre(_corps: Any) -> dict[str, str]:
            outils.images["gauche"] = ImageCamera(t=2.0, camera="gauche", jpeg=b"nouvelle image",
                                                  largeur=640, hauteur=360)
            return {"content": '{"present":true,"correspondance":true,"points":['
                               '{"point":[500,700],"label":"caisse plastique rouge"}]}'}

        fs.chat = repondre
        config = config_essai(fs.base, env={}, profil="gx10")
        ecoute = Ecoute(bus)
        vision = Vision(bus, Profil.charger(reglages=None), config, outils)
        try:
            points = await vision.chercher("caisse rouge")
        finally:
            await vision.inference.fermer()
            await config.passerelle().fermer()
    assert points[0]["label"] == "caisse plastique rouge"
    assert base64.b64decode(ecoute.vision[0]["image"]) == original.jpeg
    assert ecoute.vision[0]["image_t"] == original.t
    assert len(fs.requetes) == 1 and not outils.plans


async def test_gx10_reprise_liste_point_2d_vers_confirmation_schema_meme_capture() -> None:
    async with faux_serveur() as fs:
        bus, outils = Bus(), FauxOutils()
        ecoute = Ecoute(bus)
        original = outils.images["gauche"]

        def repondre(_corps: Any) -> dict[str, str]:
            if len(fs.requetes) == 1:
                outils.images["gauche"] = ImageCamera(t=2.0, camera="gauche", jpeg=b"autre image",
                                                      largeur=640, hauteur=360)
                return {"content": '```json\n[{"point_2d":[690,447],"label":"sac jaune"}]\n```'}
            return {"content": '{"present":true,"correspondance":true,"points":['
                               '{"point_2d":[690,447],"label":"sac jaune"}]}'}

        fs.chat = repondre
        config = config_essai(fs.base, env={}, profil="gx10")
        vision = Vision(bus, Profil.charger(reglages=None), config, outils)
        try:
            resultat = await vision.chercher("sac jaune")
        finally:
            await vision.inference.fermer()
            await config.passerelle().fermer()
    assert len(fs.requetes) == 2
    assert resultat[0]["u"] == 0.69 and resultat[0]["v"] == 0.447
    assert resultat[0]["label"] == "sac jaune"
    corps = [r[1] for r in fs.requetes]
    assert all(c["response_format"]["type"] == "json_schema" for c in corps)
    assert set(corps[0]["response_format"]["json_schema"]["schema"]["required"]) == {
        "present", "correspondance", "points"}
    assert corps[1]["messages"][0]["role"] == "system"
    premier = corps[0]["messages"][0]["content"]
    second = corps[1]["messages"][1]["content"]
    assert premier == second  # texte et JPEG originaux conservés
    assert base64.b64decode(ecoute.vision[0]["image"]) == original.jpeg
    assert ecoute.vision[0]["image_t"] == original.t
    assert len(ecoute.vision) == 1 and not outils.plans


async def test_gx10_deux_listes_sans_boolean_ne_confirment_jamais_cible() -> None:
    async with faux_serveur() as fs:
        fs.chat = lambda _corps: {
            "content": '```json\n[{"point_2d":[690,447],"label":"sac jaune"}]\n```'}
        bus, outils = Bus(), FauxOutils()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base, env={}, profil="gx10")
        vision = Vision(bus, Profil.charger(reglages=None), config, outils)
        try:
            with pytest.raises(ErreurIA, match="illisible après une reprise"):
                await vision.chercher("sac jaune")
        finally:
            await vision.inference.fermer()
            await config.passerelle().fermer()
    assert len(fs.requetes) == 2
    assert ecoute.vision == [] and outils.projections == [] and outils.plans == []
