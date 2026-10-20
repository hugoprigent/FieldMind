"""Vision : points [y, x] 0–1000 → u, v normalisés, projection au sol, message `vision`."""

from __future__ import annotations

import base64
from typing import Any

import pytest
from test_ia_faux_serveur import Ecoute, FauxOutils, config_essai, faux_serveur

from fieldmind.bus import Bus
from fieldmind.config import FICHIER_PROFIL, Profil
from fieldmind.ia.fournisseurs import ErreurIA
from fieldmind.ia.vision import (
    Vision,
    consigne_pointage,
    lire_detections,
    lire_points,
    preparer_image,
)
from fieldmind.types import ImageCamera


def test_points_yx_vers_uv() -> None:
    points = lire_points([{"point": [720, 610], "label": "chaise rouge"},
                          {"point": [0, 1000], "label": "coin"}])
    assert points == [(0.61, 0.72, "chaise rouge"), (1.0, 0.0, "coin")]


def test_points_xy_pour_qwen() -> None:
    assert lire_points([{"point": [610, 720], "label": "chaise"}], "xy_1000") == [
        (0.61, 0.72, "chaise")]


def test_points_mal_formes_ecartes_un_a_un() -> None:
    points = lire_points({"points": [
        {"point": [500, 500], "label": "bon"},
        {"point": [1500, 200], "label": "hors image"},
        {"point": [100], "label": "incomplet"},
        {"point": ["a", "b"]},
        {"point": [float("nan"), 3]},
        "pas un objet",
        {"point": [1003, -2], "label": "au bord"},  # tolérance de 5 : ramené sur le bord
        {"point": {"y": 250, "x": 750}},
    ]}, label_defaut="objet")
    assert points == [(0.5, 0.5, "bon"), (0.0, 1.0, "au bord"), (0.75, 0.25, "objet")]


def test_au_plus_dix_points() -> None:
    assert len(lire_points([{"point": [i, i]} for i in range(30)])) == 10


def test_consigne_de_pointage_recommandee() -> None:
    texte = consigne_pointage("chaise rouge")
    assert "Point to" in texte and "no more than 10 items" in texte
    assert '[{"point": [y, x], "label": <label1>}, ...]' in texte
    assert "[y, x] format normalized to 0-1000" in texte
    assert "[x, y]" in consigne_pointage("chaise", "xy_1000")


def profil_rover() -> Profil:
    return Profil.charger(FICHIER_PROFIL, reglages=None)


async def test_chercher_projette_et_publie() -> None:
    async with faux_serveur() as fs:
        fs.chat = lambda corps: {"content": '```json\n[{"point": [720, 610], "label": '
                                            '"chaise rouge"}, {"point": [300, 100], '
                                            '"label": "chaise"}]\n```'}
        bus, outils = Bus(), FauxOutils()
        outils.sol = (2.1, -0.6)
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        vision = Vision(bus, profil_rover(), config, outils)
        points = await vision.chercher("chaise rouge", "gauche")
        await config.passerelle().fermer()
    assert points == [
        {"u": 0.61, "v": 0.72, "label": "chaise rouge", "sol": [2.1, -0.6]},
        {"u": 0.1, "v": 0.3, "label": "chaise", "sol": [2.1, -0.6]},
    ]
    assert outils.projections == [("gauche", 0.61, 0.72), ("gauche", 0.1, 0.3)]
    message = ecoute.vision[-1]
    assert message["type"] == "vision" and message["demande"] == "chercher"
    assert message["objet"] == "chaise rouge" and message["camera"] == "gauche"
    assert message["modele"] == "gemini-robotics-er-2-preview"
    assert base64.b64decode(message["image"]) == outils.images["gauche"].jpeg
    assert message["description"] == ("Je vois 2 « chaise rouge », le plus proche à 2,1 m "
                                      "devant, 0,6 m à droite.")
    assert ecoute.textes()[-1] == message["description"]
    # requête : image en data URL JPEG + consigne de pointage, réflexion « medium »
    corps = fs.requetes[0][1]
    assert corps["model"] == "gemini-robotics-er-2-preview"
    assert corps["reasoning_effort"] == "medium"
    contenu = corps["messages"][0]["content"]
    assert contenu[0]["image_url"]["url"].startswith("data:image/jpeg;base64,")
    assert "chaise rouge" in contenu[1]["text"]


async def test_chercher_objet_absent() -> None:
    async with faux_serveur() as fs:
        fs.chat = lambda corps: {"content": "[]"}
        bus, outils = Bus(), FauxOutils()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        points = await Vision(bus, profil_rover(), config, outils).chercher("girafe")
        await config.passerelle().fermer()
    assert points == []
    assert ecoute.vision[-1]["points"] == []
    assert ecoute.textes()[-1] == "Je ne vois pas de « girafe » (caméra gauche)."


async def test_point_sans_projection_au_sol() -> None:
    async with faux_serveur() as fs:
        fs.chat = lambda corps: {"content": '[{"point": [100, 500], "label": "lampe"}]'}
        bus, outils = Bus(), FauxOutils()
        outils.sol = None  # au-dessus de l'horizon
        config = config_essai(fs.base)
        points = await Vision(bus, profil_rover(), config, outils).chercher("lampe")
        await config.passerelle().fermer()
    assert points == [{"u": 0.5, "v": 0.1, "label": "lampe", "sol": None}]


async def test_reponse_illisible_et_image_absente() -> None:
    async with faux_serveur() as fs:
        fs.chat = lambda corps: {"content": "Je ne sais pas."}
        bus, outils = Bus(), FauxOutils()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        vision = Vision(bus, profil_rover(), config, outils)
        with pytest.raises(ErreurIA, match="illisible"):
            await vision.chercher("chaise")
        with pytest.raises(ErreurIA, match="pas d'image de la caméra droite"):
            await vision.decrire("droite")
        await config.passerelle().fermer()
    assert [m["role"] for m in ecoute.chat] == ["systeme", "systeme"]
    assert ecoute.vision == []


async def test_decrire() -> None:
    async with faux_serveur() as fs:
        fs.chat = lambda corps: {"content": "Un couloir **dégagé**, une chaise à droite."}
        bus = Bus()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        message: dict[str, Any] = await Vision(bus, profil_rover(), config,
                                               FauxOutils()).decrire()
        await config.passerelle().fermer()
    assert message["description"] == "Un couloir dégagé, une chaise à droite."
    assert message["demande"] == "decrire" and message["points"] == []
    assert ecoute.vision == [message]


async def test_vision_profil_gx10() -> None:
    """Profil gx10 : « chercher » passe par le modèle incarné RoboBrain 2.5 (consigne et
    réponse `[(x, y)]` propres au modèle), pas par le modèle de vision."""
    async with faux_serveur() as fs:
        fs.chat = lambda corps: {"content": "[(500, 250)]"}
        config = config_essai(fs.base, env={}, profil="gx10")
        vision = Vision(Bus(), profil_rover(), config, FauxOutils())
        points = await vision.chercher("porte")
        await vision.inference.fermer()
        await config.passerelle().fermer()
    assert points[0]["u"] == 0.5 and points[0]["v"] == 0.25
    chat = [corps for chemin, corps in fs.requetes if chemin == "/v1/chat/completions"]
    assert chat[0]["model"] == "BAAI/RoboBrain2.5-4B"
    assert "[(x, y)]" in chat[0]["messages"][0]["content"][1]["text"]


def test_image_reduite_a_640_px() -> None:
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    image = np.zeros((720, 1280, 3), np.uint8)
    image[:, 640:] = (0, 0, 255)
    ok, jpeg = cv2.imencode(".jpg", image)
    assert ok
    reduite, largeur, hauteur = preparer_image(
        ImageCamera(t=0.0, camera="gauche", jpeg=jpeg.tobytes(), largeur=1280, hauteur=720), 640)
    assert (largeur, hauteur) == (640, 360)
    assert cv2.imdecode(np.frombuffer(reduite, np.uint8), cv2.IMREAD_COLOR).shape == (360, 640, 3)


def test_lire_detections_avec_boite_et_categorie() -> None:
    detections = lire_detections([
        {"point": [700, 500], "box_2d": [400, 420, 750, 580],
         "label": "caisse bleue", "category": "bloc"},
        {"box_2d": [200, 100, 800, 200], "label": "poteau", "category": "pilier"},
    ])
    assert len(detections) == 2
    assert detections[0]["u"] == 0.5 and detections[0]["v"] == 0.7
    assert detections[0]["boite"] == (0.42, 0.4, 0.58, 0.75)
    assert detections[0]["categorie"] == "bloc"
    assert detections[1]["u"] == pytest.approx(0.15)
    assert detections[1]["v"] == pytest.approx(0.8)


async def test_analyser_fusion_lidar_stereo() -> None:
    async with faux_serveur() as fs:
        fs.chat = lambda corps: {"content": (
            '[{"point": [680, 520], "box_2d": [420, 440, 720, 600], '
            '"label": "caisse plastique", "category": "bloc"}]'
        )}
        bus, outils = Bus(), FauxOutils()
        outils.mesurer_objet = lambda cam, u, v, boite=None, categorie=None: {  # type: ignore[attr-defined]
            "sol": [1.85, -0.12],
            "hauteur": 0.48,
            "largeur": 0.55,
            "longueur": 0.42,
            "cap": 0.1,
            "type_3d": "bloc",
            "source": "lidar+stereo",
        }
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        vision = Vision(bus, profil_rover(), config, outils)
        message = await vision.analyser("gauche")
        await config.passerelle().fermer()
    points = message["points"]
    assert len(points) == 1
    pt = points[0]
    assert pt["label"] == "caisse plastique"
    assert pt["sol"] == [1.85, -0.12]
    assert pt["hauteur"] == 0.48 and pt["largeur"] == 0.55
    assert pt["type_3d"] == "bloc" and pt["source"] == "lidar+stereo"
    assert ecoute.vision[-1]["demande"] == "analyser"

