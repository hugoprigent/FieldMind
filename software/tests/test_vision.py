"""Tests de la vision : géométrie de la caméra, lecture des réponses du modèle, client OpenRouter
(gratuit seulement), couche vision de la carte, détecteur de la simulation et chaîne complète."""

from __future__ import annotations

import asyncio
import base64
import io
import json
import math
import time
from pathlib import Path
from typing import Any

import httpx
import pytest
from fastapi.testclient import TestClient
from PIL import Image as ImagePil

from services.api.config import Reglages
from services.api.serveur import ModeleRefuse, _client_openrouter, app
from services.cerveau.cerveau import Cerveau
from services.mission_planner.api_openai import (
    ClientCompatibleOpenAI,
    EchecFournisseur,
    InfosModele,
    extraire_json,
)
from services.perception.camera import ModeleCamera
from services.perception.observations import CoucheVision, placer_objets
from services.perception.vision import (
    AnalyseVision,
    DemandeVision,
    FournisseurVisionOpenAI,
    ObjetVu,
    ResultatVision,
    lire_analyse,
)
from services.protocole.fiche_rover import FicheRover
from services.protocole.messages import Image
from services.protocole.scenario import ScenarioPublic
from simulation.banc_essai import BancEssai
from simulation.camera_simulee import CameraSimulee
from simulation.monde import charger_monde
from simulation.robot_simule import RobotSimule
from simulation.vision_oracle import DetecteurVerite

RACINE = Path(__file__).resolve().parents[1]
SCENARIO_CHEMIN = RACINE / "simulation" / "scenarios" / "foret_demo.json"
FICHE = FicheRover.charger(RACINE / "config" / "rover_fieldmind.json")
SCENARIO = ScenarioPublic.charger(SCENARIO_CHEMIN)
CAMERA = FICHE.capteurs.camera
assert CAMERA is not None


def _demande(x: float = 0.0, y: float = 0.0, cap: float = 0.0, t: float = 1.0) -> DemandeVision:
    return DemandeVision(1, t, "", 640, 360, x, y, cap, "operateur")


# ------------------------------------------------------------------ géométrie


@pytest.mark.parametrize(("px", "py"), [(3.0, 0.0), (5.0, 1.5), (2.0, -1.0), (8.0, 2.0)])
def test_projection_puis_retour_au_sol(px: float, py: float) -> None:
    camera = ModeleCamera(CAMERA)
    x, y, cap = 2.0, -3.0, 0.7
    c, s = math.cos(cap), math.sin(cap)
    monde = (x + c * px - s * py, y + s * px + c * py)
    u, v, _ = camera.projeter(*monde, 0.0, x, y, cap)  # type: ignore[misc]
    sol = camera.point_au_sol(u, v, x, y, cap)
    assert sol is not None
    assert math.dist((sol.x, sol.y), monde) < 1e-6
    # L'incertitude croît avec la distance (caméra à 27 cm du sol).
    proche = camera.point_au_sol(u, 0.9, x, y, cap)
    assert proche is not None
    assert proche.incertitude < sol.incertitude + 1e-9


def test_au_dessus_de_l_horizon_pas_de_position() -> None:
    assert ModeleCamera(CAMERA).point_au_sol(0.5, 0.05, 0, 0, 0) is None


# ------------------------------------------------------------------ réponses du modèle


def test_extraction_json_tolerante() -> None:
    assert extraire_json('```json\n{"a": 1}\n```') == {"a": 1}
    assert extraire_json('Voici : {"a": [1, 2]} fin.') == {"a": [1, 2]}
    with pytest.raises(ValueError, match="Expecting value"):
        extraire_json("pas de json")


def test_objets_mal_formes_ecartes_un_a_un() -> None:
    analyse = lire_analyse(
        {
            "resume": "x" * 1000,
            "objets": [
                {
                    "categorie": "tronc_couche",
                    "boite": [64, 180, 576, 500],
                    "bloque_passage": True,
                    "confiance": 0.9,
                },
                {
                    "categorie": "licorne",
                    "boite": [0, 0, 10, 10],
                    "bloque_passage": False,
                    "confiance": 1,
                },
                {
                    "categorie": "rocher",
                    "boite": [5, 5, 5, 5],
                    "bloque_passage": True,
                    "confiance": 0.5,
                },
                {"categorie": "arbre", "boite": "n'importe quoi"},
            ],
        },
        640,
        360,
    )
    assert len(analyse.resume) == 400
    assert [o.categorie for o in analyse.objets] == ["tronc_couche"]
    # Pixels de l'image 640 × 360 ramenés sur 0–1000, bornés à l'image.
    assert analyse.objets[0].boite == (100, 500, 900, 1000)


# ------------------------------------------------------------------ client OpenRouter


def _client(reponse: httpx.Response, capture: list[dict[str, Any]]) -> ClientCompatibleOpenAI:
    def gerer(requete: httpx.Request) -> httpx.Response:
        capture.append({"entetes": dict(requete.headers), "corps": json.loads(requete.content)})
        return reponse

    return ClientCompatibleOpenAI(
        "https://openrouter.test/api/v1",
        "cle-de-test",
        "stealth/space-bunny-alpha",
        5.0,
        openrouter=True,
        modeles_secours=("google/gemma-4-31b-it:free",),
        mode_json="objet",
        transport=httpx.MockTransport(gerer),
    )


def test_requete_openrouter_gratuite_avec_image() -> None:
    contenu = json.dumps(
        {
            "resume": "Piste dégagée.",
            "objets": [
                {
                    "categorie": "tronc_couche",
                    "boite": [192, 216, 448, 252],
                    "bloque_passage": True,
                    "confiance": 0.8,
                }
            ],
        }
    )
    capture: list[dict[str, Any]] = []
    client = _client(
        httpx.Response(
            200,
            json={
                "model": "google/gemma-4-31b-it:free",
                "choices": [{"message": {"content": contenu}}],
            },
        ),
        capture,
    )
    fournisseur = FournisseurVisionOpenAI(client, "consigne", 2000)
    demande = DemandeVision(3, 1.0, "QUJD", 640, 360, 0, 0, 0, "vigilance")
    analyse = asyncio.run(fournisseur.analyser(demande))

    assert analyse.objets[0].categorie == "tronc_couche"
    assert analyse.objets[0].boite == pytest.approx((300, 600, 700, 700))  # pixels → 0–1000
    assert fournisseur.dernier_modele == "google/gemma-4-31b-it:free"
    corps = capture[0]["corps"]
    assert corps["provider"]["max_price"] == {"prompt": 0, "completion": 0, "image": 0}
    assert corps["reasoning"]["effort"] == "low"
    assert corps["response_format"] == {"type": "json_object"}
    assert "Schéma JSON" in corps["messages"][0]["content"]
    assert "640 × 360 pixels" in corps["messages"][1]["content"][0]["text"]
    image = corps["messages"][1]["content"][1]["image_url"]["url"]
    assert image == "data:image/jpeg;base64,QUJD"
    assert capture[0]["entetes"]["authorization"] == "Bearer cle-de-test"


@pytest.mark.parametrize(
    ("reponse", "message", "attente"),
    [
        (httpx.Response(429, headers={"retry-after": "30"}), "limite de requêtes", 30.0),
        (httpx.Response(402), "gratuit seulement", None),
        (httpx.Response(401), "clé d'API refusée", None),
        (
            httpx.Response(
                200, json={"choices": [{"message": {"content": ""}, "finish_reason": "length"}]}
            ),
            "tronquée",
            None,
        ),
        (httpx.Response(200, json={"error": {"code": 502}}), "fournisseur", None),
    ],
)
def test_echecs_openrouter(reponse: httpx.Response, message: str, attente: float | None) -> None:
    with pytest.raises(EchecFournisseur, match=message) as erreur:
        asyncio.run(_client(reponse, []).completer([], {}, "x", 100))
    assert erreur.value.reessayer_dans_s == attente


def test_seuls_les_modeles_gratuits_sont_acceptes() -> None:
    infos = {
        "gratuit/vision": InfosModele(gratuit=True, images=True, schema_json=True),
        "gratuit/texte": InfosModele(gratuit=True, images=False, schema_json=False),
        "payant/vision": InfosModele(gratuit=False, images=True, schema_json=True),
    }

    def client(
        modele: str, secours: tuple[str, ...] = (), images: bool = True
    ) -> ClientCompatibleOpenAI:
        return _client_openrouter("https://x/api/v1", modele, "cle", 5.0, secours, infos, images)

    with pytest.raises(ModeleRefuse, match="payant"):
        client("payant/vision")
    with pytest.raises(ModeleRefuse, match="images"):
        client("gratuit/texte")
    with pytest.raises(ModeleRefuse, match="absent"):
        client("inconnu/modele")
    retenu = client("gratuit/vision", ("payant/vision", "gratuit/texte", "inconnu/x"))
    assert retenu.modeles_secours == ()
    assert retenu.mode_json == "schema"
    assert client("gratuit/texte", ("gratuit/vision",), images=False).modeles_secours == (
        "gratuit/vision",
    )


def test_reglages_openrouter_automatique(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("FIELDMIND_LLM_PROVIDER", raising=False)
    monkeypatch.delenv("FIELDMIND_VISION_PROVIDER", raising=False)
    monkeypatch.setenv("OPENROUTER_API_KEY", "cle")
    cfg = Reglages(_env_file=None)  # type: ignore[call-arg]
    assert cfg.fournisseur_chat == "openrouter"
    assert cfg.modele_chat == "qwen/qwen3.8-27b:free"
    assert cfg.fournisseur_vision == "openrouter"
    assert cfg.modele_vision == "qwen/qwen3.8-27b:free"
    monkeypatch.delenv("OPENROUTER_API_KEY")
    cfg = Reglages(_env_file=None)  # type: ignore[call-arg]
    assert cfg.fournisseur_chat == "aucun"
    assert cfg.fournisseur_vision == "simulation"


# ------------------------------------------------------------------ couche vision


def _analyse_tronc(
    boite: tuple[float, float, float, float], confiance: float = 0.9
) -> AnalyseVision:
    return AnalyseVision(
        resume="",
        objets=[
            ObjetVu(categorie="tronc_couche", boite=boite, bloque_passage=True, confiance=confiance)
        ],
    )


def test_obstacles_bas_seulement_et_vues_accumulees() -> None:
    camera = ModeleCamera(CAMERA)
    couche = CoucheVision(SCENARIO.limites, 0.1)
    # Caméra à 27 cm du sol, 90° de champ : v = 0,384 → ~6 m ; v = 0,424 → ~3 m.
    loin = placer_objets(_analyse_tronc((400, 360, 600, 384)), _demande(), camera, 1)
    assert loin[0].dans_carte
    assert 4 < loin[0].distance < 12
    assert len(couche.fusionner(loin)) == 1
    assert couche.masque.any()
    assert couche.fusionner(loin) == [], "un doublon exact est ignoré"

    # Même tronc revu de plus près : la vue nettement plus précise remplace la vue lointaine
    # qu'elle recoupe.
    proche = placer_objets(
        _analyse_tronc((350, 395, 650, 424)), _demande(x=loin[0].distance - 3), camera, 2
    )
    assert len(couche.fusionner(proche)) == 1
    assert [o.id for o in couche.obstacles] == [2]

    # Un rocher (vu par le lidar) ou une détection peu sûre n'entrent pas dans la carte.
    rocher = AnalyseVision(
        resume="",
        objets=[
            ObjetVu(categorie="rocher", boite=(0, 500, 100, 700), bloque_passage=True, confiance=1)
        ],
    )
    assert not placer_objets(rocher, _demande(), camera, 3)[0].dans_carte
    assert not placer_objets(_analyse_tronc((400, 400, 600, 440), 0.2), _demande(), camera, 4)[
        0
    ].dans_carte


def test_deux_troncs_voisins_restent_distincts() -> None:
    camera = ModeleCamera(CAMERA)
    couche = CoucheVision(SCENARIO.limites, 0.1)
    # Deux troncs de 0,7 m à ~3 m, vus nettement depuis deux positions décalées de 3 m.
    a = placer_objets(_analyse_tronc((450, 400, 550, 440)), _demande(y=0.0), camera, 1)
    b = placer_objets(_analyse_tronc((450, 400, 550, 440)), _demande(y=3.0), camera, 2)
    couche.fusionner(a)
    assert len(couche.fusionner(b)) == 1
    assert len(couche.obstacles) == 2


def test_fausse_detection_retiree_par_preuve_negative() -> None:
    cerveau = Cerveau(FICHE, SCENARIO)
    cerveau.vision_disponible = True
    demande = DemandeVision(
        1,
        0.0,
        "",
        640,
        360,
        *(SCENARIO.depart.x, SCENARIO.depart.y, SCENARIO.depart.cap),
        "operateur",
    )
    cerveau._vision_en_cours = demande
    texte = cerveau.sur_resultat_vision(
        demande, ResultatVision(_analyse_tronc((300, 420, 700, 450)), "test", 1.0)
    )
    assert "ajouté(s) à la carte" in texte
    assert cerveau.couche_vision.masque.any()
    # La même zone, nettement visible à 2 m, analysée trois fois sans rien y voir : retirée.
    vide = AnalyseVision(resume="", objets=[])
    for k, attendu in ((2, 1), (3, 1), (4, 0)):
        suivante = DemandeVision(k, 0.5 * k, "", 640, 360, *demande_pose(demande), "vigilance")
        cerveau._vision_en_cours = suivante
        cerveau.sur_resultat_vision(suivante, ResultatVision(vide, "test", 1.0))
        assert len(cerveau.couche_vision.obstacles) == attendu
    assert not cerveau.couche_vision.masque.any()


def test_detection_gardee_si_le_rover_l_atteint_avant_la_reponse() -> None:
    """Le modèle répond en plusieurs secondes : un obstacle vu devant reste un obstacle même si
    le rover est arrivé dessus entre la prise de vue et la réponse."""
    cerveau = Cerveau(FICHE, SCENARIO)
    cerveau.vision_disponible = True
    demande = DemandeVision(1, 0.0, "", 640, 360, 0.0, 0.0, 0.0, "vigilance")
    obs = placer_objets(_analyse_tronc((300, 420, 700, 450)), demande, ModeleCamera(CAMERA), 1)[0]
    # Localisation qui place le rover sur l'obstacle au moment de la réponse.
    cerveau.localisation.pose = lambda: type(  # type: ignore[method-assign]
        "Pose", (), {"x": obs.x, "y": obs.y, "cap": 0.0}
    )()
    cerveau._vision_en_cours = demande
    cerveau.sur_resultat_vision(
        demande, ResultatVision(_analyse_tronc((300, 420, 700, 450)), "t", 1.0)
    )
    assert len(cerveau.couche_vision.obstacles) == 1


def demande_pose(d: DemandeVision) -> tuple[float, float, float]:
    return d.x, d.y, d.cap


def test_emprise_bord_proche_plus_epaisseur() -> None:
    camera = ModeleCamera(CAMERA)
    obs = placer_objets(_analyse_tronc((300, 440, 700, 470)), _demande(), camera, 1)[0]
    # Cap nul : x est la distance vers l'avant. L'emprise couvre la bande d'incertitude du bord
    # proche (± 1,5 % de la hauteur de l'image), plus l'épaisseur du tronc (40 cm).
    contact = ModeleCamera(CAMERA).point_au_sol(0.5, 0.47, 0, 0, 0)
    assert contact is not None
    assert obs.contour[1][0] < contact.x < obs.contour[2][0] - 0.4
    assert obs.contour[2][0] - obs.contour[1][0] < 3.4
    assert obs.contour[0][1] > obs.contour[1][1], "proche gauche à gauche de proche droite"


# ------------------------------------------------------------------ simulation


def _robot_devant_le_tronc(cap_deg: float) -> RobotSimule:
    monde = charger_monde(SCENARIO_CHEMIN)
    robot = RobotSimule(FICHE, monde)
    tronc = monde.troncs_couches[0]
    cap = math.radians(cap_deg)
    # 5 m avant le tronc, dans l'axe de la piste (perpendiculaire au tronc).
    axe = tronc.angle + math.pi / 2
    robot.donnees.qpos[0] = tronc.x - 5 * math.cos(axe)
    robot.donnees.qpos[1] = tronc.y - 5 * math.sin(axe)
    q = axe + cap
    robot.donnees.qpos[3:7] = [math.cos(q / 2), 0, 0, math.sin(q / 2)]
    robot.avancer(0.6)
    return robot


def test_detecteur_de_la_simulation_voit_le_tronc_devant_seulement() -> None:
    for cap_deg, attendu in ((0.0, 1), (180.0, 0)):
        robot = _robot_devant_le_tronc(cap_deg)
        capture = robot.prendre_capture()
        assert capture is not None
        analyse = DetecteurVerite(lambda r=robot: r).analyser_immediat(_demande(t=capture.t))
        assert len(analyse.objets) == attendu
    # Boîte cohérente : le bas de la boîte, placé au sol, tombe près du vrai tronc.
    robot = _robot_devant_le_tronc(0.0)
    capture = robot.prendre_capture()
    assert capture is not None
    verite = robot.verite_a(capture.t)
    assert verite is not None
    analyse = DetecteurVerite(lambda: robot).analyser_immediat(_demande(t=capture.t))
    obs = placer_objets(
        analyse, _demande(verite.x, verite.y, verite.cap, capture.t), ModeleCamera(CAMERA), 1
    )
    tronc = robot.monde.troncs_couches[0]
    assert math.hypot(obs[0].x - tronc.x, obs[0].y - tronc.y) < 1.0


def test_camera_simulee_produit_un_jpeg() -> None:
    robot = _robot_devant_le_tronc(0.0)
    capture = robot.prendre_capture()
    assert capture is not None
    camera = CameraSimulee(robot.modele, CAMERA, FICHE.capteurs.stereo)
    images = camera.rendre(capture.qpos, capture.t)
    if not images:
        pytest.skip(f"rendu hors écran indisponible ici : {camera.erreur}")
    assert [m.type for m in images] == ["image", "stereo"]
    image = images[0]
    assert isinstance(image, Image)
    pixels = ImagePil.open(io.BytesIO(base64.b64decode(image.donnees)))
    assert pixels.size == (CAMERA.largeur, CAMERA.hauteur)
    # Bas de l'image : le sol (terre de la piste), pas le ciel.
    r, _v, b = pixels.convert("RGB").getpixel((320, 350))  # type: ignore[misc]
    assert r > b
    camera.fermer()


def _mission_vers_mi_piste(vision: str, stereo: bool = False) -> BancEssai:
    banc = BancEssai(
        FICHE,
        charger_monde(SCENARIO_CHEMIN),
        graine_bruit=3,
        vision=vision,  # type: ignore[arg-type]
        stereo=stereo,
    )
    banc.avancer(1.0)
    assert banc.cerveau.definir_objectif(-10.0, -7.0) is None
    banc.avancer(90, pendant=lambda b: b.cerveau.etat in ("atteint", "echec"))
    return banc


def test_la_camera_evite_le_tronc_couche_invisible_au_lidar() -> None:
    banc = _mission_vers_mi_piste("oracle")
    assert banc.cerveau.etat == "atteint"
    assert banc.verite().collisions == 0
    assert banc.cerveau.couche_vision.obstacles, "le tronc doit être dans la couche vision"


@pytest.mark.lent
def test_sans_camera_le_rover_heurte_le_tronc() -> None:
    banc = _mission_vers_mi_piste("aucune")
    assert banc.verite().collisions > 0


@pytest.mark.lent
def test_la_stereo_seule_evite_le_tronc_couche() -> None:
    """Sans modèle de vision : la paire stéréo voit le tronc que le lidar ne voit pas."""
    banc = _mission_vers_mi_piste("aucune", stereo=True)
    assert banc.cerveau.etat == "atteint"
    assert banc.verite().collisions == 0
    assert banc.cerveau.paires_traitees > 10
    assert banc.cerveau.hauteurs.obstacles().any()


# ------------------------------------------------------------------ session complète


def test_analyse_demandee_par_websocket() -> None:
    with TestClient(app) as client, client.websocket_connect("/ws") as ws:
        bienvenue = json.loads(ws.receive_text())
        assert bienvenue["vision"]["fournisseur"].startswith("simulation")
        assert bienvenue["rover"]["capteurs"]["camera"]["largeur"] == 640
        ws.send_text(json.dumps({"type": "chat", "texte": "que vois-tu ?"}))
        recus: dict[str, dict[str, Any]] = {}
        fin = time.monotonic() + 15
        while time.monotonic() < fin and not {"image", "vision"} <= recus.keys():
            message = json.loads(ws.receive_text())
            recus.setdefault(message["type"], message)
        assert recus["image"]["donnees"], "le flux de la caméra arrive au poste"
        vision = recus["vision"]
        assert vision["raison"] == "operateur"
        assert vision["erreur"] is None
        assert vision["image"], "l'image analysée accompagne le résultat (preuve)"


def test_boites_au_format_gemini() -> None:
    """Gemini : [y_min, x_min, y_max, x_max] sur 0–1000, ramené à [x_min, y_min, x_max, y_max]."""
    donnees = {
        "resume": "",
        "objets": [
            {
                "categorie": "tronc_couche",
                "boite": [600, 300, 700, 700],
                "bloque_passage": True,
                "confiance": 0.9,
            }
        ],
    }
    analyse = lire_analyse(donnees, 640, 360, "gemini")
    assert analyse.objets[0].boite == pytest.approx((300, 600, 700, 700))
