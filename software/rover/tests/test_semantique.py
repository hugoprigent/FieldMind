"""Mémoire spatiale : uniquement observations situées, persistantes et bornées."""

from __future__ import annotations

import math
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import pytest

from fieldmind.bus import Bus, S
from fieldmind.estimation import semantique
from fieldmind.estimation.semantique import MemoireSemantique
from fieldmind.types import Odometrie, Pose2D


@pytest.fixture
def monde(monkeypatch):
    horloge = SimpleNamespace(t=100.0)
    monkeypatch.setattr(semantique, "time", SimpleNamespace(monotonic=lambda: horloge.t))
    bus = Bus()
    bus.publier(S.ODOM, Odometrie(t=100.0, pose=Pose2D()))
    return bus, MemoireSemantique(bus), horloge


def vision(label="porte blanche", sol=(2.0, 0.0), **extras):
    return {"points": [{"label": label, "sol": sol, **extras}]}


def test_pose_de_capture_et_dimensions(monde):
    bus, memoire, _ = monde
    bus.publier(S.VISION, {**vision(hauteur=2.1, largeur=0.9, cap=0.1, categorie="porte",
                                  source="lidar+stereo"),
                           "pose_image": {"x": 1.0, "y": 3.0, "cap": math.pi / 2},
                           "image_t": 99.0})
    objet, = memoire.resume()["objets"]
    assert (objet["x"], objet["y"]) == pytest.approx((1, 5))
    assert objet["age_s"] == 1.0
    assert objet["cap"] == pytest.approx(math.pi / 2 + 0.1, abs=0.0001)
    assert objet["categorie"] == "porte" and objet["dimensions_estimees"]
    assert objet["source_dimensions"] == "lidar+stereo"
    assert objet["hauteur"] == 2.1 and objet["largeur"] == 0.9
    assert "longueur" not in objet  # aucune dimension de volume inventée
    assert objet["pose_observation"] == pytest.approx({"x": 1, "y": 3, "cap": math.pi / 2},
                                                     abs=0.0001)


def test_association_suit_la_pose_et_enrichit_le_nom(monde):
    bus, memoire, horloge = monde
    bus.publier(S.VISION, vision("porte"))
    horloge.t += 1
    bus.publier(S.VISION, {**vision(), "pose_image": {"x": 0.5, "y": 0, "cap": 0},
                           "points": [{"label": "porte blanche", "sol": [1.5, 0]}]})
    objet, = memoire.resume()["objets"]
    assert objet["vues"] == 2 and objet["label"] == "porte blanche" and objet["x"] == 2.0
    assert memoire.rechercher("la porte") == [objet]
    assert memoire.rechercher("porte rouge") == []


def test_deux_objets_proches_ne_fusionnent_pas_dans_meme_image(monde):
    bus, memoire, horloge = monde
    message = {"points": [{"label": "chaise", "sol": [1, y]} for y in (0, 0.5)]}
    bus.publier(S.VISION, message)
    horloge.t += 1
    bus.publier(S.VISION, message)
    objets = memoire.resume()["objets"]
    assert len(objets) == 2 and all(o["vues"] == 2 for o in objets)
    assert abs(objets[0]["y"] - objets[1]["y"]) == 0.5


def test_age_capture_et_expiration_mobile_et_statique(monde):
    bus, memoire, horloge = monde
    bus.publier(S.VISION, vision())
    bus.publier(S.DETECTIONS, {"objets": [{"label": "chien", "sol": [1, 1]}],
                               "image_t": 98})
    assert len(memoire.resume()["objets"]) == 2
    assert not memoire.rechercher("chien")  # une ancienne position mobile n'est pas une cible
    horloge.t = 103
    assert [o["label"] for o in memoire.resume()["objets"]] == ["porte blanche"]
    horloge.t = 3700
    assert not memoire.resume()["objets"]


def test_reponse_ancienne_ne_deplace_pas_objet_recent(monde):
    bus, memoire, horloge = monde
    bus.publier(S.VISION, {**vision(), "image_t": 99})
    horloge.t = 102
    bus.publier(S.VISION, {**vision(sol=(2.2, 0)), "image_t": 101})
    bus.publier(S.VISION, {**vision(sol=(1.8, 0)), "image_t": 100})
    objet, = memoire.resume()["objets"]
    assert objet["vues"] == 2 and objet["x"] == 2.08 and objet["age_s"] == 1


def test_reset_rejette_reponse_de_l_ancien_repere(monde):
    bus, memoire, horloge = monde
    bus.publier(S.VISION, vision())
    horloge.t = 101
    memoire.effacer()
    bus.publier(S.VISION, {**vision(), "image_t": 100})
    assert not memoire.resume()["objets"]
    bus.publier(S.VISION, {**vision(), "image_t": 101})
    assert len(memoire.resume()["objets"]) == 1


@pytest.mark.parametrize("extras", [
    {"pose_image": None}, {"pose_image": {"x": 1}},
    {"pose_image": {"x": float("nan"), "y": 0, "cap": 0}},
    {"image_t": float("nan")}, {"image_t": "erreur"}, {"image_t": 101},
    {"geometrie_valide": False},
])
def test_geometrie_inutilisable_rejetee(monde, extras):
    bus, memoire, _ = monde
    bus.publier(S.VISION, {**vision(), **extras})
    assert not memoire.resume()["objets"]


def test_pas_de_position_sans_odometrie():
    memoire = MemoireSemantique(Bus())
    memoire.integrer(vision())
    assert not memoire.resume()["objets"]


def test_entrees_invalides_isolees_et_reponse_independante(monde):
    bus, memoire, _ = monde
    for invalide in (None, [], {"points": None}, {"points": 1}):
        memoire.integrer(invalide)
    bus.publier(S.VISION, {"points": [None, "porte", {"sol": 1, "label": "porte"},
                                     {"sol": [float("nan"), 1], "label": "porte"},
                                     {"sol": [31, 0], "label": "porte"},
                                     {"sol": [1, 0], "label": ["porte"]},
                                     {"sol": [1, 0], "label": "porte", "hauteur": True,
                                      "largeur": float("inf"), "longueur": -1}]})
    objet, = memoire.resume()["objets"]
    assert not ({"hauteur", "largeur", "longueur"} & objet.keys())
    objet["x"] = 999
    assert memoire.resume()["objets"][0]["x"] == 1


def scene(*murs, **extras):
    return {"repere": "odom", "objets": list(murs), **extras}


def mur(x, longueur=1.0, **extras):
    return {"id": 1, "type": "mur", "x": x, "y": 0, "cap": 0,
            "longueur": longueur, **extras}


def test_murs_mesures_persistants_sans_boucher_une_porte(monde):
    bus, memoire, horloge = monde
    bus.publier(S.SCENE, scene(mur(1), mur(1.2, extrapole=True), mur(3)))
    bus.publier(S.SCENE, scene(mur(1.3, id=99)))
    murs = sorted(memoire.resume()["murs"], key=lambda m: m["x"])
    assert len(murs) == 2 and murs[0]["longueur"] == pytest.approx(1.3)
    horloge.t += 60
    bus.publier(S.SCENE, scene())
    assert len(memoire.resume()["murs"]) == 2
    horloge.t += 3600
    assert not memoire.resume()["murs"]


def test_murs_non_mesures_invalides_ou_dans_ancien_repere_rejetes(monde):
    bus, memoire, horloge = monde
    bus.publier(S.SCENE, scene(mur(1), repere="base_link"))
    assert not memoire.resume()["murs"]
    bus.publier(S.SCENE, scene(None, mur(float("nan")), mur(1, longueur=0),
                              mur(1, memorise=True), mur(2, age_s=-1), mur(3, cap=None)))
    assert not memoire.resume()["murs"]
    horloge.t = 101
    memoire.effacer()
    bus.publier(S.SCENE, scene(mur(1), t=100))
    assert not memoire.resume()["murs"]


def test_concurrence_et_capacite_bornees(monde):
    _, memoire, _ = monde

    def integrer(index):
        memoire.integrer({**vision(f"caisse {index}", sol=(1 + index % 20, 0)),
                          "pose_image": {"x": 0, "y": 0, "cap": 0}})
        return memoire.resume()

    with ThreadPoolExecutor(max_workers=4) as pool:
        resultats = list(pool.map(integrer, range(300)))
    assert len(memoire.resume()["objets"]) == 256
    assert all(len(r["objets"]) <= 256 for r in resultats)


def cuisine(memoire, **extras):
    return {"nom": "cuisine", "objets": [o["id"] for o in memoire.resume()["objets"]],
            "raison": "four et micro-ondes observés ensemble", "confiance": "forte", **extras}


def equipements(bus, *labels, positions=None, horloge=None):
    message = {
        "pose_image": {"x": 0, "y": 0, "cap": 0},
        "points": [{"label": label, "sol": positions[i] if positions else [2, i],
                    "source": "lidar+stereo"} for i, label in enumerate(labels)],
    }
    bus.publier(S.VISION, message)
    if horloge is not None:
        horloge.t += 0.6
        bus.publier(S.VISION, message)


def test_piece_confirmee_a_partir_de_preuves_et_non_d_un_nom(monde):
    bus, memoire, horloge = monde
    assert memoire.integrer_pieces([{"nom": "cuisine"}]) == []
    assert memoire.rechercher_piece("la cuisine") == []
    equipements(bus, "four", "micro-ondes", horloge=horloge)
    piece, = memoire.integrer_pieces([cuisine(memoire, nom="La CUISINÉ")])
    assert piece["statut"] == "confirmee"
    assert {p["label"] for p in piece["preuves"]} == {"four", "micro-ondes"}
    assert all(p["source"] == "lidar+stereo" and p["age_s"] == 0 for p in piece["preuves"])
    assert all(p["pose_observation"] == {"x": 0, "y": 0, "cap": 0}
               for p in piece["preuves"])
    assert not ({"x", "y"} & piece.keys())  # aucune cible inventée au centre de la pièce
    assert memoire.rechercher_piece("cuisine") == [piece]
    piece["preuves"][0]["x"] = 999
    assert memoire.rechercher_piece("la cuisine")[0]["preuves"][0]["x"] == 2


@pytest.mark.parametrize("labels", [("table", "chaises"), ("table en bois", "chaise rouge")])
def test_indices_generiques_restent_hypothese_meme_si_modele_affirme_forte(monde, labels):
    bus, memoire, _ = monde
    equipements(bus, *labels)
    piece, = memoire.integrer_pieces([cuisine(memoire)])
    assert piece["statut"] == "hypothese" and piece["confiance"] == "faible"


def test_hypothese_faible_ne_devient_pas_destination_par_ses_objets(monde):
    bus, memoire, horloge = monde
    equipements(bus, "four", "micro-ondes", horloge=horloge)
    piece, = memoire.integrer_pieces([cuisine(memoire, confiance="faible")])
    assert piece["statut"] == "hypothese"
    confirmee, = memoire.integrer_pieces([cuisine(memoire)])
    assert confirmee["id"] == piece["id"] and confirmee["statut"] == "confirmee"
    assert len(memoire.resume()["pieces"]) == 1


@pytest.mark.parametrize("extras", [{"objets": [999, 998]}, {"objets": [1, 1]},
                                    {"objets": [True, 2]}, {"objets": [{"id": 1}, 2]},
                                    {"x": 20, "y": 15}, {"position": [20, 15]},
                                    {"confiance": "certaine"}, {"nom": ""}])
def test_piece_sans_preuves_verifiables_ou_avec_coordonnees_modele_rejetee(monde, extras):
    bus, memoire, _ = monde
    equipements(bus, "four", "micro-ondes")
    assert memoire.integrer_pieces([cuisine(memoire, **extras)]) == []
    assert memoire.resume()["pieces"] == []


@pytest.mark.parametrize("labels", [("four noir", "four blanc"), ("oven", "four"),
                                    ("micro-ondes", "microwave"), ("table", "tables")])
def test_deux_descriptions_du_meme_type_objet_ne_sont_pas_deux_indices(monde, labels):
    bus, memoire, _ = monde
    equipements(bus, *labels)
    assert memoire.integrer_pieces([cuisine(memoire)]) == []


def test_preuves_mobiles_lointaines_ou_separees_par_mur_rejetees(monde):
    bus, memoire, _ = monde
    equipements(bus, "four", "personne")
    assert memoire.integrer_pieces([cuisine(memoire)]) == []
    memoire.effacer()
    equipements(bus, "four", "micro-ondes", positions=[[1, 0], [6, 0]])
    assert memoire.integrer_pieces([cuisine(memoire)]) == []
    memoire.effacer()
    equipements(bus, "four", "micro-ondes", positions=[[1, 0], [3, 0]])
    bus.publier(S.SCENE, scene(mur(2, cap=math.pi / 2, longueur=3)))
    assert memoire.integrer_pieces([cuisine(memoire)]) == []


def test_nouvelle_cloison_et_expiration_invalident_piece(monde):
    bus, memoire, horloge = monde
    equipements(bus, "four", "micro-ondes", positions=[[1, 0], [3, 0]])
    assert memoire.integrer_pieces([cuisine(memoire)])
    bus.publier(S.SCENE, scene(mur(2, cap=math.pi / 2, longueur=3)))
    assert memoire.resume()["pieces"] == []
    memoire.effacer()
    equipements(bus, "four", "micro-ondes")
    proposition = cuisine(memoire)
    assert memoire.integrer_pieces([proposition])
    horloge.t += semantique.OUBLI_STATIQUE_S
    assert memoire.resume()["pieces"] == []
    assert memoire.integrer_pieces([proposition]) == []


def test_reset_refuse_resultat_ancienne_generation(monde):
    bus, memoire, _ = monde
    equipements(bus, "four", "micro-ondes")
    generation = memoire.resume()["generation"]
    assert memoire.integrer_pieces([cuisine(memoire)], generation=generation)
    memoire.effacer()
    assert memoire.resume()["pieces"] == []
    assert memoire.resume()["generation"] == generation + 1
    equipements(bus, "four", "micro-ondes")
    assert memoire.integrer_pieces([cuisine(memoire)], generation=generation) == []
    assert memoire.integrer_pieces([cuisine(memoire)], generation=generation + 1)


def test_raison_et_capacite_pieces_bornees(monde):
    bus, memoire, _ = monde
    equipements(bus, "four", "micro-ondes")
    for i in range(70):
        memoire.integrer_pieces([cuisine(memoire, nom=f"piece {i}", raison="a" * 800)])
    pieces = memoire.resume()["pieces"]
    assert len(pieces) == 64
    assert all(len(p["raison"]) == 400 for p in pieces)


@pytest.mark.parametrize("label", ["herbe", "herbe verte", "grass", "brindille sèche"])
def test_terrain_continu_ne_cree_pas_repere_ponctuel(monde, label):
    bus, memoire, horloge = monde
    equipements(bus, label, horloge=horloge)
    assert memoire.resume()["objets"] == []
    assert memoire.rechercher(label) == []


@pytest.mark.parametrize("label", ["arbuste", "buisson", "arbre"])
def test_repere_vegetal_attend_deux_acquisitions_distinctes(monde, label):
    bus, memoire, horloge = monde
    message = {**vision(label), "pose_image": {"x": 0, "y": 0, "cap": 0}, "image_t": 100}
    bus.publier(S.DETECTIONS, message)
    horloge.t += 0.6
    bus.publier(S.VISION, message)  # modèle GX10 tardif sur la même image D-FINE
    objet, = memoire.resume()["objets"]
    assert objet["statut"] == "hypothese" and objet["observations_distinctes"] == 1
    bus.publier(S.VISION, {**message, "image_t": horloge.t})
    objet, = memoire.resume()["objets"]
    assert objet["statut"] == "confirme" and objet["observations_distinctes"] == 2


def test_rafale_et_reponse_retardee_ne_confirment_pas_un_objet(monde):
    bus, memoire, horloge = monde
    message = {**vision("four"), "pose_image": {"x": 0, "y": 0, "cap": 0}, "image_t": 100}
    bus.publier(S.VISION, message)
    horloge.t += 0.2
    bus.publier(S.DETECTIONS, {**message, "image_t": horloge.t})
    bus.publier(S.VISION, {**message, "image_t": 99})
    assert memoire.resume()["objets"][0]["statut"] == "hypothese"
    horloge.t += 0.4
    bus.publier(S.VISION, {**message, "image_t": horloge.t})
    assert memoire.resume()["objets"][0]["statut"] == "confirme"


def test_piece_forte_attend_corroboration_de_toutes_ses_preuves(monde):
    bus, memoire, horloge = monde
    equipements(bus, "table", "four")
    piece, = memoire.integrer_pieces([cuisine(memoire)])
    assert piece["statut"] == "hypothese"
    horloge.t += 0.6
    equipements(bus, "four", positions=[[2, 1]])
    assert memoire.resume()["pieces"][0]["statut"] == "hypothese"
    equipements(bus, "table", positions=[[2, 0]])
    piece, = memoire.resume()["pieces"]
    assert piece["statut"] == "confirmee"
    assert all(p["observations_distinctes"] == 2 for p in piece["preuves"])


def test_rechercher_reste_compatible_et_expose_objet_non_confirme(monde):
    bus, memoire, _ = monde
    equipements(bus, "arbuste")
    objet, = memoire.rechercher("l'arbuste")
    assert objet["statut"] == "hypothese" and objet["observations_distinctes"] == 1


def test_meme_capture_avec_horloge_legerement_en_avance_ne_confirme_pas(monde):
    bus, memoire, horloge = monde
    message = {**vision("arbuste"), "pose_image": {"x": 0, "y": 0, "cap": 0},
               "image_t": 100.5}
    bus.publier(S.DETECTIONS, message)
    horloge.t = 101
    bus.publier(S.VISION, message)
    assert memoire.resume()["objets"][0]["observations_distinctes"] == 1
    assert memoire.resume()["objets"][0]["statut"] == "hypothese"


def test_preuves_piece_exportent_horodatage_capture_pour_confirmation_arrivee(monde):
    bus, memoire, horloge = monde
    equipements(bus, "four", "micro-ondes", horloge=horloge)
    piece, = memoire.integrer_pieces([cuisine(memoire)])
    capture = horloge.t
    horloge.t += 5
    piece, = memoire.rechercher_piece("la cuisine")
    assert all(p["image_t"] == capture and p["age_s"] == 5 for p in piece["preuves"])
    # Un résultat reçu après l'arrivée peut provenir d'une acquisition ancienne.
    bus.publier(S.VISION, {"pose_image": {"x": 0, "y": 0, "cap": 0}, "image_t": capture,
                          "points": [{"label": "four", "sol": [2, 0]}]})
    assert all(p["image_t"] == capture for p in memoire.resume()["pieces"][0]["preuves"])


def test_couleurs_contradictoires_ne_fusionnent_pas_les_caisses_proches(monde):
    bus, memoire, horloge = monde
    equipements(bus, "caisse rouge", positions=[[2, 0]])
    horloge.t += 0.6
    equipements(bus, "caisse bleue", positions=[[2.1, 0]])
    objets = memoire.resume()["objets"]
    assert len(objets) == 2
    assert all(o["statut"] == "hypothese" and o["observations_distinctes"] == 1 for o in objets)
    rouge, = memoire.rechercher("caisse rouge")
    bleue, = memoire.rechercher("caisse bleue")
    assert rouge["id"] != bleue["id"]


@pytest.mark.parametrize("marqueur", [None, "detections", "vision"])
def test_d_fine_ne_confirme_jamais_seul_meme_si_type_absent_ou_incorrect(monde, marqueur):
    bus, memoire, horloge = monde
    message = {"source": "d_fine", "image_t": horloge.t,
               "pose_image": {"x": 0, "y": 0, "cap": 0},
               "objets": [{"label": "chaise", "sol": [2, 0], "boite": [0.2, 0.1, 0.4, 0.8]}]}
    if marqueur is not None:
        message["type"] = marqueur
    bus.publier(S.DETECTIONS, message)
    horloge.t += 0.6
    bus.publier(S.DETECTIONS, {**message, "image_t": horloge.t})
    objet, = memoire.resume()["objets"]
    assert objet["vues"] == 2 and objet["observations_distinctes"] == 2
    assert objet["observations_visuelles"] == 0 and objet["statut"] == "hypothese"


def test_deux_acquisitions_vision_confirment_un_objet_d_fine(monde):
    bus, memoire, horloge = monde
    message = {**vision("chaise"), "pose_image": {"x": 0, "y": 0, "cap": 0}, "image_t": 100}
    bus.publier(S.DETECTIONS, message)
    bus.publier(S.VISION, {**message, "type": "vision"})
    objet, = memoire.resume()["objets"]
    ident = objet["id"]
    assert objet["observations_visuelles"] == 1 and objet["statut"] == "hypothese"
    horloge.t += 0.6
    bus.publier(S.VISION, {**message, "image_t": horloge.t, "type": "vision"})
    objet, = memoire.resume()["objets"]
    assert objet["id"] == ident and objet["observations_visuelles"] == 2
    assert objet["statut"] == "confirme"


def test_d_fine_recent_ne_bloque_pas_confirmation_vlm_plus_ancienne(monde):
    bus, memoire, horloge = monde
    message = {**vision("chaise"), "pose_image": {"x": 0, "y": 0, "cap": 0}, "image_t": 100}
    bus.publier(S.DETECTIONS, message)
    horloge.t = 102
    bus.publier(S.DETECTIONS, {**message, "image_t": 102,
                               "points": [{"label": "chaise", "sol": [2.2, 0]}]})
    x_recent = memoire.resume()["objets"][0]["x"]
    bus.publier(S.VISION, {**message, "image_t": 100.2, "type": "vision"})
    bus.publier(S.VISION, {**message, "image_t": 101.2, "type": "vision"})
    objet, = memoire.resume()["objets"]
    assert objet["statut"] == "confirme" and objet["observations_visuelles"] == 2
    assert objet["x"] == x_recent and objet["age_s"] == 0


def test_d_fine_ne_rafraichit_pas_preuves_visuelles_arrivee(monde):
    bus, memoire, horloge = monde
    equipements(bus, "four", "micro-ondes", horloge=horloge)
    memoire.integrer_pieces([cuisine(memoire)])
    capture_visuelle = horloge.t
    horloge.t += 5
    bus.publier(S.DETECTIONS, {
        "pose_image": {"x": 0.2, "y": 0, "cap": 0}, "image_t": horloge.t,
        "objets": [{"label": "four", "sol": [2, 0]},
                   {"label": "micro-ondes", "sol": [2, 1]}],
    })
    piece, = memoire.rechercher_piece("cuisine")
    assert piece["statut"] == "confirmee"
    assert all(p["image_t"] == capture_visuelle and p["age_s"] == 5 for p in piece["preuves"])
    assert all(p["pose_observation"]["x"] == 0 for p in piece["preuves"])
    assert all(o["pose_observation"]["x"] == 0.2 for o in memoire.resume()["objets"])


def test_hypotheses_expirent_apres_45s_objets_confirmes_persistent(monde):
    bus, memoire, horloge = monde
    equipements(bus, "four", horloge=horloge)
    equipements(bus, "chaise")
    horloge.t += semantique.OUBLI_HYPOTHESE_S
    objets = memoire.resume()["objets"]
    assert [o["label"] for o in objets] == ["four"]
    assert objets[0]["statut"] == "confirme"


@pytest.mark.parametrize("pixels", [
    [{"u": 0.4, "v": 0.7}, {"u": 0.41, "v": 0.7}],
    [{"boite": [0.3, 0.2, 0.5, 0.8]}, {"boite": [0.31, 0.2, 0.51, 0.8]}],
])
def test_doublon_meme_image_proche_en_pixels_et_sol_ignore(monde, pixels):
    bus, memoire, horloge = monde
    points = [{"label": "chaise", "sol": [2 + i * 0.08, 0], **pixel}
              for i, pixel in enumerate(pixels)]
    message = {"type": "vision", "pose_image": {"x": 0, "y": 0, "cap": 0},
               "points": points, "image_t": horloge.t}
    bus.publier(S.VISION, message)
    objet, = memoire.resume()["objets"]
    assert objet["statut"] == "hypothese" and objet["observations_visuelles"] == 1
    horloge.t += 0.6
    bus.publier(S.VISION, {**message, "image_t": horloge.t})
    objet, = memoire.resume()["objets"]
    assert objet["statut"] == "confirme" and objet["observations_visuelles"] == 2


@pytest.mark.parametrize("pixels", [None, [(0.3, 0.7), (0.7, 0.7)]])
def test_deux_objets_distincts_proches_ne_sont_pas_dedoublonnes(monde, pixels):
    bus, memoire, _ = monde
    points = [{"label": "chaise", "sol": [2 + i * 0.1, 0],
               **({"u": pixels[i][0], "v": pixels[i][1]} if pixels else {})} for i in range(2)]
    bus.publier(S.VISION, {"pose_image": {"x": 0, "y": 0, "cap": 0}, "points": points})
    assert len(memoire.resume()["objets"]) == 2


def test_d_fine_ne_prolonge_pas_hypothese_vlm_unique(monde):
    bus, memoire, horloge = monde
    message = {**vision("chaise"), "pose_image": {"x": 0, "y": 0, "cap": 0}, "image_t": 100}
    bus.publier(S.VISION, message)
    for seconde in (110, 120, 130, 140, 144):
        horloge.t = seconde
        bus.publier(S.DETECTIONS, {**message, "image_t": horloge.t})
    objet, = memoire.resume()["objets"]
    assert objet["observations_visuelles"] == 1 and objet["age_s"] == 0
    horloge.t = 145
    assert memoire.resume()["objets"] == []


def test_d_fine_ne_prolonge_pas_memoire_visuelle_confirmee_au_dela_une_heure(monde):
    bus, memoire, horloge = monde
    equipements(bus, "four", horloge=horloge)
    capture_visuelle = horloge.t
    horloge.t = capture_visuelle + semantique.OUBLI_STATIQUE_S - 1
    bus.publier(S.DETECTIONS, {**vision("four"), "image_t": horloge.t,
                               "pose_image": {"x": 0, "y": 0, "cap": 0}})
    assert memoire.resume()["objets"][0]["statut"] == "confirme"
    horloge.t += 1
    assert memoire.resume()["objets"] == []
