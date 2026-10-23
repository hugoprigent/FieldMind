"""Missions de pièce : preuves, accès connus, annulation et confirmation d'arrivée."""

from concurrent.futures import Future
from dataclasses import replace

import pytest
from test_exploration import IA, PoolImmediat, carte_libre, contexte

import fieldmind.mission.exploration as E
import fieldmind.mission.pieces as P
from fieldmind.types import Odometrie, Pose2D


class Monde:
    def __init__(self, pieces=()):
        self.pieces = list(pieces)
        self.generation = 0

    def resume(self):
        return {"generation": self.generation}

    def rechercher_piece(self, nom):
        return [p for p in self.pieces if p["nom"] == nom]


class Executeur(IA):
    def __init__(self, pieces=()):
        super().__init__()
        self.monde = Monde(pieces)


def piece(*, statut="confirmee", identifiant=1, x=2.0):
    return {"id": identifiant, "nom": "cuisine", "statut": statut,
            "preuves": [{"id": 1, "label": "four", "x": x, "y": 0.9, "age_s": 1,
                         "image_t": -1.0,
                         "pose_observation": {"x": 0, "y": 0}},
                        {"id": 2, "label": "micro-ondes", "x": x + 0.3, "y": 1.1,
                         "age_s": 1, "image_t": -1.0,
                         "pose_observation": {"x": 0, "y": 0}}]}


@pytest.fixture
def synchrone(monkeypatch):
    monkeypatch.setattr(P, "ThreadPoolExecutor", PoolImmediat)
    monkeypatch.setattr(E, "ThreadPoolExecutor", PoolImmediat)


def test_aucune_preuve_lance_identification_puis_exploration_mesuree(synchrone):
    ia = Executeur()
    mission = P.RejoindrePiece({"type": "rejoindre_piece", "nom": "cuisine"}, ia)
    try:
        assert mission.pas(contexte()).vitesse == 0
        assert ia.appels == [("identifier_piece", {"nom": "cuisine"})]
        assert mission._sous is None
        mission.pas(contexte(t=1))
        assert mission._piece is None and mission._phase == "explorer"
        assert isinstance(mission._sous, E.Explorer)
        assert mission._sous.action == {"type": "explorer", "piece": "cuisine"}
        choix = [a for demande, a in ia.appels if demande == "choisir_exploration"]
        assert len(choix) == 1 and choix[0]["candidats"]
        assert mission._sous._sous.but == (choix[0]["candidats"][0]["x"],
                                           choix[0]["candidats"][0]["y"])
    finally:
        mission.fermer()


def test_hypothese_de_piece_ne_declenche_pas_trajet_vers_sa_position(synchrone):
    ia = Executeur([piece(statut="hypothese")])
    mission = P.RejoindrePiece({"type": "rejoindre_piece", "nom": "cuisine"}, ia)
    try:
        premier = mission.pas(contexte())
        assert premier.vitesse == 0 and mission._phase == "identifier"
        assert mission._sous is None and mission._piece is None
        mission.pas(contexte(t=1))
        assert mission._phase == "explorer" and mission._piece is None
    finally:
        mission.fermer()


def test_deux_pieces_confirmees_sont_ambigues_sans_mouvement(synchrone):
    ia = Executeur([piece(), piece(identifiant=2, x=-2)])
    mission = P.RejoindrePiece({"type": "rejoindre_piece", "nom": "cuisine"}, ia)
    pas = mission.pas(contexte())
    assert pas.etat == "echec" and "ambiguë" in pas.detail and pas.vitesse == 0
    assert ia.appels == [] and mission._sous is None


@pytest.mark.parametrize("occupation", [100, -1])
def test_piece_derriere_mur_ou_inconnu_na_pas_de_destination(occupation, synchrone):
    carte = carte_libre()
    carte.cellules[:, 60] = occupation  # barrière continue à x = 1 m
    connue = piece(x=3.5)
    ctx = contexte(carte=carte)
    assert P.destination_piece(connue, ctx, 0.325) is None
    mission = P.RejoindrePiece({"type": "rejoindre_piece", "nom": "cuisine"},
                               Executeur([connue]))
    pas = mission.pas(ctx)
    assert pas.etat == "echec" and "aucun accès connu" in pas.detail
    assert pas.vitesse == 0 and mission._sous is None


def test_destination_libre_pres_des_preuves_et_jamais_centre_objet(synchrone):
    connue, ctx = piece(), contexte()
    cible, chemin = P.destination_piece(connue, ctx, 0.325)
    assert E.chemin_connu(ctx.carte, chemin, 0.325)
    assert all((p["x"] - cible[0]) ** 2 + (p["y"] - cible[1]) ** 2 >= 0.65 ** 2
               for p in connue["preuves"])


def test_generation_changee_annule_identification_et_navigue_pas(synchrone):
    ia = Executeur()
    attente = Future()
    ia.lancer_ia = lambda *_args: attente
    mission = P.RejoindrePiece({"type": "rejoindre_piece", "nom": "cuisine"}, ia)
    assert mission.pas(contexte()).vitesse == 0
    ia.monde.generation += 1
    pas = mission.pas(contexte(t=1))
    assert pas.etat == "echec" and "réinitialisée" in pas.detail and pas.vitesse == 0
    assert attente.cancelled() and mission._sous is None
    assert mission.pas(contexte(t=2)) is pas


def test_annulation_en_attente_ignore_toute_reponse_future(synchrone):
    ia, attente = Executeur(), Future()
    ia.lancer_ia = lambda *_args: attente
    mission = P.RejoindrePiece({"type": "rejoindre_piece", "nom": "cuisine"}, ia)
    mission.pas(contexte())
    mission.fermer()
    ia.monde.pieces = [piece()]
    pas = mission.pas(contexte(t=1))
    assert attente.cancelled() and pas.etat == "echec" and pas.vitesse == 0
    assert "annulée" in pas.detail and mission._sous is None


def test_confirmation_continue_pendant_identification_annule_future_avant_planifier(synchrone):
    ia, attente = Executeur(), Future()
    ia.lancer_ia = lambda *_args: attente
    mission = P.RejoindrePiece({"type": "rejoindre_piece", "nom": "cuisine"}, ia)
    mission.pas(contexte())
    ia.monde.pieces = [piece()]
    try:
        pas = mission.pas(contexte(t=1))
        assert attente.cancelled()
        assert mission._phase == "trajet" and mission._sous is not None
        assert pas.etat == "route"
    finally:
        mission.fermer()


@pytest.mark.parametrize("preuves_valides", [0, 1, 2])
def test_arrivee_exige_deux_preuves_fraiches_depuis_le_point_atteint(
        preuves_valides, synchrone):
    connue = piece()
    ia = Executeur([connue])
    mission = P.RejoindrePiece({"type": "rejoindre_piece", "nom": "cuisine"}, ia)
    ctx = contexte()
    try:
        mission.pas(ctx)
        assert mission._phase == "trajet" and mission._sous is not None
        cible = mission._sous.but
        for i, preuve in enumerate(connue["preuves"]):
            preuve["pose_observation"] = {"x": cible[0], "y": cible[1]}
            preuve["age_s"] = 1 if i < preuves_valides else 46
            preuve["image_t"] = 1.0
        arrivee = replace(ctx, t=1, odom=Odometrie(1, Pose2D(*cible)))
        pas = mission.pas(arrivee)
        assert pas.vitesse == 0
        assert pas.etat == ("termine" if preuves_valides == 2 else "echec")
        assert any(demande == "observer" for demande, _ in ia.appels)
        if preuves_valides == 2:
            assert "four" in pas.detail and "micro-ondes" in pas.detail
        else:
            assert "non confirmée" in pas.detail
        assert mission.pas(replace(arrivee, t=2)) is pas
    finally:
        mission.fermer()


def test_preuves_fraiches_mais_vues_depuis_un_autre_point_ne_confirment_pas(synchrone):
    connue = piece()
    ia = Executeur([connue])
    mission = P.RejoindrePiece({"type": "rejoindre_piece", "nom": "cuisine"}, ia)
    ctx = contexte()
    try:
        mission.pas(ctx)
        cible = mission._sous.but
        for preuve in connue["preuves"]:
            preuve["pose_observation"] = {"x": cible[0] + 1, "y": cible[1]}
            preuve["image_t"] = 1.0
        pas = mission.pas(replace(ctx, t=1, odom=Odometrie(1, Pose2D(*cible))))
        assert pas.etat == "echec" and "non confirmée" in pas.detail
    finally:
        mission.fermer()


def test_image_anterieure_a_arrivee_ne_confirme_pas_malgre_age_et_proximite(synchrone):
    connue = piece()
    ia = Executeur([connue])
    mission = P.RejoindrePiece({"type": "rejoindre_piece", "nom": "cuisine"}, ia)
    ctx = contexte()
    try:
        mission.pas(ctx)
        cible = mission._sous.but
        for preuve in connue["preuves"]:
            preuve["pose_observation"] = {"x": cible[0], "y": cible[1]}
            preuve["age_s"] = 1.0
            preuve["image_t"] = 0.0
        pas = mission.pas(replace(ctx, t=1, odom=Odometrie(1, Pose2D(*cible))))
        assert pas.etat == "echec" and "non confirmée" in pas.detail
    finally:
        mission.fermer()
