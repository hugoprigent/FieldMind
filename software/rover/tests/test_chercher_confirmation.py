"""Objet terrain : corroboration d'images puis trajet dans l'espace connu uniquement."""

from __future__ import annotations

from concurrent.futures import Future
from types import SimpleNamespace

import pytest
from test_exploration import PoolImmediat, contexte

import fieldmind.estimation.semantique as M
import fieldmind.mission.executeur as X
import fieldmind.mission.exploration as E
from fieldmind.bus import Bus, S
from fieldmind.config import Profil


@pytest.fixture
def terrain(monkeypatch):
    horloge = SimpleNamespace(t=100.0)
    monkeypatch.setattr(M, "time", SimpleNamespace(monotonic=lambda: horloge.t))
    monkeypatch.setattr(X, "ThreadPoolExecutor", PoolImmediat)
    monkeypatch.setattr(E, "ThreadPoolExecutor", PoolImmediat)
    bus = Bus()
    monde = M.MemoireSemantique(bus)
    appels = []

    def lancer(demande, argument):
        appels.append(demande)
        futur = Future()
        point = {"label": "caisse rouge", "sol": [3, 0], "pose_odom": [0, 0, 0],
                 "image_t": horloge.t}
        futur.set_result({"points": [point]} if demande == "observer" else [point])
        return futur

    executeur = SimpleNamespace(monde=monde, profil=Profil.charger(reglages=None),
                               lancer_ia=lancer)

    def voir(sol=(3, 0), label="caisse rouge", image_t=None):
        bus.publier(S.VISION, {"pose_image": {"x": 0, "y": 0, "cap": 0},
                              "image_t": horloge.t if image_t is None else image_t,
                              "points": [{"label": label, "sol": sol}]})

    return executeur, monde, horloge, appels, voir


def test_objet_image_unique_attend_seconde_acquisition_sans_avancer(terrain):
    executeur, monde, horloge, appels, voir = terrain
    voir()
    recherche = X.Chercher({"type": "chercher", "objet": "caisse rouge"}, executeur)
    ctx = contexte(t=horloge.t)
    assert recherche.pas(ctx).vitesse == 0
    pas = recherche.pas(ctx)
    assert pas.etat == "route" and pas.vitesse == 0 and recherche._sous is None
    assert "seconde observation" in pas.detail
    assert monde.rechercher("caisse rouge")[0]["statut"] == "hypothese"
    assert recherche.pas(ctx).vitesse == 0
    assert appels == ["chercher"]
    horloge.t += 0.6
    voir()
    ctx.t = horloge.t
    recherche.pas(ctx)
    pas = recherche.pas(ctx)
    assert pas.etat == "route" and recherche._mode == "approche"
    assert recherche._sous is not None
    assert recherche.pas(ctx).vitesse > 0
    assert isinstance(recherche._navigation, E._NavigationConnue)
    recherche.fermer()


def test_memoire_confirmee_ne_traverse_pas_zone_inconnue(terrain):
    executeur, _, horloge, _, voir = terrain
    voir()
    horloge.t += 0.6
    voir()
    ctx = contexte(t=horloge.t)
    ctx.carte.cellules[:, 60] = -1  # x=1m entre rover et caisse
    recherche = X.Chercher({"type": "chercher", "objet": "caisse rouge"}, executeur)
    assert recherche.pas(ctx).vitesse == 0
    assert recherche._mode == "memoire"
    assert isinstance(recherche._navigation, E._NavigationConnue)
    recherche.fermer()


def test_production_refuse_approche_sans_carte(terrain):
    executeur, _, horloge, _, voir = terrain
    voir()
    horloge.t += 0.6
    voir()
    ctx = contexte(t=horloge.t)
    ctx.carte = None
    recherche = X.Chercher({"type": "chercher", "objet": "caisse rouge"}, executeur)
    pas = recherche.pas(ctx)
    assert pas.etat == "echec" and pas.vitesse == 0
    assert "carte" in pas.raison


def test_reset_memoire_interrompt_approche_du_repere(terrain):
    executeur, monde, horloge, _, voir = terrain
    voir()
    horloge.t += 0.6
    voir()
    recherche = X.Chercher({"type": "chercher", "objet": "caisse rouge"}, executeur)
    ctx = contexte(t=horloge.t)
    assert recherche.pas(ctx).vitesse > 0
    monde.effacer()
    pas = recherche.pas(ctx)
    assert pas.vitesse == 0 and recherche._sous is None
    assert "confirmer" in pas.detail
    recherche.fermer()


def test_hypothese_proche_ne_termine_pas_simple_reperage(terrain):
    executeur, _, horloge, _, voir = terrain
    voir(sol=(0.3, 0))

    def vision_proche(*args):
        futur = Future()
        futur.set_result([{"sol": [0.3, 0], "pose_odom": [0, 0, 0], "image_t": horloge.t}])
        return futur

    executeur.lancer_ia = vision_proche
    recherche = X.Chercher({"type": "chercher", "objet": "caisse rouge", "approcher": False},
                           executeur)
    ctx = contexte(t=horloge.t)
    recherche.pas(ctx)
    assert recherche.pas(ctx).etat != "termine"
    assert recherche.trouve is None
    recherche.fermer()


def test_fermer_annule_analyse_et_sous_primitive(terrain):
    executeur, _, _, _, _ = terrain
    recherche = X.Chercher({"type": "chercher", "objet": "caisse rouge"}, executeur)
    analyse, calcul = Future(), Future()
    fermetures = []
    recherche._analyse = analyse
    recherche._sous = SimpleNamespace(fermer=lambda: fermetures.append("sous"))
    recherche._navigation = SimpleNamespace(fermer=lambda: calcul.cancel())
    recherche.fermer()
    assert analyse.cancelled() and calcul.cancelled() and fermetures == ["sous"]
    assert recherche._sous is None and recherche._navigation is None


def test_explorer_ne_suit_pas_hypothese_et_reobserve(terrain):
    executeur, _, horloge, appels, voir = terrain
    voir()
    exploration = E.Explorer({"type": "explorer", "objet": "caisse rouge"}, executeur)
    ctx = contexte(t=horloge.t)
    pas = exploration.pas(ctx)
    assert pas.vitesse == 0 and not exploration._approche and exploration._sous is None
    assert "seconde observation" in pas.detail
    exploration.pas(ctx)
    assert appels == ["observer"]
    horloge.t += 0.6
    voir()
    ctx.t = horloge.t
    assert exploration.pas(ctx).etat == "route"
    assert exploration._approche and exploration._sous is not None
    exploration.fermer()


def test_vue_sans_distance_ne_termine_pas_mission(terrain):
    executeur, _, horloge, _, _ = terrain

    def vue_sans_profondeur(*args):
        futur = Future()
        futur.set_result([{"label": "caisse rouge", "u": 0.5}])
        return futur

    executeur.lancer_ia = vue_sans_profondeur
    recherche = X.Chercher({"type": "chercher", "objet": "caisse rouge"}, executeur)
    ctx = contexte(t=horloge.t)
    recherche.pas(ctx)
    pas = recherche.pas(ctx)
    assert pas.etat != "termine" and recherche._mode == "exploration"
    recherche.fermer()
