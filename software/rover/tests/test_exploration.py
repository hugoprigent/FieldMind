"""Scénarios d'exploration sans matériel ni appel réseau."""

from __future__ import annotations

import math
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

import fieldmind.mission.exploration as E
from fieldmind.config import Profil
from fieldmind.mission.actions import Contexte
from fieldmind.mission.executeur import Navigation
from fieldmind.navigation.local import CommandeLocale
from fieldmind.types import CarteOccupation, Chemin, EtatSecurite, Odometrie, Pose2D


def carte_libre(taille=100):
    grille = np.zeros((taille, taille), np.int8)
    grille[[0, -1], :] = 100
    grille[:, [0, -1]] = 100
    return CarteOccupation(0.0, 0.1, -taille * 0.05, -taille * 0.05, grille)


def chemin(points):
    points = np.asarray(points, float).reshape(-1, 2)
    but = tuple(points[-1]) if len(points) else (0.0, 0.0)
    return Chemin(0, points, but, float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()))


class LigneDroite:
    def planifier(self, carte, pose, cible):
        return chemin([(pose.x, pose.y), cible])

    def commande(self, odom, route, obstacles, vitesse_max, indicatifs=None):
        return CommandeLocale(0.25, 0.0)


class PoolImmediat:
    """Le même contrat Future, avec ordonnancement déterministe pour les scénarios."""

    def __init__(self, **kwargs):
        pass

    def submit(self, f, *args):
        futur = Future()
        try:
            futur.set_result(f(*args))
        except Exception as erreur:
            futur.set_exception(erreur)
        return futur

    def shutdown(self, **kwargs):
        pass


class IA:
    def __init__(self, choix=0, observations=None):
        self.profil = Profil.charger(reglages=None)
        self.choix = choix
        self.observations = list(observations or [])
        self.appels = []

    def lancer_ia(self, demande, argument):
        self.appels.append((demande, argument))
        resultat = ({"index": self.choix, "raison": "observer cette frontière"}
                    if demande == "choisir_exploration"
                    else self.observations.pop(0) if self.observations else {})
        if isinstance(resultat, Future):
            return resultat
        futur = Future()
        futur.set_result(resultat)
        return futur


def contexte(carte=None, pose=None, t=0.0, nav=None):
    return Contexte(t=t, odom=Odometrie(t, pose or Pose2D()),
                    securite=EtatSecurite(t, arret=False, vitesse_max_avant=0.5,
                                         vitesse_max_arriere=0.3),
                    carte=carte if carte is not None else carte_libre(),
                    obstacles=np.empty((0, 2)), vitesse_max=0.35, vitesse_defaut=0.3,
                    braquage_max=math.radians(31), empattement=0.4,
                    navigation=nav or LigneDroite())


@pytest.fixture
def synchrone(monkeypatch):
    monkeypatch.setattr(E, "ThreadPoolExecutor", PoolImmediat)


def test_points_deterministes_connus_connectes_et_dans_rayon():
    carte = carte_libre()
    carte.cellules[:, 60:] = -1
    carte.cellules[20:80, 75:95] = 0  # îlot connu déconnecté derrière l'inconnu
    points = E.points_exploration(carte, Pose2D(), Pose2D(), [(0, 0)], 2.0)
    assert points == E.points_exploration(carte, Pose2D(), Pose2D(), [(0, 0)], 2.0)
    assert points
    for p in points:
        assert math.hypot(p["x"], p["y"]) <= 2.0
        assert p["x"] < 1.0
        assert E.chemin_connu(carte, chemin([(p["x"], p["y"])]), 0.35)
        assert math.hypot(p["x"], p["y"]) >= 1.1


@pytest.mark.parametrize("points", [[], [[float("nan"), 0]], [[20, 0]], [[-5.1, 0]]])
def test_chemin_invalide_refuse(points):
    assert not E.chemin_connu(carte_libre(), chemin(points), 0.325)


def test_chemin_refuse_traversee_inconnue_et_marge_mur():
    carte = carte_libre()
    carte.cellules[:, 50] = -1
    assert not E.chemin_connu(carte, chemin([[-1, 0], [1, 0]]), 0.325)
    assert not E.chemin_connu(carte_libre(), chemin([[-4.8, 0], [-4.8, 1]]), 0.325)


def test_chemin_detour_mesure_reel_autour_mur():
    carte = carte_libre()
    carte.cellules[25:75, 50] = 100
    profil = Profil.charger(reglages=None)
    nav = Navigation(profil)
    nav._planificateur.utiliser_numba = False
    route = E._planifier(nav, carte, Pose2D(-1, 0), (1, 0), 0.325)
    assert route is not None
    assert route.longueur_m > 5
    assert E.chemin_connu(carte, route, 0.325)
    carte.cellules[0:25, 50] = -1
    carte.cellules[75:, 50] = -1
    assert E._planifier(nav, carte, Pose2D(-1, 0), (1, 0), 0.325) is None


def test_exploration_progressive_reobserve_et_borne_points(synchrone):
    ia = IA(choix=1)
    exploration = E.Explorer({"type": "explorer", "points_max": 4}, ia)
    ctx = contexte()
    for tick in range(30):
        pas = exploration.pas(ctx)
        if pas.etat in ("termine", "echec"):
            break
        if exploration._sous is not None:
            cible = exploration._sous.but
            ctx = replace(ctx, odom=Odometrie(tick, Pose2D(*cible)))  # navigation idéale
        ctx = replace(ctx, t=tick + 1)
    else:
        pytest.fail("exploration n'a pas terminé")
    assert pas.etat == "termine"
    assert "partielle" in pas.detail
    points = exploration.resume_exploration()["points"]
    assert len(points) == 4
    assert all(p["etat"] == "visite" for p in points)
    assert all(math.dist((a["x"], a["y"]), (b["x"], b["y"])) >= 1.1
               for i, a in enumerate(points) for b in points[i + 1:])
    assert sum(d == "observer" for d, _ in ia.appels) == 5
    choix = [a for d, a in ia.appels if d == "choisir_exploration"]
    assert len(choix) == 4
    assert points[0]["x"] == choix[0]["candidats"][1]["x"]
    assert exploration.pas(ctx).etat == "termine"  # pas de relance après terminaison


def test_exploration_decouvre_progressivement_nouveaux_points(synchrone):
    carte = carte_libre(140)
    carte.cellules[:, 80:] = -1
    exploration = E.Explorer({"type": "explorer", "points_max": 3}, IA())
    ctx = contexte(carte=carte)
    premier = exploration.pas(ctx)
    assert premier.etat == "route" and exploration._sous is not None
    x_initial_max = max(p["x"] for p in exploration._prochains)
    # Après déplacement les capteurs étendent la zone mesurée.
    cible = exploration._sous.but
    carte2 = replace(carte, t=1, cellules=carte.cellules.copy())
    carte2.cellules[:, 80:110] = 0
    ctx = replace(ctx, t=1, carte=carte2, odom=Odometrie(1, Pose2D(*cible)))
    exploration.pas(ctx)
    assert any(p["x"] > x_initial_max for p in exploration._prochains)
    exploration.fermer()


@pytest.mark.parametrize("index", [-1, 100, True, "0", None])
def test_choix_gx10_invalide_ne_deplace_pas(synchrone, index):
    exploration = E.Explorer({"type": "explorer"}, IA(choix=index))
    pas = exploration.pas(contexte())
    assert pas.etat == "echec" and "GX10 invalide" in pas.detail
    assert exploration._sous is None
    assert pas.vitesse == 0


def test_vision_en_attente_non_bloquante_annulable(synchrone):
    futur = Future()
    exploration = E.Explorer({"type": "explorer"}, IA(observations=[futur]))
    assert exploration.pas(contexte()).vitesse == 0
    exploration.fermer()
    assert futur.cancelled()
    assert exploration.pas(contexte(t=1)).etat == "echec"


def test_vision_timeout_et_duree_ne_declarent_pas_objet_trouve(synchrone):
    futur = Future()
    exploration = E.Explorer({"type": "explorer", "objet": "porte"},
                             IA(observations=[futur]))
    exploration.pas(contexte())
    pas = exploration.pas(contexte(t=46))
    assert pas.etat == "echec" and futur.cancelled()
    exploration = E.Explorer({"type": "explorer", "objet": "porte", "duree_s": 10}, IA())
    exploration.pas(contexte())
    assert exploration.pas(contexte(t=10)).etat == "echec"
    assert exploration.resume_exploration()["points"][0]["etat"] == "interrompu"


def test_planification_lente_ne_bloque_pas_et_annulation_ignore_resultat():
    entre, libre = threading.Event(), threading.Event()

    class Lent(LigneDroite):
        def planifier(self, carte, pose, cible):
            entre.set()
            libre.wait(3)
            return super().planifier(carte, pose, cible)

    exploration = E.Explorer({"type": "explorer"}, IA())
    try:
        debut = time.monotonic()
        pas = exploration.pas(contexte(nav=Lent()))
        assert time.monotonic() - debut < 0.2
        assert pas.vitesse == 0 and entre.wait(1)
        debut = time.monotonic()
        exploration.fermer()
        assert time.monotonic() - debut < 0.2
        assert exploration.pas(contexte(t=1)).etat == "echec"
        assert exploration._sous is None
    finally:
        libre.set()
        exploration.fermer()


def test_carte_change_pendant_decision_gx10_force_replanification(synchrone):
    ia = IA()
    futur = Future()
    original = ia.lancer_ia
    ia.lancer_ia = lambda demande, argument: (futur if demande == "choisir_exploration"
                                             else original(demande, argument))
    exploration = E.Explorer({"type": "explorer"}, ia)
    ctx = contexte()
    assert exploration.pas(ctx).vitesse == 0
    ctx.carte.cellules[:] = 100
    futur.set_result({"index": 0})
    pas = exploration.pas(replace(ctx, t=1))
    assert pas.vitesse == 0 and exploration._sous is None
    assert "carte modifiée" in pas.detail
    exploration.fermer()


def test_navigation_connue_stoppe_ancien_chemin_et_arc_sortant():
    carte = carte_libre()
    with ThreadPoolExecutor(max_workers=1) as pool:
        nav = E._NavigationConnue(LigneDroite(), pool, 0.325, 0.4, carte)
        route = chemin([[0, 0], [2, 0]])
        carte.cellules[:, 60] = 100
        cmd = nav.commande(Odometrie(0, Pose2D()), route, np.empty((0, 2)), 0.35)
        assert cmd.bloque and cmd.vitesse == 0
        carte = carte_libre()
        carte.cellules[53:, :] = -1
        nav.carte = carte
        nav.navigation = SimpleNamespace(commande=lambda *a: CommandeLocale(1.0, 0.0))
        cmd = nav.commande(Odometrie(0, Pose2D(0, -0.1, math.pi / 2)),
                           chemin([[0, -0.1], [1, -0.1]]), np.empty((0, 2)), 1)
        assert cmd.bloque and "arc" in cmd.raison


def test_recherche_seule_arrete_sur_objet_sans_navigation(synchrone):
    ia = IA(observations=[{"points": [{"sol": [2, 0]}]}])
    exploration = E.Explorer({"type": "explorer", "objet": "chaise", "approcher": False}, ia)
    pas = exploration.pas(contexte())
    assert pas.etat == "termine" and "repéré" in pas.detail
    assert exploration._sous is None


def test_approche_utilise_pose_capture_et_rejette_detection_expiree(synchrone):
    attente = Future()
    ia = IA(observations=[attente])
    exploration = E.Explorer({"type": "explorer", "objet": "chaise"}, ia)
    exploration.pas(contexte(t=10))
    attente.set_result({"points": [{"sol": [2, 0], "pose_odom": [1, 1, math.pi / 2],
                                    "image_t": 10}]})
    pas = exploration.pas(contexte(t=11, pose=Pose2D(1, 0)))
    assert pas.etat == "route" and exploration._approche
    assert exploration._sous.action["x"] == pytest.approx(1)
    assert exploration._sous.action["y"] == pytest.approx(2.3)
    exploration.fermer()
    ia = IA(observations=[{"points": [{"sol": [0.1, 0], "image_t": -100}]}])
    exploration = E.Explorer({"type": "explorer", "objet": "chaise"}, ia)
    pas = exploration.pas(contexte())
    assert pas.etat == "route" and not exploration._approche
    exploration.fermer()


def test_mission_ideale_cinematique_arrive_et_reobserve(synchrone):
    profil = Profil.charger(reglages=None)
    nav = Navigation(profil)
    nav._planificateur.utiliser_numba = False
    ia = IA()
    lancer = ia.lancer_ia

    def choisir(demande, argument):
        if demande == "choisir_exploration":
            candidats = argument["candidats"]
            ia.choix = min(range(len(candidats)),
                           key=lambda i: abs(candidats[i]["y"]) + abs(candidats[i]["x"] - 1.5))
        return lancer(demande, argument)

    ia.lancer_ia = choisir
    exploration = E.Explorer({"type": "explorer", "points_max": 1}, ia)
    pose, historique = Pose2D(), []
    for tick in range(500):
        ctx = contexte(t=tick * 0.1, pose=pose, nav=nav)
        pas = exploration.pas(ctx)
        historique.append(pas)
        if pas.etat in ("termine", "echec"):
            break
        v, w = pas.vitesse, pas.vitesse * math.tan(pas.braquage) / ctx.empattement
        pose = Pose2D(pose.x + v * math.cos(pose.cap) * 0.1,
                      pose.y + v * math.sin(pose.cap) * 0.1, pose.cap + w * 0.1)
    else:
        pytest.fail("mission non terminée en 50 secondes simulées")
    assert pas.etat == "termine"
    assert math.hypot(pose.x, pose.y) > 0.7
    assert any(p.vitesse > 0.05 for p in historique)
    assert exploration.resume_exploration()["points"][0]["etat"] == "visite"
    assert sum(d == "observer" for d, _ in ia.appels) == 2


def test_choix_gx10_expire_et_ne_redemarre_pas(synchrone):
    ia = IA()
    futur = Future()
    lancer = ia.lancer_ia
    ia.lancer_ia = lambda demande, argument: (futur if demande == "choisir_exploration"
                                             else lancer(demande, argument))
    exploration = E.Explorer({"type": "explorer"}, ia)
    assert exploration.pas(contexte()).vitesse == 0
    pas = exploration.pas(contexte(t=31))
    assert pas.etat == "echec" and "GX10 expiré" in pas.detail
    assert futur.cancelled()
    assert exploration.pas(contexte(t=32)) is pas
    assert not exploration.resume_exploration()["points"]


def test_obstacle_nouveau_en_route_arrete_puis_echoue_sans_boucle(synchrone):
    exploration = E.Explorer({"type": "explorer", "objet": "chaise", "duree_s": 30}, IA())
    ctx = contexte()
    exploration.pas(ctx)
    assert exploration._sous is not None
    carte = replace(ctx.carte, cellules=np.full_like(ctx.carte.cellules, 100))
    etats = []
    for tick in range(1, 32):
        pas = exploration.pas(replace(ctx, carte=carte, t=tick))
        etats.append(pas)
        assert pas.vitesse == 0
        if pas.etat in ("termine", "echec"):
            break
    assert pas.etat == "echec"
    assert exploration._sous is None
    assert all(p["etat"] == "inaccessible" for p in exploration.resume_exploration()["points"])
