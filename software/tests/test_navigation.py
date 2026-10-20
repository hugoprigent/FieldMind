"""Tests de la planification, du suivi et d'une mission complète sur le banc d'essai."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from services.navigation.grille import L_MAX, GrilleOccupation
from services.navigation.planificateur import (
    EchecPlanification,
    ParametresPlanification,
    construire_carte_couts,
    planifier,
    prechauffer,
)
from services.navigation.suivi import SuiviTrajectoire
from services.protocole.fiche_rover import FicheRover
from services.protocole.scenario import Limites
from simulation.banc_essai import BancEssai
from simulation.monde import charger_monde

RACINE = Path(__file__).resolve().parents[1]
P = ParametresPlanification()


@pytest.fixture(scope="module", autouse=True)
def _compiler() -> None:
    prechauffer()


@pytest.fixture(scope="module")
def fiche() -> FicheRover:
    return FicheRover.charger(RACINE / "config" / "rover_fieldmind.json")


def grille_avec_mur(ouverture: bool) -> GrilleOccupation:
    """Mur nord-sud en x = 0, avec ou sans passage de 2 m en y ∈ [-1, 1]."""
    grille = GrilleOccupation(Limites(xmin=-10, xmax=10, ymin=-10, ymax=10), 0.1)
    grille.observee[:] = True
    for lig in range(grille.hauteur):
        _, y = grille.centre(0, lig)
        if ouverture and -1.0 < y < 1.0:
            continue
        c, _ = (int(v) for v in grille.cellule(np.array(0.0), np.array(y)))
        grille.log_odds[lig, c - 1 : c + 2] = L_MAX
    return grille


def test_chemin_passe_par_l_ouverture_avec_marge() -> None:
    carte = construire_carte_couts(grille_avec_mur(ouverture=True), P)
    plan = planifier(carte, (-6.0, -6.0), (6.0, 6.0), P)
    assert math.dist(plan.points[-1], (6.0, 6.0)) < 0.3
    assert min(carte.distance_en(*p) for p in plan.points) >= P.rayon_gonflement - 1e-6
    passage = [p for p in plan.points if abs(p[0]) < 0.3]
    assert passage
    assert all(abs(p[1]) < 1.0 for p in passage)


def test_aucun_chemin_si_mur_ferme() -> None:
    carte = construire_carte_couts(grille_avec_mur(ouverture=False), P)
    with pytest.raises(EchecPlanification):
        planifier(carte, (-6.0, 0.0), (6.0, 0.0), P)


def test_objectif_dans_un_obstacle_deplace_vers_le_libre() -> None:
    carte = construire_carte_couts(grille_avec_mur(ouverture=True), P)
    plan = planifier(carte, (-6.0, -6.0), (0.0, 5.0), P)
    assert 0.3 < plan.objectif_deplace < 1.0
    assert carte.distance_en(*plan.objectif) >= P.rayon_gonflement


def test_inconnu_traversable() -> None:
    grille = GrilleOccupation(Limites(xmin=-10, xmax=10, ymin=-10, ymax=10), 0.1)
    plan = planifier(construire_carte_couts(grille, P), (-5.0, 0.0), (5.0, 0.0), P)
    assert plan.longueur == pytest.approx(10.0, abs=0.3)


def test_poursuite_pure_braque_vers_le_chemin(fiche: FicheRover) -> None:
    suivi = SuiviTrajectoire(fiche)
    suivi.definir_chemin([(0.25 * k, 1.0) for k in range(40)])
    consigne = suivi.consigne(0.0, 0.0, 0.0, 0.5, math.inf)
    assert consigne.braquage > 0, "chemin à gauche : braquer à gauche"
    assert consigne.vitesse > 0


def test_poursuite_pure_demi_tour_si_cible_derriere(fiche: FicheRover) -> None:
    suivi = SuiviTrajectoire(fiche)
    suivi.definir_chemin([(-0.25 * k, 0.1) for k in range(40)])
    consigne = suivi.consigne(0.0, 0.0, 0.0, 0.0, math.inf)
    assert abs(consigne.braquage) == pytest.approx(fiche.limites.braquage_max)
    assert abs(consigne.angle_cible) > math.pi / 2


@pytest.mark.lent
def test_mission_complete_jusqu_a_la_clairiere(fiche: FicheRover) -> None:
    monde = charger_monde(RACINE / "simulation" / "scenarios" / "foret_demo.json")
    banc = BancEssai(fiche, monde)
    banc.avancer(1.0)
    lieu = monde.scenario.lieu("clairiere")
    assert lieu is not None
    assert banc.cerveau.definir_objectif(lieu.x, lieu.y) is None
    banc.avancer(150.0, lambda b: b.cerveau.etat in ("atteint", "echec"))
    v = banc.verite()
    assert banc.cerveau.etat == "atteint"
    assert math.hypot(v.x - lieu.x, v.y - lieu.y) < 0.6
    assert v.collisions == 0
