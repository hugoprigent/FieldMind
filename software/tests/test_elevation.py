"""Carte des hauteurs et traversabilité (caméra stéréo), sur des nuages synthétiques."""

from __future__ import annotations

import math

import numpy as np

from services.perception.elevation import LIBRE, MARCHE, PENTE, CarteHauteurs
from services.protocole.scenario import Limites

LIMITES = Limites(xmin=-10, xmax=10, ymin=-10, ymax=10)
RNG = np.random.default_rng(1)


def _sol(x0: float, x1: float, y0: float, y1: float, z=lambda x, y: 0 * x, pas: float = 0.03):
    x, y = np.meshgrid(np.arange(x0, x1, pas), np.arange(y0, y1, pas))
    x, y = x.ravel(), y.ravel()
    return np.column_stack([x, y, z(x, y)])


def _distances(points: np.ndarray, d: float) -> np.ndarray:
    return np.full(len(points), d)


def _deux_vues(
    carte: CarteHauteurs, points: np.ndarray, distances: np.ndarray, rayon: float
) -> None:
    """Un obstacle se confirme sur deux vues successives."""
    for t in (0.0, 0.5):
        carte.ajouter_nuage(points, distances, t=t, recaler=False)
        carte.classer(0.0, 0.0, rayon)


def test_sol_plat_bruite_reste_libre() -> None:
    carte = CarteHauteurs(LIMITES)
    sol = _sol(0.5, 4.0, -1.5, 1.5)
    sol[:, 2] += RNG.normal(0, 0.01, len(sol))
    carte.ajouter_nuage(sol, np.hypot(sol[:, 0], sol[:, 1]), t=0.0)
    carte.classer(0.0, 0.0, 6.0)
    assert not carte.obstacles().any()
    assert (carte.etat == LIBRE).sum() > 800


def test_tronc_couche_est_une_marche() -> None:
    carte = CarteHauteurs(LIMITES)
    sol = _sol(0.5, 4.0, -1.5, 1.5)
    # Tronc couché de 26 cm de diamètre, en travers, à 2,5 m.
    tronc = _sol(2.4, 2.66, -1.0, 1.0, pas=0.02)
    tronc[:, 2] = 0.26 * np.sin(np.pi * (tronc[:, 0] - 2.4) / 0.26)
    nuage = np.vstack([sol, tronc])
    carte.ajouter_nuage(nuage, np.hypot(nuage[:, 0], nuage[:, 1]), t=0.0)
    carte.classer(0.0, 0.0, 6.0)
    assert not carte.obstacles().any(), "une seule vue ne suffit pas"
    _deux_vues(carte, nuage, np.hypot(nuage[:, 0], nuage[:, 1]), 6.0)
    lig, col = np.nonzero(carte.etat == MARCHE)
    x = carte.origine[0] + (col + 0.5) * carte.resolution
    assert lig.size > 10
    assert np.all((x > 2.2) & (x < 2.9)), "les obstacles sont sur le tronc, pas ailleurs"
    libre = carte.distance_libre(0.0, 0.0, 0.0, 0.3, 0.35, 4.0)
    assert 1.9 < libre < 2.2


def test_artefact_d_une_seule_image_ignore() -> None:
    """Une rangée mal appariée fait une bosse dans une image, pas dans la suivante."""
    carte = CarteHauteurs(LIMITES)
    sol = _sol(0.5, 3.0, -1.0, 1.0)
    bosse = sol.copy()
    bosse[:, 2] += 0.3 * ((bosse[:, 0] > 2.0) & (bosse[:, 0] < 2.3))
    carte.ajouter_nuage(bosse, _distances(bosse, 2.0), t=0.0, recaler=False)
    carte.classer(0.0, 0.0, 4.0)
    carte.ajouter_nuage(sol, _distances(sol, 2.0), t=0.5, recaler=False)
    carte.classer(0.0, 0.0, 4.0)
    assert not carte.obstacles().any()


def test_vue_lointaine_ne_fait_pas_d_obstacle_et_ne_remplace_pas_une_vue_proche() -> None:
    carte = CarteHauteurs(LIMITES)
    sol = _sol(1.0, 3.0, -1.0, 1.0)
    carte.ajouter_nuage(sol, _distances(sol, 2.0), t=0.0)
    # Même zone revue de loin, avec une bosse fantôme de 40 cm (bruit de distance).
    fantome = sol.copy()
    fantome[:, 2] += 0.4 * (fantome[:, 0] > 2.0)
    carte.ajouter_nuage(fantome, _distances(fantome, 6.0), t=1.0, recaler=False)
    carte.classer(0.0, 0.0, 6.0)
    assert not carte.obstacles().any()
    # Une zone vue seulement de loin n'est jamais un obstacle.
    loin = _sol(5.0, 6.0, -0.5, 0.5)
    loin[:, 2] += 0.4 * (loin[:, 0] > 5.5)
    carte.ajouter_nuage(loin, _distances(loin, 5.5), t=2.0, recaler=False)
    carte.classer(0.0, 0.0, 8.0)
    assert not carte.obstacles().any()


def test_pente_forte_detectee_pente_douce_non() -> None:
    for pente_deg, attendu in ((10.0, False), (35.0, True)):
        carte = CarteHauteurs(LIMITES)
        k = math.tan(math.radians(pente_deg))
        sol = _sol(0.5, 2.5, -1.0, 1.0, z=lambda x, y, k=k: k * np.maximum(x - 1.5, 0))
        _deux_vues(carte, sol, _distances(sol, 1.5), 4.0)
        assert bool((carte.etat == PENTE).any()) is attendu, pente_deg


def test_cellule_isolee_ignoree_et_disque_du_rover_libere() -> None:
    carte = CarteHauteurs(LIMITES)
    sol = _sol(0.5, 3.0, -1.0, 1.0)
    pic = np.array([[1.55, 0.05, 0.5], [1.56, 0.06, 0.5]])
    nuage = np.vstack([sol, pic])
    _deux_vues(carte, nuage, _distances(nuage, 1.5), 4.0)
    assert not carte.obstacles().any(), "une seule cellule haute est du bruit"
    bloc = _sol(1.5, 1.8, -0.2, 0.2, pas=0.02)
    bloc[:, 2] = 0.3
    nuage = np.vstack([sol, bloc])
    for t in (1.0, 1.5):
        carte.ajouter_nuage(nuage, _distances(nuage, 1.5), t=t)
        carte.classer(0.0, 0.0, 4.0)
    assert carte.obstacles().any()
    carte.liberer_disque(1.65, 0.0, 0.5)
    assert not carte.obstacles().any()


def test_recalage_de_l_altitude() -> None:
    carte = CarteHauteurs(LIMITES)
    sol = _sol(0.5, 3.0, -1.0, 1.0)
    carte.ajouter_nuage(sol, _distances(sol, 1.5), t=0.0)
    decale = sol.copy()
    decale[:, 2] += 0.04  # altitude intégrée qui a dérivé de 4 cm
    assert abs(carte.ajouter_nuage(decale, _distances(decale, 1.5), t=1.0) - 0.04) < 0.01
