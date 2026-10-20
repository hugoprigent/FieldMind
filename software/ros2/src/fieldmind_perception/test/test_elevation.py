import math

import numpy as np
from fieldmind_perception.elevation import INCONNU, LIBRE, OBSTACLE, CarteElevation


def sol(x0, x1, y0, y1, hauteur, pas=0.03):
    """Nuage dense posé sur la surface z = hauteur(x, y)."""
    x, y = np.meshgrid(np.arange(x0, x1, pas), np.arange(y0, y1, pas))
    x, y = x.ravel(), y.ravel()
    return np.column_stack([x, y, hauteur(x, y)])


def etat(carte, c, x, y):
    iy, ix, _ = carte.indices(np.array([x]), np.array([y]))
    return c.etat[iy[0], ix[0]]


def test_pente_praticable_libre_et_pente_raide_obstacle():
    carte = CarteElevation()
    douce = math.tan(math.radians(12))
    raide = math.tan(math.radians(35))
    carte.ajouter_nuage(sol(0, 4, -2, 2, lambda x, y: douce * x), t=0)
    carte.ajouter_nuage(sol(5, 8, -2, 2, lambda x, y: raide * (x - 5)), t=0)
    c = carte.classer(t=0)
    assert etat(carte, c, 2.0, 0.0) == LIBRE
    assert etat(carte, c, 6.5, 0.0) == OBSTACLE
    assert etat(carte, c, -3.0, 0.0) == INCONNU


def test_rocher_et_tronc_sont_des_obstacles_meme_en_pente():
    carte = CarteElevation()
    pente = math.tan(math.radians(10))
    terrain = sol(0, 6, -2, 2, lambda x, y: pente * x)
    rocher = sol(3, 3.4, -0.2, 0.2, lambda x, y: pente * x + 0.25)
    carte.ajouter_nuage(np.vstack([terrain, rocher]), t=0)
    c = carte.classer(t=0)
    assert etat(carte, c, 3.2, 0.0) == OBSTACLE
    assert etat(carte, c, 1.5, 1.0) == LIBRE


def test_branches_hautes_ignorees():
    carte = CarteElevation(hauteur_max=1.2)
    terrain = sol(0, 4, -2, 2, lambda x, y: 0 * x)
    branches = sol(1, 2, -0.5, 0.5, lambda x, y: 0 * x + 2.0)
    carte.ajouter_nuage(np.vstack([terrain, branches]), t=0)
    c = carte.classer(t=0)
    assert etat(carte, c, 1.5, 0.0) == LIBRE


def test_lidar_sur_le_sol_filtre_et_lidar_sur_un_tronc_obstacle():
    carte = CarteElevation()
    pente = math.tan(math.radians(12))
    carte.ajouter_nuage(sol(0, 6, -2, 2, lambda x, y: pente * x), t=0)
    # Le faisceau du lidar (parallèle au rover, 0,32 m) touche la montée à ~1,5 m.
    impact_sol = np.array([[1.5, 0.0, pente * 1.5 + 0.02]])
    impact_tronc = np.array([[4.0, 1.0, pente * 4.0 + 0.32]])
    impact_hors_camera = np.array([[-3.0, 0.0, 0.32]])
    carte.ajouter_lidar(np.vstack([impact_sol, impact_tronc, impact_hors_camera]), t=0)
    c = carte.classer(t=0, robot=(0.0, 0.0))
    assert etat(carte, c, 1.5, 0.0) == LIBRE
    assert etat(carte, c, 4.0, 1.0) == OBSTACLE
    # Loin de tout sol vu : peut-être une montée, on ne conclut pas.
    assert etat(carte, c, -3.0, 0.0) == INCONNU
    filtre = carte.est_sol(c, np.vstack([impact_sol, impact_tronc, impact_hors_camera]))
    assert filtre.tolist() == [True, False, False]


def test_lidar_pres_du_sol_vu_ou_tout_pres_du_rover():
    carte = CarteElevation()
    carte.ajouter_nuage(sol(0, 4, -2, 2, lambda x, y: 0 * x), t=0)
    tronc_pres_du_sol_vu = [-0.8, 0.0, 0.32]  # à 0,8 m du sol plat vu
    montee_pres_du_sol_vu = [-0.8, 1.0, 0.10]  # le terrain se relève un peu
    tout_pres = [0.3, -3.0, 0.32]  # rien de vu autour, mais à 1 m du rover
    carte.ajouter_lidar(np.array([tronc_pres_du_sol_vu, montee_pres_du_sol_vu, tout_pres]), t=0)
    c = carte.classer(t=0, robot=(0.3, -2.0))
    assert etat(carte, c, -0.8, 0.0) == OBSTACLE
    assert etat(carte, c, -0.8, 1.0) == INCONNU
    assert etat(carte, c, 0.3, -3.0) == OBSTACLE


def test_obstacle_lidar_oublie_apres_la_memoire_lidar():
    carte = CarteElevation(memoire_lidar_s=4.0)
    carte.ajouter_lidar(np.array([[1.0, 0.0, 0.3]]), t=0)
    assert etat(carte, carte.classer(t=1, robot=(0.0, 0.0)), 1.0, 0.0) == OBSTACLE
    assert etat(carte, carte.classer(t=10, robot=(0.0, 0.0)), 1.0, 0.0) == INCONNU


def test_rayons_libres_s_arretent_a_l_obstacle_et_detectent_un_trou():
    carte = CarteElevation()
    plat = lambda x, y: 0 * x  # noqa: E731
    # Devant (+x) : sol vu jusqu'à 1,5 m, fossé masqué de 1,5 à 2,5 m, sol vu au-delà.
    devant = [sol(0.3, 1.5, -1, 1, plat), sol(2.5, 6, -1, 1, plat)]
    # À gauche (+y) : sol vu, mur à 3 m.
    gauche = [sol(-1, 1, 0.3, 5, plat), sol(-1, 1, 3.0, 3.2, lambda x, y: 0 * x + 0.5)]
    carte.ajouter_nuage(np.vstack(devant + gauche), t=0)
    c = carte.classer(t=0)
    angles, libre, trou = carte.rayons(c, 0.0, 0.0, nombre=4)
    devant = int(np.argmin(np.abs(angles)))
    gauche = int(np.argmin(np.abs(angles - math.pi / 2)))
    assert 1.3 < libre[devant] < 1.55
    assert 1.4 < trou[devant] < 1.6
    assert 2.7 < libre[gauche] < 3.0
    assert np.isnan(trou[gauche])


def test_recentrage_conserve_les_observations():
    carte = CarteElevation(taille=12.0)
    rocher = sol(4, 5, -0.5, 0.5, lambda x, y: 0 * x + 0.4)
    carte.ajouter_nuage(np.vstack([sol(3, 6, -1.5, 1.5, lambda x, y: 0 * x), rocher]), t=0)
    carte.recentrer(5.0, 0.0)
    assert carte.centre()[0] > 4
    c = carte.classer(t=0)
    assert etat(carte, c, 4.5, 0.0) == OBSTACLE
    assert etat(carte, c, 3.3, 1.2) == LIBRE


def test_recalage_vertical_corrige_la_derive_d_altitude():
    carte = CarteElevation()
    pente = math.tan(math.radians(8))
    carte.ajouter_nuage(sol(0, 5, -2, 2, lambda x, y: pente * x), t=0)
    # Même sol revu avec une altitude du rover fausse de +0,2 m, plus une zone nouvelle.
    revu = sol(2, 8, -2, 2, lambda x, y: pente * x + 0.2)
    decalage = carte.ajouter_nuage(revu, t=1)
    assert abs(decalage - 0.2) < 0.02
    c = carte.classer(t=1)
    # Sans recalage, la jonction ancien/nouveau sol ferait une marche de 0,2 m.
    for x in np.arange(0.5, 7.5, 0.5):
        assert etat(carte, c, x, 0.0) == LIBRE, x


def test_crete_suivie_d_une_descente_n_est_pas_un_trou():
    carte = CarteElevation()
    # Sol plat jusqu'à 1,5 m, puis descente masquée : le sol revu à 2,5 m est 0,4 m plus bas.
    pres = sol(0.3, 1.5, -1, 1, lambda x, y: 0 * x)
    loin = sol(2.5, 6, -1, 1, lambda x, y: -0.4 - 0.15 * (x - 2.5))
    carte.ajouter_nuage(np.vstack([pres, loin]), t=0)
    c = carte.classer(t=0)
    angles, libre, trou = carte.rayons(c, 0.0, 0.0, nombre=4)
    devant = int(np.argmin(np.abs(angles)))
    assert np.isnan(trou[devant])
    assert 1.3 < libre[devant] < 1.55
