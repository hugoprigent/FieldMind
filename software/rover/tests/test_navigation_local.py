"""Pilote local (poursuite pure + arcs Ackermann), puis chaîne complète dans la salle simulée :
odométrie lidar, carte, stéréo sur images rendues, planificateur et pilote local."""

from __future__ import annotations

import math
import time
import tomllib

import cv2
import numpy as np
import pytest
from budget import dans_le_budget

from fieldmind.config import FICHIER_PROFIL, Profil, braquage_max
from fieldmind.estimation.cartographie import Cartographe
from fieldmind.estimation.odometrie import Odometre
from fieldmind.navigation.local import PiloteLocal, filtrer_isoles
from fieldmind.navigation.planificateur import Planificateur
from fieldmind.perception.stereo import Stereo
from fieldmind.simulation.monde import Monde, deplacer
from fieldmind.types import (
    Chemin,
    CommandeRoues,
    ImageCamera,
    Imu,
    Obstacles,
    Odometrie,
    Pose2D,
    Scan,
    normaliser_angle,
)


@pytest.fixture(scope="module")
def profil() -> Profil:
    with open(FICHIER_PROFIL, "rb") as f:
        base = tomllib.load(f)
    base["cameras"]["calibration"] = "/inexistant/stereo.npz"
    return Profil(base, {}, None)


def ligne(x0: float, x1: float, y: float = 0.0) -> Chemin:
    xs = np.arange(x0, x1 + 1e-9, 0.1)
    points = np.column_stack((xs, np.full(len(xs), y)))
    return Chemin(t=0.0, points=points, but=(x1, y), longueur_m=x1 - x0)


def segment(x0: float, y0: float, x1: float, y1: float, pas: float = 0.03) -> np.ndarray:
    n = max(2, int(math.dist((x0, y0), (x1, y1)) / pas))
    return np.column_stack((np.linspace(x0, x1, n), np.linspace(y0, y1, n)))


def vers_base(pose: Pose2D, points: np.ndarray) -> np.ndarray:
    c, s = math.cos(pose.cap), math.sin(pose.cap)
    dx, dy = points[:, 0] - pose.x, points[:, 1] - pose.y
    return np.column_stack((c * dx + s * dy, -s * dx + c * dy))


def touche(pose: Pose2D, points: np.ndarray) -> bool:
    """L'emprise du rover (−0,11..0,46 × ±0,225) contient-elle un point (repère odom) ?"""
    local = vers_base(pose, points)
    return bool(np.any((local[:, 0] > -0.11) & (local[:, 0] < 0.46)
                       & (np.abs(local[:, 1]) < 0.225)))


def test_ligne_droite_libre(profil: Profil) -> None:
    pilote = PiloteLocal(profil)
    c = pilote.commande(Odometrie(0.0, Pose2D(), vitesse=0.4), ligne(0, 5), None, 0.5)
    assert c.vitesse == pytest.approx(0.5)
    assert abs(c.braquage) < 1e-6
    assert not c.arrive and not c.bloque and c.raison == ""
    assert c.distance_restante == pytest.approx(5.0, abs=0.05)
    # Écart latéral : on braque vers le chemin, sans dépasser le braquage de la roue virtuelle.
    c = pilote.commande(Odometrie(0.0, Pose2D(0.0, -0.8, 0.0)), ligne(0, 5), None, 0.5)
    assert 0 < c.braquage <= braquage_max(profil) + 1e-9
    # Formats d'obstacles acceptés.
    vide = Obstacles(0.0, "lidar", np.zeros((0, 2), np.float32))
    for obstacles in (vide, np.zeros((0, 2)), [vide, np.zeros((0, 2))], None):
        assert pilote.commande(Pose2D(), ligne(0, 5), obstacles, 0.5).vitesse > 0


def test_arrivee_et_ralentissement(profil: Profil) -> None:
    pilote = PiloteLocal(profil)
    chemin = ligne(0, 3)
    loin = pilote.commande(Pose2D(0.0, 0.0, 0.0), chemin, None, 0.5)
    pres = pilote.commande(Pose2D(2.6, 0.0, 0.0), chemin, None, 0.5)
    assert pres.vitesse < loin.vitesse
    fin = pilote.commande(Pose2D(2.9, 0.02, 0.0), chemin, None, 0.5)
    assert fin.arrive and fin.vitesse == 0.0 and fin.raison == "arrive"


def test_indicatifs_orientent_sans_bloquer(profil: Profil) -> None:
    chemin = ligne(0, 5)
    amas = np.array([[1.1 + dx, 0.15 + dy] for dx in (-0.03, 0, 0.03)
                    for dy in (-0.03, 0, 0.03)])
    libre = PiloteLocal(profil).commande(Pose2D(), chemin, None, 0.5)
    indicatif = PiloteLocal(profil).commande(Pose2D(), chemin, None, 0.5, amas)
    assert indicatif.braquage < libre.braquage  # l'amas à gauche incite à passer à droite
    assert indicatif.vitesse > 0 and not indicatif.bloque
    # Même un mur de stéréo ne force ni arrêt ni marche arrière.
    mur = segment(0.75, -2, 0.75, 2)
    doux = PiloteLocal(profil).commande(Pose2D(), chemin, None, 0.5, mur)
    dur = PiloteLocal(profil).commande(Pose2D(), chemin, mur, 0.5)
    assert doux.vitesse > 0 and not doux.bloque
    assert dur.vitesse < 0  # le même mur vu au lidar impose un dégagement


def test_points_isoles_invalides_et_doublons_ignores(profil: Profil) -> None:
    bruit = np.array([[0.8, 0.0], [1.2, 0.1], [np.nan, 0], [np.inf, 0], [0.8, 0.0]])
    assert len(filtrer_isoles(bruit)) == 0
    commande = PiloteLocal(profil).commande(Pose2D(), ligne(0, 5), None, 0.5, bruit)
    assert commande.braquage == 0 and commande.vitesse == pytest.approx(0.5)


def test_evite_un_obstacle_imprevu(profil: Profil) -> None:
    """Une boîte de 50 cm, absente de la carte, est posée sur le chemin : le pilote la
    contourne et arrive, sans que l'emprise ne la touche."""
    boite = np.vstack([segment(2.0, -0.25, 2.0, 0.25), segment(2.5, -0.25, 2.5, 0.25),
                       segment(2.0, -0.25, 2.5, -0.25), segment(2.0, 0.25, 2.5, 0.25)])
    pilote = PiloteLocal(profil)
    chemin = ligne(0, 5)
    pose = Pose2D()
    raisons = set()
    for _ in range(300):
        c = pilote.commande(Odometrie(0.0, pose, vitesse=0.3), chemin, vers_base(pose, boite),
                            0.5)
        raisons.add(c.raison)
        assert not c.bloque
        if c.arrive:
            break
        for _ in range(10):
            pose = deplacer(pose, c.vitesse, c.braquage, 0.01)
            assert not touche(pose, boite)
    assert c.arrive
    assert "evitement" in raisons


def test_but_derriere_en_marche_arriere(profil: Profil) -> None:
    """But à 1 m derrière, place libre : marche arrière directe (pas de demi-tour), braquée du
    bon côté ; mur juste derrière : pas de marche arrière directe."""
    pilote = PiloteLocal(profil)
    chemin = Chemin(t=1.0, points=np.array([[0.0, 0.0], [-0.5, 0.1], [-1.0, 0.3]]), but=(-1.0, 0.3))
    c = pilote.commande(Pose2D(), chemin, np.zeros((0, 2)), 0.5)
    assert c.raison == "marche_arriere" and c.vitesse < 0 and c.courbure > 0
    pilote.reinitialiser()
    c = pilote.commande(Pose2D(), chemin, segment(-0.3, -1.0, -0.3, 1.0), 0.5)
    assert c.raison != "marche_arriere"


def test_impasse_recul_puis_blocage(profil: Profil) -> None:
    pilote = PiloteLocal(profil)
    chemin = ligne(0, 5)
    mur_avant = segment(0.75, -1.5, 0.75, 1.5)
    c = pilote.commande(Pose2D(), chemin, mur_avant, 0.5)
    assert c.raison == "recul_degagement" and c.vitesse < 0 and not c.bloque
    # Le recul continue sur sa lancée tant que la distance de recul n'est pas faite.
    c2 = pilote.commande(Pose2D(-0.1, 0.0, 0.0), chemin, mur_avant + np.array([0.1, 0.0]), 0.5)
    assert c2.raison == "recul_degagement" and c2.vitesse < 0
    # Mur devant et mur derrière : bloqué.
    pilote.reinitialiser()
    mur_arriere = segment(-0.35, -1.5, -0.35, 1.5)
    c3 = pilote.commande(Pose2D(), chemin, np.vstack([mur_avant, mur_arriere]), 0.5)
    assert c3.bloque and c3.vitesse == 0.0 and c3.raison == "aucun_arc_libre"


def test_temps_de_calcul(profil: Profil) -> None:
    rng = np.random.default_rng(0)
    nuage = rng.uniform(-4, 4, (600, 2))
    nuage = nuage[np.hypot(nuage[:, 0], nuage[:, 1]) > 1.2]
    pilote = PiloteLocal(profil)
    durees = []
    for _ in range(20):
        debut = time.perf_counter()
        pilote.commande(Pose2D(), ligne(0, 5), nuage, 0.5)
        durees.append(time.perf_counter() - debut)
    print(f"\npilote local, {len(nuage)} points : {1000 * np.median(durees):.2f} ms")
    assert dans_le_budget(float(np.median(durees)), 0.02)


def test_salle_coin_a_coin(profil: Profil) -> None:
    """Du coin sud-ouest au coin nord-est de la salle, à travers les obstacles bas (poutre et
    carton bas sur la diagonale, invisibles au lidar) : odométrie lidar + gyroscope, carte,
    stéréo à 2 Hz sur les images rendues, planification toutes les 2 s, pilote à 10 Hz. La
    vraie emprise ne touche jamais rien (obstacles bas compris), stéréo bloquante."""
    profil = Profil(profil.dict(), {}, None)
    profil.regler("securite.stereo_bloquante", True)
    monde = Monde(0, "salle")
    depart = monde.depart
    rng = np.random.default_rng(2)
    odometre, cartographe = Odometre(profil), Cartographe(profil)
    planificateur, pilote, stereo = Planificateur(profil), PiloteLocal(profil), Stereo(profil)
    origine = depart.inverse()
    but_monde = Pose2D(*monde.but, 0.0)
    but_odom = origine.composer(but_monde)
    but = (but_odom.x, but_odom.y)
    pose, pose_tour = depart, depart
    t, dt = 0.0, 0.01
    vitesse = braquage = 0.0
    chemin: Chemin | None = None
    odom: Odometrie | None = None
    dernier_plan = -math.inf
    lidar = np.zeros((0, 2))
    vus = Obstacles(0.0, "stereo", np.zeros((0, 2), np.float32))
    marge_min = math.inf
    arrive = False
    durees_plan: list[float] = []
    while t < 90.0 and not arrive:
        suivante = deplacer(pose, vitesse * 0.85, braquage * 0.97, dt)
        omega = normaliser_angle(suivante.cap - pose.cap) / dt
        pose = suivante
        t += dt
        k = int(round(t / dt))
        if k % 5 == 0:
            marge_min = min(marge_min, monde.distance_obstacles(pose, profil))
        if k % 2 == 0:
            odometre.imu(Imu(t, (0.0, 0.0, 1.0), (0.0, 0.0, omega + 0.01 + rng.normal(0, 0.005))))
        if k % 10:
            continue
        angles, distances = monde.lancer_lidar(pose, profil, pose_debut=pose_tour, rng=rng)
        pose_tour = pose
        scan = Scan(t, angles, distances, 0.1)
        odom = odometre.scan(scan)
        assert odom is not None
        cartographe.integrer_scan(scan, odom.pose)
        lidar = scan.points().astype(np.float64)
        if k % 50 == 0:
            images = []
            for camera in ("gauche", "droite"):
                bgr = monde.rendre_camera(pose, camera, 640, 360, profil, rng=rng)
                jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()
                images.append(ImageCamera(t, camera, jpeg, 640, 360))
            resultat = stereo.traiter(images[0], images[1], pose=odom.pose)
            assert resultat is not None
            vus = resultat.obstacles
            cartographe.integrer_obstacles(vus, odom.pose)
        if t - dernier_plan > 2.0 or chemin is None:
            debut = time.perf_counter()
            nouveau = planificateur.planifier(cartographe.carte(), odom.pose, but)
            durees_plan.append(time.perf_counter() - debut)
            dernier_plan = t
            chemin = nouveau or chemin
        if chemin is None:
            vitesse = 0.0
            continue
        memoire = cartographe.obstacles_proches(odom.pose, 3.0, bas_seulement=True)
        c = pilote.commande(odom, chemin, [lidar, vus, memoire], 0.5)
        if c.bloque:
            dernier_plan = -math.inf
        arrive = c.arrive
        vitesse, braquage = c.vitesse, c.braquage
        pwm = round(35 * vitesse)
        odometre.commande(CommandeRoues(t, pwm, pwm, 0.0, vitesse, braquage))
    erreur = math.dist((pose.x, pose.y), monde.but)
    print(f"\nsalle coin à coin : {t:.1f} s, arrivé à {erreur:.2f} m du but, marge minimale "
          f"{marge_min:.2f} m, planification {1000 * np.median(durees_plan):.1f} ms (médiane)")
    assert arrive
    assert erreur < 0.4
    assert marge_min > 0.05  # aucune collision, obstacles bas compris
    assert t < 60.0
