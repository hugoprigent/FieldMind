"""Odométrie lidar + gyroscope sur un rover cinématique roulant dans le monde simulé.

Le « vrai » rover suit un chemin (poursuite pure sur sa pose vraie), à une vitesse réelle de
85 % de la commande (PWM → m/s mal calibré), braquage réel 95 % du commandé, gyroscope biaisé de
0,8 °/s et bruité ; le lidar tourne pendant que le rover avance (tours déformés).
"""

from __future__ import annotations

import itertools
import math
import time
import tomllib
from dataclasses import dataclass, field

import numpy as np
import pytest
from budget import dans_le_budget

from fieldmind.config import FICHIER_PROFIL, Profil, braquage_max
from fieldmind.estimation.odometrie import Odometre
from fieldmind.simulation.monde import Monde, deplacer
from fieldmind.types import CommandeRoues, Imu, Pose2D, Scan, normaliser_angle

EMPATTEMENT = 0.38


def profil_test(**surcharges: object) -> Profil:
    with open(FICHIER_PROFIL, "rb") as f:
        base = tomllib.load(f)
    base["cameras"]["calibration"] = "/inexistant/stereo.npz"
    for cle, valeur in surcharges.items():
        section, nom = cle.split("__")
        base.setdefault(section, {})[nom] = valeur
    return Profil(base, {}, None)


def huit(cx1: float = 6.3, cx2: float = 4.3, cy: float = 3.0, r: float = 1.0,
         tours: int = 2) -> np.ndarray:
    """Deux cercles tangents en (5,3 ; 3,0), parcourus à gauche puis à droite."""
    a = np.linspace(math.pi, 3 * math.pi, 200)
    droite = np.column_stack((cx1 + r * np.cos(a), cy + r * np.sin(a)))
    b = np.linspace(0, -2 * math.pi, 200)
    gauche = np.column_stack((cx2 + r * np.cos(b), cy + r * np.sin(b)))
    return np.vstack([np.vstack([droite, gauche])] * tours)


class Suiveur:
    """Poursuite pure du chemin par le vrai rover (pose vraie)."""

    def __init__(self, chemin: np.ndarray, braquage_limite: float = 0.45) -> None:
        self.braquage_limite = braquage_limite
        points = [chemin[0]]
        for a, b in itertools.pairwise(chemin):
            n = max(1, int(math.dist(a, b) / 0.05))
            points.extend(a + (b - a) * k / n for k in range(1, n + 1))
        self.p = np.array(points)
        self.i = 0

    def __call__(self, pose: Pose2D) -> tuple[float, bool]:
        fin = min(len(self.p), self.i + 40)
        d = np.hypot(self.p[self.i:fin, 0] - pose.x, self.p[self.i:fin, 1] - pose.y)
        self.i += int(np.argmin(d))
        j = min(self.i + 12, len(self.p) - 1)
        lx, ly = pose.vers_local(*self.p[j])
        alpha = math.atan2(ly, lx)
        braquage = math.atan2(2 * EMPATTEMENT * math.sin(alpha), max(math.hypot(lx, ly), 0.3))
        limite = self.braquage_limite
        return float(np.clip(braquage, -limite, limite)), self.i >= len(self.p) - 3


@dataclass
class Bilan:
    parcouru: float
    erreur_m: float
    erreur_cap_deg: float
    pire_m: float
    collisions: int
    durees_ms: np.ndarray
    sources: list[str] = field(default_factory=list)
    odometre: Odometre | None = None


def rouler(scenario: str, depart: Pose2D, chemin: np.ndarray | None, profil: Profil,
           vitesse: float = 0.4, rapport: float = 0.85, biais: float = 0.014,
           attente_s: float = 0.0, duree_max: float = 120.0, bloque: bool = False,
           graine: int = 1) -> Bilan:
    monde = Monde(graine, scenario)
    # Le rover cinématique traverse les obstacles bas (sans effet sur le lidar) ; il ne doit pas
    # traverser ce que voit le lidar.
    visible = Monde(graine, scenario)
    visible.obstacles = [o for o in visible.obstacles if o.genre != "bas"]
    rng = np.random.default_rng(graine)
    odo = Odometre(profil)
    suiveur = Suiveur(chemin, braquage_max(profil)) if chemin is not None else None
    pose, pose_tour = depart, depart
    t, dt = 0.0, 0.01
    parcouru, pire, collisions = 0.0, 0.0, 0
    durees, sources = [], []
    while t < duree_max:
        braquage, fini = suiveur(pose) if suiveur is not None else (0.0, False)
        if fini:
            break
        v_cmd = 0.0 if t < attente_s else vitesse
        v_reelle = 0.0 if bloque else v_cmd * rapport
        k = int(round(t / dt))
        if k % 5 == 0:
            pwm = round(v_cmd * 35)
            odo.commande(CommandeRoues(t, pwm, pwm, 0.0, v_cmd, braquage))
        suivante = deplacer(pose, v_reelle, braquage * 0.95, dt, EMPATTEMENT)
        omega = normaliser_angle(suivante.cap - pose.cap) / dt
        parcouru += math.dist((pose.x, pose.y), (suivante.x, suivante.y))
        pose = suivante
        t += dt
        k += 1
        if k % 2 == 0:
            odo.imu(Imu(t, (0.0, 0.0, 1.0), (0.0, 0.0, omega + biais + rng.normal(0, 0.005))))
        if k % 10 == 0:
            angles, distances = monde.lancer_lidar(pose, profil, pose_debut=pose_tour, rng=rng)
            pose_tour = pose
            collisions += int(visible.distance_obstacles(pose, profil) <= 0)
            debut = time.perf_counter()
            o = odo.scan(Scan(t, angles, distances, 0.1))
            durees.append((time.perf_counter() - debut) * 1000)
            assert o is not None
            sources.append(o.source)
            vrai = depart.inverse().composer(pose)
            pire = max(pire, math.hypot(o.pose.x - vrai.x, o.pose.y - vrai.y))
    vrai = depart.inverse().composer(pose)
    o = odo.courant(t)
    return Bilan(parcouru, math.hypot(o.pose.x - vrai.x, o.pose.y - vrai.y),
                 math.degrees(abs(normaliser_angle(o.pose.cap - vrai.cap))), pire, collisions,
                 np.array(durees), sources, odo)


def test_salle_huit_20_m() -> None:
    """Dérive < 2 % de la distance et < 3° sur plus de 20 m dans la salle meublée."""
    profil = profil_test()
    bilan = rouler("salle", Pose2D(5.3, 3.0, -math.pi / 2), huit(), profil)
    assert bilan.parcouru > 20.0
    assert bilan.collisions == 0
    assert bilan.erreur_m < 0.02 * bilan.parcouru
    assert bilan.pire_m < 0.02 * bilan.parcouru
    assert bilan.erreur_cap_deg < 3.0
    odo = bilan.odometre
    assert odo is not None
    # Calibrations apprises en route : biais du gyroscope, vitesse réelle, PWM par m/s.
    assert math.degrees(odo.biais_gyro) == pytest.approx(0.80, abs=0.1)
    assert odo.gain_gyro == pytest.approx(1.0, abs=0.05)
    assert odo.rapport_vitesse == pytest.approx(0.85, abs=0.03)
    assert odo.pwm_par_m_s == pytest.approx(14 / 0.34, rel=0.05)
    print(f"\nsalle : {bilan.parcouru:.1f} m, erreur {bilan.erreur_m * 100:.1f} cm "
          f"({100 * bilan.erreur_m / bilan.parcouru:.2f} %), cap {bilan.erreur_cap_deg:.2f}°, "
          f"ICP {bilan.durees_ms.mean():.2f} ms (p95 {np.percentile(bilan.durees_ms, 95):.2f})")
    assert dans_le_budget(float(np.median(bilan.durees_ms)) / 1000, 0.010)


def test_jardin() -> None:
    profil = profil_test()
    chemin = np.array([(1.5, 1.5), (4.0, 3.0), (7.0, 4.0), (7.5, 9.0), (10.0, 9.5),
                       (12.0, 7.0), (15.0, 5.0)])
    bilan = rouler("jardin", Pose2D(1.5, 1.5, 0.5), chemin, profil)
    assert bilan.parcouru > 18.0
    assert bilan.erreur_m < 0.02 * bilan.parcouru
    assert bilan.erreur_cap_deg < 3.0


def test_couloir_degenere() -> None:
    """Au milieu d'un couloir de 30 m, les extrémités sont hors de portée : l'ICP ne contraint
    plus l'avance (signalé « degenere »), l'a priori de la commande la porte."""
    profil = profil_test()
    chemin = np.column_stack((np.linspace(1, 25, 200), np.ones(200)))
    bilan = rouler("couloir", Pose2D(1.0, 1.0, 0.0), chemin, profil)
    assert any("degenere" in s for s in bilan.sources)
    assert bilan.erreur_m < 0.03 * bilan.parcouru
    assert bilan.erreur_cap_deg < 2.0


def test_champ_ouvert_repli_gyro_commande() -> None:
    """Rien à portée du lidar : repli sur gyroscope + commande, biais appris à l'arrêt."""
    profil = profil_test()
    chemin = np.vstack([np.column_stack((np.linspace(0, 8, 50), np.zeros(50))),
                        np.column_stack((8 + 3 * np.sin(np.linspace(0, 3, 60)),
                                         3 - 3 * np.cos(np.linspace(0, 3, 60))))])
    bilan = rouler("champ", Pose2D(0.0, 0.0, 0.0), chemin, profil, rapport=1.0, attente_s=5.0)
    assert all(s in ("gyro+commande", "initialisation") for s in bilan.sources)
    odo = bilan.odometre
    assert odo is not None
    assert math.degrees(odo.biais_gyro) == pytest.approx(0.80, abs=0.15)
    assert bilan.erreur_m < 0.05 * bilan.parcouru
    assert bilan.erreur_cap_deg < 5.0


def test_arret_sans_derive_et_blocage() -> None:
    """Commande de 0,4 m/s mais roues bloquées : la pose ne bouge pas et le blocage est vu."""
    profil = profil_test()
    bilan = rouler("salle", Pose2D(5.3, 3.0, 0.0), None, profil, bloque=True, duree_max=6.0)
    odo = bilan.odometre
    assert odo is not None
    assert bilan.erreur_m < 0.02
    assert bilan.erreur_cap_deg < 0.3
    assert abs(odo.vitesse) < 0.03
    assert odo.bloque


def test_recalage_aberrant_rejete() -> None:
    """Un tour venu d'ailleurs (rover soulevé, lidar masqué) ne fait pas sauter la pose."""
    profil = profil_test()
    monde = Monde(0, "salle")
    odo = Odometre(profil)
    pose = Pose2D(5.3, 3.0, 0.0)
    t = 0.0
    for _ in range(10):
        t += 0.1
        odo.commande(CommandeRoues(t))
        odo.imu(Imu(t, (0, 0, 1), (0, 0, 0)))
        a, d = monde.lancer_lidar(pose, profil)
        o = odo.scan(Scan(t, a, d))
    t += 0.1
    odo.imu(Imu(t, (0, 0, 1), (0, 0, 0)))
    ailleurs = Pose2D(2.0, 4.5, 2.0)
    a, d = monde.lancer_lidar(ailleurs, profil)
    o = odo.scan(Scan(t, a, d))
    assert o is not None
    assert math.hypot(o.pose.x, o.pose.y) < 0.05
    assert abs(o.pose.cap) < math.radians(1)
    assert o.qualite == 0.0
    # Retour à la normale au tour suivant.
    t += 0.1
    odo.imu(Imu(t, (0, 0, 1), (0, 0, 0)))
    a, d = monde.lancer_lidar(pose, profil)
    o = odo.scan(Scan(t, a, d))
    assert o is not None and o.qualite > 0.5 and o.source.startswith("icp")


def test_courant_et_remise_a_zero() -> None:
    profil = profil_test()
    monde = Monde(0, "salle")
    odo = Odometre(profil)
    assert odo.courant(0.0).source == "aucune"
    pose = Pose2D(5.3, 3.0, 0.0)
    t = 0.0
    for _ in range(30):
        t += 0.1
        odo.commande(CommandeRoues(t, 14, 14, 0.0, 0.4, 0.0))
        for k in range(5):
            odo.imu(Imu(t - 0.08 + 0.02 * k, (0, 0, 1), (0, 0, 0)))
        pose = deplacer(pose, 0.4, 0.0, 0.1)
        a, d = monde.lancer_lidar(pose, profil)
        odo.scan(Scan(t, a, d))
    derniere = odo.courant(t)
    plus_tard = odo.courant(t + 0.05)
    assert plus_tard.pose.x - derniere.pose.x == pytest.approx(0.4 * 0.05, abs=0.006)
    assert odo.scan(Scan(t - 0.05, a, d)) is None  # tour daté du passé : ignoré
    odo.remettre_a_zero()
    assert odo.courant(t + 0.1).pose == Pose2D()
