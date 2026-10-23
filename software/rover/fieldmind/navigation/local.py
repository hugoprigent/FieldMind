"""Pilote local : suivi du chemin par poursuite pure et évitement par arcs Ackermann.

À chaque pas (10 Hz) :

1. **Avancement** sur le chemin (point le plus proche, recherche vers l'avant) et distance
   restante. Arrivée quand l'essieu arrière est à moins de `navigation.tolerance_arrivee_m` du
   bout du chemin (ou l'a dépassé de côté).
2. **Poursuite pure** sur l'essieu arrière (le point dont la trajectoire suit le modèle
   bicyclette) : point visé à `Ld` devant sur le chemin (Ld croît avec la vitesse), courbure
   κ = 2 sin α / Ld, bornée à tan(δmax) / L.
3. **Éventail d'arcs** de courbure constante (Ackermann, |κ| ≤ κmax) : pour chacun, l'emprise
   du rover (rectangle des roues et de la coque, plus une marge) avance le long de l'arc par pas
   de 5 cm jusqu'à `navigation.horizon_m` ; on retient la distance libre avant le premier point
   d'obstacle (lidar, stéréo, mémoire de la carte, en base_link). Un arc est admissible si sa
   distance libre couvre l'arrêt à la vitesse visée plus une marge. On choisit l'arc admissible
   le plus proche de la poursuite pure, en préférant le dégagement et la continuité.
4. **Vitesse** : la plus faible de la vitesse maximale, de l'accélération latérale admissible,
   de l'arrêt avant l'obstacle sur l'arc choisi, de l'arrêt en douceur au but, et d'une vitesse
   réduite quand la cible est loin de l'axe ou que l'arc frôle un obstacle.
5. Aucun arc avant admissible : **marche arrière courte** (`navigation.distance_recul_m`) en
   braquant pour ramener le nez vers la cible, si l'arrière est libre (lidar). Sinon `bloque`.
6. But **derrière et proche** (moins de `navigation.recul_direct_max_m` à vol d'oiseau, au-delà de
   120° de l'axe) : on y va **en marche arrière** (poursuite pure à reculons, κ = 2y/d²) si
   l'arrière est libre, plutôt que de chercher la place d'un demi-tour.

Le superviseur de sécurité limite encore la vitesse et arrête : le pilote évite, la sécurité
arrête.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from fieldmind.config import Profil, braquage_max
from fieldmind.types import Chemin, Obstacles, Odometrie, Pose2D


@dataclass(slots=True)
class CommandeLocale:
    vitesse: float  # m/s, négative en marche arrière
    braquage: float  # rad, + à gauche
    arrive: bool = False
    bloque: bool = False
    distance_restante: float = 0.0
    raison: str = ""  # "" | "recul_degagement" | "aucun_arc_libre" | "evitement" | "arrive"…
    libre_m: float = math.inf  # distance libre le long de l'arc choisi
    courbure: float = 0.0
    indice: int = 0  # point du chemin le plus proche


ObstaclesEntree = Obstacles | np.ndarray | Sequence[Obstacles | np.ndarray] | None


def points_obstacles(obstacles: ObstaclesEntree) -> np.ndarray:
    """(N, 2) float64 base_link depuis un tableau, des `Obstacles`, ou une liste des deux."""
    if obstacles is None:
        return np.zeros((0, 2))
    if isinstance(obstacles, Obstacles):
        return np.asarray(obstacles.points, dtype=np.float64).reshape(-1, 2)
    if isinstance(obstacles, np.ndarray):
        return np.asarray(obstacles, dtype=np.float64).reshape(-1, 2)
    morceaux = [points_obstacles(o) for o in obstacles]
    return np.vstack(morceaux) if morceaux else np.zeros((0, 2))


def filtrer_isoles(points: np.ndarray, rayon: float = 0.10, voisins: int = 2) -> np.ndarray:
    """Points qui ont au moins `voisins` autres points à moins de `rayon` : une détection
    isolée de la stéréo (joint, reflet) est écartée, un vrai objet en fait plusieurs."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    points = np.unique(points[np.isfinite(points).all(axis=1)], axis=0)
    if len(points) <= voisins:
        return np.zeros((0, 2))
    nombres = cKDTree(points).query_ball_point(points, rayon, return_length=True)
    return points[np.asarray(nombres) > voisins]


class PiloteLocal:
    def __init__(self, profil: Profil) -> None:
        v = profil.valeur
        self.profil = profil
        self.empattement = float(v("geometrie.empattement_m", 0.38))
        # Roue virtuelle centrale (≈ 25,8°), pas la butée de la roue intérieure (31°).
        self.braquage_max = braquage_max(profil)
        self.courbure_max = math.tan(self.braquage_max) / self.empattement
        self.arriere = float(v("geometrie.emprise_arriere_m", -0.11))
        self.avant = float(v("geometrie.emprise_avant_m", 0.46))
        self.demi_largeur = float(v("geometrie.emprise_demi_largeur_m", 0.225))
        self.marge_laterale = float(v("navigation.marge_laterale_m", 0.06))
        self.marge_avant = float(v("navigation.marge_avant_m", 0.10))
        self.nombre_arcs = int(v("navigation.arcs", 21))
        self.horizon = float(v("navigation.horizon_m", 2.0))
        self.pas = float(v("navigation.pas_arc_m", 0.05))
        self.tolerance_arrivee = float(v("navigation.tolerance_arrivee_m", 0.20))
        self.visee_min = float(v("navigation.visee_min_m", 0.6))
        self.visee_max = float(v("navigation.visee_max_m", 1.2))
        self.visee_par_vitesse = float(v("navigation.visee_par_vitesse_s", 0.8))
        self.deceleration = float(v("navigation.deceleration_m_s2", 0.5))
        self.latence = float(v("securite.latence_s", 0.25))
        self.marge_arret = float(v("navigation.marge_arret_m", 0.15))
        self.acceleration_laterale = float(v("navigation.acceleration_laterale_m_s2", 0.6))
        self.vitesse_min = float(v("navigation.vitesse_min_m_s", 0.12))
        self.vitesse_recul = min(float(v("vitesses.max_arriere_m_s", 0.3)),
                                 float(v("navigation.vitesse_recul_m_s", 0.15)))
        self.distance_recul = float(v("navigation.distance_recul_m", 0.4))
        self.recul_direct_max = float(v("navigation.recul_direct_max_m", 1.5))
        self.poids_indicatifs = float(v("navigation.poids_indicatifs", 2.0))
        self.reinitialiser()

    def reinitialiser(self) -> None:
        self._chemin_cle: tuple[float, int, float, float] | None = None
        self._indice = 0
        self._courbure_prec = 0.0
        self._recul: tuple[Pose2D, float] | None = None  # pose de départ, courbure

    # ------------------------------------------------------------------ commande
    def commande(self, odom: Odometrie | Pose2D, chemin: Chemin | None,
                 obstacles: ObstaclesEntree, vitesse_max: float,
                 indicatifs: np.ndarray | None = None) -> CommandeLocale:
        """`obstacles` bloquent (lidar) ; `indicatifs` (stéréo, mémoire des obstacles bas)
        rendent seulement un arc plus coûteux : on passe là où il y en a le moins."""
        pose = odom.pose if isinstance(odom, Odometrie) else odom
        vitesse_mesuree = odom.vitesse if isinstance(odom, Odometrie) else 0.0
        if chemin is None or len(chemin.points) == 0:
            return CommandeLocale(0.0, 0.0, raison="pas_de_chemin")
        points = np.asarray(chemin.points, dtype=np.float64).reshape(-1, 2)
        cle = (float(chemin.t), len(points), float(points[-1, 0]), float(points[-1, 1]))
        if cle != self._chemin_cle:
            self._chemin_cle = cle
            self._indice = int(np.argmin(np.hypot(points[:, 0] - pose.x,
                                                  points[:, 1] - pose.y)))
        abscisse = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(points, axis=0).T))])
        indice, s = self._avancer(points, abscisse, pose)
        fin = points[-1]
        distance_but = math.hypot(fin[0] - pose.x, fin[1] - pose.y)
        restante = max(float(abscisse[-1] - s), 0.0)
        restante = max(restante, distance_but) if restante < 0.5 else restante
        lx_fin, _ = pose.vers_local(float(fin[0]), float(fin[1]))
        if distance_but <= self.tolerance_arrivee or (
                indice >= len(points) - 2 and lx_fin < 0 and distance_but < 2.5
                * self.tolerance_arrivee):
            self._recul = None
            return CommandeLocale(0.0, 0.0, arrive=True, distance_restante=distance_but,
                                  raison="arrive", indice=indice)

        obstacles_xy = points_obstacles(obstacles)
        obstacles_xy = self._filtrer(obstacles_xy)

        # Poursuite pure.
        visee = min(max(self.visee_min + self.visee_par_vitesse * abs(vitesse_mesuree),
                        self.visee_min), self.visee_max)
        cible = self._point_a(points, abscisse, s + visee)
        tx, ty = pose.vers_local(cible[0], cible[1])
        alpha = math.atan2(ty, tx)
        distance_cible = max(math.hypot(tx, ty), 0.2)
        if abs(alpha) > math.radians(100):
            kappa_pp = math.copysign(self.courbure_max, alpha)
        else:
            kappa_pp = float(np.clip(2 * math.sin(alpha) / distance_cible, -self.courbure_max,
                                     self.courbure_max))

        # But juste derrière : marche arrière directe.
        bx_l, by_l = pose.vers_local(float(fin[0]), float(fin[1]))
        if (self._recul is None and distance_but < self.recul_direct_max and bx_l < 0
                and abs(math.atan2(by_l, bx_l)) > math.radians(120)):
            d2 = max(bx_l * bx_l + by_l * by_l, 0.04)
            kappa_r = float(np.clip(2 * by_l / d2, -self.courbure_max, self.courbure_max))
            libre_r = float(self._distance_libre(np.array([kappa_r]), obstacles_xy, arriere=True,
                                                 longueur=distance_but + 0.2)[0])
            if libre_r >= min(distance_but, 0.3):
                au_but = max(distance_but - self.tolerance_arrivee / 2, 0.0)
                v_r = min(self.vitesse_recul, max(math.sqrt(2 * self.deceleration * au_but),
                                                  self.vitesse_min))
                return self._sortie(-v_r, kappa_r, restante, "marche_arriere", libre_r, indice)

        # Manœuvre de recul en cours ?
        if self._recul is not None:
            depart, kappa_r = self._recul
            parcouru = math.hypot(pose.x - depart.x, pose.y - depart.y)
            libre_r = self._distance_libre(np.array([kappa_r]), obstacles_xy, arriere=True)[0]
            if parcouru < self.distance_recul and libre_r > 0.1:
                return self._sortie(-self.vitesse_recul, kappa_r, restante, "recul_degagement",
                                    libre_r, indice)
            self._recul = None

        # Éventail d'arcs.
        kappas = np.unique(np.concatenate([
            np.linspace(-self.courbure_max, self.courbure_max, self.nombre_arcs), [kappa_pp]]))
        libres = self._distance_libre(kappas, obstacles_xy)
        degages = self._distance_libre(kappas, obstacles_xy, marge_sup=0.15)
        v_cible = max(min(vitesse_max, max(abs(vitesse_mesuree), self.vitesse_min) + 0.2),
                      self.vitesse_min)
        besoin = (v_cible**2 / (2 * self.deceleration) + v_cible * self.latence
                  + self.marge_arret)
        besoin = min(besoin, restante + self.marge_arret)
        admissibles = libres >= max(min(besoin, self.horizon), self.pas * 2)
        if not admissibles.any():
            # Plus lent : un arc qui permet au moins de s'arrêter à vitesse minimale.
            besoin_min = (self.vitesse_min**2 / (2 * self.deceleration)
                          + self.vitesse_min * self.latence + self.marge_arret)
            admissibles = libres >= min(besoin_min, restante + 0.05)
        if not admissibles.any():
            return self._degager(pose, alpha, obstacles_xy, restante, indice)

        ecart = np.abs(kappas - kappa_pp) / (2 * self.courbure_max)
        cout = (4.0 * ecart + 1.0 * (1 - np.minimum(libres, self.horizon) / self.horizon)
                + 0.5 * (1 - np.minimum(degages, self.horizon) / self.horizon)
                + 0.3 * np.abs(kappas - self._courbure_prec) / (2 * self.courbure_max))
        if indicatifs is not None and len(indicatifs):
            doux = filtrer_isoles(self._filtrer(np.asarray(indicatifs).reshape(-1, 2)))
            if len(doux):
                libres_doux = self._distance_libre(kappas, doux)
                cout += self.poids_indicatifs * (1 - np.minimum(libres_doux, self.horizon)
                                                 / self.horizon)
        cout[~admissibles] = np.inf
        k = int(np.argmin(cout))
        kappa = float(kappas[k])
        libre = float(libres[k])
        self._courbure_prec = kappa

        # Vitesse.
        v = vitesse_max
        if abs(kappa) > 1e-3:
            v = min(v, math.sqrt(self.acceleration_laterale / abs(kappa)))
        freinage = max(libre - self.marge_arret, 0.0)
        v = min(v, max(math.sqrt(2 * self.deceleration * freinage)
                       - self.deceleration * self.latence, 0.0))
        # Arrêt en douceur au milieu de la tolérance d'arrivée (latence comprise).
        au_but = max(restante - self.tolerance_arrivee / 2, 0.0)
        v = min(v, max(math.sqrt(2 * self.deceleration * au_but)
                       - self.deceleration * self.latence, self.vitesse_min))
        if abs(alpha) > math.radians(60):
            v = min(v, 0.2)
        if degages[k] < 0.6:
            v = min(v, 0.25)
        v = max(v, self.vitesse_min) if freinage > 0.02 else 0.0
        raison = "evitement" if abs(kappa - kappa_pp) > 0.15 * self.courbure_max else ""
        return self._sortie(v, kappa, restante, raison, libre, indice)

    def _sortie(self, vitesse: float, kappa: float, restante: float, raison: str, libre: float,
                indice: int, bloque: bool = False) -> CommandeLocale:
        return CommandeLocale(vitesse=float(vitesse),
                              braquage=float(math.atan(kappa * self.empattement)),
                              bloque=bloque, distance_restante=float(restante), raison=raison,
                              libre_m=float(libre), courbure=float(kappa), indice=indice)

    def _degager(self, pose: Pose2D, alpha: float, obstacles_xy: np.ndarray, restante: float,
                 indice: int) -> CommandeLocale:
        """Aucun arc avant libre : recul en braquant à l'opposé de la cible (le nez tourne vers
        elle), ou blocage."""
        v = self.profil.valeur
        if bool(v("remorque.attelee", False)) and not bool(
                v("remorque.marche_arriere_auto", False)):
            # Remorque : pas de recul automatique (mise en portefeuille, rien ne voit derrière bas)
            return self._sortie(0.0, 0.0, restante, "aucun_arc_libre", 0.0, indice, bloque=True)
        signe = -1.0 if alpha >= 0 else 1.0
        candidats = np.array([signe * self.courbure_max, signe * 0.5 * self.courbure_max, 0.0,
                              -signe * 0.5 * self.courbure_max])
        libres = self._distance_libre(candidats, obstacles_xy, arriere=True)
        for kappa, libre in zip(candidats, libres, strict=True):
            if libre >= min(self.distance_recul, 0.25):
                self._recul = (Pose2D(pose.x, pose.y, pose.cap), float(kappa))
                return self._sortie(-self.vitesse_recul, float(kappa), restante,
                                    "recul_degagement", float(libre), indice)
        return self._sortie(0.0, 0.0, restante, "aucun_arc_libre", 0.0, indice, bloque=True)

    # ------------------------------------------------------------------ chemin
    def _avancer(self, points: np.ndarray, abscisse: np.ndarray, pose: Pose2D
                 ) -> tuple[int, float]:
        debut = max(self._indice - 3, 0)
        fin = min(len(points), self._indice + 80)
        distances = np.hypot(points[debut:fin, 0] - pose.x, points[debut:fin, 1] - pose.y)
        i = debut + int(np.argmin(distances))
        self._indice = i
        s = float(abscisse[i])
        if i + 1 < len(points):
            a, b = points[i], points[i + 1]
            ab = b - a
            longueur2 = float(ab @ ab)
            if longueur2 > 1e-12:
                t = float(np.clip((np.array([pose.x, pose.y]) - a) @ ab / longueur2, 0.0, 1.0))
                s += t * math.sqrt(longueur2)
        return i, s

    @staticmethod
    def _point_a(points: np.ndarray, abscisse: np.ndarray, s: float) -> tuple[float, float]:
        s = min(max(s, 0.0), float(abscisse[-1]))
        if len(points) == 1:
            return float(points[0, 0]), float(points[0, 1])
        return (float(np.interp(s, abscisse, points[:, 0])),
                float(np.interp(s, abscisse, points[:, 1])))

    # ------------------------------------------------------------------ arcs
    def _filtrer(self, obstacles_xy: np.ndarray) -> np.ndarray:
        """Points utiles (à portée de l'horizon), sans ceux qui tombent dans l'emprise même du
        rover (échos parasites : la sécurité les traite), regroupés par cellules de 4 cm."""
        if len(obstacles_xy) == 0:
            return obstacles_xy
        x, y = obstacles_xy[:, 0], obstacles_xy[:, 1]
        portee = self.horizon + self.avant + 0.5
        garde = (x * x + y * y <= portee * portee) & np.isfinite(x) & np.isfinite(y)
        dedans = (x > self.arriere) & (x < self.avant) & (np.abs(y) < self.demi_largeur)
        utiles = obstacles_xy[garde & ~dedans]
        if len(utiles) > 400:
            cles = np.unique(np.floor(utiles / 0.04).astype(np.int64), axis=0)
            utiles = (cles + 0.5) * 0.04
        return utiles

    def _distance_libre(self, kappas: np.ndarray, obstacles_xy: np.ndarray,
                        arriere: bool = False, marge_sup: float = 0.0,
                        longueur: float | None = None) -> np.ndarray:
        """Distance (m, ≥ 0) parcourue le long de chaque arc avant que l'emprise (agrandie des
        marges) ne touche un point ; `horizon` (ou la distance de recul) si rien."""
        if longueur is None:
            longueur = self.distance_recul + 0.2 if arriere else self.horizon
        n = int(math.ceil(longueur / self.pas))
        s = np.arange(1, n + 1) * self.pas * (-1.0 if arriere else 1.0)  # (S,)
        if len(obstacles_xy) == 0:
            return np.full(len(kappas), longueur)
        theta = kappas[:, None] * s[None, :]  # (K, S)
        with np.errstate(divide="ignore", invalid="ignore"):
            droit = np.abs(kappas) < 1e-6
            rayon = np.where(droit, 0.0, 1.0 / np.where(droit, 1.0, kappas))
            px = np.where(droit[:, None], s[None, :], rayon[:, None] * np.sin(theta))
            py = np.where(droit[:, None], 0.0, rayon[:, None] * (1 - np.cos(theta)))
        c, sn = np.cos(theta), np.sin(theta)
        ox = obstacles_xy[:, 0][None, None, :] - px[..., None]  # (K, S, M)
        oy = obstacles_xy[:, 1][None, None, :] - py[..., None]
        lx = c[..., None] * ox + sn[..., None] * oy
        ly = -sn[..., None] * ox + c[..., None] * oy
        marge_l = self.marge_laterale + marge_sup
        marge_a = self.marge_avant + marge_sup
        x0, y0 = obstacles_xy[:, 0], obstacles_xy[:, 1]
        # Points déjà dans les marges au départ (rover garé près d'un mur) : contrôlés contre
        # l'emprise seule, sinon aucun arc ne serait jamais libre.
        deja = (x0 >= self.arriere - marge_a) & (x0 <= self.avant + marge_a) & (
            np.abs(y0) <= self.demi_largeur + marge_l)
        marge_l_i = np.where(deja, 0.0, marge_l)[None, None, :]
        marge_a_i = np.where(deja, 0.0, marge_a)[None, None, :]
        marge_s_i = np.where(deja, 0.0, marge_sup)[None, None, :]
        if arriere:
            x_min, x_max = self.arriere - marge_a_i, self.avant + marge_s_i
        else:
            x_min, x_max = self.arriere - marge_s_i, self.avant + marge_a_i
        touche = ((lx >= x_min) & (lx <= x_max) & (np.abs(ly) <= self.demi_largeur + marge_l_i)
                  ).any(axis=2)  # (K, S)
        premier = np.where(touche.any(axis=1), touche.argmax(axis=1), n)
        resultat: np.ndarray = premier * self.pas
        return resultat
