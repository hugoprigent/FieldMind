"""Recalage lidar 2D : ICP point-à-ligne robuste contre une sous-carte locale.

Chaque point du tour courant est apparié au point le plus proche de la sous-carte (arbre k-d),
qui porte une normale estimée sur son propre tour (ACP des voisins le long du balayage). Le
résidu est la distance du point à la droite locale de la référence, n · (R p + t − m) : un mur
contraint le recalage perpendiculairement à lui-même, pas le long. On minimise par Gauss-Newton
avec un noyau de Cauchy (les points aberrants — une personne qui passe, une chaise déplacée —
pèsent peu), une distance d'appariement qui se resserre au fil des itérations, et un **a priori**
(gyroscope pour le lacet, commande pour l'avance) pondéré par son information : dans un couloir,
la direction le long des murs n'est pas observable par le lidar et c'est l'a priori qui la fixe ;
dans une pièce, le lidar domine.

Sortie : la pose, l'information (hessien) du seul lidar, sa direction la plus faible
(dégénérescence), la proportion de points appariés, le résidu et une qualité 0..1.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from fieldmind.types import Pose2D


@dataclass(slots=True)
class ParametresIcp:
    iterations_max: int = 20
    distance_initiale_m: float = 0.40
    distance_finale_m: float = 0.12
    resserrement: float = 0.7
    echelle_cauchy_m: float = 0.03
    seuil_inlier_m: float = 0.05
    cos_normales_min: float = 0.5
    """Normales source et référence à moins de 60° : on n'apparie pas les deux faces d'un mur."""
    convergence_m: float = 1e-4
    convergence_rad: float = 1e-4
    rapport_degenere: float = 0.02
    sigma_m: float = 0.03
    """Écart type effectif d'un résidu (bruit du lidar, points voisins corrélés, modèle) : il
    met l'information du lidar à l'échelle de celle de l'a priori."""
    """λmin / λmax du bloc translation du hessien sous lequel une direction n'est pas observée."""


@dataclass(slots=True)
class ResultatIcp:
    pose: np.ndarray  # (3,) x, y, cap : base_link (du tour) dans le repère de la sous-carte
    information: np.ndarray  # (3, 3) hessien du lidar seul à la solution
    covariance: np.ndarray  # (3, 3) covariance a posteriori (lidar + a priori)
    appariements: int
    proportion: float  # points appariés sous `seuil_inlier_m` / points du tour
    residu_rms: float
    iterations: int
    qualite: float
    degenere: bool
    direction_faible: np.ndarray  # (2,) direction de translation la moins contrainte (odom)
    rapport_valeurs_propres: float


def normales_balayage(points: np.ndarray, voisins: int = 3, rayon: float = 0.15
                      ) -> tuple[np.ndarray, np.ndarray]:
    """Normales (N, 2) et planéité (N,) 0..1 d'un tour trié par angle.

    ACP sur les `voisins` points de part et d'autre (dans l'ordre du balayage), retenus s'ils sont
    à moins de `rayon`. Normales orientées vers le capteur (origine). Planéité 0 : pas de droite
    locale (point isolé, coin aigu) ; ces points restent utilisables avec un poids faible.
    """
    n = len(points)
    if n < 3:
        return np.zeros((n, 2), dtype=np.float64), np.zeros(n)
    decalages = np.arange(-voisins, voisins + 1)
    indices = (np.arange(n)[:, None] + decalages[None, :]) % n
    voisins_xy = points[indices]  # (N, K, 2)
    ecarts = voisins_xy - points[:, None, :]
    garde = np.hypot(ecarts[..., 0], ecarts[..., 1]) <= rayon
    nombre = garde.sum(axis=1)
    poids = garde.astype(np.float64)
    moyenne = (voisins_xy * poids[..., None]).sum(axis=1) / np.maximum(nombre, 1)[:, None]
    centres = (voisins_xy - moyenne[:, None, :]) * poids[..., None]
    a = (centres[..., 0] ** 2).sum(axis=1)
    b = (centres[..., 0] * centres[..., 1]).sum(axis=1)
    c = (centres[..., 1] ** 2).sum(axis=1)
    phi = 0.5 * np.arctan2(2 * b, a - c)  # direction de la droite
    normales = np.column_stack((-np.sin(phi), np.cos(phi)))
    vers_capteur = np.einsum("ij,ij->i", normales, points) > 0
    normales[vers_capteur] *= -1
    demi_trace = 0.5 * (a + c)
    racine = np.sqrt(np.maximum((0.5 * (a - c)) ** 2 + b * b, 0.0))
    lmax, lmin = demi_trace + racine, demi_trace - racine
    with np.errstate(divide="ignore", invalid="ignore"):
        planeite = np.where(lmax > 1e-12, 1.0 - np.sqrt(np.maximum(lmin, 0) / lmax), 0.0)
    planeite = np.clip(planeite, 0.0, 1.0)
    planeite[nombre < 3] = 0.0
    return normales, planeite


def transformer(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]]) + pose[:2]


@dataclass(slots=True)
class ImageCle:
    pose: Pose2D
    points: np.ndarray  # (N, 2) odom
    normales: np.ndarray  # (N, 2) odom
    planeite: np.ndarray  # (N,)
    t: float


class SousCarte:
    """Les derniers tours retenus (images clés), dans le repère odom, avec un arbre k-d."""

    def __init__(self, images_max: int = 12) -> None:
        self.images: deque[ImageCle] = deque(maxlen=images_max)
        self.points = np.zeros((0, 2))
        self.normales = np.zeros((0, 2))
        self.planeite = np.zeros(0)
        self.arbre: cKDTree | None = None

    def __len__(self) -> int:
        return len(self.points)

    def vider(self) -> None:
        self.images.clear()
        self._reconstruire()

    def ajouter(self, pose: Pose2D, points_locaux: np.ndarray, t: float) -> None:
        """Ajoute un tour (points dans base_link, triés par angle) vu depuis `pose` (odom)."""
        normales, planeite = normales_balayage(points_locaux)
        p = np.array([pose.x, pose.y, pose.cap])
        c, s = math.cos(pose.cap), math.sin(pose.cap)
        rotation = np.array([[c, s], [-s, c]])
        self.images.append(ImageCle(pose, transformer(points_locaux, p), normales @ rotation,
                                    planeite, t))
        self._reconstruire()

    def _reconstruire(self) -> None:
        if not self.images:
            self.points = np.zeros((0, 2))
            self.normales = np.zeros((0, 2))
            self.planeite = np.zeros(0)
            self.arbre = None
            return
        self.points = np.concatenate([i.points for i in self.images])
        self.normales = np.concatenate([i.normales for i in self.images])
        self.planeite = np.concatenate([i.planeite for i in self.images])
        self.arbre = cKDTree(self.points, leafsize=16, balanced_tree=False)

    def distance_derniere(self, pose: Pose2D) -> tuple[float, float]:
        """Distance et écart de cap entre `pose` et la dernière image clé."""
        if not self.images:
            return math.inf, math.inf
        d = self.images[-1].pose
        ecart = math.atan2(math.sin(pose.cap - d.cap), math.cos(pose.cap - d.cap))
        return math.hypot(pose.x - d.x, pose.y - d.y), abs(ecart)

    def distance_plus_proche(self, x: float, y: float) -> float:
        if not self.images:
            return math.inf
        return min(math.hypot(x - i.pose.x, y - i.pose.y) for i in self.images)


def recaler(source: np.ndarray, normales_source: np.ndarray | None, carte: SousCarte,
            initiale: np.ndarray, a_priori: np.ndarray | None = None,
            information_a_priori: np.ndarray | None = None,
            parametres: ParametresIcp | None = None) -> ResultatIcp | None:
    """Pose (x, y, cap) du tour `source` (points base_link) dans le repère de la sous-carte.

    `a_priori` (3,) et `information_a_priori` (3, 3) : estimation prédite et son information
    (inverse de la covariance). Renvoie None si la sous-carte est vide.
    """
    p = parametres or ParametresIcp()
    if carte.arbre is None or len(source) < 3:
        return None
    x = np.array(initiale, dtype=np.float64)
    prior = None if a_priori is None else np.asarray(a_priori, dtype=np.float64)
    lam = None if information_a_priori is None else np.asarray(information_a_priori)
    distance_max = p.distance_initiale_m
    iterations = 0
    for iterations in range(1, p.iterations_max + 1):  # noqa: B007 (rendu dans le résultat)
        jac, residus, poids, _ = _lineariser(x, source, normales_source, carte, distance_max, p)
        h = np.zeros((3, 3))
        g = np.zeros(3)
        if jac is not None:
            jw = jac * poids[:, None]
            h = jw.T @ jac
            g = jw.T @ residus
        if prior is not None and lam is not None:
            ecart = x - prior
            ecart[2] = math.atan2(math.sin(ecart[2]), math.cos(ecart[2]))
            h = h + lam
            g = g + lam @ ecart
        try:
            pas = -np.linalg.solve(h + np.eye(3) * 1e-9, g)
        except np.linalg.LinAlgError:
            break
        x += pas
        distance_max = max(p.distance_finale_m, distance_max * p.resserrement)
        if (math.hypot(pas[0], pas[1]) < p.convergence_m and abs(pas[2]) < p.convergence_rad
                and distance_max <= p.distance_finale_m + 1e-9):
            break

    # Bilan à la solution.
    jac, residus, poids, apparies = _lineariser(x, source, normales_source, carte,
                                                p.distance_finale_m, p)
    information = np.zeros((3, 3))
    proportion = 0.0
    rms = math.inf
    if jac is not None:
        information = (jac * poids[:, None]).T @ jac
        bons = np.abs(residus) < p.seuil_inlier_m
        proportion = float(bons.sum()) / len(source)
        rms = float(np.sqrt(np.mean(residus[bons] ** 2))) if bons.any() else math.inf
    total = information + (lam if lam is not None else np.eye(3) * 1e-6)
    try:
        covariance = np.linalg.inv(total + np.eye(3) * 1e-12)
    except np.linalg.LinAlgError:
        covariance = np.eye(3)
    # Dégénérescence : bloc translation du hessien du lidar.
    valeurs, vecteurs = np.linalg.eigh(information[:2, :2])
    rapport = float(valeurs[0] / valeurs[1]) if valeurs[1] > 1e-9 else 0.0
    degenere = rapport < p.rapport_degenere
    qualite = _qualite(proportion, rms, apparies, len(source), p)
    return ResultatIcp(
        pose=x, information=information, covariance=covariance, appariements=apparies,
        proportion=proportion, residu_rms=rms, iterations=iterations, qualite=qualite,
        degenere=degenere, direction_faible=vecteurs[:, 0], rapport_valeurs_propres=rapport)


def _lineariser(x: np.ndarray, source: np.ndarray, normales_source: np.ndarray | None,
                carte: SousCarte, distance_max: float, p: ParametresIcp
                ) -> tuple[np.ndarray | None, np.ndarray, np.ndarray, int]:
    """Jacobien (M, 3), résidus (M,) et poids (M,) des appariements valides."""
    c, s = math.cos(x[2]), math.sin(x[2])
    qx = c * source[:, 0] - s * source[:, 1] + x[0]
    qy = s * source[:, 0] + c * source[:, 1] + x[1]
    assert carte.arbre is not None
    distances, indices = carte.arbre.query(np.column_stack((qx, qy)), k=1,
                                           distance_upper_bound=distance_max)
    ok = np.isfinite(distances)
    if normales_source is not None and ok.any():
        nsx = c * normales_source[:, 0] - s * normales_source[:, 1]
        nsy = s * normales_source[:, 0] + c * normales_source[:, 1]
        idx = np.where(ok, indices, 0)
        compatibles = (nsx * carte.normales[idx, 0] + nsy * carte.normales[idx, 1]
                       ) >= p.cos_normales_min
        ok &= compatibles
    m = int(ok.sum())
    if m < 3:
        return None, np.zeros(0), np.zeros(0), m
    idx = indices[ok]
    nx, ny = carte.normales[idx, 0], carte.normales[idx, 1]
    mx, my = carte.points[idx, 0], carte.points[idx, 1]
    residus = nx * (qx[ok] - mx) + ny * (qy[ok] - my)
    sx, sy = source[ok, 0], source[ok, 1]
    derivee = nx * (-s * sx - c * sy) + ny * (c * sx - s * sy)
    jac = np.column_stack((nx, ny, derivee))
    cauchy = 1.0 / (1.0 + (residus / p.echelle_cauchy_m) ** 2)
    poids = cauchy * (0.25 + 0.75 * carte.planeite[idx]) / p.sigma_m**2
    return jac, residus, poids, m


def _qualite(proportion: float, rms: float, apparies: int, total: int, p: ParametresIcp
             ) -> float:
    if total == 0 or not math.isfinite(rms):
        return 0.0
    q_prop = min(1.0, proportion / 0.6)
    q_rms = math.exp(-((rms / (1.5 * p.echelle_cauchy_m)) ** 2))
    q_nombre = min(1.0, apparies / 60.0)
    return float(q_prop * (0.4 + 0.6 * q_rms) * q_nombre)
