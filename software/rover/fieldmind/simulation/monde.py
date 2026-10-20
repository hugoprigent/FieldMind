"""Monde simulé : sol plat, murs, meubles, haies, arbres et obstacles bas, vus par les capteurs.

Le monde est fait de primitives 3D simples : boîtes orientées autour de la verticale, cylindres
verticaux, et l'intérieur d'une pièce (murs et plafond). Il sert de vérité aux tests et au
matériel simulé (`materiel_simule.py`) :

- `lancer_lidar(pose, profil)` : un tour du RPLIDAR C1, plan laser à `lidar.z_m` (0,322 m) ;
  seuls les objets qui coupent ce plan sont vus. Bruit de 1 cm, quelques pertes. Avec
  `pose_debut`, chaque faisceau part de la pose du rover à son instant (le lidar tourne pendant
  que le rover bouge), comme sur le vrai capteur : le tour commence à l'angle 0 du lidar.
- `rendre_camera(pose, camera, largeur, hauteur, profil)` : image BGR par lancer de rayons
  vectorisé (un rayon par pixel), perspective exacte du modèle de `perception/projection.py`
  (modèle par défaut, sans distorsion). Toutes les surfaces portent une texture de bruit
  multi-échelle attachée au monde (identique vue des deux caméras, pour que SGBM trouve ses
  correspondances), filtrée selon la taille du pixel au sol (niveaux de détail) pour éviter le
  repliement de spectre au loin.
- `collision(pose, profil)` : l'emprise du rover touche-t-elle un obstacle ?

Scénarios : `salle` (pièce de 10 × 7 m meublée), `jardin` (20 × 15 m, haies, arbres, rochers),
`couloir` (30 × 2 m sans relief : l'ICP y est dégénéré le long du couloir), `champ` (rien à
portée du lidar). Repère du monde : x vers l'est, y vers le nord, sol à z = 0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from fieldmind.config import Profil
from fieldmind.perception.projection import ModeleCamera
from fieldmind.types import Pose2D, normaliser_angle

TEXEL_M = 0.004
"""Taille d'un texel du niveau le plus fin (m) : une tuile de 512 texels couvre 2 m."""
TAILLE_TUILE = 512
NIVEAUX_TEXTURE = 6
LUMIERE = np.array([0.35, 0.45, 0.82]) / np.linalg.norm([0.35, 0.45, 0.82])
SOL, PIECE, CIEL = 0, 1, -1
"""Identifiants de surface du rendu ; les objets suivent (2 + indice), la mire est la dernière."""


# ---------------------------------------------------------------- primitives


@dataclass(slots=True)
class Boite:
    """Pavé posé à `z0` au-dessus du sol, orienté de `cap` autour de la verticale."""

    x: float
    y: float
    longueur: float  # selon l'axe local x
    largeur: float  # selon l'axe local y
    hauteur: float
    cap: float = 0.0
    z0: float = 0.0
    couleur: tuple[int, int, int] = (140, 140, 140)  # BGR
    nom: str = "boite"
    genre: str = "meuble"  # meuble | bas | haie | rocher | mur…

    @property
    def haut(self) -> float:
        return self.z0 + self.hauteur

    def coins(self) -> np.ndarray:
        """(4, 2) coins au sol, sens trigonométrique."""
        c, s = math.cos(self.cap), math.sin(self.cap)
        dx, dy = self.longueur / 2, self.largeur / 2
        locaux = np.array([[dx, dy], [-dx, dy], [-dx, -dy], [dx, -dy]])
        return locaux @ np.array([[c, s], [-s, c]]) + (self.x, self.y)


@dataclass(slots=True)
class Cylindre:
    x: float
    y: float
    rayon: float
    hauteur: float
    z0: float = 0.0
    couleur: tuple[int, int, int] = (60, 90, 120)
    nom: str = "cylindre"
    genre: str = "meuble"

    @property
    def haut(self) -> float:
        return self.z0 + self.hauteur


@dataclass(slots=True)
class Piece:
    """Intérieur d'une pièce rectangulaire alignée sur les axes : murs et plafond."""

    xmin: float
    ymin: float
    xmax: float
    ymax: float
    hauteur: float = 2.5
    couleur: tuple[int, int, int] = (190, 200, 205)


@dataclass(slots=True)
class Mire:
    """Damier plan tenu devant les caméras (calibration). Invisible du lidar, sans collision.

    `rotation` : colonnes = axe des colonnes du damier, axe des lignes, normale ; dans le monde.
    `colonnes` × `lignes` coins intérieurs, cases de `case_m`, bord blanc d'une case.
    """

    centre: np.ndarray
    rotation: np.ndarray
    colonnes: int = 9
    lignes: int = 6
    case_m: float = 0.025
    bord_cases: float = 1.0


Obstacle = Boite | Cylindre


@dataclass(slots=True)
class _Materiau:
    """Variante de la texture commune : échelle, décalage et rotation propres, pour que deux
    surfaces ne portent jamais le même motif à une translation près (SGBM les confondrait)."""

    echelle: float
    decalage: tuple[float, float]
    angle: float = 0.0


# ---------------------------------------------------------------- textures


_TEXTURES: dict[int, np.ndarray] = {}


def _texture(graine: int) -> np.ndarray:
    """Tuile périodique de bruit multi-échelle (niveaux, N, N) en uint8, et ses versions
    filtrées (niveau k : passe-bas à l'échelle de 2^k texels)."""
    if graine in _TEXTURES:
        return _TEXTURES[graine]
    rng = np.random.default_rng(10_000 + graine)
    n = TAILLE_TUILE
    frequences = np.fft.fftfreq(n)
    fx, fy = np.meshgrid(frequences, frequences)
    rayon2 = fx**2 + fy**2
    spectre_blanc = np.fft.fft2(rng.standard_normal((n, n)))
    # Somme de bandes : grains de 1, 3, 9, 27 et 81 texels (4 mm à 30 cm).
    filtre = np.zeros((n, n))
    for sigma_texels, poids in ((0.7, 0.9), (2.0, 1.0), (6.0, 1.0), (18.0, 1.0), (54.0, 0.8)):
        bande = np.exp(-2 * (math.pi * sigma_texels) ** 2 * rayon2)
        filtre += poids * bande * sigma_texels  # même énergie par octave
    niveaux = []
    reference = None
    for k in range(NIVEAUX_TEXTURE):
        passe_bas = np.exp(-2 * (math.pi * 0.6 * (2.0**k if k else 0.0)) ** 2 * rayon2)
        champ = np.real(np.fft.ifft2(spectre_blanc * filtre * passe_bas))
        if reference is None:
            reference = (float(champ.mean()), float(champ.std()))
        niveaux.append(np.clip(128 + 46 * (champ - reference[0]) / reference[1], 0, 255))
    tuile = np.stack(niveaux).astype(np.uint8)
    _TEXTURES[graine] = tuile
    return tuile


# ---------------------------------------------------------------- monde


class Monde:
    def __init__(self, graine: int = 0, scenario: str = "salle") -> None:
        self.graine = graine
        self.scenario = scenario
        self._rng = np.random.default_rng(graine)
        self.obstacles: list[Obstacle] = []
        self.piece: Piece | None = None
        self.mire: Mire | None = None
        self.couleur_sol: tuple[int, int, int] = (120, 140, 160)
        self.couleur_ciel: tuple[int, int, int] = (235, 205, 170)
        self.depart = Pose2D()
        self.but: tuple[float, float] = (0.0, 0.0)
        self.limites: tuple[float, float, float, float] = (-10.0, -10.0, 10.0, 10.0)
        constructeurs = {"salle": self._salle, "jardin": self._jardin,
                         "couloir": self._couloir, "champ": self._champ}
        if scenario not in constructeurs:
            raise ValueError(f"scénario inconnu : {scenario!r} ({', '.join(constructeurs)})")
        constructeurs[scenario]()
        if graine:
            self._varier()
        self._tuile = _texture(graine)
        self._tuile_plate = self._tuile.reshape(-1)
        self._grain = np.random.default_rng(graine + 1).normal(0, 1, 1 << 20).astype(np.float32)
        self._materiaux = [self._materiau() for _ in range(len(self.obstacles) + 3)]
        self._cache_plans: dict[float, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        self._cache_directions: dict[tuple[Any, ...], np.ndarray] = {}

    # ------------------------------------------------------------------ scénarios
    def _salle(self) -> None:
        self.piece = Piece(0.0, 0.0, 10.0, 7.0, 2.5, (185, 195, 200))
        self.limites = (0.0, 0.0, 10.0, 7.0)
        self.couleur_sol = (95, 125, 160)  # parquet
        self.depart = Pose2D(0.8, 0.8, 0.6)
        self.but = (9.0, 6.3)
        b, c = Boite, Cylindre
        # Meubles le long des murs, centre de la pièce (x 3–7,6 ; y 1,5–4,5) dégagé de tout ce
        # que voit le lidar ; obstacles bas sur la diagonale du départ au but.
        self.obstacles = [
            b(2.2, 6.45, 2.0, 0.85, 0.80, 0.0, couleur=(70, 60, 140), nom="canape"),
            b(2.2, 5.25, 1.0, 0.5, 0.08, 0.0, z0=0.30, couleur=(60, 100, 130), nom="table_basse"),
            c(1.8, 5.1, 0.03, 0.30, couleur=(50, 70, 90), nom="pied_table_basse"),
            c(2.6, 5.4, 0.03, 0.30, couleur=(50, 70, 90), nom="pied_table_basse"),
            b(6.8, 5.75, 1.6, 0.9, 0.04, 0.0, z0=0.72, couleur=(80, 120, 150), nom="table"),
            *[c(6.8 + sx * 0.72, 5.75 + sy * 0.37, 0.03, 0.72, couleur=(60, 90, 110),
                nom="pied_table") for sx in (-1, 1) for sy in (-1, 1)],
            b(5.75, 5.7, 0.45, 0.45, 0.88, 0.1, couleur=(90, 140, 110), nom="chaise"),
            b(7.9, 5.8, 0.45, 0.45, 0.88, -0.2, couleur=(90, 140, 110), nom="chaise"),
            b(9.65, 1.2, 0.6, 1.2, 1.9, 0.0, couleur=(120, 150, 170), nom="armoire"),
            b(0.25, 3.8, 0.35, 1.6, 1.2, 0.0, couleur=(60, 80, 110), nom="bibliotheque"),
            c(3.6, 4.9, 0.15, 2.5, couleur=(170, 170, 175), nom="poteau"),
            b(4.2, 0.6, 0.5, 0.4, 0.45, 0.3, couleur=(80, 150, 190), nom="carton"),
            b(9.5, 4.2, 0.6, 0.45, 0.5, -0.2, couleur=(80, 150, 190), nom="carton"),
            # Obstacles bas : sous le plan du lidar (0,322 m), invisibles pour lui.
            b(3.4, 2.2, 1.4, 0.10, 0.10, 1.1, couleur=(40, 90, 150), nom="poutre", genre="bas"),
            b(8.4, 2.4, 0.9, 0.7, 0.16, 0.0, couleur=(110, 110, 120), nom="marche", genre="bas"),
            c(6.4, 1.0, 0.16, 0.28, couleur=(50, 80, 170), nom="pot", genre="bas"),
            b(4.6, 3.3, 0.45, 0.35, 0.22, 0.4, couleur=(70, 140, 180), nom="carton_bas",
              genre="bas"),
            b(1.6, 2.6, 0.3, 0.2, 0.12, 0.5, couleur=(150, 60, 60), nom="jouet", genre="bas"),
        ]

    def _jardin(self) -> None:
        self.piece = None
        self.limites = (0.0, 0.0, 20.0, 15.0)
        self.couleur_sol = (70, 130, 85)  # pelouse
        self.depart = Pose2D(1.5, 1.5, 0.5)
        self.but = (18.3, 13.3)
        haie = (40, 95, 45)
        b, c = Boite, Cylindre
        self.obstacles = [
            b(10.0, -0.3, 20.6, 0.6, 1.4, couleur=haie, nom="haie", genre="haie"),
            b(10.0, 15.3, 20.6, 0.6, 1.4, couleur=haie, nom="haie", genre="haie"),
            b(-0.3, 7.5, 0.6, 15.0, 1.4, couleur=haie, nom="haie", genre="haie"),
            b(20.3, 7.5, 0.6, 15.0, 1.4, couleur=haie, nom="haie", genre="haie"),
            b(9.0, 4.0, 0.6, 8.0, 1.1, couleur=haie, nom="haie", genre="haie"),
            b(14.5, 10.0, 6.0, 0.6, 1.1, couleur=haie, nom="haie", genre="haie"),
            c(4.0, 6.0, 0.25, 3.5, couleur=(40, 60, 90), nom="arbre"),
            c(6.5, 11.0, 0.35, 4.0, couleur=(40, 60, 90), nom="arbre"),
            c(13.0, 5.0, 0.2, 3.0, couleur=(40, 60, 90), nom="arbre"),
            c(16.5, 3.0, 0.3, 3.5, couleur=(40, 60, 90), nom="arbre"),
            c(11.5, 13.0, 0.22, 3.0, couleur=(40, 60, 90), nom="arbre"),
            b(3.0, 12.5, 1.5, 0.45, 0.45, 0.2, couleur=(70, 110, 150), nom="banc"),
            b(17.0, 7.0, 2.0, 1.2, 0.75, 0.0, couleur=(140, 140, 140), nom="abri"),
            # Obstacles bas.
            b(5.5, 3.0, 0.45, 0.35, 0.20, 0.7, couleur=(120, 125, 130), nom="rocher",
              genre="bas"),
            c(11.0, 8.0, 0.28, 0.22, couleur=(115, 120, 125), nom="rocher", genre="bas"),
            b(7.0, 8.5, 2.5, 0.08, 0.12, 0.3, couleur=(150, 150, 160), nom="bordure",
              genre="bas"),
            c(14.0, 12.0, 0.18, 0.30, couleur=(50, 80, 170), nom="pot", genre="bas"),
            c(12.5, 2.5, 0.18, 0.30, couleur=(50, 80, 170), nom="pot", genre="bas"),
            b(15.5, 13.0, 1.0, 0.12, 0.14, -0.4, couleur=(40, 90, 150), nom="poutre",
              genre="bas"),
        ]

    def _couloir(self) -> None:
        self.piece = Piece(0.0, 0.0, 30.0, 2.0, 2.5, (185, 195, 200))
        self.limites = (0.0, 0.0, 30.0, 2.0)
        self.couleur_sol = (110, 120, 130)
        self.depart = Pose2D(1.0, 1.0, 0.0)
        self.but = (28.0, 1.0)

    def _champ(self) -> None:
        self.piece = None
        self.limites = (-30.0, -30.0, 30.0, 30.0)
        self.couleur_sol = (60, 120, 90)
        self.depart = Pose2D(0.0, 0.0, 0.0)
        self.but = (10.0, 0.0)
        self.obstacles = [Cylindre(25.0, 10.0, 0.3, 4.0, couleur=(40, 60, 90), nom="arbre"),
                          Cylindre(-20.0, 22.0, 0.3, 4.0, couleur=(40, 60, 90), nom="arbre")]

    def _varier(self) -> None:
        """Petites variations de position et d'orientation selon la graine (sauf les haies)."""
        for o in self.obstacles:
            if o.genre == "haie":
                continue
            o.x += float(self._rng.uniform(-0.05, 0.05))
            o.y += float(self._rng.uniform(-0.05, 0.05))
            if isinstance(o, Boite):
                o.cap += float(self._rng.uniform(-0.05, 0.05))

    def _materiau(self) -> _Materiau:
        return _Materiau(float(self._rng.uniform(0.8, 1.25)),
                         (float(self._rng.uniform(0, 100)), float(self._rng.uniform(0, 100))),
                         float(self._rng.uniform(0, 2 * math.pi)))

    # ------------------------------------------------------------------ description
    def description(self) -> dict[str, Any]:
        objets = []
        for o in self.obstacles:
            if isinstance(o, Boite):
                objets.append({"type": "boite", "nom": o.nom, "genre": o.genre, "x": o.x,
                               "y": o.y, "longueur": o.longueur, "largeur": o.largeur,
                               "hauteur": o.hauteur, "z0": o.z0, "cap": o.cap,
                               "couleur": list(o.couleur)})
            else:
                objets.append({"type": "cylindre", "nom": o.nom, "genre": o.genre, "x": o.x,
                               "y": o.y, "rayon": o.rayon, "hauteur": o.hauteur, "z0": o.z0,
                               "couleur": list(o.couleur)})
        piece = None
        if self.piece is not None:
            p = self.piece
            piece = {"xmin": p.xmin, "ymin": p.ymin, "xmax": p.xmax, "ymax": p.ymax,
                     "hauteur": p.hauteur}
        return {"scenario": self.scenario, "graine": self.graine, "limites": list(self.limites),
                "piece": piece, "obstacles": objets,
                "depart": {"x": self.depart.x, "y": self.depart.y, "cap": self.depart.cap},
                "but": list(self.but)}

    def obstacles_bas(self, z: float = 0.322) -> list[Obstacle]:
        """Obstacles entièrement sous le plan `z` (invisibles pour le lidar)."""
        return [o for o in self.obstacles if o.haut < z]

    # ------------------------------------------------------------------ lidar
    def _plan(self, z: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Segments (M, 4) et cercles (C, 3) de la coupe du monde par le plan horizontal z."""
        cle = round(z, 4)
        if cle in self._cache_plans:
            return self._cache_plans[cle]
        segments = []
        cercles = []
        for o in self.obstacles:
            if not o.z0 <= z <= o.haut:
                continue
            if isinstance(o, Boite):
                coins = o.coins()
                for i in range(4):
                    segments.append((*coins[i], *coins[(i + 1) % 4]))
            else:
                cercles.append((o.x, o.y, o.rayon))
        if self.piece is not None and z <= self.piece.hauteur:
            p = self.piece
            coins_p = [(p.xmin, p.ymin), (p.xmax, p.ymin), (p.xmax, p.ymax), (p.xmin, p.ymax)]
            for i in range(4):
                segments.append((*coins_p[i], *coins_p[(i + 1) % 4]))
        resultat = (np.array(segments, dtype=np.float64).reshape(-1, 4),
                    np.array(cercles, dtype=np.float64).reshape(-1, 3),
                    np.zeros(0))
        self._cache_plans[cle] = resultat
        return resultat

    def distances_2d(self, ox: np.ndarray, oy: np.ndarray, angles: np.ndarray, z: float,
                     portee: float = 100.0) -> np.ndarray:
        """Distance du premier obstacle le long de rayons horizontaux à la hauteur z (inf si
        rien). Origines (N,) et angles (N,) dans le monde."""
        segments, cercles, _ = self._plan(z)
        dx, dy = np.cos(angles), np.sin(angles)
        meilleur = np.full(angles.shape, np.inf)
        if len(segments):
            px, py = segments[:, 0][None, :], segments[:, 1][None, :]
            ex = (segments[:, 2] - segments[:, 0])[None, :]
            ey = (segments[:, 3] - segments[:, 1])[None, :]
            rx, ry = px - ox[:, None], py - oy[:, None]
            denominateur = dx[:, None] * ey - dy[:, None] * ex
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (rx * ey - ry * ex) / denominateur
                s = (rx * dy[:, None] - ry * dx[:, None]) / denominateur
            ok = (np.abs(denominateur) > 1e-12) & (t > 1e-6) & (s >= 0) & (s <= 1)
            t = np.where(ok, t, np.inf)
            meilleur = np.minimum(meilleur, t.min(axis=1))
        if len(cercles):
            cx, cy, r = cercles[:, 0][None, :], cercles[:, 1][None, :], cercles[:, 2][None, :]
            fx, fy = ox[:, None] - cx, oy[:, None] - cy
            b = dx[:, None] * fx + dy[:, None] * fy
            c = fx**2 + fy**2 - r**2
            disc = b**2 - c
            with np.errstate(invalid="ignore"):
                t = -b - np.sqrt(disc)
            t = np.where((disc >= 0) & (t > 1e-6), t, np.inf)
            meilleur = np.minimum(meilleur, t.min(axis=1))
        meilleur[meilleur > portee] = np.inf
        return meilleur

    def lancer_lidar(self, pose: Pose2D, profil: Profil, pose_debut: Pose2D | None = None,
                     rng: np.random.Generator | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Un tour de lidar vu depuis `pose` (pose du rover à la fin du tour, repère du monde).

        Renvoie (angles, distances) en float32 dans base_link, polaires autour de l'origine de
        base_link comme le pilote `materiel/lidar.py`, triés par angle croissant ; les mesures
        perdues ou hors portée sont absentes. `pose_debut` : pose au début du tour (sinon le
        rover est supposé immobile pendant le tour).
        """
        rng = self._rng if rng is None else rng
        z = float(profil.valeur("lidar.z_m", 0.322))
        xl, yl = float(profil.valeur("lidar.x_m", 0.0)), float(profil.valeur("lidar.y_m", 0.0))
        dmin = float(profil.valeur("lidar.portee_min_m", 0.12))
        dmax = float(profil.valeur("lidar.portee_max_m", 12.0))
        decalage = math.radians(float(profil.valeur("lidar.decalage_deg", 0.0)))
        horaire = bool(profil.valeur("lidar.sens_horaire", True))
        n = int(profil.valeur("simulation.lidar_points", 400))
        bruit = float(profil.valeur("simulation.lidar_bruit_m", 0.01))
        pertes = float(profil.valeur("simulation.lidar_pertes", 0.02))

        # Fraction du tour de chaque faisceau : l'angle du lidar croît de 0 à 2π pendant le tour.
        fraction = (np.arange(n) + rng.uniform(0, 1)) / n
        angle_lidar = 2 * math.pi * fraction + rng.normal(0, 0.002, n)
        angle_base = decalage - angle_lidar if horaire else decalage + angle_lidar
        if pose_debut is None:
            px = np.full(n, pose.x)
            py = np.full(n, pose.y)
            cap = np.full(n, pose.cap)
        else:
            px = pose_debut.x + fraction * (pose.x - pose_debut.x)
            py = pose_debut.y + fraction * (pose.y - pose_debut.y)
            cap = pose_debut.cap + fraction * normaliser_angle(pose.cap - pose_debut.cap)
        c, s = np.cos(cap), np.sin(cap)
        ox = px + c * xl - s * yl
        oy = py + s * xl + c * yl
        d = self.distances_2d(ox, oy, cap + angle_base, z, portee=dmax + 1.0)
        d = d + rng.normal(0, bruit, n)
        garde = np.isfinite(d) & (d >= dmin) & (d <= dmax) & (rng.uniform(0, 1, n) >= pertes)
        d, a = d[garde], angle_base[garde]
        # Polaire autour de l'origine de base_link (le pilote fait de même).
        x = xl + d * np.cos(a)
        y = yl + d * np.sin(a)
        angles = np.arctan2(y, x)
        distances = np.hypot(x, y)
        ordre = np.argsort(angles)
        return angles[ordre].astype(np.float32), distances[ordre].astype(np.float32)

    # ------------------------------------------------------------------ collisions
    def collision(self, pose: Pose2D, profil: Profil, marge: float = 0.0) -> bool:
        """Vrai si l'emprise du rover (agrandie de `marge`) touche un obstacle ou un mur."""
        return self.distance_obstacles(pose, profil) <= marge

    def distance_obstacles(self, pose: Pose2D, profil: Profil) -> float:
        """Distance (m) entre l'emprise du rover et l'obstacle le plus proche (≤ 0 : contact).

        Seuls comptent les obstacles qui descendent sous le haut du rover (un plateau de table
        à 0,72 m ne gêne pas) et dépassent 2 cm du sol.
        """
        g = profil.geometrie
        arriere, avant = float(g.emprise_arriere_m), float(g.emprise_avant_m)
        demi = float(g.emprise_demi_largeur_m)
        hauteur_rover = float(profil.valeur("geometrie.hauteur_max_m", 0.36))
        c, s = math.cos(pose.cap), math.sin(pose.cap)
        locaux = np.array([[avant, demi], [arriere, demi], [arriere, -demi], [avant, -demi]])
        coins = locaux @ np.array([[c, s], [-s, c]]) + (pose.x, pose.y)
        meilleur = math.inf
        if self.piece is not None:
            p = self.piece
            marges = np.concatenate([coins[:, 0] - p.xmin, p.xmax - coins[:, 0],
                                     coins[:, 1] - p.ymin, p.ymax - coins[:, 1]])
            meilleur = min(meilleur, float(marges.min()))
        for o in self.obstacles:
            if o.z0 >= hauteur_rover or o.haut < 0.02:
                continue
            if isinstance(o, Boite):
                d = _distance_polygones(coins, o.coins())
            else:
                # Cercle contre rectangle, dans le repère du rover.
                lx, ly = pose.vers_local(o.x, o.y)
                qx = min(max(lx, arriere), avant)
                qy = min(max(ly, -demi), demi)
                d = math.hypot(lx - qx, ly - qy) - o.rayon
                if arriere <= lx <= avant and -demi <= ly <= demi:
                    d = -o.rayon
            meilleur = min(meilleur, d)
        return meilleur

    # ------------------------------------------------------------------ caméras
    def placer_mire(self, centre: tuple[float, float, float], roulis: float = 0.0,
                    tangage: float = 0.0, lacet: float = 0.0, colonnes: int = 9,
                    lignes: int = 6, case_m: float = 0.025) -> Mire:
        """Pose une mire en damier face à l'axe −x du monde (vers une caméra qui regarde +x),
        tournée de (roulis, tangage, lacet). Renvoie la mire (aussi dans `self.mire`)."""
        # Mire de face : colonnes vers −y (droite de la caméra), lignes vers −z, normale −x.
        face = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        cr, sr = math.cos(roulis), math.sin(roulis)
        cp, sp = math.cos(tangage), math.sin(tangage)
        cl, sl = math.cos(lacet), math.sin(lacet)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cl, -sl, 0], [sl, cl, 0], [0, 0, 1]])
        self.mire = Mire(np.array(centre, dtype=np.float64), rz @ ry @ rx @ face, colonnes,
                         lignes, case_m)
        return self.mire

    def _directions(self, k: np.ndarray, largeur: int, hauteur: int) -> np.ndarray:
        cle = (largeur, hauteur, *np.round(k.reshape(-1), 6))
        if cle not in self._cache_directions:
            colonnes = (np.arange(largeur, dtype=np.float64) - k[0, 2]) / k[0, 0]
            lignes = (np.arange(hauteur, dtype=np.float64) - k[1, 2]) / k[1, 1]
            xn, yn = np.meshgrid(colonnes, lignes)
            d = np.stack([xn, yn, np.ones_like(xn)], axis=-1)
            d /= np.linalg.norm(d, axis=-1, keepdims=True)
            if len(self._cache_directions) > 8:
                self._cache_directions.clear()
            self._cache_directions[cle] = d.astype(np.float32)
        return self._cache_directions[cle]

    def rendre_camera(self, pose: Pose2D, camera: str, largeur: int, hauteur: int,
                      profil: Profil, surechantillonnage: int = 1, bruit: float = 1.5,
                      rng: np.random.Generator | None = None) -> np.ndarray:
        """Image BGR (hauteur, largeur, 3) uint8 de la caméra `camera` du rover en `pose`."""
        rng = self._rng if rng is None else rng
        modele = ModeleCamera(profil, camera, calibration=False)
        ss = max(1, int(surechantillonnage))
        w, h = largeur * ss, hauteur * ss
        k = modele.matrice(w, h)
        focale = float(k[0, 0])
        c, s = math.cos(pose.cap), math.sin(pose.cap)
        lacet = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        r_monde = (lacet @ modele.rotation).astype(np.float32)  # caméra → monde
        ox, oy = pose.vers_parent(float(modele.centre[0]), float(modele.centre[1]))
        origine = np.array([ox, oy, float(modele.centre[2])], dtype=np.float32)
        d = self._directions(k, w, h) @ r_monde.T  # (h, w, 3)

        t = np.full((h, w), np.inf, dtype=np.float32)
        ident = np.full((h, w), CIEL, dtype=np.int16)
        dz = d[..., 2]
        with np.errstate(divide="ignore"):
            t_sol = np.where(dz < -1e-6, -origine[2] / dz, np.inf).astype(np.float32)
        t = np.minimum(t, t_sol)
        ident[np.isfinite(t_sol)] = SOL
        if self.piece is not None:
            t_piece = self._sortie_piece(origine, d)
            plus_pres = t_piece < t
            t[plus_pres] = t_piece[plus_pres]
            ident[plus_pres] = PIECE

        projection = _Projection(modele.rotation, lacet, origine, k, w, h)
        for i, o in enumerate(self.obstacles):
            zone = projection.zone(o)
            if zone is None:
                continue
            l0, l1, c0, c1 = zone
            dd = d[l0:l1, c0:c1]
            if isinstance(o, Boite):
                t_o = _intersection_boite(o, origine, dd)
            else:
                t_o = _intersection_cylindre(o, origine, dd)
            bloc_t = t[l0:l1, c0:c1]
            plus_pres = t_o < bloc_t
            bloc_t[plus_pres] = t_o[plus_pres]
            ident[l0:l1, c0:c1][plus_pres] = 2 + i
        id_mire = 2 + len(self.obstacles)
        if self.mire is not None:
            t_m = _intersection_mire(self.mire, origine, d)
            plus_pres = t_m < t
            t[plus_pres] = t_m[plus_pres]
            ident[plus_pres] = id_mire

        image = self._ombrer(origine, d, t, ident, focale)
        if ss > 1:
            import cv2

            image = cv2.resize(image, (largeur, hauteur), interpolation=cv2.INTER_AREA)
        if bruit > 0:
            # Bruit du capteur, tiré d'une réserve à un décalage aléatoire (même grain sur les
            # trois canaux).
            n = hauteur * largeur
            debut = int(rng.integers(0, len(self._grain) - n)) if n < len(self._grain) else 0
            grain = self._grain[debut:debut + n]
            if len(grain) < n:
                grain = np.resize(grain, n)
            grain = (grain * np.float32(bruit)).reshape(hauteur, largeur, 1)
            image = image + grain
        sortie: np.ndarray = np.clip(image, 0, 255).astype(np.uint8)
        return sortie

    def detecter_camera(self, pose: Pose2D, camera: str, profil: Profil,
                        largeur: int = 160, hauteur: int = 90) -> list[dict[str, Any]]:
        """Objets effectivement visibles (non occultés) dans l'image de `camera` en `pose` :
        boîte 2D normalisée `(u0, v0, u1, v1)`, point `(u, v)`, `label` et `categorie`."""
        libelles = {
            "canape": "canapé", "table_basse": "table basse", "table": "table",
            "chaise": "chaise", "armoire": "armoire", "bibliotheque": "bibliothèque",
            "poteau": "poteau", "carton": "carton", "poutre": "poutre",
            "marche": "marche", "pot": "pot", "carton_bas": "carton bas",
            "jouet": "jouet", "arbre": "arbre", "banc": "banc",
            "abri": "abri", "rocher": "rocher", "bordure": "bordure",
        }
        modele = ModeleCamera(profil, camera, calibration=False)
        w, h = largeur, hauteur
        k = modele.matrice(w, h)
        c, s = math.cos(pose.cap), math.sin(pose.cap)
        lacet = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        r_monde = (lacet @ modele.rotation).astype(np.float32)
        ox, oy = pose.vers_parent(float(modele.centre[0]), float(modele.centre[1]))
        origine = np.array([ox, oy, float(modele.centre[2])], dtype=np.float32)
        d = self._directions(k, w, h) @ r_monde.T

        t = np.full((h, w), np.inf, dtype=np.float32)
        ident = np.full((h, w), CIEL, dtype=np.int16)
        dz = d[..., 2]
        with np.errstate(divide="ignore"):
            t_sol = np.where(dz < -1e-6, -origine[2] / dz, np.inf).astype(np.float32)
        t = np.minimum(t, t_sol)
        ident[np.isfinite(t_sol)] = SOL
        if self.piece is not None:
            t_piece = self._sortie_piece(origine, d)
            plus_pres = t_piece < t
            t[plus_pres] = t_piece[plus_pres]
            ident[plus_pres] = PIECE

        projection = _Projection(modele.rotation, lacet, origine, k, w, h)
        for i, o in enumerate(self.obstacles):
            zone = projection.zone(o)
            if zone is None:
                continue
            l0, l1, c0, c1 = zone
            dd = d[l0:l1, c0:c1]
            t_o = (_intersection_boite(o, origine, dd) if isinstance(o, Boite)
                   else _intersection_cylindre(o, origine, dd))
            bloc_t = t[l0:l1, c0:c1]
            plus_pres = t_o < bloc_t
            bloc_t[plus_pres] = t_o[plus_pres]
            ident[l0:l1, c0:c1][plus_pres] = 2 + i

        detections: list[dict[str, Any]] = []
        for i, o in enumerate(self.obstacles):
            if o.nom in ("pied_table", "pied_table_basse", "haie"):
                continue
            yy, xx = np.nonzero(ident == 2 + i)
            if len(xx) < 6:
                continue
            dist_med = float(np.median(t[yy, xx]))
            if dist_med > 7.5:
                continue
            u0, u1 = float(xx.min() / w), float(min(w, xx.max() + 1) / w)
            v0, v1 = float(yy.min() / h), float(min(h, yy.max() + 1) / h)
            u = float((np.median(xx) + 0.5) / w)
            v = float((np.percentile(yy, 80) + 0.5) / h)
            categorie = ("obstacle_bas" if o.genre == "bas"
                         else "pilier" if isinstance(o, Cylindre) and o.haut >= 1.2
                         else "bloc")
            detections.append({
                "u": round(u, 4),
                "v": round(v, 4),
                "boite": (round(u0, 4), round(v0, 4), round(u1, 4), round(v1, 4)),
                "label": libelles.get(o.nom, o.nom),
                "categorie": categorie,
                "distance": dist_med,
            })
        detections.sort(key=lambda item: float(item["distance"]))
        return detections[:10]

    def _sortie_piece(self, o: np.ndarray, d: np.ndarray) -> np.ndarray:
        """Distance de sortie de la pièce (murs, plafond) le long des rayons."""
        p = self.piece
        assert p is not None
        t = np.full(d.shape[:2], np.inf, dtype=np.float32)
        with np.errstate(divide="ignore", invalid="ignore"):
            for axe, bas, haut in ((0, p.xmin, p.xmax), (1, p.ymin, p.ymax), (2, -1e3, p.hauteur)):
                composante = d[..., axe]
                borne = np.where(composante > 0, haut, bas)
                t_axe = (borne - o[axe]) / composante
                t_axe = np.where(np.abs(composante) > 1e-9, t_axe, np.inf)
                t = np.minimum(t, t_axe.astype(np.float32))
        return t

    def _ombrer(self, o: np.ndarray, d: np.ndarray, t: np.ndarray, ident: np.ndarray,
                focale: float) -> np.ndarray:
        """Couleur (float32) de chaque pixel : couleur de base × texture × éclairage."""
        h, w = t.shape
        # Pixels triés par surface : chaque surface est traitée sur une tranche contiguë.
        ids = ident.reshape(-1)
        ordre = np.argsort(ids, kind="stable")
        ids_tries = ids[ordre]
        presents, debuts = np.unique(ids_tries, return_index=True)
        fins = np.append(debuts[1:], len(ids_tries))
        d_tries = d.reshape(-1, 3)[ordre]
        t_tries = t.reshape(-1)[ordre]
        couleurs = np.empty((len(ids_tries), 3), dtype=np.float32)
        id_mire = 2 + len(self.obstacles)
        lumiere = LUMIERE.astype(np.float32)
        for i, a, b in zip(presents.tolist(), debuts.tolist(), fins.tolist(), strict=True):
            if i == CIEL:
                couleurs[a:b] = self.couleur_ciel
                continue
            dd, tt = d_tries[a:b], t_tries[a:b]
            q = o + dd * tt[:, None]
            if i == SOL:
                s_, r_ = q[:, 0], q[:, 1]
                n_ = np.broadcast_to(np.array([0.0, 0.0, 1.0], dtype=np.float32), q.shape)
                couleur = self.couleur_sol
            elif i == PIECE and self.piece is not None:
                s_, r_, n_ = _coordonnees_piece(self.piece, q)
                couleur = self.piece.couleur
            elif i == id_mire and self.mire is not None:
                couleurs[a:b] = _couleur_mire(self.mire, q)[:, None]
                continue
            else:
                o_i = self.obstacles[i - 2]
                if isinstance(o_i, Boite):
                    s_, r_, n_ = _coordonnees_boite(o_i, q)
                else:
                    s_, r_, n_ = _coordonnees_cylindre(o_i, q)
                couleur = o_i.couleur
            materiau = self._materiaux[(i + 1) % len(self._materiaux)]
            incidence = np.maximum(np.abs(np.einsum("ij,ij->i", n_, dd)), 0.15)
            texture = self._echantillonner(s_, r_, tt / focale / incidence, materiau)
            eclairage = 0.62 + 0.38 * np.maximum(n_ @ lumiere, 0.0)
            facteur = (0.35 + 1.3 * texture) * eclairage
            couleurs[a:b] = facteur[:, None] * np.asarray(couleur, dtype=np.float32)
        image = np.empty((h * w, 3), dtype=np.float32)
        image[ordre] = couleurs
        return image.reshape(h, w, 3)

    def _echantillonner(self, s: np.ndarray, r: np.ndarray, taille_pixel: np.ndarray,
                        materiau: _Materiau) -> np.ndarray:
        """Texture (0..1) aux coordonnées (s, r) en mètres, niveau de détail selon la taille
        du pixel sur la surface."""
        texel = TEXEL_M * materiau.echelle
        c, sn = np.float32(math.cos(materiau.angle)), np.float32(math.sin(materiau.angle))
        s, r = c * s - sn * r, sn * s + c * r
        # Niveau = arrondi de log2(taille du pixel / texel) : exposant de frexp de x·√2.
        _, exposant = np.frexp(taille_pixel * np.float32(math.sqrt(2) / texel))
        niveau = np.clip(exposant - 1, 0, NIVEAUX_TEXTURE - 1)
        n = TAILLE_TUILE  # puissance de 2 : le modulo est un masque binaire
        ix = np.floor((s + materiau.decalage[0]) * np.float32(1 / texel)).astype(np.int32) & (n - 1)
        iy = np.floor((r + materiau.decalage[1]) * np.float32(1 / texel)).astype(np.int32) & (n - 1)
        indice = (niveau.astype(np.int32) * n + iy) * n + ix
        valeurs: np.ndarray = self._tuile_plate.take(indice).astype(np.float32)
        valeurs *= np.float32(1 / 255)
        return valeurs


# ---------------------------------------------------------------- géométrie du rendu


class _Projection:
    """Projette les boîtes englobantes des objets pour ne lancer leurs rayons que dans leur
    rectangle de l'image."""

    def __init__(self, rotation: np.ndarray, lacet: np.ndarray, origine: np.ndarray,
                 k: np.ndarray, largeur: int, hauteur: int) -> None:
        self.r = (lacet @ rotation).T  # monde → caméra
        self.o = origine.astype(np.float64)
        self.k = k
        self.w, self.h = largeur, hauteur

    def zone(self, o: Obstacle) -> tuple[int, int, int, int] | None:
        if isinstance(o, Boite):
            coins2 = o.coins()
        else:
            r = o.rayon
            coins2 = np.array([[o.x + r, o.y + r], [o.x - r, o.y + r], [o.x - r, o.y - r],
                               [o.x + r, o.y - r]])
        coins = np.vstack([np.column_stack((coins2, np.full(4, o.z0))),
                           np.column_stack((coins2, np.full(4, o.haut)))])
        cam = (coins - self.o) @ self.r.T
        if np.all(cam[:, 2] <= 0.01):
            return None
        if np.any(cam[:, 2] <= 0.01):
            return 0, self.h, 0, self.w  # en partie derrière la caméra : toute l'image
        px = self.k[0, 0] * cam[:, 0] / cam[:, 2] + self.k[0, 2]
        py = self.k[1, 1] * cam[:, 1] / cam[:, 2] + self.k[1, 2]
        c0, c1 = int(max(math.floor(px.min()) - 1, 0)), int(min(math.ceil(px.max()) + 2, self.w))
        l0, l1 = int(max(math.floor(py.min()) - 1, 0)), int(min(math.ceil(py.max()) + 2, self.h))
        if c0 >= c1 or l0 >= l1:
            return None
        return l0, l1, c0, c1


def _intersection_boite(b: Boite, o: np.ndarray, d: np.ndarray) -> np.ndarray:
    """Distance d'entrée des rayons (origine o, directions d (h, w, 3)) dans la boîte."""
    c, s = math.cos(b.cap), math.sin(b.cap)
    ox, oy = o[0] - b.x, o[1] - b.y
    lox, loy = c * ox + s * oy, -s * ox + c * oy
    ldx = c * d[..., 0] + s * d[..., 1]
    ldy = -s * d[..., 0] + c * d[..., 1]
    entree = np.full(d.shape[:2], -np.inf, dtype=np.float32)
    sortie = np.full(d.shape[:2], np.inf, dtype=np.float32)
    with np.errstate(divide="ignore", invalid="ignore"):
        for origine, direction, bas, haut in ((lox, ldx, -b.longueur / 2, b.longueur / 2),
                                              (loy, ldy, -b.largeur / 2, b.largeur / 2),
                                              (o[2], d[..., 2], b.z0, b.haut)):
            t1 = (bas - origine) / direction
            t2 = (haut - origine) / direction
            parallele = np.abs(direction) < 1e-9
            dehors = parallele & ((origine < bas) | (origine > haut))
            tmin = np.where(parallele, -np.inf, np.minimum(t1, t2))
            tmax = np.where(parallele, np.inf, np.maximum(t1, t2))
            tmax = np.where(dehors, -np.inf, tmax)
            entree = np.maximum(entree, tmin)
            sortie = np.minimum(sortie, tmax)
    touche = (entree <= sortie) & (entree > 1e-4)
    return np.where(touche, entree, np.inf).astype(np.float32)


def _intersection_cylindre(cy: Cylindre, o: np.ndarray, d: np.ndarray) -> np.ndarray:
    fx, fy = o[0] - cy.x, o[1] - cy.y
    a = d[..., 0] ** 2 + d[..., 1] ** 2
    b = 2 * (fx * d[..., 0] + fy * d[..., 1])
    c = fx * fx + fy * fy - cy.rayon**2
    disc = b * b - 4 * a * c
    with np.errstate(invalid="ignore", divide="ignore"):
        t_cote = (-b - np.sqrt(disc)) / (2 * a)
        z = o[2] + t_cote * d[..., 2]
        cote = (disc >= 0) & (a > 1e-12) & (t_cote > 1e-4) & (z >= cy.z0) & (z <= cy.haut)
        resultat = np.where(cote, t_cote, np.inf)
        # Dessus (et dessous si l'objet flotte au-dessus de la caméra).
        for plan in (cy.haut, cy.z0):
            t_plan = (plan - o[2]) / d[..., 2]
            px = fx + t_plan * d[..., 0]
            py = fy + t_plan * d[..., 1]
            dessus = (t_plan > 1e-4) & (px * px + py * py <= cy.rayon**2)
            resultat = np.where(dessus & (t_plan < resultat), t_plan, resultat)
    return resultat.astype(np.float32)


def _intersection_mire(m: Mire, o: np.ndarray, d: np.ndarray) -> np.ndarray:
    normale = m.rotation[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        denominateur = d @ normale.astype(np.float32)
        t = ((m.centre - o) @ normale) / denominateur
    p = o + d * np.where(np.isfinite(t), t, 0)[..., None]
    a = (p - m.centre) @ m.rotation[:, 0]
    b = (p - m.centre) @ m.rotation[:, 1]
    demi_a = (m.colonnes + 1 + 2 * m.bord_cases) * m.case_m / 2
    demi_b = (m.lignes + 1 + 2 * m.bord_cases) * m.case_m / 2
    dedans = (np.abs(a) <= demi_a) & (np.abs(b) <= demi_b) & (t > 1e-4)
    return np.where(dedans, t, np.inf).astype(np.float32)


def _couleur_mire(m: Mire, q: np.ndarray) -> np.ndarray:
    a = (q - m.centre) @ m.rotation[:, 0]
    b = (q - m.centre) @ m.rotation[:, 1]
    # Origine au coin du damier : les coins intérieurs tombent sur des multiples de la case.
    ia = np.floor(a / m.case_m + (m.colonnes + 1) / 2).astype(np.int64)
    ib = np.floor(b / m.case_m + (m.lignes + 1) / 2).astype(np.int64)
    damier = (ia >= 0) & (ia <= m.colonnes) & (ib >= 0) & (ib <= m.lignes)
    noir = damier & ((ia + ib) % 2 == 0)
    return np.where(noir, 25.0, 235.0).astype(np.float32)


def _coordonnees_piece(p: Piece, q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ecarts = np.column_stack([np.abs(q[:, 0] - p.xmin), np.abs(q[:, 0] - p.xmax),
                              np.abs(q[:, 1] - p.ymin), np.abs(q[:, 1] - p.ymax),
                              np.abs(q[:, 2] - p.hauteur)])
    face = np.argmin(ecarts, axis=1)
    normales = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, -1]],
                        dtype=np.float32)[face]
    s = np.where(face < 2, q[:, 1], q[:, 0])
    r = np.where(face < 4, q[:, 2], q[:, 1])
    return s, r, normales


def _coordonnees_boite(b: Boite, q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    c, s = math.cos(b.cap), math.sin(b.cap)
    dx, dy = q[:, 0] - b.x, q[:, 1] - b.y
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    lz = q[:, 2]
    ecarts = np.column_stack([np.abs(np.abs(lx) - b.longueur / 2),
                              np.abs(np.abs(ly) - b.largeur / 2),
                              np.minimum(np.abs(lz - b.haut), np.abs(lz - b.z0))])
    face = np.argmin(ecarts, axis=1)
    nx = np.where(face == 0, np.sign(lx), 0.0)
    ny = np.where(face == 1, np.sign(ly), 0.0)
    nz = np.where(face == 2, np.where(np.abs(lz - b.haut) < np.abs(lz - b.z0), 1.0, -1.0), 0.0)
    normales = np.column_stack([c * nx - s * ny, s * nx + c * ny, nz]).astype(np.float32)
    s_ = np.where(face == 0, ly, lx)
    r_ = np.where(face == 2, ly, lz)
    return s_, r_, normales


def _coordonnees_cylindre(cy: Cylindre, q: np.ndarray
                          ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    dx, dy = q[:, 0] - cy.x, q[:, 1] - cy.y
    rayon = np.hypot(dx, dy)
    dessus = (np.abs(q[:, 2] - cy.haut) < 1e-3) | (np.abs(q[:, 2] - cy.z0) < 1e-3)
    dessus &= rayon < cy.rayon * 0.999
    angle = np.arctan2(dy, dx)
    s = np.where(dessus, dx, angle * cy.rayon)
    r = np.where(dessus, dy, q[:, 2])
    with np.errstate(invalid="ignore", divide="ignore"):
        nx = np.where(dessus, 0.0, dx / np.maximum(rayon, 1e-9))
        ny = np.where(dessus, 0.0, dy / np.maximum(rayon, 1e-9))
    nz = np.where(dessus, np.sign(q[:, 2] - cy.z0 - 1e-6), 0.0)
    return s, r, np.column_stack([nx, ny, nz]).astype(np.float32)


def _distance_polygones(a: np.ndarray, b: np.ndarray) -> float:
    """Distance entre deux polygones convexes (4 coins) ; négative s'ils se chevauchent
    (profondeur de pénétration minimale selon les axes séparateurs)."""
    penetration = math.inf
    for poly in (a, b):
        for i in range(len(poly)):
            arete = poly[(i + 1) % len(poly)] - poly[i]
            axe = np.array([-arete[1], arete[0]])
            axe /= np.linalg.norm(axe) + 1e-12
            pa, pb = a @ axe, b @ axe
            ecart = max(pb.min() - pa.max(), pa.min() - pb.max())
            if ecart > 0:
                return _distance_separes(a, b)
            penetration = min(penetration, -ecart)
    return -penetration


def _distance_separes(a: np.ndarray, b: np.ndarray) -> float:
    meilleur = math.inf
    for p, poly in ((a, b), (b, a)):
        for i in range(len(poly)):
            u, v = poly[i], poly[(i + 1) % len(poly)]
            e = v - u
            t = np.clip(((p - u) @ e) / max(float(e @ e), 1e-12), 0, 1)
            proj = u + t[:, None] * e
            meilleur = min(meilleur, float(np.hypot(*(p - proj).T).min()))
    return meilleur


# ---------------------------------------------------------------- cinématique


def deplacer(pose: Pose2D, vitesse: float, braquage: float, dt: float,
             empattement: float = 0.38) -> Pose2D:
    """Modèle bicyclette exact (arc de cercle) de l'essieu arrière pendant `dt`."""
    distance = vitesse * dt
    courbure = math.tan(braquage) / empattement
    dtheta = distance * courbure
    if abs(dtheta) < 1e-9:
        dx, dy = distance, 0.0
    else:
        dx = math.sin(dtheta) / courbure
        dy = (1 - math.cos(dtheta)) / courbure
    return pose.composer(Pose2D(dx, dy, dtheta))

