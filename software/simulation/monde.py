"""Génération déterministe de la forêt à partir de la graine du scénario.

Le monde n'existe que dans le simulateur (physique, lidar) et dans l'affichage du poste
opérateur. Il n'est jamais transmis au cerveau.
"""

from __future__ import annotations

import base64
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from services.protocole.scenario import ScenarioPublic
from simulation.relief import ParamsRelief, Relief, generer_relief

# --------------------------------------------------------------------------- paramètres


class _Modele(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class ParamsPiste(_Modele):
    source: str = ""
    points: list[tuple[float, float]] = Field(min_length=2)
    demi_largeur: float = Field(gt=0)
    degagement: float = Field(ge=0)
    """Distance supplémentaire, au-delà de la bande de roulement, sans arbre ni rocher."""


class ParamsClairiere(_Modele):
    x: float
    y: float
    rayon: float = Field(gt=0)


class ParamsArbres(_Modele):
    nombre: int = Field(ge=0)
    rayon_min: float = Field(gt=0)
    rayon_max: float = Field(gt=0)
    espacement_min: float = Field(gt=0)
    hauteur_min: float = Field(gt=0)
    hauteur_max: float = Field(gt=0)


class ParamsRochers(_Modele):
    nombre: int = Field(ge=0)
    rayon_min: float = Field(gt=0)
    rayon_max: float = Field(gt=0)
    hauteur_min: float = Field(gt=0)
    hauteur_max: float = Field(gt=0)


class RocherFixe(_Modele):
    x: float
    y: float
    rayon: float = Field(gt=0)
    hauteur: float = Field(gt=0)


class ParamsLisiere(_Modele):
    """Bande de forêt autour de la zone : physique (le lidar la voit), pas de mission."""

    largeur: float = Field(ge=0)
    densite: float = Field(ge=0)
    """Arbres par mètre carré."""


class TroncCoucheFixe(_Modele):
    x: float
    y: float
    angle_deg: float
    longueur: float = Field(gt=0)
    rayon: float = Field(gt=0)


class ParamsTroncsCouches(_Modele):
    """Troncs couchés au sol : plus bas que le plan du lidar, seule la caméra peut les voir."""

    nombre: int = Field(ge=0)
    longueur_min: float = Field(gt=0)
    longueur_max: float = Field(gt=0)
    rayon_min: float = Field(gt=0)
    rayon_max: float = Field(gt=0)
    fixes: list[TroncCoucheFixe] = Field(default_factory=list)


class ParamsMonde(_Modele):
    piste: ParamsPiste
    clairiere: ParamsClairiere
    arbres: ParamsArbres
    rochers: ParamsRochers
    rochers_fixes: list[RocherFixe] = Field(default_factory=list)
    degagement_lieux: float = Field(ge=0)
    lisiere: ParamsLisiere = ParamsLisiere(largeur=0.0, densite=0.0)
    troncs_couches: ParamsTroncsCouches = ParamsTroncsCouches(
        nombre=0, longueur_min=1.0, longueur_max=1.0, rayon_min=0.1, rayon_max=0.1
    )
    relief: ParamsRelief | None = None


# --------------------------------------------------------------------------- monde généré


@dataclass(frozen=True, slots=True)
class Arbre:
    x: float
    y: float
    rayon: float
    hauteur: float


@dataclass(frozen=True, slots=True)
class Rocher:
    """Prisme convexe : contour au sol (sens trigonométrique), sommet réduit de `retrait_sommet`."""

    x: float
    y: float
    contour: tuple[tuple[float, float], ...]
    hauteur: float
    retrait_sommet: float = 0.85

    @property
    def rayon(self) -> float:
        return max(math.hypot(px, py) for px, py in self.contour)


@dataclass(frozen=True, slots=True)
class TroncCouche:
    """Cylindre couché sur le sol, centré en (x, y), d'axe horizontal orienté selon `angle`."""

    x: float
    y: float
    angle: float
    longueur: float
    rayon: float

    @property
    def extremites(self) -> tuple[tuple[float, float], tuple[float, float]]:
        dx = 0.5 * self.longueur * math.cos(self.angle)
        dy = 0.5 * self.longueur * math.sin(self.angle)
        return (self.x - dx, self.y - dy), (self.x + dx, self.y + dy)


@dataclass(frozen=True)
class Monde:
    graine: int
    scenario: ScenarioPublic
    params: ParamsMonde
    arbres: list[Arbre] = field(default_factory=list)
    rochers: list[Rocher] = field(default_factory=list)
    lisiere: list[Arbre] = field(default_factory=list)
    """Arbres hors de la zone de mission, dans la bande de lisière (physiques eux aussi)."""
    piste: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    """Ligne centrale de la piste, rééchantillonnée tous les 0,5 m."""
    troncs_couches: list[TroncCouche] = field(default_factory=list)
    relief: Relief | None = None

    def hauteur_sol(self, x: float, y: float) -> float:
        return 0.0 if self.relief is None else self.relief.h(x, y)

    def en_json(self) -> dict[str, object]:
        """Géométrie pour l'affichage du poste opérateur (jamais envoyée au cerveau)."""
        return {
            "graine": self.graine,
            "limites": self.scenario.limites.model_dump(),
            "piste": {
                "points": np.round(self.piste, 3).tolist(),
                "demi_largeur": self.params.piste.demi_largeur,
            },
            "clairiere": self.params.clairiere.model_dump(),
            "arbres": [
                [round(a.x, 3), round(a.y, 3), round(a.rayon, 3), round(a.hauteur, 2)]
                for a in self.arbres
            ],
            "lisiere": [
                [round(a.x, 3), round(a.y, 3), round(a.rayon, 3), round(a.hauteur, 2)]
                for a in self.lisiere
            ],
            "largeur_lisiere": self.params.lisiere.largeur,
            "troncs_couches": [
                [
                    round(t.x, 3),
                    round(t.y, 3),
                    round(t.angle, 4),
                    round(t.longueur, 3),
                    round(t.rayon, 3),
                ]
                for t in self.troncs_couches
            ],
            "rochers": [
                {
                    "x": round(r.x, 3),
                    "y": round(r.y, 3),
                    "hauteur": round(r.hauteur, 3),
                    "retrait_sommet": r.retrait_sommet,
                    "contour": [[round(px, 3), round(py, 3)] for px, py in r.contour],
                }
                for r in self.rochers
            ],
            "relief": self._relief_json(),
        }

    def _relief_json(self) -> dict[str, object] | None:
        """Hauteurs au pas de 0,5 m, en centimètres (int16 little-endian, base64)."""
        if self.relief is None:
            return None
        r = self.relief
        facteur = max(1, round(0.5 / r.pas))
        h = r.hauteurs[::facteur, ::facteur]
        donnees = np.round(h * 100).astype("<i2").tobytes()
        return {
            "origine": [round(r.origine[0], 3), round(r.origine[1], 3)],
            "pas": r.pas * facteur,
            "lignes": int(h.shape[0]),
            "colonnes": int(h.shape[1]),
            "hauteurs_cm": base64.b64encode(donnees).decode("ascii"),
        }


# --------------------------------------------------------------------------- géométrie


def _catmull_rom(points: np.ndarray, pas: float) -> np.ndarray:
    """Courbe de Catmull-Rom passant par les points, rééchantillonnée tous les `pas` mètres."""
    etendus = np.vstack([2 * points[0] - points[1], points, 2 * points[-1] - points[-2]])
    dense: list[np.ndarray] = []
    for i in range(1, len(etendus) - 2):
        p0, p1, p2, p3 = etendus[i - 1 : i + 3]
        t = np.linspace(0.0, 1.0, 40, endpoint=False)[:, None]
        t2, t3 = t * t, t * t * t
        dense.append(
            0.5
            * (
                2 * p1
                + (-p0 + p2) * t
                + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                + (-p0 + 3 * p1 - 3 * p2 + p3) * t3
            )
        )
    dense.append(points[-1:])
    courbe = np.vstack(dense)
    segments = np.linalg.norm(np.diff(courbe, axis=0), axis=1)
    abscisse = np.concatenate([[0.0], np.cumsum(segments)])
    cibles = np.arange(0.0, abscisse[-1], pas)
    return np.column_stack(
        [np.interp(cibles, abscisse, courbe[:, 0]), np.interp(cibles, abscisse, courbe[:, 1])]
    )


def distance_a_polyligne(points: np.ndarray, polyligne: np.ndarray) -> np.ndarray:
    """Distance de chaque point (N, 2) à la polyligne (M, 2)."""
    a = polyligne[:-1][None, :, :]
    b = polyligne[1:][None, :, :]
    p = points[:, None, :]
    ab = b - a
    longueur2 = np.maximum(np.sum(ab * ab, axis=2), 1e-12)
    t = np.clip(np.sum((p - a) * ab, axis=2) / longueur2, 0.0, 1.0)
    projete = a + t[..., None] * ab
    distances: np.ndarray = np.min(np.linalg.norm(p - projete, axis=2), axis=1)
    return distances


def _enveloppe_convexe(points: np.ndarray) -> list[tuple[float, float]]:
    """Enveloppe convexe (chaîne monotone d'Andrew), sens trigonométrique."""
    tries = sorted({(float(x), float(y)) for x, y in points})

    def orientation(
        o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]
    ) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    bas: list[tuple[float, float]] = []
    for p in tries:
        while len(bas) >= 2 and orientation(bas[-2], bas[-1], p) <= 0:
            bas.pop()
        bas.append(p)
    haut: list[tuple[float, float]] = []
    for p in reversed(tries):
        while len(haut) >= 2 and orientation(haut[-2], haut[-1], p) <= 0:
            haut.pop()
        haut.append(p)
    return bas[:-1] + haut[:-1]


def _contour_rocher(rng: np.random.Generator, rayon: float) -> tuple[tuple[float, float], ...]:
    """Contour convexe irrégulier d'un rocher de rayon moyen `rayon`."""
    angles = np.sort(rng.uniform(0, 2 * math.pi, 11))
    rayons = rayon * rng.uniform(0.65, 1.15, angles.size)
    allongement = rng.uniform(0.7, 1.0)
    points = np.column_stack([rayons * np.cos(angles), rayons * np.sin(angles) * allongement])
    rotation = rng.uniform(0, 2 * math.pi)
    c, s = math.cos(rotation), math.sin(rotation)
    points = points @ np.array([[c, s], [-s, c]])
    return tuple(_enveloppe_convexe(points))


# --------------------------------------------------------------------------- génération


def generer_monde(
    scenario: ScenarioPublic, params: ParamsMonde, graine: int | None = None
) -> Monde:
    """Place la piste, les rochers puis les arbres, de façon reproductible pour une graine."""
    graine = scenario.graine if graine is None else graine
    rng = np.random.default_rng(graine)
    lim = scenario.limites
    relief = None
    if params.relief is not None:
        # Emprise du sol texturé (voir camera_simulee.emprise_sol) : zone, lisière et marge.
        cx, cy = (lim.xmin + lim.xmax) / 2, (lim.ymin + lim.ymax) / 2
        demi = max(lim.xmax - lim.xmin, lim.ymax - lim.ymin) / 2 + params.lisiere.largeur + 25.0
        relief = generer_relief(params.relief, (cx, cy), demi, graine)
    piste = _catmull_rom(np.asarray(params.piste.points, dtype=float), pas=0.5)
    zone_piste = params.piste.demi_largeur + params.piste.degagement
    cl = params.clairiere
    lieux = np.array(
        [[lieu.x, lieu.y] for lieu in scenario.lieux] + [[scenario.depart.x, scenario.depart.y]]
    )

    def libre_de_decor(p: np.ndarray, rayon: float) -> np.ndarray:
        """Masque des candidats hors piste, hors clairière et loin des lieux nommés."""
        loin_piste = distance_a_polyligne(p, piste) > zone_piste + rayon
        loin_clairiere = np.hypot(p[:, 0] - cl.x, p[:, 1] - cl.y) > cl.rayon + rayon
        d_lieux = np.min(np.linalg.norm(p[:, None, :] - lieux[None, :, :], axis=2), axis=1)
        loin_lieux = d_lieux > params.degagement_lieux + rayon
        dans_zone = (
            (p[:, 0] > lim.xmin + rayon)
            & (p[:, 0] < lim.xmax - rayon)
            & (p[:, 1] > lim.ymin + rayon)
            & (p[:, 1] < lim.ymax - rayon)
        )
        masque: np.ndarray = loin_piste & loin_clairiere & loin_lieux & dans_zone
        if relief is not None:
            # Ni arbre ni rocher dans le fossé ou sur la butte : le relief y est l'obstacle.
            masque &= relief.distance_fosse(p[:, 0], p[:, 1]) > 3.0 + rayon
            masque &= ~relief.dans_butte(p[:, 0], p[:, 1], 0.5 + rayon)
        return masque

    rochers: list[Rocher] = []
    for fixe in params.rochers_fixes:
        rochers.append(Rocher(fixe.x, fixe.y, _contour_rocher(rng, fixe.rayon), fixe.hauteur))
    essais = 0
    pr = params.rochers
    while len(rochers) < len(params.rochers_fixes) + pr.nombre and essais < 5000:
        essais += 1
        rayon = float(rng.uniform(pr.rayon_min, pr.rayon_max))
        p = np.array([[rng.uniform(lim.xmin, lim.xmax), rng.uniform(lim.ymin, lim.ymax)]])
        if not libre_de_decor(p, rayon * 1.2)[0]:
            continue
        if any(math.hypot(r.x - p[0, 0], r.y - p[0, 1]) < r.rayon + rayon + 2.0 for r in rochers):
            continue
        hauteur = float(rng.uniform(pr.hauteur_min, pr.hauteur_max))
        rochers.append(Rocher(float(p[0, 0]), float(p[0, 1]), _contour_rocher(rng, rayon), hauteur))

    # Arbres : tirage par lots, puis rejet de ceux trop proches d'un voisin ou d'un rocher.
    pa = params.arbres
    arbres: list[Arbre] = []
    positions = np.zeros((0, 2))
    rayons = np.zeros(0)
    for _ in range(200):
        if len(arbres) >= pa.nombre:
            break
        n = 400
        cand = np.column_stack(
            [rng.uniform(lim.xmin, lim.xmax, n), rng.uniform(lim.ymin, lim.ymax, n)]
        )
        r_cand = rng.uniform(pa.rayon_min, pa.rayon_max, n)
        h_cand = rng.uniform(pa.hauteur_min, pa.hauteur_max, n)
        garde = libre_de_decor(cand, pa.rayon_max)
        for i in np.flatnonzero(garde):
            if len(arbres) >= pa.nombre:
                break
            p = cand[i]
            if (
                positions.size
                and np.min(np.linalg.norm(positions - p, axis=1) - rayons - r_cand[i])
                < pa.espacement_min
            ):
                continue
            if any(math.hypot(r.x - p[0], r.y - p[1]) < r.rayon + r_cand[i] + 0.8 for r in rochers):
                continue
            arbres.append(Arbre(float(p[0]), float(p[1]), float(r_cand[i]), float(h_cand[i])))
            positions = np.vstack([positions, p])
            rayons = np.append(rayons, r_cand[i])

    # Lisière : tirage uniforme dans la bande qui entoure la zone, avec le même espacement.
    pl = params.lisiere
    lisiere: list[Arbre] = []
    if pl.largeur > 0 and pl.densite > 0:
        x0, x1 = lim.xmin - pl.largeur, lim.xmax + pl.largeur
        y0, y1 = lim.ymin - pl.largeur, lim.ymax + pl.largeur
        aire = (x1 - x0) * (y1 - y0) - (lim.xmax - lim.xmin) * (lim.ymax - lim.ymin)
        cible = round(aire * pl.densite)
        voisins = positions.copy()
        rayons_voisins = rayons.copy()
        for _ in range(cible * 80):
            if len(lisiere) >= cible:
                break
            p = np.array([rng.uniform(x0, x1), rng.uniform(y0, y1)])
            r = float(rng.uniform(pa.rayon_min, pa.rayon_max))
            if lim.xmin - r < p[0] < lim.xmax + r and lim.ymin - r < p[1] < lim.ymax + r:
                continue
            if (
                voisins.size
                and np.min(np.linalg.norm(voisins - p, axis=1) - rayons_voisins - r)
                < pa.espacement_min
            ):
                continue
            lisiere.append(
                Arbre(
                    float(p[0]), float(p[1]), r, float(rng.uniform(pa.hauteur_min, pa.hauteur_max))
                )
            )
            voisins = np.vstack([voisins, p])
            rayons_voisins = np.append(rayons_voisins, r)

    # Troncs couchés : tirés en dernier, pour ne pas changer la forêt d'une graine donnée.
    ptc = params.troncs_couches
    troncs = [
        TroncCouche(f.x, f.y, math.radians(f.angle_deg), f.longueur, f.rayon) for f in ptc.fixes
    ]
    obstacles = [(a.x, a.y, a.rayon) for a in arbres] + [(r.x, r.y, r.rayon) for r in rochers]
    obstacles_xy = np.array([[x, y] for x, y, _ in obstacles]).reshape(-1, 2)
    obstacles_r = np.array([r for _, _, r in obstacles])
    essais = 0
    while len(troncs) < len(ptc.fixes) + ptc.nombre and essais < 5000:
        essais += 1
        longueur = float(rng.uniform(ptc.longueur_min, ptc.longueur_max))
        rayon = float(rng.uniform(ptc.rayon_min, ptc.rayon_max))
        tronc = TroncCouche(
            float(rng.uniform(lim.xmin, lim.xmax)),
            float(rng.uniform(lim.ymin, lim.ymax)),
            float(rng.uniform(0, math.pi)),
            longueur,
            rayon,
        )
        (xa, ya), (xb, yb) = tronc.extremites
        k = np.linspace(0.0, 1.0, max(2, math.ceil(longueur / 0.2)))[:, None]
        echantillons = np.array([xa, ya]) + k * np.array([xb - xa, yb - ya])
        if not np.all(libre_de_decor(echantillons, rayon + 0.3)):
            continue
        if obstacles_xy.size:
            d = np.linalg.norm(echantillons[:, None, :] - obstacles_xy[None, :, :], axis=2)
            if np.any(d < obstacles_r[None, :] + rayon + 0.15):
                continue
        if any(
            math.hypot(t.x - tronc.x, t.y - tronc.y) < 0.5 * (t.longueur + longueur) + 1.0
            for t in troncs
        ):
            continue
        troncs.append(tronc)

    return Monde(
        graine,
        scenario,
        params,
        arbres,
        rochers,
        piste=piste,
        lisiere=lisiere,
        troncs_couches=troncs,
        relief=relief,
    )


def charger_monde(chemin_scenario: Path, graine: int | None = None) -> Monde:
    """Lit le scénario complet (partie publique et section « monde ») et génère la forêt."""
    donnees = json.loads(chemin_scenario.read_text(encoding="utf-8"))
    params = ParamsMonde.model_validate(donnees["monde"])
    scenario = ScenarioPublic.charger(chemin_scenario)
    return generer_monde(scenario, params, graine)
