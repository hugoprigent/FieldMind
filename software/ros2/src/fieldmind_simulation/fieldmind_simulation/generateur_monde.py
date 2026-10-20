"""Générateur de mondes Gazebo réalistes pour le rover : relief, eau, végétation, rochers, hameau.

Un monde de 140 × 140 m est tiré d'une graine :
- relief : collines douces (bruit multi-échelle), une butte dont un flanc est trop raide pour le
  rover, un ruisseau encaissé (berges raides, obstacle « négatif » invisible pour un lidar 2D)
  franchissable seulement à un gué, un hameau aplani ;
- piste de terre du hameau à une clairière, en passant par le gué ;
- forêt de pins et de chênes, sous-bois, buissons, souches, troncs couchés (dont en travers de la
  piste) ; champ de rochers ; hameau avec maisons, ruine, voitures, clôture, fûts, personnes.

Le terrain est un maillage (sa géométrie exacte est connue : objets posés au centimètre, même
terrain dans l'interface web) peint d'une texture unique (herbe, sous-bois, terre, gravier,
roche sur les fortes pentes, vase). Les modèles d'arbres, rochers, maisons, véhicules et
personnes viennent de Gazebo Fuel (OpenRobotics, licences CC0 et CC-BY 4.0, voir CREDITS) ;
troncs, souches, buissons et clôtures sont générés ici.

    ros2 run fieldmind_simulation generer_monde --graine 7 --sortie ~/.cache/fieldmind/mondes/7
"""

# Gabarits SDF sur une ligne et grilles numpy X, Y, Z : exemptés de E501 et N806.
# ruff: noqa: E501, N806

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

FUEL = "https://fuel.gazebosim.org/1.0/OpenRobotics/models"
TAILLE_M = 140.0
PAS_M = 1.0
TEXTURE_PX = 2048
EMPRISES = {
    # Rectangle au sol (xmin, xmax, ymin, ymax) dans le repère du modèle, mesuré dans les maillages
    # Fuel (échelle du modèle comprise), et hauteur (m).
    "maison1": ((-12.9, 1.7, -6.1, 6.8), 5.6),
    "maison2": ((-12.3, 4.9, -7.0, 5.7), 6.0),
    "maison3": ((-8.3, 4.4, -2.4, 2.4), 10.6),
    "ruine": ((-8.1, 8.1, -6.5, 6.5), 5.8),
    "voiture": ((-2.1, 2.1, -0.9, 0.9), 1.5),
    "pickup": ((-2.8, 2.8, -1.1, 1.1), 1.9),
}


# ----------------------------------------------------------------------------- outils


def _bruit(rng: np.random.Generator, n: int, cellules: int) -> np.ndarray:
    grossier = rng.uniform(-1, 1, (cellules + 3, cellules + 3))
    fin = ndimage.zoom(grossier, (n + 2 * n // cellules) / (cellules + 3), order=3)
    decalage = n // cellules
    return fin[decalage : decalage + n, decalage : decalage + n]


def _catmull_rom(points: list[tuple[float, float]], pas: float) -> np.ndarray:
    p = np.asarray(points, float)
    p = np.vstack([2 * p[0] - p[1], p, 2 * p[-1] - p[-2]])
    dense = []
    for i in range(1, len(p) - 2):
        p0, p1, p2, p3 = p[i - 1 : i + 3]
        t = np.linspace(0, 1, 30, endpoint=False)[:, None]
        dense.append(
            0.5
            * (
                2 * p1
                + (-p0 + p2) * t
                + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t**2
                + (-p0 + 3 * p1 - 3 * p2 + p3) * t**3
            )
        )
    dense.append(p[-2:-1])
    c = np.vstack(dense)
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(c, axis=0), axis=1))])
    cibles = np.arange(0, s[-1], pas)
    return np.column_stack([np.interp(cibles, s, c[:, 0]), np.interp(cibles, s, c[:, 1])])


def _distance_polyligne(x: np.ndarray, y: np.ndarray, ligne: np.ndarray) -> np.ndarray:
    """Distance de chaque point (x, y) à la polyligne, par transformée de distance sur grille."""
    return np.min(np.hypot(x[..., None] - ligne[:, 0], y[..., None] - ligne[:, 1]), axis=-1)


@dataclass
class Objet:
    type: str
    x: float
    y: float
    z: float
    lacet: float = 0.0
    echelle: float = 1.0
    rayon: float = 0.5
    """Rayon d'encombrement au sol (m), pour l'évaluation et l'affichage."""
    hauteur: float = 1.0
    extra: dict[str, float] = field(default_factory=dict)


# ----------------------------------------------------------------------------- monde


class Monde:
    def __init__(self, graine: int) -> None:
        self.graine = graine
        self.rng = np.random.default_rng(graine)
        n = int(TAILLE_M / PAS_M) + 1
        self.n = n
        self.xs = np.linspace(-TAILLE_M / 2, TAILLE_M / 2, n)
        self.X, self.Y = np.meshgrid(self.xs, self.xs)
        self.objets: list[Objet] = []
        self._relief()

    # ------------------------------------------------------------------ relief
    def _relief(self) -> None:
        rng, X, Y = self.rng, self.X, self.Y
        n = self.n
        z = 2.2 * _bruit(rng, n, 4) + 0.7 * _bruit(rng, n, 12) + 0.15 * _bruit(rng, n, 40)
        # Butte au nord-est ; son flanc sud est une petite falaise (2,5 m sur 2 m).
        bx, by = rng.uniform(25, 40), rng.uniform(25, 40)
        self.butte = (bx, by)
        z += 8.0 * np.exp(-((X - bx) ** 2 + (Y - by) ** 2) / (2 * 13.0**2))
        falaise = 1 / (1 + np.exp(-(Y - (by - 11)) / 0.5))
        masque_falaise = np.exp(-((X - bx) ** 2) / (2 * 9.0**2))
        z += 2.5 * falaise * masque_falaise
        # Hameau aplani au sud-ouest.
        hx, hy = rng.uniform(-42, -30), rng.uniform(-42, -30)
        self.hameau = (hx, hy)
        d_h = np.hypot(X - hx, Y - hy)
        poids_h = np.clip((24 - d_h) / 8, 0, 1)
        z_h = float(np.mean(z[d_h < 6]))
        z = z * (1 - poids_h) + z_h * poids_h
        # Ruisseau d'ouest en est, qui serpente au milieu.
        y0 = rng.uniform(-8, 8)
        pts = [(-75, y0 + rng.uniform(-6, 6))]
        for x in np.linspace(-50, 50, 6):
            pts.append((float(x), y0 + rng.uniform(-9, 9)))
        pts.append((75, y0 + rng.uniform(-6, 6)))
        self.ruisseau = _catmull_rom(pts, 0.5)
        # Piste : hameau → gué → forêt → clairière (au pied de la butte, côté ouest).
        idx_gue = int(len(self.ruisseau) * rng.uniform(0.35, 0.45))
        gue = tuple(self.ruisseau[idx_gue])
        self.gue = (float(gue[0]), float(gue[1]))
        cx, cy = bx - rng.uniform(22, 30), by + rng.uniform(-6, 4)
        self.clairiere = (float(cx), float(cy))
        piste = [
            (hx + 8, hy + 6),
            (hx + 20, hy + 20),
            (gue[0] - 4, gue[1] - 10),
            gue,
            (gue[0] + 4, gue[1] + 12),
            ((gue[0] + cx) / 2 + 6, (gue[1] + cy) / 2),
            (cx, cy),
        ]
        self.piste = _catmull_rom(piste, 0.5)
        d_piste = _distance_polyligne(X, Y, self.piste)
        d_r = _distance_polyligne(X, Y, self.ruisseau)
        self.d_piste, self.d_ruisseau = d_piste, d_r
        # Piste : terrain lissé le long du tracé (profil adouci), sur 2 m de part et d'autre.
        lisse = ndimage.gaussian_filter(z, 4)
        poids_p = np.clip((3.5 - d_piste) / 1.5, 0, 1)
        z = z * (1 - poids_p) + lisse * poids_p
        # Lit du ruisseau : 0,9 m de profondeur, berges raides ; au gué, berges douces.
        d_gue = np.hypot(X - gue[0], Y - gue[1])
        largeur_berge = 1.2 + 3.5 * np.clip((6 - d_gue) / 4, 0, 1)
        profondeur = 0.9 * np.clip((1.0 + largeur_berge - d_r) / largeur_berge, 0, 1)
        profondeur *= 1 - 0.75 * np.clip((6 - d_gue) / 4, 0, 1)
        z -= profondeur
        self.niveau_eau_relatif = 0.35
        # Clairière légèrement aplanie.
        d_c = np.hypot(X - cx, Y - cy)
        poids_c = np.clip((9 - d_c) / 4, 0, 1)
        z = z * (1 - poids_c) + ndimage.gaussian_filter(z, 6) * poids_c
        self.z = z - float(z.min())
        gy, gx = np.gradient(self.z, PAS_M)
        self.pente_deg = np.degrees(np.arctan(np.hypot(gx, gy)))

    def h(self, x: float, y: float) -> float:
        """Hauteur du terrain (interpolation bilinéaire, comme le maillage)."""
        u = (x + TAILLE_M / 2) / PAS_M
        v = (y + TAILLE_M / 2) / PAS_M
        i0, j0 = (
            int(np.clip(math.floor(v), 0, self.n - 2)),
            int(np.clip(math.floor(u), 0, self.n - 2)),
        )
        fv, fu = v - i0, u - j0
        z = self.z
        return float(
            z[i0, j0] * (1 - fu) * (1 - fv)
            + z[i0, j0 + 1] * fu * (1 - fv)
            + z[i0 + 1, j0] * (1 - fu) * fv
            + z[i0 + 1, j0 + 1] * fu * fv
        )

    def _cellule(self, x: float, y: float) -> tuple[int, int]:
        i = int(np.clip(round((y + TAILLE_M / 2) / PAS_M), 0, self.n - 1))
        j = int(np.clip(round((x + TAILLE_M / 2) / PAS_M), 0, self.n - 1))
        return i, j

    def pente(self, x: float, y: float) -> float:
        return float(self.pente_deg[self._cellule(x, y)])

    def _detour_possible(self, x: float, y: float, rayon: float = 5.0) -> bool:
        """Terrain praticable (pente < 18°, pas de ruisseau) sur `rayon` m autour du point."""
        for dx in np.arange(-rayon, rayon + 0.1, 1.0):
            for dy in np.arange(-rayon, rayon + 0.1, 1.0):
                if dx * dx + dy * dy > rayon * rayon:
                    continue
                cellule = self._cellule(x + dx, y + dy)
                if self.pente_deg[cellule] > 18 or self.d_ruisseau[cellule] < 3:
                    return False
        return True

    # ------------------------------------------------------------------ objets
    def _libre(self, x: float, y: float, rayon: float, marge_piste: float = 2.5) -> bool:
        if abs(x) > TAILLE_M / 2 - 4 or abs(y) > TAILLE_M / 2 - 4:
            return False
        i, j = self._cellule(x, y)
        if self.d_piste[i, j] < marge_piste + rayon or self.d_ruisseau[i, j] < 3.0 + rayon:
            return False
        if math.hypot(x - self.hameau[0], y - self.hameau[1]) < 22 + rayon:
            return False
        if math.hypot(x - self.clairiere[0], y - self.clairiere[1]) < 8 + rayon:
            return False
        return all(
            math.hypot(o.extra.get("ex", o.x) - x, o.extra.get("ey", o.y) - y)
            > o.rayon + rayon + 0.9
            for o in self.objets
        )

    def _zone_foret(self, x: float, y: float) -> float:
        """Densité relative de la forêt (0 à 1) : nord-ouest dense, champ rocailleux à l'est."""
        f = 0.9 * (1 / (1 + math.exp((x - 5) / 8))) + 0.5 * (1 / (1 + math.exp(-(y - 10) / 8)))
        return min(1.0, f)

    def placer(self) -> None:
        rng = self.rng
        self._hameau()
        # Arbres
        for _ in range(9000):
            if sum(o.type in ("pin", "chene") for o in self.objets) >= 170:
                break
            x, y = rng.uniform(-66, 66, 2)
            if rng.random() > self._zone_foret(x, y) or self.pente(x, y) > 28:
                continue
            chene = rng.random() < 0.3
            echelle = rng.uniform(1.0, 1.5) if chene else rng.uniform(1.2, 2.1)
            rayon = (0.25 if chene else 0.15) * echelle
            if not self._libre(x, y, rayon + 0.4):
                continue
            self.objets.append(
                Objet(
                    "chene" if chene else "pin",
                    x,
                    y,
                    self.h(x, y),
                    rng.uniform(0, 6.28),
                    echelle,
                    rayon,
                    (6.5 if chene else 5.1) * echelle,
                )
            )
        # Rochers : champ à l'est, et quelques-uns dispersés
        modeles = [("rocher1", 1.49), ("rocher2", 0.95), ("rocher3", 0.65), ("rocher5", 0.42)]
        for k in range(600):
            if sum(o.type.startswith("rocher") for o in self.objets) >= 55:
                break
            champ = k % 5 != 0
            x = rng.uniform(15, 60) if champ else rng.uniform(-60, 60)
            y = rng.uniform(-55, 5) if champ else rng.uniform(-60, 60)
            nom, diametre = modeles[int(rng.integers(len(modeles)))]
            echelle = rng.uniform(0.5, 1.6)
            rayon = 0.5 * diametre * echelle
            if not self._libre(x, y, rayon, marge_piste=1.8):
                continue
            hauteur = diametre * 0.93 * echelle
            self.objets.append(
                Objet(
                    nom,
                    x,
                    y,
                    self.h(x, y) + 0.2 * hauteur,
                    rng.uniform(0, 6.28),
                    echelle,
                    rayon,
                    hauteur,
                )
            )
        # Troncs couchés : 2 en travers de la piste, les autres en forêt
        # (seulement là où un détour existe : terrain praticable autour, loin du ruisseau).
        idx = []
        for bornes in ((0.5, 0.7), (0.75, 0.95)):
            for _ in range(60):
                i = int(len(self.piste) * rng.uniform(*bornes))
                if self._detour_possible(*self.piste[i]):
                    idx.append(i)
                    break
        for i in idx:
            px, py = self.piste[i]
            dx, dy = self.piste[min(i + 1, len(self.piste) - 1)] - self.piste[max(i - 1, 0)]
            angle = math.atan2(dy, dx) + math.pi / 2 + rng.uniform(-0.3, 0.3)
            longueur, rayon = rng.uniform(3.0, 4.5), rng.uniform(0.12, 0.17)
            decal = rng.choice([-1, 1]) * rng.uniform(0.6, 1.2)
            x, y = px + decal * math.cos(angle), py + decal * math.sin(angle)
            self._tronc(x, y, angle, longueur, rayon)
        for _ in range(400):
            if sum(o.type == "tronc" for o in self.objets) >= 14:
                break
            x, y = rng.uniform(-62, 62, 2)
            if rng.random() > self._zone_foret(x, y):
                continue
            angle, longueur, rayon = rng.uniform(0, 3.14), rng.uniform(2, 6), rng.uniform(0.1, 0.22)
            if not self._libre(x, y, longueur / 2):
                continue
            self._tronc(x, y, angle, longueur, rayon)
        # Souches et buissons
        for _ in range(3000):
            if (
                sum(o.type == "souche" for o in self.objets) >= 25
                and sum(o.type == "buisson" for o in self.objets) >= 60
            ):
                break
            x, y = rng.uniform(-64, 64, 2)
            if rng.random() > 0.3 + 0.7 * self._zone_foret(x, y) or self.pente(x, y) > 30:
                continue
            if rng.random() < 0.3:
                r, hh = rng.uniform(0.18, 0.35), rng.uniform(0.2, 0.55)
                if self._libre(x, y, r, marge_piste=1.5):
                    self.objets.append(Objet("souche", x, y, self.h(x, y), 0, 1, r, hh))
            else:
                r = rng.uniform(0.4, 1.0)
                if self._libre(x, y, r, marge_piste=1.8):
                    self.objets.append(
                        Objet(
                            "buisson",
                            x,
                            y,
                            self.h(x, y),
                            rng.uniform(0, 6.28),
                            1,
                            r,
                            r * rng.uniform(0.9, 1.4),
                        )
                    )

    def _tronc(self, x: float, y: float, angle: float, longueur: float, rayon: float) -> None:
        dx, dy = 0.5 * longueur * math.cos(angle), 0.5 * longueur * math.sin(angle)
        za, zb = self.h(x - dx, y - dy), self.h(x + dx, y + dy)
        tangage = -math.atan2(zb - za, longueur)
        self.objets.append(
            Objet(
                "tronc",
                x,
                y,
                0.5 * (za + zb) + rayon * 0.85,
                angle,
                1,
                longueur / 2,
                2 * rayon,
                {"longueur": longueur, "rayon": rayon, "tangage": tangage},
            )
        )

    def _emprise(self, nom: str, x: float, y: float, lacet: float) -> tuple[float, float, float]:
        """Cercle (x, y, rayon) qui contient l'emprise tournée du modèle posé en (x, y, lacet)."""
        (x0, x1, y0, y1), _ = EMPRISES[nom]
        cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
        c, s = math.cos(lacet), math.sin(lacet)
        return x + c * cx - s * cy, y + s * cx + c * cy, 0.5 * math.hypot(x1 - x0, y1 - y0)

    def _placer_emprise(
        self, nom: str, centre: tuple[float, float], rayon_min: float, rayon_max: float
    ) -> bool:
        rng = self.rng
        dx0, dy0 = self.depart()[:2]
        for _ in range(400):
            a, r = rng.uniform(0, 2 * math.pi), rng.uniform(rayon_min, rayon_max)
            x, y, lacet = (
                centre[0] + r * math.cos(a),
                centre[1] + r * math.sin(a),
                rng.uniform(0, 2 * math.pi),
            )
            ex, ey, er = self._emprise(nom, x, y, lacet)
            i, j = self._cellule(ex, ey)
            if self.d_piste[i, j] < er + 2.5 or math.hypot(ex - dx0, ey - dy0) < er + 4:
                continue
            if abs(ex) > TAILLE_M / 2 - er - 2 or abs(ey) > TAILLE_M / 2 - er - 2:
                continue
            if any(
                math.hypot(o.extra.get("ex", o.x) - ex, o.extra.get("ey", o.y) - ey)
                < o.rayon + er + 1.5
                for o in self.objets
            ):
                continue
            hauteur = EMPRISES[nom][1]
            self.objets.append(
                Objet(nom, x, y, self.h(ex, ey), lacet, 1, er, hauteur, {"ex": ex, "ey": ey})
            )
            return True
        return False

    def _hameau(self) -> None:
        rng = self.rng
        hx, hy = self.hameau
        for nom in ("maison1", "maison2", "maison3"):
            self._placer_emprise(nom, (hx, hy), 8, 17)
        self._placer_emprise("voiture", (hx, hy), 3, 12)
        self._placer_emprise("pickup", (hx, hy), 3, 12)
        z = self.h(hx, hy)
        # Rangée de fûts de chantier, chacun posé sur le sol local
        for _ in range(200):
            fx, fy = hx + rng.uniform(-15, 15), hy + rng.uniform(-15, 15)
            angle = rng.uniform(0, math.pi)
            if self._libre_hameau(fx, fy, 1.8):
                for k in range(-2, 3):
                    bx, by = fx + 0.7 * k * math.cos(angle), fy + 0.7 * k * math.sin(angle)
                    self.objets.append(Objet("fut", bx, by, self.h(bx, by), 0, 1, 0.3, 1.0))
                break
        for _ in range(2):
            for _ in range(60):
                px, py = hx + rng.uniform(-9, 9), hy + rng.uniform(-9, 9)
                if self._libre_hameau(px, py, 0.6):
                    self.objets.append(
                        Objet("personne", px, py, z, rng.uniform(0, 6.28), 1, 0.3, 1.8)
                    )
                    break
        # Clôture d'un potager (poteaux et deux lisses), entrée ouverte
        for _ in range(100):
            x0, y0 = hx + rng.uniform(-24, 10), hy + rng.uniform(-24, 10)
            cote = 9.0
            if not self._libre_hameau(x0 + cote / 2, y0 + cote / 2, cote * 0.72):
                continue
            coins = [(x0, y0), (x0 + cote, y0), (x0 + cote, y0 + cote), (x0, y0 + cote)]
            for (xa, ya), (xb, yb) in list(zip(coins, coins[1:] + coins[:1], strict=True))[:3]:
                mx, my = 0.5 * (xa + xb), 0.5 * (ya + yb)
                self.objets.append(
                    Objet(
                        "cloture",
                        mx,
                        my,
                        self.h(mx, my),
                        math.atan2(yb - ya, xb - xa),
                        1,
                        cote / 2,
                        1.1,
                        {"longueur": cote},
                    )
                )
            break
        # Ruine en forêt
        for _ in range(300):
            x, y = rng.uniform(-60, 10), rng.uniform(10, 60)
            if self._zone_foret(x, y) > 0.5 and self._placer_ruine(x, y):
                break

    def _libre_hameau(self, x: float, y: float, rayon: float) -> bool:
        i, j = self._cellule(x, y)
        if self.d_piste[i, j] < rayon + 2.0:
            return False
        dx0, dy0 = self.depart()[:2]
        if math.hypot(x - dx0, y - dy0) < rayon + 3:
            return False
        return all(
            math.hypot(o.extra.get("ex", o.x) - x, o.extra.get("ey", o.y) - y)
            > o.rayon + rayon + 0.8
            for o in self.objets
        )

    def _placer_ruine(self, x: float, y: float) -> bool:
        lacet = self.rng.uniform(0, 2 * math.pi)
        ex, ey, er = self._emprise("ruine", x, y, lacet)
        if not self._libre(ex, ey, er):
            return False
        self.objets.append(
            Objet("ruine", x, y, self.h(ex, ey), lacet, 1, er, 5.8, {"ex": ex, "ey": ey})
        )
        return True

    def depart(self) -> tuple[float, float, float, float]:
        x, y = self.piste[3]
        dx, dy = self.piste[6] - self.piste[3]
        return float(x), float(y), self.h(float(x), float(y)) + 0.25, math.atan2(dy, dx)

    # ------------------------------------------------------------------ export
    def texture(self) -> Image.Image:
        rng = np.random.default_rng(self.graine + 1)
        n = TEXTURE_PX
        g = np.linspace(-TAILLE_M / 2, TAILLE_M / 2, n)
        X, Y = np.meshgrid(g, g)
        zoom = n / self.n
        pente = ndimage.zoom(self.pente_deg, zoom, order=1)[:n, :n]
        d_piste = ndimage.zoom(self.d_piste, zoom, order=1)[:n, :n]
        d_r = ndimage.zoom(self.d_ruisseau, zoom, order=1)[:n, :n]
        grain = _bruit(rng, n, 30) * 0.6 + _bruit(rng, n, 120) * 0.4
        fin = rng.uniform(-1, 1, (n, n))
        herbe = np.array([0.36, 0.47, 0.20])
        couleur = herbe * (1 + 0.14 * grain[..., None])
        # sous-bois sous les arbres
        couvert = np.zeros((n, n))
        for o in self.objets:
            if o.type in ("pin", "chene"):
                i = int((o.y + TAILLE_M / 2) / TAILLE_M * (n - 1))
                j = int((o.x + TAILLE_M / 2) / TAILLE_M * (n - 1))
                couvert[i, j] += 1
        couvert = np.clip(ndimage.gaussian_filter(couvert, 2.5 * n / TAILLE_M) * 60, 0, 1)
        sous_bois = np.array([0.30, 0.27, 0.17])
        couleur = (
            couleur * (1 - couvert[..., None])
            + sous_bois * (1 + 0.1 * grain[..., None]) * couvert[..., None]
        )
        # roche sur les fortes pentes
        roche = np.clip((pente - 24) / 10, 0, 1)[..., None]
        couleur = (
            couleur * (1 - roche)
            + np.array([0.48, 0.46, 0.42]) * (1 + 0.15 * grain[..., None]) * roche
        )
        # vase et galets dans le lit du ruisseau
        lit = np.clip((2.2 - d_r) / 1.2, 0, 1)[..., None]
        couleur = (
            couleur * (1 - lit) + np.array([0.36, 0.32, 0.25]) * (1 + 0.2 * grain[..., None]) * lit
        )
        # piste de terre, ornières
        piste = np.clip((1.4 + 0.25 * grain - d_piste) / 0.5, 0, 1)[..., None]
        ornieres = np.clip(1 - np.abs(np.abs(d_piste) - 0.55) / 0.12, 0, 1)[..., None] * piste
        terre = np.array([0.52, 0.42, 0.29]) * (1 + 0.1 * grain[..., None])
        couleur = couleur * (1 - piste) + terre * piste
        couleur = couleur * (1 - 0.25 * ornieres)
        # gravier du hameau
        d_h = np.hypot(X - self.hameau[0], Y - self.hameau[1])
        gravier = np.clip((14 - d_h) / 3, 0, 1)[..., None] * (1 - piste)
        couleur = (
            couleur * (1 - gravier)
            + np.array([0.55, 0.53, 0.48]) * (1 + 0.1 * fin[..., None]) * gravier
        )
        couleur *= 1 + 0.05 * fin[..., None]
        image = np.clip(couleur * 255, 0, 255).astype(np.uint8)
        # ligne 0 de l'image = y max (nord) ; les coordonnées de texture du maillage suivent.
        return Image.fromarray(image[::-1], "RGB")

    def ecrire_obj_terrain(self, chemin: Path) -> None:
        X, Y, Z = self.X, self.Y, self.z
        gy, gx = np.gradient(Z, PAS_M)
        normales = np.dstack([-gx, -gy, np.ones_like(Z)])
        normales /= np.linalg.norm(normales, axis=2, keepdims=True)
        n = self.n
        lignes = [f"mtllib {chemin.stem}.mtl", "usemtl sol"]
        lignes += [
            f"v {x:.2f} {y:.2f} {z:.3f}"
            for x, y, z in zip(X.ravel(), Y.ravel(), Z.ravel(), strict=True)
        ]
        lignes += [
            f"vt {(x + TAILLE_M / 2) / TAILLE_M:.5f} {(y + TAILLE_M / 2) / TAILLE_M:.5f}"
            for x, y in zip(X.ravel(), Y.ravel(), strict=True)
        ]
        lignes += [f"vn {a:.3f} {b:.3f} {c:.3f}" for a, b, c in normales.reshape(-1, 3)]
        for i in range(n - 1):
            for j in range(n - 1):
                a = i * n + j + 1
                b, c, d = a + 1, a + n, a + n + 1
                lignes.append(f"f {a}/{a}/{a} {b}/{b}/{b} {d}/{d}/{d}")
                lignes.append(f"f {a}/{a}/{a} {d}/{d}/{d} {c}/{c}/{c}")
        chemin.write_text("\n".join(lignes) + "\n")
        (chemin.parent / f"{chemin.stem}.mtl").write_text(
            "newmtl sol\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd sol.png\n"
        )

    def ecrire_obj_eau(self, chemin: Path) -> None:
        """Ruban d'eau le long du ruisseau, au niveau de l'eau (visuel seulement)."""
        r = self.ruisseau
        lignes = [f"mtllib {chemin.stem}.mtl", "usemtl eau"]
        sommets = []
        for k in range(len(r)):
            a = r[max(k - 1, 0)]
            b = r[min(k + 1, len(r) - 1)]
            t = (b - a) / (np.linalg.norm(b - a) + 1e-9)
            nrm = np.array([-t[1], t[0]])
            fond = min(self.h(*(r[k] + d * nrm)) for d in (-0.6, 0.0, 0.6))
            # Au gué, l'eau s'étale sur un haut-fond : 6 cm, franchissable par le rover.
            d_gue = math.hypot(r[k][0] - self.gue[0], r[k][1] - self.gue[1])
            hauteur = self.niveau_eau_relatif + (0.06 - self.niveau_eau_relatif) * min(
                max((6 - d_gue) / 4, 0.0), 1.0
            )
            z = fond + hauteur
            for d in (-1.3, 1.3):
                p = r[k] + d * nrm
                sommets.append((p[0], p[1], z))
        lignes += [f"v {x:.2f} {y:.2f} {z:.3f}" for x, y, z in sommets]
        lignes.append("vn 0 0 1")
        for k in range(len(r) - 1):
            a, b, c, d = 2 * k + 1, 2 * k + 2, 2 * k + 3, 2 * k + 4
            lignes.append(f"f {a}//1 {b}//1 {d}//1")
            lignes.append(f"f {a}//1 {d}//1 {c}//1")
        chemin.write_text("\n".join(lignes) + "\n")
        (chemin.parent / f"{chemin.stem}.mtl").write_text("newmtl eau\nKd 0.25 0.38 0.45\nd 0.75\n")

    def description(self) -> dict[str, object]:
        x, y, z, lacet = self.depart()
        return {
            "graine": self.graine,
            "taille_m": TAILLE_M,
            "pas_m": PAS_M,
            "depart": {"x": x, "y": y, "z": z, "lacet": lacet},
            "hameau": self.hameau,
            "clairiere": self.clairiere,
            "butte": self.butte,
            "gue": self.gue,
            "piste": np.round(self.piste[::4], 2).tolist(),
            "ruisseau": np.round(self.ruisseau[::4], 2).tolist(),
            "objets": [asdict(o) for o in self.objets],
        }


# ----------------------------------------------------------------------------- SDF


def _modele_visuel_collision(nom: str, o: Objet, visuel: str, collision: str) -> str:
    return (
        f'<model name="{nom}"><static>true</static><pose>{o.x:.3f} {o.y:.3f} {o.z:.3f} 0 0 {o.lacet:.3f}</pose>'
        f'<link name="l">{visuel}{collision}</link></model>'
    )


def _mesh(uri: str, echelle: float) -> str:
    return f"<mesh><uri>{uri}</uri><scale>{echelle} {echelle} {echelle}</scale></mesh>"


def sdf_objet(k: int, o: Objet) -> str:
    nom = f"{o.type}_{k}"
    e = o.echelle
    if o.type in ("pin", "chene"):
        base = f"{FUEL}/pine%20tree/6/files" if o.type == "pin" else f"{FUEL}/oak%20tree/7/files"
        maillage = f"{base}/meshes/{'pine_tree' if o.type == 'pin' else 'oak_tree'}.dae"
        branches = "branch_2_diffuse.png" if o.type == "pin" else "branch_diffuse.png"
        vis = "".join(
            f'<visual name="{nom}"><geometry><mesh><uri>{maillage}</uri><submesh><name>{sous}</name></submesh>'
            f"<scale>{e} {e} {e}</scale></mesh></geometry><material>{double}<diffuse>1 1 1</diffuse>"
            f"<pbr><metal><albedo_map>{base}/materials/textures/{tex}</albedo_map></metal></pbr></material></visual>"
            for nom, sous, tex, double in (
                ("branches", "Branch", branches, "<double_sided>true</double_sided>"),
                ("ecorce", "Bark", "bark_diffuse.png", ""),
            )
        )
        haut = 3.0
        col = (
            f'<collision name="c"><pose>0 0 {haut / 2} 0 0 0</pose><geometry><cylinder><radius>{o.rayon:.3f}</radius>'
            f"<length>{haut}</length></cylinder></geometry></collision>"
        )
        return _modele_visuel_collision(nom, o, vis, col)
    if o.type.startswith("rocher"):
        num = o.type[-1]
        base = f"{FUEL}/falling%20rock%20{num}/6/files"
        tex = f"{base}/materials/textures/FallingRocks_Albedo.jpg"
        vis = (
            f'<visual name="v"><geometry>{_mesh(f"{base}/meshes/FallingRock0{num}.dae", 0.01 * e)}</geometry>'
            f"<material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{tex}</albedo_map>"
            f"<roughness>0.9</roughness></metal></pbr></material></visual>"
        )
        col = f'<collision name="c"><geometry>{_mesh(f"{base}/meshes/FallingRock0{num}_Collider.dae", 0.01 * e)}</geometry></collision>'
        return _modele_visuel_collision(nom, o, vis, col)
    if o.type == "tronc":
        L, r, tangage = o.extra["longueur"], o.extra["rayon"], o.extra["tangage"]
        geo = f"<cylinder><radius>{r:.3f}</radius><length>{L:.3f}</length></cylinder>"
        return (
            f'<model name="{nom}"><static>true</static><pose>{o.x:.3f} {o.y:.3f} {o.z:.3f} 0 {1.5708 + tangage:.4f} {o.lacet:.4f}</pose>'
            f'<link name="l"><visual name="v"><geometry>{geo}</geometry><material><diffuse>0.33 0.24 0.16 1</diffuse>'
            f'<ambient>0.33 0.24 0.16 1</ambient></material></visual><collision name="c"><geometry>{geo}</geometry></collision></link></model>'
        )
    if o.type == "souche":
        geo = f"<cylinder><radius>{o.rayon:.3f}</radius><length>{o.hauteur:.3f}</length></cylinder>"
        return (
            f'<model name="{nom}"><static>true</static><pose>{o.x:.3f} {o.y:.3f} {o.z + o.hauteur / 2 - 0.05:.3f} 0 0 0</pose>'
            f'<link name="l"><visual name="v"><geometry>{geo}</geometry><material><diffuse>0.38 0.28 0.18 1</diffuse>'
            f'<ambient>0.38 0.28 0.18 1</ambient></material></visual><collision name="c"><geometry>{geo}</geometry></collision></link></model>'
        )
    if o.type == "buisson":
        r, h = o.rayon, o.hauteur
        geo = f"<ellipsoid><radii>{r:.2f} {r * 0.85:.2f} {h / 2:.2f}</radii></ellipsoid>"
        return (
            f'<model name="{nom}"><static>true</static><pose>{o.x:.3f} {o.y:.3f} {o.z + h * 0.4:.3f} 0 0 {o.lacet:.3f}</pose>'
            f'<link name="l"><visual name="v"><geometry>{geo}</geometry><material><diffuse>0.18 0.34 0.14 1</diffuse>'
            f'<ambient>0.18 0.34 0.14 1</ambient></material></visual><collision name="c"><geometry>{geo}</geometry></collision></link></model>'
        )
    if o.type == "cloture":
        L = o.extra["longueur"]
        poteaux = "".join(
            f'<visual name="p{k}"><pose>{-L / 2 + k * L / 5:.2f} 0 0.55 0 0 0</pose><geometry><box><size>0.1 0.1 1.1</size></box></geometry>'
            f"<material><diffuse>0.45 0.35 0.22 1</diffuse></material></visual>"
            f'<collision name="cp{k}"><pose>{-L / 2 + k * L / 5:.2f} 0 0.55 0 0 0</pose><geometry><box><size>0.1 0.1 1.1</size></box></geometry></collision>'
            for k in range(6)
        )
        lisses = "".join(
            f'<visual name="l{k}"><pose>0 0 {z} 0 0 0</pose><geometry><box><size>{L} 0.04 0.1</size></box></geometry>'
            f"<material><diffuse>0.5 0.4 0.26 1</diffuse></material></visual>"
            f'<collision name="cl{k}"><pose>0 0 {z} 0 0 0</pose><geometry><box><size>{L} 0.04 0.1</size></box></geometry></collision>'
            for k, z in enumerate((0.45, 0.9))
        )
        return (
            f'<model name="{nom}"><static>true</static><pose>{o.x:.3f} {o.y:.3f} {o.z:.3f} 0 0 {o.lacet:.3f}</pose>'
            f'<link name="l">{poteaux}{lisses}</link></model>'
        )
    # Bâtiments : les modèles Fuel décrivent leurs matériaux en scripts OGRE 1, ignorés par le
    # rendu ogre2 (maisons noires) ; on reprend leur maillage avec la texture en PBR.
    batiments = {
        "maison1": ("house%201/3", "house_1.dae", "House_1_Diffuse.png"),
        "maison2": ("house%202/3", "house_2.dae", "House_2_Diffuse.png"),
        "maison3": ("house%203/3", "house_3.dae", "House_3_Diffuse.png"),
        "ruine": ("collapsed%20house/4", "collapsed_house.dae", "collapsed_house_diffuse.jpg"),
    }
    if o.type in batiments:
        chemin, maillage, texture = batiments[o.type]
        base = f"{FUEL}/{chemin}/files"
        geo = _mesh(f"{base}/meshes/{maillage}", 1.5)
        vis = (
            f'<visual name="v"><geometry>{geo}</geometry><material><diffuse>1 1 1 1</diffuse>'
            f"<pbr><metal><albedo_map>{base}/materials/textures/{texture}</albedo_map>"
            f"<roughness>0.85</roughness></metal></pbr></material></visual>"
        )
        col = f'<collision name="c"><geometry>{geo}</geometry></collision>'
        return _modele_visuel_collision(nom, o, vis, col)
    inclusions = {
        "maison1": "House 1",
        "maison2": "House 2",
        "maison3": "House 3",
        "ruine": "Collapsed House",
        "voiture": "Hatchback",
        "pickup": "Pickup",
        "fut": "Construction Barrel",
        "personne": "Standing person",
    }
    modele = inclusions[o.type].replace(" ", "%20")
    return (
        f"<include><name>{nom}</name><uri>{FUEL}/{modele}</uri>"
        f"<pose>{o.x:.3f} {o.y:.3f} {o.z:.3f} 0 0 {o.lacet:.3f}</pose><static>true</static></include>"
    )


def camera_spectateur(x: float, y: float, z: float, lacet: float, tangage: float) -> str:
    """Caméra fixe pour voir la scène de l'extérieur (captures, documentation) ; coûte du rendu."""
    return (
        f'<model name="spectateur"><static>true</static><pose>{x:.2f} {y:.2f} {z:.2f} 0 {tangage:.3f} {lacet:.3f}</pose>'
        '<link name="l"><sensor name="vue" type="camera"><update_rate>1</update_rate><topic>/spectateur/image</topic>'
        "<camera><horizontal_fov>1.3</horizontal_fov><image><width>1280</width><height>720</height></image>"
        "<clip><near>0.1</near><far>300</far></clip></camera></sensor></link></model>"
    )


def ecrire_monde(graine: int, dossier: Path, spectateur: bool = False) -> Path:
    dossier.mkdir(parents=True, exist_ok=True)
    m = Monde(graine)
    m.placer()
    m.texture().save(dossier / "sol.png", optimize=True)
    m.ecrire_obj_terrain(dossier / "sol.obj")
    m.ecrire_obj_eau(dossier / "eau.obj")
    (dossier / "description.json").write_text(json.dumps(m.description(), ensure_ascii=False))
    objets = "\n".join(sdf_objet(k, o) for k, o in enumerate(m.objets))
    if spectateur:
        x, y, z, lacet = m.depart()
        objets += "\n" + camera_spectateur(
            x - 9 * math.cos(lacet), y - 9 * math.sin(lacet), z + 4.5, lacet, 0.33
        )
    sol = str((dossier / "sol.obj").resolve())
    eau = str((dossier / "eau.obj").resolve())
    sdf = f"""<?xml version="1.0"?>
<!-- Monde FieldMind généré (graine {graine}). Modèles Gazebo Fuel : OpenRobotics, CC0 et CC-BY 4.0. -->
<sdf version="1.10">
  <world name="fieldmind_{graine}">
    <physics name="4ms" type="dart"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor>
      <dart><collision_detector>bullet</collision_detector></dart></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>48.40</latitude_deg><longitude_deg>-4.50</longitude_deg><elevation>60</elevation><heading_deg>0</heading_deg>
    </spherical_coordinates>
    <scene><ambient>0.45 0.45 0.45 1</ambient><background>0.70 0.80 0.90 1</background><shadows>true</shadows>
      <sky/></scene>
    <light type="directional" name="soleil"><cast_shadows>true</cast_shadows><pose>0 0 50 0 0 0</pose>
      <direction>-0.45 0.3 -0.8</direction><diffuse>0.95 0.9 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular></light>
    <model name="sol"><static>true</static><link name="l">
      <collision name="c"><geometry><mesh><uri>{sol}</uri></mesh></geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><mesh><uri>{sol}</uri></mesh></geometry></visual>
    </link></model>
    <model name="eau"><static>true</static><link name="l">
      <visual name="v"><cast_shadows>false</cast_shadows><geometry><mesh><uri>{eau}</uri></mesh></geometry>
        <material><diffuse>0.25 0.38 0.45 0.75</diffuse><ambient>0.2 0.3 0.35 1</ambient><specular>0.6 0.6 0.6 1</specular></material>
        <transparency>0.3</transparency></visual>
    </link></model>
{objets}
  </world>
</sdf>
"""
    chemin = dossier / "monde.sdf"
    chemin.write_text(sdf)
    return chemin


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--graine", type=int, default=7)
    p.add_argument("--sortie", type=Path, default=Path.home() / ".cache" / "fieldmind" / "mondes")
    p.add_argument("--spectateur", action="store_true", help="caméra fixe derrière le départ")
    a = p.parse_args()
    dossier = a.sortie / str(a.graine) if a.sortie.name != str(a.graine) else a.sortie
    chemin = ecrire_monde(a.graine, dossier, a.spectateur)
    d = json.loads((dossier / "description.json").read_text())
    comptes: dict[str, int] = {}
    for o in d["objets"]:
        comptes[o["type"]] = comptes.get(o["type"], 0) + 1
    print(chemin)
    print("départ", d["depart"], comptes)


if __name__ == "__main__":
    main()
