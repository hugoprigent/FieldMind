"""Carte des hauteurs du terrain et traversabilité, à partir des nuages de la caméra stéréo.

Grille de la zone de mission (même géométrie que la grille d'occupation du lidar). Chaque cellule
garde, pour la dernière vue qui l'a couverte, la hauteur la plus basse (le sol) et la plus haute
jusqu'à `hauteur_max` au-dessus de ce sol : ce qui dépasse (branches, houppiers) ne gêne pas le
rover et n'entre pas dans le calcul.

Une cellule garde la vue la plus proche qui l'a couverte : la stéréo perd vite en précision
avec la distance, une vue lointaine ne remplace donc pas une vue proche (sauf si elle est
ancienne). Le sol et le haut d'une cellule sont ses deuxièmes plus basse et plus haute valeurs :
le minimum et le maximum de quelques points bruités s'écartent d'autant plus que la mesure est
lointaine, et deux cellules de sol plat paraîtraient alors séparées d'une marche.

Le bruit de hauteur de la stéréo croît comme le carré de la distance (σ ≈ 0,4 cm à 1 m, 1,6 cm à
2 m, 3,6 cm à 3 m, mesuré sur sol plat) : le seuil de marche est relevé de deux σ à la distance
de la vue.

Une cellule est classée :
- obstacle « marche » si elle dépasse de plus de `pas_max` le sol voisin (tronc couché, rocher,
  souche, pied d'arbre) et descend à moins de `garde_haute` de ce sol (le rover la heurterait) ;
- obstacle « pente » si la pente du sol dépasse `pente_max_deg` (flanc de butte, berge) ;
- libre sinon, inconnue si aucune vue ne l'a couverte.

Trou (obstacle « négatif » : fossé, berge qui plonge) : la caméra ne voit pas la paroi qui
descend devant elle. Dans l'image de profondeur, un bord de sol suivi, sans transition, d'une
surface bien plus lointaine marque une zone cachée (`ProfondeurStereo.bords_occultation`) :
c'est un trou probable (`signaler_trous`), jusqu'à ce qu'une vue la montre.

Seules les cellules vues à moins de `portee_fiable` peuvent devenir des obstacles, et seulement
après deux vues successives qui les classent ainsi (`confirmations`) : une erreur de mise en
correspondance sur une rangée de l'image fait, au sol, un arc de « bosse » autour de la caméra,
qui ne se reproduit pas à l'image suivante. Le lidar procède de même (log-odds).

Adapté de la pile ROS 2 (`ros2/src/fieldmind_perception/elevation.py`) : même classement, sur
une grille fixe de la zone plutôt que glissante, et reclassée seulement autour du rover.

Altitude : le rover ne connaît son altitude qu'à une dérive près (intégration du tangage). Avant
d'insérer un nuage, on le recale en hauteur sur le sol déjà cartographié (médiane des écarts
sur les cellules communes) ; seules des hauteurs relatives cohérentes comptent.
"""

from __future__ import annotations

import base64
import math
import zlib

import numpy as np
from scipy import ndimage

from services.protocole.scenario import Limites

INCONNU, LIBRE, MARCHE, PENTE, TROU = 0, 1, 2, 3, 4


class CarteHauteurs:
    def __init__(
        self,
        limites: Limites,
        resolution: float = 0.1,
        pas_max: float = 0.15,
        pente_max_deg: float = 22.0,
        hauteur_max: float = 1.2,
        garde_haute: float = 0.7,
        points_min: int = 2,
        recalage_min: int = 40,
        recalage_max: float = 0.05,
        tolerance_vue: float = 0.5,
        oubli_s: float = 30.0,
        portee_fiable: float = 4.0,
        portee_pente: float = 3.0,
        bruit_quadratique: float = 0.004,
    ) -> None:
        self.resolution = resolution
        self.origine = (limites.xmin, limites.ymin)
        self.largeur = math.ceil((limites.xmax - limites.xmin) / resolution)
        self.hauteur = math.ceil((limites.ymax - limites.ymin) / resolution)
        self.pas_max = pas_max
        self.pente_max = pente_max_deg
        self.hauteur_max = hauteur_max
        self.garde_haute = garde_haute
        self.points_min = points_min
        self.recalage_min = recalage_min
        self.recalage_max = recalage_max
        self.tolerance_vue = tolerance_vue
        self.oubli_s = oubli_s
        self.portee_fiable = portee_fiable
        self.portee_pente = portee_pente
        self.bruit_quadratique = bruit_quadratique
        forme = (self.hauteur, self.largeur)
        self.zmin = np.full(forme, np.nan, dtype=np.float32)
        self.zmax = np.full(forme, np.nan, dtype=np.float32)
        self.vue = np.zeros(forme, dtype=bool)
        self.distance_vue = np.full(forme, np.inf, dtype=np.float32)
        self.t_vue = np.full(forme, -np.inf, dtype=np.float32)
        self.etat = np.zeros(forme, dtype=np.uint8)
        self.confirmations = np.zeros(forme, dtype=np.uint8)
        self.trous = np.zeros(forme, dtype=np.uint8)
        """Nombre de vues successives où la cellule était cachée derrière un bord de sol."""
        self.nuages = 0
        self._t_dernier_nuage = -np.inf
        self._sale: list[int] | None = None

    # ------------------------------------------------------------------ coordonnées
    def indices(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        c = np.floor((np.asarray(x) - self.origine[0]) / self.resolution).astype(np.int64)
        lig = np.floor((np.asarray(y) - self.origine[1]) / self.resolution).astype(np.int64)
        dedans = (c >= 0) & (c < self.largeur) & (lig >= 0) & (lig < self.hauteur)
        return lig, c, dedans

    # ------------------------------------------------------------------ observations
    def ajouter_nuage(
        self, points: np.ndarray, distances: np.ndarray, t: float, recaler: bool = True
    ) -> float:
        """Points (N, 3) dans le repère du monde et leur distance à la caméra (N,). Renvoie le
        décalage de hauteur appliqué au nuage (m)."""
        lig, c, ok = self.indices(points[:, 0], points[:, 1])
        if not ok.any():
            return 0.0
        z = points[ok, 2].astype(np.float64)
        dist = distances[ok]
        k = lig[ok] * self.largeur + c[ok]
        # Travail sur les seules cellules touchées : indices compacts.
        cellules, inverse = np.unique(k, return_inverse=True)
        n = cellules.size
        dmin = np.full(n, np.inf)
        np.minimum.at(dmin, inverse, dist)
        ordre = np.lexsort((z, inverse))
        zs, iz = z[ordre], inverse[ordre]
        groupes = np.arange(n)
        debuts = np.searchsorted(iz, groupes, side="left")
        fins = np.searchsorted(iz, groupes, side="right") - 1
        nombre = fins - debuts + 1
        garde = nombre >= self.points_min
        # Sol : deuxième plus basse valeur.
        fmin = zs[np.minimum(debuts + 1, fins)]
        decalage = 0.0
        if recaler:
            commun = garde & self.vue.reshape(-1)[cellules]
            if commun.sum() >= self.recalage_min:
                anciens = self.zmin.reshape(-1)[cellules[commun]]
                decalage = float(np.median(fmin[commun] - anciens))
                decalage = max(-self.recalage_max, min(self.recalage_max, decalage))
                zs = zs - decalage
                fmin = fmin - decalage
        # Haut : deuxième plus haute valeur sous `hauteur_max` au-dessus du sol (les points
        # sont triés par cellule puis par hauteur : on cherche la fin de la tranche utile).
        limite = fmin + self.hauteur_max
        cle = iz.astype(np.float64) * 1e4 + (zs - limite[iz]).clip(-1e3, 1e3)
        hauts = np.searchsorted(cle, groupes * 1e4, side="left") - 1
        hauts = np.clip(hauts, debuts, fins)
        fmax = zs[np.maximum(hauts - 1, debuts)]
        # Remplacement : vue au moins aussi proche (à la tolérance près), ou ancienne.
        plat_d = self.distance_vue.reshape(-1)
        plat_t = self.t_vue.reshape(-1)
        meilleure = (dmin <= plat_d[cellules] + self.tolerance_vue) | (
            plat_t[cellules] < t - self.oubli_s
        )
        retenues = garde & meilleure
        cibles = cellules[retenues]
        self.zmin.reshape(-1)[cibles] = fmin[retenues]
        self.zmax.reshape(-1)[cibles] = fmax[retenues]
        self.vue.reshape(-1)[cibles] = True
        plat_d[cibles] = dmin[retenues]
        plat_t[cibles] = t
        self.trous.reshape(-1)[cellules[garde]] = 0
        self._t_dernier_nuage = t
        self.nuages += 1
        return decalage

    # ------------------------------------------------------------------ traversabilité
    def classer(self, x: float, y: float, rayon: float = 10.0) -> None:
        """Reclasse les cellules à moins de `rayon` du point (x, y) (fenêtre carrée)."""
        r = math.ceil(rayon / self.resolution)
        lig0, c0, _ = self.indices(np.array(x), np.array(y))
        l0, l1 = max(int(lig0) - r, 0), min(int(lig0) + r + 1, self.hauteur)
        k0, k1 = max(int(c0) - r, 0), min(int(c0) + r + 1, self.largeur)
        if l0 >= l1 or k0 >= k1:
            return
        # Marge de 3 cellules pour les filtres, retirée à l'écriture.
        m = 5
        a0, a1 = max(l0 - m, 0), min(l1 + m, self.hauteur)
        b0, b1 = max(k0 - m, 0), min(k1 + m, self.largeur)
        vue = self.vue[a0:a1, b0:b1]
        # Au-delà de la portée fiable, l'erreur de distance (±35 cm à 5 m) étale les objets sur
        # le sol voisin : ces cellules peuvent être libres, jamais obstacles. L'obstacle se
        # confirme quand le rover approche.
        distance = self.distance_vue[a0:a1, b0:b1].astype(np.float64)
        fiable = distance <= self.portee_fiable
        seuil = self.pas_max + 2 * self.bruit_quadratique * np.minimum(distance, 10.0) ** 2
        zmin = self.zmin[a0:a1, b0:b1].astype(np.float64)
        zmax = self.zmax[a0:a1, b0:b1].astype(np.float64)
        sol = np.where(vue, zmin, np.nan)
        # Sol de référence : le plus bas des cellules voisines (3 × 3) vues de près. Une voisine
        # vue de loin a un sol trop bruité (±10 cm à 6 m) : elle créerait une fausse marche à la
        # frontière entre zones vues de près et de loin.
        ref = ndimage.minimum_filter(np.where(vue & fiable, sol, np.inf), size=3, mode="nearest")
        ref = np.where(np.isfinite(ref), ref, sol)
        with np.errstate(invalid="ignore"):
            # Cellule vue seulement en hauteur (houppier, branche) : ni sol, ni obstacle.
            surplomb = vue & (zmin - ref > self.garde_haute)
            marche = vue & fiable & ~surplomb & (zmax - ref > seuil)
        # Pente calculée sur les seules cellules vues de près : mêler au lissage des cellules
        # lointaines (plus bruitées) crée de fausses pentes à la frontière.
        proche = distance <= self.portee_pente
        sol_seul = np.where(surplomb | ~proche, np.nan, sol)
        pente = _pente_deg(sol_seul, self.resolution)
        with np.errstate(invalid="ignore"):
            raide = proche & (pente > self.pente_max)
        # Un vrai obstacle (tronc couché de 25 cm, pied d'arbre, rocher) couvre plusieurs
        # cellules voisines : une cellule isolée est du bruit de mesure.
        voisins = ndimage.uniform_filter(marche.astype(np.float32), size=3, mode="constant") * 9
        marche &= voisins >= 2.5
        voisins = ndimage.uniform_filter(raide.astype(np.float32), size=3, mode="constant") * 9
        raide &= voisins >= 2.5
        # Confirmation : une cellule mesurée par le dernier nuage compte une vue de plus si elle
        # y est un obstacle, repart de zéro sinon ; une cellule non remesurée garde son compte.
        interieur = (slice(l0 - a0, l1 - a0), slice(k0 - b0, k1 - b0))
        obstacle = (marche | raide)[interieur]
        mesuree = self.t_vue[l0:l1, k0:k1] >= self._t_dernier_nuage
        compte = self.confirmations[l0:l1, k0:k1]
        compte[mesuree & obstacle] = np.minimum(compte[mesuree & obstacle] + 1, 3)
        compte[mesuree & ~obstacle] = 0
        confirme = compte >= 2
        etat = np.full(obstacle.shape, INCONNU, np.uint8)
        etat[(vue & ~surplomb)[interieur]] = LIBRE
        etat[raide[interieur] & confirme] = PENTE
        etat[marche[interieur] & confirme] = MARCHE
        trou = (self.trous[a0:a1, b0:b1] >= 2) & ~(vue & fiable)
        voisins = ndimage.uniform_filter(trou.astype(np.float32), size=3, mode="constant") * 9
        trou &= voisins >= 3.5
        etat[trou[interieur]] = TROU
        self.etat[l0:l1, k0:k1] = etat
        self._marquer_sale(k0, l0, k1, l1)

    def signaler_trous(self, proches: np.ndarray, lointains: np.ndarray, t: float) -> int:
        """Bords d'occultation de la stéréo (repère du monde, (M, 2) ou (M, 3)) : la zone entre
        le bord de sol proche et le point lointain est cachée ; on en signale au plus 1,5 m.
        À appeler après `ajouter_nuage` du même instant. Renvoie le nombre de cellules."""
        if len(proches) == 0:
            return 0
        d = lointains[:, :2] - proches[:, :2]
        longueur = np.hypot(d[:, 0], d[:, 1])
        u = d / np.maximum(longueur, 1e-6)[:, None]
        s = 0.15 + 0.1 * np.arange(15)
        dedans = s[None, :] < np.minimum(longueur - 0.05, 1.5)[:, None]
        xs = proches[:, 0, None] + u[:, 0, None] * s[None, :]
        ys = proches[:, 1, None] + u[:, 1, None] * s[None, :]
        lig, c, ok = self.indices(xs[dedans], ys[dedans])
        k = np.unique(lig[ok] * self.largeur + c[ok])
        # Une cellule mesurée par ce même nuage n'est pas cachée.
        k = k[self.t_vue.reshape(-1)[k] < t]
        plat = self.trous.reshape(-1)
        plat[k] = np.minimum(plat[k] + 1, 3)
        return int(k.size)

    def liberer_disque(self, x: float, y: float, rayon: float) -> None:
        """Le rover occupe cet espace : rien ne l'y bloque (la caméra ne voit pas sous lui)."""
        r = math.ceil(rayon / self.resolution)
        lig0, c0, _ = self.indices(np.array(x), np.array(y))
        l0, l1 = max(int(lig0) - r, 0), min(int(lig0) + r + 1, self.hauteur)
        k0, k1 = max(int(c0) - r, 0), min(int(c0) + r + 1, self.largeur)
        if l0 >= l1 or k0 >= k1:
            return
        ll, cc = np.mgrid[l0:l1, k0:k1]
        dedans = (ll - int(lig0)) ** 2 + (cc - int(c0)) ** 2 <= r * r
        bloc = self.etat[l0:l1, k0:k1]
        efface = dedans & (bloc >= MARCHE)
        if efface.any():
            bloc[efface] = LIBRE
            self.confirmations[l0:l1, k0:k1][efface] = 0
            self.trous[l0:l1, k0:k1][efface] = 0
            self._marquer_sale(k0, l0, k1, l1)

    def echos_au_sol(self, points: np.ndarray) -> np.ndarray:
        """Vrai pour les impacts (N, 3) posés sur le sol vu de près par la caméra : un lidar 2D
        ne distingue pas une montée d'un mur, la caméra si (sol mesuré, sans marche)."""
        lig, c, ok = self.indices(points[:, 0], points[:, 1])
        sol = np.zeros(len(points), dtype=bool)
        if not ok.any():
            return sol
        li, ci = lig[ok], c[ok]
        etat = self.etat[li, ci]
        connu = (etat == LIBRE) & (self.distance_vue[li, ci] <= self.portee_fiable)
        with np.errstate(invalid="ignore"):
            pres = np.abs(points[ok, 2] - self.zmin[li, ci]) <= self.pas_max
        sol[np.flatnonzero(ok)] = connu & pres
        return sol

    def sol_libre_proche(self, portee: float = 3.0) -> np.ndarray:
        """Cellules vues de près sans rien entre le sol et 1,2 m : rien ne peut y arrêter le
        faisceau du lidar, sinon le sol lui-même."""
        masque: np.ndarray = (self.etat == LIBRE) & (self.distance_vue <= portee)
        return masque

    def obstacles(self) -> np.ndarray:
        masque: np.ndarray = self.etat >= MARCHE
        return masque

    def distance_libre(
        self, x: float, y: float, cap: float, demi_largeur: float, depart: float, portee: float
    ) -> float:
        """Distance libre devant (x, y) dans l'axe `cap`, dans un couloir de ±`demi_largeur`,
        à partir de `depart` mètres du point ; `inf` si aucun obstacle jusqu'à `portee`."""
        pas = self.resolution / 2
        d = np.arange(depart, portee, pas)
        lat = np.linspace(-demi_largeur, demi_largeur, max(3, math.ceil(2 * demi_largeur / pas)))
        c, s = math.cos(cap), math.sin(cap)
        px = x + c * d[:, None] - s * lat[None, :]
        py = y + s * d[:, None] + c * lat[None, :]
        lig, col, ok = self.indices(px, py)
        bloque = np.zeros(px.shape, dtype=bool)
        bloque[ok] = self.etat[lig[ok], col[ok]] >= MARCHE
        lignes = np.flatnonzero(bloque.any(axis=1))
        return float(d[lignes[0]] - depart) if lignes.size else math.inf

    # ------------------------------------------------------------------ poste opérateur
    def _marquer_sale(self, c0: int, l0: int, c1: int, l1: int) -> None:
        s = self._sale
        self._sale = (
            [c0, l0, c1, l1]
            if s is None
            else [min(s[0], c0), min(s[1], l0), max(s[2], c1), max(s[3], l1)]
        )

    def extraire_modifications(
        self, complet: bool = False
    ) -> tuple[tuple[int, int, int, int], str] | None:
        """Rectangle modifié depuis le dernier appel : états (0 à 4), zlib, base64."""
        if complet:
            rect = [0, 0, self.largeur, self.hauteur]
        elif self._sale is None:
            return None
        else:
            rect = self._sale
        self._sale = None
        c0, l0, c1, l1 = rect
        donnees = zlib.compress(np.ascontiguousarray(self.etat[l0:l1, c0:c1]).tobytes(), level=6)
        return (c0, l0, c1 - c0, l1 - l0), base64.b64encode(donnees).decode("ascii")


def _pente_deg(sol: np.ndarray, res: float) -> np.ndarray:
    """Pente du sol (degrés) sur une base de 0,4 m, après lissage 5 × 5 ; NaN si inconnue."""
    connu = np.isfinite(sol)
    somme = ndimage.uniform_filter(np.where(connu, sol, 0.0), size=5, mode="constant")
    poids = ndimage.uniform_filter(connu.astype(float), size=5, mode="constant")
    with np.errstate(invalid="ignore", divide="ignore"):
        lisse = np.where(poids > 0.6, somme / poids, np.nan)
    gx = np.full(sol.shape, np.nan)
    gy = np.full(sol.shape, np.nan)
    gx[:, 2:-2] = (lisse[:, 4:] - lisse[:, :-4]) / (4 * res)
    gy[2:-2, :] = (lisse[4:, :] - lisse[:-4, :]) / (4 * res)
    pente = np.degrees(np.arctan(np.hypot(gx, gy)))
    return np.where(connu, pente, np.nan)
