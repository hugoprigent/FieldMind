"""Profondeur stéréo (SGBM) et obstacles bas, dans base_link.

Chaîne, à 4 Hz sur la paire la plus récente :

1. Décodage des deux MJPEG directement en taille réduite (`perception.stereo_largeur`, 640 px :
   le décodeur JPEG réduit d'un facteur 2 sans décoder la pleine image), en niveaux de gris.
2. Rectification si la paire est calibrée (`cameras.calibration`) ; sinon caméras supposées
   parallèles, champ par défaut (`ResultatStereo.calibree = False` : la profondeur ne vaut que ce
   que vaut le montage).
3. SGBM (Hirschmüller, OpenCV, mode 3 directions) : blocs de 5 px, contrôle gauche-droite,
   unicité, filtre des taches ; on écarte en plus les pixels de bord d'objet (disparité qui
   varie trop dans le voisinage 3 × 3 : un bloc à cheval sur l'objet et le fond donne une
   profondeur intermédiaire, posée dans le vide) et les zones sans texture (ciel, mur uni :
   gradient horizontal moyen sous `perception.stereo_texture_min`), où SGBM apparie le bruit.
4. Nuage de points (un pixel sur deux) dans base_link, redressé du roulis et du tangage si on
   les donne.
5. **Sol** : plan z = a·x + b·y + c ajusté par RANSAC sur les points bas entre 0,5 et 3 m, accepté
   s'il reste proche du plan théorique (inclinaison < 10°, hauteur < 10 cm) ; sinon plan du
   montage (z = 0). Le plan corrige un tangage de montage mal connu et l'assiette du rover.
6. **Obstacles** : points entre `obstacle_hauteur_min_m` et `obstacle_hauteur_max_m` au-dessus
   du sol, à moins de `portee_stereo_m` ; **trous** : points à plus de `trou_profondeur_m` sous
   le sol (on voit le fond d'une descente au-delà de son bord), à moins de 3 m. Regroupés en
   cellules de 5 cm (assez de points par cellule), puis **confirmés sur deux images** : une
   erreur d'appariement sur une rangée de l'image fait une fausse bosse qui ne se reproduit pas.

Précision : δZ ≈ Z²·δd / (f·B). Base 0,12 m, f ≈ 395 px à 640 px : 1/4 px vaut 2 cm à 2 m et
8 cm à 4 m. La hauteur des points, qui décide de l'obstacle, reste précise (1–3 cm).
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy.spatial import cKDTree

from fieldmind.config import Profil
from fieldmind.perception.projection import ModeleCamera, rotation_assiette
from fieldmind.types import ImageCamera, Obstacles, Odometrie, Pose2D

TAILLE_BLOC = 5


@dataclass(slots=True)
class ResultatStereo:
    t: float
    obstacles: Obstacles  # confirmés, base_link ; hauteurs > 0 obstacles, < 0 trous
    profondeur_jpeg: bytes  # profondeur en fausses couleurs (proche rouge, loin bleu)
    duree_s: float
    calibree: bool
    sol: tuple[float, float, float]  # plan z = a·x + b·y + c (base_link)
    sol_estime: bool  # False : plan théorique du montage (RANSAC refusé)
    couverture: float  # part des pixels avec une profondeur
    candidats: int  # cellules d'obstacle avant confirmation
    trous: int  # cellules de trou confirmées
    profondeur: np.ndarray | None = field(default=None, repr=False)
    """Profondeur le long de l'axe optique rectifié (m), image réduite, NaN si inconnue."""


@dataclass(slots=True)
class _Geometrie:
    largeur: int
    hauteur: int
    reduction: int  # facteur de décodage JPEG (1, 2, 4, 8)
    focale: float
    cx: float
    cy: float
    base: float
    rotation: np.ndarray  # repère rectifié gauche → base_link
    centre: np.ndarray  # centre optique gauche, base_link
    cartes: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None
    rectif_g: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None  # K, D, R, P
    rectif_d: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None
    rayons: np.ndarray  # (h', w', 3) directions base_link (z rectifié = 1), sous-échantillonnées
    taille_entree: tuple[int, int]


class Stereo:
    def __init__(self, profil: Profil) -> None:
        v = profil.valeur
        self.profil = profil
        self.largeur = int(v("perception.stereo_largeur", 640))
        self.hauteur_min = float(v("perception.obstacle_hauteur_min_m", 0.06))
        self.hauteur_max = float(v("perception.obstacle_hauteur_max_m", 0.60))
        self.portee = float(v("perception.portee_stereo_m", 4.0))
        self.portee_trou = float(v("perception.portee_trou_m", 3.0))
        self.trou_profondeur = float(v("perception.trou_profondeur_m", 0.10))
        self.pas = int(v("perception.stereo_pas_nuage", 2))
        self.cellule = float(v("perception.stereo_cellule_m", 0.05))
        self.points_min = int(v("perception.stereo_points_min", 4))
        self.tolerance_confirmation = float(v("perception.stereo_confirmation_m", 0.10))
        self.tolerance_sans_pose = float(v("perception.stereo_confirmation_sans_pose_m", 0.25))
        self.texture_min = float(v("perception.stereo_texture_min", 6.0))
        disparites = int(v("perception.stereo_disparites", 128))
        self.disparites = max(16, 16 * math.ceil(disparites * self.largeur / 640 / 16))
        self.modele_g = ModeleCamera(profil, "gauche")
        self.modele_d = ModeleCamera(profil, "droite", calibration=self.modele_g.calibration
                                     if self.modele_g.calibree else False)
        self._sgbm = cv2.StereoSGBM.create(
            minDisparity=0, numDisparities=self.disparites, blockSize=TAILLE_BLOC,
            P1=8 * TAILLE_BLOC**2, P2=32 * TAILLE_BLOC**2, disp12MaxDiff=1, preFilterCap=31,
            uniquenessRatio=15, speckleWindowSize=100, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self._geo: _Geometrie | None = None
        self._sol_lisse: np.ndarray | None = None
        self._precedents: tuple[np.ndarray, bool] | None = None  # (points, en odom)
        self._derniere: tuple[_Geometrie, np.ndarray] | None = None  # pour point_3d
        self._nuage_3d: tuple[np.ndarray, np.ndarray] | None = None  # (xy, hauteurs) hors sol
        self._verrou = threading.Lock()

    @property
    def calibree(self) -> bool:
        return self.modele_g.calibree

    # ------------------------------------------------------------------ géométrie
    def _preparer(self, largeur_entree: int, hauteur_entree: int) -> _Geometrie:
        geo = self._geo
        if geo is not None and geo.taille_entree == (largeur_entree, hauteur_entree):
            return geo
        reduction = 1
        for r in (8, 4, 2):
            if largeur_entree % r == 0 and largeur_entree // r >= self.largeur:
                reduction = r
                break
        w = self.largeur
        h = int(round(hauteur_entree * w / largeur_entree))
        cartes = rectif_g = rectif_d = None
        if self.calibree:
            cal = self.modele_g.calibration
            assert cal is not None
            k1 = self.modele_g.matrice(w, h)
            k2 = self.modele_d.matrice(w, h)
            d1 = np.asarray(cal["d1"], np.float64).reshape(-1)
            d2 = np.asarray(cal["d2"], np.float64).reshape(-1)
            r = np.asarray(cal["r"], np.float64)
            t = np.asarray(cal["t"], np.float64).reshape(3, 1)
            r1, r2, p1, p2, _q, _, _ = cv2.stereoRectify(
                k1, d1, k2, d2, (w, h), r, t, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
            g1 = cv2.initUndistortRectifyMap(k1, d1, r1, p1, (w, h), cv2.CV_16SC2)
            g2 = cv2.initUndistortRectifyMap(k2, d2, r2, p2, (w, h), cv2.CV_16SC2)
            cartes = (g1[0], g1[1], g2[0], g2[1])
            rectif_g, rectif_d = (k1, d1, r1, p1), (k2, d2, r2, p2)
            focale, cx, cy = float(p1[0, 0]), float(p1[0, 2]), float(p1[1, 2])
            base = float(np.linalg.norm(t))
            rotation = self.modele_g.rotation @ r1.T
        else:
            k = self.modele_g.matrice(w, h)
            focale, cx, cy = float(k[0, 0]), float(k[0, 2]), float(k[1, 2])
            base = self.modele_g.base
            rotation = self.modele_g.rotation
        lignes = np.arange(0, h, self.pas, dtype=np.float64)
        colonnes = np.arange(0, w, self.pas, dtype=np.float64)
        u, vv = np.meshgrid(colonnes, lignes)
        directions = np.stack([(u - cx) / focale, (vv - cy) / focale, np.ones_like(u)], axis=-1)
        rayons = directions @ rotation.T
        geo = _Geometrie(w, h, reduction, focale, cx, cy, base, rotation,
                         self.modele_g.centre.copy(), cartes, rectif_g, rectif_d,
                         rayons.astype(np.float32), (largeur_entree, hauteur_entree))
        self._geo = geo
        return geo

    def _decoder(self, image: ImageCamera, geo: _Geometrie | None) -> np.ndarray | None:
        reduction = geo.reduction if geo is not None else 1
        drapeaux = {1: cv2.IMREAD_GRAYSCALE, 2: cv2.IMREAD_REDUCED_GRAYSCALE_2,
                    4: cv2.IMREAD_REDUCED_GRAYSCALE_4, 8: cv2.IMREAD_REDUCED_GRAYSCALE_8}
        gris = cv2.imdecode(np.frombuffer(image.jpeg, np.uint8), drapeaux[reduction])
        if gris is None:
            return None
        if geo is not None and (gris.shape[1], gris.shape[0]) != (geo.largeur, geo.hauteur):
            gris = cv2.resize(gris, (geo.largeur, geo.hauteur), interpolation=cv2.INTER_AREA)
        return gris

    # ------------------------------------------------------------------ traitement
    def traiter(self, gauche: ImageCamera, droite: ImageCamera, roulis: float = 0.0,
                tangage: float = 0.0, pose: Pose2D | Odometrie | None = None
                ) -> ResultatStereo | None:
        """Obstacles de la paire. `roulis`, `tangage` (centrale) redressent le nuage ; `pose`
        (odom, à l'instant des images) rend la confirmation sur deux images insensible au
        déplacement du rover entre elles. None si la paire est illisible."""
        debut = time.perf_counter()
        largeur_entree = gauche.largeur or 0
        hauteur_entree = gauche.hauteur or 0
        if largeur_entree <= 0 or hauteur_entree <= 0:
            essai = self._decoder(gauche, None)
            if essai is None:
                return None
            hauteur_entree, largeur_entree = essai.shape[:2]
        geo = self._preparer(largeur_entree, hauteur_entree)
        g = self._decoder(gauche, geo)
        d = self._decoder(droite, geo)
        if g is None or d is None or g.shape != d.shape:
            return None
        if geo.cartes is not None:
            g = cv2.remap(g, geo.cartes[0], geo.cartes[1], cv2.INTER_LINEAR)
            d = cv2.remap(d, geo.cartes[2], geo.cartes[3], cv2.INTER_LINEAR)
        disparite = self._sgbm.compute(g, d).astype(np.float32) * (1.0 / 16.0)
        noyau = np.ones((3, 3), np.uint8)
        etendue = cv2.dilate(disparite, noyau) - cv2.erode(disparite, noyau)
        valide = (disparite > 0.5) & (etendue <= np.maximum(1.0, 0.08 * disparite))
        if self.texture_min > 0:
            # Zones sans texture (ciel, mur uni, surexposition) : SGBM y apparie le bruit du
            # capteur et invente des profondeurs.
            gradient = np.abs(cv2.Sobel(g, cv2.CV_16S, 1, 0, ksize=3)).astype(np.float32)
            valide &= cv2.boxFilter(gradient, -1, (9, 9)) >= self.texture_min
        valide[:, : self.disparites] = False  # bande sans correspondant dans l'image droite
        with np.errstate(divide="ignore"):
            z = np.where(valide, geo.focale * geo.base / disparite, np.nan).astype(np.float32)
        couverture = float(valide[:, self.disparites:].mean())
        with self._verrou:
            self._derniere = (geo, z)

        # Nuage sous-échantillonné, base_link (redressé de l'assiette).
        zs = z[:: self.pas, :: self.pas]
        ok = np.isfinite(zs) & (zs < self.portee * 1.6)
        points = geo.rayons[ok] * zs[ok, None] + geo.centre.astype(np.float32)
        if roulis or tangage:
            points = points @ rotation_assiette(roulis, tangage).T.astype(np.float32)
        sol, estime = self._sol(points)
        a, b, c = sol
        hauteurs = (points[:, 2] - (a * points[:, 0] + b * points[:, 1] + c)) / math.sqrt(
            1 + a * a + b * b)
        distance = np.hypot(points[:, 0] - geo.centre[0], points[:, 1] - geo.centre[1])
        hors_sol = (hauteurs >= self.hauteur_min) & (hauteurs <= 2.45) & (
            distance <= self.portee * 1.5)
        with self._verrou:
            self._nuage_3d = (points[hors_sol, :2].copy(), hauteurs[hors_sol].copy())
        obstacle = (hauteurs >= self.hauteur_min) & (hauteurs <= self.hauteur_max) & (
            distance <= self.portee)
        trou = (hauteurs <= -self.trou_profondeur) & (distance <= self.portee_trou)
        cand_o, haut_o = self._cellules(points[obstacle], hauteurs[obstacle], max)
        cand_t, haut_t = self._cellules(points[trou], hauteurs[trou], min)
        candidats = np.vstack([cand_o, cand_t])
        hauteurs_c = np.concatenate([haut_o, haut_t])
        confirmes = self._confirmer(candidats, pose)
        obstacles = Obstacles(t=max(gauche.t, droite.t), source="stereo",
                              points=candidats[confirmes].astype(np.float32),
                              hauteurs=hauteurs_c[confirmes].astype(np.float32),
                              portee_m=self.portee)
        jpeg = image_profondeur(z, self.portee)
        return ResultatStereo(
            t=obstacles.t, obstacles=obstacles, profondeur_jpeg=jpeg,
            duree_s=time.perf_counter() - debut, calibree=self.calibree,
            sol=(float(a), float(b), float(c)), sol_estime=estime, couverture=couverture,
            candidats=len(cand_o), trous=int(confirmes[len(cand_o):].sum()), profondeur=z)

    def _sol(self, points: np.ndarray) -> tuple[tuple[float, float, float], bool]:
        """Plan du sol par RANSAC sur les points bas ; plan théorique (0, 0, 0) en repli."""
        rng = np.random.default_rng(len(points))
        distance = points[:, 0]
        bas = (points[:, 2] < 0.25) & (distance > 0.5) & (distance < 3.0)
        candidats = points[bas]
        if len(candidats) > 3000:
            candidats = candidats[rng.choice(len(candidats), 3000, replace=False)]
        if len(candidats) < 200:
            return self._lisser(None)
        iterations = 60
        triplets = rng.integers(0, len(candidats), (iterations, 3))
        p0, p1, p2 = (candidats[triplets[:, k]] for k in range(3))
        normales = np.cross(p1 - p0, p2 - p0)
        norme = np.linalg.norm(normales, axis=1)
        utiles = norme > 1e-6
        normales = normales[utiles] / norme[utiles, None]
        p0 = p0[utiles]
        normales *= np.sign(normales[:, 2:3] + 1e-12)
        horizontales = normales[:, 2] > math.cos(math.radians(10))
        if not horizontales.any():
            return self._lisser(None)
        normales, p0 = normales[horizontales], p0[horizontales]
        ecarts = np.abs((candidats[None, :, :] - p0[:, None, :]) @ normales[:, :, None])[..., 0]
        scores = (ecarts < 0.025).sum(axis=1)
        meilleur = int(np.argmax(scores))
        dedans = ecarts[meilleur] < 0.025
        if dedans.sum() < max(150, 0.3 * len(candidats)):
            return self._lisser(None)
        q = candidats[dedans].astype(np.float64)
        matrice = np.column_stack((q[:, 0], q[:, 1], np.ones(len(q))))
        coefficients, *_ = np.linalg.lstsq(matrice, q[:, 2], rcond=None)
        a, b, c = coefficients
        if math.hypot(a, b) > math.tan(math.radians(10)) or abs(c) > 0.10:
            return self._lisser(None)
        return self._lisser(np.array([a, b, c]))

    def _lisser(self, plan: np.ndarray | None) -> tuple[tuple[float, float, float], bool]:
        if plan is None:
            if self._sol_lisse is not None:
                # Mémoire courte du dernier plan estimé, qui revient vers le plan théorique.
                self._sol_lisse = 0.8 * self._sol_lisse
                a, b, c = self._sol_lisse
                return (float(a), float(b), float(c)), False
            return (0.0, 0.0, 0.0), False
        if self._sol_lisse is None:
            self._sol_lisse = plan
        else:
            self._sol_lisse = 0.6 * self._sol_lisse + 0.4 * plan
        a, b, c = self._sol_lisse
        return (float(a), float(b), float(c)), True

    def _cellules(self, points: np.ndarray, hauteurs: np.ndarray, choix: object
                  ) -> tuple[np.ndarray, np.ndarray]:
        """Regroupe les points en cellules de `cellule` m ; garde celles qui ont assez de
        points. Renvoie le barycentre (x, y) et la hauteur extrême (max ou min) par cellule."""
        if len(points) == 0:
            return np.zeros((0, 2)), np.zeros(0)
        ix = np.floor(points[:, 0] / self.cellule).astype(np.int64)
        iy = np.floor(points[:, 1] / self.cellule).astype(np.int64)
        cle = ix * 100_003 + iy
        uniques, inverse, nombres = np.unique(cle, return_inverse=True, return_counts=True)
        sx = np.bincount(inverse, points[:, 0], len(uniques))
        sy = np.bincount(inverse, points[:, 1], len(uniques))
        extreme = np.full(len(uniques), -np.inf if choix is max else np.inf)
        if choix is max:
            np.maximum.at(extreme, inverse, hauteurs)
        else:
            np.minimum.at(extreme, inverse, hauteurs)
        # Au loin, moins de points par cellule : seuil abaissé avec le carré de la distance.
        cx, cy = sx / nombres, sy / nombres
        distance = np.hypot(cx, cy)
        seuil = np.maximum(2, np.minimum(self.points_min, 2 + 8.0 / np.maximum(distance, 0.5)**2))
        garde = nombres >= seuil
        return np.column_stack((cx[garde], cy[garde])), extreme[garde]

    def _confirmer(self, candidats: np.ndarray, pose: Pose2D | Odometrie | None) -> np.ndarray:
        """Cellules qui ont un voisin parmi celles de l'image précédente."""
        p = pose.pose if isinstance(pose, Odometrie) else pose
        if p is not None and len(candidats):
            c, s = math.cos(p.cap), math.sin(p.cap)
            courants = np.column_stack((p.x + c * candidats[:, 0] - s * candidats[:, 1],
                                        p.y + s * candidats[:, 0] + c * candidats[:, 1]))
        else:
            courants = candidats
        en_odom = p is not None
        confirmes = np.zeros(len(candidats), dtype=bool)
        if self._precedents is not None and len(candidats) and len(self._precedents[0]):
            precedents, precedents_odom = self._precedents
            tolerance = (self.tolerance_confirmation if en_odom and precedents_odom
                         else self.tolerance_sans_pose)
            if en_odom == precedents_odom:
                distances, _ = cKDTree(precedents).query(courants, k=1,
                                                         distance_upper_bound=tolerance)
                confirmes = np.isfinite(distances)
        self._precedents = (courants, en_odom)
        return confirmes

    # ------------------------------------------------------------------ requêtes
    def point_3d(self, camera: str, u: float, v: float) -> tuple[float, float, float] | None:
        """Point 3D (base_link) vu au pixel normalisé (u, v) de `camera`, d'après la dernière
        profondeur ; None sans profondeur à cet endroit."""
        with self._verrou:
            derniere = self._derniere
        if derniere is None:
            return None
        geo, z = derniere
        px = u * geo.largeur - 0.5
        py = v * geo.hauteur - 0.5
        rectif = geo.rectif_g if camera == "gauche" else geo.rectif_d
        if rectif is not None:
            k, dist, r, p = rectif
            rect = cv2.undistortPoints(np.array([[[px, py]]], np.float64), k, dist, R=r, P=p)
            px, py = float(rect[0, 0, 0]), float(rect[0, 0, 1])
        ligne = int(round(py))
        if not 0 <= ligne < geo.hauteur:
            return None
        if camera == "droite":
            # Pixel gauche dont la disparité ramène sur ce pixel droit : u_g − d(u_g) = u_d.
            ug = np.arange(int(px), min(int(px) + self.disparites, geo.largeur))
            if not len(ug):
                return None
            zl = z[ligne, ug]
            with np.errstate(divide="ignore", invalid="ignore"):
                d = geo.focale * geo.base / zl
            ecart = np.abs(ug - d - px)
            ecart[~np.isfinite(ecart)] = np.inf
            if not np.isfinite(ecart.min()) or ecart.min() > 1.5:
                return None
            px = float(ug[int(np.argmin(ecart))])
        colonne = int(round(px))
        if not 0 <= colonne < geo.largeur:
            return None
        fenetre = z[max(ligne - 2, 0):ligne + 3, max(colonne - 2, 0):colonne + 3]
        valeurs = fenetre[np.isfinite(fenetre)]
        if len(valeurs) < 3:
            return None
        profondeur = float(np.median(valeurs))
        direction = np.array([(px - geo.cx) / geo.focale, (py - geo.cy) / geo.focale, 1.0])
        point = geo.rotation @ (direction * profondeur) + geo.centre
        return float(point[0]), float(point[1]), float(point[2])

    def mesurer_zone(self, camera: str, u: float, v: float,
                     boite: tuple[float, float, float, float] | None = None
                     ) -> dict[str, float] | None:
        """Mesure 3D (base_link) d'un objet pointé en `(u, v)` (et dans `boite` u0, v0, u1, v1) :
        position `(x, y)`, `hauteur`, `largeur` et `longueur` d'après la carte de profondeur."""
        with self._verrou:
            derniere = self._derniere
            sol = self._sol_lisse
        if derniere is None:
            return None
        geo, z = derniere
        u0, v0, u1, v1 = boite if boite is not None else (
            max(0.0, u - 0.05), max(0.0, v - 0.25), min(1.0, u + 0.05), min(1.0, v + 0.02))
        c0 = max(0, int(math.floor(min(u0, u1) * geo.largeur)))
        c1 = min(geo.largeur, int(math.ceil(max(u0, u1) * geo.largeur)) + 1)
        r0 = max(0, int(math.floor(min(v0, v1) * geo.hauteur)))
        r1 = min(geo.hauteur, int(math.ceil(max(v0, v1) * geo.hauteur)) + 1)
        if c1 - c0 < 2 or r1 - r0 < 2:
            p3 = self.point_3d(camera, u, v)
            return None if p3 is None else {"x": p3[0], "y": p3[1], "hauteur": max(0.1, p3[2])}
        sous_z = z[r0:r1, c0:c1]
        valide = np.isfinite(sous_z) & (sous_z > 0.2) & (sous_z < 12.0)
        if int(np.count_nonzero(valide)) < 4:
            p3 = self.point_3d(camera, u, v)
            return None if p3 is None else {"x": p3[0], "y": p3[1], "hauteur": max(0.1, p3[2])}
        # Profondeur de référence : premier quartile des profondeurs de la boîte (avant de l'objet)
        z_ref = float(np.percentile(sous_z[valide], 30))
        masque = valide & ( np.abs(sous_z - z_ref) < 0.45 )
        if int(np.count_nonzero(masque)) < 3:
            masque = valide
        yy, xx = np.nonzero(masque)
        px = (c0 + xx).astype(np.float64)
        py = (r0 + yy).astype(np.float64)
        prof = sous_z[yy, xx].astype(np.float64)
        dirs = np.column_stack(((px - geo.cx) / geo.focale, (py - geo.cy) / geo.focale,
                                np.ones_like(px)))
        pts = (dirs * prof[:, None]) @ geo.rotation.T + geo.centre
        z_sol = (pts[:, 0] * sol[0] + pts[:, 1] * sol[1] + sol[2]) if sol is not None else 0.0
        h_pts = pts[:, 2] - z_sol
        x_med = float(np.median(pts[:, 0]))
        y_med = float(np.median(pts[:, 1]))
        hauteur = float(np.clip(np.percentile(h_pts, 92), 0.05, 3.0))
        largeur = float(np.clip(np.percentile(pts[:, 1], 90) - np.percentile(pts[:, 1], 10),
                                0.08, 3.0))
        longueur = float(np.clip(np.percentile(pts[:, 0], 90) - np.percentile(pts[:, 0], 10),
                                 0.08, 3.0))
        return {"x": x_med, "y": y_med, "hauteur": hauteur, "largeur": largeur,
                "longueur": longueur}

    def mesurer_cluster_local(self, x_local: float, y_local: float,
                              rayon: float = 0.25) -> dict[str, float] | None:
        """Mesure la hauteur et l'étendue 3D stéréo autour d'un impact lidar `(x_local, y_local)`
        dans `base_link`. Renvoie `None` si l'objet est hors du champ stéréo."""
        if x_local < 0.35 or math.hypot(x_local, y_local) > self.portee * 1.5:
            return None
        with self._verrou:
            nuage = self._nuage_3d
        if nuage is None or len(nuage[0]) < 4:
            return None
        xy, hauteurs = nuage
        seuil = max(0.22, min(0.65, rayon + 0.14))
        proches = np.hypot(xy[:, 0] - x_local, xy[:, 1] - y_local) <= seuil
        if int(np.count_nonzero(proches)) < 4:
            return None
        pts = xy[proches]
        h_pts = hauteurs[proches]
        hauteur = float(np.clip(np.percentile(h_pts, 92), 0.08, 2.50))
        z_bas = float(np.clip(np.percentile(h_pts, 10), 0.0, 2.0))
        etendue = float(np.hypot(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])))
        return {"hauteur": hauteur, "z_bas": z_bas, "etendue": etendue}


def image_profondeur(z: np.ndarray, portee: float, largeur: int = 320) -> bytes:
    """Profondeur en fausses couleurs (proche : rouge, loin : bleu, inconnu : noir), JPEG."""
    hauteur = max(1, round(z.shape[0] * largeur / z.shape[1]))
    petite = cv2.resize(z, (largeur, hauteur), interpolation=cv2.INTER_NEAREST)
    niveau = np.nan_to_num(np.clip(petite / portee, 0, 1), nan=1.0)
    couleurs = cv2.applyColorMap((255 * (1 - niveau)).astype(np.uint8), cv2.COLORMAP_TURBO)
    couleurs[~np.isfinite(petite)] = 0
    ok, jpeg = cv2.imencode(".jpg", couleurs, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return jpeg.tobytes() if ok else b""
