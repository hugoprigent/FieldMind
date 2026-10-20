"""Navigation GPS : recalage de l'odométrie sur le GPS, zone autorisée et lieux mémorisés.

Le GPS ATGM336H est précis à 2–3 m, bruité d'une seconde à l'autre, et ne donne pas de cap à
l'arrêt. L'odométrie (lidar + gyroscope) est précise au centimètre sur quelques mètres mais
dérive lentement, et son repère `odom` n'est pas orienté vers le nord. On garde donc les deux :

- chaque seconde, la position de l'**antenne** dans `odom` (bras de levier du profil) est
  appariée au point GPS converti en mètres est/nord (ENU local autour d'une référence) ;
- sur une fenêtre glissante (5 min), la transformation rigide `odom → ENU` (rotation + translation)
  qui superpose au mieux les deux trajectoires est calculée par moindres carrés pondérés
  (Procrustes 2D, poids 1/HDOP²). La rotation n'est fiable qu'après quelques mètres parcourus :
  l'alignement est déclaré bon quand l'étendue du trajet dépasse `alignement_etendue_m` et que
  l'écart résiduel reste sous `alignement_residu_max_m` ;
- un but GPS est converti dans `odom` avec cette transformation, puis atteint avec la navigation
  locale (lidar, stéréo, carte) : le GPS dit où aller, les capteurs proches disent comment y
  aller sans danger. La fenêtre glissante absorbe la dérive de l'odométrie.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from fieldmind.config import DOSSIER_UTILISATEUR, Profil
from fieldmind.types import Gnss, Odometrie, Pose2D

A_WGS84 = 6378137.0
E2_WGS84 = 6.69437999014e-3


def rayons_terre(latitude: float) -> tuple[float, float]:
    """Rayons de courbure (méridien, premier vertical) à cette latitude, en mètres."""
    s = math.sin(math.radians(latitude))
    w = 1 - E2_WGS84 * s * s
    return A_WGS84 * (1 - E2_WGS84) / w**1.5, A_WGS84 / math.sqrt(w)


class Enu:
    """Plan local est/nord autour d'une référence (exact à quelques cm sur quelques km)."""

    def __init__(self, latitude: float, longitude: float) -> None:
        self.lat0, self.lon0 = latitude, longitude
        m, n = rayons_terre(latitude)
        self.m_par_deg_lat = math.radians(1) * m
        self.m_par_deg_lon = math.radians(1) * n * math.cos(math.radians(latitude))

    def vers_enu(self, latitude: float, longitude: float) -> tuple[float, float]:
        return ((longitude - self.lon0) * self.m_par_deg_lon,
                (latitude - self.lat0) * self.m_par_deg_lat)

    def vers_latlon(self, est: float, nord: float) -> tuple[float, float]:
        return self.lat0 + nord / self.m_par_deg_lat, self.lon0 + est / self.m_par_deg_lon


@dataclass
class Alignement:
    ok: bool = False
    rotation: float = 0.0  # angle de odom vers ENU (rad) : ENU = R(rotation)·odom + translation
    translation: tuple[float, float] = (0.0, 0.0)
    residu_m: float = math.inf
    etendue_m: float = 0.0
    points: int = 0


def procrustes_2d(p: np.ndarray, q: np.ndarray, w: np.ndarray
                  ) -> tuple[float, np.ndarray, float]:
    """Rotation θ et translation t minimisant Σ w ‖R(θ)·p + t − q‖² ; renvoie aussi le résidu
    quadratique moyen pondéré."""
    w = w / w.sum()
    pm, qm = w @ p, w @ q
    dp, dq = p - pm, q - qm
    sxx = float(w @ (dp[:, 0] * dq[:, 0]))
    syy = float(w @ (dp[:, 1] * dq[:, 1]))
    sxy = float(w @ (dp[:, 0] * dq[:, 1]))
    syx = float(w @ (dp[:, 1] * dq[:, 0]))
    theta = math.atan2(sxy - syx, sxx + syy)
    c, s = math.cos(theta), math.sin(theta)
    r = np.array([[c, -s], [s, c]])
    t = qm - r @ pm
    ecarts = (p @ r.T + t) - q
    residu = math.sqrt(float(w @ (ecarts**2).sum(axis=1)))
    return theta, t, residu


def dans_polygone(x: float, y: float, polygone: np.ndarray) -> bool:
    dedans = False
    n = len(polygone)
    for i in range(n):
        x1, y1 = polygone[i]
        x2, y2 = polygone[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            dedans = not dedans
    return dedans


class NavigationGps:
    def __init__(self, profil: Profil, dossier: Path = DOSSIER_UTILISATEUR) -> None:
        self.profil = profil
        self.dossier = dossier
        self._verrou = threading.Lock()
        self.enu: Enu | None = None
        lat0 = profil.valeur("gnss.reference_lat")
        lon0 = profil.valeur("gnss.reference_lon")
        if lat0 is not None and lon0 is not None:
            self.enu = Enu(float(lat0), float(lon0))
        self._paires: deque[tuple[float, float, float, float, float, float]] = deque(maxlen=600)
        self.alignement = Alignement()
        self._rotation_connue: float | None = None
        self.dernier_fix: Gnss | None = None
        self.trace: deque[tuple[float, float]] = deque(maxlen=600)
        self.zone: list[list[float]] | None = self._lire("zone.json")
        self.lieux: list[dict[str, Any]] = self._lire("lieux.json") or []
        self.session = f"{time.time():.0f}"  # les positions odom ne valent que pour cette session

    # ------------------------------------------------------------------ persistance

    def _lire(self, nom: str) -> Any:
        try:
            return json.loads((self.dossier / nom).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None

    def _ecrire(self, nom: str, valeur: Any) -> None:
        self.dossier.mkdir(parents=True, exist_ok=True)
        temporaire = self.dossier / f"{nom}.tmp"
        temporaire.write_text(json.dumps(valeur, ensure_ascii=False, indent=1), encoding="utf-8")
        temporaire.replace(self.dossier / nom)

    # ------------------------------------------------------------------ mesures

    def mesure(self, fix: Gnss, odom: Odometrie | None) -> None:
        """Un point GPS (1 Hz) apparié à la pose odométrique du même instant."""
        self.dernier_fix = fix
        hdop = fix.hdop or 5.0
        if not fix.valide or odom is None or hdop > float(self.profil.valeur(
                "gnss.hdop_max", 5.0)) or fix.latitude is None or fix.longitude is None:
            return
        with self._verrou:
            if self.enu is None:
                self.enu = Enu(fix.latitude, fix.longitude)
            e, n = self.enu.vers_enu(fix.latitude, fix.longitude)
            ax, ay = self._antenne(odom.pose)
            self._paires.append((fix.t, ax, ay, e, n, 1.0 / (hdop * hdop)))
            self._aligner(fix.t)
            if self.alignement.ok:
                lat, lon = self.odom_vers_latlon(odom.pose.x, odom.pose.y)
                if self.trace:
                    e0, n0 = self.enu.vers_enu(*self.trace[-1])
                    e1, n1 = self.enu.vers_enu(lat, lon)
                    if math.hypot(e1 - e0, n1 - n0) < 0.3:
                        return
                self.trace.append((round(lat, 7), round(lon, 7)))

    def _antenne(self, pose: Pose2D) -> tuple[float, float]:
        return pose.vers_parent(float(self.profil.valeur("gnss.x_m", 0.0)),
                                float(self.profil.valeur("gnss.y_m", 0.0)))

    def _aligner(self, maintenant: float) -> None:
        fenetre = float(self.profil.valeur("gnss.fenetre_s", 300.0))
        while self._paires and maintenant - self._paires[0][0] > fenetre:
            self._paires.popleft()
        a = Alignement(points=len(self._paires))
        if len(self._paires) < 5:
            self.alignement = a
            return
        donnees = np.array(self._paires)
        p, q = donnees[:, 1:3], donnees[:, 3:5]
        # Les points récents comptent plus : la dérive de l'odométrie est oubliée doucement
        age = maintenant - donnees[:, 0]
        w = donnees[:, 5] * np.exp(-age / fenetre)
        a.etendue_m = float(np.max(np.linalg.norm(p - p.mean(axis=0), axis=1))) * 2
        etendue_min = float(self.profil.valeur("gnss.alignement_etendue_m", 6.0))
        if a.etendue_m >= etendue_min:
            theta, t, residu = procrustes_2d(p, q, w)
            self._rotation_connue = theta
        elif self._rotation_connue is not None:
            # Trajet trop court pour la rotation : on garde la dernière, on recale la position
            theta = self._rotation_connue
            c, s = math.cos(theta), math.sin(theta)
            r = np.array([[c, -s], [s, c]])
            wn = w / w.sum()
            t = wn @ q - r @ (wn @ p)
            ecarts = p @ r.T + t - q
            residu = math.sqrt(float(wn @ (ecarts**2).sum(axis=1)))
        else:
            self.alignement = a
            return
        a.rotation, a.translation, a.residu_m = theta, (float(t[0]), float(t[1])), residu
        a.ok = residu <= float(self.profil.valeur("gnss.alignement_residu_max_m", 4.0))
        self.alignement = a

    def oublier_odometrie(self) -> None:
        """Le repère odom a changé (remise à zéro) : le recalage repart de rien."""
        with self._verrou:
            self._paires.clear()
            self._rotation_connue = None
            self.alignement = Alignement()
            self.session = f"{time.time():.0f}"

    # ------------------------------------------------------------------ conversions

    def odom_vers_enu(self, x: float, y: float) -> tuple[float, float]:
        a = self.alignement
        c, s = math.cos(a.rotation), math.sin(a.rotation)
        return c * x - s * y + a.translation[0], s * x + c * y + a.translation[1]

    def enu_vers_odom(self, e: float, n: float) -> tuple[float, float]:
        a = self.alignement
        dx, dy = e - a.translation[0], n - a.translation[1]
        c, s = math.cos(a.rotation), math.sin(a.rotation)
        return c * dx + s * dy, -s * dx + c * dy

    def odom_vers_latlon(self, x: float, y: float) -> tuple[float, float]:
        assert self.enu is not None
        return self.enu.vers_latlon(*self.odom_vers_enu(x, y))

    def latlon_vers_odom(self, latitude: float, longitude: float) -> tuple[float, float] | None:
        if not self.alignement.ok or self.enu is None:
            return None
        return self.enu_vers_odom(*self.enu.vers_enu(latitude, longitude))

    def cap_vrai_deg(self, cap_odom: float) -> float:
        """Cap par rapport au nord vrai (0 = nord, 90 = est) d'un cap du repère odom."""
        angle_enu = cap_odom + self.alignement.rotation  # 0 = est, trigonométrique
        return (90.0 - math.degrees(angle_enu)) % 360.0

    # ------------------------------------------------------------------ zone autorisée

    def regler_zone(self, points: list[list[float]] | None) -> None:
        if points is not None:
            if len(points) < 3 or len(points) > 100:
                raise ValueError("une zone a de 3 à 100 sommets")
            points = [[float(lat), float(lon)] for lat, lon in points]
        self.zone = points
        if points is None:
            (self.dossier / "zone.json").unlink(missing_ok=True)
        else:
            self._ecrire("zone.json", points)

    def hors_zone(self, pose: Pose2D, sens: int) -> bool | None:
        """True si le mouvement (sens +1/-1) sort de la zone ou y reste dehors en s'éloignant ;
        None si on ne peut pas savoir (pas de zone, alignement pas encore fait)."""
        if not self.zone or not self.alignement.ok or self.enu is None:
            return None
        polygone = np.array([self.enu.vers_enu(lat, lon) for lat, lon in self.zone])
        x, y = self.odom_vers_enu(pose.x, pose.y)
        cap = pose.cap + self.alignement.rotation + (0 if sens > 0 else math.pi)
        devant = (x + 1.0 * math.cos(cap), y + 1.0 * math.sin(cap))
        if dans_polygone(*devant, polygone):
            return False
        if not dans_polygone(x, y, polygone):
            # Déjà dehors : seulement en revenant vers l'intérieur
            centre = polygone.mean(axis=0)
            return math.cos(cap) * (centre[0] - x) + math.sin(cap) * (centre[1] - y) <= 0
        return True

    # ------------------------------------------------------------------ lieux

    def memoriser(self, nom: str, pose: Pose2D) -> dict[str, Any]:
        nom = " ".join(str(nom).split())[:40]
        if not nom:
            raise ValueError("nom de lieu vide")
        lieu: dict[str, Any] = {"nom": nom, "x": round(pose.x, 3), "y": round(pose.y, 3),
                                "session": self.session}
        if self.alignement.ok:
            lieu["lat"], lieu["lon"] = (round(v, 7) for v in self.odom_vers_latlon(pose.x, pose.y))
        self.lieux = [ancien for ancien in self.lieux if ancien["nom"] != nom] + [lieu]
        self._ecrire("lieux.json", self.lieux)
        return lieu

    def supprimer(self, nom: str) -> None:
        self.lieux = [lieu for lieu in self.lieux if lieu["nom"] != nom]
        self._ecrire("lieux.json", self.lieux)

    def lieu(self, nom: str) -> dict[str, Any] | None:
        nom = nom.strip().lower()
        for lieu in self.lieux:
            if lieu["nom"].lower() == nom:
                return lieu
        return None

    def lieu_vers_odom(self, lieu: dict[str, Any]) -> tuple[float, float] | None:
        """Position du lieu dans odom : par le GPS si possible, sinon odom de la même session."""
        if "lat" in lieu and self.alignement.ok:
            return self.latlon_vers_odom(lieu["lat"], lieu["lon"])
        if lieu.get("session") == self.session:
            return float(lieu["x"]), float(lieu["y"])
        return None

    # ------------------------------------------------------------------ message

    def message(self, odom: Odometrie | None, but: dict[str, float] | None = None,
                parcours: list[list[float]] | None = None) -> dict[str, Any]:
        f = self.dernier_fix
        a = self.alignement
        rover = None
        if a.ok and odom is not None and self.enu is not None:
            lat, lon = self.odom_vers_latlon(odom.pose.x, odom.pose.y)
            rover = {"lat": round(lat, 7), "lon": round(lon, 7),
                     "precision_m": round(max(a.residu_m, 0.5), 2),
                     "cap_vrai_deg": round(self.cap_vrai_deg(odom.pose.cap), 1)}
        return {
            "type": "gps", "t": round(time.monotonic(), 3),
            "fix": f.fix if f else "aucun", "satellites": f.satellites if f else 0,
            "hdop": f.hdop if f else None,
            "brut": {"lat": f.latitude, "lon": f.longitude} if f and f.valide else None,
            "rover": rover,
            "alignement": {"ok": a.ok, "residu_m": None if math.isinf(a.residu_m)
                           else round(a.residu_m, 2), "etendue_m": round(a.etendue_m, 1),
                           "points": a.points},
            "reference": {"lat": self.enu.lat0, "lon": self.enu.lon0} if self.enu else None,
            "trace": [list(p) for p in self.trace][-500:],
            "but": but, "parcours": parcours or [], "zone": self.zone,
            "lieux": [{"nom": lieu["nom"], "lat": lieu.get("lat"), "lon": lieu.get("lon")}
                      for lieu in self.lieux],
        }
