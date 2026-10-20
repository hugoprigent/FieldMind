"""Localisation du rover : filtre de Kalman étendu (EKF) sur (x, y, cap, biais du gyroscope).

- Prédiction à chaque message IMU (50 Hz) : vitesse des roues arrière (codeurs) et vitesse de
  lacet du gyroscope corrigée de son biais estimé.
- Correction par le GNSS RTK, avec son bras de levier (antenne décalée du centre du rover) et sa
  latence : la mesure datée du passé est appliquée à l'état de cet instant, puis les prédictions
  suivantes sont rejouées. Les mesures incohérentes (test du χ²) sont rejetées.
- Correction lente par le cap absolu de l'IMU (magnétomètre), faiblement pondérée : son erreur
  varie lentement et ne doit pas être prise pour du bruit blanc.

La pose estimée est celle du centre de l'emprise du rover (même point que la vérité simulée).
"""

from __future__ import annotations

import math
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass

import numpy as np

from services.protocole.fiche_rover import FicheRover
from services.protocole.messages import Gnss, Imu, Odometrie
from services.protocole.scenario import PoseDepart

HISTORIQUE_S = 1.5
SEUIL_CHI2_GNSS = 13.8
"""χ² à 2 degrés de liberté, probabilité 0,999."""
PERIODE_CAP_ABSOLU_S = 0.5


def angle_normalise(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def cap_depuis_quaternion(q: tuple[float, float, float, float]) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


@dataclass(frozen=True, slots=True)
class PoseEstimee:
    t: float
    x: float
    y: float
    cap: float
    sigma_xy: float
    sigma_cap: float
    vitesse: float


@dataclass(slots=True)
class _Etape:
    """État après une prédiction, et l'entrée qui l'a produite (pour pouvoir la rejouer)."""

    t: float
    etat: np.ndarray
    cov: np.ndarray
    vitesse: float
    lacet: float
    dt: float


class Localisation:
    """EKF déterministe, piloté uniquement par l'horodatage des messages."""

    def __init__(self, fiche: FicheRover, depart: PoseDepart) -> None:
        g = fiche.geometrie
        self._rayon_roue = g.rayon_roue
        self._bras_essieu = -g.essieu_arriere_x  # centre du rover devant l'essieu arrière
        gx, gy, _ = fiche.capteurs.gnss.position
        self._antenne = (gx, gy)
        self._sigma_cap_absolu = math.radians(max(fiche.capteurs.imu.bruit_cap_absolu_deg, 1.0) * 2)
        self.etat = np.array([depart.x, depart.y, depart.cap, 0.0])
        self.cov = np.diag([0.5**2, 0.5**2, math.radians(10) ** 2, 0.01**2])
        self.t: float | None = None
        self.vitesse = 0.0
        self._historique: deque[_Etape] = deque()
        self._t_cap_absolu = -math.inf
        self.gnss_acceptes = 0
        self.gnss_rejetes = 0

    # ------------------------------------------------------------------ entrées
    def sur_odometrie(self, message: Odometrie) -> None:
        self.vitesse = 0.5 * (message.roue_ar_g + message.roue_ar_d) * self._rayon_roue

    def sur_imu(self, message: Imu) -> None:
        lacet = message.vitesse_angulaire[2]
        if self.t is None:
            self.t = message.t
        dt = message.t - self.t
        if dt > 0:
            # En pente, seule la composante horizontale de la vitesse déplace le rover sur la
            # carte (tangage de l'orientation fusionnée de la centrale).
            w, x, y, z = message.orientation
            tangage = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
            vitesse = self.vitesse * math.cos(tangage)
            self._predire(dt, vitesse, lacet)
            self.t = message.t
            self._historique.append(
                _Etape(self.t, self.etat.copy(), self.cov.copy(), vitesse, lacet, dt)
            )
            while self._historique and self._historique[0].t < self.t - HISTORIQUE_S:
                self._historique.popleft()
        if message.t - self._t_cap_absolu >= PERIODE_CAP_ABSOLU_S:
            self._t_cap_absolu = message.t
            self._corriger_cap(cap_depuis_quaternion(message.orientation))

    def sur_gnss(self, message: Gnss) -> bool:
        """Applique une mesure GNSS datée ; renvoie False si elle est rejetée."""
        if message.fix == "aucun":
            return False
        index = self._index_avant(message.t)
        if index is None:
            return self._corriger_gnss(message)  # rien à rejouer
        etape = self._historique[index]
        suivantes = list(self._historique)[index + 1 :]
        sauvegarde = (self.etat.copy(), self.cov.copy())
        self.etat, self.cov = etape.etat.copy(), etape.cov.copy()
        accepte = self._corriger_gnss(message)
        if not accepte:
            self.etat, self.cov = sauvegarde
            return False
        # Rejeu des prédictions postérieures à la mesure, et mise à jour de l'historique.
        etape.etat, etape.cov = self.etat.copy(), self.cov.copy()
        for suivante in suivantes:
            self._predire(suivante.dt, suivante.vitesse, suivante.lacet)
            suivante.etat, suivante.cov = self.etat.copy(), self.cov.copy()
        return True

    # ------------------------------------------------------------------ filtre
    def _index_avant(self, t: float) -> int | None:
        temps = [e.t for e in self._historique]
        i = bisect_right(temps, t) - 1
        if i < 0 or i == len(temps) - 1:
            return None
        return i

    def _predire(self, dt: float, vitesse: float, lacet: float) -> None:
        x, y, cap, biais = self.etat
        omega = lacet - biais
        d = self._bras_essieu
        c, s = math.cos(cap), math.sin(cap)
        # Vitesse du centre : v le long de l'axe + ω·d latéralement (point devant l'essieu arrière).
        self.etat = np.array(
            [
                x + (vitesse * c - omega * d * s) * dt,
                y + (vitesse * s + omega * d * c) * dt,
                angle_normalise(cap + omega * dt),
                biais,
            ]
        )
        jac = np.eye(4)
        jac[0, 2] = (-vitesse * s - omega * d * c) * dt
        jac[1, 2] = (vitesse * c - omega * d * s) * dt
        jac[0, 3] = d * s * dt
        jac[1, 3] = -d * c * dt
        jac[2, 3] = -dt
        # Bruit : glissement des pneus (proportionnel à la vitesse et au virage), gyroscope, biais.
        sigma_v = 0.03 + 0.08 * abs(vitesse) + 0.1 * abs(omega * vitesse)
        sigma_lateral = 0.02 + 0.1 * abs(omega * vitesse)
        q_local = np.diag([sigma_v**2, sigma_lateral**2]) * dt
        rot = np.array([[c, -s], [s, c]])
        bruit = np.zeros((4, 4))
        bruit[:2, :2] = rot @ q_local @ rot.T
        bruit[2, 2] = (0.01**2 + (0.05 * omega) ** 2) * dt
        bruit[3, 3] = (2e-4) ** 2 * dt
        self.cov = jac @ self.cov @ jac.T + bruit

    def _corriger(self, innovation: np.ndarray, h: np.ndarray, r: np.ndarray) -> float:
        s = h @ self.cov @ h.T + r
        s_inv = np.linalg.inv(s)
        gain = self.cov @ h.T @ s_inv
        self.etat = self.etat + gain @ innovation
        self.etat[2] = angle_normalise(self.etat[2])
        # Forme de Joseph : covariance symétrique et définie positive.
        i_kh = np.eye(4) - gain @ h
        self.cov = i_kh @ self.cov @ i_kh.T + gain @ r @ gain.T
        return float(innovation @ s_inv @ innovation)

    def _corriger_gnss(self, message: Gnss) -> bool:
        x, y, cap, _ = self.etat
        gx, gy = self._antenne
        c, s = math.cos(cap), math.sin(cap)
        predit = np.array([x + gx * c - gy * s, y + gx * s + gy * c])
        innovation = np.array([message.x, message.y]) - predit
        h = np.zeros((2, 4))
        h[0, 0] = h[1, 1] = 1.0
        h[0, 2] = -gx * s - gy * c
        h[1, 2] = gx * c - gy * s
        r = np.eye(2) * message.sigma**2
        s_mat = h @ self.cov @ h.T + r
        mahalanobis = float(innovation @ np.linalg.inv(s_mat) @ innovation)
        if mahalanobis > SEUIL_CHI2_GNSS and self.gnss_acceptes > 5:
            self.gnss_rejetes += 1
            return False
        self._corriger(innovation, h, r)
        self.gnss_acceptes += 1
        return True

    def _corriger_cap(self, cap_mesure: float) -> None:
        innovation = np.array([angle_normalise(cap_mesure - self.etat[2])])
        h = np.array([[0.0, 0.0, 1.0, 0.0]])
        self._corriger(innovation, h, np.array([[self._sigma_cap_absolu**2]]))

    # ------------------------------------------------------------------ sorties
    def pose(self) -> PoseEstimee:
        sigma_xy = math.sqrt(max(float(np.linalg.eigvalsh(self.cov[:2, :2]).max()), 0.0))
        return PoseEstimee(
            t=self.t or 0.0,
            x=float(self.etat[0]),
            y=float(self.etat[1]),
            cap=float(self.etat[2]),
            sigma_xy=sigma_xy,
            sigma_cap=math.sqrt(max(float(self.cov[2, 2]), 0.0)),
            vitesse=self.vitesse,
        )

    def pose_a(self, t: float) -> PoseEstimee:
        """Pose estimée à l'instant `t` (interpolée dans l'historique, ou la plus récente)."""
        courante = self.pose()
        if not self._historique or t >= self._historique[-1].t:
            return courante
        temps = [e.t for e in self._historique]
        i = bisect_right(temps, t)
        if i == 0:
            e = self._historique[0]
            return PoseEstimee(
                t, e.etat[0], e.etat[1], e.etat[2], courante.sigma_xy, courante.sigma_cap, e.vitesse
            )
        a, b = self._historique[i - 1], self._historique[i]
        f = (t - a.t) / max(b.t - a.t, 1e-9)
        cap = a.etat[2] + f * angle_normalise(b.etat[2] - a.etat[2])
        return PoseEstimee(
            t,
            float(a.etat[0] + f * (b.etat[0] - a.etat[0])),
            float(a.etat[1] + f * (b.etat[1] - a.etat[1])),
            angle_normalise(float(cap)),
            courante.sigma_xy,
            courante.sigma_cap,
            a.vitesse,
        )
