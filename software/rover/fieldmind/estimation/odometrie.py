"""Odométrie du rover sans codeurs de roue : recalage lidar + gyroscope + a priori de la commande.

À chaque tour du lidar (10 Hz) :

1. **Prédiction** depuis la pose du tour précédent : lacet intégré du gyroscope (corrigé de son
   biais et de son gain, estimés en marche), avance = vitesse mesurée au tour précédent, tirée
   vers la vitesse commandée (× le rapport vitesse réelle / vitesse commandée appris) avec une
   accélération bornée. Sans centrale : braquage commandé (modèle bicyclette).
2. **Redressement du tour** : le lidar tourne pendant que le rover avance (5 cm et jusqu'à 4°
   par tour) ; chaque point est ramené à l'instant de fin du tour avec le mouvement prédit.
   L'instant d'un point se déduit de son angle (le tour commence à l'angle 0 du lidar, sens
   `lidar.sens_horaire`), indépendamment de l'ordre du tableau.
3. **ICP point-à-ligne** contre la sous-carte des dernières images clés (pas seulement le tour
   précédent : la dérive ne s'accumule qu'au changement d'image clé, et pas du tout à l'arrêt),
   avec la prédiction comme a priori pondéré (cf. `icp.py`).
4. **Contrôle** : qualité minimale, nombre de points appariés, écart plausible à la prédiction.
   Sinon (champ ouvert, lidar masqué, recalage aberrant), la pose suit la prédiction seule
   (« gyro+commande ») et la sous-carte est réinitialisée si le rover s'en est trop éloigné.
5. **Mesures** : vitesse réelle (déplacement recalé / durée), vitesse de lacet ; quand le
   recalage est bon, on apprend en ligne le biais et le gain du gyroscope (moindres carrés
   récursifs, ce qui corrige aussi un axe inversé), le rapport vitesse réelle / commandée et le
   PWM nécessaire par m/s (calibration de la traction), et on détecte un blocage (consigne de
   mouvement sans déplacement mesuré).

Dans un couloir, l'ICP ne contraint pas l'avance (direction dégénérée) : l'a priori la fixe
et la vitesse mesurée n'apprend rien dans cette direction. Le GNSS (2–3 m) ne corrige pas la
pose (le repère odom doit rester continu) ; sa vitesse sert seulement d'appoint quand l'ICP
manque, au-delà de 0,4 m/s.
"""

from __future__ import annotations

import bisect
import math
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

from fieldmind.config import Profil
from fieldmind.estimation.icp import (
    ParametresIcp,
    ResultatIcp,
    SousCarte,
    normales_balayage,
    recaler,
)
from fieldmind.types import CommandeRoues, Gnss, Imu, Odometrie, Pose2D, Scan

HISTORIQUE_IMU_S = 3.0
HISTORIQUE_COMMANDE_S = 5.0


@dataclass(slots=True)
class _Commande:
    t: float
    vitesse: float
    braquage: float
    pwm: float


def arc(distance: float, dtheta: float) -> Pose2D:
    """Déplacement relatif le long d'un arc de cercle (longueur `distance`, rotation `dtheta`)."""
    if abs(dtheta) < 1e-6:
        return Pose2D(distance, distance * dtheta / 2, dtheta)
    rayon = distance / dtheta
    return Pose2D(rayon * math.sin(dtheta), rayon * (1 - math.cos(dtheta)), dtheta)


class Odometre:
    def __init__(self, profil: Profil) -> None:
        v = profil.valeur
        self.empattement = float(v("geometrie.empattement_m", 0.38))
        self.decalage_lidar = math.radians(float(v("lidar.decalage_deg", 0.0)))
        self.lidar_horaire = bool(v("lidar.sens_horaire", True))
        self.portee_min = float(v("lidar.portee_min_m", 0.12))
        self.portee_max = float(v("odometrie.portee_max_m", 10.0))
        self.qualite_min = float(v("odometrie.qualite_min", 0.35))
        self.points_min = int(v("odometrie.points_min", 40))
        self.distance_image_cle = float(v("odometrie.distance_image_cle_m", 0.25))
        self.angle_image_cle = math.radians(float(v("odometrie.angle_image_cle_deg", 8.0)))
        self.desinclinaison = bool(v("odometrie.desinclinaison", True))
        self.utiliser_lacet_esp = bool(v("odometrie.utiliser_lacet_esp", True))
        self.ecart_max_m = float(v("odometrie.ecart_prediction_max_m", 0.30))
        self.ecart_max_rad = math.radians(float(v("odometrie.ecart_prediction_max_deg", 12.0)))
        self.acceleration_max = float(v("odometrie.acceleration_max_m_s2", 1.2))
        self.blocage_s = float(v("securite.blocage_s", 2.5))
        self.parametres_icp = ParametresIcp(
            distance_initiale_m=float(v("odometrie.icp_distance_initiale_m", 0.40)),
            distance_finale_m=float(v("odometrie.icp_distance_finale_m", 0.12)),
            iterations_max=int(v("odometrie.icp_iterations", 20)))
        self.carte = SousCarte(int(v("odometrie.images_cles", 12)))

        # Calibrations apprises (conservées par remettre_a_zero).
        self.rapport_vitesse = float(v("odometrie.rapport_vitesse_initial", 1.0))
        self.pwm_par_m_s = float(v("traction.pwm_par_m_s", 35.0))
        self._rls_theta = np.array([1.0, 0.0])  # Δi = a·Δbrut + b·dt : a = gain, b = −gain·biais
        self._rls_p0 = np.diag([0.15**2, 0.03**2]) / 1e-3**2
        self._rls_p = self._rls_p0.copy()

        self._imu_t: list[float] = []
        self._imu_cumul: list[float] = []
        self._imu_vitesse = 0.0
        self._imu_lacet_precedent: float | None = None
        self._commandes: deque[_Commande] = deque()
        self._gnss: Gnss | None = None
        self._historique: deque[tuple[float, float, float, bool]] = deque()
        self._acceptes: deque[bool] = deque(maxlen=50)
        self.duree_derniere_s = 0.0
        self.dernier_resultat: ResultatIcp | None = None
        self.remettre_a_zero()

    # ------------------------------------------------------------------ état
    def remettre_a_zero(self) -> None:
        """Repère odom reposé sur la pose actuelle ; sous-carte vidée. Les calibrations
        apprises (biais du gyroscope, rapport de vitesse…) sont conservées."""
        self.pose = Pose2D()
        self.vitesse = 0.0
        self.vitesse_lacet = 0.0
        self.covariance = np.diag([1e-6, 1e-6, 1e-6])
        self.qualite = 0.0
        self.source = "aucune"
        self._t: float | None = None
        self.carte.vider()
        self._historique.clear()

    @property
    def gain_gyro(self) -> float:
        return float(self._rls_theta[0])

    @property
    def biais_gyro(self) -> float:
        a, b = self._rls_theta
        return float(-b / a) if abs(a) > 1e-3 else 0.0

    @property
    def bloque(self) -> bool:
        """Consigne de mouvement depuis `securite.blocage_s` sans déplacement mesuré au lidar."""
        if not self._historique:
            return False
        t_fin = self._historique[-1][0]
        if t_fin - self._historique[0][0] < self.blocage_s * 0.95:
            return False
        for t, v_cmd, v_mes, recale in self._historique:
            if t < t_fin - self.blocage_s:
                continue
            if abs(v_cmd) < 0.08 or abs(v_mes) > 0.03 or not recale:
                return False
        return True

    def etat(self) -> dict[str, float | int | str | bool]:
        return {
            "source": self.source,
            "qualite": round(self.qualite, 3),
            "vitesse_m_s": round(self.vitesse, 3),
            "biais_gyro_deg_s": round(math.degrees(self.biais_gyro), 3),
            "gain_gyro": round(self.gain_gyro, 3),
            "rapport_vitesse": round(self.rapport_vitesse, 3),
            "pwm_par_m_s": round(self.pwm_par_m_s, 2),
            "taux_recalage": round(sum(self._acceptes) / max(len(self._acceptes), 1), 3),
            "images_cles": len(self.carte.images),
            "points_sous_carte": len(self.carte),
            "duree_ms": round(self.duree_derniere_s * 1000, 2),
            "bloque": self.bloque,
        }

    # ------------------------------------------------------------------ entrées
    def imu(self, mesure: Imu) -> None:
        if mesure.etat == "ABSENT":
            return
        t = mesure.t
        omega = float(mesure.gyro_rad_s[2])
        if self._imu_t and t <= self._imu_t[-1]:
            return
        if not self._imu_t:
            cumul = 0.0
        elif self.utiliser_lacet_esp and mesure.lacet_rad is not None \
                and self._imu_lacet_precedent is not None:
            ecart = mesure.lacet_rad - self._imu_lacet_precedent
            cumul = self._imu_cumul[-1] + math.atan2(math.sin(ecart), math.cos(ecart))
        else:
            dt = min(t - self._imu_t[-1], 0.5)
            cumul = self._imu_cumul[-1] + 0.5 * (omega + self._imu_vitesse) * dt
        self._imu_lacet_precedent = mesure.lacet_rad
        self._imu_vitesse = omega
        self._imu_t.append(t)
        self._imu_cumul.append(cumul)
        if len(self._imu_t) > 16 and self._imu_t[0] < t - HISTORIQUE_IMU_S:
            coupe = bisect.bisect_left(self._imu_t, t - HISTORIQUE_IMU_S)
            del self._imu_t[:coupe]
            del self._imu_cumul[:coupe]

    def commande(self, commande: CommandeRoues) -> None:
        self._commandes.append(_Commande(commande.t, float(commande.vitesse),
                                         float(commande.braquage),
                                         0.5 * (commande.pwm_g + commande.pwm_d)))
        while self._commandes and self._commandes[0].t < commande.t - HISTORIQUE_COMMANDE_S:
            self._commandes.popleft()

    def gnss(self, mesure: Gnss) -> None:
        self._gnss = mesure

    # ------------------------------------------------------------------ gyroscope
    def _cumul_brut(self, t: float) -> float | None:
        ts = self._imu_t
        if not ts or t < ts[0] - 0.05 or t > ts[-1] + 0.25:
            return None
        if t >= ts[-1]:
            return self._imu_cumul[-1] + self._imu_vitesse * (t - ts[-1])
        i = bisect.bisect_right(ts, t)
        if i == 0:
            return self._imu_cumul[0]
        t0, t1 = ts[i - 1], ts[i]
        f = (t - t0) / max(t1 - t0, 1e-9)
        return self._imu_cumul[i - 1] + f * (self._imu_cumul[i] - self._imu_cumul[i - 1])

    def _rotation_brute(self, t1: float, t2: float) -> float | None:
        a, b = self._cumul_brut(t1), self._cumul_brut(t2)
        if a is None or b is None:
            return None
        return b - a

    def _rotation_gyro(self, t1: float, t2: float) -> float | None:
        brut = self._rotation_brute(t1, t2)
        if brut is None:
            return None
        a, b = self._rls_theta
        return float(a * brut + b * (t2 - t1))

    def _apprendre_gyro(self, brut: float, dt: float, mesure: float) -> None:
        """Moindres carrés récursifs (oubli 0,998) sur Δicp = a·Δbrut + b·dt.

        Sans rotation (ligne droite, arrêt), Δbrut ≈ biais·dt : a et b ne sont pas séparables ;
        on n'apprend alors que b (le biais), gain tenu.
        """
        p = self._rls_p
        oubli = 0.998
        erreur = mesure - (self._rls_theta[0] * brut + self._rls_theta[1] * dt)
        # Régresseur réduit à b quand le rover ne tourne pas (mise à jour cohérente de P).
        phi = np.array([brut, dt]) if abs(mesure) >= 0.01 else np.array([0.0, dt])
        k = p @ phi / (oubli + phi @ p @ phi)
        self._rls_theta = self._rls_theta + k * erreur
        p = (p - np.outer(k, phi @ p)) / oubli
        # Sans excitation, la covariance croîtrait sans fin : bornée à l'a priori.
        p[0, 0] = min(p[0, 0], self._rls_p0[0, 0])
        p[1, 1] = min(p[1, 1], self._rls_p0[1, 1])
        self._rls_p = 0.5 * (p + p.T)
        self._rls_theta[0] = float(np.clip(self._rls_theta[0], -1.5, 1.5))
        if abs(self._rls_theta[0]) < 0.3:  # gain absurde : on repart de l'a priori
            self._rls_theta = np.array([math.copysign(1.0, self._rls_theta[0]), 0.0])
            self._rls_p = self._rls_p0.copy()

    def _immobile_commande(self, t1: float, t2: float) -> bool:
        """Aucune consigne de mouvement depuis 1 s avant t1 jusqu'à t2 (traction coupée). Il faut
        recevoir les commandes (même nulles) : sans elles, on ne sait rien."""
        if not self._commandes or self._commandes[-1].t < t2 - 0.6:
            return False
        for c in reversed(self._commandes):
            if c.t > t2:
                continue
            if abs(c.vitesse) > 0.005 or abs(c.pwm) > 0:
                return False
            if c.t < t1 - 1.0:
                break
        return True

    # ------------------------------------------------------------------ commande
    def _commande_a(self, t: float) -> _Commande | None:
        if not self._commandes:
            return None
        derniere = None
        for c in reversed(self._commandes):
            if c.t <= t:
                derniere = c
                break
        if derniere is None or t - derniere.t > 0.6:
            return None  # plus de commande récente : l'ESP coupe la traction (chien de garde)
        return derniere

    # ------------------------------------------------------------------ tour du lidar
    def scan(self, scan: Scan) -> Odometrie | None:
        debut = time.perf_counter()
        t = float(scan.t)
        if self._t is not None and t <= self._t:
            return None
        angles = np.asarray(scan.angles, dtype=np.float64)
        distances = np.asarray(scan.distances, dtype=np.float64)
        garde = (distances >= self.portee_min) & (distances <= self.portee_max)
        angles, distances = angles[garde], distances[garde]
        points = np.column_stack((distances * np.cos(angles), distances * np.sin(angles)))

        if self._t is None:
            self._t = t
            if len(points) >= self.points_min:
                self.carte.ajouter(self.pose, points, t)
            self.source = "initialisation"
            self.qualite = 1.0 if len(points) >= self.points_min else 0.0
            self.duree_derniere_s = time.perf_counter() - debut
            return self._sortie(t)

        dt = t - self._t
        commande = self._commande_a(t)
        brut = self._rotation_brute(self._t, t)
        dtheta_gyro = self._rotation_gyro(self._t, t)

        # Prédiction.
        if commande is not None:
            cible = commande.vitesse * self.rapport_vitesse
            ecart = cible - self.vitesse
            v_pred = self.vitesse + float(np.clip(ecart, -self.acceleration_max * dt,
                                                  self.acceleration_max * dt))
        else:
            cible = self.vitesse
            v_pred = self.vitesse
        sans_recalage = not self._acceptes or not self._acceptes[-1]
        if sans_recalage:
            v_pred = self._appoint_gnss(v_pred, t)
        if dtheta_gyro is not None:
            dtheta = dtheta_gyro
            sigma_theta = 0.002 + 0.03 * abs(dtheta)
        elif commande is not None:
            dtheta = v_pred * math.tan(commande.braquage) / self.empattement * dt
            sigma_theta = 0.01 + 0.3 * abs(dtheta) + 0.1 * abs(v_pred) * dt
        else:
            dtheta = self.vitesse_lacet * dt
            sigma_theta = 0.01 + 0.5 * abs(dtheta)
        prediction = self.pose.composer(arc(v_pred * dt, dtheta))
        sigma_avance = 0.01 + 0.3 * abs(cible - self.vitesse) * dt + 0.15 * abs(v_pred) * dt
        sigma_lateral = 0.005 + 0.05 * abs(v_pred) * dt

        if self.desinclinaison and len(points):
            points = self._redresser(points, angles, float(scan.duree), v_pred, dtheta / dt)

        # Recalage.
        x0 = np.array([prediction.x, prediction.y, prediction.cap])
        c, s = math.cos(prediction.cap), math.sin(prediction.cap)
        rot = np.array([[c, -s], [s, c]])
        information = np.zeros((3, 3))
        information[:2, :2] = rot @ np.diag([1 / sigma_avance**2, 1 / sigma_lateral**2]) @ rot.T
        information[2, 2] = 1 / sigma_theta**2
        resultat = None
        if len(points) >= self.points_min and len(self.carte):
            normales, _ = normales_balayage(points)
            resultat = recaler(points, normales, self.carte, x0, x0, information,
                               self.parametres_icp)
        self.dernier_resultat = resultat
        accepte = resultat is not None and self._plausible(resultat, x0)

        precedente = self.pose
        if accepte:
            assert resultat is not None
            self.pose = Pose2D(float(resultat.pose[0]), float(resultat.pose[1]),
                               math.atan2(math.sin(resultat.pose[2]),
                                          math.cos(resultat.pose[2])))
            self.qualite = resultat.qualite * (0.6 if resultat.degenere else 1.0)
            base = "icp+gyro" if dtheta_gyro is not None else "icp+commande"
            self.source = base + ("+degenere" if resultat.degenere else "")
            q_increment = resultat.covariance
        else:
            self.pose = prediction
            self.qualite = 0.0
            self.source = "gyro+commande" if dtheta_gyro is not None else "commande"
            q_increment = np.linalg.inv(information)
        self.covariance = self.covariance + q_increment
        self._acceptes.append(accepte)

        # Mesures : vitesse et vitesse de lacet dans le repère du rover.
        relatif = precedente.inverse().composer(self.pose)
        v_instant = relatif.x / dt
        avance_observee = accepte and resultat is not None and not (
            resultat.degenere and abs(resultat.direction_faible @ np.array([
                math.cos(self.pose.cap), math.sin(self.pose.cap)])) > 0.7)
        if avance_observee:
            self.vitesse = 0.6 * v_instant + 0.4 * self.vitesse
        else:
            self.vitesse = v_pred
        self.vitesse_lacet = relatif.cap / dt

        if accepte and resultat is not None and resultat.qualite >= 0.5:
            if brut is not None:
                self._apprendre_gyro(brut, dt, relatif.cap)
            if avance_observee and commande is not None:
                self._apprendre_traction(commande, v_instant, t)
        elif brut is not None and self._immobile_commande(self._t, t):
            # Pas de recalage mais traction coupée : le rover est immobile, le gyroscope ne
            # mesure que son biais.
            self._apprendre_gyro(brut, dt, 0.0)
        self._historique.append((t, commande.vitesse if commande else 0.0, self.vitesse,
                                 bool(avance_observee)))
        while self._historique and self._historique[0][0] < t - self.blocage_s - 0.5:
            self._historique.popleft()

        self._gerer_images_cles(points, t, accepte)
        self._t = t
        self.duree_derniere_s = time.perf_counter() - debut
        return self._sortie(t)

    def _plausible(self, r: ResultatIcp, prediction: np.ndarray) -> bool:
        if r.qualite < self.qualite_min or r.appariements < self.points_min:
            return False
        ecart = r.pose - prediction
        angle = abs(math.atan2(math.sin(ecart[2]), math.cos(ecart[2])))
        return math.hypot(ecart[0], ecart[1]) <= self.ecart_max_m and angle <= self.ecart_max_rad

    def _redresser(self, points: np.ndarray, angles: np.ndarray, duree: float, vitesse: float,
                   omega: float) -> np.ndarray:
        """Ramène chaque point à l'instant de fin du tour (vitesses constantes pendant le tour)."""
        if self.lidar_horaire:
            angle_lidar = np.mod(self.decalage_lidar - angles, 2 * math.pi)
        else:
            angle_lidar = np.mod(angles - self.decalage_lidar, 2 * math.pi)
        tau = -duree * (1.0 - angle_lidar / (2 * math.pi))  # ≤ 0 : avant la fin du tour
        theta = omega * tau
        c, s = np.cos(theta), np.sin(theta)
        avance = vitesse * tau
        x = c * points[:, 0] - s * points[:, 1] + avance * np.cos(theta / 2)
        y = s * points[:, 0] + c * points[:, 1] + avance * np.sin(theta / 2)
        return np.column_stack((x, y))

    def _appoint_gnss(self, v_pred: float, t: float) -> float:
        g = self._gnss
        if g is None or not g.valide or g.vitesse_m_s is None or t - g.t > 2.0:
            return v_pred
        if g.vitesse_m_s < 0.4 or abs(v_pred) < 0.2:
            return v_pred
        return 0.7 * v_pred + 0.3 * math.copysign(g.vitesse_m_s, v_pred)

    def _apprendre_traction(self, commande: _Commande, v_instant: float, t: float) -> None:
        if abs(commande.vitesse) < 0.1 or abs(v_instant) < 0.05:
            return
        recentes = [c.vitesse for c in self._commandes if c.t >= t - 0.6]
        if not recentes or max(recentes) - min(recentes) > 0.05:
            return  # consigne en rampe : le moteur n'est pas en régime établi
        if (v_instant > 0) != (commande.vitesse > 0):
            return
        rapport = float(np.clip(v_instant / commande.vitesse, 0.3, 2.5))
        self.rapport_vitesse += 0.05 * (rapport - self.rapport_vitesse)
        if abs(commande.pwm) > 0:
            pwm = float(np.clip(abs(commande.pwm) / abs(v_instant), 5.0, 150.0))
            self.pwm_par_m_s += 0.05 * (pwm - self.pwm_par_m_s)

    def _gerer_images_cles(self, points: np.ndarray, t: float, accepte: bool) -> None:
        if len(points) < self.points_min:
            return
        if not accepte:
            # Sans recalage depuis longtemps, la sous-carte ne recouvre plus rien : on repart
            # du tour courant (la pose continue sur la prédiction).
            if self.carte.distance_plus_proche(self.pose.x, self.pose.y) > 2.0 or not len(
                    self.carte):
                self.carte.vider()
                self.carte.ajouter(self.pose, points, t)
            return
        distance, angle = self.carte.distance_derniere(self.pose)
        if distance >= self.distance_image_cle or angle >= self.angle_image_cle:
            self.carte.ajouter(self.pose, points, t)

    # ------------------------------------------------------------------ sorties
    def _sortie(self, t: float) -> Odometrie:
        return Odometrie(t=t, pose=Pose2D(self.pose.x, self.pose.y, self.pose.cap),
                         vitesse=self.vitesse, vitesse_lacet=self.vitesse_lacet,
                         covariance=self.covariance.copy(), qualite=self.qualite,
                         source=self.source)

    def courant(self, t: float) -> Odometrie:
        """Pose extrapolée à l'instant t (entre deux tours : gyroscope et vitesse mesurée)."""
        if self._t is None:
            return Odometrie(t=t, pose=Pose2D(), covariance=self.covariance.copy(), qualite=0.0,
                             source="aucune")
        dt = min(max(t - self._t, 0.0), 0.5)
        if dt <= 0:
            return self._sortie(t)
        dtheta = self._rotation_gyro(self._t, self._t + dt)
        if dtheta is None:
            dtheta = self.vitesse_lacet * dt
        pose = self.pose.composer(arc(self.vitesse * dt, dtheta))
        return Odometrie(t=t, pose=pose, vitesse=self.vitesse,
                         vitesse_lacet=dtheta / dt, covariance=self.covariance.copy(),
                         qualite=self.qualite, source=self.source + "+extrapolee")
