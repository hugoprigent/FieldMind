"""Différentiel électronique : vitesse de chaque roue arrière en virage.

Le rover n'a pas de différentiel mécanique : chaque roue arrière a son moteur. En virage, la
roue extérieure doit aller plus vite que l'intérieure, sinon une roue patine, le rover tourne
mal (sous-virage) et les moteurs forcent. Quatre étages, du plus simple au plus précis :

1. **Géométrie Ackermann** sur l'**angle réel** des roues avant : le servo tourne à vitesse
   limitée, on suit donc sa position (télémétrie de l'ESP) et non la consigne. Rayon de l'essieu
   arrière R = L / tan δ ; roues à R ∓ voie/2, donc v_g,d = v·(1 ∓ κ·voie/2), κ = tan δ / L.
2. **Boucle de lacet** (gyroscope, 50 Hz avec ROVER_3) : le rover devrait tourner à
   ω = v·κ ; s'il tourne moins (herbe, roue avant qui glisse), la roue extérieure accélère et
   l'intérieure ralentit ; s'il tourne trop, l'inverse. En ligne droite, la même boucle compense
   deux moteurs qui ne tournent pas tout à fait pareil. Correction bornée.
3. **Équilibrage par roue** : gain PWM → vitesse propre à chaque moteur (`traction.equilibre_d`).
4. **Capteurs de vitesse des roues** (sorties SC des drivers, firmware `roues 1`), s'ils sont
   câblés : régulation de chaque roue sur sa vitesse mesurée.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from fieldmind.config import Profil, braquage_max
from fieldmind.types import EtatEsp, Imu, Odometrie

CENTRALE_RAPIDE_HZ = 15.0
"""Sous cette cadence (firmware ROVER_2 : 2 Hz), pas de boucle de lacet."""


@dataclass(slots=True)
class Repartition:
    v_g: float  # m/s visée, roue gauche
    v_d: float
    braquage_reel: float  # rad
    lacet_attendu: float  # rad/s
    lacet_mesure: float | None
    correction: float  # m/s ajoutés à droite et retirés à gauche par la boucle de lacet
    mode: str  # "geometrie" | "lacet" | "roues"


class Differentiel:
    def __init__(self, profil: Profil) -> None:
        self.profil = profil
        self._lacet_filtre: float | None = None
        self._dernier_imu = 0.0
        self.frequence_imu = 0.0
        self._integrale = 0.0
        self._integrales_roues = [0.0, 0.0]

    # ------------------------------------------------------------------ entrées

    def imu(self, imu: Imu) -> None:
        """À chaque mesure de la centrale : filtre du lacet et cadence réelle."""
        dt = imu.t - self._dernier_imu if self._dernier_imu else 0.0
        self._dernier_imu = imu.t
        if 0 < dt < 1.0:
            self.frequence_imu = 0.9 * self.frequence_imu + 0.1 / dt
            alpha = min(1.0, dt / 0.08)  # filtre du premier ordre, 80 ms
            w = imu.gyro_rad_s[2]
            self._lacet_filtre = w if self._lacet_filtre is None else \
                self._lacet_filtre + alpha * (w - self._lacet_filtre)

    def braquage_reel(self, esp: EtatEsp | None, braquage_consigne: float, t: float) -> float:
        """Angle des roues avant d'après la position du servo rapportée par l'ESP."""
        p = self.profil
        maximum = braquage_max(p)
        if esp is None or not esp.connecte or not esp.servo_actif or t - esp.t > 0.3:
            return max(-maximum, min(maximum, braquage_consigne))
        neutre = float(p.valeur("direction.servo_neutre_deg", 0.0))
        rapport = max(float(p.valeur("direction.servo_par_degre_roue", 1.0)), 1e-3)
        return max(-maximum, min(maximum, math.radians((esp.servo_deg - neutre) / rapport)))

    # ------------------------------------------------------------------ répartition

    def repartir(self, v: float, braquage_consigne: float, esp: EtatEsp | None,
                 odom: Odometrie | None, t: float, dt: float) -> Repartition:
        p = self.profil
        delta = self.braquage_reel(esp, braquage_consigne, t)
        kappa = math.tan(delta) / float(p.geometrie.empattement_m)
        demi_voie = float(p.geometrie.voie_arriere_m) / 2
        if not bool(p.valeur("traction.differentiel", True)):
            return Repartition(v, v, delta, v * kappa, self._lacet_filtre, 0.0, "aucun")
        v_g, v_d = v * (1 - kappa * demi_voie), v * (1 + kappa * demi_voie)
        # Vitesse réelle si l'odométrie lidar est fiable : le lacet attendu en dépend
        v_reelle = v
        if odom is not None and odom.qualite >= 0.6 and t - odom.t < 0.3 and odom.vitesse * v > 0:
            v_reelle = odom.vitesse
        attendu = v_reelle * kappa
        mesure = self._lacet_filtre
        correction, mode = 0.0, "geometrie"
        lacet_actif = (bool(p.valeur("traction.boucle_lacet", True)) and mesure is not None
                       and self.frequence_imu >= CENTRALE_RAPIDE_HZ and t - self._dernier_imu < 0.15
                       and abs(v) >= 0.08)
        if lacet_actif and mesure is not None:
            erreur = attendu - mesure
            kp = float(p.valeur("traction.lacet_kp", 0.8))
            ki = float(p.valeur("traction.lacet_ki", 0.6))
            self._integrale += erreur * dt
            borne = min(0.25 * abs(v), float(p.valeur("traction.lacet_correction_max_m_s", 0.10)))
            if ki > 0:  # anti-emballement : l'intégrale ne dépasse pas la borne à elle seule
                self._integrale = max(-borne / (ki * demi_voie), min(borne / (ki * demi_voie),
                                                                     self._integrale))
            correction = (kp * erreur + ki * self._integrale) * demi_voie
            correction = max(-borne, min(borne, correction))
            v_g, v_d = v_g - correction, v_d + correction
            mode = "lacet"
        else:
            self._integrale *= 0.9  # rover arrêté ou centrale lente : on oublie doucement
        # La roue intérieure ne doit pas changer de sens (pas de demi-tour sur place)
        if v > 0:
            v_g, v_d = max(v_g, 0.0), max(v_d, 0.0)
        elif v < 0:
            v_g, v_d = min(v_g, 0.0), min(v_d, 0.0)
        return Repartition(v_g, v_d, delta, attendu, mesure, correction, mode)

    # ------------------------------------------------------------------ PWM

    def pwm(self, r: Repartition, esp: EtatEsp | None, correction_commune: float,
            dt: float) -> tuple[int, int]:
        """PWM signés de chaque roue : seuil + gain·|v| (+ équilibrage, + régulation par roue si
        les capteurs de vitesse existent)."""
        p = self.profil
        gain = float(p.valeur("traction.pwm_par_m_s", 35.0))
        seuil = float(p.valeur("traction.pwm_seuil", 4))
        equilibre = (1.0, float(p.valeur("traction.equilibre_d", 1.0)))
        plafond = int(p.valeur("traction.pwm_max", 40))
        mesurees = self._vitesses_roues(esp)
        sortie = []
        for i, v in enumerate((r.v_g, r.v_d)):
            if abs(v) < 0.01:
                self._integrales_roues[i] = 0.0
                sortie.append(0)
                continue
            brut = seuil + abs(v) * gain * equilibre[i] + correction_commune
            if mesurees is not None:
                erreur = abs(v) - abs(mesurees[i]) if mesurees[i] * v >= 0 else abs(v)
                self._integrales_roues[i] = max(-10.0, min(10.0, self._integrales_roues[i]
                                                           + 25.0 * erreur * dt))
                brut += 15.0 * erreur + self._integrales_roues[i]
                r.mode = "roues"
            valeur = round(min(max(brut, 0.0), plafond))
            sortie.append(valeur if v > 0 else -valeur)
        return sortie[0], sortie[1]

    def _vitesses_roues(self, esp: EtatEsp | None) -> tuple[float, float] | None:
        if esp is None or esp.hz_g is None or esp.hz_d is None or not esp.connecte:
            return None
        par_metre = float(self.profil.valeur("roues.impulsions_par_metre", 0.0))
        if par_metre <= 0:
            return None
        return esp.hz_g / par_metre, esp.hz_d / par_metre
