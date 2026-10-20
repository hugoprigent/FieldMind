"""Automate avec retours mesurés ; aucune altitude ni rotation déduite du temps.

Les sorties sont des vitesses, pas des octets RC. Leur conversion exige des axes
calibrés pour le contrôleur réellement installé. Le moteur de mission est commun
au banc et à un futur adaptateur matériel validé.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .modele import Observation, Ordre, angle, borner


@dataclass(frozen=True)
class ReglagesMission:
    hauteur_scan_m: float = 1.0
    palier_m: float = 0.15
    hauteur_pose_m: float = 0.22
    stabilisation_s: float = 0.8
    duree_max_s: float = 120.0

    def __post_init__(self) -> None:
        for v in (
            self.hauteur_scan_m,
            self.palier_m,
            self.hauteur_pose_m,
            self.stabilisation_s,
            self.duree_max_s,
        ):
            if not math.isfinite(v):
                raise ValueError("réglage non fini")
        if not (
            0.6 <= self.hauteur_scan_m <= 1.5
            and 0.05 <= self.palier_m <= 0.2
            and 0.18 <= self.hauteur_pose_m <= 0.3
            and 0.5 <= self.stabilisation_s <= 3
            and 30 <= self.duree_max_s <= 180
        ):
            raise ValueError("réglages de mission hors limites")


class Autopilote:
    def __init__(self, reglages: ReglagesMission | None = None) -> None:
        self.reglages = reglages or ReglagesMission()
        self.phase = "inactif"
        self.raison = "attente"
        self.photos = 0
        self.paliers = 0
        self.rotation_rad = 0.0
        self.hauteur_cible = self.reglages.hauteur_scan_m
        self.journal: list[dict] = []
        self._debut = 0.0
        self._etape = 0.0
        self._stable: float | None = None
        self._seq_stable: int | None = None
        self._seq_precedente = -1
        self._cap_precedent = 0.0
        self._mesure_t = 0.0
        self._cible_perdue: float | None = None
        self._annuler = False
        self._sol_depuis: float | None = None
        self._envol_confirme = False
        self._sol_confirme = True

    def _phase(self, phase: str, t: float, raison: str) -> None:
        self.phase, self.raison = phase, raison
        self._etape, self._stable = t, None
        self._seq_stable = None
        self.journal.append({"t": round(t, 3), "phase": phase, "raison": raison})

    def demarrer(self, o: Observation, maintenant: float) -> Ordre:
        if self.phase not in ("inactif", "termine", "interrompu", "erreur"):
            raise ValueError("une mission drone est déjà en cours")
        if not o.valide(maintenant) or o.au_sol is not True or o.camera != "avant":
            raise ValueError("prévol : sol, cap, hauteur et caméra avant doivent être confirmés")
        if o.batterie_pct is not None and o.batterie_pct < 25:
            raise ValueError("batterie trop faible pour une reconnaissance")
        self.__init__(self.reglages)
        self._debut = self._mesure_t = maintenant
        self._cap_precedent = o.cap_rad
        self._seq_precedente = o.sequence
        self._phase("decollage", maintenant, "Décollage demandé ; attente de montée mesurée")
        return Ordre(action="decoller")

    def annuler(self) -> None:
        self._annuler = True

    def _stable_pendant(self, condition: bool, o: Observation, t: float) -> bool:
        if not condition:
            self._stable = None
            self._seq_stable = None
            return False
        if self._stable is None:
            self._stable, self._seq_stable = t, o.sequence
        return t - self._stable >= self.reglages.stabilisation_s and o.sequence != self._seq_stable

    def _repli(self, t: float, raison: str) -> Ordre:
        if not self._envol_confirme:
            self._phase(
                "interrompu" if self._sol_confirme else "erreur",
                t,
                raison + " ; aucun envol confirmé, état à vérifier",
            )
            return Ordre()
        self._phase("repli", t, raison)
        return Ordre(action="atterrir")

    def pas(self, o: Observation, t: float) -> Ordre:
        self._sol_confirme = o.valide(t) and o.au_sol is True and o.hauteur_m <= 0.06
        if o.valide(t) and o.au_sol is False and o.hauteur_m > 0.08:
            self._envol_confirme = True
        if self.phase in ("inactif", "termine", "interrompu", "erreur"):
            return Ordre()
        if self.phase in ("repli", "atterrissage"):
            # Une seule impulsion de land : certains protocoles utilisent le même
            # bit pour décoller et atterrir. Ne jamais répéter ce basculement.
            if o.valide(t) and o.au_sol is True and o.hauteur_m <= 0.06:
                if self._sol_depuis is None:
                    self._sol_depuis = t
                if t - self._sol_depuis >= 0.6:
                    phase = "termine" if self.phase == "atterrissage" else "interrompu"
                    self._phase(
                        phase,
                        t,
                        "Contact au sol confirmé"
                        if phase == "termine"
                        else self.raison + " ; retour au sol confirmé",
                    )
            else:
                self._sol_depuis = None
            if t - self._etape > 15:
                self._phase("erreur", t, "Posé non confirmé : reprise manuelle nécessaire")
            return Ordre()
        if self._annuler:
            return self._repli(t, "Mission interrompue par l'opérateur")
        if t - self._debut > self.reglages.duree_max_s:
            return self._repli(t, "Durée maximale de mission atteinte")
        if not o.valide(t):
            return self._repli(t, "Estimation de vol périmée ou absente")
        if o.sequence < self._seq_precedente:
            return self._repli(t, "Ordre des mesures incohérent")
        nouvelle = o.sequence > self._seq_precedente
        if nouvelle:
            dt = o.t - self._mesure_t
            delta = angle(o.cap_rad - self._cap_precedent)
            if dt <= 0 or abs(delta) > 1.5 * dt + 0.08:
                return self._repli(t, "Saut de cap incohérent")
            if self.phase == "rotation":
                self.rotation_rad += delta
            self._cap_precedent, self._mesure_t = o.cap_rad, o.t
            self._seq_precedente = o.sequence
        if o.batterie_pct is not None and o.batterie_pct < 15:
            return self._repli(t, "Batterie faible")
        if o.image_t is None or not 0 <= t - o.image_t <= 0.6:
            return self._repli(t, "Vidéo périmée : aucune poursuite de reconnaissance")
        h = o.hauteur_m
        vz = borner(1.4 * (self.hauteur_cible - h) - 0.4 * o.vitesse_z, 0.22)
        if self.phase == "decollage":
            if t - self._etape > 12:
                return self._repli(t, "Décollage ou altitude non confirmé")
            if nouvelle and self._stable_pendant(
                o.au_sol is False
                and abs(h - self.hauteur_cible) < 0.05
                and abs(o.vitesse_z) < 0.06,
                o,
                t,
            ):
                self._phase("rotation", t, "Altitude stable ; rotation par secteurs de 60°")
            return Ordre(montee_m_s=vz)
        if self.phase == "rotation":
            if t - self._etape > 65:
                return self._repli(t, "Tour complet non confirmé dans le délai")
            cible_angle = self.photos * math.pi / 3
            erreur_angle = cible_angle - self.rotation_rad
            lacet = borner(1.8 * erreur_angle - 0.3 * o.vitesse_lacet, 0.6)
            stable = (
                abs(erreur_angle) < 0.035
                and abs(o.vitesse_lacet) < 0.07
                and abs(h - self.hauteur_cible) < 0.06
                and o.camera == "avant"
            )
            if nouvelle and self._stable_pendant(stable, o, t):
                self._stable = None
                if self.photos < 6:
                    self.photos += 1
                    return Ordre(montee_m_s=vz, photo=self.photos)
                self._phase("camera_bas", t, "Tour mesuré terminé ; caméra du bas demandée")
                return Ordre(montee_m_s=vz, action="camera_bas")
            return Ordre(montee_m_s=vz, lacet_rad_s=lacet)
        if self.phase == "camera_bas":
            if o.camera == "bas" and o.image_t > self._etape:
                self._phase("recentrage", t, "Nouvelle image de la caméra du bas confirmée")
            elif t - self._etape > 5:
                return self._repli(t, "Caméra du bas non confirmée")
            return Ordre(montee_m_s=vz)
        if self.phase in ("recentrage", "descente"):
            cible = o.cible
            if o.camera != "bas" or cible is None or not cible.valide(t):
                self._stable = None
                if self._cible_perdue is None:
                    self._cible_perdue = t
                if self.phase == "descente":
                    self.hauteur_cible = h  # interrompre la descente dès la perte de cible
                    self._phase("recentrage", t, "Croix perdue : descente suspendue")
                if t - self._cible_perdue > 2:
                    return self._repli(t, "Croix perdue durablement : atterrissage de repli")
                return Ordre()  # vitesse verticale demandée nulle, jamais une nouvelle descente
            self._cible_perdue = None
            # Refuser une estimation visuelle d'altitude incompatible avec le retour local.
            if abs(cible.hauteur_m - h) > max(0.15, h * 0.3):
                return self._repli(t, "Hauteurs visuelle et locale incompatibles")
            distance = math.hypot(cible.avant_m, cible.droite_m)
            tolerance = max(0.015, h * 0.035)
            avant = borner(cible.avant_m * 0.9, 0.12)
            droite = borner(cible.droite_m * 0.9, 0.12)
            if self.phase == "descente" and distance > tolerance * 1.8:
                self.hauteur_cible = h
                self._phase("recentrage", t, "Dérive : stabiliser et recentrer avant de descendre")
                return Ordre(avant_m_s=avant, droite_m_s=droite)
            if self.phase == "recentrage":
                vz = borner(1.4 * (self.hauteur_cible - h) - 0.4 * o.vitesse_z, 0.15)
                if nouvelle and self._stable_pendant(
                    distance < tolerance and abs(o.vitesse_z) < 0.05,
                    o,
                    t,
                ):
                    if h <= self.reglages.hauteur_pose_m + 0.035:
                        self._phase("atterrissage", t, "Centré à basse hauteur ; posé demandé")
                        return Ordre(action="atterrir")
                    self.hauteur_cible = max(
                        self.reglages.hauteur_pose_m, h - self.reglages.palier_m
                    )
                    self.paliers += 1
                    self._phase("descente", t, f"Descendre au palier {self.hauteur_cible:.2f} m")
            elif abs(h - self.hauteur_cible) < 0.025 and abs(o.vitesse_z) < 0.045:
                self._phase("recentrage", t, "Palier atteint ; vérifier le centrage")
            return Ordre(montee_m_s=vz, avant_m_s=avant, droite_m_s=droite)
        return self._repli(t, "État de mission inconnu")

    def etat(self) -> dict:
        return {
            "phase": self.phase,
            "raison": self.raison,
            "photos": self.photos,
            "paliers": self.paliers,
            "rotation_deg": round(math.degrees(self.rotation_rad), 1),
            "hauteur_cible_m": round(self.hauteur_cible, 3),
        }
