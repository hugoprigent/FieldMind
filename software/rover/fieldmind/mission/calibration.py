"""Calibrations mesurées par le rover lui-même (odométrie lidar), lancées depuis le poste.

- **vitesse** (mode auto, 2,5 m libres devant) : trois paliers de vitesse ; on relève le PWM
  envoyé et la vitesse mesurée, puis on ajuste `traction.pwm_seuil` et `traction.pwm_par_m_s`.
- **braquage** (mode auto, 1,5 m libres) : deux arcs à ±15° ; la courbure mesurée donne l'angle
  réel des roues, d'où `direction.servo_par_degre_roue` et `direction.servo_neutre_deg`.
- **imu_repos** (rover immobile) : gravité et biais du gyroscope ; vérifie les axes du profil.
- **sens_roues** (mode banc, roues levées) : chaque roue tourne à 8 % pendant 1,5 s, l'une après
  l'autre ; l'opérateur regarde et corrige avec `inverseg` / `inversed` dans la console.

Les résultats sont enregistrés par `Profil.regler` (bornés) et annoncés sur le sujet
`calibration` du bus (message `calibration` du poste).
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from fieldmind.bus import Bus, S
from fieldmind.mission.actions import ActionRefusee, Contexte, Pas
from fieldmind.types import Action, CommandeRoues, Consigne, Imu

SUJET = "calibration"


def annoncer(bus: Bus, nom: str, etat: str, message: str,
             resultat: dict[str, Any] | None = None) -> None:
    bus.publier(SUJET, {"type": "calibration", "nom": nom, "etat": etat, "message": message,
                        "resultat": resultat})
    bus.journaliser("calibration", f"{nom} : {message}",
                    {"termine": "succes", "echec": "alerte"}.get(etat, "info"))


@dataclass
class _Palier:
    vitesse: float
    braquage: float = 0.0
    duree: float = 2.0
    mesure_depuis: float = 1.0  # régime établi après ce délai
    vitesses: list[float] = field(default_factory=list)
    lacets: list[float] = field(default_factory=list)
    pwm: list[float] = field(default_factory=list)
    impulsions: list[float] = field(default_factory=list)  # (hz_g + hz_d) / 2 si capteurs SC
    servo: list[float] = field(default_factory=list)


class CalibrationMouvement:
    """Enchaîne des paliers (vitesse, braquage), mesure, puis calcule le réglage."""

    def __init__(self, nom: str, executeur: Any, paliers: list[_Palier], libre_min: float) -> None:
        self.nom = nom
        self.executeur = executeur
        self.bus: Bus = executeur.bus
        self.paliers = paliers
        self.libre_min = libre_min
        self.indice = 0
        self.debut: float | None = None
        self.action: Action = {"type": "calibration", "nom": nom}
        self._annonce = False

    def pas(self, ctx: Contexte) -> Pas:
        conduite = self.executeur.conduite
        if not self._annonce:
            self._annonce = True
            libre = ctx.securite.libre_avant_m if ctx.securite else None
            if libre is None or libre < self.libre_min:
                annoncer(self.bus, self.nom, "echec", f"il faut {self.libre_min:.1f} m libres "
                         "devant le rover")
                return Pas(etat="echec", raison="pas assez de place devant")
            conduite.figer_correction = True
            annoncer(self.bus, self.nom, "en_cours", "mesure en cours")
        if self.indice >= len(self.paliers):
            conduite.figer_correction = False
            try:
                resultat = self.calculer()
            except ValueError as erreur:
                annoncer(self.bus, self.nom, "echec", str(erreur))
                return Pas(etat="echec", raison=str(erreur))
            annoncer(self.bus, self.nom, "termine", "réglage enregistré", resultat)
            return Pas(etat="termine", progression=1.0)
        palier = self.paliers[self.indice]
        if self.debut is None:
            self.debut = ctx.t
        ecoule = ctx.t - self.debut
        if ecoule >= palier.duree:
            self.indice += 1
            self.debut = None
            return Pas(vitesse=0.0, braquage=palier.braquage, detail="palier suivant")
        if ecoule >= palier.mesure_depuis and ctx.odom.qualite >= 0.5:
            commande: CommandeRoues | None = self.bus.dernier(S.COMMANDE)
            if commande is not None and abs(commande.vitesse - palier.vitesse) < 0.01:
                palier.vitesses.append(ctx.odom.vitesse)
                palier.lacets.append(ctx.odom.vitesse_lacet)
                palier.pwm.append(0.5 * (abs(commande.pwm_g) + abs(commande.pwm_d)))
                esp = self.bus.dernier(S.ESP)
                if esp is not None and esp.hz_g is not None and esp.hz_d is not None:
                    palier.impulsions.append(0.5 * (abs(esp.hz_g) + abs(esp.hz_d)))
                palier.servo.append(commande.servo_deg)
        progression = (self.indice + min(1.0, ecoule / palier.duree)) / len(self.paliers)
        return Pas(vitesse=palier.vitesse, braquage=palier.braquage, progression=progression,
                   detail=f"palier {self.indice + 1}/{len(self.paliers)}")

    def calculer(self) -> dict[str, Any]:
        raise NotImplementedError


class CalibrationVitesse(CalibrationMouvement):
    def __init__(self, executeur: Any) -> None:
        super().__init__("vitesse", executeur, [_Palier(0.15), _Palier(0.25), _Palier(0.35)], 2.5)

    def calculer(self) -> dict[str, Any]:
        mesures = [(float(np.median(p.vitesses)), float(np.median(p.pwm)))
                   for p in self.paliers if len(p.vitesses) >= 5]
        if len(mesures) < 2 or max(v for v, _ in mesures) < 0.05:
            raise ValueError("vitesse non mesurée (le rover a-t-il roulé ? lidar dégagé ?)")
        v = np.array([m[0] for m in mesures])
        pwm = np.array([m[1] for m in mesures])
        pente, origine = np.polyfit(v, pwm, 1)
        profil = self.executeur.profil
        resultat = {"pwm_par_m_s": profil.regler("traction.pwm_par_m_s", float(pente)),
                    "pwm_seuil": profil.regler("traction.pwm_seuil", float(origine)),
                    "mesures": [{"vitesse_m_s": round(a, 3), "pwm": round(b, 1)}
                                for a, b in mesures]}
        # Capteurs de roue (SC) : impulsions par mètre, d'après la vitesse mesurée au lidar
        rapports = [float(np.median(p.impulsions)) / float(np.median(p.vitesses))
                    for p in self.paliers if len(p.impulsions) >= 5 and len(p.vitesses) >= 5
                    and float(np.median(p.vitesses)) > 0.08]
        if rapports:
            resultat["impulsions_par_metre"] = profil.regler("roues.impulsions_par_metre",
                                                             float(np.median(rapports)))
        return resultat


class CalibrationBraquage(CalibrationMouvement):
    ANGLE = math.radians(15)

    def __init__(self, executeur: Any) -> None:
        super().__init__("braquage", executeur, [
            _Palier(0.25, +self.ANGLE, duree=2.2, mesure_depuis=1.2),
            _Palier(0.25, -self.ANGLE, duree=2.2, mesure_depuis=1.2),
        ], 1.5)

    def calculer(self) -> dict[str, Any]:
        profil = self.executeur.profil
        empattement = float(profil.geometrie.empattement_m)
        reels, servos = [], []
        for p in self.paliers:
            if len(p.vitesses) < 5:
                raise ValueError("courbure non mesurée")
            v = float(np.median(p.vitesses))
            if v < 0.05:
                raise ValueError("le rover n'a pas roulé")
            courbure = float(np.median(p.lacets)) / v
            reels.append(math.degrees(math.atan(courbure * empattement)))
            servos.append(float(np.median(p.servo)))
        if abs(reels[0] - reels[1]) < 2:
            raise ValueError("la direction ne répond pas (servo ? tringlerie ?)")
        pente = (servos[0] - servos[1]) / (reels[0] - reels[1])
        neutre = servos[0] - pente * reels[0]
        return {"servo_par_degre_roue": profil.regler("direction.servo_par_degre_roue", pente),
                "servo_neutre_deg": profil.regler("direction.servo_neutre_deg", neutre),
                "braquages_mesures_deg": [round(r, 1) for r in reels]}


@dataclass
class CalibrationImu:
    executeur: Any
    action: Action = field(default_factory=lambda: {"type": "calibration", "nom": "imu_repos"})
    debut: float | None = None
    accel: list[tuple[float, float, float]] = field(default_factory=list)
    gyro: list[tuple[float, float, float]] = field(default_factory=list)
    _dernier: float = 0.0

    def pas(self, ctx: Contexte) -> Pas:
        bus = self.executeur.bus
        if self.debut is None:
            self.debut = ctx.t
            annoncer(bus, "imu_repos", "en_cours", "ne pas toucher le rover (3 s)")
        imu: Imu | None = bus.dernier(S.IMU)
        if imu is not None and imu.t != self._dernier:
            self._dernier = imu.t
            self.accel.append(imu.accel_g)
            self.gyro.append(imu.gyro_rad_s)
        if ctx.t - self.debut < 3.0:
            return Pas(progression=(ctx.t - self.debut) / 3, detail="mesure au repos")
        if len(self.accel) < 3:
            annoncer(bus, "imu_repos", "echec", "centrale muette")
            return Pas(etat="echec", raison="centrale muette")
        a = np.mean(np.array(self.accel), axis=0)
        w = np.mean(np.array(self.gyro), axis=0)
        bruit = float(np.std(np.array(self.gyro)[:, 2]))
        message = "axes cohérents" if a[2] > 0.9 else (
            f"gravité mesurée sur {a.round(2).tolist()} : revoir imu.axe_x/axe_y/axe_z")
        esp = self.executeur.conduite._esp()
        if esp is not None and esp.rover3:
            esp.envoyer_ligne("lacet0", journal=False)
        annoncer(bus, "imu_repos", "termine" if a[2] > 0.9 else "echec", message,
                 {"gravite_g": a.round(3).tolist(), "biais_gyro_deg_s":
                  np.degrees(w).round(3).tolist(), "bruit_lacet_deg_s": round(
                      math.degrees(bruit), 3)})
        return Pas(etat="termine", progression=1.0)


def creer_calibration(action: Action, executeur: Any) -> Any:
    nom = action["nom"]
    if nom == "vitesse":
        return CalibrationVitesse(executeur)
    if nom == "braquage":
        return CalibrationBraquage(executeur)
    if nom == "imu_repos":
        return CalibrationImu(executeur)
    raise ActionRefusee(f"la calibration {nom} ne se lance pas en mode auto")


def sens_roues(bus: Bus, conduite: Any) -> None:
    """Mode banc, roues levées : gauche puis droite à +8 % pendant 1,5 s chacune."""
    if conduite.mode != "banc" or not conduite.arme:
        annoncer(bus, "sens_roues", "echec", "passer en mode banc (roues levées) et armer")
        return

    def jouer() -> None:
        annoncer(bus, "sens_roues", "en_cours", "roue gauche en avant")
        for nom, pwm in (("gauche", (8, 0)), ("droite", (0, 8))):
            annoncer(bus, "sens_roues", "en_cours", f"roue {nom} en avant ?")
            fin = time.monotonic() + 1.5
            while time.monotonic() < fin:
                if conduite.mode != "banc" or not conduite.arme:
                    annoncer(bus, "sens_roues", "echec", "interrompu")
                    return
                conduite.proposer(Consigne(t=time.monotonic(), source="banc", pwm_brut=pwm))
                time.sleep(0.1)
            conduite.proposer(Consigne(t=time.monotonic(), source="banc", pwm_brut=(0, 0)))
            time.sleep(1.2)
        annoncer(bus, "sens_roues", "termine", "si une roue a reculé : console « inverseg 1 » "
                 "ou « inversed 1 » (valeur opposée à celle d'« etat »)")

    threading.Thread(target=jouer, name="calibration-sens", daemon=True).start()
