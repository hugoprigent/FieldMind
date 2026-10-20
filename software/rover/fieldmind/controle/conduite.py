"""Conduite : boucle à 20 Hz qui transforme la consigne active en commandes de l'ESP.

    consigne (poste, mission, ROS 2, banc) → sécurité → rampes → différentiel → PWM + servo → ESP

- **Mode** : `arret` au démarrage, désarmé. `manuel` prend la consigne du poste, `auto` celle de
  l'exécuteur de mission (sinon celle de ROS 2), `banc` des pourcentages bruts.
- **Rampes** : accélération douce, décélération plus franche ; un arrêt de la sécurité coupe la
  traction tout de suite (les moteurs sont en roue libre, sans frein).
- **Différentiel électronique** : en virage, la roue arrière intérieure tourne moins vite
  (v·(1 ∓ κ·voie/2)), comme sur un vrai essieu à différentiel.
- **PWM** : pwm = seuil + |v|·(pwm par m/s), corrigé lentement par la vitesse mesurée (odométrie
  lidar) quand elle est fiable ; plafond `traction.pwm_max`.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import asdict

from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.controle.differentiel import Differentiel, Repartition
from fieldmind.materiel.esp import PiloteEsp
from fieldmind.securite.superviseur import Superviseur
from fieldmind.types import CommandeRoues, Consigne, EtatEsp, EtatSecurite, Mode, Odometrie

PERIODE_S = 0.05
CONSIGNE_FRAICHE_S = 0.5


class Conduite:
    def __init__(self, bus: Bus, profil: Profil, esp: Callable[[], PiloteEsp | None],
                 superviseur: Superviseur) -> None:
        self.bus, self.profil = bus, profil
        self._esp = esp
        self.superviseur = superviseur
        self.mode: Mode = "arret"
        self.arme = False
        self.pilote: str | None = None  # poste qui a la main en manuel
        self._consignes: dict[str, Consigne] = {}
        self._dernier_retard_signale = -math.inf
        self._verrou = threading.Lock()
        self._vitesse = 0.0  # après rampe
        self._correction = 0.0  # correction intégrale du PWM (%)
        self.figer_correction = False  # pendant une calibration de vitesse
        self._derniere = CommandeRoues(t=time.monotonic())
        self._actif = False
        self._fil = threading.Thread(target=self._boucle, name="conduite", daemon=True)
        self.securite = EtatSecurite(t=time.monotonic())
        self.differentiel = Differentiel(profil)
        self.repartition: Repartition | None = None
        bus.abonner(S.IMU, self.differentiel.imu)

    # ------------------------------------------------------------------ commandes

    def demarrer(self) -> None:
        self._actif = True
        self._fil.start()
        self._publier_mode()

    def arreter(self) -> None:
        self._actif = False
        esp = self._esp()
        if esp:
            esp.arret_traction()

    def proposer(self, consigne: Consigne) -> None:
        """Dépose la consigne d'une source (manuel, auto, ros2, banc)."""
        with self._verrou:
            self._consignes[consigne.source] = consigne

    def proposer_arret(self) -> None:
        """Arrêt tout de suite (« stop » dit ou écrit), sans verrou : toutes les consignes à
        zéro et traction coupée sans rampe. Le rover reste armé."""
        t = time.monotonic()
        with self._verrou:
            for source in list(self._consignes):
                self._consignes[source] = Consigne(t=t, source=self._consignes[source].source)
            self._vitesse = 0.0
        esp = self._esp()
        if esp:
            esp.arret_traction()

    def changer_mode(self, mode: Mode, confirmer: bool = False) -> str | None:
        """Renvoie un message d'erreur, ou None."""
        if mode not in ("arret", "manuel", "auto", "banc"):
            return "mode inconnu"
        if mode == "banc" and not confirmer:
            return "le mode banc demande une confirmation (roues levées)"
        if mode == self.mode:
            return None
        with self._verrou:
            self._consignes.clear()
            self._vitesse = 0.0
            self._correction = 0.0
            ancien, self.mode = self.mode, mode
            if mode == "arret":
                self.arme = False
        self.bus.journaliser("conduite", f"mode {ancien} → {mode}")
        self._publier_mode()
        return None

    def armer(self) -> str | None:
        if self.superviseur.arret_urgence:
            return "arrêt d'urgence actif : réarmer d'abord"
        esp = self._esp()
        if esp is None or not esp.connecte():
            return "ESP absent"
        if self.mode == "arret":
            return "choisir un mode (manuel, auto ou banc) avant d'armer"
        self.arme = True
        self.bus.journaliser("conduite", "armé : traction autorisée", "succes")
        self._publier_mode()
        return None

    def desarmer(self, raison: str = "operateur") -> None:
        if self.arme:
            self.bus.journaliser("conduite", f"désarmé ({raison})")
        self.arme = False
        with self._verrou:
            self._consignes.clear()
            self._vitesse = 0.0
        self._publier_mode()

    def arret_urgence(self, raison: str = "operateur") -> None:
        self.superviseur.declencher(raison)
        self.arme = False
        with self._verrou:
            self._consignes.clear()
            self._vitesse = 0.0
        esp = self._esp()
        if esp:
            esp.arret_traction()
        self._publier_mode()

    def rearmer(self) -> None:
        self.superviseur.rearmer()
        self._publier_mode()

    def etat_mode(self) -> dict[str, object]:
        return {"mode": self.mode, "arme": self.arme,
                "arret_urgence": self.superviseur.arret_urgence,
                "raison_urgence": self.superviseur.raison_urgence, "pilote": self.pilote}

    def _publier_mode(self) -> None:
        self.bus.publier(S.MODE, self.etat_mode())

    # ------------------------------------------------------------------ boucle

    def _boucle(self) -> None:
        prochaine = time.monotonic()
        while self._actif:
            try:
                self._cycle(time.monotonic())
            except Exception as erreur:  # la boucle de conduite ne doit jamais mourir
                self.bus.journaliser("conduite", f"erreur : {erreur!r}", "erreur")
                esp = self._esp()
                if esp:
                    esp.arret_traction()
            prochaine += PERIODE_S
            attente = prochaine - time.monotonic()
            if attente > 0:
                time.sleep(attente)
            else:
                self._signaler_retard(-attente)
                prochaine = time.monotonic()

    def _signaler_retard(self, retard: float) -> None:
        """Un tour en retard de plus de 200 ms approche le chien de garde de l'ESP (400 ms)."""
        maintenant = time.monotonic()
        if retard < 0.2 or maintenant - self._dernier_retard_signale < 30.0:
            return
        self._dernier_retard_signale = maintenant
        self.bus.journaliser("conduite", f"boucle de conduite en retard de {retard * 1000:.0f} ms "
                             "(processeur du Pi saturé ou bridé ?)", "alerte")

    def consigne_courante(self) -> Consigne:
        """Consigne demandée en ce moment (avant sécurité), pour l'affichage."""
        return self._consigne_active(time.monotonic())

    def _consigne_active(self, t: float) -> Consigne:
        with self._verrou:
            consignes = dict(self._consignes)
        if self.mode == "manuel":
            manuel = consignes.get("manuel")
            if manuel is not None and t - manuel.t < CONSIGNE_FRAICHE_S:
                return manuel
            return Consigne(t=t, source="manuel")  # homme mort : plus de nouvelles du poste
        if self.mode == "auto":
            auto = consignes.get("auto")
            if auto is not None and t - auto.t < CONSIGNE_FRAICHE_S:
                return auto
            ros = consignes.get("ros2")
            if ros is not None and t - ros.t < CONSIGNE_FRAICHE_S:
                return ros
            return Consigne(t=t, source="auto")
        if self.mode == "banc":
            banc = consignes.get("banc")
            if banc is not None and t - banc.t < CONSIGNE_FRAICHE_S:
                return banc
            return Consigne(t=t, source="banc", pwm_brut=(0, 0))
        return Consigne(t=t, source="aucune")

    def _cycle(self, t: float) -> None:
        demande = self._consigne_active(t)
        limitee, securite = self.superviseur.evaluer(demande, self.mode, self.arme, t)
        self.securite = securite
        if securite.arret_urgence and self.arme:
            self.arme = False
            self._publier_mode()
        commande = self._commande_roues(limitee, securite, t)
        esp = self._esp()
        if esp is not None:
            esp.appliquer(commande)
        self._derniere = commande
        self.bus.publier(S.COMMANDE, commande)
        self.bus.publier(S.SECURITE, securite)

    def _commande_roues(self, c: Consigne, securite: EtatSecurite, t: float) -> CommandeRoues:
        p = self.profil
        servo = self._servo(c.braquage)
        if c.pwm_brut is not None:  # banc
            self._vitesse = 0.0
            return CommandeRoues(t=t, pwm_g=c.pwm_brut[0], pwm_d=c.pwm_brut[1], servo_deg=servo,
                                 braquage=c.braquage)
        # Rampe à l'accélération seulement. Les moteurs sont en roue libre (pas de frein) :
        # baisser le PWM ne freine pas plus fort, donc toute baisse (sécurité, arrivée, arrêt)
        # s'applique tout de suite.
        cible = c.vitesse
        if securite.arret or cible == 0 or cible * self._vitesse < 0 or \
                abs(cible) <= abs(self._vitesse):
            self._vitesse = cible if cible * self._vitesse >= 0 else 0.0
        else:
            pas = float(p.valeur("traction.rampe_acceleration_m_s2", 0.6)) * PERIODE_S
            self._vitesse += max(-pas, min(pas, cible - self._vitesse))
        # Un changement de sens passe par zéro (le firmware attend ensuite l'arrêt de la roue)
        if cible * self._vitesse < 0:
            self._vitesse = 0.0
        v = self._vitesse
        if v == 0:
            self._correction *= 0.9
            return CommandeRoues(t=t, pwm_g=0, pwm_d=0, servo_deg=servo, vitesse=0.0,
                                 braquage=c.braquage)
        self._corriger(v, stable=abs(cible - v) < 1e-6)
        esp: EtatEsp | None = self.bus.dernier(S.ESP)
        odom: Odometrie | None = self.bus.dernier(S.ODOM)
        r = self.differentiel.repartir(v, c.braquage, esp, odom, t, PERIODE_S)
        self.repartition = r
        pwm_g, pwm_d = self.differentiel.pwm(r, esp, self._correction, PERIODE_S)
        return CommandeRoues(t=t, pwm_g=pwm_g, pwm_d=pwm_d, servo_deg=servo, vitesse=v,
                             braquage=c.braquage)

    def _corriger(self, v: float, stable: bool) -> None:
        """Correction intégrale lente du PWM d'après la vitesse mesurée (lidar), bornée à ±8 %."""
        odom: Odometrie | None = self.bus.dernier(S.ODOM)
        if odom is None or odom.qualite < 0.6 or self.bus.age(S.ODOM) > 0.3:
            return
        if self.figer_correction:
            self._correction = 0.0
            return
        if abs(v) < 0.08 or not stable:
            return
        erreur = abs(v) - abs(odom.vitesse) if odom.vitesse * v >= 0 else abs(v)
        self._correction = max(-8.0, min(8.0, self._correction + 6.0 * erreur * PERIODE_S))

    def _servo(self, braquage: float) -> float:
        p = self.profil
        angle = (float(p.valeur("direction.servo_neutre_deg", 0.0))
                 + float(p.valeur("direction.servo_par_degre_roue", 1.0)) * math.degrees(braquage))
        maximum = float(p.valeur("direction.servo_max_deg", 40.0))
        return round(max(-maximum, min(maximum, angle)), 1)

    def resume(self) -> dict[str, object]:
        d = asdict(self._derniere)
        d.pop("t", None)
        r = self.repartition
        if r is not None and self._derniere.vitesse != 0:
            d["differentiel"] = {"v_g": r.v_g, "v_d": r.v_d, "braquage_reel": r.braquage_reel,
                                 "lacet_attendu": r.lacet_attendu, "lacet_mesure": r.lacet_mesure,
                                 "correction": r.correction, "mode": r.mode}
        return d
