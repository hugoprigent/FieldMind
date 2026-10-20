"""Banc d'essai hors ligne : le cerveau pilote le rover simulé plus vite que le temps réel.

Le cerveau est exactement celui de la démo ; seule l'exécution change : les messages lui sont
remis dans l'ordre, sans réseau, et le flux lent (carte, chemin) est traité à chaque balayage.
La vérité terrain sert uniquement à mesurer (erreur de localisation, collisions, réussite).
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Literal

from services.cerveau.cerveau import Cerveau
from services.cerveau.evenements import EvenementCerveau
from services.perception.vision import DemandeVision, ResultatVision
from services.protocole.fiche_rover import FicheRover
from services.protocole.messages import Commande, Image, Scan
from simulation.camera_simulee import CameraSimulee
from simulation.monde import Monde
from simulation.robot_simule import MessageCapteur, RobotSimule, VeriteTerrain
from simulation.vision_oracle import DetecteurVerite

PAS_S = 0.01


@dataclass(slots=True)
class Releve:
    """Mesures accumulées pendant un essai."""

    erreurs_position: list[float] = field(default_factory=list)
    erreurs_cap: list[float] = field(default_factory=list)
    sigmas: list[float] = field(default_factory=list)
    duree_cerveau_s: float = 0.0
    duree_simulee_s: float = 0.0
    evenements: list[EvenementCerveau] = field(default_factory=list)

    def erreur_position_max(self) -> float:
        return max(self.erreurs_position, default=0.0)

    def erreur_position_moyenne(self) -> float:
        return sum(self.erreurs_position) / max(len(self.erreurs_position), 1)


class BancEssai:
    def __init__(
        self,
        fiche: FicheRover,
        monde: Monde,
        graine_bruit: int = 0,
        latence_commande_s: float = 0.0,
        vision: Literal["aucune", "oracle"] = "aucune",
        latence_vision_s: float = 2.0,
        stereo: bool = True,
    ) -> None:
        """`stereo` : la caméra et la paire stéréo sont rendues (comme en démo) et le cerveau
        en tire la carte des hauteurs. `vision="oracle"` : détecteur parfait (`DetecteurVerite`),
        en vigilance, dont le résultat arrive `latence_vision_s` après la prise de vue."""
        camera = fiche.capteurs.camera
        avec_vision = vision == "oracle" and camera is not None
        avec_stereo = stereo and camera is not None and fiche.capteurs.stereo is not None
        self.robot = RobotSimule(
            fiche, monde, graine_bruit=graine_bruit, camera=avec_vision or avec_stereo
        )
        self.rendu = (
            CameraSimulee(self.robot.modele, camera, fiche.capteurs.stereo)
            if avec_stereo and camera is not None
            else None
        )
        self.cerveau = Cerveau(fiche, monde.scenario)
        self.latence_commande_s = latence_commande_s
        self._commandes: list[tuple[float, Commande]] = []
        self.releve = Releve()
        self.detecteur = DetecteurVerite(lambda: self.robot) if avec_vision else None
        self.latence_vision_s = latence_vision_s
        self._analyses: list[tuple[float, DemandeVision, ResultatVision]] = []
        if self.detecteur is not None:
            self.cerveau.vision_disponible = True
            self.cerveau.vigilance = True
            self.cerveau.periode_vigilance = 4.0  # détecteur parfait : pas de quota

    def verite(self) -> VeriteTerrain:
        return self.robot.verite()

    def avancer(self, duree: float, pendant: Callable[[BancEssai], bool] | None = None) -> Releve:
        """Fait tourner l'ensemble pendant `duree` secondes simulées (arrêt si `pendant` → True)."""
        fin = self.robot.t + duree
        cerveau = self.cerveau
        while self.robot.t < fin:
            while self._commandes and self._commandes[0][0] <= self.robot.t:
                self.robot.recevoir(self._commandes.pop(0)[1])
            messages = self.robot.avancer(PAS_S)
            self._capturer(messages)
            debut = time.perf_counter()
            if self.detecteur is not None:
                self._vision()
            for message in messages:
                commande = cerveau.sur_message_robot(message)
                if commande is not None:
                    self._commandes.append((self.robot.t + self.latence_commande_s, commande))
                if isinstance(message, Image):
                    self._demander_vision()
                if isinstance(message, Scan):
                    travail = cerveau.prendre_travail()
                    if travail is not None:
                        cerveau.appliquer(cerveau.traiter_perception(travail))
                    self._mesurer()
            self.releve.evenements.extend(cerveau.vider_sortie())
            self.releve.duree_cerveau_s += time.perf_counter() - debut
            self.releve.duree_simulee_s += PAS_S
            if pendant is not None and pendant(self):
                break
        return self.releve

    def _capturer(self, messages: list[MessageCapteur]) -> None:
        """Prise de vue : rendue (caméra et paire stéréo), ou seulement datée pour l'oracle."""
        capture = self.robot.prendre_capture()
        camera = self.robot.fiche.capteurs.camera
        if capture is None or camera is None:
            return
        if self.rendu is not None:
            messages.extend(self.rendu.rendre(capture.qpos, capture.t))
        elif self.detecteur is not None:
            messages.append(
                Image(t=capture.t, largeur=camera.largeur, hauteur=camera.hauteur, donnees="")
            )

    def fermer(self) -> None:
        if self.rendu is not None:
            self.rendu.fermer()

    def _vision(self) -> None:
        """Analyses de l'oracle livrées après la latence."""
        prets = [a for a in self._analyses if a[0] <= self.robot.t]
        self._analyses = [a for a in self._analyses if a[0] > self.robot.t]
        for _, demande, resultat in prets:
            self.cerveau.sur_resultat_vision(demande, resultat)

    def _demander_vision(self) -> None:
        demande = self.cerveau.prendre_demande_vision()
        if demande is None or self.detecteur is None:
            return
        analyse = self.detecteur.analyser_immediat(demande)
        resultat = ResultatVision(analyse, self.detecteur.nom, self.latence_vision_s * 1000)
        self._analyses.append((demande.t + self.latence_vision_s, demande, resultat))

    def _mesurer(self) -> None:
        v = self.robot.verite()
        p = self.cerveau.localisation.pose()
        self.releve.erreurs_position.append(math.hypot(p.x - v.x, p.y - v.y))
        self.releve.erreurs_cap.append(
            abs(math.atan2(math.sin(p.cap - v.cap), math.cos(p.cap - v.cap)))
        )
        self.releve.sigmas.append(p.sigma_xy)
