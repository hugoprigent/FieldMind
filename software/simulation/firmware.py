"""Firmware simulé du contrôleur embarqué (MCU) du rover.

Ce module reproduit ce que fera le microcontrôleur du rover réel, indépendamment du cerveau :
- chien de garde : sans consigne pendant `chien_de_garde_s`, freinage jusqu'à l'arrêt ;
- arrêt d'urgence : obstacle dans le secteur avant et le couloir du rover, plus proche que la
  distance de freinage d'urgence augmentée de la marge `distance_arret_avant` (idem en marche
  arrière). Entre deux balayages lidar, la distance libre est diminuée du chemin parcouru mesuré
  par les codeurs : à 1,5 m/s le rover avance de 25 cm entre deux scans à 6 Hz ;
- limitation des consignes : accélération, décélération, vitesse et braquage maximaux, vitesse du
  servo de direction ;
- régulation de vitesse PI sur la vitesse des roues arrière (rôle du VESC) ;
- géométrie Ackermann : un angle de braquage « bicyclette » donne un angle par roue avant.

Il tourne à fréquence fixe (100 Hz) sans allocation, pour rester transposable en C++ sur le MCU.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from services.protocole.fiche_rover import FicheRover
from services.protocole.messages import RaisonArret

FREQUENCE_HZ = 100.0
HAUTEUR_IMPACT_SOL = 0.12
"""Impact du lidar à moins de 12 cm au-dessus du sol sous le rover : le terrain (rover incliné)."""
VITESSE_MAINTIEN = 0.15
"""Sous cette vitesse (m/s), consigne nulle : maintien à l'arrêt (PI), plus freinage seul."""


@dataclass(slots=True)
class SortiesFirmware:
    couple_moteur: float = 0.0
    braquage_gauche: float = 0.0
    braquage_droit: float = 0.0


@dataclass(slots=True)
class Batterie:
    """Batterie Li-ion : tension à vide selon la charge, résistance interne, décharge."""

    cellules: int
    capacite_ah: float
    resistance: float
    etat_charge: float = 0.92
    courant: float = 0.0
    tension: float = 0.0

    def tension_a_vide(self) -> float:
        return self.cellules * (3.0 + 1.2 * self.etat_charge)

    def consommer(self, puissance_w: float, dt: float) -> None:
        a_vide = self.tension_a_vide()
        self.courant = puissance_w / max(a_vide, 1.0)
        self.tension = a_vide - self.courant * self.resistance
        self.etat_charge = max(
            0.0, self.etat_charge - self.courant * dt / (self.capacite_ah * 3600)
        )


class FirmwareSimule:
    """Boucle de contrôle du MCU. `cycle()` est appelée à FREQUENCE_HZ par le simulateur."""

    def __init__(self, fiche: FicheRover) -> None:
        self._f = fiche
        g = fiche.geometrie
        self._dt = 1.0 / FREQUENCE_HZ
        lx = fiche.capteurs.lidar.position[0]
        self._avant_depuis_lidar = g.longueur / 2 - lx
        self._arriere_depuis_lidar = g.longueur / 2 + lx
        self._demi_couloir = g.demi_largeur + 0.05
        # Consigne reçue du cerveau
        self._vitesse_demandee = 0.0
        self._braquage_demande = 0.0
        self._t_derniere_consigne = 0.0  # au démarrage, le chien de garde laisse un délai de grâce
        # États internes
        self.vitesse_consigne = 0.0
        self.braquage_consigne = 0.0
        self.arret: RaisonArret | None = None
        self.arret_operateur = False
        self._integrale = 0.0
        self._distance_avant = math.inf
        self._distance_arriere = math.inf
        self._parcouru_depuis_scan = 0.0
        self.sorties = SortiesFirmware()
        self.regime_moteur = 0.0
        b = fiche.physique.batterie
        self.batterie = Batterie(b.cellules_serie, b.capacite_ah, b.resistance_interne)
        self.batterie.consommer(b.consommation_repos_w, 0.0)

    # ------------------------------------------------------------------ entrées
    def recevoir_consigne(self, t: float, vitesse: float, braquage: float) -> None:
        lim = self._f.limites
        self._vitesse_demandee = min(max(vitesse, -lim.vitesse_arriere_max), lim.vitesse_max)
        self._braquage_demande = min(max(braquage, -lim.braquage_max), lim.braquage_max)
        self._t_derniere_consigne = t

    def recevoir_scan(
        self,
        angle_min: float,
        pas: float,
        distances: list[float | None],
        roulis: float = 0.0,
        tangage: float = 0.0,
    ) -> None:
        """Distances libres devant le pare-chocs avant et derrière le pare-chocs arrière.

        Le roulis et le tangage (centrale inertielle, lue par le MCU) écartent les impacts qui
        tombent plus bas que le sol sous le rover : en pente, le plan de mesure plonge et touche
        le terrain, qui n'est pas un obstacle."""
        secteur = math.radians(self._f.securite.secteur_avant_deg)
        hauteur_lidar = self._f.capteurs.lidar.position[2]
        sr, sp, cp = math.sin(roulis), math.sin(tangage), math.cos(tangage)
        avant = arriere = math.inf
        for i, d in enumerate(distances):
            if d is None:
                continue
            a = angle_min + i * pas
            x, y = d * math.cos(a), d * math.sin(a)
            # Hauteur de l'impact par rapport au sol sous le rover (tangage positif : nez bas).
            if hauteur_lidar - sp * x + cp * sr * y < HAUTEUR_IMPACT_SOL:
                continue
            if abs(y) > self._demi_couloir:
                continue
            a_norm = math.atan2(math.sin(a), math.cos(a))
            if abs(a_norm) <= secteur:
                avant = min(avant, x - self._avant_depuis_lidar)
            elif abs(a_norm) >= math.pi - secteur:
                arriere = min(arriere, -x - self._arriere_depuis_lidar)
        self._distance_avant = avant
        self._distance_arriere = arriere
        self._parcouru_depuis_scan = 0.0

    def distance_arret(self, vitesse: float) -> float:
        """Distance libre minimale pour s'arrêter à temps depuis `vitesse` (m)."""
        lim, sec = self._f.limites, self._f.securite
        reaction = 2 * self._dt  # un cycle pour décider, un pour appliquer
        return (
            sec.distance_arret_avant
            + abs(vitesse) * reaction
            + vitesse**2 / (2 * lim.deceleration_urgence)
        )

    # ------------------------------------------------------------------ cycle
    def cycle(
        self, t: float, omega_essieu_arriere: float, braquage_mesure: float
    ) -> SortiesFirmware:
        """Un cycle de régulation. `omega_essieu_arriere` : vitesse des roues arrière (rad/s)."""
        f = self._f
        lim, sec, g = f.limites, f.securite, f.geometrie
        dt = self._dt
        vitesse_mesuree = omega_essieu_arriere * g.rayon_roue
        self._parcouru_depuis_scan += vitesse_mesuree * dt
        libre_avant = self._distance_avant - max(self._parcouru_depuis_scan, 0.0)
        libre_arriere = self._distance_arriere + min(self._parcouru_depuis_scan, 0.0)
        freinage = self.distance_arret(max(vitesse_mesuree, self.vitesse_consigne, key=abs))

        # 1. Protections locales : elles priment sur toute consigne du cerveau.
        cible = self._vitesse_demandee
        self.arret = None
        if self.arret_operateur:
            self.arret = "arret_operateur"
        elif t - self._t_derniere_consigne > sec.chien_de_garde_s:
            self.arret = "chien_de_garde"
        elif cible > 0 and libre_avant < freinage:
            self.arret = "obstacle_avant"
        elif cible < 0 and libre_arriere < freinage:
            self.arret = "obstacle_arriere"
        if self.arret is not None:
            cible = 0.0

        # 2. Rampe de vitesse (décélération d'urgence si une protection est active).
        ecart = cible - self.vitesse_consigne
        accelere = abs(cible) > abs(self.vitesse_consigne) and cible * self.vitesse_consigne >= 0
        if accelere:
            pas_max = lim.acceleration_max * dt
        elif self.arret is not None:
            pas_max = lim.deceleration_urgence * dt
        else:
            pas_max = lim.deceleration_max * dt
        self.vitesse_consigne += max(-pas_max, min(pas_max, ecart))

        # 3. Direction : consigne limitée en vitesse (servo), puis Ackermann.
        ecart_b = self._braquage_demande - self.braquage_consigne
        pas_b = lim.vitesse_braquage * dt
        self.braquage_consigne += max(-pas_b, min(pas_b, ecart_b))
        gauche, droit = self.ackermann(self.braquage_consigne)

        # 4. Régulation de vitesse PI avec anti-emballement de l'intégrale. Consigne nulle :
        #    freinage moteur proportionnel (mode « frein » du VESC) ; près de l'arrêt, maintien
        #    en vitesse nulle avec intégrale, sans quoi le rover glisserait dans une pente.
        moteur = f.physique.moteur
        erreur = self.vitesse_consigne - vitesse_mesuree
        kp, ki, k_frein = 3.0, 5.0, 8.0
        if self.vitesse_consigne == 0.0 and abs(vitesse_mesuree) > VITESSE_MAINTIEN:
            self._integrale = 0.0
            couple = -k_frein * vitesse_mesuree
        elif self.vitesse_consigne == 0.0:
            # Maintien : rigide, et l'intégrale compense vite la pente.
            couple = k_frein * erreur + 4 * ki * self._integrale
            if abs(couple) < moteur.couple_max:
                self._integrale += erreur * dt
        else:
            couple = kp * erreur + ki * self._integrale
            if abs(couple) < moteur.couple_max:
                self._integrale += erreur * dt
        couple = max(-moteur.couple_max, min(moteur.couple_max, couple))

        # 5. Moteur et batterie.
        omega_moteur = omega_essieu_arriere * moteur.rapport_transmission
        self.regime_moteur = omega_moteur * 60 / (2 * math.pi)
        puissance_meca = couple * omega_moteur
        puissance = (
            puissance_meca / moteur.rendement if puissance_meca > 0 else 0.3 * puissance_meca
        )
        self.batterie.consommer(f.physique.batterie.consommation_repos_w + puissance, dt)

        s = self.sorties
        s.couple_moteur, s.braquage_gauche, s.braquage_droit = couple, gauche, droit
        return s

    def ackermann(self, braquage: float) -> tuple[float, float]:
        """Angle de chaque roue avant (gauche, droite) pour un angle « bicyclette » donné."""
        if abs(braquage) < 1e-6:
            return 0.0, 0.0
        g = self._f.geometrie
        rayon = g.empattement / math.tan(braquage)  # rayon au milieu de l'essieu arrière
        demi_voie = g.voie_avant / 2
        gauche = math.atan(g.empattement / (rayon - demi_voie))
        droit = math.atan(g.empattement / (rayon + demi_voie))
        return gauche, droit
