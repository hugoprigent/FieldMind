"""Superviseur de sécurité : chaque consigne passe ici avant d'aller aux moteurs (20 Hz).

Il ne dépend d'aucun modèle d'IA ni du réseau, seulement des capteurs du rover :
- **obstacles** : points du lidar (plan à 32 cm) et obstacles bas de la stéréo, cherchés dans le
  couloir que balaie le rover le long de l'arc prévu (braquage courant), devant ou derrière selon
  le sens. La vitesse autorisée est celle qui permet de s'arrêter avant le premier obstacle :
  d = v·latence + v²/(2·décélération) + marge ;
- **capteurs** : lidar périmé → arrêt ; stéréo absente → vitesse plafonnée (ce qui est plus bas
  que 32 cm est invisible au lidar) ; ESP absent ou en défaut → arrêt ;
- **état du rover** : inclinaison, choc (arrêt d'urgence verrouillé), blocage (consigne sans
  déplacement mesuré), geofence en mode auto ;
- **homme mort** : une consigne trop vieille vaut zéro ;
- **remorque** (facultative) : son emprise, entraînée le long de l'arc avec son angle d'attelage
  estimé, est cherchée dans les obstacles ; en reculant, l'angle doit rester sous la limite de
  mise en portefeuille sur toute la distance d'arrêt, et être bien connu ; freinage allongé par
  sa masse, vitesses plafonnées, pas de marche arrière en automatique.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from fieldmind.bus import Bus, S
from fieldmind.config import Profil, braquage_max
from fieldmind.controle.remorque import distance_libre_remorque
from fieldmind.types import (
    Consigne,
    EtatEsp,
    EtatSecurite,
    Imu,
    Mode,
    Obstacles,
    Odometrie,
    Pose2D,
    Scan,
)

PAS_ARC_M = 0.04
PORTEE_ARC_M = 3.0


@dataclass(slots=True)
class Couloir:
    """Espace libre le long d'un arc, pour un sens de marche."""

    libre_m: float
    point: tuple[float, float] | None = None  # premier obstacle touché (base_link)
    source: str | None = None


def distance_libre(points: np.ndarray, braquage: float, sens: int, profil: Profil,
                   portee: float = PORTEE_ARC_M, pas: float = PAS_ARC_M) -> tuple[float, int]:
    """Longueur d'arc (m, mesurée sur l'essieu arrière) que le rover peut parcourir avant que son
    emprise, élargie de la marge latérale, touche un des `points` (base_link). Renvoie aussi
    l'indice du point touché (-1 si aucun).

    `sens` = +1 en marche avant, -1 en arrière. Le modèle est celui de la bicyclette : courbure
    κ = tan(braquage)/empattement autour du centre instantané (0, 1/κ)."""
    if len(points) == 0:
        return portee, -1
    g = profil.geometrie
    marge = float(profil.valeur("securite.marge_laterale_m", 0.10))
    x0, x1 = g.emprise_arriere_m, g.emprise_avant_m
    demi = g.emprise_demi_largeur_m + marge
    kappa = math.tan(braquage) / g.empattement_m
    # Pré-filtre : seuls les points de la bande (droit) ou de l'anneau (arc) balayés par
    # l'emprise peuvent être touchés.
    indices = np.arange(len(points))
    if abs(kappa) < 1e-4:
        garde = np.abs(points[:, 1]) < demi
        garde &= (points[:, 0] > x0) if sens > 0 else (points[:, 0] < x1)
        garde &= np.abs(points[:, 0]) < portee + max(abs(x0), abs(x1)) + pas
    else:
        rayon = 1.0 / kappa
        coins_y = np.array([-demi, demi, -demi, demi]) - rayon
        coins_x = np.array([x0, x0, x1, x1])
        r_coins = np.hypot(coins_x, coins_y)
        r_min = abs(rayon) - demi if abs(rayon) > demi else 0.0
        r_max = float(r_coins.max())
        r_points = np.hypot(points[:, 0], points[:, 1] - rayon)
        garde = (r_points >= r_min - 1e-3) & (r_points <= r_max + 1e-3)
    points, indices = points[garde], indices[garde]
    if len(points) == 0:
        return portee, -1
    tout_s = np.arange(pas, portee + pas, pas) * sens
    for debut in range(0, len(tout_s), 16):  # par tranches : on s'arrête au premier contact
        s = tout_s[debut : debut + 16]
        if abs(kappa) < 1e-4:
            px = points[None, :, 0] - s[:, None]
            py = np.broadcast_to(points[None, :, 1], px.shape)
        else:
            theta = kappa * s  # cap du rover après la longueur d'arc s
            cx, cy = np.sin(theta) / kappa, (1 - np.cos(theta)) / kappa
            c, si = np.cos(theta), np.sin(theta)
            dx = points[None, :, 0] - cx[:, None]
            dy = points[None, :, 1] - cy[:, None]
            px = c[:, None] * dx + si[:, None] * dy
            py = -si[:, None] * dx + c[:, None] * dy
        dedans = (px > x0) & (px < x1) & (np.abs(py) < demi)
        touche = dedans.any(axis=1)
        if touche.any():
            k = int(np.argmax(touche))
            indice = int(indices[int(np.argmax(dedans[k]))])
            return max(0.0, abs(float(s[k])) - pas), indice
    return portee, -1


def vitesse_admissible(libre_m: float, latence: float, deceleration: float, marge: float) -> float:
    """Plus grande vitesse v telle que v·latence + v²/(2a) + marge ≤ libre."""
    utile = libre_m - marge
    if utile <= 0:
        return 0.0
    return deceleration * (-latence + math.sqrt(latence**2 + 2 * utile / deceleration))


@dataclass
class _Blocage:
    depuis: float | None = None
    verrouille: bool = False


@dataclass
class Superviseur:
    bus: Bus
    profil: Profil
    arret_urgence: bool = False
    raison_urgence: str | None = None
    operateurs: int = 0  # postes connectés (tenu à jour par le serveur)
    memoire_obstacles: Any = None  # () -> points base_link des obstacles bas mémorisés (carte)
    zone: Any = None  # (pose odom, sens) -> True si le mouvement sort de la zone GPS autorisée
    remorque: Any = None  # fieldmind.controle.remorque.SuiviRemorque (facultatif)
    _blocage: _Blocage = field(default_factory=_Blocage)
    _dernier: EtatSecurite | None = None

    # ------------------------------------------------------------------ verrou

    def declencher(self, raison: str) -> None:
        if not self.arret_urgence:
            self.bus.journaliser("securite", f"arrêt d'urgence : {raison}", "erreur")
        self.arret_urgence = True
        self.raison_urgence = raison

    def rearmer(self) -> None:
        if self.arret_urgence:
            self.bus.journaliser("securite", "arrêt d'urgence levé", "info")
        self.arret_urgence = False
        self.raison_urgence = None
        self._blocage = _Blocage()

    # ------------------------------------------------------------------ évaluation

    def evaluer(self, consigne: Consigne, mode: Mode, arme: bool,
                t: float | None = None) -> tuple[Consigne, EtatSecurite]:
        t = time.monotonic() if t is None else t
        p = self.profil
        raisons: list[str] = []
        arret = False

        if self.arret_urgence:
            raisons.append("arret_urgence")
            if self.raison_urgence in ("choc", "inclinaison"):
                raisons.append(self.raison_urgence)
            arret = True
        if mode == "arret":
            raisons.append("mode_arret")
            arret = True
        if not arme:
            raisons.append("non_arme")
            arret = True

        esp: EtatEsp | None = self.bus.dernier(S.ESP)
        if esp is None or not esp.connecte or self.bus.age(S.ESP) > 1.5:
            raisons.append("esp_absent")
            arret = True
        elif esp.defaut:
            raisons.append("esp_defaut")
            arret = True

        perimee = float(p.valeur("securite.consigne_manuelle_perimee_s", 0.5))
        if consigne.source != "aucune" and t - consigne.t > perimee:
            raisons.append("consigne_perimee")
            consigne = Consigne(t=t, source=consigne.source)

        if mode == "manuel" and self.operateurs == 0:
            raisons.append("operateur_perdu")
            arret = True

        imu: Imu | None = self.bus.dernier(S.IMU)
        if imu is not None and self.bus.age(S.IMU) < 1.0:
            roulis, tangage = imu.roulis_tangage()
            if max(abs(roulis), abs(tangage)) > math.radians(
                    float(p.valeur("securite.inclinaison_max_deg", 25.0))):
                raisons.append("inclinaison")
                arret = True
            ax, ay, _ = imu.accel_g
            moteurs = esp is not None and (esp.pwm_g or esp.pwm_d)
            if moteurs and math.hypot(ax, ay) > float(p.valeur("securite.choc_g", 1.6)):
                self.declencher("choc")
                raisons.append("choc")
                arret = True

        # ---------------- banc : PWM bruts, pas de contrôle d'obstacles
        if mode == "banc":
            raisons.append("banc")
            if arret or consigne.pwm_brut is None:
                return self._finir(Consigne(t=t, source="banc", pwm_brut=(0, 0)), t, 0.0, 0.0,
                                   raisons, None, None)
            plafond = int(p.valeur("traction.pwm_max_banc", 15))
            g, d = consigne.pwm_brut
            g, d = max(-plafond, min(plafond, g)), max(-plafond, min(plafond, d))
            return self._finir(Consigne(t=t, source="banc", pwm_brut=(g, d),
                                        braquage=consigne.braquage), t, 0.0, 0.0, raisons,
                               None, None)

        # ---------------- obstacles
        braquage_maxi = braquage_max(p)
        braquage = max(-braquage_maxi, min(braquage_maxi, consigne.braquage))
        scan: Scan | None = self.bus.dernier(S.SCAN)
        age_scan = self.bus.age(S.SCAN)
        if scan is None or age_scan > float(p.valeur("securite.lidar_perime_s", 0.5)):
            raisons.append("lidar_absent")
            arret = True
            points_lidar = np.zeros((0, 2), np.float32)
        else:
            points_lidar = scan.points()
        stereo: Obstacles | None = self.bus.dernier(S.OBSTACLES_STEREO)
        stereo_ok = (bool(p.valeur("perception.stereo_active", True)) and stereo is not None
                     and self.bus.age(S.OBSTACLES_STEREO) < float(
                         p.valeur("securite.stereo_perimee_s", 1.0)))
        points_avant = points_lidar
        if stereo_ok and stereo is not None and len(stereo.points):
            points_avant = np.vstack((points_lidar, stereo.points.astype(np.float32)))

        # Obstacles bas mémorisés (vus par la stéréo, sortis du champ de la caméra en virage)
        points_arriere = points_lidar
        memoire = self._memoire() if stereo_ok else np.zeros((0, 2), np.float32)
        if len(memoire):
            points_avant = np.vstack((points_avant, memoire))
            points_arriere = np.vstack((points_lidar, memoire))
        libre_av, i_av = distance_libre(points_avant, braquage, +1, p)
        libre_ar, _ = distance_libre(points_arriere, braquage, -1, p)

        latence = float(p.valeur("securite.latence_s", 0.25)) + min(age_scan, 0.5)
        decel = float(p.valeur("securite.deceleration_m_s2", 0.8))
        attelage = self._remorque()
        raisons_remorque: list[str] = []
        if attelage is not None:
            etat_r, g_r = attelage
            # Remorque derrière : lidar et obstacles bas mémorisés (les caméras regardent devant)
            libre_r, i_r = distance_libre_remorque(points_arriere, braquage, +1, etat_r.angle, p,
                                                   g_r)
            if libre_r < libre_av:
                libre_av, i_av = libre_r, -1  # touché par la remorque : « obstacle devant »
            angle_max = math.radians(float(p.valeur("remorque.angle_max_arriere_deg", 20.0)))
            libre_r, i_r = distance_libre_remorque(points_arriere, braquage, -1, etat_r.angle, p,
                                                   g_r, angle_max=angle_max)
            if libre_r < libre_ar:
                libre_ar = libre_r
                if i_r < 0:
                    raisons_remorque.append("remorque_portefeuille")
            if math.degrees(etat_r.incertitude) > float(
                    p.valeur("remorque.incertitude_max_recul_deg", 8.0)):
                libre_ar = 0.0
                raisons_remorque.append("remorque_incertaine")
            if mode == "auto" and not bool(p.valeur("remorque.marche_arriere_auto", False)):
                libre_ar = 0.0
                raisons_remorque.append("remorque_recul_auto")
            # Roues de la remorque libres : sa masse pousse le rover au freinage
            masse = float(p.geometrie.masse_kg)
            decel *= masse / (masse + float(p.valeur("remorque.masse_kg", 2.2)))
        marge = float(p.valeur("securite.marge_avant_m", 0.25))
        v_av = vitesse_admissible(libre_av, latence, decel, marge)
        v_ar = vitesse_admissible(libre_ar, latence, decel, marge)
        absolu = float(p.valeur("securite.distance_arret_absolu_m", 0.30))
        if libre_av < absolu:  # trop près : même au pas, on ne s'approche plus
            v_av = 0.0
        if libre_ar < absolu:
            v_ar = 0.0

        v_mode = float(p.valeur("vitesses.max_manuel_m_s" if mode == "manuel"
                                else "vitesses.max_auto_m_s", 0.5))
        v_av = min(v_av, v_mode)
        v_ar = min(v_ar, float(p.valeur("vitesses.max_arriere_m_s", 0.3)))
        if attelage is not None:
            raisons.append("remorque")
            v_av = min(v_av, float(p.valeur("remorque.vitesse_max_m_s", 0.4)))
            v_ar = min(v_ar, float(p.valeur("remorque.vitesse_max_arriere_m_s", 0.12)))
        if not stereo_ok:
            raisons.append("stereo_absente")
            if mode == "auto" and bool(p.valeur("securite.exiger_stereo_en_auto", False)):
                arret = True
            v_av = min(v_av, float(p.valeur("securite.vitesse_sans_stereo_m_s", 0.3)))

        # Détections (personnes, animaux) : seulement plus de prudence ; sans détecteur, rien
        prudence = self._prudence_detections(braquage)
        if prudence is not None and v_av > prudence:
            v_av = prudence
            raisons.append("personne_proche")

        if consigne.vitesse > 0 and v_av < consigne.vitesse:
            bas = stereo_ok and i_av >= len(points_lidar)
            if v_av <= 0.02:
                raisons.append("obstacle_bas" if bas else "obstacle_avant")
            else:
                raisons.append("ralenti_obstacle")
        if consigne.vitesse < 0 and v_ar < -consigne.vitesse:
            if raisons_remorque:
                raisons.extend(raisons_remorque)
            else:
                raisons.append("obstacle_arriere" if v_ar <= 0.02 else "ralenti_obstacle")

        # ---------------- blocage et geofence
        odom: Odometrie | None = self.bus.dernier(S.ODOM)
        if self._bloque(consigne, odom, t):
            raisons.append("blocage")
            arret = True
        if mode == "auto" and odom is not None and self._hors_zone(consigne, braquage, odom):
            raisons.append("geofence")
            arret = True
        if mode == "auto" and odom is not None and self.zone is not None and \
                abs(consigne.vitesse) > 0:
            pose_zone = odom.pose
            if attelage is not None and consigne.vitesse < 0:  # la remorque passe la première
                recul = attelage[1].attelage_m + attelage[1].arriere_m
                pose_zone = Pose2D(pose_zone.x - recul * math.cos(pose_zone.cap),
                                   pose_zone.y - recul * math.sin(pose_zone.cap), pose_zone.cap)
            try:
                dehors = self.zone(pose_zone, 1 if consigne.vitesse > 0 else -1)
            except Exception:
                dehors = None
            if dehors:
                raisons.append("zone")
                arret = True

        if arret:
            v_av = v_ar = 0.0
        vitesse = max(-v_ar, min(v_av, consigne.vitesse))
        if abs(vitesse) < 0.02:
            vitesse = 0.0
        sortie = Consigne(t=t, vitesse=vitesse, braquage=braquage, source=consigne.source)
        return self._finir(sortie, t, v_av, v_ar, raisons, libre_av, libre_ar)

    def _finir(self, consigne: Consigne, t: float, v_av: float, v_ar: float, raisons: list[str],
               libre_av: float | None, libre_ar: float | None) -> tuple[Consigne, EtatSecurite]:
        etat = EtatSecurite(
            t=t, arret=v_av <= 0 and v_ar <= 0 and consigne.pwm_brut is None,
            vitesse_max_avant=round(v_av, 3), vitesse_max_arriere=round(v_ar, 3),
            raisons=list(dict.fromkeys(raisons)), libre_avant_m=libre_av, libre_arriere_m=libre_ar,
            arret_urgence=self.arret_urgence)
        precedent = self._dernier
        if precedent is not None:
            for nouvelle in set(etat.raisons) - set(precedent.raisons):
                if nouvelle in _MESSAGES:
                    self.bus.journaliser("securite", _MESSAGES.get(nouvelle, nouvelle), "alerte")
        self._dernier = etat
        return consigne, etat

    def _prudence_detections(self, braquage: float) -> float | None:
        """Vitesse plafond si un être vivant détecté est devant, à moins de 4 m, dans un couloir
        élargi (1 m de chaque côté) ; None si rien (ou pas de détecteur, ou détection vieille)."""
        message = self.bus.dernier(S.DETECTIONS)
        if not isinstance(message, dict) or self.bus.age(S.DETECTIONS) > 1.5:
            return None
        demi = self.profil.geometrie.emprise_demi_largeur_m + 1.0
        vivants = {"personne", "person", "enfant", "chien", "dog", "chat", "cat", "animal",
                   "cheval", "horse", "vache", "cow", "oiseau", "bird", "mouton", "sheep"}
        for objet in message.get("objets") or []:
            sol = objet.get("sol")
            if not sol or str(objet.get("label", "")).lower() not in vivants:
                continue
            x, y = float(sol[0]), float(sol[1])
            # dans le repère de l'arc : on tolère la courbure par une marge latérale large
            if 0 < x < 4.0 and abs(y - x * math.tan(braquage) * 0.5) < demi:
                return float(self.profil.valeur("securite.vitesse_pres_personne_m_s", 0.25))
        return None

    def _remorque(self) -> tuple[Any, Any] | None:
        """(EtatRemorque, GeometrieRemorque) si une remorque est attelée."""
        if self.remorque is None or not bool(self.profil.valeur("remorque.attelee", False)):
            return None
        etat = self.remorque.etat
        return (etat, self.remorque.geometrie()) if etat.attelee else None

    def _memoire(self) -> np.ndarray:
        if self.memoire_obstacles is None:
            return np.zeros((0, 2), np.float32)
        try:
            return np.asarray(self.memoire_obstacles(), np.float32).reshape(-1, 2)
        except Exception:  # la mémoire est un plus : son absence ne doit pas tout arrêter
            return np.zeros((0, 2), np.float32)

    def _bloque(self, consigne: Consigne, odom: Odometrie | None, t: float) -> bool:
        b = self._blocage
        if abs(consigne.vitesse) < 0.08 or odom is None:
            b.depuis = None
            b.verrouille = False
            return False
        if b.verrouille:
            return True
        if odom.qualite < 0.4 or abs(odom.vitesse) > 0.03:
            b.depuis = None
            return False
        if b.depuis is None:
            b.depuis = t
        if t - b.depuis > float(self.profil.valeur("securite.blocage_s", 2.5)):
            b.verrouille = True  # levé quand la consigne revient à zéro
            return True
        return False

    def _hors_zone(self, consigne: Consigne, braquage: float, odom: Odometrie) -> bool:
        rayon = float(self.profil.valeur("securite.geofence_m", 60.0))
        x, y = odom.pose.x, odom.pose.y
        if math.hypot(x, y) < rayon:
            return False
        sens = 1 if consigne.vitesse >= 0 else -1
        cap = odom.pose.cap if sens > 0 else odom.pose.cap + math.pi
        # Autorisé seulement en se rapprochant du départ
        return math.cos(cap) * -x + math.sin(cap) * -y <= 0


_MESSAGES = {
    "obstacle_avant": "obstacle devant : arrêt",
    "obstacle_bas": "obstacle bas vu par la stéréo : arrêt",
    "obstacle_arriere": "obstacle derrière : arrêt",
    "lidar_absent": "lidar absent ou muet : traction coupée",
    "esp_absent": "ESP absent : traction coupée",
    "esp_defaut": "ESP en défaut : traction coupée",
    "inclinaison": "inclinaison excessive : arrêt",
    "blocage": "rover bloqué (consigne sans déplacement) : arrêt",
    "geofence": "limite de distance au départ atteinte",
    "zone": "bord de la zone autorisée (GPS) : arrêt",
    "personne_proche": "personne ou animal devant : ralenti",
    "operateur_perdu": "plus de poste connecté : arrêt",
    "remorque_portefeuille": "remorque trop braquée pour reculer : avancer pour la redresser",
    "remorque_incertaine": "angle de la remorque incertain : avancer 1 m tout droit, ou la "
                           "déclarer dans l'axe",
    "remorque_recul_auto": "remorque : pas de marche arrière en automatique",
}
