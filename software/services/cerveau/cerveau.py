"""Cerveau du rover : il reçoit les messages du robot et de l'opérateur, et produit des consignes.

Le cerveau est déterministe et piloté par l'horodatage des messages du robot : il ne lit jamais
l'horloge murale pour décider. On peut donc le faire tourner à l'identique en temps réel, en
accéléré (évaluation) ou sur le rover réel.

Deux flux :
- rapide (fil principal) : localisation, suivi de trajectoire à 25 Hz, mission, opérateur ;
- lent (`traiter_perception`, exécutable dans un autre fil) : intégration du dernier balayage lidar
  dans la carte, puis, au plus à 2 Hz ou aussitôt que le chemin est coupé, calcul du chemin.
  Seul le balayage le plus récent est traité.

Profondeur : la paire stéréo est traitée dans le flux lent (SGBM, nuage de points daté à l'instant
de la prise de vue, carte des hauteurs, traversabilité). Les obstacles qu'elle trouve (troncs
couchés, rochers, souches, pentes trop fortes) entrent dans la carte de coûts comme ceux du lidar,
et limitent la vitesse quand ils sont devant le rover. C'est elle qui voit ce que le lidar, dont
le plan est à 32 cm du sol, ne voit pas.

Vision : le cerveau choisit quelles images de la caméra analyser (à la demande de l'opérateur, ou
périodiquement en « vigilance » pendant une mission) et les remet à la couche d'exécution
(`prendre_demande_vision`), qui interroge le modèle de vision. Le résultat revient par
`sur_resultat_vision` : objets placés au sol, obstacles bas ajoutés à la couche vision de la carte.
"""

from __future__ import annotations

import base64
import itertools
import math
from collections import deque
from dataclasses import dataclass, field, replace
from typing import Literal

import numpy as np

from services.cerveau.evenements import (
    Carte,
    ChatReponse,
    Chemin,
    EtatMission,
    EtatVision,
    EvenementCerveau,
    Journal,
    Mesures,
    Niveau,
    NoteIA,
    ObjetPlace,
    ObstacleVision,
    PoseEstimee,
    Profondeur,
    Statut,
    Traversabilite,
    Vision,
)
from services.cerveau.mission import Mission, OrigineObjectif, Recul
from services.mission_planner.intentions import (
    Action,
    AllerA,
    Arreter,
    Chercher,
    Observer,
    Parcours,
    PoseCourante,
    Vigilance,
)
from services.navigation.grille import GrilleOccupation, PoseCapteur
from services.navigation.localisation import Localisation
from services.navigation.localisation import PoseEstimee as Pose
from services.navigation.planificateur import (
    CarteCouts,
    DepartBloque,
    EchecPlanification,
    ParametresPlanification,
    Plan,
    chemin_bloque,
    construire_carte_couts,
    planifier,
)
from services.navigation.suivi import SuiviTrajectoire
from services.perception.camera import ModeleCamera
from services.perception.elevation import CarteHauteurs
from services.perception.observations import (
    CoucheVision,
    Observation,
    decrire,
    distance_point_polygone,
    placer_objets,
)
from services.perception.stereo import ProfondeurStereo, image_profondeur, rotation
from services.perception.vision import DemandeVision, ResultatVision
from services.protocole.fiche_rover import FicheRover
from services.protocole.liaison import MessageRobot
from services.protocole.messages import (
    Commande,
    Firmware,
    Gnss,
    Image,
    ImageStereo,
    Imu,
    Odometrie,
    Scan,
    Stop,
    Teleoperation,
)
from services.protocole.scenario import ScenarioPublic

PERIODE_CONTROLE_S = 0.04
"""Consignes envoyées à 25 Hz : un message d'odométrie (50 Hz) sur deux."""
DELAI_TELEOP_S = 0.3
"""Sans message de téléopération pendant ce délai, la conduite manuelle revient au neutre."""
PERIODE_POSE_S = 0.1
PERIODE_CARTE_S = 0.5
PERIODE_MESURES_S = 1.0
PERIODE_PLANIFICATION_S = 0.5
PERIODE_AMELIORATION_S = 5.0
RESOLUTION_CARTE = 0.1
TOLERANCE_OBJECTIF = 0.4
TOLERANCE_ETAPE = 1.0
DELAI_SANS_PROGRES_S = 6.0
DELAI_ARRET_OBSTACLE_S = 1.5
ESPACE_DEMI_TOUR_M = 1.2
"""Espace libre devant sous lequel un demi-tour se fait en trois temps (marche arrière braquée)."""
RECUPERATIONS_MAX = 4
DEMI_TOURS_BLOQUES_MAX = 6
ECHECS_CALCUL_MAX = 3
GAIN_AMELIORATION = 0.7
"""Un nouveau chemin ne remplace le chemin courant que s'il coûte 30 % de moins (hystérésis) :
sinon le rover hésite entre deux passages équivalents."""
DETECTER_TROUS = False
AVANCE_DEPART_M = 0.8
"""Départ du calcul décalé devant le rover : un rover à direction de voiture ne pivote pas sur
place, un chemin qui part vers l'arrière impose un demi-tour."""
MARGE_LIMITES = 0.5
PERIODE_VIGILANCE_S = 30.0
"""Observation IA : une image analysée toutes les 30 s pendant une mission (quota du modèle)."""
VITESSE_RALENTIE = 0.5
DUREE_RALENTI_S = 20.0
"""« Ralentir » décidé par l'IA d'observation : vitesse plafonnée pendant 20 s."""
CATEGORIES_NOTABLES = ("personne", "animal", "vehicule", "eau", "trou")
CONFIANCE_TROUVE = 0.6
"""Confiance minimale du modèle de vision pour déclarer la cible d'une recherche trouvée."""
AGE_MAX_IMAGE_S = 1.0
HAUTEUR_IMPACT_SOL_M = 0.12
"""Un impact du lidar à moins de 12 cm au-dessus du sol sous le rover vient d'un faisceau qui
plonge (rover incliné) : c'est le terrain. Le lidar est à 32 cm ; les obstacles plus bas sont
l'affaire de la caméra."""
RAYON_CLASSEMENT_M = 9.0
"""Autour du rover, la traversabilité est reclassée à chaque paire stéréo (portée utile : 8 m)."""
PORTEE_LIBRE_STEREO_M = 4.0
"""Portée du couloir devant le rover dans lequel la caméra limite la vitesse."""
PERIODE_TRAVERSABILITE_S = 0.5


@dataclass(frozen=True, slots=True)
class DemandePlan:
    """Instantané de la mission, transmis au flux lent avec le balayage."""

    mission_id: int
    objectif: tuple[float, float]
    depart: tuple[float, float]
    cap: float
    chemin: tuple[tuple[float, float], ...]
    indice: int
    forcer: bool
    t_dernier_plan: float
    obstacles_vision: np.ndarray | None = None


@dataclass(frozen=True, slots=True)
class Attitude:
    """Roulis et tangage (centrale inertielle) et altitude intégrée, à un instant."""

    t: float
    roulis: float
    tangage: float
    z: float


@dataclass(frozen=True, slots=True)
class TravailStereo:
    """Paire stéréo à traiter, avec la pose complète du rover à l'instant de la prise de vue."""

    message: ImageStereo
    pose: Pose
    attitude: Attitude


@dataclass(frozen=True, slots=True)
class TravailPerception:
    """Balayage à intégrer, avec la pose du lidar estimée à l'instant de la mesure."""

    scan: Scan
    pose_lidar: PoseCapteur
    pose_rover: Pose
    poids: float
    demande: DemandePlan | None = None
    stereo: TravailStereo | None = None
    sol: np.ndarray | None = None
    """Faisceaux arrêtés par le sol (montée), d'après la carte des hauteurs."""


@dataclass(slots=True)
class ResultatPerception:
    evenements: list[EvenementCerveau] = field(default_factory=list)
    echos: int = 0
    mission_id: int | None = None
    plan: Plan | None = None
    raison_plan: str = ""
    echec: str | None = None
    depart_bloque: bool = False
    decalage_altitude: float = 0.0


def _cout_chemin(carte: CarteCouts, points: list[tuple[float, float]]) -> float:
    return sum(
        math.dist(a, b) * float(carte.cout[carte.cellule(*b)])
        for a, b in itertools.pairwise(points)
    )


class Cerveau:
    """Orchestration du cerveau. Les gestionnaires `sur_*` sont rapides : ils renvoient une consigne
    éventuelle et déposent des événements pour le poste opérateur dans `sortie`."""

    def __init__(
        self,
        fiche: FicheRover,
        scenario: ScenarioPublic,
        planification: ParametresPlanification | None = None,
    ) -> None:
        self.fiche = fiche
        self.scenario = scenario
        self.parametres_planification = planification or ParametresPlanification()
        self.t = 0.0
        self.etat: EtatMission = "attente"
        self.sortie: list[EvenementCerveau] = []
        self.localisation = Localisation(fiche, scenario.depart)
        self.grille = GrilleOccupation(scenario.limites, RESOLUTION_CARTE)
        self.suivi = SuiviTrajectoire(fiche)
        self.mission: Mission | None = None
        self.derniere_mission: Mission | None = None
        self.etapes: list[AllerA] = []
        """Étapes restantes d'un parcours, après l'objectif en cours."""
        self._etapes_suspendues: list[AllerA] = []
        self.recherche: tuple[str, str] | None = None
        """Mission de recherche en cours : (catégorie cherchée, nom en clair)."""
        self._recherche_suspendue: tuple[str, str] | None = None
        self.charge_cpu: float | None = None
        """Fraction d'un cœur utilisée par le cerveau, mesurée par la couche d'exécution."""
        self._travail: TravailPerception | None = None
        self._recul: Recul | None = None
        self._forcer_plan = False
        self._t_dernier_plan = -math.inf
        self._compteur_missions = 0
        self._compteur_chemins = 0
        self._t_controle = -math.inf
        self._t_pose = -math.inf
        self._t_carte = -math.inf
        self._t_mesures = -math.inf
        self._teleop: Teleoperation | None = None
        self._t_teleop = -math.inf
        self._firmware: Firmware | None = None
        self._distance_parcourue = 0.0
        self._t_odometrie: float | None = None
        self._scans_integres = 0
        self._libre_avant = math.inf
        self._obstacle_min = math.inf
        self._dernier_braquage = 0.0
        self._derniere_duree_plan_ms: float | None = None
        self._nb_calculs_total = 0
        # Profondeur (paire stéréo)
        camera = fiche.capteurs.camera
        stereo = fiche.capteurs.stereo
        self.stereo = (
            ProfondeurStereo(camera, stereo) if camera is not None and stereo is not None else None
        )
        self.hauteurs = CarteHauteurs(scenario.limites, RESOLUTION_CARTE)
        self._attitudes: deque[Attitude] = deque(maxlen=250)
        self._altitude = 0.0
        """Altitude intégrée (tangage × vitesse), sans correction : elle dérive."""
        self._correction_altitude = 0.0
        """Correction par l'altitude GNSS et le recalage sur la carte des hauteurs."""
        self._altitude_gnss = False
        """Vrai dès que le GNSS fournit l'altitude : le recalage sur la carte devient inutile."""
        self._tangage = 0.0
        self._t_altitude: float | None = None
        self._stereo_en_attente: TravailStereo | None = None
        self._t_traversabilite = -math.inf
        self._libre_stereo = math.inf
        self.paires_traitees = 0
        self._duree_stereo_ms: float | None = None
        # Vision
        self.camera = ModeleCamera(camera) if camera is not None else None
        self.couche_vision = CoucheVision(scenario.limites, RESOLUTION_CARTE)
        self.vision_disponible = False
        """Vrai quand la couche d'exécution dispose d'un modèle de vision."""
        self.vigilance = False
        self.periode_vigilance = PERIODE_VIGILANCE_S
        self.analyses_vision = 0
        self._derniere_image: Image | None = None
        self._demande_vision: DemandeVision | None = None
        self._vision_en_cours: DemandeVision | None = None
        self._analyse_operateur = False
        self._t_derniere_analyse = -math.inf
        self._vision_pause_jusqu_a = -math.inf
        self._ralenti_jusqu_a = -math.inf
        self._compteur_vision = 0
        self._compteur_observations = 0
        self._statut("attente", "Prêt. Clique sur la carte, écris une consigne ou conduis.")

    # ================================================================== événements
    def _statut(self, etat: EtatMission, message: str) -> None:
        self.etat = etat
        self.sortie.append(Statut(t=self.t, etat=etat, message=message))

    def journal(self, texte: str, niveau: Niveau = "info", **mesures: float | int | str) -> None:
        self.sortie.append(Journal(t=self.t, niveau=niveau, texte=texte, mesures=mesures))

    def vider_sortie(self) -> list[EvenementCerveau]:
        sortie, self.sortie = self.sortie, []
        return sortie

    def _publier_chemin(
        self, points: list[tuple[float, float]], objectif: tuple[float, float] | None
    ) -> None:
        self._compteur_chemins += 1
        self.sortie.append(
            Chemin(
                t=self.t,
                id=self._compteur_chemins,
                points=points,
                objectif=objectif,
                vitesse_max=self.suivi.p.vitesse_max,
            )
        )

    # ================================================================== robot
    def sur_message_robot(self, message: MessageRobot) -> Commande | None:
        """Traite un message du robot ; renvoie une consigne quand le cycle de contrôle échoit."""
        self.t = max(self.t, message.t)
        commande = None
        if isinstance(message, Imu):
            self.localisation.sur_imu(message)
            self._sur_attitude(message)
        elif isinstance(message, Odometrie):
            self._sur_odometrie(message)
            if message.t - self._t_controle >= PERIODE_CONTROLE_S - 1e-6:
                self._t_controle = message.t
                commande = self._controle(message.t)
        elif isinstance(message, Gnss):
            self._sur_gnss(message)
        elif isinstance(message, Scan):
            self._sur_scan(message)
        elif isinstance(message, Firmware):
            self._sur_firmware(message)
        elif isinstance(message, Image):
            self._sur_image(message)
        elif isinstance(message, ImageStereo):
            self._sur_stereo(message)
        self._publier_periodiques()
        return commande

    def _sur_odometrie(self, message: Odometrie) -> None:
        self.localisation.sur_odometrie(message)
        if self._t_odometrie is not None:
            dt = message.t - self._t_odometrie
            self._distance_parcourue += abs(self.localisation.vitesse) * dt
        self._t_odometrie = message.t

    def _sur_attitude(self, message: Imu) -> None:
        """Roulis, tangage et altitude (intégrée : vitesse × pente), historisés pour dater les
        nuages de la caméra."""
        w, x, y, z = message.orientation
        roulis = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        tangage = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        if self._t_altitude is not None:
            dt = message.t - self._t_altitude
            if 0 < dt < 0.5:
                # Tangage positif : nez vers le bas (rotation autour de y, qui pointe à gauche).
                self._altitude -= self.localisation.vitesse * math.sin(tangage) * dt
        self._t_altitude = message.t
        self._tangage = tangage
        self._attitudes.append(Attitude(message.t, roulis, tangage, self._altitude))

    def attitude_a(self, t: float) -> Attitude:
        """Attitude à l'instant `t`, altitude corrigée."""
        if not self._attitudes:
            return Attitude(t, 0.0, 0.0, self.altitude)
        a = min(self._attitudes, key=lambda a: abs(a.t - t))
        return replace(a, z=a.z + self._correction_altitude)

    @property
    def altitude(self) -> float:
        """Altitude estimée du sol sous le rover, par rapport au départ (m)."""
        return self._altitude + self._correction_altitude

    def _corriger_altitude(self, message: Gnss) -> None:
        """Filtre complémentaire : l'altitude intégrée (rapide, dérive) est ramenée vers celle du
        GNSS (lente, bruitée de quelques centimètres en RTK)."""
        if message.z is None or message.sigma_z is None or not self._attitudes:
            return
        self._altitude_gnss = True
        a = self.attitude_a(message.t)
        gx, gy, gz = self.fiche.capteurs.gnss.position
        # Hauteur de l'antenne au-dessus du sol, rover incliné.
        cr, sr = math.cos(a.roulis), math.sin(a.roulis)
        cp, sp = math.cos(a.tangage), math.sin(a.tangage)
        hauteur = -sp * gx + cp * sr * gy + cp * cr * gz
        ecart = message.z - hauteur - a.z
        gain = 0.1 if message.sigma_z <= 0.05 else 0.01
        self._correction_altitude += gain * ecart

    def _sur_stereo(self, message: ImageStereo) -> None:
        if self.stereo is None:
            return
        self._stereo_en_attente = TravailStereo(
            message, self.localisation.pose_a(message.t), self.attitude_a(message.t)
        )

    def _sur_gnss(self, message: Gnss) -> None:
        self._corriger_altitude(message)
        rejets_avant = self.localisation.gnss_rejetes
        self.localisation.sur_gnss(message)
        rejets = self.localisation.gnss_rejetes
        if rejets > rejets_avant and rejets % 20 == 1:
            self.journal("Mesure GNSS incohérente rejetée par la localisation.", "alerte")

    def _sur_scan(self, message: Scan) -> None:
        pose = self.localisation.pose_a(message.t)
        lx, ly, _ = self.fiche.capteurs.lidar.position
        c, s = math.cos(pose.cap), math.sin(pose.cap)
        pose_lidar = PoseCapteur(pose.x + c * lx - s * ly, pose.y + s * lx + c * ly, pose.cap)
        sol = self._echos_au_sol(message, pose)
        self._mesurer_espace_libre(message, sol)
        demande = self._demande_plan(pose)
        poids = GrilleOccupation.poids_pour(pose.sigma_xy)
        self._travail = TravailPerception(message, pose_lidar, pose, poids, demande, sol=sol)

    def _echos_au_sol(self, scan: Scan, pose: Pose) -> np.ndarray | None:
        """Échos du lidar qui sont le sol, pas un obstacle. Le plan de mesure penche avec le
        rover : en descente (ou en montée, vers l'arrière), les faisceaux plongent et touchent
        le terrain en bas de la pente. Deux règles :
        - tout autour : l'impact est plus bas que le sol sous le rover (le faisceau descendait) ;
        - devant : l'impact est posé sur le sol vu par la caméra (face à une montée)."""
        d = np.array([np.nan if v is None else v for v in scan.distances])
        a = scan.angle_min + scan.pas * np.arange(d.size)
        ok = ~np.isnan(d)
        if not ok.any():
            return None
        lx, ly, lz = self.fiche.capteurs.lidar.position
        points = np.column_stack(
            [lx + d[ok] * np.cos(a[ok]), ly + d[ok] * np.sin(a[ok]), np.full(ok.sum(), lz)]
        )
        att = self.attitude_a(scan.t)
        monde = points @ rotation(att.roulis, att.tangage, pose.cap).T + np.array(
            [pose.x, pose.y, att.z]
        )
        au_sol = monde[:, 2] < att.z + HAUTEUR_IMPACT_SOL_M
        if self.stereo is not None and self.hauteurs.nuages:
            au_sol |= self.hauteurs.echos_au_sol(monde)
        sol = np.zeros(d.size, dtype=bool)
        sol[np.flatnonzero(ok)] = au_sol
        return sol if sol.any() else None

    def _mesurer_espace_libre(self, scan: Scan, sol: np.ndarray | None = None) -> None:
        """Espace libre devant le pare-chocs (couloir du rover) et obstacle le plus proche."""
        g = self.fiche.geometrie
        d = np.array([np.nan if v is None else v for v in scan.distances])
        if sol is not None:
            d[sol] = np.nan
        a = scan.angle_min + scan.pas * np.arange(d.size)
        x, y = d * np.cos(a), d * np.sin(a)
        lx = self.fiche.capteurs.lidar.position[0]
        couloir = (np.abs(y) < g.demi_largeur + 0.1) & (x > 0)
        avant = x[couloir] - (g.longueur / 2 - lx)
        libre_lidar = float(np.nanmin(avant)) if np.any(~np.isnan(avant)) else math.inf
        self._obstacle_min = float(np.nanmin(d)) if np.any(~np.isnan(d)) else math.inf
        # Obstacles bas vus par la caméra (sous le plan du lidar) dans le même couloir.
        self._libre_stereo = math.inf
        if self.stereo is not None and self.hauteurs.nuages:
            p = self.localisation.pose()
            self._libre_stereo = self.hauteurs.distance_libre(
                p.x, p.y, p.cap, g.demi_largeur + 0.05, g.longueur / 2, PORTEE_LIBRE_STEREO_M
            )
        self._libre_avant = min(libre_lidar, self._libre_stereo)

    def _sur_firmware(self, message: Firmware) -> None:
        precedent = self._firmware.arret if self._firmware else None
        if message.arret != precedent and message.arret is not None:
            textes = {
                "obstacle_avant": "Arrêt d'urgence du contrôleur : obstacle devant.",
                "obstacle_arriere": "Arrêt d'urgence du contrôleur : obstacle derrière.",
                "chien_de_garde": "Chien de garde du contrôleur : consignes interrompues.",
                "arret_operateur": "Arrêt demandé par l'opérateur.",
            }
            self.journal(textes[message.arret], "alerte")
        self._firmware = message

    def _publier_periodiques(self) -> None:
        if self.t - self._t_pose >= PERIODE_POSE_S:
            self._t_pose = self.t
            p = self.localisation.pose()
            self.sortie.append(
                PoseEstimee(
                    t=p.t,
                    x=round(p.x, 4),
                    y=round(p.y, 4),
                    cap=round(p.cap, 5),
                    sigma_xy=round(p.sigma_xy, 4),
                    sigma_cap=round(p.sigma_cap, 5),
                    vitesse=round(p.vitesse, 3),
                    source="ekf_gnss_imu_odometrie",
                )
            )
        if self.t - self._t_mesures >= PERIODE_MESURES_S:
            self._t_mesures = self.t
            self.sortie.append(Mesures(t=self.t, valeurs=self.mesures()))

    def mesures(self) -> dict[str, float | int | None]:
        p = self.localisation.pose()
        m = self.mission
        obstacle = None if math.isinf(self._obstacle_min) else round(self._obstacle_min, 2)
        valeurs: dict[str, float | int | None] = {
            "distance_parcourue": round(self._distance_parcourue, 2),
            "sigma_position": round(p.sigma_xy, 4),
            "gnss_rejetes": self.localisation.gnss_rejetes,
            "scans_integres": self._scans_integres,
            "nb_calculs": self._nb_calculs_total,
            "cpu_cerveau": None if self.charge_cpu is None else round(self.charge_cpu, 4),
            "distance_min_obstacle": obstacle,
            "analyses_vision": self.analyses_vision,
            "obstacles_vision": len(self.couche_vision.obstacles),
            "paires_stereo": self.paires_traitees,
            "altitude": round(self.altitude, 3),
        }
        if self._duree_stereo_ms is not None:
            valeurs["temps_stereo_ms"] = round(self._duree_stereo_ms, 1)
        if self._derniere_duree_plan_ms is not None:
            valeurs["temps_calcul_ms"] = round(self._derniere_duree_plan_ms, 1)
        if m is not None:
            cible = m.objectif_effectif or m.objectif
            valeurs["distance_objectif"] = round(math.hypot(cible[0] - p.x, cible[1] - p.y), 2)
            valeurs["duree_mission"] = round(self.t - m.t_debut, 1)
        return valeurs

    # ================================================================== perception (flux lent)
    def _demande_plan(self, pose: Pose) -> DemandePlan | None:
        m = self.mission
        if m is None or self.etat not in ("calcul", "route") or self._recul is not None:
            return None
        if not self._forcer_plan and self.t - self._t_dernier_plan < PERIODE_PLANIFICATION_S:
            return None
        chemin = tuple((float(x), float(y)) for x, y in self.suivi.points)
        return DemandePlan(
            mission_id=m.id,
            objectif=m.objectif,
            depart=(pose.x, pose.y),
            cap=pose.cap,
            chemin=chemin,
            indice=self.suivi.indice,
            forcer=self._forcer_plan or not self.suivi.actif,
            t_dernier_plan=self._t_dernier_plan,
            obstacles_vision=self.couche_vision.masque if self.couche_vision.obstacles else None,
        )

    def prendre_travail(self) -> TravailPerception | None:
        """Retire le balayage en attente (fil principal), avec la dernière paire stéréo reçue."""
        travail, self._travail = self._travail, None
        if travail is not None and self._stereo_en_attente is not None:
            travail = replace(travail, stereo=self._stereo_en_attente)
            self._stereo_en_attente = None
        if travail is not None and travail.demande is not None:
            self._t_dernier_plan = travail.scan.t
            self._forcer_plan = False
        return travail

    def traiter_perception(self, travail: TravailPerception) -> ResultatPerception:
        """Carte puis, si demandé, chemin. Ne lit que la grille et la demande : exécutable dans
        un autre fil."""
        resultat = ResultatPerception()
        self.grille.liberer_disque(
            travail.pose_rover.x, travail.pose_rover.y, self.fiche.geometrie.demi_largeur
        )
        self.hauteurs.liberer_disque(
            travail.pose_rover.x, travail.pose_rover.y, self.fiche.geometrie.demi_largeur
        )
        resultat.echos = self.grille.integrer(
            travail.pose_lidar, travail.scan, travail.poids, travail.sol
        )
        if travail.stereo is not None:
            self._integrer_stereo(travail.stereo, travail.poids, resultat)
        if travail.scan.t - self._t_carte >= PERIODE_CARTE_S:
            self._t_carte = travail.scan.t
            resultat.evenements.append(self.evenement_carte(travail.scan.t))
        if travail.demande is not None:
            self._planifier(travail.demande, travail.scan.t, resultat)
        return resultat

    def _integrer_stereo(
        self, travail: TravailStereo, poids: float, resultat: ResultatPerception
    ) -> None:
        """Profondeur, nuage dans le repère du monde, carte des hauteurs et traversabilité."""
        assert self.stereo is not None
        if poids <= 0:
            return  # position trop incertaine : on ne cartographie pas
        try:
            profondeur = self.stereo.calculer(travail.message)
        except ValueError as erreur:
            resultat.evenements.append(
                Journal(t=travail.message.t, niveau="alerte", texte=f"Paire stéréo : {erreur}.")
            )
            return
        p, a = travail.pose, travail.attitude
        points, distances = self.stereo.nuage(profondeur)
        monde = points @ rotation(a.roulis, a.tangage, p.cap).T + np.array([p.x, p.y, a.z])
        resultat.decalage_altitude = self.hauteurs.ajouter_nuage(
            monde, distances, profondeur.t, recaler=not self._altitude_gnss
        )
        # Trous probables (bords d'occultation) : désactivés tant que le scénario n'a pas de fossé.
        # Sur un sol ondulé, ils s'allument puis s'éteignent et font changer le chemin de côté.
        vide = np.zeros((0, 3))
        proches, lointains = vide, vide
        if DETECTER_TROUS:
            proches, lointains = self.stereo.bords_occultation(profondeur)
        if len(proches):
            rot = rotation(a.roulis, a.tangage, p.cap).T
            decalage = np.array([p.x, p.y, a.z])
            self.hauteurs.signaler_trous(
                proches @ rot + decalage, lointains @ rot + decalage, profondeur.t
            )
        self.hauteurs.classer(p.x, p.y, RAYON_CLASSEMENT_M)
        self.paires_traitees += 1
        self._duree_stereo_ms = profondeur.duree_ms
        g = self.fiche.geometrie
        libre = self.hauteurs.distance_libre(
            p.x, p.y, p.cap, g.demi_largeur + 0.05, g.longueur / 2, PORTEE_LIBRE_STEREO_M
        )
        portee = self.stereo.stereo.portee_max
        resultat.evenements.append(
            Profondeur(
                t=profondeur.t,
                image=base64.b64encode(image_profondeur(profondeur, portee)).decode("ascii"),
                portee=portee,
                duree_ms=round(profondeur.duree_ms, 1),
                points=int(points.shape[0]),
                obstacles_devant=None if math.isinf(libre) else round(libre, 2),
            )
        )
        if profondeur.t - self._t_traversabilite >= PERIODE_TRAVERSABILITE_S:
            self._t_traversabilite = profondeur.t
            resultat.evenements.append(self.evenement_traversabilite(profondeur.t))

    def _preferer_l_avant(self, carte: CarteCouts, d: DemandePlan, plan: Plan) -> Plan | None:
        """Si le chemin part vers l'arrière du rover, essaie le même calcul depuis un point situé
        devant lui : un léger détour vaut mieux qu'un demi-tour en trois temps."""
        if len(plan.points) < 3:
            return None
        cible = next((q for q in plan.points if math.dist(q, d.depart) > 1.0), plan.points[-1])
        angle = math.atan2(cible[1] - d.depart[1], cible[0] - d.depart[0]) - d.cap
        if abs(math.atan2(math.sin(angle), math.cos(angle))) < math.radians(100):
            return None
        avant = (
            d.depart[0] + AVANCE_DEPART_M * math.cos(d.cap),
            d.depart[1] + AVANCE_DEPART_M * math.sin(d.cap),
        )
        if carte.distance_en(*avant) < self.parametres_planification.rayon_gonflement:
            return None
        try:
            devant = planifier(carte, avant, d.objectif, self.parametres_planification)
        except EchecPlanification:
            return None
        if devant.cout > 1.3 * plan.cout + 2.0:
            return None
        return replace(
            devant,
            points=[d.depart, *devant.points],
            longueur=devant.longueur + AVANCE_DEPART_M,
        )

    def evenement_traversabilite(self, t: float, complete: bool = False) -> Traversabilite:
        extrait = self.hauteurs.extraire_modifications(complet=complete)
        rect, donnees = extrait if extrait else ((0, 0, 0, 0), "")
        return Traversabilite(
            t=t,
            resolution=self.hauteurs.resolution,
            origine=self.hauteurs.origine,
            largeur=self.hauteurs.largeur,
            hauteur=self.hauteurs.hauteur,
            rect=rect,
            donnees=donnees,
        )

    def _planifier(self, d: DemandePlan, t: float, resultat: ResultatPerception) -> None:
        p = self.parametres_planification
        supplementaires = d.obstacles_vision
        sol_verifie = None
        if self.hauteurs.nuages:
            stereo = self.hauteurs.obstacles()
            supplementaires = stereo if supplementaires is None else supplementaires | stereo
            sol_verifie = self.hauteurs.sol_libre_proche()
        carte = construire_carte_couts(self.grille, p, supplementaires, sol_verifie)
        chemin = list(d.chemin)
        raison = ""
        if d.forcer or len(chemin) < 2:
            raison = "initial"
        elif chemin_bloque(carte, chemin, d.indice + 3, horizon_m=12.0):
            raison = "bloque"
        elif t - d.t_dernier_plan >= PERIODE_AMELIORATION_S:
            raison = "amelioration"
        if not raison:
            return
        resultat.mission_id = d.mission_id
        try:
            plan = planifier(carte, d.depart, d.objectif, p)
        except EchecPlanification as erreur:
            resultat.echec = str(erreur)
            resultat.depart_bloque = isinstance(erreur, DepartBloque)
            return
        plan = self._preferer_l_avant(carte, d, plan) or plan
        if raison == "amelioration":
            actuel = _cout_chemin(carte, chemin[d.indice :])
            if _cout_chemin(carte, plan.points) > GAIN_AMELIORATION * actuel:
                return
        resultat.plan = plan
        resultat.raison_plan = raison

    def evenement_carte(self, t: float, complete: bool = False) -> Carte:
        extrait = self.grille.extraire_modifications(complet=complete)
        rect, donnees = extrait if extrait else ((0, 0, 0, 0), "")
        return Carte(
            t=t,
            resolution=self.grille.resolution,
            origine=self.grille.origine,
            largeur=self.grille.largeur,
            hauteur=self.grille.hauteur,
            rect=rect,
            donnees=donnees,
        )

    def appliquer(self, resultat: ResultatPerception) -> None:
        """Applique le résultat du flux lent (fil principal)."""
        self._scans_integres += 1
        # Recalage de l'altitude sur le sol déjà cartographié.
        self._correction_altitude -= resultat.decalage_altitude
        self.sortie.extend(resultat.evenements)
        m = self.mission
        if resultat.mission_id is None or m is None or resultat.mission_id != m.id:
            return
        if self.etat not in ("calcul", "route") or self._recul is not None:
            return
        if resultat.echec is not None and resultat.depart_bloque:
            # Collé à un obstacle (souvent découvert tard par la caméra) : s'en écarter d'abord.
            self._demarrer_recul("obstacle contre le rover")
            return
        if resultat.echec is not None:
            m.echecs_calcul += 1
            self.journal(f"Aucun chemin trouvé ({resultat.echec}).", "alerte")
            if m.echecs_calcul >= ECHECS_CALCUL_MAX:
                self._terminer_en_echec("aucun chemin vers l'objectif avec la carte actuelle")
            else:
                self._forcer_plan = True
            return
        plan = resultat.plan
        if plan is None:
            return
        m.echecs_calcul = 0
        m.nb_calculs += 1
        m.duree_calculs_ms.append(plan.duree_ms)
        self._nb_calculs_total += 1
        self._derniere_duree_plan_ms = plan.duree_ms
        if m.objectif_effectif is None and plan.objectif_deplace > 0.05:
            self.journal(
                "Objectif dans un obstacle : visée de la cellule libre la plus proche.",
                "alerte",
                deplacement_m=round(plan.objectif_deplace, 2),
            )
        m.objectif_effectif = plan.objectif
        # La progression se mesure sur le chemin courant : un détour rallonge la distance restante.
        m.meilleure_restante = math.inf
        m.t_progres = self.t
        self.suivi.definir_chemin(plan.points)
        self._publier_chemin(plan.points, plan.objectif)
        textes = {
            "initial": "Chemin calculé",
            "bloque": "Nouveau calcul : obstacle découvert sur le chemin",
            "amelioration": "Nouveau calcul : chemin plus court trouvé",
        }
        self.journal(
            f"{textes[resultat.raison_plan]} en {plan.duree_ms:.0f} ms.",
            longueur_m=round(plan.longueur, 1),
            noeuds=plan.noeuds_explores,
        )
        if self.etat == "calcul":
            self._statut("route", f"En route vers {m.libelle} ({plan.longueur:.0f} m).")

    # ================================================================== vision
    def _sur_image(self, message: Image) -> None:
        if self.camera is None:
            return
        self._derniere_image = message
        self._preparer_demande_vision()

    def _preparer_demande_vision(self) -> None:
        image = self._derniere_image
        if (
            not self.vision_disponible
            or image is None
            or self._vision_en_cours is not None
            or self._demande_vision is not None
            or self.t - image.t > AGE_MAX_IMAGE_S
        ):
            return
        if self._analyse_operateur:
            raison: Literal["vigilance", "operateur"] = "operateur"
        elif (
            self.vigilance
            and self.mission is not None
            and self.etat in ("calcul", "route")
            and image.t - self._t_derniere_analyse >= self.periode_vigilance
            and self.t >= self._vision_pause_jusqu_a
        ):
            raison = "vigilance"
        else:
            return
        pose = self.localisation.pose_a(image.t)
        self._compteur_vision += 1
        self._demande_vision = DemandeVision(
            id=self._compteur_vision,
            t=image.t,
            image_jpeg_b64=image.donnees,
            largeur=image.largeur,
            hauteur=image.hauteur,
            x=pose.x,
            y=pose.y,
            cap=pose.cap,
            raison=raison,
            contexte=self._contexte_mission(),
        )
        self._analyse_operateur = False
        self._t_derniere_analyse = image.t
        self._publier_etat_vision()

    def _contexte_mission(self) -> str:
        m = self.mission
        if m is None:
            return "Pas de mission en cours : le rover est à l'arrêt."
        if self.recherche is not None:
            return (
                f"Mission en cours : explorer la zone pour trouver {self.recherche[1]}. S'il est "
                "visible, liste-le dans objets avec une boîte précise."
            )
        texte = f"Mission en cours : aller vers {m.libelle}"
        if self.etapes:
            texte += ", puis " + ", ".join(e.libelle for e in self.etapes)
        return texte + "."

    def prendre_demande_vision(self) -> DemandeVision | None:
        """Image à analyser, s'il y en a une (couche d'exécution)."""
        demande, self._demande_vision = self._demande_vision, None
        if demande is not None:
            self._vision_en_cours = demande
        return demande

    def demander_analyse(self) -> str | None:
        """Analyse de la prochaine image, à la demande de l'opérateur ; renvoie un refus."""
        if self.camera is None:
            return "ce rover n'a pas de caméra"
        if not self.vision_disponible:
            return "aucun modèle de vision n'est configuré"
        if self._vision_en_cours is not None or self._demande_vision is not None:
            return "une analyse de la caméra est déjà en cours"
        self._analyse_operateur = True
        self._preparer_demande_vision()
        return None

    def definir_vigilance(self, active: bool) -> str | None:
        if active and (self.camera is None or not self.vision_disponible):
            return "aucun modèle de vision n'est configuré"
        if active != self.vigilance:
            self.vigilance = active
            self.journal(
                "Observation IA activée : une image analysée toutes les "
                f"{self.periode_vigilance:.0f} s en mission."
                if active
                else "Observation IA coupée."
            )
        self._publier_etat_vision()
        return None

    def _publier_etat_vision(self, message: str = "") -> None:
        en_cours = self._vision_en_cours is not None or self._demande_vision is not None
        if not message:
            if not self.vision_disponible:
                message = "Aucun modèle de vision configuré."
            elif en_cours:
                message = "Analyse de l'image en cours…"
            elif self.t < self._vision_pause_jusqu_a:
                reprise = self._vision_pause_jusqu_a
                message = f"En pause jusqu'à t = {reprise:.0f} s (limite du fournisseur)."
            else:
                message = (
                    f"Observation active : une image toutes les {self.periode_vigilance:.0f} s "
                    "en mission."
                    if self.vigilance
                    else "Observation coupée."
                )
        self.sortie.append(
            EtatVision(
                t=self.t,
                disponible=self.vision_disponible and self.camera is not None,
                vigilance=self.vigilance,
                en_cours=en_cours,
                analyses=self.analyses_vision,
                message=message,
            )
        )

    def sur_resultat_vision(self, demande: DemandeVision, resultat: ResultatVision) -> str:
        """Intègre le résultat d'une analyse ; renvoie le texte de réponse composé par le code."""
        if self._vision_en_cours is not None and self._vision_en_cours.id == demande.id:
            self._vision_en_cours = None
        if resultat.analyse is None or self.camera is None:
            erreur = resultat.erreur or "pas de caméra"
            if resultat.reessayer_dans_s:
                self._vision_pause_jusqu_a = self.t + resultat.reessayer_dans_s
            self.journal(f"Analyse de la caméra impossible : {erreur}.", "alerte")
            self.sortie.append(self._evenement_vision(demande, resultat, [], erreur))
            self._publier_etat_vision()
            return f"Je n'ai pas pu analyser l'image : {erreur}."
        self.analyses_vision += 1
        observations = placer_objets(
            resultat.analyse, demande, self.camera, self._compteur_observations + 1
        )
        # Un obstacle sous le rover *au moment de la prise de vue* est impossible (la caméra ne
        # voit pas sous lui) : détection écartée. Surtout pas la pose actuelle : le modèle répond
        # en plusieurs secondes, le rover a pu atteindre entre-temps un obstacle bien réel.
        rayon = self.fiche.geometrie.rayon_englobant
        observations = [
            replace(o, dans_carte=False)
            if o.dans_carte and distance_point_polygone((demande.x, demande.y), o.contour) < rayon
            else o
            for o in observations
        ]
        self._compteur_observations += len(resultat.analyse.objets)
        ajouts = self.couche_vision.fusionner(observations)
        for obs in self.couche_vision.constater_absences(
            demande, self.camera, observations, self.grille.obstacle_entre
        ):
            self.journal(
                f"Obstacle de la caméra retiré ({obs.nom}) : la zone est nettement visible et il "
                "n'y est plus vu (fausse détection probable).",
                "alerte",
            )
        for obs in ajouts:
            self.journal(
                f"Caméra : {obs.nom} à {obs.distance:.1f} m, ajouté à la carte "
                "(obstacle trop bas pour le lidar).",
                "alerte",
                incertitude_m=round(obs.incertitude, 2),
                confiance=round(obs.confiance, 2),
            )
        self.sortie.append(self._evenement_vision(demande, resultat, observations, None))
        self._publier_etat_vision()
        self._appliquer_decision(demande, resultat, observations)
        self._verifier_recherche(observations)
        return decrire(resultat.analyse, observations)

    def _verifier_recherche(self, observations: list[Observation]) -> None:
        """Mission de recherche : la cible vue par la caméra (confiance suffisante) arrête
        l'exploration ; l'opérateur en est averti dans le chat."""
        if self.recherche is None or self.mission is None:
            return
        cible = self.recherche[0]
        trouves = [
            o for o in observations if o.categorie == cible and o.confiance >= CONFIANCE_TROUVE
        ]
        if not trouves:
            return
        o = min(trouves, key=lambda o: o.distance)
        self.recherche = None
        self.etapes = []
        self._etapes_suspendues = []
        self.mission = None
        self._recul = None
        self.suivi.definir_chemin([])
        self._publier_chemin([], None)
        texte = (
            f"Trouvé : {o.nom} à {o.distance:.0f} m devant moi (position {o.x:.1f} ; {o.y:.1f}). "
            "Je me suis arrêté."
        )
        self.journal(texte, "succes", confiance=round(o.confiance, 2))
        self._statut("atteint", texte)
        self.sortie.append(
            ChatReponse(t=self.t, texte=texte, refus=None, fournisseur="observation IA")
        )

    def _appliquer_decision(
        self, demande: DemandeVision, resultat: ResultatVision, observations: list[Observation]
    ) -> None:
        """Note de l'IA d'observation et validation de sa décision par le code : seulement en
        mission, et seulement pour les analyses périodiques (pas pour « que vois-tu ? »)."""
        analyse = resultat.analyse
        assert analyse is not None
        decision = analyse.decision
        raison = analyse.raison_decision or "sans raison donnée"
        appliquee = False
        if demande.raison == "vigilance" and self.mission is not None:
            if decision == "s_arreter":
                self._arret_par_ia(raison)
                appliquee = True
            elif decision == "ralentir":
                self._ralenti_jusqu_a = self.t + DUREE_RALENTI_S
                self.journal(
                    f"IA d'observation : ralentir ({raison}).",
                    "alerte",
                    vitesse_max=VITESSE_RALENTIE,
                )
                appliquee = True
        reperes = sorted({o.nom for o in observations if o.categorie in CATEGORIES_NOTABLES})
        self.sortie.append(
            NoteIA(
                t=self.t,
                t_image=demande.t,
                x=round(demande.x, 2),
                y=round(demande.y, 2),
                texte=analyse.resume,
                decision=decision,
                raison=raison,
                appliquee=appliquee,
                reperes=reperes,
                fournisseur=resultat.fournisseur,
            )
        )

    def _arret_par_ia(self, raison: str) -> None:
        """Arrêt décidé par l'IA d'observation : la mission est suspendue, l'opérateur décide."""
        self._teleop = None
        self._recul = None
        self.journal(f"Arrêt décidé par l'IA d'observation : {raison}.", "alerte")
        self._etapes_suspendues, self.etapes = self.etapes, []
        self._recherche_suspendue, self.recherche = self.recherche, None
        self.mission = None
        self.suivi.definir_chemin([])
        self._publier_chemin([], None)
        self._statut("attente", f"Arrêt par l'IA : {raison}. « Reprendre » pour continuer.")

    def _evenement_vision(
        self,
        demande: DemandeVision,
        resultat: ResultatVision,
        observations: list[Observation],
        erreur: str | None,
    ) -> Vision:
        return Vision(
            t=self.t,
            demande=demande.id,
            raison=demande.raison,
            t_image=demande.t,
            fournisseur=resultat.fournisseur,
            duree_ms=round(resultat.duree_ms, 1),
            resume=resultat.analyse.resume if resultat.analyse else None,
            erreur=erreur,
            objets=[
                ObjetPlace(
                    categorie=o.categorie,
                    nom=o.nom,
                    boite=o.boite,
                    x=round(o.x, 3),
                    y=round(o.y, 3),
                    distance=round(o.distance, 2),
                    incertitude=round(o.incertitude, 2),
                    bloque=o.bloque,
                    confiance=round(o.confiance, 2),
                    dans_carte=o.dans_carte,
                )
                for o in observations
            ],
            obstacles=[
                ObstacleVision(
                    id=o.id,
                    categorie=o.categorie,
                    nom=o.nom,
                    contour=[(round(x, 3), round(y, 3)) for x, y in o.contour],
                    marge=round(o.marge, 3),
                    t=o.t,
                )
                for o in self.couche_vision.obstacles
            ],
            image=demande.image_jpeg_b64,
        )

    # ================================================================== opérateur
    def definir_objectif(
        self,
        x: float,
        y: float,
        origine: OrigineObjectif = "carte",
        libelle: str = "",
        garder_etapes: bool = False,
    ) -> str | None:
        """Nouvel objectif ; renvoie la raison du refus, ou None s'il est accepté. Un objectif
        donné hors d'un parcours annule les étapes restantes."""
        if not self.scenario.limites.contient(x, y, MARGE_LIMITES):
            return "hors de la zone de mission"
        if not garder_etapes:
            self.etapes = []
            self._etapes_suspendues = []
            self.recherche = None
            self._recherche_suspendue = None
        self._compteur_missions += 1
        pose = self.localisation.pose()
        if not libelle:
            proche = min(self.scenario.lieux, key=lambda lieu: math.hypot(lieu.x - x, lieu.y - y))
            if math.hypot(proche.x - x, proche.y - y) < 2.0:
                libelle = proche.nom
        libelle = libelle or f"({x:.1f} ; {y:.1f})"
        self.mission = Mission(
            id=self._compteur_missions,
            objectif=(x, y),
            origine=origine,
            t_debut=self.t,
            libelle=libelle,
            t_progres=self.t,
            distance_depart=self._distance_parcourue,
        )
        self.derniere_mission = self.mission
        self._teleop = None
        self._recul = None
        self._forcer_plan = True
        self.suivi.definir_chemin([])
        self._statut("calcul", f"Calcul du chemin vers {libelle}…")
        self.journal(
            f"Objectif reçu : {libelle}.",
            distance_m=round(math.hypot(x - pose.x, y - pose.y), 1),
            origine=origine,
        )
        return None

    def sur_teleoperation(self, message: Teleoperation) -> None:
        actif = abs(message.avance) > 0.01 or abs(message.direction) > 0.01
        if actif and self.etat != "manuel":
            if self.mission is not None:
                self.journal("Mission suspendue : l'opérateur reprend la main.")
            self.mission = None
            self._recul = None
            self.etapes = []
            self.suivi.definir_chemin([])
            self._publier_chemin([], None)
            self._statut("manuel", "Conduite manuelle.")
        self._teleop = message
        self._t_teleop = self.t

    def sur_stop(self, _: Stop | None = None) -> None:
        self._teleop = None
        self._recul = None
        if self.mission is not None:
            self.journal("Mission arrêtée par l'opérateur.", "alerte")
        self._etapes_suspendues, self.etapes = self.etapes, []
        self._recherche_suspendue, self.recherche = self.recherche, None
        self.mission = None
        self.suivi.definir_chemin([])
        self._publier_chemin([], None)
        self._statut("attente", "Arrêté. « Reprendre » relance le dernier objectif.")

    def executer(self, action: Action) -> str | None:
        """Exécute une action issue d'une consigne validée ; renvoie la raison d'un refus."""
        if isinstance(action, AllerA):
            return self.definir_objectif(action.x, action.y, "chat", action.libelle)
        if isinstance(action, Chercher):
            premiere, *reste = action.etapes
            refus = self.definir_objectif(premiere.x, premiere.y, "chat", premiere.libelle)
            if refus is None:
                self.etapes = list(reste)
                self.recherche = (action.cible, action.nom)
            return refus
        if isinstance(action, Parcours):
            premiere, *suite = action.etapes
            refus = self.definir_objectif(premiere.x, premiere.y, "chat", premiere.libelle)
            if refus is None:
                self.etapes = list(suite)
            return refus
        if isinstance(action, Arreter):
            self.sur_stop()
            return None
        if isinstance(action, Observer):
            return self.demander_analyse()
        if isinstance(action, Vigilance):
            return self.definir_vigilance(action.active)
        return self.reprendre()

    def pose_courante(self) -> PoseCourante:
        p = self.localisation.pose()
        return PoseCourante(p.x, p.y, p.cap)

    def reprendre(self) -> str | None:
        m = self.derniere_mission
        if m is None or self.mission is not None:
            return "aucune mission interrompue à reprendre"
        suite, recherche = self._etapes_suspendues, self._recherche_suspendue
        refus = self.definir_objectif(m.objectif[0], m.objectif[1], "reprise", m.libelle)
        if refus is None:
            self.etapes = suite
            self.recherche = recherche
        return refus

    def _etape_suivante(self) -> bool:
        """Passe à l'étape suivante d'un parcours ; faux s'il n'y en a plus."""
        if not self.etapes:
            return False
        suivante = self.etapes.pop(0)
        suite = self.etapes
        self.definir_objectif(suivante.x, suivante.y, "chat", suivante.libelle, garder_etapes=True)
        self.etapes = suite
        if not suivante.libelle.startswith("point "):  # un trajet : pas une ligne par point
            reste = f" ({len(suite)} ensuite)" if suite else " (dernière étape)"
            self.journal(f"Étape suivante : {suivante.libelle}{reste}.")
        return True

    # ================================================================== contrôle (25 Hz)
    def _terminer_en_echec(self, raison: str) -> None:
        self.journal(f"Échec de la mission : {raison}.", "erreur")
        self.mission = None
        self._recul = None
        self.suivi.definir_chemin([])
        self._publier_chemin([], None)
        self._statut("echec", f"Échec : {raison}. Donne un autre objectif.")
        if self.etapes:
            self.journal("Étape impossible : je passe à la suivante.", "alerte")
            self._etape_suivante()

    def _demarrer_recul(self, raison: str) -> None:
        m = self.mission
        if m is None:
            return
        m.recuperations += 1
        if m.recuperations > RECUPERATIONS_MAX:
            self._terminer_en_echec(
                f"rover bloqué ({raison}) malgré {RECUPERATIONS_MAX} dégagements"
            )
            return
        pose = self.localisation.pose()
        # Recul en braquant du côté opposé au virage demandé, pour se dégager en biais.
        braquage = -0.5 * self._dernier_braquage if abs(self._dernier_braquage) > 0.1 else 0.0
        self._recul = Recul(t_debut=self.t, x0=pose.x, y0=pose.y, braquage=braquage)
        self._statut("bloque", f"Rover coincé ({raison}) : recul d'un mètre, puis nouveau calcul.")
        self.journal(f"Rover coincé ({raison}) : dégagement n° {m.recuperations}.", "alerte")

    def _fin_demi_tour(self, r: Recul, parcouru: float, t: float, pose: Pose) -> Commande:
        """Fin d'une manœuvre de demi-tour. Arrêtée presque aussitôt par un obstacle, elle est
        suivie de la manœuvre inverse (marche avant braquée de l'autre côté) ; si rien ne passe,
        la mission échoue au lieu de boucler."""
        arret = Commande(t=t, vitesse=0.0, braquage=0.0)
        m = self.mission
        assert m is not None
        if parcouru >= 0.3:
            m.demi_tours_bloques = 0
            self._statut("route", f"En route vers {m.libelle}.")
            return arret
        m.demi_tours_bloques += 1
        if m.demi_tours_bloques > DEMI_TOURS_BLOQUES_MAX:
            self._terminer_en_echec("demi-tour impossible : obstacles devant et derrière")
            return arret
        vitesse = -r.vitesse
        libre = self._libre_avant if vitesse > 0 else 1.0
        self._recul = Recul(
            t_debut=t,
            x0=pose.x,
            y0=pose.y,
            distance=max(0.3, min(0.9, libre - 0.4)),
            vitesse=vitesse,
            braquage=-r.braquage,
            volontaire=True,
        )
        self._statut("route", "Demi-tour : manœuvre inverse, la précédente était bloquée.")
        return arret

    def _controle(self, t: float) -> Commande:
        arret = Commande(t=t, vitesse=0.0, braquage=0.0)
        if self.etat == "manuel":
            teleop, lim = self._teleop, self.fiche.limites
            if teleop is None or t - self._t_teleop >= DELAI_TELEOP_S:
                return arret
            v_max = lim.vitesse_max if teleop.avance >= 0 else lim.vitesse_arriere_max
            return Commande(
                t=t, vitesse=teleop.avance * v_max, braquage=teleop.direction * lim.braquage_max
            )

        m = self.mission
        if m is None:
            return arret
        pose = self.localisation.pose()

        if self._recul is not None:
            r = self._recul
            recule = math.hypot(pose.x - r.x0, pose.y - r.y0)
            sens_bloque = "obstacle_arriere" if r.vitesse < 0 else "obstacle_avant"
            bloque = self._firmware is not None and self._firmware.arret == sens_bloque
            if recule >= r.distance or t - r.t_debut > r.duree_max or bloque:
                self._recul = None
                m.t_progres = t
                m.meilleure_restante = math.inf
                m.t_arret_obstacle = None
                if r.volontaire:
                    return self._fin_demi_tour(r, recule, t, pose)
                self._forcer_plan = True
                self.suivi.definir_chemin([])
                self._statut("calcul", f"Nouveau calcul du chemin vers {m.libelle}…")
                return arret
            return Commande(t=t, vitesse=r.vitesse, braquage=r.braquage)

        if self.etat != "route" or not self.suivi.actif:
            return arret

        c = self.suivi.consigne(pose.x, pose.y, pose.cap, pose.vitesse, self._libre_avant)
        self._dernier_braquage = c.braquage
        if abs(c.angle_cible) > math.pi / 2 and self._libre_avant < ESPACE_DEMI_TOUR_M:
            # Demi-tour en trois temps : marche arrière en braquant du côté opposé à la cible.
            self._recul = Recul(
                t_debut=t,
                x0=pose.x,
                y0=pose.y,
                distance=0.9,
                braquage=-math.copysign(self.fiche.limites.braquage_max, c.angle_cible),
                volontaire=True,
            )
            self._statut("route", "Demi-tour en trois temps : marche arrière braquée.")
            return arret
        # Étape intermédiaire d'un parcours ou d'un trajet : on la passe sans s'arrêter.
        tolerance = TOLERANCE_ETAPE if self.etapes else TOLERANCE_OBJECTIF
        if c.distance_restante < tolerance:
            duree = t - m.t_debut
            parcouru = self._distance_parcourue - m.distance_depart
            if not (self.etapes and m.libelle.startswith("point ")):
                self.journal(
                    f"Objectif atteint : {m.libelle}.",
                    "succes",
                    duree_s=round(duree, 1),
                    distance_m=round(parcouru, 1),
                    calculs=m.nb_calculs,
                )
            self.mission = None
            self.suivi.definir_chemin([])
            if self._etape_suivante():
                return arret
            self._statut(
                "atteint", f"Objectif atteint en {duree:.0f} s ({parcouru:.1f} m parcourus)."
            )
            return arret

        # Surveillance : progression vers l'objectif et arrêts d'urgence répétés.
        if c.distance_restante < m.meilleure_restante - 0.2:
            m.meilleure_restante = c.distance_restante
            m.t_progres = t
        elif t - m.t_progres > DELAI_SANS_PROGRES_S:
            self._demarrer_recul("aucun progrès depuis 6 s")
            return arret
        if self._firmware is not None and self._firmware.arret == "obstacle_avant":
            if m.t_arret_obstacle is None:
                m.t_arret_obstacle = t
            if t - m.t_arret_obstacle > DELAI_ARRET_OBSTACLE_S:
                self._demarrer_recul("obstacle devant")
                return arret
        else:
            m.t_arret_obstacle = None
        vitesse = c.vitesse
        if t < self._ralenti_jusqu_a:
            vitesse = min(vitesse, VITESSE_RALENTIE)
        return Commande(t=t, vitesse=round(vitesse, 4), braquage=round(c.braquage, 5))
