"""Assemblage du rover : matériel, sécurité, conduite, estimation, perception, mission, IA.

    rover = Rover(profil, simulation=False)
    rover.demarrer()        # fils du matériel, de la conduite, de l'estimation, de la stéréo…
    await rover.demarrer_ia(boucle)   # agent, vision, voix (dans la boucle asyncio du serveur)
    rover.arreter()

Un module qui manque ou tombe (stéréo non calibrée, IA sans clé…) est signalé dans le journal ;
le reste continue. Seule la sécurité et la conduite sont indispensables au mouvement.
"""

from __future__ import annotations

import asyncio
import copy
import math
import threading
import time
from collections import deque
from types import SimpleNamespace
from typing import Any

import numpy as np

from fieldmind.bus import Bus, S
from fieldmind.config import DOSSIER_UTILISATEUR, Profil
from fieldmind.controle.conduite import Conduite
from fieldmind.mission.actions import ActionRefusee
from fieldmind.mission.executeur import Executeur, Navigation
from fieldmind.securite.superviseur import Superviseur
from fieldmind.types import (
    Action,
    CommandeRoues,
    Gnss,
    ImageCamera,
    Imu,
    Obstacles,
    Odometrie,
    Pose2D,
    Scan,
)


class Estimation:
    """Fil d'estimation : odométrie à chaque tour du lidar, carte d'occupation à 2 Hz."""

    def __init__(self, bus: Bus, profil: Profil) -> None:
        from fieldmind.estimation.cartographie import Cartographe
        from fieldmind.estimation.odometrie import Odometre

        self.bus, self.profil = bus, profil
        self.odometre = Odometre(profil)
        self.cartographe = Cartographe(profil)
        self._verrou = threading.Lock()
        self._scans = bus.file(S.SCAN, 2)
        self._actif = False
        self.trajectoire: deque[tuple[float, float]] = deque(maxlen=2000)
        self.duree_icp_ms = 0.0
        self._derniere_carte = 0.0
        # Entrées venues d'autres fils (conduite à 20 Hz, lecteur de l'ESP à 50 Hz, GPS) : file
        # sans verrou, vidée par l'estimation avant chaque calcul. La conduite ne doit jamais
        # attendre l'ICP et la carte (jusqu'à 300 ms sous le verrou sur le Pi).
        self._entrees: deque[tuple[str, Any]] = deque(maxlen=2000)
        self._memoire_bas = np.zeros((0, 2), np.float32)
        self._fils = [threading.Thread(target=self._boucle, name="estimation", daemon=True),
                      threading.Thread(target=self._prediction, name="odom-20hz", daemon=True)]
        bus.abonner(S.IMU, self._imu)
        bus.abonner(S.COMMANDE, self._commande)
        bus.abonner(S.GNSS, self._gnss)
        bus.abonner(S.OBSTACLES_STEREO, self._obstacles)

    def demarrer(self) -> None:
        self._actif = True
        for fil in self._fils:
            fil.start()

    def arreter(self) -> None:
        self._actif = False

    def remettre_a_zero(self) -> None:
        with self._verrou:
            self.odometre.remettre_a_zero()
            self.cartographe.effacer()
            self.trajectoire.clear()
        self.bus.journaliser("estimation", "origine de l'odométrie remise sur le rover ; carte "
                             "effacée")

    def _imu(self, imu: Imu) -> None:
        self._entrees.append(("imu", imu))

    def _commande(self, commande: CommandeRoues) -> None:
        self._entrees.append(("commande", commande))

    def _gnss(self, gnss: Gnss) -> None:
        self._entrees.append(("gnss", gnss))

    def _vider_entrees(self) -> None:
        """Appelé verrou pris : transmet à l'odométrie ce qui est arrivé depuis le dernier
        calcul, dans l'ordre d'arrivée."""
        while self._entrees:
            genre, valeur = self._entrees.popleft()
            if genre == "imu":
                self.odometre.imu(valeur)
            elif genre == "commande":
                self.odometre.commande(valeur)
            else:
                self.odometre.gnss(valeur)

    def _obstacles(self, obstacles: Obstacles) -> None:
        odom: Odometrie | None = self.bus.dernier(S.ODOM)
        if odom is None:
            return
        with self._verrou:  # même vide : c'est ce qui efface un obstacle bas enlevé
            self.cartographe.integrer_obstacles(obstacles, odom.pose)

    def memoire_obstacles_bas(self) -> np.ndarray:
        """Obstacles bas mémorisés à moins de 3 m (base_link), y compris ceux sortis du champ de
        la caméra : la sécurité et le pilote local en tiennent compte en virage."""
        odom: Odometrie | None = self.bus.dernier(S.ODOM)
        if odom is None or self.bus.age(S.ODOM) > 0.5:
            return np.zeros((0, 2), np.float32)
        # Appelé par la sécurité à 20 Hz : jamais d'attente. Carte occupée (ICP, mise à jour) :
        # la dernière liste calculée, vieille de quelques dizaines de millisecondes au plus.
        if not self._verrou.acquire(blocking=False):
            return self._memoire_bas
        try:
            self._memoire_bas = self.cartographe.obstacles_proches(
                odom.pose, 3.0, bas_seulement=True).points
        finally:
            self._verrou.release()
        return self._memoire_bas

    def _boucle(self) -> None:
        import queue

        while self._actif:
            try:
                scan: Scan = self._scans.prendre(0.5)
            except queue.Empty:
                continue
            try:
                debut = time.perf_counter()
                with self._verrou:
                    self._vider_entrees()
                    odom = self.odometre.scan(scan)
                    if odom is not None:
                        self.cartographe.integrer_scan(scan, odom.pose)
                self.duree_icp_ms = 0.9 * self.duree_icp_ms + 100 * (time.perf_counter() - debut)
                if odom is None:
                    continue
                self.bus.publier(S.ODOM, odom)
                p = odom.pose
                if not self.trajectoire or math.hypot(
                        p.x - self.trajectoire[-1][0], p.y - self.trajectoire[-1][1]) > 0.1:
                    self.trajectoire.append((round(p.x, 3), round(p.y, 3)))
                if time.monotonic() - self._derniere_carte > 0.5:
                    self._derniere_carte = time.monotonic()
                    with self._verrou:
                        carte = self.cartographe.carte()
                    self.bus.publier(S.CARTE, carte)
            except Exception as erreur:
                self.bus.journaliser("estimation", f"erreur : {erreur!r}", "erreur")

    def _prediction(self) -> None:
        """Entre deux tours du lidar, pose prédite (gyroscope + commande) à 20 Hz."""
        while self._actif:
            time.sleep(0.05)
            if self.bus.age(S.ODOM) < 0.04 or self.bus.dernier(S.ODOM) is None:
                continue
            try:
                with self._verrou:
                    self._vider_entrees()
                    odom = self.odometre.courant(time.monotonic())
                self.bus.publier(S.ODOM, odom)
            except Exception as erreur:
                self.bus.journaliser("estimation", f"prédiction : {erreur!r}", "erreur")
                time.sleep(1.0)


class Perception:
    """Fil de la stéréo : dernière paire (images à moins de 60 ms d'écart), 4 Hz."""

    def __init__(self, bus: Bus, profil: Profil, simulee: bool = False) -> None:
        from fieldmind.perception.stereo import Stereo

        self.bus, self.profil = bus, profil
        self._averti = False
        self.simulee = simulee  # caméras virtuelles : exactement le modèle par défaut
        self.stereo = Stereo(profil)
        self._actif = False
        self.duree_ms = 0.0
        self.hz = 0.0
        self._fil = threading.Thread(target=self._boucle, name="stereo", daemon=True)

    def demarrer(self) -> None:
        self._actif = True
        self._fil.start()

    def arreter(self) -> None:
        self._actif = False

    def recharger(self) -> None:
        from fieldmind.perception.stereo import Stereo

        self.stereo = Stereo(self.profil)

    def resume(self) -> dict[str, Any]:
        age = self.bus.age(S.OBSTACLES_STEREO)
        return {"ok": age < 1.0, "hz": round(self.hz, 1), "age_s": round(min(age, 99), 2),
                "calibree": bool(getattr(self.stereo, "calibree", False)),
                "duree_ms": round(self.duree_ms, 1)}

    def _boucle(self) -> None:
        dernier_numero = (-1, -1)
        precedent = time.monotonic()
        while self._actif:
            periode = 1.0 / max(0.5, float(self.profil.valeur("perception.stereo_ips", 4)))
            debut = time.monotonic()
            if not bool(self.profil.valeur("perception.stereo_active", True)):
                time.sleep(0.5)
                continue
            g: ImageCamera | None = self.bus.dernier(S.IMAGE_GAUCHE)
            d: ImageCamera | None = self.bus.dernier(S.IMAGE_DROITE)
            if g is not None and d is not None and (g.numero, d.numero) != dernier_numero \
                    and abs(g.t - d.t) < 0.06 and debut - max(g.t, d.t) < 0.5:
                dernier_numero = (g.numero, d.numero)
                try:
                    t0 = time.perf_counter()
                    imu: Imu | None = self.bus.dernier(S.IMU)
                    roulis, tangage = imu.roulis_tangage() if imu is not None else (0.0, 0.0)
                    odom: Odometrie | None = self.bus.dernier(S.ODOM)
                    pose = odom.pose if odom is not None and self.bus.age(S.ODOM) < 0.5 else None
                    resultat = self.stereo.traiter(g, d, roulis, tangage, pose)
                    self.duree_ms = 0.8 * self.duree_ms + 200 * (time.perf_counter() - t0)
                    fiable = self.stereo.calibree or self.simulee or bool(
                        self.profil.valeur("perception.stereo_sans_calibration", False))
                    if resultat is not None and not fiable and not self._averti:
                        self._averti = True
                        self.bus.journaliser(
                            "stereo", "stéréo non calibrée : ses obstacles ne servent pas à la "
                            "sécurité (lidar seul, vitesse plafonnée) ; la calibrer avec le "
                            "damier (Système > Calibration > Stéréo)", "alerte")
                    if resultat is not None and fiable:
                        # Sans calibration, la profondeur est fausse et invente des obstacles
                        # bas : seulement l'image de profondeur, pas d'obstacles pour la sécurité
                        self.bus.publier(S.OBSTACLES_STEREO, resultat.obstacles)
                        if getattr(resultat, "profondeur_jpeg", None):
                            self.bus.publier(S.PROFONDEUR, resultat.profondeur_jpeg)
                        maintenant = time.monotonic()
                        self.hz = 0.8 * self.hz + 0.2 / max(maintenant - precedent, 1e-3)
                        precedent = maintenant
                except Exception as erreur:
                    self.bus.journaliser("stereo", f"erreur : {erreur!r}", "erreur")
                    time.sleep(2.0)
            time.sleep(max(0.01, periode - (time.monotonic() - debut)))


class Scene:
    """Fil de la scène 3D (message `scene`, 4 Hz) : lidar, stéréo, vision, carte."""

    def __init__(self, bus: Bus, profil: Profil, executeur: Executeur,
                 rover: Rover | None = None) -> None:
        from fieldmind.perception.scene import ReconstructeurScene

        self.bus, self.profil, self.executeur = bus, profil, executeur
        self.rover = rover
        mesurer_stereo = (rover.perception.stereo.mesurer_cluster_local
                          if rover is not None and rover.perception is not None else None)
        self.reconstructeur = ReconstructeurScene(profil, mesurer_stereo=mesurer_stereo)
        self._visions: list[dict[str, Any]] = []
        self._actif = False
        self._dernier_scan = -1.0
        self._dernier_stereo = -1.0
        self._dernier_auto = 0.0
        self._pose_auto: Pose2D | None = None
        self._origine_t = time.monotonic()
        self._fil = threading.Thread(target=self._boucle, name="scene", daemon=True)
        bus.abonner(S.VISION, lambda m: self._visions.append(m) if isinstance(m, dict) else None)
        self._detections: list[dict[str, Any]] = []
        bus.abonner(S.DETECTIONS,
                    lambda m: self._detections.append(m) if isinstance(m, dict) else None)

    def demarrer(self) -> None:
        self._actif = True
        self._fil.start()

    def arreter(self) -> None:
        self._actif = False

    def _boucle(self) -> None:
        while self._actif:
            debut = time.monotonic()
            # Vue 3D seulement : rien à reconstruire sans poste connecté, et 2 Hz suffisent
            # (c'était le plus gros utilisateur du processeur, devant l'odométrie)
            periode = 1.0 / max(0.5, float(self.profil.valeur("perception.scene_hz", 2.0)))
            if (self.rover is None or self.rover.superviseur.operateurs > 0
                    or self.executeur.occupe()):
                try:
                    self._cycle(debut)
                except Exception as erreur:
                    self.bus.journaliser("scene", f"erreur : {erreur!r}", "erreur")
                    time.sleep(1.0)
            time.sleep(max(0.02, periode - (time.monotonic() - debut)))

    def _cycle(self, t: float) -> None:
        bus = self.bus
        odom: Odometrie | None = bus.dernier(S.ODOM)
        pose = odom.pose if odom is not None and bus.age(S.ODOM) < 1.0 else None
        r = self.reconstructeur
        if pose is None:
            r.objets.clear()  # sans odométrie, pas de mémoire : seulement le tour courant
        scan: Scan | None = bus.dernier(S.SCAN)
        if scan is not None and scan.t != self._dernier_scan and bus.age(S.SCAN) < 0.5:
            self._dernier_scan = scan.t
            r.integrer_scan(scan, pose or Pose2D(), t)
        stereo: Obstacles | None = bus.dernier(S.OBSTACLES_STEREO)
        if stereo is not None and stereo.t != self._dernier_stereo and \
                bus.age(S.OBSTACLES_STEREO) < 1.0:
            self._dernier_stereo = stereo.t
            r.integrer_stereo(stereo, pose or Pose2D(), t)
        self._fusionner_camera_auto(pose or Pose2D(), t)
        while self._visions:
            vision = self._visions.pop(0)
            if vision.get("image_t", t) < self._origine_t:
                continue
            acquisition = vision.get("pose_image")
            r.integrer_vision(vision, Pose2D(**acquisition) if acquisition is not None
                             else pose or Pose2D(), t)
        while self._detections:
            r.integrer_detections(self._detections.pop(0), pose or Pose2D(), t)
        chemin, but = self.executeur.chemin_courant()
        message = r.message(pose, bus.dernier(S.CARTE), bus.dernier(S.SECURITE), chemin, but, t)
        if self.rover is not None:
            memoire = self.rover.monde_semantique.resume()
            message["memoire"] = memoire
            for o in memoire["objets"]:
                if o.get("observations_visuelles", 1) < 1:
                    continue
                if not any(obj.get("texte") == o["label"] and math.hypot(
                        obj["x"] - o["x"], obj["y"] - o["y"]) < 0.8 for obj in message["objets"]):
                    message["objets"].append({"id": 500000 + o["id"], "type": "etiquette",
                                              "x": o["x"], "y": o["y"], "texte": o["label"],
                                              "age_s": o["age_s"], "memorise": True})
        bus.publier(S.SCENE, message)

    def _fusionner_camera_auto(self, pose: Pose2D, t: float) -> None:
        """En tâche de fond, fusionne les objets vus par la caméra avec le lidar et la profondeur
        stéréo dès que le rover bouge ou toutes les 2,5 s."""
        if self.rover is None or t - self._dernier_auto < 1.2:
            return
        if self.bus.age(S.IMAGE_GAUCHE) > 1.5:
            return
        deplace = (self._pose_auto is None
                   or math.hypot(pose.x - self._pose_auto.x, pose.y - self._pose_auto.y) >= 0.20
                   or abs(math.atan2(math.sin(pose.cap - self._pose_auto.cap),
                                     math.cos(pose.cap - self._pose_auto.cap))) >= 0.15)
        if not deplace and t - self._dernier_auto < 2.5:
            return
        self._dernier_auto = t
        self._pose_auto = pose
        points = self.rover.detecter_camera_fusionnee("gauche")
        if points:
            self.reconstructeur.integrer_vision({"points": points}, pose, t)


class OutilsRover:
    """Implémentation de fieldmind.ia.outils.OutilsRover : ce que l'IA peut voir et demander."""

    def __init__(self, rover: Rover) -> None:
        self.rover = rover

    def etat_resume(self) -> dict[str, Any]:
        r = self.rover
        odom: Odometrie | None = r.bus.dernier(S.ODOM)
        securite = r.conduite.securite
        capteurs = r.materiel.capteurs()
        return {
            "pose_odom": None if odom is None else {
                "x_m": round(odom.pose.x, 2), "y_m": round(odom.pose.y, 2),
                "cap_deg": round(math.degrees(odom.pose.cap), 1),
                "vitesse_m_s": round(odom.vitesse, 2)},
            "mode": r.conduite.mode, "arme": r.conduite.arme,
            "arret_urgence": r.superviseur.arret_urgence,
            "securite": {"raisons": securite.raisons,
                         "libre_avant_m": _arrondi(securite.libre_avant_m),
                         "libre_arriere_m": _arrondi(securite.libre_arriere_m)},
            "mission": r.executeur.resume(),
            "capteurs": {nom: bool(c.get("ok")) for nom, c in capteurs.items()},
            "stereo": r.perception.resume() if r.perception else {"ok": False},
            "lieux": [{"nom": lieu["nom"], "gps": lieu.get("lat") is not None}
                      for lieu in r.gps.lieux],
            "gps": {"aligne": r.gps.alignement.ok, "fix": r.gps.dernier_fix.fix
                    if r.gps.dernier_fix else "aucun"},
            "situation": _situation(r.bus),
            "monde": r.monde_semantique.resume(),
            "cible_suivie": r.suivi.resume() if getattr(r, "suivi", None) is not None else None,
        }

    def ajouter_actions(self, actions: list[Action], remplacer: bool) -> str:
        return self.rover.executeur.ajouter(actions, remplacer=remplacer, origine="ia")

    def memoriser_pieces(self, propositions: list[dict[str, Any]], *,
                        generation: int | None = None) -> list[dict[str, Any]]:
        return self.rover.monde_semantique.integrer_pieces(propositions, generation=generation)

    def annuler(self) -> None:
        self.rover.executeur.annuler("annulé par l'IA")

    def arret_immediat(self, raison: str) -> None:
        r = self.rover
        r.executeur.annuler(raison)
        r.conduite.proposer_arret()
        r.bus.journaliser("ia", f"arrêt demandé : {raison}", "alerte")

    def derniere_image(self, camera: str) -> ImageCamera | None:
        sujet = {"gauche": S.IMAGE_GAUCHE, "droite": S.IMAGE_DROITE,
                 "drone": S.IMAGE_DRONE}.get(camera)
        if sujet is None or self.rover.bus.age(sujet) > 1.5:
            return None
        image = self.rover.bus.dernier(sujet)
        return image if image is not None and abs(time.monotonic() - image.t) < 1.5 else None

    def snapshot_image(self, camera: str = "gauche") -> ObservationCamera | None:
        image = self.derniere_image(camera)
        return None if image is None else ObservationCamera(self.rover, image)

    def pixel_vers_sol(self, camera: str, u: float, v: float) -> tuple[float, float] | None:
        return self.rover.pixel_vers_sol(camera, u, v)

    def mesurer_objet(self, camera: str, u: float, v: float,
                      boite: tuple[float, float, float, float] | None = None,
                      categorie: str | None = None) -> dict[str, Any] | None:
        return self.rover.mesurer_objet(camera, u, v, boite, categorie)

    def detecter_camera(self, camera: str = "gauche") -> list[dict[str, Any]]:
        return self.rover.detecter_camera_fusionnee(camera)

    def pose_odom(self) -> Pose2D:
        odom: Odometrie | None = self.rover.bus.dernier(S.ODOM)
        return odom.pose if odom is not None else Pose2D()


def _arrondi(valeur: float | None) -> float | None:
    return None if valeur is None else round(valeur, 2)


class _BusObservation(Bus):
    def age(self, sujet: str) -> float:
        # La fraîcheur est vérifiée lors de la capture ; le temps réseau ne vieillit pas la scène.
        return 0.0 if self.dernier(sujet) is not None else math.inf


class ObservationCamera:
    """Image et géométrie figées avant le réseau, utilisables pendant que le rover avance.

    La copie n'expose que les méthodes de mesure ; aucun matériel n'est commandé. Les tableaux
    de profondeur sont copiés sous leur verrou, les scans sont immuables après publication.
    """

    def __init__(self, rover: Rover, image: ImageCamera) -> None:
        self.image, self.image_t = image, image.t
        historique = tuple(rover._poses_camera)
        if not historique:
            odom = rover.bus.dernier(S.ODOM)
            historique = (odom,) if odom is not None else ()
        historique = tuple(o for o in historique if o.t >= rover._origine_odom_t
                           and all(math.isfinite(v) for v in (o.pose.x, o.pose.y, o.pose.cap)))
        proche = min(historique, key=lambda o: abs(o.t - image.t)) if historique else None
        self.pose = (copy.copy(proche.pose) if proche is not None and abs(proche.t - image.t) < 0.35
                     else None)
        mesure = self._mesure = copy.copy(rover)
        mesure.bus = _BusObservation()
        mesure.profil = Profil(rover.profil.dict(), {}, None)
        mesure._projections = dict(rover._projections)
        for sujet in (S.SCAN, S.IMU):
            valeur = rover.bus.dernier(sujet)
            if valeur is not None and abs(valeur.t - image.t) < 0.4:
                mesure.bus.publier(sujet, valeur)
        mesure.perception = None
        obstacles = rover.bus.dernier(S.OBSTACLES_STEREO)
        if (rover.perception is not None and obstacles is not None
                and abs(obstacles.t - image.t) < 0.5):
            stereo = rover.perception.stereo
            with stereo._verrou:
                copie = copy.copy(stereo)
                copie._verrou = threading.Lock()
                if stereo._derniere is not None:
                    geo, profondeur = stereo._derniere
                    copie._derniere = (geo, profondeur.copy())
                copie._sol_lisse = (None if stereo._sol_lisse is None
                                    else np.array(stereo._sol_lisse, copy=True))
            mesure.perception = SimpleNamespace(stereo=copie)

    def pixel_vers_sol(self, camera: str, u: float, v: float) -> tuple[float, float] | None:
        return self._mesure.pixel_vers_sol(camera, u, v)

    def mesurer_objet(self, camera: str, u: float, v: float,
                      boite: tuple[float, float, float, float] | None = None,
                      categorie: str | None = None) -> dict[str, Any] | None:
        return self._mesure.mesurer_objet(camera, u, v, boite, categorie)


class Rover:
    def __init__(self, profil: Profil, simulation: bool = False, **options: Any) -> None:
        import sys

        # Python change de fil toutes les 2 ms (5 par défaut) : la boucle de conduite à 20 Hz
        # attend moins son tour derrière la perception
        sys.setswitchinterval(0.002)
        self.profil = profil
        options = dict(options)
        enregistrer = options.pop("enregistrer", not simulation)
        dossier_utilisateur = options.pop("dossier_utilisateur", DOSSIER_UTILISATEUR)
        self.bus = Bus()
        self.debut = time.monotonic()
        if simulation:
            from fieldmind.simulation.materiel_simule import MaterielSimule

            self.materiel: Any = MaterielSimule(self.bus, profil, **options)
        else:
            from fieldmind.materiel.detection import MaterielReel

            self.materiel = MaterielReel(self.bus, profil)
        self.superviseur = Superviseur(self.bus, profil)
        from fieldmind.controle.remorque import SuiviRemorque

        self.remorque = SuiviRemorque(self.bus, profil)
        self.superviseur.remorque = self.remorque
        self.conduite = Conduite(self.bus, profil, self.materiel.esp, self.superviseur)
        self.estimation = self._facultatif("estimation", lambda: Estimation(self.bus, profil))
        self.perception = self._facultatif("stéréo",
                                           lambda: Perception(self.bus, profil, simulation))
        self._projections: dict[str, Any] = {}
        navigation = self._facultatif("navigation", lambda: Navigation(profil))
        self.executeur = Executeur(self.bus, profil, self.conduite, navigation)
        self._poses_camera: deque[Odometrie] = deque(maxlen=128)
        self._origine_odom_t = float("-inf")
        self.bus.abonner(S.ODOM, self._poses_camera.append)
        from fieldmind.estimation.semantique import MemoireSemantique

        self.monde_semantique = MemoireSemantique(self.bus)
        self.executeur.monde = self.monde_semantique
        self.executeur.pixel_vers_sol = self.pixel_vers_sol
        self.suivi = self._facultatif("suivi de personne", lambda: self._suivi(profil))
        if self.suivi is not None:
            self.executeur.suivi = self.suivi
            self.superviseur.cible_suivie = self.suivi.pour_securite
        if self.estimation is not None:
            self.superviseur.memoire_obstacles = self.estimation.memoire_obstacles_bas
            self.executeur.memoire_obstacles = self.estimation.memoire_obstacles_bas
        self.outils = OutilsRover(self)
        from fieldmind.estimation.gps import NavigationGps

        self.gps = NavigationGps(profil, dossier=dossier_utilisateur)
        self.bus.abonner(S.GNSS, lambda fix: self.gps.mesure(fix, self.bus.dernier(S.ODOM)))
        self.superviseur.zone = self.gps.hors_zone
        self.executeur.gps = self.gps
        profil.sur_changement(self._reglage_change)
        self.scene = self._facultatif(
            "scène 3D", lambda: Scene(self.bus, profil, self.executeur, self))
        self.enregistreur: Any = None
        if enregistrer and bool(
                profil.valeur("enregistrement.actif", True)):
            from fieldmind.enregistreur import Enregistreur

            self.enregistreur = Enregistreur(self.bus)
        self.ia: Any = None  # fieldmind.ia, branché par demarrer_ia
        self.calibration_stereo: Any = None

    def _suivi(self, profil: Profil) -> Any:
        from fieldmind.perception.suivi import SuiviCible

        return SuiviCible(self.bus, profil)

    def _facultatif(self, nom: str, fabrique: Any) -> Any:
        try:
            return fabrique()
        except Exception as erreur:  # module absent ou en erreur : le reste du rover continue
            self.bus.journaliser("rover", f"{nom} indisponible : {erreur!r}", "erreur")
            return None

    # ------------------------------------------------------------------ cycle de vie

    def demarrer(self) -> None:
        if self.enregistreur is not None:
            self._facultatif("boîte noire", self.enregistreur.demarrer)
        self.materiel.demarrer()
        self.conduite.demarrer()
        for module in (self.estimation, self.perception, self.scene):
            if module is not None:
                module.demarrer()
        if self.suivi is not None:
            self.suivi.demarrer_fil()
        self.executeur.demarrer()
        self.bus.journaliser("rover", f"démarré ({self.materiel.nom}) : mode arrêt, désarmé",
                             "succes")

    async def demarrer_ia(self, boucle: asyncio.AbstractEventLoop, config: Any = None) -> None:
        try:
            from fieldmind.ia.integration import Ia

            self.ia = Ia(self.bus, self.profil, self.outils, config)
            await self.ia.demarrer()
        except Exception as erreur:
            self.ia = None
            self.bus.journaliser("ia", f"IA indisponible : {erreur!r}", "erreur")
            return
        self.executeur.boucle_asyncio = boucle
        self.executeur.ia = self.ia.pour_mission

    def arreter(self) -> None:
        self.executeur.arreter()
        self.conduite.arreter()
        for module in (self.estimation, self.perception, self.scene):
            if module is not None:
                module.arreter()
        self.materiel.arreter()
        if self.enregistreur is not None:
            self.enregistreur.arreter()

    # ------------------------------------------------------------------ services

    def pixel_vers_sol(self, camera: str, u: float, v: float) -> tuple[float, float] | None:
        """Point du sol visé sur l'image (base_link) : profondeur stéréo si elle existe, sinon
        projection sur le sol plan."""
        if self.perception is not None:
            try:
                point = self.perception.stereo.point_3d(camera, u, v)
                if point is not None and 0.2 < math.hypot(point[0], point[1]) < 12:
                    return float(point[0]), float(point[1])
            except Exception:
                pass
        modele = self._projections.get(camera)
        if modele is None:
            from fieldmind.perception.projection import ModeleCamera

            modele = self._projections[camera] = ModeleCamera(self.profil, camera)
        imu: Imu | None = self.bus.dernier(S.IMU)
        roulis, tangage = imu.roulis_tangage() if imu is not None else (0.0, 0.0)
        point = modele.pixel_vers_sol(u, v, roulis, tangage)
        if point is None or math.hypot(point[0], point[1]) > 12:
            return None
        return float(point[0]), float(point[1])

    def mesurer_objet(self, camera: str, u: float, v: float,
                      boite: tuple[float, float, float, float] | None = None,
                      categorie: str | None = None) -> dict[str, Any] | None:
        """Fusionne caméra (u, v, boîte 2D), profondeur stéréo et scan lidar pour estimer la
        position (x, y), les dimensions 3D et la catégorie géométrique d'un objet."""
        from fieldmind.perception.scene import ajuster_droite
        from fieldmind.types import normaliser_angle

        mesure_stereo: dict[str, float] | None = None
        if self.perception is not None:
            try:
                mesure_stereo = self.perception.stereo.mesurer_zone(camera, u, v, boite)
            except Exception:
                mesure_stereo = None
        sol = self.pixel_vers_sol(camera, u, v)
        if mesure_stereo is not None:
            x0, y0 = mesure_stereo["x"], mesure_stereo["y"]
            hauteur = mesure_stereo.get("hauteur", 0.5)
            largeur = mesure_stereo.get("largeur", 0.35)
            longueur = mesure_stereo.get("longueur", 0.35)
            source = "stereo"
        elif sol is not None:
            x0, y0 = sol
            hauteur, largeur, longueur = 0.5, 0.35, 0.35
            source = "sol"
        else:
            return None

        dist0 = math.hypot(x0, y0)
        azimut0 = math.atan2(y0, x0)
        hfov = math.radians(float(self.profil.valeur("cameras.hfov_deg", 78.0)))
        demi_cone = max(0.08, 0.55 * abs(boite[2] - boite[0]) * hfov) if boite else 0.12

        # Hauteur géométrique déduite de la boîte 2D si présente
        if boite is not None:
            vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * (9.0 / 16.0))
            h_boite = dist0 * abs(boite[3] - boite[1]) * vfov
            w_boite = dist0 * abs(boite[2] - boite[0]) * hfov
            if source == "sol" or hauteur < 0.12:
                hauteur = float(np.clip(h_boite, 0.08, 2.5))
            else:
                hauteur = float(np.clip(0.6 * hauteur + 0.4 * h_boite, 0.08, 2.5))
            largeur = max(largeur, float(np.clip(w_boite, 0.08, 2.5)))

        cap = azimut0
        lidar_trouve = False
        scan: Scan | None = self.bus.dernier(S.SCAN)
        if scan is not None and self.bus.age(S.SCAN) < 0.8 and len(scan.distances):
            pts = scan.points()
            ecarts_ang = np.abs(np.arctan2(np.sin(scan.angles - azimut0),
                                           np.cos(scan.angles - azimut0)))
            marge_d = max(0.85, 0.35 * dist0)
            dans_cone = (ecarts_ang <= demi_cone + 0.06) & (scan.distances < 12.0) & (
                np.abs(scan.distances - dist0) <= marge_d)
            if int(np.count_nonzero(dans_cone)) >= 3:
                pts_cone = pts[dans_cone]
                d_cone = scan.distances[dans_cone]
                d_avant = float(np.percentile(d_cone, 25))
                proches = pts_cone[np.abs(d_cone - d_avant) < 0.35]
                if len(proches) >= 3:
                    centre, direction, long_l, _ = ajuster_droite(proches)
                    normale = np.array([-direction[1], direction[0]])
                    larg_l = max(0.12, float(np.ptp((proches - centre) @ normale)))
                    portee = max(math.hypot(*centre), 1e-6)
                    decal = min(0.25, 0.35 * max(larg_l, min(long_l, 0.5)))
                    cx, cy = centre + centre / portee * decal
                    x0, y0 = float(cx), float(cy)
                    cap = normaliser_angle(math.atan2(direction[1], direction[0]))
                    longueur = float(np.clip(max(long_l, 0.12), 0.10, 4.0))
                    largeur = float(np.clip(max(larg_l, largeur * 0.7), 0.10, 2.0))
                    lidar_trouve = True
                    source = "lidar+stereo" if source == "stereo" else "lidar"

        z_lidar = float(self.profil.valeur("lidar.z_m", 0.322))
        cat = (categorie or "").lower().strip()
        if cat in ("bas", "obstacle_bas") or (not lidar_trouve and hauteur < z_lidar):
            type_3d = "bas"
            hauteur = min(hauteur, z_lidar - 0.02)
        elif cat in ("cylindre", "poteau", "pilier", "arbre") or (
                max(longueur, largeur) < 0.30 and cat != "bloc"):
            type_3d = "arbre"
        elif cat == "mur" and longueur >= 0.8:
            type_3d = "mur"
        else:
            type_3d = "bloc"
            if lidar_trouve and hauteur < z_lidar:
                hauteur = max(hauteur, 0.55)

        return {
            "sol": [round(x0, 2), round(y0, 2)],
            "hauteur": round(float(hauteur), 2),
            "largeur": round(float(largeur), 2),
            "longueur": round(float(longueur), 2),
            "cap": round(float(cap), 3),
            "type_3d": type_3d,
            "source": source,
        }

    def detecter_camera_fusionnee(self, camera: str = "gauche") -> list[dict[str, Any]]:
        """Détecte les objets visibles dans la caméra et les fusionne avec le lidar et la
        profondeur stéréo pour produire leurs coordonnées et dimensions 3D."""
        monde = getattr(self.materiel, "monde", None)
        physique = getattr(self.materiel, "physique", None)
        if monde is None or physique is None or not hasattr(monde, "detecter_camera"):
            return []
        try:
            detections = monde.detecter_camera(physique.pose, camera, self.profil)
        except Exception:
            return []
        points: list[dict[str, Any]] = []
        for det in detections:
            u, v = float(det["u"]), float(det["v"])
            boite = det.get("boite")
            cat = det.get("categorie")
            mesure = self.mesurer_objet(camera, u, v, boite, cat)
            if mesure is None:
                continue
            entree: dict[str, Any] = {
                "u": u,
                "v": v,
                "label": det["label"],
                "sol": mesure.get("sol"),
            }
            if boite is not None:
                entree["boite"] = [round(b, 4) for b in boite]
            if cat is not None:
                entree["categorie"] = cat
            for cle in ("hauteur", "largeur", "longueur", "cap", "type_3d", "source"):
                if cle in mesure:
                    entree[cle] = mesure[cle]
            points.append(entree)
        return points

    def capteurs(self) -> dict[str, Any]:
        c = self.materiel.capteurs()
        c["stereo"] = self.perception.resume() if self.perception else {"ok": False}
        if self.estimation is not None:
            c["odometrie"] = {"ok": self.bus.age(S.ODOM) < 0.5,
                              "duree_ms": round(self.estimation.duree_icp_ms, 1)}
        drone = self.bus.dernier(S.DRONE)
        age_drone = self.bus.age(S.DRONE)
        c["drone"] = {
            "ok": bool(drone and drone.connecte and age_drone < 5.0),
            "marche": None if drone is None else drone.etat != "inactif",
            "ips": round(drone.ips, 1) if drone else 0.0,
            "resolution": [drone.largeur, drone.hauteur] if drone else [640, 360],
            "age_s": None if drone is None else round(min(age_drone, 99), 2),
        }
        return c

    def remise_a_zero_odom(self) -> None:
        self._origine_odom_t = time.monotonic()
        self.executeur.annuler("remise à zéro de l'odométrie")
        self.executeur._derniere_exploration = None
        self.monde_semantique.effacer()
        self._poses_camera.clear()
        if self.scene is not None:
            self.scene.reconstructeur.objets.clear()
            self.scene._origine_t = time.monotonic()
            self.scene._visions.clear()
        if self.estimation is not None:
            self.estimation.remettre_a_zero()
        self.gps.oublier_odometrie()

    def reinitialiser_environnement(self) -> None:
        """Repart d'un environnement vierge : missions annulées, position remise à l'origine,
        carte, objets vus, scène 3D et conversation de l'agent effacés. Réglages, calibrations
        et lieux nommés sont gardés."""
        self.remise_a_zero_odom()
        agent = getattr(self.ia, "agent", None) if self.ia is not None else None
        if agent is not None:
            agent.effacer_historique()
        self.bus.journaliser("environnement", "environnement réinitialisé : carte, objets, "
                             "position et missions effacés", "succes")

    def _reglage_change(self, cle: str, _valeur: Any) -> None:
        """Applique tout de suite un réglage qui touche le matériel ou la perception."""
        if cle in ("direction.servo_vitesse_deg_s", "esp.chien_de_garde_ms", "roues.capteurs",
                   "traction.delai_inversion_s"):
            esp = self.materiel.esp()
            if esp is not None:
                esp.reconfigurer()
        if cle == "capteurs.lidar":
            self.materiel.lidar_marche(bool(_valeur))
            etat = "remis en marche" if _valeur else "coupé : le rover ne roulera pas"
            self.bus.journaliser("capteurs", f"lidar {etat}", "alerte")
        elif cle.startswith("capteurs."):
            etat = "actif" if _valeur else "coupé"
            self.bus.journaliser("capteurs", f"{cle.split('.')[1]} {etat}")
        if cle == "remorque.volets_deplies":
            esp = self.materiel.esp()
            if esp is not None and hasattr(esp, "mouvoir_volets"):
                deplies = bool(_valeur)

                def mouvoir() -> None:
                    if esp.mouvoir_volets(deplies):
                        etat = "dépliés" if deplies else "repliés"
                        self.bus.journaliser("remorque", f"volets {etat}")

                threading.Thread(target=mouvoir, name="volets", daemon=True).start()
        if cle.startswith("cameras.") or cle.startswith("perception."):
            self._projections.clear()
            if self.perception is not None and cle in ("cameras.inclinaison_deg",
                                                       "cameras.hfov_deg"):
                self.perception.recharger()

    def points_obstacles(self) -> np.ndarray:
        stereo: Obstacles | None = self.bus.dernier(S.OBSTACLES_STEREO)
        if stereo is None or self.bus.age(S.OBSTACLES_STEREO) > 1.0:
            return np.zeros((0, 2), np.float32)
        return stereo.points

    def refuser_si_roule(self) -> None:
        """Lève ActionRefusee si le rover roule ou peut rouler (console, lidar, réglages)."""
        if self.conduite.arme and self.conduite.mode != "banc":
            raise ActionRefusee("désarmer le rover d'abord")
        odom: Odometrie | None = self.bus.dernier(S.ODOM)
        if odom is not None and abs(odom.vitesse) > 0.05:
            raise ActionRefusee("le rover roule")


def _situation(bus: Bus) -> dict[str, Any] | None:
    """Dernier résumé de l'agent de veille et dernières détections, pour l'agent de
    conversation (« qu'est-ce qui se passe ? »)."""
    situation = bus.dernier(S.SITUATION)
    detections = bus.dernier(S.DETECTIONS)
    sortie: dict[str, Any] = {}
    if isinstance(situation, dict) and bus.age(S.SITUATION) < 120:
        sortie.update({k: situation.get(k) for k in ("resume", "niveau", "alerte")})
        sortie["age_s"] = round(bus.age(S.SITUATION), 1)
    if isinstance(detections, dict) and bus.age(S.DETECTIONS) < 10:
        sortie["vus"] = [{"label": o.get("label"), "distance_m": o.get("distance_m")}
                         for o in detections.get("objets") or [] if o.get("score", 0) > 0.5][:8]
    return sortie or None
