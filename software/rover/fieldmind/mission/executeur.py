"""Exécuteur de mission : file d'actions jouées en mode auto, à 10 Hz, dans son propre fil.

Les actions viennent du poste (boutons, clic sur l'image ou la carte), de l'agent IA (consignes
écrites ou vocales) ou de ROS 2 (but). Elles sont validées avant d'entrer dans la file
(`actions.valider`). Chaque pas produit une consigne `auto` pour la conduite, qui la fait passer
par la sécurité. Quitter le mode auto, désarmer ou déclencher l'arrêt d'urgence annule la file.
"""

from __future__ import annotations

import asyncio
import itertools
import math
import threading
import time
from collections import deque
from collections.abc import Callable, Coroutine
from concurrent.futures import Future
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from fieldmind.bus import Bus, S
from fieldmind.config import Profil, braquage_max
from fieldmind.mission import actions as A
from fieldmind.mission.actions import ActionRefusee, Contexte, Pas
from fieldmind.types import (
    Action,
    CarteOccupation,
    Chemin,
    Consigne,
    ImageCamera,
    Obstacles,
    Odometrie,
    Pose2D,
    Scan,
)

PERIODE_S = 0.1
SANS_MOUVEMENT = {"memoriser_lieu", "photo"}
DOSSIER_PHOTOS = Path("~/fieldmind/photos").expanduser()


class Navigation:
    """Planificateur (A*) et pilote local (arcs Ackermann) de fieldmind.navigation."""

    def __init__(self, profil: Profil) -> None:
        from fieldmind.navigation.local import PiloteLocal
        from fieldmind.navigation.planificateur import Planificateur

        self._planificateur = Planificateur(profil)
        self._pilote = PiloteLocal(profil)

    def planifier(self, carte: CarteOccupation | None, depart: Pose2D,
                  but: tuple[float, float]) -> Chemin | None:
        if carte is None:  # pas encore de carte : ligne droite, le pilote local évitera
            points = np.array([[depart.x, depart.y], [but[0], but[1]]], np.float32)
            return Chemin(t=time.monotonic(), points=points, but=but,
                          longueur_m=float(math.hypot(but[0] - depart.x, but[1] - depart.y)))
        return self._planificateur.planifier(carte, depart, but)

    def commande(self, odom: Odometrie, chemin: Chemin, obstacles: np.ndarray,
                 vitesse_max: float) -> Any:
        return self._pilote.commande(odom, chemin, obstacles, vitesse_max)


@dataclass
class Chercher:
    """Vision IA : pointer l'objet ; s'il n'est pas vu, tourner de 60° et recommencer (6 fois)."""

    action: Action
    executeur: Executeur
    essais: int = 0
    _analyse: Future[Any] | None = None
    _pose_image: Pose2D | None = None
    _sous: A.Primitive | None = None
    _mode: str = "analyse"  # analyse | approche | rotation
    trouve: dict[str, Any] | None = None

    def pas(self, ctx: Contexte) -> Pas:
        objet = self.action["objet"]
        if self._sous is not None:
            resultat = self._sous.pas(ctx)
            if resultat.etat == "termine":
                self._sous = None
                if self._mode == "approche":
                    return Pas(etat="termine", progression=1.0, detail=f"« {objet} » atteint")
                self._mode = "analyse"
            elif resultat.etat == "echec":
                return resultat
            else:
                resultat.detail = f"{objet} : {resultat.detail}"
                return resultat
        if self._analyse is None:
            if self.essais >= 6:
                return Pas(etat="echec", raison=f"« {objet} » introuvable autour du rover")
            self.essais += 1
            self._pose_image = ctx.odom.pose
            self._analyse = self.executeur.lancer_ia(
                "chercher", (objet, self.action.get("camera", "gauche")))
            if self._analyse is None:
                return Pas(etat="echec", raison="IA indisponible")
            return Pas(detail=f"analyse de l'image ({self.essais}/6)")
        if not self._analyse.done():
            return Pas(detail=f"analyse de l'image ({self.essais}/6)")
        try:
            points = self._analyse.result() or []
        except Exception as erreur:
            return Pas(etat="echec", raison=f"vision : {erreur}")
        finally:
            self._analyse = None
        au_sol = [p for p in points if p.get("sol")]
        if au_sol:
            self.trouve = au_sol[0]
            if not self.action.get("approcher", True):
                return Pas(etat="termine", progression=1.0, detail=f"« {objet} » repéré")
            x, y = au_sol[0]["sol"]
            distance = math.hypot(x, y)
            arret = max(0.0, distance - 0.5)  # s'arrêter à 50 cm de l'objet
            if arret < A.TOLERANCE_BUT_M:
                return Pas(etat="termine", progression=1.0, detail=f"« {objet} » devant le rover")
            assert self._pose_image is not None
            cible = self._pose_image.vers_parent(x * arret / distance, y * arret / distance)
            self._mode = "approche"
            self._sous = A.AllerA({"type": "aller_a", "x": cible[0], "y": cible[1],
                                   "repere": "odom"})
            return Pas(detail=f"« {objet} » à {distance:.1f} m".replace(".", ","))
        if points:  # vu mais pas au sol (trop loin, au-dessus de l'horizon) : se tourner vers lui
            u = float(points[0].get("u", 0.5))
            angle = -(u - 0.5) * float(self.executeur.profil.valeur("cameras.hfov_deg", 78.0))
            if abs(angle) > 8:
                self._mode = "rotation"
                self._sous = A.Tourner({"type": "tourner", "angle_deg": angle})
                return Pas(detail=f"« {objet} » au loin : orientation")
            self._mode = "approche"
            self._sous = A.Avancer({"type": "avancer", "distance_m": 1.5})
            return Pas(detail=f"« {objet} » au loin : approche")
        self._mode = "rotation"
        self._sous = A.Tourner({"type": "tourner", "angle_deg": 60})
        return Pas(detail=f"« {objet} » pas en vue : je tourne")


@dataclass
class AllerGps:
    """But GPS : converti dans odom par le recalage odométrie ↔ GPS, puis atteint par la
    navigation locale (sous-buts à 12 m au plus, recalculés quand le recalage s'affine). Tant que
    l'odométrie n'est pas orientée sur le nord, le rover roule d'abord quelques mètres droit."""

    action: Action
    executeur: Executeur
    _sous: Any = None
    _phase: str = ""
    _essais: int = 0
    _sous_but: tuple[float, float] | None = None
    _maj: float = -1e9
    _distance_initiale: float | None = None

    def pas(self, ctx: Contexte) -> Pas:
        gps = self.executeur.gps
        if gps is None:
            return Pas(etat="echec", raison="GPS indisponible")
        tolerance = float(self.action.get("tolerance_m", self.executeur.profil.valeur(
            "gnss.tolerance_m", 2.5)))
        cible = gps.latlon_vers_odom(self.action["lat"], self.action["lon"])
        pose = ctx.odom.pose
        if cible is None:
            return self._aligner(ctx)
        if self._phase == "alignement":
            self._sous, self._phase = None, ""
        distance = math.hypot(cible[0] - pose.x, cible[1] - pose.y)
        if distance > float(self.executeur.profil.valeur("gnss.distance_max_m", 2000.0)):
            return Pas(etat="echec", raison=f"but GPS à {distance:.0f} m : trop loin")
        if self._distance_initiale is None:
            self._distance_initiale = max(distance, 1e-3)
        progression = max(0.0, min(1.0, 1 - distance / self._distance_initiale))
        detail = f"{distance:.1f} m du but GPS".replace(".", ",")
        if distance <= tolerance:
            return Pas(etat="termine", progression=1.0, detail="but GPS atteint")
        portee = 12.0
        if distance > portee:
            sous_but = (pose.x + (cible[0] - pose.x) * portee / distance,
                        pose.y + (cible[1] - pose.y) * portee / distance)
        else:
            sous_but = cible
        deplace = self._sous_but is None or math.hypot(
            sous_but[0] - self._sous_but[0], sous_but[1] - self._sous_but[1]) > 0.5
        if self._sous is None or (deplace and ctx.t - self._maj > 3.0):
            self._sous = A.AllerA({"type": "aller_a", "x": sous_but[0], "y": sous_but[1],
                                   "repere": "odom"})
            self._sous_but, self._maj, self._phase = sous_but, ctx.t, "route"
        resultat = self._sous.pas(ctx)
        if resultat.etat == "termine":
            self._sous = None  # sous-but atteint : le suivant au prochain pas
            return Pas(detail=detail, progression=progression)
        if resultat.etat == "echec":
            return resultat
        resultat.detail, resultat.progression = detail, progression
        return resultat

    def _aligner(self, ctx: Contexte) -> Pas:
        """Pas encore orienté sur le nord : avancer, tourner, avancer (6 m à chaque fois)."""
        if self._phase != "alignement" or self._sous is None:
            if self._essais >= 3:
                return Pas(etat="echec", raison="GPS : orientation impossible (fix ? trajet ?)")
            self._essais += 1
            self._phase = "alignement"
            self._sous = (A.Tourner({"type": "tourner", "angle_deg": 90}) if self._essais == 2
                          else A.Avancer({"type": "avancer", "distance_m": 6.0}))
        resultat = self._sous.pas(ctx)
        if resultat.etat in ("termine", "echec"):
            self._sous = None
            return Pas(detail="orientation sur le GPS")
        resultat.detail = "orientation sur le GPS (quelques mètres droit)"
        return resultat


@dataclass
class AttenteIa:
    """Action sans mouvement qui attend le résultat de l'IA (décrire la scène)."""

    action: Action
    executeur: Executeur
    _analyse: Future[Any] | None = None

    def pas(self, ctx: Contexte) -> Pas:
        if self._analyse is None:
            self._analyse = self.executeur.lancer_ia("decrire", self.action.get("camera"))
            if self._analyse is None:
                return Pas(etat="echec", raison="IA indisponible")
        if not self._analyse.done():
            return Pas(detail="analyse de l'image")
        try:
            self._analyse.result()
        except Exception as erreur:
            return Pas(etat="echec", raison=f"vision : {erreur}")
        return Pas(etat="termine", progression=1.0)


@dataclass
class _Courante:
    plan_id: str
    action: Action
    primitive: Any
    debut: float
    dernier: Pas = field(default_factory=Pas)


class Executeur:
    def __init__(self, bus: Bus, profil: Profil, conduite: Any,
                 navigation: Navigation | None = None) -> None:
        self.bus, self.profil = bus, profil
        self.conduite = conduite
        self.navigation = navigation
        self._file: deque[tuple[str, Action]] = deque()
        self._courante: _Courante | None = None
        self._verrou = threading.RLock()
        self._ids = itertools.count(1)
        self.etat = "attente"
        self.en_pause = False
        self._dernier_message: dict[str, Any] = {}
        self._actif = False
        self._fil = threading.Thread(target=self._boucle, name="mission", daemon=True)
        # Branchements facultatifs (app.py)
        self.boucle_asyncio: asyncio.AbstractEventLoop | None = None
        self.ia: Callable[[str, Any], Coroutine[Any, Any, Any]] | None = None
        self.pixel_vers_sol: Callable[[str, float, float], tuple[float, float] | None] | None = None
        self.memoire_obstacles: Callable[[], np.ndarray] | None = None
        self.gps: Any = None  # fieldmind.estimation.gps.NavigationGps

    # ------------------------------------------------------------------ interface

    def demarrer(self) -> None:
        self._actif = True
        self._fil.start()

    def arreter(self) -> None:
        self._actif = False

    def ajouter(self, actions: list[Action], remplacer: bool = False,
                origine: str = "poste") -> str:
        """Valide puis ajoute des actions ; renvoie l'identifiant du plan (ou ActionRefusee)."""
        if not actions:
            raise ActionRefusee("aucune action")
        propres = [A.valider(a) for a in actions]
        if any(a["type"] == "stop" for a in propres):
            self.annuler("stop demandé")
            return "stop"
        if all(a["type"] in SANS_MOUVEMENT for a in propres):
            return self._sans_mouvement(propres)  # même rover désarmé : rien ne bouge
        c = self.conduite
        if c.superviseur.arret_urgence:
            raise ActionRefusee("arrêt d'urgence actif : réarmer d'abord")
        if not c.arme:
            raise ActionRefusee("rover désarmé : armer d'abord")
        if c.mode == "banc":
            raise ActionRefusee("mode banc : passer en manuel ou auto")
        propres = [self._ancrer(a) for a in propres]
        p = self.profil
        if bool(p.valeur("remorque.attelee", False)) and not bool(
                p.valeur("remorque.marche_arriere_auto", False)) and any(
                a["type"] == "avancer" and float(a["distance_m"]) < 0 for a in propres):
            raise ActionRefusee("remorque attelée : pas de marche arrière en automatique "
                                "(reculer en manuel, lentement)")
        if c.mode != "auto":
            erreur = c.changer_mode("auto")
            if erreur:
                raise ActionRefusee(erreur)
        plan_id = f"p{next(self._ids)}"
        with self._verrou:
            if remplacer:
                self._file.clear()
                self._courante = None
            for action in propres:
                self._file.append((plan_id, action))
            self.en_pause = False
        resume = ", ".join(A.decrire_action(a) for a in propres)
        self.bus.journaliser("mission", f"plan {plan_id} ({origine}) : {resume}")
        return plan_id

    def _sans_mouvement(self, actions: list[Action]) -> str:
        odom: Odometrie | None = self.bus.dernier(S.ODOM)
        for action in actions:
            if action["type"] == "photo":
                enregistrer_photo(self.bus, action.get("camera", "gauche"))
            elif action["type"] == "memoriser_lieu":
                if self.gps is None or odom is None:
                    raise ActionRefusee("position du rover inconnue")
                try:
                    lieu = self.gps.memoriser(action["nom"], odom.pose)
                except ValueError as erreur:
                    raise ActionRefusee(str(erreur)) from None
                self.bus.journaliser("mission", f"lieu « {lieu['nom']} » mémorisé "
                                     + ("(GPS)" if "lat" in lieu else "(odométrie)"), "succes")
        return f"p{next(self._ids)}"

    def _ancrer(self, action: Action) -> Action:
        """Fixe tout de suite ce qui dépend de l'instant de la demande (clic sur l'image)."""
        if action["type"] != "aller_pixel":
            return action
        if self.pixel_vers_sol is None:
            raise ActionRefusee("projection de l'image indisponible")
        point = self.pixel_vers_sol(action["camera"], action["u"], action["v"])
        if point is None:
            raise ActionRefusee("ce point de l'image n'est pas au sol (trop haut ou trop loin)")
        odom: Odometrie | None = self.bus.dernier(S.ODOM)
        if odom is None:
            raise ActionRefusee("odométrie indisponible")
        x, y = odom.pose.vers_parent(*point)
        return {"type": "aller_a", "x": x, "y": y, "repere": "odom", "depuis": "image"}

    def annuler(self, raison: str = "annulé") -> None:
        with self._verrou:
            avait = self._courante is not None or bool(self._file)
            self._file.clear()
            self._courante = None
            self.en_pause = False
            self.etat = "annule" if avait else self.etat
        if avait:
            self.bus.journaliser("mission", f"mission arrêtée : {raison}", "alerte")
        self.conduite.proposer(Consigne(t=time.monotonic(), source="auto"))
        self._publier(force=True)

    def pause(self) -> None:
        self.en_pause = True

    def reprendre(self) -> None:
        self.en_pause = False

    def buts_gps(self) -> tuple[dict[str, float] | None, list[list[float]]]:
        """But GPS en cours et points GPS de la file (pour la carte GPS du poste)."""
        c = self._courante
        but = None
        if c is not None:
            primitive = c.primitive
            primitive = getattr(primitive, "_sous", None) if isinstance(primitive, _AllerLieu) \
                else primitive
            if isinstance(primitive, AllerGps):
                but = {"lat": primitive.action["lat"], "lon": primitive.action["lon"]}
        parcours = [[a["lat"], a["lon"]] for _, a in list(self._file) if a["type"] == "aller_gps"]
        return but, parcours

    def chemin_courant(self) -> tuple[Chemin | None, tuple[float, float] | None]:
        """Chemin planifié et but de l'action en cours (pour la carte du poste)."""
        c = self._courante
        primitive = c.primitive if c else None
        for _ in range(3):  # AllerLieu → AllerGps → AllerA
            sous = getattr(primitive, "_sous", None)
            if sous is None:
                break
            primitive = sous
        if isinstance(primitive, A.AllerA):
            return primitive.chemin, primitive.but
        return None, None

    def occupe(self) -> bool:
        return self._courante is not None or bool(self._file)

    def resume(self) -> dict[str, Any]:
        with self._verrou:
            c = self._courante
            return {
                "etat": "pause" if self.en_pause and c else self.etat,
                "action": _publique(c.action) if c else None,
                "progression": round(c.dernier.progression, 3) if c else None,
                "detail": c.dernier.detail if c else None,
                "file": [_publique(a) for _, a in self._file],
                "plan_id": c.plan_id if c else None,
            }

    # ------------------------------------------------------------------ IA

    def lancer_ia(self, demande: str, argument: Any) -> Future[Any] | None:
        if self.ia is None or self.boucle_asyncio is None:
            return None
        return asyncio.run_coroutine_threadsafe(self.ia(demande, argument), self.boucle_asyncio)

    # ------------------------------------------------------------------ boucle

    def _boucle(self) -> None:
        prochaine = time.monotonic()
        while self._actif:
            try:
                self._cycle(time.monotonic())
            except Exception as erreur:
                self.bus.journaliser("mission", f"erreur de l'exécuteur : {erreur!r}", "erreur")
                self.annuler("erreur interne")
            prochaine += PERIODE_S
            time.sleep(max(0.0, prochaine - time.monotonic()))
            if time.monotonic() - prochaine > 1.0:
                prochaine = time.monotonic()

    def _cycle(self, t: float) -> None:
        c = self.conduite
        if self.occupe() and (c.mode != "auto" or not c.arme or c.superviseur.arret_urgence):
            raison = ("arrêt d'urgence" if c.superviseur.arret_urgence
                      else "rover désarmé" if not c.arme else f"mode {c.mode}")
            self.annuler(raison)
            return
        with self._verrou:
            if self._courante is None and self._file:
                plan_id, action = self._file.popleft()
                self._courante = _Courante(plan_id, action, self._primitive(action), t)
                self.etat = "route"
                self.bus.journaliser("mission", f"début : {A.decrire_action(action)}")
            courante = self._courante
        if courante is None:
            self._publier()
            return
        if self.en_pause:
            c.proposer(Consigne(t=t, source="auto"))
            self._publier()
            return
        ctx = self._contexte(t)
        if ctx is None:
            c.proposer(Consigne(t=t, source="auto"))
            courante.dernier = Pas(etat="bloque", detail="odométrie indisponible")
            self.etat = "bloque"
            if t - courante.debut > 5:
                self._fin(courante, Pas(etat="echec", raison="odométrie indisponible"))
            self._publier()
            return
        pas = courante.primitive.pas(ctx)
        courante.dernier = pas
        if pas.etat in ("termine", "echec"):
            c.proposer(Consigne(t=t, source="auto"))
            self._fin(courante, pas)
        else:
            self.etat = pas.etat
            c.proposer(Consigne(t=t, vitesse=pas.vitesse, braquage=pas.braquage, source="auto"))
        self._publier()

    def _fin(self, courante: _Courante, pas: Pas) -> None:
        texte = A.decrire_action(courante.action)
        with self._verrou:
            self._courante = None
            if pas.etat == "echec":
                self._file.clear()
                self.etat = "echec"
            elif not self._file:
                self.etat = "termine"
        if pas.etat == "echec":
            message = f"échec : {texte} ({pas.raison})"
            self.bus.journaliser("mission", message, "alerte")
            self.bus.publier(S.CHAT, {"type": "chat", "id": f"x{time.monotonic_ns()}",
                                      "t": time.time(), "role": "systeme", "texte": message})
        else:
            self.bus.journaliser("mission", f"terminé : {texte}", "succes")
            if self.etat == "termine":
                self.bus.publier(S.CHAT, {"type": "chat", "id": f"x{time.monotonic_ns()}",
                                          "t": time.time(), "role": "systeme",
                                          "texte": f"Terminé : {texte}."})

    def _primitive(self, action: Action) -> Any:
        t = action["type"]
        if t == "chercher":
            return Chercher(action, self)
        if t == "decrire":
            return AttenteIa(action, self)
        if t == "cap":
            return _Cap(action)
        if t == "photo":
            return _Photo(action, self.bus)
        if t == "aller_gps":
            return AllerGps(action, self)
        if t == "memoriser_lieu":
            return _MemoriserLieu(action, self)
        if t == "aller_lieu":
            return _AllerLieu(action, self)
        if t == "calibration":
            from fieldmind.mission.calibration import creer_calibration

            return creer_calibration(action, self)
        return A.creer(action)

    def _contexte(self, t: float) -> Contexte | None:
        odom: Odometrie | None = self.bus.dernier(S.ODOM)
        if odom is None or self.bus.age(S.ODOM) > 1.0:
            return None
        p = self.profil
        scan: Scan | None = self.bus.dernier(S.SCAN)
        morceaux = [scan.points()] if scan is not None and self.bus.age(S.SCAN) < 0.5 else []
        stereo: Obstacles | None = self.bus.dernier(S.OBSTACLES_STEREO)
        if stereo is not None and self.bus.age(S.OBSTACLES_STEREO) < 1.0 and len(stereo.points):
            morceaux.append(stereo.points.astype(np.float32))
        if self.memoire_obstacles is not None:
            try:
                memoire = self.memoire_obstacles()
                if len(memoire):
                    morceaux.append(np.asarray(memoire, np.float32))
            except Exception:
                pass
        obstacles = np.vstack(morceaux) if morceaux else np.zeros((0, 2), np.float32)
        return Contexte(
            t=t, odom=odom, securite=self.bus.dernier(S.SECURITE), carte=self.bus.dernier(S.CARTE),
            obstacles=obstacles, vitesse_max=float(p.valeur("vitesses.max_auto_m_s", 0.5)),
            vitesse_defaut=float(p.valeur("vitesses.defaut_action_m_s", 0.35)),
            braquage_max=braquage_max(p),
            empattement=float(p.geometrie.empattement_m), navigation=self.navigation)

    def _publier(self, force: bool = False) -> None:
        message = self.resume()
        if force or message != self._dernier_message:
            self._dernier_message = message
            self.bus.publier(S.MISSION, message)


@dataclass
class _Cap:
    action: Action
    _tourner: A.Tourner | None = None

    def pas(self, ctx: Contexte) -> Pas:
        if self._tourner is None:
            ecart = math.degrees(A.normaliser_angle(
                math.radians(float(self.action["cap_deg"])) - ctx.odom.pose.cap))
            self._tourner = A.Tourner({"type": "tourner", "angle_deg": ecart})
        return self._tourner.pas(ctx)


@dataclass
class _MemoriserLieu:
    action: Action
    executeur: Executeur

    def pas(self, ctx: Contexte) -> Pas:
        if self.executeur.gps is None:
            return Pas(etat="echec", raison="lieux indisponibles")
        lieu = self.executeur.gps.memoriser(self.action["nom"], ctx.odom.pose)
        precision = "GPS" if "lat" in lieu else "odométrie seulement (GPS pas encore aligné)"
        self.executeur.bus.journaliser("mission", f"lieu « {lieu['nom']} » mémorisé ({precision})",
                                       "succes")
        return Pas(etat="termine", progression=1.0)


@dataclass
class _AllerLieu:
    action: Action
    executeur: Executeur
    _sous: Any = None

    def pas(self, ctx: Contexte) -> Pas:
        if self._sous is None:
            gps = self.executeur.gps
            lieu = gps.lieu(self.action["nom"]) if gps is not None else None
            if lieu is None:
                return Pas(etat="echec", raison=f"lieu « {self.action['nom']} » inconnu")
            if "lat" in lieu and lieu.get("lat") is not None:
                self._sous = AllerGps({"type": "aller_gps", "lat": lieu["lat"],
                                       "lon": lieu["lon"]}, self.executeur)
            elif gps is not None and (position := gps.lieu_vers_odom(lieu)) is not None:
                self._sous = A.AllerA({"type": "aller_a", "x": position[0], "y": position[1],
                                       "repere": "odom"})
            else:
                return Pas(etat="echec", raison="lieu mémorisé sans GPS lors d'une autre session")
        return self._sous.pas(ctx)


@dataclass
class _Photo:
    action: Action
    bus: Bus

    def pas(self, ctx: Contexte) -> Pas:
        enregistrer_photo(self.bus, self.action.get("camera", "gauche"))
        return Pas(etat="termine", progression=1.0)


def enregistrer_photo(bus: Bus, camera: str) -> Path | None:
    if camera == "profondeur":
        donnees = bus.dernier(S.PROFONDEUR)
    elif camera == "drone":
        image_drone: ImageCamera | None = bus.dernier(S.IMAGE_DRONE)
        donnees = image_drone.jpeg if image_drone else None
    else:
        image: ImageCamera | None = bus.dernier(S.IMAGE_GAUCHE if camera == "gauche"
                                                else S.IMAGE_DROITE)
        donnees = image.jpeg if image else None
    if not donnees:
        bus.journaliser("photo", f"pas d'image {camera}", "alerte")
        return None
    DOSSIER_PHOTOS.mkdir(parents=True, exist_ok=True)
    chemin = DOSSIER_PHOTOS / f"{time.strftime('%Y%m%d_%H%M%S')}_{camera}.jpg"
    chemin.write_bytes(donnees)
    bus.journaliser("photo", f"photo enregistrée : {chemin.name}", "succes")
    return chemin


def _publique(action: Action) -> Action:
    return {k: v for k, v in action.items() if k != "depuis"}
