"""Primitives de mouvement exécutées par la file de mission (mode auto).

Chaque action est une petite machine à états : `pas(ctx, t)` rend une consigne (vitesse,
braquage) et son état. Les distances et les angles sont mesurés par l'odométrie (recalage lidar +
gyroscope). La sécurité reste au-dessus : si elle bloque, l'action attend que la voie se libère
(`ATTENTE_BLOCAGE_S`), puis échoue avec sa raison.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Literal, Protocol

import numpy as np

from fieldmind.types import (
    Action,
    CarteOccupation,
    Chemin,
    EtatSecurite,
    Odometrie,
    Pose2D,
    normaliser_angle,
)

ATTENTE_BLOCAGE_S = 10.0
TOLERANCE_DISTANCE_M = 0.04
TOLERANCE_ANGLE_RAD = math.radians(3.0)
TOLERANCE_BUT_M = 0.25

EtatAction = Literal["route", "bloque", "termine", "echec"]


class ActionRefusee(Exception):
    """Action invalide ou impossible maintenant ; le message est lisible par l'opérateur."""

    def __init__(self, raison: str) -> None:
        super().__init__(raison)
        self.raison = raison


@dataclass
class Contexte:
    """Ce que voit une action à chaque pas (rempli par l'exécuteur)."""

    t: float
    odom: Odometrie
    securite: EtatSecurite | None
    carte: CarteOccupation | None
    obstacles: np.ndarray  # (N, 2) base_link : lidar + stéréo
    vitesse_max: float  # m/s du mode auto
    vitesse_defaut: float
    braquage_max: float  # rad
    empattement: float
    navigation: Any = None  # fieldmind.mission.executeur.Navigation
    ia: Any = None


@dataclass
class Pas:
    vitesse: float = 0.0
    braquage: float = 0.0
    etat: EtatAction = "route"
    detail: str = ""
    progression: float = 0.0
    raison: str | None = None


class Primitive(Protocol):
    action: Action

    def pas(self, ctx: Contexte) -> Pas: ...


# ---------------------------------------------------------------- validation


def _nombre(action: Action, cle: str, bas: float, haut: float, defaut: float | None = None
            ) -> float:
    valeur = action.get(cle, defaut)
    if valeur is None:
        raise ActionRefusee(f"{action.get('type')} : « {cle} » manquant")
    try:
        nombre = float(valeur)
    except (TypeError, ValueError):
        raise ActionRefusee(f"{action.get('type')} : « {cle} » n'est pas un nombre") from None
    if not math.isfinite(nombre) or not bas <= nombre <= haut:
        raise ActionRefusee(
            f"{action.get('type')} : {cle} = {nombre:g} hors de [{bas:g}, {haut:g}]")
    return nombre


def valider(action: Action) -> Action:
    """Action propre (types convertis, bornes vérifiées) ou ActionRefusee."""
    if not isinstance(action, dict) or not isinstance(action.get("type"), str):
        raise ActionRefusee("action sans type")
    t = action["type"]
    if t == "avancer":
        d = _nombre(action, "distance_m", -20, 20)
        if abs(d) < 0.02:
            raise ActionRefusee("avancer : distance trop petite")
        propre: Action = {"type": t, "distance_m": d}
        if action.get("vitesse_m_s") is not None:
            propre["vitesse_m_s"] = _nombre(action, "vitesse_m_s", 0.05, 1.5)
        return propre
    if t == "tourner":
        return {"type": t, "angle_deg": _nombre(action, "angle_deg", -720, 720)}
    if t == "cap":
        return {"type": t, "cap_deg": _nombre(action, "cap_deg", -360, 360)}
    if t == "aller_a":
        repere = action.get("repere", "robot")
        if repere not in ("robot", "odom"):
            raise ActionRefusee("aller_a : repère « robot » ou « odom »")
        x, y = _nombre(action, "x", -100, 100), _nombre(action, "y", -100, 100)
        if repere == "robot" and math.hypot(x, y) > 30:
            raise ActionRefusee("aller_a : but à plus de 30 m")
        return {"type": t, "x": x, "y": y, "repere": repere}
    if t == "aller_pixel":
        camera = action.get("camera", "gauche")
        if camera not in ("gauche", "droite"):
            raise ActionRefusee("aller_pixel : caméra inconnue")
        return {"type": t, "camera": camera, "u": _nombre(action, "u", 0, 1),
                "v": _nombre(action, "v", 0, 1)}
    if t == "attendre":
        return {"type": t, "duree_s": _nombre(action, "duree_s", 0, 600)}
    if t in ("retour_depart", "stop", "decrire"):
        propre = {"type": t}
        if t == "decrire" and action.get("camera") in ("gauche", "droite"):
            propre["camera"] = action["camera"]
        if t == "decrire" and isinstance(action.get("question"), str):
            propre["question"] = action["question"][:200]
        return propre
    if t == "chercher":
        objet = action.get("objet")
        if not isinstance(objet, str) or not 1 <= len(objet.strip()) <= 80:
            raise ActionRefusee("chercher : objet manquant")
        propre = {"type": t, "objet": objet.strip(),
                  "approcher": bool(action.get("approcher", action.get("aller", True)))}
        if action.get("camera") in ("gauche", "droite"):
            propre["camera"] = action["camera"]
        return propre
    if t == "photo":
        camera = action.get("camera", "gauche")
        if camera not in ("gauche", "droite", "profondeur"):
            raise ActionRefusee("photo : caméra inconnue")
        return {"type": t, "camera": camera}
    if t == "aller_gps":
        propre = {"type": t, "lat": _nombre(action, "lat", -90, 90),
                  "lon": _nombre(action, "lon", -180, 180)}
        if action.get("tolerance_m") is not None:
            propre["tolerance_m"] = _nombre(action, "tolerance_m", 0.5, 20)
        return propre
    if t in ("memoriser_lieu", "aller_lieu"):
        nom = action.get("nom")
        if not isinstance(nom, str) or not 1 <= len(nom.strip()) <= 40:
            raise ActionRefusee(f"{t} : nom de lieu manquant")
        return {"type": t, "nom": " ".join(nom.split())}
    if t == "calibration":
        nom = action.get("nom")
        if nom not in ("sens_roues", "vitesse", "braquage", "imu_repos"):
            raise ActionRefusee("calibration inconnue")
        return {"type": t, "nom": nom}
    raise ActionRefusee(f"type d'action inconnu : {t}")


def decrire_action(action: Action) -> str:
    t = action["type"]
    if t == "avancer":
        d = action["distance_m"]
        return f"{'avancer' if d > 0 else 'reculer'} de {abs(d):.2f} m".replace(".", ",")
    if t == "tourner":
        a = action["angle_deg"]
        return f"tourner de {abs(a):.0f}° à {'gauche' if a > 0 else 'droite'}"
    if t == "cap":
        return f"s'orienter au cap {action['cap_deg']:.0f}°"
    if t == "aller_a":
        texte = f"aller en ({action['x']:.1f} ; {action['y']:.1f}) {action['repere']}"
        return texte.replace(".", ",")
    if t == "aller_pixel":
        return "aller au point visé sur l'image"
    if t == "attendre":
        return f"attendre {action['duree_s']:.0f} s"
    if t == "chercher":
        return f"chercher « {action['objet']} »"
    if t == "aller_gps":
        return f"aller au point GPS {action['lat']:.6f}, {action['lon']:.6f}"
    if t == "memoriser_lieu":
        return f"mémoriser le lieu « {action['nom']} »"
    if t == "aller_lieu":
        return f"aller à « {action['nom']} »"
    return {"retour_depart": "retour au départ", "stop": "arrêt", "decrire": "décrire la scène",
            "photo": "photo", "calibration": f"calibration {action.get('nom')}"}.get(t, t)


# ---------------------------------------------------------------- outils communs


def _ralentir(reste: float, vitesse: float, deceleration: float = 0.6, mini: float = 0.08
              ) -> float:
    """Vitesse qui permet de s'arrêter au but (en m/s, ≥ mini tant qu'il reste du chemin)."""
    return max(mini, min(vitesse, math.sqrt(max(0.0, 2 * deceleration * reste))))


def _bloque(ctx: Contexte, sens: int) -> bool:
    s = ctx.securite
    if s is None:
        return True
    return (s.vitesse_max_avant if sens > 0 else s.vitesse_max_arriere) < 0.04


@dataclass
class _Attente:
    depuis: float | None = None

    def suivre(self, bloque: bool, t: float) -> float:
        if not bloque:
            self.depuis = None
            return 0.0
        if self.depuis is None:
            self.depuis = t
        return t - self.depuis


# ---------------------------------------------------------------- primitives


@dataclass
class Avancer:
    """Ligne droite sur le cap de départ, distance mesurée par l'odométrie."""

    action: Action
    depart: Pose2D | None = None
    _attente: _Attente = field(default_factory=_Attente)

    def pas(self, ctx: Contexte) -> Pas:
        distance = float(self.action["distance_m"])
        sens = 1 if distance > 0 else -1
        pose = ctx.odom.pose
        if self.depart is None:
            self.depart = pose
        d = self.depart
        # Position le long de la droite de départ, écart latéral, erreur de cap
        lx, ly = d.vers_local(pose.x, pose.y)
        fait = lx * sens
        reste = abs(distance) - fait
        progression = max(0.0, min(1.0, fait / abs(distance)))
        detail = f"{max(0.0, fait):.2f} m sur {abs(distance):.2f} m".replace(".", ",")
        if reste <= TOLERANCE_DISTANCE_M:
            return Pas(etat="termine", progression=1.0, detail=detail)
        attente = self._attente.suivre(_bloque(ctx, sens), ctx.t)
        if attente > ATTENTE_BLOCAGE_S:
            return Pas(etat="echec", raison="obstacle : voie bloquée", detail=detail,
                       progression=progression)
        vitesse = float(self.action.get("vitesse_m_s", ctx.vitesse_defaut))
        vitesse = _ralentir(reste, min(vitesse, ctx.vitesse_max))
        # Retour sur la droite : visée à 0,8 m devant, sur la droite de départ (poursuite pure)
        visee = 0.8
        cible_x, cible_y = d.vers_parent(lx + sens * visee, 0.0)
        bx, by = pose.vers_local(cible_x, cible_y)
        # Arc tangent au cap passant par la visée : même courbure en avant et en arrière
        # (ω = v·tan(δ)/L avec v signée), donc même braquage.
        courbure = 2 * by / max(bx * bx + by * by, 1e-6)
        braquage = math.atan(courbure * ctx.empattement)
        braquage = max(-ctx.braquage_max, min(ctx.braquage_max, braquage))
        _ = ly
        return Pas(vitesse=sens * vitesse, braquage=braquage,
                   etat="bloque" if attente > 0 else "route", detail=detail,
                   progression=progression)


@dataclass
class Tourner:
    """Changement de cap sur un arc au braquage maximal ; manœuvre en plusieurs temps (avant /
    arrière en contre-braquant) si la place manque."""

    action: Action
    cap_initial: float | None = None
    cumul: float = 0.0
    _precedent: float | None = None
    phase: int = 1  # +1 : en avant, -1 : en arrière
    _pose_phase: Pose2D | None = None
    _attente: _Attente = field(default_factory=_Attente)
    manoeuvres: int = 0

    def pas(self, ctx: Contexte) -> Pas:
        angle = math.radians(float(self.action["angle_deg"]))
        pose = ctx.odom.pose
        if self._precedent is None:
            self._precedent = pose.cap
            self._pose_phase = pose
        self.cumul += normaliser_angle(pose.cap - self._precedent)
        self._precedent = pose.cap
        reste = angle - self.cumul
        progression = max(0.0, min(1.0, self.cumul / angle)) if angle else 1.0
        detail = f"{math.degrees(self.cumul):.0f}° sur {math.degrees(angle):.0f}°"
        if abs(reste) <= TOLERANCE_ANGLE_RAD:
            return Pas(etat="termine", progression=1.0, detail=detail)
        gauche = 1 if reste > 0 else -1
        # En avant on braque du côté du virage ; en arrière, à l'opposé (le cap tourne pareil)
        braquage = gauche * ctx.braquage_max * self.phase
        assert self._pose_phase is not None
        parcouru = math.hypot(pose.x - self._pose_phase.x, pose.y - self._pose_phase.y)
        bloque_av, bloque_ar = _bloque(ctx, 1), _bloque(ctx, -1)
        changer = (self.phase > 0 and bloque_av and not bloque_ar) or (
            self.phase < 0 and not bloque_av and (bloque_ar or parcouru > 0.5))
        if changer:
            if self.manoeuvres >= 8:
                return Pas(etat="echec", raison="pas la place de tourner", detail=detail,
                           progression=progression)
            self.phase = -self.phase
            self._pose_phase = pose
            self.manoeuvres += 1
            return Pas(vitesse=0.0, braquage=gauche * ctx.braquage_max * self.phase,
                       detail=detail + " (manœuvre)", progression=progression)
        attente = self._attente.suivre(bloque_av and bloque_ar if self.phase < 0
                                       else bloque_av, ctx.t)
        if attente > ATTENTE_BLOCAGE_S:
            return Pas(etat="echec", raison="pas la place de tourner", detail=detail,
                       progression=progression)
        # Vitesse : lente, et plus lente encore à l'approche du cap
        rayon = ctx.empattement / math.tan(ctx.braquage_max)
        v = _ralentir(abs(reste) * rayon, min(0.3, ctx.vitesse_max), deceleration=0.4)
        return Pas(vitesse=self.phase * v, braquage=braquage,
                   etat="bloque" if attente > 0 else "route", detail=detail,
                   progression=progression)


@dataclass
class Attendre:
    action: Action
    debut: float | None = None

    def pas(self, ctx: Contexte) -> Pas:
        if self.debut is None:
            self.debut = ctx.t
        duree = float(self.action["duree_s"])
        ecoule = ctx.t - self.debut
        if ecoule >= duree:
            return Pas(etat="termine", progression=1.0)
        return Pas(progression=ecoule / duree if duree else 1.0,
                   detail=f"{duree - ecoule:.0f} s")


@dataclass
class AllerA:
    """But dans le repère odom : planificateur (A*) sur la carte + pilote local (arcs)."""

    action: Action
    but: tuple[float, float] | None = None
    chemin: Chemin | None = None
    _dernier_calcul: float = -1e9
    _distance_initiale: float | None = None
    _attente: _Attente = field(default_factory=_Attente)
    _echecs_calcul: int = 0

    def pas(self, ctx: Contexte) -> Pas:
        pose = ctx.odom.pose
        if self.but is None:
            x, y = float(self.action["x"]), float(self.action["y"])
            self.but = pose.vers_parent(x, y) if self.action.get("repere") == "robot" else (x, y)
        bx, by = self.but
        distance = math.hypot(bx - pose.x, by - pose.y)
        if self._distance_initiale is None:
            self._distance_initiale = max(distance, 1e-3)
        progression = max(0.0, min(1.0, 1 - distance / self._distance_initiale))
        detail = f"{distance:.1f} m restants".replace(".", ",")
        if distance <= TOLERANCE_BUT_M:
            return Pas(etat="termine", progression=1.0, detail="arrivé")
        nav = ctx.navigation
        if nav is None:
            return Pas(etat="echec", raison="navigation indisponible")
        if ctx.t - self._dernier_calcul > 2.0 or self.chemin is None:
            self._dernier_calcul = ctx.t
            chemin = nav.planifier(ctx.carte, pose, self.but)
            if chemin is None:
                self._echecs_calcul += 1
                if self._echecs_calcul >= 5:
                    return Pas(etat="echec", raison="aucun chemin vers le but", detail=detail)
            else:
                self._echecs_calcul = 0
                self.chemin = chemin
        if self.chemin is None:
            return Pas(etat="bloque", detail="calcul du chemin", progression=progression)
        commande = nav.commande(ctx.odom, self.chemin, ctx.obstacles, ctx.vitesse_max)
        if commande.arrive:
            return Pas(etat="termine", progression=1.0, detail="arrivé")
        attente = self._attente.suivre(commande.bloque or (
            _bloque(ctx, 1 if commande.vitesse >= 0 else -1) and abs(commande.vitesse) > 0), ctx.t)
        if attente > ATTENTE_BLOCAGE_S:
            return Pas(etat="echec", raison=getattr(commande, "raison", None) or "voie bloquée",
                       detail=detail, progression=progression)
        if commande.bloque:
            self._dernier_calcul = -1e9  # recalculer au prochain pas
        return Pas(vitesse=commande.vitesse, braquage=commande.braquage,
                   etat="bloque" if attente > 0 else "route", detail=detail,
                   progression=progression)


def creer(action: Action) -> Primitive:
    t = action["type"]
    if t == "avancer":
        return Avancer(action)
    if t == "tourner":
        return Tourner(action)
    if t == "attendre":
        return Attendre(action)
    if t == "aller_a":
        return AllerA(action)
    if t == "retour_depart":
        return AllerA({"type": "aller_a", "x": 0.0, "y": 0.0, "repere": "odom"})
    raise ActionRefusee(f"{t} n'est pas une primitive de mouvement")
