"""Remorque porte-drone attelée (facultative) : géométrie, angle d'attelage, emprise balayée.

Cotes : `cao/prototype_final/remorque` (vis d'attelage à 16 cm derrière l'essieu arrière du
rover, essieu de la remorque à 31 cm derrière la vis, planche de 10 à 48 cm derrière elle).

**Angle d'attelage** φ = cap du rover − cap de la remorque (φ > 0 : remorque partie à gauche,
derrière). Il n'y a pas de capteur à l'attelage : φ est intégré depuis l'odométrie (vitesse et
vitesse de lacet mesurées) avec le modèle d'une remorque à attelage déporté :

    dφ/dt = ω·(1 + (M/L)·cos φ) − (v/L)·sin φ

M : essieu arrière du rover → attelage ; L : attelage → essieu de la remorque.

- **En marche avant**, ce modèle est stable : une erreur sur φ est divisée par e environ tous
  les L mètres parcourus. L'estimation converge même partie de faux (≈ 1 m tout droit suffit).
- **En marche arrière**, il est instable : l'erreur est multipliée d'autant, et la remorque se
  met en portefeuille si on ne la rattrape pas. D'où les règles du superviseur : marche arrière
  lente, seulement si φ est connu (incertitude faible) et reste sous `angle_max_arriere_deg` sur
  toute la distance d'arrêt ; jamais en automatique, sauf réglage contraire.

Le lidar (plan à 32 cm) passe au-dessus de la remorque (27 cm, volets repliés) : il ne la voit
pas. Les obstacles bas derrière ne sont vus par aucun capteur (les caméras regardent devant) :
seule la mémoire des obstacles bas, vus avant par la stéréo, les connaît.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Any

import numpy as np

from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.types import Odometrie

PAS_M = 0.04
SOUS_PAS = 2  # intégration de φ : 2 pas d'Euler par pas d'arc
INCERTITUDE_INITIALE_DEG = 25.0  # remorque déclarée attelée, angle inconnu
INCERTITUDE_ALIGNEE_DEG = 4.0  # « remorque dans l'axe » confirmé par l'opérateur
BRUIT_DEG_PAR_M = 1.5  # erreur de modèle (glissement, jeu de l'attelage), par mètre roulé


@dataclass(frozen=True, slots=True)
class GeometrieRemorque:
    attelage_m: float  # M : essieu arrière du rover → vis d'attelage (vers l'arrière)
    timon_m: float  # L : vis d'attelage → essieu de la remorque
    avant_m: float  # vis d'attelage → bord avant de la planche
    arriere_m: float  # vis d'attelage → bord arrière
    demi_largeur_m: float  # roues comprises, selon les volets
    angle_contact: float  # rad : le coin avant touche une roue arrière du rover

    @classmethod
    def depuis(cls, profil: Profil) -> GeometrieRemorque:
        v = profil.valeur
        volets = bool(v("remorque.volets_deplies", False))
        return cls(
            attelage_m=float(v("remorque.attelage_m", 0.16)),
            timon_m=float(v("remorque.timon_m", 0.31)),
            avant_m=float(v("remorque.avant_m", 0.10)),
            arriere_m=float(v("remorque.arriere_m", 0.48)),
            demi_largeur_m=float(v("remorque.demi_largeur_volets_m" if volets
                                   else "remorque.demi_largeur_m", 0.265 if volets else 0.19)),
            angle_contact=math.radians(float(v("remorque.angle_contact_deg", 52.0))),
        )


def attelee(profil: Profil) -> bool:
    return bool(profil.valeur("remorque.attelee", False))


def derivee_angle(phi: float, v: float, omega: float, g: GeometrieRemorque) -> float:
    """dφ/dt (rad/s) pour une vitesse v (m/s, signée, essieu arrière) et un lacet ω (rad/s)."""
    return omega * (1.0 + g.attelage_m / g.timon_m * math.cos(phi)) - v / g.timon_m * math.sin(phi)


def angle_equilibre(kappa: float, g: GeometrieRemorque) -> float:
    """φ en virage établi, en marche avant, à la courbure κ : sin φ = κ·(L + M·cos φ)."""
    phi = 0.0
    for _ in range(30):  # point fixe, contractant pour les courbures du rover
        s = kappa * (g.timon_m + g.attelage_m * math.cos(phi))
        phi = math.asin(max(-1.0, min(1.0, s)))
    return phi


def angles_le_long(phi0: float, kappa: float, s: np.ndarray, g: GeometrieRemorque) -> np.ndarray:
    """φ après chaque longueur d'arc signée de `s` (régulièrement espacées, même signe)."""
    phis = np.empty(len(s))
    phi, precedent = phi0, 0.0
    for i, si in enumerate(s):
        ds = (float(si) - precedent) / SOUS_PAS
        for _ in range(SOUS_PAS):
            # dφ/ds = κ(1 + (M/L)cos φ) − sin φ / L ; ds < 0 en marche arrière
            phi += ds * (kappa * (1.0 + g.attelage_m / g.timon_m * math.cos(phi))
                         - math.sin(phi) / g.timon_m)
        phis[i] = phi
        precedent = float(si)
    return phis


def dans_remorque(px: np.ndarray, py: np.ndarray, phi: np.ndarray | float, g: GeometrieRemorque,
                  marge: float) -> np.ndarray:
    """Points (repère du rover) dans l'emprise de la remorque à l'angle φ, élargie de `marge`.
    Le timon, étroit et bas, est compté dans la largeur de la planche (par prudence)."""
    c, s = np.cos(phi), np.sin(phi)
    dx = px + g.attelage_m
    xt = c * dx - s * py  # repère de la remorque : origine à l'attelage, x vers son avant
    yt = s * dx + c * py
    return (xt < -0.02) & (xt > -g.arriere_m - marge) & (np.abs(yt) < g.demi_largeur_m + marge)


def distance_libre_remorque(points: np.ndarray, braquage: float, sens: int, phi0: float,
                            profil: Profil, g: GeometrieRemorque, portee: float = 1.5,
                            angle_max: float | None = None) -> tuple[float, int]:
    """Longueur d'arc (essieu arrière du rover) avant que la remorque, entraînée le long de l'arc
    au braquage courant, touche un des `points` (base_link), ou que φ dépasse `angle_max`
    (défaut : angle de contact avec le rover). Renvoie aussi l'indice du point (-1 sinon).

    Les points déjà dans l'emprise au départ sont ignorés : c'est la remorque elle-même (ou son
    drone) si un capteur la voit, ou un obstacle qui la touche déjà."""
    marge = float(profil.valeur("securite.marge_laterale_m", 0.10)) * 0.5
    kappa = math.tan(braquage) / profil.geometrie.empattement_m
    limite = g.angle_contact - math.radians(4.0) if angle_max is None else angle_max
    s = np.arange(PAS_M, portee + PAS_M, PAS_M) * sens
    phis = angles_le_long(phi0, kappa, s, g)
    hors = np.nonzero(np.abs(phis) > limite)[0]
    fin = len(s) if len(hors) == 0 else int(hors[0])  # mise en portefeuille : là, on s'arrête
    if abs(phi0) > limite:
        # Déjà au-delà : seul un mouvement qui referme l'angle est permis
        fin = 0 if len(phis) == 0 or abs(phis[0]) >= abs(phi0) else len(s)
    libre_angle = abs(float(s[fin - 1])) if fin > 0 else 0.0
    if fin == len(s):
        libre_angle = portee
    if len(points) == 0 or fin == 0:
        return (libre_angle, -1)
    indices = np.arange(len(points))
    portee_utile = portee + g.arriere_m + g.attelage_m + g.demi_largeur_m + marge
    garde = np.hypot(points[:, 0], points[:, 1]) < portee_utile
    garde &= ~dans_remorque(points[:, 0], points[:, 1], phi0, g, marge)
    points, indices = points[garde], indices[garde]
    if len(points) == 0:
        return (libre_angle, -1)
    for debut in range(0, fin, 16):
        sl = slice(debut, min(fin, debut + 16))
        ss, ph = s[sl], phis[sl]
        if abs(kappa) < 1e-4:
            px = points[None, :, 0] - ss[:, None]
            py = np.broadcast_to(points[None, :, 1], px.shape)
        else:
            theta = kappa * ss
            cx, cy = np.sin(theta) / kappa, (1 - np.cos(theta)) / kappa
            c, si = np.cos(theta), np.sin(theta)
            dx = points[None, :, 0] - cx[:, None]
            dy = points[None, :, 1] - cy[:, None]
            px = c[:, None] * dx + si[:, None] * dy
            py = -si[:, None] * dx + c[:, None] * dy
        dedans = dans_remorque(px, py, ph[:, None], g, marge)
        touche = dedans.any(axis=1)
        if touche.any():
            k = int(np.argmax(touche))
            indice = int(indices[int(np.argmax(dedans[k]))])
            return max(0.0, abs(float(ss[k])) - PAS_M), indice
    return (libre_angle, -1)


def emprise_monde(x: float, y: float, cap: float, phi: float,
                  g: GeometrieRemorque) -> list[tuple[float, float]]:
    """Coins de la planche (repère du rover donné par x, y, cap), pour l'affichage."""
    hx, hy = x - g.attelage_m * math.cos(cap), y - g.attelage_m * math.sin(cap)
    cap_r = cap - phi
    c, s = math.cos(cap_r), math.sin(cap_r)
    coins = [(-g.avant_m, g.demi_largeur_m), (-g.arriere_m, g.demi_largeur_m),
             (-g.arriere_m, -g.demi_largeur_m), (-g.avant_m, -g.demi_largeur_m)]
    return [(round(hx + c * a - s * b, 3), round(hy + s * a + c * b, 3)) for a, b in coins]


@dataclass(slots=True)
class EtatRemorque:
    t: float
    attelee: bool
    angle: float = 0.0  # rad, φ estimé
    incertitude: float = math.radians(INCERTITUDE_INITIALE_DEG)  # rad, ~ 2 écarts types

    def resume(self, g: GeometrieRemorque, profil: Profil) -> dict[str, Any]:
        recul = (self.attelee and math.degrees(self.incertitude) <= float(
            profil.valeur("remorque.incertitude_max_recul_deg", 8.0)))
        return {"attelee": self.attelee, "angle_deg": round(math.degrees(self.angle), 1),
                "incertitude_deg": round(math.degrees(self.incertitude), 1),
                "recul_possible": recul,
                "geometrie": {"attelage_m": g.attelage_m, "timon_m": g.timon_m,
                              "avant_m": g.avant_m, "arriere_m": g.arriere_m,
                              "demi_largeur_m": g.demi_largeur_m}}


class SuiviRemorque:
    """Intègre φ à chaque odométrie publiée et publie `S.REMORQUE`."""

    def __init__(self, bus: Bus, profil: Profil) -> None:
        self.bus, self.profil = bus, profil
        self._verrou = threading.Lock()
        self._dernier_t: float | None = None
        self.etat = EtatRemorque(t=0.0, attelee=attelee(profil))
        bus.abonner(S.ODOM, self._odom)
        profil.sur_changement(self._reglage)

    def geometrie(self) -> GeometrieRemorque:
        return GeometrieRemorque.depuis(self.profil)

    def aligner(self) -> None:
        """L'opérateur confirme que la remorque est dans l'axe du rover."""
        with self._verrou:
            self.etat.angle = 0.0
            self.etat.incertitude = math.radians(INCERTITUDE_ALIGNEE_DEG)
        self.bus.journaliser("remorque", "remorque déclarée dans l'axe")
        self.bus.publier(S.REMORQUE, self.etat)

    def _reglage(self, cle: str, valeur: Any) -> None:
        if cle != "remorque.attelee":
            return
        with self._verrou:
            self.etat = EtatRemorque(t=self.etat.t, attelee=bool(valeur))
        if valeur:
            self.bus.journaliser("remorque", "remorque attelée : vitesse réduite, marche arrière "
                                 "limitée ; la mettre dans l'axe ou rouler 1 m tout droit")
        else:
            self.bus.journaliser("remorque", "remorque dételée")
        self.bus.publier(S.REMORQUE, self.etat)

    def _odom(self, odom: Odometrie) -> None:
        with self._verrou:
            precedent, self._dernier_t = self._dernier_t, odom.t
            e = self.etat
            e.t = odom.t
            if not e.attelee or precedent is None:
                return
            dt = odom.t - precedent
            if dt <= 0 or dt > 0.5:
                return
            g = self.geometrie()
            v, omega = odom.vitesse, odom.vitesse_lacet
            e.angle += dt * derivee_angle(e.angle, v, omega, g)
            limite = g.angle_contact + math.radians(8.0)  # rien au-delà : l'attelage bute
            e.angle = max(-limite, min(limite, e.angle))
            ds = v * dt
            # Erreur linéarisée : d(δφ)/ds = −(cos φ + κ·M·sin φ)/L · δφ, κ ≈ ω/v
            kappa = omega / v if abs(v) > 1e-3 else 0.0
            taux = (math.cos(e.angle) + kappa * g.attelage_m * math.sin(e.angle)) / g.timon_m
            e.incertitude = e.incertitude * math.exp(-taux * ds) + math.radians(
                BRUIT_DEG_PAR_M) * abs(ds)
            e.incertitude = min(e.incertitude, math.radians(60.0))
        self.bus.publier(S.REMORQUE, self.etat)
