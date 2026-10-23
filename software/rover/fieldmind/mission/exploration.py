"""Exploration progressive, bornée, dans l'espace effectivement mesuré.

Les calculs de chemin et les appels vision ne bloquent pas la boucle de mission. Chaque
point visité déclenche une nouvelle observation ; l'inconnu est une frontière à observer,
jamais un raccourci autorisé pour rejoindre un point de vue.
"""

from __future__ import annotations

import math
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field, replace
from typing import Any

import cv2
import numpy as np

from fieldmind.mission.actions import AllerA, Contexte, Pas
from fieldmind.navigation.local import CommandeLocale
from fieldmind.types import CarteOccupation, Chemin, Pose2D


def _distance_connue(carte: CarteOccupation) -> np.ndarray:
    libre = ((carte.cellules >= 0) & (carte.cellules < 65)).astype(np.uint8)
    # Le bord de carte est aussi une frontière inconnue (OpenCV l'ignorerait sinon).
    return cv2.distanceTransform(np.pad(libre, 1), cv2.DIST_L2, 5)[1:-1, 1:-1] * carte.resolution


def points_exploration(carte: CarteOccupation, pose: Pose2D, origine: Pose2D,
                       visites: list[tuple[float, float]], rayon: float,
                       marge: float = 0.35,
                       distance: np.ndarray | None = None) -> list[dict[str, float]]:
    grille, res = carte.cellules, carte.resolution
    distance = _distance_connue(carte) if distance is None else distance
    libre = (distance >= marge).astype(np.uint8)
    lig_pose = math.floor((pose.y - carte.origine_y) / res)
    col_pose = math.floor((pose.x - carte.origine_x) / res)
    if not (0 <= lig_pose < grille.shape[0] and 0 <= col_pose < grille.shape[1]):
        return []
    _, composantes = cv2.connectedComponents(libre, connectivity=4)
    composante = composantes[lig_pose, col_pose]
    if not composante:
        return []
    pas = max(1, int(round(0.6 / res)))
    lig, col = np.mgrid[0:grille.shape[0]:pas, 0:grille.shape[1]:pas]
    x, y = carte.origine_x + (col + 0.5) * res, carte.origine_y + (lig + 0.5) * res
    d = np.hypot(x - pose.x, y - pose.y)
    garde = ((composantes[lig, col] == composante) & (grille[lig, col] <= 45)
             & (d >= 0.7) & (d <= 3.5)
             & (np.hypot(x - origine.x, y - origine.y) <= rayon))
    for vx, vy in visites:
        garde &= np.hypot(x - vx, y - vy) >= 1.1
    lig, col, x, y, d = (v[garde] for v in (lig, col, x, y, d))
    n = max(3, int(round(2.5 / res)) | 1)
    gain = cv2.boxFilter((grille < 0).astype(np.float32), -1, (n, n),
                        borderType=cv2.BORDER_CONSTANT)[lig, col]
    scores = (1 + 4 * gain) / (0.8 + d)
    resultat: list[dict[str, float]] = []
    for k in np.argsort(-scores, kind="stable"):
        if all(math.hypot(x[k] - p["x"], y[k] - p["y"]) > 0.8 for p in resultat):
            resultat.append({"x": float(x[k]), "y": float(y[k]), "gain": float(gain[k])})
        if len(resultat) >= 12:
            break
    return resultat


def chemin_connu(carte: CarteOccupation, chemin: Chemin, marge: float,
                 distance: np.ndarray | None = None) -> bool:
    """Valide le trajet ; une distance précalculée appartient à cette version de carte."""
    points = np.asarray(chemin.points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points) or not np.isfinite(points).all():
        return False
    distance = _distance_connue(carte) if distance is None else distance
    if distance.shape != carte.cellules.shape:
        return False
    origine = np.array([carte.origine_x, carte.origine_y])
    dimensions = np.array([distance.shape[1], distance.shape[0]])
    cellules = (points - origine) / carte.resolution
    if np.any(cellules < 0) or np.any(cellules >= dimensions):
        return False
    if len(points) > 1:
        # Tous les segments dans un même lot : éviter 100 petits contrôles NumPy par chemin.
        deltas = np.diff(points, axis=0)
        nombres = np.maximum(1, np.ceil(np.linalg.norm(deltas, axis=1)
                                       / (carte.resolution * 0.5)).astype(int))
        segments = np.repeat(np.arange(len(deltas)), nombres)
        debuts = np.repeat(np.cumsum(nombres) - nombres, nombres)
        fractions = (np.arange(len(segments)) - debuts) / np.repeat(nombres, nombres)
        echantillons = points[segments] + deltas[segments] * fractions[:, None]
        points = np.vstack((echantillons, points[-1]))
    indices = np.floor((points - origine) / carte.resolution).astype(int)
    return bool(np.all(distance[indices[:, 1], indices[:, 0]] >= marge))


def _planifier(navigation: Any, carte: CarteOccupation, pose: Pose2D,
               cible: tuple[float, float], marge: float,
               distance: np.ndarray | None = None) -> Chemin | None:
    # Une copie évite de changer la carte partagée par la conduite et l'interface.
    cellules = np.where(carte.cellules < 0, 100, carte.cellules).astype(np.int8)
    chemin = navigation.planifier(replace(carte, cellules=cellules), pose, cible)
    if (chemin is None or math.dist(chemin.but, cible) > 0.25
            or not chemin_connu(carte, chemin, marge, distance)):
        return None
    return chemin


@dataclass
class _NavigationConnue:
    navigation: Any
    pool: ThreadPoolExecutor
    marge: float
    empattement: float
    carte: CarteOccupation
    _calcul: Future[Any] | None = None
    _cle_distance: tuple[Any, ...] | None = None
    _distance: np.ndarray | None = None
    _carte_distance: CarteOccupation | None = None

    def _distance_courante(self) -> np.ndarray:
        """Cache local : chaque carte publiée est immuable ; une mutation exige un nouveau t.

        L'estimation publie une nouvelle carte à 2 Hz. Invalider immédiatement à chaque version
        évite d'ignorer un nouvel obstacle entre deux calculs ; aucun délai artificiel de 1 s.
        La référence conservée empêche aussi la réutilisation d'identité d'un objet détruit.
        """
        carte = self.carte
        cle = (id(carte), carte.t, id(carte.cellules), carte.cellules.shape,
               carte.resolution, carte.origine_x, carte.origine_y)
        if cle != self._cle_distance:
            self._distance = _distance_connue(carte)
            self._cle_distance = cle
            self._carte_distance = carte
        assert self._distance is not None
        return self._distance

    def fermer(self) -> None:
        if self._calcul is not None:
            self._calcul.cancel()
            self._calcul = None

    def planifier(self, carte: CarteOccupation, pose: Pose2D,
                  cible: tuple[float, float]) -> Chemin | None:
        self.carte = carte
        if self._calcul is None:
            self._calcul = self.pool.submit(_planifier, self.navigation, carte, pose,
                                            cible, self.marge)
        if not self._calcul.done():
            return None
        calcul, self._calcul = self._calcul, None
        chemin = calcul.result()
        return chemin if chemin is not None and chemin_connu(
            carte, chemin, self.marge, self._distance_courante()) else None

    def commande(self, odom: Any, chemin: Chemin, obstacles: Any, vitesse_max: float,
                 indicatifs: Any = None) -> Any:
        distance = self._distance_courante()
        # Un obstacle nouvellement cartographié doit arrêter même un ancien chemin en cache.
        indice = int(np.argmin(np.linalg.norm(chemin.points - (odom.pose.x, odom.pose.y), axis=1)))
        restant = np.vstack(((odom.pose.x, odom.pose.y), chemin.points[indice:]))
        if not chemin_connu(self.carte, replace(chemin, points=restant), self.marge, distance):
            return CommandeLocale(0.0, 0.0, bloque=True, raison="chemin hors de la zone connue")
        commande = self.navigation.commande(odom, chemin, obstacles, vitesse_max, indicatifs)
        # Le pilote local peut choisir un arc d'évitement : vérifier aussi cet arc mesuré.
        temps = np.linspace(0.0, 1.0, 21)
        v = commande.vitesse
        w = v * math.tan(commande.braquage) / self.empattement
        cap = odom.pose.cap
        if abs(w) < 1e-6:
            xs = odom.pose.x + v * temps * math.cos(cap)
            ys = odom.pose.y + v * temps * math.sin(cap)
        else:
            xs = odom.pose.x + v / w * (np.sin(cap + w * temps) - math.sin(cap))
            ys = odom.pose.y - v / w * (np.cos(cap + w * temps) - math.cos(cap))
        if not chemin_connu(self.carte, replace(chemin, points=np.column_stack((xs, ys))),
                            self.marge, distance):
            return CommandeLocale(0.0, 0.0, bloque=True, raison="arc hors de la zone connue")
        return commande


@dataclass
class Explorer:
    action: dict[str, Any]
    executeur: Any
    _debut: float | None = None
    _origine: Pose2D | None = None
    _visites: list[tuple[float, float]] = field(default_factory=list)
    _points: list[dict[str, Any]] = field(default_factory=list)
    _prochains: list[dict[str, float]] = field(default_factory=list)
    _sous: Any = None
    _analyse: Any = None
    _analyse_debut: float = 0.0
    _pose_image: Pose2D | None = None
    _observer: bool = True
    _approche: bool = False
    _bilan: str = ""
    _terminal: Pas | None = None
    _pool: ThreadPoolExecutor | None = None
    _calcul: Future[Any] | None = None
    _calcul_debut: float = 0.0
    _navigation: _NavigationConnue | None = None
    _ferme: bool = False
    _choix: Future[Any] | None = None
    _choix_debut: float = 0.0
    _chemins: list[Chemin] = field(default_factory=list)
    _generation: int = 0
    _confirmation_a: float = 0.0
    _essais_confirmation: int = 0
    _objet_id: int | None = None
    piece_trouvee: dict[str, Any] | None = None

    def fermer(self) -> None:
        self._ferme = True
        for nom in ("_analyse", "_calcul", "_choix"):
            future = getattr(self, nom)
            if future is not None:
                future.cancel()
                setattr(self, nom, None)
        if self._navigation is not None:
            self._navigation.fermer()
        if self._pool is not None:
            self._pool.shutdown(wait=False, cancel_futures=True)
            self._pool = None
        for point in self._points:
            if point["etat"] == "en_cours":
                point["etat"] = "interrompu"
        self._sous = None

    def resume_exploration(self) -> dict[str, Any]:
        return {"points": [dict(p) for p in self._points],
                "prochains": [dict(p) for p in self._prochains], "bilan": self._bilan}

    def _fin(self, detail: str, *, echec: bool = False) -> Pas:
        self.fermer()
        self._bilan = detail
        self._terminal = Pas(etat="echec" if echec else "termine", progression=1.0, detail=detail,
                             raison=detail if echec else None)
        return self._terminal

    def _choisir(self, ctx: Contexte, marge: float) -> tuple[list[dict[str, float]], list[Chemin]]:
        assert self._origine is not None and ctx.carte is not None
        distance = _distance_connue(ctx.carte)
        propositions = points_exploration(ctx.carte, ctx.odom.pose, self._origine,
                                         list(self._visites),
                                         self.action.get("rayon_m", 8.0), marge, distance)
        accessibles, chemins = [], []
        for point in propositions:
            cible = (point["x"], point["y"])
            chemin = _planifier(ctx.navigation, ctx.carte, ctx.odom.pose, cible, marge, distance)
            if chemin is not None:
                accessibles.append({**point, "longueur_m": chemin.longueur_m})
                chemins.append(chemin)
        return accessibles, chemins

    def pas(self, ctx: Contexte) -> Pas:
        if self._terminal is not None:
            return self._terminal
        if self._ferme:
            return self._fin("exploration annulée", echec=True)
        if self._debut is None:
            self._debut, self._origine = ctx.t, ctx.odom.pose
            self._visites.append((ctx.odom.pose.x, ctx.odom.pose.y))
        if ctx.t - self._debut >= self.action.get("duree_s", 600):
            return self._fin("limite de durée atteinte : exploration partielle",
                             echec=bool(self.action.get("objet") or self.action.get("piece")))
        if ctx.carte is None or ctx.navigation is None:
            return self._fin("carte ou navigation indisponible", echec=True)
        marge = float(self.executeur.profil.valeur("geometrie.emprise_demi_largeur_m", 0.225)) + 0.1
        if self._pool is None:
            self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="exploration")
            self._navigation = _NavigationConnue(ctx.navigation, self._pool, marge,
                                                ctx.empattement, ctx.carte)
        assert self._navigation is not None
        self._navigation.carte = ctx.carte
        if self._sous is not None:
            monde = getattr(self.executeur, "monde", None)
            if (self._approche and self.action.get("objet")
                    and callable(getattr(monde, "resume", None))
                    and not any(o.get("id") == self._objet_id and o.get("statut") == "confirme"
                                for o in monde.rechercher(self.action["objet"]))):
                self._sous, self._approche, self._observer = None, False, True
                self._navigation.fermer()
                return Pas(detail="objet : repère à confirmer de nouveau")
            try:
                pas = self._sous.pas(replace(ctx, navigation=self._navigation))
            except Exception as erreur:
                return self._fin(f"navigation indisponible : {erreur}", echec=True)
            if pas.etat not in ("termine", "echec"):
                pas.detail = f"exploration, point {len(self._points)} : {pas.detail}"
                return pas
            self._sous = None
            self._navigation.fermer()
            if self._approche:
                return self._fin(f"« {self.action['objet']} » atteint" if pas.etat == "termine"
                                 else "objet repéré, approche impossible",
                                 echec=pas.etat == "echec")
            self._points[-1]["etat"] = "visite" if pas.etat == "termine" else "inaccessible"
            self._observer = pas.etat == "termine"
            self._essais_confirmation = 0
        if self._observer:
            if self._analyse is None:
                if ctx.t < self._confirmation_a:
                    return Pas(detail="exploration : attente d'une acquisition indépendante")
                self._pose_image = ctx.odom.pose
                self._analyse = self.executeur.lancer_ia("observer", {
                    "camera": self.action.get("camera", "gauche"),
                    "objet": self.action.get("objet"), "piece": self.action.get("piece")})
                self._analyse_debut = ctx.t
                if self._analyse is None:
                    return self._fin("vision indisponible pour explorer", echec=True)
            if not self._analyse.done():
                if ctx.t - self._analyse_debut > 45:
                    return self._fin("analyse vidéo expirée", echec=True)
                return Pas(detail="exploration : observation caméra et mise à jour de la carte")
            try:
                resultat = self._analyse.result() or {}
            except Exception as erreur:
                return self._fin(f"vision indisponible : {erreur}", echec=True)
            if not isinstance(resultat, dict):
                return self._fin("réponse vision invalide", echec=True)
            self._analyse, self._observer = None, False
            if self.action.get("piece"):
                # Reconsulter la mémoire validée plutôt que croire le texte de la vision.
                monde = getattr(self.executeur, "monde", None)
                pieces = monde.rechercher_piece(self.action["piece"]) if monde else []
                confirmees = [p for p in pieces if p.get("statut") == "confirmee"]
                if confirmees:
                    if len(confirmees) != 1:
                        return self._fin("plusieurs pièces correspondent : destination ambiguë",
                                         echec=True)
                    self.piece_trouvee = confirmees[0]
                    return self._fin(f"« {self.action['piece']} » identifiée par ses objets")
            if self.action.get("objet"):
                points = [p for p in resultat.get("points", []) if isinstance(p, dict)
                          and p.get("sol") is not None]
                monde = getattr(self.executeur, "monde", None)
                monde_mesure = callable(getattr(monde, "resume", None))
                confirmes = ([o for o in monde.rechercher(self.action["objet"])
                              if o.get("statut") == "confirme"] if monde_mesure else [])
                if points and monde_mesure and not confirmes:
                    self._essais_confirmation += 1
                    if self._essais_confirmation < 3:
                        self._observer = True
                        self._confirmation_a = ctx.t + 0.5
                        return Pas(detail="objet aperçu : seconde observation nécessaire")
                    points = []  # trois images sans preuve stable : continuer l'exploration
                for point in points:
                    try:
                        x, y = map(float, point["sol"])
                    except (TypeError, ValueError):
                        continue
                    if not math.isfinite(x) or not math.isfinite(y):
                        continue
                    assert self._pose_image is not None
                    try:
                        acquisition = point.get("pose_odom")
                        pose_image = (Pose2D(*map(float, acquisition)) if acquisition is not None
                                      else self._pose_image)
                        image_t = float(point.get("image_t", resultat.get(
                            "image_t", self._analyse_debut)))
                        if (not all(math.isfinite(v) for v in (
                                pose_image.x, pose_image.y, pose_image.cap, image_t))
                                or not -1 <= ctx.t - image_t <= 45):
                            continue
                    except (TypeError, ValueError):
                        continue
                    x, y = pose_image.vers_parent(x, y)
                    if monde_mesure:
                        correspondants = [o for o in confirmes
                                          if math.hypot(o["x"] - x, o["y"] - y) < 0.8]
                        if not correspondants:
                            continue
                        cible = min(correspondants,
                                    key=lambda o: math.hypot(o["x"] - x, o["y"] - y))
                        self._objet_id = cible.get("id")
                        x, y = cible["x"], cible["y"]
                    dx, dy = x - ctx.odom.pose.x, y - ctx.odom.pose.y
                    d = math.hypot(dx, dy)
                    if d < 0.7 or not self.action.get("approcher", True):
                        return self._fin(f"« {self.action['objet']} » repéré devant le rover")
                    x, y = x - dx * 0.7 / d, y - dy * 0.7 / d
                    if math.hypot(x - self._origine.x, y - self._origine.y) > self.action.get(
                            "rayon_m", 8.0):
                        return self._fin("objet repéré hors du rayon de mission", echec=True)
                    self._sous = AllerA({"type": "aller_a", "x": x, "y": y, "repere": "odom"})
                    self._approche = True
                    return Pas(detail=f"« {self.action['objet']} » repéré : approche")
        if len(self._points) >= self.action.get("points_max", 20):
            return self._fin("limite de points atteinte : exploration partielle",
                             echec=bool(self.action.get("objet") or self.action.get("piece")))
        if self._calcul is None and self._choix is None:
            self._calcul = self._pool.submit(self._choisir, ctx, marge)
            self._calcul_debut = ctx.t
        if self._calcul is not None:
            if not self._calcul.done():
                if ctx.t - self._calcul_debut > 15:
                    return self._fin("planification de l'exploration expirée", echec=True)
                return Pas(detail="exploration : recherche des points de vue accessibles")
            calcul, self._calcul = self._calcul, None
            try:
                self._prochains, self._chemins = calcul.result()
            except Exception as erreur:
                return self._fin(f"planification indisponible : {erreur}", echec=True)
            if not self._prochains:
                return self._fin("aucun nouveau point de vue accessible dans la zone connue",
                                 echec=bool(self.action.get("objet") or self.action.get("piece")))
            self._generation += 1
            self._choix = self.executeur.lancer_ia("choisir_exploration", {
                "candidats": [dict(p) for p in self._prochains],
                "visites": [{"x": x, "y": y} for x, y in self._visites],
                "objectif": self.action.get("objet") or (
                    f"identifier {self.action['piece']} grâce aux objets observés"
                    if self.action.get("piece") else "explorer la zone"),
                "generation": self._generation})
            self._choix_debut = ctx.t
            if self._choix is None:
                return self._fin("GX10 indisponible : aucun choix d'exploration", echec=True)
        assert self._choix is not None
        if not self._choix.done():
            if ctx.t - self._choix_debut > 30:
                return self._fin("choix du prochain point par GX10 expiré", echec=True)
            return Pas(detail="exploration : GX10 choisit le prochain point de vue")
        choix, self._choix = self._choix, None
        try:
            decision = choix.result()
        except Exception as erreur:
            return self._fin(f"choix GX10 indisponible : {erreur}", echec=True)
        indice = decision.get("index") if isinstance(decision, dict) else None
        if type(indice) is not int or not 0 <= indice < len(self._prochains):
            return self._fin("choix GX10 invalide : aucun point autorisé", echec=True)
        point, chemin = self._prochains[indice], self._chemins[indice]
        cible = (point["x"], point["y"])
        if not chemin_connu(ctx.carte, chemin, marge):
            return Pas(detail="exploration : carte modifiée, nouveau calcul des points de vue")
        self._visites.append(cible)
        self._points.append({**point, "etat": "en_cours", "raison": decision.get("raison", "")})
        self._sous = AllerA({"type": "aller_a", "x": cible[0], "y": cible[1], "repere": "odom"},
                            but=cible, chemin=chemin, _dernier_calcul=ctx.t)
        return Pas(detail=f"exploration : nouveau point {len(self._points)} choisi par GX10")
