"""Rejoindre une région inférée depuis des preuves, jamais une coordonnée du modèle."""

from __future__ import annotations

import math
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from fieldmind.mission.actions import AllerA, Contexte, Pas
from fieldmind.mission.exploration import (
    Explorer,
    _distance_connue,
    _NavigationConnue,
    _planifier,
    chemin_connu,
)


def destination_piece(piece: dict[str, Any], ctx: Contexte, marge: float) -> Any:
    """Cherche un emplacement libre près des preuves, avec un trajet entièrement connu.

    Les objets occupent de la place : leur centre n'est pas un waypoint. La position
    d'observation seule peut être dans le couloir ; elle ne définit pas la pièce.
    """
    if ctx.carte is None or ctx.navigation is None or piece.get("statut") != "confirmee":
        return None
    preuves = piece.get("preuves") or []
    if len(preuves) < 2:
        return None
    points = np.array([[p["x"], p["y"]] for p in preuves], dtype=float)
    if not np.isfinite(points).all():
        return None
    centre = points.mean(axis=0)
    carte = ctx.carte
    distance = _distance_connue(carte)
    pas = max(1, round(0.35 / carte.resolution))
    lig, col = np.mgrid[0:distance.shape[0]:pas, 0:distance.shape[1]:pas]
    xy = np.column_stack((carte.origine_x + (col.ravel() + 0.5) * carte.resolution,
                          carte.origine_y + (lig.ravel() + 0.5) * carte.resolution))
    ecarts = np.linalg.norm(xy[:, None] - points[None], axis=2)
    dcentre = np.linalg.norm(xy - centre, axis=1)
    garde = ((distance[lig.ravel(), col.ravel()] >= marge) & (dcentre <= 1.6)
             & (ecarts.min(axis=1) >= 0.65) & (ecarts.max(axis=1) <= 3.0))
    xy, dcentre = xy[garde], dcentre[garde]
    scores = dcentre + 0.15 * np.linalg.norm(xy - (ctx.odom.pose.x, ctx.odom.pose.y), axis=1)
    # Un choix local borné ; tous les chemins sont validés contre l'inconnu.
    for i in np.argsort(scores, kind="stable")[:32]:
        cible = tuple(map(float, xy[i]))
        chemin = _planifier(ctx.navigation, carte, ctx.odom.pose, cible, marge, distance)
        if chemin is not None:
            return cible, chemin
    return None


@dataclass
class RejoindrePiece:
    action: dict[str, Any]
    executeur: Any
    _debut: float | None = None
    _generation: Any = None
    _phase: str = "identifier"
    _future: Future[Any] | None = None
    _attente: float = 0.0
    _sous: Any = None
    _piece: dict[str, Any] | None = None
    _pool: ThreadPoolExecutor | None = None
    _navigation: _NavigationConnue | None = None
    _terminal: Pas | None = None
    _ferme: bool = False

    def fermer(self) -> None:
        self._ferme = True
        if self._future is not None:
            self._future.cancel()
            self._future = None
        if self._sous is not None and hasattr(self._sous, "fermer"):
            self._sous.fermer()
        if self._navigation is not None:
            self._navigation.fermer()
        if self._pool is not None:
            self._pool.shutdown(wait=False, cancel_futures=True)
            self._pool = None

    def _fin(self, texte: str, echec: bool = True) -> Pas:
        self.fermer()
        self._terminal = Pas(etat="echec" if echec else "termine", detail=texte,
                             raison=texte if echec else None, progression=1.0)
        return self._terminal

    def _connues(self) -> list[dict[str, Any]]:
        return [p for p in self.executeur.monde.rechercher_piece(self.action["nom"])
                if p.get("statut") == "confirmee"]

    def pas(self, ctx: Contexte) -> Pas:
        if self._terminal is not None:
            return self._terminal
        if self._ferme:
            return self._fin("recherche de pièce annulée")
        monde = getattr(self.executeur, "monde", None)
        if monde is None:
            return self._fin("mémoire des observations indisponible")
        generation = monde.resume().get("generation")
        if self._debut is None:
            self._debut, self._generation = ctx.t, generation
        if generation != self._generation:
            return self._fin("carte réinitialisée : destination à identifier de nouveau")
        if ctx.t - self._debut > 600:
            return self._fin("pièce non atteinte dans la durée de recherche")
        nom = self.action["nom"]
        connues = self._connues()
        if len(connues) > 1:
            return self._fin(f"plusieurs « {nom} » observées : destination ambiguë")
        if self._phase == "identifier":
            if connues:
                if self._future is not None:
                    self._future.cancel()
                    self._future = None
                self._piece, self._phase = connues[0], "planifier"
            elif self._future is None:
                self._future = self.executeur.lancer_ia("identifier_piece", {"nom": nom})
                self._attente = ctx.t
                if self._future is None:
                    return self._fin("GX10 indisponible pour identifier la pièce")
                return Pas(detail=f"identifier « {nom} » à partir des objets observés")
            elif not self._future.done():
                if ctx.t - self._attente > 20:
                    return self._fin("identification de la pièce expirée")
                return Pas(detail=f"GX10 examine les indices de « {nom} »")
            else:
                try:
                    self._future.result()
                except Exception as erreur:
                    return self._fin(f"identification indisponible : {erreur}")
                self._future = None
                connues = self._connues()
                if len(connues) > 1:
                    return self._fin("destination ambiguë")
                if connues:
                    self._piece, self._phase = connues[0], "planifier"
                else:
                    self._phase = "explorer"
                    self._sous = Explorer({"type": "explorer", "piece": nom}, self.executeur)
        if self._phase == "explorer":
            resultat = self._sous.pas(ctx)
            if resultat.etat == "echec":
                return self._fin(f"« {nom} » non identifiée : {resultat.raison}")
            if resultat.etat != "termine":
                return resultat
            connues = self._connues()
            if len(connues) != 1:
                return self._fin(f"preuves insuffisantes pour « {nom} »")
            self._piece, self._sous, self._phase = connues[0], None, "planifier"
        if (self._phase in ("planifier", "trajet", "confirmer")
                and (not connues or self._piece is None or connues[0]["id"] != self._piece["id"])):
            return self._fin("les preuves de la destination ne sont plus valides")
        marge = float(self.executeur.profil.valeur("geometrie.emprise_demi_largeur_m", 0.225)) + 0.1
        if self._phase == "planifier":
            if ctx.carte is None or ctx.navigation is None:
                return self._fin("carte ou navigation indisponible")
            if self._pool is None:
                self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="piece")
                self._navigation = _NavigationConnue(ctx.navigation, self._pool, marge,
                                                    ctx.empattement, ctx.carte)
            if self._future is None:
                self._future = self._pool.submit(destination_piece, self._piece, ctx, marge)
                self._attente = ctx.t
            if not self._future.done():
                if ctx.t - self._attente > 15:
                    return self._fin("calcul d'accès à la pièce expiré")
                return Pas(detail=f"recherche d'un accès mesuré vers « {nom} »")
            try:
                destination = self._future.result()
            except Exception as erreur:
                return self._fin(f"accès à la pièce indisponible : {erreur}")
            self._future = None
            if destination is None:
                return self._fin(f"« {nom} » identifiée, aucun accès connu et libre")
            cible, chemin = destination
            if not chemin_connu(ctx.carte, chemin, marge):
                return Pas(detail="carte modifiée : nouveau calcul d'accès")
            self._sous = AllerA({"type": "aller_a", "x": cible[0], "y": cible[1],
                                  "repere": "odom"}, but=cible, chemin=chemin,
                                _dernier_calcul=ctx.t)
            self._phase = "trajet"
        if self._phase == "trajet":
            assert self._navigation is not None
            if ctx.carte is None:
                return self._fin("carte indisponible pendant le trajet")
            self._navigation.carte = ctx.carte
            resultat = self._sous.pas(replace(ctx, navigation=self._navigation))
            if resultat.etat == "echec":
                return self._fin(f"accès bloqué vers « {nom} » : {resultat.raison}")
            if resultat.etat != "termine":
                resultat.detail = f"« {nom} » : {resultat.detail}"
                return resultat
            self._sous, self._phase = None, "confirmer"
        if self._phase == "confirmer":
            if self._future is None:
                self._future = self.executeur.lancer_ia("observer", {"piece": nom})
                self._attente = ctx.t
                if self._future is None:
                    return self._fin("arrivée à confirmer, caméra indisponible")
            if not self._future.done():
                if ctx.t - self._attente > 45:
                    return self._fin("confirmation visuelle de l'arrivée expirée")
                return Pas(detail=f"confirmation visuelle de « {nom} » à l'arrivée")
            try:
                self._future.result()
            except Exception as erreur:
                return self._fin(f"arrivée non confirmée : {erreur}")
            self._future = None
            pieces = self._connues()
            preuves = pieces[0].get("preuves", []) if len(pieces) == 1 else []
            proches = [p for p in preuves if p.get("age_s", 3600) <= 45
                       and p.get("image_t", float("-inf")) >= self._attente - 0.1
                       and isinstance(p.get("pose_observation"), dict)
                       and math.hypot(p["pose_observation"]["x"] - ctx.odom.pose.x,
                                      p["pose_observation"]["y"] - ctx.odom.pose.y) <= 0.75]
            if len(proches) < 2:
                return self._fin(f"zone atteinte, mais « {nom} » non confirmée depuis ce point")
            labels = ", ".join(p["label"] for p in proches[:4])
            return self._fin(f"« {nom} » atteinte et confirmée : {labels}", echec=False)
        return Pas(detail=f"recherche de « {nom} »")
