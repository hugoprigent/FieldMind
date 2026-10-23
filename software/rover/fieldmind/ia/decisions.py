"""Choix du prochain point d'exploration sur GX10, parmi les candidats calculés sur le Pi.

Le modèle ne produit ni coordonnées exécutables, ni commandes moteurs. Le résultat reste
une proposition pour la primitive, qui revalide sa génération et son chemin localement.
"""

from __future__ import annotations

import asyncio
import json
import math
from typing import Any

from fieldmind.ia.fournisseurs import ErreurIA, extraire_json


def contexte_borne(etat: dict[str, Any]) -> dict[str, Any]:
    """Contexte spatial compact : les observations restent sur le rover entre deux appels."""
    monde = etat.get("monde") or {}
    if not isinstance(monde, dict):
        monde = {}
    monde = {k: (v[:40] if isinstance(v, list) else v) for k, v in monde.items()}
    return {k: etat.get(k) for k in (
        "pose_odom", "mode", "arme", "arret_urgence", "securite", "mission", "situation"
    )} | {"monde": monde}


def _identite(etat: dict[str, Any]) -> tuple[Any, ...]:
    mission = etat.get("mission") or {}
    return (mission.get("generation"), mission.get("plan_id"), etat.get("mode"),
            etat.get("arme"), etat.get("arret_urgence"), mission.get("etat"))


class ChoixExploration:
    def __init__(self, config: Any, outils: Any) -> None:
        self.config, self.outils = config, outils
        self._verrou = asyncio.Lock()

    @property
    def occupee(self) -> bool:
        return self._verrou.locked()

    async def choisir(self, argument: dict[str, Any]) -> dict[str, Any] | None:
        candidats = argument.get("candidats")
        if not isinstance(candidats, list) or not candidats or self._verrou.locked():
            return None
        # Contrat volontairement strict : candidats invalides refusés sans réindexation.
        if len(candidats) > 32 or any(not isinstance(c, dict) for c in candidats):
            return None
        for candidat in candidats:
            for cle in ("x", "y"):
                valeur = candidat.get(cle)
                if isinstance(valeur, bool) or not isinstance(valeur, (int, float)) or not (
                        math.isfinite(valeur)):
                    return None
        if not self.config.disponibilite("agent")[0]:
            return None
        async with self._verrou:
            etat = self.outils.etat_resume()
            if etat.get("arret_urgence") or etat.get("mode") != "auto" or not etat.get("arme"):
                return None
            identite = _identite(etat)
            profil = self.config.actif
            prompt = (
                "Tu choisis la prochaine étape d'un rover qui explore progressivement une pièce. "
                "La carte contient seulement ce qui a été observé. Le Raspberry a calculé les "
                "candidats accessibles ci-dessous. Choisis UN index (0 pour le premier), favorise "
                "les zones encore inconnues et les passages utiles à la mission, évite de "
                "revisiter sans raison. Ne crée aucune coordonnée et n'invente pas de pièce "
                "cachée. Si aucun candidat n'est pertinent, retourne index null. Réponds "
                'strictement en JSON {"index": 0, "raison": "raison courte en français"}.\n'
                + json.dumps({"contexte": contexte_borne(etat),
                              "mission": str(argument.get("objectif") or "explorer")[:500],
                              "visites": (argument.get("visites") or [])[-40:],
                              "candidats": candidats}, ensure_ascii=False, default=str)
            )
            try:
                async with asyncio.timeout(20.0):
                    reponse = await self.config.passerelle().chat(
                        "agent", [{"role": "user", "content": prompt}])
                donnees = extraire_json(reponse.texte)
            except (ErreurIA, TimeoutError, ValueError):
                return None
            if profil is not self.config.actif or identite != _identite(self.outils.etat_resume()):
                return None
            if not isinstance(donnees, dict):
                return None
            index = donnees.get("index")
            if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(
                    candidats):
                return None
            return {"index": index, "raison": str(donnees.get("raison") or "")[:240],
                    "generation_executeur": identite[0], "plan_id": identite[1]}
