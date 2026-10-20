"""Interprétation des consignes en langage naturel par un modèle de langage.

Fournisseurs :
- `anthropic` : API Claude (SDK officiel), sortie structurée selon le schéma de l'intention ;
- `openrouter` : modèles gratuits d'OpenRouter (API compatible OpenAI, prix plafonné à zéro) ;
- `openai` : toute API compatible OpenAI, en particulier vLLM servant un modèle local sur le GX10
  (même schéma, via `response_format`) ;
- `aucun` : pas de modèle ; les consignes écrites ne sont pas traitées (la carte reste utilisable).

Coût maîtrisé : jetons de sortie plafonnés, délai limite, quotas par session et par jour. Si le
modèle ne répond pas (saturé, quota, délai, refus), la consigne n'est pas exécutée et l'opérateur
en reçoit la raison : aucune interprétation de secours.
"""

from __future__ import annotations

import datetime as dt
import json
import logging
import time
from dataclasses import dataclass
from typing import Any, Literal, Protocol

import anthropic
from anthropic.types.beta import BetaOutputConfigParam
from pydantic import ValidationError

from services.mission_planner.api_openai import (
    ClientCompatibleOpenAI,
    EchecFournisseur,
    extraire_json,
)
from services.mission_planner.intentions import Intention, schema_json
from services.protocole.scenario import ScenarioPublic

journal = logging.getLogger("fieldmind.consignes")

MODELES_AVEC_REPLI_SERVEUR = ("claude-opus-5", "claude-fable-5", "claude-sonnet-5-5")
"""Modèles qui acceptent `fallbacks: "default"` (reprise automatique après un refus)."""


def consigne_systeme(scenario: ScenarioPublic) -> str:
    """Consigne donnée au modèle : noms des lieux (sans coordonnées), actions et règles."""
    lieux = "\n".join(
        f"- {lieu.id} : {lieu.nom}" + (f" (aussi : {', '.join(lieu.alias)})" if lieu.alias else "")
        for lieu in scenario.lieux
    )
    lignes = [
        "Tu traduis la consigne d'un opérateur en une intention JSON pour un rover de "
        "reconnaissance en forêt.",
        "Tu ne conduis pas le rover : un programme valide l'intention, puis la navigation "
        "calcule le chemin et évite seule les obstacles.",
        "",
        "Actions possibles :",
        "- aller_a : aller à un lieu nommé (champ lieu = identifiant ci-dessous) ou à un point "
        "x, y en mètres (x vers l'est, y vers le nord) s'il est donné explicitement ;",
        "- parcours : visiter plusieurs lieux dans l'ordre (champ lieux = liste d'identifiants, "
        "par ex. « va à la clairière puis au grand rocher ») ;",
        "- faire_le_tour : faire le tour de la carte, visiter tous les lieux puis revenir au "
        "départ (« fais le tour », « fais une ronde », « explore la zone ») ;",
        "- chercher : explorer la zone pour trouver quelque chose avec la caméra (cible = "
        "tronc_couche, branche, rocher, trou, eau, personne, animal ou vehicule) : « trouve un "
        "tronc couché », « explore la carte et cherche un rocher », « y a-t-il quelqu'un ? » ;",
        "- trajet : suivre une forme ou un dessin demandé (carré, cercle, triangle, zigzag, huit, "
        "smiley, lettre…) : points = liste ordonnée de [avant_m, gauche_m] relatifs à la position "
        "actuelle du rover (avant_m vers l'avant, gauche_m vers la gauche), de 4 à 40 points, "
        "figure de 4 à 20 m de large ; le rover roule d'un point au suivant sans lever de "
        "crayon, donc trace une seule ligne continue (pour un smiley : le contour, puis les "
        "yeux et la bouche reliés au plus court) ;",
        "- deplacement_relatif : se déplacer de distance_m mètres dans la direction angle_deg "
        "(0 = tout droit, 90 = à gauche, -90 = à droite, 180 = derrière) ;",
        "- s_arreter : s'arrêter ;",
        "- reprendre : reprendre la mission interrompue ;",
        "- revenir_depart : revenir au point de départ ;",
        "- observer : regarder avec la caméra et décrire ce qui est devant le rover (« que "
        "vois-tu ? », « décris la scène ») ;",
        "- vigilance_activer / vigilance_desactiver : activer ou couper l'analyse automatique "
        "de la caméra pendant les déplacements ;",
        "- inconnu : tout le reste (question, demande hors sujet, consigne dangereuse comme "
        "foncer dans un obstacle, lieu inconnu).",
        "",
        "Lieux connus :",
        lieux,
        "",
        "- chercher : nombre = combien d'objets différents trouver (1 par défaut ; « trouves-en "
        "3 autres » après une recherche : nombre 3, même cible) ;",
        "- inconnu : pour une question ou une remarque qui n'est pas une consigne, reponse = une "
        "phrase courte et polie en français, fondée seulement sur l'état du rover donné (ne "
        "jamais inventer ce que le rover a vu) ; null pour les autres actions.",
        "",
        "Les messages précédents de la conversation sont fournis : sers-t'en pour comprendre "
        "« encore », « autre », « en », « là-bas », etc.",
        "",
        "Réponds uniquement avec l'intention. Mets à null les champs qui ne servent pas à "
        "l'action. N'invente jamais de coordonnées de lieux ; les points d'un trajet sont "
        "relatifs au rover.",
    ]
    return "\n".join(lignes)


@dataclass(frozen=True, slots=True)
class Interpretation:
    intention: Intention | None
    """None : le modèle n'a pas répondu, la consigne n'est pas exécutée."""
    fournisseur: str
    duree_ms: float
    echec: str | None = None
    """Pourquoi il n'y a pas d'intention (modèle saturé, quota, délai, refus)."""


@dataclass(frozen=True, slots=True)
class ContexteChat:
    """Ce que le modèle doit savoir en plus de la consigne : état du rover et conversation."""

    etat: str = ""
    historique: tuple[tuple[Literal["user", "assistant"], str], ...] = ()


def messages_chat(systeme: str, texte: str, contexte: ContexteChat | None) -> list[dict[str, Any]]:
    if contexte is not None and contexte.etat:
        systeme += f"\n\nÉtat actuel du rover : {contexte.etat}"
    messages: list[dict[str, Any]] = [{"role": "system", "content": systeme}]
    for role, contenu in contexte.historique if contexte else ():
        messages.append({"role": role, "content": contenu})
    messages.append({"role": "user", "content": texte})
    return messages


class Fournisseur(Protocol):
    nom: str

    async def interpreter(self, texte: str, contexte: ContexteChat | None = None) -> Intention: ...


class FournisseurAnthropic:
    def __init__(
        self, scenario: ScenarioPublic, modele: str, cle: str, delai_s: float, jetons_max: int
    ) -> None:
        self.nom = f"anthropic · {modele}"
        self._modele = modele
        self._systeme = consigne_systeme(scenario)
        self._schema = schema_json(scenario)
        self._jetons_max = jetons_max
        self._client = anthropic.AsyncAnthropic(api_key=cle, timeout=delai_s, max_retries=1)

    async def interpreter(self, texte: str, contexte: ContexteChat | None = None) -> Intention:
        messages = messages_chat(self._systeme, texte, contexte)
        output_config: BetaOutputConfigParam = {
            "format": {"type": "json_schema", "schema": self._schema}
        }
        repli = self._modele.startswith(MODELES_AVEC_REPLI_SERVEUR)
        if repli:
            # Consigne courte et simple : effort bas ; reprise serveur si un classifieur refuse.
            output_config["effort"] = "low"
        try:
            reponse = await self._client.beta.messages.create(
                model=self._modele,
                max_tokens=self._jetons_max,
                system=messages[0]["content"],
                messages=messages[1:],  # type: ignore[arg-type]
                output_config=output_config,
                betas=["server-side-fallback-2026-07-01"] if repli else anthropic.omit,
                fallbacks="default" if repli else anthropic.omit,
            )
        except anthropic.RateLimitError as erreur:
            raise EchecFournisseur("quota de l'API atteint") from erreur
        except anthropic.APITimeoutError as erreur:
            raise EchecFournisseur("délai dépassé") from erreur
        except anthropic.APIStatusError as erreur:
            raise EchecFournisseur(f"erreur de l'API ({erreur.status_code})") from erreur
        except anthropic.APIConnectionError as erreur:
            raise EchecFournisseur("API injoignable") from erreur
        if reponse.stop_reason == "refusal":
            raise EchecFournisseur("consigne refusée par le modèle")
        if reponse.stop_reason == "max_tokens":
            raise EchecFournisseur("réponse tronquée")
        texte_json = next((b.text for b in reponse.content if b.type == "text"), "")
        try:
            return Intention.model_validate_json(texte_json)
        except ValidationError as erreur:
            raise EchecFournisseur("réponse non conforme au schéma") from erreur


class FournisseurCompatibleOpenAI:
    """API /v1/chat/completions compatible OpenAI : OpenRouter (gratuit), vLLM, llama.cpp…"""

    def __init__(
        self,
        scenario: ScenarioPublic,
        client: ClientCompatibleOpenAI,
        jetons_max: int,
    ) -> None:
        prefixe = "openrouter" if client.openrouter else "openai"
        self.nom = f"{prefixe} · {client.modele}"
        self._client = client
        self._systeme = consigne_systeme(scenario)
        self._schema = schema_json(scenario)
        if client.mode_json == "objet":
            self._systeme += (
                "\n\nRéponds par un unique objet JSON conforme à ce schéma, sans texte autour :\n"
                + json.dumps(self._schema, ensure_ascii=False)
            )
        self._jetons_max = jetons_max

    async def interpreter(self, texte: str, contexte: ContexteChat | None = None) -> Intention:
        reponse = await self._client.completer(
            messages_chat(self._systeme, texte, contexte),
            self._schema,
            "intention",
            self._jetons_max,
        )
        try:
            return Intention.model_validate(extraire_json(reponse.contenu))
        except (ValidationError, ValueError) as erreur:
            raise EchecFournisseur("réponse non conforme au schéma") from erreur


MOTS_MAX_DIRECT = 8


class QuotaJournalier:
    """Nombre d'appels au modèle par jour, pour tout le serveur (sessions comprises)."""

    def __init__(self, maximum: int) -> None:
        self.maximum = maximum
        self._jour = dt.date.today()
        self.utilises = 0

    def prendre(self) -> bool:
        aujourd_hui = dt.date.today()
        if aujourd_hui != self._jour:
            self._jour, self.utilises = aujourd_hui, 0
        if self.utilises >= self.maximum:
            return False
        self.utilises += 1
        return True


class InterpreteConsignes:
    def __init__(
        self, scenario: ScenarioPublic, fournisseur: Fournisseur | None, quota: QuotaJournalier
    ) -> None:
        self.scenario = scenario
        self.fournisseur = fournisseur
        self.quota = quota

    @property
    def nom(self) -> str:
        return self.fournisseur.nom if self.fournisseur else "aucun modèle"

    async def interpreter(self, texte: str, contexte: ContexteChat | None = None) -> Interpretation:
        debut = time.perf_counter()

        def duree() -> float:
            return (time.perf_counter() - debut) * 1000

        if self.fournisseur is None:
            return Interpretation(None, self.nom, duree(), "aucun modèle de langage configuré")
        if not self.quota.prendre():
            return Interpretation(None, self.nom, duree(), "quota journalier du modèle atteint")
        try:
            intention = await self.fournisseur.interpreter(texte, contexte)
        except EchecFournisseur as erreur:
            journal.warning("modèle de langage indisponible : %s", erreur)
            return Interpretation(None, self.fournisseur.nom, duree(), str(erreur))
        return Interpretation(intention, self.fournisseur.nom, duree())
