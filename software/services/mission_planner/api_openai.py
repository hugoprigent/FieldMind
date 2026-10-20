"""Client des API compatibles OpenAI (/v1/chat/completions) : OpenRouter, vLLM, llama.cpp server.

Utilisé pour les consignes (texte) et la vision (texte + image).

OpenRouter : seuls les modèles gratuits sont permis. Chaque requête plafonne le prix à zéro
(`provider.max_price`), ce qui interdit à OpenRouter de router vers un fournisseur payant, et le
serveur vérifie au démarrage que les modèles configurés sont gratuits (`verifier_modeles`).

Sortie JSON : `json_schema` quand le modèle le permet (vLLM, modèles OpenRouter annonçant
`structured_outputs`), sinon `json_object` avec le schéma dans la consigne. La réponse est
toujours validée par le code.
"""

from __future__ import annotations

import json
import logging
import re
from dataclasses import dataclass
from typing import Any, Literal

import httpx

journal = logging.getLogger("fieldmind.api_openai")

URL_OPENROUTER = "https://openrouter.ai/api/v1"
MODELE_OPENROUTER_DEFAUT = "qwen/qwen3.8-27b:free"
"""Gratuit, accepte les images, répond en moins d'une seconde quand sa réflexion est coupée
(mesuré le 1er octobre 2026)."""
MODELES_SECOURS_DEFAUT = (
    "google/gemma-4-31b-it:free",
    "stealth/space-bunny-alpha",
    "openrouter/free",
)
"""Modèles gratuits acceptant les images, essayés dans l'ordre si le premier est saturé.
`openrouter/free` : routeur d'OpenRouter vers un modèle gratuit disponible (dernier recours)."""
REFLEXION_OBLIGATOIRE = frozenset({"stealth/space-bunny-alpha"})
"""Modèles qui refusent une requête sans réflexion (erreur 400) : réflexion courte pour eux."""

ModeJson = Literal["schema", "objet"]


class EchecFournisseur(Exception):
    """Le modèle n'a pas donné de réponse exploitable (délai, erreur, quota, format)."""

    def __init__(
        self, message: str, reessayer_dans_s: float | None = None, passager: bool = False
    ) -> None:
        super().__init__(message)
        self.reessayer_dans_s = reessayer_dans_s
        self.passager = passager
        """Saturation ou panne du fournisseur : un autre modèle peut répondre."""


@dataclass(frozen=True, slots=True)
class Reponse:
    contenu: str
    modele: str
    """Modèle qui a réellement répondu (OpenRouter peut basculer sur un modèle de secours)."""


def extraire_json(texte: str) -> Any:
    """Objet JSON d'une réponse de modèle, même entourée de texte ou d'un bloc ```json."""
    texte = texte.strip()
    bloc = re.search(r"```(?:json)?\s*(.*?)```", texte, re.DOTALL)
    if bloc:
        texte = bloc.group(1).strip()
    try:
        return json.loads(texte)
    except json.JSONDecodeError:
        debut, fin = texte.find("{"), texte.rfind("}")
        if debut < 0 or fin <= debut:
            raise
        return json.loads(texte[debut : fin + 1])


class ClientCompatibleOpenAI:
    def __init__(
        self,
        url: str,
        cle: str,
        modele: str,
        delai_s: float,
        openrouter: bool = False,
        modeles_secours: tuple[str, ...] = (),
        mode_json: ModeJson = "schema",
        transport: httpx.AsyncBaseTransport | None = None,
    ) -> None:
        self.modele = modele
        self.openrouter = openrouter
        self.modeles_secours = modeles_secours
        self.mode_json = mode_json
        self._url = url.rstrip("/") + "/chat/completions"
        self._entetes = {"Authorization": f"Bearer {cle}"} if cle else {}
        if openrouter:
            self._entetes |= {
                "HTTP-Referer": "https://github.com/hugoprigent/FieldMind",
                "X-Title": "FieldMind",
            }
        self._delai_s = delai_s
        self._transport = transport

    def _corps(
        self,
        messages: list[dict[str, Any]],
        schema: dict[str, Any],
        nom_schema: str,
        jetons_max: int,
        modele: str | None = None,
    ) -> dict[str, Any]:
        modele = modele or self.modele
        corps: dict[str, Any] = {
            "model": modele,
            "max_tokens": jetons_max,
            "temperature": 0,
            "messages": messages,
        }
        if self.mode_json == "schema":
            corps["response_format"] = {
                "type": "json_schema",
                "json_schema": {"name": nom_schema, "strict": True, "schema": schema},
            }
        else:
            corps["response_format"] = {"type": "json_object"}
        if self.openrouter:
            # Gratuit seulement : aucun fournisseur payant ne peut servir la requête.
            corps["provider"] = {"max_price": {"prompt": 0, "completion": 0, "image": 0}}
            # Tâche simple : sans réflexion (latence), sauf pour les modèles qui l'imposent.
            effort = "low" if modele in REFLEXION_OBLIGATOIRE else "none"
            corps["reasoning"] = {"effort": effort, "exclude": True}
        else:
            # Gemini (API compatible) : la réflexion consomme le plafond de jetons de la réponse,
            # qui serait coupée. Réflexion courte ; retirée si le modèle refuse le paramètre.
            corps["reasoning_effort"] = "low"
        return corps

    async def completer(
        self,
        messages: list[dict[str, Any]],
        schema: dict[str, Any],
        nom_schema: str,
        jetons_max: int,
    ) -> Reponse:
        """Essaie le modèle principal puis, s'il est saturé ou en panne, chaque modèle de
        secours (les modèles gratuits sont des ressources partagées, souvent saturées)."""
        modeles = (self.modele, *self.modeles_secours)
        for i, modele in enumerate(modeles):
            corps = self._corps(messages, schema, nom_schema, jetons_max, modele)
            try:
                try:
                    return await self._requete(corps, modele)
                except EchecFournisseur as erreur:
                    if "reasoning_effort" not in corps or "(400)" not in str(erreur):
                        raise
                    del corps["reasoning_effort"]
                    return await self._requete(corps, modele)
            except EchecFournisseur as erreur:
                if not erreur.passager or i == len(modeles) - 1:
                    raise
                journal.info("modèle %s indisponible (%s), modèle suivant", modele, erreur)
        raise EchecFournisseur("aucun modèle")

    async def _requete(self, corps: dict[str, Any], modele: str) -> Reponse:
        try:
            async with httpx.AsyncClient(timeout=self._delai_s, transport=self._transport) as c:
                reponse = await c.post(self._url, json=corps, headers=self._entetes)
        except httpx.TimeoutException as erreur:
            raise EchecFournisseur("délai dépassé", passager=True) from erreur
        except httpx.HTTPError as erreur:
            raise EchecFournisseur("API injoignable") from erreur
        if reponse.status_code == 429:
            attente = reponse.headers.get("retry-after")
            raise EchecFournisseur(
                "limite de requêtes du fournisseur atteinte",
                float(attente) if attente and attente.isdigit() else 60.0,
                passager=True,
            )
        if reponse.status_code in (401, 403):
            raise EchecFournisseur("clé d'API refusée")
        if reponse.status_code == 402:
            raise EchecFournisseur("modèle payant ou crédit requis : refusé (gratuit seulement)")
        if reponse.status_code >= 400:
            raise EchecFournisseur(
                f"erreur de l'API ({reponse.status_code})", passager=reponse.status_code >= 500
            )
        try:
            donnees = reponse.json()
            if "error" in donnees:
                code = donnees["error"].get("code", "?")
                raise EchecFournisseur(f"erreur du fournisseur ({code})", passager=True)
            choix = donnees["choices"][0]
            contenu = choix["message"].get("content") or ""
        except (KeyError, IndexError, TypeError, ValueError) as erreur:
            raise EchecFournisseur("réponse illisible") from erreur
        if not contenu.strip():
            fin = choix.get("finish_reason")
            raise EchecFournisseur(
                "réponse tronquée" if fin == "length" else "réponse vide du modèle"
            )
        return Reponse(contenu, str(donnees.get("model") or modele))


@dataclass(frozen=True, slots=True)
class InfosModele:
    gratuit: bool
    images: bool
    schema_json: bool


def infos_modeles_openrouter(
    modeles: list[str], url: str = URL_OPENROUTER, delai_s: float = 10.0
) -> dict[str, InfosModele] | None:
    """Tarifs et capacités des modèles, d'après le catalogue public d'OpenRouter (sans clé).

    None si le catalogue est injoignable : le plafond de prix à zéro de chaque requête reste la
    garantie de gratuité.
    """
    try:
        reponse = httpx.get(url.rstrip("/") + "/models", timeout=delai_s)
        reponse.raise_for_status()
        catalogue = {m["id"]: m for m in reponse.json()["data"]}
    except (httpx.HTTPError, KeyError, ValueError) as erreur:
        journal.warning("catalogue OpenRouter injoignable (%s)", erreur)
        return None
    infos: dict[str, InfosModele] = {}
    for nom in modeles:
        m = catalogue.get(nom)
        if m is None:
            continue
        prix = m.get("pricing", {})
        gratuit = all(
            float(prix.get(k, "0") or 0) == 0 for k in ("prompt", "completion", "image", "request")
        )
        parametres = m.get("supported_parameters", [])
        infos[nom] = InfosModele(
            gratuit=gratuit,
            images="image" in m.get("architecture", {}).get("input_modalities", []),
            schema_json="structured_outputs" in parametres,
        )
    return infos
