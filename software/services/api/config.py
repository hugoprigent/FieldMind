"""Configuration du serveur, lue dans l'environnement et dans software/.env (jamais dans git)."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Literal

from pydantic import AliasChoices, Field, SecretStr
from pydantic_settings import BaseSettings, SettingsConfigDict

from services.mission_planner.api_openai import (
    MODELE_OPENROUTER_DEFAUT,
    MODELES_SECOURS_DEFAUT,
    URL_OPENROUTER,
)

RACINE_LOGICIEL = Path(__file__).resolve().parents[2]
"""Dossier software/ : racine de tous les chemins relatifs du projet."""


class Reglages(BaseSettings):
    """Réglages du serveur de démo. Chaque champ correspond à une variable FIELDMIND_*."""

    model_config = SettingsConfigDict(
        env_prefix="FIELDMIND_",
        env_file=RACINE_LOGICIEL / ".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )

    llm_provider: Literal["auto", "openrouter", "anthropic", "openai", "aucun"] = "auto"
    """auto : OpenRouter (modèles gratuits) si OPENROUTER_API_KEY est définie, sinon aucun modèle
    (les consignes écrites ne sont pas traitées)."""
    llm_model: str = ""
    """Vide : `stealth/space-bunny-alpha` avec OpenRouter."""
    llm_api_key: SecretStr = SecretStr("")
    llm_base_url: str = ""
    llm_modeles_secours: str = ",".join(MODELES_SECOURS_DEFAUT)
    """OpenRouter : modèles gratuits essayés si le premier est indisponible (séparés par des
    virgules ; vide pour aucun)."""
    openrouter_api_key: SecretStr = Field(
        default=SecretStr(""),
        validation_alias=AliasChoices("OPENROUTER_API_KEY", "FIELDMIND_OPENROUTER_API_KEY"),
    )
    openrouter_url: str = URL_OPENROUTER
    """Adresse de l'API OpenRouter (modifiable pour les essais avec un faux serveur)."""
    llm_delai_s: float = 20.0
    llm_jetons_max: int = 1024
    """Plafond de jetons de sortie par consigne. Les modèles récents réfléchissent avant de
    répondre (réflexion comprise dans ce plafond) : 300 jetons tronqueraient la réponse."""

    vision_provider: Literal["auto", "openrouter", "openai", "simulation", "aucun"] = "auto"
    """auto : même modèle que le chat s'il passe par OpenRouter ou vLLM, sinon le détecteur
    parfait de la simulation (sessions simulées seulement)."""
    vision_model: str = ""
    """Vide : le modèle du chat."""
    vision_modeles_secours: str = ""
    """Modèles de secours de la vision (séparés par des virgules) ; vide : ceux du chat."""
    vision_delai_s: float = 45.0
    vision_jetons_max: int = 4000
    vision_vigilance: bool = True
    """IA d'observation active au début de chaque session : pendant une mission, une image de
    la caméra analysée toutes les `vision_periode_s` (note, éléments repérés, décision validée
    par le cerveau). Elle s'ajoute à la stéréo, qui seule assure l'évitement des obstacles."""
    vision_periode_s: float = 30.0
    vision_max_par_session: int = 150
    vision_max_par_jour: int = 35
    """Le compte gratuit d'OpenRouter permet 50 requêtes par jour, chat compris."""

    jeton_robot: SecretStr = SecretStr("")
    """Jeton du pont du robot distant (/ws/robot). Vide : accès fermé."""

    sessions_max: int = 10
    chat_max_par_session: int = 30
    chat_max_par_jour: int = 500
    session_inactivite_s: float = 600.0

    scenario: Path = RACINE_LOGICIEL / "simulation" / "scenarios" / "foret_demo.json"
    fiche_rover: Path = RACINE_LOGICIEL / "config" / "rover_fieldmind.json"
    dossier_web: Path = RACINE_LOGICIEL / "interface" / "web"
    dossier_journaux: Path = RACINE_LOGICIEL / "logs"

    @property
    def cle_openrouter(self) -> str:
        cle = self.openrouter_api_key.get_secret_value()
        if not cle and self.llm_provider == "openrouter":
            cle = self.llm_api_key.get_secret_value()
        return cle

    @property
    def fournisseur_chat(self) -> Literal["openrouter", "anthropic", "openai", "aucun"]:
        """Fournisseur effectif du chat, une fois « auto » résolu et les paramètres vérifiés."""
        choix = self.llm_provider
        if choix == "auto":
            choix = "openrouter" if self.cle_openrouter else "aucun"
        if choix == "openrouter" and not self.cle_openrouter:
            return "aucun"
        if choix == "anthropic" and not (self.llm_model and self.llm_api_key.get_secret_value()):
            return "aucun"
        if choix == "openai" and not (self.llm_model and self.llm_base_url):
            return "aucun"
        return choix

    @property
    def chat_llm_actif(self) -> bool:
        return self.fournisseur_chat != "aucun"

    @property
    def modele_chat(self) -> str:
        if self.fournisseur_chat == "openrouter":
            return self.llm_model or MODELE_OPENROUTER_DEFAUT
        return self.llm_model

    @property
    def fournisseur_vision(self) -> Literal["openrouter", "openai", "simulation", "aucun"]:
        choix = self.vision_provider
        if choix == "auto":
            chat = self.fournisseur_chat
            choix = chat if chat in ("openrouter", "openai") else "simulation"
        if choix == "openrouter" and not self.cle_openrouter:
            return "simulation"
        if choix == "openai" and not (self.llm_base_url and (self.vision_model or self.llm_model)):
            return "simulation"
        return choix

    @property
    def modele_vision(self) -> str:
        if self.vision_model:
            return self.vision_model
        if self.fournisseur_vision == "openrouter":
            return (
                self.llm_model
                if self.fournisseur_chat == "openrouter" and self.llm_model
                else (MODELE_OPENROUTER_DEFAUT)
            )
        return self.llm_model

    @property
    def modeles_secours(self) -> tuple[str, ...]:
        return tuple(m.strip() for m in self.llm_modeles_secours.split(",") if m.strip())

    @property
    def modeles_secours_vision(self) -> tuple[str, ...]:
        """Secours de la vision : les siens s'ils sont donnés, sinon ceux du chat."""
        liste = self.vision_modeles_secours or self.llm_modeles_secours
        return tuple(m.strip() for m in liste.split(",") if m.strip() and m != self.modele_vision)


@lru_cache
def reglages() -> Reglages:
    """Réglages chargés une seule fois par processus."""
    return Reglages()
