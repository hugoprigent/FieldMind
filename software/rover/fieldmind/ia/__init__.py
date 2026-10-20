"""IA du rover : agent (consignes → actions), vision (décrire, pointer), transcription, écoute.

Fournisseurs interchangeables par profil (config/ia.toml) : `gemini` (en ligne), `gx10` (modèles
locaux servis par vLLM sur l'ASUS Ascent GX10, même API compatible OpenAI), `aucun`.
La sécurité ne dépend jamais de ce paquet : il ne fait que proposer des actions à la mission.

Branchement (voir le compte rendu d'intégration et chaque module) :

    config_ia = ConfigIA.charger()
    vision = Vision(bus, profil, config_ia, outils)          # outils : OutilsRover (mission)
    agent = Agent(bus, profil, config_ia, outils, vision=vision)
    voix = Voix(bus, profil, config_ia, Transcripteur(config_ia),
                lambda texte: agent.consigne(texte, origine="voix"))
"""

from fieldmind.ia.agent import Agent, ResultatConsigne, detecter_arret
from fieldmind.ia.config import ConfigIA, ErreurConfigIA, ProfilIA
from fieldmind.ia.fournisseurs import ErreurIA, IAIndisponible, Passerelle, QuotaAtteint
from fieldmind.ia.outils import ActionRefusee, OutilsRover, schemas_fonctions, valider_plan
from fieldmind.ia.transcription import Transcripteur, pcm_vers_wav
from fieldmind.ia.vision import Vision
from fieldmind.ia.voix import SUJET_NIVEAU, DetecteurParole, Voix

__all__ = [
    "SUJET_NIVEAU",
    "ActionRefusee",
    "Agent",
    "ConfigIA",
    "DetecteurParole",
    "ErreurConfigIA",
    "ErreurIA",
    "IAIndisponible",
    "OutilsRover",
    "Passerelle",
    "ProfilIA",
    "QuotaAtteint",
    "ResultatConsigne",
    "Transcripteur",
    "Vision",
    "Voix",
    "detecter_arret",
    "pcm_vers_wav",
    "schemas_fonctions",
    "valider_plan",
]
