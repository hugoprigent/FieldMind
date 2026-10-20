"""Transcription de la parole (PCM 16 bits mono) en texte français, selon le profil actif.

    transcripteur = Transcripteur(config_ia)
    texte = await transcripteur.transcrire(pcm, 16000)

Modes (`[profils.<nom>.transcription] mode`) :
- `gemini_natif` — Gemini 3.5 Transcribe : `models/<modele>:generateContent`, audio WAV en
  `inlineData`, `generationConfig.audioTranscriptionConfig` (`languageCodes`, `customVocabulary`,
  `mode` SMART). C'est l'appel documenté pour ce modèle (il n'a ni appel de fonctions ni
  réflexion) ; si les options sont refusées, nouvelle tentative avec la langue seule.
- `chat_audio` — chat/completions compatible OpenAI avec une partie `input_audio` (WAV base64) :
  tout modèle Gemini généraliste (ou Robotics ER 2, qui accepte l'audio).
- `openai_transcriptions` — POST /audio/transcriptions multipart (Whisper large-v3-turbo servi par
  vLLM sur le GX10).
"""

from __future__ import annotations

import base64
import io
import wave
from typing import Any

from fieldmind.ia.config import ConfigIA
from fieldmind.ia.fournisseurs import ErreurIA

MARQUE_SILENCE = "[silence]"


def pcm_vers_wav(pcm16_mono: bytes, frequence_hz: int) -> bytes:
    """Enveloppe WAV (RIFF, PCM 16 bits little-endian, mono)."""
    if len(pcm16_mono) % 2:
        pcm16_mono = pcm16_mono[:-1]
    tampon = io.BytesIO()
    with wave.open(tampon, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(int(frequence_hz))
        w.writeframes(pcm16_mono)
    return tampon.getvalue()


def consigne_chat_audio(langue: str, vocabulaire: tuple[str, ...]) -> str:
    texte = (
        f"Transcris exactement la parole de cet enregistrement (langue : {langue}). Écris les "
        "nombres en chiffres. Réponds uniquement par la transcription, sans guillemets ni "
        f"commentaire. S'il n'y a pas de parole intelligible, réponds exactement {MARQUE_SILENCE}."
    )
    if vocabulaire:
        texte += " Mots possibles : " + ", ".join(vocabulaire) + "."
    return texte


def nettoyer(texte: str) -> str:
    texte = " ".join(texte.split()).strip().strip("«»\"' ")
    if texte.lower() in (MARQUE_SILENCE, "silence", "(silence)", "[musique]", "..."):
        return ""
    return texte


class Transcripteur:
    def __init__(self, config_ia: ConfigIA) -> None:
        self.config_ia = config_ia

    async def transcrire(self, pcm16_mono: bytes, frequence_hz: int = 16000) -> str:
        """Texte de la parole ("" s'il n'y en a pas). Lève ErreurIA (IAIndisponible si le
        profil n'a pas de transcription)."""
        if not pcm16_mono:
            return ""
        wav = pcm_vers_wav(pcm16_mono, frequence_hz)
        return await self.config_ia.passerelle().avec_secours(
            "transcription", lambda acces: self._transcrire(acces, wav))

    async def _transcrire(self, acces: Any, wav: bytes) -> str:
        role = acces.role
        if role.mode == "gemini_natif":
            if acces.natif is None:
                raise ErreurIA("transcription Gemini native : `url_natif` absente du profil")
            texte = await acces.natif.transcrire(
                role.modele, wav, langue=role.langue, style=role.style,
                vocabulaire=role.vocabulaire, delai_s=role.delai_s, quota=acces.quota)
        elif role.mode == "chat_audio":
            messages = [{"role": "user", "content": [
                {"type": "text", "text": consigne_chat_audio(role.langue, role.vocabulaire)},
                {"type": "input_audio", "input_audio": {
                    "data": base64.b64encode(wav).decode("ascii"), "format": "wav"}},
            ]}]
            reponse = await acces.client.chat(
                role.modele, messages, reflexion=role.reflexion, jetons_max=role.jetons_max,
                temperature=role.temperature, delai_s=role.delai_s, quota=acces.quota)
            texte = reponse.texte
        else:
            # Amorce limitée au vocabulaire (noms propres) : Whisper recopie parfois son amorce
            # sur du bruit, elle ne doit donc jamais ressembler à une consigne.
            consigne = (", ".join(role.vocabulaire) + ".") if role.vocabulaire else None
            texte = await acces.client.transcrire(
                role.modele, wav, langue=role.langue, consigne=consigne,
                delai_s=role.delai_s, quota=acces.quota)
            if consigne and nettoyer(texte).rstrip(".").lower() == consigne.rstrip(".").lower():
                return ""
        return nettoyer(texte)
