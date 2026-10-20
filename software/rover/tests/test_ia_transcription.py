"""Transcription : WAV construit, Gemini natif (generateContent), chat audio, Whisper du GX10."""

from __future__ import annotations

import base64
import io
import wave
from typing import Any

import pytest
from test_ia_faux_serveur import Panne, config_essai, faux_serveur

from fieldmind.ia.fournisseurs import IAIndisponible
from fieldmind.ia.transcription import Transcripteur, pcm_vers_wav

PCM = (b"\x10\x00\xf0\xff" * 8000)  # 0,5 s à 16 kHz


def test_wav() -> None:
    wav = pcm_vers_wav(PCM + b"\x01", 16000)  # octet impair retiré
    with wave.open(io.BytesIO(wav)) as w:
        assert (w.getnchannels(), w.getsampwidth(), w.getframerate()) == (1, 2, 16000)
        assert w.readframes(w.getnframes()) == PCM


async def test_gemini_natif() -> None:
    async with faux_serveur() as fs:
        config = config_essai(fs.base)
        texte = await Transcripteur(config).transcrire(PCM, 16000)
        await config.passerelle().fermer()
    assert texte == "avance de 2 mètres"
    chemin, corps = fs.requetes[0]
    assert chemin == "/v1beta/models/gemini-3.5-transcribe:generateContent"
    assert corps["_entetes"]["x-goog-api-key"] == "cle-essai"
    partie = corps["contents"][0]["parts"][0]["inlineData"]
    assert partie["mimeType"] == "audio/wav"
    with wave.open(io.BytesIO(base64.b64decode(partie["data"]))) as w:
        assert w.getframerate() == 16000
    options = corps["generationConfig"]["audioTranscriptionConfig"]
    assert options["languageCodes"] == ["fr-FR"]
    assert options["mode"] == "SMART"
    assert "FieldMind" in options["customVocabulary"]


async def test_gemini_natif_options_refusees_puis_langue_seule() -> None:
    async with faux_serveur() as fs:
        def natif(modele: str, corps: dict[str, Any]) -> Any:
            if "mode" in corps["generationConfig"]["audioTranscriptionConfig"]:
                return Panne(400, {"error": {"message": "Invalid JSON payload: mode"}})
            return {"candidates": [{"content": {"parts": [{"audioTranscription": {
                "words": [{"word": "tourne"}, {"word": "à"}, {"word": "gauche"}]}}]}}]}

        fs.natif = natif
        config = config_essai(fs.base)
        transcripteur = Transcripteur(config)
        assert await transcripteur.transcrire(PCM) == "tourne à gauche"
        assert await transcripteur.transcrire(PCM) == "tourne à gauche"
        await config.passerelle().fermer()
    # la seconde fois, directement sans les options refusées
    assert len(fs.requetes) == 3
    assert fs.requetes[2][1]["generationConfig"]["audioTranscriptionConfig"] == {
        "languageCodes": ["fr-FR"]}


async def test_chat_audio() -> None:
    async with faux_serveur() as fs:
        def retouche(d: dict[str, Any]) -> None:
            d["profils"]["gemini"]["transcription"]["mode"] = "chat_audio"
            d["profils"]["gemini"]["transcription"]["modele"] = "gemini-3.8-flash"

        fs.chat = lambda corps: {"content": "« Recule de 1 mètre. »"}
        config = config_essai(fs.base, retouche=retouche)
        texte = await Transcripteur(config).transcrire(PCM)
        await config.passerelle().fermer()
    assert texte == "Recule de 1 mètre."
    corps = fs.requetes[0][1]
    assert corps["model"] == "gemini-3.8-flash"
    audio = corps["messages"][0]["content"][1]
    assert audio["type"] == "input_audio" and audio["input_audio"]["format"] == "wav"


async def test_chat_audio_silence() -> None:
    async with faux_serveur() as fs:
        def retouche(d: dict[str, Any]) -> None:
            d["profils"]["gemini"]["transcription"]["mode"] = "chat_audio"

        fs.chat = lambda corps: {"content": "[silence]"}
        config = config_essai(fs.base, retouche=retouche)
        assert await Transcripteur(config).transcrire(PCM) == ""
        await config.passerelle().fermer()


async def test_whisper_gx10() -> None:
    async with faux_serveur() as fs:
        config = config_essai(fs.base, env={"GX10_API_KEY": "secret"}, profil="gx10")
        texte = await Transcripteur(config).transcrire(PCM, 16000)
        await config.passerelle().fermer()
    assert texte == "avance de deux mètres"
    chemin, champs = fs.requetes[0]
    assert chemin == "/v1/audio/transcriptions"
    assert champs["model"] == "Qwen/Qwen3-ASR-1.7B"
    assert champs["language"] == "fr"
    assert champs["_wav"][:4] == b"RIFF"
    assert champs["_entetes"]["Authorization"] == "Bearer secret"
    assert "avance" not in champs["prompt"]  # amorce : vocabulaire seulement


async def test_whisper_recopie_son_amorce_sur_du_bruit() -> None:
    async with faux_serveur() as fs:
        config = config_essai(fs.base, env={}, profil="gx10")
        fs.texte_transcription = "FieldMind, rover, lidar, demi-tour, mètres, degrés."
        assert await Transcripteur(config).transcrire(PCM) == ""
        await config.passerelle().fermer()


async def test_sans_transcription() -> None:
    async with faux_serveur() as fs:
        config = config_essai(fs.base, profil="aucun")
        with pytest.raises(IAIndisponible, match="aucun modèle"):
            await Transcripteur(config).transcrire(PCM)
    assert fs.requetes == []
