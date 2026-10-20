"""Vérifie les points d'accès du profil d'IA actif (ou de --profil) : modèles, chat, appel de
fonctions, image, transcription.

    cd software/rover
    .venv/bin/python deploiement/gx10/verifier.py                 # profil actif (config/ia.toml)
    .venv/bin/python deploiement/gx10/verifier.py --profil gx10 --wav essai.wav

Chaque essai fait une vraie requête au fournisseur (avec Gemini : quelques requêtes facturées
ou décomptées du quota gratuit). Code de sortie 0 si tout passe.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import struct
import sys
import time
import wave
import zlib
from collections.abc import Awaitable, Callable
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # software/rover

import aiohttp

from fieldmind.ia.config import FICHIER_IA, ConfigIA
from fieldmind.ia.fournisseurs import ErreurIA, extraire_json, url_donnees
from fieldmind.ia.outils import ContexteValidation, schemas_fonctions, valider_plan
from fieldmind.ia.transcription import Transcripteur
from fieldmind.ia.vision import consigne_pointage, lire_points

CARRE_U, CARRE_V = 0.70, 0.40
"""Centre du carré rouge de l'image d'essai (normalisé)."""


def image_essai_png(largeur: int = 320, hauteur: int = 240) -> bytes:
    """PNG (bibliothèque standard) : fond gris clair, carré rouge centré en (0,70 ; 0,40)."""
    cote = hauteur // 5
    cx, cy = int(CARRE_U * largeur), int(CARRE_V * hauteur)
    lignes = []
    for y in range(hauteur):
        ligne = bytearray(b"\x00")
        for x in range(largeur):
            rouge = abs(x - cx) <= cote // 2 and abs(y - cy) <= cote // 2
            ligne += b"\xd0\x20\x20" if rouge else b"\xe8\xe8\xe8"
        lignes.append(bytes(ligne))

    def bloc(nom: bytes, donnees: bytes) -> bytes:
        return (struct.pack(">I", len(donnees)) + nom + donnees
                + struct.pack(">I", zlib.crc32(nom + donnees) & 0xFFFFFFFF))

    entete = struct.pack(">IIBBBBB", largeur, hauteur, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + bloc(b"IHDR", entete)
            + bloc(b"IDAT", zlib.compress(b"".join(lignes))) + bloc(b"IEND", b""))


def pcm_essai(duree_s: float = 1.0, frequence_hz: int = 16000) -> bytes:
    """Son de 440 Hz très bas : le serveur doit répondre (texte vide ou court), sans erreur."""
    n = int(duree_s * frequence_hz)
    return b"".join(struct.pack("<h", int(300 * math.sin(2 * math.pi * 440 * i / frequence_hz)))
                    for i in range(n))


def lire_wav(chemin: Path) -> tuple[bytes, int]:
    with wave.open(str(chemin), "rb") as w:
        if w.getnchannels() != 1 or w.getsampwidth() != 2:
            raise SystemExit("WAV attendu : mono, 16 bits")
        return w.readframes(w.getnframes()), w.getframerate()


async def verifier(config: ConfigIA, wav: Path | None = None,
                   ecrire: Callable[[str], None] = print) -> bool:
    profil = config.actif
    ecrire(f"Profil « {profil.nom} » — {profil.description}")
    disponible, raison = config.disponibilite("agent")
    if not disponible:
        ecrire(f"  ÉCHEC  disponibilité : {raison}")
        return False
    passerelle = config.passerelle()
    resultats: list[bool] = []

    async def essai(nom: str, fonction: Callable[[], Awaitable[str]]) -> None:
        debut = time.monotonic()
        try:
            detail = await fonction()
            ecrire(f"  OK     {nom} ({time.monotonic() - debut:.1f} s) {detail or ''}")
            resultats.append(True)
        except (ErreurIA, AssertionError, ValueError) as erreur:
            ecrire(f"  ÉCHEC  {nom} : {erreur}")
            resultats.append(False)

    async def modeles() -> str:
        acces = passerelle.acces("agent")
        entetes = {"Authorization": f"Bearer {cle}"} if (cle := config.cle()) else {}
        async with aiohttp.ClientSession() as session, session.get(
                f"{acces.role.url.rstrip('/')}/models", headers=entetes,
                timeout=aiohttp.ClientTimeout(total=15)) as reponse:
            if reponse.status >= 400:
                raise ErreurIA(f"/models : HTTP {reponse.status}")
            donnees = await reponse.json(content_type=None)
        servis = {m.get("id", "").removeprefix("models/") for m in donnees.get("data", [])}
        attendus = {r.modele for r in (profil.agent, profil.vision) if r is not None}
        manquants = attendus - servis
        if profil.api == "openai" and manquants:
            raise ErreurIA(f"modèles non servis : {', '.join(sorted(manquants))}")
        return f"{len(servis)} modèle(s)"

    async def chat() -> str:
        reponse = await passerelle.chat("agent", [
            {"role": "user", "content": "Réponds seulement par le mot OK."}])
        return f"« {reponse.texte[:40]} »"

    async def fonctions() -> str:
        reponse = await passerelle.chat("agent", [
            {"role": "system", "content": "Tu pilotes un rover : appelle les fonctions."},
            {"role": "user", "content": "Avance de 2 mètres puis tourne à gauche."}],
            outils=schemas_fonctions())
        plan = valider_plan(reponse.appels, ContexteValidation())
        types = [a["type"] for a in plan.actions]
        assert types[:2] == ["avancer", "tourner"], f"plan inattendu : {plan.actions}"
        return str(plan.actions)

    async def image() -> str:
        format_points = passerelle.acces("vision").role.format_points
        reponse = await passerelle.chat("vision", [{"role": "user", "content": [
            {"type": "image_url", "image_url": {"url": url_donnees(image_essai_png(),
                                                                   "image/png")}},
            {"type": "text", "text": consigne_pointage("carré rouge", format_points)}]}])
        points = lire_points(extraire_json(reponse.texte), format_points)
        assert points, "aucun point renvoyé"
        u, v, _ = points[0]
        # Le pointage est demandé au pied de l'objet : tolérance verticale plus large.
        assert abs(u - CARRE_U) < 0.12 and abs(v - CARRE_V) < 0.2, (
            f"point ({u:.2f}, {v:.2f}) loin du carré ({CARRE_U}, {CARRE_V})")
        return f"point ({u:.2f}, {v:.2f})"

    async def transcription() -> str:
        pcm, frequence = lire_wav(wav) if wav else (pcm_essai(), 16000)
        texte = await Transcripteur(config).transcrire(pcm, frequence)
        return f"« {texte[:60]} »"

    try:
        await essai("liste des modèles", modeles)
        await essai("chat", chat)
        await essai("appel de fonctions", fonctions)
        if profil.vision is not None:
            await essai("image (pointage)", image)
        if profil.transcription is not None:
            await essai(f"transcription ({profil.transcription.mode})", transcription)
    finally:
        await passerelle.fermer()
    ok = all(resultats)
    ecrire("Tout est bon." if ok else "Des essais ont échoué.")
    return ok


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--profil", help="profil à vérifier (défaut : profil actif)")
    parser.add_argument("--wav", type=Path, help="enregistrement WAV mono 16 bits à transcrire")
    parser.add_argument("--config", type=Path, default=FICHIER_IA, help="fichier ia.toml")
    args = parser.parse_args(argv)
    config = ConfigIA.charger(args.config)
    if args.profil:
        config.changer_profil(args.profil, enregistrer=False)
    return 0 if asyncio.run(verifier(config, args.wav)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
