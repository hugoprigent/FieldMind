"""Faux serveur compatible OpenAI (et generateContent de Gemini), aides des tests de l'IA, et tests
du client HTTP (reprises, erreurs, réflexion, quotas).

Aucun appel réel : tout passe par un serveur aiohttp local sur 127.0.0.1.
Les autres fichiers `test_ia_*.py` importent ce module (`from test_ia_faux_serveur import …`).
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import tomllib
from collections.abc import AsyncIterator, Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from aiohttp import web

from fieldmind.bus import Bus, S
from fieldmind.ia.config import FICHIER_IA, ConfigIA
from fieldmind.ia.fournisseurs import (
    ClientOpenAI,
    ErreurIA,
    Quota,
    QuotaAtteint,
    extraire_json,
    lire_transcription_gemini,
)
from fieldmind.ia.outils import ActionRefusee
from fieldmind.types import Action, ImageCamera, Pose2D

FS = 16000

# ------------------------------------------------------------------ faux serveur


@dataclass
class Panne:
    statut: int
    corps: Any = None
    entetes: dict[str, str] = field(default_factory=dict)


def appels(*fonctions: tuple[str, dict[str, Any]], texte: str = "") -> dict[str, Any]:
    """Message assistant avec des appels de fonctions (arguments en JSON, comme OpenAI)."""
    return {"content": texte or None, "tool_calls": [
        {"id": f"appel_{i}", "type": "function",
         "function": {"name": nom, "arguments": json.dumps(args)}}
        for i, (nom, args) in enumerate(fonctions)]}


class FauxServeur:
    """Routes : /v1/chat/completions, /v1beta/openai/chat/completions, /v1/audio/transcriptions,
    /v1beta/models/{modele}:generateContent, /v1/models."""

    def __init__(self) -> None:
        self.requetes: list[tuple[str, Any]] = []
        self.chat: Callable[[dict[str, Any]], Any] = lambda corps: {"content": "OK"}
        self.natif: Callable[[str, dict[str, Any]], Any] = lambda modele, corps: {
            "candidates": [{"content": {"parts": [{"text": "avance de 2 mètres"}],
                                        "role": "model"}, "finishReason": "STOP"}]}
        self.texte_transcription = "avance de deux mètres"
        self.modeles = ["nvidia/Qwen3.6-35B-A3B-NVFP4"]
        self.pannes: list[Panne] = []
        self.delai_s = 0.0
        self.base = ""
        self._runner: web.AppRunner | None = None

    def chemins(self, prefixe: str) -> list[str]:
        return [c for c, _ in self.requetes if c.startswith(prefixe) or prefixe in c]

    async def demarrer(self) -> str:
        app = web.Application(client_max_size=32 * 1024 * 1024)
        app.router.add_post("/v1/chat/completions", self._chat)
        app.router.add_post("/v1beta/openai/chat/completions", self._chat)
        app.router.add_post("/v1/audio/transcriptions", self._transcription)
        app.router.add_post("/v1beta/models/{modele}", self._natif)
        app.router.add_get("/v1/models", self._liste)
        app.router.add_get("/v1beta/openai/models", self._liste)
        self._runner = web.AppRunner(app)
        await self._runner.setup()
        site = web.TCPSite(self._runner, "127.0.0.1", 0)
        await site.start()
        hote, port = self._runner.addresses[0][:2]
        self.base = f"http://{hote}:{port}"
        return self.base

    async def arreter(self) -> None:
        if self._runner is not None:
            await self._runner.cleanup()

    async def _avant(self, chemin: str, corps: Any) -> web.Response | None:
        self.requetes.append((chemin, corps))
        if self.delai_s:
            await asyncio.sleep(self.delai_s)
        if self.pannes:
            panne = self.pannes.pop(0)
            return web.json_response(panne.corps or {"error": {"message": "panne simulée"}},
                                     status=panne.statut, headers=panne.entetes)
        return None

    @staticmethod
    def _reponse(resultat: Any, modele: str) -> web.Response:
        if isinstance(resultat, Panne):
            return web.json_response(resultat.corps or {"error": {"message": "refus"}},
                                     status=resultat.statut, headers=resultat.entetes)
        message = {"role": "assistant", **resultat}
        fin = "tool_calls" if message.get("tool_calls") else "stop"
        return web.json_response({"id": "essai", "object": "chat.completion", "model": modele,
                                  "choices": [{"index": 0, "message": message,
                                               "finish_reason": fin}]})

    async def _chat(self, requete: web.Request) -> web.Response:
        corps = await requete.json()
        corps["_entetes"] = dict(requete.headers)
        if (refus := await self._avant(requete.path, corps)) is not None:
            return refus
        return self._reponse(self.chat(corps), corps.get("model", ""))

    async def _transcription(self, requete: web.Request) -> web.Response:
        formulaire = await requete.post()
        fichier = formulaire["file"]
        champs = {k: v for k, v in formulaire.items() if k != "file"}
        champs["_wav"] = fichier.file.read()  # type: ignore[union-attr]
        champs["_entetes"] = dict(requete.headers)
        if (refus := await self._avant(requete.path, champs)) is not None:
            return refus
        return web.json_response({"text": self.texte_transcription})

    async def _natif(self, requete: web.Request) -> web.Response:
        corps = await requete.json()
        corps["_entetes"] = dict(requete.headers)
        modele = requete.match_info["modele"].removesuffix(":generateContent")
        if (refus := await self._avant(requete.path, corps)) is not None:
            return refus
        resultat = self.natif(modele, corps)
        if isinstance(resultat, Panne):
            return web.json_response(resultat.corps or {"error": {"message": "refus"}},
                                     status=resultat.statut)
        return web.json_response(resultat)

    async def _liste(self, requete: web.Request) -> web.Response:
        return web.json_response({"object": "list", "data": [{"id": m} for m in self.modeles]})


@contextlib.asynccontextmanager
async def faux_serveur() -> AsyncIterator[FauxServeur]:
    serveur = FauxServeur()
    await serveur.demarrer()
    try:
        yield serveur
    finally:
        await serveur.arreter()


def donnees_ia(base: str) -> dict[str, Any]:
    """config/ia.toml du dépôt, adresses remplacées par celles du faux serveur."""
    with open(FICHIER_IA, "rb") as f:
        donnees = tomllib.load(f)
    gemini = donnees["profils"]["gemini"]
    gemini["url"] = f"{base}/v1beta/openai/"
    gemini["url_natif"] = f"{base}/v1beta/"
    gx10 = donnees["profils"]["gx10"]
    gx10["url"] = f"{base}/v1"
    for role in gx10.values():  # rôles à adresse propre (pointage, outils du GX10…)
        if isinstance(role, dict) and "url" in role:
            role["url"] = f"{base}/v1"
    return donnees


def config_essai(base: str, env: dict[str, str] | None = None, profil: str | None = None,
                 fichier_choix: Path | None = None,
                 retouche: Callable[[dict[str, Any]], None] | None = None) -> ConfigIA:
    donnees = donnees_ia(base)
    if retouche is not None:
        retouche(donnees)
    config = ConfigIA.depuis_dict(donnees, {"GEMINI_API_KEY": "cle-essai"} if env is None
                                  else env, fichier_choix)
    if profil:
        config.changer_profil(profil, enregistrer=False)
    return config


# ------------------------------------------------------------------ faux rover


class FauxOutils:
    """OutilsRover de test : enregistre ce que l'IA demande."""

    def __init__(self, arme: bool = True) -> None:
        self.arme = arme
        self.plans: list[tuple[list[Action], bool]] = []
        self.arrets: list[str] = []
        self.annulations = 0
        self.lieux: list[dict[str, Any]] = [{"nom": "garage", "gps": True},
                                            {"nom": "porte", "gps": False}]
        self.gps = {"aligne": True, "fix": "autonome"}
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.images: dict[str, ImageCamera] = {
            "gauche": ImageCamera(t=1.0, camera="gauche", jpeg=b"\xff\xd8 faux jpeg \xff\xd9",
                                  largeur=640, hauteur=360)}
        self.sol: tuple[float, float] | None = (2.1, -0.6)
        self.projections: list[tuple[str, float, float]] = []

    def etat_resume(self) -> dict[str, Any]:
        return {"pose": {"x": self.pose.x, "y": self.pose.y, "cap_deg": 0.0},
                "vitesse_m_s": 0.0, "mode": "auto", "arme": self.arme, "securite": [],
                "mission": {"etat": "attente"}, "capteurs": {"lidar": True, "gauche": True},
                "lieux": self.lieux, "gps": self.gps}

    def ajouter_actions(self, actions: list[Action], remplacer: bool) -> str:
        if not self.arme:
            raise ActionRefusee("rover non armé")
        self.plans.append((actions, remplacer))
        return f"p{len(self.plans)}"

    def annuler(self) -> None:
        self.annulations += 1

    def arret_immediat(self, raison: str) -> None:
        self.arrets.append(raison)

    def derniere_image(self, camera: str) -> ImageCamera | None:
        return self.images.get(camera)

    def pixel_vers_sol(self, camera: str, u: float, v: float) -> tuple[float, float] | None:
        self.projections.append((camera, u, v))
        return self.sol

    def pose_odom(self) -> Pose2D:
        return self.pose


class Ecoute:
    """Messages publiés sur le bus par l'IA."""

    def __init__(self, bus: Bus) -> None:
        self.chat: list[dict[str, Any]] = []
        self.voix: list[dict[str, Any]] = []
        self.vision: list[dict[str, Any]] = []
        bus.abonner(S.CHAT, self.chat.append)
        bus.abonner(S.VOIX, self.voix.append)
        bus.abonner(S.VISION, self.vision.append)

    def textes(self, role: str = "rover") -> list[str]:
        return [m["texte"] for m in self.chat if m["role"] == role]

    def evenements(self) -> list[str]:
        return [m["evenement"] for m in self.voix]


# ------------------------------------------------------------------ signaux synthétiques


def bruit_moteur(duree: float, niveau_db: float = -32.0, graine: int = 0, f_rot: float = 90.0,
                 depart: float | None = None, gain_depart_db: float = 14.0,
                 balayage: tuple[float, float, float] | None = None) -> np.ndarray:
    """Bruit de moteur : souffle large bande, grondement harmonique, sifflement à 2,2 kHz ;
    `depart` : le moteur démarre (rampe de 0,3 s, +gain_depart_db) ; `balayage` : (t0, t1,
    facteur) accélération (fréquences multipliées par facteur entre t0 et t1)."""
    rng = np.random.default_rng(graine)
    n = int(duree * FS)
    t = np.arange(n) / FS
    blanc = rng.standard_normal(n)
    souffle = 0.6 * np.convolve(blanc, np.ones(8) / 8, mode="same") + 0.4 * blanc
    f = np.full(n, f_rot)
    f_siff = np.full(n, 2200.0)
    if balayage is not None:
        t0, t1, facteur = balayage
        rampe = np.clip((t - t0) / (t1 - t0), 0, 1)
        f = f_rot * (1 + (facteur - 1) * rampe)
        f_siff = 2200.0 * (1 + (facteur - 1) * rampe)
    phase = 2 * np.pi * np.cumsum(f) / FS
    harmoniques = sum(np.sin(k * phase) / k for k in range(1, 12))
    x = souffle + 0.8 * harmoniques + 0.5 * np.sin(2 * np.pi * np.cumsum(f_siff) / FS)
    x = x / np.sqrt(np.mean(x ** 2))
    gain = np.full(n, 10 ** (niveau_db / 20))
    if depart is not None:
        gain = gain * 10 ** (gain_depart_db * np.clip((t - depart) / 0.3, 0, 1) / 20)
    return x * gain


def parole(duree: float = 1.4, niveau_db: float = -26.0, graine: int = 1) -> np.ndarray:
    """Parole simulée : syllabes voisées (fondamentale 110–170 Hz, harmoniques façonnées par des
    formants) de 180 ms environ, séparées de consonnes et de courtes pauses."""
    rng = np.random.default_rng(graine)
    n = int(duree * FS)
    sortie = np.zeros(n)
    formants = [(600, 1200, 2500), (400, 1800, 2600), (700, 1100, 2400), (300, 2200, 2900)]
    i = 0
    while i < n:
        longueur = min(int(rng.uniform(0.7, 1.3) * 0.18 * FS), n - i)
        t = np.arange(longueur) / FS
        f0 = rng.uniform(110, 170) * (1 + 0.1 * np.sin(2 * np.pi * 3 * t))
        phase = 2 * np.pi * np.cumsum(f0) / FS
        choix = formants[rng.integers(len(formants))]
        syllabe = np.zeros(longueur)
        for k in range(1, 30):
            fk = k * f0.mean()
            if fk > 4000:
                break
            syllabe += (sum(1 / (1 + ((fk - fi) / 120) ** 2) for fi in choix) + 0.05) * np.sin(
                k * phase)
        enveloppe = np.sin(np.pi * np.arange(longueur) / max(longueur, 1)) ** 0.6
        sortie[i : i + longueur] = syllabe * enveloppe * rng.uniform(0.6, 1.0)
        i += longueur
        pause = min(int(rng.uniform(0.6, 1.4) * 0.07 * FS), n - i)
        if pause > 0 and rng.random() < 0.5:
            sortie[i : i + pause] = 0.15 * rng.standard_normal(pause)
        i += max(pause, 0)
    actifs = sortie[sortie != 0]
    return sortie / np.sqrt(np.mean(actifs ** 2) + 1e-12) * 10 ** (niveau_db / 20)


def melanger(fond: np.ndarray, voix: np.ndarray | None = None, debut: float = 0.0
             ) -> np.ndarray:
    x = fond.copy()
    if voix is not None:
        i = int(debut * FS)
        x[i : i + voix.size] += voix[: x.size - i]
    return np.clip(x * 32768, -32768, 32767).astype(np.int16)


def blocs(signal: np.ndarray, taille: int = 320) -> list[bytes]:
    """Blocs de 20 ms (320 échantillons à 16 kHz), comme le pilote du micro."""
    return [signal[i : i + taille].tobytes() for i in range(0, signal.size, taille)]


# ------------------------------------------------------------------ tests du client HTTP


async def test_chat_gemini_envoie_reflexion_et_cle() -> None:
    async with faux_serveur() as fs:
        config = config_essai(fs.base)
        reponse = await config.passerelle().chat("agent", [{"role": "user", "content": "x"}])
        await config.passerelle().fermer()
    assert reponse.texte == "OK"
    chemin, corps = fs.requetes[0]
    assert chemin == "/v1beta/openai/chat/completions"
    assert corps["model"] == "gemini-4-argon"
    assert corps["reasoning_effort"] == "low"
    assert corps["_entetes"]["Authorization"] == "Bearer cle-essai"
    assert "temperature" not in corps  # Gemini : température par défaut du modèle


async def test_chat_gx10_coupe_la_reflexion_par_le_gabarit() -> None:
    async with faux_serveur() as fs:
        config = config_essai(fs.base, env={}, profil="gx10")
        await config.passerelle().chat("agent", [{"role": "user", "content": "x"}])
        await config.passerelle().fermer()
    _, corps = fs.requetes[0]
    assert corps["model"] == "nvidia/Qwen3.6-35B-A3B-NVFP4"
    assert corps["chat_template_kwargs"] == {"enable_thinking": False}
    assert "reasoning_effort" not in corps
    assert "Authorization" not in corps["_entetes"]  # clé facultative absente


async def test_reprise_sur_erreur_passagere() -> None:
    async with faux_serveur() as fs:
        fs.pannes = [Panne(503)]
        config = config_essai(fs.base)
        reponse = await config.passerelle().chat("agent", [{"role": "user", "content": "x"}])
        await config.passerelle().fermer()
    assert reponse.texte == "OK"
    assert len(fs.requetes) == 2


async def test_erreurs_lisibles_sans_reprise() -> None:
    async with faux_serveur() as fs:
        config = config_essai(fs.base)
        fs.pannes = [Panne(401, {"error": {"message": "API key not valid"}})]
        with pytest.raises(ErreurIA, match="clé d'API refusée"):
            await config.passerelle().chat("agent", [{"role": "user", "content": "x"}])
        assert len(fs.requetes) == 1
        # 429 avec une longue attente : pas de reprise, message clair
        fs.pannes = [Panne(429, [{"error": {"message": "quota"}}], {"Retry-After": "120"})]
        with pytest.raises(ErreurIA, match="limite de requêtes") as erreur:
            await config.passerelle().chat("agent", [{"role": "user", "content": "x"}])
        assert erreur.value.reessayer_dans_s == 120
        assert len(fs.requetes) == 2
        # 429 avec une attente courte (détails Gemini) : une reprise, puis succès
        fs.pannes = [Panne(429, [{"error": {"message": "quota", "details": [
            {"@type": "type.googleapis.com/google.rpc.RetryInfo", "retryDelay": "0s"}]}}])]
        assert (await config.passerelle().chat("agent", [{"role": "user", "content": "x"}])
                ).texte == "OK"
        assert len(fs.requetes) == 4
        fs.pannes = [Panne(404, {"error": {"message": "models/x is not found"}})]
        with pytest.raises(ErreurIA, match="introuvable"):
            await config.passerelle().chat("agent", [{"role": "user", "content": "x"}])
        await config.passerelle().fermer()


async def test_delai_depasse_sans_reprise() -> None:
    async with faux_serveur() as fs:
        fs.delai_s = 0.5
        client = ClientOpenAI(f"{fs.base}/v1", None, reprises=2)
        with pytest.raises(ErreurIA, match="délai dépassé"):
            await client.chat("m", [{"role": "user", "content": "x"}], delai_s=0.1)
        await client.fermer()
        await asyncio.sleep(0.5)
    assert len(fs.requetes) == 1


async def test_reflexion_refusee_reessayee_sans() -> None:
    async with faux_serveur() as fs:
        def chat(corps: dict[str, Any]) -> Any:
            if "reasoning_effort" in corps:
                return Panne(400, {"error": {"message": "reasoning_effort not supported"}})
            return {"content": "sans réflexion"}

        fs.chat = chat
        config = config_essai(fs.base)
        reponse = await config.passerelle().chat("agent", [{"role": "user", "content": "x"}])
        await config.passerelle().fermer()
    assert reponse.texte == "sans réflexion"
    assert len(fs.requetes) == 2


async def test_serveur_injoignable() -> None:
    client = ClientOpenAI("http://127.0.0.1:9/v1", None, reprises=0)
    with pytest.raises(ErreurIA, match="injoignable"):
        await client.chat("m", [{"role": "user", "content": "x"}], delai_s=2)
    await client.fermer()


async def test_quota_du_role_bloque_avant_la_requete() -> None:
    async with faux_serveur() as fs:
        def retouche(d: dict[str, Any]) -> None:
            d["profils"]["gemini"]["agent"]["requetes_par_heure"] = 2

        config = config_essai(fs.base, retouche=retouche)
        for _ in range(2):
            await config.passerelle().chat("agent", [{"role": "user", "content": "x"}])
        with pytest.raises(QuotaAtteint, match="quota horaire"):
            await config.passerelle().chat("agent", [{"role": "user", "content": "x"}])
        assert len(fs.requetes) == 2
        # la vision a son propre quota
        await config.passerelle().chat("vision", [{"role": "user", "content": "x"}])
        assert config.passerelle().quotas()["gemini.agent"]["heure"] == 2
        await config.passerelle().fermer()


def test_quota_fenetre_horaire_et_jour() -> None:
    maintenant = [1_760_000_000.0]
    quota = Quota(par_heure=2, par_jour=3, horloge=lambda: maintenant[0])
    quota.prendre()
    quota.prendre()
    with pytest.raises(QuotaAtteint):
        quota.prendre()
    maintenant[0] += 3601
    quota.prendre()
    with pytest.raises(QuotaAtteint, match="journalier"):
        quota.prendre()
    maintenant[0] += 86400  # lendemain
    quota.prendre()
    assert quota.etat()["jour"] == 1


def test_extraire_json() -> None:
    assert extraire_json('```json\n[{"point": [1, 2]}]\n```') == [{"point": [1, 2]}]
    assert extraire_json('Voici : {"a": 1} fin') == {"a": 1}
    assert extraire_json("[]") == []
    with pytest.raises(ValueError):
        extraire_json("rien")


def test_lire_transcription_gemini_mots() -> None:
    donnees = {"candidates": [{"content": {"parts": [{"audioTranscription": {
        "speakerLabel": "spk_1", "words": [
            {"word": "avance", "startOffset": "0.1s", "endOffset": "0.4s"},
            {"word": "de", "startOffset": "0.4s", "endOffset": "0.5s"},
            {"word": "2", "startOffset": "0.5s", "endOffset": "0.6s"},
            {"word": "mètres", "startOffset": "0.6s", "endOffset": "0.9s"},
            {"word": ".", "startOffset": "0.9s", "endOffset": "0.9s"}]}}]}}]}
    assert lire_transcription_gemini(donnees) == "avance de 2 mètres."
    assert lire_transcription_gemini({"candidates": []}) == ""
