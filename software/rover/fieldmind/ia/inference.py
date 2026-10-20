"""Client du serveur des outils du GX10 (deploiement/gx10/outils/serveur.py).

Rôles servis (fournisseur `inference` dans config/ia.toml) : `detection` (D-FINE), `profondeur`
(Depth Anything V2 métrique), `memoire` (empreintes Qwen3-VL-Embedding) et `synthese`
(Qwen3-TTS). La segmentation (SAM 2.1) passe par le même point d'accès que la profondeur.

Une image part une fois avec toutes les tâches voulues (`analyser`) ; le serveur renvoie un
`image_id` réutilisable quelques minutes : les demandes suivantes sur la même image n'envoient
que cet identifiant. Les connexions restent ouvertes : sur une liaison qui perd des paquets,
ouvrir une connexion coûte parfois plusieurs secondes.
"""

from __future__ import annotations

import base64
import logging
import time
from typing import Any

import aiohttp

from fieldmind.ia.config import ConfigIA, Role
from fieldmind.ia.fournisseurs import ErreurIA, IAIndisponible

journal = logging.getLogger("fieldmind.ia.inference")


class ClientInference:
    def __init__(self, config: ConfigIA) -> None:
        self.config = config
        self._sessions: dict[str, aiohttp.ClientSession] = {}

    def disponible(self, role: Role) -> bool:
        profil = self.config.actif
        c = profil.role(role)
        return c is not None and c.fournisseur == "inference" and bool(c.url)

    def modele(self, role: Role) -> str:
        c = self.config.actif.role(role)
        return c.modele if c is not None else ""

    def _role(self, role: Role) -> tuple[str, dict[str, str], float]:
        if not self.disponible(role):
            raise IAIndisponible(f"pas de modèle de {role} dans le profil "
                                 f"« {self.config.actif.nom} »")
        c = self.config.actif.role(role)
        assert c is not None
        cle = self.config.cle_role(role)
        entetes = {"Authorization": f"Bearer {cle}"} if cle else {}
        return c.url.rstrip("/"), entetes, c.delai_s

    def _session(self, url: str) -> aiohttp.ClientSession:
        session = self._sessions.get(url)
        if session is None or session.closed:
            session = aiohttp.ClientSession(
                connector=aiohttp.TCPConnector(limit=4, keepalive_timeout=120))
            self._sessions[url] = session
        return session

    async def _poster(self, role: Role, chemin: str, corps: dict[str, Any],
                      binaire: bool = False) -> Any:
        url, entetes, delai = self._role(role)
        debut = time.monotonic()
        try:
            async with self._session(url).post(
                    f"{url}{chemin}", json=corps, headers=entetes,
                    timeout=aiohttp.ClientTimeout(total=delai)) as reponse:
                if reponse.status >= 400:
                    texte = (await reponse.text())[:200]
                    raise ErreurIA(f"outils du GX10 ({role}) : HTTP {reponse.status} {texte}")
                donnees = await reponse.read() if binaire else await reponse.json()
        except (TimeoutError, aiohttp.ClientError) as erreur:
            raise ErreurIA(f"outils du GX10 ({role}) injoignables : {erreur!r}") from erreur
        journal.debug("%s %s en %.2f s", role, chemin, time.monotonic() - debut)
        return donnees

    async def analyser(self, jpeg: bytes | None = None, *, image_id: str | None = None,
                       taches: tuple[str, ...] = ("detection",),
                       points: list[tuple[float, float]] | None = None,
                       boites: list[tuple[float, float, float, float]] | None = None,
                       seuil: float = 0.4, role: Role = "detection") -> dict[str, Any]:
        """Détection, profondeur, segmentation et empreinte d'une même image, en un envoi.
        Renvoie la réponse du serveur (`image_id`, `detections`, `profondeur`, `segmentation`,
        `empreinte`, `durees_ms`, `erreurs`)."""
        corps: dict[str, Any] = {"taches": list(taches), "seuil": seuil,
                                 "points": [list(p) for p in points or []],
                                 "boites": [list(b) for b in boites or []]}
        if image_id:
            corps["image_id"] = image_id
        elif jpeg is not None:
            corps["image"] = base64.b64encode(jpeg).decode("ascii")
        else:
            raise ValueError("jpeg ou image_id attendu")
        return dict(await self._poster(role, "/analyse", corps))

    async def empreinte(self, jpeg: bytes | None = None, texte: str | None = None) -> list[float]:
        corps: dict[str, Any] = {"texte": texte} if texte else {}
        if jpeg is not None:
            corps["image"] = base64.b64encode(jpeg).decode("ascii")
        reponse = await self._poster("memoire", "/empreintes", corps)
        return [float(x) for x in reponse["vecteur"]]

    async def parole(self, texte: str) -> bytes:
        """Texte → WAV (voix du rover)."""
        c = self.config.actif.role("synthese")
        corps: dict[str, Any] = {"input": texte[:400], "language": "French"}
        if c is not None and c.voix:
            corps["voice"] = c.voix
        return bytes(await self._poster("synthese", "/audio/speech", corps, binaire=True))

    async def fermer(self) -> None:
        for session in self._sessions.values():
            await session.close()
        self._sessions.clear()


def similarite(a: list[float], b: list[float]) -> float:
    """Cosinus de deux empreintes (déjà normalisées par le serveur)."""
    if not a or len(a) != len(b):
        return 0.0
    return float(sum(x * y for x, y in zip(a, b, strict=True)))
