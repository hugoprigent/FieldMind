"""Poste FieldMind public avec un rover simulé distinct par navigateur.

À publier derrière un proxy TLS sous /fieldmind/. Chaque navigateur reçoit un cookie
opaque et son propre Rover, Bus, carte, réglages, photos et atelier drone simulé. Aucun
pilote réel n'est initialisé dans ce mode. L'IA est servie côté serveur : GX10 si joignable,
Gemini si le GX10 est indisponible et qu'une clé Gemini est configurée.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import os
import secrets
import shutil
import tempfile
import time
import tomllib
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import aiohttp
from aiohttp import web

from fieldmind.api.serveur import DOSSIER_WEB, SUJETS_EVENEMENTS, Serveur
from fieldmind.app import Rover
from fieldmind.config import FICHIER_PROFIL, Profil
from fieldmind.drone.atelier import AtelierDrone
from fieldmind.ia.config import ConfigIA
from fieldmind.mission.actions import ActionRefusee

JOURNAL = logging.getLogger("fieldmind.public_demo")
NOM_COOKIE = "fieldmind_public_session"
MAX_SESSIONS = 4
DUREE_INACTIVITE_S = 120


@dataclass(slots=True)
class Session:
    identifiant: str
    dossier: Path
    rover: Rover
    serveur: ServeurSession


class ServeurSession(Serveur):
    """Un serveur et un bus privés pour une seule session du poste public."""

    def __init__(self, rover: Rover, dossier: Path, config_ia: ConfigIA,
                 profil_ia_public: str) -> None:
        super().__init__(rover, jeton="")
        self.dossier_photos = dossier / "photos"
        self.atelier_drone = AtelierDrone(self.bus, dossier=dossier / "drone")
        self.derniere_activite = time.monotonic()
        self.config_ia = config_ia
        self.profil_ia_public = profil_ia_public

    async def _au_demarrage(self, _app: web.Application) -> None:
        self.boucle = asyncio.get_running_loop()
        await self.rover.demarrer_ia(self.boucle, self.config_ia)
        for sujet, nom in SUJETS_EVENEMENTS.items():
            self._taches.append(asyncio.create_task(self._relayer(sujet, nom)))
        self._taches.append(asyncio.create_task(self._relayer_console()))
        self._taches.append(asyncio.create_task(self._diffuser()))

    async def _recevoir(self, poste: Any, texte: str) -> None:
        self.derniere_activite = time.monotonic()
        await super()._recevoir(poste, texte)

    async def _ws_poste(self, requete: web.Request) -> web.WebSocketResponse:
        async def actualiser_activite() -> None:
            while True:
                self.derniere_activite = time.monotonic()
                await asyncio.sleep(30)

        tache = asyncio.create_task(actualiser_activite())
        try:
            return await super()._ws_poste(requete)
        finally:
            tache.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await tache

    async def _traiter(self, poste: Any, message: dict[str, Any]) -> Any:
        type_message = message.get("type")
        if type_message == "ia_cle":
            raise ActionRefusee("Les clés d’IA sont gérées côté serveur dans cette simulation.")
        if type_message == "ia_profil" and message.get("profil") not in (
                self.profil_ia_public, "aucun"):
            raise ActionRefusee("Le modèle de cette simulation est géré côté serveur.")
        return await super()._traiter(poste, message)


class ServeurPublic:
    VARIABLES_IA = frozenset({
        "GX10_API_KEY", "GEMINI_API_KEY", "GEMINI_API_KEY_GRATUITE",
        "FIELDMIND_GX10_HOTE",
    })

    def __init__(self, *, scenario: str = "jardin", max_sessions: int = MAX_SESSIONS,
                 inactivite_s: int = DUREE_INACTIVITE_S) -> None:
        self.scenario = scenario
        self.max_sessions = max(1, min(MAX_SESSIONS, max_sessions))
        self.inactivite_s = max(60, min(600, inactivite_s))
        self.cookie_path = os.environ.get("FIELDMIND_DEMO_COOKIE_PATH", "/")
        self.sessions: dict[str, Session] = {}
        self.profil_ia_public = "aucun"
        self._env_ia: dict[str, str] = {}
        self._allocation = asyncio.Lock()
        self._reaper: asyncio.Task[None] | None = None
        self.app = web.Application(middlewares=[self._entetes])
        self.app.add_routes([
            web.get("/", self._accueil),
            web.get("/api/sante", self._sante),
            web.get("/ws", self._route("_ws_poste")),
            web.get("/flux/{camera}", self._route("_flux")),
            web.get("/photo/{camera}", self._route("_photo")),
            web.get("/api/etat", self._route("_api_etat")),
            web.get("/api/profil", self._route("_api_profil")),
            web.get("/api/journal", self._route("_api_journal")),
            web.get("/api/drone/recon/{fichier}", self._route("_recon_fichier")),
            web.get("/api/drone/atelier/{fichier}", self._route("_atelier_fichier")),
        ])
        if DOSSIER_WEB.exists():
            self.app.router.add_static("/", DOSSIER_WEB, show_index=False)
        self.app.on_startup.append(self._demarrer)
        self.app.on_shutdown.append(self._arreter)

    @web.middleware
    async def _entetes(
        self,
        requete: web.Request,
        handler: Callable[[web.Request], Awaitable[web.StreamResponse]],
    ) -> web.StreamResponse:
        if requete.path in ("/ws", "/ws/ros"):
            self._verifier_origine(requete)
        reponse = await handler(requete)
        reponse.headers.setdefault("Cache-Control", "no-store")
        reponse.headers.setdefault("X-Content-Type-Options", "nosniff")
        reponse.headers.setdefault("Referrer-Policy", "same-origin")
        return reponse

    @staticmethod
    def _verifier_origine(requete: web.Request) -> None:
        origine = requete.headers.get("Origin")
        hote = requete.headers.get("X-Forwarded-Host", requete.host).split(",", 1)[0].strip()
        if not origine or origine.rstrip("/").rsplit("://", 1)[-1].lower() != hote.lower():
            raise web.HTTPForbidden(text="origine WebSocket refusée")

    def _route(self, methode: str) -> Callable[[web.Request], Awaitable[web.StreamResponse]]:
        async def traiter(requete: web.Request) -> web.StreamResponse:
            session = self._depuis_cookie(requete)
            if session is None:
                raise web.HTTPUnauthorized(text="session de simulation absente ; recharge la page")
            return await getattr(session.serveur, methode)(requete)

        return traiter

    def _depuis_cookie(self, requete: web.Request) -> Session | None:
        identifiant = requete.cookies.get(NOM_COOKIE)
        return self.sessions.get(identifiant or "")

    async def _accueil(self, requete: web.Request) -> web.StreamResponse:
        session = self._depuis_cookie(requete)
        if session is None:
            try:
                session = await self._nouvelle_session()
            except RuntimeError:
                reponse = web.Response(
                    status=503,
                    text=("Toutes les simulations sont occupées. Réessaie dans 15 secondes ou "
                          "recharge cette page."),
                    content_type="text/plain",
                    headers={"Retry-After": "15"},
                )
                return reponse
            except Exception:
                JOURNAL.exception("impossible de démarrer une session de simulation")
                return web.Response(
                    status=503,
                    text="La simulation est momentanément indisponible.",
                )
        reponse = await session.serveur._accueil(requete)
        https = requete.headers.get("X-Forwarded-Proto", requete.scheme) == "https"
        reponse.set_cookie(
            NOM_COOKIE, session.identifiant, path=self.cookie_path, httponly=True,
            secure=https, samesite="Lax", max_age=self.inactivite_s + 60,
        )
        return reponse

    async def _nouvelle_session(self) -> Session:
        async with self._allocation:
            if len(self.sessions) >= self.max_sessions:
                raise RuntimeError("capacité atteinte")
            dossier = Path(tempfile.mkdtemp(prefix="fieldmind-public-"))
            rover: Rover | None = None
            serveur: ServeurSession | None = None
            try:
                def construire() -> Rover:
                    with FICHIER_PROFIL.open("rb") as fichier:
                        base = tomllib.load(fichier)
                    base.setdefault("cameras", {})["calibration"] = str(dossier / "stereo.npz")
                    base.setdefault("enregistrement", {})["actif"] = False
                    # caméras simulées allégées : rendu logiciel partagé entre les visiteurs
                    base["cameras"].update(ips=5)
                    base.setdefault("simulation", {})["camera_repli_largeur"] = 384
                    base.setdefault("ia", {}).update(detection_continue=False,
                                                     gx10_periode_s=8.0, veille=False)
                    base.setdefault("perception", {}).update(stereo_largeur=320, stereo_ips=2)
                    profil = Profil(base, {}, dossier / "reglages.json")
                    return Rover(profil, simulation=True, scenario=self.scenario, graine=0,
                                 version_esp="ROVER_3", enregistrer=False,
                                 dossier_utilisateur=dossier / "utilisateur")

                rover = await asyncio.to_thread(construire)
                rover.executeur.dossier_photos = dossier / "photos"
                rover.demarrer()
                config_ia = self._config_ia_session()
                serveur = ServeurSession(rover, dossier, config_ia, self.profil_ia_public)
                await serveur._au_demarrage(self.app)
                identifiant = secrets.token_urlsafe(32)
                session = Session(identifiant, dossier, rover, serveur)
                self.sessions[identifiant] = session
                JOURNAL.info("session simulée créée (%d/%d)", len(self.sessions), self.max_sessions)
                return session
            except Exception:
                if serveur is not None:
                    await serveur._a_l_arret(self.app)
                if rover is not None:
                    await asyncio.to_thread(rover.arreter)
                shutil.rmtree(dossier, ignore_errors=True)
                raise

    async def _sante(self, _requete: web.Request) -> web.Response:
        return web.json_response({
            "ok": True,
            "simulation": True,
            "ia": self.profil_ia_public,
            "sessions_actives": len(self.sessions),
            "sessions_max": self.max_sessions,
        })

    async def _demarrer(self, _app: web.Application) -> None:
        # Ce processus ne lit ni le .env du projet ni les secrets du compte réel. Seules les
        # deux clés explicitement prévues pour l'IA publique et l'adresse du GX10 sont conservées.
        for cle in tuple(os.environ):
            if (cle.endswith("_API_KEY") or cle.endswith("_API_KEY_GRATUITE")) \
                    and cle not in self.VARIABLES_IA:
                os.environ.pop(cle, None)
        for cle in ("FIELDMIND_JETON",):
            os.environ.pop(cle, None)
        self._env_ia = {cle: os.environ[cle] for cle in self.VARIABLES_IA if os.environ.get(cle)}
        self.profil_ia_public = await self._choisir_profil_ia()
        os.environ["FIELDMIND_IA_PROFIL"] = self.profil_ia_public
        JOURNAL.info("profil IA public : %s", self.profil_ia_public)
        self._reaper = asyncio.create_task(self._expirer_sessions(), name="fieldmind-public-reaper")

    def _config_ia_session(self) -> ConfigIA:
        config = ConfigIA.charger(fichier_env=None, fichier_choix=None, env=self._env_ia)
        config.changer_profil(self.profil_ia_public, enregistrer=False)
        profil = config.profil(self.profil_ia_public)
        if self.profil_ia_public == "gemini":
            roles = {}
            for nom in ("agent", "vision", "transcription"):
                role = profil.role(nom)
                if role is not None:
                    roles[nom] = role  # pas de plafond supplémentaire (demande de Hugo, 23/10)
            profil = replace(profil, **roles)
            config.profils[self.profil_ia_public] = profil
        if self.profil_ia_public == "gemini":
            proxy = os.environ.get("FIELDMIND_PUBLIC_GEMINI_PROXY")
            if proxy not in (None, "http://127.0.0.1:3139"):
                raise ValueError("proxy Gemini public non autorisé")
            config.proxy_gemini = proxy
        # Les visiteurs peuvent couper l'IA, mais pas envoyer les requêtes à un autre fournisseur.
        aucun = config.profil("aucun")  # à lire avant de restreindre la liste des profils
        config.profils = {self.profil_ia_public: profil}
        if self.profil_ia_public != "aucun":
            config.profils["aucun"] = aucun
        return config

    async def _choisir_profil_ia(self) -> str:
        """Préfère le GX10 réellement joignable ; utilise Gemini seulement en secours."""
        config = ConfigIA.charger(fichier_env=None, fichier_choix=None, env=self._env_ia)
        for _essai in range(3):  # le GX10 peut répondre lentement quand il est chargé
            if await self._gx10_disponible(config):
                return "gx10"
            await asyncio.sleep(2)
        if config.disponibilite("agent", "gemini")[0]:
            return "gemini"
        return "aucun"

    async def _gx10_disponible(self, config: ConfigIA) -> bool:
        profil = config.profil("gx10")
        role = profil.agent
        if role is None:
            return False
        cle = config.cle("gx10")
        entetes = {"Authorization": f"Bearer {cle}"} if cle else {}
        try:
            delai = aiohttp.ClientTimeout(total=10)
            async with (aiohttp.ClientSession(timeout=delai) as session,
                        session.get(f"{role.url.rstrip('/')}/models",
                                    headers=entetes) as reponse):
                if reponse.status != 200:
                    return False
                donnees = await reponse.json(content_type=None)
            if not isinstance(donnees, dict):
                return False
            liste = donnees.get("data", [])
            if not isinstance(liste, list):
                return False
            modeles = {str(m.get("id", "")) for m in liste if isinstance(m, dict)}
            return role.modele in modeles
        except (aiohttp.ClientError, TimeoutError, ValueError, TypeError):
            return False

    async def _arreter(self, _app: web.Application) -> None:
        if self._reaper is not None:
            self._reaper.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._reaper
        for session in list(self.sessions.values()):
            await self._fermer_session(session)

    async def _expirer_sessions(self) -> None:
        while True:
            await asyncio.sleep(15)
            maintenant = time.monotonic()
            expires = [session for session in self.sessions.values()
                       if maintenant - session.serveur.derniere_activite >= self.inactivite_s]
            for session in expires:
                JOURNAL.info("session simulée expirée après inactivité")
                await self._fermer_session(session)

    async def _fermer_session(self, session: Session) -> None:
        self.sessions.pop(session.identifiant, None)
        await session.serveur._a_l_arret(self.app)
        await asyncio.to_thread(session.rover.arreter)
        shutil.rmtree(session.dossier, ignore_errors=True)


def main() -> None:
    parseur = argparse.ArgumentParser(description=__doc__)
    parseur.add_argument("--hote", default="127.0.0.1")
    parseur.add_argument("--port", type=int, default=8085)
    parseur.add_argument("--scenario", choices=("jardin", "salle"), default="jardin")
    parseur.add_argument("--sessions", type=int, default=MAX_SESSIONS,
                         help="nombre de sessions simultanées (1 ou 2)")
    parseur.add_argument("--inactivite", type=int, default=DUREE_INACTIVITE_S,
                         help="secondes avant l'arrêt d'une session inactive")
    args = parseur.parse_args()
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s : %(message)s")
    logging.getLogger("aiohttp.access").setLevel(logging.WARNING)
    serveur = ServeurPublic(scenario=args.scenario, max_sessions=args.sessions,
                            inactivite_s=args.inactivite)
    web.run_app(serveur.app, host=args.hote, port=args.port, print=None, shutdown_timeout=5)


if __name__ == "__main__":
    main()
