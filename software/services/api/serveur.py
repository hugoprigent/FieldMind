"""Serveur FastAPI de la démo : poste opérateur (fichiers statiques), API de santé et WebSocket.

Lancement depuis software/ :
    uvicorn services.api.serveur:app --reload
"""

from __future__ import annotations

import asyncio
import base64
import hashlib
import logging
import re
import uuid
from collections.abc import Awaitable, Callable
from functools import lru_cache

from fastapi import FastAPI, Request, Response, WebSocket, WebSocketDisconnect, status
from fastapi.staticfiles import StaticFiles

from services.api.config import Reglages, reglages
from services.api.journal import JournalJsonl
from services.api.robot_distant import LiaisonRobotDistante, PosteRobot
from services.api.session import Session, VisionPartagee
from services.mission_planner.api_openai import (
    ClientCompatibleOpenAI,
    InfosModele,
    infos_modeles_openrouter,
)
from services.mission_planner.consignes import (
    Fournisseur,
    FournisseurAnthropic,
    FournisseurCompatibleOpenAI,
    InterpreteConsignes,
    QuotaJournalier,
)
from services.navigation.planificateur import prechauffer
from services.perception.vision import (
    FournisseurVision,
    FournisseurVisionOpenAI,
    consigne_vision,
    format_boites_pour,
)
from services.protocole.fiche_rover import FicheRover
from services.protocole.scenario import ScenarioPublic
from simulation.monde import Monde, charger_monde
from simulation.robot_simule import RobotSimule

journal = logging.getLogger("fieldmind.serveur")

VERSION = "0.1.0"


def _empreinte_import_map(html: str) -> str:
    """Empreinte CSP du bloc <script type="importmap">, seul script en ligne de la page."""
    bloc = re.search(r'<script type="importmap">(.*?)</script>', html, re.DOTALL)
    if bloc is None:
        return ""
    digest = hashlib.sha256(bloc.group(1).encode("utf-8")).digest()
    return f"'sha256-{base64.b64encode(digest).decode('ascii')}'"


def _politique_securite(empreinte: str) -> str:
    """Content-Security-Policy : tout vient du serveur lui-même, aucune ressource externe."""
    return "; ".join(
        [
            "default-src 'self'",
            f"script-src 'self' 'wasm-unsafe-eval' {empreinte}".strip(),
            "style-src 'self'",
            "img-src 'self' data: blob:",
            "connect-src 'self'",
            "worker-src 'self' blob:",
            "frame-ancestors 'none'",
            "base-uri 'none'",
            "form-action 'none'",
        ]
    )


class ModeleRefuse(Exception):
    """Modèle OpenRouter absent du catalogue, payant, ou sans entrée image pour la vision."""


def _client_openrouter(
    url: str,
    modele: str,
    cle: str,
    delai_s: float,
    secours: tuple[str, ...],
    infos: dict[str, InfosModele] | None,
    images: bool,
) -> ClientCompatibleOpenAI:
    """Client OpenRouter limité aux modèles gratuits (vérifiés dans le catalogue public)."""

    def accepte(nom: str) -> str | None:
        if infos is None:
            return None  # catalogue injoignable : le plafond de prix à zéro protège
        info = infos.get(nom)
        if info is None:
            return "absent du catalogue OpenRouter"
        if not info.gratuit:
            return "payant (seuls les modèles gratuits sont permis)"
        if images and not info.images:
            return "n'accepte pas les images"
        return None

    raison = accepte(modele)
    if raison:
        raise ModeleRefuse(f"{modele} : {raison}")
    secours_ok = tuple(m for m in secours if accepte(m) is None)
    for m in set(secours) - set(secours_ok):
        journal.warning("modèle de secours %s écarté : %s", m, accepte(m))
    schema = bool(infos and infos[modele].schema_json)
    return ClientCompatibleOpenAI(
        url,
        cle,
        modele,
        delai_s,
        openrouter=True,
        modeles_secours=secours_ok,
        mode_json="schema" if schema else "objet",
    )


def _infos_openrouter(cfg: Reglages) -> dict[str, InfosModele] | None:
    modeles = {cfg.modele_chat, cfg.modele_vision, *cfg.modeles_secours}
    if "openrouter" not in (cfg.fournisseur_chat, cfg.fournisseur_vision):
        return None
    return infos_modeles_openrouter(sorted(modeles), cfg.openrouter_url)


def creer_fournisseur(
    cfg: Reglages, scenario: ScenarioPublic, infos: dict[str, InfosModele] | None = None
) -> Fournisseur | None:
    """Fournisseur de modèle de langage du chat, ou None (consignes écrites non traitées)."""
    fournisseur = cfg.fournisseur_chat
    if fournisseur == "aucun":
        return None
    cle = cfg.llm_api_key.get_secret_value()
    if fournisseur == "anthropic":
        return FournisseurAnthropic(
            scenario, cfg.llm_model, cle, cfg.llm_delai_s, cfg.llm_jetons_max
        )
    if fournisseur == "openrouter":
        try:
            client = _client_openrouter(
                cfg.openrouter_url,
                cfg.modele_chat,
                cfg.cle_openrouter,
                cfg.llm_delai_s,
                cfg.modeles_secours,
                infos,
                images=False,
            )
        except ModeleRefuse as erreur:
            journal.error("chat : modèle refusé, consignes écrites désactivées (%s)", erreur)
            return None
    else:
        client = ClientCompatibleOpenAI(
            cfg.llm_base_url,
            cle,
            cfg.llm_model,
            cfg.llm_delai_s,
            modeles_secours=cfg.modeles_secours,
        )
    return FournisseurCompatibleOpenAI(scenario, client, cfg.llm_jetons_max)


def creer_vision(
    cfg: Reglages, fiche: FicheRover, infos: dict[str, InfosModele] | None = None
) -> FournisseurVision | None:
    """Modèle de vision partagé par les sessions ; None pour le détecteur de la simulation ou
    sans vision (voir `Reglages.fournisseur_vision`)."""
    fournisseur = cfg.fournisseur_vision
    camera = fiche.capteurs.camera
    if fournisseur in ("simulation", "aucun") or camera is None:
        return None
    format_boites = format_boites_pour(cfg.modele_vision)
    consigne = consigne_vision(
        camera.position[2], fiche.geometrie.largeur, fiche.geometrie.garde_au_sol, format_boites
    )
    if fournisseur == "openrouter":
        try:
            client = _client_openrouter(
                cfg.openrouter_url,
                cfg.modele_vision,
                cfg.cle_openrouter,
                cfg.vision_delai_s,
                cfg.modeles_secours_vision,
                infos,
                images=True,
            )
        except ModeleRefuse as erreur:
            journal.error("vision : modèle refusé (%s)", erreur)
            return None
    else:
        client = ClientCompatibleOpenAI(
            cfg.llm_base_url,
            cfg.llm_api_key.get_secret_value(),
            cfg.modele_vision,
            cfg.vision_delai_s,
            modeles_secours=cfg.modeles_secours_vision,
        )
    return FournisseurVisionOpenAI(client, consigne, cfg.vision_jetons_max, format_boites)


def creer_application() -> FastAPI:
    """Construit l'application : routes, en-têtes de sécurité et fichiers du poste opérateur."""
    cfg = reglages()
    app = FastAPI(title="FieldMind — démo de navigation", version=VERSION, docs_url=None)

    index = cfg.dossier_web / "index.html"
    csp = _politique_securite(_empreinte_import_map(index.read_text(encoding="utf-8")))

    @app.middleware("http")
    async def entetes_securite(
        request: Request, call_next: Callable[[Request], Awaitable[Response]]
    ) -> Response:
        reponse = await call_next(request)
        reponse.headers["Content-Security-Policy"] = csp
        reponse.headers["X-Content-Type-Options"] = "nosniff"
        reponse.headers["Referrer-Policy"] = "no-referrer"
        reponse.headers["Permissions-Policy"] = "camera=(), microphone=(), geolocation=()"
        return reponse

    prechauffer()  # compile A* (numba) avant la première session
    fiche = FicheRover.charger(cfg.fiche_rover)
    scenario = ScenarioPublic.charger(cfg.scenario)
    sessions: dict[str, Session] = {}
    infos = _infos_openrouter(cfg)
    consignes = InterpreteConsignes(
        scenario, creer_fournisseur(cfg, scenario, infos), QuotaJournalier(cfg.chat_max_par_jour)
    )
    app.state.consignes = consignes
    vision = VisionPartagee(
        creer_vision(cfg, fiche, infos),
        simulation=cfg.fournisseur_vision == "simulation",
        quota=QuotaJournalier(cfg.vision_max_par_jour),
    )
    journal_jsonl = JournalJsonl(cfg.dossier_journaux)
    journal.info("consignes : %s ; vision : %s", consignes.nom, vision.nom)

    @lru_cache(maxsize=16)
    def monde_pour(graine: int) -> Monde:
        return charger_monde(cfg.scenario, graine)

    poste_robot = PosteRobot(cfg.jeton_robot.get_secret_value())

    @app.get("/api/sante")
    async def sante() -> dict[str, str | int | bool]:
        robot = poste_robot.liaison
        return {
            "statut": "ok",
            "version": VERSION,
            "sessions": len(sessions),
            "robot_distant_connecte": robot is not None and robot.connecte,
        }

    @app.websocket("/ws")
    async def websocket(ws: WebSocket, graine: int | None = None, robot: str | None = None) -> None:
        await ws.accept()
        if len(sessions) >= cfg.sessions_max:
            await ws.send_json({"type": "fin_session", "raison": "complet"})
            await ws.close(code=status.WS_1013_TRY_AGAIN_LATER)
            return
        identifiant = uuid.uuid4().hex[:8]
        distant = None
        if robot == "externe":
            distant = poste_robot.prendre(identifiant)
            if distant is None:
                await ws.send_json({"type": "fin_session", "raison": "robot_indisponible"})
                await ws.close(code=status.WS_1013_TRY_AGAIN_LATER)
                return
        graine_monde = scenario.graine if graine is None else max(0, min(graine, 2**31 - 1))
        monde = None
        robot_simule = None
        if distant is None:
            # Monde, texture du sol et modèle MuJoCo : hors de la boucle (les autres sessions
            # continuent de tourner pendant ce temps).
            monde = await asyncio.to_thread(monde_pour, graine_monde)
            robot_simule = await asyncio.to_thread(
                RobotSimule, fiche, monde, graine_bruit=monde.graine
            )
        session = Session(
            ws,
            cfg,
            fiche,
            scenario,
            monde,
            consignes,
            journal_jsonl,
            robot_distant=distant,
            identifiant=identifiant,
            vision=vision,
            robot_simule=robot_simule,
        )
        sessions[session.id] = session
        mode = "robot distant" if distant else f"simulation, graine {graine_monde}"
        journal.info("session %s ouverte (%s, %d actives)", session.id, mode, len(sessions))
        try:
            await session.executer()
        finally:
            sessions.pop(session.id, None)
            poste_robot.rendre(session.id)
            journal.info("session %s fermée", session.id)

    @app.websocket("/ws/robot")
    async def websocket_robot(ws: WebSocket, jeton: str | None = None) -> None:
        """Pont du robot réel (ou d'un simulateur séparé) : un seul à la fois, avec jeton."""
        if not poste_robot.jeton_valide(jeton):
            await ws.close(code=status.WS_1008_POLICY_VIOLATION)
            return
        await ws.accept()
        if poste_robot.liaison is not None and poste_robot.liaison.connecte:
            await ws.close(code=status.WS_1013_TRY_AGAIN_LATER, reason="un robot est déjà connecté")
            return
        liaison = LiaisonRobotDistante(ws)
        poste_robot.liaison = liaison
        journal.info("robot distant connecté")
        try:
            await liaison.pomper()
        except WebSocketDisconnect:
            pass
        finally:
            if poste_robot.liaison is liaison:
                poste_robot.liaison = None
            journal.info("robot distant déconnecté")

    app.mount("/", StaticFiles(directory=cfg.dossier_web, html=True), name="web")
    return app


app = creer_application()
