"""Clé payante prioritaire, clé gratuite de secours (même modèle) sur quota ou panne passagère."""

from __future__ import annotations

import contextlib
from collections.abc import AsyncIterator

import pytest
from aiohttp import web
from test_ia_faux_serveur import donnees_ia

from fieldmind.ia.config import ConfigIA
from fieldmind.ia.fournisseurs import ErreurIA

PAYANTE, GRATUITE = "cle-payante", "cle-gratuite"


@contextlib.asynccontextmanager
async def serveur(statut_payante: int) -> AsyncIterator[tuple[str, list[tuple[str, str]]]]:
    """Faux Gemini : la clé payante reçoit `statut_payante`, la gratuite répond normalement."""
    vus: list[tuple[str, str]] = []

    async def chat(requete: web.Request) -> web.Response:
        cle = requete.headers.get("Authorization", "").removeprefix("Bearer ")
        corps = await requete.json()
        vus.append((cle, corps["model"]))
        if cle == PAYANTE and statut_payante != 200:
            return web.json_response({"error": {"message": "saturé"}}, status=statut_payante)
        return web.json_response({"choices": [{"message": {"role": "assistant",
                                                           "content": "OK"}}]})

    app = web.Application()
    app.router.add_post("/v1beta/openai/chat/completions", chat)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", 0)
    await site.start()
    port = site._server.sockets[0].getsockname()[1]
    try:
        yield f"http://127.0.0.1:{port}", vus
    finally:
        await runner.cleanup()


def config(base: str, env: dict[str, str]) -> ConfigIA:
    donnees = donnees_ia(base)
    for profil in donnees["profils"].values():
        profil["reprises"] = 0
    return ConfigIA.depuis_dict(donnees, env, None)


MESSAGES = [{"role": "user", "content": "test"}]


@pytest.mark.parametrize("statut", [429, 503])
async def test_vision_bascule_sur_la_cle_gratuite_meme_modele(statut: int) -> None:
    async with serveur(statut) as (base, vus):
        c = config(base, {"GEMINI_API_KEY": PAYANTE, "GEMINI_API_KEY_GRATUITE": GRATUITE})
        reponse = await c.passerelle().chat("vision", MESSAGES)
        await c.passerelle().fermer()
    assert reponse.texte == "OK"
    assert [cle for cle, _ in vus] == [PAYANTE, GRATUITE]
    assert vus[0][1] == vus[1][1] == "gemini-robotics-er-2-preview"


async def test_agent_argon_jamais_sur_la_cle_gratuite() -> None:
    async with serveur(429) as (base, vus):
        c = config(base, {"GEMINI_API_KEY": PAYANTE, "GEMINI_API_KEY_GRATUITE": GRATUITE})
        with pytest.raises(ErreurIA):
            await c.passerelle().chat("agent", MESSAGES)
        await c.passerelle().fermer()
    assert [cle for cle, _ in vus] == [PAYANTE]


async def test_cle_refusee_ne_bascule_pas() -> None:
    async with serveur(403) as (base, vus):
        c = config(base, {"GEMINI_API_KEY": PAYANTE, "GEMINI_API_KEY_GRATUITE": GRATUITE})
        with pytest.raises(ErreurIA):
            await c.passerelle().chat("vision", MESSAGES)
        await c.passerelle().fermer()
    assert [cle for cle, _ in vus] == [PAYANTE]


async def test_sans_cle_payante_vision_sur_la_gratuite_agent_indisponible() -> None:
    async with serveur(200) as (base, vus):
        c = config(base, {"GEMINI_API_KEY_GRATUITE": GRATUITE})
        assert c.disponibilite("vision")[0] and not c.disponibilite("agent")[0]
        await c.passerelle().chat("vision", MESSAGES)
        await c.passerelle().fermer()
    assert [cle for cle, _ in vus] == [GRATUITE]
