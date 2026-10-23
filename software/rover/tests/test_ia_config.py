"""Configuration de l'IA : config/ia.toml, profils, bascule en marche, clés, script verifier.py."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import Any

import pytest
from test_ia_faux_serveur import FauxOutils, appels, config_essai, donnees_ia, faux_serveur

from fieldmind.bus import Bus
from fieldmind.ia.agent import Agent
from fieldmind.ia.config import FICHIER_IA, ConfigIA, ErreurConfigIA, lire_env

RACINE = Path(__file__).resolve().parent.parent


def test_fichier_du_depot() -> None:
    config = ConfigIA.charger(FICHIER_IA, fichier_env=None, fichier_choix=None, env={})
    assert config.nom_actif == "gemini"
    assert list(config.profils) == ["gemini", "gx10", "aucun"]
    gemini = config.profil("gemini")
    assert gemini.url == "https://generativelanguage.googleapis.com/v1beta/openai/"
    assert gemini.modeles() == {"agent": "gemini-4-argon",
                                "vision": "gemini-robotics-er-2-preview",
                                "transcription": "gemini-3.5-transcribe"}
    assert gemini.agent is not None and gemini.agent.reflexion == "low"
    assert gemini.transcription is not None and gemini.transcription.mode == "gemini_natif"
    gx10 = config.profil("gx10")
    assert gx10.url == "http://gx10.local:8090/modele/v1"  # passerelle du GX10
    assert gx10.transcription is not None
    assert gx10.transcription.url == "http://gx10.local:8090/transcription/v1"
    assert gx10.transcription.mode == "openai_transcriptions"
    assert config.profil("aucun").agent is None
    assert config.voix.duree_min_s == 0.4 and config.voix.duree_max_s == 15.0
    assert config.voix.pre_roulement_s == 0.3 and config.voix.mot_eveil == ""


def test_resume_pour_bienvenue_sans_cle() -> None:
    config = ConfigIA.charger(FICHIER_IA, fichier_env=None, fichier_choix=None,
                              env={"GEMINI_API_KEY": "secret"})
    resume = config.resume()
    assert resume == {"profil": "gemini", "disponible": True, "modeles": {
        "agent": "gemini-4-argon", "vision": "gemini-robotics-er-2-preview",
        "transcription": "gemini-3.5-transcribe"}, "profils": ["gemini", "gx10", "aucun"],
        "raison": None}
    assert "secret" not in json.dumps(resume)
    sans_cle = ConfigIA.charger(FICHIER_IA, fichier_env=None, fichier_choix=None, env={})
    assert sans_cle.resume()["disponible"] is False
    assert "GEMINI_API_KEY" in sans_cle.resume()["raison"]


def test_cle_lue_dans_le_fichier_env(tmp_path: Path) -> None:
    env = tmp_path / ".env"
    env.write_text("# clés\nexport GEMINI_API_KEY=\"depuis-fichier\"\nAUTRE=1 # note\n",
                   encoding="utf-8")
    assert lire_env(env) == {"GEMINI_API_KEY": "depuis-fichier", "AUTRE": "1"}
    config = ConfigIA.charger(FICHIER_IA, fichier_env=env, fichier_choix=None, env={})
    assert config.cle() == "depuis-fichier"
    # l'environnement du processus a priorité sur le fichier
    config = ConfigIA.charger(FICHIER_IA, fichier_env=env, fichier_choix=None,
                              env={"GEMINI_API_KEY": "processus"})
    assert config.cle() == "processus"


def test_hote_du_gx10_par_variable() -> None:
    config = ConfigIA.charger(FICHIER_IA, fichier_env=None, fichier_choix=None,
                              env={"FIELDMIND_GX10_HOTE": "http://gx10.example.internal:8090"})
    gx10 = config.profil("gx10")
    assert gx10.url == "http://gx10.example.internal:8090/modele/v1"
    assert gx10.transcription is not None
    assert gx10.transcription.url == "http://gx10.example.internal:8090/transcription/v1"
    assert gx10.url_tunnel == "http://gx10.example.internal:8090/tunnel"
    assert config.disponibilite("agent", "gx10") == (True, None)  # clé facultative


def test_bascule_de_profil_enregistree(tmp_path: Path) -> None:
    choix = tmp_path / "ia.json"
    config = ConfigIA.charger(FICHIER_IA, fichier_env=None, fichier_choix=choix, env={})
    vus: list[str] = []
    config.sur_changement(lambda profil: vus.append(profil.nom))
    assert config.changer_profil("gx10").nom == "gx10"
    assert config.resume()["profil"] == "gx10"
    assert config.resume()["modeles"]["agent"] == "nvidia/Qwen3.6-35B-A3B-NVFP4"
    assert vus == ["gx10"]
    assert json.loads(choix.read_text()) == {"profil": "gx10"}
    # relu au démarrage suivant ; la variable FIELDMIND_IA_PROFIL a priorité
    assert ConfigIA.charger(FICHIER_IA, fichier_env=None, fichier_choix=choix,
                            env={}).nom_actif == "gx10"
    assert ConfigIA.charger(FICHIER_IA, fichier_env=None, fichier_choix=choix,
                            env={"FIELDMIND_IA_PROFIL": "aucun"}).nom_actif == "aucun"
    with pytest.raises(ErreurConfigIA, match="inconnu"):
        config.changer_profil("openai")
    assert config.nom_actif == "gx10"


async def test_bascule_en_marche_change_de_serveur() -> None:
    async with faux_serveur() as fs:
        fs.chat = lambda corps: appels(("avancer", {"distance_m": 1}))
        outils = FauxOutils()
        config = config_essai(fs.base, env={"GEMINI_API_KEY": "g"})
        agent = Agent(Bus(), None, config, outils)
        await agent.consigne("avance de 1 m")
        config.changer_profil("gx10", enregistrer=False)
        await agent.consigne("avance de 1 m")
        config.changer_profil("aucun", enregistrer=False)
        resultat = await agent.consigne("avance de 1 m")
        await config.passerelle().fermer()
    # Une étape explicite appelle le GX10 une seule fois : « avance de 1 m » est complète.
    assert [c for c, _ in fs.requetes] == ["/v1beta/openai/chat/completions",
                                           "/v1/chat/completions"]
    assert [corps["model"] for _, corps in fs.requetes] == [
        "gemini-4-argon", "nvidia/Qwen3.6-35B-A3B-NVFP4"]
    assert fs.requetes[1][1]["messages"][-1]["role"] == "user"
    assert resultat.statut == "indisponible" and len(outils.plans) == 2


@pytest.mark.parametrize(("cle", "valeur", "motif"), [
    ("api", "anthropic", "api"),
    ("reflexion", "maximale", "réflexion"),
    ("mode", "sténo", "transcription"),
    ("format_points", "xyz", "format_points"),
])
def test_configuration_incoherente(cle: str, valeur: str, motif: str) -> None:
    donnees: dict[str, Any] = donnees_ia("http://x")
    if cle == "api":
        donnees["profils"]["gemini"]["api"] = valeur
    elif cle == "mode":
        donnees["profils"]["gemini"]["transcription"]["mode"] = valeur
    else:
        donnees["profils"]["gemini"]["vision"][cle] = valeur
    with pytest.raises(ErreurConfigIA, match=motif):
        ConfigIA.depuis_dict(donnees, {})


def charger_verifier() -> Any:
    chemin = RACINE / "deploiement" / "gx10" / "verifier.py"
    spec = importlib.util.spec_from_file_location("verifier_gx10", chemin)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


async def test_verifier_contre_le_faux_serveur() -> None:
    verifier = charger_verifier()
    lignes: list[str] = []
    async with faux_serveur() as fs:
        def chat(corps: dict[str, Any]) -> Any:
            if corps.get("tools"):
                return appels(("avancer", {"distance_m": 2}), ("tourner", {"angle_deg": 90}))
            contenu = corps["messages"][-1]["content"]
            if isinstance(contenu, list):  # image d'essai : carré rouge en (0,70 ; 0,40)
                # Qwen répond en [x, y]
                return {"content": '[{"point": [700, 430], "label": "carré rouge"}]'}
            return {"content": "OK"}

        fs.chat = chat
        config = config_essai(fs.base, env={}, profil="gx10")
        ok = await verifier.verifier(config, ecrire=lignes.append)
    assert ok, lignes
    assert sum(ligne.strip().startswith("OK") for ligne in lignes) == 5
    assert {c for c, _ in fs.requetes} == {"/v1/chat/completions", "/v1/audio/transcriptions"}


async def test_verifier_signale_un_echec() -> None:
    verifier = charger_verifier()
    lignes: list[str] = []
    async with faux_serveur() as fs:
        fs.modeles = []  # le modèle configuré n'est pas servi
        config = config_essai(fs.base, env={}, profil="gx10")
        ok = await verifier.verifier(config, ecrire=lignes.append)
    assert not ok
    assert any("modèles non servis" in ligne for ligne in lignes)


def test_image_d_essai_png() -> None:
    png = charger_verifier().image_essai_png()
    assert png.startswith(b"\x89PNG\r\n\x1a\n") and b"IEND" in png


def test_definir_cle_enregistre_dans_env(tmp_path: Path) -> None:
    env_file = tmp_path / ".env"
    config = ConfigIA.charger(FICHIER_IA, fichier_env=env_file, fichier_choix=None, env={})
    assert not config.resume()["disponible"]
    config.definir_cle("ma-cle-secrete", "gemini")
    assert config.resume()["disponible"]
    assert config.cle("gemini") == "ma-cle-secrete"
    assert lire_env(env_file)["GEMINI_API_KEY"] == "ma-cle-secrete"

