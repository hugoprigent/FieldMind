"""Écoute : détection de parole sur signaux synthétiques (parole simulée + bruit de moteur),
durées, pré-roulement, mot d'éveil, enchaînement transcription → consigne."""

from __future__ import annotations

import asyncio
import dataclasses
from typing import Any

import numpy as np
import pytest
from test_ia_faux_serveur import (
    FS,
    Ecoute,
    blocs,
    bruit_moteur,
    config_essai,
    faux_serveur,
    melanger,
    parole,
)

from fieldmind.bus import Bus, S
from fieldmind.ia.config import ConfigVoix
from fieldmind.ia.voix import SUJET_NIVEAU, DetecteurParole, Segment, Voix, retirer_mot_eveil
from fieldmind.types import EchantillonAudio


def segments(signal: np.ndarray, config: ConfigVoix | None = None
             ) -> tuple[list[Segment], int]:
    detecteur = DetecteurParole(config or ConfigVoix())
    trouves: list[Segment] = []
    debuts = 0
    for bloc in blocs(signal):
        for nom, segment in detecteur.traiter(bloc):
            debuts += nom == "debut"
            if segment is not None:
                trouves.append(segment)
    return trouves, debuts


@pytest.mark.parametrize("graine", range(4))
def test_bruit_de_moteur_seul(graine: int) -> None:
    assert segments(melanger(bruit_moteur(6, graine=graine))) == ([], 0)


@pytest.mark.parametrize("graine", range(4))
def test_moteur_qui_demarre_ou_accelere(graine: int) -> None:
    assert segments(melanger(bruit_moteur(8, graine=graine, depart=2.0))) == ([], 0)
    assert segments(melanger(bruit_moteur(8, graine=graine, depart=2.0,
                                          gain_depart_db=25))) == ([], 0)
    assert segments(melanger(bruit_moteur(8, graine=graine, balayage=(2.0, 3.5, 1.7)))) == (
        [], 0)


def test_choc() -> None:
    fond = bruit_moteur(6, graine=4)
    fond[3 * FS : 3 * FS + 400] += np.random.default_rng(0).standard_normal(400) * 0.3
    assert segments(melanger(fond)) == ([], 0)


@pytest.mark.parametrize("niveau_db", [-26, -30, -33])
@pytest.mark.parametrize("graine", range(3))
def test_parole_dans_le_bruit_du_moteur(niveau_db: float, graine: int) -> None:
    """Parole de 1,4 s à partir de 2 s ; moteur à −32 dBFS (la parole peut être plus faible)."""
    trouves, debuts = segments(melanger(bruit_moteur(6, graine=graine),
                                        parole(1.4, niveau_db, graine=graine + 10), 2.0))
    assert debuts == 1 and len(trouves) == 1
    segment = trouves[0]
    # sous le bruit (−33 dBFS), une dernière syllabe faible peut ne pas compter : la traîne
    # de 350 ms la garde dans l'audio envoyé à la transcription
    assert (1.1 if niveau_db > -33 else 0.8) <= segment.duree_parole_s <= 1.6
    assert not segment.coupe
    # pré-roulement de 300 ms + parole + 350 ms de traîne
    assert 1.4 <= len(segment.pcm) / 2 / FS <= 2.2


def test_parole_apres_demarrage_du_moteur() -> None:
    trouves, _ = segments(melanger(bruit_moteur(8, graine=1, depart=1.0),
                                   parole(1.5, -24, graine=3), 4.0))
    assert len(trouves) == 1


def test_duree_minimale() -> None:
    assert segments(melanger(bruit_moteur(6, graine=2), parole(0.25, -26, graine=9), 2.0)) == (
        [], 0)
    trouves, _ = segments(melanger(bruit_moteur(6, graine=2), parole(0.45, -26, graine=9), 2.0))
    assert len(trouves) == 1


def test_duree_maximale_coupe_a_15_s() -> None:
    trouves, _ = segments(melanger(bruit_moteur(25, graine=2), parole(18, -26, graine=9), 2.0))
    assert trouves[0].coupe and 14.5 <= trouves[0].duree_parole_s <= 15.0
    assert len(trouves) == 2  # la suite de la phrase forme un second segment


def test_pre_roulement_garde_le_debut() -> None:
    signal = melanger(bruit_moteur(5, graine=3), parole(1.0, -26, graine=4), 2.0)
    segment = segments(signal)[0][0]
    debut = signal.tobytes().find(segment.pcm[:640])
    assert debut >= 0
    # le segment commence environ 300 ms avant le début de la parole (2,0 s)
    assert 1.6 <= debut / 2 / FS <= 1.8


def test_seuils_configurables() -> None:
    signal = melanger(bruit_moteur(6, graine=0), parole(1.4, -33, graine=10), 2.0)
    strict = dataclasses.replace(ConfigVoix(), seuil_entree_db=20.0)
    assert segments(signal, strict) == ([], 0)


def test_blocs_de_taille_quelconque() -> None:
    signal = melanger(bruit_moteur(5, graine=0), parole(1.2, -26, graine=2), 2.0)
    detecteur = DetecteurParole(ConfigVoix())
    fins = [e for i in range(0, signal.size, 1000)
            for e in detecteur.traiter(signal[i : i + 1000].tobytes()) if e[0] == "fin"]
    assert len(fins) == 1


def test_mot_eveil() -> None:
    assert retirer_mot_eveil("Rover, avance de 2 mètres", "rover") == "avance de 2 mètres"
    assert retirer_mot_eveil("eh rover tourne à gauche", "rover") == "tourne à gauche"
    assert retirer_mot_eveil("Robert, stop", "rover", ("robert",)) == "stop"
    assert retirer_mot_eveil("avance de 2 mètres", "rover") is None


# ------------------------------------------------------------------ écoute complète


class FauxTranscripteur:
    def __init__(self, textes: list[str], duree_s: float = 0.0) -> None:
        self.textes = textes
        self.duree_s = duree_s
        self.en_cours = 0
        self.max_simultanes = 0
        self.recus: list[int] = []

    async def transcrire(self, pcm16_mono: bytes, frequence_hz: int = 16000) -> str:
        self.en_cours += 1
        self.max_simultanes = max(self.max_simultanes, self.en_cours)
        self.recus.append(len(pcm16_mono))
        await asyncio.sleep(self.duree_s)
        self.en_cours -= 1
        return self.textes.pop(0) if self.textes else ""


async def attendre(condition: Any, delai_s: float = 3.0) -> None:
    fin = asyncio.get_running_loop().time() + delai_s
    while not condition():
        if asyncio.get_running_loop().time() > fin:
            raise AssertionError("délai dépassé")
        await asyncio.sleep(0.01)


def publier_signal(bus: Bus, signal: np.ndarray) -> None:
    for bloc in blocs(signal):
        bus.publier(S.AUDIO, EchantillonAudio(t=0.0, pcm=bloc, frequence_hz=FS))


async def test_ecoute_jusqu_a_la_consigne() -> None:
    async with faux_serveur() as fs:
        bus = Bus()
        ecoute = Ecoute(bus)
        niveaux: list[dict[str, Any]] = []
        bus.abonner(SUJET_NIVEAU, niveaux.append)
        consignes: list[str] = []

        async def sur_consigne(texte: str) -> None:
            consignes.append(texte)

        config = config_essai(fs.base)
        voix = Voix(bus, None, config, FauxTranscripteur(["avance de deux mètres"]),
                    sur_consigne)
        voix.activer(True)
        assert voix.etat()["active"]
        publier_signal(bus, melanger(bruit_moteur(5, graine=1), parole(1.2, -26, graine=5), 1.5))
        await attendre(lambda: consignes)
        voix.activer(False)
        await voix.fermer()
    assert consignes == ["avance de deux mètres"]
    assert ecoute.evenements() == ["debut_parole", "fin_parole", "transcription"]
    assert ecoute.voix[2]["texte"] == "avance de deux mètres"
    assert all(m["type"] == "voix" for m in ecoute.voix)
    assert niveaux and max(n["niveau"] for n in niveaux) > 0.3
    assert len(niveaux) <= 5 * 10 + 2  # 10 Hz au plus sur 5 s


async def test_une_transcription_a_la_fois() -> None:
    async with faux_serveur() as fs:
        bus = Bus()
        consignes: list[str] = []

        async def sur_consigne(texte: str) -> None:
            consignes.append(texte)

        transcripteur = FauxTranscripteur(["avance", "stop"], duree_s=0.2)
        voix = Voix(bus, None, config_essai(fs.base), transcripteur, sur_consigne)
        voix.activer(True)
        fond = bruit_moteur(7, graine=2)
        signal = melanger(fond, parole(1.0, -26, graine=1), 1.0)
        signal = signal + melanger(np.zeros_like(fond), parole(1.0, -26, graine=2), 3.5)
        publier_signal(bus, signal)
        await attendre(lambda: len(consignes) == 2)
        await voix.fermer()
    assert transcripteur.max_simultanes == 1
    assert consignes == ["avance", "stop"]


async def test_mot_eveil_exige() -> None:
    async with faux_serveur() as fs:
        bus = Bus()
        ecoute = Ecoute(bus)
        consignes: list[str] = []

        async def sur_consigne(texte: str) -> None:
            consignes.append(texte)

        def retouche(d: dict[str, Any]) -> None:
            d["voix"]["mot_eveil"] = "rover"

        voix = Voix(bus, None, config_essai(fs.base, retouche=retouche),
                    FauxTranscripteur(["tu viens manger ?", "Rover, tourne à gauche"]),
                    sur_consigne)
        voix.activer(True)
        fond = bruit_moteur(7, graine=3)
        signal = melanger(fond, parole(1.0, -26, graine=1), 1.0)
        signal = signal + melanger(np.zeros_like(fond), parole(1.0, -26, graine=2), 3.5)
        publier_signal(bus, signal)
        await attendre(lambda: consignes)
        await voix.fermer()
    assert consignes == ["tourne à gauche"]
    ignores = [m for m in ecoute.voix if m["evenement"] == "ignore"]
    assert ignores[0]["raison"] == "sans mot d'éveil"


async def test_transcription_vide_ou_en_erreur() -> None:
    async with faux_serveur() as fs:
        bus = Bus()
        ecoute = Ecoute(bus)
        consignes: list[str] = []

        async def sur_consigne(texte: str) -> None:
            consignes.append(texte)

        # profil « aucun » : l'écoute démarre (vumètre) mais signale l'absence de transcription
        config = config_essai(fs.base, profil="aucun")
        from fieldmind.ia.transcription import Transcripteur

        voix = Voix(bus, None, config, Transcripteur(config), sur_consigne)
        voix.activer(True)
        publier_signal(bus, melanger(bruit_moteur(4, graine=1), parole(1.0, -26, graine=5), 1.0))
        await attendre(lambda: ecoute.evenements().count("erreur") == 2)
        await voix.fermer()
    assert consignes == []
    assert "aucun modèle" in ecoute.voix[0]["message"]


async def test_inactive_ignore_l_audio() -> None:
    async with faux_serveur() as fs:
        bus = Bus()
        ecoute = Ecoute(bus)

        async def sur_consigne(texte: str) -> None:
            raise AssertionError("pas de consigne attendue")

        voix = Voix(bus, None, config_essai(fs.base), FauxTranscripteur(["x"]), sur_consigne)
        publier_signal(bus, melanger(bruit_moteur(3, graine=1), parole(1.0, -26, graine=5), 1.0))
        await asyncio.sleep(0.05)
        await voix.fermer()
    assert ecoute.voix == [] and not voix.etat()["active"]
