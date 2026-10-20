"""Écoute du micro de la webcam : détection de parole → transcription → consigne de l'agent.

    voix = Voix(bus, profil, config_ia, Transcripteur(config_ia),
                lambda texte: agent.consigne(texte, origine="voix"))
    voix.activer(True)        # depuis la boucle asyncio (message `voix` du poste)
    voix.etat()               # champ `voix` du message `etat` : {"active", "parle", "niveau"}

Détection de parole (`DetecteurParole`), pensée pour le bruit des moteurs du rover :
- trames de 20 ms, spectre de la bande de la parole (250–3500 Hz par défaut : le roulement et le
  grondement des moteurs sont en dessous) ;
- plancher de bruit adaptatif **par fréquence** : un sifflement de moteur stable est absorbé, la
  parole se mesure en rapport moyen au plancher (dB) ;
- seuils d'entrée et de sortie distincts (hystérésis), 60 ms au-dessus du seuil pour ouvrir ;
- **modulation syllabique** exigée : la parole varie de plusieurs dB d'une syllabe à l'autre, un
  moteur qui démarre ou accélère donne un niveau stable ; un tel segment est abandonné et son
  bruit absorbé dans le plancher ;
- durées minimale (0,4 s) et maximale (15 s), pré-roulement de 300 ms, fin après 0,7 s de silence.

Une transcription à la fois (file de deux segments au plus, le plus ancien est jeté), puis
`sur_consigne(texte)` dans une tâche à part : un « stop » dit pendant que l'agent réfléchit est
transcrit et traité tout de suite. Mot d'éveil facultatif (`[voix] mot_eveil`).
"""

from __future__ import annotations

import asyncio
import collections
import logging
import math
import threading
import time
import unicodedata
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np

from fieldmind.bus import Bus, S
from fieldmind.ia.config import ConfigIA, ConfigVoix
from fieldmind.ia.fournisseurs import ErreurIA
from fieldmind.ia.messages import publier_voix
from fieldmind.types import EchantillonAudio

journal = logging.getLogger("fieldmind.ia.voix")

SUJET_NIVEAU = "voix.niveau"
"""Sujet du bus du niveau sonore (dict {"t", "niveau", "parle"}, `niveau_hz` fois par seconde),
pour le vumètre (à ajouter à `S` : S.VOIX_NIVEAU)."""
DUREE_TRAME_S = 0.02
TAILLE_FFT = 512
TRAMES_INIT = 10  # 200 ms pour amorcer le plancher de bruit
TRAMES_ENTREE = 3  # 60 ms au-dessus du seuil d'entrée
CONFIRMATION_MAX_S = 1.2  # sans modulation de parole après ce délai : bruit, segment abandonné
FENETRE_MODULATION = 9  # trames (180 ms) de la moyenne glissante retirée avant l'écart-type
FENETRE_ACTIVITE = 15  # trames (300 ms) : une trame n'est active que si le score y varie
VARIATION_MIN_DB = 1.5  # écart-type minimal du score sur FENETRE_ACTIVITE
PLANCHER_ABSOLU = 1e-9  # puissance par case : sous ≈ −90 dBFS, rien n'est de la parole
SUIVI = 0.03  # suivi du plancher hors parole, par trame (≈ 0,7 s)
DESCENTE = 0.2  # quand le bruit baisse nettement (moteur coupé)
TRAINE_S = 0.35  # audio gardé après la dernière trame active (fin de mot faible)


@dataclass(slots=True)
class Segment:
    pcm: bytes  # PCM 16 bits mono, pré-roulement compris
    frequence_hz: int
    duree_parole_s: float  # du début détecté à la dernière trame active
    modulation_db: float
    coupe: bool = False  # durée maximale atteinte


class DetecteurParole:
    """Détection de parole sur des blocs PCM 16 bits mono de taille quelconque.

    `traiter(pcm)` renvoie les événements : ("debut", None) quand une parole est confirmée,
    ("fin", Segment) quand elle se termine. Synchrone, sans verrou (un seul fil à la fois).

    Le plancher de bruit (par case de fréquence) suit le bruit hors parole avec un retard de
    TRAMES_ENTREE trames, pour que le début d'un mot n'y entre pas ; il est figé pendant un
    segment. Le score d'une trame est le rapport moyen puissance / plancher sur la bande (dB)."""

    def __init__(self, config: ConfigVoix, frequence_hz: int = 16000) -> None:
        self.config = config
        self.frequence_hz = frequence_hz
        self.n_trame = max(1, round(frequence_hz * DUREE_TRAME_S))
        self._fenetre = np.hanning(self.n_trame).astype(np.float64)
        frequences = np.fft.rfftfreq(TAILLE_FFT, 1.0 / frequence_hz)
        bas, haut = config.bande_hz
        self._bande = (frequences >= bas) & (frequences <= min(haut, frequence_hz / 2 - 1))
        self._reste = np.zeros(0, np.int16)
        self._bruit: np.ndarray | None = None
        self._init: list[np.ndarray] = []
        self._retard: collections.deque[np.ndarray] = collections.deque()
        taille_pre = round(config.pre_roulement_s / DUREE_TRAME_S) + TRAMES_ENTREE
        self._pre: collections.deque[tuple[np.ndarray, np.ndarray, float]] = collections.deque(
            maxlen=max(TRAMES_ENTREE, taille_pre))
        self._pre_segment: list[np.ndarray] = []
        self.niveau = 0.0  # 0..1, pour le vumètre
        self.score_db = 0.0
        self.en_parole = False  # segment ouvert (confirmé ou non)
        self.confirme = False
        self._au_dessus = 0
        self._trames: list[np.ndarray] = []
        self._puissances: list[np.ndarray] = []
        self._scores: list[float] = []
        self._derniere_active = 0

    # ------------------------------------------------------------------ mesures

    def _mesurer(self, trame: np.ndarray) -> np.ndarray:
        x = trame.astype(np.float64) / 32768.0
        rms = math.sqrt(float(np.mean(x * x))) if x.size else 0.0
        niveau = min(1.0, max(0.0, (20 * math.log10(rms + 1e-9) + 60.0) / 60.0))
        self.niveau = max(niveau, self.niveau * 0.85)  # retombée douce du vumètre
        spectre = np.fft.rfft(x * self._fenetre, TAILLE_FFT)
        return (spectre.real ** 2 + spectre.imag ** 2)[self._bande]

    def _score(self, puissance: np.ndarray) -> float:
        assert self._bruit is not None
        rapport = float(np.mean(puissance / self._bruit))
        return 10.0 * math.log10(max(rapport, 1e-6))

    def _adapter(self, puissance: np.ndarray) -> None:
        assert self._bruit is not None
        alpha = DESCENTE if puissance.sum() < 0.7 * self._bruit.sum() else SUIVI
        self._bruit += alpha * (puissance - self._bruit)
        np.maximum(self._bruit, PLANCHER_ABSOLU, out=self._bruit)

    @staticmethod
    def modulation(scores: list[float]) -> float:
        """Écart-type (dB) du score autour de sa moyenne glissante : variation syllabique."""
        if len(scores) < FENETRE_MODULATION + 2:
            return 0.0
        s = np.asarray(scores, np.float64)
        noyau = np.ones(FENETRE_MODULATION) / FENETRE_MODULATION
        moyenne = np.convolve(s, noyau, mode="valid")
        centre = s[FENETRE_MODULATION // 2 : FENETRE_MODULATION // 2 + moyenne.size]
        return float(np.std(centre - moyenne))

    # ------------------------------------------------------------------ automate

    def traiter(self, pcm: bytes | np.ndarray) -> list[tuple[str, Segment | None]]:
        echantillons = (np.frombuffer(pcm, np.int16) if isinstance(pcm, bytes | bytearray)
                        else np.asarray(pcm, np.int16))
        donnees = np.concatenate((self._reste, echantillons))
        n = self.n_trame
        complet = (donnees.size // n) * n
        self._reste = donnees[complet:].copy()
        evenements: list[tuple[str, Segment | None]] = []
        for i in range(0, complet, n):
            evenements.extend(self._trame(donnees[i : i + n]))
        return evenements

    def _trame(self, trame: np.ndarray) -> list[tuple[str, Segment | None]]:
        puissance = self._mesurer(trame)
        if self._bruit is None:
            self._init.append(puissance)
            if len(self._init) >= TRAMES_INIT:
                self._bruit = np.maximum(np.median(np.asarray(self._init), axis=0),
                                         PLANCHER_ABSOLU)
                self._init = []
            return []
        score = self._score(puissance)
        self.score_db = score
        c = self.config
        if not self.en_parole:
            self._retard.append(puissance)
            if len(self._retard) > TRAMES_ENTREE:
                self._adapter(self._retard.popleft())
            self._pre.append((trame, puissance, score))
            self._au_dessus = self._au_dessus + 1 if score >= c.seuil_entree_db else 0
            if self._au_dessus >= TRAMES_ENTREE:
                self._ouvrir()
            return []
        self._trames.append(trame)
        self._puissances.append(puissance)
        self._scores.append(score)
        indice = len(self._scores) - 1
        recents = self._scores[-FENETRE_ACTIVITE:]
        varie = len(recents) < FENETRE_ACTIVITE or float(np.std(recents)) >= VARIATION_MIN_DB
        if score >= c.seuil_sortie_db and varie:
            self._derniere_active = indice
        evenements: list[tuple[str, Segment | None]] = []
        duree_active = (self._derniere_active + 1) * DUREE_TRAME_S
        if not self.confirme and duree_active >= c.duree_min_s and self.modulation(
                self._scores[: self._derniere_active + 1]) >= c.modulation_min_db:
            self.confirme = True
            evenements.append(("debut", None))
        silence = (indice - self._derniere_active) * DUREE_TRAME_S
        if not self.confirme and len(self._scores) * DUREE_TRAME_S >= CONFIRMATION_MAX_S:
            self._absorber()  # niveau élevé mais stable : bruit (moteur qui démarre…)
        elif silence >= c.silence_fin_s:
            evenements.extend(self._fermer(coupe=False))
        elif len(self._scores) * DUREE_TRAME_S >= c.duree_max_s:
            evenements.extend(self._fermer(coupe=True))
        return evenements

    def _ouvrir(self) -> None:
        self.en_parole = True
        self.confirme = False
        recentes = list(self._pre)
        entree = recentes[-TRAMES_ENTREE:]  # les trames qui ont passé le seuil : de la parole
        self._pre_segment = [t for t, _, _ in recentes[:-TRAMES_ENTREE]]
        self._trames = [t for t, _, _ in entree]
        self._puissances = [p for _, p, _ in entree]
        self._scores = [s for _, _, s in entree]
        self._derniere_active = len(self._scores) - 1
        self._retard.clear()  # le début de la parole n'entre pas dans le plancher

    def _absorber(self) -> None:
        """Segment sans parole : le bruit récent devient le plancher."""
        recentes = np.asarray(self._puissances[-TRAMES_INIT:])
        self._bruit = np.maximum(np.median(recentes, axis=0), PLANCHER_ABSOLU)
        self._reinitialiser()

    def _fermer(self, coupe: bool) -> list[tuple[str, Segment | None]]:
        c = self.config
        fin = len(self._trames) if coupe else min(
            len(self._trames), self._derniere_active + 1 + round(TRAINE_S / DUREE_TRAME_S))
        duree = (self._derniere_active + 1) * DUREE_TRAME_S
        modulation = self.modulation(self._scores[: self._derniere_active + 1])
        confirme = self.confirme
        trames = self._pre_segment + self._trames[:fin]
        # La fin du segment (silence ou bruit stable) redevient du bruit pour le plancher.
        queue = self._puissances[fin:]
        self._reinitialiser()
        for puissance in queue:
            self._adapter(puissance)
        if not confirme or duree < c.duree_min_s:
            return []
        pcm = np.concatenate(trames).astype("<i2").tobytes()
        return [("fin", Segment(pcm, self.frequence_hz, round(duree, 2), round(modulation, 2),
                                coupe))]

    def _reinitialiser(self) -> None:
        self.en_parole = False
        self.confirme = False
        self._trames = []
        self._puissances = []
        self._scores = []
        self._pre_segment = []
        self._pre.clear()
        self._retard.clear()
        self._au_dessus = 0
        self._derniere_active = 0


# ------------------------------------------------------------------ mot d'éveil


def _normaliser(texte: str) -> str:
    decompose = unicodedata.normalize("NFD", texte.lower())
    return "".join(c if c.isalnum() else " " for c in decompose
                   if not unicodedata.combining(c))


def retirer_mot_eveil(texte: str, mot: str, variantes: tuple[str, ...] = ()) -> str | None:
    """Consigne sans le mot d'éveil s'il figure parmi les trois premiers mots, sinon None."""
    formes = {_normaliser(m).strip() for m in (mot, *variantes) if m.strip()}
    mots = texte.split()
    for i, brut in enumerate(mots[:3]):
        if _normaliser(brut).strip() in formes:
            return " ".join(mots[i + 1 :]).lstrip(",.;:!? ").strip()
    return None


# ------------------------------------------------------------------ écoute


class Transcripteur(Protocol):
    async def transcrire(self, pcm16_mono: bytes, frequence_hz: int = 16000) -> str: ...


class Voix:
    SEGMENTS_EN_ATTENTE = 2

    def __init__(self, bus: Bus, profil: Any, config_ia: ConfigIA, transcripteur: Transcripteur,
                 sur_consigne: Callable[[str], Awaitable[Any]],
                 boucle: asyncio.AbstractEventLoop | None = None) -> None:
        self.bus = bus
        self.profil = profil
        self.config_ia = config_ia
        self.transcripteur = transcripteur
        self.sur_consigne = sur_consigne
        self._boucle = boucle
        self._verrou = threading.Lock()
        self._detecteur: DetecteurParole | None = None
        self._desabonner: Callable[[], None] | None = None
        self._file: asyncio.Queue[Segment] | None = None
        self._travail: asyncio.Task[None] | None = None
        self._taches: set[asyncio.Task[Any]] = set()
        self._dernier_niveau = 0.0
        self.active = False
        self.transcription_en_cours = False

    # ------------------------------------------------------------------ commande

    def activer(self, active: bool) -> None:
        """Démarre ou arrête l'écoute. À appeler depuis la boucle asyncio (ou donner `boucle`)."""
        if active == self.active:
            return
        if not active:
            self.active = False
            if self._desabonner is not None:
                self._desabonner()
                self._desabonner = None
            if self._travail is not None:
                self._travail.cancel()
                self._travail = None
            with self._verrou:
                self._detecteur = None
            self._publier_niveau(0.0, False, force=True)
            return
        if self._boucle is None:
            self._boucle = asyncio.get_running_loop()
        disponible, raison = self.config_ia.disponibilite("transcription")
        if not disponible:
            publier_voix(self.bus, "erreur", message=f"pas de transcription : {raison}")
        self._file = asyncio.Queue(self.SEGMENTS_EN_ATTENTE)
        self._travail = self._boucle.create_task(self._travailler())
        self.active = True
        self._desabonner = self.bus.abonner(S.AUDIO, self.traiter_bloc)

    @property
    def occupee(self) -> bool:
        return self.transcription_en_cours

    def etat(self) -> dict[str, Any]:
        """Champ `voix` du message `etat`."""
        detecteur = self._detecteur
        return {"active": self.active,
                "parle": bool(detecteur and detecteur.confirme),
                "niveau": round(detecteur.niveau, 3) if detecteur else 0.0}

    async def fermer(self) -> None:
        self.activer(False)
        for tache in list(self._taches):
            tache.cancel()

    # ------------------------------------------------------------------ audio (fil du micro)

    def traiter_bloc(self, echantillon: EchantillonAudio) -> None:
        """Rappel du bus (fil du micro) : quelques dizaines de µs par bloc de 20 ms."""
        if not self.active:
            return
        with self._verrou:
            if (self._detecteur is None
                    or self._detecteur.frequence_hz != echantillon.frequence_hz):
                self._detecteur = DetecteurParole(self.config_ia.voix, echantillon.frequence_hz)
            detecteur = self._detecteur
            evenements = detecteur.traiter(echantillon.pcm)
            niveau, parle = detecteur.niveau, detecteur.confirme
        self._publier_niveau(niveau, parle)
        for nom, segment in evenements:
            if nom == "debut":
                publier_voix(self.bus, "debut_parole")
            elif segment is not None:
                publier_voix(self.bus, "fin_parole", duree_s=segment.duree_parole_s,
                             coupe=segment.coupe)
                self._deposer(segment)

    def _publier_niveau(self, niveau: float, parle: bool, force: bool = False) -> None:
        maintenant = time.monotonic()
        periode = 1.0 / max(self.config_ia.voix.niveau_hz, 0.1)
        if force or maintenant - self._dernier_niveau >= periode:
            self._dernier_niveau = maintenant
            self.bus.publier(SUJET_NIVEAU, {"t": time.time(), "niveau": round(niveau, 3),
                                            "parle": parle})

    def _deposer(self, segment: Segment) -> None:
        boucle, file = self._boucle, self._file
        if boucle is None or file is None:
            return

        def deposer() -> None:
            if file.full():  # on garde les plus récents (un « stop » dit en dernier)
                file.get_nowait()
            file.put_nowait(segment)

        try:
            boucle.call_soon_threadsafe(deposer)
        except RuntimeError:  # boucle fermée
            pass

    # ------------------------------------------------------------------ transcription (boucle)

    async def _travailler(self) -> None:
        assert self._file is not None
        file = self._file
        while True:
            segment = await file.get()
            self.transcription_en_cours = True
            try:
                await self._traiter_segment(segment)
            except asyncio.CancelledError:
                raise
            except Exception:
                journal.exception("traitement d'un segment de parole")
            finally:
                self.transcription_en_cours = False

    async def _traiter_segment(self, segment: Segment) -> None:
        try:
            texte = await self.transcripteur.transcrire(segment.pcm, segment.frequence_hz)
        except ErreurIA as erreur:
            publier_voix(self.bus, "erreur", message=str(erreur))
            return
        if not texte:
            publier_voix(self.bus, "ignore", texte="", raison="pas de parole reconnue")
            return
        publier_voix(self.bus, "transcription", texte=texte)
        c = self.config_ia.voix
        if c.mot_eveil:
            reste = retirer_mot_eveil(texte, c.mot_eveil, c.variantes_mot_eveil)
            if reste is None:
                publier_voix(self.bus, "ignore", texte=texte, raison="sans mot d'éveil")
                return
            if not reste:
                publier_voix(self.bus, "ignore", texte=texte, raison="mot d'éveil seul")
                return
            texte = reste
        tache = asyncio.get_running_loop().create_task(self._consigne(texte))
        self._taches.add(tache)
        tache.add_done_callback(self._taches.discard)

    async def _consigne(self, texte: str) -> None:
        try:
            await self.sur_consigne(texte)
        except Exception as erreur:
            journal.exception("consigne vocale en erreur")
            publier_voix(self.bus, "erreur", message=f"consigne non traitée : {erreur}")
