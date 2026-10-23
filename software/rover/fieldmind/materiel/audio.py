"""Micro d'une webcam (ALSA) : PCM 16 bits mono 16 kHz, en blocs de 20 ms sur le bus.

`arecord` (paquet alsa-utils, présent sur Raspberry Pi OS) lit la carte son USB de la webcam ;
aucune bibliothèque Python n'est nécessaire. La capture ne tourne que quand le mode vocal est
actif (`activer(True)`).
"""

from __future__ import annotations

import re
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

from fieldmind.bus import Bus, S
from fieldmind.types import EchantillonAudio

DUREE_BLOC_S = 0.02


_CACHE_CARTES: tuple[float, list[dict[str, str]]] = (-1e9, [])


def cartes_usb() -> list[dict[str, str]]:
    """Cartes de capture USB d'après /proc/asound (webcams, micros USB), relues au plus toutes
    les 5 s : l'état du poste les demande 10 fois par seconde."""
    global _CACHE_CARTES
    quand, cartes = _CACHE_CARTES
    if time.monotonic() - quand < 5.0:
        return cartes
    cartes = _lire_cartes()
    _CACHE_CARTES = (time.monotonic(), cartes)
    return cartes


def _lire_cartes() -> list[dict[str, str]]:
    cartes = []
    try:
        texte = Path("/proc/asound/cards").read_text()
    except OSError:
        return cartes
    for m in re.finditer(r"^\s*(\d+)\s+\[(\S+)\s*\]:\s*(\S+)\s+-\s+(.+)$", texte, re.M):
        numero, ident, pilote, nom = m.groups()
        if "USB" not in pilote and "usb" not in nom.lower():
            continue
        if not list(Path(f"/proc/asound/card{numero}").glob("pcm*c")):
            continue  # pas de voie de capture
        cartes.append({"numero": numero, "id": ident, "nom": nom.strip()})
    return cartes


class PiloteMicro:
    def __init__(self, bus: Bus, peripherique: str = "", frequence_hz: int = 16000) -> None:
        self.bus = bus
        self.peripherique_demande = peripherique
        self.frequence = frequence_hz
        self.peripherique: str | None = None
        self.nom: str | None = None
        self.erreur: str | None = None
        self._processus: subprocess.Popen[bytes] | None = None
        self._fil: threading.Thread | None = None
        self._verrou = threading.Lock()

    @property
    def actif(self) -> bool:
        return self._processus is not None and self._processus.poll() is None

    def disponible(self) -> bool:
        return shutil.which("arecord") is not None and bool(
            self.peripherique_demande or cartes_usb())

    def resume(self) -> dict[str, Any]:
        cartes = cartes_usb()
        return {"ok": self.actif or bool(cartes), "actif": self.actif,
                "nom": self.nom or (cartes[0]["nom"] if cartes else None), "erreur": self.erreur}

    def activer(self, actif: bool) -> None:
        with self._verrou:
            if actif and not self.actif:
                self._lancer()
            elif not actif and self._processus is not None:
                self._processus.terminate()
                self._processus = None

    def _lancer(self) -> None:
        peripherique = self.peripherique_demande
        if not peripherique:
            cartes = cartes_usb()
            if not cartes:
                self.erreur = "aucun micro USB"
                self.bus.journaliser("micro", self.erreur, "alerte")
                return
            peripherique = f"plughw:CARD={cartes[0]['id']},DEV=0"
            self.nom = cartes[0]["nom"]
        self.peripherique = peripherique
        try:
            self._processus = subprocess.Popen(
                ["arecord", "-q", "-D", peripherique, "-f", "S16_LE", "-r", str(self.frequence),
                 "-c", "1", "-t", "raw", "--buffer-time=200000"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except OSError as erreur:
            self.erreur = str(erreur)
            self.bus.journaliser("micro", f"arecord impossible : {erreur}", "erreur")
            return
        self.erreur = None
        self.bus.journaliser("micro", f"écoute sur {self.nom or peripherique}", "succes")
        self._fil = threading.Thread(target=self._lire, args=(self._processus,), name="micro",
                                     daemon=True)
        self._fil.start()

    def _lire(self, processus: subprocess.Popen[bytes]) -> None:
        taille = int(self.frequence * DUREE_BLOC_S) * 2
        assert processus.stdout is not None
        while processus.poll() is None:
            bloc = processus.stdout.read(taille)
            if not bloc:
                break
            self.bus.publier(S.AUDIO, EchantillonAudio(t=time.monotonic(), pcm=bloc,
                                                       frequence_hz=self.frequence))
        if processus is self._processus:  # arrêt non demandé
            message = (processus.stderr.read().decode(errors="replace").strip()
                       if processus.stderr else "")
            self.erreur = message or "capture interrompue"
            self.bus.journaliser("micro", f"capture arrêtée : {self.erreur}", "alerte")
            self._processus = None
