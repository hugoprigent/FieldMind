"""Journal JSONL des sessions (événements du cerveau et consignes), un fichier par jour, et
preuves des analyses de la caméra (image analysée + résultat), une paire de fichiers par analyse.

Rien de secret n'y est écrit : ni clé d'API, ni adresse IP. Dossier ignoré par git.
"""

from __future__ import annotations

import datetime as dt
import json
import threading
from pathlib import Path
from typing import Any


class JournalJsonl:
    def __init__(self, dossier: Path) -> None:
        self.dossier = dossier
        self._verrou = threading.Lock()

    def ecrire(self, session: str, evenement: dict[str, Any]) -> None:
        maintenant = dt.datetime.now(dt.UTC)
        ligne = {"horodatage": maintenant.isoformat(timespec="milliseconds"), "session": session}
        ligne.update(evenement)
        chemin = self.dossier / f"demo-{maintenant:%Y-%m-%d}.jsonl"
        with self._verrou:
            self.dossier.mkdir(parents=True, exist_ok=True)
            with chemin.open("a", encoding="utf-8") as fichier:
                fichier.write(json.dumps(ligne, ensure_ascii=False, separators=(",", ":")) + "\n")

    def ecrire_preuve(
        self, session: str, analyse: int, jpeg: bytes, resultat: dict[str, Any]
    ) -> Path:
        """Image analysée et résultat, dans preuves/AAAA-MM-JJ/ ; renvoie le chemin de l'image."""
        maintenant = dt.datetime.now(dt.UTC)
        dossier = self.dossier / "preuves" / f"{maintenant:%Y-%m-%d}"
        base = f"{maintenant:%H%M%S}-{session}-{analyse:04d}"
        with self._verrou:
            dossier.mkdir(parents=True, exist_ok=True)
            (dossier / f"{base}.jpg").write_bytes(jpeg)
            (dossier / f"{base}.json").write_text(
                json.dumps(resultat, ensure_ascii=False, indent=1), encoding="utf-8"
            )
        return dossier / f"{base}.jpg"
