"""Boîte noire : enregistre ce que le rover mesure et décide, pour comprendre un essai après coup.

Fichiers `~/fieldmind/enregistrements/AAAAMMJJ_HHMMSS.jsonl.gz`, une ligne JSON par message :
`{"t": temps monotone, "s": sujet, "v": valeur}`. Sujets : lidar (10 Hz), centrale (25 Hz),
odométrie (10 Hz), commande (20 Hz), sécurité (à chaque changement de raisons, sinon 2 Hz),
mode, ESP (2 Hz), GPS, mission, journal, chat. Pas d'images (trop lourd) : les photos sont à
part. Environ 40 Mo par heure ; les plus anciens fichiers sont effacés au-delà de 1 Go.

Relecture : `python -m fieldmind.enregistreur <fichier>` (résumé : durée, arrêts de sécurité et
leurs raisons, distance parcourue, défauts) ou `--csv` pour la trajectoire et les commandes.
"""

from __future__ import annotations

import argparse
import base64
import gzip
import json
import math
import queue
import threading
import time
from collections import Counter
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any

import numpy as np

from fieldmind.bus import Bus, S

DOSSIER = Path("~/fieldmind/enregistrements").expanduser()
TAILLE_MAX_OCTETS = 1_000_000_000
CADENCES = {S.SCAN: 0.1, S.IMU: 0.04, S.ODOM: 0.1, S.COMMANDE: 0.05, S.ESP: 0.5, S.GNSS: 0.5,
            S.SECURITE: 0.5, S.MODE: 0.0, S.MISSION: 0.0, S.JOURNAL: 0.0, S.CHAT: 0.0}


def _jsonable(valeur: Any) -> Any:
    if is_dataclass(valeur) and not isinstance(valeur, type):
        return {k: _jsonable(v) for k, v in asdict(valeur).items()}
    if isinstance(valeur, np.ndarray):
        if valeur.dtype.kind == "f" and valeur.size > 16:
            return {"f16": base64.b64encode(valeur.astype("<f2").tobytes()).decode(),
                    "forme": list(valeur.shape)}
        return valeur.tolist()
    if isinstance(valeur, dict):
        return {str(k): _jsonable(v) for k, v in valeur.items()}
    if isinstance(valeur, (list, tuple)):
        return [_jsonable(v) for v in valeur]
    if isinstance(valeur, float) and not math.isfinite(valeur):
        return None
    if isinstance(valeur, (np.floating, np.integer)):
        return valeur.item()
    return valeur


class Enregistreur:
    def __init__(self, bus: Bus, dossier: Path = DOSSIER) -> None:
        self.bus = bus
        self.dossier = dossier
        self._file: queue.Queue[tuple[float, str, Any]] = queue.Queue(5000)
        self._derniers: dict[str, float] = {}
        self._raisons: tuple[str, ...] = ()
        self._actif = False
        self.chemin: Path | None = None
        self.pertes = 0
        self._fil = threading.Thread(target=self._ecrire, name="enregistreur", daemon=True)

    def demarrer(self) -> None:
        self.dossier.mkdir(parents=True, exist_ok=True)
        self._menage()
        self.chemin = self.dossier / f"{time.strftime('%Y%m%d_%H%M%S')}.jsonl.gz"
        self._actif = True
        for sujet in CADENCES:
            self.bus.abonner(sujet, lambda v, s=sujet: self._recevoir(s, v))
        self._fil.start()
        self.bus.journaliser("enregistreur", f"boîte noire : {self.chemin.name}")

    def arreter(self) -> None:
        self._actif = False

    def _recevoir(self, sujet: str, valeur: Any) -> None:
        if not self._actif:
            return
        t = time.monotonic()
        if sujet == S.SECURITE:
            raisons = tuple(getattr(valeur, "raisons", ()))
            change = raisons != self._raisons
            self._raisons = raisons
            if not change and t - self._derniers.get(sujet, -1e9) < CADENCES[sujet]:
                return
        elif t - self._derniers.get(sujet, -1e9) < CADENCES[sujet]:
            return
        self._derniers[sujet] = t
        try:
            self._file.put_nowait((t, sujet, valeur))
        except queue.Full:
            self.pertes += 1

    def _ecrire(self) -> None:
        assert self.chemin is not None
        with gzip.open(self.chemin, "at", encoding="utf-8", compresslevel=5) as f:
            dernier_vidage = time.monotonic()
            while self._actif or not self._file.empty():
                try:
                    t, sujet, valeur = self._file.get(timeout=0.5)
                except queue.Empty:
                    continue
                try:
                    f.write(json.dumps({"t": round(t, 4), "s": sujet, "v": _jsonable(valeur)},
                                       separators=(",", ":"), ensure_ascii=False) + "\n")
                except (TypeError, ValueError):
                    continue
                if time.monotonic() - dernier_vidage > 5:  # rien de perdu si le Pi s'éteint
                    f.flush()
                    dernier_vidage = time.monotonic()

    def _menage(self) -> None:
        fichiers = sorted(self.dossier.glob("*.jsonl.gz"))
        total = sum(f.stat().st_size for f in fichiers)
        while fichiers and total > TAILLE_MAX_OCTETS:
            ancien = fichiers.pop(0)
            total -= ancien.stat().st_size
            ancien.unlink()


# ---------------------------------------------------------------- relecture


def lire(chemin: Path) -> list[dict[str, Any]]:
    lignes = []
    with gzip.open(chemin, "rt", encoding="utf-8") as f:
        for ligne in f:
            try:
                lignes.append(json.loads(ligne))
            except ValueError:
                break  # fichier coupé net (rover éteint) : on garde ce qui précède
    return lignes


def resume(lignes: list[dict[str, Any]]) -> dict[str, Any]:
    if not lignes:
        return {"messages": 0}
    sujets = Counter(m["s"] for m in lignes)
    distance, precedente = 0.0, None
    arrets: Counter[str] = Counter()
    raisons_precedentes: set[str] = set()
    vitesse_max = 0.0
    for m in lignes:
        if m["s"] == S.ODOM:
            pose = m["v"]["pose"]
            if precedente is not None:
                distance += math.hypot(pose["x"] - precedente["x"], pose["y"] - precedente["y"])
            precedente = pose
            vitesse_max = max(vitesse_max, abs(m["v"].get("vitesse", 0.0)))
        elif m["s"] == S.SECURITE:
            raisons = set(m["v"].get("raisons", []))
            for r in raisons - raisons_precedentes:
                arrets[r] += 1
            raisons_precedentes = raisons
    return {
        "messages": len(lignes), "duree_s": round(lignes[-1]["t"] - lignes[0]["t"], 1),
        "sujets": dict(sujets), "distance_m": round(distance, 2),
        "vitesse_max_m_s": round(vitesse_max, 2), "apparitions_des_raisons": dict(arrets),
        "erreurs": [m["v"]["message"] for m in lignes
                    if m["s"] == S.JOURNAL and m["v"].get("niveau") == "erreur"][:20],
    }


def csv(lignes: list[dict[str, Any]]) -> str:
    """Trajectoire et commandes : t, x, y, cap, vitesse mesurée, vitesse visée, pwm g/d, servo."""
    sortie = ["t,x,y,cap,vitesse,vitesse_visee,pwm_g,pwm_d,servo_deg"]
    commande: dict[str, Any] = {}
    for m in lignes:
        if m["s"] == S.COMMANDE:
            commande = m["v"]
        elif m["s"] == S.ODOM:
            p = m["v"]["pose"]
            sortie.append(f"{m['t']:.3f},{p['x']:.3f},{p['y']:.3f},{p['cap']:.4f},"
                          f"{m['v'].get('vitesse', 0):.3f},{commande.get('vitesse', 0):.3f},"
                          f"{commande.get('pwm_g', 0)},{commande.get('pwm_d', 0)},"
                          f"{commande.get('servo_deg', 0)}")
    return "\n".join(sortie) + "\n"


def main() -> None:
    parseur = argparse.ArgumentParser(description="Relecture de la boîte noire du rover")
    parseur.add_argument("fichier", type=Path, nargs="?",
                         help="fichier .jsonl.gz (défaut : le plus récent)")
    parseur.add_argument("--csv", action="store_true", help="trajectoire et commandes en CSV")
    args = parseur.parse_args()
    fichier = args.fichier or max(DOSSIER.glob("*.jsonl.gz"), default=None)
    if fichier is None:
        raise SystemExit("aucun enregistrement")
    lignes = lire(fichier)
    if args.csv:
        print(csv(lignes), end="")
    else:
        print(json.dumps(resume(lignes), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
