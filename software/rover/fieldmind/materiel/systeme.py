"""État du Pi : température, alimentation, charge, mémoire, disque et Wi-Fi (message `systeme`)."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

BITS_ALIMENTATION = {0: "sous_tension", 1: "frequence_plafonnee", 2: "ralenti", 3: "thermique"}


def _lire(chemin: str) -> str | None:
    try:
        return Path(chemin).read_text().strip()
    except OSError:
        return None


def _commande(*args: str) -> str:
    try:
        return subprocess.run(args, capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return ""


class Systeme:
    """Mesures du système ; les plus lentes (réseau, alimentation) sont mises en cache 5 s."""

    def __init__(self) -> None:
        self._cpu_precedent: tuple[int, int] | None = None
        self._processus_precedent: tuple[float, float] | None = None
        self._cache: dict[str, Any] = {}
        self._cache_t = -1e9

    def _cpu_pourcent(self) -> float | None:
        ligne = _lire("/proc/stat")
        if not ligne:
            return None
        valeurs = [int(v) for v in ligne.splitlines()[0].split()[1:]]
        inactif, total = valeurs[3] + valeurs[4], sum(valeurs)
        precedent, self._cpu_precedent = self._cpu_precedent, (inactif, total)
        if precedent is None or total == precedent[1]:
            return None
        return round(100 * (1 - (inactif - precedent[0]) / (total - precedent[1])), 1)

    def _processus_pourcent(self) -> float | None:
        maintenant, cpu = time.monotonic(), time.process_time()
        precedent, self._processus_precedent = self._processus_precedent, (maintenant, cpu)
        if precedent is None or maintenant <= precedent[0]:
            return None
        return round(100 * (cpu - precedent[1]) / (maintenant - precedent[0]), 1)

    def _lent(self) -> dict[str, Any]:
        sortie = _commande("vcgencmd", "get_throttled").strip()
        try:
            valeur = int(sortie.split("=")[1], 16)
            throttled: str | None = hex(valeur)
        except (IndexError, ValueError):
            valeur, throttled = 0, None
        wifi: dict[str, Any] = {"ssid": None, "signal_pct": None, "ip": None}
        sortie = _commande("nmcli", "-t", "-f", "ACTIVE,SSID,SIGNAL", "dev", "wifi")
        for ligne in sortie.splitlines():
            actif, _, reste = ligne.partition(":")
            if actif == "yes" or actif == "oui":
                ssid, _, signal = reste.rpartition(":")
                wifi["ssid"] = ssid.replace("\\:", ":") or None
                wifi["signal_pct"] = int(signal) if signal.isdigit() else None
        try:
            for interface in json.loads(_commande("ip", "-j", "-4", "addr") or "[]"):
                if interface.get("ifname") == "wlan0":
                    for adresse in interface.get("addr_info", []):
                        wifi["ip"] = adresse.get("local")
        except (ValueError, KeyError, TypeError):
            pass
        try:  # réseaux candidats, publiés par le service fieldmind-wifi
            candidats = json.loads(_lire("/run/fieldmind/wifi.json") or "{}")
            wifi["candidats"] = candidats.get("visibles")
        except ValueError:
            wifi["candidats"] = None
        return {"throttled": throttled, "sous_tension": bool(valeur & 1),
                "ralenti": bool(valeur & 0b1110),
                "sous_tension_depuis_demarrage": bool(valeur >> 16 & 1),
                "wifi": wifi}

    def etat(self) -> dict[str, Any]:
        if time.monotonic() - self._cache_t > 5:
            self._cache = self._lent()
            self._cache_t = time.monotonic()
        memoire: dict[str, int] = {}
        for ligne in (_lire("/proc/meminfo") or "").splitlines():
            cle, _, valeur = ligne.partition(":")
            if valeur.split():
                memoire[cle] = int(valeur.split()[0])
        disque = shutil.disk_usage("/")
        temperature = _lire("/sys/class/thermal/thermal_zone0/temp")
        total = memoire.get("MemTotal")
        return {
            "cpu_pct": self._cpu_pourcent(),
            "processus_cpu_pct": self._processus_pourcent(),
            "temperature_c": int(temperature) / 1000 if temperature else None,
            "memoire_pct": round(100 * (1 - memoire.get("MemAvailable", 0) / total), 1)
            if total else None,
            "disque_pct": round(100 * (1 - disque.free / disque.total), 1),
            "charge": [round(c, 2) for c in os.getloadavg()],
            "uptime_s": float((_lire("/proc/uptime") or "0").split()[0]),
            **self._cache,
        }
