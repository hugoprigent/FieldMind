"""GPS NMEA (ATGM336H sur l'UART du Pi, /dev/ttyS0, 9 600 bauds) → Gnss sur le bus."""

from __future__ import annotations

import re
import threading
import time
from typing import Any

from fieldmind.bus import Bus, S
from fieldmind.materiel.esp import Transport
from fieldmind.types import Gnss

NMEA = re.compile(r"^\$([A-Z]{2})([A-Z]{2,4}),[^*]*\*([0-9A-Fa-f]{2})$")
QUALITES_FIX = {0: "aucun", 1: "autonome", 2: "dgnss", 4: "rtk_fixe", 5: "rtk_flottant",
                6: "estime"}


def nmea_valide(ligne: str) -> bool:
    trame = NMEA.match(ligne)
    if not trame:
        return False
    somme = 0
    for caractere in ligne[1 : ligne.index("*")]:
        somme ^= ord(caractere)
    return somme == int(trame.group(3), 16)


def _degres(valeur: str, hemisphere: str) -> float | None:
    """« ddmm.mmmm » NMEA en degrés décimaux signés."""
    if not valeur:
        return None
    point = valeur.index(".") if "." in valeur else len(valeur)
    degres = float(valeur[: point - 2]) + float(valeur[point - 2 :]) / 60
    return -degres if hemisphere in ("S", "W") else degres


class DecodeurNmea:
    def __init__(self) -> None:
        self.dernier = Gnss(t=0.0)
        self.en_vue: dict[str, int] = {}
        self.mode: str | None = None
        self.trames = 0

    def traiter(self, ligne: str, t: float) -> Gnss | None:
        """Renvoie une mesure complète à chaque trame GGA, sinon None."""
        if not nmea_valide(ligne):
            return None
        champs = ligne[1 : ligne.index("*")].split(",")
        emetteur, phrase = champs[0][:2], champs[0][2:]
        self.trames += 1
        g = self.dernier
        try:
            if phrase == "GGA" and len(champs) >= 10:
                fix = QUALITES_FIX.get(int(champs[6] or 0), champs[6])
                lat = _degres(champs[2], champs[3]) if fix != "aucun" else None
                lon = _degres(champs[4], champs[5]) if fix != "aucun" else None
                self.dernier = Gnss(
                    t=t, fix=fix, latitude=lat, longitude=lon,
                    altitude_m=float(champs[9]) if champs[9] else None,
                    hdop=float(champs[8]) if champs[8] else None,
                    satellites=int(champs[7] or 0), vitesse_m_s=g.vitesse_m_s, cap_deg=g.cap_deg)
                return self.dernier
            if phrase == "RMC" and len(champs) >= 9:
                g.vitesse_m_s = float(champs[7]) * 0.514444 if champs[7] else None
                g.cap_deg = float(champs[8]) if champs[8] else None
            elif phrase == "GSA" and len(champs) >= 3:
                self.mode = {"1": "pas de fix", "2": "2D", "3": "3D"}.get(champs[2])
            elif phrase == "GSV" and len(champs) >= 4 and champs[3]:
                self.en_vue[emetteur] = int(champs[3])
        except ValueError:
            pass
        return None


class PiloteGnss:
    def __init__(self, bus: Bus, transport: Transport, nom: str = "gnss") -> None:
        self.bus, self.nom = bus, nom
        self._t = transport
        self._decodeur = DecodeurNmea()
        self._actif = True
        self.erreur: str | None = None
        self._fil = threading.Thread(target=self._boucle, name=nom, daemon=True)

    def demarrer(self) -> None:
        self._fil.start()

    def arreter(self) -> None:
        self._actif = False

    @property
    def actif(self) -> bool:
        return self._actif

    def resume(self) -> dict[str, Any]:
        return {"ok": self._actif and self.bus.age(S.GNSS) < 3,
                "age_s": round(min(self.bus.age(S.GNSS), 99), 1), "mode": self._decodeur.mode,
                "satellites_en_vue": sum(self._decodeur.en_vue.values()),
                "trames": self._decodeur.trames, "erreur": self.erreur}

    def _boucle(self) -> None:
        tampon = b""
        premier_fix = True
        try:
            while self._actif:
                morceau = self._t.read(512)
                if not morceau:
                    continue
                tampon += morceau
                *lignes, tampon = tampon.split(b"\n")
                if len(tampon) > 2048:
                    tampon = b""
                for brute in lignes:
                    mesure = self._decodeur.traiter(brute.decode("latin-1").strip(),
                                                    time.monotonic())
                    if mesure is None:
                        continue
                    self.bus.publier(S.GNSS, mesure)
                    if premier_fix and mesure.valide:
                        premier_fix = False
                        self.bus.journaliser(self.nom, f"position GPS ({mesure.satellites} "
                                             "satellites)", "succes")
        except OSError as erreur:
            self.erreur = str(erreur)
            self.bus.journaliser(self.nom, f"GPS perdu : {erreur}", "alerte")
        finally:
            self._actif = False
            try:
                self._t.close()
            except OSError:
                pass
