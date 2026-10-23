"""Lidar Slamtec RPLIDAR C1 : balayage standard (protocole binaire), tours datés dans base_link.

Requête : 0xA5 puis la commande. Réponse : descripteur de 7 octets (0xA5 0x5A, longueur, mode,
type) puis les données. En balayage (SCAN), chaque mesure tient en 5 octets : qualité et bit de
début de tour, angle en 1/64 de degré, distance en 1/4 de millimètre.

Le C1 tourne dans le sens horaire vu de dessus, 0° à l'opposé du câble. Sur le rover, ce 0°
regarde l'avant ; les angles sont convertis dans base_link (trigonométrique), puis les points qui
tombent sur le rover (mât des caméras, coque) sont retirés.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np

from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.materiel.esp import Transport
from fieldmind.types import Scan

STOP, RESET, SCAN = b"\xa5\x25", b"\xa5\x40", b"\xa5\x20"
GET_INFO, GET_HEALTH = b"\xa5\x50", b"\xa5\x52"
MODELES = {0x41: "RPLIDAR C1", 0x18: "RPLIDAR A1", 0x28: "RPLIDAR A2", 0x38: "RPLIDAR A3"}
SANTE = {0: "bonne", 1: "avertissement", 2: "erreur"}


def _reponse(port: Transport, commande: bytes, longueur: int) -> bytes | None:
    port.write(commande)
    reponse = b""
    fin = time.monotonic() + 1.0
    while len(reponse) < 7 + longueur and time.monotonic() < fin:
        reponse += port.read(7 + longueur - len(reponse))
    if len(reponse) != 7 + longueur or reponse[:2] != b"\xa5\x5a":
        return None
    return reponse[7:]


def identifier(port: Transport) -> dict[str, Any] | None:
    """Fiche du lidar si un RPLIDAR répond sur ce port (déjà ouvert au bon débit)."""
    port.write(STOP)
    time.sleep(0.05)
    _vider(port)
    info = _reponse(port, GET_INFO, 20)
    if info is None:
        return None
    sante = _reponse(port, GET_HEALTH, 3)
    return {
        "modele": MODELES.get(info[0], f"RPLIDAR {info[0]:#04x}"),
        "firmware": f"{info[2]}.{info[1]:02d}",
        "materiel": info[3],
        "numero_serie": info[4:].hex().upper(),
        "sante": SANTE.get(sante[0], str(sante[0])) if sante else None,
    }


def _vider(port: Transport) -> None:
    for _ in range(50):
        if not port.read(4096):
            return


class FiltreRover:
    """Convertit (angle lidar, distance) en polaire base_link et retire les points du rover."""

    def __init__(self, profil: Profil) -> None:
        self.profil: Profil | None = profil
        self.x_l = float(profil.valeur("lidar.x_m", 0.0))
        self.y_l = float(profil.valeur("lidar.y_m", 0.0))
        self.decalage = math.radians(float(profil.valeur("lidar.decalage_deg", 0.0)))
        self.horaire = bool(profil.valeur("lidar.sens_horaire", True))
        self.dmin = float(profil.valeur("lidar.portee_min_m", 0.12))
        self.dmax = float(profil.valeur("lidar.portee_max_m", 12.0))
        g = profil.geometrie
        # Le lidar est au-dessus de tout le rover sauf le haut du mât des caméras, à l'avant :
        # on retire ce qui tombe dans l'emprise, agrandie de 2 cm.
        self.geometrie = g
        self.boite = self._boite(0.05)

    def _boite(self, marge: float) -> tuple[float, float, float]:
        g = self.geometrie
        return (g.emprise_arriere_m - marge, g.emprise_avant_m + marge,
                g.emprise_demi_largeur_m + marge)

    def convertir(self, angles_deg: np.ndarray, distances_mm: np.ndarray
                  ) -> tuple[np.ndarray, np.ndarray]:
        if self.profil is not None:  # réglable depuis le poste
            self.decalage = math.radians(float(self.profil.valeur("lidar.decalage_deg", 0.0)))
        d = distances_mm.astype(np.float64) / 1000.0
        a = np.radians(angles_deg.astype(np.float64))
        a = self.decalage - a if self.horaire else self.decalage + a
        garde = (d >= self.dmin) & (d <= self.dmax)
        d, a = d[garde], a[garde]
        x = self.x_l + d * np.cos(a)
        y = self.y_l + d * np.sin(a)
        if self.profil is not None:  # marge réglable : le support des caméras dépasse parfois
            self.boite = self._boite(float(self.profil.valeur("lidar.masque_marge_m", 0.05)))
        xmin, xmax, demi = self.boite
        dedans = (x > xmin) & (x < xmax) & (np.abs(y) < demi)
        x, y = x[~dedans], y[~dedans]
        angles = np.arctan2(y, x)
        distances = np.hypot(x, y)
        ordre = np.argsort(angles)
        return angles[ordre].astype(np.float32), distances[ordre].astype(np.float32)


class Decodeur:
    """Décode le flux SCAN ; rend chaque tour complet (angles en degrés lidar, distances en mm)."""

    def __init__(self) -> None:
        self._tampon = bytearray()
        self._angles: list[float] = []
        self._distances: list[float] = []
        self._qualites: list[int] = []
        self.rejetees = 0

    def recevoir(self, donnees: bytes) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
        tours = []
        tampon = self._tampon
        tampon += donnees
        i = 0
        while len(tampon) - i >= 5:
            b0, b1, b2, b3, b4 = tampon[i : i + 5]
            debut, debut_inverse, controle = b0 & 1, (b0 >> 1) & 1, b1 & 1
            if debut == debut_inverse or controle != 1:
                self.rejetees += 1
                i += 1  # désynchronisé : on avance d'un octet
                continue
            if debut and self._angles:
                tours.append((np.array(self._angles), np.array(self._distances),
                              np.array(self._qualites)))
                self._angles, self._distances, self._qualites = [], [], []
            distance = (b3 | b4 << 8) / 4
            if distance > 0:
                self._angles.append(((b1 >> 1) | b2 << 7) / 64)
                self._distances.append(distance)
                self._qualites.append(b0 >> 2)
            i += 5
        del tampon[:i]
        return tours


class PiloteLidar:
    def __init__(self, bus: Bus, profil: Profil, transport: Transport, nom: str = "lidar") -> None:
        self.bus, self.profil, self.nom = bus, profil, nom
        self._t = transport
        self._filtre = FiltreRover(profil)
        self._actif = True
        self.marche = True
        self.info: dict[str, Any] | None = None
        self.erreur: str | None = None
        self.frequence_hz = 0.0
        self.points = 0
        self.tours = 0
        self._dernier_tour: float | None = None
        self._fil = threading.Thread(target=self._boucle, name=nom, daemon=True)

    def demarrer(self) -> None:
        self._fil.start()

    def arreter(self) -> None:
        self._actif = False

    @property
    def actif(self) -> bool:
        return self._actif

    def resume(self) -> dict[str, Any]:
        return {"ok": self._actif and self.marche and self.bus.age(S.SCAN) < 0.5,
                "hz": round(self.frequence_hz, 1), "points": self.points,
                "age_s": round(min(self.bus.age(S.SCAN), 99.0), 2), "marche": self.marche,
                "info": self.info, "erreur": self.erreur}

    def _boucle(self) -> None:
        port = self._t
        try:
            self.info = identifier(port)
            if self.info:
                self.bus.journaliser(self.nom, f"{self.info['modele']} (firmware "
                                     f"{self.info['firmware']}, santé {self.info['sante']})",
                                     "succes")
            decodeur = Decodeur()
            en_balayage = False
            while self._actif:
                if self.marche and not en_balayage:
                    port.write(STOP)
                    time.sleep(0.05)
                    _vider(port)
                    port.write(SCAN)
                    entete = b""
                    fin = time.monotonic() + 1.0
                    while len(entete) < 7 and time.monotonic() < fin:
                        entete += port.read(7 - len(entete))
                    if entete[:2] != b"\xa5\x5a":
                        raise OSError("le lidar refuse le balayage")
                    decodeur = Decodeur()
                    en_balayage = True
                elif not self.marche and en_balayage:
                    port.write(STOP)
                    en_balayage = False
                if not en_balayage:
                    time.sleep(0.1)
                    continue
                donnees = port.read(2048)
                if not donnees:
                    continue
                for angles, distances, _qualites in decodeur.recevoir(donnees):
                    self._publier(angles, distances)
        except OSError as erreur:
            self.erreur = str(erreur)
            self.bus.journaliser(self.nom, f"lidar perdu : {erreur}", "erreur")
        finally:
            try:
                port.write(STOP)  # arrête le moteur du lidar
                port.close()
            except OSError:
                pass
            self._actif = False

    def _publier(self, angles_deg: np.ndarray, distances_mm: np.ndarray) -> None:
        maintenant = time.monotonic()
        duree = 0.1
        if self._dernier_tour is not None:
            duree = min(max(maintenant - self._dernier_tour, 0.03), 0.5)
            self.frequence_hz = 0.8 * self.frequence_hz + 0.2 / duree if self.tours else 1 / duree
        self._dernier_tour = maintenant
        self.tours += 1
        angles, distances = self._filtre.convertir(angles_deg, distances_mm)
        self.points = len(angles)
        if len(angles) < 10:
            return
        self.bus.publier(S.SCAN, Scan(t=maintenant, angles=angles, distances=distances,
                                      duree=duree))
