"""Encodage pur du protocole E88 RCFPVPro ; aucun envoi à l'import.

Spécification observée par e88pilot (Volonid), MIT : UDP 7060 → 7080,
handshake 7070 → 7080. Ces octets ne sont PAS validés pour CooleerX BLE.
https://github.com/Volonid/e88pilot/blob/master/backend/protocols/e88pro_rc_protocol_adapter.py
"""

from __future__ import annotations

import hashlib
import json
import math
import socket
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

HANDSHAKE = bytes.fromhex("cc5a01820236b7")
PORT_RC = 7060
PORT_VIDEO = 7070
PORT_DRONE = 7080
PERIODE_RC_S = 0.04


@dataclass(frozen=True)
class AxesRC:
    montee: float = 0.0
    lacet: float = 0.0
    avant: float = 0.0
    droite: float = 0.0

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(v) and abs(v) <= 1
            for v in (self.montee, self.lacet, self.avant, self.droite)
        ):
            raise ValueError("axes RC attendus entre -1 et +1, sans NaN")


def paquet_rc(axes: AxesRC, action: str | None = None, amplitude: float = 0.15) -> bytes:
    if not math.isfinite(amplitude) or not 0 < amplitude <= 0.25:
        raise ValueError("amplitude du banc RC hors limites")
    if action not in (None, "decoller", "atterrir", "stop", "calibrer"):
        raise ValueError("action non documentée pour le protocole RCFPVPro")
    # E88 RCFPVPro partage le même bit bascule pour takeoff ET land. L'action
    # doit être impulsionnelle, jamais répétée par un battement de vol.
    drapeau = {None: 0, "decoller": 1, "atterrir": 1, "stop": 2, "calibrer": 4}[action]
    valeurs = [
        round(128 + v * 127 * amplitude) for v in (axes.droite, axes.avant, axes.montee, axes.lacet)
    ]
    interne = bytearray([0x66, 20, *valeurs, drapeau, 0, *([0] * 10), 0, 0x99])
    for b in interne[2:8]:
        interne[18] ^= b
    corps = bytes([1, 0x83, 21]) + bytes(interne)
    checksum = 0
    for b in corps:
        checksum ^= b
    return b"\xcc\x5a" + corps + bytes([checksum])


@dataclass(frozen=True)
class ValidationMateriel:
    identite: str
    protocole: str
    capture: Path
    sha256: str
    axes_valides: bool
    arret_valide: bool
    perte_liaison_validee: bool

    @classmethod
    def charger(cls, chemin: Path, identite: str) -> ValidationMateriel:
        d = json.loads(chemin.read_text(encoding="utf-8"))
        v = cls(
            identite=d["identite"],
            protocole=d["protocole"],
            capture=(chemin.parent / d["capture"]).resolve(),
            sha256=d["sha256"],
            axes_valides=d.get("axes_valides") is True,
            arret_valide=d.get("arret_valide") is True,
            perte_liaison_validee=d.get("perte_liaison_validee") is True,
        )
        v.verifier(identite)
        return v

    def verifier(self, identite: str) -> None:
        v = self
        if v.identite != identite or v.protocole != "e88_rcfpvpro_udp":
            raise ValueError("validation d'une autre identité ou d'un autre protocole")
        if not all((v.axes_valides, v.arret_valide, v.perte_liaison_validee)):
            raise ValueError("validation des axes/arrêt/perte de liaison incomplète")
        if (
            not v.capture.is_file()
            or v.capture.stat().st_size > 100 * 1024 * 1024
            or hashlib.sha256(v.capture.read_bytes()).hexdigest() != v.sha256
        ):
            raise ValueError("capture de référence absente, trop grande ou modifiée")


class Emetteur(Protocol):
    def envoyer(self, paquet: bytes) -> None: ...
    def fermer(self) -> None: ...


class TransportUDP:
    """Ouverture explicite après validation, jamais depuis la découverte BLE.

    Aucun réarmement/reconnect automatique. Le socket vidéo 7070 appartient à la
    même session et devra être partagé avec le récepteur vidéo de cette variante.
    """

    def __init__(
        self, validation: ValidationMateriel, *, autoriser: bool = False, ip: str = "192.168.28.1"
    ) -> None:
        if not autoriser or validation.protocole != "e88_rcfpvpro_udp":
            raise ValueError("ouverture du transport matériel non autorisée")
        validation.verifier(validation.identite)
        if ip != "192.168.28.1":
            raise ValueError("adresse différente du profil RCFPVPro validé")
        self.adresse = (ip, PORT_DRONE)
        self.rc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.rc.bind(("", PORT_RC))
            self.video.bind(("", PORT_VIDEO))
            self.video.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
            self.video.settimeout(0.2)
            for _ in range(3):
                self.video.sendto(HANDSHAKE, self.adresse)
        except Exception:
            self.fermer()
            raise

    def envoyer(self, paquet: bytes) -> None:
        self.rc.sendto(paquet, self.adresse)

    def fermer(self) -> None:
        self.rc.close()
        self.video.close()


class BattementRC:
    """Fil RC indépendant de la vision : dernière consigne, TTL, land une seule fois.

    L'absence de consigne récente déclenche un atterrissage via le profil testé,
    puis la session est verrouillée jusqu'à une nouvelle création explicite.
    """

    def __init__(self, emetteur: Emetteur) -> None:
        self.emetteur = emetteur
        self._verrou = threading.Lock()
        self._arret = threading.Event()
        self._fil = threading.Thread(target=self._boucle, name="drone-rc-25hz", daemon=True)
        self._axes = AxesRC()
        self._date: float | None = None
        self._action: str | None = None
        self.verrouille = False
        self.erreur: str | None = None
        self._decollage_envoye = False
        self._pose_demandee = False
        self._stop_envoye = False
        self._vol_confirme = False

    def confirmer_en_vol(self, en_vol: bool) -> None:
        """Retour observé, jamais déduit de l'envoi d'une commande takeoff."""
        with self._verrou:
            self._vol_confirme = en_vol is True

    def proposer(self, axes: AxesRC, maintenant: float, action: str | None = None) -> None:
        if action not in (None, "decoller", "atterrir", "stop", "calibrer"):
            raise ValueError("action RC inconnue")
        with self._verrou:
            if self.verrouille:
                raise ValueError("session RC interrompue : nouvelle session explicite nécessaire")
            if not math.isfinite(maintenant) or (
                self._date is not None and maintenant < self._date
            ):
                raise ValueError("horloge RC invalide")
            if action is not None and self._action is not None:
                raise ValueError("action RC déjà en attente")
            self._axes, self._date = axes, maintenant
            if action:
                self._action = action

    def pas(self, maintenant: float) -> bytes:
        with self._verrou:
            if self._date is None:
                raise ValueError("aucune consigne RC explicite")
            if not math.isfinite(maintenant) or maintenant < self._date:
                self.verrouille = True
                self._action = None
                raise ValueError("horloge RC incohérente")
            action = self._action
            self._action = None
            if not self.verrouille and maintenant - self._date > 0.3:
                self.verrouille = True
                action = "atterrir" if self._vol_confirme and not self._pose_demandee else None
            if action == "decoller":
                if self._decollage_envoye:
                    raise ValueError("décollage déjà demandé dans cette session")
                self._decollage_envoye = True
            if action == "atterrir":
                if self._pose_demandee or not self._vol_confirme:
                    action = None  # ne pas basculer vers takeoff sur un drone au sol
                else:
                    self._pose_demandee = True
            axes = AxesRC() if self.verrouille else self._axes
            if action == "stop":
                self.verrouille = True
                self._stop_envoye = True
                axes = AxesRC()
            return paquet_rc(axes, action)

    def demarrer(self) -> None:
        if self._date is None:
            raise ValueError("proposer une consigne avant de démarrer le battement")
        self._fil.start()

    def _boucle(self) -> None:
        prochaine = time.monotonic()
        while not self._arret.is_set():
            try:
                self.emetteur.envoyer(self.pas(time.monotonic()))
            except Exception as erreur:
                self.erreur = str(erreur)
                self.verrouille = True
                break  # pas de nouvelle session ou de réarmement automatique
            prochaine += PERIODE_RC_S
            maintenant = time.monotonic()
            if prochaine < maintenant:
                prochaine = maintenant  # ne jamais rejouer une rafale en retard
            self._arret.wait(max(0, prochaine - maintenant))

    def fermer(self) -> None:
        self._arret.set()
        if self._fil.is_alive():
            self._fil.join(timeout=0.3)
        try:
            if self._vol_confirme and not self._pose_demandee and not self._stop_envoye:
                self.emetteur.envoyer(paquet_rc(AxesRC(), "atterrir"))
                self._pose_demandee = True
        finally:
            self.emetteur.fermer()
