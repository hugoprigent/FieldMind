"""Pilote du XIAO ESP32-C5 (firmware software/esp32/esp_c5) : traction, servo et centrale.

Deux versions du firmware sont gérées :
- **ROVER_4** : commande combinée `p <g> <d> <servo>` à 20 Hz, chien de garde `wd` (le firmware
  coupe la traction si le Pi se tait), télémétrie `T …` et centrale `I …` rapides. L'inversion de
  sens d'une roue est séquencée par le firmware.
- **ROVER_2** (et inconnu) : commandes `g`, `d`, `sensg`, `sensd`, `s` envoyées seulement quand
  elles changent ; le passage par zéro avant une inversion est séquencé ici. Ce firmware n'a pas
  de chien de garde : la traction est plafonnée plus bas et un arrêt est envoyé à la fermeture.

Le port est ouvert DTR/RTS bas pour ne pas redémarrer l'ESP (USB-JTAG natif).
"""

from __future__ import annotations

import base64
import math
import re
import threading
import time
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import replace
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.drone.cooleerx import DRAPEAUX_ACTION, paquet_camera, paquet_rc
from fieldmind.materiel.reconnaissance import REFUS_VOL, ReconnaissanceDrone
from fieldmind.types import CommandeRoues, EtatDrone, EtatEsp, ImageCamera, Imu

PWM_MAX_SANS_CHIEN = 20
"""Plafond de traction avec un firmware sans chien de garde (ROVER_2)."""
SANS_LIGNE_S = 1.5
"""Sans aucune ligne reçue pendant ce délai, l'ESP est considéré comme absent."""

_NB = r"([-+]?\d+(?:\.\d+)?)"
LIGNE_IMU_TEXTE = re.compile(
    rf"^IMU (\S+)\s+a=\[\s*{_NB}\s+{_NB}\s+{_NB}\] g"
    rf"\s+w=\[\s*{_NB}\s+{_NB}\s+{_NB}\] deg/s\s+{_NB} C"
)
LIGNE_MOTEURS = re.compile(r"^\[MOTEURS\] G=(\d+) % ; D=(\d+) % ; BRAKE=(\d) ; defaut=(\d)")
LIGNE_ROUE = re.compile(
    r"^\[(GAUCHE|DROITE)\] Demande=(\d+) % ; PWM=(-?\d+) % ; sens=(\d) ; DIR=(\d) ; inverse=(\d)"
)
LIGNE_SERVO = re.compile(
    rf"^\[SERVO\] Signal (actif|coupe) ; consigne en cours {_NB} ; cible {_NB}"
)
LIGNE_DEFAUT = re.compile(r"^\[DEFAUT\] Cause=([^;]+)")
LIGNE_FIRMWARE = re.compile(r"^\[FIRMWARE\] (\S+)")
LIGNE_CHIEN = re.compile(r"^\[CHIEN\]\s*(.*)$")
LIGNE_DRONE_JPG = re.compile(r"^\[DRONE_JPG\]\s+(\d+)\s+(\d+)/(\d+)\s+(\S+)$")


class Transport(Protocol):
    """Ce que le pilote attend d'un port série (pyserial ou faux ESP de la simulation)."""

    def read(self, n: int = 1) -> bytes: ...
    def write(self, donnees: bytes) -> int | None: ...
    def close(self) -> None: ...


def ouvrir_port(peripherique: str, debit: int) -> Transport:
    import serial  # pyserial, seulement sur le matériel réel

    port = serial.Serial()
    port.port, port.baudrate, port.timeout = peripherique, debit, 0.05
    # Écriture bornée : un ESP qui ne lit plus ne doit pas bloquer la boucle de conduite (la
    # commande suivante part 50 ms plus tard ; son chien de garde gère les vrais blocages)
    port.write_timeout = 0.05
    port.dtr = port.rts = False
    port.exclusive = True
    port.open()
    return port


def rotation_niveau(gravite: Sequence[float]) -> np.ndarray:
    """Rotation qui amène la gravité mesurée au repos, rover à plat, sur +z : corrige un
    GY-521 monté de biais (au-delà des simples échanges d'axes imu.axe_*). Le lacet autour
    de la verticale n'est pas observable ainsi : seuls roulis, tangage et vitesse de lacet."""
    v = np.asarray(gravite, dtype=np.float64)
    v = v / max(float(np.linalg.norm(v)), 1e-9)
    z = np.array([0.0, 0.0, 1.0])
    k = np.cross(v, z)
    s, c = float(np.linalg.norm(k)), float(v @ z)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + kx + kx @ kx * ((1 - c) / (s * s))


def axes_imu(profil: Profil) -> Callable[[tuple[float, float, float]], tuple[float, float, float]]:
    """Fonction qui ramène un vecteur du repère du MPU-6050 dans base_link (profil [imu])."""
    regles = []
    for axe in ("axe_x", "axe_y", "axe_z"):
        texte = str(profil.valeur(f"imu.{axe}", axe[-1])).strip()
        signe = -1.0 if texte.startswith("-") else 1.0
        indice = "xyz".index(texte.lstrip("+-")[:1] or "x")
        regles.append((indice, signe))

    def convertir(v: tuple[float, float, float]) -> tuple[float, float, float]:
        return (
            regles[0][1] * v[regles[0][0]],
            regles[1][1] * v[regles[1][0]],
            regles[2][1] * v[regles[2][0]],
        )

    return convertir


# Préparation d'une roue dans ROVER_3 (100 ms), avec une marge pour la liaison série
PREPARATION_ROVER3_S = 0.16


class PiloteEsp:
    def __init__(self, bus: Bus, profil: Profil, transport: Transport, nom: str = "esp",
                 *, drone_reel: bool = True) -> None:
        self.bus = bus
        self.profil = profil
        self.nom = nom
        self._drone_reel = drone_reel
        self._t = transport
        self._verrou_ecriture = threading.Lock()
        self._ecritures_sautees = 0
        # Préparation des roues dans ROVER_3 (voir appliquer) : dernière consigne p envoyée,
        # instant jusqu'auquel on n'envoie que des battements
        self._p_envoye = (0, 0)
        self._sens_envoye = [True, True]  # dernier sens non nul (True = avant de la sortie)
        self._preparation_fin = 0.0
        self._actif = True
        self.etat = EtatEsp(t=time.monotonic())
        self.etat_drone = EtatDrone(t=time.monotonic(), largeur=240, hauteur=320)
        self._drone_seq = 0
        self._drone_total = 0
        self._drone_tranches: dict[int, str] = {}
        self._drone_num_image = 0
        self._derniere_image_drone_t = 0.0
        self._derniere_mire_drone_t = 0.0
        self._ble_connecte = False
        self._ble_derniere_activite_t = 0.0
        self._ble_cible: str | None = None
        self._ble_scan_demande = False
        self._ble_deconnecter_demande = False
        self._ble_cmd_file: deque[bytes] = deque(maxlen=8)
        self._ble_vus: list[str] = []
        self._wifi_vus: list[str] = []
        self.drone_erreur: str | None = None
        self.recon = ReconnaissanceDrone(
            bus, self._actualiser_recon,
            Path(str(profil.valeur(
                "drone.dossier_recon", "~/.local/share/fieldmind/reconnaissances",
            ))).expanduser(),
        )
        self._derniere_ligne = 0.0
        self._traction_demandee = False
        self.version: str | None = None
        self.erreur: str | None = None
        self._axes = axes_imu(profil)
        self._niveau_texte: str | None = None
        self._niveau: np.ndarray | None = None
        # Volets : dernier état commandé (au démarrage, celui du réglage) ; un mouvement à la fois
        self._volets_deplies = bool(profil.valeur("remorque.volets_deplies", False))
        self._verrou_volets = threading.Lock()
        self._signe_lacet = self._axes((0.0, 0.0, 1.0))[2]
        self._fil = threading.Thread(target=self._lire, name=f"{nom}-lecture", daemon=True)
        self._fil_ble = threading.Thread(
            target=self._lire_drone_ble, name=f"{nom}-drone-ble", daemon=True
        )
        # Séquenceur ROVER_2 : par roue, pourcentage et sens appliqués, et inversion en attente
        self._applique = [0, 0]
        self._sens = [0, 0]
        self._zero_depuis = [time.monotonic(), time.monotonic()]
        self._servo_envoye: float | None = None
        self._configure = False

    # ------------------------------------------------------------------ cycle de vie

    def demarrer(self) -> None:
        self._fil.start()
        if self._drone_reel:
            self._fil_ble.start()
        self.recon.demarrer()
        self.envoyer_ligne("etat", journal=False)

    def arreter(self) -> None:
        """Traction à zéro (servo maintenu), puis fermeture du port."""
        try:
            self._ecrire("p 0 0 " + self._fmt_servo(self.etat.servo_deg) if self.rover3 else "m 0")
            self._ecrire("stop" if not self.rover3 else "wd 0")
        except OSError:
            pass
        self._actif = False
        self._ble_cmd_file.clear()
        self.recon.arreter()
        try:
            self._t.close()
        except OSError:
            pass

    @property
    def actif(self) -> bool:
        return self._actif

    @property
    def rover3(self) -> bool:
        return (
            bool(self.version) and self.version.startswith("ROVER_") and _numero(self.version) >= 3
        )

    @property
    def chien_de_garde(self) -> bool:
        return self.rover3 and self._configure

    def connecte(self) -> bool:
        return self._actif and time.monotonic() - self._derniere_ligne < SANS_LIGNE_S

    # ------------------------------------------------------------------ commandes

    def appliquer(self, cmd: CommandeRoues) -> None:
        """Appelé par la conduite à chaque période (20 Hz) : sert aussi de battement."""
        if not self._actif:
            return
        g, d = int(cmd.pwm_g), int(cmd.pwm_d)
        if self._permutes():  # moteurs branchés à l'envers : sortie gauche → moteur droit
            g, d = d, g
        if self._sens_inverse():  # avancer faisait reculer : les deux sens retournés
            g, d = -g, -d
        self._traction_demandee = g != 0 or d != 0
        servo = max(-90.0, min(90.0, cmd.servo_deg))
        if self.rover3:
            if not self._configure:
                self._configurer()
            self._ecrire(self._ligne_rover3(g, d, servo))
            return
        if not self._configure:  # ROVER_2 : seulement la vitesse du servo
            vitesse = float(self.profil.valeur("direction.servo_vitesse_deg_s", 150))
            self._ecrire(f"v {max(1, min(180, vitesse)):.0f}")
            self._configure = True
        plafond = PWM_MAX_SANS_CHIEN
        g, d = max(-plafond, min(plafond, g)), max(-plafond, min(plafond, d))
        self._sequencer(0, g)
        self._sequencer(1, d)
        if self._servo_envoye is None or abs(servo - self._servo_envoye) >= 0.5:
            self._ecrire(f"s {self._fmt_servo(servo)}")
            self._servo_envoye = servo

    def _ligne_rover3(self, g: int, d: int, servo: float) -> str:
        """Ligne `p`, ou `hb` pendant la préparation d'une roue.

        Dans ROVER_3, une roue qui démarre (ou change de sens, après `inv` ms à zéro) passe
        100 ms en préparation avant de recevoir sa PWM, et chaque nouvelle ligne `p` pendant
        ce temps relance la préparation : à 20 Hz, la roue ne démarrait jamais. On laisse donc
        passer la préparation en n'envoyant que le battement (le chien de garde reste nourri).
        Un arrêt (une roue à zéro) part toujours tout de suite."""
        maintenant = time.monotonic()
        demarre = inverse = arret = False
        for i, (v, avant) in enumerate(zip((g, d), self._p_envoye, strict=True)):
            if not v:
                arret = arret or bool(avant)
            elif (v > 0) != self._sens_envoye[i]:
                inverse = True  # le firmware attend d'abord `inv` ms roue à zéro
            elif not avant:
                demarre = True
        if not (arret or demarre or inverse) and maintenant < self._preparation_fin:
            return "hb"
        if demarre or inverse:
            attente = PREPARATION_ROVER3_S
            if inverse:
                attente += float(self.profil.valeur("traction.delai_inversion_s", 0.6))
            self._preparation_fin = max(self._preparation_fin, maintenant + attente)
        self._p_envoye = (g, d)
        self._sens_envoye = [v > 0 if v else ancien for v, ancien in zip((g, d), self._sens_envoye,
                                                                strict=True)]
        return f"p {g} {d} {self._fmt_servo(servo)}"

    def reconfigurer(self) -> None:
        """Un réglage a changé (chien de garde, vitesse du servo…) : renvoyé au prochain cycle."""
        self._configure = False
        self._servo_envoye = None

    def arret_traction(self) -> None:
        """Traction à zéro tout de suite (arrêt d'urgence), servo maintenu."""
        if self.rover3:
            self._ecrire(f"p 0 0 {self._fmt_servo(self.etat.servo_deg)}")
        else:
            self._ecrire("m 0")
            self._applique = [0, 0]
            now = time.monotonic()
            self._zero_depuis = [now, now]

    def envoyer_ligne(self, texte: str, journal: bool = True) -> bool:
        """Ligne brute (console du poste). L'appelant vérifie que le rover est à l'arrêt."""
        texte = texte.strip()[:90]
        if not texte or not all(32 <= ord(c) < 127 for c in texte):
            return False
        if texte.split()[0].lower() == "drone":
            return self.commander_drone(texte[5:].strip(), journal=journal)
        self._ecrire(texte)
        if journal:
            self.bus.publier(S.CONSOLE_ESP, (time.time(), f"→ {texte}"))
        if texte.split()[0] in {"stop", "m", "g", "d", "sens", "sensg", "sensd"}:
            self._servo_envoye = None  # l'état réel n'est plus connu : renvoyer au prochain cycle
        return True

    def _actualiser_recon(self, **champs: Any) -> None:
        self.etat_drone = replace(self.etat_drone, **champs)
        self.bus.publier(S.DRONE, self.etat_drone)

    def commander_drone(self, sous_commande: str, journal: bool = True) -> bool:
        """Valide la demande avant tout envoi ; aucun armement sur protocole inconnu."""
        self.drone_erreur = None
        if not self._actif:
            self.drone_erreur = "pilote ESP arrêté"
            return False
        propre = sous_commande.strip()
        if not propre or len(propre) > 76 or not all(32 <= ord(c) < 127 for c in propre):
            self.drone_erreur = "commande drone invalide"
            return False
        mots = propre.split()
        action = mots[0].lower()
        maintenant = time.monotonic()
        try:
            if action == "cmd" and len(mots) == 2:
                cmd = mots[1].lower()
                if cmd in ("decoller", "recon_vol", "calibrer"):
                    raise ValueError(REFUS_VOL)
                if cmd in ("cibler_croix", "recon_test", "recon_capture", "stop_recon"):
                    self.recon.commander(cmd, maintenant)
                    return True
                if cmd not in ("camera", "atterrir", "stop"):
                    raise ValueError("commande de vol inconnue")
                if cmd in ("stop", "atterrir"):
                    self.recon.commander("stop_recon", maintenant)
                    self._ble_cmd_file.clear()
                elif cmd == "camera":
                    # Un changement de capteur invalide immédiatement le centrage.
                    self.recon.interrompre(
                        "Caméra changée — relancer la visée sur la nouvelle image",
                    )
                if not self.etat_drone.connecte:
                    raise ValueError("drone déconnecté — commande non mise en attente")
                if self._ble_connecte:
                    if cmd == "camera":
                        self._cam_drone_index = 1 - getattr(self, "_cam_drone_index", 0)
                        cible = self._cam_drone_index + 1
                        # HD dragon 1.1.0, v2.y : sélection caméra hors RC.
                        self._ble_cmd_file.append(paquet_camera(cible))
                    else:
                        pkt = _paquet_vol_ble(cmd)
                        if pkt:
                            self._ble_cmd_file.append(pkt)
                else:
                    if cmd == "camera":
                        raise ValueError("bascule caméra disponible seulement via BLE actuellement")
                    self._ecrire(f"drone cmd {cmd}")
                self._actualiser_recon(commande_vol=cmd)
                return True
            if action == "manette":
                raise ValueError(REFUS_VOL)
            if action in ("scan", "reconnecter", "deconnecter", "ssid"):
                self._ble_cmd_file.clear()
                self.recon.interrompre("Liaison modifiée — essai interrompu")
            if action in ("scan", "reconnecter"):
                self._ble_scan_demande = True
            elif action == "deconnecter":
                self._ble_deconnecter_demande = True
            elif action == "ssid" and len(mots) >= 2:
                cible = " ".join(mots[1:]).strip()
                self._ble_cible = None if cible.lower() == "auto" else cible
                self._ble_scan_demande = True
            elif action not in ("etat", "flux"):
                raise ValueError("action drone inconnue")
            if not self._ble_connecte:
                ligne = f"drone {propre}"
                self._ecrire(ligne)
                if journal:
                    self.bus.publier(S.CONSOLE_ESP, (time.time(), f"→ {ligne}"))
            return True
        except (ValueError, OSError) as erreur:
            self.drone_erreur = str(erreur)
            return False

    def _configurer(self) -> None:
        chien = int(self.profil.valeur("esp.chien_de_garde_ms", 400))
        inversion = int(1000 * float(self.profil.valeur("traction.delai_inversion_s", 0.6)))
        vitesse = float(self.profil.valeur("direction.servo_vitesse_deg_s", 150))
        for ligne in (
            f"inv {max(100, min(3000, inversion))}",
            f"v {max(1, min(180, vitesse)):.0f}",
            "tel 20",
            "imu 50",
            f"wd {max(100, min(2000, chien))}",
        ):
            self._ecrire(ligne)
        # Sorties SC des drivers sur D2/D3 (niveau électrique adapté d'abord !)
        self._ecrire("roues 1" if bool(self.profil.valeur("roues.capteurs", False)) else "roues 0")
        self._configure = True
        self.bus.journaliser(
            self.nom,
            f"{self.version} : chien de garde {chien} ms, télémétrie 20 Hz, centrale 50 Hz",
            "succes",
        )

    def volets_disponibles(self) -> bool:
        """Firmware ROVER_4 et D2/D3 libres (pas de capteurs SC des roues)."""
        return (self._actif and bool(self.version) and _numero(self.version) >= 4
                and not bool(self.profil.valeur("roues.capteurs", False)))

    def mouvoir_volets(self, deplies: bool) -> bool:
        """Volets de la remorque, servos à **rotation continue** (D2/D3) : l'impulsion règle la
        vitesse, pas la position (90 = arrêt). Pour déplier ou replier, chaque servo tourne à
        `remorque.volets_vitesse` % dans le bon sens le temps de `remorque.volets_angle_deg`
        (voir duree_volets), puis
        son signal est coupé (arrêt). Sans capteur de position : rien n'est fait si l'état
        demandé est déjà l'état connu. Bloquant (à lancer dans un fil) ; faux si rien n'a bougé."""
        if not self.volets_disponibles() or deplies == self._volets_deplies:
            return False
        if not self._verrou_volets.acquire(blocking=False):
            return False  # déjà en mouvement
        try:
            vitesse = max(0.0, min(100.0, float(self.profil.valeur("remorque.volets_vitesse", 40))))
            duree = self.duree_volets()
            for cote in ("g", "d"):
                sens = 1.0 if deplies else -1.0
                if bool(self.profil.valeur(f"remorque.volet_{cote}_inverse", cote == "d")):
                    sens = -sens
                self._ecrire(f"vol{cote} {round(90 + sens * 0.9 * vitesse)}")
            fin = time.monotonic() + duree
            while time.monotonic() < fin and self._actif:
                time.sleep(0.02)
            self._volets_deplies = deplies
            return True
        finally:
            self._ecrire("volg -1")  # arrêt, même en cas d'erreur
            self._ecrire("vold -1")
            self._verrou_volets.release()

    def duree_volets(self) -> float:
        """Temps de rotation pour `remorque.volets_angle_deg` (90° par défaut) à
        `remorque.volets_vitesse` %, d'après la vitesse réelle des servos à fond
        (`remorque.volets_tours_min`, ×6 en °/s ; supposée proportionnelle)."""
        v = self.profil.valeur
        degres_s = (max(5.0, float(v("remorque.volets_vitesse", 40))) / 100
                    * float(v("remorque.volets_tours_min", 100.0)) * 6)
        return max(0.05, float(v("remorque.volets_angle_deg", 90)) / max(degres_s, 1.0))

    def arreter_volets(self) -> None:
        self._ecrire("volg -1")
        self._ecrire("vold -1")

    def _sequencer(self, roue: int, cible: int) -> None:
        """ROVER_2 : passage par zéro et attente avant de changer le sens d'une roue."""
        maintenant = time.monotonic()
        nom = "gd"[roue]
        sens_voulu = 1 if cible < 0 else 0
        valeur = abs(cible)
        if valeur and sens_voulu != self._sens[roue]:
            if self._applique[roue] != 0:
                self._ecrire(f"{nom} 0")
                self._applique[roue] = 0
                self._zero_depuis[roue] = maintenant
                return
            attente = float(self.profil.valeur("traction.delai_inversion_s", 0.6))
            if maintenant - self._zero_depuis[roue] < attente:
                return
            self._ecrire(f"sens{nom} {sens_voulu}")
            self._sens[roue] = sens_voulu
        if valeur != self._applique[roue]:
            self._ecrire(f"{nom} {valeur}")
            if valeur == 0:
                self._zero_depuis[roue] = maintenant
            self._applique[roue] = valeur

    @staticmethod
    def _fmt_servo(angle: float) -> str:
        return f"{angle:.1f}"

    def _ecrire(self, ligne: str) -> None:
        if not self._verrou_ecriture.acquire(timeout=0.05):
            self._ecritures_sautees += 1
            return
        try:
            self._t.write(ligne.encode("ascii") + b"\n")
        except Exception as erreur:  # délai d'écriture dépassé (port plein) : ligne sautée
            if type(erreur).__name__ != "SerialTimeoutException":
                raise
            self._ecritures_sautees += 1
        finally:
            self._verrou_ecriture.release()

    # ------------------------------------------------------------------ lecture

    def _lire(self) -> None:
        tampon = b""
        try:
            while self._actif:
                morceau = self._t.read(512)
                if not morceau:
                    self._verifier_presence()
                    continue
                tampon += morceau
                *lignes, tampon = tampon.split(b"\n")
                if len(tampon) > 1024:
                    tampon = b""
                for brute in lignes:
                    ligne = brute.decode("utf-8", "replace").strip()
                    if ligne:
                        self._traiter(ligne)
        except (OSError, ValueError) as erreur:  # SerialException hérite d'OSError
            self.erreur = str(erreur)
        finally:
            self._actif = False
            self.etat = replace(self.etat, t=time.monotonic(), connecte=False)
            self.bus.publier(S.ESP, self.etat)

    def _verifier_presence(self) -> None:
        if self.etat.connecte and not self.connecte():
            self.etat = replace(self.etat, t=time.monotonic(), connecte=False)
            self.bus.publier(S.ESP, self.etat)
            self.bus.journaliser(self.nom, "plus aucune ligne de l'ESP", "alerte")

    def _traiter(self, ligne: str) -> None:
        maintenant = time.monotonic()
        self._derniere_ligne = maintenant
        if ligne.startswith("T "):
            self._telemetrie(ligne, maintenant)
            return
        if ligne.startswith("I "):
            self._centrale_rapide(ligne, maintenant)
            return
        if m := LIGNE_DRONE_JPG.match(ligne):
            self._traiter_drone_jpg(
                int(m.group(1)), int(m.group(2)), int(m.group(3)), m.group(4), maintenant
            )
            return
        if ligne.startswith("[DRONE] "):
            self._traiter_drone(ligne[8:], maintenant)
            return
        if ligne.startswith("[DRONE_SCAN] "):
            self._traiter_drone_scan(ligne[13:], maintenant)
            self.bus.publier(S.CONSOLE_ESP, (time.time(), ligne))
            return
        self.bus.publier(S.CONSOLE_ESP, (time.time(), ligne))
        if m := LIGNE_IMU_TEXTE.match(ligne):
            if m.group(1) == "ABSENT":
                pass  # centrale non détectée par l'ESP : ses zéros ne sont pas des mesures
            elif not self.rover3 or not self._configure:
                v = [float(x) for x in m.groups()[1:]]
                self._publier_imu(
                    maintenant, (v[0], v[1], v[2]), (v[3], v[4], v[5]), None, v[6], m.group(1)
                )
            self._mettre_a_jour(maintenant)
        elif m := LIGNE_FIRMWARE.match(ligne):
            ancienne = self.version
            self.version = m.group(1)
            if ancienne != self.version:
                self._configure = False
                self.bus.journaliser(
                    self.nom,
                    f"firmware {self.version}"
                    + (
                        ""
                        if self.rover3
                        else " : pas de chien de garde, "
                        f"traction plafonnée à {PWM_MAX_SANS_CHIEN} %"
                    ),
                    "succes" if self.rover3 else "alerte",
                )
            self._mettre_a_jour(maintenant, firmware=self.version)
        elif m := LIGNE_MOTEURS.match(ligne):
            self._mettre_a_jour(maintenant, frein=m.group(3) == "1", defaut=m.group(4) == "1")
        elif m := LIGNE_ROUE.match(ligne):
            cote = "g" if m.group(1) == "GAUCHE" else "d"
            if self._permutes():
                cote = "d" if cote == "g" else "g"
            pwm = max(0, int(m.group(3)))
            sens = int(m.group(4)) ^ int(self._sens_inverse())
            champs: dict[str, Any] = {
                f"pwm_{cote}": pwm,
                f"sens_{cote}": sens,
                f"inverse_{cote}": m.group(6) == "1",
            }
            self._mettre_a_jour(maintenant, **champs)
            if not self.rover3:  # resynchroniser le séquenceur sur l'état réel
                roue = 0 if cote == "g" else 1
                self._sens[roue] = int(m.group(4))  # sens de la sortie, pas de la roue
        elif m := LIGNE_SERVO.match(ligne):
            self._mettre_a_jour(
                maintenant,
                servo_actif=m.group(1) == "actif",
                servo_deg=float(m.group(2)),
                servo_cible_deg=float(m.group(3)),
            )
        elif m := LIGNE_DEFAUT.match(ligne):
            raison = m.group(1).strip()
            if not self.etat.defaut:
                self.bus.journaliser(self.nom, f"défaut verrouillé de l'ESP : {raison}", "erreur")
            self._mettre_a_jour(maintenant, defaut=True, raison_defaut=raison)
        elif m := LIGNE_CHIEN.match(ligne):
            if "expire" in m.group(1):
                if self._traction_demandee:
                    self.bus.journaliser(
                        self.nom, "chien de garde de l'ESP expiré : traction coupée", "alerte"
                    )
                else:  # à l'arrêt : seulement un retard du Pi, sans effet ; reprise au prochain p
                    self.bus.journaliser(
                        self.nom,
                        "chien de garde de l'ESP : le Pi a tardé plus "
                        "de 400 ms (traction déjà à zéro, sans effet)",
                        "info",
                    )
        elif ligne.startswith("=== FieldMind") and self.version:
            # Accueil : l'ESP vient (re)démarrer. Il faut le reconfigurer.
            self._configure = False
            self._servo_envoye = None
            self._applique = [0, 0]
            self.envoyer_ligne("etat", journal=False)

    def _traiter_drone(self, contenu: str, maintenant: float) -> None:
        # Si le drone est connecté (ou en cours de liaison) en BLE sur le Raspberry Pi,
        # ne pas écraser l'état BLE par l'état Wi-Fi inactif de l'ESP32-C5.
        if self._ble_connecte or (maintenant - self._ble_derniere_activite_t < 8.0):
            return
        kv: dict[str, str] = {}
        for morceau in contenu.split():
            if "=" in morceau:
                cle, val = morceau.split("=", 1)
                kv[cle] = val
        etat = kv.get("etat", self.etat_drone.etat)
        connecte = etat == "connecte"
        ancien_etat = self.etat_drone.etat
        ssid = kv.get("ssid")
        if ssid in ("-", ""):
            ssid = None
        bssid = kv.get("bssid")
        if bssid in ("-", ""):
            bssid = None
        ip = kv.get("ip")
        if ip in ("-", ""):
            ip = None
        gw = kv.get("gw")
        if gw in ("-", ""):
            gw = None
        proto = kv.get("proto")
        if proto in ("-", ""):
            proto = None
        try:
            bat = int(kv.get("bat", "-1"))
        except ValueError:
            bat = -1
        try:
            rssi = int(kv.get("rssi", "0"))
        except ValueError:
            rssi = 0
        try:
            ips = float(kv.get("ips", "0"))
        except ValueError:
            ips = 0.0
        self.etat_drone = replace(
            self.etat_drone,
            t=maintenant,
            etat=etat,
            connecte=connecte,
            ssid=ssid,
            bssid=bssid,
            canal=_entier(kv.get("canal"), 0),
            rssi_dbm=rssi if rssi < 0 else None,
            ip_locale=ip,
            passerelle=gw,
            protocole=proto,
            port_video=_entier(kv.get("port_video"), 0),
            port_cmd=_entier(kv.get("port_cmd"), 8888),
            paquets_rx=_entier(kv.get("rx_pkts"), 0),
            ko_rx=_entier(kv.get("rx_kb"), 0),
            paquets_tx=_entier(kv.get("tx_pkts"), 0),
            ips=ips,
            images_rx=_entier(kv.get("img"), 0),
            batterie_pct=bat if 0 <= bat <= 100 else None,
            commande_vol=kv.get("cmd", "aucune"),
            reseaux_scan=self._ble_vus + self._wifi_vus,
            age_s=0.0,
        )
        if ancien_etat != etat:
            if connecte:
                self.bus.journaliser(
                    "drone",
                    f"connecté au Wi-Fi du drone « {ssid or '?'} » (IP {ip}, passerelle {gw})",
                    "succes",
                )
            elif ancien_etat == "connecte":
                self.recon.interrompre("Liaison Wi-Fi perdue — essai interrompu")
                self.bus.journaliser("drone", "Wi-Fi du drone déconnecté", "alerte")
        self.bus.publier(S.DRONE, self.etat_drone)
        # Tant qu'aucune trame JPEG réelle n'arrive depuis 1,5 s, publier une mire HUD à ~1 Hz
        if (
            maintenant - self._derniere_image_drone_t > 1.5
            and maintenant - self._derniere_mire_drone_t >= 1.0
        ):
            self._derniere_mire_drone_t = maintenant
            self._publier_mire_drone(maintenant)

    def _traiter_drone_scan(self, contenu: str, maintenant: float) -> None:
        kv: dict[str, str] = {}
        for morceau in contenu.split():
            if "=" in morceau:
                cle, val = morceau.split("=", 1)
                kv[cle] = val
        candidat = kv.get("Candidat")
        if candidat in ("aucun", "-", ""):
            candidat = None
        brut = kv.get("reseaux", "")
        self._wifi_vus = [r for r in brut.split(";") if r and r != "aucun"]
        self.etat_drone = replace(
            self.etat_drone,
            t=maintenant,
            candidat_scan=candidat or self.etat_drone.candidat_scan,
            reseaux_scan=self._ble_vus + self._wifi_vus,
            age_s=0.0,
        )
        self.bus.publier(S.DRONE, self.etat_drone)

    def _traiter_drone_jpg(
        self, seq: int, idx: int, total: int, b64: str, maintenant: float
    ) -> None:
        if seq != self._drone_seq or total != self._drone_total:
            self._drone_seq = seq
            self._drone_total = total
            self._drone_tranches.clear()
        self._drone_tranches[idx] = b64
        if len(self._drone_tranches) < total:
            return
        try:
            complet = "".join(self._drone_tranches[i] for i in range(1, total + 1))
            jpeg = base64.b64decode(complet)
        except Exception:
            self._drone_tranches.clear()
            return
        self._drone_tranches.clear()
        if len(jpeg) < 128 or not jpeg.startswith(b"\xff\xd8"):
            return
        self._drone_num_image += 1
        self._derniere_image_drone_t = maintenant
        img = ImageCamera(
            t=maintenant,
            camera="drone",
            jpeg=jpeg,
            largeur=self.etat_drone.largeur,
            hauteur=self.etat_drone.hauteur,
            numero=self._drone_num_image,
        )
        self.recon.deposer(img)

    def _publier_mire_drone(self, maintenant: float) -> None:
        jpeg = _generer_mire_drone(self.etat_drone)
        if not jpeg:
            return
        self._drone_num_image += 1
        self.bus.publier(
            S.IMAGE_DRONE,
            ImageCamera(
                t=maintenant,
                camera="drone",
                jpeg=jpeg,
                largeur=640,
                hauteur=360,
                numero=self._drone_num_image,
            ),
        )

    # ---------------------------------------------------------- drone BLE (CooleerX / BL616)

    def _lire_drone_ble(self) -> None:
        """Connexion directe BLE 5.0 (BlueZ D-Bus AcquireNotify) au drone CooleerX
        (B4:E8:42:...)."""
        try:
            import os
            import select

            import cv2
            import dbus
            import numpy as np
        except ImportError:
            return

        UUID_SERVICE_DRONE = "07af27a5-9c22-11ea-9afe-02fcdc4e7412"
        UUID_CHAR_VIDEO = "07af27a9-9c22-11ea-9afe-02fcdc4e7412"
        UUID_CHAR_CMD = "07af27a7-9c22-11ea-9afe-02fcdc4e7412"
        PREFIXES_BLE_DRONE = (
            "cooleerx",
            "hd-drone",
            "hd_drone",
            "ufo",
            "fpv",
            "drone",
            "e88",
            "e99",
            "flow",
        )

        try:
            bus_sys = dbus.SystemBus()
        except Exception:
            return

        paquets_rx = 0
        octets_rx = 0
        paquets_tx = 0
        images_rx = 0
        ips_courant = 0.0
        batterie_pct: int | None = None
        esp_scan_coupe = False

        while self._actif:
            try:
                om = dbus.Interface(
                    bus_sys.get_object("org.bluez", "/"), "org.freedesktop.DBus.ObjectManager"
                )
                objets = om.GetManagedObjects()
                adaptateur_path = None
                adaptateur_mac = "hci0"
                dev_cible_path = None
                dev_cible_nom = "CooleerX"
                dev_cible_mac = ""
                dev_cible_rssi: int | None = None
                ble_liste: list[str] = []

                for path, ifaces in objets.items():
                    if "org.bluez.Adapter1" in ifaces and adaptateur_path is None:
                        adaptateur_path = str(path)
                        adaptateur_mac = str(ifaces["org.bluez.Adapter1"].get("Address", "hci0"))
                    if "org.bluez.Device1" in ifaces:
                        props = ifaces["org.bluez.Device1"]
                        mac = str(props.get("Address", ""))
                        nom = str(props.get("Name", props.get("Alias", "")))
                        rssi = int(props["RSSI"]) if "RSSI" in props else None
                        uuids = [str(u).lower() for u in props.get("UUIDs", [])]
                        est_connecte = bool(props.get("Connected", False))

                        if nom and nom.replace("-", ":").upper() != mac.upper():
                            etiq = f"BLE:{nom}({rssi if rssi is not None else '?'}dBm,{mac})"
                            ble_liste.append(etiq)

                        correspond = False
                        if self._ble_cible:
                            c = self._ble_cible.lower()
                            correspond = (c in nom.lower()) or (c in mac.lower())
                        else:
                            correspond = (
                                any(nom.lower().startswith(p) for p in PREFIXES_BLE_DRONE)
                                or mac.upper().startswith("B4:E8:42:")
                                or UUID_SERVICE_DRONE in uuids
                            )
                        if correspond and (
                            dev_cible_path is None or est_connecte or rssi is not None
                        ):
                            dev_cible_path = str(path)
                            dev_cible_nom = nom or "CooleerX"
                            dev_cible_mac = mac
                            if rssi is not None:
                                dev_cible_rssi = rssi

                self._ble_vus = ble_liste[:8]

                if self._ble_deconnecter_demande:
                    self._ble_deconnecter_demande = False
                    if dev_cible_path:
                        try:
                            dbus.Interface(
                                bus_sys.get_object("org.bluez", dev_cible_path), "org.bluez.Device1"
                            ).Disconnect()
                        except Exception:
                            pass
                    self._ble_connecte = False
                    time.sleep(1.0)
                    continue

                # Si le drone BLE n'est pas encore visible ou si un scan est demandé,
                # lancer une découverte BLE
                if adaptateur_path and (dev_cible_path is None or self._ble_scan_demande):
                    self._ble_scan_demande = False
                    adaptateur = dbus.Interface(
                        bus_sys.get_object("org.bluez", adaptateur_path), "org.bluez.Adapter1"
                    )
                    try:
                        adaptateur.StartDiscovery()
                    except Exception:
                        pass
                    time.sleep(2.5)
                    try:
                        adaptateur.StopDiscovery()
                    except Exception:
                        pass
                    continue

                if dev_cible_path is None:
                    time.sleep(1.5)
                    continue

                dev_obj = bus_sys.get_object("org.bluez", dev_cible_path)
                dev_iface = dbus.Interface(dev_obj, "org.bluez.Device1")
                dev_props = dbus.Interface(dev_obj, "org.freedesktop.DBus.Properties")

                est_connecte = bool(dev_props.Get("org.bluez.Device1", "Connected"))
                if not est_connecte:
                    maintenant = time.monotonic()
                    self._ble_derniere_activite_t = maintenant
                    self.etat_drone = replace(
                        self.etat_drone,
                        t=maintenant,
                        etat="connexion",
                        connecte=False,
                        ssid=f"{dev_cible_nom} (BLE)",
                        bssid=dev_cible_mac,
                        rssi_dbm=dev_cible_rssi,
                        candidat_scan=f"{dev_cible_nom} ({dev_cible_mac})",
                        reseaux_scan=self._ble_vus + self._wifi_vus,
                        age_s=0.0,
                    )
                    self.bus.publier(S.DRONE, self.etat_drone)
                    try:
                        dev_iface.Connect()
                    except Exception:
                        # Le drone est peut-être hors de portée : relancer un court scan
                        # au tour suivant
                        self._ble_scan_demande = True
                        time.sleep(1.5)
                        continue

                # Attendre la résolution des services GATT
                for _ in range(35):
                    if not self._actif:
                        return
                    if bool(dev_props.Get("org.bluez.Device1", "ServicesResolved")):
                        break
                    time.sleep(0.1)

                # Chercher les caractéristiques vidéo (07af27a9) et commande (07af27a7)
                objets = om.GetManagedObjects()
                char_video_path = None
                char_cmd_path = None
                for path, ifaces in objets.items():
                    if not str(path).startswith(dev_cible_path + "/"):
                        continue
                    if "org.bluez.GattCharacteristic1" in ifaces:
                        u = str(ifaces["org.bluez.GattCharacteristic1"].get("UUID", "")).lower()
                        if u == UUID_CHAR_VIDEO:
                            char_video_path = str(path)
                        elif u == UUID_CHAR_CMD:
                            char_cmd_path = str(path)

                if not char_video_path:
                    time.sleep(1.0)
                    continue

                char_video = dbus.Interface(
                    bus_sys.get_object("org.bluez", char_video_path),
                    "org.bluez.GattCharacteristic1",
                )
                char_cmd = (
                    dbus.Interface(
                        bus_sys.get_object("org.bluez", char_cmd_path),
                        "org.bluez.GattCharacteristic1",
                    )
                    if char_cmd_path
                    else None
                )

                fd_obj, mtu_dbus = char_video.AcquireNotify(dbus.Dictionary({}, signature="sv"))
                fd = fd_obj.take()
                mtu = int(mtu_dbus)
                try:
                    os.set_blocking(fd, False)
                except OSError:
                    pass

                # Couper le scan Wi-Fi de l'ESP32-C5 pendant que le flux BLE est actif
                if not esp_scan_coupe:
                    try:
                        self._ecrire("drone deconnecter")
                        esp_scan_coupe = True
                    except Exception:
                        pass

                self._ble_cmd_file.clear()
                self._ble_connecte = True
                maintenant = time.monotonic()
                self._ble_derniere_activite_t = maintenant
                self.bus.journaliser(
                    "drone",
                    f"connecté au drone BLE « {dev_cible_nom} » ({dev_cible_mac}, MTU {mtu})",
                    "succes",
                )

                hdr_chunks: list[bytes] = []
                en_reception_hdr = False
                bandes: dict[int, tuple[int, dict[int, bytes]]] = {}
                bandes_valides: set[int] = set()
                toile = np.zeros((320, 240, 3), dtype=np.uint8)
                fenetre_t = maintenant
                fenetre_img = 0
                dernier_etat_t = 0.0
                dernier_rssi_t = 0.0
                dernier_paquet_t = maintenant

                try:
                    while (
                        self._actif
                        and not self._ble_deconnecter_demande
                        and not self._ble_scan_demande
                    ):
                        # Envoyer les éventuelles commandes de vol sur char_cmd (07af27a7)
                        while self._ble_cmd_file and char_cmd is not None:
                            pkt_cmd = self._ble_cmd_file.popleft()
                            try:
                                char_cmd.WriteValue(
                                    dbus.Array(pkt_cmd, signature="y"),
                                    dbus.Dictionary({"type": "command"}, signature="sv"),
                                )
                                paquets_tx += 1
                            except Exception:
                                pass

                        r, _, _ = select.select([fd], [], [], 0.20)
                        maintenant = time.monotonic()
                        if not r:
                            if maintenant - dernier_paquet_t > 3.5:
                                break
                            continue

                        # Vider tous les paquets disponibles dans le tube noyau pour éviter
                        # tout engorgement de bluetoothd
                        paquets_lus: list[bytes] = []
                        for _ in range(32):
                            try:
                                pkt = os.read(fd, 512)
                            except BlockingIOError:
                                break
                            except OSError:
                                pkt = b""
                                break
                            if not pkt:
                                break
                            paquets_lus.append(pkt)

                        if not paquets_lus:
                            break

                        dernier_paquet_t = maintenant
                        self._ble_derniere_activite_t = maintenant

                        for pkt in paquets_lus:
                            paquets_rx += 1
                            octets_rx += len(pkt)

                            if (
                                len(pkt) >= 7 and pkt[0] == 0x48
                            ):  # 'H' : en-tête JPEG (tables DQT/SOF0/DHT/SOS)
                                if not en_reception_hdr:
                                    hdr_chunks.clear()
                                    en_reception_hdr = True
                                hdr_chunks.append(pkt[6:])
                            elif (
                                len(pkt) >= 7 and pkt[0] == 0x53 and hdr_chunks
                            ):  # 'S' : bande JPEG 240x40
                                en_reception_hdr = False
                                sid, tot, idx = pkt[1], pkt[2], pkt[3]
                                # S[4] identifie le modèle/résolution dans HD dragon,
                                # pas la batterie (84 = SPI_KL41_8K). Niveau inconnu.
                                if sid == 1 and idx == 1:
                                    bandes_valides.clear()
                                if idx == 1 or sid not in bandes or bandes[sid][0] != tot:
                                    bandes[sid] = (tot, {})
                                bandes[sid][1][idx] = pkt[6:]
                                if len(bandes[sid][1]) == tot and 1 <= sid <= 8:
                                    jpeg_bande = b"".join(hdr_chunks) + b"".join(
                                        bandes[sid][1][k]
                                        for k in range(1, tot + 1)
                                        if k in bandes[sid][1]
                                    )
                                    arr = np.frombuffer(jpeg_bande, dtype=np.uint8)
                                    im_bande = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                                    if im_bande is not None and im_bande.shape[:2] == (40, 240):
                                        y0 = (sid - 1) * 40
                                        toile[y0 : y0 + 40, :, :] = im_bande
                                        bandes_valides.add(sid)
                                        if sid == 8 and len(bandes_valides) == 8:
                                            # Le capteur CooleerX arrive en portrait, décalé
                                            # d'un quart de tour : corriger avant publication
                                            # pour le flux, les captures et la vision du drone.
                                            orientee = cv2.rotate(toile, cv2.ROTATE_90_CLOCKWISE)
                                            ok, buf = cv2.imencode(
                                                ".jpg",
                                                orientee,
                                                [int(cv2.IMWRITE_JPEG_QUALITY), 88],
                                            )
                                            if ok:
                                                images_rx += 1
                                                fenetre_img += 1
                                                self._drone_num_image += 1
                                                self._derniere_image_drone_t = maintenant
                                                self.recon.deposer(
                                                    ImageCamera(
                                                        t=maintenant,
                                                        camera="drone",
                                                        jpeg=bytes(buf),
                                                        largeur=320,
                                                        hauteur=240,
                                                        numero=self._drone_num_image,
                                                    ),
                                                )

                        if maintenant - fenetre_t >= 1.0:
                            ips_courant = round(fenetre_img / (maintenant - fenetre_t), 1)
                            fenetre_img = 0
                            fenetre_t = maintenant

                        if maintenant - dernier_etat_t >= 0.4:
                            dernier_etat_t = maintenant
                            if maintenant - dernier_rssi_t >= 4.0:
                                dernier_rssi_t = maintenant
                                try:
                                    dev_cible_rssi = int(dev_props.Get("org.bluez.Device1", "RSSI"))
                                except Exception:
                                    pass
                            self.etat_drone = replace(
                                self.etat_drone,
                                t=maintenant,
                                etat="connecte",
                                connecte=True,
                                ssid=f"{dev_cible_nom} (BLE 5.0)",
                                bssid=dev_cible_mac,
                                canal=2,
                                rssi_dbm=dev_cible_rssi if dev_cible_rssi is not None else -38,
                                ip_locale=adaptateur_mac,
                                passerelle=dev_cible_mac,
                                protocole=f"BLE-GATT-MJPEG (MTU {mtu})",
                                port_video=mtu,
                                port_cmd=0x27A7,
                                paquets_rx=paquets_rx,
                                ko_rx=octets_rx // 1024,
                                paquets_tx=paquets_tx,
                                ips=ips_courant,
                                images_rx=images_rx,
                                largeur=240,
                                hauteur=320,
                                batterie_pct=batterie_pct,
                                candidat_scan=f"{dev_cible_nom} ({dev_cible_mac})",
                                reseaux_scan=self._ble_vus + self._wifi_vus,
                                age_s=0.0,
                            )
                            self.bus.publier(S.DRONE, self.etat_drone)
                finally:
                    try:
                        os.close(fd)
                    except OSError:
                        pass
                    self._ble_connecte = False
                    self._ble_cmd_file.clear()
                    self.recon.interrompre("Liaison BLE perdue — essai interrompu")
                    maintenant = time.monotonic()
                    self.etat_drone = replace(
                        self.etat_drone,
                        t=maintenant,
                        etat="scan",
                        connecte=False,
                        ips=0.0,
                        age_s=0.0,
                    )
                    self.bus.publier(S.DRONE, self.etat_drone)
                    self.bus.journaliser(
                        "drone", f"drone BLE « {dev_cible_nom} » déconnecté", "alerte"
                    )
            except Exception:
                self._ble_connecte = False
                self._ble_cmd_file.clear()
                self.recon.interrompre("Liaison BLE indisponible — essai interrompu")
                time.sleep(1.5)

    def _permutes(self) -> bool:
        """Moteurs gauche et droite branchés sur les sorties de l'autre côté (réglage) : les
        commandes et les mesures sont échangées ici, le reste du logiciel parle des vraies
        roues."""
        return bool(self.profil.valeur("esp.moteurs_permutes", False))

    def _sens_inverse(self) -> bool:
        """Les deux moteurs tournent à l'envers (réglage) : commandes et mesures retournées ici."""
        return bool(self.profil.valeur("esp.sens_inverse", False))

    def _telemetrie(self, ligne: str, maintenant: float) -> None:
        champs = ligne.split()
        if len(champs) < 8:
            return
        if self._permutes():
            champs[2], champs[3] = champs[3], champs[2]
            if len(champs) >= 10:
                champs[8], champs[9] = champs[9], champs[8]
        signe = -1 if self._sens_inverse() else 1
        try:
            g, d = signe * int(champs[2]), signe * int(champs[3])
            self._mettre_a_jour(
                maintenant,
                pwm_g=abs(g),
                pwm_d=abs(d),
                sens_g=int(g < 0),
                sens_d=int(d < 0),
                servo_deg=float(champs[4]),
                servo_actif=True,
                frein=champs[5] == "1",
                defaut=champs[6] == "1",
                chien_de_garde_ms=int(self.profil.valeur("esp.chien_de_garde_ms", 400))
                if champs[7] == "1"
                else 0,
                hz_g=signe * float(champs[8]) if len(champs) >= 10 else None,
                hz_d=signe * float(champs[9]) if len(champs) >= 10 else None,
            )
        except ValueError:
            return

    def _centrale_rapide(self, ligne: str, maintenant: float) -> None:
        champs = ligne.split()
        if len(champs) < 9:
            return
        try:
            v = [float(x) for x in champs[2:9]]
        except ValueError:
            return
        self._publier_imu(maintenant, (v[0], v[1], v[2]), (v[3], v[4], v[5]), v[6], None, "OK")

    def _publier_imu(
        self,
        t: float,
        accel: tuple[float, float, float],
        gyro_dps: tuple[float, float, float],
        lacet_deg: float | None,
        temperature: float | None,
        etat: str,
    ) -> None:
        if not bool(self.profil.valeur("capteurs.imu", True)):
            return  # centrale coupée depuis le poste
        a = self._axes(accel)
        w = self._axes(tuple(math.radians(x) for x in gyro_dps))  # type: ignore[arg-type]
        lacet = None if lacet_deg is None else self._signe_lacet * math.radians(lacet_deg)
        niveau = self._rotation_niveau()
        if niveau is not None:
            a = tuple(float(x) for x in niveau @ np.asarray(a))  # type: ignore[assignment]
            w = tuple(float(x) for x in niveau @ np.asarray(w))  # type: ignore[assignment]
            lacet = None  # lacet du firmware intégré autour de l'axe z du capteur, penché
        self.bus.publier(
            S.IMU,
            Imu(
                t=t, accel_g=a, gyro_rad_s=w, lacet_rad=lacet, temperature_c=temperature, etat=etat
            ),
        )

    def _rotation_niveau(self) -> np.ndarray | None:
        """Mise à niveau enregistrée par la calibration « centrale au repos » (imu.niveau :
        gravité « gx gy gz » dans le repère des axes du profil), ou None."""
        texte = str(self.profil.valeur("imu.niveau", "") or "").strip()
        if texte != self._niveau_texte:
            self._niveau_texte = texte
            try:
                g = [float(x) for x in texte.split()]
                self._niveau = rotation_niveau(g) if len(g) == 3 else None
            except ValueError:
                self._niveau = None
        return self._niveau

    def _mettre_a_jour(self, maintenant: float, **champs: Any) -> None:
        self.etat = replace(
            self.etat, t=maintenant, connecte=True, age_derniere_ligne_s=0.0, **champs
        )
        self.bus.publier(S.ESP, self.etat)


def _numero(version: str) -> int:
    m = re.search(r"(\d+)$", version)
    return int(m.group(1)) if m else 0


def _entier(val: str | None, defaut: int = 0) -> int:
    if val is None:
        return defaut
    try:
        return int(val)
    except ValueError:
        return defaut


def _paquet_vol_ble(cmd: str) -> bytes | None:
    if cmd not in DRAPEAUX_ACTION:
        return None
    return paquet_rc(action=cmd)


def _generer_mire_drone(d: EtatDrone) -> bytes | None:
    try:
        import cv2
        import numpy as np
    except ImportError:
        return None
    img = np.full((360, 640, 3), (18, 20, 24), dtype=np.uint8)
    # Grille de visée FPV
    cv2.line(img, (320, 40), (320, 320), (38, 44, 52), 1)
    cv2.line(img, (60, 180), (580, 180), (38, 44, 52), 1)
    cv2.circle(img, (320, 180), 48, (48, 56, 68), 1)

    police = cv2.FONT_HERSHEY_SIMPLEX
    couleur_etat = (
        (80, 200, 120)
        if d.connecte
        else (70, 175, 245)
        if d.etat in ("scan", "connexion")
        else (130, 140, 155)
    )
    titre = f"DRONE HD-DRONE / CooleerX (BLE + Wi-Fi) : {d.etat.upper()}"
    cv2.putText(img, titre, (24, 42), police, 0.60, couleur_etat, 2, cv2.LINE_AA)

    lignes = [
        f"Liaison : {d.ssid or 'recherche auto BLE CooleerX / Wi-Fi...'}"
        f"   Canal : {d.canal or '-'}",
        f"Signal RSSI : {f'{d.rssi_dbm} dBm' if d.rssi_dbm is not None else '-'}"
        f"   Batterie : {f'{d.batterie_pct}%' if d.batterie_pct is not None else 'N/D'}",
        f"Hote : {d.ip_locale or '-'}   Drone MAC/IP : {d.passerelle or d.bssid or '-'}",
        f"Protocole : {d.protocole or 'attente'}   MTU/Port : {d.port_video or '-'}",
        f"Paquets RX : {d.paquets_rx} ({d.ko_rx} ko)   TX : {d.paquets_tx}"
        f"   Images JPEG : {d.images_rx} ({d.ips:.1f} i/s)",
    ]
    y = 95
    for texte in lignes:
        cv2.putText(img, texte, (28, y), police, 0.48, (215, 222, 230), 1, cv2.LINE_AA)
        y += 32

    if d.reseaux_scan:
        apercu = ", ".join(d.reseaux_scan[:4])
        cv2.putText(
            img,
            f"Detectes : {apercu[:68]}",
            (28, 285),
            police,
            0.44,
            (150, 170, 190),
            1,
            cv2.LINE_AA,
        )
    else:
        cv2.putText(
            img,
            "Allumez le drone : connexion automatique BLE (CooleerX) ou Wi-Fi",
            (28, 285),
            police,
            0.45,
            (150, 170, 190),
            1,
            cv2.LINE_AA,
        )

    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return bytes(buf) if ok else None


