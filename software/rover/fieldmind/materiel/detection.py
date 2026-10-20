"""Matériel réel : détection à chaud des appareils et démarrage de leurs pilotes.

Chaque seconde, on compare ce qui est branché à ce qui tourne :
- port série USB natif Espressif (303a:*) → ESP du rover ;
- pont USB-série (CP210x…) → lidar RPLIDAR s'il répond à GET_INFO, sinon GPS s'il parle NMEA ;
- UART du GPIO (/dev/ttyS0, si la console série est coupée) → GPS ;
- webcams UVC → gauche / droite selon leur port USB ;
- carte son USB → micro (démarré seulement en mode vocal).
Un pilote qui tombe est relancé 5 s plus tard, si l'appareil est toujours là.
"""

from __future__ import annotations

import glob
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.materiel import lidar as lidar_mod
from fieldmind.materiel.audio import PiloteMicro
from fieldmind.materiel.cameras import Camera, cameras_presentes
from fieldmind.materiel.esp import PiloteEsp, ouvrir_port
from fieldmind.materiel.gnss import PiloteGnss, nmea_valide
from fieldmind.materiel.lidar import PiloteLidar
from fieldmind.materiel.systeme import Systeme

DELAI_NOUVEL_ESSAI_S = 5.0
DELAI_APPAREIL_INCONNU_S = 20.0
ESPRESSIF = "303a"


def _usb_du_port(peripherique: str) -> dict[str, str] | None:
    dossier = Path(f"/sys/class/tty/{Path(peripherique).name}/device").resolve()
    for parent in (dossier, *dossier.parents):
        if (parent / "idVendor").exists():
            def lire(nom: str, d: Path = parent) -> str:
                try:
                    return (d / nom).read_text().strip()
                except OSError:
                    return ""
            return {"id": f"{lire('idVendor')}:{lire('idProduct')}", "produit": lire("product"),
                    "port": parent.name}
    return None


def _uart_gpio_libre() -> bool:
    try:
        sortie = subprocess.run(["systemctl", "is-active", "serial-getty@ttyS0"],
                                capture_output=True, text=True, timeout=3).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return False
    return sortie != "active"


def _ecouter_nmea(peripherique: str, debit: int = 9600, duree: float = 2.0) -> bool:
    try:
        port = ouvrir_port(peripherique, debit)
    except OSError:
        return False
    try:
        donnees, fin = b"", time.monotonic() + duree
        while time.monotonic() < fin and len(donnees) < 4096:
            donnees += port.read(512)
    finally:
        port.close()
    lignes = donnees.decode("latin-1").splitlines()
    return sum(nmea_valide(ligne.strip()) for ligne in lignes) >= 2


class MaterielReel:
    nom = "reel"

    def __init__(self, bus: Bus, profil: Profil) -> None:
        self.bus, self.profil = bus, profil
        self._systeme = Systeme()
        self._esp: PiloteEsp | None = None
        self._lidar: PiloteLidar | None = None
        self._gnss: PiloteGnss | None = None
        self._cameras: dict[str, Camera] = {}
        self._ports: dict[str, Any] = {}  # périphérique -> pilote
        self._inconnus: dict[str, float] = {}
        self._pannes: dict[str, float] = {}
        self.micro = PiloteMicro(bus, str(profil.valeur("audio.peripherique", "")),
                                 int(profil.valeur("audio.frequence_hz", 16000)))
        self._uart_gpio = _uart_gpio_libre()
        self._lidar_marche = bool(profil.valeur("capteurs.lidar", True))
        self._actif = True
        self._fil = threading.Thread(target=self._boucle, name="detection", daemon=True)
        self._verrou = threading.Lock()

    # ------------------------------------------------------------------ interface Materiel

    def demarrer(self) -> None:
        if not self._uart_gpio:
            self.bus.journaliser("uart", "UART du GPIO occupé par la console série : GPS sur "
                                 "ttyS0 non lu", "alerte")
        self._fil.start()

    def arreter(self) -> None:
        self._actif = False
        with self._verrou:
            if self._esp:
                self._esp.arreter()
            for pilote in (self._lidar, self._gnss):
                if pilote:
                    pilote.arreter()
            for camera in self._cameras.values():
                camera.arreter()
        self.micro.activer(False)

    def esp(self) -> PiloteEsp | None:
        esp = self._esp
        return esp if esp is not None and esp.actif else None

    def capteurs(self) -> dict[str, Any]:
        c: dict[str, Any] = {
            "lidar": self._lidar.resume() if self._lidar and self._lidar.actif else
            {"ok": False, "marche": self._lidar_marche},
            "gnss": self._gnss.resume() if self._gnss and self._gnss.actif else {"ok": False},
            "esp": {"ok": bool(self.esp() and self.esp().connecte()),  # type: ignore[union-attr]
                    "age_s": round(min(self.bus.age(S.ESP), 99), 2),
                    "firmware": self._esp.version if self._esp else None},
            "imu": {"ok": self.bus.age(S.IMU) < 1.5,
                    "age_s": round(min(self.bus.age(S.IMU), 99), 2)},
            "micro": self.micro.resume(),
        }
        for role in ("gauche", "droite"):
            camera = self._cameras.get(role)
            c[role] = camera.resume() if camera else {"ok": False}
        return c

    def systeme(self) -> dict[str, Any]:
        return self._systeme.etat()

    def inventaire(self) -> dict[str, Any]:
        return {
            "ports": {p: type(pilote).__name__ for p, pilote in self._ports.items()},
            "inconnus": list(self._inconnus),
            "cameras": {role: {"port": c.port, "nom": c.nom, "peripherique": c.peripherique}
                        for role, c in self._cameras.items()},
            "uart_gpio_libre": self._uart_gpio,
        }

    def lidar_marche(self, marche: bool) -> bool:
        self._lidar_marche = marche
        if self._lidar:
            self._lidar.marche = marche
            return True
        return False

    def micro_actif(self, actif: bool) -> bool:
        self.micro.activer(actif)
        return self.micro.actif == actif

    def cameras_inversees(self, inverse: bool) -> None:
        """Échange gauche et droite (enregistré par port USB), puis relance les caméras."""
        ports = sorted(cameras_presentes())
        actuels = {role: c.port for role, c in self._cameras.items()}
        gauche = actuels.get("gauche") or (ports[0] if ports else "")
        droite = actuels.get("droite") or (ports[1] if len(ports) > 1 else "")
        self.profil.regler("cameras.port_gauche", droite)
        self.profil.regler("cameras.port_droite", gauche)
        with self._verrou:
            for camera in self._cameras.values():
                camera.arreter()
            self._cameras.clear()

    # ------------------------------------------------------------------ détection

    def _boucle(self) -> None:
        tour = 0
        while self._actif:
            try:
                self._balayer_ports()
                self._balayer_cameras()
                if tour % 5 == 0:
                    self.bus.publier(S.MATERIEL, self.inventaire())
            except Exception as erreur:  # la détection ne doit jamais s'arrêter
                self.bus.journaliser("detection", f"erreur : {erreur!r}", "erreur")
            tour += 1
            time.sleep(1.0)

    def _peut_ouvrir(self, ident: str) -> bool:
        return time.monotonic() - self._pannes.get(ident, -1e9) > DELAI_NOUVEL_ESSAI_S

    def _balayer_ports(self) -> None:
        if self._gnss is not None and not bool(self.profil.valeur("capteurs.gnss", True)):
            self._gnss.arreter()  # GPS coupé depuis le poste : son port est refermé au tour suivant
        motifs = ["/dev/ttyACM*", "/dev/ttyUSB*"] + (["/dev/ttyS0"] if self._uart_gpio else [])
        presents = sorted({p for motif in motifs for p in glob.glob(motif)})
        with self._verrou:
            for peripherique, pilote in list(self._ports.items()):
                if peripherique not in presents or not pilote.actif:
                    pilote.arreter()
                    del self._ports[peripherique]
                    self._pannes[peripherique] = time.monotonic()
                    self.bus.journaliser("detection", f"{peripherique} fermé", "alerte")
                    if pilote is self._esp:
                        self._esp = None
                    elif pilote is self._lidar:
                        self._lidar = None
                    elif pilote is self._gnss:
                        self._gnss = None
        for peripherique in presents:
            if peripherique in self._ports or not self._peut_ouvrir(peripherique):
                continue
            if time.monotonic() - self._inconnus.get(peripherique, -1e9) < DELAI_APPAREIL_INCONNU_S:
                continue
            try:
                self._ouvrir(peripherique)
            except OSError as erreur:
                self._pannes[peripherique] = time.monotonic()
                self.bus.journaliser("detection", f"{peripherique} : {erreur}", "alerte")

    def _ouvrir(self, peripherique: str) -> None:
        usb = _usb_du_port(peripherique)
        if usb and usb["id"].startswith(ESPRESSIF) and self._esp is None:
            port = ouvrir_port(peripherique, int(self.profil.valeur("esp.debit_bauds", 115200)))
            esp = PiloteEsp(self.bus, self.profil, port)
            with self._verrou:
                self._ports[peripherique] = self._esp = esp
            esp.demarrer()
            self.bus.journaliser("detection", f"ESP du rover sur {peripherique}", "succes")
            return
        if usb and self._lidar is None:
            port = ouvrir_port(peripherique, int(self.profil.valeur("lidar.debit_bauds", 460800)))
            port.timeout = 0.2  # type: ignore[attr-defined]
            if lidar_mod.identifier(port) is not None:
                lidar = PiloteLidar(self.bus, self.profil, port)
                lidar.marche = self._lidar_marche
                with self._verrou:
                    self._ports[peripherique] = self._lidar = lidar
                lidar.demarrer()
                return
            port.close()
        if self._gnss is None and bool(self.profil.valeur("capteurs.gnss", True)) \
                and _ecouter_nmea(peripherique):
            port = ouvrir_port(peripherique, 9600)
            port.timeout = 0.5  # type: ignore[attr-defined]
            gnss = PiloteGnss(self.bus, port)
            with self._verrou:
                self._ports[peripherique] = self._gnss = gnss
            gnss.demarrer()
            self.bus.journaliser("detection", f"GPS sur {peripherique}", "succes")
            return
        self._inconnus[peripherique] = time.monotonic()
        self.bus.journaliser("detection", f"{peripherique} : appareil non reconnu "
                             f"({(usb or {}).get('produit') or 'UART'})", "alerte")

    def _balayer_cameras(self) -> None:
        presentes = cameras_presentes()
        gauche = str(self.profil.valeur("cameras.port_gauche", "") or "")
        droite = str(self.profil.valeur("cameras.port_droite", "") or "")
        ports = sorted(presentes)
        roles: dict[str, str] = {}
        if gauche in presentes:
            roles["gauche"] = gauche
        if droite in presentes and droite != gauche:
            roles["droite"] = droite
        for port in ports:  # ports non attribués : dans l'ordre, aux rôles libres
            if port in roles.values():
                continue
            for role in ("gauche", "droite"):
                if role not in roles:
                    roles[role] = port
                    break
        for role in ("gauche", "droite"):  # caméras coupées depuis le poste
            if not bool(self.profil.valeur(f"capteurs.camera_{role}", True)):
                roles.pop(role, None)
        with self._verrou:
            for role, camera in list(self._cameras.items()):
                if roles.get(role) != camera.port or not camera.actif:
                    camera.arreter()
                    del self._cameras[role]
                    if camera.erreur:
                        self._pannes[f"cam-{camera.port}"] = time.monotonic()
            for role, port in roles.items():
                if role in self._cameras or not self._peut_ouvrir(f"cam-{port}"):
                    continue
                peripherique, nom = presentes[port]
                camera = Camera(self.bus, role, peripherique, nom, port,
                                int(self.profil.valeur("cameras.largeur", 1280)),
                                int(self.profil.valeur("cameras.hauteur", 720)),
                                int(self.profil.valeur("cameras.ips", 15)))
                self._cameras[role] = camera
                camera.demarrer()
