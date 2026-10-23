"""Matériel simulé : le même logiciel tourne sur un PC, sans le rover.

- **Faux ESP** : il parle le protocole texte du firmware (ROVER_3, ou ROVER_2 pour tester le
  repli), chien de garde et inversion de sens compris. Le vrai `PiloteEsp` lui parle comme au
  vrai XIAO : le protocole est donc testé de bout en bout.
- **Physique** : modèle bicyclette, réponse du premier ordre des moteurs, zone morte et rapport
  PWM → vitesse volontairement différents du profil (pour exercer la calibration et la boucle de
  vitesse), dynamique du servo, collisions avec le monde (le rover reste bloqué).
- **Capteurs** : lidar (`Monde.lancer_lidar`), paire stéréo rendue (`Monde.rendre_camera`),
  centrale (lignes `I` du faux ESP), GPS bruité autour d'un point de référence, micro silencieux.
"""

from __future__ import annotations

import math
import random
import threading
import time
from collections import deque
from typing import Any

import cv2
import numpy as np

from fieldmind.bus import Bus, S
from fieldmind.config import Profil, braquage_max
from fieldmind.materiel.esp import PiloteEsp
from fieldmind.types import EchantillonAudio, Gnss, ImageCamera, Pose2D, Scan

REFERENCE_GPS = (43.7102, -1.0536)  # Dax
METRES_PAR_DEGRE_LAT = 111_320.0


class PhysiqueRover:
    """Rover cinématique : deux roues arrière motrices, direction avant au servo."""

    def __init__(self, profil: Profil, pose: Pose2D | None = None) -> None:
        g = profil.geometrie
        self.empattement = g.empattement_m
        self.voie = g.voie_arriere_m
        self.braquage_max = braquage_max(profil)
        self.servo_par_degre = float(profil.valeur("direction.servo_par_degre_roue", 1.0))
        # « Vérité » du simulateur, différente du profil (à retrouver par calibration)
        self.pwm_par_m_s = 38.0
        self.pwm_seuil = 5.0
        self.servo_neutre_reel = 1.5  # le rover tire un peu à gauche au neutre
        self.constante_s = 0.25
        self.gain_droit = 0.92  # moteur droit 8 % moins rapide à PWM égal
        self.sous_virage = 0.20  # roues avant qui glissent (herbe) : 20 % de lacet en moins
        self.effet_differentiel = 0.5  # part du lacet imposée par l'écart des roues arrière
        self.pose = pose or Pose2D()
        self.v_g = self.v_d = 0.0
        self.braquage = 0.0
        self.vitesse = 0.0
        self.vitesse_lacet = 0.0
        self.accel_long = 0.0
        self.bloque = False

    def vitesse_roue(self, pwm: float) -> float:
        utile = max(0.0, abs(pwm) - self.pwm_seuil)
        return math.copysign(utile / self.pwm_par_m_s, pwm) if utile else 0.0

    def avancer(self, dt: float, pwm_g: float, pwm_d: float, servo_deg: float,
                collision: Any = None) -> None:
        alpha = min(1.0, dt / self.constante_s)
        self.v_g += (self.vitesse_roue(pwm_g) - self.v_g) * alpha
        self.v_d += (self.gain_droit * self.vitesse_roue(pwm_d) - self.v_d) * alpha
        angle = math.radians((servo_deg - self.servo_neutre_reel) / self.servo_par_degre)
        self.braquage = max(-self.braquage_max, min(self.braquage_max, angle))
        v = 0.5 * (self.v_g + self.v_d)
        kappa = math.tan(self.braquage) / self.empattement
        # Les roues avant imposent le virage, en glissant un peu (sous-virage) ; un écart des
        # roues arrière différent de l'écart géométrique (v·κ·voie) pousse le rover à tourner
        # plus ou moins (comme un char).
        w = v * kappa * (1 - self.sous_virage)
        w += self.effet_differentiel * ((self.v_d - self.v_g) - v * kappa * self.voie) / self.voie
        p = self.pose
        cap = p.cap + w * dt
        suivante = Pose2D(p.x + v * math.cos(p.cap + 0.5 * w * dt) * dt,
                          p.y + v * math.sin(p.cap + 0.5 * w * dt) * dt,
                          math.atan2(math.sin(cap), math.cos(cap)))
        if collision is not None and abs(v) > 1e-4 and collision(suivante):
            self.bloque = True
            self.v_g = self.v_d = 0.0
            v = w = 0.0
        else:
            self.bloque = False
            self.pose = suivante
        self.accel_long = (v - self.vitesse) / dt if dt > 0 else 0.0
        self.vitesse, self.vitesse_lacet = v, w


class FauxEsp:
    """Émulation du firmware esp_c5 côté port série (méthodes read/write/close de pyserial)."""

    def __init__(self, version: str = "ROVER_3") -> None:
        self.version = version
        self._sortie: deque[bytes] = deque()
        self._condition = threading.Condition()
        self._entree = b""
        self.ouvert = True
        # état du firmware
        self.pwm = [0, 0]  # % appliqué, signé selon le sens logique
        self.demande = [0, 0]  # ROVER_2 : pourcentage demandé (non signé)
        self.sens = [0, 0]
        self.attente_inversion: list[tuple[float, int] | None] = [None, None]
        self.servo_deg = 0.0
        self.servo_cible = 0.0
        self.servo_actif = True
        self.servo_vitesse = 30.0
        self.volets: list[float | None] = [None, None]  # ROVER_4 : angle des servos D2/D3
        self.frein = False
        self.defaut = False
        self.chien_ms = 0
        self.dernier_battement = time.monotonic()
        self.chien_expire = False
        self.inv_ms = 500
        self.tel_hz = 0
        self.imu_hz = 0
        self._prochaine_tel = 0.0
        self._prochaine_imu = 0.0
        self._prochain_etat = 0.0
        self._prochain_texte = 0.0
        self.lacet_deg = 0.0
        self.imu = ((0.0, 0.0, 1.0), (0.0, 0.0, 0.0))  # accel g, gyro °/s (repère capteur)
        self._ecrire_lignes([f"=== FieldMind {version} - traction + servo D7 ===",
                             f"[FIRMWARE] {version}"])

    # ------------------------------------------------------------------ côté port série

    def read(self, n: int = 1) -> bytes:
        with self._condition:
            if not self._sortie:
                self._condition.wait(0.05)
            morceaux = b""
            while self._sortie and len(morceaux) < n:
                morceaux += self._sortie.popleft()
            return morceaux

    def write(self, donnees: bytes) -> int:
        if not self.ouvert:
            raise OSError("port fermé")
        self._entree += donnees
        *lignes, self._entree = self._entree.split(b"\n")
        for ligne in lignes:
            self._executer(ligne.decode("ascii", "replace").strip())
        return len(donnees)

    def close(self) -> None:
        self.ouvert = False

    def _ecrire_lignes(self, lignes: list[str]) -> None:
        with self._condition:
            for ligne in lignes:
                self._sortie.append(ligne.encode() + b"\n")
            self._condition.notify_all()

    # ------------------------------------------------------------------ firmware

    def _executer(self, texte: str) -> None:
        if not texte:
            return
        mots = texte.split()
        nom, args = mots[0], mots[1:]
        rover3 = self.version != "ROVER_2"
        try:
            if nom == "stop":
                self.pwm = [0, 0]
                self.demande = [0, 0]
                self.servo_actif = False
                self._ecrire_lignes(["[STOP] PWM moteur zero ; signal servo coupe."])
            elif nom == "etat":
                self._etat()
            elif nom == "m" and len(args) == 1:
                self.demande = [int(args[0])] * 2
                self.pwm = [self._signe(i, self.demande[i]) for i in range(2)]
            elif nom in ("g", "d") and len(args) == 1:
                i = 0 if nom == "g" else 1
                self.demande[i] = int(args[0])
                self.pwm[i] = self._signe(i, self.demande[i])
            elif nom in ("sensg", "sensd") and len(args) == 1:
                i = 0 if nom == "sensg" else 1
                if self.demande[i] != 0:
                    self._erreur("Mettre la roue concernee a zero et attendre son arret physique.")
                    return
                self.sens[i] = int(args[0])
            elif nom == "s" and len(args) == 1:
                self.servo_cible = float(args[0])
                self.servo_actif = True
            elif nom == "centre":
                self.servo_cible = 0.0
                self.servo_actif = True
            elif nom == "v" and len(args) == 1:
                self.servo_vitesse = float(args[0])
            elif rover3 and nom == "wd" and len(args) == 1:
                self.chien_ms = int(args[0])
                self.chien_expire = False
                self.dernier_battement = time.monotonic()
            elif rover3 and nom == "hb":
                self.dernier_battement = time.monotonic()
            elif rover3 and nom == "inv" and len(args) == 1:
                self.inv_ms = int(args[0])
            elif rover3 and nom == "tel" and len(args) == 1:
                self.tel_hz = int(args[0])
            elif rover3 and nom == "imu" and len(args) == 1:
                self.imu_hz = int(args[0])
            elif rover3 and nom == "lacet0":
                self.lacet_deg = 0.0
            elif (self.version.startswith("ROVER_") and int(self.version[6:]) >= 4
                  and nom in ("volg", "vold") and len(args) == 1):
                angle = float(args[0])
                self.volets[0 if nom == "volg" else 1] = None if angle < 0 else angle
            elif rover3 and nom == "p" and len(args) == 3:
                self.dernier_battement = time.monotonic()
                self.chien_expire = False
                for i in range(2):
                    self._piloter(i, int(args[i]))
                self.servo_cible = float(args[2])
                self.servo_actif = True
            else:
                self._erreur("Commande invalide.")
        except ValueError:
            self._erreur("Commande invalide.")

    def _signe(self, i: int, valeur: int) -> int:
        return -valeur if self.sens[i] else valeur

    def _piloter(self, i: int, valeur: int) -> None:
        if self.frein or self.defaut:
            self.pwm[i] = 0
            return
        sens = 1 if valeur < 0 else 0
        if valeur != 0 and sens != self.sens[i]:
            if self.pwm[i] != 0 or self.attente_inversion[i] is None:
                self.pwm[i] = 0
                if self.attente_inversion[i] is None:
                    self.attente_inversion[i] = (time.monotonic(), valeur)
                return
            self.attente_inversion[i] = (self.attente_inversion[i][0], valeur)
            return
        self.attente_inversion[i] = None
        self.pwm[i] = valeur

    def _erreur(self, message: str) -> None:
        self.pwm = [0, 0]
        self.demande = [0, 0]
        self._ecrire_lignes([f"[ERREUR] {message} Traction a zero, signal servo coupe."])

    def _etat(self) -> None:
        self._ecrire_lignes([
            f"[FIRMWARE] {self.version}",
            f"[MOTEURS] G={abs(self.pwm[0])} % ; D={abs(self.pwm[1])} % ; BRAKE={int(self.frein)}"
            f" ; defaut={int(self.defaut)}",
            *(f"[{nom}] Demande={abs(self.pwm[i])} % ; PWM={abs(self.pwm[i])} % ; "
              f"sens={self.sens[i]} ; DIR={self.sens[i]} ; inverse=0"
              for i, nom in enumerate(("GAUCHE", "DROITE"))),
            f"[SERVO] Signal {'actif' if self.servo_actif else 'coupe'} ; consigne en cours "
            f"{self.servo_deg:+.1f} ; cible {self.servo_cible:+.1f} ; vitesse max "
            f"{self.servo_vitesse:.1f} deg/s",
        ])

    def tick(self, t: float, dt: float) -> None:
        """Avance le temps du firmware (appelé par la simulation à 100 Hz)."""
        if self.chien_ms and not self.chien_expire and \
                (t - self.dernier_battement) * 1000 > self.chien_ms:
            self.chien_expire = True
            self.pwm = [0, 0]
            self.attente_inversion = [None, None]
            self._ecrire_lignes(["[CHIEN] expire : traction a zero"])
        for i in range(2):
            attente = self.attente_inversion[i]
            if attente is not None and (t - attente[0]) * 1000 >= self.inv_ms:
                self.sens[i] = 1 if attente[1] < 0 else 0
                self.pwm[i] = attente[1]
                self.attente_inversion[i] = None
        if self.servo_actif:
            pas = self.servo_vitesse * dt
            self.servo_deg += max(-pas, min(pas, self.servo_cible - self.servo_deg))
        self.lacet_deg += self.imu[1][2] * dt
        if self.tel_hz and t >= self._prochaine_tel:
            self._prochaine_tel = t + 1 / self.tel_hz
            chien = int(bool(self.chien_ms) and not self.chien_expire)
            self._ecrire_lignes([f"T {int(t * 1000)} {self.pwm[0]} {self.pwm[1]} "
                                 f"{self.servo_deg:.1f} {int(self.frein)} {int(self.defaut)} "
                                 f"{chien}"])
        elif not self.tel_hz and t >= self._prochain_etat:
            self._prochain_etat = t + 1.0
            self._etat()
        (ax, ay, az), (gx, gy, gz) = self.imu
        if self.imu_hz and t >= self._prochaine_imu:
            self._prochaine_imu = t + 1 / self.imu_hz
            self._ecrire_lignes([f"I {int(t * 1000)} {ax:.3f} {ay:.3f} {az:.3f} {gx:.2f} "
                                 f"{gy:.2f} {gz:.2f} {self.lacet_deg:.2f}"])
        elif not self.imu_hz and t >= self._prochain_texte:
            self._prochain_texte = t + 0.5
            self._ecrire_lignes([
                f"IMU OK      a=[{ax:+5.2f} {ay:+5.2f} {az:+5.2f}] g  w=[{gx:+7.1f} {gy:+7.1f} "
                f"{gz:+7.1f}] deg/s  31.0 C | BARO OK      1013.20 hPa  22.0 C | KNOB ABSENT pos "
                f"   +0        | Accueil"])

    def pwm_effectifs(self) -> tuple[int, int]:
        return self.pwm[0], self.pwm[1]


class MaterielSimule:
    """Interface `Materiel` sur un monde simulé (fieldmind.simulation.monde)."""

    nom = "simulation"

    def __init__(self, bus: Bus, profil: Profil, graine: int = 0, scenario: str = "salle",
                 version_esp: str = "ROVER_3", vitesse_rendu_ips: float = 6.0,
                 depart: Pose2D | None = None) -> None:
        from fieldmind.simulation.monde import Monde  # monde 3D simple (murs, boîtes…)

        self.bus, self.profil = bus, profil
        self.monde = Monde(graine=graine, scenario=scenario)
        self.physique = PhysiqueRover(profil, depart or self._depart())
        self.faux_esp = FauxEsp(version_esp)
        self._esp = PiloteEsp(bus, profil, self.faux_esp, drone_reel=False)
        self._actif = False
        self._lidar_marche = bool(profil.valeur("capteurs.lidar", True))
        self._micro = False
        self._inverse = False
        self.ips_rendu = vitesse_rendu_ips
        self._fils: list[threading.Thread] = []
        self._hasard = random.Random(graine)
        self._stats = {"lidar_hz": 0.0, "cam_ips": 0.0}
        self.vitesse_temps = 1.0

    def _depart(self) -> Pose2D:
        depart = getattr(self.monde, "depart", None)
        if isinstance(depart, Pose2D):
            return depart
        if isinstance(depart, (tuple, list)) and len(depart) >= 2:
            return Pose2D(float(depart[0]), float(depart[1]),
                          float(depart[2]) if len(depart) > 2 else 0.0)
        return Pose2D()

    # ------------------------------------------------------------------ interface Materiel

    def demarrer(self) -> None:
        self._actif = True
        self._esp.demarrer()
        for cible, nom in ((self._physique, "sim-physique"), (self._lidar, "sim-lidar"),
                           (self._cameras, "sim-cameras"), (self._gps, "sim-gps"),
                           (self._audio, "sim-micro")):
            fil = threading.Thread(target=cible, name=nom, daemon=True)
            fil.start()
            self._fils.append(fil)
        self.bus.journaliser("simulation", f"monde « {getattr(self.monde, 'scenario', '?')} » : "
                             "matériel simulé", "succes")

    def arreter(self) -> None:
        self._actif = False
        self._esp.arreter()

    def esp(self) -> PiloteEsp | None:
        return self._esp if self._esp.actif else None

    def capteurs(self) -> dict[str, Any]:
        age = self.bus.age
        return {
            "lidar": {"ok": self._lidar_marche and age(S.SCAN) < 0.5,
                      "hz": round(self._stats["lidar_hz"], 1),
                      "points": len(getattr(self.bus.dernier(S.SCAN), "angles", ())),
                      "age_s": round(min(age(S.SCAN), 99), 2), "marche": self._lidar_marche},
            "gauche": {"ok": age(S.IMAGE_GAUCHE) < 1.0, "ips": round(self._stats["cam_ips"], 1),
                       "age_s": round(min(age(S.IMAGE_GAUCHE), 99), 2), "resolution": [640, 360]},
            "droite": {"ok": age(S.IMAGE_DROITE) < 1.0, "ips": round(self._stats["cam_ips"], 1),
                       "age_s": round(min(age(S.IMAGE_DROITE), 99), 2), "resolution": [640, 360]},
            "gnss": {"ok": age(S.GNSS) < 3, "age_s": round(min(age(S.GNSS), 99), 1)},
            "esp": {"ok": self._esp.connecte(), "age_s": round(min(age(S.ESP), 99), 2),
                    "firmware": self._esp.version},
            "imu": {"ok": age(S.IMU) < 1.5, "age_s": round(min(age(S.IMU), 99), 2)},
            "micro": {"ok": True, "actif": self._micro, "nom": "micro simulé"},
        }

    def systeme(self) -> dict[str, Any]:
        return {"cpu_pct": None, "processus_cpu_pct": None, "temperature_c": None,
                "memoire_pct": None, "disque_pct": None, "charge": None, "uptime_s": None,
                "throttled": None, "sous_tension": False, "ralenti": False,
                "wifi": {"ssid": "simulation", "signal_pct": 100, "ip": "127.0.0.1"}}

    def inventaire(self) -> dict[str, Any]:
        return {"simulation": True, "monde": getattr(self.monde, "description", dict)()}

    def lidar_marche(self, marche: bool) -> bool:
        self._lidar_marche = marche
        return True

    def micro_actif(self, actif: bool) -> bool:
        self._micro = actif
        return True

    def cameras_inversees(self, inverse: bool) -> None:
        self._inverse = not self._inverse

    def verite(self) -> dict[str, float]:
        p = self.physique.pose
        return {"x": p.x, "y": p.y, "cap": p.cap, "vitesse": self.physique.vitesse}

    # ------------------------------------------------------------------ fils de simulation

    def _physique(self) -> None:
        dt = 0.01
        prochaine = time.monotonic()
        while self._actif:
            t = time.monotonic()
            g, d = self.faux_esp.pwm_effectifs()
            self.physique.avancer(dt * self.vitesse_temps, g, d, self.faux_esp.servo_deg,
                                  lambda pose: self.monde.collision(pose, self.profil))
            ph = self.physique
            bruit = self._hasard.gauss
            accel = (ph.accel_long / 9.81 + bruit(0, 0.01),
                     ph.vitesse * ph.vitesse_lacet / 9.81 + bruit(0, 0.01), 1.0 + bruit(0, 0.01))
            gyro = (bruit(0, 0.2), bruit(0, 0.2), math.degrees(ph.vitesse_lacet) + bruit(0, 0.3))
            self.faux_esp.imu = (accel, gyro)
            self.faux_esp.tick(t, dt)
            prochaine += dt
            attente = prochaine - time.monotonic()
            if attente > 0:
                time.sleep(attente)
            else:
                prochaine = time.monotonic()

    def _lidar(self) -> None:
        periode = 0.1
        precedent = time.monotonic()
        pose_debut = self.physique.pose
        while self._actif:
            debut = time.monotonic()
            if self._lidar_marche:
                pose = self.physique.pose
                angles, distances = self.monde.lancer_lidar(pose, self.profil, pose_debut)
                pose_debut = pose
                garde = distances > float(self.profil.valeur("lidar.portee_min_m", 0.12))
                maintenant = time.monotonic()
                self.bus.publier(S.SCAN, Scan(t=maintenant,
                                              angles=np.asarray(angles, np.float32)[garde],
                                              distances=np.asarray(distances, np.float32)[garde],
                                              duree=periode))
                self._stats["lidar_hz"] = 1 / max(maintenant - precedent, 1e-3)
                precedent = maintenant
            time.sleep(max(0.0, periode - (time.monotonic() - debut)))

    def _cameras(self) -> None:
        numero = 0
        while self._actif:
            debut = time.monotonic()
            pose = self.physique.pose
            for role in ("gauche", "droite"):
                if not bool(self.profil.valeur(f"capteurs.camera_{role}", True)):
                    continue
                camera = role if not self._inverse else ("droite" if role == "gauche" else "gauche")
                image = self.monde.rendre_camera(pose, camera, 640, 360, self.profil)
                ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    numero += 1
                    self.bus.publier(S.IMAGE_GAUCHE if role == "gauche" else S.IMAGE_DROITE,
                                     ImageCamera(t=debut, camera=role, jpeg=jpeg.tobytes(),
                                                 largeur=640, hauteur=360, numero=numero))
            duree = time.monotonic() - debut
            self._stats["cam_ips"] = 1 / max(duree, 1 / self.ips_rendu)
            time.sleep(max(0.0, 1 / self.ips_rendu - duree))

    def _gps(self) -> None:
        cap_lat = METRES_PAR_DEGRE_LAT
        cap_lon = METRES_PAR_DEGRE_LAT * math.cos(math.radians(REFERENCE_GPS[0]))
        derive = [0.0, 0.0]
        while self._actif:
            if not bool(self.profil.valeur("capteurs.gnss", True)):
                time.sleep(1.0)
                continue
            p = self.physique.pose
            derive = [0.95 * derive[0] + self._hasard.gauss(0, 0.4),
                      0.95 * derive[1] + self._hasard.gauss(0, 0.4)]
            self.bus.publier(S.GNSS, Gnss(
                t=time.monotonic(), fix="autonome",
                latitude=REFERENCE_GPS[0] + (p.y + derive[1]) / cap_lat,
                longitude=REFERENCE_GPS[1] + (p.x + derive[0]) / cap_lon,
                altitude_m=12.0, hdop=1.2, satellites=7, vitesse_m_s=abs(self.physique.vitesse),
                cap_deg=(90 - math.degrees(p.cap)) % 360))
            time.sleep(1.0)

    def _audio(self) -> None:
        bloc = 320
        while self._actif:
            if self._micro:
                bruit = (np.random.default_rng().normal(0, 40, bloc)).astype(np.int16)
                self.bus.publier(S.AUDIO, EchantillonAudio(t=time.monotonic(),
                                                           pcm=bruit.tobytes()))
            time.sleep(0.02)
