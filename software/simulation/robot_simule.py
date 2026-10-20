"""Rover simulé : physique MuJoCo, capteurs bruités et firmware du MCU.

Vu du cerveau, ce rover se comporte comme le rover réel : il publie des messages de capteurs
horodatés (`services.protocole.messages`) et reçoit des consignes Ackermann. La vérité terrain
(`VeriteTerrain`) ne sert qu'à l'affichage et à l'évaluation ; elle ne passe jamais par le cerveau.
"""

from __future__ import annotations

import math
import threading
from collections import deque
from dataclasses import dataclass

import mujoco
import numpy as np

from services.protocole.fiche_rover import FicheRover
from services.protocole.messages import (
    Commande,
    Firmware,
    Gnss,
    Image,
    ImageStereo,
    Imu,
    Odometrie,
    Scan,
    TypeFix,
)
from simulation.camera_simulee import TAILLE_TEXTURE, emprise_sol, texture_sol
from simulation.firmware import FREQUENCE_HZ, FirmwareSimule
from simulation.modele_mujoco import (
    GROUPE_MONDE,
    GROUPE_ROVER,
    ROUES,
    TERRAIN,
    TEXTURE_SOL,
    construire_mjcf,
)
from simulation.monde import Monde

MessageCapteur = Scan | Imu | Odometrie | Gnss | Firmware | Image | ImageStereo


@dataclass(frozen=True, slots=True)
class VeriteTerrain:
    """État exact du rover dans la simulation (affichage et évaluation uniquement)."""

    t: float
    x: float
    y: float
    z: float
    roulis: float
    tangage: float
    cap: float
    vitesse: float
    braquage_gauche: float
    braquage_droit: float
    angles_roues: tuple[float, float, float, float]
    collisions: int
    en_contact: bool


def _lacet_roulis_tangage(q: np.ndarray) -> tuple[float, float, float]:
    """Quaternion (w, x, y, z) → (cap, roulis, tangage), convention ZYX."""
    w, x, y, z = (float(v) for v in q)
    cap = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    roulis = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    tangage = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return cap, roulis, tangage


_TEXTURES: dict[tuple[int, str], np.ndarray] = {}
_VERROU_TEXTURES = threading.Lock()


def _texture_sol(monde: Monde) -> np.ndarray:
    cle = (monde.graine, monde.scenario.nom)
    with _VERROU_TEXTURES:
        if cle not in _TEXTURES:
            if len(_TEXTURES) >= 4:
                _TEXTURES.clear()
            _TEXTURES[cle] = texture_sol(monde)
        return _TEXTURES[cle]


@dataclass(frozen=True, slots=True)
class Capture:
    """Prise de vue de la caméra : instant et état articulaire, à rendre hors de la boucle."""

    t: float
    qpos: np.ndarray


class RobotSimule:
    """Un rover dans sa forêt. `avancer()` fait avancer le temps et renvoie les messages publiés.

    Caméra : à sa cadence, le rover mémorise une `Capture` que la couche d'exécution rend en
    image (`CameraSimulee`) hors de la boucle de la physique. `camera=False` la supprime.
    """

    def __init__(
        self, fiche: FicheRover, monde: Monde, graine_bruit: int = 0, camera: bool = True
    ) -> None:
        self.fiche = fiche
        self.monde = monde
        self.avec_camera = camera and fiche.capteurs.camera is not None
        if self.avec_camera:
            self.modele = mujoco.MjModel.from_xml_string(
                construire_mjcf(fiche, monde, sol=emprise_sol(monde), taille_texture=TAILLE_TEXTURE)
            )
            pixels = _texture_sol(monde)
            t = self.modele.texture(TEXTURE_SOL)
            debut = int(self.modele.tex_adr[t.id])
            self.modele.tex_data[debut : debut + pixels.size] = pixels.reshape(-1)
        else:
            self.modele = mujoco.MjModel.from_xml_string(construire_mjcf(fiche, monde))
        if monde.relief is not None:
            h = monde.relief.hauteurs
            etendue = max(float(h.max() - h.min()), 0.01)
            champ = self.modele.hfield(TERRAIN)
            debut = int(self.modele.hfield_adr[champ.id])
            self.modele.hfield_data[debut : debut + h.size] = ((h - h.min()) / etendue).ravel()
        self.donnees = mujoco.MjData(self.modele)
        self.firmware = FirmwareSimule(fiche)
        self._rng = np.random.default_rng(graine_bruit)
        self._nstep = round(1.0 / FREQUENCE_HZ / self.modele.opt.timestep)
        m = self.modele

        self._id_chassis = m.body("chassis").id
        self._id_moteur = m.actuator("moteur").id
        self._id_servo = (m.actuator("servo_g").id, m.actuator("servo_d").id)
        self._site_lidar = m.site("lidar").id
        self._site_gnss = m.site("gnss").id
        self._adr_capteur = {
            nom: (m.sensor(nom).adr[0], m.sensor(nom).dim[0])
            for nom in (
                "gyro",
                "accel",
                "orientation",
                "pos_braquage_g",
                "pos_braquage_d",
                *(f"vel_{r}" for r in ROUES),
            )
        }
        self._qpos_roues = [m.jnt_qposadr[m.joint(r).id] for r in ROUES]
        self._geoms_rover = {i for i in range(m.ngeom) if m.geom_group[i] == GROUPE_ROVER}
        self._geom_sol = m.geom("sol").id

        # Lidar : directions des faisceaux dans le repère du capteur.
        lidar = fiche.capteurs.lidar
        self._angle_min = -math.pi
        self._pas_lidar = 2 * math.pi / lidar.mesures_par_tour
        angles = self._angle_min + self._pas_lidar * np.arange(lidar.mesures_par_tour)
        self._faisceaux = np.column_stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)])
        self._geomgroup = np.zeros(6, dtype=np.uint8)
        self._geomgroup[GROUPE_MONDE] = 1
        self._arbres_xy = np.array([[a.x, a.y] for a in monde.arbres + monde.lisiere]).reshape(
            -1, 2
        )

        # Cadencement des capteurs (période, prochaine échéance).
        c = fiche.capteurs
        self._periodes = {
            "imu": 1 / c.imu.frequence_hz,
            "odometrie": 1 / c.codeurs.frequence_hz,
            "gnss": 1 / c.gnss.frequence_hz,
            "scan": 1 / lidar.frequence_hz,
            "firmware": 0.1,
        }
        if self.avec_camera and c.camera is not None:
            self._periodes["camera"] = 1 / c.camera.frequence_hz
        self._capture: Capture | None = None
        self._verites_capture: deque[VeriteTerrain] = deque(maxlen=64)
        self._echeances = dict.fromkeys(self._periodes, 0.0)
        self._gnss_en_vol: deque[tuple[float, Gnss]] = deque()
        self._ticks_precedents = np.zeros(4)
        self._t_codeurs_precedent = 0.0
        self._derive_cap = 0.0
        self._collisions = 0
        self._en_contact = False
        self._sortie: list[MessageCapteur] = []
        self._t_cible = 0.0
        mujoco.mj_forward(m, self.donnees)
        depart = monde.scenario.depart
        self._z_origine = monde.hauteur_sol(depart.x, depart.y)
        """Altitude du sol au départ (origine locale des altitudes GNSS)."""
        self._ticks_precedents = self._ticks()

    # ------------------------------------------------------------------ interface
    @property
    def t(self) -> float:
        return float(self.donnees.time)

    def recevoir(self, commande: Commande) -> None:
        """Consigne du cerveau, reçue à l'instant courant de la simulation."""
        self.firmware.recevoir_consigne(self.t, commande.vitesse, commande.braquage)

    def avancer(self, duree: float) -> list[MessageCapteur]:
        """Fait avancer la simulation d'au moins `duree` secondes ; renvoie les messages publiés."""
        # Horloge cible cumulée : pas de dérive quand `duree` n'est pas un multiple du cycle.
        self._t_cible += duree
        demi_cycle = 0.5 * self._nstep * self.modele.opt.timestep
        while self.t < self._t_cible - demi_cycle:
            self._cycle()
        sortie, self._sortie = self._sortie, []
        return sortie

    # ------------------------------------------------------------------ boucle
    def _cycle(self) -> None:
        m, d = self.modele, self.donnees
        omega_ar = 0.5 * (self._capteur("vel_roue_ar_g")[0] + self._capteur("vel_roue_ar_d")[0])
        braquage_mesure = self._braquage_equivalent()
        sorties = self.firmware.cycle(self.t, omega_ar, braquage_mesure)
        d.ctrl[self._id_moteur] = sorties.couple_moteur
        d.ctrl[self._id_servo[0]] = sorties.braquage_gauche
        d.ctrl[self._id_servo[1]] = sorties.braquage_droit
        mujoco.mj_step(m, d, nstep=self._nstep)
        self._detecter_collisions()
        self._publier()

    def _publier(self) -> None:
        t = self.t
        for nom, periode in self._periodes.items():
            if t + 1e-9 >= self._echeances[nom]:
                self._echeances[nom] += periode
                if self._echeances[nom] <= t:
                    self._echeances[nom] = t + periode
                message = getattr(self, f"_mesurer_{nom}")()
                if message is not None:
                    self._sortie.append(message)
        while self._gnss_en_vol and self._gnss_en_vol[0][0] <= t:
            self._sortie.append(self._gnss_en_vol.popleft()[1])

    # ------------------------------------------------------------------ capteurs
    def _capteur(self, nom: str) -> np.ndarray:
        adr, dim = self._adr_capteur[nom]
        valeurs: np.ndarray = self.donnees.sensordata[adr : adr + dim]
        return valeurs

    def _braquage_equivalent(self) -> float:
        """Angle « bicyclette » équivalent aux deux angles de fusée mesurés."""
        g = self.fiche.geometrie
        tg = math.tan(float(self._capteur("pos_braquage_g")[0]))
        td = math.tan(float(self._capteur("pos_braquage_d")[0]))
        courbure = 0.0
        if abs(tg) > 1e-6 and abs(td) > 1e-6:
            courbure = 0.5 * (
                1 / (g.empattement / tg + g.voie_avant / 2)
                + 1 / (g.empattement / td - g.voie_avant / 2)
            )
        return math.atan(g.empattement * courbure) if courbure else 0.0

    def _mesurer_scan(self) -> Scan:
        lidar = self.fiche.capteurs.lidar
        d = self.donnees
        origine = d.site_xpos[self._site_lidar]
        rotation = d.site_xmat[self._site_lidar].reshape(3, 3)
        directions = (self._faisceaux @ rotation.T).ravel()
        n = lidar.mesures_par_tour
        distances = np.zeros(n)
        geoms = np.zeros(n, dtype=np.int32)
        mujoco.mj_multiRay(
            self.modele,
            d,
            origine,
            directions,
            self._geomgroup,
            1,
            -1,
            geoms,
            distances,
            None,
            n,
            lidar.portee_max,
        )
        if self.monde.relief is not None:
            sol = self._impacts_terrain(origine, directions.reshape(n, 3), lidar.portee_max)
            touche = sol < np.where(geoms >= 0, distances, np.inf)
            distances = np.where(touche, sol, distances)
            geoms = np.where(touche, self._geom_sol, geoms)
        bruit = self._rng.normal(0.0, lidar.bruit_sigma, n)
        perdues = self._rng.random(n) < lidar.taux_pertes
        valides = (
            (geoms >= 0)
            & (distances >= lidar.portee_min)
            & (distances <= lidar.portee_max)
            & ~perdues
        )
        mesures = np.round(distances + bruit, 3)
        liste: list[float | None] = [
            float(v) if ok else None for v, ok in zip(mesures, valides, strict=True)
        ]
        _, roulis, tangage = _lacet_roulis_tangage(self._capteur("orientation"))
        self.firmware.recevoir_scan(self._angle_min, self._pas_lidar, liste, roulis, tangage)
        return Scan(
            t=self.t,
            angle_min=self._angle_min,
            pas=self._pas_lidar,
            portee_min=lidar.portee_min,
            portee_max=lidar.portee_max,
            distances=liste,
        )

    def _impacts_terrain(
        self, origine: np.ndarray, directions: np.ndarray, portee: float
    ) -> np.ndarray:
        """Distance de chaque faisceau au terrain (`inf` s'il ne le touche pas) : marche par pas
        de 4 cm puis interpolation linéaire entre le dernier point au-dessus et le premier
        dessous."""
        assert self.monde.relief is not None
        s = np.arange(0.04, portee + 0.04, 0.04)
        points = origine[None, None, :] + directions[:, None, :] * s[None, :, None]
        ecart = points[..., 2] - self.monde.relief.hauteur(points[..., 0], points[..., 1])
        dessous = ecart <= 0
        touche = dessous.any(axis=1)
        resultat = np.full(len(directions), np.inf)
        if touche.any():
            i = np.argmax(dessous[touche], axis=1)
            e1 = ecart[touche, i]
            e0 = np.where(i > 0, ecart[touche, np.maximum(i - 1, 0)], e1)
            s1, s0 = s[i], np.where(i > 0, s[np.maximum(i - 1, 0)], s[i])
            f = np.where(e0 > e1, e0 / np.maximum(e0 - e1, 1e-9), 1.0)
            resultat[touche] = s0 + f * (s1 - s0)
        return resultat

    def _mesurer_imu(self) -> Imu:
        imu = self.fiche.capteurs.imu
        rng = self._rng
        gyro = self._capteur("gyro") + rng.normal(0, imu.bruit_gyro, 3)
        gyro[2] += imu.biais_gyro_z
        accel = self._capteur("accel") + rng.normal(0, imu.bruit_accel, 3)
        # Cap absolu (magnétomètre) : erreur lente, type Ornstein-Uhlenbeck de σ donné.
        sigma = math.radians(imu.bruit_cap_absolu_deg)
        dt = self._periodes["imu"]
        tau = 20.0
        self._derive_cap += (
            -self._derive_cap * dt / tau + sigma * math.sqrt(2 * dt / tau) * rng.normal()
        )
        w, x, y, z = (float(v) for v in self._capteur("orientation"))
        c, s = math.cos(self._derive_cap / 2), math.sin(self._derive_cap / 2)
        # Rotation supplémentaire autour de la verticale du monde : q' = qz ⊗ q.
        q = (c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w)
        return Imu(
            t=self.t,
            orientation=q,
            vitesse_angulaire=(float(gyro[0]), float(gyro[1]), float(gyro[2])),
            acceleration=(float(accel[0]), float(accel[1]), float(accel[2])),
        )

    def _ticks(self) -> np.ndarray:
        n = self.fiche.capteurs.codeurs.impulsions_par_tour
        angles = self.donnees.qpos[self._qpos_roues]
        ticks: np.ndarray = np.floor(angles * n / (2 * math.pi))
        return ticks

    def _mesurer_odometrie(self) -> Odometrie:
        n = self.fiche.capteurs.codeurs.impulsions_par_tour
        ticks = self._ticks()
        dt = max(self.t - self._t_codeurs_precedent, 1e-6)
        omega = (ticks - self._ticks_precedents) * 2 * math.pi / n / dt
        self._ticks_precedents, self._t_codeurs_precedent = ticks, self.t
        return Odometrie(
            t=self.t,
            roue_av_g=float(omega[0]),
            roue_av_d=float(omega[1]),
            roue_ar_g=float(omega[2]),
            roue_ar_d=float(omega[3]),
            braquage=round(self._braquage_equivalent(), 4),
        )

    def _mesurer_gnss(self) -> None:
        gnss = self.fiche.capteurs.gnss
        antenne = self.donnees.site_xpos[self._site_gnss]
        # Qualité selon le couvert : les troncs proches masquent une partie du ciel.
        proches = 0
        if self._arbres_xy.size:
            proches = int(np.count_nonzero(np.hypot(*(self._arbres_xy - antenne[:2]).T) < 5.0))
        fix: TypeFix
        if proches >= 12:
            fix, sigma, satellites = "dgnss", 0.5, 11
        elif proches >= 7:
            fix, sigma, satellites = "rtk_flottant", 0.12, 16
        else:
            fix, sigma, satellites = "rtk_fixe", gnss.sigma_rtk_fixe, 22
        bruit = self._rng.normal(0, sigma, 3)
        # Le vertical est environ 1,5 fois moins précis que l'horizontal (géométrie des satellites).
        sigma_z = 1.5 * sigma
        message = Gnss(
            t=self.t,
            fix=fix,
            x=round(float(antenne[0] + bruit[0]), 4),
            y=round(float(antenne[1] + bruit[1]), 4),
            sigma=sigma,
            satellites=satellites,
            z=round(float(antenne[2] - self._z_origine + bruit[2] * 1.5), 4),
            sigma_z=sigma_z,
        )
        self._gnss_en_vol.append((self.t + gnss.latence_s, message))

    def _mesurer_camera(self) -> None:
        self._capture = Capture(self.t, self.donnees.qpos.copy())
        self._verites_capture.append(self.verite())

    def prendre_capture(self) -> Capture | None:
        """Dernière prise de vue non encore rendue (les plus anciennes sont abandonnées)."""
        capture, self._capture = self._capture, None
        return capture

    def verite_a(self, t: float) -> VeriteTerrain | None:
        """Vérité terrain à l'instant d'une prise de vue (détecteur oracle, évaluation)."""
        for verite in reversed(self._verites_capture):
            if abs(verite.t - t) < 1e-6:
                return verite
        return None

    def _mesurer_firmware(self) -> Firmware:
        fw = self.firmware
        return Firmware(
            t=self.t,
            arret=fw.arret,
            vitesse_consigne=round(fw.vitesse_consigne, 4),
            braquage_consigne=round(fw.braquage_consigne, 4),
            couple_moteur=round(fw.sorties.couple_moteur, 4),
            regime_moteur=round(fw.regime_moteur, 1),
            tension_batterie=round(fw.batterie.tension, 3),
            courant_batterie=round(fw.batterie.courant, 3),
            etat_charge=round(fw.batterie.etat_charge, 5),
        )

    # ------------------------------------------------------------------ vérité terrain
    def _detecter_collisions(self) -> None:
        d = self.donnees
        contact = False
        for i in range(d.ncon):
            g1, g2 = int(d.contact.geom1[i]), int(d.contact.geom2[i])
            if self._geom_sol in (g1, g2):
                continue
            if (g1 in self._geoms_rover) != (g2 in self._geoms_rover):
                contact = True
                break
        if contact and not self._en_contact:
            self._collisions += 1
        self._en_contact = contact

    def verite(self) -> VeriteTerrain:
        d = self.donnees
        pos = d.xpos[self._id_chassis]
        cap, roulis, tangage = _lacet_roulis_tangage(d.xquat[self._id_chassis])
        vitesse_monde = d.qvel[:3]  # vitesse linéaire de la liaison libre, repère du monde
        vitesse = float(vitesse_monde[0] * math.cos(cap) + vitesse_monde[1] * math.sin(cap))
        angles = tuple(float(d.qpos[a]) for a in self._qpos_roues)
        return VeriteTerrain(
            t=self.t,
            x=float(pos[0]),
            y=float(pos[1]),
            z=float(pos[2]),
            roulis=roulis,
            tangage=tangage,
            cap=cap,
            vitesse=vitesse,
            braquage_gauche=float(self._capteur("pos_braquage_g")[0]),
            braquage_droit=float(self._capteur("pos_braquage_d")[0]),
            angles_roues=(angles[0], angles[1], angles[2], angles[3]),
            collisions=self._collisions,
            en_contact=self._en_contact,
        )
