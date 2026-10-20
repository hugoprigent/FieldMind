"""Banc de dynamique simplifiée : vitesses avec inertie, dérive, capteurs et images.

Les mesures hauteur/cap sont des capteurs simulés, pas une capacité du CooleerX.
La cible est obtenue à partir des pixels rendus ; sa position réelle n'est jamais
injectée dans l'autopilote. Aucune dépendance à un port BLE, série ou socket réseau.
"""

from __future__ import annotations

import math
from dataclasses import asdict

import cv2
import numpy as np

from .autopilote import Autopilote
from .modele import Observation, Ordre, angle
from .vision import CalibrationCroix, VisionCroix

SCENARIOS = (
    "normal",
    "derive",
    "croix_perdue",
    "video_perdue",
    "camera_bloquee",
    "cap_perdu",
    "gx10_retard",
)


class DroneSimule:
    def __init__(self, graine: int = 0, scenario: str = "normal") -> None:
        if scenario not in SCENARIOS:
            raise ValueError("scénario drone inconnu")
        rng = np.random.default_rng(graine)
        self.scenario = scenario
        self.t = 0.0
        self.x, self.y = rng.uniform(-0.09, 0.09, 2)
        self.h = 0.0
        self.cap = 0.0
        self.vz = self.vl = self.va = self.vd = 0.0
        self.camera = "avant"
        self.au_sol = True
        self._atterrir = False
        self._camera_demandee: float | None = None
        self.sequence = 0
        self.image_sequence = 0
        self.image_t = 0.0
        self.image_brute = self._rendre()
        self.vision = VisionCroix()
        self.cible = None
        self._prochaine_image = 0.0
        self.actions: list[tuple[float, str]] = []
        self.vent = (0.008, -0.006) if scenario == "derive" else (0.001, -0.001)
        self._bottom_t: float | None = None
        self._derniere_obs: Observation | None = None

    def avancer(self, ordre: Ordre, dt: float = 0.04) -> None:
        if not 0 < dt <= 0.1:
            raise ValueError("pas de simulation hors limites")
        self.t += dt
        self.sequence += 1
        if ordre.action:
            self.actions.append((self.t, ordre.action))
        if ordre.action == "decoller" and self.au_sol:
            self.au_sol = False
            self._atterrir = False
        elif ordre.action == "atterrir" and not self.au_sol:
            self._atterrir = True
        elif ordre.action == "stop":
            self.au_sol, self.h = True, 0.0
        elif ordre.action == "camera_bas":
            self._camera_demandee = self.t
        if (
            self._camera_demandee is not None
            and self.t - self._camera_demandee > 0.3
            and self.scenario != "camera_bloquee"
        ):
            self.camera = "bas"
            if self._bottom_t is None:
                self._bottom_t = self.t
        if not self.au_sol:
            tau = min(1.0, dt / 0.16)
            self.vz += tau * ((-0.16 if self._atterrir else ordre.montee_m_s) - self.vz)
            self.vl += tau * (ordre.lacet_rad_s - self.vl)
            self.va += tau * (ordre.avant_m_s - self.va)
            self.vd += tau * (ordre.droite_m_s - self.vd)
            self.cap += self.vl * dt
            self.x += (
                self.va * math.cos(self.cap) - self.vd * math.sin(self.cap) + self.vent[0]
            ) * dt
            self.y += (
                self.va * math.sin(self.cap) + self.vd * math.cos(self.cap) + self.vent[1]
            ) * dt
            self.h = max(0.0, self.h + self.vz * dt)
            if self._atterrir and self.h <= 0.005:
                self.h, self.vz, self.vl, self.va, self.vd = 0.0, 0.0, 0.0, 0.0, 0.0
                self.au_sol = True
        if self.t >= self._prochaine_image:
            self._prochaine_image = self.t + 0.2
            if self.scenario != "video_perdue" or self.t < 15:
                self.image_t = self.t
                self.image_sequence += 1
                self.image_brute = self._rendre()
                self.cible = (
                    self.vision.estimer(self.image_brute, self.t, self.image_sequence)
                    if self.camera == "bas"
                    else None
                )
                if self.scenario == "gx10_retard" and self.cible is not None:
                    # Résultat parfait mais trop vieux : doit être refusé comme sur réseau lent.
                    self.cible = type(self.cible)(**{**asdict(self.cible), "t": self.t - 1.0})

    def observer(self) -> Observation:
        if self.scenario == "cap_perdu" and self.t > 15 and self._derniere_obs:
            return self._derniere_obs
        o = Observation(
            t=self.t,
            sequence=self.sequence,
            hauteur_m=self.h,
            cap_rad=angle(self.cap),
            vitesse_z=self.vz,
            vitesse_lacet=self.vl,
            camera=self.camera,
            au_sol=self.au_sol,
            cible=self.cible,
            image_t=self.image_t,
            batterie_pct=90,
            source="capteurs_simules",
        )
        self._derniere_obs = o
        return o

    def _rendre(self) -> np.ndarray:
        img = np.full((240, 320, 3), (226, 231, 234), np.uint8)
        if self.camera != "bas":
            # Repères angulaires pour différencier les six photos ; pas d'oracle de position.
            for azimut in range(-360, 721, 60):
                erreur = angle(math.radians(azimut) - self.cap)
                if abs(erreur) < 0.7:
                    px = round(160 + erreur * 180)
                    cv2.rectangle(img, (px - 18, 60), (px + 18, 185), (70, 130, 80), -1)
                    cv2.putText(
                        img,
                        f"{azimut % 360}",
                        (px - 18, 55),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (30, 50, 30),
                        1,
                    )
            return img
        if (
            self.scenario == "croix_perdue"
            and self._bottom_t is not None
            and self.t - self._bottom_t > 5
        ):
            return img
        c = CalibrationCroix()
        longueur, e = c.bras_m / 2, c.epaisseur_m / 2
        contour = np.array(
            [
                [-e, -longueur],
                [e, -longueur],
                [e, -e],
                [longueur, -e],
                [longueur, e],
                [e, e],
                [e, longueur],
                [-e, longueur],
                [-e, e],
                [-longueur, e],
                [-longueur, -e],
                [-e, -e],
            ],
            dtype=np.float64,
        )
        pixels = []
        for px, py in contour:
            dx, dy = px - self.x, py - self.y
            avant = dx * math.cos(self.cap) + dy * math.sin(self.cap)
            droite = -dx * math.sin(self.cap) + dy * math.cos(self.cap)
            pixels.append(
                [c.cx + c.fx * droite / max(0.12, self.h), c.cy - c.fy * avant / max(0.12, self.h)]
            )
        cv2.fillPoly(img, [np.round(pixels).astype(np.int32)], (5, 5, 5))
        return img

    def image_annotee(self, pilote: Autopilote) -> np.ndarray:
        img = self.image_brute.copy()
        cv2.rectangle(img, (0, 0), (320, 23), (30, 35, 45), -1)
        cv2.putText(
            img,
            f"SIMULATION | {pilote.phase} | {self.camera}",
            (4, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (100, 230, 250),
            1,
        )
        return img


def executer(graine: int = 0, scenario: str = "normal") -> tuple[dict, list[np.ndarray]]:
    drone, pilote = DroneSimule(graine, scenario), Autopilote()
    ordre = pilote.demarrer(drone.observer(), drone.t)
    photos = []
    for _ in range(5000):
        drone.avancer(ordre)
        o = drone.observer()
        ordre = pilote.pas(o, drone.t)
        if ordre.photo:
            photos.append(drone.image_brute.copy())
        if pilote.phase in ("termine", "interrompu", "erreur"):
            break
    return {
        **pilote.etat(),
        "scenario": scenario,
        "graine": graine,
        "duree_s": round(drone.t, 2),
        "distance_pose_m": round(math.hypot(drone.x, drone.y), 4),
        "au_sol": drone.au_sol,
        "source": "simulation",
        "actions": drone.actions,
        "journal": pilote.journal,
    }, photos
