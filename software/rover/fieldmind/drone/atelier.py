"""Atelier du poste : simulation isolée, sans référence au pilote ESP réel."""

from __future__ import annotations

import json
import math
import threading
import time
import uuid
from dataclasses import asdict
from pathlib import Path

import cv2

from fieldmind.bus import Bus, S
from fieldmind.types import ImageCamera

from .autopilote import Autopilote
from .modele import Ordre, borner
from .simulation import SCENARIOS, DroneSimule


class AtelierDrone:
    def __init__(self, bus: Bus, dossier: Path | None = None) -> None:
        self.bus = bus
        self.dossier = dossier or Path("~/.local/share/fieldmind/drone-simulations").expanduser()
        self._verrou = threading.RLock()
        self._arret = threading.Event()
        self._fil: threading.Thread | None = None
        self.drone: DroneSimule | None = None
        self.pilote = Autopilote()
        self.mode = "inactif"
        self._ordre = Ordre()
        self._manuel = Ordre()
        self._action: str | None = None
        self._dernier_poste = 0.0
        self._hauteur_manuelle = 0.0
        self._vitesse = 1
        self._session: Path | None = None
        self._photos: list = []
        self._image_seq = -1
        self._termine_archive = False
        self._pose_manuelle = False
        self.etat: dict = {"source": "simulation", "mode": "inactif", "phase": "inactif"}

    def demarrer(
        self, mode: str, scenario: str = "normal", graine: int = 0, vitesse: int = 1
    ) -> None:
        if mode not in ("mission", "manuel") or scenario not in SCENARIOS:
            raise ValueError("mode/scénario du banc inconnu")
        if vitesse not in (1, 4) or not 0 <= graine <= 10000:
            raise ValueError("vitesse/graine du banc invalide")
        if self.drone is not None and not self.drone.au_sol:
            raise ValueError("terminer ou interrompre le vol simulé courant avant de recommencer")
        self.arreter()
        with self._verrou:
            self.drone = DroneSimule(graine, scenario)
            self.pilote = Autopilote()
            self.mode = mode
            self._vitesse = vitesse
            self._manuel = Ordre()
            self._action = None
            self._dernier_poste = time.monotonic()
            self._hauteur_manuelle = 0.0
            self._session = None
            self._photos = []
            self._image_seq = -1
            self._termine_archive = self._pose_manuelle = False
            self._ordre = (
                self.pilote.demarrer(self.drone.observer(), self.drone.t)
                if mode == "mission"
                else Ordre()
            )
            self._arret = threading.Event()
            self._fil = threading.Thread(target=self._boucle, name="drone-simule", daemon=True)
            self._publier()
            self._fil.start()

    def commander(self, action: str, message: dict) -> None:
        if action in ("mission", "manuel"):
            self.demarrer(
                action,
                str(message.get("scenario", "normal")),
                int(message.get("graine", 0)),
                int(message.get("vitesse", 1)),
            )
            return
        with self._verrou:
            if self.drone is None or self._arret.is_set():
                raise ValueError("démarrer l'atelier drone simulé")
            if action == "interrompre":
                if self.mode == "mission":
                    self.pilote.annuler()
                else:
                    self._action = "atterrir"
                    self._pose_manuelle = True
                return
            if self.mode != "manuel":
                raise ValueError("commande manuelle refusée pendant la mission simulée")
            if action == "axes":
                valeurs = [float(message.get(k, 0)) for k in ("montee", "lacet", "avant", "droite")]
                if not all(math.isfinite(v) and abs(v) <= 1 for v in valeurs):
                    raise ValueError("axes du banc invalides")
                self._dernier_poste = time.monotonic()
                self._manuel = Ordre(
                    valeurs[0] * 0.18, valeurs[1] * 0.5, valeurs[2] * 0.1, valeurs[3] * 0.1
                )
            elif action in ("decoller", "atterrir", "camera_bas"):
                if self._action is not None:
                    raise ValueError("action simulée déjà en attente")
                if action == "decoller":
                    if not self.drone.au_sol or self._pose_manuelle:
                        raise ValueError("décollage simulé refusé dans cet état")
                    self._hauteur_manuelle = 0.7
                if action == "atterrir":
                    self._pose_manuelle = True
                self._action = action
                self._dernier_poste = time.monotonic()
            else:
                raise ValueError("action du banc inconnue")

    def _boucle(self) -> None:
        try:
            while not self._arret.wait(0.04):
                with self._verrou:
                    for _ in range(self._vitesse):
                        self._pas()
                        if self._arret.is_set():
                            break
        except Exception as e:
            self.etat = {**self.etat, "phase": "erreur", "raison": str(e)}
            self.bus.publier(S.DRONE_ATELIER, self.etat)
            self.bus.journaliser("drone_simule", f"atelier interrompu : {e}", "erreur")
            self._arret.set()

    def _pas(self) -> None:
        d = self.drone
        assert d is not None
        d.avancer(self._ordre)
        if self.mode == "mission":
            self._ordre = self.pilote.pas(d.observer(), d.t)
            if self._ordre.photo:
                self._photos.append(d.image_brute.copy())
            fini = self.pilote.phase in ("termine", "interrompu", "erreur")
        else:
            action = self._action
            self._action = None
            if (
                not d.au_sol
                and time.monotonic() - self._dernier_poste > 0.5
                and not self._pose_manuelle
            ):
                action = "atterrir"
                self._pose_manuelle = True
            m = self._manuel if not self._pose_manuelle else Ordre()
            self._hauteur_manuelle = max(
                0.15, min(1.5, self._hauteur_manuelle + m.montee_m_s * 0.04)
            )
            vz = borner(1.4 * (self._hauteur_manuelle - d.h) - 0.4 * d.vz, 0.22)
            self._ordre = Ordre(
                montee_m_s=vz if not d.au_sol or action == "decoller" else 0,
                lacet_rad_s=m.lacet_rad_s,
                avant_m_s=m.avant_m_s,
                droite_m_s=m.droite_m_s,
                action=action,
            )
            fini = self._pose_manuelle and d.au_sol
        if fini and not self._termine_archive:
            self._archiver()
            self._termine_archive = True
            self._publier()
            self._arret.set()
        elif self._image_seq != d.image_sequence:
            self._image_seq = d.image_sequence
            self._publier()

    def _publier(self) -> None:
        d = self.drone
        if d is None:
            return
        phase = (
            self.pilote.phase
            if self.mode == "mission"
            else (
                "termine"
                if d.au_sol and self._pose_manuelle
                else "atterrissage"
                if self._pose_manuelle
                else "manuel"
            )
        )
        self.etat = {
            **self.pilote.etat(),
            "phase": phase,
            "mode": self.mode,
            "source": "simulation",
            "scenario": d.scenario,
            "t_simule": round(d.t, 2),
            "hauteur_m": round(d.h, 3),
            "cap_deg": round(math.degrees(d.cap), 1),
            "camera": d.camera,
            "au_sol": d.au_sol,
            "cible": asdict(d.cible) if d.cible else None,
            "ordre": asdict(self._ordre),
            "journal": self.pilote.journal[-30:],
            "archive": self._session is not None,
        }
        self.bus.publier(S.DRONE_ATELIER, self.etat)
        im = d.image_annotee(self.pilote)
        ok, jpeg = cv2.imencode(".jpg", im)
        if ok:
            self.bus.publier(
                S.IMAGE_DRONE_SIMULEE,
                ImageCamera(
                    t=time.monotonic(),
                    camera="drone_simule",
                    jpeg=bytes(jpeg),
                    largeur=320,
                    hauteur=240,
                    numero=d.image_sequence,
                ),
            )

    def _archiver(self) -> None:
        self.dossier.mkdir(parents=True, exist_ok=True)
        if sum(p.stat().st_size for p in self.dossier.rglob("*") if p.is_file()) > 95 * 1024**2:
            raise ValueError("quota d'archives simulées atteint (100 Mio)")
        session = self.dossier / (time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:8])
        session.mkdir()
        for i, photo in enumerate(self._photos, 1):
            if not cv2.imwrite(str(session / f"vue-{i}.jpg"), photo):
                raise OSError("capture simulée impossible")
        if len(self._photos) == 6:
            montage = cv2.vconcat([cv2.hconcat(self._photos[:3]), cv2.hconcat(self._photos[3:])])
            if not cv2.imwrite(str(session / "montage.jpg"), montage):
                raise OSError("montage simulé impossible")
        d = self.drone
        rapport = {
            **self.pilote.etat(),
            "source": "simulation",
            "mode": self.mode,
            "journal": self.pilote.journal,
            "distance_pose_simulee_m": math.hypot(d.x, d.y),
            "actions": d.actions,
            "au_sol_simule": d.au_sol,
        }
        (session / "rapport.json").write_text(
            json.dumps(rapport, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        self._session = session

    def fichier(self, nom: str) -> Path | None:
        if self._session is None or nom not in {"rapport.json", "montage.jpg"}:
            return None
        p = self._session / nom
        return p if p.is_file() else None

    def arreter(self) -> None:
        self._arret.set()
        if self._fil is not None and self._fil.is_alive():
            self._fil.join(timeout=1)
