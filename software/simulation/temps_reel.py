"""Exécution du rover simulé en temps réel, derrière l'interface `LiaisonRobot`.

La simulation avance au rythme de l'horloge murale ; ses messages sont mis en file comme s'ils
arrivaient d'un robot par le réseau. Une latence de transmission peut être ajoutée dans chaque sens.

Les images de la caméra sont rendues dans un fil dédié (le rendu libère le GIL et ne ralentit pas
la physique) ; une prise de vue arrive donc un peu après les autres mesures du même instant, comme
sur le vrai rover.
"""

from __future__ import annotations

import asyncio
import contextlib
import time
from concurrent.futures import ThreadPoolExecutor

from services.protocole.liaison import MessageRobot
from services.protocole.messages import Commande, Image, ImageStereo
from simulation.camera_simulee import CameraSimulee
from simulation.robot_simule import RobotSimule, VeriteTerrain

PERIODE_BOUCLE_S = 0.01
RETARD_MAX_S = 0.1
"""Au-delà, la simulation « perd » le temps au lieu de rattraper (machine surchargée)."""


class SimulationTempsReel:
    """Rover simulé, avançant en temps réel dans une tâche asyncio."""

    def __init__(self, robot: RobotSimule, latence_s: float = 0.0, taille_file: int = 2000) -> None:
        self.robot = robot
        self.latence_s = latence_s
        self._file: asyncio.Queue[MessageRobot] = asyncio.Queue(maxsize=taille_file)
        self._commandes: list[tuple[float, Commande]] = []
        self._tache: asyncio.Task[None] | None = None
        self.messages_perdus = 0
        self.charge_cpu = 0.0
        """Fraction d'un cœur utilisée par la physique et les capteurs (moyenne glissante)."""
        self.charge_camera = 0.0
        """Fraction d'un cœur utilisée par le rendu de la caméra (fil dédié)."""
        self._executeur = ThreadPoolExecutor(max_workers=1, thread_name_prefix="camera")
        self._camera = self._nouvelle_camera(robot)
        self._rendu_en_cours = False
        self._t_charge_camera = time.perf_counter()
        self._duree_rendu = 0.0

    @staticmethod
    def _nouvelle_camera(robot: RobotSimule) -> CameraSimulee | None:
        camera = robot.fiche.capteurs.camera
        if not robot.avec_camera or camera is None:
            return None
        return CameraSimulee(robot.modele, camera, robot.fiche.capteurs.stereo)

    @property
    def erreur_camera(self) -> str | None:
        return None if self._camera is None else self._camera.erreur

    def demarrer(self) -> None:
        self._tache = asyncio.create_task(self._boucle(), name="simulation")

    async def _boucle(self) -> None:
        precedent = time.perf_counter()
        while True:
            await asyncio.sleep(PERIODE_BOUCLE_S)
            maintenant = time.perf_counter()
            ecoule = min(maintenant - precedent, RETARD_MAX_S)
            precedent = maintenant
            self._appliquer_commandes(maintenant)
            debut_cpu = time.perf_counter()
            messages = self.robot.avancer(ecoule)
            duree_cpu = time.perf_counter() - debut_cpu
            self.charge_cpu = 0.95 * self.charge_cpu + 0.05 * (duree_cpu / max(ecoule, 1e-3))
            for message in messages:
                self._deposer(message)
            self._lancer_rendu()

    def _lancer_rendu(self) -> None:
        capture = self.robot.prendre_capture()
        camera = self._camera
        if capture is None or camera is None or self._rendu_en_cours or camera.erreur:
            return
        self._rendu_en_cours = True
        robot = self.robot
        future = asyncio.get_running_loop().run_in_executor(
            self._executeur, camera.rendre, capture.qpos, capture.t
        )

        def rendue(f: asyncio.Future[list[Image | ImageStereo]]) -> None:
            self._rendu_en_cours = False
            if f.cancelled() or f.exception() is not None or robot is not self.robot:
                return
            images = f.result()
            if images:
                self._duree_rendu += camera.duree_ms / 1000
            for image in images:
                self._deposer(image)
            maintenant = time.perf_counter()
            if maintenant - self._t_charge_camera >= 2.0:
                self.charge_camera = self._duree_rendu / (maintenant - self._t_charge_camera)
                self._duree_rendu, self._t_charge_camera = 0.0, maintenant

        future.add_done_callback(rendue)

    def _deposer(self, message: MessageRobot) -> None:
        if self.latence_s > 0:
            loop = asyncio.get_running_loop()
            loop.call_later(self.latence_s, self._deposer_immediat, message)
        else:
            self._deposer_immediat(message)

    def _deposer_immediat(self, message: MessageRobot) -> None:
        try:
            self._file.put_nowait(message)
        except asyncio.QueueFull:
            self.messages_perdus += 1

    def _appliquer_commandes(self, maintenant: float) -> None:
        pretes = [c for echeance, c in self._commandes if echeance <= maintenant]
        self._commandes = [(e, c) for e, c in self._commandes if e > maintenant]
        for commande in pretes:
            self.robot.recevoir(commande)

    def remplacer_robot(self, robot: RobotSimule) -> None:
        """Remplace le rover (réinitialisation) ; les messages en attente de l'ancien sont jetés."""
        ancienne = self._camera
        if ancienne is not None:
            self._executeur.submit(ancienne.fermer)
        self.robot = robot
        self._camera = self._nouvelle_camera(robot)
        self._commandes.clear()
        while not self._file.empty():
            self._file.get_nowait()

    # ------------------------------------------------------------------ LiaisonRobot
    async def recevoir(self) -> MessageRobot:
        return await self._file.get()

    async def envoyer(self, commande: Commande) -> None:
        if self.latence_s > 0:
            self._commandes.append((time.perf_counter() + self.latence_s, commande))
        else:
            self.robot.recevoir(commande)

    async def fermer(self) -> None:
        if self._tache is not None:
            self._tache.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._tache
        if self._camera is not None:
            self._executeur.submit(self._camera.fermer)
        self._executeur.shutdown(wait=False)

    # ------------------------------------------------------------------ affichage
    def verite(self) -> VeriteTerrain:
        return self.robot.verite()
