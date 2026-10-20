"""Détecteur parfait pour la simulation : les boîtes englobantes des troncs couchés, tirées de la
vérité terrain au lieu d'un modèle de vision.

Il remplace le modèle de vision dans les tests, l'évaluation automatique et la démo sans clé
d'API, avec la même interface (`FournisseurVision`) : le cerveau reçoit des boîtes dans l'image
et doit les placer au sol lui-même, avec sa pose estimée. Ce qui est mesuré est donc la chaîne
complète (géométrie de la caméra, localisation, carte, calcul du chemin), avec une détection
idéale. Il ne voit que les troncs couchés (les obstacles que le lidar ne voit pas) et respecte
les occultations (un tronc caché derrière un arbre n'est pas vu).
"""

from __future__ import annotations

import asyncio
import math
from collections.abc import Callable

import mujoco
import numpy as np

from services.perception.camera import ModeleCamera
from services.perception.vision import AnalyseVision, DemandeVision, ObjetVu
from simulation.modele_mujoco import GROUPE_MONDE
from simulation.robot_simule import RobotSimule

PORTEE_M = 11.0
"""Au-delà, un tronc de 25 cm de haut fait moins de 8 pixels : on considère qu'il n'est pas vu."""


class DetecteurVerite:
    nom = "simulation · détecteur parfait (vérité terrain)"

    def __init__(self, robot: Callable[[], RobotSimule], latence_s: float = 1.5) -> None:
        self._robot = robot
        self.latence_s = latence_s

    async def analyser(self, demande: DemandeVision) -> AnalyseVision:
        analyse = self.analyser_immediat(demande)
        await asyncio.sleep(self.latence_s)
        return analyse

    def analyser_immediat(self, demande: DemandeVision) -> AnalyseVision:
        robot = self._robot()
        camera = robot.fiche.capteurs.camera
        verite = robot.verite_a(demande.t)
        if camera is None or verite is None:
            return AnalyseVision(resume="Image indisponible.", objets=[])
        modele = ModeleCamera(camera)
        m, d = robot.modele, robot.donnees
        cx, cy, cz = camera.position
        c, s = math.cos(verite.cap), math.sin(verite.cap)
        sol_rover = verite.z - robot.fiche.geometrie.rayon_roue
        oeil = np.array([verite.x + c * cx - s * cy, verite.y + s * cx + c * cy, sol_rover + cz])
        groupes = np.zeros(6, dtype=np.uint8)
        groupes[GROUPE_MONDE] = 1
        geom = np.zeros(1, dtype=np.int32)
        objets: list[ObjetVu] = []
        for i, tronc in enumerate(robot.monde.troncs_couches):
            id_geom = m.geom(f"tronc_couche_{i}").id
            (xa, ya), (xb, yb) = tronc.extremites
            if math.hypot(tronc.x - oeil[0], tronc.y - oeil[1]) > PORTEE_M + tronc.longueur:
                continue
            points_bas: list[tuple[float, float]] = []
            points_haut: list[tuple[float, float]] = []
            for k in np.linspace(0.0, 1.0, 13):
                px, py = xa + k * (xb - xa), ya + k * (yb - ya)
                if math.hypot(px - oeil[0], py - oeil[1]) > PORTEE_M:
                    continue
                cible = np.array([px, py, robot.monde.hauteur_sol(px, py) + tronc.rayon])
                direction = cible - oeil
                distance = float(np.linalg.norm(direction))
                touche = mujoco.mj_ray(
                    m, d, oeil, direction / distance, groupes, 1, robot_id(robot), geom
                )
                if int(geom[0]) != id_geom and 0 <= touche < distance - tronc.rayon - 0.05:
                    continue  # caché par un arbre ou un rocher
                bas = modele.projeter(px, py, 0.0, verite.x, verite.y, verite.cap)
                haut = modele.projeter(px, py, 2 * tronc.rayon, verite.x, verite.y, verite.cap)
                if bas is None or haut is None:
                    continue
                if not (0 <= bas[0] <= 1 and haut[1] <= 1 and bas[1] >= 0):
                    continue
                points_bas.append((bas[0], bas[1]))
                points_haut.append((haut[0], haut[1]))
            if len(points_bas) < 2:
                continue
            us = [u for u, _ in points_bas + points_haut]
            x0, x1 = max(min(us), 0.0), min(max(us), 1.0)
            y0 = max(min(v for _, v in points_haut), 0.0)
            y1 = min(max(v for _, v in points_bas), 1.0)
            objets.append(
                ObjetVu(
                    categorie="tronc_couche",
                    boite=(x0 * 1000, y0 * 1000, x1 * 1000, y1 * 1000),
                    bloque_passage=True,
                    confiance=1.0,
                )
            )
        n = len(objets)
        resume = (
            f"Détecteur parfait de la simulation : {n} tronc(s) couché(s) dans le champ."
            if n
            else "Détecteur parfait de la simulation : aucun tronc couché dans le champ."
        )
        return AnalyseVision(resume=resume, objets=objets)


def robot_id(robot: RobotSimule) -> int:
    """Corps exclu des rayons : le châssis du rover (la caméra est dedans)."""
    return int(robot.modele.body("chassis").id)
