"""Rover simulé exécuté comme un robot distant : même chemin que le rover réel.

Le simulateur tourne dans son propre processus, en temps réel, et se connecte au serveur sur
/ws/robot comme le fera le pont du rover réel (nœud ROS 2 ou liaison série) : il publie ses
messages de capteurs et applique les consignes reçues. Il ne transmet aucune vérité terrain.

    python -m simulation.pont_ws --url ws://127.0.0.1:8000/ws/robot --jeton "$FIELDMIND_JETON_ROBOT"

Puis ouvrir le poste opérateur sur http://127.0.0.1:8000/?robot=externe
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import time
from pathlib import Path

import websockets
from pydantic import ValidationError

from services.protocole.fiche_rover import FicheRover
from services.protocole.messages import Commande
from simulation.monde import charger_monde
from simulation.robot_simule import RobotSimule

RACINE = Path(__file__).resolve().parents[1]
PERIODE_S = 0.01
journal = logging.getLogger("fieldmind.pont_ws")


async def executer(url: str, jeton: str, graine: int | None, duree_max_s: float | None) -> None:
    fiche = FicheRover.charger(RACINE / "config" / "rover_fieldmind.json")
    monde = charger_monde(RACINE / "simulation" / "scenarios" / "foret_demo.json", graine)
    robot = RobotSimule(fiche, monde, graine_bruit=monde.graine)
    separateur = "&" if "?" in url else "?"
    async with websockets.connect(f"{url}{separateur}jeton={jeton}", max_size=2**20) as ws:
        journal.info("connecté à %s", url)

        async def recevoir_consignes() -> None:
            async for texte in ws:
                try:
                    robot.recevoir(Commande.model_validate_json(texte))
                except ValidationError:
                    journal.warning("consigne invalide ignorée")

        reception = asyncio.create_task(recevoir_consignes())
        debut = precedent = time.perf_counter()
        try:
            while duree_max_s is None or time.perf_counter() - debut < duree_max_s:
                await asyncio.sleep(PERIODE_S)
                maintenant = time.perf_counter()
                messages = robot.avancer(min(maintenant - precedent, 0.1))
                precedent = maintenant
                for message in messages:
                    await ws.send(message.model_dump_json())
                if reception.done():
                    break
        finally:
            reception.cancel()
            with contextlib.suppress(asyncio.CancelledError, websockets.ConnectionClosed):
                await reception


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("--url", default="ws://127.0.0.1:8000/ws/robot")
    parser.add_argument("--jeton", required=True)
    parser.add_argument("--graine", type=int, default=None)
    parser.add_argument("--duree", type=float, default=None, help="durée maximale (s)")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    asyncio.run(executer(args.url, args.jeton, args.graine, args.duree))


if __name__ == "__main__":
    main()
