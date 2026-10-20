"""Lancement : python -m fieldmind [--materiel reel|simulation] [--port 8080]

Sur le Pi : `python3 -m fieldmind` (service fieldmind-rover). Sur un PC, sans le rover :
`python -m fieldmind --materiel simulation --scenario salle`, puis http://127.0.0.1:8080/.
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
from pathlib import Path

from aiohttp import web

from fieldmind.app import Rover
from fieldmind.config import FICHIER_PROFIL, Profil


def charger_env(fichier: Path) -> None:
    """Variables d'un fichier .env (CLE=valeur), sans écraser l'environnement."""
    if not fichier.exists():
        return
    for ligne in fichier.read_text(encoding="utf-8").splitlines():
        ligne = ligne.strip()
        if not ligne or ligne.startswith("#") or "=" not in ligne:
            continue
        cle, _, valeur = ligne.partition("=")
        os.environ.setdefault(cle.strip(), valeur.strip().strip('"').strip("'"))


def main() -> None:
    parseur = argparse.ArgumentParser(description="Logiciel du rover FieldMind")
    parseur.add_argument("--materiel", choices=("reel", "simulation"), default="reel")
    parseur.add_argument("--scenario", default="salle", help="simulation : salle ou jardin")
    parseur.add_argument("--graine", type=int, default=0, help="simulation : monde n°")
    parseur.add_argument("--esp", default="ROVER_3", help="simulation : version du firmware")
    parseur.add_argument("--hote", default=None)
    parseur.add_argument("--port", type=int, default=None)
    parseur.add_argument("--profil", type=Path, default=FICHIER_PROFIL)
    parseur.add_argument("--enregistrer", action=argparse.BooleanOptionalAction, default=None,
                         help="boîte noire (défaut : oui sur le rover, non en simulation)")
    parseur.add_argument("-v", "--verbeux", action="store_true")
    args = parseur.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbeux else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s : %(message)s")
    logging.getLogger("aiohttp.access").setLevel(logging.WARNING)
    racine = Path(__file__).resolve().parent.parent
    charger_env(racine / ".env")
    charger_env(Path("~/.config/fieldmind/env").expanduser())

    profil = Profil.charger(args.profil)
    simulation = args.materiel == "simulation"
    options: dict[str, object] = {"scenario": args.scenario, "graine": args.graine,
                                  "version_esp": args.esp} if simulation else {}
    if args.enregistrer is not None:
        options["enregistrer"] = args.enregistrer
    rover = Rover(profil, simulation=simulation, **options)

    from fieldmind.api.serveur import Serveur

    serveur = Serveur(rover)
    rover.demarrer()

    def arreter(*_: object) -> None:
        rover.arreter()  # traction à zéro avant tout le reste
        raise web.GracefulExit()

    signal.signal(signal.SIGTERM, arreter)
    hote = args.hote or str(profil.valeur("serveur.hote", "0.0.0.0"))
    port = args.port or int(profil.valeur("serveur.port", 8080))
    try:
        web.run_app(serveur.app, host=hote, port=port, print=None,
                    handle_signals=False, shutdown_timeout=2)
    except KeyboardInterrupt:
        pass
    finally:
        rover.arreter()


if __name__ == "__main__":
    main()
