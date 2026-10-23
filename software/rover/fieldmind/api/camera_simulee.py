"""Transport des caméras WebGL : un navigateur volontaire par rover simulé.

Une trame binaire contient un petit en-tête JSON, LF, puis un JPEG. Ce canal ne
modifie aucune commande de conduite et n'est jamais accepté sur le matériel réel.
"""

from __future__ import annotations

import asyncio
import json
import time
from typing import Any


def recevoir(serveur: Any, poste: Any, donnees: bytes) -> None:
    cameras = getattr(serveur.rover.materiel, "cameras_navigateur", None)
    if cameras is None or getattr(serveur, "_poste_camera", None) is not poste:
        return
    fin = donnees.find(b"\n", 0, 256)
    if fin < 0:
        return
    try:
        entete = json.loads(donnees[:fin])
        if isinstance(entete, dict):
            cameras.recevoir(entete.get("id"), entete.get("camera"), donnees[fin + 1 :])
    except (ValueError, TypeError):
        return


async def diffuser(serveur: Any, poste: Any) -> None:
    materiel = serveur.rover.materiel
    cameras = getattr(materiel, "cameras_navigateur", None)
    if cameras is None:
        return
    version = None
    try:
        while not poste.ws.closed:
            proprietaire = getattr(serveur, "_poste_camera", None)
            if not getattr(poste, "cameras_pretes", False) or proprietaire not in (None, poste):
                await asyncio.sleep(0.2)
                continue
            serveur._poste_camera = poste
            debut = time.monotonic()
            description = cameras.description()
            if description["version"] != version:
                poste.envoyer({"type": "monde_simule", **description})
                version = description["version"]
            vues = {
                role: (
                    role if not materiel._inverse else ("droite" if role == "gauche" else "gauche")
                )
                for role in ("gauche", "droite")
                if materiel.profil.valeur(f"capteurs.camera_{role}", True)
            }
            demande = cameras.nouvelle_demande(materiel.physique.pose, vues)
            if demande:
                poste.envoyer(demande)
                try:
                    await asyncio.wait_for(cameras.paire_recue.wait(), timeout=1.5)
                except TimeoutError:
                    pass
                if not cameras.complete():
                    cameras.abandonner()
            await asyncio.sleep(max(0.01, 1 / cameras.ips - (time.monotonic() - debut)))
    finally:
        if getattr(serveur, "_poste_camera", None) is poste:
            serveur._poste_camera = None
            cameras.abandonner()
