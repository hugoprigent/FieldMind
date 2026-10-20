"""Ce que les outils du GX10 ajoutent au rover : détection continue, voix du rover, mémoire des
lieux. Tout est facultatif et conseil seulement : sans GX10 (ou liaison coupée), rien ne change
dans la conduite ni dans la sécurité.

- **Détection continue** (D-FINE + profondeur) : une petite image de la caméra gauche toutes les
  `ia.detection_periode_s` ; personnes et animaux publiés sur `S.DETECTIONS` (le superviseur
  ralentit près d'eux, la vue 3D les montre). Un objet sans position au sol est placé avec la
  profondeur métrique, dans la direction de la boîte.
- **Voix du rover** (Qwen3-TTS) : chaque réponse du rover dans le chat devient un WAV publié sur
  `S.PAROLE`, que le poste joue.
- **Mémoire des lieux** (Qwen3-VL-Embedding) : à chaque lieu mémorisé, l'empreinte de la vue est
  gardée ; « que vois-tu ? » dit ensuite à quel lieu la vue ressemble.
"""

from __future__ import annotations

import asyncio
import base64
import json
import logging
import math
import time
from pathlib import Path
from typing import Any

from fieldmind.bus import Bus, S
from fieldmind.ia.fournisseurs import ErreurIA
from fieldmind.ia.inference import ClientInference, similarite
from fieldmind.ia.vision import preparer_image

journal = logging.getLogger("fieldmind.ia.distant")

LABELS_FR = {"person": "personne", "dog": "chien", "cat": "chat", "horse": "cheval",
             "cow": "vache", "bird": "oiseau", "sheep": "mouton", "bicycle": "vélo",
             "car": "voiture", "motorcycle": "moto", "bench": "banc", "chair": "chaise",
             "potted plant": "plante", "backpack": "sac à dos", "ball": "ballon",
             "sports ball": "ballon", "bottle": "bouteille", "cup": "tasse"}
VIVANTS = {"personne", "chien", "chat", "cheval", "vache", "oiseau", "mouton"}
LARGEUR_DETECTION = 416  # px : ≈ 15 ko en JPEG, une demi-seconde sur une liaison lente
FICHIER_SOUVENIRS = Path.home() / ".config" / "fieldmind" / "souvenirs.json"
SEUIL_RESSEMBLANCE = 0.85


def position_au_sol(outils: Any, profil: Any, camera: str, boite: list[float],
                    distance: float | None) -> list[float] | None:
    """Pied de la boîte projeté au sol ; sinon, distance métrique dans la direction de la boîte
    (champ horizontal de la caméra)."""
    u, v = (boite[0] + boite[2]) / 2, boite[3]
    try:
        sol = outils.pixel_vers_sol(camera, u, v)
    except Exception:
        sol = None
    if sol is not None:
        return [round(float(sol[0]), 2), round(float(sol[1]), 2)]
    if distance is None or not math.isfinite(distance) or distance <= 0:
        return None
    champ = math.radians(float(profil.valeur("cameras.hfov_deg", 70.0)))
    azimut = (0.5 - u) * champ
    x_cam = float(profil.valeur("cameras.x_m", 0.415))
    return [round(x_cam + distance * math.cos(azimut), 2), round(distance * math.sin(azimut), 2)]


class DetectionContinue:
    def __init__(self, bus: Bus, profil: Any, client: ClientInference, outils: Any) -> None:
        self.bus, self.profil, self.client, self.outils = bus, profil, client, outils
        self.dernier: dict[str, Any] | None = None
        self.echecs = 0

    def active(self) -> bool:
        return bool(self.profil.valeur("ia.detection_continue", True)) and \
            self.client.disponible("detection")

    async def cycle(self, camera: str = "gauche") -> dict[str, Any] | None:
        image = self.outils.derniere_image(camera)
        if image is None:
            return None
        jpeg, _, _ = await asyncio.to_thread(preparer_image, image, LARGEUR_DETECTION)
        avec_profondeur = self.client.disponible("profondeur")
        reponse = await self.client.analyser(
            jpeg, taches=("detection", "profondeur") if avec_profondeur else ("detection",),
            seuil=float(self.profil.valeur("ia.detection_seuil", 0.45)))
        objets = []
        for det in reponse.get("detections") or []:
            label = LABELS_FR.get(det["label"], det["label"])
            distance = det.get("distance_m")
            objets.append({"label": label, "score": det["score"], "boite": det["boite"],
                           "distance_m": distance, "vivant": label in VIVANTS,
                           "sol": position_au_sol(self.outils, self.profil, camera, det["boite"],
                                                  distance)})
        message = {"t": time.time(), "camera": camera, "source": self.client.modele("detection"),
                   "objets": objets, "durees_ms": reponse.get("durees_ms", {})}
        self.dernier = message
        self.bus.publier(S.DETECTIONS, message)
        return message

    async def boucle(self) -> None:
        await asyncio.sleep(2.0)
        while True:
            periode = float(self.profil.valeur("ia.detection_periode_s", 2.5))
            try:
                if self.active():
                    await self.cycle()
                    if self.echecs:
                        self.bus.journaliser("ia", "détection continue rétablie", "succes")
                    self.echecs = 0
            except asyncio.CancelledError:
                raise
            except ErreurIA as erreur:
                self.echecs += 1
                if self.echecs == 3:  # une fois, pas à chaque tour
                    self.bus.journaliser("ia", f"détection continue en panne : {erreur}", "alerte")
                periode = min(30.0, periode * (1 + self.echecs))
            except Exception:
                journal.exception("détection continue")
            await asyncio.sleep(periode)


class VoixRover:
    def __init__(self, bus: Bus, profil: Any, client: ClientInference) -> None:
        self.bus, self.profil, self.client = bus, profil, client
        self._file: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=4)
        self._boucle_asyncio: asyncio.AbstractEventLoop | None = None
        bus.abonner(S.CHAT, self._chat)

    def active(self) -> bool:
        return bool(self.profil.valeur("ia.voix_rover", True)) and \
            self.client.disponible("synthese")

    def _chat(self, message: Any) -> None:
        if not (isinstance(message, dict) and message.get("role") == "rover"
                and message.get("texte") and self.active() and self._boucle_asyncio):
            return
        # Appelé dans le fil de l'émetteur : la file appartient à la boucle asyncio
        self._boucle_asyncio.call_soon_threadsafe(self._deposer, message)

    def _deposer(self, message: dict[str, Any]) -> None:
        if not self._file.full():  # en retard : les anciennes phrases passent avant
            self._file.put_nowait(message)

    async def boucle(self) -> None:
        self._boucle_asyncio = asyncio.get_running_loop()
        while True:
            message = await self._file.get()
            try:
                debut = time.monotonic()
                wav = await self.client.parole(str(message["texte"]))
                self.bus.publier(S.PAROLE, {
                    "type": "parole", "id": message.get("id"), "t": time.time(),
                    "texte": message["texte"], "audio": base64.b64encode(wav).decode("ascii"),
                    "format": "wav", "modele": self.client.modele("synthese"),
                    "duree_s": round(time.monotonic() - debut, 2)})
            except asyncio.CancelledError:
                raise
            except ErreurIA as erreur:
                journal.info("voix du rover indisponible : %s", erreur)
            except Exception:
                journal.exception("voix du rover")


class MemoireLieux:
    def __init__(self, client: ClientInference, fichier: Path = FICHIER_SOUVENIRS) -> None:
        self.client, self.fichier = client, fichier
        self.souvenirs: list[dict[str, Any]] = self._lire()

    def _lire(self) -> list[dict[str, Any]]:
        try:
            donnees = json.loads(self.fichier.read_text(encoding="utf-8"))
            return [s for s in donnees if isinstance(s, dict) and s.get("vecteur")]
        except (OSError, ValueError):
            return []

    def _ecrire(self) -> None:
        try:
            self.fichier.parent.mkdir(parents=True, exist_ok=True)
            self.fichier.write_text(json.dumps(self.souvenirs), encoding="utf-8")
        except OSError:
            journal.exception("souvenirs non enregistrés")

    def disponible(self) -> bool:
        return self.client.disponible("memoire")

    async def memoriser(self, nom: str, image: Any) -> bool:
        if image is None or not self.disponible():
            return False
        jpeg, _, _ = await asyncio.to_thread(preparer_image, image, 512)
        vecteur = await self.client.empreinte(jpeg)
        self.souvenirs = [s for s in self.souvenirs if s.get("nom") != nom]
        self.souvenirs.append({"nom": nom, "t": time.time(), "vecteur": vecteur})
        self._ecrire()
        return True

    async def reconnaitre(self, jpeg: bytes) -> tuple[str, float] | None:
        """(lieu le plus ressemblant, similarité) si elle dépasse le seuil."""
        if not self.souvenirs or not self.disponible():
            return None
        vecteur = await self.client.empreinte(jpeg)
        nom, score = max(((s["nom"], similarite(vecteur, s["vecteur"])) for s in self.souvenirs),
                         key=lambda x: x[1])
        return (nom, score) if score >= SEUIL_RESSEMBLANCE else None
