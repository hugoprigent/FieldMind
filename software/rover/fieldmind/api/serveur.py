"""Serveur web du rover (aiohttp) : poste opérateur, WebSocket `/ws`, MJPEG, pont `/ws/ros`.

Contrat : PROTOCOLE.md (poste) et ARCHITECTURE.md, section `/ws/ros` (pont ROS 2).
Sans jeton (`FIELDMIND_JETON`), à n'ouvrir que sur le réseau local de confiance.
"""

from __future__ import annotations

import asyncio
import base64
import contextlib
import itertools
import json
import math
import os
import secrets
import time
from collections import deque
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from aiohttp import WSMsgType, web

from fieldmind import __version__
from fieldmind.api import camera_simulee
from fieldmind.api import diffusion as D
from fieldmind.bus import S
from fieldmind.config import REGLABLES, TEXTES_REGLABLES, braquage_max, details_reglables
from fieldmind.drone.atelier import AtelierDrone
from fieldmind.mission.actions import ActionRefusee
from fieldmind.types import Consigne, ImageCamera, Imu, Odometrie, Scan

DOSSIER_WEB = Path(__file__).resolve().parent.parent / "web"
SUJETS_EVENEMENTS = {S.JOURNAL: "journal", S.CHAT: "chat", S.VOIX: "voix", S.VISION: "vision",
                     S.PAROLE: "parole", "calibration": "calibration"}


class Poste:
    def __init__(self, ws: web.WebSocketResponse) -> None:
        self.ws = ws
        self.ident = f"poste-{secrets.token_hex(2)}"
        self.file: asyncio.Queue[str] = asyncio.Queue(200)
        self.dernier_pilotage = 0.0

    def envoyer(self, message: dict[str, Any]) -> None:
        texte = json.dumps(message, ensure_ascii=False, separators=(",", ":"))
        if self.file.full():  # poste trop lent : on jette les plus vieux messages
            with contextlib.suppress(asyncio.QueueEmpty):
                self.file.get_nowait()
        self.file.put_nowait(texte)


class Serveur:
    def __init__(self, rover: Any, jeton: str | None = None) -> None:
        self.rover = rover
        self.bus = rover.bus
        self.atelier_drone = AtelierDrone(self.bus)
        self.dossier_photos = None
        self.jeton = jeton if jeton is not None else (os.environ.get("FIELDMIND_JETON") or None)
        self.postes: dict[str, Poste] = {}
        self.ponts_ros: set[web.WebSocketResponse] = set()
        self._journal: deque[dict[str, Any]] = deque(maxlen=300)
        self._console: deque[list[Any]] = deque(maxlen=300)
        self._taches: list[asyncio.Task[Any]] = []
        self._en_cours: set[asyncio.Task[Any]] = set()
        self.boucle: asyncio.AbstractEventLoop | None = None
        self._ids = itertools.count(1)
        self.app = web.Application(middlewares=[self._securite])
        self.app.add_routes([
            web.get("/", self._accueil),
            web.get("/ws", self._ws_poste),
            web.get("/ws/ros", self._ws_ros),
            web.get("/flux/{camera}", self._flux),
            web.get("/photo/{camera}", self._photo),
            web.get("/api/etat", self._api_etat),
            web.get("/api/profil", self._api_profil),
            web.get("/api/journal", self._api_journal),
            web.get("/api/sante", self._api_sante),
            web.get("/api/drone/recon/{fichier}", self._recon_fichier),
            web.get("/api/drone/atelier/{fichier}", self._atelier_fichier),
        ])
        if DOSSIER_WEB.exists():
            self.app.router.add_static("/", DOSSIER_WEB, show_index=False)
        self.app.on_startup.append(self._au_demarrage)
        self.app.on_shutdown.append(self._a_l_arret)

    # ------------------------------------------------------------------ cycle de vie

    async def _au_demarrage(self, _app: web.Application) -> None:
        self.boucle = asyncio.get_running_loop()
        for sujet, nom in SUJETS_EVENEMENTS.items():
            self._taches.append(asyncio.create_task(self._relayer(sujet, nom)))
        self._taches.append(asyncio.create_task(self._relayer_console()))
        self._taches.append(asyncio.create_task(self._diffuser()))
        await self.rover.demarrer_ia(self.boucle)

    async def _a_l_arret(self, _app: web.Application) -> None:
        self.atelier_drone.arreter()
        for tache in self._taches:
            tache.cancel()
        for poste in list(self.postes.values()):
            await poste.ws.close()
        for ws in list(self.ponts_ros):
            await ws.close()
        if self.rover.ia is not None:
            with contextlib.suppress(Exception):
                await self.rover.ia.fermer()

    @web.middleware
    async def _securite(self, requete: web.Request, handler: Any) -> web.StreamResponse:
        if self.jeton and requete.path not in ("/", "/api/sante") and not requete.path.startswith(
                ("/css/", "/js/", "/img/", "/vendor/", "/favicon")):
            fourni = requete.query.get("jeton") or requete.headers.get("X-Fieldmind-Jeton", "")
            if not secrets.compare_digest(fourni, self.jeton):
                raise web.HTTPUnauthorized(text="jeton manquant ou faux")
        reponse = await handler(requete)
        reponse.headers.setdefault("Cache-Control", "no-store")
        return reponse

    # ------------------------------------------------------------------ HTTP

    async def _atelier_fichier(self, requete: web.Request) -> web.StreamResponse:
        chemin = self.atelier_drone.fichier(requete.match_info["fichier"])
        if chemin is None:
            raise web.HTTPNotFound(text="archive de simulation absente")
        return web.FileResponse(chemin)

    async def _recon_fichier(self, requete: web.Request) -> web.StreamResponse:
        esp = self.rover.materiel.esp()
        recon = getattr(esp, "recon", None)
        chemin = None if recon is None else recon.fichier(requete.match_info["fichier"])
        if chemin is None:
            raise web.HTTPNotFound(text="capture de reconnaissance absente")
        return web.FileResponse(chemin)

    async def _accueil(self, _requete: web.Request) -> web.StreamResponse:
        index = DOSSIER_WEB / "index.html"
        if not index.exists():
            return web.Response(text="poste opérateur absent (fieldmind/web/index.html)",
                                status=503)
        return web.FileResponse(index)

    async def _api_etat(self, _requete: web.Request) -> web.Response:
        return web.json_response(D.message_etat(self.rover))

    async def _api_profil(self, _requete: web.Request) -> web.Response:
        return web.json_response(self.rover.profil.dict())

    async def _api_journal(self, requete: web.Request) -> web.Response:
        n = max(1, min(300, int(requete.query.get("n", 200))))
        return web.json_response(list(self._journal)[-n:])

    async def _api_sante(self, _requete: web.Request) -> web.Response:
        return web.json_response({"ok": True, "version": __version__})

    async def _photo(self, requete: web.Request) -> web.Response:
        donnees = self._derniere_image(requete.match_info["camera"])
        if donnees is None:
            raise web.HTTPServiceUnavailable(text="pas d'image")
        nom = f"{requete.match_info['camera']}_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
        return web.Response(body=donnees, content_type="image/jpeg",
                            headers={"Content-Disposition": f'inline; filename="{nom}"'})

    def _derniere_image(self, camera: str) -> bytes | None:
        if camera == "drone_simule":
            im = self.bus.dernier(S.IMAGE_DRONE_SIMULEE)
            return im.jpeg if im is not None else None
        if camera == "profondeur":
            return self.bus.dernier(S.PROFONDEUR)
        if camera == "drone":
            image_drone: ImageCamera | None = self.bus.dernier(S.IMAGE_DRONE)
            if image_drone is not None:
                return image_drone.jpeg
            from fieldmind.materiel.esp import _generer_mire_drone
            from fieldmind.types import EtatDrone

            return _generer_mire_drone(self.bus.dernier(S.DRONE) or EtatDrone(t=time.monotonic()))
        if camera not in ("gauche", "droite"):
            raise web.HTTPNotFound()
        image: ImageCamera | None = self.bus.dernier(
            S.IMAGE_GAUCHE if camera == "gauche" else S.IMAGE_DROITE)
        return image.jpeg if image else None

    async def _flux(self, requete: web.Request) -> web.StreamResponse:
        camera = requete.match_info["camera"]
        sujet = {"gauche": S.IMAGE_GAUCHE, "droite": S.IMAGE_DROITE,
                 "profondeur": S.PROFONDEUR, "drone": S.IMAGE_DRONE,
                 "drone_simule": S.IMAGE_DRONE_SIMULEE}.get(camera)
        if sujet is None:
            raise web.HTTPNotFound()
        ips = min(max(float(requete.query.get("ips", 10)), 0.5), 30)
        largeur = int(requete.query.get("largeur", 0) or 0)
        reponse = web.StreamResponse(headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=image",
            "Cache-Control": "no-store"})
        await reponse.prepare(requete)
        assert self.boucle is not None
        q, desabonner = self.bus.file_async(sujet, self.boucle, 2)
        try:
            derniere = self.bus.dernier(sujet)
            if derniere is None and camera == "drone":
                derniere = self._derniere_image("drone")
            if derniere is not None:
                q.put_nowait(derniere)
            prochaine = 0.0
            while True:
                valeur = await q.get()
                if time.monotonic() < prochaine:
                    continue
                prochaine = time.monotonic() + 1 / ips
                jpeg = valeur if isinstance(valeur, bytes) else valeur.jpeg
                if largeur and isinstance(valeur, ImageCamera) and largeur < valeur.largeur:
                    jpeg = await asyncio.to_thread(_reduire, valeur, largeur)
                await reponse.write(b"--image\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                    + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        finally:
            desabonner()
        return reponse

    # ------------------------------------------------------------------ WebSocket du poste

    async def _ws_poste(self, requete: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20.0, max_msg_size=1 << 20)
        await ws.prepare(requete)
        poste = Poste(ws)
        poste.cameras_pretes = False
        self.postes[poste.ident] = poste
        self.rover.superviseur.operateurs = len(self.postes)
        envoi = asyncio.create_task(self._envoyer_au_poste(poste))
        rendu = asyncio.create_task(camera_simulee.diffuser(self, poste))
        poste.envoyer({**self._bienvenue(), "poste": poste.ident})
        for evenement in list(self._journal)[-100:]:
            poste.envoyer({"type": "journal", **evenement})
        if self._console:
            poste.envoyer({"type": "console", "lignes": list(self._console)[-100:]})
        try:
            async for message in ws:
                if message.type == WSMsgType.TEXT:
                    await self._recevoir(poste, message.data)
                elif message.type == WSMsgType.BINARY:
                    camera_simulee.recevoir(self, poste, message.data)
                elif message.type in (WSMsgType.ERROR, WSMsgType.CLOSE):
                    break
        finally:
            envoi.cancel()
            rendu.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await rendu
            del self.postes[poste.ident]
            self.rover.superviseur.operateurs = len(self.postes)
            if self.rover.conduite.pilote == poste.ident:
                self.rover.conduite.proposer(Consigne(t=time.monotonic(), source="manuel"))
                self.rover.conduite.pilote = None
        return ws

    async def _envoyer_au_poste(self, poste: Poste) -> None:
        with contextlib.suppress(ConnectionResetError, asyncio.CancelledError, RuntimeError):
            while not poste.ws.closed:
                await poste.ws.send_str(await poste.file.get())

    def _bienvenue(self) -> dict[str, Any]:
        profil = self.rover.profil
        return {
            "type": "bienvenue", "version": __version__, "materiel": self.rover.materiel.nom,
            "profil": _profil_publie(profil),
            "reglables": {**{k: list(v) for k, v in REGLABLES.items()},
                          **{k: "texte" for k in TEXTES_REGLABLES}},
            "reglages": profil.reglages(),
            "reglables_details": details_reglables(),
            "cameras": ["gauche", "droite", "drone"],
            "cameras_navigateur": hasattr(self.rover.materiel, "cameras_navigateur"),
            "ia": self.rover.ia.resume() if self.rover.ia is not None else {
                "profil": "aucun", "disponible": False, "modeles": {}, "profils": [],
                "raison": "module IA non chargé"},
        }

    def _diffuser_a_tous(self, message: dict[str, Any]) -> None:
        for poste in list(self.postes.values()):
            poste.envoyer(message)

    async def _relayer(self, sujet: str, nom: str) -> None:
        assert self.boucle is not None
        q, _ = self.bus.file_async(sujet, self.boucle, 100)
        while True:
            valeur = await q.get()
            message = {**valeur, "type": nom} if isinstance(valeur, dict) else {"type": nom,
                                                                               "valeur": valeur}
            if nom == "journal":
                self._journal.append({k: v for k, v in message.items() if k != "type"})
            self._diffuser_a_tous(message)

    async def _relayer_console(self) -> None:
        assert self.boucle is not None
        q, _ = self.bus.file_async(S.CONSOLE_ESP, self.boucle, 400)
        while True:
            lignes = [list(await q.get())]
            await asyncio.sleep(0.1)  # regrouper les lignes par paquets de 100 ms
            while not q.empty():
                lignes.append(list(q.get_nowait()))
            self._console.extend(lignes)
            self._diffuser_a_tous({"type": "console", "lignes": lignes})

    async def _diffuser(self) -> None:
        """etat 10 Hz, scan et obstacles 5 Hz, carte 1 Hz, systeme 1 Hz ; pont ROS 2."""
        tour = 0
        while True:
            debut = time.monotonic()
            try:
                if self.postes:
                    self._diffuser_a_tous(D.message_etat(self.rover))
                    scene = self.bus.dernier(S.SCENE)
                    if tour % 2 == 1 and scene is not None and self.bus.age(S.SCENE) < 1.0:
                        self._diffuser_a_tous(scene)
                    if tour % 2 == 0:
                        scan: Scan | None = self.bus.dernier(S.SCAN)
                        if scan is not None and self.bus.age(S.SCAN) < 0.5:
                            self._diffuser_a_tous(D.message_scan(scan))
                        stereo = self.bus.dernier(S.OBSTACLES_STEREO)
                        if stereo is not None and self.bus.age(S.OBSTACLES_STEREO) < 1.0:
                            self._diffuser_a_tous(D.message_obstacles(stereo))
                    if tour % 10 == 5:
                        but, parcours = self.rover.executeur.buts_gps()
                        self._diffuser_a_tous(self.rover.gps.message(
                            self.bus.dernier(S.ODOM), but, parcours))
                    if tour % 10 == 0:
                        await self._envoyer_carte()
                        systeme = await asyncio.to_thread(self.rover.materiel.systeme)
                        self._diffuser_a_tous(D.message_systeme(systeme))
                if self.ponts_ros:
                    await self._diffuser_ros(tour)
            except Exception as erreur:
                self.bus.journaliser("serveur", f"diffusion : {erreur!r}", "erreur")
            tour += 1
            await asyncio.sleep(max(0.0, 0.1 - (time.monotonic() - debut)))

    async def _envoyer_carte(self) -> None:
        carte = self.bus.dernier(S.CARTE)
        if carte is None:
            return
        estimation = self.rover.estimation
        trajectoire = list(estimation.trajectoire) if estimation is not None else []
        chemin, but = self.rover.executeur.chemin_courant()
        message = await asyncio.to_thread(D.message_carte, carte, trajectoire, chemin, but)
        self._diffuser_a_tous(message)

    # ------------------------------------------------------------------ demandes du poste

    async def _recevoir(self, poste: Poste, texte: str) -> None:
        try:
            message = json.loads(texte)
            if not isinstance(message, dict):
                raise ValueError
        except ValueError:
            return
        ident = message.get("id")
        try:
            resultat = await self._traiter(poste, message)
            if ident is not None:
                poste.envoyer({"type": "reponse", "id": ident, "ok": True,
                               **({"valeur": resultat} if resultat is not None else {})})
        except ActionRefusee as refus:
            if ident is not None:
                poste.envoyer({"type": "reponse", "id": ident, "ok": False,
                               "erreur": refus.raison})
            else:
                poste.envoyer({"type": "journal", "t": time.time(), "niveau": "alerte",
                               "source": "poste", "message": refus.raison})
        except (KeyError, TypeError, ValueError) as erreur:
            if ident is not None:
                poste.envoyer({"type": "reponse", "id": ident, "ok": False,
                               "erreur": f"demande invalide : {erreur}"})

    async def _traiter(self, poste: Poste, m: dict[str, Any]) -> Any:
        r = self.rover
        c = r.conduite
        p = r.profil
        t = m["type"]
        if t == "camera_simulee_prete":
            poste.cameras_pretes = bool(m.get("prete"))
            if not poste.cameras_pretes and getattr(self, "_poste_camera", None) is poste:
                self._poste_camera = None
            return None
        if t == "drone_atelier":
            try:
                self.atelier_drone.commander(str(m.get("action", "")), m)
            except ValueError as e:
                raise ActionRefusee(str(e)) from e
            return None
        if t == "pilotage":
            if c.mode != "manuel":
                return None
            v = max(-1.0, min(1.0, float(m.get("vitesse", 0))))
            b = max(-1.0, min(1.0, float(m.get("braquage", 0))))
            vmax = float(p.valeur("vitesses.max_manuel_m_s", 0.8))
            if v < 0:
                vmax = min(vmax, float(p.valeur("vitesses.max_arriere_m_s", 0.3)))
            c.pilote = poste.ident
            poste.dernier_pilotage = time.monotonic()
            c.proposer(Consigne(t=time.monotonic(), vitesse=v * vmax,
                                braquage=b * braquage_max(p),
                                source="manuel"))
            return None
        if t == "banc":
            if c.mode != "banc":
                raise ActionRefusee("passer en mode banc d'abord")
            plafond = int(p.valeur("traction.pwm_max_banc", 15))
            g = max(-plafond, min(plafond, int(m.get("pwm_g", 0))))
            d = max(-plafond, min(plafond, int(m.get("pwm_d", 0))))
            servo = float(m.get("servo_deg", 0))
            braquage = math.radians(servo / max(float(p.valeur(
                "direction.servo_par_degre_roue", 1.0)), 1e-3))
            c.proposer(Consigne(t=time.monotonic(), source="banc", pwm_brut=(g, d),
                                braquage=braquage))
            return None
        if t == "mode":
            if m["mode"] != "auto" and r.executeur.occupe():
                r.executeur.annuler(f"passage en mode {m['mode']}")
            erreur = c.changer_mode(m["mode"], bool(m.get("confirmer")))
            if erreur:
                raise ActionRefusee(erreur)
            return None
        if t == "armer":
            erreur = c.armer()
            if erreur:
                raise ActionRefusee(erreur)
            return None
        if t == "desarmer":
            c.desarmer()
            return None
        if t == "arret_urgence":
            r.executeur.annuler("arrêt d'urgence")
            c.arret_urgence("poste")
            return None
        if t == "rearmer":
            c.rearmer()
            return None
        if t == "action":
            return r.executeur.ajouter([m["action"]], remplacer=bool(m.get("remplacer")))
        if t == "annuler":
            r.executeur.annuler("annulé depuis le poste")
            return None
        if t == "pause":
            r.executeur.pause()
            return None
        if t == "reprendre":
            r.executeur.reprendre()
            return None
        if t == "aller_pixel":
            return r.executeur.ajouter([{"type": "aller_pixel", "camera": m.get("camera", "gauche"),
                                         "u": m["u"], "v": m["v"]}], remplacer=True)
        if t == "but":
            return r.executeur.ajouter([{"type": "aller_a", "x": m["x"], "y": m["y"],
                                         "repere": "odom"}], remplacer=True)
        if t == "but_gps":
            return r.executeur.ajouter([{"type": "aller_gps", "lat": m["lat"], "lon": m["lon"]}],
                                       remplacer=True)
        if t == "parcours_gps":
            points = list(m["points"])[:50]
            if bool(m.get("boucle")) and points:
                points.append(points[0])
            return r.executeur.ajouter([{"type": "aller_gps", "lat": p[0], "lon": p[1]}
                                        for p in points], remplacer=True)
        if t == "zone":
            try:
                r.gps.regler_zone(m.get("points"))
            except (ValueError, TypeError) as erreur:
                raise ActionRefusee(str(erreur)) from None
            self.bus.journaliser("gps", "zone autorisée retirée" if m.get("points") is None
                                 else f"zone autorisée : {len(m['points'])} sommets", "succes")
            return None
        if t == "lieu":
            action, nom = m.get("action"), str(m.get("nom", ""))
            if action == "memoriser":
                odom = self.bus.dernier(S.ODOM)
                if odom is None:
                    raise ActionRefusee("odométrie indisponible")
                try:
                    lieu = r.gps.memoriser(nom, odom.pose)
                except ValueError as erreur:
                    raise ActionRefusee(str(erreur)) from None
                if r.ia is not None:  # empreinte de la vue (GX10), pour reconnaître le lieu
                    self._lancer(r.ia.memoriser_vue(lieu["nom"]))
                return lieu
            if action == "supprimer":
                r.gps.supprimer(nom)
                return None
            if action == "aller":
                return r.executeur.ajouter([{"type": "aller_lieu", "nom": nom}], remplacer=True)
            raise ActionRefusee("action de lieu inconnue")
        if t == "environnement_reinitialiser":
            r.refuser_si_roule()
            r.reinitialiser_environnement()
            return None
        if t == "remise_a_zero_odom":
            r.refuser_si_roule()
            r.remise_a_zero_odom()
            return None
        if t == "console":
            if not (c.mode == "banc" or not c.arme):
                raise ActionRefusee("console refusée : désarmer ou passer en mode banc")
            esp = r.materiel.esp()
            if esp is None or not esp.envoyer_ligne(str(m["texte"])):
                raise ActionRefusee("ESP absent ou ligne invalide")
            return None
        if t == "reglages_reinitialiser":
            retires = p.reinitialiser()
            self.bus.journaliser("reglages", f"réglages réinitialisés ({len(retires)}) ; "
                                 "branchement et calibrations gardés")
            return retires
        if t == "reglage":
            cle = str(m["cle"])
            try:
                valeur = p.regler(cle, m["valeur"])
            except (KeyError, ValueError) as erreur:
                raise ActionRefusee(str(erreur)) from None
            if cle.startswith("cameras.port"):
                return valeur
            self.bus.journaliser("reglages", f"{cle} = {valeur}")
            return valeur
        if t == "anticollision":
            couper = not bool(m.get("active", True))
            if couper and c.mode != "manuel":
                raise ActionRefusee("seulement en mode manuel")
            r.superviseur.anticollision_coupee = couper
            self.bus.journaliser("securite", "anticollision COUPÉE (manuel, conduite à vue)"
                                 if couper else "anticollision rétablie",
                                 "alerte" if couper else "succes")
            return None
        if t == "decollage_preparer":
            # Volets ouverts (servos à rotation continue, durée réglée) : plateau prêt pour le
            # drone. Aucune commande n'est envoyée au drone lui-même (protocole non validé).
            ouvrir = bool(m.get("ouvrir", True))
            esp = r.materiel.esp()
            if esp is None or not hasattr(esp, "volets_disponibles") or \
                    not esp.volets_disponibles():
                raise ActionRefusee("volets indisponibles (firmware ROVER_4 requis, ESP présent)")
            duree = esp.duree_volets()
            p.regler("remorque.volets_deplies", ouvrir)  # le mouvement part du rappel du réglage
            self._lancer(self._annoncer_volets(ouvrir, duree))
            return duree
        if t == "remorque_alignee":
            if not r.remorque.etat.attelee:
                raise ActionRefusee("aucune remorque attelée (Réglages > Remorque)")
            r.refuser_si_roule()
            r.remorque.aligner()
            return None
        if t == "cameras_inverser":
            r.materiel.cameras_inversees(True)
            return None
        if t == "lidar":
            if bool(m.get("marche")) is False:
                r.refuser_si_roule()
            r.materiel.lidar_marche(bool(m.get("marche")))
            return None
        if t == "photo":
            from fieldmind.mission.executeur import enregistrer_photo

            chemin = await asyncio.to_thread(enregistrer_photo, self.bus,
                                             m.get("camera", "gauche"), self.dossier_photos)
            return None if chemin is None else chemin.name
        if t == "calibration":
            return await self._calibration(m)
        if t == "drone":
            esp = r.materiel.esp()
            if esp is None or not hasattr(esp, "commander_drone"):
                raise ActionRefusee("ESP32 absent")

            def envoyer(commande: str, journal: bool = True) -> None:
                if not esp.commander_drone(commande, journal=journal):
                    raise ActionRefusee(esp.drone_erreur or "commande drone refusée")

            action = str(m.get("action", "etat")).strip()
            if action == "scan":
                envoyer("scan")
            elif action == "connecter":
                ssid = str(m.get("ssid", "auto")).strip() or "auto"
                mdp = str(m.get("mdp", "")).strip()
                envoyer(f"ssid {ssid} {mdp}".strip())
            elif action == "deconnecter":
                envoyer("deconnecter")
            elif action == "flux":
                envoyer(f"flux {1 if m.get('actif', True) else 0}")
            elif action == "cmd":
                cmd = str(m.get("cmd", "stop")).strip()
                if cmd not in (
                    "decoller", "atterrir", "stop", "calibrer", "camera",
                    "cibler_croix", "recon_test", "recon_capture", "recon_vol", "stop_recon",
                ):
                    raise ActionRefusee("commande de vol inconnue")
                envoyer(f"cmd {cmd}")
            elif action == "manette":
                gaz = int(m.get("gaz", 0))
                lacet = int(m.get("lacet", 0))
                tg = int(m.get("tangage", 0))
                rl = int(m.get("roulis", 0))
                envoyer(f"manette {gaz} {lacet} {tg} {rl}", journal=False)
            elif action == "etat":
                envoyer("etat")
            else:
                raise ActionRefusee("action drone inconnue")
            return None
        if t in ("chat", "voix", "vision", "ia_profil", "ia_cle"):
            return await self._ia(t, m)
        raise ActionRefusee(f"demande inconnue : {t}")

    async def _annoncer_volets(self, ouvrir: bool, duree: float) -> None:
        await asyncio.sleep(duree + 0.2)
        if ouvrir:
            self.bus.journaliser("remorque", "prêt au décollage : volets ouverts", "succes")
        self._diffuser_a_tous({"type": "decollage", "pret": bool(
            self.rover.profil.valeur("remorque.volets_deplies", False))})

    def _lancer(self, coroutine: Any) -> None:
        """Tâche de fond gardée en référence jusqu'à sa fin (sinon le ramasse-miettes la coupe)."""
        tache = asyncio.create_task(coroutine)
        self._en_cours.add(tache)
        tache.add_done_callback(self._en_cours.discard)

    async def _ia(self, t: str, m: dict[str, Any]) -> Any:
        ia = self.rover.ia
        if ia is None:
            raise ActionRefusee("IA indisponible")
        if t == "chat":
            texte = str(m.get("texte", "")).strip()[:500]
            if texte:
                self._lancer(ia.consigne(texte, "texte"))
            return None
        if t == "voix":
            actif = bool(m.get("active"))
            if not self.rover.materiel.micro_actif(actif) and actif:
                raise ActionRefusee("micro indisponible")
            ia.voix_active(actif)
            return None
        if t == "vision":
            demande = m.get("demande", "decrire")
            if demande not in ("decrire", "chercher", "analyser"):
                raise ActionRefusee("demande de vision inconnue")
            if demande == "chercher" and not str(m.get("objet", "")).strip():
                raise ActionRefusee("objet à chercher manquant")
            self._lancer(ia.vision(demande, m.get("camera", "gauche"), m.get("objet")))
            return None
        if t == "ia_profil":
            resume = await ia.changer_profil(str(m["profil"]))
            self._diffuser_a_tous(self._bienvenue())
            return resume
        if t == "ia_cle":
            resume = ia.definir_cle(str(m.get("cle", "")), m.get("profil"))
            self._diffuser_a_tous(self._bienvenue())
            return resume
        return None

    async def _calibration(self, m: dict[str, Any]) -> Any:
        r = self.rover
        nom = m.get("nom")
        if m.get("arreter"):
            r.executeur.annuler("calibration interrompue")
            return None
        if nom == "sens_roues":
            from fieldmind.mission.calibration import sens_roues

            sens_roues(self.bus, r.conduite)
            return None
        if nom in ("vitesse", "braquage", "imu_repos"):
            return r.executeur.ajouter([{"type": "calibration", "nom": nom}], remplacer=True,
                                       origine="calibration")
        if nom in ("stereo_capture", "stereo_calculer", "stereo_effacer", "stereo_apercu"):
            return await asyncio.to_thread(self._calibration_stereo, nom)
        raise ActionRefusee("calibration inconnue")

    def _calibration_stereo(self, nom: str) -> Any:
        from fieldmind.mission.calibration import annoncer
        from fieldmind.perception.calibration_stereo import CalibrationStereo

        r = self.rover
        if r.calibration_stereo is None or nom == "stereo_effacer":
            r.calibration_stereo = CalibrationStereo(r.profil)
            if nom == "stereo_effacer":
                annoncer(self.bus, "stereo", "en_cours", "paires effacées")
                return 0
        cal = r.calibration_stereo
        if nom == "stereo_apercu":  # assistant : mire vue dans chaque caméra, en direct
            from fieldmind.perception.calibration_stereo import PAIRES_MIN

            g = self.bus.dernier(S.IMAGE_GAUCHE)
            d = self.bus.dernier(S.IMAGE_DROITE)
            return {"gauche": cal.apercu(g.bgr()) if g is not None else None,
                    "droite": cal.apercu(d.bgr()) if d is not None else None,
                    "paires": cal.nombre, "minimum": PAIRES_MIN, "conseille": 15,
                    "couverture": cal.couverture(), "mire": [cal.colonnes, cal.lignes, cal.case_m]}
        if nom == "stereo_capture":
            g: ImageCamera | None = self.bus.dernier(S.IMAGE_GAUCHE)
            d: ImageCamera | None = self.bus.dernier(S.IMAGE_DROITE)
            if g is None or d is None or abs(g.t - d.t) > 0.06:
                raise ActionRefusee("pas de paire d'images synchrone")
            trouvee = cal.ajouter_paire(g.bgr(), d.bgr())
            annoncer(self.bus, "stereo", "en_cours",
                     f"mire {'trouvée' if trouvee else 'non trouvée'} ({cal.nombre} paires)",
                     {"paires": cal.nombre})
            return cal.nombre
        try:
            resultat = cal.calculer()
            cal.enregistrer()
        except Exception as erreur:
            annoncer(self.bus, "stereo", "echec", str(erreur))
            raise ActionRefusee(f"calibration stéréo : {erreur}") from None
        if r.perception is not None:
            r.perception.recharger()
        r._projections.clear()
        annoncer(self.bus, "stereo", "termine", "calibration enregistrée", _jsonable(resultat))
        return _jsonable(resultat)

    # ------------------------------------------------------------------ pont ROS 2

    async def _ws_ros(self, requete: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20.0, max_msg_size=1 << 20)
        await ws.prepare(requete)
        ws["images"] = False
        self.ponts_ros.add(ws)
        self.bus.journaliser("ros2", "pont ROS 2 connecté", "succes")
        assert self.boucle is not None
        imu_q, desabonner_imu = self.bus.file_async(S.IMU, self.boucle, 200)
        scan_q, desabonner_scan = self.bus.file_async(S.SCAN, self.boucle, 4)
        envoi = asyncio.create_task(self._envoyer_ros(ws, imu_q, scan_q))
        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    continue
                try:
                    m = json.loads(message.data)
                    self._recevoir_ros(ws, m)
                except (ValueError, KeyError, TypeError, ActionRefusee) as erreur:
                    self.bus.journaliser("ros2", f"message refusé : {erreur}", "alerte")
        finally:
            envoi.cancel()
            desabonner_imu()
            desabonner_scan()
            self.ponts_ros.discard(ws)
            self.bus.journaliser("ros2", "pont ROS 2 déconnecté", "alerte")
        return ws

    def _recevoir_ros(self, ws: web.WebSocketResponse, m: dict[str, Any]) -> None:
        r = self.rover
        if m.get("type") == "cmd_vel":
            v = float(m.get("vitesse", 0.0))
            w = float(m.get("vitesse_lacet", 0.0))
            if not (math.isfinite(v) and math.isfinite(w)):
                return
            braquage = math.atan(r.profil.geometrie.empattement_m * w / v) if abs(v) > 0.03 else 0.0
            r.conduite.proposer(Consigne(t=time.monotonic(), vitesse=v, braquage=braquage,
                                         source="ros2"))
        elif m.get("type") == "but":
            r.executeur.ajouter([{"type": "aller_a", "x": m["x"], "y": m["y"], "repere": "odom"}],
                                remplacer=True, origine="ros2")
        elif m.get("type") == "abonner":
            ws["images"] = bool(m.get("images"))

    async def _envoyer_ros(self, ws: web.WebSocketResponse, imu_q: asyncio.Queue[Any],
                           scan_q: asyncio.Queue[Any]) -> None:
        with contextlib.suppress(ConnectionResetError, asyncio.CancelledError, RuntimeError):
            while not ws.closed:
                await asyncio.sleep(0.02)
                while not scan_q.empty():
                    scan: Scan = scan_q.get_nowait()
                    await ws.send_str(json.dumps({
                        "type": "scan", "t": scan.t, "t_envoi": time.monotonic(),
                        "duree": scan.duree,
                        "angles_b64": base64.b64encode(
                            scan.angles.astype("<f4").tobytes()).decode(),
                        "distances_b64": base64.b64encode(
                            scan.distances.astype("<f4").tobytes()).decode()}))
                while not imu_q.empty():
                    imu: Imu = imu_q.get_nowait()
                    await ws.send_str(json.dumps({
                        "type": "imu", "t": imu.t, "t_envoi": time.monotonic(),
                        "accel_g": list(imu.accel_g), "gyro_rad_s": list(imu.gyro_rad_s)}))

    async def _diffuser_ros(self, tour: int) -> None:
        odom: Odometrie | None = self.bus.dernier(S.ODOM)
        messages: list[dict[str, Any]] = []
        maintenant = time.monotonic()
        if odom is not None:
            cov = odom.covariance if odom.covariance is not None else np.diag([0.01, 0.01, 0.005])
            messages.append({"type": "odom", "t": odom.t, "t_envoi": maintenant,
                             "x": odom.pose.x, "y": odom.pose.y, "cap": odom.pose.cap,
                             "vitesse": odom.vitesse, "vitesse_lacet": odom.vitesse_lacet,
                             "covariance": np.asarray(cov, float).ravel().tolist()})
        if tour % 10 == 0:
            gnss = self.bus.dernier(S.GNSS)
            if gnss is not None:
                messages.append({"type": "gnss", "t": gnss.t, "t_envoi": maintenant,
                                 "fix": gnss.fix, "latitude": gnss.latitude,
                                 "longitude": gnss.longitude, "altitude_m": gnss.altitude_m,
                                 "hdop": gnss.hdop})
            etat = D.message_etat(self.rover)
            messages.append({"type": "etat", "t_envoi": maintenant,
                             **{k: etat[k] for k in ("mode", "arme", "arret_urgence", "securite",
                                                     "mission")}})
        images = []
        if tour % 2 == 0:
            for camera, sujet in (("gauche", S.IMAGE_GAUCHE), ("droite", S.IMAGE_DROITE)):
                image: ImageCamera | None = self.bus.dernier(sujet)
                if image is not None and self.bus.age(sujet) < 0.5:
                    images.append({"type": "image", "t": image.t, "t_envoi": maintenant,
                                   "camera": camera,
                                   "jpeg_b64": base64.b64encode(image.jpeg).decode()})
        for ws in list(self.ponts_ros):
            with contextlib.suppress(ConnectionResetError, RuntimeError):
                for message in messages + (images if ws.get("images") else []):
                    await ws.send_str(json.dumps(message))


def _reduire(image: ImageCamera, largeur: int) -> bytes:
    facteur = 2 if image.largeur / largeur < 3 else 4
    bgr = image.bgr(facteur)
    if bgr.shape[1] > largeur:
        bgr = cv2.resize(bgr, (largeur, int(bgr.shape[0] * largeur / bgr.shape[1])),
                         interpolation=cv2.INTER_AREA)
    ok, jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return jpeg.tobytes() if ok else image.jpeg


def _jsonable(valeur: Any) -> Any:
    if isinstance(valeur, dict):
        return {str(k): _jsonable(v) for k, v in valeur.items()}
    if isinstance(valeur, (list, tuple)):
        return [_jsonable(v) for v in valeur]
    if isinstance(valeur, np.ndarray):
        return valeur.round(5).tolist()
    if isinstance(valeur, (np.floating, np.integer)):
        return valeur.item()
    return valeur


def _profil_publie(profil: Any) -> dict[str, Any]:
    """Profil envoyé au poste, avec le braquage maximal de la roue virtuelle centrale."""
    d = profil.dict()
    d.setdefault("geometrie", {})["braquage_max_central_deg"] = round(
        math.degrees(braquage_max(profil)), 2)
    return d
