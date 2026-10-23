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
import contextlib
import json
import logging
import math
import time
from pathlib import Path
from typing import Any

from fieldmind.bus import Bus, S
from fieldmind.ia.decisions import contexte_borne
from fieldmind.ia.fournisseurs import ErreurIA
from fieldmind.ia.inference import ClientInference, similarite
from fieldmind.ia.vision import preparer_image, recaler_distances, recouvrement

journal = logging.getLogger("fieldmind.ia.distant")

LABELS_FR = {"person": "personne", "dog": "chien", "cat": "chat", "horse": "cheval",
             "cow": "vache", "bird": "oiseau", "sheep": "mouton", "bicycle": "vélo",
             "car": "voiture", "motorcycle": "moto", "bench": "banc", "chair": "chaise",
             "potted plant": "plante", "backpack": "sac à dos", "ball": "ballon",
             "sports ball": "ballon", "bottle": "bouteille", "cup": "tasse",
             "diningtable": "table", "dining table": "table", "sofa": "canapé",
             "couch": "canapé", "tvmonitor": "télévision", "tv": "télévision",
             "refrigerator": "réfrigérateur", "pottedplant": "plante"}
SEUIL_VIVANT = 0.6  # personne ou animal vu une seule fois : retenu seulement au-dessus
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
        self.echelle: float | None = None  # profondeur métrique → mètres réels
        self._vivants_precedents: list[list[float]] = []
        self._derniere_image: tuple[str, float, int] | None = None
        self._verrou = asyncio.Lock()

    def active(self) -> bool:
        return bool(self.profil.valeur("ia.detection_continue", True)) and \
            self.client.disponible("detection")

    async def cycle(self, camera: str = "gauche") -> dict[str, Any] | None:
        # Pas d'accumulation si un appel explicite arrive pendant le cycle automatique.
        if self._verrou.locked():
            return None
        async with self._verrou:
            return await self._cycle(camera)

    async def _cycle(self, camera: str) -> dict[str, Any] | None:
        snapshot = getattr(self.outils, "snapshot_image", lambda _c: None)(camera)
        outils = snapshot if snapshot is not None else self.outils
        image = snapshot.image if snapshot is not None else self.outils.derniere_image(camera)
        if image is None:
            return None
        if not 0 <= time.monotonic() - image.t <= 5.0:
            return None
        identite = (image.camera, image.t, image.numero)
        if identite == self._derniere_image:
            return None
        pose = getattr(snapshot, "pose", None)
        profil = self.client.config.actif
        jpeg, _, _ = await asyncio.to_thread(preparer_image, image, LARGEUR_DETECTION)
        avec_profondeur = self.client.disponible("profondeur")
        reponse = await self.client.analyser(
            jpeg, taches=("detection", "profondeur") if avec_profondeur else ("detection",),
            seuil=float(self.profil.valeur("ia.detection_seuil", 0.45)))
        if profil is not self.client.config.actif or time.monotonic() - image.t > 5.0:
            return None
        objets = []
        for det in reponse.get("detections") or []:
            label = LABELS_FR.get(det["label"], det["label"])
            b = det["boite"]
            try:
                sol = outils.pixel_vers_sol(camera, (b[0] + b[2]) / 2, b[3])
            except Exception:
                sol = None
            objets.append({"label": label, "score": det["score"], "boite": b,
                           "distance_m": det.get("distance_m"), "vivant": label in VIVANTS,
                           "sol": [round(float(sol[0]), 2), round(float(sol[1]), 2)]
                           if sol is not None else None})
        # Personne ou animal : retenu si le score est franc, ou s'il est revu au même endroit au
        # tour suivant ; une ombre ou une image floue ne déclenche plus d'alerte d'un coup
        vus = [o["boite"] for o in objets if o["vivant"]]
        objets = [o for o in objets if not o["vivant"] or o["score"] >= SEUIL_VIVANT
                  or any(recouvrement(o["boite"], b) > 0.3 for b in self._vivants_precedents)]
        self._vivants_precedents = vus
        # Profondeur recalée sur les objets placés au sol, puis position des autres par elle
        self.echelle = recaler_distances(objets, float(self.profil.valeur("cameras.x_m", 0.415)),
                                         self.echelle)
        for o in objets:
            if o["sol"] is None and o.get("distance_m"):
                o["sol"] = position_au_sol(outils, self.profil, camera, o["boite"],
                                           o["distance_m"])
        message = {"t": time.time(), "camera": camera, "source": self.client.modele("detection"),
                   "image_t": image.t, "pose_image": None if pose is None else {
                       "x": pose.x, "y": pose.y, "cap": pose.cap},
                   "objets": objets, "durees_ms": reponse.get("durees_ms", {})}
        self._derniere_image = identite
        self.dernier = message
        self.bus.publier(S.DETECTIONS, message)
        return message

    async def boucle(self) -> None:
        await asyncio.sleep(2.0)
        while True:
            periode = float(self.profil.valeur("ia.detection_periode_s", 2.5))
            try:
                if self.active():
                    image = self.outils.derniere_image("gauche")
                    if image is not None and 0 <= time.monotonic() - image.t <= 5.0:
                        async with asyncio.timeout(12.0):
                            await self.cycle()
                    if self.echecs:
                        self.bus.journaliser("ia", "détection continue rétablie", "succes")
                    self.echecs = 0
            except asyncio.CancelledError:
                raise
            except (ErreurIA, TimeoutError) as erreur:
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
        self._file: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=1)
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
        # Une phrase à la fois, la plus récente : une réponse en retard ne sert plus à rien
        if self._file.full():
            self._file.get_nowait()
        self._file.put_nowait(message)

    async def boucle(self) -> None:
        self._boucle_asyncio = asyncio.get_running_loop()
        while True:
            message = await self._file.get()
            try:
                # Phrase par phrase : la première est dite en 1–2 s pendant que la suite se
                # calcule (la synthèse va à peu près en temps réel)
                parties = decouper_phrases(str(message["texte"]))
                for i, partie in enumerate(parties):
                    if not self._file.empty():
                        break  # une réponse plus récente attend : on passe à elle
                    debut = time.monotonic()
                    wav = await self.client.parole(partie)
                    self.bus.publier(S.PAROLE, {
                        "type": "parole", "id": message.get("id"), "partie": i,
                        "parties": len(parties), "t": time.time(), "texte": partie,
                        "audio": base64.b64encode(wav).decode("ascii"), "format": "wav",
                        "modele": self.client.modele("synthese"),
                        "duree_s": round(time.monotonic() - debut, 2)})
            except asyncio.CancelledError:
                raise
            except ErreurIA as erreur:
                journal.info("voix du rover indisponible : %s", erreur)
            except Exception:
                journal.exception("voix du rover")


def decouper_phrases(texte: str, longueur: int = 140, total: int = 420) -> list[str]:
    """Texte → phrases d'au plus `longueur` caractères (coupées aux virgules si besoin)."""
    import re

    texte = " ".join(texte.split())[:total]
    phrases: list[str] = []
    for phrase in re.split(r"(?<=[.!?…])\s+", texte):
        while len(phrase) > longueur:
            coupe = phrase.rfind(", ", 0, longueur)
            coupe = coupe + 1 if coupe > 40 else phrase.rfind(" ", 0, longueur)
            phrases.append(phrase[:coupe].strip())
            phrase = phrase[coupe:].strip()
        if phrase:
            phrases.append(phrase)
    return phrases


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


class TunnelGx10:
    """Choisit la route vers le GX10 : son tunnel Cloudflare ou l'adresse de base (Tailscale).

    La liaison directe France → Allemagne perd des paquets à l'interconnexion entre opérateurs,
    plus ou moins selon l'heure ; le détour par Cloudflare est plus régulier. Toutes les
    `COMPARAISON_S`, un envoi de 64 ko (la taille d'une image) part par chaque route et la plus
    rapide est gardée ; entre deux, une petite requête vérifie chaque minute que la route
    retenue répond, sinon on passe sur l'autre. L'adresse du tunnel change à chaque redémarrage
    du GX10 : elle est relue sur `url_tunnel` (passerelle, par l'adresse de base)."""

    PERIODE_S = 60.0
    COMPARAISON_S = 600.0
    TAILLE_ESSAI = 48_000  # octets avant base64 : ≈ 64 ko envoyés

    def __init__(self, bus: Bus, config: Any) -> None:
        self.bus, self.config = bus, config
        self.mesures: dict[str, float] = {}
        self._derniere_comparaison = -math.inf
        self._reveil: asyncio.Event | None = None
        self._boucle_asyncio: asyncio.AbstractEventLoop | None = None
        config.sur_changement(self._profil_change)

    def _profil_change(self, _profil: Any) -> None:
        """Nouveau profil : choisir sa route tout de suite, pas à la minute suivante."""
        if self._reveil is not None and self._boucle_asyncio is not None:
            self._boucle_asyncio.call_soon_threadsafe(self._reveil.set)

    async def _lire(self, session: Any, url: str) -> Any:
        import aiohttp

        async with session.get(url, timeout=aiohttp.ClientTimeout(total=8)) as reponse:
            if reponse.status != 200:
                raise ErreurIA(f"HTTP {reponse.status}")
            return await reponse.json(content_type=None)

    async def _envoi(self, session: Any, base: str) -> float:
        """Durée d'un envoi de 64 ko au serveur des outils (réponse 400 attendue : image
        factice), ou l'infini si la route ne répond pas."""
        import os

        import aiohttp

        cle = self.config.cle_role("detection") or self.config.cle()
        corps = {"image": base64.b64encode(os.urandom(self.TAILLE_ESSAI)).decode("ascii")}
        debut = time.monotonic()
        try:
            async with session.post(f"{base}/outils/v1/analyse", json=corps,
                                    headers={"Authorization": f"Bearer {cle}"} if cle else {},
                                    timeout=aiohttp.ClientTimeout(total=20)) as reponse:
                await reponse.read()
                if reponse.status >= 500:
                    return math.inf
        except Exception:
            return math.inf
        return time.monotonic() - debut

    async def _tunnel(self, session: Any, profil: Any) -> str | None:
        try:
            hote = (await self._lire(session, profil.url_tunnel)).get("hostname")
            if not hote:
                return None
            candidat = hote if "://" in hote else f"https://{hote}"
            await self._lire(session, f"{candidat}/sante")
            return str(candidat)
        except Exception as erreur:
            journal.info("tunnel du GX10 indisponible : %r", erreur)
            if profil.hote != profil.hote_base:  # l'ancien tunnel répond-il encore ?
                try:
                    await self._lire(session, f"{profil.hote}/sante")
                    return str(profil.hote)
                except Exception:
                    pass
            return None

    async def verifier(self, session: Any, comparer: bool | None = None) -> str | None:
        """Un tour : renvoie l'adresse retenue (None si le profil n'a pas de tunnel)."""
        profil = self.config.actif
        if not profil.url_tunnel:
            return None
        tunnel = await self._tunnel(session, profil)
        choix = tunnel
        if comparer is None:
            comparer = time.monotonic() - self._derniere_comparaison > self.COMPARAISON_S
        if tunnel and comparer:
            self._derniere_comparaison = time.monotonic()
            directe, par_tunnel = (await self._envoi(session, profil.hote_base),
                                   await self._envoi(session, tunnel))
            self.mesures = {"directe_s": round(directe, 2), "tunnel_s": round(par_tunnel, 2)}
            journal.info("routes du GX10 : directe %.2f s, tunnel %.2f s", directe, par_tunnel)
            choix = tunnel if par_tunnel <= directe else None
        elif tunnel and profil.hote == profil.hote_base and self.mesures and \
                self.mesures.get("directe_s", math.inf) < self.mesures.get("tunnel_s", math.inf):
            choix = None  # la dernière comparaison a donné la route directe gagnante
        actuel = profil.hote if profil.hote != profil.hote_base else None
        if choix != actuel:
            nouveau = self.config.changer_hote(profil.nom, choix)
            if choix:
                self.bus.journaliser("ia", "GX10 joint par le tunnel Cloudflare "
                                           f"{self._chiffres()}", "succes")
            else:
                self.bus.journaliser("ia", "GX10 joint en direct (Tailscale) "
                                           f"{self._chiffres()}", "info")
            return str(nouveau.hote)
        return str(profil.hote)

    def _chiffres(self) -> str:
        if not self.mesures:
            return ""
        return (f"(envoi de 64 ko : directe {self.mesures['directe_s']} s, tunnel "
                f"{self.mesures['tunnel_s']} s)")

    async def boucle(self) -> None:
        import aiohttp

        self._boucle_asyncio = asyncio.get_running_loop()
        self._reveil = asyncio.Event()
        async with aiohttp.ClientSession() as session:
            while True:
                try:
                    await self.verifier(session)
                except asyncio.CancelledError:
                    raise
                except Exception:
                    journal.exception("choix de la route du GX10")
                self._reveil.clear()
                with contextlib.suppress(TimeoutError):
                    await asyncio.wait_for(self._reveil.wait(), self.PERIODE_S)


CONSIGNE_VEILLE = (
    "Tu es l'agent de veille d'un petit rover terrestre autonome. Voici l'image de sa caméra "
    "avant et ce que mesurent ses capteurs :\n{contexte}\n"
    "Analyse la situation et réponds STRICTEMENT en JSON, en français :\n"
    '{{"resume": "une phrase : où est le rover, ce qui se passe", '
    '"niveau": "rien" | "info" | "attention" | "danger", '
    '"alerte": null ou une phrase courte pour l\'opérateur si quelque chose mérite son attention '
    "(personne ou animal qui approche, enfant, obstacle ou trou sur le chemin, terrain glissant, "
    "porte fermée, objet tombé, fumée, le rover semble coincé…)}}\n"
    "N'invente rien. « danger » seulement si quelqu'un ou le rover risque d'être blessé ou "
    "abîmé. Sans rien de notable : niveau « rien », alerte null."
)


class VeilleSituation:
    """Agent de veille : toutes les `ia.veille_periode_s`, le modèle de vision regarde la
    caméra avec les mesures du rover (mission, sécurité, détections) et résume la situation.
    Le résumé va dans l'état que voit l'agent de conversation (« qu'est-ce qui se passe ? ») et
    dans le poste ; une alerte nouvelle (« attention », « danger ») est dite dans le chat, donc à
    voix haute. Conseil seulement : la sécurité ne dépend pas de lui.

    Gratuit avec un profil local (GX10) : actif par défaut ; avec Gemini, chaque analyse est
    payante : seulement si `ia.vigilance` est activée."""

    def __init__(self, bus: Bus, profil: Any, config: Any, outils: Any,
                 occupee: Any = None) -> None:
        self.bus, self.profil, self.config, self.outils = bus, profil, config, outils
        self._autre_occupee = occupee or (lambda: False)
        self.dernier: dict[str, Any] | None = None
        self._derniere_alerte: tuple[str, float] = ("", 0.0)
        self._verrou = asyncio.Lock()
        self._derniere_image: tuple[str, float, int] | None = None

    @property
    def occupee(self) -> bool:
        return self._verrou.locked()

    def active(self) -> bool:
        if not bool(self.profil.valeur("ia.veille", True)):
            return False
        if (self.config.actif.api == "openai"
                and bool(self.profil.valeur("ia.gx10_cartographie", True))):
            # L'analyse de scène publie déjà SITUATION : pas de seconde requête pour la même vue.
            return False
        disponible, _ = self.config.disponibilite("vision")
        if not disponible:
            return False
        if self.config.actif.api == "gemini":
            return bool(self.profil.valeur("ia.vigilance", False))
        return True

    def _contexte(self) -> str:
        lignes = []
        try:
            etat = self.outils.etat_resume()
        except Exception:
            etat = {}
        mission = etat.get("mission") or {}
        if mission.get("action"):
            lignes.append(f"- mission en cours : {mission.get('action')} ({mission.get('detail')})")
        pose = etat.get("pose_odom") or {}
        if pose:
            lignes.append(f"- vitesse : {pose.get('vitesse_m_s', 0)} m/s, mode {etat.get('mode')}")
        securite = etat.get("securite") or {}
        if securite.get("raisons"):
            lignes.append(f"- sécurité : {', '.join(securite['raisons'])} ; libre devant "
                          f"{securite.get('libre_avant_m')} m")
        detections = self.bus.dernier(S.DETECTIONS)
        if isinstance(detections, dict) and self.bus.age(S.DETECTIONS) < 10:
            vus = [f"{o['label']}" + (f" à {o['distance_m']} m" if o.get("distance_m") else "")
                   for o in detections.get("objets") or [] if o.get("score", 0) > 0.5][:6]
            if vus:
                lignes.append(f"- détecteur : {', '.join(vus)}")
        lignes.append("- carte et contexte : " + json.dumps(
            contexte_borne(etat), ensure_ascii=False, default=str))
        return "\n".join(lignes)

    async def cycle(self, camera: str = "gauche") -> dict[str, Any] | None:
        if self._verrou.locked():
            return None
        async with self._verrou:
            return await self._cycle(camera)

    async def _cycle(self, camera: str) -> dict[str, Any] | None:
        from fieldmind.ia.fournisseurs import extraire_json, url_donnees
        from fieldmind.ia.messages import publier_chat

        image = self.outils.derniere_image(camera)
        if image is None:
            return None
        identite = (image.camera, image.t, image.numero)
        if identite == self._derniere_image:
            return None
        profil = self.config.actif
        passerelle = self.config.passerelle()
        acces = passerelle.acces("vision")
        jpeg, _, _ = await asyncio.to_thread(preparer_image, image, min(512,
                                                                         acces.role.largeur_image))
        messages = [{"role": "user", "content": [
            {"type": "image_url", "image_url": {"url": url_donnees(jpeg, "image/jpeg")}},
            {"type": "text", "text": CONSIGNE_VEILLE.format(contexte=self._contexte())}]}]
        debut = time.monotonic()
        reponse = await passerelle.chat("vision", messages)
        if profil is not self.config.actif:
            return None
        try:
            donnees = extraire_json(reponse.texte)
        except ValueError:
            donnees = {"resume": " ".join(reponse.texte.split())[:200], "niveau": "info"}
        if not isinstance(donnees, dict):
            return None
        niveau = str(donnees.get("niveau") or "rien")
        alerte = donnees.get("alerte") if isinstance(donnees.get("alerte"), str) else None
        situation = {"t": time.time(), "resume": str(donnees.get("resume") or "")[:300],
                     "image_t": image.t,
                     "niveau": niveau if niveau in ("rien", "info", "attention", "danger")
                     else "info", "alerte": alerte, "modele": reponse.modele or acces.role.modele,
                     "duree_s": round(time.monotonic() - debut, 2)}
        self.dernier = situation
        self._derniere_image = identite
        self.bus.publier(S.SITUATION, situation)
        if alerte and situation["niveau"] in ("attention", "danger"):
            texte, quand = self._derniere_alerte
            if alerte != texte or time.monotonic() - quand > 60:
                self._derniere_alerte = (alerte, time.monotonic())
                publier_chat(self.bus, "rover", ("Attention : " if situation["niveau"] ==
                                                 "attention" else "Danger : ") + alerte)
        return situation

    async def boucle(self) -> None:
        await asyncio.sleep(5.0)
        while True:
            periode = float(self.profil.valeur("ia.veille_periode_s", 8.0))
            try:
                if self.active() and not self._autre_occupee():
                    image = self.outils.derniere_image("gauche")
                    if image is not None and 0 <= time.monotonic() - image.t <= 5.0:
                        async with asyncio.timeout(35.0):
                            await self.cycle()
            except asyncio.CancelledError:
                raise
            except (ErreurIA, TimeoutError) as erreur:
                journal.info("veille : %s", erreur)
                periode *= 3
            except Exception:
                journal.exception("veille")
            await asyncio.sleep(periode)
