"""Clients HTTP des modèles : API compatible OpenAI (Gemini, vLLM) et API native Gemini.

Appels directs avec aiohttp (pas de SDK) :
- `ClientOpenAI.chat` : POST {url}/chat/completions — texte, images (data URL JPEG), audio
  (`input_audio`), appel de fonctions (`tools`), sortie JSON (`response_format`) ;
- `ClientOpenAI.transcrire` : POST {url}/audio/transcriptions (multipart, Whisper sur vLLM) ;
- `ClientGeminiNatif.transcrire` : POST {url_natif}models/{modele}:generateContent avec l'audio
  en `inlineData` et `generationConfig.audioTranscriptionConfig` (Gemini 3.5 Transcribe, qui n'est
  documenté que sur l'API native).

Chaque requête : délai total, reprises limitées sur erreur passagère (réseau, 5xx, 429 avec une
attente courte ; jamais sur délai dépassé, pour borner l'attente de l'opérateur), quota par rôle
(compté à chaque tentative), erreur lisible en français (`ErreurIA`).
"""

from __future__ import annotations

import asyncio
import base64
import collections
import datetime as dt
import json
import logging
import re
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import aiohttp

if TYPE_CHECKING:
    from fieldmind.ia.config import ConfigIA, ConfigRole, ProfilIA, Role

journal = logging.getLogger("fieldmind.ia")

ATTENTE_429_MAX_S = 5.0
"""Au-delà, une limite du fournisseur n'est pas attendue : l'erreur remonte tout de suite."""


class ErreurIA(Exception):
    """Le modèle n'a pas donné de réponse exploitable. Message lisible pour l'opérateur."""

    def __init__(self, message: str, *, passager: bool = False,
                 reessayer_dans_s: float | None = None, statut: int | None = None) -> None:
        super().__init__(message)
        self.passager = passager
        self.reessayer_dans_s = reessayer_dans_s
        self.statut = statut


class IAIndisponible(ErreurIA):
    """Pas de modèle utilisable : profil « aucun », clé absente, rôle non configuré."""


class QuotaAtteint(ErreurIA):
    """Plafond de requêtes du rover atteint (par heure ou par jour)."""


# ------------------------------------------------------------------ quotas


class Quota:
    """Plafonds de requêtes : fenêtre glissante d'une heure et jour calendaire (heure locale)."""

    def __init__(self, par_heure: int, par_jour: int,
                 horloge: Callable[[], float] = time.time) -> None:
        self.par_heure = par_heure
        self.par_jour = par_jour
        self._horloge = horloge
        self._heure: collections.deque[float] = collections.deque()
        self._jour = dt.date.fromtimestamp(horloge())
        self._compte_jour = 0

    def _mettre_a_jour(self, maintenant: float) -> None:
        while self._heure and maintenant - self._heure[0] >= 3600.0:
            self._heure.popleft()
        jour = dt.date.fromtimestamp(maintenant)
        if jour != self._jour:
            self._jour, self._compte_jour = jour, 0

    def prendre(self) -> None:
        """Compte une requête ; lève QuotaAtteint si un plafond est atteint."""
        maintenant = self._horloge()
        self._mettre_a_jour(maintenant)
        if self._compte_jour >= self.par_jour:
            raise QuotaAtteint(f"quota journalier atteint ({self.par_jour} requêtes)")
        if len(self._heure) >= self.par_heure:
            attente = 3600.0 - (maintenant - self._heure[0])
            raise QuotaAtteint(
                f"quota horaire atteint ({self.par_heure} requêtes), réessayer dans "
                f"{max(1, round(attente / 60))} min", reessayer_dans_s=attente)
        self._heure.append(maintenant)
        self._compte_jour += 1

    def etat(self) -> dict[str, int]:
        self._mettre_a_jour(self._horloge())
        return {"heure": len(self._heure), "par_heure": self.par_heure,
                "jour": self._compte_jour, "par_jour": self.par_jour}


# ------------------------------------------------------------------ réponses


@dataclass(slots=True)
class AppelFonction:
    """Appel de fonction proposé par le modèle (arguments None si le JSON est illisible)."""

    nom: str
    arguments: dict[str, Any] | None
    id: str = ""
    brut: str = ""


@dataclass(slots=True)
class ReponseModele:
    texte: str
    appels: list[AppelFonction] = field(default_factory=list)
    modele: str = ""
    duree_s: float = 0.0
    fin: str | None = None


def url_donnees(octets: bytes, type_mime: str = "image/jpeg") -> str:
    return f"data:{type_mime};base64,{base64.b64encode(octets).decode('ascii')}"


def extraire_json(texte: str) -> Any:
    """Objet ou tableau JSON d'une réponse de modèle, même entouré de texte ou d'un bloc ```json."""
    texte = texte.strip()
    bloc = re.search(r"```(?:json)?\s*(.*?)```", texte, re.DOTALL)
    if bloc:
        texte = bloc.group(1).strip()
    try:
        return json.loads(texte)
    except json.JSONDecodeError:
        pass
    debuts = [i for i in (texte.find("["), texte.find("{")) if i >= 0]
    if not debuts:
        raise ValueError("pas de JSON dans la réponse")
    debut = min(debuts)
    fin = texte.rfind("]" if texte[debut] == "[" else "}")
    if fin <= debut:
        raise ValueError("JSON incomplet dans la réponse")
    return json.loads(texte[debut : fin + 1])


def _message_erreur(corps: Any) -> str:
    """Message d'erreur du fournisseur : {"error": {...}} (OpenAI, vLLM) ou [{"error": ...}]."""
    if isinstance(corps, list) and corps:
        corps = corps[0]
    if isinstance(corps, dict):
        erreur = corps.get("error", corps)
        if isinstance(erreur, dict):
            return str(erreur.get("message") or erreur.get("status") or "")[:200]
        if isinstance(erreur, str):
            return erreur[:200]
    return ""


def _attente_429(retry_after: str | None, corps: Any) -> float | None:
    """Attente demandée : en-tête Retry-After, ou `retryDelay` (« 35s ») des détails Gemini."""
    if retry_after:
        try:
            return max(0.0, float(retry_after))
        except ValueError:
            pass
    if isinstance(corps, list) and corps:
        corps = corps[0]
    erreur = corps.get("error") if isinstance(corps, dict) else None
    for detail in (erreur.get("details") or []) if isinstance(erreur, dict) else []:
        delai = detail.get("retryDelay") if isinstance(detail, dict) else None
        if isinstance(delai, str) and delai.endswith("s"):
            try:
                return max(0.0, float(delai[:-1]))
            except ValueError:
                pass
    return None


# ------------------------------------------------------------------ base HTTP


class _ClientHttp:
    def __init__(self, nom: str, reprises: int = 1, *, proxy: str | None = None) -> None:
        self.nom = nom
        self.reprises = reprises
        self.proxy = proxy
        self._session: aiohttp.ClientSession | None = None
        self._boucle: asyncio.AbstractEventLoop | None = None

    def _session_courante(self) -> aiohttp.ClientSession:
        boucle = asyncio.get_running_loop()
        if self._session is None or self._session.closed or self._boucle is not boucle:
            self._session = aiohttp.ClientSession()
            self._boucle = boucle
        return self._session

    async def fermer(self) -> None:
        if (self._session is not None and not self._session.closed
                and self._boucle is asyncio.get_running_loop()):
            await self._session.close()
        self._session = None

    async def _envoyer(self, url: str, *, entetes: dict[str, str], delai_s: float,
                       quota: Quota | None, json_corps: Any = None,
                       formulaire: Callable[[], aiohttp.FormData] | None = None) -> Any:
        """POST avec reprises ; renvoie le JSON décodé. Lève ErreurIA."""
        tentatives = 1 + max(0, self.reprises)
        derniere: ErreurIA | None = None
        for i in range(tentatives):
            if quota is not None:
                quota.prendre()
            try:
                return await self._une_requete(url, entetes, delai_s, json_corps, formulaire)
            except ErreurIA as erreur:
                derniere = erreur
                attente = erreur.reessayer_dans_s
                if (not erreur.passager or i == tentatives - 1 or erreur.statut == 0
                        or (attente is not None and attente > ATTENTE_429_MAX_S)):
                    raise
                pause = attente if attente is not None else 0.5 * (2 ** i)
                journal.info("%s : %s, nouvelle tentative dans %.1f s", self.nom, erreur, pause)
                await asyncio.sleep(pause)
        raise derniere or ErreurIA("aucune tentative")

    async def _une_requete(self, url: str, entetes: dict[str, str], delai_s: float,
                           json_corps: Any,
                           formulaire: Callable[[], aiohttp.FormData] | None) -> Any:
        session = self._session_courante()
        donnees = formulaire() if formulaire is not None else None
        try:
            async with session.post(url, json=json_corps if donnees is None else None,
                                    data=donnees, headers=entetes, proxy=self.proxy,
                                    timeout=aiohttp.ClientTimeout(total=delai_s)) as reponse:
                texte = await reponse.text()
                statut = reponse.status
                retry_after = reponse.headers.get("Retry-After")
        except TimeoutError as erreur:
            # statut 0 : pas de reprise (l'opérateur attend déjà depuis delai_s)
            raise ErreurIA(f"{self.nom} : délai dépassé ({delai_s:.0f} s)", passager=True,
                           statut=0) from erreur
        except aiohttp.ClientError as erreur:
            raise ErreurIA(f"{self.nom} injoignable ({type(erreur).__name__})",
                           passager=True) from erreur
        try:
            corps = json.loads(texte) if texte else None
        except json.JSONDecodeError:
            corps = None
        if statut >= 400:
            raise self._erreur_http(statut, corps, retry_after)
        if corps is None:
            raise ErreurIA(f"{self.nom} : réponse illisible (pas du JSON)")
        if isinstance(corps, dict) and "error" in corps and "choices" not in corps:
            raise ErreurIA(f"{self.nom} : {_message_erreur(corps) or 'erreur du fournisseur'}",
                           passager=True)
        return corps

    def _erreur_http(self, statut: int, corps: Any, retry_after: str | None) -> ErreurIA:
        detail = _message_erreur(corps)
        suffixe = f" : {detail}" if detail else ""
        if statut == 429:
            attente = _attente_429(retry_after, corps)
            return ErreurIA(f"{self.nom} : limite de requêtes du fournisseur atteinte{suffixe}",
                            passager=True, reessayer_dans_s=60.0 if attente is None else attente,
                            statut=statut)
        if statut in (401, 403):
            return ErreurIA(f"{self.nom} : clé d'API refusée ({statut}){suffixe}", statut=statut)
        if statut == 404:
            return ErreurIA(f"{self.nom} : modèle ou adresse introuvable (404){suffixe}",
                            statut=statut)
        if statut >= 500:
            return ErreurIA(f"{self.nom} : fournisseur en panne ({statut}){suffixe}",
                            passager=True, statut=statut)
        return ErreurIA(f"{self.nom} : requête refusée ({statut}){suffixe}", statut=statut)


# ------------------------------------------------------------------ API compatible OpenAI


class ClientOpenAI(_ClientHttp):
    """Serveur compatible OpenAI : Gemini (`api="gemini"`), vLLM ou autre (`api="openai"`)."""

    def __init__(self, url: str, cle: str | None, *, api: str = "openai", reprises: int = 1,
                 nom: str | None = None, proxy: str | None = None) -> None:
        super().__init__(nom or ("Gemini" if api == "gemini" else "serveur de modèles"),
                         reprises, proxy=proxy)
        self.url = url.rstrip("/")
        self.api = api
        self._entetes = {"Authorization": f"Bearer {cle}"} if cle else {}

    def _corps_chat(self, modele: str, messages: list[dict[str, Any]], *,
                    outils: list[dict[str, Any]] | None, choix_outil: str | None,
                    schema_json: dict[str, Any] | None, objet_json: bool,
                    reflexion: str | None, jetons_max: int | None,
                    temperature: float | None) -> dict[str, Any]:
        corps: dict[str, Any] = {"model": modele, "messages": messages}
        if jetons_max:
            corps["max_tokens"] = jetons_max
        if temperature is not None:
            corps["temperature"] = temperature
        if outils:
            corps["tools"] = outils
            corps["tool_choice"] = choix_outil or "auto"
        if schema_json is not None:
            corps["response_format"] = {"type": "json_schema", "json_schema": {
                "name": "reponse", "schema": schema_json}}
        elif objet_json:
            corps["response_format"] = {"type": "json_object"}
        if reflexion:
            if self.api == "gemini":
                # Gemini 3 et suivants ne coupent pas la réflexion : « aucune » → la plus courte.
                corps["reasoning_effort"] = "low" if reflexion == "aucune" else reflexion
            else:
                # vLLM (Gemma 4, Qwen) : réflexion activée ou non par le gabarit de conversation.
                corps["chat_template_kwargs"] = {"enable_thinking": reflexion != "aucune"}
        return corps

    async def chat(self, modele: str, messages: list[dict[str, Any]], *,
                   outils: list[dict[str, Any]] | None = None, choix_outil: str | None = None,
                   schema_json: dict[str, Any] | None = None, objet_json: bool = False,
                   reflexion: str | None = None, jetons_max: int | None = None,
                   temperature: float | None = None, delai_s: float = 30.0,
                   quota: Quota | None = None) -> ReponseModele:
        corps = self._corps_chat(modele, messages, outils=outils, choix_outil=choix_outil,
                                 schema_json=schema_json, objet_json=objet_json,
                                 reflexion=reflexion, jetons_max=jetons_max,
                                 temperature=temperature)
        debut = time.monotonic()
        url = f"{self.url}/chat/completions"
        try:
            donnees = await self._envoyer(url, entetes=self._entetes, delai_s=delai_s,
                                          quota=quota, json_corps=corps)
        except ErreurIA as erreur:
            # Un modèle qui refuse le réglage de réflexion : une seule nouvelle tentative sans.
            if erreur.statut != 400 or not ({"reasoning_effort", "chat_template_kwargs"} & set(
                    corps)):
                raise
            corps.pop("reasoning_effort", None)
            corps.pop("chat_template_kwargs", None)
            donnees = await self._envoyer(url, entetes=self._entetes, delai_s=delai_s,
                                          quota=quota, json_corps=corps)
        return self._lire_chat(donnees, modele, time.monotonic() - debut)

    def _lire_chat(self, donnees: Any, modele: str, duree_s: float) -> ReponseModele:
        try:
            choix = donnees["choices"][0]
            message = choix.get("message") or {}
        except (KeyError, IndexError, TypeError) as erreur:
            raise ErreurIA(f"{self.nom} : réponse sans choix") from erreur
        texte = message.get("content") or ""
        if isinstance(texte, list):  # contenu en parties (certains serveurs)
            texte = "".join(p.get("text", "") for p in texte if isinstance(p, dict))
        appels: list[AppelFonction] = []
        for brut in message.get("tool_calls") or []:
            fonction = brut.get("function") or {}
            nom = str(fonction.get("name") or "")
            arguments_bruts = fonction.get("arguments")
            arguments: dict[str, Any] | None
            if isinstance(arguments_bruts, dict):
                arguments = arguments_bruts
            else:
                try:
                    lu = json.loads(arguments_bruts or "{}")
                    arguments = lu if isinstance(lu, dict) else None
                except (TypeError, json.JSONDecodeError):
                    arguments = None
            appels.append(AppelFonction(nom, arguments, str(brut.get("id") or ""),
                                        str(arguments_bruts or "")))
        fin = choix.get("finish_reason")
        if not str(texte).strip() and not appels:
            if fin == "length":
                raise ErreurIA(f"{self.nom} : réponse tronquée (plafond de jetons)")
            raise ErreurIA(f"{self.nom} : réponse vide du modèle")
        return ReponseModele(str(texte).strip(), appels, str(donnees.get("model") or modele),
                             duree_s, fin)

    async def transcrire(self, modele: str, wav: bytes, *, langue: str | None = None,
                         consigne: str | None = None, delai_s: float = 15.0,
                         quota: Quota | None = None) -> str:
        """POST /audio/transcriptions (Whisper servi par vLLM ou serveur compatible)."""

        def formulaire() -> aiohttp.FormData:
            f = aiohttp.FormData()
            f.add_field("file", wav, filename="parole.wav", content_type="audio/wav")
            f.add_field("model", modele)
            f.add_field("response_format", "json")
            f.add_field("temperature", "0")
            if langue:
                f.add_field("language", langue.split("-")[0])
            if consigne:
                f.add_field("prompt", consigne)
            return f

        donnees = await self._envoyer(f"{self.url}/audio/transcriptions", entetes=self._entetes,
                                      delai_s=delai_s, quota=quota, formulaire=formulaire)
        if isinstance(donnees, dict) and isinstance(donnees.get("text"), str):
            return donnees["text"].strip()
        raise ErreurIA(f"{self.nom} : transcription illisible")


# ------------------------------------------------------------------ API native Gemini


class ClientGeminiNatif(_ClientHttp):
    """generateContent de l'API Gemini (seulement pour ce que l'API compatible ne couvre pas)."""

    def __init__(self, url: str, cle: str | None, reprises: int = 1, *,
                 proxy: str | None = None) -> None:
        super().__init__("Gemini", reprises, proxy=proxy)
        self.url = url.rstrip("/")
        self._entetes = {"x-goog-api-key": cle} if cle else {}
        self._config_minimale: set[str] = set()
        """Modèles qui ont refusé les options de transcription (style, vocabulaire)."""

    async def generer(self, modele: str, parties: list[dict[str, Any]],
                      config: dict[str, Any] | None = None, *, delai_s: float = 30.0,
                      quota: Quota | None = None) -> dict[str, Any]:
        corps: dict[str, Any] = {"contents": [{"role": "user", "parts": parties}]}
        if config:
            corps["generationConfig"] = config
        donnees = await self._envoyer(f"{self.url}/models/{modele}:generateContent",
                                      entetes=self._entetes, delai_s=delai_s, quota=quota,
                                      json_corps=corps)
        if not isinstance(donnees, dict):
            raise ErreurIA("Gemini : réponse illisible")
        return donnees

    async def transcrire(self, modele: str, wav: bytes, *, langue: str = "fr-FR",
                         style: str = "", vocabulaire: tuple[str, ...] = (),
                         delai_s: float = 15.0, quota: Quota | None = None) -> str:
        parties = [{"inlineData": {"mimeType": "audio/wav",
                                   "data": base64.b64encode(wav).decode("ascii")}}]
        options: dict[str, Any] = {"languageCodes": [langue] if langue else []}
        complet = dict(options)
        if vocabulaire:
            complet["customVocabulary"] = list(vocabulaire)
        if style:
            complet["mode"] = style
        essais = [options] if (modele in self._config_minimale or complet == options) else [
            complet, options]
        for i, config in enumerate(essais):
            try:
                donnees = await self.generer(modele, parties, {"audioTranscriptionConfig": config},
                                             delai_s=delai_s, quota=quota)
                break
            except ErreurIA as erreur:
                if erreur.statut != 400 or i == len(essais) - 1:
                    raise
                journal.warning("transcription Gemini : options refusées (%s), sans options",
                                erreur)
                self._config_minimale.add(modele)
        return lire_transcription_gemini(donnees)


def lire_transcription_gemini(donnees: dict[str, Any]) -> str:
    """Texte d'une réponse generateContent : parties `text`, sinon `audioTranscription`."""
    try:
        candidat = (donnees.get("candidates") or [])[0]
    except IndexError:
        retour = donnees.get("promptFeedback", {}).get("blockReason")
        if retour:
            raise ErreurIA(f"Gemini : audio refusé ({retour})") from None
        return ""
    textes: list[str] = []
    for partie in (candidat.get("content") or {}).get("parts") or []:
        if not isinstance(partie, dict):
            continue
        if isinstance(partie.get("text"), str):
            textes.append(partie["text"])
            continue
        transcription = partie.get("audioTranscription")
        if isinstance(transcription, dict):
            for cle in ("text", "transcript"):
                if isinstance(transcription.get(cle), str):
                    textes.append(transcription[cle])
                    break
            else:
                mots = [str(m.get("word", "")) for m in transcription.get("words") or []
                        if isinstance(m, dict)]
                textes.append(" ".join(m for m in mots if m))
    texte = " ".join(t.strip() for t in textes if t and t.strip())
    return re.sub(r"\s+([,.;:!?])", r"\1", texte).strip()


# ------------------------------------------------------------------ passerelle (profil actif)


@dataclass(slots=True)
class Acces:
    """Ce qu'il faut pour appeler un rôle du profil actif."""

    profil: ProfilIA
    role: ConfigRole
    client: ClientOpenAI
    natif: ClientGeminiNatif | None
    quota: Quota


class Passerelle:
    """Clients (une session HTTP par adresse) et quotas (par profil et par rôle), partagés par
    l'agent, la vision et la transcription. Suit le profil actif de `ConfigIA`."""

    def __init__(self, config: ConfigIA) -> None:
        self.config = config
        self._clients: dict[tuple[str, str, str | None], ClientOpenAI] = {}
        self._natifs: dict[tuple[str, str | None], ClientGeminiNatif] = {}
        self._quotas: dict[tuple[str, str], Quota] = {}
        self.bascules: dict[str, int] = {}  # passages sur la clé de secours, par rôle

    def acces(self, role: Role, secours: bool = False) -> Acces:
        """Lève IAIndisponible si le profil actif ne permet pas ce rôle. `secours` : avec la
        clé de secours du rôle (même modèle)."""
        profil = self.config.actif
        disponible, raison = self.config.disponibilite(role, profil.nom)
        config_role = profil.role(role)
        if not disponible or config_role is None:
            raise IAIndisponible(raison or "modèle indisponible")
        cle = self.config.cle_role(role, profil.nom)
        if secours or cle is None:
            cle_secours = self.config.cle_role(role, profil.nom, secours=True)
            if cle_secours is None:
                if secours:
                    raise IAIndisponible(f"pas de clé de secours pour le rôle {role}")
            else:
                cle = cle_secours
        cle_client = (profil.nom, config_role.url, cle)
        client = self._clients.get(cle_client)
        if client is None:
            client = ClientOpenAI(config_role.url, cle, api=profil.api, reprises=profil.reprises,
                                  nom="Gemini" if profil.api == "gemini" else
                                  f"serveur {profil.nom}",
                                  proxy=self.config.proxy_gemini
                                  if profil.api == "gemini" else None)
            self._clients[cle_client] = client
        natif = None
        if profil.api == "gemini" and profil.url_natif:
            natif = self._natifs.get((profil.url_natif, cle))
            if natif is None:
                natif = ClientGeminiNatif(profil.url_natif, cle, profil.reprises,
                                         proxy=self.config.proxy_gemini)
                self._natifs[(profil.url_natif, cle)] = natif
        quota = self._quotas.get((profil.nom, role))
        if quota is None:
            quota = Quota(config_role.requetes_par_heure, config_role.requetes_par_jour)
            self._quotas[(profil.nom, role)] = quota
        return Acces(profil, config_role, client, natif, quota)

    async def avec_secours(self, role: Role,
                           appel: Callable[[Acces], Awaitable[Any]]) -> Any:
        """Appel avec la clé principale (payante) ; sur quota dépassé (429) ou panne passagère
        (5xx, réseau, délai), même appel avec la clé de secours (gratuite) du rôle, s'il en a une.
        Le modèle ne change pas ; un rôle sans clé de secours (agent) ne bascule jamais."""
        principal = self.acces(role)
        try:
            return await appel(principal)
        except QuotaAtteint:
            raise  # plafond du rover lui-même : la clé n'y est pour rien
        except ErreurIA as erreur:
            if not (erreur.passager or erreur.statut == 429):
                raise
            if self.config.cle_role(role, secours=True) is None or \
                    self.config.cle_role(role) is None:
                raise
            journal.warning("%s : %s ; nouvel essai avec la clé de secours", role, erreur)
            self.bascules[role] = self.bascules.get(role, 0) + 1
            return await appel(self.acces(role, secours=True))

    async def chat(self, role: Role, messages: list[dict[str, Any]], **options: Any
                   ) -> ReponseModele:
        """Chat du rôle (modèle, réflexion, délai, plafond de jetons et quota du profil), avec la
        clé de secours si besoin."""
        async def appel(a: Acces) -> ReponseModele:
            return await a.client.chat(
                a.role.modele, messages, reflexion=a.role.reflexion, jetons_max=a.role.jetons_max,
                temperature=a.role.temperature, delai_s=a.role.delai_s, quota=a.quota, **options)

        return await self.avec_secours(role, appel)

    def quotas(self) -> dict[str, dict[str, int]]:
        return {f"{p}.{r}": q.etat() for (p, r), q in self._quotas.items()}

    async def fermer(self) -> None:
        for client in [*self._clients.values(), *self._natifs.values()]:
            await client.fermer()
