"""Configuration de l'IA : profils de fournisseurs (config/ia.toml), clés, profil actif.

    config_ia = ConfigIA.charger()
    config_ia.actif.agent.modele            # "gemini-4-argon"
    config_ia.actif.role("detection")        # ConfigRole (fournisseur, modèle, adresse…) ou None
    config_ia.changer_profil("gx10")         # en marche ; choix gardé dans ~/.config/fieldmind
    config_ia.resume()                       # champ `ia` du message `bienvenue`
    config_ia.passerelle()                   # clients HTTP et quotas partagés (fournisseurs.py)

Les clés sont lues dans l'environnement, puis dans `software/rover/.env` (lignes `NOM=valeur`) :
jamais écrites dans un fichier du dépôt ni envoyées au poste.
"""

from __future__ import annotations

import json
import os
import threading
import tomllib
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

from fieldmind.config import DOSSIER_UTILISATEUR, RACINE

if TYPE_CHECKING:
    from fieldmind.ia.fournisseurs import Passerelle

FICHIER_IA = RACINE / "config" / "ia.toml"
FICHIER_ENV = RACINE / ".env"
FICHIER_CHOIX = DOSSIER_UTILISATEUR / "ia.json"
VARIABLE_PROFIL = "FIELDMIND_IA_PROFIL"

Role = Literal["agent", "vision", "pointage", "transcription", "synthese", "detection",
               "profondeur", "parole", "eveil", "memoire"]
ROLES: tuple[Role, ...] = ("agent", "vision", "pointage", "transcription", "synthese",
                           "detection", "profondeur", "parole", "eveil", "memoire")
ROLES_HTTP: tuple[Role, ...] = ("agent", "vision", "transcription")
"""Rôles servis par l'API du profil quand leur fournisseur n'est pas précisé."""
API_CONNUES = ("gemini", "openai", "aucun")
FOURNISSEURS = ("gemini", "openai", "inference", "local", "integre", "aucun")
"""gemini : API Gemini (compatible OpenAI ou native) ; openai : serveur compatible OpenAI (vLLM,
Ollama) ; inference : serveur d'inférence du projet (software/rover/inference) ; local : modèle
ONNX exécuté sur le rover (onnxruntime facultatif) ; integre : code du rover sans modèle ;
aucun : rôle désactivé."""
FOURNISSEURS_SANS_ADRESSE = ("local", "integre", "aucun")
URL_GEMINI = "https://generativelanguage.googleapis.com/v1beta/openai/"
URL_GEMINI_NATIF = "https://generativelanguage.googleapis.com/v1beta/"
VARIABLE_CLE_GEMINI = "GEMINI_API_KEY"
MODES_TRANSCRIPTION = ("gemini_natif", "chat_audio", "openai_transcriptions")
REFLEXIONS = ("aucune", "minimal", "low", "medium", "high")
FORMATS_POINTS = ("yx_1000", "xy_1000", "robobrain")
"""robobrain : RoboBrain 2.5, réponse `[(x, y)]` en 0–1000 à une consigne propre au modèle."""


class ErreurConfigIA(ValueError):
    """Fichier ia.toml incohérent ou profil inconnu."""


@dataclass(frozen=True, slots=True)
class ConfigRole:
    """Un rôle d'un profil : fournisseur, modèle, adresse, délais, quotas, réglages propres."""

    modele: str
    url: str
    fournisseur: str = "openai"
    variable_cle: str | None = None  # None : la clé du profil
    # Clé de secours (gratuite), même modèle, seulement sur quota dépassé ou panne passagère
    variable_cle_secours: str | None = None
    url_natif: str = ""  # Gemini : generateContent, interactions, embedContent
    delai_s: float = 30.0
    reflexion: str | None = None
    jetons_max: int | None = None
    temperature: float | None = None
    requetes_par_heure: int = 600
    requetes_par_jour: int = 5000
    # vision
    format_points: str = "yx_1000"
    largeur_image: int = 640
    # transcription
    mode: str = "openai_transcriptions"
    langue: str = "fr"
    style: str = ""
    vocabulaire: tuple[str, ...] = ()
    # synthèse
    voix: str = ""
    format_audio: str = "wav"
    # détection, profondeur, parole, éveil (modèles locaux)
    frequence_hz: float = 0.0
    seuil: float = 0.5
    classes: tuple[str, ...] = ()
    fichier: str = ""
    format_sortie: str = ""
    taille_entree: int = 0
    # mémoire
    dimension: int = 0

    @property
    def chemin_fichier(self) -> Path | None:
        return Path(self.fichier).expanduser() if self.fichier else None


@dataclass(frozen=True, slots=True)
class ProfilIA:
    nom: str
    description: str
    api: str  # gemini | openai | aucun
    url: str = ""
    url_natif: str = ""
    variable_cle: str = ""
    cle_obligatoire: bool = False
    reprises: int = 1
    url_inference: str = ""
    agent: ConfigRole | None = None
    vision: ConfigRole | None = None
    pointage: ConfigRole | None = None  # modèle incarné ; sans lui, la vision pointe
    transcription: ConfigRole | None = None
    synthese: ConfigRole | None = None
    detection: ConfigRole | None = None
    profondeur: ConfigRole | None = None
    parole: ConfigRole | None = None
    eveil: ConfigRole | None = None
    memoire: ConfigRole | None = None

    def role(self, role: Role) -> ConfigRole | None:
        return getattr(self, role)

    def modeles(self) -> dict[str, str | None]:
        """Modèle de chaque rôle (« intégré » pour le code du rover, None si désactivé)."""
        sortie: dict[str, str | None] = {}
        for r in ROLES:
            c = self.role(r)
            if c is None or c.fournisseur == "aucun":
                if r in ROLES_HTTP:
                    sortie[r] = None
            elif c.fournisseur == "integre":
                sortie[r] = "intégré"
            else:
                sortie[r] = c.modele or (Path(c.fichier).name if c.fichier else None)
        return sortie


@dataclass(frozen=True, slots=True)
class ConfigAgent:
    historique_echanges: int = 6
    actions_max: int = 12


@dataclass(frozen=True, slots=True)
class ConfigVoix:
    seuil_entree_db: float = 8.0
    seuil_sortie_db: float = 4.0
    modulation_min_db: float = 2.0
    duree_min_s: float = 0.4
    duree_max_s: float = 15.0
    pre_roulement_s: float = 0.3
    silence_fin_s: float = 0.7
    bande_hz: tuple[float, float] = (250.0, 3500.0)
    mot_eveil: str = ""
    variantes_mot_eveil: tuple[str, ...] = ()
    niveau_hz: float = 10.0


def lire_env(fichier: Path | None) -> dict[str, str]:
    """Fichier `.env` simple : `NOM=valeur`, `export NOM=valeur`, guillemets et # acceptés."""
    valeurs: dict[str, str] = {}
    if fichier is None:
        return valeurs
    try:
        lignes = fichier.read_text(encoding="utf-8").splitlines()
    except OSError:
        return valeurs
    for ligne in lignes:
        ligne = ligne.strip()
        if not ligne or ligne.startswith("#") or "=" not in ligne:
            continue
        nom, valeur = ligne.split("=", 1)
        nom = nom.strip().removeprefix("export ").strip()
        valeur = valeur.strip()
        if len(valeur) >= 2 and valeur[0] == valeur[-1] and valeur[0] in "\"'":
            valeur = valeur[1:-1]
        elif " #" in valeur:
            valeur = valeur.split(" #", 1)[0].strip()
        if nom:
            valeurs[nom] = valeur
    return valeurs


def _nombre(d: Mapping[str, Any], cle: str, defaut: float) -> float:
    valeur = d.get(cle, defaut)
    if isinstance(valeur, bool) or not isinstance(valeur, int | float):
        raise ErreurConfigIA(f"{cle} : nombre attendu")
    return float(valeur)


def _role(nom_profil: str, role: Role, d: Mapping[str, Any], profil: Mapping[str, Any],
          hote: str) -> ConfigRole:
    api = str(profil.get("api", "openai"))
    fournisseur = str(d.get("fournisseur") or (api if role in ROLES_HTTP else ""))
    if fournisseur not in FOURNISSEURS:
        raise ErreurConfigIA(f"profil {nom_profil}, rôle {role} : fournisseur {fournisseur!r} "
                             f"inconnu ({', '.join(FOURNISSEURS)})")
    modele = d.get("modele", "")
    if not isinstance(modele, str):
        raise ErreurConfigIA(f"profil {nom_profil}, rôle {role} : `modele` doit être un texte")
    if not modele and fournisseur in ("gemini", "openai", "inference"):
        raise ErreurConfigIA(f"profil {nom_profil}, rôle {role} : `modele` manquant")
    if fournisseur == "local" and not d.get("fichier"):
        raise ErreurConfigIA(f"profil {nom_profil}, rôle {role} : `fichier` manquant (modèle "
                             "local)")
    if fournisseur in FOURNISSEURS_SANS_ADRESSE:
        url = ""
    elif d.get("url"):
        url = str(d["url"])
    elif fournisseur == "inference":
        url = str(profil.get("url_inference", ""))
    elif fournisseur == "gemini" and api != "gemini":
        url = URL_GEMINI  # rôle Gemini dans un profil local : adresse publique
    else:
        url = str(profil.get("url", ""))
    url = url.replace("{hote}", hote)
    if not url and fournisseur not in FOURNISSEURS_SANS_ADRESSE:
        raise ErreurConfigIA(f"profil {nom_profil}, rôle {role} : adresse manquante")
    url_natif = str(d.get("url_natif") or (profil.get("url_natif") if api == "gemini" else "")
                    or (URL_GEMINI_NATIF if fournisseur == "gemini" else ""))
    variable_cle = d.get("variable_cle")
    if variable_cle is None and fournisseur == "gemini" and api != "gemini":
        variable_cle = VARIABLE_CLE_GEMINI
    reflexion = d.get("reflexion")
    if reflexion is not None and reflexion not in REFLEXIONS:
        raise ErreurConfigIA(f"profil {nom_profil}, rôle {role} : réflexion {reflexion!r} inconnue")
    mode = str(d.get("mode", "openai_transcriptions"))
    if role == "transcription" and mode not in MODES_TRANSCRIPTION:
        raise ErreurConfigIA(f"profil {nom_profil} : mode de transcription {mode!r} inconnu")
    format_points = str(d.get("format_points", "yx_1000"))
    if format_points not in FORMATS_POINTS:
        raise ErreurConfigIA(f"profil {nom_profil} : format_points {format_points!r} inconnu")
    temperature = d.get("temperature")
    return ConfigRole(
        modele=modele,
        url=url,
        fournisseur=fournisseur,
        variable_cle=None if variable_cle is None else str(variable_cle),
        variable_cle_secours=str(d["variable_cle_secours"]) if d.get("variable_cle_secours")
        else None,
        url_natif=url_natif.replace("{hote}", hote),
        delai_s=_nombre(d, "delai_s", 30.0),
        reflexion=reflexion,
        jetons_max=int(d["jetons_max"]) if d.get("jetons_max") else None,
        temperature=None if temperature is None else float(temperature),
        requetes_par_heure=int(_nombre(d, "requetes_par_heure", 600)),
        requetes_par_jour=int(_nombre(d, "requetes_par_jour", 5000)),
        format_points=format_points,
        largeur_image=int(_nombre(d, "largeur_image", 640)),
        mode=mode,
        langue=str(d.get("langue", "fr")),
        style=str(d.get("style", "")),
        vocabulaire=tuple(str(m) for m in d.get("vocabulaire", ())),
        voix=str(d.get("voix", "")),
        format_audio=str(d.get("format_audio", "wav")),
        frequence_hz=_nombre(d, "frequence_hz", 0.0),
        seuil=_nombre(d, "seuil", 0.5),
        classes=tuple(str(c) for c in d.get("classes", ())),
        fichier=str(d.get("fichier", "")),
        format_sortie=str(d.get("format_sortie", "")),
        taille_entree=int(_nombre(d, "taille_entree", 0)),
        dimension=int(_nombre(d, "dimension", 0)),
    )


def _profil(nom: str, d: Mapping[str, Any], env: Mapping[str, str]) -> ProfilIA:
    api = str(d.get("api", "openai"))
    if api not in API_CONNUES:
        raise ErreurConfigIA(f"profil {nom} : api {api!r} inconnue ({', '.join(API_CONNUES)})")
    if api == "aucun":
        return ProfilIA(nom=nom, description=str(d.get("description", nom)), api=api)
    hote = str(d.get("hote", ""))
    if d.get("variable_hote") and env.get(str(d["variable_hote"])):
        hote = env[str(d["variable_hote"])]
    url = str(d.get("url", "")).replace("{hote}", hote)
    roles = {r: _role(nom, r, d[r], d, hote) for r in ROLES if isinstance(d.get(r), dict)}
    return ProfilIA(
        nom=nom,
        description=str(d.get("description", nom)),
        api=api,
        url=url,
        url_natif=str(d.get("url_natif", "")).replace("{hote}", hote),
        variable_cle=str(d.get("variable_cle", "")),
        cle_obligatoire=bool(d.get("cle_obligatoire", False)),
        reprises=max(0, int(d.get("reprises", 1))),
        url_inference=str(d.get("url_inference", "")).replace("{hote}", hote),
        **roles,
    )


def _voix(d: Mapping[str, Any]) -> ConfigVoix:
    defaut = ConfigVoix()
    bande = d.get("bande_hz", defaut.bande_hz)
    if not (isinstance(bande, list | tuple) and len(bande) == 2 and 0 < bande[0] < bande[1]):
        raise ErreurConfigIA("voix.bande_hz : [bas, haut] attendu")
    return ConfigVoix(
        seuil_entree_db=_nombre(d, "seuil_entree_db", defaut.seuil_entree_db),
        seuil_sortie_db=_nombre(d, "seuil_sortie_db", defaut.seuil_sortie_db),
        modulation_min_db=_nombre(d, "modulation_min_db", defaut.modulation_min_db),
        duree_min_s=_nombre(d, "duree_min_s", defaut.duree_min_s),
        duree_max_s=_nombre(d, "duree_max_s", defaut.duree_max_s),
        pre_roulement_s=_nombre(d, "pre_roulement_s", defaut.pre_roulement_s),
        silence_fin_s=_nombre(d, "silence_fin_s", defaut.silence_fin_s),
        bande_hz=(float(bande[0]), float(bande[1])),
        mot_eveil=str(d.get("mot_eveil", "")),
        variantes_mot_eveil=tuple(str(m) for m in d.get("variantes_mot_eveil", ())),
        niveau_hz=_nombre(d, "niveau_hz", defaut.niveau_hz),
    )


@dataclass
class ConfigIA:
    """Profils chargés, profil actif (modifiable en marche) et accès aux clés."""

    profils: dict[str, ProfilIA]
    nom_defaut: str
    agent: ConfigAgent = field(default_factory=ConfigAgent)
    voix: ConfigVoix = field(default_factory=ConfigVoix)
    env: Mapping[str, str] = field(default_factory=dict)
    fichier_choix: Path | None = None
    fichier_env: Path | None = None

    def __post_init__(self) -> None:
        if self.nom_defaut not in self.profils:
            raise ErreurConfigIA(f"profil actif {self.nom_defaut!r} absent de la configuration")
        self._nom_actif = self.nom_defaut
        self._verrou = threading.Lock()
        self._rappels: list[Callable[[ProfilIA], None]] = []
        self._passerelle: Passerelle | None = None

    # ------------------------------------------------------------------ chargement

    @classmethod
    def depuis_dict(cls, donnees: Mapping[str, Any], env: Mapping[str, str] | None = None,
                    fichier_choix: Path | None = None,
                    fichier_env: Path | None = None) -> ConfigIA:
        env = dict(env or {})
        brut = donnees.get("profils")
        if not isinstance(brut, dict) or not brut:
            raise ErreurConfigIA("aucun profil dans la configuration de l'IA")
        profils = {nom: _profil(nom, d, env) for nom, d in brut.items()}
        agent = donnees.get("agent", {})
        config = cls(
            profils=profils,
            nom_defaut=str(donnees.get("profil_actif", next(iter(profils)))),
            agent=ConfigAgent(
                historique_echanges=int(agent.get("historique_echanges", 6)),
                actions_max=int(agent.get("actions_max", 12)),
            ),
            voix=_voix(donnees.get("voix", {})),
            env=env,
            fichier_choix=fichier_choix,
            fichier_env=fichier_env,
        )
        choix = env.get(VARIABLE_PROFIL) or config._choix_enregistre()
        if choix in profils:
            config._nom_actif = choix
        return config

    @classmethod
    def charger(cls, fichier: Path = FICHIER_IA, fichier_env: Path | None = FICHIER_ENV,
                fichier_choix: Path | None = FICHIER_CHOIX,
                env: Mapping[str, str] | None = None) -> ConfigIA:
        """`env` : variables (par défaut celles du processus, complétées par le fichier .env)."""
        with open(fichier, "rb") as f:
            donnees = tomllib.load(f)
        variables = lire_env(fichier_env) | dict(os.environ if env is None else env)
        return cls.depuis_dict(donnees, variables, fichier_choix, fichier_env)

    def _choix_enregistre(self) -> str | None:
        if self.fichier_choix is None or not self.fichier_choix.exists():
            return None
        try:
            choix = json.loads(self.fichier_choix.read_text(encoding="utf-8")).get("profil")
        except (OSError, ValueError, AttributeError):
            return None
        return choix if isinstance(choix, str) else None

    # ------------------------------------------------------------------ profil actif

    @property
    def nom_actif(self) -> str:
        return self._nom_actif

    @property
    def actif(self) -> ProfilIA:
        return self.profils[self._nom_actif]

    def profil(self, nom: str | None = None) -> ProfilIA:
        if nom is None:
            return self.actif
        try:
            return self.profils[nom]
        except KeyError:
            raise ErreurConfigIA(f"profil d'IA inconnu : {nom!r}") from None

    def changer_profil(self, nom: str, enregistrer: bool = True) -> ProfilIA:
        """Change de fournisseur en marche (message `ia_profil`). Les requêtes en cours finissent
        avec l'ancien profil. Le choix est gardé pour le prochain démarrage si `enregistrer`.
        Lève ErreurConfigIA si le profil est inconnu."""
        profil = self.profil(nom)
        with self._verrou:
            self._nom_actif = nom
            if enregistrer and self.fichier_choix is not None:
                try:
                    self.fichier_choix.parent.mkdir(parents=True, exist_ok=True)
                    temporaire = self.fichier_choix.with_suffix(".tmp")
                    temporaire.write_text(json.dumps({"profil": nom}), encoding="utf-8")
                    temporaire.replace(self.fichier_choix)
                except OSError:
                    pass  # le choix vaut pour cette session
            rappels = list(self._rappels)
        for rappel in rappels:
            rappel(profil)
        return profil

    def sur_changement(self, rappel: Callable[[ProfilIA], None]) -> None:
        self._rappels.append(rappel)

    # ------------------------------------------------------------------ clés et disponibilité

    def cle(self, nom: str | None = None) -> str | None:
        profil = self.profil(nom)
        if not profil.variable_cle:
            return None
        return self.env.get(profil.variable_cle) or None

    def cle_role(self, role: Role, nom: str | None = None, secours: bool = False) -> str | None:
        """Clé d'un rôle : la sienne, sinon celle du profil ; `secours` : sa clé de secours."""
        profil = self.profil(nom)
        config_role = profil.role(role)
        if config_role is None:
            return None
        if secours:
            variable = config_role.variable_cle_secours
            return (self.env.get(variable) or None) if variable else None
        variable = config_role.variable_cle or profil.variable_cle
        return (self.env.get(variable) or None) if variable else None

    def definir_cle(self, cle_api: str, nom: str | None = None,
                    fichier_env: Path | None = None) -> None:
        """Enregistre la clé en mémoire et dans `.env` pour le profil donné."""
        profil = self.profil(nom)
        variable = profil.variable_cle or "GEMINI_API_KEY"
        valeur = cle_api.strip()
        cible_env = self.fichier_env if fichier_env is None else fichier_env
        with self._verrou:
            env = dict(self.env)
            if valeur:
                env[variable] = valeur
            else:
                env.pop(variable, None)
            self.env = env
            if cible_env is not None:
                try:
                    _ecrire_variable(cible_env, variable, valeur)
                except OSError:
                    pass

    def disponibilite(self, role: Role = "agent", nom: str | None = None
                      ) -> tuple[bool, str | None]:
        """(disponible, raison lisible si non)."""
        profil = self.profil(nom)
        if profil.api == "aucun":
            return False, "aucun modèle (profil « aucun »)"
        if profil.role(role) is None:
            return False, f"pas de modèle de {role} dans le profil « {profil.nom} »"
        if profil.cle_obligatoire and not self.cle_role(role, profil.nom):
            if self.cle_role(role, profil.nom, secours=True):
                return True, None  # seule la clé de secours existe : elle sert pour ce rôle
            return False, f"clé d'API absente (variable {profil.variable_cle})"
        return True, None

    def resume(self) -> dict[str, Any]:
        """Champ `ia` du message `bienvenue` (PROTOCOLE.md). Ne contient jamais la clé."""
        profil = self.actif
        disponible, raison = self.disponibilite("agent")
        return {
            "profil": profil.nom,
            "disponible": disponible,
            "modeles": profil.modeles(),
            "profils": list(self.profils),
            "raison": raison,
        }

    def passerelle(self) -> Passerelle:
        """Clients HTTP et quotas partagés par l'agent, la vision et la transcription."""
        if self._passerelle is None:
            from fieldmind.ia.fournisseurs import Passerelle  # import tardif : pas de cycle

            self._passerelle = Passerelle(self)
        return self._passerelle


def _ecrire_variable(fichier: Path, variable: str, valeur: str) -> None:
    """Remplace (ou ajoute) une ligne NOM=valeur en gardant commentaires et ordre ; droits 600."""
    lignes = fichier.read_text(encoding="utf-8").splitlines() if fichier.exists() else []
    trouvee = False
    for i, ligne in enumerate(lignes):
        if ligne.split("=", 1)[0].strip() == variable and not ligne.lstrip().startswith("#"):
            lignes[i] = f"{variable}={valeur}"
            trouvee = True
    if not trouvee:
        lignes.append(f"{variable}={valeur}")
    temporaire = fichier.with_name(fichier.name + ".tmp")
    temporaire.write_text("\n".join(lignes) + "\n", encoding="utf-8")
    os.chmod(temporaire, 0o600)
    temporaire.replace(fichier)
