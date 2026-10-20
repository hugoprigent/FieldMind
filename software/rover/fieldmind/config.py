"""Profil du rover : config/rover.toml, complété des réglages enregistrés depuis le poste.

    profil = Profil.charger()
    profil.geometrie.empattement_m       # accès par attributs
    profil.valeur("securite.latence_s")  # ou par chemin
    profil.regler("direction.servo_neutre_deg", 2.5)  # borné, enregistré dans reglages.json

Seules les clés de `REGLABLES` se modifient en marche, dans leurs bornes : le poste ne peut pas
désactiver une sécurité ni dépasser les plafonds du fichier.
"""

from __future__ import annotations

import copy
import json
import math
import os
import threading
import tomllib
from collections.abc import Callable
from pathlib import Path
from typing import Any

RACINE = Path(__file__).resolve().parent.parent
FICHIER_PROFIL = RACINE / "config" / "rover.toml"
DOSSIER_UTILISATEUR = Path(os.environ.get("FIELDMIND_DOSSIER", "~/.config/fieldmind")).expanduser()

# Réglages modifiables en marche : clé -> (min, max, pas, unité, libellé, groupe).
# Les bornes protègent le rover (aucune sécurité ne peut être coupée d'ici) ; (0, 1, 1, "")
# désigne un interrupteur.
_DETAILS: dict[str, tuple[float, float, float, str, str, str]] = {
    "direction.servo_neutre_deg": (-20.0, 20.0, 0.5, "°", "Neutre du servo (tout droit)",
                                   "Direction"),
    "direction.servo_par_degre_roue": (0.2, 5.0, 0.01, "°/°", "Servo par degré de roue",
                                       "Direction"),
    "direction.servo_max_deg": (5.0, 90.0, 1.0, "°", "Butée logicielle du servo", "Direction"),
    "direction.servo_vitesse_deg_s": (10.0, 180.0, 5.0, "°/s", "Vitesse du servo", "Direction"),
    "vitesses.max_manuel_m_s": (0.05, 1.5, 0.05, "m/s", "Vitesse max en manuel", "Vitesses"),
    "vitesses.max_auto_m_s": (0.05, 1.0, 0.05, "m/s", "Vitesse max en auto", "Vitesses"),
    "vitesses.max_arriere_m_s": (0.05, 0.5, 0.05, "m/s", "Vitesse max en arrière", "Vitesses"),
    "vitesses.defaut_action_m_s": (0.05, 1.0, 0.05, "m/s", "Vitesse des actions", "Vitesses"),
    "traction.pwm_par_m_s": (5.0, 100.0, 0.5, "%/(m/s)", "PWM pour 1 m/s", "Traction"),
    "traction.pwm_seuil": (0, 20, 1, "%", "Seuil de démarrage", "Traction"),
    "traction.pwm_max": (5, 100, 1, "%", "PWM maximal", "Traction"),
    "traction.pwm_max_banc": (1, 30, 1, "%", "PWM maximal au banc", "Traction"),
    "traction.equilibre_d": (0.8, 1.25, 0.01, "×", "Gain du moteur droit", "Traction"),
    "traction.delai_inversion_s": (0.2, 3.0, 0.1, "s", "Arrêt avant inversion", "Traction"),
    "traction.rampe_acceleration_m_s2": (0.1, 2.0, 0.1, "m/s²", "Accélération", "Traction"),
    "traction.differentiel": (0, 1, 1, "", "Différentiel électronique", "Traction"),
    "traction.boucle_lacet": (0, 1, 1, "", "Correction par le gyroscope", "Traction"),
    "roues.capteurs": (0, 1, 1, "", "Capteurs de vitesse des roues (SC)", "Traction"),
    "roues.impulsions_par_metre": (0.0, 5000.0, 1.0, "imp/m", "Impulsions par mètre", "Traction"),
    "securite.marge_avant_m": (0.10, 1.0, 0.05, "m", "Marge devant", "Sécurité"),
    "securite.marge_laterale_m": (0.05, 0.5, 0.01, "m", "Marge sur les côtés", "Sécurité"),
    "securite.deceleration_m_s2": (0.2, 3.0, 0.1, "m/s²", "Freinage supposé", "Sécurité"),
    "securite.latence_s": (0.15, 1.0, 0.05, "s", "Latence", "Sécurité"),
    "securite.vitesse_sans_stereo_m_s": (0.05, 0.6, 0.05, "m/s", "Vitesse sans stéréo", "Sécurité"),
    "securite.vitesse_pres_personne_m_s": (0.05, 0.5, 0.05, "m/s", "Vitesse près d'une personne",
                                           "Sécurité"),
    "securite.exiger_stereo_en_auto": (0, 1, 1, "", "Stéréo obligatoire en auto", "Sécurité"),
    "securite.inclinaison_max_deg": (10.0, 35.0, 1.0, "°", "Inclinaison maximale", "Sécurité"),
    "securite.perte_operateur_s": (1.0, 60.0, 1.0, "s", "Perte du poste", "Sécurité"),
    "securite.geofence_m": (5.0, 500.0, 5.0, "m", "Rayon autour du départ", "Sécurité"),
    "esp.chien_de_garde_ms": (100, 2000, 50, "ms", "Chien de garde de l'ESP", "Sécurité"),
    "gnss.tolerance_m": (0.5, 10.0, 0.5, "m", "Arrivée à un but GPS", "GPS"),
    "gnss.hdop_max": (1.0, 10.0, 0.5, "", "HDOP maximal", "GPS"),
    "gnss.alignement_etendue_m": (3.0, 30.0, 1.0, "m", "Trajet avant alignement", "GPS"),
    "ia.vigilance": (0, 1, 1, "", "Analyse IA automatique des images (payante)", "IA"),
    "ia.vigilance_periode_s": (4.0, 300.0, 1.0, "s", "Période de l'analyse automatique", "IA"),
    "ia.detection_continue": (0, 1, 1, "", "Détection continue des personnes et animaux (GX10)",
                              "IA"),
    "ia.detection_periode_s": (1.0, 30.0, 0.5, "s", "Période de la détection continue", "IA"),
    "ia.voix_rover": (0, 1, 1, "", "Voix du rover (synthèse sur le GX10)", "IA"),
    "remorque.attelee": (0, 1, 1, "", "Remorque attelée", "Remorque"),
    "remorque.volets_deplies": (0, 1, 1, "", "Volets dépliés (plus large)", "Remorque"),
    "remorque.masse_kg": (0.5, 6.0, 0.1, "kg", "Masse de la remorque et du drone", "Remorque"),
    "remorque.vitesse_max_m_s": (0.1, 0.8, 0.05, "m/s", "Vitesse maximale", "Remorque"),
    "remorque.vitesse_max_arriere_m_s": (0.05, 0.2, 0.01, "m/s", "Vitesse en marche arrière",
                                         "Remorque"),
    "remorque.angle_max_arriere_deg": (5.0, 30.0, 1.0, "°", "Angle maximal en marche arrière",
                                       "Remorque"),
    "remorque.marche_arriere_auto": (0, 1, 1, "", "Marche arrière en automatique", "Remorque"),
    "capteurs.lidar": (0, 1, 1, "", "Lidar (sans lui, le rover ne roule pas)", "Capteurs actifs"),
    "capteurs.camera_gauche": (0, 1, 1, "", "Caméra gauche", "Capteurs actifs"),
    "capteurs.camera_droite": (0, 1, 1, "", "Caméra droite", "Capteurs actifs"),
    "perception.stereo_active": (0, 1, 1, "", "Stéréo (obstacles bas)", "Capteurs actifs"),
    "capteurs.gnss": (0, 1, 1, "", "GPS", "Capteurs actifs"),
    "capteurs.imu": (0, 1, 1, "", "Centrale inertielle", "Capteurs actifs"),
    "perception.stereo_ips": (1, 8, 1, "i/s", "Cadence de la stéréo", "Capteurs"),
    "lidar.decalage_deg": (-180.0, 180.0, 0.5, "°", "Orientation du lidar", "Capteurs"),
    "cameras.inclinaison_deg": (0.0, 30.0, 0.5, "°", "Inclinaison des caméras", "Capteurs"),
    "cameras.hfov_deg": (40.0, 120.0, 1.0, "°", "Champ des caméras (sans calibration)",
                         "Capteurs"),
}
REGLABLES: dict[str, tuple[float, float]] = {k: (v[0], v[1]) for k, v in _DETAILS.items()}
"""Clé -> (min, max). Les booléens sont bornés par (0, 1)."""
TEXTES_REGLABLES = {"cameras.port_gauche", "cameras.port_droite", "imu.axe_x", "imu.axe_y",
                    "imu.axe_z", "audio.peripherique"}


class Section:
    """Vue par attributs d'une table TOML (lecture seule)."""

    def __init__(self, donnees: dict[str, Any]) -> None:
        object.__setattr__(self, "_d", donnees)

    def __getattr__(self, nom: str) -> Any:
        try:
            valeur = self._d[nom]
        except KeyError:
            raise AttributeError(nom) from None
        return Section(valeur) if isinstance(valeur, dict) else valeur

    def __setattr__(self, nom: str, valeur: Any) -> None:
        raise AttributeError("profil en lecture seule : utiliser Profil.regler()")

    def get(self, nom: str, defaut: Any = None) -> Any:
        return self._d.get(nom, defaut)

    def dict(self) -> dict[str, Any]:
        return copy.deepcopy(self._d)


class Profil:
    def __init__(self, base: dict[str, Any], reglages: dict[str, Any],
                 fichier_reglages: Path | None) -> None:
        self._base = base
        self._reglages = reglages
        self._fichier = fichier_reglages
        self._verrou = threading.Lock()
        self._rappels: list[Callable[[str, Any], None]] = []
        self._fusion = self._fusionner()

    @classmethod
    def charger(cls, fichier: Path = FICHIER_PROFIL,
                reglages: Path | None = DOSSIER_UTILISATEUR / "reglages.json") -> Profil:
        with open(fichier, "rb") as f:
            base = tomllib.load(f)
        enregistres: dict[str, Any] = {}
        if reglages is not None and reglages.exists():
            try:
                enregistres = json.loads(reglages.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                enregistres = {}
        propres = {}
        for cle, valeur in enregistres.items():
            try:
                propres[cle] = _borner(cle, valeur)
            except (KeyError, TypeError, ValueError):
                continue  # réglage obsolète ou hors bornes : ignoré
        return cls(base, propres, reglages)

    def _fusionner(self) -> dict[str, Any]:
        d = copy.deepcopy(self._base)
        for cle, valeur in self._reglages.items():
            section, nom = cle.split(".", 1)
            d.setdefault(section, {})[nom] = valeur
        return d

    def __getattr__(self, nom: str) -> Section:
        if nom.startswith("_"):
            raise AttributeError(nom)
        try:
            return Section(self._fusion[nom])
        except KeyError:
            raise AttributeError(nom) from None

    def valeur(self, chemin: str, defaut: Any = None) -> Any:
        noeud: Any = self._fusion
        for partie in chemin.split("."):
            if not isinstance(noeud, dict) or partie not in noeud:
                return defaut
            noeud = noeud[partie]
        return copy.deepcopy(noeud)

    def dict(self) -> dict[str, Any]:
        return copy.deepcopy(self._fusion)

    def reglages(self) -> dict[str, Any]:
        return dict(self._reglages)

    def regler(self, cle: str, valeur: Any) -> Any:
        """Modifie un réglage autorisé, borné ; l'enregistre. Renvoie la valeur retenue."""
        valeur = _borner(cle, valeur)
        with self._verrou:
            self._reglages[cle] = valeur
            self._fusion = self._fusionner()
            if self._fichier is not None:
                self._fichier.parent.mkdir(parents=True, exist_ok=True)
                temporaire = self._fichier.with_suffix(".tmp")
                temporaire.write_text(json.dumps(self._reglages, indent=2, ensure_ascii=False),
                                      encoding="utf-8")
                temporaire.replace(self._fichier)
            rappels = list(self._rappels)
        for rappel in rappels:
            rappel(cle, valeur)
        return valeur

    def sur_changement(self, rappel: Callable[[str, Any], None]) -> None:
        self._rappels.append(rappel)


def _borner(cle: str, valeur: Any) -> Any:
    if cle in TEXTES_REGLABLES:
        if not isinstance(valeur, str) or len(valeur) > 200:
            raise ValueError(f"{cle} : texte attendu")
        return valeur
    if cle not in REGLABLES:
        raise KeyError(f"{cle} n'est pas réglable en marche")
    bas, haut = REGLABLES[cle]
    if isinstance(valeur, bool) or (bas, haut) == (0, 1):
        return bool(valeur)
    nombre = float(valeur)
    if nombre != nombre:  # NaN
        raise ValueError(f"{cle} : nombre invalide")
    nombre = min(max(nombre, bas), haut)
    return int(round(nombre)) if isinstance(bas, int) and isinstance(haut, int) else nombre


def chemin_utilisateur(texte: str) -> Path:
    return Path(texte).expanduser()


def braquage_max(profil: Profil) -> float:
    """Braquage maximal (rad) de la roue avant virtuelle centrale du modèle bicyclette.

    `geometrie.braquage_max_deg` est la butée de la roue **intérieure** (CAO) ; avec l'épure
    d'Ackermann, l'essieu arrière tourne alors autour d'un rayon R = L / tan(δi) + voie_avant / 2,
    soit un braquage central atan(L / R) : environ 25,8° pour 31°, 0,79 m de rayon minimal."""
    g = profil.geometrie
    empattement = float(g.empattement_m)
    interieur = math.radians(float(g.braquage_max_deg))
    rayon = empattement / math.tan(interieur) + float(g.get("voie_avant_m", 0.31)) / 2
    return math.atan(empattement / rayon)


def details_reglables() -> dict[str, dict[str, object]]:
    """Description des réglages pour le menu Réglages du poste (`bienvenue.reglables_details`)."""
    details: dict[str, dict[str, object]] = {}
    for cle, (bas, haut, pas, unite, libelle, groupe) in _DETAILS.items():
        booleen = (bas, haut) == (0, 1) and pas == 1 and not unite
        details[cle] = {"min": bas, "max": haut, "pas": pas, "unite": unite, "libelle": libelle,
                        "groupe": groupe, "type": "booleen" if booleen else "nombre"}
    for cle, libelle in (("cameras.port_gauche", "Port USB de la caméra gauche"),
                         ("cameras.port_droite", "Port USB de la caméra droite"),
                         ("imu.axe_x", "Axe x de la centrale"),
                         ("imu.axe_y", "Axe y de la centrale"),
                         ("imu.axe_z", "Axe z de la centrale"),  # x, -x, y, -y, z, -z
                         ("audio.peripherique", "Micro (ALSA)")):
        details[cle] = {"libelle": libelle, "groupe": "Capteurs", "type": "texte"}
    return details
