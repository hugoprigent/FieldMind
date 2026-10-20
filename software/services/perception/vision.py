"""Analyse des images de la caméra par un modèle de vision (VLM).

Règle du projet : l'IA propose, le code décide. Le modèle renvoie, pour une image, une liste
d'objets (catégorie, boîte englobante, passage bloqué ou non) et un court résumé. Le code :
- valide et borne chaque objet (un objet mal formé est ignoré, pas toute la réponse) ;
- place au sol les objets à l'aide de la géométrie calibrée de la caméra (`camera.py`) ;
- décide seul de ce qui entre dans la carte : les obstacles bas que le lidar ne voit pas.

Fournisseurs : un modèle gratuit d'OpenRouter ou un modèle local servi par vLLM (API compatible
OpenAI), ou, en simulation, un détecteur parfait tiré de la vérité terrain
(`simulation/vision_oracle.py`, pour les tests et l'évaluation).
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass
from typing import Any, Literal, Protocol, get_args

from pydantic import BaseModel, ConfigDict, Field, ValidationError

from services.mission_planner.api_openai import (
    ClientCompatibleOpenAI,
    EchecFournisseur,
    extraire_json,
)

journal = logging.getLogger("fieldmind.vision")

Categorie = Literal[
    "tronc_couche",
    "branche",
    "rocher",
    "arbre",
    "trou",
    "eau",
    "personne",
    "animal",
    "vehicule",
    "autre",
]
CATEGORIES: tuple[str, ...] = get_args(Categorie)
NOMS_CATEGORIES: dict[str, str] = {
    "tronc_couche": "tronc couché",
    "branche": "branche au sol",
    "rocher": "rocher",
    "arbre": "arbre",
    "trou": "trou ou fossé",
    "eau": "eau",
    "personne": "personne",
    "animal": "animal",
    "vehicule": "véhicule",
    "autre": "obstacle",
}
PLURIELS_CATEGORIES: dict[str, str] = {
    "tronc_couche": "troncs couchés",
    "branche": "branches au sol",
    "rocher": "rochers",
    "arbre": "arbres",
    "trou": "trous ou fossés",
    "eau": "zones d'eau",
    "personne": "personnes",
    "animal": "animaux",
    "vehicule": "véhicules",
    "autre": "obstacles",
}
OBJETS_MAX = 12


class ObjetVu(BaseModel):
    """Objet proposé par le modèle. Boîte : [x_min, y_min, x_max, y_max] ramenée sur 0–1000 par
    `lire_analyse` (le modèle répond en pixels de l'image), origine en
    haut à gauche de l'image ; le bas de la boîte est le contact avec le sol."""

    model_config = ConfigDict(extra="ignore")

    categorie: Categorie
    boite: tuple[float, float, float, float]
    bloque_passage: bool
    confiance: float = Field(ge=0, le=1)


Decision = Literal["continuer", "ralentir", "s_arreter"]


class AnalyseVision(BaseModel):
    resume: str = Field(max_length=400)
    objets: list[ObjetVu] = Field(default_factory=list, max_length=OBJETS_MAX)
    decision: Decision = "continuer"
    """Proposition de l'IA d'observation ; le cerveau la valide et l'applique."""
    raison_decision: str = Field(default="", max_length=200)


def schema_analyse() -> dict[str, Any]:
    """Schéma JSON strict de la réponse du modèle."""
    return {
        "type": "object",
        "properties": {
            "resume": {"type": "string"},
            "objets": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "categorie": {"type": "string", "enum": list(CATEGORIES)},
                        "boite": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 4,
                            "maxItems": 4,
                        },
                        "bloque_passage": {"type": "boolean"},
                        "confiance": {"type": "number"},
                    },
                    "required": ["categorie", "boite", "bloque_passage", "confiance"],
                    "additionalProperties": False,
                },
            },
            "decision": {"type": "string", "enum": ["continuer", "ralentir", "s_arreter"]},
            "raison_decision": {"type": "string"},
        },
        "required": ["resume", "objets", "decision", "raison_decision"],
        "additionalProperties": False,
    }


FormatBoites = Literal["pixels", "gemini"]
"""« gemini » : [y_min, x_min, y_max, x_max] ramenés sur 0–1000, le format natif des modèles
Gemini (Robotics-ER en particulier), plus précis qu'en pixels."""


def format_boites_pour(modele: str) -> FormatBoites:
    return "gemini" if modele.startswith("gemini") else "pixels"


def consigne_vision(
    hauteur_camera_m: float,
    largeur_rover_m: float,
    garde_sol_m: float,
    format_boites: FormatBoites = "pixels",
) -> str:
    if format_boites == "gemini":
        boite = (
            "- boite : [y_min, x_min, y_max, x_max] normalisés de 0 à 1000 (y de haut en bas, "
            "x de gauche à droite). Le bas de la boîte (y_max) doit être exactement là où "
            "l'objet touche le sol ;"
        )
    else:
        boite = (
            "- boite : [x_min, y_min, x_max, y_max] en pixels de l'image (dimensions données "
            "avec l'image ; x de gauche à droite, y de haut en bas, origine en haut à gauche). "
            "Le bas de la boîte (y_max) doit être exactement là où l'objet touche le sol ;"
        )
    return "\n".join(
        [
            "Tu es le système de vision d'un petit rover de reconnaissance en forêt. L'image "
            f"vient de sa caméra avant, grand angle, placée à {hauteur_camera_m:.2f} m du sol et "
            "légèrement inclinée vers le bas.",
            "Liste les objets utiles à la navigation : obstacles au sol à moins de 15 m environ "
            "(tronc couché, branche, rocher, trou, eau), êtres vivants, véhicules, et les arbres "
            "seulement s'ils sont à moins de 5 m.",
            "Pour chaque objet :",
            "- categorie : " + ", ".join(CATEGORIES) + " ;",
            boite,
            f"- bloque_passage : true si le rover (largeur {largeur_rover_m:.2f} m, garde au sol "
            f"{garde_sol_m * 100:.0f} cm) ne peut pas rouler dessus ;",
            "- confiance : entre 0 et 1.",
            "N'invente rien : si tu n'es pas sûr, mets une confiance basse. Au plus "
            f"{OBJETS_MAX} objets, les plus proches d'abord.",
            "resume : la note d'observation, une ou deux phrases en français : ce que voit le "
            "rover (terrain, piste, végétation, obstacles) et ce qui est notable pour la mission "
            "(présence humaine ou animale, véhicule, danger, changement de milieu).",
            "decision, pour la suite de la mission (la navigation évite déjà seule les "
            "obstacles grâce à ses capteurs) :",
            "- continuer : rien de particulier ;",
            "- ralentir : zone délicate (terrain encombré, visibilité réduite, être vivant au "
            "loin) ;",
            "- s_arreter : situation qui demande un humain (personne ou animal proche du "
            "chemin, feu, fumée, eau profonde, ravin, véhicule en mouvement).",
            "raison_decision : une phrase courte qui justifie la décision.",
            "Réponds uniquement par l'objet JSON demandé.",
        ]
    )


def lire_analyse(
    donnees: Any, largeur: int, hauteur: int, format_boites: FormatBoites = "pixels"
) -> AnalyseVision:
    """Valide la réponse du modèle (boîtes en pixels d'une image largeur × hauteur, ou au format
    Gemini) et ramène les boîtes sur 0–1000 [x_min, y_min, x_max, y_max] ; les objets mal formés
    sont écartés un à un."""
    if format_boites == "gemini" and isinstance(donnees, dict):
        # [y_min, x_min, y_max, x_max] sur 0–1000 → [x_min, y_min, x_max, y_max] sur 0–1000.
        largeur = hauteur = 1000
        for brut in donnees.get("objets") or []:
            b = brut.get("boite") if isinstance(brut, dict) else None
            if isinstance(b, list) and len(b) == 4:
                brut["boite"] = [b[1], b[0], b[3], b[2]]
    if not isinstance(donnees, dict):
        raise EchecFournisseur("réponse non conforme au schéma")
    resume = str(donnees.get("resume", ""))[:400]
    decision = donnees.get("decision")
    if decision not in ("continuer", "ralentir", "s_arreter"):
        decision = "continuer"
    raison = str(donnees.get("raison_decision") or "")[:200]
    objets: list[ObjetVu] = []
    for brut in donnees.get("objets") or []:
        try:
            objet = ObjetVu.model_validate(brut)
        except ValidationError:
            continue
        bx0, by0, bx1, by1 = objet.boite
        x0, x1 = (min(max(v / largeur * 1000, 0.0), 1000.0) for v in (bx0, bx1))
        y0, y1 = (min(max(v / hauteur * 1000, 0.0), 1000.0) for v in (by0, by1))
        if x1 - x0 < 1 or y1 - y0 < 1:
            continue
        objets.append(objet.model_copy(update={"boite": (x0, y0, x1, y1)}))
        if len(objets) >= OBJETS_MAX:
            break
    return AnalyseVision(resume=resume, objets=objets, decision=decision, raison_decision=raison)


@dataclass(frozen=True, slots=True)
class DemandeVision:
    """Image à analyser, avec la pose estimée du rover à l'instant de la prise de vue."""

    id: int
    t: float
    image_jpeg_b64: str
    largeur: int
    hauteur: int
    x: float
    y: float
    cap: float
    raison: Literal["vigilance", "operateur"]
    contexte: str = ""
    """Mission en cours, donnée au modèle avec l'image."""


@dataclass(frozen=True, slots=True)
class ResultatVision:
    analyse: AnalyseVision | None
    fournisseur: str
    duree_ms: float
    erreur: str | None = None
    reessayer_dans_s: float | None = None


class FournisseurVision(Protocol):
    nom: str

    async def analyser(self, demande: DemandeVision) -> AnalyseVision: ...


class FournisseurVisionOpenAI:
    """Modèle de vision derrière une API compatible OpenAI (OpenRouter gratuit ou vLLM)."""

    def __init__(
        self,
        client: ClientCompatibleOpenAI,
        consigne: str,
        jetons_max: int,
        format_boites: FormatBoites = "pixels",
    ) -> None:
        self.format_boites = format_boites
        prefixe = "openrouter" if client.openrouter else "openai"
        self.nom = f"{prefixe} · {client.modele}"
        self._client = client
        self._schema = schema_analyse()
        self._consigne = consigne
        if client.mode_json == "objet":
            self._consigne += "\n\nSchéma JSON de la réponse :\n" + json.dumps(
                self._schema, ensure_ascii=False
            )
        self._jetons_max = jetons_max
        self.dernier_modele: str | None = None

    async def analyser(self, demande: DemandeVision) -> AnalyseVision:
        messages: list[dict[str, Any]] = [
            {"role": "system", "content": self._consigne},
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": (f"{demande.contexte}\n" if demande.contexte else "")
                        + "Image de la caméra avant du rover, "
                        f"{demande.largeur} × {demande.hauteur} pixels :",
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{demande.image_jpeg_b64}"},
                    },
                ],
            },
        ]
        reponse = await self._client.completer(
            messages, self._schema, "analyse_vision", self._jetons_max
        )
        self.dernier_modele = reponse.modele
        try:
            return lire_analyse(
                extraire_json(reponse.contenu), demande.largeur, demande.hauteur, self.format_boites
            )
        except ValueError as erreur:
            raise EchecFournisseur("réponse non conforme au schéma") from erreur


async def executer_analyse(
    fournisseur: FournisseurVision, demande: DemandeVision
) -> ResultatVision:
    """Appelle le fournisseur et convertit les échecs en résultat (jamais d'exception)."""
    debut = time.perf_counter()
    try:
        analyse = await fournisseur.analyser(demande)
    except EchecFournisseur as erreur:
        journal.warning("analyse de la caméra impossible : %s", erreur)
        return ResultatVision(
            None,
            fournisseur.nom,
            (time.perf_counter() - debut) * 1000,
            str(erreur),
            erreur.reessayer_dans_s,
        )
    nom = fournisseur.nom
    servi = getattr(fournisseur, "dernier_modele", None)
    if servi and not nom.endswith(servi):
        nom += f" (servi par {servi})"
    return ResultatVision(analyse, nom, (time.perf_counter() - debut) * 1000)
