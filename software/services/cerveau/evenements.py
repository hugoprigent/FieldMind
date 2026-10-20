"""Événements publiés par le cerveau vers le poste opérateur."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

EtatMission = Literal["attente", "manuel", "calcul", "route", "bloque", "atteint", "echec"]
Niveau = Literal["info", "succes", "alerte", "erreur"]


class _Evenement(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class Statut(_Evenement):
    type: Literal["statut"] = "statut"
    t: float
    etat: EtatMission
    message: str


class Journal(_Evenement):
    type: Literal["journal"] = "journal"
    t: float
    niveau: Niveau = "info"
    texte: str
    mesures: dict[str, float | int | str] = Field(default_factory=dict)


class PoseEstimee(_Evenement):
    """Pose estimée par la localisation du cerveau (centre du rover), avec son incertitude."""

    type: Literal["pose_estimee"] = "pose_estimee"
    t: float
    x: float
    y: float
    cap: float
    sigma_xy: float
    sigma_cap: float
    vitesse: float
    source: str


class Chemin(_Evenement):
    type: Literal["chemin"] = "chemin"
    t: float
    id: int
    points: list[tuple[float, float]]
    objectif: tuple[float, float] | None
    vitesse_max: float


class Carte(_Evenement):
    """Rectangle modifié de la grille d'occupation, en octets zlib encodés en base64.

    Chaque octet vaut la probabilité d'occupation × 254 ; 255 signifie « jamais observé ».
    """

    type: Literal["carte"] = "carte"
    t: float
    resolution: float
    origine: tuple[float, float]
    largeur: int
    hauteur: int
    rect: tuple[int, int, int, int]
    """(colonne, ligne, largeur, hauteur) du rectangle transmis."""
    donnees: str


class ChatReponse(_Evenement):
    type: Literal["chat_reponse"] = "chat_reponse"
    t: float
    texte: str
    intention: dict[str, object] | None = None
    refus: str | None = None
    fournisseur: str
    duree_ms: float | None = None


class ObjetPlace(_Evenement):
    categorie: str
    nom: str
    boite: tuple[float, float, float, float]
    x: float
    y: float
    distance: float
    incertitude: float
    bloque: bool
    confiance: float
    dans_carte: bool


class ObstacleVision(_Evenement):
    id: int
    categorie: str
    nom: str
    contour: list[tuple[float, float]]
    """Emprise au sol (quadrilatère convexe)."""
    marge: float
    t: float


class Vision(_Evenement):
    """Résultat d'une analyse de la caméra, avec l'image analysée (preuve) et la couche vision."""

    type: Literal["vision"] = "vision"
    t: float
    demande: int
    raison: str
    t_image: float
    fournisseur: str
    duree_ms: float
    resume: str | None
    erreur: str | None
    objets: list[ObjetPlace]
    obstacles: list[ObstacleVision]
    image: str
    """JPEG en base64 de l'image analysée (vide si non transmise)."""


class EtatVision(_Evenement):
    type: Literal["etat_vision"] = "etat_vision"
    t: float
    disponible: bool
    vigilance: bool
    en_cours: bool
    analyses: int
    message: str


class Mesures(_Evenement):
    """Indicateurs du cerveau, mesurés et non estimés."""

    type: Literal["mesures"] = "mesures"
    t: float
    valeurs: dict[str, float | int | None]


class NoteIA(_Evenement):
    """Note de l'IA d'observation : ce que voit le rover, et sa décision une fois validée."""

    type: Literal["note_ia"] = "note_ia"
    t: float
    t_image: float
    x: float
    y: float
    texte: str
    decision: Literal["continuer", "ralentir", "s_arreter"]
    raison: str
    appliquee: bool
    """La décision a été appliquée (une décision hors mission n'a pas d'effet)."""
    reperes: list[str] = Field(default_factory=list)
    """Éléments notables vus (personne, animal, véhicule…)."""
    fournisseur: str


class Traversabilite(_Evenement):
    """Rectangle modifié de la carte de traversabilité (caméra stéréo), octets zlib en base64.

    0 inconnu, 1 libre, 2 obstacle (marche : tronc, rocher, souche), 3 obstacle (pente trop forte),
    4 trou probable (zone cachée derrière un bord de sol : fossé, berge).
    Même géométrie que la grille d'occupation.
    """

    type: Literal["traversabilite"] = "traversabilite"
    t: float
    resolution: float
    origine: tuple[float, float]
    largeur: int
    hauteur: int
    rect: tuple[int, int, int, int]
    donnees: str


class Profondeur(_Evenement):
    """Dernière carte de profondeur de la paire stéréo, en couleurs (JPEG base64)."""

    type: Literal["profondeur"] = "profondeur"
    t: float
    image: str
    portee: float
    duree_ms: float
    points: int
    obstacles_devant: float | None
    """Distance libre devant le rover d'après la caméra (m), ou None si rien jusqu'à 4 m."""


EvenementCerveau = (
    Statut
    | Journal
    | PoseEstimee
    | Chemin
    | Carte
    | ChatReponse
    | Mesures
    | Vision
    | EtatVision
    | Traversabilite
    | Profondeur
    | NoteIA
)
