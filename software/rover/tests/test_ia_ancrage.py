"""Les décisions moteur restent ancrées dans les demandes, jamais dans une relance GX10."""

import pytest
from test_ia_agent import ModeleScripte, profil_rover
from test_ia_faux_serveur import FauxOutils, appels, config_essai, faux_serveur

from fieldmind.bus import Bus
from fieldmind.ia.agent import Agent, _normaliser
from fieldmind.ia.outils import (
    ActionInvalide,
    ContexteValidation,
    schemas_fonctions,
    valider_action,
)

ANCIEN_PLAN_CUISINE = [
    ("chercher", {"objet": "cuisine"}),
    ("aller_a", {"x": 3.1, "y": 2.1}),
    ("avancer", {"distance_m": 2}),
    ("avancer", {"distance_m": 1}),
    ("tourner", {"angle_deg": 45}),
    ("avancer", {"distance_m": 2}),
]


@pytest.mark.parametrize("piece", ["cuisine", "salon", "chambre", "salle de bain",
                                  "entrée", "bureau", "salle à manger", "buanderie"])
async def test_piece_ne_devient_jamais_un_point_invente(piece: str) -> None:
    phrase = f"va à la {piece}"
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({phrase: ANCIEN_PLAN_CUISINE})
        config = config_essai(fs.base, profil="gx10")
        outils = FauxOutils()
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(phrase)
        await config.passerelle().fermer()
    assert resultat.statut == "plan", resultat.texte
    assert outils.plans == [([{"type": "rejoindre_piece", "nom": _normaliser(piece)}], True)]


async def test_relance_gx10_ne_continue_pas_une_mission_simple() -> None:
    async with faux_serveur() as fs:
        vus = []

        def modele(corps):
            vus.append(corps)
            return appels(*ANCIEN_PLAN_CUISINE[min(len(vus) - 1, 5):len(vus)])

        fs.chat = modele
        config = config_essai(fs.base, profil="gx10")
        outils = FauxOutils()
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne("va à la cuisine")
        await config.passerelle().fermer()
    assert resultat.actions == [{"type": "rejoindre_piece", "nom": "cuisine"}]
    assert len(vus) == 1


@pytest.mark.parametrize("phrase", ["où est la cuisine ?", "où se trouve le salon ?",
                                    "sais-tu où est la chambre ?", "dis-moi où est le bureau",
                                    "ne va pas à la cuisine", "ne rejoins jamais le salon",
                                    "pas dans la cuisine", "je me demande où est la cuisine",
                                    "si tu trouves la cuisine, va à la cuisine"])
async def test_question_et_negation_ne_donnent_pas_permission_de_bouger(phrase: str) -> None:
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({phrase: ANCIEN_PLAN_CUISINE})
        config = config_essai(fs.base, profil="gx10")
        outils = FauxOutils()
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(phrase)
        await config.passerelle().fermer()
    assert resultat.statut == "refus"
    assert outils.plans == [] and outils.arrets == []


async def test_piece_composee_conserve_seulement_les_etapes_demandees() -> None:
    phrase = "avance de 2 mètres puis va à la cuisine et tourne à gauche"
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({phrase: [
            ("avancer", {"distance_m": 2}),
            ("chercher", {"objet": "cuisine"}),
            ("aller_a", {"x": 3.1, "y": 2.1}),
            ("tourner", {"angle_deg": 90}),
            ("avancer", {"distance_m": 1}),
        ]})
        config = config_essai(fs.base, profil="gx10")
        outils = FauxOutils()
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(phrase)
        await config.passerelle().fermer()
    assert resultat.actions == [{"type": "avancer", "distance_m": 2.0},
                                {"type": "rejoindre_piece", "nom": "cuisine"},
                                {"type": "tourner", "angle_deg": 90.0}]


@pytest.mark.parametrize("phrase,proposition", [
    ("explore le jardin", [("explorer", {}), ("avancer", {"distance_m": 2})]),
    ("cherche la caisse rouge", [("chercher", {"objet": "caisse rouge"}),
                                ("tourner", {"angle_deg": 45})]),
    ("cherche la caisse rouge", [("aller_a", {"x": 2, "y": 1})]),
    ("explore le jardin", [("memoriser_lieu", {"nom": "jardin"}), ("explorer", {})]),
])
async def test_missions_autonomes_refusent_etapes_inventees(phrase, proposition) -> None:
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({phrase: proposition})
        config = config_essai(fs.base, profil="gx10")
        outils = FauxOutils()
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(phrase)
        await config.passerelle().fermer()
    assert resultat.statut == "refus"
    assert outils.plans == []


async def test_distance_demandee_ne_peut_pas_etre_remplacee_par_un_chiffre_du_modele() -> None:
    phrase = "cherche la caisse puis avance de deux mètres"
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({phrase: [("chercher", {"objet": "caisse"}),
                                        ("avancer", {"distance_m": 7})]})
        config = config_essai(fs.base, profil="gx10")
        outils = FauxOutils()
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(phrase)
        await config.passerelle().fermer()
    assert resultat.statut == "refus" and outils.plans == []


async def test_question_sur_objet_reste_une_lecture_sans_mouvement() -> None:
    phrase = "où est la caisse rouge ?"
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({phrase: [("chercher", {"objet": "caisse rouge", "aller": False})]})
        config = config_essai(fs.base, profil="gx10")
        outils = FauxOutils()
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(phrase)
        await config.passerelle().fermer()
    assert resultat.actions == [{"type": "chercher", "objet": "caisse rouge", "aller": False}]


async def test_caisse_photo_retour_reste_une_vraie_mission_composee() -> None:
    phrase = "trouve la caisse rouge puis prends une photo puis reviens au départ"
    sequence = [("chercher", {"objet": "caisse rouge"}), ("photo", {}), ("retour_depart", {})]
    async with faux_serveur() as fs:
        vus = []

        def modele(corps):
            vus.append(corps)
            return appels(sequence[len(vus) - 1])

        fs.chat = modele
        config = config_essai(fs.base, profil="gx10")
        outils = FauxOutils()
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(phrase)
        await config.passerelle().fermer()
    assert resultat.statut == "plan", resultat.texte
    assert resultat.actions == [{"type": "chercher", "objet": "caisse rouge"},
                                {"type": "photo", "camera": "gauche"},
                                {"type": "retour_depart"}]
    assert len(vus) == 3


@pytest.mark.parametrize("retour,appel_retour,action_retour", [
    ("reviens à la base", ("aller_lieu", {"nom": "base"}),
     {"type": "aller_lieu", "nom": "base"}),
    ("reviens au départ", ("retour_depart", {}), {"type": "retour_depart"}),
])
async def test_base_memorisee_et_depart_sont_deux_destinations_distinctes(
        retour, appel_retour, action_retour) -> None:
    phrase = f"trouve la caisse rouge puis prends une photo puis {retour}"
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({phrase: [("chercher", {"objet": "caisse rouge"}),
                                        ("photo", {}), appel_retour]})
        config = config_essai(fs.base, profil="gx10")
        outils = FauxOutils()
        outils.lieux.append({"nom": "base", "gps": False})
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(phrase)
        await config.passerelle().fermer()
    assert resultat.statut == "plan", resultat.texte
    assert resultat.actions == [{"type": "chercher", "objet": "caisse rouge"},
                                {"type": "photo", "camera": "gauche"}, action_retour]


@pytest.mark.parametrize("phrase,proposition,attendu", [
    ("avance de 2 m puis tourne à gauche", [("avancer", {"distance_m": 2}),
                                          ("tourner", {"angle_deg": 90})],
     [{"type": "avancer", "distance_m": 2.0}, {"type": "tourner", "angle_deg": 90.0}]),
    ("retourne au garage", [("aller_lieu", {"nom": "garage"})],
     [{"type": "aller_lieu", "nom": "garage"}]),
    ("mémorise cet endroit comme le garage", [("memoriser_lieu", {"nom": "garage"})],
     [{"type": "memoriser_lieu", "nom": "garage"}]),
])
async def test_compatibilite_commandes_mesurees_et_lieux_explicites(
        phrase, proposition, attendu) -> None:
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({phrase: proposition})
        config = config_essai(fs.base)
        outils = FauxOutils()
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(phrase)
        await config.passerelle().fermer()
    assert resultat.actions == attendu


def test_rejoindre_piece_est_disponible_sans_lieux_et_refuse_toute_coordonnee() -> None:
    noms = {o["function"]["name"] for o in schemas_fonctions(lieux=())}
    assert "rejoindre_piece" in noms
    assert valider_action("rejoindre_piece", {"nom": "la cuisine"}, ContexteValidation()) == {
        "type": "rejoindre_piece", "nom": "cuisine"}
    for args in ({"nom": "cuisine", "x": 2, "y": 1}, {"nom": "cuisine (3, 2)"},
                 {"nom": 2}, {}, {"nom": ""}):
        with pytest.raises(ActionInvalide):
            valider_action("rejoindre_piece", args, ContexteValidation())
