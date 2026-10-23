"""Agent : consignes françaises → plans validés, refus hors bornes, arrêt sans modèle, phrases
ignorées, absence de modèle. Le « modèle » est un faux serveur aux réponses écrites d'avance."""

from __future__ import annotations

import asyncio
from typing import Any

import pytest
from test_ia_faux_serveur import Ecoute, FauxOutils, Panne, appels, config_essai, faux_serveur

from fieldmind.bus import Bus
from fieldmind.config import FICHIER_PROFIL, Profil
from fieldmind.ia.agent import Agent, detecter_arret, est_bruit
from fieldmind.ia.vision import Vision


class ModeleScripte:
    """Faux modèle : la dernière consigne de l'opérateur → réponse écrite d'avance."""

    def __init__(self, reponses: dict[str, Any]) -> None:
        self.reponses = reponses
        self.vus: list[dict[str, Any]] = []

    def __call__(self, corps: dict[str, Any]) -> Any:
        self.vus.append(corps)
        consigne = next(m["content"] for m in reversed(corps["messages"]) if m["role"] == "user")
        reponse = self.reponses[consigne]
        if isinstance(reponse, str):
            return {"content": reponse}
        if isinstance(reponse, Panne):
            return reponse
        return appels(*reponse)


def profil_rover() -> Profil:
    return Profil.charger(FICHIER_PROFIL, reglages=None)


CONSIGNES: list[tuple[str, list[tuple[str, dict[str, Any]]], list[dict[str, Any]], bool]] = [
    ("explore la pièce", [("explorer", {})], [{"type": "explorer"}], True),
    ("explore la pièce pour trouver une porte", [("explorer", {"objet": "porte"})],
     [{"type": "explorer", "objet": "porte"}], True),
    ("avance de 5 mètres", [("avancer", {"distance_m": 5})],
     [{"type": "avancer", "distance_m": 5.0}], True),
    ("recule de 50 centimètres", [("avancer", {"distance_m": -0.5})],
     [{"type": "avancer", "distance_m": -0.5}], True),
    ("tourne à droite", [("tourner", {"angle_deg": -90})],
     [{"type": "tourner", "angle_deg": -90.0}], True),
    ("avance de 5 m puis tourne à gauche et cherche la chaise",
     [("avancer", {"distance_m": 5}), ("tourner", {"angle_deg": 90}),
      ("chercher", {"objet": "chaise"})],
     [{"type": "avancer", "distance_m": 5.0}, {"type": "tourner", "angle_deg": 90.0},
      {"type": "chercher", "objet": "chaise"}], True),
    ("fais demi-tour et reviens au point de départ",
     [("tourner", {"angle_deg": 180}), ("retour_depart", {})],
     [{"type": "tourner", "angle_deg": 180.0}, {"type": "retour_depart"}], True),
    ("va à la porte", [("aller_lieu", {"nom": "porte"})],
     [{"type": "aller_lieu", "nom": "porte"}], True),
    ("mémorise cet endroit comme le garage", [("memoriser_lieu", {"nom": "garage"})],
     [{"type": "memoriser_lieu", "nom": "garage"}], True),
    ("retourne au garage", [("aller_lieu", {"nom": "le Garage"})],
     [{"type": "aller_lieu", "nom": "garage"}], True),
    ("va à la cuisine", [("aller_lieu", {"nom": "cuisine"})],
     [{"type": "rejoindre_piece", "nom": "cuisine"}], True),
    ("va au point 43.7105, -1.0531", [("aller_gps", {"lat": 43.7105, "lon": -1.0531})],
     [{"type": "aller_gps", "lat": 43.7105, "lon": -1.0531}], True),
    ("va au point GPS 43.71052 -1.05318 à 1 mètre près",
     [("aller_gps", {"lat": 43.71052, "lon": -1.05318, "tolerance_m": 1})],
     [{"type": "aller_gps", "lat": 43.71052, "lon": -1.05318, "tolerance_m": 1.0}], True),
    ("va 3 mètres devant toi et un mètre à droite",
     [("aller_a", {"x": 3, "y": -1, "repere": "robot"})],
     [{"type": "aller_a", "x": 3.0, "y": -1.0, "repere": "robot"}], True),
    ("attends dix secondes puis prends une photo avec la caméra droite",
     [("attendre", {"duree_s": 10}), ("photo", {"camera": "droite"})],
     [{"type": "attendre", "duree_s": 10.0}, {"type": "photo", "camera": "droite"}], True),
    ("oriente-toi au cap 90 degrés", [("cap", {"cap_deg": 90})],
     [{"type": "cap", "cap_deg": 90.0}], True),
    ("avance doucement de 2 mètres, à 20 centimètres par seconde",
     [("avancer", {"distance_m": 2, "vitesse_m_s": 0.2})],
     [{"type": "avancer", "distance_m": 2.0, "vitesse_m_s": 0.2}], True),
    ("ensuite, avance de 2 mètres et arrête-toi",
     [("mettre_a_la_suite", {}), ("avancer", {"distance_m": 2}), ("stop", {})],
     # « stop » après d'autres actions est retiré : dans la file, il annulerait tout le plan
     [{"type": "avancer", "distance_m": 2.0}], False),
    ("cherche le ballon et décris ce que tu vois",
     [("chercher", {"objet": "ballon", "camera": "gauche"}), ("decrire", {})],
     [{"type": "chercher", "objet": "ballon", "camera": "gauche"}, {"type": "decrire"}], True),
]


@pytest.mark.parametrize(("consigne", "proposition", "attendu", "remplacer"), CONSIGNES,
                         ids=[c[0] for c in CONSIGNES])
async def test_consignes_vers_plans(consigne: str, proposition: list[tuple[str, dict[str, Any]]],
                                    attendu: list[dict[str, Any]], remplacer: bool) -> None:
    async with faux_serveur() as fs:
        modele = ModeleScripte({consigne: proposition})
        fs.chat = modele
        bus, outils = Bus(), FauxOutils()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        agent = Agent(bus, profil_rover(), config, outils)
        resultat = await agent.consigne(consigne)
        await config.passerelle().fermer()
    assert resultat.statut == "plan", resultat
    assert outils.plans == [(attendu, remplacer)]
    # Réponse composée par le code, avec le plan, au format du message `chat`
    reponse = ecoute.chat[-1]
    assert reponse["type"] == "chat" and reponse["role"] == "rover"
    assert reponse["plan"] == {"id": "p1", "actions": attendu}
    assert reponse["texte"].endswith(".")
    assert ecoute.chat[0]["role"] == "operateur" and ecoute.chat[0]["texte"] == consigne
    # Requête : outils proposés, état du rover dans la consigne système
    corps = modele.vus[0]
    noms = {o["function"]["name"] for o in corps["tools"]}
    assert {"avancer", "tourner", "aller_a", "cap", "attendre", "retour_depart", "aller_gps",
            "memoriser_lieu", "aller_lieu", "chercher", "decrire", "photo", "stop",
            "ignorer"} <= noms
    aller_lieu = next(o["function"] for o in corps["tools"]
                      if o["function"]["name"] == "aller_lieu")
    assert aller_lieu["parameters"]["properties"]["nom"]["enum"] == ["garage", "porte"]
    systeme = corps["messages"][0]["content"]
    assert '"lieux":[{"nom":"garage","gps":true}' in systeme
    assert '"gps":{"aligne":true,"fix":"autonome"}' in systeme


async def test_texte_du_plan_compose_par_le_code() -> None:
    async with faux_serveur() as fs:
        consigne = "avance de 5 m puis tourne à gauche et cherche la chaise"
        fs.chat = ModeleScripte({consigne: [
            ("avancer", {"distance_m": 5}), ("tourner", {"angle_deg": 90}),
            ("chercher", {"objet": "chaise"})]})
        bus = Bus()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        await Agent(bus, profil_rover(), config, FauxOutils()).consigne(consigne, "voix")
        await config.passerelle().fermer()
    assert ecoute.textes()[-1] == ("J'avance de 5 m, puis je tourne de 90° à gauche, puis je "
                                   "cherche « chaise ».")
    assert ecoute.chat[-1]["origine"] == "voix"
    # dite : la phrase de l'opérateur apparaît juste avant la réponse
    assert [m["role"] for m in ecoute.chat] == ["operateur", "rover"]
    assert ecoute.chat[0]["texte"] == consigne and ecoute.chat[0]["origine"] == "voix"


HORS_BORNES = [
    ("avance de 50 mètres", [("avancer", {"distance_m": 50})], "hors limites"),
    ("fais deux tours sur toi-même", [("tourner", {"angle_deg": 720})], "hors limites"),
    ("fonce à 3 m/s sur 2 m", [("avancer", {"distance_m": 2, "vitesse_m_s": 3})],
     "hors limites"),
    ("va à 100 mètres devant", [("aller_a", {"x": 100, "y": 0})], "hors limites"),
    ("va 25 m devant et 25 m à gauche", [("aller_a", {"x": 25, "y": 25})], "trop loin"),
    ("va à la porte du garage", [("aller_a", {"lieu": "porte"})], "paramètre inconnu"),
    ("va au point 91, 2", [("aller_gps", {"lat": 91, "lon": 2})], "hors limites"),
    ("va au point 43.7, -181", [("aller_gps", {"lat": 43.7, "lon": -181})], "hors limites"),
    ("va au point 0, 0", [("aller_gps", {"lat": 0, "lon": 0})], "invalides"),
    ("va au point 43.7105, -1.0531 au centimètre près",
     [("aller_gps", {"lat": 43.7105, "lon": -1.0531, "tolerance_m": 0.01})], "hors limites"),
    ("mémorise cet endroit", [("memoriser_lieu", {})], "manquant"),
    ("mémorise cet endroit comme la grande allée des chênes centenaires du parc",
     [("memoriser_lieu", {"nom": "grande allée des chênes centenaires du parc"})], "trop long"),
    ("attends une heure", [("attendre", {"duree_s": 3600})], "hors limites"),
    ("avance de cinq", [("avancer", {"distance_m": "5"})], "nombre"),
    ("avance de 2 m en volant", [("avancer", {"distance_m": 2, "voler": True})],
     "paramètre inconnu"),
    ("saute", [("sauter", {"hauteur_m": 1})], "action inconnue"),
    ("photo infrarouge", [("photo", {"camera": "infrarouge"})], "caméra"),
    ("avance de 2 m puis de 40 m", [("avancer", {"distance_m": 2}),
                                    ("avancer", {"distance_m": 40})], "hors limites"),
]


@pytest.mark.parametrize(("consigne", "proposition", "motif"), HORS_BORNES,
                         ids=[c[0] for c in HORS_BORNES])
async def test_refus_hors_bornes_rien_execute(consigne: str,
                                              proposition: list[tuple[str, dict[str, Any]]],
                                              motif: str) -> None:
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({consigne: proposition})
        bus, outils = Bus(), FauxOutils()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        resultat = await Agent(bus, profil_rover(), config, outils).consigne(consigne)
        await config.passerelle().fermer()
    assert resultat.statut == "refus"
    assert motif in resultat.texte
    assert outils.plans == [] and outils.arrets == []
    assert ecoute.textes()[-1].startswith("Je ne peux pas")


async def test_trop_d_actions_refuse() -> None:
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({"danse": [("tourner", {"angle_deg": 30})] * 13})
        outils = FauxOutils()
        config = config_essai(fs.base)
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne("danse")
        await config.passerelle().fermer()
    assert resultat.statut == "refus" and "trop long" in resultat.texte
    assert outils.plans == []


async def test_rover_non_arme_refus_de_la_mission() -> None:
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({"avance de 1 m": [("avancer", {"distance_m": 1})]})
        bus, outils = Bus(), FauxOutils(arme=False)
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        resultat = await Agent(bus, profil_rover(), config, outils).consigne("avance de 1 m")
        await config.passerelle().fermer()
    assert resultat.statut == "refus"
    assert ecoute.textes()[-1] == "Refusé : rover non armé."


async def test_exception_de_la_mission_non_convertie() -> None:
    class AutreRefus(Exception):
        def __init__(self, raison: str) -> None:
            super().__init__(raison)
            self.raison = raison

    class Outils(FauxOutils):
        def ajouter_actions(self, actions: list[dict[str, Any]], remplacer: bool) -> str:
            raise AutreRefus("file pleine")

    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({"avance de 1 m": [("avancer", {"distance_m": 1})]})
        bus = Bus()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        resultat = await Agent(bus, profil_rover(), config, Outils()).consigne("avance de 1 m")
        await config.passerelle().fermer()
    assert resultat.statut == "refus"
    assert ecoute.textes()[-1] == "Refusé : file pleine."


async def test_texte_des_actions_gps_et_lieux() -> None:
    async with faux_serveur() as fs:
        consigne = "mémorise cet endroit comme l'abri puis va au point 43.7105, -1.0531"
        fs.chat = ModeleScripte({consigne: [
            ("memoriser_lieu", {"nom": "abri"}),
            ("aller_gps", {"lat": 43.7105, "lon": -1.0531, "tolerance_m": 2.5})]})
        bus = Bus()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        await Agent(bus, profil_rover(), config, FauxOutils()).consigne(consigne)
        await config.passerelle().fermer()
    assert ecoute.textes()[-1] == ("Je mémorise cet endroit comme « abri », puis je vais au "
                                   "point GPS 43.71050, -1.05310 (à 2,5 m près).")


async def test_aller_gps_refuse_sans_fix() -> None:
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({"va au point 43.7105, -1.0531": [
            ("aller_gps", {"lat": 43.7105, "lon": -1.0531})]})
        outils = FauxOutils()
        outils.gps = {"aligne": False, "fix": "aucun"}
        config = config_essai(fs.base)
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(
            "va au point 43.7105, -1.0531")
        await config.passerelle().fermer()
    assert resultat.statut == "refus" and "pas de position GPS" in resultat.texte
    assert outils.plans == []


async def test_sans_lieu_memorise_pas_d_aller_lieu() -> None:
    async with faux_serveur() as fs:
        modele = ModeleScripte({"retourne au garage": "Je ne connais pas de lieu « garage »."})
        fs.chat = modele
        outils = FauxOutils()
        outils.lieux = []
        config = config_essai(fs.base)
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(
            "retourne au garage")
        await config.passerelle().fermer()
    noms = {o["function"]["name"] for o in modele.vus[0]["tools"]}
    assert "aller_lieu" not in noms and "memoriser_lieu" in noms
    assert resultat.statut == "reponse" and outils.plans == []


# ------------------------------------------------------------------ arrêt sans modèle


@pytest.mark.parametrize("phrase", ["stop", "Stop !", "arrête", "Arrête-toi !", "stoppe",
                                    "halte", "on s'arrête tout de suite", "rover, stop",
                                    "ARRÊTEZ", "stop stop stop", "freine"])
async def test_arret_sans_modele(phrase: str) -> None:
    bus, outils = Bus(), FauxOutils()
    ecoute = Ecoute(bus)
    # Profil « aucun » : aucun modèle ne peut être appelé, l'arrêt marche quand même.
    async with faux_serveur() as fs:
        config = config_essai(fs.base, profil="aucun")
        resultat = await Agent(bus, profil_rover(), config, outils).consigne(phrase, "voix")
    assert resultat.statut == "arret"
    assert len(outils.arrets) == 1
    assert fs.requetes == []
    assert ecoute.textes()[-1] == "Je m'arrête."


@pytest.mark.parametrize("phrase", ["ne t'arrête pas", "va jusqu'à l'arrêt de bus",
                                    "avance de deux mètres", "n'arrête pas de tourner"])
def test_pas_un_arret(phrase: str) -> None:
    assert detecter_arret(phrase) == (False, "")


def test_arret_puis_suite() -> None:
    assert detecter_arret("stop, recule de 1 mètre") == (True, "recule de 1 mètre")
    assert detecter_arret("arrête de tourner") == (True, "")
    assert detecter_arret("Arrête-toi et reviens au départ") == (True, "reviens au départ")


async def test_arret_puis_consigne_suivante() -> None:
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({"recule de 1 mètre": [("avancer", {"distance_m": -1})]})
        outils = FauxOutils()
        config = config_essai(fs.base)
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(
            "stop, recule de 1 mètre")
        await config.passerelle().fermer()
    assert len(outils.arrets) == 1
    assert resultat.statut == "plan"
    assert outils.plans == [([{"type": "avancer", "distance_m": -1.0}], True)]


async def test_arret_pendant_l_analyse_abandonne_le_plan() -> None:
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({"avance de 5 mètres": [("avancer", {"distance_m": 5})]})
        fs.delai_s = 0.4
        bus, outils = Bus(), FauxOutils()
        config = config_essai(fs.base)
        agent = Agent(bus, profil_rover(), config, outils)
        tache = asyncio.create_task(agent.consigne("avance de 5 mètres"))
        await asyncio.sleep(0.1)
        assert agent.occupee
        arret = await agent.consigne("stop")  # tout de suite, sans attendre le modèle
        assert arret.statut == "arret" and len(outils.arrets) == 1
        resultat = await tache
        await config.passerelle().fermer()
    assert resultat.statut == "abandonne"
    assert outils.plans == []


async def test_stop_propose_par_le_modele_arrete() -> None:
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({"bon, on en reste là": [("stop", {})]})
        outils = FauxOutils()
        config = config_essai(fs.base)
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(
            "bon, on en reste là")
        await config.passerelle().fermer()
    assert resultat.statut == "arret" and outils.arrets and outils.plans == []


# ------------------------------------------------------------------ pas une consigne


async def test_phrase_ignoree_par_le_modele() -> None:
    phrase = "tu as vu le match hier soir"
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({phrase: [("ignorer", {"raison": "conversation à côté"})]})
        bus, outils = Bus(), FauxOutils()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        resultat = await Agent(bus, profil_rover(), config, outils).consigne(phrase, "voix")
        await config.passerelle().fermer()
    assert resultat.statut == "ignore"
    assert ecoute.voix[-1]["evenement"] == "ignore"
    assert ecoute.voix[-1]["raison"] == "conversation à côté"
    assert ecoute.chat == [] and outils.plans == []  # rien dans la conversation


@pytest.mark.parametrize("phrase", ["euh", "hum hum", "", "Sous-titres réalisés par la "
                                    "communauté d'Amara.org"])
async def test_bruit_ignore_sans_modele(phrase: str) -> None:
    bus = Bus()
    ecoute = Ecoute(bus)
    async with faux_serveur() as fs:
        config = config_essai(fs.base)
        resultat = await Agent(bus, profil_rover(), config, FauxOutils()).consigne(phrase, "voix")
    assert resultat.statut == "ignore"
    assert fs.requetes == []
    assert ecoute.chat == []
    assert est_bruit(phrase) is not None


async def test_question_reponse_sans_action() -> None:
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({"où es-tu ?": "Je suis au point de départ, à l'arrêt."})
        bus, outils = Bus(), FauxOutils()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        resultat = await Agent(bus, profil_rover(), config, outils).consigne("où es-tu ?")
        await config.passerelle().fermer()
    assert resultat.statut == "reponse"
    assert ecoute.textes()[-1] == "Je suis au point de départ, à l'arrêt."
    assert outils.plans == []


async def test_annuler() -> None:
    async with faux_serveur() as fs:
        fs.chat = ModeleScripte({"laisse tomber la mission": [("annuler", {})]})
        outils = FauxOutils()
        config = config_essai(fs.base)
        resultat = await Agent(Bus(), profil_rover(), config, outils).consigne(
            "laisse tomber la mission")
        await config.passerelle().fermer()
    assert resultat.statut == "annule" and outils.annulations == 1


async def test_historique_court_redonne_au_modele() -> None:
    async with faux_serveur() as fs:
        modele = ModeleScripte({"avance de 2 m": [("avancer", {"distance_m": 2})],
                                "encore": [("avancer", {"distance_m": 2})]})
        fs.chat = modele
        config = config_essai(fs.base)
        agent = Agent(Bus(), profil_rover(), config, FauxOutils())
        await agent.consigne("avance de 2 m")
        await agent.consigne("encore")
        await config.passerelle().fermer()
    messages = modele.vus[1]["messages"]
    assert [m["role"] for m in messages] == ["system", "user", "assistant", "user"]
    assert messages[1]["content"] == "avance de 2 m"
    assert messages[2]["content"] == "J'avance de 2 m."


# ------------------------------------------------------------------ sans modèle disponible


async def test_sans_modele_message_clair_rien_execute() -> None:
    bus, outils = Bus(), FauxOutils()
    ecoute = Ecoute(bus)
    async with faux_serveur() as fs:
        config = config_essai(fs.base, profil="aucun")
        resultat = await Agent(bus, profil_rover(), config, outils).consigne("avance de 5 m")
    assert resultat.statut == "indisponible"
    assert ecoute.chat[-1]["role"] == "systeme"
    assert "aucun modèle" in ecoute.chat[-1]["texte"]
    assert outils.plans == [] and fs.requetes == []


async def test_sans_cle_message_clair() -> None:
    async with faux_serveur() as fs:
        bus, outils = Bus(), FauxOutils()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base, env={})
        resultat = await Agent(bus, profil_rover(), config, outils).consigne("avance de 5 m")
    assert resultat.statut == "indisponible"
    assert "GEMINI_API_KEY" in ecoute.chat[-1]["texte"]
    assert fs.requetes == []


async def test_fournisseur_en_panne_rien_execute() -> None:
    async with faux_serveur() as fs:
        fs.pannes = [Panne(503), Panne(503)]
        bus, outils = Bus(), FauxOutils()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        resultat = await Agent(bus, profil_rover(), config, outils).consigne("avance de 5 m")
        await config.passerelle().fermer()
    assert resultat.statut == "erreur"
    assert "en panne" in ecoute.chat[-1]["texte"]
    assert outils.plans == []


async def test_quota_atteint_rien_execute() -> None:
    async with faux_serveur() as fs:
        def retouche(d: dict[str, Any]) -> None:
            d["profils"]["gemini"]["agent"]["requetes_par_jour"] = 1

        fs.chat = ModeleScripte({"avance de 1 m": [("avancer", {"distance_m": 1})]})
        outils = FauxOutils()
        bus = Bus()
        ecoute = Ecoute(bus)
        config = config_essai(fs.base, retouche=retouche)
        agent = Agent(bus, profil_rover(), config, outils)
        assert (await agent.consigne("avance de 1 m")).statut == "plan"
        resultat = await agent.consigne("avance de 1 m")
        await config.passerelle().fermer()
    assert resultat.statut == "erreur"
    assert "quota journalier" in ecoute.chat[-1]["texte"]
    assert len(outils.plans) == 1 and len(fs.requetes) == 1


async def test_plan_de_vision_execute_sans_mouvement() -> None:
    """« Décris ce que tu vois » : vision tout de suite, même rover désarmé."""
    async with faux_serveur() as fs:
        def chat(corps: dict[str, Any]) -> Any:
            if corps["model"] == "gemini-4-argon":
                return appels(("decrire", {"question": "y a-t-il quelqu'un ?"}))
            return {"content": "Personne en vue. Un couloir dégagé devant."}

        fs.chat = chat
        bus, outils = Bus(), FauxOutils(arme=False)
        ecoute = Ecoute(bus)
        config = config_essai(fs.base)
        vision = Vision(bus, profil_rover(), config, outils)
        agent = Agent(bus, profil_rover(), config, outils, vision=vision)
        resultat = await agent.consigne("y a-t-il quelqu'un devant toi ?")
        await config.passerelle().fermer()
    assert resultat.statut == "vision"
    assert outils.plans == []
    assert ecoute.vision[-1]["description"] == "Personne en vue. Un couloir dégagé devant."
    assert "y a-t-il quelqu'un ?" in fs.requetes[-1][1]["messages"][0]["content"][1]["text"]
