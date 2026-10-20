"""IA d'observation : notes, décisions proposées par le modèle et validées par le cerveau."""

from __future__ import annotations

from pathlib import Path

from services.cerveau.cerveau import Cerveau
from services.cerveau.evenements import NoteIA
from services.perception.vision import AnalyseVision, DemandeVision, ResultatVision
from services.protocole.fiche_rover import FicheRover
from services.protocole.scenario import ScenarioPublic

RACINE = Path(__file__).resolve().parents[1]
FICHE = FicheRover.charger(RACINE / "config" / "rover_fieldmind.json")
SCENARIO = ScenarioPublic.charger(RACINE / "simulation" / "scenarios" / "foret_demo.json")


def _cerveau_en_mission() -> Cerveau:
    cerveau = Cerveau(FICHE, SCENARIO)
    cerveau.vision_disponible = True
    lieu = SCENARIO.lieu("clairiere")
    assert lieu is not None
    assert cerveau.definir_objectif(lieu.x, lieu.y) is None
    return cerveau


def _resultat(decision: str, raison: str = "") -> tuple[DemandeVision, ResultatVision]:
    demande = DemandeVision(1, 0.0, "", 640, 360, -29.0, -29.2, 0.7, "vigilance", "contexte")
    analyse = AnalyseVision(
        resume="Une personne marche sur la piste.",
        decision=decision,  # type: ignore[arg-type]
        raison_decision=raison,
    )
    return demande, ResultatVision(analyse, "faux modèle", 10.0)


def test_arret_decide_par_l_ia_puis_reprise() -> None:
    cerveau = _cerveau_en_mission()
    cerveau.sur_resultat_vision(*_resultat("s_arreter", "personne sur la piste"))
    assert cerveau.mission is None
    assert cerveau.etat == "attente"
    notes = [e for e in cerveau.vider_sortie() if isinstance(e, NoteIA)]
    assert notes
    assert notes[-1].appliquee
    assert notes[-1].decision == "s_arreter"
    assert cerveau.reprendre() is None, "l'opérateur peut reprendre la mission"


def test_ralentir_plafonne_la_vitesse() -> None:
    cerveau = _cerveau_en_mission()
    cerveau.sur_resultat_vision(*_resultat("ralentir", "terrain encombré"))
    assert cerveau.mission is not None
    assert cerveau._ralenti_jusqu_a > cerveau.t


def test_hors_mission_la_decision_n_a_pas_d_effet() -> None:
    cerveau = Cerveau(FICHE, SCENARIO)
    cerveau.vision_disponible = True
    cerveau.sur_resultat_vision(*_resultat("s_arreter", "personne"))
    notes = [e for e in cerveau.vider_sortie() if isinstance(e, NoteIA)]
    assert notes
    assert not notes[-1].appliquee


def test_recherche_arretee_quand_la_cible_est_vue() -> None:
    from services.cerveau.evenements import ChatReponse
    from services.mission_planner.intentions import Chercher, Intention, PoseCourante, traduire
    from services.perception.vision import ObjetVu

    cerveau = Cerveau(FICHE, SCENARIO)
    cerveau.vision_disponible = True
    action, texte = traduire(
        Intention(action="chercher", cible="tronc_couche"), SCENARIO, PoseCourante(-29, -29.2, 0.7)
    )
    assert isinstance(action, Chercher)
    assert "tronc couché" in texte
    assert cerveau.executer(action) is None
    assert cerveau.recherche is not None
    assert "trouver un tronc couché" in cerveau._contexte_mission()
    demande = DemandeVision(1, 0.0, "", 640, 360, -29.0, -29.2, 0.7, "vigilance")
    tronc = ObjetVu(
        categorie="tronc_couche", boite=(300, 600, 700, 700), bloque_passage=True, confiance=0.9
    )
    analyse = AnalyseVision(resume="Un tronc couché barre la piste.", objets=[tronc])
    cerveau.sur_resultat_vision(demande, ResultatVision(analyse, "faux modèle", 10.0))
    assert cerveau.mission is None
    assert cerveau.recherche is None
    reponses = [e for e in cerveau.vider_sortie() if isinstance(e, ChatReponse)]
    assert reponses
    assert reponses[-1].texte.startswith("Trouvé : tronc couché")
