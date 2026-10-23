"""Chaîne agent → mission : matériel et fournisseur factices, aucun réseau ni moteur réel."""

from __future__ import annotations

import asyncio
import time
from concurrent.futures import Future
from types import SimpleNamespace

import numpy as np
import pytest

from fieldmind.bus import Bus, S
from fieldmind.config import Profil
from fieldmind.controle.conduite import Conduite
from fieldmind.ia.agent import Agent
from fieldmind.ia.fournisseurs import AppelFonction, ReponseModele
from fieldmind.mission.actions import Pas
from fieldmind.mission.executeur import Chercher, Executeur
from fieldmind.mission.exploration import Explorer
from fieldmind.securite.superviseur import Superviseur
from fieldmind.types import Chemin, EtatSecurite, Odometrie, Pose2D


class EspFactice:
    def __init__(self):
        self.arrets = 0

    def connecte(self):
        return True

    def arret_traction(self):
        self.arrets += 1


class NavigationFactice:
    def planifier(self, carte, pose, but):
        return Chemin(time.monotonic(), np.array([[pose.x, pose.y], but]), but)

    def commande(self, *args):
        return SimpleNamespace(vitesse=0.2, braquage=0.0, arrive=False, bloque=False)


def equipement():
    bus, profil, esp = Bus(), Profil.charger(reglages=None), EspFactice()
    profil.regler("conduite.armer_auto", True)
    conduite = Conduite(bus, profil, lambda: esp, Superviseur(bus, profil))
    executeur = Executeur(bus, profil, conduite, NavigationFactice())
    t = time.monotonic()
    bus.publier(S.ODOM, Odometrie(t, Pose2D()))
    bus.publier(S.SECURITE, EtatSecurite(t, arret=False, vitesse_max_avant=0.5,
                                      vitesse_max_arriere=0.3))
    return executeur, esp


class OutilsFactices:
    def __init__(self, executeur):
        self.executeur = executeur

    def etat_resume(self):
        conduite = self.executeur.conduite
        return {"mission": self.executeur.resume(), "mode": conduite.mode,
                "arme": conduite.arme, "arret_urgence": conduite.superviseur.arret_urgence}

    def pose_odom(self):
        return Pose2D()

    def ajouter_actions(self, actions, remplacer):
        return self.executeur.ajouter(actions, remplacer=remplacer, origine="ia")

    def annuler(self):
        self.executeur.annuler()

    def arret_immediat(self, raison):
        self.executeur.annuler(raison)


def faux_agent(executeur, fournisseur):
    config = SimpleNamespace(agent=SimpleNamespace(historique_echanges=4, actions_max=12),
                             actif=SimpleNamespace(appels_successifs=False),
                             passerelle=lambda: fournisseur)
    return Agent(executeur.bus, executeur.profil, config, OutilsFactices(executeur))


class FournisseurFactice:
    def acces(self, *args):
        pass

    async def chat(self, *args, **kwargs):
        return ReponseModele("", [AppelFonction("explorer", {"points_max": 2})])


async def test_explorer_depuis_arret_arme_auto_et_entre_mission():
    executeur, esp = equipement()
    assert executeur.conduite.mode == "arret" and not executeur.conduite.arme
    resultat = await faux_agent(executeur, FournisseurFactice()).consigne("explore la pièce")
    assert resultat.statut == "plan"
    assert executeur.conduite.mode == "auto" and executeur.conduite.arme
    assert executeur.resume()["file"][0]["type"] == "explorer"
    assert executeur.resume()["file"][0]["points_max"] == 2
    assert esp.arrets == 0
    executeur.annuler()


@pytest.mark.parametrize("operation", ["annuler", "remplacer"])
def test_pas_revenu_apres_annulation_ou_remplacement_ne_propose_aucun_mouvement(operation):
    executeur, _ = equipement()
    executeur.ajouter([{"type": "explorer"}])
    recues = []
    proposer = executeur.conduite.proposer
    executeur.conduite.proposer = lambda c: (recues.append(c), proposer(c))

    class PrimitiveRetardee:
        def pas(self, ctx):
            if operation == "annuler":
                executeur.annuler()
            else:
                executeur.ajouter([{"type": "attendre", "duree_s": 5}], remplacer=True)
            return Pas(vitesse=0.4, braquage=0.2)

    executeur._primitive = lambda action: PrimitiveRetardee()
    executeur._cycle(time.monotonic())
    assert all(c.vitesse == 0 for c in recues)
    assert executeur.resume()["file"] == ([] if operation == "annuler" else [
        {"type": "attendre", "duree_s": 5.0}])


@pytest.mark.parametrize("operation", ["pause", "desarmer", "urgence", "mode"])
def test_changement_securite_pendant_pas_supprime_commande_retardee(operation):
    executeur, _ = equipement()
    executeur.ajouter([{"type": "explorer"}])
    recues = []
    proposer = executeur.conduite.proposer
    executeur.conduite.proposer = lambda c: (recues.append(c), proposer(c))

    class PrimitiveRetardee:
        def pas(self, ctx):
            if operation == "pause":
                executeur.pause()
            elif operation == "desarmer":
                executeur.conduite.desarmer()
            elif operation == "urgence":
                executeur.conduite.arret_urgence()
            else:
                executeur.conduite.changer_mode("manuel")
            return Pas(vitesse=0.4)

    executeur._primitive = lambda action: PrimitiveRetardee()
    executeur._cycle(time.monotonic())
    assert all(c.vitesse == 0 for c in recues)


@pytest.mark.parametrize("operation", ["annuler", "remplacer"])
async def test_reponse_agent_apres_annulation_externe_ne_recree_pas_ancienne_mission(operation):
    executeur, _ = equipement()
    entre, rendre = asyncio.Event(), asyncio.Event()

    class Lent(FournisseurFactice):
        async def chat(self, *args, **kwargs):
            entre.set()
            await rendre.wait()
            return await super().chat(*args, **kwargs)

    agent = faux_agent(executeur, Lent())
    tache = asyncio.create_task(agent.consigne("explore la pièce"))
    await entre.wait()
    if operation == "annuler":
        executeur.annuler()
    else:
        executeur.ajouter([{"type": "attendre", "duree_s": 5}], remplacer=True)
    generation = executeur.generation
    rendre.set()
    resultat = await tache
    assert resultat.statut == "abandonne"
    assert executeur.generation == generation
    assert not any(a["type"] == "explorer" for a in executeur.resume()["file"])


def test_memoire_objet_navigation_puis_confirmation_visuelle_sans_succes_premature():
    executeur, _ = equipement()
    executeur.monde = SimpleNamespace(rechercher=lambda objet: [{"x": 3.0, "y": 0.0}])
    futur, appels = Future(), []
    executeur.lancer_ia = lambda demande, arg: (appels.append(demande), futur)[1]
    chercher = Chercher({"type": "chercher", "objet": "chaise"}, executeur)
    ctx = executeur._contexte(time.monotonic())
    pas = chercher.pas(ctx)
    assert pas.vitesse > 0 and chercher._mode == "memoire"
    assert not appels
    ctx.odom = Odometrie(ctx.t, Pose2D(2.2, 0))
    pas = chercher.pas(ctx)
    assert pas.etat == "route" and pas.vitesse == 0
    assert appels == ["chercher"] and chercher._mode == "analyse"
    futur.set_result([{"sol": [0.6, 0], "pose_odom": [2.2, 0, 0], "image_t": ctx.t}])
    assert chercher.pas(ctx).etat == "termine"


def test_memoire_objet_non_confirme_declenche_exploration_apres_observations_locales(monkeypatch):
    executeur, _ = equipement()
    executeur.monde = SimpleNamespace(rechercher=lambda objet: [{"x": 3.0, "y": 0.0}])

    def observer(demande, argument):
        futur = Future()
        futur.set_result([])
        return futur

    executeur.lancer_ia = observer
    chercher = Chercher({"type": "chercher", "objet": "chaise"}, executeur)
    ctx = executeur._contexte(time.monotonic())
    chercher.pas(ctx)
    ctx.odom = Odometrie(ctx.t, Pose2D(2.2, 0))
    chercher.pas(ctx)
    assert chercher.pas(ctx).etat == "route"
    assert chercher._mode == "rotation"
    monkeypatch.setattr(chercher._sous, "pas", lambda ctx: Pas(etat="termine"))
    chercher.pas(ctx)
    pas = chercher.pas(ctx)
    assert pas.etat == "route"
    assert chercher._mode == "exploration" and isinstance(chercher._sous, Explorer)
    assert "exploration" in pas.detail


def test_odom_indisponible_ne_termine_pas_mission_remplacee_pendant_lecture_contexte():
    executeur, _ = equipement()
    executeur.ajouter([{"type": "attendre", "duree_s": 100}])
    executeur._cycle(time.monotonic())
    executeur._courante.debut -= 10

    def contexte_remplace(t):
        executeur.ajouter([{"type": "attendre", "duree_s": 2}], remplacer=True)
        return None

    executeur._contexte = contexte_remplace
    executeur._cycle(time.monotonic())
    assert executeur.resume()["file"] == [{"type": "attendre", "duree_s": 2.0}]
    assert executeur.etat != "echec"


async def test_desarmement_operateur_pendant_analyse_ne_rearme_pas_rover():
    executeur, _ = equipement()
    executeur.conduite.changer_mode("auto")
    assert executeur.conduite.arme
    entre, rendre = asyncio.Event(), asyncio.Event()

    class Lent(FournisseurFactice):
        async def chat(self, *args, **kwargs):
            entre.set()
            await rendre.wait()
            return await super().chat(*args, **kwargs)

    tache = asyncio.create_task(faux_agent(executeur, Lent()).consigne("explore la pièce"))
    await entre.wait()
    executeur.conduite.desarmer()
    rendre.set()
    resultat = await tache
    assert resultat.statut == "abandonne"
    assert not executeur.conduite.arme
    assert not executeur.occupe()


def test_chercher_ne_termine_pas_sur_detection_expiree():
    executeur, _ = equipement()
    futur = Future()
    executeur.lancer_ia = lambda demande, argument: futur
    chercher = Chercher({"type": "chercher", "objet": "chaise"}, executeur)
    ctx = executeur._contexte(time.monotonic())
    chercher.pas(ctx)
    futur.set_result([{"sol": [0.3, 0], "pose_odom": [0, 0, 0],
                        "image_t": ctx.t - 100}])
    pas = chercher.pas(ctx)
    assert pas.etat != "termine"
    assert chercher.trouve is None
