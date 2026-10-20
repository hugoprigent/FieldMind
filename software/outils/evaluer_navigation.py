"""Évaluation de la navigation autonome (critère M3 du brief) sur le banc d'essai.

Pour chaque monde (graine), le rover part du départ et reçoit N objectifs tirés au hasard, l'un
après l'autre. Le cerveau est celui de la démo ; la vérité simulée sert seulement à mesurer :
réussite, collisions, distance minimale réelle entre la carrosserie et les obstacles, durée,
distance parcourue, calculs de chemin et charge processeur.

Les troncs couchés sont plus bas que le plan du lidar. Par défaut, le rover les voit avec sa
paire stéréo (images rendues, profondeur SGBM, carte des hauteurs), exactement comme en démo.
`--sans-stereo` : lidar seul ; `--vision oracle` : en plus, le détecteur parfait de la simulation
en vigilance (boîtes exactes, livrées 2 s après la prise de vue).

    python -m outils.evaluer_navigation --graines 2026 7 11 --objectifs 10
    python -m outils.evaluer_navigation --sans-stereo   # lidar seul
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

import numpy as np

from services.navigation.planificateur import prechauffer
from services.protocole.fiche_rover import FicheRover
from simulation.banc_essai import BancEssai
from simulation.monde import Monde, charger_monde

RACINE = Path(__file__).resolve().parents[1]
DUREE_MAX_OBJECTIF_S = 180.0


@dataclass
class ResultatObjectif:
    graine: int
    numero: int
    objectif: tuple[float, float]
    distance_directe_m: float
    etat_final: str
    atteint: bool
    duree_s: float
    distance_parcourue_m: float
    collisions: int
    degagement_min_m: float
    calculs: int
    temps_calcul_moyen_ms: float | None
    temps_calcul_max_ms: float | None
    reculs: int
    demi_tours: int
    arrets_urgence: int
    erreur_finale_m: float


def obstacles_ponctuels(monde: Monde, rochers: bool = True) -> list[tuple[float, float, float]]:
    """Troncs, troncs couchés (échantillonnés tous les 20 cm) et, si demandé, rochers réduits à
    leur cercle englobant : (x, y, rayon)."""
    points = [(a.x, a.y, a.rayon) for a in monde.arbres]
    if rochers:
        points += [(ro.x, ro.y, ro.rayon) for ro in monde.rochers]
    for tronc in monde.troncs_couches:
        (xa, ya), (xb, yb) = tronc.extremites
        n = max(2, math.ceil(tronc.longueur / 0.2) + 1)
        points += [
            (xa + k / (n - 1) * (xb - xa), ya + k / (n - 1) * (yb - ya), tronc.rayon)
            for k in range(n)
        ]
    return points


def degagement(
    monde: Monde, x: float, y: float, cap: float, longueur: float, largeur: float
) -> float:
    """Distance réelle entre le rectangle du rover et le plus proche obstacle (troncs et troncs
    couchés : cercles ; rochers : leur contour au sol exact)."""
    c, s = math.cos(cap), math.sin(cap)
    hx, hy = longueur / 2, largeur / 2
    meilleur = math.inf
    rectangle = [
        (x + c * px - s * py, y + s * px + c * py)
        for px, py in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))
    ]
    for rocher in monde.rochers:
        if abs(rocher.x - x) > 6 or abs(rocher.y - y) > 6:
            continue
        contour = [(rocher.x + px, rocher.y + py) for px, py in rocher.contour]
        meilleur = min(meilleur, _distance_polygones(rectangle, contour))
    for ox, oy, r in obstacles_ponctuels(monde, rochers=False):
        if abs(ox - x) > 6 or abs(oy - y) > 6:
            continue
        # Centre de l'obstacle dans le repère du rover, puis distance au rectangle.
        lx = c * (ox - x) + s * (oy - y)
        ly = -s * (ox - x) + c * (oy - y)
        dx, dy = max(abs(lx) - hx, 0.0), max(abs(ly) - hy, 0.0)
        meilleur = min(meilleur, math.hypot(dx, dy) - r)
    return meilleur


def _distance_polygones(a: list[tuple[float, float]], b: list[tuple[float, float]]) -> float:
    """Distance entre deux polygones convexes ; négative (−1 cm) s'ils se chevauchent."""

    def separes(p: list[tuple[float, float]], q: list[tuple[float, float]]) -> bool:
        for i in range(len(p)):
            (x1, y1), (x2, y2) = p[i], p[(i + 1) % len(p)]
            nx, ny = y2 - y1, x1 - x2
            proj_p = [nx * u + ny * v for u, v in p]
            proj_q = [nx * u + ny * v for u, v in q]
            if max(proj_p) < min(proj_q) or max(proj_q) < min(proj_p):
                return True
        return False

    if not separes(a, b) and not separes(b, a):
        return -0.01

    def point_segment(
        p: tuple[float, float], s1: tuple[float, float], s2: tuple[float, float]
    ) -> float:
        dx, dy = s2[0] - s1[0], s2[1] - s1[1]
        n2 = dx * dx + dy * dy
        t = 0.0 if n2 < 1e-12 else min(max(((p[0] - s1[0]) * dx + (p[1] - s1[1]) * dy) / n2, 0), 1)
        return math.hypot(p[0] - s1[0] - t * dx, p[1] - s1[1] - t * dy)

    return min(
        point_segment(p, q[i], q[(i + 1) % len(q)])
        for p_, q in ((a, b), (b, a))
        for p in p_
        for i in range(len(q))
    )


def tirer_objectif(
    monde: Monde, rng: np.random.Generator, depuis: tuple[float, float]
) -> tuple[float, float]:
    """Point libre (à plus de 1 m de tout obstacle, hors du fossé et de la butte), à au moins
    8 m de la position actuelle."""
    lim = monde.scenario.limites
    obstacles = np.array(obstacles_ponctuels(monde))
    while True:
        x = float(rng.uniform(lim.xmin + 2, lim.xmax - 2))
        y = float(rng.uniform(lim.ymin + 2, lim.ymax - 2))
        marge = np.hypot(obstacles[:, 0] - x, obstacles[:, 1] - y) - obstacles[:, 2]
        relief = monde.relief
        if relief is not None and (
            float(relief.distance_fosse(np.array([x]), np.array([y]))[0]) < 2.5
            or bool(relief.dans_butte(np.array([x]), np.array([y]), 1.0)[0])
        ):
            continue  # ni au fond du fossé, ni sur la butte
        if marge.min() > 1.0 and math.dist((x, y), depuis) > 8.0:
            return round(x, 2), round(y, 2)


def evaluer_monde(
    fiche: FicheRover,
    graine: int,
    nb_objectifs: int,
    verbeux: bool,
    vision: Literal["aucune", "oracle"] = "aucune",
    stereo: bool = True,
    debug: Path | None = None,
) -> tuple[list[ResultatObjectif], float]:
    monde = charger_monde(RACINE / "simulation" / "scenarios" / "foret_demo.json", graine)
    banc = BancEssai(fiche, monde, graine_bruit=graine, vision=vision, stereo=stereo)
    rng = np.random.default_rng(graine * 7919 + 1)
    g = fiche.geometrie
    banc.avancer(1.0)
    resultats = []
    for numero in range(1, nb_objectifs + 1):
        v0 = banc.verite()
        objectif = tirer_objectif(monde, rng, (v0.x, v0.y))
        cerveau = banc.cerveau
        cerveau.definir_objectif(*objectif, libelle=f"objectif {numero}")
        t0, collisions0 = v0.t, v0.collisions
        distance0 = cerveau.mesures()["distance_parcourue"] or 0.0
        journal_debut = len(banc.releve.evenements)
        degagement_min = math.inf

        def surveiller(b: BancEssai, t0: float = t0) -> bool:
            nonlocal degagement_min
            v = b.verite()
            degagement_min = min(
                degagement_min, degagement(monde, v.x, v.y, v.cap, g.longueur, g.largeur)
            )
            return b.cerveau.etat in ("atteint", "echec") or v.t - t0 > DUREE_MAX_OBJECTIF_S

        banc.avancer(DUREE_MAX_OBJECTIF_S + 1, surveiller)
        v = banc.verite()
        mission = cerveau.derniere_mission
        journaux = [e for e in banc.releve.evenements[journal_debut:] if e.type == "journal"]
        statuts = [e for e in banc.releve.evenements[journal_debut:] if e.type == "statut"]
        temps = mission.duree_calculs_ms if mission else []
        resultat = ResultatObjectif(
            graine=graine,
            numero=numero,
            objectif=objectif,
            distance_directe_m=round(math.dist(objectif, (v0.x, v0.y)), 1),
            etat_final=cerveau.etat,
            atteint=cerveau.etat == "atteint",
            duree_s=round(v.t - t0, 1),
            distance_parcourue_m=round(
                (cerveau.mesures()["distance_parcourue"] or 0.0) - distance0, 1
            ),
            collisions=v.collisions - collisions0,
            degagement_min_m=round(degagement_min, 3),
            calculs=len(temps),
            temps_calcul_moyen_ms=round(statistics.fmean(temps), 1) if temps else None,
            temps_calcul_max_ms=round(max(temps), 1) if temps else None,
            reculs=mission.recuperations if mission else 0,
            demi_tours=sum("Demi-tour" in e.message for e in statuts),  # type: ignore[union-attr]
            arrets_urgence=sum("Arrêt d'urgence" in e.texte for e in journaux),  # type: ignore[union-attr]
            erreur_finale_m=round(math.dist(objectif, (v.x, v.y)), 2),
        )
        resultats.append(resultat)
        if verbeux:
            print(
                f"  graine {graine} · objectif {numero:2d} : {resultat.etat_final:8s} "
                f"{resultat.duree_s:6.1f} s  {resultat.distance_parcourue_m:5.1f} m "
                f"(direct {resultat.distance_directe_m:4.1f} m)  dégagement min "
                f"{resultat.degagement_min_m * 100:5.1f} cm  collisions {resultat.collisions}  "
                f"calculs {resultat.calculs}  reculs {resultat.reculs}  "
                f"demi-tours {resultat.demi_tours}",
                flush=True,
            )
        if cerveau.etat != "atteint" and debug is not None:
            _enregistrer_echec(debug, banc, resultat, journaux)
        if cerveau.etat != "atteint":
            banc.cerveau.sur_stop()
    charge = banc.releve.duree_cerveau_s / max(banc.releve.duree_simulee_s, 1e-9)
    banc.fermer()
    return resultats, charge


def _enregistrer_echec(
    dossier: Path, banc: BancEssai, resultat: ResultatObjectif, journaux: list[object]
) -> None:
    """Cartes du cerveau, vérité et journal au moment de l'échec, pour l'analyse."""
    dossier.mkdir(parents=True, exist_ok=True)
    nom = f"echec_{resultat.graine}_{resultat.numero}"
    c, v = banc.cerveau, banc.verite()
    np.savez_compressed(
        dossier / f"{nom}.npz",
        traversabilite=c.hauteurs.etat,
        occupation=c.grille.occupee(),
        origine=np.array(c.grille.origine),
        verite=np.array([v.x, v.y, v.cap]),
    )
    (dossier / f"{nom}.json").write_text(
        json.dumps(
            {
                "resultat": asdict(resultat),
                "verite": [v.x, v.y, v.cap],
                "journal": [
                    f"{e.t:.1f} {e.niveau} {e.texte}"  # type: ignore[attr-defined]
                    for e in journaux
                    if e.niveau != "info"  # type: ignore[attr-defined]
                ],
            },
            ensure_ascii=False,
            indent=1,
        ),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--graines", type=int, nargs="+", default=[2026])
    parser.add_argument("--objectifs", type=int, default=10)
    parser.add_argument("--rapport", type=Path, default=None)
    parser.add_argument("--silencieux", action="store_true")
    parser.add_argument(
        "--vision",
        choices=["oracle", "aucune"],
        default="aucune",
        help="oracle : en plus, détecteur parfait de la simulation en vigilance",
    )
    parser.add_argument("--sans-stereo", action="store_true", help="lidar seul")
    parser.add_argument("--debug", type=Path, default=None, help="dossier des cartes d'échec")
    args = parser.parse_args()

    prechauffer()
    fiche = FicheRover.charger(RACINE / "config" / "rover_fieldmind.json")
    debut = time.perf_counter()
    tous: list[ResultatObjectif] = []
    charges = []
    for graine in args.graines:
        resultats, charge = evaluer_monde(
            fiche,
            graine,
            args.objectifs,
            not args.silencieux,
            args.vision,
            not args.sans_stereo,
            args.debug,
        )
        tous += resultats
        charges.append(charge)

    atteints = sum(r.atteint for r in tous)
    temps = [r.temps_calcul_moyen_ms for r in tous if r.temps_calcul_moyen_ms is not None]
    synthese = {
        "vision": args.vision,
        "stereo": not args.sans_stereo,
        "objectifs": len(tous),
        "atteints": atteints,
        "taux_reussite": round(atteints / max(len(tous), 1), 3),
        "collisions": sum(r.collisions for r in tous),
        "degagement_min_m": round(min(r.degagement_min_m for r in tous), 3),
        "degagement_p10_m": round(float(np.percentile([r.degagement_min_m for r in tous], 10)), 3),
        "reculs": sum(r.reculs for r in tous),
        "demi_tours": sum(r.demi_tours for r in tous),
        "arrets_urgence_firmware": sum(r.arrets_urgence for r in tous),
        "detour_moyen": round(
            statistics.fmean(r.distance_parcourue_m / max(r.distance_directe_m, 1) for r in tous), 2
        ),
        "temps_calcul_moyen_ms": round(statistics.fmean(temps), 1) if temps else None,
        "temps_calcul_max_ms": max((r.temps_calcul_max_ms or 0) for r in tous),
        "charge_cerveau_coeur": round(statistics.fmean(charges), 4),
        "distance_totale_m": round(sum(r.distance_parcourue_m for r in tous), 1),
        "duree_simulee_totale_s": round(sum(r.duree_s for r in tous), 1),
        "duree_calcul_evaluation_s": round(time.perf_counter() - debut, 1),
    }
    print(json.dumps(synthese, indent=1, ensure_ascii=False))
    if args.rapport:
        args.rapport.parent.mkdir(parents=True, exist_ok=True)
        args.rapport.write_text(
            json.dumps(
                {"synthese": synthese, "objectifs": [asdict(r) for r in tous]},
                indent=1,
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )
    critere = synthese["taux_reussite"] >= 0.8 and synthese["collisions"] == 0
    print("CRITÈRE M3 :", "atteint" if critere else "NON ATTEINT")
    return 0 if critere else 1


if __name__ == "__main__":
    sys.exit(main())
