"""Mesure du modèle de vision sur des images simulées, face au détecteur parfait (vérité).

Pour chaque vue (rover placé devant un tronc couché à une distance donnée, ou vue sans tronc) :
tronc détecté ou non, fausses alertes, écart de la boîte avec la vérité, erreur de position au
sol du bord proche, latence. Utilise le fournisseur configuré (OPENROUTER_API_KEY, software/.env).

    python -m outils.mesurer_vision --rapport rapports/mesure_vision.json
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math
import statistics
import sys
from pathlib import Path

from services.api.config import RACINE_LOGICIEL, Reglages
from services.api.serveur import _infos_openrouter, creer_vision
from services.perception.camera import ModeleCamera
from services.perception.observations import placer_objets
from services.perception.vision import AnalyseVision, DemandeVision, executer_analyse
from services.protocole.fiche_rover import FicheRover
from services.protocole.messages import Image
from simulation.camera_simulee import CameraSimulee
from simulation.monde import charger_monde
from simulation.robot_simule import RobotSimule
from simulation.vision_oracle import DetecteurVerite

SCENARIO = RACINE_LOGICIEL / "simulation" / "scenarios" / "foret_demo.json"
# (tronc visé, distance devant, décalage de cap en degrés) ; None : vue sans tronc couché.
VUES: list[tuple[int | None, float, float]] = [
    (0, 2.0, 0.0),
    (0, 3.0, 0.0),
    (0, 5.0, 0.0),
    (0, 8.0, 0.0),
    (1, 3.0, 0.0),
    (1, 5.0, 15.0),
    (0, 4.0, -25.0),
    (None, 0.0, 0.0),
    (None, 0.0, 0.0),
]


def _placer(robot: RobotSimule, x: float, y: float, cap: float) -> None:
    robot.donnees.qpos[0], robot.donnees.qpos[1] = x, y
    robot.donnees.qpos[3:7] = [math.cos(cap / 2), 0, 0, math.sin(cap / 2)]
    robot.avancer(0.6)


def _garder_image(chemin: Path, donnees_b64: str) -> None:
    chemin.parent.mkdir(parents=True, exist_ok=True)
    chemin.write_bytes(base64.b64decode(donnees_b64))


def _iou(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


async def mesurer(dossier_images: Path | None) -> dict[str, object]:
    cfg = Reglages()
    fiche = FicheRover.charger(cfg.fiche_rover)
    camera = fiche.capteurs.camera
    assert camera is not None
    fournisseur = creer_vision(cfg, fiche, _infos_openrouter(cfg))
    if fournisseur is None:
        raise SystemExit("aucun modèle de vision configuré (OPENROUTER_API_KEY ?)")
    monde = charger_monde(SCENARIO)
    modele_camera = ModeleCamera(camera)
    resultats = []
    points_libres = [(-26.5, -26.0, 0.72 + math.pi / 2), (16.0, 24.0, -2.2)]
    for k, (indice, distance, decalage) in enumerate(VUES):
        robot = RobotSimule(fiche, monde)
        if indice is None:
            _placer(robot, *points_libres[k % len(points_libres)])
        else:
            tronc = monde.troncs_couches[indice]
            axe = tronc.angle + math.pi / 2
            _placer(
                robot,
                tronc.x - distance * math.cos(axe),
                tronc.y - distance * math.sin(axe),
                axe + math.radians(decalage),
            )
        capture = robot.prendre_capture()
        verite = robot.verite_a(capture.t) if capture else None
        assert capture is not None
        assert verite is not None
        rendu = CameraSimulee(robot.modele, camera)
        images = rendu.rendre(capture.qpos, capture.t)
        rendu.fermer()
        assert images, rendu.erreur
        image = images[0]
        assert isinstance(image, Image)
        demande = DemandeVision(
            k,
            capture.t,
            image.donnees,
            image.largeur,
            image.hauteur,
            verite.x,
            verite.y,
            verite.cap,
            "operateur",
        )
        attendu = DetecteurVerite(lambda r=robot: r).analyser_immediat(demande)
        resultat = await executer_analyse(fournisseur, demande)
        analyse = resultat.analyse or AnalyseVision(resume="", objets=[])
        troncs = [o for o in analyse.objets if o.categorie == "tronc_couche"]
        mesure: dict[str, object] = {
            "vue": k,
            "tronc": indice,
            "distance_m": distance,
            "decalage_cap_deg": decalage,
            "troncs_verite": len(attendu.objets),
            "troncs_vus": len(troncs),
            "duree_s": round(resultat.duree_ms / 1000, 1),
            "erreur": resultat.erreur,
            "fournisseur": resultat.fournisseur,
            "resume": analyse.resume,
        }
        if attendu.objets and troncs:
            vrai = attendu.objets[0]
            meilleur = max(troncs, key=lambda o: _iou(o.boite, vrai.boite))
            mesure["iou"] = round(_iou(meilleur.boite, vrai.boite), 2)
            mesure["ecart_bas_px"] = round(
                (meilleur.boite[3] - vrai.boite[3]) * image.hauteur / 1000, 1
            )
            place_vrai = placer_objets(
                AnalyseVision(resume="", objets=[vrai]), demande, modele_camera, 1
            )
            place = placer_objets(
                AnalyseVision(resume="", objets=[meilleur]), demande, modele_camera, 1
            )
            if place_vrai and place:
                mesure["erreur_sol_m"] = round(
                    math.dist((place[0].x, place[0].y), (place_vrai[0].x, place_vrai[0].y)), 2
                )
        if dossier_images is not None:
            _garder_image(dossier_images / f"vue_{k}.jpg", image.donnees)
        resultats.append(mesure)
        print(json.dumps(mesure, ensure_ascii=False), flush=True)

    avec = [r for r in resultats if r["troncs_verite"]]
    sans = [r for r in resultats if not r["troncs_verite"]]
    durees = [float(r["duree_s"]) for r in resultats if not r["erreur"]]  # type: ignore[arg-type]
    synthese = {
        "fournisseur": fournisseur.nom,
        "vues": len(resultats),
        "echecs_appel": sum(1 for r in resultats if r["erreur"]),
        "troncs_detectes": f"{sum(1 for r in avec if r['troncs_vus'])} / {len(avec)}",
        "fausses_alertes": f"{sum(1 for r in sans if r['troncs_vus'])} / {len(sans)}",
        "iou_moyen": round(statistics.fmean([r["iou"] for r in avec if "iou" in r]), 2)  # type: ignore[misc]
        if any("iou" in r for r in avec)
        else None,
        "erreur_sol_mediane_m": statistics.median(
            [r["erreur_sol_m"] for r in avec if "erreur_sol_m" in r]
        )  # type: ignore[type-var]
        if any("erreur_sol_m" in r for r in avec)
        else None,
        "latence_mediane_s": statistics.median(durees) if durees else None,
    }
    return {"synthese": synthese, "vues": resultats}


def main() -> int:
    parseur = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parseur.add_argument("--rapport", type=Path, default=None)
    parseur.add_argument("--images", type=Path, default=None, help="dossier où garder les vues")
    args = parseur.parse_args()
    rapport = asyncio.run(mesurer(args.images))
    print(json.dumps(rapport["synthese"], indent=1, ensure_ascii=False))
    if args.rapport:
        args.rapport.parent.mkdir(parents=True, exist_ok=True)
        args.rapport.write_text(json.dumps(rapport, indent=1, ensure_ascii=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
