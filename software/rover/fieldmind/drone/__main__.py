"""Exécuter les scénarios de drone en local, sans matériel ni réseau."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

from .simulation import SCENARIOS, executer


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scenario", choices=(*SCENARIOS, "tous"), default="normal")
    p.add_argument("--graines", type=int, default=10)
    p.add_argument("--sortie", type=Path, required=True)
    a = p.parse_args()
    if not 1 <= a.graines <= 100:
        p.error("--graines doit être compris entre 1 et 100")
    a.sortie.mkdir(parents=True, exist_ok=True)
    scenarios = SCENARIOS if a.scenario == "tous" else (a.scenario,)
    resultats = []
    for sc in scenarios:
        for graine in range(a.graines):
            d, photos = executer(graine, sc)
            resultats.append(d)
            if graine == 0:
                dossier = a.sortie / sc
                dossier.mkdir(exist_ok=True)
                for i, photo in enumerate(photos, 1):
                    if not cv2.imwrite(str(dossier / f"vue-{i}.jpg"), photo):
                        raise OSError("écriture photo simulée impossible")
                if len(photos) == 6:
                    montage = cv2.vconcat([cv2.hconcat(photos[:3]), cv2.hconcat(photos[3:])])
                    if not cv2.imwrite(str(dossier / "montage.jpg"), montage):
                        raise OSError("écriture montage impossible")
    (a.sortie / "rapport.json").write_text(
        json.dumps(resultats, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    for sc in scenarios:
        essais = [d for d in resultats if d["scenario"] == sc]
        print(
            f"{sc}: {len(essais)} essais, "
            f"{sum(d['phase'] == 'termine' for d in essais)} missions terminées, "
            f"{sum(d['au_sol'] for d in essais)} retours au sol simulés"
        )
    # Une panne simulée doit produire un repli, jamais une fausse mission réussie.
    attendus = {"normal": "termine", "derive": "termine", "cap_perdu": "erreur"}
    if any(d["phase"] != attendus.get(d["scenario"], "interrompu") for d in resultats):
        raise SystemExit("échec : comportement de scénario inattendu (voir rapport)")


if __name__ == "__main__":
    main()
