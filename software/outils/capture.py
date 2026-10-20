"""Capture d'écran (et vidéo facultative) du poste opérateur dans Chromium sans écran.

Exemple, serveur lancé sur le port 8000 :
    python -m outils.capture --url http://127.0.0.1:8000/ --sortie captures/m0.png --attente 3
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from playwright.sync_api import sync_playwright

CHROMIUM_PREINSTALLE = Path("/opt/pw-browsers/chromium-1194/chrome-linux/chrome")


def capturer(
    url: str, sortie: Path, attente_s: float, largeur: int, hauteur: int, video: Path | None
) -> list[str]:
    """Ouvre la page, attend, enregistre une capture ; renvoie les messages de la console."""
    sortie.parent.mkdir(parents=True, exist_ok=True)
    console: list[str] = []
    with sync_playwright() as pw:
        options: dict[str, object] = {
            "args": ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"]
        }
        if CHROMIUM_PREINSTALLE.exists() and not os.environ.get("PLAYWRIGHT_CHROMIUM"):
            options["executable_path"] = str(CHROMIUM_PREINSTALLE)
        navigateur = pw.chromium.launch(**options)  # type: ignore[arg-type]
        contexte = navigateur.new_context(
            viewport={"width": largeur, "height": hauteur},
            record_video_dir=str(video) if video else None,
            record_video_size={"width": largeur, "height": hauteur} if video else None,
        )
        page = contexte.new_page()
        page.on("console", lambda m: console.append(f"[{m.type}] {m.text}"))
        page.on("pageerror", lambda e: console.append(f"[pageerror] {e}"))
        page.goto(url)
        page.wait_for_timeout(attente_s * 1000)
        page.screenshot(path=str(sortie))
        contexte.close()
        navigateur.close()
    return console


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8000/")
    parser.add_argument("--sortie", type=Path, default=Path("captures/capture.png"))
    parser.add_argument("--attente", type=float, default=3.0)
    parser.add_argument("--largeur", type=int, default=1600)
    parser.add_argument("--hauteur", type=int, default=900)
    parser.add_argument("--video", type=Path, default=None, help="dossier où enregistrer la vidéo")
    args = parser.parse_args()
    console = capturer(args.url, args.sortie, args.attente, args.largeur, args.hauteur, args.video)
    for ligne in console:
        print(ligne)
    print(f"capture : {args.sortie}")


if __name__ == "__main__":
    main()
