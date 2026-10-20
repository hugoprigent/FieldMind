"""Vision par modèle : décrire la scène, pointer un objet (Gemini Robotics ER 2, ou modèle local).

    vision = Vision(bus, profil, config_ia, outils)
    message = await vision.decrire("gauche")                  # message `vision`, aussi dans le chat
    points = await vision.chercher("chaise rouge", "gauche")  # [{"u", "v", "label", "sol"}]

Pointage : consigne recommandée par Google pour Robotics ER (points `[y, x]` normalisés 0–1000,
au plus 10, JSON `[{"point": [y, x], "label": ...}]`), demandée au pied de l'objet pour que la
projection au sol (`outils.pixel_vers_sol`, stéréo ou sol plan) donne sa vraie position. Un
modèle Qwen-VL répond en `[x, y]` : `format_points = "xy_1000"` dans config/ia.toml.

Le modèle propose, le code décide : points hors de l'image ou mal formés écartés un à un, texte
de résultat composé ici. `chercher` est aussi ce qu'appelle l'action de mission « chercher ».
"""

from __future__ import annotations

import asyncio
import base64
import logging
import math
import re
import time
from typing import Any

from fieldmind.bus import Bus, S
from fieldmind.ia.config import ConfigIA
from fieldmind.ia.fournisseurs import ErreurIA, extraire_json, url_donnees
from fieldmind.ia.inference import ClientInference
from fieldmind.ia.messages import nouvel_id, publier_chat
from fieldmind.ia.outils import OutilsRover
from fieldmind.types import ImageCamera

journal = logging.getLogger("fieldmind.ia.vision")

POINTS_MAX = 10
TOLERANCE_POINTS = 5.0
"""Coordonnées 0–1000 acceptées jusqu'à 5 au-delà des bords (ramenées sur le bord)."""


def preparer_image(image: ImageCamera, largeur_max: int) -> tuple[bytes, int, int]:
    """JPEG réduit à `largeur_max` px de large (OpenCV s'il est installé, sinon tel quel)."""
    if image.largeur <= largeur_max:
        return image.jpeg, image.largeur, image.hauteur
    try:
        import cv2  # paquet python3-opencv du Pi ; facultatif ici
        import numpy as np
    except ImportError:
        return image.jpeg, image.largeur, image.hauteur
    rapport = image.largeur / largeur_max
    reduction = 8 if rapport >= 8 else 4 if rapport >= 4 else 2 if rapport >= 2 else 1
    drapeau = {1: cv2.IMREAD_COLOR, 2: cv2.IMREAD_REDUCED_COLOR_2,
               4: cv2.IMREAD_REDUCED_COLOR_4, 8: cv2.IMREAD_REDUCED_COLOR_8}[reduction]
    bgr = cv2.imdecode(np.frombuffer(image.jpeg, np.uint8), drapeau)
    if bgr is None:
        raise ErreurIA("image de la caméra illisible (JPEG)")
    hauteur, largeur = bgr.shape[:2]
    if largeur > largeur_max:
        nouvelle_hauteur = max(1, round(hauteur * largeur_max / largeur))
        bgr = cv2.resize(bgr, (largeur_max, nouvelle_hauteur), interpolation=cv2.INTER_AREA)
    ok, tampon = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return image.jpeg, image.largeur, image.hauteur
    return tampon.tobytes(), bgr.shape[1], bgr.shape[0]


def consigne_pointage(objet: str, format_points: str = "yx_1000") -> str:
    if format_points == "robobrain":  # gabarit de RoboBrain 2.5 (inference.py du dépôt BAAI)
        return (f"{objet}. Please provide its 2D coordinates. Your answer should be formatted as "
                "a tuple, i.e. [(x, y)], where the tuple contains the x and y coordinates of a "
                "point satisfying the conditions above.")
    ordre = "[y, x]" if format_points == "yx_1000" else "[x, y]"
    return (
        f'Point to each "{objet}" in the image (the object is described in French). Point to '
        "no more than 10 items. Place each point at the bottom of the object, where it touches "
        "the ground (its center if it does not touch the ground). The label returned should be "
        "an identifying name for the object detected, in French. The answer should follow the "
        f'json format: [{{"point": {ordre}, "label": <label1>}}, ...]. The points are in '
        f"{ordre} format normalized to 0-1000. If there is no such object in the image, answer "
        "[]."
    )


def consigne_description(camera: str, hauteur_m: float, question: str | None) -> str:
    texte = (
        "Tu es la vision d'un petit rover terrestre. Cette image vient de sa caméra avant "
        f"{camera}, à {hauteur_m:.2f} m du sol, légèrement inclinée vers le bas. Décris en deux "
        "ou trois phrases courtes, en français, ce que voit le rover : type de lieu, sol, "
        "obstacles et passages libres, personnes ou animaux, objets notables avec leur position "
        "(à gauche, au centre, à droite ; près, loin). N'invente rien ; si l'image est floue, "
        "sombre ou masquée, dis-le. Réponds par le texte seul, sans mise en forme."
    )
    if question:
        texte += f"\nRéponds d'abord à la question de l'opérateur : « {question} »"
    return texte


def consigne_analyse_scene(format_points: str = "yx_1000") -> str:
    ordre_pt = "[y, x]" if format_points == "yx_1000" else "[x, y]"
    ordre_box = "[ymin, xmin, ymax, xmax]" if format_points == "yx_1000" else (
        "[xmin, ymin, xmax, ymax]")
    return (
        "Detect the distinct physical objects and obstacles visible in the image (furniture, "
        "boxes, chairs, tables, poles/columns, low obstacles on the floor, etc.; ignore walls, "
        "floor, and ceiling). Return no more than 10 items. For each item, provide:\n"
        f'- "point": {ordre_pt} at the bottom center of the object where it touches the ground '
        "(normalized 0-1000),\n"
        f'- "box_2d": {ordre_box} bounding box of the object (normalized 0-1000),\n'
        '- "label": short name in French (e.g. "canapé", "table basse", "chaise", "poteau", '
        '"carton", "poutre", "marche"),\n'
        '- "categorie": one of "bloc" (furniture, box, chair, table), "cylindre" (pole, column, '
        'leg, tree trunk), or "bas" (low flat obstacle on the floor < 30 cm).\n'
        f'Answer strictly in JSON format: [{{"point": {ordre_pt}, "box_2d": {ordre_box}, '
        '"label": "...", "categorie": "bloc|cylindre|bas"}}, ...]. If none, answer [].'
    )


def lire_points_tuples(texte: str, label: str) -> list[tuple[float, float, str]]:
    """Réponse RoboBrain `[(x, y), ...]` (0–1000) → [(u, v, label)] ; mal formés écartés."""
    points: list[tuple[float, float, str]] = []
    for x_txt, y_txt in re.findall(r"\(\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*\)", texte):
        x, y = float(x_txt), float(y_txt)
        if not all(-TOLERANCE_POINTS <= c <= 1000 + TOLERANCE_POINTS for c in (x, y)):
            continue
        points.append((round(min(max(x, 0.0), 1000.0) / 1000.0, 4),
                       round(min(max(y, 0.0), 1000.0) / 1000.0, 4), label))
        if len(points) >= POINTS_MAX:
            break
    return points


def lire_points(donnees: Any, format_points: str = "yx_1000",
                label_defaut: str = "") -> list[tuple[float, float, str]]:
    """Points du modèle → [(u, v, label)] avec u, v normalisés 0..1 (0 = gauche / haut).

    Accepte `[{"point": [y, x], "label": ...}]` (ou `[x, y]` en `xy_1000`), un objet
    `{"points": [...]}`, et `{"y": .., "x": ..}` à la place de la liste. Un point mal formé ou
    hors de l'image est écarté."""
    if isinstance(donnees, dict):
        donnees = donnees.get("points", donnees.get("objets", []))
    if not isinstance(donnees, list):
        raise ValueError("liste de points attendue")
    points: list[tuple[float, float, str]] = []
    for element in donnees:
        if not isinstance(element, dict):
            continue
        brut = element.get("point")
        if isinstance(brut, dict):
            brut = [brut.get("y"), brut.get("x")] if format_points == "yx_1000" else [
                brut.get("x"), brut.get("y")]
        if not (isinstance(brut, list | tuple) and len(brut) == 2):
            continue
        try:
            a, b = float(brut[0]), float(brut[1])
        except (TypeError, ValueError):
            continue
        if not (math.isfinite(a) and math.isfinite(b)):
            continue
        y, x = (a, b) if format_points == "yx_1000" else (b, a)
        if not all(-TOLERANCE_POINTS <= c <= 1000 + TOLERANCE_POINTS for c in (x, y)):
            continue
        u, v = min(max(x, 0.0), 1000.0) / 1000.0, min(max(y, 0.0), 1000.0) / 1000.0
        label = element.get("label")
        label = " ".join(label.split())[:60] if isinstance(label, str) and label.strip() else (
            label_defaut)
        points.append((round(u, 4), round(v, 4), label))
        if len(points) >= POINTS_MAX:
            break
    return points


def lire_detections(donnees: Any, format_points: str = "yx_1000"
                    ) -> list[dict[str, Any]]:
    """Lit les objets détectés : point `(u, v)`, `boite` `(u0, v0, u1, v1)` facultative,
    `label` et `categorie`."""
    if isinstance(donnees, dict):
        donnees = donnees.get("points", donnees.get("objets", []))
    if not isinstance(donnees, list):
        raise ValueError("liste d'objets attendue")
    sortie: list[dict[str, Any]] = []
    for element in donnees:
        if not isinstance(element, dict):
            continue
        boite: tuple[float, float, float, float] | None = None
        brut_box = element.get("box_2d") or element.get("box") or element.get("boite")
        if isinstance(brut_box, list | tuple) and len(brut_box) == 4:
            try:
                vals = [float(v) for v in brut_box]
                if all(math.isfinite(v) for v in vals):
                    if format_points == "yx_1000":
                        y0, x0, y1, x1 = vals
                    else:
                        x0, y0, x1, y1 = vals
                    u0 = min(max(min(x0, x1), 0.0), 1000.0) / 1000.0
                    v0 = min(max(min(y0, y1), 0.0), 1000.0) / 1000.0
                    u1 = min(max(max(x0, x1), 0.0), 1000.0) / 1000.0
                    v1 = min(max(max(y0, y1), 0.0), 1000.0) / 1000.0
                    if u1 - u0 > 0.01 and v1 - v0 > 0.01:
                        boite = (round(u0, 4), round(v0, 4), round(u1, 4), round(v1, 4))
            except (TypeError, ValueError):
                boite = None
        pts = lire_points([element], format_points, "objet")
        if pts:
            u, v, label = pts[0]
        elif boite is not None:
            u = round(0.5 * (boite[0] + boite[2]), 4)
            v = round(boite[3], 4)
            brut_label = element.get("label")
            label = " ".join(brut_label.split())[:60] if isinstance(brut_label, str) and (
                brut_label.strip()) else "objet"
        else:
            continue
        cat = str(element.get("categorie") or element.get("category")
                  or element.get("type") or "bloc").lower().strip()
        if cat not in ("bloc", "cylindre", "pilier", "arbre", "bas", "obstacle_bas", "mur"):
            cat = "bloc"
        sortie.append({"u": u, "v": v, "boite": boite, "label": label, "categorie": cat})
        if len(sortie) >= POINTS_MAX:
            break
    return sortie


def _fr(valeur: float) -> str:
    return f"{valeur:.1f}".replace(".", ",")


def texte_recherche(objet: str, camera: str, points: list[dict[str, Any]]) -> str:
    if not points:
        return f"Je ne vois pas de « {objet} » (caméra {camera})."
    au_sol = [p for p in points if p.get("sol")]
    texte = f"Je vois {len(points)} « {objet} »" if len(points) > 1 else f"Je vois « {objet} »"
    if au_sol:
        x, y = min((p["sol"] for p in au_sol), key=lambda s: math.hypot(s[0], s[1]))
        cote = (f", {_fr(abs(y))} m à {'gauche' if y > 0 else 'droite'}" if abs(y) >= 0.05
                else "")
        texte += (", le plus proche" if len(points) > 1 else "") + f" à {_fr(x)} m devant{cote}"
    else:
        distances = [p["distance_m"] for p in points if p.get("distance_m")]
        if distances:  # pas de position au sol : distance de la profondeur métrique
            texte += f", à environ {_fr(min(distances))} m"
    return texte + "."


def texte_analyse(camera: str, points: list[dict[str, Any]]) -> str:
    if not points:
        return f"Aucun objet repéré sur la caméra {camera}."
    morceaux = []
    for p in points:
        nom = p.get("label") or "objet"
        sol = p.get("sol")
        if sol:
            d = _fr(math.hypot(sol[0], sol[1]))
            h = f", h {_fr(p['hauteur'])} m" if p.get("hauteur") else ""
            src = f" [{p['source']}]" if p.get("source") else ""
            morceaux.append(f"{nom} ({d} m{h}{src})")
        else:
            morceaux.append(nom)
    return f"{len(points)} objet(s) fusionné(s) : " + ", ".join(morceaux) + "."


class Vision:
    def __init__(self, bus: Bus, profil: Any, config_ia: ConfigIA, outils: OutilsRover,
                 inference: ClientInference | None = None) -> None:
        self.bus = bus
        self.profil = profil
        self.config_ia = config_ia
        self.outils = outils
        self.inference = inference or ClientInference(config_ia)
        self.dernier: dict[str, Any] | None = None
        """Dernier message `vision` publié."""
        self._verrou = asyncio.Lock()

    @property
    def occupee(self) -> bool:
        return self._verrou.locked()

    def _hauteur_camera(self) -> float:
        try:
            return float(self.profil.valeur("cameras.z_m", 0.265))
        except (AttributeError, TypeError, ValueError):
            return 0.265

    async def _image(self, camera: str, role: str = "vision") -> tuple[bytes, int, int]:
        image = self.outils.derniere_image(camera)
        if image is None:
            raise ErreurIA(f"pas d'image de la caméra {camera}")
        largeur = self.config_ia.passerelle().acces(role).role.largeur_image  # type: ignore[arg-type]
        return await asyncio.to_thread(preparer_image, image, largeur)

    async def _analyser(self, camera: str, consigne: str, role: str = "vision"
                        ) -> tuple[str, bytes, str, float]:
        """(texte du modèle, JPEG analysé, modèle, durée). Lève ErreurIA."""
        acces = self.config_ia.passerelle().acces(role)  # type: ignore[arg-type]
        jpeg, _, _ = await self._image(camera, role)
        messages = [{"role": "user", "content": [
            {"type": "image_url", "image_url": {"url": url_donnees(jpeg, "image/jpeg")}},
            {"type": "text", "text": consigne},
        ]}]
        debut = time.monotonic()
        reponse = await self.config_ia.passerelle().chat(role, messages)  # type: ignore[arg-type]
        return reponse.texte, jpeg, reponse.modele or acces.role.modele, time.monotonic() - debut

    def _role_pointage(self) -> str:
        """Modèle incarné (RoboBrain) s'il est dans le profil, sinon le modèle de vision."""
        return "pointage" if self.config_ia.actif.role("pointage") is not None else "vision"

    async def _enrichir(self, jpeg: bytes, points: list[dict[str, Any]],
                        boites: list[tuple[float, float, float, float]] | None = None) -> None:
        """Distance (profondeur métrique) et contour (segmentation) de chaque point, depuis le
        serveur des outils, en un seul envoi de l'image. Sans serveur : rien ne change."""
        if not points or not self.inference.disponible("profondeur"):
            return
        try:
            reponse = await self.inference.analyser(
                jpeg, taches=("profondeur", "segmentation"), role="profondeur",
                points=[(p["u"], p["v"]) for p in points], boites=boites)
        except ErreurIA as erreur:
            journal.info("profondeur et segmentation indisponibles : %s", erreur)
            return
        distances = (reponse.get("profondeur") or {}).get("points") or []
        masques = [m for m in reponse.get("segmentation") or [] if "point" in m]
        for point, distance in zip(points, distances, strict=False):
            point["distance_m"] = distance
        for point, masque in zip(points, masques, strict=False):
            if masque.get("contour"):
                point["contour"] = masque["contour"]

    def _publier(self, demande: str, camera: str, description: str, jpeg: bytes, modele: str,
                 duree_s: float, objet: str | None = None,
                 points: list[dict[str, Any]] | None = None,
                 boites: list[dict[str, Any]] | None = None,
                 *, silencieux: bool = False) -> dict[str, Any]:
        message: dict[str, Any] = {
            "type": "vision", "id": nouvel_id("v"), "t": time.time(), "camera": camera,
            "demande": demande, "description": description,
            "image": base64.b64encode(jpeg).decode("ascii"),
            "points": points or [], "boites": boites or [], "modele": modele,
            "duree_s": round(duree_s, 2),
        }
        if objet is not None:
            message["objet"] = objet
        self.dernier = message
        self.bus.publier(S.VISION, message)
        if not silencieux:
            publier_chat(self.bus, "rover", description)
        return message

    def _signaler(self, erreur: ErreurIA) -> None:
        publier_chat(self.bus, "systeme", f"Vision indisponible : {erreur}.")

    async def decrire(self, camera: str = "gauche", question: str | None = None
                      ) -> dict[str, Any]:
        """Décrit la scène ; publie `vision` et `chat`. Renvoie le message `vision`."""
        async with self._verrou:
            try:
                consigne = consigne_description(camera, self._hauteur_camera(), question)
                texte, jpeg, modele, duree = await self._analyser(camera, consigne)
            except ErreurIA as erreur:
                self._signaler(erreur)
                raise
            description = " ".join(texte.replace("*", "").split())[:600]
            return self._publier("decrire", camera, description, jpeg, modele, duree)

    async def analyser(self, camera: str = "gauche",
                       *, silencieux: bool = False) -> dict[str, Any]:
        """Détecte les objets de la scène par l'API Vision et les fusionne avec le lidar et la
        profondeur stéréo pour mettre à jour la scène 3D."""
        async with self._verrou:
            detecter_sim = getattr(self.outils, "detecter_camera", None)
            try:
                format_points = self.config_ia.passerelle().acces("vision").role.format_points
                texte, jpeg, modele, duree = await self._analyser(
                    camera, consigne_analyse_scene(format_points))
                try:
                    detections = lire_detections(extraire_json(texte), format_points)
                except ValueError as erreur:
                    raise ErreurIA("réponse de la vision illisible") from erreur
            except ErreurIA as erreur:
                if detecter_sim is not None:
                    pts_sim = detecter_sim(camera)
                    image = self.outils.derniere_image(camera)
                    if image is not None and pts_sim is not None:
                        jpeg_s, _, _ = await asyncio.to_thread(preparer_image, image, 640)
                        boites_s = [{"u0": p["boite"][0], "v0": p["boite"][1],
                                     "u1": p["boite"][2], "v1": p["boite"][3],
                                     "label": p["label"]}
                                    for p in pts_sim if p.get("boite")]
                        return self._publier("analyser", camera, texte_analyse(camera, pts_sim),
                                             jpeg_s, "fusion-lidar-stereo", 0.02,
                                             points=pts_sim, boites=boites_s,
                                             silencieux=silencieux)
                if not silencieux:
                    self._signaler(erreur)
                raise
            points: list[dict[str, Any]] = []
            boites: list[dict[str, Any]] = []
            mesurer = getattr(self.outils, "mesurer_objet", None)
            for det in detections:
                u, v, boite, label, cat = (det["u"], det["v"], det["boite"], det["label"],
                                           det["categorie"])
                if boite is not None:
                    boites.append({"u0": boite[0], "v0": boite[1], "u1": boite[2],
                                   "v1": boite[3], "label": label})
                mesure = None
                if mesurer is not None:
                    try:
                        mesure = mesurer(camera, u, v, boite, cat)
                    except Exception:
                        journal.exception("mesurer_objet en erreur")
                if mesure is not None:
                    points.append({"u": u, "v": v, "label": label, "categorie": cat, **mesure})
                else:
                    try:
                        sol = self.outils.pixel_vers_sol(camera, u, v)
                    except Exception:
                        sol = None
                    points.append({
                        "u": u, "v": v, "label": label, "categorie": cat,
                        "sol": [round(float(sol[0]), 2), round(float(sol[1]), 2)]
                        if sol is not None else None,
                    })
            if detecter_sim is not None:
                for p_sim in detecter_sim(camera):
                    sol_s = p_sim.get("sol")
                    if not sol_s:
                        continue
                    deja = any(
                        p.get("sol") and math.hypot(p["sol"][0] - sol_s[0],
                                                    p["sol"][1] - sol_s[1]) < 0.55
                        for p in points
                    )
                    if not deja:
                        points.append(p_sim)
                        if p_sim.get("boite"):
                            b = p_sim["boite"]
                            boites.append({"u0": b[0], "v0": b[1], "u1": b[2], "v1": b[3],
                                           "label": p_sim["label"]})
            await self._enrichir(jpeg, points)
            return self._publier("analyser", camera, texte_analyse(camera, points), jpeg, modele,
                                 duree, points=points, boites=boites, silencieux=silencieux)

    async def chercher(self, objet: str, camera: str = "gauche") -> list[dict[str, Any]]:
        """Pointe l'objet ; renvoie [{"u", "v", "label", "sol": [x, y] | None}] (sol dans
        base_link, au moment de l'image). Liste vide si l'objet n'est pas vu. Lève ErreurIA si
        l'analyse est impossible (pas d'image, pas de modèle, réponse illisible)."""
        objet = " ".join(objet.split())[:80]
        async with self._verrou:
            try:
                role = self._role_pointage()
                format_points = self.config_ia.passerelle().acces(
                    role).role.format_points  # type: ignore[arg-type]
                texte, jpeg, modele, duree = await self._analyser(
                    camera, consigne_pointage(objet, format_points), role)
                try:
                    bruts = (lire_points_tuples(texte, objet) if format_points == "robobrain"
                             else lire_points(extraire_json(texte), format_points, objet))
                except ValueError as erreur:
                    raise ErreurIA("réponse de la vision illisible") from erreur
            except ErreurIA as erreur:
                self._signaler(erreur)
                raise
            points = []
            mesurer = getattr(self.outils, "mesurer_objet", None)
            for u, v, label in bruts:
                mesure = None
                if mesurer is not None:
                    try:
                        mesure = mesurer(camera, u, v, None, None)
                    except Exception:
                        mesure = None
                if mesure is not None and mesure.get("sol") is not None:
                    sol_pt = mesure["sol"]
                else:
                    try:
                        sol = self.outils.pixel_vers_sol(camera, u, v)
                    except Exception:  # projection de l'intégrateur en panne : point sans sol
                        journal.exception("pixel_vers_sol en erreur")
                        sol = None
                    sol_pt = [round(float(sol[0]), 2), round(float(sol[1]), 2)] \
                        if sol is not None else None
                entree: dict[str, Any] = {"u": u, "v": v, "label": label, "sol": sol_pt}
                if mesure is not None:
                    for cle in ("hauteur", "largeur", "longueur", "cap", "type_3d", "source"):
                        if cle in mesure:
                            entree[cle] = mesure[cle]
                points.append(entree)
            await self._enrichir(jpeg, points)
            self._publier("chercher", camera, texte_recherche(objet, camera, points), jpeg,
                          modele, duree, objet, points)
            return points
