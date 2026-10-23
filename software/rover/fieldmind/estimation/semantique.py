"""Mémoire de session alimentée par les observations, dans le repère odom.

Les positions viennent des capteurs/projections, jamais de distances inventées par le modèle.
Une remise à zéro de l'odométrie doit effacer cette mémoire avec la carte. Les observations
mobiles expirent vite ; les objets et segments mesurés restent disponibles hors champ.
"""

from __future__ import annotations

import copy
import math
import threading
import time
import unicodedata
from typing import Any

from fieldmind.bus import Bus, S
from fieldmind.types import Pose2D, normaliser_angle

OUBLI_STATIQUE_S = 3600.0
OUBLI_MOBILE_S = 5.0
OUBLI_HYPOTHESE_S = 45.0
MOBILES = {"personne", "homme", "femme", "enfant", "chien", "chat", "person", "dog", "cat",
           "animal", "cheval", "vache", "oiseau", "mouton", "people", "bicycle", "car"}
DIAMETRE_PIECE_M = 4.0
INTERVALLE_CONFIRMATION_S = 0.5
TERRAIN_CONTINU = {"herbe", "herbes", "gazon", "pelouse", "grass", "brindille", "brindilles",
                   "brin", "brins", "touffe", "sec", "seche", "seches", "dense", "haute",
                   "haut", "basse", "bas"}
# Ces mots ne caractérisent aucune pièce à eux seuls. Le GX10 reste responsable
# de l'interprétation sémantique ; cette liste empêche seulement une fausse certitude.
INDICES_GENERIQUES = {
    "table", "tables", "chaise", "chaises", "chair", "chairs", "desk", "bureau",
    "porte", "portes", "door", "doors", "mur", "murs", "wall", "walls",
    "fenetre", "fenetres", "window", "windows", "sol", "floor", "plafond", "ceiling",
    "meuble", "meubles", "furniture", "objet", "objets", "object", "objects",
    "lampe", "lamp", "lumiere", "light", "carton", "box", "caisse", "placard",
    "rangement", "shelf", "etagere", "cabinet", "couloir", "passage",
}
QUALIFICATIFS = {"blanc", "blanche", "noir", "noire", "rouge", "bleu", "bleue", "vert",
                 "verte", "gris", "grise", "petit", "petite", "grand", "grande", "bois",
                 "metal", "white", "black", "red", "blue", "small", "large"}


def mots_objet(texte: str) -> set[str]:
    texte = unicodedata.normalize("NFKD", texte.lower())
    texte = "".join(c if c.isalnum() else " " for c in texte if not unicodedata.combining(c))
    return set(texte.split()) - {"le", "la", "les", "un", "une", "des", "de", "du", "l", "d"}


def _classe_objet(label: str) -> set[str]:
    """Identité approximative, sans associer un type d'objet à une pièce."""
    mots = mots_objet(label) - QUALIFICATIFS
    if {"microondes", "microwave"} & mots or {"micro", "ondes"} <= mots:
        return {"microondes"}
    if {"four", "oven"} & mots:
        return {"four"}
    return {m[:-1] if len(m) > 3 and m.endswith("s") else m for m in mots}


def _cle_piece(nom: str) -> tuple[str, ...]:
    return tuple(sorted(mots_objet(nom)))


def normaliser_piece(nom: str) -> tuple[str, ...]:
    return _cle_piece(nom)


def indices_piece_distinctifs(labels: list[str]) -> bool:
    generiques = {mot for label in INDICES_GENERIQUES for mot in _classe_objet(label)}
    return any(classe and not classe & generiques for label in labels
               if (classe := _classe_objet(label)))


def _nombre(valeur: Any) -> float | None:
    if isinstance(valeur, bool):
        return None
    try:
        nombre = float(valeur)
    except (TypeError, ValueError, OverflowError):
        return None
    return nombre if math.isfinite(nombre) else None


def _elements(message: dict[str, Any], cle: str, autre: str = "objets") -> list[Any]:
    valeurs = message.get(cle, message.get(autre))
    return valeurs[:256] if isinstance(valeurs, list | tuple) else []


class MemoireSemantique:
    def __init__(self, bus: Bus) -> None:
        self.bus = bus
        self._verrou = threading.RLock()
        self._objets: list[dict[str, Any]] = []
        self._murs: dict[int, dict[str, Any]] = {}
        self._pieces: list[dict[str, Any]] = []
        self._generation = 0
        self._ident = 0
        # Le premier message peut porter une image capturée avant la création du module.
        self._efface_a = float("-inf")
        bus.abonner(S.VISION, self.integrer)
        bus.abonner(S.DETECTIONS, self._detections)
        bus.abonner(S.SCENE, self._scene)

    def effacer(self) -> None:
        with self._verrou:
            self._objets.clear()
            self._murs.clear()
            self._pieces.clear()
            self._generation += 1
            self._efface_a = time.monotonic()

    def _date(self, message: dict[str, Any], maintenant: float) -> float | None:
        # image_t est monotonic sur le Pi ; le champ t de vision est une date Unix.
        cle = "image_t" if "image_t" in message else "t_image"
        date = _nombre(message.get(cle)) if cle in message else maintenant
        if date is None or date < self._efface_a or date > maintenant + 0.5:
            return None
        return min(date, maintenant)

    def _pose(self, message: dict[str, Any]) -> Pose2D | None:
        if "pose_image" in message:
            acquisition = message["pose_image"]
            if not isinstance(acquisition, dict):
                return None
            coord = [_nombre(acquisition.get(cle)) for cle in ("x", "y", "cap")]
            if any(v is None for v in coord):
                return None
            return Pose2D(*coord)
        odom = self.bus.dernier(S.ODOM)
        if odom is None or self.bus.age(S.ODOM) >= 1.0:
            return None
        pose = getattr(odom, "pose", None)
        if not isinstance(pose, Pose2D) or not all(math.isfinite(v) for v in (
                pose.x, pose.y, pose.cap)):
            return None
        return pose

    def _purger(self, maintenant: float) -> None:
        self._objets = [o for o in self._objets
                        if maintenant - o.get("_vu_visuel", o["vu"]) < (
            OUBLI_MOBILE_S if o["mobile"] else OUBLI_STATIQUE_S
            if o["statut"] == "confirme" else OUBLI_HYPOTHESE_S)]
        self._murs = {i: m for i, m in self._murs.items()
                      if maintenant - m["vu"] < OUBLI_STATIQUE_S}
        # Une pièce ne survit jamais à la disparition de ses preuves ni à une
        # nouvelle mesure montrant qu'elles sont séparées par un mur.
        self._pieces = [p for p in self._pieces if self._preuves_piece(p["objets"]) is not None]

    def _detections(self, message: dict[str, Any]) -> None:
        # Le producteur réel ia/distant.py n'a pas toujours de champ type.
        # Le sujet du bus fait foi : D-FINE fournit des propositions géométriques,
        # jamais une validation sémantique autonome.
        if isinstance(message, dict):
            self.integrer({**message, "type": "detections"})

    def _noter_vision(self, objet: dict[str, Any], capture: float, date: float,
                     pose: Pose2D) -> None:
        precedente = objet.get("_capture_visuelle")
        if precedente is None or capture - precedente >= INTERVALLE_CONFIRMATION_S:
            objet["observations_visuelles"] += 1
            objet["_capture_visuelle"] = capture
        if date >= objet.get("_vu_visuel", float("-inf")):
            objet["_vu_visuel"] = date
            objet["_pose_visuelle"] = {"x": round(pose.x, 3), "y": round(pose.y, 3),
                                        "cap": round(pose.cap, 4)}
        objet["statut"] = "confirme" if objet["observations_visuelles"] >= 2 else "hypothese"

    def integrer(self, message: dict[str, Any]) -> None:
        if not isinstance(message, dict) or message.get("geometrie_valide") is False:
            return
        pose = self._pose(message)
        if pose is None:
            return
        maintenant = time.monotonic()
        with self._verrou:
            date = self._date(message, maintenant)
            if date is None:
                return
            # Garder l'identité d'acquisition exacte même lorsque _date tolère
            # puis borne une horloge de capture légèrement en avance.
            capture = _nombre(message.get("image_t", message.get("t_image", date)))
            visuelle = message.get("type") in (None, "vision")
            self._purger(maintenant)
            associes: set[int] = set()
            points_acquisition: list[dict[str, Any]] = []
            for point in _elements(message, "points"):
                if not isinstance(point, dict):
                    continue
                sol, label = point.get("sol"), point.get("label")
                if not isinstance(label, str) or not mots_objet(label):
                    continue
                label = " ".join(label.split())[:80]
                # L'herbe couvre une surface continue : ne pas créer un repère de
                # navigation ponctuel pour chaque touffe détectée. Les obstacles
                # physiques restent gérés indépendamment dans la carte lidar.
                if not mots_objet(label) - QUALIFICATIFS - TERRAIN_CONTINU:
                    continue
                if not isinstance(sol, list | tuple) or len(sol) != 2:
                    continue
                px, py = _nombre(sol[0]), _nombre(sol[1])
                if px is None or py is None or math.hypot(px, py) > 30:
                    continue
                # Doublons certains dans une même image uniquement : proximité
                # en pixels ET au sol. Sans pixels, deux chaises proches restent
                # deux objets distincts ; aucune fusion aveugle par distance seule.
                pixels = self._pixels(point)
                if pixels is not None and any(
                        ancien["label"] == mots_objet(label)
                        and math.dist(pixels, ancien["pixels"]) < 0.025
                        and math.hypot(px - ancien["x"], py - ancien["y"]) < 0.2
                        for ancien in points_acquisition):
                    continue
                if pixels is not None:
                    points_acquisition.append({"label": mots_objet(label), "pixels": pixels,
                                               "x": px, "y": py})
                mobile = bool(mots_objet(label) & MOBILES) or point.get("categorie") in (
                    "personne", "animal")
                if maintenant - date >= (OUBLI_MOBILE_S if mobile else OUBLI_STATIQUE_S):
                    continue
                x, y = pose.vers_parent(px, py)
                mots = mots_objet(label)
                candidats = [o for o in self._objets if o["id"] not in associes
                             and (mots <= mots_objet(o["label"]) or mots_objet(o["label"]) <= mots)
                             and o["mobile"] == mobile
                             and math.hypot(o["x"] - x, o["y"] - y) < 0.8]
                if candidats:
                    objet = min(candidats, key=lambda o: math.hypot(o["x"] - x, o["y"] - y))
                    associes.add(objet["id"])
                    if visuelle:
                        self._noter_vision(objet, capture, date, pose)
                        if len(mots) > len(mots_objet(objet["label"])):
                            objet["label"] = label
                    if date <= objet["vu"]:
                        continue  # réponse tardive : ne pas reculer une position plus récente
                    x, y = 0.6 * objet["x"] + 0.4 * x, 0.6 * objet["y"] + 0.4 * y
                    objet["vues"] += 1
                    if capture - objet["_capture_confirmation"] >= INTERVALLE_CONFIRMATION_S:
                        objet["observations_distinctes"] += 1
                        objet["_capture_confirmation"] = capture
                    if len(mots) > len(mots_objet(objet["label"])):
                        objet["label"] = label
                else:
                    self._ident += 1
                    objet = {"id": self._ident, "label": label, "vues": 1,
                             "observations_distinctes": 1, "_capture_confirmation": capture,
                             "observations_visuelles": 0, "statut": "hypothese"}
                    if visuelle:
                        self._noter_vision(objet, capture, date, pose)
                    self._objets.append(objet)
                    associes.add(objet["id"])
                source = str(point.get("source") or "projection_sol")[:80]
                objet.update(x=round(x, 3), y=round(y, 3), vu=date, mobile=mobile,
                             categorie=str(point.get("categorie") or objet.get("categorie")
                                           or "objet")[:40], source=source,
                             pose_observation={"x": round(pose.x, 3), "y": round(pose.y, 3),
                                               "cap": round(pose.cap, 4)})
                dimensions = {cle: valeur for cle in ("hauteur", "largeur", "longueur")
                              if (valeur := _nombre(point.get(cle))) is not None
                              and 0 < valeur <= 30}
                if dimensions:
                    objet.update({cle: round(v, 3) for cle, v in dimensions.items()})
                    # Une boîte vidéo projetée n'est pas une mesure complète du volume.
                    objet["dimensions_estimees"] = point.get("dimensions_estimees", True)
                    objet["source_dimensions"] = source
                cap = _nombre(point.get("cap"))
                if cap is not None:
                    objet["cap"] = round(normaliser_angle(cap + pose.cap), 4)
            self._objets = sorted(self._objets, key=lambda o: o["vu"], reverse=True)[:256]

    @staticmethod
    def _pixels(point: dict[str, Any]) -> tuple[float, float] | None:
        u, v = _nombre(point.get("u")), _nombre(point.get("v"))
        if u is None or v is None:
            boite = point.get("boite")
            if not isinstance(boite, list | tuple) or len(boite) != 4:
                return None
            valeurs = [_nombre(n) for n in boite]
            if any(n is None for n in valeurs):
                return None
            u, v = (valeurs[0] + valeurs[2]) / 2, (valeurs[1] + valeurs[3]) / 2
        return (u, v) if 0 <= u <= 1 and 0 <= v <= 1 else None

    def _scene(self, message: dict[str, Any]) -> None:
        if not isinstance(message, dict) or message.get("repere") != "odom":
            return
        maintenant = time.monotonic()
        with self._verrou:
            self._purger(maintenant)
            for mur in _elements(message, "objets"):
                if (not isinstance(mur, dict) or mur.get("type") != "mur"
                        or mur.get("extrapole") or mur.get("memorise")):
                    continue
                valeurs = [_nombre(mur.get(k)) for k in ("x", "y", "cap", "longueur")]
                age = _nombre(mur.get("age_s", 0))
                if any(v is None for v in valeurs) or age is None or age < 0:
                    continue
                x, y, cap, longueur = valeurs
                if not 0 < longueur <= 30 or age >= OUBLI_STATIQUE_S:
                    continue
                scene_t = _nombre(message.get("t", maintenant))
                if scene_t is None or scene_t > maintenant + 0.5:
                    continue
                date = min(scene_t, maintenant) - age
                if date < self._efface_a:
                    continue
                # Associer par géométrie : la scène locale peut oublier puis recréer un ID.
                candidat = None
                for ancien in self._murs.values():
                    if abs(normaliser_angle(2 * (cap - ancien["cap"]))) / 2 > math.radians(10):
                        continue
                    dx, dy = x - ancien["x"], y - ancien["y"]
                    c, s = math.cos(ancien["cap"]), math.sin(ancien["cap"])
                    decalage = c * dx + s * dy
                    if abs(-s * dx + c * dy) > 0.15:
                        continue
                    # Recouvrement seulement : ne jamais boucher une porte non mesurée.
                    if abs(decalage) > (longueur + ancien["longueur"]) / 2:
                        continue
                    candidat = ancien
                    bas = min(-ancien["longueur"] / 2, decalage - longueur / 2)
                    haut = max(ancien["longueur"] / 2, decalage + longueur / 2)
                    if haut - bas <= 30:
                        milieu = (bas + haut) / 2
                        ancien.update(x=ancien["x"] + c * milieu, y=ancien["y"] + s * milieu,
                                      longueur=haut - bas, vu=max(ancien["vu"], date))
                    break
                if candidat is None:
                    self._ident += 1
                    self._murs[self._ident] = {"id": self._ident, "x": x, "y": y,
                                              "cap": cap, "longueur": longueur, "vu": date,
                                              "source": "lidar", "hauteur_estimee": True}
            self._murs = dict(sorted(self._murs.items(), key=lambda item: item[1]["vu"])[-128:])

    def _mur_entre(self, a: dict[str, Any], b: dict[str, Any]) -> bool:
        """Intersection intérieure avec un segment mesuré, sans inventer de cloisons."""
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        for mur in self._murs.values():
            sx = math.cos(mur["cap"]) * mur["longueur"]
            sy = math.sin(mur["cap"]) * mur["longueur"]
            denom = dx * sy - dy * sx
            if abs(denom) < 1e-8:
                continue
            ox, oy = mur["x"] - sx / 2 - a["x"], mur["y"] - sy / 2 - a["y"]
            t = (ox * sy - oy * sx) / denom
            u = (ox * dy - oy * dx) / denom
            if 0.02 < t < 0.98 and 0 <= u <= 1:
                return True
        return False

    def _preuves_piece(self, ids: list[int]) -> list[dict[str, Any]] | None:
        objets = {o["id"]: o for o in self._objets}
        preuves = [objets[i] for i in ids if i in objets and not objets[i]["mobile"]]
        if len(preuves) != len(ids) or len(preuves) < 2:
            return None
        classes = [_classe_objet(o["label"]) for o in preuves]
        if not any(a and b and not (a & b) for i, a in enumerate(classes)
                   for b in classes[i + 1:]):
            return None  # deux fours ou deux descriptions du même appareil ne suffisent pas
        for i, a in enumerate(preuves):
            for b in preuves[i + 1:]:
                if (math.hypot(a["x"] - b["x"], a["y"] - b["y"]) > DIAMETRE_PIECE_M
                        or self._mur_entre(a, b)):
                    return None
        return preuves

    def _exporter_piece(self, piece: dict[str, Any], maintenant: float) -> dict[str, Any]:
        preuves = self._preuves_piece(piece["objets"]) or []
        distinctive = indices_piece_distinctifs([o["label"] for o in preuves])
        corroboration = all(o["statut"] == "confirme" for o in preuves)
        statut = ("confirmee" if piece["confiance"] == "forte" and distinctive and corroboration
                  else "hypothese")
        return {"id": piece["id"], "nom": piece["nom"], "statut": statut,
                "confiance": "forte" if statut == "confirmee" else "faible",
                "raison": piece["raison"], "objets": list(piece["objets"]),
                "preuves": [{"id": o["id"], "label": o["label"], "x": o["x"], "y": o["y"],
                             "source": o["source"], "statut": o["statut"],
                             "observations_distinctes": o["observations_distinctes"],
                             "observations_visuelles": o["observations_visuelles"],
                             "image_t": o.get("_vu_visuel", o["vu"]),
                             "pose_observation": copy.deepcopy(o.get(
                                 "_pose_visuelle", o["pose_observation"])),
                             "age_s": round(max(0.0, maintenant - o.get("_vu_visuel", o["vu"])), 1)}
                            for o in preuves]}

    def integrer_pieces(self, propositions: list[dict[str, Any]], *,
                        generation: int | None = None) -> list[dict[str, Any]]:
        """Valide les hypothèses du GX10 uniquement contre les objets déjà mesurés.

        Aucune coordonnée du modèle n'est utilisée. Une hypothèse faible reste
        visible pour guider l'exploration mais n'est jamais une destination confirmée.
        """
        if not isinstance(propositions, list):
            return []
        maintenant = time.monotonic()
        with self._verrou:
            if generation is not None and generation != self._generation:
                return []
            self._purger(maintenant)
            acceptees = []
            for proposition in propositions[:32]:
                if not isinstance(proposition, dict):
                    continue
                if {"x", "y", "sol", "position", "centre", "coordonnees"} & proposition.keys():
                    continue
                nom, ids = proposition.get("nom"), proposition.get("objets")
                confiance = proposition.get("confiance")
                raison = proposition.get("raison", "")
                if (not isinstance(nom, str) or not _cle_piece(nom)
                        or len(nom) > 80 or len(nom.split()) > 8
                        or confiance not in ("forte", "faible") or not isinstance(raison, str)
                        or not isinstance(ids, list) or not 2 <= len(ids) <= 32
                        or any(type(i) is not int for i in ids) or len(set(ids)) != len(ids)):
                    continue
                preuves = self._preuves_piece(ids)
                if preuves is None:
                    continue
                # Plusieurs pièces du même nom restent possibles ; ne pas fusionner
                # deux ensembles spatialement disjoints sous un seul nom.
                piece = next((p for p in self._pieces
                              if _cle_piece(p["nom"]) == _cle_piece(nom)
                              and set(p["objets"]) & set(ids)), None)
                if piece is None:
                    self._ident += 1
                    piece = {"id": self._ident}
                    self._pieces.append(piece)
                piece.update(nom=" ".join(nom.split()), objets=list(ids), confiance=confiance,
                             raison=" ".join(raison.split())[:400])
                acceptees.append(self._exporter_piece(piece, maintenant))
            self._pieces = self._pieces[-64:]
            return copy.deepcopy(acceptees)

    def rechercher_piece(self, nom: str) -> list[dict[str, Any]]:
        if not isinstance(nom, str) or not _cle_piece(nom):
            return []
        return [p for p in self.resume()["pieces"] if _cle_piece(p["nom"]) == _cle_piece(nom)]

    def resume(self) -> dict[str, Any]:
        maintenant = time.monotonic()
        with self._verrou:
            self._purger(maintenant)

            def exporter(objet: dict[str, Any]) -> dict[str, Any]:
                return {**{k: v for k, v in objet.items() if k != "vu" and not k.startswith("_")},
                        "age_s": round(max(0.0, maintenant - objet["vu"]), 1)}

            return copy.deepcopy({"repere": "odom", "generation": self._generation,
                                  "objets": [exporter(o) for o in self._objets],
                                  "murs": [exporter(m) for m in self._murs.values()],
                                  "pieces": [self._exporter_piece(p, maintenant)
                                             for p in self._pieces]})

    def rechercher(self, description: str) -> list[dict[str, Any]]:
        mots = mots_objet(description)
        return [o for o in self.resume()["objets"] if mots and mots <= mots_objet(o["label"])
                and not o["mobile"]]
