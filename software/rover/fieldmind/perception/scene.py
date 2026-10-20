"""Scène reconstruite autour du rover pour la vue 3D du poste (message `scene`, façon Tesla).

À partir de ce que le rover mesure déjà :
- **lidar** : chaque tour est découpé en groupes de points voisins ; un groupe long et droit
  devient un ou plusieurs **murs** (découpage récursif en segments), un petit groupe compact un
  **arbre** (tronc, poteau, pied de meuble), le reste un **bloc** (boîte orientée) ;
- **stéréo** : les obstacles bas (sous le plan du lidar) sont regroupés en **blocs bas** avec
  leur hauteur mesurée ;
- **vision IA** : les objets nommés (« chercher », « décrire ») deviennent des **étiquettes** ;
- **carte d'occupation** : les murs sont **prolongés** au-delà de ce qui a été vu tant que la
  carte ne dit pas que la place est libre (extrapolation, dessinée en transparence).

Les objets sont suivis dans le repère odom d'un tour à l'autre (association au plus proche),
lissés, fusionnés (un mur s'allonge au fil du trajet) et oubliés 30 s après la dernière vue.
Ce n'est qu'une représentation : la sécurité travaille sur les mesures brutes, pas sur la scène.
"""

from __future__ import annotations

import itertools
import math
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from fieldmind.config import Profil
from fieldmind.types import (
    CarteOccupation,
    EtatSecurite,
    Obstacles,
    Pose2D,
    Scan,
    normaliser_angle,
)

OUBLI_S = 30.0
PORTEE_M = 15.0
HAUTEUR_MUR_M = 1.0


@dataclass
class Objet:
    id: int
    type: str  # mur | arbre | bloc | bas | etiquette
    x: float
    y: float
    cap: float = 0.0
    longueur: float = 0.0
    largeur: float = 0.0
    hauteur: float = 0.0
    rayon: float = 0.0
    texte: str = ""
    vu: float = field(default_factory=time.monotonic)
    vues: int = 1

    def message(self, maintenant: float) -> dict[str, Any]:
        m: dict[str, Any] = {"id": self.id, "type": self.type, "x": round(self.x, 3),
                             "y": round(self.y, 3), "age_s": round(maintenant - self.vu, 2)}
        if self.type == "mur":
            m |= {"cap": round(self.cap, 4), "longueur": round(self.longueur, 3),
                  "epaisseur": 0.12, "hauteur": self.hauteur, "extrapole": False}
        elif self.type == "arbre":
            m |= {"rayon": round(self.rayon, 3), "hauteur": round(self.hauteur, 2)}
        elif self.type in ("personne", "animal"):
            m |= {"rayon": round(self.rayon, 2), "hauteur": round(self.hauteur, 2),
                  "texte": self.texte}
        elif self.type in ("bloc", "bas"):
            m |= {"cap": round(self.cap, 4), "longueur": round(self.longueur, 3),
                  "largeur": round(self.largeur, 3), "hauteur": round(self.hauteur, 3)}
        else:
            m["texte"] = self.texte
        return m


def _hasard_stable(ident: int) -> float:
    """Nombre dans [0, 1[ fixe pour un objet (taille des arbres, hauteur des blocs)."""
    return ((ident * 2654435761) % 2**32) / 2**32


# ---------------------------------------------------------------- géométrie


def segmenter(points: np.ndarray, angles: np.ndarray, distances: np.ndarray) -> list[np.ndarray]:
    """Groupes de points consécutifs (dans l'ordre angulaire) séparés par un saut de distance.
    Seuil adaptatif : 12 cm ou 4 % de la portée, plus l'écart dû au pas angulaire."""
    if len(points) < 3:
        return []
    ecarts = np.hypot(*np.diff(points, axis=0).T)
    pas = np.abs(np.diff(angles))
    seuil = np.maximum(0.12, 0.04 * distances[1:]) + 1.5 * distances[1:] * pas
    coupures = np.nonzero(ecarts > seuil)[0] + 1
    groupes = np.split(points, coupures)
    # Le tour est circulaire : recoller le dernier groupe au premier s'ils se touchent
    if len(groupes) > 1 and np.hypot(*(groupes[0][0] - groupes[-1][-1])) < 0.15:
        groupes[0] = np.vstack((groupes[-1], groupes[0]))
        groupes.pop()
    return [g for g in groupes if len(g) >= 3]


def decouper_segments(points: np.ndarray, tolerance: float = 0.05) -> list[tuple[int, int]]:
    """Découpage récursif d'une suite de points en segments droits : indices (début, fin)."""
    pile = [(0, len(points) - 1)]
    segments = []
    while pile:
        i, j = pile.pop()
        if j - i < 2:
            segments.append((i, j))
            continue
        a, b = points[i], points[j]
        d = b - a
        n = math.hypot(*d)
        if n < 1e-6:
            segments.append((i, j))
            continue
        ecarts = np.abs(d[0] * (points[i:j + 1, 1] - a[1]) - d[1] * (points[i:j + 1, 0] - a[0])) / n
        k = int(np.argmax(ecarts))
        if ecarts[k] > tolerance and 0 < k < j - i:
            pile.append((i, i + k))
            pile.append((i + k, j))
        else:
            segments.append((i, j))
    return sorted(segments)


def ajuster_droite(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Centre, direction unitaire, longueur (étendue projetée) et épaisseur (écart-type)."""
    centre = points.mean(axis=0)
    _, valeurs, vecteurs = np.linalg.svd(points - centre, full_matrices=False)
    direction = vecteurs[0]
    projections = (points - centre) @ direction
    longueur = float(projections.max() - projections.min())
    centre = centre + direction * float(projections.max() + projections.min()) / 2
    epaisseur = float(valeurs[1] / math.sqrt(max(len(points), 1))) if len(valeurs) > 1 else 0.0
    return centre, direction, longueur, epaisseur


# ---------------------------------------------------------------- reconstruction


class ReconstructeurScene:
    def __init__(self, profil: Profil, mesurer_stereo: Any = None) -> None:
        self.profil = profil
        self.mesurer_stereo = mesurer_stereo
        self.objets: dict[int, Objet] = {}
        self._ids = itertools.count(1)
        self.libre: list[list[float]] = []

    # -- entrées

    def integrer_scan(self, scan: Scan, pose: Pose2D, t: float) -> None:
        points = scan.points()
        garde = scan.distances < PORTEE_M
        points, angles, distances = points[garde], scan.angles[garde], scan.distances[garde]
        self.libre = self._espace_libre(angles, distances, pose)
        for groupe in segmenter(points, angles, distances):
            for detection in self._classer(groupe):
                self._associer(detection, pose, t)

    def integrer_stereo(self, obstacles: Obstacles, pose: Pose2D, t: float) -> None:
        if not len(obstacles.points):
            return
        hauteurs = obstacles.hauteurs if obstacles.hauteurs is not None else \
            np.full(len(obstacles.points), 0.15)
        # Grille de 10 cm, composantes connexes (8-voisinage) par parcours simple
        cellules: dict[tuple[int, int], list[int]] = {}
        for i, (x, y) in enumerate(obstacles.points):
            cellules.setdefault((int(math.floor(x / 0.1)), int(math.floor(y / 0.1))), []).append(i)
        vues: set[tuple[int, int]] = set()
        for depart in cellules:
            if depart in vues:
                continue
            pile, composante = [depart], []
            vues.add(depart)
            while pile:
                c = pile.pop()
                composante.extend(cellules[c])
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        voisin = (c[0] + dx, c[1] + dy)
                        if voisin in cellules and voisin not in vues:
                            vues.add(voisin)
                            pile.append(voisin)
            if len(composante) < 3:
                continue
            p = obstacles.points[composante]
            centre, direction, longueur, _ = ajuster_droite(p) if len(p) >= 2 else (
                p[0], np.array([1.0, 0.0]), 0.1, 0.0)
            largeur = float(np.ptp((p - centre) @ np.array([-direction[1], direction[0]])))
            self._associer({"type": "bas", "x": float(centre[0]), "y": float(centre[1]),
                            "cap": math.atan2(direction[1], direction[0]),
                            "longueur": max(longueur, 0.1), "largeur": max(largeur, 0.1),
                            "hauteur": float(np.max(hauteurs[composante]))}, pose, t)

    def integrer_detections(self, message: dict[str, Any], pose: Pose2D, t: float) -> None:
        """Détections continues : personnes et animaux placés au sol, suivis comme les autres."""
        types = {"personne": "personne", "person": "personne", "enfant": "personne",
                 "chien": "animal", "dog": "animal", "chat": "animal", "cat": "animal",
                 "animal": "animal", "cheval": "animal", "vache": "animal", "oiseau": "animal",
                 "mouton": "animal"}
        for objet in message.get("objets") or []:
            sol = objet.get("sol")
            genre = types.get(str(objet.get("label", "")).lower())
            if not sol or genre is None:
                continue
            x, y = pose.vers_parent(float(sol[0]), float(sol[1]))
            proche = [o for o in self.objets.values() if o.type == genre
                      and math.hypot(o.x - x, o.y - y) < 0.8]
            if proche:
                o = proche[0]
                o.x, o.y, o.vu, o.vues = 0.6 * o.x + 0.4 * x, 0.6 * o.y + 0.4 * y, t, o.vues + 1
            else:
                ident = next(self._ids)
                self.objets[ident] = Objet(ident, genre, x, y, hauteur=1.7 if genre == "personne"
                                           else 0.6, rayon=0.25, texte=str(objet["label"]), vu=t)

    def integrer_vision(self, message: dict[str, Any], pose: Pose2D, t: float) -> None:
        for point in message.get("points") or []:
            sol = point.get("sol")
            if not sol or not point.get("label"):
                continue
            x, y = pose.vers_parent(float(sol[0]), float(sol[1]))
            proche = [o for o in self.objets.values() if o.type == "etiquette"
                      and (o.texte == point["label"] or math.hypot(o.x - x, o.y - y) < 0.50)
                      and math.hypot(o.x - x, o.y - y) < 1.0]
            if proche:
                o = min(proche, key=lambda c: math.hypot(c.x - x, c.y - y))
                o.texte = str(point["label"])[:40]
                o.x, o.y, o.vu = 0.5 * (o.x + x), 0.5 * (o.y + y), t
            else:
                ident = next(self._ids)
                self.objets[ident] = Objet(ident, "etiquette", x, y, texte=str(point["label"])[:40],
                                           vu=t)
            type_3d = point.get("type_3d")
            if type_3d not in ("bloc", "arbre", "bas"):
                continue
            hauteur = float(point.get("hauteur") or (0.18 if type_3d == "bas" else 0.7))
            largeur = float(point.get("largeur") or 0.35)
            longueur = float(point.get("longueur") or largeur)
            cap = normaliser_angle(float(point.get("cap") or 0.0) + pose.cap)
            candidats = [o for o in self.objets.values()
                         if o.type in ("bloc", "arbre", "bas")
                         and math.hypot(o.x - x, o.y - y) < 0.75]
            if candidats:
                cible = min(candidats, key=lambda o: math.hypot(o.x - x, o.y - y))
                if type_3d == "bloc":
                    for autre in candidats:
                        if autre.id != cible.id and autre.type == "arbre":
                            self.objets.pop(autre.id, None)
                cible.type = type_3d
                cible.x, cible.y = 0.65 * cible.x + 0.35 * x, 0.65 * cible.y + 0.35 * y
                cible.hauteur = 0.4 * cible.hauteur + 0.6 * hauteur
                if type_3d == "arbre":
                    cible.rayon = max(0.04, min(0.25, 0.5 * min(largeur, longueur)))
                else:
                    cible.longueur = max(cible.longueur, longueur)
                    cible.largeur = max(cible.largeur, largeur)
                    cible.cap = _moyenne_axes(cible.cap, cap, 0.35)
                cible.vu = t
            else:
                ident = next(self._ids)
                rayon = max(0.04, min(0.25, 0.5 * min(largeur, longueur)))
                self.objets[ident] = Objet(
                    ident, type_3d, x, y, cap=cap,
                    longueur=longueur if type_3d != "arbre" else 0.0,
                    largeur=largeur if type_3d != "arbre" else 0.0,
                    hauteur=hauteur,
                    rayon=rayon if type_3d == "arbre" else 0.0,
                    vu=t,
                )

    # -- classement

    def _classer(self, groupe: np.ndarray) -> list[dict[str, Any]]:
        centre, direction, longueur, _ = ajuster_droite(groupe)
        detections: list[dict[str, Any]] = []
        if longueur < 1.2:
            return self._petit_objet(groupe, centre, direction, longueur)
        for i, j in decouper_segments(groupe):
            morceau = groupe[i:j + 1]
            if len(morceau) < 4:
                continue
            c, d, longueur_s, epaisseur_s = ajuster_droite(morceau)
            if longueur_s >= 0.8 and epaisseur_s < 0.05:
                detections.append({"type": "mur", "x": float(c[0]), "y": float(c[1]),
                                   "cap": math.atan2(d[1], d[0]), "longueur": longueur_s})
        if detections:
            return detections
        return self._petit_objet(groupe, centre, direction, longueur)

    def _petit_objet(self, groupe: np.ndarray, centre: np.ndarray, direction: np.ndarray,
                     longueur: float) -> list[dict[str, Any]]:
        m3d = (self.mesurer_stereo(float(centre[0]), float(centre[1]), max(0.20, 0.5 * longueur))
               if self.mesurer_stereo is not None else None)
        h_stereo = float(m3d["hauteur"]) if m3d is not None else None
        etendue_3d = float(m3d["etendue"]) if m3d is not None else 0.0
        if longueur < 0.30 and (h_stereo is None or (h_stereo >= 1.15 and etendue_3d < 0.38)):
            # Petit objet compact (poteau, pilier, cylindre) : on en voit la face avant
            rayon = max(0.04, min(0.20, 0.5 * longueur))
            portee = math.hypot(*centre)
            cx, cy = centre + centre / max(portee, 1e-6) * rayon * 0.7
            d_arbre: dict[str, Any] = {"type": "arbre", "x": float(cx), "y": float(cy),
                                       "rayon": rayon}
            if h_stereo is not None:
                d_arbre["hauteur"] = h_stereo
            return [d_arbre]
        normale = np.array([-direction[1], direction[0]])
        largeur = max(0.25, float(np.ptp((groupe - centre) @ normale)))
        if etendue_3d > 0.25:
            longueur = max(longueur, min(0.65, etendue_3d))
        cx, cy = centre + centre / max(math.hypot(*centre), 1e-6) * largeur * 0.4
        d_bloc: dict[str, Any] = {"type": "bloc", "x": float(cx), "y": float(cy),
                                  "cap": math.atan2(direction[1], direction[0]),
                                  "longueur": max(0.25, longueur), "largeur": largeur}
        if h_stereo is not None:
            d_bloc["hauteur"] = h_stereo
        return [d_bloc]

    # -- suivi

    def _associer(self, d: dict[str, Any], pose: Pose2D, t: float) -> None:
        x, y = pose.vers_parent(d["x"], d["y"])
        cap = normaliser_angle(d.get("cap", 0.0) + pose.cap)
        if d["type"] == "mur":
            self._associer_mur(x, y, cap, d["longueur"], t)
            return
        meilleur, distance = None, 0.5
        for o in self.objets.values():
            if o.type != d["type"]:
                continue
            e = math.hypot(o.x - x, o.y - y)
            if e < distance:
                meilleur, distance = o, e
        if meilleur is None:
            ident = next(self._ids)
            h = _hasard_stable(ident)
            o = Objet(ident, d["type"], x, y, cap, vu=t)
            if d["type"] == "arbre":
                o.rayon = d["rayon"]
                o.hauteur = float(d.get("hauteur") or (0.75 + 0.35 * h))
            elif d["type"] == "bloc":
                o.longueur, o.largeur = d["longueur"], d["largeur"]
                o.hauteur = float(d.get("hauteur") or (0.5 + 0.4 * h))
            else:
                o.longueur, o.largeur, o.hauteur = d["longueur"], d["largeur"], d["hauteur"]
            self.objets[ident] = o
            return
        o = meilleur
        a = 0.3  # lissage
        o.x += a * (x - o.x)
        o.y += a * (y - o.y)
        o.vu, o.vues = t, o.vues + 1
        if o.type == "arbre":
            o.rayon += a * (d["rayon"] - o.rayon)
            if d.get("hauteur") is not None:
                o.hauteur += a * (float(d["hauteur"]) - o.hauteur)
        else:
            o.cap = _moyenne_axes(o.cap, cap, a)
            o.longueur = max(o.longueur * 0.9, d["longueur"])  # on voit rarement tout l'objet
            o.largeur = max(o.largeur * 0.9, d["largeur"])
            if o.type == "bas" or d.get("hauteur") is not None:
                o.hauteur += a * (float(d["hauteur"]) - o.hauteur)

    def _associer_mur(self, x: float, y: float, cap: float, longueur: float, t: float) -> None:
        u = np.array([math.cos(cap), math.sin(cap)])
        for o in self.objets.values():
            if o.type != "mur":
                continue
            ecart_axes = abs(normaliser_angle(2 * (o.cap - cap))) / 2
            if ecart_axes > math.radians(12):
                continue
            uo = np.array([math.cos(o.cap), math.sin(o.cap)])
            normale = np.array([-uo[1], uo[0]])
            if abs(float((np.array([x, y]) - [o.x, o.y]) @ normale)) > 0.2:
                continue
            # Recouvrement le long de l'axe : fusion en un mur plus long
            s0 = float((np.array([x, y]) - [o.x, o.y]) @ uo)
            a1, b1 = -o.longueur / 2, o.longueur / 2
            a2, b2 = s0 - longueur / 2, s0 + longueur / 2
            if a2 > b1 + 0.3 or b2 < a1 - 0.3:
                continue
            bas, haut = min(a1, a2), max(b1, b2)
            if haut - bas > 30:  # garde-fou
                continue
            milieu = (bas + haut) / 2
            o.x, o.y = o.x + uo[0] * milieu, o.y + uo[1] * milieu
            o.longueur = haut - bas
            o.cap = _moyenne_axes(o.cap, cap, 0.2 * longueur / max(o.longueur, 0.1))
            o.vu, o.vues = t, o.vues + 1
            return
        ident = next(self._ids)
        self.objets[ident] = Objet(ident, "mur", x, y, math.atan2(u[1], u[0]), longueur,
                                   hauteur=HAUTEUR_MUR_M, vu=t)

    def _consolider_murs(self) -> None:
        """Fusionne les murs existants qui se recouvrent (ils ont grandi l'un vers l'autre)."""
        murs = sorted((o for o in self.objets.values() if o.type == "mur"),
                      key=lambda o: -o.longueur)
        for i, a in enumerate(murs):
            if a.id not in self.objets:
                continue
            ua = np.array([math.cos(a.cap), math.sin(a.cap)])
            na = np.array([-ua[1], ua[0]])
            for b in murs[i + 1:]:
                if b.id not in self.objets:
                    continue
                if abs(normaliser_angle(2 * (a.cap - b.cap))) / 2 > math.radians(10):
                    continue
                ecart = np.array([b.x - a.x, b.y - a.y])
                if abs(float(ecart @ na)) > 0.2:
                    continue
                s0 = float(ecart @ ua)
                bas = min(-a.longueur / 2, s0 - b.longueur / 2)
                haut = max(a.longueur / 2, s0 + b.longueur / 2)
                if haut - bas > a.longueur + b.longueur + 0.3:
                    continue  # trop loin l'un de l'autre le long de l'axe
                milieu = (bas + haut) / 2
                a.x, a.y = a.x + ua[0] * milieu, a.y + ua[1] * milieu
                a.longueur = haut - bas
                a.vues += b.vues
                a.vu = max(a.vu, b.vu)
                del self.objets[b.id]

    # -- sortie

    def _espace_libre(self, angles: np.ndarray, distances: np.ndarray,
                      pose: Pose2D) -> list[list[float]]:
        """Polygone du sol libre : par secteur de 4°, la mesure la plus proche (5 m sans écho)."""
        secteurs = 90
        rayons = np.full(secteurs, 5.0)
        if len(angles):
            k = ((angles + math.pi) / (2 * math.pi) * secteurs).astype(int) % secteurs
            np.minimum.at(rayons, k, np.minimum(distances, 8.0))
        centres = -math.pi + (np.arange(secteurs) + 0.5) * 2 * math.pi / secteurs
        polygone = []
        for a, r in zip(centres, rayons, strict=True):
            x, y = pose.vers_parent(r * math.cos(a), r * math.sin(a))
            polygone.append([round(x, 2), round(y, 2)])
        return polygone

    def _prolongements(self, carte: CarteOccupation | None, maintenant: float
                       ) -> list[dict[str, Any]]:
        sortie = []
        for o in self.objets.values():
            if o.type != "mur" or o.vues < 3 or o.longueur < 0.8:
                continue
            u = (math.cos(o.cap), math.sin(o.cap))
            for cote in (-1, 1):
                bout = (o.x + cote * u[0] * o.longueur / 2, o.y + cote * u[1] * o.longueur / 2)
                prolongement = 0.0
                while prolongement < 1.5:
                    s = prolongement + 0.1
                    px, py = bout[0] + cote * u[0] * s, bout[1] + cote * u[1] * s
                    if carte is not None:
                        i, j = carte.indice(px, py)
                        if 0 <= i < carte.cellules.shape[0] and 0 <= j < carte.cellules.shape[1]:
                            valeur = int(carte.cellules[i, j])
                            if 0 <= valeur < 30:  # vu libre : le mur s'arrête là
                                break
                    prolongement = s
                if prolongement >= 0.3:
                    cx = bout[0] + cote * u[0] * prolongement / 2
                    cy = bout[1] + cote * u[1] * prolongement / 2
                    sortie.append({"id": o.id * 10 + (1 if cote < 0 else 2), "type": "mur",
                                   "x": round(cx, 3), "y": round(cy, 3), "cap": round(o.cap, 4),
                                   "longueur": round(prolongement, 2), "epaisseur": 0.12,
                                   "hauteur": o.hauteur, "extrapole": True,
                                   "age_s": round(maintenant - o.vu, 2)})
        return sortie

    def message(self, pose: Pose2D | None, carte: CarteOccupation | None,
                securite: EtatSecurite | None, chemin: Any = None,
                but: tuple[float, float] | None = None, maintenant: float | None = None
                ) -> dict[str, Any]:
        maintenant = time.monotonic() if maintenant is None else maintenant
        p = pose or Pose2D()
        for ident in [i for i, o in self.objets.items()
                      if maintenant - o.vu > (5.0 if o.type in ("personne", "animal") else OUBLI_S)
                      or math.hypot(o.x - p.x, o.y - p.y) > 25]:
            del self.objets[ident]
        self._consolider_murs()
        objets = [o.message(maintenant) for o in self.objets.values()
                  if o.type != "mur" or o.vues >= 2 or maintenant - o.vu < 0.5]
        objets += self._prolongements(carte, maintenant)
        self._marquer_dangers(objets, p, securite)
        return {"type": "scene", "t": round(maintenant, 3),
                "repere": "odom" if pose is not None else "base_link",
                "rover": {"x": round(p.x, 3), "y": round(p.y, 3), "cap": round(p.cap, 4)},
                "objets": objets, "libre": self.libre,
                "chemin": None if chemin is None else np.round(chemin.points, 2).tolist(),
                "but": None if but is None else [round(but[0], 2), round(but[1], 2)]}

    def _marquer_dangers(self, objets: list[dict[str, Any]], pose: Pose2D,
                         securite: EtatSecurite | None) -> None:
        if securite is None:
            return
        raisons = set(securite.raisons)
        avant = raisons & {"obstacle_avant", "obstacle_bas", "ralenti_obstacle"}
        arriere = "obstacle_arriere" in raisons
        if not avant and not arriere:
            return
        g = self.profil.geometrie
        demi = g.emprise_demi_largeur_m + 0.3
        for objet in objets:
            if objet["type"] == "etiquette" or objet.get("extrapole"):
                continue
            x, y = pose.vers_local(objet["x"], objet["y"])
            portee = max(objet.get("longueur", 0.0), 2 * objet.get("rayon", 0.0)) / 2
            dans_couloir = abs(y) < demi + portee
            devant = avant and 0 < x < g.emprise_avant_m + (securite.libre_avant_m or 0) + 0.6 \
                + portee
            derriere = arriere and -(securite.libre_arriere_m or 0) - 0.6 - portee < x < 0
            if dans_couloir and (devant or derriere):
                objet["danger"] = True


def _moyenne_axes(a: float, b: float, poids: float) -> float:
    """Moyenne pondérée de deux orientations d'axe (définies à π près)."""
    ecart = normaliser_angle(2 * (b - a)) / 2
    return normaliser_angle(a + poids * ecart)
