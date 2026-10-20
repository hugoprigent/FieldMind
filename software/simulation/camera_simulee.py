"""Caméra avant simulée : rendu MuJoCo hors écran, compressé en JPEG comme la vraie caméra.

Le rendu coûte plusieurs dizaines de millisecondes : il ne se fait jamais dans la boucle de la
physique. Le rover simulé photographie l'état (positions articulaires) à la cadence de la caméra ;
`CameraSimulee.rendre` produit l'image à partir de cet instantané, dans un fil dédié (un contexte
OpenGL appartient au fil qui l'a créé). Le message `Image` porte l'horodatage de la prise de vue.

Habillage : sol texturé (herbe, sous-bois, piste en terre, clairière), houppiers des conifères,
ciel et brume. Le rover lui-même n'apparaît pas dans l'image.
"""

from __future__ import annotations

import base64
import io
import itertools
import logging
import math
import time

import cv2
import mujoco
import numpy as np
from PIL import Image as ImagePil
from scipy import ndimage

from services.protocole.fiche_rover import Camera, Stereo
from services.protocole.messages import Image, ImageStereo
from simulation.modele_mujoco import GROUPE_ROVER, GROUPE_TERRAIN
from simulation.monde import Monde

journal = logging.getLogger("fieldmind.camera")

TAILLE_TEXTURE = 4096
"""Côté de la texture du sol (pixels) : ~3 cm par pixel, le grain dont la stéréo a besoin."""
TAILLE_COULEURS = 1024
"""Les couleurs (piste, couvert, clairière) varient lentement : calculées à cette taille, puis
agrandies ; seul le grain fin est tiré à pleine résolution."""
MARGE_SOL_M = 25.0


def emprise_sol(monde: Monde) -> tuple[float, float, float]:
    """(centre x, centre y, demi-côté) du plan texturé : zone, lisière et marge."""
    lim = monde.scenario.limites
    cx, cy = (lim.xmin + lim.xmax) / 2, (lim.ymin + lim.ymax) / 2
    demi = max(lim.xmax - lim.xmin, lim.ymax - lim.ymin) / 2
    return cx, cy, demi + monde.params.lisiere.largeur + MARGE_SOL_M


def _bruit(rng: np.random.Generator, n: int, echelles: tuple[int, ...]) -> np.ndarray:
    """Bruit de valeur multi-échelle, normalisé dans [-1, 1]."""
    total = np.zeros((n, n))
    for k, cellules in enumerate(echelles):
        grossier = rng.uniform(-1, 1, (cellules + 1, cellules + 1))
        total += ndimage.zoom(grossier, n / (cellules + 1), order=1)[:n, :n] / (k + 1)
    borne: float = float(np.max(np.abs(total))) or 1.0
    return total / borne


def texture_sol(monde: Monde) -> np.ndarray:
    """Image RVB (TAILLE_TEXTURE², uint8) du sol, vue de dessus. La ligne 0 correspond au nord
    (y max) du plan."""
    cx, cy, demi = emprise_sol(monde)
    n = TAILLE_COULEURS
    resolution = 2 * demi / n
    rng = np.random.default_rng(monde.graine + 1)

    def pixels(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        colonne = np.clip(((x - (cx - demi)) / resolution).astype(int), 0, n - 1)
        ligne = np.clip((((cy + demi) - y) / resolution).astype(int), 0, n - 1)
        return ligne, colonne

    # Distance à la ligne centrale de la piste (rastérisée tous les 5 cm).
    piste = monde.piste
    dense = np.vstack(
        [
            np.linspace(a, b, max(2, math.ceil(np.linalg.norm(b - a) / 0.05)))
            for a, b in itertools.pairwise(piste)
        ]
    )
    axe = np.ones((n, n), dtype=bool)
    axe[pixels(dense[:, 0], dense[:, 1])] = False
    d_piste = ndimage.distance_transform_edt(axe) * resolution

    # Densité du couvert : les arbres assombrissent le sol autour d'eux (aiguilles, ombre).
    couvert = np.zeros((n, n))
    arbres = monde.arbres + monde.lisiere
    if arbres:
        xs = np.array([a.x for a in arbres])
        ys = np.array([a.y for a in arbres])
        np.add.at(couvert, pixels(xs, ys), 1.0)
        couvert = ndimage.gaussian_filter(couvert, 2.2 / resolution)
        couvert = np.clip(couvert / (np.percentile(couvert[couvert > 0], 90) + 1e-9), 0, 1)

    grain = _bruit(rng, n, (6, 24, 96))
    fin = rng.uniform(-1, 1, (n, n))
    herbe = np.array([0.40, 0.50, 0.24])
    sous_bois = np.array([0.30, 0.30, 0.18])
    terre = np.array([0.55, 0.45, 0.32])
    clairiere = np.array([0.50, 0.58, 0.27])

    couleur = herbe[None, None, :] * (1 + 0.12 * grain[..., None])
    couleur = couleur * (1 - couvert[..., None]) + sous_bois * couvert[..., None]
    lignes, colonnes = np.mgrid[0:n, 0:n]
    x = cx - demi + (colonnes + 0.5) * resolution
    y = cy + demi - (lignes + 0.5) * resolution
    cl = monde.params.clairiere
    d_cl = np.hypot(x - cl.x, y - cl.y)
    part_cl = np.clip((cl.rayon + 1.5 - d_cl) / 3.0, 0, 1)[..., None]
    couleur = couleur * (1 - part_cl) + clairiere * (1 + 0.1 * grain[..., None]) * part_cl
    demi_piste = monde.params.piste.demi_largeur
    part_piste = np.clip((demi_piste + 0.35 - d_piste) / 0.7 + 0.25 * grain, 0, 1)[..., None]
    couleur = couleur * (1 - part_piste) + terre * (1 + 0.1 * grain[..., None]) * part_piste
    couleur = couleur * (1 + 0.06 * fin[..., None])
    grande = cv2.resize(
        couleur.astype(np.float32), (TAILLE_TEXTURE, TAILLE_TEXTURE), interpolation=cv2.INTER_LINEAR
    )
    # Grain fin (gravillons, aiguilles, brindilles) : deux échelles, à pleine résolution.
    grain_fin = rng.uniform(-1.0, 1.0, (TAILLE_TEXTURE, TAILLE_TEXTURE)).astype(np.float32)
    moyen = rng.uniform(-1.0, 1.0, (TAILLE_TEXTURE // 4, TAILLE_TEXTURE // 4)).astype(np.float32)
    grain_moyen = cv2.resize(
        moyen, (TAILLE_TEXTURE, TAILLE_TEXTURE), interpolation=cv2.INTER_LINEAR
    )
    grande *= (1 + 0.10 * grain_fin + 0.08 * grain_moyen)[..., None]
    octets: np.ndarray = np.clip(grande * 255, 0, 255).astype(np.uint8)
    return octets


QUALITE_STEREO = 90
"""Qualité JPEG de la paire stéréo : les artefacts de compression gênent la mise en
correspondance, on garde donc une qualité élevée."""


def _jpeg(pixels: np.ndarray, qualite: int) -> str:
    tampon = io.BytesIO()
    mode = "L" if pixels.ndim == 2 else "RGB"
    ImagePil.fromarray(pixels, mode).save(tampon, format="JPEG", quality=qualite)
    return base64.b64encode(tampon.getvalue()).decode("ascii")


def _gris(pixels: np.ndarray) -> np.ndarray:
    gris: np.ndarray = np.round(pixels @ np.array([0.299, 0.587, 0.114])).astype(np.uint8)
    return gris


class CameraSimulee:
    """Rendu de la caméra avant et, si le rover en a une, de la paire stéréo dont elle est l'œil
    gauche. À appeler toujours depuis le même fil."""

    def __init__(
        self, modele: mujoco.MjModel, camera: Camera, stereo: Stereo | None = None
    ) -> None:
        self.camera = camera
        self.stereo = stereo
        self._modele = modele
        self._donnees = mujoco.MjData(modele)
        self._rendu: mujoco.Renderer | None = None
        self._option = mujoco.MjvOption()
        self._option.geomgroup[GROUPE_ROVER] = 0
        self._option.geomgroup[GROUPE_TERRAIN] = 1
        self.erreur: str | None = None
        self.duree_ms = 0.0

    def rendre_pixels(self, qpos: np.ndarray, nom: str = "camera") -> np.ndarray | None:
        """Image RVB (hauteur, largeur, 3) vue depuis l'état `qpos`, ou None si pas de rendu."""
        if self.erreur is not None:
            return None
        if self._rendu is None:
            try:
                self._rendu = mujoco.Renderer(
                    self._modele, self.camera.hauteur, self.camera.largeur, max_geom=4000
                )
            except Exception as erreur:  # bibliothèque OpenGL absente, pilote refusé, etc.
                self.erreur = f"rendu hors écran indisponible ({erreur})"
                journal.warning("caméra simulée désactivée : %s", self.erreur)
                return None
            self._rendu.scene.flags[mujoco.mjtRndFlag.mjRND_FOG] = True
        d = self._donnees
        d.qpos[:] = qpos
        mujoco.mj_kinematics(self._modele, d)
        mujoco.mj_camlight(self._modele, d)
        self._rendu.update_scene(d, camera=nom, scene_option=self._option)
        pixels: np.ndarray = self._rendu.render()
        return pixels.copy()

    def rendre(self, qpos: np.ndarray, t: float) -> list[Image | ImageStereo]:
        """Messages de la prise de vue faite à l'instant `t` : l'image couleur de la caméra avant
        et, s'il y a une paire stéréo, les deux images en niveaux de gris."""
        debut = time.perf_counter()
        pixels = self.rendre_pixels(qpos)
        if pixels is None:
            return []
        c = self.camera
        messages: list[Image | ImageStereo] = [
            Image(
                t=t,
                largeur=c.largeur,
                hauteur=c.hauteur,
                format="jpeg",
                donnees=_jpeg(pixels, c.qualite_jpeg),
            )
        ]
        if self.stereo is not None:
            droite = self.rendre_pixels(qpos, "camera_droite")
            if droite is not None:
                messages.append(
                    ImageStereo(
                        t=t,
                        largeur=c.largeur,
                        hauteur=c.hauteur,
                        gauche=_jpeg(_gris(pixels), QUALITE_STEREO),
                        droite=_jpeg(_gris(droite), QUALITE_STEREO),
                    )
                )
        self.duree_ms = (time.perf_counter() - debut) * 1000
        return messages

    def fermer(self) -> None:
        if self._rendu is not None:
            self._rendu.close()
            self._rendu = None
