"""Serveur des outils de perception, de voix et de mémoire du rover, sur le GX10.

Une image envoyée une fois sert à plusieurs modèles : le rover l'envoie (JPEG compressé) avec la
liste des tâches, le serveur la garde en mémoire quelques minutes sous un identifiant, et les
requêtes suivantes sur la même image ne renvoient que cet identifiant. La liaison avec le rover
est lente : ce qui revient est un petit JSON, jamais une image.

Modèles (chacun facultatif : s'il ne se charge pas, les autres restent servis) :
- détection : D-FINE (ustc-community/dfine-xlarge-obj2coco, 80 classes COCO) ;
- profondeur : Depth Anything V2 métrique, extérieur (mètres) ;
- segmentation : SAM 2.1 (contour d'un objet désigné par un point ou une boîte) ;
- empreintes : Qwen3-VL-Embedding-2B (image ou texte → vecteur, pour reconnaître un lieu) ;
- synthèse vocale : Qwen3-TTS-0.6B (texte → WAV).

Points d'accès (clé : en-tête `Authorization: Bearer <VLLM_API_KEY>`) :
  GET  /v1/sante              modèles chargés, mémoire, erreurs
  POST /v1/analyse            {"image": base64 | "image_id", "taches": [...], "points": [[u, v]],
                               "boites": [[u0, v0, u1, v1]], "seuil": 0.4}
  POST /v1/empreintes         {"image" | "image_id" | "texte"}
  POST /v1/audio/speech       {"input": "...", "voice": "...", "language": "French"} → audio/wav
Coordonnées d'image normalisées 0..1 (u vers la droite, v vers le bas).
"""

from __future__ import annotations

import base64
import io
import logging
import os
import threading
import time
import uuid
from collections import OrderedDict
from typing import Any

import cv2
import numpy as np
import torch
from fastapi import Depends, FastAPI, Header, HTTPException
from fastapi.responses import Response
from PIL import Image

journal = logging.getLogger("outils")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s : %(message)s")

CLE = os.environ.get("VLLM_API_KEY", "")
APPAREIL = "cuda" if torch.cuda.is_available() else "cpu"
TYPE = torch.bfloat16 if APPAREIL == "cuda" else torch.float32
MODELES = {
    "detection": os.environ.get("MODELE_DETECTION", "ustc-community/dfine-xlarge-obj2coco"),
    "profondeur": os.environ.get("MODELE_PROFONDEUR",
                                 "depth-anything/Depth-Anything-V2-Metric-Outdoor-Large-hf"),
    "segmentation": os.environ.get("MODELE_SEGMENTATION", "facebook/sam2.1-hiera-large"),
    "empreintes": os.environ.get("MODELE_EMPREINTES", "Qwen/Qwen3-VL-Embedding-2B"),
    "synthese": os.environ.get("MODELE_SYNTHESE", "Qwen/Qwen3-TTS-12Hz-0.6B-CustomVoice"),
}
ACTIFS = {t for t in os.environ.get("OUTILS_ACTIFS", ",".join(MODELES)).split(",") if t}
IMAGES_MAX, IMAGE_DUREE_S = 32, 600.0
GRILLE_PROFONDEUR = (16, 9)  # colonnes, lignes : carte de profondeur résumée renvoyée au rover


class Etat:
    def __init__(self) -> None:
        self.modeles: dict[str, Any] = {}
        self.erreurs: dict[str, str] = {}
        self.durees_chargement: dict[str, float] = {}
        self.images: OrderedDict[str, tuple[float, Image.Image]] = OrderedDict()
        self.verrou_images = threading.Lock()
        self.verrous = {tache: threading.Lock() for tache in MODELES}  # un calcul à la fois


etat = Etat()
app = FastAPI(title="FieldMind — outils du GX10")


def verifier_cle(authorization: str = Header(default="")) -> None:
    if CLE and authorization != f"Bearer {CLE}":
        raise HTTPException(401, "clé absente ou fausse")


# ------------------------------------------------------------------ chargement des modèles

def _charger_detection() -> Any:
    from transformers import AutoImageProcessor, DFineForObjectDetection

    nom = MODELES["detection"]
    return (AutoImageProcessor.from_pretrained(nom),
            DFineForObjectDetection.from_pretrained(nom).to(APPAREIL).eval())


def _charger_profondeur() -> Any:
    from transformers import AutoImageProcessor, AutoModelForDepthEstimation

    nom = MODELES["profondeur"]
    return (AutoImageProcessor.from_pretrained(nom),
            AutoModelForDepthEstimation.from_pretrained(nom).to(APPAREIL).eval())


def _charger_segmentation() -> Any:
    from transformers import Sam2Model, Sam2Processor

    nom = MODELES["segmentation"]
    return Sam2Processor.from_pretrained(nom), Sam2Model.from_pretrained(nom).to(APPAREIL).eval()


def _charger_empreintes() -> Any:
    from transformers import AutoProcessor, Qwen3VLForConditionalGeneration

    nom = MODELES["empreintes"]
    modele = Qwen3VLForConditionalGeneration.from_pretrained(nom, dtype=TYPE).to(APPAREIL).eval()
    return AutoProcessor.from_pretrained(nom), modele


def _charger_synthese() -> Any:
    from qwen_tts import Qwen3TTSModel

    return Qwen3TTSModel.from_pretrained(MODELES["synthese"], device_map=APPAREIL, dtype=TYPE)


CHARGEURS = {"detection": _charger_detection, "profondeur": _charger_profondeur,
             "segmentation": _charger_segmentation, "empreintes": _charger_empreintes,
             "synthese": _charger_synthese}


def charger_tout() -> None:
    for tache in MODELES:
        if tache not in ACTIFS:
            continue
        debut = time.monotonic()
        try:
            etat.modeles[tache] = CHARGEURS[tache]()
            etat.durees_chargement[tache] = round(time.monotonic() - debut, 1)
            journal.info("%s prêt (%s) en %.1f s", tache, MODELES[tache], time.monotonic() - debut)
        except Exception as erreur:  # un modèle en panne ne bloque pas les autres
            etat.erreurs[tache] = f"{type(erreur).__name__}: {erreur}"[:300]
            journal.exception("%s indisponible", tache)


@app.on_event("startup")
def demarrage() -> None:
    threading.Thread(target=charger_tout, name="chargement", daemon=True).start()


def _modele(tache: str) -> Any:
    if tache not in etat.modeles:
        raise HTTPException(503, f"{tache} indisponible : "
                                 f"{etat.erreurs.get(tache, 'en cours de chargement')}")
    return etat.modeles[tache]


# ------------------------------------------------------------------ images

def _image(corps: dict[str, Any]) -> tuple[str, Image.Image]:
    maintenant = time.monotonic()
    with etat.verrou_images:
        for cle in [c for c, (t, _) in etat.images.items() if maintenant - t > IMAGE_DUREE_S]:
            del etat.images[cle]
        if corps.get("image_id"):
            trouvee = etat.images.get(str(corps["image_id"]))
            if trouvee is None:
                raise HTTPException(404, "image inconnue ou expirée : la renvoyer")
            etat.images.move_to_end(str(corps["image_id"]))
            return str(corps["image_id"]), trouvee[1]
    if not corps.get("image"):
        raise HTTPException(400, "`image` (JPEG en base64) ou `image_id` attendu")
    try:
        image = Image.open(io.BytesIO(base64.b64decode(corps["image"]))).convert("RGB")
    except Exception as erreur:
        raise HTTPException(400, f"image illisible : {erreur}") from erreur
    ident = uuid.uuid4().hex[:12]
    with etat.verrou_images:
        etat.images[ident] = (maintenant, image)
        while len(etat.images) > IMAGES_MAX:
            etat.images.popitem(last=False)
    return ident, image


# ------------------------------------------------------------------ tâches

@torch.inference_mode()
def detecter(image: Image.Image, seuil: float) -> list[dict[str, Any]]:
    processeur, modele = _modele("detection")
    with etat.verrous["detection"]:
        entrees = processeur(images=image, return_tensors="pt").to(APPAREIL)
        sorties = modele(**entrees)
        resultats = processeur.post_process_object_detection(
            sorties, target_sizes=[(image.height, image.width)], threshold=seuil)[0]
    lg, h = image.width, image.height
    objets = []
    for score, label, boite in zip(resultats["scores"].tolist(), resultats["labels"].tolist(),
                                   resultats["boxes"].tolist(), strict=True):
        x0, y0, x1, y1 = boite
        objets.append({"label": modele.config.id2label[int(label)], "score": round(score, 3),
                       "boite": [round(max(0.0, x0 / lg), 4), round(max(0.0, y0 / h), 4),
                                 round(min(1.0, x1 / lg), 4), round(min(1.0, y1 / h), 4)]})
    return sorted(objets, key=lambda o: -o["score"])[:40]


@torch.inference_mode()
def carte_profondeur(image: Image.Image) -> np.ndarray:
    processeur, modele = _modele("profondeur")
    with etat.verrous["profondeur"]:
        entrees = processeur(images=image, return_tensors="pt").to(APPAREIL)
        sorties = modele(**entrees)
        carte = processeur.post_process_depth_estimation(
            sorties, target_sizes=[(image.height, image.width)])[0]["predicted_depth"]
    return carte.float().cpu().numpy()


def _profondeur_en(carte: np.ndarray, u: float, v: float, rayon: int = 4) -> float:
    h, lg = carte.shape
    x, y = int(np.clip(u * lg, 0, lg - 1)), int(np.clip(v * h, 0, h - 1))
    zone = carte[max(0, y - rayon): y + rayon + 1, max(0, x - rayon): x + rayon + 1]
    return round(float(np.median(zone)), 2)


def _profondeur_boite(carte: np.ndarray, boite: list[float]) -> float:
    """Médiane du tiers central de la boîte : le fond autour de l'objet compte peu."""
    h, lg = carte.shape
    u0, v0, u1, v1 = boite
    cx0, cx1 = u0 + (u1 - u0) / 3, u1 - (u1 - u0) / 3
    cy0, cy1 = v0 + (v1 - v0) / 3, v1 - (v1 - v0) / 3
    zone = carte[int(cy0 * h): max(int(cy1 * h), int(cy0 * h) + 1),
                 int(cx0 * lg): max(int(cx1 * lg), int(cx0 * lg) + 1)]
    return round(float(np.median(zone)), 2) if zone.size else _profondeur_en(
        carte, (u0 + u1) / 2, (v0 + v1) / 2)


@torch.inference_mode()
def segmenter(image: Image.Image, points: list[list[float]], boites: list[list[float]]
              ) -> list[dict[str, Any]]:
    processeur, modele = _modele("segmentation")
    lg, h = image.width, image.height
    resultats = []
    demandes = [("point", p) for p in points] + [("boite", b) for b in boites]
    for genre, valeur in demandes[:8]:
        with etat.verrous["segmentation"]:
            if genre == "point":
                entrees = processeur(images=image,
                                     input_points=[[[[valeur[0] * lg, valeur[1] * h]]]],
                                     input_labels=[[[1]]], return_tensors="pt").to(APPAREIL)
            else:
                entrees = processeur(images=image, input_boxes=[[[valeur[0] * lg, valeur[1] * h,
                                                                  valeur[2] * lg, valeur[3] * h]]],
                                     return_tensors="pt").to(APPAREIL)
            sorties = modele(**entrees, multimask_output=False)
            masque = processeur.post_process_masks(sorties.pred_masks.cpu(),
                                                   entrees["original_sizes"])[0]
        masque = masque.squeeze().numpy().astype(np.uint8)
        contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            resultats.append({genre: valeur, "contour": [], "aire": 0.0})
            continue
        grand = max(contours, key=cv2.contourArea)
        simple = cv2.approxPolyDP(grand, 0.004 * cv2.arcLength(grand, True), True)[:, 0, :]
        x, y, lb, hb = cv2.boundingRect(grand)
        resultats.append({
            genre: valeur,
            "contour": [[round(px / lg, 4), round(py / h, 4)] for px, py in simple[:64]],
            "boite": [round(x / lg, 4), round(y / h, 4), round((x + lb) / lg, 4),
                      round((y + hb) / h, 4)],
            "aire": round(float(masque.sum()) / (lg * h), 4),
        })
    return resultats


@torch.inference_mode()
def empreinte(image: Image.Image | None, texte: str | None) -> list[float]:
    processeur, modele = _modele("empreintes")
    contenu: list[dict[str, Any]] = [{"type": "image", "image": image}] if image is not None else []
    if texte:
        contenu.append({"type": "text", "text": texte})
    messages = [{"role": "system", "content": [{"type": "text", "text":
                                                "Represent the user's input."}]},
                {"role": "user", "content": contenu}]
    with etat.verrous["empreintes"]:
        entrees = processeur.apply_chat_template(
            messages, tokenize=True, add_generation_prompt=True, return_dict=True,
            return_tensors="pt").to(APPAREIL)
        sorties = modele(**entrees, output_hidden_states=True)
        vecteur = sorties.hidden_states[-1][0, -1].float()
    vecteur = torch.nn.functional.normalize(vecteur, dim=0)
    return [round(float(x), 5) for x in vecteur.cpu()]


def synthese(texte: str, voix: str | None, langue: str) -> bytes:
    import soundfile

    modele = _modele("synthese")
    voix_dispo = list(getattr(modele, "get_supported_speakers", lambda: [])() or [])
    langues = list(getattr(modele, "get_supported_languages", lambda: [])() or [])
    langue_ok = next((x for x in langues if x.lower() == langue.lower()), langues[0] if langues
                     else langue)
    voix_ok = voix if voix in voix_dispo else (voix_dispo[0] if voix_dispo else voix)
    with etat.verrous["synthese"]:
        ondes, frequence = modele.generate_custom_voice(text=texte[:400], language=langue_ok,
                                                        speaker=voix_ok)
    tampon = io.BytesIO()
    soundfile.write(tampon, np.asarray(ondes[0], dtype=np.float32), frequence, format="WAV",
                    subtype="PCM_16")
    return tampon.getvalue()


# ------------------------------------------------------------------ points d'accès

@app.get("/v1/sante")
def sante() -> dict[str, Any]:
    libre, total = (torch.cuda.mem_get_info() if APPAREIL == "cuda" else (0, 0))
    return {"modeles": {t: MODELES[t] for t in etat.modeles},
            "indisponibles": {t: etat.erreurs.get(t, "en cours de chargement")
                              for t in MODELES if t in ACTIFS and t not in etat.modeles},
            "chargement_s": etat.durees_chargement, "images_en_memoire": len(etat.images),
            "memoire_libre_go": round(libre / 1e9, 1), "memoire_totale_go": round(total / 1e9, 1)}


@app.post("/v1/analyse", dependencies=[Depends(verifier_cle)])
def analyse(corps: dict[str, Any]) -> dict[str, Any]:
    debut = time.perf_counter()
    ident, image = _image(corps)
    taches = list(corps.get("taches") or ["detection"])
    points = [list(map(float, p))[:2] for p in corps.get("points") or []]
    boites = [list(map(float, b))[:4] for b in corps.get("boites") or []]
    reponse: dict[str, Any] = {"image_id": ident, "largeur": image.width, "hauteur": image.height,
                               "durees_ms": {}, "erreurs": {}}

    def chrono(nom: str, fonction: Any) -> Any:
        t0 = time.perf_counter()
        try:
            return fonction()
        except HTTPException as erreur:
            reponse["erreurs"][nom] = erreur.detail
        except Exception as erreur:
            journal.exception("tâche %s", nom)
            reponse["erreurs"][nom] = f"{type(erreur).__name__}: {erreur}"[:200]
        finally:
            reponse["durees_ms"][nom] = round(1000 * (time.perf_counter() - t0), 1)
        return None

    if "detection" in taches:
        objets = chrono("detection", lambda: detecter(image, float(corps.get("seuil", 0.4))))
        if objets is not None:
            reponse["detections"] = objets
            boites_profondeur = [o["boite"] for o in objets]
        else:
            boites_profondeur = []
    else:
        boites_profondeur = []
    if "profondeur" in taches:
        carte = chrono("profondeur", lambda: carte_profondeur(image))
        if carte is not None:
            c, r = GRILLE_PROFONDEUR
            petite = cv2.resize(carte, (c, r), interpolation=cv2.INTER_AREA)
            reponse["profondeur"] = {
                "unite": "m", "grille": [[round(float(x), 2) for x in ligne] for ligne in petite],
                "points": [_profondeur_en(carte, u, v) for u, v in points],
                "boites": [_profondeur_boite(carte, b) for b in boites],
                "min_m": round(float(carte.min()), 2), "max_m": round(float(carte.max()), 2)}
            for objet, distance in zip(reponse.get("detections", []),
                                       [_profondeur_boite(carte, b) for b in boites_profondeur],
                                       strict=False):
                objet["distance_m"] = distance
    if "segmentation" in taches and (points or boites):
        masques = chrono("segmentation", lambda: segmenter(image, points, boites))
        if masques is not None:
            reponse["segmentation"] = masques
    if "empreinte" in taches:
        vecteur = chrono("empreinte", lambda: empreinte(image, None))
        if vecteur is not None:
            reponse["empreinte"] = vecteur
    reponse["durees_ms"]["total"] = round(1000 * (time.perf_counter() - debut), 1)
    return reponse


@app.post("/v1/empreintes", dependencies=[Depends(verifier_cle)])
def empreintes(corps: dict[str, Any]) -> dict[str, Any]:
    image = _image(corps)[1] if (corps.get("image") or corps.get("image_id")) else None
    texte = corps.get("texte")
    if image is None and not texte:
        raise HTTPException(400, "`image`, `image_id` ou `texte` attendu")
    vecteur = empreinte(image, texte)
    return {"modele": MODELES["empreintes"], "dimension": len(vecteur), "vecteur": vecteur}


@app.post("/v1/audio/speech", dependencies=[Depends(verifier_cle)])
def parole(corps: dict[str, Any]) -> Response:
    texte = str(corps.get("input") or "").strip()
    if not texte:
        raise HTTPException(400, "`input` vide")
    wav = synthese(texte, corps.get("voice"), str(corps.get("language") or "French"))
    return Response(content=wav, media_type="audio/wav")


@app.get("/v1/audio/voix")
def voix() -> dict[str, Any]:
    modele = _modele("synthese")
    return {"voix": list(getattr(modele, "get_supported_speakers", lambda: [])() or []),
            "langues": list(getattr(modele, "get_supported_languages", lambda: [])() or [])}
