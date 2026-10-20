"""Reconnaissance au banc : captures et visée, sans aucune commande d'actionneur.

Le contrôleur de vol du mini-drone n'est pas identifié. Une rotation ou un posé ne
peuvent donc pas être déduits d'un chronomètre. Les six vues sont prises à la demande
de l'opérateur ; le centrage d'une croix reste une observation dans l'image.
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from collections.abc import Callable
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from fieldmind.bus import Bus, S
from fieldmind.types import ImageCamera

REFUS_VOL = (
    "pilotage des moteurs non validé pour ce drone : décollage et vol autonome bloqués ; "
    "utiliser Reconnaissance sans vol ou Viser la croix"
)
AGE_IMAGE_MAX_S = 1.5
PERIODE_S = 0.2  # 5 analyses/s au plus ; la réception BLE/série reste indépendante
ATTENTE_CROIX_S = 20.0
DUREE_MAX_S = 180.0
QUOTA_OCTETS = 100 * 1024 * 1024


class ReconnaissanceDrone:
    def __init__(
        self,
        bus: Bus,
        publier_etat: Callable[..., None],
        dossier: Path,
    ) -> None:
        self.bus = bus
        self._publier_etat = publier_etat
        self.dossier = dossier
        self._verrou = threading.RLock()
        self._arret = threading.Event()
        self._fil = threading.Thread(target=self._tourner, name="drone-vision", daemon=True)
        self._derniere: ImageCamera | None = None
        self._traitee: ImageCamera | None = None
        self.mode = "inactif"
        self._capture = False
        self._capture_apres = 0.0
        self._debut = 0.0
        self._etape = 0.0
        self._stable_depuis: float | None = None
        self._stable_n = 0
        self._session: Path | None = None
        self._vues: list[np.ndarray] = []
        self._dates: list[float] = []
        self._analyse_ms = 0.0
        self._montage: str | None = None

    def demarrer(self) -> None:
        self._fil.start()

    def arreter(self) -> None:
        self._arret.set()
        if self._fil.is_alive():
            self._fil.join(timeout=1.0)

    def deposer(self, image: ImageCamera) -> None:
        # Une seule image en attente, pas de file qui accumule de la latence.
        # Affectation atomique d'une référence : ne jamais attendre le verrou du
        # traitement (qui peut écrire une capture sur disque) depuis le fil BLE.
        self._derniere = image
        if self.mode in ("inactif", "erreur"):
            self.bus.publier(S.IMAGE_DRONE, image)

    def _etat(self, consigne: str, **champs: Any) -> None:
        self._publier_etat(
            mode_recon=self.mode,
            secteur_360=len(self._vues),
            consigne_vol=consigne,
            analyse_ms=round(self._analyse_ms, 2),
            montage_recon=self._montage,
            **champs,
        )

    def commander(self, commande: str, maintenant: float) -> None:
        """Lève ValueError si la demande est impossible, sans acquittement trompeur."""
        with self._verrou:
            if commande == "stop_recon":
                self.mode = "inactif"
                self._capture = False
                self._stable_depuis = None
                self._etat(
                    "Essai arrêté — aucune commande moteur",
                    croix_trouvee=False,
                    croix_verrouillee=False,
                )
                return
            if commande == "recon_capture":
                if self.mode != "rotation_360":
                    raise ValueError("démarrer la reconnaissance sans vol avant de capturer")
                if self._capture:
                    raise ValueError("capture déjà en attente d'une nouvelle image")
                self._capture = True
                self._capture_apres = maintenant
                return
            if commande not in ("recon_test", "cibler_croix"):
                raise ValueError("commande de reconnaissance inconnue")
            if self._derniere is None or maintenant - self._derniere.t > AGE_IMAGE_MAX_S:
                raise ValueError("aucune image réelle récente du drone (la mire ne suffit pas)")
            if commande == "cibler_croix" and self.mode == "ciblage_croix":
                self.commander("stop_recon", maintenant)
                return
            self._capture = False
            self._stable_depuis = None
            self._stable_n = 0
            if commande == "recon_test":
                self._vues.clear()
                self._dates.clear()
                self._montage = None
                self._session = None
            self._debut = self._etape = maintenant
            self.mode = "rotation_360" if commande == "recon_test" else "ciblage_croix"
            self._etat(
                "Tourner le drone à la main puis Capturer vue (1/6)"
                if commande == "recon_test"
                else "Visée uniquement — choisir la caméra du bas",
                croix_trouvee=False,
                croix_verrouillee=False,
            )
            self.bus.journaliser("drone", "essai visuel sans commande moteur démarré")

    def interrompre(self, raison: str) -> None:
        with self._verrou:
            if self.mode in ("inactif", "erreur"):
                return
            self.mode = "erreur"
            self._capture = False
            self._stable_depuis = None
            self._etat(raison, croix_trouvee=False, croix_verrouillee=False)
            self.bus.journaliser("drone", raison, "alerte")

    def _tourner(self) -> None:
        while not self._arret.wait(PERIODE_S):
            try:
                self.traiter(time.monotonic())
            except Exception as erreur:  # une panne de vision ne doit pas tuer la réception
                self.interrompre(f"Vision interrompue : {erreur}")

    def traiter(self, maintenant: float) -> None:
        with self._verrou:
            if self.mode in ("inactif", "erreur"):
                return
            image = self._derniere
            if image is None or maintenant - image.t > AGE_IMAGE_MAX_S:
                self.interrompre("Image drone perdue — essai interrompu, relancer explicitement")
                return
            if maintenant - self._debut > DUREE_MAX_S:
                self.interrompre("Durée maximale de l'essai atteinte")
                return
            if self._traitee is not None and image.t <= self._traitee.t:
                return  # une image figée ne permet jamais de confirmer le centrage
            if self._traitee is not None and image.t - self._traitee.t > 0.65:
                self._stable_depuis = None
                self._stable_n = 0
            self._traitee = image
            debut = time.perf_counter()
            toile = cv2.imdecode(np.frombuffer(image.jpeg, np.uint8), cv2.IMREAD_COLOR)
            if toile is None:
                self.interrompre("JPEG drone invalide — essai interrompu")
                return
            h, w = toile.shape[:2]
            if max(h, w) > 320:
                toile = cv2.resize(toile, (round(w * 320 / max(h, w)), round(h * 320 / max(h, w))))
            det = {"trouvee": False, "verrouillee": False, "x": 0.0, "y": 0.0, "taille_pct": 0.0}
            if self.mode == "rotation_360":
                if self._capture and image.t > self._capture_apres:
                    self._enregistrer_vue(toile, image.t)
                    self._capture = False
                consigne = f"Vue {len(self._vues)}/6 — tourner à la main, puis Capturer vue"
                if len(self._vues) == 6:
                    self._enregistrer_montage()
                    self.mode = "recherche_croix"
                    self._etape = maintenant
                    consigne = "6 vues enregistrées — choisir la caméra du bas et montrer la croix"
            else:
                det = detecter_croix(toile)
                if det["verrouillee"]:
                    if self._stable_depuis is None:
                        self._stable_depuis = image.t
                        self._stable_n = 0
                    self._stable_n += 1
                else:
                    self._stable_depuis = None
                    self._stable_n = 0
                stable = (
                    self._stable_depuis is not None
                    and image.t - self._stable_depuis >= 0.8
                    and self._stable_n >= 5
                )
                det["verrouillee"] = stable
                if stable:
                    consigne = "Croix centrée dans l'image — aucun atterrissage commandé"
                    if self.mode == "recherche_croix":
                        self.mode = "termine"
                elif det["trouvee"]:
                    consigne = f"Écart image : X {det['x']:+.2f} / Y {det['y']:+.2f}"
                else:
                    consigne = "Croix non visible — aucune correction moteur"
                if self.mode == "termine" and not stable:
                    self.mode = "recherche_croix"
                    self._etape = maintenant
                if self.mode == "recherche_croix" and maintenant - self._etape > ATTENTE_CROIX_S:
                    self.interrompre(
                        "Croix non centrée après 20 s — vues conservées, essai interrompu"
                    )
                    return
            annotee = annoter_image(toile, det, self.mode, len(self._vues), consigne)
            ok, jpeg = cv2.imencode(".jpg", annotee, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise ValueError("encodage JPEG impossible")
            self._analyse_ms = (time.perf_counter() - debut) * 1000
            self._etat(
                consigne,
                croix_trouvee=det["trouvee"],
                croix_verrouillee=det["verrouillee"],
                croix_x=det["x"],
                croix_y=det["y"],
                croix_taille_pct=det["taille_pct"],
            )
            self.bus.publier(
                S.IMAGE_DRONE,
                ImageCamera(
                    t=image.t,
                    camera="drone",
                    jpeg=bytes(jpeg),
                    numero=image.numero,
                    largeur=annotee.shape[1],
                    hauteur=annotee.shape[0],
                ),
            )

    def _enregistrer_vue(self, toile: np.ndarray, date: float) -> None:
        self.dossier.mkdir(parents=True, exist_ok=True)
        utilise = sum(p.stat().st_size for p in self.dossier.rglob("*") if p.is_file())
        if utilise > QUOTA_OCTETS - 2 * 1024 * 1024:
            raise ValueError(
                "quota reconnaissance de 100 Mio atteint ; exporter/retirer des essais"
            )
        if self._session is None:
            self._session = self.dossier / (time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:8])
            self._session.mkdir()
        chemin = self._session / f"vue-{len(self._vues) + 1}.jpg"
        if not cv2.imwrite(str(chemin), toile):
            raise OSError("impossible d'enregistrer la vue")
        self._vues.append(toile.copy())
        self._dates.append(date)
        self.bus.journaliser("drone", f"vue {len(self._vues)}/6 enregistrée (angle non mesuré)")

    def _enregistrer_montage(self) -> None:
        assert self._session is not None
        # Les vues peuvent provenir de deux capteurs de résolutions différentes.
        vues = [cv2.resize(v, (240, 320)) for v in self._vues]
        montage = cv2.vconcat([cv2.hconcat(vues[:3]), cv2.hconcat(vues[3:])])
        chemin = self._session / "montage.jpg"
        if not cv2.imwrite(str(chemin), montage):
            raise OSError("impossible d'enregistrer le montage")
        (self._session / "rapport.json").write_text(
            json.dumps(
                {
                    "mode": "sans_vol",
                    "couverture_360_verifiee": False,
                    "atterrissage_commande": False,
                    "vues": [
                        {"fichier": f"vue-{i + 1}.jpg", "t_monotone": t}
                        for i, t in enumerate(self._dates)
                    ],
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        self._montage = str(chemin)
        self.bus.journaliser(
            "drone", f"six vues et rapport enregistrés : {self._session}", "succes"
        )

    def fichier(self, nom: str) -> Path | None:
        autorises = {"montage.jpg", "rapport.json", *(f"vue-{i}.jpg" for i in range(1, 7))}
        if nom not in autorises or self._session is None:
            return None
        chemin = self._session / nom
        return chemin if chemin.is_file() else None


def detecter_croix(bgr: Any) -> dict[str, Any]:
    """Détecte un + ou X rouge/orange, ou sombre sur fond clair, par gabarit."""

    h, w = bgr.shape[:2]
    aire_img = float(h * w)

    # 1) Masque couleur (croix rouge / orange / magenta)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m_rouge1 = cv2.inRange(hsv, np.array([0, 75, 55]), np.array([18, 255, 255]))
    m_rouge2 = cv2.inRange(hsv, np.array([162, 75, 55]), np.array([180, 255, 255]))
    masque_couleur = cv2.bitwise_or(m_rouge1, m_rouge2)

    # 2) Masque contraste sombre sur fond clair (ex : croix noire tracée sur une feuille ou sur
    #    le toit du rover)
    gris = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    flou = cv2.GaussianBlur(gris, (5, 5), 0)
    masque_sombre = cv2.adaptiveThreshold(
        flou, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 31, 14
    )

    noyau = np.ones((3, 3), np.uint8)
    meilleur: dict[str, Any] = {
        "trouvee": False,
        "verrouillee": False,
        "x": 0.0,
        "y": 0.0,
        "taille_pct": 0.0,
        "contour": None,
        "centre": None,
        "score": 0.0,
    }

    for bonus, masque_brut in ((1.25, masque_couleur), (1.0, masque_sombre)):
        propre = cv2.morphologyEx(masque_brut, cv2.MORPH_OPEN, noyau)
        propre = cv2.morphologyEx(propre, cv2.MORPH_CLOSE, noyau)
        contours, _ = cv2.findContours(propre, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            aire = cv2.contourArea(cnt)
            if aire < 130 or aire > aire_img * 0.48:
                continue
            bx, by, bw, bh = cv2.boundingRect(cnt)
            if bx <= 2 or by <= 2 or (bx + bw) >= (w - 2) or (by + bh) >= (h - 2):
                continue
            ratio = min(bw, bh) / max(bw, bh, 1)
            if ratio < 0.45:
                continue
            env = cv2.convexHull(cnt)
            aire_env = cv2.contourArea(env)
            if aire_env <= 0:
                continue
            solidite = aire / aire_env
            # Une croix (+ ou X) a 4 encoches concaves : sa solidité est comprise entre 0.28 et 0.75
            if not (0.28 <= solidite <= 0.75):
                continue

            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.028 * peri, True)
            if len(approx) < 7 or len(approx) > 20:
                continue

            # Vérification par gabarit 32x32 (+ orthogonal ou X diagonal)
            vignette = np.zeros((bh, bw), dtype=np.uint8)
            cv2.drawContours(vignette, [cnt - np.array([[bx, by]])], -1, 255, -1)
            v32 = (cv2.resize(vignette, (32, 32), interpolation=cv2.INTER_NEAREST) > 127).astype(
                np.float32
            )

            centre_plein = float(np.mean(v32[11:21, 11:21]))
            if centre_plein < 0.50:
                continue

            # Gabarit '+' : bras horizontal + vertical pleins, 4 coins vides
            bras_plus = 0.5 * (float(np.mean(v32[12:20, :])) + float(np.mean(v32[:, 12:20])))
            coins_plus = 0.25 * (
                float(np.mean(v32[:9, :9]))
                + float(np.mean(v32[:9, 23:]))
                + float(np.mean(v32[23:, :9]))
                + float(np.mean(v32[23:, 23:]))
            )
            score_plus = bras_plus - 1.15 * coins_plus

            # Gabarit 'X' : diagonales pleines, milieux des 4 bords vides
            idx = np.arange(32)
            diag1 = float(np.mean([v32[i, max(0, i - 2) : min(32, i + 3)].mean() for i in idx]))
            diag2 = float(np.mean([v32[i, max(0, 29 - i) : min(32, 34 - i)].mean() for i in idx]))
            bords_x = 0.25 * (
                float(np.mean(v32[:7, 11:21]))
                + float(np.mean(v32[25:, 11:21]))
                + float(np.mean(v32[11:21, :7]))
                + float(np.mean(v32[11:21, 25:]))
            )
            score_x = 0.5 * (diag1 + diag2) - 1.15 * bords_x

            score_forme = max(score_plus, score_x)
            if score_forme < 0.24:
                continue

            M = cv2.moments(cnt)
            if M["m00"] <= 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            score_total = bonus * score_forme * ratio * (1.0 - abs(solidite - 0.50))
            if score_total > meilleur["score"]:
                dx = (cx - (w / 2.0)) / (w / 2.0)
                dy = (cy - (h / 2.0)) / (h / 2.0)
                verrou = abs(dx) <= 0.18 and abs(dy) <= 0.18
                meilleur = {
                    "trouvee": True,
                    "verrouillee": verrou,
                    "x": round(float(dx), 3),
                    "y": round(float(dy), 3),
                    "taille_pct": round(100.0 * aire / aire_img, 1),
                    "contour": cnt,
                    "centre": (cx, cy),
                    "boite": (bx, by, bw, bh),
                    "score": score_total,
                }

    return meilleur


def annoter_image(
    toile: Any, det: dict[str, Any], mode_recon: str, secteur_360: int, consigne_txt: str
) -> Any:
    """Dessine le HUD tactique de reconnaissance 360° et de visée sur croix d'atterrissage."""
    import cv2

    img = toile.copy()
    h, w = img.shape[:2]
    cx0, cy0 = w // 2, h // 2
    police = cv2.FONT_HERSHEY_SIMPLEX

    # Réticule central d'alignement (zone de verrouillage +-18%)
    r_verrou = int(min(w, h) * 0.18)
    coul_reticule = (90, 230, 120) if det.get("verrouillee") else (180, 195, 210)
    cv2.circle(img, (cx0, cy0), r_verrou, coul_reticule, 1, cv2.LINE_AA)
    cv2.line(img, (cx0 - 12, cy0), (cx0 + 12, cy0), coul_reticule, 1)
    cv2.line(img, (cx0, cy0 - 12), (cx0, cy0 + 12), coul_reticule, 1)

    if det.get("trouvee") and det.get("centre") is not None:
        tx, ty = det["centre"]
        coul = (60, 235, 100) if det.get("verrouillee") else (0, 195, 255)
        if det.get("contour") is not None:
            cv2.drawContours(img, [det["contour"]], -1, coul, 2)
        if det.get("boite") is not None:
            bx, by, bw, bh = det["boite"]
            cv2.rectangle(img, (bx, by), (bx + bw, by + bh), coul, 1)
        # Flèche d'asservissement visuel du centre caméra vers la croix
        cv2.arrowedLine(img, (cx0, cy0), (tx, ty), coul, 2, cv2.LINE_AA, tipLength=0.22)
        cv2.circle(img, (tx, ty), 4, coul, -1)

    # Bandeau supérieur : mode de mission / progression 360°
    cv2.rectangle(img, (0, 0), (w, 24), (16, 18, 22), -1)
    etiq_haut = f"SANS VOL: {mode_recon.upper()}"
    if mode_recon == "rotation_360":
        etiq_haut = f"ESSAI SANS VOL : VUE {secteur_360}/6"
    cv2.putText(img, etiq_haut[:32], (6, 16), police, 0.38, (110, 220, 255), 1, cv2.LINE_AA)

    # Bandeau inférieur : consigne d'atterrissage / erreur X/Y
    cv2.rectangle(img, (0, h - 24), (w, h), (16, 18, 22), -1)
    coul_bas = (80, 235, 120) if det.get("verrouillee") else (230, 235, 240)
    cv2.putText(img, consigne_txt[:34], (6, h - 8), police, 0.36, coul_bas, 1, cv2.LINE_AA)

    return img
