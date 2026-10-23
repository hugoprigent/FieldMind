"""Calibration de la paire stéréo avec une mire en damier.

Mode d'emploi (poste : Réglages > Caméras > Calibration stéréo) : tenir la mire (par défaut
9 × 6 coins intérieurs, cases de 25 mm, imprimée à plat et collée sur un support rigide) devant
les deux caméras, à 0,4–1,2 m, et capturer 10 à 20 paires en variant la position dans l'image
(centre, coins, bords), la distance et l'inclinaison (jusqu'à ±30°). Une paire n'est gardée que
si la mire est vue en entier dans les deux images.

Calcul : calibration de chaque caméra (focales, centre optique, distorsion radiale k1, k2 et
tangentielle), puis `cv2.stereoCalibrate` (rotation R et translation T de la caméra gauche vers
la droite, intrinsèques affinés), puis `cv2.stereoRectify`. Contrôles : erreur de reprojection
RMS (< 1 px attendu), erreur épipolaire après rectification (écart vertical des coins appariés,
< 0,5 px attendu), base mesurée comparée à `cameras.base_m`.

Fichier : npz au chemin `cameras.calibration` (`~/.config/fieldmind/stereo.npz`), lu par
`projection.lire_calibration`. Clés : k1, d1, k2, d2, r, t, e, f, r1, r2, p1, p2, q, taille
(largeur, hauteur des images de calibration), erreur_rms, erreur_epipolaire_px, base_m, mire
(colonnes, lignes, case_m), date.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from fieldmind.config import Profil
from fieldmind.perception.projection import lire_calibration

PAIRES_MIN = 5


class CalibrationStereo:
    def __init__(self, profil: Profil, colonnes: int | None = None, lignes: int | None = None,
                 case_m: float | None = None) -> None:
        v = profil.valeur
        self.colonnes = int(colonnes or v("perception.mire_colonnes", 9))
        self.lignes = int(lignes or v("perception.mire_lignes", 6))
        self.case_m = float(case_m or v("perception.mire_case_m", 0.025))
        self.chemin = Path(str(v("cameras.calibration", "~/.config/fieldmind/stereo.npz"))
                           ).expanduser()
        self.base_profil = float(v("cameras.base_m", 0.12))
        self.taille: tuple[int, int] | None = None
        self._coins_g: list[np.ndarray] = []
        self._coins_d: list[np.ndarray] = []
        self._donnees: dict[str, Any] | None = None
        self.resultat: dict[str, Any] | None = None
        objet = np.zeros((self.lignes * self.colonnes, 3), np.float32)
        grille = np.mgrid[0:self.colonnes, 0:self.lignes].T.reshape(-1, 2)
        objet[:, :2] = grille * self.case_m
        self._objet = objet

    @property
    def nombre(self) -> int:
        return len(self._coins_g)

    def effacer(self) -> None:
        self._coins_g.clear()
        self._coins_d.clear()
        self.taille = None
        self._donnees = None
        self.resultat = None

    # ------------------------------------------------------------------ capture
    def detecter(self, bgr: np.ndarray) -> np.ndarray | None:
        """Coins intérieurs (N, 1, 2) float32 à la précision sous-pixel, ou None."""
        gris = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        motif = (self.colonnes, self.lignes)
        try:
            trouve, coins = cv2.findChessboardCornersSB(
                gris, motif, flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)
        except cv2.error:
            trouve, coins = False, None
        if not trouve or coins is None:
            trouve, coins = cv2.findChessboardCorners(
                gris, motif, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not trouve or coins is None:
                return None
            critere = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3)
            coins = cv2.cornerSubPix(gris, coins, (7, 7), (-1, -1), critere)
        return np.asarray(coins, np.float32).reshape(-1, 1, 2)

    def apercu(self, bgr: np.ndarray, largeur_max: int = 640) -> list[list[float]] | None:
        """Coins de la mire en coordonnées normalisées (0–1), vite (image réduite, abandon
        rapide sans mire) : pour l'aperçu en direct de l'assistant, pas pour le calcul."""
        h, lg = bgr.shape[:2]
        echelle = min(1.0, largeur_max / lg)
        petite = cv2.resize(bgr, (int(lg * echelle), int(h * echelle))) if echelle < 1 else bgr
        gris = cv2.cvtColor(petite, cv2.COLOR_BGR2GRAY) if petite.ndim == 3 else petite
        trouve, coins = cv2.findChessboardCorners(
            gris, (self.colonnes, self.lignes),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK)
        if not trouve or coins is None:
            return None
        hp, lp = gris.shape[:2]
        return [[round(float(x) / lp, 4), round(float(y) / hp, 4)] for x, y in coins.reshape(-1, 2)]

    def couverture(self) -> list[list[float]]:
        """Centre (u, v) et taille relative de la mire pour chaque paire gardée (caméra gauche),
        pour montrer les zones de l'image déjà couvertes."""
        if self.taille is None:
            return []
        lg, h = self.taille
        sortie = []
        for coins in self._coins_g:
            pts = coins.reshape(-1, 2)
            centre = pts.mean(axis=0)
            etendue = float(np.ptp(pts[:, 0]) / lg)
            sortie.append([round(float(centre[0]) / lg, 3), round(float(centre[1]) / h, 3),
                           round(etendue, 3)])
        return sortie

    def ajouter_paire(self, bgr_g: np.ndarray, bgr_d: np.ndarray) -> bool:
        """Cherche la mire dans les deux images ; garde la paire si elle est vue dans les deux
        (et n'est pas presque identique à une paire déjà gardée). Renvoie True si gardée."""
        taille = (int(bgr_g.shape[1]), int(bgr_g.shape[0]))
        if (bgr_d.shape[1], bgr_d.shape[0]) != taille:
            return False
        if self.taille is not None and taille != self.taille:
            return False
        coins_g = self.detecter(bgr_g)
        if coins_g is None:
            return False
        coins_d = self.detecter(bgr_d)
        if coins_d is None:
            return False
        # Même ordre des coins dans les deux images (le premier coin du même côté).
        sens_g = coins_g[-1, 0] - coins_g[0, 0]
        sens_d = coins_d[-1, 0] - coins_d[0, 0]
        if float(sens_g @ sens_d) < 0:
            coins_d = coins_d[::-1].copy()
        for precedent in self._coins_g:
            if float(np.mean(np.linalg.norm(precedent - coins_g, axis=2))) < 0.01 * taille[0]:
                return False  # la mire n'a pas bougé : paire inutile
        self.taille = taille
        self._coins_g.append(coins_g)
        self._coins_d.append(coins_d)
        return True

    # ------------------------------------------------------------------ calcul
    def calculer(self) -> dict[str, Any]:
        """Calibre la paire. Lève ValueError s'il manque des paires. Renvoie le bilan (JSON)."""
        if self.nombre < PAIRES_MIN or self.taille is None:
            raise ValueError(f"au moins {PAIRES_MIN} paires où la mire est vue des deux caméras "
                             f"({self.nombre} pour l'instant)")
        objets = [self._objet] * self.nombre
        taille = self.taille
        drapeaux = cv2.CALIB_FIX_K3
        critere = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
        rms_g, k1, d1, _, _ = cv2.calibrateCamera(objets, self._coins_g, taille, None, None,
                                                  flags=drapeaux, criteria=critere)
        rms_d, k2, d2, _, _ = cv2.calibrateCamera(objets, self._coins_d, taille, None, None,
                                                  flags=drapeaux, criteria=critere)
        rms, k1, d1, k2, d2, r, t, e, f = cv2.stereoCalibrate(
            objets, self._coins_g, self._coins_d, k1, d1, k2, d2, taille,
            criteria=critere, flags=cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3)
        r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(k1, d1, k2, d2, taille, r, t,
                                                    flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        # Erreur épipolaire : après rectification, les coins appariés sont sur la même ligne.
        ecarts = []
        for cg, cd in zip(self._coins_g, self._coins_d, strict=True):
            rg = cv2.undistortPoints(cg, k1, d1, R=r1, P=p1).reshape(-1, 2)
            rd = cv2.undistortPoints(cd, k2, d2, R=r2, P=p2).reshape(-1, 2)
            ecarts.append(np.abs(rg[:, 1] - rd[:, 1]))
        epipolaire = float(np.sqrt(np.mean(np.concatenate(ecarts) ** 2)))
        base = float(np.linalg.norm(t))
        angle = float(np.degrees(np.linalg.norm(cv2.Rodrigues(r)[0])))
        hfov = math.degrees(2 * math.atan(taille[0] / (2 * float(k1[0, 0]))))
        avertissements = []
        if rms > 1.0:
            avertissements.append("erreur de reprojection élevée : mire floue, pliée ou mal vue")
        if epipolaire > 0.5:
            avertissements.append("erreur épipolaire élevée : caméras désynchronisées ou "
                                  "support qui bouge")
        if abs(base - self.base_profil) > 0.25 * self.base_profil:
            avertissements.append(f"base mesurée {base * 1000:.0f} mm loin des "
                                  f"{self.base_profil * 1000:.0f} mm du profil : taille de case ?")
        if float(t.reshape(-1)[0]) > 0:
            avertissements.append("caméras gauche et droite inversées ?")
        self._donnees = {
            "k1": k1, "d1": d1, "k2": k2, "d2": d2, "r": r, "t": t, "e": e, "f": f,
            "r1": r1, "r2": r2, "p1": p1, "p2": p2, "q": q,
            "taille": np.array(taille), "erreur_rms": np.array(rms),
            "erreur_epipolaire_px": np.array(epipolaire), "base_m": np.array(base),
            "mire": np.array([self.colonnes, self.lignes, self.case_m]),
            "date": np.array(time.time()),
        }
        self.resultat = {
            "paires": self.nombre, "taille": list(taille),
            "erreur_rms_px": round(float(rms), 4),
            "erreur_rms_gauche_px": round(float(rms_g), 4),
            "erreur_rms_droite_px": round(float(rms_d), 4),
            "erreur_epipolaire_px": round(epipolaire, 4),
            "base_m": round(base, 5), "base_profil_m": self.base_profil,
            "angle_relatif_deg": round(angle, 3),
            "focale_px": round(float(k1[0, 0]), 2), "hfov_deg": round(hfov, 2),
            "ok": not avertissements, "avertissements": avertissements,
        }
        return self.resultat

    def enregistrer(self, chemin: str | Path | None = None) -> Path:
        """Écrit le fichier npz (calculer() d'abord). Renvoie son chemin."""
        if self._donnees is None:
            raise ValueError("rien à enregistrer : calculer() d'abord")
        fichier = Path(chemin).expanduser() if chemin is not None else self.chemin
        fichier.parent.mkdir(parents=True, exist_ok=True)
        temporaire = fichier.with_name(fichier.stem + ".tmp.npz")
        np.savez(temporaire, **self._donnees)
        temporaire.replace(fichier)
        return fichier

    @staticmethod
    def charger(chemin: str | Path) -> dict[str, Any] | None:
        return lire_calibration(chemin)
