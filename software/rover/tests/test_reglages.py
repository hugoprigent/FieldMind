"""Réglages enregistrés et « tout réinitialiser »."""

from __future__ import annotations

import json
from pathlib import Path

from fieldmind.config import FICHIER_PROFIL, Profil


def test_reinitialiser_garde_branchement_et_calibrations(tmp_path: Path) -> None:
    fichier = tmp_path / "reglages.json"
    profil = Profil.charger(FICHIER_PROFIL, reglages=fichier)
    origine = profil.valeur("vitesses.max_manuel_m_s")
    profil.regler("vitesses.max_manuel_m_s", 1.2)
    profil.regler("esp.sens_inverse", True)
    profil.regler("traction.pwm_par_m_s", 65.0)
    # Enregistré : retrouvé à la session suivante
    assert Profil.charger(FICHIER_PROFIL, reglages=fichier).valeur("vitesses.max_manuel_m_s") == 1.2
    vus: list[str] = []
    profil.sur_changement(lambda cle, _v: vus.append(cle))
    assert profil.reinitialiser() == ["vitesses.max_manuel_m_s"]
    assert profil.valeur("vitesses.max_manuel_m_s") == origine
    assert vus == ["vitesses.max_manuel_m_s"]
    gardes = json.loads(fichier.read_text())
    assert gardes == {"esp.sens_inverse": True, "traction.pwm_par_m_s": 65.0}
