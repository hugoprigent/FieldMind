"""Vecteurs du chemin BLE de HD dragon ; aucune connexion au matériel."""

import json
from pathlib import Path

import pytest

from fieldmind.drone.cooleerx import (
    DUREES_ACTION_S,
    PERIODE_RC_S,
    QUITTER_MANETTE,
    paquet_camera,
    paquet_rc,
)

REFERENCE_DEX = json.loads(
    (Path(__file__).parent / "fixtures/cooleerx_hd_dragon_1_1_0.json").read_text()
)


@pytest.mark.parametrize("vecteur", REFERENCE_DEX["vectors"], ids=lambda v: v["label"])
def test_conformite_aux_instructions_dex_originales(vecteur):
    # Résultats d'un interpréteur DEX indépendant de notre encodeur et de JADX.
    champs = vecteur["fields"]
    actions = {"u": "decoller", "v": "atterrir", "w": "stop", "A": "calibrer"}
    action = next((nom for champ, nom in actions.items() if champs.get(champ)), None)
    r, p, g, y = vecteur["axes_l_m_n_o"]
    assert paquet_rc(
        roulis=r, tangage=p, gaz=g, lacet=y, action=action, sans_tete=champs.get("z", False)
    ) == bytes.fromhex(vecteur["hex"])


@pytest.mark.parametrize(
    ("action", "hexadecimal"),
    [
        (None, "02 66 80 80 80 80 00 00 99"),
        ("decoller", "02 66 80 80 80 80 01 01 99"),
        ("atterrir", "02 66 80 80 80 80 02 02 99"),
        ("stop", "02 66 80 80 80 80 04 04 99"),
        ("calibrer", "02 66 80 80 80 80 80 80 99"),
    ],
)
def test_vecteurs_actions_du_profil_ble(action, hexadecimal):
    assert paquet_rc(action=action) == bytes.fromhex(hexadecimal)


def test_ordre_axes_et_checksum_sur_valeurs_distinctes():
    # Le checksum ne comprend ni 02, ni 66, ni 99.
    assert paquet_rc(roulis=17, tangage=34, gaz=51, lacet=68) == bytes.fromhex(
        "02 66 11 22 33 44 00 44 99"
    )
    assert paquet_rc(action="decoller", sans_tete=True) == bytes.fromhex(
        "02 66 80 80 80 80 11 11 99"
    )


def test_extremites_et_normalisation_gaz_de_application():
    assert paquet_rc(roulis=0, tangage=255, gaz=1, lacet=0) == bytes.fromhex(
        "02 66 01 ff 00 01 00 ff 99"
    )
    assert paquet_rc(gaz=0)[4] == 0
    assert paquet_rc(gaz=255)[4] == 255


@pytest.mark.parametrize("valeur", [-1, 256, 1.5, True, None, float("nan")])
@pytest.mark.parametrize("axe", ["roulis", "tangage", "gaz", "lacet"])
def test_axes_invalides_refuses(axe, valeur):
    with pytest.raises(ValueError):
        paquet_rc(**{axe: valeur})


def test_camera_hors_rc_et_sortie_mode_distincte_du_stop():
    assert paquet_camera(1) == bytes.fromhex("01 01")
    assert paquet_camera(2) == bytes.fromhex("01 02")
    assert bytes.fromhex("03 01") == QUITTER_MANETTE
    assert paquet_rc(action="stop") != QUITTER_MANETTE
    for index in (0, 3, True, 1.0):
        with pytest.raises(ValueError):
            paquet_camera(index)
    with pytest.raises(ValueError):
        paquet_rc(action="camera")  # 08 correspond au retournement, pas à la caméra
    with pytest.raises(ValueError):
        paquet_rc(sans_tete=1)


def test_cadence_et_durees_ne_sont_pas_celles_du_protocole_wifi():
    assert PERIODE_RC_S == 0.05
    assert DUREES_ACTION_S == {
        "decoller": 1.0, "atterrir": 1.0, "stop": 1.0, "calibrer": 2.0,
    }
