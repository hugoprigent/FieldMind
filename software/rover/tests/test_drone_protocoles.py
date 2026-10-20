"""Trames RCFPVPro et cycle RC : sockets simulés, aucun envoi physique."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import replace

import pytest

from fieldmind.drone.protocoles import (
    HANDSHAKE,
    AxesRC,
    BattementRC,
    TransportUDP,
    ValidationMateriel,
    paquet_rc,
)


class Collecteur:
    def __init__(self):
        self.paquets = []
        self.ferme = False

    def envoyer(self, p):
        self.paquets.append(p)

    def fermer(self):
        self.ferme = True


def test_trame_neutre_et_checksums_de_la_specification():
    paquet = paquet_rc(AxesRC())
    assert paquet == bytes.fromhex("cc5a01831566148080808000000000000000000000000000997c")
    assert len(paquet) == 26
    for action, flag in [("decoller", 1), ("atterrir", 1), ("stop", 2), ("calibrer", 4)]:
        p = paquet_rc(AxesRC(), action)
        assert p[11] == flag
        assert p[23] == flag  # checksum interne
        xor = 0
        for b in p[2:-1]:
            xor ^= b
        assert xor == p[-1]
    assert HANDSHAKE.hex() == "cc5a01820236b7"
    with pytest.raises(ValueError):
        paquet_rc(AxesRC(), "camera_bas")  # pas de commande caméra inventée


@pytest.mark.parametrize("v", [float("nan"), float("inf"), -2, 3])
def test_axes_invalides_refuses(v):
    with pytest.raises(ValueError):
        AxesRC(montee=v)


def test_axes_centres_et_amplitude_limitee():
    p = paquet_rc(AxesRC(montee=1, lacet=-1, avant=1, droite=-1))
    assert list(p[7:11]) == [109, 147, 147, 109]
    with pytest.raises(ValueError):
        paquet_rc(AxesRC(), amplitude=1)


def test_actions_impulsionnelles_et_homme_mort():
    b = BattementRC(Collecteur())
    b.proposer(AxesRC(montee=0.5), 1, "decoller")
    assert b.pas(1)[11] == 1
    b.confirmer_en_vol(True)
    assert b.pas(1.04)[11] == 0
    assert b.pas(1.31)[11] == 1  # land une seule fois si le poste se tait
    assert b.verrouille
    assert b.pas(1.35)[11] == 0
    assert list(b.pas(1.39)[7:11]) == [128] * 4
    with pytest.raises(ValueError):
        b.proposer(AxesRC(), 2, "decoller")


def test_demande_perimee_au_sol_ne_declenche_jamais_takeoff_ou_land_toggle():
    b = BattementRC(Collecteur())
    b.proposer(AxesRC(), 1, "decoller")
    assert b.pas(2)[11] == 0
    assert b.verrouille
    b.fermer()
    assert b.emetteur.paquets == []


def test_land_sur_sol_nest_pas_un_decollage_et_fermeture_ne_rejoue_pas_land():
    b = BattementRC(Collecteur())
    b.proposer(AxesRC(), 1, "atterrir")
    assert b.pas(1)[11] == 0
    b.proposer(AxesRC(), 1.1, "decoller")
    assert b.pas(1.1)[11] == 1
    b.confirmer_en_vol(True)
    b.fermer()
    assert len(b.emetteur.paquets) == 1 and b.emetteur.paquets[0][11] == 1
    b.fermer()
    assert len(b.emetteur.paquets) == 1


def test_apres_stop_pas_de_rearmement_automatique():
    b = BattementRC(Collecteur())
    b.proposer(AxesRC(), 1, "decoller")
    b.pas(1)
    b.proposer(AxesRC(), 1.1, "stop")
    assert b.pas(1.1)[11] == 2
    assert b.verrouille
    b.fermer()
    assert b.emetteur.paquets == []


def test_battement_avant_consigne_ou_horloge_invalide_refuses():
    b = BattementRC(Collecteur())
    with pytest.raises(ValueError):
        b.demarrer()
    with pytest.raises(ValueError):
        b.proposer(AxesRC(), math.nan)


def test_validation_liee_aux_octets_capture_et_identite(tmp_path, monkeypatch):
    capture = tmp_path / "capture-test.pcapng"
    capture.write_bytes(b"fixture sans trafic reel")
    d = {
        "identite": "fixture",
        "protocole": "e88_rcfpvpro_udp",
        "capture": capture.name,
        "sha256": hashlib.sha256(capture.read_bytes()).hexdigest(),
        "axes_valides": True,
        "arret_valide": True,
        "perte_liaison_validee": True,
    }
    fiche = tmp_path / "validation.json"
    fiche.write_text(json.dumps(d))
    validation = ValidationMateriel.charger(fiche, "fixture")
    with pytest.raises(ValueError):
        ValidationMateriel.charger(fiche, "autre-drone")
    with pytest.raises(ValueError):
        TransportUDP(validation)  # même avec une fiche, ouverture explicite requise
    with pytest.raises(ValueError):
        TransportUDP(replace(validation, axes_valides=False), autoriser=True)
    connexions = []

    class FauxSocket:
        def __init__(self, *args):
            self.port = None

        def bind(self, adresse):
            self.port = adresse[1]

        def setsockopt(self, *args):
            pass

        def settimeout(self, *args):
            pass

        def sendto(self, paquet, adresse):
            connexions.append((self.port, paquet, adresse))

        def close(self):
            pass

    import socket

    monkeypatch.setattr(socket, "socket", FauxSocket)
    t = TransportUDP(validation, autoriser=True)
    assert len(connexions) == 3
    assert all(c == (7070, HANDSHAKE, ("192.168.28.1", 7080)) for c in connexions)
    t.envoyer(paquet_rc(AxesRC()))
    assert connexions[-1][0] == 7060
    t.fermer()
    capture.write_bytes(b"capture modifiee")
    with pytest.raises(ValueError):
        ValidationMateriel.charger(fiche, "fixture")


def test_takeoff_envoye_sans_envol_confirme_ne_declenche_pas_land_toggle():
    c = Collecteur()
    b = BattementRC(c)
    b.proposer(AxesRC(), 1, "decoller")
    assert b.pas(1)[11] == 1
    assert b.pas(1.4)[11] == 0
    assert b.verrouille
    b.fermer()
    assert c.paquets == []


def test_horloge_du_battement_doit_rester_finie_et_monotone():
    b = BattementRC(Collecteur())
    b.proposer(AxesRC(), 10)
    with pytest.raises(ValueError, match="horloge"):
        b.pas(9)
    assert b.verrouille
