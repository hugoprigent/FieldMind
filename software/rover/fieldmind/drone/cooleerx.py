"""Encodage pur du BLE CooleerX extrait de HD dragon 1.1.0.

Les tests vérifient uniquement l'encodage de valeurs hors ligne. Ce module n'ouvre aucune liaison
et ne valide pas le comportement du drone réel.
Les trames RC doivent être entretenues à 20 Hz par un futur transport dédié.
"""

from __future__ import annotations

SERVICE_UUID = "07af27a5-9c22-11ea-9afe-02fcdc4e7412"
COMMANDE_UUID = "07af27a7-9c22-11ea-9afe-02fcdc4e7412"
VIDEO_UUID = "07af27a9-9c22-11ea-9afe-02fcdc4e7412"
PERIODE_RC_S = 0.05
# Durées pendant lesquelles les boutons de l'application maintiennent leur bit.
DUREES_ACTION_S = {"decoller": 1.0, "atterrir": 1.0, "stop": 1.0, "calibrer": 2.0}
DRAPEAUX_ACTION = {"decoller": 0x01, "atterrir": 0x02, "stop": 0x04, "calibrer": 0x80}
QUITTER_MANETTE = b"\x03\x01"  # sortie de mode ; pas une preuve de coupure moteur


def _octet(nom: str, valeur: int) -> int:
    if type(valeur) is not int or not 0 <= valeur <= 255:
        raise ValueError(f"{nom} : entier attendu entre 0 et 255")
    return valeur


def paquet_rc(
    *,
    roulis: int = 128,
    tangage: int = 128,
    gaz: int = 128,
    lacet: int = 128,
    action: str | None = None,
    sans_tete: bool = False,
) -> bytes:
    """Trame courte, profil 10 par défaut d'une session BLE de l'application.

    Valeurs brutes des axes, pas des vitesses physiques ni des pourcentages.
    Le neutre gaz à 128 correspond au mode maintien de hauteur de l'application.
    Un bouton d'arrêt conserve les axes fournis, comme dans l'APK.
    """
    if action is not None and action not in DRAPEAUX_ACTION:
        raise ValueError("action inconnue pour le RC CooleerX")
    if type(sans_tete) is not bool:
        raise ValueError("sans_tete : booléen attendu")
    r = max(1, _octet("roulis", roulis))
    p = max(1, _octet("tangage", tangage))
    g = _octet("gaz", gaz)
    y = max(1, _octet("lacet", lacet))
    if g == 1:
        g = 0  # normalisation explicite présente dans R7$b.run
    drapeau = (DRAPEAUX_ACTION[action] if action else 0) | (0x10 if sans_tete else 0)
    return bytes((0x02, 0x66, r, p, g, y, drapeau, r ^ p ^ g ^ y ^ drapeau, 0x99))


def paquet_camera(index: int) -> bytes:
    """Sélection 1/2 de v2.y ; l'identité physique reste à observer dans l'image."""
    if type(index) is not int or index not in (1, 2):
        raise ValueError("caméra CooleerX attendue : 1 ou 2")
    return bytes((0x01, index))
