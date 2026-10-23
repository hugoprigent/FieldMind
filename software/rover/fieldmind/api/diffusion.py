"""Construction des messages envoyés au poste (PROTOCOLE.md) à partir du bus et des modules."""

from __future__ import annotations

import base64
import math
from typing import Any

import cv2
import numpy as np

from fieldmind.bus import S
from fieldmind.types import (
    CarteOccupation,
    Chemin,
    EtatDrone,
    EtatEsp,
    Gnss,
    Imu,
    Obstacles,
    Odometrie,
    Scan,
)


def _r(valeur: float | None, n: int = 3) -> float | None:
    if valeur is None or not math.isfinite(valeur):
        return None
    return round(valeur, n)


def message_etat(rover: Any) -> dict[str, Any]:
    bus = rover.bus
    c = rover.conduite
    consigne = c.consigne_courante()
    commande = c.resume()
    odom: Odometrie | None = bus.dernier(S.ODOM)
    esp: EtatEsp | None = bus.dernier(S.ESP)
    drone: EtatDrone | None = bus.dernier(S.DRONE)
    imu: Imu | None = bus.dernier(S.IMU)
    gnss: Gnss | None = bus.dernier(S.GNSS)
    s = c.securite
    message: dict[str, Any] = {
        "type": "etat", "t": round(bus_temps(), 3),
        **c.etat_mode(),
        "consigne": {"vitesse": _r(consigne.vitesse), "braquage": _r(consigne.braquage),
                     "source": consigne.source},
        "commande": {k: (_r(v) if isinstance(v, float) else v) for k, v in commande.items()},
        "odom": None if odom is None else {
            "x": _r(odom.pose.x), "y": _r(odom.pose.y), "cap": _r(odom.pose.cap, 4),
            "vitesse": _r(odom.vitesse), "vitesse_lacet": _r(odom.vitesse_lacet),
            "qualite": _r(odom.qualite, 2)},
        "securite": {"arret": s.arret, "vitesse_max_avant": _r(s.vitesse_max_avant),
                     "vitesse_max_arriere": _r(s.vitesse_max_arriere), "raisons": s.raisons,
                     "libre_avant_m": _r(s.libre_avant_m, 2),
                     "libre_arriere_m": _r(s.libre_arriere_m, 2)},
        "esp": None if esp is None else {
            "connecte": esp.connecte, "firmware": esp.firmware, "defaut": esp.defaut,
            "raison_defaut": esp.raison_defaut, "pwm_g": esp.pwm_g, "pwm_d": esp.pwm_d,
            "sens_g": esp.sens_g, "sens_d": esp.sens_d, "frein": esp.frein,
            "servo_actif": esp.servo_actif, "servo_deg": _r(esp.servo_deg, 1),
            "chien_de_garde_ms": esp.chien_de_garde_ms, "age_s": _r(min(bus.age(S.ESP), 99), 2)},
        "drone": None if drone is None else {
            "etat": drone.etat, "connecte": drone.connecte, "ssid": drone.ssid,
            "bssid": drone.bssid, "canal": drone.canal, "rssi_dbm": drone.rssi_dbm,
            "ip_locale": drone.ip_locale, "passerelle": drone.passerelle,
            "protocole": drone.protocole, "port_video": drone.port_video,
            "port_cmd": drone.port_cmd, "paquets_rx": drone.paquets_rx, "ko_rx": drone.ko_rx,
            "paquets_tx": drone.paquets_tx, "ips": _r(drone.ips, 1),
            "images_rx": drone.images_rx, "batterie_pct": drone.batterie_pct,
            "commande_vol": drone.commande_vol, "candidat_scan": drone.candidat_scan,
            "reseaux_scan": drone.reseaux_scan, "mode_recon": drone.mode_recon,
            "secteur_360": drone.secteur_360, "croix_trouvee": drone.croix_trouvee,
            "croix_verrouillee": drone.croix_verrouillee, "croix_x": _r(drone.croix_x, 2),
            "croix_y": _r(drone.croix_y, 2), "croix_taille_pct": _r(drone.croix_taille_pct, 1),
            "consigne_vol": drone.consigne_vol,
            "pilotage_valide": drone.pilotage_valide,
            "analyse_ms": _r(drone.analyse_ms, 2), "montage_recon": drone.montage_recon,
            "age_s": _r(min(bus.age(S.DRONE), 99), 2)},
        "drone_atelier": bus.dernier(S.DRONE_ATELIER),
        "imu": None if imu is None else _imu(imu),
        "gnss": None if gnss is None else {
            "fix": gnss.fix, "latitude": gnss.latitude, "longitude": gnss.longitude,
            "altitude_m": gnss.altitude_m, "hdop": gnss.hdop, "satellites": gnss.satellites,
            "vitesse_m_s": _r(gnss.vitesse_m_s, 2), "cap_deg": gnss.cap_deg},
        "capteurs": rover.capteurs(),
        "remorque": _remorque(rover),
        "cible": rover.suivi.resume() if getattr(rover, "suivi", None) is not None else None,
        "situation": _situation(bus),
        "mission": rover.executeur.resume(),
        "monde": rover.monde_semantique.resume() if hasattr(rover, "monde_semantique") else None,
        "voix": rover.ia.etat_voix() if rover.ia is not None else {"active": False},
        "ia": {"occupee": bool(rover.ia.occupee()) if rover.ia is not None else False,
               "consigne_en_cours": bool(getattr(getattr(rover.ia, "agent", None),
                                                 "occupee", False))},
    }
    verite = getattr(rover.materiel, "verite", None)
    if verite is not None:
        message["simulation"] = {k: _r(v) for k, v in verite().items()}
    return message


def _situation(bus: Any) -> dict[str, Any] | None:
    """Agent de veille : dernier résumé (moins de 2 min)."""
    situation = bus.dernier(S.SITUATION)
    if not isinstance(situation, dict) or bus.age(S.SITUATION) > 120:
        return None
    return {"resume": situation.get("resume"), "niveau": situation.get("niveau"),
            "alerte": situation.get("alerte"), "age_s": _r(bus.age(S.SITUATION), 1)}


def _remorque(rover: Any) -> dict[str, Any] | None:
    suivi = getattr(rover, "remorque", None)
    if suivi is None or not suivi.etat.attelee:
        return None
    return suivi.etat.resume(suivi.geometrie(), rover.profil)


def _imu(imu: Imu) -> dict[str, Any]:
    roulis, tangage = imu.roulis_tangage()
    return {"etat": imu.etat, "accel_g": [_r(v) for v in imu.accel_g],
            "gyro_rad_s": [_r(v, 4) for v in imu.gyro_rad_s],
            "roulis_deg": _r(math.degrees(roulis), 1), "tangage_deg": _r(math.degrees(tangage), 1),
            "lacet_deg": None if imu.lacet_rad is None else _r(math.degrees(imu.lacet_rad), 1),
            "temperature_c": imu.temperature_c}


def bus_temps() -> float:
    import time

    return time.monotonic()


def message_scan(scan: Scan) -> dict[str, Any]:
    points = np.round(scan.points() * 100).astype(np.int32).ravel()
    return {"type": "scan", "t": round(scan.t, 3), "xy": points.tolist()}


def message_obstacles(obstacles: Obstacles) -> dict[str, Any]:
    points = obstacles.points
    if len(points) > 600:  # le poste n'a pas besoin de tout
        points = points[np.linspace(0, len(points) - 1, 600).astype(int)]
    return {"type": "obstacles", "t": round(obstacles.t, 3), "source": obstacles.source,
            "xy": np.round(points * 100).astype(np.int32).ravel().tolist(),
            "portee_m": obstacles.portee_m}


def message_carte(carte: CarteOccupation, trajectoire: list[tuple[float, float]],
                  chemin: Chemin | None, but: tuple[float, float] | None) -> dict[str, Any]:
    c = carte.cellules
    gris = np.full(c.shape, 128, np.uint8)
    connu = c >= 0
    gris[connu] = (255 - np.clip(c[connu].astype(np.int32) * 255 // 100, 0, 255)).astype(np.uint8)
    ok, png = cv2.imencode(".png", np.flipud(gris), [cv2.IMWRITE_PNG_COMPRESSION, 6])
    return {
        "type": "carte", "t": round(carte.t, 3), "resolution": carte.resolution,
        "origine": [round(carte.origine_x, 3), round(carte.origine_y, 3)],
        "largeur": int(c.shape[1]), "hauteur": int(c.shape[0]),
        "png": base64.b64encode(png.tobytes()).decode() if ok else "",
        "trajectoire": [list(p) for p in trajectoire[-800:]],
        "chemin": None if chemin is None else np.round(chemin.points, 2).tolist(),
        "but": None if but is None else [round(but[0], 2), round(but[1], 2)],
    }


def message_systeme(systeme: dict[str, Any]) -> dict[str, Any]:
    return {"type": "systeme", **systeme}

