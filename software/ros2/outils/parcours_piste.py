"""Évaluation : parcours de la piste d'un monde généré, par étapes, avec Nav2.

À lancer avec la simulation (graine N) et l'autonomie en marche :

    python3 outils/parcours_piste.py ~/.cache/fieldmind/mondes/7/description.json --pas 8

La vérité terrain de la simulation (/sim/verite) sert seulement à mesurer. Les cibles sont prises
le long de la piste, tous les `pas` points (≈ 2 m chacun). Une cible qui tombe sur un obstacle
(tronc en travers) est reportée au point libre suivant. Pour chaque étape, on mesure le statut
Nav2, l'écart final à la cible réelle, le temps simulé, la distance minimale aux objets et
l'erreur de la localisation (repère map comparé à la vérité, correspondance fixée au départ).
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.time import Time

DEPASSEMENT_S = 900.0
ECART_ARRET_M = 3.0


def lacet(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def distance_objet(x: float, y: float, o: dict) -> float:
    """Distance au bord d'un objet : segment pour un tronc, disque sinon."""
    ox, oy = o["extra"].get("ex", o["x"]), o["extra"].get("ey", o["y"])
    if o["type"] == "tronc":
        demi = o["extra"]["longueur"] / 2
        dx, dy = math.cos(o["lacet"]), math.sin(o["lacet"])
        t = max(-demi, min(demi, (x - ox) * dx + (y - oy) * dy))
        return math.hypot(x - ox - t * dx, y - oy - t * dy) - o["extra"]["rayon"]
    return math.hypot(x - ox, y - oy) - o["rayon"]


def cibles(desc: dict, pas: int) -> list[tuple[float, float]]:
    piste = [tuple(p) for p in desc["piste"]]

    def libre(p) -> bool:
        return all(distance_objet(p[0], p[1], o) > 1.2 for o in desc["objets"])

    indices = list(range(pas, len(piste), pas))
    if (len(piste) - 1) % pas:
        indices.append(len(piste) - 1)
    resultat: list[tuple[float, float]] = []
    for i in indices:
        j = i
        while j < len(piste) - 1 and not libre(piste[j]):
            j += 1
        if not libre(piste[j]):
            j = i
            while j > 0 and not libre(piste[j]):
                j -= 1
        if not resultat or piste[j] != resultat[-1]:
            resultat.append(piste[j])
    return resultat


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("description", type=Path)
    p.add_argument("--pas", type=int, default=8)
    p.add_argument("--sortie", type=Path, help="bilan JSON")
    a = p.parse_args()
    desc = json.loads(a.description.expanduser().read_text())

    rclpy.init()
    n = rclpy.create_node(
        "parcours_piste", parameter_overrides=[Parameter("use_sim_time", value=True)]
    )
    tf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf, n)
    etat: dict = {}
    n.create_subscription(PoseStamped, "/sim/verite", lambda m: etat.__setitem__("p", m.pose), 10)
    n.create_subscription(
        DiagnosticStatus, "/gnss/etat", lambda m: etat.__setitem__("gnss", m.message), 10
    )

    def tourner(duree: float) -> None:
        fin = time.time() + duree
        while time.time() < fin:
            rclpy.spin_once(n, timeout_sec=0.05)

    def pose_map():
        for _ in range(40):
            try:
                return tf.lookup_transform("map", "base_link", Time()).transform
            except tf2_ros.TransformException:
                tourner(0.25)
        raise RuntimeError("pas de transformée map → base_link")

    client = ActionClient(n, NavigateToPose, "navigate_to_pose")
    client.wait_for_server()
    while "p" not in etat:
        tourner(0.2)
    # Correspondance monde → map fixée une fois : les erreurs de localisation s'accumulent.
    t = pose_map()
    v = etat["p"]
    bx, by = t.translation.x, t.translation.y
    wx, wy = v.position.x, v.position.y
    r = lacet(t.rotation) - lacet(v.orientation)

    def vers_map(x: float, y: float) -> tuple[float, float]:
        return (
            bx + math.cos(r) * (x - wx) - math.sin(r) * (y - wy),
            by + math.sin(r) * (x - wx) + math.cos(r) * (y - wy),
        )

    bilan = []
    etapes = cibles(desc, a.pas)
    for k, (cx, cy) in enumerate(etapes):
        gx, gy = vers_map(cx, cy)
        but = NavigateToPose.Goal()
        but.pose.header.frame_id = "map"
        but.pose.pose.position.x, but.pose.pose.position.y = gx, gy
        but.pose.pose.orientation.w = 1.0
        envoi = client.send_goal_async(but)
        rclpy.spin_until_future_complete(n, envoi)
        poignee = envoi.result()
        resultat = poignee.get_result_async()
        t0, ts0 = time.time(), n.get_clock().now().nanoseconds * 1e-9
        marge, proche, err_loc, modes = math.inf, "", 0.0, set()
        while not resultat.done() and time.time() - t0 < DEPASSEMENT_S:
            rclpy.spin_once(n, timeout_sec=0.1)
            q = etat["p"].position
            for o in desc["objets"]:
                d = distance_objet(q.x, q.y, o)
                if d < marge:
                    marge, proche = d, o["type"]
            try:
                m = tf.lookup_transform("map", "base_link", Time()).transform.translation
                mx, my = vers_map(q.x, q.y)
                err_loc = max(err_loc, math.hypot(m.x - mx, m.y - my))
            except tf2_ros.TransformException:
                pass
            modes.add(etat.get("gnss", "?"))
        if not resultat.done():
            poignee.cancel_goal_async()
            tourner(2)
        q = etat["p"].position
        ecart = math.hypot(q.x - cx, q.y - cy)
        statut = resultat.result().status if resultat.done() else "délai"
        duree = n.get_clock().now().nanoseconds * 1e-9 - ts0
        ligne = {
            "etape": k + 1,
            "cible": [round(cx, 2), round(cy, 2)],
            "statut": statut,
            "atteinte": statut == 4,
            "ecart_m": round(ecart, 2),
            "duree_s": round(duree, 1),
            "marge_min_m": round(marge, 2),
            "objet_proche": proche,
            "erreur_localisation_max_m": round(err_loc, 2),
            "gnss": sorted(modes),
        }
        bilan.append(ligne)
        print(json.dumps(ligne, ensure_ascii=False), flush=True)
        if ecart > ECART_ARRET_M:
            break
    atteintes = sum(e["atteinte"] for e in bilan)
    print(f"bilan : {atteintes}/{len(etapes)} étapes atteintes", flush=True)
    if a.sortie:
        a.sortie.write_text(
            json.dumps({"graine": desc["graine"], "etapes": bilan}, ensure_ascii=False, indent=1)
        )
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
