"""Construction du modèle MuJoCo (MJCF) : sol, forêt et rover FieldMind.

Le rover est un châssis rigide sur quatre roues, sans suspension :
- direction Ackermann par deux pivots de fusée commandés en position (servo) ;
- propulsion par un moteur unique qui entraîne l'essieu arrière rigide par la courroie 3:1
  (les deux roues arrière tournent à la même vitesse, comme sur le rover réel) ;
- contact pneu-sol avec frottement de Coulomb (glissement possible au braquage et au freinage).

Groupes de géométries : 0 = monde (vu par le lidar), 1 = rover (ignoré par le lidar),
2 = habillage visuel sans physique (houppiers, vu seulement par la caméra).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from services.protocole.fiche_rover import FicheRover
from simulation.monde import Monde

GROUPE_MONDE = 0
GROUPE_ROVER = 1
GROUPE_HABILLAGE = 2
GROUPE_TERRAIN = 3
"""Relief : hors du lancer de rayons de MuJoCo (lent sur une « heightfield »), le lidar calcule
lui-même l'impact de ses faisceaux sur le terrain."""
HAUTEUR_PHYSIQUE_TRONC = 3.0
"""Hauteur des troncs dans la physique : largement au-dessus du rover et du lidar."""
PAS_PHYSIQUE = 0.002

ROUES = ("roue_av_g", "roue_av_d", "roue_ar_g", "roue_ar_d")
TERRAIN = "terrain"
"""Relief : « heightfield » déclarée à la bonne taille, hauteurs copiées après compilation."""
ENFOUISSEMENT = 0.3
"""Les troncs et rochers descendent sous le sol : pas de jour entre eux et une pente."""
TEXTURE_SOL = "sol"
"""Texture du sol : déclarée unie à la bonne taille, ses pixels sont copiés après compilation
(`RobotSimule`), sans passer par un fichier image."""
HOUPPIERS = ((1.3, 5.0), (1.7, 7.0), (2.1, 9.0), (2.5, 11.0))
"""Cônes des houppiers (rayon, hauteur) : un maillage par taille, choisi selon l'arbre."""


def _f(*valeurs: float) -> str:
    return " ".join(f"{v:.6g}" for v in valeurs)


def _cone(rayon: float, hauteur: float, cotes: int = 7) -> str:
    sommets = [0.0, 0.0, hauteur]
    for k in range(cotes):
        a = 2 * math.pi * k / cotes
        sommets += [rayon * math.cos(a), rayon * math.sin(a), 0.0]
    return _f(*sommets)


def construire_mjcf(
    fiche: FicheRover,
    monde: Monde,
    sol: tuple[float, float, float] | None = None,
    taille_texture: int = 1024,
) -> str:
    """Renvoie le modèle MJCF complet sous forme de texte XML.

    `sol` : (centre x, centre y, demi-côté) du plan texturé par `TEXTURE_SOL`, ou None pour un
    sol uni (la physique est la même : le plan est infini pour les contacts).
    """
    g = fiche.geometrie
    ph = fiche.physique
    r = g.rayon_roue
    braquage_butee = math.radians(40)  # butée du servo (±40°)

    racine = ET.Element("mujoco", model="fieldmind_foret")
    ET.SubElement(racine, "compiler", angle="radian", autolimits="true")
    ET.SubElement(
        racine,
        "option",
        timestep=str(PAS_PHYSIQUE),
        integrator="implicitfast",
        cone="elliptic",
        impratio="5",
    )
    ET.SubElement(racine, "size", memory="20M")

    defaut = ET.SubElement(racine, "default")
    ET.SubElement(defaut, "geom", solref="0.01 1", solimp="0.95 0.99 0.001")
    classe_rover = ET.SubElement(defaut, "default", {"class": "rover"})
    ET.SubElement(classe_rover, "geom", group=str(GROUPE_ROVER), friction="0.5 0.005 0.0001")
    classe_roue = ET.SubElement(classe_rover, "default", {"class": "roue"})
    ET.SubElement(
        classe_roue,
        "geom",
        type="cylinder",
        size=_f(r),
        fromto=_f(0, -g.largeur_roue / 2, 0, 0, g.largeur_roue / 2, 0),
        friction=_f(ph.frottement_pneu, 0.005, 0.0005),
        condim="4",
        mass=_f(ph.masse_roue),
    )
    ET.SubElement(
        classe_roue, "joint", type="hinge", axis="0 1 0", damping="0.004", armature="0.002"
    )

    # ------------------------------------------------------------------ monde
    visuel = ET.SubElement(racine, "visual")
    ET.SubElement(visuel, "headlight", ambient="0.42 0.42 0.42", diffuse="0.25 0.25 0.25")
    ET.SubElement(visuel, "map", fogstart="0.35", fogend="1.6", znear="0.002")
    ET.SubElement(visuel, "rgba", fog="0.84 0.84 0.78 1", haze="0.84 0.84 0.78 1")
    ET.SubElement(visuel, "quality", offsamples="0")
    ET.SubElement(racine, "statistic", extent="60", center="-3 0 1")
    assets = ET.SubElement(racine, "asset")
    ET.SubElement(
        assets,
        "texture",
        type="skybox",
        builtin="gradient",
        rgb1="0.63 0.77 0.9",
        rgb2="0.9 0.9 0.84",
        width="64",
        height="64",
    )
    attributs_sol: dict[str, str] = {"rgba": "0.35 0.3 0.22 1", "size": "0 0 1"}
    if sol is not None:
        ET.SubElement(
            assets,
            "texture",
            name=TEXTURE_SOL,
            type="2d",
            builtin="flat",
            rgb1="0.4 0.5 0.24",
            width=str(taille_texture),
            height=str(taille_texture),
        )
        ET.SubElement(assets, "material", name="sol", texture="sol", texrepeat="1 1")
        attributs_sol = {
            "material": "sol",
            "size": _f(sol[2], sol[2], 1),
            "pos": _f(sol[0], sol[1], 0),
        }
    # Grain des surfaces (écorce, pierre, bois mort) : sans lui, la paire stéréo ne trouverait
    # rien à apparier sur un tronc de couleur unie, pas plus qu'une vraie caméra sur un mur lisse.
    for nom, rgb1, rgb2, marque in (
        ("tronc", "0.36 0.25 0.17", "0.27 0.18 0.12", "0.5 0.38 0.27"),
        ("rocher", "0.53 0.53 0.51", "0.42 0.42 0.4", "0.7 0.7 0.66"),
        ("bois_mort", "0.36 0.26 0.17", "0.25 0.17 0.11", "0.52 0.42 0.3"),
    ):
        ET.SubElement(
            assets,
            "texture",
            name=f"grain_{nom}",
            type="2d",
            builtin="checker",
            rgb1=rgb1,
            rgb2=rgb2,
            mark="random",
            random="0.35",
            markrgb=marque,
            width="64",
            height="64",
        )
    ET.SubElement(
        assets, "material", name="tronc", texture="grain_tronc", texrepeat="3 3", specular="0"
    )
    ET.SubElement(assets, "material", name="houppier", rgba="0.16 0.3 0.16 1", specular="0")
    ET.SubElement(
        assets, "material", name="rocher", texture="grain_rocher", texrepeat="4 4", specular="0.05"
    )
    ET.SubElement(
        assets,
        "material",
        name="bois_mort",
        texture="grain_bois_mort",
        texrepeat="4 4",
        specular="0",
    )
    for k, (rayon, hauteur) in enumerate(HOUPPIERS):
        ET.SubElement(assets, "mesh", name=f"houppier_{k}", vertex=_cone(rayon, hauteur))
    monde_xml = ET.SubElement(racine, "worldbody")
    ET.SubElement(
        monde_xml,
        "light",
        directional="true",
        pos="0 0 30",
        dir="-0.5 -0.45 -0.75",
        diffuse="0.75 0.7 0.6",
        castshadow="false",
    )
    relief = monde.relief
    if relief is None:
        ET.SubElement(
            monde_xml,
            "geom",
            name="sol",
            type="plane",
            friction="0.8 0.005 0.0001",
            attrib={"group": str(GROUPE_MONDE), **attributs_sol},
        )
    else:
        n, m = relief.forme
        rx, ry = (m - 1) * relief.pas / 2, (n - 1) * relief.pas / 2
        zmin, zmax = float(relief.hauteurs.min()), float(relief.hauteurs.max())
        ET.SubElement(
            assets,
            "hfield",
            name=TERRAIN,
            nrow=str(n),
            ncol=str(m),
            size=_f(rx, ry, max(zmax - zmin, 0.01), 0.5),
        )
        attributs_terrain = {k: v for k, v in attributs_sol.items() if k == "material"}
        ET.SubElement(
            monde_xml,
            "geom",
            name="sol",
            type="hfield",
            hfield=TERRAIN,
            pos=_f(relief.origine[0] + rx, relief.origine[1] + ry, zmin),
            friction="0.8 0.005 0.0001",
            attrib={"group": str(GROUPE_TERRAIN), **attributs_terrain},
        )
    sol_en = monde.hauteur_sol
    for i, arbre in enumerate(monde.arbres + monde.lisiere):
        z_sol = sol_en(arbre.x, arbre.y)
        ET.SubElement(
            monde_xml,
            "geom",
            name=f"arbre_{i}",
            type="cylinder",
            pos=_f(arbre.x, arbre.y, z_sol + HAUTEUR_PHYSIQUE_TRONC / 2 - ENFOUISSEMENT),
            size=_f(arbre.rayon, HAUTEUR_PHYSIQUE_TRONC / 2),
            group=str(GROUPE_MONDE),
            material="tronc",
        )
        # Houppier de conifère : visuel seulement (ni contact, ni lidar).
        k = min(range(len(HOUPPIERS)), key=lambda j: abs(HOUPPIERS[j][1] - 0.75 * arbre.hauteur))
        ET.SubElement(
            monde_xml,
            "geom",
            type="mesh",
            mesh=f"houppier_{k}",
            pos=_f(arbre.x, arbre.y, z_sol + arbre.hauteur - HOUPPIERS[k][1]),
            euler=_f(0, 0, (i * 0.7) % 6.28),
            group=str(GROUPE_HABILLAGE),
            contype="0",
            conaffinity="0",
            material="houppier",
        )
    for i, rocher in enumerate(monde.rochers):
        sommets: list[float] = []
        for px, py in rocher.contour:
            sommets += [px, py, -ENFOUISSEMENT]
        for px, py in rocher.contour:
            sommets += [px * rocher.retrait_sommet, py * rocher.retrait_sommet, rocher.hauteur]
        ET.SubElement(assets, "mesh", name=f"rocher_{i}", vertex=_f(*sommets))
        ET.SubElement(
            monde_xml,
            "geom",
            name=f"rocher_{i}",
            type="mesh",
            mesh=f"rocher_{i}",
            pos=_f(rocher.x, rocher.y, sol_en(rocher.x, rocher.y)),
            group=str(GROUPE_MONDE),
            material="rocher",
        )
    for i, tronc in enumerate(monde.troncs_couches):
        (xa, ya), (xb, yb) = tronc.extremites
        ET.SubElement(
            monde_xml,
            "geom",
            name=f"tronc_couche_{i}",
            type="cylinder",
            fromto=_f(xa, ya, sol_en(xa, ya) + tronc.rayon, xb, yb, sol_en(xb, yb) + tronc.rayon),
            size=_f(tronc.rayon),
            group=str(GROUPE_MONDE),
            friction="0.7 0.005 0.0001",
            material="bois_mort",
        )

    # ------------------------------------------------------------------ rover
    # Origine du corps « chassis » : centre de l'emprise au sol, à la hauteur des moyeux.
    d = monde.scenario.depart
    chassis = ET.SubElement(
        monde_xml,
        "body",
        name="chassis",
        pos=_f(d.x, d.y, sol_en(d.x, d.y) + r + 0.02),
        euler=_f(0, 0, d.cap),
        childclass="rover",
    )
    ET.SubElement(chassis, "freejoint", name="libre")

    masse_roues = 4 * ph.masse_roue + 2 * 0.1
    masse_batterie = 0.25 * ph.masse_totale
    masse_accessoires = 0.5  # pare-chocs et boîtier du lidar
    masse_caisse = ph.masse_totale - masse_roues - masse_batterie - masse_accessoires
    bas = g.garde_au_sol - r  # face inférieure du châssis, dans le repère du corps
    hauteur_caisse = 0.30 - g.garde_au_sol
    demi_long = g.longueur / 2 - 0.05
    demi_larg = g.voie_arriere / 2 - g.largeur_roue / 2 - 0.02
    ET.SubElement(
        chassis,
        "geom",
        name="caisse",
        type="box",
        size=_f(demi_long, demi_larg, hauteur_caisse / 2),
        pos=_f(0, 0, bas + hauteur_caisse / 2),
        mass=_f(masse_caisse),
        rgba="0.75 0.1 0.08 1",
    )
    ET.SubElement(
        chassis,
        "geom",
        name="batterie",
        type="box",
        size=_f(0.12, 0.08, 0.04),
        pos=_f(-0.05, 0, bas + 0.045),
        mass=_f(masse_batterie),
        contype="0",
        conaffinity="0",
        rgba="0.2 0.2 0.2 1",
    )
    ET.SubElement(
        chassis,
        "geom",
        name="pare_chocs_avant",
        type="box",
        size=_f(0.025, 0.14, 0.05),
        pos=_f(g.longueur / 2 - 0.025, 0, bas + 0.09),
        mass="0.2",
        rgba="0.15 0.15 0.15 1",
    )
    ET.SubElement(
        chassis,
        "geom",
        name="pare_chocs_arriere",
        type="box",
        size=_f(0.025, 0.14, 0.05),
        pos=_f(-g.longueur / 2 + 0.025, 0, bas + 0.09),
        mass="0.2",
        rgba="0.15 0.15 0.15 1",
    )
    lx, ly, lz = fiche.capteurs.lidar.position
    ET.SubElement(
        chassis,
        "geom",
        name="lidar_boitier",
        type="cylinder",
        size=_f(0.035, 0.02),
        pos=_f(lx, ly, lz - r - 0.025),
        mass="0.1",
        rgba="0.1 0.1 0.1 1",
    )
    ET.SubElement(chassis, "site", name="lidar", pos=_f(lx, ly, lz - r))
    gx, gy, gz = fiche.capteurs.gnss.position
    ET.SubElement(chassis, "site", name="gnss", pos=_f(gx, gy, gz - r))
    ET.SubElement(chassis, "site", name="imu", pos=_f(0, 0, 0.05))
    camera = fiche.capteurs.camera
    if camera is not None:
        # Caméra MuJoCo : elle regarde selon −Z, +X à droite de l'image, +Y vers le haut.
        cx, cy, cz = camera.position
        plongee = -math.radians(camera.inclinaison_deg)
        s, c = math.sin(plongee), math.cos(plongee)
        ET.SubElement(
            chassis,
            "camera",
            name="camera",
            pos=_f(cx, cy, cz - r),
            xyaxes=_f(0, -1, 0, s, 0, c),
            fovy=_f(camera.champ_vertical_deg),
        )
        if fiche.capteurs.stereo is not None:
            # Œil droit de la paire : même orientation, décalé de la base vers la droite (−y).
            ET.SubElement(
                chassis,
                "camera",
                name="camera_droite",
                pos=_f(cx, cy - fiche.capteurs.stereo.base_m, cz - r),
                xyaxes=_f(0, -1, 0, s, 0, c),
                fovy=_f(camera.champ_vertical_deg),
            )

    x_av = g.essieu_arriere_x + g.empattement
    x_ar = g.essieu_arriere_x
    for cote, signe in (("g", 1.0), ("d", -1.0)):
        fusee = ET.SubElement(
            chassis, "body", name=f"fusee_{cote}", pos=_f(x_av, signe * g.voie_avant / 2, 0)
        )
        ET.SubElement(
            fusee,
            "joint",
            name=f"braquage_{cote}",
            type="hinge",
            axis="0 0 1",
            range=_f(-braquage_butee, braquage_butee),
            damping="0.3",
            armature="0.01",
        )
        ET.SubElement(
            fusee,
            "geom",
            type="sphere",
            size="0.02",
            mass="0.1",
            contype="0",
            conaffinity="0",
        )
        roue = ET.SubElement(fusee, "body", name=f"roue_av_{cote}")
        ET.SubElement(roue, "joint", {"name": f"roue_av_{cote}", "class": "roue"})
        ET.SubElement(roue, "geom", {"name": f"pneu_av_{cote}", "class": "roue"})

        roue_ar = ET.SubElement(
            chassis, "body", name=f"roue_ar_{cote}", pos=_f(x_ar, signe * g.voie_arriere / 2, 0)
        )
        ET.SubElement(roue_ar, "joint", {"name": f"roue_ar_{cote}", "class": "roue"})
        ET.SubElement(roue_ar, "geom", {"name": f"pneu_ar_{cote}", "class": "roue"})

    # Les roues ne frottent pas contre la caisse : sur le rover, le débattement est dégagé.
    exclusions = ET.SubElement(racine, "contact")
    for roue_nom in ROUES:
        ET.SubElement(exclusions, "exclude", body1="chassis", body2=roue_nom)

    # Essieu arrière rigide : les deux roues arrière tournent ensemble.
    tendons = ET.SubElement(racine, "tendon")
    essieu = ET.SubElement(tendons, "fixed", name="essieu_arriere")
    ET.SubElement(essieu, "joint", joint="roue_ar_g", coef="0.5")
    ET.SubElement(essieu, "joint", joint="roue_ar_d", coef="0.5")
    egalites = ET.SubElement(racine, "equality")
    ET.SubElement(
        egalites,
        "joint",
        joint1="roue_ar_g",
        joint2="roue_ar_d",
        polycoef="0 1 0 0 0",
        solref="0.004 1",
    )

    actionneurs = ET.SubElement(racine, "actuator")
    moteur = ph.moteur
    ET.SubElement(
        actionneurs,
        "motor",
        name="moteur",
        tendon="essieu_arriere",
        gear=_f(moteur.rapport_transmission),
        ctrlrange=_f(-moteur.couple_max, moteur.couple_max),
    )
    for cote in ("g", "d"):
        ET.SubElement(
            actionneurs,
            "position",
            name=f"servo_{cote}",
            joint=f"braquage_{cote}",
            kp="300",
            kv="4",
            forcerange=_f(-ph.servo.couple_max, ph.servo.couple_max),
            ctrlrange=_f(-braquage_butee, braquage_butee),
        )

    capteurs = ET.SubElement(racine, "sensor")
    ET.SubElement(capteurs, "gyro", name="gyro", site="imu")
    ET.SubElement(capteurs, "accelerometer", name="accel", site="imu")
    ET.SubElement(capteurs, "framequat", name="orientation", objtype="site", objname="imu")
    for roue_nom in ROUES:
        ET.SubElement(capteurs, "jointvel", name=f"vel_{roue_nom}", joint=roue_nom)
    for cote in ("g", "d"):
        ET.SubElement(capteurs, "jointpos", name=f"pos_braquage_{cote}", joint=f"braquage_{cote}")

    ET.indent(racine)
    return ET.tostring(racine, encoding="unicode")
