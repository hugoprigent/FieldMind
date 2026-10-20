"""Export du rover V3 (Robot_V3_rig.blend) en GLB allégé pour le poste opérateur.

À lancer avec Blender 5.2 ou le module Python `bpy` 5.2 (Python 3.13), depuis la racine du dépôt :
    blender -b --python software/outils/blender/exporter_rover_glb.py
    python3.13 software/outils/blender/exporter_rover_glb.py        # avec pip install bpy==5.2.0

Le fichier .blend source n'est jamais modifié (il est ouvert, transformé en mémoire, puis exporté).

Ce que fait le script :
- ne garde que le rover (la remorque et les drones ne sont pas dans la simulation physique) ;
- supprime les drivers et contraintes, et note dans un fichier JSON ce qu'il faut rejouer dans
  three.js : table du palonnier de servo, cibles des biellettes ;
- fusionne les pièces par pivot mobile (roues, fusées, essieu, arbre moteur...) ;
- supprime la visserie minuscule, soude et décime les maillages ;
- remplace les matériaux procéduraux (non exportables) par des matériaux PBR simples ;
- exporte en GLB (Y vers le haut, avant du rover vers +X).

Compression ensuite (meshopt, décodée par three.js sans fichier WASM séparé) :
    npx @gltf-transform/cli meshopt software/interface/web/modeles/rover_v3.glb \
        software/interface/web/modeles/rover_v3.glb
"""

from __future__ import annotations

import json
import math
import sys
from collections import defaultdict
from pathlib import Path

import bpy  # isort: skip  (bpy doit être importé avant bmesh et mathutils)
import bmesh
from mathutils import Vector

RACINE = Path(__file__).resolve().parents[3]
SOURCE = RACINE / "hardware/rover/animation/blender/Robot_V3_rig.blend"
SORTIE = RACINE / "software/interface/web/modeles/rover_v3.glb"
SORTIE_RIG = RACINE / "software/interface/web/modeles/rover_v3.rig.json"
TRIANGLES_CIBLE = 110_000
TAILLE_MIN_PIECE = 0.007
"""Diagonale (m) en dessous de laquelle une pièce (vis, écrou) est supprimée."""
PIVOTS_A_EXCLURE = ("PIV_Trailer",)


def journal(texte: str) -> None:
    print(f"[export] {texte}", flush=True)


def descendants(objet: bpy.types.Object) -> list[bpy.types.Object]:
    resultat = []
    for enfant in objet.children:
        resultat.append(enfant)
        resultat.extend(descendants(enfant))
    return resultat


def supprimer(objets: list[bpy.types.Object]) -> None:
    for objet in objets:
        bpy.data.objects.remove(objet, do_unlink=True)


def triangles(objet: bpy.types.Object) -> int:
    return sum(len(p.vertices) - 2 for p in objet.data.polygons) if objet.type == "MESH" else 0


def tables_drivers(robot: bpy.types.Object) -> dict[str, object]:
    """Évalue les drivers utiles sur leur plage et enregistre des tables pour three.js."""
    rig: dict[str, object] = {}
    horn = bpy.data.objects.get("PIV_ServoHorn")
    if horn and horn.animation_data:
        fc = next(f for f in horn.animation_data.drivers if f.data_path == "rotation_euler")
        expression = fc.driver.expression
        pas = list(range(-35, 36))
        valeurs = [eval(expression, {"__builtins__": {}}, {"s": float(s)}) for s in pas]
        rig["palonnier"] = {
            "noeud": "PIV_ServoHorn",
            "axe_blender": "xyz"[fc.array_index],
            "braquage_moyen_deg": pas,
            "angle_rad": [round(v, 6) for v in valeurs],
        }
    biellettes = []
    for nom in ("PIV_TieRod_L", "PIV_TieRod_R"):
        objet = bpy.data.objects.get(nom)
        if objet is None:
            continue
        contrainte = next((c for c in objet.constraints if c.type == "DAMPED_TRACK"), None)
        if contrainte is not None and contrainte.target is not None:
            biellettes.append(
                {
                    "noeud": nom,
                    "cible": contrainte.target.name,
                    "axe_blender": contrainte.track_axis,
                }
            )
    rig["biellettes"] = biellettes
    rig["rayon_roue"] = float(robot.get("wheel_radius_m", 0.1))
    return rig


def simplifier_materiaux(objets: list[bpy.types.Object]) -> int:
    """Un matériau PBR simple par couleur d'affichage : les nœuds procéduraux ne s'exportent pas."""
    remplacement: dict[str, bpy.types.Material] = {}
    for objet in objets:
        for emplacement in objet.material_slots:
            source = emplacement.material
            if source is None:
                continue
            if source.name not in remplacement:
                principled = None
                if source.node_tree:
                    principled = next(
                        (n for n in source.node_tree.nodes if n.type == "BSDF_PRINCIPLED"), None
                    )
                metal = float(principled.inputs["Metallic"].default_value) if principled else 0.0
                rugosite = 0.35 if metal > 0.5 else 0.6
                nom_lower = source.name.lower()
                if any(m in nom_lower for m in ("rubber", "tyre", "graphite", "black")):
                    rugosite = 0.85
                nouveau = bpy.data.materials.new(f"web_{source.name}")
                nouveau.use_nodes = True
                bsdf = nouveau.node_tree.nodes["Principled BSDF"]
                bsdf.inputs["Base Color"].default_value = tuple(source.diffuse_color)
                bsdf.inputs["Metallic"].default_value = metal
                bsdf.inputs["Roughness"].default_value = rugosite
                remplacement[source.name] = nouveau
            emplacement.material = remplacement[source.name]
    return len(remplacement)


def main() -> None:
    journal(f"ouverture de {SOURCE.relative_to(RACINE)}")
    bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
    robot = bpy.data.objects["CTRL_Robot"]
    rig = tables_drivers(robot)

    # 1. Ne garder que le rover.
    a_garder = {robot, *descendants(robot)}
    for nom in PIVOTS_A_EXCLURE:
        for objet in list(a_garder):
            if objet.name.startswith(nom):
                a_garder -= {objet, *descendants(objet)}
    a_garder = {o for o in a_garder if o.type in {"MESH", "EMPTY"}}
    supprimer([o for o in bpy.data.objects if o not in a_garder])
    journal(f"{len(bpy.data.objects)} objets gardés")

    # 2. Pose de référence : tout au neutre, sans drivers ni contraintes.
    for objet in bpy.data.objects:
        if objet.animation_data:
            for fc in list(objet.animation_data.drivers):
                objet.driver_remove(fc.data_path, fc.array_index)
            objet.animation_data_clear()
        for contrainte in list(objet.constraints):
            objet.constraints.remove(contrainte)
    robot.location = (0.0, 0.0, 0.0)
    robot.rotation_euler = (0.0, 0.0, 0.0)
    for objet in bpy.data.objects:
        if objet.name.startswith("PIV_"):
            objet.rotation_euler = (0.0, 0.0, 0.0)
    bpy.context.view_layer.update()

    # 3. Suppression de la visserie minuscule.
    petits = []
    for objet in bpy.data.objects:
        if objet.type == "MESH":
            coins = [objet.matrix_world @ Vector(c) for c in objet.bound_box]
            etendue = Vector(
                (
                    max(c.x for c in coins) - min(c.x for c in coins),
                    max(c.y for c in coins) - min(c.y for c in coins),
                    max(c.z for c in coins) - min(c.z for c in coins),
                )
            )
            if etendue.length < TAILLE_MIN_PIECE:
                petits.append(objet)
    supprimer(petits)
    journal(f"{len(petits)} petites pièces supprimées")

    # 4. Regroupement des maillages par pivot mobile (ou par le robot pour les pièces fixes).
    def pivot_de(objet: bpy.types.Object) -> bpy.types.Object:
        parent = objet.parent
        while parent is not None and not (parent.name.startswith("PIV_") or parent == robot):
            parent = parent.parent
        return parent or robot

    groupes: dict[str, list[bpy.types.Object]] = defaultdict(list)
    for objet in bpy.data.objects:
        if objet.type == "MESH":
            groupes[pivot_de(objet).name].append(objet)
    journal(f"{len(groupes)} groupes : " + ", ".join(sorted(groupes)))

    nb_materiaux = simplifier_materiaux([o for g in groupes.values() for o in g])
    journal(f"{nb_materiaux} matériaux simplifiés")

    fusionnes: list[bpy.types.Object] = []
    for nom_pivot, maillages in groupes.items():
        pivot = bpy.data.objects[nom_pivot]
        for m in maillages:
            if m.data.users > 1:
                m.data = m.data.copy()
        bpy.ops.object.select_all(action="DESELECT")
        for m in maillages:
            m.select_set(True)
        bpy.context.view_layer.objects.active = maillages[0]
        bpy.ops.object.convert(target="MESH")  # applique les modificateurs éventuels
        if len(maillages) > 1:
            bpy.ops.object.join()
        objet = bpy.context.view_layer.objects.active
        matrice = objet.matrix_world.copy()
        objet.parent = pivot
        objet.matrix_world = matrice
        objet.name = f"MAILLAGE_{nom_pivot}"
        fusionnes.append(objet)

    # Les empties intermédiaires (non pivots) ne servent plus.
    supprimer(
        [
            o
            for o in bpy.data.objects
            if o.type == "EMPTY" and o != robot and not o.name.startswith("PIV_")
        ]
    )

    # 5. Soudure et décimation.
    total = sum(triangles(o) for o in fusionnes)
    ratio = min(1.0, TRIANGLES_CIBLE / max(total, 1))
    journal(f"{total} triangles avant décimation, ratio {ratio:.3f}")
    for objet in fusionnes:
        maillage = bmesh.new()
        maillage.from_mesh(objet.data)
        bmesh.ops.remove_doubles(maillage, verts=maillage.verts, dist=0.0002)
        maillage.to_mesh(objet.data)
        maillage.free()
        if ratio < 1.0:
            modificateur = objet.modifiers.new("decimation", "DECIMATE")
            modificateur.ratio = ratio
            modificateur.use_collapse_triangulate = True
        objet.data.shade_smooth()
        objet.data.set_sharp_from_angle(angle=math.radians(35))
    bpy.context.view_layer.update()

    # 6. Export.
    SORTIE.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=str(SORTIE),
        export_format="GLB",
        export_yup=True,
        export_apply=True,
        export_animations=False,
        export_cameras=False,
        export_lights=False,
        export_extras=False,
        export_materials="EXPORT",
        export_normals=True,
        export_texcoords=False,
        export_draco_mesh_compression_enable=False,
    )
    total_final = sum(
        len(o.evaluated_get(bpy.context.evaluated_depsgraph_get()).data.polygons) for o in fusionnes
    )
    rig["noeuds"] = sorted(o.name for o in bpy.data.objects if o.name.startswith("PIV_"))
    rig["source"] = str(SOURCE.relative_to(RACINE))
    SORTIE_RIG.write_text(json.dumps(rig, indent=1, ensure_ascii=False), encoding="utf-8")
    taille = SORTIE.stat().st_size / 1e6
    journal(f"écrit {SORTIE.relative_to(RACINE)} : {taille:.2f} Mo, ~{total_final} faces")


if __name__ == "__main__":
    main()
    sys.exit(0)
