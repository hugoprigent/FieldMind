from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Ouverture de la coque autour du mât du lidar agrandie, en place sur la pièce C4 (sans refaire la
coque) : 61 × 61 mm au lieu de 43, pour que la coque passe par-dessus le mât monté (support et C1).
Les vis M8 du support restent sous le toit : on les serre avant de poser la coque. Même cote que
PROFIL_LIDAR et PASSE_LIDAR de 02_coque.py. Document du rover actif. Réexécutable.
Ensuite : 04_exporter.py (STL de C4).
"""
import adsk.core, adsk.fusion, json
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/pf_helpers.py'), encoding='utf-8').read(), 'pf_helpers.py', 'exec'))
YC, A, PASSE_L, PASSE_P = 380.0, 30.5, 12.0, 10.0
NOMS = ('ouv_lidar_manchon', 'ouv_cables_lidar_manchon')


def run(_context: str):
    co = next(o for o in group().component.occurrences if o.component.name == 'PF_IMP_Coque')
    c = co.component
    for coll in (c.features.extrudeFeatures, c.sketches, c.constructionPlanes):
        for x in reversed([coll.item(i) for i in range(coll.count)]):
            if x.isValid and x.name.endswith(NOMS):
                x.deleteMe()
    b = c.bRepBodies.itemByName('coque_C4')
    boite(c, NOMS[0], -A, YC - A, A, YC + A, ZB + 20, 100, op=CUT, bodies=[b])
    boite(c, NOMS[1], -PASSE_L, YC + A - 1, PASSE_L, YC + A + PASSE_P, ZB + 20, 100, op=CUT, bodies=[b])
    bx = b.boundingBox
    print(json.dumps({'C4_morceaux': b.lumps.count, 'C4_volume_cm3': round(b.volume, 1),
                      'C4_bbox': [round(v * 10, 1) for v in bx.minPoint.asArray() + bx.maxPoint.asArray()]}))
