from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Planche de 5 mm, modification en place (sans reconstruire, pour garder les modifications
faites à la main dans le document, ex. congés ajoutés sur les fusées).

Le servo est posé par ses oreilles sur le dessus de la planche : avec une planche plus fine il
descend, et la colonne du palonnier imprimé doit raccourcir d'autant. On rend donc dépendants de
d4_planche_ep : colonne, centrage du palonnier rond, trous et puits M2, cannelure et palonnier rond
du servo de référence. La direction, les roues et les supports moteurs sont référencés au dessous
de la planche (130 mm du sol) et ne bougent pas.
"""
import adsk.core, adsk.fusion
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
GROUP = 'FM_Direction_V04'


def distance(f, expr):
    adsk.fusion.DistanceExtentDefinition.cast(f.extentOne).distance.expression = expr


def decalage(c, nom, expr):
    p = c.constructionPlanes.itemByName(nom)
    adsk.fusion.ConstructionPlaneOffsetDefinition.cast(p.definition).offset.expression = expr


def run(_context: str):
    d = design()
    pal = occ('D4_IMP_Palonnier').component
    ex = pal.features.extrudeFeatures
    distance(ex.itemByName('extr_colonne'), 'd4_planche_ep')
    decalage(pal, 'plan_centrage_disque', 'd4_planche_ep - 13 mm')
    for k in range(4):
        decalage(pal, 'plan_trou_M2_%d' % k, 'd4_planche_ep - 18 mm')
        distance(ex.itemByName('extr_puits_M2_%d' % k), 'd4_planche_ep + 3 mm')
    srv = occ('D4_REF_Servo_DT996').component
    decalage(srv, 'plan_palonnier_rond', 'd4_planche_ep - 12 mm')
    # cannelure : esquisse sur le plan XY, extrusion décalée au départ (retour en arrière dans la timeline)
    fc = srv.features.extrudeFeatures.itemByName('extr_cannelure')
    fc.timelineObject.rollTo(True)
    fc.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByString('d4_planche_ep - 10 mm'))
    distance(fc, '3 mm')
    d.timeline.moveToEnd()
    param('d4_planche_ep', '5 mm', 'V04 | Epaisseur de planche (5 mm) ; la colonne du palonnier et le servo la suivent')
    tl = d.timeline
    print('fonctions KO', [tl.item(i).name for i in range(tl.count) if tl.item(i).healthState != 0])
    g = group()
    for nom in ('D4_IMP_Palonnier', 'D4_REF_Servo_DT996', 'D4_REF_Planche_bois'):
        o = next(x for x in g.childOccurrences if x.component.name == nom)
        for b in o.bRepBodies:
            bb = b.boundingBox
            print(nom, b.name, 'Z', round(bb.minPoint.z * 10, 1), round(bb.maxPoint.z * 10, 1))
