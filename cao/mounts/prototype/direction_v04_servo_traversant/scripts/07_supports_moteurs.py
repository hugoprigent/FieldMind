from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Supports moteurs arrière (pièce Onshape d'Hugo, sources/support_moteur_onshape.step) et planche 30 x 50 cm.

Pièce importée : bloc 40 x 60 x 108 mm, face d'appui (40 x 108) plaquée sous la planche, deux pattes
de 10 mm avec trous Ø9 (M8 traversantes, entraxe 90 mm), alésage moteur Ø46 avec épaulement Ø52
de 2,5 mm côté roue, axe moteur à 30 mm sous la face d'appui.

Repère de la pièce importée : X = axe moteur (X = 0 côté épaulement Ø52, donc côté roue),
Y = vertical (Y = -30 face d'appui), Z = longueur (pattes à Z = ±45).
Composant D4_REF_Support_moteur_arriere : la pièce est tournée pour le côté droit, origine sur l'axe moteur
au droit de la face côté roue. Côté gauche : même composant tourné de 180° autour de la verticale.
Placement : face côté roue au bord de planche (±d4_planche_larg/2), axe à d4_empattement derrière
les pivots, à d4_roue_ar_d/2 du sol.
"""
import adsk.core, adsk.fusion, math
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
GROUP = 'FM_Direction_V04'
SRC = str(_FM_ROOT / 'cao/mounts/prototype/direction_v04_servo_traversant/sources/support_moteur_onshape.step')


def run(_context: str):
    d = design()
    root = d.rootComponent
    # nettoyage de l'import d'analyse
    for o in list(root.occurrences):
        if o.component.name == 'SUPPORT_MOTEUR_ANALYSE':
            o.deleteMe()
    param('d4_planche_long', '500 mm', 'V04 | Longueur de planche (50 cm)')
    param('d4_empattement', '380 mm', 'V04 | Pivots avant -> axe des moteurs arriere (support a 6 mm du bord arriere)')
    g = group()
    gc = g.component
    for j in list(gc.joints):
        if j.name.startswith('D4_place_support_moteur'):
            j.deleteMe()
    for oo in list(gc.occurrences):
        if oo.component.name == 'D4_REF_Support_moteur_arriere':
            oo.deleteMe()
    # import temporaire, copie de la géométrie orientée côté droit dans un corps de base sans lien
    tmp = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    tmp.component.name = 'TMP_import_support'
    im = adsk.core.Application.get().importManager
    im.importToTarget(im.createSTEPImportOptions(SRC), tmp.component)
    src = ([x for x in tmp.component.bRepBodies] + [x for oo in tmp.component.allOccurrences for x in oo.bRepBodies])[0]
    tbm = adsk.fusion.TemporaryBRepManager.get()
    tb = tbm.copy(src)
    # rotation côté droit : X pièce -> -X rover, Y pièce -> -Z rover, Z pièce -> -Y rover (rotation propre)
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(adsk.core.Point3D.create(0, 0, 0),
                              adsk.core.Vector3D.create(-1, 0, 0),
                              adsk.core.Vector3D.create(0, 0, -1),
                              adsk.core.Vector3D.create(0, -1, 0))
    tbm.transform(tb, m)
    tmp.deleteMe()
    o, c = part('D4_REF_Support_moteur_arriere', statut='PIECE_HUGO_ONSHAPE')
    bf = c.features.baseFeatures.add()
    bf.name = 'support_moteur_onshape_fige'
    bf.startEdit()
    b = c.bRepBodies.add(tb, bf)
    bf.finishEdit()
    b = c.bRepBodies.item(0)
    b.name = 'support_moteur_onshape'
    paint(b, 'FM_PERFBOARD_bleu')
    o_g = g.component.occurrences.addExistingComponent(c, adsk.core.Matrix3D.create())
    for occ_, tag, sx, ang in ((o, 'D', 'd4_planche_larg / 2', None), (o_g, 'G', '-d4_planche_larg / 2', '180 deg')):
        occ_.isGroundToParent = False
        ji = gc.joints.createInput(origin_geo(occ_), adsk.fusion.JointGeometry.createByPoint(gc.originConstructionPoint))
        ji.setAsRigidJointMotion()
        j = gc.joints.add(ji)
        j.name = 'D4_place_support_moteur_' + tag
        j.offsetX.expression = sx
        j.offsetY.expression = 'd4_empattement'
        j.offset.expression = 'd4_roue_ar_d / 2'
        if ang:
            j.angle.expression = ang
    for oo in g.childOccurrences:
        if oo.component.name == 'D4_REF_Support_moteur_arriere':
            bb = oo.boundingBox
            print(oo.name, [round(v * 10, 1) for v in bb.minPoint.asArray()], [round(v * 10, 1) for v in bb.maxPoint.asArray()])
    pl = next(oo for oo in g.childOccurrences if oo.component.name == 'D4_REF_Planche_bois').boundingBox
    print('planche', [round(v * 10, 1) for v in pl.minPoint.asArray()], [round(v * 10, 1) for v in pl.maxPoint.asArray()])
