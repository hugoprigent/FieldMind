from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Jantes avant : flasque extérieur démontable et arête centrale plus propre, modification en place.

- Pneu en TPU 90A (le 70A ne s'imprime pas) : trop raide pour passer par-dessus un rebord fixe
  (il faudrait étirer son diamètre intérieur de 119 à 124 mm, +4 %). Le rebord extérieur devient un
  flasque vissé : on enfile le pneu à plat sur la portée Ø120, puis on visse le flasque.
  - la jante est raccourcie de 2 mm côté extérieur (le rebord fixe disparaît) ;
  - 6 bossages Ø8 au bout des rayons portent des inserts M3 à chaud (trous Ø4,0 × 6,5) ;
  - flasque Ø124 × Ø96 × 3 mm, 6 passages Ø3,4, vis M3×8 ;
  - chanfrein de 1 mm à l'entrée de la portée du pneu.
- Arête entre les deux 608ZZ : alésage Ø17 → Ø19,5. Elle n'appuie toujours que sur les bagues
  extérieures (épaulement de 608 : Ø20 maxi), et son dessous, imprimé en surplomb, passe de 2,6 à
  1,3 mm de large : moins de bavure qui précontraindrait les roulements.
Réexécutable : supprime d'abord ce qu'un passage précédent a ajouté.
"""
import adsk.core, adsk.fusion, math
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
GROUP = 'FM_Direction_V04'

ARETE_D = 19.5
FLASQUE_EP, FLASQUE_DI, REBORD_D = 3.0, 96.0, 124.0
R_VIS, N_VIS, BOSSAGE_D, BOSSAGE_L = 53.5, 6, 8.0, 9.0
INSERT_D, INSERT_P, PASSAGE_M3 = 4.0, 6.5, 3.4
CHANF_ENTREE = 1.0
NOMS = ('coupe_flasque', 'bossages_inserts', 'trous_inserts', 'flasque', 'trous_flasque')


def plan_x(c, x, name):
    i = c.constructionPlanes.createInput()
    i.setByOffset(c.yZConstructionPlane, V(x))
    p = c.constructionPlanes.add(i)
    p.name = 'plan_' + name
    p.isLightBulbOn = False
    return p


def cercles_x(c, x, name, d, pts=None):
    """Esquisse dans le plan x : un cercle centré sur l'axe, ou un cercle par point (y, z)."""
    s = sketch(c, plan_x(c, x, name), name)
    for (y, z) in (pts or [(0.0, 0.0)]):
        s.sketchCurves.sketchCircles.addByCenterRadius(s.modelToSketchSpace(P(x, y, z)), d / 20)
    return s


def run(_context: str):
    d = design()
    g = group()
    w = ev('d4_roue_larg')
    xb = 1.5 + w - 2.0                     # nouvelle face extérieure de la jante (ancien début du rebord)
    vis = [(R_VIS * math.sin(2 * math.pi * k / N_VIS), R_VIS * math.cos(2 * math.pi * k / N_VIS)) for k in range(N_VIS)]
    for s_, tag in ((1, 'D'), (-1, 'G')):
        c = next(o.component for o in g.childOccurrences if o.component.name == 'D4_IMP_Roue_' + tag)
        for coll in (c.features.chamferFeatures, c.features.extrudeFeatures, c.sketches, c.constructionPlanes):
            for x in [coll.item(i) for i in range(coll.count)]:
                if x.name == 'chanfrein_entree_pneu' or any(x.name in ('extr_' + n, 'esq_' + n, 'plan_' + n) for n in NOMS):
                    x.deleteMe()
        j = c.bRepBodies.itemByName('jante_' + tag)
        # arête centrale : Ø19,5
        sk = c.sketches.itemByName('esq_passage_moyeu')
        sk.sketchDimensions.item(0).parameter.expression = '%r mm' % ARETE_D
        # rebord extérieur fixe supprimé : la jante s'arrête à xb
        s = cercles_x(c, s_ * xb, 'coupe_flasque', 150.0)
        extrude(c, profiles(s), 3.0, CUT, 'extr_coupe_flasque', [j], down=s_ < 0)
        # bossages au bout des rayons, inserts M3
        s = cercles_x(c, s_ * (xb - BOSSAGE_L), 'bossages_inserts', BOSSAGE_D, vis)
        extrude(c, profiles(s), BOSSAGE_L, JOIN, 'extr_bossages_inserts', [j], down=s_ < 0)
        s = cercles_x(c, s_ * xb, 'trous_inserts', INSERT_D, vis)
        extrude(c, profiles(s), INSERT_P, CUT, 'extr_trous_inserts', [j], down=s_ > 0)
        # chanfrein d'entrée de la portée du pneu
        bords = adsk.core.ObjectCollection.create()
        for e in j.edges:
            gm = e.geometry
            if gm.curveType == adsk.core.Curve3DTypes.Circle3DCurveType:
                ci = adsk.core.Circle3D.cast(gm)
                if abs(ci.radius * 10 - ev('d4_roue_av_d - 2 * d4_pneu_ep') / 2) < 0.01 and abs(ci.center.x * 10 - s_ * xb) < 0.01:
                    bords.add(e)
        ci = c.features.chamferFeatures.createInput2()
        ci.chamferEdgeSets.addEqualDistanceChamferEdgeSet(bords, V(CHANF_ENTREE), False)
        c.features.chamferFeatures.add(ci).name = 'chanfrein_entree_pneu'
        # flasque vissé
        s = cercles_x(c, s_ * xb, 'flasque', REBORD_D)
        s.sketchCurves.sketchCircles.addByCenterRadius(s.modelToSketchSpace(P(s_ * xb, 0, 0)), FLASQUE_DI / 20)
        fl = body(extrude(c, profiles(s, 'ring'), FLASQUE_EP, NEW, 'extr_flasque', down=s_ < 0), 'flasque_pneu_' + tag)
        s = cercles_x(c, s_ * xb, 'trous_flasque', PASSAGE_M3, vis)
        extrude(c, profiles(s), FLASQUE_EP, CUT, 'extr_trous_flasque', [fl], down=s_ < 0)
        bb = j.boundingBox
        fb = fl.boundingBox
        print(tag, 'jante x', round(bb.minPoint.x * 10, 2), round(bb.maxPoint.x * 10, 2), 'vol', round(j.volume, 2),
              '| flasque x', round(fb.minPoint.x * 10, 2), round(fb.maxPoint.x * 10, 2), 'vol', round(fl.volume, 2), '| chanfrein', bords.count)
    tl = d.timeline
    print('fonctions KO', [tl.item(i).name for i in range(tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup])
