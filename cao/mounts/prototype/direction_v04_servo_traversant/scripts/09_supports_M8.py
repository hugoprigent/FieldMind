from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Supports de pivot fixés en M8 au lieu de 4 x M4, modification en place (sans reconstruire,
pour garder les modifications faites à la main dans le document).

Par support : 2 vis H M8 sur l'axe de la plaque, à 31 et 60 mm de l'axe du pivot. La tête est
prisonnière dans une empreinte hexagonale sous le support, et affleure : rien ne dépasse du côté
de la fusée, qui tourne juste en dessous. Écrou et rondelle sur le dessus de la planche.
Réexécutable : supprime d'abord les trous M4 et les trous M8 d'un passage précédent.
"""
import adsk.core, adsk.fusion
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
GROUP = 'FM_Direction_V04'

TROUS_M8 = [(-31.0, -4.0), (-60.0, -4.0)]   # repère du support droit (s = +1), axe du pivot en (0, 0)
PASSAGE_M8 = 8.4                            # passage libre (le trou imprimé sort un peu plus petit)
TETE_M8_S, TETE_M8_H = 13.3, 5.5            # tête H M8 : 13 sur plats, 5,3 de haut


def supprimer(c, prefixes):
    for coll in (c.features.extrudeFeatures, c.sketches, c.constructionPlanes):
        for x in [coll.item(i) for i in range(coll.count)]:
            nom = x.name
            for p in prefixes:
                if nom.startswith(p):
                    x.deleteMe()
                    break


def run(_context: str):
    d = design()
    for tag, s in (('D', +1), ('G', -1)):
        c = next(x for x in d.allComponents if x.name == 'D4_IMP_Support_pivot_' + tag)
        avant = len(c.features.extrudeFeatures)
        for k in range(4):
            supprimer(c, ('extr_trou_M4_%d' % k, 'esq_trou_M4_%d' % k, 'plan_trou_M4_%d' % k))
        for k in range(2):
            supprimer(c, ('extr_trou_M8_%d' % k, 'esq_trou_M8_%d' % k, 'plan_trou_M8_%d' % k,
                          'empreinte_tete_M8_%d' % k, 'esq_tete_M8_%d' % k, 'plan_tete_M8_%d' % k))
        B = [c.bRepBodies.itemByName('support_pivot_' + tag)]
        for k, (x, y) in enumerate(TROUS_M8):
            zcyl(c, 'trou_M8_%d' % k, s * x, y, PASSAGE_M8, -12, 13, CUT, B)
            sk = sketch(c, plane_z(c, -11, 'tete_M8_%d' % k), 'tete_M8_%d' % k)
            poly(sk, hexagon(sk, s * x, y, TETE_M8_S))
            extrude(c, profiles(sk), TETE_M8_H, CUT, 'empreinte_tete_M8_%d' % k, B)
        b = c.bRepBodies.itemByName('support_pivot_' + tag)
        bb = b.boundingBox
        print(tag, 'extrusions', avant, '->', len(c.features.extrudeFeatures), 'faces', b.faces.count,
              'volume cm3', round(b.volume, 2),
              'boite', [round(v * 10, 1) for v in (bb.minPoint.x, bb.minPoint.y, bb.minPoint.z, bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z)])
    tl = d.timeline
    print('fonctions KO', [tl.item(i).name for i in range(tl.count) if tl.item(i).healthState != 0])
