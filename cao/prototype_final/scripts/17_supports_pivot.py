from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Supports de pivot avant renforcés, en place sur les pièces V04 (D4_IMP_Support_pivot_D/G) :
- plaque prolongée vers l'avant (y -30 au lieu de -22) ;
- bloc épaissi à 20 mm vers l'intérieur, là où la fusée et les biellettes ne passent pas
  (de 40 à 95 mm de l'axe du pivot, y -30 à 14) ;
- 4 vis H M8 au lieu de 2 : les 2 d'origine (31 et 60 mm de l'axe, y -4) et 2 nouvelles à 85 mm de
  l'axe (y -19 et 5), soit x ±70 sur la planche. Têtes prisonnières dans des empreintes dessous,
  qui gardent 5,5 mm de plastique sous la tête : mêmes vis (M8×20 avec la planche de 5 mm) et on
  serre toujours par l'écrou sur le dessus de la planche.
Repère des supports : origine sur l'axe du pivot, z = 0 sous la planche, côté intérieur vers -s·x.
Réexécutable. Ajoute les 2 nouveaux trous par support à trous_composants.json (05_percages.py).
"""
import adsk.core, adsk.fusion, json
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/pf_helpers.py'), encoding='utf-8').read(), 'pf_helpers.py', 'exec'))
DOSSIER = str(_FM_ROOT / 'cao/prototype_final')
X_PIVOT = 155.0
AVANT = (-72.0, -30.0, -22.0)          # prolongement avant de la plaque de 11 mm : x de -72 à 0, y de -30 à -22
BLOC = (-95.0, -40.0, -30.0, 14.0, -20.0)   # bloc épaissi : x, y, dessous à z -20
VIS = [(-31.0, -4.0), (-60.0, -4.0), (-85.0, -19.0), (-85.0, 5.0)]
PASSAGE, TETE_S, SOUS_TETE = 8.4, 13.3, 5.5


def renforcer_pivot(c, b, s):
    """Ajoute le prolongement, le bloc épaissi et les 2 vis M8 au corps b (repère du support de pivot,
    s = +1 à droite, -1 à gauche)."""
    B = [b]
    x0, x1 = sorted((s * AVANT[0], 0.0))
    zbox(c, 'renfort_avant', x0, AVANT[1], x1, AVANT[2] + 0.5, -11, 11, JOIN, B)
    bx0, bx1 = sorted((s * BLOC[0], s * BLOC[1]))
    zbox(c, 'renfort_bloc', bx0, BLOC[2], bx1, BLOC[3], BLOC[4], -BLOC[4], JOIN, B)
    for k, (x, y) in enumerate(VIS):
        if k == 0:
            continue                      # vis d'origine à 31 mm : hors du bloc, son trou et son empreinte existent déjà
        zcyl(c, 'renfort_trou_%d' % k, s * x, y, PASSAGE, BLOC[4] - 1, -BLOC[4] + 2, CUT, B)
        sk = sketch(c, plane_z(c, BLOC[4] - 1, 'renfort_tete_%d' % k), 'renfort_tete_%d' % k)
        poly(sk, hexagon(sk, s * x, y, TETE_S))
        extrude(c, profiles(sk), -BLOC[4] + 1 - SOUS_TETE, CUT, 'extr_renfort_tete_%d' % k, B)
    return b


def run(_context: str):
    d = design()
    out = {}
    for tag, s in (('D', 1), ('G', -1)):
        c = next(x for x in d.allComponents if x.name == 'D4_IMP_Support_pivot_' + tag)
        for coll in (c.features.extrudeFeatures, c.sketches, c.constructionPlanes):
            for x in reversed([coll.item(i) for i in range(coll.count)]):
                if x.isValid and 'renfort' in x.name:
                    x.deleteMe()
        renforcer_pivot(c, c.bRepBodies.itemByName('support_pivot_' + tag), s)
        B = [c.bRepBodies.itemByName('support_pivot_' + tag)]
        b = B[0]
        bb = b.boundingBox
        out[tag] = {'volume_cm3': round(b.volume, 1), 'morceaux': b.lumps.count,
                    'boite_locale': [round(v * 10, 1) for v in bb.minPoint.asArray() + bb.maxPoint.asArray()]}
    tc = json.load(open(DOSSIER + '/trous_composants.json', encoding='utf-8'))
    tc['trous'] = [t for t in tc['trous'] if t[3] != 'support_pivot']
    for s in (1, -1):
        for x, y in VIS[2:]:
            tc['trous'].append([round(s * (X_PIVOT + x), 2), y, 8.5, 'support_pivot'])
    json.dump(tc, open(DOSSIER + '/trous_composants.json', 'w', encoding='utf-8'), indent=1)
    tl = d.timeline
    out['fonctions_KO'] = [tl.item(i).name for i in range(max(0, tl.count - 60), tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup]
    print(json.dumps(out))
