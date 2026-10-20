from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Fusées avant renforcées, en place dans le document du rover (D4_IMP_Fusee_D/G, congés faits à la main
conservés) : la fusée est en L (tube du pivot, pont de 8 mm, jambe qui porte l'axe de roue) et le coin
du L plie. On ajoute un gousset triangulaire sous le pont, sur toute la largeur (26 mm), contre la jambe :
- de 11 à 21 mm de l'axe du pivot : la douille de 13 passe toujours sur la tête de la vis-pivot ;
- jusqu'à 44 mm sous la planche : la poche de l'écrou de roue (côté intérieur de la jambe) reste libre.
La fusée tourne avec la direction : elle reste dans le rover (une pièce d'un document inséré ne suivrait
pas les liaisons). Réexécutable. Exporte les STL (face du bras sur le plateau).
Repère des fusées : origine sur l'axe du pivot, z = 0 sous la planche, jambe du côté +s·x.
"""
import adsk.core, adsk.fusion, json
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/04_exporter.py'), encoding='utf-8').read(), '04_exporter.py', 'exec'))
DOSSIER = str(_FM_ROOT / 'cao/prototype_final')
GOUSSET = [(11.0, -28.0), (21.0, -28.0), (21.0, -44.0)]   # (distance à l'axe, z) ; pont de z -28 à -20, jambe à 21
LARGEUR = 26.0


def gousset(c, nom, pts_xz, op, bodies):
    s = sketch(c, plane_y(c, 0.0, nom), nom)
    poly3(s, [(x, 0.0, z) for x, z in pts_xz])
    i = c.features.extrudeFeatures.createInput(profiles(s), op)
    i.setSymmetricExtent(V(LARGEUR), True)
    i.participantBodies = list(bodies)
    f = c.features.extrudeFeatures.add(i)
    f.name = 'extr_' + nom
    return f


def run(_context: str):
    d = design()
    out = {}
    for tag, s in (('D', 1), ('G', -1)):
        c = next(x for x in d.allComponents if x.name == 'D4_IMP_Fusee_' + tag)
        for coll in (c.features.extrudeFeatures, c.sketches, c.constructionPlanes):
            for x in reversed([coll.item(i) for i in range(coll.count)]):
                if x.isValid and 'gousset_fusee' in x.name:
                    x.deleteMe()
        b = c.bRepBodies.itemByName('fusee_' + tag)
        v0 = b.volume
        gousset(c, 'gousset_fusee', [(s * x, z) for x, z in GOUSSET], JOIN, [b])
        out[tag] = {'volume_cm3': [round(v0, 1), round(b.volume, 1)], 'morceaux': b.lumps.count,
                    'stl_mm': stl(b, DOSSIER + '/exports/stl/fusee_renforcee_%s.stl' % tag, ('x', 180))}
    tl = d.timeline
    out['fonctions_KO'] = [tl.item(i).name for i in range(max(0, tl.count - 40), tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup]
    print(json.dumps(out))
