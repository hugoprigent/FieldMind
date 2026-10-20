from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Planche de 10 mm (deux 5 mm collées), en place dans le document du rover, sans rien reconstruire :
- d4_planche_ep = 10 mm : la planche, les mâts (liaisons par SOL), le servo et la colonne du palonnier
  suivent le paramètre ; le dessous de la planche reste à 130 mm du sol, donc tout ce qui est dessous
  (supports de pivot, fusées, roues, supports moteurs, moteurs, chape d'attelage) ne bouge pas ;
- les composants du prototype final posés sur la planche (coque, batterie, électronique...) montent de
  5 mm par une fonction « Déplacer » nommée planche_10mm (une seule fois par composant).
pf_helpers.ZB passe à 140 pour les scripts suivants. Réexécutable.
"""
import adsk.core, adsk.fusion, json
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/pf_helpers.py'), encoding='utf-8').read(), 'pf_helpers.py', 'exec'))
EP = 10.0
DESSOUS = ('PF_REF_Moteur_arriere_', 'PF_REF_Roue_arriere_', 'PF_IMP_Attelage')


def run(_context: str):
    d = design()
    p = d.userParameters.itemByName('d4_planche_ep')
    avant = p.value * 10
    dz = EP - avant
    p.expression = '%g mm' % EP
    out = {'planche_mm': [avant, EP], 'deplaces': [], 'deja': []}
    for o in group().childOccurrences:
        c = o.component
        if c.name.startswith(DESSOUS):
            continue
        if c.features.moveFeatures.itemByName('planche_10mm'):
            out['deja'].append(c.name)
            continue
        corps = [b for b in c.bRepBodies]
        if not corps or abs(dz) < 1e-6:
            continue
        col = adsk.core.ObjectCollection.create()
        for b in corps:
            col.add(b)
        m = adsk.core.Matrix3D.create()
        m.translation = adsk.core.Vector3D.create(0, 0, dz / 10)
        f = c.features.moveFeatures.add(c.features.moveFeatures.createInput(col, m))
        f.name = 'planche_10mm'
        out['deplaces'].append(c.name)
    tl = d.timeline
    out['KO'] = [tl.item(i).name for i in range(max(0, tl.count - 80), tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup]
    print(json.dumps(out))
