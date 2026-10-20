from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Planche de 30 cm : l'écartement des pivots suit la largeur de planche (pivots à
d4_debord_pivot au-delà de chaque bord, pour que les pneus, qui dépassent au-dessus du
dessous de planche, restent dehors en braquant). Contrôle du braquage avec roues et planche."""
import adsk.core, adsk.fusion, math
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
GROUP = 'FM_Direction_V04'


def run(_context: str):
    d = design()
    param('d4_planche_larg', '300 mm', 'V04 | Largeur de planche (30 cm, donnee Hugo)')
    param('d4_debord_pivot', '5 mm', 'V04 | Axe de pivot au-dela du bord de planche ; >= 3 mm sinon les pneus touchent la planche en braquant')
    param('d4_E', 'd4_planche_larg + 2 * d4_debord_pivot', 'V04 | Ecartement des axes de pivot (derive de la largeur de planche)')
    g = group()
    sv = next(j for j in g.component.joints if j.name == 'D4_SERVO_DIRECTION')
    bodies = [b for o in g.childOccurrences for b in o.bRepBodies]
    planche = [b for b in bodies if b.name.startswith('REF_planche')][0]
    pire = None
    for a in range(-31, 32, 2):
        sv.jointMotion.rotationValue = math.radians(a)
        coll = [x for x in interferences(bodies) if x[2] > 0.001 and not (x[0].startswith('pneu_TPU') and x[1].startswith('jante')) and not (x[1].startswith('pneu_TPU') and x[0].startswith('jante'))]
        if coll:
            print(a, coll)
        # distance mini pneu / planche
        for t in ('D', 'G'):
            pn = next(b for b in bodies if b.name == 'pneu_TPU_' + t)
            r = adsk.core.Application.get().measureManager.measureMinimumDistance(pn, planche)
            dist = r.value * 10
            if pire is None or dist < pire[0]:
                pire = (round(dist, 2), a, t)
    sv.jointMotion.rotationValue = 0
    tl = d.timeline
    print('E', ev('d4_E'), 'biellette', round(ev('d4_biellette_L'), 2), 'jeu mini pneu/planche', pire,
          'liaisons KO', [j.name for j in g.component.joints if j.healthState != 0],
          'fonctions KO', [tl.item(i).name for i in range(tl.count) if tl.item(i).healthState != 0])
