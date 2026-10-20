from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Apparences du prototype final : bleu et blanc.
- Toutes les pièces déjà imprimées (direction V04, roues, supports du lidar et des caméras) l'ont été
  en blanc : leurs corps imprimés passent en blanc, sauf les pneus TPU (gris foncé).
- Coque : flancs blancs, bande centrale bleue (PLA) ; berceau blanc.
- Attelage : chape blanche (vis M10 et écrou en métal).
"""
import adsk.core, adsk.fusion
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/pf_helpers.py'), encoding='utf-8').read(), 'pf_helpers.py', 'exec'))


def run(_context: str):
    d = design()
    blanc = d.appearances.itemByName(BLANC)
    bleu = d.appearances.itemByName('FM_PLA_bleu') or d.appearances.itemByName(BLEU)
    tpu = d.appearances.itemByName(NOIR)
    n = {'blanc': 0, 'tpu': 0, 'bleu': 0}
    for g in d.rootComponent.occurrences:
        if g.component.name not in ('FM_Direction_V04', 'FM_Prototype_Final'):
            continue
        for o in g.component.occurrences:
            nom = o.component.name
            if not nom.startswith(('D4_IMP_', 'PF_IMP_')):
                continue
            for b in o.component.bRepBodies:
                if b.name.startswith('REF_'):
                    continue
                if b.name.startswith('pneu'):
                    b.appearance = tpu
                    n['tpu'] += 1
                elif b.name.startswith(('coque_C', 'bouton_')):
                    b.appearance = bleu
                    n['bleu'] += 1
                else:
                    b.appearance = blanc
                    n['blanc'] += 1
    print(n, 'bleu =', bleu.name)
