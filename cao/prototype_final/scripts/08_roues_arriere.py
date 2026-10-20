from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Roues arrière (et moteurs) seules, sans refaire les autres composants : supprime
PF_REF_Moteur_arriere_* et PF_REF_Roue_arriere_*, puis les recrée par moteurs_et_roues_arriere()
de 01_composants.py. Document du rover « prtotype final fieldmind » actif."""
import adsk.core, adsk.fusion, math, json
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/01_composants.py'), encoding='utf-8').read(), '01_composants.py', 'exec'))


def run(_context: str):
    nettoyer(['PF_REF_Moteur_arriere_', 'PF_REF_Roue_arriere_'])
    moteurs_et_roues_arriere()
    out = {}
    for o in group().childOccurrences:
        if o.component.name.startswith(('PF_REF_Roue_arriere_', 'PF_REF_Moteur_arriere_')):
            for b in o.bRepBodies:
                bx = b.boundingBox
                out[b.name] = [[round(v * 10, 1) for v in bx.minPoint.asArray()], [round(v * 10, 1) for v in bx.maxPoint.asArray()], b.lumps.count]
    tl = design().timeline
    out['fonctions_KO'] = [tl.item(i).name for i in range(max(0, tl.count - 80), tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup]
    print(json.dumps(out, ensure_ascii=False))
