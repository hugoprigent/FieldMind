from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Batterie à l'avant, à gauche du servo de direction (équilibrage avant/arrière et gauche/droite), dans
le document du rover, sans refaire les autres composants : supprime puis recrée PF_REF_Batterie et
PF_IMP_Berceau_batterie (berceau sans pattes, fond de 5 mm à écrous M3 noyés, vis M3×10 par-dessous la
planche), met à jour les trous du berceau dans trous_composants.json et exporte le STL du berceau.
Ensuite : 05_percages.py (trous de la planche), 13_centre_de_gravite.py (bilan).
"""
import adsk.core, adsk.fusion, json, math, os
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/01_composants.py'), encoding='utf-8').read(), '01_composants.py', 'exec'))
DOSSIER = str(_FM_ROOT / 'cao/prototype_final')


def run(_context: str):
    nettoyer(['PF_REF_Batterie', 'PF_IMP_Berceau'])
    _, c = piece('PF_REF_Batterie', 'COTES_MESUREES_82x68x51')
    boite(c, 'batterie', BATT_X - BATT[0] / 2, BATT_Y - BATT[1] / 2, BATT_X + BATT[0] / 2, BATT_Y + BATT[1] / 2, ZB + BERCEAU_FOND, BATT[2], NOIR)
    _, c = piece('PF_IMP_Berceau_batterie', 'IMPRIME_PETG')
    trous = berceau(c)
    ft = DOSSIER + '/trous_composants.json'
    tc = json.load(open(ft, encoding='utf-8'))
    tc['trous'] = [t for t in tc['trous'] if t[3] != 'berceau'] + [list(t) for t in trous]
    json.dump(tc, open(ft, 'w', encoding='utf-8'), indent=1)
    b = c.bRepBodies.itemByName('berceau_batterie')
    em = design().exportManager
    o = em.createSTLExportOptions(b, DOSSIER + '/exports/stl/berceau_batterie.stl')
    o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    em.execute(o)
    bx = b.boundingBox
    print(json.dumps({'berceau_mm': [[round(v * 10, 1) for v in bx.minPoint.asArray()], [round(v * 10, 1) for v in bx.maxPoint.asArray()]],
                      'morceaux': b.lumps.count, 'trous': trous}))
