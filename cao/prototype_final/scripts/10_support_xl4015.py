from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Support des deux convertisseurs XL4015 AZDelivery, dans l'assemblage du rover : supprime puis recrée
PF_IMP_Support_XL4015 et les deux modules PF_REF_XL4015_* par xl4015_et_support() de 01_composants.py
(plaque, plots de 3,6 mm, vis M3×8 + écrous M3 dans des poches de 3,6 mm sous la plaque, modules à 5 mm l'un de l'autre), sans refaire les
autres composants. Document du rover « prtotype final fieldmind » actif.
Exporte aussi ../exports/stl/support_xl4015.stl et ../exports/support_xl4015.json (pour le gabarit 1:1
de 11_gabarit_xl4015.py). Entraxe des trous du module (XL_TROUS) mesuré ; place sur la planche : XL_X0, XL_Y0
(à l'avant droit, en miroir de la batterie, pour l'équilibrage).
"""
import adsk.core, adsk.fusion, json, math, os
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/01_composants.py'), encoding='utf-8').read(), '01_composants.py', 'exec'))
SORTIE = str(_FM_ROOT / 'cao/prototype_final/exports')


def run(_context: str):
    nettoyer(['PF_IMP_Support_XL4015', 'PF_REF_XL4015_'])
    b, fix = xl4015_et_support()
    em = design().exportManager
    o = em.createSTLExportOptions(b, SORTIE + '/stl/support_xl4015.stl')
    o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    em.execute(o)
    W, L = XL_PCB
    bx = b.boundingBox
    res = {'pcb': [W, L], 'trous_pcb': XL_TROUS, 'ecart': XL_ECART, 'ep': XL_EP, 'plot_h': XL_PLOT_H, 'pied': XL_PIED,
           'plaque': [-XL_MARGE, -XL_MARGE - XL_OREILLE, 2 * W + XL_ECART + XL_MARGE, L + XL_MARGE + XL_OREILLE],
           'trous_vis': [{'module': m, 'x': m * (W + XL_ECART) + tx, 'y': ty} for m in range(2) for tx, ty in XL_TROUS],
           'fixations': [[x - XL_X0, y - XL_Y0] for x, y in fix],
           'place_sur_la_planche': {'coin_module_Pi': [XL_X0, XL_Y0], 'fixations_planche': fix},
           'fixation_modules': 'vis M3×8 par le dessus + écrou M3 dans la poche sous la plaque',
           'taille_mm': [round((bx.maxPoint.asArray()[i] - bx.minPoint.asArray()[i]) * 10, 1) for i in range(3)],
           'volume_cm3': round(b.volume, 2), 'morceaux': b.lumps.count}
    open(SORTIE + '/support_xl4015.json', 'w', encoding='utf-8').write(texte(json.dumps(res, indent=1, ensure_ascii=False)))
    # trous de la plaque dans la planche du rover (plan de perçage : 05_percages.py puis 06 et 09)
    ft = str(_FM_ROOT / 'cao/prototype_final/trous_composants.json')
    tc = json.load(open(ft, encoding='utf-8'))
    tc['trous'] = [t for t in tc['trous'] if t[3] != 'xl4015'] + [[round(x, 2), round(y, 2), 3.5, 'xl4015'] for x, y in fix]
    json.dump(tc, open(ft, 'w', encoding='utf-8'), indent=1)
    print(texte(json.dumps(res, ensure_ascii=False)))
