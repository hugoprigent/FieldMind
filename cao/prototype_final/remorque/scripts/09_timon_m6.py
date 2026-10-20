from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Timon fixé par 4 vis M6×25 à tête bombée (par le dessus, écrou M6 noyé sous le timon) au lieu de
6 vis M3×30 dans les rebords avant : modification en place dans le document de la remorque, sans le
reconstruire (même résultat que 01_remorque.py). Retire les anciens trous M3 de la planche, du timon et
des rebords avant, perce les 4 trous Ø6,5 et met à jour remorque_bilan.json (plan : 04_plan_planche.py).
"""
import adsk.core, adsk.fusion, json
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/remorque/scripts/rm_helpers.py'), encoding='utf-8').read(), 'rm_helpers.py', 'exec'))
BILAN = str(_FM_ROOT / 'cao/prototype_final/remorque/remorque_bilan.json')
ANCIENS = ('+20_649', '+50_649', '+78_649', '-20_649', '-50_649', '-78_649')


def supprimer(c, test):
    for coll in (c.features.extrudeFeatures, c.sketches, c.constructionPlanes):
        for x in reversed([coll.item(i) for i in range(coll.count)]):
            if x.isValid and test(x.name):
                x.deleteMe()


def run(_context: str):
    a = app()
    doc = next(d_ for d_ in a.documents if d_.name.startswith(DOC_REMORQUE))
    doc.activate()
    d = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    comp = {c.name: c for c in d.allComponents}
    cp, ct, cr = comp['RM_REF_Planche_bois'], comp['RM_IMP_Timon'], comp['RM_IMP_Rebords']
    supprimer(cp, lambda n: 'planche_vis_' in n and (n.endswith(ANCIENS) or n.endswith('_665')))
    supprimer(cr, lambda n: n.endswith(ANCIENS))
    supprimer(ct, lambda n: 'timon_vis_' in n or 'timon_lamage_' in n or 'timon_ecrou_' in n)
    pb = cp.bRepBodies.item(0)
    tb = ct.bRepBodies.itemByName('timon')
    vis = [v for v in vis_planche() if v[4] == 'timon']
    for x, y, L, nom, role in vis:
        trou_z(cp, 'planche_vis_%+d_%d' % (x, y), x, y, 6.5, ZP - 1, EP + 2, [pb])
        trou_z(ct, 'timon_vis_%+d' % x, x, y, 6.5, TIMON_Z0 - 1, TIMON_EP + TIMON_CALE + 2, [tb])
        hexa_z(ct, 'timon_ecrou_%+d' % x, x, y, M6_HEX, TIMON_Z0 - 0.01, M6_ECROU_H + 0.2, [tb])
    b = json.load(open(BILAN, encoding='utf-8'))
    b['vis_planche'] = [list(v) for v in vis_planche()]
    open(BILAN, 'w', encoding='utf-8').write(json.dumps(b, indent=1, ensure_ascii=False))
    tl = d.timeline
    print(json.dumps({'vis_timon': vis, 'timon_morceaux': tb.lumps.count, 'timon_cm3': round(tb.volume, 1),
                      'KO': [tl.item(i).name for i in range(max(0, tl.count - 40), tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup]}))
