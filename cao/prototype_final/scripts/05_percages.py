from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Perçages de la planche pour le prototype final, ajoutés en place à la planche du groupe V04 :
- 10 trous Ø6,5 pour les vis M6 qui tiennent la coque par-dessous (écrous dans la coque) ;
- 4 trous Ø3,5 pour le berceau de batterie (vis M3 + écrous sous la planche) ;
- 2 trous Ø6,5 pour la chape d'attelage de la remorque et 1 trou Ø8 pour les câbles de ses volets
  (remorque/scripts/01_remorque.py), si elle existe.
Puis relevé de tous les trous de la planche dans plan_percages.json (plan : 06_plan_percages.py).
Réexécutable.
"""
import adsk.core, adsk.fusion, json, os
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/pf_helpers.py'), encoding='utf-8').read(), 'pf_helpers.py', 'exec'))
DOSSIER = str(_FM_ROOT / 'cao/prototype_final')


def run(_context: str):
    d = design()
    bilan = json.load(open(DOSSIER + '/coque_bilan.json', encoding='utf-8'))
    trous = [(x, y, 6.5) for x, y in bilan['fixations_m6']]
    trous += [(x, y, dd) for x, y, dd, _ in json.load(open(DOSSIER + '/trous_composants.json', encoding='utf-8'))['trous']]
    rb = DOSSIER + '/remorque/remorque_bilan.json'
    if os.path.exists(rb):
        rbj = json.load(open(rb, encoding='utf-8'))
        trous += [(x, y, 6.5) for x, y in rbj['patte_vis']]
        if 'trou_cable_rover' in rbj:                    # passage des câbles des volets vers la coque
            trous.append(tuple(rbj['trou_cable_rover']))
    v04 = next(o for o in d.rootComponent.occurrences if o.component.name == 'FM_Direction_V04')
    pc = next(o.component for o in v04.component.occurrences if o.component.name == 'D4_REF_Planche_bois')
    for coll in (pc.features.extrudeFeatures, pc.sketches, pc.constructionPlanes):
        for x in [coll.item(i) for i in range(coll.count)]:
            if x.name in ('extr_perc_prototype_final', 'esq_perc_prototype_final', 'plan_perc_prototype_final'):
                x.deleteMe()
    pb = pc.bRepBodies.itemByName('REF_planche_bois_non_imprimee')
    s = sketch(pc, plane_z(pc, -1, 'perc_prototype_final'), 'perc_prototype_final')
    for x, y, dd in trous:
        circle(s, x, y, dd)
    extrude(pc, profiles(s), 'd4_planche_ep + 2 mm', CUT, 'extr_perc_prototype_final', [pb])
    # relevé de tous les trous (repère de la planche = repère du rover en X, Y)
    out = []
    for f in pb.faces:
        gm = f.geometry
        if gm.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and abs(abs(gm.axis.z) - 1) < 1e-6:
            t = {'x': round(gm.origin.x * 10, 2), 'y': round(gm.origin.y * 10, 2), 'd': round(gm.radius * 20, 2)}
            if t not in out:
                out.append(t)
    sy = ev('d4_servo_y')
    yl, yc = ev('d4_lidar_y'), ev('d4_cam_y')
    plan = {
        'planche': {'x0': -ev('d4_planche_larg') / 2, 'x1': ev('d4_planche_larg') / 2, 'y0': -ev('d4_planche_avant'),
                    'y1': ev('d4_planche_long') - ev('d4_planche_avant'), 'ep': ev('d4_planche_ep')},
        'trous': out,
        'decoupe_servo': {'x0': -10.5, 'x1': 10.5, 'y0': round(sy - 10 - 0.5, 2), 'y1': round(sy - 10 + 40.4 + 0.5, 2)},
        'empreintes': {'profile_lidar': {'x0': -20, 'x1': 20, 'y0': yl - 20, 'y1': yl + 20},
                       'profile_cam': {'x0': -20, 'x1': 20, 'y0': yc - 20, 'y1': yc + 20},
                       'coque': {'x0': -150, 'x1': 150, 'y0': -63, 'y1': 443},
                       'dessous_chape_attelage': {'x0': -22, 'x1': 22, 'y0': 405, 'y1': 440}},
    }
    json.dump(plan, open(DOSSIER + '/plan_percages.json', 'w', encoding='utf-8'), indent=1, ensure_ascii=False)
    print(len(out), 'trous', sorted((t['d'] for t in out)))
