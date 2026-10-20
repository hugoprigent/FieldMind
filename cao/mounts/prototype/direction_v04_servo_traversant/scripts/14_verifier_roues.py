from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Contrôle des roues avant : jeux entre ce qui tourne avec la roue (jante, pneu, flasque) et ce qui
reste fixe (fusée, vis d'axe, rondelles serrées sur les bagues intérieures, supports, planche...),
direction au neutre et à ±31°. Relevé de l'empilage sur l'axe de roue.
Écrit verification_roues.json à côté des scripts.
"""
import adsk.core, adsk.fusion, json, math
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
GROUP = 'FM_Direction_V04'
DOSSIER = str(_FM_ROOT / 'cao/mounts/prototype/direction_v04_servo_traversant')
TOURNE = ('jante_', 'pneu_TPU_', 'flasque_')          # corps qui tournent avec la roue
CONTACT_VOULU = ('REF_608ZZ_roue',)                    # bagues extérieures emmanchées dans la jante


def run(_context: str):
    d = design()
    gp = group()
    servo = next(j for j in gp.component.joints if j.name == 'D4_SERVO_DIRECTION')
    mm = app().measureManager
    res = {'jeux_mini_mm': {}, 'empilage_mm': {}}
    for a in (0, -31, 31):
        servo.jointMotion.rotationValue = math.radians(a)
        for tag in ('D', 'G'):
            roue = next(o for o in gp.childOccurrences if o.component.name == 'D4_IMP_Roue_' + tag)
            tournants = [b for b in roue.bRepBodies if b.name.startswith(TOURNE)]
            fixes = [b for b in roue.bRepBodies if not b.name.startswith(TOURNE + CONTACT_VOULU)]
            fixes += [b for o in gp.childOccurrences if not o.component.name.startswith('D4_IMP_Roue_') for b in o.bRepBodies
                      if not o.component.name.startswith(('D4_REF_Lidar_C1',))]
            for t in tournants:
                pire = None
                for f in fixes:
                    dist = mm.measureMinimumDistance(t, f).value * 10
                    if pire is None or dist < pire[0]:
                        pire = (round(dist, 2), '%s/%s' % (f.assemblyContext.component.name if f.assemblyContext else '', f.name))
                cle = '%s_servo_%+d' % (t.name, a)
                res['jeux_mini_mm'][cle] = {'distance': pire[0], 'avec': pire[1]}
    servo.jointMotion.rotationValue = 0
    # empilage le long de l'axe (repère de la roue droite : x depuis la face de la jambe de fusée)
    c = next(o.component for o in gp.childOccurrences if o.component.name == 'D4_IMP_Roue_D')
    for b in c.bRepBodies:
        bb = b.boundingBox
        res['empilage_mm'][b.name] = [round(bb.minPoint.x * 10, 2), round(bb.maxPoint.x * 10, 2)]
    open(DOSSIER + '/verification_roues.json', 'w', encoding='utf-8').write(json.dumps(res, ensure_ascii=False, indent=1))
    print(json.dumps(res, ensure_ascii=False))
