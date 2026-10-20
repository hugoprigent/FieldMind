from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""V04 : balayage du servo (angles des fusées + interférences), essais de paramètres,
hauteurs (planche, point le plus bas). Écrit mouvement_verifie.json et remet les valeurs initiales."""
import adsk.core, adsk.fusion, json, math
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
GROUP = 'FM_Direction_V04'
OUT = str(_FM_ROOT / 'cao/mounts/prototype/direction_v04_servo_traversant/mouvement_verifie.json')


def angles():
    r = []
    for t in ('D', 'G'):
        o = next(x for x in group().childOccurrences if x.component.name == 'D4_IMP_Fusee_' + t)
        v = adsk.core.Vector3D.create(1, 0, 0)
        v.transformBy(o.transform2)
        r.append(round(math.degrees(math.atan2(v.y, v.x)), 2))
    return r


def corps():
    out = []
    for o in group().childOccurrences:
        out += [b for b in o.bRepBodies]
    return out


def sans_serrage_pneu(coll):
    return [x for x in coll if not ({x[0][:5], x[1][:5]} == {'pneu_', 'jante'})]


def run(_context: str):
    d = design()
    g = group().component
    servo = next(j for j in g.joints if j.name == 'D4_SERVO_DIRECTION')
    res = {'poses': [], 'largeurs': []}
    for a in (-31, -30, -20, -10, 0, 10, 20, 30, 31):
        servo.jointMotion.rotationValue = math.radians(a)
        dd = angles()
        coll = sans_serrage_pneu([x for x in interferences(corps()) if x[2] > 0.001])
        res['poses'].append({'servo_deg': a, 'fusee_D_deg': dd[0], 'fusee_G_deg': dd[1], 'collisions': coll})
        print(a, dd, coll)
    servo.jointMotion.rotationValue = 0
    # au-delà de la borne servo : contrôle de la butée imprimée (fusée intérieure à ~38°)
    U = d.userParameters
    essais = [{'d4_planche_larg': '250 mm'}, {'d4_planche_larg': '350 mm'}, {'d4_planche_ep': '10 mm'}, {'d4_roue_av_d': '130 mm'},
              {'d4_planche_larg': '300 mm', 'd4_planche_ep': '5 mm', 'd4_roue_av_d': '150 mm'}]
    for essai in essais:
        for k, v in essai.items():
            U.itemByName(k).expression = v
        w = essai
        tl = d.timeline
        erreurs = [tl.item(i).name for i in range(tl.count) if tl.item(i).healthState != 0]
        servo.jointMotion.rotationValue = math.radians(30)
        a30 = angles()
        servo.jointMotion.rotationValue = 0
        bad = [j.name for j in g.joints if j.healthState != 0]
        bl = next(x for x in group().childOccurrences if x.component.name == 'D4_IMP_Biellette_D').bRepBodies.itemByName('biellette_D').boundingBox
        coll = sans_serrage_pneu([x for x in interferences(corps()) if x[2] > 0.001])
        res['largeurs'].append({'essai': w, 'fonctions_en_erreur': erreurs, 'collisions_neutre': coll, 'E_mm': ev('d4_E'), 'axe_av_sous_planche_mm': round(ev('d4_axe_av_sous_planche'), 1), 'biellette_L_mm': round(ev('d4_biellette_L'), 2),
                                'fusees_a_30deg': a30, 'liaisons_en_erreur': bad,
                                'biellette_x_mm': [round(bl.minPoint.x * 10, 1), round(bl.maxPoint.x * 10, 1)]})
        print(res['largeurs'][-1])
    lows = min(b.boundingBox.minPoint.z for b in corps()) * 10
    res['point_le_plus_bas_mm'] = round(lows, 1)
    res['dessous_planche_mm'] = round(ev('d4_roue_ar_d / 2 + d4_axe_ar_sous_planche'), 1)
    res['axe_avant_sol_mm'] = round(ev('d4_roue_ar_d / 2 + d4_axe_ar_sous_planche - d4_axe_av_sous_planche'), 1)
    print('point le plus bas', round(lows, 1), 'dessous planche', res['dessous_planche_mm'], 'axe avant', res['axe_avant_sol_mm'])
    open(OUT, 'w', encoding='utf-8').write(json.dumps(res, ensure_ascii=False, indent=1))
