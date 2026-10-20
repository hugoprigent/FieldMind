from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Contrôles des capteurs (script 11) et relevé des perçages de la planche.

- Collisions entre tous les corps du groupe (STEP du C1 compris, hors recouvrements internes au
  STEP et pneu/jante emmanchés), direction au neutre puis à ±31°.
- Champ des webcams : rayons sur tout le champ (75° x 45°) depuis chaque centre optique ; tout corps
  du rover touché à moins de 3 m est un masquage.
- Plan laser du C1 : 360 rayons horizontaux à la hauteur optique (29,8 mm au-dessus de sa base),
  plus deux nappes à ±1,5° (planéité annoncée du balayage).
- Perçages : trous cylindriques de la planche (centre, diamètre), découpe du servo, contours utiles.
Écrit verification_capteurs.json et plan_percages.json à côté des scripts.
"""
import adsk.core, adsk.fusion, json, math, re
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
GROUP = 'FM_Direction_V04'
DOSSIER = str(_FM_ROOT / 'cao/mounts/prototype/direction_v04_servo_traversant')
HFOV, VFOV = 75.0, 45.0
C1_OPTIQUE, C1_DEMI = 29.8, 27.8


def tous_corps(gp):
    out = []
    for o in gp.childOccurrences:
        out += [(o.name, b) for b in o.bRepBodies]
        for oo in o.childOccurrences:
            out += [(o.name, b) for b in oo.bRepBodies]
    return out


def collisions(gp):
    corps = tous_corps(gp)
    col = adsk.core.ObjectCollection.create()
    for _, b in corps:
        col.add(b)
    i = design().createInterferenceInput(col)
    i.areCoincidentFacesIncluded = False
    out = []
    for r in design().analyzeInterference(i):
        a, b = r.entityOne, r.entityTwo
        na, nb = (x.assemblyContext.name if x.assemblyContext else '?' for x in (a, b))
        if re.fullmatch(r'Corps\d+', a.name) and re.fullmatch(r'Corps\d+', b.name):
            continue   # recouvrements internes du STEP Slamtec (corps « CorpsN » du C1)
        if {a.name.split('_')[0], b.name.split('_')[0]} == {'pneu', 'jante'}:
            continue   # pneu emmanché serré sur la jante (voulu)
        out.append({'a': '%s/%s' % (na, a.name), 'b': '%s/%s' % (nb, b.name), 'volume_mm3': round(r.interferenceBody.volume * 1000, 3)})
    return out


def rayons(root, origine, directions, exclure, portee):
    """Premier corps du rover touché par chaque rayon (hors corps exclus), à moins de `portee` mm."""
    touches = []
    for dname, dv in directions:
        hits = adsk.core.ObjectCollection.create()
        ents = root.findBRepUsingRay(origine, adsk.core.Vector3D.create(*dv), adsk.fusion.BRepEntityTypes.BRepBodyEntityType, -1.0, True, hits)
        best = None
        for k in range(ents.count):
            b = ents.item(k)
            if any(x in b.assemblyContext.fullPathName for x in exclure) if b.assemblyContext else False:
                continue
            hp = hits.item(k)
            dist = origine.distanceTo(hp) * 10
            if dist <= portee and (best is None or dist < best[1]):
                best = ('%s/%s' % (b.assemblyContext.name if b.assemblyContext else '', b.name), round(dist, 1))
        if best:
            touches.append({'rayon': dname, 'corps': best[0], 'distance_mm': best[1]})
    return touches


def run(_context: str):
    d = design()
    gp = group()
    g = gp.component
    root = d.rootComponent
    servo = next(j for j in g.joints if j.name == 'D4_SERVO_DIRECTION')
    res = {'collisions': {}}
    for a in (0, -31, 31):
        servo.jointMotion.rotationValue = math.radians(a)
        res['collisions']['servo_%+d_deg' % a] = collisions(gp)
    servo.jointMotion.rotationValue = 0

    # champ des webcams
    av, h = ev('d4_cam_axe_devant_vis'), ev('d4_cam_axe_sur_pied')
    res['webcams'] = {}
    for o in gp.childOccurrences:
        if not o.component.name.startswith('D4_REF_Webcam_UGREEN'):
            continue
        m = o.transform2
        og, xa, ya, za = m.getAsCoordinateSystem()
        L = P(0, -av, h)
        L.transformBy(m)
        dirs = []
        for ha in range(-35, 36, 5):
            for va in range(-20, 21, 5):
                th, ph = math.radians(ha * HFOV / 70), math.radians(va * VFOV / 40)
                v = [-math.cos(ph) * math.cos(th) * ya.asArray()[i] + math.cos(ph) * math.sin(th) * xa.asArray()[i] + math.sin(ph) * za.asArray()[i] for i in range(3)]
                dirs.append(('h%+.1f_v%+.1f' % (ha * HFOV / 70, va * VFOV / 40), v))
        dep = adsk.core.Point3D.create(*[L.asArray()[i] - 0.1 * ya.asArray()[i] for i in range(3)])   # 1 mm devant l'objectif
        res['webcams'][o.name] = {'centre_optique_mm': [round(v * 10, 1) for v in L.asArray()],
                                  'visee': [round(-v, 4) for v in ya.asArray()],
                                  'inclinaison_deg': round(math.degrees(math.asin(ya.asArray()[2])), 2),
                                  'rayons': len(dirs), 'masquages': rayons(root, dep, dirs, [o.name], 3000)}
    cams = list(res['webcams'].values())
    if len(cams) == 2:
        res['base_stereo_mm'] = round(math.dist(cams[0]['centre_optique_mm'], cams[1]['centre_optique_mm']), 2)

    # plan laser du C1
    yl = ev('d4_lidar_y')
    z_base = ev('d4_roue_ar_d / 2 + d4_axe_ar_sous_planche + d4_planche_ep + d4_profil_h') + 7.5
    zl = z_base + C1_OPTIQUE
    res['lidar'] = {'centre_mm': [0.0, round(yl, 1), round(zl, 1)], 'nappes': {}}
    for el in (0.0, -1.5, 1.5):
        dirs = []
        for k in range(0, 360, 2):
            t = math.radians(k)
            dirs.append(('az%03d' % k, [math.cos(t) * math.cos(math.radians(el)), math.sin(t) * math.cos(math.radians(el)), math.sin(math.radians(el))]))
        touches = []
        for nom, v in dirs:
            o = P(42 * v[0], yl + 42 * v[1], zl + 42 * v[2])
            touches += rayons(root, o, [(nom, v)], ['D4_REF_Lidar_C1'], 5000)
        res['lidar']['nappes']['%+.1f_deg' % el] = touches
    res['lidar']['hauteur_plan_sol_mm'] = round(zl, 1)

    # perçages de la planche (repère : X depuis l'axe, Y depuis l'axe des pivots ; Z dessus de planche)
    pc = next(o.component for o in g.occurrences if o.component.name == 'D4_REF_Planche_bois')
    pb = pc.bRepBodies.itemByName('REF_planche_bois_non_imprimee')
    trous = []
    for f in pb.faces:
        gm = f.geometry
        if gm.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and abs(abs(gm.axis.z) - 1) < 1e-6:
            t = {'x': round(gm.origin.x * 10, 2), 'y': round(gm.origin.y * 10, 2), 'd': round(gm.radius * 20, 2)}
            if t not in trous:
                trous.append(t)
    sy = ev('d4_servo_y')
    plan = {
        'planche': {'x0': -ev('d4_planche_larg') / 2, 'x1': ev('d4_planche_larg') / 2, 'y0': -ev('d4_planche_avant'),
                    'y1': ev('d4_planche_long') - ev('d4_planche_avant'), 'ep': ev('d4_planche_ep')},
        'trous': trous,
        'decoupe_servo': {'x0': -10.5, 'x1': 10.5, 'y0': round(sy - 10 - 0.5, 2), 'y1': round(sy - 10 + 40.4 + 0.5, 2)},
        'empreintes': {},
        'parametres': {n: round(ev(n), 2) for n in ('d4_E', 'd4_empattement', 'd4_lidar_y', 'd4_profil_h', 'd4_cam_y', 'd4_cam_profil_h', 'd4_stereo_base', 'd4_servo_y')},
    }
    for o in gp.childOccurrences:
        nom = o.component.name
        if nom.startswith(('D4_REF_Profile_40x40_', 'D4_REF_Equerre_40_')):
            bb = o.boundingBox
            cle = nom.replace('D4_REF_Profile_40x40_', 'profile_').replace('D4_REF_Equerre_40_', 'equerre_')
            plan['empreintes'][cle] = {'x0': round(bb.minPoint.x * 10, 1), 'x1': round(bb.maxPoint.x * 10, 1),
                                       'y0': round(bb.minPoint.y * 10, 1), 'y1': round(bb.maxPoint.y * 10, 1)}
        if nom in ('D4_IMP_Support_pivot_D', 'D4_IMP_Support_pivot_G', 'D4_REF_Support_moteur_arriere'):
            bb = o.boundingBox
            plan['empreintes']['dessous_' + o.name] = {'x0': round(bb.minPoint.x * 10, 1), 'x1': round(bb.maxPoint.x * 10, 1),
                                                       'y0': round(bb.minPoint.y * 10, 1), 'y1': round(bb.maxPoint.y * 10, 1)}
    open(DOSSIER + '/verification_capteurs.json', 'w', encoding='utf-8').write(json.dumps(res, ensure_ascii=False, indent=1))
    open(DOSSIER + '/plan_percages.json', 'w', encoding='utf-8').write(json.dumps(plan, ensure_ascii=False, indent=1))
    print(json.dumps({'collisions': {k: v for k, v in res['collisions'].items()},
                      'webcams': {k: {kk: vv for kk, vv in v.items() if kk != 'rayons'} for k, v in res['webcams'].items()},
                      'base': res.get('base_stereo_mm'),
                      'lidar': {k: (len(v) if isinstance(v, list) else v) for k, v in res['lidar']['nappes'].items()},
                      'lidar_h': res['lidar']['hauteur_plan_sol_mm'], 'nb_trous': len(trous)}, ensure_ascii=False)[:5000])
