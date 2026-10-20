from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Contrôles du prototype final, direction au neutre et à ±31° :
- collisions entre la coque (12 pièces), les composants et tout le rover (hors recouvrements
  internes du STEP du lidar et pneu/jante emmanchés) ;
- jeux mini entre la coque et les roues (avant braquées, arrière), la barre des caméras, le support
  du lidar ;
- champ des webcams (75° × 45°) et plan laser du C1 (±1,5°) : aucun corps du rover touché ;
- encombrement de chaque pièce imprimée.
Écrit verification_prototype_final.json.
"""
import adsk.core, adsk.fusion, json, math, re
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/pf_helpers.py'), encoding='utf-8').read(), 'pf_helpers.py', 'exec'))
DOSSIER = str(_FM_ROOT / 'cao/prototype_final')
HFOV, VFOV = 75.0, 45.0


def groupes():
    """Groupes du rover et document des supports de roues inséré par référence."""
    root = design().rootComponent
    return [o for o in root.occurrences if o.component.name in ('FM_Direction_V04', 'FM_Prototype_Final')
            or (o.isReferencedComponent and o.component.name.startswith('supports roues fieldmind'))]


def corps_tous():
    out = []
    for g in groupes():
        for o in g.childOccurrences:
            out += [(o.component.name, b) for b in o.bRepBodies if b.isLightBulbOn]   # corps masqués : remplacés (anciens supports de pivot)
            for oo in o.childOccurrences:
                out += [(o.component.name, b) for b in oo.bRepBodies if b.isLightBulbOn]
    return out


def collisions(corps):
    col = adsk.core.ObjectCollection.create()
    for _, b in corps:
        col.add(b)
    i = design().createInterferenceInput(col)
    i.areCoincidentFacesIncluded = False
    out = []
    for r in design().analyzeInterference(i):
        a, b = r.entityOne, r.entityTwo
        if re.fullmatch(r'Corps\d+', a.name) and re.fullmatch(r'Corps\d+', b.name):
            continue
        if {a.name.split('_')[0], b.name.split('_')[0]} == {'pneu', 'jante'}:
            continue
        if not (a.name.startswith('coque_') or b.name.startswith('coque_') or a.name.startswith('berceau') or b.name.startswith('berceau')):
            continue
        out.append({'a': a.name, 'b': b.name, 'volume_mm3': round(r.interferenceBody.volume * 1000, 3)})
    return out


def rayons(root, origine, directions, exclure, portee):
    touches = []
    for nom, dv in directions:
        hits = adsk.core.ObjectCollection.create()
        ents = root.findBRepUsingRay(origine, adsk.core.Vector3D.create(*dv), adsk.fusion.BRepEntityTypes.BRepBodyEntityType, -1.0, True, hits)
        best = None
        for k in range(ents.count):
            b = ents.item(k)
            ctx = b.assemblyContext.fullPathName if b.assemblyContext else ''
            if any(x in ctx for x in exclure):
                continue
            dist = origine.distanceTo(hits.item(k)) * 10
            if dist <= portee and (best is None or dist < best[1]):
                best = ('%s/%s' % (b.assemblyContext.component.name if b.assemblyContext else '', b.name), round(dist, 1))
        if best:
            touches.append({'rayon': nom, 'corps': best[0], 'distance_mm': best[1]})
    return touches


def run(_context: str):
    d = design()
    root = d.rootComponent
    v04 = next(g for g in groupes() if g.component.name == 'FM_Direction_V04')
    servo = next(j for j in v04.component.joints if j.name == 'D4_SERVO_DIRECTION')
    mm = app().measureManager
    corps = corps_tous()
    coque = [b for n, b in corps if b.name.startswith('coque_')]
    res = {'collisions': {}, 'jeux_mini_mm': {}}
    for a in (0, -31, 31):
        servo.jointMotion.rotationValue = math.radians(a)
        res['collisions']['servo_%+d' % a] = collisions(corps)
        for nom in ('pneu_TPU_D', 'pneu_TPU_G', 'jante_D', 'jante_G'):
            pb = next(b for n, b in corps if b.name == nom)
            res['jeux_mini_mm']['%s_servo_%+d' % (nom, a)] = round(min(mm.measureMinimumDistance(pb, s).value for s in coque) * 10, 2)
    servo.jointMotion.rotationValue = 0
    for nom in ('pneu_arriere_D', 'pneu_arriere_G', 'jante_arriere_D', 'jante_arriere_G', 'barre_stereo', 'support_lidar', 'REF_webcam_pied'):
        for n, b in corps:
            if b.name == nom:
                res['jeux_mini_mm']['%s/%s' % (n, nom)] = round(min(mm.measureMinimumDistance(b, s).value for s in coque) * 10, 2)
    # champ des webcams
    av, h = ev('d4_cam_axe_devant_vis'), ev('d4_cam_axe_sur_pied')
    res['webcams'] = {}
    for o in v04.childOccurrences:
        if not o.component.name.startswith('D4_REF_Webcam_UGREEN'):
            continue
        m = o.transform2
        _, xa, ya, za = m.getAsCoordinateSystem()
        L = P(0, -av, h)
        L.transformBy(m)
        dirs = []
        for ha in range(-35, 36, 5):
            for va in range(-20, 21, 5):
                th, ph = math.radians(ha * HFOV / 70), math.radians(va * VFOV / 40)
                v = [-math.cos(ph) * math.cos(th) * ya.asArray()[i] + math.cos(ph) * math.sin(th) * xa.asArray()[i] + math.sin(ph) * za.asArray()[i] for i in range(3)]
                dirs.append(('h%+.1f_v%+.1f' % (ha * HFOV / 70, va * VFOV / 40), v))
        dep = adsk.core.Point3D.create(*[L.asArray()[i] - 0.1 * ya.asArray()[i] for i in range(3)])
        res['webcams'][o.component.name] = rayons(root, dep, dirs, [o.name], 3000)
    # plan laser du C1
    yl = ev('d4_lidar_y')
    rh = ' + d4_rehausse_lidar' if design().userParameters.itemByName('d4_rehausse_lidar') else ''   # rehausse sous le support (profilé de 90)
    zl = ev('d4_roue_ar_d / 2 + d4_axe_ar_sous_planche + d4_planche_ep + d4_profil_h' + rh) + 7.5 + 29.8
    res['lidar'] = {}
    for el in (0.0, -1.5, 1.5):
        touches = []
        for k in range(0, 360, 2):
            t = math.radians(k)
            v = [math.cos(t) * math.cos(math.radians(el)), math.sin(t) * math.cos(math.radians(el)), math.sin(math.radians(el))]
            o_ = P(42 * v[0], yl + 42 * v[1], zl + 42 * v[2])
            touches += rayons(root, o_, [('az%03d' % k, v)], ['D4_REF_Lidar_C1'], 5000)
        res['lidar']['%+.1f_deg' % el] = touches
    # encombrement des pièces imprimées
    res['pieces'], res['morceaux'] = {}, {}
    for n, b in corps:
        if b.name.startswith(('coque_', 'berceau')):
            bx = b.boundingBox
            res['pieces'][b.name] = sorted(round((bx.maxPoint.asArray()[i] - bx.minPoint.asArray()[i]) * 10, 1) for i in range(3))
            res['morceaux'][b.name] = b.lumps.count      # 1 attendu : une pièce d'un seul tenant
    res['pieces_en_plusieurs_morceaux'] = [k for k, v in res['morceaux'].items() if v != 1]
    json.dump(res, open(DOSSIER + '/verification_prototype_final.json', 'w', encoding='utf-8'), indent=1, ensure_ascii=False)
    print(json.dumps({'collisions': res['collisions'], 'jeux': res['jeux_mini_mm'],
                      'webcams': {k: len(v) for k, v in res['webcams'].items()},
                      'lidar': {k: len(v) for k, v in res['lidar'].items()}, 'pieces': res['pieces'],
                      'en_plusieurs_morceaux': res['pieces_en_plusieurs_morceaux']}, ensure_ascii=False)[:5000])
