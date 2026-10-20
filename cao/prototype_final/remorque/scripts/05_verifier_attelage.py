from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Contrôles de l'attelage, dans le document du rover où la remorque est insérée (référence au document
« remorque fieldmind », volets repliés comme enregistrés) :
- collisions remorque / rover, remorque dans l'axe ;
- braquage autour de l'axe d'attelage : premier angle de contact de chaque côté (les contacts déjà
  présents dans l'axe sont ignorés) ;
- tangage et roulis de ±10° : la tête du timon pivote autour du milieu de son trou en diabolo et se
  soulève de 1,7 mm (elle roule sur le bord de l'anneau d'appui) ; elle ne doit toucher ni la vis M10,
  ni la chape, ni les rondelles ;
- jeu entre le timon et le rover hors attelage.
La remorque est remise dans l'axe à la fin. Écrit verification_attelage.json.
"""
import adsk.core, adsk.fusion, json, math, re
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/remorque/scripts/rm_helpers.py'), encoding='utf-8').read(), 'rm_helpers.py', 'exec'))
DOC_PREFIXE = DOC_ROVER
ATTELAGE = ('chape_attelage', 'REF_vis_H_M10x55_attelage', 'REF_ecrou_frein_M10_attelage',
            'REF_rondelle_M10_haut_attelage', 'REF_rondelle_M10_bas_attelage')


def corps_de(occ):
    out = list(occ.bRepBodies)
    for o in occ.childOccurrences:
        out += corps_de(o)
    return out


def collisions(corps, garder):
    col = adsk.core.ObjectCollection.create()
    for b in corps:
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
        if not (garder(a) or garder(b)):
            continue
        out.append({'a': a.name, 'b': b.name, 'volume_mm3': round(r.interferenceBody.volume * 1000, 3)})
    return out


def rotation(angle_deg, axe, origine):
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.radians(angle_deg), adsk.core.Vector3D.create(*axe), P(*origine))
    return m


def run(_context: str):
    root = design().rootComponent
    occ = {o.component.name: o for o in root.occurrences}
    rm = next(o for o in root.occurrences if o.isReferencedComponent and o.component.name.startswith(DOC_REMORQUE))
    pf, v04 = occ['FM_Prototype_Final'], occ['FM_Direction_V04']
    rotule = next((j for j in root.asBuiltJoints if j.name == 'attelage_remorque'), None)
    if rotule:
        rotule.isSuppressed = True                   # essais par déplacement direct de la remorque
    rm.transform2 = adsk.core.Matrix3D.create()
    remorque = corps_de(rm)
    rover = corps_de(pf) + corps_de(v04)
    noms_remorque = {b.name for b in remorque}
    est_remorque = lambda b: b.name in noms_remorque and not re.fullmatch(r'Corps\d+', b.name)
    patte = [b for b in rover if b.name in ATTELAGE]
    # arrière du rover seulement, sans les modèles importés (lidar, Knob) : sinon Fusion sature la mémoire
    proches = [b for b in rover if b.boundingBox.maxPoint.y * 10 > 300 and not re.fullmatch(r'Corps\d+|REF_knob_\d+', b.name)]
    remorque = [b for b in remorque if not b.name.startswith(('REF_SG90', 'REF_palonnier'))]
    obst = [b for b in proches if b.name not in ATTELAGE]
    res = {}
    res['collisions_dans_axe'] = collisions(remorque + proches, lambda b: est_remorque(b) or b.name in ATTELAGE)
    cle = lambda r: tuple(sorted((r['a'], r['b'])))
    base = {cle(r) for r in collisions(remorque + obst, est_remorque)}
    res['braquage_max_deg'] = {}
    for signe, nom in ((1, 'vers_la_gauche'), (-1, 'vers_la_droite')):
        dernier_libre, premier = 0, None
        for a in list(range(10, 50, 10)) + list(range(50, 92, 2)):
            rm.transform2 = rotation(signe * a, (0, 0, 1), (0, PIVOT_Y, 0))
            c = [r for r in collisions(remorque + obst, est_remorque) if cle(r) not in base]
            if c:
                premier = {'angle_deg': a, 'contact': c[:3]}
                break
            dernier_libre = a
        res['braquage_max_deg'][nom] = {'libre_jusqu_a': dernier_libre, 'premier_contact': premier}
    timon = [b for b in remorque if b.name == 'timon']
    res['bascule'] = {}
    for nom, axe in (('tangage', (1, 0, 0)), ('roulis', (0, 1, 0))):
        for a in (-10, 10):
            m = rotation(a, axe, (0, PIVOT_Y, TIMON_Z0 + TIMON_EP / 2))
            m.translation = adsk.core.Vector3D.create(m.translation.x, m.translation.y, m.translation.z + 0.17)
            rm.transform2 = m
            res['bascule']['%s_%+d' % (nom, a)] = collisions(timon + patte, lambda b: True)
    rm.transform2 = adsk.core.Matrix3D.create()
    mm = app().measureManager
    res['jeu_timon_rover_hors_attelage_mm'] = round(min(mm.measureMinimumDistance(timon[0], b).value for b in obst) * 10, 2)
    if rotule:
        rotule.isSuppressed = False
    json.dump(res, open(DOSSIER + '/verification_attelage.json', 'w', encoding='utf-8'), indent=1, ensure_ascii=False)
    print(json.dumps(res, ensure_ascii=False)[:5000])
