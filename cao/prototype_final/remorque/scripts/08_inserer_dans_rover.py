from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Active le document du rover et y insère la remorque par référence (à l'identité : elle est dessinée
dans le repère du rover), ou met la référence à jour si elle y est déjà. L'occurrence n'est pas
ancrée : 05_verifier_attelage.py la fait tourner autour de l'axe d'attelage."""
import adsk.core, adsk.fusion, json, math
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/remorque/scripts/rm_helpers.py'), encoding='utf-8').read(), 'rm_helpers.py', 'exec'))


def run(_context: str):
    a = app()
    rem = next(d_ for d_ in a.documents if d_.name.startswith(DOC_REMORQUE))
    rover = next(d_ for d_ in a.documents if d_.name.startswith(DOC_ROVER))
    rover.activate()
    d = adsk.fusion.Design.cast(rover.products.itemByProductType('DesignProductType'))
    refs = [o for o in d.rootComponent.occurrences if o.isReferencedComponent and o.component.name.startswith(DOC_REMORQUE)]
    out = {}
    if refs:
        for j in list(d.rootComponent.asBuiltJoints):   # la rotule tient l'ancien timon : sinon la mise à jour échoue
            if j.name == 'attelage_remorque':
                j.deleteMe()
        for r in rover.documentReferences:
            if r.dataFile and r.dataFile.name == DOC_REMORQUE:
                try:
                    r.getLatestVersion()
                    out['mise_a_jour'] = True
                except Exception as e:
                    out['mise_a_jour'] = str(e)[:200]
        o = refs[0]
    else:
        o = d.rootComponent.occurrences.addByInsert(rem.dataFile, adsk.core.Matrix3D.create(), True)
        out['inseree'] = True
    o.isGrounded = False
    o.transform2 = adsk.core.Matrix3D.create()
    out['occurrence'] = o.name
    out['rotule'] = rotule_attelage(d, o)
    print(json.dumps(out, ensure_ascii=False))


def rotule_attelage(d, orem):
    """Liaison rotule (as-built) entre la remorque et la chape, centrée au milieu du trou en diabolo de la
    tête du timon : lacet ±64° (libre de contact, voir 05_verifier_attelage.py), tangage et roulis ±10°.
    Le rover sert de référence : ses deux groupes sont ancrés."""
    root = d.rootComponent
    for j in list(root.asBuiltJoints):
        if j.name == 'attelage_remorque':
            j.deleteMe()
    occ = {o_.component.name: o_ for o_ in root.occurrences}
    for n in ('FM_Prototype_Final', 'FM_Direction_V04'):
        occ[n].isGrounded = True
    och = next(o_ for o_ in occ['FM_Prototype_Final'].childOccurrences if o_.component.name == 'PF_IMP_Attelage')
    cc = och.component
    for coll, n in ((cc.sketches, 'esq_centre_attelage'), (cc.constructionPlanes, 'plan_centre_attelage')):
        x = coll.itemByName(n)
        if x:
            x.deleteMe()
    s = sketch(cc, plane_z(cc, TIMON_Z0 + TIMON_EP / 2, 'centre_attelage'), 'centre_attelage')
    pt = s.sketchPoints.add(sp(s, 0.0, PIVOT_Y))
    # seule orientation acceptée par Fusion pour une rotule sur un point : « pitch » autour de Z (vertical),
    # « yaw » autour de X ; « roll » retombe sur Z quand le tangage est nul (limite de la rotule de Fusion)
    otim = next(o_ for o_ in orem.childOccurrences if o_.component.name == 'RM_IMP_Timon')   # le timon, lié au reste par le groupe rigide
    ji = root.asBuiltJoints.createInput(otim, och, adsk.fusion.JointGeometry.createByPoint(pt.createForAssemblyContext(och)))
    ji.setAsBallJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection, adsk.fusion.JointDirections.XAxisJointDirection)
    j = root.asBuiltJoints.add(ji)
    j.name = 'attelage_remorque'
    m = j.jointMotion
    axes = {'pitch': 'lacet (Z) ±64°', 'yaw': 'tangage (X) ±10°', 'roll': '±10°'}
    for lim, deg in ((m.pitchLimits, 64.0), (m.yawLimits, 10.0), (m.rollLimits, 10.0)):
        lim.isMinimumValueEnabled = lim.isMaximumValueEnabled = True
        lim.minimumValue, lim.maximumValue = -math.radians(deg), math.radians(deg)
    return {'nom': j.name, 'axes': axes}
