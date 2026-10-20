from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Supports des roues dans leur propre document Fusion, « supports roues fieldmind », enregistré dans le
dossier du rover et inséré par référence dans le document du rover (comme la remorque) : le document du
rover reste plus léger, et ces pièces s'enregistrent à part.
- SR_IMP_Support_moteur_arriere : support moteur agrandi (16_supports_moteurs.py), une seule pièce
  placée deux fois, la gauche tournée de 180° ;
Les supports de pivot restent ceux du rover (le renfort de 17_supports_pivot.py n'est pas retenu : c'est
la fusée qu'il faut renforcer, 19_fusees_renfort.py). Dans le rover : l'ancien support moteur est vidé. Document du rover actif. Réexécutable :
refait le contenu du document des supports et met la référence à jour.
Exporte les STL et ajoute les trous de la planche à trous_composants.json (05_percages.py).
"""
import adsk.core, adsk.fusion, json, math
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/16_supports_moteurs.py'), encoding='utf-8').read(), '16_supports_moteurs.py', 'exec'))
DOC_ROVER, DOC_SUPPORTS = 'prtotype final fieldmind', 'supports roues fieldmind'


def design():
    """Design du document actif, quel qu'il soit (les helpers du rover exigent le document du rover)."""
    return adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)


def matrice(tx, ty, tz, demi_tour=False):
    m = adsk.core.Matrix3D.create()
    if demi_tour:
        m.setToRotation(math.pi, adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
    m.translation = adsk.core.Vector3D.create(tx / 10, ty / 10, tz / 10)
    return m


def run(_context: str):
    a = app()
    rover = next(d_ for d_ in a.documents if d_.name.startswith(DOC_ROVER))
    rover.activate()
    dr = adsk.fusion.Design.cast(rover.products.itemByProductType('DesignProductType'))
    v04 = next(o for o in dr.rootComponent.occurrences if o.component.name == 'FM_Direction_V04')
    for tag in ('D', 'G'):                 # supports de pivot d'origine : renforts d'un essai retirés, corps visibles
        c = next(o.component for o in v04.childOccurrences if o.component.name == 'D4_IMP_Support_pivot_' + tag)
        for coll in (c.features.extrudeFeatures, c.sketches, c.constructionPlanes):
            for x in reversed([coll.item(i) for i in range(coll.count)]):
                if x.isValid and 'renfort' in x.name:
                    x.deleteMe()
        c.bRepBodies.itemByName('support_pivot_' + tag).isLightBulbOn = True
    # document des supports
    dossier = rover.dataFile.parentFolder
    existant = [f for f in dossier.dataFiles if f.name == DOC_SUPPORTS]
    ouvert = next((d_ for d_ in a.documents if d_.name.startswith(DOC_SUPPORTS)), None)
    for d_ in list(a.documents):          # documents « Sans nom » vides laissés par un essai
        if not d_.isSaved and d_.name.startswith('Sans nom'):
            r_ = adsk.fusion.Design.cast(d_.products.itemByProductType('DesignProductType')).rootComponent
            if r_.occurrences.count == 0 and r_.bRepBodies.count == 0:
                d_.close(False)
    doc = ouvert or (a.documents.open(existant[0], True) if existant else a.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType))
    doc.activate()
    d = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    root = d.rootComponent
    for o in list(root.occurrences):
        o.deleteMe()
    om = root.occurrences.addNewComponent(matrice(X_ROUE, Y_AXE, 100.0))
    om.isGroundToParent = False
    cm = om.component
    cm.name = 'SR_IMP_Support_moteur_arriere'
    bm = support_moteur(cm, 5.0)
    root.occurrences.addExistingComponent(cm, matrice(-X_ROUE, Y_AXE, 100.0, True))
    sortie = {'moteur': stl(bm, DOSSIER + '/exports/stl/support_moteur_arriere.stl', ('y', 90))}
    if doc.isSaved:
        doc.save('Supports des roues (18_document_supports.py)')
    else:
        doc.saveAs(DOC_SUPPORTS, dossier, 'Supports des roues du rover FieldMind (insérés dans le document du rover)', '')
    # rover : anciens corps retirés ou masqués, référence insérée ou mise à jour
    rover.activate()
    cmo = next(o.component for o in v04.childOccurrences if o.component.name == 'D4_REF_Support_moteur_arriere')
    for coll in (cmo.features.baseFeatures, cmo.features.extrudeFeatures, cmo.sketches, cmo.constructionPlanes):
        for x in reversed([coll.item(i) for i in range(coll.count)]):
            try:
                if x.isValid:
                    x.deleteMe()
            except RuntimeError:
                pass
    refs = [o for o in dr.rootComponent.occurrences if o.isReferencedComponent and o.component.name.startswith(DOC_SUPPORTS)]
    if refs:
        for r in rover.documentReferences:
            if r.dataFile and r.dataFile.name == DOC_SUPPORTS:
                r.getLatestVersion()
        o = refs[0]
    else:
        o = dr.rootComponent.occurrences.addByInsert(doc.dataFile, adsk.core.Matrix3D.create(), True)
    o.isGrounded = True
    tc = json.load(open(DOSSIER + '/trous_composants.json', encoding='utf-8'))
    tc['trous'] = [t for t in tc['trous'] if t[3] != 'support_pivot']   # plus de trous de renfort des pivots
    json.dump(tc, open(DOSSIER + '/trous_composants.json', 'w', encoding='utf-8'), indent=1)
    rover.activate()                       # la mise à jour de la référence peut rendre la main au document des supports
    print(json.dumps({'document': doc.name, 'version': doc.dataFile.versionNumber, 'reference': o.name,
                      'volume_cm3': round(bm.volume, 1), 'morceaux': bm.lumps.count, 'stl_mm': sortie}))
