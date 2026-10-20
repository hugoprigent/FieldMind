from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Crée le document Fusion de la remorque, « remorque fieldmind », dans le même dossier que le document
du rover, et retire l'ancienne remorque (groupe FM_Remorque) du document du rover. À lancer une seule
fois, document du rover actif. Le document de la remorque est enregistré (il faut qu'il le soit pour
l'insérer par référence) ; celui du rover ne l'est pas.
"""
import adsk.core, adsk.fusion, json
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/remorque/scripts/rm_helpers.py'), encoding='utf-8').read(), 'rm_helpers.py', 'exec'))


def run(_context: str):
    a = app()
    rover = a.activeDocument
    assert rover.name.startswith(DOC_ROVER), 'Activer le document du rover'
    d = adsk.fusion.Design.cast(rover.products.itemByProductType('DesignProductType'))
    for o in list(d.rootComponent.occurrences):
        if o.component.name in ('FM_Remorque', 'TMP_TEST', 'TMP_IMPORT'):
            o.deleteMe()
    dossier = rover.dataFile.parentFolder
    existant = [f for f in dossier.dataFiles if f.name == DOC_REMORQUE]
    if existant:
        doc = a.documents.open(existant[0], True)
    else:
        doc = a.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        doc.saveAs(DOC_REMORQUE, dossier, 'Remorque porte-drone FieldMind (insérée dans le document du rover)', '')
    doc.activate()
    print(json.dumps({'remorque': doc.name, 'dossier': dossier.name, 'deja_existant': bool(existant)}, ensure_ascii=False))
