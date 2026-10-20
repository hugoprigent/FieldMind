from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Enregistre une nouvelle version du document de la remorque (volets repliés, comme les laisse
02_verifier.py), pour que l'assemblage du rover puisse se mettre à jour."""
import adsk.core, adsk.fusion, json
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/remorque/scripts/rm_helpers.py'), encoding='utf-8').read(), 'rm_helpers.py', 'exec'))


def run(_context: str):
    doc = next(d_ for d_ in app().documents if d_.name.startswith(DOC_REMORQUE))
    doc.activate()
    volets_replies(True)
    d = design()
    d.rootComponent.isJointsFolderLightBulbOn = False     # symboles des liaisons masqués dans les vues
    if d.snapshots.hasPendingSnapshot:              # capture la position des volets (sinon non enregistrée)
        d.snapshots.add()
    ok = doc.save('Remorque reconstruite par 01_remorque.py, volets repliés')
    print(json.dumps({'enregistre': ok, 'version': doc.dataFile.versionNumber}))
