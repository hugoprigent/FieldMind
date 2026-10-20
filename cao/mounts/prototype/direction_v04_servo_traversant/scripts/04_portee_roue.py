from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Ajoute sur la face extérieure de chaque fusée une portée annulaire de 1,5 mm qui n'appuie
que sur la bague intérieure du roulement de roue (le roulement est dans le moyeu de la roue,
la vis de roue est l'axe fixe). Diamètre : d4_portee_roue_d (12 mm pour 608ZZ, 14 pour 6000)."""
import adsk.core, adsk.fusion, math
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v04_servo_traversant/scripts/01_construire.py'), encoding='utf-8').read(), '01_construire.py', 'exec'))
_construire = run


def run(_context: str):
    param('d4_portee_roue_d', '12 mm', 'V04 | Portee de roue sur la fusee : bague interieure seulement (608ZZ : 12 ; 6000 : 14)')
    for s, tag in ((+1, 'D'), (-1, 'G')):
        c = occ('D4_IMP_Fusee_' + tag).component
        b = c.bRepBodies.itemByName('fusee_' + tag)
        x0 = s * JAMBE_X1
        sk = sketch(c, plane_x(c, x0, 'portee_roue'), 'portee_roue')
        ctr = centre_axe_roue(sk, x0)
        for nom, expr in (('ext', 'd4_portee_roue_d'), ('int', 'd4_vis_roue + 0.4 mm')):
            cc = sk.sketchCurves.sketchCircles.addByCenterRadius(ctr.geometry, ev(expr) / 20)
            sk.geometricConstraints.addCoincident(cc.centerSketchPoint, ctr)
            off = 1.0 if nom == 'ext' else 0.5
            sk.sketchDimensions.addDiameterDimension(cc, adsk.core.Point3D.create(ctr.geometry.x + off, ctr.geometry.y + off, 0)).parameter.expression = expr
        extrude(c, profiles(sk, 'ring'), 1.5, JOIN, 'portee_roue_bague_int', [b], down=s < 0)
        print(tag, 'portee ok', sk.isFullyConstrained, round(b.volume * 1000, 1))
