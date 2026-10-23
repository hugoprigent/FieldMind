from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""SG90 à la place du DT996 (grillé) pour la direction, dans son propre document « servo SG90 direction
fieldmind » (dessiné dans le repère du rover, enregistré, pas inséré dans le rover pour ne pas l'alourdir) :
- support_sg90 : plaque de 3 mm posée sur la planche, vissée dans les 4 trous des oreilles du DT996
  (S1–S4, M2,5 + écrou sous la planche), avec un bouchon qui entre dans la découpe 41,4 × 21 du DT996.
  Les oreilles du SG90 reposent dans une cuvette (fond 1,5 mm au-dessus de la planche), vis M2,5 dans des
  avant-trous Ø2,2 ; le bas du SG90 passe dans la découpe, l'axe au même endroit que celui du DT996 ;
- palonnier_sg90 : le palonnier imprimé du rover (biellettes inchangées), empreinte de l'étoile bouchée,
  plus un bossage avec l'empreinte du palonnier 1 bras du SG90 (sans picots, comme les ferrures de la
  remorque). Une vis M2,5 + rondelle large, par-dessous, traverse le palonnier et la tête du SG90 et se
  visse dans l'axe du servo.
Contrôle : palonnier, tête et SG90 contre la planche et le support, au neutre et à ±31°. STL. Rover actif.
"""
import adsk.core, adsk.fusion, json, math
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/04_exporter.py'), encoding='utf-8').read(), '04_exporter.py', 'exec'))
DOSSIER = str(_FM_ROOT / 'cao/prototype_final')
SOURCES = DOSSIER + '/sources/'
DOC_ROVER, DOC_SG90 = 'prtotype final fieldmind', 'servo SG90 direction fieldmind'
AXE = (0.0, 57.3)                    # axe du DT996 (cannelure)
ZP = 140.0                           # dessus de la planche de 10 mm
FOND, PLAQUE = 1.5, 3.0              # fond de la cuvette des oreilles et épaisseur du support au-dessus de la planche
BOUCHON = (-10.2, 47.1, 10.2, 87.9, 134.0)   # découpe 41,4 × 21 moins 0,3 par côté, jusqu'à 6 mm sous le dessus
TROUS_S = [(-5.0, 43.5), (5.0, 43.5), (-5.0, 91.5), (5.0, 91.5)]
H_TETE = ZP + FOND - 13.6            # face extérieure de la tête du SG90 (13,6 mm sous la face des oreilles) : 127,9
EMPREINTE, ECHELLE = 1.0, 1.04


def design():
    return adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)


def copie_corps(c, t, nom):
    bf = c.features.baseFeatures.add()
    bf.startEdit()
    c.bRepBodies.add(t, bf)
    bf.finishEdit()
    b = bf.bodies.item(0)
    b.name = nom
    return b


def volume_commun(a, b, m=None):
    tbm = adsk.fusion.TemporaryBRepManager.get()
    ta, tb = tbm.copy(a), tbm.copy(b)
    if m:
        tbm.transform(ta, m)
    try:
        tbm.booleanOperation(ta, tb, adsk.fusion.BooleanTypes.IntersectionBooleanType)
        return round(ta.volume * 1000, 2)
    except Exception:
        return 0.0


def run(_context: str):
    a = app()
    tbm = adsk.fusion.TemporaryBRepManager.get()
    rover = next(d_ for d_ in a.documents if d_.name.startswith(DOC_ROVER))
    dr = adsk.fusion.Design.cast(rover.products.itemByProductType('DesignProductType'))
    pal0 = planche0 = None
    for o in dr.rootComponent.allOccurrences:
        for b in o.bRepBodies:
            if o.component.name == 'D4_IMP_Palonnier' and b.name == 'palonnier':
                pal0 = tbm.copy(b)
            if 'planche' in b.name.lower() and b.boundingBox.minPoint.z * 10 > 125 and b.boundingBox.maxPoint.z * 10 < 145 \
                    and (planche0 is None or b.volume > planche0.volume):
                planche0 = tbm.copy(b)
    # document du SG90 (refait à chaque fois)
    dossier = rover.dataFile.parentFolder
    doc = next((d_ for d_ in a.documents if d_.name.startswith(DOC_SG90)), None)
    if doc is None:
        ex = [f for f in dossier.dataFiles if f.name == DOC_SG90]
        doc = a.documents.open(ex[0], True) if ex else a.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    doc.activate()
    root = design().rootComponent
    for o in list(root.occurrences):
        o.deleteMe()

    def piece(nom):
        o = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        o.isGroundToParent = False
        o.component.name = nom
        return o.component
    # références : planche (avec ses trous), SG90 et sa tête
    cr = piece('SG_REF_Planche')
    planche = copie_corps(cr, planche0, 'REF_planche')
    cs = piece('SG_REF_SG90')
    importer_step(cs, SOURCES + 'servo_SG90_freecad.step', (AXE[0], AXE[1] + 16.6, ZP + FOND + 18.4), (1, 0, 0), (0, -1, 0), (0, 0, -1), 'SG90')
    sg = [b for b in cs.bRepBodies]
    ct = piece('SG_REF_Tete_SG90')
    importer_step(ct, SOURCES + 'servo_SG90_palonnier_1_bras_freecad.step', (AXE[0] + 36.3, AXE[1] - 27.0, H_TETE - 25.67), (0, 1, 0), (-1, 0, 0), (0, 0, 1), 'tete_SG90')
    tete = [b for b in ct.bRepBodies]
    smn = [min(b.boundingBox.minPoint.asArray()[i] for b in sg) * 10 for i in range(3)]
    smx = [max(b.boundingBox.maxPoint.asArray()[i] for b in sg) * 10 for i in range(3)]
    tmn = [min(b.boundingBox.minPoint.asArray()[i] for b in tete) * 10 for i in range(3)]
    tmx = [max(b.boundingBox.maxPoint.asArray()[i] for b in tete) * 10 for i in range(3)]
    # trous des oreilles du SG90 : cylindres verticaux de petit rayon hors du corps
    oreilles = set()
    for b in sg:
        for f in b.faces:
            g = f.geometry
            if g.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and abs(abs(g.axis.z) - 1) < 1e-3 and 0.06 < g.radius < 0.15 \
                    and abs(g.origin.y * 10 - (smn[1] + smx[1]) / 2) > 11.5:
                oreilles.add((round(g.origin.x * 10, 1), round(g.origin.y * 10, 1)))
    if len(oreilles) != 2:
        yc = (smn[1] + smx[1]) / 2
        oreilles = {(AXE[0], yc - 13.9), (AXE[0], yc + 13.9)}
    # ---- support
    c = piece('SG_IMP_Support_SG90')
    b = body(zbox(c, 'plaque', -10.0, 40.0, 10.0, 95.0, ZP, PLAQUE), 'support_sg90', BLANC)
    zbox(c, 'bouchon', BOUCHON[0], BOUCHON[1], BOUCHON[2], BOUCHON[3], BOUCHON[4], ZP - BOUCHON[4] + 0.01, JOIN, [b])
    zbox(c, 'cuvette', smn[0] - 0.3, smn[1] - 0.3, smx[0] + 0.3, smx[1] + 0.3, ZP + FOND, PLAQUE, CUT, [b])
    yc = (smn[1] + smx[1]) / 2
    zbox(c, 'passage', smn[0] - 0.3, yc - 11.55, smx[0] + 0.3, yc + 11.55, BOUCHON[4] - 1, ZP + PLAQUE - BOUCHON[4] + 2, CUT, [b])
    for k, (x, y) in enumerate(sorted(oreilles)):
        zcyl(c, 'avant_trou_%d' % k, x, y, 2.2, BOUCHON[4] - 1, ZP + PLAQUE - BOUCHON[4] + 2, CUT, [b])
    for k, (x, y) in enumerate(TROUS_S):
        zcyl(c, 'trou_S%d' % (k + 1), x, y, 2.9, ZP - 1, PLAQUE + 2, CUT, [b])
    support = b
    # ---- palonnier : empreinte de l'étoile bouchée, bossage et empreinte de la tête du SG90
    c = piece('SG_IMP_Palonnier_SG90')
    p = copie_corps(c, pal0, 'palonnier_sg90')
    paint(p, BLANC)
    zcyl(c, 'bouche_etoile', AXE[0], AXE[1], 33.0, 121.5, 124.0 - 121.5, JOIN, [p])
    larg = tmx[0] - tmn[0]
    zcyl(c, 'bossage_moyeu', AXE[0], AXE[1], larg + 4.0, 123.9, H_TETE + EMPREINTE - 123.9, JOIN, [p])
    zbox(c, 'bossage_bras', AXE[0] - larg / 2 - 2.0, AXE[1], AXE[0] + larg / 2 + 2.0, tmx[1] + 2.0, 123.9, H_TETE + EMPREINTE - 123.9, JOIN, [p])
    m = adsk.core.Matrix3D.create()
    for i in range(3):
        m.setCell(i, i, ECHELLE)
    for i, v in enumerate((AXE[0], AXE[1], H_TETE)):
        m.setCell(i, 3, v / 10 * (1 - ECHELLE))
    bf = c.features.baseFeatures.add()
    bf.startEdit()
    for t in tete:
        c.bRepBodies.add(tbm.copy(t), bf)
        tt = tbm.copy(t)
        tbm.transform(tt, m)
        c.bRepBodies.add(tt, bf)
    bf.finishEdit()
    col = adsk.core.ObjectCollection.create()
    for x in bf.bodies:
        col.add(x)
    ci = c.features.combineFeatures.createInput(p, col)
    ci.operation = CUT
    c.features.combineFeatures.add(ci)
    p = c.bRepBodies.itemByName('palonnier_sg90')
    trous = set()
    for t in tete:
        for f in t.faces:
            g = f.geometry
            if g.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and abs(abs(g.axis.z) - 1) < 1e-3 and g.radius * 10 < 3.0:
                trous.add((round(g.origin.x * 10, 2), round(g.origin.y * 10, 2), round(g.radius * 10, 2)))
    for k, (x, y, r) in enumerate(sorted(trous)):
        zcyl(c, 'picot_%d' % k, x, y, 2 * r * ECHELLE + 1.0, H_TETE - 0.5, EMPREINTE + 1.0, CUT, [p])
    zcyl(c, 'vis_axe', AXE[0], AXE[1], 3.0, 105.0, H_TETE - 105.0 + 1, CUT, [p])
    p = c.bRepBodies.itemByName('palonnier_sg90')
    # ---- contrôles : rotation du palonnier et de la tête autour de l'axe
    contact = {}
    for ang in (0.0, -31.0, 31.0):
        r = adsk.core.Matrix3D.create()
        r.setToRotation(math.radians(ang), adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(AXE[0] / 10, AXE[1] / 10, 0))
        for nom, mob in [('palonnier', p)] + [('tete', t) for t in tete]:
            for nom2, fixe in [('planche', planche), ('support', support)] + [('SG90', s) for s in sg]:
                v = volume_commun(mob, fixe, r)
                if v > 0.01 and not (nom == 'tete' and nom2 == 'SG90'):
                    contact['%s/%s_%+d' % (nom, nom2, ang)] = v
    for nom2, fixe in (('planche', planche),):
        for s in sg + [support]:
            v = volume_commun(s, fixe)
            if v > 0.01:
                contact['%s/%s' % (s.name, nom2)] = v
    sortie = {'support': stl(support, DOSSIER + '/exports/stl/support_sg90.stl', ('x', 180)),
              'palonnier': stl(p, DOSSIER + '/exports/stl/palonnier_sg90.stl', None)}
    if doc.isSaved:
        doc.save('SG90 à la place du DT996 (22_servo_sg90.py)')
    else:
        doc.saveAs(DOC_SG90, dossier, 'SG90 de direction à la place du DT996 (dans le repère du rover)', '')
    rover.activate()
    print(json.dumps({'document': doc.name, 'oreilles': sorted(oreilles), 'sg90_bbox': [round(v, 1) for v in smn + smx],
                      'tete_bbox': [round(v, 1) for v in tmn + tmx], 'morceaux': [support.lumps.count, p.lumps.count],
                      'contacts_mm3': contact, 'stl_mm': sortie}))
