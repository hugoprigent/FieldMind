from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Empreinte du palonnier 1 bras du SG90 dans la ferrure avant de chaque volet, en place dans le document
de la remorque : une plaque de 1,5 mm devant la face de la ferrure (côté servo, dans les 5,5 mm libres
avant le boîtier), creusée de la forme exacte du palonnier (modèle FreeCAD), agrandie de 4 % autour du
moyeu pour le jeu d'impression. Le palonnier s'y encastre comme l'étoile dans le palonnier de la
direction, sans picots (les trous du palonnier sont rasés), et tient par la vis M2,5 du servo passée
par le trou Ø3 dans l'axe, à travers la ferrure. Réexécutable.
Ensuite : 03_exporter.py (STL des ferrures), 07_enregistrer_remorque.py.
"""
import adsk.core, adsk.fusion, json, math
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/remorque/scripts/rm_helpers.py'), encoding='utf-8').read(), 'rm_helpers.py', 'exec'))
PLAQUE, MARGE, ECHELLE = 1.5, 2.5, 1.04   # plaque de l'épaisseur du bras : 0,6 mm devant le boîtier du SG90
R_AXE, BAS, HAUT = 4.5, 5.0, 6.0          # côté axe : arrondi de 4,5 (la planche est à 5 mm de l'axe quand le volet se replie) ;
                                          # sous l'axe 5 mm (le boîtier du servo est à 5,5), au-dessus 6


def run(_context: str):
    a = app()
    doc = next(d_ for d_ in a.documents if d_.name.startswith(DOC_REMORQUE))
    doc.activate()
    d = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    tbm = adsk.fusion.TemporaryBRepManager.get()
    out = {}
    for sx, cote in ((1, 'G'), (-1, 'D')):
        c = next(x for x in d.allComponents if x.name == 'RM_IMP_Volet_' + cote)
        for coll in (c.features.combineFeatures, c.features.baseFeatures, c.features.extrudeFeatures, c.sketches, c.constructionPlanes):
            for x in reversed([coll.item(i) for i in range(coll.count)]):
                if x.isValid and 'empreinte_pal' in x.name:
                    x.deleteMe()
        f = c.bRepBodies.itemByName('ferrure_AV_' + cote)
        pal = [b for b in c.bRepBodies if b.name.startswith('REF_palonnier_SG90_' + cote)]
        mn = [min(b.boundingBox.minPoint.asArray()[i] for b in pal) * 10 for i in range(3)]
        mx = [max(b.boundingBox.maxPoint.asArray()[i] for b in pal) * 10 for i in range(3)]
        # plaque devant la face de la ferrure, sur l'emprise du palonnier (côté servo : y < Y_PAL)
        s = sketch(c, plane_y(c, Y_PAL, 'empreinte_pal_' + cote), 'empreinte_pal_' + cote)
        bout = (mx[0] if sx > 0 else -mn[0]) + MARGE         # bout du bras, compté vers l'extérieur
        pts = [(X_AX + R_AXE * math.cos(math.radians(a_)), R_AXE * math.sin(math.radians(a_))) for a_ in range(90, 271, 15)]
        pts += [(X_AX + 8.0, -BAS), (bout, -BAS), (bout, HAUT), (X_AX + 8.0, HAUT)]
        poly3(s, [(sx * u, Y_PAL, Z_AX + w) for u, w in pts])
        ei = c.features.extrudeFeatures.createInput(profiles(s), JOIN)
        ei.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(V(PLAQUE)), adsk.fusion.ExtentDirections.PositiveExtentDirection)
        ei.participantBodies = [f]
        e1 = c.features.extrudeFeatures.add(ei)
        e1.name = 'extr_empreinte_pal_plaque_' + cote
        if f.boundingBox.minPoint.y * 10 > Y_PAL - PLAQUE + 0.1:     # mauvais sens : on repart de l'autre côté
            e1.deleteMe()
            ei = c.features.extrudeFeatures.createInput(profiles(s), JOIN)
            ei.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(V(PLAQUE)), adsk.fusion.ExtentDirections.NegativeExtentDirection)
            ei.participantBodies = [f]
            c.features.extrudeFeatures.add(ei).name = 'extr_empreinte_pal_plaque_' + cote
        # outil : le palonnier agrandi de 4 % autour de son moyeu (axe de la charnière, face de la ferrure)
        m = adsk.core.Matrix3D.create()
        cx, cy, cz = sx * X_AX, Y_PAL, Z_AX
        for i, v in enumerate((ECHELLE, ECHELLE, ECHELLE)):
            m.setCell(i, i, v)
        m.setCell(0, 3, cx / 10 * (1 - ECHELLE))
        m.setCell(1, 3, cy / 10 * (1 - ECHELLE))
        m.setCell(2, 3, cz / 10 * (1 - ECHELLE))
        bf = c.features.baseFeatures.add()
        bf.name = 'empreinte_pal_outil_' + cote
        bf.startEdit()
        outils = []
        for b in pal:                          # palonnier exact et palonnier agrandi : rien ne reste dans la ferrure
            outils.append(c.bRepBodies.add(tbm.copy(b), bf))
            t = tbm.copy(b)
            tbm.transform(t, m)
            outils.append(c.bRepBodies.add(t, bf))
        bf.finishEdit()
        col = adsk.core.ObjectCollection.create()
        for b in bf.bodies:
            col.add(b)
        ci = c.features.combineFeatures.createInput(f, col)
        ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        ci.isKeepToolBodies = False
        c.features.combineFeatures.add(ci).name = 'empreinte_pal_coupe_' + cote
        f = c.bRepBodies.itemByName('ferrure_AV_' + cote)
        # pas de picots : les trous du palonnier (bras et moyeu) laissaient des picots dans l'empreinte, on les rase
        trous = set()
        for b in pal:
            for fa in b.faces:
                g = fa.geometry
                if g.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and abs(abs(g.axis.y) - 1) < 1e-3 and g.radius * 10 < 3.0:
                    trous.add((round(g.origin.x * 10, 2), round(g.origin.z * 10, 2), round(g.radius * 10, 2)))
        for k, (x, z, r) in enumerate(sorted(trous)):
            cyl_y(c, 'empreinte_pal_picot_%s_%d' % (cote, k), x, z, 2 * r * ECHELLE + 1.0, Y_PAL - PLAQUE - 0.5, Y_PAL + 0.01, CUT, [f])
        # trou de la vis du palonnier (M2,5) dans l'axe du servo, à travers toute la ferrure
        cyl_y(c, 'empreinte_pal_vis_axe_' + cote, sx * X_AX, Z_AX, 3.0, Y_PAL - PLAQUE - 1, FERR_AV[1] + 1, CUT, [f])
        f = c.bRepBodies.itemByName('ferrure_AV_' + cote)
        out['picots_' + cote] = len(trous)
        bb = f.boundingBox
        out[cote] = {'palonnier_mm': [round(mx[i] - mn[i], 1) for i in range(3)], 'ferrure_morceaux': f.lumps.count,
                     'ferrure_y': [round(bb.minPoint.y * 10, 1), round(bb.maxPoint.y * 10, 1)], 'Y_PAL': Y_PAL}
    tl = d.timeline
    out['KO'] = [tl.item(i).name for i in range(max(0, tl.count - 30), tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup]
    print(json.dumps(out))
