from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Incrustations de couleur sur la coque, en place (composant PF_IMP_Coque du document du rover) :
- FIELDMIND sur les deux flancs du tronçon 3 (à la place de l'inscription gravée de 0,8 mm) ;
Chaque texte est une poche de 1,2 mm (la moitié de la paroi de 2,4 : la paroi reste pleine et l'intérieur
des lettres tient au fond) et un insert de la couleur opposée, de la forme exacte des lettres, à coller
dans la poche : lettres bleues sur les flancs blancs. Imprimer les
inserts à plat avec une compensation XY de -0,1 mm (« X-Y contour compensation ») pour qu'ils entrent.
Un ASUS sur la bande C2 a été essayé puis retiré (pas beau) ; ASUS = (x, y, hauteur) le referait.
Réexécutable. Exporte les STL des inserts.
"""
import adsk.core, adsk.fusion, json, math
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/04_exporter.py'), encoding='utf-8').read(), '04_exporter.py', 'exec'))
DOSSIER = str(_FM_ROOT / 'cao/prototype_final')
XE, H, POCHE = 150.0, 85.0, 1.2
FIELDMIND = (229.5, ZB + 47.5, 1.8)       # centre du mot (y), bas des lettres (z), boîte de texte (cm) : lettres de 13 mm entre les ouïes (ZB+44) et le chanfrein (ZB+65)
ASUS = (0.0, 132.0, 2.4)                   # centre (x, y) à l'arrière de la bande C2, derrière la barre des caméras ; lettres de 18 mm
NOMS = ('texte_G', 'texte_D', 'incrust_')


def nettoyer(c):
    for coll in (c.features.extrudeFeatures, c.sketches, c.constructionPlanes):
        for x in reversed([coll.item(i) for i in range(coll.count)]):
            n = x.name
            if x.isValid and (n in ('texte_G', 'texte_D', 'esq_texte_G', 'esq_texte_D', 'plan_texte_G', 'plan_texte_D') or 'incrust_' in n):
                x.deleteMe()
    for b in list(c.bRepBodies):
        if b.isValid and b.name.startswith('incrustation_'):
            b.deleteMe()


def texte(s, mot, hauteur, ancre, angle=0.0, miroir=False):
    ti = s.sketchTexts.createInput(mot, hauteur, ancre)
    ti.angle = angle
    ti.fontName = 'Arial Black'
    if miroir:
        ti.isHorizontalFlip = True
    return s.sketchTexts.add(ti)


def extruder(c, st, op, sens_negatif, nom, bodies=None):
    ei = c.features.extrudeFeatures.createInput(st, op)
    ei.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(V(POCHE)),
                        adsk.fusion.ExtentDirections.NegativeExtentDirection if sens_negatif else adsk.fusion.ExtentDirections.PositiveExtentDirection)
    if bodies is not None:
        ei.participantBodies = bodies
    f = c.features.extrudeFeatures.add(ei)
    f.name = nom
    return f


def incruster(c, s, mot, hauteur, ancre_fn, angle, miroir, sens_negatif, paroi, nom, couleur):
    """Crée le texte, le recentre (deux passes), puis l'insert (corps neuf) et la poche dans la paroi."""
    st = texte(s, mot, hauteur, ancre_fn(0.0), angle, miroir)
    bb = st.boundingBox
    larg = max(bb.maxPoint.x - bb.minPoint.x, bb.maxPoint.y - bb.minPoint.y) * 10
    st.deleteMe()
    st = texte(s, mot, hauteur, ancre_fn(larg), angle, miroir)
    f = extruder(c, st, adsk.fusion.FeatureOperations.NewBodyFeatureOperation, sens_negatif, 'extr_incrust_' + nom)
    ins = f.bodies.item(0)
    for k in range(1, f.bodies.count):          # un corps par lettre : on les regroupe sous un nom
        f.bodies.item(k).name = 'incrustation_%s_%d' % (nom, k)
        paint(f.bodies.item(k), couleur)
    ins.name = 'incrustation_%s_0' % nom
    paint(ins, couleur)
    extruder(c, st, CUT, sens_negatif, 'extr_incrust_poche_' + nom, [c.bRepBodies.itemByName(paroi)])   # référence relue après la création des inserts
    return [f.bodies.item(k) for k in range(f.bodies.count)], larg


def run(_context: str):
    d = design()
    oc = next(o for o in group().childOccurrences if o.component.name == 'PF_IMP_Coque')
    c = oc.component
    nettoyer(c)
    P_ = {b.name: b for b in c.bRepBodies}
    out = {}
    lettres = {}
    # FIELDMIND sur les flancs (même orientation de lecture que l'ancienne gravure de 02_coque.py)
    for sx, cote in ((1, 'G'), (-1, 'D')):
        s = sketch(c, plane_x(c, sx * XE, 'incrust_texte_%s' % cote), 'incrust_texte_%s' % cote)
        o_ = s.sketchToModelSpace(adsk.core.Point3D.create(0, 0, 0))
        ux = s.sketchToModelSpace(adsk.core.Point3D.create(1, 0, 0))
        uy = s.sketchToModelSpace(adsk.core.Point3D.create(0, 1, 0))
        ex_ = [ux.asArray()[i] - o_.asArray()[i] for i in range(3)]
        ey_ = [uy.asArray()[i] - o_.asArray()[i] for i in range(3)]
        lect, haut = (0.0, float(sx), 0.0), (0.0, 0.0, 1.0)
        nrm = [ex_[1] * ey_[2] - ex_[2] * ey_[1], ex_[2] * ey_[0] - ex_[0] * ey_[2], ex_[0] * ey_[1] - ex_[1] * ey_[0]]
        lxh = (lect[1] * haut[2] - lect[2] * haut[1], lect[2] * haut[0] - lect[0] * haut[2], lect[0] * haut[1] - lect[1] * haut[0])
        miroir = sum(a * b for a, b in zip(nrm, lxh)) < 0
        base = tuple(-v for v in lect) if miroir else lect
        ang = math.atan2(sum(a * b for a, b in zip(base, ey_)), sum(a * b for a, b in zip(base, ex_)))
        yc, zb, hh = FIELDMIND
        ancre = lambda larg, sx=sx, s=s: s.modelToSketchSpace(P(sx * XE, yc - sx * larg / 2, zb))
        lettres[cote], larg = incruster(c, s, 'FIELDMIND', hh, ancre, ang, miroir, sx > 0, 'coque_%s3' % cote,
                                        'FIELDMIND_' + cote, 'FM_PLA_bleu')
        out['FIELDMIND_' + cote] = {'lettres': len(lettres[cote]), 'longueur_mm': round(larg, 1)}
    # STL des inserts, posés à plat (face visible contre le plateau)
    for nom, rot in (('FIELDMIND_G', ('y', 90)), ('FIELDMIND_D', ('y', -90))):
        cote = nom.split('_')[-1] if nom.startswith('FIELDMIND') else 'ASUS'
        tbm = adsk.fusion.TemporaryBRepManager.get()
        tout = None
        for b in lettres[cote]:
            t = tbm.copy(b)
            if tout is None:
                tout = t
            else:
                tbm.booleanOperation(tout, t, adsk.fusion.BooleanTypes.UnionBooleanType)
        # le maillage d'un corps temporaire passe par un corps de base temporaire : on exporte lettre par lettre réunies
        bf = c.features.baseFeatures.add()
        bf.startEdit()
        tmp = c.bRepBodies.add(tout, bf)
        bf.finishEdit()
        b_tmp = bf.bodies.item(0)
        out[nom]['stl_mm'] = stl(b_tmp, DOSSIER + '/exports/stl/incrustation_%s.stl' % nom, rot)
        bf.deleteMe()
    for cote in ('G', 'D'):
        out['vol_paroi_%s3_cm3' % cote] = round(c.bRepBodies.itemByName('coque_%s3' % cote).volume, 1)
    tl = d.timeline
    out['fonctions_KO'] = [tl.item(i).name for i in range(max(0, tl.count - 40), tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup]
    print(texte_json(out) if 'texte_json' in globals() else json.dumps(out))
