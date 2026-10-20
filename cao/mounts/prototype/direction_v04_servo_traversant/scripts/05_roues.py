from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Roues avant V04 : jante rigide (PETG) + pneu TPU 70A à crampons, 2 x 608ZZ dans le moyeu.
La vis de roue M8x30 est l'axe fixe : tête logée dans le moyeu (accès douille Ø24), elle traverse
les deux 608ZZ placés côté fusée et se visse dans l'écrou M8 prisonnier de la jambe ; la bague
intérieure du roulement intérieur appuie sur la portée Ø12 de la fusée.

Repère local de la roue : origine sur l'axe de roue, au droit de la face de la jambe de fusée
(x = 0), x vers l'extérieur (×s). La roue occupe x ∈ [1,5 ; 1,5 + d4_roue_larg].
Paramètres : d4_roue_av_d, d4_roue_larg, d4_pneu_ep, d4_crampon_h, d4_crampon_n, d4_pneu_serrage, d4_pneu_jeu.
Pneu démontable : serrage de 1 mm au diamètre, rebords de 2 mm, chanfrein extérieur pour le monter.
Crampons trapézoïdaux biseautés à 45° aux deux bouts : le pneu s'imprime à plat sans support.
"""
import adsk.core, adsk.fusion, math
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v04_servo_traversant/scripts/01_construire.py'), encoding='utf-8').read(), '01_construire.py', 'exec'))

D = 'd4_roue_av_d'
DJ = '(d4_roue_av_d - 2 * d4_pneu_ep)'   # diamètre de portée du pneu sur la jante (Ø120 pour Ø150 et 15 mm)
W = 'd4_roue_larg'
PARAMS_ROUE = [
    ('d4_roue_larg', '35 mm', 'Largeur de roue (jante et pneu)'),
    ('d4_pneu_ep', '15 mm', 'Epaisseur radiale totale du pneu TPU, crampons compris (15 maxi)'),
    ('d4_crampon_h', '3 mm', 'Hauteur des crampons (compris dans d4_pneu_ep)'),
    ('d4_pneu_serrage', '1 mm', 'Serrage diametral du pneu sur la jante (Ø interieur pneu = Ø jante - serrage)'),
    ('d4_pneu_jeu', '0.2 mm', 'Jeu lateral entre pneu et chaque rebord de jante'),
    ('d4_crampon_n', '24', 'Nombre de crampons par rangee (2 rangees decalees)'),
]


def xplane(c, s, expr, name):
    i = c.constructionPlanes.createInput()
    i.setByOffset(c.yZConstructionPlane, adsk.core.ValueInput.createByString(expr if s > 0 else '-(%s)' % expr))
    p = c.constructionPlanes.add(i)
    p.name = 'plan_' + name
    p.isLightBulbOn = False
    return p


def anneau_x(c, s, name, d_out, d_in, x0, length, op=NEW, bodies=None):
    """Anneau d'axe X : diamètres et positions en expressions, extrudé de `length` vers l'extérieur."""
    sk = sketch(c, xplane(c, s, x0, name), name)
    o = sk.originPoint
    for dexpr in (d_out, d_in):
        if dexpr is None:
            continue
        cc = sk.sketchCurves.sketchCircles.addByCenterRadius(o.geometry, ev(dexpr) / 20)
        sk.geometricConstraints.addCoincident(cc.centerSketchPoint, o)
        sk.sketchDimensions.addDiameterDimension(cc, adsk.core.Point3D.create(ev(dexpr) / 25, ev(dexpr) / 25, 0)).parameter.expression = dexpr
    prof = profiles(sk, 'ring') if d_in else profiles(sk)
    f = extrude(c, prof, length, op, 'extr_' + name, bodies, down=s < 0)
    return f


def bloc_radial(c, s, name, x0, length, r_in, r_out, largeur, angle_expr, bodies, op=JOIN):
    """Bloc radial (crampon) : rectangle dans le plan YZ, centré sur une demi-droite faisant
    angle_expr avec +Z (None = vertical), entre les rayons r_in et r_out (expressions)."""
    sk = sketch(c, xplane(c, s, x0, name), name)
    gc, dims = sk.geometricConstraints, sk.sketchDimensions
    o = sk.originPoint
    up = sk.modelToSketchSpace(P(0, 0, 10))
    a = 0.0 if angle_expr is None else ev_angle(angle_expr)
    # direction de la demi-droite dans l'esquisse
    base = adsk.core.Vector3D.create(up.x - o.geometry.x, up.y - o.geometry.y, 0)
    base.normalize()
    ca, sa = math.cos(a), math.sin(a)
    sgn = 1.0
    dirv = (base.x * ca - sgn * base.y * sa, base.x * sa + sgn * base.y * ca)
    rin, rout = ev(r_in), ev(r_out)
    L = sk.sketchCurves.sketchLines
    ax = L.addByTwoPoints(o.geometry, adsk.core.Point3D.create(dirv[0] * rout / 10, dirv[1] * rout / 10, 0))
    ax.isConstruction = True
    gc.addCoincident(ax.startSketchPoint, o)
    if angle_expr is None:
        (gc.addVertical if abs(dirv[0]) < abs(dirv[1]) else gc.addHorizontal)(ax)
    else:
        ref = L.addByTwoPoints(o.geometry, adsk.core.Point3D.create(base.x * rout / 10, base.y * rout / 10, 0))
        ref.isConstruction = True
        gc.addCoincident(ref.startSketchPoint, o)
        (gc.addVertical if abs(base.x) < abs(base.y) else gc.addHorizontal)(ref)
        dims.addAngularDimension(ref, ax, adsk.core.Point3D.create(dirv[0] * rout / 20 + base.x * rout / 20, dirv[1] * rout / 20 + base.y * rout / 20, 0)).parameter.expression = angle_expr
    dims.addDistanceDimension(ax.startSketchPoint, ax.endSketchPoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                              adsk.core.Point3D.create(dirv[0] * rout / 20 + 0.5, dirv[1] * rout / 20, 0)).parameter.expression = r_out
    pin = sk.sketchPoints.add(adsk.core.Point3D.create(dirv[0] * rin / 10, dirv[1] * rin / 10, 0))
    gc.addCoincident(pin, ax)
    dims.addDistanceDimension(o, pin, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
                              adsk.core.Point3D.create(dirv[0] * rin / 20 - 0.5, dirv[1] * rin / 20, 0)).parameter.expression = r_in
    n = (-dirv[1], dirv[0])
    h = largeur / 20
    def pt(r, k):
        return adsk.core.Point3D.create(dirv[0] * r / 10 + k * n[0] * h, dirv[1] * r / 10 + k * n[1] * h, 0)
    l_in = L.addByTwoPoints(pt(rin, -1), pt(rin, 1))
    l_out = L.addByTwoPoints(pt(rout, -1), pt(rout, 1))
    l_a = L.addByTwoPoints(l_in.startSketchPoint, l_out.startSketchPoint)
    l_b = L.addByTwoPoints(l_in.endSketchPoint, l_out.endSketchPoint)
    gc.addMidPoint(pin, l_in)
    gc.addMidPoint(ax.endSketchPoint, l_out)
    gc.addPerpendicular(l_in, ax)
    gc.addPerpendicular(l_out, ax)
    dims.addDistanceDimension(l_in.startSketchPoint, l_in.endSketchPoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, pt(rin - 2, 0)).parameter.expression = '%r mm' % largeur
    dims.addDistanceDimension(l_out.startSketchPoint, l_out.endSketchPoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, pt(rout + 2, 0)).parameter.expression = '%r mm' % largeur
    prof = profiles(sk, lambda p: True)
    f = extrude(c, prof, length, op, 'extr_' + name, bodies, down=s < 0)
    return f


def crampon(c, s, name, xs, xe, angle_expr, bodies):
    """Crampon trapézoïdal imprimable sans support : profil dans le plan (axe, rayon) avec les deux
    extrémités biseautées à 45° sur toute la hauteur d4_crampon_h, extrudé de ±3,5 mm de part et
    d'autre du plan (crampon de 7 mm de large). Le plan passe par l'axe de roue, à angle_expr de +Z."""
    if angle_expr is None:
        pl = c.xZConstructionPlane
    else:
        i = c.constructionPlanes.createInput()
        i.setByAngle(c.xConstructionAxis, adsk.core.ValueInput.createByString(angle_expr), c.xZConstructionPlane)
        pl = c.constructionPlanes.add(i)
        pl.name = 'plan_' + name
        pl.isLightBulbOn = False
    sk = sketch(c, pl, name)
    nrm = pl.geometry.normal
    X = adsk.core.Vector3D.create(1, 0, 0)
    rdir = X.crossProduct(nrm)
    rdir.normalize()
    if rdir.z < 0:
        rdir.scaleBy(-1)
    h = 'd4_crampon_h'
    r_out = '%s / 2' % D
    r_surf = '%s / 2 - %s - 0.1 mm' % (D, h)          # départ du biseau juste sous la surface du pneu
    r_in = '%s / 2 - %s - 0.5 mm' % (D, h)
    hb = '(%s + 0.1 mm)' % h                           # biseau à 45° sur toute la hauteur
    pts = [(xs, r_in), (xe, r_in), (xe, r_surf), ('%s - %s' % (xe, hb), r_out), ('%s + %s' % (xs, hb), r_out), (xs, r_surf)]
    def model(x, r):
        xv, rv = ev(x) / 10, ev(r) / 10
        return adsk.core.Point3D.create(s * xv, rv * rdir.y, rv * rdir.z)
    sk_pts = [sk.modelToSketchSpace(model(x, r)) for x, r in pts]
    o = sk.originPoint
    ax = sk.modelToSketchSpace(adsk.core.Point3D.create(s * 1.0, 0, 0))
    axial_horiz = abs(ax.x - o.geometry.x) > abs(ax.y - o.geometry.y)
    L = sk.sketchCurves.sketchLines
    lines = []
    prev = None
    for k in range(6):
        a = prev if prev is not None else sk_pts[0]
        b = sk_pts[k + 1] if k < 5 else lines[0].startSketchPoint
        ln = L.addByTwoPoints(a, b)
        lines.append(ln)
        prev = ln.endSketchPoint
    # Traits axiaux (0 et 3) et radiaux (1 et 5) contraints explicitement ; chaque coordonnée
    # n'est cotée qu'une fois : P0 (axial+radial), P1 axial, P2 radial, P3 (axial+radial), P4 axial, P5 radial.
    gc = sk.geometricConstraints
    axial_c, radial_c = (gc.addHorizontal, gc.addVertical) if axial_horiz else (gc.addVertical, gc.addHorizontal)
    axial_c(lines[0])
    axial_c(lines[3])
    radial_c(lines[1])
    radial_c(lines[5])
    dims = sk.sketchDimensions
    H_ = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
    V_ = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
    OA, OR = (H_, V_) if axial_horiz else (V_, H_)
    cotes = {0: ('a', 'r'), 1: ('a',), 2: ('r',), 3: ('a', 'r'), 4: ('a',), 5: ('r',)}
    for k, (x, r) in enumerate(pts):
        sp_ = lines[k].startSketchPoint
        g = sp_.geometry
        if 'a' in cotes[k]:
            dims.addDistanceDimension(o, sp_, OA, adsk.core.Point3D.create(g.x + 0.3, g.y + 0.3, 0)).parameter.expression = x
        if 'r' in cotes[k]:
            dims.addDistanceDimension(o, sp_, OR, adsk.core.Point3D.create(g.x - 0.3, g.y - 0.3, 0)).parameter.expression = r
    i = c.features.extrudeFeatures.createInput(profiles(sk), JOIN)
    i.setSymmetricExtent(adsk.core.ValueInput.createByString('3.5 mm'), False)
    i.participantBodies = list(bodies)
    f = c.features.extrudeFeatures.add(i)
    f.name = 'extr_' + name
    return f


def ev_angle(expr):
    return design().unitsManager.evaluateExpression(expr, 'rad')


def motif(c, feats, n_expr, name):
    col = adsk.core.ObjectCollection.create()
    for f in feats:
        col.add(f)
    i = c.features.circularPatternFeatures.createInput(col, c.xConstructionAxis)
    i.quantity = adsk.core.ValueInput.createByString(n_expr)
    i.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    i.isSymmetric = False
    i.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
    p = c.features.circularPatternFeatures.add(i)
    p.name = 'motif_' + name
    return p


def aretes_axiales(b, rayon_mm, tol=0.6):
    """Arêtes droites parallèles à l'axe de roue (X) situées à rayon_mm de l'axe."""
    out = adsk.core.ObjectCollection.create()
    for e in b.edges:
        g = e.geometry
        if g.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
            continue
        ln = adsk.core.Line3D.cast(g)
        d = ln.startPoint.vectorTo(ln.endPoint)
        if abs(d.y) > 1e-6 or abs(d.z) > 1e-6:
            continue
        r = math.hypot(ln.startPoint.y, ln.startPoint.z) * 10
        if abs(r - rayon_mm) < tol:
            out.add(e)
    return out


def conge(c, edges, r, name):
    i = c.features.filletFeatures.createInput()
    try:
        i.edgeSetInputs.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByString(r), False)
    except AttributeError:
        i.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByString(r), False)
    f = c.features.filletFeatures.add(i)
    f.name = name
    return f


def conges_rayons(c, s, jante):
    """Congés 3 mm rayon/moyeu et 5 mm rayon/fût (arêtes axiales des 6 rayons)."""
    rh = aretes_axiales(jante, 16.0)
    rb = aretes_axiales(jante, ev('%s / 2 - 3 mm' % DJ))
    print('conges', rh.count, rb.count)
    if rh.count:
        conge(c, rh, '3 mm', 'conges_rayons_moyeu')
    rb = aretes_axiales(jante, ev('%s / 2 - 3 mm' % DJ))
    if rb.count:
        conge(c, rb, '5 mm', 'conges_rayons_fut')


def rampe_rebord_int(c, s, jante):
    """Rampe à 45° (2 mm) entre la portée du pneu et le rebord intérieur : supprime le surplomb
    de ce rebord quand la jante est imprimée face extérieure sur le plateau."""
    rj = ev(DJ) / 20
    col = adsk.core.ObjectCollection.create()
    for e in jante.edges:
        g = e.geometry
        if g.curveType == adsk.core.Curve3DTypes.Circle3DCurveType and abs(adsk.core.Circle3D.cast(g).radius - rj) < 0.01:
            x = adsk.core.Circle3D.cast(g).center.x * 10 * s
            if abs(x - 3.5) < 0.05:
                col.add(e)
    i = c.features.chamferFeatures.createInput2()
    i.chamferEdgeSets.addEqualDistanceChamferEdgeSet(col, adsk.core.ValueInput.createByString('2 mm'), False)
    c.features.chamferFeatures.add(i).name = 'rampe_rebord_interieur'
    print('rampe', col.count)


def chanfreins(c, s, jante, pneu):
    """Chanfrein 1,5 mm sur l'arête extérieure du rebord extérieur (montage du pneu) et
    chanfreins 2 mm sur les deux arêtes intérieures du pneu (épousent la rampe du rebord intérieur)."""
    def cercles(b):
        for e in b.edges:
            g = e.geometry
            if g.curveType == adsk.core.Curve3DTypes.Circle3DCurveType:
                yield e, adsk.core.Circle3D.cast(g)
    rj = ev('%s + 4 mm' % DJ) / 20
    cand = [e for e, g in cercles(jante) if abs(g.radius - rj) < 0.01]
    if not cand:
        raise RuntimeError('arete rebord introuvable, rayons : %s' % sorted({round(g.radius * 10, 2) for e, g in cercles(jante)}))
    ext = max(cand, key=lambda e: s * (e.boundingBox.maxPoint.x + e.boundingBox.minPoint.x))
    col = adsk.core.ObjectCollection.create()
    col.add(ext)
    i = c.features.chamferFeatures.createInput2()
    i.chamferEdgeSets.addEqualDistanceChamferEdgeSet(col, adsk.core.ValueInput.createByString('1.5 mm'), False)
    c.features.chamferFeatures.add(i).name = 'chanfrein_montage_pneu'
    rp = ev('%s - d4_pneu_serrage' % DJ) / 20
    col2 = adsk.core.ObjectCollection.create()
    for e, g in cercles(pneu):
        if abs(g.radius - rp) < 0.01:
            col2.add(e)
    i2 = c.features.chamferFeatures.createInput2()
    i2.chamferEdgeSets.addEqualDistanceChamferEdgeSet(col2, adsk.core.ValueInput.createByString('2 mm'), False)
    c.features.chamferFeatures.add(i2).name = 'chanfreins_pneu'
    print('chanfreins', col.count, col2.count)


def roue(s, tag):
    o, c = part('D4_IMP_Roue_' + tag)
    # jante : moyeu, 6 rayons, fût, rebords (rampe à 45° côté intérieur)
    j = body(anneau_x(c, s, 'moyeu', '32 mm', None, '1.5 mm', W), 'jante_' + tag)
    J = [j]
    # 6 rayons pleine largeur (parois verticales à l'impression : rapides et rigides), congés aux raccords
    fr = bloc_radial(c, s, 'rayon', '1.5 mm', W, '13 mm', '%s / 2 - 2 mm' % DJ, 4.5, None, J)
    motif(c, [fr], '6', 'rayons')
    anneau_x(c, s, 'fut', DJ, '%s - 6 mm' % DJ, '1.5 mm', W, JOIN, J)
    anneau_x(c, s, 'rebord_int', '%s + 4 mm' % DJ, '%s - 1 mm' % DJ, '1.5 mm', '2 mm', JOIN, J)
    anneau_x(c, s, 'rebord_ext', '%s + 4 mm' % DJ, '%s - 1 mm' % DJ, '%s - 0.5 mm' % W, '2 mm', JOIN, J)
    # roulements 608ZZ : logements des deux côtés, arête centrale sur bagues extérieures
    anneau_x(c, s, 'siege_608_int', '22.15 mm', None, '1.5 mm', '7 mm', CUT, J)
    anneau_x(c, s, 'siege_608_ext', '22.15 mm', None, '10.5 mm', '7 mm', CUT, J)
    anneau_x(c, s, 'passage_moyeu', '17 mm', None, '1 mm', '10 mm', CUT, J)
    # au-delà des roulements : alésage Ø24 pour la douille de 13 (tête de vis logée dans le moyeu)
    anneau_x(c, s, 'acces_douille', '24 mm', None, '17.5 mm', '%s' % W, CUT, J)
    conges_rayons(c, s, j)
    rampe_rebord_int(c, s, j)
    # pneu TPU entre les rebords
    p = body(anneau_x(c, s, 'pneu', '%s - 2 * d4_crampon_h' % D, '%s - d4_pneu_serrage' % DJ, '3.5 mm + d4_pneu_jeu', '%s - 4 mm - 2 * d4_pneu_jeu' % W), 'pneu_TPU_' + tag, 'FM_NOIR')
    Pn = [p]
    lg = '((%s - 4 mm) / 2 - 0.5 mm - d4_pneu_jeu)' % W
    xa = '(3.5 mm + d4_pneu_jeu)'
    xb = '(3.5 mm + (%s - 4 mm) / 2 + 0.5 mm)' % W
    fa = crampon(c, s, 'crampon_A', xa, '(%s + %s)' % (xa, lg), None, Pn)
    fb = crampon(c, s, 'crampon_B', xb, '(%s + %s)' % (xb, lg), '180 deg / d4_crampon_n', Pn)
    motif(c, [fa, fb], 'd4_crampon_n', 'crampons')
    chanfreins(c, s, j, p)
    # pièces associées : entretoise imprimée entre bagues intérieures, rondelle imprimée sous la tête
    body(anneau_x(c, s, 'entretoise_roue', '12 mm', '8.4 mm', '8.5 mm', '2 mm'), 'rondelle_2_roue_' + tag)
    body(anneau_x(c, s, 'rondelle_tete', '12 mm', '8.4 mm', '17.5 mm', '2 mm'), 'rondelle_2_tete_roue_' + tag)
    # références : 608ZZ x 2, vis H M8 (axe de roue)
    body(anneau_x(c, s, 'REF_608ZZ_int', '22 mm', '8 mm', '1.5 mm', '7 mm'), 'REF_608ZZ_roue_int_' + tag, 'FM_METAL')
    body(anneau_x(c, s, 'REF_608ZZ_ext', '22 mm', '8 mm', '10.5 mm', '7 mm'), 'REF_608ZZ_roue_ext_' + tag, 'FM_METAL')
    vb = body(anneau_x(c, s, 'REF_vis_roue', 'd4_vis_roue', None, '-10.5 mm', '30 mm'), 'REF_vis_H_M8x30_roue_' + tag, 'FM_METAL')
    anneau_x(c, s, 'REF_tete_vis_roue', '13 mm', None, '19.5 mm', '5.3 mm', JOIN, [vb])
    # point d'ancrage sur l'axe, dans le plan YZ (même orientation que le point de la fusée)
    ska = sketch(c, c.yZConstructionPlane, 'ancrage_axe')
    ska.sketchPoints.add(ska.originPoint.geometry)
    return o


def ancrage(o):
    sk = next(x for x in o.component.sketches if x.name == 'esq_ancrage_axe')
    return point_geo(sk.sketchPoints.item(sk.sketchPoints.count - 1), o)


def run(_context: str):
    for n, e, cmt in PARAMS_ROUE:
        param(n, e, 'V04 roues | ' + cmt, '' if n == 'd4_crampon_n' else 'mm')
    g = group().component
    for s, tag in ((+1, 'D'), (-1, 'G')):
        of = occ('D4_IMP_Fusee_' + tag)
        skp = next(x for x in of.component.sketches if x.name == 'esq_portee_roue')
        ctr = min((p for p in skp.sketchPoints), key=lambda p: skp.sketchToModelSpace(p.geometry).z)
        orr = roue(s, tag)
        ji = g.joints.createInput(ancrage(orr), point_geo(ctr, of))
        ji.setAsRigidJointMotion()
        j = g.joints.add(ji)
        j.name = 'D4_roue_sur_fusee_' + tag
        bb = next(x for x in group().childOccurrences if x.component.name == orr.component.name).boundingBox
        print(tag, [round(v * 10, 1) for v in bb.minPoint.asArray()], [round(v * 10, 1) for v in bb.maxPoint.asArray()])
