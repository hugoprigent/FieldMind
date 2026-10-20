from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Capteurs sur la planche : deux mâts en profilé 40 x 40 type B (rainure 10), plus les perçages de la planche.

- Mât lidar : profilé vertical sur l'axe des roues arrière. Le support imprimé s'emboîte sur le
  haut du profilé (languettes dans les 4 rainures, 2 vis M8 latérales dans des écrous en T M8)
  et porte le RPLIDAR C1 par ses 4 trous M2,5 sur 43 x 43, vissés par-dessous à travers des
  puits dans les coins de la jupe.
- Mât caméras : profilé vertical au bord avant. Une barre imprimée s'emboîte sur son haut de la
  même façon et porte les deux webcams UGREEN à d4_stereo_base d'écartement, axes parallèles,
  inclinées de d4_cam_inclinaison vers le sol, centre optique à ~d4_cam_optique_sol du sol (pose
  de la caméra simulée : 0,269 m et -10°). L'emboîtement est incliné dans la barre : la face
  d'appui des caméras est plane et s'imprime sur le plateau. Une vis photo par caméra, par-dessous.
- Chaque profilé tient sur la planche par 2 équerres 40 (gauche et droite), vis M8 ; la géométrie
  des équerres est celle d'une équerre 40 x 40 du commerce : 38 de large, ailes de 36, trou Ø8,5
  à 20 mm de la face du profilé.
- Planche : perçages des supports de pivot, des supports moteurs, des oreilles du servo et des
  équerres.
Réexécutable : supprime d'abord les composants et perçages d'un passage précédent.

Repères locaux (avant = -Y, droite = +X, Z vers le haut) :
- profilés, équerres, barre stéréo : axe du profilé, sur le dessus de la planche ;
- support lidar : centre du haut du profilé (fond de l'emboîtement) ;
- C1 : repère du STEP Slamtec (base centrée, dessous à Z = 0, 0° vers +X, câble vers -X), tourné
  pour que son 0° regarde l'avant du rover et que le câble parte vers l'arrière ;
- webcam : axe de la vis sur la face d'appui du pied ; Z = haut de la caméra, -Y = visée.
"""
import adsk.core, adsk.fusion, math
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
GROUP = 'FM_Direction_V04'
C1_STEP = str(_FM_ROOT / 'cao/mounts/prototype/fusion_v01_2026-10-13/sources/c1_officiel.stp')
SOL = 'd4_roue_ar_d / 2 + d4_axe_ar_sous_planche + d4_planche_ep'   # dessus de planche au-dessus du sol
NOMS = ('D4_REF_Profile_40x40', 'D4_IMP_Support_lidar', 'D4_REF_Lidar_C1', 'D4_IMP_Support_stereo',
        'D4_IMP_Barre_stereo', 'D4_REF_Webcam', 'D4_REF_Equerre')

# profilé 40 x 40 type B, section simplifiée : rainure de 10 à lèvres de 3, chambre 18 x 5,5, alésage Ø8,5
PROF, FENTE, LEVRE, CHAMBRE_L, CHAMBRE_FOND, ALESAGE = 40.0, 10.0, 3.0, 18.0, 8.5, 8.5
# support lidar
EMB = PROF + 2 * 0.25        # emboîtement : 0,25 de jeu par côté
EMB_H = 20.0                 # profondeur d'emboîtement
JUPE, PLAQUE, CHANF = 53.0, 56.0, 3.0
PLAQUE_EP = 7.5              # logement de tête 3 + 4,5 sous tête : M2,5x8 entre de 3,5 dans le C1 (4 maxi)
LANG_L, LANG_P = 9.6, 1.8    # languettes dans l'ouverture des rainures (10)
C1_DEMI = 43.0 / 2
PUITS_D, TETE_H, PASSAGE = 6.0, 3.0, 3.0
M8_D, M8_Z = 8.6, -EMB_H / 2      # vis M8 latérales dans les écrous en T M8 des rainures
CABLE_L, CABLE_P = 12.0, 2.0  # gorge de sortie du câble du C1 (arrière)
# barre stéréo
BARRE_L, BARRE_P, BARRE_EP = 170.0, 56.0, 16.0
SOUS_ASSISE = 8.0            # matière au-dessus du haut du profilé, au bord avant de l'emboîtement
GOUSSET, GOUSSET_EP, GOUSSET_Y = 15.0, 6.0, 15.0
TROU_CAM, LAMAGE, LAMAGE_SOUS = 6.8, 14.0, 8.0   # vis 1/4"-20 (ou M6) x 12 : 8 mm de plastique sous la tête
# équerre 40 x 40 du commerce
EQ_L, EQ_AILE, EQ_EP, EQ_TROU, EQ_D = 38.0, 36.0, 5.0, 20.0, 8.5
# webcam UGREEN 15728 (enveloppe : 81,3 x 48,3 x 47 annoncés ; pied et objectif à mesurer)
CAM_L, PIED_L, PIED_P, PIED_H, CORPS_P, OBJ_D = 81.3, 45.0, 40.0, 19.0, 28.0, 18.0


def octo(demi, ch):
    return [(-demi + ch, -demi), (demi - ch, -demi), (demi, -demi + ch), (demi, demi - ch),
            (demi - ch, demi), (-demi + ch, demi), (-demi, demi - ch), (-demi, -demi + ch)]


def plane_y(c, y, name):
    """Plan parallèle à XZ, à l'ordonnée y (mm), quel que soit le sens de la normale du plan XZ."""
    for signe in (1, -1):
        i = c.constructionPlanes.createInput()
        i.setByOffset(c.xZConstructionPlane, V(signe * y))
        p = c.constructionPlanes.add(i)
        if abs(p.geometry.origin.y * 10 - y) < 1e-6:
            p.name = 'plan_' + name
            p.isLightBulbOn = False
            return p
        p.deleteMe()
    raise AssertionError('plan y introuvable')


def poly3(s, pts):
    """Polygone fermé à partir de points modèle (x, y, z) en mm."""
    lines = s.sketchCurves.sketchLines
    sk = [s.modelToSketchSpace(P(*p)) for p in pts]
    first = prev = None
    for k in range(len(sk)):
        a = prev if prev is not None else sk[0]
        b = sk[k + 1] if k + 1 < len(sk) else first
        ln = lines.addByTwoPoints(a, b)
        if first is None:
            first = ln.startSketchPoint
        prev = ln.endSketchPoint


def sym(c, profs, total, op, name, bodies=None):
    i = c.features.extrudeFeatures.createInput(profs, op)
    i.setSymmetricExtent(V(total), True)
    if bodies is not None:
        i.participantBodies = list(bodies)
    f = c.features.extrudeFeatures.add(i)
    f.name = name
    return f


def profil_cercle(sk, centre, r_mm):
    """Profils contenus dans le cercle tracé : une esquisse posée sur une face reprend aussi le contour
    de la face (et ses trous déjà percés, qui coupent un lamage en deux profils)."""
    r = r_mm / 10 + 1e-4
    dedans = lambda p: all(abs(p.boundingBox.minPoint.asArray()[i] - centre.asArray()[i]) <= r and
                           abs(p.boundingBox.maxPoint.asArray()[i] - centre.asArray()[i]) <= r for i in range(2))
    return profiles(sk, dedans)


def place(g, o, nom, ox, oy, oz, ang=None):
    ji = g.joints.createInput(origin_geo(o), adsk.fusion.JointGeometry.createByPoint(g.originConstructionPoint))
    ji.setAsRigidJointMotion()
    j = g.joints.add(ji)
    j.name = nom
    j.offsetX.expression = ox
    j.offsetY.expression = oy
    j.offset.expression = oz
    if ang:
        j.angle.expression = ang
    return j


def profile(nom, hauteur):
    o, c = part('D4_REF_Profile_40x40_' + nom, statut='REF_PROFILE_TYPE_B_SIMPLIFIE')
    h, fd, cd = PROF / 2, FENTE / 2, CHAMBRE_L / 2
    motif = [(-h, h), (-fd, h), (-fd, h - LEVRE), (-cd, h - LEVRE), (-cd, h - CHAMBRE_FOND),
             (cd, h - CHAMBRE_FOND), (cd, h - LEVRE), (fd, h - LEVRE), (fd, h)]
    pts = []
    for k in range(4):   # une face après l'autre, en tournant de -90°
        a = -k * math.pi / 2
        pts += [(x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)) for x, y in motif]
    s = sketch(c, plane_z(c, 0, 'profile'), 'profile')
    poly(s, pts)
    circle(s, 0, 0, ALESAGE)
    body(extrude(c, profiles(s, 'ring'), hauteur, NEW, 'extr_profile'), 'REF_profile_40x40_typeB', 'FM_METAL')
    return o


def equerre(nom):
    """Équerre 40 côté +X du profilé (l'autre côté : même pièce tournée de 180°)."""
    o, c = part('D4_REF_Equerre_40_' + nom, statut='REF_EQUERRE_40_COMMERCE')
    x0, l = PROF / 2, EQ_L / 2
    b = body(zbox(c, 'aile_planche', x0, -l, x0 + EQ_AILE, l, 0, EQ_EP), 'REF_equerre_40', 'FM_METAL')
    zbox(c, 'aile_profile', x0, -l, x0 + EQ_EP, l, 0, EQ_AILE, JOIN, [b])
    for sy in (1, -1):
        s = sketch(c, plane_y(c, sy * (l - 1.5), 'joue_%+d' % sy), 'joue_%+d' % sy)
        poly3(s, [(x0 + EQ_EP, sy * (l - 1.5), EQ_EP), (x0 + EQ_AILE, sy * (l - 1.5), EQ_EP), (x0 + EQ_EP, sy * (l - 1.5), EQ_AILE)])
        sym(c, profiles(s), 3, JOIN, 'extr_joue_%+d' % sy, [b])
    zcyl(c, 'trou_planche', x0 + EQ_TROU, 0, EQ_D, -1, EQ_EP + 2, CUT, [b])
    xcyl(c, 'trou_profile', x0 - 1, 0, EQ_TROU, EQ_D, EQ_EP + 2, CUT, [b])
    return o


def support_lidar():
    o, c = part('D4_IMP_Support_lidar', statut='ETUDE_V04_CAPTEURS')
    b = body(zpoly(c, 'jupe', octo(JUPE / 2, CHANF), -EMB_H, EMB_H), 'support_lidar', 'FM_PROVISOIRE_orange')
    zpoly(c, 'plaque', octo(PLAQUE / 2, CHANF), 0, PLAQUE_EP, JOIN, [b])
    zbox(c, 'emboitement', -EMB / 2, -EMB / 2, EMB / 2, EMB / 2, -EMB_H - 1, EMB_H + 1, CUT, [b])
    # languettes dans l'ouverture des 4 rainures : le support ne tourne pas sur le profilé
    e, l = EMB / 2, LANG_L / 2
    for nom, (x0, y0, x1, y1) in (('X+', (e - LANG_P, -l, e + 0.05, l)), ('X-', (-e - 0.05, -l, -e + LANG_P, l)),
                                  ('Y+', (-l, e - LANG_P, l, e + 0.05)), ('Y-', (-l, -e - 0.05, l, -e + LANG_P))):
        zbox(c, 'languette_' + nom, x0, y0, x1, y1, -EMB_H, EMB_H, JOIN, [b])
    # coins : puits de la clé et de la tête (jusqu'à 3 mm dans la plaque), puis passage M2,5
    for k, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        zcyl(c, 'puits_C1_%d' % k, sx * C1_DEMI, sy * C1_DEMI, PUITS_D, -EMB_H - 1, EMB_H + 1 + TETE_H, CUT, [b])
        zcyl(c, 'passage_C1_%d' % k, sx * C1_DEMI, sy * C1_DEMI, PASSAGE, TETE_H - 0.1, PLAQUE_EP, CUT, [b])
    # vis M8 latérales (gauche et droite) dans des écrous en T M8 rainure 10
    xcyl(c, 'vis_M8_D', EMB / 2 - LANG_P - 1, 0, M8_Z, M8_D, JUPE / 2 - EMB / 2 + LANG_P + 2, CUT, [b])
    xcyl(c, 'vis_M8_G', -(EMB / 2 - LANG_P - 1), 0, M8_Z, M8_D, -(JUPE / 2 - EMB / 2 + LANG_P + 2), CUT, [b])
    # gorge du câble du C1, vers l'arrière
    zbox(c, 'gorge_cable', -CABLE_L / 2, PLAQUE / 2 - 8, CABLE_L / 2, PLAQUE / 2 + 1, PLAQUE_EP - CABLE_P, CABLE_P + 1, CUT, [b])
    return o


def lidar():
    o, c = part('D4_REF_Lidar_C1', statut='STEP_OFFICIEL_SLAMTEC')
    im = app().importManager
    im.importToTarget(im.createSTEPImportOptions(C1_STEP), c)
    return o


def barre_stereo(a, zc, zs):
    """Barre des webcams emboîtée sur le haut du profilé avant (origine : axe du profilé, dessus de
    planche). Assise plane inclinée de a vers l'avant ; emboîtement vertical, donc incliné dans la
    barre ; goussets sous les bras ; trous des vis photo perpendiculaires à l'assise."""
    o, c = part('D4_IMP_Barre_stereo', statut='ETUDE_V04_CAPTEURS')
    ca, sa = math.cos(a), math.sin(a)
    pt = lambda d, t: (0, d * ca - t * sa, zs + d * sa + t * ca)   # d le long de l'assise (vers l'arrière), t selon sa normale
    s = sketch(c, c.yZConstructionPlane, 'barre')
    poly3(s, [pt(-BARRE_P / 2, 0), pt(BARRE_P / 2, 0), pt(BARRE_P / 2, -BARRE_EP), pt(-BARRE_P / 2, -BARRE_EP)])
    b = body(sym(c, profiles(s), BARRE_L, NEW, 'extr_barre'), 'barre_stereo', 'FM_PROVISOIRE_orange')
    zbas = zc - EMB_H
    zpoly(c, 'jupe_barre', octo(JUPE / 2, CHANF), zbas, zs + 15 - zbas, JOIN, [b])
    s = sketch(c, c.yZConstructionPlane, 'barre_assise')
    poly3(s, [pt(-60, 0), pt(60, 0), pt(60, 40), pt(-60, 40)])
    sym(c, profiles(s), BARRE_L + 20, CUT, 'extr_barre_assise', [b])
    zbox(c, 'emboitement_barre', -EMB / 2, -EMB / 2, EMB / 2, EMB / 2, zbas - 1, EMB_H + 1, CUT, [b])
    e, l = EMB / 2, LANG_L / 2
    for nom, (x0, y0, x1, y1) in (('X+', (e - LANG_P, -l, e + 0.05, l)), ('X-', (-e - 0.05, -l, -e + LANG_P, l)),
                                  ('Y+', (-l, e - LANG_P, l, e + 0.05)), ('Y-', (-l, -e - 0.05, l, -e + LANG_P))):
        zbox(c, 'languette_barre_' + nom, x0, y0, x1, y1, zbas, EMB_H, JOIN, [b])
    xcyl(c, 'vis_M8_barre_D', EMB / 2 - LANG_P - 1, 0, zc - EMB_H / 2, M8_D, JUPE / 2 - EMB / 2 + LANG_P + 2, CUT, [b])
    xcyl(c, 'vis_M8_barre_G', -(EMB / 2 - LANG_P - 1), 0, zc - EMB_H / 2, M8_D, -(JUPE / 2 - EMB / 2 + LANG_P + 2), CUT, [b])
    # goussets sous les bras, devant et derrière les vis M8 latérales
    for sx in (1, -1):
        for sy in (1, -1):
            y0 = sy * GOUSSET_Y
            zu = zs - BARRE_EP / ca + y0 * math.tan(a)   # dessous de la barre à cette ordonnée
            nom = 'gousset_%s%s' % ('D' if sx > 0 else 'G', 'ar' if sy > 0 else 'av')
            s = sketch(c, plane_y(c, y0, nom), nom)
            poly3(s, [(sx * (JUPE / 2 - 0.5), y0, zu + 3), (sx * (JUPE / 2 + GOUSSET), y0, zu + 3), (sx * (JUPE / 2 - 0.5), y0, zu + 3 - GOUSSET - 0.5)])
            sym(c, profiles(s), GOUSSET_EP, JOIN, 'extr_' + nom, [b])
    # trous des vis photo, perpendiculaires à l'assise, et lamages par-dessous
    u = (0.0, -sa, ca)
    # la face d'assise change à chaque perçage : la rechercher avant chaque esquisse
    assise = lambda: max((f for f in b.faces if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType),
                         key=lambda f: sum(n * w for n, w in zip(f.evaluator.getNormalAtPoint(f.pointOnFace)[1].asArray(), u)))
    cercles = {}
    for tag, sx in (('D', 1), ('G', -1)):
        x = sx * ev('d4_stereo_base') / 2
        sk = c.sketches.add(assise())
        sk.name = 'esq_cam_trou_' + tag
        sk.isLightBulbOn = False
        ctr = sk.modelToSketchSpace(P(x, 0, zs))
        cercles[tag] = sk.sketchCurves.sketchCircles.addByCenterRadius(ctr, TROU_CAM / 20)
        extrude(c, profil_cercle(sk, ctr, TROU_CAM / 2), 30, CUT, 'cam_trou_' + tag, [b], down=True)
        sk2 = c.sketches.add(assise())
        sk2.name = 'esq_cam_lamage_' + tag
        sk2.isLightBulbOn = False
        ctr2 = sk2.modelToSketchSpace(P(x, 0, zs))
        sk2.sketchCurves.sketchCircles.addByCenterRadius(ctr2, LAMAGE / 20)
        extrude(c, profil_cercle(sk2, ctr2, LAMAGE / 2), 30, CUT, 'cam_lamage_' + tag, [b], down=True, start_offset='%r mm' % -LAMAGE_SOUS)
    return o, cercles


def webcam(tag, h, av):
    o, c = part('D4_REF_Webcam_UGREEN_' + tag, statut='ENVELOPPE_PROVISOIRE_A_MESURER')
    pied = body(zbox(c, 'pied', -PIED_L / 2, -PIED_P / 2, PIED_L / 2, PIED_P / 2, 0, PIED_H), 'REF_webcam_pied', 'Opaque(76,76,76)')
    corps = body(zbox(c, 'corps', -CAM_L / 2, -av + 2, CAM_L / 2, -av + 2 + CORPS_P, h - CORPS_P / 2, CORPS_P), 'REF_webcam_corps', 'Opaque(204,204,204)')
    s = sketch(c, plane_y(c, -av + 1, 'objectif'), 'objectif')   # objectif de 2 mm devant le corps
    s.sketchCurves.sketchCircles.addByCenterRadius(s.modelToSketchSpace(P(0, -av + 1, h)), OBJ_D / 20)
    body(sym(c, profiles(s), 2, NEW, 'extr_objectif'), 'REF_webcam_objectif', 'Opaque(51,51,51)')
    return o, c


def lier_webcam(g, o, cercle, nom, a):
    """Liaison rigide du pied de la webcam sur le cercle du trou de vis (centre et normale de l'assise).
    Le sens et l'angle de la liaison ne se modifient pas après coup : on essaie les combinaisons et on
    garde celle qui donne Z caméra = haut incliné et -Y = visée (pose lue dans le contexte du groupe)."""
    vz = (0.0, -math.sin(a), math.cos(a))
    vy = (0.0, math.cos(a), math.sin(a))
    dot = lambda p, q: sum(x * y for x, y in zip(p, q))
    essais = []
    for flip in (False, True):
        for ang in (0, 90, 180, 270):
            ji = g.joints.createInput(origin_geo(o), adsk.fusion.JointGeometry.createByCurve(cercle, adsk.fusion.JointKeyPointTypes.CenterKeyPoint))
            ji.setAsRigidJointMotion()
            ji.isFlipped = flip
            ji.angle = V('%d deg' % ang)
            j = g.joints.add(ji)
            og, xa, ya, za = next(x for x in group().childOccurrences if x.name == o.name).transform2.getAsCoordinateSystem()
            essais.append((flip, ang, [round(v, 3) for v in og.asArray()], [round(v, 3) for v in ya.asArray()], [round(v, 3) for v in za.asArray()]))
            if dot(za.asArray(), vz) > 0.999 and dot(ya.asArray(), vy) > 0.999:
                j.name = nom
                return flip, ang
            j.deleteMe()
    raise AssertionError('orientation webcam introuvable %r' % essais)


def percages(g):
    """Perçages de la planche : supports de pivot (M8), supports moteurs (M8), oreilles du servo (M3),
    équerres des deux profilés (M8, à 20 mm de la face du profilé, gauche et droite)."""
    trous = []
    e2 = ev('d4_E') / 2
    for sx in (1, -1):
        for dx in (31.0, 60.0):
            trous.append(('pivot', sx * (e2 - dx), -4.0, 8.5))
    for oo in group().childOccurrences:
        if oo.component.name != 'D4_REF_Support_moteur_arriere':
            continue
        vus = []
        for b in oo.bRepBodies:
            for f in b.faces:
                gm = f.geometry
                if gm.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and abs(gm.radius * 10 - 4.5) < 0.05 and abs(abs(gm.axis.z) - 1) < 1e-6:
                    xy = (round(gm.origin.x * 10, 2), round(gm.origin.y * 10, 2))
                    if xy not in vus:
                        vus.append(xy)
                        trous.append(('moteur', xy[0], xy[1], 8.5))
    yc = ev('d4_servo_y') - 10.0 + 40.4 / 2
    for sx in (1, -1):
        for sy in (1, -1):
            trous.append(('servo', sx * 5.0, yc + sy * 24.0, 3.4))
    for nom, y in (('equerre_lidar', ev('d4_lidar_y')), ('equerre_cam', ev('d4_cam_y'))):
        for sx in (1, -1):
            trous.append((nom, sx * (PROF / 2 + EQ_TROU), y, EQ_D))
    c = next(oo.component for oo in g.occurrences if oo.component.name == 'D4_REF_Planche_bois')
    pb = c.bRepBodies.itemByName('REF_planche_bois_non_imprimee')
    s = sketch(c, plane_z(c, -1, 'perc_planche'), 'perc_planche')
    for _, x, y, d in trous:
        circle(s, x, y, d)
    extrude(c, profiles(s), 'd4_planche_ep + 2 mm', CUT, 'extr_perc_planche', [pb])
    return trous


def run(_context: str):
    d = design()
    g = group().component
    for oo in list(g.occurrences):
        if oo.component.name.startswith(NOMS):   # Fusion suffixe parfois « (1) » à la reconstruction
            oo.deleteMe()
    pl = next(oo.component for oo in g.occurrences if oo.component.name == 'D4_REF_Planche_bois')
    for coll in (pl.features.extrudeFeatures, pl.sketches, pl.constructionPlanes):
        for x in [coll.item(i) for i in range(coll.count)]:
            if x.name in ('extr_perc_planche', 'esq_perc_planche', 'plan_perc_planche'):
                x.deleteMe()

    param('d4_lidar_y', 'd4_empattement', 'V04 | Axe du mat lidar derriere les pivots : sur l axe des roues arriere, centre de rotation du rover')
    param('d4_profil_h', '150 mm', 'V04 | Profile 40x40 au-dessus de la planche ; plan laser du C1 a ~322 mm du sol (lidar simule : 0,323 m)')
    param('d4_stereo_base', '120 mm', 'V04 | Ecartement des centres optiques des webcams')
    param('d4_cam_inclinaison', '10 deg', 'V04 | Webcams inclinees vers le sol (camera simulee : -10 deg)', 'deg')
    param('d4_cam_optique_sol', '270 mm', 'V04 | Centre optique des webcams au-dessus du sol (camera simulee : 0,269 m)')
    if d.userParameters.itemByName('d4_cam_y_vis'):   # paramètre de l'ancien cadre stéréo
        d.userParameters.itemByName('d4_cam_y_vis').deleteMe()
    param('d4_cam_y', '-d4_planche_avant + 25 mm', 'V04 | Axe du profile des cameras (et des vis photo), 25 mm derriere le bord avant')
    param('d4_cam_axe_sur_pied', '33 mm', 'V04 | PROVISOIRE a mesurer : centre optique au-dessus de la face d appui du pied')
    param('d4_cam_axe_devant_vis', '22 mm', 'V04 | PROVISOIRE a mesurer : centre optique devant l axe de la vis photo')

    a = d.unitsManager.evaluateExpression('d4_cam_inclinaison', 'deg')
    h, av = ev('d4_cam_axe_sur_pied'), ev('d4_cam_axe_devant_vis')
    # hauteur du profilé avant, arrondie au mm, pour un centre optique à d4_cam_optique_sol
    zs_cible = ev('d4_cam_optique_sol') - ev(SOL) - h * math.cos(a) + av * math.sin(a)
    marge = SOUS_ASSISE + EMB / 2 * math.tan(a)        # assise -> haut du profilé
    param('d4_cam_profil_h', '%d mm' % round(zs_cible - marge), 'V04 | Profile 40x40 des cameras au-dessus de la planche (derive de d4_cam_optique_sol, arrondi au mm)')
    zc = ev('d4_cam_profil_h')
    zs = zc + marge

    o = profile('lidar', 'd4_profil_h')
    place(g, o, 'D4_place_profile_lidar', '0 mm', 'd4_lidar_y', SOL)
    o = profile('cam', 'd4_cam_profil_h')
    place(g, o, 'D4_place_profile_cam', '0 mm', 'd4_cam_y', SOL)
    for mat, y in (('lidar', 'd4_lidar_y'), ('cam', 'd4_cam_y')):
        for tag, ang in (('D', None), ('G', '180 deg')):
            place(g, equerre('%s_%s' % (mat, tag)), 'D4_place_equerre_%s_%s' % (mat, tag), '0 mm', y, SOL, ang)
    o = support_lidar()
    place(g, o, 'D4_place_support_lidar', '0 mm', 'd4_lidar_y', '%s + d4_profil_h' % SOL)
    ol = lidar()
    jl = place(g, ol, 'D4_place_lidar_C1', '0 mm', 'd4_lidar_y', '%s + d4_profil_h + %r mm' % (SOL, PLAQUE_EP), '-90 deg')
    bb = ol.boundingBox
    if bb.maxPoint.y * 10 < ev('d4_lidar_y') + 60:   # le câble (130 mm) doit partir vers l'arrière (+Y)
        jl.angle.expression = '90 deg'
    os_, cercles = barre_stereo(a, zc, zs)
    place(g, os_, 'D4_place_barre_stereo', '0 mm', 'd4_cam_y', SOL)
    poses = {}
    for tag in ('D', 'G'):   # un composant par côté (une copie collée laisse traîner des noms suffixés)
        ow, _ = webcam(tag, h, av)
        poses[tag] = lier_webcam(g, ow, cercles[tag].createForAssemblyContext(os_), 'D4_place_webcam_' + tag, a)
    trous = percages(g)

    # contrôle
    gp = group()
    for ow_ in gp.childOccurrences:
        if ow_.component.name.startswith('D4_REF_Webcam_UGREEN'):
            m = ow_.transform2
            p = P(0, -av, h)
            p.transformBy(m)
            print('centre optique', ow_.name, [round(v * 10, 1) for v in p.asArray()])
    for ow_ in gp.childOccurrences:
        if ow_.component.name.startswith(NOMS):
            b_ = ow_.boundingBox
            print(ow_.name, [round(v * 10, 1) for v in b_.minPoint.asArray()], [round(v * 10, 1) for v in b_.maxPoint.asArray()])
    vb = next(x for x in gp.childOccurrences if x.component.name == 'D4_IMP_Barre_stereo').component.bRepBodies.item(0)
    vl = next(x for x in gp.childOccurrences if x.component.name == 'D4_IMP_Support_lidar').component.bRepBodies.item(0)
    print('volumes cm3 : barre stereo', round(vb.volume, 1), 'lidar', round(vl.volume, 1), 'profile cameras', ev('d4_cam_profil_h'))
    print('liaisons webcam', poses, 'angle C1', jl.angle.expression, 'zs', round(zs, 2))
    print('trous', [(t, round(x, 1), round(y, 1), dd) for t, x, y, dd in trous])
    tl = d.timeline
    print('fonctions KO', [tl.item(i).name for i in range(tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup],
          'liaisons KO', [j.name for j in g.joints if j.healthState != 0])
