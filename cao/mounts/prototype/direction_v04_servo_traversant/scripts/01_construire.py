from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Direction V04 — tout sous la planche, servo à travers la planche.
Document Fusion « systeme de direction simple v1 », groupe FM_Direction_V04.

Par côté : un SUPPORT plat vissé sous la planche (tient la vis-pivot M8, porte les butées)
et une FUSÉE imprimée à plat (2 x 608ZZ, bras de direction intégré, axe de roue en bas).
Au centre : le servo traverse la planche, un PALONNIER en colonne descend au niveau des bras
et porte deux BIELLETTES sur roulements 3x10x4.

Repère local : axe de pivot en (0, 0), Z = 0 au DESSOUS de la planche, avant = -Y,
s = +1 droite, -1 gauche. Le groupe est posé à d4_roue_ar_d/2 + d4_axe_ar_sous_planche du sol,
pour que la planche soit horizontale avec les roues arrière ; l'axe avant est à
d4_axe_av_sous_planche sous la planche.

Étages (Z, mm) : support [-11 ; 0] (écrou M8 prisonnier côté planche, bossage [-12 ; -11],
butées [-19 ; -11]) ; fusée : tube [-28 ; -12] (608ZZ, arête 2 mm, 608ZZ), bras [-20 ; -12],
pont et jambe sous -20, jambe jusqu'à l'axe - 13 ; vis-pivot M8x30 par-dessous ;
biellettes : oeil [-24,5 ; -20,5], barre jusqu'à -26,5 ; palonnier : plateau [-20 ; -12],
rebord [-12 ; -11] avec l'empreinte du palonnier étoile d'origine (face des branches à -13,3 avec la
planche de 5 mm : d4_etoile_sous_oreilles = 18,3 mm sous la face d'appui des oreilles).
"""
import adsk.core, adsk.fusion, math
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
GROUP = 'FM_Direction_V04'

ALPHA = math.radians(12)
BRAS, R_PAL, B_PAL = 30.0, 28.0, 8.0
SIEGE_608, SIEGE_3104 = 22.15, 10.15
OEIL = 14.0
Z_FUS_HAUT, Z_FUS_BAS = -12.0, -28.0   # tube de fusée : 608ZZ + arête 2 mm + 608ZZ (vis M8x30)
Z_BRAS0 = -20.0                       # dessous du bras / du plateau de palonnier
Z_OEIL0 = -24.5                       # dessous de l'oeil de biellette (rondelle M3 de 0,5 mm au-dessus)
JAMBE_X0, JAMBE_X1, HW = 21.0, 32.0, 13.0   # jambe de 11 mm : vis de roue M8x30 + écrou M8 simple
BUTEE_OFF = 38.0 + math.degrees(math.asin(5 / 20) + math.asin(3 / 20))
TROUS_SUPPORT = [(-31, -4), (-60, -4)]   # 2 vis H M8 par support, tête prisonnière dessous (09_supports_M8.py)
# Servo DT996 mesuré (15/10) : boîtier 40,4 x 20 x 39,3, oreilles 2,2 dont le dessous est à 28,5 de la base,
# bossage jusqu'à 40,8, cannelure jusqu'à 44,8, palonnier étoile monté jusqu'à 49 (10_servo_mesure_etoile.py).
SERVO_L, SERVO_W, BOITIER_H, AXE_BOUT = 40.4, 20.0, 39.3, 10.0
APPUI_OREILLES = 28.5 + 2.2
OREILLES_L, OREILLE_EP, ENTRAXE_OY, ENTRAXE_OX = 54.0, 2.2, 48.0, 10.0
BOSSAGE_D, BOSSAGE_H, CANNELURE_D = 12.0, 1.5, 6.0
ETOILE_EP, ETOILE_TOTAL, MOYEU_D = 2.6, 5.5, 9.0
R_BOUT, U_BOUT, DEMI_PIED, U_PIED, PHASE = 2.4, 13.3, 3.5, 7.0, 30.0
JEU_ETOILE, REBORD_SOUS_FACE = 0.25, 2.3
M2_R, M2_ANGLES = 10.3, (90, 210, 330)
FACE = 'd4_planche_ep - d4_etoile_sous_oreilles'   # face des branches de l'étoile


def etoile(jeu=0.0, n_bout=10):
    """Contour du palonnier étoile à 6 branches (mm, axe du servo en 0), décalé de `jeu`."""
    hw = lambda u: (DEMI_PIED if u <= U_PIED else DEMI_PIED - (DEMI_PIED - R_BOUT) * (u - U_PIED) / (U_BOUT - U_PIED)) + jeu
    u_creux = (2 * (DEMI_PIED + jeu)) * math.cos(math.radians(30))
    pts = []
    for k in range(6):
        t = math.radians(PHASE + 60 * k)
        c, s = math.cos(t), math.sin(t)
        xy = lambda u, v: (u * c - v * s, u * s + v * c)
        pts.append(xy(u_creux, -hw(u_creux)))
        pts.append(xy(U_PIED, -hw(U_PIED)))
        for i in range(n_bout + 1):
            a = -math.pi / 2 + math.pi * i / n_bout
            pts.append(xy(U_BOUT + (R_BOUT + jeu) * math.cos(a), (R_BOUT + jeu) * math.sin(a)))
        pts.append(xy(U_PIED, hw(U_PIED)))
    return pts

PARAMS = [
    ('d4_planche_larg', '300 mm', 'Largeur de planche (30 cm)'),
    ('d4_debord_pivot', '5 mm', 'Axe de pivot au-dela du bord de planche ; >= 3 mm sinon les pneus touchent la planche en braquant'),
    ('d4_E', 'd4_planche_larg + 2 * d4_debord_pivot', 'Ecartement des axes de pivot (derive de la largeur de planche)'),
    ('d4_planche_ep', '5 mm', 'Epaisseur de planche (5 mm) ; la colonne du palonnier et le servo la suivent'),
    ('d4_planche_long', '450 mm', 'Longueur de planche (reference visuelle)'),
    ('d4_planche_avant', '60 mm', 'Debord de planche devant les pivots (reference visuelle)'),
    ('d4_roue_ar_d', '200 mm', 'Diametre total des roues arriere (crampons compris)'),
    ('d4_axe_ar_sous_planche', '30 mm', 'Axe des roues arriere sous le dessous de la planche'),
    ('d4_roue_av_d', '150 mm', 'Diametre des roues avant'),
    ('d4_axe_av_sous_planche', 'd4_roue_ar_d / 2 + d4_axe_ar_sous_planche - d4_roue_av_d / 2',
     'Axe avant sous la planche pour une planche horizontale (derive) ; garder >= 55 mm'),
    ('d4_vis_roue', '8 mm', 'Diametre de la vis de roue (8 pour M8, 10 pour M10)'),
    ('d4_ecrou_roue_s', '13 mm', 'Ecrou captif de roue, cote sur plats (M8 : 13 ; M10 : 16 ou 17)'),
    ('d4_ecrou_roue_h', '6.5 mm', 'Ecrou captif de roue, hauteur (ecrou M8 simple : 6,5)'),
    ('d4_servo_y', '30 mm * cos(12 deg) + 28 mm', 'Recul de l axe servo derriere la ligne des pivots (derive)'),
    ('d4_etoile_sous_oreilles', '18.3 mm', 'Face des branches du palonnier etoile sous la face d appui des oreilles (49 - 30,7)'),
    ('d4_biellette_L', 'd4_E / 2 - 30 mm * sin(12 deg) - 8 mm', 'Entraxe des biellettes (derive de d4_E)'),
]
AXE = 'd4_axe_av_sous_planche'


def ring(c, name, d_out, d_in, z0, h, color='FM_PROVISOIRE_orange', x=0.0, y=0.0):
    sk = sketch(c, plane_z(c, z0, name), name)
    circle(sk, x, y, d_out)
    circle(sk, x, y, d_in)
    return body(extrude(c, profiles(sk, 'ring'), h, NEW, 'extr_' + name), name, color)


def centre_axe_roue(s, x0):
    """Point d'esquisse (plan YZ en x0) placé sur l'axe de roue, à d4_axe_av_sous_planche sous Z = 0."""
    ref = s.sketchPoints.add(s.modelToSketchSpace(P(x0, 0, 0)))
    ref.isFixed = True
    zz = ev(AXE)
    c = s.sketchPoints.add(s.modelToSketchSpace(P(x0, 0, -zz)))
    g0, g1 = ref.geometry, c.geometry
    vertical = abs(g1.x - g0.x) < abs(g1.y - g0.y)
    (s.geometricConstraints.addVerticalPoints if vertical else s.geometricConstraints.addHorizontalPoints)(ref, c)
    o = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation if vertical else adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
    d = s.sketchDimensions.addDistanceDimension(ref, c, o, adsk.core.Point3D.create(g0.x + 1.5, (g0.y + g1.y) / 2, 0))
    d.parameter.expression = AXE
    return c


def trou_roue(c, x0, length, bodies):
    s = sketch(c, plane_x(c, x0, 'vis_roue'), 'vis_roue')
    ctr = centre_axe_roue(s, x0)
    cc = s.sketchCurves.sketchCircles.addByCenterRadius(ctr.geometry, ev('d4_vis_roue + 0.4 mm') / 20)
    s.geometricConstraints.addCoincident(cc.centerSketchPoint, ctr)
    s.sketchDimensions.addDiameterDimension(cc, adsk.core.Point3D.create(ctr.geometry.x + 1, ctr.geometry.y + 1, 0)).parameter.expression = 'd4_vis_roue + 0.4 mm'
    return extrude(c, profiles(s), abs(length), CUT, 'perc_vis_roue', bodies, down=length < 0)


def ecrou_roue(c, x0, sens, bodies):
    """Poche hexagonale paramétrique (cote sur plats d4_ecrou_roue_s), sommet vers ±Z."""
    s = sketch(c, plane_x(c, x0, 'ecrou_roue'), 'ecrou_roue')
    ctr = centre_axe_roue(s, x0)
    zz = ev(AXE)
    r = ev('d4_ecrou_roue_s + 0.3 mm') / math.sqrt(3)
    cc = s.sketchCurves.sketchCircles.addByCenterRadius(ctr.geometry, r / 10)
    cc.isConstruction = True
    gc = s.geometricConstraints
    gc.addCoincident(cc.centerSketchPoint, ctr)
    # sommet vers ±Z : la fusée s'imprime à plat, la poche horizontale garde un toit en pointe
    pts = [s.modelToSketchSpace(P(x0, r * math.cos(math.pi / 2 + k * math.pi / 3), -zz + r * math.sin(math.pi / 2 + k * math.pi / 3))) for k in range(6)]
    lines = s.sketchCurves.sketchLines
    L = []
    prev = None
    for k in range(6):
        a = prev if prev is not None else pts[0]
        b = pts[k + 1] if k < 5 else L[0].startSketchPoint
        ln = lines.addByTwoPoints(a, b)
        L.append(ln)
        prev = ln.endSketchPoint
    for ln in L:
        gc.addCoincident(ln.startSketchPoint, cc)
    for ln in L[1:]:
        gc.addEqual(L[0], ln)
    g1, g2 = L[1].startSketchPoint.geometry, L[1].endSketchPoint.geometry
    (gc.addHorizontal if abs(g2.y - g1.y) < abs(g2.x - g1.x) else gc.addVertical)(L[1])
    s.sketchDimensions.addDiameterDimension(cc, adsk.core.Point3D.create(ctr.geometry.x + 1.2, ctr.geometry.y + 1.2, 0)).parameter.expression = '(d4_ecrou_roue_s + 0.3 mm) / cos(30 deg)'
    prof = profiles(s, lambda p: p.profileLoops.count == 1)
    return extrude(c, prof, 'd4_ecrou_roue_h + 0.2 mm', CUT, 'poche_ecrou_roue', bodies, down=sens < 0)


def support(s, tag):
    o, c = part('D4_IMP_Support_pivot_' + tag)
    sk = sketch(c, plane_z(c, -11, 'support'), 'support')
    x0, x1 = sorted((s * -72, 0.0))
    rect(sk, x0, -22, x1, 14)
    circle(sk, 0, 0, 48)
    b = body(extrude(c, profiles(sk), 11, NEW, 'extr_support'), 'support_pivot_' + tag)
    B = [b]
    zcyl(c, 'portee_bague_int', 0, 0, 12, -12, 1, JOIN, B)
    for k, sg in enumerate((+1, -1)):
        a = math.radians(90 + 12) + sg * math.radians(BUTEE_OFF)
        zcyl(c, 'butee_%d' % k, s * 20 * math.cos(a), 20 * math.sin(a), 6, -19, 8, JOIN, B)
    zcyl(c, 'passage_pivot', 0, 0, 8.4, -13, 14, CUT, B)
    skh = sketch(c, plane_z(c, 0, 'ecrou_M8'), 'ecrou_M8')
    poly(skh, hexagon(skh, 0, 0, 13.3))
    extrude(c, profiles(skh), 6.7, CUT, 'empreinte_ecrou_M8', B, down=True)
    for k, (x, y) in enumerate(TROUS_SUPPORT):
        zcyl(c, 'trou_M8_%d' % k, s * x, y, 8.4, -12, 13, CUT, B)
        skt = sketch(c, plane_z(c, -11, 'tete_M8_%d' % k), 'tete_M8_%d' % k)
        poly(skt, hexagon(skt, s * x, y, 13.3))
        extrude(c, profiles(skt), 5.5, CUT, 'empreinte_tete_M8_%d' % k, B)
    return o


def fusee(s, tag):
    o, c = part('D4_IMP_Fusee_' + tag)
    f = body(zcyl(c, 'tube', 0, 0, 32, Z_FUS_BAS, Z_FUS_HAUT - Z_FUS_BAS), 'fusee_' + tag)
    B = [f]
    x0, x1 = sorted((0.0, s * JAMBE_X0))
    zbox(c, 'pont', x0, -HW, x1, HW, Z_FUS_BAS, Z_BRAS0 - Z_FUS_BAS, JOIN, B)
    j0, j1 = sorted((s * JAMBE_X0, s * JAMBE_X1))
    zbox(c, 'jambe', j0, -HW, j1, HW, '-%s - 13 mm' % AXE, '%s + 13 mm - %r mm' % (AXE, -Z_BRAS0), JOIN, B)
    u = (-s * math.sin(ALPHA), math.cos(ALPHA))
    n = (math.cos(ALPHA), s * math.sin(ALPHA))
    pts = [(5 * n[0], 5 * n[1]), (BRAS * u[0] + 5 * n[0], BRAS * u[1] + 5 * n[1]),
           (BRAS * u[0] - 5 * n[0], BRAS * u[1] - 5 * n[1]), (-5 * n[0], -5 * n[1])]
    zpoly(c, 'bras', pts, Z_BRAS0, Z_FUS_HAUT - Z_BRAS0, JOIN, B)
    ex, ey = BRAS * u[0], BRAS * u[1]
    zcyl(c, 'oeil_bras', ex, ey, OEIL, Z_BRAS0, Z_FUS_HAUT - Z_BRAS0, JOIN, B)
    # logements 608ZZ haut et bas, arête centrale sur les bagues extérieures
    zcyl(c, 'siege_haut', 0, 0, SIEGE_608, Z_FUS_HAUT - 7, 8, CUT, B)
    zcyl(c, 'siege_bas', 0, 0, SIEGE_608, Z_FUS_BAS - 1, 8, CUT, B)
    zcyl(c, 'passage_pivot', 0, 0, 17, Z_FUS_BAS - 1, Z_FUS_HAUT - Z_FUS_BAS + 2, CUT, B)
    # articulation de biellette : trou M3, écrou M3 prisonnier sur le dessus du bras
    zcyl(c, 'trou_M3', ex, ey, 3.4, Z_BRAS0 - 1, 10, CUT, B)          # passage libre jusqu'à l'écrou
    skn = sketch(c, plane_z(c, Z_FUS_HAUT, 'ecrou_M3'), 'ecrou_M3')
    poly(skn, hexagon(skn, ex, ey, 5.8))
    extrude(c, profiles(skn), 2.6, CUT, 'poche_ecrou_M3', B, down=True)
    # roue : vis dans un écrou prisonnier, côté intérieur de la jambe
    trou_roue(c, s * (JAMBE_X0 - 1), s * (JAMBE_X1 - JAMBE_X0 + 3), B)
    ecrou_roue(c, s * (JAMBE_X0 - 0.01), s, B)
    joint_point(c, 'bras', ex, ey, 0)
    return o


def entretoises(tag):
    o, c = part('D4_IMP_Entretoises_' + tag)
    ring(c, 'rondelle_2', 12, 8.4, Z_FUS_BAS + 7, 2)              # entre les bagues intérieures
    ring(c, 'rondelle_2_tete', 12, 8.4, Z_FUS_BAS - 2, 2)         # sous la tête de vis
    return o


def pivot_ref(tag):
    o, c = part('D4_REF_Pivot_' + tag, statut='REFERENCE_ACHAT_NON_IMPRIMEE')
    ring(c, 'REF_608ZZ_haut', 22, 8, Z_FUS_HAUT - 7, 7, 'FM_METAL')
    ring(c, 'REF_608ZZ_bas', 22, 8, Z_FUS_BAS, 7, 'FM_METAL')
    v = body(zcyl(c, 'REF_vis_M8x30', 0, 0, 8, Z_FUS_BAS - 2, 30), 'REF_vis_H_M8x30', 'FM_METAL')
    skh = sketch(c, plane_z(c, Z_FUS_BAS - 2, 'tete_M8_ref'), 'tete_M8_ref')
    poly(skh, hexagon(skh, 0, 0, 13))
    extrude(c, profiles(skh), 5.3, JOIN, 'extr_tete_M8_ref', [v], down=True)
    skn = sketch(c, plane_z(c, -6.5, 'ecrou_M8'), 'ecrou_M8')
    poly(skn, hexagon(skn, 0, 0, 13))
    circle(skn, 0, 0, 8)
    body(extrude(c, profiles(skn, 'ring'), 6.5, NEW, 'extr_ecrou_M8'), 'REF_ecrou_M8_prisonnier', 'FM_METAL')
    return o


def articulation_ref(c, nm, x, y):
    """3x10x4 dans l'oeil, rondelle M3 0,5 mm dessus, vis M3x12 tête bombée par-dessous + rondelle."""
    ring(c, 'REF_3x10x4_' + nm, 10, 3, Z_OEIL0, 4, 'FM_METAL', x=x, y=y)
    vb = body(zcyl(c, 'REF_vis_M3_' + nm, x, y, 3, Z_OEIL0 - 0.5, Z_FUS_HAUT - (Z_OEIL0 - 0.5)), 'REF_vis_M3x12_tete_bombee_' + nm, 'FM_METAL')
    zcyl(c, 'tete_M3_' + nm, x, y, 7, Z_OEIL0 - 2.2, 1.7, JOIN, [vb])


def palonnier():
    o, c = part('D4_IMP_Palonnier')
    # rebord : du plateau des biellettes (-12) jusqu'à 2,3 mm au-dessus de la face des branches de l'étoile
    b = body(zcyl(c, 'colonne', 0, 0, 37, Z_FUS_HAUT, '(%s + %r mm) - (%r mm)' % (FACE, REBORD_SOUS_FACE, Z_FUS_HAUT)), 'palonnier')
    B = [b]
    zcyl(c, 'plateau', 0, 0, 37, Z_BRAS0, Z_FUS_HAUT - Z_BRAS0, JOIN, B)
    zbox(c, 'bras_pal', -6, -R_PAL, 6, 0, Z_BRAS0, 8, JOIN, B)
    zbox(c, 'traverse', -B_PAL, -R_PAL - OEIL / 2, B_PAL, -R_PAL + OEIL / 2, Z_BRAS0, 8, JOIN, B)
    for sx in (-1, 1):
        zcyl(c, 'oeil_%d' % sx, sx * B_PAL, -R_PAL, OEIL, Z_BRAS0, 8, JOIN, B)
        zcyl(c, 'trou_M3_%d' % sx, sx * B_PAL, -R_PAL, 3.4, Z_BRAS0 - 1, 10, CUT, B)
        skn = sketch(c, plane_z(c, Z_FUS_HAUT, 'ecrou_M3_%d' % sx), 'ecrou_M3_%d' % sx)
        poly(skn, hexagon(skn, sx * B_PAL, -R_PAL, 5.8))
        extrude(c, profiles(skn), 2.6, CUT, 'poche_ecrou_M3_%d' % sx, B, down=True)
        joint_point(c, 'pal_%d' % sx, sx * B_PAL, -R_PAL, 0)
        articulation_ref(c, 'palonnier_%s' % ('D' if sx > 0 else 'G'), sx * B_PAL, -R_PAL)
    # palonnier étoile d'origine encastré (jeu 0,25), accès à la vis centrale, 3 vis M2 par-dessous
    zcyl(c, 'acces_vis_centrale', 0, 0, 7, Z_BRAS0 - 1, 25, CUT, B)
    sk = sketch(c, plane_z(c, '%s + %r mm' % (FACE, REBORD_SOUS_FACE), 'empreinte_etoile'), 'empreinte_etoile')
    poly(sk, etoile(JEU_ETOILE))
    extrude(c, profiles(sk), '%r mm' % REBORD_SOUS_FACE, CUT, 'empreinte_etoile', B, down=True)
    for k, a in enumerate(M2_ANGLES):
        x, y = M2_R * math.cos(math.radians(a)), M2_R * math.sin(math.radians(a))
        zcyl(c, 'trou_M2_%d' % k, x, y, 2.4, '%s - 3 mm' % FACE, 3.2, CUT, B)
        zcyl(c, 'puits_M2_%d' % k, x, y, 4.5, Z_BRAS0 - 1, '(%s - 3 mm) - (%r mm)' % (FACE, Z_BRAS0 - 1), CUT, B)
    sk2 = sketch(c, plane_z(c, FACE, 'etoile_ref'), 'etoile_ref')
    poly(sk2, etoile(0.0))
    e = body(extrude(c, profiles(sk2), '%r mm' % ETOILE_EP, NEW, 'extr_etoile_ref'), 'REF_palonnier_etoile_origine', 'FM_METAL')
    zcyl(c, 'moyeu_ref', 0, 0, MOYEU_D, '%s + %r mm' % (FACE, ETOILE_EP), ETOILE_TOTAL - ETOILE_EP, JOIN, [e])
    return o


def ref_servo():
    o, c = part('D4_REF_Servo_DT996', statut='REFERENCE_ACHAT_MESUREE')
    y0 = -AXE_BOUT
    yc = y0 + SERVO_L / 2
    face = 'd4_planche_ep - %r mm' % round(BOITIER_H - APPUI_OREILLES, 2)   # face côté axe, sous la planche
    b = body(zbox(c, 'boitier', -SERVO_W / 2, y0, SERVO_W / 2, y0 + SERVO_L, face, BOITIER_H), 'REF_servo_boitier', 'FM_PERFBOARD_bleu')
    zbox(c, 'oreilles', -SERVO_W / 2, yc - OREILLES_L / 2, SERVO_W / 2, yc + OREILLES_L / 2, 'd4_planche_ep', OREILLE_EP, JOIN, [b])
    zcyl(c, 'bossage', 0, 0, BOSSAGE_D, face, BOSSAGE_H, JOIN, [b], down=True)
    body(zcyl(c, 'cannelure', 0, 0, CANNELURE_D, '%s + %r mm' % (FACE, ETOILE_TOTAL),
              '(d4_etoile_sous_oreilles - %r mm) - %r mm' % (ETOILE_TOTAL, round(BOITIER_H - APPUI_OREILLES + BOSSAGE_H, 2))),
         'REF_servo_cannelure', 'FM_METAL')
    return o, yc


def planche(yc):
    o, c = part('D4_REF_Planche_bois')
    s = sketch(c, plane_z(c, 0, 'planche'), 'planche')
    lines = rect(s, -142, -60, 142, 390)
    gc, dims = s.geometricConstraints, s.sketchDimensions
    for k, ln in enumerate(lines):
        (gc.addHorizontal if k % 2 == 0 else gc.addVertical)(ln)
    dims.addDistanceDimension(lines.item(0).startSketchPoint, lines.item(0).endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, P(0, -80)).parameter.expression = 'd4_planche_larg'
    dims.addDistanceDimension(lines.item(1).startSketchPoint, lines.item(1).endSketchPoint, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, P(170, 100)).parameter.expression = 'd4_planche_long'
    sym = s.sketchCurves.sketchLines.addByTwoPoints(sp(s, 0, -100), sp(s, 0, 400))
    sym.isConstruction = True
    gc.addVertical(sym)
    gc.addCoincident(s.originPoint, sym)
    gc.addSymmetry(lines.item(0).startSketchPoint, lines.item(0).endSketchPoint, sym)
    dims.addDistanceDimension(s.originPoint, lines.item(0).startSketchPoint, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, P(-170, -30)).parameter.expression = 'd4_planche_avant'
    bb = body(extrude(c, profiles(s, 'largest'), 'd4_planche_ep', NEW, 'extr_planche'), 'REF_planche_bois_non_imprimee', 'Opaque(171,152,82)')
    # découpe du servo (boîtier + 0,5 mm) ; le Ø26 (ancien palonnier rond) est sans effet avec le palonnier étoile
    sy = ev('d4_servo_y')
    s2 = sketch(c, plane_z(c, -1, 'decoupe_servo'), 'decoupe_servo')
    r = s2.sketchCurves.sketchLines.addTwoPointRectangle(sp(s2, -10.5, sy - AXE_BOUT - 0.5), sp(s2, 10.5, sy - AXE_BOUT + SERVO_L + 0.5))
    g2, d2 = s2.geometricConstraints, s2.sketchDimensions
    for k, ln in enumerate(r):
        (g2.addHorizontal if k % 2 == 0 else g2.addVertical)(ln)
    cc = s2.sketchCurves.sketchCircles.addByCenterRadius(sp(s2, 0, sy), 1.3)
    d2.addDiameterDimension(cc, sp(s2, 15, sy + 15))
    d2.addDistanceDimension(s2.originPoint, cc.centerSketchPoint, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, sp(s2, -30, sy / 2)).parameter.expression = 'd4_servo_y'
    g2.addVerticalPoints(s2.originPoint, cc.centerSketchPoint)
    d2.addDistanceDimension(r.item(0).startSketchPoint, r.item(0).endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, sp(s2, 0, sy - 20))
    d2.addDistanceDimension(r.item(1).startSketchPoint, r.item(1).endSketchPoint, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, sp(s2, 20, sy))
    d2.addDistanceDimension(cc.centerSketchPoint, r.item(0).startSketchPoint, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, sp(s2, -20, sy - 5))
    d2.addDistanceDimension(cc.centerSketchPoint, r.item(0).startSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, sp(s2, -5, sy - 15))
    extrude(c, profiles(s2), 'd4_planche_ep + 2 mm', CUT, 'decoupe_servo', [bb])
    bb.opacity = 0.35
    return o


def fixe(g, o, nom, ox=None, oy=None):
    ji = g.joints.createInput(origin_geo(o), adsk.fusion.JointGeometry.createByPoint(g.originConstructionPoint))
    ji.setAsRigidJointMotion()
    j = g.joints.add(ji)
    j.name = nom
    if ox:
        j.offsetX.expression = ox
    if oy:
        j.offsetY.expression = oy
    j.offset.expression = 'd4_roue_ar_d / 2 + d4_axe_ar_sous_planche'
    return j


def lie(g, o1, nom, pivot=False, o2=None, geo2=None):
    ji = g.joints.createInput(origin_geo(o1), geo2 or origin_geo(o2))
    if pivot:
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    else:
        ji.setAsRigidJointMotion()
    j = g.joints.add(ji)
    j.name = nom
    return j


def borne(j, deg):
    lim = j.jointMotion.rotationLimits
    lim.isMinimumValueEnabled = True
    lim.isMaximumValueEnabled = True
    lim.minimumValue = math.radians(-deg)
    lim.maximumValue = math.radians(deg)


def pt(o, nom):
    return next(x for x in o.component.sketches if x.name == 'esq_jp_' + nom).sketchPoints.item(1)


def biellette(s, tag):
    """Oeil côté fusée à l'origine, autre oeil à -s * d4_biellette_L ; yeux 4 mm (3x10x4),
    barre épaissie de 2 mm vers le bas à 7,5 mm des centres."""
    o, c = part('D4_IMP_Biellette_' + tag)
    L = ev('d4_biellette_L')
    sk = sketch(c, plane_z(c, Z_OEIL0, 'biellette'), 'biellette')
    lines, circles = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchCircles
    gc, dims = sk.geometricConstraints, sk.sketchDimensions
    axe = lines.addByTwoPoints(sp(sk, 0, 0), sp(sk, -s * L, 0))
    axe.isConstruction = True
    gc.addCoincident(axe.startSketchPoint, sk.originPoint)
    gc.addHorizontal(axe)
    dims.addDistanceDimension(axe.startSketchPoint, axe.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, sp(sk, -s * L / 2, -14)).parameter.expression = 'd4_biellette_L'
    for p in (axe.startSketchPoint, axe.endSketchPoint):
        for dia in (OEIL, SIEGE_3104):
            cc = circles.addByCenterRadius(p.geometry, dia / 20)
            gc.addCoincident(cc.centerSketchPoint, p)
            dims.addDiameterDimension(cc, adsk.core.Point3D.create(p.geometry.x + 0.9, p.geometry.y + 0.9, 0))
    w = 4.0
    r = lines.addTwoPointRectangle(sp(sk, 0, -w), sp(sk, -s * L, w))
    for k, ln in enumerate(r):
        (gc.addHorizontal if k % 2 == 0 else gc.addVertical)(ln)
    v = sorted([r.item(1), r.item(3)], key=lambda ln: abs(ln.startSketchPoint.geometry.x))
    gc.addMidPoint(axe.startSketchPoint, v[0])
    gc.addMidPoint(axe.endSketchPoint, v[1])
    dims.addDistanceDimension(v[1].startSketchPoint, v[1].endSketchPoint, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, sp(sk, -s * (L + 10), 0))
    centres = [(0.0, 0.0), (-s * L / 10, 0.0)]
    rs = SIEGE_3104 / 20 + 0.001
    def pts_contour(p):
        for lp in p.profileLoops:
            for pc in lp.profileCurves:
                e = pc.geometry.evaluator
                ok, t0, t1 = e.getParameterExtents()
                for k in range(7):
                    ok, q = e.getPointAtParameter(t0 + (t1 - t0) * k / 6)
                    yield q
    def hors_logement(p):
        q = list(pts_contour(p))
        return not any(all(math.hypot(a.x - cx, a.y - cy) <= rs for a in q) for cx, cy in centres)
    b = body(extrude(c, profiles(sk, hors_logement), 4, NEW, 'extr_biellette'), 'biellette_' + tag)
    s2 = sketch(c, plane_z(c, Z_OEIL0, 'barre'), 'barre')
    r2 = s2.sketchCurves.sketchLines.addTwoPointRectangle(sp(s2, -s * 7.5, -w), sp(s2, -s * (L - 7.5), w))
    g2, d2 = s2.geometricConstraints, s2.sketchDimensions
    for k, ln in enumerate(r2):
        (g2.addHorizontal if k % 2 == 0 else g2.addVertical)(ln)
    vv = sorted([r2.item(1), r2.item(3)], key=lambda ln: abs(ln.startSketchPoint.geometry.x))
    mid = s2.sketchCurves.sketchLines.addByTwoPoints(sp(s2, 0, 0), sp(s2, -s * 7.5, 0))
    mid.isConstruction = True
    g2.addCoincident(mid.startSketchPoint, s2.originPoint)
    g2.addHorizontal(mid)
    g2.addMidPoint(mid.endSketchPoint, vv[0])
    d2.addDistanceDimension(mid.startSketchPoint, mid.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, sp(s2, -s * 4, -10)).parameter.expression = '7.5 mm'
    d2.addDistanceDimension(vv[0].startSketchPoint, vv[1].startSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, sp(s2, -s * L / 2, 10)).parameter.expression = 'd4_biellette_L - 15 mm'
    d2.addDistanceDimension(vv[0].startSketchPoint, vv[0].endSketchPoint, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, sp(s2, -s * 3, 0)).parameter.expression = '8 mm'
    extrude(c, profiles(s2), 2, JOIN, 'extr_barre', [b], down=True)
    articulation_ref(c, 'fusee', 0.0, 0.0)
    print('biellette', tag, sk.isFullyConstrained, s2.isFullyConstrained, round(L, 2))
    return o


def run(_context: str):
    d = design()
    d.activateRootComponent()
    for n, e, cmt in PARAMS:
        param(n, e, 'V04 | ' + cmt)
    for o in d.rootComponent.occurrences:
        if o.component.name.startswith('FM_Direction_simple_V03'):
            o.isLightBulbOn = False
    g = group().component
    for s, tag in ((+1, 'D'), (-1, 'G')):
        os_ = support(s, tag)
        of = fusee(s, tag)
        oe = entretoises(tag)
        oq = pivot_ref(tag)
        fixe(g, os_, 'D4_place_support_' + tag, ox=('d4_E / 2' if s > 0 else '-d4_E / 2'))
        borne(lie(g, of, 'D4_pivot_' + tag, True, o2=os_), 40)
        lie(g, oe, 'D4_entretoises_' + tag, o2=of)
        lie(g, oq, 'D4_vis_pivot_' + tag, o2=os_)
    osv, yc = ref_servo()
    fixe(g, osv, 'D4_place_servo', oy='d4_servo_y')
    op = palonnier()
    borne(lie(g, op, 'D4_SERVO_DIRECTION', True, o2=osv), 31)
    for s, tag in ((+1, 'D'), (-1, 'G')):
        ol = biellette(s, tag)
        of = occ('D4_IMP_Fusee_' + tag)
        lie(g, ol, 'D4_rotule_bras_' + tag, True, geo2=point_geo(pt(of, 'bras'), of))
    for s, tag in ((+1, 'D'), (-1, 'G')):
        ol = occ('D4_IMP_Biellette_' + tag)
        ji = g.asBuiltJoints.createInput(ol, op, point_geo(pt(op, 'pal_%d' % s), op))
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        g.asBuiltJoints.add(ji).name = 'D4_rotule_palonnier_' + tag
    fixe(g, planche(yc), 'D4_place_planche')
    print('ok', d.timeline.count, [o.component.name for o in g.occurrences])
