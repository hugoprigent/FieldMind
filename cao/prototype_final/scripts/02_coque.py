from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Prototype final : coque imprimable (A1 mini, 180 × 180 × 180), en 12 pièces, sans vis apparente.

Forme, dans l'esprit du premier rendu FieldMind :
- caisse à pans coupés de 300 × 506 mm, 85 mm au-dessus de la planche : les flancs arrivent au ras
  des côtés de la planche, les faces avant et arrière la débordent et descendent sur sa tranche
  (on ne voit plus de rebord de bois) ; chanfreins de 20 mm sur les arêtes hautes ;
- nez incliné : face avant de 40 mm puis pente jusqu'à 85 mm, qui passe sous la barre des caméras ;
  grille de ventilation dans la face avant ;
- bande centrale de 100 mm (bleue) du nez à l'arrière : ouverture des deux profilés avec un passage
  de câbles derrière chacun, fenêtre de l'écran et Knob au-dessus de la breadboard ; flancs blancs
  avec ouïes et inscription FIELDMIND en creux.
Découpe : bande centrale (|X| < 50) et flancs, en 4 tronçons (Y = 30, 163, 296).
Assemblage, toutes les vis à l'intérieur (la coque s'assemble à l'envers sur la table) :
- la bande centrale repose sur un rebord de chaque flanc ; ses bossages entrent dans des trous du
  rebord et reçoivent un insert M3 ; vis M3×6 par-dessous avec une rondelle large ;
- à chaque coupe transversale, deux nervures face à face (une par tronçon), boulonnées M3 + écrou ;
- la coque se fixe sur la planche par 10 vis M6 passées par-dessous la planche, dans des écrous M6
  glissés dans des fentes hexagonales au pied de la coque.
Toutes les pièces s'impriment sans support : flancs couchés sur leur paroi extérieure, bandes dessus
contre le plateau, nez debout sur sa face avant.
Réexécutable : supprime d'abord la coque d'un passage précédent.
"""
import adsk.core, adsk.fusion, math, json
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/pf_helpers.py'), encoding='utf-8').read(), 'pf_helpers.py', 'exec'))
DOSSIER = str(_FM_ROOT / 'cao/prototype_final')

XE, T = 150.0, 2.4                  # flancs au ras des côtés de la planche, paroi
YAV, YAR = -63.0, 443.0             # faces avant et arrière, 0,6 mm devant les bords de la planche
JUPE = 5.0                          # faces avant et arrière descendues sur la tranche de la planche
H, CH = 85.0, 20.0                  # hauteur au-dessus de la planche, chanfrein des arêtes hautes
NEZ_H, NEZ_Y = 40.0, 30.0           # face avant, fin de la pente du nez
AR_CH, COIN = 20.0, 3.0             # chanfrein haut arrière ; pan des coins (couvre le coin de la planche)
XS = 50.0                           # coutures entre bande centrale et flancs
YS = [NEZ_Y, 163.0, 296.0]          # coupes transversales
REBORD, REBORD_L = 6.5, 14.0        # rebord des flancs sous la bande centrale
BOSS_D, BOSS_TROU, BOSS_JEU = 8.0, 8.6, 1.0   # bossages de la bande dans les trous du rebord
INSERT_D, INSERT_P = 4.0, 5.5
NERVURE, NERVURE_EP = 8.0, 4.0      # nervures des coupes transversales
M6_ECROU, M6_ECROU_H, M6_BASE = 10.2, 5.4, 3.0
PROFIL_CAM = (-35.0, 21.5)          # axe Y du profilé caméras, demi-ouverture
PROFIL_LIDAR = (380.0, 30.5)        # 61 × 61 : la coque passe par-dessus le mât du lidar monté (support et C1)
PASSE_CABLES = (12.0, 14.0)          # demi-largeur et profondeur du passage derrière chaque profilé
PASSE_LIDAR = 10.0                   # passage derrière le mât du lidar, moins profond (chanfrein arrière)
A7_Y0 = 175.0                        # carte A7 au-dessus de la breadboard
A7_TROUS = [(sx * 17.5, A7_Y0 + dy) for sx in (1, -1) for dy in (2.5, 57.5)]
A7_ZDESSUS = 68.6                   # dessus de la carte A7 au-dessus de la planche (01_composants.py)
# écran OLED 0,96" (plan du fabricant) : fenêtre = zone affichée + 0,6 mm, biseautée à 30° vers le dessus ;
# le module est vissé sous la bande par 4 vis M2 (trous Ø2 à 2 mm des bords)
OLED_Y0 = A7_Y0 + 4.37
ECRAN = (0.0, OLED_Y0 + 4.27 + 2.10 + 10.86 / 2, 21.74 + 1.2, 10.86 + 1.2)
ECRAN_BISEAU = 30.0
OLED_ZPCB = A7_ZDESSUS + 8.5 + 2.5 + 1.2      # dessus du circuit de l'écran (support femelle + entretoise des broches)
OLED_TROUS = [(sx * 11.65, OLED_Y0 + dy) for sx in (1, -1) for dy in (2.0, 25.8)]
# Modulino Knob (modèle officiel) : axe Ø6, 4 trous Ø3,2 à 32 × 16, vissé sous la bande par 4 vis M3
KNOB = (0.0, 257.7)
KNOB_ZPCB = 71.0
KNOB_TROUS = [(KNOB[0] + sx * 16.0, KNOB[1] + sy * 8.0) for sx in (1, -1) for sy in (1, -1)]
X_VIS_BANDE = XS - REBORD_L / 2      # 43
VIS_BANDE = {2: (55, 100, 145), 3: (185, 222, 275), 4: (318, 362, 400)}
FIX_M6 = [(1, 'cote', -40.0), (1, 'cote', 100.0), (1, 'cote', 230.0),
          (1, 'avant', 75.0), (1, 'arriere', 90.0)]      # par côté (miroir en -X)


def z_nez(y):
    return ZB + NEZ_H + (H - NEZ_H) * (y - YAV) / (NEZ_Y - YAV)


def cyl_y(c, name, x, y0, y1, z, d, op, bodies):
    s = sketch(c, plane_y(c, (y0 + y1) / 2, name), name)
    s.sketchCurves.sketchCircles.addByCenterRadius(s.modelToSketchSpace(P(x, (y0 + y1) / 2, z)), d / 20)
    return sym(c, profiles(s), abs(y1 - y0), op, 'extr_' + name, bodies)


def coupe_y(poly, ya, yb):
    """Garde la partie d'un polygone (y, z) comprise entre ya et yb (Sutherland-Hodgman)."""
    def tranche(pts, dedans, coupe):
        out = []
        for i in range(len(pts)):
            a, b = pts[i - 1], pts[i]
            if dedans(b):
                if not dedans(a):
                    out.append(coupe(a, b))
                out.append(b)
            elif dedans(a):
                out.append(coupe(a, b))
        return out
    def en(y0):
        return lambda a, b: (y0, a[1] + (b[1] - a[1]) * (y0 - a[0]) / (b[0] - a[0]))
    p = tranche(poly, lambda q: q[0] >= ya - 1e-9, en(ya))
    p = tranche(p, lambda q: q[0] <= yb + 1e-9, en(yb))
    propre = []
    for q in p:
        if not propre or math.hypot(q[0] - propre[-1][0], q[1] - propre[-1][1]) > 1e-3:
            propre.append(q)
    if math.hypot(propre[0][0] - propre[-1][0], propre[0][1] - propre[-1][1]) < 1e-3:
        propre.pop()
    return propre


def troncon(y):
    return 1 if y < YS[0] else 2 if y < YS[1] else 3 if y < YS[2] else 4


def face_normale(b, n):
    return max((f for f in b.faces if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType),
               key=lambda f: sum(a * w for a, w in zip(f.evaluator.getNormalAtPoint(f.pointOnFace)[1].asArray(), n)))


def bleu_pla():
    """Apparence bleue (PLA) créée au besoin à partir du blanc."""
    d = design()
    a = d.appearances.itemByName('FM_PLA_bleu')
    if a:
        return 'FM_PLA_bleu'
    try:
        a = d.appearances.addByCopy(d.appearances.itemByName(BLANC), 'FM_PLA_bleu')
        for i in range(a.appearanceProperties.count):
            p = a.appearanceProperties.item(i)
            if p.objectType == adsk.core.ColorProperty.classType() and p.value:
                p.value = adsk.core.Color.create(25, 95, 200, 255)
        return 'FM_PLA_bleu'
    except Exception:
        return BLEU


def ecrou_m6(c, piece_, sx, genre, pos):
    """Bloc au pied de la coque : trou Ø6,6 vertical, fente hexagonale pour un écrou M6 glissé depuis
    l'intérieur, 3 mm de base sous l'écrou. Vis M6×16 par-dessous la planche."""
    L, W, Hb = 16.0, 18.0, 12.0
    if genre == 'cote':
        x_mur, y = sx * (XE - T), pos
        xc = x_mur - sx * L / 2
        boite(c, 'bloc_m6_%+d_%d' % (sx, pos), x_mur + sx * 0.01, y - W / 2, x_mur - sx * L, y + W / 2, ZB, Hb, op=JOIN, bodies=[piece_])
        fente = (xc + sx * 5.8, y - M6_ECROU / 2, x_mur - sx * (L + 1), y + M6_ECROU / 2)
        centre = (xc, y)
    else:
        sy = -1 if genre == 'avant' else 1
        y_mur = YAV + T if genre == 'avant' else YAR - T
        x = sx * pos
        yc = y_mur - sy * L / 2
        boite(c, 'bloc_m6_%s_%+d' % (genre, sx), x - W / 2, y_mur + sy * 0.01, x + W / 2, y_mur - sy * L, ZB, Hb, op=JOIN, bodies=[piece_])
        fente = (x - M6_ECROU / 2, yc + sy * 5.8, x + M6_ECROU / 2, y_mur - sy * (L + 1))
        centre = (x, yc)
    cyl_z(c, 'trou_m6_%s_%+d_%d' % (genre, sx, pos), centre[0], centre[1], 6.6, ZB - 1, Hb + 2, op=CUT, bodies=[piece_])
    boite(c, 'fente_m6_%s_%+d_%d' % (genre, sx, pos), *fente, ZB + M6_BASE, M6_ECROU_H, op=CUT, bodies=[piece_])
    return (round(centre[0], 2), round(centre[1], 2))


def run(_context: str):
    d = design()
    nettoyer(['PF_IMP_Coque'])
    _, c = piece('PF_IMP_Coque', 'IMPRIME_PETG_PLA')
    bleu = bleu_pla()
    # ---- volume extérieur
    sec = [(-XE, ZB), (XE, ZB), (XE, ZB + H - CH), (XE - CH, ZB + H), (-XE + CH, ZB + H), (-XE, ZB + H - CH)]
    b = body(prisme_y(c, 'volume', (YAV + YAR) / 2, sec, YAR - YAV), 'coque', BLANC)
    prisme_x(c, 'coupe_nez', 0, [(YAV - 10, z_nez(YAV - 10)), (NEZ_Y + 10, z_nez(NEZ_Y + 10)), (NEZ_Y + 10, ZB + 250), (YAV - 10, ZB + 250)], 400, CUT, [b])
    prisme_x(c, 'coupe_arriere', 0, [(YAR - AR_CH - 10, ZB + H + 10), (YAR + 10, ZB + H - AR_CH - 10), (YAR + 10, ZB + 250), (YAR - AR_CH - 10, ZB + 250)], 400, CUT, [b])
    for sx in (1, -1):
        for yc, sy in ((YAV, 1), (YAR, -1)):
            zpoly(c, 'coin_%+d_%s' % (sx, 'av' if sy > 0 else 'ar'),
                  [(sx * (XE - COIN - 5), yc - sy * 5), (sx * (XE + 5), yc - sy * 5), (sx * (XE + 5), yc + sy * (COIN + 5))], ZB - 1, H + 2, CUT, [b])
    # ---- coque creuse (fond ouvert)
    fond = adsk.core.ObjectCollection.create()
    fond.add(face_normale(b, (0, 0, -1)))
    si = c.features.shellFeatures.createInput(fond, False)
    si.insideThickness = V(T)
    c.features.shellFeatures.add(si).name = 'coque_creuse'
    # ---- jupes avant et arrière sur la tranche de la planche
    for nom, y0, y1 in (('jupe_avant', YAV, YAV + T), ('jupe_arriere', YAR - T, YAR)):
        boite(c, nom, -(XE - COIN), y0, XE - COIN, y1, ZB - JUPE, JUPE + 0.01, op=JOIN, bodies=[b])
    # ---- ouvertures
    for nom, (yc, a), pp in (('profile_cameras', PROFIL_CAM, PASSE_CABLES[1]), ('profile_lidar', PROFIL_LIDAR, PASSE_LIDAR)):
        boite(c, 'ouv_' + nom, -a, yc - a, a, yc + a, ZB + 20, 100, op=CUT, bodies=[b])
        boite(c, 'ouv_cables_' + nom, -PASSE_CABLES[0], yc + a - 1, PASSE_CABLES[0], yc + a + pp, ZB + 20, 100, op=CUT, bodies=[b])
    ex, ey, ew, eh = ECRAN
    s_ = sketch(c, plane_z(c, ZB + H - T, 'ouv_ecran'), 'ouv_ecran')
    rect(s_, ex - ew / 2, ey - eh / 2, ex + ew / 2, ey + eh / 2)
    extrude(c, profiles(s_), T + 1, CUT, 'extr_ouv_ecran', [b], taper_deg=ECRAN_BISEAU)    # s'élargit vers le dessus
    cyl_z(c, 'ouv_knob', KNOB[0], KNOB[1], 7, ZB + 70, 30, op=CUT, bodies=[b])
    for k, z in enumerate((8, 15, 22, 29)):   # grille avant
        boite(c, 'grille_%d' % k, -36, YAV - 1, 36, YAV + T + 1, ZB + z, 3.5, op=CUT, bodies=[b])
    ouies = [(-1, 105, 150), (-1, 175, 255), (-1, 345, 400), (1, 105, 150), (1, 180, 255), (1, 345, 400)]
    for k, (sx, y0, y1) in enumerate(ouies):
        for j, z in enumerate((16, 24, 32, 40)):
            boite(c, 'ouie_%d_%d' % (k, j), sx * (XE - T - 1), y0, sx * (XE + 1), y1, ZB + z, 4, op=CUT, bodies=[b])
    # ---- découpe en 12 pièces
    plans = [plane_x(c, sx * XS, 'coupe_x%+d' % sx) for sx in (1, -1)] + [plane_y(c, y, 'coupe_y%d' % y) for y in YS]
    for pl in plans:
        tous = adsk.core.ObjectCollection.create()
        for bb in c.bRepBodies:
            tous.add(bb)
        c.features.splitBodyFeatures.add(c.features.splitBodyFeatures.createInput(tous, pl, True))
    P_ = {}
    for bb in list(c.bRepBodies):
        bx = bb.boundingBox
        cx, cy = (bx.minPoint.x + bx.maxPoint.x) * 5, (bx.minPoint.y + bx.maxPoint.y) * 5
        cote = 'G' if cx > XS else 'D' if cx < -XS else 'C'
        nom = 'coque_%s%d' % (cote, troncon(cy))
        bb.name = nom
        paint(bb, bleu if cote == 'C' else BLANC)
        P_[nom] = bb
    assert len(P_) == 12, sorted(P_)
    # ---- rebords des flancs sous la bande centrale
    profil = [(YAR, ZB + 3), (YAR, ZB + H - AR_CH), (YAR - AR_CH, ZB + H), (NEZ_Y, ZB + H), (YAV, ZB + NEZ_H), (YAV, ZB + 24)]
    rebord = bande(profil, T, T + REBORD)
    bornes = [YAV - 1, YS[0], YS[1], YS[2], YAR + 1]
    for sx, cote in ((1, 'G'), (-1, 'D')):
        for k in range(4):
            pts = coupe_y(rebord, bornes[k], bornes[k + 1])
            s = sketch(c, plane_x(c, sx * XS, 'rebord_%s%d' % (cote, k + 1)), 'rebord_%s%d' % (cote, k + 1))
            poly3(s, [(sx * XS, y, z) for y, z in pts])
            extrude(c, profiles(s), REBORD_L, JOIN, 'extr_rebord_%s%d' % (cote, k + 1), [P_['coque_%s%d' % (cote, k + 1)]], down=sx > 0)
    # ---- bande centrale : bossages dans les trous des rebords, insert M3, vis par-dessous
    z_dessous = ZB + H - T
    vis_bande = 0
    for sx, cote in ((1, 'G'), (-1, 'D')):
        x = sx * X_VIS_BANDE
        for k, ys in VIS_BANDE.items():
            for y in ys:
                n = '%s%d_%d' % (cote, k, y)
                cyl_z(c, 'trou_rebord_' + n, x, y, BOSS_TROU, z_dessous - REBORD - 1, REBORD + 1, op=CUT, bodies=[P_['coque_%s%d' % (cote, k)]])
                cyl_z(c, 'boss_bande_' + n, x, y, BOSS_D, z_dessous - REBORD + BOSS_JEU, REBORD - BOSS_JEU + 0.01, op=JOIN, bodies=[P_['coque_C%d' % k]])
                cyl_z(c, 'ins_bande_' + n, x, y, INSERT_D, z_dessous - REBORD + BOSS_JEU - 0.1, INSERT_P + 0.1, op=CUT, bodies=[P_['coque_C%d' % k]])
                vis_bande += 1
        # nez : bossage horizontal derrière la face avant, dans le rebord de la face avant
        zf, yf = ZB + 32, YAV + T
        cyl_y(c, 'trou_rebord_%s1_avant' % cote, x, yf - 0.5, yf + REBORD + 1, zf, BOSS_TROU, CUT, [P_['coque_%s1' % cote]])
        cyl_y(c, 'boss_bande_%s1_avant' % cote, x, yf - 0.01, yf + REBORD - BOSS_JEU, zf, BOSS_D, JOIN, [P_['coque_C1']])
        cyl_y(c, 'ins_bande_%s1_avant' % cote, x, yf + REBORD - BOSS_JEU - INSERT_P, yf + REBORD - BOSS_JEU + 0.1, zf, INSERT_D, CUT, [P_['coque_C1']])
        vis_bande += 1
    # nez : languette sous l'avant de la bande C2, qui retient l'arrière du nez. Elle part de la pente
    # (4,5 mm avant l'arête) et reste 0,4 mm sous la bande : sous l'arête, la paroi intérieure de la
    # pente déborde de 0,55 mm sur C2 et la languette ne doit pas la toucher.
    boite(c, 'languette_nez', -(XS - REBORD_L - 0.5), NEZ_Y - 4.5, XS - REBORD_L - 0.5, NEZ_Y + 10, z_dessous - 2.8, 2.4, op=JOIN, bodies=[P_['coque_C1']])
    # ---- nervures des coupes transversales (une par tronçon, face à face), boulons M3 + écrous
    boulons = 0
    for sx, cote in ((1, 'G'), (-1, 'D')):
        if sx > 0:
            sec_fl = [(XE, ZB + 3), (XE, ZB + H - CH), (XE - CH, ZB + H), (XS, ZB + H)]
        else:
            sec_fl = [(-XS, ZB + H), (-XE + CH, ZB + H), (-XE, ZB + H - CH), (-XE, ZB + 3)]
        nerv = bande(sec_fl, T - 0.5, T + NERVURE)
        for k, ys in enumerate(YS, start=1):
            av, ar = P_['coque_%s%d' % (cote, k)], P_['coque_%s%d' % (cote, k + 1)]
            prisme_y(c, 'nervure_%s%d_ar' % (cote, k), ys - NERVURE_EP / 2, nerv, NERVURE_EP, JOIN, [av])
            prisme_y(c, 'nervure_%s%d_av' % (cote, k + 1), ys + NERVURE_EP / 2, nerv, NERVURE_EP, JOIN, [ar])
            for nom, x, z in (('paroi', sx * (XE - T - NERVURE / 2), ZB + 35), ('dessus', sx * 95, ZB + H - T - NERVURE / 2)):
                for pc in (av, ar):
                    cyl_y(c, 'boulon_%s_%s%d_%s' % (nom, cote, k, pc.name[-1]), x, ys - NERVURE_EP - 0.5, ys + NERVURE_EP + 0.5, z, 3.4, CUT, [pc])
                boulons += 1
    # ---- fixation sur la planche : écrous M6 dans des fentes, vis par-dessous
    fix = []
    for sx, cote in ((1, 'G'), (-1, 'D')):
        for _, genre, pos in FIX_M6:
            if genre == 'cote':
                pc = P_['coque_%s%d' % (cote, troncon(pos))]
            elif genre == 'avant':
                pc = P_['coque_%s1' % cote]
            else:
                pc = P_['coque_%s4' % cote]
            fix.append(ecrou_m6(c, pc, sx, genre, pos))
    # ---- bossages de la carte A7 sous la bande C3 (vis M3 directement dans le plastique)
    for k, (x, y) in enumerate(A7_TROUS):
        cyl_z(c, 'boss_A7_%d' % k, x, y, 5.0, ZB + A7_ZDESSUS, H - T - A7_ZDESSUS, op=JOIN, bodies=[P_['coque_C3']])
        cyl_z(c, 'trou_A7_%d' % k, x, y, 2.6, ZB + A7_ZDESSUS - 1, 10, op=CUT, bodies=[P_['coque_C3']])
    # ---- écran vissé sous la bande C3 : 4 bossages Ø4, vis M2×4 dans des trous Ø1,6
    for k, (x, y) in enumerate(OLED_TROUS):
        cyl_z(c, 'boss_ecran_%d' % k, x, y, 4.0, ZB + OLED_ZPCB, H - T - OLED_ZPCB, op=JOIN, bodies=[P_['coque_C3']])
        cyl_z(c, 'trou_ecran_%d' % k, x, y, 1.6, ZB + OLED_ZPCB - 1, H - 0.8 - OLED_ZPCB + 1, op=CUT, bodies=[P_['coque_C3']])
    # ---- Knob vissé sous la bande C3 : 4 bossages Ø6, vis M3×12 dans des trous Ø2,6
    for k, (x, y) in enumerate(KNOB_TROUS):
        cyl_z(c, 'boss_knob_%d' % k, x, y, 6.0, ZB + KNOB_ZPCB, H - T - KNOB_ZPCB, op=JOIN, bodies=[P_['coque_C3']])
        cyl_z(c, 'trou_knob_%d' % k, x, y, 2.6, ZB + KNOB_ZPCB - 1, H - 1.0 - KNOB_ZPCB + 1, op=CUT, bodies=[P_['coque_C3']])
    # ---- inscription en creux sur les flancs du tronçon 3
    txt = []
    for sx, cote in ((1, 'G'), (-1, 'D')):
        try:
            s = sketch(c, plane_x(c, sx * XE, 'texte_%s' % cote), 'texte_%s' % cote)
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
            y_debut = 229.5 - sx * 52        # début du mot, côté avant à gauche, côté arrière à droite
            ti = s.sketchTexts.createInput('FIELDMIND', 1.2, s.modelToSketchSpace(P(sx * XE, y_debut, ZB + 50)))
            ti.angle = ang
            ti.fontName = 'Arial Black'
            if miroir:
                ti.isHorizontalFlip = True
            st = s.sketchTexts.add(ti)
            ei = c.features.extrudeFeatures.createInput(st, CUT)
            ei.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(V(0.8)),
                                adsk.fusion.ExtentDirections.NegativeExtentDirection if sx > 0 else adsk.fusion.ExtentDirections.PositiveExtentDirection)
            ei.participantBodies = [P_['coque_%s3' % cote]]
            c.features.extrudeFeatures.add(ei).name = 'texte_%s' % cote
            txt.append(cote)
        except Exception as e:
            txt.append('%s : %s' % (cote, str(e)[:120]))
    # ---- bilan
    bilan = {'pieces': {}, 'fixations_m6': fix, 'vis_bande': vis_bande, 'boulons_nervures': boulons, 'texte': txt, 'bleu': bleu}
    for nom, bb in sorted(P_.items()):
        bx = bb.boundingBox
        bilan['pieces'][nom] = {'taille_mm': [round((bx.maxPoint.asArray()[i] - bx.minPoint.asArray()[i]) * 10, 1) for i in range(3)],
                                'volume_cm3': round(bb.volume, 1), 'morceaux': bb.lumps.count}
    json.dump(bilan, open(DOSSIER + '/coque_bilan.json', 'w', encoding='utf-8'), indent=1, ensure_ascii=False)
    print(json.dumps(bilan, ensure_ascii=False)[:3000])
    tl = d.timeline
    print('fonctions KO', [tl.item(i).name for i in range(tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup])
