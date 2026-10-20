from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Servo DT996 mesuré et palonnier en étoile, modification en place (sans reconstruire, pour garder
les modifications faites à la main dans le document).

Mesures d'Hugo (15/10/2026), servo debout, depuis la base :
  boîtier 40,4 x 20 x 39,3 ; oreilles de 2,2 mm, dessous à 28,5 (face d'appui sur la planche à 30,7) ;
  bossage rond jusqu'à 40,8 ; cannelure jusqu'à 44,8 ; palonnier monté jusqu'à 49.
Palonnier en étoile à 6 branches (photo au pied à coulisse) : pointes à 15,7 mm du centre, branches
de 7,0 mm de large jusqu'à 7 mm du centre puis 4,8 mm à 13,3 mm, bout arrondi R2,4, épaisseur 2,6 ;
moyeu côté servo, épaisseur totale 5,5 ; trous Ø1,6 à 7,3 / 10,3 / 13,3 mm du centre.

Le servo est tête en bas, oreilles posées sur la planche : la face des branches de l'étoile est à
d4_etoile_sous_oreilles (49 - 30,7 = 18,3 mm) sous la face d'appui des oreilles. Le palonnier imprimé
reçoit l'étoile dans une empreinte (jeu 0,25 mm) creusée dans son plateau et un rebord, et y est tenu
par 3 vis M2 par-dessous ; l'entraînement se fait par la forme.
"""
import adsk.core, adsk.fusion, math
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
GROUP = 'FM_Direction_V04'

SERVO_L, SERVO_W, BOITIER_H = 40.4, 20.0, 39.3
APPUI_OREILLES = 28.5 + 2.2          # base -> face des oreilles posée sur la planche
OREILLE_EP, OREILLES_L, AXE_BOUT = 2.2, 54.0, 10.0
BOSSAGE_D, BOSSAGE_H = 12.0, 1.5     # diamètre du bossage non mesuré (à vérifier)
CANNELURE_D = 6.0
ETOILE_EP, ETOILE_TOTAL, MOYEU_D = 2.6, 5.5, 9.0
R_POINTE, R_BOUT, U_BOUT, DEMI_PIED, U_PIED = 15.7, 2.4, 13.3, 3.5, 7.0
JEU_ETOILE = 0.25
PHASE = 30.0                         # branches à 30° + k x 60° (deux branches sur l'axe Y)
M2_R, M2_ANGLES = 10.3, (90, 210, 330)
REBORD_SOUS_FACE = 2.3               # le rebord tient 2,3 mm des 2,6 mm de branche
Z_FUS_HAUT, Z_BRAS0 = -12.0, -20.0
FACE = 'd4_planche_ep - d4_etoile_sous_oreilles'           # face des branches (côté palonnier imprimé)


def etoile(jeu=0.0, n_bout=10):
    """Contour de l'étoile (mm, repère du servo), décalé de `jeu` vers l'extérieur."""
    hw = lambda u: (DEMI_PIED if u <= U_PIED else DEMI_PIED - (DEMI_PIED - R_BOUT) * (u - U_PIED) / (U_BOUT - U_PIED)) + jeu
    u_creux = (2 * (DEMI_PIED + jeu)) * math.cos(math.radians(30))   # creux sur la bissectrice
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


def supprimer(c, noms):
    for coll in (c.features.extrudeFeatures, c.sketches, c.constructionPlanes):
        for x in [coll.item(i) for i in range(coll.count)]:
            if any(x.name == n or x.name in ('extr_' + n, 'esq_' + n, 'plan_' + n) for n in noms):
                x.deleteMe()


def rayon_esquisse(f, d_mm):
    sk = f.profile.parentSketch
    sk.sketchCurves.sketchCircles.item(0).radius = d_mm / 20


def distance(f, expr):
    adsk.fusion.DistanceExtentDefinition.cast(f.extentOne).distance.expression = expr


def run(_context: str):
    d = design()
    param('d4_etoile_sous_oreilles', '18.3 mm',
          'V04 | Face des branches du palonnier etoile sous la face d appui des oreilles (49 - 30,7 mesure servo DT996)')
    comp = lambda n: next(x for x in d.allComponents if x.name == n)

    # ---- servo de référence
    sv = comp('D4_REF_Servo_DT996')
    supprimer(sv, ('boitier', 'oreilles', 'cannelure', 'palonnier_rond', 'bossage'))
    y0 = -AXE_BOUT
    yc = y0 + SERVO_L / 2
    face_boitier = 'd4_planche_ep - %r mm' % round(BOITIER_H - APPUI_OREILLES, 2)
    b = body(zbox(sv, 'boitier', -SERVO_W / 2, y0, SERVO_W / 2, y0 + SERVO_L, face_boitier, BOITIER_H), 'REF_servo_boitier', 'FM_PERFBOARD_bleu')
    zbox(sv, 'oreilles', -SERVO_W / 2, yc - OREILLES_L / 2, SERVO_W / 2, yc + OREILLES_L / 2, 'd4_planche_ep', OREILLE_EP, JOIN, [b])
    zcyl(sv, 'bossage', 0, 0, BOSSAGE_D, face_boitier, BOSSAGE_H, JOIN, [b], down=True)
    # cannelure : du bossage jusqu'au moyeu de l'étoile (elle continue dans le moyeu)
    bas_bossage = '%s - %r mm' % (face_boitier, BOSSAGE_H)
    body(zcyl(sv, 'cannelure', 0, 0, CANNELURE_D, '%s + %r mm' % (FACE, ETOILE_TOTAL),
              '(d4_etoile_sous_oreilles - %r mm) - %r mm' % (ETOILE_TOTAL, round(BOITIER_H - APPUI_OREILLES + BOSSAGE_H, 2))),
         'REF_servo_cannelure', 'FM_METAL')

    # ---- palonnier imprimé
    pc = comp('D4_IMP_Palonnier')
    fx = pc.features.extrudeFeatures
    supprimer(pc, ['centrage_disque', 'etoile_ref', 'moyeu_ref', 'empreinte_etoile'] +
              ['trou_M2_%d' % k for k in range(4)] + ['puits_M2_%d' % k for k in range(4)])
    rayon_esquisse(fx.itemByName('extr_plateau'), 37)
    rayon_esquisse(fx.itemByName('extr_colonne'), 37)
    # le rebord (ancienne colonne) monte du plateau jusqu'à 2,3 mm au-dessus de la face des branches
    distance(fx.itemByName('extr_colonne'), '(%s + %r mm) - (%r mm)' % (FACE, REBORD_SOUS_FACE, Z_FUS_HAUT))
    B = [pc.bRepBodies.itemByName('palonnier')]
    sk = sketch(pc, plane_z(pc, '%s + %r mm' % (FACE, REBORD_SOUS_FACE), 'empreinte_etoile'), 'empreinte_etoile')
    poly(sk, etoile(JEU_ETOILE))
    extrude(pc, profiles(sk), '%r mm' % REBORD_SOUS_FACE, CUT, 'empreinte_etoile', B, down=True)
    for k, a in enumerate(M2_ANGLES):
        x, y = M2_R * math.cos(math.radians(a)), M2_R * math.sin(math.radians(a))
        zcyl(pc, 'trou_M2_%d' % k, x, y, 2.4, '%s - 3 mm' % FACE, 3.2, CUT, B)
        zcyl(pc, 'puits_M2_%d' % k, x, y, 4.5, Z_BRAS0 - 1, '(%s - 3 mm) - (%r mm)' % (FACE, Z_BRAS0 - 1), CUT, B)
    # étoile d'origine (référence) : dans le palonnier, elle tourne avec lui
    sk2 = sketch(pc, plane_z(pc, FACE, 'etoile_ref'), 'etoile_ref')
    poly(sk2, etoile(0.0))
    e = body(extrude(pc, profiles(sk2), '%r mm' % ETOILE_EP, NEW, 'extr_etoile_ref'), 'REF_palonnier_etoile_origine', 'FM_METAL')
    zcyl(pc, 'moyeu_ref', 0, 0, MOYEU_D, '%s + %r mm' % (FACE, ETOILE_EP), ETOILE_TOTAL - ETOILE_EP, JOIN, [e])

    # ---- découpe de la planche : boîtier + 0,5 mm
    pl = comp('D4_REF_Planche_bois')
    s = pl.sketches.itemByName('esq_decoupe_servo')
    for i in range(s.sketchDimensions.count):
        p = s.sketchDimensions.item(i).parameter
        if abs(p.value * 10 - 41.0) < 0.01 or abs(p.value * 10 - 41.4) < 0.01:
            p.expression = '%r mm' % (SERVO_L + 1.0)

    tl = d.timeline
    print('fonctions KO', [tl.item(i).name for i in range(tl.count) if tl.item(i).healthState != 0])
    for nom in ('D4_REF_Servo_DT996', 'D4_IMP_Palonnier', 'D4_REF_Planche_bois'):
        for bb_ in comp(nom).bRepBodies:
            bb = bb_.boundingBox
            print(nom, bb_.name, [round(v * 10, 2) for v in (bb.minPoint.x, bb.minPoint.y, bb.minPoint.z, bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z)],
                  'vol', round(bb_.volume, 2))
