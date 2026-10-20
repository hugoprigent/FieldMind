from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Prototype final : composants embarqués posés sur la planche (volumes de référence) et berceau de
batterie imprimé. Réexécutable : supprime d'abord ce qu'un passage précédent a créé.

Seul le matériel documenté comme acheté ou en place est posé (pas d'arrêt d'urgence, de
réarmement, d'interrupteur général, de fusibles ni de relais : Hugo n'en a pas).
Implantation (X depuis l'axe, +X = gauche du rover vu de l'arrière ; Y depuis l'axe des pivots) :
- au centre, la breadboard 830 points (le GPS y est enfiché) ;
- côté -X (droite), le Raspberry Pi 4 dans son boîtier ;
- à l'avant, à gauche du servo de direction, la batterie couchée en long dans son berceau, et en miroir à
  droite du servo, les deux XL4015 (5,1 V pour le Pi, 6 V pour le servo) : du poids sur les roues avant,
  centre de gravité au milieu en largeur (13_centre_de_gravite.py) ;
- de chaque côté du mât du lidar, un contrôleur BLDC, au plus près de son moteur ;
- derrière le mât, l'adaptateur USB du lidar ; sous le dessus de la coque, l'antenne GNSS ;
- sous la bande centrale de la coque, juste au-dessus de la breadboard : la carte A7 (écran derrière
  une fenêtre) et le Knob.
Volumes aux cotes annoncées (ZS-X11H 63 × 45 × 31 ; XL4015 54 × 23,5 ; breadboard 165 × 55) ou
mesurées (batterie 82 × 68 × 51) ; boîtier du Pi, GPS, adaptateur du lidar, moteurs et roues
arrière : à mesurer.
"""
import adsk.core, adsk.fusion, math, json
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/pf_helpers.py'), encoding='utf-8').read(), 'pf_helpers.py', 'exec'))

# (nom, x0, y0, x1, y1, z0 au-dessus de la planche, hauteur, couleur, statut)
BLOCS = [
    ('PF_REF_Raspberry_Pi4_boitier', -122, 105, -54, 201, 0, 32, GRIS_F, 'ENVELOPPE_A_MESURER (boitier du Pi)'),
    ('PF_REF_Breadboard_830', -27.3, 110, 27.3, 275.1, 0, 8.5, BLANC, 'COTES_ELEGOO_830'),
    ('PF_REF_GPS_ATGM336H', -13, 230, 13, 263, 8.5, 6, BLEU, 'ENVELOPPE_A_MESURER'),
    ('PF_REF_Controleur_BLDC_G', -122, 345, -77, 408, 0, 31, GRIS_F, 'COTES_ANNONCEES_ZS-X11H'),
    ('PF_REF_Controleur_BLDC_D', 77, 345, 122, 408, 0, 31, GRIS_F, 'COTES_ANNONCEES_ZS-X11H'),
    ('PF_REF_Adaptateur_USB_C1', -20, 401, 20, 426, 0, 6, BLEU, 'ENVELOPPE_A_MESURER'),
]
BATT = (68.0, 82.0, 51.0)          # couchée, en long : 68 en X, 82 en Y, 51 de haut
BATT_X, BATT_Y = 50.5, 63.5        # à l'avant, à gauche du servo de direction : du poids sur les roues avant (équilibrage)
BERCEAU_JEU, BERCEAU_EP, BERCEAU_FOND, BERCEAU_H = 1.0, 2.5, 5.0, 30.0   # fond de 5 : écrous M3 noyés, vis par-dessous
SANGLE_L, SANGLE_H = 22.0, 3.5
A7_Y0, A7_ZDESSUS = 175.0, 68.6    # carte A7 au-dessus de la breadboard : bord avant (rangée 0) et dessus de la plaque
# Écran OLED 0,96" I2C : cotes du plan du fabricant (LCDwiki MC096GX, https://www.lcdwiki.com/0.96inch_OLED_Module_MC096GX)
OLED_PCB = (27.3, 27.8, 1.2)                 # circuit
OLED_DALLE = (26.7, 19.27, 1.5, 4.27)        # dalle de verre : largeur, hauteur, épaisseur, marge depuis le bord des broches
OLED_ACTIF = (21.74, 10.86, 2.10)            # zone affichée : largeur, hauteur, marge depuis le haut de la dalle
OLED_TROU = (2.0, 2.0)                       # trous Ø2 à 2 mm des bords (entraxe 23,3 × 23,8)
OLED_SUPPORT = 8.5 + 2.5                     # support femelle 8,5 sur la carte A7 + entretoise des broches mâles 2,5
OLED_Y0 = A7_Y0 + 4.37                       # bord du circuit côté broches (vers l'avant)
# Modulino Knob (modèle officiel Arduino ABX00107, sources/modulino_knob_ABX00107.step) : dessus du circuit à Z = 0
KNOB_STEP = str(_FM_ROOT / 'cao/prototype_final/sources/modulino_knob_ABX00107.step')
KNOB_AXE = (0.0, 257.7)                      # axe de l'encodeur (centre du circuit 41 × 25,36)
KNOB_ZPCB = 71.0                             # dessus du circuit au-dessus de la planche : l'axe dépasse de 5,5 au-dessus de la coque


def berceau(c):
    """Bac imprimé sans pattes (étroit : il tient à côté du servo de direction) : fond de 5 mm avec 4 écrous M3
    noyés par le dessus, vis M3×10 + rondelle par-dessous la planche ; 2 passages de sangle auto-agrippante."""
    lx, ly, _ = BATT
    xi, yi = lx / 2 + BERCEAU_JEU, ly / 2 + BERCEAU_JEU
    xe, ye = xi + BERCEAU_EP, yi + BERCEAU_EP
    b = boite(c, 'berceau_batterie', BATT_X - xe, BATT_Y - ye, BATT_X + xe, BATT_Y + ye, ZB, BERCEAU_H, BLANC)
    boite(c, 'berceau_creux', BATT_X - xi, BATT_Y - yi, BATT_X + xi, BATT_Y + yi, ZB + BERCEAU_FOND, BERCEAU_H, op=CUT, bodies=[b])
    trous = []
    for sx in (1, -1):
        for sy in (1, -1):
            px, py = BATT_X + sx * (lx / 2 - 8), BATT_Y + sy * (ly / 2 - 10)
            cyl_z(c, 'trou_fond_%+d%+d' % (sx, sy), px, py, 3.4, ZB - 1, BERCEAU_FOND + 2, op=CUT, bodies=[b])
            zpoly(c, 'ecrou_fond_%+d%+d' % (sx, sy), hexagon(None, px, py, 5.7), ZB + BERCEAU_FOND - 2.6, 2.61, CUT, [b])
            trous.append((round(px, 2), round(py, 2), 3.5, 'berceau'))
    for sy in (1, -1):   # sangles en travers (selon X), au ras du fond
        boite(c, 'sangle_%+d' % sy, BATT_X - xe - 1, BATT_Y + sy * 20 - SANGLE_L / 2, BATT_X + xe + 1, BATT_Y + sy * 20 + SANGLE_L / 2,
              ZB + BERCEAU_FOND, SANGLE_H, op=CUT, bodies=[b])
    return trous


def carte_a7(c):
    """Plaque 40 × 60 (rangée 0 vers l'avant), écran, GY-521, BMP280 dessus ; XIAO dessous."""
    z = ZB + A7_ZDESSUS
    boite(c, 'plaque_A7', -20, A7_Y0, 20, A7_Y0 + 60, z - 1.6, 1.6, BLEU)
    # écran : support femelle sur la carte, puis le module (circuit, dalle, zone affichée), vissé sous la coque en M2
    lx, ly, ep = OLED_PCB
    yb = OLED_Y0 + 1.5                                   # rangée des 4 broches (1,5 du bord)
    boite(c, 'support_ecran', -5.1, yb - 1.27, 5.1, yb + 1.27, z, 8.5, NOIR)
    zo = z + OLED_SUPPORT
    pcb = boite(c, 'ecran_OLED_circuit', -lx / 2, OLED_Y0, lx / 2, OLED_Y0 + ly, zo, ep, BLEU)
    for sx in (1, -1):
        for yy in (OLED_Y0 + OLED_TROU[1], OLED_Y0 + ly - OLED_TROU[1]):
            cyl_z(c, 'ecran_trou_%+d_%d' % (sx, yy), sx * (lx / 2 - OLED_TROU[0]), yy, 2.0, zo - 1, ep + 2, op=CUT, bodies=[pcb])
    dl, dh, de, dm = OLED_DALLE
    boite(c, 'ecran_OLED_dalle', -dl / 2, OLED_Y0 + dm, dl / 2, OLED_Y0 + dm + dh, zo + ep, de, NOIR)
    al, ah, am = OLED_ACTIF
    boite(c, 'ecran_OLED_zone_affichee', -al / 2, OLED_Y0 + dm + am, al / 2, OLED_Y0 + dm + am + ah, zo + ep + de, 0.05, 'Opaque(90,200,255)')
    for i in range(4):
        cyl_z(c, 'ecran_broche_%d' % i, (i - 1.5) * 2.54, yb, 0.64, zo - 2.5, 2.5 + ep + 1.0, METAL)
    # positions de la carte A7 (plan_c5_a7.py) : GY-521 broche 1 en (1, 19), BMP280 en (13, 17) tourné de 90°
    boite(c, 'GY521', -15.7, A7_Y0 + 39.2, 5.5, A7_Y0 + 55.6, z, 11, BLEU)
    boite(c, 'BMP280', 6.3, A7_Y0 + 35, 17.8, A7_Y0 + 50.5, z, 11, BLEU)
    boite(c, 'XIAO_C5_dessous', -9, A7_Y0 + 20, 9, A7_Y0 + 41, z - 1.6 - 14, 14, GRIS_F)


def knob(c):
    """Modèle 3D officiel du Modulino Knob, axe en KNOB_AXE, dessus du circuit à ZB + KNOB_ZPCB."""
    importer_step(c, KNOB_STEP, (KNOB_AXE[0] - 20.5, KNOB_AXE[1] - 12.68, ZB + KNOB_ZPCB), (1, 0, 0), (0, 1, 0), (0, 0, 1), 'knob')


def capuchon_knob(c):
    """Bouton imprimé sur l'axe Ø6 à méplat (4,5) du Knob : cannelé, 1,5 mm au-dessus de la coque (course du poussoir)."""
    x, y = KNOB_AXE
    z0 = ZB + 85.0 + 1.5
    b = body(zcyl(c, 'capuchon_knob', x, y, 22.0, z0, 9.0), 'capuchon_knob', BLANC)
    cyl_z(c, 'capuchon_dessus', x, y, 16.0, z0 + 9.0, 2.0, op=JOIN, bodies=[b])
    for i in range(8):
        a = math.radians(22.5 + 45 * i)
        cyl_z(c, 'capuchon_cannelure_%d' % i, x + 12.3 * math.cos(a), y + 12.3 * math.sin(a), 4.0, z0 - 1, 11.0, op=CUT, bodies=[b])
    # alésage en D : Ø6,15, méplat à 4,6 de la face opposée, 4 mm de prise
    r, f = 3.075, 1.6
    a0 = math.degrees(math.asin(f / r))
    arc = [-a0 + (180 + 2 * a0) * k / 24 for k in range(25)]       # de -31° à 211° par le haut, la corde ferme le méplat
    zpoly(c, 'capuchon_alesage', [(x + r * math.cos(math.radians(t)), y + r * math.sin(math.radians(t))) for t in arc], z0 - 0.01, 4.0, CUT, [b])
    return b



# ---- roues arrière : roues de tondeuse Ø200 × 50, montées directement sur l'arbre de sortie des moteurs
#      NGP RM24 (pas d'axe ni de roulement). Aspect indicatif dans le style des roues avant ; cotes à mesurer.
AR_Y, AR_Z = 380.0, 100.0            # axe des moteurs : 380 derrière les pivots, 30 mm sous la planche
AR_X0, AR_LARG, AR_D = 153.0, 50.0, 200.0   # face intérieure de la roue (3 mm après le support moteur)
AR_ARBRE = 10.0                      # arbre de sortie du moteur (diamètre supposé)
MOTEUR = [(46.0, 55.5), (42.0, 58.0)]   # mesuré par Hugo, depuis la face côté roue : réducteur Ø46, puis le
                                        # moteur Ø42 sur les 58 derniers mm ; 113,5 mm en tout


def revolution(c, nom, s, profil, couleur, op=NEW, bodies=None):
    """Profil (u, r) tourné autour de l'axe des roues arrière : u depuis la face intérieure de la roue,
    vers l'extérieur ; r depuis l'axe. Renvoie (corps ou fonction, ligne d'axe de l'esquisse)."""
    sk = sketch(c, plane_y(c, AR_Y, nom), nom)
    poly3(sk, [(s * (AR_X0 + u), AR_Y, AR_Z + r) for u, r in profil])
    ax = sk.sketchCurves.sketchLines.addByTwoPoints(sk.modelToSketchSpace(P(s * (AR_X0 - 10), AR_Y, AR_Z)),
                                                    sk.modelToSketchSpace(P(s * (AR_X0 + 60), AR_Y, AR_Z)))
    ax.isConstruction = True
    i = c.features.revolveFeatures.createInput(profiles(sk), ax, op)
    i.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
    if bodies is not None:
        i.participantBodies = list(bodies)
    f = c.features.revolveFeatures.add(i)
    f.name = 'rev_' + nom
    return (body(f, nom, couleur) if op == NEW else f), ax


def motif_axe(c, feat, axe, n, nom):
    col = adsk.core.ObjectCollection.create()
    col.add(feat)
    i = c.features.circularPatternFeatures.createInput(col, axe)
    i.quantity = adsk.core.ValueInput.createByReal(n)
    i.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    i.isSymmetric = False
    i.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
    p = c.features.circularPatternFeatures.add(i)
    p.name = 'motif_' + nom
    return p


def roue_arriere(c, s, tag):
    """Jante à 5 rayons (blanche), pneu à crampons en chevron (noir), enjoliveur bleu ; moyeu sur l'arbre."""
    jante, ax = revolution(c, 'jante_arriere_' + tag, s,
                           [(0, 76), (0, 86), (3, 86), (3, 80), (47, 80), (47, 86), (50, 86), (50, 76)], BLANC)
    revolution(c, 'moyeu_arriere_' + tag, s, [(0, AR_ARBRE / 2), (0, 18), (40, 18), (44, 14), (44, AR_ARBRE / 2)], BLANC,
               op=JOIN, bodies=[jante])
    # un rayon, côté extérieur (roue creusée), puis 5 en motif
    ray = xpoly(c, 'rayon_arriere_' + tag, s * (AR_X0 + 30), [(AR_Y - 7, AR_Z + 15), (AR_Y + 7, AR_Z + 15), (AR_Y + 4.5, AR_Z + 77),
                                                             (AR_Y - 4.5, AR_Z + 77)], s * 12, JOIN, [jante])
    motif_axe(c, ray, ax, 5, 'rayons_arriere_' + tag)
    restes = [b_ for b_ in c.bRepBodies if b_ != jante]       # le moyeu reste parfois un corps à part : on le soude
    if restes:
        col = adsk.core.ObjectCollection.create()
        for b_ in restes:
            col.add(b_)
        ci = c.features.combineFeatures.createInput(jante, col)
        ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        c.features.combineFeatures.add(ci).name = 'soudure_moyeu_arriere_' + tag
    pneu, ax2 = revolution(c, 'pneu_arriere_' + tag, s,
                           [(3, 80), (47, 80), (47, 86), (50, 88.5), (50, 91.5), (46.5, 95), (3.5, 95), (0, 91.5), (0, 88.5), (3, 86)], NOIR)
    # crampon en chevron sur le dessus (plan tangent), puis motif sur le tour
    chev = [(25, 8.5), (47, -3.5), (47, -10.5), (25, 1.5), (3, -10.5), (3, -3.5)]
    cr = zpoly(c, 'crampon_arriere_' + tag, [(s * (AR_X0 + u), AR_Y + v) for u, v in chev], AR_Z + AR_D / 2, 7.0, JOIN, [pneu], down=True)
    motif_axe(c, cr, ax2, 22, 'crampons_arriere_' + tag)
    e = cyl_x(c, 'enjoliveur_arriere_' + tag, s * (AR_X0 + 44), AR_Y, AR_Z, 30, s * 3, 'FM_PLA_bleu')
    cyl_x(c, 'enjoliveur_creux_' + tag, s * (AR_X0 + 46), AR_Y, AR_Z, 18, s * 1.5, op=CUT, bodies=[e])
    return jante, pneu


def moteurs_et_roues_arriere():
    for s, tag in ((1, 'D'), (-1, 'G')):
        _, c = piece('PF_REF_Moteur_arriere_' + tag, 'COTES_MESUREES_HUGO (NGP RM24)')
        m = cyl_x(c, 'moteur', s * 150, AR_Y, AR_Z, MOTEUR[0][0], -s * MOTEUR[0][1], METAL)
        x = 150 - MOTEUR[0][1]
        for k, (dm, lg) in enumerate(MOTEUR[1:]):
            xcyl(c, 'moteur_%d' % k, s * x, AR_Y, AR_Z, dm, -s * lg, JOIN, [m])
            x -= lg
        cyl_x(c, 'REF_arbre_moteur_' + tag, s * 150, AR_Y, AR_Z, AR_ARBRE, s * 44, METAL)
        _, c = piece('PF_REF_Roue_arriere_' + tag, 'ASPECT_INDICATIF (roue de tondeuse sur l arbre du moteur, cotes a mesurer)')
        roue_arriere(c, s, tag)



# ---- convertisseurs XL4015 (5,1 V pour le Pi, 6 V pour le servo) sur leur plaque imprimée : côte à côte à
#      5 mm l'un de l'autre (les pastilles IN/OUT ne se touchent pas), bout IN vers l'avant, sur 2 plots par
#      module ; vis M3×8 par le dessus, écrou M3 dans une poche ouverte sous la plaque. Circuit 1,6 + plot 3,6
#      + plaque 3 = 8,2 : la vis s'arrête 0,2 mm au-dessus du dessous, rien ne dépasse. La poche fait 3,6 de
#      haut pour un écrou de 2,4 : l'écrou remonte 1 mm plus près de la vis, qui l'attrape sans forcer
XL_PCB = (23.0, 54.0)                      # largeur (X) × longueur (Y) du circuit
XL_TROUS = [(2.0, 9.4), (21.0, 44.6)]      # trous Ø3 en diagonale, depuis le coin « bord gauche, bout IN » ; mesurés :
                                           # 40 mm en diagonale entre centres, 19 en largeur → √(40² − 19²) = 35,2 en longueur
XL_ECART, XL_MARGE, XL_OREILLE, XL_EP, XL_PLOT_H, XL_PLOT_D = 5.0, 3.0, 9.0, 3.0, 3.6, 8.0
XL_POCHE = 3.6                             # poche de l'écrou M3 (2,4) sous la plaque : 1 mm plus haute que l'écrou
XL_PIED = 0.0                              # pieds sous la plaque (aucun : rien ne dépasse dessous)
XL_X0, XL_Y0 = -100.25, 34.0               # coin « bord gauche, bout IN » du circuit du Pi : à l'avant droit, en
                                           # miroir de la batterie (équilibrage gauche-droite), à 5 mm devant le Pi
XL_H = 15.0                                # hauteur du module, composants compris (réserve)


def support_xl4015(c):
    """Plaque, plots, trous de vis et poches d'écrou ; 2 trous Ø3,4 aux bouts pour la visser sur la planche
    (vis M3×12 : plaque 3 + planche 5 + rondelle + écrou)."""
    W, L = XL_PCB
    x0, x1 = XL_X0 - XL_MARGE, XL_X0 + 2 * W + XL_ECART + XL_MARGE
    y0, y1 = XL_Y0 - XL_MARGE - XL_OREILLE, XL_Y0 + L + XL_MARGE + XL_OREILLE
    zp = ZB + XL_PIED                       # dessous de la plaque
    k = 3.0
    contour = [(x0 + k, y0), (x1 - k, y0), (x1, y0 + k), (x1, y1 - k), (x1 - k, y1), (x0 + k, y1), (x0, y1 - k), (x0, y0 + k)]
    b = body(zpoly(c, 'support_xl4015', contour, zp, XL_EP), 'support_xl4015', BLANC)
    fix = [((x0 + x1) / 2, y0 + XL_OREILLE / 2 + 1), ((x0 + x1) / 2, y1 - XL_OREILLE / 2 - 1)]
    pieds = [(x0 + 5, y0 + 5), (x1 - 5, y0 + 5), (x0 + 5, y1 - 5), (x1 - 5, y1 - 5)] + fix
    for j, (x, y) in enumerate(pieds if XL_PIED > 0 else []):
        cyl_z(c, 'xl_pied_%d' % j, x, y, 9.0, ZB, XL_PIED + 0.01, op=JOIN, bodies=[b])
    for m in range(2):
        for j, (tx, ty) in enumerate(XL_TROUS):
            x, y = XL_X0 + m * (W + XL_ECART) + tx, XL_Y0 + ty
            cyl_z(c, 'xl_plot_%d_%d' % (m, j), x, y, XL_PLOT_D, zp + XL_EP - 0.01, XL_PLOT_H + 0.01, op=JOIN, bodies=[b])
            cyl_z(c, 'xl_vis_%d_%d' % (m, j), x, y, 3.4, zp - 1, XL_EP + XL_PLOT_H + 2, op=CUT, bodies=[b])
            zpoly(c, 'xl_ecrou_%d_%d' % (m, j), hexagon(None, x, y, 5.7), zp - 0.01, XL_POCHE + 0.01, CUT, [b])
    for j, (x, y) in enumerate(fix):
        cyl_z(c, 'xl_fixation_%d' % j, x, y, 3.4, ZB - 1, XL_PIED + XL_EP + 2, op=CUT, bodies=[b])
    return b, fix


def xl4015_et_support():
    """Plaque support (imprimée) et les deux modules posés sur ses plots."""
    _, c = piece('PF_IMP_Support_XL4015', 'IMPRIME_PETG')
    b, fix = support_xl4015(c)
    W, L = XL_PCB
    for m, (nom, st) in enumerate((('PF_REF_XL4015_1_Pi_5V1', 'COTES_AZDELIVERY'), ('PF_REF_XL4015_2_servo_6V', 'COTES_AZDELIVERY'))):
        _, c2 = piece(nom, st + ' (trous a mesurer)')
        x = XL_X0 + m * (W + XL_ECART)
        boite(c2, nom.replace('PF_REF_', '').lower(), x, XL_Y0, x + W, XL_Y0 + L, ZB + XL_PIED + XL_EP + XL_PLOT_H, XL_H, BLEU)
    return b, fix

def run(_context: str):
    d = design()
    nettoyer(['PF_REF_', 'PF_IMP_Berceau', 'PF_IMP_Capuchon', 'PF_IMP_Support_XL4015'])
    for o in list(d.rootComponent.occurrences):      # imports d'essai
        if o.component.name in ('TMP_IMPORT', 'TMP_TEST'):
            o.deleteMe()
    g = group().component
    for nom, x0, y0, x1, y1, z0, h, col, st in BLOCS:
        _, c = piece(nom, st)
        boite(c, nom.replace('PF_REF_', '').lower(), x0, y0, x1, y1, ZB + z0, h, col)
    _, fix_xl = xl4015_et_support()
    # batterie et berceau
    _, c = piece('PF_REF_Batterie', 'COTES_MESUREES_82x68x51')
    boite(c, 'batterie', BATT_X - BATT[0] / 2, BATT_Y - BATT[1] / 2, BATT_X + BATT[0] / 2, BATT_Y + BATT[1] / 2, ZB + BERCEAU_FOND, BATT[2], NOIR)
    _, c = piece('PF_IMP_Berceau_batterie', 'IMPRIME_PETG')
    trous = berceau(c)
    # carte capteurs A7 et Knob sous la bande centrale, au-dessus de la breadboard
    _, c = piece('PF_REF_Carte_capteurs_A7', 'PLAQUE_A7_PERFBOARD_40x60')
    carte_a7(c)
    _, c = piece('PF_REF_Modulino_Knob', 'MODELE_OFFICIEL_ARDUINO_ABX00107')
    knob(c)
    _, c = piece('PF_IMP_Capuchon_Knob', 'IMPRIME_PETG')
    capuchon_knob(c)
    # antenne GNSS collée sous le dessus de la coque (côté -X, derrière le Pi)
    _, c = piece('PF_REF_Antenne_GNSS', 'ENVELOPPE_A_MESURER')
    boite(c, 'antenne_GNSS', -92.5, 317, -67.5, 342, ZB + 74.6, 8, GRIS_C)
    # sous la planche : moteurs (Ø46 dans les supports) et roues arrière Ø200 × 50, directement sur l'arbre
    moteurs_et_roues_arriere()
    trous += [(round(x, 2), round(y, 2), 3.5, 'xl4015') for x, y in fix_xl]
    json.dump({'trous': trous}, open(str(_FM_ROOT / 'cao/prototype_final/trous_composants.json'), 'w', encoding='utf-8'), indent=1)
    gp = group()
    for o in gp.childOccurrences:
        bb = o.boundingBox
        print(o.component.name, [round(v * 10, 1) for v in bb.minPoint.asArray()], [round(v * 10, 1) for v in bb.maxPoint.asArray()])
    tl = d.timeline
    print('fonctions KO', [tl.item(i).name for i in range(tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup])
