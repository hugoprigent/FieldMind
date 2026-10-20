from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Remorque porte-drone du prototype final, attelée derrière le rover. À exécuter dans le document
« remorque fieldmind » (le document du rover doit être ouvert : roues avant, couleurs). Réexécutable.

- Planche de bois 300 × 380 × 5 (même largeur que celle du rover), coins avant coupés à 45°.
- Rebords imprimés (bleu) à l'avant et à l'arrière, vis M3 par-dessous, écrous noyés dans la lèvre ;
  joues au ras du plateau sur les côtés, là où les volets se déplient.
- Deux volets latéraux, chacun mû par un SG90 logé à l'avant, dans le coin coupé (câble court jusqu'au
  timon), charnière libre à l'arrière. Volet déplié posé sur les butées de ces deux pièces.
- Pièces blanches vissées sur le bois : vis à tête bombée par le dessus (pas de fraisage dans le bois), écrou
  noyé dans la pièce ; M6 pour
  les supports de roue (ils portent la remorque), M3 pour les boîtiers des servos, blocs arrière et ferrures.
- Deux roues identiques aux roues avant (jante, pneu TPU, flasque), chacune folle sur sa vis M5.
  Pas de 608 : deux roulements 5×16×5 par roue, dans des bagues imprimées qui prennent la place
  des 608 dans la jante.
- Attelage : chape en C sous l'arrière du rover, timon en A sous l'avant de la remorque, sa tête
  entre les deux mâchoires ; une vis H M10 qui traverse le tout, écrou frein dessous.
  Trou de la tête en diabolo : tangage et roulis de ±10°.
- Croix d'atterrissage peinte au centre du plateau (REF, épaisseur de peinture).
"""
import adsk.core, adsk.fusion, json, math
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/remorque/scripts/rm_helpers.py'), encoding='utf-8').read(), 'rm_helpers.py', 'exec'))


def timon(c):
    """Timon en A (PLA bleu) : deux bras de la planche à la tête, évidement, congés, dessus chanfreiné.
    La tête Ø36 se loge dans la chape du rover ; trou en diabolo autour de la vis d'attelage."""
    z0 = TIMON_Z0
    p0 = (0.0, PIVOT_Y)
    b = body(zbox(c, 'timon_base', -84, YF - 2, 84, YF + 50, z0, TIMON_EP), 'timon', BLEU_PLA)
    demi = 9.0
    for sx in (1, -1):
        e = (sx * 72.0, YF + 12)
        L = math.hypot(e[0] - p0[0], e[1] - p0[1])
        d = ((e[0] - p0[0]) / L, (e[1] - p0[1]) / L)
        n = (abs(d[1]) * sx, -abs(d[0]))            # normale vers l'extérieur du bras
        pts = [(p0[0] + demi * n[0], p0[1] + demi * n[1]), (e[0] + demi * n[0], e[1] + demi * n[1]),
               (e[0] - demi * n[0], e[1] - demi * n[1]), (p0[0] - demi * n[0], p0[1] - demi * n[1])]
        zpoly(c, 'timon_bras_%+d' % sx, pts, z0, TIMON_EP, JOIN, [b])
        if sx > 0:
            q = (p0[0] - demi * n[0], p0[1] - demi * n[1])
            x_de = lambda y: q[0] + (y - q[1]) * d[0] / d[1]
    cyl_z(c, 'timon_tete', 0, PIVOT_Y, OEIL_D, z0, TIMON_EP, op=JOIN, bodies=[b])
    y1, y2 = PIVOT_Y + OEIL_D / 2 + 14, YF - 2
    zpoly(c, 'timon_evidement', [(-x_de(y1), y1), (x_de(y1), y1), (x_de(y2), y2), (-x_de(y2), y2)], z0 - 1, TIMON_EP + 2, CUT, [b])
    r = arrondir(c, 'timon_conges', b, 'z', TIMON_EP - 0.5, (5.0, 3.0, 1.5))
    # dessus chanfreiné (arêtes extérieures du dessus, en chaîne tangente)
    plans = [f for f in b.faces if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType
             and abs(abs(f.geometry.normal.z) - 1) < 1e-6 and abs(f.pointOnFace.z * 10 - (z0 + TIMON_EP)) < 1e-3]
    ch = 0
    if plans:
        haut = max(plans, key=lambda f: f.area)
        ar = [e for e in haut.edges if e.geometry.curveType == adsk.core.Curve3DTypes.Line3DCurveType]
        ch = chanfreiner(c, 'timon_chanfrein', ar, 2.0)
    # cale sous la planche (la planche est 8 mm au-dessus du timon)
    boite(c, 'timon_cale', -84, YF - 2, 84, YF + 50, ZP - TIMON_CALE, TIMON_CALE, op=JOIN, bodies=[b])
    for x, y, L_, reb, role in vis_planche():
        if role == 'timon':
            trou_z(c, 'timon_vis_%+d' % x, x, y, 6.5, z0 - 1, TIMON_EP + TIMON_CALE + 2, [b])
            hexa_z(c, 'timon_ecrou_%+d' % x, x, y, M6_HEX, z0 - 0.01, M6_ECROU_H + 0.2, [b])   # écrou M6 noyé par-dessous
    # câbles des volets : encoche dans le dessus de la cale (sous la planche), descente, rainure sous chaque
    # bras jusqu'à la prise 4 broches logée sous la tête
    px, py, pl, pe, pp = PRISE
    for sx in (1, -1):
        boite(c, 'timon_cable_cale_%+d' % sx, sx * 62, Y_CABLE - CABLE_L / 2, sx * 85, Y_CABLE + CABLE_L / 2, ZP - CABLE_P, CABLE_P + 0.01, op=CUT, bodies=[b])
        trou_z(c, 'timon_cable_descente_%+d' % sx, sx * 62, Y_CABLE, 5.0, z0 - 1, TIMON_EP + TIMON_CALE + 2, [b])
        ligne = [(sx * 62, Y_CABLE), (sx * 71, YF + 12), (sx * 14.1, PIVOT_Y + 21.9), (sx * (pl / 2 - 1), py)]
        zpoly(c, 'timon_cable_rainure_%+d' % sx, bande(ligne, -CABLE_L / 2, CABLE_L / 2), z0 - 0.01, CABLE_P + 0.01, CUT, [b])
    boite(c, 'timon_prise', px - pl / 2, py - pe / 2, px + pl / 2, py + pe / 2, z0 - 0.01, pp, op=CUT, bodies=[b])
    # trou en diabolo : Ø10,6 au milieu (vis M10), évasé vers le haut et vers le bas de BASCULE degrés
    zm = z0 + TIMON_EP / 2
    for nom, bas in (('haut', False), ('bas', True)):
        s = sketch(c, plane_z(c, zm, 'diabolo_' + nom), 'diabolo_' + nom)
        circle(s, 0, PIVOT_Y, M10_TIMON)
        extrude(c, profiles(s), TIMON_EP / 2 + 1, CUT, 'extr_diabolo_' + nom, [b], down=bas, taper_deg=BASCULE)   # dépouille positive : la découpe s'élargit
    return b, {'conges_mm': r, 'chanfrein_mm': ch}


SOURCES = str(_FM_ROOT / 'cao/prototype_final/sources/')


def semelle_joue(c, nom, sx, y0, y1, corps):
    """Logement de la semelle de la joue latérale sur le dessus d'un bras : la semelle y est pincée."""
    boite(c, 'semelle_' + nom, sx * JOUE_SEMELLE[0], y0, sx * (LARG / 2 + REB_EP), y1, ZP - JOUE_SEMELLE[1], JOUE_SEMELLE[1] + 0.01, op=CUT, bodies=[corps])


def semelle_et_butee(c, nom, sx, y0, y1, corps):
    """Plaque sous la planche (bras des vis, de x = 115 au bord) prolongée dehors jusqu'à X_BUTEE, plus bas :
    son dessus porte la ferrure du volet déplié (butée)."""
    x = LARG / 2 + REB_EP
    prof = [(115, ZP - 10), (X_BUTEE, ZP - 10), (X_BUTEE, Z_APPUI - 3), (x, Z_APPUI - 3), (x, ZP), (115, ZP)]
    return prisme_y(c, nom, (y0 + y1) / 2, [(sx * u, v) for u, v in prof], y1 - y0, JOIN, [corps])


def bloc_arriere(c, sx, cote):
    """Bloc de charnière arrière : joue au ras du plateau derrière la joue latérale, noeud autour de l'axe
    (vis M3×20 libre, écrou frein), semelle sous la planche vers l'avant (1 vis M3, écrou noyé) dont
    le dessus, dehors, est la butée de la ferrure arrière du volet déplié."""
    x = LARG / 2
    pts = [(115, ZP - 10), (X_BUTEE, ZP - 10), (X_BUTEE, Z_APPUI - 3), (x + REB_EP, Z_APPUI - 3), (x + REB_EP, ZP + EP),
           (x, ZP + EP), (x, ZP), (115, ZP)]
    b = body(prisme_y(c, 'bloc_ar_' + cote, (BLOC_AR[0] + BLOC_AR[1]) / 2, [(sx * u, v) for u, v in pts], BLOC_AR[1] - BLOC_AR[0]),
             'charniere_AR_' + cote, BLANC)
    cyl_y(c, 'bloc_ar_noeud_' + cote, sx * X_AX, Z_AX, 2 * CHARN_R, BLOC_AR[0], BLOC_AR[1], JOIN, [b])
    cyl_y(c, 'bloc_ar_axe_' + cote, sx * X_AX, Z_AX, 3.4, BLOC_AR[0] - 1, BLOC_AR[1] + 1, CUT, [b])
    semelle_et_butee(c, 'bloc_ar_semelle_' + cote, sx, Y_SEM_AR, BLOC_AR[0] + 0.01, b)
    semelle_joue(c, 'bloc_ar_' + cote, sx, Y_SEM_AR - 1, V_Y1 + 0.5, b)
    # dégagement autour de l'axe : le noeud de la ferrure arrière y tourne
    cyl_y(c, 'bloc_ar_degagement_' + cote, sx * X_AX, Z_AX, 2 * CHARN_R + 1.5, FERR_AR[0] - 1, BLOC_AR[0], CUT, [b])
    return b


def bloc_servo(c, sx, cote):
    """Boîtier du SG90, à l'avant, dans le coin coupé de la planche (une seule pièce) :
    - logement du servo, ouvert à l'avant : le servo y entre par l'avant, arbre dans l'axe de la charnière,
      palonnier vissé sur la ferrure avant du volet ; ses oreilles sont vissées dans le boîtier ;
    - semelle sous la planche, 2 vis M3 à tête bombée, écrous noyés dessous ; dehors, la butée du volet déplié ;
    - câble : il sort sous le servo, passe dans un tunnel vers l'arrière, remonte et file vers le timon
      dans une rainure sous la planche."""
    g = SG90
    y0 = Y_PAL - 5.5                                   # face arrière (le moyeu du palonnier commence à Y_PAL − 5)
    yb = y0 - 28.5                                     # face avant (logement ouvert)
    x_in = X_AX - 9.5                                  # paroi intérieure : 1,5 mm du rebord avant
    b = boite(c, 'servo_corps_' + cote, sx * x_in, yb, sx * X_BUTEE, y0, Z_AX - 25, 39, BLANC)
    b.name = 'boitier_servo_' + cote
    semelle_et_butee(c, 'servo_semelle_' + cote, sx, y0 - 4, Y_SEM_AV, b)
    zh, zb_ = Z_AX + g['axe_bout'], Z_AX + g['axe_bout'] - g['L']            # haut et bas du corps du servo
    jeu = 0.3
    # corps (de la face avant au bossage), oreilles (de la face avant à leur appui), bossage et son lobe
    boite(c, 'servo_logement_' + cote, sx * (X_AX - g['l'] / 2 - jeu), yb - 1, sx * (X_AX + g['l'] / 2 + jeu), Y_PAL - 9.3 + jeu, zb_ - jeu, g['L'] + 2 * jeu, op=CUT, bodies=[b])
    zo0, lo = zb_ - g['oreilles_bas'], g['L'] + g['oreilles_bas'] + g['oreilles_haut']
    boite(c, 'servo_oreilles_' + cote, sx * (X_AX - g['l'] / 2 - jeu), yb - 1, sx * (X_AX + g['l'] / 2 + jeu), Y_PAL - 13.4, zo0 - jeu, lo + 2 * jeu, op=CUT, bodies=[b])
    cyl_y(c, 'servo_bossage_' + cote, sx * X_AX, Z_AX, 12.4, Y_PAL - 9.5, y0 + 1, CUT, [b])
    boite(c, 'servo_lobe_' + cote, sx * (X_AX - 2.8), Y_PAL - 9.5, sx * (X_AX + 2.8), y0 + 1, Z_AX - 11.5, 11.5, op=CUT, bodies=[b])
    for k, zz in enumerate(zb_ + t for t in g['trous_oreilles']):    # vis des oreilles (fournies avec le servo)
        cyl_y(c, 'servo_vis_%s_%d' % (cote, k), sx * X_AX, zz, 1.6, Y_PAL - 13.5, Y_PAL - 7.0, CUT, [b])
    # dégagement autour de l'axe : le moyeu du palonnier et le noeud de la ferrure avant y tournent
    cyl_y(c, 'servo_degagement_' + cote, sx * X_AX, Z_AX, 2 * CHARN_R + 1.5, y0 - 0.01, FERR_AV[1] + 1, CUT, [b])
    # câble : tunnel sous le logement (depuis le dessous des oreilles), montée, rainure sous la planche
    xc0, xc1 = sx * (x_in + 1.2), sx * (x_in + 5.0)
    zt = zo0 - jeu
    boite(c, 'servo_tunnel_' + cote, xc0, Y_PAL - 16.0, xc1, y0 + 0.01, zt, 3.5, op=CUT, bodies=[b])
    boite(c, 'servo_montee_' + cote, xc0, y0 - 2.5, xc1, y0 + 0.01, zt, ZP - 4.5 - zt + 0.01, op=CUT, bodies=[b])
    boite(c, 'servo_rainure_' + cote, sx * 114.0, Y_CABLE - CABLE_L / 2, xc1, Y_CABLE + CABLE_L / 2, ZP - 4.5, 4.51, op=CUT, bodies=[b])
    semelle_joue(c, 'boitier_' + cote, sx, V_Y0, Y_SEM_AV + 1, b)
    return b


def volet(c, sx, cote):
    """Volet : panneau de contreplaqué, ferrure avant (palonnier du SG90 sur sa face avant), ferrure arrière
    (noeud sur l'axe M3). Chaque ferrure est vissée sous le panneau par 2 vis M3×16 à tête bombée par le dessus
    du panneau, écrous noyés sous des bossages."""
    xa = sx * X_AX
    pan = boite(c, 'volet_panneau_' + cote, xa, V_Y0, sx * (X_AX + VOLET_W), V_Y1, Z_APPUI, VOLET_EP, BLANC)
    pan.name = 'REF_volet_contreplaque_' + cote
    f_av = cyl_y(c, 'ferrure_av_noeud_' + cote, xa, Z_AX, 2 * CHARN_R, FERR_AV[0], FERR_AV[1], NEW, couleur=BLANC)
    f_av.name = 'ferrure_AV_' + cote
    boite(c, 'ferrure_av_barre_' + cote, xa, FERR_AV[0], sx * (X_AX + 23), FERR_AV[1], Z_AX - CHARN_R, 2 * CHARN_R, op=JOIN, bodies=[f_av])
    boite(c, 'ferrure_av_plaque_' + cote, sx * 157, FERR_AV[0], sx * 200, V_Y0 + 26, Z_APPUI - 3, 3, op=JOIN, bodies=[f_av])
    for d_ in (9.0, 13.0):                                     # vis du palonnier (trous du bras à 9 et 13 mm de l'axe)
        cyl_y(c, 'ferrure_av_palonnier_%s_%d' % (cote, d_), sx * (X_AX + d_), Z_AX, 1.2, FERR_AV[0] - 0.5, FERR_AV[0] + 4, CUT, [f_av])
    f_ar = cyl_y(c, 'ferrure_ar_noeud_' + cote, xa, Z_AX, 2 * CHARN_R, FERR_AR[0], FERR_AR[1], NEW, couleur=BLANC)
    f_ar.name = 'ferrure_AR_' + cote
    boite(c, 'ferrure_ar_plaque_' + cote, sx * 157, Y_SEM_AR, sx * 200, FERR_AR[1], Z_APPUI - 3, 3, op=JOIN, bodies=[f_ar])
    cyl_y(c, 'ferrure_ar_axe_' + cote, xa, Z_AX, 3.4, FERR_AR[0] - 1, FERR_AR[1] + 1, CUT, [f_ar])     # axe M3 libre
    for f, yv in ((f_av, V_Y0 + 13), (f_ar, V_Y1 - 10)):
        for xv in (175.0, 192.0):
            n = '%s_%d_%d' % (cote, yv, xv)
            zb = Z_APPUI + VOLET_EP - 16.0                 # bout de la vis M3×16 : dessous du bossage
            cyl_z(c, 'ferrure_bossage_' + n, sx * xv, yv, 9.0, zb, Z_APPUI - 3 - zb + 0.01, op=JOIN, bodies=[f])
            trou_z(c, 'ferrure_vis_' + n, sx * xv, yv, 3.4, zb - 1, Z_APPUI - zb + 2, [f])
            hexa_z(c, 'ferrure_ecrou_' + n, sx * xv, yv, M3_HEX, zb - 0.01, M3_ECROU_H, [f])
            trou_z(c, 'volet_vis_' + n, sx * xv, yv, 3.4, Z_APPUI - 1, VOLET_EP + 2, [pan])
    # palonnier 1 bras du SG90 (modèle FreeCAD) : moyeu vers le servo (vers l'avant), bras dans le sens du volet
    importer_step(c, SOURCES + 'servo_SG90_palonnier_1_bras_freecad.step',
                  (xa - sx * 27.0, Y_PAL + 25.67, Z_AX - sx * 36.3), (sx, 0, 0), (0, 0, sx), (0, -1, 0), 'palonnier_SG90_' + cote)
    return pan, f_av, f_ar


def sg90(c, sx, cote):
    """SG90 (modèle FreeCAD) : arbre le long de +Y dans l'axe de la charnière, corps vers le bas, devant le volet."""
    importer_step(c, SOURCES + 'servo_SG90_freecad.step', (sx * X_AX, Y_PAL - 32.0, Z_AX - 16.6), (-1, 0, 0), (0, 0, 1), (0, 1, 0), 'SG90_' + cote)


def support_roue(c, sx, cote):
    """Support de roue : bras sous la planche (de BRAS_X0 au bord : 4 vis M6×16 à tête bombée, 2 près du bord à
    travers la joue, 2 vers le milieu de la planche) + jambe verticale + 2 nervures ; axe M5 à Z_AXE."""
    xi, xe = sx * (X_JAMBE - JAMBE_EP), sx * X_JAMBE
    jambe = xpoly(c, 'jambe_' + cote, xi, [(Y_AXE - 60, ZP), (Y_AXE + 60, ZP), (Y_AXE + 14, Z_AXE), (Y_AXE - 14, Z_AXE)], sx * JAMBE_EP)
    b = body(jambe, 'support_roue_' + cote, BLANC)
    cyl_x(c, 'jambe_oeil_' + cote, xi, Y_AXE, Z_AXE, 28.0, sx * JAMBE_EP, op=JOIN, bodies=[b])
    boite(c, 'bras_' + cote, sx * BRAS_X0, Y_AXE - 60, xe, Y_AXE + 60, ZP - BRAS_EP, BRAS_EP, op=JOIN, bodies=[b])
    for k, dy in enumerate((-12.0, 12.0)):
        tri = [(sx * 112.0, ZP - BRAS_EP + 1), (xi + sx * 1, ZP - BRAS_EP + 1), (xi + sx * 1, Z_AXE + 20)]
        prisme_y(c, 'nervure_%s%d' % (cote, k), Y_AXE + dy, tri, 6.0, JOIN, [b])
    cyl_x(c, 'axe_' + cote, xi - sx, Y_AXE, Z_AXE, 5.3, sx * (JAMBE_EP + 2), op=CUT, bodies=[b])
    for x, y, L, reb, role in vis_planche():
        if role == 'support de roue' and x * sx > 0:
            trou_z(c, 'support_vis_%s_%d_%d' % (cote, x, y), x, y, 6.5, ZP - BRAS_EP - 1, BRAS_EP + 2, [b])            # M6
            hexa_z(c, 'support_ecrou_%s_%d_%d' % (cote, x, y), x, y, M6_HEX, ZP - BRAS_EP - 0.01, M6_ECROU_H, [b])
    semelle_joue(c, 'support_' + cote, sx, Y_AXE - 61, Y_AXE + 61, b)
    return b


def place_roue(jante_native, sx):
    """Place d'une roue avant (corps natif, repère de la roue) sur la remorque : face intérieure du moyeu
    en x = ±X_MOYEU, axe à Z_AXE (le décalage en Y = Y_AXE est ajouté à la copie)."""
    bx = jante_native.boundingBox
    x_moyeu_v04 = bx.minPoint.x * 10 if sx > 0 else bx.maxPoint.x * 10
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create((sx * X_MOYEU - x_moyeu_v04) / 10, 0, Z_AXE / 10)
    return m


def roue(c, cf, sx, cote, v04):
    """Jante, pneu et flasque copiés de la roue avant du même côté ; bagues, roulements 5x16x5 (dans c,
    la partie qui tourne) ; axe M5, écrou, rondelles et entretoises (dans cf, la partie fixe)."""
    src_cote = 'D' if sx > 0 else 'G'           # les roues V04 « _D » sont côté +X
    o = next(oo for oo in v04.childOccurrences if oo.component.name == 'D4_IMP_Roue_' + src_cote)
    # corps natifs (repère de la roue : axe X, centre en y = z = 0), sans la pose de l'occurrence : la
    # direction du rover peut être braquée, la roue de la remorque doit rester droite
    natifs = {b.name: b for b in o.component.bRepBodies}
    out = []
    for nom, coul in (('jante', BLANC), ('pneu_TPU', NOIR), ('flasque_pneu', BLANC)):
        out.append(copier_corps(c, natifs['%s_%s' % (nom, src_cote)], place_roue(natifs['jante_' + src_cote], sx), 0, Y_AXE,
                                '%s_remorque_%s' % (nom, cote), coul))
    X = lambda u: sx * (X_MOYEU + u)        # abscisse depuis la face intérieure du moyeu, vers l'extérieur
    y, z = Y_AXE, Z_AXE
    # bagues imprimées 22 -> 16 : roulement côté extérieur de chaque logement de 608, lèvre côté arête
    for nom, u0, levre in (('int', 0.0, 5.0), ('ext', BAGUE_L + ARETE, 0.0)):
        b = anneau_x(c, 'bague_625_%s_%s' % (nom, cote), X(u0), sx * BAGUE_L, y, z, BAGUE_DE, LEVRE_625, BLANC)
        u_loge = u0 - 0.5 if nom == 'int' else u0 + 2.0
        cyl_x(c, 'loge_625_%s_%s' % (nom, cote), X(u_loge), y, z, LOGE_625, sx * (R625[2] + 0.5), op=CUT, bodies=[b])
        out.append(b)
    u_ext = 5.0 + ENTRETOISE
    for nom, u0 in (('int', 0.0), ('ext', u_ext)):
        out.append(anneau_x(c, 'REF_625ZZ_%s_%s' % (nom, cote), X(u0), sx * R625[2], y, z, R625[1], R625[0], METAL))
    out.append(anneau_x(cf, 'entretoise_625_%s' % cote, X(5.0), sx * ENTRETOISE, y, z, APPUI_D, 5.3, BLANC))
    out.append(anneau_x(cf, 'rondelle_tete_%s' % cote, X(u_ext + 5.0), sx * 2.0, y, z, APPUI_D, 5.3, BLANC))
    out.append(anneau_x(cf, 'cale_jambe_%s' % cote, X(-CALE), sx * CALE, y, z, APPUI_D, 5.3, BLANC))
    # vis CHC M5x40, rondelle et écrou frein M5 côté intérieur de la jambe
    u_tete = u_ext + 5.0 + 2.0
    v = cyl_x(cf, 'REF_vis_CHC_M5x40_' + cote, X(u_tete), y, z, 5.0, -sx * 40.0, METAL)
    cyl_x(cf, 'REF_vis_tete_' + cote, X(u_tete), y, z, 8.5, sx * 5.0, op=JOIN, bodies=[v])
    xi = sx * (X_JAMBE - JAMBE_EP)
    out.append(anneau_x(cf, 'REF_rondelle_M5_' + cote, xi, -sx * 1.0, y, z, 10.0, 5.3, METAL))
    hexa = [(y + 4.62 * math.cos(math.radians(30 + 60 * k)), z + 4.62 * math.sin(math.radians(30 + 60 * k))) for k in range(6)]
    e = body(xpoly(cf, 'REF_ecrou_M5_' + cote, xi - sx * 1.0, hexa, -sx * 5.0), 'REF_ecrou_frein_M5_' + cote, METAL)
    cyl_x(cf, 'REF_ecrou_M5_taraudage_' + cote, xi, y, z, 5.0, -sx * 7.0, op=CUT, bodies=[e])
    return out


def pivot_roue(ow, osup, sx, cote):
    """Liaison pivot (as-built) de la roue sur son support, autour de l'axe M5 : la roue tourne librement."""
    root = design().rootComponent
    cw = ow.component
    s = sketch(cw, plane_x(cw, sx * X_MOYEU, 'axe_roue_' + cote), 'axe_roue_' + cote)
    pt = s.sketchPoints.add(s.modelToSketchSpace(P(sx * X_MOYEU, Y_AXE, Z_AXE)))
    ji = root.asBuiltJoints.createInput(ow, osup, adsk.fusion.JointGeometry.createByPoint(pt.createForAssemblyContext(ow)))
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)      # Z de l'esquisse = axe X du rover
    j = root.asBuiltJoints.add(ji)
    j.name = 'pivot_roue_' + cote
    return j


def charniere_volet(ov, of, sx, cote):
    """Liaison pivot (as-built) du volet sur les charnières : axe le long de Y, butées à 0° (déplié)
    et 90° (replié, debout contre la joue du rebord). On peut la manipuler dans Fusion."""
    root = design().rootComponent
    cv = ov.component
    s = sketch(cv, plane_y(cv, V_Y0, 'axe_charniere_' + cote), 'axe_charniere_' + cote)
    pt = s.sketchPoints.add(s.modelToSketchSpace(P(sx * X_AX, V_Y0, Z_AX)))
    ji = root.asBuiltJoints.createInput(ov, of, adsk.fusion.JointGeometry.createByPoint(pt.createForAssemblyContext(ov)))
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)      # Z de l'esquisse = axe Y du rover
    j = root.asBuiltJoints.add(ji)
    j.name = 'charniere_volet_' + cote
    pan = next(b for b in ov.bRepBodies if b.name.startswith('REF_volet'))
    replie = None
    for v in (-sx * math.pi / 2, sx * math.pi / 2):          # sens qui met le volet debout
        j.jointMotion.rotationValue = v
        if pan.boundingBox.maxPoint.z * 10 > Z_AX + VOLET_W - 1:
            replie = v
            break
    assert replie is not None, 'sens de la charnière introuvable'
    lim = j.jointMotion.rotationLimits
    lim.isMinimumValueEnabled = lim.isMaximumValueEnabled = True
    lim.minimumValue, lim.maximumValue = min(0.0, replie), max(0.0, replie)
    j.jointMotion.rotationValue = 0.0
    return j


def etape(nom):
    """Journal d'avancement (si le script bloque, la dernière ligne dit où)."""
    import time
    open(DOSSIER + '/remorque_progression.log', 'a', encoding='utf-8').write(texte('%s %s\n' % (time.strftime('%H:%M:%S'), nom)))


def run(_context: str):
    open(DOSSIER + '/remorque_progression.log', 'w', encoding='utf-8').close()
    etape('début')
    d = design()                                     # document « remorque fieldmind »
    root = d.rootComponent
    src = doc_rover()                                # document du rover : roues avant et couleurs
    v04 = next(o for o in src.rootComponent.occurrences if o.component.name == 'FM_Direction_V04')
    planche_v04 = next(o for o in v04.childOccurrences if o.component.name == 'D4_REF_Planche_bois').bRepBodies.item(0)
    bois = planche_v04.appearance.name
    couleurs_depuis_rover([bois, BLANC, BLEU_PLA, NOIR, METAL, GRIS_F, GRIS_C])
    nettoyer_r(['RM_'])
    # ---- planche
    _, cp = piece_r('RM_REF_Planche_bois', 'REF_BOIS')
    pb = body(zpoly(cp, 'planche', contour(), ZP, EP), 'REF_planche_remorque', BLANC)
    paint(pb, bois)
    for x, y, L, reb, role in vis_planche():
        trou_z(cp, 'planche_vis_%+d_%d' % (x, y), x, y, trou_vis(role), ZP - 1, EP + 2, [pb])
    etape('planche')
    # ---- croix d'atterrissage (peinture)
    _, cc = piece_r('RM_REF_Croix_peinte', 'REF_PEINTURE')
    body(zpoly(cc, 'croix', [(-18, Y_CROIX - 100), (18, Y_CROIX - 100), (18, Y_CROIX - 18), (100, Y_CROIX - 18), (100, Y_CROIX + 18),
                             (18, Y_CROIX + 18), (18, Y_CROIX + 100), (-18, Y_CROIX + 100), (-18, Y_CROIX + 18), (-100, Y_CROIX + 18),
                             (-100, Y_CROIX - 18), (-18, Y_CROIX - 18)], ZP + EP, 0.3), 'REF_croix_peinte', BLEU_PLA)
    # ---- boîtiers des servos (avant), blocs de charnière (arrière), rebords
    _, cr = piece_r('RM_IMP_Rebords', 'IMPRIME_PLA_BLEU')
    _, ch = piece_r('RM_IMP_Charnieres', 'IMPRIME_PETG')
    reb = {}
    for sx, cote in ((1, 'G'), (-1, 'D')):
        bloc_servo(ch, sx, cote)
        bloc_arriere(ch, sx, cote)
    etape('boîtiers des servos, blocs arrière')
    for nom, ligne in rebords().items():                # avant et arrière : lèvre sur le plateau
        b = body(zpoly(cr, 'rebord_' + nom, bande(ligne, -REB_EP, REB_RECOUV), ZP + EP, REB_LEVRE), 'rebord_' + nom, BLEU_PLA)
        zpoly(cr, 'joue_' + nom, bande(ligne, -REB_EP, 0.0), ZP, EP, JOIN, [b])
        reb[nom] = b
    jo = {}
    for nom, ligne in joues().items():                  # côtés des volets : au ras du plateau, semelle sous la planche
        b = body(zpoly(cr, 'joue_lat_' + nom, bande(ligne, -REB_EP, 0.0), ZP, EP), 'joue_' + nom, BLEU_PLA)
        zpoly(cr, 'semelle_lat_' + nom, bande(ligne, -REB_EP, REB_RECOUV), ZP - JOUE_SEMELLE[1], JOUE_SEMELLE[1], JOIN, [b])
        jo[nom] = b
    dans = lambda b_, x, y: (b_.boundingBox.minPoint.x * 10 <= x <= b_.boundingBox.maxPoint.x * 10
                             and b_.boundingBox.minPoint.y * 10 <= y <= b_.boundingBox.maxPoint.y * 10)
    for x, y, L, nom, role in vis_planche():
        n = '%+d_%d' % (x, y)
        if nom == 'TIMON':                              # vis du timon : écrou noyé dans le timon (fonction timon)
            continue
        if nom != 'BRAS':                               # écrou noyé dans la lèvre du rebord
            hexa_z(cr, 'ecrou_' + n, x, y, M3_HEX, ZP + EP - 0.01, M3_ECROU_H, [reb[nom]])
            trou_z(cr, 'vis_' + n, x, y, 3.4, ZP + EP - 0.01, REB_LEVRE - 1.0, [reb[nom]])
            continue
        js = [b_ for b_ in jo.values() if dans(b_, x, y)]
        if js:
            trou_z(cr, 'vis_joue_' + n, x, y, trou_vis(role), ZP - JOUE_SEMELLE[1] - 1, JOUE_SEMELLE[1] + 2, js)
        bras = [b_ for b_ in ch.bRepBodies if b_.boundingBox.minPoint.z * 10 < ZP - 9 and dans(b_, x, y)]
        if bras:                                        # boîtiers des servos, blocs arrière : trou et écrou noyé sous le bras
            trou_z(ch, 'vis_bras_' + n, x, y, 3.4, ZP - 11, 11.01, bras)
            hexa_z(ch, 'ecrou_bras_' + n, x, y, M3_HEX, ZP - 10 - 0.01, M3_ECROU_H, bras)
    etape('rebords et écrous')
    # ---- volets (composants libres, à la racine : on peut les replier) et SG90
    for sx, cote in ((1, 'G'), (-1, 'D')):
        _, cv = piece_r('RM_IMP_Volet_' + cote, 'IMPRIME_PETG_ET_CONTREPLAQUE', fixe=False)
        pan, _, _ = volet(cv, sx, cote)
        paint(pan, bois)
        etape('volet ' + cote)
        _, cs9 = piece_r('RM_REF_SG90_' + cote, 'MODELE_FREECAD_SG90')
        sg90(cs9, sx, cote)
        etape('SG90 ' + cote)
    for sx, cote in ((1, 'G'), (-1, 'D')):
        occ_ = {o.component.name: o for o in root.occurrences}
        charniere_volet(occ_['RM_IMP_Volet_' + cote], occ_['RM_IMP_Charnieres'], sx, cote)
    etape('charnières des volets')
    # ---- timon
    _, ct = piece_r('RM_IMP_Timon', 'IMPRIME_PLA_BLEU')
    tb, finitions = timon(ct)
    etape('timon')
    # ---- supports de roue et roues (jante, pneu, flasque copiés des roues avant du rover)
    for sx, cote in ((1, 'G'), (-1, 'D')):
        osup, cs = piece_r('RM_IMP_Support_roue_' + cote, 'IMPRIME_PETG')
        support_roue(cs, sx, cote)
        ow, cw = piece_r('RM_IMP_Roue_' + cote, 'IMPRIME_PETG_TPU', fixe=False)
        _, cf = piece_r('RM_REF_Axe_roue_' + cote, 'VISSERIE_AXE_M5')
        roue(cw, cf, sx, cote, v04)
        pivot_roue(ow, osup, sx, cote)
        etape('support et roue ' + cote)

    def libre(x, z):
        return tb.pointContainment(P(x, PIVOT_Y, z)) == adsk.fusion.PointContainment.PointOutsidePointContainment
    zm = TIMON_Z0 + TIMON_EP / 2
    diabolo = {'milieu_r5.2_libre': libre(5.2, zm), 'milieu_r5.5_libre': libre(5.5, zm),
               'haut_r6.4_libre': libre(6.4, TIMON_Z0 + TIMON_EP - 0.1), 'bas_r6.4_libre': libre(6.4, TIMON_Z0 + 0.1)}
    # ---- bilan
    pieces = {}
    for oc in root.occurrences:
        for b in oc.component.bRepBodies:
            bx = b.boundingBox
            if b.name.startswith('REF_') and not b.name.startswith('REF_volet'):
                continue
            pieces[b.name] = {'taille_mm': [round((bx.maxPoint.asArray()[i] - bx.minPoint.asArray()[i]) * 10, 1) for i in range(3)],
                              'volume_cm3': round(b.volume, 2), 'morceaux': b.lumps.count}
    bilan = {'pieces': pieces, 'diabolo': diabolo, 'finitions': finitions, 'vis_planche': vis_planche(), 'patte_vis': PATTE_VIS,
             'pivot_y': PIVOT_Y, 'y_axe': Y_AXE, 'bras_x0': BRAS_X0, 'y_croix': Y_CROIX, 'chanfrein': CHANF,
             'empreintes_dessous': [[sx * a, y0, sx * b_, y1] for sx in (1, -1) for a, b_, y0, y1 in
                                    [(115, LARG / 2, Y_PAL - 9.5, Y_SEM_AV), (115, LARG / 2, Y_SEM_AR, YF + LONG)]],
             'servos': {'y_palonnier': Y_PAL, 'boitier_y': [Y_PAL - 34.0, Y_PAL - 5.5]},
             'volets': {'largeur': VOLET_W, 'y0': V_Y0, 'y1': V_Y1, 'x_axe': X_AX,
                        'trou_x': [round(175.0 - X_AX, 1), round(192.0 - X_AX, 1)], 'trou_y': [13.0, 10.0]},
             'trou_cable_rover': list(TROU_CABLE_ROVER),
             'planche': {'contour': contour(), 'ep': EP, 'z_dessous': ZP}}
    json.dump(bilan, open(DOSSIER + '/remorque_bilan.json', 'w', encoding='utf-8'), indent=1, ensure_ascii=False)
    groupe_rigide()
    volets_replies(True)                             # état enregistré : volets repliés (position de route)
    etape('bilan écrit')
    tl = d.timeline
    print(json.dumps({'diabolo': diabolo, 'finitions': finitions, 'pieces': {k: (v['taille_mm'], v['morceaux']) for k, v in pieces.items()}}, ensure_ascii=False)[:4000])
    print('fonctions KO', [tl.item(i).name for i in range(tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup])
