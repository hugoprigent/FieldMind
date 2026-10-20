from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Outils et cotes de la remorque porte-drone.

La remorque a son propre document Fusion, « remorque fieldmind », inséré par référence dans le
document du rover « prtotype final fieldmind ». Les pièces sont à la racine de ce document, dans le
repère du rover : X à droite, Y vers l'arrière, Z depuis le sol, dessus de la planche du rover à
Z = 135. Insérée à l'identité, la remorque est attelée dans l'axe. Côtés : G = +X, D = -X.
La chape d'attelage reste dans le document du rover (00_chape_rover.py).
"""
import adsk.core, adsk.fusion, math
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/pf_helpers.py'), encoding='utf-8').read(), 'pf_helpers.py', 'exec'))
DOC_ROVER, DOC_REMORQUE = 'prtotype final fieldmind', 'remorque fieldmind'
DOC_PREFIXE = DOC_REMORQUE      # les scripts côté rover repassent DOC_PREFIXE = DOC_ROVER
GROUP = 'FM_Prototype_Final'
DOSSIER = str(_FM_ROOT / 'cao/prototype_final/remorque')
BLEU_PLA = 'FM_PLA_bleu'

# ---- attelage : chape sous l'arrière du rover, tête du timon entre ses deux mâchoires, une vis H M10
#      verticale qui traverse le tout, mise par le dessus ; rondelle et écrou frein M10 dessous
PIVOT_Y = 540.0                 # axe d'attelage : 100 mm derrière la planche du rover (bord arrière à Y = 440)
ZB_ROVER = 135.0                # dessus de la planche du rover (dessous à 130)
PATTE_VIS = [(-14.0, 434.0), (14.0, 434.0)]     # 2 vis M6 dans la planche du rover, derrière l'adaptateur USB du lidar
CHAPE_L = 44.0                  # largeur de la chape
CHAPE_Z0, MACH_INF, MACH_SUP, CHAPE_Z1 = 118.0, 128.0, 147.0, 155.0   # dessous, haut mâchoire basse, bas mâchoire haute, dessus
BAGUE_H, BAGUE_D = 2.0, 18.0    # anneau d'appui de la tête du timon sur la mâchoire basse
OEIL_D = 36.0                   # tête du timon
BASCULE = 12.0                  # évasement du trou en diabolo de la tête (degrés) : tangage et roulis de ±10° contrôlés
M6_HEX, M6_ECROU_H, M6_TETE_H = 10.2, 5.2, 4.2
M10_TROU, M10_TIMON = 10.5, 10.6                  # passage dans la chape, milieu du trou en diabolo du timon
M10_SURPLATS, M10_TETE_H, M10_ECROU_FREIN_H, M10_RONDELLE = 17.0, 6.4, 10.0, (20.0, 2.0)
VIS_ATTELAGE_L = 55.0                             # H M10×55 : chape 37 + 2 rondelles + écrou frein, filet qui dépasse

# ---- planche de la remorque (bois, comme celle du rover)
LARG, LONG, EP = 300.0, 380.0, 5.0
YF = PIVOT_Y + 100.0            # bord avant de la planche
TIMON_EP = 12.0
ZP = 155.5                      # dessous de la planche : relevée pour que les volets dépliés, au ras du plateau, passent au-dessus des roues
TIMON_Z0 = MACH_INF + BAGUE_H                      # dessous du timon (sa tête repose sur l'anneau de la chape) : 130
TIMON_CALE = ZP - (TIMON_Z0 + TIMON_EP)            # cale du timon sous la planche : 8
CHANF = 50.0                    # coins avant coupés à 45° : la remorque braque plus avant de toucher les roues du rover
Y_CROIX = YF + LONG / 2         # centre de la croix d'atterrissage

# ---- rebords (L renversé : lèvre sur le dessus, joue sur la tranche)
REB_EP, REB_RECOUV, REB_LEVRE = 2.4, 14.0, 5.5
VIS_RETRAIT = 9.5               # axe des vis M3 depuis le bord du bois
TIMON_VIS_X, TIMON_VIS_Y = (20.0, 52.0), 25.0   # vis M6 du timon : x ±20 et ±52, 25 mm derrière le bord avant (hors du rebord et des câbles)
M3_HEX, M3_ECROU_H = 5.7, 2.6

# ---- volets latéraux : un panneau de contreplaqué de chaque côté, sur charnière le long du bord.
#      Replié : debout (ridelle), appuyé contre la joue. Déplié : à plat, au ras du plateau, posé sur les
#      butées du boîtier du servo (avant) et du bloc de charnière (arrière). Un SG90 par côté, dans l'axe
#      de la charnière, à l'AVANT, logé dans le coin coupé de la planche : câble court jusqu'au timon.
VOLET_W, VOLET_EP = 110.0, 5.0
X_AX = LARG / 2 + REB_EP + VOLET_EP / 2           # axe de charnière : 154,9 (volet replié contre la joue)
Z_AX = ZP + EP - VOLET_EP / 2                     # 158 : volet déplié au ras du plateau (côtés sans lèvre)
JOUE_SEMELLE = (LARG / 2 - REB_RECOUV, 2.0)       # joues latérales : semelle sous la planche (début en X, épaisseur)
CHARN_R = 4.5                                     # rayon des noeuds de charnière
FERR_AV = (YF + 46.0, YF + 51.5)                  # noeud de la ferrure avant du volet (palonnier du SG90 sur sa face avant)
V_Y0, V_Y1 = YF + 51.5, YF + 368.0                # panneau
FERR_AR = (YF + 368.5, YF + 374.0)                # noeud de la ferrure arrière (axe M3)
BLOC_AR = (YF + 374.4, YF + 382.4)                # bloc de charnière arrière (affleure le rebord arrière)
REB_AV_FIN = 45.0                                 # le rebord avant s'arrête sur le coin coupé, avant le boîtier du servo
YS_M = YF + 224.0                                 # coupe des joues latérales
Z_APPUI = Z_AX - VOLET_EP / 2                     # dessous du volet déplié : 155,5
X_BUTEE = X_AX + 11.0                             # bord extérieur du boîtier et des butées : 165,9
Y_SEM_AV = V_Y0 + 30.0                            # fin de la semelle du boîtier du servo, sous la planche
Y_SEM_AR = V_Y1 - 20.0                            # début de la semelle du bloc arrière
# SG90 (cotes FreeCAD / Tower Pro) : arbre le long de +Y, corps devant le volet
SG90 = {'L': 22.5, 'l': 11.8, 'h': 22.7, 'axe_bout': 5.9, 'bossage': 4.0, 'cannelure': 3.2,
        'oreilles_bas': 4.7, 'oreilles_haut': 5.2, 'oreilles_ep': 2.5, 'oreilles_z': 15.9,
        'trous_oreilles': (-2.4, 24.9)}             # oreilles (modèle FreeCAD) : 4,7 sous le corps, 5,2 au-dessus
Y_PAL = FERR_AV[0]                                # face du palonnier contre la ferrure avant
# ---- câbles des servos : tunnel sous le servo, montée, rainure sous la planche jusqu'au timon, rainure
#      sous chaque bras du timon, prise 4 broches sous sa tête, trou dans la planche du rover
CABLE_L, CABLE_P = 4.5, 3.0
Y_CABLE = YF + 40.0                               # rainure du boîtier et entrée dans le timon, alignées
PRISE = (0.0, PIVOT_Y + 26.0, 12.0, 6.0, 8.0)      # prise 4 broches sous le timon : x, y, largeur, épaisseur, profondeur
TROU_CABLE_ROVER = (0.0, 436.0, 8.0)               # passage vers l'intérieur de la coque, entre les vis de la chape

# ---- roues : mêmes jante, pneu et flasque qu'à l'avant, 2 roulements 5x16x5 par roue dans des bagues
Y_AXE, Z_AXE = YF + 210.0, 75.0       # 20 mm derrière le centre du plateau : la remorque appuie un peu sur l'attelage
X_JAMBE, JAMBE_EP = 152.5, 8.0        # face extérieure de la jambe du support
CALE = 3.0                            # entretoise entre la jambe et la bague intérieure du roulement
X_MOYEU = X_JAMBE + CALE              # face intérieure du moyeu de la jante
BAGUE_L, BAGUE_DE, LOGE_625, LEVRE_625, ARETE = 7.0, 22.0, 16.15, 13.5, 2.0
R625 = (5.0, 16.0, 5.0)
ENTRETOISE = 2 * 2.0 + ARETE + 0.2    # entre les bagues intérieures des deux roulements
APPUI_D = 7.5                         # rondelles et entretoises imprimées : bague intérieure seulement
BRAS_X0 = 75.0                        # bras du support sous la planche, jusque vers le milieu (vis intérieures)
X_VIS_SUPPORT = (LARG / 2 - 12.5, 85.0)   # vis M6 des supports de roue : rangée du bord (l'écrou passe à côté de
                                      # la jambe), rangée vers le milieu (hors de la croix peinte)
BRAS_EP = 10.0                        # épaisseur du bras (vis à tête bombée par le dessus, écrou noyé dessous)


def vis_m6(role):
    """Pièces qui portent la remorque : vis M6. Les autres pièces vissées sur le bois restent en M3."""
    return role in ('support de roue', 'timon')


def trou_vis(role):
    return 6.5 if vis_m6(role) else 3.4


def fraise_vis(role):
    return 12.4 if vis_m6(role) else 6.4


def contour():
    """Contour de la planche (sens trigonométrique vu de dessus)."""
    return [(-LARG / 2 + CHANF, YF), (LARG / 2 - CHANF, YF), (LARG / 2, YF + CHANF), (LARG / 2, YF + LONG),
            (-LARG / 2, YF + LONG), (-LARG / 2, YF + CHANF)]


def rebords():
    """Rebords : nom -> ligne brisée ouverte le long du contour (intérieur à gauche).
    À l'avant, ils s'arrêtent sur le coin coupé, juste avant le boîtier du servo."""
    xc, e = LARG / 2 - CHANF, REB_AV_FIN
    return {
        'AV_G': [(0, YF), (xc, YF), (xc + e, YF + e)],
        'AR_G': [(LARG / 2, YF + LONG), (0, YF + LONG)],
        'AR_D': [(0, YF + LONG), (-LARG / 2, YF + LONG)],
        'AV_D': [(-xc - e, YF + e), (-xc, YF), (0, YF)],
    }


def joues():
    """Joues latérales (bleues, sans lèvre) le long des volets : elles cachent la tranche du bois au ras du
    plateau et leur semelle est pincée sous la planche par les boîtiers des servos, les supports de roue et
    les blocs arrière."""
    x = LARG / 2
    return {'M1_G': [(x, FERR_AV[1] + 0.5), (x, YS_M)], 'M2_G': [(x, YS_M), (x, V_Y1)],
            'M2_D': [(-x, V_Y1), (-x, YS_M)], 'M1_D': [(-x, YS_M), (-x, FERR_AV[1] + 0.5)]}


def vis_planche():
    """Vis M3 de la planche : (x, y, longueur, porteur de l'écrou, rôle).
    Avant et arrière : par-dessous, écrou noyé dans la lèvre du rebord (porteur AV_/AR_).
    Côtés (porteur BRAS) : vis à tête bombée par le dessus (pas de fraisage dans le bois), à travers la semelle de la
    joue et le bras de la pièce, écrou noyé sous le bras ; M6×16 pour les supports de roue, M3×16 sinon."""
    r, out = VIS_RETRAIT, []
    for sx, c in ((1, 'G'), (-1, 'D')):
        xs = sx * (LARG / 2 - r)
        for x in TIMON_VIS_X:                       # timon : 4 vis M6×25 à tête bombée par le dessus, écrou M6 noyé sous le timon
            out.append((sx * x, YF + TIMON_VIS_Y, 25, 'TIMON', 'timon'))
        u = 1 / math.sqrt(2)
        out.append((sx * (LARG / 2 - CHANF / 2 - r * u), YF + CHANF / 2 + r * u, 10, 'AV_' + c, 'rebord'))
        for y in (YF + 57.0, YF + 76.0):
            out.append((xs, y, 16, 'BRAS', 'boîtier du servo'))
        for xv in X_VIS_SUPPORT:                     # supports de roue : 4 vis M6×16 à tête bombée, au bord et vers le milieu
            for dy in (-45.0, 45.0):
                out.append((sx * xv, Y_AXE + dy, 16, 'BRAS', 'support de roue'))
        out.append((xs, Y_SEM_AR + 10.0, 16, 'BRAS', 'bloc de charnière arrière'))
        for x in (100.0, 40.0):
            out.append((sx * x, YF + LONG - r, 10, 'AR_' + c, 'rebord'))
    return out


def doc_rover():
    """Design du document du rover (il doit être ouvert) : roues avant et couleurs."""
    for d_ in app().documents:
        if d_.name.startswith(DOC_ROVER):
            return adsk.fusion.Design.cast(d_.products.itemByProductType('DesignProductType'))
    raise AssertionError('Ouvrir le document « %s »' % DOC_ROVER)


def couleurs_depuis_rover(noms):
    """Copie dans le document de la remorque les apparences du rover qui lui manquent."""
    d, src = design(), doc_rover()
    for n in noms:
        if n and d.appearances.itemByName(n) is None:
            a = src.appearances.itemByName(n)
            if a:
                d.appearances.addByCopy(a, n)


def piece_r(nom, statut, fixe=True):
    """Composant à la racine du document de la remorque (ancré, sauf les volets)."""
    o = design().rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    o.component.name = nom
    o.component.attributes.add('FieldMind', 'statut', statut)
    o.isGrounded = fixe
    return o, o.component


def nettoyer_r(prefixes):
    """Supprime les pièces de la racine dont le composant commence par un des préfixes. Les groupes
    rigides d'abord : Fusion refuse d'enlever les derniers membres d'un groupe."""
    for g in list(design().rootComponent.rigidGroups):
        g.deleteMe()
    for o in list(design().rootComponent.occurrences):
        if o.component.name.startswith(tuple(prefixes)):
            o.deleteMe()


def charnieres_volets():
    """Liaisons pivot des volets (document de la remorque) : {côté: (liaison, valeur repliée)}."""
    out = {}
    for j in design().rootComponent.asBuiltJoints:
        if j.name.startswith('charniere_volet_'):
            lim = j.jointMotion.rotationLimits
            out[j.name[-1]] = (j, lim.minimumValue if abs(lim.minimumValue) > 1e-6 else lim.maximumValue)
    return out


def groupe_rigide():
    """Groupe rigide de toutes les pièces fixes de la remorque (tout sauf volets et roues, reliés par leurs
    liaisons). Indispensable une fois la remorque insérée dans le rover : l'ancrage interne n'y compte
    plus, et sans ce groupe chaque pièce y serait libre."""
    root = design().rootComponent
    for g in list(root.rigidGroups):
        if g.name == 'chassis_remorque':
            g.deleteMe()
    col = adsk.core.ObjectCollection.create()
    for o in root.occurrences:
        if not o.component.name.startswith(('RM_IMP_Volet_', 'RM_IMP_Roue_')):
            col.add(o)
    g = root.rigidGroups.add(col, True)
    g.name = 'chassis_remorque'
    return col.count


def volets_replies(oui):
    """Replie (oui) ou déplie les volets par leurs liaisons pivot."""
    for cote, (j, v) in charnieres_volets().items():
        j.jointMotion.rotationValue = v if oui else 0.0


def tourner(p, rot):
    if rot is None:
        return p
    axe, deg = rot
    c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    x, y, z = p
    if axe == 'x':
        return (x, y * c - z * s_, y * s_ + z * c)
    return (x * c + z * s_, y, -x * s_ + z * c)


def stl(b, chemin, rot):
    """STL binaire du corps, tourné pour l'impression, posé à Z = 0 et centré ; renvoie sa taille (mm)."""
    import struct
    calc = b.meshManager.createMeshCalculator()
    calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.HighQualityTriangleMesh)
    m = calc.calculate()
    co, idx = m.nodeCoordinatesAsDouble, m.nodeIndices
    pts = [tourner((co[3 * i] * 10, co[3 * i + 1] * 10, co[3 * i + 2] * 10), rot) for i in range(len(co) // 3)]
    mn = [min(p[k] for p in pts) for k in range(3)]
    mx = [max(p[k] for p in pts) for k in range(3)]
    dec = [-(mn[0] + mx[0]) / 2, -(mn[1] + mx[1]) / 2, -mn[2]]
    pts = [(p[0] + dec[0], p[1] + dec[1], p[2] + dec[2]) for p in pts]
    with open(chemin, 'wb') as f:
        f.write(('FieldMind remorque ' + b.name).encode()[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(idx) // 3))
        for t in range(len(idx) // 3):
            p = [pts[idx[3 * t + k]] for k in range(3)]
            u = [p[1][i] - p[0][i] for i in range(3)]; v = [p[2][i] - p[0][i] for i in range(3)]
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            l = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5 or 1
            f.write(struct.pack('<12fH', n[0] / l, n[1] / l, n[2] / l, *p[0], *p[1], *p[2], 0))
    return [round(mx[k] - mn[k], 1) for k in range(3)]


# ---- outils de modélisation communs (remorque et chape du rover)
def trou_z(c, nom, x, y, d, z0, h, corps):
    return zcyl(c, nom, x, y, d, z0, h, CUT, corps)


def hexa_z(c, nom, x, y, af, z0, h, corps):
    return zpoly(c, nom, hexagon(None, x, y, af), z0, h, CUT, corps)


def fraisage(c, nom, x, y, z_haut, corps, d=6.4, trou=3.4):
    """Fraisage à 90° pour une vis à tête fraisée (M3 : Ø6,4 ; M6 : Ø12,4), depuis la face z_haut :
    la tête affleure."""
    s_ = sketch(c, plane_z(c, z_haut, nom), nom)
    circle(s_, x, y, d)
    return extrude(c, profiles(s_), (d - trou) / 2, CUT, 'extr_' + nom, corps, down=True, taper_deg=-45)


def anneau_x(c, nom, x0, long_, y, z, de, di, couleur):
    """Anneau d'axe X, de x0 sur long_ (signé)."""
    b = cyl_x(c, nom, x0, y, z, de, long_, couleur)
    s = 1 if long_ > 0 else -1
    cyl_x(c, nom + '_alesage', x0 - s * 0.5, y, z, di, long_ + s * 1.0, op=CUT, bodies=[b])
    return b


def copier_corps(c, src, place, dx, dy, nom, couleur):
    """Copie un corps natif dans le composant c : placé comme son occurrence (place), puis décalé de (dx, dy)."""
    tb = adsk.fusion.TemporaryBRepManager.get()
    t = tb.copy(src)
    tb.transform(t, place)
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(dx / 10, dy / 10, 0)
    tb.transform(t, m)
    bf = c.features.baseFeatures.add()
    bf.startEdit()
    c.bRepBodies.add(t, bf)
    bf.finishEdit()
    bf.name = 'copie_' + nom
    b = bf.bodies.item(0)
    b.name = nom
    paint(b, couleur)
    return b


def lignes_paralleles(b, axe, long_min):
    """Arêtes rectilignes du corps parallèles à l'axe ('x' ou 'z'), plus longues que long_min (mm)."""
    k = 'xyz'.index(axe)
    out = []
    for e in b.edges:
        if e.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
            continue
        ok, pa, pb = e.evaluator.getEndPoints()       # (la géométrie d'une arête n'est pas bornée)
        if not ok:
            continue
        a, c_ = pa.asArray(), pb.asArray()
        d = [c_[i] - a[i] for i in range(3)]
        l = math.sqrt(sum(v * v for v in d)) * 10
        if l > long_min and abs(abs(d[k]) * 10 - l) < 1e-3:
            out.append(e)
    return out


def arrondir(c, nom, corps, axe, long_min, rayons):
    """Congés sur les arêtes rectilignes du corps parallèles à axe : toutes ensemble si possible, sinon
    une par une (repérées par leur milieu), avec le plus grand rayon qui passe. Renvoie {rayon: nombre}."""
    def essai(eds, r, n):
        col = adsk.core.ObjectCollection.create()
        for e in eds:
            col.add(e)
        try:
            fi = c.features.filletFeatures.createInput()
            fi.addConstantRadiusEdgeSet(col, V(r), True)
            c.features.filletFeatures.add(fi).name = n
            return True
        except Exception:
            return False

    def milieu(e):
        ok, a, b = e.evaluator.getEndPoints()
        return tuple(round((a.asArray()[i] + b.asArray()[i]) * 5, 1) for i in range(3))
    aretes = lignes_paralleles(corps, axe, long_min)
    for r in rayons:
        if aretes and essai(aretes, r, nom):
            return {r: len(aretes)}
    vus, bilan, k = set(), {}, 0
    while True:
        cand = [e for e in lignes_paralleles(corps, axe, long_min) if milieu(e) not in vus]
        if not cand:
            return bilan
        e = cand[0]
        vus.add(milieu(e))
        for r in rayons:
            if essai([e], r, '%s_%d' % (nom, k)):
                bilan[r] = bilan.get(r, 0) + 1
                break
        k += 1


def chanfreiner(c, nom, aretes, d):
    col = adsk.core.ObjectCollection.create()
    for e in aretes:
        col.add(e)
    try:
        try:
            ci = c.features.chamferFeatures.createInput2()
            ci.chamferEdgeSets.addEqualDistanceChamferEdgeSet(col, V(d), True)
        except Exception:
            ci = c.features.chamferFeatures.createInput(col, True)
            ci.setToEqualDistance(V(d))
        c.features.chamferFeatures.add(ci).name = nom
        return d
    except Exception:
        return 0


def cyl_y(c, nom, x, z, d, y0, y1, op=NEW, bodies=None, couleur=GRIS_F):
    """Cylindre d'axe Y entre y0 et y1 (le sens d'extrusion suit la normale du plan)."""
    pl = plane_y(c, y0, nom)
    s_ = sketch(c, pl, nom)
    s_.sketchCurves.sketchCircles.addByCenterRadius(s_.modelToSketchSpace(P(x, y0, z)), d / 20)
    vers_plus = (s_.xDirection.crossProduct(s_.yDirection).y > 0) == (y1 > y0)
    i = c.features.extrudeFeatures.createInput(profiles(s_), op)
    i.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(V(abs(y1 - y0))),
                       adsk.fusion.ExtentDirections.PositiveExtentDirection if vers_plus else adsk.fusion.ExtentDirections.NegativeExtentDirection)
    if bodies is not None:
        i.participantBodies = list(bodies)
    f = c.features.extrudeFeatures.add(i)
    f.name = 'extr_' + nom
    if op == NEW:
        return body(f, nom, couleur)
    return f
