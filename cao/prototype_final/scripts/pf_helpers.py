from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Outils communs du prototype final (document Fusion « prtotype final fieldmind »).

Reprend d3_helpers (direction V03/V04) et l'adapte : autre document, groupe FM_Prototype_Final,
pièces posées directement dans le repère du rover (groupe à l'identité, composants ancrés).
Repère : X vers la droite, Y vers l'arrière (avant = -Y), Z vers le haut depuis le sol ; axe des
pivots avant en Y = 0 ; dessus de la planche à Z = ZB.
"""
import adsk.core, adsk.fusion, math
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
DOC_PREFIXE = 'prtotype final fieldmind'
GROUP = 'FM_Prototype_Final'
ZB = 140.0          # dessus de la planche (roues arrière Ø200, axe 30 mm sous la planche, planche de 10 mm : 2 × 5 collées)



def texte(s):
    """Remet d'aplomb les accents écrits dans le script principal : le serveur MCP de Fusion le lit en
    latin-1 (« Ã© » pour « é »). Le texte des helpers, lus en UTF-8, n'est pas touché."""
    import re

    def un(m):
        try:
            return m.group().encode('latin-1').decode('utf-8')
        except UnicodeError:
            return m.group()
    return re.sub('[Â-ô][\u0080-¿]+', un, s)


BLANC, ORANGE, BLEU = 'Opaque(251,250,245)', 'FM_PROVISOIRE_orange', 'FM_PERFBOARD_bleu'
NOIR, GRIS_F, GRIS_C, METAL = 'Opaque(51,51,51)', 'Opaque(76,76,76)', 'Opaque(204,204,204)', 'FM_METAL'
ROUGE, VERT, JAUNE = 'Opaque(255,0,0)', 'Opaque(0,153,0)', 'Opaque(255,255,0)'


def piece(name, statut):
    """Composant ancré dans le groupe, repère confondu avec celui du rover."""
    o, c = part(name, statut=statut)
    o.isGroundToParent = True
    return o, c


def nettoyer(prefixes):
    g = group().component
    for oo in list(g.occurrences):
        if oo.component.name.startswith(tuple(prefixes)):
            oo.deleteMe()


def boite(c, name, x0, y0, x1, y1, z0, h, color=GRIS_F, op=NEW, bodies=None):
    """Pavé en coordonnées du rover (z0 depuis le sol)."""
    f = zbox(c, name, min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1), z0, h, op, bodies)
    if op == NEW:
        return body(f, name, color)
    return f


def cyl_z(c, name, x, y, d, z0, h, color=GRIS_F, op=NEW, bodies=None):
    f = zcyl(c, name, x, y, d, z0, h, op, bodies)
    return body(f, name, color) if op == NEW else f


def cyl_x(c, name, x0, y, z, d, length, color=GRIS_F, op=NEW, bodies=None):
    f = xcyl(c, name, x0, y, z, d, length, op, bodies)
    return body(f, name, color) if op == NEW else f


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


def prisme_x(c, name, x_mil, pts_yz, larg, op=NEW, bodies=None):
    """Polygone dans le plan YZ (x = x_mil), extrudé symétriquement sur larg selon X."""
    s = sketch(c, plane_x(c, x_mil, name) if abs(x_mil) > 1e-9 else c.yZConstructionPlane, name)
    poly3(s, [(x_mil, y, z) for y, z in pts_yz])
    return sym(c, profiles(s), larg, op, 'extr_' + name, bodies)


def prisme_y(c, name, y_mil, pts_xz, long_, op=NEW, bodies=None):
    """Polygone dans le plan XZ (y = y_mil), extrudé symétriquement sur long_ selon Y."""
    s = sketch(c, plane_y(c, y_mil, name), name)
    poly3(s, [(x, y_mil, z) for x, z in pts_xz])
    return sym(c, profiles(s), long_, op, 'extr_' + name, bodies)


def decale(poly, d):
    """Décale une ligne brisée ouverte [(u, v), ...] de d vers sa gauche (sens de parcours)."""
    lignes = []
    for (u0, v0), (u1, v1) in zip(poly[:-1], poly[1:]):
        l = math.hypot(u1 - u0, v1 - v0)
        nu, nv = -(v1 - v0) / l, (u1 - u0) / l
        lignes.append(((u0 + d * nu, v0 + d * nv), (u1 - u0, v1 - v0)))
    pts = [lignes[0][0]]
    for (p0, d0), (p1, d1) in zip(lignes[:-1], lignes[1:]):
        det = d0[0] * (-d1[1]) - d0[1] * (-d1[0])
        if abs(det) < 1e-9:
            pts.append(p1)
            continue
        t = ((p1[0] - p0[0]) * (-d1[1]) - (p1[1] - p0[1]) * (-d1[0])) / det
        pts.append((p0[0] + t * d0[0], p0[1] + t * d0[1]))
    (p, dd) = lignes[-1]
    pts.append((p[0] + dd[0], p[1] + dd[1]))
    return pts


def bande(poly, d0, d1):
    """Bande fermée entre les décalés à d0 et d1 (vers la gauche) d'une ligne brisée."""
    a, b = decale(poly, d0), decale(poly, d1)
    return a + b[::-1]


def importer_step(c, chemin, origine, ax, ay, az, nom):
    """Importe un modèle STEP dans le composant c, à la place voulue : le point (0,0,0) du STEP en origine
    (mm), ses axes X, Y, Z sur ax, ay, az. Les corps sont recopiés dans c (fonction de base, couleurs
    gardées) puis l'import est supprimé : Fusion ne sait pas placer durablement une occurrence imbriquée.
    Corps nommés REF_<nom>_<n>."""
    im = app().importManager
    avant = c.occurrences.count
    im.importToTarget(im.createSTEPImportOptions(chemin), c)
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(*origine), adsk.core.Vector3D.create(*ax), adsk.core.Vector3D.create(*ay), adsk.core.Vector3D.create(*az))
    tb = adsk.fusion.TemporaryBRepManager.get()
    copies = []

    def parcourir(oc, t):
        for b in oc.component.bRepBodies:
            tmp = tb.copy(b)
            tb.transform(tmp, t)
            copies.append((tmp, b.appearance))
        for o2 in oc.component.occurrences:
            t2 = o2.transform2.copy()
            t2.transformBy(t)
            parcourir(o2, t2)
    nouveaux = [c.occurrences.item(i) for i in range(avant, c.occurrences.count)]
    for o in nouveaux:
        t = o.transform2.copy()
        t.transformBy(m)
        parcourir(o, t)
    for o in nouveaux:
        o.deleteMe()
    bf = c.features.baseFeatures.add()
    bf.startEdit()
    for tmp, _ in copies:
        c.bRepBodies.add(tmp, bf)
    bf.finishEdit()
    bf.name = 'import_' + nom
    for k in range(bf.bodies.count):
        b = bf.bodies.item(k)
        b.name = 'REF_%s_%d' % (nom, k + 1)
        if k < len(copies) and copies[k][1] is not None:
            try:
                b.appearance = copies[k][1]
            except Exception:
                pass
    return [bf.bodies.item(k) for k in range(bf.bodies.count)]
