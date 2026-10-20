"""FieldMind direction V03 simple : outils communs (copie adaptée des outils V02), exécutés dans Fusion par le serveur MCP local.
Expressions en mm ; l'API Fusion travaille en cm. Repère : Z vers le haut, avant = -Y,
droite = +X. Chaque pièce est dessinée dans son repère local (Z = 0 à hauteur d'axe de roue),
puis placée par des liaisons dont les décalages sont des expressions de paramètres.
"""
import adsk.core, adsk.fusion, math

DOC_PREFIXE = 'systeme de direction simple'
GROUP = 'FM_Direction_simple_V03'
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def app():
    return adsk.core.Application.get()


def design():
    a = app()
    assert a.activeDocument.name.startswith(DOC_PREFIXE), 'Document actif inattendu : ' + a.activeDocument.name
    return adsk.fusion.Design.cast(a.activeProduct)


def P(x, y, z=0.0):
    """Point en mm -> cm."""
    return adsk.core.Point3D.create(x / 10, y / 10, z / 10)


def V(expr):
    if isinstance(expr, (int, float)):
        expr = '%r mm' % expr
    return adsk.core.ValueInput.createByString(expr)


def ev(expr):
    """Valeur en mm d'une expression."""
    return design().unitsManager.evaluateExpression(expr, 'mm') * 10


def param(name, expr, comment, unit='mm'):
    d = design()
    p = d.userParameters.itemByName(name)
    if p:
        p.expression = expr
        p.comment = comment
    else:
        d.userParameters.add(name, adsk.core.ValueInput.createByString(expr), unit, comment)
    return name


def group():
    root = design().rootComponent
    for o in root.occurrences:
        if o.component.name == GROUP:
            return o
    o = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    o.component.name = GROUP
    return o


def part(name, xyz=(0, 0, 0), statut='ETUDE_V02_NON_VALIDEE'):
    g = group().component
    for o in g.occurrences:
        if o.component.name == name:
            raise AssertionError('Composant deja present : ' + name)
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(*[v / 10 for v in xyz])
    o = g.occurrences.addNewComponent(m)
    o.component.name = name
    o.isGroundToParent = False      # Fusion ancre le premier sous-composant d'un document neuf
    o.component.attributes.add('FieldMind', 'statut', statut)
    return o, o.component


def occ(name):
    return next(o for o in group().component.occurrences if o.component.name == name)


def plane_z(c, expr, name):
    if isinstance(expr, (int, float)) and abs(expr) < 1e-9:
        return c.xYConstructionPlane
    i = c.constructionPlanes.createInput()
    i.setByOffset(c.xYConstructionPlane, V(expr))
    p = c.constructionPlanes.add(i)
    p.name = 'plan_' + name
    p.isLightBulbOn = False
    return p


def plane_x(c, x, name):
    """Plan parallele a YZ, a l'abscisse x (mm)."""
    if abs(x) < 1e-9:
        return c.yZConstructionPlane
    i = c.constructionPlanes.createInput()
    i.setByOffset(c.yZConstructionPlane, V(x))
    p = c.constructionPlanes.add(i)
    p.name = 'plan_' + name
    p.isLightBulbOn = False
    return p


def sketch(c, pl, name):
    s = c.sketches.add(pl)
    s.name = 'esq_' + name
    s.isLightBulbOn = False
    return s


def sp(s, x, y, z=None):
    """Point modele (mm) -> point esquisse. z par defaut = hauteur du plan."""
    if z is None:
        return adsk.core.Point3D.create(x / 10, y / 10, 0)
    return s.modelToSketchSpace(P(x, y, z))


def circle(s, x, y, d, z=None):
    return s.sketchCurves.sketchCircles.addByCenterRadius(sp(s, x, y, z), d / 20)


def rect(s, x0, y0, x1, y1):
    return s.sketchCurves.sketchLines.addTwoPointRectangle(sp(s, x0, y0), sp(s, x1, y1))


def poly(s, pts, z=None):
    lines = s.sketchCurves.sketchLines
    first = prev = None
    for i, (x, y) in enumerate(pts):
        p = sp(s, x, y, z)
        if i == 0:
            first = p
            prev = p
            continue
        ln = lines.addByTwoPoints(prev, p)
        prev = ln.endSketchPoint
        if i == 1:
            first = ln.startSketchPoint
    lines.addByTwoPoints(prev, first)


def hexagon(s, x, y, af, z=None, vertex_dir=(0.0, 1.0)):
    """Hexagone de cote sur plat af, un sommet dans la direction vertex_dir (repere modele)."""
    r = af / math.sqrt(3)
    a0 = math.atan2(vertex_dir[1], vertex_dir[0])
    return [(x + r * math.cos(a0 + k * math.pi / 3), y + r * math.sin(a0 + k * math.pi / 3)) for k in range(6)]


def profiles(s, pick='all'):
    col = adsk.core.ObjectCollection.create()
    profs = [s.profiles.item(i) for i in range(s.profiles.count)]
    if pick == 'all':
        for p in profs:
            col.add(p)
    elif pick == 'largest':
        col.add(max(profs, key=lambda p: p.areaProperties().area))
    elif pick == 'smallest':
        col.add(min(profs, key=lambda p: p.areaProperties().area))
    elif pick == 'ring':
        col.add(next(p for p in profs if p.profileLoops.count == 2))
    else:
        for p in profs:
            if pick(p):
                col.add(p)
    return col


def extrude(c, profs, dist, op, name, bodies=None, down=False, taper_deg=None, start_offset=None):
    i = c.features.extrudeFeatures.createInput(profs, op)
    direction = adsk.fusion.ExtentDirections.NegativeExtentDirection if down else adsk.fusion.ExtentDirections.PositiveExtentDirection
    ext = adsk.fusion.DistanceExtentDefinition.create(V(dist))
    if taper_deg is None:
        i.setOneSideExtent(ext, direction)
    else:
        i.setOneSideExtent(ext, direction, adsk.core.ValueInput.createByString('%r deg' % taper_deg))
    if start_offset is not None:
        i.startExtent = adsk.fusion.OffsetStartDefinition.create(V(start_offset))
    if bodies is not None:
        i.participantBodies = list(bodies)
    f = c.features.extrudeFeatures.add(i)
    f.name = name
    return f


def zbox(c, name, x0, y0, x1, y1, z0, h, op=NEW, bodies=None):
    s = sketch(c, plane_z(c, z0, name), name)
    rect(s, x0, y0, x1, y1)
    return extrude(c, profiles(s), h, op, 'extr_' + name, bodies)


def zcyl(c, name, x, y, d, z0, h, op=NEW, bodies=None, down=False):
    s = sketch(c, plane_z(c, z0, name), name)
    circle(s, x, y, d)
    return extrude(c, profiles(s), h, op, 'extr_' + name, bodies, down=down)


def zpoly(c, name, pts, z0, h, op=NEW, bodies=None, down=False):
    s = sketch(c, plane_z(c, z0, name), name)
    poly(s, pts)
    return extrude(c, profiles(s), h, op, 'extr_' + name, bodies, down=down)


def xcyl(c, name, x0, y, z, d, length, op=NEW, bodies=None):
    """Cylindre d'axe X partant de x0 sur `length` (signe = sens)."""
    s = sketch(c, plane_x(c, x0, name), name)
    circle(s, x0, y, d, z)
    return extrude(c, profiles(s), abs(length), op, 'extr_' + name, bodies, down=length < 0)


def xpoly(c, name, x0, pts_yz, length, op=NEW, bodies=None):
    s = sketch(c, plane_x(c, x0, name), name)
    lines = s.sketchCurves.sketchLines
    sk = [s.modelToSketchSpace(P(x0, y, z)) for (y, z) in pts_yz]
    prev = None
    first = None
    for i, p in enumerate(sk[1:] + [None]):
        a = prev if prev is not None else sk[0]
        b = p if p is not None else first
        ln = lines.addByTwoPoints(a, b)
        if first is None:
            first = ln.startSketchPoint
        prev = ln.endSketchPoint
    return extrude(c, profiles(s), abs(length), op, 'extr_' + name, bodies, down=length < 0)


def body(f, name, color='FM_PROVISOIRE_orange'):
    b = f.bodies.item(0)
    b.name = name
    a = design().appearances.itemByName(color)
    if a:
        b.appearance = a
    return b


def paint(b, color):
    a = design().appearances.itemByName(color)
    if a:
        b.appearance = a


def joint_point(c, name, x, y, z=0.0):
    s = sketch(c, plane_z(c, z, 'jp_' + name), 'jp_' + name)
    return s.sketchPoints.add(sp(s, x, y))


def proxy(entity, o):
    return entity.createForAssemblyContext(o)


def origin_geo(o):
    return adsk.fusion.JointGeometry.createByPoint(o.component.originConstructionPoint.createForAssemblyContext(o))


def point_geo(pt, o):
    return adsk.fusion.JointGeometry.createByPoint(pt.createForAssemblyContext(o))


def interferences(bodies_list):
    d = design()
    col = adsk.core.ObjectCollection.create()
    for b in bodies_list:
        col.add(b)
    i = d.createInterferenceInput(col)
    i.areCoincidentFacesIncluded = False
    res = d.analyzeInterference(i)
    out = []
    for r in res:
        out.append((r.entityOne.name, r.entityTwo.name, round(r.interferenceBody.volume * 1000, 3)))
    return out
