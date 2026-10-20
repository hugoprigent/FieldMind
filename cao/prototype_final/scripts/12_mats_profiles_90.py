from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Mâts avec les deux profilés disponibles, de 90 mm, dans le document du rover (en place) :
- caméras : profilé de 95 → 90 mm ; la barre stéréo et les webcams descendent de 5 mm (centres optiques
  à environ 265 mm du sol au lieu de 270) ;
- lidar : profilé de 150 → 90 mm ; le support imprimé est refait d'une seule pièce, 60 mm plus haut :
  - une colonne octogonale de 56 mm (la largeur de la plaque du C1) qui se termine en bas par un
    emboîtement de 31 mm enfilé sur le haut du profilé, comme le support V1 ;
  - des languettes dans les rainures avant et arrière ;
  - 2 vis M8×16 latérales (gauche et droite), 23 mm sous le haut du profilé, dans des écrous en T M8
    rainure 10 glissés par le haut des rainures. Rien par-dessous la planche ;
  - les têtes des vis restent sous le toit de la coque : on visse le mât complet avant de poser la
    coque, qui passe par-dessus (ouverture agrandie : 15_ouverture_coque_lidar.py) ;
  - les puits des vis du C1 sont des gorges ouvertes sur les pans des coins, de la plaque au haut du
    profilé : clé Allen de 2 par le côté ; l'emboîtement reste fermé ;
  - un évidement central allège la colonne ; le haut du profilé bute sur son rebord.
  Le lidar reste à la même hauteur (plan laser à ~322 mm).
Réexécutable : refait tout le corps du support. Exporte le STL du support (plaque du C1 contre le
plateau) et retire de trous_composants.json l'ancien trou de la tige filetée (05_percages.py).
"""
import adsk.core, adsk.fusion, json, math, os, struct
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/pf_helpers.py'), encoding='utf-8').read(), 'pf_helpers.py', 'exec'))
SOL = 'd4_roue_ar_d / 2 + d4_axe_ar_sous_planche + d4_planche_ep'
PROFIL_H = 90.0
REHAUSSE = 60.0             # 150 - 90 : le support remonte d'autant
BAISSE_CAM = 5.0            # 95 - 90
SUP, SUP_CH = 28.0, 13.0    # colonne : octogone de 56 ; pans des coins sur la ligne des vis du C1 (gorges ouvertes)
PLAQUE, PLAQUE_CH, PLAQUE_EP = 28.0, 3.0, 7.5
EMB, EMB_H = 40.5, 31.0     # emboîtement sur le profilé : 0,25 de jeu par côté, 31 de profondeur
VIDE, VIDE_CH, VIDE_HAUT = 17.0, 3.0, -12.0   # évidement central (octogone de 34) jusqu'à 12 mm sous la plaque
LANG_L, LANG_P = 9.6, 1.8   # languettes dans l'ouverture des rainures avant et arrière (10)
M8_TROU, M8_Z = 8.6, 23.0   # vis M8×16 latérales, à 23 mm sous le haut du profilé : têtes 3 mm sous le rebord des flancs de la coque
C1_DEMI, PUITS_D, TETE_H, PASSAGE = 21.5, 6.0, 3.0, 3.0
CABLE_L, CABLE_P = 12.0, 2.0
ECROU_T = (4.4, 17.0, 16.0)  # écrou en T M8 rainure 10 (référence) : épaisseur, largeur, longueur
STL_SUPPORT = str(_FM_ROOT / 'cao/mounts/prototype/direction_v04_servo_traversant/exports/stl/support_lidar_C1.stl')
TROUS_COMPOSANTS = str(_FM_ROOT / 'cao/prototype_final/trous_composants.json')


def octo(demi, ch):
    return [(-demi + ch, -demi), (demi - ch, -demi), (demi, -demi + ch), (demi, demi - ch),
            (demi - ch, demi), (-demi + ch, demi), (-demi, demi - ch), (-demi, -demi + ch)]


def tourner(p, rot):
    if rot is None:
        return p
    axe, deg = rot
    c_, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    x, y, z = p
    if axe == 'x':
        return (x, y * c_ - z * s_, y * s_ + z * c_)
    return (x * c_ + z * s_, y, -x * s_ + z * c_)


def stl(b, chemin, rot):
    """STL binaire du corps, tourné pour l'impression, posé à Z = 0 et centré ; renvoie sa taille (mm)."""
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
        f.write(('FieldMind ' + b.name).encode()[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(idx) // 3))
        for t in range(len(idx) // 3):
            p = [pts[idx[3 * t + k]] for k in range(3)]
            u = [p[1][i] - p[0][i] for i in range(3)]; v = [p[2][i] - p[0][i] for i in range(3)]
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            l = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5 or 1
            f.write(struct.pack('<12fH', n[0] / l, n[1] / l, n[2] / l, *p[0], *p[1], *p[2], 0))
    return [round(mx[k] - mn[k], 1) for k in range(3)]


def param(d, nom, expr, note):
    p = d.userParameters.itemByName(nom)
    if p:
        p.expression = expr
        p.comment = note
    else:
        d.userParameters.add(nom, adsk.core.ValueInput.createByString(expr), 'mm', note)


def run(_context: str):
    d = design()
    v04 = next(o for o in d.rootComponent.occurrences if o.component.name == 'FM_Direction_V04')
    g = v04.component
    # 1. profilés de 90, rehausse du lidar
    param(d, 'd4_profil_h', '%g mm' % PROFIL_H, 'V04 | Profile 40x40 du lidar : 90 mm (profile disponible) ; rehausse imprimee de d4_rehausse_lidar')
    param(d, 'd4_cam_profil_h', '%g mm' % PROFIL_H, 'V04 | Profile 40x40 des cameras : 90 mm (profile disponible) ; cameras 5 mm plus bas')
    param(d, 'd4_rehausse_lidar', '%g mm' % REHAUSSE, 'Rehausse imprimee sous le support du lidar : plan laser a la meme hauteur qu avec 150 mm')
    j = {jj.name: jj for jj in g.joints}
    for nom in ('D4_place_support_lidar', 'D4_place_lidar_C1'):
        e = j[nom].offset.expression
        if 'd4_rehausse_lidar' not in e:
            j[nom].offset.expression = e.replace('d4_profil_h', 'd4_profil_h + d4_rehausse_lidar')
    jb = j['D4_place_barre_stereo']
    jb.offset.expression = '%s - %g mm' % (SOL, BAISSE_CAM)
    # 2. support refait d'une pièce (repère du support : z = 0 sous la plaque du C1 ; haut du profilé à z0)
    sup = next(o for o in g.occurrences if o.component.name == 'D4_IMP_Support_lidar')
    c = sup.component
    for coll in (c.features.extrudeFeatures, c.features.combineFeatures, c.sketches, c.constructionPlanes):
        for x in reversed([coll.item(i) for i in range(coll.count)]):
            try:
                if x.isValid and not x.name.startswith(('esq_jp_', 'plan_jp_')):
                    x.deleteMe()
            except RuntimeError:                  # déjà supprimé avec une fonction dont il dépendait
                pass
    for x in list(c.bRepBodies):
        if x.isValid:
            x.deleteMe()
    z0 = -REHAUSSE                              # haut du profilé
    zb = z0 - EMB_H                             # bas de l'emboîtement
    b = body(zpoly(c, 'colonne', octo(SUP, SUP_CH), zb, -zb), 'support_lidar', BLANC)
    zpoly(c, 'plaque', octo(PLAQUE, PLAQUE_CH), 0, PLAQUE_EP, JOIN, [b])
    zbox(c, 'emboitement', -EMB / 2, -EMB / 2, EMB / 2, EMB / 2, zb - 1, EMB_H + 1, CUT, [b])
    zpoly(c, 'evidement', octo(VIDE, VIDE_CH), z0 - 0.5, VIDE_HAUT - z0 + 0.5, CUT, [b])
    e, l = EMB / 2, LANG_L / 2
    for nom, (x0, y0, x1, y1) in (('Y+', (-l, e - LANG_P, l, e + 0.05)), ('Y-', (-l, -e - 0.05, l, -e + LANG_P))):
        zbox(c, 'languette_' + nom, x0, y0, x1, y1, zb, EMB_H, JOIN, [b])
    for k, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        # gorge ouverte sur le pan du coin, de la plaque jusqu'au haut du profilé (l'emboîtement reste fermé)
        zcyl(c, 'puits_C1_%d' % k, sx * C1_DEMI, sy * C1_DEMI, PUITS_D, z0, -z0 + TETE_H, CUT, [b])
        zcyl(c, 'passage_C1_%d' % k, sx * C1_DEMI, sy * C1_DEMI, PASSAGE, TETE_H - 0.1, PLAQUE_EP, CUT, [b])
    zv = z0 - M8_Z
    for nom, sx in (('D', 1), ('G', -1)):
        xcyl(c, 'trou_M8_' + nom, sx * (e - 1), 0, zv, M8_TROU, sx * (SUP - e + 2), CUT, [b])
    zbox(c, 'gorge_cable', -CABLE_L / 2, PLAQUE - 8, CABLE_L / 2, PLAQUE + 1, PLAQUE_EP - CABLE_P, CABLE_P + 1, CUT, [b])
    # 3. vis M8×16 et écrous en T (références) ; la vis traverse la paroi (SUP - EMB/2) et la lèvre de la rainure
    ep, la, lo = ECROU_T
    for nom, sx in (('D', 1), ('G', -1)):
        v = body(xcyl(c, 'REF_vis_M8_' + nom, sx * SUP, 0, zv, 13.0, sx * 8.0), 'REF_vis_M8x16_support_lidar_' + nom, METAL)
        xcyl(c, 'REF_vis_M8_tige_' + nom, sx * SUP, 0, zv, 8.0, -sx * 16.0, JOIN, [v])
        t = body(zbox(c, 'REF_ecrou_T_' + nom, min(sx * 17.0, sx * (17.0 - ep)) - 0.05 * sx, -la / 2, max(sx * 17.0, sx * (17.0 - ep)) - 0.05 * sx, la / 2, zv - lo / 2, lo), 'REF_ecrou_T_M8_support_lidar_' + nom, METAL)
        xcyl(c, 'REF_ecrou_T_trou_' + nom, sx * 17.5, 0, zv, 8.0, -sx * (ep + 1), CUT, [t])
    # 4. plus de tige filetée : retirer son trou de la planche (05_percages.py le rebouche)
    tc = json.load(open(TROUS_COMPOSANTS, encoding='utf-8'))
    tc['trous'] = [x for x in tc['trous'] if x[3] != 'tige_lidar']
    json.dump(tc, open(TROUS_COMPOSANTS, 'w', encoding='utf-8'), indent=1)
    # 5. STL du support : plaque du C1 contre le plateau (colonne vers le haut, emboîtement en haut)
    taille = stl(b, STL_SUPPORT, ('x', 180))
    bb = sup.boundingBox
    out = {'support_mm': taille, 'volume_cm3': round(b.volume, 1), 'morceaux': b.lumps.count,
           'support_z': [round(bb.minPoint.z * 10, 1), round(bb.maxPoint.z * 10, 1)],
           'vis_M8_z_monde': round(zv + bb.maxPoint.z * 10 - PLAQUE_EP, 1),
           'offsets': {n: j[n].offset.expression for n in ('D4_place_support_lidar', 'D4_place_lidar_C1', 'D4_place_barre_stereo')}}
    for o in g.occurrences:
        if o.component.name.startswith(('D4_REF_Profile', 'D4_REF_Lidar', 'D4_IMP_Barre', 'D4_REF_Webcam')):
            bx = o.boundingBox
            out[o.component.name] = [round(bx.minPoint.z * 10, 1), round(bx.maxPoint.z * 10, 1)]
    tl = d.timeline
    out['ko'] = [tl.item(i).name for i in range(max(0, tl.count - 60), tl.count) if tl.item(i).healthState != 0 and not tl.item(i).isGroup]
    out['liaisons_ko'] = [jj.name for jj in g.joints if jj.healthState != 0]
    print(texte(json.dumps(out, ensure_ascii=False)))
