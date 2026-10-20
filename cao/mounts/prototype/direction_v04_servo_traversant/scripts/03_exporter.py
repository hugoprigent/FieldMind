from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""STL des pièces imprimées (une par forme : les paliers et entretoises identiques ne sont exportés
qu'une fois), supports du lidar et des webcams compris, STEP + F3D du groupe FM_Direction_V04, manifeste."""
import adsk.core, adsk.fusion, json, math, os, struct, time
exec(compile(open(str(_FM_ROOT / 'cao/mounts/prototype/direction_v03_simple/scripts/d3_helpers.py'), encoding='utf-8').read(), 'd3_helpers.py', 'exec'))
GROUP = 'FM_Direction_V04'
OUT = str(_FM_ROOT / 'cao/mounts/prototype/direction_v04_servo_traversant/exports')
A_EXPORTER = {  # corps -> (nom de fichier, quantité à imprimer)
    'support_pivot_D': ('support_pivot_D', 1), 'support_pivot_G': ('support_pivot_G', 1),
    'fusee_D': ('fusee_D', 1), 'fusee_G': ('fusee_G', 1),
    'rondelle_2': ('rondelle_2mm', 4),   # 1 sous la tête de chaque vis-pivot + 1 sous la tête de chaque vis de roue ; entre les 608ZZ : précharge (pivots), entretoise 2,2 (roues)
    'palonnier': ('palonnier', 1), 'biellette_D': ('biellette', 2),
    'jante_D': ('jante_608ZZ', 2), 'pneu_TPU_D': ('pneu_TPU', 2), 'flasque_pneu_D': ('flasque_pneu', 2),
    'support_lidar': ('support_lidar_C1', 1), 'barre_stereo': ('barre_stereo', 1),
}


def stl_maillage(b, chemin):
    """Secours : l'export STL de Fusion répond parfois « réussi » sans écrire le fichier (vu sur le
    palonnier à empreinte en étoile, 15/10). On écrit alors le maillage nous-mêmes (mm)."""
    calc = b.meshManager.createMeshCalculator()
    calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.VeryHighQualityTriangleMesh)
    m = calc.calculate()
    co, idx = m.nodeCoordinatesAsDouble, m.nodeIndices
    with open(chemin, 'wb') as f:
        f.write(('FieldMind ' + b.name).encode()[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(idx) // 3))
        for t in range(len(idx) // 3):
            p = [[v * 10 for v in co[3 * idx[3 * t + k]: 3 * idx[3 * t + k] + 3]] for k in range(3)]
            u = [p[1][i] - p[0][i] for i in range(3)]; v = [p[2][i] - p[0][i] for i in range(3)]
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            l = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5 or 1
            f.write(struct.pack('<12fH', n[0] / l, n[1] / l, n[2] / l, *p[0], *p[1], *p[2], 0))


# pièces exportées déjà couchées pour l'impression : rotation autour de X (degrés)
COUCHER = {'barre_stereo': 170.0}   # assise inclinée de 10° posée à plat sur le plateau


def coucher(chemin, deg):
    """Tourne un STL binaire autour de X, recalcule les normales et pose le point le plus bas à Z = 0."""
    b = open(chemin, 'rb').read()
    n = struct.unpack('<I', b[80:84])[0]
    c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    tris = []
    for t in range(n):
        v = struct.unpack('<12f', b[84 + 50 * t: 84 + 50 * t + 48])
        tris.append([(v[3 * k + 3], v[3 * k + 4] * c - v[3 * k + 5] * s_, v[3 * k + 4] * s_ + v[3 * k + 5] * c) for k in range(3)])
    zmin = min(p[2] for tr in tris for p in tr)
    with open(chemin, 'wb') as f:
        f.write(b[:80])
        f.write(struct.pack('<I', n))
        for tr in tris:
            p = [(x, y, z - zmin) for x, y, z in tr]
            u = [p[1][i] - p[0][i] for i in range(3)]; w = [p[2][i] - p[0][i] for i in range(3)]
            nn = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
            l = (nn[0] ** 2 + nn[1] ** 2 + nn[2] ** 2) ** 0.5 or 1
            f.write(struct.pack('<12fH', nn[0] / l, nn[1] / l, nn[2] / l, *p[0], *p[1], *p[2], 0))


def run(_context: str):
    d = design()
    em = d.exportManager
    g = group()
    next(j for j in g.component.joints if j.name == 'D4_SERVO_DIRECTION').jointMotion.rotationValue = 0
    os.makedirs(OUT + '/stl', exist_ok=True)
    man = {'d4_E_mm': ev('d4_E'), 'd4_planche_ep_mm': ev('d4_planche_ep'), 'd4_biellette_L_mm': round(ev('d4_biellette_L'), 2), 'd4_axe_av_sous_planche_mm': round(ev('d4_axe_av_sous_planche'), 1), 'pieces': []}
    for o in g.component.occurrences:
        for b in o.component.bRepBodies:
            if b.name not in A_EXPORTER:
                continue
            nom, qte = A_EXPORTER[b.name]
            if any(p['stl'] == 'stl/%s.stl' % nom for p in man['pieces']):
                continue
            chemin = '%s/stl/%s.stl' % (OUT, nom)
            avant = os.path.getmtime(chemin) if os.path.exists(chemin) else 0
            t0 = time.time()
            opt = em.createSTLExportOptions(b, chemin)
            opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
            em.execute(opt)
            if not os.path.exists(chemin) or os.path.getmtime(chemin) in (avant,) or os.path.getmtime(chemin) < t0 - 1:
                stl_maillage(b, chemin)
            if b.name in COUCHER:
                if open(chemin, 'rb').read(5).lower() == b'solid':   # STL texte : réécrire en binaire
                    stl_maillage(b, chemin)
                coucher(chemin, COUCHER[b.name])
            bb = b.boundingBox
            man['pieces'].append({'stl': 'stl/%s.stl' % nom, 'quantite': qte, 'corps_cao': b.name,
                                  'encombrement_mm': [round((bb.maxPoint.asArray()[i] - bb.minPoint.asArray()[i]) * 10, 1) for i in range(3)],
                                  'volume_cm3': round(b.volume, 1)})
    if os.path.exists(OUT + '/stl/rondelle_M3_0.6mm.stl'):   # rondelles des biellettes : anneau généré hors Fusion
        man['pieces'].append({'stl': 'stl/rondelle_M3_0.6mm.stl', 'quantite': 8, 'corps_cao': None, 'encombrement_mm': [5.0, 5.0, 0.6],
                              'note': 'anneau Ø5 x Ø3,3 : appui sur la bague intérieure des 3x10x4 seulement (remplace les rondelles M3 métal)'})
    for h in ('1.8', '1.9'):   # rondelles de précharge des pivots : anneaux générés hors Fusion
        if os.path.exists('%s/stl/rondelle_precharge_%smm.stl' % (OUT, h)):
            man['pieces'].append({'stl': 'stl/rondelle_precharge_%smm.stl' % h, 'quantite': 2, 'corps_cao': None,
                                  'encombrement_mm': [12.0, 12.0, float(h)], 'note': 'anneau Ø12 x Ø8,4 entre les 608ZZ des fusées'})
    if os.path.exists(OUT + '/stl/entretoise_roue_2.2mm.stl'):   # entre les 608ZZ des roues : généré par 16_roue_annexes.py
        man['pieces'].append({'stl': 'stl/entretoise_roue_2.2mm.stl', 'quantite': 2, 'corps_cao': None, 'encombrement_mm': [12.0, 12.0, 2.2],
                              'note': 'anneau Ø12 x Ø8,4 entre les bagues intérieures des 608ZZ de chaque roue, 0,2 de plus que l arête de la jante'})
    em.execute(em.createSTEPExportOptions(OUT + '/direction_V04.step', g.component))
    em.execute(em.createFusionArchiveExportOptions(OUT + '/direction_V04.f3d', g.component))
    open(OUT + '/manifest.json', 'w', encoding='utf-8').write(json.dumps(man, ensure_ascii=False, indent=1))
    print(json.dumps(man, ensure_ascii=False))
