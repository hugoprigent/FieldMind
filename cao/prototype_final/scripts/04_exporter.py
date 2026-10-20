from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""STL des pièces imprimées du prototype final, déjà orientées pour l'impression (posées à Z = 0,
centrées sur le plateau), STEP + F3D du groupe, manifeste.

Orientations (aucun support, voir README) :
- flancs (8 pièces) : couchés sur leur paroi extérieure ;
- nez central : debout sur sa face avant ;
- bandes centrales des tronçons 2 à 4 : dessus contre le plateau ;
- berceau de batterie : fond contre le plateau.
"""
import adsk.core, adsk.fusion, json, math, os, struct
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/pf_helpers.py'), encoding='utf-8').read(), 'pf_helpers.py', 'exec'))
OUT = str(_FM_ROOT / 'cao/prototype_final/exports')
PLATEAU = 180.0
# corps -> (fichier, couleur conseillée, rotation (axe, degrés), orientation)
PIECES = {
    'coque_C1': ('coque_C1_nez', 'bleu', ('x', 90), 'debout sur la face avant (grille)'),
    'coque_C2': ('coque_C2_bande_avant', 'bleu', ('x', 180), 'dessus contre le plateau'),
    'coque_C3': ('coque_C3_bande_ecran_knob', 'bleu', ('x', 180), 'dessus contre le plateau'),
    'coque_C4': ('coque_C4_bande_arriere_lidar', 'bleu', ('x', 180), 'dessus contre le plateau'),
}
for _k, _nom in ((1, 'nez'), (2, 'flanc_avant'), (3, 'flanc_milieu'), (4, 'flanc_arriere')):
    PIECES['coque_G%d' % _k] = ('coque_G%d_%s_gauche' % (_k, _nom), 'blanc', ('y', 90), 'couché sur la paroi extérieure')
    PIECES['coque_D%d' % _k] = ('coque_D%d_%s_droit' % (_k, _nom), 'blanc', ('y', -90), 'couché sur la paroi extérieure')
PIECES['berceau_batterie'] = ('berceau_batterie', 'blanc', None, 'fond contre le plateau')
PIECES['capuchon_knob'] = ('capuchon_knob', 'blanc', None, 'debout, alésage en D contre le plateau')
PIECES['support_xl4015'] = ('support_xl4015', 'blanc', None, 'à plat, dessous contre le plateau (poches des écrous en dessous)')


def tourner(p, rot):
    if rot is None:
        return p
    axe, deg = rot
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    x, y, z = p
    if axe == 'x':
        return (x, y * c - z * s, y * s + z * c)
    return (x * c + z * s, y, -x * s + z * c)


def stl(b, chemin, rot):
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


def run(_context: str):
    d = design()
    em = d.exportManager
    g = group()
    os.makedirs(OUT + '/stl', exist_ok=True)
    man = {'pieces': [], 'plateau_mm': PLATEAU}
    for o in g.component.occurrences:
        for b in o.component.bRepBodies:
            if b.name not in PIECES:
                continue
            nom, coul, rot, orient = PIECES[b.name]
            taille = stl(b, '%s/stl/%s.stl' % (OUT, nom), rot)
            man['pieces'].append({'stl': 'stl/%s.stl' % nom, 'quantite': 1, 'couleur': coul, 'orientation': orient,
                                  'taille_impression_mm': taille, 'tient_sur_A1_mini': all(t <= PLATEAU for t in taille),
                                  'volume_cm3': round(b.volume, 1)})
    em.execute(em.createSTEPExportOptions(OUT + '/prototype_final.step', g.component))
    em.execute(em.createFusionArchiveExportOptions(OUT + '/prototype_final.f3d', g.component))
    open(OUT + '/manifest.json', 'w', encoding='utf-8').write(texte(json.dumps(man, ensure_ascii=False, indent=1)))
    print(json.dumps([(p['stl'], p['taille_impression_mm'], p['tient_sur_A1_mini'], p['volume_cm3']) for p in man['pieces']], ensure_ascii=False))
