from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Centre de gravité du rover et répartition des charges, dans le document du rover (lecture seule).

Masses : masses.json (pièces imprimées par leur volume, bois, profilés au mètre, visserie en acier, le
reste estimé en attendant les pesées). Positions : centres de gravité des corps dans l'assemblage.
Essieu avant en Y = 0 (axe des roues avant), essieu arrière en Y = d4_empattement. La remorque n'est
pas comptée, sauf sa charge estimée sur l'attelage (cas « avec remorque »).
Écrit centre_de_gravite.json.
"""
import adsk.core, adsk.fusion, json, math
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/pf_helpers.py'), encoding='utf-8').read(), 'pf_helpers.py', 'exec'))
DOSSIER = str(_FM_ROOT / 'cao/prototype_final')


def corps(o):
    out = list(o.bRepBodies)
    for oo in o.childOccurrences:
        out += corps(oo)
    return out


def run(_context: str):
    d = design()
    M = json.load(open(DOSSIER + '/masses.json', encoding='utf-8'))
    imp, bois = M['imprime'], M['bois']
    lignes, ignores = [], []
    for g in d.rootComponent.occurrences:
        if g.component.name not in ('FM_Prototype_Final', 'FM_Direction_V04'):
            continue
        for o in g.childOccurrences:
            n = o.component.name
            bs = [b for b in corps(o) if b.isSolid]
            if not bs:
                continue
            table = next((k for k in M['composants'] if n.startswith(k)), None)
            for b in bs:
                pp = b.physicalProperties
                v = pp.volume                                   # cm³
                cg = [x * 10 for x in pp.centerOfMass.asArray()]
                nb = b.name.lower()
                if table:
                    continue                                    # traité plus bas, en bloc
                if nb.startswith('ref_') and any(w in nb for w in M['metal_mots']):
                    m = v * M['metal_densite_g_cm3']             # visserie de référence rangée dans une pièce imprimée
                elif n.startswith(tuple(imp['prefixes'])):
                    m = v * imp['densite_g_cm3'] * imp['remplissage']
                elif n.startswith(tuple(bois['prefixes'])):
                    m = v * bois['densite_g_cm3']
                elif n.startswith('D4_REF_Profile'):
                    bx = b.boundingBox
                    m = M['profile_g_par_mm'] * (bx.maxPoint.z - bx.minPoint.z) * 10
                elif any(w in nb for w in M['metal_mots']):
                    m = v * M['metal_densite_g_cm3']
                else:
                    ignores.append('%s/%s' % (n, b.name))
                    continue
                lignes.append((n, m, cg))
            if table:                                           # masse du tableau, au centre des volumes du composant
                vt = sum(b.physicalProperties.volume for b in bs) or 1
                cg = [sum(b.physicalProperties.centerOfMass.asArray()[i] * 10 * b.physicalProperties.volume for b in bs) / vt for i in range(3)]
                lignes.append((n, M['composants'][table]['g'], cg))
    ey = ev('d4_empattement')

    def bilan(lst):
        mt = sum(m for _, m, _ in lst)
        c = [sum(m * p[i] for _, m, p in lst) / mt for i in range(3)]
        av = (ey - c[1]) / ey
        return {'masse_kg': round(mt / 1000, 2), 'cg_mm': [round(v, 1) for v in c],
                'avant_pct': round(100 * av, 1), 'arriere_pct': round(100 * (1 - av), 1),
                'gauche_pct (+X)': round(100 * (0.5 + c[0] / 390.0), 1), 'droite_pct (-X)': round(100 * (0.5 - c[0] / 390.0), 1)}
    res = {'sans_remorque': bilan(lignes),
           'avec_remorque_sur_attelage': bilan(lignes + [('attelage', M['remorque_sur_attelage_g'], [0.0, 540.0, 136.0])])}
    gros = {}
    for n, m, cg in lignes:
        k = n.rstrip('DG').rstrip('_')
        gros.setdefault(k, [0.0, 0.0, 0.0])
        gros[k][0] += m
        gros[k][1] += m * cg[0]
        gros[k][2] += m * cg[1]
    res['principales_masses'] = sorted(([k, round(v[0]), round(v[1] / v[0]), round(v[2] / v[0])] for k, v in gros.items() if v[0] > 40), key=lambda r: -r[1])
    res['corps_ignores'] = sorted(set(i.split('/')[0] for i in ignores))
    open(DOSSIER + '/centre_de_gravite.json', 'w', encoding='utf-8').write(texte(json.dumps(res, indent=1, ensure_ascii=False)))
    print(texte(json.dumps(res, ensure_ascii=False)))
