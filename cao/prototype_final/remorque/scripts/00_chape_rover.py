from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Chape d'attelage de la remorque, côté rover : à exécuter dans le document du rover
(« prtotype final fieldmind », groupe FM_Prototype_Final). Réexécutable.

Profil en C vu de côté, largeur constante (imprimée couchée sur le flanc, sans support) : semelle
sous la planche du rover (2 vis M6, écrous dans des fentes), montant incliné, mâchoires haute et
basse. La tête du timon de la remorque se loge entre les deux. Une vis H M10×55 traverse le tout,
mise par le dessus, rondelle et écrou frein M10 dessous : un attelage classique, solide. Encoche du
câble des volets sous le trou Ø8 de la planche. Exporte aussi le STL de la chape.
"""
import adsk.core, adsk.fusion, json, math, os
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/remorque/scripts/rm_helpers.py'), encoding='utf-8').read(), 'rm_helpers.py', 'exec'))
DOC_PREFIXE = DOC_ROVER


def chape_rover():
    """Chape d'attelage sous l'arrière du rover (groupe FM_Prototype_Final), vis M10 et écrou (références).

    Profil en C vu de côté, largeur constante : elle s'imprime couchée sur le flanc, sans support.
    Semelle sous la planche du rover (2 vis M6, écrous dans des fentes), montant incliné, mâchoires
    haute et basse ; la tête du timon de la remorque se loge entre les deux. Vis H M10 verticale par
    le dessus, rondelle et écrou frein M10 sous la mâchoire basse.
    """
    nettoyer(['PF_IMP_Attelage'])
    _, c = piece('PF_IMP_Attelage', 'IMPRIME_PETG')
    p, ye = PIVOT_Y, PIVOT_Y + 12
    z0, zi, zs, z1, zb = CHAPE_Z0, MACH_INF, MACH_SUP, CHAPE_Z1, ZB_ROVER - 5
    profil = [(405, z0 + 3), (408, z0), (ye - 3, z0), (ye, z0 + 3), (ye, zi), (512, zi), (512, zs), (ye, zs), (ye, z1 - 3),
              (ye - 3, z1), (503, z1), (503 - (z1 - zi), zi), (440, zi), (440, zb), (408, zb), (405, zb - 3)]
    b = body(prisme_x(c, 'chape', 0, profil, CHAPE_L), 'chape_attelage', BLANC)
    # bouts des mâchoires coupés à 45° en plan
    for sx in (1, -1):
        zpoly(c, 'chape_pan_%+d' % sx, [(sx * 8, ye + 1), (sx * 8, ye), (sx * (CHAPE_L / 2 + 1), p - 3), (sx * (CHAPE_L / 2 + 1), ye + 1)],
              z0 - 1, z1 - z0 + 2, CUT, [b])
    cyl_z(c, 'chape_bague', 0, p, BAGUE_D, zi, BAGUE_H, op=JOIN, bodies=[b])
    trou_z(c, 'chape_axe', 0, p, M10_TROU, z0 - 1, z1 - z0 + 2, [b])
    for k, (x, y) in enumerate(PATTE_VIS):
        trou_z(c, 'chape_vis_%d' % k, x, y, 6.5, z0 - 1, zb - z0 + 2, [b])
        hexa_z(c, 'chape_ecrou_%d' % k, x, y, M6_HEX, z0 - 0.01, M6_ECROU_H, [b])
    r = arrondir(c, 'chape_conges', b, 'x', 10, (2.5, 1.5))
    xc, yc, dc = TROU_CABLE_ROVER                    # câble vers la coque : encoche sur la semelle, sous le trou de la planche
    boite(c, 'chape_cable', xc - 2.5, yc - 4, xc + 2.5, 441, zb - 3, 3.01, op=CUT, bodies=[b])
    # vis H M10×55 par le dessus, rondelle sous la tête ; rondelle et écrou frein M10 sous la mâchoire basse
    rd, re_ = M10_RONDELLE
    r_h = cyl_z(c, 'REF_rondelle_M10_haut_attelage', 0, p, rd, z1, re_, METAL)
    trou_z(c, 'REF_rondelle_M10_haut_trou', 0, p, M10_TROU, z1 - 1, re_ + 2, [r_h])
    r_b = cyl_z(c, 'REF_rondelle_M10_bas_attelage', 0, p, rd, z0 - re_, re_, METAL)
    trou_z(c, 'REF_rondelle_M10_bas_trou', 0, p, M10_TROU, z0 - re_ - 1, re_ + 2, [r_b])
    zt = z1 + re_                                     # dessous de la tête
    v = body(zpoly(c, 'REF_vis_H_M10x55_attelage', hexagon(None, 0, p, M10_SURPLATS), zt, M10_TETE_H), 'REF_vis_H_M10x55_attelage', METAL)
    cyl_z(c, 'REF_vis_M10_tige', 0, p, 10.0, zt - VIS_ATTELAGE_L, VIS_ATTELAGE_L + 0.01, op=JOIN, bodies=[v])
    e = body(zpoly(c, 'REF_ecrou_frein_M10_attelage', hexagon(None, 0, p, M10_SURPLATS), z0 - re_ - M10_ECROU_FREIN_H, M10_ECROU_FREIN_H),
             'REF_ecrou_frein_M10_attelage', METAL)
    trou_z(c, 'REF_ecrou_M10_taraudage', 0, p, 10.0, z0 - re_ - M10_ECROU_FREIN_H - 1, M10_ECROU_FREIN_H + 2, [e])
    return b, r


def run(_context: str):
    b, conges = chape_rover()
    c = b.parentComponent
    os.makedirs(DOSSIER + '/exports/stl', exist_ok=True)
    out = {}
    for corps, nom, rot in ((b, 'chape_attelage_rover', ('y', 90)),):
        out[nom] = {'taille_impression_mm': stl(corps, '%s/exports/stl/%s.stl' % (DOSSIER, nom), rot),
                    'volume_cm3': round(corps.volume, 1), 'morceaux': corps.lumps.count}
    out['conges'] = conges
    json.dump(out, open(DOSSIER + '/chape_bilan.json', 'w', encoding='utf-8'), indent=1, ensure_ascii=False)
    print(json.dumps(out, ensure_ascii=False))
