"""Hors Fusion (Python 3 + rsvg-convert) : pièces et schéma annexes des roues avant.

- exports/stl/entretoise_roue_2.2mm.stl : entretoise Ø12 × Ø8,4 × 2,2 entre les bagues intérieures
  des deux 608ZZ du moyeu (0,2 mm de plus que l'arête de la jante : pas de précontrainte possible).
- vues/19_coupe_moyeu_roue.png : demi-coupe du moyeu, pièces fixes et pièces qui tournent.
"""
import math, os, struct, subprocess

ICI = os.path.dirname(os.path.abspath(__file__))
STL = os.path.join(ICI, '..', 'exports', 'stl')
VUES = os.path.join(ICI, '..', 'vues')


def anneau_stl(chemin, d_ext, d_int, ep, n=96):
    """Anneau fermé (STL binaire, mm), à plat sur Z = 0."""
    re_, ri = d_ext / 2, d_int / 2
    pts = [(math.cos(2 * math.pi * k / n), math.sin(2 * math.pi * k / n)) for k in range(n)]
    tris = []
    for k in range(n):
        (c0, s0), (c1, s1) = pts[k], pts[(k + 1) % n]
        e0, e1, i0, i1 = (re_ * c0, re_ * s0), (re_ * c1, re_ * s1), (ri * c0, ri * s0), (ri * c1, ri * s1)
        for z, sens in ((0.0, -1), (ep, 1)):   # faces du bas et du haut
            a, b, c, d = (*e0, z), (*e1, z), (*i1, z), (*i0, z)
            tris += [(a, b, c), (a, c, d)] if sens > 0 else [(a, c, b), (a, d, c)]
        tris += [((*e0, 0), (*e1, 0), (*e1, ep)), ((*e0, 0), (*e1, ep), (*e0, ep))]   # paroi extérieure
        tris += [((*i0, 0), (*i1, ep), (*i1, 0)), ((*i0, 0), (*i0, ep), (*i1, ep))]   # paroi intérieure
    with open(chemin, 'wb') as f:
        f.write(b'FieldMind entretoise roue'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for p in tris:
            u = [p[1][i] - p[0][i] for i in range(3)]; v = [p[2][i] - p[0][i] for i in range(3)]
            nn = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            l = math.sqrt(sum(x * x for x in nn)) or 1
            f.write(struct.pack('<12fH', *(x / l for x in nn), *p[0], *p[1], *p[2], 0))


# ---- demi-coupe du moyeu (x : de la jambe de fusée vers l'extérieur ; r : depuis l'axe)
K, OX, OY = 15.0, 260.0, 420.0         # px par mm, origine (x = 0, r = 0) sur la page
FIXE, TOURNE, METAL, IMPR = '#5d7fa3', '#e07b39', '#b7bcc4', '#7f9bb8'
E_EP = 2.2                              # entretoise


def P_(x, r):
    return OX + x * K, OY - r * K


def rect(x0, x1, r0, r1, coul, trait='#333', op=1.0):
    a, b = P_(x0, r1)
    return '<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="%s" fill-opacity="%s" stroke="%s" stroke-width="1"/>' % (a, b, (x1 - x0) * K, (r1 - r0) * K, coul, op, trait)


def poly(pts, coul, op=1.0):
    return '<polygon points="%s" fill="%s" fill-opacity="%s" stroke="#333" stroke-width="1"/>' % (' '.join('%.1f,%.1f' % P_(x, r) for x, r in pts), coul, op)


def txt(x, y, s, t=15, ancre='start', poids='normal', coul='#222'):
    return '<text x="%.1f" y="%.1f" font-size="%s" text-anchor="%s" font-weight="%s" fill="%s" font-family="Helvetica, Arial, sans-serif">%s</text>' % (x, y, t, ancre, poids, coul, s)


def etiquette(x, r, tx, ty, s, coul='#222'):
    a, b = P_(x, r)
    return ('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#555" stroke-width="0.8"/><circle cx="%.1f" cy="%.1f" r="2.5" fill="#555"/>' % (a, b, tx, ty - 4, a, b)
            + txt(tx, ty, s, 14, 'middle', coul=coul))


def roulement(x0):
    c = [rect(x0, x0 + 7, 4.0, 6.05, METAL), rect(x0, x0 + 7, 9.6, 11.0, METAL)]
    cx, cr = P_(x0 + 3.5, 7.8)
    c.append('<circle cx="%.1f" cy="%.1f" r="%.1f" fill="#d8dbe0" stroke="#333"/>' % (cx, cr, 1.98 * K))
    for xs in (x0 + 0.35, x0 + 6.65):   # flasques ZZ, en retrait des faces
        a, b = P_(xs, 9.6)
        _, d = P_(xs, 6.4)
        c.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#333" stroke-width="1.5"/>' % (a, b, a, d))
    return c


def coupe():
    c = []
    # jambe de fusée (fixe) avec portée Ø12 et écrou M8 prisonnier
    c.append(poly([(-11, 0), (0, 0), (0, 4.2), (1.5, 4.2), (1.5, 6), (0, 6), (0, 18), (-11, 18)], FIXE, 0.85))
    c.append(rect(-9, -2.5, 4.0, 7.5, METAL))
    # vis M8×30 (axe fixe) : tige et tête
    c.append(rect(-10.5, 17.5 + 2 + 0.2, 0, 4.0, METAL))
    c.append(rect(19.7, 25.0, 0, 7.2, METAL))
    # roulements, entretoise et rondelle sous tête (fixes avec les bagues intérieures)
    c += roulement(1.5)
    c.append(rect(8.5, 8.5 + E_EP, 4.2, 6.0, IMPR))
    c += roulement(8.5 + E_EP)
    c.append(rect(15.5 + E_EP, 17.5 + E_EP, 4.2, 6.0, IMPR))
    # jante (tourne) : sièges Ø22,15, arête Ø19,5 entre les bagues extérieures, alésage Ø24 de la douille
    c.append(poly([(1.5, 11.07), (8.5, 11.07), (8.5, 9.75), (10.5, 9.75), (10.5, 11.07), (17.5, 11.07), (17.5, 12.0),
                   (34.5, 12.0), (34.5, 18), (1.5, 18)], TOURNE, 0.9))
    a, b = P_(20, 18)
    c.append(txt(a, b - 8, '↑ moyeu, rayons, pneu et flasque', 13, 'middle', coul='#8a4a1f'))
    # axe
    a, b = P_(-14, 0)
    d, _ = P_(40, 0)
    c.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#333" stroke-width="1" stroke-dasharray="14 4 2 4"/>' % (a, b, d, b))
    # étiquettes
    c.append(etiquette(-6, 14, 150, 75, 'jambe de fusée (fixe)'))
    c.append(etiquette(-5.7, 6.5, 110, 505, 'écrou M8 prisonnier'))
    c.append(etiquette(0.8, 5.1, 245, 545, 'portée Ø12 : bague intérieure seule'))
    c.append(etiquette(5, 5, 400, 505, 'bague intérieure (serrée, fixe)'))
    c.append(etiquette(9.6, 5.1, 545, 545, 'entretoise 2,2 mm (fixe)'))
    c.append(etiquette(18.7, 5.0, 660, 505, 'rondelle sous tête'))
    c.append(etiquette(23, 3, 820, 545, 'vis H M8×30 = axe fixe'))
    c.append(etiquette(5, 10.3, 330, 110, 'bague extérieure (emmanchée, tourne)'))
    c.append(etiquette(9.5, 10.4, 600, 75, 'ARÊTE de la jante, entre les deux bagues extérieures', '#8a4a1f'))
    c.append(etiquette(28, 15, 900, 120, 'jante (tourne)', '#8a4a1f'))
    # légende et principe
    c.append(txt(40, 36, 'Moyeu de roue avant : demi-coupe au-dessus de l\'axe (échelle ×15)', 22, poids='bold'))
    for k, (coul, s) in enumerate(((FIXE, 'fixe (fusée)'), (METAL, 'acier'), (IMPR, 'rondelles imprimées, fixes'), (TOURNE, 'tourne avec la roue'))):
        c.append('<rect x="%d" y="595" width="18" height="12" fill="%s" stroke="#333"/>' % (40 + 230 * k, coul))
        c.append(txt(64 + 230 * k, 606, s, 14))
    lignes = [
        'Serrage de la vis : tête → rondelle → bague int. → entretoise → bague int. → portée de la fusée → écrou. Rien d\'autre n\'est serré.',
        'La jante tire vers l\'extérieur : son arête pousse la bague extérieure du roulement extérieur, retenu par la tête de vis.',
        'La jante pousse vers la fusée : son arête pousse la bague extérieure du roulement intérieur, retenu par la portée.',
        'La jante ne peut donc pas sortir, même avec des roulements montés libres. L\'emmanchement sert seulement à les empêcher de tourner dans la jante.',
    ]
    for k, s in enumerate(lignes):
        c.append(txt(40, 645 + 22 * k, s, 14))
    return '<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="740" viewBox="0 0 1100 740"><rect width="1100" height="740" fill="white"/>%s</svg>' % ''.join(c)


def main():
    anneau_stl(os.path.join(STL, 'entretoise_roue_2.2mm.stl'), 12.0, 8.4, E_EP)
    f = os.path.join(VUES, '19_coupe_moyeu_roue.svg')
    open(f, 'w', encoding='utf-8').write(coupe())
    subprocess.run(['rsvg-convert', '-f', 'png', '-o', f[:-4] + '.png', f], check=True)
    os.remove(f)
    print('ok')


if __name__ == '__main__':
    main()
