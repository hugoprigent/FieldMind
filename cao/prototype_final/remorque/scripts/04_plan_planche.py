"""Plan de découpe et de perçage de la planche de la remorque, à partir de remorque_bilan.json
(écrit par 01_remorque.py). Hors Fusion (Python 3 + rsvg-convert), mêmes outils que le plan du rover.

Produit dans ../plan_planche/ : plan_planche.pdf (page 1 plan au 1:2, page 2 coordonnées, pages 3
à 6 gabarit à l'échelle 1 sur 4 feuilles A4), plan_planche.png (aperçu de la page 1), SVG des pages.
Planche vue de dessus, avant (côté rover) en haut. X depuis le bord gauche, Y depuis le bord avant.
"""
import importlib.util, json, os, subprocess

ICI = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('plan_rover', os.path.join(ICI, '..', '..', 'scripts', '06_plan_percages.py'))
pr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pr)
txt, svg, croix, fr, A4, FONT = pr.txt, pr.svg, pr.croix, pr.fr, pr.A4, pr.FONT
SRC = os.path.join(ICI, '..', 'remorque_bilan.json')
OUT = os.path.join(ICI, '..', 'plan_planche')
GROUPES = [   # (code, rôles dans le bilan, couleur, libellé, vis, Ø de perçage)
    ('T', ('timon',), '#2471a3', 'Timon (attelage), M6', 'Vis M6×25 à tête bombée par le dessus, écrou M6 noyé sous le timon', 6.5),
    ('R', ('rebord',), '#16a085', 'Rebords avant et arrière', 'Vis M3×10 + rondelle par-dessous, dans l\'écrou noyé du rebord', 3.4),
    ('S', ('support de roue',), '#c0392b', 'Supports de roue, M6',
     'Vis M6×16 à tête bombée par le dessus (pas de fraisage), à travers la planche et le bras du support, écrou M6 noyé sous le bras', 6.5),
    ('F', ('boîtier du servo', 'bloc de charnière arrière'), '#8e44ad', 'Boîtiers des servos et blocs arrière, M3',
     'Vis M3×16 à tête bombée par le dessus (pas de fraisage), à travers la planche, la semelle de la joue et le bras, écrou noyé sous le bras', 3.4),
]


def charger():
    b = json.load(open(SRC, encoding='utf-8'))
    cont = b['planche']['contour']
    x0, y0 = min(p[0] for p in cont), min(p[1] for p in cont)
    W, H = max(p[0] for p in cont) - x0, max(p[1] for p in cont) - y0
    trous = []
    for code, role, coul, lib, vis, d in GROUPES:
        sel = sorted((v for v in b['vis_planche'] if v[4] in role), key=lambda v: (v[1], v[0]))
        for k, (x, y, L, reb, _) in enumerate(sel, 1):
            trous.append({'id': '%s%d' % (code, k), 'X': x - x0, 'Y': y - y0, 'xa': x, 'd': d, 'L': L,
                          'coul': coul, 'role': lib, 'rebord': reb})
    poly = [(x - x0, y - y0) for x, y in cont]
    return b, (W, H), poly, trous, (x0, y0)


def dessin(taille, poly, trous, s, ox, oy, etiquettes=True, fin=0.25, extras=None, croix_b=None):
    W, H = taille
    T = lambda X, Y: (ox + X * s, oy + Y * s)
    pts = ' '.join('%.2f,%.2f' % T(X, Y) for X, Y in poly)
    c = ['<polygon points="%s" fill="#f6efe0" stroke="#7a5c2e" stroke-width="%.2f"/>' % (pts, 2 * fin)]
    ax, _ = T(W / 2, 0)
    c.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#999" stroke-width="%.2f" stroke-dasharray="6 2 1 2"/>' % (ax, oy - 3, ax, oy + H * s + 3, fin))
    if croix_b is not None:
        c.append(croix_peinte(croix_b[0], croix_b[1], croix_b[2], T, fin))
    for (X, Y, w, h, coul, tiret) in (extras or []):
        x, y = T(X, Y)
        c.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="none" stroke="%s" stroke-width="%.2f" stroke-dasharray="%s"/>' % (x, y, w * s, h * s, coul, fin, tiret))
    for t in trous:
        x, y = T(t['X'], t['Y'])
        c.append(croix(x, y, t['d'] / 2 * s, t['coul'], fin * 1.4))
        if etiquettes:
            g = t['xa'] < 0
            c.append(txt(x + (-1 if g else 1) * (t['d'] / 2 * s + 1.2), y - 1.2, t['id'], 2.6 if s < 1 else 3.2, 'end' if g else 'start', 'bold', t['coul']))
    return c


def extras(b, x0, y0):
    """Empreintes sous la planche : timon, supports de roue, boîtiers des servos, blocs arrière."""
    yf = min(p[1] for p in b['planche']['contour'])
    y_axe = b['y_axe']
    out = [(-84 - x0, 0, 168, 50, '#888', '1 1.2')]
    for sx in (1, -1):
        xa, xb = sorted((sx * b.get('bras_x0', 100.0), sx * 152.5))
        out.append((xa - x0, y_axe - 60 - y0, xb - xa, 120, '#888', '1 1.2'))
    for x0_, y0_, x1_, y1_ in b.get('empreintes_dessous', []):      # boîtiers des servos, blocs arrière
        xa, xb = sorted((x0_, x1_))
        out.append((xa - x0, y0_ - y0, xb - xa, y1_ - y0_, '#888', '1 1.2'))
    return out


def croix_peinte(b, x0, y0, T, fin):
    yc = b['y_croix'] - y0
    xc = -x0
    pts = [(-18, -100), (18, -100), (18, -18), (100, -18), (100, 18), (18, 18), (18, 100), (-18, 100), (-18, 18), (-100, 18), (-100, -18), (-18, -18)]
    return '<polygon points="%s" fill="#1f4e9c" fill-opacity="0.12" stroke="#1f4e9c" stroke-width="%.2f" stroke-dasharray="2 1"/>' % (
        ' '.join('%.2f,%.2f' % T(xc + u, yc + v) for u, v in pts), fin)


def page_plan(b, taille, poly, trous, org):
    W, H = taille
    s = 0.5
    ox, oy = (A4[0] - W * s) / 2, 34
    ch = b['chanfrein']
    c = [txt(15, 14, 'FieldMind remorque — découpe et perçage de la planche %g × %g × %g mm' % (W, H, b['planche']['ep']), 4.6, poids='bold'),
         txt(15, 20, 'Vue de dessus, avant (côté rover) en haut, échelle 1:2. Coordonnées en page 2 ; gabarit à l\'échelle 1 en pages 3 à 6.', 2.9),
         txt(A4[0] / 2, oy - 6, 'AVANT (attelage)', 3.5, 'middle', 'bold')]
    c += dessin(taille, poly, trous, s, ox, oy, extras=extras(b, *org), croix_b=(b,) + tuple(org))
    c.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#444" stroke-width="0.2"/>' % (ox, oy + H * s + 6, ox + W * s, oy + H * s + 6))
    c.append(txt(ox + W * s / 2, oy + H * s + 10, '%g mm' % W, 3, 'middle'))
    c.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#444" stroke-width="0.2"/>' % (ox - 6, oy, ox - 6, oy + H * s))
    c.append('<text x="%.2f" y="%.2f" font-size="3" text-anchor="middle" transform="rotate(-90 %.2f %.2f)" %s>%g mm</text>' % (ox - 8, oy + H * s / 2, ox - 8, oy + H * s / 2, FONT, H))
    c.append(txt(ox + 4, oy + ch * s + 7, 'coins coupés à 45° : %g × %g' % (ch, ch), 2.6, coul='#7a5c2e'))
    ya = (b['y_axe'] - org[1]) * s + oy
    c.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#8e44ad" stroke-width="0.25" stroke-dasharray="4 1.5"/>' % (ox - 4, ya, ox + W * s + 4, ya))
    c.append(txt(ox + W * s + 5, ya + 1, 'axe des roues', 2.6, coul='#8e44ad'))
    yc = (b['y_croix'] - org[1]) * s + oy
    c.append(txt(ox + W * s / 2 + 22, yc - 22, 'croix peinte (200 × 200, bras de 36)', 2.6, coul='#1f4e9c'))
    y = oy + H * s + 18
    for code, _, coul, lib, vis, d in GROUPES:
        n = sum(1 for t in trous if t['id'].startswith(code))
        c.append(croix(20, y - 1, 1.6, coul, 0.35))
        c.append(txt(25, y, '%s : Ø%s — %s (%d trous)' % (code, fr(d), lib, n), 3, poids='bold', coul=coul))
        y += 5
    c.append('<rect x="17" y="%.2f" width="6" height="3" fill="none" stroke="#888" stroke-width="0.3" stroke-dasharray="1 1.2"/>' % (y - 3))
    c.append(txt(25, y, 'Pointillés fins : timon, supports de roue, boîtiers des servos et blocs arrière, sous la planche', 2.8))
    y += 5
    c.append('<rect x="17" y="%.2f" width="6" height="3" fill="none" stroke="#1f4e9c" stroke-width="0.3" stroke-dasharray="2 1"/>' % (y - 3))
    c.append(txt(25, y, 'Croix d\'atterrissage à peindre (dessus)', 3))
    return svg(c)


def page_tableau(b, taille, trous):
    c = [txt(15, 16, 'Coordonnées des trous (mm) — planche vue de dessus, avant en haut', 4.5, poids='bold'),
         txt(15, 22, 'X depuis le bord GAUCHE ; Y depuis le bord AVANT ; X axe = depuis l\'axe de la planche (+ à droite).', 3)]
    cols = [(15, 'Trou'), (32, 'Ø'), (46, 'X gauche'), (68, 'Y avant'), (90, 'X axe'), (110, 'Vis'), (130, 'Rôle')]
    y = 32
    for x, h in cols:
        c.append(txt(x, y, h, 3.2, poids='bold'))
    c.append('<line x1="15" y1="%.1f" x2="195" y2="%.1f" stroke="#444" stroke-width="0.3"/>' % (y + 1.5, y + 1.5))
    y += 7
    groupe = None
    for t in trous:
        if t['id'][0] != groupe:
            if groupe:
                y += 2
            groupe = t['id'][0]
        vals = [t['id'], fr(t['d']), fr(t['X']), fr(t['Y']), fr(t['xa'], '%+.1f'), '%s×%d' % ('M6' if t['d'] > 5 else 'M3', t['L']), t['role']]
        for (x, _), v in zip(cols, vals):
            c.append(txt(x, y, v, 3.2, poids='bold' if v == t['id'] else 'normal', coul=t['coul'] if v == t['id'] else '#222'))
        y += 5.5
    y += 6
    c.append(txt(15, y, 'Visserie', 3.8, poids='bold'))
    y += 6
    for code, _, coul, lib, vis, d in GROUPES:
        c.append(txt(15, y, '%s — %s' % (code, vis), 2.9, coul=coul))
        y += 5
    y += 4
    for n in ['Découpe : planche %g × %g, puis les deux coins AVANT coupés à 45° sur %g mm (mesurés le long de chaque bord).' % (taille[0], taille[1], b['chanfrein']),
              'Trous des bords à 9,5 mm du bois (S : 12,5 mm). Aucun fraisage : les vis S et F ont une tête bombée posée sur le bois.',
              'Contrôle : les trous vont par paires symétriques par rapport à l\'axe (même Y, X axe opposés).',
              'Perçage : avant-trou Ø2 partout, planche tenue sur une cale martyre, puis Ø3,4 (Ø6,5 pour les trous S).']:
        c.append(txt(15, y, n, 2.9))
        y += 5
    return svg(c)


def pages_gabarit(b, taille, poly, trous, org):
    W, H = taille
    marge, uw, uh = 10.0, 190.0, 277.0
    pages = []
    for j, Y0 in enumerate([0.0, H - uh]):
        for i, X0 in enumerate([0.0, W - uw]):
            nom = '%s%d' % ('AB'[i], j + 1)
            c = ['<defs><clipPath id="z"><rect x="%g" y="%g" width="%g" height="%g"/></clipPath></defs>' % (marge, marge, uw, uh), '<g clip-path="url(#z)">']
            c += dessin(taille, poly, trous, 1.0, marge - X0, marge - Y0, fin=0.2, extras=extras(b, *org), croix_b=(b,) + tuple(org))
            for gx in range(0, int(W) + 1, 50):
                for gy in range(0, int(H) + 1, 50):
                    x, y = marge - X0 + gx, marge - Y0 + gy
                    c.append('<line x1="%g" y1="%g" x2="%g" y2="%g" stroke="#27ae60" stroke-width="0.2"/>'
                             '<line x1="%g" y1="%g" x2="%g" y2="%g" stroke="#27ae60" stroke-width="0.2"/>' % (x - 3, y, x + 3, y, x, y - 3, x, y + 3))
                    c.append(txt(x + 1, y - 1, '%d,%d' % (gx, gy), 1.8, coul='#27ae60'))
            c.append('</g>')
            c.append('<rect x="%g" y="%g" width="%g" height="%g" fill="none" stroke="#bbb" stroke-width="0.2"/>' % (marge, marge, uw, uh))
            c.append(txt(marge + uw - 1, marge + 5, 'Feuille %s — échelle 1, imprimer à 100 %%' % nom, 3.2, 'end', 'bold'))
            c.append(txt(marge + uw - 1, marge + 9, 'Planche X %g→%g, Y %g→%g (mm)' % (X0, X0 + uw, Y0, Y0 + uh), 2.6, 'end'))
            rx, ry = marge + uw - 105, marge + uh - 6
            c.append('<line x1="%g" y1="%g" x2="%g" y2="%g" stroke="#000" stroke-width="0.4"/>' % (rx, ry, rx + 100, ry))
            for k in range(0, 101, 10):
                c.append('<line x1="%g" y1="%g" x2="%g" y2="%g" stroke="#000" stroke-width="0.3"/>' % (rx + k, ry, rx + k, ry - (3 if k % 50 == 0 else 1.5)))
            c.append(txt(rx + 50, ry + 4, 'contrôle : 100 mm', 2.6, 'middle'))
            pages.append((nom, svg(c)))
    return pages


def main():
    b, taille, poly, trous, org = charger()
    os.makedirs(OUT, exist_ok=True)
    fichiers = []
    for nom, contenu in [('1_plan', page_plan(b, taille, poly, trous, org)), ('2_coordonnees', page_tableau(b, taille, trous))] + \
                        [('3_gabarit_' + n, s) for n, s in pages_gabarit(b, taille, poly, trous, org)]:
        f = os.path.join(OUT, 'page_%s.svg' % nom)
        open(f, 'w', encoding='utf-8').write(contenu)
        fichiers.append(f)
    subprocess.run(['rsvg-convert', '-f', 'pdf', '-o', os.path.join(OUT, 'plan_planche.pdf')] + fichiers, check=True)
    subprocess.run(['rsvg-convert', '-f', 'png', '-d', '160', '-p', '160', '-o', os.path.join(OUT, 'plan_planche.png'), fichiers[0]], check=True)
    for t in trous:
        print(t['id'], round(t['X'], 1), round(t['Y'], 1), t['xa'], ('M6x%d' if t['d'] > 6 else 'M3x%d') % t['L'])


if __name__ == '__main__':
    main()
