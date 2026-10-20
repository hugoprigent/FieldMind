"""Plan de perçage de la planche du prototype final, à partir de plan_percages.json (écrit par
05_percages.py) : trous de la direction V04 et des capteurs, plus ceux de la coque et du berceau.

Hors Fusion (Python 3 + rsvg-convert). Produit dans ../plan_percages/ :
- plan_percages.pdf : page 1 plan coté au 1:2,5, page 2 tableau des trous, pages 3 à 6 gabarit à
  l'échelle 1 sur 4 feuilles A4 (imprimer à 100 %, contrôler la règle de 100 mm, scotcher les
  feuilles sur les croix de repère, pointer au centre des croix) ;
- plan_percages.png : aperçu de la page 1 ; les SVG des pages.
Planche vue de dessus, avant en haut. X depuis le bord gauche, Y depuis le bord avant.
"""
import json, math, os, subprocess

ICI = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(ICI, '..', 'plan_percages.json')
OUT = os.path.join(ICI, '..', 'plan_percages')
A4 = (210.0, 297.0)
FONT = 'font-family="Helvetica, Arial, sans-serif"'
GROUPES = [   # (code, test, couleur, rôle, vis)
    ('P', lambda t: abs(t['d'] - 8.5) < 0.1 and abs(t['x']) > 60 and t['y'] < 100, '#c0392b', 'Supports de pivot', 'Vis H M8×25 par-dessous (tête prisonnière dans le support), rondelle + écrou M8 dessus'),
    ('M', lambda t: abs(t['d'] - 8.5) < 0.1 and abs(t['x']) > 60 and t['y'] >= 300, '#8e44ad', 'Supports moteurs arrière (4 par support)', 'Vis H M8×30 par-dessous (tête sous la semelle), rondelle + écrou sur la planche'),
    ('S', lambda t: abs(t['d'] - 3.4) < 0.1, '#d35400', 'Oreilles du servo (à pointer avec le servo)', 'Vis M3×16 + écrous ; entraxe 48 × 10 supposé, à vérifier sur le DT996'),
    ('E', lambda t: abs(t['d'] - 8.5) < 0.1 and abs(t['x']) < 60, '#2471a3', 'Équerres des profilés (caméras à l\'avant, lidar à l\'arrière)', 'Vis H M8×25 + rondelles ; écrou M8 sous la planche (trou à 20 mm de la face du profilé)'),
    ('K', lambda t: abs(t['d'] - 6.5) < 0.1 and abs(t['x']) >= 50, '#16a085', 'Fixation de la coque (M6)', 'Vis M6×20 + rondelle par-dessous la planche, dans l\'écrou M6 glissé dans la fente de la coque'),
    ('B', lambda t: abs(t['d'] - 3.5) < 0.1 and t['x'] > 0, '#7f8c8d', 'Berceau de batterie (avant gauche)', 'Vis M3×16 + rondelle par-dessous la planche, dans l\'écrou M3 noyé dans le fond du berceau'),
    ('X', lambda t: abs(t['d'] - 3.5) < 0.1 and t['x'] < 0, '#5d6d7e', 'Plaque des deux XL4015 (avant droit)', 'Vis M3×16 + rondelle, écrou sous la planche (ou double-face)'),
    ('C', lambda t: abs(t['d'] - 8.0) < 0.1, '#e67e22', 'Passage des câbles des volets de la remorque', 'Ébavurer ; le câble descend dans l\'encoche de la chape'),
    ('A', lambda t: abs(t['d'] - 6.5) < 0.1 and abs(t['x']) < 50, '#b7950b', 'Chape d\'attelage de la remorque', 'Vis M6×25 + rondelle par le dessus (sous la coque), écrou M6 dans la fente de la chape'),
]


def charger():
    p = json.load(open(SRC, encoding='utf-8'))
    pl = p['planche']
    x0, y0 = pl['x0'], pl['y0']
    trous = []
    for code, test, coul, role, vis in GROUPES:
        sel = sorted((t for t in p['trous'] if test(t)), key=lambda t: (t['y'], t['x']))
        for k, t in enumerate(sel, 1):
            trous.append({'id': '%s%d' % (code, k), 'X': t['x'] - x0, 'Y': t['y'] - y0, 'xa': t['x'], 'd': t['d'],
                          'coul': coul, 'role': role, 'vis': vis})
    ignores = [t for t in p['trous'] if not any(g[1](t) for g in GROUPES)]
    rect = lambda r: (r['x0'] - x0, r['y0'] - y0, r['x1'] - r['x0'], r['y1'] - r['y0'])
    return p, (pl['x1'] - pl['x0'], pl['y1'] - pl['y0']), trous, ignores, rect


def fr(v, f='%.1f'):
    return (f % v).replace('.', ',')


def svg(contenu, w=A4[0], h=A4[1]):
    return ('<svg xmlns="http://www.w3.org/2000/svg" width="%smm" height="%smm" viewBox="0 0 %s %s">'
            '<rect width="%s" height="%s" fill="white"/>%s</svg>' % (w, h, w, h, w, h, ''.join(contenu)))


def txt(x, y, s, taille=3.0, ancre='start', poids='normal', coul='#222'):
    s = s.replace('&', '&amp;').replace('<', '&lt;')
    return '<text x="%.2f" y="%.2f" font-size="%.2f" text-anchor="%s" font-weight="%s" fill="%s" %s>%s</text>' % (x, y, taille, ancre, poids, coul, FONT, s)


def croix(x, y, r, coul, ep=0.25):
    return ('<circle cx="%.3f" cy="%.3f" r="%.3f" fill="none" stroke="%s" stroke-width="%s"/>'
            '<line x1="%.3f" y1="%.3f" x2="%.3f" y2="%.3f" stroke="%s" stroke-width="%s"/>'
            '<line x1="%.3f" y1="%.3f" x2="%.3f" y2="%.3f" stroke="%s" stroke-width="%s"/>'
            % (x, y, r, coul, ep, x - r - 2, y, x + r + 2, y, coul, ep, x, y - r - 2, x, y + r + 2, coul, ep))


def dessin(p, taille, trous, rect, s, ox, oy, etiquettes=True, fin=0.25):
    """Planche à l'échelle s, coin avant gauche en (ox, oy) sur la page."""
    W, H = taille
    T = lambda X, Y: (ox + X * s, oy + Y * s)
    c = ['<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="#f6efe0" stroke="#7a5c2e" stroke-width="%.2f"/>' % (ox, oy, W * s, H * s, 2 * fin)]
    ax, _ = T(W / 2, 0)
    c.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#999" stroke-width="%.2f" stroke-dasharray="6 2 1 2"/>' % (ax, oy - 3, ax, oy + H * s + 3, fin))
    for nom, r in p['empreintes'].items():
        X, Y, w, h = rect(r)
        dessous = nom.startswith('dessous_')
        X0, Y0 = max(X, 0), max(Y, 0)
        w, h = min(X + w, W) - X0, min(Y + h, H) - Y0
        x, y = T(X0, Y0)
        c.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="none" stroke="%s" stroke-width="%.2f" stroke-dasharray="%s"/>'
                 % (x, y, w * s, h * s, '#888' if dessous else '#2c3e50', fin, '1 1.2' if dessous else '3 1.5'))
    X, Y, w, h = rect(p['decoupe_servo'])
    x, y = T(X, Y)
    c.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="#fff" stroke="#d35400" stroke-width="%.2f"/>' % (x, y, w * s, h * s, 2 * fin))
    for t in trous:
        x, y = T(t['X'], t['Y'])
        c.append(croix(x, y, t['d'] / 2 * s, t['coul'], fin * 1.4))
        if etiquettes:   # étiquette du côté extérieur
            g = t['xa'] < 0
            c.append(txt(x + (-1 if g else 1) * (t['d'] / 2 * s + 1.2), y - 1.2, t['id'], 2.6 if s < 1 else 3.2,
                         'end' if g else 'start', 'bold', t['coul']))
    return c


def page_plan(p, taille, trous, ignores, rect):
    W, H = taille
    s = 0.4
    ox, oy = (A4[0] - W * s) / 2, 32
    c = [txt(15, 14, 'FieldMind prototype final — perçage de la planche %g × %g × %g mm' % (W, H, p['planche']['ep']), 5, poids='bold'),
         txt(15, 20, 'Vue de dessus, avant en haut, échelle 1:2,5. Coordonnées en page 2 ; gabarit à l\'échelle 1 en pages 3 à 6.', 3),
         txt(A4[0] / 2, oy - 6, 'AVANT', 3.5, 'middle', 'bold')]
    c += dessin(p, taille, trous, rect, s, ox, oy)
    # cotes hors tout
    c.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#444" stroke-width="0.2"/>' % (ox, oy + H * s + 6, ox + W * s, oy + H * s + 6))
    c.append(txt(ox + W * s / 2, oy + H * s + 10, '%g mm' % W, 3, 'middle'))
    c.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#444" stroke-width="0.2"/>' % (ox - 6, oy, ox - 6, oy + H * s))
    c.append('<text x="%.2f" y="%.2f" font-size="3" text-anchor="middle" transform="rotate(-90 %.2f %.2f)" %s>%g mm</text>' % (ox - 8, oy + H * s / 2, ox - 8, oy + H * s / 2, FONT, H))
    # repères des empreintes
    e = p['empreintes']
    for cle, texte in (('profile_lidar', 'Profilé 40×40 du lidar'), ('profile_cam', 'Profilé 40×40 des caméras')):
        pr = rect(e[cle])
        y_txt = oy + pr[1] * s - 3 if cle == 'profile_lidar' else oy + (pr[1] + pr[3]) * s + 9   # au-dessus : la patte d'attelage est dessous
        c.append(txt(ox + (pr[0] + pr[2] / 2) * s, y_txt, texte, 2.4, 'middle', coul='#2c3e50'))
    sv = rect(p['decoupe_servo'])
    c.append(txt(ox + (sv[0] + sv[2]) * s + 6, oy + (sv[1] + sv[3] / 2) * s + 1, 'Découpe servo %s × %s' % (fr(sv[2], '%g'), fr(round(sv[3], 1), '%g')), 2.4, coul='#d35400'))
    # légende
    y = oy + H * s + 18
    for code, _, coul, role, vis in GROUPES:
        d = next(t['d'] for t in trous if t['id'].startswith(code))
        c.append(croix(20, y - 1, 1.6, coul, 0.35))
        c.append(txt(25, y, '%s : Ø%s — %s' % (code, fr(d, '%g'), role), 3, poids='bold', coul=coul))
        y += 5
    c.append('<rect x="17" y="%.2f" width="6" height="3" fill="none" stroke="#2c3e50" stroke-width="0.3" stroke-dasharray="3 1.5"/>' % (y - 3))
    c.append(txt(25, y, 'Pointillés : profilés et contour extérieur de la coque', 3))
    y += 5
    c.append('<rect x="17" y="%.2f" width="6" height="3" fill="none" stroke="#888" stroke-width="0.3" stroke-dasharray="1 1.2"/>' % (y - 3))
    c.append(txt(25, y, 'Pointillés fins : pièces vissées sous la planche (supports de pivot et moteurs, chape d\'attelage)', 3))
    if ignores:
        y += 5
        c.append(txt(15, y, 'Ne pas percer : le Ø%g du modèle au centre de la découpe servo (ancien palonnier rond, inutile).' % ignores[0]['d'], 2.8, coul='#666'))
    return svg(c)


def page_tableau(p, taille, trous):
    c = [txt(15, 16, 'Coordonnées des trous (mm) — planche vue de dessus, avant en haut', 4.5, poids='bold'),
         txt(15, 22, 'X depuis le bord GAUCHE ; Y depuis le bord AVANT ; X axe = depuis l\'axe de la planche (+ à droite).', 3)]
    cols = [(15, 'Trou'), (32, 'Ø'), (46, 'X gauche'), (68, 'Y avant'), (90, 'X axe'), (110, 'Rôle')]
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
        vals = [t['id'], fr(t['d'], '%g'), fr(t['X']), fr(t['Y']), fr(t['xa'], '%+.1f'), t['role']]
        for (x, _), v in zip(cols, vals):
            c.append(txt(x, y, v, 3.2, poids='bold' if v == t['id'] else 'normal', coul=t['coul'] if v == t['id'] else '#222'))
        y += 5.5
    y += 6
    c.append(txt(15, y, 'Visserie', 3.8, poids='bold'))
    y += 6
    for code, _, coul, role, vis in GROUPES:
        c.append(txt(15, y, '%s — %s' % (code, vis), 3, coul=coul))
        y += 5
    y += 4
    e = p['empreintes']
    pr, pc, sv = e['profile_lidar'], e['profile_cam'], p['decoupe_servo']
    pl = p['planche']
    notes = [
        'Profilés 40×40 centrés sur l\'axe : caméras de Y %s à %s, lidar de Y %s à %s (axe des roues arrière).' % (
            fr(pc['y0'] - pl['y0'], '%g'), fr(pc['y1'] - pl['y0'], '%g'), fr(pr['y0'] - pl['y0'], '%g'), fr(pr['y1'] - pl['y0'], '%g')),
        'Chaque profilé : une équerre à gauche et une à droite, trous E à 20 mm de la face du profilé (équerre 40 du commerce).',
        'Découpe servo : rectangle de X %s à %s et de Y %s à %s (21 × 41,4).' % (fr(sv['x0'] - pl['x0'], '%g'), fr(sv['x1'] - pl['x0'], '%g'), fr(sv['y0'] - pl['y0']), fr(sv['y1'] - pl['y0'])),
        'Contrôle : les trous vont par paires symétriques par rapport à l\'axe (même Y, X axe opposés).',
        'Perçage : avant-trou Ø3 partout, planche tenue sur une cale martyre, puis diamètre final.',
    ]
    for n in notes:
        c.append(txt(15, y, n, 3))
        y += 5
    return svg(c)


def pages_gabarit(p, taille, trous, rect):
    """Échelle 1 sur A4 portrait : 2 colonnes x 2 rangées, recouvrement, croix de repère tous les 50 mm."""
    W, H = taille
    marge, uw, uh = 10.0, 190.0, 277.0
    xs = [0.0, W - uw]
    ys = [0.0, H - uh]
    pages = []
    for j, Y0 in enumerate(ys):
        for i, X0 in enumerate(xs):
            nom = '%s%d' % ('AB'[i], j + 1)
            c = ['<defs><clipPath id="z"><rect x="%g" y="%g" width="%g" height="%g"/></clipPath></defs>' % (marge, marge, uw, uh),
                 '<g clip-path="url(#z)">']
            c += dessin(p, taille, trous, rect, 1.0, marge - X0, marge - Y0, fin=0.2)
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
    p, taille, trous, ignores, rect = charger()
    os.makedirs(OUT, exist_ok=True)
    fichiers = []
    for nom, contenu in [('1_plan', page_plan(p, taille, trous, ignores, rect)), ('2_coordonnees', page_tableau(p, taille, trous))] + \
                        [('3_gabarit_' + n, s) for n, s in pages_gabarit(p, taille, trous, rect)]:
        f = os.path.join(OUT, 'page_%s.svg' % nom)
        open(f, 'w', encoding='utf-8').write(contenu)
        fichiers.append(f)
    subprocess.run(['rsvg-convert', '-f', 'pdf', '-o', os.path.join(OUT, 'plan_percages.pdf')] + fichiers, check=True)
    subprocess.run(['rsvg-convert', '-f', 'png', '-d', '160', '-p', '160', '-o', os.path.join(OUT, 'plan_percages.png'), fichiers[0]], check=True)
    for t in trous:
        print(t['id'], t['d'], round(t['X'], 1), round(t['Y'], 1), round(t['xa'], 1))
    print('ignorés', ignores)


if __name__ == '__main__':
    main()
