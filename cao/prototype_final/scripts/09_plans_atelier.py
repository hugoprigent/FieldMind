"""Plans d'atelier en un seul PDF, à partir des relevés existants (hors Fusion, Python 3 + rsvg-convert) :
1. remorque : découpe de la planche et des deux volets (contreplaqué 5 mm) ;
2. remorque : perçage de la planche ;
3. remorque : tableau des trous, rangée par rangée ;
4. volets : perçage ;
5. rover : découpe et perçage de la planche ;
6. rover : tableau des trous, rangée par rangée.
Une couleur par taille de trou, la même sur toutes les pages. Cotes cumulées depuis le bord gauche
(X) et le bord avant (Y) ; les trous alignés partagent la même ligne de rappel.
Sources : remorque_bilan.json (04_plan_planche.py) et plan_percages.json (06_plan_percages.py).
Produit ../plans_atelier/plans_atelier.pdf.
"""
import importlib.util, os, subprocess

ICI = os.path.dirname(os.path.abspath(__file__))


def module(nom, chemin):
    spec = importlib.util.spec_from_file_location(nom, chemin)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


pr = module('plan_rover', os.path.join(ICI, '06_plan_percages.py'))
rm = module('plan_remorque', os.path.join(ICI, '..', 'remorque', 'scripts', '04_plan_planche.py'))
txt, svg, fr, A4 = pr.txt, pr.svg, pr.fr, pr.A4
OUT = os.path.join(ICI, '..', 'plans_atelier')

# une couleur par taille : (couleur, libellé, Ø du fraisage sur le dessus ou None)
TYPES = {
    'M3': ('#1f6fb2', 'FORET 4 — vis M3', None),
    'M3F': ('#8e44ad', 'FORET 4 + fraisage Ø6 sur le dessus — vis M3 à tête fraisée', 6.4),
    'M6': ('#16a085', 'FORET 7 — vis M6', None),
    'M6F': ('#c0392b', 'FORET 7 + fraisage Ø12 sur le dessus — vis M6 à tête fraisée', 12.4),
    'M8': ('#9a7d0a', 'FORET 9 — vis M8', None),
    'C8': ('#e67e22', 'FORET 8 — passage de câble', None),
}
ORDRE = ['M3', 'M3F', 'M6', 'M6F', 'M8', 'C8']


def entete(titre, sous_titre):
    return [txt(15, 16, titre, 5, poids='bold'), txt(15, 22, sous_titre, 3, coul='#555')]


def legende(trous, x, y):
    c = []
    for k in ORDRE:
        n = sum(1 for t in trous if t['type'] == k)
        if not n:
            continue
        coul, lib, fd = TYPES[k]
        c.append('<circle cx="%.2f" cy="%.2f" r="1.6" fill="%s"/>' % (x + 2, y - 1, coul))
        if fd:
            c.append('<circle cx="%.2f" cy="%.2f" r="2.6" fill="none" stroke="%s" stroke-width="0.3" stroke-dasharray="0.8 0.5"/>' % (x + 2, y - 1, coul))
        c.append(txt(x + 6, y, '%s : %d trou%s' % (lib, n, 's' if n > 1 else ''), 3, poids='bold', coul=coul))
        y += 5.5
    return c, y


def niveaux(valeurs, ecart):
    """Étiquettes trop proches : décalées sur plusieurs rangs. Renvoie {valeur: rang}."""
    rangs, derniers = {}, []
    for v in valeurs:
        for k, d in enumerate(derniers):
            if v - d >= ecart:
                derniers[k] = v
                rangs[v] = k
                break
        else:
            derniers.append(v)
            rangs[v] = len(derniers) - 1
    return rangs


def plan(contour, taille, trous, s, ox, oy, decoupes=(), cotes_contour=(), x_depuis_droite=False, titre_x='X depuis le bord gauche (mm)'):
    """Contour (mm, X depuis le bord gauche, Y depuis le bord avant), trous, cotes cumulées en haut (X) et à
    gauche (Y) avec lignes de rappel jusqu'aux trous alignés."""
    W, H = taille
    T = lambda X, Y: (ox + X * s, oy + Y * s)
    c = ['<polygon points="%s" fill="#f6efe0" stroke="#7a5c2e" stroke-width="0.5"/>' % ' '.join('%.2f,%.2f' % T(X, Y) for X, Y in contour)]
    for (X0, Y0, X1, Y1, lib) in decoupes:
        x0, y0 = T(X0, Y0)
        c.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="white" stroke="#7a5c2e" stroke-width="0.4"/>' % (x0, y0, (X1 - X0) * s, (Y1 - Y0) * s))
    # cotes d'ensemble
    xb, yb = T(0, H)
    c.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#444" stroke-width="0.25"/>' % (ox, yb + 8, ox + W * s, yb + 8))
    c.append(txt(ox + W * s / 2, yb + 12, '%s mm' % fr(W, '%g'), 3.2, 'middle', 'bold'))
    xr = ox + W * s + 8
    c.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#444" stroke-width="0.25"/>' % (xr, oy, xr, oy + H * s))
    c.append('<text x="%.2f" y="%.2f" font-size="3.2" font-weight="bold" text-anchor="middle" transform="rotate(90 %.2f %.2f)" %s>%s mm</text>'
             % (xr + 3, oy + H * s / 2, xr + 3, oy + H * s / 2, pr.FONT, fr(H, '%g')))
    for lib, X, Y, ancre in cotes_contour:
        x, y = T(X, Y)
        c.append(txt(x, y, lib, 2.6, ancre, coul='#7a5c2e'))
    # cotes cumulées : X en haut, Y à gauche ; une ligne de rappel par valeur, jusqu'au dernier trou aligné
    xs = sorted({round(t['X'], 1) for t in trous})
    ys = sorted({round(t['Y'], 1) for t in trous})
    rx, ry = niveaux([v * s for v in xs], 3.4), niveaux([v * s for v in ys], 3.6)
    for v in xs:
        alignes = [t for t in trous if abs(t['X'] - v) < 0.06]
        coul = TYPES[alignes[0]['type']][0] if len({t['type'] for t in alignes}) == 1 else '#555'
        x, _ = T(v, 0)
        ytxt = oy - 3 - rx[v * s] * 9
        ybas = max(T(0, t['Y'])[1] for t in alignes)
        c.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="%s" stroke-width="0.15" stroke-dasharray="0.6 0.6"/>' % (x, ytxt + 0.8, x, ybas, coul))
        c.append('<text x="%.2f" y="%.2f" font-size="2.5" transform="rotate(-90 %.2f %.2f)" fill="%s" %s>%s</text>' % (x + 0.9, ytxt, x + 0.9, ytxt, coul, pr.FONT, fr(W - v if x_depuis_droite else v, '%g')))
    for v in ys:
        alignes = [t for t in trous if abs(t['Y'] - v) < 0.06]
        coul = TYPES[alignes[0]['type']][0] if len({t['type'] for t in alignes}) == 1 else '#555'
        _, y = T(0, v)
        xtxt = ox - 2 - ry[v * s] * 11
        xdroite = max(T(t['X'], 0)[0] for t in alignes)
        c.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="%s" stroke-width="0.15" stroke-dasharray="0.6 0.6"/>' % (xtxt + 0.5, y, xdroite, y, coul))
        c.append(txt(xtxt, y + 0.9, fr(v, '%g'), 2.5, 'end', coul=coul))
    if trous:
        c.append(txt(ox, oy - 3 - (max(rx.values()) + 1) * 9 - 1, titre_x, 2.6, coul='#555'))
        c.append(txt(ox - 2, oy + H * s + 4, 'Y depuis le bord avant (mm)', 2.6, 'end', coul='#555'))
    # trous
    for t in trous:
        coul, _, fd = TYPES[t['type']]
        x, y = T(t['X'], t['Y'])
        r = t['d'] / 2 * s
        if fd:
            c.append('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="none" stroke="%s" stroke-width="0.25" stroke-dasharray="0.8 0.5"/>' % (x, y, fd / 2 * s, coul))
        c.append('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="%s"/>' % (x, y, max(r, 0.9), coul))
        g = t['X'] > W / 2
        c.append(txt(x + (-1 if g else 1) * (max(r, fd / 2 * s if fd else 0) + 1.2), y - 1.4, t['id'], 2.3, 'end' if g else 'start', 'bold', coul))
    return c


def tableau(titre, sous_titre, trous, notes):
    """Une ligne par rangée (même Y), les trous de la rangée avec leur X, en couleur."""
    c = entete(titre, sous_titre)
    y = 34
    c += [txt(15, y, 'Y avant', 3.2, poids='bold'), txt(36, y, 'Trous de la rangée : repère et X depuis le bord gauche (mm)', 3.2, poids='bold')]
    c.append('<line x1="15" y1="%.1f" x2="195" y2="%.1f" stroke="#444" stroke-width="0.3"/>' % (y + 1.6, y + 1.6))
    y += 7
    for v in sorted({round(t['Y'], 1) for t in trous}):
        rang = sorted((t for t in trous if abs(t['Y'] - v) < 0.06), key=lambda t: t['X'])
        c.append(txt(15, y, fr(v, '%g'), 3.2, poids='bold'))
        x = 36
        for t in rang:
            coul = TYPES[t['type']][0]
            if x > 180:
                x, y = 36, y + 5
            c.append('<circle cx="%.2f" cy="%.2f" r="1.2" fill="%s"/>' % (x + 1.2, y - 1.1, coul))
            c.append(txt(x + 3.4, y, '%s  %s' % (t['id'], fr(t['X'], '%g')), 3, coul=coul))
            x += 25
        y += 6.5
    y += 4
    leg, y = legende(trous, 15, y)
    c += leg
    y += 3
    for n in notes:
        c.append(txt(15, y, n, 2.9))
        y += 5
    return svg(c)


# ---------------------------------------------------------------- remorque
def remorque():
    b, taille, poly, trous_r, org = rm.charger()
    type_r = {'T': 'M3', 'R': 'M3', 'F': 'M3', 'S': 'M6'}          # aucun fraisage dans le bois
    trous = [dict(t, type=type_r[t['id'][0]], X=round(t['X'], 1), Y=round(t['Y'], 1)) for t in trous_r]
    W, H = taille
    ch = b['chanfrein']
    vol = b['volets']
    lv = vol['y1'] - vol['y0']
    # 1. découpe
    c = entete('Remorque — découpe du contreplaqué 5 mm', 'Planche du plateau et deux volets. Cotes en mm, échelle 1:3.')
    s = 1 / 3
    ox, oy = 40, 45
    c += plan(poly, taille, [], s, ox, oy, cotes_contour=[
        ('%g' % ch, ch / 2, -2, 'middle'), ('%g' % ch, W - ch / 2, -2, 'middle'),
        ('%g' % ch, -2, ch / 2, 'end'),
        ('coins coupés à 45°', W / 2, ch / 2 + 8, 'middle'), ('AVANT (attelage)', W / 2, -10, 'middle'),
        ('Planche × 1', W / 2, H / 2, 'middle')])
    rect = [(0, 0), (lv, 0), (lv, vol['largeur']), (0, vol['largeur'])]
    c += plan(rect, (lv, vol['largeur']), [], s, ox, oy + H * s + 30, cotes_contour=[
        ('Volet × 2 (identiques au débit)', lv / 2, vol['largeur'] / 2 + 3, 'middle')])
    y = oy + 8
    for n in ['Débit, contreplaqué 5 mm', '(même bois que la planche du rover) :', '',
              '1 planche %g × %g,' % (W, H), 'puis les deux coins AVANT', 'coupés à 45° : %g mm' % ch, 'le long de chaque bord.', '',
              '2 volets %s × %s' % (fr(vol['largeur'], '%g'), fr(lv, '%g')), '(rectangles).', '',
              'Poncer les chants.', 'Trous : pages suivantes.']:
        c.append(txt(155, y, n, 3))
        y += 5.2
    p1 = svg(c)
    # 2. perçage de la planche
    c = entete('Remorque — perçage de la planche (vue de dessus)', 'Échelle 1:2. Une couleur par taille de trou. Trous droits, aucun fraisage.')
    s = 0.5
    ox, oy = 45, 62
    c += plan(poly, taille, trous, s, ox, oy)
    leg, _ = legende(trous, 15, oy + H * s + 20)
    c += leg
    p2 = svg(c)
    # 3. tableau
    p3 = tableau('Remorque — trous de la planche, rangée par rangée', 'Y depuis le bord avant, X depuis le bord gauche, en mm. Repères du plan de la page précédente.', trous, [
        'Tous les trous vont par paires symétriques : même Y, X à égale distance des deux bords (X + X\' = %g).' % W,
        'Perçage : avant-trou au foret 2 ou 3, planche sur une cale martyre, puis le foret de la couleur. Aucun fraisage.',
        'Vis par le dessus à tête bombée (M3×16 et M6×16) : la tête reste posée sur le bois.',
        'T : timon (vis par-dessous). R : rebords. S : supports de roue. F : boîtiers des servos et blocs arrière.',
        'Gabarit à l\'échelle 1 : plan_planche/plan_planche.pdf, pages 3 à 6.'])
    # 4. volets
    xg, xd = vol.get('trou_x', (round(175 - vol['x_axe'], 1), round(192 - vol['x_axe'], 1)))   # depuis le bord côté charnière
    ya, yr = vol.get('trou_y', (13.0, 10.0))                                                   # depuis les bouts avant et arrière
    c = entete('Volets — perçage (vue de dessus, volets dépliés)', 'Échelle 1:2. 4 trous droits par volet (foret 4), sans fraisage. Les deux volets sont symétriques.')
    s = 0.5
    trous_v = []
    for k, (ox, nom, charn_droite) in enumerate(((40, 'volet de gauche', True), (130, 'volet de droite', False))):
        tv = []
        for j, (xx, yy) in enumerate([(xg, ya), (xd, ya), (xg, lv - yr), (xd, lv - yr)], 1):
            X = vol['largeur'] - xx if charn_droite else xx
            tv.append({'id': 'V%s%d' % ('G' if charn_droite else 'D', j), 'X': X, 'Y': yy, 'd': 3.4, 'type': 'M3'})
        rect = [(0, 0), (vol['largeur'], 0), (vol['largeur'], lv), (0, lv)]
        bord = vol['largeur'] if charn_droite else 0
        c += plan(rect, (vol['largeur'], lv), tv, s, ox, 66, x_depuis_droite=charn_droite, titre_x='X depuis la charnière (mm)', cotes_contour=[
            ('charnière', bord + (3 if charn_droite else -3), -4, 'start' if charn_droite else 'end'), ('AVANT', vol['largeur'] / 2, -16, 'middle'),
            (nom, vol['largeur'] / 2, lv / 2, 'middle')])
        x_ch = ox + bord * s
        c.append('<line x1="%.2f" y1="66" x2="%.2f" y2="%.2f" stroke="#7a5c2e" stroke-width="1.2"/>' % (x_ch, x_ch, 66 + lv * s))
        trous_v += tv
    y = 66 + lv * s + 22
    for n in ['Repères depuis le bord côté charnière et depuis le bout avant :',
              '- 2 trous à %s et %s mm du bord côté charnière, à %s mm du bout AVANT (ferrure avant, palonnier du servo) ;' % (fr(xg, '%g'), fr(xd, '%g'), fr(ya, '%g')),
              '- 2 trous aux mêmes distances du bord, à %s mm du bout ARRIÈRE (ferrure arrière, axe M3).' % fr(yr, '%g'),
              'Le bord côté charnière (trait épais) est celui qui longe la planche. Volets vus comme sur la remorque, avant en haut.',
              'Vis M3×16 à tête bombée par le dessus, écrous noyés sous les ferrures. Aucun fraisage.']:
        c.append(txt(15, y, n, 3))
        y += 5.5
    leg, _ = legende(trous_v, 15, y + 3)
    c += leg
    p4 = svg(c)
    return [('1_remorque_decoupe', p1), ('2_remorque_percage', p2), ('3_remorque_tableau', p3), ('4_volets_percage', p4)]


# ---------------------------------------------------------------- rover
def rover():
    p, taille, trous_p, ignores, rect = pr.charger()
    W, H = taille
    type_p = {'P': 'M8', 'M': 'M8', 'E': 'M8', 'S': 'M3', 'B': 'M3', 'X': 'M3', 'K': 'M6', 'A': 'M6', 'C': 'C8'}
    trous = [dict(t, type=type_p[t['id'][0]], X=round(t['X'], 1), Y=round(t['Y'], 1)) for t in trous_p]
    pl = p['planche']
    ds = p['decoupe_servo']
    dec = (ds['x0'] - pl['x0'], ds['y0'] - pl['y0'], ds['x1'] - pl['x0'], ds['y1'] - pl['y0'], 'servo')
    poly = [(0, 0), (W, 0), (W, H), (0, H)]
    c = entete('Rover — découpe et perçage de la planche (vue de dessus)',
               'Planche %g × %g × 5 mm, avant en haut. Échelle 1:2,7. Une couleur par taille de trou.' % (W, H))
    s = 0.37
    ox, oy = 55, 58
    c += plan(poly, taille, trous, s, ox, oy, decoupes=[dec])
    leg, y = legende(trous, 15, oy + H * s + 18)
    c += leg
    c.append('<rect x="15.4" y="%.2f" width="3.2" height="4" fill="white" stroke="#7a5c2e" stroke-width="0.4"/>' % (y - 4))
    c.append(txt(21, y, 'Découpe du servo, %s × %s : X %s à %s, Y %s à %s' % (fr(dec[2] - dec[0], '%g'), fr(dec[3] - dec[1], '%g'), fr(dec[0], '%g'),
                                                                                fr(dec[2], '%g'), fr(dec[1], '%.1f'), fr(dec[3], '%.1f')), 3, poids='bold', coul='#7a5c2e'))
    p5 = svg(c)
    p6 = tableau('Rover — trous de la planche, rangée par rangée', 'Y depuis le bord avant, X depuis le bord gauche, en mm. Repères du plan de la page précédente.', trous, [
        'Tous les trous vont par paires symétriques, sauf C1 et le servo (sur l\'axe, X = %g).' % (W / 2),
        'P : supports de pivot. M : supports moteurs. E : équerres des mâts. S : oreilles du servo (pointer avec le servo).',
        'K : coque. A : chape d\'attelage. B : berceau de batterie. X : plaque des XL4015. C : câble des volets de la remorque.',
        'Découpe du servo : percer un trou dans chaque coin, puis scier entre les trous et limer.',
        'Gabarit à l\'échelle 1 : plan_percages/plan_percages.pdf, pages 3 à 6.'])
    return [('5_rover_percage', p5), ('6_rover_tableau', p6)]


def main():
    os.makedirs(OUT, exist_ok=True)
    fichiers = []
    for nom, contenu in remorque() + rover():
        f = os.path.join(OUT, 'page_%s.svg' % nom)
        open(f, 'w', encoding='utf-8').write(contenu)
        fichiers.append(f)
    subprocess.run(['rsvg-convert', '-f', 'pdf', '-o', os.path.join(OUT, 'plans_atelier.pdf')] + fichiers, check=True)
    for f in fichiers:
        subprocess.run(['rsvg-convert', '-f', 'png', '-d', '110', '-p', '110', '-o', f[:-4] + '.png', f], check=True)
    print('\n'.join(fichiers))


if __name__ == '__main__':
    main()
