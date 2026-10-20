"""Gabarit à l'échelle 1 du support des XL4015, à partir de exports/support_xl4015.json (écrit par
10_support_xl4015.py). Hors Fusion (Python 3 + rsvg-convert). Imprimer à 100 %, contrôler la règle de
50 mm, poser chaque module sur son contour (IN en bas) : ses deux trous doivent tomber sur les croix.
Produit exports/support_xl4015_gabarit.pdf.
"""
import importlib.util, json, os, subprocess

ICI = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('plan_rover', os.path.join(ICI, '06_plan_percages.py'))
pr = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pr)
txt, svg, croix, fr = pr.txt, pr.svg, pr.croix, pr.fr
EXP = os.path.join(ICI, '..', 'exports')


def main():
    s = json.load(open(os.path.join(EXP, 'support_xl4015.json'), encoding='utf-8'))
    x0, y0, x1, y1 = s['plaque']
    W, L = s['pcb']
    ox, oy = 60, 70                       # coin bas-gauche du circuit de gauche (bout IN) sur la feuille
    T = lambda x, y: (ox + x, oy + (y1 - y) - (y1 - y0) + (y1 - y0))   # Y vers le haut sur la feuille
    T = lambda x, y: (ox + x, oy + (L + 12) - y)
    c = [txt(15, 16, 'Support des 2 XL4015 — gabarit à l\'échelle 1', 5, poids='bold'),
         txt(15, 22, 'Imprimer à 100 % et vérifier la règle de 50 mm. Poser chaque module sur son contour, IN en bas.', 3, coul='#555'),
         txt(15, 27, 'Ses deux trous doivent tomber sur les croix. Sinon, mesurer A et B (voir en bas) et me les donner.', 3, coul='#555')]
    pts = [T(x0, y0), T(x1, y0), T(x1, y1), T(x0, y1)]
    c.append('<polygon points="%s" fill="#f4f4f4" stroke="#999" stroke-width="0.3"/>' % ' '.join('%.2f,%.2f' % p for p in pts))
    for m, nom in enumerate(('Pi 5,1 V', 'servo 6 V')):
        dx = m * (W + s['ecart'])
        a, b = T(dx, 0), T(dx + W, L)
        c.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="none" stroke="#1f6fb2" stroke-width="0.4"/>' % (a[0], b[1], W, L))
        c.append(txt(a[0] + W / 2, a[1] - 3, 'IN', 3, 'middle', 'bold', '#1f6fb2'))
        c.append(txt(a[0] + W / 2, b[1] + 4.5, 'OUT', 3, 'middle', 'bold', '#1f6fb2'))
        c.append(txt(a[0] + W / 2, (a[1] + b[1]) / 2, nom, 2.8, 'middle', coul='#1f6fb2'))
    for t in s['trous_vis']:
        x, y = T(t['x'], t['y'])
        c.append(croix(x, y, 1.5, '#c0392b', 0.3))
    for x, y in s['fixations']:
        X, Y = T(x, y)
        c.append(croix(X, Y, 1.7, '#555', 0.3))
    # cotes du circuit de gauche
    (tx1, ty1), (tx2, ty2) = s['trous_pcb']
    yb = oy + L + 12 + 24
    lignes = ['Circuit %s × %s mm, 2 trous Ø3 en diagonale :' % (fr(W, '%g'), fr(L, '%g')),
              '- trou côté IN : %s mm du bord gauche, %s mm du bout IN ;' % (fr(tx1, '%g'), fr(ty1, '%g')),
              '- trou côté OUT : %s mm du bord droit, %s mm du bout OUT ;' % (fr(W - tx2, '%g'), fr(L - ty2, '%g')),
              '- A = entraxe en longueur : %s mm ; B = entraxe en largeur : %s mm.' % (fr(ty2 - ty1, '%g'), fr(tx2 - tx1, '%g')),
              'Écart entre les deux circuits : %s mm. Plots de %s mm ; vis M3×8 par le dessus, écrou M3 dans la poche sous la plaque.' % (fr(s['ecart'], '%g'), fr(s['plot_h'], '%g')),
              'Croix grises : fixation de la plaque sur la planche (vis M3×12 + rondelle + écrou, foret 4 dans la planche).']
    for k, l in enumerate(lignes):
        c.append(txt(15, yb + k * 5.5, l, 3))
    yr = yb + len(lignes) * 5.5 + 6
    c.append('<line x1="15" y1="%.1f" x2="65" y2="%.1f" stroke="#222" stroke-width="0.4"/>' % (yr, yr))
    for k in range(6):
        c.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#222" stroke-width="0.3"/>' % (15 + 10 * k, yr - 2, 15 + 10 * k, yr))
    c.append(txt(70, yr + 1, 'règle de contrôle : 50 mm', 3))
    f = os.path.join(EXP, 'support_xl4015_gabarit.svg')
    open(f, 'w', encoding='utf-8').write(svg(c))
    subprocess.run(['rsvg-convert', '-f', 'pdf', '-o', os.path.join(EXP, 'support_xl4015_gabarit.pdf'), f], check=True)
    subprocess.run(['rsvg-convert', '-f', 'png', '-d', '110', '-p', '110', '-o', f[:-4] + '.png', f], check=True)
    print('ok')


if __name__ == '__main__':
    main()
