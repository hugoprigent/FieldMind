#!/usr/bin/env python3
"""Plan de repérage A7 sans fils : composants à leur place et nom de chaque broche.

Écrit exports/c5_a7/plan_broches.svg, .png et .html (deux faces : dessus, et
dessous vu en miroir). Ne modifie pas le projet KiCad.
"""
import subprocess
from html import escape
from pathlib import Path

import plan_c5_a7 as P

BASE = Path(__file__).resolve().parents[1]
OUT = BASE / 'exports/c5_a7'
S = 20                       # px par mm
NETCOL = {'+3V3': '#c8353b', 'GND': '#26364a', 'I2C_SDA': '#0b84c6', 'I2C_SCL': '#11905a', 'GNSS_RX': '#7a52b3',
          'GNSS_TX': '#e07b14', 'VBUS_USB': '#a01e6a', 'EXT_D0': '#b98900', 'EXT_D1': '#5f6b7a', 'EXT_D2': '#00a3a3',
          'EXT_D3': '#7d9c1c', 'EXT_D8': '#c25a00', 'EXT_D9': '#f08ab8', 'EXT_D10': '#6b4518'}
FILL = {'module': '#2f8f62', 'header': '#111'}
LABEL = {'XIAO': 'U1 · XIAO ESP32C5', 'OLED': 'J3 · Écran OLED', 'GY521': 'J1 · GY-521', 'BMP280': 'J2 · BMP280'}
HEADER = {'J4': 'J4 · GPS', 'J6': 'J6 · Extension', 'J7': 'J7 · Knob', 'J8': 'J8 · D0-D3'}


def color(net):
    return NETCOL.get(net, '#7a8594' if not net else '#b5651d')


def face_svg(face, ox, oy):
    mirror = face == 'B'
    X = lambda x: ox + (P.BOARD_W - x if mirror else x) * S
    Y = lambda y: oy + y * S
    s = [f'<rect x="{ox}" y="{oy}" width="{P.BOARD_W * S}" height="{P.BOARD_H * S}" rx="10" fill="#1f4f9a"/>']
    for (mx, my) in P.MOUNT_HOLES:
        s.append(f'<circle cx="{X(mx)}" cy="{Y(my)}" r="{1.6 * S}" fill="#f3f5f8" opacity=".85"/>')
    for c in range(P.COLS):
        s.append(f'<text x="{X(P.hxy((c, 0))[0])}" y="{Y(2.9)}" class="grid">{P.LETTERS[c]}</text>')
    for r in range(P.ROWS):
        s.append(f'<text x="{X(1.2)}" y="{Y(P.hxy((0, r))[1]) + 4}" class="grid">{r + 1}</text>')
    for r in range(P.ROWS):
        for c in range(P.COLS):
            x, y = P.hxy((c, r))
            s.append(f'<circle cx="{X(x)}" cy="{Y(y)}" r="{0.8 * S}" fill="#c9a64b" opacity=".55"/>'
                     f'<circle cx="{X(x)}" cy="{Y(y)}" r="{0.35 * S}" fill="#0b1c35"/>')
    parts = [p for p in P.PARTS if p['face']]
    # contours : cartes de l'autre face en tirets, sous celles de la face montrée
    for p in sorted(parts, key=lambda p: p['face'] == face):
        r = P.body_rect(p)
        x0, x1 = sorted([X(r[0]), X(r[2])])
        if p['face'] == face:
            s.append(f'<rect x="{x0}" y="{Y(r[1])}" width="{x1 - x0}" height="{(r[3] - r[1]) * S}" rx="5" '
                     f'fill="{FILL[p["kind"]]}" fill-opacity="{0.35 if p["kind"] == "module" else 0.85}" stroke="#0a0f14" stroke-width="2"/>')
        else:
            s.append(f'<rect x="{x0}" y="{Y(r[1])}" width="{x1 - x0}" height="{(r[3] - r[1]) * S}" rx="5" fill="none" '
                     f'stroke="#d5dfec" stroke-width="1.5" stroke-dasharray="7 5"/>')
        for sr in P.socket_rects(p):
            a, b = sorted([X(sr[0]), X(sr[2])])
            if p['face'] == face:
                s.append(f'<rect x="{a}" y="{Y(sr[1])}" width="{b - a}" height="{(sr[3] - sr[1]) * S}" fill="#0d0d0d" opacity=".55"/>')
    for s_ in P.SUPPORTS:
        if P.BYREF[s_['ref']]['face'] == face:
            for (x, y) in P.support_points(s_):
                s.append(f'<circle cx="{X(x)}" cy="{Y(y)}" r="{s_["d"] / 2 * S}" fill="#f2f2ea" fill-opacity=".5" stroke="#333" stroke-dasharray="4 3"/>')
    # titres des composants
    for p in parts:
        r = P.body_rect(p)
        name = LABEL.get(p.get('module')) or HEADER.get(p['ref'], p['ref'])
        cx, cy = (X(r[0]) + X(r[2])) / 2, Y((r[1] + r[3]) / 2)
        cls = 'title' if p['face'] == face else 'title other'
        if p['kind'] == 'header':
            pins = [P.hxy(n) for n in p['pins']]
            ty = Y(min(y for _, y in pins)) - 0.9 * S
            s.append(f'<text x="{X(pins[0][0])}" y="{ty}" class="{cls} small">{escape(p["ref"])}</text>')
        elif p['module'] == 'XIAO':
            s.append(f'<text x="{cx}" y="{Y(r[3]) - 0.9 * S}" class="{cls}">{escape(name)}</text>')
            s.append(f'<text x="{cx}" y="{Y(r[3]) + 1.3 * S}" class="{cls} small">USB-C au bord haut</text>')
        elif p['module'] == 'OLED':
            s.append(f'<text x="{cx}" y="{cy + 2 * S}" class="{cls}">{escape(name)}</text>')
        else:
            s.append(f'<text x="{cx}" y="{cy}" class="{cls}">{escape(name)}</text>')
    # broches : repère et nom
    for p in parts:
        mine = p['face'] == face
        names = p.get('names') or [str(i + 1) for i in range(len(p['pins']))]
        cols = {c for c, _ in p['pins']}
        vertical = len(cols) == 1 or p.get('module') == 'XIAO'
        for i, (n, nm, net) in enumerate(zip(p['pins'], names, p['nets']), 1):
            x, y = P.hxy(n)
            col = color(net)
            if i == 1:
                s.append(f'<rect x="{X(x) - 0.75 * S}" y="{Y(y) - 0.75 * S}" width="{1.5 * S}" height="{1.5 * S}" fill="{col}" stroke="#fff" stroke-width="2"/>')
            else:
                s.append(f'<circle cx="{X(x)}" cy="{Y(y)}" r="{0.75 * S}" fill="{col}" stroke="#fff" stroke-width="2"/>')
            text = nm if net else f'{nm} ×'
            if vertical:
                # étiquette vers l'intérieur de la plaque (vue montrée)
                inside_right = (X(x) - ox) < P.BOARD_W * S / 2
                if p.get('module') == 'XIAO':
                    xs = sorted(cols)
                    inside_right = (n[0] == xs[0]) != mirror
                tx = X(x) + (1.0 * S if inside_right else -1.0 * S)
                anchor = 'start' if inside_right else 'end'
                s.append(f'<text x="{tx}" y="{Y(y) + 5}" class="pin{"" if mine else " other"}" text-anchor="{anchor}" fill="{col}">{escape(text)}</text>')
            else:
                up = y > P.BOARD_H / 2           # rangée du bas : étiquettes au-dessus
                ty = Y(y) + (-1.0 * S if up else 1.0 * S)
                anchor = 'start' if up else 'end'
                s.append(f'<text x="{X(x) + 4}" y="{ty}" class="pin{"" if mine else " other"}" text-anchor="{anchor}" fill="{col}" '
                         f'transform="rotate(-90 {X(x) + 4} {ty})">{escape(text)}</text>')
    return '\n'.join(s)


def svg():
    W, H = 2 * P.BOARD_W * S + 3 * 70 + 360, P.BOARD_H * S + 190
    bx = 70 + P.BOARD_W * S + 70
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
         '<style>text{font-family:Helvetica,Arial,sans-serif;fill:#1d2733}.grid{font-size:13px;font-weight:bold;fill:#e8eef6;text-anchor:middle;paint-order:stroke;stroke:#1f4f9a;stroke-width:4px}'
         '.title{font-size:17px;font-weight:bold;fill:#fff;text-anchor:middle;paint-order:stroke;stroke:#0a0f14;stroke-width:3px}'
         '.small{font-size:13px}.other{opacity:.55}'
         '.pin{font-size:13px;font-weight:bold;paint-order:stroke;stroke:#fff;stroke-width:3.5px}</style>',
         f'<rect width="{W}" height="{H}" fill="#f3f5f8"/>',
         '<text x="70" y="48" font-size="28" font-weight="bold">FieldMind C5 · A7 · plan des broches (sans fils)</text>',
         '<text x="70" y="78" font-size="16">Positions des composants de la révision A7. Carré = broche 1 ; × = broche non câblée. '
         'Même couleur = même signal : ces broches sont toutes reliées entre elles.</text>',
         f'<text x="70" y="140" font-size="20" font-weight="bold">FACE DESSUS (côté écran)</text>',
         f'<text x="{bx}" y="140" font-size="20" font-weight="bold">FACE DESSOUS, vue de dessous (miroir)</text>',
         face_svg('F', 70, 160), face_svg('B', bx, 160)]
    lx, ly = bx + P.BOARD_W * S + 70, 190
    s.append(f'<text x="{lx}" y="{ly}" font-size="19" font-weight="bold">Couleurs</text>')
    legend = [('GND', 'GND'), ('+3V3', '3V3'), ('I2C_SDA', 'SDA (D4)'), ('I2C_SCL', 'SCL (D5)'), ('GNSS_TX', 'D6 → RX GPS'),
              ('GNSS_RX', 'D7 ← TX GPS'), ('VBUS_USB', '5V USB'), ('EXT_D0', 'D0'), ('EXT_D1', 'D1'), ('EXT_D2', 'D2'),
              ('EXT_D3', 'D3'), ('EXT_D8', 'D8'), ('EXT_D9', 'D9'), ('EXT_D10', 'D10'), (None, 'non câblé')]
    for i, (net, t) in enumerate(legend):
        y = ly + 34 + i * 30
        s.append(f'<circle cx="{lx + 10}" cy="{y - 5}" r="9" fill="{color(net)}"/><text x="{lx + 30}" y="{y}" font-size="15">{escape(t)}</text>')
    notes = ['Rectangle plein : composant sur la face montrée.', 'Tirets : composant de l’autre face.',
             'Noms de trous identiques sur les deux vues', '(vus du dessus : colonnes A…N, rangées 1…20).',
             'Dessous, les noms sont en miroir gauche/droite', 'comme quand on retourne la plaque.',
             'Ronds clairs : entretoises collées sous les cartes.']
    for i, t in enumerate(notes):
        s.append(f'<text x="{lx}" y="{ly + 34 + len(legend) * 30 + 30 + i * 24}" font-size="14">{escape(t)}</text>')
    return '\n'.join(s + ['</svg>']) + '\n'


def table():
    rows = []
    for p in P.PARTS:
        if not p['face']:
            continue
        names = p.get('names') or [str(i + 1) for i in range(len(p['pins']))]
        cells = ' · '.join(f'<b style="color:{color(net)}">{escape(nm)}</b> {P.hole_name(n)}' + ('' if net else ' (non câblé)')
                           for n, nm, net in zip(p['pins'], names, p['nets']))
        face = 'dessus' if p['face'] == 'F' else 'dessous'
        rows.append(f'<tr><td><b>{p["ref"]}</b></td><td>{escape(p["value"])}</td><td>{face}</td><td>{cells}</td></tr>')
    return ''.join(rows)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    view = svg()
    (OUT / 'plan_broches.svg').write_text(view)
    subprocess.run(['rsvg-convert', '-w', '2400', OUT / 'plan_broches.svg', '-o', OUT / 'plan_broches.png'], check=True)
    (OUT / 'plan_broches.html').write_text(
        '<!doctype html><html lang="fr"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
        '<title>FieldMind A7 — plan des broches</title><style>body{font:16px system-ui;margin:20px;background:#f3f5f8;color:#1d2733}'
        'svg{width:100%;height:auto}table{border-collapse:collapse;background:#fff;width:100%;margin-top:16px}'
        'td,th{padding:7px 9px;border-bottom:1px solid #d8e0e8;text-align:left;font-size:15px;vertical-align:top}</style>'
        f'{view}<h2>Broches par composant (trou vu du dessus)</h2><table><thead><tr><th>Réf.</th><th>Composant</th><th>Face</th>'
        f'<th>Broches</th></tr></thead><tbody>{table()}</tbody></table></html>\n')
    print(OUT / 'plan_broches.html')


if __name__ == '__main__':
    main()
