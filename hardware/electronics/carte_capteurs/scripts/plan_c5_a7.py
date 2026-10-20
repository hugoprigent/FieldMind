#!/usr/bin/env python3
"""A7 : une seule perfboard 4 x 6 cm, TOUS les fils sur la face dessous.

A7 = A6 (mêmes liaisons, toutes les broches du XIAO sorties) avec :
  - face dessus (côté écran) : uniquement les cartes et connecteurs sur leurs
    supports, et des ponts de soudure visibles, AUCUN fil ;
  - J4 (GNSS), J6 (extension) et J7 (Knob) passent sur la face dessus, au bord :
    leurs broches se soudent dessous, directement sur les fils ;
  - le XIAO reste dessous sur ses supports : ses broches ne se soudent que
    dessus. Chaque broche reliée y est unie au trou voisin par un PONT DE
    SOUDURE (pas de fil) ; le fil part de ce trou voisin, dessous ;
  - un fil isolé peut croiser un autre fil à angle droit, par-dessus.

Module de données et de contrôle, sans écriture de fichier. Repère : vue DESSUS,
trou (colonne, rangée), colonnes A…N, rangées 1…20. Face F = dessus (écran,
IMU, baromètre, J4, J6, J7), face B = dessous (XIAO et tous les fils).

Orientation physique des cartes : chaque carte est décrite dans sa vue de
référence (côté composants vers soi, barrette en haut ou en bas, ordre de
broches de gauche à droite tel qu'imprimé habituellement). Une carte montée
en face B est vue en miroir depuis le dessus. Les positions des broches sont
CALCULÉES à partir de (broche 1, rotation, face) : on ne peut pas obtenir un
ordre de broches impossible physiquement.

Fils (face dessous) : fil nu étamé posé sur une ligne de trous et soudé à chaque
pastille traversée, ou fil isolé passant entre les pastilles (soudé à ses deux
bouts). Au plus une extrémité de fil par pastille ; jamais deux fils dans le
même trou ; un croisement n'est permis qu'à angle droit, entre deux pastilles,
le fil qui passe dessus étant isolé.
"""
import heapq
import math

COLS, ROWS, PITCH = 14, 20, 2.54
BOARD_W, BOARD_H = 40.0, 60.0
X0 = round((BOARD_W - (COLS - 1) * PITCH) / 2, 3)
Y0 = round((BOARD_H - (ROWS - 1) * PITCH) / 2, 3)
LETTERS = 'ABCDEFGHIJKLMN'
SOCKET_H, MALE_SPACER, MODULE_Z = 8.5, 2.5, 11.0
FACES = ('F', 'B')


def hole_name(node):
    return f'{LETTERS[node[0]]}{node[1] + 1}'


def hxy(node):
    return (round(X0 + PITCH * node[0], 4), round(Y0 + PITCH * node[1], 4))


NC = None
XIAO_NAMES = ['D0', 'D1', 'D2', 'D3', 'D4', 'D5', 'D6', 'D7', 'D8', 'D9', 'D10', '3V3', 'GND', '5V']
XIAO_GPIO = ['GPIO1', 'GPIO0', 'GPIO25', 'GPIO7', 'GPIO23', 'GPIO24', 'GPIO11', 'GPIO12', 'GPIO8', 'GPIO9', 'GPIO10', '', '', '']

# Vue de référence (côté composants, lecture normale). Cotes typiques NON mesurées.
#   header='top'  : barrette au bord haut, corps en dessous ;  'bottom' : l'inverse.
MODULES = {
    'XIAO': dict(L=21.0, W=17.8, t=1.0, src='Seeed XIAO ESP32C5, 21 x 17,8 mm (doc Seeed) ; USB-C en haut'),
    'OLED': dict(L=27.3, D=27.8, e=1.5, t=1.2, header='top',
                 src='OLED 0,96 po I2C : barrette en haut, GND VCC SCL SDA de gauche à droite (variante courante)'),
    'GY521': dict(L=21.2, D=16.4, e=1.5, t=1.6, header='bottom',
                  src='GY-521 : barrette en bas, VCC GND SCL SDA XDA XCL AD0 INT de gauche à droite (courant)'),
    'BMP280': dict(L=15.5, D=11.5, e=1.3, t=1.1, header='bottom',
                   src='GY-BMP280-3.3 : barrette en bas, VCC GND SCL SDA CSB SDO de gauche à droite (courant)'),
}


def rot(x, y, deg):
    a = math.radians(deg)
    return (x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a))


def placed_module(p):
    """Broches (trous) et rectangle du corps (mm) depuis (pin1, rotation, face)."""
    m = MODULES[p['module']]
    if p['module'] == 'XIAO':
        loc = [(0.0, i * PITCH) for i in range(7)] + [(6 * PITCH, (6 - i) * PITCH) for i in range(7)]
        cx, cy = 3 * PITCH, 3 * PITCH
        corners = [(cx - m['W'] / 2, cy - m['L'] / 2), (cx + m['W'] / 2, cy + m['L'] / 2)]
        usb = [(cx, cy - m['L'] / 2)]
    else:
        n = len(p['nets'])
        loc = [(i * PITCH, 0.0) for i in range(n)]
        cx = (n - 1) * PITCH / 2
        s = 1 if m['header'] == 'top' else -1          # sens du corps depuis la barrette
        y0, y1 = -s * m['e'], -s * m['e'] + s * m['D']
        corners = [(cx - m['L'] / 2, y0), (cx + m['L'] / 2, y1)]
        usb = []
    def tf(x, y):
        if p['face'] == 'B':
            x = -x
        return rot(x, y, p['rot'])
    x1, y1 = hxy(p['pin1'])
    pins = []
    for (x, y) in loc:
        u, v = tf(x, y)
        c, r = p['pin1'][0] + u / PITCH, p['pin1'][1] + v / PITCH
        assert abs(c - round(c)) < 1e-6 and abs(r - round(r)) < 1e-6
        pins.append((round(c), round(r)))
    pts = [tf(x, y) for x, y in [corners[0], (corners[0][0], corners[1][1]), corners[1], (corners[1][0], corners[0][1])]]
    xs = [x1 + u for u, _ in pts]; ys = [y1 + v for _, v in pts]
    rect = (round(min(xs), 3), round(min(ys), 3), round(max(xs), 3), round(max(ys), 3))
    usbpt = [(x1 + tf(*u)[0], y1 + tf(*u)[1]) for u in usb]
    return pins, rect, usbpt


def line(origin, direction, n, pitch=1):
    dc, dr = {'R': (1, 0), 'L': (-1, 0), 'D': (0, 1), 'U': (0, -1)}[direction]
    return [(origin[0] + dc * i * pitch, origin[1] + dr * i * pitch) for i in range(n)]


# Connecteurs au bord de la face dessus (prises Dupont par le dessus), alignés sur
# les broches du XIAO : chaque liaison vers l'extension est un fil droit d'une rangée.
J6_NETS = ['VBUS_USB', 'GND', '+3V3', 'EXT_D10', 'EXT_D9', 'EXT_D8']   # bord gauche, A2…A7, face aux broches 5V…D8
J8_NETS = ['EXT_D0', 'EXT_D1', 'EXT_D2', 'EXT_D3']                    # bord droit, N2…N5, face à D0…D3
NAME_OF = {'+3V3': '3V3', 'VBUS_USB': '5V'}
J6_NAMES = [NAME_OF.get(n, n.replace('EXT_', '')) for n in J6_NETS]
J8_NAMES = [n.replace('EXT_', '') for n in J8_NETS]
J6_PINS = line((0, 1), 'D', 6)
J8_PINS = line((13, 1), 'D', 4)
J7_PINS = line((13, 6), 'D', 4)     # bord droit, N7…N10
J4_PINS = line((0, 11), 'U', 4)     # bord gauche, A12 (broche 1) … A9

PARTS = [
    # ---- face F (dessus) : écran et capteurs I2C
    dict(ref='J3', face='F', kind='module', module='OLED', pin1=(5, 0), rot=0, value='OLED 0,96 I2C', sheet='B',
         nets=['GND', '+3V3', 'I2C_SCL', 'I2C_SDA'], names=['GND', 'VCC', 'SCL', 'SDA'],
         note='Écran en haut de la face dessus, lecture normale.'),
    dict(ref='J1', face='F', kind='module', module='GY521', pin1=(1, 19), rot=0, value='GY-521 MPU-6050', sheet='B',
         nets=['+3V3', 'GND', 'I2C_SCL', 'I2C_SDA', NC, NC, NC, NC], names=['VCC', 'GND', 'SCL', 'SDA', 'XDA', 'XCL', 'AD0', 'INT'],
         note='Face dessus, barrette au bord bas. VCC en 3V3 direct. Fixée rigidement (2 entretoises). AD0 libre : rappel GND sur la carte (0x68), à vérifier.'),
    dict(ref='J2', face='F', kind='module', module='BMP280', pin1=(13, 17), rot=90, value='GY-BMP280-3.3', sheet='B',
         nets=['+3V3', 'GND', 'I2C_SCL', 'I2C_SDA', NC, NC], names=['VCC', 'GND', 'SCL', 'SDA', 'CSB', 'SDO'],
         note='Face dessus, barrette au bord droit. CSB/SDO libres : rappels sur la carte (I2C 0x76), à vérifier.'),
    # ---- face B (dessous) : XIAO seul ; connecteurs de câble au bord de la face dessus
    dict(ref='U1', face='B', kind='module', module='XIAO', pin1=(9, 1), rot=0, value='Seeed XIAO ESP32C5', sheet='A',
         nets=['EXT_D0', 'EXT_D1', 'EXT_D2', 'EXT_D3', 'I2C_SDA', 'I2C_SCL', 'GNSS_TX', 'GNSS_RX', 'EXT_D8', 'EXT_D9', 'EXT_D10',
               '+3V3', 'GND', 'VBUS_USB'],
         names=XIAO_NAMES, note='Face dessous, USB-C au bord haut de la plaque.'),
    dict(ref='J4', face='F', kind='header', pins=J4_PINS, value='GNSS déporté (câble)', sheet='A',
         nets=['GNSS_TX', 'GND', '+3V3', 'GNSS_RX'], names=['RX_GPS', 'GND', 'VCC', 'TX_GPS'],
         note='Module GNSS et antenne déportés. VCC branché seulement après contrôle.'),
    dict(ref='J6', face='F', kind='header', pins=J6_PINS, value='Extension 5V/3V3/D8-D10 (Dupont)', sheet='A',
         nets=J6_NETS, names=J6_NAMES,
         note='5V USB, GND, 3V3, D10, D9, D8. Signaux 3,3 V maximum. 5V : USB seulement, pas de servo.'),
    dict(ref='J8', face='F', kind='header', pins=J8_PINS, value='Extension D0-D3 (Dupont)', sheet='A',
         nets=J8_NETS, names=J8_NAMES,
         note='D0 à D3, 3,3 V maximum. D2/D3 : broches de démarrage, rien qui les force à l’allumage.'),
    dict(ref='J7', face='F', kind='header', pins=J7_PINS, value='Modulino Knob (I2C, câble)', sheet='A',
         nets=['GND', '+3V3', 'I2C_SDA', 'I2C_SCL'], names=['GND', '3V3', 'SDA', 'SCL'],
         note='Même ordre que le connecteur 1x4 du Modulino Knob. Adresse bus 0x3A (0x76 en notation Arduino).'),
]
BYREF = {p['ref']: p for p in PARTS}
for _p in PARTS:
    if _p['kind'] == 'module':
        _p['pins'], _p['rect'], _p['usb'] = placed_module(_p)

# Appuis : entretoises nylon collées sur la face de la carte (aucun perçage).
SUPPORTS = [dict(ref='J3', d=4.0, label='OLED : 2 entretoises M2 x 11 mm sous les coins libres'),
            dict(ref='J1', d=5.5, label='GY-521 : 2 entretoises M3 x 11 mm (fixation rigide de l’IMU)'),
            dict(ref='J2', d=4.0, label='BMP280 : cale isolante 11 mm collée sous le bord libre')]


# Fixation de la plaque : 4 trous M3 aux coins. Dessous = entretoises (vers le
# châssis, au moins 18 mm), dessus = têtes de vis. Rien au ras de la plaque ne
# doit chevaucher ces cercles (les cartes surélevées à 11 mm passent au-dessus).
MOUNT_HOLES = [(2.5, 2.5), (BOARD_W - 2.5, 2.5), (2.5, BOARD_H - 2.5), (BOARD_W - 2.5, BOARD_H - 2.5)]
MOUNT_D = {'B': 6.4, 'F': 5.5}      # entretoise hexagonale M3 (sur angles) ; tête de vis M3


def mount_conflicts():
    out = []
    for p in PARTS:
        if not p['face']:
            continue
        low = list(socket_rects(p)) + ([body_rect(p)] if p['kind'] in ('resistor', 'capacitor', 'cpol') else [])
        for s_ in SUPPORTS:
            if s_['ref'] == p['ref']:
                low += [(x - s_['d'] / 2, y - s_['d'] / 2, x + s_['d'] / 2, y + s_['d'] / 2) for x, y in support_points(s_)]
        for (mx, my) in MOUNT_HOLES:
            r = MOUNT_D[p['face']] / 2
            for q_ in low:
                dx = max(q_[0] - mx, mx - q_[2], 0); dy = max(q_[1] - my, my - q_[3], 0)
                if math.hypot(dx, dy) < r:
                    out.append((p['ref'], p['face'], (mx, my), round(r - math.hypot(dx, dy), 2)))
    return out


def other(face):
    return 'B' if face == 'F' else 'F'


def plastic_rect(p):
    xs = [hxy(n)[0] for n in p['pins']]; ys = [hxy(n)[1] for n in p['pins']]
    return (min(xs) - 1.27, min(ys) - 1.27, max(xs) + 1.27, max(ys) + 1.27)


def socket_rects(p):
    """Bases plastiques au ras de la plaque (supports femelles, barrettes)."""
    if p['kind'] == 'module':
        groups = [p['pins'][:7], p['pins'][7:]] if p['module'] == 'XIAO' else [p['pins']]
        out = []
        for g in groups:
            xs = [hxy(n)[0] for n in g]; ys = [hxy(n)[1] for n in g]
            out.append((min(xs) - 1.27, min(ys) - 1.27, max(xs) + 1.27, max(ys) + 1.27))
        return out
    if p['kind'] in ('header', 'jumper'):
        return [plastic_rect(p)]
    return []


def body_rect(p):
    if p['kind'] == 'module':
        return p['rect']
    if p['kind'] in ('header', 'jumper'):
        return plastic_rect(p)
    (x1, y1), (x2, y2) = hxy(p['pins'][0]), hxy(p['pins'][-1])
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    horiz = abs(y1 - y2) < 1e-6
    if p['kind'] == 'resistor':
        w, h = (6.3, 2.5) if horiz else (2.5, 6.3)
    elif p['kind'] == 'cpol':
        w = h = 5.0
    else:
        w, h = (5.0, 2.5) if horiz else (2.5, 5.0)
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def support_points(s):
    p = BYREF[s['ref']]; r = p['rect']; inset = 2.5
    pins = [hxy(n) for n in p['pins']]
    px = sum(x for x, _ in pins) / len(pins); py = sum(y for _, y in pins) / len(pins)
    horizontal = len({n[1] for n in p['pins']}) == 1
    if p['ref'] == 'J2':
        far_x = r[0] + 2.2 if abs(px - r[2]) < abs(px - r[0]) else r[2] - 2.2
        return [(far_x, (r[1] + r[3]) / 2)]
    if horizontal:
        y = r[3] - inset if abs(py - r[1]) < abs(py - r[3]) else r[1] + inset
        return [(r[0] + inset, y), (r[2] - inset, y)]
    x = r[2] - inset if abs(px - r[0]) < abs(px - r[2]) else r[0] + inset
    return [(x, r[1] + inset), (x, r[3] - inset)]


def nodes_in(rect, margin=0.0):
    out = []
    for c in range(COLS):
        for r in range(ROWS):
            x, y = hxy((c, r))
            if rect[0] - margin < x < rect[2] + margin and rect[1] - margin < y < rect[3] + margin:
                out.append((c, r))
    return out


def blocked_faces():
    """Trous dont la pastille d'une face est recouverte (base plastique, corps couché, appui collé)."""
    blk = {f: set() for f in FACES}
    for p in PARTS:
        if not p['face']:
            continue
        for r in socket_rects(p):
            blk[p['face']] |= set(nodes_in(r))
        if p['kind'] in ('resistor', 'capacitor', 'cpol'):
            blk[p['face']] |= set(nodes_in(body_rect(p), 0.4)) | set(p['pins'])
        if p['kind'] in ('module', 'header', 'jumper') or True:
            blk[p['face']] |= set(p['pins'])     # pastille sous la base : non soudable de ce côté
    for s in SUPPORTS:
        f = BYREF[s['ref']]['face']
        for (x, y) in support_points(s):
            blk[f] |= set(nodes_in((x - s['d'] / 2, y - s['d'] / 2, x + s['d'] / 2, y + s['d'] / 2), 0.9))
    return blk


def blocked_half():
    """Nœuds au demi-pas recouverts sur chaque face (coordonnées doublées)."""
    blk = {f: set() for f in FACES}
    def cover(f, rect, margin=0.0):
        for i in range(-1, 2 * COLS):
            for j in range(-1, 2 * ROWS):
                x, y = X0 + PITCH * i / 2, Y0 + PITCH * j / 2
                if rect[0] - margin < x < rect[2] + margin and rect[1] - margin < y < rect[3] + margin:
                    blk[f].add((i, j))
    for p in PARTS:
        if not p['face']:
            continue
        for r in socket_rects(p):
            cover(p['face'], r)
        if p['kind'] in ('resistor', 'capacitor', 'cpol'):
            cover(p['face'], body_rect(p), 0.4)
        for n in p['pins']:
            blk[p['face']].add((2 * n[0], 2 * n[1]))
    for s in SUPPORTS:
        f = BYREF[s['ref']]['face']
        for (x, y) in support_points(s):
            cover(f, (x - s['d'] / 2, y - s['d'] / 2, x + s['d'] / 2, y + s['d'] / 2), 0.9)
    # bord de plaque : rester à l'intérieur de la grille de trous (± un demi-pas)
    return blk


def gap(a, b):
    dx = max(a[0] - b[2], b[0] - a[2], 0); dy = max(a[1] - b[3], b[1] - a[3], 0)
    return -1.0 if dx == 0 and dy == 0 else math.hypot(dx, dy)



# ------------------------------------------------------------------ routage
NET_ORDER = ['I2C_SDA', 'I2C_SCL', 'GND', '+3V3', 'VBUS_USB', 'GNSS_TX', 'GNSS_RX',
             'EXT_D0', 'EXT_D1', 'EXT_D2', 'EXT_D3', 'EXT_D8', 'EXT_D9', 'EXT_D10']
WIRE_FACE = 'B'                 # tous les fils ; face F : ponts de soudure seulement
BEND_COST = 4.0
INSULATED_STEP = 2.5            # un pas entre pastilles coûte plus qu'un pas sur une ligne de trous
CROSS_COST = 25.0               # croisement par-dessus un autre fil : à éviter autant que possible
# Rails : lignes communes en fil nu, depuis les broches de l'écran, entre les deux supports du XIAO.
# En bas, SDA, SCL et GND tournent en L jusqu'aux broches du GY-521 (sans se croiser).
RAILS = {'GND': [(5, 0), (5, 16), (2, 16), (2, 19)], '+3V3': [(6, 0), (6, 15)],
         'I2C_SCL': [(7, 0), (7, 17), (3, 17), (3, 19)], 'I2C_SDA': [(8, 0), (8, 18), (4, 18), (4, 19)]}
# Ponts de soudure (face dessus) : broche du XIAO -> trou(s) voisin(s). 'out' = côté extérieur du support.
BRIDGES = {'5V': ('out',), 'GND': ('out', 'in'), '3V3': ('out', 'in'), 'D10': ('out',), 'D9': ('out',), 'D8': ('out',),
           'D7': ('out',), 'D0': ('out',), 'D1': ('out',), 'D2': ('out',), 'D3': ('out',), 'D4': ('in',), 'D5': ('out',),
           'D6': ('out',)}


def bridge_holes(p, n, name):
    """Trous voisins reliés par pont de soudure à une broche du XIAO."""
    cols = sorted({c for c, _ in p['pins']})
    out = 1 if n[0] == cols[-1] else -1
    return [(n[0] + (out if side == 'out' else -out), n[1]) for side in BRIDGES[name]]


def expand(waypoints):
    """Trous (colonne, rangée) en ligne droite -> nœuds au demi-pas."""
    nodes = [(2 * waypoints[0][0], 2 * waypoints[0][1])]
    for (c, r) in waypoints[1:]:
        tx, ty = 2 * c, 2 * r
        while nodes[-1] != (tx, ty):
            x, y = nodes[-1]
            nodes.append((x + (tx > x) - (tx < x), y + (ty > y) - (ty < y)))
    return nodes


def axis_of(a, b):
    return 'h' if a[1] == b[1] else 'v'


def node_axes(nodes):
    """Pour chaque nœud entre pastilles d'un fil : axes occupés (un seul si le fil passe tout droit)."""
    out = {}
    for j in range(1, len(nodes) - 1):
        k = nodes[j]
        if not Router.is_hole(k):
            out[k] = {axis_of(nodes[j - 1], k), axis_of(k, nodes[j + 1])}
    return out


class Router:
    """Routage négocié sur la face dessous, grille au demi-pas.

    Coordonnées doublées : (2c, 2r) = trou ; les autres nœuds sont entre les
    pastilles. Un trou appartient à un seul net. Un nœud entre pastilles peut
    être partagé par deux nets s'ils le traversent tout droit à angle droit
    (croisement : le fil du dessus est forcément isolé, car il passe entre deux
    pastilles). Un fil qui ne touche que des lignes de trous est un fil nu.
    Les broches du XIAO (soudables seulement dessus) sont reliées par un pont
    de soudure, face dessus, au trou voisin, qui sert de départ au fil.
    """

    def __init__(self):
        self.blk = blocked_half()
        self.pin, self.term, self.bridges, self.group = {}, {}, [], {}
        for p in PARTS:
            if not p['face']:
                continue
            for i, (n, net) in enumerate(zip(p['pins'], p['nets']), 1):
                k = (2 * n[0], 2 * n[1])
                assert k not in self.pin, ('deux broches dans un trou', n)
                self.pin[k] = (p['ref'], i, net)
        self.rails = []
        for net, wp in RAILS.items():
            nodes = expand(wp)
            for k in nodes:
                assert k not in self.blk[WIRE_FACE] or k in self.pin, ('rail sur une zone recouverte', net, k)
                assert self.net_at(k) in (None, net), ('rail sur une broche d’un autre net', net, k)
            self.rails.append(dict(net=net, face=WIRE_FACE, nodes=nodes, rail=True))
            self._join([k for k in nodes if self.is_hole(k)], net)
        for p in PARTS:
            if not p['face']:
                continue
            for i, (n, net) in enumerate(zip(p['pins'], p['nets']), 1):
                k = (2 * n[0], 2 * n[1])
                if not net:
                    continue
                if k not in self.blk[WIRE_FACE]:
                    self.term[k] = net
                    continue
                assert p['face'] == WIRE_FACE and k not in self.blk['F'], ('broche inaccessible', p['ref'], hole_name(n))
                same = []
                for nb in bridge_holes(p, n, p['names'][i - 1]):
                    kb = (2 * nb[0], 2 * nb[1])
                    mid = ((k[0] + kb[0]) // 2, k[1])
                    assert kb not in self.pin and self.term.get(kb, net) == net, ('pont vers un trou occupé', p['ref'], hole_name(nb))
                    assert kb not in self.blk['F'] and mid not in self.blk['F'] and kb not in self.blk[WIRE_FACE], \
                        ('pont impossible', p['ref'], hole_name(nb))
                    self.term[kb] = net
                    same.append(nb)
                # un seul pont par broche : goutte sur 2 pastilles, ou 3 si la broche est pontée des deux côtés
                wp = [same[0], n] + same[1:] if len(same) > 1 else [n, same[0]]
                self.bridges.append(dict(net=net, face='F', nodes=expand(wp), bridge=True, ref=p['ref'], pin=i))
                self._join([(2 * c, 2 * r) for c, r in same], net)
        for r in self.rails:
            for k in r['nodes']:
                if self.is_hole(k):
                    self.term.setdefault(k, r['net'])
        self.hist, self.routes, self.failed = {}, {}, []

    def _join(self, nodes, net):
        """Trous déjà reliés entre eux (rail, ou ponts d'une même broche)."""
        g = set(nodes)
        for k in nodes:
            g |= self.group.get(k, set())
        for k in g:
            self.group[k] = g

    @staticmethod
    def is_hole(k):
        return k[0] % 2 == 0 and k[1] % 2 == 0

    def net_at(self, k):
        if k in self.pin:
            return self.pin[k][2]
        return self.term.get(k)

    def usage(self, skip=None):
        u = {}
        for net, ts in self.routes.items():
            if net == skip:
                continue
            for t in ts:
                ax = node_axes(t['nodes'])
                for k in t['nodes']:
                    keys = [k] if self.is_hole(k) else [(k, a) for a in ax.get(k, ('h', 'v'))]
                    for key in keys:
                        u.setdefault(key, set()).add(net)
        return u

    def route_net(self, net, pres):
        use = self.usage(skip=net)
        pins, seen_g = [], set()
        for k in sorted(k for k, v in self.term.items() if v == net):
            g = frozenset(self.group.get(k, {k}))
            if g not in seen_g:
                seen_g.add(g); pins.append(k)
        best = None
        for root in sorted(pins, key=lambda n: -sum(abs(n[0] - m[0]) + abs(n[1] - m[1]) for m in pins)):
            res = self._tree(net, root, pins, use, pres)
            if best is None or (len(res[1]), res[2]) < (len(best[1]), best[2]):
                best = res
            if not res[1]:
                break
        return best[0], best[1]

    def _tree(self, net, root, pins, use, pres):
        rails = [r for r in self.rails if r['net'] == net]
        tree = set(self.group.get(root, {root}))
        if rails:                                      # le rail est le tronc : on part de lui
            tree = {k for r in rails for k in r['nodes']}
            for k in list(tree):
                tree |= self.group.get(k, set())
        occ = {k for r in rails for k in r['nodes']}
        ends = {}
        for r in rails:
            for e in (r['nodes'][0], r['nodes'][-1]):
                ends[e] = ends.get(e, 0) + 1
        traces, failed, cost = [dict(r) for r in rails], [], 0.0
        todo = [p for p in pins if p not in tree]
        while todo:
            todo.sort(key=lambda p: min(abs(p[0] - t[0]) + abs(p[1] - t[1]) for t in tree if self.is_hole(t)))
            p = todo.pop(0)
            if p in tree:
                continue
            starts = [q for q in self.group.get(p, {p}) if q not in self.blk[WIRE_FACE]]
            goals = {k for k in tree if self.is_hole(k) and k not in self.blk[WIRE_FACE] and not ends.get(k)}
            res = None
            for q in sorted(starts):
                r_ = self.search(net, q, goals, tree, occ, ends, use, pres)
                if r_ and (res is None or r_[1] < res[1]):
                    res = r_
            if res is None:
                failed.append((net, hole_name((p[0] // 2, p[1] // 2)))); continue
            path, c = res
            cost += c
            occ.update(path); tree.update(path); tree |= self.group.get(p, set())
            for e in (path[0], path[-1]):
                ends[e] = ends.get(e, 0) + 1
            traces.append(dict(net=net, face=WIRE_FACE, nodes=list(path)))
        return traces, failed, cost

    def search(self, net, start, goals, tree, occ, ends, use, pres):
        if not goals or start in self.blk[WIRE_FACE] or ends.get(start):
            return None
        def h(n):
            return min(abs(n[0] - t[0]) + abs(n[1] - t[1]) for t in goals) * 0.5
        def ncost(key):
            return self.hist.get(key, 0.0) + pres * len(use.get(key, ()))
        q, seen, tick = [], {}, iter(range(10 ** 9))
        heapq.heappush(q, (h(start), 0.0, next(tick), start, None, (start,)))
        W, H = 2 * (COLS - 1), 2 * (ROWS - 1)
        blk = self.blk[WIRE_FACE]
        while q:
            _, g, _, n, d, path = heapq.heappop(q)
            if len(path) > 1 and n in goals:
                return path, g
            if seen.get((n, d), 1e18) <= g:
                continue
            seen[(n, d)] = g
            for dd in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                m = (n[0] + dd[0], n[1] + dd[1])
                if not (-1 <= m[0] <= W + 1 and -1 <= m[1] <= H + 1) or m in blk or m in path:
                    continue
                hole = self.is_hole(m)
                if m not in goals:
                    if m in tree or m in occ:
                        continue
                    if hole and (self.net_at(m) not in (None, net) or (m in self.pin and self.pin[m][2] != net)):
                        continue                         # broche d'un autre net, ou broche non câblée
                ax = 'h' if dd[1] == 0 else 'v'
                on_grid = (n[1] % 2 == 0) if dd[1] == 0 else (n[0] % 2 == 0)   # le long d'une ligne de trous
                cost = g + (0.5 if on_grid else INSULATED_STEP)
                if d and d != dd:
                    cost += BEND_COST
                    if not self.is_hole(n):                  # virage entre pastilles : occupe aussi l'autre axe
                        cost += ncost((n, ax))
                if hole:
                    cost += ncost(m)
                else:
                    cost += ncost((m, ax))
                    if use.get((m, 'v' if ax == 'h' else 'h')):
                        cost += CROSS_COST
                heapq.heappush(q, (cost + h(m), cost, next(tick), m, dd, path + (m,)))
        return None

    def run(self, iterations=80):
        pres = 0.3
        for it in range(iterations):
            self.failed = []
            for net in NET_ORDER:
                self.routes[net], f = self.route_net(net, pres)
                self.failed += f
            u = self.usage()
            conflicts = [k for k, nets in u.items() if len(nets) > 1]
            if not conflicts and not self.failed:
                break
            for k in conflicts:
                self.hist[k] = self.hist.get(k, 0.0) + 0.6
            pres = min(pres * 1.25, 500)
        self.iterations = it + 1
        if conflicts:
            self.failed += [('conflit', str(k)) for k in conflicts]
        self.traces = []
        for net in NET_ORDER:
            for t in self.routes[net]:
                t['bare'] = all(((a[1] % 2 == 0) if a[1] == b[1] else (a[0] % 2 == 0)) for a, b in zip(t['nodes'], t['nodes'][1:]))
                self.traces.append(t)
        for i, t in enumerate(self.traces, 1):
            t['id'] = f'P{i:02}'
        for i, b in enumerate(self.bridges, 1):
            b['bare'] = True
            b['id'] = f'S{i:02}'
        self.traces += self.bridges
        for t in self.traces:
            t['path'] = [(k[0] / 2, k[1] / 2) for k in t['nodes']]
        self._layers()
        return self

    def _layers(self):
        """Fils isolés : un calque KiCad par « étage » pour que deux fils isolés qui se croisent
        soient sur des calques différents (représentation seulement : physiquement, l'un passe sur l'autre)."""
        self.crossings = []
        at = {}
        for t in self.traces:
            if t.get('bridge'):
                t['layer'] = 0
                continue
            for k, ax in node_axes(t['nodes']).items():
                at.setdefault(k, []).append(t)
        for k, ts in at.items():
            if len(ts) == 2:
                self.crossings.append((k, ts[0]['id'], ts[1]['id']))
        adj = {}
        for _, a, b in self.crossings:
            adj.setdefault(a, set()).add(b); adj.setdefault(b, set()).add(a)
        for t in self.traces:
            if t.get('bridge'):
                continue
            if t['bare']:
                t['layer'] = 0
                continue
            used = {u['layer'] for u in self.traces if u['id'] in adj.get(t['id'], ()) and not u['bare'] and 'layer' in u}
            t['layer'] = next(i for i in range(1, 10) if i not in used)


def route():
    return Router().run()


def analyse(R=None):
    """Contrôles complets ; renvoie (router, rapport). Lève AssertionError."""
    R = R or route()
    assert not R.failed, ('liaisons non routées ou en conflit', R.failed[:6])
    blk = blocked_half()
    hole = Router.is_hole
    owner, ends, between = {}, {}, {}
    for k, (ref, i, net) in R.pin.items():
        owner[k] = net or ('non câblé', ref, i)       # aucun fil ne traverse une broche non câblée
    for t in R.traces:
        f = t['face']
        assert len(t['nodes']) >= 2
        if f == 'F':
            assert t.get('bridge') and len(t['nodes']) in (3, 5), ('fil sur la face dessus', t['id'])
        for a_, b_ in zip(t['nodes'], t['nodes'][1:]):
            assert abs(a_[0] - b_[0]) + abs(a_[1] - b_[1]) == 1, (t['id'], a_, b_)
        assert hole(t['nodes'][0]) and hole(t['nodes'][-1]), ('extrémité hors trou', t['id'])
        for k in t['nodes']:
            assert k not in blk[f], ('piste sur une zone recouverte', t['id'], k, f)
            if hole(k):
                assert owner.setdefault(k, t['net']) == t['net'], ('court-circuit au trou', t['id'], k)
        for k, ax in node_axes(t['nodes']).items():
            between.setdefault((k, f), []).append((t, ax))
        for e in (t['nodes'][0], t['nodes'][-1]):
            ends[(e, f)] = ends.get((e, f), 0) + 1
    assert max(ends.values()) == 1, [k for k, v in ends.items() if v > 1]
    crossings = 0
    for (k, f), lst in between.items():
        if len(lst) == 1:
            continue
        assert len(lst) == 2, ('plus de deux fils au même endroit', k)
        (t1, a1), (t2, a2) = lst
        assert t1['net'] != t2['net'] and len(a1) == 1 and len(a2) == 1 and a1 != a2, ('fils superposés', t1['id'], t2['id'], k)
        assert not (t1['bare'] and t2['bare']), ('deux fils nus se croisent', t1['id'], t2['id'])
        crossings += 1
    passes = {}
    for t in R.traces:
        for k in t['nodes'][1:-1]:
            if hole(k):
                assert (k, t['face']) not in passes, ('deux pistes traversent un trou', t['id'])
                passes[(k, t['face'])] = t['id']
    for net in NET_ORDER:
        adj = {}
        for t in R.traces:
            if t['net'] == net:
                for a_, b_ in zip(t['nodes'], t['nodes'][1:]):
                    adj.setdefault(a_, set()).add(b_); adj.setdefault(b_, set()).add(a_)
        pins = {k for k, v in R.pin.items() if v[2] == net}
        start = next(iter(pins)); comp = {start}; todo = [start]
        while todo:
            u = todo.pop()
            for v in adj.get(u, ()):
                if v not in comp:
                    comp.add(v); todo.append(v)
        assert pins <= comp, ('net non relié', net)
    gaps = []
    for f in FACES:
        rects = [(p['ref'], body_rect(p)) for p in PARTS if p['face'] == f]
        for s_ in SUPPORTS:
            if BYREF[s_['ref']]['face'] == f:
                for i, (x, y) in enumerate(support_points(s_)):
                    rects.append((f'{s_["ref"]}_appui{i + 1}', (x - s_['d'] / 2, y - s_['d'] / 2, x + s_['d'] / 2, y + s_['d'] / 2)))
        for i, (ra, a_) in enumerate(rects):
            lim = 2.0 if BYREF.get(ra, {}).get('kind') == 'module' else 0.0
            assert a_[0] >= -lim and a_[1] >= -lim and a_[2] <= BOARD_W + lim and a_[3] <= BOARD_H + lim, (ra, a_)
            for rb, b_ in rects[i + 1:]:
                if ra.split('_')[0] == rb.split('_')[0]:
                    continue
                g = gap(a_, b_)
                assert g >= 0.5, ('encombrements trop proches', f, ra, rb, round(g, 2))
                gaps.append((round(g, 2), f, ra, rb))
    gaps.sort()
    mc = mount_conflicts()
    assert not mc, ('fixation de la plaque gênée', mc)
    wires = [t for t in R.traces if not t.get('bridge')]
    rep = dict(wires=len(wires), bare=sum(t['bare'] for t in wires), insulated=sum(not t['bare'] for t in wires),
               wires_face_top=sum(t['face'] == 'F' for t in wires), solder_bridges_top=len(R.bridges),
               bridge_holes=[hole_name((b['nodes'][-1][0] // 2, b['nodes'][-1][1] // 2)) for b in R.bridges],
               crossings=crossings, insulated_layers=max([t['layer'] for t in wires] + [0]),
               wire_ends=sum(v for (k, f), v in ends.items() if f == WIRE_FACE),
               max_wire_ends_per_pad=max(ends.values()), holes_used=sum(1 for k in owner if hole(k)),
               wire_length_mm=round(sum((len(t['nodes']) - 1) * PITCH / 2 for t in wires)),
               router_iterations=R.iterations, smallest_gaps=gaps[:6])
    return R, rep


if __name__ == '__main__':
    import json
    print(json.dumps(analyse()[1], ensure_ascii=False, indent=1))
