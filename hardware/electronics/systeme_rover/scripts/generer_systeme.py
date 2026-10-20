#!/usr/bin/env python3
"""Schéma électrique complet du rover prototype FieldMind (KiCad 10, sans PCB).

Feuilles : vue d'ensemble (racine), puissance et arrêt, Raspberry Pi et USB,
carte capteurs A7 et périphériques. Symboles des bibliothèques KiCad standard
(batterie, fusibles, interrupteurs, relais Finder, moteurs, servo, fiche USB-C,
alimentations) ; blocs locaux seulement pour les modules absents des bibliothèques
(Pi 4 côté ports, XIAO sur sa carte A7, XL4015, webcams, lidar, GPS, Knob,
contrôleur de traction du donneur).

Écrit hardware/electronics/systeme_rover/{cad,exports}. Sources au format KiCad 9,
converties par `kicad-cli sch upgrade`. Refuse d'écraser une version enregistrée
par KiCad sans --force.
"""
import json
import math
import re
import subprocess
import sys
from pathlib import Path
from uuid import NAMESPACE_URL, uuid5

BASE = Path(__file__).resolve().parents[1]
CAD = BASE / 'cad'
OUT = BASE / 'exports'
PROJECT = 'fieldmind_rover_systeme'
LIB = 'FieldMind_Rover'
KICAD_CLI = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
KICAD_SYMBOLS = Path('/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols')
DATE = '2026-10-13'
REV = 'S1 / non testé'


def uid(key):
    return str(uuid5(NAMESPACE_URL, 'fieldmind/rover-systeme/' + key))


def q(s):
    return json.dumps(str(s), ensure_ascii=False)


def fmt(v):
    return f'{round(v, 4):g}'


def eff(size=1.27, justify='', hide=False):
    return f'(effects (font (size {size} {size})){" (justify " + justify + ")" if justify else ""}{" (hide yes)" if hide else ""})'


ROOT = uid('sheet/root')
SHEETS = {  # clé : (nom, fichier, page)
    'P': ('Puissance et arrêt', 'puissance_arret.kicad_sch', 2),
    'C': ('Raspberry Pi et USB', 'pi_usb.kicad_sch', 3),
    'S': ('Carte capteurs et périphériques', 'capteurs_peripheriques.kicad_sch', 4),
}
SHEET_UUID = {k: uid('sheet/' + k) for k in SHEETS}


def sheet_path(key):
    return '/' + ROOT if key == 'R' else f'/{ROOT}/{SHEET_UUID[key]}'


# ====================================================================== symboles
def sexpr_block(text, start):
    depth, j = 0, start
    while True:
        c = text[j]
        if c == '"':
            j += 1
            while text[j] != '"':
                j += 2 if text[j] == '\\' else 1
        elif c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return text[start:j + 1]
        j += 1


def std_symbol(lib, name):
    text = (KICAD_SYMBOLS / f'{lib}.kicad_sym').read_text()
    i = text.find(f'(symbol "{name}"')
    assert i >= 0, (lib, name)
    block = sexpr_block(text, i)
    assert '(extends' not in block, (lib, name)
    return block


def lib_pins(block):
    pins = {}
    for m in re.finditer(r'\(pin \w+ \w+\s*\(at ([-\d.]+) ([-\d.]+) (\d+)\).*?\(number "([^"]*)"', block, re.S):
        pins.setdefault(m.group(4), (float(m.group(1)), float(m.group(2))))
    return pins


def block_symbol(name, ref, pins, w=10.16, top=None, bottom=None, title=None):
    """Bloc rectangulaire. pins = [(num, nom, 'L'|'R', y)] (y vers le haut, multiple de 2,54)."""
    ys = [p[3] for p in pins]
    top = top if top is not None else max(ys) + 2.54
    bottom = bottom if bottom is not None else min(ys) - 2.54
    s = [f'(symbol {q(name)} (pin_names (offset 0.762)) (exclude_from_sim no) (in_bom yes) (on_board yes)',
         f'(property "Reference" {q(ref)} (at {fmt(-w)} {fmt(top + 1.27)} 0) {eff(justify="left")})',
         f'(property "Value" {q(title or name)} (at {fmt(-w)} {fmt(bottom - 1.27)} 0) {eff(justify="left")})',
         f'(property "Footprint" "" (at 0 0 0) {eff(hide=True)})',
         f'(property "Datasheet" "" (at 0 0 0) {eff(hide=True)})',
         f'(property "Description" "" (at 0 0 0) {eff(hide=True)})',
         f'(symbol "{name}_0_1" (rectangle (start {fmt(-w)} {fmt(top)}) (end {fmt(w)} {fmt(bottom)}) '
         f'(stroke (width 0.254) (type default)) (fill (type background))))',
         f'(symbol "{name}_1_1"']
    for num, label, side, y in pins:
        x, a = (-w - 5.08, 0) if side == 'L' else (w + 5.08, 180)
        label = label.replace(' ', '\u00a0')       # KiCad remplace les espaces par « _ » dans les noms de broches
        s.append(f'(pin passive line (at {fmt(x)} {fmt(y)} {a}) (length 5.08) (name {q(label)} {eff()}) (number {q(num)} {eff()}))')
    return '\n'.join(s + ['))'])


def col(n, start=0.0):
    """n positions de haut en bas, pas 2,54, à partir de start."""
    return [start - 2.54 * i for i in range(n)]


CUSTOM = {}


def custom(key, ref, pins, **kw):
    CUSTOM[key] = block_symbol(key, ref, pins, **kw)


# Raspberry Pi 4 vu par ses ports (le connecteur GPIO 40 broches n'est pas utilisé)
custom('RPi4_Ports', 'U', [('PWR', '5V (USB-C)', 'L', 2.54), ('GND', 'GND (USB-C)', 'L', 0.0),
                           ('USB3A', 'USB 3.0 n°1 (bleu)', 'R', 5.08), ('USB3B', 'USB 3.0 n°2 (bleu)', 'R', 2.54),
                           ('USB2A', 'USB 2.0 n°1', 'R', -2.54), ('USB2B', 'USB 2.0 n°2', 'R', -5.08),
                           ('WIFI', 'Wi-Fi maison', 'R', -10.16)], w=17.78, top=10.16, bottom=-12.7)
custom('Module_XL4015', 'U', [('1', 'IN+', 'L', 1.27), ('2', 'IN-', 'L', -1.27), ('3', 'OUT+', 'R', 1.27),
                              ('4', 'OUT-', 'R', -1.27)], w=7.62, top=3.81, bottom=-3.81)
custom('Webcam_USB', 'U', [('USB', 'USB-A', 'L', 0.0)], w=10.16, top=2.54, bottom=-2.54)
custom('RPLIDAR_C1', 'U', [('1', 'VCC (rouge)', 'R', 2.54), ('2', 'TX (jaune)', 'R', 0.0),
                           ('3', 'RX (vert)', 'R', -2.54), ('4', 'GND (noir)', 'R', -5.08)], w=10.16, top=5.08, bottom=-7.62)
custom('Adaptateur_USB_C1', 'U', [('1', '5V', 'L', 2.54), ('2', 'RX', 'L', 0.0), ('3', 'TX', 'L', -2.54),
                                  ('4', 'GND', 'L', -5.08), ('USB', 'USB', 'R', -1.27)], w=10.16, top=5.08, bottom=-7.62)
custom('ATGM336H', 'U', [('RXD', 'RXD', 'L', 5.08), ('GND', 'GND', 'L', 2.54), ('VCC', 'VCC', 'L', 0.0),
                         ('TXD', 'TXD', 'L', -2.54), ('PPS', 'PPS', 'L', -5.08)], w=10.16, top=7.62, bottom=-7.62)
custom('Modulino_Knob', 'U', [('1', 'GND', 'L', 2.54), ('2', '3.3V', 'L', 0.0), ('3', 'SDA', 'L', -2.54),
                              ('4', 'SCL', 'L', -5.08)], w=10.16, top=3.81, bottom=-7.62)
custom('Controleur_Traction', 'U', [('V+', 'V+ (batterie)', 'L', 1.27), ('GND', 'GND', 'L', -1.27),
                                    ('M1A', 'Moteur 1', 'R', 6.35), ('M1B', 'Moteur 1', 'R', 3.81),
                                    ('M2A', 'Moteur 2', 'R', 1.27), ('M2B', 'Moteur 2', 'R', -1.27),
                                    ('CMD', 'Commande', 'R', -3.81), ('RET', 'Retours roues', 'R', -6.35)],
       w=12.7, top=8.89, bottom=-8.89)
BOARD_PINS = [('USB', 'USB-C du XIAO', 'L', 0.0)]
for i, (n, y) in enumerate([('J4.1 RX_GPS (D6)', 22.86), ('J4.2 GND', 20.32), ('J4.3 3V3', 17.78), ('J4.4 TX_GPS (D7)', 15.24)]):
    BOARD_PINS.append((f'J4.{i + 1}', n, 'R', y))
for i, (n, y) in enumerate([('J7.1 GND', 7.62), ('J7.2 3V3', 5.08), ('J7.3 SDA (D4)', 2.54), ('J7.4 SCL (D5)', 0.0)]):
    BOARD_PINS.append((f'J7.{i + 1}', n, 'R', y))
for i, (n, y) in enumerate([('J6.1 5V USB', -5.08), ('J6.2 GND', -7.62), ('J6.3 3V3', -10.16), ('J6.4 D10', -12.7),
                            ('J6.5 D9', -15.24), ('J6.6 D8', -17.78)]):
    BOARD_PINS.append((f'J6.{i + 1}', n, 'R', y))
for i, (n, y) in enumerate([('J8.1 D0', -22.86), ('J8.2 D1', -25.4), ('J8.3 D2', -27.94), ('J8.4 D3', -30.48)]):
    BOARD_PINS.append((f'J8.{i + 1}', n, 'R', y))
custom('Carte_Capteurs_A7', 'A', BOARD_PINS, w=20.32, top=27.94, bottom=-35.56)

STD = {'Battery': ('Device', 'Battery'), 'Fuse': ('Device', 'Fuse'), 'SW_SPST': ('Switch', 'SW_SPST'),
       'SW_Push_Open': ('Switch', 'SW_Push_Open'), 'SW_Push': ('Switch', 'SW_Push'),
       'FINDER-40.52': ('Relay', 'FINDER-40.52'), 'FINDER-40.11': ('Relay', 'FINDER-40.11'),
       'Motor_DC': ('Motor', 'Motor_DC'), 'Motor_Servo': ('Motor', 'Motor_Servo'),
       'USB_C_Plug_USB2.0': ('Connector', 'USB_C_Plug_USB2.0'), 'GND': ('power', 'GND'), 'PWR_FLAG': ('power', 'PWR_FLAG')}
SYMS = {}
for key, (lib, name) in STD.items():
    SYMS[key] = (f'{lib}:{name}', std_symbol(lib, name))
for key, block in CUSTOM.items():
    SYMS[key] = (f'{LIB}:{key}', block)
SYMPINS = {k: lib_pins(v[1]) for k, v in SYMS.items()}


def lib_entry(key):
    full, block = SYMS[key]
    name = full.split(':', 1)[1]
    return block.replace(f'(symbol "{name}"', f'(symbol "{full}"', 1)


# ====================================================================== feuilles
class Sheet:
    def __init__(self, key, title):
        self.key, self.title, self.items, self.n, self.used = key, title, [], 0, set()

    def ident(self, kind):
        self.n += 1
        return uid(f'{self.key}/{kind}/{self.n}')

    def wire(self, *pts):
        for a, b in zip(pts, pts[1:]):
            assert a != b and (abs(a[0] - b[0]) < 1e-6 or abs(a[1] - b[1]) < 1e-6), (a, b)
            self.items.append(f'(wire (pts (xy {fmt(a[0])} {fmt(a[1])}) (xy {fmt(b[0])} {fmt(b[1])})) '
                              f'(stroke (width 0) (type default)) (uuid {self.ident("w")}))')

    def junction(self, pt):
        self.items.append(f'(junction (at {fmt(pt[0])} {fmt(pt[1])}) (diameter 0) (color 0 0 0 0) (uuid {self.ident("j")}))')

    def nc(self, pt):
        self.items.append(f'(no_connect (at {fmt(pt[0])} {fmt(pt[1])}) (uuid {self.ident("nc")}))')

    def glabel(self, net, pt, angle, shape='bidirectional'):
        just = {0: 'left', 180: 'right', 90: 'left', 270: 'right'}[angle]
        self.items.append(f'(global_label {q(net)} (shape {shape}) (at {fmt(pt[0])} {fmt(pt[1])} {angle}) '
                          f'(fields_autoplaced yes) {eff(justify=just)} (uuid {self.ident("gl")}) '
                          f'(property "Intersheetrefs" "${{INTERSHEET_REFS}}" (at {fmt(pt[0])} {fmt(pt[1])} 0) {eff(hide=True)}))')

    def label(self, net, pt, angle=0):
        just = 'left bottom' if angle == 0 else 'right bottom'
        self.items.append(f'(label {q(net)} (at {fmt(pt[0])} {fmt(pt[1])} {angle}) {eff(justify=just)} (uuid {self.ident("l")}))')

    def text(self, t, x, y, size=1.27, bold=False):
        b = f'(effects (font (size {size} {size}){" (bold yes)" if bold else ""}) (justify left bottom))'
        self.items.append(f'(text {q(t)} (exclude_from_sim no) (at {fmt(x)} {fmt(y)} 0) {b} (uuid {self.ident("t")}))')

    def box(self, x0, y0, x1, y1):
        self.items.append(f'(rectangle (start {fmt(x0)} {fmt(y0)}) (end {fmt(x1)} {fmt(y1)}) '
                          f'(stroke (width 0.2) (type dash) (color 72 72 72 1)) (fill (type none)) (uuid {self.ident("r")}))')

    def place(self, key, ref, value, x, y, angle=0, mirror=False, ref_at=None, val_at=None, just='', in_bom=True, hide_value=False):
        self.used.add(key)
        power = ref.startswith('#')
        rx, ry = ref_at or (x + 3.81, y - 2.54)
        vx, vy = val_at or (x + 3.81, y + 2.54)
        fa = 90 if angle in (90, 270) else 0
        pins = {}
        for num, (px, py) in SYMPINS[key].items():
            if mirror:
                px = -px
            a = math.radians(angle)
            qx, qy = px * math.cos(a) - py * math.sin(a), px * math.sin(a) + py * math.cos(a)
            pins[num] = (round(x + qx, 4), round(y - qy, 4))
        pin_uuids = ' '.join(f'(pin {q(n)} (uuid {uid(f"{self.key}/{ref}/pin/{n}")}))' for n in SYMPINS[key])
        self.items.append(
            f'(symbol (lib_id {q(SYMS[key][0])}) (at {fmt(x)} {fmt(y)} {angle}){" (mirror y)" if mirror else ""} (unit 1) '
            f'(exclude_from_sim no) (in_bom {"yes" if in_bom else "no"}) (on_board yes) (dnp no) '
            f'(uuid {uid(f"{self.key}/sym/{ref}")}) '
            f'(property "Reference" {q(ref)} (at {fmt(rx)} {fmt(ry)} {fa}) {eff(justify=just, hide=power)}) '
            f'(property "Value" {q(value)} (at {fmt(vx)} {fmt(vy)} {fa}) {eff(justify=just, hide=hide_value)}) '
            f'(property "Footprint" "" (at {fmt(x)} {fmt(y)} 0) {eff(hide=True)}) '
            f'(property "Datasheet" "" (at {fmt(x)} {fmt(y)} 0) {eff(hide=True)}) '
            f'{pin_uuids} (instances (project {q(PROJECT)} (path {q(sheet_path(self.key))} (reference {q(ref)}) (unit 1)))))')
        return pins

    def gnd(self, pt, angle=0):
        self.n += 1
        self.place('GND', f'#PWR{self.key}{self.n:03}', 'GND', pt[0], pt[1], angle, in_bom=False, val_at=(pt[0], pt[1] + 3.81), hide_value=True)

    def flag(self, pt, angle=0):
        self.n += 1
        self.place('PWR_FLAG', f'#FLG{self.key}{self.n:03}', 'PWR_FLAG', pt[0], pt[1], angle, in_bom=False,
                   val_at=(pt[0], pt[1] - 3.81), hide_value=True)

    def render(self, sheets=()):
        s = [f'(kicad_sch (version 20250114) (generator "fieldmind_rover") (generator_version "s1") '
             f'(uuid {ROOT if self.key == "R" else uid("file/" + self.key)}) (paper "A3")',
             f'(title_block (title {q(self.title)}) (date {q(DATE)}) (rev {q(REV)}) (company "FieldMind")'
             f' (comment 1 "Schéma de câblage du prototype : modules et liaisons, sans routage")'
             f' (comment 2 "Batterie, contrôleur et moteurs du donneur E600 : à identifier au démontage"))',
             '(lib_symbols', '\n'.join(lib_entry(k) for k in sorted(self.used)), ')']
        s += self.items
        for name, file, sid, x, y, w, h, page in sheets:
            s.append(f'(sheet (at {fmt(x)} {fmt(y)}) (size {fmt(w)} {fmt(h)}) (exclude_from_sim no) (in_bom yes) '
                     f'(on_board yes) (dnp no) (fields_autoplaced yes) (stroke (width 0.1524) (type solid)) '
                     f'(fill (color 255 255 225 1.0000)) (uuid {sid}) '
                     f'(property "Sheetname" {q(name)} (at {fmt(x)} {fmt(y - 0.7)} 0) {eff(1.8, justify="left bottom")}) '
                     f'(property "Sheetfile" {q(file)} (at {fmt(x)} {fmt(y + h + 0.6)} 0) {eff(justify="left top")}) '
                     f'(instances (project {q(PROJECT)} (path {q("/" + ROOT)} (page {q(page)})))))')
        if self.key == 'R':
            s.append('(sheet_instances (path "/" (page "1")))')
        return '\n'.join(s + [')']) + '\n'


def hwire(sh, a, b):
    """Liaison en L : horizontale puis verticale."""
    if abs(a[0] - b[0]) < 1e-6 or abs(a[1] - b[1]) < 1e-6:
        sh.wire(a, b)
    else:
        sh.wire(a, (b[0], a[1]), b)


# ---------------------------------------------------------------- puissance et arrêt
def sheet_power():
    P = Sheet('P', 'FieldMind rover / puissance et arrêt d’urgence')
    P.text('PUISSANCE ET ARRÊT D’URGENCE', 25.4, 30.48, 3.0, True)
    P.text('Une seule batterie. Chaque usage a son fusible et son convertisseur. Les sorties 5 V, 6 V et 12 V ne sont jamais reliées entre elles.', 25.4, 36.83, 1.6)
    # batterie, fusible principal, interrupteur général
    bt = P.place('Battery', 'BT1', 'Batterie du E600 (≤ 36 V chargée)', 35.56, 76.2, ref_at=(40.64, 74.93), val_at=(40.64, 78.74), just='left')
    f0 = P.place('Fuse', 'F0', 'Fusible principal DC', 50.8, 50.8, 90, ref_at=(50.8, 46.99), val_at=(50.8, 55.88))
    s1 = P.place('SW_SPST', 'S1', 'Interrupteur général DC', 68.58, 50.8, ref_at=(68.58, 46.99), val_at=(68.58, 59.69))
    bus = 88.9
    P.wire(bt['1'], (35.56, 50.8), f0['1'])
    P.wire(f0['2'], s1['1'])
    P.wire(s1['2'], (bus, 50.8))
    P.wire(bt['2'], (35.56, 91.44))
    P.gnd((35.56, 91.44))
    P.junction((35.56, 86.36))
    P.wire((35.56, 86.36), (30.48, 86.36))
    P.flag((30.48, 86.36))
    P.label('BUS+', (bus + 1.27, 50.8))
    P.text('BT1 : chimie, tension max chargée, BMS et', 20.32, 101.6)
    P.text('chargeur à relever au démontage. XL4015 :', 20.32, 105.41)
    P.text('entrée 8-36 V → pack ≤ 36 V chargé.', 20.32, 109.22)
    P.text('Fusibles et porte-fusibles : tension DC ≥', 20.32, 114.3)
    P.text('tension max du pack ; calibres à choisir', 20.32, 118.11)
    P.text('selon câbles et courants mesurés.', 20.32, 121.92)
    P.text('Masse : BT1− → barre de masse unique ;', 20.32, 127.0)
    P.text('chaque retour de puissance revient à la', 20.32, 130.81)
    P.text('barre par son propre fil.', 20.32, 134.62)
    rows = {'pi': 50.8, 'servo': 101.6, 'cmd': 152.4, 'trac': 223.52}
    P.wire((bus, 50.8), (bus, 101.6), (bus, 152.4), (bus, 223.52))
    for y in (50.8, 101.6, 152.4):
        P.junction((bus, y))

    def branch(y, ref_f, val_f, ref_u, val_u, ux=132.08):
        f = P.place('Fuse', ref_f, val_f, 101.6, y, 90, ref_at=(101.6, y - 3.81), val_at=(101.6, y + 5.08))
        u = P.place('Module_XL4015', ref_u, val_u, ux, y + 1.27, ref_at=(ux - 7.62, y - 5.08), val_at=(ux - 7.62, y + 7.62), just='left')
        P.wire((bus, y), f['1'])
        P.wire(f['2'], u['1'])
        P.wire(u['2'], (u['2'][0] - 2.54, u['2'][1]), (u['2'][0] - 2.54, u['2'][1] + 5.08))
        P.gnd((u['2'][0] - 2.54, u['2'][1] + 5.08))
        return u

    # Pi : 5,1 V
    u2 = branch(rows['pi'], 'F1', 'Fusible branche Pi', 'U2', 'XL4015 n°1 réglé à 5,1 V')
    P.wire(u2['3'], (167.64, rows['pi']))
    P.glabel('+5V_PI', (167.64, rows['pi']), 0, 'output')
    P.wire(u2['4'], (152.4, u2['4'][1]), (152.4, u2['4'][1] + 5.08))
    P.gnd((152.4, u2['4'][1] + 5.08))
    P.text('→ câble USB-C du Pi (feuille Raspberry Pi et USB).', 152.4, 44.45)
    P.text('Régler 5,1 V à vide, puis vérifier sous charge (Pi + USB).', 152.4, 64.77)
    # servo : 6 V, retour direct
    u3 = branch(rows['servo'], 'F2', 'Fusible branche servo', 'U3', 'XL4015 n°2 réglé à 6,0 V')
    m3 = P.place('Motor_Servo', 'M3', 'Servo direction Heemol DT996', 175.26, rows['servo'], ref_at=(182.88, 99.06), val_at=(182.88, 102.87), just='left')
    P.wire(u3['3'], m3['2'])
    P.wire(u3['4'], m3['3'])
    P.wire(m3['1'], (162.56, m3['1'][1]))
    P.glabel('SERVO_PWM', (162.56, m3['1'][1]), 180, 'input')
    P.junction((152.4, u3['4'][1]))
    P.wire((152.4, u3['4'][1]), (152.4, u3['4'][1] + 5.08))
    P.gnd((152.4, u3['4'][1] + 5.08))
    P.text('Le − du servo revient directement à OUT− du XL4015 n°2.', 152.4, 115.57)
    P.text('Signal PWM : J6.6 (D8) de la carte capteurs, référence GND J6.2.', 152.4, 119.38)
    P.text('Logique 3,3 V : vérifier que le DT996 la reconnaît.', 152.4, 123.19)
    # commande 12 V : chaîne d'arrêt à réarmement manuel
    u4 = branch(rows['cmd'], 'F4', 'Fusible commande', 'U4', 'XL4015 n°3 réglé à 12 V')
    s2 = P.place('SW_Push_Open', 'S2', 'Arrêt d’urgence (NF, verrouillable)', 172.72, rows['cmd'], ref_at=(172.72, 144.78), val_at=(172.72, 148.59))
    s3 = P.place('SW_Push', 'S3', 'Réarmement (NO)', 195.58, rows['cmd'], ref_at=(195.58, 148.59), val_at=(195.58, 157.48))
    P.wire(u4['3'], s2['1'])
    P.wire(u4['4'], (152.4, u4['4'][1]), (152.4, u4['4'][1] + 5.08))
    P.gnd((152.4, u4['4'][1] + 5.08))
    P.label('+12V_CMD', (u4['3'][0] + 1.27, rows['cmd']))
    P.wire(s2['2'], s3['1'])
    k1 = P.place('FINDER-40.52', 'K1', 'Relais commande 12 V', 231.14, 167.64, ref_at=(246.38, 165.1), val_at=(246.38, 168.91), just='left')
    latch = 220.98
    P.wire(s3['2'], (latch, rows['cmd']), k1['A1'])
    P.wire(k1['A2'], (latch, 180.34))
    P.gnd((latch, 180.34))
    # auto-maintien K1 11-14 en parallèle sur S3
    P.wire(k1['14'], (k1['14'][0], rows['cmd']))
    P.junction((k1['14'][0], rows['cmd']))
    P.wire((latch, rows['cmd']), (k1['14'][0], rows['cmd']))
    P.wire(k1['11'], (k1['11'][0], 185.42), (185.42, 185.42), (185.42, rows['cmd']))
    P.junction((185.42, rows['cmd']))
    P.junction((latch, rows['cmd']))
    P.nc(k1['12'])
    P.nc(k1['22'])
    # contact d'état K1 21-24 vers l'ESP
    P.wire(k1['21'], (k1['21'][0], 180.34))
    P.gnd((k1['21'][0], 180.34))
    P.wire(k1['24'], (254.0, k1['24'][1]))
    P.glabel('ARRET_OK', (254.0, k1['24'][1]), 0, 'output')
    # contacteur de traction K2, bobine en parallèle sur K1
    k2 = P.place('FINDER-40.11', 'K2', 'Contacteur DC traction', 284.48, 167.64, ref_at=(284.48, 140.97), val_at=(284.48, 144.78), just='left')
    P.wire((k1['14'][0], rows['cmd']), (k2['A1'][0], rows['cmd']), k2['A1'])
    P.wire(k2['A2'], (k2['A2'][0], 180.34))
    P.gnd((k2['A2'][0], 180.34))
    P.nc(k2['12'])
    P.text('Appui sur S3 : K1 et K2 collent, K1 11-14 maintient. Appui sur S2 (ou coupure 12 V) : tout retombe.', 190.5, 193.04)
    P.text('Relâcher S2 ne relance rien : il faut réarmer avec S3, puis armer par logiciel.', 190.5, 196.85)
    P.text('K1 21-24 (contact sec) → ARRET_OK = J6.5 (D9) : fermé vers GND = chaîne disponible.', 190.5, 200.66)
    # traction
    f3 = P.place('Fuse', 'F3', 'Fusible traction', 101.6, rows['trac'], 90, ref_at=(101.6, rows['trac'] - 3.81), val_at=(101.6, rows['trac'] + 5.08))
    P.wire((bus, rows['trac']), f3['1'])
    P.wire(f3['2'], (k2['11'][0], rows['trac']), k2['11'])
    u5 = P.place('Controleur_Traction', 'U5', 'Contrôleur de traction du E600 (à identifier)', 332.74, 162.56,
                 ref_at=(320.04, 151.13), val_at=(307.34, 177.8), just='left')
    P.wire(k2['14'], (k2['14'][0], 157.48), (302.26, 157.48), (302.26, u5['V+'][1]), u5['V+'])
    P.label('TRACTION+', (292.1 + 1.27, 157.48))
    P.wire(u5['GND'], (307.34, u5['GND'][1]), (307.34, 170.18))
    P.gnd((307.34, 170.18))
    m1 = P.place('Motor_DC', 'M1', 'Moteur roue gauche (E600)', 365.76, 147.32, ref_at=(370.84, 146.05), val_at=(370.84, 149.86), just='left')
    m2 = P.place('Motor_DC', 'M2', 'Moteur roue droite (E600)', 365.76, 190.5, ref_at=(370.84, 189.23), val_at=(370.84, 193.04), just='left')
    x0 = u5['M1A'][0]
    P.wire(u5['M1A'], (x0 + 5.08, u5['M1A'][1]), (x0 + 5.08, m1['1'][1]), m1['1'])
    P.wire(u5['M1B'], (x0 + 7.62, u5['M1B'][1]), (x0 + 7.62, m1['2'][1]), m1['2'])
    P.wire(u5['M2A'], (x0 + 5.08, u5['M2A'][1]), (x0 + 5.08, m2['1'][1]), m2['1'])
    P.wire(u5['M2B'], (x0 + 2.54, u5['M2B'][1]), (x0 + 2.54, m2['2'][1]), m2['2'])
    P.nc(u5['CMD'])
    P.nc(u5['RET'])
    P.text('Moteurs, contrôleur et retours roues : type (DC ou brushless), tension,', 299.72, 213.36)
    P.text('commande et capteurs à relever sur le E600. CMD/RET non câblés tant', 299.72, 217.17)
    P.text('qu’ils ne sont pas identifiés (pas de broche ESP réservée hors J8/D10).', 299.72, 220.98)
    P.text('Pré-charge du contrôleur à prévoir si gros condensateurs d’entrée.', 299.72, 224.79)
    P.box(15.24, 40.64, 85.09, 139.7)
    P.box(147.32, 139.7, 312.42, 203.2)
    P.text('Chaîne d’arrêt câblée (indépendante du Pi et de l’ESP)', 147.32, 138.43, 1.6, True)
    return P


# ---------------------------------------------------------------- Pi et USB
def sheet_pi():
    C = Sheet('C', 'FieldMind rover / Raspberry Pi 4 et périphériques USB')
    C.text('RASPBERRY PI 4 ET USB', 25.4, 30.48, 3.0, True)
    C.text('Le Pi est alimenté par le XL4015 n°1 via un câble USB-C. Ses 4 ports USB : 2 webcams, lidar, XIAO.', 25.4, 36.83, 1.6)
    pi = C.place('RPi4_Ports', 'U1', 'Raspberry Pi 4 Model B', 177.8, 116.84, ref_at=(160.02, 104.14), val_at=(160.02, 132.08), just='left')
    p1 = C.place('USB_C_Plug_USB2.0', 'P1', 'Fiche USB-C (câble alimentation Pi)', 101.6, 116.84, ref_at=(91.44, 91.44), val_at=(91.44, 142.24), just='left')
    C.wire(p1['A4'], (137.16, p1['A4'][1]), (137.16, pi['PWR'][1]), pi['PWR'])
    C.glabel('+5V_PI', (127.0, p1['A4'][1]), 90, 'input')
    C.wire(p1['A1'], (p1['A1'][0], 147.32))
    C.gnd((p1['A1'][0], 147.32))
    C.wire(pi['GND'], (144.78, pi['GND'][1]), (144.78, 147.32))
    C.gnd((144.78, 147.32))
    used = {p1['A4'], p1['A1']}
    for k, pt in p1.items():
        if pt not in used:
            used.add(pt)
            C.nc(pt)
    C.text('Câble USB-C : seulement VBUS et GND reliés au XL4015 n°1 (5,1 V).', 63.5, 152.4)
    # webcams sur USB 3.0
    for ref, val, y, port, xv in [('U6', 'Webcam UGREEN 1080p (gauche)', 96.52, 'USB3A', 261.62),
                                  ('U7', 'Webcam UGREEN 1080p (droite)', 76.2, 'USB3B', 264.16)]:
        c = C.place('Webcam_USB', ref, val, 284.48, y, ref_at=(274.32, y - 5.08), val_at=(274.32, y + 5.08), just='left')
        C.wire(pi[port], (xv, pi[port][1]), (xv, c['USB'][1]), c['USB'])
        C.label('USB_CAM_G' if ref == 'U6' else 'USB_CAM_D', (pi[port][0] + 2.54, pi[port][1]))
    # lidar + adaptateur sur USB 2.0 n°1
    ad = C.place('Adaptateur_USB_C1', 'U8', 'Adaptateur USB du kit C1', 254.0, 132.08, mirror=True, ref_at=(243.84, 123.19), val_at=(243.84, 142.24), just='left')
    li = C.place('RPLIDAR_C1', 'U9', 'Slamtec RPLIDAR C1', 304.8, 132.08, mirror=True, ref_at=(294.64, 123.19), val_at=(294.64, 142.24), just='left')
    C.wire(li['1'], ad['1'])
    C.wire(li['2'], ad['2'])
    C.wire(li['3'], ad['3'])
    C.wire(li['4'], ad['4'])
    C.wire(pi['USB2A'], (228.6, pi['USB2A'][1]), (228.6, ad['USB'][1]), ad['USB'])
    C.label('USB_LIDAR', (pi['USB2A'][0] + 2.54, pi['USB2A'][1]))
    C.text('Faisceau 4 fils du kit : rouge 5V, jaune TX lidar → RX, vert RX lidar ← TX, noir GND.', 236.22, 152.4)
    # XIAO sur USB 2.0 n°2
    C.wire(pi['USB2B'], (220.98, pi['USB2B'][1]), (220.98, 172.72))
    C.glabel('USB_XIAO', (220.98, 172.72), 270)
    C.text('→ USB-C du XIAO (carte capteurs) : données série + alimentation 5 V de la carte.', 223.52, 177.8)
    C.nc(pi['WIFI'])
    C.text('Wi-Fi : réseau de la maison (box), vers l’ASUS GX10 distant.', 160.02, 135.89)
    C.text('Bilan USB : le Pi 4 fournit 1,2 A au total sur ses ports. Mesurer 2 webcams + lidar + XIAO ensemble ;', 63.5, 200.66)
    C.text('si c’est trop, ajouter un hub USB alimenté (qui ne doit pas réalimenter le Pi par l’USB).', 63.5, 204.47)
    C.text('Connecteur GPIO 40 broches du Pi : non utilisé (tout passe par l’USB).', 63.5, 208.28)
    return C


# ---------------------------------------------------------------- carte capteurs
def sheet_sensors():
    S = Sheet('S', 'FieldMind rover / carte capteurs A7 et périphériques')
    S.text('CARTE CAPTEURS A7 ET PÉRIPHÉRIQUES', 25.4, 30.48, 3.0, True)
    S.text('La carte A7 (perfboard 4 × 6 cm) porte le XIAO ESP32C5, l’écran OLED, le GY-521 et le BMP280 :', 25.4, 36.83, 1.6)
    S.text('détail broche par broche dans ../carte_capteurs/cad/perfboard_c5_a7. Ici : ce qui se branche sur ses connecteurs.', 25.4, 40.64, 1.6)
    a = S.place('Carte_Capteurs_A7', 'A1', 'Carte capteurs A7 (XIAO ESP32C5 + OLED + GY-521 + BMP280)', 127.0, 127.0,
                ref_at=(106.68, 100.33), val_at=(106.68, 165.1), just='left')
    S.wire(a['USB'], (93.98, a['USB'][1]))
    S.glabel('USB_XIAO', (93.98, a['USB'][1]), 180)
    S.text('USB-C : Pi USB 2.0 n°2', 71.12, 121.92)
    # GPS sur J4 : symbole dans l'ordre de J4, fils droits
    g = S.place('ATGM336H', 'U10', 'GPS ATGM336H (AYWHP) + antenne', 223.52, a['J4.1'][1] + 5.08, ref_at=(213.36, 96.52), val_at=(213.36, 99.06), just='left')
    for pin, gp in (('J4.1', 'RXD'), ('J4.2', 'GND'), ('J4.3', 'VCC'), ('J4.4', 'TXD')):
        S.wire(a[pin], g[gp])
    S.nc(g['PPS'])
    S.label('3V3_XIAO', (a['J4.3'][0] + 1.27, a['J4.3'][1]))
    S.label('GND', (a['J4.2'][0] + 1.27, a['J4.2'][1]))
    S.text('TX du GPS → TX_GPS (J4.4, D7) ; RX du GPS ← RX_GPS (J4.1, D6). PPS non utilisé.', 256.54, 99.06)
    S.text('Ordre des broches du module à lire sur l’exemplaire reçu.', 256.54, 102.87)
    S.text('Antenne dégagée vers le ciel, loin du Pi et du lidar.', 256.54, 106.68)
    # Knob sur J7
    k = S.place('Modulino_Knob', 'U11', 'Arduino Modulino Knob (I2C 0x3A)', 223.52, a['J7.1'][1] + 2.54, ref_at=(213.36, 132.08), val_at=(213.36, 134.62), just='left')
    for i in range(1, 5):
        S.wire(a[f'J7.{i}'], k[str(i)])
    S.label('GND', (a['J7.1'][0] + 1.27, a['J7.1'][1]))
    S.label('3V3_XIAO', (a['J7.2'][0] + 1.27, a['J7.2'][1]))
    S.text('Câble Qwiic → Dupont, ou fils soudés sur les pastilles 1×4 du Knob, même ordre.', 256.54, 124.46)
    # J6 : servo PWM, état d'arrêt
    S.wire(a['J6.6'], (205.74, a['J6.6'][1]))
    S.glabel('SERVO_PWM', (205.74, a['J6.6'][1]), 0, 'output')
    S.wire(a['J6.5'], (205.74, a['J6.5'][1]))
    S.glabel('ARRET_OK', (205.74, a['J6.5'][1]), 0, 'input')
    S.wire(a['J6.2'], (165.1, a['J6.2'][1]))
    S.gnd((165.1, a['J6.2'][1]), 90)
    S.label('GND', (a['J6.2'][0] + 1.27, a['J6.2'][1]))
    for p in ('J6.1', 'J6.3', 'J6.4'):
        S.nc(a[p])
    S.text('J6.6 D8 → signal du servo M3 ; J6.2 GND : référence (un fil le long du signal).', 228.6, 139.7)
    S.text('J6.5 D9 ← contact K1 21-24 (tirage interne, bas = chaîne d’arrêt disponible).', 228.6, 143.51)
    S.text('J6.1 5V, J6.3 3V3, J6.4 D10 : libres pour un futur capteur.', 228.6, 147.32)
    for p in ('J8.1', 'J8.2', 'J8.3', 'J8.4'):
        S.nc(a[p])
    S.text('J8 (D0-D3) : réservé aux retours roues ou à la commande du contrôleur de traction,', 167.64, 160.02)
    S.text('à affecter après démontage du E600. D2/D3 : broches de démarrage (rien qui les force à l’allumage).', 167.64, 163.83)
    S.text('Dans la carte, les broches de même nom sont reliées : GND (J4.2, J6.2, J7.1), 3V3 (J4.3, J6.3, J7.2),', 25.4, 223.52)
    S.text('SDA/SCL de J7 avec l’OLED, le GY-521 et le BMP280. Adresses I2C attendues : OLED 0x3C, GY-521 0x68, BMP280 0x76, Knob 0x3A.', 25.4, 227.33)
    return S


def root(sheets_meta):
    R = Sheet('R', 'FieldMind rover prototype / schéma électrique complet')
    R.text('FIELDMIND · ROVER PROTOTYPE · SCHÉMA ÉLECTRIQUE COMPLET', 25.4, 30.48, 3.2, True)
    R.text('Vue d’ensemble. Les trois feuilles détaillent le câblage ; les étiquettes globales relient les feuilles.', 25.4, 37.47, 1.6)
    lines = [
        ('Puissance et arrêt', 'Batterie E600 → F0 → S1 → bus. Branches fusiblées : XL4015 5,1 V (Pi), 6 V (servo), 12 V (commande), traction.'),
        ('', 'Arrêt d’urgence S2 (NF) + réarmement S3 → relais K1 (auto-maintien) → contacteur K2 de traction. Contact K1 21-24 → ESP D9.'),
        ('Raspberry Pi et USB', 'Pi 4 alimenté en USB-C. USB 3.0 : 2 webcams. USB 2.0 : lidar C1 (via son adaptateur) et XIAO de la carte capteurs.'),
        ('Carte capteurs', 'XIAO ESP32C5 + OLED + GY-521 + BMP280 sur la perfboard A7. J4 → GPS, J7 → Knob, J6 → servo (D8) et état d’arrêt (D9).'),
    ]
    y = 165.1
    for title, t in lines:
        if title:
            R.text(title + ' :', 25.4, y, 1.6, True)
            y += 3.81
        R.text(t, 30.48, y, 1.5)
        y += 3.81
    y += 3.81
    R.text('Étiquettes entre feuilles', 25.4, y, 1.6, True); y += 3.81
    for t in ['+5V_PI : XL4015 n°1 → USB-C du Pi', 'USB_XIAO : port USB 2.0 n°2 du Pi → USB-C du XIAO',
              'SERVO_PWM : J6.6 (D8) → signal du servo M3', 'ARRET_OK : contact K1 21-24 → J6.5 (D9), fermé vers GND = chaîne disponible']:
        R.text('• ' + t, 30.48, y, 1.5); y += 3.81
    y += 3.81
    R.text('À relever avant câblage définitif', 25.4, y, 1.6, True); y += 3.81
    for t in ['Batterie, BMS, chargeur, contrôleur et moteurs du E600 (type, tension, courants, commande, retours).',
              'Calibres des fusibles F0-F4, section des câbles, tension DC des porte-fusibles ≥ tension max chargée.',
              'Réglage des XL4015 à vide puis en charge avant de brancher Pi, servo ou relais.',
              'Brochage du GPS reçu ; signal 3,3 V reconnu par le servo ; bilan USB du Pi.',
              'Rien n’a été essayé physiquement.']:
        R.text('• ' + t, 30.48, y, 1.5); y += 3.81
    return R


def run(*args):
    r = subprocess.run([KICAD_CLI, *map(str, args)], capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(f'kicad-cli {" ".join(map(str, args))}\n{r.stdout}\n{r.stderr}')
    return r.stdout


def main():
    force = '--force' in sys.argv
    rootf = CAD / (PROJECT + '.kicad_sch')
    if rootf.exists() and '(generator "eeschema")' in rootf.read_text()[:400] and not force:
        raise SystemExit('Projet déjà enregistré par KiCad : relancer avec --force (éditions manuelles perdues).')
    CAD.mkdir(parents=True, exist_ok=True); OUT.mkdir(parents=True, exist_ok=True)
    pages = {'P': sheet_power(), 'C': sheet_pi(), 'S': sheet_sensors()}
    meta = [(SHEETS['P'][0], SHEETS['P'][1], SHEET_UUID['P'], 25.4, 63.5, 101.6, 76.2, 2),
            (SHEETS['C'][0], SHEETS['C'][1], SHEET_UUID['C'], 152.4, 63.5, 101.6, 76.2, 3),
            (SHEETS['S'][0], SHEETS['S'][1], SHEET_UUID['S'], 279.4, 63.5, 101.6, 76.2, 4)]
    R = root(meta)
    texts = {'P': ['Batterie, fusibles, interrupteur général', '3 × XL4015 (5,1 V / 6 V / 12 V)', 'Arrêt d’urgence, relais, contacteur',
                   'Contrôleur et moteurs du E600', 'Servo de direction DT996'],
             'C': ['Raspberry Pi 4 (USB-C 5,1 V)', '2 webcams UGREEN (USB 3.0)', 'RPLIDAR C1 + adaptateur (USB 2.0)', 'Liaison USB vers le XIAO'],
             'S': ['Carte A7 : XIAO, OLED, GY-521, BMP280', 'GPS ATGM336H sur J4', 'Modulino Knob sur J7', 'Servo (D8) et état d’arrêt (D9) sur J6',
                   'J8 réservé (retours roues)']}
    for (name, file, sid, x, y, w, h, page), k in zip(meta, 'PCS'):
        for i, t in enumerate(texts[k]):
            R.text('• ' + t, x + 3.81, y + 10.16 + i * 5.08, 1.6)
    (CAD / (PROJECT + '.kicad_sch')).write_text(R.render(meta))
    for k, sh in pages.items():
        (CAD / SHEETS[k][1]).write_text(sh.render())
    (CAD / (LIB + '.kicad_sym')).write_text('(kicad_symbol_lib (version 20241209) (generator "fieldmind_rover") (generator_version "s1")\n'
                                            + '\n'.join(CUSTOM.values()) + '\n)\n')
    (CAD / 'sym-lib-table').write_text(f'(sym_lib_table (version 7) (lib (name {q(LIB)}) (type "KiCad") (uri "${{KIPRJMOD}}/{LIB}.kicad_sym") (options "") (descr "Modules du rover FieldMind")))\n')
    pro = {'meta': {'filename': PROJECT + '.kicad_pro', 'version': 3},
           'sheets': [[ROOT, 'Root']] + [[SHEET_UUID[k], SHEETS[k][0]] for k in 'PCS']}
    (CAD / (PROJECT + '.kicad_pro')).write_text(json.dumps(pro, indent=2, ensure_ascii=False) + '\n')
    for f in [PROJECT + '.kicad_sch'] + [SHEETS[k][1] for k in 'PCS']:
        run('sch', 'upgrade', '--force', CAD / f)
    run('sym', 'upgrade', '--force', CAD / (LIB + '.kicad_sym'))
    print('ok')


if __name__ == '__main__':
    main()
