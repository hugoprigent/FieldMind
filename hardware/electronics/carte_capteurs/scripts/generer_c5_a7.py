#!/usr/bin/env python3
"""Génère la révision A7 (UNE perfboard 4 x 6 cm, tous les fils dessous, ponts de soudure dessus) :
projet KiCad, modèles 3D, vues des deux faces, tables et guide. N'écrit que dans :

  cad/perfboard_c5_a7/   exports/c5_a7/   fils_c5_a7.csv   occupation_trous_c5_a7.csv

Sources écrites au format KiCad 9 puis converties par `kicad-cli ... upgrade --force`.
Refuse d'écraser un projet A7 déjà enregistré par KiCad 10 sans `--force`.
"""
import csv
import json
import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from html import escape
from pathlib import Path
from uuid import NAMESPACE_URL, uuid5

import plan_c5_a7 as P

BASE = Path(__file__).resolve().parents[1]
CAD = BASE / 'cad/perfboard_c5_a7'
OUT = BASE / 'exports/c5_a7'
PROJECT = 'fieldmind_c5_a7'
LIB = 'FieldMind_C5_A7'
KICAD_CLI = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
KICAD_SYMBOLS = Path('/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols')
DATE = '2026-10-13'
REV = 'A7 / non testé'


def uid(key):
    return str(uuid5(NAMESPACE_URL, 'fieldmind/A7/' + key))


def q(s):
    return json.dumps(str(s), ensure_ascii=False)


def fmt(v):
    return f'{round(v, 4):g}'


ROOT = uid('sheet/root')
SHEET_B = uid('sheet/B')
SHEET_FILE = {'A': PROJECT + '.kicad_sch', 'B': 'capteurs_i2c.kicad_sch'}
SHEET_PATH = {'A': '/' + ROOT, 'B': f'/{ROOT}/{SHEET_B}'}
PCB_PATH_PREFIX = {'A': '', 'B': '/' + SHEET_B}
SHEET_NAME = {'A': '/', 'B': '/Capteurs I2C et écran/'}


# ====================================================================== symboles
def sexpr_block(text, start):
    depth, j, n = 0, start, len(text)
    while j < n:
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
    raise ValueError('bloc non fermé')


def std_symbol(lib, name):
    text = (KICAD_SYMBOLS / f'{lib}.kicad_sym').read_text()
    i = text.find(f'(symbol "{name}"')
    assert i >= 0, (lib, name)
    return sexpr_block(text, i)


def parse(s):
    stack, root = [], None
    for t in re.findall(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()]+', s):
        if t == '(':
            n = []
            (stack[-1].append(n) if stack else None)
            root = root or n
            stack.append(n)
        elif t == ')':
            stack.pop()
        else:
            stack[-1].append(json.loads(t) if t.startswith('"') else t)
    return root


def walk(n):
    yield n
    for v in n:
        if isinstance(v, list):
            yield from walk(v)


def lib_pins(block):
    pins = {}
    for n in walk(parse(block)):
        if n and n[0] == 'pin':
            at = next(v for v in n if isinstance(v, list) and v[0] == 'at')
            num = next(v for v in n if isinstance(v, list) and v[0] == 'number')[1]
            pins[num] = (float(at[1]), float(at[2]))
    return pins


STD = {'R': ('Device', 'R'), 'C': ('Device', 'C'), 'CP': ('Device', 'C_Polarized'),
       'SW': ('Switch', 'SW_Push'), 'CONN2': ('Connector_Generic', 'Conn_01x02'),
       'CONN3': ('Connector_Generic', 'Conn_01x03'), 'CONN4': ('Connector_Generic', 'Conn_01x04'),
       'CONN5': ('Connector_Generic', 'Conn_01x05'), 'CONN6': ('Connector_Generic', 'Conn_01x06'), 'P3V3': ('power', '+3V3'), 'GND': ('power', 'GND'),
       'PWR_FLAG': ('power', 'PWR_FLAG')}


def eff(size=1.27, justify='', hide=False):
    return f'(effects (font (size {size} {size})){" (justify " + justify + ")" if justify else ""}{" (hide yes)" if hide else ""})'


def custom_symbol(name, ref, left, right=(), w=7.62):
    """Symbole rectangulaire : broches gauche (num, nom) de haut en bas, idem droite."""
    n = max(len(left), len(right))
    h = round((n - 1) * 1.27 + 2.54, 2)
    s = [f'(symbol {q(name)} (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)',
         f'(property "Reference" {q(ref)} (at 0 {fmt(h + 1.27)} 0) {eff()})',
         f'(property "Value" {q(name)} (at 0 {fmt(-h - 1.27)} 0) {eff()})',
         f'(property "Footprint" "" (at 0 0 0) {eff(hide=True)})',
         f'(property "Datasheet" "" (at 0 0 0) {eff(hide=True)})',
         f'(property "Description" "" (at 0 0 0) {eff(hide=True)})',
         f'(symbol "{name}_0_1" (rectangle (start {fmt(-w)} {fmt(h)}) (end {fmt(w)} {fmt(-h)}) '
         f'(stroke (width 0.254) (type default)) (fill (type background))))',
         f'(symbol "{name}_1_1"']
    for side, pins in (('L', left), ('R', right)):
        for i, (num, label) in enumerate(pins):
            y = round((len(pins) - 1) * 1.27 - i * 2.54, 2)
            x, a = (-w - 5.08, 0) if side == 'L' else (w + 5.08, 180)
            s.append(f'(pin passive line (at {fmt(x)} {fmt(y)} {a}) (length 5.08) (name {q(label)} {eff()}) (number {q(num)} {eff()}))')
    return '\n'.join(s + ['))'])


def make_symbols():
    syms = {}
    for key, (lib, name) in STD.items():
        syms[key] = (name, std_symbol(lib, name))
    xiao = P.BYREF['U1']
    left = [(str(i + 1), f'{xiao["names"][i]}/{P.XIAO_GPIO[i]}') for i in range(7)]
    right = [(str(n), xiao['names'][n - 1] + (f'/{P.XIAO_GPIO[n - 1]}' if P.XIAO_GPIO[n - 1] else ''))
             for n in [14, 13, 12, 11, 10, 9, 8]]
    syms['XIAO'] = ('XIAO_ESP32C5', custom_symbol('XIAO_ESP32C5', 'U', left, right, w=15.24))
    for ref, name in [('J3', 'Carte_OLED_I2C'), ('J1', 'Carte_GY521'), ('J2', 'Carte_BMP280')]:
        p = P.BYREF[ref]
        syms[ref] = (name, custom_symbol(name, 'J', [(str(i + 1), n) for i, n in enumerate(p['names'])]))
    pins = {k: lib_pins(v[1]) for k, v in syms.items()}
    return syms, pins


SYMS, SYMPINS = make_symbols()


def lib_entry(key, full=True):
    name, block = SYMS[key]
    if not full:
        return block
    return block.replace(f'(symbol "{name}"', f'(symbol "{LIB}:{name}"', 1)


def symbol_key(p):
    if p['kind'] == 'module':
        return 'XIAO' if p['module'] == 'XIAO' else p['ref']
    return {'resistor': 'R', 'capacitor': 'C', 'cpol': 'CP', 'switch': 'SW'}.get(p['kind'], f'CONN{len(p["nets"])}')


# ====================================================================== schéma
class Sheet:
    def __init__(self, key, title):
        self.key, self.title, self.items, self.n = key, title, [], 0
        self.placed = {}

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

    def glabel(self, net, pt, angle):
        just = {0: 'left', 180: 'right', 90: 'left', 270: 'right'}[angle]
        self.items.append(f'(global_label {q(net)} (shape bidirectional) (at {fmt(pt[0])} {fmt(pt[1])} {angle}) '
                          f'(fields_autoplaced yes) {eff(justify=just)} (uuid {self.ident("gl")}) '
                          f'(property "Intersheetrefs" "${{INTERSHEET_REFS}}" (at {fmt(pt[0])} {fmt(pt[1])} 0) {eff(hide=True)}))')

    def label(self, net, pt):
        self.items.append(f'(label {q(net)} (at {fmt(pt[0])} {fmt(pt[1])} 0) {eff(justify="left bottom")} (uuid {self.ident("l")}))')

    def text(self, t, x, y, size=1.27):
        self.items.append(f'(text {q(t)} (exclude_from_sim no) (at {fmt(x)} {fmt(y)} 0) {eff(size, "left bottom")} (uuid {self.ident("t")}))')

    def place(self, key, ref, value, x, y, angle=0, mirror=False, footprint='', uuid=None,
              in_bom=True, on_board=True, ref_at=None, val_at=None, hide_value=False, just=''):
        """Place un symbole ; renvoie {numéro de broche: point de connexion}."""
        uuid = uuid or uid(f'{self.key}/sym/{ref}')
        rx, ry = ref_at or (x, y - 6)
        vx, vy = val_at or (x, y + 6)
        power = ref.startswith('#')
        # KiCad compose l'angle du champ avec celui du symbole : texte toujours horizontal.
        fa = 90 if angle in (90, 270) else 0
        pins = {}
        for num, (px, py) in SYMPINS[key].items():
            if mirror:
                px = -px
            a = math.radians(angle)
            qx, qy = px * math.cos(a) - py * math.sin(a), px * math.sin(a) + py * math.cos(a)
            pins[num] = (round(x + qx, 4), round(y - qy, 4))
        name = SYMS[key][0]
        pin_uuids = ' '.join(f'(pin {q(n)} (uuid {uid(f"{self.key}/{ref}/pin/{n}")}))' for n in SYMPINS[key])
        self.items.append(
            f'(symbol (lib_id "{LIB}:{name}") (at {fmt(x)} {fmt(y)} {angle}){" (mirror y)" if mirror else ""} (unit 1) '
            f'(exclude_from_sim no) (in_bom {"yes" if in_bom else "no"}) (on_board {"yes" if on_board else "no"}) (dnp no) '
            f'(uuid {uuid}) '
            f'(property "Reference" {q(ref)} (at {fmt(rx)} {fmt(ry)} {fa}) {eff(justify=just, hide=power)}) '
            f'(property "Value" {q(value)} (at {fmt(vx)} {fmt(vy)} {fa}) {eff(justify=just, hide=hide_value)}) '
            f'(property "Footprint" {q(footprint)} (at {fmt(x)} {fmt(y)} 0) {eff(hide=True)}) '
            f'(property "Datasheet" "" (at {fmt(x)} {fmt(y)} 0) {eff(hide=True)}) '
            f'{pin_uuids} (instances (project {q(PROJECT)} (path {q(SHEET_PATH[self.key])} '
            f'(reference {q(ref)}) (unit 1)))))')
        self.placed[ref] = pins
        return pins

    def power(self, kind, pt, angle=0):
        self.n += 1
        ref = f'#PWR{self.key}{self.n:02}' if kind != 'PWR_FLAG' else f'#FLG{self.key}{self.n:02}'
        value = {'P3V3': '+3V3', 'GND': 'GND', 'PWR_FLAG': 'PWR_FLAG'}[kind]
        up = {'P3V3': (0, -1), 'PWR_FLAG': (0, -1), 'GND': (0, 1)}[kind]   # sens du dessin à 0°
        a = math.radians(angle)
        dx = up[0] * math.cos(a) + up[1] * math.sin(a)
        dy = -up[0] * math.sin(a) + up[1] * math.cos(a)
        dist = 4.6 if dx else (3.6 if dy > 0 else 3.4)
        self.place(kind, ref, value, pt[0], pt[1], angle, in_bom=False,
                   val_at=(pt[0] + dx * dist, pt[1] + dy * dist))

    def part(self, ref, x, y, angle=0, mirror=False, **kw):
        p = P.BYREF[ref]
        fp = f'{LIB}:A7_{ref}' if p['face'] else ''
        return self.place(symbol_key(p), ref, p['value'], x, y, angle, mirror, fp,
                          uuid=uid(f'symbol/{ref}'), on_board=bool(p['face']), **kw)

    def render(self, sheets=()):
        keys = sorted({k for k in SYMS})
        s = [f'(kicad_sch (version 20250114) (generator "fieldmind_a4") (generator_version "a4") '
             f'(uuid {ROOT if self.key == "A" else uid("file/" + self.key)}) (paper "A3")',
             f'(title_block (title {q(self.title)}) (date {q(DATE)}) (rev {q(REV)}) '
             f'(comment 1 "Une perfboard 4 x 6 cm : cartes dessus, XIAO et tous les fils dessous") '
             f'(comment 2 "Brochage des cartes capteurs et dimensions : provisoires, à contrôler sur les unités"))',
             '(lib_symbols', '\n'.join(lib_entry(k) for k in keys), ')']
        s += self.items
        for i, (name, file, sid, x, y, w, h) in enumerate(sheets):
            s.append(f'(sheet (at {fmt(x)} {fmt(y)}) (size {fmt(w)} {fmt(h)}) (exclude_from_sim no) (in_bom yes) '
                     f'(on_board yes) (dnp no) (fields_autoplaced yes) (stroke (width 0.1524) (type solid)) '
                     f'(fill (color 0 0 0 0.0000)) (uuid {sid}) '
                     f'(property "Sheetname" {q(name)} (at {fmt(x)} {fmt(y - 0.7)} 0) {eff(justify="left bottom")}) '
                     f'(property "Sheetfile" {q(file)} (at {fmt(x)} {fmt(y + h + 0.6)} 0) {eff(justify="left top")}) '
                     f'(instances (project {q(PROJECT)} (path {q("/" + ROOT)} (page {q(i + 2)})))))')
        if self.key == 'A':
            s.append('(sheet_instances (path "/" (page "1")))')
        return '\n'.join(s + [')']) + '\n'


def conn_side(sheet, pins, nets, x_end, angle, powers=True):
    """Fils horizontaux depuis chaque broche jusqu'à x_end puis étiquette / alimentation."""
    for num, net in nets.items():
        pt = pins[num]
        end = (x_end, pt[1])
        if net is None:
            sheet.nc(pt)
            continue
        sheet.wire(pt, end)
        if net == '+3V3' and powers:
            sheet.power('P3V3', end, 90 if angle == 180 else 270)
        elif net == 'GND' and powers:
            sheet.power('GND', end, 270 if angle == 180 else 90)
        else:
            sheet.glabel(net, end, angle)


def build_schematics():
    A = Sheet('A', 'FieldMind C5 A7 / plaque unique : XIAO, GNSS, Knob, extension')
    # --- XIAO
    A.text('XIAO ESP32C5 (face dessous, USB-C au bord haut ; broches pontées côté écran)', 50.8, 55.88, 2.0)
    u = A.part('U1', 101.6, 88.9, ref_at=(101.6, 76.2), val_at=(101.6, 101.6))
    xi = P.BYREF['U1']
    conn_side(A, u, {str(i): xi['nets'][i - 1] for i in range(1, 8)}, 71.12, 180)
    conn_side(A, u, {str(i): xi['nets'][i - 1] for i in range(8, 15)}, 132.08, 0)
    A.text('USB-C : console + lien Raspberry Pi. D2/D3 : strapping, laisser libres.', 50.8, 111.76)
    A.text('Toutes les broches sont reliées : D0-D3 sur J8, D8-D10, 5V, 3V3, GND sur J6, I2C sur J7 (Knob).', 50.8, 115.57)
    # --- drapeaux d'alimentation (aucun découplage ajouté : déjà sur le XIAO et les cartes)
    A.text('Alimentation', 147.32, 66.04, 1.8)
    A.power('P3V3', (152.4, 74.93)); A.wire((152.4, 74.93), (165.1, 74.93)); A.power('PWR_FLAG', (165.1, 74.93))
    A.power('GND', (152.4, 87.63)); A.wire((152.4, 87.63), (165.1, 87.63)); A.power('PWR_FLAG', (165.1, 87.63), 180)
    A.text('3V3 et GND fournis par le XIAO (USB). Pas de C1/C2 :', 147.32, 99.06)
    A.text('découplage déjà présent sur le XIAO et sur chaque carte.', 147.32, 102.87)
    # --- J4 GNSS
    A.text('GNSS déporté (antenne côté ciel)', 205.74, 119.38, 1.8)
    j4 = A.part('J4', 228.6, 133.35, ref_at=(227.33, 128.27), val_at=(227.33, 140.97), just='left')
    conn_side(A, j4, dict(zip('1234', P.BYREF['J4']['nets'])), 213.36, 180)
    A.text('1 RX GPS <- D6   2 GND   3 VCC   4 TX GPS -> D7', 205.74, 149.86)
    A.text('VCC seulement après contrôle tension/courant ; RX GPS facultatif.', 205.74, 153.67)
    # --- J6 extension
    A.text('Extension J6 (bord gauche)', 205.74, 166.37, 1.8)
    j6 = A.part('J6', 228.6, 185.42, ref_at=(227.33, 176.53), val_at=(227.33, 195.58), just='left')
    conn_side(A, j6, {str(i + 1): n for i, n in enumerate(P.BYREF['J6']['nets'])}, 213.36, 180)
    A.text('3,3 V maximum sur chaque broche. D8/D9 prévus servo et retour arrêt.', 205.74, 209.55)
    # --- J8 extension D0-D3
    A.text('Extension J8 (bord droit) : D0, D1, D2, D3', 139.7, 223.52, 1.8)
    j8 = A.part('J8', 162.56, 238.76, ref_at=(161.29, 231.14), val_at=(161.29, 246.38), just='left')
    conn_side(A, j8, {str(i + 1): n for i, n in enumerate(P.BYREF['J8']['nets'])}, 147.32, 180)
    A.text('D2/D3 : strapping, rien qui les force au démarrage.', 205.74, 213.36)
    A.text('5V = USB du XIAO : capteurs seulement, jamais de servo ni moteur.', 205.74, 217.17)
    # --- J7 Modulino Knob
    A.text('Modulino Knob (encodeur I2C), J7 au bord droit', 63.5, 147.32, 1.8)
    j7 = A.part('J7', 101.6, 170.18, ref_at=(100.33, 162.56), val_at=(100.33, 177.8), just='left')
    conn_side(A, j7, dict(zip('1234', P.BYREF['J7']['nets'])), 86.36, 180)
    A.text('Ordre du connecteur 1x4 du Knob : GND, 3.3V, SDA, SCL (broche 1 = GND).', 63.5, 187.96)
    A.text('Adresse bus 0x3A (0x76 en notation Arduino) : pas de conflit avec BMP280 0x76.', 63.5, 191.77)
    A.text('Remplace le panneau de boutons : plus de R1, R2, C3, J5 ni de réseau de résistances.', 63.5, 195.58)
    # --- notes générales
    A.text('A7 : une plaque 4 x 6 cm. Dessus : OLED, GY-521, BMP280, J4, J6, J7, J8. Dessous : XIAO et tous les fils.', 279.4, 160.02)
    A.text('Fils dessous seulement ; dessus : ponts de soudure du XIAO vers le trou voisin. Une extrémité de fil au plus par pastille.', 279.4, 163.83)
    A.text('Vues des deux faces et liste des fils : ../../exports/c5_a7/.', 279.4, 167.64)
    A.text('Aucun essai physique. Cotes des cartes : valeurs typiques à mesurer.', 279.4, 171.45)
    sheets = [('Capteurs I2C et écran', SHEET_FILE['B'], SHEET_B, 279.4, 66.04, 76.2, 12.7)]
    A.text('Face dessus : OLED, GY-521, BMP280 sur supports femelles.', 279.4, 86.36)

    B = Sheet('B', 'FieldMind C5 A7 / capteurs I2C et écran (face dessus)')
    B.text('Cartes montées sur supports femelles, face dessus de la plaque unique', 50.8, 45.72, 2.0)
    B.text('SDA/SCL rejoignent la feuille racine par étiquettes globales ; 3V3 et GND par symboles.', 50.8, 106.68)
    for ref, x, y in [('J3', 165.1, 76.2), ('J1', 165.1, 124.46), ('J2', 165.1, 175.26)]:
        p = P.BYREF[ref]
        h = (len(p['nets']) - 1) * 1.27 + 2.54
        pins = B.part(ref, x, y, ref_at=(x - 7.62, y - h - 1.27), val_at=(x - 7.62, y + h + 2.54), just='left')
        conn_side(B, pins, {str(i + 1): n for i, n in enumerate(p['nets'])}, x - 22.86, 180)
    B.text('OLED : ordre GND/VCC/SCL/SDA à lire sur la carte ; barrette en bas, affichage tourné de 180° par logiciel.', 203.2, 73.66)
    B.text('GY-521 : VCC en 3V3 direct (régulateur de la carte). AD0 libre = rappel à GND (0x68), à vérifier.', 203.2, 121.92)
    B.text('XDA/XCL/INT non câblés. Fixer la carte rigidement ; axes repérés.', 203.2, 125.73)
    B.text('BMP280 3,3 V : CSB et SDO libres (rappels CSB->VCC, SDO->GND sur la carte, I2C 0x76).', 203.2, 172.72)
    B.text('Contrôle ohmmètre avant montage : CSB-VCC et SDO-GND ~10k. Évent à l’air libre.', 203.2, 176.53)
    B.text('Ordres de broches PROVISOIRES : confirmer les inscriptions de chaque carte avant soudure.', 50.8, 220.98, 1.6)

    return {'A': A.render(sheets), 'B': B.render()}


# ====================================================================== PCB
PAD, DRILL, BARE_W, INSUL_W = 1.8, 1.0, 0.5, 0.3
OX, OY = 30.0, 40.0
FLAYER = {'F': {'fab': 'F.Fab', 'crt': 'F.CrtYd', 'cu': 'F.Cu'}, 'B': {'fab': 'B.Fab', 'crt': 'B.CrtYd', 'cu': 'B.Cu'}}


def bxy(node):
    x, y = P.hxy(node)
    return round(OX + x, 4), round(OY + y, 4)


def nxy(k):
    """Nœud au demi-pas (coordonnées doublées) -> mm sur la planche KiCad."""
    return round(OX + P.X0 + P.PITCH * k[0] / 2, 4), round(OY + P.Y0 + P.PITCH * k[1] / 2, 4)


def fp_rect(r, layer, width=0.1, key=''):
    return (f'(fp_rect (start {fmt(r[0])} {fmt(r[1])}) (end {fmt(r[2])} {fmt(r[3])}) '
            f'(stroke (width {width}) (type solid)) (fill none) (layer {q(layer)}) (uuid {uid(key)}))')


def fp_text(t, x, y, layer, size=1.0, key='', mirror=False):
    j = ' (justify mirror)' if mirror else ''
    return (f'(fp_text user {q(t)} (at {fmt(x)} {fmt(y)} 0) (layer {q(layer)}) (uuid {uid(key)}) '
            f'(effects (font (size {size} {size}) (thickness {fmt(size * 0.15)})){j}))')


def footprint_text(p, nets=None, placed=False):
    """Empreinte propre à la pièce, origine = broche 1. Placée en face B : géométrie
    réelle (déjà retournée) et couches B.* ; en bibliothèque : version de face avant."""
    ref, face = p['ref'], p['face']
    flip = face == 'B' and not placed            # bibliothèque : on « dé-retourne »
    side = 'B' if face == 'B' and placed else 'F'
    L = FLAYER[side]
    x1, y1 = P.hxy(p['pins'][0])
    sy = -1 if flip else 1          # KiCad retourne une empreinte haut/bas (y -> -y)
    local = [(P.hxy(n)[0] - x1, sy * (P.hxy(n)[1] - y1)) for n in p['pins']]
    body = P.body_rect(p)
    ys = sorted([sy * (body[1] - y1), sy * (body[3] - y1)])
    rel = (body[0] - x1, ys[0], body[2] - x1, ys[1])
    crt = (rel[0] - 0.25, rel[1] - 0.25, rel[2] + 0.25, rel[3] + 0.25)
    name = f'A7_{ref}'
    s = [f'(footprint {q(LIB + ":" + name if placed else name)}' + ('' if placed else ' (version 20241229) (generator "fieldmind_a7") (generator_version "a7")'),
         f'(layer {q(L["cu"])})']
    if placed:
        bx, by = bxy(p['pins'][0])
        s += [f'(uuid {uid("fp/" + ref)})', f'(at {fmt(bx)} {fmt(by)})']
    s += [f'(property "Reference" {q(ref if placed else "REF**")} (at {fmt((rel[0] + rel[2]) / 2)} {fmt((rel[1] + rel[3]) / 2)} 0) '
          f'(layer {q(L["fab"])}) (uuid {uid("fpref/" + ref)}) (effects (font (size 1 1) (thickness 0.15)){" (justify mirror)" if side == "B" else ""}))',
          f'(property "Value" {q(p["value"])} (at 0 {fmt(rel[3] + 1.5)} 0) (layer {q(L["fab"])}) (hide yes) '
          f'(uuid {uid("fpval/" + ref)}) (effects (font (size 1 1) (thickness 0.15)){" (justify mirror)" if side == "B" else ""}))']
    if placed:
        s += [f'(path {q(PCB_PATH_PREFIX[p["sheet"]] + "/" + uid("symbol/" + ref))})',
              f'(sheetname {q(SHEET_NAME[p["sheet"]])})', f'(sheetfile {q(SHEET_FILE[p["sheet"]])})']
    s.append('(attr through_hole)')
    s.append(fp_rect(rel, L['fab'], 0.12, f'{ref}/fab'))
    s.append(fp_rect(crt, L['crt'], 0.05, f'{ref}/crt'))
    for i, r in enumerate(P.socket_rects(p)):
        ys = sorted([sy * (r[1] - y1), sy * (r[3] - y1)])
        s.append(fp_rect((r[0] - x1, ys[0], r[2] - x1, ys[1]), L['fab'], 0.1, f'{ref}/plastic/{i}'))
    names = p.get('names') or [str(i + 1) for i in range(len(p['nets']))]
    for i, ((lx, ly), net) in enumerate(zip(local, p['nets']), 1):
        net_txt = ''
        if placed:
            n = pcb_net(nets[(ref, str(i))])
            net_txt = f'(net {NETID[n]} {q(n)}) '
        s.append(f'(pad {q(i)} thru_hole {"rect" if i == 1 else "circle"} (at {fmt(lx)} {fmt(ly)}) (size {PAD} {PAD}) '
                 f'(drill {DRILL}) (layers "*.Cu" "*.Mask") (remove_unused_layers no) {net_txt}'
                 f'(pinfunction {q(names[i - 1])}) (pintype "passive") (uuid {uid(f"pad/{ref}/{i}")}))')
    s.append(f'(model "${{KIPRJMOD}}/3d/{name}.wrl" (offset (xyz 0 0 0)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0)))')
    return '\n'.join(s + [')'])


def perf_footprint(owner, pinholes, placed=False):
    """Toutes les pastilles sans broche ; net quand un fil y est soudé."""
    name = 'A7_PERF'
    s = [f'(footprint {q(LIB + ":" + name if placed else name)}' + ('' if placed else ' (version 20241229) (generator "fieldmind_a7") (generator_version "a7")'),
         '(layer "F.Cu")']
    if placed:
        s += [f'(uuid {uid("fp/perf")})', f'(at {fmt(OX)} {fmt(OY)})']
    s += [f'(property "Reference" {q("PL1" if placed else "REF**")} (at 20 -2 0) (layer "F.Fab") '
          f'(uuid {uid("perfref")}) (effects (font (size 1.2 1.2) (thickness 0.18))))',
          f'(property "Value" "Perfboard 4x6 cm 14x20 trous, double face" (at 20 62 0) (layer "F.Fab") (hide yes) '
          f'(uuid {uid("perfval")}) (effects (font (size 1 1) (thickness 0.15))))',
          '(attr through_hole board_only exclude_from_pos_files exclude_from_bom)',
          fp_rect((0, 0, P.BOARD_W, P.BOARD_H), 'F.Fab', 0.1, 'perf/outline')]
    for r in range(P.ROWS):
        for c in range(P.COLS):
            k = (2 * c, 2 * r)
            if k in pinholes:
                continue
            x, y = P.hxy((c, r))
            net = f'(net {NETID[owner[k]]} {q(owner[k])}) ' if placed and k in owner else ''
            label = P.hole_name((c, r))
            s.append(f'(pad {q(label)} thru_hole circle (at {fmt(x)} {fmt(y)}) (size {PAD} {PAD}) (drill {DRILL}) '
                     f'(layers "*.Cu" "*.Mask") (remove_unused_layers no) {net}(uuid {uid("perfpad/" + label)}))')
    for i, (x, y) in enumerate([(2.5, 2.5), (P.BOARD_W - 2.5, 2.5), (2.5, P.BOARD_H - 2.5), (P.BOARD_W - 2.5, P.BOARD_H - 2.5)]):
        s.append(f'(pad "" np_thru_hole circle (at {fmt(x)} {fmt(y)}) (size 2 2) (drill 2) (layers "*.Cu" "*.Mask") '
                 f'(uuid {uid(f"perfmount/{i}")}))')
    for c in range(P.COLS):
        x, _ = P.hxy((c, 0))
        s.append(fp_text(P.LETTERS[c], x, 3.4, 'Dwgs.User', 0.9, f'perf/col/{c}'))
    for r in range(P.ROWS):
        _, y = P.hxy((0, r))
        s.append(fp_text(str(r + 1), 1.4, y, 'Dwgs.User', 0.8, f'perf/row/{r}'))
    return '\n'.join(s + [')'])


NETID = {}


def pcb_net(name):
    if name.startswith('unconnected-('):
        return 'unconnected-(' + name[len('unconnected-('):].replace('/', '{slash}')
    return name


def corners(nodes):
    out = [nodes[0]]
    for a, b, c in zip(nodes, nodes[1:], nodes[2:]):
        if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
            out.append(b)
    out.append(nodes[-1])
    return out


def build_pcb(R, owner, pin_nets):
    nets = sorted({pcb_net(n) for n in pin_nets.values()})
    NETID.clear(); NETID.update({n: i for i, n in enumerate(nets, 1)})
    n_in = max([t['layer'] for t in R.traces] + [0])
    inner = ''.join(f'({4 + 2 * (i - 1)} "In{i}.Cu" signal "B.Isolés{i}") ' for i in range(1, n_in + 1))
    layers = ('(layers (0 "F.Cu" signal) ' + inner + '(2 "B.Cu" signal) (5 "F.SilkS" user "F.Silkscreen") '
              '(7 "B.SilkS" user "B.Silkscreen") (1 "F.Mask" user) (3 "B.Mask" user) (17 "Dwgs.User" user "User.Drawings") '
              '(19 "Cmts.User" user "User.Comments") (25 "Edge.Cuts" user) (27 "Margin" user) (31 "F.CrtYd" user "F.Courtyard") '
              '(29 "B.CrtYd" user "B.Courtyard") (35 "F.Fab" user) (33 "B.Fab" user))')
    stack = ('(stackup (layer "F.SilkS" (type "Top Silk Screen") (color "White")) '
             '(layer "F.Mask" (type "Top Solder Mask") (color "Blue") (thickness 0.01)) '
             '(layer "F.Cu" (type "copper") (thickness 0.035)) '
             + ''.join(f'(layer "dielectric {i + 1}" (type "core") (color "FR4 natural") (thickness {round(1.51 / (n_in + 1), 3)}) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02)) '
                       + (f'(layer "In{i + 1}.Cu" (type "copper") (thickness 0.035)) ' if i < n_in else '') for i in range(n_in + 1)) +
             '(layer "B.Cu" (type "copper") (thickness 0.035)) '
             '(layer "B.Mask" (type "Bottom Solder Mask") (color "Blue") (thickness 0.01)) '
             '(layer "B.SilkS" (type "Bottom Silk Screen") (color "White")) (copper_finish "HAL SnPb") (dielectric_constraints no))')
    b = [f'(kicad_pcb (version 20241229) (generator "fieldmind_a7") (generator_version "a7") '
         f'(general (thickness 1.6) (legacy_teardrops no)) (paper "A4") '
         f'(title_block (title "FieldMind C5 A7 / une perfboard 4 x 6 cm, deux faces") (date {q(DATE)}) (rev {q(REV)}) '
         f'(comment 1 "F.Cu = ponts de soudure (dessus). B.Cu = fils nus dessous. In*.Cu = fils isolés dessous (couches = étages de croisement).") '
         f'(comment 2 "Un fil isolé passe par-dessus un autre fil à angle droit ; aucun fil côté écran."))',
         layers, f'(setup {stack} (pad_to_mask_clearance 0) (allow_soldermask_bridges_in_footprints no))',
         '(net 0 "")'] + [f'(net {i} {q(n)})' for n, i in NETID.items()]
    for p in P.PARTS:
        if p['face']:
            b.append(footprint_text(p, pin_nets, True))
    pinholes = set(R.pin)
    b.append(perf_footprint(owner, pinholes, True))
    b.append(f'(gr_rect (start {fmt(OX)} {fmt(OY)}) (end {fmt(OX + P.BOARD_W)} {fmt(OY + P.BOARD_H)}) '
             f'(stroke (width 0.1) (type solid)) (fill none) (layer "Edge.Cuts") (uuid {uid("edge")}))')
    for s_ in P.SUPPORTS:
        f = P.BYREF[s_['ref']]['face']
        for i, (x, y) in enumerate(P.support_points(s_)):
            b.append(f'(gr_circle (center {fmt(OX + x)} {fmt(OY + y)}) (end {fmt(OX + x + s_["d"] / 2)} {fmt(OY + y)}) '
                     f'(stroke (width 0.12) (type dash)) (fill none) (layer {q(FLAYER[f]["fab"])}) (uuid {uid(f"support/{s_["ref"]}/{i}")}))')
    for t in R.traces:
        cu = FLAYER[t['face']]['cu'] if t['face'] == 'F' or t['bare'] else f'In{t["layer"]}.Cu'
        w = BARE_W if t['bare'] else INSUL_W
        for i, (u, v) in enumerate(zip(corners(t['nodes']), corners(t['nodes'])[1:])):
            (x1, y1), (x2, y2) = nxy(u), nxy(v)
            b.append(f'(segment (start {fmt(x1)} {fmt(y1)}) (end {fmt(x2)} {fmt(y2)}) (width {w}) (layer {q(cu)}) '
                     f'(net {NETID[t["net"]]}) (uuid {uid(f"seg/{t["id"]}/{i}")}))')
        mx, my = nxy(t['nodes'][len(t['nodes']) // 2])
        b.append(f'(gr_text {q(t["id"])} (at {fmt(mx)} {fmt(my - 0.9)} 0) (layer "Cmts.User") (uuid {uid("tid/" + t["id"])}) '
                 f'(effects (font (size 0.7 0.7) (thickness 0.1))))')
    return '\n'.join(b + [')']) + '\n'


# ====================================================================== modèles 3D
COL = {'black': (0.08, 0.08, 0.09), 'gold': (0.85, 0.68, 0.25), 'grey': (0.30, 0.31, 0.33), 'silver': (0.75, 0.76, 0.78),
       'xiao': (0.10, 0.12, 0.14), 'blue': (0.10, 0.28, 0.65), 'purple': (0.42, 0.18, 0.55), 'glass': (0.03, 0.04, 0.06),
       'white': (0.92, 0.92, 0.88), 'beige': (0.86, 0.75, 0.55), 'cap': (0.80, 0.45, 0.15), 'elec': (0.08, 0.12, 0.35),
       'chip': (0.12, 0.12, 0.12), 'oled': (0.15, 0.35, 0.75)}


class Model:
    """Volumes simples en VRML (unité KiCad : 2,54 mm). Pour une empreinte de face B,
    KiCad retourne le modèle haut/bas (y -> -y, z -> -z) : on écrit donc y non inversé."""

    def __init__(self, origin, flip=False):
        self.ox, self.oy, self.sx, self.shapes = origin[0], origin[1], 1, []
        self.fy = 1 if flip else -1

    def _mat(self, color, transp=0.0):
        r, g, b = COL[color]
        return f'appearance Appearance {{ material Material {{ diffuseColor {r} {g} {b} specularColor 0.2 0.2 0.2 transparency {transp} }} }}'

    def box(self, x0, y0, x1, y1, z0, z1, color, transp=0.0):
        cx, cy, cz = self.sx * ((x0 + x1) / 2 - self.ox), (y0 + y1) / 2 - self.oy, (z0 + z1) / 2
        u = 2.54
        self.shapes.append(f'Transform {{ translation {cx / u:.4f} {self.fy * cy / u:.4f} {cz / u:.4f} children [ Shape {{ {self._mat(color, transp)} '
                           f'geometry Box {{ size {abs(x1 - x0) / u:.4f} {abs(y1 - y0) / u:.4f} {abs(z1 - z0) / u:.4f} }} }} ] }}')

    def cyl(self, x, y, z0, z1, r, color, axis='z'):
        u = 2.54
        if axis == 'z':
            t = f'translation {self.sx * (x - self.ox) / u:.4f} {self.fy * (y - self.oy) / u:.4f} {(z0 + z1) / 2 / u:.4f} rotation 1 0 0 1.5708'
        else:
            t = f'translation {self.sx * ((z0 + z1) / 2 - self.ox) / u:.4f} {self.fy * (y - self.oy) / u:.4f} {x / u:.4f} rotation 0 0 1 1.5708'
        self.shapes.append(f'Transform {{ {t} children [ Shape {{ {self._mat(color)} geometry Cylinder {{ radius {r / u:.4f} height {(z1 - z0) / u:.4f} }} }} ] }}')

    def text(self):
        return '#VRML V2.0 utf8\n' + '\n'.join(self.shapes) + '\n'


def model_for(p):
    x1, y1 = P.hxy(p['pins'][0])
    m = Model((x1, y1), flip=p['face'] == 'B')
    pts = [P.hxy(n) for n in p['pins']]
    if p['kind'] == 'module':
        for r in P.socket_rects(p):
            m.box(*r, 0, P.SOCKET_H, 'black')
            m.box(*r, P.SOCKET_H, P.MODULE_Z, 'black')
        for (x, y) in pts:
            m.box(x - 0.32, y - 0.32, x + 0.32, y + 0.32, P.MODULE_Z - 0.2, P.MODULE_Z + 2.4, 'gold')
        body = p['rect']
        mod = P.MODULES[p['module']]
        z0, z1 = P.MODULE_Z, P.MODULE_Z + mod['t']
        color = {'XIAO': 'xiao', 'OLED': 'oled', 'GY521': 'blue', 'BMP280': 'purple'}[p['module']]
        m.box(*body, z0, z1, color)
        cx, cy = (body[0] + body[2]) / 2, (body[1] + body[3]) / 2
        if p['module'] == 'XIAO':
            ux, uy = p['usb'][0]
            m.box(cx - 6.5, cy - 7.0, cx + 6.5, cy + 6.0, z1, z1 + 2.2, 'silver')
            m.box(ux - 4.5, uy - 1.0, ux + 4.5, uy + 6.5, z1, z1 + 3.2, 'silver')
        elif p['module'] == 'OLED':
            m.box(body[0] + 0.6, body[1] + 6.0, body[2] - 0.6, body[3] - 1.5, z1, z1 + 1.6, 'glass')
            m.box(body[0] + 2.0, body[1] + 8.5, body[2] - 2.0, body[3] - 5.0, z1 + 1.6, z1 + 1.65, 'oled', 0.2)
        elif p['module'] == 'GY521':
            m.box(cx - 2, cy - 2, cx + 2, cy + 2, z1, z1 + 0.9, 'chip')
        else:
            m.box(cx - 1.25, cy - 1.0, cx + 1.25, cy + 1.0, z1, z1 + 0.95, 'silver')
        for s_ in P.SUPPORTS:
            if s_['ref'] == p['ref']:
                for (x, y) in P.support_points(s_):
                    if p['ref'] == 'J2':
                        m.box(x - 1.5, y - 2.5, x + 1.5, y + 2.5, 0, P.MODULE_Z, 'white')
                    else:
                        m.cyl(x, y, 0, P.MODULE_Z, s_['d'] / 2, 'white')
    elif p['kind'] in ('header', 'jumper'):
        m.box(*P.plastic_rect(p), 0, 2.5, 'black')
        for (x, y) in pts:
            m.box(x - 0.32, y - 0.32, x + 0.32, y + 0.32, -3.0, 8.5, 'gold')
            if p['kind'] == 'header':
                m.box(x - 1.25, y - 1.25, x + 1.25, y + 1.25, 2.6, 16.5, 'grey')
    elif p['kind'] == 'resistor':
        (ax, ay), (bx, by) = pts[0], pts[-1]
        if abs(ay - by) < 1e-6:
            m.box(min(ax, bx), ay - 0.25, max(ax, bx), ay + 0.25, 1.25, 1.75, 'silver')
            m.cyl(1.6, ay, (ax + bx) / 2 - 3.15, (ax + bx) / 2 + 3.15, 1.25, 'beige', axis='x')
        else:
            m.box(ax - 0.25, min(ay, by), ax + 0.25, max(ay, by), 1.25, 1.75, 'silver')
            cy_ = (ay + by) / 2
            m.box(ax - 1.25, cy_ - 3.15, ax + 1.25, cy_ + 3.15, 0.4, 2.9, 'beige')
    elif p['kind'] == 'capacitor':
        (ax, ay), (bx, by) = pts
        cx, cy = (ax + bx) / 2, (ay + by) / 2
        w, h = (2.5, 1.25) if abs(ay - by) < 1e-6 else (1.25, 2.5)
        m.box(cx - w, cy - h, cx + w, cy + h, 1.5, 6.5, 'cap')
    elif p['kind'] == 'cpol':
        (ax, ay), (bx, by) = pts
        cx, cy = (ax + bx) / 2, (ay + by) / 2
        m.cyl(cx, cy, 0.5, 11.5, 2.5, 'elec')
        m.cyl(cx, cy, 11.5, 11.6, 2.0, 'silver')
    return m.text()


# ====================================================================== vues SVG
NETCOL = {'+3V3': '#c8353b', 'GND': '#26364a', 'I2C_SDA': '#0b84c6', 'I2C_SCL': '#11905a',
          'GNSS_RX': '#7a52b3', 'GNSS_TX': '#e07b14', 'VBUS_USB': '#a01e6a', 'EXT_D0': '#b98900',
          'EXT_D1': '#5f6b7a', 'EXT_D2': '#00a3a3', 'EXT_D3': '#7d9c1c', 'EXT_D8': '#c25a00', 'EXT_D9': '#8e3fc0', 'EXT_D10': '#3f6fd8'}
PART_FILL = {'module': '#2f8f62', 'header': '#111', 'jumper': '#111', 'resistor': '#e3c995', 'capacitor': '#d27d2c', 'cpol': '#1f2f5c'}


def face_changes(R):
    """Trous reliés à une broche du XIAO par un pont de soudure (face dessus)."""
    return {k for b in R.bridges for k in (b['nodes'][0], b['nodes'][-1]) if k not in R.pin}


def svg_face(R, face, scale, ox, oy):
    """Une face vue de son côté : dessus normale, dessous en miroir gauche/droite."""
    mirror = face == 'B'
    vias = face_changes(R)
    X = lambda x: ox + (P.BOARD_W - x if mirror else x) * scale
    Y = lambda y: oy + y * scale
    s = [f'<rect x="{ox}" y="{oy}" width="{P.BOARD_W * scale}" height="{P.BOARD_H * scale}" rx="6" fill="#1f4f9a" stroke="#0e2a55" stroke-width="2"/>']
    for (mx, my) in [(2.5, 2.5), (37.5, 2.5), (2.5, 57.5), (37.5, 57.5)]:
        s.append(f'<circle cx="{X(mx)}" cy="{Y(my)}" r="{scale}" fill="#f4f6f8"/>')
    for c in range(P.COLS):
        s.append(f'<text x="{X(P.hxy((c, 0))[0])}" y="{Y(2.6)}" class="lab" text-anchor="middle">{P.LETTERS[c]}</text>')
    for r in range(P.ROWS):
        s.append(f'<text x="{X(1.4)}" y="{Y(P.hxy((0, r))[1]) + 3}" class="lab" text-anchor="middle">{r + 1}</text>')
    for r in range(P.ROWS):
        for c in range(P.COLS):
            x, y = P.hxy((c, r))
            s.append(f'<circle cx="{X(x)}" cy="{Y(y)}" r="{0.85 * scale}" fill="#c9a64b"/><circle cx="{X(x)}" cy="{Y(y)}" r="{0.42 * scale}" fill="#0b1c35"/>')
    for p in P.PARTS:
        if not p['face']:
            continue
        r = P.body_rect(p); x0, x1 = sorted([X(r[0]), X(r[2])])
        if p['face'] == face:
            op = 0.5 if p['kind'] == 'module' else 0.9
            s.append(f'<g><rect x="{x0}" y="{Y(r[1])}" width="{x1 - x0}" height="{(r[3] - r[1]) * scale}" rx="3" '
                     f'fill="{PART_FILL[p["kind"]]}" fill-opacity="{op}" stroke="#0a0f14" stroke-width="1.5"/>')
            lab = p['ref'] + {'XIAO': ' XIAO C5', 'OLED': ' OLED', 'GY521': ' GY-521', 'BMP280': ' BMP280'}.get(p.get('module'), '')
            s.append(f'<text x="{(x0 + x1) / 2}" y="{Y((r[1] + r[3]) / 2) + 5}" class="ref" text-anchor="middle">{escape(lab)}</text></g>')
        else:
            s.append(f'<rect x="{x0}" y="{Y(r[1])}" width="{x1 - x0}" height="{(r[3] - r[1]) * scale}" rx="3" fill="none" '
                     f'stroke="#c8d4e4" stroke-width="1" stroke-dasharray="5 4"/>')
    for s_ in P.SUPPORTS:
        if P.BYREF[s_['ref']]['face'] == face:
            for (x, y) in P.support_points(s_):
                s.append(f'<circle cx="{X(x)}" cy="{Y(y)}" r="{s_["d"] / 2 * scale}" fill="#f2f2ea" fill-opacity=".85" stroke="#333" stroke-dasharray="4 2"/>')
    for t in sorted(R.traces, key=lambda t: (not t['bare'], t.get('layer', 0))):   # isolés par-dessus
        if t['face'] != face:
            continue
        col = NETCOL[t['net']]
        pts = ' '.join(f'{X(P.X0 + P.PITCH * a / 2)},{Y(P.Y0 + P.PITCH * b / 2)}' for a, b in t['nodes'])
        w = 0.6 if t['bare'] else 0.38
        dash = '' if t['bare'] else f' stroke-dasharray="{0.9 * scale} {0.45 * scale}"'
        s.append(f'<g data-net="{escape(t["net"])}"><title>{t["id"]} {escape(t["net"])} ({"nu" if t["bare"] else "isolé"})</title>'
                 f'<polyline points="{pts}" fill="none" stroke="#f7f4e8" stroke-width="{(w + 0.35) * scale}" stroke-linejoin="round" stroke-linecap="round"/>'
                 f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="{w * scale}" stroke-linejoin="round" stroke-linecap="round"{dash}/>')
        for k in (t['nodes'][0], t['nodes'][-1]):
            x, y = P.X0 + P.PITCH * k[0] / 2, P.Y0 + P.PITCH * k[1] / 2
            via = k in vias
            s.append(f'<circle cx="{X(x)}" cy="{Y(y)}" r="{0.55 * scale}" fill="{"#ffd84a" if via else "#fff"}" stroke="{col}" stroke-width="2"/>')
        mid = t['nodes'][len(t['nodes']) // 2]
        s.append(f'<text x="{X(P.X0 + P.PITCH * mid[0] / 2) + 4}" y="{Y(P.Y0 + P.PITCH * mid[1] / 2) - 5}" class="tid" fill="{col}">{t["id"]}</text></g>')
    for k, (ref, i, net) in R.pin.items():
        x, y = P.X0 + P.PITCH * k[0] / 2, P.Y0 + P.PITCH * k[1] / 2
        mine = P.BYREF[ref]['face'] == face
        if i == 1:
            s.append(f'<rect x="{X(x) - 0.75 * scale}" y="{Y(y) - 0.75 * scale}" width="{1.5 * scale}" height="{1.5 * scale}" fill="none" stroke="#fff" stroke-width="{2 if mine else 1}"/>')
        else:
            s.append(f'<circle cx="{X(x)}" cy="{Y(y)}" r="{0.75 * scale}" fill="none" stroke="#fff" stroke-width="{1.6 if mine else 0.8}"/>')
        if not net:
            s.append(f'<text x="{X(x)}" y="{Y(y) + 4}" class="nc" text-anchor="middle">×</text>')
    return '\n'.join(s)


def svg_view(R):
    scale, W, H = 12, 1560, 1010
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
         '<style>text{font-family:Helvetica,Arial,sans-serif;fill:#1d2733}.lab{font-size:10px;fill:#e8eef6}'
         '.ref{font-size:13px;font-weight:bold;fill:#fff}.tid{font-size:10px;font-weight:bold}.nc{font-size:14px;fill:#ffb3b3}</style>',
         f'<rect width="{W}" height="{H}" fill="#f3f5f8"/>',
         '<text x="40" y="40" font-size="26" font-weight="bold">FieldMind C5 · A7 · une perfboard 4 × 6 cm, tous les fils dessous</text>',
         '<text x="40" y="68" font-size="15">Dessus : aucun fil, seulement des ponts de soudure (S..) du XIAO vers le trou voisin (rond jaune). '
         'Dessous : trait plein = fil nu ; tirets = fil isolé, qui passe par-dessus.</text>']
    for face, x, title in [('F', 40, 'FACE DESSUS (côté écran)'), ('B', 590, 'FACE DESSOUS, vue de dessous (miroir)')]:
        s.append(f'<text x="{x}" y="112" font-size="19" font-weight="bold">{title}</text>')
        s.append(svg_face(R, face, scale, x, 130))
    lx = 1140
    s.append(f'<text x="{lx}" y="150" font-size="17" font-weight="bold">Liaisons</text>')
    for i, (net, col) in enumerate(NETCOL.items()):
        y = 180 + i * 26
        s.append(f'<line x1="{lx}" y1="{y - 5}" x2="{lx + 32}" y2="{y - 5}" stroke="{col}" stroke-width="7" stroke-linecap="round"/>'
                 f'<text x="{lx + 42}" y="{y}" font-size="14">{escape(net)}</text>')
    notes = ['Ponts (dessus) : goutte de soudure entre la', 'broche du XIAO et le trou voisin, sans fil.',
             'Rectangles pleins : cartes de la face montrée.', 'Rectangles en tirets : cartes de l’autre face.',
             'Carré = broche 1 ; × = broche non câblée.', 'Noms de trous identiques sur les deux vues',
             '(colonnes A…N et rangées 1…20 vues du dessus).', 'Souder chaque face avant d’enficher', 'les cartes qui la recouvrent.',
             'Aucun essai physique réalisé.']
    for i, t in enumerate(notes):
        s.append(f'<text x="{lx}" y="{200 + len(NETCOL) * 26 + i * 22}" font-size="13">{escape(t)}</text>')
    return '\n'.join(s + ['</svg>']) + '\n'


# ====================================================================== tables
def hole_label(k):
    return P.hole_name((k[0] // 2, k[1] // 2))


def route_text(t):
    out = []
    for k in corners(t['nodes']):
        if k[0] % 2 == 0 and k[1] % 2 == 0:
            out.append(hole_label(k))
        else:
            c, r = k[0] / 2, k[1] / 2
            out.append(f'({c + 1:g};{r + 1:g})')
    return ' > '.join(out)


def kind_of(t):
    if t.get('bridge'):
        return 'pont de soudure'
    if t.get('rail'):
        return 'rail nu'
    return 'nu' if t['bare'] else 'isolé'


def connections():
    """Pour chaque broche reliée : net et autres broches du même net."""
    members = {}
    for p in P.PARTS:
        if p['face']:
            for i, net in enumerate(p['nets'], 1):
                if net:
                    members.setdefault(net, []).append((p['ref'], i, (p.get('names') or [str(i)] * 99)[i - 1]))
    rows = []
    for p in P.PARTS:
        if not p['face']:
            continue
        for i, net in enumerate(p['nets'], 1):
            name = (p.get('names') or [str(i)] * 99)[i - 1]
            hole = P.hole_name(p['pins'][i - 1])
            others = [f'{r}.{n} ({nm})' for r, n, nm in members.get(net, []) if (r, n) != (p['ref'], i)] if net else []
            rows.append((p['ref'], p['value'], i, name, hole, net or 'non câblé', ', '.join(others)))
    return rows


def write_tables(R, owner):
    with (BASE / 'connexions_c5_a7.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['composant', 'carte', 'broche', 'nom', 'trou', 'net', 'relie_a'])
        w.writerows(connections())
    with (BASE / 'fils_c5_a7.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['fil', 'face', 'net', 'type', 'extremite_1', 'extremite_2', 'trajet_coins', 'longueur_mm'])
        for t in R.traces:
            L = (len(t['nodes']) - 1) * P.PITCH / 2
            w.writerow([t['id'], 'dessus' if t['face'] == 'F' else 'dessous', t['net'], kind_of(t),
                        hole_label(t['nodes'][0]), hole_label(t['nodes'][-1]), route_text(t), round(L, 1)])
    ends = {}
    for t in R.traces:
        for k in (t['nodes'][0], t['nodes'][-1]):
            ends.setdefault(k, []).append(f'{t["id"]}/{t["face"]}')
    with (BASE / 'occupation_trous_c5_a7.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['trou', 'composant', 'broche', 'face_composant', 'net', 'extremites_de_fil'])
        for r in range(P.ROWS):
            for c in range(P.COLS):
                k = (2 * c, 2 * r)
                pin = R.pin.get(k)
                if not pin and k not in owner:
                    continue
                w.writerow([P.hole_name((c, r)), pin[0] if pin else 'pastille', pin[1] if pin else '',
                            P.BYREF[pin[0]]['face'] if pin else '', (pin[2] if pin else owner.get(k)) or 'non câblé',
                            ' '.join(ends.get(k, []))])


def guide_html(view, R):
    buttons = ''.join(f'<button data-select="{escape(n)}" style="border-color:{c}">{escape(n)}</button>' for n, c in NETCOL.items())
    rows = ''.join(f'<tr data-net="{escape(t["net"])}"><td>{t["id"]}</td><td>{"dessus" if t["face"] == "F" else "dessous"}</td>'
                   f'<td>{escape(t["net"])}</td><td>{kind_of(t)}</td><td>{escape(route_text(t))}</td></tr>' for t in R.traces)
    conn = ''.join(f'<tr data-net="{escape(net)}"><td>{ref}</td><td>{escape(val)}</td><td>{i}</td><td>{escape(name)}</td><td>{hole}</td>'
                   f'<td>{escape(net)}</td><td>{escape(oth)}</td></tr>' for ref, val, i, name, hole, net, oth in connections())
    return ('<!doctype html><html lang="fr"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
            '<title>FieldMind C5 A7 — câblage</title><style>body{font:16px system-ui;margin:20px;background:#f3f5f8;color:#1d2733}'
            'button{padding:7px 11px;border:2px solid;border-radius:7px;background:#fff;cursor:pointer;margin:3px}'
            'button.active{background:#1d2733;color:#fff}svg{width:100%;height:auto}table{border-collapse:collapse;background:#fff;width:100%}'
            'td,th{padding:6px 8px;border-bottom:1px solid #d8e0e8;text-align:left;font-size:14px}[data-net].dim{opacity:.08}'
            'nav{position:sticky;top:0;background:#f3f5f8;padding:8px 0;z-index:2}</style>'
            '<h1>FieldMind C5 · A7 · une perfboard, tous les fils dessous</h1><p>Choisir une liaison pour la suivre. '
            'Dessus : ponts de soudure seulement. Dessous : trait plein = fil nu, tirets = fil isolé (passe par-dessus).</p>'
            f'<nav><button id="all">Tout</button>{buttons}</nav>{view}'
            '<h2>Connexions par composant</h2><table><thead><tr><th>Composant</th><th>Carte</th><th>Broche</th><th>Nom</th>'
            f'<th>Trou</th><th>Net</th><th>Relié à</th></tr></thead><tbody>{conn}</tbody></table>'
            '<h2>Fils et ponts</h2><table><thead><tr><th>Fil</th><th>Face</th>'
            f'<th>Net</th><th>Type</th><th>Trajet (coins)</th></tr></thead><tbody>{rows}</tbody></table><script>'
            "const choose=n=>{document.querySelectorAll('[data-net]').forEach(e=>e.classList.toggle('dim',!!n&&e.dataset.net!==n));"
            "document.querySelectorAll('[data-select]').forEach(e=>e.classList.toggle('active',e.dataset.select===n));};"
            "document.querySelectorAll('[data-select]').forEach(b=>b.onclick=()=>choose(b.classList.contains('active')?null:b.dataset.select));"
            "document.getElementById('all').onclick=()=>choose(null);</script></html>\n")


# ====================================================================== principal
def run(*args):
    r = subprocess.run([KICAD_CLI, *map(str, args)], capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(f'kicad-cli {" ".join(map(str, args))}\n{r.stdout}\n{r.stderr}')
    return r.stdout


def main():
    force = '--force' in sys.argv
    sch = CAD / (PROJECT + '.kicad_sch')
    if sch.exists() and '(generator "eeschema")' in sch.read_text()[:400] and not force:
        raise SystemExit('A7 déjà enregistré par KiCad 10 : relancer avec --force pour écraser (éditions manuelles perdues).')
    R, report = P.analyse()
    owner = {}
    for t in R.traces:
        for k in t['nodes']:
            if k[0] % 2 == 0 and k[1] % 2 == 0:
                owner[k] = t['net']
    CAD.mkdir(parents=True, exist_ok=True); OUT.mkdir(parents=True, exist_ok=True)
    (CAD / '3d').mkdir(exist_ok=True)
    pretty = CAD / (LIB + '.pretty'); pretty.mkdir(exist_ok=True)
    keys = sorted(SYMS)
    (CAD / (LIB + '.kicad_sym')).write_text('(kicad_symbol_lib (version 20241209) (generator "fieldmind_a7") (generator_version "a7")\n'
                                            + '\n'.join(lib_entry(k, False) for k in keys) + '\n)\n')
    (CAD / 'sym-lib-table').write_text(f'(sym_lib_table (version 7) (lib (name {q(LIB)}) (type "KiCad") (uri "${{KIPRJMOD}}/{LIB}.kicad_sym") (options "") (descr "FieldMind A7 local")))\n')
    (CAD / 'fp-lib-table').write_text(f'(fp_lib_table (version 7) (lib (name {q(LIB)}) (type "KiCad") (uri "${{KIPRJMOD}}/{LIB}.pretty") (options "") (descr "Empreintes perfboard A7")))\n')
    for key, text in build_schematics().items():
        (CAD / SHEET_FILE[key]).write_text(text)
    pro = {'meta': {'filename': PROJECT + '.kicad_pro', 'version': 3},
           # Trous de perfboard sous les cartes surélevées : attendus.
           'board': {'design_settings': {'rule_severities': {'pth_inside_courtyard': 'ignore', 'npth_inside_courtyard': 'ignore'},
                                         'rules': {'min_clearance': 0.2, 'min_track_width': 0.2, 'min_through_hole_diameter': 0.3}}},
           'sheets': [[ROOT, 'Root'], [SHEET_B, 'Capteurs I2C et écran']]}
    (CAD / (PROJECT + '.kicad_pro')).write_text(json.dumps(pro, indent=2, ensure_ascii=False) + '\n')
    net_xml = CAD / 'netlist_tmp.xml'
    run('sch', 'export', 'netlist', '--format', 'kicadxml', '-o', net_xml, sch)
    pin_nets = {}
    for n in ET.parse(net_xml).getroot().findall('./nets/net'):
        for node in n.findall('node'):
            pin_nets[(node.attrib['ref'], node.attrib['pin'])] = n.attrib['name']
    net_xml.unlink()
    for p in P.PARTS:
        for i, net in enumerate(p['nets'], 1):
            got = pin_nets.get((p['ref'], str(i)))
            if net:
                assert got == net or (net.startswith('KEY_BRANCH') and got.endswith('/' + net)), ('schéma != plan', p['ref'], i, got, net)
            else:
                assert got and got.startswith('unconnected-'), (p['ref'], i, got)
    for p in P.PARTS:
        if p['face']:
            (pretty / f'A7_{p["ref"]}.kicad_mod').write_text(footprint_text(p) + '\n')
            (CAD / '3d' / f'A7_{p["ref"]}.wrl').write_text(model_for(p))
    (pretty / 'A7_PERF.kicad_mod').write_text(perf_footprint(owner, set(R.pin)) + '\n')
    (CAD / (PROJECT + '.kicad_pcb')).write_text(build_pcb(R, owner, pin_nets))
    for f in SHEET_FILE.values():
        run('sch', 'upgrade', '--force', CAD / f)
    run('pcb', 'upgrade', '--force', CAD / (PROJECT + '.kicad_pcb'))
    run('sym', 'upgrade', '--force', CAD / (LIB + '.kicad_sym'))
    run('fp', 'upgrade', '--force', pretty)
    view = svg_view(R)
    (OUT / 'deux_faces.svg').write_text(view)
    (OUT / 'guide_cablage.html').write_text(guide_html(view, R))
    write_tables(R, owner)
    manifest = dict(revision='A7 C5 / une perfboard 4 x 6 cm, tous les fils dessous', date=DATE, manufacture_ready=False, physical_tests=False,
                    board=dict(size_mm=[P.BOARD_W, P.BOARD_H], grid=[P.COLS, P.ROWS], pitch_mm=P.PITCH, first_hole_mm=[P.X0, P.Y0],
                               plated_through_holes_assumed=True),
                    parts=P.PARTS, traces=[{k: v for k, v in t.items()} for t in R.traces],
                    supports=P.SUPPORTS, modules=P.MODULES, module_dimensions_measured=False,
                    module_pin_order_verified=False, checks=report)
    (CAD / 'cablage_a7.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=1, default=list) + '\n')
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
