#!/usr/bin/env python3
"""Contrôle indépendant de A7 (une plaque, fils dessous, ponts de soudure dessus) à partir des fichiers KiCad 10.

ERC, netlist, DRC avec parité ; netlist == plan ; PCB relu : pastilles aux bons
trous, segments F.Cu/B.Cu == fils du plan (au demi-pas) ; une extrémité au plus
par pastille et par face ; DRC sans aucune violation ni liaison ouverte ; format
KiCad 10. Écrit validation/{erc,drc}_c5_a7.*, netlist_c5_a7.xml, controles_c5_a7.json.
"""
import json
import re
import subprocess
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

import plan_c5_a7 as P

B = Path(__file__).resolve().parents[1]
C = B / 'cad/perfboard_c5_a7'
V = B / 'validation'
SCH, PCB = C / 'fieldmind_c5_a7.kicad_sch', C / 'fieldmind_c5_a7.kicad_pcb'
K = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
OX, OY = 30.0, 40.0


def cli(*a):
    r = subprocess.run([K, *map(str, a)], capture_output=True, text=True)
    return r.stdout + r.stderr


def parse(s):
    stack, root = [], None
    for t in re.findall(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()]+', s):
        if t == '(':
            n = []
            if stack:
                stack[-1].append(n)
            else:
                root = n
            stack.append(n)
        elif t == ')':
            stack.pop()
        else:
            stack[-1].append(json.loads(t) if t.startswith('"') else t)
    return root


def kids(n, k):
    return [v for v in n if isinstance(v, list) and v and v[0] == k]


def first(n, k):
    return kids(n, k)[0]


def to_node(x, y):
    i, j = (x - OX - P.X0) / (P.PITCH / 2), (y - OY - P.Y0) / (P.PITCH / 2)
    assert abs(i - round(i)) < 1e-3 and abs(j - round(j)) < 1e-3, (x, y)
    return round(i), round(j)


R, plan_rep = P.analyse()
rep = {'revision': 'A7 C5 / une perfboard 4 x 6 cm, deux faces'}
cli('sch', 'erc', '--severity-all', '-o', V / 'erc_c5_a7.rpt', SCH)
cli('sch', 'export', 'netlist', '--format', 'kicadxml', '-o', V / 'netlist_c5_a7.xml', SCH)
cli('pcb', 'drc', '--severity-all', '--schematic-parity', '--format', 'json', '-o', V / 'drc_c5_a7.json', PCB)
cli('pcb', 'drc', '--severity-all', '--schematic-parity', '-o', V / 'drc_c5_a7.rpt', PCB)
assert 'ERC messages: 0  Errors 0  Warnings 0' in (V / 'erc_c5_a7.rpt').read_text()

root = ET.parse(V / 'netlist_c5_a7.xml').getroot()
pin_net = {(nd.attrib['ref'], nd.attrib['pin']): n.attrib['name'] for n in root.findall('./nets/net') for nd in n.findall('node')}
for p in P.PARTS:
    for i, net in enumerate(p['nets'], 1):
        got = pin_net[(p['ref'], str(i))]
        assert (got.startswith('unconnected-') if net is None else got == net or got.endswith('/' + net)), (p['ref'], i, got, net)
assert len(root.findall('./components/comp')) == len(P.PARTS)

board = parse(PCB.read_text())
pads = {}
for fp in kids(board, 'footprint'):
    ref = next(p[2] for p in kids(fp, 'property') if p[1] == 'Reference')
    layer = first(fp, 'layer')[1]
    part = P.BYREF.get(ref)
    if part:
        assert layer == ('B.Cu' if part['face'] == 'B' else 'F.Cu'), (ref, layer)
    bx, by = map(float, first(fp, 'at')[1:3])
    for pad in kids(fp, 'pad'):
        if pad[2] == 'np_thru_hole':
            continue
        at = first(pad, 'at')
        k = to_node(bx + float(at[1]), by + float(at[2]))
        assert k[0] % 2 == 0 and k[1] % 2 == 0 and k not in pads, (ref, pad[1], k)
        nets = kids(pad, 'net')
        pads[k] = (ref, pad[1], nets[0][-1] if nets else None)
assert len(pads) == P.COLS * P.ROWS
for k, (ref, i, net) in R.pin.items():
    assert pads[k][:2] == (ref, str(i)), (k, pads[k], ref, i)
owner = {}
for t in R.traces:
    for k in t['nodes']:
        if k[0] % 2 == 0 and k[1] % 2 == 0:
            owner[k] = t['net']
for k, (ref, i, net) in pads.items():
    if k in owner:
        assert net == owner[k], (k, net, owner[k])
    elif ref == 'PL1':
        assert net is None, (k, net)

native = Counter()
top_segments = []
for seg in kids(board, 'segment'):
    layer = first(seg, 'layer')[1]
    a, b = to_node(*map(float, first(seg, 'start')[1:])), to_node(*map(float, first(seg, 'end')[1:]))
    assert a[0] == b[0] or a[1] == b[1]
    net = first(seg, 'net')[-1]
    if a[0] == b[0]:
        cells = [(a[0], j) for j in range(min(a[1], b[1]), max(a[1], b[1]) + 1)]
    else:
        cells = [(i, a[1]) for i in range(min(a[0], b[0]), max(a[0], b[0]) + 1)]
    for u, v in zip(cells, cells[1:]):
        face = 'F' if layer == 'F.Cu' else 'B'          # In*.Cu = fils isolés de la face dessous
        native[(face, net, frozenset((u, v)))] += 1
    if layer == 'F.Cu':
        top_segments.append((a, b))
plan_edges = Counter((t['face'], t['net'], frozenset((u, v))) for t in R.traces for u, v in zip(t['nodes'], t['nodes'][1:]))
assert native == plan_edges, (native - plan_edges, plan_edges - native)
assert not kids(board, 'via') and not kids(board, 'zone')

ends = Counter((k, t['face']) for t in R.traces for k in (t['nodes'][0], t['nodes'][-1]))
assert sum(1 for t in R.traces if t['face'] == 'F' and not t.get('bridge')) == 0
assert max(ends.values()) == 1
drc = json.loads((V / 'drc_c5_a7.json').read_text())
assert not drc.get('violations') and not drc.get('unconnected_items') and not drc.get('schematic_parity'), \
    ([v['description'] for v in drc.get('violations', [])], drc.get('unconnected_items'), drc.get('schematic_parity'))

fmt = {}
for f in [*C.glob('*.kicad_sch'), PCB, C / 'FieldMind_C5_A7.kicad_sym', *sorted((C / 'FieldMind_C5_A7.pretty').glob('*.kicad_mod'))]:
    head = f.read_text()[:400]
    assert re.search(r'\(generator_version "([^"]+)"\)', head).group(1) == '10.0', f.name
    fmt[f.name] = re.search(r'\(version (\d+)\)', head).group(1)
assert all(fmt[f.name] == '20260306' for f in C.glob('*.kicad_sch')) and fmt[PCB.name] == '20260206'

# Toutes les broches du XIAO sont reliées : GPIO libres sortis sur J6.
xiao = P.BYREF['U1']
xiao_open = [xiao['names'][i - 1] for i in range(1, 15) if pin_net[('U1', str(i))].startswith('unconnected-')]
assert not xiao_open, xiao_open
j6 = {xiao['names'][i]: pin_net[('U1', str(i + 1))] for i in range(14)}
j6_map = {}
for ref in ('J6', 'J8'):
    for i, name in enumerate(P.BYREF[ref]['names'], 1):
        j6_map[f'{ref}.{i} {name}'] = pin_net[(ref, str(i))]
        if name in j6:
            assert j6[name] == pin_net[(ref, str(i))], (ref, name)
# Face dessus : uniquement des ponts de soudure (un pas ou deux, sur une rangée, autour d'une broche du XIAO)
for a, b in top_segments:
    assert a[1] == b[1] and abs(a[0] - b[0]) <= 4, ('segment F.Cu autre qu’un pont', a, b)
    assert any(k in R.pin and R.pin[k][0] == 'U1' for k in [(x, a[1]) for x in range(min(a[0], b[0]), max(a[0], b[0]) + 1)]), (a, b)
assert len(top_segments) == len(R.bridges)
rep.update(erc_errors=0, erc_warnings=0, schematic_components=len(P.PARTS),
           board_components=sum(1 for p in P.PARTS if p['face']),
           components_face_top=[p['ref'] for p in P.PARTS if p['face'] == 'F'],
           components_face_bottom=[p['ref'] for p in P.PARTS if p['face'] == 'B'],
           perfboard_holes_modelled=len(pads), wires=plan_rep['wires'], bare_wires=plan_rep['bare'],
           insulated_wires=plan_rep['insulated'], wires_top=plan_rep['wires_face_top'], wires_bottom=plan_rep['wires'],
           solder_bridges_top=plan_rep['solder_bridges_top'], crossings=plan_rep['crossings'],
           wire_ends=plan_rep['wire_ends'], max_wire_ends_per_pad_and_face=1,
           drc_violations=0, drc_unconnected=0, drc_schematic_parity_issues=0, pcb_matches_plan=True, netlist_matches_plan=True,
           file_versions=fmt, smallest_body_gaps_mm=plan_rep['smallest_gaps'],
           plated_through_holes_assumed=True, module_dimensions_measured=False, module_pin_order_verified=False,
           xiao_pins_unconnected=xiao_open, extension_pinout=j6_map,
           physical_tests=False, manufacture_ready=False,
           caveat='Contrôles de CAO seulement. Trous métallisés à vérifier ; cotes et brochages des cartes à confirmer.')
(V / 'controles_c5_a7.json').write_text(json.dumps(rep, ensure_ascii=False, indent=1) + '\n')
print(json.dumps({k: v for k, v in rep.items() if k != 'file_versions'}, ensure_ascii=False, indent=1))
