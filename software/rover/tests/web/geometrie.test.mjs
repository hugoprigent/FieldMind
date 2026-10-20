// Tests de la logique pure du poste (node --test software/rover/tests/web).
import { test } from 'node:test';
import assert from 'node:assert/strict';

import {
  ModeleCamera, PROFIL_DEFAUT, DEG, poseSurArc, enveloppe, traverse, decoderXY, cadreContenu,
  completerProfil, emprise, longueurPrevue,
} from '../../fieldmind/web/js/geometrie.js';

const proche = (a, b, tol = 1e-6) => assert.ok(Math.abs(a - b) <= tol, `${a} ≠ ${b} (±${tol})`);
const cam = PROFIL_DEFAUT.cameras;

test("l'axe optique touche le sol à h / tan(inclinaison) devant la caméra", () => {
  const m = new ModeleCamera(cam, 0);
  const d = cam.z_m / Math.tan(cam.inclinaison_deg * DEG);
  const [u, v] = m.solVersPixel(cam.x_m + d, 0);
  proche(u, 0.5);
  proche(v, 0.5);
});

test('pixel → sol → pixel revient au même point (caméras gauche et droite)', () => {
  for (const cote of [1, -1]) {
    const m = new ModeleCamera(cam, cote);
    for (const [u, v] of [[0.5, 0.8], [0.1, 0.95], [0.9, 0.6], [0.3, 0.45]]) {
      const sol = m.pixelVersSol(u, v);
      assert.ok(sol, `pas de sol pour ${u}, ${v}`);
      const [u2, v2] = m.solVersPixel(...sol);
      proche(u2, u, 1e-9);
      proche(v2, v, 1e-9);
    }
  }
});

test('au-dessus de l’horizon, pas de point au sol ; un point lointain tend vers l’horizon', () => {
  const m = new ModeleCamera(cam, 0);
  assert.equal(m.pixelVersSol(0.5, m.horizon() - 0.01), null);
  const [, v] = m.solVersPixel(1000, 0);
  proche(v, m.horizon(), 1e-3);
});

test('caméra gauche décalée vers y > 0 : un point droit devant le rover apparaît à droite de son image', () => {
  const gauche = new ModeleCamera(cam, 1), droite = new ModeleCamera(cam, -1);
  assert.ok(gauche.solVersPixel(2, 0)[0] > 0.5);
  assert.ok(droite.solVersPixel(2, 0)[0] < 0.5);
});

test('une polyligne qui passe derrière la caméra est coupée au plan proche', () => {
  const m = new ModeleCamera(cam, 0);
  const troncons = m.projeterPolyligne([[3, 0], [0, 0], [-2, 0]]);
  assert.equal(troncons.length, 1);
  for (const [u, v] of troncons[0]) assert.ok(Number.isFinite(u) && Number.isFinite(v));
  assert.deepEqual(m.projeterPolyligne([[0, 0.5], [-1, 0.5]]), []);
});

test('arc : un quart de tour de rayon R mène en (R, R) avec un cap de 90°', () => {
  const R = 1.5, p = poseSurArc((Math.PI * R) / 2, 1 / R);
  proche(p.x, R, 1e-9);
  proche(p.y, R, 1e-9);
  proche(p.cap, Math.PI / 2, 1e-9);
  const droit = poseSurArc(2, 0);
  assert.deepEqual([droit.x, droit.y, droit.cap], [2, 0, 0]);
});

test('enveloppe en ligne droite : deux bords parallèles à ±demi-largeur depuis le nez', () => {
  const env = enveloppe({ braquage: 0, empattement: 0.38, xBord: 0.46, demiLargeur: 0.225, longueur: 2 });
  proche(env.gauche[0][0], 0.46);
  proche(env.gauche.at(-1)[0], 2.46);
  for (const [, y] of env.gauche) proche(y, 0.225);
  for (const [, y] of env.droite) proche(y, -0.225);
});

test('braquage à gauche : la trajectoire part vers y > 0 ; en marche arrière elle passe derrière', () => {
  const avant = enveloppe({ braquage: 20 * DEG, empattement: 0.38, xBord: 0.46, demiLargeur: 0.2, longueur: 1.5 });
  assert.ok(avant.gauche.at(-1)[1] > 0.5);
  const arriere = enveloppe({ braquage: 20 * DEG, empattement: 0.38, xBord: -0.11, demiLargeur: 0.2, longueur: 1, sens: -1 });
  assert.ok(arriere.gauche.at(-1)[0] < -0.5);
  const [a, b] = traverse({ braquage: 0, empattement: 0.38, xBord: 0.46, demiLargeur: 0.2, s: 1 });
  assert.deepEqual([a[0], b[0]], [1.46, 1.46]);
});

test('scan : centimètres entiers aplatis → mètres', () => {
  const p = decoderXY([120, -3, 118, 0, 7]);
  assert.equal(p.length, 4);
  proche(p[0], 1.2);
  proche(p[1], -0.03);
  assert.equal(decoderXY(null).length, 0);
});

test('image en object-fit: contain : cadre centré', () => {
  assert.deepEqual(cadreContenu(1000, 1000, 1280, 720), { x: 0, y: 218.75, l: 1000, h: 562.5 });
});

test('profil complété par défaut, emprise et longueur prévue bornée', () => {
  const p = completerProfil({ geometrie: { empattement_m: 0.4 } });
  assert.equal(p.geometrie.empattement_m, 0.4);
  assert.equal(p.geometrie.braquage_max_deg, 31);
  assert.equal(emprise(p.geometrie).length, 4);
  assert.equal(longueurPrevue(0), 1.5);
  assert.equal(longueurPrevue(5), 4);
});
