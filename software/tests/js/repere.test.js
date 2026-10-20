import { test } from 'node:test';
import assert from 'node:assert/strict';

import {
  solVers3d, troisDVersSol, orientationVersQuaternion3d,
} from '../../interface/web/js/repere.js';

const proche = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) < eps, `${a} ≠ ${b}`);

test('aller-retour sol → three.js → sol', () => {
  const [X, Y, Z] = solVers3d(3, -7, 0.5);
  assert.deepEqual(troisDVersSol(X, Y, Z), { x: 3, y: -7, z: 0.5 });
});

test('le nord du protocole est -Z dans three.js', () => {
  assert.deepEqual(solVers3d(0, 1), [0, 0, -1]);
});

test('un cap de 90° tourne l’avant (+X) vers le nord (-Z)', () => {
  const [qx, qy, qz, qw] = orientationVersQuaternion3d(0, 0, Math.PI / 2);
  // Rotation du vecteur (1, 0, 0) par le quaternion.
  const vx = 1, vy = 0, vz = 0;
  const tx = 2 * (qy * vz - qz * vy), ty = 2 * (qz * vx - qx * vz), tz = 2 * (qx * vy - qy * vx);
  const r = [vx + qw * tx + (qy * tz - qz * ty), vy + qw * ty + (qz * tx - qx * tz),
    vz + qw * tz + (qx * ty - qy * tx)];
  proche(r[0], 0); proche(r[1], 0); proche(r[2], -1);
});
