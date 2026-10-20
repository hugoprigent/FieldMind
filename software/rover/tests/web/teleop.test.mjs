// Homme mort de la téléopération (node --test software/rover/tests/web).
import { test } from 'node:test';
import assert from 'node:assert/strict';

import { Teleoperation, directionTouche } from '../../fieldmind/web/js/teleop.js';

function nouvelle() {
  const envois = [];
  const t = new Teleoperation((m) => envois.push(m));
  return { t, envois };
}

test('le joystick envoie tout de suite, puis à chaque pas tant qu’il est tenu', () => {
  const { t, envois } = nouvelle();
  t.joystick(0, 1);
  assert.equal(envois.length, 1);
  assert.deepEqual(envois[0], { type: 'pilotage', vitesse: 1, braquage: 0 });
  t.pas(1 / 15);
  t.pas(1 / 15);
  assert.equal(envois.length, 3);
});

test('au relâchement : un seul 0, 0, puis plus rien', () => {
  const { t, envois } = nouvelle();
  t.joystick(0.5, 0.5);
  t.relacherJoystick();
  assert.deepEqual(envois.at(-1), { type: 'pilotage', vitesse: 0, braquage: 0 });
  const n = envois.length;
  t.pas(1 / 15);
  t.pas(1 / 15);
  assert.equal(envois.length, n);
  assert.equal(t.actif, false);
});

test('joystick à gauche = braquage positif (vers la gauche) ; zone morte au centre', () => {
  const { t, envois } = nouvelle();
  t.joystick(-1, 0);
  assert.equal(envois[0].braquage, 1);
  assert.equal(envois[0].vitesse, 0);
  t.joystick(0.03, 0.03);
  assert.deepEqual(envois.at(-1), { type: 'pilotage', vitesse: 0, braquage: 0 });
});

test('la vitesse maximale choisie réduit la vitesse, pas le braquage', () => {
  const { t, envois } = nouvelle();
  t.definirFacteur(0.4);
  t.joystick(-1, 0);
  t.joystick(0, 1);
  assert.equal(envois.at(-1).vitesse, 0.4);
  assert.equal(envois[0].braquage, 1);
});

test('arrêt (perte du focus ou de la liaison) : 0, 0 si une commande était active, sinon rien', () => {
  const { t, envois } = nouvelle();
  t.arreter();
  assert.equal(envois.length, 0);
  t.appuyer('avant');
  t.appuyer('gauche');
  t.arreter();
  assert.deepEqual(envois.at(-1), { type: 'pilotage', vitesse: 0, braquage: 0 });
  assert.equal(t.touches.size, 0);
  t.pas(1 / 15);
  assert.equal(envois.at(-1).vitesse, 0);
});

test('hors mode manuel, rien n’est envoyé ; perdre l’autorisation arrête la commande', () => {
  const { t, envois } = nouvelle();
  t.definirAutorisation(false);
  t.joystick(0, 1);
  t.pas(1 / 15);
  assert.equal(envois.length, 0);
  t.relacherJoystick();
  t.definirAutorisation(true);
  t.joystick(0, 1);
  t.definirAutorisation(false);
  assert.deepEqual(envois.at(-1), { type: 'pilotage', vitesse: 0, braquage: 0 });
});

test('clavier : le braquage monte progressivement et revient à zéro au relâchement', () => {
  const { t, envois } = nouvelle();
  t.appuyer('avant');
  t.appuyer('gauche');
  const b0 = envois.at(-1).braquage;
  assert.ok(b0 > 0 && b0 < 1);
  for (let i = 0; i < 30; i++) t.pas(1 / 15);
  assert.equal(envois.at(-1).braquage, 1);
  t.relacher('gauche');
  for (let i = 0; i < 30; i++) t.pas(1 / 15);
  assert.equal(envois.at(-1).braquage, 0);
  assert.equal(envois.at(-1).vitesse, 1);
});

test('touches : ZQSD, WASD et flèches', () => {
  assert.equal(directionTouche('z'), 'avant');
  assert.equal(directionTouche('W'), 'avant');
  assert.equal(directionTouche('ArrowLeft'), 'gauche');
  assert.equal(directionTouche('q'), 'gauche');
  assert.equal(directionTouche('d'), 'droite');
  assert.equal(directionTouche('s'), 'arriere');
  assert.equal(directionTouche('x'), null);
});
