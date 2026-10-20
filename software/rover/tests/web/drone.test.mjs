import test from 'node:test';
import assert from 'node:assert/strict';
import { ManetteDroneSimulee } from '../../fieldmind/web/js/drone.js';

test('aucun axe envoyé sans autorisation, puis commandes dans le seul atelier', () => {
  const messages = [], m = new ManetteDroneSimulee((x) => messages.push(x));
  m.appuyer('monter'); m.pas();
  assert.equal(messages.length, 0);
  m.definirAutorisation(true); m.appuyer('monter'); m.appuyer('tourner_gauche');
  assert.equal(messages.at(-1).montee, 1);
  assert.equal(messages.at(-1).lacet, -1);
  assert.ok(messages.every((x) => x.type === 'drone_atelier' && x.action === 'axes'));
});

test('relâcher ramène au neutre et le battement reste actif au repos', () => {
  const messages = [], m = new ManetteDroneSimulee((x) => messages.push(x));
  m.definirAutorisation(true); m.appuyer('droite'); m.relacher('droite'); m.pas();
  assert.equal(messages.at(-1).droite, 0);
  assert.equal(messages.at(-1).montee, 0);
  assert.equal(messages.length, 3);
});

test('perte de focus ou autorisation efface les touches sans reprise du mouvement', () => {
  const messages = [], m = new ManetteDroneSimulee((x) => messages.push(x));
  m.definirAutorisation(true); m.appuyer('avant'); m.definirAutorisation(false);
  assert.equal(messages.at(-1).avant, 0);
  const n = messages.length; m.pas(); assert.equal(messages.length, n);
  m.definirAutorisation(true); m.pas(); assert.equal(messages.at(-1).avant, 0);
});
