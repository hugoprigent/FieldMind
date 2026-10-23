import { test } from 'node:test';
import assert from 'node:assert/strict';
import { detecterLangue, tr } from '../../fieldmind/web/js/langue.js';

test('lien explicite et choix mémorisé passent avant le navigateur et le pays supposé', () => {
  assert.equal(detecterLangue({recherche:'?lang=en', choix:'fr', langues:['fr-FR'], fuseau:'Europe/Paris'}),'en');
  assert.equal(detecterLangue({choix:'fr', langues:['en-US'], fuseau:'America/New_York'}),'fr');
  assert.equal(detecterLangue({langues:['en-US','fr-FR'], fuseau:'Europe/Paris'}),'en');
  assert.equal(detecterLangue({langues:['fr-CA'], fuseau:'America/New_York'}),'fr');
  assert.equal(detecterLangue({langues:['de-DE'], fuseau:'Europe/Paris'}),'fr');
  assert.equal(detecterLangue({langues:['de-DE'], fuseau:'Europe/Berlin'}),'en');
});

test('les confirmations de sécurité et la télémétrie sont traduites sans altérer les valeurs', () => {
  assert.equal(tr('Arrêt d’urgence','en'),'Emergency stop');
  assert.equal(tr('Désarmer','en'),'Disarm');
  assert.equal(tr('Roues levées ?','en'),'Wheels lifted?');
  assert.equal(tr('SIMULATION · IA GX10','en'),'SIMULATION · AI GX10');
  assert.equal(tr('Muet depuis 2.3 s','en'),'Silent for 2.3 s');
  assert.equal(tr('Croix : centrée · X 0.25 / Y −0.12','en'),'Cross : centered · X 0.25 / Y −0.12');
  assert.equal(tr('fieldmind.ia_profil','en'),'fieldmind.ia_profil');
  assert.equal(tr('https://example.test/ia_profil','en'),'https://example.test/ia_profil');
  assert.equal(tr('ROVER_4 115200 0.25 m/s','en'),'ROVER_4 115200 0.25 m/s');
  assert.equal(tr('Armer','fr'),'Armer');
});
