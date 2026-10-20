// Remorque porte-drone attelée, en primitives grises comme le reste de la scène, dans le repère
// du rover (base_link). Origine du groupe à la vis d'attelage, tourné de −φ (angle d'attelage
// estimé, `etat.remorque`) : planche avec la croix d'atterrissage, timon en A, deux roues.

import * as THREE from '../../vendor/three/three.module.js';
import { PALETTE } from './materiaux.js';

const Z_PLANCHE = 0.158; // dessus de la planche (CAO : dessous à 155,5 mm)
const RAYON_ROUE = 0.075;

export class RemorqueModele {
  constructor() {
    this.groupe = new THREE.Group();
    this.groupe.name = 'remorque';
    this.groupe.visible = false;
    this.materiaux = {
      planche: new THREE.MeshLambertMaterial({ color: PALETTE.planche ?? 0xb9ab92 }),
      timon: new THREE.MeshPhongMaterial({ color: PALETTE.accent, shininess: 30 }),
      croix: new THREE.MeshBasicMaterial({ color: 0x1b1f25 }),
      pneu: new THREE.MeshLambertMaterial({ color: PALETTE.pneu ?? 0x2a2e34 }),
    };
    this.geometries = [];
    this.cle = '';
    this.angle = 0;
    this.cible = 0;
    this.roues = [];
  }

  /** `remorque` : objet `etat.remorque` (null si dételée). */
  definir(remorque) {
    this.groupe.visible = Boolean(remorque);
    if (!remorque) return;
    const g = remorque.geometrie || {};
    const cle = JSON.stringify(g);
    if (cle !== this.cle) {
      this.cle = cle;
      this._construire(g);
    }
    this.cible = Number.isFinite(remorque.angle_deg) ? remorque.angle_deg * Math.PI / 180 : 0;
  }

  animer(dt, vitesse) {
    if (!this.groupe.visible) return;
    this.angle += (this.cible - this.angle) * Math.min(1, dt * 8);
    this.groupe.rotation.z = -this.angle;
    for (const roue of this.roues) roue.rotation.y = (roue.rotation.y + (vitesse / RAYON_ROUE) * dt) % (2 * Math.PI);
  }

  _construire(g) {
    for (const geometrie of this.geometries) geometrie.dispose();
    this.geometries = [];
    this.roues = [];
    for (const enfant of [...this.groupe.children]) this.groupe.remove(enfant);
    const m = this.materiaux;
    const maillage = (geometrie, materiau) => {
      this.geometries.push(geometrie);
      return new THREE.Mesh(geometrie, materiau);
    };
    const attelage = g.attelage_m ?? 0.16, timon = g.timon_m ?? 0.31;
    const avant = g.avant_m ?? 0.1, arriere = g.arriere_m ?? 0.48;
    const demi = g.demi_largeur_m ?? 0.19;
    const largeurPlanche = 0.3;
    this.groupe.position.set(-attelage, 0, 0);

    const longueur = arriere - avant;
    const planche = maillage(new THREE.BoxGeometry(longueur, largeurPlanche, 0.006), m.planche);
    planche.position.set(-(avant + arriere) / 2, 0, Z_PLANCHE - 0.003);
    this.groupe.add(planche);
    // Croix d'atterrissage : 200 × 200 mm, bras de 36 mm, au centre de la planche.
    for (const [lx, ly] of [[0.2, 0.036], [0.036, 0.2]]) {
      const bras = maillage(new THREE.BoxGeometry(lx, ly, 0.002), m.croix);
      bras.position.set(-(avant + arriere) / 2, 0, Z_PLANCHE + 0.001);
      this.groupe.add(bras);
    }
    // Timon en A : deux bras des coins avant de la planche jusqu'à la vis d'attelage.
    for (const cote of [-1, 1]) {
      const dx = avant + 0.02, dy = largeurPlanche / 2 - 0.03;
      const long = Math.hypot(dx, dy);
      const bras = maillage(new THREE.BoxGeometry(long, 0.018, 0.012), m.timon);
      bras.position.set(-dx / 2, cote * dy / 2, 0.136);
      bras.rotation.z = -cote * Math.atan2(dy, dx);
      this.groupe.add(bras);
    }
    const tete = maillage(new THREE.CylinderGeometry(0.018, 0.018, 0.012, 20), m.timon);
    tete.rotation.x = Math.PI / 2;
    tete.position.set(0, 0, 0.136);
    this.groupe.add(tete);
    // Roues libres, à l'extérieur de la planche.
    for (const cote of [-1, 1]) {
      const roue = maillage(new THREE.CylinderGeometry(RAYON_ROUE, RAYON_ROUE, 0.03, 24), m.pneu);
      const pivot = new THREE.Group();
      pivot.position.set(-timon, cote * (demi - 0.017), RAYON_ROUE);
      pivot.add(roue);
      this.groupe.add(pivot);
      this.roues.push(pivot);
    }
    this.groupe.traverse((objet) => { if (objet.isMesh) objet.renderOrder = 1; });
  }

  detruire() {
    for (const geometrie of this.geometries) geometrie.dispose();
    for (const materiau of Object.values(this.materiaux)) materiau.dispose();
  }
}
