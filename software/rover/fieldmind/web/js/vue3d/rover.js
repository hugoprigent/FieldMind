// Modèle simple du rover en primitives, dans base_link (x avant, y gauche, z haut, origine au
// sol au milieu de l'essieu arrière), à partir de `profil.geometrie`, `profil.lidar` et
// `profil.cameras` : planche, coque blanche à bande bleue, quatre roues qui tournent avec la
// vitesse, roues avant braquées (épure d'Ackermann), mât et barre des caméras, mât et lidar
// dont la tête tourne.

import * as THREE from '../../vendor/three/three.module.js';
import { PALETTE, textureTache } from './materiaux.js';

const DEG = Math.PI / 180;

function nombre(valeur, defaut) {
  return Number.isFinite(valeur) ? valeur : defaut;
}

/** Dimensions utiles, complétées par les valeurs de config/rover.toml. */
export function dimensions(profil = {}) {
  const g = profil.geometrie || {}, l = profil.lidar || {}, c = profil.cameras || {};
  const empattement = nombre(g.empattement_m, 0.38);
  return {
    empattement,
    voieArriere: nombre(g.voie_arriere_m, 0.39),
    voieAvant: nombre(g.voie_avant_m, 0.31),
    rayonArriere: nombre(g.diametre_roue_arriere_m, 0.2) / 2,
    rayonAvant: nombre(g.diametre_roue_avant_m, 0.15) / 2,
    braquageMax: nombre(g.braquage_max_deg, 31) * DEG,
    arriere: nombre(g.emprise_arriere_m, -0.11),
    avant: nombre(g.emprise_avant_m, 0.46),
    demiLargeur: nombre(g.emprise_demi_largeur_m, 0.225),
    hauteur: nombre(g.hauteur_max_m, 0.36),
    plancheLongueur: nombre(g.planche_longueur_m, 0.5),
    plancheLargeur: nombre(g.planche_largeur_m, 0.3),
    lidarX: nombre(l.x_m, 0), lidarY: nombre(l.y_m, 0), lidarZ: nombre(l.z_m, 0.322),
    cameraX: nombre(c.x_m, 0.415), cameraZ: nombre(c.z_m, 0.265), cameraBase: nombre(c.base_m, 0.12),
    cameraInclinaison: nombre(c.inclinaison_deg, 10) * DEG,
  };
}

/** Cylindre d'axe y (axe des roues), centré. */
function cylindreY(rayon, largeur, segments = 24) {
  const geometrie = new THREE.CylinderGeometry(rayon, rayon, largeur, segments);
  return geometrie; // CylinderGeometry est déjà d'axe y
}

export class RoverModele {
  constructor(profil) {
    this.groupe = new THREE.Group();
    this.groupe.name = 'rover';
    this.materiaux = {
      coque: new THREE.MeshPhongMaterial({ color: PALETTE.coque, shininess: 40, specular: 0x30343a }),
      bande: new THREE.MeshPhongMaterial({ color: PALETTE.accent, shininess: 30, specular: 0x203040, emissive: 0x0a2236 }),
      planche: new THREE.MeshLambertMaterial({ color: PALETTE.planche }),
      pneu: new THREE.MeshLambertMaterial({ color: PALETTE.pneu }),
      jante: new THREE.MeshLambertMaterial({ color: PALETTE.jante }),
      mat: new THREE.MeshLambertMaterial({ color: PALETTE.mat }),
      lidar: new THREE.MeshPhongMaterial({ color: PALETTE.lidar, shininess: 60 }),
      lidarTete: new THREE.MeshPhongMaterial({ color: PALETTE.lidarTete, shininess: 80 }),
      optique: new THREE.MeshPhongMaterial({ color: PALETTE.optique, shininess: 120, specular: 0x6b7480 }),
      repere: new THREE.MeshBasicMaterial({ color: PALETTE.accent }),
      ombre: new THREE.MeshBasicMaterial({ map: textureTache(), transparent: true, depthWrite: false }),
    };
    this.angleRoues = [0, 0, 0, 0];
    this.angleLidar = 0;
    this.construire(profil);
  }

  construire(profil) {
    this._viderGeometries();
    this.d = dimensions(profil);
    const d = this.d, m = this.materiaux;
    const geometries = (this.geometries = []);
    const maillage = (geometrie, materiau) => {
      geometries.push(geometrie);
      return new THREE.Mesh(geometrie, materiau);
    };

    // Ombre douce au sol (pas de carte d'ombres).
    const ombre = maillage(new THREE.PlaneGeometry(1, 1), m.ombre);
    ombre.scale.set(d.avant - d.arriere + 0.32, 2 * d.demiLargeur + 0.26, 1);
    ombre.position.set((d.avant + d.arriere) / 2, 0, 0.004);
    ombre.renderOrder = -5;
    this.groupe.add(ombre);

    // Planche.
    const xMilieu = d.empattement / 2;
    const zPlanche = d.rayonArriere + 0.005;
    const planche = maillage(new THREE.BoxGeometry(d.plancheLongueur, d.plancheLargeur, 0.018), m.planche);
    planche.position.set(xMilieu, 0, zPlanche);
    this.groupe.add(planche);

    // Coque : profil latéral extrudé selon y, arêtes arrondies (biseau).
    const x0 = xMilieu - d.plancheLongueur / 2 - 0.005, x1 = Math.min(d.avant - 0.03, xMilieu + d.plancheLongueur / 2);
    const zb = zPlanche + 0.009, zh = Math.max(zb + 0.07, Math.min(d.lidarZ - 0.1, zb + 0.105));
    const forme = new THREE.Shape();
    forme.moveTo(x0, zb);
    forme.lineTo(x1, zb);
    forme.lineTo(x1, zb + 0.03);
    forme.lineTo(x1 - 0.07, zh);
    forme.lineTo(x0 + 0.03, zh);
    forme.quadraticCurveTo(x0, zh, x0, zh - 0.03);
    forme.lineTo(x0, zb);
    const demiCoque = d.plancheLargeur / 2 - 0.02;
    const coqueGeo = new THREE.ExtrudeGeometry(forme, {
      depth: 2 * demiCoque, bevelEnabled: true, bevelThickness: 0.012, bevelSize: 0.01, bevelSegments: 3, curveSegments: 6,
    });
    coqueGeo.rotateX(Math.PI / 2);
    coqueGeo.translate(0, demiCoque, 0);
    coqueGeo.computeVertexNormals();
    this.groupe.add(maillage(coqueGeo, m.coque));
    this.zCoque = zh + 0.01;

    // Bande bleue autour de la coque (flancs et arrière).
    const zBande = zb + 0.035, hBande = 0.02;
    const yBande = demiCoque + 0.0125;
    for (const cote of [1, -1]) {
      const bande = maillage(new THREE.BoxGeometry(x1 - x0 - 0.02, 0.004, hBande), m.bande);
      bande.position.set((x0 + x1) / 2 - 0.01, cote * yBande, zBande);
      this.groupe.add(bande);
    }
    const bandeArriere = maillage(new THREE.BoxGeometry(0.004, 2 * yBande, hBande), m.bande);
    bandeArriere.position.set(x0 - 0.0125, 0, zBande);
    this.groupe.add(bandeArriere);

    // Roues : pneu sombre, jante claire et un rayon pour voir la rotation.
    this.roues = [];
    this.pivots = [];
    const roue = (rayon, largeur) => {
      const g = new THREE.Group();
      const pivot = new THREE.Group();
      pivot.add(maillage(cylindreY(rayon, largeur, 28), m.pneu));
      const jante = maillage(cylindreY(rayon * 0.62, largeur + 0.004, 20), m.jante);
      pivot.add(jante);
      for (const angle of [0, Math.PI / 2]) {
        const rayonVisible = maillage(new THREE.BoxGeometry(rayon * 1.18, largeur + 0.008, rayon * 0.16), m.pneu);
        rayonVisible.rotation.y = angle;
        pivot.add(rayonVisible);
      }
      g.add(pivot);
      return { g, pivot };
    };
    const largeurArriere = 0.055, largeurAvant = 0.045;
    const yArriere = Math.min(d.voieArriere / 2, d.demiLargeur - largeurArriere / 2);
    for (const cote of [1, -1]) {
      const r = roue(d.rayonArriere, largeurArriere);
      r.g.position.set(0, cote * yArriere, d.rayonArriere);
      this.groupe.add(r.g);
      this.roues.push({ ...r, rayon: d.rayonArriere, y: cote * yArriere, avant: false });
    }
    const decalageFusee = 0.03;
    for (const cote of [1, -1]) {
      const fusee = new THREE.Group();
      fusee.position.set(d.empattement, cote * d.voieAvant / 2, 0);
      const r = roue(d.rayonAvant, largeurAvant);
      r.g.position.set(0, cote * decalageFusee, d.rayonAvant);
      fusee.add(r.g);
      this.groupe.add(fusee);
      this.pivots.push({ fusee, cote });
      this.roues.push({ ...r, rayon: d.rayonAvant, y: cote * (d.voieAvant / 2 + decalageFusee), avant: true });
    }

    // Mât et barre des caméras (à l'avant), inclinée vers le sol.
    const zPied = zb + 0.02;
    const mat = maillage(new THREE.BoxGeometry(0.014, 0.014, Math.max(0.02, d.cameraZ - zPied)), m.mat);
    mat.position.set(d.cameraX - 0.025, 0, (zPied + d.cameraZ) / 2);
    this.groupe.add(mat);
    const barre = new THREE.Group();
    barre.position.set(d.cameraX, 0, d.cameraZ);
    barre.rotation.y = d.cameraInclinaison;
    barre.add(maillage(new THREE.BoxGeometry(0.018, d.cameraBase + 0.06, 0.022), m.mat));
    for (const cote of [1, -1]) {
      const boitier = maillage(new THREE.BoxGeometry(0.024, 0.04, 0.03), m.lidar);
      boitier.position.set(0.012, cote * d.cameraBase / 2, 0);
      barre.add(boitier);
      const objectif = maillage(new THREE.CylinderGeometry(0.009, 0.009, 0.006, 16), m.optique);
      objectif.rotation.z = Math.PI / 2;
      objectif.position.set(0.026, cote * d.cameraBase / 2, 0);
      barre.add(objectif);
    }
    this.groupe.add(barre);

    // Mât et lidar au-dessus de l'essieu arrière ; plan laser à lidarZ.
    const zSocle = d.lidarZ - 0.02;
    const matLidar = maillage(new THREE.CylinderGeometry(0.011, 0.011, Math.max(0.02, zSocle - this.zCoque + 0.01), 10), m.mat);
    matLidar.rotation.x = Math.PI / 2;
    matLidar.position.set(d.lidarX, d.lidarY, (zSocle + this.zCoque) / 2);
    this.groupe.add(matLidar);
    const socle = maillage(new THREE.BoxGeometry(0.056, 0.056, 0.016), m.lidar);
    socle.position.set(d.lidarX, d.lidarY, zSocle);
    this.groupe.add(socle);
    this.teteLidar = new THREE.Group();
    this.teteLidar.position.set(d.lidarX, d.lidarY, zSocle + 0.008);
    const tete = maillage(new THREE.CylinderGeometry(0.027, 0.028, 0.03, 28), m.lidarTete);
    tete.rotation.x = Math.PI / 2;
    tete.position.z = 0.015;
    this.teteLidar.add(tete);
    const repere = maillage(new THREE.BoxGeometry(0.012, 0.006, 0.006), m.repere);
    repere.position.set(0.024, 0, 0.022);
    this.teteLidar.add(repere);
    this.groupe.add(this.teteLidar);

    this.groupe.traverse((objet) => {
      if (objet.isMesh && objet.material !== m.ombre) objet.renderOrder = 1;
    });
  }

  /**
   * Animation : `vitesse` (m/s, essieu arrière), `braquage` (rad, roue virtuelle centrale),
   * `lidar` (tête en rotation ou non), `dt` (s).
   */
  animer(dt, vitesse, braquage, lidar = true) {
    const d = this.d;
    const k = Math.tan(braquage) / d.empattement; // courbure
    for (let i = 0; i < this.roues.length; i++) {
      const r = this.roues[i];
      const v = vitesse * (1 - k * r.y);
      this.angleRoues[i] = (this.angleRoues[i] + (v / r.rayon) * dt) % (2 * Math.PI);
      r.pivot.rotation.y = this.angleRoues[i];
    }
    for (const { fusee, cote } of this.pivots) {
      let angle = braquage;
      if (Math.abs(braquage) > 1e-4) {
        const R = d.empattement / Math.tan(braquage);
        angle = Math.atan(d.empattement / (R - cote * d.voieAvant / 2));
      }
      fusee.rotation.z = angle;
    }
    if (lidar) this.angleLidar = (this.angleLidar - dt * 2 * Math.PI * 1.4) % (2 * Math.PI);
    this.teteLidar.rotation.z = this.angleLidar;
  }

  _viderGeometries() {
    for (const geometrie of this.geometries || []) geometrie.dispose();
    this.geometries = [];
    for (const enfant of [...this.groupe.children]) this.groupe.remove(enfant);
  }

  detruire() {
    this._viderGeometries();
    for (const materiau of Object.values(this.materiaux)) {
      materiau.map?.dispose();
      materiau.dispose();
    }
  }
}
