// Tracés au sol : chemin planifié (ruban bleu animé), but (cible), trajectoire prévue (arc
// balayé par l'avant du rover selon braquage et vitesse, blanc translucide, rouge si la sécurité
// arrête), points du lidar (poussière facultative) et points d'obstacles de la stéréo.
// Tous les tampons sont alloués une fois ; les messages les réécrivent en place.

import * as THREE from '../../vendor/three/three.module.js';
import { PALETTE } from './materiaux.js';

const CHEMIN_MAX = 1024;
const LARGEUR_CHEMIN = 0.2;

function attribut(n, taille) {
  const a = new THREE.BufferAttribute(new Float32Array(n * taille), taille);
  a.setUsage(THREE.DynamicDrawUsage);
  return a;
}

/** Index d'une bande de `n` tronçons (2 sommets par station). */
function indexBande(n) {
  const idx = new Uint32Array(n * 6);
  for (let i = 0; i < n; i++) {
    const a = 2 * i;
    idx.set([a, a + 1, a + 2, a + 1, a + 3, a + 2], 6 * i);
  }
  return new THREE.BufferAttribute(idx, 1);
}

// ------------------------------------------------------------------ chemin

export class Chemin {
  constructor(uniformes) {
    const g = (this.geometrie = new THREE.BufferGeometry());
    this.positions = attribut(2 * CHEMIN_MAX, 3);
    this.coord = attribut(2 * CHEMIN_MAX, 2); // (abscisse m, travers -1..1)
    g.setAttribute('position', this.positions);
    g.setAttribute('aCoord', this.coord);
    g.setIndex(indexBande(CHEMIN_MAX - 1));
    g.setDrawRange(0, 0);
    this.materiau = new THREE.ShaderMaterial({
      uniforms: { ...uniformes, uCouleur: { value: new THREE.Color(PALETTE.accent) }, uOpacite: { value: 0 } },
      transparent: true,
      depthWrite: false,
      side: THREE.DoubleSide,
      vertexShader: /* glsl */ `
        attribute vec2 aCoord;
        varying vec2 vCoord;
        varying vec2 vMonde;
        void main() {
          vCoord = aCoord;
          vec4 monde = modelMatrix * vec4(position, 1.0);
          vMonde = monde.xy;
          gl_Position = projectionMatrix * viewMatrix * monde;
        }`,
      fragmentShader: /* glsl */ `
        uniform vec3 uCouleur;
        uniform float uOpacite;
        uniform float uTemps;
        uniform vec2 uCentre;
        uniform vec2 uFondu;
        varying vec2 vCoord;
        varying vec2 vMonde;
        void main() {
          float t = abs(vCoord.y);
          float w = fwidth(t);
          float corps = 1.0 - smoothstep(1.0 - 1.5 * w, 1.0, t);
          float bord = smoothstep(0.55, 0.9, t) * corps;
          float fleche = smoothstep(0.35, 0.5, fract(vCoord.x / 0.6 - uTemps * 0.9)) * (1.0 - smoothstep(0.5, 0.65, fract(vCoord.x / 0.6 - uTemps * 0.9)));
          float a = corps * (0.34 + 0.5 * bord + 0.16 * fleche * (1.0 - t));
          a *= uOpacite * smoothstep(0.0, 0.5, vCoord.x) * (1.0 - smoothstep(uFondu.x, uFondu.y, distance(vMonde, uCentre)));
          gl_FragColor = vec4(uCouleur * (1.0 + 0.25 * bord), a);
          #include <colorspace_fragment>
        }`,
    });
    this.maillage = new THREE.Mesh(g, this.materiau);
    this.maillage.position.z = 0.006;
    this.maillage.renderOrder = -7;
    this.maillage.frustumCulled = false;
    this.cible = 0;
  }

  definir(points) {
    const pts = Array.isArray(points) ? points.filter((p) => p && Number.isFinite(p[0]) && Number.isFinite(p[1])) : [];
    const n = Math.min(pts.length, CHEMIN_MAX);
    if (n < 2) {
      this.cible = 0;
      return;
    }
    this.cible = 1;
    const pos = this.positions.array, co = this.coord.array;
    let s = 0;
    for (let i = 0; i < n; i++) {
      const p = pts[i], av = pts[Math.max(0, i - 1)], ap = pts[Math.min(n - 1, i + 1)];
      if (i > 0) s += Math.hypot(p[0] - av[0], p[1] - av[1]);
      let tx = ap[0] - av[0], ty = ap[1] - av[1];
      const l = Math.hypot(tx, ty) || 1;
      tx /= l;
      ty /= l;
      const nx = -ty * LARGEUR_CHEMIN / 2, ny = tx * LARGEUR_CHEMIN / 2;
      pos.set([p[0] + nx, p[1] + ny, 0, p[0] - nx, p[1] - ny, 0], 6 * i);
      co.set([s, 1, s, -1], 4 * i);
    }
    this.positions.needsUpdate = true;
    this.coord.needsUpdate = true;
    this.geometrie.setDrawRange(0, (n - 1) * 6);
  }

  animer(dt) {
    const u = this.materiau.uniforms.uOpacite;
    u.value += Math.sign(this.cible - u.value) * Math.min(Math.abs(this.cible - u.value), dt / 0.4);
    this.maillage.visible = u.value > 0.003;
  }

  detruire() {
    this.geometrie.dispose();
    this.materiau.dispose();
  }
}

// ------------------------------------------------------------------ but

export class But {
  constructor(uniformes) {
    this.groupe = new THREE.Group();
    this.materiau = new THREE.ShaderMaterial({
      uniforms: { ...uniformes, uCouleur: { value: new THREE.Color(PALETTE.accent) }, uOpacite: { value: 0 } },
      transparent: true,
      depthWrite: false,
      vertexShader: /* glsl */ `
        varying vec2 vUv;
        void main() {
          vUv = uv * 2.0 - 1.0;
          gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }`,
      fragmentShader: /* glsl */ `
        uniform vec3 uCouleur;
        uniform float uOpacite;
        uniform float uTemps;
        varying vec2 vUv;
        float anneau(float r, float centre, float demi) {
          float w = fwidth(r);
          return 1.0 - smoothstep(demi - w, demi + w, abs(r - centre));
        }
        void main() {
          float r = length(vUv);
          float a = anneau(r, 0.52, 0.06) * 0.95 + anneau(r, 0.0, 0.14) * 0.95;
          a += (1.0 - smoothstep(0.0, 0.52, r)) * 0.16;
          float p = fract(uTemps * 0.55);
          a += anneau(r, 0.52 + 0.45 * p, 0.035) * (1.0 - p) * 0.8;
          gl_FragColor = vec4(uCouleur, min(a, 1.0) * uOpacite);
          #include <colorspace_fragment>
        }`,
    });
    this.disque = new THREE.Mesh(new THREE.PlaneGeometry(1.4, 1.4), this.materiau);
    this.disque.position.z = 0.008;
    this.disque.renderOrder = -6;
    this.matTige = new THREE.MeshBasicMaterial({ color: PALETTE.accent, transparent: true, opacity: 0, depthWrite: false });
    this.tige = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.9, 8), this.matTige);
    this.tige.rotation.x = Math.PI / 2;
    this.tige.position.z = 0.45;
    this.tete = new THREE.Mesh(new THREE.SphereGeometry(0.045, 16, 10), this.matTige);
    this.tete.position.z = 0.92;
    this.groupe.add(this.disque, this.tige, this.tete);
    this.groupe.visible = false;
    this.position = new THREE.Vector2();
    this.cible = 0;
    this.valeur = 0;
  }

  definir(but) {
    if (Array.isArray(but) && Number.isFinite(but[0]) && Number.isFinite(but[1])) {
      const nouveau = this.valeur < 0.01 || Math.hypot(but[0] - this.position.x, but[1] - this.position.y) > 2;
      this.position.set(but[0], but[1]);
      if (nouveau) this.groupe.position.set(but[0], but[1], 0);
      this.cible = 1;
    } else {
      this.cible = 0;
    }
  }

  animer(dt) {
    this.valeur += Math.sign(this.cible - this.valeur) * Math.min(Math.abs(this.cible - this.valeur), dt / 0.4);
    const k = 1 - Math.exp(-dt / 0.2);
    this.groupe.position.x += (this.position.x - this.groupe.position.x) * k;
    this.groupe.position.y += (this.position.y - this.groupe.position.y) * k;
    this.materiau.uniforms.uOpacite.value = this.valeur;
    this.matTige.opacity = 0.85 * this.valeur;
    this.groupe.visible = this.valeur > 0.003;
  }

  detruire() {
    this.disque.geometry.dispose();
    this.tige.geometry.dispose();
    this.tete.geometry.dispose();
    this.materiau.dispose();
    this.matTige.dispose();
  }
}

// ------------------------------------------------------------------ trajectoire prévue

const STATIONS_ARC = 48;

export class Trajectoire {
  constructor() {
    const g = (this.geometrie = new THREE.BufferGeometry());
    this.positions = attribut(2 * (STATIONS_ARC + 1), 3);
    this.coord = attribut(2 * (STATIONS_ARC + 1), 2); // (avancement 0..1, travers -1..1)
    for (let i = 0; i <= STATIONS_ARC; i++) this.coord.array.set([i / STATIONS_ARC, 1, i / STATIONS_ARC, -1], 4 * i);
    g.setAttribute('position', this.positions);
    g.setAttribute('aCoord', this.coord);
    g.setIndex(indexBande(STATIONS_ARC));
    this.materiau = new THREE.ShaderMaterial({
      uniforms: {
        uCouleur: { value: new THREE.Color(PALETTE.blanc) },
        uOpacite: { value: 1 },
      },
      transparent: true,
      depthWrite: false,
      side: THREE.DoubleSide,
      vertexShader: /* glsl */ `
        attribute vec2 aCoord;
        varying vec2 vCoord;
        void main() {
          vCoord = aCoord;
          gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }`,
      fragmentShader: /* glsl */ `
        uniform vec3 uCouleur;
        uniform float uOpacite;
        varying vec2 vCoord;
        void main() {
          float t = abs(vCoord.y);
          float w = fwidth(t);
          float bord = smoothstep(1.0 - 4.0 * w, 1.0 - 1.5 * w, t) * (1.0 - smoothstep(1.0 - 1.5 * w, 1.0, t));
          float corps = 1.0 - smoothstep(1.0 - 1.5 * w, 1.0, t);
          float fin = 1.0 - smoothstep(0.35, 1.0, vCoord.x);
          float debut = smoothstep(0.0, 0.06, vCoord.x);
          float a = (corps * 0.13 + bord * 0.55) * fin * debut * uOpacite;
          gl_FragColor = vec4(uCouleur, a);
          #include <colorspace_fragment>
        }`,
    });
    this.maillage = new THREE.Mesh(g, this.materiau);
    this.maillage.position.z = 0.01;
    this.maillage.renderOrder = -5;
    this.maillage.frustumCulled = false;
    this.blanc = new THREE.Color(PALETTE.blanc);
    this.rouge = new THREE.Color(PALETTE.danger);
    this.melange = 0;
  }

  /**
   * Recalcule l'arc dans base_link : avant du rover (ou arrière en marche arrière), demi-largeur
   * de l'emprise, longueur ≈ 3 s de route (1,5 à 4 m), courbure du modèle bicyclette.
   */
  mettreAJour(dt, { vitesse, braquage, arret, actif }, d) {
    const k = Math.tan(braquage) / d.empattement;
    const sens = vitesse < -0.02 ? -1 : 1;
    const longueur = Math.min(4, Math.max(1.5, 1 + 3 * Math.abs(vitesse)));
    const xBord = sens > 0 ? d.avant : d.arriere;
    const w = d.demiLargeur;
    const pos = this.positions.array;
    for (let i = 0; i <= STATIONS_ARC; i++) {
      const s = (sens * longueur * i) / STATIONS_ARC;
      let x, y, cap;
      if (Math.abs(k) < 1e-6) {
        x = s; y = 0; cap = 0;
      } else {
        cap = k * s;
        x = Math.sin(cap) / k;
        y = (1 - Math.cos(cap)) / k;
      }
      const c = Math.cos(cap), sn = Math.sin(cap);
      const o = 6 * i;
      pos[o] = x + c * xBord - sn * w;
      pos[o + 1] = y + sn * xBord + c * w;
      pos[o + 2] = 0;
      pos[o + 3] = x + c * xBord + sn * w;
      pos[o + 4] = y + sn * xBord - c * w;
      pos[o + 5] = 0;
    }
    this.positions.needsUpdate = true;
    this.melange += Math.sign((arret ? 1 : 0) - this.melange) * Math.min(Math.abs((arret ? 1 : 0) - this.melange), dt / 0.2);
    this.materiau.uniforms.uCouleur.value.copy(this.blanc).lerp(this.rouge, this.melange);
    const opacite = actif ? 1 : 0.45;
    const u = this.materiau.uniforms.uOpacite;
    u.value += (opacite - u.value) * Math.min(1, dt / 0.3);
  }

  detruire() {
    this.geometrie.dispose();
    this.materiau.dispose();
  }
}

// ------------------------------------------------------------------ points

const POINTS_MAX = 4096;

/** Nuage de points dans base_link (centimètres aplatis), à hauteur fixe. */
export class Points {
  constructor({ couleur, taille, opacite, z, ordre }) {
    this.geometrie = new THREE.BufferGeometry();
    this.positions = attribut(POINTS_MAX, 3);
    this.geometrie.setAttribute('position', this.positions);
    this.geometrie.setDrawRange(0, 0);
    this.materiau = new THREE.PointsMaterial({
      color: couleur, size: taille, sizeAttenuation: false, transparent: true, opacity: opacite, depthWrite: false,
    });
    this.opacite = opacite;
    this.objet = new THREE.Points(this.geometrie, this.materiau);
    this.objet.frustumCulled = false;
    this.objet.renderOrder = ordre;
    this.z = z;
    this.recu = -Infinity;
  }

  definir(xy, maintenant) {
    const n = Array.isArray(xy) ? Math.min(POINTS_MAX, Math.floor(xy.length / 2)) : 0;
    const pos = this.positions.array;
    let k = 0;
    for (let i = 0; i < n; i++) {
      const x = xy[2 * i], y = xy[2 * i + 1];
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      pos[3 * k] = x / 100;
      pos[3 * k + 1] = y / 100;
      pos[3 * k + 2] = this.z;
      k++;
    }
    this.positions.needsUpdate = true;
    this.geometrie.setDrawRange(0, k);
    this.recu = maintenant;
  }

  /** Les points s'effacent s'ils ne sont plus rafraîchis. */
  animer(maintenant) {
    const age = maintenant - this.recu;
    this.materiau.opacity = this.opacite * (1 - Math.min(1, Math.max(0, (age - 0.6) / 0.6)));
  }

  detruire() {
    this.geometrie.dispose();
    this.materiau.dispose();
  }
}
