// Sol et espace libre. Le sol est un grand carré qui suit le rover : la grille (1 m, et 5 m
// plus marquée) est calculée en coordonnées du monde, elle reste donc fixe sous le rover qui
// avance ; le sol s'efface en périphérie dans le fond (canvas transparent sur un dégradé CSS).
// L'espace libre (`scene.libre`) est triangulé à réception, bordé d'une frange extérieure qui
// le rend doux sur les bords, et ajouté au sol (mélange additif) : deux tampons alternent pour
// un fondu croisé entre deux messages, sans creux de luminosité.

import * as THREE from '../../vendor/three/three.module.js';
import { PALETTE } from './materiaux.js';

export const RAYON_SOL = 42;

export class Sol {
  constructor(uniformes) {
    this.uniformes = uniformes;
    const geometrie = new THREE.PlaneGeometry(2 * RAYON_SOL, 2 * RAYON_SOL);
    this.materiau = new THREE.ShaderMaterial({
      uniforms: {
        ...uniformes,
        uSol: { value: new THREE.Color(PALETTE.sol) },
        uGrille: { value: new THREE.Color(PALETTE.grille) },
        uGrilleForte: { value: new THREE.Color(PALETTE.grilleForte) },
        uRayon: { value: RAYON_SOL },
      },
      transparent: true,
      depthWrite: false,
      vertexShader: /* glsl */ `
        varying vec2 vMonde;
        void main() {
          vec4 monde = modelMatrix * vec4(position, 1.0);
          vMonde = monde.xy;
          gl_Position = projectionMatrix * viewMatrix * monde;
        }`,
      fragmentShader: /* glsl */ `
        uniform vec2 uCentre;
        uniform vec3 uSol;
        uniform vec3 uGrille;
        uniform vec3 uGrilleForte;
        uniform float uRayon;
        varying vec2 vMonde;
        float lignes(vec2 p, float pas, float epaisseur) {
          vec2 q = p / pas;
          vec2 w = fwidth(q);
          vec2 g = abs(fract(q - 0.5) - 0.5) / max(w, 1e-5);
          float l = 1.0 - min(min(g.x, g.y) / epaisseur, 1.0);
          // la grille s'efface quand ses mailles deviennent plus petites que quelques pixels
          return l * (1.0 - smoothstep(0.12, 0.45, max(w.x, w.y)));
        }
        void main() {
          float r = distance(vMonde, uCentre);
          float bord = 1.0 - smoothstep(uRayon * 0.30, uRayon * 0.92, r);
          vec3 c = uSol * (1.0 + 0.35 * (1.0 - smoothstep(0.0, 9.0, r)));
          float fine = lignes(vMonde, 1.0, 1.0) * (1.0 - smoothstep(8.0, 20.0, r));
          float forte = lignes(vMonde, 5.0, 1.2) * (1.0 - smoothstep(16.0, 34.0, r));
          c = mix(c, uGrille, fine * 0.9);
          c = mix(c, uGrilleForte, forte * 0.9);
          gl_FragColor = vec4(c, bord);
          #include <colorspace_fragment>
        }`,
    });
    this.maillage = new THREE.Mesh(geometrie, this.materiau);
    this.maillage.renderOrder = -10;
    this.maillage.frustumCulled = false;
    this.libre = new EspaceLibre(uniformes);
  }

  ajouterA(parent) {
    parent.add(this.maillage);
    for (const m of this.libre.maillages) parent.add(m);
  }

  /** Le sol suit le rover par pas de 1 m (la grille reste calée sur le monde). */
  suivre(x, y) {
    this.maillage.position.set(Math.round(x), Math.round(y), 0);
  }

  animer(dt) {
    this.libre.animer(dt);
  }

  detruire() {
    this.maillage.geometry.dispose();
    this.materiau.dispose();
    this.libre.detruire();
  }
}

const POINTS_MAX = 2048;
const FRANGE_M = 0.32;

class EspaceLibre {
  constructor(uniformes) {
    this.materiaux = [];
    this.maillages = [];
    this.tampons = [];
    for (let i = 0; i < 2; i++) {
      const geometrie = new THREE.BufferGeometry();
      const positions = new THREE.BufferAttribute(new Float32Array(POINTS_MAX * 2 * 3), 3);
      const alphas = new THREE.BufferAttribute(new Float32Array(POINTS_MAX * 2), 1);
      positions.setUsage(THREE.DynamicDrawUsage);
      alphas.setUsage(THREE.DynamicDrawUsage);
      geometrie.setAttribute('position', positions);
      geometrie.setAttribute('aAlpha', alphas);
      const index = new THREE.BufferAttribute(new Uint32Array(POINTS_MAX * 9), 1);
      index.setUsage(THREE.DynamicDrawUsage);
      geometrie.setIndex(index);
      geometrie.setDrawRange(0, 0);
      const materiau = new THREE.ShaderMaterial({
        uniforms: { ...uniformes, uCouleur: { value: new THREE.Color(PALETTE.libre) }, uOpacite: { value: 0 } },
        transparent: true,
        depthWrite: false,
        blending: THREE.AdditiveBlending,
        side: THREE.DoubleSide,
        vertexShader: /* glsl */ `
          attribute float aAlpha;
          varying float vAlpha;
          varying vec2 vMonde;
          void main() {
            vAlpha = aAlpha;
            vec4 monde = modelMatrix * vec4(position, 1.0);
            vMonde = monde.xy;
            gl_Position = projectionMatrix * viewMatrix * monde;
          }`,
        fragmentShader: /* glsl */ `
          uniform vec3 uCouleur;
          uniform float uOpacite;
          uniform vec2 uCentre;
          varying float vAlpha;
          varying vec2 vMonde;
          void main() {
            float r = distance(vMonde, uCentre);
            float a = vAlpha * vAlpha * uOpacite * (1.0 - 0.55 * smoothstep(1.0, 9.0, r));
            gl_FragColor = vec4(uCouleur, a);
            #include <colorspace_fragment>
          }`,
      });
      const maillage = new THREE.Mesh(geometrie, materiau);
      maillage.position.z = 0.002;
      maillage.renderOrder = -9;
      maillage.frustumCulled = false;
      maillage.visible = false;
      this.materiaux.push(materiau);
      this.maillages.push(maillage);
      this.tampons.push({ geometrie, positions, alphas, index, cible: 0 });
    }
    this.courant = 1;
  }

  /** `points` : [[x, y], …] (repère de la scène) ; null ou vide efface l'espace libre. */
  definir(points) {
    const n0 = Array.isArray(points) ? Math.min(points.length, POINTS_MAX) : 0;
    this.courant = 1 - this.courant;
    const t = this.tampons[this.courant];
    t.cible = 1;
    this.tampons[1 - this.courant].cible = 0;
    // Contour nettoyé : points valides, sans doublons consécutifs.
    const contour = [];
    for (let i = 0; i < n0; i++) {
      const p = points[i];
      if (!p || !Number.isFinite(p[0]) || !Number.isFinite(p[1])) continue;
      const dernier = contour[contour.length - 1];
      if (dernier && Math.abs(dernier.x - p[0]) < 1e-3 && Math.abs(dernier.y - p[1]) < 1e-3) continue;
      contour.push(new THREE.Vector2(p[0], p[1]));
    }
    if (contour.length > 2 && contour[0].distanceTo(contour[contour.length - 1]) < 1e-3) contour.pop();
    const n = contour.length;
    if (n < 3) {
      t.geometrie.setDrawRange(0, 0);
      return;
    }
    let aire = 0;
    for (let i = 0; i < n; i++) {
      const a = contour[i], b = contour[(i + 1) % n];
      aire += a.x * b.y - b.x * a.y;
    }
    const sens = aire >= 0 ? 1 : -1; // normale extérieure : à droite du contour s'il tourne à gauche
    const pos = t.positions.array, alpha = t.alphas.array, idx = t.index.array;
    for (let i = 0; i < n; i++) {
      const p = contour[i], av = contour[(i + n - 1) % n], ap = contour[(i + 1) % n];
      let nx = 0, ny = 0;
      for (const [a, b] of [[av, p], [p, ap]]) {
        const dx = b.x - a.x, dy = b.y - a.y, l = Math.hypot(dx, dy) || 1;
        nx += (sens * dy) / l;
        ny += (-sens * dx) / l;
      }
      const l = Math.hypot(nx, ny) || 1;
      pos[3 * i] = p.x;
      pos[3 * i + 1] = p.y;
      pos[3 * i + 2] = 0;
      alpha[i] = 1;
      pos[3 * (n + i)] = p.x + (nx / l) * FRANGE_M;
      pos[3 * (n + i) + 1] = p.y + (ny / l) * FRANGE_M;
      pos[3 * (n + i) + 2] = 0;
      alpha[n + i] = 0;
    }
    let k = 0;
    let triangles;
    try {
      triangles = THREE.ShapeUtils.triangulateShape(contour, []);
    } catch {
      triangles = [];
    }
    for (const [a, b, c] of triangles) {
      idx[k++] = a;
      idx[k++] = b;
      idx[k++] = c;
    }
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n;
      idx[k++] = i; idx[k++] = n + i; idx[k++] = n + j;
      idx[k++] = i; idx[k++] = n + j; idx[k++] = j;
    }
    t.positions.needsUpdate = true;
    t.alphas.needsUpdate = true;
    t.index.needsUpdate = true;
    t.positions.addUpdateRange(0, 6 * n);
    t.alphas.addUpdateRange(0, 2 * n);
    t.index.addUpdateRange(0, k);
    t.geometrie.setDrawRange(0, k);
  }

  animer(dt) {
    const pas = Math.min(1, dt / 0.35);
    for (let i = 0; i < 2; i++) {
      const u = this.materiaux[i].uniforms.uOpacite;
      const cible = this.tampons[i].cible;
      u.value += Math.sign(cible - u.value) * Math.min(Math.abs(cible - u.value), pas);
      this.maillages[i].visible = u.value > 0.003;
    }
  }

  detruire() {
    for (const t of this.tampons) t.geometrie.dispose();
    for (const m of this.materiaux) m.dispose();
  }
}
