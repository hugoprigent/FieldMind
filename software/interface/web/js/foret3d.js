// Décor 3D de la forêt, style « low-poly » : facettes planes et couleurs unies, lisible et léger.
// Construit à partir de la géométrie envoyée par le simulateur. Aucune donnée ici n'est connue du
// cerveau : c'est l'affichage de la vérité simulée. Les troncs et les rochers ont exactement la
// taille et la position des obstacles physiques ; le reste (herbe, relief lointain, forêt de fond,
// montagnes) est purement décoratif et situé là où le rover ne va pas.

import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { solVers3d } from './repere.js';

const PALETTE = {
  herbe: [0x7c9a47, 0x6f8f3f, 0x88a650, 0x74944a],
  sousBois: [0x5a6636, 0x4f5b31, 0x66703d, 0x5d5a35],
  terre: [0xa47b50, 0xb0875a, 0x9a7249],
  bordPiste: [0x8e7b4c, 0x857a48],
  clairiere: [0x9ab55a, 0xa7bd62, 0x8fb052],
  collines: [0x6e8a46, 0x62803f, 0x7a9450],
  tronc: [0x6b4a32, 0x5c3f2b, 0x74513a],
  aiguilles: [0x2f6b4a, 0x3a7a50, 0x2a5e44, 0x44825a, 0x33704c],
  rocher: [0x9a978d, 0x8b887f, 0xa5a197],
  boisMort: [0x6e5238, 0x654a33, 0x5a432f],
  coeurBois: 0xc9a878,
  mousse: 0x7d8f55,
  fleurs: [0xf2d45c, 0xf4f1ea, 0xd96c8a, 0xb7a3e0],
  montagnes: [0x9fb3c2, 0xa9bccb, 0x93a9ba],
};

function aleatoire(graine) {
  // mulberry32 : générateur reproductible pour les variations purement visuelles.
  let a = graine >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Bruit de valeur 2D lissé (relief et variations de teinte).
function bruit2d(graine) {
  const hache = (i, j) => {
    let h = (i * 374761393 + j * 668265263 + graine * 144269) | 0;
    h = Math.imul(h ^ (h >>> 13), 1274126177);
    return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
  };
  const lisse = (t) => t * t * (3 - 2 * t);
  return (x, y) => {
    const i = Math.floor(x), j = Math.floor(y);
    const fx = lisse(x - i), fy = lisse(y - j);
    const a = hache(i, j), b = hache(i + 1, j), c = hache(i, j + 1), d = hache(i + 1, j + 1);
    return a + (b - a) * fx + (c - a) * fy + (a - b - c + d) * fx * fy;
  };
}

const choisir = (liste, u) => liste[Math.min(liste.length - 1, Math.floor(u * liste.length))];

function distanceSegment(px, py, ax, ay, bx, by) {
  const dx = bx - ax, dy = by - ay;
  const l2 = dx * dx + dy * dy || 1e-9;
  const t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / l2));
  return Math.hypot(px - ax - t * dx, py - ay - t * dy);
}

/** Index spatial simple (grille) pour les distances aux arbres. */
function indexArbres(arbres, cellule = 4) {
  const cases = new Map();
  for (const a of arbres) {
    const cle = `${Math.floor(a[0] / cellule)},${Math.floor(a[1] / cellule)}`;
    if (!cases.has(cle)) cases.set(cle, []);
    cases.get(cle).push(a);
  }
  return (x, y) => {
    const ci = Math.floor(x / cellule), cj = Math.floor(y / cellule);
    let d = Infinity;
    for (let i = ci - 1; i <= ci + 1; i++) {
      for (let j = cj - 1; j <= cj + 1; j++) {
        for (const a of cases.get(`${i},${j}`) ?? []) d = Math.min(d, Math.hypot(x - a[0], y - a[1]));
      }
    }
    return d;
  };
}

// --------------------------------------------------------------------- relief

/** Relief simulé (hauteurs en cm, int16) → fonction bilinéaire, bornée aux bords. */
function grilleRelief(relief) {
  if (!relief) return null;
  const octets = Uint8Array.from(atob(relief.hauteurs_cm), (c) => c.charCodeAt(0));
  const h = new Int16Array(octets.buffer);
  const { lignes, colonnes, pas } = relief;
  const [x0, y0] = relief.origine;
  return (x, y) => {
    const fx = Math.min(Math.max((x - x0) / pas, 0), colonnes - 1.000001);
    const fy = Math.min(Math.max((y - y0) / pas, 0), lignes - 1.000001);
    const c = Math.floor(fx), l = Math.floor(fy), tx = fx - c, ty = fy - l;
    const v = (ll, cc) => h[ll * colonnes + cc] / 100;
    return (v(l, c) * (1 - tx) + v(l, c + 1) * tx) * (1 - ty) + (v(l + 1, c) * (1 - tx) + v(l + 1, c + 1) * tx) * ty;
  };
}

/**
 * Hauteur du sol. Dans la zone de mission et sa lisière : le relief de la simulation (le même que
 * la physique) ; collines au-delà, pour l'horizon.
 */
function fonctionRelief(limites, marge, graine, relief = null) {
  const b1 = bruit2d(graine), b2 = bruit2d(graine + 17);
  const simule = grilleRelief(relief);
  return (x, y) => {
    const dx = Math.max(limites.xmin - marge - x, 0, x - limites.xmax - marge);
    const dy = Math.max(limites.ymin - marge - y, 0, y - limites.ymax - marge);
    const eloignement = Math.hypot(dx, dy);
    const fond = simule ? simule(x, y) : 0.015 * (b2(x * 0.9, y * 0.9) - 0.5);
    if (eloignement <= 0) return fond;
    const montee = Math.min(1, eloignement / 40);
    const collines = 9 * b1(x / 22, y / 22) + 3 * b2(x / 7, y / 7);
    return fond + montee * montee * collines + eloignement * 0.04;
  };
}

/** Grille à facettes : chaque triangle a sa propre couleur (non indexée, ombrage plat). */
function grilleFacettes(x0, x1, y0, y1, pas, relief, couleurFace, exclure = null) {
  const positions = [];
  const couleurs = [];
  const nx = Math.ceil((x1 - x0) / pas), ny = Math.ceil((y1 - y0) / pas);
  const couleur = new THREE.Color();
  // Sommets légèrement décalés : les facettes ne sont pas toutes alignées.
  const decalage = bruit2d(911);
  const sommet = (i, j) => {
    const x = x0 + i * pas + (decalage(i * 1.7, j * 1.3) - 0.5) * pas * 0.35;
    const y = y0 + j * pas + (decalage(j * 1.9, i * 1.1) - 0.5) * pas * 0.35;
    return [x, y, relief(x, y)];
  };
  const grille = [];
  for (let i = 0; i <= nx; i++) {
    grille.push([]);
    for (let j = 0; j <= ny; j++) grille[i].push(sommet(i, j));
  }
  const ajouter = (a, b, c) => {
    const cx = (a[0] + b[0] + c[0]) / 3, cy = (a[1] + b[1] + c[1]) / 3;
    if (exclure && exclure(cx, cy)) return;
    couleurFace(cx, cy, (a[2] + b[2] + c[2]) / 3, couleur);
    for (const s of [a, b, c]) {
      positions.push(...solVers3d(s[0], s[1], s[2]));
      couleurs.push(couleur.r, couleur.g, couleur.b);
    }
  };
  for (let i = 0; i < nx; i++) {
    for (let j = 0; j < ny; j++) {
      const a = grille[i][j], b = grille[i + 1][j], c = grille[i + 1][j + 1], d = grille[i][j + 1];
      if ((i + j) % 2) { ajouter(a, b, c); ajouter(a, c, d); } else { ajouter(a, b, d); ajouter(b, c, d); }
    }
  }
  const geometrie = new THREE.BufferGeometry();
  geometrie.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometrie.setAttribute('color', new THREE.Float32BufferAttribute(couleurs, 3));
  geometrie.computeVertexNormals();
  return geometrie;
}

// --------------------------------------------------------------------- végétation

/** Conifère normalisé (hauteur 1, rayon 1) : étages de cônes à facettes, clair en haut. */
function geometrieConifere(etages, evasement, graine) {
  const alea = aleatoire(graine);
  const morceaux = [];
  for (let e = 0; e < etages; e++) {
    const f = e / etages;
    const base = 0.22 + f * 0.62;
    const hauteur = (1 - base) * (e === etages - 1 ? 1 : 0.55 + evasement * 0.1);
    const rayon = (1 - f * 0.72) * (0.9 + alea() * 0.15);
    const cone = new THREE.ConeGeometry(rayon, hauteur, 6 + (e % 2), 1);
    cone.rotateY(alea() * Math.PI);
    cone.translate(0, base + hauteur / 2, 0);
    const n = cone.attributes.position.count;
    const teinte = new Float32Array(n * 3);
    for (let k = 0; k < n; k++) {
      // Dégradé vertical : bas des étages plus sombre, sommet de l'arbre plus clair.
      const y = cone.attributes.position.getY(k);
      const v = 0.72 + 0.28 * Math.min(1, y) + (y - base > hauteur * 0.5 ? 0.06 : 0);
      teinte[3 * k] = teinte[3 * k + 1] = teinte[3 * k + 2] = v;
    }
    cone.setAttribute('color', new THREE.BufferAttribute(teinte, 3));
    morceaux.push(cone.toNonIndexed());
  }
  const geometrie = mergeGeometries(morceaux);
  geometrie.computeVertexNormals();
  return geometrie;
}

function geometrieTronc() {
  const tronc = new THREE.CylinderGeometry(0.8, 1, 1, 6, 1);
  tronc.translate(0, 0.5, 0);
  return tronc;
}

/** Touffe d'herbe : trois brins triangulaires. */
function geometrieTouffe() {
  const brins = [];
  for (let k = 0; k < 3; k++) {
    const brin = new THREE.BufferGeometry();
    brin.setAttribute('position', new THREE.Float32BufferAttribute(
      [-0.05, 0, 0, 0.05, 0, 0, (k - 1) * 0.06, 1, 0.02], 3,
    ));
    brin.rotateY((k * Math.PI * 2) / 3);
    brins.push(brin);
  }
  const geometrie = mergeGeometries(brins);
  geometrie.computeVertexNormals();
  return geometrie;
}

function instances(geometrie, materiau, liste, groupe, { ombre = true } = {}) {
  if (!liste.length) return null;
  const maillage = new THREE.InstancedMesh(geometrie, materiau, liste.length);
  const m = new THREE.Matrix4();
  const q = new THREE.Quaternion();
  const axe = new THREE.Vector3(0, 1, 0);
  const couleur = new THREE.Color();
  liste.forEach(({ x, y, z, rot = 0, sx, sy, sz = sx, teinte }, i) => {
    m.compose(new THREE.Vector3(...solVers3d(x, y, z)), q.setFromAxisAngle(axe, rot), new THREE.Vector3(sx, sy, sz));
    maillage.setMatrixAt(i, m);
    if (teinte !== undefined) maillage.setColorAt(i, couleur.set(teinte));
  });
  maillage.castShadow = ombre;
  maillage.receiveShadow = ombre;
  groupe.add(maillage);
  return maillage;
}

// --------------------------------------------------------------------- rochers

function geometrieRocher(rocher, alea, z0 = 0) {
  // Même contour que la physique au sol ; épaulement et sommet à facettes irrégulières.
  const { x, y, contour, hauteur, retrait_sommet: retrait } = rocher;
  const n = contour.length;
  const anneau = (f, z, jitter) => contour.map(([px, py]) => {
    const j = 1 + (alea() - 0.5) * jitter;
    return [x + px * f * j, y + py * f * j, z0 + z + (alea() - 0.5) * jitter * hauteur * 0.3];
  });
  const bas = anneau(1.0, -0.3, 0);
  const milieu = anneau(1.0, hauteur * 0.5, 0.08);
  const haut = anneau(retrait, hauteur, 0.18);
  const centre = [x, y, z0 + hauteur * (1.05 + alea() * 0.1)];
  const positions = [];
  const couleurs = [];
  const gris = new THREE.Color(choisir(PALETTE.rocher, alea()));
  const mousse = new THREE.Color(PALETTE.mousse);
  const face = (a, b, c, dessus) => {
    for (const s of [a, b, c]) positions.push(...solVers3d(s[0], s[1], s[2]));
    const couleur = dessus && alea() < 0.6 ? mousse : gris.clone().offsetHSL(0, 0, (alea() - 0.5) * 0.06);
    for (let k = 0; k < 3; k++) couleurs.push(couleur.r, couleur.g, couleur.b);
  };
  for (let i = 0; i < n; i++) {
    const j = (i + 1) % n;
    face(bas[i], bas[j], milieu[j]); face(bas[i], milieu[j], milieu[i]);
    face(milieu[i], milieu[j], haut[j]); face(milieu[i], haut[j], haut[i]);
    face(haut[i], haut[j], centre, true);
  }
  const geometrie = new THREE.BufferGeometry();
  geometrie.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometrie.setAttribute('color', new THREE.Float32BufferAttribute(couleurs, 3));
  geometrie.computeVertexNormals();
  return geometrie;
}

// --------------------------------------------------------------------- repères

// --------------------------------------------------------------------- troncs couchés

/** Troncs couchés au sol (sous le plan du lidar), à la taille exacte des obstacles physiques. */
function construireTroncsCouches(groupe, troncs, relief, alea) {
  const ecorce = new THREE.MeshStandardMaterial({ color: 0xffffff, flatShading: true, roughness: 1 });
  const coeur = new THREE.MeshStandardMaterial({ color: PALETTE.coeurBois, flatShading: true, roughness: 1 });
  for (const [x, y, angle, longueur, rayon] of troncs ?? []) {
    const geometrie = new THREE.CylinderGeometry(rayon, rayon * 1.08, longueur, 7, 1);
    const tronc = new THREE.Mesh(geometrie, [ecorce.clone(), coeur, coeur]);
    tronc.material[0].color.set(choisir(PALETTE.boisMort, alea()));
    tronc.position.set(...solVers3d(x, y, relief(x, y) + rayon));
    tronc.rotation.set(0, angle, Math.PI / 2, 'YXZ');
    tronc.castShadow = tronc.receiveShadow = true;
    groupe.add(tronc);
  }
}

function etiquette(texte) {
  const toile = document.createElement('canvas');
  const ctx = toile.getContext('2d');
  const police = '600 40px system-ui, sans-serif';
  ctx.font = police;
  const largeur = Math.ceil(ctx.measureText(texte).width) + 36;
  toile.width = largeur;
  toile.height = 64;
  ctx.font = police;
  ctx.fillStyle = 'rgba(20,24,20,0.72)';
  ctx.beginPath();
  ctx.roundRect(0, 0, largeur, 64, 32);
  ctx.fill();
  ctx.fillStyle = '#ffffff';
  ctx.textBaseline = 'middle';
  ctx.fillText(texte, 18, 34);
  const texture = new THREE.CanvasTexture(toile);
  texture.colorSpace = THREE.SRGBColorSpace;
  // Toujours lisible, même derrière un arbre (comme un repère de navigation).
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
    map: texture, depthWrite: false, depthTest: false, sizeAttenuation: false, transparent: true,
  }));
  sprite.renderOrder = 10;
  // Taille fixe à l'écran (fraction de la hauteur de la vue), quelle que soit la distance.
  const h = 0.032;
  sprite.scale.set((largeur / 64) * h, h, 1);
  return sprite;
}

function reperes(groupe, limites, lieux) {
  // Piquets de bornage tous les 5 m, pointe orange ; fanion sur les lieux nommés.
  const piquets = [];
  const ajouter = (x, y) => piquets.push({ x, y, z: 0, sx: 0.04, sy: 0.9, rot: 0 });
  for (let x = limites.xmin; x <= limites.xmax + 1e-6; x += 5) { ajouter(x, limites.ymin); ajouter(x, limites.ymax); }
  for (let y = limites.ymin + 5; y < limites.ymax - 1e-6; y += 5) { ajouter(limites.xmin, y); ajouter(limites.xmax, y); }
  const piquet = new THREE.CylinderGeometry(1, 1, 1, 5);
  piquet.translate(0, 0.5, 0);
  instances(piquet, new THREE.MeshStandardMaterial({ color: 0xd8c7a0, flatShading: true }), piquets, groupe);
  const pointe = new THREE.ConeGeometry(1, 1, 5);
  pointe.translate(0, 0.5, 0);
  instances(
    pointe, new THREE.MeshStandardMaterial({ color: 0xf08a24, flatShading: true }),
    piquets.map((p) => ({ ...p, z: 0.9, sx: 0.06, sy: 0.12 })), groupe,
  );
  // Lieux nommés : marquage hexagonal au sol (le rover peut rouler dessus) et étiquette.
  const anneau = new THREE.RingGeometry(0.75, 0.95, 6, 1);
  anneau.rotateX(-Math.PI / 2);
  const materiauAnneau = new THREE.MeshBasicMaterial({ color: 0xf08a24, transparent: true, opacity: 0.85 });
  for (const lieu of lieux) {
    const marque = new THREE.Mesh(anneau, materiauAnneau);
    marque.position.set(...solVers3d(lieu.x, lieu.y, 0.03));
    const texte = etiquette(lieu.nom);
    texte.position.set(...solVers3d(lieu.x, lieu.y, 1.6));
    groupe.add(marque, texte);
  }
}

// --------------------------------------------------------------------- ciel et lointain

/** Dôme de ciel en dégradé, soleil et chaîne de montagnes (hors brouillard). */
export function construireCiel(scene, directionSoleil) {
  const dome = new THREE.SphereGeometry(450, 32, 16);
  const zenith = new THREE.Color(0x5e9ccf), horizon = new THREE.Color(0xf1e3c6);
  const couleurs = [];
  for (let k = 0; k < dome.attributes.position.count; k++) {
    const h = Math.max(0, dome.attributes.position.getY(k) / 450);
    const c = horizon.clone().lerp(zenith, Math.pow(h, 0.55));
    couleurs.push(c.r, c.g, c.b);
  }
  dome.setAttribute('color', new THREE.Float32BufferAttribute(couleurs, 3));
  const ciel = new THREE.Mesh(dome, new THREE.MeshBasicMaterial({
    vertexColors: true, side: THREE.BackSide, fog: false, depthWrite: false,
  }));
  ciel.renderOrder = -2;
  const soleil = new THREE.Mesh(
    new THREE.CircleGeometry(14, 24),
    new THREE.MeshBasicMaterial({ color: 0xfff4d8, fog: false, transparent: true, opacity: 0.9 }),
  );
  soleil.position.copy(directionSoleil.clone().normalize().multiplyScalar(420));
  soleil.lookAt(0, 0, 0);
  const alea = aleatoire(7);
  const montagnes = [];
  for (let k = 0; k < 46; k++) {
    const a = (k / 46) * Math.PI * 2 + alea() * 0.1;
    const d = 230 + alea() * 70;
    const m = new THREE.ConeGeometry(35 + alea() * 45, 30 + alea() * 55, 5 + Math.floor(alea() * 3), 1);
    m.rotateY(alea() * Math.PI);
    m.translate(Math.cos(a) * d, (m.parameters.height / 2) - 6, Math.sin(a) * d);
    const c = new THREE.Color(choisir(PALETTE.montagnes, alea()));
    const col = [];
    for (let i = 0; i < m.attributes.position.count; i++) {
      const y = m.attributes.position.getY(i);
      const v = c.clone().offsetHSL(0, 0, y > 25 ? 0.08 : 0);
      col.push(v.r, v.g, v.b);
    }
    m.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
    montagnes.push(m.toNonIndexed());
  }
  const chaine = new THREE.Mesh(
    mergeGeometries(montagnes),
    new THREE.MeshLambertMaterial({ vertexColors: true, flatShading: true, fog: false }),
  );
  const lointain = new THREE.Group();
  lointain.add(ciel, soleil, chaine);
  lointain.name = 'lointain';
  scene.add(lointain);
  return lointain;
}

// --------------------------------------------------------------------- construction

function couleurSol(monde, relief) {
  const b = bruit2d(monde.graine + 3), b2 = bruit2d(monde.graine + 5);
  const alea = aleatoire(monde.graine + 99);
  const tousArbres = [...monde.arbres, ...(monde.lisiere ?? [])];
  const distanceArbre = indexArbres(tousArbres);
  const piste = monde.piste?.points ?? [];
  const demi = monde.piste?.demi_largeur ?? 0;
  const cl = monde.clairiere;
  const distancePiste = (x, y) => {
    let d = Infinity;
    for (let k = 1; k < piste.length; k++) {
      const [ax, ay] = piste[k - 1], [bx, by] = piste[k];
      if (Math.abs(ax - x) > 6 && Math.abs(bx - x) > 6) continue;
      d = Math.min(d, distanceSegment(x, y, ax, ay, bx, by));
    }
    return d;
  };
  return (x, y, z, sortie) => {
    const u = alea();
    const n = b(x / 6, y / 6);
    let liste = PALETTE.herbe;
    if (z > 1.5) liste = PALETTE.collines;
    else if (cl && Math.hypot(x - cl.x, y - cl.y) < cl.rayon - 0.5 * b2(x, y)) liste = PALETTE.clairiere;
    else if (distanceArbre(x, y) < 1.6 + 1.2 * n) liste = PALETTE.sousBois;
    const dp = piste.length ? distancePiste(x, y) : Infinity;
    if (dp < demi + 0.15 * (b2(x * 2, y * 2) - 0.5)) liste = PALETTE.terre;
    else if (dp < demi + 0.55) liste = PALETTE.bordPiste;
    sortie.set(choisir(liste, (u + n) / 2));
    sortie.offsetHSL(0, 0, (u - 0.5) * 0.035);
  };
}

function construireSol(groupe, monde, relief, marge) {
  const lim = monde.limites;
  const couleur = couleurSol(monde, relief);
  const materiau = new THREE.MeshStandardMaterial({ vertexColors: true, flatShading: true, roughness: 1 });
  // Cœur fin (zone de mission et lisière), puis couronne grossière jusqu'à l'horizon.
  const x0 = lim.xmin - marge, x1 = lim.xmax + marge, y0 = lim.ymin - marge, y1 = lim.ymax + marge;
  const coeur = new THREE.Mesh(grilleFacettes(x0, x1, y0, y1, 0.45, relief, couleur), materiau);
  const dedans = (x, y) => x > x0 + 2 && x < x1 - 2 && y > y0 + 2 && y < y1 - 2;
  const couronne = new THREE.Mesh(
    grilleFacettes(x0 - 170, x1 + 170, y0 - 170, y1 + 170, 5, (x, y) => relief(x, y) - 0.05, couleur, dedans),
    materiau,
  );
  coeur.receiveShadow = couronne.receiveShadow = true;
  groupe.add(coeur, couronne);
}

function construireArbres(groupe, monde, relief, marge, alea) {
  const conifereA = geometrieConifere(4, 0.2, 11), conifereB = geometrieConifere(3, 1, 23);
  const tronc = geometrieTronc();
  const matAiguilles = new THREE.MeshStandardMaterial({ vertexColors: true, flatShading: true, roughness: 0.9 });
  const matTronc = new THREE.MeshStandardMaterial({ vertexColors: false, flatShading: true, roughness: 1 });
  const troncs = [], canopeesA = [], canopeesB = [];
  const planter = (x, y, r, h) => {
    const z = relief(x, y);
    troncs.push({ x, y, z, sx: r, sy: h * 0.45, rot: alea() * 6.3, teinte: choisir(PALETTE.tronc, alea()) });
    const largeur = Math.max(1.1, h * 0.19) * (0.85 + alea() * 0.35);
    const base = h * (0.18 + alea() * 0.08);
    const instance = {
      x, y, z: z + base, sx: largeur, sy: h - base, rot: alea() * 6.3,
      teinte: choisir(PALETTE.aiguilles, alea()),
    };
    (alea() < 0.6 ? canopeesA : canopeesB).push(instance);
  };
  // Arbres physiques (zone et lisière) : troncs au rayon exact de la simulation.
  for (const [x, y, r, h] of [...monde.arbres, ...(monde.lisiere ?? [])]) planter(x, y, r, h);
  // Forêt de fond, décorative, au-delà de la lisière (le rover n'y va jamais).
  const lim = monde.limites;
  const bord = marge + 1;
  for (let k = 0; k < 2600; k++) {
    const x = lim.xmin - 70 + alea() * (lim.xmax - lim.xmin + 140);
    const y = lim.ymin - 70 + alea() * (lim.ymax - lim.ymin + 140);
    if (x > lim.xmin - bord && x < lim.xmax + bord && y > lim.ymin - bord && y < lim.ymax + bord) continue;
    planter(x, y, 0.12 + alea() * 0.15, 7 + alea() * 10);
  }
  instances(tronc, matTronc, troncs, groupe);
  instances(conifereA, matAiguilles, canopeesA, groupe);
  instances(conifereB, matAiguilles, canopeesB, groupe);
}

function construirePetitsElements(groupe, monde, relief, alea) {
  const lim = monde.limites;
  const piste = monde.piste?.points ?? [];
  const demi = monde.piste?.demi_largeur ?? 0;
  const surPiste = (x, y) => piste.some((p, k) => k > 0
    && distanceSegment(x, y, piste[k - 1][0], piste[k - 1][1], p[0], p[1]) < demi + 0.2);
  const pres = (x, y, liste, marge) => liste.some((o) => Math.hypot(x - o.x, y - o.y) < o.rayon + marge);
  const rochers = monde.rochers.map((r) => ({ x: r.x, y: r.y, rayon: Math.max(...r.contour.map(([a, b]) => Math.hypot(a, b))) }));
  const touffes = [], fleurs = [], cailloux = [];
  for (let k = 0; k < 9000; k++) {
    const x = lim.xmin - 8 + alea() * (lim.xmax - lim.xmin + 16);
    const y = lim.ymin - 8 + alea() * (lim.ymax - lim.ymin + 16);
    if (surPiste(x, y) || pres(x, y, rochers, 0.1)) continue;
    const h = 0.14 + alea() * 0.22;
    touffes.push({ x, y, z: relief(x, y), sx: 1 + alea(), sy: h, rot: alea() * 6.3, teinte: alea() < 0.5 ? 0x8fae52 : 0x6e9140 });
  }
  const cl = monde.clairiere;
  if (cl) {
    for (let k = 0; k < 420; k++) {
      const a = alea() * Math.PI * 2, d = Math.sqrt(alea()) * cl.rayon;
      const x = cl.x + Math.cos(a) * d, y = cl.y + Math.sin(a) * d;
      fleurs.push({ x, y, z: relief(x, y) + 0.12 + alea() * 0.08, sx: 0.045, sy: 0.045, rot: alea() * 6.3, teinte: choisir(PALETTE.fleurs, alea()) });
    }
  }
  for (let k = 0; k < piste.length * 2; k++) {
    const [px, py] = piste[Math.floor(alea() * piste.length)];
    const x = px + (alea() - 0.5) * demi * 2, y = py + (alea() - 0.5) * demi * 2;
    cailloux.push({ x, y, z: relief(x, y), sx: 0.03 + alea() * 0.05, sy: 0.02 + alea() * 0.03, rot: alea() * 6.3, teinte: choisir(PALETTE.rocher, alea()) });
  }
  const matPlat = (options = {}) => new THREE.MeshStandardMaterial({ flatShading: true, roughness: 1, ...options });
  instances(geometrieTouffe(), matPlat({ side: THREE.DoubleSide }), touffes, groupe, { ombre: false });
  instances(new THREE.IcosahedronGeometry(1, 0), matPlat(), fleurs, groupe, { ombre: false });
  instances(new THREE.DodecahedronGeometry(1, 0), matPlat(), cailloux, groupe, { ombre: false });
}

/** Construit la forêt dans `groupe` à partir du message `monde` et des lieux du scénario. */
export function construireForet(groupe, monde, lieux) {
  const alea = aleatoire(monde.graine);
  const marge = (monde.largeur_lisiere ?? 10) + 2;
  const relief = fonctionRelief(monde.limites, marge, monde.graine, monde.relief);
  construireSol(groupe, monde, relief, marge);
  construireArbres(groupe, monde, relief, marge, alea);
  if (monde.rochers.length) {
    const rochers = new THREE.Mesh(
      mergeGeometries(monde.rochers.map((r) => geometrieRocher(r, alea, relief(r.x, r.y)))),
      new THREE.MeshStandardMaterial({ vertexColors: true, flatShading: true, roughness: 0.95 }),
    );
    rochers.castShadow = rochers.receiveShadow = true;
    groupe.add(rochers);
  }
  construirePetitsElements(groupe, monde, relief, alea);
  construireTroncsCouches(groupe, monde.troncs_couches, relief, alea);
  reperes(groupe, monde.limites, lieux);
  return relief;
}

export { etiquette };

/** Mode robot réel : le monde n'est pas connu. Sol neutre, repères et lieux seulement. */
export function construireSolSeul(groupe, limites, lieux) {
  const monde = { graine: 1, limites, arbres: [], lisiere: [], rochers: [], piste: null, clairiere: null };
  const relief = fonctionRelief(limites, 12, 1);
  construireSol(groupe, monde, relief, 12);
  reperes(groupe, limites, lieux);
}
