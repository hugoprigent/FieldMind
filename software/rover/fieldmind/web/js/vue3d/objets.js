// Objets du monde reconstruit (message `scene`) : murs, prolongements extrapolés, arbres
// (poteaux), blocs, obstacles bas, personnes, animaux et étiquettes. Chaque type est une couche
// de maillages instanciés (un appel de dessin par couche, quel que soit le nombre d'objets) ;
// chaque type partage un seul modèle mis à l'échelle (`hauteur`, `rayon`…). Personnes et
// animaux : silhouettes bas-poly tournées vers le rover, oubliées vite. À réception d'un message,
// seules les cibles changent ; à chaque image, positions et dimensions convergent en douceur
// (lissage exponentiel), l'opacité suit l'apparition, l'âge (`age_s`) et la disparition.

import * as THREE from '../../vendor/three/three.module.js';
import { mergeGeometries } from '../../vendor/three/addons/utils/BufferGeometryUtils.js';
import { PALETTE, materiauBoite, materiauArbre, materiauOmbre, materiauSilhouette } from './materiaux.js';

const APPARITION_S = 0.6;
const DISPARITION_S = 0.6;
const LISSAGE_S = 0.16;
const SAUT_M = 2.0; // au-delà, l'objet est replacé sans glisser (changement de repère)
const OPACITE_EXTRAPOLE = 0.5;
const AGE_MAX_S = 31; // le rover oublie les objets non revus au bout de 30 s
// Êtres vivants (détection continue) : oubliés 5 s après la dernière vue, fondu rapide.
const VIVANTS = new Set(['personne', 'animal']);
const AGE_MAX_VIVANT_S = 5.5;
const DISPARITION_VIVANT_S = 0.3;
const TEXTES_GENERIQUES = new Set(['personne', 'person', 'animal']);

function lisser(courant, cible, k) {
  return courant + (cible - courant) * k;
}

function lisserAngle(courant, cible, k) {
  let d = cible - courant;
  d = Math.atan2(Math.sin(d), Math.cos(d));
  return courant + d * k;
}

function smoothstep(a, b, x) {
  const t = Math.min(1, Math.max(0, (x - a) / (b - a)));
  return t * t * (3 - 2 * t);
}

function hacher(id) {
  let h = 2166136261;
  const s = String(id);
  for (let i = 0; i < s.length; i++) h = Math.imul(h ^ s.charCodeAt(i), 16777619);
  return ((h >>> 0) % 10000) / 10000;
}

function nombre(v, defaut) {
  return Number.isFinite(v) ? v : defaut;
}

/** Matrice translation · rotation autour de z · échelle, écrite en place (ordre colonnes). */
function ecrireMatrice(t, i, x, y, z, cap, sx, sy, sz) {
  const c = Math.cos(cap), s = Math.sin(cap), o = 16 * i;
  t[o] = c * sx; t[o + 1] = s * sx; t[o + 2] = 0; t[o + 3] = 0;
  t[o + 4] = -s * sy; t[o + 5] = c * sy; t[o + 6] = 0; t[o + 7] = 0;
  t[o + 8] = 0; t[o + 9] = 0; t[o + 10] = sz; t[o + 11] = 0;
  t[o + 12] = x; t[o + 13] = y; t[o + 14] = z; t[o + 15] = 1;
}

/** Couche d'instances : une ou plusieurs parties (même nombre d'instances, matrices propres). */
class Couche {
  constructor(parent, parties, { capacite = 64, ordre = 0, z = 0 } = {}) {
    this.parent = parent;
    this.parties = parties;
    this.ordre = ordre;
    this.z = z;
    this.occupants = [];
    this.sale = false;
    this.capacite = 0;
    this.maillages = [];
    this._agrandir(capacite);
  }

  _agrandir(capacite) {
    const anciens = this.maillages, ancienParams = this.params;
    this.params = new THREE.InstancedBufferAttribute(new Float32Array(capacite * 4), 4);
    this.params.setUsage(THREE.DynamicDrawUsage);
    if (ancienParams) this.params.array.set(ancienParams.array);
    this.maillages = this.parties.map(({ geometrie, materiau }, k) => {
      geometrie.setAttribute('aParams', this.params);
      const m = new THREE.InstancedMesh(geometrie, materiau, capacite);
      m.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
      if (anciens[k]) m.instanceMatrix.array.set(anciens[k].instanceMatrix.array);
      m.count = this.occupants.length;
      m.frustumCulled = false;
      m.renderOrder = this.ordre;
      m.position.z = this.z;
      this.parent.add(m);
      return m;
    });
    for (const m of anciens) {
      this.parent.remove(m);
      m.dispose();
    }
    this.capacite = capacite;
  }

  ajouter(entree) {
    if (this.occupants.length >= this.capacite) this._agrandir(this.capacite * 2);
    this.occupants.push(entree);
    for (const m of this.maillages) m.count = this.occupants.length;
    return this.occupants.length - 1;
  }

  /** Retire l'instance `i` en y déplaçant la dernière ; rend l'entrée déplacée (ou null). */
  retirer(i, champ) {
    const dernier = this.occupants.length - 1;
    let deplace = null;
    if (i !== dernier) {
      deplace = this.occupants[dernier];
      this.occupants[i] = deplace;
      deplace[champ] = i;
      this.params.array.copyWithin(4 * i, 4 * dernier, 4 * dernier + 4);
      for (const m of this.maillages) m.instanceMatrix.array.copyWithin(16 * i, 16 * dernier, 16 * dernier + 16);
    }
    this.occupants.pop();
    for (const m of this.maillages) {
      m.count = this.occupants.length;
      m.instanceMatrix.needsUpdate = true;
    }
    this.params.needsUpdate = true;
    return deplace;
  }

  matrices(k) {
    return this.maillages[k].instanceMatrix.array;
  }

  signalerMatrices() {
    for (const m of this.maillages) m.instanceMatrix.needsUpdate = true;
  }

  detruire() {
    for (const m of this.maillages) {
      this.parent.remove(m);
      m.dispose();
    }
    for (const { geometrie, materiau } of this.parties) {
      geometrie.dispose();
      materiau.dispose();
    }
  }
}

/** Boîte unité : x, y dans [-0,5 ; 0,5], z dans [0 ; 1]. */
function boiteUnite() {
  const g = new THREE.BoxGeometry(1, 1, 1);
  g.translate(0, 0, 0.5);
  return g;
}

/** Cylindre unité : rayon 1 dans (x, y), z dans [0 ; 1]. */
function cylindreUnite() {
  const g = new THREE.CylinderGeometry(1, 1, 1, 20, 1);
  g.rotateX(Math.PI / 2);
  g.translate(0, 0, 0.5);
  return g;
}

/** Fusion de primitives en un seul maillage bas-poly (attributs position et normale). */
function fusionner(parties) {
  const g = mergeGeometries(parties.map((p) => {
    const n = p.index ? p.toNonIndexed() : p;
    n.deleteAttribute('uv');
    if (n !== p) p.dispose();
    return n;
  }));
  g.computeVertexNormals();
  return g;
}

function capsule(rayon, longueur, x, y, z, { rx = 0, ry = 0, sx = 1, sy = 1, sz = 1 } = {}) {
  const g = new THREE.CapsuleGeometry(rayon, longueur, 2, 7);
  g.rotateX(Math.PI / 2); // axe z
  if (rx) g.rotateX(rx);
  if (ry) g.rotateY(ry);
  g.scale(sx, sy, sz);
  g.translate(x, y, z);
  return g;
}

/** Personne unité : hauteur 1, tournée vers +x (jambes, torse, bras, tête). */
function geometriePersonne() {
  const tete = new THREE.IcosahedronGeometry(0.095, 1);
  tete.translate(0, 0, 0.905);
  return fusionner([
    capsule(0.056, 0.34, 0, 0.066, 0.235),
    capsule(0.056, 0.34, 0, -0.066, 0.235),
    capsule(0.12, 0.2, 0, 0, 0.64, { sx: 0.72 }),
    capsule(0.04, 0.3, 0, 0.17, 0.6, { rx: -0.12 }),
    capsule(0.04, 0.3, 0, -0.17, 0.6, { rx: 0.12 }),
    tete,
  ]);
}

/** Animal unité : hauteur 1 (haut de la tête), tourné vers +x (corps, 4 pattes, tête, queue). */
function geometrieAnimal() {
  const pattes = [];
  for (const x of [0.3, -0.28]) {
    for (const y of [0.11, -0.11]) {
      const p = new THREE.CylinderGeometry(0.055, 0.042, 0.46, 6);
      p.rotateX(Math.PI / 2);
      p.translate(x, y, 0.23);
      pattes.push(p);
    }
  }
  const corps = capsule(0.19, 0.5, 0, 0, 0.58, { sz: 0.9, ry: Math.PI / 2 });
  const cou = new THREE.CylinderGeometry(0.075, 0.1, 0.3, 6);
  cou.rotateX(Math.PI / 2);
  cou.rotateY(0.75);
  cou.translate(0.4, 0, 0.73);
  const tete = new THREE.IcosahedronGeometry(0.14, 0);
  tete.translate(0.52, 0, 0.86);
  const museau = new THREE.BoxGeometry(0.16, 0.1, 0.09);
  museau.translate(0.67, 0, 0.82);
  const oreilles = [0.07, -0.07].map((y) => {
    const o = new THREE.ConeGeometry(0.045, 0.1, 4);
    o.rotateX(Math.PI / 2);
    o.translate(0.48, y, 0.99);
    return o;
  });
  const queue = new THREE.CylinderGeometry(0.022, 0.035, 0.32, 5);
  queue.rotateX(Math.PI / 2);
  queue.rotateY(-0.9);
  queue.translate(-0.5, 0, 0.72);
  return fusionner([...pattes, corps, cou, tete, museau, ...oreilles, queue]);
}

const TYPES_BOITES = new Set(['mur', 'bloc', 'bas']);

export class Objets {
  constructor(parent, uniformes) {
    this.parent = parent;
    this.uniformes = uniformes;
    this.entrees = new Map();
    this.etiquettes = new Map();
    this.temps = 0;
    this.couches = {
      mur: new Couche(parent, [{ geometrie: boiteUnite(), materiau: materiauBoite(uniformes, { couleur: PALETTE.mur, arete: PALETTE.murArete, trait: 0.75, adoucir: 0.03 }) }], { ordre: 0 }),
      murExtrapole: new Couche(parent, [{ geometrie: boiteUnite(), materiau: materiauBoite(uniformes, { couleur: PALETTE.mur, arete: PALETTE.murArete, trait: 0.6, degrade: true, depthWrite: false }) }], { ordre: 4, capacite: 32 }),
      bloc: new Couche(parent, [{ geometrie: boiteUnite(), materiau: materiauBoite(uniformes, { couleur: PALETTE.bloc, arete: PALETTE.blocArete, trait: 0.55, adoucir: 0.07 }) }], { ordre: 0 }),
      bas: new Couche(parent, [{ geometrie: boiteUnite(), materiau: materiauBoite(uniformes, { couleur: PALETTE.bas, arete: PALETTE.basArete, trait: 0.6, adoucir: 0.05, pied: 0.2 }) }], { ordre: 0, capacite: 32 }),
      arbre: new Couche(parent, [
        { geometrie: cylindreUnite(), materiau: materiauArbre(uniformes, { couleur: PALETTE.bloc, arete: PALETTE.blocArete, haut: 1.0, bas: 0.68 }) },
      ], { ordre: 0, capacite: 128 }),
      personne: new Couche(parent, [{ geometrie: geometriePersonne(), materiau: materiauSilhouette(uniformes) }], { ordre: 0, capacite: 16 }),
      animal: new Couche(parent, [{ geometrie: geometrieAnimal(), materiau: materiauSilhouette(uniformes) }], { ordre: 0, capacite: 16 }),
      ombre: new Couche(parent, [{ geometrie: new THREE.PlaneGeometry(1, 1), materiau: materiauOmbre(uniformes) }], { ordre: -8, capacite: 128, z: 0.003 }),
    };
    this.roverScene = { x: 0, y: 0 };
    this.listeCouches = Object.values(this.couches);
    // Étiquettes : ressources partagées.
    this.geoTige = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 1)]);
    this.geoPied = new THREE.RingGeometry(0.045, 0.075, 24);
    this.canvasMesure = document.createElement('canvas').getContext('2d');
  }

  // ---------------------------------------------------------------- réception

  /** Applique un message `scene` (cibles seulement ; le dessin converge ensuite). */
  recevoir(message, maintenant) {
    const objets = Array.isArray(message?.objets) ? message.objets : [];
    const r = message?.rover;
    if (message?.repere === 'base_link') this.roverScene = { x: 0, y: 0 };
    else if (r && Number.isFinite(r.x) && Number.isFinite(r.y)) this.roverScene = { x: r.x, y: r.y };
    const vus = new Set();
    for (const o of objets) {
      if (!o || o.id === undefined || o.id === null) continue;
      const type = o.type === 'etiquette' || o.type === 'arbre' || TYPES_BOITES.has(o.type) || VIVANTS.has(o.type) ? o.type : null;
      if (!type || !Number.isFinite(o.x) || !Number.isFinite(o.y)) continue;
      const cle = `${type === 'etiquette' ? 'e' : 'o'}${o.id}`;
      vus.add(cle);
      if (type === 'etiquette') {
        this._recevoirEtiquette(cle, o, maintenant);
      } else {
        this._recevoirObjet(cle, type, o, maintenant);
        // Étiquette du détecteur (« chien », « enfant »…) au-dessus d'un être vivant.
        const texte = VIVANTS.has(type) ? String(o.texte ?? '').trim() : '';
        if (texte && !TEXTES_GENERIQUES.has(texte.toLowerCase())) {
          vus.add(`v${o.id}`);
          this._recevoirEtiquette(`v${o.id}`, o, maintenant, true);
        }
      }
    }
    for (const [cle, e] of this.entrees) if (!vus.has(cle) && e.sortie === null) e.sortie = maintenant;
    for (const [cle, e] of this.etiquettes) if (!vus.has(cle) && e.sortie === null) e.sortie = maintenant;
    this._raccorderExtrapoles();
    this._hausserEtiquettes();
  }

  _recevoirObjet(cle, type, o, maintenant) {
    const c = {
      x: o.x, y: o.y, cap: nombre(o.cap, 0),
      l: 0, w: 0, h: 0, r: 0,
    };
    if (type === 'mur') {
      c.l = Math.max(0.05, nombre(o.longueur, 1));
      c.w = Math.max(0.05, nombre(o.epaisseur, 0.12));
      c.h = Math.max(0.1, nombre(o.hauteur, 1.0));
    } else if (type === 'arbre') {
      c.r = Math.max(0.03, nombre(o.rayon, 0.1));
      c.h = Math.max(0.2, nombre(o.hauteur, 0.9));
    } else if (VIVANTS.has(type)) {
      c.r = Math.max(0.08, nombre(o.rayon, 0.25));
      c.h = Math.max(0.15, nombre(o.hauteur, type === 'personne' ? 1.7 : 0.6));
      c.cap = Math.atan2(this.roverScene.y - o.y, this.roverScene.x - o.x); // face au rover
    } else {
      c.l = Math.max(0.04, nombre(o.longueur, 0.4));
      c.w = Math.max(0.04, nombre(o.largeur, c.l));
      c.h = Math.max(type === 'bas' ? 0.02 : 0.05, nombre(o.hauteur, type === 'bas' ? 0.1 : 0.5));
    }
    const extrap = type === 'mur' && o.extrapole === true;
    const nomCouche = extrap ? 'murExtrapole' : type;
    let e = this.entrees.get(cle);
    if (e && e.nomCouche !== nomCouche) {
      this._liberer(e);
      this.entrees.delete(cle);
      e = null;
    }
    if (!e) {
      const graine = hacher(o.id);
      e = {
        cle, id: o.id, type, nomCouche, c, v: { ...c }, naissance: maintenant, sortie: null,
        age: 0, recu: maintenant, danger: 0, dangerV: 0, extrap, degrade: 0,
        lum: 0.93 + 0.12 * graine, rotation: graine * Math.PI * 2, bouge: true, i: -1, iOmbre: -1,
        vivant: VIVANTS.has(type),
      };
      e.i = this.couches[nomCouche].ajouter(e);
      if (!extrap) e.iOmbre = this.couches.ombre.ajouter(e);
      this.entrees.set(cle, e);
    } else {
      const saut = Math.hypot(c.x - e.v.x, c.y - e.v.y) > SAUT_M;
      e.c = c;
      if (saut) e.v = { ...c };
      if (e.sortie !== null) { // revenu après une absence : on repart de l'opacité actuelle
        e.naissance = maintenant - APPARITION_S * this._alphaApparition(e, maintenant);
        e.sortie = null;
      }
      e.bouge = true;
    }
    e.age = Math.max(0, nombre(o.age_s, 0));
    e.recu = maintenant;
    e.danger = o.danger === true ? 1 : 0;
  }

  _recevoirEtiquette(cle, o, maintenant, court = false) {
    let e = this.etiquettes.get(cle);
    const texte = String(o.texte ?? '').trim().slice(0, 48) || '?';
    if (!e) {
      e = { cle, id: o.id, c: { x: o.x, y: o.y }, v: { x: o.x, y: o.y }, z: 0.8, zV: 0.8, naissance: maintenant, sortie: null, age: 0, texte: null, court };
      e.groupe = new THREE.Group();
      e.matTige = new THREE.LineBasicMaterial({ color: PALETTE.lidarPoints, transparent: true, opacity: 0, depthWrite: false });
      e.tige = new THREE.Line(this.geoTige, e.matTige);
      e.matPied = new THREE.MeshBasicMaterial({ color: PALETTE.blanc, transparent: true, opacity: 0, depthWrite: false });
      e.pied = new THREE.Mesh(this.geoPied, e.matPied);
      e.pied.position.z = 0.006;
      e.matSprite = new THREE.SpriteMaterial({ transparent: true, depthTest: false, depthWrite: false, sizeAttenuation: false, opacity: 0 });
      e.sprite = new THREE.Sprite(e.matSprite);
      e.sprite.center.set(0.5, 0);
      e.sprite.renderOrder = 20;
      e.tige.renderOrder = 3;
      e.pied.renderOrder = -6;
      e.groupe.add(e.tige, e.pied, e.sprite);
      e.tige.visible = e.pied.visible = !court;
      this.parent.add(e.groupe);
      this.etiquettes.set(cle, e);
    } else {
      e.c = { x: o.x, y: o.y };
      if (Math.hypot(o.x - e.v.x, o.y - e.v.y) > SAUT_M) e.v = { ...e.c };
      if (e.sortie !== null) {
        e.naissance = maintenant;
        e.sortie = null;
      }
    }
    if (texte !== e.texte) this._dessinerEtiquette(e, texte);
    e.age = Math.max(0, nombre(o.age_s, 0));
    e.recu = maintenant;
  }

  _dessinerEtiquette(e, texte) {
    e.texte = texte;
    const echelle = 2, police = 13 * echelle, marge = 9 * echelle, hauteur = 24 * echelle;
    const famille = 'system-ui, -apple-system, "Segoe UI", Roboto, sans-serif';
    this.canvasMesure.font = `600 ${police}px ${famille}`;
    const largeur = Math.ceil(this.canvasMesure.measureText(texte).width + 2 * marge);
    const canvas = document.createElement('canvas');
    canvas.width = largeur;
    canvas.height = hauteur;
    const g = canvas.getContext('2d');
    const r = hauteur / 2;
    g.beginPath();
    g.roundRect(1, 1, largeur - 2, hauteur - 2, r - 1);
    g.fillStyle = PALETTE.etiquetteFond;
    g.fill();
    g.lineWidth = 2;
    g.strokeStyle = PALETTE.etiquetteBord;
    g.stroke();
    g.font = `600 ${police}px ${famille}`;
    g.fillStyle = PALETTE.etiquetteTexte;
    g.textBaseline = 'middle';
    g.textAlign = 'center';
    g.fillText(texte, largeur / 2, hauteur / 2 + 1);
    e.matSprite.map?.dispose();
    const texture = new THREE.CanvasTexture(canvas);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.generateMipmaps = false;
    texture.minFilter = THREE.LinearFilter;
    e.matSprite.map = texture;
    e.matSprite.needsUpdate = true;
    e.pixels = [largeur / echelle, hauteur / echelle];
  }

  /** Sens du dégradé des prolongements : opaque du côté raccordé à un mur vu. */
  _raccorderExtrapoles() {
    const bouts = [];
    for (const e of this.couches.mur.occupants) {
      if (e.sortie !== null) continue;
      const dx = Math.cos(e.c.cap) * e.c.l / 2, dy = Math.sin(e.c.cap) * e.c.l / 2;
      bouts.push(e.c.x - dx, e.c.y - dy, e.c.x + dx, e.c.y + dy);
    }
    for (const e of this.couches.murExtrapole.occupants) {
      const dx = Math.cos(e.c.cap) * e.c.l / 2, dy = Math.sin(e.c.cap) * e.c.l / 2;
      const ax = e.c.x - dx, ay = e.c.y - dy, bx = e.c.x + dx, by = e.c.y + dy;
      let da = Infinity, db = Infinity;
      for (let k = 0; k < bouts.length; k += 2) {
        da = Math.min(da, Math.hypot(bouts[k] - ax, bouts[k + 1] - ay));
        db = Math.min(db, Math.hypot(bouts[k] - bx, bouts[k + 1] - by));
      }
      const seuil = 0.5;
      e.degrade = Math.min(da, db) > seuil ? 0 : (da <= db ? 1 : -1);
    }
  }

  /** Les étiquettes flottent au-dessus de l'objet le plus haut posé dessous. */
  _hausserEtiquettes() {
    for (const e of this.etiquettes.values()) {
      let h = 0;
      for (const o of this.entrees.values()) {
        if (o.sortie !== null || o.extrap) continue;
        const rayon = o.type === 'arbre' || o.vivant ? Math.max(0.3, o.c.r * 2) : Math.max(o.c.l, o.c.w) / 2 + 0.15;
        if (Math.hypot(o.c.x - e.c.x, o.c.y - e.c.y) < rayon) h = Math.max(h, o.type === 'arbre' ? Math.min(o.c.h, 1.2) : o.c.h);
      }
      e.z = Math.max(e.court ? 0.3 : 0.7, h + (e.court ? 0.15 : 0.3));
    }
  }

  _liberer(e) {
    const couche = this.couches[e.nomCouche];
    couche.retirer(e.i, 'i');
    if (e.iOmbre >= 0) this.couches.ombre.retirer(e.iOmbre, 'iOmbre');
    e.i = e.iOmbre = -1;
  }

  _alphaApparition(e, t) {
    return smoothstep(0, APPARITION_S, t - e.naissance);
  }

  /** Tout effacer (changement de repère, déconnexion). */
  vider() {
    for (const e of this.entrees.values()) this._liberer(e);
    this.entrees.clear();
    for (const e of this.etiquettes.values()) this._detruireEtiquette(e);
    this.etiquettes.clear();
  }

  // ---------------------------------------------------------------- animation

  /**
   * Une image : `t` (s, horloge locale), `dt` (s), `vue` = { p11, hauteurPx, cx, cy } pour la
   * taille constante des étiquettes et leur fondu au loin.
   */
  animer(t, dt, vue) {
    const k = 1 - Math.exp(-dt / LISSAGE_S);
    const kDanger = Math.min(1, dt / 0.25);
    const ombres = this.couches.ombre;
    let retirer = null;
    for (const e of this.entrees.values()) {
      if (e.bouge) {
        const v = e.v, c = e.c;
        v.x = lisser(v.x, c.x, k);
        v.y = lisser(v.y, c.y, k);
        v.cap = lisserAngle(v.cap, c.cap, k);
        v.l = lisser(v.l, c.l, k);
        v.w = lisser(v.w, c.w, k);
        v.h = lisser(v.h, c.h, k);
        v.r = lisser(v.r, c.r, k);
        const ecart = Math.abs(v.x - c.x) + Math.abs(v.y - c.y) + Math.abs(v.l - c.l) + Math.abs(v.w - c.w)
          + Math.abs(v.h - c.h) + Math.abs(v.r - c.r) + Math.abs(Math.sin(v.cap - c.cap));
        if (ecart < 2e-4) {
          Object.assign(v, c);
          e.bouge = false;
        }
        this._ecrireMatrices(e);
        this.couches[e.nomCouche].sale = true;
        if (e.iOmbre >= 0) ombres.sale = true;
      }
      e.dangerV += Math.sign(e.danger - e.dangerV) * Math.min(Math.abs(e.danger - e.dangerV), kDanger);
      const age = e.age + (t - e.recu);
      // Plus de messages (liaison coupée) : l'objet expire de lui-même comme sur le rover.
      if (e.sortie === null && age > (e.vivant ? AGE_MAX_VIVANT_S : AGE_MAX_S) && t - e.recu > 0.6) e.sortie = t;
      let alpha = this._alphaApparition(e, t) * (e.vivant ? 1 - 0.6 * smoothstep(0.5, 5, age) : 1 - 0.62 * smoothstep(2, 30, age));
      if (e.sortie !== null) {
        const duree = e.vivant ? DISPARITION_VIVANT_S : DISPARITION_S;
        alpha *= 1 - smoothstep(0, duree, t - e.sortie);
        if (t - e.sortie > duree) (retirer ||= []).push(e);
      }
      const couche = this.couches[e.nomCouche];
      const p = couche.params.array, o = 4 * e.i;
      p[o] = alpha * (e.extrap ? OPACITE_EXTRAPOLE : 1);
      p[o + 1] = e.dangerV;
      p[o + 2] = e.degrade;
      p[o + 3] = e.lum;
      if (e.iOmbre >= 0) {
        const q = this.couches.ombre.params.array, j = 4 * e.iOmbre;
        q[j] = alpha * (e.type === 'arbre' ? 0.5 : 0.85);
        q[j + 1] = e.dangerV;
        q[j + 2] = e.type === 'arbre' || e.vivant ? 1 : 0;
        q[j + 3] = 1;
      }
    }
    if (retirer) {
      for (const e of retirer) {
        this._liberer(e);
        this.entrees.delete(e.cle);
      }
    }
    for (const couche of this.listeCouches) {
      couche.params.needsUpdate = true;
      if (couche.sale) couche.signalerMatrices();
      couche.sale = false;
    }
    this._animerEtiquettes(t, dt, k, vue);
  }

  _ecrireMatrices(e) {
    const v = e.v, couche = this.couches[e.nomCouche], i = e.i;
    let ox, oy, oc = v.cap;
    if (e.type === 'arbre') {
      ecrireMatrice(couche.matrices(0), i, v.x, v.y, 0, 0, v.r, v.r, v.h);
      ox = oy = 2 * v.r + 0.22;
    } else if (e.vivant) {
      ecrireMatrice(couche.matrices(0), i, v.x, v.y, 0, v.cap, v.h, v.h, v.h);
      if (e.type === 'animal') {
        ox = 1.45 * v.h;
        oy = 0.6 * v.h;
      } else {
        ox = oy = Math.max(2 * v.r, 0.32 * v.h) + 0.12;
      }
    } else {
      ecrireMatrice(couche.matrices(0), i, v.x, v.y, 0, v.cap, v.l, v.w, v.h);
      const marge = e.type === 'bas' ? 0.14 : (e.type === 'mur' ? 0.22 : 0.26);
      ox = v.l + marge;
      oy = v.w + marge + (e.type === 'mur' ? 0.12 : 0);
    }
    if (e.iOmbre >= 0) ecrireMatrice(this.couches.ombre.matrices(0), e.iOmbre, v.x, v.y, 0, oc, ox, oy, 1);
  }

  _animerEtiquettes(t, dt, k, vue) {
    let retirer = null;
    for (const e of this.etiquettes.values()) {
      e.v.x = lisser(e.v.x, e.c.x, k);
      e.v.y = lisser(e.v.y, e.c.y, k);
      e.zV = lisser(e.zV, e.z, k);
      const age = e.age + (t - e.recu);
      if (e.sortie === null && age > (e.court ? AGE_MAX_VIVANT_S : AGE_MAX_S) && t - e.recu > 0.6) e.sortie = t;
      let alpha = smoothstep(0, APPARITION_S, t - e.naissance) * (e.court ? 1 - 0.5 * smoothstep(0.5, 5, age) : 1 - 0.5 * smoothstep(5, 30, age));
      alpha *= 1 - smoothstep(14, 24, Math.hypot(e.v.x - vue.cx, e.v.y - vue.cy));
      if (e.sortie !== null) {
        const duree = e.court ? DISPARITION_VIVANT_S : DISPARITION_S;
        alpha *= 1 - smoothstep(0, duree, t - e.sortie);
        if (t - e.sortie > duree) (retirer ||= []).push(e);
      }
      e.groupe.position.set(e.v.x, e.v.y, 0);
      e.tige.scale.set(1, 1, e.zV);
      e.sprite.position.set(0, 0, e.zV);
      if (e.pixels && vue.hauteurPx > 0) {
        const s = 2 / (vue.p11 * vue.hauteurPx);
        e.sprite.scale.set(e.pixels[0] * s, e.pixels[1] * s, 1);
      }
      e.matSprite.opacity = alpha;
      e.matTige.opacity = e.court ? 0 : 0.45 * alpha; // être vivant : texte seul au-dessus de la tête
      e.matPied.opacity = e.court ? 0 : 0.7 * alpha;
      e.groupe.visible = alpha > 0.002;
    }
    if (retirer) {
      for (const e of retirer) {
        this._detruireEtiquette(e);
        this.etiquettes.delete(e.cle);
      }
    }
  }

  _detruireEtiquette(e) {
    this.parent.remove(e.groupe);
    e.matSprite.map?.dispose();
    e.matSprite.dispose();
    e.matTige.dispose();
    e.matPied.dispose();
  }

  compter() {
    const n = {};
    for (const [nom, couche] of Object.entries(this.couches)) n[nom] = couche.occupants.length;
    n.etiquette = this.etiquettes.size;
    return n;
  }

  detruire() {
    this.vider();
    for (const couche of Object.values(this.couches)) couche.detruire();
    this.geoTige.dispose();
    this.geoPied.dispose();
  }
}
