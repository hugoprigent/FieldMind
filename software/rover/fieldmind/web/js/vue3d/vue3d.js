// Vue 3D « façon Tesla » du poste opérateur : le rover au centre d'un monde reconstruit,
// sobre, en niveaux de gris, à partir des messages du rover (PROTOCOLE.md).
//
// Utilisation (modules ES, aucune dépendance hors de web/vendor/three/) :
//
//   import { Vue3D } from './vue3d/vue3d.js';
//   const vue = new Vue3D(conteneur, { profil });  // profil = bienvenue.profil (géométrie)
//   vue.scene(message);       // message `scene` (4 Hz) : murs, arbres, blocs, bas, personnes,
//                             //   animaux, étiquettes, libre, chemin, but — repère `odom`
//                             //   (ou `base_link`)
//   vue.etat(message);        // message `etat` (10 Hz) : pose odom, vitesse, braquage
//                             //   (commande puis consigne), sécurité, armement, lidar en marche
//   vue.scan(message);        // message `scan` (points base_link en cm), facultatif
//   vue.obstacles(message);   // message `obstacles` (stéréo, base_link en cm), facultatif
//   vue.camera('suivie' | 'dessus' | 'libre');
//   vue.activer(true | false);  // lance / arrête le rendu (vue cachée : aucun coût)
//   vue.detruire();
//
// Compléments : `vue.profil(profil)` reconstruit le rover si `bienvenue` arrive après la
// création ; `vue.poussiere(true)` affiche les points du lidar (désactivés par défaut) ;
// `vue.statistiques()` rend { ips, dpr, appels, triangles, objets } ; `vue.vider()` efface
// le monde (après une remise à zéro de l'odométrie, par exemple).
//
// Le conteneur doit avoir une taille (la vue le remplit et suit ses changements de taille) ;
// s'il est en `position: static`, il passe en `position: relative`. La vue est créée inactive :
// appeler `activer(true)` quand elle devient visible. Les messages peuvent être passés même
// quand elle est inactive (seules les cibles sont mises à jour, sans dessin).
//
// Repères : le monde three.js est celui du message `scene` (x avant au départ, y à gauche,
// z en haut), en mètres. Le rover est placé à la pose `etat.odom` (prédite entre deux
// messages avec sa vitesse et sa vitesse de lacet, puis lissée), ou à `scene.rover` si
// l'odométrie manque ; en repère `base_link`, il reste à l'origine et le monde défile.
// La trajectoire prévue est rouge quand la sécurité arrête un rover armé (obstacle,
// capteur…), estompée quand le rover est désarmé ou en mode arrêt.

import * as THREE from '../../vendor/three/three.module.js';
import { OrbitControls } from '../../vendor/three/addons/controls/OrbitControls.js';
import { PALETTE, creerUniformes } from './materiaux.js';
import { RoverModele, dimensions } from './rover.js';
import { RemorqueModele } from './remorque.js';
import { Sol } from './sol.js';
import { Objets } from './objets.js';
import { Chemin, But, Trajectoire, Points } from './traces.js';

const MODES = new Set(['suivie', 'dessus', 'libre']);
const DPR_MAX = 2;
const RAISONS_REPOS = new Set(['non_arme', 'mode_arret', 'remorque']);

function fini(...valeurs) {
  return valeurs.every((v) => Number.isFinite(v));
}

function amortir(dt, tau) {
  return 1 - Math.exp(-dt / tau);
}

function ecartAngle(a, b) {
  return Math.atan2(Math.sin(b - a), Math.cos(b - a));
}

export class Vue3D {
  constructor(conteneur, { profil = null, camera = 'suivie', poussiere = false } = {}) {
    this.conteneur = conteneur;
    if (getComputedStyle(conteneur).position === 'static') conteneur.style.position = 'relative';

    this.racine = document.createElement('div');
    this.racine.className = 'vue3d';
    Object.assign(this.racine.style, {
      position: 'absolute', inset: '0', overflow: 'hidden',
      background: `radial-gradient(130% 75% at 50% 0%, ${PALETTE.fondHaut} 0%, ${PALETTE.fondMilieu} 55%, ${PALETTE.fondBas} 100%)`,
    });
    this.canvas = document.createElement('canvas');
    Object.assign(this.canvas.style, { display: 'block', width: '100%', height: '100%', outline: 'none' });
    this.racine.appendChild(this.canvas);
    conteneur.appendChild(this.racine);

    this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, alpha: true, powerPreference: 'high-performance' });
    this.dprMax = Math.min(window.devicePixelRatio || 1, DPR_MAX);
    this.dpr = this.dprMax;
    this.renderer.setPixelRatio(this.dpr);
    this.renderer.setClearColor(0x000000, 0);

    this.scene3d = new THREE.Scene();
    this.uniformes = creerUniformes();
    const ciel = new THREE.HemisphereLight(0xdfe5ec, 0x2c3037, 1.9);
    ciel.position.set(0, 0, 1);
    const soleil = new THREE.DirectionalLight(0xffffff, 1.45);
    soleil.position.set(-4, 6, 10);
    this.scene3d.add(ciel, soleil);

    this.sol = new Sol(this.uniformes);
    this.sol.ajouterA(this.scene3d);
    this.objets = new Objets(this.scene3d, this.uniformes);
    this.chemin = new Chemin(this.uniformes);
    this.but = new But(this.uniformes);
    this.scene3d.add(this.chemin.maillage, this.but.groupe);

    this.rover = new RoverModele(profil || {});
    this.d = dimensions(profil || {});
    this.scene3d.add(this.rover.groupe);
    this.trajectoire = new Trajectoire();
    this.rover.groupe.add(this.trajectoire.maillage);
    this.lidar = new Points({ couleur: PALETTE.lidarPoints, taille: 2, opacite: 0.55, z: this.d.lidarZ, ordre: -4 });
    this.lidar.objet.visible = Boolean(poussiere);
    this.stereo = new Points({ couleur: PALETTE.stereoPoints, taille: 3, opacite: 0.7, z: 0.03, ordre: -4 });
    this.rover.groupe.add(this.lidar.objet, this.stereo.objet);
    this.remorque = new RemorqueModele();
    this.rover.groupe.add(this.remorque.groupe);

    // Caméra : monde z en haut.
    this.cam = new THREE.PerspectiveCamera(50, 1, 0.05, 140);
    this.cam.up.set(0, 0, 1);
    this.controles = new OrbitControls(this.cam, this.canvas);
    this.controles.enabled = false;
    this.controles.enableDamping = true;
    this.controles.dampingFactor = 0.12;
    this.controles.minDistance = 0.8;
    this.controles.maxDistance = 40;
    this.controles.maxPolarAngle = Math.PI * 0.49;
    this.mode = MODES.has(camera) ? camera : 'suivie';
    this.camPos = new THREE.Vector3();
    this.camVise = new THREE.Vector3();
    this.camHaut = new THREE.Vector3(0, 0, 1);
    this.voulu = { pos: new THREE.Vector3(), vise: new THREE.Vector3(), haut: new THREE.Vector3() };
    this.capCamera = 0;
    this.camPlacee = false;

    // État reçu.
    this.repere = 'odom';
    this.odom = null; // { x, y, cap, v, w, t }
    this.poseScene = null; // { x, y, cap, t }
    this.pose = { x: 0, y: 0, cap: 0 };
    this.poseInitialisee = false;
    this.roverPrecedent = new THREE.Vector3();
    this.commande = { vitesse: 0, braquage: 0 };
    this.vitesseMesuree = 0;
    this.lisse = { vitesse: 0, braquage: 0, vitesseArc: 0 };
    this.arretSecurite = false;
    this.roverActif = false;
    this.lidarMarche = true;

    // Boucle.
    this.actif = false;
    this.raf = 0;
    this.tPrecedent = 0;
    this.mesure = { images: 0, debut: 0, ips: 0, somme: 0, n: 0, attente: 2 };
    this._image = this._image.bind(this);
    this.taille = { l: 0, h: 0 };
    this.observateur = new ResizeObserver(() => this._redimensionner());
    this.observateur.observe(this.racine);
    this._redimensionner();
    this.camera(this.mode);
  }

  // ---------------------------------------------------------------- API

  scene(message) {
    if (!message || typeof message !== 'object') return;
    const t = this._maintenant();
    const repere = message.repere === 'base_link' ? 'base_link' : 'odom';
    if (repere !== this.repere) {
      this.objets.vider();
      this.repere = repere;
    }
    const r = message.rover;
    if (r && fini(r.x, r.y)) this.poseScene = { x: r.x, y: r.y, cap: Number.isFinite(r.cap) ? r.cap : 0, t };
    this.objets.recevoir(message, t);
    this.sol.libre.definir(message.libre);
    this.chemin.definir(message.chemin);
    this.but.definir(message.but);
  }

  etat(message) {
    if (!message || typeof message !== 'object') return;
    const t = this._maintenant();
    const o = message.odom;
    this.odom = o && fini(o.x, o.y, o.cap)
      ? { x: o.x, y: o.y, cap: o.cap, v: Number.isFinite(o.vitesse) ? o.vitesse : 0, w: Number.isFinite(o.vitesse_lacet) ? o.vitesse_lacet : 0, t }
      : null;
    const c = message.commande || {}, k = message.consigne || {};
    const premier = (...v) => v.find((x) => Number.isFinite(x)) ?? 0;
    this.commande.vitesse = premier(c.vitesse, k.vitesse, o?.vitesse);
    this.commande.braquage = premier(c.braquage, k.braquage);
    this.vitesseMesuree = premier(o?.vitesse, message.simulation?.vitesse, c.vitesse);
    const s = message.securite || {};
    const raisons = Array.isArray(s.raisons) ? s.raisons : [];
    this.roverActif = message.arme === true && message.mode !== 'arret' && !message.arret_urgence;
    this.arretSecurite = Boolean(message.arret_urgence)
      || (s.arret === true && (this.roverActif || raisons.some((x) => !RAISONS_REPOS.has(x))));
    this.lidarMarche = message.capteurs?.lidar?.marche !== false;
    this.remorque.definir(message.remorque || null);
  }

  scan(message) {
    this.lidar.definir(message?.xy, this._maintenant());
  }

  obstacles(message) {
    this.stereo.definir(message?.xy, this._maintenant());
  }

  camera(mode) {
    if (!MODES.has(mode)) return this.mode;
    const ancien = this.mode;
    this.mode = mode;
    if (mode === 'libre') {
      this.cam.up.set(0, 0, 1);
      this.camHaut.set(0, 0, 1);
      if (this.camPlacee) {
        this.cam.position.copy(this.camPos);
        this.controles.target.copy(this.camVise);
      }
      this.controles.enabled = true;
      this.controles.update();
    } else if (ancien === 'libre') {
      this.controles.enabled = false;
      this.camPos.copy(this.cam.position);
      this.camVise.copy(this.controles.target);
      this.camHaut.copy(this.cam.up);
    }
    this.canvas.style.touchAction = mode === 'libre' ? 'none' : 'auto';
    this.canvas.style.cursor = mode === 'libre' ? 'grab' : 'default';
    return this.mode;
  }

  activer(actif) {
    actif = Boolean(actif);
    if (actif === this.actif) return;
    this.actif = actif;
    if (actif) {
      this.tPrecedent = 0;
      this.mesure.attente = 3;
      this._redimensionner();
      this.raf = requestAnimationFrame(this._image);
    } else {
      cancelAnimationFrame(this.raf);
    }
  }

  profil(profil) {
    this.rover.construire(profil || {});
    this.d = dimensions(profil || {});
    this.rover.groupe.add(this.trajectoire.maillage, this.lidar.objet, this.stereo.objet,
      this.remorque.groupe);
    this.lidar.z = this.d.lidarZ;
  }

  poussiere(visible) {
    this.lidar.objet.visible = Boolean(visible);
  }

  vider() {
    this.objets.vider();
    this.sol.libre.definir(null);
    this.chemin.definir(null);
    this.but.definir(null);
  }

  statistiques() {
    const info = this.renderer.info.render;
    return {
      ips: Math.round(this.mesure.ips * 10) / 10, dpr: this.dpr, appels: info.calls, triangles: info.triangles,
      objets: this.objets.compter(), taille: [this.taille.l, this.taille.h],
    };
  }

  detruire() {
    this.activer(false);
    this.observateur.disconnect();
    this.controles.dispose();
    this.objets.detruire();
    this.sol.detruire();
    this.chemin.detruire();
    this.but.detruire();
    this.trajectoire.detruire();
    this.lidar.detruire();
    this.stereo.detruire();
    this.rover.detruire();
    this.remorque.detruire();
    this.renderer.dispose();
    this.racine.remove();
  }

  // ---------------------------------------------------------------- interne

  _maintenant() {
    return performance.now() / 1000;
  }

  _redimensionner() {
    const l = Math.max(1, Math.round(this.racine.clientWidth)), h = Math.max(1, Math.round(this.racine.clientHeight));
    if (l === this.taille.l && h === this.taille.h) return;
    this.taille.l = l;
    this.taille.h = h;
    this.renderer.setSize(l, h, false);
    const aspect = l / h;
    this.cam.aspect = aspect;
    // En portrait, champ vertical plus ouvert pour garder une largeur utile.
    this.cam.fov = aspect >= 1 ? 46 : Math.min(70, (2 * Math.atan(Math.tan((46 * Math.PI) / 360) / Math.sqrt(aspect)) * 180) / Math.PI);
    this.cam.updateProjectionMatrix();
    if (this.actif) this._rendre();
  }

  /** Pose voulue du rover : odométrie prédite, sinon pose de la scène, sinon origine. */
  _poseCible(t, sortie) {
    if (this.repere === 'base_link') {
      sortie.x = sortie.y = sortie.cap = 0;
      return true;
    }
    const o = this.odom;
    if (o) {
      const dt = Math.min(0.4, Math.max(0, t - o.t));
      const cap = o.cap + o.w * dt;
      const capMoyen = o.cap + (o.w * dt) / 2;
      sortie.x = o.x + o.v * dt * Math.cos(capMoyen);
      sortie.y = o.y + o.v * dt * Math.sin(capMoyen);
      sortie.cap = cap;
      return true;
    }
    if (this.poseScene) {
      sortie.x = this.poseScene.x;
      sortie.y = this.poseScene.y;
      sortie.cap = this.poseScene.cap;
      return true;
    }
    return false;
  }

  _image(tMs) {
    if (!this.actif) return;
    this.raf = requestAnimationFrame(this._image);
    const t = tMs / 1000;
    const dt = this.tPrecedent ? Math.min(0.1, Math.max(0.0005, t - this.tPrecedent)) : 1 / 60;
    this.tPrecedent = t;
    this._mesurer(t, dt);
    this._animer(this._maintenant(), dt);
    this._rendre();
  }

  _animer(t, dt) {
    // Pose du rover.
    const cible = this._cible || (this._cible = { x: 0, y: 0, cap: 0 });
    if (this._poseCible(t, cible)) {
      const p = this.pose;
      const saut = !this.poseInitialisee || Math.hypot(cible.x - p.x, cible.y - p.y) > 2.5;
      if (saut) {
        p.x = cible.x; p.y = cible.y; p.cap = cible.cap;
        this.poseInitialisee = true;
        this.camPlacee = false;
      } else {
        const k = amortir(dt, 0.09);
        p.x += (cible.x - p.x) * k;
        p.y += (cible.y - p.y) * k;
        p.cap += ecartAngle(p.cap, cible.cap) * k;
      }
    }
    const p = this.pose;
    const g = this.rover.groupe;
    g.position.set(p.x, p.y, 0);
    g.rotation.z = p.cap;

    const kc = amortir(dt, 0.12);
    const l = this.lisse;
    l.vitesse += (this.vitesseMesuree - l.vitesse) * kc;
    l.braquage += (this.commande.braquage - l.braquage) * kc;
    l.vitesseArc += (this.commande.vitesse - l.vitesseArc) * kc;
    this.rover.animer(dt, l.vitesse, l.braquage, this.lidarMarche);
    this.remorque.animer(dt, l.vitesse);
    const arc = this._arc || (this._arc = { vitesse: 0, braquage: 0, arret: false, actif: false });
    arc.vitesse = l.vitesseArc;
    arc.braquage = l.braquage;
    arc.arret = this.arretSecurite;
    arc.actif = this.roverActif;
    this.trajectoire.mettreAJour(dt, arc, this.d);

    // Uniformes partagés et monde.
    this.uniformes.uTemps.value = t;
    this.uniformes.uCentre.value.set(p.x, p.y);
    this.uniformes.uCamera.value.copy(this.cam.position);
    this.sol.suivre(p.x, p.y);
    this.sol.animer(dt);
    this.chemin.animer(dt);
    this.but.animer(dt);
    this.lidar.animer(t);
    this.stereo.animer(t);
    this._placerCamera(dt);
    const vue = this._vue || (this._vue = { p11: 1, hauteurPx: 1, cx: 0, cy: 0 });
    vue.p11 = this.cam.projectionMatrix.elements[5];
    vue.hauteurPx = this.taille.h;
    vue.cx = p.x;
    vue.cy = p.y;
    this.objets.animer(t, dt, vue);
  }

  _placerCamera(dt) {
    const p = this.pose, v = Math.abs(this.lisse.vitesse);
    const portrait = this.taille.l < this.taille.h;
    if (!this.camPlacee) this.capCamera = p.cap;
    else this.capCamera += ecartAngle(this.capCamera, p.cap) * amortir(dt, 0.55);
    const cx = Math.cos(this.capCamera), cy = Math.sin(this.capCamera);
    const voulu = this.voulu;

    if (this.mode === 'libre') {
      // L'orbite suit le rover : caméra et cible translatées de son déplacement.
      const dx = p.x - this.roverPrecedent.x, dy = p.y - this.roverPrecedent.y;
      if (!this.camPlacee) {
        this._calculerSuivie(cx, cy, v, portrait, voulu);
        this.cam.position.copy(voulu.pos);
        this.controles.target.copy(voulu.vise);
        this.camPlacee = true;
      } else {
        this.cam.position.x += dx; this.cam.position.y += dy;
        this.controles.target.x += dx; this.controles.target.y += dy;
      }
      this.controles.update(dt);
      this.camPos.copy(this.cam.position);
      this.camVise.copy(this.controles.target);
    } else {
      if (this.mode === 'dessus') {
        const h = (portrait ? 9.5 : 7.5) * (1 + 0.25 * v);
        const avance = portrait ? 1.6 : 1.1;
        voulu.vise.set(p.x + cx * avance, p.y + cy * avance, 0);
        voulu.pos.set(voulu.vise.x - cx * 0.02, voulu.vise.y - cy * 0.02, h);
        voulu.haut.set(cx, cy, 0);
      } else {
        this._calculerSuivie(cx, cy, v, portrait, voulu);
      }
      if (!this.camPlacee) {
        this.camPos.copy(voulu.pos);
        this.camVise.copy(voulu.vise);
        this.camHaut.copy(voulu.haut);
        this.camPlacee = true;
      } else {
        this.camPos.lerp(voulu.pos, amortir(dt, 0.28));
        this.camVise.lerp(voulu.vise, amortir(dt, 0.16));
        this.camHaut.lerp(voulu.haut, amortir(dt, 0.3)).normalize();
      }
      this.cam.position.copy(this.camPos);
      this.cam.up.copy(this.camHaut);
      this.cam.lookAt(this.camVise);
    }
    this.roverPrecedent.set(p.x, p.y, 0);
  }

  /** Derrière et au-dessus du rover ; recule et monte un peu avec la vitesse. */
  _calculerSuivie(cx, cy, v, portrait, voulu) {
    const p = this.pose;
    const distance = (2.7 + 1.2 * v) * (portrait ? 1.3 : 1);
    const hauteur = (1.75 + 0.5 * v) * (portrait ? 1.75 : 1);
    const avance = (1.2 + 0.8 * v) * (portrait ? 2.3 : 1);
    voulu.pos.set(p.x - cx * distance, p.y - cy * distance, hauteur);
    voulu.vise.set(p.x + cx * avance, p.y + cy * avance, 0.15);
    voulu.haut.set(0, 0, 1);
  }

  _rendre() {
    this.renderer.render(this.scene3d, this.cam);
  }

  /** Images par seconde ; résolution abaissée par paliers si le rendu peine (< 40 i/s). */
  _mesurer(t, dt) {
    const m = this.mesure;
    if (!m.debut) m.debut = t;
    m.images++;
    if (t - m.debut >= 1) {
      m.ips = m.images / (t - m.debut);
      m.images = 0;
      m.debut = t;
      m.lentes = m.ips < 40 ? (m.lentes || 0) + 1 : 0;
      if (m.attente > 0) {
        m.attente--;
        m.lentes = 0;
      } else if (m.lentes >= 2 && this.dpr > 1) {
        this.dpr = Math.max(1, this.dpr - 0.25);
        this.renderer.setPixelRatio(this.dpr);
        this.renderer.setSize(this.taille.l, this.taille.h, false);
        m.attente = 2;
        m.lentes = 0;
      }
    }
  }
}
