// Vue 3D : décor, rover animé, points du lidar, chemin prévu, caméras et caméra embarquée.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { construireCiel, construireForet, construireSolSeul, etiquette } from './foret3d.js';
import { Rover3D } from './rover3d.js';
import { solVers3d, orientationVersQuaternion3d } from './repere.js';

const BRUME = 0xd9d6c0;
/** Direction du soleil (repère three.js) : bas sur l'horizon, lumière de fin d'après-midi. */
const DIRECTION_SOLEIL = new THREE.Vector3(0.55, 0.5, 0.45).normalize();

export class Scene3D {
  constructor(canvas, cadreCamera) {
    this.canvas = canvas;
    this.cadreCamera = cadreCamera;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.toneMapping = THREE.NeutralToneMapping;
    this.renderer.toneMappingExposure = 1.0;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(BRUME);
    this.scene.fog = new THREE.Fog(BRUME, 28, 160);
    this.scene.add(new THREE.HemisphereLight(0xd6e6f2, 0x6b5a3a, 1.15));
    this.soleil = new THREE.DirectionalLight(0xffe0b3, 2.6);
    this.soleil.castShadow = true;
    this.soleil.shadow.mapSize.set(2048, 2048);
    Object.assign(this.soleil.shadow.camera, { left: -18, right: 18, top: 18, bottom: -18, near: 1, far: 90 });
    this.soleil.shadow.bias = -0.0004;
    this.soleil.shadow.normalBias = 0.02;
    this.scene.add(this.soleil, this.soleil.target);
    this.ciel = construireCiel(this.scene, DIRECTION_SOLEIL);

    this.camera = new THREE.PerspectiveCamera(55, 1, 0.05, 700);
    this.cameraEmbarquee = new THREE.PerspectiveCamera(95, 16 / 9, 0.05, 700);
    this.controles = new OrbitControls(this.camera, canvas);
    this.controles.enableDamping = true;
    this.controles.maxPolarAngle = Math.PI * 0.48;
    this.controles.enabled = false;
    this.modeCamera = 'poursuite';

    this.decor = new THREE.Group();
    this.scene.add(this.decor);
    this.rover = null;
    this.vue = null;
    this.suiviPrecedent = null;

    // Points du dernier balayage lidar.
    this.geomLidar = new THREE.BufferGeometry();
    this.geomLidar.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(360 * 3), 3));
    this.pointsLidar = new THREE.Points(this.geomLidar, new THREE.PointsMaterial({
      color: 0xff5a36, size: 0.07, sizeAttenuation: true, depthWrite: false,
    }));
    this.pointsLidar.frustumCulled = false;
    this.scene.add(this.pointsLidar);

    // Chemin prévu et objectif.
    this.ligneChemin = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color: 0x3987e5, linewidth: 2 }),
    );
    this.ligneChemin.frustumCulled = false;
    this.scene.add(this.ligneChemin);
    this.objectif = new THREE.Group();
    const anneau = new THREE.Mesh(
      new THREE.RingGeometry(0.35, 0.5, 40),
      new THREE.MeshBasicMaterial({ color: 0x3987e5, side: THREE.DoubleSide, transparent: true, opacity: 0.9 }),
    );
    anneau.rotation.x = -Math.PI / 2;
    const faisceau = new THREE.Mesh(
      new THREE.CylinderGeometry(0.04, 0.04, 3, 8),
      new THREE.MeshBasicMaterial({ color: 0x3987e5, transparent: true, opacity: 0.5 }),
    );
    faisceau.position.y = 1.5;
    this.objectif.add(anneau, faisceau);
    this.objectif.visible = false;
    this.scene.add(this.objectif);

    // Obstacles bas vus par la caméra (couche vision du cerveau).
    this.obstaclesVision = new THREE.Group();
    this.scene.add(this.obstaclesVision);
    this.materiauVision = new THREE.MeshBasicMaterial({
      color: 0xf0a020, transparent: true, opacity: 0.5, depthWrite: false, side: THREE.DoubleSide,
      polygonOffset: true, polygonOffsetFactor: -4, polygonOffsetUnits: -4,
    });
  }

  afficherObstaclesVision(obstacles) {
    for (const enfant of this.obstaclesVision.children) {
      enfant.geometry?.dispose();
      enfant.material?.map?.dispose();
    }
    this.obstaclesVision.clear();
    for (const o of obstacles) {
      // Emprise au sol (repère du protocole) → forme plane couchée sur le sol.
      const forme = new THREE.Shape(o.contour.map(([x, y]) => new THREE.Vector2(x, y)));
      const zone = new THREE.Mesh(new THREE.ShapeGeometry(forme), this.materiauVision);
      zone.rotation.x = -Math.PI / 2; // (x, y) du sol → (X, -Z) de three.js
      zone.position.y = this.relief(o.contour[0][0], o.contour[0][1]) + 0.08;
      zone.renderOrder = 5;
      const cx = o.contour.reduce((a, p) => a + p[0], 0) / o.contour.length;
      const cy = o.contour.reduce((a, p) => a + p[1], 0) / o.contour.length;
      const texte = etiquette(`${o.nom} · vu par la caméra`);
      texte.position.set(...solVers3d(cx, cy, 0.9));
      this.obstaclesVision.add(zone, texte);
    }
  }

  initialiser(bienvenue) {
    this.decor.clear();
    this.troncs = bienvenue.monde ? [...bienvenue.monde.arbres, ...(bienvenue.monde.lisiere ?? [])] : [];
    this.relief = () => 0;
    if (bienvenue.monde) {
      this.relief = construireForet(this.decor, bienvenue.monde, bienvenue.scenario.lieux);
    } else {
      construireSolSeul(this.decor, bienvenue.scenario.limites, bienvenue.scenario.lieux);
    }
    if (!this.rover) {
      this.rover = new Rover3D(this.scene, bienvenue.rover.geometrie);
      this.rover.charger('modeles/rover_v3.glb', 'modeles/rover_v3.rig.json')
        .catch((erreur) => console.warn('Modèle du rover indisponible, rover simplifié', erreur));
    }
    this.lidarPosition = bienvenue.rover.capteurs.lidar.position;
    const d = bienvenue.scenario.depart;
    const [X, , Z] = solVers3d(d.x, d.y);
    this.camera.position.set(X - 4 * Math.cos(d.cap), 2.2, Z + 4 * Math.sin(d.cap));
  }

  definirCamera(mode) {
    this.modeCamera = mode;
    this.controles.enabled = mode === 'orbite';
    if (mode === 'orbite' && this.vue) {
      this.controles.target.set(...solVers3d(this.vue.x, this.vue.y, this.vue.z + 0.2));
    }
  }

  mettreAJourVue(vue) {
    this.vue = vue;
    this.rover?.mettreAJour(vue);
  }

  mettreAJourScan(scan) {
    if (!this.vue) return;
    const { x, y, z, cap } = this.vue;
    const [lx, ly, lz] = this.lidarPosition;
    const c = Math.cos(cap), s = Math.sin(cap);
    const ox = x + c * lx - s * ly, oy = y + s * lx + c * ly;
    const hauteur = z - 0.1 + lz;
    const tableau = this.geomLidar.attributes.position.array;
    let n = 0;
    scan.distances.forEach((d, i) => {
      if (d === null || n >= 360) return;
      const a = cap + scan.angle_min + i * scan.pas;
      const [X, Y, Z] = solVers3d(ox + d * Math.cos(a), oy + d * Math.sin(a), hauteur);
      tableau[3 * n] = X; tableau[3 * n + 1] = Y; tableau[3 * n + 2] = Z;
      n++;
    });
    this.geomLidar.setDrawRange(0, n);
    this.geomLidar.attributes.position.needsUpdate = true;
  }

  afficherChemin(points, objectif) {
    const sommets = points.map(([x, y]) => new THREE.Vector3(...solVers3d(x, y, this.relief(x, y) + 0.06)));
    this.ligneChemin.geometry.dispose();
    this.ligneChemin.geometry = new THREE.BufferGeometry().setFromPoints(sommets);
    this.objectif.visible = Boolean(objectif);
    if (objectif) {
      this.objectif.position.set(...solVers3d(objectif[0], objectif[1], this.relief(objectif[0], objectif[1]) + 0.02));
    }
  }

  /** Distance de recul de la caméra sans traverser un tronc (repère au sol). */
  _reculLibre(x, y, direction, souhaite) {
    const dx = Math.cos(direction), dy = Math.sin(direction);
    let recul = souhaite;
    for (const [tx, ty, r] of this.troncs ?? []) {
      const rx = tx - x, ry = ty - y;
      if (Math.abs(rx) > souhaite + 1 || Math.abs(ry) > souhaite + 1) continue;
      const le_long = rx * dx + ry * dy;
      const travers = Math.abs(-rx * dy + ry * dx);
      if (le_long > 0 && travers < r + 0.35) recul = Math.min(recul, Math.max(0.9, le_long - r - 0.35));
    }
    return recul;
  }

  _placerCameras(dt) {
    const v = this.vue;
    if (!v) return;
    const [X, , Z] = solVers3d(v.x, v.y);
    const sol = v.z - 0.1;
    const cible = new THREE.Vector3(X, sol + 0.35, Z);
    if (this.modeCamera === 'poursuite') {
      const recul = this._reculLibre(v.x, v.y, v.cap + Math.PI, 2.4);
      const xc = v.x - recul * Math.cos(v.cap), yc = v.y - recul * Math.sin(v.cap);
      const hauteur = Math.max(sol + 1.25, (this.relief?.(xc, yc) ?? 0) + 0.8);
      const souhaitee = new THREE.Vector3(X - recul * Math.cos(v.cap), hauteur, Z + recul * Math.sin(v.cap));
      const k = 1 - Math.exp(-dt * 3);
      this.camera.position.lerp(souhaitee, k);
      this.camera.lookAt(cible);
    } else if (this.modeCamera === 'dessus') {
      const souhaitee = new THREE.Vector3(X, sol + 22, Z + 0.01);
      this.camera.position.lerp(souhaitee, 1 - Math.exp(-dt * 4));
      this.camera.lookAt(cible);
    } else {
      // Orbite : la cible suit le rover, la souris tourne autour.
      if (this.suiviPrecedent) {
        const delta = cible.clone().sub(this.suiviPrecedent);
        this.camera.position.add(delta);
        this.controles.target.add(delta);
      }
      this.controles.update();
    }
    this.suiviPrecedent = cible;

    // Caméra embarquée sur la tête capteurs (vue avant grand angle).
    const [qx, qy, qz, qw] = orientationVersQuaternion3d(v.roulis, v.tangage, v.cap);
    const q = new THREE.Quaternion(qx, qy, qz, qw);
    const avant = new THREE.Vector3(0.36, 0.28 - 0.1, 0).applyQuaternion(q);
    this.cameraEmbarquee.position.set(X + avant.x, v.z + avant.y, Z + avant.z);
    // Une caméra three.js regarde vers -Z ; on la tourne pour qu'elle regarde vers +X du rover.
    this.cameraEmbarquee.quaternion.copy(q).multiply(
      new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -Math.PI / 2),
    ).multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -0.08));

    // L'ombre portée suit le rover ; le ciel suit la caméra.
    this.soleil.position.copy(cible).addScaledVector(DIRECTION_SOLEIL, 40);
    this.soleil.target.position.copy(cible);
    this.ciel.position.set(this.camera.position.x, 0, this.camera.position.z);
  }

  rendre(dt) {
    const { clientWidth: w, clientHeight: h } = this.canvas;
    if (this.canvas.width !== Math.floor(w * this.renderer.getPixelRatio())
      || this.canvas.height !== Math.floor(h * this.renderer.getPixelRatio())) {
      this.renderer.setSize(w, h, false);
      this.camera.aspect = w / h;
      this.camera.updateProjectionMatrix();
    }
    this._placerCameras(dt);
    this.renderer.setScissorTest(false);
    this.renderer.setViewport(0, 0, w, h);
    this.renderer.render(this.scene, this.camera);

    // Caméra embarquée, rendue dans le cadre prévu en bas à droite.
    const cadre = this.cadreCamera.getBoundingClientRect();
    const zone = this.canvas.getBoundingClientRect();
    if (cadre.width > 0 && this.vue && this.rendrePip !== false) {
      const x = cadre.left - zone.left, y = zone.bottom - cadre.bottom;
      this.pointsLidar.visible = false;
      this.ligneChemin.visible = false;
      this.obstaclesVision.visible = false;
      this.renderer.setScissorTest(true);
      this.renderer.setScissor(x, y, cadre.width, cadre.height);
      this.renderer.setViewport(x, y, cadre.width, cadre.height);
      this.renderer.render(this.scene, this.cameraEmbarquee);
      this.renderer.setScissorTest(false);
      this.pointsLidar.visible = this.afficherLidar ?? true;
      this.ligneChemin.visible = this.afficherLeChemin ?? true;
      this.obstaclesVision.visible = true;
    }
  }
}
