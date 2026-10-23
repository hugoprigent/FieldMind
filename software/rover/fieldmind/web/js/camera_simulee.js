// Véritables images stéréo du monde simulé, rendues en WebGL sur le poste.
// Les dimensions et extrinsèques viennent du serveur ; aucune carte perçue n'est
// réinjectée dans ces images. Le serveur conserve le rendu logiciel en repli.
import * as THREE from '../vendor/three/three.module.js';

function texture(brin = false) {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 512;
  const g = canvas.getContext('2d');
  g.fillStyle = brin ? '#737e43' : '#aaa69e';
  g.fillRect(0, 0, 512, 512);
  let graine = 34291;
  const rnd = () => { graine = Math.imul(graine, 1664525) + 1013904223 | 0; return (graine >>> 0) / 4294967296; };
  for (let i = 0; i < 16000; i++) {
    const x = rnd() * 512, y = rnd() * 512, v = Math.floor(50 + rnd() * 140);
    g.strokeStyle = `rgba(${v},${Math.min(220, v + (brin ? 22 : 0))},${brin ? Math.floor(v * .55) : v},.55)`;
    g.beginPath(); g.moveTo(x, y); g.lineTo(x + rnd() * 5 - 2.5, y - 1 - rnd() * (brin ? 7 : 2)); g.stroke();
  }
  const t = new THREE.CanvasTexture(canvas);
  t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  return t;
}

export function configurerCamera(camera, message, vue) {
  const [x, y, cap] = message.pose, c = Math.cos(cap), s = Math.sin(cap);
  const { centre, rotation: r, k: [fx, fy, cx, cy] } = message.cameras[vue];
  // OpenCV : x à droite, y en bas, z devant. Three : x à droite, y en haut, -z devant.
  const orientation = new THREE.Matrix4().set(r[0][0], -r[0][1], -r[0][2], 0, r[1][0], -r[1][1], -r[1][2], 0, r[2][0], -r[2][1], -r[2][2], 0, 0, 0, 0, 1);
  orientation.premultiply(new THREE.Matrix4().makeRotationZ(cap));
  camera.quaternion.setFromRotationMatrix(orientation);
  camera.position.set(x + c * centre[0] - s * centre[1], y + s * centre[0] + c * centre[1], centre[2]);
  const n = .03, w = message.largeur, h = message.hauteur;
  camera.projectionMatrix.makePerspective(-(cx + .5) * n / fx, (w - cx - .5) * n / fx, (cy + .5) * n / fy, -(h - cy - .5) * n / fy, n, 120);
  camera.projectionMatrixInverse.copy(camera.projectionMatrix).invert();
  camera.updateMatrixWorld(true);
}

export class CamerasSimulees {
  constructor(connexion) {
    this.connexion = connexion;
    this.version = null;
    this.occupe = false;
    this.canvas = document.createElement('canvas');
    this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(1);
    this.renderer.setSize(640, 360, false);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.camera = new THREE.PerspectiveCamera();
    this.grain = texture(false);
    this.herbe = texture(true);
    this.herbe.repeat.set(60, 60);
    this.connexion.envoyer({ type: 'camera_simulee_prete', prete: !document.hidden });
    this.visibilite = () => this.connexion.envoyer({ type: 'camera_simulee_prete', prete: !document.hidden });
    document.addEventListener('visibilitychange', this.visibilite);
    this.canvas.addEventListener('webglcontextlost', e => { e.preventDefault(); this.connexion.envoyer({ type: 'camera_simulee_prete', prete: false }); });
    this.canvas.addEventListener('webglcontextrestored', () => this.visibilite());
  }

  monde(message) {
    if (this.scene) {
      this.scene.traverse(o => { o.geometry?.dispose(); if (o.material) o.material.dispose(); });
    }
    const monde = message.monde;
    this.version = message.version;
    this.scene = new THREE.Scene();
    const couleur = bgr => new THREE.Color((bgr?.[2] ?? 160) / 255, (bgr?.[1] ?? 160) / 255, (bgr?.[0] ?? 160) / 255);
    this.scene.background = couleur(monde.couleurs?.ciel);
    this.scene.fog = new THREE.Fog(this.scene.background, 35, 100);
    this.scene.add(new THREE.HemisphereLight(0xf4f7ff, 0x5b6344, 2));
    const soleil = new THREE.DirectionalLight(0xfff1dc, 2.2);
    soleil.position.set(-5, 8, 15); this.scene.add(soleil);
    const materiau = (c, options = {}) => new THREE.MeshStandardMaterial({ color: couleur(c), roughness: .95, map: this.grain, ...options });
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(120, 120), new THREE.MeshStandardMaterial({ color: monde.scenario === 'jardin' ? 0xb6bd85 : couleur(monde.couleurs?.sol), map: monde.scenario === 'jardin' ? this.herbe : this.grain, roughness: 1 }));
    this.scene.add(floor);
    const ajouter = (geo, mat, x, y, z, cap = 0) => {
      const mesh = new THREE.Mesh(geo, mat); mesh.position.set(x, y, z); mesh.rotation.z = cap; this.scene.add(mesh); return mesh;
    };
    for (const o of monde.obstacles) {
      if (o.type === 'boite') {
        ajouter(new THREE.BoxGeometry(o.longueur, o.largeur, o.hauteur), materiau(o.couleur), o.x, o.y, (o.z0 || 0) + o.hauteur / 2, o.cap || 0);
      } else {
        const geo = new THREE.CylinderGeometry(o.rayon, o.rayon, o.hauteur, 20); geo.rotateX(Math.PI / 2);
        ajouter(geo, materiau(o.couleur), o.x, o.y, (o.z0 || 0) + o.hauteur / 2);
        if (o.genre === 'arbre' && o.hauteur > 1.5) {
          const cime = new THREE.SphereGeometry(1, 12, 10); cime.scale(o.rayon * 4, o.rayon * 4, o.hauteur * .2);
          ajouter(cime, materiau([45, 95, 55]), o.x, o.y, (o.z0 || 0) + o.hauteur * .8);
        }
      }
    }
    if (monde.piece) {
      const p = monde.piece;
      ajouter(new THREE.BoxGeometry(p.xmax - p.xmin, p.ymax - p.ymin, p.hauteur), materiau(p.couleur, { side: THREE.BackSide }), (p.xmin + p.xmax) / 2, (p.ymin + p.ymax) / 2, p.hauteur / 2);
    }
    if (monde.mire) {
      const m = monde.mire, totalX = m.colonnes + 1 + 2 * m.bord_cases, totalY = m.lignes + 1 + 2 * m.bord_cases;
      const groupe = new THREE.Group();
      for (let iy = 0; iy < totalY; iy++) for (let ix = 0; ix < totalX; ix++) {
        const noir = ix >= m.bord_cases && ix < totalX - m.bord_cases && iy >= m.bord_cases && iy < totalY - m.bord_cases && (ix + iy) % 2 === 0;
        const c = new THREE.Mesh(new THREE.PlaneGeometry(m.case_m, m.case_m), new THREE.MeshBasicMaterial({ color: noir ? 0x111111 : 0xffffff, side: THREE.DoubleSide }));
        c.position.set((ix + .5 - totalX / 2) * m.case_m, (iy + .5 - totalY / 2) * m.case_m, 0); groupe.add(c);
      }
      const r = m.rotation; const mat = new THREE.Matrix4().set(r[0][0], r[0][1], r[0][2], m.centre[0], r[1][0], r[1][1], r[1][2], m.centre[1], r[2][0], r[2][1], r[2][2], m.centre[2], 0, 0, 0, 1);
      groupe.applyMatrix4(mat); this.scene.add(groupe);
    }
  }

  async rendre(message) {
    if (!this.scene || message.monde !== this.version || this.occupe || document.hidden) return;
    this.occupe = true;
    try {
      for (const [role, vue] of Object.entries(message.vues)) {
        configurerCamera(this.camera, message, vue);
        this.renderer.render(this.scene, this.camera);
        const jpeg = await new Promise(resolve => this.canvas.toBlob(resolve, 'image/jpeg', message.qualite));
        if (jpeg) this.connexion.envoyerImage({ id: message.id, camera: role }, jpeg);
      }
    } catch (e) {
      console.warn('[simulation cameras]', e);
      this.connexion.envoyer({ type: 'camera_simulee_prete', prete: false });
    } finally { this.occupe = false; }
  }
}
