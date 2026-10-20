// Modèle 3D du rover V3 (export du rig Blender) animé par l'état de la simulation physique :
// pose du châssis, rotation de chaque roue, braquage de chaque fusée, palonnier et biellettes.

import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import { solVers3d, orientationVersQuaternion3d } from './repere.js';

const AXE_Y = new THREE.Vector3(0, 1, 0);
// Axe +Y de Blender (axe des roues, vers la gauche) = -Z dans three.js après l'export glTF.
const AXE_ROUE = new THREE.Vector3(0, 0, -1);
const AXES_SUIVI = {
  TRACK_Y: new THREE.Vector3(0, 0, -1),
  TRACK_NEGATIVE_Y: new THREE.Vector3(0, 0, 1),
  TRACK_X: new THREE.Vector3(1, 0, 0),
  TRACK_NEGATIVE_X: new THREE.Vector3(-1, 0, 0),
  TRACK_Z: new THREE.Vector3(0, 1, 0),
  TRACK_NEGATIVE_Z: new THREE.Vector3(0, -1, 0),
};

function interpoler(table, abscisses, x) {
  if (x <= abscisses[0]) return table[0];
  const n = abscisses.length - 1;
  if (x >= abscisses[n]) return table[n];
  const pas = abscisses[1] - abscisses[0];
  const i = Math.floor((x - abscisses[0]) / pas);
  const f = (x - abscisses[i]) / pas;
  return table[i] * (1 - f) + table[i + 1] * f;
}

/** Rover de secours (boîtes) si le modèle ne se charge pas. */
function roverSimplifie(geometrie) {
  const groupe = new THREE.Group();
  const caisse = new THREE.Mesh(
    new THREE.BoxGeometry(geometrie.longueur - 0.1, 0.22, geometrie.voie_arriere - 0.12),
    new THREE.MeshStandardMaterial({ color: 0xc0281e }),
  );
  caisse.position.y = 0.18;
  groupe.add(caisse);
  return { groupe, pivots: {} };
}

export class Rover3D {
  constructor(scene, geometrie) {
    this.geometrie = geometrie;
    this.racine = new THREE.Group();
    this.racine.name = 'rover';
    scene.add(this.racine);
    this.pivots = {};
    this.repos = {};
    this.rig = null;
    this.charge = false;
    const secours = roverSimplifie(geometrie);
    this.secours = secours.groupe;
    this.racine.add(this.secours);
  }

  async charger(url, urlRig) {
    const chargeur = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
    const [gltf, rig] = await Promise.all([
      chargeur.loadAsync(url),
      fetch(urlRig).then((r) => (r.ok ? r.json() : null)),
    ]);
    this.rig = rig;
    const modele = gltf.scene;
    modele.traverse((objet) => {
      if (objet.isMesh) {
        objet.castShadow = true;
        objet.receiveShadow = true;
      }
      if (objet.name.startsWith('PIV_')) {
        this.pivots[objet.name] = objet;
        this.repos[objet.name] = objet.quaternion.clone();
      }
    });
    this.racine.remove(this.secours);
    this.racine.add(modele);
    this.charge = true;
  }

  _tourner(nom, axe, angle) {
    const pivot = this.pivots[nom];
    if (!pivot) return;
    pivot.quaternion.copy(this.repos[nom]).multiply(new THREE.Quaternion().setFromAxisAngle(axe, angle));
  }

  /** Applique l'état « vue » de la simulation. */
  mettreAJour(vue) {
    const r = this.geometrie.rayon_roue;
    const [qx, qy, qz, qw] = orientationVersQuaternion3d(vue.roulis, vue.tangage, vue.cap);
    const q = new THREE.Quaternion(qx, qy, qz, qw);
    // Le corps simulé a son origine à hauteur des moyeux ; le modèle, au sol sous le centre.
    const decalage = new THREE.Vector3(0, -r, 0).applyQuaternion(q);
    const [X, Y, Z] = solVers3d(vue.x, vue.y, vue.z);
    this.racine.position.set(X + decalage.x, Y + decalage.y, Z + decalage.z);
    this.racine.quaternion.copy(q);
    if (!this.charge) return;

    const [avG, avD, arG, arD] = vue.angles_roues;
    this._tourner('PIV_Steer_FL', AXE_Y, vue.braquage_gauche);
    this._tourner('PIV_Steer_FR', AXE_Y, vue.braquage_droit);
    this._tourner('PIV_Roll_FL', AXE_ROUE, avG);
    this._tourner('PIV_Roll_FR', AXE_ROUE, avD);
    this._tourner('PIV_Roll_RL', AXE_ROUE, arG);
    this._tourner('PIV_Roll_RR', AXE_ROUE, arD);
    const essieu = (arG + arD) / 2;
    this._tourner('PIV_RearAxle', AXE_ROUE, essieu);
    this._tourner('PIV_MotorShaft', AXE_ROUE, 3 * essieu);
    this._tourner('PIV_Idler', AXE_ROUE, -2.39 * essieu);

    const palonnier = this.rig?.palonnier;
    if (palonnier) {
      const moyenDeg = THREE.MathUtils.radToDeg((vue.braquage_gauche + vue.braquage_droit) / 2);
      const angle = interpoler(palonnier.angle_rad, palonnier.braquage_moyen_deg, moyenDeg);
      this._tourner('PIV_ServoHorn', AXE_Y, angle);
    }
    this.racine.updateMatrixWorld(true);
    for (const biellette of this.rig?.biellettes ?? []) this._orienterBiellette(biellette);
  }

  _orienterBiellette({ noeud, cible, axe_blender: axe }) {
    // Équivalent de la contrainte « Damped Track » du rig : l'axe de la biellette vise sa rotule.
    const tige = this.pivots[noeud], rotule = this.pivots[cible];
    if (!tige || !rotule || !tige.parent) return;
    const cibleLocale = tige.parent.worldToLocal(rotule.getWorldPosition(new THREE.Vector3()));
    const direction = cibleLocale.sub(tige.position).normalize();
    const axeRepos = AXES_SUIVI[axe].clone().applyQuaternion(this.repos[noeud]);
    tige.quaternion.copy(new THREE.Quaternion().setFromUnitVectors(axeRepos, direction))
      .multiply(this.repos[noeud]);
  }
}
