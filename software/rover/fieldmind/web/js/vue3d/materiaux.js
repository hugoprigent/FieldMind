// Palette et matériaux de la vue 3D : niveaux de gris, accent bleu du poste, rouge critique.
// Les objets du monde sont des maillages instanciés (InstancedMesh) dont chaque instance porte
// un attribut `aParams` = (opacité, danger 0..1, sens du dégradé, luminosité). Les matériaux
// Lambert sont complétés (onBeforeCompile) : arêtes plus claires mesurées en mètres, pied plus
// sombre (occlusion factice), fondu au loin, teinte rouge pulsée. Uniformes partagés : temps,
// position du rover, distances de fondu.

import * as THREE from '../../vendor/three/three.module.js';

export const PALETTE = {
  fondHaut: '#20252d',
  fondMilieu: '#13161b',
  fondBas: '#0b0d10',
  sol: 0x14171c,
  grille: 0x1f242b,
  grilleForte: 0x2a3039,
  libre: 0x262b33, // ajouté au sol (mélange additif) : gris plus clair là où c'est praticable
  mur: 0x8c929b,
  murArete: 0xd4d9e0,
  bloc: 0x7b818a,
  blocArete: 0xc2c8d0,
  bas: 0xb9ab92,
  basArete: 0xf0dfbf,
  silhouette: 0xb3b9c2,
  tronc: 0x6c7179,
  houppier: 0x99a0a9,
  danger: 0xff5c4d,
  accent: 0x4ea8f2,
  accentFort: 0x1f7fd1,
  blanc: 0xf2f4f7,
  coque: 0xe9ecf0,
  planche: 0x3b4048,
  pneu: 0x1f2328,
  jante: 0x9aa1ab,
  mat: 0x4b525d,
  lidar: 0x2a2f37,
  lidarTete: 0x191c21,
  optique: 0x0b0d10,
  lidarPoints: 0xc9ced6,
  stereoPoints: 0xe7b974,
  etiquetteFond: 'rgba(19, 22, 27, 0.82)',
  etiquetteBord: 'rgba(242, 244, 247, 0.22)',
  etiquetteTexte: '#f2f4f7',
};

/** Uniformes communs à tous les matériaux du monde (mis à jour une fois par image). */
export function creerUniformes() {
  return {
    uTemps: { value: 0 },
    uCentre: { value: new THREE.Vector2() },
    uFondu: { value: new THREE.Vector2(20, 32) },
    uCamera: { value: new THREE.Vector3(0, 0, 10) },
    uDanger: { value: new THREE.Color(PALETTE.danger) },
  };
}

const ENTETE_VERTEX = /* glsl */ `
attribute vec4 aParams;
varying vec4 vParams;
varying vec3 vLocal;
varying vec3 vDemi;
varying vec3 vMonde;
`;

const CALCUL_VERTEX = /* glsl */ `
vParams = aParams;
vec3 echelle = vec3(length(instanceMatrix[0].xyz), length(instanceMatrix[1].xyz), length(instanceMatrix[2].xyz));
vDemi = 0.5 * echelle;
vLocal = vec3(position.x * echelle.x, position.y * echelle.y, (position.z - 0.5) * echelle.z);
vMonde = (modelMatrix * instanceMatrix * vec4(position, 1.0)).xyz;
`;

/**
 * Fondu près de la caméra et devant le rover : un objet qui masque le rover (entre la caméra
 * et lui) ou que la caméra traverse devient presque transparent (vue en écorché).
 */
const FONDU_CAMERA = /* glsl */ `
uniform vec3 uCamera;
float fonduCamera(vec3 p) {
  float a = smoothstep(0.6, 2.4, distance(p, uCamera));
  vec3 r = vec3(uCentre, 0.25);
  vec3 ab = r - uCamera;
  float t = clamp(dot(p - uCamera, ab) / max(dot(ab, ab), 1e-4), 0.0, 1.0);
  float d = distance(p, uCamera + t * ab);
  float masque = (1.0 - smoothstep(0.35, 1.1, d)) * (1.0 - smoothstep(0.8, 0.95, t));
  return a * (1.0 - 0.8 * masque);
}
`;

const ENTETE_FRAGMENT = /* glsl */ `
uniform float uTemps;
uniform vec2 uCentre;
uniform vec2 uFondu;
uniform vec3 uDanger;
uniform vec3 uArete;
varying vec4 vParams;
varying vec3 vLocal;
varying vec3 vDemi;
varying vec3 vMonde;
${FONDU_CAMERA}
`;

/**
 * Matériau des boîtes instanciées (murs, blocs, obstacles bas) : géométrie unité (x, y dans
 * [-0,5 ; 0,5], z dans [0 ; 1]) mise à l'échelle par la matrice d'instance.
 * - `trait` : intensité du liseré clair des arêtes (largeur ≈ 1,5 px) ;
 * - `adoucir` : largeur (m) du dégradé clair près des arêtes (arêtes adoucies) ;
 * - `degrade` : opacité décroissante le long de x (prolongements extrapolés) ;
 * - `pied` : assombrissement près du sol.
 */
export function materiauBoite(uniformes, { couleur, arete, trait = 0.7, adoucir = 0, degrade = false, pied = 0.38, depthWrite = true }) {
  const m = new THREE.MeshLambertMaterial({ color: couleur, transparent: true, depthWrite });
  const defines = {};
  if (adoucir > 0) defines.ADOUCIR = adoucir.toFixed(3);
  if (degrade) defines.DEGRADE = 1;
  m.defines = defines;
  const uArete = { value: new THREE.Color(arete) };
  m.onBeforeCompile = (shader) => {
    Object.assign(shader.uniforms, uniformes, { uArete });
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', `#include <common>\n${ENTETE_VERTEX}`)
      .replace('#include <begin_vertex>', `#include <begin_vertex>\n${CALCUL_VERTEX}`);
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', `#include <common>\n${ENTETE_FRAGMENT}`)
      .replace('#include <color_fragment>', /* glsl */ `#include <color_fragment>
        vec3 dBord = vDemi - abs(vLocal);
        // distance a l'arete la plus proche sur la face courante : mediane des trois ecarts
        float bord = max(min(dBord.x, dBord.y), min(max(dBord.x, dBord.y), dBord.z));
        float hauteur = vLocal.z + vDemi.z;
        float largeurTrait = fwidth(bord);
        float liser = 1.0 - smoothstep(largeurTrait * 0.5, largeurTrait * 1.6 + 0.004, bord);
        liser *= smoothstep(0.0, 0.03, hauteur); // pas de trait au ras du sol
        diffuseColor.rgb *= vParams.w * mix(${(1 - pied).toFixed(3)}, 1.0, smoothstep(0.0, 0.45, hauteur));
        float alphaObjet = vParams.x;
        #ifdef DEGRADE
          float u = clamp(vLocal.x / max(vDemi.x, 1e-3), -1.0, 1.0);
          float t = vParams.z == 0.0 ? abs(u) : 0.5 * (u * vParams.z + 1.0);
          alphaObjet *= 1.0 - smoothstep(0.0, 1.0, t) * 0.92;
        #endif
        alphaObjet *= (1.0 - smoothstep(uFondu.x, uFondu.y, distance(vMonde.xy, uCentre))) * fonduCamera(vMonde);
        if (alphaObjet < 0.01) discard;
        diffuseColor.a *= alphaObjet;`)
      .replace('#include <opaque_fragment>', /* glsl */ `
        #ifdef ADOUCIR
          outgoingLight = mix(outgoingLight, uArete, (1.0 - smoothstep(0.0, ADOUCIR, bord)) * 0.22);
        #endif
        outgoingLight = mix(outgoingLight, uArete, liser * ${trait.toFixed(3)});
        float pouls = 0.5 + 0.5 * sin(uTemps * 6.5);
        outgoingLight = mix(outgoingLight, uDanger, vParams.y * (0.32 + 0.3 * pouls));
        #include <opaque_fragment>`);
  };
  m.customProgramCacheKey = () => `fm-boite-${adoucir}-${degrade}-${trait}-${pied}`;
  return m;
}

/** Matériau des cylindres / poteaux (petits obstacles compacts vus au lidar). */
export function materiauArbre(uniformes, { couleur = PALETTE.bloc, arete = PALETTE.blocArete, haut = 1.0, bas = 0.68 }) {
  const m = new THREE.MeshLambertMaterial({ color: couleur, transparent: true });
  const uArete = { value: new THREE.Color(arete) };
  m.onBeforeCompile = (shader) => {
    Object.assign(shader.uniforms, uniformes, { uArete });
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nattribute vec4 aParams;\nvarying vec4 vParams;\nvarying vec3 vMonde;\nvarying vec3 vPos;')
      .replace('#include <begin_vertex>', '#include <begin_vertex>\nvParams = aParams;\nvMonde = (modelMatrix * instanceMatrix * vec4(position, 1.0)).xyz;\nvPos = position;');
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', `#include <common>\nuniform float uTemps;\nuniform vec2 uCentre;\nuniform vec2 uFondu;\nuniform vec3 uDanger;\nuniform vec3 uArete;\nvarying vec4 vParams;\nvarying vec3 vMonde;\nvarying vec3 vPos;\n${FONDU_CAMERA}`)
      .replace('#include <color_fragment>', `#include <color_fragment>
        diffuseColor.rgb *= vParams.w * mix(${bas.toFixed(3)}, ${haut.toFixed(3)}, clamp(vPos.z, 0.0, 1.0));
        float alphaObjet = vParams.x * (1.0 - smoothstep(uFondu.x, uFondu.y, distance(vMonde.xy, uCentre))) * fonduCamera(vMonde);
        if (alphaObjet < 0.01) discard;
        diffuseColor.a *= alphaObjet;`)
      .replace('#include <opaque_fragment>', `
        // flanc : distance au bord haut ; dessus : distance au pourtour
        float dHaut = vPos.z > 0.9995 ? 1.0 - length(vPos.xy) : 1.0 - vPos.z;
        float liser = (1.0 - smoothstep(0.0, fwidth(dHaut) * 1.6 + 0.01, dHaut)) * smoothstep(0.7, 0.98, vPos.z);
        outgoingLight = mix(outgoingLight, uArete, liser * 0.6);
        float pouls = 0.5 + 0.5 * sin(uTemps * 6.5);
        outgoingLight = mix(outgoingLight, uDanger, vParams.y * (0.32 + 0.3 * pouls));
        #include <opaque_fragment>`);
  };
  m.customProgramCacheKey = () => `fm-cylindre-${haut}-${bas}`;
  return m;
}

/**
 * Silhouettes bas-poly des êtres vivants (personnes, animaux) : facettes plates, plus claires
 * vers le haut, fondu, écorché près de la caméra, rouge pulsé en danger.
 */
export function materiauSilhouette(uniformes, { couleur = PALETTE.silhouette, haut = 1.12, bas = 0.62 } = {}) {
  const m = new THREE.MeshLambertMaterial({ color: couleur, transparent: true, flatShading: true });
  m.onBeforeCompile = (shader) => {
    Object.assign(shader.uniforms, uniformes);
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', '#include <common>\nattribute vec4 aParams;\nvarying vec4 vParams;\nvarying vec3 vMonde;\nvarying float vZ;')
      .replace('#include <begin_vertex>', '#include <begin_vertex>\nvParams = aParams;\nvMonde = (modelMatrix * instanceMatrix * vec4(position, 1.0)).xyz;\nvZ = position.z;');
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', `#include <common>\nuniform float uTemps;\nuniform vec2 uCentre;\nuniform vec2 uFondu;\nuniform vec3 uDanger;\nvarying vec4 vParams;\nvarying vec3 vMonde;\nvarying float vZ;\n${FONDU_CAMERA}`)
      .replace('#include <color_fragment>', `#include <color_fragment>
        diffuseColor.rgb *= vParams.w * mix(${bas.toFixed(3)}, ${haut.toFixed(3)}, clamp(vZ, 0.0, 1.0));
        float alphaObjet = vParams.x * (1.0 - smoothstep(uFondu.x, uFondu.y, distance(vMonde.xy, uCentre))) * fonduCamera(vMonde);
        if (alphaObjet < 0.01) discard;
        diffuseColor.a *= alphaObjet;`)
      .replace('#include <opaque_fragment>', `
        float pouls = 0.5 + 0.5 * sin(uTemps * 6.5);
        outgoingLight = mix(outgoingLight, uDanger, vParams.y * (0.3 + 0.25 * pouls));
        #include <opaque_fragment>`);
  };
  m.customProgramCacheKey = () => `fm-silhouette-${haut}-${bas}`;
  return m;
}

/**
 * Ombres de contact au sol : un carré unité par instance, rectangle arrondi flou (rond si
 * `aParams.z` = 1). La couleur vire au rouge pulsé sous un objet en danger.
 */
export function materiauOmbre(uniformes) {
  return new THREE.ShaderMaterial({
    uniforms: uniformes,
    transparent: true,
    depthWrite: false,
    vertexShader: /* glsl */ `
      attribute vec4 aParams;
      varying vec4 vParams;
      varying vec2 vLocal;
      varying vec2 vDemi;
      varying vec2 vMonde;
      void main() {
        vParams = aParams;
        vec2 echelle = vec2(length(instanceMatrix[0].xyz), length(instanceMatrix[1].xyz));
        vDemi = 0.5 * echelle;
        vLocal = position.xy * echelle;
        vec4 monde = modelMatrix * instanceMatrix * vec4(position, 1.0);
        vMonde = monde.xy;
        gl_Position = projectionMatrix * viewMatrix * monde;
      }`,
    fragmentShader: /* glsl */ `
      uniform float uTemps;
      uniform vec2 uCentre;
      uniform vec2 uFondu;
      uniform vec3 uDanger;
      varying vec4 vParams;
      varying vec2 vLocal;
      varying vec2 vDemi;
      varying vec2 vMonde;
      void main() {
        float flou = min(0.18, 0.45 * min(vDemi.x, vDemi.y));
        float coin = vParams.z > 0.5 ? min(vDemi.x, vDemi.y) : flou + 0.02;
        vec2 q = abs(vLocal) - (vDemi - vec2(coin));
        float d = length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - coin + flou;
        float a = 1.0 - smoothstep(-flou, flou, d);
        float pouls = 0.5 + 0.5 * sin(uTemps * 6.5);
        float danger = vParams.y * (0.55 + 0.45 * pouls);
        vec3 couleur = mix(vec3(0.0), uDanger, danger);
        a *= vParams.x * mix(0.55, 0.75, danger) * (1.0 - smoothstep(uFondu.x, uFondu.y, distance(vMonde, uCentre)));
        gl_FragColor = vec4(couleur, a);
        #include <colorspace_fragment>
      }`,
  });
}

/** Petite texture ronde floue (ombre du rover). */
export function textureTache(taille = 64) {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = taille;
  const g = canvas.getContext('2d');
  const degrade = g.createRadialGradient(taille / 2, taille / 2, 0, taille / 2, taille / 2, taille / 2);
  degrade.addColorStop(0, 'rgba(0,0,0,0.75)');
  degrade.addColorStop(0.55, 'rgba(0,0,0,0.45)');
  degrade.addColorStop(1, 'rgba(0,0,0,0)');
  g.fillStyle = degrade;
  g.fillRect(0, 0, taille, taille);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}
