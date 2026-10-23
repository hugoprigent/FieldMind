// Serveur simulé dans le navigateur (index.html?mock=1) : même protocole que le rover
// (PROTOCOLE.md), pour développer et tester le poste sans matériel. Jardin clos avec caisses,
// poteaux et une marche plus basse que le lidar ; cinématique bicyclette, sécurité simplifiée,
// file d'actions, lidar, carte d'occupation en PNG, caméras rendues sur canvas, IA factice.
// Ce fichier n'est jamais chargé par le vrai poste.

import { ConnexionBase } from './connexion.js';
import { ModeleCamera, completerProfil, DEG, poseSurArc, versParent, versLocal, borner } from './geometrie.js';

const PROFIL = completerProfil({
  geometrie: { empattement_m: 0.38, voie_arriere_m: 0.39, voie_avant_m: 0.31, diametre_roue_arriere_m: 0.2, diametre_roue_avant_m: 0.15, braquage_max_deg: 31, emprise_arriere_m: -0.11, emprise_avant_m: 0.46, emprise_demi_largeur_m: 0.225, hauteur_max_m: 0.36, masse_kg: 5.8 },
  lidar: { modele: 'RPLIDAR C1', x_m: 0, y_m: 0, z_m: 0.322, decalage_deg: 0, sens_horaire: true, portee_min_m: 0.12, portee_max_m: 12 },
  cameras: { largeur: 1280, hauteur: 720, ips: 15, base_m: 0.12, x_m: 0.415, z_m: 0.265, inclinaison_deg: 10, hfov_deg: 78, port_gauche: '', port_droite: '' },
  vitesses: { max_manuel_m_s: 0.8, max_auto_m_s: 0.5, max_arriere_m_s: 0.3, defaut_action_m_s: 0.35 },
  securite: { deceleration_m_s2: 0.8, latence_s: 0.25, marge_avant_m: 0.25, marge_laterale_m: 0.1, distance_arret_absolu_m: 0.3, inclinaison_max_deg: 25, geofence_m: 60 },
  traction: { pwm_par_m_s: 35, pwm_seuil: 4, pwm_max: 40, pwm_max_banc: 15 },
  direction: { servo_par_degre_roue: 1, servo_neutre_deg: 0, servo_max_deg: 40 },
});

const REGLABLES = {
  'vitesses.max_manuel_m_s': [0.05, 1.5], 'vitesses.max_auto_m_s': [0.05, 1.0], 'vitesses.max_arriere_m_s': [0.05, 0.5],
  'vitesses.defaut_action_m_s': [0.05, 1.0], 'traction.pwm_par_m_s': [5.0, 100.0], 'traction.pwm_seuil': [0, 20],
  'traction.pwm_max': [5, 100], 'traction.delai_inversion_s': [0.2, 3.0], 'direction.servo_par_degre_roue': [0.2, 5.0],
  'direction.servo_neutre_deg': [-20.0, 20.0], 'direction.servo_max_deg': [5.0, 90.0], 'securite.marge_avant_m': [0.1, 1.0],
  'securite.deceleration_m_s2': [0.2, 3.0], 'securite.perte_operateur_s': [1.0, 60.0], 'securite.geofence_m': [5.0, 500.0],
  'perception.stereo_active': [0, 1],
  'cameras.port_gauche': 'texte', 'cameras.port_droite': 'texte', 'imu.axe_x': 'texte', 'imu.axe_y': 'texte',
  'imu.axe_z': 'texte', 'audio.peripherique': 'texte',
};
const ENTIERS = new Set(['traction.pwm_seuil', 'traction.pwm_max', 'roues.impulsions_par_metre']);
Object.assign(REGLABLES, { 'traction.equilibre_d': [0.8, 1.25], 'traction.boucle_lacet': [0, 1], 'roues.impulsions_par_metre': [0, 5000] });

/** `bienvenue.reglables_details` : libellé, groupe, unité, pas, type. */
const DETAILS = Object.fromEntries(Object.entries({
  'vitesses.max_manuel_m_s': ['Vitesse max en manuel', 'Vitesses', 'm/s', 0.05],
  'vitesses.max_auto_m_s': ['Vitesse max en auto', 'Vitesses', 'm/s', 0.05],
  'vitesses.max_arriere_m_s': ['Vitesse max en arrière', 'Vitesses', 'm/s', 0.05],
  'vitesses.defaut_action_m_s': ['Vitesse des actions', 'Vitesses', 'm/s', 0.05],
  'traction.pwm_par_m_s': ['PWM pour 1 m/s', 'Traction', '%', 0.5],
  'traction.pwm_seuil': ['Zone morte', 'Traction', '%', 1],
  'traction.pwm_max': ['PWM maximal', 'Traction', '%', 1],
  'traction.delai_inversion_s': ['Délai d’inversion', 'Traction', 's', 0.05],
  'traction.equilibre_d': ['Équilibre droite / gauche', 'Traction', '×', 0.01],
  'traction.boucle_lacet': ['Tenue de cap', 'Traction', '', 1, 'booleen'],
  'roues.impulsions_par_metre': ['Impulsions par mètre', 'Roues', '', 10],
  'direction.servo_par_degre_roue': ['Servo par degré de roue', 'Direction', '°/°', 0.01],
  'direction.servo_neutre_deg': ['Neutre du servo', 'Direction', '°', 0.5],
  'direction.servo_max_deg': ['Butée du servo', 'Direction', '°', 1],
  'securite.marge_avant_m': ['Marge avant', 'Sécurité', 'm', 0.05],
  'securite.deceleration_m_s2': ['Décélération', 'Sécurité', 'm/s²', 0.1],
  'securite.perte_operateur_s': ['Perte de l’opérateur', 'Sécurité', 's', 0.5],
  'securite.geofence_m': ['Rayon autorisé', 'Sécurité', 'm', 5],
  'perception.stereo_active': ['Stéréo', 'Perception', '', 1, 'booleen'],
  'cameras.port_gauche': ['Port gauche', 'Caméras', '', 0, 'texte'],
  'cameras.port_droite': ['Port droite', 'Caméras', '', 0, 'texte'],
  'imu.axe_x': ['Axe x', 'Centrale', '', 0, 'texte'],
  'imu.axe_y': ['Axe y', 'Centrale', '', 0, 'texte'],
  'imu.axe_z': ['Axe z', 'Centrale', '', 0, 'texte'],
  'audio.peripherique': ['Périphérique ALSA', 'Voix', '', 0, 'texte'],
}).map(([cle, [libelle, groupe, unite, pas, type = 'nombre']]) => {
  const bornes = REGLABLES[cle];
  return [cle, { libelle, groupe, unite, pas, type, ...(Array.isArray(bornes) ? { min: bornes[0], max: bornes[1] } : {}) }];
}));

// GPS : jardin autour de 43,7102 ; −1,0536 (x du monde vers l'est, y vers le nord).
const LAT0 = 43.7102, LON0 = -1.0536;
const versLatLon = (x, y) => [LAT0 + y / 111320, LON0 + x / (111320 * Math.cos(LAT0 * DEG))];
const versMondeGps = (lat, lon) => [(lon - LON0) * 111320 * Math.cos(LAT0 * DEG), (lat - LAT0) * 111320];

function dansPolygone(x, y, poly) {
  let dedans = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j];
    if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) dedans = !dedans;
  }
  return dedans;
}

const MODELES_IA = {
  gemini: { agent: 'gemini-4-argon', vision: 'gemini-robotics-er-2-preview', transcription: 'gemini-3.5-transcribe' },
  gx10: { agent: 'qwen3-32b-instruct', vision: 'qwen3-vl-8b', transcription: 'whisper-large-v3' },
  aucun: {},
};

// ------------------------------------------------------------------ monde (repère « monde » = odom au démarrage)

const HAUTEUR_MUR = 1.1;
const MURS = [[-3, -4, 9, -4], [9, -4, 9, 4], [9, 4, -3, 4], [-3, 4, -3, -4]];
const BOITES = [
  { nom: 'caisse bleue', x: 3.4, y: 1.5, l: 0.6, w: 0.5, h: 0.5, couleur: [58, 111, 176] },
  { nom: 'chaise rouge', x: 5.6, y: -1.3, l: 0.45, w: 0.45, h: 0.85, couleur: [196, 52, 40] },
  { nom: 'carton', x: 7.2, y: 2.3, l: 0.8, w: 0.6, h: 0.7, couleur: [176, 136, 80] },
  { nom: 'marche', x: 2.3, y: -0.75, l: 0.5, w: 0.4, h: 0.12, couleur: [130, 130, 126], bas: true },
  { nom: 'table', x: -1.6, y: 2.7, l: 1.2, w: 0.7, h: 0.75, couleur: [138, 106, 74] },
];
const POTEAUX = [{ nom: 'poteau', x: 4.6, y: 0.25, r: 0.08, h: 1.2 }, { nom: 'poteau', x: 7.6, y: -2.6, r: 0.1, h: 1.2 }];

function aretesBoite(b) {
  const x0 = b.x - b.l / 2, x1 = b.x + b.l / 2, y0 = b.y - b.w / 2, y1 = b.y + b.w / 2;
  return [[x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0]];
}

const SEGMENTS_HAUTS = [...MURS, ...BOITES.filter((b) => !b.bas).flatMap(aretesBoite)];
const SEGMENTS_TOUS = [...MURS, ...BOITES.flatMap(aretesBoite)];

/** Points d'obstacle (pour la sécurité) : périmètres échantillonnés tous les 5 cm. */
const POINTS_OBSTACLES = (() => {
  const pts = [];
  for (const [x0, y0, x1, y1] of SEGMENTS_TOUS) {
    const n = Math.max(1, Math.ceil(Math.hypot(x1 - x0, y1 - y0) / 0.05));
    for (let i = 0; i < n; i++) pts.push(x0 + ((x1 - x0) * i) / n, y0 + ((y1 - y0) * i) / n);
  }
  for (const p of POTEAUX) for (let a = 0; a < 360; a += 30) pts.push(p.x + p.r * Math.cos(a * DEG), p.y + p.r * Math.sin(a * DEG));
  return Float32Array.from(pts);
})();

/** Lancer de rayon (m) depuis (x, y) dans la direction a, contre segments et poteaux. */
function lancer(x, y, a, segments = SEGMENTS_HAUTS, portee = 12) {
  const dx = Math.cos(a), dy = Math.sin(a);
  let t = portee;
  for (const [x0, y0, x1, y1] of segments) {
    const ex = x1 - x0, ey = y1 - y0;
    const den = dx * ey - dy * ex;
    if (Math.abs(den) < 1e-9) continue;
    const ux = x0 - x, uy = y0 - y;
    const tt = (ux * ey - uy * ex) / den, s = (ux * dy - uy * dx) / den;
    if (tt > 0 && tt < t && s >= 0 && s <= 1) t = tt;
  }
  for (const p of POTEAUX) {
    const fx = x - p.x, fy = y - p.y;
    const b = fx * dx + fy * dy, c = fx * fx + fy * fy - p.r * p.r;
    const disc = b * b - c;
    if (disc < 0) continue;
    const tt = -b - Math.sqrt(disc);
    if (tt > 0 && tt < t) t = tt;
  }
  return t;
}

const bruit = (a) => (Math.random() * 2 - 1) * a;
const normaliser = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const tMono = () => 1000 + performance.now() / 1000;
const tMur = () => Date.now() / 1000;

// ------------------------------------------------------------------ rendu des caméras

const SOL = [[86, 110, 64], [78, 101, 58]];
const CIEL_HAUT = '#9cb8d4', CIEL_BAS = '#d9e2e8';
const LUMIERE = [0.45, 0.3, 0.84];

function couleurProfondeur(d, portee = 6) {
  const f = borner(d / portee, 0, 1);
  const arrets = [[0, [220, 40, 40]], [0.25, [245, 160, 40]], [0.5, [240, 230, 70]], [0.75, [60, 190, 170]], [1, [40, 70, 160]]];
  for (let i = 1; i < arrets.length; i++) {
    if (f <= arrets[i][0]) {
      const [f0, c0] = arrets[i - 1], [f1, c1] = arrets[i];
      const r = (f - f0) / (f1 - f0);
      return c0.map((v, k) => Math.round(v + (c1[k] - v) * r));
    }
  }
  return arrets[arrets.length - 1][1];
}

function couperPolygone(pts, proche = 0.05) {
  const sortie = [];
  for (let i = 0; i < pts.length; i++) {
    const a = pts[i], b = pts[(i + 1) % pts.length];
    const devantA = a[2] > proche, devantB = b[2] > proche;
    if (devantA) sortie.push(a);
    if (devantA !== devantB) {
      const r = (proche - a[2]) / (b[2] - a[2]);
      sortie.push([a[0] + r * (b[0] - a[0]), a[1] + r * (b[1] - a[1]), proche]);
    }
  }
  return sortie;
}

/** Faces du monde : { pts: [[x, y, z]…], couleur, normale }. */
const FACES = (() => {
  const faces = [];
  const quad = (x0, y0, x1, y1, h, couleur) => {
    const nx = y1 - y0, ny = -(x1 - x0), n = Math.hypot(nx, ny);
    faces.push({ pts: [[x0, y0, 0], [x1, y1, 0], [x1, y1, h], [x0, y0, h]], couleur, normale: [nx / n, ny / n, 0] });
  };
  for (const [x0, y0, x1, y1] of MURS) quad(x0, y0, x1, y1, HAUTEUR_MUR, [156, 138, 110]);
  for (const b of BOITES) {
    for (const [x0, y0, x1, y1] of aretesBoite(b)) quad(x0, y0, x1, y1, b.h, b.couleur);
    const x0 = b.x - b.l / 2, x1 = b.x + b.l / 2, y0 = b.y - b.w / 2, y1 = b.y + b.w / 2;
    faces.push({ pts: [[x0, y0, b.h], [x1, y0, b.h], [x1, y1, b.h], [x0, y1, b.h]], couleur: b.couleur, normale: [0, 0, 1] });
  }
  for (const p of POTEAUX) {
    for (let i = 0; i < 8; i++) {
      const a0 = (i * Math.PI) / 4, a1 = ((i + 1) * Math.PI) / 4;
      quad(p.x + p.r * Math.cos(a0), p.y + p.r * Math.sin(a0), p.x + p.r * Math.cos(a1), p.y + p.r * Math.sin(a1), p.h, [200, 200, 196]);
    }
  }
  return faces;
})();

function rendreCamera(ctx, w, h, pose, camera) {
  const profondeur = camera === 'profondeur';
  const modele = new ModeleCamera({ ...PROFIL.cameras, largeur: w, hauteur: h }, camera === 'droite' ? -1 : 1);
  const versCam = (X, Y, Z) => { const [bx, by] = versLocal(pose, X, Y); return modele.versCamera(bx, by, Z); };
  const versPx = (p) => [(0.5 + (modele.fx * p[0]) / p[2]) * w, (0.5 + (modele.fy * p[1]) / p[2]) * h];
  const horizon = modele.horizon() * h;

  // Ciel puis sol.
  if (profondeur) {
    ctx.fillStyle = '#101018';
    ctx.fillRect(0, 0, w, h);
    for (let y = Math.max(0, Math.floor(horizon)); y < h; y += 4) {
      const sol = modele.pixelVersSol(0.5, (y + 2) / h, 50);
      const d = sol ? Math.hypot(sol[0] - modele.x, sol[1]) : 50;
      ctx.fillStyle = `rgb(${couleurProfondeur(d).join(',')})`;
      ctx.fillRect(0, y, w, 4);
    }
  } else {
    const ciel = ctx.createLinearGradient(0, 0, 0, Math.max(1, horizon));
    ciel.addColorStop(0, CIEL_HAUT);
    ciel.addColorStop(1, CIEL_BAS);
    ctx.fillStyle = ciel;
    ctx.fillRect(0, 0, w, h);
    ctx.fillStyle = 'rgb(70, 92, 54)';
    ctx.fillRect(0, horizon, w, h - horizon);
    // Dalles d'herbe de 1 m (visibles en mouvement).
    for (let x = -3; x < 9; x++) {
      for (let y = -4; y < 4; y++) {
        const pts = couperPolygone([versCam(x, y, 0), versCam(x + 1, y, 0), versCam(x + 1, y + 1, 0), versCam(x, y + 1, 0)]);
        if (pts.length < 3) continue;
        const c = SOL[(x + y + 100) % 2];
        ctx.fillStyle = `rgb(${c.join(',')})`;
        ctx.beginPath();
        pts.forEach((p, i) => { const [u, v] = versPx(p); if (i) ctx.lineTo(u, v); else ctx.moveTo(u, v); });
        ctx.fill();
      }
    }
  }

  // Faces, de la plus lointaine à la plus proche.
  const liste = [];
  for (const f of FACES) {
    const pts = couperPolygone(f.pts.map(([x, y, z]) => versCam(x, y, z)));
    if (pts.length < 3) continue;
    const z = pts.reduce((s, p) => s + Math.hypot(p[0], p[1], p[2]), 0) / pts.length;
    liste.push({ f, pts, z });
  }
  liste.sort((a, b) => b.z - a.z);
  for (const { f, pts, z } of liste) {
    let c;
    if (profondeur) {
      c = couleurProfondeur(z);
    } else {
      const eclairage = 0.55 + 0.45 * Math.max(0, f.normale[0] * LUMIERE[0] + f.normale[1] * LUMIERE[1] + f.normale[2] * LUMIERE[2]);
      const brume = borner((z - 4) / 14, 0, 0.35);
      c = f.couleur.map((v) => Math.round(v * eclairage * (1 - brume) + 215 * brume));
    }
    ctx.fillStyle = `rgb(${c.join(',')})`;
    ctx.beginPath();
    pts.forEach((p, i) => { const [u, v] = versPx(p); if (i) ctx.lineTo(u, v); else ctx.moveTo(u, v); });
    ctx.closePath();
    ctx.fill();
    if (!profondeur) {
      ctx.strokeStyle = 'rgba(0,0,0,0.18)';
      ctx.lineWidth = 1;
      ctx.stroke();
    }
  }
  if (!profondeur) {
    // Léger vignettage, pour l'aspect « webcam ».
    const v = ctx.createRadialGradient(w / 2, h / 2, h * 0.35, w / 2, h / 2, w * 0.7);
    v.addColorStop(0, 'rgba(0,0,0,0)');
    v.addColorStop(1, 'rgba(0,0,0,0.28)');
    ctx.fillStyle = v;
    ctx.fillRect(0, 0, w, h);
  }
}

// ------------------------------------------------------------------ IA factice

const NOMBRES = { un: 1, une: 1, deux: 2, trois: 3, quatre: 4, cinq: 5, six: 6, sept: 7, huit: 8, neuf: 9, dix: 10, demi: 0.5 };

function lireNombre(mot) {
  if (!mot) return null;
  const n = Number(String(mot).replace(',', '.'));
  return Number.isFinite(n) ? n : NOMBRES[String(mot).toLowerCase()] ?? null;
}

/** Consigne en français → { actions, reponse } (sans modèle, pour la démo). */
function comprendre(texte) {
  const t = texte.toLowerCase().normalize('NFC').trim();
  if (/^(stop|arr[êe]te|halte)\b/.test(t)) return { stop: true, reponse: 'Je m’arrête.' };
  const actions = [], phrases = [];
  for (const partie of t.split(/\s*(?:,|\bpuis\b|\bensuite\b|\bet\b)\s*/).filter(Boolean)) {
    let m;
    if ((m = partie.match(/(avance|recule)[a-z]*\s*(?:de |d’|d')?\s*([\d.,]+|un|une|deux|trois|quatre|cinq|six|sept|huit|neuf|dix)?\s*(m\b|mètres?|metres?|cm|centim)?/))) {
      let d = lireNombre(m[2]) ?? 1;
      if (m[3] && m[3].startsWith('c')) d /= 100;
      if (m[1].startsWith('recul')) d = -d;
      actions.push({ type: 'avancer', distance_m: d });
      phrases.push(d < 0 ? `je recule de ${String(-d).replace('.', ',')} m` : `j’avance de ${String(d).replace('.', ',')} m`);
    } else if ((m = partie.match(/tourne[a-z]*\s*(?:de\s*([\d.,]+)\s*(?:°|degrés?))?\s*(?:à|a|vers la)?\s*(gauche|droite)/))) {
      const angle = (lireNombre(m[1]) ?? 90) * (m[2] === 'gauche' ? 1 : -1);
      actions.push({ type: 'tourner', angle_deg: angle });
      phrases.push(`je tourne de ${Math.abs(angle)}° à ${m[2]}`);
    } else if (/(retour|reviens|rentre)/.test(partie)) {
      actions.push({ type: 'retour_depart' });
      phrases.push('je reviens au départ');
    } else if ((m = partie.match(/(?:cherche|trouve|va (?:à|a|vers) )\s*(?:la |le |les |une |un |l’|l')?(.+)/))) {
      actions.push({ type: 'chercher', objet: m[1].trim() });
      phrases.push(`je cherche « ${m[1].trim()} »`);
    } else if (/(vois|décris|decris|regarde|devant toi)/.test(partie)) {
      actions.push({ type: 'decrire' });
      phrases.push('je regarde');
    } else if ((m = partie.match(/attends?\s*([\d.,]+)?/))) {
      actions.push({ type: 'attendre', duree_s: lireNombre(m[1]) ?? 2 });
      phrases.push('j’attends');
    } else if (/photo/.test(partie)) {
      actions.push({ type: 'photo', camera: 'gauche' });
      phrases.push('je prends une photo');
    }
  }
  if (!actions.length) return { actions, reponse: 'Je n’ai pas compris la consigne.' };
  const reponse = phrases.join(', puis ');
  return { actions, reponse: reponse.charAt(0).toUpperCase() + reponse.slice(1) + '.' };
}

// ------------------------------------------------------------------ serveur simulé

export class ConnexionSimulee extends ConnexionBase {
  constructor(parametres = new URLSearchParams()) {
    super();
    this.jeton = '';
    this.materiel = parametres.get('materiel') === 'reel' ? 'reel' : 'simulation';
    this.ouverte = false;
    this.minuteries = [];
    this.reglages = { 'vitesses.max_manuel_m_s': 0.8, 'vitesses.max_auto_m_s': 0.5, 'vitesses.max_arriere_m_s': 0.3, 'vitesses.defaut_action_m_s': 0.35, 'traction.pwm_par_m_s': 35, 'traction.pwm_seuil': 4, 'traction.pwm_max': 40, 'traction.delai_inversion_s': 0.6, 'direction.servo_par_degre_roue': 1, 'direction.servo_neutre_deg': 0, 'direction.servo_max_deg': 40, 'securite.marge_avant_m': 0.25, 'securite.deceleration_m_s2': 0.8, 'securite.perte_operateur_s': 5, 'securite.geofence_m': 60, 'perception.stereo_active': true, 'traction.equilibre_d': 1, 'traction.boucle_lacet': true, 'roues.impulsions_par_metre': 0, 'cameras.port_gauche': '', 'cameras.port_droite': '', 'imu.axe_x': 'x', 'imu.axe_y': 'y', 'imu.axe_z': 'z', 'audio.peripherique': '' };
    this.gps = { trace: [], zone: null, lieux: [], parcours: [], boucle: null, biais: [0, 0], parcouru: 0, aligne: parametres.get('aligne') === '1' };
    this.inverse = false;
    this.ia = { profil: 'gemini', disponible: true, modeles: { ...MODELES_IA.gemini }, profils: ['gemini', 'gx10', 'aucun'], raison: null };
    // Rover (repère monde), commandes et états.
    this.rover = { x: 0, y: 0, cap: 0, v: 0, braquage: 0, omega: 0, accel: 0 };
    this.origine = { x: 0, y: 0, cap: 0 };
    this.mode = 'arret';
    this.arme = false;
    this.arretUrgence = false;
    this.pilotage = { vitesse: 0, braquage: 0, t: -10 };
    this.banc = { pwm_g: 0, pwm_d: 0, servo_deg: 0, t: -10 };
    this.lidarMarche = true;
    this.voix = { active: false, parle: false, niveau: 0, prochaine: 0, phrase: 0 };
    this.iaOccupee = 0;
    this.securite = { arret: true, raisons: [], vitesse_max_avant: 0, vitesse_max_arriere: 0, libre_avant_m: null, libre_arriere_m: null };
    this.commande = { vitesse: 0, braquage: 0, pwm_g: 0, pwm_d: 0, servo_deg: 0 };
    this.consigne = { vitesse: 0, braquage: 0, source: 'aucune' };
    this.mission = { file: [], courante: null, etat: 'attente', progression: 0, detail: '', plan_id: null, pause: false, attenteBloque: 0 };
    this.trajectoire = [[0, 0]];
    this.grille = new Uint8Array(800 * 800).fill(128);
    this.stereoPaires = 0;
    this.etiquettes = new Map(); // objets nommés par la vision (repère monde)
    this.libre = []; // derniers points du lidar (repère monde) pour le polygone libre
    this.compteurs = { chat: 1, vision: 1, plan: 1 };
    this.debut = performance.now();
  }

  // ---------------------------------------------------------------- liaison

  ouvrir() {
    this._changerEtat('connexion');
    setTimeout(() => {
      this.ouverte = true;
      this._changerEtat('ok');
      this._bienvenue();
      this._historique();
      const tic = (f, ms) => this.minuteries.push(setInterval(f, ms));
      tic(() => this._pas(0.05), 50);
      tic(() => this._envoyerEtat(), 100);
      tic(() => this._envoyerScan(), 200);
      tic(() => this._envoyerCarte(), 1000);
      tic(() => this._envoyerScene(), 250);
      tic(() => this._envoyerSysteme(), 1000);
      tic(() => this._envoyerGps(), 1000);
      tic(() => this._envoyerConsolePeriodique(), 1500);
    }, 250);
  }

  envoyer(message) {
    if (!this.ouverte) return false;
    const copie = JSON.parse(JSON.stringify(message));
    setTimeout(() => this._traiter(copie), 5);
    return true;
  }

  _emettre(message) {
    if (this.ouverte) this._recevoir(message);
  }

  _repondre(id, ok, extra = {}) {
    if (id === undefined) return;
    this._emettre({ type: 'reponse', id, ok, ...(ok ? {} : { erreur: extra.erreur || 'refusé' }), ...extra });
  }

  _journal(niveau, source, message) {
    this._emettre({ type: 'journal', t: tMur(), niveau, source, message });
  }

  _chat(role, texte, extra = {}) {
    this._emettre({ type: 'chat', id: `m${this.compteurs.chat++}`, t: tMur(), role, texte, origine: 'texte', ...extra });
  }

  _bienvenue() {
    this._emettre({
      type: 'bienvenue', version: '1.0.0', materiel: this.materiel, poste: 'poste-mock', profil: PROFIL, reglables: REGLABLES,
      reglables_details: DETAILS,
      reglages: { ...this.reglages }, cameras: ['gauche', 'droite'], ia: JSON.parse(JSON.stringify(this.ia)),
    });
  }

  _historique() {
    const t0 = tMur() - 40;
    const historique = [
      ['info', 'demarrage', 'FieldMind 1.0.0 (simulation)'], ['succes', 'esp', 'ESP connecté : ROVER_3, chien de garde 400 ms'],
      ['succes', 'lidar', 'RPLIDAR C1 : 10,1 Hz'], ['info', 'cameras', 'gauche /dev/video0, droite /dev/video2 (1280×720 MJPEG)'],
      ['alerte', 'stereo', 'calibration absente : modèle par défaut'], ['info', 'gnss', 'ATGM336H : 7 satellites'],
      ['info', 'ia', 'profil gemini : gemini-4-argon'], ['info', 'securite', 'démarrage en mode arrêt, désarmé'],
    ];
    historique.forEach(([niveau, source, message], i) => this._emettre({ type: 'journal', t: t0 + i * 2, niveau, source, message }));
    this._emettre({ type: 'console', lignes: [[t0, '[FIRMWARE] ROVER_3'], [t0 + 0.1, '[CHIEN] 400 ms'], [t0 + 0.2, '[IMU] MPU-6050 OK']] });
    this._chat('systeme', 'Rover prêt.');
  }

  /** Flux d'une caméra : canevas animé à la place du MJPEG. */
  flux(camera, { ips = 10, largeur = 640 } = {}) {
    const canvas = document.createElement('canvas');
    canvas.width = largeur;
    canvas.height = Math.round((largeur * 9) / 16);
    const ctx = canvas.getContext('2d');
    let minuterie = null, pret = false;
    const dessiner = () => {
      if (!this.ouverte) return;
      let nom = camera;
      if (this.inverse && nom !== 'profondeur') nom = nom === 'gauche' ? 'droite' : 'gauche';
      rendreCamera(ctx, canvas.width, canvas.height, this.rover, nom);
      pret = true;
    };
    return {
      element: canvas,
      ouvrir: () => {
        clearInterval(minuterie);
        dessiner();
        minuterie = setInterval(dessiner, 1000 / (camera === 'profondeur' ? Math.min(ips, 4) : ips));
      },
      fermer: () => { clearInterval(minuterie); minuterie = null; },
      relancer: () => {},
      dimensions: () => (pret && this.ouverte ? [canvas.width, canvas.height] : null),
    };
  }

  // ---------------------------------------------------------------- demandes du poste

  _traiter(m) {
    const id = m.id;
    const refuser = (erreur) => this._repondre(id, false, { erreur });
    switch (m.type) {
      case 'pilotage':
        if (this.mode !== 'manuel') return;
        this.pilotage = { vitesse: borner(Number(m.vitesse) || 0, -1, 1), braquage: borner(Number(m.braquage) || 0, -1, 1), t: performance.now() / 1000 };
        return;
      case 'mode': {
        if (!['arret', 'manuel', 'auto', 'banc'].includes(m.mode)) return refuser('mode inconnu');
        if (m.mode === 'banc' && m.confirmer !== true) return refuser('banc : confirmation requise');
        if (m.mode === 'banc' && Math.abs(this.rover.v) > 0.02) return refuser('le rover roule');
        this.mode = m.mode;
        this.pilotage = { vitesse: 0, braquage: 0, t: -10 };
        this._journal('info', 'conduite', `mode ${m.mode}`);
        return this._repondre(id, true);
      }
      case 'armer':
        if (this.arretUrgence) return refuser('arrêt d’urgence actif : réarmer d’abord');
        if (this.mode === 'arret') return refuser('choisir d’abord le mode (manuel, auto ou banc)');
        this.arme = true;
        this._journal('alerte', 'securite', 'armé : traction autorisée');
        return this._repondre(id, true);
      case 'desarmer':
        this.arme = false;
        this._journal('info', 'securite', 'désarmé');
        return this._repondre(id, true);
      case 'arret_urgence':
        this.arretUrgence = true;
        this.arme = false;
        this._annulerMission('annule');
        this._journal('erreur', 'securite', 'ARRÊT D’URGENCE (poste)');
        return this._repondre(id, true);
      case 'rearmer':
        this.arretUrgence = false;
        this._journal('succes', 'securite', 'arrêt d’urgence levé');
        return this._repondre(id, true);
      case 'action': {
        const action = this._preparerAction(m.action);
        if (action.erreur) return refuser(action.erreur);
        if (m.remplacer) this._annulerMission('annule');
        this.mission.file.push(action);
        if (this.arme && this.mode !== 'auto') this.mode = 'auto';
        return this._repondre(id, true);
      }
      case 'annuler':
        this._annulerMission('annule');
        return this._repondre(id, true);
      case 'pause':
        this.mission.pause = true;
        return this._repondre(id, true);
      case 'reprendre':
        this.mission.pause = false;
        return this._repondre(id, true);
      case 'but': {
        const action = this._preparerAction({ type: 'aller_a', x: m.x, y: m.y, repere: 'odom' });
        this._annulerMission('annule');
        this.mission.file.push(action);
        if (this.arme) this.mode = 'auto';
        this._journal('info', 'mission', `but ${m.x.toFixed(2)} ; ${m.y.toFixed(2)} m`);
        return this._repondre(id, true);
      }
      case 'aller_pixel': {
        const action = this._preparerAction({ type: 'aller_pixel', camera: m.camera, u: m.u, v: m.v });
        if (action.erreur) return refuser(action.erreur);
        this._annulerMission('annule');
        this.mission.file.push(action);
        if (this.arme) this.mode = 'auto';
        return this._repondre(id, true);
      }
      case 'but_gps': {
        const action = this._actionGps(Number(m.lat), Number(m.lon));
        if (action.erreur) return refuser(action.erreur);
        this._annulerMission('annule');
        this.gps.parcours = [];
        this.gps.boucle = null;
        this.mission.file.push(action);
        if (this.arme) this.mode = 'auto';
        this._journal('info', 'gps', `but GPS ${Number(m.lat).toFixed(6)} ; ${Number(m.lon).toFixed(6)}`);
        return this._repondre(id, true);
      }
      case 'parcours_gps': {
        const points = Array.isArray(m.points) ? m.points.filter((p) => Array.isArray(p) && p.length === 2) : [];
        if (!points.length) return refuser('parcours vide');
        const actions = points.map(([lat, lon]) => this._actionGps(lat, lon));
        const refus = actions.find((a) => a.erreur);
        if (refus) return refuser(refus.erreur);
        this._annulerMission('annule');
        this.mission.file.push(...actions);
        this.gps.parcours = points.map((p) => [...p]);
        this.gps.boucle = m.boucle ? points.map((p) => [...p]) : null;
        if (this.arme) this.mode = 'auto';
        this._journal('info', 'gps', `parcours de ${points.length} points${m.boucle ? ' en boucle' : ''}`);
        return this._repondre(id, true);
      }
      case 'zone': {
        if (m.points === null) {
          this.gps.zone = null;
          this._journal('info', 'gps', 'zone autorisée retirée');
          return this._repondre(id, true);
        }
        const points = Array.isArray(m.points) ? m.points.filter((p) => Array.isArray(p) && p.length === 2) : [];
        if (points.length < 3) return refuser('zone : 3 points au moins');
        this.gps.zone = points.map((p) => [...p]);
        this._journal('succes', 'gps', `zone autorisée de ${points.length} points enregistrée`);
        return this._repondre(id, true);
      }
      case 'lieu': {
        const nom = String(m.nom || '').trim().slice(0, 40);
        if (!nom) return refuser('nom manquant');
        if (m.action === 'memoriser') {
          if (!this.gps.aligne) return refuser('position GPS pas encore alignée : rouler quelques mètres');
          const [lat, lon] = versLatLon(this.rover.x, this.rover.y);
          this.gps.lieux = [...this.gps.lieux.filter((l) => l.nom !== nom), { nom, lat: round(lat, 7), lon: round(lon, 7) }];
          this._journal('succes', 'gps', `lieu « ${nom} » mémorisé`);
          return this._repondre(id, true);
        }
        const lieu = this.gps.lieux.find((l) => l.nom === nom);
        if (!lieu) return refuser(`lieu inconnu : ${nom}`);
        if (m.action === 'supprimer') {
          this.gps.lieux = this.gps.lieux.filter((l) => l !== lieu);
          return this._repondre(id, true);
        }
        if (m.action === 'aller') {
          const action = this._actionGps(lieu.lat, lieu.lon, nom);
          if (action.erreur) return refuser(action.erreur);
          this._annulerMission('annule');
          this.mission.file.push(action);
          if (this.arme) this.mode = 'auto';
          return this._repondre(id, true);
        }
        return refuser('action inconnue');
      }
      case 'remise_a_zero_odom':
        this.origine = { x: this.rover.x, y: this.rover.y, cap: this.rover.cap };
        this.grille.fill(128);
        this.trajectoire = [[0, 0]];
        this._journal('info', 'estimation', 'odométrie remise à zéro, carte effacée');
        return this._repondre(id, true);
      case 'chat':
        this._repondre(id, true);
        return this._consigne(String(m.texte || ''), 'texte');
      case 'voix':
        this.voix.active = Boolean(m.active);
        this.voix.prochaine = performance.now() / 1000 + 4;
        this._journal('info', 'voix', this.voix.active ? 'écoute activée' : 'écoute coupée');
        return this._repondre(id, true);
      case 'vision':
        if (this.ia.profil === 'aucun') return refuser('IA désactivée');
        this._repondre(id, true);
        return this._vision(m.demande === 'chercher' ? 'chercher' : 'decrire', m.camera || 'gauche', m.objet);
      case 'console': {
        const texte = String(m.texte || '').trim();
        if (Math.abs(this.rover.v) > 0.02) return refuser('le rover roule');
        if (this.arme && this.mode !== 'banc') return refuser('rover armé : console refusée hors banc');
        this._repondre(id, true);
        return this._console(texte);
      }
      case 'banc':
        if (this.mode !== 'banc') return;
        this.banc = {
          pwm_g: borner(Math.round(m.pwm_g || 0), -PROFIL.traction.pwm_max_banc, PROFIL.traction.pwm_max_banc),
          pwm_d: borner(Math.round(m.pwm_d || 0), -PROFIL.traction.pwm_max_banc, PROFIL.traction.pwm_max_banc),
          servo_deg: borner(Number(m.servo_deg) || 0, -40, 40), t: performance.now() / 1000,
        };
        return;
      case 'reglage': {
        const bornes = REGLABLES[m.cle];
        if (!bornes) return refuser(`${m.cle} n’est pas réglable en marche`);
        let valeur;
        if (bornes === 'texte') valeur = String(m.valeur ?? '').slice(0, 200);
        else if (bornes[0] === 0 && bornes[1] === 1) valeur = Boolean(m.valeur);
        else {
          valeur = borner(Number(m.valeur), bornes[0], bornes[1]);
          if (!Number.isFinite(valeur)) return refuser('nombre invalide');
          if (ENTIERS.has(m.cle)) valeur = Math.round(valeur);
        }
        this.reglages[m.cle] = valeur;
        this._journal('info', 'reglages', `${m.cle} = ${valeur}`);
        return this._repondre(id, true, { valeur });
      }
      case 'ia_profil': {
        if (!MODELES_IA[m.profil]) return refuser('profil inconnu');
        this.ia = { ...this.ia, profil: m.profil, modeles: { ...MODELES_IA[m.profil] }, disponible: m.profil !== 'aucun', raison: m.profil === 'aucun' ? 'désactivée' : null };
        this._journal('info', 'ia', `profil ${m.profil}`);
        this._repondre(id, true, { valeur: JSON.parse(JSON.stringify(this.ia)) });
        return this._bienvenue(); // comme le serveur : bienvenue rediffusé à tous les postes
      }
      case 'lidar':
        if (Math.abs(this.rover.v) > 0.02) return refuser('le rover roule');
        this.lidarMarche = Boolean(m.marche);
        this._journal(this.lidarMarche ? 'info' : 'alerte', 'lidar', this.lidarMarche ? 'rotation relancée' : 'rotation arrêtée');
        return this._repondre(id, true);
      case 'calibration':
        return this._calibration(m, id);
      case 'photo': {
        const nom = `${m.camera || 'gauche'}_${new Date().toISOString().slice(0, 19).replace(/[-:]/g, '').replace('T', '_')}.jpg`;
        this._journal('succes', 'cameras', `photo ~/fieldmind/photos/${nom}`);
        return this._repondre(id, true, { valeur: nom });
      }
      case 'cameras_inverser':
        this.inverse = !this.inverse;
        this._journal('info', 'cameras', 'gauche et droite échangées');
        return this._repondre(id, true);
      default:
        return refuser(`message inconnu : ${m.type}`);
    }
  }

  // ---------------------------------------------------------------- mission

  _poseOdom() {
    const [x, y] = versLocal(this.origine, this.rover.x, this.rover.y);
    return { x, y, cap: normaliser(this.rover.cap - this.origine.cap) };
  }

  _preparerAction(a) {
    if (!a || typeof a !== 'object' || !a.type) return { erreur: 'action invalide' };
    const action = { ...a };
    if (a.type === 'aller_pixel') {
      const modele = new ModeleCamera(PROFIL.cameras, a.camera === 'droite' ? -1 : 1);
      const sol = modele.pixelVersSol(Number(a.u), Number(a.v), 15);
      if (!sol) return { erreur: 'point au-dessus de l’horizon' };
      const [x, y] = versParent(this.rover, sol[0], sol[1]);
      action.cible = [x, y];
    }
    if (a.type === 'avancer' && !Number.isFinite(Number(a.distance_m))) return { erreur: 'distance manquante' };
    return action;
  }

  /** Action « aller à un point GPS » (cible dans le monde), refusée hors de la zone. */
  _actionGps(lat, lon, nom) {
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return { erreur: 'coordonnées invalides' };
    const [x, y] = versMondeGps(lat, lon);
    if (Math.hypot(x - this.rover.x, y - this.rover.y) > 500) return { erreur: 'but à plus de 500 m' };
    if (this.gps.zone && !dansPolygone(x, y, this.gps.zone.map(([la, lo]) => versMondeGps(la, lo)))) return { erreur: 'but hors de la zone autorisée' };
    return { type: 'aller_gps', lat: round(lat, 7), lon: round(lon, 7), ...(nom ? { nom } : {}), cible: [x, y] };
  }

  _annulerMission(etat) {
    const m = this.mission;
    if (m.courante || m.file.length) m.etat = etat;
    m.file = [];
    m.courante = null;
    m.pause = false;
    if (this.gps) { this.gps.boucle = null; this.gps.parcours = []; }
    m.progression = 0;
    m.detail = '';
  }

  _demarrer(action) {
    const r = this.rover;
    const c = { action, depart: { x: r.x, y: r.y, cap: r.cap }, tourne: 0, debut: performance.now() / 1000, bloqueDepuis: null };
    if (action.type === 'aller_a') {
      c.cible = action.repere === 'odom' ? versParent(this.origine, action.x, action.y) : versParent(r, action.x, action.y);
    } else if (action.type === 'aller_pixel' || action.type === 'aller_gps') {
      c.cible = action.cible;
    } else if (action.type === 'retour_depart') {
      c.cible = [this.origine.x, this.origine.y];
    } else if (action.type === 'cap') {
      c.angle = normaliser(Number(action.cap_deg || 0) * DEG + this.origine.cap - r.cap);
    } else if (action.type === 'tourner') {
      c.angle = Number(action.angle_deg || 0) * DEG;
    }
    if (c.cible) c.distance0 = Math.max(0.01, Math.hypot(c.cible[0] - r.x, c.cible[1] - r.y));
    this.mission.courante = c;
    this.mission.progression = 0;
    this.mission.etat = 'route';
    if (action.type === 'decrire' || action.type === 'chercher') {
      this._vision(action.type, 'gauche', action.objet, true);
    } else if (action.type === 'photo') {
      this._journal('succes', 'cameras', 'photo gauche enregistrée');
    } else if (action.type === 'stop') {
      this._annulerMission('termine');
    }
  }

  /** Une étape de la mission → consigne { vitesse, braquage } ou null. */
  _pasMission() {
    const m = this.mission, r = this.rover;
    const bmax = PROFIL.geometrie.braquage_max_deg * DEG;
    const vdef = Math.min(this.reglages['vitesses.defaut_action_m_s'], this.reglages['vitesses.max_auto_m_s']);
    if (!m.courante && m.file.length) this._demarrer(m.file.shift());
    const c = m.courante;
    if (!c) return null;
    if (m.pause) { m.detail = 'en pause'; return { vitesse: 0, braquage: r.braquage }; }
    if (!this.arme || this.mode !== 'auto') { m.detail = this.arme ? 'mode auto requis' : 'désarmé'; m.etat = 'attente'; return null; }
    if (m.etat === 'attente') m.etat = 'route';
    const a = c.action;
    let consigne = { vitesse: 0, braquage: 0 }, fini = false;
    switch (a.type) {
      case 'avancer': {
        const d = Number(a.distance_m), sens = Math.sign(d) || 1;
        const parcouru = Math.abs((r.x - c.depart.x) * Math.cos(c.depart.cap) + (r.y - c.depart.y) * Math.sin(c.depart.cap));
        const reste = Math.abs(d) - parcouru;
        const v = Math.max(0.08, Math.min(a.vitesse_m_s || vdef, Math.sqrt(2 * 0.6 * Math.max(0, reste))));
        const ecart = normaliser(c.depart.cap - r.cap);
        consigne = { vitesse: sens * (sens < 0 ? Math.min(v, this.reglages['vitesses.max_arriere_m_s']) : v), braquage: borner(2.5 * ecart * sens, -bmax, bmax) };
        m.progression = borner(parcouru / Math.abs(d || 1), 0, 1);
        m.detail = `${parcouru.toFixed(1).replace('.', ',')} m sur ${Math.abs(d).toFixed(1).replace('.', ',')} m`;
        fini = reste <= 0.02;
        break;
      }
      case 'tourner':
      case 'cap': {
        const reste = c.angle - c.tourne;
        consigne = { vitesse: 0.22, braquage: Math.sign(c.angle) * bmax };
        m.progression = borner(Math.abs(c.tourne / (c.angle || 1)), 0, 1);
        m.detail = `${Math.round(Math.abs(c.tourne) / DEG)}° sur ${Math.round(Math.abs(c.angle) / DEG)}°`;
        fini = Math.abs(reste) < 2 * DEG || Math.sign(reste) !== Math.sign(c.angle);
        break;
      }
      case 'aller_a':
      case 'aller_pixel':
      case 'aller_gps':
      case 'retour_depart': {
        const [lx, ly] = versLocal(r, c.cible[0], c.cible[1]);
        const dist = Math.hypot(lx, ly);
        const alpha = Math.atan2(ly, lx);
        consigne = {
          vitesse: Math.min(vdef, 0.12 + 0.4 * dist),
          braquage: borner(Math.atan2(2 * PROFIL.geometrie.empattement_m * Math.sin(alpha), Math.max(dist, 0.6)), -bmax, bmax),
        };
        m.progression = borner(1 - dist / c.distance0, 0, 1);
        m.detail = `${dist.toFixed(1).replace('.', ',')} m restants`;
        fini = dist < (a.type === 'aller_gps' ? 0.6 : 0.15);
        break;
      }
      case 'attendre': {
        const ecoule = performance.now() / 1000 - c.debut, duree = Number(a.duree_s || 1);
        m.progression = borner(ecoule / duree, 0, 1);
        m.detail = `${Math.ceil(duree - ecoule)} s`;
        fini = ecoule >= duree;
        break;
      }
      default:
        fini = true;
    }
    // Bloqué par la sécurité : attente de 10 s puis échec.
    const bloque = !fini && Math.abs(consigne.vitesse) > 0.01 && Math.abs(r.v) < 0.01
      && (consigne.vitesse > 0 ? this.securite.vitesse_max_avant : this.securite.vitesse_max_arriere) < 0.02;
    if (bloque) {
      c.bloqueDepuis ??= performance.now() / 1000;
      const attente = performance.now() / 1000 - c.bloqueDepuis;
      m.etat = 'bloque';
      m.detail = `obstacle : ${Math.ceil(10 - attente)} s`;
      if (attente > 10) {
        this._journal('erreur', 'mission', `${a.type} : bloqué par un obstacle`);
        this._annulerMission('echec');
        return null;
      }
    } else {
      c.bloqueDepuis = null;
      if (m.etat === 'bloque') m.etat = 'route';
    }
    if (fini) {
      m.courante = null;
      m.progression = 1;
      if (!m.file.length && this.gps.boucle) {
        for (const [lat, lon] of this.gps.boucle) m.file.push(this._actionGps(lat, lon));
      } else if (!m.file.length) {
        if (a.type === 'aller_gps') this.gps.parcours = [];
        m.etat = 'termine';
        m.detail = '';
        this._journal('succes', 'mission', 'file d’actions terminée');
      }
      return { vitesse: 0, braquage: 0 };
    }
    return consigne;
  }

  // ---------------------------------------------------------------- physique et sécurité

  /** Espace libre (m) le long de l'arc, devant (sens = 1) ou derrière (−1). */
  _libre(braquage, sens) {
    const r = this.rover, g = PROFIL.geometrie;
    const demi = g.emprise_demi_largeur_m + PROFIL.securite.marge_laterale_m;
    const proches = [];
    for (let i = 0; i < POINTS_OBSTACLES.length; i += 2) {
      const dx = POINTS_OBSTACLES[i] - r.x, dy = POINTS_OBSTACLES[i + 1] - r.y;
      if (dx * dx + dy * dy < 36) proches.push(versLocal(r, POINTS_OBSTACLES[i], POINTS_OBSTACLES[i + 1]));
    }
    const k = Math.tan(braquage) / g.empattement_m;
    for (let s = 0; s <= 5; s += 0.05) {
      const pose = poseSurArc(sens * s, k);
      for (const [px, py] of proches) {
        const [qx, qy] = versLocal(pose, px, py);
        if (qx >= g.emprise_arriere_m && qx <= g.emprise_avant_m && Math.abs(qy) <= demi) return Math.max(0, s - 0.05);
      }
    }
    return 5;
  }

  _pas(dt) {
    const r = this.rover, maintenant = performance.now() / 1000;
    const bmax = PROFIL.geometrie.braquage_max_deg * DEG;
    const raisons = [];
    // Consigne selon le mode.
    let consigne = { vitesse: 0, braquage: r.braquage, source: 'aucune' };
    if (this.mode === 'manuel') {
      const frais = maintenant - this.pilotage.t < 0.5;
      if (!frais && maintenant - this.pilotage.t < 3 && (this.pilotage.vitesse || this.pilotage.braquage)) raisons.push('consigne_perimee');
      const p = frais ? this.pilotage : { vitesse: 0, braquage: 0 };
      const vmax = p.vitesse >= 0 ? this.reglages['vitesses.max_manuel_m_s'] : this.reglages['vitesses.max_arriere_m_s'];
      consigne = { vitesse: p.vitesse * vmax, braquage: p.braquage * bmax, source: 'manuel' };
    }
    const mission = this._pasMission();
    if (this.mode === 'auto' && mission) consigne = { ...mission, source: 'auto' };

    // Sécurité.
    if (this.arretUrgence) raisons.push('arret_urgence');
    if (!this.arme) raisons.push('non_arme');
    if (this.mode === 'arret') raisons.push('mode_arret');
    if (this.mode === 'banc') raisons.push('banc');
    if (!this.lidarMarche) raisons.push('lidar_absent');
    const horsZone = this.mode === 'auto' && this.gps.zone && !dansPolygone(r.x, r.y, this.gps.zone.map(([la, lo]) => versMondeGps(la, lo)));
    if (horsZone) raisons.push('geofence');
    const libreAv = this._libre(consigne.braquage, 1), libreAr = this._libre(consigne.braquage, -1);
    const s = PROFIL.securite, a = this.reglages['securite.deceleration_m_s2'], marge = this.reglages['securite.marge_avant_m'];
    const vPour = (libre) => Math.max(0, a * (-s.latence_s + Math.sqrt(s.latence_s ** 2 + (2 * Math.max(0, libre - marge)) / a)));
    const vmaxMode = this.mode === 'auto' ? this.reglages['vitesses.max_auto_m_s'] : this.reglages['vitesses.max_manuel_m_s'];
    let vmaxAv = Math.min(vmaxMode, vPour(libreAv)), vmaxAr = Math.min(this.reglages['vitesses.max_arriere_m_s'], vPour(libreAr) * 0.8);
    if (libreAv < s.distance_arret_absolu_m) vmaxAv = 0;
    if (libreAr < s.distance_arret_absolu_m) vmaxAr = 0;
    if (consigne.vitesse > 0.01 && vmaxAv === 0) raisons.push('obstacle_avant');
    else if (consigne.vitesse < -0.01 && vmaxAr === 0) raisons.push('obstacle_arriere');
    else if (consigne.vitesse > vmaxAv + 0.01 || -consigne.vitesse > vmaxAr + 0.01) raisons.push('ralenti_obstacle');
    const arret = this.arretUrgence || !this.arme || this.mode === 'arret' || this.mode === 'banc' || !this.lidarMarche || horsZone;
    this.securite = {
      arret, vitesse_max_avant: arret ? 0 : vmaxAv, vitesse_max_arriere: arret ? 0 : vmaxAr, raisons,
      libre_avant_m: Math.round(libreAv * 100) / 100, libre_arriere_m: Math.round(libreAr * 100) / 100,
    };
    this.consigne = { vitesse: consigne.vitesse, braquage: consigne.braquage, source: consigne.source };

    // Rampes et cinématique.
    const vCible = arret ? 0 : borner(consigne.vitesse, -vmaxAr, vmaxAv);
    const rampe = Math.abs(vCible) > Math.abs(r.v) && Math.sign(vCible || 1) === Math.sign(r.v || vCible || 1) ? 0.6 : 1.5;
    const ecartV = vCible - r.v;
    const vAvant = r.v;
    r.v += Math.sign(ecartV) * Math.min(Math.abs(ecartV), rampe * dt);
    const bCible = this.mode === 'banc' && maintenant - this.banc.t < 0.5 ? (this.banc.servo_deg * DEG) : borner(consigne.braquage, -bmax, bmax);
    const ecartB = bCible - r.braquage;
    r.braquage += Math.sign(ecartB) * Math.min(Math.abs(ecartB), 2.6 * dt);
    r.omega = (r.v * Math.tan(r.braquage)) / PROFIL.geometrie.empattement_m;
    r.accel = (r.v - vAvant) / dt;
    const libreSens = r.v >= 0 ? libreAv : libreAr;
    if (Math.abs(r.v * dt) > libreSens) r.v = 0; // jamais de collision dans la démo
    this.gps.parcouru += Math.abs(r.v * dt);
    if (this.gps.parcouru > 3) this.gps.aligne = true;
    r.x += r.v * Math.cos(r.cap) * dt;
    r.y += r.v * Math.sin(r.cap) * dt;
    r.cap = normaliser(r.cap + r.omega * dt);
    const c = this.mission.courante;
    if (c && (c.action.type === 'tourner' || c.action.type === 'cap')) c.tourne += r.omega * dt;

    // Commande appliquée (PWM, différentiel, servo).
    if (this.mode === 'banc') {
      const frais = maintenant - this.banc.t < 0.5;
      this.commande = { vitesse: 0, braquage: r.braquage, pwm_g: frais ? this.banc.pwm_g : 0, pwm_d: frais ? this.banc.pwm_d : 0, servo_deg: r.braquage / DEG };
    } else {
      const pwm = r.v * this.reglages['traction.pwm_par_m_s'];
      const diff = Math.tan(r.braquage) * PROFIL.geometrie.voie_arriere_m / (2 * PROFIL.geometrie.empattement_m);
      this.commande = {
        vitesse: r.v, braquage: r.braquage, servo_deg: r.braquage / DEG * this.reglages['direction.servo_par_degre_roue'],
        pwm_g: Math.round(pwm * (1 - diff)), pwm_d: Math.round(pwm * (1 + diff)),
      };
    }
    const [ox, oy] = versLocal(this.origine, r.x, r.y);
    const dernier = this.trajectoire[this.trajectoire.length - 1];
    if (Math.hypot(ox - dernier[0], oy - dernier[1]) > 0.05) {
      this.trajectoire.push([Math.round(ox * 100) / 100, Math.round(oy * 100) / 100]);
      if (this.trajectoire.length > 3000) this.trajectoire.shift();
    }
    this._pasVoix(maintenant);
  }

  // ---------------------------------------------------------------- messages périodiques

  _envoyerEtat() {
    const r = this.rover, t = tMono(), odom = this._poseOdom();
    const m = this.mission;
    const [lat, lon] = versLatLon(r.x + this.gps.biais[0], r.y + this.gps.biais[1]);
    const capteurs = {
      lidar: { ok: this.lidarMarche, hz: this.lidarMarche ? 10 + bruit(0.3) : 0, points: this.lidarMarche ? 352 + Math.round(bruit(6)) : 0, age_s: this.lidarMarche ? 0.03 + Math.random() * 0.06 : 9, marche: this.lidarMarche },
      gauche: { ok: true, ips: 14.8 + bruit(0.3), age_s: 0.04 + Math.random() * 0.05, resolution: [1280, 720] },
      droite: { ok: true, ips: 14.9 + bruit(0.3), age_s: 0.04 + Math.random() * 0.05, resolution: [1280, 720] },
      stereo: { ok: Boolean(this.reglages['perception.stereo_active']), hz: 3.9 + bruit(0.2), age_s: 0.2 + Math.random() * 0.1, calibree: this.stereoCalibree ?? false },
      imu: { ok: true, hz: 50 + bruit(1), age_s: 0.02 }, gnss: { ok: true, hz: 1, age_s: 0.3 + Math.random() * 0.6 },
      esp: { ok: true, hz: 20 + bruit(0.5), age_s: 0.03 + Math.random() * 0.03 }, micro: { ok: true, nom: 'UGREEN 15728' },
      odometrie: { ok: true, duree_ms: round(6 + Math.random() * 4, 1) },
    };
    if (!capteurs.stereo.ok) capteurs.stereo = { ...capteurs.stereo, hz: 0 };
    const action = m.courante ? Object.fromEntries(Object.entries(m.courante.action).filter(([k]) => k !== 'cible')) : null;
    this._emettre({
      type: 'etat', t,
      mode: this.mode, arme: this.arme, arret_urgence: this.arretUrgence, raison_urgence: this.arretUrgence ? 'poste' : null, pilote: 'poste-mock',
      consigne: { vitesse: round(this.consigne.vitesse, 3), braquage: round(this.consigne.braquage, 3), source: this.consigne.source },
      commande: { ...this.commande, vitesse: round(this.commande.vitesse, 3), braquage: round(this.commande.braquage, 3), servo_deg: round(this.commande.servo_deg, 1) },
      odom: { x: round(odom.x, 3), y: round(odom.y, 3), cap: round(odom.cap, 4), vitesse: round(r.v + bruit(0.005), 3), vitesse_lacet: round(r.omega, 3), qualite: round(0.88 + bruit(0.06), 2) },
      securite: { ...this.securite, vitesse_max_avant: round(this.securite.vitesse_max_avant, 2), vitesse_max_arriere: round(this.securite.vitesse_max_arriere, 2) },
      esp: { connecte: true, firmware: 'ROVER_3', defaut: false, raison_defaut: null, pwm_g: Math.abs(this.commande.pwm_g), pwm_d: Math.abs(this.commande.pwm_d), sens_g: this.commande.pwm_g < 0 ? 1 : 0, sens_d: this.commande.pwm_d < 0 ? 1 : 0, frein: false, servo_actif: true, servo_deg: round(this.commande.servo_deg, 1), chien_de_garde_ms: 400, age_s: round(capteurs.esp.age_s, 3) },
      imu: { etat: 'OK', accel_g: [round(r.accel / 9.81 + bruit(0.01), 3), round((r.v * r.omega) / 9.81 + bruit(0.01), 3), round(1 + bruit(0.01), 3)], gyro_rad_s: [round(bruit(0.01), 3), round(bruit(0.01), 3), round(r.omega + bruit(0.008), 3)], roulis_deg: round(-1.1 + bruit(0.2), 1), tangage_deg: round(0.6 + bruit(0.2), 1), temperature_c: 31.2 },
      gnss: { fix: 'autonome', latitude: round(lat, 7), longitude: round(lon, 7), altitude_m: 12.0, hdop: 1.1, satellites: 7, vitesse_m_s: round(Math.abs(r.v), 2), cap_deg: round(((90 - r.cap / DEG) % 360 + 360) % 360, 0) },
      capteurs,
      mission: { etat: m.pause && m.courante ? 'pause' : m.etat, action, progression: round(m.progression, 3), detail: m.detail, file: m.file.map((a) => Object.fromEntries(Object.entries(a).filter(([k]) => k !== 'cible'))), plan_id: m.courante ? m.plan_id : null },
      voix: { active: this.voix.active, parle: this.voix.parle, niveau: round(this.voix.niveau, 3) },
      ia: { occupee: this.iaOccupee > 0 },
      simulation: { x: round(r.x, 3), y: round(r.y, 3), cap: round(r.cap, 4), vitesse: round(r.v, 3) },
    });
  }

  _envoyerScan() {
    if (!this.lidarMarche) return;
    const r = this.rover, xy = [];
    this.libre = [];
    for (let i = 0; i < 360; i++) {
      const a = (i - 180) * DEG;
      const d = lancer(r.x, r.y, r.cap + a);
      if (i % 6 === 0) this.libre.push(versParent(r, Math.min(d, 8) * Math.cos(a), Math.min(d, 8) * Math.sin(a)));
      if (d >= 12 || d < 0.12 || Math.random() < 0.02) continue;
      const dd = d + bruit(0.01);
      xy.push(Math.round(dd * Math.cos(a) * 100), Math.round(dd * Math.sin(a) * 100));
      // Grille d'occupation : un rayon sur deux.
      if (i % 2 === 0) this._tracerRayon(r.x, r.y, r.cap + a, d, true);
    }
    this._emettre({ type: 'scan', t: tMono(), xy });
    this._envoyerObstacles();
  }

  _envoyerObstacles() {
    if (!this.reglages['perception.stereo_active']) return;
    const r = this.rover, cam = versParent(r, PROFIL.cameras.x_m, 0), xy = [];
    const demiChamp = (PROFIL.cameras.hfov_deg / 2) * DEG;
    for (const b of [...BOITES, ...POTEAUX.map((p) => ({ x: p.x, y: p.y, l: p.r * 2, w: p.r * 2 }))]) {
      for (const [x0, y0, x1, y1] of aretesBoite(b)) {
        const n = Math.max(1, Math.ceil(Math.hypot(x1 - x0, y1 - y0) / 0.06));
        for (let i = 0; i < n; i++) {
          const px = x0 + ((x1 - x0) * i) / n, py = y0 + ((y1 - y0) * i) / n;
          const dist = Math.hypot(px - cam[0], py - cam[1]);
          if (dist > 4) continue;
          const angle = normaliser(Math.atan2(py - cam[1], px - cam[0]) - r.cap);
          if (Math.abs(angle) > demiChamp) continue;
          if (lancer(cam[0], cam[1], r.cap + angle, SEGMENTS_TOUS, dist + 1) < dist - 0.04) continue;
          const [bx, by] = versLocal(r, px, py);
          xy.push(Math.round(bx * 100), Math.round(by * 100));
          if (b.bas) this._marquerOccupe(px, py);
        }
      }
    }
    this._emettre({ type: 'obstacles', t: tMono(), source: 'stereo', xy, portee_m: 4.0 });
  }

  _indice(xMonde, yMonde) {
    const [x, y] = versLocal(this.origine, xMonde, yMonde);
    const c = Math.floor((x + 20) / 0.05), l = Math.floor((y + 20) / 0.05);
    return c >= 0 && c < 800 && l >= 0 && l < 800 ? l * 800 + c : -1;
  }

  _marquerOccupe(x, y) {
    const i = this._indice(x, y);
    if (i >= 0) this.grille[i] = Math.max(0, this.grille[i] - 60);
  }

  _tracerRayon(x, y, a, d, touche) {
    const dx = Math.cos(a), dy = Math.sin(a);
    for (let s = 0; s < d - 0.05; s += 0.05) {
      const i = this._indice(x + dx * s, y + dy * s);
      if (i >= 0 && this.grille[i] < 250) this.grille[i] = Math.min(255, this.grille[i] + 22);
    }
    if (touche) this._marquerOccupe(x + dx * d, y + dy * d);
  }

  _envoyerCarte() {
    if (!this.toile) {
      this.toile = document.createElement('canvas');
      this.toile.width = 800;
      this.toile.height = 800;
      this.pixels = this.toile.getContext('2d', { willReadFrequently: true }).createImageData(800, 800);
    }
    const d = this.pixels.data;
    // Ligne 0 du PNG = y maximal.
    for (let l = 0; l < 800; l++) {
      const ligneImage = 799 - l;
      for (let c = 0; c < 800; c++) {
        const g = this.grille[l * 800 + c], k = (ligneImage * 800 + c) * 4;
        d[k] = d[k + 1] = d[k + 2] = g;
        d[k + 3] = 255;
      }
    }
    this.toile.getContext('2d').putImageData(this.pixels, 0, 0);
    const png = this.toile.toDataURL('image/png').split(',')[1];
    const c = this.mission.courante;
    let chemin = [], but = null;
    if (c?.cible) {
      const o = this._poseOdom();
      const [bx, by] = versLocal(this.origine, c.cible[0], c.cible[1]);
      chemin = [[round(o.x, 2), round(o.y, 2)], [round(bx, 2), round(by, 2)]];
      but = [round(bx, 2), round(by, 2)];
    }
    this._emettre({ type: 'carte', t: tMono(), resolution: 0.05, origine: [-20.0, -20.0], largeur: 800, hauteur: 800, png, trajectoire: this.trajectoire.slice(), chemin, but });
  }

  /** Monde reconstruit pour la vue 3D (PROTOCOLE.md, `scene`), repère odom. */
  _envoyerScene() {
    const r = this.rover, g = PROFIL.geometrie;
    const odom = (x, y) => versLocal(this.origine, x, y).map((v) => round(v, 2));
    const cap = (c) => round(normaliser(c - this.origine.cap), 3);
    const age = (x, y) => round(Math.max(0, (Math.hypot(x - r.x, y - r.y) - 8) * 2), 1);
    const libreAvant = this.securite.libre_avant_m ?? 5;
    const danger = (x, y, rayon) => {
      const [bx, by] = versLocal(r, x, y);
      return bx > 0 && bx < g.emprise_avant_m + Math.min(libreAvant, 2) + rayon + 0.3 && Math.abs(by) < g.emprise_demi_largeur_m + 0.1 + rayon;
    };
    const objets = [];
    let id = 1;
    for (const [x0, y0, x1, y1] of MURS) {
      const longueur = Math.hypot(x1 - x0, y1 - y0), n = Math.ceil(longueur / 2);
      for (let i = 0; i < n; i++) {
        const cx = x0 + ((x1 - x0) * (i + 0.5)) / n, cy = y0 + ((y1 - y0) * (i + 0.5)) / n;
        const [x, y] = odom(cx, cy);
        const vu = Math.hypot(cx - r.x, cy - r.y) < 9;
        objets.push({ id: id++, type: 'mur', x, y, cap: cap(Math.atan2(y1 - y0, x1 - x0)), longueur: round(longueur / n, 2), epaisseur: 0.1, hauteur: HAUTEUR_MUR, age_s: vu ? 0 : 6, extrapole: !vu, danger: false });
      }
    }
    for (const b of BOITES) {
      const [x, y] = odom(b.x, b.y);
      objets.push({ id: 100 + id++, type: b.bas ? 'bas' : 'bloc', x, y, cap: cap(0), longueur: b.l, largeur: b.w, hauteur: b.h, age_s: age(b.x, b.y), danger: danger(b.x, b.y, Math.max(b.l, b.w) / 2) });
    }
    for (const p of POTEAUX) {
      const [x, y] = odom(p.x, p.y);
      objets.push({ id: 200 + id++, type: 'arbre', x, y, rayon: p.r, hauteur: p.h, age_s: age(p.x, p.y), danger: danger(p.x, p.y, p.r) });
    }
    const maintenant = performance.now() / 1000;
    for (const [texte, e] of this.etiquettes) {
      if (maintenant - e.t > 30) { this.etiquettes.delete(texte); continue; }
      const [x, y] = odom(e.x, e.y);
      objets.push({ id: 300 + id++, type: 'etiquette', x, y, texte, age_s: round(maintenant - e.t, 1) });
    }
    const o = this._poseOdom();
    const c = this.mission.courante;
    let chemin = null, but = null;
    if (c?.cible) {
      but = odom(c.cible[0], c.cible[1]);
      chemin = [[round(o.x, 2), round(o.y, 2)], but];
    }
    this._emettre({
      type: 'scene', t: tMono(), repere: 'odom', rover: { x: round(o.x, 3), y: round(o.y, 3), cap: round(o.cap, 4) },
      objets, libre: this.libre.map(([x, y]) => odom(x, y)), chemin, but,
    });
  }

  /** Message `gps` (1 Hz) : brut bruité, position fusionnée une fois l'alignement fait. */
  _envoyerGps() {
    const r = this.rover, g = this.gps;
    g.biais = g.biais.map((b) => borner(b * 0.97 + bruit(0.25), -1.8, 1.8));
    const [blat, blon] = versLatLon(r.x + g.biais[0] + bruit(0.3), r.y + g.biais[1] + bruit(0.3));
    let rover = null;
    if (g.aligne) {
      const [lat, lon] = versLatLon(r.x + bruit(0.1), r.y + bruit(0.1));
      rover = { lat: round(lat, 7), lon: round(lon, 7), precision_m: round(1.4 + Math.random() * 0.4, 1), cap_vrai_deg: round((((90 - r.cap / DEG) % 360) + 360) % 360, 1) };
      const dernier = g.trace[g.trace.length - 1];
      const [dx, dy] = dernier ? versMondeGps(...dernier) : [Infinity, Infinity];
      if (Math.hypot(dx - r.x, dy - r.y) > 0.4) {
        g.trace.push([rover.lat, rover.lon]);
        if (g.trace.length > 600) g.trace.shift();
      }
    }
    const c = this.mission.courante;
    const but = c?.action.type === 'aller_gps' ? { lat: c.action.lat, lon: c.action.lon } : null;
    this._emettre({
      type: 'gps', t: tMono(), fix: 'autonome', satellites: 7 + Math.round(Math.random()), hdop: round(1.0 + Math.random() * 0.3, 1),
      brut: { lat: round(blat, 7), lon: round(blon, 7) }, rover,
      alignement: { ok: g.aligne, residu_m: g.aligne ? round(1.1 + Math.random() * 0.3, 1) : null, etendue_m: round(g.parcouru, 1), points: g.trace.length },
      reference: { lat: LAT0, lon: LON0 }, trace: g.trace.slice(-300), but, parcours: g.parcours, zone: g.zone, lieux: g.lieux,
    });
  }

  _envoyerSysteme() {
    const cpu = 32 + bruit(8) + Math.abs(this.rover.v) * 20;
    this._emettre({
      type: 'systeme', cpu_pct: Math.round(cpu), temperature_c: round(56 + cpu / 8 + bruit(0.4), 1), memoire_pct: 42, disque_pct: 31,
      sous_tension: false, ralenti: false, throttled: '0x50000',
      wifi: { ssid: 'reseau-atelier', signal_pct: Math.round(68 + bruit(5)), ip: 'rover.local' },
      charge: [round(1.2 + bruit(0.1), 2), 1.1, 0.9], uptime_s: Math.round(5321 + (performance.now() - this.debut) / 1000), processus_cpu_pct: Math.round(cpu * 1.4),
    });
  }

  _envoyerConsolePeriodique() {
    const c = this.commande;
    this._emettre({ type: 'console', lignes: [[tMur(), `T ${Math.round(performance.now()) % 100000} ${c.pwm_g} ${c.pwm_d} ${c.servo_deg.toFixed(1)} 0 0 1`]] });
  }

  _console(texte) {
    const lignes = [[tMur(), `→ ${texte}`]];
    const reponses = {
      etat: ['[FIRMWARE] ROVER_3', `[ETAT] pwm_g=${this.commande.pwm_g} pwm_d=${this.commande.pwm_d} servo=${this.commande.servo_deg.toFixed(1)} frein=0 chien=400`],
      aide: ['[AIDE] g d sensg sensd s v stop p wd hb inv tel imu etat'],
      stop: ['[MOTEURS] arrêt'],
    };
    for (const l of reponses[texte.split(/\s+/)[0]] || ['OK']) lignes.push([tMur(), l]);
    setTimeout(() => this._emettre({ type: 'console', lignes }), 60);
  }

  // ---------------------------------------------------------------- calibrations

  _calibration(m, id) {
    const noms = ['sens_roues', 'vitesse', 'braquage', 'stereo_capture', 'stereo_calculer', 'imu_repos'];
    if (!noms.includes(m.nom)) return this._repondre(id, false, { erreur: 'calibration inconnue' });
    this.calibrations ??= {};
    if (m.arreter) {
      clearTimeout(this.calibrations[m.nom]);
      delete this.calibrations[m.nom];
      this._emettre({ type: 'calibration', nom: m.nom.startsWith('stereo_') ? 'stereo' : m.nom, etat: 'echec', message: 'interrompue', resultat: null });
      return this._repondre(id, true);
    }
    if ((m.nom === 'vitesse' || m.nom === 'braquage') && !this.arme) return this._repondre(id, false, { erreur: 'armer le rover d’abord' });
    if (m.nom === 'sens_roues' && this.mode !== 'banc') return this._repondre(id, false, { erreur: 'passer en banc (roues levées)' });
    this._repondre(id, true, m.nom === 'stereo_capture' ? { valeur: this.stereoPaires + 1 } : {});
    const etapes = {
      sens_roues: [['roue gauche…', 1200], ['roue droite…', 1200], [null, 0, { inverse_g: false, inverse_d: true }]],
      vitesse: [['PWM 10 %…', 1200], ['PWM 20 %…', 1200], ['PWM 30 %…', 1200], [null, 0, { pwm_par_m_s: 36.4, seuil_pct: 5, points: 3 }]],
      braquage: [['butée gauche…', 1500], ['butée droite…', 1500], [null, 0, { servo_par_degre_roue: 1.08, rayon_min_m: 0.66 }]],
      imu_repos: [['immobile 3 s…', 1800], [null, 0, { biais_gyro_rad_s: [0.002, -0.001, 0.004], gravite_g: 1.002 }]],
      stereo_capture: [[null, 0, null]],
      stereo_calculer: [['calcul…', 1600], [null, 0, null]],
    }[m.nom];
    const nomAnnonce = m.nom.startsWith('stereo_') ? 'stereo' : m.nom;
    const suite = (i) => {
      const [message, delai, resultat] = etapes[i];
      if (message !== null) {
        this._emettre({ type: 'calibration', nom: nomAnnonce, etat: 'en_cours', message, resultat: null });
        this.calibrations[m.nom] = setTimeout(() => suite(i + 1), delai);
        return;
      }
      delete this.calibrations[m.nom];
      if (m.nom === 'stereo_capture') {
        this.stereoPaires++;
        this._emettre({ type: 'calibration', nom: 'stereo', etat: 'en_cours', message: `mire trouvée (${this.stereoPaires} paires)`, resultat: { paires: this.stereoPaires } });
      } else if (m.nom === 'stereo_calculer') {
        if (this.stereoPaires < 5) {
          this._emettre({ type: 'calibration', nom: 'stereo', etat: 'echec', message: `5 paires nécessaires (${this.stereoPaires})`, resultat: null });
        } else {
          this.stereoCalibree = true;
          this._emettre({ type: 'calibration', nom: 'stereo', etat: 'termine', message: 'calibration enregistrée', resultat: { erreur_rms_px: 0.42, base_m: 0.1203, hfov_deg: 77.1 } });
        }
      } else {
        this._emettre({ type: 'calibration', nom: m.nom, etat: 'termine', message: 'enregistré', resultat });
        this._journal('succes', 'calibration', `${m.nom} terminée`);
      }
    };
    suite(0);
  }

  // ---------------------------------------------------------------- agent, vision, voix

  _consigne(texte, origine) {
    if (origine === 'voix') this._chat('operateur', texte, { origine: 'voix' });
    else this._chat('operateur', texte);
    if (this.ia.profil === 'aucun') {
      setTimeout(() => this._chat('systeme', 'IA désactivée.'), 200);
      return;
    }
    this.iaOccupee++;
    setTimeout(() => {
      this.iaOccupee--;
      const compris = comprendre(texte);
      if (compris.stop) {
        this._annulerMission('annule');
        this._chat('rover', compris.reponse, { origine });
        return;
      }
      if (!compris.actions.length) {
        this._chat('rover', compris.reponse, { origine });
        return;
      }
      const plan = { id: `p${this.compteurs.plan++}`, actions: compris.actions };
      this._chat('rover', compris.reponse, { origine, plan });
      this.mission.plan_id = plan.id;
      for (const a of compris.actions) this.mission.file.push(this._preparerAction(a));
      if (this.arme) this.mode = 'auto';
      else this._chat('systeme', 'Rover désarmé : plan en attente.');
    }, 900 + Math.random() * 700);
  }

  _visibles(camera) {
    const r = this.rover;
    const modele = new ModeleCamera(PROFIL.cameras, camera === 'droite' ? -1 : 1);
    const camMonde = versParent(r, modele.x, modele.y);
    const objets = [];
    for (const o of [...BOITES, ...POTEAUX.map((p) => ({ ...p, l: p.r * 2, w: p.r * 2 }))]) {
      const [bx, by] = versLocal(r, o.x, o.y);
      const centre = modele.versPixel(modele.versCamera(bx, by, Math.min(o.h, 0.9) / 2));
      if (!centre || centre[0] < 0.02 || centre[0] > 0.98 || centre[1] < 0 || centre[1] > 1) continue;
      const dist = Math.hypot(o.x - camMonde[0], o.y - camMonde[1]);
      const libre = lancer(camMonde[0], camMonde[1], Math.atan2(o.y - camMonde[1], o.x - camMonde[0]), SEGMENTS_TOUS.filter((s) => !aretesBoite(o).some((a) => a.join() === s.join())), dist + 1);
      if (libre < dist - Math.max(o.l, o.w)) continue;
      const coins = [];
      for (const dx of [-o.l / 2, o.l / 2]) for (const dy of [-o.w / 2, o.w / 2]) for (const z of [0, o.h]) {
        const [cx, cy] = versLocal(r, o.x + dx, o.y + dy);
        const p = modele.versPixel(modele.versCamera(cx, cy, z));
        if (p) coins.push(p);
      }
      const us = coins.map((p) => p[0]), vs = coins.map((p) => p[1]);
      // Point visé : bas de l'objet, face au rover (point au sol pour « sol »).
      const [sx, sy] = versLocal(r, o.x - (o.l / 2) * Math.cos(r.cap), o.y - (o.w / 2) * Math.sin(r.cap));
      const pied = modele.versPixel(modele.versCamera(sx, sy, 0));
      objets.push({
        o, dist, centre,
        boite: { u0: round(borner(Math.min(...us), 0, 1), 3), v0: round(borner(Math.min(...vs), 0, 1), 3), u1: round(borner(Math.max(...us), 0, 1), 3), v1: round(borner(Math.max(...vs), 0, 1), 3), label: o.nom },
        point: pied ? { u: round(borner(pied[0], 0, 1), 3), v: round(borner(pied[1], 0, 1), 3), label: o.nom, sol: [round(sx, 2), round(sy, 2)] } : null,
      });
    }
    return objets.sort((a, b) => a.dist - b.dist);
  }

  _vision(demande, camera, objet, depuisMission = false) {
    this.iaOccupee++;
    const debut = performance.now();
    const toile = document.createElement('canvas');
    toile.width = 640;
    toile.height = 360;
    rendreCamera(toile.getContext('2d'), 640, 360, this.rover, camera);
    const image = toile.toDataURL('image/jpeg', 0.75).split(',')[1];
    const objets = this._visibles(camera);
    setTimeout(() => {
      this.iaOccupee--;
      const position = (x) => (x.centre[0] < 0.4 ? 'à gauche' : x.centre[0] > 0.6 ? 'à droite' : 'devant');
      const distance = (x) => `${x.dist.toFixed(1).replace('.', ',')} m`;
      let description, points = [], boites = [];
      if (demande === 'chercher') {
        const mots = String(objet || '').toLowerCase().split(/\s+/).filter((m) => m.length > 2);
        const trouve = objets.find((x) => mots.some((m) => x.o.nom.includes(m)));
        if (trouve) {
          description = `${trouve.o.nom.charAt(0).toUpperCase() + trouve.o.nom.slice(1)} ${position(trouve)}, à ${distance(trouve)}.`;
          if (trouve.point) points = [trouve.point];
          boites = [trouve.boite];
          if (depuisMission && trouve.point?.sol) {
            const [x, y] = versParent(this.rover, trouve.point.sol[0] - 0.5, trouve.point.sol[1]);
            this.mission.file.unshift({ type: 'aller_a', x: round(versLocal(this.origine, x, y)[0], 2), y: round(versLocal(this.origine, x, y)[1], 2), repere: 'odom' });
          }
        } else {
          description = `Je ne vois pas « ${objet} ».`;
        }
      } else {
        description = objets.length
          ? `Je vois ${objets.slice(0, 3).map((x) => `${x.o.nom.startsWith('marche') ? 'une marche basse' : `un${/^(caisse|chaise|table)/.test(x.o.nom) ? 'e' : ''} ${x.o.nom}`} ${position(x)} à ${distance(x)}`).join(', ')}.`
          : 'Je vois l’herbe et la clôture du jardin.';
        boites = objets.slice(0, 4).map((x) => x.boite);
        points = objets.slice(0, 4).map((x) => x.point).filter(Boolean);
      }
      const nommes = demande === 'chercher' ? objets.filter((x) => boites.some((b) => b.label === x.o.nom)) : objets.slice(0, 4);
      for (const x of nommes) this.etiquettes.set(x.o.nom, { x: x.o.x, y: x.o.y, t: performance.now() / 1000 });
      const duree = (performance.now() - debut) / 1000;
      this._emettre({ type: 'vision', id: `v${this.compteurs.vision++}`, t: tMur(), camera, demande, objet: objet || null, description, image, points, boites, modele: this.ia.modeles.vision || '—', duree_s: round(duree, 1) });
      this._chat('rover', description);
    }, 1400 + Math.random() * 900);
  }

  _pasVoix(maintenant) {
    const v = this.voix;
    if (!v.active) { v.niveau = 0; v.parle = false; return; }
    v.niveau = v.parle ? 0.25 + Math.random() * 0.35 : 0.01 + Math.random() * 0.04;
    if (!v.parle && maintenant > v.prochaine) {
      v.parle = true;
      v.finParole = maintenant + 1.6;
      this._emettre({ type: 'voix', evenement: 'debut_parole', t: tMur() });
    } else if (v.parle && maintenant > v.finParole) {
      v.parle = false;
      v.prochaine = maintenant + 14;
      this._emettre({ type: 'voix', evenement: 'fin_parole', t: tMur() });
      const phrases = ['que vois-tu', 'c’est bon merci', 'tourne à gauche'];
      const phrase = phrases[v.phrase++ % phrases.length];
      setTimeout(() => {
        if (phrase.startsWith('c’est')) {
          this._emettre({ type: 'voix', evenement: 'ignore', texte: phrase, t: tMur() });
          return;
        }
        this._emettre({ type: 'voix', evenement: 'transcription', texte: phrase, t: tMur() });
        if (phrase === 'que vois-tu') {
          this._chat('operateur', phrase, { origine: 'voix' });
          this._vision('decrire', 'gauche');
        } else {
          this._consigne(phrase, 'voix');
        }
      }, 700);
    }
  }
}

function round(v, n) {
  if (!Number.isFinite(v)) return v;
  const f = 10 ** n;
  return Math.round(v * f) / f;
}

