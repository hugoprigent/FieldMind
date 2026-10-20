// Calculs purs (sans DOM) : profil par défaut, arcs Ackermann, modèle sténopé, décodage des
// scans. Repère base_link : x avant, y gauche, origine au sol sur l'essieu arrière.

export const DEG = Math.PI / 180;

/** Profil de secours (config/rover.toml) tant que `bienvenue.profil` n'est pas reçu. */
export const PROFIL_DEFAUT = {
  geometrie: {
    empattement_m: 0.38, voie_arriere_m: 0.39, voie_avant_m: 0.31, braquage_max_deg: 31.0,
    emprise_arriere_m: -0.11, emprise_avant_m: 0.46, emprise_demi_largeur_m: 0.225,
  },
  lidar: { modele: 'RPLIDAR C1', x_m: 0, y_m: 0, z_m: 0.322, portee_max_m: 12 },
  cameras: {
    largeur: 1280, hauteur: 720, ips: 15, base_m: 0.12, x_m: 0.415, z_m: 0.265,
    inclinaison_deg: 10.0, hfov_deg: 78.0,
  },
  vitesses: { max_manuel_m_s: 0.8, max_auto_m_s: 0.5, max_arriere_m_s: 0.3, defaut_action_m_s: 0.35 },
  securite: {
    deceleration_m_s2: 0.8, latence_s: 0.25, marge_avant_m: 0.25, marge_laterale_m: 0.10,
    distance_arret_absolu_m: 0.30,
  },
  traction: { pwm_max_banc: 15 },
  direction: { servo_max_deg: 40 },
};

/** Profil reçu complété par les valeurs par défaut (section par section). */
export function completerProfil(profil) {
  const sortie = {};
  for (const [section, valeurs] of Object.entries(PROFIL_DEFAUT)) {
    sortie[section] = { ...valeurs, ...(profil?.[section] || {}) };
  }
  for (const [section, valeurs] of Object.entries(profil || {})) {
    if (!(section in sortie)) sortie[section] = valeurs;
  }
  return sortie;
}

export function borner(v, min, max) {
  return Math.min(max, Math.max(min, v));
}

// ------------------------------------------------------------------ arcs (modèle bicyclette)

export function courbure(braquage, empattement) {
  return Math.tan(braquage) / empattement;
}

/** Pose de l'essieu arrière après une abscisse curviligne `s` (m, signée) sur l'arc `k`. */
export function poseSurArc(s, k) {
  if (Math.abs(k) < 1e-6) return { x: s, y: 0, cap: 0 };
  const a = k * s;
  return { x: Math.sin(a) / k, y: (1 - Math.cos(a)) / k, cap: a };
}

export function versParent(pose, x, y) {
  const c = Math.cos(pose.cap), s = Math.sin(pose.cap);
  return [pose.x + c * x - s * y, pose.y + s * x + c * y];
}

export function versLocal(pose, x, y) {
  const dx = x - pose.x, dy = y - pose.y;
  const c = Math.cos(pose.cap), s = Math.sin(pose.cap);
  return [c * dx + s * dy, -s * dx + c * dy];
}

/**
 * Enveloppe balayée par les coins avant (ou arrière, `sens` = −1) du rover le long de l'arc
 * prévu : deux polylignes (gauche, droite) dans base_link, échantillonnées tous les `pas` m.
 */
export function enveloppe({ braquage, empattement, xBord, demiLargeur, longueur, sens = 1, pas = 0.05 }) {
  const k = courbure(braquage, empattement);
  const n = Math.max(2, Math.ceil(Math.abs(longueur) / pas));
  const gauche = [], droite = [];
  for (let i = 0; i <= n; i++) {
    const pose = poseSurArc((sens * Math.abs(longueur) * i) / n, k);
    gauche.push(versParent(pose, xBord, demiLargeur));
    droite.push(versParent(pose, xBord, -demiLargeur));
  }
  return { gauche, droite, k };
}

/** Segment transversal (coin gauche → coin droit) après `s` m parcourus le long de l'arc. */
export function traverse({ braquage, empattement, xBord, demiLargeur, s }) {
  const pose = poseSurArc(s, courbure(braquage, empattement));
  return [versParent(pose, xBord, demiLargeur), versParent(pose, xBord, -demiLargeur)];
}

/** Longueur d'arc affichée : au moins 1,5 m pour voir le braquage, ~3 s de route sinon. */
export function longueurPrevue(vitesse) {
  return borner(1.0 + 3 * Math.abs(vitesse || 0), 1.5, 4.0);
}

/** Emprise du rover (rectangle) dans base_link, sens trigonométrique. */
export function emprise(geometrie) {
  const a = geometrie.emprise_arriere_m, f = geometrie.emprise_avant_m, w = geometrie.emprise_demi_largeur_m;
  return [[f, w], [a, w], [a, -w], [f, -w]];
}

// ------------------------------------------------------------------ caméra (sténopé, sol plan)

/**
 * Modèle sténopé d'une caméra de la barre stéréo. Coordonnées image normalisées : u de 0 (gauche)
 * à 1 (droite), v de 0 (haut) à 1 (bas). `cote` = +1 pour la caméra gauche (y > 0), −1 droite.
 */
export class ModeleCamera {
  constructor({ x_m, z_m, base_m = 0, inclinaison_deg, hfov_deg, largeur = 16, hauteur = 9 }, cote = 0) {
    this.x = x_m;
    this.y = (cote * base_m) / 2;
    this.z = z_m;
    this.t = inclinaison_deg * DEG;
    this.c = Math.cos(this.t);
    this.s = Math.sin(this.t);
    this.fx = 0.5 / Math.tan((hfov_deg * DEG) / 2); // focale / largeur
    this.fy = (this.fx * largeur) / hauteur; // focale / hauteur (pixels carrés)
  }

  /** Point de base_link → repère caméra [droite, bas, avant]. */
  versCamera(x, y, z = 0) {
    const dx = x - this.x, dy = y - this.y, dz = z - this.z;
    return [-dy, -this.s * dx - this.c * dz, this.c * dx - this.s * dz];
  }

  /** Repère caméra → [u, v] normalisés, ou null derrière l'objectif. */
  versPixel([X, Y, Z]) {
    if (Z <= 1e-6) return null;
    return [0.5 + (this.fx * X) / Z, 0.5 + (this.fy * Y) / Z];
  }

  solVersPixel(x, y) {
    return this.versPixel(this.versCamera(x, y, 0));
  }

  /** Pixel normalisé → point du sol [x, y] dans base_link, ou null au-dessus de l'horizon. */
  pixelVersSol(u, v, portee = 30) {
    const X = (u - 0.5) / this.fx, Y = (v - 0.5) / this.fy;
    const dx = this.c - Y * this.s, dy = -X, dz = -this.s - Y * this.c;
    if (dz >= -1e-6) return null;
    const t = -this.z / dz;
    const x = this.x + t * dx, y = this.y + t * dy;
    if (Math.hypot(x - this.x, y - this.y) > portee) return null;
    return [x, y];
  }

  /** Ligne d'horizon (v normalisé) d'un sol plan. */
  horizon() {
    return 0.5 - this.fy * Math.tan(this.t);
  }

  /**
   * Polyligne de base_link (z = 0 par défaut) → liste de tronçons [[u, v], …] en coordonnées
   * normalisées, coupés au plan proche (la partie derrière l'objectif disparaît).
   */
  projeterPolyligne(points, z = 0, proche = 0.05) {
    const troncons = [];
    let courant = null, precedent = null;
    for (const point of points) {
      const p = this.versCamera(point[0], point[1], point.length > 2 ? point[2] : z);
      const devant = p[2] > proche;
      if (precedent && (precedent[2] > proche) !== devant) {
        const r = (proche - precedent[2]) / (p[2] - precedent[2]);
        const coupe = precedent.map((a, i) => a + r * (p[i] - a));
        if (devant) {
          courant = [this.versPixel(coupe)];
        } else {
          courant.push(this.versPixel(coupe));
          troncons.push(courant);
          courant = null;
        }
      }
      if (devant) {
        if (!courant) courant = [];
        courant.push(this.versPixel(p));
      }
      precedent = p;
    }
    if (courant) troncons.push(courant);
    return troncons.filter((t) => t.length > 1);
  }
}

/** Côté de la caméra dans base_link : +1 gauche (y > 0), −1 droite. La profondeur suit la gauche. */
export function coteCamera(nom) {
  return nom === 'droite' ? -1 : 1;
}

/** Rectangle occupé par une image `l`×`h` dans une boîte `L`×`H` en `object-fit: contain`. */
export function cadreContenu(L, H, l, h) {
  if (!l || !h || !L || !H) return { x: 0, y: 0, l: L, h: H };
  const e = Math.min(L / l, H / h);
  const lc = l * e, hc = h * e;
  return { x: (L - lc) / 2, y: (H - hc) / 2, l: lc, h: hc };
}

// ------------------------------------------------------------------ scans

/** `xy` aplati en centimètres entiers → Float32Array aplatie en mètres. */
export function decoderXY(xy) {
  if (!Array.isArray(xy) && !(xy instanceof Int16Array) && !(xy instanceof Int32Array)) return new Float32Array(0);
  const n = xy.length - (xy.length % 2);
  const sortie = new Float32Array(n);
  for (let i = 0; i < n; i++) sortie[i] = xy[i] / 100;
  return sortie;
}

/** Plus proche point (m) dans un couloir droit devant (|y| ≤ demi-largeur, x ≥ xMin). */
export function plusProcheDevant(points, xMin, demiLargeur) {
  let d = Infinity;
  for (let i = 0; i < points.length; i += 2) {
    const x = points[i], y = points[i + 1];
    if (x >= xMin && Math.abs(y) <= demiLargeur) d = Math.min(d, x - xMin);
  }
  return d;
}
