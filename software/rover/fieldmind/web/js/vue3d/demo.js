// Démonstration autonome de la vue 3D (vue3d.html) : un jardin planté d'arbres, une maison
// avec un couloir et une pièce meublée. Le rover suit une boucle (poursuite pure, modèle
// bicyclette) ; une perception factice produit les messages du protocole (`scene` 4 Hz,
// `etat` 10 Hz, `scan` et `obstacles` 5 Hz) : les objets apparaissent quand le lidar les voit,
// vieillissent quand il ne les voit plus, les murs vus sont prolongés (`extrapole`), une poutre
// basse dans le couloir arrête le rover (trajectoire rouge, `danger`), la vision IA nomme une
// chaise ; une personne traverse devant le rover, un chien tourne autour d'un rocher, un enfant
// attend dans la pièce (détection caméra, oubliés 5 s après la dernière vue). Paramètres d'URL : `?t=40` démarre 40 s plus loin ; `?camera=dessus` ;
// `?poussiere=1` ; `?ips=1` affiche les images par seconde ; `?ws=1` (ou `?ws=ws://hote:port/ws`)
// remplace la démo par la WebSocket d'un vrai serveur, `&conduire=1` fait rouler un rover
// simulé (refusé si le serveur n'est pas en simulation).

import { Vue3D } from './vue3d.js';

const DEG = Math.PI / 180;
const PROFIL = {
  geometrie: {
    empattement_m: 0.38, voie_arriere_m: 0.39, voie_avant_m: 0.31, diametre_roue_arriere_m: 0.2,
    diametre_roue_avant_m: 0.15, braquage_max_deg: 31, emprise_arriere_m: -0.11, emprise_avant_m: 0.46,
    emprise_demi_largeur_m: 0.225, hauteur_max_m: 0.36,
  },
  lidar: { x_m: 0, y_m: 0, z_m: 0.322, portee_max_m: 12 },
  cameras: { base_m: 0.12, x_m: 0.415, z_m: 0.265, inclinaison_deg: 10, hfov_deg: 78 },
};

// ------------------------------------------------------------------ monde

function aleatoire(graine) {
  return () => {
    graine |= 0;
    graine = (graine + 0x6d2b79f5) | 0;
    let t = Math.imul(graine ^ (graine >>> 15), 1 | graine);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Boucle du rover : jardin → porte → couloir (poutre) → pièce → couloir → jardin. */
const ROUTE = [
  [0, 0], [2.5, 1.2], [5, -1.0], [7.5, 0.6], [10, 0], [12.6, 0], [14.15, 0], [15.0, -0.45],
  [16.3, -0.25], [17.8, 0.3], [19.6, 0.9], [21, 0.1], [20.6, -1.7], [19.0, -2.3], [17.7, -1.0],
  [16.6, -0.45], [15.0, -0.45], [13.6, -0.2], [11.2, 0], [9, 3.2], [6, 4.5], [2.5, 4.2],
  [-1.5, 3], [-3.6, 0.6], [-2.8, -2.4], [-0.9, -1.1],
];
const BUTS = new Set([5, 11, 18, 25]); // porte, fond de la pièce, sortie, retour au départ
const ARRET_POUTRE = { x0: 13.75, x1: 14.3 };

/** Êtres vivants en mouvement : position (m, monde) en fonction du temps (s). */
const VIVANTS = [
  { id: 1000, type: 'personne', texte: 'personne', hauteur: 1.72, rayon: 0.25, position: (t) => {
    const p = (t * 0.7 + 9.1) % 13; // va-et-vient : traverse devant le rover vers 10 s
    return [6.6, p < 6.5 ? -3.4 + p : 3.1 - (p - 6.5)];
  } },
  { id: 1001, type: 'animal', texte: 'chien', hauteur: 0.55, rayon: 0.25, position: (t) => [3.2 + 1.6 * Math.cos(t * 0.55), -2.6 + 1.1 * Math.sin(t * 0.55)] },
  { id: 1002, type: 'personne', texte: 'enfant', hauteur: 1.2, rayon: 0.2, position: (t) => [20.4 + 0.5 * Math.sin(t * 0.3), 1.55] },
];

function construireMonde() {
  const rnd = aleatoire(7);
  const murs = [];
  const mur = (x0, y0, x1, y1, h, e = 0.12) => murs.push({ x0, y0, x1, y1, h, e, id: murs.length });
  // Clôture du jardin (basse).
  mur(-6, -7, 11.9, -7, 0.85, 0.08);
  mur(-6, 7, 11.9, 7, 0.85, 0.08);
  mur(-6, -7, -6, 7, 0.85, 0.08);
  // Maison.
  mur(12, -5, 12, -0.75, 2.2, 0.15);
  mur(12, 0.75, 12, 5, 2.2, 0.15);
  mur(12, -5, 22, -5, 2.2, 0.15);
  mur(12, 5, 22, 5, 2.2, 0.15);
  mur(22, -5, 22, 5, 2.2, 0.15);
  // Couloir et cloisons des chambres.
  mur(12, 0.95, 17, 0.95, 2.2, 0.1);
  mur(12, -0.95, 17, -0.95, 2.2, 0.1);
  mur(17, 0.95, 17, 5, 2.2, 0.1);
  mur(17, -5, 17, -0.95, 2.2, 0.1);

  const blocs = [];
  const bloc = (x, y, cap, l, w, h, nom) => blocs.push({ x, y, cap, l, w, h, nom, id: 500 + blocs.length });
  bloc(19.8, 2.6, 0, 1.4, 0.8, 0.75, 'table');
  bloc(18.75, 3.55, 0.25, 0.45, 0.45, 0.9, 'chaise rouge');
  bloc(20.95, 2.65, 0, 0.45, 0.45, 0.9, 'chaise');
  bloc(21.3, -3.3, Math.PI / 2, 2.0, 0.85, 0.8, 'canapé');
  bloc(18.0, -4.25, 0.15, 0.6, 0.5, 0.5, 'carton');
  bloc(18.7, -4.3, -0.1, 0.5, 0.4, 0.4, 'carton');
  bloc(21.5, 4.4, 0, 0.9, 0.4, 1.8, 'armoire');
  // Rochers du jardin.
  for (const [x, y] of [[3.6, -3.4], [8.6, -4.6], [-3.4, 4.8], [6.8, 6.2], [1.2, -5.6], [-4.4, -4.6]]) {
    bloc(x, y, rnd() * Math.PI, 0.5 + rnd() * 0.5, 0.4 + rnd() * 0.4, 0.35 + rnd() * 0.3, 'rocher');
  }

  const bas = [];
  const objetBas = (x, y, cap, l, w, h, nom) => bas.push({ x, y, cap, l, w, h, nom, id: 700 + bas.length });
  objetBas(15.0, 0.42, 0, 0.22, 0.9, 0.12, 'poutre');
  objetBas(4.0, 2.4, 0, 0.32, 0.32, 0.22, 'pot de fleurs');
  objetBas(6.4, -2.2, 0, 0.3, 0.3, 0.2, 'pot');
  objetBas(9.4, -1.6, 0.4, 0.8, 0.35, 0.14, 'bordure');
  objetBas(11.3, 1.7, 0, 0.6, 0.35, 0.16, 'marche');

  // Arbres et poteaux du jardin, à l'écart de la route.
  const arbres = [];
  const procheRoute = (x, y, marge) => {
    for (let i = 0; i < ROUTE.length; i++) {
      const [ax, ay] = ROUTE[i], [bx, by] = ROUTE[(i + 1) % ROUTE.length];
      const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy || 1;
      const t = Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / l2));
      if (Math.hypot(x - ax - t * dx, y - ay - t * dy) < marge) return true;
    }
    return false;
  };
  let essais = 0;
  while (arbres.length < 46 && essais++ < 4000) {
    const x = -5.5 + rnd() * 17, y = -6.5 + rnd() * 13;
    if (procheRoute(x, y, 1.5)) continue;
    if (arbres.some((a) => Math.hypot(a.x - x, a.y - y) < 1.3)) continue;
    if ([...blocs, ...bas].some((b) => Math.hypot(b.x - x, b.y - y) < 1.2)) continue;
    const grand = rnd();
    arbres.push({ x, y, r: 0.05 + grand * 0.14, h: 0.9 + grand * 1.4 + rnd() * 0.3, id: 900 + arbres.length });
  }
  for (const [x, y] of [[11.4, -0.95], [11.4, 0.95]]) arbres.push({ x, y, r: 0.06, h: 1.1, id: 900 + arbres.length }); // poteaux du portail
  arbres.push({ x: 21.4, y: -0.6, r: 0.13, h: 1.6, id: 900 + arbres.length }); // lampadaire

  // Segments et cercles vus par le lidar (les objets bas sont sous son plan).
  const segments = murs.map((m) => [m.x0, m.y0, m.x1, m.y1]);
  for (const b of blocs) {
    const c = coins(b);
    for (let k = 0; k < 4; k++) segments.push([...c[k], ...c[(k + 1) % 4]]);
  }
  return { murs, blocs, bas, arbres, segments };
}

function coins(b) {
  const c = Math.cos(b.cap), s = Math.sin(b.cap), l = b.l / 2, w = b.w / 2;
  return [[l, w], [-l, w], [-l, -w], [l, -w]].map(([x, y]) => [b.x + c * x - s * y, b.y + s * x + c * y]);
}

function rayonSegment(px, py, dx, dy, x0, y0, x1, y1) {
  const ex = x1 - x0, ey = y1 - y0;
  const den = dx * ey - dy * ex;
  if (Math.abs(den) < 1e-12) return Infinity;
  const ux = x0 - px, uy = y0 - py;
  const t = (ux * ey - uy * ex) / den, s = (ux * dy - uy * dx) / den;
  return t > 1e-6 && s >= 0 && s <= 1 ? t : Infinity;
}

function rayonCercle(px, py, dx, dy, cx, cy, r) {
  const ox = px - cx, oy = py - cy;
  const b = ox * dx + oy * dy, c = ox * ox + oy * oy - r * r;
  const disc = b * b - c;
  if (disc < 0) return Infinity;
  const t = -b - Math.sqrt(disc);
  return t > 1e-6 ? t : Infinity;
}

// ------------------------------------------------------------------ simulation

class Demo {
  constructor(vue) {
    this.vue = vue;
    this.monde = construireMonde();
    this.t = 0;
    this.rover = { x: 0, y: 0, cap: 0.2, v: 0, braquage: 0, w: 0 };
    this.indice = 0; // segment de route courant
    this.arret = { jusqua: -1, fait: false };
    this.vus = new Map(); // id → dernière observation (s)
    this.murVu = new Map(); // id du mur → [s0, s1] (m le long du mur)
    this.etiquettes = new Map(); // id → { texte, x, y, depuis }
    this.chaiseVue = -1;
    this.prochain = { scene: 0, etat: 0, scan: 0.05, obstacles: 0.1 };
    this.danger = new Set();
    this.positionsVivants = new Map(); // id → dernière position vue
  }

  /** Avance la simulation jusqu'à `t` (s) par pas de 20 ms, messages compris. */
  avancer(jusqua) {
    while (this.t + 0.02 <= jusqua + 1e-9) {
      this._pas(0.02);
      this.t += 0.02;
      if (this.t >= this.prochain.etat) { this.prochain.etat += 0.1; this.vue.etat(this._etat()); }
      if (this.t >= this.prochain.scene) { this.prochain.scene += 0.25; this.vue.scene(this._scene()); }
      if (this.t >= this.prochain.scan) { this.prochain.scan += 0.2; this.vue.scan(this._scan()); }
      if (this.t >= this.prochain.obstacles) { this.prochain.obstacles += 0.2; this.vue.obstacles(this._obstacles()); }
    }
  }

  _pas(dt) {
    const r = this.rover, L = PROFIL.geometrie.empattement_m;
    // Avancement le long de la route : segment courant ou suivant, le plus proche.
    for (let n = 0; n < 2; n++) {
      const [ax, ay] = ROUTE[this.indice], [bx, by] = ROUTE[(this.indice + 1) % ROUTE.length];
      const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
      const t = ((r.x - ax) * dx + (r.y - ay) * dy) / l2;
      if (t > 1) {
        this.indice = (this.indice + 1) % ROUTE.length;
        if (this.indice === 0) this.arret.fait = false;
      } else break;
    }
    // Point visé à 1,0 m devant sur la route.
    const visee = this._pointDevant(1.0);
    const [lx, ly] = versLocal(r, visee[0], visee[1]);
    const alpha = Math.atan2(ly, lx);
    const braquage = Math.max(-31 * DEG, Math.min(31 * DEG, Math.atan2(2 * L * Math.sin(alpha), 1.0)));
    r.braquage += (braquage - r.braquage) * Math.min(1, dt / 0.15);
    const dedans = r.x > 11.6 && Math.abs(r.y) < 5.2;
    let vVoulue = (dedans ? 0.5 : 0.75) * (1 - 0.45 * Math.abs(r.braquage) / (31 * DEG));
    // Poutre du couloir : arrêt de sécurité au premier passage vers la pièce.
    if (!this.arret.fait && Math.cos(r.cap) > 0.8 && r.x > ARRET_POUTRE.x0 && r.x < ARRET_POUTRE.x1 && Math.abs(r.y) < 0.5) {
      this.arret = { jusqua: this.t + 2.2, fait: true };
    }
    // Être vivant dans le couloir devant : arrêt tant qu'il y reste.
    this.bloqueVivant = VIVANTS.some((b) => {
      const [x, y] = b.position(this.t);
      const [lx, ly] = versLocal(r, x, y);
      return lx > 0.3 && lx < PROFIL.geometrie.emprise_avant_m + 0.9 && Math.abs(ly) < 0.5;
    });
    if (this.t < this.arret.jusqua || this.bloqueVivant) vVoulue = 0;
    const a = vVoulue > r.v ? 0.6 : 1.4;
    r.v += Math.sign(vVoulue - r.v) * Math.min(Math.abs(vVoulue - r.v), a * dt);
    r.w = (r.v / L) * Math.tan(r.braquage);
    r.cap = Math.atan2(Math.sin(r.cap + r.w * dt), Math.cos(r.cap + r.w * dt));
    r.x += r.v * Math.cos(r.cap) * dt;
    r.y += r.v * Math.sin(r.cap) * dt;
  }

  _pointDevant(distance) {
    const r = this.rover;
    let i = this.indice;
    let [ax, ay] = ROUTE[i];
    let [bx, by] = ROUTE[(i + 1) % ROUTE.length];
    const dx = bx - ax, dy = by - ay, l = Math.hypot(dx, dy);
    let t = Math.max(0, Math.min(1, ((r.x - ax) * dx + (r.y - ay) * dy) / (l * l)));
    let px = ax + t * dx, py = ay + t * dy;
    let reste = distance;
    for (let n = 0; n < ROUTE.length; n++) {
      const sx = bx - px, sy = by - py, sl = Math.hypot(sx, sy);
      if (sl >= reste) return [px + (sx / sl) * reste, py + (sy / sl) * reste];
      reste -= sl;
      i = (i + 1) % ROUTE.length;
      [px, py] = [bx, by];
      [bx, by] = ROUTE[(i + 1) % ROUTE.length];
    }
    return [bx, by];
  }

  _visible(x, y, ignorer = -1) {
    const r = this.rover;
    const dx = x - r.x, dy = y - r.y, d = Math.hypot(dx, dy);
    if (d < 1e-3) return true;
    const ux = dx / d, uy = dy / d;
    for (const m of this.monde.murs) {
      if (m.id === ignorer) continue;
      if (rayonSegment(r.x, r.y, ux, uy, m.x0, m.y0, m.x1, m.y1) < d - 0.06) return false;
    }
    return true;
  }

  _enDanger(points) {
    const r = this.rover, g = PROFIL.geometrie;
    const portee = g.emprise_avant_m + 0.5 + 1.4 * Math.abs(r.v);
    for (const [x, y] of points) {
      const [lx, ly] = versLocal(r, x, y);
      if (lx > g.emprise_avant_m - 0.05 && lx < portee && Math.abs(ly) < g.emprise_demi_largeur_m + 0.12) return true;
    }
    return false;
  }

  _scene() {
    const r = this.rover, t = this.t, monde = this.monde;
    const objets = [];
    const bruit = () => (Math.random() - 0.5) * 0.012;
    // Murs : portion vue (cumulée) et prolongements supposés.
    for (const m of monde.murs) {
      const L = Math.hypot(m.x1 - m.x0, m.y1 - m.y0), ux = (m.x1 - m.x0) / L, uy = (m.y1 - m.y0) / L;
      let s0 = Infinity, s1 = -Infinity;
      for (let s = 0; s <= L; s += 0.12) {
        const x = m.x0 + ux * s, y = m.y0 + uy * s;
        if (Math.hypot(x - r.x, y - r.y) > 8) continue;
        if (!this._visible(x, y, m.id)) continue;
        s0 = Math.min(s0, s);
        s1 = Math.max(s1, s);
      }
      const cle = `m${m.id}`;
      if (s1 > s0) {
        const ancien = this.murVu.get(m.id);
        this.murVu.set(m.id, ancien ? [Math.min(ancien[0], s0), Math.max(ancien[1], s1)] : [s0, s1]);
        this.vus.set(cle, t);
      }
      const vu = this.murVu.get(m.id);
      if (!vu || t - this.vus.get(cle) > 30) continue;
      const age = t - this.vus.get(cle);
      const [a, b] = vu;
      const cap = Math.atan2(uy, ux);
      const pts = [];
      for (let s = a; s <= b; s += 0.2) pts.push([m.x0 + ux * s, m.y0 + uy * s]);
      objets.push({
        id: 100 + m.id, type: 'mur', x: m.x0 + ux * (a + b) / 2 + bruit(), y: m.y0 + uy * (a + b) / 2 + bruit(), cap,
        longueur: Math.max(0.12, b - a), epaisseur: m.e, hauteur: m.h, age_s: age, extrapole: false,
        danger: age < 1 && this._enDanger(pts),
      });
      if (a > 0.1) {
        const e0 = Math.max(0, a - 2.2);
        objets.push({ id: 300 + 2 * m.id, type: 'mur', x: m.x0 + ux * (e0 + a) / 2, y: m.y0 + uy * (e0 + a) / 2, cap, longueur: a - e0, epaisseur: m.e, hauteur: m.h, age_s: age, extrapole: true, danger: false });
      }
      if (b < L - 0.1) {
        const e1 = Math.min(L, b + 2.2);
        objets.push({ id: 301 + 2 * m.id, type: 'mur', x: m.x0 + ux * (b + e1) / 2, y: m.y0 + uy * (b + e1) / 2, cap, longueur: e1 - b, epaisseur: m.e, hauteur: m.h, age_s: age, extrapole: true, danger: false });
      }
    }
    const observer = (o, portee, champ = Math.PI) => {
      const d = Math.hypot(o.x - r.x, o.y - r.y);
      if (d > portee) return;
      const [lx, ly] = versLocal(r, o.x, o.y);
      if (Math.abs(Math.atan2(ly, lx)) > champ) return;
      if (this._visible(o.x, o.y)) this.vus.set(`o${o.id}`, t);
    };
    for (const a of monde.arbres) {
      observer(a, 9);
      const vu = this.vus.get(`o${a.id}`);
      if (vu === undefined || t - vu > 30) continue;
      objets.push({ id: a.id, type: 'arbre', x: a.x + bruit(), y: a.y + bruit(), rayon: a.r, hauteur: a.h, age_s: t - vu, danger: t - vu < 1 && this._enDanger([[a.x, a.y]]) });
    }
    for (const b of monde.blocs) {
      observer(b, 8);
      const vu = this.vus.get(`o${b.id}`);
      if (vu === undefined || t - vu > 30) continue;
      objets.push({ id: b.id, type: 'bloc', x: b.x + bruit(), y: b.y + bruit(), cap: b.cap, longueur: b.l, largeur: b.w, hauteur: b.h, age_s: t - vu, danger: t - vu < 1 && this._enDanger([[b.x, b.y], ...coins(b)]) });
      if (b.nom === 'chaise rouge' && t - vu < 0.3) {
        if (this.chaiseVue < 0) this.chaiseVue = t;
        if (t - this.chaiseVue > 1.5) this.etiquettes.set(800, { texte: 'chaise rouge', x: b.x, y: b.y, vu: t });
      }
    }
    for (const b of monde.bas) {
      observer(b, 4, 39 * DEG);
      const vu = this.vus.get(`o${b.id}`);
      if (vu === undefined || t - vu > 30) continue;
      const danger = (t - vu < 1 && this._enDanger([[b.x, b.y], ...coins(b)])) || (b.nom === 'poutre' && t < this.arret.jusqua);
      objets.push({ id: b.id, type: 'bas', x: b.x + bruit(), y: b.y + bruit(), cap: b.cap, longueur: b.l, largeur: b.w, hauteur: b.h, age_s: t - vu, danger });
      if (b.nom === 'pot de fleurs' && t - vu < 0.3 && Math.hypot(b.x - r.x, b.y - r.y) < 3.2) this.etiquettes.set(801, { texte: 'pot de fleurs', x: b.x, y: b.y, vu: t });
      if (b.nom === 'poutre' && t < this.arret.jusqua) this.etiquettes.set(802, { texte: 'poutre au sol', x: b.x, y: b.y, vu: t });
    }
    for (const [id, e] of this.etiquettes) {
      if (t - e.vu > 30) { this.etiquettes.delete(id); continue; }
      objets.push({ id, type: 'etiquette', x: e.x, y: e.y, texte: e.texte, age_s: t - e.vu });
    }
    // Êtres vivants (détection caméra continue) : oubliés 5 s après la dernière vue.
    for (const b of VIVANTS) {
      const [x, y] = b.position(t);
      const [lx, ly] = versLocal(r, x, y);
      if (Math.hypot(lx, ly) < 9 && Math.abs(Math.atan2(ly, lx)) < 39 * DEG && this._visible(x, y)) {
        this.vus.set(`v${b.id}`, t);
        this.positionsVivants.set(b.id, [x, y]);
      }
      const vu = this.vus.get(`v${b.id}`);
      if (vu === undefined || t - vu > 5) continue;
      const [px, py] = this.positionsVivants.get(b.id);
      const [qx, qy] = versLocal(r, px, py);
      const devant = qx > 0.3 && qx < PROFIL.geometrie.emprise_avant_m + 1.2 && Math.abs(qy) < 0.6;
      objets.push({ id: b.id, type: b.type, x: px, y: py, rayon: b.rayon, hauteur: b.hauteur, texte: b.texte, age_s: t - vu, danger: t - vu < 0.5 && devant });
    }
    // Chemin jusqu'au prochain but.
    const chemin = [[r.x, r.y]];
    let i = (this.indice + 1) % ROUTE.length;
    for (let n = 0; n < ROUTE.length; n++) {
      chemin.push(ROUTE[i]);
      if (BUTS.has(i)) break;
      i = (i + 1) % ROUTE.length;
    }
    return {
      type: 'scene', t, repere: 'odom', rover: { x: r.x, y: r.y, cap: r.cap },
      objets, libre: this._libre(120, 7), chemin, but: ROUTE[i],
    };
  }

  /** Distances du lidar sur `n` rayons (repère odom), murs, blocs et arbres. */
  _lancer(angle, portee) {
    const r = this.rover, dx = Math.cos(angle), dy = Math.sin(angle);
    let d = portee;
    for (const s of this.monde.segments) d = Math.min(d, rayonSegment(r.x, r.y, dx, dy, s[0], s[1], s[2], s[3]));
    for (const a of this.monde.arbres) {
      if (Math.abs(a.x - r.x) > d + 1 || Math.abs(a.y - r.y) > d + 1) continue;
      d = Math.min(d, rayonCercle(r.x, r.y, dx, dy, a.x, a.y, a.r));
    }
    return d;
  }

  _libre(n, portee) {
    const r = this.rover, points = [];
    for (let k = 0; k < n; k++) {
      const a = r.cap + (k / n) * 2 * Math.PI;
      const d = Math.min(portee, this._lancer(a, portee));
      points.push([r.x + d * Math.cos(a), r.y + d * Math.sin(a)]);
    }
    return points;
  }

  _scan() {
    const r = this.rover, xy = [];
    for (let k = 0; k < 360; k++) {
      const a = -Math.PI + k * DEG;
      const d = this._lancer(r.cap + a, 12);
      if (d >= 12) continue;
      xy.push(Math.round(d * Math.cos(a) * 100), Math.round(d * Math.sin(a) * 100));
    }
    return { type: 'scan', t: this.t, xy };
  }

  _obstacles() {
    const r = this.rover, xy = [];
    for (const b of this.monde.bas) {
      const [lx, ly] = versLocal(r, b.x, b.y);
      if (Math.hypot(lx, ly) > 4 || lx < 0.3 || Math.abs(Math.atan2(ly, lx)) > 39 * DEG) continue;
      const c = coins(b);
      for (let k = 0; k < 4; k++) {
        const [x0, y0] = c[k], [x1, y1] = c[(k + 1) % 4];
        const n = Math.ceil(Math.hypot(x1 - x0, y1 - y0) / 0.04);
        for (let j = 0; j < n; j++) {
          const [px, py] = versLocal(r, x0 + ((x1 - x0) * j) / n, y0 + ((y1 - y0) * j) / n);
          xy.push(Math.round(px * 100), Math.round(py * 100));
        }
      }
    }
    return { type: 'obstacles', t: this.t, source: 'stereo', xy, portee_m: 4 };
  }

  _etat() {
    const r = this.rover;
    const poutre = this.t < this.arret.jusqua;
    const arret = poutre || Boolean(this.bloqueVivant);
    return {
      type: 'etat', t: this.t, mode: 'auto', arme: true, arret_urgence: false,
      consigne: { vitesse: arret ? 0 : r.v, braquage: r.braquage, source: 'auto' },
      commande: { vitesse: r.v, braquage: r.braquage, servo_deg: r.braquage / DEG },
      odom: { x: r.x, y: r.y, cap: r.cap, vitesse: r.v, vitesse_lacet: r.w, qualite: 0.95 },
      securite: { arret, vitesse_max_avant: arret ? 0 : 0.8, vitesse_max_arriere: 0.3, raisons: poutre ? ['obstacle_bas'] : (arret ? ['obstacle_avant'] : []), libre_avant_m: arret ? 0.4 : 2.5 },
      capteurs: { lidar: { ok: true, marche: true } },
      simulation: { x: r.x, y: r.y, cap: r.cap, vitesse: r.v },
    };
  }
}

function versLocal(pose, x, y) {
  const dx = x - pose.x, dy = y - pose.y, c = Math.cos(pose.cap), s = Math.sin(pose.cap);
  return [c * dx + s * dy, -s * dx + c * dy];
}

// ------------------------------------------------------------------ page

const parametres = new URLSearchParams(location.search);
const conteneur = document.getElementById('vue');
const vue = new Vue3D(conteneur, {
  profil: PROFIL,
  camera: parametres.get('camera') || 'suivie',
  poussiere: parametres.get('poussiere') === '1',
});
vue.activer(!document.hidden);
document.addEventListener('visibilitychange', () => vue.activer(!document.hidden));

const boutons = [...document.querySelectorAll('[data-camera]')];
function choisirCamera(mode) {
  const m = vue.camera(mode);
  for (const b of boutons) b.setAttribute('aria-pressed', String(b.dataset.camera === m));
}
for (const b of boutons) b.addEventListener('click', () => choisirCamera(b.dataset.camera));
choisirCamera(parametres.get('camera') || 'suivie');
window.addEventListener('keydown', (e) => {
  const mode = { 1: 'suivie', 2: 'dessus', 3: 'libre' }[e.key];
  if (mode) choisirCamera(mode);
});

if (parametres.get('ips') === '1') {
  const ips = document.getElementById('ips');
  ips.hidden = false;
  setInterval(() => {
    const s = vue.statistiques();
    ips.textContent = `${s.ips.toFixed(0)} i/s · ${s.dpr}x · ${s.appels} appels`;
  }, 500);
}

const ws = parametres.get('ws');
let demo = null;
if (ws) {
  brancher(ws === '1' ? new URL('ws', location.href).href.replace(/^http/, 'ws') : ws);
} else {
  demo = new Demo(vue);
  const decalage = Number(parametres.get('t')) || 0;
  demo.avancer(decalage);
  const debut = performance.now() / 1000 - decalage;
  // `?figer=1` : monde figé à l'instant demandé (captures), seul l'état continue d'arriver.
  if (parametres.get('figer') === '1') {
    const scene = demo._scene();
    setInterval(() => vue.scene(scene), 250);
    setInterval(() => {
      const etat = demo._etat();
      etat.odom.vitesse = etat.odom.vitesse_lacet = 0; // pose fixe (pas de prédiction)
      vue.etat(etat);
    }, 100);
  } else {
    setInterval(() => demo.avancer(performance.now() / 1000 - debut), 20);
  }
}

function brancher(url) {
  const conduire = parametres.get('conduire') === '1';
  let minuterie = null;
  const ouvrir = () => {
    const socket = new WebSocket(url);
    socket.addEventListener('message', (evenement) => {
      let m;
      try { m = JSON.parse(evenement.data); } catch { return; }
      if (m.type === 'bienvenue') {
        vue.profil(m.profil);
        if (conduire && m.materiel === 'simulation') {
          socket.send(JSON.stringify({ type: 'mode', mode: 'manuel' }));
          socket.send(JSON.stringify({ type: 'armer' }));
          const t0 = performance.now();
          clearInterval(minuterie);
          minuterie = setInterval(() => {
            const t = (performance.now() - t0) / 1000;
            if (socket.readyState === WebSocket.OPEN) {
              socket.send(JSON.stringify({ type: 'pilotage', vitesse: 0.45, braquage: 0.55 * Math.sin(t * 0.35) }));
            }
          }, 100);
        }
      } else if (m.type === 'etat') vue.etat(m);
      else if (m.type === 'scene') vue.scene(m);
      else if (m.type === 'scan') vue.scan(m);
      else if (m.type === 'obstacles') vue.obstacles(m);
    });
    socket.addEventListener('close', () => {
      clearInterval(minuterie);
      setTimeout(ouvrir, 1500);
    });
    window.liaison = socket;
  };
  ouvrir();
}

window.vue3d = { vue, demo, choisirCamera };
