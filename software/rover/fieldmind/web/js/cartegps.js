import { tr } from './langue.js';
// Carte GPS sur canvas : tuiles raster (OpenStreetMap par défaut, en Web Mercator) sous un
// quadrillage métrique toujours lisible sans Internet, et par-dessus : zone autorisée, trace,
// parcours, but, lieux, point GPS brut et rover (cercle de précision, flèche selon le cap vrai).
// Grande carte : glisser / molette / pincement, clic selon le mode (but, parcours, zone), points
// du brouillon déplaçables. Mini-carte : rover centré, carte tournée cap en haut, sans interaction.

import { couleur, preparerCanevas, nombre, lireLocal } from './outils.js';

export const TUILES_OSM = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png';
// Tuiles OSM de 3 à 19 ; au-delà, la vue agrandit les tuiles du niveau 19 (rover de 50 cm).
const ZOOM_MIN = 3, ZOOM_MAX = 22, ZOOM_TUILES_MAX = 19;
const TERRE = 40075016.686; // circonférence équatoriale (m)
const DEG = Math.PI / 180;

// ------------------------------------------------------------------ Web Mercator (pur)

/** Latitude, longitude → coordonnées « monde » au zoom 0 (0..256). */
export function versMonde(lat, lon) {
  const s = Math.sin(Math.max(-85.0511, Math.min(85.0511, lat)) * DEG);
  return [((lon + 180) / 360) * 256, (0.5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI)) * 256];
}

export function versLatLon(x, y) {
  const n = Math.PI - (2 * Math.PI * y) / 256;
  return [Math.atan(Math.sinh(n)) / DEG, (x / 256) * 360 - 180];
}

/** Unités « monde » (zoom 0) par mètre, à cette latitude. */
export function unitesParMetre(lat) {
  return 256 / (TERRE * Math.cos(lat * DEG));
}

export function distanceM(lat1, lon1, lat2, lon2) {
  const r = 6371008.8, p1 = lat1 * DEG, p2 = lat2 * DEG;
  const a = Math.sin(((lat2 - lat1) * DEG) / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(((lon2 - lon1) * DEG) / 2) ** 2;
  return 2 * r * Math.asin(Math.min(1, Math.sqrt(a)));
}

/** Lieu, but ou point avec des coordonnées utilisables (un lieu mémorisé avant l'alignement n'en a pas). */
export function situe(p) {
  return Boolean(p) && Number.isFinite(p.lat) && Number.isFinite(p.lon);
}

/** Point décalé de (est, nord) mètres. */
export function decaler(lat, lon, est, nord) {
  return [lat + nord / 111320, lon + est / (111320 * Math.cos(lat * DEG))];
}

// ------------------------------------------------------------------ tuiles (cache partagé)

const cache = new Map(); // "z/x/y|modele" → { image, etat: 'charge' | 'ok' | 'echec', t }
const MAX_CACHE = 500;
let derniersSucces = 0, derniersEchecs = 0, echecsDeSuite = 0, dernierEssai = 0, enVol = 0;
const MAX_EN_VOL = 6; // requêtes de tuiles simultanées
const PAUSE_HORS_LIGNE_MS = 20000; // après plusieurs échecs de suite, une seule tuile d'essai par période

function modeleTuiles() {
  const m = lireLocal('tuiles', TUILES_OSM);
  return typeof m === 'string' ? m.trim() : TUILES_OSM;
}

/** Tuile du cache, ou nouvelle requête si le budget de l'image le permet (`budget.n`). */
function tuile(z, x, y, modele, budget = { n: Infinity }) {
  const n = 2 ** z;
  const xx = ((x % n) + n) % n;
  if (y < 0 || y >= n) return null;
  const cle = `${z}/${xx}/${y}|${modele}`;
  let t = cache.get(cle);
  if (t && t.etat === 'echec' && performance.now() - t.t > 30000) t = null; // nouvel essai
  if (!t) {
    // Hors ligne : pas de rafale de requêtes vouées à l'échec, un essai par période.
    if (echecsDeSuite >= 4 && performance.now() - dernierEssai < PAUSE_HORS_LIGNE_MS) return null;
    if (budget.n <= 0 || enVol >= MAX_EN_VOL) return null;
    budget.n--;
    enVol++;
    dernierEssai = performance.now();
    const image = new Image();
    t = { image, etat: 'charge', t: performance.now() };
    image.decoding = 'async';
    image.onload = () => { enVol--; t.etat = 'ok'; derniersSucces = performance.now(); echecsDeSuite = 0; };
    image.onerror = () => { enVol--; t.etat = 'echec'; t.t = performance.now(); derniersEchecs = performance.now(); echecsDeSuite++; };
    image.src = modele.replace('{z}', z).replace('{x}', xx).replace('{y}', y).replace('{s}', 'abc'[(xx + y) % 3]);
    cache.set(cle, t);
    if (cache.size > MAX_CACHE) {
      for (const [k, v] of cache) {
        if (v.etat !== 'charge') { cache.delete(k); break; }
      }
    }
  } else {
    cache.delete(cle); // ordre d'usage récent
    cache.set(cle, t);
  }
  return t;
}

/** Tuiles indisponibles (pas d'Internet, URL vide ou fausse). */
export function tuilesHorsLigne() {
  return !modeleTuiles() || (derniersEchecs > derniersSucces && performance.now() - derniersEchecs < 60000);
}

// ------------------------------------------------------------------ carte

export class CarteGPS {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {object} contexte  { donnees }
   * @param {{mini?: boolean, surClic?: Function, surChangement?: Function}} options
   */
  constructor(canvas, contexte, options = {}) {
    this.canvas = canvas;
    this.donnees = contexte.donnees;
    this.mini = Boolean(options.mini);
    this.surClic = options.surClic || null;
    this.surChangement = options.surChangement || (() => {});
    this.centre = { lat: 46.5, lon: 2.5 };
    this.zoom = this.mini ? 20 : 5;
    this.suivi = true;
    this.centre0 = false; // centrée une première fois sur des données
    this.mode = 'but'; // but | parcours | zone
    this.brouillon = { parcours: [], zone: [] };
    this.marque = null;
    this.pointeurs = new Map();
    this.geste = null;
    if (!this.mini) this._installer();
  }

  // ---------------------------------------------------------------- position du rover

  /** Position fusionnée, prolongée entre deux messages `gps` par l'odométrie (10 Hz). */
  rover() {
    const g = this.donnees.gps, r = g?.rover;
    if (!r || !Number.isFinite(r.lat)) return null;
    const ancre = this.donnees.gpsOdom, odom = this.donnees.etat?.odom;
    const age = (performance.now() - (this.donnees.recuGps || 0)) / 1000;
    if (!ancre || !odom || age > 3 || !Number.isFinite(r.cap_vrai_deg)) return { ...r };
    const dx = odom.x - ancre.x, dy = odom.y - ancre.y;
    if (Math.hypot(dx, dy) > 5) return { ...r };
    // Rotation odom → (est, nord) déduite du cap vrai au moment du message.
    const alpha = (90 - r.cap_vrai_deg) * DEG - ancre.cap;
    const est = Math.cos(alpha) * dx - Math.sin(alpha) * dy, nord = Math.sin(alpha) * dx + Math.cos(alpha) * dy;
    const [lat, lon] = decaler(r.lat, r.lon, est, nord);
    const cap = (((90 - (odom.cap + alpha) / DEG) % 360) + 360) % 360;
    return { ...r, lat, lon, cap_vrai_deg: cap };
  }

  /** Centre « naturel » : rover, sinon GPS brut, sinon référence. */
  _positionConnue() {
    const g = this.donnees.gps;
    const r = this.rover();
    if (r) return r;
    if (g?.brut && Number.isFinite(g.brut.lat)) return g.brut;
    if (g?.reference && Number.isFinite(g.reference.lat)) return g.reference;
    const gnss = this.donnees.etat?.gnss;
    if (gnss && Number.isFinite(gnss.latitude) && gnss.fix && gnss.fix !== 'aucun') return { lat: gnss.latitude, lon: gnss.longitude };
    return null;
  }

  // ---------------------------------------------------------------- repères

  _vue() {
    const { l, h } = this.taille;
    const [cwx, cwy] = versMonde(this.centre.lat, this.centre.lon);
    return { l, h, cwx, cwy, s: 2 ** this.zoom, rot: this.rotation || 0 };
  }

  versEcran(lat, lon, vue = this._vue()) {
    const [wx, wy] = versMonde(lat, lon);
    let dx = (wx - vue.cwx) * vue.s, dy = (wy - vue.cwy) * vue.s;
    if (vue.rot) {
      const c = Math.cos(vue.rot), s = Math.sin(vue.rot);
      [dx, dy] = [c * dx - s * dy, s * dx + c * dy];
    }
    return [vue.l / 2 + dx, vue.h / 2 + dy];
  }

  versLatLonEcran(px, py) {
    const r = this.canvas.getBoundingClientRect();
    const [cwx, cwy] = versMonde(this.centre.lat, this.centre.lon);
    const s = 2 ** this.zoom;
    return versLatLon(cwx + (px - r.width / 2) / s, cwy + (py - r.height / 2) / s);
  }

  // ---------------------------------------------------------------- commandes

  zoomer(facteur, px, py) {
    const r = this.canvas.getBoundingClientRect();
    if (px === undefined) { px = r.width / 2; py = r.height / 2; }
    const [lat, lon] = this.versLatLonEcran(px, py);
    this.zoom = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, this.zoom + Math.log2(facteur)));
    if (!this.suivi) {
      // Le point sous le curseur reste en place.
      const [wx, wy] = versMonde(lat, lon), s = 2 ** this.zoom;
      [this.centre.lat, this.centre.lon] = versLatLon(wx - (px - r.width / 2) / s, wy - (py - r.height / 2) / s);
    }
  }

  definirSuivi(suivi) {
    this.suivi = suivi;
    this.surChangement();
  }

  definirMode(mode) {
    this.mode = mode;
    const g = this.donnees.gps;
    if (mode === 'parcours' && !this.brouillon.parcours.length && Array.isArray(g?.parcours)) this.brouillon.parcours = g.parcours.map((p) => [...p]);
    if (mode === 'zone' && !this.brouillon.zone.length && Array.isArray(g?.zone)) this.brouillon.zone = g.zone.map((p) => [...p]);
    this.surChangement();
  }

  /** Cadre tout ce qui est connu (rover, trace, parcours, zone, lieux, but). */
  ajuster() {
    const g = this.donnees.gps || {};
    const pts = [];
    const r = this._positionConnue();
    if (r) pts.push([r.lat, r.lon]);
    for (const p of [...(g.trace || []), ...(g.parcours || []), ...(g.zone || []), ...this.brouillon.parcours, ...this.brouillon.zone]) pts.push(p);
    for (const l of g.lieux || []) if (situe(l)) pts.push([l.lat, l.lon]);
    if (situe(g.but)) pts.push([g.but.lat, g.but.lon]);
    if (!pts.length) return;
    const ms = pts.map(([la, lo]) => versMonde(la, lo));
    const x0 = Math.min(...ms.map((m) => m[0])), x1 = Math.max(...ms.map((m) => m[0]));
    const y0 = Math.min(...ms.map((m) => m[1])), y1 = Math.max(...ms.map((m) => m[1]));
    const rect = this.canvas.getBoundingClientRect();
    [this.centre.lat, this.centre.lon] = versLatLon((x0 + x1) / 2, (y0 + y1) / 2);
    const etendue = Math.max(x1 - x0, y1 - y0, 20 * unitesParMetre(this.centre.lat));
    this.zoom = Math.min(21.5, Math.max(ZOOM_MIN, Math.log2((Math.min(rect.width, rect.height) * 0.75) / etendue)));
    this.definirSuivi(false);
  }

  // ---------------------------------------------------------------- interactions

  _installer() {
    const c = this.canvas;
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const r = c.getBoundingClientRect();
      this.zoomer(e.deltaY > 0 ? 1 / 1.25 : 1.25, e.clientX - r.left, e.clientY - r.top);
    }, { passive: false });
    c.addEventListener('pointerdown', (e) => {
      c.setPointerCapture(e.pointerId);
      const r = c.getBoundingClientRect();
      const px = e.clientX - r.left, py = e.clientY - r.top;
      this.pointeurs.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (this.pointeurs.size === 2) {
        const [a, b] = [...this.pointeurs.values()];
        this.geste = { pincement: Math.hypot(a.x - b.x, a.y - b.y), zoom: this.zoom, bouge: true };
        return;
      }
      const indice = this._pointSous(px, py);
      if (indice !== null) {
        this.geste = { glisserPoint: indice, bouge: false, x: e.clientX, y: e.clientY };
      } else {
        this.geste = { x: e.clientX, y: e.clientY, depart: versMonde(this.centre.lat, this.centre.lon), bouge: false };
      }
    });
    c.addEventListener('pointermove', (e) => {
      const r = c.getBoundingClientRect();
      if (!this.pointeurs.has(e.pointerId)) {
        c.style.cursor = this._pointSous(e.clientX - r.left, e.clientY - r.top) !== null ? 'grab' : 'crosshair';
        return;
      }
      this.pointeurs.set(e.pointerId, { x: e.clientX, y: e.clientY });
      const g = this.geste;
      if (!g) return;
      if (g.pincement && this.pointeurs.size >= 2) {
        const [a, b] = [...this.pointeurs.values()];
        const voulu = g.zoom + Math.log2(Math.max(10, Math.hypot(a.x - b.x, a.y - b.y)) / g.pincement);
        this.zoomer(2 ** (voulu - this.zoom), (a.x + b.x) / 2 - r.left, (a.y + b.y) / 2 - r.top);
        return;
      }
      const dx = e.clientX - g.x, dy = e.clientY - g.y;
      if (!g.bouge && Math.hypot(dx, dy) > 6) {
        g.bouge = true;
        if (g.depart) this.definirSuivi(false);
      }
      if (!g.bouge) return;
      if (g.glisserPoint !== undefined) {
        const liste = this.brouillon[this.mode];
        liste[g.glisserPoint] = this.versLatLonEcran(e.clientX - r.left, e.clientY - r.top);
        this.surChangement();
      } else if (g.depart) {
        const s = 2 ** this.zoom;
        [this.centre.lat, this.centre.lon] = versLatLon(g.depart[0] - dx / s, g.depart[1] - dy / s);
      }
    });
    const fin = (e) => {
      if (!this.pointeurs.has(e.pointerId)) return;
      this.pointeurs.delete(e.pointerId);
      const g = this.geste;
      if (this.pointeurs.size > 0) return;
      this.geste = null;
      if (!g || g.bouge || e.type !== 'pointerup' || g.pincement) return;
      const r = c.getBoundingClientRect();
      const [lat, lon] = this.versLatLonEcran(e.clientX - r.left, e.clientY - r.top);
      if (this.mode === 'parcours' || this.mode === 'zone') {
        if (g.glisserPoint !== undefined) return;
        this.brouillon[this.mode].push([lat, lon]);
        this.surChangement();
      } else if (this.surClic) {
        this.surClic({ lat, lon, ecranX: e.clientX, ecranY: e.clientY });
      }
    };
    c.addEventListener('pointerup', fin);
    c.addEventListener('pointercancel', fin);
  }

  /** Indice du point du brouillon (mode parcours ou zone) sous le pointeur, ou null. */
  _pointSous(px, py) {
    if (this.mode !== 'parcours' && this.mode !== 'zone' || !this.taille) return null;
    const liste = this.brouillon[this.mode];
    for (let i = liste.length - 1; i >= 0; i--) {
      const [x, y] = this.versEcran(liste[i][0], liste[i][1]);
      if (Math.hypot(x - px, y - py) < 14) return i;
    }
    return null;
  }

  // ---------------------------------------------------------------- dessin

  dessiner() {
    const c = preparerCanevas(this.canvas);
    if (!c) return;
    const { ctx, l, h } = c;
    this.taille = { l, h };
    const position = this._positionConnue();
    const rover = this.rover();
    if (position && (this.suivi || !this.centre0)) {
      this.centre = { lat: position.lat, lon: position.lon };
      if (!this.centre0 && !this.mini) this.zoom = 20;
      this.centre0 = true;
    }
    this.rotation = this.mini && rover && Number.isFinite(rover.cap_vrai_deg) ? -rover.cap_vrai_deg * DEG : 0;
    const vue = this._vue();

    ctx.fillStyle = couleur('--carte-fond');
    ctx.fillRect(0, 0, l, h);
    this._grille(ctx, vue);
    this._tuiles(ctx, vue);
    if (!position && !this.mini) {
      ctx.fillStyle = couleur('--texte-3');
      ctx.font = '600 13px system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(tr('Pas de position GPS'), l / 2, h / 2);
      ctx.textAlign = 'left';
    }
    this._calques(ctx, vue, rover);
    if (!this.mini) this._echelle(ctx, l, h);
  }

  _grille(ctx, vue) {
    // Quadrillage métrique ancré sur la référence GPS (stable d'une image à l'autre).
    const g = this.donnees.gps;
    const ref = g?.reference && Number.isFinite(g.reference.lat) ? g.reference : this.centre;
    const k = unitesParMetre(this.centre.lat) * vue.s; // px par mètre
    const pas = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000].find((p) => p * k >= 48) || 100000;
    const [rx, ry] = versMonde(ref.lat, ref.lon);
    const rayon = Math.hypot(vue.l, vue.h) / 2 / k; // m visibles depuis le centre
    const ox = ((vue.cwx - rx) * vue.s) / k, oy = ((ry - vue.cwy) * vue.s) / k; // centre en m (est, nord) depuis ref
    ctx.save();
    ctx.translate(vue.l / 2, vue.h / 2);
    if (vue.rot) ctx.rotate(vue.rot);
    ctx.strokeStyle = couleur('--grille');
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let e = Math.floor((ox - rayon) / pas) * pas; e <= ox + rayon; e += pas) {
      const x = (e - ox) * k;
      ctx.moveTo(x, -rayon * k); ctx.lineTo(x, rayon * k);
    }
    for (let n = Math.floor((oy - rayon) / pas) * pas; n <= oy + rayon; n += pas) {
      const y = -(n - oy) * k;
      ctx.moveTo(-rayon * k, y); ctx.lineTo(rayon * k, y);
    }
    ctx.stroke();
    ctx.restore();
    this.pasGrille = pas;
  }

  _tuiles(ctx, vue) {
    const modele = modeleTuiles();
    if (!modele) return;
    const zt = Math.max(0, Math.min(ZOOM_TUILES_MAX, Math.round(this.zoom)));
    const taille = 256 * 2 ** (this.zoom - zt); // px d'une tuile à l'écran
    const n = 2 ** zt;
    const rayon = Math.hypot(vue.l, vue.h) / 2;
    const cx = (vue.cwx / 256) * n, cy = (vue.cwy / 256) * n; // centre en indices de tuile
    const r = rayon / taille;
    if (r > 12) return; // trop de tuiles (zoom très faible sur un grand écran)
    ctx.save();
    ctx.translate(vue.l / 2, vue.h / 2);
    if (vue.rot) ctx.rotate(vue.rot);
    ctx.imageSmoothingEnabled = true;
    // Du centre vers les bords, et au plus 4 nouvelles requêtes par image : sans Internet,
    // les premiers échecs suspendent les requêtes avant d'en lancer une rafale.
    const liste = [];
    for (let ty = Math.floor(cy - r); ty <= Math.floor(cy + r); ty++) {
      for (let tx = Math.floor(cx - r); tx <= Math.floor(cx + r); tx++) liste.push([tx, ty, (tx + 0.5 - cx) ** 2 + (ty + 0.5 - cy) ** 2]);
    }
    liste.sort((a, b) => a[2] - b[2]);
    const budget = { n: 4 };
    for (const [tx, ty] of liste) {
      const x = (tx - cx) * taille, y = (ty - cy) * taille;
      const t = tuile(zt, tx, ty, modele, budget);
      if (t?.etat === 'ok') ctx.drawImage(t.image, x, y, taille + 0.5, taille + 0.5);
      else this._tuileParente(ctx, zt, tx, ty, x, y, taille, modele);
    }
    ctx.restore();
  }

  /** Pendant le chargement, la tuile d'un zoom inférieur déjà en cache, agrandie. */
  _tuileParente(ctx, z, tx, ty, x, y, taille, modele) {
    for (let d = 1; d <= 4 && z - d >= 0; d++) {
      const f = 2 ** d;
      const px = Math.floor(tx / f), py = Math.floor(ty / f);
      const n = 2 ** (z - d);
      const cle = `${z - d}/${((px % n) + n) % n}/${py}|${modele}`;
      const t = cache.get(cle);
      if (t?.etat === 'ok') {
        const sous = 256 / f;
        ctx.drawImage(t.image, (tx - px * f) * sous, (ty - py * f) * sous, sous, sous, x, y, taille + 0.5, taille + 0.5);
        return;
      }
    }
  }

  _calques(ctx, vue, rover) {
    const g = this.donnees.gps || {};
    const E = (p) => this.versEcran(p[0], p[1], vue);
    const chemin = (points, fermer = false) => {
      ctx.beginPath();
      points.forEach((p, i) => { const [x, y] = E(p); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
      if (fermer) ctx.closePath();
    };
    const contour = (largeur, teinte, tirets = []) => {
      ctx.setLineDash(tirets);
      ctx.lineWidth = largeur + 3;
      ctx.strokeStyle = 'rgba(0,0,0,0.35)';
      ctx.stroke();
      ctx.lineWidth = largeur;
      ctx.strokeStyle = teinte;
      ctx.stroke();
      ctx.setLineDash([]);
    };

    // Zone autorisée (enregistrée), puis brouillon de zone.
    if (Array.isArray(g.zone) && g.zone.length >= 3) {
      chemin(g.zone, true);
      ctx.fillStyle = couleur('--zone-voile');
      ctx.fill();
      contour(2, couleur('--ok'), [10, 6]);
    }
    if (!this.mini && this.mode === 'zone' && this.brouillon.zone.length) {
      chemin(this.brouillon.zone, this.brouillon.zone.length >= 3);
      if (this.brouillon.zone.length >= 3) { ctx.fillStyle = couleur('--alerte-voile'); ctx.fill(); }
      contour(2, couleur('--alerte'));
    }
    // Trace : foncée sur les tuiles (claires), couleur du thème sur le quadrillage seul.
    if (Array.isArray(g.trace) && g.trace.length > 1) {
      const surTuiles = modeleTuiles() && !tuilesHorsLigne();
      chemin(g.trace);
      ctx.lineJoin = 'round';
      ctx.lineWidth = 5;
      ctx.strokeStyle = surTuiles ? 'rgba(255,255,255,0.8)' : 'rgba(0,0,0,0.35)';
      ctx.stroke();
      ctx.lineWidth = 2.5;
      ctx.strokeStyle = surTuiles ? '#3b2a8f' : couleur('--trajet');
      ctx.stroke();
    }
    // Parcours enregistré, puis brouillon.
    const numeros = (points, teinte) => {
      ctx.font = '700 11px system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      points.forEach((p, i) => {
        const [x, y] = E(p);
        ctx.fillStyle = teinte;
        ctx.beginPath(); ctx.arc(x, y, this.mini ? 4 : 9, 0, Math.PI * 2); ctx.fill();
        ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();
        if (!this.mini) { ctx.fillStyle = '#fff'; ctx.fillText(tr(String(i + 1)), x, y + 0.5); }
      });
      ctx.textAlign = 'left';
      ctx.textBaseline = 'alphabetic';
    };
    if (Array.isArray(g.parcours) && g.parcours.length) {
      if (g.parcours.length > 1) { chemin(g.parcours); contour(2.5, couleur('--accent'), [8, 6]); }
      if (!(this.mode === 'parcours' && !this.mini)) numeros(g.parcours, couleur('--accent-fort'));
    }
    if (!this.mini && this.mode === 'parcours' && this.brouillon.parcours.length) {
      if (this.brouillon.parcours.length > 1) { chemin(this.brouillon.parcours); contour(2.5, couleur('--alerte')); }
      numeros(this.brouillon.parcours, couleur('--alerte'));
    }
    if (!this.mini && this.mode === 'zone') numeros(this.brouillon.zone, couleur('--alerte'));

    // Lieux.
    ctx.font = '600 12px system-ui, sans-serif';
    for (const lieu of g.lieux || []) {
      if (!situe(lieu)) continue;
      const [x, y] = E([lieu.lat, lieu.lon]);
      ctx.fillStyle = couleur('--serie-4');
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(x - 6, y - 10);
      ctx.arc(x, y - 13, 6.5, Math.PI * 0.85, Math.PI * 0.15);
      ctx.closePath();
      ctx.fill();
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.2; ctx.stroke();
      if (!this.mini && lieu.nom) this._etiquette(ctx, x + 9, y - 13, lieu.nom);
    }
    // But.
    const but = g.but && Number.isFinite(g.but.lat) ? [g.but.lat, g.but.lon] : null;
    for (const [p, teinte] of [[but, couleur('--accent')], [this.marque, couleur('--alerte')]]) {
      if (!p) continue;
      const [x, y] = E(p);
      ctx.strokeStyle = 'rgba(0,0,0,0.5)'; ctx.lineWidth = 5;
      ctx.beginPath(); ctx.arc(x, y, 10, 0, Math.PI * 2); ctx.stroke();
      ctx.strokeStyle = teinte; ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.arc(x, y, 10, 0, Math.PI * 2); ctx.stroke();
      ctx.fillStyle = teinte;
      ctx.beginPath(); ctx.arc(x, y, 3.5, 0, Math.PI * 2); ctx.fill();
    }
    // Point GPS brut (discret).
    if (g.brut && Number.isFinite(g.brut.lat)) {
      const [x, y] = E([g.brut.lat, g.brut.lon]);
      ctx.strokeStyle = couleur('--texte-3');
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2); ctx.stroke();
    }
    // Rover : cercle de précision et flèche selon le cap vrai.
    if (rover) {
      const [x, y] = E([rover.lat, rover.lon]);
      const k = unitesParMetre(rover.lat) * vue.s;
      if (Number.isFinite(rover.precision_m)) {
        ctx.fillStyle = couleur('--accent-voile');
        ctx.strokeStyle = couleur('--accent');
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.arc(x, y, Math.max(6, rover.precision_m * k), 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      }
      const angle = (Number.isFinite(rover.cap_vrai_deg) ? rover.cap_vrai_deg : 0) * DEG + vue.rot;
      const t = this.mini ? 9 : 12;
      ctx.save();
      ctx.translate(x, y);
      ctx.rotate(angle);
      ctx.beginPath();
      ctx.moveTo(0, -t * 1.3); ctx.lineTo(t * 0.85, t); ctx.lineTo(0, t * 0.5); ctx.lineTo(-t * 0.85, t);
      ctx.closePath();
      ctx.fillStyle = couleur('--accent-fort');
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.restore();
    }
  }

  _etiquette(ctx, x, y, texte) {
    const largeur = ctx.measureText(tr(texte)).width + 10;
    ctx.fillStyle = 'rgba(0,0,0,0.6)';
    ctx.fillRect(x, y - 9, largeur, 18);
    ctx.fillStyle = '#fff';
    ctx.textBaseline = 'middle';
    ctx.fillText(tr(texte), x + 5, y);
    ctx.textBaseline = 'alphabetic';
  }

  _echelle(ctx, l, h) {
    const k = unitesParMetre(this.centre.lat) * 2 ** this.zoom;
    const metres = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000].find((m) => m * k >= 70) || 200000;
    const largeur = metres * k;
    const x = 14, y = h - 16;
    ctx.fillStyle = 'rgba(0,0,0,0.35)';
    ctx.fillRect(x - 6, y - 26, largeur + 12, 32);
    ctx.fillStyle = '#fff';
    ctx.fillRect(x, y - 1, largeur, 3);
    ctx.fillRect(x, y - 7, 2, 9);
    ctx.fillRect(x + largeur - 2, y - 7, 2, 9);
    ctx.font = '600 11px system-ui, sans-serif';
    ctx.textBaseline = 'bottom';
    ctx.fillText(tr(metres >= 1000 ? `${nombre(metres / 1000, 0)} km` : `${metres} m`), x, y - 8);
    ctx.textBaseline = 'alphabetic';
  }
}
