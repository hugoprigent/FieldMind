// Carte du repère odom : grille d'occupation (PNG du message `carte`), trajectoire, chemin,
// but, dernier scan et rover à l'échelle. Molette / pincement = zoom, glisser = déplacer,
// clic = nouveau but (le rappel décide de la confirmation).

import { couleur, preparerCanevas, nombre } from './outils.js';
import { emprise } from './geometrie.js';

function rvb(hex) {
  const h = hex.replace('#', '');
  const n = parseInt(h.length === 3 ? h.split('').map((c) => c + c).join('') : h.slice(0, 6), 16);
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
}

export class Plan {
  constructor(canvas, contexte, { surClic } = {}) {
    this.canvas = canvas;
    this.donnees = contexte.donnees;
    this.surClic = surClic;
    this.image = null; // canevas recoloré de la grille
    this.meta = null;
    this.centre = { x: 0, y: 0 };
    this.echelle = 60; // px par mètre
    this.suivi = true;
    this.pointeurs = new Map();
    this.glisse = null;
    this.marque = null;
    this.version = 0;
    this._installer();
  }

  _installer() {
    const c = this.canvas;
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const r = c.getBoundingClientRect();
      this.zoomer(e.deltaY > 0 ? 1 / 1.15 : 1.15, e.clientX - r.left, e.clientY - r.top);
    }, { passive: false });
    c.addEventListener('pointerdown', (e) => {
      c.setPointerCapture(e.pointerId);
      this.pointeurs.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (this.pointeurs.size === 1) this.glisse = { x: e.clientX, y: e.clientY, depart: { ...this.centre }, bouge: false };
      else this.glisse = { pincement: this._ecartPointeurs(), echelle: this.echelle, bouge: true };
    });
    c.addEventListener('pointermove', (e) => {
      if (!this.pointeurs.has(e.pointerId)) return;
      this.pointeurs.set(e.pointerId, { x: e.clientX, y: e.clientY });
      const g = this.glisse;
      if (!g) return;
      if (this.pointeurs.size >= 2 && g.pincement) {
        const r = c.getBoundingClientRect();
        const pts = [...this.pointeurs.values()];
        const mx = (pts[0].x + pts[1].x) / 2 - r.left, my = (pts[0].y + pts[1].y) / 2 - r.top;
        const voulu = g.echelle * (this._ecartPointeurs() / g.pincement);
        this.zoomer(voulu / this.echelle, mx, my);
        return;
      }
      const dx = e.clientX - g.x, dy = e.clientY - g.y;
      if (!g.bouge && Math.hypot(dx, dy) > 6) { g.bouge = true; this.suivi = false; this._signalerSuivi(); }
      if (g.bouge && g.depart) {
        this.centre = { x: g.depart.x - dx / this.echelle, y: g.depart.y + dy / this.echelle };
      }
    });
    const fin = (e) => {
      if (!this.pointeurs.has(e.pointerId)) return;
      this.pointeurs.delete(e.pointerId);
      const g = this.glisse;
      if (this.pointeurs.size === 0) {
        this.glisse = null;
        if (g && !g.bouge && e.type === 'pointerup' && this.surClic) {
          const r = c.getBoundingClientRect();
          const [x, y] = this.versMonde(e.clientX - r.left, e.clientY - r.top);
          this.surClic({ x, y, ecranX: e.clientX, ecranY: e.clientY });
        }
      } else if (this.pointeurs.size === 1) {
        const [p] = this.pointeurs.values();
        this.glisse = { x: p.x, y: p.y, depart: { ...this.centre }, bouge: true };
      }
    };
    c.addEventListener('pointerup', fin);
    c.addEventListener('pointercancel', fin);
  }

  _ecartPointeurs() {
    const [a, b] = [...this.pointeurs.values()];
    return Math.max(10, Math.hypot(a.x - b.x, a.y - b.y));
  }

  _signalerSuivi() {
    this.canvas.dispatchEvent(new CustomEvent('suivi', { detail: this.suivi }));
  }

  definirSuivi(suivi) {
    this.suivi = suivi;
    this._signalerSuivi();
  }

  zoomer(facteur, px, py) {
    const r = this.canvas.getBoundingClientRect();
    if (px === undefined) { px = r.width / 2; py = r.height / 2; }
    const [wx, wy] = this.versMonde(px, py);
    this.echelle = Math.min(400, Math.max(4, this.echelle * facteur));
    // Le point sous le curseur reste en place (sauf en suivi : le rover reste au centre).
    if (!this.suivi) {
      this.centre = { x: wx - (px - r.width / 2) / this.echelle, y: wy + (py - r.height / 2) / this.echelle };
    }
  }

  /** Cadre la trajectoire, le rover, le but et la partie connue de la carte. */
  ajuster() {
    const pts = [];
    const odom = this.donnees.etat?.odom;
    if (odom) pts.push([odom.x, odom.y]);
    const carte = this.donnees.carte;
    for (const p of carte?.trajectoire || []) pts.push(p);
    for (const p of carte?.chemin || []) pts.push(p);
    if (carte?.but) pts.push(carte.but);
    if (this.connu) pts.push(...this.connu);
    if (!pts.length) pts.push([0, 0]);
    const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
    const x0 = Math.min(...xs) - 1.5, x1 = Math.max(...xs) + 1.5, y0 = Math.min(...ys) - 1.5, y1 = Math.max(...ys) + 1.5;
    const r = this.canvas.getBoundingClientRect();
    this.centre = { x: (x0 + x1) / 2, y: (y0 + y1) / 2 };
    this.echelle = Math.min(200, Math.max(4, Math.min(r.width / (x1 - x0), r.height / (y1 - y0))));
    this.definirSuivi(false);
  }

  versEcran(x, y) {
    const r = this.taille;
    return [r.l / 2 + (x - this.centre.x) * this.echelle, r.h / 2 - (y - this.centre.y) * this.echelle];
  }

  versMonde(px, py) {
    const r = this.canvas.getBoundingClientRect();
    return [this.centre.x + (px - r.width / 2) / this.echelle, this.centre.y - (py - r.height / 2) / this.echelle];
  }

  /** Nouveau message `carte` : décodage du PNG et recoloration selon le thème. */
  async recevoir(m) {
    if (!m.png) { this.meta = m; return; }
    const version = ++this.version;
    const image = new Image();
    image.src = `data:image/png;base64,${m.png}`;
    try {
      await image.decode();
    } catch {
      return;
    }
    if (version !== this.version) return;
    const L = image.naturalWidth, H = image.naturalHeight;
    const toile = this.image && this.image.width === L && this.image.height === H ? this.image : Object.assign(document.createElement('canvas'), { width: L, height: H });
    const ctx = toile.getContext('2d', { willReadFrequently: true });
    ctx.clearRect(0, 0, L, H);
    ctx.drawImage(image, 0, 0);
    const pixels = ctx.getImageData(0, 0, L, H);
    const d = pixels.data;
    const libre = rvb(couleur('--carte-libre')), occupe = rvb(couleur('--carte-occupe')), inconnu = rvb(couleur('--carte-inconnu'));
    let xmin = L, xmax = -1, ymin = H, ymax = -1;
    for (let i = 0, n = 0; i < d.length; i += 4, n++) {
      const g = d[i];
      if (Math.abs(g - 128) <= 4) {
        d[i + 3] = 0; // inconnu : transparent (fond de la vue)
        continue;
      }
      const x = n % L, y = (n / L) | 0;
      if (x < xmin) xmin = x; if (x > xmax) xmax = x; if (y < ymin) ymin = y; if (y > ymax) ymax = y;
      const cible = g > 128 ? libre : occupe;
      const f = g > 128 ? Math.min(1, (g - 128) / 100) : Math.min(1, (128 - g) / 90);
      d[i] = inconnu[0] + (cible[0] - inconnu[0]) * f;
      d[i + 1] = inconnu[1] + (cible[1] - inconnu[1]) * f;
      d[i + 2] = inconnu[2] + (cible[2] - inconnu[2]) * f;
      d[i + 3] = 255;
    }
    ctx.putImageData(pixels, 0, 0);
    this.image = toile;
    this.meta = m;
    const res = m.resolution, [ox, oy] = m.origine;
    this.connu = xmax >= 0 ? [[ox + xmin * res, oy + (H - ymax) * res], [ox + (xmax + 1) * res, oy + (H - ymin) * res]] : null;
  }

  dessiner() {
    const c = preparerCanevas(this.canvas);
    if (!c) return;
    const { ctx, l, h } = c;
    this.taille = { l, h };
    const odom = this.donnees.etat?.odom;
    if (this.suivi && odom) this.centre = { x: odom.x, y: odom.y };
    ctx.fillStyle = couleur('--carte-fond');
    ctx.fillRect(0, 0, l, h);

    // Quadrillage d'un mètre (ou plus si l'on est loin).
    const pas = this.echelle > 30 ? 1 : this.echelle > 8 ? 5 : 10;
    const [x0, y1] = this.versMonde(0, 0), [x1, y0] = this.versMonde(l, h);
    ctx.strokeStyle = couleur('--grille');
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(x0 / pas) * pas; x <= x1; x += pas) { const [px] = this.versEcran(x, 0); ctx.moveTo(Math.round(px) + 0.5, 0); ctx.lineTo(Math.round(px) + 0.5, h); }
    for (let y = Math.ceil(y0 / pas) * pas; y <= y1; y += pas) { const [, py] = this.versEcran(0, y); ctx.moveTo(0, Math.round(py) + 0.5); ctx.lineTo(l, Math.round(py) + 0.5); }
    ctx.stroke();

    // Grille d'occupation : ligne 0 du PNG = y maximal.
    const m = this.meta;
    if (this.image && m) {
      const res = m.resolution, [ox, oy] = m.origine;
      const [px, py] = this.versEcran(ox, oy + this.image.height * res);
      ctx.imageSmoothingEnabled = this.echelle * res < 2;
      ctx.drawImage(this.image, px, py, this.image.width * res * this.echelle, this.image.height * res * this.echelle);
      ctx.imageSmoothingEnabled = true;
    }

    // Origine du repère odom.
    const [ox, oy] = this.versEcran(0, 0);
    ctx.strokeStyle = couleur('--texte-3');
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(ox - 7, oy); ctx.lineTo(ox + 7, oy); ctx.moveTo(ox, oy - 7); ctx.lineTo(ox, oy + 7);
    ctx.stroke();

    const carte = this.donnees.carte;
    const polyligne = (points, teinte, largeur, tirets = []) => {
      if (!points || points.length < 2) return;
      ctx.beginPath();
      points.forEach(([x, y], i) => { const [u, v] = this.versEcran(x, y); if (i) ctx.lineTo(u, v); else ctx.moveTo(u, v); });
      ctx.setLineDash(tirets);
      ctx.strokeStyle = teinte;
      ctx.lineWidth = largeur;
      ctx.lineJoin = 'round';
      ctx.stroke();
      ctx.setLineDash([]);
    };
    polyligne(carte?.trajectoire, couleur('--trajet'), 2);
    polyligne(carte?.chemin, couleur('--accent'), 2.5, [8, 6]);
    if (carte?.but) this._cible(ctx, carte.but[0], carte.but[1], couleur('--accent'));
    if (this.marque) this._cible(ctx, this.marque.x, this.marque.y, couleur('--alerte'));

    const exploration = this.donnees.etat?.mission?.exploration;
    for (const p of exploration?.prochains || []) {
      const [u, v] = this.versEcran(p.x, p.y);
      ctx.strokeStyle = couleur('--texte-3'); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(u, v, 4, 0, Math.PI * 2); ctx.stroke();
    }
    (exploration?.points || []).forEach((p, i) => {
      const [u, v] = this.versEcran(p.x, p.y);
      ctx.fillStyle = couleur(p.etat === 'visite' ? '--ok' : p.etat === 'inaccessible' ? '--alerte' : '--accent');
      ctx.beginPath(); ctx.arc(u, v, 7, 0, Math.PI * 2); ctx.fill();
      ctx.font = '11px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillStyle = couleur('--carte-fond'); ctx.fillText(String(i + 1), u, v);
    });
    ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
    for (const o of this.donnees.etat?.monde?.objets || []) {
      if ((o.observations_visuelles ?? 1) < 1) continue;
      const [u, v] = this.versEcran(o.x, o.y);
      const confirme = o.statut === 'confirme';
      ctx.fillStyle = couleur(confirme ? '--accent' : '--texte-3');
      if (confirme) ctx.fillRect(u - 2, v - 2, 4, 4);
      else { ctx.strokeStyle = ctx.fillStyle; ctx.strokeRect(u - 3, v - 3, 6, 6); }
      ctx.font = '11px sans-serif'; ctx.fillStyle = couleur('--texte-1');
      ctx.fillText(`${o.label.slice(0, 32)}${confirme ? '' : ' ?'}`, u + 6, v - 5);
    }
    for (const piece of this.donnees.etat?.monde?.pieces || []) {
      const preuves = piece.preuves || [];
      if (preuves.length < 2) continue;
      const x = preuves.reduce((s, p) => s + p.x, 0) / preuves.length;
      const y = preuves.reduce((s, p) => s + p.y, 0) / preuves.length;
      const [u, v] = this.versEcran(x, y);
      ctx.font = 'bold 12px sans-serif';
      ctx.fillStyle = couleur(piece.statut === 'confirmee' ? '--ok' : '--texte-3');
      ctx.fillText(`${piece.nom}${piece.statut === 'confirmee' ? '' : ' ?'}`, u + 8, v + 16);
    }

    if (odom) {
      const c0 = Math.cos(odom.cap), s0 = Math.sin(odom.cap);
      const scan = this.donnees.scan;
      if (scan && performance.now() - scan.recu < 2000) {
        ctx.fillStyle = couleur('--lidar');
        const t = Math.max(1.5, Math.min(3, this.echelle * 0.04));
        const p = scan.points;
        for (let i = 0; i < p.length; i += 2) {
          const [u, v] = this.versEcran(odom.x + c0 * p[i] - s0 * p[i + 1], odom.y + s0 * p[i] + c0 * p[i + 1]);
          ctx.fillRect(u - t / 2, v - t / 2, t, t);
        }
      }
      const g = this.donnees.profil.geometrie;
      const remorque = this.donnees.etat?.remorque;
      if (remorque?.geometrie) {
        // Planche de la remorque, tournée de l'angle d'attelage estimé (φ > 0 : à gauche).
        const r = remorque.geometrie, phi = (remorque.angle_deg || 0) * Math.PI / 180;
        const hx = odom.x - r.attelage_m * c0, hy = odom.y - r.attelage_m * s0;
        const cr = Math.cos(odom.cap - phi), sr = Math.sin(odom.cap - phi);
        ctx.beginPath();
        [[-r.avant_m, r.demi_largeur_m], [-r.arriere_m, r.demi_largeur_m],
          [-r.arriere_m, -r.demi_largeur_m], [-r.avant_m, -r.demi_largeur_m]].forEach(([x, y], i) => {
          const [u, v] = this.versEcran(hx + cr * x - sr * y, hy + sr * x + cr * y);
          if (i) ctx.lineTo(u, v); else ctx.moveTo(u, v);
        });
        ctx.closePath();
        ctx.fillStyle = couleur('--texte-3');
        ctx.fill();
        ctx.strokeStyle = couleur('--texte-1');
        ctx.lineWidth = 1;
        ctx.stroke();
        const [au, av] = this.versEcran(hx, hy), [bu, bv] = this.versEcran(hx - cr * r.avant_m, hy - sr * r.avant_m);
        ctx.beginPath(); ctx.moveTo(au, av); ctx.lineTo(bu, bv); ctx.lineWidth = 2; ctx.stroke();
      }
      ctx.beginPath();
      emprise(g).forEach(([x, y], i) => {
        const [u, v] = this.versEcran(odom.x + c0 * x - s0 * y, odom.y + s0 * x + c0 * y);
        if (i) ctx.lineTo(u, v); else ctx.moveTo(u, v);
      });
      ctx.closePath();
      ctx.fillStyle = couleur('--accent');
      ctx.fill();
      ctx.strokeStyle = couleur('--texte-1');
      ctx.lineWidth = 1.5;
      ctx.stroke();
      // Flèche de cap (toujours lisible, même de loin).
      const [rx, ry] = this.versEcran(odom.x, odom.y);
      const longueur = Math.max(16, (g.emprise_avant_m + 0.2) * this.echelle);
      ctx.beginPath();
      ctx.moveTo(rx, ry);
      ctx.lineTo(rx + Math.cos(odom.cap) * longueur, ry - Math.sin(odom.cap) * longueur);
      ctx.strokeStyle = couleur('--texte-1');
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // Échelle.
    const metres = [0.5, 1, 2, 5, 10, 20, 50].find((v) => v * this.echelle >= 60) || 50;
    const largeur = metres * this.echelle;
    ctx.fillStyle = couleur('--texte-2');
    ctx.fillRect(14, h - 18, largeur, 3);
    ctx.fillRect(14, h - 24, 2, 9);
    ctx.fillRect(14 + largeur - 2, h - 24, 2, 9);
    ctx.font = '600 11px system-ui, sans-serif';
    ctx.textBaseline = 'bottom';
    ctx.fillText(`${nombre(metres, metres < 1 ? 1 : 0)} m`, 14, h - 26);
  }

  _cible(ctx, x, y, teinte) {
    const [u, v] = this.versEcran(x, y);
    ctx.strokeStyle = 'rgba(0,0,0,0.5)';
    ctx.lineWidth = 5;
    ctx.beginPath(); ctx.arc(u, v, 9, 0, Math.PI * 2); ctx.stroke();
    ctx.strokeStyle = teinte;
    ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.arc(u, v, 9, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = teinte;
    ctx.beginPath(); ctx.arc(u, v, 3, 0, Math.PI * 2); ctx.fill();
  }
}
