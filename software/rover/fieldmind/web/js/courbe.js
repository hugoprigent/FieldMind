import { tr } from './langue.js';
// Courbe temporelle en direct (Canvas 2D) : séries en couleurs fixes, un seul axe, réticule
// et valeurs au survol.

import { couleur, preparerCanevas, nombre, el } from './outils.js';

const COULEURS = ['--serie-1', '--serie-2', '--serie-3', '--serie-4'];

function pasRond(etendue, n = 4) {
  const brut = etendue / n;
  const puissance = 10 ** Math.floor(Math.log10(brut));
  const f = brut / puissance;
  return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * puissance;
}

export class Courbe {
  /**
   * @param {HTMLElement} conteneur
   * @param {{series: string[], unite: string, fenetre?: number, min?: number, max?: number,
   *          decimales?: number, etendueMin?: number}} options
   */
  constructor(conteneur, options) {
    this.o = { fenetre: 30, decimales: 2, etendueMin: 0, ...options };
    this.donnees = options.series.map(() => []);
    this.tMax = 0;
    this.survol = null;
    const legende = el('div', { class: 'legende' },
      options.series.map((nom, i) => el('span', {}, el('i', { style: `background: var(${COULEURS[i]})` }), nom)));
    this.valeurs = options.series.map(() => el('b', { text: '—' }));
    [...legende.children].forEach((e, i) => e.append(this.valeurs[i]));
    this.canvas = el('canvas', { role: 'img', 'aria-label': options.series.join(', ') });
    conteneur.append(legende, el('div', { class: 'zone-courbe' }, this.canvas));
    this.canvas.addEventListener('pointermove', (e) => {
      const r = this.canvas.getBoundingClientRect();
      this.survol = e.clientX - r.left;
    });
    this.canvas.addEventListener('pointerleave', () => { this.survol = null; });
  }

  ajouter(t, valeurs) {
    if (t < this.tMax - 1) this.vider(); // horloge du rover repartie de zéro
    this.tMax = Math.max(this.tMax, t);
    const limite = t - this.o.fenetre - 1;
    valeurs.forEach((v, i) => {
      const serie = this.donnees[i];
      serie.push([t, Number.isFinite(v) ? v : null]);
      while (serie.length && serie[0][0] < limite) serie.shift();
      const texteValeur = Number.isFinite(v) ? nombre(v, this.o.decimales) : '—';
      if (this.valeurs[i].textContent !== texteValeur) this.valeurs[i].textContent = texteValeur;
    });
  }

  vider() {
    this.donnees.forEach((s) => { s.length = 0; });
    this.tMax = 0;
  }

  dessiner() {
    const c = preparerCanevas(this.canvas);
    if (!c) return;
    const { ctx, l, h } = c;
    const marge = { g: 40, d: 6, hh: 6, b: 16 };
    const t1 = this.tMax, t0 = t1 - this.o.fenetre;
    let min = this.o.min ?? Infinity, max = this.o.max ?? -Infinity;
    if (this.o.min === undefined || this.o.max === undefined) {
      for (const serie of this.donnees) {
        for (const [t, v] of serie) {
          if (v === null || t < t0) continue;
          if (this.o.min === undefined) min = Math.min(min, v);
          if (this.o.max === undefined) max = Math.max(max, v);
        }
      }
    }
    if (!Number.isFinite(min) || !Number.isFinite(max)) { min = 0; max = 1; }
    if (max - min < this.o.etendueMin) {
      const milieu = (max + min) / 2;
      min = milieu - this.o.etendueMin / 2;
      max = milieu + this.o.etendueMin / 2;
    }
    if (max - min < 1e-6) { max += 0.5; min -= 0.5; }
    const pad = (max - min) * 0.08;
    if (this.o.min === undefined) min -= pad;
    if (this.o.max === undefined) max += pad;
    const X = (t) => marge.g + ((t - t0) / (t1 - t0)) * (l - marge.g - marge.d);
    const Y = (v) => marge.hh + (1 - (v - min) / (max - min)) * (h - marge.hh - marge.b);

    ctx.font = '10px system-ui, sans-serif';
    ctx.fillStyle = couleur('--texte-3');
    ctx.strokeStyle = couleur('--grille');
    ctx.lineWidth = 1;
    const pas = pasRond(max - min, h > 140 ? 4 : 3);
    const decimalesAxe = Math.max(0, -Math.floor(Math.log10(pas) + 1e-9));
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let v = Math.ceil(min / pas) * pas; v <= max + 1e-9; v += pas) {
      const py = Math.round(Y(v)) + 0.5;
      ctx.beginPath(); ctx.moveTo(marge.g, py); ctx.lineTo(l - marge.d, py); ctx.stroke();
      ctx.fillText(tr(nombre(Math.abs(v) < pas / 1e6 ? 0 : v, decimalesAxe)), marge.g - 5, py);
    }
    ctx.textBaseline = 'top';
    ctx.textAlign = 'left';
    ctx.fillText(tr(`−${this.o.fenetre} s`), marge.g, h - marge.b + 3);
    ctx.textAlign = 'right';
    ctx.fillText(tr('0'), l - marge.d, h - marge.b + 3);

    ctx.save();
    ctx.beginPath();
    ctx.rect(marge.g, 0, l - marge.g - marge.d, h);
    ctx.clip();
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    this.donnees.forEach((serie, i) => {
      ctx.strokeStyle = couleur(COULEURS[i]);
      ctx.beginPath();
      let enCours = false;
      for (const [t, v] of serie) {
        if (v === null) { enCours = false; continue; }
        const px = X(t), py = Y(Math.min(max, Math.max(min, v)));
        if (enCours) ctx.lineTo(px, py); else ctx.moveTo(px, py);
        enCours = true;
      }
      ctx.stroke();
    });
    ctx.restore();

    if (this.survol !== null && this.survol > marge.g && t1 > 0) {
      const t = t0 + ((this.survol - marge.g) / (l - marge.g - marge.d)) * (t1 - t0);
      const px = Math.round(this.survol) + 0.5;
      ctx.strokeStyle = couleur('--texte-3');
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(px, marge.hh); ctx.lineTo(px, h - marge.b); ctx.stroke();
      const lignes = this.donnees.map((serie) => {
        let meilleur = null;
        for (const point of serie) if (!meilleur || Math.abs(point[0] - t) < Math.abs(meilleur[0] - t)) meilleur = point;
        return meilleur;
      });
      ctx.font = '600 11px system-ui, sans-serif';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      const textes = lignes.map((p) => (p && p[1] !== null ? `${nombre(p[1], this.o.decimales)} ${this.o.unite}` : '—'));
      const largeur = Math.max(...textes.map((s) => ctx.measureText(tr(s)).width)) + 26;
      const bx = px + 8 + largeur > l ? px - 8 - largeur : px + 8;
      ctx.fillStyle = couleur('--surface-3');
      ctx.fillRect(bx, 4, largeur, textes.length * 16 + 6);
      textes.forEach((s, i) => {
        ctx.fillStyle = couleur(COULEURS[i]);
        ctx.fillRect(bx + 6, 10 + i * 16, 8, 3);
        ctx.fillStyle = couleur('--texte-1');
        ctx.fillText(tr(s), bx + 18, 11 + i * 16);
      });
    }
  }
}
