import { tr } from './langue.js';
// Vue de dessus du lidar dans base_link (avant vers le haut) : points du scan, obstacles de la
// stéréo, emprise du rover et couloir de sécurité le long de l'arc prévu.

import { couleur, preparerCanevas, nombre } from './outils.js';
import { emprise, enveloppe, traverse, longueurPrevue, DEG } from './geometrie.js';

const PORTEES = [2, 4, 8, 12];

export class Radar {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {object} contexte  { donnees, prevision }
   * @param {{portee?: number, mini?: boolean}} options
   */
  constructor(canvas, contexte, { portee = 4, mini = false } = {}) {
    this.canvas = canvas;
    this.donnees = contexte.donnees;
    this.prevision = contexte.prevision;
    this.portee = portee;
    this.mini = mini;
    this.survol = null;
    if (!mini) {
      canvas.addEventListener('pointermove', (e) => { this.survol = this._versRobot(e); });
      canvas.addEventListener('pointerleave', () => { this.survol = null; });
    }
  }

  changerPortee() {
    const i = PORTEES.indexOf(this.portee);
    this.portee = PORTEES[(i + 1) % PORTEES.length];
    return this.portee;
  }

  definirPortee(p) {
    this.portee = p;
  }

  _geometrie(l, h) {
    // Le mini-radar regarde surtout devant : le rover est placé aux deux tiers de la hauteur.
    const cx = l / 2, cy = this.mini ? h * 0.66 : h / 2;
    const echelle = this.mini ? Math.min(cy / this.portee, l / 2 / (this.portee * 0.8)) : (Math.min(l, h) / 2 - 30) / this.portee;
    return { cx, cy, echelle };
  }

  _versRobot(e) {
    const r = this.canvas.getBoundingClientRect();
    const { cx, cy, echelle } = this._geometrie(r.width, r.height);
    const px = e.clientX - r.left, py = e.clientY - r.top;
    return { x: (cy - py) / echelle, y: (cx - px) / echelle, px, py };
  }

  dessiner() {
    const c = preparerCanevas(this.canvas);
    if (!c) return;
    const { ctx, l, h } = c;
    const { cx, cy, echelle } = this._geometrie(l, h);
    const ecran = (x, y) => [cx - y * echelle, cy - x * echelle];
    const profil = this.donnees.profil, g = profil.geometrie;

    // Cercles de distance et rayons.
    ctx.lineWidth = 1;
    ctx.strokeStyle = couleur('--grille');
    ctx.fillStyle = couleur('--texte-3');
    ctx.font = '11px system-ui, sans-serif';
    ctx.textBaseline = 'alphabetic';
    const anneaux = [0.5, 1, 2, 4, 8, 12].filter((r) => r <= this.portee + 1e-6 && r * echelle > 18);
    for (const rayon of anneaux) {
      ctx.beginPath();
      ctx.arc(cx, cy, rayon * echelle, 0, Math.PI * 2);
      ctx.stroke();
      const libelle = `${nombre(rayon, rayon < 1 ? 1 : 0)} m`;
      // Mini-radar : sur l'axe, au-dessus ; grand radar : sur la diagonale avant droite.
      if (this.mini) ctx.fillText(tr(libelle), cx + 4, cy - rayon * echelle - 3);
      else ctx.fillText(tr(libelle), cx + rayon * echelle * 0.74 + 3, cy - rayon * echelle * 0.67 - 3);
    }
    if (!this.mini) {
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      for (let a = 0; a < 360; a += 30) {
        const r = this.portee * echelle;
        const ar = a * DEG;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx - Math.sin(ar) * r, cy - Math.cos(ar) * r);
        ctx.stroke();
        const libelle = a <= 180 ? a : a - 360;
        ctx.fillText(tr(`${libelle}°`), cx - Math.sin(ar) * (r + 10), cy - Math.cos(ar) * (r + 10));
      }
      ctx.textAlign = 'left';
      ctx.textBaseline = 'alphabetic';
    }

    // Couloir le long de l'arc prévu (vers l'avant ou l'arrière selon le sens).
    const etat = this.donnees.etat;
    const p = this.prevision();
    const sens = p.vitesse < -0.01 ? -1 : 1;
    const marge = profil.securite.marge_laterale_m || 0;
    const demi = g.emprise_demi_largeur_m + marge;
    const xBord = sens > 0 ? g.emprise_avant_m : g.emprise_arriere_m;
    const libre = sens > 0 ? etat?.securite?.libre_avant_m : etat?.securite?.libre_arriere_m;
    let longueur = longueurPrevue(p.vitesse);
    const k = Math.tan(p.braquage) / g.empattement_m;
    if (Math.abs(k) > 1e-6) longueur = Math.min(longueur, (Math.PI * 0.75) / Math.abs(k));
    const env = enveloppe({ braquage: p.braquage, empattement: g.empattement_m, xBord, demiLargeur: demi, longueur, sens });
    const obstacleDevant = (etat?.securite?.raisons || []).some((r) => r === (sens > 0 ? 'obstacle_avant' : 'obstacle_arriere'));
    ctx.beginPath();
    env.gauche.forEach(([x, y], i) => { const [u, v] = ecran(x, y); if (i) ctx.lineTo(u, v); else ctx.moveTo(u, v); });
    for (let i = env.droite.length - 1; i >= 0; i--) { const [u, v] = ecran(...env.droite[i]); ctx.lineTo(u, v); }
    ctx.closePath();
    ctx.fillStyle = obstacleDevant ? couleur('--critique-voile') : couleur('--accent-voile');
    ctx.fill();
    ctx.strokeStyle = obstacleDevant ? couleur('--critique') : couleur('--accent');
    ctx.lineWidth = 1.5;
    ctx.stroke();
    if (Number.isFinite(libre) && libre < longueur) {
      const [a, b] = traverse({ braquage: p.braquage, empattement: g.empattement_m, xBord, demiLargeur: demi, s: sens * libre });
      ctx.strokeStyle = couleur('--critique');
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(...ecran(...a));
      ctx.lineTo(...ecran(...b));
      ctx.stroke();
    }

    // Emprise du rover.
    ctx.beginPath();
    emprise(g).forEach(([x, y], i) => { const [u, v] = ecran(x, y); if (i) ctx.lineTo(u, v); else ctx.moveTo(u, v); });
    ctx.closePath();
    ctx.fillStyle = couleur('--rover-voile');
    ctx.fill();
    ctx.strokeStyle = couleur('--texte-1');
    ctx.lineWidth = 1.5;
    ctx.stroke();
    const [nx, ny] = ecran(g.emprise_avant_m, 0);
    ctx.fillStyle = couleur('--texte-1');
    ctx.beginPath();
    ctx.moveTo(nx, ny - 5);
    ctx.lineTo(nx - 4, ny + 2);
    ctx.lineTo(nx + 4, ny + 2);
    ctx.fill();

    // Obstacles de la stéréo (losanges) puis points du lidar.
    const obstacles = this.donnees.obstacles;
    if (obstacles && performance.now() - obstacles.recu < 2000) {
      ctx.fillStyle = couleur('--stereo');
      const pts = obstacles.points;
      for (let i = 0; i < pts.length; i += 2) {
        const [u, v] = ecran(pts[i], pts[i + 1]);
        ctx.beginPath();
        ctx.moveTo(u, v - 3.5); ctx.lineTo(u + 3.5, v); ctx.lineTo(u, v + 3.5); ctx.lineTo(u - 3.5, v);
        ctx.fill();
      }
    }
    const scan = this.donnees.scan;
    if (scan && performance.now() - scan.recu < 2000) {
      ctx.fillStyle = couleur('--lidar');
      const taille = this.mini ? 2.5 : 3;
      const pts = scan.points;
      for (let i = 0; i < pts.length; i += 2) {
        const [u, v] = ecran(pts[i], pts[i + 1]);
        ctx.fillRect(u - taille / 2, v - taille / 2, taille, taille);
      }
      if (this.survol) this._dessinerSurvol(ctx, ecran, pts, l);
    }
  }

  _dessinerSurvol(ctx, ecran, pts, l) {
    let meilleur = -1, dmin = 0.4;
    for (let i = 0; i < pts.length; i += 2) {
      const d = Math.hypot(pts[i] - this.survol.x, pts[i + 1] - this.survol.y);
      if (d < dmin) { dmin = d; meilleur = i; }
    }
    if (meilleur < 0) return;
    const x = pts[meilleur], y = pts[meilleur + 1];
    const [u, v] = ecran(x, y);
    ctx.strokeStyle = couleur('--texte-1');
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(u, v, 6, 0, Math.PI * 2);
    ctx.stroke();
    const texte = `${nombre(Math.hypot(x, y), 2)} m · ${nombre(Math.atan2(y, x) / DEG, 0)}°`;
    ctx.font = '600 12px system-ui, sans-serif';
    const largeur = ctx.measureText(tr(texte)).width + 12;
    const bx = Math.min(u + 10, l - largeur - 4), by = Math.max(v - 28, 4);
    ctx.fillStyle = couleur('--surface-3');
    ctx.fillRect(bx, by, largeur, 20);
    ctx.fillStyle = couleur('--texte-1');
    ctx.textBaseline = 'middle';
    ctx.fillText(tr(texte), bx + 6, by + 10);
    ctx.textBaseline = 'alphabetic';
  }
}
