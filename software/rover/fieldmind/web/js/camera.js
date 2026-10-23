import { tr } from './langue.js';
// Vue d'une caméra : flux MJPEG (ou image simulée), ouvert seulement quand la vue est visible,
// avec en surimpression l'arc prévu projeté par le modèle sténopé et le point visé.

import { el, icone, couleur, preparerCanevas, nombre } from './outils.js';
import { ModeleCamera, coteCamera, cadreContenu, enveloppe, traverse, longueurPrevue } from './geometrie.js';

const vues = new Set();

document.addEventListener('visibilitychange', () => {
  for (const vue of vues) vue._appliquerVisibilite();
});

export class VueCamera {
  /**
   * @param {HTMLElement} conteneur  élément .camera
   * @param {object} contexte  { connexion, donnees, prevision }
   * @param {{camera?: string, ips?: number, largeur?: number, superposition?: boolean,
   *          surClic?: (cible: object) => void}} options
   */
  constructor(conteneur, contexte, options = {}) {
    this.conteneur = conteneur;
    this.connexion = contexte.connexion;
    this.donnees = contexte.donnees;
    this.prevision = contexte.prevision;
    this.o = { camera: 'gauche', ips: 10, largeur: 640, superposition: false, ...options };
    this.camera = this.o.camera;
    this.actif = false;
    this.flux = null;
    this.marque = null;
    this.sansImage = el('div', { class: 'sans-image' }, icone('camera'));
    this.canvas = el('canvas', { class: 'superposition', 'aria-hidden': 'true' });
    conteneur.prepend(this.sansImage, this.canvas);
    conteneur.addEventListener('click', (e) => this._clic(e));
    vues.add(this);
    this.donnees.sur('liaison', (etat) => { if (etat === 'ok') this.flux?.relancer(); });
  }

  definirCamera(nom) {
    if (nom === this.camera && this.flux) return;
    this.camera = nom;
    this.marque = null;
    this._fermerFlux();
    if (this.actif) this._ouvrirFlux();
  }

  activer() {
    this.actif = true;
    this._appliquerVisibilite();
  }

  desactiver() {
    this.actif = false;
    this._appliquerVisibilite();
  }

  marquer(u, v) {
    this.marque = u === null ? null : { u, v, t: performance.now() };
  }

  _appliquerVisibilite() {
    const voulu = this.actif && !document.hidden;
    if (voulu && !this.ouvert) this._ouvrirFlux();
    if (!voulu && this.ouvert) this._fermerFlux(false);
  }

  _ouvrirFlux() {
    if (!this.flux) {
      this.flux = this.connexion.flux(this.camera, { ips: this.o.ips, largeur: this.o.largeur });
      this.flux.element.classList.add('media');
      this.conteneur.prepend(this.flux.element);
    }
    this.flux.ouvrir();
    this.ouvert = true;
  }

  _fermerFlux(detruire = true) {
    if (!this.flux) return;
    this.flux.fermer();
    this.ouvert = false;
    if (detruire) {
      this.flux.element.remove();
      this.flux = null;
    }
  }

  /** Rectangle (CSS, relatif au conteneur) où l'image est réellement affichée. */
  _cadre() {
    const r = this.conteneur.getBoundingClientRect();
    const dims = this.flux?.dimensions();
    return { r, dims, cadre: cadreContenu(r.width, r.height, dims?.[0], dims?.[1]) };
  }

  _clic(e) {
    if (!this.o.surClic || e.target.closest('button, .hud')) return;
    const { r, dims, cadre } = this._cadre();
    if (!dims) return;
    const u = (e.clientX - r.left - cadre.x) / cadre.l, v = (e.clientY - r.top - cadre.y) / cadre.h;
    if (u < 0 || u > 1 || v < 0 || v > 1) return;
    this.o.surClic({ camera: this.camera === 'profondeur' ? 'gauche' : this.camera, u, v, x: e.clientX, y: e.clientY });
  }

  dessiner() {
    if (!this.actif) return;
    const capteur = this.donnees.etat?.capteurs?.[this.camera === 'profondeur' ? 'stereo' : this.camera];
    const dims = this.flux?.dimensions();
    const muet = !dims || (capteur?.ok === false && this.camera !== 'drone') || this.donnees.liaison !== 'ok';
    this.sansImage.hidden = !muet;
    const c = preparerCanevas(this.canvas);
    if (!c || !dims || !this.o.superposition) return;
    const { ctx, l, h } = c;
    const cadre = cadreContenu(l, h, dims[0], dims[1]);
    ctx.save();
    ctx.beginPath();
    ctx.rect(cadre.x, cadre.y, cadre.l, cadre.h);
    ctx.clip();
    if (this.camera !== 'drone') this._dessinerArc(ctx, cadre, dims);
    this._dessinerCible(ctx, cadre);
    if (this.marque) {
      const age = (performance.now() - this.marque.t) / 1000;
      if (age > 8) {
        this.marque = null;
      } else {
        const x = cadre.x + this.marque.u * cadre.l, y = cadre.y + this.marque.v * cadre.h;
        ctx.lineWidth = 2;
        ctx.strokeStyle = 'rgba(0,0,0,0.6)';
        ctx.beginPath(); ctx.arc(x, y, 13, 0, Math.PI * 2); ctx.stroke();
        ctx.strokeStyle = couleur('--accent');
        ctx.beginPath(); ctx.arc(x, y, 11, 0, Math.PI * 2); ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(x - 18, y); ctx.lineTo(x - 5, y); ctx.moveTo(x + 5, y); ctx.lineTo(x + 18, y);
        ctx.moveTo(x, y - 18); ctx.lineTo(x, y - 5); ctx.moveTo(x, y + 5); ctx.lineTo(x, y + 18);
        ctx.stroke();
      }
    }
    ctx.restore();
  }

  /** Personne suivie (perception.suivi sur le Pi) : boîte, état et distance. */
  _dessinerCible(ctx, cadre) {
    const c = this.donnees.etat?.cible;
    if (!c || !Array.isArray(c.boite) || c.camera !== this.camera) return;
    const [u0, v0, u1, v1] = c.boite;
    const x = cadre.x + u0 * cadre.l, y = cadre.y + v0 * cadre.h;
    const l = (u1 - u0) * cadre.l, h = (v1 - v0) * cadre.h;
    const teinte = c.etat === 'suivi' ? couleur('--ok') : couleur('--alerte');
    ctx.lineWidth = 4;
    ctx.strokeStyle = 'rgba(0,0,0,0.5)';
    ctx.strokeRect(x, y, l, h);
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = teinte;
    ctx.strokeRect(x, y, l, h);
    const texte = c.etat === 'suivi'
      ? `suivi${Number.isFinite(c.distance_m) ? ` · ${nombre(c.distance_m, 1)} m` : ''}`
      : 'perdu de vue';
    ctx.font = '600 12px system-ui, sans-serif';
    const largeur = ctx.measureText(tr(texte)).width + 10;
    ctx.fillStyle = teinte;
    ctx.fillRect(x, Math.max(cadre.y, y - 18), largeur, 18);
    ctx.fillStyle = '#fff';
    ctx.textBaseline = 'middle';
    ctx.fillText(tr(texte), x + 5, Math.max(cadre.y, y - 18) + 9);
  }

  _dessinerArc(ctx, cadre, dims) {
    const p = this.prevision();
    if (p.vitesse < -0.02) return; // marche arrière : l'arc est derrière les caméras
    const profil = this.donnees.profil, g = profil.geometrie;
    const modele = new ModeleCamera({ ...profil.cameras, largeur: dims[0], hauteur: dims[1] }, coteCamera(this.camera));
    const k = Math.tan(p.braquage) / g.empattement_m;
    let longueur = longueurPrevue(p.vitesse);
    if (Math.abs(k) > 1e-6) longueur = Math.min(longueur, (Math.PI * 0.45) / Math.abs(k)); // ≤ 80° de cap
    const commun = { braquage: p.braquage, empattement: g.empattement_m, xBord: g.emprise_avant_m, demiLargeur: g.emprise_demi_largeur_m };
    const env = enveloppe({ ...commun, longueur });
    const versEcran = ([u, v]) => [cadre.x + u * cadre.l, cadre.y + v * cadre.h];
    const tracer = (polyligne, largeur, teinte) => {
      for (const troncon of modele.projeterPolyligne(polyligne)) {
        ctx.beginPath();
        troncon.forEach((pt, i) => { const [x, y] = versEcran(pt); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
        ctx.lineWidth = largeur + 3;
        ctx.strokeStyle = 'rgba(0,0,0,0.45)';
        ctx.stroke();
        ctx.lineWidth = largeur;
        ctx.strokeStyle = teinte;
        ctx.stroke();
      }
    };
    // Voile entre les deux bords.
    const gauche = modele.projeterPolyligne(env.gauche)[0], droite = modele.projeterPolyligne(env.droite)[0];
    if (gauche && droite) {
      ctx.beginPath();
      gauche.forEach((pt, i) => { const [x, y] = versEcran(pt); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
      for (let i = droite.length - 1; i >= 0; i--) ctx.lineTo(...versEcran(droite[i]));
      ctx.closePath();
      ctx.fillStyle = couleur('--accent-voile');
      ctx.fill();
    }
    const teinte = couleur('--accent');
    tracer(env.gauche, 2.5, teinte);
    tracer(env.droite, 2.5, teinte);
    ctx.font = '600 12px system-ui, sans-serif';
    ctx.textBaseline = 'middle';
    for (let s = 1; s < longueur + 1e-6; s += 1) {
      const [a, b] = traverse({ ...commun, s });
      tracer([a, b], 1.5, teinte);
      const pa = modele.solVersPixel(...a);
      if (pa) {
        const [x, y] = versEcran(pa);
        ctx.fillStyle = 'rgba(0,0,0,0.55)';
        ctx.fillRect(x - 36, y - 9, 32, 18);
        ctx.fillStyle = '#fff';
        ctx.textAlign = 'right';
        ctx.fillText(tr(`${s} m`), x - 8, y);
      }
    }
    const libre = this.donnees.etat?.securite?.libre_avant_m;
    if (Number.isFinite(libre) && libre < longueur) {
      const [a, b] = traverse({ ...commun, s: libre });
      tracer([a, b], 4, couleur('--critique'));
      const pb = modele.solVersPixel(...b);
      if (pb) {
        const [x, y] = versEcran(pb);
        ctx.fillStyle = couleur('--critique');
        ctx.fillRect(x + 6, y - 10, 58, 20);
        ctx.fillStyle = '#fff';
        ctx.textAlign = 'left';
        ctx.fillText(tr(`${nombre(libre, 1)} m`), x + 12, y);
      }
    }
    ctx.textAlign = 'left';
  }
}
