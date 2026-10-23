// Assistant de calibration stéréo : les deux caméras en direct, les coins du damier dessinés
// dès qu'ils sont trouvés, la couverture de l'image, capture à la main ou automatique (damier vu
// des deux caméras, immobile, à un nouvel endroit), puis le calcul et son bilan en clair.

import { $, el, texte, attribut, signaler, signalerRefus, couleur, preparerCanevas, nombre } from '../outils.js';
import { VueCamera } from '../camera.js';
import { cadreContenu } from '../geometrie.js';

const PERIODE_MS = 500;
const CONSEILS = [
  'Tiens le damier bien à plat (collé sur un carton), entièrement visible des deux caméras.',
  'Varie : près et loin, au centre et dans les coins, penché à gauche, à droite, vers le haut.',
  'Reste immobile une seconde à chaque position : la capture automatique s\'en charge.',
];

export class AssistantStereo {
  constructor(ctx) {
    this.ctx = ctx;
    this.dialogue = $('#assistant-stereo');
    this.vues = {
      gauche: new VueCamera($('#as-gauche'), ctx, { camera: 'gauche', ips: 4, largeur: 480 }),
      droite: new VueCamera($('#as-droite'), ctx, { camera: 'droite', ips: 4, largeur: 480 }),
    };
    this.canevas = { gauche: $('#as-gauche-coins'), droite: $('#as-droite-coins') };
    this.dernier = null;
    this.auto = true;
    this.stable = 0;
    this.derniereCapture = null;
    texte($('#as-conseils'), '');
    $('#as-conseils').append(...CONSEILS.map((c) => el('li', { text: c })));
    $('#as-fermer').addEventListener('click', () => this.fermer());
    this.dialogue.addEventListener('close', () => this._arreter());
    $('#as-capturer').addEventListener('click', () => this._capturer(true));
    $('#as-auto').addEventListener('click', () => {
      this.auto = !this.auto;
      attribut($('#as-auto'), 'aria-pressed', this.auto ? 'true' : 'false');
    });
    $('#as-effacer').addEventListener('click', async () => {
      await this.ctx.connexion.demander({ type: 'calibration', nom: 'stereo_effacer' });
      this.derniereCapture = null;
      texte($('#as-resultat'), '');
      signaler('Paires effacées', 'info', 1500);
    });
    $('#as-calculer').addEventListener('click', () => this._calculer());
  }

  ouvrir() {
    this.dialogue.showModal();
    for (const v of Object.values(this.vues)) v.activer();
    this._interroger();
    this.minuterie = setInterval(() => this._interroger(), PERIODE_MS);
    const dessiner = () => {
      if (!this.dialogue.open) return;
      for (const [nom, v] of Object.entries(this.vues)) {
        v.dessiner();
        this._dessinerCoins(nom);
      }
      this.image = requestAnimationFrame(dessiner);
    };
    dessiner();
  }

  fermer() {
    this.dialogue.close();
  }

  _arreter() {
    clearInterval(this.minuterie);
    cancelAnimationFrame(this.image);
    for (const v of Object.values(this.vues)) v.desactiver();
  }

  async _interroger() {
    if (this.enCours || !this.dialogue.open) return;
    this.enCours = true;
    const r = await this.ctx.connexion.demander({ type: 'calibration', nom: 'stereo_apercu' }, 4000);
    this.enCours = false;
    if (!r.ok || !r.valeur) return;
    const a = r.valeur;
    this.dernier = a;
    this._majEtat(a);
    const vuDesDeux = Boolean(a.gauche && a.droite);
    // Auto : vu des deux caméras deux fois de suite au même endroit (immobile), et déplacé depuis
    // la dernière capture (sinon la paire n'apporte rien)
    const centre = vuDesDeux ? this._centre(a.gauche) : null;
    if (vuDesDeux && this.precedent && Math.hypot(centre[0] - this.precedent[0], centre[1] - this.precedent[1]) < 0.015) {
      this.stable += 1;
    } else {
      this.stable = 0;
    }
    this.precedent = centre;
    const nouveau = !this.derniereCapture || !centre
      || Math.hypot(centre[0] - this.derniereCapture[0], centre[1] - this.derniereCapture[1]) > 0.08
      || Math.abs(centre[2] - this.derniereCapture[2]) > 0.06;
    if (this.auto && vuDesDeux && this.stable >= 1 && nouveau) this._capturer(false);
  }

  _centre(coins) {
    const us = coins.map((c) => c[0]), vs = coins.map((c) => c[1]);
    return [us.reduce((s, x) => s + x, 0) / us.length, vs.reduce((s, x) => s + x, 0) / vs.length,
      Math.max(...us) - Math.min(...us)];
  }

  async _capturer(manuel) {
    if (this.capture) return;
    this.capture = true;
    const r = await this.ctx.connexion.demander({ type: 'calibration', nom: 'stereo_capture' }, 8000);
    this.capture = false;
    if (!r.ok) { if (manuel) signalerRefus(r); return; }
    const avant = this.dernier?.paires ?? 0;
    if (Number(r.valeur) > avant) {
      if (this.dernier?.gauche) this.derniereCapture = this._centre(this.dernier.gauche);
      signaler(`Paire ${r.valeur} gardée`, 'succes', 1200);
      this.stable = 0;
    } else if (manuel) {
      signaler('Damier pas vu des deux caméras, ou position déjà prise', 'alerte', 2500);
    }
  }

  async _calculer() {
    texte($('#as-resultat'), 'Calcul en cours…');
    const r = await this.ctx.connexion.demander({ type: 'calibration', nom: 'stereo_calculer' }, 60000);
    const zone = $('#as-resultat');
    zone.replaceChildren();
    if (!r.ok) { texte(zone, r.erreur || 'échec'); attribut(zone, 'data-niveau', 'critique'); return; }
    const b = r.valeur || {};
    attribut(zone, 'data-niveau', b.ok ? 'ok' : 'alerte');
    zone.append(
      el('p', { class: 'titre', text: b.ok ? 'Calibration réussie et enregistrée' : 'Calibration enregistrée, à vérifier' }),
      el('ul', {}, [
        el('li', { text: `Précision : ${nombre(b.erreur_rms_px, 2)} px (bon sous 0,5 ; acceptable sous 1)` }),
        el('li', { text: `Alignement des deux caméras : ${nombre(b.erreur_epipolaire_px, 2)} px (bon sous 0,5)` }),
        el('li', { text: `Écart entre les caméras mesuré : ${nombre((b.base_m || 0) * 1000, 0)} mm (prévu ${nombre((b.base_profil_m || 0) * 1000, 0)} mm)` }),
        el('li', { text: `Champ horizontal : ${nombre(b.hfov_deg, 0)}° ; ${b.paires} paires` }),
        ...(b.avertissements || []).map((x) => el('li', { class: 'avertissement', text: `⚠ ${x}` })),
      ]));
  }

  _majEtat(a) {
    const n = a.paires || 0;
    texte($('#as-compteur'), `${n} paire${n > 1 ? 's' : ''}`);
    texte($('#as-objectif'), n < a.minimum ? `encore ${a.minimum - n} au minimum, ${a.conseille} conseillées`
      : n < a.conseille ? `assez pour calculer ; ${a.conseille} conseillées` : 'parfait, tu peux calculer');
    $('#as-barre').style.width = `${Math.min(100, Math.round((100 * n) / a.conseille))}%`;
    $('#as-calculer').disabled = n < a.minimum;
    for (const nom of ['gauche', 'droite']) {
      const ok = Boolean(a[nom]);
      const pastille = $(`#as-${nom}-etat`);
      attribut(pastille, 'data-niveau', ok ? 'ok' : 'neutre');
      texte(pastille, ok ? 'damier vu' : 'damier non vu');
    }
    // Couverture : zones 3 × 3 de l'image (caméra gauche) où la mire a déjà été prise
    const cases = Array.from({ length: 9 }, () => 0);
    for (const [u, v] of a.couverture || []) cases[Math.min(2, Math.floor(v * 3)) * 3 + Math.min(2, Math.floor(u * 3))] += 1;
    const grille = $('#as-couverture');
    if (grille.childElementCount !== 9) grille.replaceChildren(...cases.map(() => el('span')));
    cases.forEach((c, i) => attribut(grille.children[i], 'data-niveau', c === 0 ? 'vide' : c === 1 ? 'un' : 'plusieurs'));
    const vides = cases.filter((c) => c === 0).length;
    texte($('#as-couverture-texte'), vides ? `${vides} zone${vides > 1 ? 's' : ''} encore vide${vides > 1 ? 's' : ''} : place le damier là` : 'toute l’image est couverte');
  }

  _dessinerCoins(nom) {
    const c = preparerCanevas(this.canevas[nom]);
    const coins = this.dernier?.[nom];
    const dims = this.vues[nom].flux?.dimensions();
    if (!c || !coins || !dims) return;
    const { ctx, l, h } = c;
    const cadre = cadreContenu(l, h, dims[0], dims[1]);
    const X = (u) => cadre.x + u * cadre.l, Y = (v) => cadre.y + v * cadre.h;
    const deux = Boolean(this.dernier?.gauche && this.dernier?.droite);
    ctx.strokeStyle = deux ? couleur('--ok') : couleur('--alerte');
    ctx.fillStyle = ctx.strokeStyle;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    coins.forEach(([u, v], i) => { if (i) ctx.lineTo(X(u), Y(v)); else ctx.moveTo(X(u), Y(v)); });
    ctx.stroke();
    for (const [u, v] of coins) { ctx.beginPath(); ctx.arc(X(u), Y(v), 2.5, 0, Math.PI * 2); ctx.fill(); }
  }
}
