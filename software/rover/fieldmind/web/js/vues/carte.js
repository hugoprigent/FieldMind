// Menu Carte : vue locale (grille d'occupation du repère odom) ou vue GPS (fond OpenStreetMap,
// quadrillage métrique hors ligne). Locale : clic = but odom, remise à zéro de l'odométrie.
// GPS : clic = but GPS, parcours (clics successifs, points déplaçables), zone autorisée, lieux.

import { $, $$, el, icone, texte, attribut, nombre, confirmer, signaler, signalerRefus, lireLocal, ecrireLocal } from '../outils.js';
import { Plan } from '../plan.js';
import { CarteGPS, distanceM, tuilesHorsLigne, situe } from '../cartegps.js';
import { DEG } from '../geometrie.js';
import { FIX_GNSS } from '../textes.js';

const arrondi = (v) => Math.round(v * 1e7) / 1e7;

export class VueCarte {
  constructor(ctx) {
    this.ctx = ctx;
    this.actif = false;
    this.carteDecodee = null;
    this.fond = lireLocal('fond-carte', 'locale');
    this.boucle = false;

    this.plan = new Plan($('#c-plan'), ctx, { surClic: (p) => this.clicLocal(p) });
    this.gps = new CarteGPS($('#c-gps'), ctx, {
      surClic: (p) => this.clicGPS(p),
      surChangement: () => this._majOutils(),
    });
    ctx.donnees.sur('carte', (m) => { if (this.actif && this.fond === 'locale') this._decoder(m); });
    ctx.donnees.sur('gps', () => { if (this.actif) this._majLieux(); });
    $('#c-plan').addEventListener('suivi', () => this._majOutils());

    $('#c-fond').addEventListener('click', (e) => {
      const b = e.target.closest('button[data-fond]');
      if (b) this.definirFond(b.dataset.fond);
    });
    $('#c-modes-gps').addEventListener('click', (e) => {
      const b = e.target.closest('button[data-mode]');
      if (b) this.gps.definirMode(b.dataset.mode);
    });
    const carte = () => (this.fond === 'gps' ? this.gps : this.plan);
    $('#c-plus').addEventListener('click', () => carte().zoomer(1.4));
    $('#c-moins').addEventListener('click', () => carte().zoomer(1 / 1.4));
    $('#c-suivre').addEventListener('click', () => carte().definirSuivi(!carte().suivi));
    $('#c-ajuster').addEventListener('click', () => carte().ajuster());
    $('#c-raz').addEventListener('click', async (e) => {
      if (!(await confirmer('Remettre l’odométrie à zéro ? La carte sera effacée.', e.currentTarget, { danger: true }))) return;
      const r = signalerRefus(await ctx.connexion.demander({ type: 'remise_a_zero_odom' }));
      if (r.ok) signaler('Odométrie remise à zéro', 'succes');
    });

    // Lieux.
    $('#c-lieux-bouton').addEventListener('click', () => {
      const panneau = $('#c-lieux');
      panneau.hidden = !panneau.hidden;
      attribut($('#c-lieux-bouton'), 'aria-pressed', panneau.hidden ? 'false' : 'true');
      this._majLieux();
    });
    $('#c-lieu-form').addEventListener('submit', async (e) => {
      e.preventDefault();
      const nom = $('#c-lieu-nom').value.trim();
      if (!nom) { $('#c-lieu-nom').focus(); return; }
      const r = signalerRefus(await ctx.connexion.demander({ type: 'lieu', action: 'memoriser', nom }));
      if (r.ok) { $('#c-lieu-nom').value = ''; signaler(`Lieu « ${nom} » mémorisé`, 'succes'); }
    });

    // Édition d'un parcours ou d'une zone.
    $('#c-edition-retirer').addEventListener('click', () => { this.gps.brouillon[this.gps.mode]?.pop(); this._majOutils(); });
    $('#c-edition-effacer').addEventListener('click', () => { if (this.gps.brouillon[this.gps.mode]) this.gps.brouillon[this.gps.mode] = []; this._majOutils(); });
    $('#c-edition-boucle').addEventListener('click', () => { this.boucle = !this.boucle; this._majOutils(); });
    $('#c-edition-supprimer').addEventListener('click', async (e) => {
      if (!(await confirmer('Retirer la zone autorisée ?', e.currentTarget, { danger: true }))) return;
      const r = signalerRefus(await ctx.connexion.demander({ type: 'zone', points: null }));
      if (r.ok) { this.gps.brouillon.zone = []; signaler('Zone retirée', 'succes'); this._majOutils(); }
    });
    $('#c-edition-envoyer').addEventListener('click', (e) => this._envoyerBrouillon(e.currentTarget));

    ctx.surTheme(() => { this.carteDecodee = null; if (this.actif && ctx.donnees.carte) this._decoder(ctx.donnees.carte); });
    this.definirFond(this.fond);
  }

  definirFond(fond) {
    this.fond = fond === 'gps' ? 'gps' : 'locale';
    ecrireLocal('fond-carte', this.fond);
    attribut($('#c-panneau'), 'data-fond', this.fond);
    for (const b of $$('#c-fond button')) attribut(b, 'aria-pressed', b.dataset.fond === this.fond ? 'true' : 'false');
    const gps = this.fond === 'gps';
    $('#c-plan').hidden = gps;
    $('#c-gps').hidden = !gps;
    $('#c-modes-gps').hidden = !gps;
    $('#c-pose').hidden = gps;
    $('#c-etat-gps').hidden = !gps;
    $('#c-lieux-bouton').hidden = !gps;
    if (!gps) $('#c-lieux').hidden = true;
    if (gps) this._majLieux();
    else if (this.actif && this.ctx.donnees.carte && this.ctx.donnees.carte !== this.carteDecodee) this._decoder(this.ctx.donnees.carte);
    this._majOutils();
  }

  _decoder(m) {
    this.carteDecodee = m;
    this.plan.recevoir(m);
  }

  activer() {
    this.actif = true;
    const voulu = lireLocal('fond-carte', 'locale');
    if (voulu !== this.fond) this.definirFond(voulu);
    const m = this.ctx.donnees.carte;
    if (this.fond === 'locale' && m && m !== this.carteDecodee) this._decoder(m);
    this._majLieux();
  }

  desactiver() {
    this.actif = false;
  }

  // ---------------------------------------------------------------- clics

  async clicLocal({ x, y, ecranX, ecranY }) {
    if (this.ctx.donnees.liaison !== 'ok') { signaler('Liaison coupée', 'erreur'); return; }
    this.plan.marque = { x, y };
    const ok = await confirmer(`Aller à ${nombre(x, 1)} ; ${nombre(y, 1)} m ?`, { x: ecranX, y: ecranY });
    this.plan.marque = null;
    if (!ok) return;
    signalerRefus(await this.ctx.connexion.demander({ type: 'but', x: Math.round(x * 100) / 100, y: Math.round(y * 100) / 100 }));
  }

  async clicGPS({ lat, lon, ecranX, ecranY }) {
    if (this.ctx.donnees.liaison !== 'ok') { signaler('Liaison coupée', 'erreur'); return; }
    this.gps.marque = [lat, lon];
    const rover = this.gps.rover();
    const distance = rover ? ` (${nombre(distanceM(rover.lat, rover.lon, lat, lon), 0)} m)` : '';
    const ok = await confirmer(`Aller ici${distance} ?`, { x: ecranX, y: ecranY });
    this.gps.marque = null;
    if (!ok) return;
    signalerRefus(await this.ctx.connexion.demander({ type: 'but_gps', lat: arrondi(lat), lon: arrondi(lon) }));
  }

  async _envoyerBrouillon(bouton) {
    const mode = this.gps.mode, points = (this.gps.brouillon[mode] || []).map(([la, lo]) => [arrondi(la), arrondi(lo)]);
    if (mode === 'parcours') {
      if (!points.length) return;
      if (!(await confirmer(`Parcourir ${points.length} point${points.length > 1 ? 's' : ''}${this.boucle ? ' en boucle' : ''} ?`, bouton))) return;
      const r = signalerRefus(await this.ctx.connexion.demander({ type: 'parcours_gps', points, boucle: this.boucle }));
      if (r.ok) { signaler('Parcours envoyé', 'succes'); this.gps.brouillon.parcours = []; this.gps.definirMode('but'); }
    } else if (mode === 'zone') {
      if (points.length < 3) { signaler('3 points au moins', 'alerte'); return; }
      if (!(await confirmer(`Enregistrer cette zone (${points.length} points) ?`, bouton))) return;
      const r = signalerRefus(await this.ctx.connexion.demander({ type: 'zone', points }));
      if (r.ok) { signaler('Zone enregistrée', 'succes'); this.gps.brouillon.zone = []; this.gps.definirMode('but'); }
    }
  }

  // ---------------------------------------------------------------- outils

  _majOutils() {
    const gps = this.fond === 'gps';
    const carte = gps ? this.gps : this.plan;
    attribut($('#c-suivre'), 'aria-pressed', carte.suivi ? 'true' : 'false');
    const mode = this.gps.mode;
    for (const b of $$('#c-modes-gps button')) attribut(b, 'aria-pressed', b.dataset.mode === mode ? 'true' : 'false');
    const edition = gps && (mode === 'parcours' || mode === 'zone');
    $('#c-edition').hidden = !edition;
    if (edition) {
      const n = this.gps.brouillon[mode].length;
      texte($('#c-edition-compte'), String(n));
      $('#c-edition-boucle').hidden = mode !== 'parcours';
      attribut($('#c-edition-boucle'), 'aria-pressed', this.boucle ? 'true' : 'false');
      $('#c-edition-supprimer').hidden = !(mode === 'zone' && this.ctx.donnees.gps?.zone?.length);
      $('#c-edition-envoyer').disabled = mode === 'zone' ? n < 3 : n < 1;
      $('#c-edition-retirer').disabled = n < 1;
      $('#c-edition-effacer').disabled = n < 1;
    }
    $('#c-gps').setAttribute('aria-label', mode === 'but' ? 'Carte GPS : cliquer pour donner un but' : 'Carte GPS : cliquer pour ajouter un point');
  }

  _majLieux() {
    if (this.fond !== 'gps' || $('#c-lieux').hidden) return;
    const g = this.ctx.donnees.gps;
    const lieux = Array.isArray(g?.lieux) ? g.lieux : [];
    const rover = this.gps.rover();
    const cle = JSON.stringify(lieux) + (rover ? Math.round(rover.lat * 1e5) + Math.round(rover.lon * 1e5) : '');
    const liste = $('#c-lieux-liste');
    $('#c-lieu-form button').disabled = !rover;
    if (liste.dataset.cle === cle) return;
    liste.dataset.cle = cle;
    liste.replaceChildren(...lieux.map((lieu) => {
      const aller = el('button', { type: 'button', class: 'bouton petit', title: 'Aller' }, icone('cible'));
      const supprimer = el('button', { type: 'button', class: 'bouton petit', title: 'Supprimer' }, icone('annuler'));
      aller.addEventListener('click', async () => {
        if (!(await confirmer(`Aller à « ${lieu.nom} » ?`, aller))) return;
        signalerRefus(await this.ctx.connexion.demander({ type: 'lieu', action: 'aller', nom: lieu.nom }));
      });
      supprimer.addEventListener('click', async () => {
        if (!(await confirmer(`Supprimer « ${lieu.nom} » ?`, supprimer, { danger: true }))) return;
        signalerRefus(await this.ctx.connexion.demander({ type: 'lieu', action: 'supprimer', nom: lieu.nom }));
      });
      const distance = rover && situe(lieu) ? `${nombre(distanceM(rover.lat, rover.lon, lieu.lat, lieu.lon), 0)} m` : situe(lieu) ? '' : 'sans position GPS';
      return el('li', {}, el('span', { class: 'nom' }, lieu.nom, el('small', { text: distance })), aller, supprimer);
    }));
  }

  // ---------------------------------------------------------------- dessin

  dessiner() {
    if (this.fond === 'gps') {
      this.gps.dessiner();
      this._majEtatGPS();
      return;
    }
    this.plan.dessiner();
    const odom = this.ctx.donnees.liaison === 'ok' ? this.ctx.donnees.etat?.odom : null;
    texte($('#c-x'), odom ? `${nombre(odom.x, 2)} m` : '—');
    texte($('#c-y'), odom ? `${nombre(odom.y, 2)} m` : '—');
    texte($('#c-cap'), odom ? `${nombre(odom.cap / DEG, 0)}°` : '—');
    texte($('#c-qualite'), odom && Number.isFinite(odom.qualite) ? `${Math.round(odom.qualite * 100)} %` : '—');
    const vrai = this.ctx.donnees.liaison === 'ok' ? this.ctx.donnees.etat?.simulation : null;
    $('#c-verite-bloc').hidden = !vrai;
    if (vrai) texte($('#c-verite'), `${nombre(vrai.x, 2)} ; ${nombre(vrai.y, 2)} · ${nombre(vrai.cap / DEG, 0)}°`);
  }

  _majEtatGPS() {
    const g = this.ctx.donnees.liaison === 'ok' ? this.ctx.donnees.gps : null;
    texte($('#c-gps-fix'), g ? FIX_GNSS[g.fix] || g.fix || '—' : '—');
    texte($('#c-gps-sat'), g && Number.isFinite(g.satellites) ? String(g.satellites) : '—');
    texte($('#c-gps-hdop'), nombre(g?.hdop, 1));
    texte($('#c-gps-precision'), Number.isFinite(g?.rover?.precision_m) ? `${nombre(g.rover.precision_m, 1)} m` : '—');
    const a = g?.alignement;
    const bloc = $('#c-alignement');
    attribut(bloc, 'data-niveau', !a ? 'neutre' : a.ok ? 'ok' : 'alerte');
    texte($('#c-gps-alignement'), !a ? '—' : a.ok ? `${nombre(a.residu_m, 1)} m` : 'à faire');
    attribut(bloc, 'title', a ? `résidu ${nombre(a.residu_m, 1)} m · étendue ${nombre(a.etendue_m, 0)} m · ${a.points ?? 0} points` : null);
    $('#c-attribution').hidden = !lireLocal('tuiles', 'osm') || tuilesHorsLigne();
  }
}
