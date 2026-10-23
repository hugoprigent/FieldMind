// Commandes de l'atelier simulé uniquement. Aucun message type « drone » matériel.
export class ManetteDroneSimulee {
  constructor(envoyer) {
    this.envoyer = envoyer;
    this.touches = new Set();
    this.autorise = false;
  }
  definirAutorisation(v) {
    if (!v && this.autorise) this.relacherTout();
    this.autorise = !!v;
  }
  appuyer(nom) {
    if (!this.autorise || !['monter', 'descendre', 'avant', 'arriere', 'gauche', 'droite', 'tourner_gauche', 'tourner_droite'].includes(nom)) return;
    this.touches.add(nom);
    this.pas();
  }
  relacher(nom) { this.touches.delete(nom); this.pas(); }
  relacherTout() { this.touches.clear(); this.pas(); }
  pas() {
    if (!this.autorise) return;
    const a = (p, n) => Number(this.touches.has(p)) - Number(this.touches.has(n));
    this.envoyer({ type: 'drone_atelier', action: 'axes',
      montee: a('monter', 'descendre'), lacet: a('tourner_droite', 'tourner_gauche'),
      avant: a('avant', 'arriere'), droite: a('droite', 'gauche') });
  }
}

export class AtelierDroneVue {
  constructor(ctx) {
    this.ctx = ctx;
    this.manette = new ManetteDroneSimulee((m) => ctx.connexion.envoyer(m));
    this.racine = document.querySelector('#p-drone-atelier');
    if (!this.racine) return;
    this.racine.addEventListener('click', async (e) => {
      const b = e.target.closest('[data-atelier-action]');
      if (!b) return;
      const action = b.dataset.atelierAction;
      const r = await ctx.connexion.demander({ type: 'drone_atelier', action,
        scenario: this.racine.querySelector('select').value, vitesse: 1 });
      const erreur = this.racine.querySelector('[data-atelier-erreur]');
      erreur.textContent = r.ok ? '' : r.erreur || 'Demande refusée';
    });
    this.racine.addEventListener('pointerdown', (e) => {
      const b = e.target.closest('[data-atelier-axe]');
      if (!b || b.disabled) return;
      e.preventDefault();
      b.setPointerCapture(e.pointerId);
      this.manette.appuyer(b.dataset.atelierAxe);
    });
    for (const nom of ['pointerup', 'pointercancel', 'lostpointercapture']) {
      this.racine.addEventListener(nom, (e) => {
        const b = e.target.closest('[data-atelier-axe]');
        if (b) this.manette.relacher(b.dataset.atelierAxe);
      });
    }
    window.addEventListener('blur', () => this.manette.definirAutorisation(false));
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) this.manette.definirAutorisation(false);
    });
    ctx.donnees.sur('etat', () => this.actualiser());
    ctx.donnees.sur('liaison', () => this.actualiser());
    // Le neutre continue à être transmis au repos ; absence de battement = repli.
    this.minuterie = setInterval(() => this.manette.pas(), 100);
    this.actualiser();
  }
  actualiser() {
    if (!this.racine) return;
    const d = this.ctx.donnees.etat?.drone_atelier;
    const actif = this.ctx.donnees.liaison === 'ok' && !document.hidden && document.hasFocus();
    this.manette.definirAutorisation(actif && d?.mode === 'manuel' && d?.phase === 'manuel');
    for (const b of this.racine.querySelectorAll('[data-atelier-axe]')) b.disabled = !this.manette.autorise;
    for (const b of this.racine.querySelectorAll('[data-atelier-action="decoller"], [data-atelier-action="camera_bas"]')) {
      b.disabled = !this.manette.autorise;
    }
    const etat = this.racine.querySelector('[data-atelier-etat]');
    etat.textContent = d ? `SIMULATION · ${d.phase} · ${d.hauteur_m.toFixed(2)} m · ${d.cap_deg.toFixed(0)}° · caméra ${d.camera} · ${d.photos}/6 photos · ${d.paliers} paliers` : 'Prêt à simuler, même sans drone et sans Raspberry Pi';
    this.racine.querySelector('[data-atelier-raison]').textContent = d?.raison || '';
    const img = this.racine.querySelector('img');
    if (this.ctx.donnees.liaison !== 'ok' || !d) {
      img.removeAttribute('src');
      delete img.dataset.actif;
      img.hidden = true;
    }
    if (d && !img.dataset.actif) {
      const url = new URL('flux/drone_simule', location.href);
      if (this.ctx.connexion.jeton) url.searchParams.set('jeton', this.ctx.connexion.jeton);
      img.src = url.href;
      img.dataset.actif = '1';
      img.hidden = false;
    }
    for (const lien of this.racine.querySelectorAll('[data-atelier-fichier]')) {
      lien.hidden = !d?.archive;
      const url = new URL(`api/drone/atelier/${lien.dataset.atelierFichier}`, location.href);
      if (this.ctx.connexion.jeton) url.searchParams.set('jeton', this.ctx.connexion.jeton);
      lien.href = url.href;
    }
  }
}
