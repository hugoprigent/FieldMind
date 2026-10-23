// Menu Système : processeur, température, alimentation, Wi-Fi ; journal filtrable ; console
// de l'ESP ; lidar marche/arrêt ; calibrations ; banc (PWM bruts, homme mort à 5 Hz).

import { $, $$, el, icone, texte, attribut, nombre, heure, duree, confirmer, signaler, signalerRefus } from '../outils.js';
import { CALIBRATIONS } from '../textes.js';
import { AssistantStereo } from './assistant_stereo.js';

const TUILES = [
  ['cpu', 'Processeur', 'systeme'],
  ['temperature', 'Température', 'alerte'],
  ['memoire', 'Mémoire', 'systeme'],
  ['disque', 'Disque', 'journal'],
  ['alimentation', 'Alimentation', 'arme'],
  ['wifi', 'Wi-Fi', 'wifi'],
  ['charge', 'Charge', 'capteurs'],
  ['uptime', 'En marche', 'mode'],
];

const ICONES_JOURNAL = { info: 'info', succes: 'ok', alerte: 'alerte', erreur: 'alerte' };

function resumeResultat(resultat) {
  if (!resultat || typeof resultat !== 'object') return '';
  return Object.entries(resultat).map(([cle, v]) => {
    const valeur = Array.isArray(v) ? v.map((x) => (typeof x === 'number' ? nombre(x, 3) : x)).join(' ; ')
      : typeof v === 'number' ? nombre(v, Math.abs(v) >= 100 ? 0 : Math.abs(v) >= 10 ? 1 : 3)
        : typeof v === 'boolean' ? (v ? 'oui' : 'non') : String(v);
    return `${cle.replaceAll('_', ' ')} ${valeur}`;
  }).join(' · ');
}

export class VueSysteme {
  constructor(ctx) {
    this.ctx = ctx;
    this.actif = false;
    const { donnees, connexion } = ctx;

    // Tuiles.
    this.tuiles = {};
    for (const [cle, libelle, nomIcone] of TUILES) {
      const t = el('div', { class: 'tuile', dataset: { niveau: 'neutre' } },
        el('span', { class: 'libelle' }, icone(nomIcone), libelle),
        el('span', { class: 'valeur', text: '—' }),
        el('span', { class: 'jauge' }, el('span')),
        el('span', { class: 'detail' }));
      $('#s-tuiles').append(t);
      this.tuiles[cle] = t;
    }
    donnees.sur('systeme', () => this._majSysteme());
    donnees.sur('liaison', () => this._majSysteme());

    // Journal.
    this.filtres = new Set(['info', 'succes', 'alerte', 'erreur']);
    $('#s-filtres').addEventListener('click', (e) => {
      const b = e.target.closest('button[data-niveau]');
      if (!b) return;
      if (this.filtres.has(b.dataset.niveau)) this.filtres.delete(b.dataset.niveau);
      else this.filtres.add(b.dataset.niveau);
      attribut(b, 'aria-pressed', this.filtres.has(b.dataset.niveau) ? 'true' : 'false');
      this._rendreJournal();
    });
    $('#s-source').addEventListener('change', () => this._rendreJournal());
    $('#s-recherche').addEventListener('input', () => this._rendreJournal());
    donnees.sur('journal', (m) => this._ajouterJournal(m));
    donnees.sur('bienvenue', (m) => {
      if (m.nouvelleSession) $('#s-console').replaceChildren();
      this._rendreJournal();
    });

    // Console de l'ESP.
    this.historique = [];
    this.indexHistorique = 0;
    donnees.sur('console', (m) => this._ajouterConsole(m.lignes || []));
    $('#s-console-form').addEventListener('submit', async (e) => {
      e.preventDefault();
      const champ = $('#s-console-texte');
      const ligne = champ.value.trim();
      if (!ligne) return;
      this.historique.push(ligne);
      this.indexHistorique = this.historique.length;
      champ.value = '';
      const r = await connexion.demander({ type: 'console', texte: ligne });
      if (!r.ok) {
        signalerRefus(r);
        this._ajouterConsole([[Date.now() / 1000, `✕ ${ligne} : ${r.erreur || 'refusé'}`]], 'refus');
      }
    });
    $('#s-console-texte').addEventListener('keydown', (e) => {
      if (e.key !== 'ArrowUp' && e.key !== 'ArrowDown') return;
      e.preventDefault();
      this.indexHistorique = Math.max(0, Math.min(this.historique.length, this.indexHistorique + (e.key === 'ArrowUp' ? -1 : 1)));
      e.target.value = this.historique[this.indexHistorique] || '';
    });

    // Lidar.
    $('#s-lidar').addEventListener('click', async () => {
      const marche = donnees.etat?.capteurs?.lidar?.marche !== false;
      signalerRefus(await connexion.demander({ type: 'lidar', marche: !marche }));
    });

    // Calibrations.
    this.assistantStereo = new AssistantStereo(ctx);
    for (const c of CALIBRATIONS) {
      const bouton = el('button', { type: 'button', class: 'bouton petit' }, icone('lecture'), el('span', { text: 'Lancer' }));
      const li = el('li', { dataset: { nom: c.nom, etat: '' } },
        el('span', { class: 'nom' }, icone(c.icone), c.libelle), bouton, el('span', { class: 'retour' }));
      bouton.addEventListener('click', () => this._calibrer(c, li, bouton));
      $('#s-calibrations').append(li);
    }
    donnees.sur('calibration', (m) => this._majCalibration(m));

    this._installerBanc();
    donnees.sur('etat', () => this._majEtat());
  }

  activer() {
    this.actif = true;
    this._rendreJournal();
    const console = $('#s-console');
    console.scrollTop = console.scrollHeight;
    this._majSysteme();
    this._majEtat();
  }

  desactiver() {
    this.actif = false;
    this._arreterBanc();
  }

  dessiner() {}

  // ---------------------------------------------------------------- système

  _tuile(cle, valeur, niveau = 'neutre', detail = '', jauge = null) {
    const t = this.tuiles[cle];
    attribut(t, 'data-niveau', niveau);
    texte(t.querySelector('.valeur'), valeur);
    texte(t.querySelector('.detail'), detail);
    const barre = t.querySelector('.jauge');
    barre.hidden = jauge === null;
    if (jauge !== null) barre.firstElementChild.style.width = `${Math.max(0, Math.min(100, jauge))}%`;
  }

  _majSysteme() {
    const d = this.ctx.donnees;
    const s = d.liaison === 'ok' ? d.systeme : null;
    const pct = (v, alerte, critique) => (!Number.isFinite(v) ? 'neutre' : v >= critique ? 'critique' : v >= alerte ? 'alerte' : 'ok');
    this._tuile('cpu', Number.isFinite(s?.cpu_pct) ? `${Math.round(s.cpu_pct)} %` : '—', pct(s?.cpu_pct, 80, 95),
      Number.isFinite(s?.processus_cpu_pct) ? `rover ${Math.round(s.processus_cpu_pct)} %` : '', s?.cpu_pct ?? null);
    this._tuile('temperature', Number.isFinite(s?.temperature_c) ? `${nombre(s.temperature_c, 1)} °C` : '—', pct(s?.temperature_c, 70, 80),
      '', Number.isFinite(s?.temperature_c) ? (s.temperature_c / 85) * 100 : null);
    this._tuile('memoire', Number.isFinite(s?.memoire_pct) ? `${Math.round(s.memoire_pct)} %` : '—', pct(s?.memoire_pct, 80, 92), '', s?.memoire_pct ?? null);
    this._tuile('disque', Number.isFinite(s?.disque_pct) ? `${Math.round(s.disque_pct)} %` : '—', pct(s?.disque_pct, 85, 95), '', s?.disque_pct ?? null);
    if (!s) this._tuile('alimentation', '—');
    else if (s.sous_tension) this._tuile('alimentation', 'Sous-tension', 'critique', s.throttled || '');
    else if (s.ralenti) this._tuile('alimentation', 'Bridée', 'alerte', s.throttled || '');
    else this._tuile('alimentation', 'Normale', 'ok', s.throttled && s.throttled !== '0x0' ? `déjà vu ${s.throttled}` : s.throttled || '');
    const w = s?.wifi;
    this._tuile('wifi', Number.isFinite(w?.signal_pct) ? `${Math.round(w.signal_pct)} %` : '—',
      !Number.isFinite(w?.signal_pct) ? 'neutre' : w.signal_pct >= 50 ? 'ok' : w.signal_pct >= 25 ? 'alerte' : 'critique',
      [w?.ssid, w?.ip].filter(Boolean).join(' · '), w?.signal_pct ?? null);
    const charge = Array.isArray(s?.charge) ? s.charge : null;
    this._tuile('charge', charge ? nombre(charge[0], 2) : '—', 'neutre', charge ? `5′ ${nombre(charge[1], 2)} · 15′ ${nombre(charge[2], 2)}` : '');
    this._tuile('uptime', Number.isFinite(s?.uptime_s) ? duree(s.uptime_s) : '—', 'neutre', d.bienvenue?.version ? `v${d.bienvenue.version}` : '');
  }

  // ---------------------------------------------------------------- journal

  _correspond(m) {
    if (!this.filtres.has(m.niveau || 'info')) return false;
    const source = $('#s-source').value;
    if (source && m.source !== source) return false;
    const q = $('#s-recherche').value.trim().toLowerCase();
    return !q || `${m.source} ${m.message}`.toLowerCase().includes(q);
  }

  _ligneJournal(m) {
    return el('li', { dataset: { niveau: m.niveau || 'info' } },
      el('span', { class: 't', text: heure(m.t) }), icone(ICONES_JOURNAL[m.niveau] || 'info'),
      el('span', { class: 'src', text: m.source || '' }), el('span', { class: 'msg', text: m.message || '' }));
  }

  _sources() {
    const select = $('#s-source');
    const connues = new Set([...select.options].map((o) => o.value));
    for (const m of this.ctx.donnees.journal) {
      if (m.source && !connues.has(m.source)) {
        connues.add(m.source);
        select.append(el('option', { value: m.source, text: m.source }));
      }
    }
  }

  _rendreJournal() {
    this._sources();
    const liste = $('#s-journal');
    liste.replaceChildren(...this.ctx.donnees.journal.filter((m) => this._correspond(m)).map((m) => this._ligneJournal(m)));
    liste.scrollTop = liste.scrollHeight;
  }

  _ajouterJournal(m) {
    if (!this.actif) return;
    this._sources();
    if (!this._correspond(m)) return;
    const liste = $('#s-journal');
    const enBas = liste.scrollHeight - liste.scrollTop - liste.clientHeight < 30;
    liste.append(this._ligneJournal(m));
    while (liste.childElementCount > 1000) liste.firstElementChild.remove();
    if (enBas) liste.scrollTop = liste.scrollHeight;
  }

  // ---------------------------------------------------------------- console

  _ajouterConsole(lignes, classe = '') {
    const console = $('#s-console');
    const enBas = console.scrollHeight - console.scrollTop - console.clientHeight < 30;
    const fragment = document.createDocumentFragment();
    for (const [t, ligne] of lignes) {
      const texteLigne = String(ligne ?? '');
      const envoi = texteLigne.startsWith('→');
      fragment.append(el('div', { class: envoi ? 'envoi' : classe }, el('span', { class: 't', text: `${heure(t)} ` }), texteLigne));
    }
    console.append(fragment);
    while (console.childElementCount > 600) console.firstElementChild.remove();
    if (enBas) console.scrollTop = console.scrollHeight;
  }

  // ---------------------------------------------------------------- calibrations

  async _calibrer(c, li, bouton) {
    if (c.nom === 'stereo_assistant') { this.assistantStereo.ouvrir(); return; }
    const enCours = li.dataset.etat === 'en_cours';
    if (enCours) {
      signalerRefus(await this.ctx.connexion.demander({ type: 'calibration', nom: c.nom, arreter: true }));
      return;
    }
    if (c.nom === 'sens_roues') {  // demande le banc : y passer d'ici, après confirmation
      if (!(await this._entrerBanc(bouton, 'Roues levées ? Le rover passe en banc et fait tourner chaque roue.'))) return;
    } else if (c.confirmer && !(await confirmer(c.confirmer, bouton))) return;
    if (c.nom.startsWith('stereo_')) this.derniereStereo = c.nom;
    this._majCalibration({ nom: c.nom, etat: 'en_cours', message: '…' });
    const r = await this.ctx.connexion.demander({ type: 'calibration', nom: c.nom });
    if (!r.ok) {
      signalerRefus(r);
      this._majCalibration({ nom: c.nom, etat: 'echec', message: r.erreur || 'refusé' });
    } else if (c.nom === 'stereo_capture' && li.dataset.etat === 'en_cours' && li.querySelector('.retour').textContent === '…') {
      this._majCalibration({ nom: c.nom, etat: 'termine', message: Number.isFinite(r.valeur) ? `${r.valeur} paires` : 'paire enregistrée' });
    }
  }

  _majCalibration(m) {
    // Le serveur annonce les deux étapes stéréo sous le nom « stereo ».
    const nom = m.nom === 'stereo' ? 'stereo_assistant' : m.nom;
    const li = $(`#s-calibrations li[data-nom="${CSS.escape(String(nom))}"]`);
    if (!li) return;
    // Stéréo : l'assistant mène les captures ; la ligne ne montre que le dernier bilan, sans
    // passer en « en cours » (le bouton reste « Lancer », qui rouvre l'assistant)
    const etat = nom === 'stereo_assistant' && m.etat === 'en_cours' ? '' : m.etat || '';
    li.dataset.etat = etat;
    const bouton = li.querySelector('button');
    const enCours = etat === 'en_cours';
    bouton.replaceChildren(icone(enCours ? 'arret' : 'lecture'), el('span', { text: enCours ? 'Arrêter' : 'Lancer' }));
    const morceaux = [m.message, resumeResultat(m.resultat)].filter(Boolean);
    texte(li.querySelector('.retour'), morceaux.join(' — '));
  }

  // ---------------------------------------------------------------- banc

  _installerBanc() {
    const curseurs = { g: $('#s-banc-g'), d: $('#s-banc-d'), s: $('#s-banc-s') };
    const afficher = () => {
      texte($('#s-banc-g-v'), `${curseurs.g.value} %`);
      texte($('#s-banc-d-v'), `${curseurs.d.value} %`);
      texte($('#s-banc-s-v'), `${curseurs.s.value}°`);
    };
    const bornes = () => {
      const profil = this.ctx.donnees.profil;
      const pwm = profil.traction?.pwm_max_banc ?? 15, servo = profil.direction?.servo_max_deg ?? 40;
      for (const c of [curseurs.g, curseurs.d]) { c.min = -pwm; c.max = pwm; }
      curseurs.s.min = -servo; curseurs.s.max = servo;
      afficher();
    };
    bornes();
    this.ctx.donnees.sur('bienvenue', bornes);
    for (const c of Object.values(curseurs)) c.addEventListener('input', afficher);
    $('#s-banc-zero').addEventListener('click', () => {
      for (const c of Object.values(curseurs)) c.value = 0;
      afficher();
    });
    // Entrer en banc (et en sortir) d'ici : le mode n'est plus dans Pilotage
    $('#s-banc-mode').addEventListener('click', (e) => this._basculerBanc(e.currentTarget));
    const bouton = $('#s-banc-go');
    this.minuterieBanc = null;
    const envoyer = () => this.ctx.connexion.envoyer({
      type: 'banc', pwm_g: Number(curseurs.g.value), pwm_d: Number(curseurs.d.value), servo_deg: Number(curseurs.s.value),
    });
    bouton.addEventListener('pointerdown', (e) => {
      if (this.ctx.donnees.etat?.mode !== 'banc') { signaler('Passer en banc', 'alerte', 2000); return; }
      bouton.setPointerCapture(e.pointerId);
      attribut(bouton, 'data-actif', 'true');
      envoyer();
      clearInterval(this.minuterieBanc);
      this.minuterieBanc = setInterval(envoyer, 200); // 5 Hz, homme mort côté serveur 0,5 s
    });
    for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) bouton.addEventListener(type, () => this._arreterBanc());
    this.ctx.surArret(() => this._arreterBanc());
  }

  /** Banc : roues levées, armé. `pour` : texte de la confirmation. Renvoie vrai si en banc. */
  async _entrerBanc(declencheur, pour = 'Passer en banc ? Roues levées.') {
    const e = this.ctx.donnees.etat;
    if (e?.mode === 'banc' && e?.arme) return true;
    if (!(await confirmer(pour, declencheur, { danger: true }))) return false;
    if (e?.mode !== 'banc') {
      const r = signalerRefus(await this.ctx.connexion.demander({ type: 'mode', mode: 'banc', confirmer: true }));
      if (!r.ok) return false;
    }
    const r = signalerRefus(await this.ctx.connexion.demander({ type: 'armer' }));
    return Boolean(r.ok);
  }

  async _basculerBanc(bouton) {
    if (this.ctx.donnees.etat?.mode === 'banc') {
      this._arreterBanc();
      signalerRefus(await this.ctx.connexion.demander({ type: 'mode', mode: 'arret' }));
      return;
    }
    await this._entrerBanc(bouton);
  }

  _arreterBanc() {
    if (this.minuterieBanc === null) return;
    clearInterval(this.minuterieBanc);
    this.minuterieBanc = null;
    attribut($('#s-banc-go'), 'data-actif', 'false');
    this.ctx.connexion.envoyer({ type: 'banc', pwm_g: 0, pwm_d: 0, servo_deg: Number($('#s-banc-s').value) });
  }

  _majEtat() {
    const enBanc = this.ctx.donnees.etat?.mode === 'banc';
    const boutonBanc = $('#s-banc-mode');
    texte(boutonBanc.querySelector('span'), enBanc ? 'Quitter le banc' : 'Passer en banc');
    attribut($('#s-banc'), 'data-actif', enBanc ? 'true' : 'false');
    if (!this.actif) return;
    const d = this.ctx.donnees, e = d.liaison === 'ok' ? d.etat : null;
    const lidar = e?.capteurs?.lidar;
    const bascule = $('#s-lidar');
    attribut(bascule, 'aria-checked', lidar && lidar.marche !== false ? 'true' : 'false');
    bascule.disabled = !e;
    attribut($('#s-banc'), 'data-actif', e?.mode === 'banc' ? 'true' : 'false');
    texte($('#s-console-info'), e?.esp?.firmware ? `${e.esp.firmware}${e.esp.connecte ? '' : ' · absent'}` : '');
    for (const b of $$('#s-calibrations button')) b.disabled = !e;
  }
}
