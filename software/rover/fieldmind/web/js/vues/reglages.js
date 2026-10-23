// Menu Réglages : réglages du rover rangés par groupe d'après `bienvenue.reglables_details`
// (repli : `reglables` et les libellés du poste), valeur retenue renvoyée par `reponse` ;
// fournisseur IA, inversion des caméras, voix, et options du poste (thème, jeton, carte).

import { $, $$, el, texte, attribut, nombre, signaler, signalerRefus, lireLocal, ecrireLocal, confirmer } from '../outils.js';
import { reglage } from '../textes.js';
import { enregistrerJeton } from '../connexion.js';
import { TUILES_OSM } from '../cartegps.js';

// Groupes rangés dans un panneau fixe du menu plutôt que dans une carte à part.
const PANNEAUX_FIXES = { 'Caméras': '#r-cameras', 'Voix': '#r-voix' };
// Réglages à boutons fins (−/+), appliqués tout de suite.
const FINS = { 'direction.servo_neutre_deg': 0.5 };

function pasRond(x) {
  if (!(x > 0)) return 1;
  const p = 10 ** Math.floor(Math.log10(x));
  const f = x / p;
  return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * p;
}

function decimalesPour(pas) {
  return Math.max(0, -Math.floor(Math.log10(pas) + 1e-9));
}

/** Valeur d'un chemin « section.nom » dans le profil. */
function valeurProfil(profil, cle) {
  return cle.split('.').reduce((n, p) => (n && typeof n === 'object' ? n[p] : undefined), profil);
}

/** Description complète d'un réglage : détails du serveur, sinon bornes et libellés du poste. */
export function decrireReglage(cle, bornes, details) {
  const local = reglage(cle);
  const d = details || {};
  const numerique = Array.isArray(bornes) && bornes.length === 2 && bornes.every((b) => typeof b === 'number');
  let type = d.type;
  if (!type) type = bornes === 'texte' || (!numerique && bornes !== undefined) ? 'texte' : numerique && bornes[0] === 0 && bornes[1] === 1 ? 'booleen' : 'nombre';
  const min = d.min ?? (numerique ? bornes[0] : undefined), max = d.max ?? (numerique ? bornes[1] : undefined);
  return {
    cle, type, min, max,
    libelle: d.libelle || local.libelle,
    unite: d.unite ?? local.unite ?? '',
    groupe: d.groupe || local.groupe,
    pas: d.pas || local.pas || (Number.isFinite(min) && Number.isFinite(max) ? pasRond((max - min) / 100) : 1),
  };
}

export class VueReglages {
  constructor(ctx) {
    this.ctx = ctx;
    const { donnees, connexion } = ctx;
    if (location.pathname.startsWith('/fieldmind/')) $('#r-ia-cle-form').hidden = true;

    $('#r-ia-profils').addEventListener('click', async (e) => {
      const b = e.target.closest('button[data-profil]');
      if (!b || b.getAttribute('aria-pressed') === 'true') return;
      const r = signalerRefus(await connexion.demander({ type: 'ia_profil', profil: b.dataset.profil }));
      if (r.ok) {
        const resume = r.valeur && typeof r.valeur === 'object' ? r.valeur : {};
        donnees.ia = { ...(donnees.ia || {}), profil: b.dataset.profil, ...resume };
        this._majIA();
      }
    });
    $('#r-ia-cle-form')?.addEventListener('submit', async (e) => {
      e.preventDefault();
      const champ = $('#r-ia-cle');
      const cle = champ.value.trim();
      const r = signalerRefus(await connexion.demander({ type: 'ia_cle', cle }));
      if (r.ok) {
        champ.value = '';
        const resume = r.valeur && typeof r.valeur === 'object' ? r.valeur : {};
        donnees.ia = { ...(donnees.ia || {}), ...resume };
        this._majIA();
        signaler(cle ? 'Clé API Gemini enregistrée' : 'Clé API retirée', 'succes');
      }
    });
    for (const type of ['bienvenue', 'ia']) donnees.sur(type, () => this._majIA());

    $('#r-inverser').addEventListener('click', async () => {
      const r = signalerRefus(await connexion.demander({ type: 'cameras_inverser' }));
      if (r.ok) signaler('Caméras inversées', 'succes');
    });
    $('#r-voix-active').addEventListener('click', () => ctx.basculerVoix());
    this._basculeLocale('#r-voix-lire', 'lire', false, !('speechSynthesis' in window));
    this._basculeLocale('#r-minicarte', 'minicarte', false);

    // Tuiles de la carte GPS (vide : pas de tuiles, quadrillage seul).
    const tuiles = $('#r-tuiles');
    const appliquerTuiles = (valeur) => {
      tuiles.value = valeur;
      ctx.option('tuiles', valeur.trim());
    };
    tuiles.value = lireLocal('tuiles', TUILES_OSM);
    $('#r-tuiles-form').addEventListener('submit', (e) => { e.preventDefault(); appliquerTuiles(tuiles.value); signaler('Tuiles enregistrées', 'succes'); });
    tuiles.addEventListener('change', () => appliquerTuiles(tuiles.value));
    $('#r-tuiles-osm').addEventListener('click', () => appliquerTuiles(TUILES_OSM));

    $('#r-theme').addEventListener('click', (e) => {
      const b = e.target.closest('button[data-theme]');
      if (b) ctx.appliquerTheme(b.dataset.theme);
    });
    ctx.surTheme((theme) => { for (const b of $$('#r-theme button')) attribut(b, 'aria-pressed', b.dataset.theme === theme ? 'true' : 'false'); });

    const jeton = $('#r-jeton');
    jeton.value = connexion.jeton || '';
    $('#r-jeton-form').addEventListener('submit', (e) => {
      e.preventDefault();
      enregistrerJeton(jeton.value.trim());
      connexion.definirJeton?.(jeton.value.trim());
      signaler('Jeton enregistré', 'succes');
    });
    jeton.addEventListener('change', () => $('#r-jeton-form').requestSubmit());

    $('#r-reinitialiser').addEventListener('click', async (e) => {
      if (!(await confirmer('Revenir aux réglages d’origine ? Branchement, montage et calibrations sont gardés.', e.currentTarget, { danger: true }))) return;
      const r = signalerRefus(await connexion.demander({ type: 'reglages_reinitialiser' }));
      if (r.ok) {
        signaler(`${(r.valeur || []).length} réglages remis à l’origine`, 'succes');
        setTimeout(() => location.reload(), 800); // le poste repart des valeurs du rover
      }
    });

    $('#r-environnement').addEventListener('click', async (e) => {
      if (!(await confirmer('Effacer la carte, les objets vus, la position et les missions ? Réglages, calibrations et lieux nommés sont gardés.', e.currentTarget, { danger: true }))) return;
      const r = signalerRefus(await connexion.demander({ type: 'environnement_reinitialiser' }));
      if (r.ok) signaler('Environnement réinitialisé', 'succes');
    });

    donnees.sur('bienvenue', () => this._construireGroupes());
    donnees.sur('etat', () => this._majEtat());
    this._majIA();
  }

  activer() {
    this._majEtat();
  }

  desactiver() {}

  dessiner() {}

  _basculeLocale(selecteur, nom, defaut, desactive = false) {
    const b = $(selecteur);
    attribut(b, 'aria-checked', lireLocal(nom, defaut) ? 'true' : 'false');
    b.disabled = desactive;
    b.addEventListener('click', () => {
      const v = !lireLocal(nom, defaut);
      this.ctx.option(nom, v);
      attribut(b, 'aria-checked', v ? 'true' : 'false');
    });
  }

  // ---------------------------------------------------------------- IA

  _majIA() {
    const ia = this.ctx.donnees.ia;
    const profils = ia?.profils?.length ? ia.profils : ['gemini', 'gx10', 'aucun'];
    const conteneur = $('#r-ia-profils');
    if (conteneur.dataset.cle !== profils.join(',')) {
      conteneur.dataset.cle = profils.join(',');
      conteneur.replaceChildren(...profils.map((p) => el('button', { type: 'button', dataset: { profil: p } }, p === 'aucun' ? 'Aucun' : p === 'gx10' ? 'GX10' : p === 'gemini' ? 'Gemini' : p)));
    }
    for (const b of $$('#r-ia-profils button')) attribut(b, 'aria-pressed', b.dataset.profil === ia?.profil ? 'true' : 'false');
    const pastille = $('#r-ia-etat');
    const [niveau, libelle] = !ia ? ['neutre', '—'] : ia.profil === 'aucun' ? ['neutre', 'Désactivée']
      : ia.disponible === false ? ['alerte', 'Indisponible'] : ['ok', 'Disponible'];
    attribut(pastille, 'data-niveau', niveau);
    texte(pastille.querySelector('.libelle'), libelle);
    const modeles = ia?.modeles || {};
    const lignes = [['Agent', modeles.agent], ['Vision', modeles.vision], ['Transcription', modeles.transcription]];
    if (ia?.raison) lignes.push(['Raison', ia.raison]);
    $('#r-ia-modeles').replaceChildren(...lignes.flatMap(([l, v]) => [el('dt', { text: l }), el('dd', { text: v || '—', title: v || '' })]));
  }

  // ---------------------------------------------------------------- réglages du rover

  _valeur(cle) {
    const d = this.ctx.donnees;
    return d.reglages[cle] ?? valeurProfil(d.profil, cle);
  }

  _construireGroupes() {
    const d = this.ctx.donnees;
    const details = d.bienvenue?.reglables_details || {};
    const cles = [...new Set([...Object.keys(d.reglables || {}), ...Object.keys(details)])];
    const groupes = new Map();
    for (const cle of cles) {
      const info = decrireReglage(cle, d.reglables?.[cle], details[cle]);
      if (!groupes.has(info.groupe)) groupes.set(info.groupe, []);
      groupes.get(info.groupe).push(info);
    }
    const conteneur = $('#r-groupes');
    conteneur.replaceChildren();
    for (const e of $$('[data-genere]')) e.remove();
    for (const [groupe, infos] of groupes) {
      const fixe = PANNEAUX_FIXES[groupe] ? $(PANNEAUX_FIXES[groupe]) : null;
      const panneau = fixe || el('article', { class: 'panneau' }, el('header', {}, el('h2', { text: groupe })));
      for (const info of infos) {
        const controle = this._controle(info);
        controle.dataset.genere = 'true';
        panneau.append(controle);
      }
      if (!fixe) conteneur.append(panneau);
    }
    texte($('#r-version'), [d.bienvenue?.version ? `v${d.bienvenue.version}` : null, d.materiel].filter(Boolean).join(' · ') || '—');
  }

  _controle(info) {
    const { cle } = info;
    if (info.type === 'booleen') {
      const bascule = el('button', { type: 'button', class: 'bascule', role: 'switch', 'aria-checked': this._valeur(cle) ? 'true' : 'false', 'aria-label': info.libelle }, el('span'));
      bascule.addEventListener('click', () => this._basculerReglage(cle, bascule));
      return el('div', { class: 'ligne-reglage' }, el('span', { text: info.libelle, title: cle }), bascule);
    }
    if (info.type === 'texte' || !Number.isFinite(info.min) || !Number.isFinite(info.max)) {
      const champ = el('input', { type: 'text', value: this._valeur(cle) ?? '', spellcheck: 'false', 'aria-label': info.libelle, placeholder: /port|peripherique/.test(cle) ? 'auto' : '' });
      const ligne = el('div', { class: 'curseur' }, el('span', { class: 'nom', text: info.libelle, title: cle }), el('output'), champ);
      champ.addEventListener('change', () => this._envoyer(cle, champ.value, ligne, (v) => { champ.value = v ?? ''; }));
      return ligne;
    }
    const { min, max, pas } = info;
    const decimales = Math.max(decimalesPour(pas), FINS[cle] ? decimalesPour(FINS[cle]) : 0);
    const curseur = el('input', { type: 'range', min, max, step: pas, 'aria-label': info.libelle });
    const sortie = el('output');
    const montrer = (v) => sortie.replaceChildren(nombre(Number(v), decimales), info.unite ? el('small', { text: info.unite }) : '');
    const initial = Number(this._valeur(cle));
    curseur.value = Number.isFinite(initial) ? initial : min;
    montrer(curseur.value);
    const appliquer = (v) => {
      curseur.value = v;
      montrer(v);
    };
    const enfants = [el('span', { class: 'nom', text: info.libelle, title: cle })];
    const ligne = el('div', { class: 'curseur' });
    if (FINS[cle]) {
      // Boutons fins : chaque clic part tout de suite (les roues bougent à l'arrêt : voulu).
      const fin = FINS[cle];
      const pasFin = (sens) => {
        const base = Number(this.ctx.donnees.reglages[cle] ?? curseur.value);
        const v = Math.min(max, Math.max(min, Math.round((base + sens * fin) / fin) * fin));
        this.ctx.donnees.reglages[cle] = v; // base des clics suivants, avant la réponse
        appliquer(v);
        this._envoyer(cle, v, ligne, appliquer, base);
      };
      const moins = el('button', { type: 'button', class: 'bouton', title: `−${nombre(fin, 1)}${info.unite}`, 'aria-label': `${info.libelle} moins` }, '−');
      const plus = el('button', { type: 'button', class: 'bouton', title: `+${nombre(fin, 1)}${info.unite}`, 'aria-label': `${info.libelle} plus` }, '+');
      moins.addEventListener('click', () => pasFin(-1));
      plus.addEventListener('click', () => pasFin(1));
      enfants.push(el('span', { class: 'fin' }, moins, plus));
      ligne.classList.add('avec-fin');
    }
    enfants.push(sortie, el('div', { class: 'bornes' }, el('span', { text: nombre(min, decimalesPour(pas)) }), curseur, el('span', { text: nombre(max, decimalesPour(pas)) })));
    ligne.append(...enfants);
    curseur.addEventListener('input', () => montrer(curseur.value));
    curseur.addEventListener('change', () => this._envoyer(cle, Number(curseur.value), ligne, appliquer));
    return ligne;
  }

  async _envoyer(cle, valeur, ligne, appliquer, avant = this._valeur(cle)) {
    attribut(ligne, 'data-etat', 'envoi');
    const r = await this.ctx.connexion.demander({ type: 'reglage', cle, valeur });
    if (r.ok) {
      const retenue = r.valeur !== undefined ? r.valeur : valeur;
      this.ctx.donnees.reglages[cle] = retenue;
      appliquer(retenue);
      attribut(ligne, 'data-etat', null);
      this.ctx.donnees.emettre('reglage', { cle, valeur: retenue });
    } else {
      signalerRefus(r);
      this.ctx.donnees.reglages[cle] = avant;
      appliquer(avant);
      attribut(ligne, 'data-etat', 'refus');
      setTimeout(() => attribut(ligne, 'data-etat', null), 2500);
    }
  }

  async _basculerReglage(cle, bascule) {
    const nouvelle = !this._valeur(cle);
    const r = signalerRefus(await this.ctx.connexion.demander({ type: 'reglage', cle, valeur: nouvelle }));
    if (r.ok) {
      const retenue = r.valeur !== undefined ? Boolean(r.valeur) : nouvelle;
      this.ctx.donnees.reglages[cle] = retenue;
      this.ctx.donnees.emettre('reglage', { cle, valeur: retenue });
      attribut(bascule, 'aria-checked', retenue ? 'true' : 'false');
    }
  }

  _majEtat() {
    const d = this.ctx.donnees, e = d.liaison === 'ok' ? d.etat : null;
    attribut($('#r-voix-active'), 'aria-checked', e?.voix?.active ? 'true' : 'false');
    texte($('#r-micro'), e?.capteurs?.micro?.nom || (e?.capteurs?.micro?.ok === false ? 'absent' : '—'));
  }
}
