// Menu Mission / IA : conversation (texte et voix), plan et file d'actions, commandes de la
// mission, actions rapides et panneau vision.

import { $, $$, el, icone, texte, attribut, nombre, heure, signaler, signalerRefus, couleur, preparerCanevas, lireLocal } from '../outils.js';
import { decrireAction, ETATS_MISSION } from '../textes.js';
import { cadreContenu, versParent } from '../geometrie.js';
import { VueCamera } from '../camera.js';

export class VueMission {
  constructor(ctx) {
    this.ctx = ctx;
    this.actif = false;
    this.cameraVision = 'gauche';
    this.fige = false;
    // Image en direct de la caméra choisie ; une analyse de l'IA la fige jusqu'au bouton « Direct ».
    this.direct = new VueCamera($('#m-direct'), ctx, { camera: this.cameraVision, ips: 8, largeur: 640 });
    this.premierChat = performance.now();
    const { donnees, connexion } = ctx;

    // Conversation.
    $('#m-saisie').addEventListener('submit', async (e) => {
      e.preventDefault();
      const champ = $('#m-texte');
      const consigne = champ.value.trim();
      if (!consigne) return;
      champ.value = '';
      donnees.ajouterChatLocal(consigne);
      const r = await connexion.demander({ type: 'chat', texte: consigne });
      if (!r.ok && !r.delai) signalerRefus(r);
    });
    $('#m-voix').addEventListener('click', () => ctx.basculerVoix());
    donnees.sur('chat', (m) => { this._rendreChat(); this._lireAVoixHaute(m); });
    donnees.sur('parole', (m) => this._jouerParole(m));
    donnees.sur('bienvenue', () => this._rendreChat());

    // Mission.
    const commande = (type) => async () => signalerRefus(await connexion.demander({ type }));
    $('#m-pause').addEventListener('click', commande('pause'));
    $('#m-reprendre').addEventListener('click', commande('reprendre'));
    $('#m-annuler').addEventListener('click', commande('annuler'));
    $('#m-rapides').addEventListener('click', async (e) => {
      const b = e.target.closest('button[data-action]');
      if (!b) return;
      const action = JSON.parse(b.dataset.action);
      const r = signalerRefus(await connexion.demander({ type: 'action', action }));
      if (r.ok) signaler(decrireAction(action), 'info', 1800);
    });
    donnees.sur('etat', () => this._majMission());
    donnees.sur('liaison', () => this._majMission());

    // Vision.
    $('#m-vision-camera').addEventListener('click', (e) => {
      const b = e.target.closest('button[data-camera]');
      if (!b) return;
      this.cameraVision = b.dataset.camera;
      for (const x of $$('#m-vision-camera button')) attribut(x, 'aria-pressed', x === b ? 'true' : 'false');
      this.direct.definirCamera(this.cameraVision);
      this._figer(false);
    });
    $('#m-retour-direct').addEventListener('click', () => this._figer(false));
    $('#m-decrire').addEventListener('click', () => this._demanderVision({ demande: 'decrire' }));
    $('#m-analyser')?.addEventListener('click', () => this._demanderVision({ demande: 'analyser' }));
    $('#m-chercher').addEventListener('submit', (e) => {
      e.preventDefault();
      const objet = $('#m-objet').value.trim();
      if (!objet) { $('#m-objet').focus(); return; }
      this._demanderVision({ demande: 'chercher', objet });
    });
    donnees.sur('vision', () => this._rendreVision());
    $('#m-vision-img').addEventListener('load', () => this._dessinerVision());
    new ResizeObserver(() => this._dessinerVision()).observe($('#m-vision-cadre'));
    this._rendreChat();
    this._majMission();
  }

  activer() {
    this.actif = true;
    if (!this.fige) this.direct.activer();
    const fil = $('#m-fil');
    fil.scrollTop = fil.scrollHeight;
    this._dessinerVision();
  }

  desactiver() {
    this.actif = false;
    this.direct.desactiver();
  }

  dessiner() {
    const ia = this.ctx.donnees.etat?.ia;
    const occupee = this.ctx.donnees.liaison === 'ok' && (ia?.consigne_en_cours ?? ia?.occupee);
    $('#m-occupee').hidden = !occupee;
    this.direct.dessiner();
  }

  // ---------------------------------------------------------------- conversation

  _rendreChat() {
    const fil = $('#m-fil');
    const enBas = fil.scrollHeight - fil.scrollTop - fil.clientHeight < 40;
    fil.replaceChildren(...this.ctx.donnees.chat.map((m) => this._message(m)));
    if (enBas || this.actif) fil.scrollTop = fil.scrollHeight;
  }

  _message(m) {
    const entete = el('div', { class: 'entete' },
      m.origine === 'voix' ? icone('micro') : null,
      m.origine === 'voix' ? el('span', { text: 'voix' }) : null,
      el('span', { text: heure(m.t) }));
    const li = el('li', { dataset: { role: m.role || 'systeme', local: m.local ? 'true' : null } });
    if (m.role !== 'systeme') li.append(entete);
    li.append(el('div', { class: 'texte', text: m.texte || '' }));
    const actions = m.plan?.actions;
    if (Array.isArray(actions) && actions.length) {
      li.append(el('ol', { class: 'plan' }, actions.map((a) => el('li', { text: decrireAction(a) }))));
    }
    return li;
  }

  /** Voix du rover synthétisée sur le GX10 (Qwen3-TTS) : remplace celle du navigateur. */
  _jouerParole(m) {
    if (!m.audio || !lireLocal('lire', false)) return;
    if (Number.isFinite(m.t) && Math.abs(Date.now() / 1000 - m.t) > 30) return; // historique
    // Phrases jouées à la suite ; une nouvelle réponse coupe l'ancienne
    if (m.id !== this.paroleId) {
      this.paroleId = m.id;
      this.paroles = [];
      this.paroleEnCours?.pause();
      this.paroleEnCours = null;
    }
    this.paroles.push(`data:audio/${m.format || 'wav'};base64,${m.audio}`);
    const suivante = () => {
      if (this.paroleEnCours || !this.paroles.length) return;
      this.paroleEnCours = new Audio(this.paroles.shift());
      const fin = () => { this.paroleEnCours = null; suivante(); };
      this.paroleEnCours.addEventListener('ended', fin);
      this.paroleEnCours.play().catch(fin);
    };
    suivante();
  }

  _lireAVoixHaute(m) {
    if (this.ctx.donnees.ia?.modeles?.synthese) return; // la voix vient du GX10 (_jouerParole)
    if (m.role !== 'rover' || !m.texte || !lireLocal('lire', false) || !('speechSynthesis' in window)) return;
    if (Number.isFinite(m.t) && Math.abs(Date.now() / 1000 - m.t) > 20) return; // historique
    const enonce = new SpeechSynthesisUtterance(m.texte);
    enonce.lang = 'fr-FR';
    speechSynthesis.speak(enonce);
  }

  // ---------------------------------------------------------------- mission

  _majMission() {
    const d = this.ctx.donnees;
    const m = d.liaison === 'ok' ? d.etat?.mission : null;
    const code = m ? (m.pause || m.etat === 'pause' ? 'pause' : m.etat) : null;
    const [libelle, niveau] = code ? ETATS_MISSION[code] || [code, 'neutre'] : ['—', 'neutre'];
    const pastille = $('#m-etat');
    attribut(pastille, 'data-niveau', niveau);
    texte(pastille.querySelector('.libelle'), libelle);
    texte($('#m-action'), m?.action ? decrireAction(m.action) : m ? 'Aucune action en cours' : '—');
    const progression = Number.isFinite(m?.progression) ? Math.min(1, Math.max(0, m.progression)) : 0;
    $('#m-barre').style.width = `${Math.round((m?.action ? progression : code === 'termine' ? 1 : 0) * 100)}%`;
    attribut($('#m-barre').parentElement, 'data-niveau', code === 'bloque' || code === 'pause' ? 'alerte' : code === 'echec' ? 'critique' : null);
    texte($('#m-detail'), m?.detail || '');
    const file = Array.isArray(m?.file) ? m.file : [];
    const liste = $('#m-file');
    const cle = JSON.stringify(file);
    if (liste.dataset.cle !== cle) {
      liste.dataset.cle = cle;
      liste.replaceChildren(...file.map((a) => el('li', { text: decrireAction(a) })));
    }
    const enCours = Boolean(m?.action) || file.length > 0;
    $('#m-pause').disabled = !enCours || code === 'pause';
    $('#m-reprendre').disabled = !enCours || code === 'route';
    $('#m-annuler').disabled = !enCours;
    for (const b of $$('#m-rapides button')) b.disabled = d.liaison !== 'ok';
    // Agent de veille : ce qu'il comprend de la situation, rafraîchi toutes les quelques secondes
    const s = d.liaison === 'ok' ? d.etat?.situation : null;
    const ligne = $('#m-situation');
    ligne.hidden = !s?.resume;
    if (s?.resume) {
      texte(ligne, s.alerte && s.niveau !== 'rien' ? `${s.resume} — ${s.alerte}` : s.resume);
      attribut(ligne, 'data-niveau', s.niveau || 'info');
    }
  }

  // ---------------------------------------------------------------- vision

  async _demanderVision(demande) {
    if (this.ctx.donnees.liaison !== 'ok') { signaler('Liaison coupée', 'erreur'); return; }
    const msg = demande.demande === 'chercher' ? `Recherche : ${demande.objet}`
      : demande.demande === 'analyser' ? 'Scan 3D (IA + Lidar + Stéréo)…' : 'Analyse…';
    signaler(msg, 'info', 1500);
    const r = await this.ctx.connexion.demander({ type: 'vision', camera: this.cameraVision, ...demande }, 30000);
    if (!r.ok && !r.delai) signalerRefus(r);
  }

  _rendreVision() {
    const v = this.ctx.donnees.vision;
    if (!v) return;
    this._figer(true);
    const image = $('#m-vision-img');
    if (v.image) image.src = `data:image/jpeg;base64,${v.image}`;
    $('#m-vision-vide').hidden = Boolean(v.image);
    texte($('#m-vision-texte'), v.description || '');
    const morceaux = [v.camera, v.modele, Number.isFinite(v.duree_s) ? `${nombre(v.duree_s, 1)} s` : null, heure(v.t)].filter(Boolean);
    texte($('#m-vision-meta'), morceaux.join(' · '));
    const points = $('#m-vision-points');
    points.replaceChildren();
    for (const p of v.points || []) {
      if (!Array.isArray(p.sol)) {
        if (Number.isFinite(p.distance_m)) {  // profondeur seule : pas de but au sol à viser
          points.append(el('span', { class: 'bouton petit', text: `${p.label || 'point'} · ≈ ${nombre(p.distance_m, 1)} m` }));
        }
        continue;
      }
      const details = [`${nombre(Math.hypot(p.sol[0], p.sol[1]), 1)} m`];
      if (Number.isFinite(p.hauteur)) details.push(`h ${nombre(p.hauteur, 2)} m`);
      if (p.source) details.push(p.source);
      const bouton = el('button', { type: 'button', class: 'bouton petit' }, icone('cible'),
        el('span', { text: `${p.label || 'point'} · ${details.join(' · ')}` }));
      bouton.addEventListener('click', () => this._allerVers(p.sol, v.poseRecue));
      points.append(bouton);
    }
    if (v.demande === 'chercher' && v.objet) $('#m-objet').value = v.objet;
    this._dessinerVision();
  }

  _figer(fige) {
    this.fige = fige;
    attribut($('#m-vision-cadre'), 'data-fige', fige ? 'true' : 'false');
    if (fige || !this.actif) this.direct.desactiver();
    else this.direct.activer();
    if (fige) this._dessinerVision();
  }

  async _allerVers(sol, pose) {
    // Le point est dans base_link au moment de l'analyse : on le ramène dans odom avec la pose
    // reçue à ce moment-là, pour que le but reste juste si le rover a bougé depuis.
    let action;
    if (pose && Number.isFinite(pose.x)) {
      const [x, y] = versParent({ x: pose.x, y: pose.y, cap: pose.cap }, sol[0], sol[1]);
      action = { type: 'aller_a', x: Math.round(x * 100) / 100, y: Math.round(y * 100) / 100, repere: 'odom' };
    } else {
      action = { type: 'aller_a', x: sol[0], y: sol[1], repere: 'robot' };
    }
    const r = signalerRefus(await this.ctx.connexion.demander({ type: 'action', action }));
    if (r.ok) signaler(decrireAction(action), 'info', 1800);
  }

  _dessinerVision() {
    const v = this.ctx.donnees.vision;
    const canvas = $('#m-vision-canvas');
    const c = preparerCanevas(canvas);
    if (!c || !v) return;
    const image = $('#m-vision-img');
    const { ctx, l, h } = c;
    const cadre = cadreContenu(l, h, image.naturalWidth, image.naturalHeight);
    const X = (u) => cadre.x + u * cadre.l, Y = (w) => cadre.y + w * cadre.h;
    ctx.font = '600 12px system-ui, sans-serif';
    ctx.textBaseline = 'bottom';
    const etiquette = (x, y, libelle, teinte) => {
      if (!libelle) return;
      const largeur = ctx.measureText(libelle).width + 10;
      const bx = Math.min(Math.max(0, x), l - largeur), by = Math.max(18, y);
      ctx.fillStyle = teinte;
      ctx.fillRect(bx, by - 18, largeur, 18);
      ctx.fillStyle = '#fff';
      ctx.fillText(libelle, bx + 5, by - 3);
    };
    for (const b of v.boites || []) {
      ctx.strokeStyle = 'rgba(0,0,0,0.5)';
      ctx.lineWidth = 4;
      ctx.strokeRect(X(b.u0), Y(b.v0), X(b.u1) - X(b.u0), Y(b.v1) - Y(b.v0));
      ctx.strokeStyle = couleur('--serie-4');
      ctx.lineWidth = 2;
      ctx.strokeRect(X(b.u0), Y(b.v0), X(b.u1) - X(b.u0), Y(b.v1) - Y(b.v0));
      etiquette(X(b.u0), Y(b.v0), b.label, couleur('--serie-4'));
    }
    for (const p of v.points || []) {
      if (Array.isArray(p.contour) && p.contour.length > 2) {  // contour de l'objet (SAM 2.1)
        ctx.beginPath();
        p.contour.forEach(([cu, cv], i) => { if (i) ctx.lineTo(X(cu), Y(cv)); else ctx.moveTo(X(cu), Y(cv)); });
        ctx.closePath();
        ctx.fillStyle = couleur('--accent-voile');
        ctx.fill();
        ctx.strokeStyle = couleur('--accent');
        ctx.lineWidth = 2;
        ctx.stroke();
      }
      const x = X(p.u), y = Y(p.v);
      ctx.fillStyle = 'rgba(0,0,0,0.5)';
      ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = couleur('--accent');
      ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.stroke();
      if (!(v.boites || []).some((b) => b.label === p.label)) etiquette(x + 10, y - 8, p.label, couleur('--accent-fort'));
    }
  }
}
