// Menu Pilotage : caméra avec arc prévu, mini-radar, mesures et raisons de sécurité, mode,
// armement, joystick, clavier, vitesse maximale et mode vocal.

import { $, $$, el, icone, texte, attribut, nombre, confirmer, signaler, signalerRefus, lireLocal, ecrireLocal } from '../outils.js';
import { VueCamera } from '../camera.js';
import { Radar } from '../radar.js';
import { raison } from '../textes.js';
import { directionTouche } from '../teleop.js';
import { DEG } from '../geometrie.js';
import { AtelierDroneVue } from '../drone.js';
import { CarteGPS, tuilesHorsLigne } from '../cartegps.js';

function saisieEnCours(cible) {
  return cible instanceof Element && (cible.closest('input, textarea, select, [contenteditable="true"]') !== null);
}

export class VuePilotage {
  constructor(ctx) {
    this.ctx = ctx;
    this.atelierDrone = new AtelierDroneVue(ctx);
    this.actif = false;
    const { donnees } = ctx;

    const camInitiale = lireLocal('camera', 'gauche');
    this.camera = new VueCamera($('#p-camera'), ctx, {
      camera: camInitiale, superposition: true, ips: 10, largeur: 640,
      surClic: (cible) => this.clicImage(cible),
    });
    this.doubleCam = lireLocal('double-cam', true);
    this.cameraPip = new VueCamera($('#p-camera-pip'), ctx, {
      camera: camInitiale === 'drone' ? 'gauche' : 'drone',
      superposition: false, ips: 6, largeur: 320,
    });
    // Vue principale : une caméra ou la 3D ; l'autre s'affiche en vignette (clic = échange).
    this.principale = 'camera';
    this.vue3d = null;
    this._choixCamera();
    this._synchroniserPip();
    $('#p-choix-camera').addEventListener('click', (e) => {
      const b = e.target.closest('button[data-camera]');
      if (!b) return;
      if (b.dataset.camera === '3d') {
        this._definirPrincipale('3d');
        return;
      }
      this._selectionnerCamera(b.dataset.camera);
    });
    $('#p-camera').addEventListener('click', () => { if (this.principale === '3d') this._definirPrincipale('camera'); });
    $('#p-3d').addEventListener('click', () => { if (this.principale === 'camera') this._definirPrincipale('3d'); });
    $('#p-camera-pip')?.addEventListener('click', (e) => {
      e.stopPropagation();
      this._echangerRoverDrone();
    });
    $('#p-double-cam')?.addEventListener('click', () => {
      this.doubleCam = !this.doubleCam;
      ecrireLocal('double-cam', this.doubleCam);
      this._synchroniserPip();
    });
    $('#p-camera-3d').addEventListener('click', (e) => {
      const b = e.target.closest('button[data-mode3d]');
      if (!b) return;
      this._modeCamera3D(b.dataset.mode3d);
    });
    this._chargerVue3D();

    // Mini-carte GPS (option des Réglages, désactivée par défaut) : un clic ouvre la carte GPS.
    this.minicarte = new CarteGPS($('#p-minicarte canvas'), ctx, { mini: true });
    $('#p-minicarte').addEventListener('click', () => {
      ecrireLocal('fond-carte', 'gps');
      location.hash = '#carte';
    });
    ctx.surOption?.((nom) => { if (nom === 'minicarte' || nom === 'tuiles') this._majMinicarte(); });
    this._majMinicarte();
    $('#p-photo').addEventListener('click', async () => {
      const camera = this.camera.camera === 'profondeur' ? 'gauche' : this.camera.camera;
      const r = signalerRefus(await ctx.connexion.demander({ type: 'photo', camera }));
      if (r.ok) signaler(typeof r.valeur === 'string' ? `Photo ${r.valeur}` : 'Photo enregistrée', 'succes');
    });

    this.radar = new Radar($('#p-radar'), ctx, { portee: lireLocal('portee', 4), mini: true });
    const changerPortee = () => {
      const p = this.radar.changerPortee();
      texte($('#p-portee'), `${p} m`);
      ecrireLocal('portee', p);
    };
    texte($('#p-portee'), `${this.radar.portee} m`);
    $('#p-portee').addEventListener('click', changerPortee);
    $('#p-radar').addEventListener('click', changerPortee);

    // Modes.
    $('#p-modes').addEventListener('click', async (e) => {
      const b = e.target.closest('button[data-mode]');
      if (!b) return;
      const mode = b.dataset.mode;
      if (donnees.etat?.mode === mode) return;
      let message = { type: 'mode', mode };
      if (mode === 'banc') {
        if (!(await confirmer('Passer en banc ? Roues levées.', b, { danger: true }))) return;
        message = { ...message, confirmer: true };
      }
      signalerRefus(await ctx.connexion.demander(message));
    });
    $('#p-armer').addEventListener('click', async () => {
      const arme = donnees.etat?.arme;
      signalerRefus(await ctx.connexion.demander({ type: arme ? 'desarmer' : 'armer' }));
    });

    // Vitesse maximale (fraction du maximum du mode manuel, gardée par le poste).
    const vmax = $('#p-vmax');
    vmax.value = lireLocal('vmax', 50);
    const appliquerVmax = () => {
      ctx.teleop.definirFacteur(Number(vmax.value) / 100);
      ecrireLocal('vmax', Number(vmax.value));
      this._majVmax();
    };
    vmax.addEventListener('input', appliquerVmax);
    appliquerVmax();
    for (const type of ['bienvenue', 'reponse']) donnees.sur(type, () => this._majVmax());

    this._installerJoystick();
    this._installerClavier();
    $('#p-remorque-axe').addEventListener('click', async () => {
      const r = signalerRefus(await ctx.connexion.demander({ type: 'remorque_alignee' }));
      if (r.ok) signaler('Remorque dans l’axe', 'info', 1500);
    });

    $('#p-drone-actions')?.addEventListener('click', async (e) => {
      const b = e.target.closest('button');
      if (!b) return;
      const action = b.dataset.droneAction;
      const cmd = b.dataset.droneCmd;
      if (action === 'voir') {
        this._selectionnerCamera('drone');
        return;
      }
      if (action === 'echanger') {
        this._echangerRoverDrone();
        return;
      }
      if (action === 'scan') {
        const r = signalerRefus(await ctx.connexion.demander({ type: 'drone', action: 'scan' }));
        if (r.ok) signaler('Scan BLE + Wi-Fi drone lancé', 'info');
        return;
      }
      if (action === 'connecter-ssid') {
        const ssid = $('#p-drone-ssid')?.value?.trim() || 'auto';
        const r = signalerRefus(await ctx.connexion.demander({ type: 'drone', action: 'connecter', ssid }));
        if (r.ok) signaler(`Connexion drone demandée sur « ${ssid} »`, 'info');
        return;
      }
      if (action === 'auto') {
        const champ = $('#p-drone-ssid');
        if (champ) champ.value = '';
        const r = signalerRefus(await ctx.connexion.demander({ type: 'drone', action: 'connecter', ssid: 'auto' }));
        if (r.ok) signaler('Recherche et connexion automatique au drone activées', 'info');
        return;
      }
      if (cmd) {
        const r = signalerRefus(await ctx.connexion.demander({ type: 'drone', action: 'cmd', cmd }));
        const messages = {
          camera: 'Bascule caméra demandée ; vérifier le changement dans la vidéo',
          recon_test: 'Essai sans vol : tourner à la main puis capturer six vues',
          recon_capture: 'Capture demandée sur la prochaine image',
          cibler_croix: 'Visée de la croix modifiée — aucun déplacement commandé',
          stop_recon: 'Essai arrêté',
          atterrir: 'Requête d’atterrissage envoyée — exécution non confirmée',
          stop: 'Requête d’arrêt envoyée — exécution non confirmée',
        };
        if (r.ok) signaler(messages[cmd] || `Commande drone : ${cmd}`, cmd === 'stop' ? 'alerte' : 'info');
      }
    });

    $('#p-voix').addEventListener('click', () => ctx.basculerVoix());
    donnees.sur('etat', () => this._majEtat());
    donnees.sur('voix', () => this._majVoix());
    donnees.sur('liaison', () => this._majEtat());
  }

  activer() {
    this.actif = true;
    this.camera.activer();
    this._synchroniserPip();
    this._activer3D();
  }

  desactiver() {
    this.actif = false;
    this.camera.desactiver();
    this.cameraPip?.desactiver();
    this._activer3D();
    this.ctx.teleop.arreter();
  }

  _selectionnerCamera(cam) {
    this.camera.definirCamera(cam);
    ecrireLocal('camera', cam);
    this._definirPrincipale('camera');
    this._synchroniserPip();
  }

  _echangerRoverDrone() {
    const nouvelle = this.camera.camera === 'drone' ? 'gauche' : 'drone';
    this._selectionnerCamera(nouvelle);
  }

  _synchroniserPip() {
    const pipEl = $('#p-camera-pip');
    const btn = $('#p-double-cam');
    if (btn) attribut(btn, 'aria-pressed', this.doubleCam ? 'true' : 'false');
    if (!pipEl || !this.cameraPip) return;
    if (!this.doubleCam) {
      pipEl.hidden = true;
      this.cameraPip.desactiver();
      return;
    }
    pipEl.hidden = false;
    const ciblePip = this.camera.camera === 'drone' ? 'gauche' : 'drone';
    this.cameraPip.definirCamera(ciblePip);
    texte($('#p-pip-etiquette'), ciblePip === 'drone' ? 'Caméra Drone (clic = agrandir)' : 'Caméra Rover (clic = agrandir)');
    if (this.actif) this.cameraPip.activer();
  }

  // ---------------------------------------------------------------- vue 3D (module facultatif)

  async _chargerVue3D() {
    let module;
    try {
      module = await import('../vue3d/vue3d.js');
    } catch (erreur) {
      console.info('[poste] vue 3D absente :', erreur?.message || erreur);
      return;
    }
    const d = this.ctx.donnees;
    if (!d.bienvenue) await new Promise((fin) => { const arret = d.sur('bienvenue', () => { arret(); fin(); }); });
    const conteneur = $('#p-3d');
    conteneur.hidden = false;
    try {
      this.vue3d = new module.Vue3D(conteneur, { profil: d.profil });
    } catch (erreur) {
      console.error('[poste] vue 3D :', erreur);
      conteneur.hidden = true;
      conteneur.replaceChildren();
      return;
    }
    const transmettre = (type) => (m) => {
      try {
        this.vue3d?.[type]?.(m);
      } catch (erreur) {
        console.error(`[poste] vue 3D (${type}) :`, erreur);
      }
    };
    for (const type of ['scene', 'etat', 'scan', 'obstacles']) d.sur(type, transmettre(type));
    if (d.scene) transmettre('scene')(d.scene);
    $('#p-choix-camera button[data-camera="3d"]').hidden = false;
    this._modeCamera3D(lireLocal('camera3d', 'suivie'));
    this._definirPrincipale(lireLocal('principale', 'camera'));
    document.addEventListener('visibilitychange', () => this._activer3D());
    this._activer3D();
  }

  _activer3D() {
    try {
      this.vue3d?.activer(this.actif && !document.hidden);
    } catch (erreur) {
      console.error('[poste] vue 3D :', erreur);
    }
  }

  _modeCamera3D(mode) {
    for (const b of $$('#p-camera-3d button')) attribut(b, 'aria-pressed', b.dataset.mode3d === mode ? 'true' : 'false');
    ecrireLocal('camera3d', mode);
    try {
      this.vue3d?.camera(mode);
    } catch (erreur) {
      console.error('[poste] vue 3D :', erreur);
    }
  }

  _definirPrincipale(principale) {
    if (principale === '3d' && !this.vue3d) principale = 'camera';
    this.principale = principale;
    attribut($('#p-principale'), 'data-principale', principale);
    $('#p-camera-3d').hidden = principale !== '3d';
    this.camera.marquer(null);
    ecrireLocal('principale', principale);
    this._choixCamera();
    // La taille du canevas 3D change : on le prévient comme pour un redimensionnement.
    window.dispatchEvent(new Event('resize'));
  }

  _majMinicarte() {
    $('#p-minicarte').hidden = !lireLocal('minicarte', false);
  }

  dessiner() {
    this.camera.dessiner();
    this.radar.dessiner();
    if (!$('#p-minicarte').hidden) {
      this.minicarte.dessiner();
      $('#p-minicarte-attribution').hidden = !lireLocal('tuiles', 'osm') || tuilesHorsLigne();
    }
    const p = this.ctx.prevision();
    $('#p-badge-arriere').hidden = !(p.vitesse < -0.02);
  }

  _choixCamera() {
    const choisi = this.principale === '3d' ? '3d' : this.camera.camera;
    for (const b of $$('#p-choix-camera button')) attribut(b, 'aria-pressed', b.dataset.camera === choisi ? 'true' : 'false');
  }

  _majVmax() {
    const d = this.ctx.donnees;
    const max = d.reglages['vitesses.max_manuel_m_s'] ?? d.profil.vitesses.max_manuel_m_s;
    texte($('#p-vmax-valeur'), `${nombre((max * Number($('#p-vmax').value)) / 100, 2)} m/s`);
  }

  // ---------------------------------------------------------------- image : aller au pixel

  async clicImage({ camera, u, v, x, y }) {
    if (this.principale !== 'camera' || this.ctx.donnees.etat?.mode !== 'auto') return;
    this.camera.marquer(u, v);
    const ok = await confirmer('Aller ici ?', { x, y });
    if (!ok) { this.camera.marquer(null); return; }
    const r = signalerRefus(await this.ctx.connexion.demander({
      type: 'aller_pixel', camera, u: Math.round(u * 1e4) / 1e4, v: Math.round(v * 1e4) / 1e4,
    }));
    if (!r.ok) this.camera.marquer(null);
  }

  // ---------------------------------------------------------------- joystick

  _installerJoystick() {
    const zone = $('#p-joystick'), manette = zone.querySelector('.manette');
    const teleop = this.ctx.teleop;
    let pointeur = null;
    const placer = (dx, dy) => { manette.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`; };
    const deplacer = (e) => {
      const r = zone.getBoundingClientRect();
      const rayon = r.width / 2 - manette.offsetWidth / 2;
      let dx = e.clientX - (r.left + r.width / 2), dy = e.clientY - (r.top + r.height / 2);
      const n = Math.hypot(dx, dy);
      if (n > rayon) { dx *= rayon / n; dy *= rayon / n; }
      placer(dx, dy);
      teleop.joystick(dx / rayon, -dy / rayon);
    };
    const relacher = () => {
      if (pointeur === null) return;
      pointeur = null;
      placer(0, 0);
      attribut(zone, 'data-actif', 'false');
      teleop.relacherJoystick();
    };
    zone.addEventListener('pointerdown', (e) => {
      if (pointeur !== null) return;
      if (!this._manuelPossible()) return;
      pointeur = e.pointerId;
      zone.setPointerCapture(e.pointerId);
      attribut(zone, 'data-actif', 'true');
      deplacer(e);
      e.preventDefault();
    });
    zone.addEventListener('pointermove', (e) => { if (e.pointerId === pointeur) deplacer(e); });
    zone.addEventListener('pointerup', (e) => { if (e.pointerId === pointeur) relacher(); });
    zone.addEventListener('pointercancel', relacher);
    zone.addEventListener('lostpointercapture', relacher);
    this.ctx.surArret(relacher);
  }

  _manuelPossible() {
    const e = this.ctx.donnees.etat;
    if (this.ctx.donnees.liaison !== 'ok') { signaler('Liaison coupée', 'erreur'); return false; }
    if (e?.mode !== 'manuel') { signaler('Passer en manuel', 'alerte', 2000); return false; }
    return true;
  }

  // ---------------------------------------------------------------- clavier

  _installerClavier() {
    const teleop = this.ctx.teleop;
    const touches = new Map(); // code physique → direction (relâchement fiable)
    let prevenu = 0;
    document.addEventListener('keydown', (e) => {
      if (!this.actif || e.ctrlKey || e.metaKey || e.altKey || saisieEnCours(e.target)) return;
      if (e.code === 'Space' || e.key === ' ') {
        e.preventDefault();
        if (!e.repeat) this.ctx.arretUrgence();
        return;
      }
      const direction = directionTouche(e.key);
      if (!direction) return;
      e.preventDefault();
      if (e.repeat || touches.has(e.code)) return;
      if (this.ctx.donnees.etat?.mode !== 'manuel') {
        if (performance.now() - prevenu > 3000) { signaler('Passer en manuel', 'alerte', 2000); prevenu = performance.now(); }
        return;
      }
      touches.set(e.code, direction);
      teleop.appuyer(direction);
    });
    document.addEventListener('keyup', (e) => {
      const direction = touches.get(e.code);
      if (!direction) return;
      touches.delete(e.code);
      if (![...touches.values()].includes(direction)) teleop.relacher(direction);
    });
    this.ctx.surArret(() => touches.clear());
  }

  // ---------------------------------------------------------------- état

  _majEtat() {
    const d = this.ctx.donnees, e = d.liaison === 'ok' ? d.etat : null;
    const mode = e?.mode;
    for (const b of $$('#p-modes button')) attribut(b, 'aria-pressed', b.dataset.mode === mode ? 'true' : 'false');
    const armer = $('#p-armer');
    attribut(armer, 'aria-pressed', e?.arme ? 'true' : 'false');
    texte(armer.querySelector('span'), e?.arme ? 'Désarmer' : 'Armer');
    armer.disabled = !e || Boolean(e.arret_urgence) || (!e.arme && e.mode === 'arret');
    attribut(armer, 'title', e && !e.arme && e.mode === 'arret' ? 'Choisir d’abord un mode' : null);
    attribut($('#p-joystick'), 'data-desactive', mode === 'manuel' ? 'false' : 'true');
    attribut($('#p-camera'), 'data-cliquable', mode === 'auto' && this.principale === 'camera' ? 'true' : 'false');
    $('#p-badge-auto').hidden = mode !== 'auto';
    $('#p-badge-auto').title = 'Clic sur l’image : aller au point';

    const odom = e?.odom, sec = e?.securite;
    texte($('#p-vitesse'), nombre(odom?.vitesse ?? e?.commande?.vitesse, 2));
    const braquage = e?.commande?.braquage ?? e?.consigne?.braquage;
    texte($('#p-braquage'), Number.isFinite(braquage) ? nombre(braquage / DEG, 0) : '—');
    const recul = (e?.consigne?.vitesse ?? 0) < -0.01;
    const libre = recul ? sec?.libre_arriere_m : sec?.libre_avant_m;
    texte($('#p-libre'), nombre(libre, 1));
    const mesureLibre = $('#p-libre').closest('.mesure');
    attribut(mesureLibre, 'data-niveau', !Number.isFinite(libre) ? null : libre < 0.5 ? 'critique' : libre < 1 ? 'alerte' : null);
    const limite = recul ? sec?.vitesse_max_arriere : sec?.vitesse_max_avant;
    texte($('#p-limite'), sec?.arret ? '0,00' : nombre(limite, 2));
    attribut($('#p-limite').closest('.mesure'), 'data-niveau', sec?.arret ? 'critique' : null);

    // Remorque : angle d'attelage estimé (incertitude) ; « ± » grand = marche arrière refusée.
    const rem = e?.remorque;
    $('#p-remorque').hidden = !rem;
    if (rem) {
      const angle = $('#p-remorque-angle');
      texte(angle, `${nombre(rem.angle_deg, 0)}° ± ${nombre(rem.incertitude_deg, 0)}°`);
      attribut(angle, 'data-niveau', Math.abs(rem.angle_deg) > 35 ? 'critique' : rem.recul_possible ? null : 'alerte');
      attribut(angle, 'title', rem.recul_possible ? 'Marche arrière permise (lente)'
        : 'Angle incertain : avancer 1 m tout droit, ou « Dans l’axe »');
    }

    // Raisons de sécurité en puces.
    const liste = $('#p-raisons');
    const codes = sec?.raisons || [];
    const cle = e ? codes.join(',') : 'hors-ligne';
    if (liste.dataset.cle !== cle) {
      liste.dataset.cle = cle;
      liste.replaceChildren();
      if (!e) {
        liste.append(el('li', { dataset: { niveau: 'neutre' } }, icone('liaison'), 'Pas d’état'));
      } else if (!codes.length) {
        liste.append(el('li', { dataset: { niveau: 'ok' } }, icone('ok'), 'Voie libre'));
      } else {
        for (const code of codes) {
          const [libelle, niveau] = raison(code);
          liste.append(el('li', { dataset: { niveau }, title: code }, icone(niveau === 'neutre' ? 'info' : 'alerte'), libelle));
        }
      }
    }
    this._majDrone(e?.drone);
    this._majVoix();
  }

  _majDrone(dr) {
    const pastille = $('#p-drone-etat');
    if (!pastille) return;
    const niveau = !dr ? 'neutre' : dr.connecte ? 'ok' : dr.etat === 'connexion' ? 'info' : dr.etat === 'scan' ? 'alerte' : 'neutre';
    const libelleEtat = !dr ? 'Drone : attente' : dr.connecte ? 'Drone : Connecté' : `Drone : ${dr.etat}`;
    attribut(pastille, 'data-niveau', niveau);
    texte(pastille.querySelector('.libelle'), libelleEtat);

    texte($('#p-drone-reseau'), dr?.ssid ? `Liaison : ${dr.ssid}${dr.canal ? ` (ch ${dr.canal})` : ''}` : 'Liaison : auto (BLE / Wi-Fi)');
    texte($('#p-drone-signal'), Number.isFinite(dr?.rssi_dbm) ? `Signal : ${dr.rssi_dbm} dBm` : 'Signal : —');
    texte($('#p-drone-ip'), dr?.passerelle || dr?.bssid ? `Drone : ${dr.passerelle || dr.bssid}` : 'Drone : —');
    const proto = dr?.protocole && dr.protocole !== 'attente' ? dr.protocole : (dr?.port_video ? `UDP:${dr.port_video}` : 'attente');
    texte($('#p-drone-flux'), dr ? `Flux : ${nombre(dr.ips, 1)} i/s (${dr.images_rx ?? 0} img) · ${proto} · RX ${dr.paquets_rx ?? 0} (${dr.ko_rx ?? 0} ko)` : 'Flux : —');
    texte($('#p-drone-batterie'), Number.isFinite(dr?.batterie_pct) ? `Batterie : ${dr.batterie_pct} %` : 'Batterie : 1S (LED)');
    const vus = dr?.reseaux_scan?.length ? dr.reseaux_scan.slice(0, 3).join(', ') : 'aucun';
    texte($('#p-drone-scan'), `Détectés : ${vus}`);
    texte($('#p-drone-recon'), dr?.consigne_vol || 'Essai sans vol : attente');
    texte($('#p-drone-croix'), dr?.croix_trouvee
      ? `Croix : ${dr.croix_verrouillee ? 'centrée' : 'détectée'} · X ${nombre(dr.croix_x, 2)} / Y ${nombre(dr.croix_y, 2)}`
      : 'Croix : non détectée');
    texte($('#p-drone-compute'), `Vision : ${nombre(dr?.analyse_ms || 0, 1)} ms/image · maximum 5 i/s`);
    for (const [id, fichier] of [['p-drone-montage', 'montage.jpg'], ['p-drone-rapport', 'rapport.json']]) {
      const lien = $(`#${id}`);
      if (lien) {
        lien.hidden = !dr?.montage_recon;
        const url = new URL(`/api/drone/recon/${fichier}`, location.href);
        if (this.ctx.connexion.jeton) url.searchParams.set('jeton', this.ctx.connexion.jeton);
        lien.href = url.href;
      }
    }
    const capture = document.querySelector('[data-drone-cmd="recon_capture"]');
    if (capture) {
      capture.disabled = this.ctx.donnees.liaison !== 'ok' || dr?.mode_recon !== 'rotation_360';
      texte(capture.querySelector('span'), `Capturer vue (${Math.min(6, (dr?.secteur_360 || 0) + 1)}/6)`);
    }
    attribut(document.querySelector('[data-drone-cmd="cibler_croix"]'), 'aria-pressed', dr?.mode_recon === 'ciblage_croix' ? 'true' : 'false');
  }

  _majVoix() {
    const d = this.ctx.donnees, voix = d.liaison === 'ok' ? d.etat?.voix : null;
    const bouton = $('#p-voix');
    attribut(bouton, 'aria-pressed', voix?.active ? 'true' : 'false');
    attribut(bouton, 'data-parole', voix?.active && (voix?.parle || d.parole) ? 'true' : 'false');
    const niveau = voix?.active ? Math.min(1, Math.max(0, voix.niveau || 0)) : 0;
    $('#p-niveau').style.width = `${Math.round(Math.sqrt(niveau) * 100)}%`;
    const t = d.transcription;
    const ligne = $('#p-transcription');
    texte(ligne, t ? (t.evenement === 'erreur' ? t.texte || 'erreur' : `« ${t.texte} »`) : '');
    attribut(ligne, 'data-evenement', t?.evenement || null);
  }
}
