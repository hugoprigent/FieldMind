// Barre d'état toujours visible : liaison, simulation, mode, armement, capteurs, IA, Wi-Fi,
// gros bouton ARRÊT (arrêt d'urgence) et réarmement explicite.

import { $, el, icone, texte, attribut, nombre, confirmer, signaler, signalerRefus } from '../outils.js';
import { MODES, FIX_GNSS } from '../textes.js';

const PASTILLES = [
  { cle: 'liaison', icone: 'liaison', libelle: 'Liaison', vue: 'systeme' },
  { cle: 'mode', icone: 'mode', libelle: '—', vue: 'pilotage' },
  { cle: 'arme', icone: 'arme', libelle: 'Désarmé', vue: 'pilotage' },
  { cle: 'esp', icone: 'esp', libelle: 'ESP', vue: 'capteurs' },
  { cle: 'drone', icone: 'drone', libelle: 'Drone', vue: 'pilotage' },
  { cle: 'lidar', icone: 'lidar', libelle: 'Lidar', vue: 'capteurs' },
  { cle: 'cameras', icone: 'camera', libelle: 'Caméras', vue: 'capteurs' },
  { cle: 'stereo', icone: 'stereo', libelle: 'Stéréo', vue: 'capteurs' },
  { cle: 'gps', icone: 'gps', libelle: 'GPS', vue: 'capteurs' },
  { cle: 'ia', icone: 'ia', libelle: 'IA', vue: 'reglages' },
  { cle: 'wifi', icone: 'wifi', libelle: 'Wi-Fi', vue: 'systeme' },
];

export class Barre {
  constructor(ctx) {
    this.ctx = ctx;
    const conteneur = $('#indicateurs');
    this.pastilles = {};
    for (const p of PASTILLES) {
      const lien = el('a', { class: 'pastille', href: `#${p.vue}`, dataset: { cle: p.cle, niveau: 'neutre' } },
        icone(p.icone), el('span', { class: 'libelle', text: p.libelle }));
      conteneur.append(lien);
      this.pastilles[p.cle] = { lien, libelle: lien.querySelector('.libelle') };
    }
    // Micro : appuyer, puis parler ; chaque phrase entendue part à l'agent (voix du rover)
    this.boutonVoix = $('#bouton-voix');
    this.boutonVoix.addEventListener('click', () => ctx.basculerVoix());
    ctx.donnees.sur('voix', (m) => {
      if (m.evenement === 'transcription' && m.texte) signaler(`« ${m.texte} »`, 'info', 3500);
      if (m.evenement === 'erreur') signaler(m.message || 'micro : erreur', 'erreur', 3500);
      this.majVoix();
    });
    this.boutonArret = $('#bouton-arret');
    this.boutonRearmer = $('#bouton-rearmer');
    this.boutonArret.addEventListener('click', () => ctx.arretUrgence());
    this.boutonRearmer.addEventListener('click', async () => {
      if (!(await confirmer('Réarmer ?', this.boutonRearmer))) return;
      const r = signalerRefus(await ctx.connexion.demander({ type: 'rearmer' }));
      if (r.ok) signaler('Réarmé', 'succes');
    });
    const maj = () => this.maj();
    for (const type of ['etat', 'systeme', 'bienvenue', 'liaison', 'ia', 'reponse']) ctx.donnees.sur(type, maj);
    setInterval(maj, 500);
    this.maj();
  }

  definir(cle, niveau, libelle, titre = '') {
    const p = this.pastilles[cle];
    attribut(p.lien, 'data-niveau', niveau);
    if (libelle !== undefined) texte(p.libelle, libelle);
    attribut(p.lien, 'title', titre || p.libelle.textContent);
  }

  majVoix() {
    const d = this.ctx.donnees, voix = d.liaison === 'ok' ? d.etat?.voix : null;
    attribut(this.boutonVoix, 'aria-pressed', voix?.active ? 'true' : 'false');
    attribut(this.boutonVoix, 'data-parole', voix?.active && (voix?.parle || d.parole) ? 'true' : 'false');
    texte(this.boutonVoix.querySelector('.libelle'), voix?.active ? 'J’écoute' : 'Parler');
    const niveau = voix?.active ? Math.min(1, Math.max(0, voix.niveau || 0)) : 0;
    this.boutonVoix.style.setProperty('--niveau', String(Math.round(Math.sqrt(niveau) * 100) / 100));
  }

  maj() {
    this.majVoix();
    const d = this.ctx.donnees;
    const bandeauSimulation = $('#bandeau-simulation');
    bandeauSimulation.hidden = d.materiel !== 'simulation';
    if (location.pathname.startsWith('/fieldmind/')) {
      const nomIA = d.ia?.profil === 'gx10' ? 'GX10'
        : d.ia?.profil === 'gemini' ? 'Gemini'
          : 'désactivée';
      bandeauSimulation.textContent = `SIMULATION · IA ${nomIA}`;
    } else {
      bandeauSimulation.textContent = 'SIMULATION';
    }
    const enLigne = d.liaison === 'ok';
    const age = d.ageEtat();
    const frais = enLigne && age < 1.5;
    if (!enLigne) this.definir('liaison', d.liaison === 'perdue' ? 'critique' : 'alerte', 'Liaison', d.liaison === 'perdue' ? 'Liaison perdue' : 'Connexion…');
    else if (!frais) this.definir('liaison', 'alerte', 'Liaison', Number.isFinite(age) ? `Muet depuis ${nombre(age, 1)} s` : 'En attente');
    else this.definir('liaison', 'ok', 'Liaison', 'Liaison établie');

    const e = frais ? d.etat : null;
    const s = enLigne && performance.now() - d.recuSysteme < 4000 ? d.systeme : null;

    // Mode et armement.
    if (e) {
      const niveaux = { arret: 'neutre', manuel: 'info', auto: 'info', banc: 'alerte' };
      this.definir('mode', niveaux[e.mode] || 'neutre', MODES[e.mode] || e.mode || '—', `Mode ${MODES[e.mode] || e.mode}`);
      if (e.arret_urgence) this.definir('arme', 'critique', 'Verrouillé', 'Arrêt d’urgence : réarmement nécessaire');
      else if (e.arme) this.definir('arme', 'arme', 'Armé', 'Traction autorisée');
      else this.definir('arme', 'neutre', 'Désarmé', 'Traction interdite');
    } else {
      this.definir('mode', 'neutre', '—');
      this.definir('arme', 'neutre', '—');
    }
    attribut(this.boutonArret, 'data-verrouille', e?.arret_urgence ? 'true' : 'false');
    this.boutonRearmer.hidden = !e?.arret_urgence;

    const c = e?.capteurs || {};
    // ESP.
    const esp = e?.esp;
    if (!e || esp === undefined || esp === null) this.definir('esp', 'neutre', 'ESP', 'ESP : inconnu');
    else if (!esp.connecte) this.definir('esp', 'critique', 'ESP', 'ESP absent');
    else if (esp.defaut) this.definir('esp', 'critique', 'ESP', `Défaut ESP : ${esp.raison_defaut || '?'}`);
    else this.definir('esp', 'ok', 'ESP', `ESP ${esp.firmware || ''}`.trim());

    // Drone.
    const dr = e?.drone;
    if (!e || !dr) this.definir('drone', 'neutre', 'Drone', 'Drone : en attente de l’ESP32-C5');
    else if (dr.connecte) {
      const bat = Number.isFinite(dr.batterie_pct) ? ` · ${dr.batterie_pct}%` : '';
      const rssi = Number.isFinite(dr.rssi_dbm) ? ` (${dr.rssi_dbm} dBm)` : '';
      this.definir('drone', 'ok', dr.ssid ? `Drone · ${dr.ssid.slice(0, 10)}${bat}` : 'Drone OK',
        `Drone connecté : ${dr.ssid || '?'}${rssi} · IP ${dr.ip_locale || '?'} → ${dr.passerelle || '?'} · ${nombre(dr.ips, 1)} i/s`);
    } else if (dr.etat === 'connexion') {
      this.definir('drone', 'info', 'Drone…', `Connexion au drone ${dr.ssid || ''}…`);
    } else if (dr.etat === 'scan') {
      this.definir('drone', 'alerte', 'Drone · Scan', `Recherche Wi-Fi drone (${dr.reseaux_scan?.length || 0} réseaux vus)`);
    } else {
      this.definir('drone', 'neutre', 'Drone', `Drone : ${dr.etat}`);
    }

    // Lidar.
    const lidar = c.lidar;
    if (!lidar) this.definir('lidar', 'neutre', 'Lidar', 'Lidar : inconnu');
    else if (lidar.marche === false) this.definir('lidar', 'neutre', 'Lidar', 'Lidar arrêté');
    else if (lidar.ok) this.definir('lidar', 'ok', 'Lidar', `Lidar ${nombre(lidar.hz, 1)} Hz`);
    else this.definir('lidar', 'critique', 'Lidar', 'Lidar absent');

    // Caméras.
    const noms = d.cameras.filter((n) => n === 'gauche' || n === 'droite');
    const camerasConnues = noms.filter((n) => c[n]);
    if (!e || !camerasConnues.length) this.definir('cameras', 'neutre', 'Caméras', 'Caméras : inconnu');
    else {
      const ok = noms.filter((n) => c[n]?.ok).length;
      const niveau = ok === noms.length && ok > 0 ? 'ok' : ok > 0 ? 'alerte' : 'critique';
      this.definir('cameras', niveau, 'Caméras', `Caméras ${ok}/${Math.max(2, noms.length)}`);
    }

    // Stéréo.
    const stereo = c.stereo;
    if (!stereo) this.definir('stereo', 'neutre', 'Stéréo', 'Stéréo : inconnue');
    else if (!stereo.ok) this.definir('stereo', 'alerte', 'Stéréo', 'Stéréo absente');
    else if (stereo.calibree === false) this.definir('stereo', 'alerte', 'Stéréo', 'Stéréo non calibrée');
    else this.definir('stereo', 'ok', 'Stéréo', `Stéréo ${nombre(stereo.hz, 1)} Hz`);

    // GPS.
    const gnss = e?.gnss;
    if (!gnss) this.definir('gps', 'neutre', 'GPS', 'GPS : inconnu');
    else if (!gnss.fix || gnss.fix === 'aucun') this.definir('gps', 'alerte', 'GPS', `GPS sans position · ${gnss.satellites ?? 0} sat.`);
    else this.definir('gps', 'ok', 'GPS', `GPS ${FIX_GNSS[gnss.fix] || gnss.fix} · ${gnss.satellites ?? '?'} sat.`);

    // IA.
    const ia = d.ia;
    if (!enLigne || !ia) this.definir('ia', 'neutre', 'IA', 'IA : inconnue');
    else if (ia.profil === 'aucun') this.definir('ia', 'neutre', 'IA', 'IA désactivée');
    else if (ia.disponible === false) this.definir('ia', 'alerte', 'IA', `IA indisponible${ia.raison ? ` : ${ia.raison}` : ''}`);
    else if (e?.ia?.occupee) this.definir('ia', 'info', 'IA', `IA ${ia.profil} : réflexion`);
    else this.definir('ia', 'ok', 'IA', `IA ${ia.profil}`);

    // Wi-Fi.
    const wifi = s?.wifi;
    if (!wifi || !Number.isFinite(wifi.signal_pct)) this.definir('wifi', 'neutre', 'Wi-Fi', 'Wi-Fi : inconnu');
    else {
      const niveau = wifi.signal_pct >= 50 ? 'ok' : wifi.signal_pct >= 25 ? 'alerte' : 'critique';
      this.definir('wifi', niveau, `${Math.round(wifi.signal_pct)} %`, `Wi-Fi ${wifi.ssid || ''} ${Math.round(wifi.signal_pct)} %`);
    }
  }
}
