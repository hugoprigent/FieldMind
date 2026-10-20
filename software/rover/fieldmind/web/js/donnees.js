// Dernières données reçues du rover et diffusion aux menus (par type de message).

import { completerProfil, decoderXY } from './geometrie.js';

const MAX_JOURNAL = 1000, MAX_CHAT = 300, MAX_CONSOLE = 600;

export class Donnees {
  constructor() {
    this.ecouteurs = new Map();
    this.liaison = 'connexion';
    this.bienvenue = null;
    this.profil = completerProfil(null);
    this.materiel = null;
    this.reglables = {};
    this.reglages = {};
    this.cameras = ['gauche', 'droite'];
    this.ia = null;
    this.etat = null;
    this.recuEtat = 0; // performance.now() du dernier `etat`
    this.systeme = null;
    this.recuSysteme = 0;
    this.scan = null; // { t, points: Float32Array [x0, y0, x1, y1…] en m, recu }
    this.obstacles = null;
    this.carte = null;
    this.scene = null; // dernier message `scene` (vue 3D)
    this.gps = null; // dernier message `gps` (1 Hz)
    this.gpsOdom = null; // pose odométrique à sa réception (pour prolonger la position)
    this.recuGps = 0;
    this.journal = [];
    this.chat = [];
    this.console = [];
    this.vision = null;
    this.calibrations = {};
    this.transcription = null; // { texte, t, evenement }
    this.parole = false;
    this.nouvelleSession = true; // le prochain `bienvenue` suit une (re)connexion
  }

  sur(type, fonction) {
    if (!this.ecouteurs.has(type)) this.ecouteurs.set(type, new Set());
    this.ecouteurs.get(type).add(fonction);
    return () => this.ecouteurs.get(type)?.delete(fonction);
  }

  emettre(type, valeur) {
    for (const fonction of this.ecouteurs.get(type) || []) {
      try {
        fonction(valeur);
      } catch (erreur) {
        console.error(`[poste] ${type}`, erreur);
      }
    }
  }

  definirLiaison(etat) {
    if (etat === 'ok' && this.liaison !== 'ok') this.nouvelleSession = true;
    this.liaison = etat;
    this.emettre('liaison', etat);
  }

  /** Âge (s) du dernier `etat` reçu, Infinity si aucun. */
  ageEtat() {
    return this.recuEtat ? (performance.now() - this.recuEtat) / 1000 : Infinity;
  }

  recevoir(m) {
    switch (m.type) {
      case 'bienvenue':
        this.bienvenue = m;
        this.profil = completerProfil(m.profil);
        this.materiel = m.materiel || null;
        this.reglables = m.reglables || {};
        this.reglages = { ...(m.reglages || {}) };
        if (Array.isArray(m.cameras) && m.cameras.length) this.cameras = m.cameras;
        this.ia = m.ia || null;
        m.nouvelleSession = this.nouvelleSession;
        if (this.nouvelleSession) {
          // Après une (re)connexion, le serveur renvoie les derniers événements et la console.
          // Un `bienvenue` rediffusé (changement de profil IA) ne vide rien.
          this.journal = [];
          this.console = [];
          this.nouvelleSession = false;
        }
        break;
      case 'etat':
        this.etat = m;
        this.recuEtat = performance.now();
        break;
      case 'systeme':
        this.systeme = m;
        this.recuSysteme = performance.now();
        break;
      case 'scan':
        this.scan = { t: m.t, points: decoderXY(m.xy), recu: performance.now() };
        break;
      case 'obstacles':
        this.obstacles = { t: m.t, source: m.source, portee: m.portee_m, points: decoderXY(m.xy), recu: performance.now() };
        break;
      case 'carte':
        this.carte = m;
        break;
      case 'scene':
        this.scene = m;
        break;
      case 'gps':
        this.gps = m;
        this.gpsOdom = this.etat?.odom ? { ...this.etat.odom } : null;
        this.recuGps = performance.now();
        break;
      case 'journal':
        this.journal.push(m);
        if (this.journal.length > MAX_JOURNAL) this.journal.splice(0, this.journal.length - MAX_JOURNAL);
        break;
      case 'chat': {
        const i = m.id ? this.chat.findIndex((c) => c.id === m.id) : -1;
        if (i >= 0) {
          this.chat[i] = m;
        } else {
          // Remplace l'écho local d'un message de l'opérateur.
          if (m.role === 'operateur') {
            const local = this.chat.findIndex((c) => c.local && c.texte === m.texte);
            if (local >= 0) this.chat.splice(local, 1);
          }
          this.chat.push(m);
          if (this.chat.length > MAX_CHAT) this.chat.splice(0, this.chat.length - MAX_CHAT);
        }
        break;
      }
      case 'voix':
        if (m.evenement === 'debut_parole') this.parole = true;
        if (m.evenement === 'fin_parole' || m.evenement === 'transcription' || m.evenement === 'ignore') this.parole = false;
        if (m.evenement === 'transcription' || m.evenement === 'ignore' || m.evenement === 'erreur') {
          this.transcription = { texte: m.texte || m.message || '', t: m.t, evenement: m.evenement };
        }
        break;
      case 'vision':
        this.vision = { ...m, poseRecue: this.etat?.odom ? { ...this.etat.odom } : null };
        break;
      case 'console':
        for (const ligne of m.lignes || []) this.console.push(ligne);
        if (this.console.length > MAX_CONSOLE) this.console.splice(0, this.console.length - MAX_CONSOLE);
        break;
      case 'calibration':
        this.calibrations[m.nom] = m;
        break;
      case 'ia':
        // Hors protocole 1.0 : mise à jour du fournisseur IA si le serveur l'envoie.
        this.ia = { ...(this.ia || {}), ...m };
        delete this.ia.type;
        break;
      case 'reponse':
        if (m.ok && m.cle !== undefined && m.valeur !== undefined) this.reglages[m.cle] = m.valeur;
        break;
      default:
        break;
    }
    this.emettre(m.type, m);
  }

  /** Message local (écho immédiat de ce que l'opérateur écrit). */
  ajouterChatLocal(texte) {
    const m = { type: 'chat', id: `local-${Date.now()}`, t: Date.now() / 1000, role: 'operateur', texte, origine: 'texte', local: true };
    this.chat.push(m);
    this.emettre('chat', m);
  }
}
