// Poste opérateur FieldMind : amorçage, menus, homme mort de la téléopération.
// `?mock=1` remplace le serveur par une simulation dans le navigateur (js/mock.js, chargé
// seulement dans ce cas).

import { ConnexionWeb, lireJeton } from './connexion.js';
import { Donnees } from './donnees.js';
import { Teleoperation, FREQUENCE_HZ } from './teleop.js';
import { $$, poserIcones, oublierCouleurs, ecrireLocal, lireLocal, signaler, signalerRefus, fermerConfirmation } from './outils.js';
import { DEG } from './geometrie.js';
import { Barre } from './vues/barre.js';
import { VuePilotage } from './vues/pilotage.js';
import { VueMission } from './vues/mission.js';
import { VueCarte } from './vues/carte.js';
import { VueCapteurs } from './vues/capteurs.js';
import { VueSysteme } from './vues/systeme.js';
import { VueReglages } from './vues/reglages.js';
import { initialiserLangue } from './langue.js';

const parametres = new URLSearchParams(location.search);
const jeton = lireJeton();
let connexion;
if (parametres.get('mock') === '1') {
  const { ConnexionSimulee } = await import('./mock.js');
  connexion = new ConnexionSimulee(parametres);
} else {
  connexion = new ConnexionWeb(jeton);
}

const donnees = new Donnees();
const teleop = new Teleoperation((message) => connexion.envoyer(message));
const arretsLocaux = [];
const rappelsTheme = [];
const rappelsOptions = [];

const ctx = {
  connexion,
  donnees,
  teleop,
  /** Trajectoire prévue (m/s, rad) : commande locale du joystick/clavier, sinon celle du rover. */
  prevision() {
    const p = donnees.profil;
    if (teleop.actif) {
      const c = teleop.commande();
      const vmax = donnees.reglages['vitesses.max_manuel_m_s'] ?? p.vitesses.max_manuel_m_s;
      return { vitesse: c.vitesse * vmax, braquage: c.braquage * p.geometrie.braquage_max_deg * DEG, locale: true };
    }
    const e = donnees.liaison === 'ok' ? donnees.etat : null;
    return {
      vitesse: e?.consigne?.vitesse ?? e?.commande?.vitesse ?? 0,
      braquage: e?.commande?.braquage ?? e?.consigne?.braquage ?? 0,
    };
  },
  /** Arrêt de tout ce que le poste commande localement (joystick, clavier, banc). */
  toutArreter() {
    teleop.arreter();
    for (const f of arretsLocaux) f();
  },
  surArret(fonction) {
    arretsLocaux.push(fonction);
  },
  async arretUrgence() {
    ctx.toutArreter();
    const r = await connexion.demander({ type: 'arret_urgence' });
    if (!r.ok) signalerRefus(r, 'ARRÊT non transmis : ');
  },
  async basculerVoix() {
    const active = Boolean(donnees.etat?.voix?.active);
    signalerRefus(await connexion.demander({ type: 'voix', active: !active }));
  },
  appliquerTheme(theme) {
    document.documentElement.dataset.theme = theme;
    ecrireLocal('theme', theme);
    oublierCouleurs();
    document.querySelector('meta[name="theme-color"]')?.setAttribute('content', theme === 'clair' ? '#eef1f4' : '#0b0d10');
    for (const f of rappelsTheme) f(theme);
  },
  surTheme(fonction) {
    rappelsTheme.push(fonction);
  },
  /** Option locale du poste modifiée (mini-carte, tuiles…). */
  option(nom, valeur) {
    ecrireLocal(nom, valeur);
    for (const f of rappelsOptions) f(nom, valeur);
  },
  surOption(fonction) {
    rappelsOptions.push(fonction);
  },
};

poserIcones();
initialiserLangue(() => ctx.toutArreter());
new Barre(ctx);
const vues = {
  pilotage: new VuePilotage(ctx),
  mission: new VueMission(ctx),
  carte: new VueCarte(ctx),
  capteurs: new VueCapteurs(ctx),
  systeme: new VueSysteme(ctx),
  reglages: new VueReglages(ctx),
};
ctx.appliquerTheme(lireLocal('theme', 'sombre'));

// ------------------------------------------------------------------ menus
let active = null;

function afficher(nom) {
  if (!vues[nom]) nom = 'pilotage';
  if (active === nom) return;
  fermerConfirmation(false);
  if (active) {
    vues[active].desactiver();
    document.getElementById(`vue-${active}`).hidden = true;
  }
  active = nom;
  document.getElementById(`vue-${nom}`).hidden = false;
  for (const lien of $$('.menu a')) {
    if (lien.dataset.vue === nom) lien.setAttribute('aria-current', 'page');
    else lien.removeAttribute('aria-current');
  }
  vues[nom].activer();
  ecrireLocal('vue', nom);
}

window.addEventListener('hashchange', () => afficher(location.hash.slice(1)));
afficher(location.hash.slice(1) || lireLocal('vue', 'pilotage'));

// ------------------------------------------------------------------ liaison
let camerasSimulees = null;
let camerasChargement = null;
connexion.surMessage = (message) => {
  if (message.type === 'bienvenue' && message.cameras_navigateur && connexion.envoyerImage) {
    if (camerasSimulees) camerasSimulees.visibilite();
    else if (!camerasChargement) camerasChargement = import('./camera_simulee.js').then(({ CamerasSimulees }) => {
      camerasSimulees = new CamerasSimulees(connexion);
    }).catch(e => console.warn('[simulation cameras] Software fallback:', e));
  }
  if (message.type === 'monde_simule') camerasSimulees?.monde(message);
  if (message.type === 'camera_simulee') camerasSimulees?.rendre(message);
  donnees.recevoir(message);
  if (message.type === 'etat') teleop.definirAutorisation(message.mode === 'manuel' && !message.arret_urgence);
};
connexion.surEtat = (etat) => {
  donnees.definirLiaison(etat);
  if (etat !== 'ok') ctx.toutArreter();
  if (etat === 'perdue') signaler('Liaison perdue', 'erreur', 2500);
};
connexion.ouvrir();

// Homme mort : 15 Hz tant qu'une commande est active ; tout s'arrête si l'état du rover ne
// vient plus (liaison muette) ou si la page perd le focus.
let dernierPas = performance.now();
setInterval(() => {
  const maintenant = performance.now();
  if (teleop.actif && donnees.ageEtat() > 1.5) ctx.toutArreter();
  teleop.pas(Math.min(0.2, (maintenant - dernierPas) / 1000));
  dernierPas = maintenant;
}, 1000 / FREQUENCE_HZ);
window.addEventListener('blur', () => ctx.toutArreter());
window.addEventListener('pagehide', () => ctx.toutArreter());
document.addEventListener('visibilitychange', () => { if (document.hidden) ctx.toutArreter(); });

// ------------------------------------------------------------------ dessin
function boucle() {
  if (!document.hidden && active) {
    try {
      vues[active].dessiner();
    } catch (erreur) {
      console.error('[poste] dessin', erreur);
    }
  }
  requestAnimationFrame(boucle);
}
requestAnimationFrame(boucle);

window.fieldmind = { ctx, vues }; // inspection depuis la console du navigateur
