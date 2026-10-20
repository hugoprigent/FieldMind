// Point d'entrée du poste opérateur : relie la connexion, la vue 3D, la carte, la télémétrie et le chat.

import { Connexion } from './connexion.js';
import { Scene3D } from './scene3d.js';
import { MiniCarte } from './minicarte.js';
import { Telemetrie, LIBELLES_ARRET } from './telemetrie.js';
import { Clavier } from './clavier.js';
import { formater } from './courbe.js';
import { VueReconstruite } from './vue_reconstruite.js';
import { PanneauVision } from './vision.js';

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;
const LIBELLES_ETAT = {
  attente: 'En attente', manuel: 'Conduite manuelle', calcul: 'Calcul du chemin', route: 'En route',
  bloque: 'Bloqué', atteint: 'Objectif atteint', echec: 'Échec',
};
const LIBELLES_LIAISON = {
  connexion: ['Connexion…', 'attente'], ok: ['Connecté', 'ok'], perdue: ['Liaison perdue, reconnexion…', 'alerte'],
  fermee: ['Session terminée', 'erreur'],
};

const scene = new Scene3D($('vue3d'), $('vue-camera'));
const carte = new MiniCarte($('minicarte'), (x, y) => connexion.envoyer({ type: 'objectif', x, y }));
const telemetrie = new Telemetrie($('telemetrie-corps'), document.querySelectorAll('[data-onglet]'));
const vision = new PanneauVision((m) => connexion.envoyer(m), $('flux-camera'), $('flux-profondeur'));
let fournisseurChat = '';
let vueReconstruite = null;

function pastille(element, texte, etat) {
  element.dataset.etat = etat;
  element.querySelector('.texte').textContent = texte;
}

function ajouterChat(texte, auteur, detail, refus = false) {
  const li = document.createElement('li');
  li.className = `${auteur}${refus ? ' refus' : ''}`;
  li.textContent = texte;
  if (detail) {
    const petit = document.createElement('small');
    petit.textContent = detail;
    li.append(petit);
  }
  const fil = $('chat-fil');
  fil.append(li);
  fil.scrollTop = fil.scrollHeight;
}

function mettreAJourMesures(valeurs) {
  const libelles = {
    distance_objectif: ['Distance à l’objectif', 'm', 1],
    temps_calcul_ms: ['Dernier calcul de chemin', 'ms', 1],
    nb_calculs: ['Calculs de chemin', '', 0],
    distance_parcourue: ['Distance parcourue', 'm', 1],
    distance_min_obstacle: ['Plus proche obstacle (lidar)', 'm', 2],
    temps_stereo_ms: ['Profondeur stéréo (SGBM)', 'ms', 0],
    sigma_position: ['Incertitude de position', 'cm', 0],
    duree_mission: ['Durée de la mission', 's', 1],
  };
  const dl = $('mesures-mission');
  dl.replaceChildren();
  for (const [cle, [libelle, unite, dec]] of Object.entries(libelles)) {
    if (valeurs[cle] === undefined || valeurs[cle] === null) continue;
    const dt = document.createElement('dt');
    dt.textContent = libelle;
    const dd = document.createElement('dd');
    const v = cle === 'sigma_position' ? valeurs[cle] * 100 : valeurs[cle];
    dd.textContent = `${formater(v, dec)} ${unite}`.trim();
    dl.append(dt, dd);
  }
}

function surMessage(m) {
  telemetrie.surMessage(m);
  switch (m.type) {
    case 'bienvenue':
      document.body.dataset.mode = m.mode;
      vueReconstruite = m.mode === 'robot_reel' ? new VueReconstruite(m.rover.geometrie) : null;
      scene.initialiser(m);
      carte.initialiser(m);
      telemetrie.initialiser(m);
      vision.initialiser(m, telemetrie);
      telemetrie.afficher('conduite');
      carte.obstaclesVision = [];
      scene.afficherObstaclesVision([]);
      definirSourceCamera(m.rover.capteurs.camera && m.mode === 'simulation' ? 'robot' : '3d');
      fournisseurChat = m.chat.fournisseur;
      $('chat-fournisseur').textContent = fournisseurChat;
      break;
    case 'vue':
      scene.mettreAJourVue(m);
      $('hud-vitesse').textContent = formater(m.vitesse, 2);
      $('hud-braquage').textContent = formater(((m.braquage_gauche + m.braquage_droit) / 2) * DEG, 0);
      if (!carte.poseEstimeeRecue) carte.pose = { x: m.x, y: m.y, cap: m.cap, sigma_xy: 0 };
      break;
    case 'scan':
      scene.mettreAJourScan(m);
      break;
    case 'firmware': {
      const arret = $('hud-arret');
      const [texte, niveau] = m.arret ? LIBELLES_ARRET[m.arret] : ['Aucun arrêt', 'ok'];
      arret.textContent = texte;
      arret.dataset.niveau = niveau;
      break;
    }
    case 'pose_estimee':
      carte.poseEstimeeRecue = true;
      carte.pose = m;
      if (vueReconstruite) {
        const vue = vueReconstruite.vue(m);
        scene.mettreAJourVue(vue);
        $('hud-vitesse').textContent = formater(vue.vitesse, 2);
        $('hud-braquage').textContent = formater(vueReconstruite.braquage * DEG, 0);
      }
      break;
    case 'imu':
      vueReconstruite?.surImu(m);
      break;
    case 'odometrie':
      vueReconstruite?.surOdometrie(m);
      break;
    case 'carte':
      carte.appliquerCarte(m);
      break;
    case 'image':
      vision.surImage(m);
      break;
    case 'profondeur':
      vision.surProfondeur(m);
      break;
    case 'traversabilite':
      carte.appliquerTraversabilite(m);
      break;
    case 'vision':
      vision.surVision(m);
      carte.reperesIA = m.objets.filter((o) => ['personne', 'animal', 'vehicule', 'eau', 'trou'].includes(o.categorie));
      carte.obstaclesVision = m.obstacles;
      scene.afficherObstaclesVision(m.obstacles);
      break;
    case 'etat_vision':
      vision.surEtat(m);
      etatObservation(m);
      break;
    case 'note_ia':
      ajouterNote(m);
      break;
    case 'chemin':
      carte.chemin = m.points;
      carte.objectifCourant = m.objectif;
      scene.afficherChemin(m.points, m.objectif);
      break;
    case 'statut':
      pastille($('etat-mission'), LIBELLES_ETAT[m.etat] ?? m.etat, m.etat);
      $('statut-message').textContent = m.message;
      break;
    case 'mesures':
      mettreAJourMesures(m.valeurs);
      break;
    case 'chat_reponse':
      ajouterChat(m.texte, 'cerveau', m.duree_ms >= 1 ? `${m.fournisseur} · ${formater(m.duree_ms, 0)} ms` : m.fournisseur, Boolean(m.refus));
      break;
    case 'reinitialisation':
      carte.reperesIA = [];
      $('observation-notes').replaceChildren();
      carte.grille = null;
      carte.traversabilite = null;
      carte.chemin = [];
      carte.objectifCourant = null;
      carte.obstaclesVision = [];
      scene.afficherChemin([], null);
      scene.afficherObstaclesVision([]);
      telemetrie.ajouterJournal({ t: 0, niveau: 'info', texte: 'Rover remis au départ, carte effacée.' });
      break;
    case 'erreur':
      telemetrie.ajouterJournal({ t: 0, niveau: 'erreur', texte: m.texte });
      break;
    case 'fin_session':
      $('statut-message').textContent = m.raison === 'complet'
        ? 'Trop de sessions ouvertes : réessaie dans quelques minutes.'
        : 'Session fermée après une longue inactivité. Recharge la page pour recommencer.';
      break;
    default:
  }
}

const LIBELLES_DECISION = { continuer: 'continuer', ralentir: 'ralentir', s_arreter: 'arrêt' };

function etatObservation(m) {
  const bouton = $('bascule-observation');
  bouton.setAttribute('aria-pressed', String(m.vigilance));
  bouton.textContent = m.vigilance ? 'Active' : 'Coupée';
  bouton.disabled = !m.disponible;
  $('observation-etat').textContent = m.message;
}

function ajouterNote(m) {
  const li = document.createElement('li');
  const entete = document.createElement('div');
  entete.className = 'entete';
  const quand = document.createElement('span');
  quand.textContent = `t = ${formater(m.t_image, 0)} s`;
  const decision = document.createElement('span');
  decision.className = 'decision';
  decision.dataset.decision = m.decision;
  decision.textContent = LIBELLES_DECISION[m.decision] + (m.decision !== 'continuer' && !m.appliquee ? ' (sans effet)' : '');
  entete.append(quand, decision);
  const texte = document.createElement('div');
  texte.textContent = m.texte;
  li.append(entete, texte);
  if (m.decision !== 'continuer' || m.reperes.length) {
    const detail = document.createElement('div');
    detail.className = 'reperes';
    detail.textContent = [m.reperes.join(', '), m.decision !== 'continuer' ? m.raison : ''].filter(Boolean).join(' · ');
    li.append(detail);
  }
  li.title = m.fournisseur;
  const liste = $('observation-notes');
  liste.prepend(li);
  while (liste.children.length > 30) liste.lastChild.remove();
}

$('bascule-observation').addEventListener('click', (e) => {
  const active = e.currentTarget.getAttribute('aria-pressed') !== 'true';
  connexion.envoyer({ type: 'vision', action: active ? 'vigilance_on' : 'vigilance_off' });
});

const connexion = new Connexion(surMessage, (etat) => {
  const [texte, code] = LIBELLES_LIAISON[etat] ?? [etat, 'attente'];
  pastille($('etat-liaison'), texte, code);
});
connexion.ouvrir();

function definirSourceCamera(source) {
  document.querySelector('.camera-embarquee').dataset.source = source;
  scene.rendrePip = source === '3d';
  document.querySelectorAll('[data-source-camera]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.sourceCamera === source)));
  $('legende-camera').textContent = {
    robot: 'Caméra avant (œil gauche)',
    profondeur: 'Profondeur stéréo · rouge 0 m → bleu 8 m · noir : inconnue',
    '3d': 'Vue 3D du poste',
  }[source];
}
for (const bouton of document.querySelectorAll('[data-source-camera]')) {
  bouton.addEventListener('click', () => definirSourceCamera(bouton.dataset.sourceCamera));
}

const arreter = () => connexion.envoyer({ type: 'stop' });
new Clavier((m) => connexion.envoyer(m), arreter);
$('bouton-stop').addEventListener('click', arreter);
$('bouton-reprendre').addEventListener('click', () => connexion.envoyer({ type: 'reprendre' }));
$('bouton-reinitialiser').addEventListener('click', () => connexion.envoyer({ type: 'reset' }));

$('chat-formulaire').addEventListener('submit', (e) => {
  e.preventDefault();
  const saisie = $('chat-saisie');
  const texte = saisie.value.trim();
  if (!texte) return;
  ajouterChat(texte, 'operateur');
  connexion.envoyer({ type: 'chat', texte: texte.slice(0, 200) });
  saisie.value = '';
});

for (const bouton of document.querySelectorAll('[data-camera]')) {
  bouton.addEventListener('click', () => {
    scene.definirCamera(bouton.dataset.camera);
    document.querySelectorAll('[data-camera]').forEach((b) => b.setAttribute('aria-pressed', String(b === bouton)));
  });
}
for (const bouton of document.querySelectorAll('[data-cadrage]')) {
  bouton.addEventListener('click', () => {
    carte.definirCadrage(bouton.dataset.cadrage);
    document.querySelectorAll('[data-cadrage]').forEach((b) => b.setAttribute('aria-pressed', String(b === bouton)));
  });
}
function bascule(id, appliquer) {
  const bouton = $(id);
  bouton.addEventListener('click', () => {
    const actif = bouton.getAttribute('aria-pressed') !== 'true';
    bouton.setAttribute('aria-pressed', String(actif));
    appliquer(actif);
  });
}
bascule('bascule-lidar', (actif) => { scene.afficherLidar = actif; });
bascule('bascule-chemin', (actif) => { scene.afficherLeChemin = actif; });
$('bascule-telemetrie').addEventListener('click', (e) => {
  const section = document.querySelector('.telemetrie');
  const reduite = section.classList.toggle('reduite');
  e.currentTarget.textContent = reduite ? 'Déplier' : 'Réduire';
  e.currentTarget.setAttribute('aria-expanded', String(!reduite));
});

let precedent = performance.now();
function boucle(maintenant) {
  const dt = Math.min(0.1, (maintenant - precedent) / 1000);
  precedent = maintenant;
  scene.rendre(dt);
  carte.dessiner();
  telemetrie.dessiner();
  requestAnimationFrame(boucle);
}
requestAnimationFrame(boucle);

// Accès pour les tests de bout en bout (Playwright).
window.fieldmind = { connexion, scene, carte, telemetrie, vision };
