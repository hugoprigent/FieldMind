// Libellés du poste : codes du protocole → français court.

import { nombre } from './outils.js';

export const MODES = { arret: 'Arrêt', manuel: 'Manuel', auto: 'Auto', banc: 'Banc' };

/** Raisons de sécurité : libellé et gravité (critique = arrêt, alerte = limitation, neutre). */
export const RAISONS = {
  arret_urgence: ['Arrêt d’urgence', 'critique'],
  non_arme: ['Désarmé', 'neutre'],
  mode_arret: ['Mode arrêt', 'neutre'],
  obstacle_avant: ['Obstacle devant', 'critique'],
  obstacle_arriere: ['Obstacle derrière', 'critique'],
  obstacle_bas: ['Obstacle bas', 'critique'],
  lidar_absent: ['Lidar absent', 'critique'],
  stereo_absente: ['Stéréo absente', 'alerte'],
  esp_absent: ['ESP absent', 'critique'],
  esp_defaut: ['Défaut ESP', 'critique'],
  inclinaison: ['Inclinaison', 'critique'],
  choc: ['Choc', 'critique'],
  blocage: ['Blocage', 'critique'],
  consigne_perimee: ['Consigne périmée', 'alerte'],
  operateur_perdu: ['Opérateur perdu', 'critique'],
  geofence: ['Hors zone', 'critique'],
  ralenti_obstacle: ['Ralenti', 'alerte'],
  banc: ['Banc', 'neutre'],
  remorque: ['Remorque', 'neutre'],
  remorque_portefeuille: ['Remorque trop braquée', 'critique'],
  remorque_incertaine: ['Angle remorque incertain', 'alerte'],
  remorque_recul_auto: ['Pas de recul (remorque)', 'alerte'],
  zone: ['Bord de zone', 'critique'],
  personne_proche: ['Personne proche', 'alerte'],
};

export function raison(code) {
  return RAISONS[code] || [String(code).replaceAll('_', ' '), 'alerte'];
}

export const ETATS_MISSION = {
  attente: ['En attente', 'neutre'],
  route: ['En route', 'ok'],
  bloque: ['Bloqué', 'alerte'],
  termine: ['Terminé', 'ok'],
  echec: ['Échec', 'critique'],
  annule: ['Annulé', 'neutre'],
  pause: ['En pause', 'alerte'],
};

const CAMERAS = { gauche: 'gauche', droite: 'droite', profondeur: 'profondeur' };

/** Description courte d'une action de mission (types.py, Action). */
export function decrireAction(a) {
  if (!a || typeof a !== 'object') return '—';
  switch (a.type) {
    case 'avancer': {
      const d = Number(a.distance_m) || 0;
      const v = a.vitesse_m_s ? ` à ${nombre(a.vitesse_m_s, 2)} m/s` : '';
      return d < 0 ? `Reculer ${nombre(-d, d % 1 ? 1 : 0)} m${v}` : `Avancer ${nombre(d, d % 1 ? 1 : 0)} m${v}`;
    }
    case 'tourner': {
      const angle = Number(a.angle_deg) || 0;
      return `Tourner ${nombre(Math.abs(angle), 0)}° à ${angle >= 0 ? 'gauche' : 'droite'}`;
    }
    case 'aller_a':
      return `Aller à ${nombre(a.x, 1)} ; ${nombre(a.y, 1)} m${a.repere === 'odom' ? ' (carte)' : ''}`;
    case 'aller_gps':
      return a.nom ? `Aller à « ${a.nom} »` : 'Aller au point GPS';
    case 'aller_lieu':
      return `Aller à « ${a.nom || '?'} »`;
    case 'memoriser_lieu':
      return `Mémoriser « ${a.nom || '?'} »`;
    case 'aller_pixel':
      return `Aller au point visé${a.camera ? ` (${CAMERAS[a.camera] || a.camera})` : ''}`;
    case 'cap':
      return `Cap ${nombre(a.cap_deg, 0)}°`;
    case 'attendre':
      return `Attendre ${nombre(a.duree_s, 0)} s`;
    case 'retour_depart':
      return 'Retour au départ';
    case 'chercher':
      return `Chercher « ${a.objet || '?'} »`;
    case 'decrire':
      return 'Décrire la scène';
    case 'photo':
      return `Photo ${CAMERAS[a.camera] || ''}`.trim();
    case 'stop':
      return 'Stop';
    default:
      return String(a.type || 'action');
  }
}

export const CALIBRATIONS = [
  { nom: 'sens_roues', libelle: 'Sens des roues', icone: 'banc', confirmer: 'Roues levées ?' },
  { nom: 'vitesse', libelle: 'Vitesse', icone: 'avancer', confirmer: '2 m libres devant ?' },
  { nom: 'braquage', libelle: 'Braquage', icone: 'gauche90', confirmer: 'Place pour tourner ?' },
  { nom: 'imu_repos', libelle: 'Centrale au repos', icone: 'bouclier' },
  { nom: 'stereo_capture', libelle: 'Capture mire', icone: 'camera' },
  { nom: 'stereo_calculer', libelle: 'Calcul stéréo', icone: 'stereo' },
];

/** Réglages connus : libellé, unité, groupe, pas du curseur. Les clés inconnues gardent leur nom. */
export const REGLAGES = {
  'vitesses.max_manuel_m_s': ['Max manuel', 'm/s', 'Vitesses', 0.05],
  'vitesses.max_auto_m_s': ['Max auto', 'm/s', 'Vitesses', 0.05],
  'vitesses.max_arriere_m_s': ['Max arrière', 'm/s', 'Vitesses', 0.05],
  'vitesses.defaut_action_m_s': ['Actions', 'm/s', 'Vitesses', 0.05],
  'traction.pwm_par_m_s': ['PWM pour 1 m/s', '%', 'Traction', 0.5],
  'traction.pwm_seuil': ['Zone morte', '%', 'Traction', 1],
  'traction.pwm_max': ['PWM max', '%', 'Traction', 1],
  'traction.delai_inversion_s': ['Délai inversion', 's', 'Traction', 0.05],
  'traction.equilibre_d': ['Équilibre droite / gauche', '×', 'Traction', 0.01],
  'traction.boucle_lacet': ['Tenue de cap', '', 'Traction'],
  'roues.impulsions_par_metre': ['Impulsions par mètre', '', 'Roues', 10],
  'direction.servo_par_degre_roue': ['Servo par degré', '°/°', 'Direction', 0.01],
  'direction.servo_neutre_deg': ['Neutre', '°', 'Direction', 0.5],
  'direction.servo_max_deg': ['Butée servo', '°', 'Direction', 1],
  'securite.marge_avant_m': ['Marge avant', 'm', 'Sécurité', 0.05],
  'securite.deceleration_m_s2': ['Décélération', 'm/s²', 'Sécurité', 0.1],
  'securite.perte_operateur_s': ['Perte opérateur', 's', 'Sécurité', 0.5],
  'securite.geofence_m': ['Rayon autorisé', 'm', 'Sécurité', 5],
  'perception.stereo_active': ['Stéréo', '', 'Perception'],
  'cameras.port_gauche': ['Port gauche', '', 'Caméras'],
  'cameras.port_droite': ['Port droite', '', 'Caméras'],
  'imu.axe_x': ['Axe x centrale', '', 'Centrale'],
  'imu.axe_y': ['Axe y centrale', '', 'Centrale'],
  'imu.axe_z': ['Axe z centrale', '', 'Centrale'],
  'audio.peripherique': ['Périphérique ALSA', '', 'Voix'],
};

export const GROUPES = {
  vitesses: 'Vitesses', traction: 'Traction', direction: 'Direction', securite: 'Sécurité',
  perception: 'Perception', cameras: 'Caméras', imu: 'Centrale', audio: 'Voix', voix: 'Voix', roues: 'Roues', gps: 'GPS',
};

export function reglage(cle) {
  const connu = REGLAGES[cle];
  if (connu) return { libelle: connu[0], unite: connu[1], groupe: connu[2], pas: connu[3] };
  const [section, nom = cle] = cle.split('.');
  return { libelle: nom.replaceAll('_', ' '), unite: '', groupe: GROUPES[section] || section };
}

export const FIX_GNSS = {
  aucun: 'Aucun', autonome: 'Autonome', dgnss: 'DGNSS', rtk_fixe: 'RTK fixe',
  rtk_flottant: 'RTK flottant', estime: 'Estimé',
};

export const NIVEAUX_JOURNAL = { info: 'Info', succes: 'Succès', alerte: 'Alerte', erreur: 'Erreur' };
