// Tableau de télémétrie : tous les capteurs du robot, les consignes et l'état du système, en direct.

import { CourbeTemps, Chiffres, formater } from './courbe.js';
import { VuePolaire } from './polaire.js';

const DEG = 180 / Math.PI;

const LIBELLES_ARRET = {
  obstacle_avant: ['Obstacle devant', 'critique'],
  obstacle_arriere: ['Obstacle derrière', 'critique'],
  chien_de_garde: ['Chien de garde', 'alerte'],
  arret_operateur: ['Arrêt opérateur', 'alerte'],
};
const LIBELLES_FIX = {
  rtk_fixe: 'RTK fixe', rtk_flottant: 'RTK flottant', dgnss: 'DGNSS', autonome: 'Autonome', aucun: 'Aucun',
};
const ICONES = { info: '•', succes: '✓', alerte: '!', erreur: '✕' };

function quaternionVersAngles([w, x, y, z]) {
  return {
    cap: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    roulis: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    tangage: Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
  };
}

export class Telemetrie {
  constructor(corps, boutonsOnglets) {
    this.corps = corps;
    this.boutons = boutonsOnglets;
    this.onglets = {};
    this.courbes = [];
    this.frequences = {};
    this.compteurs = {};
    this.tDerniereCommande = null;
    this.derniereCommande = null;
    this.odometrie = null;
    this.rover = null;
    this.verites = [];
    this.posesEnAttente = [];
    for (const bouton of boutonsOnglets) {
      bouton.addEventListener('click', () => this.afficher(bouton.dataset.onglet));
    }
    setInterval(() => this._calculerFrequences(), 1000);
  }

  _tuile(onglet) {
    const tuile = document.createElement('div');
    tuile.className = 'tuile';
    this.onglets[onglet].append(tuile);
    return tuile;
  }

  _courbe(onglet, options) {
    const courbe = new CourbeTemps(this._tuile(onglet), options);
    this.courbes.push(courbe);
    return courbe;
  }

  initialiser(bienvenue) {
    this.rover = bienvenue.rover;
    this.corps.replaceChildren();
    this.courbes = [];
    for (const nom of ['conduite', 'inertiel', 'capteurs', 'energie', 'vision', 'systeme']) {
      const groupe = document.createElement('div');
      groupe.className = 'onglet';
      groupe.style.display = 'contents';
      this.onglets[nom] = groupe;
      this.corps.append(groupe);
    }
    const lim = this.rover.limites;

    // Conduite
    this.cVitesse = this._courbe('conduite', {
      titre: 'Vitesse', unite: 'm/s', source: 'commande · firmware · codeurs',
      series: ['consigne du cerveau', 'consigne du contrôleur', 'mesurée'],
      min: -lim.vitesse_arriere_max, max: lim.vitesse_max,
    });
    this.cBraquage = this._courbe('conduite', {
      titre: 'Braquage', unite: '°', source: 'commande · firmware · servo',
      series: ['consigne du cerveau', 'consigne du contrôleur', 'mesuré'],
      min: -lim.braquage_max_deg - 2, max: lim.braquage_max_deg + 2, decimales: 1,
    });
    this.cRoues = this._courbe('conduite', {
      titre: 'Roues', unite: 'rad/s', source: 'codeurs 2048 imp./tour',
      series: ['avant gauche', 'avant droite', 'arrière gauche', 'arrière droite'], decimales: 1,
    });
    this.chConduite = new Chiffres(this._tuile('conduite'), 'Conduite', 'mesures', [
      ['vitesse', 'Vitesse', 'm/s'], ['distance', 'Distance parcourue', 'm'],
      ['commandes', 'Consignes reçues', '/s'], ['arret', 'Protection active', ''],
    ]);

    // Centrale inertielle
    const imu = this.rover.capteurs.imu.modele;
    this.cGyro = this._courbe('inertiel', {
      titre: 'Gyroscope', unite: '°/s', source: imu, series: ['x (roulis)', 'y (tangage)', 'z (lacet)'], decimales: 1,
    });
    this.cAccel = this._courbe('inertiel', {
      titre: 'Accéléromètre', unite: 'm/s²', source: imu, series: ['x (avant)', 'y (gauche)', 'z (haut)'],
    });
    this.cAttitude = this._courbe('inertiel', {
      titre: 'Attitude', unite: '°', source: `${imu} (orientation fusionnée)`, series: ['roulis', 'tangage'], decimales: 1,
    });
    this.chImu = new Chiffres(this._tuile('inertiel'), 'Orientation', imu, [
      ['cap', 'Cap', '°'], ['roulis', 'Roulis', '°'], ['tangage', 'Tangage', '°'], ['freq', 'Fréquence', 'Hz'],
    ]);

    // Lidar et GNSS
    this.polaire = new VuePolaire(this._tuile('capteurs'), this.rover);
    this.cLidarMin = this._courbe('capteurs', {
      titre: 'Distance libre devant', unite: 'm', source: 'lidar, secteur ±30°', series: ['plus proche écho'], min: 0, max: 8,
    });
    this.cGnss = this._courbe('capteurs', {
      titre: 'Incertitude GNSS', unite: 'm', source: this.rover.capteurs.gnss.modele, series: ['σ horizontal'], min: 0, decimales: 2,
    });
    this.cLocalisation = this._courbe('capteurs', {
      titre: 'Localisation', unite: 'cm', source: 'EKF du cerveau',
      series: ['erreur réelle (vérité simulée)', 'incertitude estimée (1σ)'], min: 0, decimales: 1,
    });
    this.chGnss = new Chiffres(this._tuile('capteurs'), 'GNSS', 'antenne ANN-MB', [
      ['fix', 'Solution', ''], ['sigma', 'Précision (1σ)', 'cm'], ['sat', 'Satellites', ''], ['freq', 'Fréquence', 'Hz'],
    ]);

    // Moteur et batterie
    this.cCouple = this._courbe('energie', {
      titre: 'Couple moteur', unite: 'N·m', source: 'VESC', series: ['couple'],
    });
    this.cRegime = this._courbe('energie', {
      titre: 'Régime moteur', unite: 'tr/min', source: 'VESC', series: ['régime'], decimales: 0,
    });
    this.cTension = this._courbe('energie', {
      titre: 'Tension batterie', unite: 'V', source: 'BMS', series: ['tension'], decimales: 2,
    });
    this.cCourant = this._courbe('energie', {
      titre: 'Courant batterie', unite: 'A', source: 'BMS', series: ['courant'], decimales: 1,
    });
    this.chEnergie = new Chiffres(this._tuile('energie'), 'Énergie', 'BMS', [
      ['charge', 'Charge', '%'], ['puissance', 'Puissance', 'W'], ['tension', 'Tension', 'V'],
    ]);

    // Système et journal
    const tuileJournal = this._tuile('systeme');
    tuileJournal.style.gridColumn = 'span 2';
    const h = document.createElement('h3');
    h.textContent = 'Journal du cerveau';
    this.journal = document.createElement('ol');
    this.journal.className = 'journal-liste';
    this.journal.setAttribute('aria-live', 'polite');
    tuileJournal.append(h, this.journal);
    this.chLiaison = new Chiffres(this._tuile('systeme'), 'Liaison robot', 'messages reçus', [
      ['scan', 'Lidar', 'Hz'], ['imu', 'IMU', 'Hz'], ['odometrie', 'Codeurs', 'Hz'], ['gnss', 'GNSS', 'Hz'],
      ['firmware', 'Contrôleur', 'Hz'], ['image', 'Caméra', 'Hz'], ['perdus', 'Télémétrie perdue', ''],
    ]);
    this.chCerveau = new Chiffres(this._tuile('systeme'), 'Calcul', 'serveur', [
      ['cpu_sim', 'Simulation', '% cœur'], ['cpu_camera', 'Rendu caméra', '% cœur'],
      ['cpu_cerveau', 'Cerveau', '% cœur'],
      ['collisions', 'Collisions (vérité)', ''],
    ]);
    this.afficher('conduite');
  }

  afficher(nom) {
    for (const [cle, groupe] of Object.entries(this.onglets)) {
      groupe.style.display = cle === nom ? 'contents' : 'none';
    }
    for (const bouton of this.boutons) bouton.setAttribute('aria-selected', String(bouton.dataset.onglet === nom));
  }

  _compter(type) {
    this.compteurs[type] = (this.compteurs[type] ?? 0) + 1;
  }

  _calculerFrequences() {
    this.frequences = { ...this.compteurs };
    this.compteurs = {};
    if (!this.chLiaison) return;
    // IMU et codeurs sont retransmis un sur deux par le serveur.
    const facteur = { imu: 2, odometrie: 2 };
    for (const type of ['scan', 'imu', 'odometrie', 'gnss', 'firmware', 'image']) {
      this.chLiaison.definir(type, formater((this.frequences[type] ?? 0) * (facteur[type] ?? 1), 0));
    }
    this.chConduite.definir('commandes', formater(this.frequences.commande ?? 0, 0));
    this.chImu.definir('freq', formater((this.frequences.imu ?? 0) * 2, 0));
    this.chGnss.definir('freq', formater(this.frequences.gnss ?? 0, 0));
  }

  // ---------------------------------------------------------------- messages
  surMessage(m) {
    if (!this.rover) return;
    this._compter(m.type);
    const r = this.rover.geometrie.rayon_roue;
    switch (m.type) {
      case 'commande':
        this.derniereCommande = m;
        break;
      case 'odometrie': {
        const v = ((m.roue_ar_g + m.roue_ar_d) / 2) * r;
        this.cRoues.ajouter(m.t, [m.roue_av_g ?? null, m.roue_av_d ?? null, m.roue_ar_g, m.roue_ar_d]);
        const c = this.derniereCommande, f = this.firmware;
        this.cVitesse.ajouter(m.t, [c?.vitesse ?? null, f?.vitesse_consigne ?? null, v]);
        this.cBraquage.ajouter(m.t, [
          c ? c.braquage * DEG : null, f ? f.braquage_consigne * DEG : null, m.braquage * DEG,
        ]);
        if (this.odometrie) this.distance = (this.distance ?? 0) + Math.abs(v) * (m.t - this.odometrie.t);
        this.odometrie = m;
        this.chConduite.definir('vitesse', formater(v, 2));
        this.chConduite.definir('distance', formater(this.distance ?? 0, 1));
        break;
      }
      case 'imu': {
        const [gx, gy, gz] = m.vitesse_angulaire;
        this.cGyro.ajouter(m.t, [gx * DEG, gy * DEG, gz * DEG]);
        this.cAccel.ajouter(m.t, m.acceleration);
        const a = quaternionVersAngles(m.orientation);
        this.cAttitude.ajouter(m.t, [a.roulis * DEG, a.tangage * DEG]);
        this.chImu.definir('cap', formater(((a.cap * DEG) + 360) % 360, 0));
        this.chImu.definir('roulis', formater(a.roulis * DEG, 1));
        this.chImu.definir('tangage', formater(a.tangage * DEG, 1));
        break;
      }
      case 'scan': {
        this.polaire.mettreAJour(m);
        let proche = null;
        m.distances.forEach((d, i) => {
          const a = m.angle_min + i * m.pas;
          if (d !== null && Math.abs(Math.atan2(Math.sin(a), Math.cos(a))) <= Math.PI / 6) {
            proche = proche === null ? d : Math.min(proche, d);
          }
        });
        this.cLidarMin.ajouter(m.t, [proche]);
        break;
      }
      case 'gnss':
        this.cGnss.ajouter(m.t, [m.sigma]);
        this.chGnss.definir('fix', LIBELLES_FIX[m.fix] ?? m.fix);
        this.chGnss.definir('sigma', formater(m.sigma * 100, 0));
        this.chGnss.definir('sat', String(m.satellites));
        break;
      case 'firmware': {
        this.firmware = m;
        this.cCouple.ajouter(m.t, [m.couple_moteur]);
        this.cRegime.ajouter(m.t, [m.regime_moteur]);
        this.cTension.ajouter(m.t, [m.tension_batterie]);
        this.cCourant.ajouter(m.t, [m.courant_batterie]);
        this.chEnergie.definir('charge', formater(m.etat_charge * 100, 1));
        this.chEnergie.definir('puissance', formater(m.tension_batterie * m.courant_batterie, 0));
        this.chEnergie.definir('tension', formater(m.tension_batterie, 1));
        this.chConduite.definir('arret', m.arret ? LIBELLES_ARRET[m.arret][0] : 'Aucune');
        break;
      }
      case 'pose_estimee':
        // Comparée à la vérité dès que la vue simulée postérieure est arrivée.
        this.posesEnAttente.push(m);
        break;
      case 'vue':
        this.verites.push({ t: m.t, x: m.x, y: m.y });
        while (this.verites.length > 90) this.verites.shift();
        while (this.posesEnAttente.length && this.posesEnAttente[0].t <= m.t) {
          const p = this.posesEnAttente.shift();
          const verite = this._veriteA(p.t);
          if (verite) this.cLocalisation.ajouter(p.t, [Math.hypot(verite.x - p.x, verite.y - p.y) * 100, p.sigma_xy * 100]);
        }
        if (this.posesEnAttente.length > 50) this.posesEnAttente.splice(0, this.posesEnAttente.length - 50);
        this.chCerveau.definir('cpu_sim', formater(m.cpu_simulation * 100, 1));
        if (m.cpu_camera !== undefined) this.chCerveau.definir('cpu_camera', formater(m.cpu_camera * 100, 1));
        this.chCerveau.definir('collisions', String(m.collisions));
        this.chLiaison.definir('perdus', String(m.telemetrie_perdue));
        break;
      case 'mesures':
        if (m.valeurs.cpu_cerveau !== undefined) {
          this.chCerveau.definir('cpu_cerveau', formater(m.valeurs.cpu_cerveau * 100, 1));
        }
        break;
      case 'journal':
        this.ajouterJournal(m);
        break;
      default:
    }
  }

  _veriteA(t) {
    // Vérité simulée interpolée à l'instant exact de la pose estimée (sinon, à 1,5 m/s, un
    // décalage de 16 ms entre les deux flux ajouterait 2,4 cm d'erreur fictive).
    const v = this.verites;
    for (let i = v.length - 1; i > 0; i--) {
      if (v[i - 1].t <= t && t <= v[i].t) {
        const f = (t - v[i - 1].t) / Math.max(v[i].t - v[i - 1].t, 1e-6);
        return { x: v[i - 1].x + f * (v[i].x - v[i - 1].x), y: v[i - 1].y + f * (v[i].y - v[i - 1].y) };
      }
    }
    return null;
  }

  ajouterJournal(m) {
    const li = document.createElement('li');
    li.className = `niveau-${m.niveau}`;
    const t = document.createElement('span');
    t.className = 't';
    t.textContent = `${formater(m.t, 1)} s`;
    const icone = document.createElement('span');
    icone.className = 'icone';
    icone.textContent = ICONES[m.niveau] ?? '•';
    const texte = document.createElement('span');
    texte.textContent = m.texte;
    const mesures = Object.entries(m.mesures ?? {});
    if (mesures.length) {
      const detail = document.createElement('span');
      detail.className = 'mesure';
      detail.textContent = ` (${mesures.map(([k, v]) => `${k} : ${typeof v === 'number' ? formater(v, 2) : v}`).join(', ')})`;
      texte.append(detail);
    }
    li.append(t, icone, texte);
    this.journal.prepend(li);
    while (this.journal.children.length > 200) this.journal.lastChild.remove();
  }

  dessiner() {
    for (const courbe of this.courbes) courbe.dessiner();
    this.polaire?.dessiner();
  }
}

export { LIBELLES_ARRET };
