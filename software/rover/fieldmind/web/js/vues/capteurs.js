// Menu Capteurs : lidar en grand, caméras et profondeur, courbes de la centrale et du cap,
// GPS, ESP, stéréo, fréquences et âges de chaque capteur.

import { $, el, texte, attribut, nombre, age, unite, lireLocal, ecrireLocal } from '../outils.js';
import { Radar } from '../radar.js';
import { VueCamera } from '../camera.js';
import { Courbe } from '../courbe.js';
import { FIX_GNSS } from '../textes.js';
import { DEG } from '../geometrie.js';

const CAPTEURS = [
  ['lidar', 'Lidar', 'hz', 'Hz'],
  ['gauche', 'Caméra gauche', 'ips', 'i/s'],
  ['droite', 'Caméra droite', 'ips', 'i/s'],
  ['drone', 'Drone (ESP32-C5)', 'ips', 'i/s'],
  ['stereo', 'Stéréo', 'hz', 'Hz'],
  ['imu', 'Centrale', 'hz', 'Hz'],
  ['gnss', 'GPS', 'hz', 'Hz'],
  ['esp', 'ESP', 'hz', 'Hz'],
  ['micro', 'Micro', 'hz', 'Hz'],
  ['odometrie', 'Odométrie', 'duree_ms', 'ms'],
];

/** Remplit un <dl> à partir de paires [libellé, valeur] (création au premier appel). */
function remplir(dl, paires) {
  if (dl.childElementCount !== paires.length * 2) {
    dl.replaceChildren(...paires.flatMap(([l]) => [el('dt', { text: l }), el('dd')]));
  }
  paires.forEach(([, v], i) => texte(dl.children[i * 2 + 1], v ?? '—'));
}

function pastille(element, niveau, libelle) {
  attribut(element, 'data-niveau', niveau);
  texte(element.querySelector('.libelle'), libelle);
}

export class VueCapteurs {
  constructor(ctx) {
    this.ctx = ctx;
    this.actif = false;
    this.radar = new Radar($('#k-radar'), ctx, { portee: lireLocal('portee-lidar', 8) });
    texte($('#k-portee'), `${this.radar.portee} m`);
    $('#k-portee').addEventListener('click', () => {
      const p = this.radar.changerPortee();
      texte($('#k-portee'), `${p} m`);
      ecrireLocal('portee-lidar', p);
    });
    this.cameras = ['gauche', 'droite', 'profondeur', 'drone'].map((nom) =>
      [nom, new VueCamera($(`#k-cam-${nom}`), ctx, { camera: nom, ips: 5, largeur: 480 })]);
    this.accel = new Courbe($('#k-accel'), { series: ['x', 'y', 'z'], unite: 'g', decimales: 2, etendueMin: 0.2 });
    this.gyro = new Courbe($('#k-gyro'), { series: ['x', 'y', 'z'], unite: '°/s', decimales: 1, etendueMin: 4 });
    this.cap = new Courbe($('#k-cap'), { series: ['odométrie'], unite: '°', decimales: 0, etendueMin: 10 });
    this.capContinu = null;
    ctx.donnees.sur('etat', (e) => this._ajouterMesures(e));
    ctx.donnees.sur('etat', () => { if (this.actif) this._majTextes(); });
    ctx.donnees.sur('bienvenue', () => {
      const presentes = new Set([...ctx.donnees.cameras, 'profondeur', 'drone']);
      for (const [nom] of this.cameras) $(`#k-cam-${nom}`).closest('figure').hidden = !presentes.has(nom);
    });
  }

  activer() {
    this.actif = true;
    for (const [, vue] of this.cameras) vue.activer();
    this._majTextes();
  }

  desactiver() {
    this.actif = false;
    for (const [, vue] of this.cameras) vue.desactiver();
  }

  dessiner() {
    this.radar.dessiner();
    for (const [, vue] of this.cameras) vue.dessiner();
    this.accel.dessiner();
    this.gyro.dessiner();
    this.cap.dessiner();
  }

  _ajouterMesures(e) {
    if (!Number.isFinite(e.t)) return;
    const imu = e.imu;
    this.accel.ajouter(e.t, imu?.accel_g || [null, null, null]);
    this.gyro.ajouter(e.t, (imu?.gyro_rad_s || [null, null, null]).map((v) => (Number.isFinite(v) ? v / DEG : null)));
    const cap = e.odom?.cap;
    if (Number.isFinite(cap)) {
      const deg = cap / DEG;
      if (this.capContinu === null) this.capContinu = deg;
      else this.capContinu += ((((deg - this.capContinu) % 360) + 540) % 360) - 180;
      this.cap.ajouter(e.t, [this.capContinu]);
    } else {
      this.cap.ajouter(e.t, [null]);
    }
  }

  _majTextes() {
    const d = this.ctx.donnees, e = d.liaison === 'ok' ? d.etat : null;
    const c = e?.capteurs || {};
    const scan = d.scan;
    texte($('#k-lidar-info'), c.lidar ? `${c.lidar.points ?? (scan ? scan.points.length / 2 : 0)} pts · ${nombre(c.lidar.hz, 1)} Hz` : '');

    for (const nom of ['gauche', 'droite', 'profondeur', 'drone']) {
      const info = nom === 'profondeur' ? c.stereo : c[nom];
      const frequence = nom === 'profondeur' ? info?.hz : info?.ips;
      const res = info?.resolution ? ` · ${info.resolution[0]}×${info.resolution[1]}` : '';
      texte($(`#k-cam-${nom}-info`), info ? `${nombre(frequence, 1)} ${nom === 'profondeur' ? 'Hz' : 'i/s'}${res}` : '');
    }

    // Centrale.
    const imu = e?.imu;
    pastille($('#k-imu-etat'), !imu ? 'neutre' : imu.etat === 'OK' ? 'ok' : imu.etat === 'DOUTEUX' ? 'alerte' : 'critique', imu?.etat || '—');
    remplir($('#k-imu'), [
      ['Roulis', unite(imu?.roulis_deg, 1, '°')],
      ['Tangage', unite(imu?.tangage_deg, 1, '°')],
      ['Lacet', unite(Number.isFinite(imu?.gyro_rad_s?.[2]) ? imu.gyro_rad_s[2] / DEG : null, 1, '°/s')],
      ['Température', unite(imu?.temperature_c, 1, '°C')],
    ]);

    // GPS.
    const g = e?.gnss;
    const fixe = g && g.fix && g.fix !== 'aucun';
    pastille($('#k-gps-etat'), !g ? 'neutre' : fixe ? 'ok' : 'alerte', g ? FIX_GNSS[g.fix] || g.fix || '—' : '—');
    remplir($('#k-gps'), [
      ['Latitude', Number.isFinite(g?.latitude) ? `${nombre(g.latitude, 6)}°` : '—'],
      ['Longitude', Number.isFinite(g?.longitude) ? `${nombre(g.longitude, 6)}°` : '—'],
      ['Altitude', unite(g?.altitude_m, 1, 'm')],
      ['Satellites', g ? String(g.satellites ?? '—') : '—'],
      ['HDOP', nombre(g?.hdop, 1)],
      ['Vitesse', unite(g?.vitesse_m_s, 2, 'm/s')],
      ['Cap vrai', unite(g?.cap_deg, 0, '°')],
    ]);

    // ESP.
    const esp = e?.esp;
    pastille($('#k-esp-etat'), !esp ? 'neutre' : !esp.connecte ? 'critique' : esp.defaut ? 'critique' : 'ok',
      !esp ? '—' : !esp.connecte ? 'Absent' : esp.defaut ? 'Défaut' : esp.firmware || 'OK');
    const sens = (s) => (s === 1 ? 'AR' : s === 0 ? 'AV' : '—');
    remplir($('#k-esp'), [
      ['Défaut', esp?.defaut ? esp.raison_defaut || 'oui' : esp ? 'non' : '—'],
      ['PWM G', esp ? `${esp.pwm_g ?? '—'} % ${sens(esp.sens_g)}` : '—'],
      ['PWM D', esp ? `${esp.pwm_d ?? '—'} % ${sens(esp.sens_d)}` : '—'],
      ['Servo', esp ? `${nombre(esp.servo_deg, 1)}°${esp.servo_actif === false ? ' (libre)' : ''}` : '—'],
      ['Frein', esp ? (esp.frein ? 'serré' : 'libre') : '—'],
      ['Chien de garde', esp?.chien_de_garde_ms ? `${esp.chien_de_garde_ms} ms` : esp ? 'aucun' : '—'],
      ['Dernière ligne', age(esp?.age_s)],
    ]);

    // Drone (BLE 5.0 CooleerX / Wi-Fi ESP32-C5).
    const dr = e?.drone;
    pastille($('#k-drone-etat'),
      !dr ? 'neutre' : dr.connecte ? 'ok' : dr.etat === 'scan' || dr.etat === 'connexion' ? 'alerte' : 'neutre',
      !dr ? '—' : dr.connecte ? 'Connecté' : dr.etat || '—');
    remplir($('#k-drone'), [
      ['État liaison', dr?.etat || '—'],
      ['Nom / Liaison', dr?.ssid || 'auto (BLE CooleerX / Wi-Fi)'],
      ['Adresse MAC / Canal', dr ? `${dr.bssid || '—'} · ch ${dr.canal || '—'}` : '—'],
      ['Signal RSSI', Number.isFinite(dr?.rssi_dbm) ? `${dr.rssi_dbm} dBm` : '—'],
      ['Hôte (Pi / ESP32)', dr?.ip_locale || '—'],
      ['Cible (MAC / IP)', dr?.passerelle || '—'],
      ['Protocole / MTU', dr ? `${dr.protocole || 'attente'} (vidéo ${dr.port_video || '—'} / cmd ${dr.port_cmd || 8888})` : '—'],
      ['Flux vidéo', dr ? `${nombre(dr.ips, 1)} i/s (${dr.images_rx ?? 0} images)` : '—'],
      ['Trafic reçu / émis', dr ? `RX ${dr.paquets_rx ?? 0} (${dr.ko_rx ?? 0} ko) · TX ${dr.paquets_tx ?? 0}` : '—'],
      ['Batterie drone', Number.isFinite(dr?.batterie_pct) ? `${dr.batterie_pct} %` : '1S LiPo (LED sur batterie)'],
      ['Commande vol', dr?.commande_vol || 'aucune'],
      ['Détectés (BLE/Wi-Fi)', dr?.reseaux_scan?.length ? dr.reseaux_scan.slice(0, 3).join(', ') : 'aucun'],
      ['Âge', age(dr?.age_s)],
    ]);

    // Stéréo.
    const st = c.stereo;
    pastille($('#k-stereo-etat'), !st ? 'neutre' : !st.ok ? 'alerte' : st.calibree ? 'ok' : 'alerte',
      !st ? '—' : !st.ok ? 'Absente' : st.calibree ? 'Calibrée' : 'Non calibrée');
    const obstacles = d.obstacles;
    remplir($('#k-stereo'), [
      ['Fréquence', unite(st?.hz, 1, 'Hz')],
      ['Âge', age(st?.age_s)],
      ['Obstacles', obstacles && performance.now() - obstacles.recu < 2000 ? String(obstacles.points.length / 2) : '—'],
      ['Portée', unite(obstacles?.portee, 1, 'm')],
    ]);

    // Fréquences et âges.
    const corps = $('#k-flux');
    if (corps.childElementCount !== CAPTEURS.length) {
      corps.replaceChildren(...CAPTEURS.map(([, libelle]) => el('tr', {},
        el('td', { text: libelle }),
        el('td', {}, el('span', { class: 'pastille', dataset: { niveau: 'neutre' } }, el('span', { class: 'point' }), el('span', { class: 'libelle', text: '—' }))),
        el('td'), el('td'))));
    }
    CAPTEURS.forEach(([cle, , champ, u], i) => {
      const ligne = corps.children[i], info = c[cle];
      const etatCapteur = !info ? ['neutre', '—'] : info.marche === false ? ['neutre', 'arrêté'] : info.ok ? ['ok', 'OK'] : ['critique', 'absent'];
      const p = ligne.children[1].firstElementChild;
      attribut(p, 'data-niveau', etatCapteur[0]);
      texte(p.querySelector('.libelle'), etatCapteur[1]);
      texte(ligne.children[2], Number.isFinite(info?.[champ]) ? `${nombre(info[champ], 1)} ${u}` : cle === 'micro' && info?.nom ? info.nom : '—');
      texte(ligne.children[3], age(info?.age_s));
    });
  }
}
