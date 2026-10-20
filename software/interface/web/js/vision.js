// Caméra : flux de la caméra du robot, profondeur de la paire stéréo, et analyses à la demande
// du modèle de vision (boîtes, objets placés au sol, résumé).

import { formater } from './courbe.js';

function variable(nom) {
  return getComputedStyle(document.documentElement).getPropertyValue(nom).trim();
}

/** JPEG en base64 → URL blob: (la CSP interdit fetch sur data:, pas les images blob:). */
function urlJpeg(base64) {
  const octets = Uint8Array.from(atob(base64), (c) => c.charCodeAt(0));
  return URL.createObjectURL(new Blob([octets], { type: 'image/jpeg' }));
}

function element(balise, classe, texte) {
  const e = document.createElement(balise);
  if (classe) e.className = classe;
  if (texte !== undefined) e.textContent = texte;
  return e;
}

export class PanneauVision {
  constructor(envoyer, imageFlux, imageProfondeur) {
    this.envoyer = envoyer;
    this.imageFlux = imageFlux;
    this.imageProfondeur = imageProfondeur;
    this.urlFlux = null;
    this.urlProfondeur = null;
    this.derniereImage = null;
    this.obstacles = [];
    this.etat = null;
  }

  initialiser(bienvenue, telemetrie) {
    this.info = bienvenue.vision ?? { fournisseur: null };
    this.camera = bienvenue.rover.capteurs.camera ?? null;
    this.obstacles = [];
    this.analyses = 0;

    // Profondeur stéréo : carte en couleurs et mesures.
    const tuileProfondeur = telemetrie._tuile('vision');
    tuileProfondeur.classList.add('tuile-vision-image');
    tuileProfondeur.append(element('h3', '', 'Profondeur stéréo'));
    this.imageProfondeurGrande = element('img', 'vision-canvas');
    this.imageProfondeurGrande.alt = 'Profondeur calculée par la paire stéréo';
    this.mesuresProfondeur = element('p', 'vision-legende', '—');
    tuileProfondeur.append(this.imageProfondeurGrande, this.mesuresProfondeur);

    // Dernière analyse : image analysée et boîtes du modèle.
    const tuileImage = telemetrie._tuile('vision');
    tuileImage.classList.add('tuile-vision-image');
    tuileImage.append(element('h3', '', 'Dernière analyse de la caméra'));
    this.canvas = element('canvas', 'vision-canvas');
    this.canvas.width = this.camera?.largeur ?? 640;
    this.canvas.height = this.camera?.hauteur ?? 360;
    this.canvas.setAttribute('aria-label', 'Image analysée, avec les objets détectés');
    this.legende = element('p', 'vision-legende', '—');
    this.resume = element('blockquote', 'vision-resume');
    this.resume.hidden = true;
    const texte = element('div', 'vision-texte');
    texte.append(this.legende, this.resume);
    tuileImage.append(this.canvas, texte);
    this._dessinerVide();

    // Objets placés au sol.
    const tuileObjets = telemetrie._tuile('vision');
    tuileObjets.append(element('h3', '', 'Objets vus'));
    this.listeObjets = element('ol', 'vision-objets');
    this.listeObjets.append(element('li', 'vide', '—'));
    tuileObjets.append(this.listeObjets);

    // Commandes et état.
    const tuileCommandes = telemetrie._tuile('vision');
    tuileCommandes.append(element('h3', '', 'Modèle de vision'));
    this.nomFournisseur = element('p', 'vision-fournisseur', this.info.fournisseur ?? 'aucun');
    const boutons = element('div', 'vision-boutons');
    this.boutonAnalyser = element('button', '', 'Analyser la vue');
    this.boutonAnalyser.type = 'button';
    this.boutonAnalyser.addEventListener('click', () => this.envoyer({ type: 'vision', action: 'analyser' }));
    boutons.append(this.boutonAnalyser);
    this.messageEtat = element('p', 'vision-etat', '');
    this.compteur = element('p', 'vision-compteur', '');
    tuileCommandes.append(this.nomFournisseur, boutons, this.messageEtat, this.compteur);
    const disponible = Boolean(this.info.fournisseur) && Boolean(this.camera);
    this.boutonAnalyser.disabled = !disponible;
  }

  surProfondeur(m) {
    const url = urlJpeg(m.image);
    const precedente = this.urlProfondeur;
    this.urlProfondeur = url;
    this.imageProfondeur.onload = () => { if (precedente) URL.revokeObjectURL(precedente); };
    this.imageProfondeur.src = url;
    this.imageProfondeurGrande.src = url;
    const devant = m.obstacles_devant === null ? '> 4 m' : `${formater(m.obstacles_devant, 1)} m`;
    this.mesuresProfondeur.textContent =
      `t = ${formater(m.t, 1)} s · ${m.points} points · ${formater(m.duree_ms, 0)} ms · obstacle devant : ${devant}`;
  }

  // ---------------------------------------------------------------- flux
  surImage(m) {
    if (!m.donnees) return;
    const url = urlJpeg(m.donnees);
    const precedente = this.urlFlux;
    this.urlFlux = url;
    this.imageFlux.onload = () => { if (precedente) URL.revokeObjectURL(precedente); };
    this.imageFlux.src = url;
    this.derniereImage = m;
  }

  surEtat(m) {
    this.etat = m;
    this.boutonAnalyser.disabled = !m.disponible || m.en_cours;
    this.messageEtat.textContent = m.message;
    this.messageEtat.dataset.enCours = String(m.en_cours);
    this.compteur.textContent = `${m.analyses} analyse(s) réussie(s)${this.info.max_analyses ? ` · limite ${this.info.max_analyses} par session` : ''}`;
  }

  // ---------------------------------------------------------------- résultats
  surVision(m) {
    this.obstacles = m.obstacles;
    const quand = `t = ${formater(m.t_image, 1)} s`;
    const origine = m.raison === 'operateur' ? 'à la demande' : 'vigilance';
    if (m.erreur) {
      this.legende.textContent = `${quand} · ${origine} · échec : ${m.erreur}`;
      this.legende.dataset.niveau = 'alerte';
      return;
    }
    this.legende.dataset.niveau = 'ok';
    this.legende.textContent = `${quand} · ${origine} · ${m.fournisseur} · ${formater(m.duree_ms / 1000, 1)} s`;
    this.resume.hidden = !m.resume;
    this.resume.textContent = m.resume ?? '';
    this.resume.title = 'Résumé rédigé par le modèle de vision';
    if (m.image) {
      const image = new Image();
      const url = urlJpeg(m.image);
      image.onload = () => { this._dessinerAnalyse(image, m.objets); URL.revokeObjectURL(url); };
      image.src = url;
    } else {
      this._dessinerVide('Image non transmise');
      this._dessinerBoites(m.objets);
    }
    this._listerObjets(m.objets);
  }

  _dessinerVide(texte = 'En attente d’une analyse') {
    const ctx = this.canvas.getContext('2d');
    ctx.fillStyle = '#1c1c1b';
    ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    ctx.fillStyle = variable('--texte-3');
    ctx.font = '20px system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(texte, this.canvas.width / 2, this.canvas.height / 2);
    ctx.textAlign = 'start';
  }

  _dessinerAnalyse(image, objets) {
    const ctx = this.canvas.getContext('2d');
    ctx.drawImage(image, 0, 0, this.canvas.width, this.canvas.height);
    this._dessinerBoites(objets);
  }

  _couleur(objet) {
    if (objet.dans_carte) return variable('--etat-critique');
    if (objet.bloque) return variable('--etat-alerte');
    return variable('--serie-1');
  }

  _dessinerBoites(objets) {
    const ctx = this.canvas.getContext('2d');
    const w = this.canvas.width, h = this.canvas.height;
    ctx.font = '600 14px system-ui, sans-serif';
    for (const o of objets) {
      const [x0, y0, x1, y1] = o.boite.map((v, i) => (v / 1000) * (i % 2 ? h : w));
      const couleur = this._couleur(o);
      ctx.strokeStyle = couleur;
      ctx.lineWidth = 3;
      ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
      const texte = `${o.nom} · ${formater(o.distance, 1)} m`;
      const largeur = ctx.measureText(texte).width + 10;
      const ty = y0 > 22 ? y0 - 22 : y1 + 2;
      ctx.fillStyle = couleur;
      ctx.fillRect(x0, ty, largeur, 20);
      ctx.fillStyle = o.dans_carte ? '#ffffff' : '#111110';
      ctx.fillText(texte, x0 + 5, ty + 15);
    }
  }

  _listerObjets(objets) {
    this.listeObjets.replaceChildren();
    if (!objets.length) {
      this.listeObjets.append(element('li', 'vide', 'Aucun objet placé au sol.'));
      return;
    }
    for (const o of [...objets].sort((a, b) => a.distance - b.distance)) {
      const li = element('li');
      const pastille = element('i', 'cle');
      pastille.style.background = this._couleur(o);
      const nom = element('strong', '', o.nom);
      const detail = element('span', 'mesure',
        ` ${formater(o.distance, 1)} m ± ${formater(o.incertitude, 1)} · confiance ${formater(o.confiance * 100, 0)} %`);
      li.append(pastille, nom, detail);
      if (o.dans_carte) li.append(element('span', 'badge', 'ajouté à la carte'));
      else if (o.bloque) li.append(element('span', 'badge discret', 'bloquant'));
      this.listeObjets.append(li);
    }
  }
}
