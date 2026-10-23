// Liaison avec le rover : WebSocket /ws (JSON) avec reconnexion, demandes numérotées
// (réponse `reponse`), jeton facultatif et flux MJPEG des caméras.

const CLE_JETON = 'fieldmind.jeton';

/** Jeton : relu dans l'URL de la page (?jeton=…), sinon celui gardé en localStorage. */
export function lireJeton() {
  const parametres = new URLSearchParams(location.search);
  const jeton = parametres.get('jeton');
  try {
    if (jeton !== null) {
      localStorage.setItem(CLE_JETON, jeton);
      parametres.delete('jeton');
      const reste = parametres.toString();
      history.replaceState(null, '', `${location.pathname}${reste ? `?${reste}` : ''}${location.hash}`);
      return jeton;
    }
    return localStorage.getItem(CLE_JETON) || '';
  } catch {
    return jeton || '';
  }
}

export function enregistrerJeton(jeton) {
  try {
    if (jeton) localStorage.setItem(CLE_JETON, jeton);
    else localStorage.removeItem(CLE_JETON);
  } catch { /* stockage indisponible */ }
}

/** Partie commune au vrai serveur et au serveur simulé : demandes et réponses. */
export class ConnexionBase {
  constructor() {
    this.surMessage = () => {};
    this.surEtat = () => {};
    this.etat = 'connexion';
    this.prochainId = 1;
    this.attentes = new Map();
  }

  /** Envoie une demande portant un `id` ; résout avec le message `reponse` (ou un échec). */
  demander(message, delai = 6000) {
    const id = this.prochainId++;
    return new Promise((resoudre) => {
      if (!this.envoyer({ ...message, id })) {
        resoudre({ type: 'reponse', id, ok: false, erreur: 'liaison coupée' });
        return;
      }
      const minuterie = setTimeout(() => {
        this.attentes.delete(id);
        resoudre({ type: 'reponse', id, ok: false, erreur: 'pas de réponse', delai: true });
      }, delai);
      this.attentes.set(id, { resoudre, minuterie });
    });
  }

  _recevoir(message) {
    if (!message || typeof message !== 'object') return;
    if (message.type === 'reponse' && this.attentes.has(message.id)) {
      const { resoudre, minuterie } = this.attentes.get(message.id);
      clearTimeout(minuterie);
      this.attentes.delete(message.id);
      resoudre(message);
    }
    this.surMessage(message);
  }

  _changerEtat(etat) {
    this.etat = etat;
    if (etat !== 'ok') {
      for (const [id, { resoudre, minuterie }] of this.attentes) {
        clearTimeout(minuterie);
        resoudre({ type: 'reponse', id, ok: false, erreur: 'liaison coupée' });
      }
      this.attentes.clear();
    }
    this.surEtat(etat);
  }
}

export class ConnexionWeb extends ConnexionBase {
  constructor(jeton = '') {
    super();
    this.jeton = jeton;
    this.ws = null;
    this.tentatives = 0;
  }

  _url(chemin, parametres = {}) {
    const url = new URL(chemin, location.href);
    for (const [cle, valeur] of Object.entries(parametres)) url.searchParams.set(cle, valeur);
    if (this.jeton) url.searchParams.set('jeton', this.jeton);
    return url;
  }

  ouvrir() {
    const url = this._url('ws');
    url.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    let ws;
    try {
      ws = new WebSocket(url);
    } catch {
      this._reessayer();
      return;
    }
    this.ws = ws;
    this._changerEtat('connexion');
    ws.addEventListener('open', () => {
      this.tentatives = 0;
      this._changerEtat('ok');
    });
    ws.addEventListener('message', (evenement) => {
      let message;
      try {
        message = JSON.parse(evenement.data);
      } catch {
        return;
      }
      this._recevoir(message);
    });
    ws.addEventListener('close', () => {
      if (this.ws !== ws) return;
      this.ws = null;
      this._changerEtat('perdue');
      this._reessayer();
    });
  }

  _reessayer() {
    const delai = Math.min(5000, 400 * 2 ** this.tentatives++);
    setTimeout(() => this.ouvrir(), delai);
  }

  /** Renvoie vrai si le message est parti. */
  envoyer(message) {
    if (this.ws?.readyState !== WebSocket.OPEN) return false;
    this.ws.send(JSON.stringify(message));
    return true;
  }

  /** JPEG simulé : transport binaire borné, sans toucher aux commandes JSON. */
  envoyerImage(entete, jpeg) {
    if (this.ws?.readyState !== WebSocket.OPEN || this.ws.bufferedAmount > 800000) return false;
    this.ws.send(new Blob([JSON.stringify(entete) + '\n', jpeg]));
    return true;
  }

  /** Changement de jeton : on rouvre la liaison avec le nouveau. */
  definirJeton(jeton) {
    this.jeton = jeton;
    const ws = this.ws;
    this.ws = null;
    if (ws) ws.close();
    this.tentatives = 0;
    this.ouvrir();
  }

  /**
   * Flux MJPEG d'une caméra dans un <img>. `fermer()` coupe la connexion HTTP (l'image est
   * remplacée par un pixel vide), `relancer()` la rouvre (après une reconnexion).
   */
  flux(camera, { ips = 10, largeur = 640 } = {}) {
    const image = new Image();
    image.alt = '';
    image.decoding = 'async';
    image.draggable = false;
    let ouvert = false, reprise = null;
    const vide = 'data:image/gif;base64,R0lGODlhAQABAAAAACH5BAEKAAEALAAAAAABAAEAAAICTAEAOw==';
    const ouvrir = () => {
      clearTimeout(reprise);
      ouvert = true;
      const adresse = this._url(`flux/${encodeURIComponent(camera)}`, { ips, largeur }).href;
      image.src = vide; // force une nouvelle requête même si l'adresse n'a pas changé
      setTimeout(() => { if (ouvert) image.src = adresse; }, 0);
    };
    image.addEventListener('error', () => {
      if (!ouvert) return;
      clearTimeout(reprise);
      reprise = setTimeout(() => ouvert && ouvrir(), 2500);
    });
    return {
      element: image,
      ouvrir,
      fermer: () => {
        ouvert = false;
        clearTimeout(reprise);
        image.src = vide;
      },
      relancer: () => ouvert && ouvrir(),
      dimensions: () => (image.naturalWidth > 1 ? [image.naturalWidth, image.naturalHeight] : null),
    };
  }
}
