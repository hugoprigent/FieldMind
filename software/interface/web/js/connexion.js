// Connexion WebSocket au serveur : une session (un rover simulé et son cerveau) par onglet.

export class Connexion {
  /** @param {(message: any) => void} surMessage @param {(etat: string) => void} surEtat */
  constructor(surMessage, surEtat) {
    this.surMessage = surMessage;
    this.surEtat = surEtat;
    this.ws = null;
    this.tentatives = 0;
    this.fermeeDefinitivement = false;
  }

  ouvrir() {
    // Adresse relative à la page : la démo peut être servie sous un préfixe (/fieldmind/).
    const url = new URL('ws', new URL('../', import.meta.url));
    url.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const parametres = new URLSearchParams(location.search);
    const requete = new URLSearchParams();
    const graine = parametres.get('graine');
    if (graine && /^\d{1,9}$/.test(graine)) requete.set('graine', graine);
    if (parametres.get('robot') === 'externe') requete.set('robot', 'externe');
    url.search = requete.toString();
    const ws = new WebSocket(url);
    this.ws = ws;
    this.surEtat('connexion');
    ws.addEventListener('open', () => {
      this.tentatives = 0;
      this.surEtat('ok');
    });
    ws.addEventListener('message', (evenement) => {
      let message;
      try {
        message = JSON.parse(evenement.data);
      } catch {
        return;
      }
      if (message.type === 'fin_session') this.fermeeDefinitivement = true;
      this.surMessage(message);
    });
    ws.addEventListener('close', () => {
      if (this.fermeeDefinitivement) {
        this.surEtat('fermee');
        return;
      }
      this.surEtat('perdue');
      const delai = Math.min(10000, 500 * 2 ** this.tentatives++);
      setTimeout(() => this.ouvrir(), delai);
    });
  }

  envoyer(message) {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(message));
  }
}
