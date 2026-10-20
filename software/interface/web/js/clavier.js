// Conduite manuelle au clavier (ZQSD, WASD ou flèches). Les consignes passent par le cerveau,
// puis par le contrôleur embarqué qui garde ses protections (arrêt d'urgence, rampes).

const AVANT = new Set(['KeyW', 'ArrowUp']);
const ARRIERE = new Set(['KeyS', 'ArrowDown']);
const GAUCHE = new Set(['KeyA', 'ArrowLeft']);
const DROITE = new Set(['KeyD', 'ArrowRight']);
// Clavier AZERTY : Z et Q ont les codes physiques W et A ; on accepte aussi les touches affichées.
const LETTRES = { z: AVANT, s: ARRIERE, q: GAUCHE, d: DROITE, w: AVANT, a: GAUCHE };

export class Clavier {
  constructor(envoyer, surArret) {
    this.envoyer = envoyer;
    this.surArret = surArret;
    this.appuyees = new Set();
    this.actif = false;
    window.addEventListener('keydown', (e) => this._touche(e, true));
    window.addEventListener('keyup', (e) => this._touche(e, false));
    window.addEventListener('blur', () => this.appuyees.clear());
    setInterval(() => this._emettre(), 50);
  }

  _touche(e, bas) {
    if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) return;
    if (e.code === 'Space' && bas) {
      e.preventDefault();
      this.appuyees.clear();
      this.surArret();
      return;
    }
    const groupe = LETTRES[e.key?.toLowerCase()];
    const code = [AVANT, ARRIERE, GAUCHE, DROITE].find((g) => g.has(e.code)) ?? groupe;
    if (!code) return;
    e.preventDefault();
    const nom = code === AVANT ? 'avant' : code === ARRIERE ? 'arriere' : code === GAUCHE ? 'gauche' : 'droite';
    if (bas) this.appuyees.add(nom); else this.appuyees.delete(nom);
  }

  _emettre() {
    const avance = (this.appuyees.has('avant') ? 1 : 0) - (this.appuyees.has('arriere') ? 1 : 0);
    const direction = (this.appuyees.has('gauche') ? 1 : 0) - (this.appuyees.has('droite') ? 1 : 0);
    if (avance || direction) {
      this.actif = true;
      this.envoyer({ type: 'teleop', avance: avance * 0.7, direction });
    } else if (this.actif) {
      this.actif = false;
      this.envoyer({ type: 'teleop', avance: 0, direction: 0 });
    }
  }
}
