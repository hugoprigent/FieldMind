// Téléopération : joystick, clavier et homme mort. Logique pure : l'appelant fournit la
// fonction d'envoi et appelle `pas(dt)` à 15 Hz. Valeurs normalisées −1..1 (PROTOCOLE.md).

export const FREQUENCE_HZ = 15;

// Par caractère (`KeyboardEvent.key`) : ZQSD en AZERTY, WASD en QWERTY, et les flèches.
const TOUCHES = {
  z: 'avant', w: 'avant', arrowup: 'avant',
  s: 'arriere', arrowdown: 'arriere',
  q: 'gauche', a: 'gauche', arrowleft: 'gauche',
  d: 'droite', arrowright: 'droite',
};

/** `KeyboardEvent.key` → direction ('avant', 'arriere', 'gauche', 'droite') ou null. */
export function directionTouche(touche) {
  return TOUCHES[String(touche || '').toLowerCase()] || null;
}

function arrondir(v) {
  return Math.round(Math.min(1, Math.max(-1, v)) * 1000) / 1000 || 0;
}

function zoneMorte(v, seuil) {
  const a = Math.abs(v);
  if (a <= seuil) return 0;
  return (Math.sign(v) * (a - seuil)) / (1 - seuil);
}

export class Teleoperation {
  /** @param {(m: {type: string, vitesse: number, braquage: number}) => void} envoyer */
  constructor(envoyer, { rampeBraquage = 2.5, seuil = 0.06 } = {}) {
    this.envoyer = envoyer;
    this.rampe = rampeBraquage; // fraction du braquage maximal par seconde (clavier)
    this.seuil = seuil;
    this.facteur = 1; // vitesse maximale choisie (fraction du maximum du mode)
    this.autorise = true; // faux hors mode manuel : rien n'est envoyé
    this.touches = new Set();
    this.joy = null;
    this.braquageClavier = 0;
    this.actif = false;
    this.derniere = { vitesse: 0, braquage: 0 };
  }

  /** Commande courante (sans effet de bord). */
  commande() {
    if (this.joy) {
      return { vitesse: arrondir(this.joy.vitesse * this.facteur), braquage: arrondir(this.joy.braquage) };
    }
    const avant = this.touches.has('avant') ? 1 : 0, arriere = this.touches.has('arriere') ? 1 : 0;
    return { vitesse: arrondir((avant - arriere) * this.facteur), braquage: arrondir(this.braquageClavier) };
  }

  get sollicite() {
    return this.joy !== null || this.touches.size > 0;
  }

  /** Joystick : x vers la droite, y vers le haut, dans le disque unité. */
  joystick(x, y) {
    this.joy = { vitesse: zoneMorte(y, this.seuil), braquage: -zoneMorte(x, this.seuil) };
    this._actualiser(true);
  }

  relacherJoystick() {
    this.joy = null;
    this._actualiser(false);
  }

  appuyer(direction) {
    if (this.touches.has(direction)) return;
    this.touches.add(direction);
    if ((direction === 'gauche' || direction === 'droite') && Math.abs(this.braquageClavier) < 0.25) {
      this.braquageClavier = direction === 'gauche' ? 0.25 : -0.25; // réponse immédiate
    }
    this._actualiser(true);
  }

  relacher(direction) {
    if (!this.touches.delete(direction)) return;
    this._actualiser(false);
  }

  definirFacteur(f) {
    this.facteur = Math.min(1, Math.max(0.05, f));
  }

  definirAutorisation(autorise) {
    if (this.autorise === autorise) return;
    this.autorise = autorise;
    if (!autorise) this.arreter();
  }

  /** À appeler à 15 Hz : rampe du braquage clavier, puis envoi tant qu'une commande est active. */
  pas(dt) {
    const cible = (this.touches.has('gauche') ? 1 : 0) - (this.touches.has('droite') ? 1 : 0);
    const vitesseRampe = cible === 0 ? this.rampe * 2 : this.rampe;
    const ecart = cible - this.braquageClavier;
    const pasMax = vitesseRampe * dt;
    this.braquageClavier = Math.abs(ecart) <= pasMax ? cible : this.braquageClavier + Math.sign(ecart) * pasMax;
    if (this.actif) this._envoyerCommande();
  }

  /** Tout relâcher (perte du focus, de la liaison, changement de mode) et envoyer un 0, 0. */
  arreter() {
    const etaitActif = this.actif;
    this.touches.clear();
    this.joy = null;
    this.braquageClavier = 0;
    this.actif = false;
    if (etaitActif) this._envoyer(0, 0);
  }

  _actualiser(immediat) {
    if (!this.autorise) return;
    if (this.sollicite) {
      this.actif = true;
      if (immediat) this._envoyerCommande();
    } else if (this.actif) {
      this.actif = false;
      this.braquageClavier = 0;
      this._envoyer(0, 0);
    }
  }

  _envoyerCommande() {
    const { vitesse, braquage } = this.commande();
    this._envoyer(vitesse, braquage);
  }

  _envoyer(vitesse, braquage) {
    this.derniere = { vitesse, braquage };
    this.envoyer({ type: 'pilotage', vitesse, braquage });
  }
}
