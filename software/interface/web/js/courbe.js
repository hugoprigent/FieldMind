// Courbe temporelle en direct (Canvas 2D) : un seul axe vertical, séries en couleurs fixes,
// légende à clés de trait, réticule et infobulle au survol listant toutes les séries.

const COULEURS = ['--serie-1', '--serie-2', '--serie-3', '--serie-4'];

function lireVariable(nom) {
  return getComputedStyle(document.documentElement).getPropertyValue(nom).trim();
}

/** Pas de graduation « rond » (1, 2 ou 5 × 10^n) pour environ `n` intervalles. */
function pasRond(etendue, n = 4) {
  const brut = etendue / n;
  const puissance = 10 ** Math.floor(Math.log10(brut));
  const f = brut / puissance;
  return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * puissance;
}

function formater(valeur, decimales) {
  if (valeur === null || valeur === undefined || Number.isNaN(valeur)) return '—';
  return valeur.toLocaleString('fr-FR', { minimumFractionDigits: decimales, maximumFractionDigits: decimales });
}

export class CourbeTemps {
  /**
   * @param {HTMLElement} tuile  conteneur .tuile
   * @param {{titre: string, source?: string, unite: string, series: string[], fenetre?: number,
   *          min?: number, max?: number, decimales?: number}} options
   */
  constructor(tuile, options) {
    this.options = { fenetre: 20, decimales: 2, ...options };
    this.couleurs = options.series.map((_, i) => lireVariable(COULEURS[i]));
    this.donnees = options.series.map(() => []);
    this.tSurvol = null;
    this.tMax = 0;

    const titre = document.createElement('h3');
    const nom = document.createElement('span');
    nom.textContent = `${options.titre} (${options.unite})`;
    titre.append(nom);
    if (options.source) {
      const source = document.createElement('span');
      source.className = 'source';
      source.textContent = options.source;
      titre.append(source);
    }
    const legende = document.createElement('div');
    legende.className = 'legende';
    if (options.series.length > 1) {
      options.series.forEach((serie, i) => {
        const entree = document.createElement('span');
        const cle = document.createElement('i');
        cle.style.background = this.couleurs[i];
        entree.append(cle, document.createTextNode(serie));
        legende.append(entree);
      });
    }
    const zone = document.createElement('div');
    zone.className = 'zone-courbe';
    this.canvas = document.createElement('canvas');
    this.canvas.setAttribute('role', 'img');
    this.canvas.setAttribute('aria-label', `Courbe : ${options.titre}`);
    this.infobulle = document.createElement('div');
    this.infobulle.className = 'infobulle';
    zone.append(this.canvas, this.infobulle);
    tuile.append(titre, legende, zone);

    zone.addEventListener('pointermove', (e) => this._survoler(e));
    zone.addEventListener('pointerleave', () => { this.tSurvol = null; this.infobulle.style.display = 'none'; });
  }

  ajouter(t, valeurs) {
    this.tMax = Math.max(this.tMax, t);
    const limite = t - this.options.fenetre - 1;
    valeurs.forEach((v, i) => {
      const serie = this.donnees[i];
      serie.push([t, v]);
      while (serie.length && serie[0][0] < limite) serie.shift();
    });
  }

  vider() {
    this.donnees.forEach((s) => { s.length = 0; });
  }

  _echelles(largeur, hauteur) {
    const marge = { g: 36, d: 6, h: 6, b: 16 };
    const t1 = this.tMax, t0 = t1 - this.options.fenetre;
    let min = this.options.min ?? Infinity, max = this.options.max ?? -Infinity;
    if (this.options.min === undefined || this.options.max === undefined) {
      for (const serie of this.donnees) {
        for (const [, v] of serie) {
          if (v === null) continue;
          if (this.options.min === undefined) min = Math.min(min, v);
          if (this.options.max === undefined) max = Math.max(max, v);
        }
      }
    }
    if (!Number.isFinite(min) || !Number.isFinite(max)) { min = 0; max = 1; }
    if (max - min < 1e-6) { max += 0.5; min -= 0.5; }
    const pad = (max - min) * 0.08;
    if (this.options.min === undefined) min -= pad;
    if (this.options.max === undefined) max += pad;
    const x = (t) => marge.g + ((t - t0) / (t1 - t0)) * (largeur - marge.g - marge.d);
    const y = (v) => marge.h + (1 - (v - min) / (max - min)) * (hauteur - marge.h - marge.b);
    return { x, y, min, max, t0, t1, marge };
  }

  _survoler(e) {
    const zone = this.canvas.getBoundingClientRect();
    const { x, t0, t1, marge } = this._echelles(zone.width, zone.height);
    const px = e.clientX - zone.left;
    if (px < marge.g) { this.tSurvol = null; this.infobulle.style.display = 'none'; return; }
    this.tSurvol = t0 + ((px - marge.g) / (zone.width - marge.g - marge.d)) * (t1 - t0);
    this.pxSurvol = x(this.tSurvol);
  }

  _valeurA(serie, t) {
    // Point le plus proche en temps (recherche dichotomique).
    let bas = 0, haut = serie.length - 1;
    if (haut < 0) return null;
    while (haut - bas > 1) {
      const milieu = (bas + haut) >> 1;
      if (serie[milieu][0] < t) bas = milieu; else haut = milieu;
    }
    const meilleur = Math.abs(serie[bas][0] - t) < Math.abs(serie[haut][0] - t) ? serie[bas] : serie[haut];
    return meilleur;
  }

  dessiner() {
    const rect = this.canvas.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return;
    const dpr = window.devicePixelRatio || 1;
    const L = Math.round(rect.width * dpr), H = Math.round(rect.height * dpr);
    if (this.canvas.width !== L || this.canvas.height !== H) { this.canvas.width = L; this.canvas.height = H; }
    const ctx = this.canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, rect.width, rect.height);
    const { x, y, min, max, t0, t1, marge } = this._echelles(rect.width, rect.height);

    // Grille horizontale en traits fins, graduations sur l'unique axe.
    ctx.font = '10px system-ui, sans-serif';
    ctx.fillStyle = lireVariable('--texte-3');
    ctx.strokeStyle = lireVariable('--grille');
    ctx.lineWidth = 1;
    const pas = pasRond(max - min, rect.height > 140 ? 4 : 3);
    const decimalesAxe = Math.max(0, -Math.floor(Math.log10(pas) + 1e-9));
    for (let v = Math.ceil(min / pas) * pas; v <= max + 1e-9; v += pas) {
      const py = Math.round(y(v)) + 0.5;
      ctx.beginPath(); ctx.moveTo(marge.g, py); ctx.lineTo(rect.width - marge.d, py); ctx.stroke();
      ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
      ctx.fillText(formater(Math.abs(v) < pas / 1e6 ? 0 : v, decimalesAxe), marge.g - 4, py);
    }
    if (min < 0 && max > 0) {
      ctx.strokeStyle = lireVariable('--axe');
      const py = Math.round(y(0)) + 0.5;
      ctx.beginPath(); ctx.moveTo(marge.g, py); ctx.lineTo(rect.width - marge.d, py); ctx.stroke();
    }
    ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillText(`−${this.options.fenetre} s`, marge.g, rect.height - marge.b + 3);
    ctx.textAlign = 'right';
    ctx.fillText('maintenant', rect.width - marge.d, rect.height - marge.b + 3);

    // Séries : traits de 2 px, coupés aux valeurs absentes.
    ctx.save();
    ctx.beginPath();
    ctx.rect(marge.g, 0, rect.width - marge.g - marge.d, rect.height);
    ctx.clip();
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    this.donnees.forEach((serie, i) => {
      ctx.strokeStyle = this.couleurs[i];
      ctx.beginPath();
      let enCours = false;
      for (const [t, v] of serie) {
        if (t < t0 - 1) continue;
        if (v === null) { enCours = false; continue; }
        const px = x(t), py = y(Math.min(max, Math.max(min, v)));
        if (enCours) ctx.lineTo(px, py); else ctx.moveTo(px, py);
        enCours = true;
      }
      ctx.stroke();
    });
    ctx.restore();

    // Réticule et infobulle.
    if (this.tSurvol !== null && this.tSurvol <= t1) {
      const px = Math.round(x(this.tSurvol)) + 0.5;
      ctx.strokeStyle = lireVariable('--texte-3');
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(px, marge.h); ctx.lineTo(px, rect.height - marge.b); ctx.stroke();
      this._remplirInfobulle(px, rect.width);
    }
  }

  _remplirInfobulle(px, largeur) {
    const bulle = this.infobulle;
    bulle.replaceChildren();
    let tPoint = null;
    this.donnees.forEach((serie, i) => {
      const point = this._valeurA(serie, this.tSurvol);
      if (!point) return;
      tPoint = point[0];
      const ligne = document.createElement('div');
      ligne.className = 'ligne';
      const cle = document.createElement('i');
      cle.style.background = this.couleurs[i];
      const valeur = document.createElement('strong');
      valeur.textContent = `${formater(point[1], this.options.decimales)} ${this.options.unite}`;
      ligne.append(cle, valeur, document.createTextNode(` ${this.options.series[i]}`));
      bulle.append(ligne);
    });
    if (tPoint === null) { bulle.style.display = 'none'; return; }
    const temps = document.createElement('div');
    temps.className = 'temps';
    temps.textContent = `il y a ${formater(this.tMax - tPoint, 1)} s`;
    bulle.prepend(temps);
    bulle.style.display = 'block';
    const gauche = px + 10 + bulle.offsetWidth > largeur ? px - 10 - bulle.offsetWidth : px + 10;
    bulle.style.left = `${gauche}px`;
    bulle.style.top = '4px';
  }
}

/** Tuile de chiffres clés (valeurs mesurées). */
export class Chiffres {
  constructor(tuile, titre, source, champs) {
    const h = document.createElement('h3');
    const nom = document.createElement('span');
    nom.textContent = titre;
    h.append(nom);
    if (source) {
      const s = document.createElement('span');
      s.className = 'source';
      s.textContent = source;
      h.append(s);
    }
    const grille = document.createElement('div');
    grille.className = 'chiffres';
    tuile.classList.add('tuile-chiffres');
    this.valeurs = {};
    for (const [cle, libelle, unite] of champs) {
      const bloc = document.createElement('div');
      bloc.className = 'chiffre';
      const l = document.createElement('span');
      l.className = 'libelle';
      l.textContent = libelle;
      const n = document.createElement('span');
      n.className = 'nombre';
      const v = document.createElement('span');
      v.textContent = '—';
      n.append(v);
      if (unite) {
        const u = document.createElement('small');
        u.textContent = unite;
        n.append(u);
      }
      bloc.append(l, n);
      grille.append(bloc);
      this.valeurs[cle] = v;
    }
    tuile.append(h, grille);
  }

  definir(cle, texte) {
    const element = this.valeurs[cle];
    if (element && element.textContent !== texte) element.textContent = texte;
  }
}

export { formater };
