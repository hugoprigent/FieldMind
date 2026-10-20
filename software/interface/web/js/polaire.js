// Vue polaire du dernier balayage lidar, dans le repère du rover (avant vers le haut).
// Montre aussi l'emprise du rover et la zone d'arrêt d'urgence du contrôleur embarqué.

function variable(nom) {
  return getComputedStyle(document.documentElement).getPropertyValue(nom).trim();
}

export class VuePolaire {
  constructor(tuile, rover) {
    this.rover = rover;
    this.scan = null;
    this.portee = 8;
    const titre = document.createElement('h3');
    const nom = document.createElement('span');
    nom.textContent = `Lidar ${rover.capteurs.lidar.modele}`;
    this.info = document.createElement('span');
    this.info.className = 'source';
    titre.append(nom, this.info);
    const zone = document.createElement('div');
    zone.className = 'zone-courbe';
    this.canvas = document.createElement('canvas');
    this.canvas.setAttribute('role', 'img');
    this.canvas.setAttribute('aria-label', 'Dernier balayage lidar, vue de dessus');
    this.infobulle = document.createElement('div');
    this.infobulle.className = 'infobulle';
    zone.append(this.canvas, this.infobulle);
    tuile.append(titre, zone);
    zone.addEventListener('pointermove', (e) => this._survoler(e));
    zone.addEventListener('pointerleave', () => { this.survol = null; this.infobulle.style.display = 'none'; });
  }

  mettreAJour(scan) {
    this.scan = scan;
    const valides = scan.distances.filter((d) => d !== null).length;
    this.info.textContent = `${valides} / ${scan.distances.length} échos`;
  }

  _geometrie() {
    const r = this.canvas.getBoundingClientRect();
    const cx = r.width / 2, cy = r.height / 2;
    const echelle = (Math.min(r.width, r.height) / 2 - 6) / this.portee;
    return { r, cx, cy, echelle };
  }

  _survoler(e) {
    if (!this.scan) return;
    const { r, cx, cy, echelle } = this._geometrie();
    const px = e.clientX - r.left, py = e.clientY - r.top;
    // Repère du capteur : x vers l'avant (haut de l'écran), y vers la gauche.
    const xs = (cy - py) / echelle, ys = (cx - px) / echelle;
    let meilleur = null, dmin = 0.6;
    this.scan.distances.forEach((d, i) => {
      if (d === null) return;
      const a = this.scan.angle_min + i * this.scan.pas;
      const ecart = Math.hypot(d * Math.cos(a) - xs, d * Math.sin(a) - ys);
      if (ecart < dmin) { dmin = ecart; meilleur = { d, a }; }
    });
    this.survol = meilleur;
    if (!meilleur) { this.infobulle.style.display = 'none'; return; }
    const deg = Math.round((meilleur.a * 180) / Math.PI);
    this.infobulle.replaceChildren();
    const valeur = document.createElement('strong');
    valeur.textContent = `${meilleur.d.toLocaleString('fr-FR', { maximumFractionDigits: 2 })} m`;
    this.infobulle.append(valeur, document.createTextNode(` à ${deg}°`));
    this.infobulle.style.display = 'block';
    this.infobulle.style.left = `${Math.min(px + 10, r.width - 110)}px`;
    this.infobulle.style.top = `${Math.max(py - 28, 0)}px`;
  }

  dessiner() {
    const { r, cx, cy, echelle } = this._geometrie();
    if (r.width === 0) return;
    const dpr = window.devicePixelRatio || 1;
    const L = Math.round(r.width * dpr), H = Math.round(r.height * dpr);
    if (this.canvas.width !== L || this.canvas.height !== H) { this.canvas.width = L; this.canvas.height = H; }
    const ctx = this.canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, r.width, r.height);
    const versEcran = (x, y) => [cx - y * echelle, cy - x * echelle];

    // Cercles de distance.
    ctx.strokeStyle = variable('--grille');
    ctx.fillStyle = variable('--texte-3');
    ctx.font = '10px system-ui, sans-serif';
    ctx.lineWidth = 1;
    for (const rayon of [1, 2, 4, 8]) {
      ctx.beginPath(); ctx.arc(cx, cy, rayon * echelle, 0, Math.PI * 2); ctx.stroke();
      ctx.fillText(`${rayon} m`, cx + 3, cy - rayon * echelle + 11);
    }

    const g = this.rover.geometrie, s = this.rover.securite;
    const lx = this.rover.capteurs.lidar.position[0];
    // Emprise du rover (repère du capteur : centre du rover à -lx).
    const coins = [[g.longueur / 2, g.largeur / 2], [g.longueur / 2, -g.largeur / 2],
      [-g.longueur / 2, -g.largeur / 2], [-g.longueur / 2, g.largeur / 2]]
      .map(([x, y]) => versEcran(x - lx, y));
    ctx.fillStyle = 'rgba(57,135,229,0.35)';
    ctx.beginPath(); coins.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y))); ctx.closePath(); ctx.fill();
    // Zone d'arrêt d'urgence devant le pare-chocs.
    const avant = g.longueur / 2 - lx;
    const demi = g.largeur / 2 + 0.05;
    const zone = [[avant, demi], [avant + s.distance_arret_avant, demi],
      [avant + s.distance_arret_avant, -demi], [avant, -demi]].map(([x, y]) => versEcran(x, y));
    ctx.strokeStyle = variable('--etat-alerte');
    ctx.beginPath(); zone.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y))); ctx.closePath(); ctx.stroke();

    if (!this.scan) return;
    ctx.fillStyle = variable('--serie-2');
    const { angle_min: a0, pas } = this.scan;
    this.scan.distances.forEach((d, i) => {
      if (d === null) return;
      const a = a0 + i * pas;
      const [x, y] = versEcran(d * Math.cos(a), d * Math.sin(a));
      ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
    });
    if (this.survol) {
      const [x, y] = versEcran(this.survol.d * Math.cos(this.survol.a), this.survol.d * Math.sin(this.survol.a));
      ctx.strokeStyle = variable('--texte-1');
      ctx.beginPath(); ctx.arc(x, y, 5, 0, Math.PI * 2); ctx.stroke();
    }
  }
}
