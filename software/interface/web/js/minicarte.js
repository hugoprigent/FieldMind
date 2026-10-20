// Mini-carte : ce que sait le cerveau. Grille d'occupation du lidar, obstacles de la caméra
// stéréo (marches et pentes), pose estimée, chemin, objectif et lieux nommés. Clic : objectif.

const INCONNU = 255;
// Traversabilité (caméra stéréo) : 2 marche (tronc, rocher, souche), 3 pente trop forte,
// 4 trou probable (zone cachée derrière un bord de sol).
const COULEURS_TRAVERSABILITE = { 2: [229, 72, 77], 3: [142, 107, 216], 4: [60, 190, 210] };

function variable(nom) {
  return getComputedStyle(document.documentElement).getPropertyValue(nom).trim();
}

async function decompresser(base64) {
  const octets = Uint8Array.from(atob(base64), (c) => c.charCodeAt(0));
  const flux = new Blob([octets]).stream().pipeThrough(new DecompressionStream('deflate'));
  return new Uint8Array(await new Response(flux).arrayBuffer());
}

export class MiniCarte {
  constructor(canvas, surObjectif) {
    this.canvas = canvas;
    this.surObjectif = surObjectif;
    this.scenario = null;
    this.grille = null;
    this.traversabilite = null;
    this.pose = null;
    this.chemin = [];
    this.objectifCourant = null;
    this.obstaclesVision = [];
    this.reperesIA = [];
    this.scan = null;
    this.cadrage = 'suivi';
    this.metresVisibles = 28;
    canvas.addEventListener('click', (e) => this._clic(e));
    canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.metresVisibles = Math.min(90, Math.max(8, this.metresVisibles * (e.deltaY > 0 ? 1.15 : 1 / 1.15)));
    }, { passive: false });
  }

  initialiser(bienvenue) {
    this.scenario = bienvenue.scenario;
    this.grille = null;
    this.traversabilite = null;
    this.chemin = [];
    this.objectifCourant = null;
  }

  definirCadrage(mode) {
    this.cadrage = mode;
  }

  async appliquerCarte(m) {
    if (!this.grille || this.grille.largeur !== m.largeur || this.grille.hauteur !== m.hauteur) {
      const image = document.createElement('canvas');
      image.width = m.largeur;
      image.height = m.hauteur;
      const ctx = image.getContext('2d');
      ctx.fillStyle = '#1c1c1b';
      ctx.fillRect(0, 0, m.largeur, m.hauteur);
      this.grille = { largeur: m.largeur, hauteur: m.hauteur, resolution: m.resolution, origine: m.origine, image, ctx };
    }
    const [c0, l0, w, h] = m.rect;
    if (w === 0 || h === 0) return;
    const valeurs = await decompresser(m.donnees);
    const pixels = this.grille.ctx.createImageData(w, h);
    // La ligne 0 de la grille est au sud ; l'image est retournée pour avoir le nord en haut.
    for (let l = 0; l < h; l++) {
      for (let c = 0; c < w; c++) {
        const v = valeurs[l * w + c];
        const k = ((h - 1 - l) * w + c) * 4;
        if (v === INCONNU) {
          pixels.data[k] = 28; pixels.data[k + 1] = 28; pixels.data[k + 2] = 27;
        } else {
          const p = v / 254;
          // Libre : vert sombre ; occupé : presque blanc.
          pixels.data[k] = 58 + p * (232 - 58);
          pixels.data[k + 1] = 74 + p * (230 - 74);
          pixels.data[k + 2] = 60 + p * (223 - 60);
        }
        pixels.data[k + 3] = 255;
      }
    }
    this.grille.ctx.putImageData(pixels, c0, this.grille.hauteur - l0 - h);
  }

  async appliquerTraversabilite(m) {
    if (!this.traversabilite || this.traversabilite.largeur !== m.largeur || this.traversabilite.hauteur !== m.hauteur) {
      const image = document.createElement('canvas');
      image.width = m.largeur;
      image.height = m.hauteur;
      this.traversabilite = {
        largeur: m.largeur, hauteur: m.hauteur, resolution: m.resolution, origine: m.origine,
        image, ctx: image.getContext('2d'),
      };
    }
    const [c0, l0, w, h] = m.rect;
    if (w === 0 || h === 0) return;
    const valeurs = await decompresser(m.donnees);
    const pixels = this.traversabilite.ctx.createImageData(w, h);
    for (let l = 0; l < h; l++) {
      for (let c = 0; c < w; c++) {
        const couleur = COULEURS_TRAVERSABILITE[valeurs[l * w + c]];
        if (!couleur) continue;
        const k = ((h - 1 - l) * w + c) * 4;
        pixels.data[k] = couleur[0]; pixels.data[k + 1] = couleur[1]; pixels.data[k + 2] = couleur[2];
        pixels.data[k + 3] = 255;
      }
    }
    this.traversabilite.ctx.putImageData(pixels, c0, this.traversabilite.hauteur - l0 - h);
  }

  _transformation() {
    const r = this.canvas.getBoundingClientRect();
    const lim = this.scenario.limites;
    let cx, cy, metres;
    if (this.cadrage === 'suivi' && this.pose) {
      cx = this.pose.x; cy = this.pose.y; metres = this.metresVisibles;
    } else {
      cx = (lim.xmin + lim.xmax) / 2; cy = (lim.ymin + lim.ymax) / 2;
      metres = Math.max(lim.xmax - lim.xmin, lim.ymax - lim.ymin) + 2;
    }
    const echelle = Math.min(r.width, r.height) / metres;
    return {
      r, echelle,
      versEcran: (x, y) => [r.width / 2 + (x - cx) * echelle, r.height / 2 - (y - cy) * echelle],
      versSol: (px, py) => [cx + (px - r.width / 2) / echelle, cy - (py - r.height / 2) / echelle],
    };
  }

  _clic(e) {
    if (!this.scenario) return;
    const { r, versSol } = this._transformation();
    const [x, y] = versSol(e.clientX - r.left, e.clientY - r.top);
    const lim = this.scenario.limites;
    if (x < lim.xmin || x > lim.xmax || y < lim.ymin || y > lim.ymax) return;
    this.surObjectif(x, y);
  }

  dessiner() {
    if (!this.scenario) return;
    const { r, echelle, versEcran } = this._transformation();
    const dpr = window.devicePixelRatio || 1;
    const L = Math.round(r.width * dpr), H = Math.round(r.height * dpr);
    if (this.canvas.width !== L || this.canvas.height !== H) { this.canvas.width = L; this.canvas.height = H; }
    const ctx = this.canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#111110';
    ctx.fillRect(0, 0, r.width, r.height);

    const lim = this.scenario.limites;
    const [x0, y0] = versEcran(lim.xmin, lim.ymax);
    const [x1, y1] = versEcran(lim.xmax, lim.ymin);
    ctx.fillStyle = '#1c1c1b';
    ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
    for (const g of [this.grille, this.traversabilite]) {
      if (!g) continue;
      const [gx0, gy0] = versEcran(g.origine[0], g.origine[1] + g.hauteur * g.resolution);
      ctx.imageSmoothingEnabled = echelle * g.resolution < 2;
      ctx.drawImage(g.image, gx0, gy0, g.largeur * g.resolution * echelle, g.hauteur * g.resolution * echelle);
    }
    ctx.strokeStyle = variable('--etat-alerte');
    ctx.lineWidth = 1;
    ctx.strokeRect(x0 + 0.5, y0 + 0.5, x1 - x0, y1 - y0);

    // Obstacles bas vus par la caméra (couche vision du cerveau) : emprise au sol.
    for (const o of this.obstaclesVision) {
      ctx.fillStyle = variable('--serie-4');
      ctx.strokeStyle = variable('--serie-4');
      ctx.lineJoin = 'round';
      ctx.lineWidth = Math.max(2, 2 * o.marge * echelle);
      ctx.beginPath();
      o.contour.forEach(([x, y], i) => { const [px, py] = versEcran(x, y); if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py); });
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    }
    // Chemin prévu.
    if (this.chemin.length > 1) {
      ctx.strokeStyle = variable('--serie-1');
      ctx.lineWidth = 2;
      ctx.beginPath();
      this.chemin.forEach(([x, y], i) => { const [px, py] = versEcran(x, y); if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py); });
      ctx.stroke();
    }
    // Dernier balayage (points).
    if (this.scan && this.pose) {
      ctx.fillStyle = variable('--serie-2');
      for (const [x, y] of this.scan) {
        const [px, py] = versEcran(x, y);
        ctx.fillRect(px - 1, py - 1, 2, 2);
      }
    }
    // Éléments notables repérés par l'IA d'observation.
    ctx.font = '11px system-ui, sans-serif';
    for (const r of this.reperesIA) {
      const [px, py] = versEcran(r.x, r.y);
      ctx.fillStyle = '#e5484d';
      ctx.beginPath(); ctx.moveTo(px, py - 6); ctx.lineTo(px + 5, py); ctx.lineTo(px, py + 6); ctx.lineTo(px - 5, py); ctx.closePath(); ctx.fill();
      ctx.fillStyle = variable('--texte-1');
      ctx.fillText(r.nom, px + 7, py + 4);
    }
    // Lieux nommés.
    ctx.font = '11px system-ui, sans-serif';
    for (const lieu of this.scenario.lieux) {
      const [px, py] = versEcran(lieu.x, lieu.y);
      ctx.fillStyle = variable('--etat-alerte');
      ctx.beginPath(); ctx.arc(px, py, 3, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = variable('--texte-2');
      ctx.fillText(lieu.nom, px + 5, py - 4);
    }
    // Objectif.
    if (this.objectifCourant) {
      const [px, py] = versEcran(...this.objectifCourant);
      ctx.strokeStyle = variable('--serie-1');
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(px, py, 7, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(px - 11, py); ctx.lineTo(px + 11, py); ctx.moveTo(px, py - 11); ctx.lineTo(px, py + 11); ctx.stroke();
    }
    // Rover (pose estimée), avec son incertitude.
    if (this.pose) {
      const [px, py] = versEcran(this.pose.x, this.pose.y);
      const sigma = (this.pose.sigma_xy ?? 0) * echelle * 2;
      if (sigma > 2) {
        ctx.fillStyle = 'rgba(57,135,229,0.18)';
        ctx.beginPath(); ctx.arc(px, py, sigma, 0, Math.PI * 2); ctx.fill();
      }
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(-this.pose.cap);
      const l = Math.max(0.7 * echelle, 9), w = Math.max(0.53 * echelle, 7);
      ctx.fillStyle = '#ffffff';
      ctx.beginPath();
      ctx.moveTo(l / 2, 0); ctx.lineTo(-l / 2, w / 2); ctx.lineTo(-l / 3, 0); ctx.lineTo(-l / 2, -w / 2);
      ctx.closePath(); ctx.fill();
      ctx.restore();
    }
    // Échelle.
    const metres = echelle > 20 ? 1 : echelle > 6 ? 5 : 10;
    ctx.fillStyle = variable('--texte-2');
    ctx.fillRect(10, r.height - 12, metres * echelle, 2);
    ctx.fillText(`${metres} m`, 10, r.height - 16);
  }
}
