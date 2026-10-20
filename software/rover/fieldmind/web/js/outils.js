// Petits outils du poste : DOM, nombres à la française, couleurs du thème, icônes,
// notifications et confirmation légère.

export const $ = (selecteur, racine = document) => racine.querySelector(selecteur);
export const $$ = (selecteur, racine = document) => [...racine.querySelectorAll(selecteur)];

export function el(balise, attributs = {}, ...enfants) {
  const e = document.createElement(balise);
  for (const [cle, valeur] of Object.entries(attributs || {})) {
    if (valeur === null || valeur === undefined || valeur === false) continue;
    if (cle === 'class') e.className = valeur;
    else if (cle === 'text') e.textContent = valeur;
    else if (cle === 'dataset') Object.assign(e.dataset, valeur);
    else if (cle.startsWith('on') && typeof valeur === 'function') e.addEventListener(cle.slice(2), valeur);
    else e.setAttribute(cle, valeur === true ? '' : valeur);
  }
  for (const enfant of enfants.flat()) {
    if (enfant === null || enfant === undefined || enfant === false) continue;
    e.append(enfant);
  }
  return e;
}

/** Ne touche au DOM que si le texte change. */
export function texte(element, valeur) {
  if (element && element.textContent !== valeur) element.textContent = valeur;
}

export function attribut(element, nom, valeur) {
  if (!element) return;
  const v = valeur === null || valeur === undefined ? null : String(valeur);
  if (element.getAttribute(nom) !== v) {
    if (v === null) element.removeAttribute(nom);
    else element.setAttribute(nom, v);
  }
}

// ------------------------------------------------------------------ nombres

const formats = new Map();

export function estNombre(v) {
  return typeof v === 'number' && Number.isFinite(v);
}

/** Nombre à la française (virgule), « — » si absent. Signe moins typographique. */
export function nombre(v, decimales = 1) {
  if (!estNombre(v)) return '—';
  let f = formats.get(decimales);
  if (!f) {
    f = new Intl.NumberFormat('fr-FR', { minimumFractionDigits: decimales, maximumFractionDigits: decimales });
    formats.set(decimales, f);
  }
  const s = f.format(v);
  return s === '-0' || /^-0[,]0+$/.test(s) ? s.slice(1) : s.replace('-', '−');
}

export function unite(v, decimales, u) {
  return estNombre(v) ? `${nombre(v, decimales)} ${u}` : '—';
}

export function heure(tMurale) {
  if (!estNombre(tMurale)) return '';
  return new Date(tMurale * 1000).toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
}

export function duree(secondes) {
  if (!estNombre(secondes)) return '—';
  const j = Math.floor(secondes / 86400), h = Math.floor((secondes % 86400) / 3600), m = Math.floor((secondes % 3600) / 60);
  if (j) return `${j} j ${h} h`;
  if (h) return `${h} h ${String(m).padStart(2, '0')}`;
  return `${m} min ${String(Math.floor(secondes % 60)).padStart(2, '0')} s`;
}

export function age(secondes) {
  if (!estNombre(secondes)) return '—';
  if (secondes < 1) return `${Math.round(secondes * 1000)} ms`;
  if (secondes < 120) return `${nombre(secondes, 1)} s`;
  return duree(secondes);
}

// ------------------------------------------------------------------ thème

const couleurs = new Map();

export function couleur(nom) {
  let c = couleurs.get(nom);
  if (c === undefined) {
    c = getComputedStyle(document.documentElement).getPropertyValue(nom).trim() || '#888';
    couleurs.set(nom, c);
  }
  return c;
}

export function oublierCouleurs() {
  couleurs.clear();
}

/** Canevas net sur écran haute densité : renvoie le contexte et la taille CSS, ou null si caché. */
export function preparerCanevas(canvas) {
  const r = canvas.getBoundingClientRect();
  if (r.width < 2 || r.height < 2) return null;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const L = Math.round(r.width * dpr), H = Math.round(r.height * dpr);
  if (canvas.width !== L || canvas.height !== H) {
    canvas.width = L;
    canvas.height = H;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, r.width, r.height);
  return { ctx, l: r.width, h: r.height, rect: r };
}

// ------------------------------------------------------------------ icônes (SVG en ligne)

const TRACES = {
  pilotage: '<circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="2.5"/><path d="M12 14.5V21M9.6 11.3 3.4 9.6M14.4 11.3l6.2-1.7"/>',
  mission: '<path d="M4 5h16v11H9l-5 4z"/><path d="M8 9.5h8M8 12.5h5"/>',
  carte: '<path d="M3 6.5 9 4l6 2.5L21 4v13.5L15 20l-6-2.5L3 20z"/><path d="M9 4v13.5M15 6.5V20"/>',
  capteurs: '<path d="M12 12 19 5"/><circle cx="12" cy="12" r="9"/><path d="M12 7a5 5 0 1 0 5 5"/><circle cx="12" cy="12" r="1.2" fill="currentColor"/>',
  systeme: '<rect x="6" y="6" width="12" height="12" rx="1.5"/><rect x="9.5" y="9.5" width="5" height="5"/><path d="M9 2.5v3M15 2.5v3M9 18.5v3M15 18.5v3M2.5 9h3M2.5 15h3M18.5 9h3M18.5 15h3"/>',
  reglages: '<path d="M4 6h10M18 6h2M4 12h4M12 12h8M4 18h12M20 18h0"/><circle cx="16" cy="6" r="2"/><circle cx="10" cy="12" r="2"/><circle cx="18" cy="18" r="2"/>',
  liaison: '<path d="M10 14a4 4 0 0 0 5.7 0l3-3a4 4 0 0 0-5.7-5.7l-1 1"/><path d="M14 10a4 4 0 0 0-5.7 0l-3 3a4 4 0 0 0 5.7 5.7l1-1"/>',
  esp: '<rect x="5" y="7" width="14" height="10" rx="1.5"/><path d="M8 4v3M12 4v3M16 4v3M8 17v3M12 17v3M16 17v3"/>',
  lidar: '<circle cx="12" cy="12" r="3"/><path d="M5.6 5.6a9 9 0 0 0 0 12.8M18.4 5.6a9 9 0 0 1 0 12.8M8.5 8.5a5 5 0 0 0 0 7M15.5 8.5a5 5 0 0 1 0 7"/>',
  camera: '<path d="M3 8h3.5L8 6h8l1.5 2H21v11H3z"/><circle cx="12" cy="13" r="3.5"/>',
  stereo: '<circle cx="7.5" cy="12" r="4"/><circle cx="16.5" cy="12" r="4"/><path d="M11.5 12h1"/>',
  gps: '<circle cx="12" cy="10" r="3"/><path d="M12 21s-7-6.2-7-11a7 7 0 0 1 14 0c0 4.8-7 11-7 11z"/>',
  ia: '<path d="M12 3l1.8 5.2L19 10l-5.2 1.8L12 17l-1.8-5.2L5 10l5.2-1.8z"/><path d="M18.5 15.5l.8 2.2 2.2.8-2.2.8-.8 2.2-.8-2.2-2.2-.8 2.2-.8z"/>',
  wifi: '<path d="M2.5 9a14 14 0 0 1 19 0M5.5 12.5a9.5 9.5 0 0 1 13 0M8.5 16a5 5 0 0 1 7 0"/><circle cx="12" cy="19.2" r="1.1" fill="currentColor"/>',
  arme: '<path d="M13 2 5 13.5h6L10 22l8.5-12H13z"/>',
  mode: '<circle cx="12" cy="12" r="9"/><path d="M12 7v5l3 2"/>',
  micro: '<rect x="9" y="3" width="6" height="11" rx="3"/><path d="M5.5 11a6.5 6.5 0 0 0 13 0M12 17.5V21M8.5 21h7"/>',
  envoyer: '<path d="M4 12 20 4l-4 16-4-6.5z"/><path d="M12 13.5 20 4"/>',
  photo: '<path d="M3 8h3.5L8 6h8l1.5 2H21v11H3z"/><circle cx="12" cy="13" r="3.5"/><path d="M17.5 10.5h.01"/>',
  plus: '<path d="M12 5v14M5 12h14"/>',
  moins: '<path d="M5 12h14"/>',
  cible: '<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="2"/><path d="M12 2v4M12 18v4M2 12h4M18 12h4"/>',
  suivre: '<path d="M12 3 19 20l-7-4-7 4z"/>',
  ajuster: '<path d="M4 9V4h5M15 4h5v5M20 15v5h-5M9 20H4v-5"/>',
  raz: '<path d="M4 12a8 8 0 1 0 2.4-5.7"/><path d="M4 4v4.5h4.5"/>',
  pause: '<rect x="6.5" y="5" width="3.5" height="14" rx="1"/><rect x="14" y="5" width="3.5" height="14" rx="1"/>',
  lecture: '<path d="M7 4.5v15L19 12z"/>',
  annuler: '<path d="M6 6l12 12M18 6 6 18"/>',
  avancer: '<path d="M12 20V5M6 11l6-6 6 6"/>',
  reculer: '<path d="M12 4v15M6 13l6 6 6-6"/>',
  gauche90: '<path d="M18 20v-7a5 5 0 0 0-5-5H5"/><path d="M9 4 5 8l4 4"/>',
  droite90: '<path d="M6 20v-7a5 5 0 0 1 5-5h8"/><path d="M15 4l4 4-4 4"/>',
  maison: '<path d="M4 11 12 4l8 7v9h-5.5v-6h-5v6H4z"/>',
  oeil: '<path d="M2 12s3.6-6.5 10-6.5S22 12 22 12s-3.6 6.5-10 6.5S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>',
  loupe: '<circle cx="10.5" cy="10.5" r="6.5"/><path d="M15.5 15.5 21 21"/>',
  arret: '<rect x="6" y="6" width="12" height="12" rx="1.5"/>',
  main: '<path d="M8 13V5.5a1.5 1.5 0 0 1 3 0V11M11 10V4.5a1.5 1.5 0 0 1 3 0V11M14 10.5V6a1.5 1.5 0 0 1 3 0v8a7 7 0 0 1-7 7h-.5a6 6 0 0 1-5-2.7L3 16a1.6 1.6 0 0 1 2.6-1.8L8 16.5"/>',
  auto: '<path d="M12 3v3M8 21h8M5 10h14v8H5z"/><circle cx="9" cy="14" r="1.3" fill="currentColor"/><circle cx="15" cy="14" r="1.3" fill="currentColor"/><path d="M8 6h8v4H8z"/>',
  banc: '<path d="M14.5 6.5a4 4 0 0 0-5.3 5.3L3.5 17.5 6.5 20.5l5.7-5.7a4 4 0 0 0 5.3-5.3l-2.5 2.5-2.5-.5-.5-2.5z"/>',
  ok: '<path d="M5 12.5 10 17.5 19.5 7"/>',
  alerte: '<path d="M12 3 2.5 20h19z"/><path d="M12 10v4.5M12 17.2v.3"/>',
  info: '<circle cx="12" cy="12" r="9"/><path d="M12 11v6M12 7.5v.3"/>',
  soleil: '<circle cx="12" cy="12" r="4"/><path d="M12 2v2.5M12 19.5V22M2 12h2.5M19.5 12H22M4.9 4.9l1.8 1.8M17.3 17.3l1.8 1.8M4.9 19.1l1.8-1.8M17.3 6.7l1.8-1.8"/>',
  lune: '<path d="M20 14.5A8 8 0 1 1 9.5 4a6.5 6.5 0 0 0 10.5 10.5z"/>',
  bouclier: '<path d="M12 3 4.5 6v6c0 4.5 3.2 7.8 7.5 9 4.3-1.2 7.5-4.5 7.5-9V6z"/>',
  cadenas: '<rect x="5" y="11" width="14" height="10" rx="1.5"/><path d="M8 11V8a4 4 0 0 1 8 0v3"/>',
  terminal: '<rect x="3" y="4.5" width="18" height="15" rx="1.5"/><path d="M7 9.5l3 2.5-3 2.5M12.5 15H17"/>',
  journal: '<path d="M5 4h14v16H5z"/><path d="M8.5 8.5h7M8.5 12h7M8.5 15.5h4"/>',
  marche: '<path d="M12 3v8"/><path d="M6.3 6.8a8 8 0 1 0 11.4 0"/>',
  parcours: '<circle cx="5.5" cy="18.5" r="2"/><circle cx="18.5" cy="5.5" r="2"/><path d="M7.5 18.5h6a3 3 0 0 0 0-6h-3a3 3 0 0 1 0-6h6"/>',
  zone: '<path d="M4 7 10 3.5l9.5 3-1.5 11L8 20.5 3.5 15z" stroke-dasharray="3 2.4"/>',
  boucle: '<path d="M17 2.5 20.5 6 17 9.5"/><path d="M3.5 11.5V10a4 4 0 0 1 4-4h13M7 21.5 3.5 18 7 14.5"/><path d="M20.5 12.5V14a4 4 0 0 1-4 4h-13"/>',
  calibre: '<path d="M3 17 17 3l4 4L7 21H3z"/><path d="M7 13l2 2M10 10l2 2M13 7l2 2"/>',
  drone: '<circle cx="6" cy="6" r="3"/><circle cx="18" cy="6" r="3"/><circle cx="6" cy="18" r="3"/><circle cx="18" cy="18" r="3"/><path d="M8.2 8.2 15.8 15.8M15.8 8.2 8.2 15.8"/><rect x="9.5" y="9.5" width="5" height="5" rx="1.2"/>',
};

/** Icône SVG (24×24, trait courant). */
export function icone(nom, classe = 'icone') {
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.setAttribute('viewBox', '0 0 24 24');
  svg.setAttribute('class', classe);
  svg.setAttribute('aria-hidden', 'true');
  svg.innerHTML = TRACES[nom] || TRACES.info;
  return svg;
}

/** Remplace les <i data-icone="nom"> du document par leur SVG. */
export function poserIcones(racine = document) {
  for (const i of racine.querySelectorAll('i[data-icone]')) {
    const svg = icone(i.dataset.icone, `icone ${i.className}`.trim());
    i.replaceWith(svg);
  }
}

// ------------------------------------------------------------------ notifications

let pile = null;

/** Message court en bas de l'écran. niveau : info, succes, alerte, erreur. */
export function signaler(message, niveau = 'info', duree_ms = 3500) {
  if (!pile) {
    pile = el('div', { class: 'notifications', 'aria-live': 'polite' });
    document.body.append(pile);
  }
  const nom = { succes: 'ok', alerte: 'alerte', erreur: 'alerte', info: 'info' }[niveau] || 'info';
  const bulle = el('div', { class: 'notification', dataset: { niveau } }, icone(nom), el('span', { text: message }));
  pile.append(bulle);
  while (pile.childElementCount > 4) pile.firstElementChild.remove();
  setTimeout(() => {
    bulle.classList.add('sortie');
    setTimeout(() => bulle.remove(), 250);
  }, duree_ms);
}

/** Affiche le refus d'une demande (`reponse.ok === false`). Renvoie la réponse. */
export function signalerRefus(reponse, prefixe = '') {
  if (reponse && reponse.ok === false) signaler(`${prefixe}${reponse.erreur || 'refusé'}`, 'erreur', 5000);
  return reponse;
}

// ------------------------------------------------------------------ confirmation légère

let bulleConfirmation = null;

/**
 * Petite bulle « question ✓ ✗ » près d'un point de l'écran ou d'un élément. Se ferme seule
 * après 6 s (refus). Renvoie une promesse booléenne.
 */
export function confirmer(question, ancre, { danger = false, oui = 'Oui', non = 'Non' } = {}) {
  fermerConfirmation(false);
  return new Promise((resoudre) => {
    const boutonOui = el('button', { type: 'button', class: danger ? 'bouton bouton-danger' : 'bouton bouton-principal' }, icone('ok'), el('span', { text: oui }));
    const boutonNon = el('button', { type: 'button', class: 'bouton' }, icone('annuler'), el('span', { text: non }));
    const bulle = el('div', { class: 'confirmation', role: 'dialog', 'aria-label': question },
      el('div', { class: 'question', text: question }), el('div', { class: 'choix' }, boutonNon, boutonOui));
    document.body.append(bulle);
    let x, y;
    if (ancre instanceof Element) {
      const r = ancre.getBoundingClientRect();
      x = r.left + r.width / 2;
      y = r.bottom + 8;
    } else {
      x = ancre?.x ?? innerWidth / 2;
      y = (ancre?.y ?? innerHeight / 2) + 14;
    }
    const r = bulle.getBoundingClientRect();
    x = Math.min(innerWidth - r.width - 8, Math.max(8, x - r.width / 2));
    if (y + r.height > innerHeight - 8) y = Math.max(8, y - r.height - 36);
    bulle.style.left = `${x}px`;
    bulle.style.top = `${y}px`;
    const terminer = (valeur) => {
      if (bulleConfirmation?.bulle !== bulle) return;
      clearTimeout(bulleConfirmation.minuterie);
      document.removeEventListener('keydown', clavier, true);
      document.removeEventListener('pointerdown', dehors, true);
      bulle.remove();
      bulleConfirmation = null;
      resoudre(valeur);
    };
    const clavier = (e) => {
      if (e.key === 'Escape') { e.preventDefault(); terminer(false); }
      if (e.key === 'Enter') { e.preventDefault(); terminer(true); }
    };
    const dehors = (e) => { if (!bulle.contains(e.target)) terminer(false); };
    boutonOui.addEventListener('click', () => terminer(true));
    boutonNon.addEventListener('click', () => terminer(false));
    setTimeout(() => {
      document.addEventListener('keydown', clavier, true);
      document.addEventListener('pointerdown', dehors, true);
    }, 0);
    bulleConfirmation = { bulle, terminer, minuterie: setTimeout(() => terminer(false), 6000) };
    boutonOui.focus({ preventScroll: true });
  });
}

export function fermerConfirmation(valeur = false) {
  bulleConfirmation?.terminer(valeur);
}

// ------------------------------------------------------------------ stockage local

export function lireLocal(cle, defaut) {
  try {
    const v = localStorage.getItem(`fieldmind.${cle}`);
    return v === null ? defaut : JSON.parse(v);
  } catch {
    return defaut;
  }
}

export function ecrireLocal(cle, valeur) {
  try {
    localStorage.setItem(`fieldmind.${cle}`, JSON.stringify(valeur));
  } catch { /* navigation privée */ }
}
