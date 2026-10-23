// Traduction de présentation uniquement : les valeurs, identifiants et messages du
// protocole restent identiques. Aucun service de géolocalisation n'est interrogé.
import { anglais } from './langue.en.js';

export function detecterLangue({ recherche = '', choix = null, langues = [], fuseau = '' } = {}) {
  const demandee = new URLSearchParams(recherche).get('lang');
  if (['fr', 'en'].includes(demandee)) return demandee;
  if (['fr', 'en'].includes(choix)) return choix;
  for (const langue of langues) {
    if (/^fr(?:-|$)/i.test(langue)) return 'fr';
    if (/^en(?:-|$)/i.test(langue)) return 'en';
  }
  return ['Europe/Paris', 'Europe/Brussels', 'Europe/Monaco', 'Europe/Luxembourg', 'America/Montreal'].includes(fuseau) ? 'fr' : 'en';
}
let choix = null;
try { choix = localStorage.getItem('portfolio-lang'); } catch { /* stockage facultatif */ }
export let langue = detecterLangue({ recherche: globalThis.location?.search, choix,
  langues: globalThis.navigator?.languages || [], fuseau: Intl.DateTimeFormat().resolvedOptions().timeZone });
const sources = new WeakMap();
const attributs = new WeakMap();
const ignorer = 'script,style,code,pre,textarea,[translate="no"],[data-i18n-ignore]';
const fragments = Object.entries(anglais).filter(([fr]) => fr.length >= 4)
  .sort((a, b) => b[0].length - a[0].length);
const echapper = s => s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
const motif = new RegExp(`(?<![\\p{L}\\p{N}_])(?:${fragments.map(([fr]) => echapper(fr)).join('|')})(?![\\p{L}\\p{N}_])`, 'gu');

export function tr(valeur, locale = langue) {
  if (locale === 'fr' || typeof valeur !== 'string' || !/[\p{L}]/u.test(valeur)) return valeur;
  const debut = valeur.match(/^\s*/)[0], fin = valeur.match(/\s*$/)[0], centre = valeur.trim();
  if (Object.hasOwn(anglais, centre)) return debut + anglais[centre] + fin;
  // Les libellés de télémétrie comportent des nombres variables. On traduit les
  // fragments connus une seule fois, sans retraduire le texte anglais obtenu.
  return debut + centre.replace(motif, mot => anglais[mot]) + fin;
}

export function poserTexte(element, valeur) {
  if (!element) return;
  const source = String(valeur ?? ''), visible = element.closest?.(ignorer) ? source : tr(source);
  if (element.textContent !== visible) element.textContent = visible;
  if (element.childNodes.length === 1 && element.firstChild.nodeType === 3) sources.set(element.firstChild, { source, visible });
}

function traduireNoeud(noeud) {
  if (noeud.nodeType === 3) {
    if (noeud.parentElement?.closest(ignorer)) return;
    const ancien = sources.get(noeud), source = ancien?.visible === noeud.data ? ancien.source : noeud.data;
    const visible = tr(source);
    sources.set(noeud, { source, visible });
    if (noeud.data !== visible) noeud.data = visible;
  } else if (noeud.nodeType === 1 && !noeud.matches(ignorer)) {
    let memoire = attributs.get(noeud);
    if (!memoire) attributs.set(noeud, memoire = new Map());
    for (const cle of ['title', 'aria-label', 'placeholder', 'alt']) {
      if (!noeud.hasAttribute(cle)) continue;
      const ancien = memoire.get(cle), actuel = noeud.getAttribute(cle);
      const source = ancien?.visible === actuel ? ancien.source : actuel, visible = tr(source);
      memoire.set(cle, { source, visible });
      if (actuel !== visible) noeud.setAttribute(cle, visible);
    }
    for (const enfant of noeud.childNodes) traduireNoeud(enfant);
  }
}

export function initialiserLangue(arret = () => {}) {
  const groupe = document.createElement('div');
  groupe.className = 'choix-langue'; groupe.setAttribute('role', 'group'); groupe.setAttribute('aria-label', 'FR / EN'); groupe.dataset.i18nIgnore = '';
  for (const l of ['fr', 'en']) {
    const bouton = document.createElement('button'); bouton.type = 'button'; bouton.textContent = l.toUpperCase(); bouton.lang = l;
    bouton.title = l === 'fr' ? 'Français' : 'English'; bouton.dataset.langue = l;
    bouton.addEventListener('click', () => {
      arret(); langue = l;
      try { localStorage.setItem('portfolio-lang', l); } catch { /* stockage facultatif */ }
      const url = new URL(location.href); url.searchParams.set('lang', l); history.replaceState(history.state, '', url);
      appliquer(); window.dispatchEvent(new Event('fieldmind-langue'));
    });
    groupe.append(bouton);
  }
  document.querySelector('#bouton-arret').before(groupe);
  const observer = new MutationObserver(changements => {
    observer.disconnect();
    for (const m of changements) {
      if (m.type === 'childList') for (const n of m.addedNodes) traduireNoeud(n);
      else traduireNoeud(m.target);
    }
    observer.observe(document.documentElement, options);
  });
  const options = { subtree: true, childList: true, characterData: true, attributes: true, attributeFilter: ['title', 'aria-label', 'placeholder', 'alt'] };
  function appliquer() {
    observer.disconnect(); document.documentElement.lang = langue;
    traduireNoeud(document.documentElement);
    for (const b of groupe.children) b.setAttribute('aria-pressed', String(b.dataset.langue === langue));
    const touches = document.querySelectorAll('.clavier kbd');
    for (const kbd of touches) {
      if (['Z', 'W'].includes(kbd.textContent)) kbd.textContent = langue === 'en' ? 'W' : 'Z';
      if (['Q', 'A'].includes(kbd.textContent)) kbd.textContent = langue === 'en' ? 'A' : 'Q';
    }
    observer.observe(document.documentElement, options);
  }
  appliquer();
}
