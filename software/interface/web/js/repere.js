// Seule frontière entre le repère au sol du protocole et le repère de three.js.
//
// Protocole (SI) : x vers l'est, y vers le nord, z vers le haut, cap en radians dans le sens
// trigonométrique (0 = est). three.js : Y vers le haut. On pose X3 = x, Y3 = z, Z3 = -y, ce qui
// garde un repère direct et fait tourner les objets autour de Y3 du même angle que le cap.

/** Point au sol (x, y) et hauteur z → coordonnées three.js [X, Y, Z]. */
export function solVers3d(x, y, z = 0) {
  return [x, z, -y];
}

/** Coordonnées three.js → point au sol { x, y, z }. */
export function troisDVersSol(X, Y, Z) {
  return { x: X, y: -Z, z: Y };
}

/** Cap du protocole → rotation autour de l'axe Y de three.js. */
export function capVersRotationY(cap) {
  return cap;
}

/**
 * Orientation complète (roulis, tangage, cap du protocole, convention ZYX) → quaternion three.js
 * [x, y, z, w]. Roulis autour de l'axe avant du rover, tangage autour de son axe gauche.
 */
export function orientationVersQuaternion3d(roulis, tangage, cap) {
  // Quaternion au sol (w, x, y, z) pour R = Rz(cap) · Ry(tangage) · Rx(roulis).
  const cr = Math.cos(roulis / 2), sr = Math.sin(roulis / 2);
  const cp = Math.cos(tangage / 2), sp = Math.sin(tangage / 2);
  const cy = Math.cos(cap / 2), sy = Math.sin(cap / 2);
  const w = cr * cp * cy + sr * sp * sy;
  const x = sr * cp * cy - cr * sp * sy;
  const y = cr * sp * cy + sr * cp * sy;
  const z = cr * cp * sy - sr * sp * cy;
  // Changement de base (x, y, z) → (X3 = x, Y3 = z, Z3 = -y) appliqué à la partie vectorielle.
  return [x, z, -y, w];
}
