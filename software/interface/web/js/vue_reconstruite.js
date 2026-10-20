// Mode robot réel : aucune vérité terrain. La vue 3D du rover est reconstruite à partir de ce que
// le cerveau et les capteurs publient : pose estimée (EKF), roulis et tangage de l'IMU, rotation
// des roues et braquage mesurés par les codeurs.

function anglesImu([w, x, y, z]) {
  return {
    roulis: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    tangage: Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
  };
}

export class VueReconstruite {
  constructor(geometrie) {
    this.g = geometrie;
    this.roulis = 0;
    this.tangage = 0;
    this.angles = [0, 0, 0, 0];
    this.braquage = 0;
    this.tOdometrie = null;
    this.vitesse = 0;
  }

  surImu(m) {
    ({ roulis: this.roulis, tangage: this.tangage } = anglesImu(m.orientation));
  }

  surOdometrie(m) {
    if (this.tOdometrie !== null) {
      const dt = m.t - this.tOdometrie;
      const arriere = (m.roue_ar_g + m.roue_ar_d) / 2;
      // Sans codeur sur les roues avant (rover réel), elles roulent à la vitesse de l'essieu arrière.
      const vitesses = [m.roue_av_g ?? arriere, m.roue_av_d ?? arriere, m.roue_ar_g, m.roue_ar_d];
      this.angles = this.angles.map((a, i) => a + vitesses[i] * dt);
    }
    this.tOdometrie = m.t;
    this.braquage = m.braquage;
    this.vitesse = ((m.roue_ar_g + m.roue_ar_d) / 2) * this.g.rayon_roue;
  }

  /** Vue au format du message « vue » de la simulation, à partir de la pose estimée. */
  vue(pose) {
    const { empattement: L, voie_avant: voie } = this.g;
    let gauche = 0, droit = 0;
    if (Math.abs(this.braquage) > 1e-4) {
      const rayon = L / Math.tan(this.braquage);
      gauche = Math.atan(L / (rayon - voie / 2));
      droit = Math.atan(L / (rayon + voie / 2));
    }
    return {
      t: pose.t, x: pose.x, y: pose.y, z: this.g.rayon_roue, roulis: this.roulis,
      tangage: this.tangage, cap: pose.cap, vitesse: this.vitesse,
      braquage_gauche: gauche, braquage_droit: droit, angles_roues: this.angles,
    };
  }
}
