#pragma once
// Lecture facultative des sorties SC (impulsions de vitesse) des deux drivers ZS-X11D1.
// Fronts montants comptes par interruption, avec anti-rebond : un front arrivant moins de
// INTERVALLE_MIN_US apres le precedent front accepte est rejete (compte a part).
// La frequence (impulsions/s) est la moyenne des impulsions de chaque fenetre d'environ
// 100 ms ; sans front pendant ARRET_US, elle vaut 0. Logique sans materiel, testee sur le PC.
#include <cstdint>

namespace roues_sc {
constexpr uint32_t INTERVALLE_MIN_US = 200; // > 5000 impulsions/s : rebond ou parasite
constexpr uint32_t ARRET_US = 500000;       // plus lent que 2 impulsions/s : roue arretee
constexpr uint32_t MESURE_MS = 100;         // fenetre de lissage

struct Releve { uint32_t fronts = 0, rejets = 0, dernierUs = 0, periodeUs = 0; };

// Ecrit uniquement par l'interruption (et remis a zero sous section critique).
struct Compteur {
  volatile uint32_t fronts = 0, rejets = 0, dernierUs = 0, periodeUs = 0;

  // Appele par l'interruption : reste court et sans appel de fonction.
  inline __attribute__((always_inline)) void front(uint32_t us) {
    if (fronts) {
      const uint32_t ecart = us - dernierUs;
      if (ecart < INTERVALLE_MIN_US) { rejets = rejets + 1; return; }
      periodeUs = ecart;
    }
    fronts = fronts + 1;
    dernierUs = us;
  }
  void remettreAZero() { fronts = 0; rejets = 0; dernierUs = 0; periodeUs = 0; }
  Releve releve() const { Releve r; r.fronts = fronts; r.rejets = rejets; r.dernierUs = dernierUs; r.periodeUs = periodeUs; return r; }
};

struct Frequence {
  uint32_t frontsPrec = 0, dernierPrecUs = 0;
  float hz = 0; // impulsions/s, non signe

  void remettreAZero() { frontsPrec = 0; dernierPrecUs = 0; hz = 0; }

  // Toutes les MESURE_MS environ, avec un releve coherent du compteur.
  void actualiser(const Releve &r, uint32_t maintenantUs) {
    const uint32_t nouveaux = r.fronts - frontsPrec;
    if (nouveaux) {
      const uint32_t ecart = r.dernierUs - dernierPrecUs; // duree des nouvelles periodes
      if (frontsPrec && ecart > 0 && ecart <= ARRET_US) hz = nouveaux * 1e6f / static_cast<float>(ecart);
      else if (r.periodeUs && r.periodeUs <= ARRET_US) hz = 1e6f / static_cast<float>(r.periodeUs);
      else hz = 0; // premier front apres un arret : pas encore de periode
    } else {
      const uint32_t age = maintenantUs - r.dernierUs;
      if (!r.fronts || age > ARRET_US) hz = 0;
      else if (age > 0 && hz > 1e6f / static_cast<float>(age)) hz = 1e6f / static_cast<float>(age); // ralentit
    }
    frontsPrec = r.fronts;
    dernierPrecUs = r.dernierUs;
  }
};

// Signe du sens logique applique a la roue (true = arriere) ; jamais de -0.0.
inline float signe(float hz, bool arriere) { return hz > 0 && arriere ? -hz : hz; }
}  // namespace roues_sc
