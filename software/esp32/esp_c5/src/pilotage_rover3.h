#pragma once
// Extension ROVER_3 : chien de garde, consigne signee par roue avec passage par zero,
// formats des lignes T et I. Logique sans materiel, testee sur le PC.
#include <cstdint>
#include <cstdio>
#include "moteur_policy.h"

namespace rover3 {
constexpr uint32_t CHIEN_MIN_MS = 100, CHIEN_MAX_MS = 2000;
constexpr uint32_t INVERSION_MIN_MS = 100, INVERSION_MAX_MS = 3000, INVERSION_DEFAUT_MS = 500;
constexpr uint32_t TELEMETRIE_MAX_HZ = 50, CENTRALE_MAX_HZ = 100;

inline bool atteint(uint32_t maintenant, uint32_t limite) {
  return static_cast<int32_t>(maintenant - limite) >= 0;
}

// Sans ligne p ou hb acceptee pendant delai ms : expiration (une seule fois).
// Pas de reprise automatique : seule une nouvelle consigne p remet la traction.
struct ChienDeGarde {
  uint32_t delai = 0; // 0 = coupe (etat au demarrage)
  uint32_t dernier = 0;
  bool expire = false;

  bool regler(uint32_t ms, uint32_t maintenant) {
    if (ms != 0 && (ms < CHIEN_MIN_MS || ms > CHIEN_MAX_MS)) return false;
    delai = ms;
    dernier = maintenant;
    expire = false;
    return true;
  }
  void renouveler(uint32_t maintenant) { dernier = maintenant; expire = false; }
  bool actif() const { return delai != 0; }
  bool ok() const { return delai != 0 && !expire; }
  bool echu(uint32_t maintenant) const { return delai != 0 && atteint(maintenant, dernier + delai); }
  // Vrai uniquement au tour ou le chien expire.
  bool verifier(uint32_t maintenant) {
    if (expire || !echu(maintenant)) return false;
    expire = true;
    return true;
  }
  const char *nom() const { return !delai ? "coupe" : expire ? "expire" : "ok"; }
};

// Roue pilotee par une consigne signee (>= 0 avant logique, < 0 arriere logique).
// Un changement de sens d'une roue commandee passe par zero, attend inversionMs roue
// a zero, change le D/R puis applique la valeur. Rien ne bloque : actualiser() a chaque tour.
struct RoueSignee {
  bool enAttente = false;
  bool sensVise = false; // true = arriere (Policy::direction)
  uint8_t valeurVisee = 0;
  uint32_t dernierCommande = 0; // dernier instant ou la roue avait une demande non nulle

  void noter(const moteur::Policy &roue, uint32_t maintenant) {
    if (roue.demande != 0) dernierCommande = maintenant;
  }
  void annuler() { enAttente = false; }
  bool aZeroDepuis(const moteur::Policy &roue, uint32_t ms, uint32_t maintenant) const {
    return roue.demande == 0 && atteint(maintenant, dernierCommande + ms);
  }

  // Faux si la consigne est refusee (frein, defaut) : l'appelant arrete tout.
  // sensChange : le D/R logique a change, l'appelant doit reecrire les broches DIR.
  bool demander(moteur::Policy &roue, int signe, uint32_t inversionMs, uint32_t maintenant,
                bool &sensChange) {
    sensChange = false;
    noter(roue, maintenant);
    const bool sens = signe < 0;
    const int valeur = signe < 0 ? -signe : signe;
    if (valeur == 0 || sens == roue.direction) {
      enAttente = false;
      return roue.consigne(valeur, maintenant);
    }
    if (roue.frein || roue.defaut) {
      enAttente = false;
      roue.stop("consigne_rejetee");
      return false;
    }
    if (aZeroDepuis(roue, inversionMs, maintenant)) {
      enAttente = false;
      roue.direction = sens;
      sensChange = true;
      return roue.consigne(valeur, maintenant);
    }
    if (roue.demande != 0) roue.consigne(0, maintenant); // passage par zero
    enAttente = true;
    sensVise = sens;
    valeurVisee = static_cast<uint8_t>(valeur);
    return true;
  }

  // A chaque tour. Vrai si une inversion vient d'etre faite (reecrire DIR) ;
  // refus est mis a vrai si la valeur a ete refusee (frein/defaut).
  bool actualiser(moteur::Policy &roue, uint32_t inversionMs, uint32_t maintenant, bool &refus) {
    refus = false;
    noter(roue, maintenant);
    if (!enAttente || !aZeroDepuis(roue, inversionMs, maintenant)) return false;
    enAttente = false;
    if (roue.frein || roue.defaut) { roue.stop("consigne_rejetee"); refus = true; return false; }
    roue.direction = sensVise;
    refus = !roue.consigne(valeurVisee, maintenant);
    return true;
  }
};

// PWM signe : valeur appliquee, signe = sens logique applique.
inline int pwmSigne(int pourcentApplique, bool arriere) {
  const int p = pourcentApplique > 0 ? pourcentApplique : 0;
  return arriere ? -p : p;
}

// T <millis> <pwm_g> <pwm_d> <servo_deg> <frein> <defaut> <chien> [<hz_g> <hz_d>]
// Les deux derniers champs (impulsions SC/s signees) seulement si hzRoues est fourni (roues 1).
inline int formaterTelemetrie(char *tampon, size_t taille, uint32_t ms, int g, int d,
                              float servoDeg, bool frein, bool defaut, bool chien,
                              const float *hzRoues = nullptr) {
  if (hzRoues)
    return std::snprintf(tampon, taille, "T %lu %d %d %.1f %u %u %u %.1f %.1f\n",
                         static_cast<unsigned long>(ms), g, d, static_cast<double>(servoDeg),
                         frein ? 1u : 0u, defaut ? 1u : 0u, chien ? 1u : 0u,
                         static_cast<double>(hzRoues[0]), static_cast<double>(hzRoues[1]));
  return std::snprintf(tampon, taille, "T %lu %d %d %.1f %u %u %u\n",
                       static_cast<unsigned long>(ms), g, d, static_cast<double>(servoDeg),
                       frein ? 1u : 0u, defaut ? 1u : 0u, chien ? 1u : 0u);
}

// I <millis> <ax> <ay> <az> <gx> <gy> <gz> <lacet_deg> (g, deg/s, deg)
inline int formaterCentrale(char *tampon, size_t taille, uint32_t ms, const float a[3],
                            const float g[3], float lacetDeg) {
  return std::snprintf(tampon, taille, "I %lu %.3f %.3f %.3f %.2f %.2f %.2f %.2f\n",
                       static_cast<unsigned long>(ms), static_cast<double>(a[0]),
                       static_cast<double>(a[1]), static_cast<double>(a[2]),
                       static_cast<double>(g[0]), static_cast<double>(g[1]),
                       static_cast<double>(g[2]), static_cast<double>(lacetDeg));
}

// Periode entiere (ms) garantissant au moins hz lignes par seconde.
inline uint32_t periodeMs(uint32_t hz) { return hz ? 1000 / hz : 0; }

// Echeancier sans derive ; recale si le retard depasse une periode.
struct Cadence {
  uint32_t hz = 0, prochain = 0;
  void regler(uint32_t valeur, uint32_t maintenant) { hz = valeur; prochain = maintenant; }
  bool due(uint32_t maintenant) {
    if (!hz || !atteint(maintenant, prochain)) return false;
    prochain += periodeMs(hz);
    if (atteint(maintenant, prochain)) prochain = maintenant + periodeMs(hz);
    return true;
  }
};
}  // namespace rover3
