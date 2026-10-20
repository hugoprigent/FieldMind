#pragma once

#include <stdint.h>

// Logique indépendante du matériel, testable sur le PC sans moteur.
namespace moteur {
enum class Phase {
  Desarme, Arme, Preparation, Impulsion,
#if defined(FIELDMIND_MOTOR_ONLY) || defined(FIELDMIND_DUAL_MOTOR)
  PreparationContinue, Continu,
#endif
};

struct Policy {
  static constexpr uint32_t ARMEMENT_MS = 5000;
  static constexpr uint32_t PREPARATION_MS = 100;
  static constexpr uint32_t DUREE_MIN_MS = 50;
  static constexpr uint32_t DUREE_MAX_MS = 500;
  static constexpr int POURCENT_MAX = 5;
  static constexpr uint32_t TEST_MS = 200;

  Phase phase = Phase::Desarme;
  bool frein = false;
  bool defaut = false;
  bool direction = false;
  uint8_t demande = 0;
  uint32_t limiteArmement = 0;
  uint32_t debutImpulsion = 0;
  uint32_t finImpulsion = 0;
  const char *raison = "demarrage";

  static bool atteint(uint32_t maintenant, uint32_t limite) {
    return static_cast<int32_t>(maintenant - limite) >= 0;
  }

  void stop(const char *motif = "stop") {
    phase = Phase::Desarme;
    demande = 0;
    raison = motif;
  }

  void panne(const char *motif) {
    stop(motif);
    defaut = true;
  }

  void tick(uint32_t maintenant) {
#if defined(FIELDMIND_MOTOR_ONLY) || defined(FIELDMIND_DUAL_MOTOR)
    if (phase == Phase::PreparationContinue) {
      if (atteint(maintenant, debutImpulsion)) phase = Phase::Continu;
      return;
    }
#endif
    if (phase == Phase::Arme && atteint(maintenant, limiteArmement)) {
      stop("armement_expire");
    } else if (phase == Phase::Preparation || phase == Phase::Impulsion) {
      if (atteint(maintenant, finImpulsion)) stop("impulsion_terminee");
      else if (atteint(maintenant, debutImpulsion)) phase = Phase::Impulsion;
    }
  }

  bool armer(uint32_t maintenant) {
    tick(maintenant);
    if (defaut || frein || phase != Phase::Desarme) return false;
    phase = Phase::Arme;
    demande = 0;
    limiteArmement = maintenant + ARMEMENT_MS;
    raison = "armement_explicite";
    return true;
  }

  bool impulsion(int pourcentSigne, uint32_t duree, uint32_t maintenant) {
    tick(maintenant);
    if (defaut || frein || phase != Phase::Arme || pourcentSigne == 0 ||
        pourcentSigne < -POURCENT_MAX || pourcentSigne > POURCENT_MAX ||
        duree < DUREE_MIN_MS || duree > DUREE_MAX_MS) {
      stop("impulsion_rejetee");
      return false;
    }
    direction = pourcentSigne < 0;
    demande = static_cast<uint8_t>(pourcentSigne < 0 ? -pourcentSigne : pourcentSigne);
    debutImpulsion = maintenant + PREPARATION_MS;
    finImpulsion = debutImpulsion + duree;
    phase = Phase::Preparation;
    raison = "impulsion";
    return true;
  }

  void reglerFrein(bool actif) {
    stop(actif ? "frein_demande" : "frein_relache");
    frein = actif;
  }

  // Une ligne « test » est une demande explicite pour une seule impulsion.
  // Un nouvel essai pendant une impulsion coupe celle-ci au lieu de la prolonger.
  bool essai(int pourcentSigne, uint32_t maintenant) {
    tick(maintenant);
    if (phase != Phase::Desarme || defaut || frein || pourcentSigne == 0 ||
        pourcentSigne < -POURCENT_MAX || pourcentSigne > POURCENT_MAX) {
      stop("test_rejete");
      return false;
    }
    return armer(maintenant) && impulsion(pourcentSigne, TEST_MS, maintenant);
  }

#if defined(FIELDMIND_MOTOR_ONLY) || defined(FIELDMIND_DUAL_MOTOR)
  // Console continue du test un moteur ou du main deux moteurs. Une nouvelle valeur remplace
  // la precedente et conserve le sens. Pas d'expiration automatique.
  bool consigne(int pourcent, uint32_t maintenant) {
    if (pourcent < 0 || pourcent > 100) {
      stop("consigne_invalide");
      return false;
    }
    if (pourcent == 0) {
      stop("stop_operateur");
      return true;
    }
    tick(maintenant);
    if (defaut || frein) {
      stop("consigne_rejetee");
      return false;
    }
    demande = static_cast<uint8_t>(pourcent);
    if (phase == Phase::Continu || phase == Phase::Impulsion) {
      phase = Phase::Continu; // changement de valeur sans coupure intermediaire
    } else {
      debutImpulsion = maintenant + PREPARATION_MS;
      phase = Phase::PreparationContinue;
    }
    raison = "consigne_directe";
    return true;
  }
#endif

  uint8_t pwm(uint32_t maintenant) {
    tick(maintenant);
    const bool actif = phase == Phase::Impulsion
#if defined(FIELDMIND_MOTOR_ONLY) || defined(FIELDMIND_DUAL_MOTOR)
        || phase == Phase::Continu
#endif
        ;
    return !defaut && !frein && actif ? demande : 0;
  }

  const char *nom() const {
    if (defaut) return "DEFAUT";
    switch (phase) {
      case Phase::Arme: return "ARME";
      case Phase::Preparation: return "PREPARATION";
      case Phase::Impulsion: return "IMPULSION";
#if defined(FIELDMIND_MOTOR_ONLY) || defined(FIELDMIND_DUAL_MOTOR)
      case Phase::PreparationContinue: return "PREPARATION_CONTINUE";
      case Phase::Continu: return "CONTINU";
#endif
      default: return "DESARME";
    }
  }
};
}  // namespace moteur
