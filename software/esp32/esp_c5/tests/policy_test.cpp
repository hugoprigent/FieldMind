#include "../src/moteur_policy.h"
#include "../src/moteur_sorties.h"
#include <limits.h>
#include <initializer_list>
#include <stdio.h>

using moteur::Phase;
using moteur::Policy;
static int verifications = 0;
#define CHECK(expression) do { ++verifications; if (!(expression)) { \
  fprintf(stderr, "ECHEC ligne %d : %s\n", __LINE__, #expression); return 1; } } while (0)

int main() {
  Policy p;
  CHECK(p.phase == Phase::Desarme);
  CHECK(p.pwm(0) == 0);
  CHECK(!p.impulsion(5, 200, 0));
  CHECK(p.pwm(1000) == 0);
  CHECK(p.armer(1000));
  CHECK(!p.armer(5999));
  CHECK(p.pwm(6000) == 0);
  CHECK(p.phase == Phase::Desarme);
  CHECK(!p.impulsion(5, 200, 6000));
  for (int signe : {-1, 1}) {
    for (int pourcent = 1; pourcent <= Policy::POURCENT_MAX; ++pourcent) {
      p = Policy{};
      CHECK(p.armer(100));
      CHECK(p.impulsion(signe * pourcent, 200, 101));
      CHECK(p.direction == (signe < 0));
      CHECK(p.pwm(101) == 0);
      CHECK(p.pwm(200) == 0);
      CHECK(p.pwm(201) == pourcent);
      CHECK(p.pwm(400) == pourcent);
      CHECK(p.pwm(401) == 0);
      CHECK(p.phase == Phase::Desarme);
      CHECK(!p.impulsion(signe * pourcent, 200, 402));
    }
  }
  for (int invalide : {0, -6, 6, INT_MIN, INT_MAX}) {
    p = Policy{};
    CHECK(p.armer(0));
    CHECK(!p.impulsion(invalide, 100, 1));
    CHECK(p.pwm(200) == 0);
    CHECK(p.phase == Phase::Desarme);
  }
  for (uint32_t invalide : {0U, 49U, 501U, UINT32_MAX}) {
    p = Policy{};
    CHECK(p.armer(0));
    CHECK(!p.impulsion(1, invalide, 1));
    CHECK(p.phase == Phase::Desarme);
  }
  for (uint32_t borne : {50U, 500U}) {
    p = Policy{};
    CHECK(p.armer(0));
    CHECK(p.impulsion(5, borne, 1));
    CHECK(p.pwm(100) == 0);
    CHECK(p.pwm(101) == 5);
    CHECK(p.pwm(101 + borne) == 0);
  }
  p = Policy{};
  CHECK(p.armer(0));
  CHECK(p.impulsion(5, 500, 1));
  CHECK(p.pwm(101) == 5);
  CHECK(!p.impulsion(-5, 500, 102));
  CHECK(p.pwm(103) == 0);
  CHECK(p.phase == Phase::Desarme);
  CHECK(p.armer(200));
  CHECK(p.impulsion(5, 500, 201));
  p.stop("usb_deconnecte");
  CHECK(p.pwm(400) == 0);
  CHECK(!p.impulsion(5, 500, 401));
  p.reglerFrein(true);
  CHECK(!p.armer(500));
  CHECK(p.pwm(600) == 0);
  p.reglerFrein(false);
  CHECK(p.phase == Phase::Desarme);
  CHECK(p.armer(700));
  CHECK(p.impulsion(5, 500, 701));
  p.reglerFrein(true);
  CHECK(p.pwm(900) == 0);
  CHECK(p.phase == Phase::Desarme);
  p = Policy{};
  CHECK(p.armer(0));
  CHECK(p.impulsion(5, 500, 1));
  p.panne("pwm");
  CHECK(p.pwm(150) == 0);
  CHECK(!p.armer(160));
  p.reglerFrein(false);
  CHECK(!p.armer(170));
  p = Policy{};
  const uint32_t t = UINT32_MAX - 20;
  CHECK(p.armer(t));
  CHECK(p.impulsion(-3, 200, t));
  CHECK(p.pwm(t + 99) == 0);
  CHECK(p.pwm(t + 100) == 3);
  CHECK(p.pwm(t + 299) == 3);
  CHECK(p.pwm(t + 300) == 0);
  p = Policy{};
  CHECK(p.armer(t));
  CHECK(!p.impulsion(5, 200, t + 5000));
  CHECK(p.pwm(t + 6000) == 0);
  p = Policy{};
  CHECK(p.armer(0));
  CHECK(p.impulsion(5, 200, 1));
  CHECK(p.pwm(100000) == 0);
  CHECK(p.phase == Phase::Desarme);
  p = Policy{};
  CHECK(p.essai(1, 0));
  CHECK(p.pwm(99) == 0);
  CHECK(p.pwm(100) == 1);
  CHECK(p.pwm(299) == 1);
  CHECK(p.pwm(300) == 0);
  CHECK(p.phase == Phase::Desarme);
  CHECK(p.essai(2, 400));
  CHECK(!p.essai(3, 500)); // pas de répétition ou de prolongation en cours de test
  CHECK(p.pwm(501) == 0);
  CHECK(p.essai(-1, 600));
  CHECK(p.direction);
  CHECK(p.pwm(700) == 1);
  p.stop();
  CHECK(p.pwm(701) == 0);
  for (int invalide : {0, -6, 6, INT_MIN, INT_MAX}) {
    CHECK(!p.essai(invalide, 800));
    CHECK(p.pwm(900) == 0);
  }
  p.reglerFrein(true);
  CHECK(!p.essai(1, 1000));
  p.reglerFrein(false);
  p.panne("pwm");
  CHECK(!p.essai(1, 1001));
#if defined(FIELDMIND_MOTOR_ONLY) || defined(FIELDMIND_DUAL_MOTOR)
  for (int pourcent : {1, 50, 100}) {
    p = Policy{};
    CHECK(p.consigne(pourcent, 1000));
    CHECK(p.pwm(1099) == 0);
    CHECK(p.pwm(1100) == pourcent);
    CHECK(p.pwm(4099) == pourcent);
    CHECK(p.pwm(4100) == pourcent);
    CHECK(p.phase == Phase::Continu);
    CHECK(p.pwm(1000000) == pourcent);
    p.stop("stop_operateur");
    CHECK(p.pwm(1000001) == 0);
  }
  for (int invalide : {-1, 101, INT_MIN, INT_MAX}) {
    p = Policy{};
    CHECK(p.consigne(50, 0));
    CHECK(!p.consigne(invalide, 200));
    CHECK(p.pwm(201) == 0);
    CHECK(p.phase == Phase::Desarme);
  }
  p = Policy{};
  CHECK(p.consigne(100, 0));
  CHECK(p.consigne(0, 50)); // annulation avant le debut
  CHECK(p.pwm(100) == 0);
  CHECK(p.consigne(1, 1000));
  CHECK(p.pwm(1100) == 1);
  CHECK(p.consigne(50, 2000)); // changement direct sans coupure ou expiration
  CHECK(p.pwm(2000) == 50);
  CHECK(p.pwm(2100) == 50);
  CHECK(p.pwm(4100) == 50);
  CHECK(p.pwm(5100) == 50);
  CHECK(p.consigne(100, 6000));
  CHECK(p.pwm(6100) == 100);
  CHECK(p.consigne(0, 6101));
  CHECK(p.pwm(6102) == 0);
  CHECK(p.consigne(100, 7000));
  p.stop("usb_deconnecte");
  CHECK(p.pwm(7100) == 0);
  p.reglerFrein(true);
  CHECK(!p.consigne(50, 8000));
  CHECK(p.consigne(0, 8001));
  CHECK(p.frein);
  p.reglerFrein(false);
  CHECK(p.consigne(50, 8002));
  p.panne("pwm");
  CHECK(p.pwm(8102) == 0);
  CHECK(!p.consigne(50, 8103));
  CHECK(p.consigne(0, 8104));
  CHECK(p.defaut); // zero ne reinitialise pas un defaut
  CHECK(!p.consigne(50, 8105));
  p = Policy{};
  CHECK(p.essai(-1, 0));
  p.stop();
  CHECK(p.consigne(50, 1000));
  CHECK(p.direction); // aucune inversion de sens implicite
  p = Policy{};
  CHECK(p.consigne(100, t));
  CHECK(p.pwm(t + 99) == 0);
  CHECK(p.pwm(t + 100) == 100);
  CHECK(p.pwm(t + 3099) == 100);
  CHECK(p.pwm(t + 3100) == 100);
  CHECK(p.pwm(t + 0x80000000U) == 100); // aucune echeance fictive a long terme
  CHECK(p.pwm(t) == 100); // maintien meme apres un cycle complet de l'horloge
  CHECK(p.consigne(10, t));
  CHECK(p.pwm(t) == 10);
  p.stop("stop_operateur");
  CHECK(p.pwm(t) == 0);
  CHECK(p.phase == Phase::Desarme);
#endif
  CHECK(moteur::brochesMoteursValides());
#ifdef FIELDMIND_DUAL_MOTOR
  CHECK(moteur::NOMBRE_MOTEURS == 2);
  CHECK(moteur::PINS_PWM[0] == 1);
  CHECK(moteur::PINS_PWM[1] == 9);
  const uint8_t broches[] = {1, 9};
  uint32_t sorties[10] = {};
  int echecSur = -1;
  const auto ecrire = [&](uint8_t pin, uint32_t duty) {
    if (pin == echecSur && duty != 0) return false;
    sorties[pin] = duty;
    return true;
  };
  // Meme consigne, arret et echeance sur les deux sorties.
  p = Policy{};
  CHECK(p.consigne(10, 0));
  CHECK(moteur::ecrireGroupePwm(broches, p.pwm(100), ecrire));
  CHECK(sorties[1] == 10 && sorties[9] == 10);
  p.stop();
  CHECK(moteur::ecrireGroupePwm(broches, p.pwm(101), ecrire));
  CHECK(sorties[1] == 0 && sorties[9] == 0);
  CHECK(p.essai(1, 200));
  CHECK(moteur::ecrireGroupePwm(broches, p.pwm(300), ecrire));
  CHECK(sorties[1] == 1 && sorties[9] == 1);
  CHECK(moteur::ecrireGroupePwm(broches, p.pwm(500), ecrire));
  CHECK(sorties[1] == 0 && sorties[9] == 0);
  for (int pin : {1, 9}) {
    echecSur = -1;
    CHECK(moteur::ecrireGroupePwm(broches, 100, ecrire));
    echecSur = pin;
    CHECK(!moteur::ecrireGroupePwm(broches, 50, ecrire));
    CHECK(sorties[1] == 0 && sorties[9] == 0); // aucun moteur laisse actif
  }
#else
  CHECK(moteur::NOMBRE_MOTEURS == 1);
#endif
  printf("%d verifications OK (sans acces USB ni moteur).\n", verifications);
  return 0;
}
