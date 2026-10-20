#include "../src/servo_test_reglages.h"
#include <limits.h>
#include <stdio.h>
#include <initializer_list>

int main() {
  int verifications = 0;
#define CHECK(c) do { ++verifications; if (!(c)) { fprintf(stderr, "ECHEC servo ligne %d\n", __LINE__); return 1; } } while (0)
  uint16_t us = 0;
  CHECK(servo_test::angleVersImpulsion(0, us) && us == 1000);
  CHECK(servo_test::angleVersImpulsion(90, us) && us == 1500);
  CHECK(servo_test::angleVersImpulsion(180, us) && us == 2000);
  for (long invalide : {-1L, 181L, LONG_MIN, LONG_MAX})
    CHECK(!servo_test::angleVersImpulsion(invalide, us));
  CHECK(!servo_test::impulsionValide(999));
  CHECK(servo_test::impulsionValide(1000));
  CHECK(servo_test::impulsionValide(2000));
  CHECK(!servo_test::impulsionValide(2001));
  CHECK(servo_test::FREQUENCE_HZ == 50 && servo_test::PERIODE_US == 20000);
  CHECK(!servo_test::usbPerdu(1000, 1000));
  CHECK(!servo_test::usbPerdu(1249, 1000));
  CHECK(servo_test::usbPerdu(1250, 1000));
  CHECK(servo_test::usbPerdu(2000, 1000));
  CHECK(!servo_test::usbPerdu(20, UINT32_MAX - 100));
  CHECK(servo_test::usbPerdu(150, UINT32_MAX - 100));
  for (uint16_t cible : {1000, 1250, 1500, 1750, 2000}) {
    const uint32_t pas = 1U << servo_test::RESOLUTION_BITS;
    const uint32_t duty = servo_test::rapportPwm(cible);
    const long reel = (duty * servo_test::PERIODE_US + pas / 2) / pas;
    CHECK(duty > 0 && duty < pas);
    CHECK(reel >= cible - 1 && reel <= cible + 1);
  }
  printf("%d verifications servo OK (sans acces USB).\n", verifications);
  return 0;
}
