#include "../src/servo_angles.h"
#include <cstdio>
#include <limits>

int main() {
  int verifications = 0, us = -1;
#define CHECK(c) do { ++verifications; if (!(c)) { std::fprintf(stderr, "ECHEC angles ligne %d\n", __LINE__); return 1; } } while (0)
  CHECK(servo_angles::convertir(0, 180, 700, us) && us == 1500);
  CHECK(servo_angles::convertir(-90, 180, 700, us) && us == 800);
  CHECK(servo_angles::convertir(90, 180, 700, us) && us == 2200);
  CHECK(servo_angles::convertir(-45, 180, 700, us) && us == 1150);
  CHECK(servo_angles::convertir(45, 180, 700, us) && us == 1850);
  CHECK(servo_angles::convertir(-22.5f, 180, 700, us) && us == 1325);
  // Une commande hors course est refusee sans modifier la cible precedente.
  us = 1500;
  CHECK(!servo_angles::convertir(180, 180, 700, us) && us == 1500);
  CHECK(!servo_angles::convertir(-180, 180, 700, us) && us == 1500);
  CHECK(!servo_angles::convertir(90.1f, 180, 700, us) && us == 1500);
  // Exemple de mesure totale 120 : le centre reste zero, les cotes sont +/-60.
  CHECK(servo_angles::convertir(0, 120, 700, us) && us == 1500);
  CHECK(servo_angles::convertir(-60, 120, 700, us) && us == 800);
  CHECK(servo_angles::convertir(60, 120, 700, us) && us == 2200);
  CHECK(!servo_angles::convertir(61, 120, 700, us));
  CHECK(servo_angles::convertir(0, 125, 700, us) && us == 1500);
  CHECK(servo_angles::convertir(62.5f, 125, 700, us) && us == 2200);
  // L'elargissement change les extremites mais ne deplace pas l'origine.
  CHECK(servo_angles::convertir(0, 180, 1000, us) && us == 1500);
  CHECK(servo_angles::convertir(-90, 180, 1000, us) && us == 500);
  CHECK(servo_angles::convertir(90, 180, 1000, us) && us == 2500);
  CHECK(!servo_angles::convertir(0, 0, 700, us));
  CHECK(!servo_angles::convertir(0, 180, 1001, us));
  CHECK(!servo_angles::convertir(0, 180, 0, us));
  CHECK(!servo_angles::convertir(std::numeric_limits<float>::infinity(), 180, 700, us));
  CHECK(!servo_angles::convertir(std::numeric_limits<float>::quiet_NaN(), 180, 700, us));
  std::printf("%d verifications angles centres OK (sans acces USB).\n", verifications);
  return 0;
}
