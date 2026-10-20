// Compile le vrai lecteur de frequence avec un backend simule, sans port USB.
#include "../src/test_tout_moteur_pwm.h"
#include <cstdio>

static ledc_channel_handle_t servo = {8, 0, 14, 0, 50};
static ledc_channel_handle_t moteur1 = {1, 4, 10, 1, 5000};
static ledc_channel_handle_t moteur2 = {9, 5, 10, 1, 5000};
static uint32_t timers[] = {50, 5000, 123, 0};
static uint32_t duty[] = {0, 0, 0};
static bool attaches[] = {true, true, true};
static unsigned lecturesTimer = 0;

void *perimanGetPinBus(uint8_t pin, peripheral_bus_type_t) {
  if (pin == 8 && attaches[0]) return &servo;
  if (pin == 1 && attaches[1]) return &moteur1;
  if (pin == 9 && attaches[2]) return &moteur2;
  return nullptr;
}
uint32_t ledc_get_freq(ledc_mode_t mode, ledc_timer_t timer) {
  ++lecturesTimer;
  return mode == LEDC_LOW_SPEED_MODE ? timers[timer] : 0;
}
uint32_t ledcReadFreq(uint8_t pin) {
  // Reproduit le garde-fou duty==0 de la version Arduino-ESP32 installee.
  const int i = pin == 8 ? 0 : pin == 1 ? 1 : 2;
  const auto *bus = static_cast<const ledc_channel_handle_t *>(perimanGetPinBus(pin, ESP32_BUS_TYPE_LEDC));
  return bus && duty[i] ? timers[bus->timer_num] : 0;
}

int main() {
  int n = 0;
#define CHECK(c) do { ++n; if (!(c)) { std::fprintf(stderr, "ECHEC PWM combine ligne %d\n", __LINE__); return 1; } } while (0)
  // Avant la premiere impulsion, l'ancienne API renvoie zero : pas de faux defaut.
  CHECK(ledcReadFreq(8) == 0);
  CHECK(test_actionneurs::frequencePwm(8) == 50);
  CHECK(test_actionneurs::frequencePwm(1) == 5000);
  CHECK(test_actionneurs::frequencePwm(9) == 5000);
  CHECK(test_actionneurs::frequenceValide(test_actionneurs::frequencePwm(8), 50));
  CHECK(test_actionneurs::frequenceValide(test_actionneurs::frequencePwm(1), 5000));
  // Duty normal ou plein : la frequence lue ne change pas.
  duty[0] = 1228; duty[1] = 1024; duty[2] = 102;
  CHECK(test_actionneurs::frequencePwm(8) == 50 && ledcReadFreq(8) == 50);
  CHECK(test_actionneurs::frequencePwm(1) == 5000 && ledcReadFreq(1) == 5000);
  // Lire le vrai timer, pas la frequence demandee stockee dans les metadonnees.
  timers[1] = 50;
  CHECK(!test_actionneurs::frequenceValide(test_actionneurs::frequencePwm(1), 5000));
  CHECK(!test_actionneurs::frequenceValide(test_actionneurs::frequencePwm(9), 5000));
  timers[0] = 5000;
  CHECK(!test_actionneurs::frequenceValide(test_actionneurs::frequencePwm(8), 50));
  timers[0] = 49;
  CHECK(test_actionneurs::frequenceValide(test_actionneurs::frequencePwm(8), 50));
  CHECK(test_actionneurs::frequenceValide(51, 50));
  CHECK(!test_actionneurs::frequenceValide(48, 50));
  CHECK(test_actionneurs::frequenceValide(4950, 5000));
  CHECK(test_actionneurs::frequenceValide(5050, 5000));
  CHECK(!test_actionneurs::frequenceValide(4949, 5000));
  CHECK(!test_actionneurs::frequenceValide(0, 50));
  CHECK(!test_actionneurs::frequenceValide(50, 0));
  attaches[0] = false;
  const unsigned avant = lecturesTimer;
  CHECK(test_actionneurs::frequencePwm(8) == 0);
  CHECK(lecturesTimer == avant); // Broche detachee : ne pas interroger un timer arbitraire.
  CHECK(test_actionneurs::frequencePwm(27) == 0);
  std::printf("%d verifications lecture PWM OK (sans acces USB).\n", n);
}
