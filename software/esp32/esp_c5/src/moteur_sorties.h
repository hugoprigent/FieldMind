#pragma once

#include <stddef.h>
#include <stdint.h>

namespace moteur {
constexpr uint8_t PIN_PWM = 1;    // D0, driver 1
constexpr uint8_t PIN_DIR = 0;    // D1, commun aux deux drivers
constexpr uint8_t PIN_FREIN = 10; // D10, commun aux deux drivers
constexpr uint8_t PIN_I2C_SDA = 24, PIN_I2C_SCL = 11; // cablage actuel
constexpr uint8_t PIN_SERVO_RESERVEE = 8, PIN_GPS_RX_RESERVEE = 12;
constexpr uint8_t PINS_PWM[] = {
    PIN_PWM,
#ifdef FIELDMIND_DUAL_MOTOR
    9, // D9, driver 2 ; D8 reserve au servo et D7 au GPS
#endif
};
constexpr size_t NOMBRE_MOTEURS = sizeof(PINS_PWM) / sizeof(PINS_PWM[0]);

constexpr bool brochesMoteursValides() {
  for (size_t i = 0; i < NOMBRE_MOTEURS; ++i) {
    if (PINS_PWM[i] == PIN_DIR || PINS_PWM[i] == PIN_FREIN ||
        PINS_PWM[i] == PIN_I2C_SDA || PINS_PWM[i] == PIN_I2C_SCL ||
        PINS_PWM[i] == PIN_SERVO_RESERVEE || PINS_PWM[i] == PIN_GPS_RX_RESERVEE) return false;
    for (size_t j = i + 1; j < NOMBRE_MOTEURS; ++j)
      if (PINS_PWM[i] == PINS_PWM[j]) return false;
  }
  return PIN_DIR != PIN_FREIN;
}
static_assert(brochesMoteursValides(), "Conflit de broches moteur/I2C/servo/GPS");

// Si une ecriture echoue, demander zero sur TOUT le groupe avant de signaler
// la panne. L'appelant remet ensuite toutes les sorties en GPIO bas.
template <size_t N, typename Ecriture>
bool ecrireGroupePwm(const uint8_t (&broches)[N], uint32_t valeur, Ecriture ecrire) {
  for (uint8_t broche : broches) {
    if (!ecrire(broche, valeur)) {
      for (uint8_t autre : broches) ecrire(autre, 0);
      return false;
    }
  }
  return true;
}
}  // namespace moteur
