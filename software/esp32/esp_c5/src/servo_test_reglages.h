#pragma once
#include <stdint.h>

namespace servo_test {
constexpr uint8_t GPIO_SIGNAL = 1; // D0 sur XIAO ESP32-C5 (pas GPIO0)
constexpr uint32_t FREQUENCE_HZ = 50;
constexpr uint32_t PERIODE_US = 1000000 / FREQUENCE_HZ;
constexpr uint8_t RESOLUTION_BITS = 14;
constexpr uint16_t IMPULSION_MIN_US = 1000, IMPULSION_MAX_US = 2000;
constexpr uint16_t CENTRE_US = 1500;
constexpr uint32_t DELAI_PERTE_USB_MS = 250;

// Le statut USB natif peut fluctuer quelques ms ; conserver le PWM pendant
// ces fluctuations, puis couper sans reprise automatique si la perte dure.
constexpr bool usbPerdu(uint32_t maintenant, uint32_t dernierePresence) {
  return static_cast<uint32_t>(maintenant - dernierePresence) >= DELAI_PERTE_USB_MS;
}

// Plage de test RC initiale, pas une calibration de la course physique DT996.
inline bool impulsionValide(long us) {
  return us >= IMPULSION_MIN_US && us <= IMPULSION_MAX_US;
}
inline bool angleVersImpulsion(long angle, uint16_t &us) {
  if (angle < 0 || angle > 180) return false;
  us = IMPULSION_MIN_US + (angle * (IMPULSION_MAX_US - IMPULSION_MIN_US) + 90) / 180;
  return true;
}
constexpr uint32_t rapportPwm(uint16_t us) {
  return (static_cast<uint32_t>(us) * (1U << RESOLUTION_BITS) + PERIODE_US / 2) / PERIODE_US;
}
}  // namespace servo_test
