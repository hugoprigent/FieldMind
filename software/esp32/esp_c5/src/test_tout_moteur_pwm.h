#pragma once
#include <esp32-hal-ledc.h>
#include <esp32-hal-periman.h>
#include <driver/ledc.h>

namespace test_actionneurs {
// Arduino-ESP32 3.3.11 : ledcReadFreq() renvoie 0 lorsque le duty LU vaut 0,
// y compris avant le premier cycle apres ledcWrite(). Lire le timer directement
// conserve la verification de frequence sans ce faux defaut de demarrage.
inline uint32_t frequencePwm(uint8_t pin) {
  const auto *bus = static_cast<const ledc_channel_handle_t *>(perimanGetPinBus(pin, ESP32_BUS_TYPE_LEDC));
  if (!bus) return 0;
  return ledc_get_freq(static_cast<ledc_mode_t>(bus->channel / SOC_LEDC_CHANNEL_NUM),
                       static_cast<ledc_timer_t>(bus->timer_num));
}

inline bool frequenceValide(uint32_t lue, uint32_t attendue) {
  if (!lue || !attendue) return false;
  const uint32_t tolerance = attendue / 100 > 0 ? attendue / 100 : 1;
  const uint32_t ecart = lue > attendue ? lue - attendue : attendue - lue;
  return ecart <= tolerance;
}
}  // namespace test_actionneurs
