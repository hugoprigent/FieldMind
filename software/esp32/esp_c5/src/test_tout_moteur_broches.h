#pragma once
#include <stddef.h>
#include <stdint.h>

// Cablage commun au main et au test combine. D7 ne peut pas accueillir un GPS en meme temps.
namespace test_actionneurs {
constexpr uint8_t PIN_SERVO = 12; // D7 / GPIO12
constexpr uint8_t PINS_DIR[] = {0, 23}; // D1 pour gauche D0 ; D4 pour droite D9
// Sorties SC facultatives des drivers (commande roues 1) : D2/GPIO25 gauche, D3/GPIO7 droite.
// Broches de strapping (front SDIO, source JTAG si eFuse) : ne fixent pas le mode de boot.
constexpr uint8_t PINS_SC[] = {25, 7};
// Gauche conservee ; droite retournee par rapport au precedent defaut (true).
// inverseg/inversed memorisent les corrections ulterieures.
constexpr bool SENS_INVERSE[] = {true, false};
constexpr bool niveauDirection(size_t moteur, bool sens) {
  return sens != SENS_INVERSE[moteur];
}
constexpr bool brochesTestValides() {
  const uint8_t pins[] = {1, 9, PINS_DIR[0], PINS_DIR[1], 10, PIN_SERVO, 24, 11, PINS_SC[0], PINS_SC[1]};
  for (size_t i = 0; i < sizeof(pins); ++i)
    for (size_t j = i + 1; j < sizeof(pins); ++j)
      if (pins[i] == pins[j]) return false;
  return true;
}
static_assert(brochesTestValides(), "Conflit traction/servo/I2C du test combine");
} // namespace test_actionneurs
