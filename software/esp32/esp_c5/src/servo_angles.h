#pragma once
#include <cmath>

namespace servo_angles {
constexpr int CENTRE_US = 1500;

// Echelle relative : zero au centre, course totale repartie de chaque cote.
// La mesure de la course reste a fournir par l'utilisateur.
inline bool convertir(float angle, int courseTotale, int amplitudeUs, int &impulsion) {
  if (!std::isfinite(angle) || courseTotale <= 0 || courseTotale > 360 ||
      amplitudeUs <= 0 || amplitudeUs > 1000 ||
      std::fabs(angle) > courseTotale / 2.0f) return false;
  impulsion = CENTRE_US + static_cast<int>(std::lround(2.0f * amplitudeUs * angle / courseTotale));
  return true;
}
}  // namespace servo_angles
