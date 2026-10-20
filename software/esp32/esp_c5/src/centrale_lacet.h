#pragma once
// Lacet integre du gyroscope Z, avec biais estime quand le rover est immobile.
// Immobile = traction a zero depuis plus d'une seconde (fourni par l'appelant) et
// gyroscope calme : chaque axe a moins de SEUIL_CALME_DPS de sa moyenne sur ~0,25 s et,
// une fois le biais estime (2 s d'arret), gz a moins de SEUIL_ROTATION_DPS du biais.
// A l'arret, le biais est ajuste et le lacet reste fige. Un ecart stable au biais
// pendant 5 s, traction au repos, relance l'estimation. Logique testee sur le PC.
#include <cmath>
#include <cstdint>

namespace centrale {
struct Lacet {
  static constexpr float DT_MAX_S = 0.25f;     // trou de mesures : pas d'integration
  static constexpr float SEUIL_CALME_DPS = 1.0f;
  static constexpr float TAU_COURT_S = 0.25f;  // moyenne courte du test de calme
  static constexpr float TAU_BIAIS_S = 4.0f;   // moyenne simple, puis glissante sur ~4 s
  static constexpr float SEUIL_ROTATION_DPS = 3.0f;
  static constexpr float DUREE_INIT_S = 2.0f;
  static constexpr float DUREE_REPRISE_S = 5.0f;

  float lacet = 0;  // degres, dans [-180, 180)
  float biais = 0;  // deg/s
  uint32_t echantillonsBiais = 0;
  float tempsBiais = 0, tempsEcart = 0; // s d'estimation ; s stable mais loin du biais
  float moyenne[3] = {};
  bool initialise = false, immobile = false;

  void remettreAZero() { lacet = 0; }

  static float borner(float angle) {
    angle = std::fmod(angle + 180.0f, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle - 180.0f;
  }

  // g en deg/s, dt en s depuis l'echantillon precedent (0 pour le premier).
  void ajouter(float gx, float gy, float gz, float dt, bool tractionAuRepos) {
    const float g[3] = {gx, gy, gz};
    if (!initialise) {
      for (int i = 0; i < 3; ++i) moyenne[i] = g[i];
      initialise = true;
    }
    const float a = dt > 0 ? std::fmin(1.0f, dt / TAU_COURT_S) : 0.0f;
    bool stable = true;
    for (int i = 0; i < 3; ++i) {
      stable = stable && std::fabs(g[i] - moyenne[i]) < SEUIL_CALME_DPS;
      moyenne[i] += a * (g[i] - moyenne[i]);
    }
    const bool procheBiais = tempsBiais < DUREE_INIT_S || std::fabs(gz - biais) < SEUIL_ROTATION_DPS;
    if (tractionAuRepos && stable && !procheBiais && dt <= DT_MAX_S) {
      tempsEcart += dt;
      if (tempsEcart >= DUREE_REPRISE_S) { echantillonsBiais = 0; tempsBiais = 0; tempsEcart = 0; }
    } else {
      tempsEcart = 0;
    }
    immobile = tractionAuRepos && stable && procheBiais;
    if (immobile) {
      ++echantillonsBiais;
      if (dt <= DT_MAX_S) tempsBiais += dt;
      const float k = std::fmax(1.0f / static_cast<float>(echantillonsBiais),
                                dt > 0 ? std::fmin(1.0f, dt / TAU_BIAIS_S) : 0.0f);
      biais += k * (gz - biais);
      return;
    }
    if (dt <= 0 || dt > DT_MAX_S) return;
    lacet = borner(lacet + (gz - biais) * dt);
  }
};
}  // namespace centrale
