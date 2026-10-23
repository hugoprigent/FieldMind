#pragma once
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include "servo_angles.h"

namespace test_actionneurs {
enum class Type { Vide, Moteur, Gauche, Droite, Servo, Vitesse, Centre, Stop, Etat, Aide,
                  Sens, SensGauche, SensDroite, InverseGauche, InverseDroite, Frein, Course,
                  // ROVER_3 : p g d servo, hb, wd ms, inv ms, tel hz, imu hz, lacet0, roues 0/1.
                  Pilotage, Battement, Chien, Inversion, Telemetrie, Centrale, Lacet0, Roues,
                  // ROVER_4 : volets de la remorque, servos sur D2 (volg) et D3 (vold).
                  VoletGauche, VoletDroite, Invalide };
// valeur2/valeur3 : roue droite et angle servo de p.
struct Commande { Type type = Type::Invalide; float valeur = 0, valeur2 = 0, valeur3 = 0; };

inline bool entier(float v, float min, float max) {
  return v >= min && v <= max && std::floor(v) == v;
}

// p <g> <d> <servo> : deux entiers -100..100 puis un angle (bornes de s).
inline Commande analyserPilotage(const char *texte) {
  float v[3] = {};
  for (int i = 0; i < 3; ++i) {
    errno = 0;
    char *fin = nullptr;
    v[i] = std::strtof(texte, &fin);
    if (errno || fin == texte || !std::isfinite(v[i])) return {};
    if (i < 2 && *fin != ' ' && *fin != '\t') return {};
    texte = fin;
    while (*texte == ' ' || *texte == '\t') ++texte;
  }
  if (*texte || !entier(v[0], -100, 100) || !entier(v[1], -100, 100) || std::fabs(v[2]) > 180)
    return {};
  return {Type::Pilotage, v[0], v[1], v[2]};
}

inline Commande analyser(const char *texte) {
  if (!texte) return {};
  while (*texte == ' ' || *texte == '\t') ++texte;
  if (!*texte) return {Type::Vide, 0};
  char nom[20] = {};
  unsigned n = 0;
  while (*texte && *texte != ' ' && *texte != '\t') {
    if (n >= sizeof(nom) - 1) return {};
    nom[n++] = *texte++;
  }
  while (*texte == ' ' || *texte == '\t') ++texte;
  if (!*texte) {
    if (!std::strcmp(nom, "stop")) return {Type::Stop, 0};
    if (!std::strcmp(nom, "centre")) return {Type::Centre, 0};
    if (!std::strcmp(nom, "etat") || !std::strcmp(nom, "diag")) return {Type::Etat, 0};
    if (!std::strcmp(nom, "aide")) return {Type::Aide, 0};
    if (!std::strcmp(nom, "hb")) return {Type::Battement, 0};
    if (!std::strcmp(nom, "lacet0")) return {Type::Lacet0, 0};
    return {};
  }
  if (!std::strcmp(nom, "p")) return analyserPilotage(texte);
  errno = 0;
  char *fin = nullptr;
  const float valeur = std::strtof(texte, &fin);
  if (errno || fin == texte || !std::isfinite(valeur)) return {};
  while (*fin == ' ' || *fin == '\t') ++fin;
  if (*fin) return {};
  if (!std::strcmp(nom, "m") || !std::strcmp(nom, "moteur")) {
    if (valeur >= 0 && valeur <= 100 && std::floor(valeur) == valeur) return {Type::Moteur, valeur};
  } else if (!std::strcmp(nom, "g") || !std::strcmp(nom, "d")) {
    if (valeur >= 0 && valeur <= 100 && std::floor(valeur) == valeur)
      return {nom[0] == 'g' ? Type::Gauche : Type::Droite, valeur};
  } else if (!std::strcmp(nom, "s") || !std::strcmp(nom, "servo")) {
    if (std::fabs(valeur) <= 180) return {Type::Servo, valeur};
  } else if (!std::strcmp(nom, "v") || !std::strcmp(nom, "vitesse_servo")) {
    if (valeur >= 1 && valeur <= 180) return {Type::Vitesse, valeur};
  } else if (!std::strcmp(nom, "sens")) {
    if (valeur == 0 || valeur == 1) return {Type::Sens, valeur};
  } else if (!std::strcmp(nom, "sensg") || !std::strcmp(nom, "sensd")) {
    if (valeur == 0 || valeur == 1)
      return {nom[4] == 'g' ? Type::SensGauche : Type::SensDroite, valeur};
  } else if (!std::strcmp(nom, "inverseg") || !std::strcmp(nom, "inversed")) {
    if (valeur == 0 || valeur == 1)
      return {nom[7] == 'g' ? Type::InverseGauche : Type::InverseDroite, valeur};
  } else if (!std::strcmp(nom, "frein")) {
    if (valeur == 0 || valeur == 1) return {Type::Frein, valeur};
  } else if (!std::strcmp(nom, "course")) {
    if (valeur >= 1 && valeur <= 360 && std::floor(valeur) == valeur) return {Type::Course, valeur};
  } else if (!std::strcmp(nom, "wd")) {
    if (valeur == 0 || entier(valeur, 100, 2000)) return {Type::Chien, valeur};
  } else if (!std::strcmp(nom, "inv")) {
    if (entier(valeur, 100, 3000)) return {Type::Inversion, valeur};
  } else if (!std::strcmp(nom, "tel")) {
    if (entier(valeur, 0, 50)) return {Type::Telemetrie, valeur};
  } else if (!std::strcmp(nom, "imu")) {
    if (entier(valeur, 0, 100)) return {Type::Centrale, valeur};
  } else if (!std::strcmp(nom, "roues")) {
    if (valeur == 0 || valeur == 1) return {Type::Roues, valeur};
  } else if (!std::strcmp(nom, "volg") || !std::strcmp(nom, "vold")) {
    // Angle 0..180 (servo standard 500..2500 us) ; -1 coupe le signal.
    if (valeur == -1 || (valeur >= 0 && valeur <= 180))
      return {nom[3] == 'g' ? Type::VoletGauche : Type::VoletDroite, valeur};
  }
  return {};
}

struct ServoProgressif {
  static constexpr int AMPLITUDE_US = 700; // 800..2200 us, deja essayes.
  float positionUs = servo_angles::CENTRE_US; // Derniere consigne, pas un capteur.
  float departUs = positionUs, cibleUs = positionUs;
  float vitesse = 30; // Maximum de vitesse de CONSIGNE, en degres indicatifs/s.
  int course = 180;
  bool courseRenseignee = false, signal = false;
  uint32_t debut = 0, duree = 0;

  float angle(float us) const {
    return (us - servo_angles::CENTRE_US) * course / (2.0f * AMPLITUDE_US);
  }
  int impulsion() const { return static_cast<int>(std::lround(positionUs)); }
  bool enMouvement() const { return signal && duree != 0; }

  void actualiser(uint32_t maintenant) {
    if (!signal || !duree) return;
    const uint32_t ecoule = maintenant - debut;
    if (ecoule >= duree) { positionUs = cibleUs; duree = 0; return; }
    const float x = static_cast<float>(ecoule) / duree;
    positionUs = departUs + (cibleUs - departUs) * x * x * (3.0f - 2.0f * x);
  }
  void preparer(float cible, uint32_t maintenant) {
    departUs = positionUs;
    cibleUs = cible;
    debut = maintenant;
    const float distance = std::fabs(angle(cibleUs) - angle(departUs));
    // Le pic de derivee de smoothstep vaut 1,5 : la vitesse ne depasse pas v.
    duree = distance < 0.001f ? 0 : static_cast<uint32_t>(std::ceil(1500.0f * distance / vitesse));
    if (!duree) positionUs = cibleUs;
  }
  bool demander(float degres, uint32_t maintenant) {
    int cible = 0;
    if (!servo_angles::convertir(degres, course, AMPLITUDE_US, cible)) return false;
    actualiser(maintenant);
    signal = true;
    preparer(cible, maintenant);
    return true;
  }
  bool reglerVitesse(float valeur, uint32_t maintenant) {
    if (!std::isfinite(valeur) || valeur < 1 || valeur > 180) return false;
    actualiser(maintenant);
    vitesse = valeur;
    if (enMouvement()) preparer(cibleUs, maintenant);
    return true;
  }
  bool reglerCourse(int valeur, uint32_t maintenant) {
    if (valeur < 1 || valeur > 360) return false;
    actualiser(maintenant);
    course = valeur;
    courseRenseignee = true;
    if (enMouvement()) preparer(cibleUs, maintenant);
    return true;
  }
  void stop() {
    signal = false;
    duree = 0;
    cibleUs = positionUs; // Une reconnexion ne reprend jamais l'ancienne cible.
  }
};
}  // namespace test_actionneurs
