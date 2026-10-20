#include "../src/test_tout_moteur_logique.h"
#include "../src/moteur_policy.h"
#include "../src/test_tout_moteur_broches.h"
#include <cstdio>
#include <limits>
#include <initializer_list>

int main() {
  using namespace test_actionneurs;
  int verifications = 0;
#define CHECK(c) do { ++verifications; if (!(c)) { std::fprintf(stderr, "ECHEC combine ligne %d\n", __LINE__); return 1; } } while (0)
  CHECK(brochesTestValides() && PIN_SERVO == 12);
  CHECK(PINS_DIR[0] == 0 && PINS_DIR[1] == 23);
  CHECK(niveauDirection(0, false) == SENS_INVERSE[0]);
  CHECK(niveauDirection(1, false) == SENS_INVERSE[1]);
  CHECK(niveauDirection(0, false) != niveauDirection(0, true));
  CHECK(niveauDirection(1, false) != niveauDirection(1, true));
  CHECK(analyser("m 10").type == Type::Moteur && analyser("m 10").valeur == 10);
  CHECK(analyser("moteur 100").type == Type::Moteur);
  CHECK(analyser("m 0").type == Type::Moteur);
  CHECK(analyser("g 10").type == Type::Gauche && analyser("g 10").valeur == 10);
  CHECK(analyser("d 20").type == Type::Droite && analyser("d 20").valeur == 20);
  CHECK(analyser("sensg 1").type == Type::SensGauche);
  CHECK(analyser("sensd 0").type == Type::SensDroite);
  CHECK(analyser("inverseg 0").type == Type::InverseGauche);
  CHECK(analyser("inversed 1").type == Type::InverseDroite);
  CHECK(analyser("s -22.5").type == Type::Servo && analyser("s -22.5").valeur == -22.5f);
  CHECK(analyser("servo 0").type == Type::Servo);
  CHECK(analyser("  vitesse_servo\t30 \t").type == Type::Vitesse);
  CHECK(analyser("centre").type == Type::Centre);
  CHECK(analyser("stop \t").type == Type::Stop);
  CHECK(analyser("etat").type == Type::Etat && analyser("diag").type == Type::Etat);
  CHECK(analyser("sens 1").type == Type::Sens && analyser("frein 0").type == Type::Frein);
  CHECK(analyser("course 140").type == Type::Course);
  CHECK(analyser(" \t ").type == Type::Vide);
  for (const char *commande : {"10", "0", "90", "stop 1", "m -1", "m 101", "m 10.5", "m 10 extra",
       "s nan", "s inf", "s -181", "s 20oops", "v 0", "v -30", "v 181", "sens 2", "frein -1",
       "course 0", "course 180.5", "course 361", "m", "s", "v", "m 1e9999", "inconnue",
       "g -1", "d 101", "g 2.5", "sensg 2", "sensd -1", "inverseg 2", "inversed nan"}) {
    CHECK(analyser(commande).type == Type::Invalide);
  }

  // ROVER_3 : pilotage combine, chien de garde, inversion, telemetrie, centrale.
  const Commande p = analyser("p 12 -8 5.5");
  CHECK(p.type == Type::Pilotage && p.valeur == 12 && p.valeur2 == -8 && p.valeur3 == 5.5f);
  const Commande bornes = analyser("  p\t-100  100 \t-180 ");
  CHECK(bornes.type == Type::Pilotage && bornes.valeur == -100 && bornes.valeur2 == 100 &&
        bornes.valeur3 == -180);
  CHECK(analyser("p 0 0 0").type == Type::Pilotage && analyser("p -0 7 90").valeur2 == 7);
  CHECK(analyser("p 3 4 -0.1").valeur3 == -0.1f);
  CHECK(analyser("hb").type == Type::Battement && analyser(" hb \t").type == Type::Battement);
  CHECK(analyser("lacet0").type == Type::Lacet0);
  CHECK(analyser("wd 0").type == Type::Chien && analyser("wd 0").valeur == 0);
  CHECK(analyser("wd 100").valeur == 100 && analyser("wd 2000").type == Type::Chien);
  CHECK(analyser("inv 100").type == Type::Inversion && analyser("inv 3000").valeur == 3000);
  CHECK(analyser("tel 0").type == Type::Telemetrie && analyser("tel 50").valeur == 50);
  CHECK(analyser("imu 0").type == Type::Centrale && analyser("imu 100").valeur == 100);
  CHECK(analyser("inverseg 1").type == Type::InverseGauche); // inv ne masque pas inverseg
  CHECK(analyser("roues 1").type == Type::Roues && analyser("roues 1").valeur == 1);
  CHECK(analyser(" roues\t0 ").type == Type::Roues && analyser("roues 0").valeur == 0);
  CHECK(PINS_SC[0] == 25 && PINS_SC[1] == 7); // D2 gauche, D3 droite
  for (const char *commande : {"p", "p 1", "p 1 2", "p 1 2 3 4", "p 101 0 0", "p 0 -101 0",
       "p 1.5 0 0", "p 0 2.5 0", "p 0 0 181", "p 0 0 -180.5", "p 0 0 nan", "p inf 0 0",
       "p 12-8 5", "p 1,2,3", "p 1 2 3x", "p 1 2 x3", "p 1e9999 0 0", "hb 1", "lacet0 0",
       "wd", "wd 1", "wd 99", "wd 2001", "wd -1", "wd 100.5", "inv", "inv 0", "inv 99",
       "inv 3001", "inv 500.5", "tel", "tel -1", "tel 51", "tel 2.5", "imu", "imu -1",
       "imu 101", "imu 12.5", "P 1 2 3", "HB", "wd 400 1", "tel 20x",
       "roues", "roues 2", "roues -1", "roues 0.5", "roues 1 1", "roues x", "roue 1"}) {
    CHECK(analyser(commande).type == Type::Invalide);
  }

  ServoProgressif s;
  CHECK(!s.signal && s.impulsion() == 1500 && !s.enMouvement());
  CHECK(s.demander(0, 100) && s.signal && !s.enMouvement());
  CHECK(s.demander(60, 100));
  CHECK(s.duree >= 3000 && s.duree <= 3010); // Arrondi de 60 deg en impulsions entieres.
  const uint32_t duree = s.duree;
  s.actualiser(100 + duree / 2);
  CHECK(s.impulsion() >= 1733 && s.impulsion() <= 1734); // Milieu, arrondi a la ms/us pres.
  CHECK(s.enMouvement());
  s.actualiser(100 + duree);
  CHECK(s.impulsion() == 1967 && !s.enMouvement());
  CHECK(!s.demander(91, 3200) && s.impulsion() == 1967);
  CHECK(!s.demander(std::numeric_limits<float>::quiet_NaN(), 3200));

  CHECK(s.demander(-60, 4000));
  s.actualiser(5000);
  const int positionArret = s.impulsion();
  s.stop();
  s.actualiser(100000);
  CHECK(!s.signal && !s.enMouvement() && s.impulsion() == positionArret);
  CHECK(!s.reglerVitesse(0, 100001));
  CHECK(!s.reglerVitesse(std::numeric_limits<float>::infinity(), 100001));
  CHECK(s.reglerVitesse(60, 100002) && !s.signal); // v ne redemarre pas apres stop.
  CHECK(s.demander(0, 100003));
  CHECK(s.impulsion() == positionArret); // Nouvelle cible : pas de saut de position.
  s.actualiser(200000);
  CHECK(s.impulsion() == 1500);

  CHECK(s.reglerCourse(120, 200001));
  CHECK(s.courseRenseignee && s.angle(1500) == 0);
  CHECK(s.demander(60, 200002));
  s.actualiser(300000);
  CHECK(s.impulsion() == 2200);
  CHECK(!s.demander(61, 300001));
  CHECK(!s.reglerCourse(0, 300001));

  ServoProgressif limite;
  const uint32_t debut = UINT32_MAX - 1000;
  CHECK(limite.demander(60, debut));
  const uint32_t dureeLimite = limite.duree;
  limite.actualiser(debut + dureeLimite / 2);
  CHECK(limite.impulsion() >= 1733 && limite.impulsion() <= 1734); // Passage de millis a zero.
  limite.actualiser(debut + dureeLimite);
  CHECK(limite.impulsion() == 1967);

  // La consigne moteur persiste pendant un mouvement servo, et stop annule les deux.
  moteur::Policy moteur;
  ServoProgressif direction;
  CHECK(moteur.consigne(12, 0));
  CHECK(direction.demander(-45, 0));
  direction.actualiser(1000);
  CHECK(moteur.pwm(1000) == 12 && direction.enMouvement());
  moteur.stop("stop_operateur"); direction.stop();
  CHECK(moteur.pwm(1001) == 0 && !direction.signal);
  direction.actualiser(5000);
  CHECK(moteur.pwm(5000) == 0 && !direction.enMouvement());

  // Augmenter v pendant le trajet conserve la consigne courante et la cible.
  ServoProgressif vitesse;
  CHECK(vitesse.demander(90, 0));
  vitesse.actualiser(1000);
  const int avant = vitesse.impulsion();
  CHECK(vitesse.reglerVitesse(60, 1000) && vitesse.impulsion() == avant);
  CHECK(vitesse.cibleUs == 2200);
  vitesse.actualiser(100000);
  CHECK(vitesse.impulsion() == 2200 && !vitesse.enMouvement());
  std::printf("%d verifications combinees OK (sans acces USB).\n", verifications);
  return 0;
}
