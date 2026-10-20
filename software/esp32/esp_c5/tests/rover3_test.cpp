// Logique ROVER_3 sans materiel : chien de garde, passage par zero, formats T/I, lacet.
#include "../src/pilotage_rover3.h"
#include "../src/centrale_lacet.h"
#include "../src/roues_sc.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

static int verifications = 0;
#define CHECK(c) do { ++verifications; if (!(c)) { \
  std::fprintf(stderr, "ECHEC rover3 ligne %d : %s\n", __LINE__, #c); return 1; } } while (0)

static std::string ligneT(uint32_t ms, int g, int d, float servo, bool frein, bool defaut, bool chien,
                          const float *hz = nullptr) {
  char t[112];
  const int n = rover3::formaterTelemetrie(t, sizeof t, ms, g, d, servo, frein, defaut, chien, hz);
  return n > 0 ? std::string(t, static_cast<size_t>(n)) : std::string();
}
static int champs(const std::string &ligne) {
  int n = 0;
  bool dans = false;
  for (char c : ligne) {
    const bool espace = c == ' ' || c == '\n';
    if (!espace && !dans) ++n;
    dans = !espace;
  }
  return n;
}

int main() {
  using rover3::ChienDeGarde;
  using rover3::RoueSignee;

  // --- Chien de garde
  ChienDeGarde chien;
  CHECK(!chien.actif() && !chien.ok() && !chien.echu(1000000) && !chien.verifier(1000000));
  CHECK(std::strcmp(chien.nom(), "coupe") == 0);
  CHECK(!chien.regler(99, 0) && !chien.regler(2001, 0) && !chien.actif());
  CHECK(chien.regler(400, 1000) && chien.ok() && std::strcmp(chien.nom(), "ok") == 0);
  CHECK(!chien.verifier(1399));                 // 399 ms : encore valide
  chien.renouveler(1300);                       // p ou hb accepte
  CHECK(!chien.verifier(1699) && chien.ok());
  CHECK(chien.verifier(1700) && !chien.ok());   // 400 ms apres le dernier renouvellement
  CHECK(std::strcmp(chien.nom(), "expire") == 0);
  CHECK(!chien.verifier(1701) && !chien.verifier(5000)); // une seule fois
  chien.renouveler(5000);
  CHECK(chien.ok() && !chien.verifier(5399) && chien.verifier(5400));
  CHECK(chien.regler(0, 6000) && !chien.actif() && !chien.verifier(100000));
  ChienDeGarde limite;                          // passage de millis par zero
  CHECK(limite.regler(100, UINT32_MAX - 50));
  CHECK(!limite.verifier(UINT32_MAX) && !limite.verifier(48) && limite.verifier(49));

  // --- Consigne signee : passage par zero et attente inv, autre roue non affectee
  moteur::Policy g, d;
  RoueSignee rg, rd;
  bool sens = false, refus = false;
  const uint32_t inv = 500;
  CHECK(rg.demander(g, 10, inv, 2000, sens) && !sens && !g.direction && g.demande == 10);
  CHECK(rd.demander(d, 20, inv, 2000, sens) && !sens);
  CHECK(g.pwm(2100) == 10 && d.pwm(2100) == 20);
  rg.noter(g, 2100); rd.noter(d, 2100);
  // Inversion gauche : zero immediat, D/R inchange, droite intacte.
  CHECK(rg.demander(g, -15, inv, 2200, sens) && !sens);
  CHECK(g.demande == 0 && g.pwm(2200) == 0 && !g.direction && rg.enAttente);
  CHECK(d.pwm(2200) == 20);
  CHECK(rg.demander(g, -12, inv, 2300, sens) && !sens && rg.valeurVisee == 12); // valeur mise a jour
  CHECK(!rg.actualiser(g, inv, 2699, refus) && !refus && !g.direction && g.pwm(2699) == 0);
  CHECK(rg.actualiser(g, inv, 2700, refus) && !refus && g.direction && !rg.enAttente);
  CHECK(g.demande == 12 && g.pwm(2700) == 0);   // preparation de 100 ms apres le D/R
  CHECK(g.pwm(2800) == 12 && d.pwm(2800) == 20);
  // Retour dans l'ancien sens pendant l'attente : pas de changement de D/R, valeur directe.
  CHECK(rg.demander(g, 8, inv, 3000, sens) && rg.enAttente && g.demande == 0);
  CHECK(rg.demander(g, -9, inv, 3100, sens) && !rg.enAttente && !sens && g.direction && g.demande == 9);
  // Roue a zero depuis assez longtemps : changement de sens immediat.
  CHECK(rg.demander(g, 0, inv, 3200, sens) && g.demande == 0 && !rg.enAttente);
  rg.noter(g, 3300);
  CHECK(rg.demander(g, 5, inv, 3699, sens) && rg.enAttente && !sens); // 499 ms seulement
  CHECK(rg.demander(g, 5, inv, 3700, sens) && sens && !g.direction && g.demande == 5 && !rg.enAttente);
  // Zero pendant l'attente : la roue reste a zero, rien ne repart.
  CHECK(rg.demander(g, -5, inv, 4000, sens) && rg.enAttente);
  CHECK(rg.demander(g, 0, inv, 4100, sens) && !rg.enAttente);
  CHECK(!rg.actualiser(g, inv, 9000, refus) && g.demande == 0 && !g.direction);
  // Frein ou defaut : refus, roue a zero, aucune inversion en attente.
  moteur::Policy f;
  RoueSignee rf;
  f.reglerFrein(true);
  CHECK(rf.demander(f, 0, inv, 10000, sens));
  CHECK(!rf.demander(f, 10, inv, 10000, sens) && f.demande == 0);
  CHECK(!rf.demander(f, -10, inv, 10000, sens) && !rf.enAttente && !f.direction);
  moteur::Policy pf;
  RoueSignee rpf;
  CHECK(rpf.demander(pf, 10, inv, 0, sens));
  CHECK(rpf.demander(pf, -10, inv, 100, sens) && rpf.enAttente);
  pf.reglerFrein(true);                          // frein pendant l'attente
  CHECK(!rpf.actualiser(pf, inv, 700, refus) && refus && !pf.direction && pf.demande == 0);

  // --- Formats
  CHECK(ligneT(123456, 12, -8, 5.5f, false, false, true) == "T 123456 12 -8 5.5 0 0 1\n");
  CHECK(ligneT(0, 0, 0, -30.04f, true, true, false) == "T 0 0 0 -30.0 1 1 0\n");
  CHECK(ligneT(4294967295u, -100, 100, 90, false, false, false) == "T 4294967295 -100 100 90.0 0 0 0\n");
  CHECK(champs(ligneT(123456, 12, -8, 5.5f, false, false, true)) == 8);
  {
    const float hz[2] = {45.04f, -44.46f};
    const std::string t10 = ligneT(1000, 10, -10, 0, false, false, true, hz);
    CHECK(t10 == "T 1000 10 -10 0.0 0 0 1 45.0 -44.5\n" && champs(t10) == 10);
    const float zero[2] = {0, roues_sc::signe(0, true)};
    CHECK(ligneT(5, 0, 0, 0, false, false, false, zero) == "T 5 0 0 0.0 0 0 0 0.0 0.0\n");
  }
  CHECK(rover3::pwmSigne(10, true) == -10 && rover3::pwmSigne(10, false) == 10);
  CHECK(rover3::pwmSigne(-1, true) == 0 && rover3::pwmSigne(0, true) == 0);
  {
    char t[112];
    const float a[3] = {0.0123f, -0.5f, 1.0f}, w[3] = {1.234f, -0.006f, 250.0f};
    const int n = rover3::formaterCentrale(t, sizeof t, 42, a, w, -179.996f);
    CHECK(n > 0 && std::string(t) == "I 42 0.012 -0.500 1.000 1.23 -0.01 250.00 -180.00\n");
  }
  CHECK(rover3::periodeMs(20) == 50 && rover3::periodeMs(30) == 33 && rover3::periodeMs(7) == 142);
  CHECK(rover3::periodeMs(100) == 10 && rover3::periodeMs(0) == 0);
  rover3::Cadence cadence;
  CHECK(!cadence.due(0));
  cadence.regler(20, 1000);
  int lignes = 0;
  for (uint32_t t = 1000; t < 2000; ++t) lignes += cadence.due(t);
  CHECK(lignes == 20);
  CHECK(cadence.due(2001) && cadence.due(5000) && !cadence.due(5049) && cadence.due(5050)); // recalage

  // --- Lacet : biais estime a l'arret, lacet fige a l'arret, integration en mouvement
  centrale::Lacet lacet;
  const float dt = 0.02f, biais = 1.5f;
  for (int i = 0; i < 250; ++i)                  // 5 s a l'arret, bruit faible
    lacet.ajouter(0.1f * ((i % 3) - 1), 0.05f, biais + 0.05f * ((i % 2) ? 1 : -1), i ? dt : 0, true);
  CHECK(lacet.immobile && std::fabs(lacet.biais - biais) < 0.05f && lacet.lacet == 0);
  for (int i = 0; i < 100; ++i) lacet.ajouter(0, 0, biais + 10.0f, dt, false); // 2 s a 10 deg/s
  CHECK(!lacet.immobile && std::fabs(lacet.lacet - 20.0f) < 0.3f);
  for (int i = 0; i < 50; ++i) lacet.ajouter(0, 0, biais, dt, true);          // le gyroscope se calme
  CHECK(lacet.immobile && std::fabs(lacet.lacet - 20.0f) < 0.3f);
  const float avant = lacet.lacet;
  for (int i = 0; i < 50; ++i) lacet.ajouter(0, 0, biais, dt, true);          // arret : fige
  CHECK(lacet.lacet == avant && std::fabs(lacet.biais - biais) < 0.05f);
  // Traction au repos mais gyroscope agite : pas d'estimation du biais, integration.
  const float biaisAvant = lacet.biais;
  lacet.ajouter(0, 0, biais + 30.0f, dt, true);
  CHECK(!lacet.immobile && lacet.biais == biaisAvant && lacet.lacet > avant);
  // Trou de mesures > 0,25 s : pas d'integration.
  const float avantTrou = lacet.lacet;
  lacet.ajouter(0, 0, biais + 30.0f, 0.3f, false);
  CHECK(lacet.lacet == avantTrou);
  // Rotation reguliere a la main, traction au repos : integree, pas prise pour un biais.
  for (int i = 0; i < 50; ++i) lacet.ajouter(0, 0, biais, dt, true);
  const float avantMain = lacet.lacet;
  for (int i = 0; i < 100; ++i) lacet.ajouter(0, 0, biais + 10.0f, dt, true);  // 2 s a 10 deg/s
  CHECK(std::fabs(lacet.lacet - avantMain - 20.0f) < 0.5f && std::fabs(lacet.biais - biais) < 0.05f);
  // Ecart stable au biais pendant 5 s : estimation relancee (biais faux au demarrage).
  centrale::Lacet faux;
  for (int i = 0; i < 150; ++i) faux.ajouter(0, 0, 4.0f, i ? dt : 0, true);    // biais appris : 4
  CHECK(std::fabs(faux.biais - 4.0f) < 0.01f && faux.tempsBiais >= 2.0f);
  for (int i = 0; i < 400; ++i) faux.ajouter(0, 0, 0.5f, dt, true);           // vrai biais : 0,5
  CHECK(std::fabs(faux.biais - 0.5f) < 0.05f && faux.immobile);
  lacet.remettreAZero();
  CHECK(lacet.lacet == 0);
  for (int i = 0; i < 1000; ++i) lacet.ajouter(0, 0, biais - 20.0f, dt, false); // -400 deg
  CHECK(lacet.lacet >= -180.0f && lacet.lacet < 180.0f && std::fabs(lacet.lacet - (-40.0f)) < 1.0f);
  CHECK(centrale::Lacet::borner(180.0f) == -180.0f && centrale::Lacet::borner(-190.0f) == 170.0f);

  // --- Sorties SC : anti-rebond, frequence lissee, signe
  roues_sc::Compteur sc;
  sc.front(1000);
  CHECK(sc.fronts == 1 && sc.rejets == 0 && sc.periodeUs == 0);
  sc.front(1100);                                  // 100 us : rebond rejete
  CHECK(sc.fronts == 1 && sc.rejets == 1 && sc.dernierUs == 1000);
  sc.front(1199);                                  // 199 us apres le front accepte : rejete
  sc.front(1200);                                  // 200 us : accepte
  CHECK(sc.fronts == 2 && sc.rejets == 2 && sc.periodeUs == 200 && sc.dernierUs == 1200);
  roues_sc::Compteur bord;                         // passage de micros par zero
  bord.front(UINT32_MAX - 99);
  bord.front(50);                                  // 150 us : rejete
  bord.front(4900);                                // 5000 us : accepte
  CHECK(bord.fronts == 2 && bord.rejets == 1 && bord.periodeUs == 5000);
  sc.remettreAZero();
  CHECK(sc.fronts == 0 && sc.rejets == 0 && sc.releve().dernierUs == 0);

  // 200 impulsions/s regulieres, mesure toutes les 100 ms.
  roues_sc::Compteur compteur;
  roues_sc::Frequence frequence;
  uint32_t us = 0;
  for (int fenetre = 1; fenetre <= 10; ++fenetre) {
    for (; us < static_cast<uint32_t>(fenetre) * 100000u; us += 5000) compteur.front(us + 1000000u);
    frequence.actualiser(compteur.releve(), fenetre * 100000u + 1000000u);
    if (fenetre >= 2) CHECK(frequence.hz == 200.0f);
  }
  CHECK(compteur.fronts == 200 && compteur.rejets == 0);
  // Arret : borne 1/age, puis 0 apres 0,5 s sans front.
  const uint32_t dernier = compteur.dernierUs;
  frequence.actualiser(compteur.releve(), dernier + 100000);
  CHECK(std::fabs(frequence.hz - 10.0f) < 1e-3f);
  frequence.actualiser(compteur.releve(), dernier + 400000);
  CHECK(std::fabs(frequence.hz - 2.5f) < 1e-3f);
  frequence.actualiser(compteur.releve(), dernier + 500001);
  CHECK(frequence.hz == 0);
  // Redemarrage : un seul front, pas de periode ; deux fronts, periode instantanee.
  const uint32_t reprise = dernier + 2000000;
  compteur.front(reprise);
  frequence.actualiser(compteur.releve(), reprise + 1000);
  CHECK(frequence.hz == 0);
  compteur.front(reprise + 20000);
  compteur.front(reprise + 40000);
  frequence.actualiser(compteur.releve(), reprise + 100000);
  CHECK(frequence.hz == 50.0f);
  roues_sc::Frequence neuve;                       // jamais de front
  neuve.actualiser(roues_sc::Releve{}, 123456);
  CHECK(neuve.hz == 0);
  CHECK(roues_sc::signe(12.5f, true) == -12.5f && roues_sc::signe(12.5f, false) == 12.5f);
  CHECK(roues_sc::signe(0, true) == 0 && !std::signbit(roues_sc::signe(0, true)));

  std::printf("%d verifications ROVER_3 OK (chien, inversion, formats, lacet, roues SC).\n", verifications);
  return 0;
}
