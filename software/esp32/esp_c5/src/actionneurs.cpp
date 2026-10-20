// Controle commun traction + servo : programme principal et test manuel.
// Servo D7/GPIO12 ; traction D0/GPIO1 et D9/GPIO9 ; DIR D1/D4 ; BRAKE D10.
// Commandes : m 10, s -30, v 30, centre, stop, etat, aide.
// ROVER_3 (pilotage par le Pi) : p g d servo, hb, wd ms, inv ms, tel hz, imu hz, lacet0.
// Facultatif : roues 1 lit les sorties SC des drivers sur D2 (gauche) et D3 (droite).
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <atomic>
#include <cmath>
#include "console_serie.h"
#include "moteur_sorties.h"
#include "moteur_policy.h"
#include "pilotage_rover3.h"
#include "roues_sc.h"
#include "test_tout_moteur_logique.h"
#include "test_tout_moteur_pwm.h"
#include "test_tout_moteur_broches.h"
#include "actionneurs.h"
#ifdef FIELDMIND_ROVER_COMPLET
#include "drone_wifi.h"
#endif

namespace actionneurs {
#ifdef FIELDMIND_ROVER_COMPLET
constexpr const char *VERSION = "ROVER_3";
constexpr bool CENTRALE_PRESENTE = true; // IMU lue par main.cpp
#else
constexpr const char *VERSION = "TOUT_MOTEUR_4";
constexpr bool CENTRALE_PRESENTE = false;
#endif

using test_actionneurs::Type;
constexpr uint8_t PIN_SERVO = test_actionneurs::PIN_SERVO;
constexpr uint32_t MOTEUR_HZ = 5000;
constexpr uint8_t MOTEUR_BITS = 10;
// ESP32Servo est limite aux canaux 0/1 ; traction sur 4/5, sans collision.
constexpr uint8_t CANAUX_MOTEUR[] = {4, 5};
static_assert(moteur::NOMBRE_MOTEURS == 2, "Compiler avec FIELDMIND_DUAL_MOTOR");
static_assert(PIN_SERVO == D7 && test_actionneurs::PINS_DIR[0] == D1 &&
              test_actionneurs::PINS_DIR[1] == D4 &&
              moteur::PIN_PWM == D0 && moteur::PINS_PWM[1] == D9 &&
              test_actionneurs::PINS_SC[0] == D2 && test_actionneurs::PINS_SC[1] == D3,
              "Verifier les broches XIAO ESP32-C5");

Servo servo;
test_actionneurs::ServoProgressif position;
moteur::Policy traction[2];
Preferences reglages;
bool memoireDisponible = false;
bool inverse[2] = {test_actionneurs::SENS_INVERSE[0], test_actionneurs::SENS_INVERSE[1]};
// Nouvelle reference droite : ignorer uniquement son ancien reglage inv_d.
// Le reglage gauche reste conserve ; les prochaines corrections restent memorisees.
constexpr const char *CLES_INVERSION[] = {"inv_g", "inv_d_v2"};
bool pwmAttache[2] = {}, defaut = false, accueil = false, sessionUsb = false;
bool jeterLigne = false;
const char *raisonDefaut = "aucun";
int brocheDefaut = -1;
uint32_t frequenceDefaut = 0, dutyDefaut = 0;
uint32_t dernierUsb = 0, dernierServo = 0;
int dernierPourcent[2] = {-1, -1};
char ligne[96] = {};
size_t longueur = 0;

// ROVER_3 : coupes au demarrage et a chaque perte USB detectee (sauf inv).
rover3::ChienDeGarde chien;
rover3::RoueSignee roues[2];
rover3::Cadence telemetrie;
uint32_t delaiInversion = rover3::INVERSION_DEFAUT_MS;
std::atomic<uint32_t> cadenceImu{0}, dernierMouvement{0};
std::atomic<bool> remiseLacet{false};

// Sorties SC facultatives (roues 1) : coupees au demarrage, broches non configurees.
bool rouesActives = false;
roues_sc::Compteur compteursSC[2];
roues_sc::Frequence frequencesSC[2];
uint32_t derniereMesureSC = 0;
#if defined(ARDUINO_ARCH_ESP32)
portMUX_TYPE verrouSC = portMUX_INITIALIZER_UNLOCKED;
#define SC_ENTRER() portENTER_CRITICAL(&verrouSC)
#define SC_SORTIR() portEXIT_CRITICAL(&verrouSC)
#define SC_ENTRER_ISR() portENTER_CRITICAL_ISR(&verrouSC)
#define SC_SORTIR_ISR() portEXIT_CRITICAL_ISR(&verrouSC)
#else
#define SC_ENTRER()
#define SC_SORTIR()
#define SC_ENTRER_ISR()
#define SC_SORTIR_ISR()
#endif
void IRAM_ATTR frontSCGauche() { SC_ENTRER_ISR(); compteursSC[0].front(micros()); SC_SORTIR_ISR(); }
void IRAM_ATTR frontSCDroite() { SC_ENTRER_ISR(); compteursSC[1].front(micros()); SC_SORTIR_ISR(); }
// Impulsions/s signees du sens logique applique a la roue (la sortie SC ne donne pas le sens).
float hzSigne(size_t roue) { return roues_sc::signe(frequencesSC[roue].hz, traction[roue].direction); }

void niveauBas(uint8_t pin) {
  gpio_set_level(static_cast<gpio_num_t>(pin), 0);
  pinMode(pin, OUTPUT);
}

void appliquerSens() {
  for (size_t i = 0; i < moteur::NOMBRE_MOTEURS; ++i)
    digitalWrite(test_actionneurs::PINS_DIR[i],
                 (traction[i].direction != inverse[i]) ? HIGH : LOW);
}

bool ecrireTraction(uint32_t duty) {
  return moteur::ecrireGroupePwm(moteur::PINS_PWM, duty, [](uint8_t pin, uint32_t valeur) {
    return ledcWrite(pin, valeur);
  });
}

void annulerInversions(unsigned masque = 3) {
  for (size_t i = 0; i < 2; ++i)
    if (masque & (1u << i)) roues[i].annuler();
}

void couperServo() {
  position.stop();
  if (servo.attached()) servo.detach();
  niveauBas(PIN_SERVO);
}

void annoncerDefaut() {
  console::printf("[DEFAUT] Cause=%s ; GPIO=%d ; frequence avant coupure=%lu Hz ; duty lu=%lu.\n",
                  raisonDefaut, brocheDefaut, static_cast<unsigned long>(frequenceDefaut),
                  static_cast<unsigned long>(dutyDefaut));
}

void verrouillerDefaut(const char *raison, int pin = -1) {
  const bool premierDefaut = !defaut;
  if (premierDefaut) {
    raisonDefaut = raison;
    brocheDefaut = pin;
    if (pin >= 0) {
      frequenceDefaut = test_actionneurs::frequencePwm(static_cast<uint8_t>(pin));
      dutyDefaut = ledcRead(static_cast<uint8_t>(pin));
    }
  }
  defaut = true;
  annulerInversions();
  for (auto &controle : traction) controle.panne("defaut_sorties");
  for (size_t i = 0; i < moteur::NOMBRE_MOTEURS; ++i) {
    if (pwmAttache[i]) { ledcWrite(moteur::PINS_PWM[i], 0); ledcDetach(moteur::PINS_PWM[i]); }
    pwmAttache[i] = false;
    niveauBas(moteur::PINS_PWM[i]);
  }
  couperServo();
  if (premierDefaut && Serial) annoncerDefaut();
}

void arreterTout(const char *raison) {
  annulerInversions();
  for (auto &controle : traction) controle.stop(raison);
  if (!defaut && pwmAttache[0] && pwmAttache[1] && !ecrireTraction(0)) {
    verrouillerDefaut("arret_ecriture_moteurs");
  }
  dernierPourcent[0] = dernierPourcent[1] = 0;
  couperServo();
}

bool connecterServo() {
  if (defaut) return false;
  if (!servo.attached()) {
    servo.setPeriodHertz(50);
    servo.attach(PIN_SERVO, 800, 2200);
    servo.setTimerWidth(14); // Apres attach() pour ESP32Servo 3.2.1.
  }
  servo.writeMicroseconds(position.impulsion());
  if (!servo.attached()) {
    verrouillerDefaut("servo_non_attache", PIN_SERVO);
    return false;
  }
  if (!test_actionneurs::frequenceValide(test_actionneurs::frequencePwm(PIN_SERVO), 50)) {
    verrouillerDefaut("initialisation_frequence_servo", PIN_SERVO);
    return false;
  }
  return true;
}

void actualiserSorties(uint32_t maintenant) {
  if (defaut) return;
  for (size_t i = 0; i < moteur::NOMBRE_MOTEURS; ++i) {
    const int pourcent = traction[i].pwm(maintenant);
    const uint8_t pin = moteur::PINS_PWM[i];
    if (pourcent != dernierPourcent[i]) {
      if (!ledcWrite(pin, static_cast<uint32_t>(pourcent) * 1023 / 100)) {
        verrouillerDefaut("ecriture_moteur", pin); return;
      }
      dernierPourcent[i] = pourcent;
    }
    if (pourcent > 0 &&
        !test_actionneurs::frequenceValide(test_actionneurs::frequencePwm(pin), MOTEUR_HZ)) {
      verrouillerDefaut("frequence_moteur", pin); return;
    }
  }
  position.actualiser(maintenant);
  if (position.signal && maintenant - dernierServo >= 20) {
    dernierServo = maintenant;
    if (!servo.attached()) { verrouillerDefaut("servo_detache", PIN_SERVO); return; }
    servo.writeMicroseconds(position.impulsion());
    if (!test_actionneurs::frequenceValide(test_actionneurs::frequencePwm(PIN_SERVO), 50)) {
      verrouillerDefaut("frequence_servo", PIN_SERVO); return;
    }
  }
}

void aide() {
  console::printf("=== FieldMind %s - traction + servo D7 ===\n", VERSION);
  console::printf("Version %s - " __DATE__ " " __TIME__ "\n", VERSION);
  console::println("Servo au centre (1500 us) au demarrage ; traction a zero.");
  console::println("Cablage : servo D7 ; driver D0 DIR=D1 ; driver D9 DIR=D4 ; BRAKE commun D10.");
  console::println("m 10       : 10 % PWM sur les deux drivers, maintenu jusqu'a m 0 ou stop");
  console::println("m 0        : traction a zero, servo toujours maintenu");
  console::println("g 10 / d 20 : PWM gauche / droite independant, 0..100 %");
  console::println("sensg 0/1 ; sensd 0/1 : avant/arriere par roue, a l'arret de cette roue");
  console::println("inverseg 0/1 ; inversed 0/1 : corriger l'avant, deux roues a zero ; memorise");
  console::println("s -30      : position servo relative ; 0 = milieu ; echelle -90..+90 indicative");
  console::println("v 30       : vitesse de consigne servo, 1..180 degres indicatifs/seconde");
  console::println("centre     : servo au milieu ; stop : PWM moteur zero ET signal servo coupe");
  console::println("sens 0 / 1 : sens commun corrige pour montage miroir, apres m 0 et arret mecanique");
  console::println("frein 0 / 1: PWM zero puis niveau BRAKE commun ; polarite physique a verifier");
  console::println("course 140 : uniquement si 140 degres de course totale ont ete mesures");
  console::println("p 12 -8 5.5 : PWM signes gauche/droite -100..100 (signe = sens) + angle servo ; silencieux");
  console::println("inv 500    : roue a zero 100..3000 ms avant qu'un p change son sens");
  console::println("wd 400     : chien de garde 100..2000 ms ; sans p ni hb, traction a zero ; wd 0 coupe");
  console::println("hb         : renouvelle le chien de garde, sans reponse");
  console::println("tel 20     : ligne T ms g d servo frein defaut chien, 0..50 Hz (0 = coupee)");
  if (CENTRALE_PRESENTE) {
    console::println("imu 50     : ligne I ms ax ay az gx gy gz lacet, 0..100 Hz (0 = ligne IMU texte)");
    console::println("lacet0     : remettre le lacet integre a zero");
  } else {
    console::println("imu / lacet0 : centrale absente de ce programme (main esp_c5 seulement)");
  }
  console::println("roues 1/0  : lire les sorties SC des drivers sur D2 (gauche) / D3 (droite) ; T + hz_g hz_d");
  console::println("etat / aide : afficher les consignes / les commandes");
  console::println("Chaque ligne finit par Entree. STOP du driver n'est pas cable par ce code.");
}

void etat() {
  console::printf("[FIRMWARE] %s\n", VERSION);
  console::printf("[MOTEURS] G=%u %% ; D=%u %% ; BRAKE=%u ; defaut=%u\n",
                  traction[0].demande, traction[1].demande, traction[0].frein, defaut);
  for (size_t i = 0; i < 2; ++i)
    console::printf("[%s] Demande=%u %% ; PWM=%d %% ; sens=%u ; DIR=%u ; inverse=%u\n",
                    i == 0 ? "GAUCHE" : "DROITE", traction[i].demande, dernierPourcent[i],
                    traction[i].direction, traction[i].direction != inverse[i], inverse[i]);
  console::printf("[REGLAGES] Inversions memorisables=%u ; DIR gauche D1 / droite D4 ; servo D7.\n", memoireDisponible);
  console::printf("[SERVO] Signal %s ; consigne en cours %+.1f ; cible %+.1f ; vitesse max %.1f deg/s\n",
                  position.signal ? "actif" : "coupe", position.angle(position.positionUs),
                  position.angle(position.cibleUs), position.vitesse);
  console::printf("        Plage 800..2200 us ; angles +/-%.1f (%s). Position physique non mesuree.\n",
                  position.course / 2.0f, position.courseRenseignee ? "course renseignee" : "INDICATIFS");
  console::printf("[PWM] Timers : servo D7=%lu Hz ; moteurs D0=%lu / D9=%lu Hz ; lecture independante du duty.\n",
                  static_cast<unsigned long>(test_actionneurs::frequencePwm(PIN_SERVO)),
                  static_cast<unsigned long>(test_actionneurs::frequencePwm(D0)),
                  static_cast<unsigned long>(test_actionneurs::frequencePwm(D9)));
  char imu[16] = "absente";
  if (CENTRALE_PRESENTE) std::snprintf(imu, sizeof imu, "%lu Hz", static_cast<unsigned long>(cadenceImu.load()));
  console::printf("[PILOTAGE] Chien=%lu ms (%s) ; inv=%lu ms ; tel=%lu Hz ; imu=%s ; lignes perdues=%lu\n",
                  static_cast<unsigned long>(chien.delai), chien.nom(),
                  static_cast<unsigned long>(delaiInversion), static_cast<unsigned long>(telemetrie.hz),
                  imu, static_cast<unsigned long>(console::perdues()));
  if (rouesActives) {
    roues_sc::Releve r[2];
    SC_ENTRER();
    r[0] = compteursSC[0].releve();
    r[1] = compteursSC[1].releve();
    SC_SORTIR();
    console::printf("[ROUES] SC gauche D2/GPIO25 : fronts=%lu rejets=%lu %.1f imp/s ; "
                    "droite D3/GPIO7 : fronts=%lu rejets=%lu %.1f imp/s\n",
                    static_cast<unsigned long>(r[0].fronts), static_cast<unsigned long>(r[0].rejets), hzSigne(0),
                    static_cast<unsigned long>(r[1].fronts), static_cast<unsigned long>(r[1].rejets), hzSigne(1));
  }
  if (defaut) annoncerDefaut();
}

void erreur(const char *message) {
  arreterTout("commande_invalide");
  console::printf("[ERREUR] %s Traction a zero, signal servo coupe.\n", message);
}

// roues 1 : D2/D3 en entree, interruptions sur front montant, compteurs remis a zero.
// roues 0 : interruptions retirees, D2/D3 en entree haute impedance.
void reglerRoues(bool actif) {
  if (rouesActives)
    for (uint8_t pin : test_actionneurs::PINS_SC) detachInterrupt(digitalPinToInterrupt(pin));
  for (uint8_t pin : test_actionneurs::PINS_SC) pinMode(pin, INPUT);
  SC_ENTRER();
  for (auto &compteur : compteursSC) compteur.remettreAZero();
  SC_SORTIR();
  for (auto &frequence : frequencesSC) frequence.remettreAZero();
  rouesActives = actif;
  derniereMesureSC = millis();
  if (actif) {
    attachInterrupt(digitalPinToInterrupt(test_actionneurs::PINS_SC[0]), frontSCGauche, RISING);
    attachInterrupt(digitalPinToInterrupt(test_actionneurs::PINS_SC[1]), frontSCDroite, RISING);
  }
}

// Frequences lissees sur environ 100 ms, a partir d'un releve coherent des deux compteurs.
void mesurerRoues(uint32_t maintenant) {
  if (!rouesActives || maintenant - derniereMesureSC < roues_sc::MESURE_MS) return;
  derniereMesureSC = maintenant;
  roues_sc::Releve releves[2];
  SC_ENTRER();
  releves[0] = compteursSC[0].releve();
  releves[1] = compteursSC[1].releve();
  const uint32_t us = micros();
  SC_SORTIR();
  for (size_t i = 0; i < 2; ++i) frequencesSC[i].actualiser(releves[i], us);
}

void envoyerTelemetrie(uint32_t maintenant) {
  char texte[112];
  const int g = defaut ? 0 : rover3::pwmSigne(dernierPourcent[0], traction[0].direction);
  const int d = defaut ? 0 : rover3::pwmSigne(dernierPourcent[1], traction[1].direction);
  const float hz[2] = {hzSigne(0), hzSigne(1)};
  const int n = rover3::formaterTelemetrie(texte, sizeof texte, maintenant, g, d,
                                           position.angle(position.positionUs), traction[0].frein,
                                           defaut, chien.ok(), rouesActives ? hz : nullptr);
  if (n > 0 && static_cast<size_t>(n) < sizeof texte) console::ecrire(texte, static_cast<size_t>(n));
}

// Chien de garde, a chaque tour de la tache, independamment de la reception.
void verifierChien(uint32_t maintenant) {
  if (!chien.echu(maintenant)) return;
  const bool premier = chien.verifier(maintenant);
  if (premier || traction[0].demande || traction[1].demande || roues[0].enAttente || roues[1].enAttente) {
    annulerInversions();
    for (auto &controle : traction) controle.stop("chien_expire"); // comme m 0 : servo maintenu
    actualiserSorties(maintenant);
  }
  if (premier) console::println("[CHIEN] expire : traction a zero, servo maintenu ; envoyer p pour reprendre.");
}

// Inversions demandees par p : D/R change apres inv ms roue a zero, sans bloquer.
void actualiserInversions(uint32_t maintenant) {
  bool sensChange = false;
  for (size_t i = 0; i < 2; ++i) {
    bool refus = false;
    if (roues[i].actualiser(traction[i], delaiInversion, maintenant, refus)) sensChange = true;
    if (refus) {
      if (sensChange) appliquerSens();
      erreur("Commande moteur refusee : verifier frein et etat.");
      return;
    }
  }
  if (sensChange) appliquerSens(); // PWM encore nul : phase de preparation de 100 ms.
}

void noterMouvement(uint32_t maintenant) {
  if (traction[0].demande || traction[1].demande) dernierMouvement.store(maintenant);
}

void pilotage(const test_actionneurs::Commande &commande, uint32_t maintenant) {
  const int cible[2] = {static_cast<int>(commande.valeur), static_cast<int>(commande.valeur2)};
  const float angle = commande.valeur3;
  int impulsion = 0;
  if (!servo_angles::convertir(angle, position.course, test_actionneurs::ServoProgressif::AMPLITUDE_US,
                               impulsion)) {
    erreur("Angle hors de la course actuelle. Tape etat."); return;
  }
  if ((cible[0] || cible[1]) && (traction[0].frein || traction[1].frein)) {
    erreur("Commande moteur refusee : verifier frein et etat."); return;
  }
  bool sensChange = false;
  for (size_t i = 0; i < 2; ++i) {
    bool change = false;
    const bool accepte = roues[i].demander(traction[i], cible[i], delaiInversion, maintenant, change);
    sensChange = sensChange || change;
    if (!accepte) {
      if (sensChange) appliquerSens();
      erreur("Commande moteur refusee : verifier frein et etat."); return;
    }
  }
  if (sensChange) appliquerSens();
  chien.renouveler(maintenant);
  // Meme cible : ne pas relancer la trajectoire (elle repartirait de vitesse nulle).
  if (!position.signal || impulsion != static_cast<int>(std::lround(position.cibleUs))) {
    position.demander(angle, maintenant);
    if (!connecterServo()) { console::println("[DEFAUT] Initialisation servo impossible."); return; }
  }
  actualiserSorties(maintenant);
}

// Reglages ROVER_3, acceptes aussi en defaut. Vrai si la commande a ete traitee.
bool reglerRover3(const test_actionneurs::Commande &commande, uint32_t maintenant) {
  const auto valeur = static_cast<uint32_t>(commande.valeur);
  switch (commande.type) {
    case Type::Battement: chien.renouveler(maintenant); return true;
    case Type::Chien:
      chien.regler(valeur, maintenant);
      if (chien.actif()) console::printf("[CHIEN] Delai %lu ms ; renouvele par p ou hb, sinon traction a zero.\n",
                                         static_cast<unsigned long>(valeur));
      else console::println("[CHIEN] Coupe.");
      return true;
    case Type::Inversion:
      delaiInversion = valeur;
      console::printf("[PILOTAGE] Inversion : roue a zero %lu ms avant changement de D/R.\n",
                      static_cast<unsigned long>(valeur));
      return true;
    case Type::Telemetrie:
      telemetrie.regler(valeur, maintenant);
      if (valeur) console::printf("[PILOTAGE] Telemetrie T a %lu Hz%s.\n", static_cast<unsigned long>(valeur),
#ifdef FIELDMIND_ROVER_COMPLET
                                  " ; etat periodique suspendu"
#else
                                  ""
#endif
        );
      else console::println("[PILOTAGE] Telemetrie coupee.");
      return true;
    case Type::Roues:
      reglerRoues(commande.valeur != 0);
      if (rouesActives) console::println("[ROUES] Lecture SC active : gauche D2/GPIO25, droite D3/GPIO7, "
                                         "fronts montants ; T + hz_g hz_d ; compteurs a zero.");
      else console::println("[ROUES] Lecture SC coupee ; D2/D3 en entree haute impedance.");
      return true;
    case Type::Centrale: case Type::Lacet0:
      if (!CENTRALE_PRESENTE) { console::println("[CENTRALE] Absente de ce programme ; commande ignoree."); return true; }
      if (commande.type == Type::Lacet0) {
        remiseLacet.store(true);
        console::println("[CENTRALE] Lacet remis a zero.");
      } else {
        cadenceImu.store(valeur);
        if (valeur) console::printf("[CENTRALE] Lignes I a %lu Hz ; ligne IMU texte suspendue.\n",
                                    static_cast<unsigned long>(valeur));
        else console::println("[CENTRALE] Ligne IMU texte toutes les 500 ms.");
      }
      return true;
    default: return false;
  }
}

void executer(const char *texte) {
#ifdef FIELDMIND_ROVER_COMPLET
  if (drone::traiterCommande(texte)) return;
#endif
  const auto commande = test_actionneurs::analyser(texte);
  const uint32_t maintenant = millis();
  if (commande.type == Type::Vide) return;
  if (commande.type == Type::Stop) {
    arreterTout("stop_operateur");
    console::println("[STOP] PWM moteur zero ; signal servo coupe. Alimentation non coupee."); return;
  }
  if (commande.type == Type::Etat) { etat(); return; }
  if (commande.type == Type::Aide) { aide(); return; }
  if (commande.type == Type::Invalide) { erreur("Commande invalide. Exemples : m 10, s -30, v 30, stop."); return; }
  if (reglerRover3(commande, maintenant)) return;
  if (defaut) { annoncerDefaut(); console::println("Sorties bloquees ; voir etat apres redemarrage."); return; }
  switch (commande.type) {
    case Type::Pilotage: pilotage(commande, maintenant); break;
    case Type::Moteur: case Type::Gauche: case Type::Droite: {
      const unsigned masque = commande.type == Type::Moteur ? 3 : commande.type == Type::Gauche ? 1 : 2;
      if (commande.valeur != 0 && chien.actif() && (chien.expire || chien.echu(maintenant))) {
        console::println("[CHIEN] Delai depasse : m, g ou d refuse ; envoyer hb ou p."); return;
      }
      annulerInversions(masque);
      for (size_t i = 0; i < 2; ++i) {
        if ((masque & (1u << i)) && !traction[i].consigne(static_cast<int>(commande.valeur), maintenant)) {
          erreur("Commande moteur refusee : verifier frein et etat."); return;
        }
      }
      actualiserSorties(maintenant);
      console::printf("[COMMANDE] %s : %d %% PWM.\n",
                      masque == 3 ? "deux moteurs" : masque == 1 ? "gauche" : "droite",
                      static_cast<int>(commande.valeur));
      break;
    }
    case Type::Servo: case Type::Centre: {
      const float angle = commande.type == Type::Centre ? 0 : commande.valeur;
      if (!position.demander(angle, maintenant)) { erreur("Angle hors de la course actuelle. Tape etat."); return; }
      if (!connecterServo()) { console::println("[DEFAUT] Initialisation servo impossible."); return; }
      console::printf("[SERVO] Cible %+.1f ; vitesse max %.1f deg/s (%s).\n", angle, position.vitesse,
                      position.courseRenseignee ? "course renseignee" : "angles indicatifs");
      break;
    }
    case Type::Vitesse:
      position.reglerVitesse(commande.valeur, maintenant);
      console::printf("[SERVO] Vitesse max de consigne : %.1f deg/s.\n", position.vitesse);
      break;
    case Type::Course:
      position.reglerCourse(static_cast<int>(commande.valeur), maintenant);
      etat(); break;
    case Type::Sens: case Type::SensGauche: case Type::SensDroite: {
      const unsigned masque = commande.type == Type::Sens ? 3 : commande.type == Type::SensGauche ? 1 : 2;
      annulerInversions(masque);
      for (size_t i = 0; i < 2; ++i)
        if ((masque & (1u << i)) && traction[i].demande != 0) {
          erreur("Mettre la roue concernee a zero et attendre son arret physique avant sens."); return;
        }
      actualiserSorties(maintenant); // Appliquer zero avant de modifier un DIR.
      for (size_t i = 0; i < 2; ++i)
        if (masque & (1u << i)) traction[i].direction = commande.valeur != 0;
      appliquerSens();
      console::println("[SENS] Direction reglee ; aucune reprise automatique de la roue arretee.");
      break;
    }
    case Type::InverseGauche: case Type::InverseDroite: {
      annulerInversions();
      if (traction[0].demande || traction[1].demande) {
        erreur("Envoyer m 0 et attendre l'arret physique avant de corriger l'avant."); return;
      }
      const size_t i = commande.type == Type::InverseGauche ? 0 : 1;
      const bool valeur = commande.valeur != 0;
      actualiserSorties(maintenant);
      if (!memoireDisponible || reglages.putBool(CLES_INVERSION[i], valeur) != sizeof(bool)) {
        erreur("Echec memoire : inversion non modifiee."); return;
      }
      inverse[i] = valeur;
      appliquerSens();
      console::printf("[REGLAGES] %s inverse=%u memorise ; traction reste a zero.\n", i == 0 ? "gauche" : "droite", valeur);
      break;
    }
    case Type::Frein:
      annulerInversions();
      for (auto &controle : traction) controle.reglerFrein(commande.valeur != 0);
      actualiserSorties(maintenant);
      digitalWrite(moteur::PIN_FREIN, traction[0].frein ? HIGH : LOW);
      console::printf("[MOTEURS] PWM zero ; BRAKE=%u.\n", traction[0].frein);
      break;
    default: break;
  }
}

void demarrer() {
  for (uint8_t pin : moteur::PINS_PWM) niveauBas(pin);
  for (uint8_t pin : test_actionneurs::PINS_DIR) niveauBas(pin);
  niveauBas(moteur::PIN_FREIN); niveauBas(PIN_SERVO);
  memoireDisponible = reglages.begin("fieldmind", false);
  if (memoireDisponible) {
    inverse[0] = reglages.getBool(CLES_INVERSION[0], inverse[0]);
    inverse[1] = reglages.getBool(CLES_INVERSION[1], inverse[1]);
  }
  appliquerSens(); // Les deux commandes PWM sont encore a zero.
  Serial.begin(115200);
  console::demarrer(); // File de sortie : la tache actionneurs n'attend jamais l'USB.
  ESP32PWM::allocateTimer(0); // Reserve uniquement les canaux 0/1 pour ESP32Servo.
  position.demander(0, millis()); // Centrage au demarrage demande par Hugo.
  if (!connecterServo()) return;
  for (size_t i = 0; i < moteur::NOMBRE_MOTEURS; ++i) {
    pwmAttache[i] = ledcAttachChannel(moteur::PINS_PWM[i], MOTEUR_HZ, MOTEUR_BITS, CANAUX_MOTEUR[i]);
    if (!pwmAttache[i]) { verrouillerDefaut("initialisation_moteur", moteur::PINS_PWM[i]); return; }
    if (!ledcWrite(moteur::PINS_PWM[i], 0)) { verrouillerDefaut("initialisation_zero_moteur", moteur::PINS_PWM[i]); return; }
  }
  dernierPourcent[0] = dernierPourcent[1] = 0;
}

void actualiser() {
  const uint32_t maintenant = millis();
  const bool usbPresent = static_cast<bool>(Serial) || Serial.available() > 0;
  if (usbPresent) { sessionUsb = true; dernierUsb = maintenant; }
  else if (sessionUsb && maintenant - dernierUsb >= 250) {
    arreterTout("usb_deconnecte");
    sessionUsb = false; accueil = false; longueur = 0; jeterLigne = false;
    chien.regler(0, maintenant); telemetrie.regler(0, maintenant); cadenceImu.store(0);
  }
  verifierChien(maintenant);
  actualiserInversions(maintenant);
  actualiserSorties(maintenant);
  noterMouvement(maintenant);
  mesurerRoues(maintenant);
  if (!usbPresent) { delay(1); return; }
  if (!accueil) { aide(); etat(); accueil = true; }
#ifdef FIELDMIND_ROVER_COMPLET
  static uint32_t dernierEtat = 0;
  if (!telemetrie.hz && maintenant - dernierEtat >= 1000) { dernierEtat = maintenant; etat(); }
#endif
  for (size_t lus = 0; lus < 128 && Serial.available(); ++lus) {
    const int c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (!jeterLigne) { ligne[longueur] = 0; executer(ligne); }
      longueur = 0; jeterLigne = false;
    } else if (c == '\b' || c == 127) {
      if (longueur && !jeterLigne) --longueur;
    } else if (!jeterLigne) {
      if (c < 32 || c > 126 || longueur >= sizeof(ligne) - 1) {
        longueur = 0; jeterLigne = true;
        erreur("Ligne invalide : jetee jusqu'au prochain retour a la ligne.");
      } else ligne[longueur++] = static_cast<char>(c);
    }
  }
  const uint32_t fin = millis();
  actualiserSorties(fin);
  noterMouvement(fin);
  if (telemetrie.due(fin)) envoyerTelemetrie(fin);
  delay(1);
}

uint32_t cadenceCentrale() { return cadenceImu.load(); }

bool tractionAuRepos(uint32_t maintenant, uint32_t dureeMs) {
  // Signe : la tache actionneurs peut avoir note un instant posterieur a maintenant.
  return static_cast<int32_t>(maintenant - dernierMouvement.load()) >= static_cast<int32_t>(dureeMs);
}

bool remiseLacetDemandee() { return remiseLacet.exchange(false); }

// La commande USB et les sorties continuent pendant les lectures/diagnostics I2C.
void tache(void *) {
  for (;;) actualiser(); // actualiser() cede la main avec delay(1).
}
bool demarrerTache() {
  static bool demarree = false;
  if (demarree) return true;
  if (xTaskCreate(tache, "actionneurs", 6144, nullptr, 2, nullptr) != pdPASS) {
    verrouillerDefaut("creation_tache_actionneurs");
    return false;
  }
  demarree = true;
  return true;
}
} // namespace actionneurs
