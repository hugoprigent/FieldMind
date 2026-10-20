// Recherche de course avec ESP32Servo, D0 / GPIO1.
// D0 debranche du driver moteur. Servo alimente separement, GND commun.
// Palonnier libre. Pas de detection automatique des butees !
#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include <errno.h>
#include <stdlib.h>
#include "servo_angles.h"

Servo servo;
constexpr int PIN_SERVO = D0;
constexpr int CENTRE_US = servo_angles::CENTRE_US;
constexpr int MIN_TEST_US = 500, MAX_TEST_US = 2500;
constexpr int PAS_AMPLITUDE_US = 100;
constexpr unsigned long TRAJET_MS = 3000;
int amplitude = 700; // 800..2200 us : plage confirmee par Hugo.
int courseDegres = 180; // Echelle indicative tant que la course n'est pas mesuree.
bool courseRenseignee = false;
int positionDemandee = CENTRE_US;
bool arret = false;
String ligne, demande;

void lireConsole() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\r' || c == '\n') {
      ligne.trim();
      if (ligne == "stop") {
        arret = true;
        demande = "";
        servo.detach();
        pinMode(PIN_SERVO, OUTPUT);
        digitalWrite(PIN_SERVO, LOW);
        Serial.printf("STOP : signal coupe. Derniere consigne : %d us.\n", positionDemandee);
      } else if (ligne.length()) {
        demande = ligne; // Reprise uniquement apres une nouvelle commande.
      }
      ligne = "";
    } else if (ligne.length() < 32) {
      ligne += c;
    } else {
      // Une ligne trop longue annule aussi le test en cours.
      arret = true;
      demande = "";
      servo.detach();
      pinMode(PIN_SERVO, OUTPUT);
      digitalWrite(PIN_SERVO, LOW);
      ligne = "";
      Serial.println("Ligne trop longue : signal coupe.");
    }
  }
}

bool connecterServo() {
  if (!servo.attached()) {
    servo.setPeriodHertz(50);
    servo.attach(PIN_SERVO, MIN_TEST_US, MAX_TEST_US);
    servo.setTimerWidth(14); // Reglage apres attach() pour ESP32Servo 3.2.1.
  }
  if (servo.attached()) return true;
  arret = true;
  Serial.println("ERREUR : connexion servo impossible.");
  return false;
}

bool deplacerVers(int cible) {
  const int depart = positionDemandee;
  for (unsigned long t = 0; t < TRAJET_MS; t += 20) {
    lireConsole(); // stop reste actif pendant tout le mouvement.
    if (arret) return false;
    const float x = float(t) / TRAJET_MS;
    const float progression = x * x * (3.0f - 2.0f * x);
    positionDemandee = lroundf(depart + (cible - depart) * progression);
    servo.writeMicroseconds(positionDemandee);
    delay(20);
  }
  lireConsole();
  if (arret) return false;
  positionDemandee = cible;
  servo.writeMicroseconds(cible);
  return true;
}

void aide() {
  Serial.println("=== ANGLES ET AMPLITUDE - ESP32Servo - D0 / GPIO1 ===");
  Serial.println("-45 / 0 / 45 : rejoindre une consigne relative puis la maintenir");
  Serial.println("0 = milieu ; angles negatifs et positifs = les deux cotes ; etat = reglages");
  Serial.println("course 140 : declarer 140 degres REELLEMENT mesures entre les extremites");
  Serial.println("Sans mesure : echelle indicative -90..+90, pas une calibration.");
  Serial.println("-180..+180 representerait 360 degres, pas les 180 degres annonces du DT996.");
  Serial.println("test : aller-retour dans la plage actuelle, puis milieu");
  Serial.println("plus : elargir de 100 us de chaque cote, puis tester");
  Serial.println("moins : reduire de 100 us de chaque cote, puis tester");
  Serial.println("centre : revenir au milieu ; stop : couper le signal");
  Serial.println("Buttees NON detectees : stop si effort, bruit ou absence de mouvement.");
}

void etat() {
  Serial.printf("PLAGE : %d..%d us ; angles -%.1f..+%.1f degres ; 0 = milieu (%s).\n",
                CENTRE_US - amplitude, CENTRE_US + amplitude, courseDegres / 2.0f, courseDegres / 2.0f,
                courseRenseignee ? "course renseignee par l'utilisateur" : "INDICATIFS, course non mesuree");
}

bool lireEntier(const String &texte, int minimum, int maximum, int &valeur) {
  if (!texte.length()) return false;
  errno = 0;
  char *fin = nullptr;
  const char *debut = texte.c_str();
  const long nombre = strtol(debut, &fin, 10);
  if (errno || fin == debut || *fin || nombre < minimum || nombre > maximum) return false;
  valeur = static_cast<int>(nombre);
  return true;
}

bool lireAngle(const String &texte, float &angle, int &impulsion) {
  if (!texte.length()) return false;
  errno = 0;
  char *fin = nullptr;
  const char *debut = texte.c_str();
  angle = strtof(debut, &fin);
  return !errno && fin != debut && !*fin &&
         servo_angles::convertir(angle, courseDegres, amplitude, impulsion);
}

void setup() {
  Serial.begin(115200);
  if (connecterServo()) servo.writeMicroseconds(CENTRE_US);
  aide();
  etat();
  // Retour au milieu au demarrage ; le test attend ensuite une commande.
}

void loop() {
  lireConsole();
  if (!demande.length()) { delay(2); return; }
  const String commande = demande;
  demande = "";
  if (commande == "aide") { aide(); return; }
  if (commande == "etat") { etat(); return; }
  if (commande.startsWith("course ")) {
    String mesure = commande.substring(7);
    mesure.trim();
    int valeur = 0;
    if (!lireEntier(mesure, 1, 360, valeur)) {
      Serial.println("Tape course suivi de la course totale mesuree (1..360 degres).");
      return;
    }
    courseDegres = valeur;
    courseRenseignee = true;
    etat(); // Cette commande de calibration ne deplace pas le servo.
    return;
  }
  String texteAngle = commande.startsWith("angle ") ? commande.substring(6) : commande;
  texteAngle.trim();
  float angle = 0;
  int cible = 0;
  if (lireAngle(texteAngle, angle, cible)) {
    arret = false;
    if (!connecterServo()) return;
    Serial.printf("ANGLE %+.1f degres depuis le milieu (%s) -> %d us.\n", angle,
                  courseRenseignee ? "course renseignee" : "INDICATIF", cible);
    if (deplacerVers(cible)) Serial.println("Consigne maintenue. Nouvelle valeur ou stop pour changer.");
    return;
  }
  if (commande != "plus" && commande != "moins" && commande != "test" && commande != "centre") {
    Serial.printf("Tape un angle de -%.1f a +%.1f (0 = milieu), centre, stop, test ou aide.\n",
                  courseDegres / 2.0f, courseDegres / 2.0f);
    return;
  }
  if (commande == "plus") {
    if (amplitude >= MAX_TEST_US - CENTRE_US) {
      Serial.println("Plafond d'essai atteint : 500..2500 us. Aucun mouvement lance.");
      return;
    }
    amplitude += PAS_AMPLITUDE_US;
  }
  if (commande == "moins" && amplitude > PAS_AMPLITUDE_US) amplitude -= PAS_AMPLITUDE_US;
  if (commande == "plus" || commande == "moins") {
    courseDegres = 180;
    courseRenseignee = false; // Changer de plage invalide la mesure precedente.
    etat();
  }
  arret = false;
  if (!connecterServo()) return;
  if (!deplacerVers(CENTRE_US)) return;
  if (commande == "centre") {
    Serial.println("Milieu demande : 1500 us.");
    return;
  }
  Serial.printf("PLAGE TESTEE : %d..%d us. Tape stop si le servo force.\n",
                CENTRE_US - amplitude, CENTRE_US + amplitude);
  if (!deplacerVers(CENTRE_US - amplitude)) return;
  if (!deplacerVers(CENTRE_US + amplitude)) return;
  if (!deplacerVers(CENTRE_US)) return;
  Serial.println("Retour au milieu. Tape plus pour elargir, test pour recommencer.");
}
