// Test indépendant d'UN moteur ZS-X11D1 sur XIAO ESP32-C5.
// PlatformIO : environnement test_moteur. main.cpp est exclu de ce firmware.
// Console 115200 bauds : chaque commande se termine par Entrée.
//   0..100  -> pourcentage PWM maintenu jusqu'à stop, 0 ou nouvelle valeur
//   50      -> 50 % de PWM ; 0 -> demande PWM nulle
//   test    -> 1 % de PWM pendant 200 ms, puis coupure automatique
//   stop    -> coupe la demande PWM
//   etat    -> affiche la commande actuelle ; aide -> toutes les commandes
//   diag    -> registre PWM, frequence, motif et compteur des coupures USB
// Aucun essai automatique. Ni écran, ni capteurs, ni servo initialisés.
// Ni mesure de vitesse/courant ni garantie d'arrêt matériel par le PWM seul.
// PWM D0/GPIO1 ; DIR D1/GPIO0 ; BRAKE D10/GPIO10 ; masse commune au driver.
// Broches et console : moteur_zs_x11d1.cpp ; limites : moteur_policy.h.

#include <Arduino.h>
#include "moteur_zs_x11d1.h"

void setup() {
  moteur::demarrer();
  Serial.begin(115200);
  const uint32_t debut = millis();
  while (!Serial && millis() - debut < 3000) delay(10);
  moteur::annoncer();
}

void loop() {
  moteur::lireConsole();
  delay(1);
}
