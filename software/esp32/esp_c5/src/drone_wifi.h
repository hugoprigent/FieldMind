#pragma once
// Passerelle Wi-Fi / UDP / video vers le mini-drone camera (application HD-DRONE).
// Tourne dans une tache FreeRTOS de basse priorite (1) pour ne jamais retarder
// la tache actionneurs (2) ni la tache centrale IMU (2).

#include <Arduino.h>

namespace drone {

// Demarre la tache Wi-Fi du drone en arriere-plan.
bool demarrerTache();

// Intercepte les commandes serie commencant par "drone".
// Renvoie true si la ligne a ete prise en charge.
bool traiterCommande(const char *texte);

} // namespace drone
