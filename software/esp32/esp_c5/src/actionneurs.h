#pragma once
#include <stdint.h>

// Controleur partage entre le programme principal et le test sans capteurs.
namespace actionneurs {
void demarrer(); // PWM traction nul, servo au milieu.
void actualiser(); // Console USB + sorties ; utilise uniquement par le test.
bool demarrerTache(); // Programme principal : autonome vis-a-vis des capteurs I2C.

// ROVER_3, lus par la tache centrale du programme principal (sans verrou : atomiques).
uint32_t cadenceCentrale(); // Hz demandes par « imu » ; 0 = ligne IMU texte toutes les 500 ms
bool tractionAuRepos(uint32_t maintenant, uint32_t dureeMs); // demande nulle depuis dureeMs
bool remiseLacetDemandee(); // vrai une seule fois apres « lacet0 »
}
