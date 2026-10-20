#pragma once

#ifdef FIELDMIND_MOTOR_CONTROL
namespace moteur {
void demarrer();
void annoncer();
// En mode deux moteurs, les mesures periodiques se reglent avec capteurs 0/1.
bool journalCapteursActif();
#ifdef FIELDMIND_DUAL_MOTOR
// Lit les commandes USB dans une tache distincte des capteurs I2C.
bool demarrerConsoleAutonome();
#endif
// Lit des lignes bornées. Renvoie 'd' ou 'b' pour les diagnostics capteurs.
char lireConsole();
}  // namespace moteur
#endif
