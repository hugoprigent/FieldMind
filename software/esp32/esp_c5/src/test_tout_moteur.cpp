// Test sans capteurs, utilisant le meme controleur que le main.
#include "actionneurs.h"
void setup() { actionneurs::demarrer(); }
void loop() { actionneurs::actualiser(); }
