# Architecture du prototype

Le **Raspberry Pi 4** exécute les pilotes, la perception géométrique, l'estimation de pose, la carte, la navigation et les missions. Le **XIAO ESP32-C5** lit les capteurs et applique les consignes de traction et de servo ; son chien de garde annule la traction si les consignes cessent. Le poste web affiche les capteurs, la carte et la mission, et permet le pilotage manuel.

L'agent IA propose des appels d'outils structurés. Le logiciel en vérifie les arguments et les transforme en actions de mission ; les commandes passent par le même superviseur que les autres modes de conduite. Le modèle n'écrit pas directement les PWM moteur.

Le profil `gx10` utilise des API de modèles exécutés sur le **GB10 du ASUS Ascent GX10**. Le profil `aucun` permet la simulation et les commandes du poste sans service IA. Un profil Gemini externe reste disponible pour comparaison et développement ; il doit être sélectionné explicitement et nécessite une clé.

La simulation actuelle remplace les pilotes matériels : faux ESP au même protocole, lidar, images stéréo et GNSS simulés. Le reste de la chaîne est celui du rover. La première démo MuJoCo dans `software/services`, `software/simulation` et `software/interface` est une pile distincte, conservée pour reproduire la démo en ligne.

Le drone et la remorque sont des extensions expérimentales. Les scénarios du drone sont simulés ; un protocole vidéo ou un rapport de simulation ne prouve pas un vol autonome réel.

Pour les interfaces détaillées : [architecture rover](../software/rover/ARCHITECTURE.md), [protocole web](../software/rover/PROTOCOLE.md), [firmware](../software/esp32/esp_c5/README.md), [déploiement IA](../software/rover/deploiement/gx10/README.md).
