**Français** · [English](DEUX_MOTEURS.en.md)

# Main : deux moteurs, servo et capteurs

Le programme actuel est **esp_c5 / ROVER_3** (commandes ROVER_2 conservees), selectionne par defaut.
Voir [le guide Raspberry et les connexions completes](CONNEXION_RASPBERRY.md).
Il utilise le meme controleur actionneurs.cpp que test_tout_moteur, en plus
des capteurs et de l'ecran.

| ESP32-C5 | Destination | Couleur |
| --- | --- | --- |
| D0 / GPIO1 | Vitesse gauche | Rouge |
| D1 / GPIO0 | D/R gauche, inverse en logiciel | Jaune |
| D9 / GPIO9 | Vitesse droite | Rouge |
| D4 / GPIO23 | D/R droite | Jaune |
| D10 / GPIO10 | BRAKE commun | Vert |
| GND | Reference commune drivers | Orange |
| D7 / GPIO12 | Signal servo | Selon faisceau |

Les deux D/R ne sont plus relies ensemble. Ne pas modifier le phasage/Hall
pour ce correctif. Le bus capteurs reste SDA D5 et SCL D6.

Commandes : m 10, m 0, s -10, centre, v 30, sens 0/1, frein 0/1, stop, etat.
Commandes independantes : g 10 / d 20, g 0 / d 0, sensg 0/1 et sensd 0/1.
Corriger l'avant avec inverseg 0/1 ou inversed 0/1, deux roues a zero :
reglage memorise. La droite est inversee par rapport a ROVER_1, gauche conservee.
Le frein reste commun. Les directions individuelles ne sont pas imposees par m.
Au demarrage, servo au centre et traction a zero. STOP du driver reste non
pilote. Les consignes physiques ne sont pas mesurees.

Le protocole JSON propose, le GPS et la navigation autonome ne sont pas ajoutes
par cette integration. Utiliser la console USB de la supervision Pi pour ces commandes.
