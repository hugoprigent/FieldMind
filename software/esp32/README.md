**Français** · [English](README.en.md)

# Logiciels ESP32 du rover

Projet du XIAO ESP32-C5 : [esp_c5/](esp_c5/README.md).
Ouvrir software/esp32/esp_c5 dans VS Code avec PlatformIO.

Le programme par defaut **esp_c5 / ROVER_2** integre les deux moteurs,
le servo D7, l'ecran, l'IMU, le barometre et le Knob facultatif.
Voir [le guide Raspberry et le brochage](esp_c5/CONNEXION_RASPBERRY.md).
La supervision du Pi reste dans embarque/ (référence de travail non incluse).

Le test_tout_moteur reutilise le meme controleur, sans capteurs.
Les tests moteur seul (D0) et servo seul (D0, debranche du driver)
restent disponibles dans leurs environnements separes.

USB 115200 : m 10, m 0, s -10, centre, v 30, stop, etat.
Le main publie les capteurs au format deja decode par le Pi.
La lecture USB/sorties est separee de l'I2C. Le contrat JSON propose et
la navigation autonome ne sont pas implementes par ce changement.

Pour verifier les logiques sur PC : python tests/verifier_policy.py.
