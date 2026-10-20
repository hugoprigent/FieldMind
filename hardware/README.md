# Électronique du prototype

Les [connexions actuelles](electronics/CONNEXIONS.md) et le [firmware C5](../software/esp32/esp_c5/README.md) sont le point de départ pour comprendre le montage. Le GNSS est relié au Raspberry Pi ; le XIAO ESP32-C5 relie capteurs I²C, écran, servo et commandes des deux drivers de moteurs.

La carte capteurs A7 est une étude de câblage sur plaque à pastilles, représentée dans KiCad ; ses couches internes représentent des fils isolés qui se croisent, pas un circuit imprimé multicouche à commander.

| Source | Usage et statut |
| --- | --- |
| [`CONNEXIONS.md`](electronics/CONNEXIONS.md) | Connexions relevées et statuts des essais ; prime sur les anciennes études |
| [`CONNEXIONS_MOTEUR_ZS_X11D1.md`](electronics/CONNEXIONS_MOTEUR_ZS_X11D1.md) | Interfaces des drivers moteur |
| [`Carte A7`](electronics/carte_capteurs/cad/perfboard_c5_a7/README.md) | Schéma, empreintes, routage de fils et vue 3D |
| [`Guide A7`](electronics/carte_capteurs/perfboard_une_plaque_c5_a7.md) | Étude de fabrication à adapter aux branchements actuels |
| [`Schéma système`](electronics/systeme_rover/README.md) | Étude antérieure de puissance et d'arrêt, non validée physiquement et différente de certains branchements courants |

Les plans A7 et système ne sont pas une photographie exacte de tous les branchements actuels. Le brochage et l'alimentation ont évolué après ces études. Lire les dates et statuts, et vérifier le montage avant mise sous tension. Les fusibles et une coupure physique de traction restent à dimensionner et à valider sur le rover.
