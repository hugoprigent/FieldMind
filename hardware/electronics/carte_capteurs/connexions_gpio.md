# Connexions et budget GPIO — A0

**Le montage retenu est maintenant le C5 sur perfboard A1 (référence de travail non incluse), avec boutons sur ADC.** Les affectations C5 de ce plan sont documentées par Seeed, pas testées physiquement. Ce document A0 conserve la proposition générique à quatre entrées boutons et ses réservations ; ne pas câbler ses 22 fonctions simultanément sur les 11 GPIO du XIAO.

**Aucune affectation GPIO n'est encore validée sur la carte possédée.** Le schéma utilise U1 comme symbole de fonctions, sans empreinte et sans numéro de broche ESP réel. J0 est une interface d'adaptation proposée ; son numéro de pad ne désigne pas une broche XIAO. La table complète, générée depuis la CAO, est connexions_support.csv (référence de travail non incluse).

## Connecteur d'adaptation J0

J0 est une proposition de barrette 2 × 12 au pas 2,54 mm, numérotée par rangées : 1/2, 3/4, …, 23/24. Le pad 1 est carré dans le PCB. Vérifier le sens d'enfichage réel et étiqueter le faisceau ; aucun raccordement direct par ressemblance à un DevKit/XIAO.

| Pad J0 | Fonction | Sens vu de l'ESP | GPIO réel |
| --- | --- | --- | --- |
| 1 | +3V3 depuis la sortie régulée ESP, budget à vérifier | Alimentation | Pas un GPIO |
| 2 | GND | Référence | Pas un GPIO |
| 3 / 4 | I2C_SDA / I2C_SCL | Bus bidirectionnel / horloge | TBD |
| 5 / 6 | GNSS_RX / GNSS_TX | Entrée GPS TX / sortie GPS RX | TBD |
| 7 / 8 / 9 / 10 | BTN_UP / BTN_DOWN / BTN_OK / BTN_BACK | Entrées actives bas | TBD |
| 11 | SERVO_PWM | Sortie, sans impulsions au démarrage | TBD |
| 12 | HW_CHAIN_N | Entrée : bas = contact sec fermé ; haut = arrêt/non disponible | TBD |
| 13 / 14 | ENC1_A / ENC1_B | Entrées, interface à confirmer | TBD |
| 15 / 16 | ENC2_A / ENC2_B | Entrées, interface à confirmer | TBD |
| 17 à 20 | CTRL_IO1…4 | Direction/interface non définie | TBD |
| 21 | IMU_INT optionnel | Entrée | TBD |
| 22 | GNSS_PPS optionnel | Entrée | TBD |
| 23 | BUMP_N optionnel | Entrée contact sec | TBD |
| 24 | RESERVE | À attribuer | TBD |

L'USB est celui de la carte ESP existante, raccordé directement au Pi. Il ne passe pas par J0 ni par la carte support. Réserver ses signaux, ainsi que les broches de démarrage, flash/PSRAM, LED et extensions occupées par la carte exacte.

## Connecteurs périphériques

Ces ordres de fils sont des **choix de carte support**. Fabriquer un faisceau croisé vers le brochage réel du module, documenté et contrôlé au multimètre. Tous les modules sont initialement débranchés ; les straps ne suffisent pas à empêcher l'alimentation parasite par les signaux.

| Connecteur | Pads dans l'ordre | Conditions avant branchement |
| --- | --- | --- |
| J1 IMU | 1 V_IMU, 2 GND, 3 SDA, 4 SCL, 5 IMU_INT_EXT | GY-521 identifié ; régulateur, pull-ups et entrée d'alimentation vérifiés |
| J2 baromètre | 1 V_BARO, 2 GND, 3 SDA, 4 SCL | Historique : BMP280 3,3 V ; variante/pins possédés à contrôler, mode I2C et straps CS/SDO si module à 6 broches |
| J3 OLED | 1 V_OLED, 2 GND, 3 SDA, 4 SCL | Modèle et ordre VCC/GND connus, adresse relevée ; OLED « classique » ne définit pas sa tension ni son contrôleur |
| J4 extension | 1 V_EXT, 2 GND, 3 SDA, 4 SCL | Option non montée ; capteur déporté sur liaison courte ou avec adaptation séparée |
| J5 GNSS | 1 V_GNSS, 2 GND, 3 GPS_TX_EXT, 4 GPS_RX_EXT, 5 GPS_PPS_EXT | GPS reçu, tension d'entrée et niveaux UART vérifiés. TX GPS vers RX ESP ; PPS facultatif |
| J6 servo | 1 SERVO_SIG, 2 GND référence | Positif et retour de puissance servo **hors carte**. R3 220 Ω et R4 100 kΩ ne convertissent pas 3,3 V vers 5 V |
| J7 boutons | 1 GND, 2 UP_EXT, 3 DOWN_EXT, 4 OK_EXT, 5 BACK_EXT | Quatre boutons NO chacun entre sa ligne et GND ; attention aux paires de pattes d'un bouton 4 pattes |
| J8/J9 encodeurs | 1 V_ENCn, 2 GND, 3 A_EXT, 4 B_EXT | Alimentations, type de sortie, fréquence et niveau à identifier ; links ouverts en A0 |
| J10 contrôle | 1 GND, 2…5 CTRL_EXT1…4 | Aucun protocole choisi. Pas de moteur, PWM, CAN ou UART raccordé sans interface adaptée |
| J11 pare-chocs | 1 GND, 2 BUMP_EXT | Option contact sec ; composantes R7/R8 DNP en A0 |
| J12 état arrêt | 1 GND, 2 HW_CHAIN_EXT | Seulement contact sec auxiliaire ; jamais le contact de puissance ni tension de bobine |

JP1…5 : alimentations modules ouvertes par défaut. JP6 : INT optionnel ouvert. JP7…9 : signaux GNSS ouverts jusqu'au contrôle des niveaux. JP10…19 : extensions encodeurs/contrôleurs ouvertes. Les bridges ne se ferment qu'après identification. R1/R2 de 4,7 kΩ restent DNP jusqu'au bilan des pull-ups ; les autres straps ne sélectionnent pas de tension 5 V.

## Bilan de GPIO

| Fonction | Nombre minimal / variante | Réservation |
| --- | --- | --- |
| I2C partagé IMU/baro/OLED | 2 | SDA/SCL compatibles avec la carte réelle |
| GNSS UART | 2, ou 1 en lecture seule si RX GPS inutilisé | UART matériel disponible ; USB de debug indépendant |
| Quatre boutons séparés | 4 | Entrées hors broches de strapping sensibles |
| Signal servo | 1 | Sortie PWM, état de reset vérifié |
| Retour arrêt matériel | 1 | Entrée obligatoire, rappel externe |
| **Base interface + capteurs + servo** | **10** (9 si GNSS TX omis) | Aucun contrôle moteur dans cette somme |
| 2 encodeurs quadrature A/B | +4 | Conditionnel ; alternative télémétrie documentée |
| 1 interface contrôleur UART TX/RX | +2 | Hypothèse de budget, pas choix de contrôleur |
| Contrôle en 4 lignes | +4 à la place des +2 | Seulement si le contrôleur le demande |
| INT IMU / PPS GPS / pare-chocs | +1 chacun | Options, peuvent être omises |
| Réserve | +1 souhaité | À conserver si possible |

Avec quadrature × 2 et contrôle UART, la cible atteint **16 GPIO**, ou **20** avec les trois options et une réserve. Le maximum prévu sur J0 est 22 lignes logiques. La présence d'un connecteur ne garantit pas que le MCU les expose toutes.

La documentation [Seeed XIAO S3](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) distingue les variantes standard, Sense et Plus ; la version standard annonce 11 GPIO sur l'interface principale. **Le scénario 16 GPIO ne rentre pas directement sur ces seules broches.** Si c'est la carte possédée, privilégier la télémétrie du contrôleur, omettre INT/PPS/options et préparer une extension de boutons sur I2C ou une lecture résistive ADC. Ces adaptations ne sont pas dans le schéma A0 et nécessitent une révision ; aucun achat d'expander n'est décidé. Réutiliser une autre carte ESP déjà possédée si son brochage donne la marge requise. Deux ESP disponibles permettent aussi une séparation future, mais augmentent firmware et câblage pour cette semaine.

La documentation [Seeed C5](https://wiki.seeedstudio.com/xiao_esp32c5_getting_started/) doit être utilisée seulement après confirmation de cette variante. Les broches USB/strapping/mémoire d'un S3 et d'un C5 ne sont pas interchangeables. La documentation [GPIO S3 Espressif](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html) sert au contrôle des réservations, pas à déduire l'empreinte d'une carte.

## Valeurs proposées et mesures

- Boutons : rappel 10 kΩ à +3V3, 330 Ω en série côté panneau, 100 nF côté GPIO vers GND. Fermé, environ 0,11 V à 3,3 V selon valeurs nominales ; remontée RC de l'ordre de 1 ms. Anti-rebond logiciel proposé 20 ms, à ajuster par mesure ; ces chiffres sont des calculs, pas des essais.
- Retour arrêt : rappel 10 kΩ, série 1 kΩ, 100 nF ; fermé, environ 0,30 V nominal. Vérifier la marge VIH/VIL de la carte réelle et le temps de détection. Filtrage logiciel ne doit pas masquer une ouverture prolongée.
- I2C : commencer à 100 kHz, un module à la fois. Relever les pull-ups à **3,3 V**, leur résistance parallèle et les fronts. La relation `R_eq = 1 / Σ(1/R_i)` montre pourquoi ne pas ajouter systématiquement 4,7 kΩ. Les limites dépendent des puces/capacité ; voir [NXP UM10204](https://www.nxp.com/docs/en/user-guide/UM10204.pdf). Une adresse devinée n'est pas une adresse testée.

Dans le dossier d'identification, marquer pour chaque ligne **proposée**, **vérifiée sur documentation de la carte exacte**, puis **testée physiquement**, avec date et résultat. Aucun signal 5 V ne va directement à un GPIO ESP.
