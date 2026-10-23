**Français** · [English](README.en.md)

# FieldMind — schéma électrique complet du rover prototype

Schéma de câblage de tout le rover, en complément de la carte capteurs A7. Il est fait au **format KiCad 10**, sans routage ni PCB, et a été contrôlé par l'ERC (0 erreur, 0 avertissement).

[Fichier des connexions](../CONNEXIONS.md) · [Projet KiCad](cad/fieldmind_rover_systeme.kicad_pro) · [PDF des 4 feuilles](exports/schema_rover_complet.pdf) · [Nomenclature](exports/nomenclature.csv) · [Carte capteurs A7](../carte_capteurs/perfboard_une_plaque_c5_a7.md)

| Feuille | Contenu |
| --- | --- |
| 1. Vue d'ensemble | Les trois blocs, les étiquettes qui relient les feuilles, ce qui reste à relever |
| 2. Puissance et arrêt | Batterie, fusibles, interrupteur général, 3 XL4015, arrêt d'urgence, relais, contacteur, contrôleur et moteurs du E600, servo |
| 3. Raspberry Pi et USB | Alimentation USB-C du Pi, 2 webcams, lidar C1 et son adaptateur, liaison USB vers le XIAO |
| 4. Carte capteurs et périphériques | Connecteurs de la carte A7 : GPS sur J4, Knob sur J7, servo et état d'arrêt sur J6, J8 réservé |

Les symboles viennent des bibliothèques KiCad standard : `Device:Battery`, `Device:Fuse`, `Switch:SW_SPST`, `SW_Push_Open` (arrêt d'urgence NF), `SW_Push`, `Relay:FINDER-40.52` et `FINDER-40.11`, `Motor:Motor_DC`, `Motor:Motor_Servo`, `Connector:USB_C_Plug_USB2.0`, ainsi que `power:GND` et `power:PWR_FLAG`. Les modules absents des bibliothèques sont dessinés en blocs dans `FieldMind_Rover.kicad_sym` : Pi 4 vu par ses ports, carte A7, XL4015, webcams, lidar, adaptateur, GPS, Knob et contrôleur du E600.

## Câblage, liaison par liaison

**Puissance**

| De | Vers | Remarque |
| --- | --- | --- |
| BT1 + | F0 → S1 → bus | Fusible principal au plus près de la batterie |
| Bus | F1 → XL4015 n°1 IN+ | Sortie réglée à **5,1 V** → câble USB-C du Pi (VBUS et GND seulement) |
| Bus | F2 → XL4015 n°2 IN+ | Sortie réglée à **6,0 V** → servo + ; servo − directement sur OUT− |
| Bus | F4 → XL4015 n°3 IN+ | Sortie réglée à **12 V** → chaîne d'arrêt (bobines K1 et K2) |
| Bus | F3 → K2 contact 11 → 14 | Puissance de traction → V+ du contrôleur du E600 |
| Tous les IN− et OUT− des XL4015, BT1 −, GND du contrôleur | Barre de masse | Chaque retour de puissance a son propre fil |

**Arrêt d'urgence** : +12 V → S2 (arrêt, NF) → S3 (réarmement, NO) → bobines K1 et K2 (A1), puis A2 à GND.
- K1 11-14, câblé en parallèle sur S3, assure le maintien : un appui sur S3 colle K1 et K2, un appui sur S2 fait tout retomber.
- Relâcher S2 ne relance rien : il faut réarmer avec S3.
- K1 21-24 est un contact sec d'état : 21 va à GND, 24 à **J6.5 (D9)**. Fermé, il indique que la chaîne est disponible.

**Pi et USB**

| Port du Pi | Appareil |
| --- | --- |
| USB-C | Alimentation 5,1 V (XL4015 n°1) |
| USB 3.0 n°1 et n°2 (bleus) | Webcam gauche, webcam droite |
| USB 2.0 n°1 | Adaptateur du lidar C1. Faisceau du lidar : rouge 5V, jaune TX → RX, vert RX ← TX, noir GND |
| USB 2.0 n°2 | USB-C du XIAO (données et alimentation de la carte capteurs) |

**Carte capteurs A7**

| Connecteur | Branché sur |
| --- | --- |
| J4.1 RX_GPS (D6) | RXD du GPS |
| J4.2 GND, J4.3 3V3 | GND et VCC du GPS |
| J4.4 TX_GPS (D7) | TXD du GPS (PPS non utilisé) |
| J7.1 à J7.4 | GND, 3.3V, SDA, SCL du Modulino Knob, dans le même ordre |
| J6.6 D8 | Signal (PWM) du servo, avec J6.2 GND comme référence |
| J6.5 D9 | K1 contact 24 (état de la chaîne d'arrêt) |
| J6.1 5V, J6.3 3V3, J6.4 D10 | Libres |
| J8.1 à J8.4 (D0 à D3) | Réservés aux retours roues ou à la commande du contrôleur, après démontage du E600 |

## Ce qui reste à identifier

- **Donneur E600** : batterie (chimie, tension maximale chargée, BMS, chargeur), contrôleur de traction (commande, retours), moteurs (DC ou brushless).
  - La commande (CMD) et les retours (RET) du contrôleur sont laissés non câblés tant qu'ils ne sont pas connus.
  - Pré-charge à prévoir si le contrôleur a de gros condensateurs d'entrée.
- **Pack** : il doit rester **≤ 36 V chargé**, la limite d'entrée des XL4015. Les fusibles et porte-fusibles doivent tenir la tension maximale du pack.
- **Calibres** des fusibles F0 à F4, du contacteur K2 et des câbles : à fixer sur les courants mesurés.
- **Le XL4015 n°3 et les relais 12 V** sont une proposition faite avec le matériel du lot. Le relais K1 (Finder 40.52, bobine 12 V) et le contacteur K2 ne sont **pas achetés**.
- Signal 3,3 V reconnu par le servo, brochage du GPS reçu, bilan USB du Pi (1,2 A au total ; hub alimenté sinon).

**Statut : schéma contrôlé dans KiCad (ERC), non testé physiquement.** Régénération : `python3 scripts/generer_systeme.py --force`, ce qui écrase toute édition manuelle.
