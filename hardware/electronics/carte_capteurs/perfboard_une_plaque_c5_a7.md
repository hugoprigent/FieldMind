**Français** · [English](perfboard_une_plaque_c5_a7.en.md)

# FieldMind — XIAO ESP32C5 et capteurs sur une perfboard 4 × 6 cm, tous les fils dessous (révision A7)

**A7 est la révision courante.** Elle reprend A6 : mêmes liaisons, aucune résistance ni condensateur, et toutes les broches du XIAO accessibles. Le câblage est refait pour que **la face de l'écran ne porte aucun fil**. A0 à A6 sont conservés. Rien n'a été acheté.

[Projet KiCad 10](cad/perfboard_c5_a7/fieldmind_c5_a7.kicad_pro) · [Guide interactif](exports/c5_a7/guide_cablage.html) · [Plan des broches, sans fils](exports/c5_a7/plan_broches.html) · [Connexions par composant](connexions_c5_a7.csv) · [Liste des fils](fils_c5_a7.csv) · [Occupation des trous](occupation_trous_c5_a7.csv) · Validation (référence de travail non incluse)

![Les deux faces](exports/c5_a7/deux_faces.png)

## Principe

| Face | Ce qu'on y trouve |
| --- | --- |
| **Dessus (côté écran)** | L'écran, le GY-521 et le BMP280 sur leurs supports, ainsi que les barrettes J4 (GPS), J6 et J8 (extensions) et J7 (Knob) au bord. **Aucun fil** : seulement 14 ponts de soudure visibles |
| **Dessous** | Le XIAO sur ses supports (USB-C au bord haut) et **tous les fils** |

- **Ponts de soudure** : sous son support, le XIAO cache ses pastilles côté dessous ; ses broches ne se soudent que côté écran. Chaque broche y est reliée **au trou voisin par une goutte de soudure**, sans fil. Le fil part ensuite de ce trou, côté dessous. Les broches GND et 3V3 sont pontées des deux côtés (3 pastilles). Sur une plaque non métallisée, faire passer le bout du fil par ce trou jusqu'au pont.
- **Lignes communes** : quatre rails en fil nu (GND, 3V3, SCL, SDA) descendent depuis les broches de l'écran, entre les deux supports du XIAO. En bas, SDA, SCL et GND tournent en L jusqu'au GY-521. Les autres cartes s'y raccordent par de courts fils.
- **Extensions alignées** : J6 (bord gauche) fait face aux broches 5V, GND, 3V3, D10, D9, D8 du XIAO, et J8 (bord droit) fait face à D0, D1, D2, D3. Chacune de ces liaisons est un fil nu droit sur une rangée.
- **30 fils** (19 nus, 11 isolés, 607 mm au total), **16 croisements**. Un fil isolé passe par-dessus un autre, toujours à angle droit et entre deux pastilles. Une extrémité de fil au plus par trou.

## Connexions par composant

Trous nommés en vue de dessus (colonnes A à N, rangées 1 à 20). Pour un composant posé dessus, le trou est soudé dessous.

| Composant | Broche → trou | Relié à |
| --- | --- | --- |
| **Écran OLED (J3)** | 1 GND → F1 · 2 VCC → G1 · 3 SCL → H1 · 4 SDA → I1 | GND, 3V3, SCL, SDA communs |
| **GY-521 (J1)** | 1 VCC → B20 · 2 GND → C20 · 3 SCL → D20 · 4 SDA → E20 | idem ; XDA, XCL, AD0, INT (F20 à I20) non câblés |
| **BMP280 (J2)** | 1 VCC → N18 · 2 GND → N17 · 3 SCL → N16 · 4 SDA → N15 | idem ; CSB (N14), SDO (N13) non câblés |
| **Modulino Knob (J7)** | 1 GND → N7 · 2 3V3 → N8 · 3 SDA → N9 · 4 SCL → N10 | idem (ordre du connecteur 1×4 du Knob) |
| **GPS (J4)** | 1 RX_GPS → A12 · 2 GND → A11 · 3 VCC → A10 · 4 TX_GPS → A9 | RX_GPS ← D6 du XIAO, TX_GPS → D7 du XIAO |
| **Extension J6** | 1 5V → A2 · 2 GND → A3 · 3 3V3 → A4 · 4 D10 → A5 · 5 D9 → A6 · 6 D8 → A7 | broche du XIAO du même nom |
| **Extension J8** | 1 D0 → N2 · 2 D1 → N3 · 3 D2 → N4 · 4 D3 → N5 | broche du XIAO du même nom |

| XIAO (U1, dessous) | Trou | Pont vers | Rôle |
| --- | --- | --- | --- |
| D0, D1, D2, D3 | J2, J3, J4, J5 | K2, K3, K4, K5 | J8.1 à J8.4 |
| D4 / GPIO23 | J6 | I6 (rail SDA) | SDA |
| D5 / GPIO24 | J7 | K7 | SCL |
| D6 / GPIO11 | J8 | K8 | TX ESP → RX GPS (J4.1) |
| D7 / GPIO12 | D8 | C8 | RX ESP ← TX GPS (J4.4) |
| D8, D9, D10 | D7, D6, D5 | C7, C6, C5 | J6.6, J6.5, J6.4 |
| 3V3 | D4 | C4 et E4 | J6.3 et rail 3V3 |
| GND | D3 | C3 et E3 | J6.2 et rail GND |
| 5V | D2 | C2 | J6.1 |

Le tableau complet, broche par broche avec toutes les broches reliées, est dans `connexions_c5_a7.csv` et en tête du guide interactif.

## Montage

1. Côté dessus : souder les supports des trois cartes et les barrettes J4, J6, J7 et J8, ainsi que les **14 ponts** du XIAO, après avoir soudé ses supports côté dessous.
2. Côté dessous : poser d'abord les 4 rails et les fils nus, puis les fils isolés par-dessus.
3. Contrôler la continuité net par net, et l'absence de court-circuit entre 5V, 3V3 et GND, cartes non enfichées.
4. Enficher les cartes.

**Encombrement** : environ 11 mm de cartes de chaque côté. Les prises Dupont de J4, J6, J7 et J8 se branchent par le dessus, au bord. Utiliser des entretoises d'au moins 14 mm sous la plaque (XIAO et fils), 18 mm par sécurité.

## Avant de souder

- Les dimensions et l'ordre des broches des cartes, en les posant sur la plaque. L'écart entre le GY-521 et le BMP280 (environ 0,8 mm) et celui entre le BMP280 et une entretoise du GY-521 (environ 0,5 mm) sont faibles.
- Les rappels CSB/SDO (BMP280) et AD0 (GY-521), à l'ohmmètre.
- L'alimentation du GNSS, avant de brancher le VCC de J4.
- L'adresse du Knob, par un scan I2C (attendu 0x3A, 0x3C, 0x68, 0x76).
- Signaux en 3,3 V maximum. Le 5V de J6.1 vient de l'USB : pour un capteur, jamais pour un servo.

**Statut : plan contrôlé dans KiCad, non testé physiquement, non fabriqué.**
