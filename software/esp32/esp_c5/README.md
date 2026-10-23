**Français** · [English](README.en.md)

# Logiciel ESP32 du rover - ESP C5

Ouvrir `software/esp32/esp_c5/` dans VS Code avec PlatformIO.
Le programme par defaut est **esp_c5**, identifiant **ROVER_4** (ROVER_3 plus les servos des volets de la remorque sur D2/D3) : deux moteurs,
servo, IMU, barometre, ecran et Knob facultatif. ROVER_3 garde toutes les
commandes de ROVER_2 et ajoute le pilotage par le Raspberry Pi (voir plus bas).

**Pour la connexion au Raspberry Pi : [guide et brochage actuels](CONNEXION_RASPBERRY.md).**

- Moteur gauche : vitesse D0/GPIO1, D/R D1/GPIO0 inverse en logiciel.
- Moteur droit : vitesse D9/GPIO9, D/R D4/GPIO23.
- BRAKE commun D10/GPIO10 ; servo D7/GPIO12.
- I2C conserve : SDA D5/GPIO24, SCL D6/GPIO11.
- Au demarrage : traction a zero et servo au centre (1500 us).

Les deux fils jaunes D/R restent separes. D7 ne peut pas recevoir de GPS
en meme temps que le servo. Les masses sont communes ; les actionneurs
conservent leurs alimentations adaptees.

| Environnement PlatformIO | Programme |
| --- | --- |
| **esp_c5** (par defaut) | Main complet : capteurs/ecran + deux moteurs et servo |
| esp_c5_complet | Alias du main complet |
| test_tout_moteur | Meme controleur moteurs/servo, sans capteurs |
| test_moteur | Ancien test d'un moteur, PWM D0, D/R D1, BRAKE D10 |
| test_servo | Test servo seul sur D0, a debrancher du driver |
| esp_c5_capteurs | Capteurs/ecran seuls, aucune sortie actionneur configuree |

## Televerser et commander

Choisir **PlatformIO > Project Tasks > esp_c5 > General > Upload**.
Fermer le moniteur avant d'ouvrir le port depuis le Raspberry ou un navigateur.
Le main et le test combine acceptent les memes commandes a 115200 bauds :

```text
etat
m 5
m 0
s -10
centre
v 30
stop
```

`m` regle le pourcentage commun des deux drivers, `s` l'angle relatif au milieu,
`v` la vitesse de consigne servo. Les valeurs sont des consignes, pas des mesures.
`g 10` et `d 20` reglent les roues independamment ; `g 0` ou `d 0` n'arrete
que la roue choisie. `sensg 0/1` et `sensd 0/1` reglent leur avant/arriere,
roue concernee a zero et apres son arret physique.
`inverseg 0/1` et `inversed 0/1` corrigent le sens de l'avant, deux moteurs
a zero ; ces deux reglages sont memorises en flash. Voir le guide Raspberry.
La consigne moteur persiste jusqu'a une autre commande, stop ou une perte USB detectee.
`stop` coupe la commande traction et le signal servo ; les alimentations restent presentes.
`sens 0/1` change le sens logique commun, uniquement apres m 0 et arret mecanique.
Les nombres seuls et les anciennes commandes avance/armer/test ne sont pas
les commandes du nouveau main. La console Pi utilise simplement les lignes ci-dessus.

La page [USB locale sur PC](INTERFACE_USB.md) accepte ROVER_3, ROVER_2 et TOUT_MOTEUR_4.
Le [test combine](TEST_TOUT_MOTEUR.md) conserve sa console separee.
Les [reglages du test servo seul](TEST_SERVO.md) concernent D0, pas D7.
Le [diagnostic moteur](DIAGNOSTIC_MOTEUR.md) concerne surtout le test moteur seul.

## Extension ROVER_3 : pilotage par le Raspberry Pi

Commandes ajoutees, toutes coupees au demarrage (detail et formats dans le
[guide Raspberry](CONNEXION_RASPBERRY.md#extension-rover_3)) :

```text
wd 400        chien de garde : sans p ni hb pendant 400 ms, traction a zero
hb            renouvelle le chien de garde, sans reponse
p 12 -8 5.5   PWM signes gauche/droite et angle servo, silencieux sauf erreur
inv 500       roue a zero 500 ms avant qu'un p change son sens
tel 20        ligne T compacte a 20 Hz (etat periodique suspendu)
imu 50        ligne I a 50 Hz avec lacet integre ; lacet0 le remet a zero
```

Le pilote du Pi (`software/rover/fieldmind/materiel/esp.py`) lit `[FIRMWARE]` :
s'il voit ROVER_3, il envoie `inv`, `v`, `tel 20`, `imu 50` et `wd 400`, puis une
ligne `p` a chaque periode de conduite (20 Hz), qui sert aussi de battement. Avec
ROVER_2, il revient aux commandes `g`, `d`, `sensg`, `sensd`, `s`.
Sans `wd`, `tel` ni `imu`, le firmware se comporte comme ROVER_2 (meme lignes).

Facultatif : `roues 1` lit la sortie `SC` (impulsions de vitesse) de chaque
driver sur D2 / GPIO25 (gauche) et D3 / GPIO7 (droite), et ajoute `<hz_g> <hz_d>`
a la fin de la ligne T. Par defaut (`roues 0`), ces broches ne sont pas touchees.
Le niveau electrique de SC n'est pas documente : le mesurer et adapter le
signal avant de le relier ([branchement propose](CONNEXION_RASPBERRY.md#sorties-sc-des-drivers-facultatif)).

## Sources et verification

- `src/main.cpp` : mesures, affichage et navigation Knob.
- `src/actionneurs.cpp` : controleur commun, console USB et sorties ; tache autonome dans le main.
- `src/actionneurs.h` : interface du controleur.
- `src/test_tout_moteur.cpp` : lance le controleur sans capteurs.
- `src/test_tout_moteur_broches.h` : broches et inversion partagees main/test.
- `src/test_tout_moteur_logique.h` : commandes et trajectoire servo.
- `src/test_tout_moteur_pwm.h` : relecture des frequences LEDC.
- `src/pilotage_rover3.h` : chien de garde, passage par zero avant inversion, formats T et I.
- `src/centrale_lacet.h` : lacet integre et biais du gyroscope estime a l'arret.
- `src/console_serie.h` : file de sortie USB et tache d'ecriture unique.
- `src/roues_sc.h` : comptage des impulsions SC, anti-rebond et frequence lissee.
- `src/moteur_zs_x11d1.cpp` : ancien controleur utilise par test_moteur seulement.

Les mesures restent publiees toutes les 500 ms au format texte decode par
la supervision Pi (sauf apres `imu` > 0) ; l'etat actionneurs est publie environ
chaque seconde (sauf apres `tel` > 0). La tache actionneurs reste independante
des acces I2C. La disparition logique USB detectee pendant 250 ms demande l'arret,
sans garantie de detection d'un plantage de l'application hote : c'est le role
du chien de garde `wd`, coupe tant que l'hote ne l'a pas demande.
L'IMU est lue par une tache separee (25 Hz, ou la cadence `imu`), hors du rythme
de l'ecran ; un verrou protege les lectures I2C partagees avec l'ecran et le Knob.
Les lignes USB des taches actionneurs, capteurs et centrale passent par une file
et une seule tache d'ecriture : elles ne s'entremelent pas, et une tache de
commande n'attend jamais l'USB (si l'hote ne lit plus, des lignes sont perdues ;
`etat` affiche leur nombre).

Verification sans materiel : `python tests/verifier_policy.py` ; le controleur
reel est aussi execute avec horloge, USB et GPIO simules (centrage, inversion,
commandes simultanees, stop, perte USB et echec de creation de tache), puis
avec les commandes ROVER_3 (chien de garde, inversion par `p`, lignes T) et la
lecture SC (impulsions simulees, rebond, signe, ligne T a 10 champs).
Les compilations ne prouvent pas le comportement mecanique ni la navigation autonome.

## Écran et capteurs

Les sections concernant l'écran et les capteurs s'appliquent aux environnements
`esp_c5`, `esp_c5_complet` et `esp_c5_capteurs`.

Programme PlatformIO pour le **XIAO ESP32C5** de la [carte capteurs A7](../../../hardware/electronics/carte_capteurs/perfboard_une_plaque_c5_a7.md). Il lit le **GY-521** (MPU-6050) et le **BMP280**, et les affiche sur l'**écran OLED**. Si un **Modulino Knob** est branché, l'écran devient le bureau d'accueil du rover, qu'on parcourt au Knob. Sans Knob, l'écran affiche en continu les mesures des capteurs. Tous sont sur le bus I2C, avec SDA sur D5 (GPIO24) et SCL sur D6 (GPIO11).

## Lancer

```sh
cd software/esp32/esp_c5
pio run -e test_moteur                         # compiler le test moteur
pio run -e test_moteur -t upload --upload-port COM9  # téléverser le test moteur
pio device monitor --port COM9 --baud 115200    # moniteur série
```

Pour remettre le programme principal :

```sh
pio run -e esp_c5 -t upload --upload-port COM9
```

Fermer le moniteur série avant de téléverser, sinon le port est occupé. Si le téléversement ne démarre pas, maintenir le bouton **BOOT** du XIAO en branchant l'USB, puis relancer.

`COM9` est le port identifié sur ce PC le 17 octobre ; il peut changer.
Dans PowerShell, si `pio` n'est pas dans le PATH, le remplacer par
`& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"`.
L'environnement `esp_c5_capteurs` permet de compiler le programme capteurs
seul, sans configurer les sorties moteur : `pio run -e esp_c5_capteurs`.

La plateforme est [pioarduino](https://github.com/pioarduino/platform-espressif32) (Arduino-ESP32 3.3), la seule à gérer l'ESP32-C5. Le programme a été compilé avec sa version 55.03.311.

## Naviguer (avec le Knob)

| Geste sur le Knob | Effet |
| --- | --- |
| Tourner | À l'accueil, ouvre le menu. Dans le menu, change d'appli. Dans une appli, passe à la voisine |
| Appui court | Ouvre l'appli choisie, ou revient au menu |
| Appui long (0,7 s) | Revient à l'accueil |

Le menu revient seul à l'accueil après 30 s sans geste.

## Les écrans

En haut de chaque écran, une barre d'état donne le titre, le temps depuis le démarrage et trois pastilles : **I** pour le GY-521, **B** pour le baromètre et **K** pour le Knob. Une pastille pleine indique un composant présent, une pastille vide un composant absent.

- **Accueil** : les yeux du robot. Ils clignent, glissent vers le côté qui penche et louchent quand on secoue la carte. Si un composant manque, son nom s'affiche en bas.
- **Capteurs** (seul écran sans Knob) : verdict et norme de l'accélération, accélérations (g) et rotations (°/s) du GY-521 ; verdict, pression (hPa) et température du BMP280.
- **Niveau** : un niveau à bulle, avec le roulis et le tangage en degrés. « A plat » s'affiche sous 1,5°.
- **Diagnostic** : les adresses présentes sur le bus I2C, rebalayées chaque seconde, avec un verdict pour chaque composant connu.
- **Infos** : carte, adresse MAC, mémoire libre et date de compilation.

Le moniteur série reçoit toutes les 500 ms l'état du GY-521, du BMP280 et du Knob, et l'écran affiché. Taper **`d`** dans le moniteur lance un diagnostic du bus, à 100 kHz puis à 400 kHz : niveau des lignes SDA et SCL au repos, 200 balayages avec le nombre de réponses par adresse, et lecture répétée du chip-ID du BMP280. La LED du XIAO clignote tant que le programme tourne.

Verdicts :

| Verdict | Signification |
| --- | --- |
| `OK` | Il répond et ses valeurs sont plausibles : au repos, environ 1 g d'accélération ; pression entre 800 et 1100 hPa |
| `DOUTEUX` | Il répond mais les valeurs sont anormales : lecture ratée, capteur secoué ou abîmé |
| `ABSENT` | Aucune réponse à son adresse |

Un composant absent est recherché à nouveau toutes les 2 s : on peut l'enficher sans redémarrer.

## Réglages (en tête de `src/main.cpp`)

- **Écran à l'envers** : `OLED_ROTATION` vaut 2, car l'écran est monté tête en bas sur la carte. Mettre 0 s'il apparaît à l'envers. Les yeux et la bulle suivent automatiquement.
- **Menu qui défile à l'envers** : `KNOB_SENS = -1`. Si un cran fait sauter plusieurs applis, augmenter `KNOB_PAS`.
- **Yeux ou bulle sur le mauvais axe** : `AXES_ECHANGES = true`. Pour un axe inversé, `SENS_X = -1` ou `SENS_Y = -1`.
- **Erreurs sur le bus** : le bus tourne à 400 kHz pour que l'écran soit fluide. Repasser `I2C_HZ` à 100000 si des composants disparaissent.

## Si un composant manque

- **Rien sur le bus** : vérifier le 3V3, le GND et les fils SDA/SCL. Vérifier aussi qu'au moins une carte a ses résistances de tirage I2C.
- **Knob absent** : il répond en 0x3A (ou 0x3B). Brancher ses fils d'après les noms sérigraphiés, pas d'après la position des broches.
- **BMP280 en 0x77 ou GY-521 en 0x69** : ils sont trouvés automatiquement. Cela signifie que SDO ou AD0 est tiré à 3V3 sur la carte.
- **BMP280 qui change d'adresse (0x76 puis 0x77)** : sa broche SDO n'est reliée à rien, ni sur le module ni sur la carte A7, et elle flotte. Le pilote du programme le tolère : il essaie les deux adresses et ne garde que les lectures confirmées deux fois. La vraie correction est un fil entre SDO et GND, plus un fil entre CSB et 3V3.
- **Température à -138,6 °C (température brute à 0) ou pression qui saute de centaines d'hPa** : le bus et la calibration sont bons, mais la partie mesure de la puce est morte. La cause la plus probable est une alimentation en 5 V : le module GY-BMP280-3.3 n'a pas de régulateur et ne supporte pas plus de 4,25 V. Taper `b` dans le moniteur affiche les registres bruts.
- **Pas de moniteur série** : le XIAO utilise l'USB natif. Attendre la fin du redémarrage, ou appuyer sur RESET.
