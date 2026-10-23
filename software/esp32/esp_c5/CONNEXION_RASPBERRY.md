**Français** · [English](CONNEXION_RASPBERRY.en.md)

# Firmware du rover et liaison Raspberry Pi

Programme a televerser : **esp_c5**, selectionne par defaut dans PlatformIO.
Sources : `src/main.cpp` (capteurs/ecran) et `src/actionneurs.cpp` (moteurs/servo).
L'alias esp_c5_complet produit le meme programme. Identifiant serie : **ROVER_3**
(commandes ROVER_2 inchangees, plus l'[extension ROVER_3](#extension-rover_3)).
Le test_tout_moteur reste disponible, avec le meme controleur et sans capteurs.

## Connexions actuelles

| XIAO ESP32-C5 | Destination |
| --- | --- |
| D0 / GPIO1 | Vitesse moteur gauche, fil rouge |
| D1 / GPIO0 | D/R moteur gauche, fil jaune, sens inverse en logiciel |
| D9 / GPIO9 | Vitesse moteur droit, fil rouge |
| D4 / GPIO23 | D/R moteur droit, fil jaune |
| D10 / GPIO10 | BRAKE des deux drivers, fils verts |
| D7 / GPIO12 | Signal servo |
| D5 / GPIO24 | SDA commun IMU, barometre, OLED et Knob |
| D6 / GPIO11 | SCL commun IMU, barometre, OLED et Knob |
| GND | References communes ESP, drivers (orange), alimentation servo |
| D2 / GPIO25 | Signal du servo du **volet gauche** de la remorque (ROVER_4, `volg`) — ou, au choix, sortie SC du driver gauche ([voir plus bas](#sorties-sc-des-drivers-facultatif)) |
| D3 / GPIO7 | Signal du servo du **volet droit** de la remorque (ROVER_4, `vold`) — ou sortie SC du driver droit |

Les deux fils jaunes D/R restent separes. D8 n'est plus utilisee.
D7 est occupee par le servo : aucun GPS sur cette broche dans ce montage.
Servo alimente par sa branche externe adaptee ; moteurs par leurs drivers.
Le cable USB ESP-Pi transporte la liaison serie et l'alimentation de la carte,
pas la puissance des moteurs ou du servo. STOP des drivers reste non pilote.

## Mise en route

### Deux roues dans le sens de marche

Le dernier essai de Hugo indique gauche correcte et droite toujours a l'envers.
Le nouveau defaut est gauche=1, droite=0 : seul le defaut droit change depuis
la revision precedente. L'ancienne cle droite inv_d est ignoree ; les nouvelles
corrections droites sont memorisees sous inv_d_v2. Le reglage gauche inv_g reste
conserve. Les corrections enregistrees avec cette revision gardent priorite
aux demarrages suivants ; `etat` affiche chaque valeur.
Le sens physique est a verifier roues libres du sol, pas deduit des couleurs.

1. Envoyer `m 0`, attendre l'arret physique, puis `sens 0` (avant logique).
2. Essayer `g 5`, puis `g 0`. La droite ne doit pas recevoir cette commande.
3. Essayer `d 5`, puis `d 0`. La gauche ne doit pas recevoir cette commande.
4. Si une roue recule, corriger uniquement son inversion, deux moteurs a zero :
   `inverseg 0` ou `inverseg 1` pour la gauche ; `inversed 0` ou `inversed 1`
   pour la droite. Choisir l'oppose de la valeur affichee par `etat`.
5. Refaire l'essai de cette roue, puis envoyer `m 5` pour essayer les deux.

La correction est memorisee en flash et partagee entre main et test combine.
Elle ne demarre aucun moteur ; vitesses et directions de marche ne sont pas
restaurees au reboot. Les boutons Corriger l'avant de la page PC font ce reglage.

1. Televerser **PlatformIO > esp_c5 > General > Upload**, puis fermer le moniteur.
2. Connecter le XIAO en USB au Raspberry Pi. Le servo demande le centre
   **1500 us au demarrage** ; traction a zero. Le premier centrage peut etre rapide.
3. Ouvrir la supervision du Pi, normalement `http://rover.local:8080/` (adresse du Pi à adapter).
   L'adresse peut changer avec le reseau. Le service est fieldmind-embarque.
4. Dans Ports serie, reperer l'ESP USB natif (`/dev/ttyACM0` le plus souvent).
   Verifier **ROVER_3**, puis envoyer `etat` dans le champ de ce port.
5. Tester roues libres du sol : `m 5`, `m 0`, `s -10`, `centre`, puis `stop`.

| Commande, terminee par Entree | Effet |
| --- | --- |
| m 10 | Meme commande PWM 10 % aux deux moteurs, maintenue jusqu'a m 0 ou stop |
| m 0 | Traction a zero, servo maintenu |
| g 10 / d 20 | PWM gauche 10 % / droite 20 %, sans changer l'autre moteur |
| g 0 / d 0 | Arreter la commande d'une roue seulement |
| sensg 0 / sensg 1 | Sens avant/arriere gauche ; gauche a zero et arret physique requis |
| sensd 0 / sensd 1 | Sens avant/arriere droite ; droite a zero et arret physique requis |
| inverseg 0/1 / inversed 0/1 | Calibration de l'avant par roue, deux moteurs a zero ; memorisee |
| s -10 / s 10 | Position servo relative au milieu (angles indicatifs) |
| centre | Cible milieu, 1500 us |
| v 30 | Vitesse maximale de consigne servo, en degres indicatifs/seconde |
| sens 0 / sens 1 | Inverser les deux sens ; apres m 0 et arret physique complet |
| frein 0 / frein 1 | PWM nul puis niveau BRAKE commun |
| stop | PWM traction nul et signal servo coupe ; alimentations toujours presentes |
| etat / aide | Consignes, defauts, frequences / commandes disponibles |
| p 12 -8 5.5 | ROVER_3 : PWM signes gauche / droite (-100..100) et angle servo ; silencieux |
| hb | ROVER_3 : renouvelle le chien de garde, sans reponse |
| wd 400 / wd 0 | ROVER_3 : chien de garde 100..2000 ms / coupe (coupe au demarrage) |
| inv 500 | ROVER_3 : roue a zero 100..3000 ms avant qu'un p change son sens |
| tel 20 / tel 0 | ROVER_3 : ligne T a 0..50 Hz ; etat periodique suspendu tant que tel > 0 |
| imu 50 / imu 0 | ROVER_3 : ligne I a 0..100 Hz ; imu 0 = ligne IMU texte d'origine |
| lacet0 | ROVER_3 : remet le lacet integre a zero |
| roues 1 / roues 0 | Lecture des sorties SC sur D2/D3 et deux champs de plus dans T / coupee (defaut) |
| volg 0..180 / vold 0..180 | ROVER_4 : servo du volet gauche (D2) / droit (D3), 50 Hz, 500..2500 us ; premiere consigne directe, puis 90 deg/s ; -1 coupe le signal. Refuse si roues 1 ; roues 0 garde les volets |

Les commandes avance/armer/test et les nombres seuls de l'ancien main sont
remplaces par ces commandes communes avec le test combine. Ne pas envoyer
simplement 10 : envoyer **m 10**.

Le main publie les mesures au format texte deja decode par la supervision
du Pi toutes les 500 ms, et l'etat actionneurs environ chaque seconde quand
l'USB est connecte. Le Pi dispose deja de l'envoi de lignes sur chaque port :
aucune conversion reseau vers JSON n'est requise pour cette console.
La page PC Web Serial accepte aussi ROVER_2, pour un essai directement sur PC.
Elle propose des commandes de vitesse/sens et la correction de l'avant pour
chaque roue. La commande m regle les deux pourcentages mais conserve leurs
directions individuelles. BRAKE reste partage sur D10.
Un seul logiciel doit ouvrir le port USB a la fois.

## Extension ROVER_3

Le pilote du Pi lit `[FIRMWARE]`. Avec ROVER_3, il envoie `inv`, `v`, `tel 20`,
`imu 50` et `wd 400`, puis une ligne `p` a chaque periode de conduite (20 Hz),
qui sert aussi de battement. Avec ROVER_2, il garde `g`, `d`, `sensg`, `sensd`, `s`.
Sans `wd`, `tel` ni `imu`, ROVER_3 se comporte comme ROVER_2.

- **wd** : sans `p` accepte ni `hb` pendant le delai, traction a zero comme
  `m 0`, servo maintenu, une seule ligne `[CHIEN] expire`. Aucune reprise : il
  faut un nouveau `p`. Tant que le delai est depasse, `m`, `g` et `d` non nuls
  sont refuses. Les autres lignes, valides ou non, ne renouvellent pas le chien.
  Il est verifie a chaque tour de la tache actionneurs (environ 1 ms).
- **p** : entiers -100..100, signe = sens logique de la roue (>= 0 avant,
  < 0 arriere) ; angle avec l'echelle et les bornes de `s`. Le servo est rattache
  s'il etait coupe ; sa trajectoire n'est relancee que si la cible change.
  Changement de signe d'une roue commandee : PWM a zero tout de suite, attente
  `inv` ms roue a zero, D/R change, puis 100 ms de preparation et la nouvelle
  valeur ; rien ne bloque et l'autre roue continue. Refus et arret si frein ou
  defaut, comme `m`. Aucun plafond ajoute (0..100 %, comme `m`).
- **T** : `T <ms> <pwm_g> <pwm_d> <servo_deg> <frein> <defaut> <chien>`, par
  exemple `T 123456 12 -8 5.5 0 0 1`. PWM appliques (0 pendant une attente ou
  la preparation), signe = sens logique applique ; servo = consigne en cours, une
  decimale ; chien = 1 si actif et non expire. Avec `roues 1`, deux champs de
  plus a la fin (`<hz_g> <hz_d>`, voir les sorties SC).
- **I** : `I <ms> <ax> <ay> <az> <gx> <gy> <gz> <lacet_deg>`, par exemple
  `I 42 0.012 -0.500 1.000 1.23 -0.01 0.15 -12.40`. Accelerations en g (3
  decimales), rotations en deg/s et lacet en degres (2 decimales), repere du
  GY-521. Lacet integre du gyroscope Z, dans [-180, 180). Traction a zero depuis
  plus de 1 s et gyroscope calme (chaque axe a moins de 1 deg/s de sa moyenne
  sur 0,25 s et, une fois le biais appris en 2 s d'arret, Z a moins de 3 deg/s
  du biais) : biais ajuste et lacet fige. Lignes emises seulement apres une
  lecture reussie ; tant que `imu` > 0, la ligne `IMU ...` de 500 ms est suspendue.
- Une perte USB detectee (250 ms) remet `wd`, `tel` et `imu` a zero ; `inv` reste.
- `etat` ajoute `[PILOTAGE] Chien=400 ms (ok|expire|coupe) ; inv=500 ms ;
  tel=20 Hz ; imu=50 Hz ; lignes perdues=0` (lignes USB jetees, file pleine).

| Evenement | Ligne envoyee |
| --- | --- |
| wd 400 / wd 0 | `[CHIEN] Delai 400 ms ; renouvele par p ou hb, sinon traction a zero.` / `[CHIEN] Coupe.` |
| Expiration | `[CHIEN] expire : traction a zero, servo maintenu ; envoyer p pour reprendre.` |
| m, g, d apres expiration | `[CHIEN] Delai depasse : m, g ou d refuse ; envoyer hb ou p.` |
| inv 600 | `[PILOTAGE] Inversion : roue a zero 600 ms avant changement de D/R.` |
| tel 20 / tel 0 | `[PILOTAGE] Telemetrie T a 20 Hz ; etat periodique suspendu.` / `[PILOTAGE] Telemetrie coupee.` |
| imu 50 / imu 0 | `[CENTRALE] Lignes I a 50 Hz ; ligne IMU texte suspendue.` / `[CENTRALE] Ligne IMU texte toutes les 500 ms.` |
| lacet0 | `[CENTRALE] Lacet remis a zero.` |
| p mal forme ou hors bornes | `[ERREUR] Commande invalide. Exemples : m 10, s -30, v 30, stop. Traction a zero, signal servo coupe.` |
| p non nul, frein actif | `[ERREUR] Commande moteur refusee : verifier frein et etat. Traction a zero, signal servo coupe.` |
| p hors course servo | `[ERREUR] Angle hors de la course actuelle. Tape etat. Traction a zero, signal servo coupe.` |
| p en defaut | `[DEFAUT] Cause=...` puis `Sorties bloquees ; voir etat apres redemarrage.` |

Parmi les lignes `[CHIEN]`, seule l'expiration contient le mot « expire ». Le test_tout_moteur garde
l'identifiant TOUT_MOTEUR_4 et accepte `p`, `hb`, `wd`, `inv` et `tel` ; il
repond `[CENTRALE] Absente de ce programme ; commande ignoree.` a `imu` et `lacet0`.

## Sorties SC des drivers (facultatif)

`roues 1` compte les fronts montants de la sortie `SC` de chaque driver, par
interruption : gauche sur D2 / GPIO25, droite sur D3 / GPIO7. Au demarrage
(`roues 0`), le code ne touche pas ces broches et rien ne change. `roues 1` les
met en entree sans tirage interne et remet les compteurs a zero ; `roues 0`
retire les interruptions et les laisse en entree haute impedance. Ce reglage
n'est pas annule par une perte USB (comme `inv`).

- **Anti-rebond** : un front arrivant moins de 200 us apres le front accepte
  precedent est rejete et compte a part (plus de 5000 impulsions/s).
- **Frequence** : moyenne des impulsions de chaque fenetre d'environ 100 ms,
  en impulsions/s, une decimale ; 0 sans front pendant 0,5 s (minimum 2 imp/s).
  SC ne donne pas le sens : le signe est celui du sens logique applique a la
  roue (une roue poussee en arriere alors qu'elle est commandee en avant
  apparait positive).
- **T** avec `roues 1` : 10 champs, `T <ms> <pwm_g> <pwm_d> <servo_deg> <frein>
  <defaut> <chien> <hz_g> <hz_d>`, par exemple `T 123456 12 -8 5.5 0 0 1 45.0 -38.5`.
  Sans `roues 1`, la ligne T garde exactement ses 8 champs. Le pilote du Pi lit
  les 8 premiers champs et n'utilise pas encore les deux derniers.
- **etat** avec `roues 1` : `[ROUES] SC gauche D2/GPIO25 : fronts=1234 rejets=0
  45.0 imp/s ; droite D3/GPIO7 : fronts=1190 rejets=2 -38.5 imp/s` (cumuls depuis
  `roues 1`). Reponses : `[ROUES] Lecture SC active : ...` et
  `[ROUES] Lecture SC coupee ; D2/D3 en entree haute impedance.`
- **Etalonnage** : le nombre d'impulsions par tour de roue depend du moteur.
  Roues libres, `roues 1`, faire un tour de roue a la main, driver alimente,
  puis lire `fronts` dans `etat`.

### Ce que l'on sait du signal SC

- Le fabricant ne le documente pas : selon [MAD-EE](https://mad-ee.com/easy-inexpensive-hoverboard-motor-controller/),
  les documents du ZS-X11H le presentent comme une interface reservee, sans
  support technique ; la sortie change d'etat a chaque changement d'un capteur
  Hall (90 changements par tour de roue sur leur moteur d'hoverboard, mesures
  a l'analyseur logique). Une impulsion comptee ici correspondrait alors a deux
  changements Hall.
- Sur le ZS-X11A, un utilisateur decrit une sortie logique de rapport cyclique
  1:1, de frequence liee a la vitesse, sans niveau de tension
  ([forum Arduino](https://forum.arduino.cc/t/interfacing-to-zs-x11a/499628)).
- Aucune source trouvee ne donne le niveau haut (5 V ou 3,3 V) ni le type de
  sortie (active ou collecteur ouvert), ni pour le ZS-X11H ni pour le ZS-X11D1.
  La carte alimente ses capteurs Hall en 5 V : un niveau haut de 5 V est
  plausible, **pas etabli**.
- L'ESP32-C5 accepte au plus VDD + 0,3 V (3,6 V) sur une entree et reconnait un
  niveau haut a partir de 0,75 x VDD, environ 2,5 V ; tirage interne de 45 kohm
  ([fiche technique ESP32-C5 v1.5](https://documentation.espressif.com/esp32-c5_datasheet_en.pdf), table 5-4).

### Branchement propose, apres mesure

Mesurer d'abord, driver alimente et **XIAO non relie** : tension entre `SC` et
le `GND` du driver au multimetre (ou a l'oscilloscope), roue tournee lentement
a la main.

| Mesure sur SC | Adaptation vers D2 (gauche) ou D3 (droite) |
| --- | --- |
| Alterne entre 0 V et environ 5 V | Pont diviseur : SC → 10 kohm → entree ; entree → 20 kohm → GND. Haut ≈ 5 V x 20/30 = 3,3 V |
| Reste bas ou flotte, sans niveau haut franc | Collecteur ouvert probable : tirage 10 kohm de l'entree au **3V3 du XIAO** (jamais au 5 V du driver), 1 kohm en serie entre SC et l'entree |
| Alterne entre 0 V et environ 3,3 V | 1 kohm en serie suffit |

Avec le pont, verifier au point milieu, avant de le relier au XIAO : niveau
haut entre 2,5 et 3,3 V, niveau bas sous 0,8 V. Un haut sous 2,5 V signale une
sortie a forte resistance interne : mesurer avant de changer les valeurs. Le
`GND` du driver reste commun avec celui de l'ESP ; un fil SC par driver, sans
relier les SC des deux cartes. Eviter de laisser un driver alimente quand le
XIAO ne l'est pas : le courant passerait par l'entree (limite a moins de
0,5 mA par la resistance de 10 kohm).

D2 / GPIO25 et D3 / GPIO7 sont des broches de strapping de l'ESP32-C5 (fiche
technique, section 3) : GPIO25 choisit le front d'echantillonnage SDIO (non
utilise ici), GPIO7 la source JTAG seulement si l'eFuse EFUSE_JTAG_SEL_ENABLE
est programme (0 par defaut). Le mode de demarrage depend de GPIO26/27/28, pas
de D2/D3. D8 / GPIO8 reste libre et n'est pas utilisee par ce code.

## Verification et limites

Le controle USB/servo/moteurs tourne dans une tache distincte de l'I2C.
Le servo reste sur ESP32Servo 3.2.1, 50 Hz, 800..2200 us. Les moteurs gardent
5 kHz. Les timers sont verifies independamment du rapport cyclique.
Une erreur coupe les sorties et verrouille le defaut ; une perte USB detectee
pendant 250 ms demande l'arret sans reprise automatique de la traction.
La fermeture d'une page ou le plantage du Pi ne garantit pas que l'USB soit
declare deconnecte. Sans `wd`, les commandes moteur n'ont pas d'expiration
automatique ; avec `wd`, le silence du Pi coupe la traction apres le delai.
Les sorties USB passent par une file : si l'hote ne lit plus, des lignes sont
perdues mais la tache actionneurs et le chien de garde continuent.

Les pourcentages et angles sont des consignes, pas des mesures physiques.
Le brochage GPS precedent et le contrat JSON propose dans contrat_esp_pi.md
ne sont pas implementes ici. Camera et lidar restent geres directement par le Pi.
La navigation autonome est sur le Pi ; le firmware n'ajoute que le chien de
garde, la commande combinee et les lignes rapides.

La compilation et les essais logiciels sans USB reel ne valident pas la
mecanique ni l'arret physique. Aucune commande de mouvement ni aucun flash
n'est envoye automatiquement au montage par Codex. La liaison Pi etait
injoignable lors de la preparation : sa validation en USB reste a effectuer.

Validation ROVER_3 du 19 octobre 2026 (avec la lecture SC), sans materiel :
compilations esp_c5, test_tout_moteur, esp_c5_capteurs, test_moteur et test_servo
reussies ; tests PC reussis, dont 75 verifications ROVER_3 du controleur reel
simule (69 pour test_tout_moteur) et 87 de la logique ROVER_3 et SC. Ni
televersement ni essai sur le rover : chien de garde, inversion, lignes T/I,
tache centrale et lecture SC (niveau electrique compris) restent a essayer.

Validation precedente (ROVER_2) : compilations esp_c5, test_tout_moteur et
esp_c5_capteurs reussies. Tests PC existants reussis, plus 34 verifications du
controleur reel avec materiel simule (28 nominales, 6 echec de tache), soit 34
au total : vitesses et sens independants, calibration memorisee et refusee
en mouvement, echec de sauvegarde et echec PWM d'une seule roue.
Transport USB : 4 tests reussis ; navigateur teste avec ROVER_2 et TOUT_MOTEUR_4
simules, commandes m/s/v/centre, STOP et blocage en defaut/firmware inconnu.
