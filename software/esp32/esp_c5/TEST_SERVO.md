# Test servo simple avec ESP32Servo

Environnement PlatformIO **test_servo**, a selectionner explicitement.
Pour la traction et le servo ensemble, choisir **test_tout_moteur** (defaut),
avec le servo sur D8 : voir [TEST_TOUT_MOTEUR.md](TEST_TOUT_MOTEUR.md).
Source : `src/test_servo.cpp`. Bibliotheque : ESP32Servo **3.2.1**.
Elle fournit la syntaxe Arduino `Servo`, `attach()` et `write()` sur ESP32-C5 :
https://github.com/madhephaestus/ESP32Servo/tree/3.2.1

## Identification du servo

Hugo lit DT996 et une inscription correspondant a DIYMORE-TOOL sur le boitier.
L'achat etait reference sous Heemol ; ce nom commercial ne suffit pas a identifier
la plage de commande. Un [projet de Chalmers de 2025](https://odr.chalmers.se/bitstreams/ee50e6f6-9bc7-4191-8657-cada9aee898c/download)
utilise exactement le DIYMORE-TOOL DT996 pour une course de 180 degres (p. 7).
Les auteurs y calibrent les limites PWM progressivement (p. 20) et alimentent
les servos a 6 V. C'est un retour d'experience, pas une fiche constructeur.
Aucune fiche officielle DT996 confirmant les impulsions extremes, le courant
de blocage ou la tension maximale n'a ete retrouvee le 18 octobre 2026.
Ne pas utiliser les caracteristiques du MG996R pour ce DT996.

## Branchement

Couper les alimentations avant de cabler. Debrancher la puissance des drivers
moteurs et surtout leur liaison D0 : cette broche sert au servo pour ce test.

| Depart | Destination |
| --- | --- |
| XIAO D0 / GPIO1 (pas GPIO0) | Signal du servo |
| Alimentation externe adaptee, branche 6 V prevue pour le DT996 | + du servo |
| - de cette alimentation | - du servo et GND de l'ESP |

Le servo ne doit pas etre alimente par le 3V3 de l'ESP ni par la batterie de
traction directement. Le courant du servo retourne directement a son alimentation.
Le plan general du rover conserve D8 pour le servo ; seul ce test utilise D0.

## Utilisation : angles et recherche de l'amplitude

Le servo demande le milieu (1500 us) au demarrage, puis attend une commande.
Liberer le palonnier avant de televerser. Ouvrir le moniteur a 115200 bauds.
Chaque commande doit etre suivie d'Entree.

| Commande | Action |
| --- | --- |
| -45, 0, 45, ou angle -45 | Rejoindre progressivement cet angle relatif au milieu puis maintenir |
| course 140 | Declarer une course totale REELLEMENT mesuree de 140 degres, sans mouvement |
| etat | Afficher la plage d'impulsions et si l'echelle d'angle a ete renseignee |
| test | Essayer la plage actuelle : milieu, cote bas, cote haut, puis milieu |
| plus | Elargir de 100 us de chaque cote, puis essayer cette nouvelle plage |
| moins | Reduire de 100 us de chaque cote, puis essayer |
| centre | Demander le milieu a 1500 us |
| stop | Detacher le servo et mettre le signal bas, y compris pendant un mouvement |
| aide | Revoir les commandes |

La plage initiale est maintenant 800..2200 us, confirmee utilisable par Hugo.
Les commandes plus donnent ensuite 700..2300, 600..2400 et enfin 500..2500 us.
Chaque trajet dure environ 3 secondes avec acceleration et ralentissement
progressifs. Les valeurs affichees sont des consignes, pas des mesures d'angle.
Le plafond 500..2500 us est une limite logicielle d'essai, pas une plage
mecanique validee pour le DT996. Aucune detection d'effort ou de butee n'existe.

Par defaut, les angles -90..+90 sont **indicatifs** : -90 correspond a 800 us,
0 au milieu a 1500 us et +90 a 2200 us. La conversion est lineaire entre
les extremites de la plage actuelle ; cela ne prouve pas une course de 180 degres.
L'origine 0 est le milieu commande par 1500 us, pas une position mesuree.
Les angles negatifs et positifs commandent les deux cotes ; le sens physique
depend du montage. -180..+180 ferait 360 degres au total, au-dela de la course
de 180 degres annoncee pour ce modele. Avec l'echelle initiale, ces commandes
sont refusees sans mouvement ; elles ne sont pas ramenees silencieusement aux limites.

Pour convertir en degres physiques approximatifs, mesurer l'angle total entre
les extremites. Par exemple, si la plage couvre REELLEMENT 140 degres, taper
`course 140` : les commandes deviennent -70..+70, le milieu reste 0 et `45`
demande environ 45 degres depuis le milieu. Ce nombre est un exemple, pas une
mesure du DT996. Le servo ne renvoie aucune position reelle.
Les decimales avec un point sont acceptees, par exemple -22.5.
Changer de plage avec plus ou moins invalide cette calibration et remet
l'echelle indicative -90..+90. Redemarrer remet aussi cette echelle indicative ;
aucun reglage de course n'est enregistre en memoire permanente.

L'estimation d'environ 120 degres donnee puis retiree par Hugo n'est pas une
mesure et n'est pas utilisee pour calibrer le code.

**Si le servo force, bourdonne ou ne gagne plus de course : stop immediatement,
et couper son alimentation si necessaire. Ne pas elargir davantage.**
Noter la derniere plage qui fonctionnait sans effort. Une nouvelle commande de
mouvement est necessaire apres stop ; elle ne reprend pas automatiquement.
Le signal coupe ne coupe pas l'alimentation du servo.

Hugo a confirme le fonctionnement avec ESP32Servo et environ 90 degres reels
avec l'ancienne plage 1000..2000 us. Le nouveau test permet de rechercher les
extremites utilisables, sans annoncer des angles physiques non calibres.

## Tester sans ESP

Un testeur de servos RC ou un recepteur radio RC compatible peut produire les
impulsions a la place de l'ESP. L'alimentation seule ne commande pas une position.
Voir https://www.pololu.com/blog/16/electrical-characteristics-of-servos-and-introduction-to-the-servo-control-interface

## Verification

Compilation PlatformIO test_servo verifiee le 18 octobre 2026.
Les conversions d'angle relatif et les refus hors plage sont verifies sur PC
par tests/servo_angles_test.cpp, lance avec python tests/verifier_policy.py.
Aucun televersement ni mouvement physique declenche depuis le PC.
Les anciens fichiers servo_test_reglages.h et tests/servo_test.cpp concernent
l'ancien test PWM et ne sont plus utilises par ce programme simplifie.
