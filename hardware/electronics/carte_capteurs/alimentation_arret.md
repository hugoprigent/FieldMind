# Alimentation et arrêt — A0

La petite carte ne distribue que le 3,3 V des petits modules et les signaux. Batterie, fusibles, conversions, coupure de traction et alimentation servo sont hors carte. Le schéma fonctionnel matérialise ces branches ; ce document fixe les conditions de raccordement.

## Montage sur table

1. Pi : alimentation 5 V adaptée au Pi 4 et à ses périphériques, câble approprié. ESP : USB de données depuis le Pi. Ne pas ajouter une alimentation au 5 V/3,3 V du MCU tant que son schéma n'a pas été examiné.
2. Le 3,3 V exporté par l'ESP peut alimenter seulement les modules identifiés compatibles et dans la réserve de courant documentée puis mesurée du régulateur. Consommations OLED, IMU, baromètre et GNSS sont **à relever**, aucune somme validée actuellement.
3. Servo : source régulée externe réglée dans la plage confirmée à réception, cible 6 V. Source coupée pendant le branchement. V+ et GND de puissance vont directement au servo ; signal PWM et GND de référence partent de J6.
4. Les rails positifs 5 V Pi, 3,3 V logique et 6 V servo restent distincts. Identifier le lien GND existant par l'USB, la conversion Pi et les autres branches avant d'ajouter des straps de masse. Ne jamais interrompre volontairement une masse de signal sans architecture d'isolation adaptée.

## Chemins des courants

```text
BAT+ -> F0 -> sectionneur -> F_PI -> buck Pi -> câble Pi
                         -> F_SERVO -> buck servo -> SERVO V+
                         -> F_MOTEUR -> coupure DC -> contrôleur -> moteur
BAT- <- retours dédiés des bucks et du contrôleur

buck servo GND <------ retour de puissance servo (direct, hors petite carte)
              <------ référence signal GND J6 (liaison dédiée, courant limité)
Pi USB GND ------------ ESP GND / carte support / référence des interfaces
```

Ce tracé est une intention de câblage. Si les bucks sont non isolés, leurs entrées/sorties négatives et l'USB créent déjà plusieurs liens GND : relever la topologie réelle, éviter les boucles importantes et garder les retours servo/moteur hors des pistes de capteurs. Mesurer la différence GND ESP–servo en activité et la stabilité 3,3 V. Une coupure de traction doit laisser le diagnostic Pi/ESP vivant si la distribution choisie le permet.

## Batterie et conversions : critères avant choix

Le pack n'est pas choisi. Relever chimie, configuration, tension nominale, tension maximale chargée, BMS, chargeur et état. Comparer **toute** la plage réelle aux convertisseurs avec marge pour transitoires et aux équipements DC.

Le module [XL4015 AZDelivery](https://www.az-delivery.de/products/spannungswandler-5a-8-36v-zu-1-25-32v) annonce 8–36 V en entrée. Un pack Li-ion 10S de 36 V nominal peut atteindre 42 V : **ne pas brancher directement ce pack au XL4015**. Les 5 A annoncés ne constituent pas un courant continu validé en boîtier. Le 5 V du Pi n'est pas une entrée adaptée à ce module pour la branche servo. Régler chaque buck avec une source compatible, vérifier à vide puis sous charge avant de brancher Pi ou servo.

Remplir le tableau avec mesures et fiches ; ne pas utiliser de chiffres de simulation.

| Branche | Tension | Courant nominal / pointe | Câble / longueur | Fusible DC / courbe / pouvoir de coupure | État |
| --- | --- | --- | --- | --- | --- |
| Principal F0 | Pack : TBD | TBD | TBD | TBD | À dimensionner |
| Pi + USB | 5 V cible | TBD, bilan Pi + 2 webcams + lidar + ESP | TBD | TBD | À mesurer |
| Servo DT996 | 6 V cible, plage à confirmer | TBD, démarrage/effort direction | TBD | TBD | Commandé, non essayé |
| Traction | TBD | TBD, démarrage/blocage/contrôleur | TBD | TBD | Non choisie |
| Petits capteurs | 3,3 V logique, alimentation modules à confirmer | TBD | Faisceaux courts | Protection adaptée au régulateur/source | À identifier |

Le fusible protège le câble et la branche. Son courant, sa courbe, sa tension DC et son pouvoir de coupure dépendent de la source et du câblage ; les calibres ne sont pas figés. Un fusible/porte-fusible seulement 32 V ne convient pas à une branche pouvant atteindre 42 V.

Le Pi 4 fournit au maximum **1,2 A au total sur USB** selon la [documentation Raspberry Pi](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#maximum-power-output). Mesurer les quatre périphériques ensemble ; si un hub alimenté est utilisé, vérifier qu'il ne réalimente pas le Pi par l'USB. Dimensionner aussi la source 5 V Pi et la chute du câble.

## Arrêt matériel et logiciel

L'arrêt local agit par une chaîne câblée indépendante de l'ESP, du Pi et du réseau. Après choix du contrôleur, définir un organe DC qui retire l'autorisation de traction ou coupe son alimentation selon la fonction effectivement documentée. Un petit bouton de menu ou un relais quelconque n'est pas cet organe. Le relâchement de l'arrêt ne doit pas produire de redémarrage ; prévoir un réarmement manuel de la chaîne et un nouvel armement logiciel.

J12 ne lit que le **contact sec auxiliaire** : fermeture vers GND = chaîne disponible ; ouverture, câble débranché ou ESP redémarré = état logiciel non autorisé. Ne pas injecter le pack ou la bobine d'un contacteur sur cette entrée. Une entrée simple ne diagnostique pas tous les courts-circuits : la coupure réelle reste dans la chaîne externe. Le logiciel ne peut pas court-circuiter cette chaîne.

Le retrait de puissance ne garantit pas un freinage : roue libre, freinage contrôlé, alimentation résiduelle et comportement du servo dépendent des actionneurs. Mesurer le comportement roues levées, puis au sol à faible vitesse et en zone libre avant de définir une vitesse démonstrable. Tant que moteur et contrôleur sont inconnus, aucun état de sortie ne peut être déclaré « arrêt garanti ».

La sortie servo est basse au démarrage, sans PWM. Après défaut, retirer la traction puis appliquer le comportement servo validé (maintien de dernière consigne, suppression des impulsions ou autre). Ne pas imposer un retour au centre pendant le roulage. La résistance série de signal n'est pas un convertisseur de niveau : si 3,3 V n'est pas reconnu par le DT996, adapter l'interface avant essai.
