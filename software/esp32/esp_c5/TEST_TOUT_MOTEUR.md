**Français** · [English](TEST_TOUT_MOTEUR.en.md)

# Test manuel : traction et servo

Source : `src/test_tout_moteur.cpp`. Environnement PlatformIO
**test_tout_moteur** (test sans capteurs ; le programme par defaut est esp_c5). ESP32Servo 3.2.1 est conserve
pour le servo. Le main, le test moteur et le test servo restent disponibles
dans leurs environnements respectifs. Ce programme ne lit aucun capteur.

## Connexions

Couper les alimentations avant de deplacer les fils. **Le servo passe de D0
(ancien test isole) a D7 dans ce programme. D0 retourne au driver moteur 1.**

| Broche XIAO ESP32-C5 | Destination | Couleur indiquee par Hugo |
| --- | --- | --- |
| D7 / GPIO12 | Signal servo DT996 | Selon faisceau servo |
| D0 / GPIO1 | Entree vitesse driver moteur 1, 0-5 V | Rouge |
| D9 / GPIO9 | Entree vitesse driver moteur 2, 0-5 V | Rouge |
| D1 / GPIO0 | D/R du driver vitesse D0 uniquement | Jaune |
| D4 / GPIO23 | D/R du driver vitesse D9 uniquement | Jaune |
| D10 / GPIO10 | BRAKE des deux drivers | Vert |
| GND | GND des drivers et reference alimentation servo | Orange cote drivers |

Les vitesses et directions des moteurs sont maintenant independantes : g et d,
sensg et sensd. m garde la commande de pourcentage aux deux moteurs.
Le frein reste commun D10. Chaque roue a son reglage d'inversion de l'avant,
inverseg/inversed 0 ou 1, memorise en flash. Gauche conservee et droite inversee
selon le dernier retour de Hugo : defaults gauche=1, droite=0. La cle droite
historique inv_d est ignoree, les nouvelles corrections utilisent inv_d_v2.
Le reglage gauche inv_g reste conserve ; les nouvelles corrections memorisees
ont priorite aux demarrages suivants.
**Separer les deux fils jaunes : driver D0 sur D1, driver D9 sur D4.**
Ne pas modifier les phases ni les capteurs Hall pour ce correctif.
D7 est utilisee par le servo et ne peut pas etre utilisee par un GPS simultanement.
D4 est disponible car le bus I2C actuel reste D5/D6.
Le programme principal esp_c5 utilise maintenant ce meme cablage et le meme
controleur actionneurs.cpp, avec les capteurs en plus. Si un seul driver est branche, utiliser D0 pour celui-ci ;
la sortie D9 reste disponible pour le second. L'interface vitesse de chaque
driver doit etre configuree comme lors du test moteur qui fonctionne.
Les phases et capteurs Hall restent branches a leur propre driver.

Le servo garde son alimentation externe adaptee (branche 6 V prevue) ; son
retour de courant va directement a cette alimentation. Relier sa reference
GND au GND de l'ESP. Les drivers gardent l'alimentation de traction. Ne pas
relier leurs sorties 5 V entre elles ni a l'alimentation USB de l'ESP.

## Televerser et utiliser

Mettre les roues de traction libres du sol avant les essais.
PlatformIO > Project Tasks > **test_tout_moteur > General > Upload**.
Ouvrir le moniteur serie a **115200 bauds**, commandes suivies d'Entree.
Aucun televersement n'a ete effectue par Codex.

| Commande | Effet |
| --- | --- |
| m 10 | 10 % de PWM aux deux drivers, maintien jusqu'a m 0, stop ou perte USB detectee |
| m 0 | Commande de traction nulle ; position du servo maintenue |
| g 10 / d 20 | Pourcentage gauche / droite, sans modifier l'autre roue |
| sensg 0/1 / sensd 0/1 | Sens par roue, apres sa commande a zero et son arret physique |
| inverseg 0/1 / inversed 0/1 | Corriger l'avant par roue ; moteurs a zero ; memorise |
| s -30 | Rejoindre -30 par rapport au milieu, puis maintenir la position demandee |
| s 0 ou centre | Rejoindre le milieu du servo |
| s 30 | Rejoindre +30 ; la vitesse de traction reste la meme |
| v 30 | Limiter la vitesse de consigne servo a 30 degres indicatifs par seconde |
| stop | PWM traction nul et signal servo coupe ; les alimentations restent presentes |
| etat ou diag | Consignes, niveaux DIR/BRAKE, position demandee, frequences PWM |
| aide | Liste des commandes |
| sens 0 / sens 1 | Niveau de D/R ; uniquement apres m 0 et arret physique du moteur |
| frein 0 / frein 1 | PWM nul puis niveau BRAKE commun ; une nouvelle commande m est necessaire |
| course 140 | Declarer une course totale reellement mesuree de 140 degres |

Les alias `moteur`, `servo` et `vitesse_servo` sont aussi acceptes. Un nombre
seul est refuse pour eviter la confusion entre vitesse moteur et angle servo.
Une commande invalide coupe les commandes des deux actionneurs.

Exemple, une ligne a la fois :

```text
v 30
s 0
m 5
s -20
s 20
m 0
stop
```

Au demarrage, la traction est a zero et **le servo demande le centre (1500 us)**.
Ce comportement a ete retabli a la demande de Hugo, une fois le servo degage
du mecanisme. Le main et le test utilisent le meme controleur actionneurs.cpp.

### Regler le milieu mecanique

1. Couper l'alimentation servo et deconnecter la tringlerie ou retirer le palonnier
   (le bras visse sur l'axe). L'axe doit pouvoir bouger sans entrainer le rover.
2. Alimenter le servo avec son alimentation adaptee et sa masse commune,
   signal sur D7. Envoyer `centre` ou cliquer sur Recentrer a 0 dans le navigateur.
   Cela demande 1500 us. Le premier mouvement peut etre rapide : l'ESP ne mesure
   pas la position physique et ne peut pas garantir un premier trajet progressif.
3. Une fois le servo stabilise, reperer la position de l'axe. Couper son alimentation,
   puis remettre le palonnier pour aligner la direction tout droit sans tourner l'axe.
4. Realimenter, demander `centre`, puis essayer `s -10`, `centre`, `s 10`.
   Garder les petits debattements tant que la mecanique n'est pas reglee.

1500 us est notre zero de commande ; ce n'est pas une mesure du milieu exact
des butees. Si une consigne fixe fait tourner l'axe continuellement au lieu de
le stabiliser a une position, arreter l'essai : il faut identifier le type du servo.
Un servo a rotation continue commande une vitesse autour du neutre, pas un angle.
Voir [l'explication du fabricant Pololu](https://www.pololu.com/blog/24/continuous-rotation-servos-and-multi-turn-servos).

La plage servo reste **800..2200 us**, deja essayee par Hugo ; ce programme
ne l'elargit pas. L'echelle **-90..+90 est indicative**, 0 est le milieu.
La position physique du servo et les tours/minute des moteurs ne sont pas
mesures. `m 10` est un pourcentage de commande PWM, pas une vitesse mesuree.
`v` agit sur la rapidite de deplacement de la consigne servo, pas sa position
ni son couple. L'acceleration et le ralentissement sont progressifs.

Une course totale ne doit etre renseignee qu'apres mesure. Par exemple,
`course 140` donnerait une plage -70..+70 avec 0 au centre ; 140 n'est pas
une mesure du DT996. Ce reglage est perdu au redemarrage.

`stop` ne commande pas la broche STOP du driver et ne coupe pas la batterie.
Une commande PWM nulle ne garantit pas l'immobilisation physique du montage.
Le signal servo coupe ne garantit pas le maintien de la direction mecanique.

## Implementation et verification

Le servo utilise ESP32Servo a 50 Hz, resolution 14 bits, avec son allocation
limitee aux canaux LEDC 0/1. Les deux sorties moteurs utilisent explicitement
les canaux 4/5 a 5 kHz, resolution 10 bits. Cette separation evite que les
commandes servo et moteur reutilisent le meme canal. Les frequences sont
relues en fonctionnement ; une erreur coupe les sorties et verrouille un defaut.

La version **TOUT_MOTEUR_2** lit directement les timers par le driver LEDC,
independamment du duty. Dans Arduino-ESP32 3.3.11, ledcReadFreq() peut renvoyer
zero avant la premiere impulsion ou quand le duty est nul ; l'ancienne version
traitait ce zero comme une panne. La verification actuelle tolere 1 % ou 1 Hz
d'arrondi mais continue de refuser un timer absent ou une frequence incorrecte.
Au demarrage, etat affiche environ 50 Hz pour D7 et 5000 Hz
pour D0/D9 meme a 0 % de traction.
Une sortie detachee reste affichee a 0 Hz.

En cas de defaut, etat et les commandes refusees affichent `Cause=...`, la
broche et la frequence/duty captures AVANT coupure. Le premier motif est
conserve : les zeros consecutifs au detachement ne remplacent pas la cause.
La version actuelle **TOUT_MOTEUR_4** conserve ce correctif et deplace le
servo sur D7/GPIO12. Elle separe les D/R sur D1/D4 et permet de regler chaque roue independamment.
Les broches et inversions sont dans src/test_tout_moteur_broches.h.
Le deplacement des fils decrit plus haut est necessaire pour cette version.

Le controleur actionneurs.cpp etant partage avec le main, ce programme garde
l'identifiant TOUT_MOTEUR_4 mais accepte aussi les commandes ROVER_3 `p`, `hb`,
`wd`, `inv` et `tel` (voir le [guide Raspberry](CONNEXION_RASPBERRY.md#extension-rover_3)).
`imu` et `lacet0` repondent que la centrale est absente de ce programme. Sans ces
commandes, son comportement est celui de TOUT_MOTEUR_4 ; ses lignes USB passent
desormais par une file, sans attente de l'USB dans la boucle de commande.

La progression servo est non bloquante et mise a jour toutes les 20 ms ; la
console et la traction restent traitees pendant le mouvement. Une perte USB
detectee pendant au moins 250 ms coupe les commandes, sans reprise automatique.
Cela n'est pas une garantie de detection de fermeture du moniteur serie.

Les tests PC `python tests/verifier_policy.py` verifient les commandes,
l'independance traction/servo, l'interruption, le changement de vitesse sans
saut de consigne, les limites et le debordement de millis. Ils ne remplacent
pas l'essai physique des deux actionneurs ensemble.

Validation PC du 18 octobre 2026 : compilation `test_tout_moteur` réussie,
91 vérifications combinées et 22 régressions de lecture PWM réussies ; les contrôles existants restent réussis
(213 / 292 / 310 / 28 / 23). Aucun port série ouvert pour cette préparation.
