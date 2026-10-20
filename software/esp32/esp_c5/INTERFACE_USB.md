# Pilotage depuis le navigateur, par USB

Lancer dans ce dossier :

```powershell
python lancer_interface_usb.py
```

La page est servie uniquement sur ce PC, a http://127.0.0.1:8767/.
Ouvrir cette adresse dans **Chrome ou Edge**. Aucun Wi-Fi, compte, service
externe ou installation de module Python n'est necessaire. Le serveur ne lit
que le dossier `interface_usb/` et n'ouvre pas lui-meme le port serie.

1. L'ESP doit avoir le firmware **esp_c5 / ROVER_3** (ou ROVER_2) ou **test_tout_moteur / TOUT_MOTEUR_4**.
2. Fermer le moniteur serie PlatformIO, puis cliquer **Connecter l'ESP**.
3. Choisir le port de l'ESP32-C5 dans le selecteur du navigateur.
4. Regler une valeur et cliquer sur **Appliquer la vitesse** ou **Appliquer l'angle**.

La page ne fait qu'envoyer `etat` a la connexion. Le firmware recentre deja
le servo au demarrage ; la traction reste a zero sans commande explicite. Les deux drivers recoivent le meme pourcentage.
Servo sur D7/GPIO12, traction D0/D9, D/R separes sur D1/D4 ; voir [le cablage du test combine](TEST_TOUT_MOTEUR.md).

Le firmware actuel recentre le servo au demarrage (1500 us). Pour regler le zero, deconnecter la tringlerie, puis cliquer
sur **Recentrer a 0** : consigne de 1500 us. Le premier mouvement peut etre
rapide, car la position physique n'est pas connue. Couper ensuite l'alimentation
servo pour repositionner le palonnier sans tourner l'axe, direction tout droit.

Le bouton **Moteurs a zero** conserve le signal de direction. **STOP** ou
**Echap** envoie `stop` : PWM traction nul et signal servo coupe. Le bouton
STOP supprime aussi les commandes qui attendent encore leur envoi dans la page.
Les positions affichees sont des consignes, pas des mesures de position reelle.
Les blocs Gauche et Droite reglent chaque pourcentage independamment.
Avant/Arriere change uniquement le sens de la roue choisie, roue a 0 % et
arret physique requis. Corriger l'avant inverse le reglage de reference de
cette roue et le memorise ; les deux moteurs doivent etre a zero.
ROVER_1 et TOUT_MOTEUR_3 sont refuses par cette page mise a jour : televerser
ROVER_3 ou TOUT_MOTEUR_4 pour les nouveaux controles. ROVER_3 est accepte car il
garde les commandes de ROVER_2 ; la page n'utilise pas `p`, `wd`, `tel` ni `imu`.
Si le Pi a laisse `tel` ou `imu` actifs, envoyer `tel 0` et `imu 0` (ou rebrancher
l'USB) pour retrouver l'etat periodique ; `wd 0` si le chien de garde est reste arme.

Le navigateur lit les messages serie et bloque les controles de mouvement si
le firmware n'est pas reconnu ou si l'ESP annonce un defaut. Le retour serie
reste visible, avec le motif du defaut. Une connexion refusee indique souvent
que VS Code ou un autre logiciel utilise encore le port.

STOP est demande avant une deconnexion et lorsque l'onglet devient masque.
La fermeture brutale du navigateur, une panne du PC ou une liaison bloquee
ne garantissent pas la livraison de ce message : utiliser STOP avant de
quitter la page. Cela ne coupe pas les alimentations et ne remplace pas une
coupure physique du rover. Il n'y a pas de commande de mouvement automatique
apres reconnexion et la page ne modifie pas le firmware.

Pour fermer le serveur lance dans un terminal : Ctrl+C. Si le serveur tourne
deja, relancer le script rouvre la page existante. `--no-browser` le lance sans
ouvrir de fenetre ; `--port 8768` permet de changer son port HTTP.

Implementation : [API Web Serial documentee par Chrome](https://developer.chrome.com/docs/capabilities/serial).
Tout le HTML/CSS/JavaScript est local ; aucune ressource distante n'est chargee.

Verification du 18 octobre 2026 : 4 tests du transport avec `node --test
tests/interface_usb.test.mjs`, test navigateur Chrome avec port serie simule
(connexion sans mouvement, commandes m/s/v, STOP, defaut, firmware incompatible,
affichage mobile). Aucun acces a un port USB reel ni mouvement physique pendant
les verifications. Le script `tests/interface_usb_browser_test.mjs` utilise
Playwright installe dans le cache npm et la variable PLAYWRIGHT_MODULE vers
son fichier index.mjs ; il ne demande jamais un vrai port.
