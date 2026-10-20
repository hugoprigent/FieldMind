# Drone : reconnaissance et retour sur une croix

La suite du travail est préparée **en local, sans Pi ni drone allumés** :
[mission complète simulée, manette et service de vision GX10](DRONE_MISSION.md).
Cette préparation ne modifie pas le statut de validation du matériel ci-dessous.

## Matériel retrouvé et constat du 20 octobre 2026

- Pi : `rover.local`, SSH `utilisateur@rover.local`, poste <http://rover.local:8080>.
  Ces accès sont également consignés dans `software/embarque/ETAT_ACTUEL.md`.
- Achat retrouvé dans le chat **Évaluer l’atterrissage autonome surE** : annonce
  AliExpress **1005012472001870**, variante double caméra, référencée E88/E88 Pro Max.
  Le nom E800 avait été utilisé au début de cet échange ; l'étiquette commerciale
  ne garantit pas le protocole de cette variante.
- Le Pi reçoit réellement le flux **CooleerX**, MAC **B4:E8:42:8E:E7:42**,
  via BLE, MTU 247, images 240 × 320, environ 5 images/s au relevé avant mise à jour.
- UUID vidéo `07af27a9-9c22-11ea-9afe-02fcdc4e7412`, écriture
  `07af27a7-9c22-11ea-9afe-02fcdc4e7412`. Cet UUID d'écriture est aussi utilisé
  par un exemple générique du SDK Bouffalo : il ne prouve pas un accès aux moteurs.
- La réception vidéo est confirmée. Le protocole moteur, la bascule réelle du
  capteur avant/bas, l'orientation de l'image et une mesure d'altitude/cap ne le sont pas.
  On ne peut pas déduire que le BLE ne sert *qu'à* la caméra à partir de ces constats.

Le projet [e88pilot](https://github.com/Volonid/e88pilot) rapporte un pilotage
fonctionnel sur la variante Wi-Fi de son auteur, avec trames UDP et session continue
vers `192.168.28.1:7080`. Il annonce aussi l'absence de télémétrie et une bascule
caméra non vérifiée. Copier ses trames vers notre caractéristique BLE ne valide pas
leur compatibilité. [SDK BLE Bouffalo/Ai-Thinker](https://docs.ai-thinker.com/en/ai-m6x_devel/wireless/ble/ble_slave.html).

## Ce qui fonctionne dans cette version

Dans **Pilotage**, ouvrir **Flux Drone** puis :

1. **Reconnaissance sans vol** : nécessite une image réelle datant de moins de 1,5 s.
   Une mire ou une vidéo perdue ne permettent pas de démarrer.
2. Drone au banc, le tourner manuellement d'environ 60° entre chaque vue, puis
   cliquer **Capturer vue** six fois. Chaque clic attend une nouvelle image du flux.
   Aucun moteur ni changement de caméra n'est commandé par cet essai.
3. Les six JPEG, un montage 3 × 2 et `rapport.json` sont enregistrés dans
   `~/.local/share/fieldmind/reconnaissances/<date-id>/`. **Ouvrir le montage** et
   **Rapport de l'essai** deviennent disponibles dans le poste. Ce montage conserve
   les vues ; il n'est pas un panorama géométriquement recalé, et ne certifie pas 360°.
4. Après les captures, montrer une croix à la caméra pour tester le centrage.
   On peut aussi utiliser directement **Viser la croix**. Détection d'un `+` ou `X`
   rouge/orange ou sombre sur fond clair, écarts normalisés X/Y à l'écran.
5. Le centrage est confirmé après au moins cinq images nouvelles, sur au moins
   0,8 s, sans interruption de plus de 0,65 s. Une cible décentrée ou perdue annule
   immédiatement cette confirmation. « Centrée » ne signifie pas « posé ».
6. **Arrêter l'essai** arrête seulement le traitement visuel. Changer de caméra
   interrompt la visée : vérifier le nouveau flux puis relancer **Viser la croix**.
   Les fichiers de la reconnaissance restent conservés.

Les pertes d'image (> 1,5 s), de liaison et les erreurs disque interrompent l'essai.
Il ne reprend pas automatiquement à la reconnexion. Une recherche après les six
captures expire après 20 s ; l'ensemble de l'essai expire après 180 s. Le quota
sur disque est de 100 Mio : exporter/retirer des dossiers pour libérer de la place.
Le prochain essai ne détruit pas les précédents ; les liens du poste donnent accès
au dernier essai de ce processus. Les fichiers restent accessibles par SSH après redémarrage.

Les endpoints `GET /api/drone/recon/montage.jpg`, `rapport.json`, `vue-1.jpg` à
`vue-6.jpg` suivent l'authentification du poste. Aucun chemin arbitraire n'est accepté.
Les JPEG enregistrés proviennent des images sans réticule ni annotation.

## Commandes et limite actuelle

**Décollage, manette, calibration et reconnaissance en vol sont refusés côté pilote**,
y compris depuis la console série du poste. Le bouton indique **Vol indisponible**.
Ce blocage correspond au protocole moteur non validé ; il n'est pas levé par un
simple réglage de l'interface.

**Cam. drone 1/2** demande la bascule par les requêtes caméra historiques, encore
expérimentales. Le serveur accepte la commande `camera` et renvoie un refus lisible
si la liaison ne permet pas l'envoi. Il n'envoie plus de drapeau caméra dans une
trame de vol. L'interface demande de vérifier le changement dans la vidéo ; aucun
accusé de réception ne prouve quel capteur est actif.

**Demander atterrissage / Demander arrêt** restent des requêtes expérimentales.
L'interface ne les présente pas comme exécutées, ni comme un coupe-circuit validé.
Elles ne remplacent pas le contrôle manuel d'origine ou une coupure physique.
Aucun décollage, déplacement ni basculement caméra réel n'a été effectué pendant
la validation logicielle de cette version.

## Calcul et évolution vers le vol autonome

La détection géométrique utilise OpenCV, sans LLM, réseau ni clé API. Elle traite
au plus 5 images/s et 320 pixels sur le plus grand côté, dans un fil distinct du
BLE et de la série. Seule la dernière image est conservée en attente ; six captures
au maximum restent en mémoire. Le poste affiche les millisecondes d'analyse.
Le scan ne déclenche plus une analyse Gemini payante au démarrage.

Le relevé initial du Pi donne 3,3 Gio de mémoire disponible et une faible charge
(0,16 / 0,20 / 0,43). Il s'agit d'un instantané, pas d'une garantie sous charge.
Le besoin d'une autre machine ne peut pas être conclu de la taille du programme.
Pour cette détection de croix, mesurer le temps affiché et la latence du flux sur
le Pi ; réserver le GX10/Mac aux descriptions sémantiques des vues si nécessaire.
La boucle de pilotage devra rester locale et indépendante des réponses d'un LLM.

Pour rendre le scénario **décoller → 360° → repérer le plateau → descendre** réel :

- identifier et valider le transport moteur de cette variante (capture des commandes
  de l'application d'origine, hélices retirées pour le banc) et ses pertes de liaison ;
- vérifier la caméra ventrale, son orientation, sa calibration et sa latence ;
- obtenir une estimation de cap/rotation et d'altitude, ou une estimation visuelle
  suffisamment fiable : une temporisation ne mesure ni 360° ni la hauteur ;
- prévoir un contrôleur avec battement indépendant du flux vidéo, télémétrie et
  comportement de repli vérifiés ; tester les axes et gains avant de descendre ;
- passer par un simulateur puis des essais supervisés sur grande cible au sol avant
  la remorque immobile. La croix fournit un centrage 2D ; un marqueur codé de taille
  connue et une caméra calibrée permettraient une estimation de pose plus complète.
  [Documentation OpenCV](https://docs.opencv.org/4.11.0/d5/dae/tutorial_aruco_detection.html).

Cette version prépare et valide la perception au banc. Elle ne prétend pas
résoudre le contrôle physique manquant par davantage de puissance de calcul.

## Vérifier sans matériel

Depuis `software/rover` :

```sh
.venv/bin/python -m pytest -q tests/test_drone.py
.venv/bin/ruff check fieldmind tests
node --check fieldmind/web/js/vues/pilotage.js
```

## Validation et déploiement du 20 octobre

- **338 tests Python réussis**, deux scénarios longs exclus (`-m 'not lent'`),
  dont 27 nouveaux tests drone ; lint Python et 19 tests JavaScript réussis.
- Déploiement ciblé des modules drone/API/poste, sans remplacement des réglages,
  clés ou firmware. Rover vérifié arrêté, désarmé et traction nulle avant redémarrage.
  Sauvegarde Pi : `~/fieldmind/sauvegardes/drone-20261020-avant/logiciel.tgz`.
- Service `fieldmind-rover` redémarré et actif ; flux réel toujours autour de 4,9 i/s.
- Visée activée trois secondes par WebSocket sur le Pi, puis arrêtée :
  `mode_recon=ciblage_croix`, `analyse_ms=16.46`, compteur `paquets_tx=0` inchangé.
  La scène de cet essai ne contenait pas de croix détectée. Les tests de détection
  positive/centrage ont utilisé des images synthétiques ; pas de posé physique validé.
- Banc sur une image réelle, 100 mesures après échauffement, détection + annotation
  + réencodage : médiane **9,85 ms**, P95 **10,02 ms**. À cinq analyses/s cela représente
  environ 5 % du temps d'un cœur pour ces opérations, hors autres tâches du rover.
  L'indicateur temps réel inclut aussi le décodage JPEG et la concurrence du service.
  **Aucune machine supplémentaire nécessaire pour la vision géométrique actuelle**
  au vu de ces mesures ; la charge d'une future reconstruction/IA reste à mesurer.
- Vérification automatisée du poste par Browser Control indisponible :
  `Target.createTarget: Host access is restricted by policy`. Aucun contournement.
  Syntaxe JavaScript, tests et échanges HTTP/WebSocket vérifiés indépendamment.
