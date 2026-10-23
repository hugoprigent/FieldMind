**Français** · [English](README.en.md)

# Logiciel du rover FieldMind

Le logiciel qui tourne **sur le Raspberry Pi du rover** : pilotes du matériel, sécurité,
conduite, odométrie lidar, stéréo, navigation, missions, agent IA avec commande vocale, et le
poste opérateur web. Il remplace la supervision `software/embarque` (mêmes ports, mêmes
caméras). Architecture et interfaces : [ARCHITECTURE.md](ARCHITECTURE.md) ; messages du poste :
[PROTOCOLE.md](PROTOCOLE.md).

État du prototype au 23 octobre : [évolutions récentes et limites](../../docs/status-2026-10-23.md).
Prochain essai : [mission caisse rouge, photo et retour à la base](TEST_TERRAIN_2026-10-22.md).

## Essayer sur le PC, sans le rover

Le matériel est simulé (monde 3D, faux ESP au protocole du firmware, lidar, paire stéréo
rendue, GPS) ; le reste est le code du rover.

```bash
cd software/rover
uv venv --python 3.13 .venv
uv pip install --python .venv/bin/python numpy scipy opencv-python-headless pyserial aiohttp \
  pytest pytest-asyncio ruff
.venv/bin/python -m fieldmind --materiel simulation          # monde « salle » ; --scenario jardin
```

Puis <http://127.0.0.1:8080/>. Démo du poste sans serveur : `fieldmind/web/index.html?mock=1`.

Tests : `.venv/bin/python -m pytest` ; style : `.venv/bin/ruff check fieldmind tests`.

### Serveur de la simulation publique

Chaque navigateur obtient son propre rover simulé, sa carte et ses réglages ; aucune interface
avec le matériel réel n'est initialisée. Le serveur choisit le GX10 s'il est joignable, puis
Gemini si une clé est configurée, sinon fonctionne sans IA :

```bash
.venv/bin/python -m fieldmind.public_demo --hote 127.0.0.1 --port 8085
```

Ouvrir <http://127.0.0.1:8085/>. Le service est prévu pour être placé derrière un proxy TLS
avant toute exposition publique. Sa configuration de déploiement n'est pas incluse ici.

## Ce que fait le logiciel

- **Drone** : [reconnaissance sans vol et visée de la croix](DRONE.md), six captures,
  montage et rapport téléchargeables. Le vol autonome reste bloqué tant que le
  protocole moteur de la variante CooleerX n'est pas validé.
  L'analyse hors ligne de l'application d'origine a permis d'identifier une trame de commande,
  mais le flux RC n'est pas raccordé ni validé au banc ; aucune commande réelle n'est activée.
  [Atelier local de la mission complète](DRONE_MISSION.md) : décollage et tour avec
  photos, caméra du bas et descente par paliers simulés, manette et préparation GX10.

- **Conduite sûre** : sécurité à 20 Hz sur le Pi (couloir le long de l'arc prévu, lidar +
  stéréo + mémoire des obstacles bas, capteurs périmés, inclinaison, choc, blocage, homme mort,
  zone autorisée GPS, ralenti près d'une personne détectée) ; firmware ROVER_4 avec chien de garde.
- **Direction** : après la panne du servo DT996, le rover réel utilise le différentiel des deux
  roues arrière ; le logiciel conserve aussi le support de la direction servo. Une adaptation SG90
  a été modélisée, sans validation sous charge.
- **Où il est** : odométrie lidar (ICP) + gyroscope, carte d'occupation, recalage sur le GPS
  (position et cap vrais).
- **Missions** : avancer, tourner (avec manœuvre), aller à un point (carte, image, GPS), lieux
  mémorisés, chercher un objet (vision), exploration progressive, photo, calibrations automatiques.
- **Suivre une personne** : bouton *Suivre* puis clic sur elle dans Pilotage, ou « suis la
  personne en t-shirt rouge ». Désignation par le GX10 (RoboBrain 2.5, ou SAM 2.1 pour un clic),
  suivi image par image sur le Pi (VitTrack, ≈ 10 Hz), distance par le lidar ; arrêt si elle est
  perdue de vue, reprise si une détection lui ressemble. La sécurité s'applique comme toujours.
- **Remorque porte-drone** (facultative, Réglages > Remorque) : angle d'attelage estimé par
  l'odométrie, emprise de la remorque dans le couloir de sécurité, vitesses et freinage adaptés ;
  marche arrière lente, en manuel seulement, refusée si la remorque se mettrait en portefeuille
  ou si son angle est incertain (avancer 1 m tout droit, ou « Dans l'axe » dans Pilotage).
- **IA** : consignes écrites ou dites (micro d'une webcam), vision, profils Gemini / GX10 / Mac
  ([MODELES.md](MODELES.md)). Sur le GX10 : agent Qwen3.6, pointage RoboBrain 2.5, détection
  continue, profondeur, contours, voix du rover, mémoire des lieux, et un **agent de veille** qui
  analyse la situation en continu (on lui parle : « qu'est-ce qui se passe ? »).
- **Poste web** : pilotage, vue 3D façon Tesla, carte locale et carte GPS OpenStreetMap,
  mission et IA, capteurs, système, réglages.
- **Boîte noire** : `python -m fieldmind.enregistreur` relit un essai. **ROS 2** : `ros2/`.

## Modèles d'IA

Clés dans `software/rover/.env` (jamais dans git, modèle commenté : `.env.exemple`), copiées
sur le Pi par le déploiement ; la clé Gemini peut aussi être saisie dans le poste (Réglages > IA).
Vérification des modèles : `.venv/bin/python deploiement/gx10/verifier.py --profil gemini`.

Profils dans [config/ia.toml](config/ia.toml) : `gemini` (en ligne : Gemini 4 Argon pour l'agent,
Gemini Robotics ER 2 pour la vision, transcription Gemini), `gx10` (modèles locaux de l'ASUS
Ascent GX10, voir [deploiement/gx10/](deploiement/gx10/README.md)), `aucun`. Le choix se fait dans
le poste, menu Réglages. La sécurité ne dépend d'aucun modèle : sans IA, le rover se pilote,
évite les obstacles et exécute les actions du poste.

## Mettre sur le rover

Rien n'est installé tant qu'on ne lance pas ces scripts. Depuis `software/rover/deploiement` :

1. Une fois : `ssh utilisateur@adresse-du-pi 'bash -s' < installer_pi.sh` (paquets apt, groupes, UART du GPS).
2. **Firmware ESP ROVER_4** (chien de garde et sorties de volets) : téléverser `software/esp32/esp_c5` (environnement
   `esp_c5`). Avec l'ancien ROVER_2, le logiciel fonctionne mais plafonne la traction à 20 %.
3. `./deployer.sh utilisateur@adresse-du-pi` : copie le code, arrête `fieldmind-embarque`, installe et démarre
   le service `fieldmind-rover`. Poste : `http://<ip du Pi>:8080`.

Journal : `ssh utilisateur@adresse-du-pi journalctl _SYSTEMD_USER_UNIT=fieldmind-rover.service -f -o cat`.

## Premiers essais (dans cet ordre)

1. **Roues levées**, mode *banc* : Système > Calibration > *Sens des roues* ; corriger avec
   `inverseg` / `inversed` dans la console si une roue recule.
2. Rover posé, immobile : *Centrale au repos* (vérifie les axes de la MPU-6050).
3. **Stéréo** : imprimer un damier 9 × 6 (cases de 25 mm), une vingtaine de captures sous des
   angles variés, puis *Calculer*.
4. 2,5 m libres devant : *Vitesse* (PWM → m/s), puis *Braquage* (angle du servo → roues).
5. Mode *manuel*, armer, rouler doucement ; vérifier l'arrêt devant un carton (lidar) et devant
   un objet bas, sous 30 cm (stéréo).
6. Mode *auto* : « avance de 2 mètres », « tourne à gauche », clic sur l'image, puis la voix.

## Réglages

### Exploration et carte mentale sur le GX10

« Explore la pièce » ou le bouton **Explorer la pièce** lance une mission progressive. Le Pi
calcule plusieurs prochains points dans l'espace connu, avec la place nécessaire pour le rover
et un chemin accessible. Le GX10 reçoit ces candidats, la mission, les objets et les murs
observés, puis choisit la prochaine étape. Le Pi revalide ce choix et exécute la navigation
locale. À l'arrivée, une nouvelle observation enrichit la carte et la boucle recommence.
« Explore pour trouver la porte » ajoute une recherche ; un objet mémorisé est revérifié avec
la caméra avant d'être déclaré retrouvé. Une recherche locale infructueuse peut aussi ouvrir
une exploration. Les points visités, refusés et à venir sont visibles sur la carte.

En profil GX10, la caméra alimente aussi la compréhension de la scène en continu, même à
l'arrêt : `ia.gx10_cartographie = true`, intervalle visé `ia.gx10_periode_s = 3`. Une seule
analyse de scène est en cours ; si une inférence dure plus longtemps, le rover reprend avec
l'image récente suivante. Chaque image emporte sa pose et ses mesures lidar/stéréo figées
avant l'envoi. D-FINE fournit en parallèle les détections rapides ; la même analyse visuelle
alimente la carte et le résumé de veille, sans deuxième appel au modèle pour la même vue.
Le modèle reçoit à chaque décision le contexte spatial actualisé. Sur l'essai réel du 21/10,
une analyse prenait environ 6 à 7 s : l'intervalle de 3 s reste un objectif, pas un débit garanti.

Un objet doit être revu sur deux captures analysées par le modèle visuel, espacées d'au
moins 0,5 s, pour devenir un repère confirmé ; le détecteur rapide seul ne le confirme pas.
Les hypothèses expirent après 45 s sans observation visuelle, les objets statiques confirmés
après une heure et les mobiles après cinq secondes. Les segments de murs mesurés restent
en mémoire de session une heure. Une destination comme « cuisine » exige plusieurs objets
distincts et caractéristiques, confirmés dans la même zone ; une pièce ambiguë est refusée.
Les annotations sont affichées en 2D et en 3D ; les portes,
fenêtres et façades peuvent être nommées par le modèle. Les positions et dimensions gardent
leur provenance : une projection sur le sol ou une hauteur visuelle restent des estimations.
Cette représentation est schématique, sans maillage 3D dense ni géométrie inventée derrière
les portes. La remise à zéro de l'odométrie efface aussi cette mémoire et invalide les
réponses d'images antérieures. La mémoire ne se recharge pas dans un nouveau repère au boot.

Par défaut, une exploration est limitée à 20 étapes, 8 m autour du départ de la mission et
10 minutes. Les limites atteintes et zones inaccessibles sont annoncées comme une exploration
partielle ; aucun chemin ou objet absent n'est présenté comme trouvé. Une indisponibilité du
GX10 arrête la progression au lieu d'exécuter un choix inventé. Pause, annulation, désarmement
et urgence invalident les décisions en vol. Le superviseur et le chien de garde restent sur le Pi.

La liaison utilise `FIELDMIND_GX10_HOTE` (par exemple l'adresse Tailscale du GX10) et
`GX10_API_KEY` dans le fichier privé `.env`, copié dans `~/.config/fieldmind/env` au déploiement.
Le programme du Pi y accède par son lien `.env`. Ces fichiers ne sont jamais versionnés.

### Réglages du poste

Les réglages du poste sont enregistrés sur le Pi (`~/.config/fieldmind/reglages.json`) et
retrouvés à chaque démarrage. Réglages > Poste > **Tout réinitialiser** revient aux valeurs de
`config/rover.toml`, en gardant le branchement, le montage et les calibrations (`esp.*` dont
`esp.moteurs_permutes` et `esp.sens_inverse`, ports et inclinaison des caméras, axes de la
centrale, PWM par m/s, servo).

Les réglages locaux du Pi peuvent différer des valeurs du dépôt. La configuration livrée
active la stéréo (`perception.stereo_active = true`, largeur 640 px, objectif 4 images/s),
avec un rôle indicatif (`securite.stereo_bloquante = false`). Elle était également active
au dernier essai du 21–22/10 ; l'ancienne note « stéréo coupée » ne décrit plus cet état.
Le relevé matériel du 21/10 indiquait des caméras horizontales et une entrée VR des drivers
alimentée en 3,3 V par l'ESP : 100 % de PWM correspondait à environ deux tiers de la vitesse
maximale au potentiomètre. Conserver les calibrations du montage réel lors d'un déploiement.

## Volets de la remorque

Deux servos **à rotation continue** sur **D2** (gauche) et **D3** (droit) de l'ESP, firmware
**ROVER_4** (`volg`, `vold` : 90 = arrêt, au-dessus un sens, en dessous l'autre, -1 coupe le
signal). Sans capteur de position, déplier ou replier = tourner à `remorque.volets_vitesse` %
le temps de faire `remorque.volets_angle_deg` (90°), calculé d'après
`remorque.volets_tours_min` (vitesse réelle à fond : volet trop loin → l'augmenter), puis arrêt ; rien n'est envoyé au démarrage, et rien ne
bouge si l'état demandé est déjà l'état connu. Le sens de chaque volet s'inverse dans
Réglages > Remorque. Pilotage > **Déplier / Replier les volets**, ou panneau Drone > **Prêt au
décollage** (aucune commande n'est envoyée au drone, dont le protocole moteur reste à
identifier : [DRONE.md](DRONE.md)). Dépliés, le gabarit de la remorque utilisé par la sécurité
s'élargit. D2/D3 ne lisent plus alors les capteurs de vitesse des roues.

## Centrale inertielle montée de biais

Système > Calibration > **Centrale au repos**, rover posé **à plat** et immobile 3 s : la
gravité mesurée définit la verticale (`imu.niveau`), quel que soit l'angle du GY-521. Roulis,
tangage et vitesse de lacet sont alors corrects ; le lacet intégré par le firmware (autour de
l'axe penché du capteur) n'est plus utilisé, le Pi intègre la vitesse de lacet.

## Sécurité

- Démarrage en mode *arrêt*, désarmé. Le réglage `conduite.armer_auto` arme au passage en
  *manuel* ou *auto*, ainsi qu'à la levée explicite de l'arrêt d'urgence. Il n'arme pas
  automatiquement en mode *banc*. Désactivé par défaut, activé sur le Pi pour les essais.
  La levée de l'urgence efface les anciennes consignes : une nouvelle commande est nécessaire.
- Chaque consigne (poste, mission, IA, ROS 2) passe par le superviseur : couloir balayé le long
  de l'arc prévu, distance d'arrêt, capteurs périmés, inclinaison, choc, blocage, homme mort,
  limite de zone.
- **Seul le lidar arrête** (`securite.stereo_bloquante = false`, par défaut) : un obstacle bas
  vu par la stéréo dans le couloir ne fait que ralentir (`securite.vitesse_obstacle_bas_m_s`),
  et la mémoire de ces obstacles s'efface en quelques secondes s'ils ne sont pas revus
  (`cartographie.oubli_bas_s`). En navigation, les détections isolées sont filtrées et les amas
  rendent le passage plus coûteux, sans l'interdire : le rover préfère l'endroit le plus dégagé.
  Le lissage conserve ce détour. `navigation.penalite_indicatif` règle le coût sur la carte et
  `navigation.poids_indicatifs` celui des arcs locaux. En mode `securite.stereo_bloquante = true`,
  les obstacles bas redeviennent bloquants.
- Manuel avec l'anticollision coupée : obstacles et détection de « blocage » ignorés (roues
  levées), vitesse plafonnée par `securite.vitesse_sans_anticollision_m_s`.
- Firmware ROVER_4 : sans consigne pendant 400 ms, traction à zéro. Les sorties D2/D3 des volets
  restent arrêtées au démarrage ; leur mouvement temporisé n'est pas un retour de position.
- Bouton ARRÊT (ou Espace) : arrêt verrouillé, réarmement explicite. « Stop » à la voix : arrêt
  immédiat.
- **Il n'y a pas encore d'arrêt d'urgence matériel** (coupure de la batterie de traction) :
  c'est à câbler avant de rouler près de quelqu'un.

### Français / English et rendu des caméras simulées

Le bouton **FR / EN**, en haut à droite, traduit le poste sans perdre les réglages ou la mission. Au premier passage, la langue préférée du navigateur est utilisée ; un lien `?lang=en` ou `?lang=fr` force la langue. Les codes du protocole restent identiques.

En simulation, les deux caméras peuvent être rendues en WebGL dans le navigateur puis renvoyées au rover simulé. Elles utilisent la pose vraie et la géométrie des caméras, avec une date commune à chaque paire stéréo. Le rendu logiciel reste disponible si WebGL échoue ou si aucun navigateur ne fournit d'images. Aucun matériel réel n'est commandé par ce canal. Voir [PROTOCOLE.md](PROTOCOLE.md).
