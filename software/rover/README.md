# Logiciel du rover FieldMind

Le logiciel qui tourne **sur le Raspberry Pi du rover** : pilotes du matériel, sécurité,
conduite, odométrie lidar, stéréo, navigation, missions, agent IA avec commande vocale, et le
poste opérateur web. Il remplace la supervision `software/embarque` (mêmes ports, mêmes
caméras). Architecture et interfaces : [ARCHITECTURE.md](ARCHITECTURE.md) ; messages du poste :
[PROTOCOLE.md](PROTOCOLE.md).

## Essayer sur le PC, sans le rover

Le matériel est simulé (monde 3D, faux ESP au protocole du firmware, lidar, paire stéréo
rendue, GPS) ; le reste est le code du rover.

```bash
cd software/rover
uv venv --python 3.13 .venv
uv pip install --python .venv/bin/python numpy scipy opencv-python-headless pyserial aiohttp \
  pytest pytest-asyncio ruff
FIELDMIND_IA_PROFIL=aucun .venv/bin/python -m fieldmind --materiel simulation          # monde « salle » ; --scenario jardin
```

Puis <http://127.0.0.1:8080/>. Démo du poste sans serveur : `fieldmind/web/index.html?mock=1`.

Tests : `.venv/bin/python -m pytest` ; style : `.venv/bin/ruff check fieldmind tests`.

## Ce que fait le logiciel

- **Drone** : [reconnaissance sans vol et visée de la croix](DRONE.md), six captures,
  montage et rapport téléchargeables. Le vol autonome reste bloqué tant que le
  protocole moteur de la variante CooleerX n'est pas validé.
  [Atelier local de la mission complète](DRONE_MISSION.md) : décollage et tour avec
  photos, caméra du bas et descente par paliers simulés, manette et préparation GX10.

- **Conduite sûre** : sécurité à 20 Hz sur le Pi (couloir le long de l'arc prévu, lidar +
  stéréo + mémoire des obstacles bas, capteurs périmés, inclinaison, choc, blocage, homme mort,
  zone autorisée GPS, ralenti près d'une personne détectée) ; firmware ROVER_3 avec chien de garde.
- **Virages** : différentiel électronique sur l'angle réel du servo, corrigé par le gyroscope ;
  capteurs de vitesse des roues (sorties SC) en option.
- **Où il est** : odométrie lidar (ICP) + gyroscope, carte d'occupation, recalage sur le GPS
  (position et cap vrais).
- **Missions** : avancer, tourner (avec manœuvre), aller à un point (carte, image, GPS), lieux
  mémorisés, chercher un objet (vision), photo, calibrations automatiques.
- **Remorque porte-drone** (facultative, Réglages > Remorque) : angle d'attelage estimé par
  l'odométrie, emprise de la remorque dans le couloir de sécurité, vitesses et freinage adaptés ;
  marche arrière lente, en manuel seulement, refusée si la remorque se mettrait en portefeuille
  ou si son angle est incertain (avancer 1 m tout droit, ou « Dans l'axe » dans Pilotage).
- **IA** : consignes écrites ou dites (micro d'une webcam), vision, profils Gemini / GX10 / Mac
  ([MODELES.md](MODELES.md)).
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

1. Une fois : `ssh pi-wifi 'bash -s' < installer_pi.sh` (paquets apt, groupes, UART du GPS).
2. **Firmware ESP ROVER_3** (chien de garde) : téléverser `software/esp32/esp_c5` (environnement
   `esp_c5`). Avec l'ancien ROVER_2, le logiciel fonctionne mais plafonne la traction à 20 %.
3. `./deployer.sh pi-wifi` : copie le code, arrête `fieldmind-embarque`, installe et démarre
   le service `fieldmind-rover`. Poste : `http://<ip du Pi>:8080`.

Journal : `ssh pi-wifi journalctl _SYSTEMD_USER_UNIT=fieldmind-rover.service -f -o cat`.

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

## Sécurité

- Démarrage en mode *arrêt*, désarmé ; rien ne bouge sans « armer ».
- Chaque consigne (poste, mission, IA, ROS 2) passe par le superviseur : couloir balayé le long
  de l'arc prévu (lidar + stéréo), distance d'arrêt, capteurs périmés, inclinaison, choc,
  blocage, homme mort, limite de zone.
- Firmware ROVER_3 : sans consigne pendant 400 ms, traction à zéro.
- Bouton ARRÊT (ou Espace) : arrêt verrouillé, réarmement explicite. « Stop » à la voix : arrêt
  immédiat.
- **Il n'y a pas encore d'arrêt d'urgence matériel** (coupure de la batterie de traction) :
  c'est à câbler avant de rouler près de quelqu'un.
