// Catalogue des libellés visibles, y compris confirmations et états de sécurité.
// Les clés du protocole, les noms de fichiers et les entrées de l'opérateur ne changent pas.
export const anglais = Object.fromEntries(`
FieldMind — poste|FieldMind — control station
Parler au rover : appuyer, puis parler ; chaque phrase part à l'agent, qui exécute|Talk to the rover: press, then speak. Each sentence is sent to the agent for execution
Parler|Speak
Réarmer|Reset emergency stop
Arrêt d’urgence|Emergency stop
ARRÊT|STOP
Menus|Navigation
Pilotage|Driving
Carte|Map
Capteurs|Sensors
Système|System
Réglages|Settings
Cliquer pour échanger la caméra principale (Rover ↔ Drone)|Click to swap the main camera (Rover ↔ Drone)
Caméra Drone|Drone camera
Carte GPS|GPS map
Mini-carte GPS|GPS mini-map
Vue|View
Gauche|Left
Droite|Right
Profondeur|Depth
Caméra 3D|3D camera
Suivie|Follow
Dessus|Top
Libre|Clearance
Double caméra simultanée (Rover + Drone)|Two simultaneous cameras (Rover + Drone)
Suivre une personne : cliquer ensuite sur elle dans l'image|Follow a person: then click on them in the image
Suivre|Follow
Vitesse|Speed
Braquage|Steering
Limite|Limit
Sécurité|Safety
Remorque|Trailer
La remorque est droite, dans l'axe du rover : l'angle repart de 0 et la marche arrière est permise|The trailer is straight behind the rover: reset its angle to 0 and allow reversing
Dans l'axe|Aligned
Volets de la remorque (servos D2/D3) : angles dans Réglages › Remorque|Trailer flaps (D2/D3 servos): angles in Settings › Trailer
Volets|Flaps
Atelier drone — simulation locale complète|Drone workshop — full local simulation
Décollage, tour avec photos, caméra du bas, recentrage et descente par paliers. Les commandes ci-dessous agissent sur un drone simulé.|Takeoff, photo survey, downward camera, centering and stepped descent. These controls operate a simulated drone.
Scénario|Scenario
Scénario du drone simulé|Simulated drone scenario
Conditions normales|Normal conditions
Dérive / vent modéré|Drift / moderate wind
Croix perdue pendant la descente|Cross lost during descent
Perte vidéo|Video loss
Caméra du bas bloquée|Downward camera stuck
Perte de l'estimation de position|Position estimate lost
Réponse GX10 trop ancienne|Stale GX10 response
Simuler la mission complète|Simulate the full mission
Essayer le pilotage manuel|Try manual flight
Décoller en simulation|Simulated takeoff
Caméra du bas simulée|Simulated downward camera
Interrompre et poser en simulation|Abort and land in simulation
Manette du drone simulé : maintenir les boutons|Simulated drone controls: hold the buttons
Monter|Ascend
Descendre|Descend
Avancer|Move forward
Reculer|Reverse
Tourner à gauche|Turn left
Tourner à droite|Turn right
Prêt à simuler|Ready to simulate
Prêt à simuler, même sans drone et sans Raspberry Pi|Ready to simulate, even without a drone or Raspberry Pi
Vue du drone simulé, distincte du flux réel|Simulated drone view, separate from the real stream
Photos simulées|Simulated photos
Rapport de simulation|Simulation report
Liaison|Connection
Flux|Stream
Batterie|Battery
Détectés|Detected
Essai sans vol : attente|No-flight test: waiting
Croix|Cross
Vision : inactive|Vision: inactive
Ouvrir le montage|Open the photo montage
Rapport de l'essai|Test report
Nom BLE / Wi-Fi (ex: CooleerX)|BLE / Wi-Fi name (e.g. CooleerX)
Nom BLE ou Wi-Fi du drone|Drone BLE or Wi-Fi name
Forcer la connexion à ce drone|Force connection to this drone
Connecter|Connect
Afficher le flux vidéo du drone en grand|Show the drone video stream full size
Flux Drone|Drone stream
Échanger la caméra Rover et la caméra Drone|Swap rover and drone cameras
Basculer entre la caméra avant et la caméra dessous du drone|Switch between the front and downward drone cameras
Cam. drone 1/2|Drone cam. 1/2
Relancer la détection BLE + Wi-Fi|Restart BLE + Wi-Fi discovery
Connexion automatique BLE CooleerX / Wi-Fi|Automatic CooleerX BLE / Wi-Fi connection
Six vues prises à la main, puis essai de détection de la croix. Aucune commande moteur.|Six manually captured views, then cross detection. No motor commands.
Reconnaissance sans vol|No-flight survey
Tourner le drone à la main d'environ 60°, puis capturer la vue suivante|Turn the drone by hand about 60°, then capture the next view
Capturer vue|Capture view
Détection de la croix et écarts dans l'image, sans déplacement|Cross detection and image offsets, without movement
Viser la croix|Locate the cross
Arrêter l'essai|Stop the test
Ouvre lentement les volets de la remorque et les tient ouverts (aucune commande au drone)|Slowly open and hold the trailer flaps (no drone commands)
Prêt au décollage|Ready for takeoff
Contrôleur de vol et protocole moteur à identifier et valider|Flight controller and motor protocol still need identification and validation
Vol indisponible|Flight unavailable
Requête expérimentale ; exécution par le drone non confirmée|Experimental request; execution by the drone is unconfirmed
Demander atterrissage|Request landing
Requête d'arrêt expérimentale ; ne remplace pas une coupure matérielle|Experimental stop request; does not replace a hardware power cutoff
Demander arrêt|Request stop
Lidar, vue de dessus|Lidar, top view
Portée|Range
Mode|Mode
Arrêt|Stopped
Manuel|Manual
Armer|Arm
Manuel seulement : ignorer les obstacles et conduire à vue (vitesse plafonnée). Revient tout seul au changement de mode.|Manual mode only: ignore obstacles and drive by sight (speed limited). Resets when the mode changes.
Anticollision : active|Collision avoidance: on
Joystick|Joystick
Espace|Space
0,5 m|0.5 m
Mode vocal|Voice mode
Consigne|Instruction
Envoyer|Send
Agent de veille : analyse continue de la caméra et des capteurs|Monitoring agent: continuously analyze the camera and sensors
Reprendre|Resume
Annuler|Cancel
Explorer la pièce|Explore the room
Retour au départ|Return to start
Départ|Start
Caméra|Camera
Revenir à l'image en direct|Return to the live image
Direct|Live
Que vois-tu ?|What can you see?
Détecter les objets (IA + Lidar + Stéréo)|Detect objects (AI + Lidar + Stereo)
Scanner|Scan
Scanner 3D|3D scan
Chercher…|Search…
Objet à chercher|Object to find
Chercher|Find
Carte locale : cliquer pour donner un but|Local map: click to set a goal
Locale|Local
Clic sur la carte|Map click
But|Goal
Parcours|Route
Zone|Area
cap|heading
qualité|quality
Pose vraie (simulation)|Ground-truth pose (simulation)
vrai|true
sat.|sat.
précision|accuracy
alignement|alignment
Zoom avant|Zoom in
Zoom arrière|Zoom out
Suivre le rover|Follow the rover
Tout voir|Fit all
Remise à zéro de l’odométrie|Reset odometry
Lieux|Places
Nom|Name
Nom du lieu|Place name
Mémoriser la position actuelle|Save the current position
Ici|Here
Retirer le dernier point|Remove the last point
Effacer le brouillon|Clear the draft
Boucler le parcours|Loop the route
Boucle|Loop
Retirer la zone du rover|Remove the rover boundary
libre|free
occupé|occupied
trajet|track
chemin|path
Lidar, vue polaire|Lidar, polar view
Caméras & Drone|Cameras & Drone
Accélération|Acceleration
Gyroscope|Gyroscope
Cap|Heading
Centrale|IMU
Centrale GY-521 (MPU-6050) non détectée sur le bus I2C de l'ESP : vérifier son alimentation (VCC, GND) et SDA → D5, SCL → D6. Les courbes se remplissent dès qu'elle répond.|GY-521 (MPU-6050) not detected on the ESP I2C bus: check power (VCC, GND) and SDA → D5, SCL → D6. The plots update when it responds.
Stéréo|Stereo
Fréquences|Rates
État|Status
Fréq.|Rate
Âge|Age
Journal|Log
Succès|Success
Alerte|Warning
Erreur|Error
Source|Source
Toutes|All
Filtrer|Filter
Filtrer le journal|Filter the log
Console ESP|ESP console
ligne|line
Ligne vers l’ESP|Line to the ESP
Calibration|Calibration
Banc|Bench
Essais moteurs roues levées : commandes brutes, sans sécurité d'obstacles.|Motor tests with wheels lifted: raw commands, without obstacle protection.
Passer en banc|Enter bench mode
Maintenir|Hold
Zéro|Zero
IA|AI
Fournisseur IA|AI provider
Clé API Gemini (GEMINI_API_KEY)|Gemini API key (GEMINI_API_KEY)
Clé API Gemini|Gemini API key
Enregistrer|Save
Caméras|Cameras
Gauche / droite|Left / right
Inverser|Swap
Voix|Voice
Lire les réponses|Read responses aloud
Poste|Control station
Thème|Theme
Sombre|Dark
Clair|Light
Jeton|Token
Tuiles|Tiles
aucune|none
Réglages du rover|Rover settings
Branchement, montage et calibrations gardés|Wiring, mounting and calibrations are kept
Tout réinitialiser|Reset all
Environnement|Environment
Carte, objets vus, position, missions et conversation de l'agent effacés ; réglages, calibrations et lieux nommés gardés|Clear the map, observed objects, position, missions and agent conversation; keep settings, calibrations and named places
Calibration stéréo avec le damier|Stereo checkerboard calibration
Fermer|Close
damier non vu|checkerboard not detected
0 paire|0 pairs
Zones de l'image couvertes|Image coverage
Capture dès que le damier est vu des deux caméras, immobile, à un nouvel endroit|Capture when both cameras see the stationary checkerboard in a new position
Capture auto|Auto capture
Capturer|Capture
Tout effacer|Clear all
Calculer|Compute
Pas de position GPS|No GPS position
liaison coupée|connection lost
pas de réponse|no response
Demande refusée|Request refused
ARRÊT non transmis|STOP was not transmitted
Liaison perdue|Connection lost
Vitesse max en manuel|Maximum manual speed
Vitesse max en auto|Maximum auto speed
Vitesse max en arrière|Maximum reverse speed
Vitesse des actions|Action speed
Zone morte|Dead zone
Délai d’inversion|Reversal delay
Équilibre droite / gauche|Right / left balance
Tenue de cap|Heading hold
Impulsions par mètre|Pulses per metre
Servo par degré de roue|Servo per wheel degree
Neutre du servo|Servo neutral
Butée du servo|Servo limit
Marge avant|Front margin
Décélération|Deceleration
Perte de l’opérateur|Operator loss
Rayon autorisé|Allowed radius
Port gauche|Left port
Port droite|Right port
Axe x|X axis
Axe y|Y axis
Axe z|Z axis
Périphérique ALSA|ALSA device
Je m’arrête.|Stopping.
je reviens au départ|returning to start
Je n’ai pas compris la consigne.|I did not understand the instruction.
refusé|refused
calibration absente : modèle par défaut|no calibration: default model
Rover prêt.|Rover ready.
arrêt d’urgence actif : réarmer d’abord|emergency stop active: reset it first
armé : traction autorisée|armed: drive enabled
désarmé|disarmed
ARRÊT D’URGENCE (poste)|EMERGENCY STOP (control station)
arrêt d’urgence levé|emergency stop cleared
zone autorisée retirée|allowed area removed
position GPS pas encore alignée : rouler quelques mètres|GPS position not aligned yet: drive a few metres
odométrie remise à zéro, carte effacée|odometry reset, map cleared
écoute activée|listening enabled
écoute coupée|listening disabled
IA désactivée|AI disabled
rover armé : console refusée hors banc|rover armed: console only allowed in bench mode
rotation relancée|rotation restarted
rotation arrêtée|rotation stopped
gauche et droite échangées|left and right swapped
coordonnées invalides|invalid coordinates
but à plus de 500 m|goal over 500 m away
but hors de la zone autorisée|goal outside the allowed area
photo gauche enregistrée|left photo saved
file d’actions terminée|action queue completed
[MOTEURS] arrêt|[MOTORS] stopped
passer en banc (roues levées)|enter bench mode (wheels lifted)
butée gauche…|left stop…
butée droite…|right stop…
calibration enregistrée|calibration saved
enregistré|saved
Rover désarmé : plan en attente.|Rover disarmed: plan queued.
à gauche|left
à droite|right
Je vois l’herbe et la clôture du jardin.|I can see grass and the garden fence.
tourne à gauche|turn left
Désarmé|Disarmed
Mode arrêt|Stopped mode
Obstacle devant|Obstacle ahead
Obstacle derrière|Obstacle behind
Obstacle bas|Low obstacle
Lidar absent|Lidar missing
Stéréo absente|Stereo missing
Défaut ESP|ESP fault
Inclinaison|Tilt
Choc|Impact
Blocage|Blocked
Consigne périmée|Expired command
Opérateur perdu|Operator lost
Hors zone|Outside the area
Ralenti|Slow down
Anticollision coupée|Collision avoidance off
Remorque trop braquée|Trailer jackknifing
Angle remorque incertain|Uncertain trailer angle
Pas de recul (remorque)|No reversing (trailer)
Bord de zone|Area boundary
Personne proche|Person nearby
En attente|Waiting
En route|En route
Bloqué|Blocked
Terminé|Completed
Échec|Failed
Annulé|Cancelled
En pause|Paused
Aller au point GPS|Go to the GPS point
Décrire la scène|Describe the scene
Sens des roues|Wheel direction
Roues levées ?|Wheels lifted?
Place pour tourner ?|Enough room to turn?
Centrale au repos|Stationary IMU
Stéréo (damier)|Stereo (checkerboard)
Max manuel|Manual max
Max auto|Auto max
Max arrière|Reverse max
Servo par degré|Servo per degree
Butée servo|Servo stop
Délai inversion|Reversal delay
Perte opérateur|Operator loss
Axe x centrale|IMU X axis
Axe y centrale|IMU Y axis
Axe z centrale|IMU Z axis
Estimé|Estimated
canapé|sofa
Tiens le damier bien à plat (collé sur un carton), entièrement visible des deux caméras.|Keep the checkerboard flat (on cardboard), fully visible to both cameras.
Varie : près et loin, au centre et dans les coins, penché à gauche, à droite, vers le haut.|Vary the position: near and far, centre and corners, tilted left, right and upward.
Reste immobile une seconde à chaque position : la capture automatique s'en charge.|Hold still for a second in each position: auto capture takes care of the rest.
Paires effacées|Pairs cleared
Damier pas vu des deux caméras, ou position déjà prise|Checkerboard not visible in both cameras, or position already captured
Calcul en cours…|Computing…
échec|failed
Calibration réussie et enregistrée|Calibration successful and saved
Calibration enregistrée, à vérifier|Calibration saved, needs checking
Réarmer ?|Reset emergency stop?
Réarmé|Emergency stop reset
J’écoute|Listening
désactivée|disabled
Liaison établie|Connected
Verrouillé|Locked
Arrêt d’urgence : réarmement nécessaire|Emergency stop: explicit reset required
Armé|Armed
Traction autorisée|Drive enabled
Traction interdite|Drive disabled
Drone OK|Drone OK
Lidar arrêté|Lidar stopped
Caméras : inconnu|Cameras: unknown
Stéréo : inconnue|Stereo: unknown
Stéréo non calibrée|Stereo not calibrated
Caméra gauche|Left camera
Caméra droite|Right camera
Odométrie|Odometry
odométrie|odometry
Vitesse de lacet|Yaw rate
Cap intégré (ESP)|Integrated heading (ESP)
Température|Temperature
Cap vrai|True heading
Défaut|Fault
serré|engaged
Chien de garde|Watchdog
Dernière ligne|Last line
Connecté|Connected
État liaison|Connection status
Adresse MAC / Canal|MAC address / Channel
Signal RSSI|RSSI signal
Hôte (Pi / ESP32)|Host (Pi / ESP32)
Flux vidéo|Video stream
Trafic reçu / émis|Traffic received / sent
Batterie drone|Drone battery
Commande vol|Flight command
Détectés (BLE/Wi-Fi)|Detected (BLE/Wi-Fi)
Calibrée|Calibrated
Non calibrée|Not calibrated
Fréquence|Rate
arrêté|stopped
Remettre l’odométrie à zéro ? La carte sera effacée.|Reset odometry? This will clear the map.
Odométrie remise à zéro|Odometry reset
Retirer la zone autorisée ?|Remove the allowed boundary?
Zone retirée|Boundary removed
Liaison coupée|Connection lost
Parcours envoyé|Route sent
Zone enregistrée|Boundary saved
Carte GPS : cliquer pour donner un but|GPS map: click to set a goal
Carte GPS : cliquer pour ajouter un point|GPS map: click to add a point
à faire|queued
Aucune action en cours|No action in progress
Scan 3D (IA + Lidar + Stéréo)…|3D scan (AI + Lidar + Stereo)…
Cliquer sur la personne à suivre|Click the person to follow
Photo enregistrée|Photo saved
Passer en banc ? Roues levées.|Enter bench mode? Lift the wheels.
Couper l’anticollision ? Le rover ignorera les obstacles : tu conduis à vue, vitesse plafonnée.|Disable collision avoidance? The rover will ignore obstacles: drive by sight, speed limited.
Volets dépliés|Flaps open
Volets repliés|Flaps closed
Remorque dans l’axe|Trailer aligned
Ouverture des volets…|Opening flaps…
Fermeture des volets…|Closing flaps…
Prêt au décollage ✓ — replier|Ready for takeoff ✓ — close flaps
Prêt au décollage : volets ouverts|Ready for takeoff: flaps open
Scan BLE + Wi-Fi drone lancé|Drone BLE + Wi-Fi scan started
Recherche et connexion automatique au drone activées|Automatic drone discovery and connection enabled
Bascule caméra demandée ; vérifier le changement dans la vidéo|Camera switch requested; check the video
Essai sans vol : tourner à la main puis capturer six vues|No-flight test: turn by hand, then capture six views
Capture demandée sur la prochaine image|Capture requested on the next frame
Visée de la croix modifiée — aucun déplacement commandé|Cross target changed — no movement commanded
Essai arrêté|Test stopped
Requête d’atterrissage envoyée — exécution non confirmée|Landing request sent — execution unconfirmed
Requête d’arrêt envoyée — exécution non confirmée|Stop request sent — execution unconfirmed
Caméra Drone (clic = agrandir)|Drone camera (click to enlarge)
Caméra Rover (clic = agrandir)|Rover camera (click to enlarge)
Suivi lancé|Tracking started
Aller ici ?|Go here?
Replier les volets|Close flaps
Déplier les volets|Open flaps
Passer en manuel|Switch to manual
Désarmer|Disarm
Choisir d’abord un mode|Choose a mode first
Anticollision COUPÉE : rétablir|Collision avoidance OFF: restore
Clic sur l’image : aller au point|Click the image to set a goal
Marche arrière permise (lente)|Reversing allowed (slow)
Angle incertain : avancer 1 m tout droit, ou « Dans l’axe »|Uncertain angle: move forward 1 m straight, or choose “Aligned”
Pas d’état|No status
Voie libre|Path clear
Drone : Connecté|Drone: connected
centrée|centered
détectée|detected
Croix : non détectée|Cross: not detected
Clé API Gemini enregistrée|Gemini API key saved
Clé API retirée|API key removed
Caméras inversées|Cameras swapped
Tuiles enregistrées|Tiles saved
Jeton enregistré|Token saved
Revenir aux réglages d’origine ? Branchement, montage et calibrations sont gardés.|Restore default settings? Wiring, mounting and calibrations are kept.
Effacer la carte, les objets vus, la position et les missions ? Réglages, calibrations et lieux nommés sont gardés.|Clear the map, observed objects, position and missions? Settings, calibrations and named places are kept.
Environnement réinitialisé|Environment reset
Désactivée|Disabled
Mémoire|Memory
En marche|Running
Bridée|Throttled
Roues levées ? Le rover passe en banc et fait tourner chaque roue.|Wheels lifted? The rover will enter bench mode and spin each wheel.
paire enregistrée|pair saved
Arrêter|Stop
Quitter le banc|Leave bench mode
Oui|Yes
Non|No
gauche|left
droite|right
profondeur|depth
attente|waiting
manuel|manual
auto|auto
banc|bench
terminé|completed
erreur|error
avancer|forward
reculer|reverse
Tourner|Turn
Aller à|Go to
Aller au point visé|Go to the selected point
Rejoindre|Reach
selon les observations|based on observations
Mémoriser|Remember
Attendre|Wait
Explorer pour trouver|Explore to find
personne|person
chien|dog
enfant|child
chaise|chair
rocher|rock
poteau|post
mur|wall
arbre|tree
bas|low
bloque|blocked
termine|completed
echec|failed
annule|cancelled
pause|paused
inconnu|unknown
inconnue|unknown
connecté|connected
connecte|connected
connexion|connecting
hors_ligne|offline
caméra|camera
paliers|steps
SIMULATION · IA|SIMULATION · AI
module IA non chargé|AI module not loaded
IA indisponible|AI unavailable
micro indisponible|microphone unavailable
arrêt d'urgence|emergency stop
aucun|none
Traction|Drive
Vitesses|Speeds
Roues|Wheels
Direction|Steering
DGNSS|DGNSS
RTK fixe|RTK fixed
RTK flottant|RTK float
Autonome|Standalone
Aucun|None
Muet depuis|Silent for
En veille|Monitoring
Connexion…|Connecting…
Capturer vue|Capture view
fréquence|rate
Actions|Actions
`.trim().split('\n').map(ligne => ligne.split('|')));
