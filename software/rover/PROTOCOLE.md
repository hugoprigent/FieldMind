**Français** · [English](PROTOCOLE.en.md)

# Protocole du poste opérateur

Le rover sert le poste (`fieldmind/web/`) et son API sur `http://<rover>:8080`. Tout passe par
une **WebSocket** `/ws` (JSON) ; la vidéo passe par des flux **MJPEG** HTTP. Unités SI, repères
de [types.py](fieldmind/types.py) : `base_link` (x avant, y gauche, origine sur l'essieu arrière),
`odom` (fixe, au départ), angles en radians trigonométriques sauf les champs `_deg`.

Jeton facultatif : si le rover est lancé avec `FIELDMIND_JETON`, ajouter `?jeton=…` à l'URL de
la WebSocket et des flux (le poste le relit dans l'URL de la page et le garde en `localStorage`).

## HTTP

| Route | Rôle |
| --- | --- |
| `GET /` | le poste (fichiers statiques de `fieldmind/web/`) |
| `GET /flux/<camera>?ips=10&largeur=640` | MJPEG `multipart/x-mixed-replace` ; `camera` = `gauche`, `droite` ou `profondeur` (carte de profondeur en fausses couleurs) |
| `GET /photo/<camera>` | dernière image JPEG |
| `GET /api/etat` | dernier message `etat` (JSON), pour un essai rapide |
| `GET /api/profil` | profil complet du rover (contenu de `bienvenue.profil`) |
| `GET /api/journal?n=200` | derniers événements du journal |
| `GET /api/sante` | `{"ok": true, "version": "1.0.0"}` |

## WebSocket `/ws`

Chaque message est un objet `{"type": "...", ...}`. Le serveur envoie `bienvenue` à la connexion.
Plusieurs postes peuvent être connectés ; le dernier qui pilote en manuel a la main.

### Serveur → poste

**`bienvenue`** (une fois)
```json
{"type": "bienvenue", "version": "1.0.0", "materiel": "reel", "poste": "poste-3f2a",
 "profil": {"geometrie": {...}, "lidar": {...}, "cameras": {...}, "vitesses": {...}, "securite": {...}},
 "reglables": {"vitesses.max_manuel_m_s": [0.05, 1.5], "perception.stereo_active": [0, 1],
               "cameras.port_gauche": "texte"},
 "reglages": {"vitesses.max_manuel_m_s": 0.6},
 "reglables_details": {"direction.servo_neutre_deg": {"min": -20, "max": 20, "pas": 0.5,
     "unite": "°", "libelle": "Neutre du servo", "groupe": "Direction", "type": "nombre"}},
 "cameras": ["gauche", "droite"],
 "ia": {"profil": "gemini", "disponible": true, "modeles": {"agent": "gemini-4-argon",
        "vision": "gemini-robotics-er-2-preview", "transcription": "gemini-3.5-transcribe"},
        "profils": ["gemini", "gx10", "aucun"], "raison": null}}
```
`reglables_details` donne pour chaque réglage son libellé, son groupe, son unité, son pas et
son type (`nombre`, `booleen`, `texte`) : le poste s'en sert pour ranger le menu Réglages. Un
réglage modifié s'applique tout de suite (le neutre du servo fait bouger les roues à l'arrêt).
`materiel` vaut `reel` ou `simulation` (le poste affiche alors un bandeau « SIMULATION »).

**`etat`** (10 Hz) — tout ce qui change vite, en un seul message :
```json
{"type": "etat", "t": 1234.56,
 "mode": "manuel", "arme": true, "arret_urgence": false, "pilote": "poste-3f2a",
 "consigne": {"vitesse": 0.30, "braquage": 0.10, "source": "manuel"},
 "commande": {"pwm_g": 12, "pwm_d": 13, "servo_deg": 5.7, "vitesse": 0.28, "braquage": 0.10},
 "odom": {"x": 1.20, "y": -0.35, "cap": 0.12, "vitesse": 0.27, "vitesse_lacet": 0.05, "qualite": 0.9},
 "securite": {"arret": false, "vitesse_max_avant": 0.45, "vitesse_max_arriere": 0.30,
              "raisons": ["stereo_absente"], "libre_avant_m": 1.8, "libre_arriere_m": 3.2},
 "esp": {"connecte": true, "firmware": "ROVER_3", "defaut": false, "raison_defaut": null,
         "pwm_g": 12, "pwm_d": 13, "sens_g": 0, "sens_d": 0, "frein": false,
         "servo_actif": true, "servo_deg": 5.7, "chien_de_garde_ms": 400, "age_s": 0.04},
 "imu": {"etat": "OK", "accel_g": [0.01, -0.02, 0.99], "gyro_rad_s": [0.0, 0.0, 0.05],
         "roulis_deg": -1.1, "tangage_deg": 0.6, "temperature_c": 31.2},
 "gnss": {"fix": "autonome", "latitude": 43.71, "longitude": -1.05, "altitude_m": 12.0,
          "hdop": 1.1, "satellites": 7, "vitesse_m_s": 0.2, "cap_deg": 87.0},
 "capteurs": {"lidar": {"ok": true, "hz": 10.2, "points": 360, "age_s": 0.05, "marche": true},
              "gauche": {"ok": true, "ips": 14.8, "age_s": 0.07, "resolution": [1280, 720]},
              "droite": {"ok": true, "ips": 14.9, "age_s": 0.06, "resolution": [1280, 720]},
              "stereo": {"ok": true, "hz": 3.9, "age_s": 0.2, "calibree": true},
              "imu": {"ok": true, "age_s": 0.04}, "gnss": {"ok": true, "age_s": 0.6},
              "esp": {"ok": true, "age_s": 0.04}, "micro": {"ok": true, "nom": "UGREEN 15728"}},
 "mission": {"etat": "route", "action": {"type": "avancer", "distance_m": 5.0},
             "progression": 0.42, "detail": "2,1 m sur 5,0 m", "file": [{"type": "tourner", "angle_deg": 90}],
             "plan_id": "p7"},
 "voix": {"active": true, "parle": false, "niveau": 0.12},
 "ia": {"occupee": false}}
```
Tout champ peut valoir `null` si l'information manque (capteur absent). En simulation
seulement, `etat.simulation` = `{"x", "y", "cap", "vitesse"}` donne la pose vraie (repère du
monde simulé) ; `capteurs.odometrie` = `{"ok", "duree_ms"}` quand l'estimation tourne. `mission.etat` :
`attente`, `route`, `bloque`, `pause`, `termine`, `echec`, `annule`. `etat.remorque` vaut `null`
sans remorque attelée, sinon `{"attelee": true, "angle_deg": 12.0, "incertitude_deg": 1.5,
"recul_possible": true, "geometrie": {"attelage_m", "timon_m", "avant_m", "arriere_m",
"demi_largeur_m"}}` : angle d'attelage estimé (> 0 : remorque à gauche, derrière), planche en mètres
depuis la vis d'attelage ; raisons de sécurité associées : `remorque`, `remorque_portefeuille`,
`remorque_incertaine`, `remorque_recul_auto`. `raison_urgence` (texte ou
`null`) dit ce qui a déclenché l'arrêt d'urgence (`poste`, `choc`…). Codes de `securite.raisons` (le poste
les traduit) : `arret_urgence`, `non_arme`, `mode_arret`, `obstacle_avant`, `obstacle_arriere`,
`obstacle_bas` (stéréo), `lidar_absent`, `stereo_absente`, `esp_absent`, `esp_defaut`,
`inclinaison`, `choc`, `blocage`, `consigne_perimee`, `operateur_perdu`, `geofence`,
`ralenti_obstacle`, `banc`.

**`systeme`** (1 Hz)
```json
{"type": "systeme", "cpu_pct": 38, "temperature_c": 61.5, "memoire_pct": 42, "disque_pct": 31,
 "sous_tension": false, "ralenti": false, "throttled": "0x0",
 "wifi": {"ssid": "reseau-atelier", "signal_pct": 71, "ip": "rover.local"}, "charge": [1.2, 1.1, 0.9],
 "uptime_s": 5321, "processus_cpu_pct": 55}
```

**`scan`** (5 Hz) — points du lidar dans `base_link`, en centimètres entiers aplatis :
```json
{"type": "scan", "t": 1234.5, "xy": [120, -3, 118, 0, 117, 4]}
```

**`obstacles`** (5 Hz) — obstacles vus par la stéréo (et autres sources), `base_link`, cm :
```json
{"type": "obstacles", "t": 1234.5, "source": "stereo", "xy": [80, 10, 82, 12], "portee_m": 4.0}
```

**`carte`** (1 Hz) — grille d'occupation du repère `odom` en PNG (niveaux de gris : 0 noir =
occupé, 255 blanc = libre, 128 = inconnu ; ligne 0 du PNG = **y maximal**, image prête à dessiner) :
```json
{"type": "carte", "t": 1234.5, "resolution": 0.05, "origine": [-20.0, -20.0],
 "largeur": 800, "hauteur": 800, "png": "<base64>",
 "trajectoire": [[0.0, 0.0], [0.1, 0.0]], "chemin": [[1.2, -0.3], [2.0, 0.1]], "but": [5.0, 0.0]}
```

**`scene`** (4 Hz) — monde reconstruit autour du rover pour la vue 3D (façon Tesla), dans le
repère `odom` (`repere` vaut `base_link` tant que l'odométrie manque : rover fixe à l'origine) :
```json
{"type": "scene", "t": 1234.5, "repere": "odom", "rover": {"x": 1.2, "y": -0.3, "cap": 0.12},
 "objets": [
   {"id": 4, "type": "mur", "x": 3.1, "y": 0.4, "cap": 1.57, "longueur": 2.8, "epaisseur": 0.12,
    "hauteur": 1.0, "age_s": 0.1, "extrapole": false, "danger": false},
   {"id": 9, "type": "arbre", "x": 5.0, "y": -1.2, "rayon": 0.14, "hauteur": 3.4, "age_s": 2.5},
   {"id": 12, "type": "bloc", "x": 2.0, "y": 1.5, "cap": 0.3, "longueur": 0.6, "largeur": 0.4,
    "hauteur": 0.6, "age_s": 0.0, "danger": true},
   {"id": 15, "type": "bas", "x": 1.6, "y": -0.1, "cap": 0.0, "longueur": 0.5, "largeur": 0.2,
    "hauteur": 0.12, "age_s": 0.3},
   {"id": 20, "type": "etiquette", "x": 2.1, "y": 1.4, "texte": "chaise rouge", "age_s": 8.0}],
 "libre": [[1.2, -0.3], [4.0, -1.0], [4.2, 1.1]],
 "chemin": [[1.2, -0.3], [2.0, 0.0]], "but": [5.0, 0.0]}
```
- `mur` : segment (centre, cap le long du mur, longueur) ; `extrapole: true` = prolongement
  supposé au-delà de ce qui a été vu (à dessiner plus transparent).
- `arbre` : petit objet compact (tronc, poteau, pied) ; `hauteur` est une estimation stable par
  objet, pour varier les tailles.
- `bloc` : objet plus large coupé par le lidar (meuble, rocher, carton), boîte orientée.
- `bas` : obstacle sous le plan du lidar, vu par la stéréo (marche, poutre, pot), hauteur mesurée.
- `etiquette` : objet nommé par la vision IA (« chercher », « décrire »), placé au sol.
- `personne`, `animal` : être vivant détecté en continu (rôle `detection` de l'IA), avec
  `rayon`, `hauteur` et `texte` (étiquette du détecteur) ; oublié 5 s après la dernière vue.
- `danger` : dans le couloir de sécurité devant ou derrière le rover.
- `age_s` : temps depuis la dernière observation (les objets non revus s'estompent puis
  disparaissent au bout de 30 s) ; `libre` : polygone de l'espace libre vu par le dernier tour du
  lidar (sol praticable).

**`gps`** (1 Hz) — navigation GPS : position brute, position fusionnée (odométrie recalée en
continu sur le GPS), trace, but, parcours, zone autorisée et lieux mémorisés :
```json
{"type": "gps", "t": 1234.5, "fix": "autonome", "satellites": 7, "hdop": 1.1,
 "brut": {"lat": 43.71021, "lon": -1.05362},
 "rover": {"lat": 43.71022, "lon": -1.05360, "precision_m": 1.8, "cap_vrai_deg": 87.0},
 "alignement": {"ok": true, "residu_m": 1.2, "etendue_m": 14.0, "points": 120},
 "reference": {"lat": 43.7102, "lon": -1.0536},
 "trace": [[43.71020, -1.05362], [43.71021, -1.05361]],
 "but": {"lat": 43.7105, "lon": -1.0531}, "parcours": [[43.7105, -1.0531]],
 "zone": [[43.7100, -1.0540], [43.7108, -1.0540], [43.7108, -1.0528], [43.7100, -1.0528]],
 "lieux": [{"nom": "garage", "lat": 43.7102, "lon": -1.0536}]}
```
`rover` vaut `null` tant que l'alignement odométrie ↔ GPS n'est pas fait (il faut avoir roulé
quelques mètres) ; `cap_vrai_deg` est le cap par rapport au nord vrai (0 = nord, 90 = est).
Le GPS ATGM336H est précis à 2–3 m : un but GPS est atteint à `tolerance_m` près (2,5 m par
défaut) ; l'évitement des obstacles reste fait par le lidar et la stéréo.

**`journal`** — un événement : `{"type": "journal", "t": 1729250000.1, "niveau": "alerte",
"source": "securite", "message": "obstacle à 0,42 m devant : arrêt"}` (`t` = heure murale ;
niveaux `info`, `succes`, `alerte`, `erreur`). À la connexion, les 100 derniers sont envoyés.

**`chat`** — un message de la conversation avec le rover :
```json
{"type": "chat", "id": "m12", "t": 1729250000.1, "role": "rover", "texte": "J'avance de 5 m.",
 "origine": "voix", "plan": {"id": "p7", "actions": [{"type": "avancer", "distance_m": 5.0}]}}
```
`role` : `operateur` (écrit ou dit), `rover` (réponse de l'agent), `systeme`. `origine` :
`texte` ou `voix`. `plan` facultatif.

**`voix`** — événements de l'écoute : `{"type": "voix", "evenement": "transcription",
"texte": "avance de cinq mètres", "t": ...}` ; `evenement` : `debut_parole`, `fin_parole`,
`transcription`, `ignore` (pas une consigne), `erreur` (avec `message`).

**`vision`** — résultat d'une analyse d'image par l'IA :
```json
{"type": "vision", "id": "v3", "t": 1729250000.1, "camera": "gauche", "demande": "chercher",
 "objet": "chaise rouge", "description": "Une chaise rouge à droite, près du mur.",
 "image": "<jpeg base64 de l'image analysée, 640 px>",
 "points": [{"u": 0.61, "v": 0.72, "label": "chaise rouge", "sol": [2.1, -0.6]}],
 "boites": [{"u0": 0.5, "v0": 0.4, "u1": 0.7, "v1": 0.9, "label": "chaise"}],
 "modele": "gemini-robotics-er-2-preview", "duree_s": 2.8}
```
`u`, `v` normalisés (0 à gauche/en haut, 1 à droite/en bas) ; `sol` = point au sol dans
`base_link` si la projection est possible, sinon `null`.

**`console`** — lignes du port de l'ESP : `{"type": "console", "lignes": [[1729250000.1, "[MOTEURS] G=0 % ..."]]}`.

**`calibration`** — avancement d'une calibration (`nom` : `sens_roues`, `vitesse`, `braquage`,
`imu_repos`, ou `stereo` pour la capture et le calcul de la mire) : `{"type": "calibration", "nom": "vitesse",
"etat": "en_cours" | "termine" | "echec", "message": "...", "resultat": {...}}`.

**`reponse`** — accusé d'une demande du poste qui porte un `id` : `{"type": "reponse", "id": 4,
"ok": false, "erreur": "obstacle devant : action refusée"}`.

### Poste → serveur

Toute demande peut porter un `id` numérique ; le serveur répond alors par `reponse`.

| Message | Effet |
| --- | --- |
| `{"type": "pilotage", "vitesse": 0.5, "braquage": -0.3}` | Téléopération. Valeurs **normalisées −1..1** (fraction de la vitesse maximale du mode et du braquage maximal). À envoyer à 10–20 Hz tant qu'une touche ou le joystick est actif, puis une fois `0, 0` : sans message pendant 0,5 s, le rover s'arrête (homme mort). Ignoré hors mode `manuel`. |
| `{"type": "mode", "mode": "manuel"}` | `arret`, `manuel`, `auto`, `banc`. Passer en `banc` demande `"confirmer": true`. |
| `{"type": "armer"}` / `{"type": "desarmer"}` | Autorise / interdit la traction. Le rover démarre désarmé. |
| `{"type": "arret_urgence"}` | Arrêt immédiat, verrouillé jusqu'à `{"type": "rearmer"}`. |
| `{"type": "action", "action": {"type": "avancer", "distance_m": 2}}` | Ajoute une action à la file (types dans `types.py`, `Action`). `"remplacer": true` vide d'abord la file. Passe en mode `auto` si le rover est armé. |
| `{"type":"action","action":{"type":"explorer","rayon_m":8,"points_max":20,"duree_s":600}}` | Exploration progressive : candidats connus accessibles calculés sur le Pi, choix de l'index sur GX10, observation et nouveau choix à chaque arrivée. `objet` facultatif, `approcher:false` pour repérer sans rejoindre. |
| `{"type": "annuler"}` | Vide la file et arrête l'action en cours. |
| `{"type": "pause"}` / `{"type": "reprendre"}` | Suspend / reprend l'action en cours. |
| `{"type": "chat", "texte": "avance de 5 mètres puis tourne à gauche"}` | Consigne à l'agent IA ; réponse en `chat`. |
| `{"type": "voix", "active": true}` | Active l'écoute du micro de la webcam (mode vocal). |
| `{"type": "vision", "demande": "decrire", "camera": "gauche"}` | `decrire` ou `chercher` (avec `"objet": "..."`). Réponse en `vision` (et `chat`). |
| `{"type": "aller_pixel", "camera": "gauche", "u": 0.52, "v": 0.71}` | Clic sur l'image : aller au point du sol visé. |
| `{"type": "but", "x": 3.0, "y": 1.5}` | Clic sur la carte : aller au point (repère `odom`). |
| `{"type": "but_gps", "lat": 43.7105, "lon": -1.0531}` | Aller à ce point GPS (remplace la mission). |
| `{"type": "parcours_gps", "points": [[lat, lon], ...], "boucle": false}` | Enchaîner des points GPS. |
| `{"type": "zone", "points": [[lat, lon], ...]}` | Zone autorisée (polygone) : le rover refuse d'en sortir en mode auto. `"points": null` la retire. Enregistrée sur le rover. |
| `{"type": "lieu", "action": "memoriser", "nom": "garage"}` | `memoriser` (position actuelle), `aller`, `supprimer`. |
| `{"type": "remise_a_zero_odom"}` | La pose courante devient l'origine ; la carte est effacée. |
| `{"type": "console", "texte": "etat"}` | Ligne brute vers l'ESP. Refusé si le rover roule ou est armé hors mode `banc`. |
| `{"type": "banc", "pwm_g": 5, "pwm_d": 0, "servo_deg": 0}` | Mode `banc` seulement : pourcentages signés bruts (plafond `traction.pwm_max_banc`), à renvoyer à 5 Hz (homme mort 0,5 s). |
| `{"type": "reglage", "cle": "vitesses.max_manuel_m_s", "valeur": 0.6}` | Modifie un réglage de `reglables` ; réponse avec la valeur retenue (bornée). |
| `{"type": "ia_profil", "profil": "gx10"}` | Change de fournisseur de modèles (`gemini`, `gx10`, `aucun`). |
| `{"type": "action", "action": {"type": "suivre", "u": 0.52, "v": 0.6, "camera": "gauche", "distance_m": 1.5}}` | Suivre la personne cliquée (ou `"description": "la personne en t-shirt rouge"`). `etat.cible` = `{"etat": "suivi"|"perdu", "boite": [u0, v0, u1, v1], "distance_m", "source_distance": "lidar"|"taille", "traqueur", "hz"}` ou `null`. |
| (serveur → poste) `{"type": "parole", "id": "m12", "texte": "...", "audio": "<WAV base64>", "format": "wav", "modele": "..."}` | Voix du rover pour la réponse `chat` de même `id` (profil avec un rôle `synthese`) ; le poste la joue si la lecture à voix haute est active. |
| `{"type": "remorque_alignee"}` | La remorque est dans l'axe du rover : l'angle estimé repart de 0 avec une petite incertitude (marche arrière permise). Refusé sans remorque ou si le rover roule. |
| `{"type": "cameras_inverser"}` | Échange les caméras gauche et droite (mémorisé par port USB). |
| `{"type": "lidar", "marche": false}` | Arrête / relance la rotation du lidar (refusé si le rover roule). |
| `{"type": "calibration", "nom": "vitesse"}` | Lance une calibration : `sens_roues` (roues levées), `vitesse`, `braquage`, `stereo_capture` (une paire de photos de la mire), `stereo_calculer`, `imu_repos`. `{"nom": "...", "arreter": true}` l'interrompt. |
| `{"type": "photo", "camera": "gauche"}` | Enregistre une photo dans `~/fieldmind/photos` sur le rover. |
| `{"type": "drone", "action": "cmd", "cmd": "recon_test"}` | Essai sans vol, sur image réelle récente. `recon_capture` prend la vue suivante (six vues) ; `cibler_croix` active/désactive la visée ; `stop_recon` arrête l'essai. |
| `{"type": "drone", "action": "cmd", "cmd": "camera"}` | Demande expérimentale de bascule caméra BLE ; vérifier le flux, capteur non confirmé. `decoller`, `calibrer`, `recon_vol` et `action: manette` sont refusés (protocole moteur non validé). |

L'état `drone` expose `mode_recon`, `secteur_360` (nombre de vues, angle non mesuré),
`croix_trouvee`, `croix_verrouillee` (centrage stable dans l'image), `croix_x/y`,
`croix_taille_pct`, `consigne_vol`, `pilotage_valide: false`, `analyse_ms`, `montage_recon`.
Les requêtes `stop` / `atterrir` ne prouvent pas l'exécution par le drone.
Voir [DRONE.md](DRONE.md) pour les exports HTTP et les limites physiques.

**Atelier indépendant du matériel** : `{"type":"drone_atelier","action":"mission",
"scenario":"normal"}` démarre le drone simulé ; `action:"manuel"` démarre le banc
manuel au sol. Actions manuelles : `decoller`, `camera_bas`, `axes` (champs `montee`,
`lacet`, `avant`, `droite`, chacun entre -1 et 1). `interrompre` demande un posé simulé.
Le poste envoie les axes à 10 Hz, neutre compris ; sans demande depuis 0,5 s, le
pilotage manuel en vol déclenche son repli. Un nouveau `drone_atelier` peut aussi
indiquer `graine` (0..10000), `vitesse` (1 ou 4) et un scénario de
[DRONE_MISSION.md](DRONE_MISSION.md). Les axes sont refusés pendant une mission.

`etat.drone_atelier` expose la source `simulation`, phase, hauteur, cap, caméra,
cible, paliers, photos et journal. Il ne remplace jamais `etat.drone` matériel.
Images : `/flux/drone_simule`, `/photo/drone_simule`. Exports protégés par le jeton
du poste : `/api/drone/atelier/montage.jpg` et `rapport.json`.

### Garanties côté serveur

- Le poste ne commande jamais les moteurs directement : tout passe par la sécurité, qui peut
  ralentir ou refuser (`securite.raisons` dit pourquoi).
- Une demande refusée (action impossible, réglage hors liste, console pendant le roulage) reçoit
  une `reponse` avec `ok: false` et une `erreur` lisible.
- La fermeture de la WebSocket du pilote en mode `manuel` arrête le rover dans les 0,5 s.

## Langues et caméras simulées dans le navigateur

Le poste accepte `?lang=fr` et `?lang=en`. Le sélecteur FR / EN mémorise le choix dans le navigateur. La traduction concerne l'affichage ; les noms de messages, valeurs de mode, identifiants de capteurs et paramètres du protocole restent identiques.

En matériel simulé uniquement, `bienvenue.cameras_navigateur` annonce le transport WebGL. Le poste volontaire envoie `{"type":"camera_simulee_prete","prete":true}` ; `false` libère le rendu, notamment quand la page passe en arrière-plan. Un seul poste fournit les images d'un rover simulé.

Le serveur envoie d'abord `monde_simule` (version, obstacles, couleurs, pièce et damier), puis des demandes `camera_simulee` : numéro `id`, date monotone `t`, pose vraie `[x,y,cap]`, vues, intrinsèques et extrinsèques, largeur 640 et hauteur 360. Les deux images d'une demande portent la même date de pose, même si elles arrivent séparément.

Chaque réponse est une trame WebSocket **binaire** : en-tête JSON `{"id":123,"camera":"gauche"}`, saut de ligne LF, puis le JPEG. L'en-tête est borné à 255 octets ; le JPEG à 400 000 octets. Seules les vues demandées pour le numéro courant sont acceptées, une fois par vue. Les images sont publiées sur le même bus que les caméras physiques et alimentent la stéréo, la vision, les photos et l'IA.

Une page sans WebGL conserve le rendu logiciel. Si aucune image du navigateur n'arrive pendant deux secondes, le matériel simulé reprend ce rendu. Ce transport ne reçoit jamais d'images sur le matériel réel et ne modifie aucune commande de conduite.
