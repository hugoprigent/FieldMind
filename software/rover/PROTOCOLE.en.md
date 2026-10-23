[Français](PROTOCOLE.md) · **English**

<a id="protocole-du-poste-opérateur"></a>
# Operator Station Protocol

The rover serves the operator station (`fieldmind/web/`) and its API on `http://<rover>:8080`. Everything goes through a **WebSocket** `/ws` (JSON); video goes through HTTP **MJPEG** streams. SI units, reference frames from [types.py](fieldmind/types.py): `base_link` (x forward, y left, origin on the rear axle), `odom` (fixed, at the start), angles in trigonometric radians except for the `_deg` fields.

Optional token: if the rover is launched with `FIELDMIND_JETON`, add `?jeton=…` to the WebSocket and stream URLs (the station reads it from the page URL and keeps it in `localStorage`).

## HTTP

| Route | Role |
| --- | --- |
| `GET /` | the station (static files from `fieldmind/web/`) |
| `GET /flux/<camera>?ips=10&largeur=640` | MJPEG `multipart/x-mixed-replace` ; `camera` = `gauche`, `droite` or `profondeur` (false-color depth map) |
| `GET /photo/<camera>` | last JPEG image |
| `GET /api/etat` | last `etat` message (JSON), for a quick test |
| `GET /api/profil` | full rover profile (content of `bienvenue.profil`) |
| `GET /api/journal?n=200` | latest log events |
| `GET /api/sante` | `{"ok": true, "version": "1.0.0"}` |

## WebSocket `/ws`

Each message is an `{"type": "...", ...}` object. The server sends `bienvenue` upon connection. Multiple stations can be connected; the last one to take manual control has the hand.

<a id="serveur--poste"></a>
### Server → Station

**`bienvenue`** (once)
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
`reglables_details` provides for each setting its label, group, unit, step, and type (`nombre`, `booleen`, `texte`): the station uses this to organize the Settings menu. A modified setting is applied immediately (the servo neutral causes the wheels to move while stopped). `materiel` is `reel` or `simulation` (the station then displays a "SIMULATION" banner).

**`etat`** (10 Hz) — everything that changes quickly, in a single message:
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
Any field may be `null` if information is missing (sensor absent). In simulation only, `etat.simulation` = `{"x", "y", "cap", "vitesse"}` gives the true pose (simulated world frame); `capteurs.odometrie` = `{"ok", "duree_ms"}` when estimation is running. `mission.etat`: `attente`, `route`, `bloque`, `pause`, `termine`, `echec`, `annule`. `etat.remorque` is `null` without a hitched trailer, otherwise `{"attelee": true, "angle_deg": 12.0, "incertitude_deg": 1.5, "recul_possible": true, "geometrie": {"attelage_m", "timon_m", "avant_m", "arriere_m", "demi_largeur_m"}}`: estimated hitch angle (> 0: trailer to the left, behind), board in meters from the hitch pin; associated safety reasons: `remorque`, `remorque_portefeuille`, `remorque_incertaine`, `remorque_recul_auto`. `raison_urgence` (text or `null`) states what triggered the emergency stop (`poste`, `choc`…). Codes from `securite.raisons` (the station translates them): `arret_urgence`, `non_arme`, `mode_arret`, `obstacle_avant`, `obstacle_arriere`, `obstacle_bas` (stereo), `lidar_absent`, `stereo_absente`, `esp_absent`, `esp_defaut`, `inclinaison`, `choc`, `blocage`, `consigne_perimee`, `operateur_perdu`, `geofence`, `ralenti_obstacle`, `banc`.

**`systeme`** (1 Hz)
```json
{"type": "systeme", "cpu_pct": 38, "temperature_c": 61.5, "memoire_pct": 42, "disque_pct": 31,
 "sous_tension": false, "ralenti": false, "throttled": "0x0",
 "wifi": {"ssid": "reseau-atelier", "signal_pct": 71, "ip": "rover.local"}, "charge": [1.2, 1.1, 0.9],
 "uptime_s": 5321, "processus_cpu_pct": 55}
```

**`scan`** (5 Hz) — lidar points in `base_link`, in flattened integer centimeters:
```json
{"type": "scan", "t": 1234.5, "xy": [120, -3, 118, 0, 117, 4]}
```

**`obstacles`** (5 Hz) — obstacles seen by stereo (and other sources), `base_link`, cm:
```json
{"type": "obstacles", "t": 1234.5, "source": "stereo", "xy": [80, 10, 82, 12], "portee_m": 4.0}
```

**`carte`** (1 Hz) — occupancy grid of frame `odom` in PNG (grayscale levels: 0 black = occupied, 255 white = free, 128 = unknown; row 0 of the PNG = **maximum y**, image ready to draw):
```json
{"type": "carte", "t": 1234.5, "resolution": 0.05, "origine": [-20.0, -20.0],
 "largeur": 800, "hauteur": 800, "png": "<base64>",
 "trajectoire": [[0.0, 0.0], [0.1, 0.0]], "chemin": [[1.2, -0.3], [2.0, 0.1]], "but": [5.0, 0.0]}
```

**`scene`** (4 Hz) — reconstructed world around the rover for the 3D view (Tesla-style), in frame `odom` (`repere` is `base_link` as long as odometry is missing: rover fixed at the origin):
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
- `mur`: segment (center, heading along the wall, length); `extrapole: true` = assumed extension beyond what has been seen (to be drawn more transparently).
- `arbre`: small compact object (trunk, pole, leg); `hauteur` is a stable estimate per object, to vary sizes.
- `bloc`: wider object cut by the lidar (furniture, rock, cardboard), oriented bounding box.
- `bas`: obstacle below the lidar plane, seen by stereo (step, beam, pot), measured height.
- `etiquette`: object named by AI vision ('search', 'describe'), placed on the ground.
- `personne`, `animal`: living being detected continuously (role `detection` of the AI), with `rayon`, `hauteur` and `texte` (detector label); forgotten after 5 s since the last sighting.
- `danger`: in the safety corridor in front of or behind the rover.
- `age_s`: time since last observation (objects not seen again fade and disappear after 30 s); `libre`: polygon of free space seen by the last lidar scan (traversable ground).

**`gps`** (1 Hz) — GPS navigation: raw position, fused position (odometry continuously recalibrated on GPS), track, goal, path, allowed zone and memorized locations:
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
`rover` is `null` as long as the odometry ↔ GPS alignment has not been done (you need to have driven a few meters); `cap_vrai_deg` is the heading relative to true north (0 = north, 90 = east). The ATGM336H GPS is accurate to 2–3 m: a GPS goal is reached within `tolerance_m` (2,5 m by default); obstacle avoidance is still handled by the lidar and stereo.

**`journal`** — an event: `{"type": "journal", "t": 1729250000.1, "niveau": "alerte", "source": "securite", "message": "obstacle à 0,42 m devant : arrêt"}` (`t` = wall clock time; levels `info`, `succes`, `alerte`, `erreur`). On connection, the last 100 are sent.

**`chat`** — a message from the conversation with the rover:
```json
{"type": "chat", "id": "m12", "t": 1729250000.1, "role": "rover", "texte": "J'avance de 5 m.",
 "origine": "voix", "plan": {"id": "p7", "actions": [{"type": "avancer", "distance_m": 5.0}]}}
```
`role` : `operateur` (written or spoken), `rover` (agent response), `systeme`. `origine` : `texte` or `voix`. `plan` optional.

**`voix`** — listen events: `{"type": "voix", "evenement": "transcription", "texte": "avance de cinq mètres", "t": ...}` ; `evenement` : `debut_parole`, `fin_parole`, `transcription`, `ignore` (not a command), `erreur` (with `message`).

**`vision`** — result of an AI image analysis:
```json
{"type": "vision", "id": "v3", "t": 1729250000.1, "camera": "gauche", "demande": "chercher",
 "objet": "chaise rouge", "description": "Une chaise rouge à droite, près du mur.",
 "image": "<jpeg base64 de l'image analysée, 640 px>",
 "points": [{"u": 0.61, "v": 0.72, "label": "chaise rouge", "sol": [2.1, -0.6]}],
 "boites": [{"u0": 0.5, "v0": 0.4, "u1": 0.7, "v1": 0.9, "label": "chaise"}],
 "modele": "gemini-robotics-er-2-preview", "duree_s": 2.8}
```
`u`, `v` normalized (0 left/top, 1 right/bottom) ; `sol` = ground point in `base_link` if projection is possible, otherwise `null`.

**`console`** — lines from the ESP port: `{"type": "console", "lignes": [[1729250000.1, "[MOTEURS] G=0 % ..."]]}`

**`calibration`** — calibration progress (`nom` : `sens_roues`, `vitesse`, `braquage`, `imu_repos`, or `stereo` for pattern capture and calculation): `{"type": "calibration", "nom": "vitesse", "etat": "en_cours" | "termine" | "echec", "message": "...", "resultat": {...}}`

**`reponse`** — acknowledgment of a request from the station carrying a `id`: `{"type": "reponse", "id": 4, "ok": false, "erreur": "obstacle devant : action refusée"}`

<a id="poste--serveur"></a>
### Station → Server

Any request may carry a `id` number; the server then responds with `reponse`

| Message | Effect |
| --- | --- |
| `{"type": "pilotage", "vitesse": 0.5, "braquage": -0.3}` | Teleoperation. **Normalized values −1..1** (fraction of the mode's maximum speed and maximum steering). Send to 10–20 Hz as long as a key or joystick is active, then once `0, 0`: no message for 0,5 s, the rover stops (dead man). Ignored outside `manuel` mode. |
| `{"type": "mode", "mode": "manuel"}` | `arret`, `manuel`, `auto`, `banc`. Switching to `banc` requires `"confirmer": true`
| `{"type": "armer"}` / `{"type": "desarmer"}` | Allow / prohibit traction. The rover starts disarmed.
| `{"type": "arret_urgence"}` | Immediate stop, locked until `{"type": "rearmer"}`
| `{"type": "action", "action": {"type": "avancer", "distance_m": 2}}` | Adds an action to the queue (types in `types.py`, `Action`). `"remplacer": true` clears the queue first. Switches to `auto` mode if the rover is armed.
| `{"type":"action","action":{"type":"explorer","rayon_m":8,"points_max":20,"duree_s":600}}` | Progressive exploration: accessible known candidates calculated on the Pi, index selection on GX10, observation and new selection upon each arrival. `objet` optional, `approcher:false` to mark without reaching.
| `{"type": "annuler"}` | Clears the queue and stops the current action.
| `{"type": "pause"}` / `{"type": "reprendre"}` | Suspend / resume the current action.
| `{"type": "chat", "texte": "avance de 5 mètres puis tourne à gauche"}` | Instruction to the AI agent; response in `chat`. |
| `{"type": "voix", "active": true}` | Activates the webcam microphone listening (voice mode). |
| `{"type": "vision", "demande": "decrire", "camera": "gauche"}` | `decrire` or `chercher` (with `"objet": "..."`). Response in `vision` (and `chat`). |
| `{"type": "aller_pixel", "camera": "gauche", "u": 0.52, "v": 0.71}` | Click on the image: go to the targeted ground point. |
| `{"type": "but", "x": 3.0, "y": 1.5}` | Click on the map: go to the point (reference `odom`). |
| `{"type": "but_gps", "lat": 43.7105, "lon": -1.0531}` | Go to this GPS point (replaces the mission). |
| `{"type": "parcours_gps", "points": [[lat, lon], ...], "boucle": false}` | Chain GPS points. |
| `{"type": "zone", "points": [[lat, lon], ...]}` | Authorized zone (polygon): the rover refuses to leave it in auto mode. `"points": null` removes it. Saved on the rover. |
| `{"type": "lieu", "action": "memoriser", "nom": "garage"}` | `memoriser` (current position), `aller`, `supprimer`. |
| `{"type": "remise_a_zero_odom"}` | The current pose becomes the origin; the map is cleared. |
| `{"type": "console", "texte": "etat"}` | Raw line to the ESP. Refused if the rover is moving or armed outside `banc` mode. |
| `{"type": "banc", "pwm_g": 5, "pwm_d": 0, "servo_deg": 0}` | `banc` mode only: raw signed percentages (ceiling `traction.pwm_max_banc`), to be sent back at 5 Hz (dead man 0,5 s). |
| `{"type": "reglage", "cle": "vitesses.max_manuel_m_s", "valeur": 0.6}` | Modifies a `reglables` setting; response with the accepted value (clamped). |
| `{"type": "ia_profil", "profil": "gx10"}` | Switches model providers (`gemini`, `gx10`, `aucun`). |
| `{"type": "action", "action": {"type": "suivre", "u": 0.52, "v": 0.6, "camera": "gauche", "distance_m": 1.5}}` | Follow the clicked person (or `"description": "la personne en t-shirt rouge"`). `etat.cible` = `{"etat": "suivi"|"perdu", "boite": [u0, v0, u1, v1], "distance_m", "source_distance": "lidar"|"taille", "traqueur", "hz"}` or `null`. |
| (server → workstation) `{"type": "parole", "id": "m12", "texte": "...", "audio": "<WAV base64>", "format": "wav", "modele": "..."}` | Rover voice for the `chat` response of the same `id` (profile with a `synthese` role); the workstation plays it if text-to-speech is active. |
| `{"type": "remorque_alignee"}` | The trailer is aligned with the rover: the estimated angle restarts from 0 with a small uncertainty (reverse allowed). Refused without a trailer or if the rover is moving. |
| `{"type": "cameras_inverser"}` | Swaps the left and right cameras (remembered by USB port). |
| `{"type": "lidar", "marche": false}` | Stops / restarts the lidar rotation (refused if the rover is moving). |
| `{"type": "calibration", "nom": "vitesse"}` | Starts a calibration: `sens_roues` (wheels lifted), `vitesse`, `braquage`, `stereo_capture` (a pair of target board photos), `stereo_calculer`, `imu_repos`. `{"nom": "...", "arreter": true}` interrupts it. |
| `{"type": "photo", "camera": "gauche"}` | Saves a photo in `~/fieldmind/photos` on the rover. |
| `{"type": "drone", "action": "cmd", "cmd": "recon_test"}` | Test without flight, on a recent real image. `recon_capture` takes the next view (six views); `cibler_croix` enables/disables aiming; `stop_recon` stops the test. |
| `{"type": "drone", "action": "cmd", "cmd": "camera"}` | Experimental BLE camera flip request; check the stream, sensor not confirmed. `decoller`, `calibrer`, `recon_vol` and `action: manette` are refused (motor protocol not validated). |

The `drone` state exposes `mode_recon`, `secteur_360` (number of views, angle not measured), `croix_trouvee`, `croix_verrouillee` (stable centering in the image), `croix_x/y`, `croix_taille_pct`, `consigne_vol`, `pilotage_valide: false`, `analyse_ms`, `montage_recon`. The `stop` / `atterrir` requests do not prove execution by the drone. See [DRONE.md](DRONE.en.md) for HTTP exports and physical limits.

**Hardware-independent workshop**: `{"type":"drone_atelier","action":"mission", "scenario":"normal"}` starts the simulated drone; `action:"manuel"` starts the manual ground bench. Manual actions: `decoller`, `camera_bas`, `axes` (fields `montee`, `lacet`, `avant`, `droite`, each between -1 and 1). `interrompre` requests a simulated exposure. The station sends axes at 10 Hz, including neutral; without a request for 0,5 s, manual flight control triggers its retraction. A new `drone_atelier` can also indicate `graine` (0..10000), `vitesse` (1 or 4) and a scenario from [DRONE_MISSION.md](DRONE_MISSION.en.md). Axes are refused during a mission.

`etat.drone_atelier` exposes the source `simulation`, phase, height, heading, camera, target, steps, photos and log. It never replaces `etat.drone` hardware. Images: `/flux/drone_simule`, `/photo/drone_simule`. Exports protected by the station token: `/api/drone/atelier/montage.jpg` and `rapport.json`.

<a id="garanties-côté-serveur"></a>
### Server-side guarantees

- The station never commands the motors directly: everything goes through safety, which can slow down or refuse (`securite.raisons` explains why).
- A refused request (impossible action, setting out of list, console during driving) receives an `reponse` with `ok: false` and a readable `erreur`.
- Closing the pilot's WebSocket in `manuel` mode stops the rover within 0,5 s.

<a id="langues-et-caméras-simulées-dans-le-navigateur"></a>
## Languages and simulated cameras in the browser

The station accepts `?lang=fr` and `?lang=en`. The FR / EN selector memorizes the choice in the browser. Translation concerns the display; message names, mode values, sensor identifiers and protocol parameters remain identical.

In simulated hardware only, `bienvenue.cameras_navigateur` announces WebGL transport. The station voluntarily sends `{"type":"camera_simulee_prete","prete":true}`; `false` releases rendering, especially when the page goes to the background. Only one station provides images for a simulated rover.

The server first sends `monde_simule` (version, obstacles, colors, piece and checkerboard), then `camera_simulee` requests: number `id`, monotonic date `t`, true pose `[x,y,cap]`, views, intrinsics and extrinsics, width 640 and height 360. The two images in a request share the same pose date, even if they arrive separately.

Each response is a **binary** WebSocket frame: JSON header `{"id":123,"camera":"gauche"}`, LF line break, then the JPEG. The header is bounded to 255 bytes; the JPEG to 400 000 bytes. Only views requested for the current number are accepted, once per view. The images are published on the same bus as the physical cameras and feed stereo, vision, photos, and AI.

A page without WebGL retains software rendering. If no browser image arrives for two seconds, the simulated hardware resumes this rendering. This transport never receives images on real hardware and does not modify any driving commands.
