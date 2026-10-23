[Français](README.md) · **English**

# FieldMind rover software

The software that runs **on the rover's Raspberry Pi**: hardware drivers, safety,
driving, lidar odometry, stereo, navigation, missions, AI agent with voice control, and the
web operator station. It replaces the `software/embarque` supervision (same ports, same
cameras). Architecture and interfaces: [ARCHITECTURE.md](ARCHITECTURE.md); station messages:
[PROTOCOLE.md](PROTOCOLE.md) (both in French).

Prototype status on 23 October: [recent changes and limits](../../docs/status-2026-10-23.en.md).
Next test: [red box mission, photo and return to base](TEST_TERRAIN_2026-10-22.md) (in French).

The web station is available in French and English (FR / EN, top right). It detects the preferred browser language and remembers a manual choice. Protocol identifiers remain unchanged.

## Trying it on a PC, without the rover

The hardware is simulated (3D world, fake ESP speaking the firmware protocol, lidar, rendered
stereo pair, GPS); the rest is the rover's code.

```bash
cd software/rover
uv venv --python 3.13 .venv
uv pip install --python .venv/bin/python numpy scipy opencv-python-headless pyserial aiohttp \
  pytest pytest-asyncio ruff
.venv/bin/python -m fieldmind --materiel simulation          # « salle » (room) world; --scenario jardin
```

Then <http://127.0.0.1:8080/>. Station demo without a server: `fieldmind/web/index.html?mock=1`.

Tests: `.venv/bin/python -m pytest`; style: `.venv/bin/ruff check fieldmind tests`.

### Public simulation server

Each browser gets its own simulated rover, map and settings; no interface
with the real hardware is initialized. The server picks the GX10 if it is reachable, then
Gemini if a key is configured, otherwise it runs without AI:

```bash
.venv/bin/python -m fieldmind.public_demo --hote 127.0.0.1 --port 8085
```

Open <http://127.0.0.1:8085/>. The service is meant to sit behind a TLS proxy
before any public exposure. Its deployment configuration is not included here.

## What the software does

- **Drone**: [reconnaissance without flying and aiming at the cross](DRONE.md), six captures,
  downloadable mosaic and report. Autonomous flight stays blocked until the
  motor protocol of the CooleerX variant is validated.
  Offline analysis of the original app made it possible to identify a command frame,
  but the RC stream is neither connected nor validated on the bench; no real command is enabled.
  [Local workshop for the complete mission](DRONE_MISSION.md): take-off and lap with
  photos, bottom camera and simulated stepwise descent, gamepad and GX10 preparation (both documents in French).

- **Safe driving**: safety at 20 Hz on the Pi (corridor along the planned arc, lidar +
  stereo + memory of low obstacles, stale sensors, tilt, impact, stall, dead man's switch,
  GPS-allowed zone, slowing down near a detected person); ROVER_4 firmware with watchdog.
- **Steering**: since the DT996 servo failed, the real rover uses the differential of the two
  rear wheels; the software also keeps support for servo steering. An SG90 adaptation
  has been modeled, without validation under load.
- **Where it is**: lidar odometry (ICP) + gyroscope, occupancy map, alignment on GPS
  (true position and heading).
- **Missions**: move forward, turn (with maneuver), go to a point (map, image, GPS), saved
  places, search for an object (vision), progressive exploration, photo, automatic calibrations.
- **Following a person**: *Suivre* (Follow) button then click on the person in *Pilotage*, or « suis la
  personne en t-shirt rouge » ("follow the person in the red t-shirt"). Designation by the GX10 (RoboBrain 2.5, or SAM 2.1 for a click),
  frame-by-frame tracking on the Pi (VitTrack, ≈ 10 Hz), distance from the lidar; stops if the person is
  lost from view, resumes if a detection resembles them. Safety applies as always.
- **Drone-carrier trailer** (optional, *Réglages > Remorque*): hitch angle estimated by
  odometry, trailer footprint in the safety corridor, adapted speeds and braking;
  slow reversing, in manual mode only, refused if the trailer would jackknife
  or if its angle is uncertain (drive 1 m straight ahead, or *Dans l'axe* (in line) in *Pilotage*).
- **AI**: written or spoken instructions (webcam microphone), vision, Gemini / GX10 / Mac profiles
  ([MODELES.md](MODELES.md), in French). On the GX10: Qwen3.6 agent, RoboBrain 2.5 pointing, continuous
  detection, depth, outlines, rover voice, memory of places, and a **watch agent** that
  analyzes the situation continuously (you can talk to it: « qu'est-ce qui se passe ? », "what's going on?").
- **Web station**: driving, Tesla-style 3D view, local map and OpenStreetMap GPS map,
  mission and AI, sensors, system, settings.
- **Black box**: `python -m fieldmind.enregistreur` replays a test. **ROS 2**: `ros2/`.

## AI models

Keys in `software/rover/.env` (never in git; commented template: `.env.exemple`), copied
to the Pi by the deployment; the Gemini key can also be entered in the station (*Réglages > IA*).
Model check: `.venv/bin/python deploiement/gx10/verifier.py --profil gemini`.

Profiles in [config/ia.toml](config/ia.toml): `gemini` (online: Gemini 4 Argon for the agent,
Gemini Robotics ER 2 for vision, Gemini transcription), `gx10` (local models of the ASUS
Ascent GX10, see [deploiement/gx10/](deploiement/gx10/README.en.md)), `aucun` (none). The choice is made in
the station, *Réglages* (Settings) menu. Safety does not depend on any model: without AI, the rover can be driven,
avoids obstacles and executes the station's actions.

## Putting it on the rover

Nothing is installed until these scripts are run. From `software/rover/deploiement`:

1. Once: `ssh utilisateur@adresse-du-pi 'bash -s' < installer_pi.sh` (apt packages, groups, GPS UART).
2. **ESP firmware ROVER_4** (watchdog and flap outputs): upload `software/esp32/esp_c5` (environment
   `esp_c5`). With the older ROVER_2, the software works but caps traction at 20 %.
3. `./deployer.sh utilisateur@adresse-du-pi`: copies the code, stops `fieldmind-embarque`, installs and starts
   the `fieldmind-rover` service. Station: `http://<Pi IP>:8080`.

Log: `ssh utilisateur@adresse-du-pi journalctl _SYSTEMD_USER_UNIT=fieldmind-rover.service -f -o cat`.

`utilisateur@adresse-du-pi` stands for the Pi's user and address.

## First tests (in this order)

1. **Wheels lifted**, *banc* (bench) mode: *Système > Calibration > Sens des roues* (wheel direction); correct with
   `inverseg` / `inversed` in the console if a wheel turns backwards.
2. Rover on the ground, stationary: *Centrale au repos* (IMU at rest; checks the MPU-6050 axes).
3. **Stereo**: print a 9 × 6 checkerboard (25 mm squares), take about twenty captures from
   varied angles, then *Calculer* (compute).
4. 2.5 m clear in front: *Vitesse* (speed; PWM → m/s), then *Braquage* (steering; servo angle → wheels).
5. *Manuel* mode, arm, drive slowly; check that it stops in front of a cardboard box (lidar) and in front of
   a low object, under 30 cm (stereo).
6. *Auto* mode: « avance de 2 mètres » ("move forward 2 meters"), « tourne à gauche » ("turn left"), click on the image, then voice.

## Settings

### Exploration and mental map on the GX10

« Explore la pièce » ("explore the room") or the **Explorer la pièce** button starts a progressive mission. The Pi
computes several next points in the known space, with enough clearance for the rover
and an accessible path. The GX10 receives these candidates, the mission, and the objects and walls
observed, then chooses the next step. The Pi re-validates that choice and runs the local
navigation. On arrival, a new observation enriches the map and the loop starts again.
« Explore pour trouver la porte » ("explore to find the door") adds a search; a remembered object is re-checked with
the camera before being declared found. An unsuccessful local search can also open
an exploration. The visited, refused and upcoming points are shown on the map.

In the GX10 profile, the camera also feeds scene understanding continuously, even when
stopped: `ia.gx10_cartographie = true`, target interval `ia.gx10_periode_s = 3`. Only one
scene analysis is in progress at a time; if an inference takes longer, the rover resumes with
the next recent image. Each image carries its pose and its lidar/stereo measurements frozen
before sending. D-FINE provides the fast detections in parallel; the same visual analysis
feeds the map and the watch summary, without a second model call for the same view.
The model receives the updated spatial context at each decision. In the real test of 21/10,
an analysis took about 6 to 7 s: the 3 s interval remains a target, not a guaranteed rate.

An object must be seen again in two captures analyzed by the visual model, at least
0.5 s apart, to become a confirmed landmark; the fast detector alone does not confirm it.
Hypotheses expire after 45 s without visual observation, confirmed static objects
after one hour and moving ones after five seconds. Measured wall segments stay
in session memory for one hour. A destination such as "kitchen" requires several distinct,
characteristic objects confirmed in the same area; an ambiguous room is refused.
Annotations are displayed in 2D and 3D; doors,
windows and facades can be named by the model. Positions and dimensions keep
their provenance: a projection onto the ground or a visual height remain estimates.
This representation is schematic, without a dense 3D mesh or invented geometry behind
doors. Resetting the odometry also erases this memory and invalidates
earlier image responses. The memory is not reloaded into a new frame at boot.

By default, an exploration is limited to 20 steps, 8 m around the mission's starting point and
10 minutes. Limits reached and inaccessible areas are reported as a partial
exploration; no missing path or object is presented as found. If the GX10 is unavailable,
progress stops instead of executing an invented choice. Pause, cancel, disarm
and emergency stop invalidate in-flight decisions. The supervisor and the watchdog stay on the Pi.

The link uses `FIELDMIND_GX10_HOTE` (for example the GX10's Tailscale address) and
`GX10_API_KEY` in the private `.env` file, copied to `~/.config/fieldmind/env` at deployment.
The Pi program reaches it through its `.env` link. These files are never versioned.

### Station settings

The station settings are saved on the Pi (`~/.config/fieldmind/reglages.json`) and
restored at each startup. Réglages > Poste > **Tout réinitialiser** (reset all) returns to the values of
`config/rover.toml`, while keeping the wiring, the mounting and the calibrations (`esp.*` including
`esp.moteurs_permutes` and `esp.sens_inverse`, camera ports and tilt, IMU
axes, PWM per m/s, servo).

The Pi's local settings may differ from the repository values. The shipped configuration
enables stereo (`perception.stereo_active = true`, width 640 px, target 4 images/s),
in an advisory role (`securite.stereo_bloquante = false`). It was also enabled
during the last test on 21–22/10; the old "stereo off" note no longer describes this state.
The hardware survey of 21/10 noted horizontal cameras and a driver VR input
powered at 3.3 V by the ESP: 100 % PWM corresponded to about two thirds of the maximum
speed on the potentiometer. Keep the calibrations of the real assembly when deploying.

## Trailer flaps

Two **continuous rotation** servos on **D2** (left) and **D3** (right) of the ESP, firmware
**ROVER_4** (`volg`, `vold`: 90 = stop, above one direction, below the other, -1 cuts the
signal). With no position sensor, unfolding or folding = turning at `remorque.volets_vitesse` %
for the time needed to cover `remorque.volets_angle_deg` (90°), computed from
`remorque.volets_tours_min` (actual speed at full throttle: flap goes too far → increase it), then stop; nothing is sent at startup, and nothing
moves if the requested state is already the known state. The direction of each flap can be inverted in
*Réglages > Remorque*. *Pilotage* > **Déplier / Replier les volets** (unfold / fold the flaps), or *Drone* panel > **Prêt au
décollage** (ready for take-off; no command is sent to the drone, whose motor protocol has yet to be
identified: [DRONE.md](DRONE.md)). When unfolded, the trailer outline used by safety
widens. D2/D3 then no longer read the wheel speed sensors.

## IMU mounted at an angle

Système > Calibration > **Centrale au repos** (IMU at rest), rover **flat** and stationary for 3 s: the
measured gravity defines the vertical (`imu.niveau`), whatever the angle of the GY-521. Roll,
pitch and yaw rate are then correct; the yaw integrated by the firmware (around the
tilted axis of the sensor) is no longer used, the Pi integrates the yaw rate.

## Safety

- Starts in *arrêt* (stop) mode, disarmed. The `conduite.armer_auto` setting arms when switching to
  *manuel* or *auto*, as well as when the emergency stop is explicitly released. It does not arm
  automatically in *banc* mode. Off by default, enabled on the Pi for the tests.
  Releasing the emergency stop clears old setpoints: a new command is required.
- Every setpoint (station, mission, AI, ROS 2) goes through the supervisor: corridor swept along
  the planned arc, stopping distance, stale sensors, tilt, impact, stall, dead man's switch,
  zone limit.
- **Only the lidar stops the rover** (`securite.stereo_bloquante = false`, by default): a low obstacle
  seen by the stereo in the corridor only slows it down (`securite.vitesse_obstacle_bas_m_s`),
  and the memory of these obstacles fades within a few seconds if they are not seen again
  (`cartographie.oubli_bas_s`). In navigation, isolated detections are filtered out and clusters
  make the passage more costly, without forbidding it: the rover prefers the clearest spot.
  Smoothing keeps this detour. `navigation.penalite_indicatif` sets the cost on the map and
  `navigation.poids_indicatifs` that of the local arcs. With `securite.stereo_bloquante = true`,
  low obstacles become blocking again.
- Manual with collision avoidance off: obstacles and "stall" detection ignored (wheels
  lifted), speed capped by `securite.vitesse_sans_anticollision_m_s`.
- ROVER_4 firmware: with no setpoint for 400 ms, traction goes to zero. The D2/D3 flap outputs
  stay stopped at startup; their timed movement is not position feedback.
- ARRÊT (stop) button (or Space): latched stop, explicit re-arming. « Stop » by voice: immediate
  stop.
- **There is no hardware emergency stop yet** (cutting the traction battery):
  it must be wired before driving near anyone.
