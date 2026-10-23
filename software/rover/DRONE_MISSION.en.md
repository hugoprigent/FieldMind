[Français](DRONE_MISSION.md) · **English**

<a id="préparation-locale-de-la-mission-drone"></a>
# Local drone mission preparation

The drone can be turned off and the Pi unplugged: the workshop runs on this Mac. No actual flight or deployment has been performed for this preparation. Connection to the CooleerX is not declared validated by a simulation result.

<a id="essayer-maintenant"></a>
## Try now

From the project:

```sh
software/rover/deploiement/lancer_atelier_drone.sh
```

Open <http://127.0.0.1:8096>, **Piloting → Drone workshop — full local simulation**. The launcher selects simulated hardware, listening on localhost, separate settings in `.donnees-atelier/`, and disables AI models. It does not contact the Pi, does not open a real serial port, and does not start physical BLE discovery.

- **Simulate the full mission**: climbs to 1 m, takes six photos around a measured tower, waits for the downward camera then re-centers on the cross and descends in steps of 15 cm down to 22 cm above the plate, before the final landing.
- **Try manual piloting**, then **Take off in simulation**: climbs towards 70 cm. Holding **Up/Down** changes the altitude setpoint; releasing maintains this target altitude. Other buttons command yaw, forward/backward translation and left/right. **Simulated downward camera** changes the bench sensor.
- **Interrupt and land in simulation** triggers the return to ground. Loss of the joystick heartbeat during 0,5 s also causes this retreat; no automatic movement resumption on reconnection or return of focus.
- Choose a failure scenario to check behavior. The archives of the last attempt are accessible via **Simulated photos / Simulation report**. Files are kept in `~/.local/share/fieldmind/drone-simulations/`.

The commands of this workshop use `drone_atelier` and `image.drone_simule` images, distinct from the real drone's driver and stream. The server has no automatic launch button on restart.

<a id="boucle-de-commande-préparée"></a>
## Prepared command loop

`fieldmind/drone/autopilote.py` produces **bounded speeds**, not invented positions or hardware-specific packets. It receives dated altitude, heading, ground state, camera identity and visual target. The bench simulates altitude/heading sensors; these sensors are not magically available on the CooleerX.

Rotation is tracked by unwrapping the heading around ±180°. Each sector is photographed after stabilization, then the tour is completed before the camera change. A requested camera is not considered active before a recent image identified as coming from the downward sensor.

Descent follows the loop: **detect → re-center → stabilize → descend one step → check again**. If the cross leaves the centering zone, the step is suspended. If the target disappears, the requested vertical speed returns to zero; a prolonged loss triggers a retreat landing, only after confirmed takeoff. A landing command is never used to prove that landing succeeded: the return to ground must be observed. After loss of any estimation, the bench signals "landing not confirmed" even if the simulated dynamics actually touched the ground.

Attitude stabilization must remain in the drone's onboard controller. The Pi holds the mission and the setpoint heartbeat; it does not replace an inertial navigation system with five images per second. RC speed/axis gains and real signs must be measured before connecting these outputs to an aircraft.

<a id="pourquoi-le-bouton-décoller-pouvait-ne-rien-faire"></a>
## Why the Take off button might do nothing

The initial driver sent an isolated generic frame on the BLE characteristic that also receives video commands. No test proved that the motor controller understood this frame. "You need to increase throttle after the click" is a hypothesis, not an established procedure for this unit.

On the **Wi-Fi RCFPVPro** variant documented by [e88pilot](https://github.com/Volonid/e88pilot/blob/master/backend/protocols/e88pro_rc_protocol_adapter.py), an open session from local UDP 7070 is required, followed by a continuous RC stream from 7060 to `192.168.28.1:7080`, approximately every 40 ms. Its format differs from simple `66 … 99`. The same bit toggles takeoff/landing: repeating it may be incorrect. This variant announces the absence of telemetry and unconfirmed camera commands.

`fieldmind/drone/protocoles.py` prepares this encoding, checksums, heartbeat at 25 Hz, command expiration, and impulse actions. It is **not** connected to the BLE CooleerX. An open UDP transport session explicitly requires a validation form linked to identity, protocol, and SHA-256 of a capture, with tests for axes, stop, and link loss. No actual validation form is provided. No hardware connection is opened during an import.

The heartbeat does not deduct "in flight" from the takeoff send: an observed return must call `confirmer_en_vol(True)` before an automatic landing. It never rearms a session after stop or expiration. The previous hardware interface remains blocked until the protocol of **this unit** is identified.

<a id="répartition-pi--asus-ascent-gx10"></a>
## Pi / ASUS Ascent GX10 Distribution

- **Pi**: reception, image timestamping, mission, bounded commands, and independent RC heartbeat. Local crosshair vision remains possible (previous measurement on Pi: approx. 10–16 ms/image at 240 × 320).
- **Optional GX10**: crosshair detection and relative estimation from images. The prepared service has no flight endpoints and uses no LLM. It returns only a target, the image sequence, and the calibration identifier.
- **Pi Client**: one flight request at a time, maximum delay of 300 ms; age evaluated with the local capture time, never with the reception time or that of the GX10. Bad sequence, bad calibration, NaN, or results older than 500 ms are ignored. The GX10 delay suspends descent and eventually causes fallback.

The metric algorithm uses a known-size planar `+` cross, optical calibration, and a camera approximately perpendicular to the light plate. It estimates height by the cross area, avoiding mistaking rotation for scale change. Tilt and flat ground assumptions must be verified on the real drone; this is not a general 3D pose estimation or GPS localization. A cross without calibration yields only an offset in the image.

Example local service with **simulator-only** calibration:

```sh
cd software/rover
# Définir FIELDMIND_DRONE_VISION_JETON dans l'environnement, sans le publier.
.venv/bin/python -m fieldmind.drone.vision_service \
  --calibration config/drone_vision_simulation.json --hote 127.0.0.1 --port 8092
```

On the GX10, install Python, NumPy, OpenCV, and aiohttp, and provide real calibration before hardware use. No GX10 service was installed or contacted here. The token is mandatory, the server limits JPEG size/dimensions, and refuses to accumulate images when busy. The client is in `vision_distante.py`.

<a id="essais-à-reprendre-au-retour-du-drone-chargé"></a>
## Tests to resume upon drone return loaded

1. Find the protocol of the actually used application and record exchanges at stop, then for camera, calibration, arming, and sticks. For an Android application, an HCI log exported from the **phone** is useful for BLE, and a network capture of the application for Wi-Fi. `btmon` on the Pi alone does not allow capturing an established link between the phone and the drone.
2. Bench with propellers removed: validate identity and transport frames, cadence, neutral, signs, calibration, arming/takeoff, and stops. Do not test random arming sequences indoors with propellers mounted.
3. Confirm camera switching, its orientation, and calibration. Without an identified ventral image, the mission engine does not allow approach to the cross.
4. Establish an altitude and heading estimate for the entire mission. The cross area provides relative altitude when visible; it does not provide altitude during the front camera scan. A flight feedback or another validated estimate will be required for this phase.
5. Adjust axes and gains, then conduct supervised separate tests: climb/hold, yaw, translations, aiming, stops on large ground targets, then stationary tow.

These physical tests remain necessary: neither simulation software nor a more powerful GX10 can remotely determine how an unpowered drone reacts.

<a id="réexécuter-la-validation"></a>
## Re-run validation

```sh
cd software/rover
.venv/bin/python -m fieldmind.drone --scenario tous --graines 20 --sortie ../../tmp/drone-validation
.venv/bin/python -m pytest -q -m 'not lent'
.venv/bin/ruff check fieldmind tests
node --test tests/web/*.test.mjs
```

The test bench varies initial positions, adds inertia and drift, and returns synthetic images analyzed by the detector. The target is not injected from the simulator's actual position. Its dynamics remain simplified: the accuracies obtained in these tests are not guaranteed accuracies on the real drone.

<a id="résultats-de-cette-préparation-locale"></a>
## Results of this local preparation

- **420 Python tests successful**, two existing long scenarios excluded; **22 JavaScript tests successful**, lint and formatting of new modules verified.
- **140 simulations**: twenty seeds per scenario. Forty `normal/derive` missions completed with six photos and at least five stops. The other hundred tests produce the expected fallback or unconfirmed landed state, without rearming. [Test summary](validation/drone/synthese-2026-10-20.json).
- Local browser control: mission launch, six photos, measured turn, sensor switch, six stops, and simulated contact; then manual test of takeoff, climb, altitude hold, yaw, descent, bottom camera, and abort/landing.
- Local preview restarted with the launcher isolated, AI disabled. No access to the Pi after the offline work instruction, no deployment or real flight.

The lack of motor validation on this drone remains a physical task to resume, with the steps above. Fallback scenarios test software logic: they do not prove that the CooleerX understands the fallback framework.
