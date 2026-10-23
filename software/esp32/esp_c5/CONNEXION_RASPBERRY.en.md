[Français](CONNEXION_RASPBERRY.md) · **English**

<a id="firmware-du-rover-et-liaison-raspberry-pi"></a>
# Rover firmware and Raspberry Pi link

Program to upload: **esp_c5**, selected by default in PlatformIO. Sources: `src/main.cpp` (sensors/screen) and `src/actionneurs.cpp` (motors/servo). The esp_c5_complet alias produces the same program. Serial identifier: **ROVER_3** (ROVER_2 commands unchanged, plus the [ROVER_3 extension](#extension-rover_3)). The test_tout_moteur remains available, with the same controller and no sensors.

<a id="connexions-actuelles"></a>
## Current connections

| XIAO ESP32-C5 | Destination |
| --- | --- |
| D0 / GPIO1 | Left motor speed, red wire |
| D1 / GPIO0 | Left motor D/R, yellow wire, reversed direction in software |
| D9 / GPIO9 | Right motor speed, red wire |
| D4 / GPIO23 | Right motor D/R, yellow wire |
| D10 / GPIO10 | BRAKE for both drivers, green wires |
| D7 / GPIO12 | Servo signal |
| D5 / GPIO24 | Common SDA for IMU, barometer, OLED and Knob |
| D6 / GPIO11 | Common SCL for IMU, barometer, OLED and Knob |
| GND | Common references for ESP, drivers (orange), servo power supply |
| D2 / GPIO25 | Signal for the **left flap** servo of the trailer (ROVER_4, `volg`) — or, optionally, the left driver's SC output ([see below](#sorties-sc-des-drivers-facultatif)) |
| D3 / GPIO7 | Signal for the **right flap** servo of the trailer (ROVER_4, `vold`) — or the right driver's SC output |

The two yellow D/R wires remain separate. D8 is no longer used. D7 is occupied by the servo: no GPS on this pin in this setup. The servo is powered by its adapted external branch; motors by their drivers. The ESP-Pi USB cable carries the serial link and board power, not the power for the motors or servo. The drivers' STOP remains unpiloted.

<a id="mise-en-route"></a>
## Startup

<a id="deux-roues-dans-le-sens-de-marche"></a>
### Two wheels in the direction of travel

Hugo's latest test shows correct left and right still reversed. The new fault is left=1, right=0 : only the right fault has changed since the previous revision. The old right key inv_d is ignored ; new right corrections are stored under inv_d_v2. The left setting inv_g remains preserved. Corrections saved with this revision take priority on subsequent startups ; `etat` displays each value. The physical direction must be verified with wheels free from the ground, not deduced from colors.

1. Send `m 0`, wait for physical stop, then `sens 0` (before logic).
2. Try `g 5`, then `g 0`. The right side must not receive this command.
3. Try `d 5`, then `d 0`. The left side must not receive this command.
4. If a wheel reverses, correct only its inversion, two motors at zero : `inverseg 0` or `inverseg 1` for left ; `inversed 0` or `inversed 1` for right. Choose the opposite of the value displayed by `etat`.
5. Retest this wheel, then send `m 5` to test both.

The correction is stored in flash and shared between main and combined test. It does not start any motor ; running speeds and directions are not restored on reboot. The "Correct Front" buttons on the PC page perform this setting.

1. Upload **PlatformIO > esp_c5 > General > Upload**, then close the monitor.
2. Connect the XIAO via USB to the Raspberry Pi. The servo requests center **1500 us at startup** ; traction at zero. The first centering may be fast.
3. Open Pi supervision, normally `http://rover.local:8080/` (Pi address to be adapted). The address may change with the network. The service is fieldmind-embarque.
4. In Serial Ports, identify the native ESP USB (`/dev/ttyACM0` most often). Check **ROVER_3**, then send `etat` in the field for this port.
5. Test wheels free from the ground : `m 5`, `m 0`, `s -10`, `centre`, then `stop`.

| Command, terminated by Enter | Effect |
| --- | --- |
| m 10 | Same PWM command 10 % to both motors, maintained until m 0 or stop |
| m 0 | Traction at zero, servo maintained |
| g 10 / d 20 | Left PWM 10 % / right 20 %, without changing the other motor |
| g 0 / d 0 | Stop command for one wheel only |
| sensg 0 / sensg 1 | Left forward/reverse direction ; left at zero and physical stop required |
| sensd 0 / sensd 1 | Right forward/reverse; right at zero and physical stop required |
| inverseg 0/1 / inversed 0/1 | Front calibration by wheel, two motors at zero; memorized |
| s -10 / s 10 | Servo position relative to center (indicative angles) |
| centre | Center target, 1500 us |
| v 30 | Maximum servo command speed, in indicative degrees/second |
| sens 0 / sens 1 | Reverse both directions; after m 0 and complete physical stop |
| frein 0 / frein 1 | Zero PWM then common BRAKE level |
| stop | Zero traction PWM and servo signal cut; power supplies remain active |
| etat / aide | Commands, faults, frequencies / available commands |
| p 12 -8 5.5 | ROVER_3: Left/right signed PWM (-100..100) and servo angle; silent |
| hb | ROVER_3: refreshes the watchdog, no response |
| wd 400 / wd 0 | ROVER_3: watchdog 100..2000 ms / off (off at startup) |
| inv 500 | ROVER_3: wheel at zero 100..3000 ms before a p changes its direction |
| tel 20 / tel 0 | ROVER_3: T line at 0..50 Hz; periodic state suspended as long as tel > 0 |
| imu 50 / imu 0 | ROVER_3: I line at 0..100 Hz; imu 0 = original text IMU line |
| lacet0 | ROVER_3: resets the integrated yaw to zero |
| roues 1 / roues 0 | Read SC outputs on D2/D3 and two additional fields in T / off (fault) |
| volg 0..180 / vold 0..180 | ROVER_4: left flap servo (D2) / right (D3), 50 Hz, 500..2500 us; first direct command, then 90 deg/s; -1 cuts the signal. Refused if wheels 1; wheels 0 keep the flaps |

The advance/arm/test commands and standalone numbers from the old main are replaced by these common commands with the combined test. Do not send just 10 : send **m 10**.

The main publishes measurements in text format already decoded by the Pi supervision every 500 ms, and actuator state approximately every second when USB is connected. The Pi already has line sending on each port: no network-to-JSON conversion is required for this console. The PC Web Serial page also accepts ROVER_2, for direct testing on PC. It offers speed/direction commands and front correction for each wheel. The m command sets both percentages but preserves their individual directions. BRAKE remains shared on D10. Only one software should open the USB port at a time.

<a id="extension-rover_3"></a>
## ROVER_3 Extension

The Pi driver reads `[FIRMWARE]`. With ROVER_3, it sends `inv`, `v`, `tel 20`, `imu 50` and `wd 400`, then a line `p` at each driving period (20 Hz), which also serves as a heartbeat. With ROVER_2, it keeps `g`, `d`, `sensg`, `sensd`, `s`. Without `wd`, `tel` nor `imu`, ROVER_3 behaves like ROVER_2.

- **wd** : without `p` accepted nor `hb` during the delay, traction at zero like `m 0`, servo maintained, single line `[CHIEN] expire`. No recovery: a new `p` is required. As long as the delay is exceeded, `m`, `g` and `d` non-zero values are refused. Other lines, valid or not, do not refresh the watchdog. It is checked at each loop of the actuators task (approximately 1 ms).
- **p** : integers -100..100, sign = logical direction of the wheel (>= 0 forward,
  < 0 backward) ; angle with the scale and limits of `s`. The servo is reattached if it was disconnected; its trajectory is only restarted if the target changes. Sign change of a commanded wheel: PWM to zero immediately, wait `inv` ms wheel at zero, D/R change, then 100 ms preparation and the new value; nothing blocks and the other wheel continues. Refusal and stop if brake or fault, like `m`. No additional ceiling added (0..100 %, like `m`).
- **T** : `T <ms> <pwm_g> <pwm_d> <servo_deg> <frein> <defaut> <chien>`, for example `T 123456 12 -8 5.5 0 0 1`. PWM applied (0 during a wait or preparation), sign = applied logical direction; servo = current setpoint, one decimal; watchdog = 1 if active and not expired. With `roues 1`, two more fields at the end (`<hz_g> <hz_d>`, see SC outputs).
- **I** : `I <ms> <ax> <ay> <az> <gx> <gy> <gz> <lacet_deg>`, for example `I 42 0.012 -0.500 1.000 1.23 -0.01 0.15 -12.40`. Accelerations in g (3 decimals), rotations in deg/s and yaw in degrees (2 decimals), GY-521 reference. Integrated yaw from Z gyroscope, in [-180, 180). Traction at zero for more than 1 s and gyroscope calm (each axis less than 1 deg/s from its average over 0,25 s and, once bias learned in 2 s of stop, Z less than 3 deg/s from the bias): bias adjusted and yaw frozen. Lines emitted only after a successful read; as long as `imu` > 0, the line `IMU ...` of 500 ms is suspended.
- A detected USB loss (250 ms) resets `wd`, `tel` and `imu` to zero; `inv` remains.
- `etat` adds `[PILOTAGE] Chien=400 ms (ok|expire|coupe) ; inv=500 ms ; tel=20 Hz ; imu=50 Hz ; lignes perdues=0` (discarded USB lines, full queue).

| Event | Line sent |
| --- | --- |
| wd 400 / wd 0 | `[CHIEN] Delai 400 ms ; renouvele par p ou hb, sinon traction a zero.` / `[CHIEN] Coupe.` |
| Expiration | `[CHIEN] expire : traction a zero, servo maintenu ; envoyer p pour reprendre.` |
| m, g, d after expiration | `[CHIEN] Delai depasse : m, g ou d refuse ; envoyer hb ou p.` |
| inv 600 | `[PILOTAGE] Inversion : roue a zero 600 ms avant changement de D/R.` |
| tel 20 / tel 0 | `[PILOTAGE] Telemetrie T a 20 Hz ; etat periodique suspendu.` / `[PILOTAGE] Telemetrie coupee.` |
| imu 50 / imu 0 | `[CENTRALE] Lignes I a 50 Hz ; ligne IMU texte suspendue.` / `[CENTRALE] Ligne IMU texte toutes les 500 ms.` |
| lacet0 | `[CENTRALE] Lacet remis a zero.` |
| malformed or out-of-bounds pin | `[ERREUR] Commande invalide. Exemples : m 10, s -30, v 30, stop. Traction a zero, signal servo coupe.` |
| p non-zero, active brake | `[ERREUR] Commande moteur refusee : verifier frein et etat. Traction a zero, signal servo coupe.` |
| p out of range servo | `[ERREUR] Angle hors de la course actuelle. Tape etat. Traction a zero, signal servo coupe.` |
| p default | `[DEFAUT] Cause=...` then `Sorties bloquees ; voir etat apres redemarrage.` |

Among the `[CHIEN]` lines, only the expiration contains the word "expire". The test_tout_moteur keeps the identifier TOUT_MOTEUR_4 and accepts `p`, `hb`, `wd`, `inv` and `tel`; it responds `[CENTRALE] Absente de ce programme ; commande ignoree.` to `imu` and `lacet0`.

<a id="sorties-sc-des-drivers-facultatif"></a>
## SC outputs from drivers (optional)

`roues 1` counts the rising edges of the `SC` output from each driver, via interrupt: left on D2 / GPIO25, right on D3 / GPIO7. At startup (`roues 0`), the code does not touch these pins and nothing changes. `roues 1` sets them as inputs with no internal pull-up and resets the counters to zero; `roues 0` removes the interrupts and leaves them in high-impedance input mode. This setting is not reset by a USB disconnection (unlike `inv`).

- **Anti-bounce**: a rising edge arriving less than 200 us after the previous accepted rising edge is rejected and counted separately (more than 5000 pulses/s).
- **Frequency**: average of the pulses in each window of approximately 100 ms, in pulses/s, one decimal place; 0 no edge during 0,5 s (minimum 2 imp/s). SC does not provide direction: the sign is that of the logical direction applied to the wheel (a wheel pushed backward while commanded forward appears positive).
- **T** with `roues 1` : 10 fields, `T <ms> <pwm_g> <pwm_d> <servo_deg> <frein>
  <defaut> <chien> <hz_g> <hz_d>`, par exemple `T 123456 12 -8 5.5 0 0 1 45.0 -38.5`. Sans `roues 1`, the T line keeps exactly its 8 fields. The Pi driver reads the 8 first fields and does not yet use the last two.
- **status** with `roues 1` : `[ROUES] SC gauche D2/GPIO25 : fronts=1234 rejets=0 45.0 imp/s ; droite D3/GPIO7 : fronts=1190 rejets=2 -38.5 imp/s` (accumulations since `roues 1`). Responses: `[ROUES] Lecture SC active : ...` and `[ROUES] Lecture SC coupee ; D2/D3 en entree haute impedance.`
- **Calibration**: the number of pulses per wheel rotation depends on the motor. With free wheels, `roues 1`, manually rotate the wheel one full turn with the driver powered, then read `fronts` in `etat`.

<a id="ce-que-lon-sait-du-signal-sc"></a>
### What we know about the SC signal

- The manufacturer does not document it: according to [MAD-EE](https://mad-ee.com/easy-inexpensive-hoverboard-motor-controller/), the ZS-X11H documents present it as a reserved interface, with no technical support; the output changes state with each Hall sensor change (90 changes per wheel revolution on their hoverboard motor, measured with a logic analyzer). A counted pulse here would correspond to two Hall changes.
- On the ZS-X11A, a user describes a logic output with a duty cycle 1:1, frequency linked to speed, without voltage level ([Arduino forum](https://forum.arduino.cc/t/interfacing-to-zs-x11a/499628)).
- No source found specifies the high level (5 V or 3,3 V) nor the output type (active or open collector), neither for the ZS-X11H nor for the ZS-X11D1. The board powers its Hall sensors at 5 V: a high level of 5 V is plausible, **not established**.
- The ESP32-C5 accepts at most VDD + 0,3 V (3,6 V) on an input and recognizes a high level starting from 0,75 x VDD, approximately 2,5 V; internal pull-up of 45 kohm ([ESP32-C5 datasheet v1.5](https://documentation.espressif.com/esp32-c5_datasheet_en.pdf), table 5-4).

<a id="branchement-propose-apres-mesure"></a>
### Proposed connection, after measurement

Measure first, driver powered and **XIAO not connected**: voltage between `SC` and the `GND` of the driver with a multimeter (or oscilloscope), wheel turned slowly by hand.

| Measurement on SC | Adaptation towards D2 (left) or D3 (right) |
| --- | --- |
| Alternates between 0 V and approximately 5 V | Voltage divider: SC → 10 kohm → input; input → 20 kohm → GND. High ≈ 5 V x 20/30 = 3,3 V |
| Stays low or floats, without a clear high level | Open collector likely: pull-up 10 kohm from input to **3V3 of the XIAO** (never to the 5 V of the driver), 1 kohm in series between SC and the input |
| Alternates between 0 V and approximately 3,3 V | 1 kohm in series is sufficient |

With the divider, verify at the midpoint before connecting it to the XIAO: high level between 2,5 and 3,3 V, low level below 0,8 V. A high level below 2,5 V indicates an output with high internal resistance: measure before changing values. The `GND` of the driver remains common with that of the ESP; one SC wire per driver, without connecting the SCs of both cards. Avoid leaving a driver powered when the XIAO is not: current would flow through the input (limited to less than 0,5 mA by the 10 kohm resistance).

D2 / GPIO25 and D3 / GPIO7 are strapping pins of the ESP32-C5 (datasheet, section 3): GPIO25 selects the SDIO sampling edge (not used here), GPIO7 the JTAG source only if the eFuse EFUSE_JTAG_SEL_ENABLE is programmed (0 by default). The boot mode depends on GPIO26/27/28, not on D2/D3. D8 / GPIO8 remains free and is not used by this code.

<a id="verification-et-limites"></a>
## Verification and limits

The USB/servo/motor control runs in a separate task from I2C. The servo remains on ESP32Servo 3.2.1, 50 Hz, 800..2200 us. The motors keep 5 kHz. Timers are verified independently of the duty cycle. An error cuts the outputs and locks the fault; a USB loss detected during 250 ms requires stopping without automatic resumption of traction. Closing a page or the Pi crashing does not guarantee that USB is declared disconnected. Without `wd`, motor commands have no automatic expiration; with `wd`, silence from the Pi cuts traction after the delay. USB outputs go through a queue: if the host no longer reads, lines are lost but the actuator task and watchdog continue.

Percentages and angles are setpoints, not physical measurements. The previous GPS pinout and the JSON contract proposed in contrat_esp_pi.md are not implemented here. Camera and lidar remain managed directly by the Pi. Autonomous navigation is on the Pi; the firmware only adds the watchdog, combined command, and fast lines.

Compilation and software tests without real USB do not validate the mechanics or physical stop. No movement commands nor any flash are automatically sent to the assembly by Codex. The Pi link was unreachable during preparation: its validation via USB remains to be performed.

ROVER_3 validation of 19 October 2026 (with SC reading), without hardware: esp_c5, test_tout_moteur, esp_c5_capteurs, test_moteur and test_servo compilations successful; PC tests successful, including 75 verifications of ROVER_3 simulated real controller (69 for test_tout_moteur) and 87 of ROVER_3 and SC logic. No upload nor test on the rover: watchdog, inversion, T/I lines, central task and SC reading (including electrical level) remain to be tested.

Previous validation (ROVER_2): successful builds of esp_c5, test_tout_moteur, and esp_c5_capteurs. Existing PC tests passed, plus 34 verifications of the real controller with simulated hardware (28 nominal, 6 task failure), totaling 34: independent speeds and directions, calibration remembered and rejected during movement, save failure and PWM failure of a single wheel. USB transport: 4 tests passed; browser tested with simulated ROVER_2 and TOUT_MOTEUR_4, m/s/v/center commands, STOP, and blocking in fault/unknown firmware state.
