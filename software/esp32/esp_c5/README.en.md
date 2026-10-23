[Français](README.md) · **English**

# Rover ESP32 software - ESP C5

Open `software/esp32/esp_c5/` in VS Code with PlatformIO.
The default program is **esp_c5**, identifier **ROVER_3**: two motors,
servo, IMU, barometer, display and optional Knob. ROVER_3 keeps all the
ROVER_2 commands and adds driving by the Raspberry Pi (see below).

**For the connection to the Raspberry Pi: [current guide and pinout](CONNEXION_RASPBERRY.md) (in French).**

- Left motor: speed D0/GPIO1, D/R D1/GPIO0 inverted in software.
- Right motor: speed D9/GPIO9, D/R D4/GPIO23.
- Common BRAKE D10/GPIO10; servo D7/GPIO12.
- I2C kept: SDA D5/GPIO24, SCL D6/GPIO11.
- At startup: traction at zero and servo centered (1500 us).

The two yellow D/R wires stay separate. D7 cannot receive a GPS
at the same time as the servo. Grounds are common; the actuators
keep their own suitable power supplies.

| PlatformIO environment | Program |
| --- | --- |
| **esp_c5** (default) | Full main: sensors/display + two motors and servo |
| esp_c5_complet | Alias of the full main |
| test_tout_moteur | Same motor/servo controller, without sensors |
| test_moteur | Old single-motor test, PWM D0, D/R D1, BRAKE D10 |
| test_servo | Servo-only test on D0, to be disconnected from the driver |
| esp_c5_capteurs | Sensors/display only, no actuator output configured |

## Uploading and commanding

Choose **PlatformIO > Project Tasks > esp_c5 > General > Upload**.
Close the monitor before opening the port from the Raspberry Pi or a browser.
The main and the combined test accept the same commands at 115200 baud:

```text
etat
m 5
m 0
s -10
centre
v 30
stop
```

`m` sets the common percentage of both drivers, `s` the angle relative to center,
`v` the servo setpoint speed. The values are setpoints, not measurements.
`g 10` and `d 20` set the wheels independently; `g 0` or `d 0` stops
only the chosen wheel. `sensg 0/1` and `sensd 0/1` set their forward/reverse,
with the wheel concerned at zero and after it has physically stopped.
`inverseg 0/1` and `inversed 0/1` correct which way is forward, with both motors
at zero; these two settings are stored in flash. See the Raspberry Pi guide.
The motor setpoint persists until another command, stop or a detected USB loss.
`stop` cuts the traction command and the servo signal; the power supplies stay on.
`sens 0/1` changes the common logical direction, only after m 0 and a mechanical stop.
Bare numbers and the old avance/armer/test commands are not
commands of the new main. The Pi console simply uses the lines above.

The [local USB page on a PC](INTERFACE_USB.md) accepts ROVER_3, ROVER_2 and TOUT_MOTEUR_4.
The [combined test](TEST_TOUT_MOTEUR.md) keeps its separate console.
The [servo-only test settings](TEST_SERVO.md) concern D0, not D7.
The [motor diagnosis](DIAGNOSTIC_MOTEUR.md) mainly concerns the single-motor test.
These pages are in French.

## ROVER_3 extension: driving by the Raspberry Pi

Added commands, all off at startup (details and formats in the
[Raspberry Pi guide](CONNEXION_RASPBERRY.md#extension-rover_3)):

```text
wd 400        watchdog: without p or hb for 400 ms, traction to zero
hb            renews the watchdog, no reply
p 12 -8 5.5   signed left/right PWM and servo angle, silent except on error
inv 500       wheel at zero for 500 ms before a p changes its direction
tel 20        compact T line at 20 Hz (periodic status suspended)
imu 50        I line at 50 Hz with integrated yaw; lacet0 resets it to zero
```

The Pi driver (`software/rover/fieldmind/materiel/esp.py`) reads `[FIRMWARE]`:
if it sees ROVER_3, it sends `inv`, `v`, `tel 20`, `imu 50` and `wd 400`, then a
`p` line at each driving period (20 Hz), which also serves as the heartbeat. With
ROVER_2, it falls back to the `g`, `d`, `sensg`, `sensd`, `s` commands.
Without `wd`, `tel` or `imu`, the firmware behaves like ROVER_2 (same lines).

Optional: `roues 1` reads the `SC` output (speed pulses) of each
driver on D2 / GPIO25 (left) and D3 / GPIO7 (right), and appends `<hz_g> <hz_d>`
to the end of the T line. By default (`roues 0`), these pins are not touched.
The electrical level of SC is not documented: measure it and adapt the
signal before connecting it ([proposed wiring](CONNEXION_RASPBERRY.md#sorties-sc-des-drivers-facultatif)).

## Sources and verification

- `src/main.cpp`: measurements, display and Knob navigation.
- `src/actionneurs.cpp`: common controller, USB console and outputs; stand-alone task in the main.
- `src/actionneurs.h`: controller interface.
- `src/test_tout_moteur.cpp`: runs the controller without sensors.
- `src/test_tout_moteur_broches.h`: pins and inversion shared by main/test.
- `src/test_tout_moteur_logique.h`: commands and servo trajectory.
- `src/test_tout_moteur_pwm.h`: read-back of the LEDC frequencies.
- `src/pilotage_rover3.h`: watchdog, pass through zero before reversing, T and I formats.
- `src/centrale_lacet.h`: integrated yaw and gyroscope bias estimated at rest.
- `src/console_serie.h`: USB output queue and single writer task.
- `src/roues_sc.h`: SC pulse counting, debouncing and smoothed frequency.
- `src/moteur_zs_x11d1.cpp`: old controller used by test_moteur only.

Measurements are still published every 500 ms in the text format decoded by
the Pi supervision (except after `imu` > 0); the actuator status is published about
every second (except after `tel` > 0). The actuator task stays independent
of I2C accesses. A logical USB disconnection detected for 250 ms requests a stop,
with no guarantee of detecting a crash of the host application: that is the role
of the `wd` watchdog, off until the host requests it.
The IMU is read by a separate task (25 Hz, or the `imu` rate), outside the display
rhythm; a lock protects the I2C reads shared with the display and the Knob.
The USB lines of the actuator, sensor and IMU tasks go through a queue
and a single writer task: they do not interleave, and a command task
never waits for USB (if the host stops reading, lines are lost;
`etat` shows how many).

Verification without hardware: `python tests/verifier_policy.py`; the real
controller is also run with simulated clock, USB and GPIO (centering, inversion,
simultaneous commands, stop, USB loss and task creation failure), then
with the ROVER_3 commands (watchdog, inversion by `p`, T lines) and the
SC reading (simulated pulses, bounce, sign, 10-field T line).
Builds do not prove the mechanical behavior or autonomous navigation.

## Display and sensors

The sections about the display and sensors apply to the
`esp_c5`, `esp_c5_complet` and `esp_c5_capteurs` environments.

PlatformIO program for the **XIAO ESP32C5** of the [A7 sensor board](../../../hardware/electronics/carte_capteurs/perfboard_une_plaque_c5_a7.md) (in French). It reads the **GY-521** (MPU-6050) and the **BMP280**, and displays them on the **OLED display**. If a **Modulino Knob** is connected, the display becomes the rover's home desktop, browsed with the Knob. Without the Knob, the display continuously shows the sensor readings. All are on the I2C bus, with SDA on D5 (GPIO24) and SCL on D6 (GPIO11).

## Running

```sh
cd software/esp32/esp_c5
pio run -e test_moteur                         # build the motor test
pio run -e test_moteur -t upload --upload-port COM9  # upload the motor test
pio device monitor --port COM9 --baud 115200    # serial monitor
```

To put the main program back:

```sh
pio run -e esp_c5 -t upload --upload-port COM9
```

Close the serial monitor before uploading, otherwise the port is busy. If the upload does not start, hold the XIAO's **BOOT** button while plugging in the USB, then try again.

`COM9` is the port identified on this PC on 17 October; it may change.
In PowerShell, if `pio` is not in the PATH, replace it with
`& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"`.
The `esp_c5_capteurs` environment builds the sensor program
alone, without configuring the motor outputs: `pio run -e esp_c5_capteurs`.

The platform is [pioarduino](https://github.com/pioarduino/platform-espressif32) (Arduino-ESP32 3.3), the only one that supports the ESP32-C5. The program was built with its version 55.03.311.

## Navigating (with the Knob)

| Gesture on the Knob | Effect |
| --- | --- |
| Turn | On the home screen, opens the menu. In the menu, changes app. In an app, moves to the next one |
| Short press | Opens the chosen app, or returns to the menu |
| Long press (0.7 s) | Returns to the home screen |

The menu returns to the home screen on its own after 30 s without a gesture.

## The screens

At the top of each screen, a status bar shows the title, the time since startup and three dots: **I** for the GY-521, **B** for the barometer and **K** for the Knob. A filled dot indicates a component that is present, an empty dot a missing component.

- **Accueil** (home): the robot's eyes. They blink, slide toward the side that tilts and go cross-eyed when the board is shaken. If a component is missing, its name is shown at the bottom.
- **Capteurs** (sensors; the only screen without the Knob): verdict and norm of the acceleration, accelerations (g) and rotations (°/s) of the GY-521; verdict, pressure (hPa) and temperature of the BMP280.
- **Niveau** (level): a spirit level, with roll and pitch in degrees. « A plat » (flat) is shown below 1.5°.
- **Diagnostic**: the addresses present on the I2C bus, rescanned every second, with a verdict for each known component.
- **Infos**: board, MAC address, free memory and build date.

The serial monitor receives the status of the GY-521, the BMP280 and the Knob, and the screen displayed, every 500 ms. Typing **`d`** in the monitor runs a bus diagnosis, at 100 kHz then at 400 kHz: idle level of the SDA and SCL lines, 200 scans with the number of replies per address, and repeated reading of the BMP280 chip ID. The XIAO's LED blinks as long as the program is running.

Verdicts:

| Verdict | Meaning |
| --- | --- |
| `OK` | It replies and its values are plausible: at rest, about 1 g of acceleration; pressure between 800 and 1100 hPa |
| `DOUTEUX` | It replies but the values are abnormal: failed read, sensor shaken or damaged |
| `ABSENT` | No reply at its address |

A missing component is searched for again every 2 s: it can be plugged in without restarting.

## Settings (at the top of `src/main.cpp`)

- **Upside-down display**: `OLED_ROTATION` is 2, because the display is mounted upside down on the board. Set it to 0 if it appears upside down. The eyes and the bubble follow automatically.
- **Menu scrolling the wrong way**: `KNOB_SENS = -1`. If one notch skips several apps, increase `KNOB_PAS`.
- **Eyes or bubble on the wrong axis**: `AXES_ECHANGES = true`. For an inverted axis, `SENS_X = -1` or `SENS_Y = -1`.
- **Bus errors**: the bus runs at 400 kHz so the display is smooth. Set `I2C_HZ` back to 100000 if components disappear.

## If a component is missing

- **Nothing on the bus**: check 3V3, GND and the SDA/SCL wires. Also check that at least one board has its I2C pull-up resistors.
- **Knob missing**: it replies at 0x3A (or 0x3B). Connect its wires according to the silkscreened names, not the position of the pins.
- **BMP280 at 0x77 or GY-521 at 0x69**: they are found automatically. This means SDO or AD0 is pulled to 3V3 on the board.
- **BMP280 changing address (0x76 then 0x77)**: its SDO pin is connected to nothing, neither on the module nor on the A7 board, and it floats. The program's driver tolerates it: it tries both addresses and keeps only readings confirmed twice. The real fix is a wire between SDO and GND, plus a wire between CSB and 3V3.
- **Temperature at -138.6 °C (raw temperature at 0) or pressure jumping by hundreds of hPa**: the bus and the calibration are fine, but the measuring part of the chip is dead. The most likely cause is a 5 V supply: the GY-BMP280-3.3 module has no regulator and does not tolerate more than 4.25 V. Typing `b` in the monitor shows the raw registers.
- **No serial monitor**: the XIAO uses native USB. Wait for the restart to finish, or press RESET.
