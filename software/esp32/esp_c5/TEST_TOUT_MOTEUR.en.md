[Français](TEST_TOUT_MOTEUR.md) · **English**

<a id="test-manuel--traction-et-servo"></a>
# Manual test: traction and servo

Source: `src/test_tout_moteur.cpp`. PlatformIO environment **test_tout_moteur** (test without sensors; the default program is esp_c5). ESP32Servo 3.2.1 is retained for the servo. The main program, motor test, and servo test remain available in their respective environments. This program does not read any sensors.

<a id="connexions"></a>
## Connections

Cut power supplies before moving wires. **The servo moves from D0 (previous isolated test) to D7 in this program. D0 returns to the motor driver 1.**

| XIAO ESP32-C5 Pin | Destination | Color indicated by Hugo |
| --- | --- | --- |
| D7 / GPIO12 | DT996 servo signal | According to servo harness |
| D0 / GPIO1 | Motor driver speed input 1, 0-5 V | Red |
| D9 / GPIO9 | Motor driver speed input 2, 0-5 V | Red |
| D1 / GPIO0 | D/R for driver speed D0 only | Yellow |
| D4 / GPIO23 | D/R for driver speed D9 only | Yellow |
| D10 / GPIO10 | BRAKE for both drivers | Green |
| GND | GND of drivers and servo power supply reference | Orange on driver side |

Motor speeds and directions are now independent: g and d, sensg and sensd. m keeps the percentage command for both motors. The brake remains common on D10. Each wheel has its own forward/reverse adjustment, inverseg/inversed 0 or 1, stored in flash. Left is preserved and right is reversed according to Hugo's latest feedback: defaults left=1, right=0. The historical right key inv_d is ignored; new corrections use inv_d_v2. The left adjustment inv_g is still preserved; newly memorized corrections take priority on subsequent startups. **Separate the two yellow wires: driver D0 on D1, driver D9 on D4.** Do not modify the phases or Hall sensors for this fix. D7 is used by the servo and cannot be used by a GPS simultaneously. D4 is available because the current I2C bus remains D5/D6. The main esp_c5 program now uses this same wiring and the same actuators controller actionneurs.cpp, with the sensors added. If only one driver is connected, use D0 for it; the D9 output remains available for the second. The speed interface of each driver must be configured as in the working motor test. The phases and Hall sensors remain connected to their respective driver.

The servo keeps its external power supply adapter (connected to 6 V as planned); its current return goes directly to this power supply. Connect its GND reference to the ESP's GND. The drivers keep the traction power supply. Do not connect their 5 V outputs to each other or to the ESP's USB power supply.

<a id="televerser-et-utiliser"></a>
## Upload and use

Place the traction wheels free of the ground before testing. PlatformIO > Project Tasks > **test_tout_moteur > General > Upload**. Open the serial monitor at **115200 bauds**, commands followed by Enter. No upload was performed by Codex.

| Command | Effect |
| --- | --- |
| m 10 | 10 % PWM to both drivers, maintained until m 0, stop, or USB loss detected |
| m 0 | Zero traction command; servo position maintained |
| g 10 / d 20 | Left/right percentage, without modifying the other wheel |
| sensg 0/1 / sensd 0/1 | Direction per wheel, after its command is set to zero and physical stop |
| inverseg 0/1 / inversed 0/1 | Correct front per wheel; motors at zero; memorized |
| s -30 | Move to -30 relative to center, then maintain the requested position |
| s 0 or centre | Move to the servo center |
| s 30 | Move to +30; traction speed remains the same |
| v 30 | Limit servo setpoint speed to 30 approximate degrees per second |
| stop | Traction PWM zero and servo signal off; power supplies remain present |
| etat or diag | Setpoints, DIR/BRAKE levels, requested position, PWM frequencies |
| aide | List of commands |
| sens 0 / sens 1 | D/R level; only after m 0 and physical motor stop |
| frein 0 / frein 1 | PWM zero then common BRAKE level; a new m command is required |
| course 140 | Declare a total stroke actually measured as 140 degrees |

The aliases `moteur`, `servo` and `vitesse_servo` are also accepted. A number alone is rejected to avoid confusion between motor speed and servo angle. An invalid command cuts the commands for both actuators.

Example, one line at a time:

```text
v 30
s 0
m 5
s -20
s 20
m 0
stop
```

At startup, traction is zero and **the servo requests center (1500 us)**. This behavior was restored at Hugo's request, once the servo was disengaged from the mechanism. The main and test use the same actuators controller actuators.cpp.

<a id="regler-le-milieu-mecanique"></a>
### Set mechanical center

1. Disconnect the servo power supply and disconnect the linkage or remove the horn (the arm screwed onto the shaft). The shaft must be able to move without driving the rover.
2. Power the servo with its appropriate power supply and common ground, signal on D7. Send `centre` or click on Recenter to 0 in the browser. This takes 1500 us. The first movement may be fast: the ESP does not measure the physical position and cannot guarantee a gradual first trajectory.
3. Once the servo is stabilized, note the position of the shaft. Disconnect its power supply, then reattach the horn to align the straight-ahead direction without turning the shaft.
4. Re-power, request `centre`, then try `s -10`, `centre`, `s 10`. Keep small deflections until the mechanics are adjusted.

1500 us is our command zero; it is not a measurement of the exact middle of the limits. If a fixed setpoint causes the shaft to rotate continuously instead of stabilizing at a position, stop the test: you need to identify the servo type. A continuous rotation servo commands a speed around the neutral point, not an angle. See [the Pololu manufacturer's explanation](https://www.pololu.com/blog/24/continuous-rotation-servos-and-multi-turn-servos).

The servo range remains **800..2200 us**, already tested by Hugo; this program does not widen it. The scale **-90..+90 is indicative**, 0 is the middle. The physical position of the servo and the revolutions per minute of the motors are not measured. `m 10` is a PWM command percentage, not a measured speed. `v` affects the speed of the servo setpoint movement, not its position or torque. Acceleration and deceleration are gradual.

A total travel range should only be entered after measurement. For example, `course 140` would give a range of -70..+70 with 0 in the center; 140 is not a measurement of the DT996. This setting is lost on restart.

`stop` does not control the STOP pin of the driver and does not cut the battery. A zero PWM command does not guarantee physical immobilization of the assembly. A cut servo signal does not guarantee maintenance of the mechanical direction.

<a id="implementation-et-verification"></a>
## Implementation and verification

The servo uses ESP32Servo at 50 Hz, resolution 14 bits, with its allocation limited to LEDC channels 0/1. The two motor outputs explicitly use channels 4/5 at 5 kHz, resolution 10 bits. This separation prevents servo and motor commands from reusing the same channel. Frequencies are read during operation; an error cuts the outputs and locks a fault.

The **TOUT_MOTEUR_2** version reads the timers directly via the LEDC driver, independently of the duty. In Arduino-ESP32 3.3.11, ledcReadFreq() may return zero before the first pulse or when the duty is zero; the old version treated this zero as a failure. The current verification tolerates 1 % or 1 Hz rounding but continues to reject an absent timer or incorrect frequency. At startup, state displays approximately 50 Hz for D7 and 5000 Hz for D0/D9 even at 0 % traction. A detached output remains displayed at 0 Hz.

In case of a fault, state and rejected commands display `Cause=...`, the pin and frequency/duty captured BEFORE cutoff. The first pattern is preserved: consecutive zeros upon detachment do not replace the cause. The current version **TOUT_MOTEUR_4** keeps this fix and moves the servo to D7/GPIO12. It separates D/R on D1/D4 and allows adjusting each wheel independently. The pins and inversions are in src/test_tout_moteur_broches.h. Moving the wires described above is necessary for this version.

Since the actuators controller is shared with the main program, this program keeps the identifier TOUT_MOTEUR_4 but also accepts ROVER_3 commands `p`, `hb`, `wd`, `inv` and `tel` (see the [Raspberry guide](CONNEXION_RASPBERRY.en.md#extension-rover_3)). `imu` and `lacet0` respond that the central unit is absent from this program. Without these commands, its behavior is that of TOUT_MOTEUR_4; its USB lines now go through a queue, without waiting for USB in the control loop.

Servo progression is non-blocking and updated every 20 ms; the console and traction remain processed during movement. A USB loss detected for at least 250 ms cuts off commands, with no automatic recovery. This is not a guarantee of detecting serial monitor closure.

PC tests `python tests/verifier_policy.py` check commands, traction/servo independence, interruption, speed change without setpoint jump, limits, and millis overflow. They do not replace the physical test of both actuators together.

PC validation of 18 October 2026: `test_tout_moteur` compilation successful, 91 combined checks and 22 successful PWM read regressions; existing controls remain successful (213 / 292 / 310 / 28 / 23). No serial port opened for this preparation.
