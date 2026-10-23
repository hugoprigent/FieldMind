[Français](TEST_SERVO.md) · **English**

<a id="test-servo-simple-avec-esp32servo"></a>
# Simple servo test with ESP32Servo

PlatformIO environment **test_servo**, to be selected explicitly. For the traction and servo together, choose **test_tout_moteur** (default), with the servo on D8: see [TEST_TOUT_MOTEUR.md](TEST_TOUT_MOTEUR.en.md). Source: `src/test_servo.cpp`. Library: ESP32Servo **3.2.1**. It provides the Arduino syntax `Servo`, `attach()` and `write()` on ESP32-C5: https://github.com/madhephaestus/ESP32Servo/tree/3.2.1

<a id="identification-du-servo"></a>
## Servo identification

Hugo reads DT996 and an inscription corresponding to DIYMORE-TOOL on the casing. The purchase was referenced under Heemol; this trade name is not sufficient to identify the control range. A [Chalmers project from 2025](https://odr.chalmers.se/bitstreams/ee50e6f6-9bc7-4191-8657-cada9aee898c/download) uses exactly the DIYMORE-TOOL DT996 for a range of 180 degrees (p. 7). The authors there calibrate the PWM limits progressively (p. 20) and power the servos at 6 V. This is a lesson learned, not a manufacturer's datasheet. No official DT996 datasheet confirming extreme pulses, stall current or maximum voltage has been found on 18 October 2026. Do not use the characteristics of the MG996R for this DT996.

<a id="branchement"></a>
## Wiring

Cut power supplies before wiring. Disconnect power from the motor drivers and especially their D0 link: this pin is used for the servo for this test.

| Source | Destination |
| --- | --- |
| XIAO D0 / GPIO1 (not GPIO0) | Servo signal |
| External power supply adapted, wired to 6 V intended for the DT996 | + of the servo |
| - of this power supply | - of the servo and ESP GND |

The servo must not be powered by the ESP's 3V3 nor by the traction battery directly. The servo current returns directly to its power supply. The rover's general plan keeps D8 for the servo; only this test uses D0.

<a id="utilisation--angles-et-recherche-de-lamplitude"></a>
## Usage: angles and range search

The servo requests the middle (1500 us) at startup, then waits for a command. Release the joystick before uploading. Open the monitor at 115200 bauds. Each command must be followed by Enter.

| Command | Action |
| --- | --- |
| -45, 0, 45, or angle -45 | Progressively reach this angle relative to the middle then hold |
| range 140 | Declare a total range ACTUALLY measured at 140 degrees, without movement |
| status | Display the pulse range and whether the angle scale has been set |
| test | Try the current range: middle, low side, high side, then middle |
| plus | Widen by 100 us on each side, then try this new range |
| moins | Reduce by 100 us on each side, then try |
| centre | Request the center at 1500 us |
| stop | Detach the servo and set the signal low, including during movement |
| aide | Review the commands |

The initial range is now 800..2200 us, confirmed usable by Hugo. The plus commands then give 700..2300, 600..2400 and finally 500..2500 us. Each travel takes about 3 seconds with gradual acceleration and deceleration. The displayed values are setpoints, not angle measurements. The ceiling 500..2500 us is a software test limit, not a mechanical range validated for the DT996. No effort or end-stop detection exists.

By default, the angles -90..+90 are **indicative**: -90 corresponds to 800 us, 0 to the center at 1500 us and +90 to 2200 us. The conversion is linear between the ends of the current range; this does not prove a travel of 180 degrees. The origin 0 is the center controlled by 1500 us, not a measured position. Negative and positive angles command both sides; the physical direction depends on the mounting. -180..+180 would make 360 degrees in total, beyond the announced travel of 180 degrees for this model. With the initial scale, these commands are rejected without movement; they are not silently clamped to the limits.

To convert to approximate physical degrees, measure the total angle between the extremes. For example, if the range covers ACTUALLY 140 degrees, enter `course 140` : the commands become -70..+70, the center remains 0 and `45` requests about 45 degrees from the center. This number is an example, not a measurement of the DT996. The servo does not return any real position. Decimals with a dot are accepted, for example -22.5. Changing the range with plus or minus invalidates this calibration and resets the indicative scale -90..+90. Restarting also resets this indicative scale; no travel adjustment is saved in permanent memory.

The estimate of about 120 degrees given then withdrawn by Hugo is not a measurement and is not used to calibrate the code.

**If the servo forces, buzzes or no longer gains travel: stop immediately, and cut its power supply if necessary. Do not widen further.** Note the last range that worked without effort. A new movement command is required after stop; it does not resume automatically. The cut signal does not cut the servo's power supply.

Hugo confirmed operation with ESP32Servo and about 90 degrees real with the old range 1000..2000 us. The new test allows searching for usable extremes, without announcing uncalibrated physical angles.

<a id="tester-sans-esp"></a>
## Test without ESP

An RC servo tester or a compatible RC radio receiver can generate the pulses instead of the ESP. Power alone does not control a position. See https://www.pololu.com/blog/16/electrical-characteristics-of-servos-and-introduction-to-the-servo-control-interface

## Verification

PlatformIO compilation test_servo verified on 18 October 2026. Relative angle conversions and out-of-range rejections are verified on PC by tests/servo_angles_test.cpp, launched with python tests/verifier_policy.py. No upload or physical movement triggered from the PC. The old files servo_test_reglages.h and tests/servo_test.cpp concern the old PWM test and are no longer used by this simplified program.
