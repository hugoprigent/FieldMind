[Français](DEUX_MOTEURS.md) · **English**

<a id="main--deux-moteurs-servo-et-capteurs"></a>
# Main: two motors, servo and sensors

The current program is **esp_c5 / ROVER_3** (ROVER_2 commands preserved), selected by default. See [the Raspberry guide and complete connections](CONNEXION_RASPBERRY.en.md). It uses the same actuator controller actionneurs.cpp as test_tout_moteur, in addition to the sensors and screen.

| ESP32-C5 | Destination | Color |
| --- | --- | --- |
| D0 / GPIO1 | Left speed | Red |
| D1 / GPIO0 | F/R left, reversed in software | Yellow |
| D9 / GPIO9 | Right speed | Red |
| D4 / GPIO23 | F/R right | Yellow |
| D10 / GPIO10 | Common BRAKE | Green |
| GND | Common driver reference | Orange |
| D7 / GPIO12 | Servo signal | According to harness |

The two F/R are no longer connected together. Do not modify the phasing/Hall for this fix. The sensor bus remains SDA D5 and SCL D6.

Commands: m 10, m 0, s -10, center, v 30, direction 0/1, brake 0/1, stop, status. Independent commands: g 10 / d 20, g 0 / d 0, directiong 0/1 and directiond 0/1. Correct the front with inverseg 0/1 or inversed 0/1, two wheels at zero: setting memorized. The right is reversed relative to ROVER_1, left preserved. The brake remains common. Individual directions are not imposed by m. At startup, servo at center and traction at zero. Driver STOP remains unpiloted. Physical setpoints are not measured.

The proposed JSON protocol, GPS and autonomous navigation are not added by this integration. Use the Pi supervision USB console for these commands.
