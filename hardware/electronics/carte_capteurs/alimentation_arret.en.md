[Français](alimentation_arret.md) · **English**

<a id="alimentation-et-arrêt--a0"></a>
# Power supply and shutdown — A0

The small board distributes only the 3,3 V for the small modules and the signals. Battery, fuses, conversions, traction cutoff, and servo power supply are off-board. The functional diagram illustrates these branches; this document sets the connection conditions.

<a id="montage-sur-table"></a>
## Bench setup

1. Pi: power supply 5 V suitable for the Pi 4 and its peripherals, appropriate cable. ESP: data USB from the Pi. Do not add a power supply to the MCU 5 V/3,3 V until its schematic has been reviewed.
2. The 3,3 V exported by the ESP can only power the identified compatible modules and within the documented and then measured current reserve of the regulator. OLED, IMU, barometer, and GNSS consumptions are **to be recorded**; no sum has been validated yet.
3. Servo: external regulated source set within the confirmed range upon receipt, target 6 V. Source disconnected during connection. Power V+ and GND go directly to the servo; PWM signal and reference GND come from J6.
4. The positive rails 5 V Pi, 3,3 V logic, and 6 V servo remain separate. Identify the existing GND link via USB, Pi conversion, and other branches before adding ground straps. Never intentionally interrupt a signal ground without suitable isolation architecture.

<a id="chemins-des-courants"></a>
## Current paths

```text
BAT+ -> F0 -> sectionneur -> F_PI -> buck Pi -> câble Pi
                         -> F_SERVO -> buck servo -> SERVO V+
                         -> F_MOTEUR -> coupure DC -> contrôleur -> moteur
BAT- <- retours dédiés des bucks et du contrôleur

buck servo GND <------ retour de puissance servo (direct, hors petite carte)
              <------ référence signal GND J6 (liaison dédiée, courant limité)
Pi USB GND ------------ ESP GND / carte support / référence des interfaces
```

This routing is a wiring intention. If the buck converters are non-isolated, their negative inputs/outputs and USB already create multiple GND links: determine the actual topology, avoid large loops, and keep servo/motor returns away from sensor traces. Measure the ESP–servo GND difference under activity and 3,3 V stability. A traction cutoff must leave Pi/ESP diagnostics alive if the chosen distribution allows it.

<a id="batterie-et-conversions--critères-avant-choix"></a>
## Battery and conversions: criteria before selection

The pack is not selected. Record chemistry, configuration, nominal voltage, maximum charged voltage, BMS, charger, and status. Compare **all** of the actual range to the converters with margin for transients and to DC equipment.

The [XL4015 AZDelivery](https://www.az-delivery.de/products/spannungswandler-5a-8-36v-zu-1-25-32v) module announces 8–36 V input. A 10S Li-ion pack of 36 V nominal can reach 42 V: **do not connect this pack directly to the XL4015**. The announced 5 A do not constitute a validated continuous current in the enclosure. The Pi 5 V is not a suitable input for this module for the servo branch. Set each buck with a compatible source, verify no-load then under load before connecting Pi or servo.

Fill the table with measurements and datasheets; do not use simulation figures.

| Branch | Voltage | Nominal / peak current | Cable / length | DC fuse / curve / breaking capacity | Status |
| --- | --- | --- | --- | --- | --- |
| Main F0 | Pack: TBD | TBD | TBD | TBD | To size |
| Pi + USB | 5 V target | TBD, Pi balance + 2 webcams + lidar + ESP | TBD | TBD | To measure |
| Servo DT996 | 6 V target, range to confirm | TBD, start/steering effort | TBD | TBD | Ordered, not tested |
| Traction | TBD | TBD, start/block/controller | TBD | TBD | Not selected |
| Small sensors | 3,3 V logic, module power supply to be confirmed | TBD | Short harnesses | Protection suitable for the regulator/source | To be identified |

The fuse protects the cable and the branch. Its current, curve, DC voltage, and breaking capacity depend on the source and wiring; the ratings are not fixed. A fuse/fuse holder rated only for 32 V is not suitable for a branch that can reach 42 V.

The Pi 4 provides a maximum of **1,2 A total on USB** according to the [Raspberry Pi documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#maximum-power-output). Measure the four peripherals together; if a powered hub is used, verify that it does not back-feed the Pi via USB. Also size the 5 V Pi source and the cable voltage drop.

<a id="arrêt-matériel-et-logiciel"></a>
## Hardware and software shutdown

Local shutdown acts via a wired chain independent of the ESP, Pi, and network. After choosing the controller, define a DC element that removes traction permission or cuts its power supply according to the function actually documented. A small menu button or any relay is not this element. Releasing the shutdown must not cause a restart; provide manual reset of the chain and software re-arming.

J12 reads only the **auxiliary dry contact**: closure to GND = chain available; open, unplugged cable, or ESP restart = software state not authorized. Do not inject the pack or coil of a contactor into this input. A single input does not diagnose all short circuits: the actual disconnection remains in the external chain. The software cannot short-circuit this chain.

Power removal does not guarantee braking: freewheeling, controlled braking, residual power, and servo behavior depend on the actuators. Measure behavior with wheels lifted, then on the ground at low speed and in a free area before defining a demonstrable speed. As long as the motor and controller are unknown, no output state can be declared as "guaranteed stop".

The servo output is low at startup, without PWM. After a fault, remove traction and then apply the validated servo behavior (maintain last command, remove pulses, or other). Do not impose a return to center during rolling. The series signal resistor is not a level converter: if 3,3 V is not recognized by the DT996, adapt the interface before testing.
