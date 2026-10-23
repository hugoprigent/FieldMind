[Français](README.md) · **English**

<a id="fieldmind--schéma-électrique-complet-du-rover-prototype"></a>
# FieldMind — complete electrical schematic of the prototype rover

Wiring diagram for the entire rover, complementing the A7 sensor board. It is in **KiCad format 10**, without routing or PCB, and has been checked by the ERC (0 error, 0 warning).

[Connections file](../CONNEXIONS.en.md) · [KiCad project](cad/fieldmind_rover_systeme.kicad_pro) · [PDF of 4 sheets](exports/schema_rover_complet.pdf) · [Bill of materials](exports/nomenclature.csv) · [A7 sensor board](../carte_capteurs/perfboard_une_plaque_c5_a7.en.md)

| Sheet | Content |
| --- | --- |
| 1. Overview | The three blocks, the labels connecting the sheets, what remains to be documented |
| 2. Power and shutdown | Battery, fuses, main switch, 3 XL4015, emergency stop, relay, contactor, E600 controller and motors, servo |
| 3. Raspberry Pi and USB | USB-C power supply for the Pi, 2 webcams, C1 lidar and its adapter, USB link to the XIAO |
| 4. Sensor board and peripherals | A7 board connectors: GPS on J4, Knob on J7, servo and shutdown status on J6, J8 reserved |

The symbols come from the standard KiCad libraries: `Device:Battery`, `Device:Fuse`, `Switch:SW_SPST`, `SW_Push_Open` (normally closed emergency stop), `SW_Push`, `Relay:FINDER-40.52` and `FINDER-40.11`, `Motor:Motor_DC`, `Motor:Motor_Servo`, `Connector:USB_C_Plug_USB2.0`, as well as `power:GND` and `power:PWR_FLAG`. Modules missing from the libraries are drawn as blocks in `FieldMind_Rover.kicad_sym`: Pi 4 viewed via its ports, A7 board, XL4015, webcams, lidar, adapter, GPS, Knob and E600 controller.

<a id="câblage-liaison-par-liaison"></a>
## Wiring, link by link

**Power**

| From | To | Note |
| --- | --- | --- |
| BT1 + | F0 → S1 → bus | Main fuse as close to the battery as possible |
| Bus | F1 → XL4015 no.1 IN+ | Output set to **5,1 V** → Pi USB-C cable (VBUS and GND only) |
| Bus | F2 → XL4015 no.2 IN+ | Output set to **6,0 V** → servo + ; servo − directly to OUT− |
| Bus | F4 → XL4015 no.3 IN+ | Output set to **12 V** → shutdown chain (K1 and K2 coils) |
| Bus | F3 → K2 contact 11 → 14 | Traction power → V+ of the E600 controller |
| All XL4015 IN− and OUT−, BT1 −, controller GND | Ground bar | Each power return has its own wire |

**Emergency stop** : +12 V → S2 (stop, NC) → S3 (reset, NO) → coils K1 and K2 (A1), then A2 to GND.
- K1 11-14, wired in parallel on S3, provides latching: pressing S3 engages K1 and K2, pressing S2 releases everything.
- Releasing S2 does not restart anything: you must reset with S3.
- K1 21-24 is a dry contact of state: 21 goes to GND, 24 to **J6.5 (D9)**. Closed, it indicates that the chain is available.

**Pi and USB**

| Pi Port | Device |
| --- | --- |
| USB-C | Power supply 5,1 V (XL4015 #1) |
| USB 3.0 #1 and #2 (blue) | Left webcam, right webcam |
| USB 2.0 #1 | Lidar C1 adapter. Lidar harness: red 5V, yellow TX → RX, green RX ← TX, black GND |
| USB 2.0 #2 | XIAO USB-C (data and power for the sensor board) |

**Sensor board A7**

| Connector | Plugged into |
| --- | --- |
| J4.1 RX_GPS (D6) | GPS RXD |
| J4.2 GND, J4.3 3V3 | GPS GND and VCC |
| J4.4 TX_GPS (D7) | GPS TXD (PPS not used) |
| J7.1 to J7.4 | GND, 3.3V, SDA, SCL of the Knob Modulino, in that order |
| J6.6 D8 | Signal (PWM) of the servo, with J6.2 GND as reference |
| J6.5 D9 | K1 contact 24 (state of the stop chain) |
| J6.1 5V, J6.3 3V3, J6.4 D10 | Free |
| J8.1 to J8.4 (D0 to D3) | Reserved for wheel feedback or controller command, after removing the E600 |

<a id="ce-qui-reste-à-identifier"></a>
## What remains to be identified

- **E600 Donor** : battery (chemistry, maximum charged voltage, BMS, charger), traction controller (command, feedback), motors (DC or brushless).
  - The controller's command (CMD) and feedback (RET) signals are left unconnected until they are known.
  - Pre-charge must be provided if the controller has large input capacitors.
- **Pack** : it must remain **≤ 36 V when charged**, the input limit of the XL4015. The fuses and fuse holders must withstand the pack's maximum voltage.
- **Ratings** of fuses F0 to F4, contactor K2, and cables: to be determined based on measured currents.
- **XL4015 n°3 and 12 V relays** are a proposal made with the lot's hardware. Relay K1 (Finder 40.52, coil 12 V) and contactor K2 are **not purchased**.
- Signal 3,3 V recognized by the servo, pinout of the received GPS, Pi USB summary (1,2 A total; powered hub otherwise).

**Status: schematic verified in KiCad (ERC), not physically tested.** Regeneration: `python3 scripts/generer_systeme.py --force`, which overwrites any manual edits.
