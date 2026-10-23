[Français](connexions_gpio.md) · **English**

<a id="connexions-et-budget-gpio--a0"></a>
# GPIO Connections and Budget — A0

**The selected assembly is now C5 on perfboard A1 (working reference not included), with buttons on ADC.** The C5 assignments in this plan are documented by Seeed, not physically tested. This A0 document retains the generic four-button input proposal and its reservations; do not wire its 22 functions simultaneously on the 11 GPIOs of the XIAO.

**No GPIO assignments have been validated yet on the board owned.** The schematic uses U1 as a symbol for functions, without footprint and without real ESP pin numbers. J0 is a proposed adaptation interface; its pad number does not designate a XIAO pin. The complete table, generated from the CAD, is connexions_support.csv (working reference not included).

<a id="connecteur-dadaptation-j0"></a>
## Adaptation Connector J0

J0 is a proposal for a 2 × 12 header with a pitch of 2,54 mm, numbered by rows: 1/2, 3/4, …, 23/24. Pad 1 is square on the PCB. Verify the actual insertion direction and label the harness; no direct connection based on resemblance to a DevKit/XIAO.

| J0 Pad | Function | View from ESP | Real GPIO |
| --- | --- | --- | --- |
| 1 | +3V3 from regulated ESP output, budget to be verified | Power Supply | Not a GPIO |
| 2 | GND | Reference | Not a GPIO |
| 3 / 4 | I2C_SDA / I2C_SCL | Bidirectional bus / clock | TBD |
| 5 / 6 | GNSS_RX / GNSS_TX | GPS TX input / GPS RX output | TBD |
| 7 / 8 / 9 / 10 | BTN_UP / BTN_DOWN / BTN_OK / BTN_BACK | Active-low inputs | TBD |
| 11 | SERVO_PWM | Output, no pulses at startup | TBD |
| 12 | HW_CHAIN_N | Input: low = closed dry contact; high = stop/unavailable | TBD |
| 13 / 14 | ENC1_A / ENC1_B | Inputs, interface to be confirmed | TBD |
| 15 / 16 | ENC2_A / ENC2_B | Inputs, interface to be confirmed | TBD |
| 17 to 20 | CTRL_IO1…4 | Direction/interface not defined | TBD |
| 21 | IMU_INT optional | Input | TBD |
| 22 | GNSS_PPS optional | Input | TBD |
| 23 | BUMP_N optional | Dry contact input | TBD |
| 24 | RESERVED | To be assigned | TBD |

The USB is that of the existing ESP board, connected directly to the Pi. It does not pass through J0 or the support board. Reserve its signals, as well as the boot, flash/PSRAM, LED, and extension pins occupied by the exact board.

<a id="connecteurs-périphériques"></a>
## Peripheral connectors

These wire orders are **support board choices**. Build a crossover harness towards the actual pinout of the module, documented and checked with a multimeter. All modules are initially unplugged; jumpers alone are not sufficient to prevent parasitic power supply from signals.

| Connector | Pads in order | Conditions before plugging in |
| --- | --- | --- |
| J1 IMU | 1 V_IMU, 2 GND, 3 SDA, 4 SCL, 5 IMU_INT_EXT | GY-521 identified; regulator, pull-ups, and power input verified |
| J2 barometer | 1 V_BARO, 2 GND, 3 SDA, 4 SCL | History: BMP280 3,3 V; variant/pins owned to be checked, I2C mode and CS/SDO jumpers if module has 6 pins |
| J3 OLED | 1 V_OLED, 2 GND, 3 SDA, 4 SCL | Model and VCC/GND order known, address noted; 'classic' OLED does not define its voltage or controller |
| J4 extension | 1 V_EXT, 2 GND, 3 SDA, 4 SCL | Option not mounted; remote sensor on short link or with separate adaptation |
| J5 GNSS | 1 V_GNSS, 2 GND, 3 GPS_TX_EXT, 4 GPS_RX_EXT, 5 GPS_PPS_EXT | GPS received, input voltage and UART levels verified. GPS TX to ESP RX; PPS optional |
| J6 servo | 1 SERVO_SIG, 2 GND reference | Positive and servo power return **off-board**. R3 220 Ω and R4 100 kΩ do not convert 3,3 V to 5 V |
| J7 buttons | 1 GND, 2 UP_EXT, 3 DOWN_EXT, 4 OK_EXT, 5 BACK_EXT | Four NO buttons each between its line and GND; beware of pin pairs on a 4 pin button |
| J8/J9 encoders | 1 V_ENCn, 2 GND, 3 A_EXT, 4 B_EXT | Power supplies, output type, frequency, and level to be identified; open links on A0 |
| J10 control | 1 GND, 2…5 CTRL_EXT1…4 | No protocol chosen. No motor, PWM, CAN, or UART connected without an appropriate interface |
| J11 bumper | 1 GND, 2 BUMP_EXT | Dry contact option; R7/R8 components DNP in A0 |
| J12 shutdown status | 1 GND, 2 HW_CHAIN_EXT | Auxiliary dry contact only; never the power contact nor coil voltage |

JP1…5: module power supplies open by default. JP6: optional INT open. JP7…9: GNSS signals open until level verification. JP10…19: encoder/controller extensions open. Bridges are only closed after identification. R1/R2 of 4,7 kΩ remain DNP until the pull-up assessment; other jumpers do not select 5 V voltage.

<a id="bilan-de-gpio"></a>
## GPIO Summary

| Function | Minimum number / variant | Reservation |
| --- | --- | --- |
| Shared I2C IMU/barometer/OLED | 2 | SDA/SCL compatible with the actual board |
| GNSS UART | 2, or 1 in read-only mode if GPS RX is unused | Hardware UART available; independent debug USB |
| Four separate buttons | 4 | Inputs outside sensitive strapping pins |
| Servo signal | 1 | PWM output, reset state verified |
| Hardware stop feedback | 1 | Mandatory input, external pull-up |
| **Interface base + sensors + servo** | **10** (9 if GNSS TX omitted) | No motor control in this sum |
| 2 quadrature encoders A/B | +4 | Conditional; documented telemetry alternative |
| 1 UART TX/RX controller interface | +2 | Budget assumption, not a controller choice |
| 4-line control | +4 instead of +2 | Only if the controller requires it |
| IMU INT / GPS PPS / bumpers | +1 each | Options, can be omitted |
| Reserve | +1 desired | Keep if possible |

With quadrature × 2 and UART control, the target reaches **16 GPIO**, or **20** with the three options and a reserve. The maximum planned on J0 is 22 logical lines. The presence of a connector does not guarantee that the MCU exposes all of them.

The [Seeed XIAO S3](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) documentation distinguishes standard, Sense, and Plus variants; the standard version announces 11 GPIO on the main interface. **The 16 GPIO scenario does not fit directly on these pins alone.** If this is the board owned, prioritize the controller's telemetry, omit INT/PPS/options, and prepare an extension of buttons on I2C or a resistive ADC reading. These adaptations are not in schematic A0 and require a revision; no expander purchase is decided. Reuse another ESP board already owned if its pinout provides the required margin. Two available ESPs also allow for future separation, but increase firmware and wiring for this week.

The [Seeed C5](https://wiki.seeedstudio.com/xiao_esp32c5_getting_started/) documentation should only be used after confirmation of this variant. The USB/strapping/memory pins of an S3 and a C5 are not interchangeable. The [GPIO S3 Espressif](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html) documentation is used for reservation control, not to deduce a board's footprint.

<a id="valeurs-proposées-et-mesures"></a>
## Proposed values and measurements

- Buttons: 10 kΩ pull-up to +3V3, 330 Ω in series on the panel side, 100 nF on the GPIO side to GND. Closed, approximately 0,11 V to 3,3 V depending on nominal values; RC rise time of the order of 1 ms. Software debouncing proposed at 20 ms, to be adjusted by measurement; these figures are calculations, not tests.
- Stop return: 10 kΩ pull-up, 1 kΩ in series, 100 nF; closed, approximately 0,30 V nominal. Verify the VIH/VIL margin of the actual board and detection time. Software filtering must not mask a prolonged opening.
- I2C: start at 100 kHz, one module at a time. Record the pull-ups at **3,3 V**, their parallel resistance, and the edges. The relationship `R_eq = 1 / Σ(1/R_i)` shows why not to systematically add 4,7 kΩ. Limits depend on chips/capacitance; see [NXP UM10204](https://www.nxp.com/docs/en/user-guide/UM10204.pdf). A guessed address is not a tested address.

In the identification folder, mark for each line **proposed**, **verified on the exact board documentation**, then **physically tested**, with date and result. No 5 V signal goes directly to an ESP GPIO.
