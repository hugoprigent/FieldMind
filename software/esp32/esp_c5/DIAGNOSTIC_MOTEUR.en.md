[Français](DIAGNOSTIC_MOTEUR.md) · **English**

<a id="le-moteur-tourne-au-potentiomètre-mais-ignore-lesp"></a>
# The motor runs with the potentiometer but ignores the ESP

Keep the phase/Hall combination that currently allows rotation. The first test isolates the ESP: the motor and driver are not involved.

<a id="1-vérifier-la-sortie-électrique-de-lesp"></a>
## 1. Check the ESP's electrical output

1. Disconnect the battery, then the USB.
2. Disconnect the **four wires between the ESP and the driver**: D0, D1, D10, and GND. Keep the battery disconnected during this entire measurement.
3. Upload the PlatformIO environment **`test_moteur`**, then open the monitor at 115200 bauds. Only the ESP is powered via USB.
4. Set the multimeter to **DC voltage V⎓**, with probes in COM and V. Place the black probe on **ESP GND** and the red probe on **D0 / GPIO1**. Do not confuse D0 with D1 / GPIO0.
5. Send each command below using Enter and wait one second. The values remain applied until `stop` or `0`.

| Command | Expected voltage between D0 and GND |
| --- | --- |
| `0` | Approximately 0 V |
| `100` | Approximately 3,3 V, constant high level |
| `50` | Often around 1,65 V in continuous measurement, average of the PWM |
| `stop` | Return to approximately 0 V |

The measurement at 50 % depends on the multimeter: it does not validate the pulse shape on its own. The constant levels `0` and `100` are prioritized. **Never perform this sequence at 100 % with the motor powered and connected to the control.** Finish with `stop`, then disconnect the USB before any rewiring.

<a id="2-lire-le-diagnostic-logiciel"></a>
## 2. Read the software diagnostic

During the isolated measurement, after `100`, enter **`diag`** before `stop`. The command displays:

- the firmware name and compilation date;
- the requested PWM and the duty cycle read back from the PWM peripheral;
- the read frequency, expected to be close to 5000 Hz when the PWM is non-zero;
- the number of commands cancelled due to USB loss detection;
- the software reason for the current state and the motor task activity.

At 0 %, the Arduino API returns a frequency of 0 Hz even if the generator is attached: this result alone is not a failure. The register at 100 % should indicate 100 %, and the PWM remains at 100 % until another command. Reading the registers does not replace measurement on D0.

| Result | Where to look next |
| --- | --- |
| `diag` is unknown | Wrong environment or old firmware uploaded |
| PWM cancelled and USB counter increased | USB link or software-side USB detection; keep the diagnostic before restarting |
| Brake requested or fault displayed | Software reason indicated by `etat` / `diag` |
| Register at 100 %, but D0 does not rise to 3,3 V, ESP isolated | Pin measured, GPIO/PWM configuration or ESP board; the Hall sensors and driver are out of circuit |
| D0 goes well from 0 to about 3,3 V | The ESP output responds; then check the electrical interface with the driver |

<a id="3-vérifier-linterface-du-driver-une-fois-létape-1-réussie"></a>
## 3. Check the driver interface, once step 1 is successful

A correctly placed wire does not guarantee that the signal is suitable for the input. Identify the exact label of the input: `VR / 0–5V` or `P / PWM`, as well as the selection jumpers present on the received revision. Keep the potentiometer at minimum, stop position already observed.

A documented implementation of the ZS-X11D1 distinguishes the analog input and the PWM input `P`. For the latter, it indicates pulses from 2,5 to 5 V, and a J1 jumper to close on **this revision**. This configuration is not an instruction to jumper a different board without identifying it. See the [illustrated description, pages 20–22](https://thesis.unipd.it/retrieve/dfe158de-ec8a-41d3-a9b9-6854970fbf38/Gervasio_Danilo.pdf).

The current code generates 0/3,3 V pulses at 5 kHz. It does not generate an analog voltage covering 0–5 V. Depending on the identified input, the correction will be selecting the appropriate PWM input, adapting levels, or converting PWM to an analog voltage. This choice requires the actual board revision; increasing the percentage does not fix an incompatible signal.

Hardware shutdown and phasing are not validated by these measurements alone.
