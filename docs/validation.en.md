[Français](validation.md) · **English**

# Status and validation — 23 October 2026

| Item | Available evidence | Limit |
| --- | --- | --- |
| Physical rover | The rover drives; autonomous navigation tested indoors on 21 October with the GX10 ("explore la pièce" / "explore the room", round trip on the lidar map) | No outdoor mission validated; no success-rate measurement |
| Rover software | 756 Python tests passed in a fresh venv (Python 3.13), 2 long scenarios deselected; 22 web tests | Drivers and models replaced by test simulators; not a GX10 measurement |
| GX10 inference | Eight models served on the GB10; scene analyses of about 6 to 9 s during the real tests | No mission benchmark; remote GX10 reached over a private network |
| Trailer | Built; flaps driven by servos (ROVER_4 firmware flashed) | Autonomous towing not validated; no flap position feedback |
| Drone | Video stream received on the Pi, battery received; complete mission in simulation | No autonomous flight validated; charge level not measured reliably |
| CAD | Parts, drawings and geometric checks of the project | Some components are still approximated; shell not fitted; SG90 mount not validated under load |

Details: [status on 23 October](status-2026-10-23.en.md).

## Reproducing the tests of the current code

After the installation described in the [README](../README.en.md), from `software/rover`:

```bash
.venv/bin/python -m pytest -q -m "not lent"
node --test tests/web/*.test.mjs
```

The AI function tests use fake servers, except for an explicit check via `deploiement/gx10/verifier.py --profil gx10`. The checker contacts real services; no fake-server result should be presented as a local inference measurement.

The configuration of the complementary models on the GX10 is being integrated. The CAD and code parameters are initial values, to be calibrated on the real assembly. The presence of a feature in the software does not mean it has been validated in the field.

## Check of the 23 October edition

Fresh installation in a Python 3.13 venv (uv) from `software/rover/requirements-ci.txt`:
**756 rover tests** passed (`-m "not lent"`, 2 deselected), **22 rover web tests**, Ruff 0.16.9
with no errors. The ESP32-C5 firmware compiles (`pio run -e esp_c5`). The public simulation
server (`python -m fieldmind.public_demo`, profile `aucun`) answers HTTP 200 on `/`,
`/js/main.js` and `/api/sante`. Local Markdown links point to published files.
The first MuJoCo demo has not changed since the 21 October edition and was not rerun
locally; it is still tested by the `demo` job of GitHub Actions. Summary:
[`checks-2026-10-23.json`](checks-2026-10-23.json). These checks were run locally on the Mac; they are not
GX10 benchmarks.

## Check of the 21 October edition

A fresh installation in the publication copy also passed **425 rover tests**, **22 rover web tests**, **98 first-demo tests** and **3 demo web tests**. Ruff passes on the rover; its simulator serves the interface, state and camera image with HTTP 200. The commands and limits are in [`checks-2026-10-21.json`](checks-2026-10-21.json). These checks were run locally on the Mac; they are not GX10 benchmarks.

The **ESP32-C5** firmware also compiles in this copy with PlatformIO 6.2.0 (`pio run -e esp_c5`), without uploading to the hardware.

## Check on GitHub Actions

23 October edition: the **`rover`**, **`demo`** and **`firmware`** jobs of
[run 39937600059](https://github.com/hugoprigent/FieldMind/actions/runs/39937600059) completed
successfully for
[commit `22076d5`](https://github.com/hugoprigent/FieldMind/commit/22076d5d3b8932f53c0528f37da80ed26bfc1338).
As for the previous edition, this run checks the code and the build under Linux; it is
not a test of the physical rover.

21 October edition:

The three **`rover`**, **`demo`** and **`firmware`** jobs of
[GitHub Actions run 39510932511](https://github.com/hugoprigent/FieldMind/actions/runs/39510932511)
completed successfully for
[commit `c1562b3`](https://github.com/hugoprigent/FieldMind/commit/c1562b35b958a626d3878688a7d8265450dd447f).
This run checks the code and the build under Linux; it is not a test of the
physical rover. Later documentation touch-ups do not change the tested code.
