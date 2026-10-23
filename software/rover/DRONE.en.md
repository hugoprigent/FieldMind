[Français](DRONE.md) · **English**

<a id="drone--reconnaissance-et-statut-du-pilotage"></a>
# Drone: reconnaissance and piloting status

Public status as of 23 October 2026. The full mission exists in the [simulator](DRONE_MISSION.en.md), but no real flight has been validated.

<a id="ce-qui-a-été-observé"></a>
## What has been observed

- The Raspberry Pi receives images from a small dual-camera drone via BLE. The video stream and image capture have been verified on the hardware; the images measure 240 × 320 pixels.
- The software can record six views, produce a montage and a report, then detect a cross in the image to assess its centering. This recognition is done without commanding the motors.
- The repository contains an experimental frame encoder, established by offline analysis of version 1.1.0 of the original application, and tests with simulated vectors. This does not prove that the drone accepts frames, nor that the axes, flapping, or stopping work. The RC transport is not connected and no flight commands are activated in the driver.
- The battery level is not reliably known. The rover's IMU measurements do not provide the drone's altitude or heading.

Radio addresses, the third-party application, its resources, and access notes are not published in this repository. The actual driver remains blocked until reproducible bench tests.

<a id="reconnaissance-sans-vol"></a>
## Recognition without flight

In **Piloting > Drone Stream**:

1. Verify that the received image is recent.
2. Manually rotate the drone between views, then choose **Capture view** six times. This test sends no commands to the motors and does not switch the camera itself.
3. Consult the locally saved montage and report on the Pi.
4. Use **Aim at the cross** to verify detection and centering in the image. A centered cross does not attest to a landing.

Geometric detection uses OpenCV, without an LLM or API key. A prolonged loss of the stream stops the analysis; it does not automatically resume upon reconnection. Camera switching remains experimental and must be verified visually.

<a id="ce-qui-reste-à-valider"></a>
## What remains to be validated

Before any real commands, the direction and neutrality of each axis, the frame rate, the behavior in case of BLE loss, and stopping must be verified on the bench with propellers removed. The active camera must also be visually identified, available flight data and their units confirmed, and a controller tested with flapping independent of the video. Only then can the simulated mission be adapted for supervised ground and flight tests.

The [simulated mission](DRONE_MISSION.en.md) includes takeoff, reconnaissance loop, recentering, and return to the platform. This scenario remains a preparation tool; its results do not constitute proof of flight.

<a id="vérifier-le-code-sans-matériel"></a>
## Verify code without hardware

Since `software/rover`:

```sh
.venv/bin/python -m pytest -q tests/test_drone.py
.venv/bin/ruff check fieldmind tests
node --check fieldmind/web/js/vues/pilotage.js
```
