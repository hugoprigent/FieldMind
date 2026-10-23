[Français](README.md) · **English**

# FieldMind software

## Current rover

The software used with the prototype is in [`rover/`](rover/README.en.md): Raspberry Pi, drivers, navigation, missions, AI, web station and hardware simulation. The firmware is in [`esp32/esp_c5/`](esp32/esp_c5/README.en.md). Start with the [main README](../README.en.md).

## First web demo — MuJoCo

This stack is the project's first MuJoCo demo. It illustrates navigation in a simulated forest and has its own services and interface. It is separate from the current rover stack; the [public simulation](https://hugoprigent.com/fieldmind/) now serves the rover's station (`software/rover/fieldmind/public_demo.py`).

Python 3.12 and, on Linux, OSMesa (`sudo apt install libosmesa6`):

```bash
cd software
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements-dev.txt
.venv/bin/uvicorn services.api.serveur:app --host 127.0.0.1 --port 8000
```

Open http://127.0.0.1:8000/. On Linux without a display, set `MUJOCO_GL=osmesa`. Without an API key, the map and the simulated navigation remain available; model-driven instructions are not. Perception based on ground truth is labeled as simulated.

```bash
# Quick tests for this stack
.venv/bin/python -m pytest -q -m "not lent and not navigateur"
node --test "tests/js/**/*.test.js"
```

A Docker image is described in [`Dockerfile`](Dockerfile). The hosting configurations are in [`deployment/demo_web/`](deployment/demo_web/README.md) (in French). Publishing this code does not change the service already online.

## ROS 2 studies

[`ros2/`](ros2/README.md) keeps the first ROS 2 / Gazebo trials and their tools. The ROS 2 bridge connected to the current software is in [`rover/ros2/`](rover/ros2/README.md) (both in French). The old capability descriptions do not amount to validation on the physical prototype.
