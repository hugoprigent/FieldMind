[Français](README.md) · **English**

<a id="pile-ros-2-du-rover-fieldmind"></a>
# ROS 2 stack for the FieldMind rover

The rover does not run ROS: its Python program (`python -m fieldmind`, on the Pi) exposes the WebSocket `/ws/ros` (format in [../ARCHITECTURE.md](../ARCHITECTURE.en.md)). This ROS 2 Jazzy stack, in Docker on the ASUS GX10 (or on the Pi), connects to it:

```
rover (Pi) ── /ws/ros ──► pont ──► /scan /odom /tf /imu/data_raw /gps/fix /camera/*/compressed
     ▲                     │        └──► slam_toolbox (/map, map → odom) ──► Nav2
     └── cmd_vel, but ◄────┴──────────── /cmd_vel ◄───────────────────────────┘
```

Nav2 is only a **command source**: the rover follows `/cmd_vel` only in `auto` mode, armed, with no mission in progress, and each command goes through its own safety checks (lidar, stereo, tilt, dead man). The bridge never repeats a command and keeps none during a disconnection: if ROS goes silent, the rover stops on its own.

| Package | Content |
| --- | --- |
| `fieldmind_rover_ros` | `pont` node: `conversions.py` (no ROS), `client.py` (aiohttp, reconnection), `pont.py` (rclpy) |
| `fieldmind_rover_description` | URDF of the real rover (`config/rover.toml` values), robot_state_publisher |
| `fieldmind_rover_bringup` | `pont`, `slam`, `navigation`, `tout.launch.py` ; Nav2 settings, slam_toolbox, RViz |

## Installation

**GX10** (DGX OS, Docker and the compose plugin already installed) :

```bash
cd software/rover/ros2/docker
cp env.exemple .env        # adresse du rover, jeton
./lancer.sh construire     # 10 à 30 min la première fois
./lancer.sh                # pont + slam_toolbox + Nav2 + Foxglove (port 8765)
```

**Raspberry Pi 4** (**64 bits** OS required, `uname -m` = `aarch64`) : `curl -fsSL https://get.docker.com | sh`, then the same with `FIELDMIND_ROVER=127.0.0.1:8080` in `.env` and `./lancer.sh pi` (RPP controller instead of MPPI, without images). The GX10 and the Pi are both arm64: the image built on the GX10 is used as-is, `docker save fieldmind/rover-ros2:jazzy | ssh pi docker load`. The Pi 4 already runs the rover: mapping and Nav2 take up a good portion of the CPU there, so the GX10 is preferable.

Other commands: `./lancer.sh pont` (bridge only), `slam` (without Nav2), `rviz`, `etat`, `journal`, `shell`, `arreter`. Without Docker (Ubuntu 24.04 + ROS 2 Jazzy) : `colcon build` in `software/rover/ros2`, then `ros2 launch fieldmind_rover_bringup tout.launch.py rover:=…`.

<a id="utilisation"></a>
## Usage

1. Start the rover, open the station (`http://rover.local:8080`), **auto** mode, **arm**.
2. Open **Foxglove** (app or browser) → *Open connection* → *Foxglove WebSocket* `ws://<gx10>:8765`. 3D panel, frame `map`: `/map`, `/scan`, `/global_costmap/costmap`, `/plan`, the rover model (`/robot_description`); Image panel: `/camera/gauche/image_raw/compressed`; Diagnostics panel: `/diagnostics` (link, rover status, safety); `/fieldmind/etat` (raw JSON) in Raw Messages. **RViz**: `./lancer.sh rviz` on a machine with a screen (configuration `rviz/rover.rviz`).
3. **Send a goal**: *Publish* button (pose) from Foxglove's 3D panel, topic set to `/goal_pose` in the panel settings, or *2D Goal Pose* from RViz, or `./lancer.sh but 3.0 1.5` (frame `map`). With Nav2 running, the goal goes to Nav2; the rover's own navigation remains accessible on `/fieldmind/but`. Without Nav2 (`./lancer.sh pont` or `slam`), `/goal_pose` goes directly to the rover (message `but`).

| Topic | Type | Frame |
| --- | --- | --- |
| `/scan` | LaserScan, 400 rays from 0,9° to 360°, +inf = no echo | `lidar_link` (z = 0,322 m) |
| `/odom` + TF `odom → base_link` | Odometry (lidar recalibration + rover gyroscope) | `base_link` on the ground, rear axle |
| `/imu/data_raw` | Imu, without orientation (MPU-6050 without magnetometer) | `imu_link` |
| `/gps/fix` | NavSatFix, σ = HDOP × 2,5 m | `gnss_link` |
| `/camera/{gauche,droite}/image_raw/compressed`, `…/camera_info` | JPEG at 5 Hz, approximate CameraInfo (78°) | `camera_*_optical_frame` |
| `/fieldmind/etat`, `/fieldmind/connecte`, `/diagnostics` | String (JSON), Bool, DiagnosticArray | |
| `/cmd_vel` (read) | Twist → `cmd_vel` (limits 0,5 / −0,3 m/s, curvature preserved) | |
| `/goal_pose` or `/fieldmind/but` (read) | PoseStamped → `but` (brought into `odom` by TF) | |

Pi timestamps (`t`, `t_envoi`) are brought back to the ROS machine clock (minimal latency on 20 s sliding): no need to synchronize clocks. Settings: `config/pont.yaml`, `nav2.yaml` (Smac Hybrid-A* Reeds-Shepp, MPPI Ackermann, radius 0,80 m, footprint 0,57 × 0,45 m), `nav2_rpp.yaml`, `slam.yaml`.

<a id="vérifié"></a>
## Verified

- `software/rover/tests/test_ros2_*.py`, without ROS: conversions, client against a fake `/ws/ros` (reception, sending, token, reconnection, silence, nothing kept during an outage), URDF / Nav2 / bridge consistency with `rover.toml` (`.venv/bin/python -m pytest tests/test_ros2_*.py`).
- On a Mac (RoboStack ROS 2 Jazzy): `colcon build`, `check_urdf`, and the full stack against `python -m fieldmind --materiel simulation`: topics at correct frequencies, `cmd_vel` followed by the simulated rover, three Nav2 goals reached (MPPI and RPP, including one with reverse), reconnection after rover restart.

<a id="limites"></a>
## Limits

- **The Docker image was not built here** (no Docker daemon): to be verified on the GX10. Nothing has run yet with the real rover.
- Obstacles lower than 32 cm: invisible to the lidar, hence to Nav2; only the rover's safety system (stereo) sees them. Sectors without echo are not cleared from the costmaps (`inf_is_valid: false`): outside, a partial obstacle may remain marked.
- No wheel encoders: `/odom` is the rover's estimation (lidar + gyroscope). If the rover restarts or resets its odometry, `odom` jumps: restart the stack (slam_toolbox).
- The GPS (2 to 3 m) is published but not fused; no absolute heading (no magnetometer).
- Images: approximately 2 × 50 at 150 Ko to 5 Hz on Wi-Fi; `IMAGES=false` if the link struggles.
- Fast DDS is limited to UDP (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`): with shared memory, Nav2 nodes remained stuck at activation during tests.
- Turning radius: 0,63 m if 31° was the angle of the central virtual wheel, but 31° is the limit of the inner wheel (≈ 0,79 m at the rear axle); Nav2 plans at 0,80 m.
