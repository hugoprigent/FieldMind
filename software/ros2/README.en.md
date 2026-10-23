[Français](README.md) · **English**

> **Previous study.** The current prototype software is in [`../rover/`](../rover/README.en.md). The Gazebo results below are simulated; they do not validate a real autonomous rover.

<a id="pile-ros-2-du-rover-fieldmind"></a>
# FieldMind Rover ROS 2 Stack

The same stack runs on the real rover and in simulation: only the sensor drivers change (Gazebo instead of hardware) and `use_sim_time`.

- **ROS 2 Jazzy**, **Gazebo Harmonic** (DART physics engine), **Nav2**, **robot_localization**, **slam_toolbox** (optional).
- Generated natural world: terrain, stream with ford, hamlet, forest, rocks, fallen logs.

## Installation (Ubuntu 24.04)

```bash
# ROS 2 Jazzy : https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt install ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization ros-jazzy-slam-toolbox ros-jazzy-ros-gz ros-jazzy-xacro \
  python3-colcon-common-extensions python3-numpy python3-scipy python3-pil
source /opt/ros/jazzy/setup.bash
cd software/ros2
colcon build --symlink-install
source install/setup.bash
```

<a id="sur-quelle-machine"></a>
## On which machine

Ubuntu 24.04, installed directly, in a virtual machine or under WSL2 (Windows). A headless server is suitable: Gazebo runs without an interface and renders cameras off-screen.

| | Minimum | Recommended |
|---|---|---|
| Processor | 4 cores | 8 cores or more |
| Memory | 8 GB | 16 GB |
| Graphics card | none | Recent NVIDIA (GTX 1060, RTX…) |
| Disk | approx. 15 GB (ROS 2 + Gazebo + models) | same |

Reference measurement: 4 cores, 16 GB, no graphics card. Simulation runs at about half real-time speed (Gazebo uses slightly more than one core), with about 3 GB of memory. The entire stack follows the simulated clock: behavior is the same, just slower. With a graphics card, simulation runs in real-time and cameras are smooth.

The planned mission agent will use free models from OpenRouter, so no local computation. Local vision models would require an NVIDIA card with 8 to 12 GB.

On the real rover, only autonomy (`fieldmind_bringup` and `fieldmind_perception`) runs on the onboard computer; Gazebo is replaced by sensors.

<a id="lancer"></a>
## Launch

```bash
# 1. Simulation : monde n° 7 généré (quelques secondes la première fois, puis mis en cache dans
#    ~/.cache/fieldmind/mondes/7), rover posé au départ de la piste.
ros2 launch fieldmind_simulation simulation.launch.py graine:=7

# 2. Autonomie (identique sur le rover réel, sans use_sim_time)
ros2 launch fieldmind_bringup autonomie.launch.py use_sim_time:=true

# 3. Une destination (repère map : origine au point de départ, x vers l'est, y vers le nord)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 12.0}, orientation: {w: 1.0}}}}"
```

Options:

- `graine:=N` : another world; `spectateur:=true` : fixed external camera (`/spectateur/image` to be connected to `ros_gz_bridge`);
- `monde:=<fichier.sdf> x:=… y:=… z:=… lacet:=…` : any world (for example `worlds/essai_plat.sdf`);
- `localisation:=slam` (autonomy): localization via lidar instead of GNSS, for a location without sky (building).

To see what the rover does: `rviz2`, fixed frame `map`, display `/global_costmap/costmap`, `/traversabilite/carte`, `/plan`, `/camera/image_raw`.

## Architecture

| Step | Package / node | Inputs → outputs |
|---|---|---|
| Simulated rover | `fieldmind_description` (URDF), Gazebo | 2D lidar `/scan`, camera `/camera/image_raw`, depth `/profondeur/points`, central unit `/imu/data`, GNSS |
| Simulated GNSS receiver | `fieldmind_simulation/recepteur_gnss` | exact position → `/gnss/fix` with the error of a ZED-F9P: fixed RTK (≈ 1,5 cm) in open terrain, floating then autonomous under trees |
| Simulated microcontroller | `fieldmind_simulation/controleur_embarque` | `/cmd_vel` → wheels and servo (ramps, watchdog, emergency stop on lidar, servo model: speed, dead zone, backlash); `/odom_roues`, `/mcu/etat` |
| Traversability | `fieldmind_perception/traversabilite` | depth + lidar + central unit → local elevation map → `/traversabilite/carte` (free, obstacle, unknown), `/traversabilite/raison`, `/scan_filtre` |
| Localization | `robot_localization` : `ekf_local`, `navsat_transform`, `ekf_global` | wheels + gyroscope → `odom` ; + GNSS and heading → `map` |
| Navigation | Nav2: Smac Hybrid-A* (Reeds-Shepp), MPPI Ackermann | destination → `/cmd_vel` ; replanning every 10 s and as soon as the path is blocked |

<a id="traversabilité"></a>
### Traversability

A 2D lidar alone is not enough outdoors: on an incline, its beam (32 cm above the ground) hits the terrain and mistakes it for a wall. The depth camera builds an elevation map (cells of 10 cm, 24 × 24 m around the rover). A cell is an obstacle when:

- **step**: a point exceeds the neighboring ground by more than 15 cm (rock, trunk, stump, wall);
- **slope**: the ground is steeper than 22° (hillside, bank);
- **hole**: a masked area between two seen ground areas, near the rover (ditch, bank that drops off);
- **lidar**: an impact exceeds the ground seen by the camera, or it is less than 1,2 m from the rover.

Each image is re-aligned in height on the ground already seen, because the integrated altitude drifts. The local costmap is replaced by the map at each update: what the map has forgotten also disappears from the costmap. The global costmap keeps track of confirmed obstacles. Outside of what has been seen, planning is optimistic (the unknown is treated as free) and the path is corrected while driving.

<a id="localisation"></a>
### Localization

Architecture of hardware/sensors/positionnement_gnss.md (working reference not included): GNSS RTK, IMU (absolute reference, such as the BNO085 in "rotation vector" mode) and wheel odometry, fused by two EKF filters. The accuracy announced by the receiver sets the filter's confidence: under trees, positioning relies more on odometry.

<a id="évaluation"></a>
## Evaluation

Track traversal in steps of approximately 15 m. The simulation's ground truth is used only for measurement:

Steps of approximately 15 m (`--pas 8`):

| World (village → hill → ford → forest → clearing, 70 to 90 m) | Steps reached | Final deviation | Max localization error | Closest to an obstacle |
|---|---|---|---|---|
| seed 1 | 6 / 6 | 0,33 to 0,46 m | 0,16 m | 1,45 m |
| seed 7 | 5 / 5 | 0,32 to 0,77 m | 0,71 m (RTK float in forest) | 1,30 m |
| seed 42 | 5 / 5 | 0,23 to 0,72 m | 1,08 m (RTK float in forest) | 0,60 m (fallen trunk) |

Detail: rapports/parcours_piste.json (working reference not included). Reproduce with simulation (`graine:=7`) and autonomy launched: `python3 outils/parcours_piste.py ~/.cache/fieldmind/mondes/7/description.json`.

Two traps found along the way, and corrected:

- Place a measurement using the last known pose instead of the pose at the time of measurement. On a curve on a slope, this produced false steps; the rover even cut across the stream instead of going through the ford, then got trapped at the bottom of the bed.
- Mistaking the shadow of a ridge for a ditch.

Remaining to do: avoid a descent with no visible exit (the planner is optimistic about the unknown).

<a id="limites-connues"></a>
## Known limitations

- The depth camera only sees forward (73°). On the sides and rear, only the 2D lidar sees, and far from any ground already seen, it cannot distinguish an obstacle from a slope.
- Water is seen as a flat surface: only steep banks stop the rover, not the water depth.
- The free front wheels slip slightly in simulation (sphere/mesh contact); odometry comes from the rear wheels, as on the rover.
- Without a GPU, the simulation runs at about half real-time speed.
