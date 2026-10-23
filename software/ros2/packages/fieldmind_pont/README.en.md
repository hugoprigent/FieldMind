[Français](README.md) · **English**

<a id="pont-ros-2-du-rover-fieldmind"></a>
# FieldMind Rover ROS 2 Bridge

Connects the real rover's ROS 2 topics to the FieldMind brain, which runs on the GX10 (server `uvicorn services.api.serveur:app`). The brain receives exactly the same messages as in simulation: this is what allows the same navigation code to run on the rover.

```
LD14P ──/scan──────┐
BNO085 ─/imu/data──┤                    WebSocket /ws/robot
VESC ───/odom──────┼──► pont_fieldmind ◄────────────────────► cerveau FieldMind (GX10)
ZED-F9P ─/fix──────┤         │
MCU ────/firmware/etat       └──► /ackermann_cmd ──► contrôleur embarqué (MCU)
```

| Topic | Type | Protocol Message |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `scan` |
| `/imu/data` | `sensor_msgs/Imu` | `imu` |
| `/odom` | `nav_msgs/Odometry` (rear axle speed, VESC) | `odometrie` |
| `/fix` | `sensor_msgs/NavSatFix` (ZED-F9P) | `gnss`, in the coordinate system of the configured origin |
| `/firmware/etat` | `diagnostic_msgs/DiagnosticStatus` | `firmware` |
| `/ackermann_cmd` (published) | `ackermann_msgs/AckermannDriveStamped` | `commande` |

<a id="lancement-ubuntu-2404-ros-2-jazzy"></a>
## Launch (Ubuntu 24.04, ROS 2 Jazzy)

```bash
cd ~/ros2_ws/src && ln -s <dépôt>/software/ros2/packages/fieldmind_pont .
cd ~/ros2_ws && colcon build --packages-select fieldmind_pont && source install/setup.bash
ros2 launch fieldmind_pont pont.launch.py jeton:=$FIELDMIND_JETON_ROBOT
```

Then open the operator station on `http://<GX10>:8000/?robot=externe`.

<a id="état"></a>
## Status

- `conversions.py` (pure functions) is tested in `software/tests/test_pont_ros2.py` against the protocol validator.
- `noeud.py` (rclpy layer) **has not yet run on ROS**: no ROS in the development environment. To be validated on the rover, with drivers in place (LD14P, BNO085, VESC, ZED-F9P).
- Topic names depend on the chosen drivers: adapt them via remapping at launch.
- The onboard controller must keep its watchdog: if the bridge or the brain goes silent, it stops the rover on its own.
