**Français** · [English](README.en.md)

# Pont ROS 2 du rover FieldMind

Relie les topics ROS 2 du rover réel au cerveau FieldMind, qui tourne sur le GX10 (serveur
`uvicorn services.api.serveur:app`). Le cerveau reçoit exactement les mêmes messages qu'en
simulation : c'est ce qui permet de faire rouler le même code de navigation sur le rover.

```
LD14P ──/scan──────┐
BNO085 ─/imu/data──┤                    WebSocket /ws/robot
VESC ───/odom──────┼──► pont_fieldmind ◄────────────────────► cerveau FieldMind (GX10)
ZED-F9P ─/fix──────┤         │
MCU ────/firmware/etat       └──► /ackermann_cmd ──► contrôleur embarqué (MCU)
```

| Topic | Type | Message du protocole |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `scan` |
| `/imu/data` | `sensor_msgs/Imu` | `imu` |
| `/odom` | `nav_msgs/Odometry` (vitesse de l'essieu arrière, VESC) | `odometrie` |
| `/fix` | `sensor_msgs/NavSatFix` (ZED-F9P) | `gnss`, dans le repère local de l'origine configurée |
| `/firmware/etat` | `diagnostic_msgs/DiagnosticStatus` | `firmware` |
| `/ackermann_cmd` (publié) | `ackermann_msgs/AckermannDriveStamped` | `commande` |

## Lancement (Ubuntu 24.04, ROS 2 Jazzy)

```bash
cd ~/ros2_ws/src && ln -s <dépôt>/software/ros2/packages/fieldmind_pont .
cd ~/ros2_ws && colcon build --packages-select fieldmind_pont && source install/setup.bash
ros2 launch fieldmind_pont pont.launch.py jeton:=$FIELDMIND_JETON_ROBOT
```

Puis ouvrir le poste opérateur sur `http://<GX10>:8000/?robot=externe`.

## État

- `conversions.py` (fonctions pures) est testé dans `software/tests/test_pont_ros2.py` contre le
  validateur du protocole.
- `noeud.py` (couche rclpy) **n'a pas encore tourné sur ROS** : pas de ROS dans l'environnement
  de développement. À valider sur le rover, pilotes en place (LD14P, BNO085, VESC, ZED-F9P).
- Les noms de topics dépendent des pilotes choisis : les adapter par remappage au lancement.
- Le contrôleur embarqué doit garder son chien de garde : si le pont ou le cerveau se tait, il
  arrête le rover de lui-même.
