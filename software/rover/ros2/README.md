**Français** · [English](README.en.md)

# Pile ROS 2 du rover FieldMind

Le rover n'exécute pas ROS : son programme Python (`python -m fieldmind`, sur le Pi) expose la
WebSocket `/ws/ros` (format dans [../ARCHITECTURE.md](../ARCHITECTURE.md)). Cette pile ROS 2
Jazzy, dans Docker sur l'ASUS GX10 (ou sur le Pi), s'y connecte :

```
rover (Pi) ── /ws/ros ──► pont ──► /scan /odom /tf /imu/data_raw /gps/fix /camera/*/compressed
     ▲                     │        └──► slam_toolbox (/map, map → odom) ──► Nav2
     └── cmd_vel, but ◄────┴──────────── /cmd_vel ◄───────────────────────────┘
```

Nav2 n'est qu'une **source de consignes** : le rover ne suit `/cmd_vel` qu'en mode `auto`, armé,
sans mission en cours, et chaque consigne passe par sa propre sécurité (lidar, stéréo,
inclinaison, homme mort). Le pont ne répète jamais une consigne et n'en garde aucune pendant une
coupure : si ROS se tait, le rover s'arrête de lui-même.

| Paquet | Contenu |
| --- | --- |
| `fieldmind_rover_ros` | nœud `pont` : `conversions.py` (sans ROS), `client.py` (aiohttp, reconnexion), `pont.py` (rclpy) |
| `fieldmind_rover_description` | URDF du rover réel (valeurs de `config/rover.toml`), robot_state_publisher |
| `fieldmind_rover_bringup` | `pont`, `slam`, `navigation`, `tout.launch.py` ; réglages Nav2, slam_toolbox, RViz |

## Installation

**GX10** (DGX OS, Docker et le greffon compose déjà installés) :

```bash
cd software/rover/ros2/docker
cp env.exemple .env        # adresse du rover, jeton
./lancer.sh construire     # 10 à 30 min la première fois
./lancer.sh                # pont + slam_toolbox + Nav2 + Foxglove (port 8765)
```

**Raspberry Pi 4** (OS **64 bits** obligatoire, `uname -m` = `aarch64`) :
`curl -fsSL https://get.docker.com | sh`, puis la même chose avec `FIELDMIND_ROVER=127.0.0.1:8080`
dans `.env` et `./lancer.sh pi` (contrôleur RPP au lieu de MPPI, sans images). Le GX10 et le Pi
sont tous deux arm64 : l'image construite sur le GX10 sert telle quelle,
`docker save fieldmind/rover-ros2:jazzy | ssh pi docker load`. Le Pi 4 fait déjà tourner le rover :
la cartographie et Nav2 y prennent une bonne part du processeur, le GX10 est préférable.

Autres commandes : `./lancer.sh pont` (pont seul), `slam` (sans Nav2), `rviz`, `etat`,
`journal`, `shell`, `arreter`. Sans Docker (Ubuntu 24.04 + ROS 2 Jazzy) : `colcon build` dans
`software/rover/ros2`, puis `ros2 launch fieldmind_rover_bringup tout.launch.py rover:=…`.

## Utilisation

1. Lancer le rover, ouvrir le poste (`http://rover.local:8080`), mode **auto**, **armer**.
2. Ouvrir **Foxglove** (application ou navigateur) → *Open connection* → *Foxglove WebSocket*
   `ws://<gx10>:8765`. Panneau 3D, repère `map` : `/map`, `/scan`, `/global_costmap/costmap`,
   `/plan`, le modèle du rover (`/robot_description`) ; panneau Image :
   `/camera/gauche/image_raw/compressed` ; panneau Diagnostics : `/diagnostics` (liaison, état du
   rover, sécurité) ; `/fieldmind/etat` (JSON brut) en Raw Messages.
   **RViz** : `./lancer.sh rviz` sur une machine avec écran (configuration `rviz/rover.rviz`).
3. **Envoyer un but** : bouton *Publish* (pose) du panneau 3D de Foxglove, sujet réglé sur
   `/goal_pose` dans les réglages du panneau, ou
   *2D Goal Pose* de RViz, ou `./lancer.sh but 3.0 1.5` (repère `map`). Avec Nav2 lancé, le but va
   à Nav2 ; la navigation propre du rover reste accessible sur `/fieldmind/but`. Sans Nav2
   (`./lancer.sh pont` ou `slam`), `/goal_pose` part directement au rover (message `but`).

| Sujet | Type | Repère |
| --- | --- | --- |
| `/scan` | LaserScan, 400 rayons de 0,9° sur 360°, +inf = pas d'écho | `lidar_link` (z = 0,322 m) |
| `/odom` + TF `odom → base_link` | Odometry (recalage lidar + gyroscope du rover) | `base_link` au sol, essieu arrière |
| `/imu/data_raw` | Imu, sans orientation (MPU-6050 sans magnétomètre) | `imu_link` |
| `/gps/fix` | NavSatFix, σ = HDOP × 2,5 m | `gnss_link` |
| `/camera/{gauche,droite}/image_raw/compressed`, `…/camera_info` | JPEG à 5 Hz, CameraInfo approchée (78°) | `camera_*_optical_frame` |
| `/fieldmind/etat`, `/fieldmind/connecte`, `/diagnostics` | String (JSON), Bool, DiagnosticArray | |
| `/cmd_vel` (lu) | Twist → `cmd_vel` (bornes 0,5 / −0,3 m/s, courbure conservée) | |
| `/goal_pose` ou `/fieldmind/but` (lu) | PoseStamped → `but` (ramené dans `odom` par TF) | |

Les dates du Pi (`t`, `t_envoi`) sont ramenées à l'horloge de la machine ROS (latence minimale
sur 20 s glissantes) : pas besoin de synchroniser les horloges. Réglages : `config/pont.yaml`,
`nav2.yaml` (Smac Hybrid-A* Reeds-Shepp, MPPI Ackermann, rayon 0,80 m, emprise 0,57 × 0,45 m),
`nav2_rpp.yaml`, `slam.yaml`.

## Vérifié

- `software/rover/tests/test_ros2_*.py`, sans ROS : conversions, client contre un faux `/ws/ros`
  (réception, envoi, jeton, reconnexion, silence, rien de gardé pendant une coupure), cohérence
  URDF / Nav2 / pont avec `rover.toml` (`.venv/bin/python -m pytest tests/test_ros2_*.py`).
- Sur un Mac (ROS 2 Jazzy de RoboStack) : `colcon build`, `check_urdf`, et la pile complète contre
  `python -m fieldmind --materiel simulation` : sujets aux bonnes fréquences, `cmd_vel` suivis par
  le rover simulé, trois buts Nav2 atteints (MPPI et RPP, dont un avec marche arrière),
  reconnexion après redémarrage du rover.

## Limites

- **L'image Docker n'a pas été construite ici** (pas de démon Docker) : à vérifier sur le GX10.
  Rien n'a encore tourné avec le vrai rover.
- Obstacles plus bas que 32 cm : invisibles au lidar, donc à Nav2 ; seule la sécurité du rover
  (stéréo) les voit. Les secteurs sans écho ne sont pas effacés des costmaps (`inf_is_valid:
  false`) : dehors, un obstacle parti peut rester marqué.
- Pas de codeurs de roue : `/odom` est l'estimation du rover (lidar + gyroscope). Si le rover
  redémarre ou remet son odométrie à zéro, `odom` saute : relancer la pile (slam_toolbox).
- Le GPS (2 à 3 m) est publié mais pas fusionné ; pas de cap absolu (pas de magnétomètre).
- Images : environ 2 × 50 à 150 Ko à 5 Hz sur le Wi-Fi ; `IMAGES=false` si la liaison peine.
- Fast DDS est limité à UDP (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`) : avec la mémoire partagée, les
  nœuds Nav2 restaient bloqués à l'activation lors des essais.
- Rayon de braquage : 0,63 m si 31° était l'angle de la roue virtuelle centrale, mais 31° est la
  butée de la roue intérieure (≈ 0,79 m à l'essieu arrière) ; Nav2 planifie à 0,80 m.
