> **Étude antérieure.** Le logiciel courant du prototype est dans [`../rover/`](../rover/README.md). Les résultats Gazebo ci-dessous sont simulés ; ils ne valident pas un rover autonome réel.

# Pile ROS 2 du rover FieldMind

La même pile tourne sur le rover réel et en simulation : seuls les pilotes des capteurs changent
(Gazebo à la place du matériel) et `use_sim_time`.

- **ROS 2 Jazzy**, **Gazebo Harmonic** (moteur physique DART), **Nav2**, **robot_localization**,
  **slam_toolbox** (en option).
- Monde naturel généré : relief, ruisseau avec gué, hameau, forêt, rochers, troncs couchés.

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

## Sur quelle machine

Ubuntu 24.04, installé directement, dans une machine virtuelle ou sous WSL2 (Windows). Un serveur
sans écran convient : Gazebo tourne sans interface et rend les caméras hors écran.

| | Minimum | Recommandé |
|---|---|---|
| Processeur | 4 cœurs | 8 cœurs ou plus |
| Mémoire | 8 Go | 16 Go |
| Carte graphique | aucune | NVIDIA récente (GTX 1060, RTX…) |
| Disque | environ 15 Go (ROS 2 + Gazebo + modèles) | idem |

Mesure de référence : 4 cœurs, 16 Go, sans carte graphique. La simulation tourne à environ la
moitié du temps réel (Gazebo occupe un peu plus d'un cœur), avec environ 3 Go de mémoire. Toute la
pile suit l'horloge simulée : le comportement est le même, seulement plus lent. Avec une carte
graphique, la simulation passe en temps réel et les caméras sont fluides.

L'agent de mission prévu utilisera des modèles gratuits d'OpenRouter, donc aucun calcul local. Des
modèles de vision locaux demanderaient une carte NVIDIA de 8 à 12 Go.

Sur le rover réel, seule l'autonomie (`fieldmind_bringup` et `fieldmind_perception`) tourne sur
l'ordinateur de bord ; Gazebo est remplacé par les capteurs.

## Lancer

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

Options :

- `graine:=N` : un autre monde ; `spectateur:=true` : caméra extérieure fixe
  (`/spectateur/image` à relier avec `ros_gz_bridge`) ;
- `monde:=<fichier.sdf> x:=… y:=… z:=… lacet:=…` : un monde quelconque (par exemple
  `worlds/essai_plat.sdf`) ;
- `localisation:=slam` (autonomie) : localisation par le lidar au lieu du GNSS, pour un lieu
  sans ciel (bâtiment).

Pour voir ce que fait le rover : `rviz2`, repère fixe `map`, afficher `/global_costmap/costmap`,
`/traversabilite/carte`, `/plan`, `/camera/image_raw`.

## Architecture

| Étape | Paquet / nœud | Entrées → sorties |
|---|---|---|
| Rover simulé | `fieldmind_description` (URDF), Gazebo | lidar 2D `/scan`, caméra `/camera/image_raw`, profondeur `/profondeur/points`, centrale `/imu/data`, GNSS |
| Récepteur GNSS simulé | `fieldmind_simulation/recepteur_gnss` | position exacte → `/gnss/fix` avec l'erreur d'un ZED-F9P : RTK fixe (≈ 1,5 cm) en terrain dégagé, flottant puis autonome sous les arbres |
| Microcontrôleur simulé | `fieldmind_simulation/controleur_embarque` | `/cmd_vel` → roues et servo (rampes, chien de garde, arrêt d'urgence sur le lidar, modèle du servo : vitesse, zone morte, jeu) ; `/odom_roues`, `/mcu/etat` |
| Traversabilité | `fieldmind_perception/traversabilite` | profondeur + lidar + centrale → carte d'élévation locale → `/traversabilite/carte` (libre, obstacle, inconnu), `/traversabilite/raison`, `/scan_filtre` |
| Localisation | `robot_localization` : `ekf_local`, `navsat_transform`, `ekf_global` | roues + gyroscope → `odom` ; + GNSS et cap → `map` |
| Navigation | Nav2 : Smac Hybrid-A* (Reeds-Shepp), MPPI Ackermann | destination → `/cmd_vel` ; replanification toutes les 10 s et dès que le chemin est bloqué |

### Traversabilité

Un lidar 2D seul ne suffit pas dehors : sur une montée, son faisceau (32 cm au-dessus du sol)
touche le terrain et le prend pour un mur. La caméra de profondeur construit une carte
d'élévation (cellules de 10 cm, 24 × 24 m autour du rover). Une cellule est un obstacle quand :

- **marche** : un point dépasse le sol voisin de plus de 15 cm (rocher, tronc, souche, mur) ;
- **pente** : le sol est plus raide que 22° (flanc de butte, berge) ;
- **trou** : une zone masquée entre deux zones de sol vues, près du rover (fossé, berge qui
  plonge) ;
- **lidar** : un impact dépasse le sol vu par la caméra, ou il est à moins de 1,2 m du rover.

Chaque image est recalée en hauteur sur le sol déjà vu, car l'altitude intégrée dérive. La
costmap locale est remplacée par la carte à chaque mise à jour : ce que la carte a oublié
disparaît aussi de la costmap. La costmap globale garde en mémoire les obstacles confirmés. Hors
de ce qui a été vu, la planification est optimiste (l'inconnu est traité comme libre) et le
chemin est corrigé en roulant.

### Localisation

Architecture de hardware/sensors/positionnement_gnss.md (référence de travail non incluse) :
GNSS RTK, centrale (cap absolu, comme le BNO085 en mode « rotation vector ») et odométrie des
roues, fusionnés par deux EKF. La précision annoncée par le récepteur règle la confiance du
filtre : sous les arbres, la position repose davantage sur l'odométrie.

## Évaluation

Parcours de la piste en étapes d'environ 15 m. La vérité terrain de la simulation sert
seulement à la mesure :

Étapes d'environ 15 m (`--pas 8`) :

| Monde (hameau → colline → gué → forêt → clairière, 70 à 90 m) | Étapes atteintes | Écart final | Erreur de localisation max | Plus près d'un obstacle |
|---|---|---|---|---|
| graine 1 | 6 / 6 | 0,33 à 0,46 m | 0,16 m | 1,45 m |
| graine 7 | 5 / 5 | 0,32 à 0,77 m | 0,71 m (RTK flottant en forêt) | 1,30 m |
| graine 42 | 5 / 5 | 0,23 à 0,72 m | 1,08 m (RTK flottant en forêt) | 0,60 m (tronc couché) |

Détail : rapports/parcours_piste.json (référence de travail non incluse). Reproduire avec la
simulation (`graine:=7`) et l'autonomie lancées :
`python3 outils/parcours_piste.py ~/.cache/fieldmind/mondes/7/description.json`.

Deux pièges trouvés en route, et corrigés :

- Placer une mesure avec la dernière pose connue au lieu de la pose à l'instant de la mesure. En
  virage sur une pente, cela produisait de fausses marches ; le rover a même coupé à travers le
  ruisseau au lieu de passer par le gué, puis s'est retrouvé piégé au fond du lit.
- Prendre l'ombre d'une crête pour un fossé.

Reste à faire : éviter une descente sans issue visible (le planificateur est optimiste dans
l'inconnu).

## Limites connues

- La caméra de profondeur ne voit que devant (73°). Sur les côtés et à l'arrière, seul le lidar
  2D voit, et loin de tout sol déjà vu il ne distingue pas un obstacle d'une pente.
- L'eau est vue comme une surface plane : seules les berges raides arrêtent le rover, pas la
  profondeur de l'eau.
- Les roues avant libres glissent un peu en simulation (contact sphère/maillage) ; l'odométrie
  vient des roues arrière, comme sur le rover.
- Sans GPU, la simulation tourne à environ la moitié du temps réel.
