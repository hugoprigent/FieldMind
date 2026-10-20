#!/usr/bin/env bash
# Pile ROS 2 du rover FieldMind dans Docker (ASUS GX10 ou Raspberry Pi 4, OS 64 bits).
#
#   ./lancer.sh construire     construit l'image (la première fois : 10 à 30 min)
#   ./lancer.sh [tout]         pont + slam_toolbox + Nav2 (MPPI) + Foxglove, en arrière-plan
#   ./lancer.sh pont           pont + description + Foxglove seulement (le plus léger)
#   ./lancer.sh slam           pont + slam_toolbox + Foxglove, sans Nav2
#   ./lancer.sh pi             tout, réglé pour le Pi : RPP au lieu de MPPI, sans images
#   ./lancer.sh rviz           RViz sur l'écran de cette machine (X11)
#   ./lancer.sh but X Y        but Nav2 dans le repère map (m)
#   ./lancer.sh etat           dernier état du rover et de la liaison
#   ./lancer.sh journal        suit les journaux
#   ./lancer.sh shell          invite ROS 2 dans le conteneur
#   ./lancer.sh arreter        arrête tout
#
# Adresse et jeton du rover : variables FIELDMIND_ROVER (défaut rover.local:8080) et
# FIELDMIND_JETON, ou fichier docker/.env (voir env.exemple).
set -euo pipefail
cd "$(dirname "$0")"

if ! command -v docker >/dev/null 2>&1; then
    echo "Docker est absent : https://docs.docker.com/engine/install/ (Ubuntu ou Debian)" >&2
    exit 1
fi
case "$(uname -m)" in
    aarch64 | arm64 | x86_64) ;;
    *) echo "Architecture $(uname -m) : il faut un OS 64 bits (arm64 ou amd64)" >&2; exit 1 ;;
esac

compose() { docker compose -f docker-compose.yml "$@"; }
dans() { compose exec ros2 /entrypoint.sh "$@"; }

commande="${1:-tout}"
case "$commande" in
    construire) compose build ros2 ;;
    tout) compose up -d ros2 ;;
    pont) SLAM=false NAVIGATION=false compose up -d ros2 ;;
    slam) NAVIGATION=false compose up -d ros2 ;;
    pi) CONTROLEUR=rpp IMAGES=false compose up -d ros2 ;;
    rviz)
        command -v xhost >/dev/null 2>&1 && xhost +local: >/dev/null
        compose --profile rviz up -d rviz
        ;;
    but)
        [ $# -eq 3 ] || { echo "usage : $0 but X Y" >&2; exit 2; }
        dans ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
            "{pose: {header: {frame_id: map}, pose: {position: {x: $2, y: $3}, orientation: {w: 1.0}}}}"
        ;;
    etat)
        dans ros2 topic echo --once /fieldmind/connecte
        dans ros2 topic echo --once --full-length /fieldmind/etat
        ;;
    journal) compose logs -f --tail 100 ;;
    shell) dans bash ;;
    arreter) compose --profile rviz down ;;
    *) sed -n '2,20p' "$0" >&2; exit 2 ;;
esac
