#!/bin/bash
# Charge ROS 2 et l'espace de travail du rover, puis exécute la commande demandée.
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /ws/install/setup.bash
exec "$@"
