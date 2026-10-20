# Pile ROS 2 Jazzy du rover FieldMind : pont /ws/ros, description, slam_toolbox, Nav2, Foxglove
# bridge, RViz. Image multi-architecture : arm64 (ASUS GX10, Raspberry Pi 4 sous OS 64 bits) et
# amd64. Construite depuis software/rover/ros2 (voir docker-compose.yml, lancer.sh).
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-tf2-tools \
        python3-aiohttp \
        python3-numpy \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/ src/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --event-handlers console_direct- \
    && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "fieldmind_rover_bringup", "tout.launch.py"]
