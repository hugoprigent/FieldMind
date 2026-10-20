"""Pile autonome du rover : traversabilité, localisation (EKF), SLAM et Nav2. Identique en
simulation et sur le rover réel (seul `use_sim_time` change).

    ros2 launch fieldmind_bringup autonomie.launch.py use_sim_time:=true localisation:=gnss

localisation:=gnss (défaut, extérieur) : GNSS RTK + odométrie + centrale, deux EKF et
navsat_transform. localisation:=slam : slam_toolbox sur le lidar (intérieur, pas de ciel).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NOEUDS_NAV2 = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "velocity_smoother",
    "bt_navigator",
]


def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory("fieldmind_bringup")) / "config"
    sim = LaunchConfiguration("use_sim_time")
    commun = {"use_sim_time": sim}
    nav2 = str(config / "nav2.yaml")
    arbre = str(config / "naviguer_ackermann.xml")
    arbre_points = str(config / "naviguer_points_ackermann.xml")
    ekf = str(config / "ekf.yaml")
    gnss = LaunchConfigurationEquals("localisation", "gnss")
    slam = LaunchConfigurationEquals("localisation", "slam")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("localisation", default_value="gnss", choices=["gnss", "slam"]),
            Node(
                package="fieldmind_perception",
                executable="traversabilite",
                parameters=[commun],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_local",
                parameters=[ekf, commun],
                remappings=[("odometry/filtered", "/odometry/filtered")],
            ),
            # --- Localisation globale par GNSS RTK (map → odom)
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                parameters=[ekf, commun],
                remappings=[
                    ("imu", "/imu/data"),
                    ("gps/fix", "/gnss/fix"),
                    ("odometry/filtered", "/odometry/global"),
                    ("odometry/gps", "/odometry/gps"),
                    ("gps/filtered", "/gnss/filtre"),
                ],
                condition=gnss,
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_global",
                parameters=[ekf, commun],
                remappings=[("odometry/filtered", "/odometry/global")],
                condition=gnss,
            ),
            # --- Ou localisation par le lidar (map → odom)
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[str(config / "slam.yaml"), commun],
                condition=slam,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_slam",
                parameters=[
                    commun,
                    {"autostart": True, "node_names": ["slam_toolbox"], "bond_timeout": 0.0},
                ],
                condition=slam,
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                parameters=[nav2, commun],
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(package="nav2_smoother", executable="smoother_server", parameters=[nav2, commun]),
            Node(package="nav2_planner", executable="planner_server", parameters=[nav2, commun]),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                parameters=[nav2, commun],
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                parameters=[nav2, commun],
                remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "/cmd_vel")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                parameters=[
                    nav2,
                    commun,
                    {
                        "default_nav_to_pose_bt_xml": arbre,
                        "default_nav_through_poses_bt_xml": arbre_points,
                    },
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[commun, {"autostart": True, "node_names": NOEUDS_NAV2}],
            ),
        ]
    )
