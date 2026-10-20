"""Nav2 pour le rover Ackermann : Smac Hybrid-A* (Reeds-Shepp) et MPPI Ackermann (ou RPP).

    ros2 launch fieldmind_rover_bringup navigation.launch.py controleur:=mppi

Nécessite map → odom (slam.launch.py) et /odom, /scan (pont.launch.py). Nav2 n'est qu'une
source de consignes : /cmd_vel va au rover, qui garde sa propre sécurité.
Chaîne : controller_server et behavior_server → /cmd_vel_nav → velocity_smoother → /cmd_vel.
`controleur:=rpp` : Regulated Pure Pursuit, plus léger (Raspberry Pi 4).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NOEUDS_NAV2 = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "velocity_smoother",
    "bt_navigator",
    "waypoint_follower",
]


def _noeuds(contexte: LaunchContext) -> list[Node]:
    config = Path(get_package_share_directory("fieldmind_rover_bringup")) / "config"
    controleur = LaunchConfiguration("controleur").perform(contexte)
    if controleur not in ("mppi", "rpp"):
        raise ValueError(f"controleur:={controleur} : mppi ou rpp attendu")
    sim = LaunchConfiguration("use_sim_time").perform(contexte).lower() == "true"
    commun = {"use_sim_time": sim}
    parametres: list = [str(config / "nav2.yaml")]
    if controleur == "rpp":
        parametres.append(str(config / "nav2_rpp.yaml"))
    parametres.append(commun)
    vers_lisseur = [("cmd_vel", "cmd_vel_nav")]
    return [
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=parametres, remappings=vers_lisseur),
        Node(package="nav2_smoother", executable="smoother_server", output="screen",
             parameters=parametres),
        Node(package="nav2_planner", executable="planner_server", output="screen",
             parameters=parametres),
        Node(package="nav2_behaviors", executable="behavior_server", output="screen",
             parameters=parametres, remappings=vers_lisseur),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", output="screen",
             parameters=parametres,
             remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "/cmd_vel")]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", output="screen",
             parameters=[*parametres, {
                 "default_nav_to_pose_bt_xml": str(config / "naviguer_ackermann.xml"),
                 "default_nav_through_poses_bt_xml": str(config
                                                         / "naviguer_points_ackermann.xml"),
             }]),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower", output="screen",
             parameters=parametres),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[commun, {"autostart": True, "node_names": NOEUDS_NAV2}]),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("controleur", default_value="mppi",
                              description="Contrôleur local : mppi ou rpp"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_noeuds),
    ])
