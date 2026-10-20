"""Tout : pont + description, slam_toolbox, Nav2, Foxglove bridge (et RViz en option).

    ros2 launch fieldmind_rover_bringup tout.launch.py rover:=rover.local:8080

Arguments : `rover`, `jeton`, `images` (comme pont.launch.py), `slam` (true), `navigation`
(true, nécessite slam), `controleur` (mppi | rpp), `foxglove` (true), `port_foxglove` (8765),
`rviz` (false).

Avec Nav2, un but publié sur /goal_pose (RViz « 2D Goal Pose », Foxglove) va à Nav2, qui
envoie /cmd_vel au rover ; le pont écoute alors /fieldmind/but pour la navigation propre du
rover. Sans Nav2 (navigation:=false), /goal_pose va directement au rover (`but`).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    partage = Path(get_package_share_directory("fieldmind_rover_bringup"))
    lancements = partage / "launch"
    navigation = LaunchConfiguration("navigation")

    def inclure(nom: str, condition: IfCondition | None = None, **arguments: object):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lancements / nom)),
            launch_arguments=list(arguments.items()), condition=condition)

    sujet_but = PythonExpression(
        ["'/fieldmind/but' if '", navigation, "'.lower() == 'true' else '/goal_pose'"])
    return LaunchDescription([
        DeclareLaunchArgument("rover", default_value=EnvironmentVariable(
            "FIELDMIND_ROVER", default_value="rover.local:8080")),
        DeclareLaunchArgument("jeton", default_value=EnvironmentVariable(
            "FIELDMIND_JETON", default_value="")),
        DeclareLaunchArgument("images", default_value="true"),
        DeclareLaunchArgument("slam", default_value="true"),
        DeclareLaunchArgument("navigation", default_value="true"),
        DeclareLaunchArgument("controleur", default_value="mppi"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("port_foxglove", default_value="8765"),
        DeclareLaunchArgument("rviz", default_value="false"),
        inclure("pont.launch.py", rover=LaunchConfiguration("rover"),
                jeton=LaunchConfiguration("jeton"), images=LaunchConfiguration("images"),
                sujet_but=sujet_but),
        inclure("slam.launch.py", IfCondition(LaunchConfiguration("slam"))),
        inclure("navigation.launch.py", IfCondition(navigation),
                controleur=LaunchConfiguration("controleur")),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            parameters=[{
                "port": ParameterValue(LaunchConfiguration("port_foxglove"), value_type=int),
                "address": "0.0.0.0",
                "send_buffer_limit": 50_000_000,  # images des deux webcams
            }],
            output="screen",
            condition=IfCondition(LaunchConfiguration("foxglove")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(partage / "rviz" / "rover.rviz")],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
