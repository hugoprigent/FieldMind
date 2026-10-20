"""Pont avec le rover et description du robot (TF statiques des capteurs).

    ros2 launch fieldmind_rover_bringup pont.launch.py rover:=rover.local:8080 jeton:=…

Arguments : `rover` (défaut : $FIELDMIND_ROVER, sinon rover.local:8080), `jeton` (défaut :
$FIELDMIND_JETON), `images` (true), `sujet_but` (/goal_pose : un but RViz/Foxglove part au
rover, qui navigue lui-même ; tout.launch.py le déplace sur /fieldmind/but quand Nav2 tourne).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory("fieldmind_rover_bringup")) / "config"
    description = (Path(get_package_share_directory("fieldmind_rover_description"))
                   / "launch" / "description.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument(
            "rover", default_value=EnvironmentVariable("FIELDMIND_ROVER",
                                                       default_value="rover.local:8080"),
            description="Adresse du rover (hôte:port) ou URL ws://…"),
        DeclareLaunchArgument(
            "jeton", default_value=EnvironmentVariable("FIELDMIND_JETON", default_value=""),
            description="Jeton FIELDMIND_JETON du rover (vide : aucun)"),
        DeclareLaunchArgument("images", default_value="true",
                              description="Recevoir les images des webcams"),
        DeclareLaunchArgument("sujet_but", default_value="/goal_pose",
                              description="Sujet PoseStamped envoyé au rover comme `but`"),
        DeclareLaunchArgument("description", default_value="true",
                              description="Lancer robot_state_publisher"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(description)),
                                 condition=IfCondition(LaunchConfiguration("description"))),
        Node(
            package="fieldmind_rover_ros",
            executable="pont",
            name="pont_fieldmind",
            parameters=[
                str(config / "pont.yaml"),
                {
                    "rover": ParameterValue(LaunchConfiguration("rover"), value_type=str),
                    "jeton": ParameterValue(LaunchConfiguration("jeton"), value_type=str),
                    "images": ParameterValue(LaunchConfiguration("images"), value_type=bool),
                    "sujet_but": ParameterValue(LaunchConfiguration("sujet_but"),
                                                value_type=str),
                },
            ],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
