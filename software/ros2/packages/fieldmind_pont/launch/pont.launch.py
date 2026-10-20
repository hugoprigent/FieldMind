"""Lance le pont FieldMind : ros2 launch fieldmind_pont pont.launch.py jeton:=<jeton>"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    parametres = Path(get_package_share_directory("fieldmind_pont")) / "config" / "pont.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("jeton", description="Jeton FIELDMIND_JETON_ROBOT du serveur"),
            Node(
                package="fieldmind_pont",
                executable="pont",
                name="pont_fieldmind",
                parameters=[str(parametres), {"jeton": LaunchConfiguration("jeton")}],
                output="screen",
            ),
        ]
    )
