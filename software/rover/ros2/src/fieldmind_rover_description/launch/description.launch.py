"""Publie la description du rover (TF statiques des capteurs) : robot_state_publisher.

    ros2 launch fieldmind_rover_description description.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    xacro = Path(get_package_share_directory("fieldmind_rover_description")) / "urdf"
    description = ParameterValue(Command(["xacro ", str(xacro / "rover.urdf.xacro")]),
                                 value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": description,
                         "use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        ),
    ])
