"""slam_toolbox en ligne, asynchrone : carte /map et TF map → odom à partir de /scan et /odom.

    ros2 launch fieldmind_rover_bringup slam.launch.py

Enregistrer la carte : `ros2 run nav2_map_server map_saver_cli -f ~/carte` (si nav2_map_server
est installé), ou le service /slam_toolbox/save_map.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory("fieldmind_rover_bringup")) / "config"
    slam_toolbox = (Path(get_package_share_directory("slam_toolbox")) / "launch"
                    / "online_async_launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("slam_params_file", default_value=str(config / "slam.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        # Le lancement de slam_toolbox gère le cycle de vie du nœud (configure puis active).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_toolbox)),
            launch_arguments={
                "slam_params_file": LaunchConfiguration("slam_params_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": "true",
            }.items(),
        ),
    ])
