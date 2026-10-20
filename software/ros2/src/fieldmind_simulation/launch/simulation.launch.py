"""Simulation Gazebo du rover : monde, rover, pont vers ROS 2 et contrôleur embarqué simulé.

    ros2 launch fieldmind_simulation simulation.launch.py graine:=7   # monde naturel généré
    ros2 launch fieldmind_simulation simulation.launch.py monde:=<fichier .sdf> x:=.. y:=..

Avec `graine`, le monde (relief, rivière, hameau, forêt, rochers) est généré une fois dans
~/.cache/fieldmind/mondes/<graine> et le rover est posé au départ de la piste.

Le reste de la pile (localisation, SLAM, Nav2, perception, agent) se lance avec
fieldmind_bringup, à l'identique sur le rover réel.
"""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _monde_et_depart(contexte) -> list:
    """Choisit le fichier du monde et la pose de départ (monde généré si `graine` est donnée)."""
    graine = LaunchConfiguration("graine").perform(contexte).strip()
    monde = LaunchConfiguration("monde").perform(contexte)
    pose = [LaunchConfiguration(n).perform(contexte) for n in ("x", "y", "z", "lacet")]
    description = ""
    if graine:
        from fieldmind_simulation.generateur_monde import ecrire_monde

        spectateur = LaunchConfiguration("spectateur").perform(contexte).lower() == "true"
        dossier = Path.home() / ".cache" / "fieldmind" / "mondes" / graine
        chemin = dossier / "monde.sdf"
        if not chemin.exists() or spectateur != ('name="spectateur"' in chemin.read_text()):
            ecrire_monde(int(graine), dossier, spectateur)
        monde = str(chemin)
        description = str(dossier / "description.json")
        depart = json.loads((dossier / "description.json").read_text())["depart"]
        pose = [str(depart[n]) for n in ("x", "y", "z", "lacet")]
    return [
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "-r", "--headless-rendering", monde], output="screen"
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                "rover",
                "-topic",
                "robot_description",
                "-x",
                pose[0],
                "-y",
                pose[1],
                "-z",
                pose[2],
                "-Y",
                pose[3],
            ],
            output="screen",
        ),
        # Récepteur GNSS : erreur selon le ciel visible (arbres, bâtiments du monde généré).
        Node(
            package="fieldmind_simulation",
            executable="recepteur_gnss",
            parameters=[{"use_sim_time": True, "description": description}],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    partage = Path(get_package_share_directory("fieldmind_simulation"))
    description = Path(get_package_share_directory("fieldmind_description"))
    profondeur = LaunchConfiguration("profondeur")
    urdf = ParameterValue(
        Command(
            ["xacro ", str(description / "urdf" / "rover.urdf.xacro"), " profondeur:=", profondeur]
        ),
        value_type=str,
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("graine", default_value="", description="monde naturel généré"),
            DeclareLaunchArgument("spectateur", default_value="false"),
            DeclareLaunchArgument(
                "monde", default_value=str(partage / "worlds" / "essai_plat.sdf")
            ),
            DeclareLaunchArgument("profondeur", default_value="true"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.15"),
            DeclareLaunchArgument("lacet", default_value="0.0"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(partage / "worlds")),
            # Serveur seul, rendu des capteurs sans écran (EGL) : pas d'interface Gazebo.
            OpaqueFunction(function=_monde_et_depart),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": urdf, "use_sim_time": True}],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {"config_file": str(partage / "config" / "pont_gz.yaml"), "use_sim_time": True}
                ],
                output="screen",
            ),
            Node(
                package="fieldmind_simulation",
                executable="controleur_embarque",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
