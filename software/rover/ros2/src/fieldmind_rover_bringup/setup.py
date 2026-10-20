from glob import glob

from setuptools import setup

setup(
    name="fieldmind_rover_bringup",
    version="1.0.0",
    packages=["fieldmind_rover_bringup"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fieldmind_rover_bringup"]),
        ("share/fieldmind_rover_bringup", ["package.xml"]),
        ("share/fieldmind_rover_bringup/launch", glob("launch/*.py")),
        ("share/fieldmind_rover_bringup/config", glob("config/*")),
        ("share/fieldmind_rover_bringup/rviz", glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Lancements ROS 2 du rover FieldMind réel",
    license="Propriétaire",
    entry_points={"console_scripts": []},
)
