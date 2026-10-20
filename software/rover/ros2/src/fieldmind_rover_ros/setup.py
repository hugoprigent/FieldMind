from setuptools import setup

setup(
    name="fieldmind_rover_ros",
    version="1.0.0",
    packages=["fieldmind_rover_ros"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fieldmind_rover_ros"]),
        ("share/fieldmind_rover_ros", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Pont ROS 2 du rover FieldMind (client de /ws/ros)",
    license="Propriétaire",
    entry_points={"console_scripts": ["pont = fieldmind_rover_ros.pont:main"]},
)
