from glob import glob

from setuptools import setup

setup(
    name="fieldmind_bringup",
    version="0.2.0",
    packages=["fieldmind_bringup"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fieldmind_bringup"]),
        ("share/fieldmind_bringup", ["package.xml"]),
        ("share/fieldmind_bringup/launch", glob("launch/*.py")),
        ("share/fieldmind_bringup/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": []},
)
