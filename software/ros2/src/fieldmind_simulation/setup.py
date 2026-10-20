from glob import glob

from setuptools import setup

setup(
    name="fieldmind_simulation",
    version="0.2.0",
    packages=["fieldmind_simulation"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fieldmind_simulation"]),
        ("share/fieldmind_simulation", ["package.xml", "CREDITS.md"]),
        ("share/fieldmind_simulation/launch", glob("launch/*.py")),
        ("share/fieldmind_simulation/config", glob("config/*")),
        ("share/fieldmind_simulation/worlds", glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "controleur_embarque = fieldmind_simulation.controleur_embarque:main",
            "generer_monde = fieldmind_simulation.generateur_monde:main",
            "recepteur_gnss = fieldmind_simulation.recepteur_gnss:main",
        ]
    },
)
