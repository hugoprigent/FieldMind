from setuptools import setup

setup(
    name="fieldmind_pont",
    version="0.1.0",
    packages=["fieldmind_pont"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fieldmind_pont"]),
        ("share/fieldmind_pont", ["package.xml"]),
        ("share/fieldmind_pont/config", ["config/pont.yaml"]),
        ("share/fieldmind_pont/launch", ["launch/pont.launch.py"]),
    ],
    install_requires=["setuptools", "websockets"],
    entry_points={"console_scripts": ["pont = fieldmind_pont.noeud:main"]},
)
