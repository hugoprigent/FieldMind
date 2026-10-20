from setuptools import setup

setup(
    name="fieldmind_perception",
    version="0.1.0",
    packages=["fieldmind_perception"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fieldmind_perception"]),
        ("share/fieldmind_perception", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "traversabilite = fieldmind_perception.traversabilite:main",
        ]
    },
)
