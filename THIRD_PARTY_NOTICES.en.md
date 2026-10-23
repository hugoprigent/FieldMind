[Français](THIRD_PARTY_NOTICES.md) · **English**

# Credits and third-party components

## Bundled web code

- **three.js**: code distributed in `software/interface/web/vendor/three/` and `software/rover/fieldmind/web/vendor/three/`, under the MIT license. Each copy retains its `LICENSE` file. Supporting libraries, including the meshoptimizer decoder, retain their notices in the source files.
- Other Python, Arduino, ROS and JavaScript libraries are installed through their package managers. They are not incorporated into FieldMind's original code. Requested versions are listed in the dependency files and PlatformIO configuration.

## Simulation and mapping

- The ROS 2 worlds reference **Open Robotics** Gazebo Fuel assets downloaded at runtime. Credits and CC0 / CC BY 4.0 licenses are detailed in [`CREDITS.md`](software/ros2/src/fieldmind_simulation/CREDITS.en.md).
- The station can display **OpenStreetMap** maps, with attribution shown in the interface. These basemaps require an Internet connection.
- The `software/interface/web/modeles/rover_v3.glb` model is the earlier FieldMind concept, exported for the demo and distinct from the current physical prototype.

## Electronics design

Schematics and footprints use symbols from the official **KiCad** libraries and simplified blocks and models created for this prototype. The [KiCad library license](https://www.kicad.org/libraries/license/) applies to elements from those libraries; no license change is claimed here.

## CAD and references

The printable parts and their construction scripts were designed for FieldMind. Earlier studies consulted **OpenRC Truggy, Daniel Norée, CC BY 3.0**; downloaded reference archives are not included in this edition.

Arduino Modulino Knob component STEP files, the FreeCAD SG90 library, manufacturer lidar models and third-party mounts used in studies are not redistributed in this copy. Complete assemblies that may incorporate these geometries are also excluded until their redistribution has been checked. Sources to locate are listed in [`cao/README.en.md`](cao/README.en.md).

## AI models

Configuration files offer Qwen / NVIDIA, RoboBrain, D-FINE, Depth Anything and SAM models. Their weights are not included. Each download remains subject to its publisher's terms; listing an identifier in the configuration does not imply a common license or validation on the rover.

## Experimental drone protocol

`software/rover/fieldmind/drone/cooleerx.py` contains an experimental encoder written for FieldMind,
based on offline analysis of the third-party HD dragon 1.1.0 application. No binaries,
assets or source code from that application are included. The frames are not validated on the
real drone; publishing them does not authorize reuse of the application or its
components.
