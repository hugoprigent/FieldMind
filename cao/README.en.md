[Français](README.md) · **English**

# Mechanical design

The current prototype uses a 10 mm plywood base, steering by a central servo, printed front wheels with rigid rims and lugged TPU tires, and rear gear motors salvaged from a robotic lawn mower.

- [`prototype_final/`](prototype_final/README.en.md): shell, mounts, reinforcements, drawings, fasteners and current parts. The printable parts are in [`exports/stl/`](prototype_final/exports/stl/).
- [`direction V04`](mounts/prototype/direction_v04_servo_traversant/README.en.md): steering principle, wheels and the history of its adjustments. The final prototype brings later changes, including the 10 mm board and the reinforcements; use its updated parts first.
- [`remorque`](prototype_final/remorque/README.en.md): design of the trailer, still being prepared physically.

These three documents are in French.

## Fusion sources

The scripts require Autodesk Fusion and an open document matching the model they modify. Set the `FIELDMIND_ROOT` environment variable to the local root of this repository, or run from that root. These are scripts that build and modify the working model; they do not replace a complete stand-alone assembly and may modify the open document.

External STEP files and the assemblies that incorporate them are not distributed in this edition, so as not to attribute an unverified license to them. To rebuild those parts, the references are:

- Arduino Modulino Knob: [official documentation and CAD](https://docs.arduino.cc/hardware/modulino-knob/).
- SG90: [FreeCAD library, Servos/SG-90](https://github.com/FreeCAD/FreeCAD-library/tree/master/Electrical%20Parts/Servos/SG-90).
- RPLIDAR C1: manufacturer geometry to be obtained from SLAMTEC, under its terms.

The STL files of FieldMind parts are available directly. Check the fit and dimensions of the real hardware before fabrication; a geometric validation is not a mechanical validation under load.
