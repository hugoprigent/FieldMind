[Français](README.md) · **English**

# Prototype electronics

The [current wiring](electronics/CONNEXIONS.md) (in French) and the [C5 firmware](../software/esp32/esp_c5/README.en.md) are the starting point for understanding the assembly. The GNSS is connected to the Raspberry Pi; the XIAO ESP32-C5 links the I²C sensors, the display, the servo and the commands of the two motor drivers.

The A7 sensor board is a wiring study on perfboard, drawn in KiCad; its inner layers represent insulated wires that cross each other, not a multilayer printed circuit board to order.

| Source | Use and status |
| --- | --- |
| [`CONNEXIONS.md`](electronics/CONNEXIONS.md) | Recorded connections and test status; takes precedence over older studies |
| [`CONNEXIONS_MOTEUR_ZS_X11D1.md`](electronics/CONNEXIONS_MOTEUR_ZS_X11D1.md) | Motor driver interfaces |
| [`Carte A7`](electronics/carte_capteurs/cad/perfboard_c5_a7/README.md) | Schematic, footprints, wire routing and 3D view |
| [`Guide A7`](electronics/carte_capteurs/perfboard_une_plaque_c5_a7.md) | Fabrication study to be adapted to the current connections |
| [`Schéma système`](electronics/systeme_rover/README.md) | Earlier power and shutdown study, not validated physically and different from some current connections |

These documents are in French.

The A7 and system drawings are not an exact picture of all current connections. The pinout and power supply have changed since these studies. Read the dates and statuses, and check the assembly before powering up. The fuses and a physical traction cut-off are still to be sized and validated on the rover.
