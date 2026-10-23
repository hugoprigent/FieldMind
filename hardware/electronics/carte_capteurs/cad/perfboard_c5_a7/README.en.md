[Français](README.md) · **English**

<a id="projet-courant-a7--une-perfboard-4--6-cm-tous-les-fils-dessous"></a>
# A7 current project — a perfboard 4 × 6 cm, all wires underneath

Open `fieldmind_c5_a7.kicad_pro` in KiCad 10. All sources have been saved by KiCad 10.0.6.

- **Schematic**: one root sheet (XIAO, GNSS J4, J6 and J8 extensions, Modulino Knob J7) and one sub-sheet, "I2C Sensors and display".
- **PCB**:
  - **F.Cu**: only the solder bridges of the XIAO, display side;
  - **B.Cu**: bare wires on the underside;
  - **In1.Cu / In2.Cu**: insulated wires on the underside. These are not real layers: they separate crossing wires for DRC purposes;
  - `PL1` carries the 280 holes; `Cmts.User` carries the wire numbers;
  - the 3D view opens with Alt+3.
- Do not perform "Update PCB from schematic" without caution: the pads of `PL1` have their nets assigned manually.

Documentation: `../../perfboard_une_plaque_c5_a7.md`. Regeneration: `scripts/generer_c5_a7.py --force`, which **overwrites any manual edits**.
