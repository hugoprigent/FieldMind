[Français](README.md) · **English**

<a id="fieldmind--prototype-final"></a>
# FieldMind — final prototype

> **Status, 23 October 2026.** This page describes the CAD design and its geometric checks. The images are views of the model. The actual rover rolls with its chassis and electronics visible; the shell is not mounted. The drone-carrying trailer has been manufactured. CAD checks do not prove mechanical load-bearing capacity. See the [results and limitations](../../docs/validation.en.md).

Fusion document **`prtotype final fieldmind`**. It contains two groups:
- `FM_Direction_V04`: the rover from the [V04 direction](../mounts/prototype/direction_v04_servo_traversant/README.en.md), with the board, steering, wheels, two masts, and sensors;
- `FM_Prototype_Final`: the onboard components and the shell.

![Final prototype](vues/01_ensemble_avant.png)

**Sides**: *left* and *right* are understood as seen from the rear of the rover, looking forward. The left is the +X side of Fusion. The `G…` shell parts are on the left, the `D…` on the right. Note: the V04 steering parts named `_D` are, however, on the +X side.

**Colors**: blue and white.
- All parts already printed (steering, wheels, lidar and camera mounts) are white in the model, as in reality.
- For the shell, the sides are white and the central band is blue.

<a id="composants-posés"></a>
## Placed components

Only hardware documented as purchased or already in place is mounted. There is no emergency stop, no main switch, no fuses, no relays.

| Component | Location | Volume (mm) | Dimensions |
| --- | --- | --- | --- |
| Raspberry Pi 4 in its case | right, behind the servo | 68 × 96 × 32 | case to be measured |
| Breadboard 830 points, GPS ATGM336H plugged on it | center | 55 × 165 × 8,5 | Elegoo |
| A7 sensor board (XIAO, GY-521, BMP280) | under the central band, **directly above the breadboard** | 40 × 60 | A7 plate |
| OLED screen 0,96" I2C 128 × 64, plugged on the A7 board | behind its window, screwed under the band by 4 M2 screws | 27,3 × 27,8 | manufacturer's plan ([LCDwiki MC096GX dimensions](https://www.lcdwiki.com/res/MC096GX/MC096GX-size-EN.jpg)) |
| Modulino Knob | next to the screen, above the breadboard; axis and cap on top, screwed under the band by 4 M3 screws | 41 × 25,36 | **official Arduino 3D model** (STEP (working reference not included), [datasheet](https://docs.arduino.cc/resources/datasheets/ABX00107-datasheet.pdf)) |
| Battery, laid lengthwise in its printed cradle | **at the front left**, against the steering servo (6,5 mm), center 63,5 mm behind the pivot axis | 82 × 68 × 51 | measured |
| 2 XL4015: 5,1 V for the Pi, 6 V for the servo | **at the front right**, mirrored relative to the battery, on their printed plate (`support_xl4015`) 5 mm apart, M3×8 screws + nuts | 23 × 54 × 15 | AZDelivery; hole spacing to be verified with the [template 1:1](exports/support_xl4015_gabarit.pdf) |
| 2 BLDC controllers (ZS-X11H) | on each side of the LiDAR mast, above the motors | 45 × 63 × 31 | stated |
| LiDAR C1 USB adapter | behind the LiDAR mast | 40 × 25 × 6 | to be measured |
| GNSS antenna | glued under the top of the hull, on the right | 25 × 25 × 8 | to be measured |
| NGP RM24 motors and rear wheels | under the board, in printed collar supports (85 mm long, 4 M8 screws); each wheel directly on its motor shaft | Ø46; Ø200 × 50 | to be measured (wheel appearance is indicative) |

<a id="roues-arrière"></a>
### Rear wheels

These are mower wheels mounted **directly on the output shaft** of the NGP RM24 motors: no separate axle or bearing, the geared motor carries the wheel. In the model:
- wheel of Ø200 × 50, at 3 mm from the motor support;
- white rim with 5 spokes, hub Ø36 on the shaft (Ø10 assumed);
- black tire with 22 chevron tread blocks, blue hubcap like the hull stripes.

This is an indicative appearance, in the style of the front wheels: diameter, width, offset, and shaft must be measured on the real wheels. The `08_roues_arriere.py` script regenerates them alone, without touching the rest.

![Rear wheel on the motor shaft](vues/06_roue_arriere.png)

<a id="support-moteur-v2-plus-léger"></a>
### V2 Motor Support, lighter

The V1 motor support (solid block of 85 mm, solid base of 10 mm) is bulky. The **V2** keeps exactly the same interfaces (stepped bore Ø46,3 / Ø42,3, shoulder Ø52, 4 Ø4 holes on the wheel side, split collar with 2 M6×25 screws, 4 M8 screws at the same board holes, hull screw notch): same fasteners, same drillings, no new collisions (it fits within the V1 envelope).

| | V1 | V2 |
| --- | ---: | ---: |
| Volume | 233,5 cm³ | 161,4 cm³ (−31 %) |
| Solid PETG (upper bound) | 297 g | 205 g |

What changed:
- **round tube** of 5,85 mm wall thickness around the motor (R 29 on the reducer, R 27 on the motor) instead of the square block; **square flange of 12 mm** on the wheel side only (Ø4 holes and shoulder);
- **two-ear collar** of 18 mm around the M6 screws, instead of a keel along the entire length; the tube remains slitted and clamped along its entire length;
- **hollowed-out sole underneath**: 6 mm pockets (4 mm skin against the board), transverse rib in the middle, solid rail of 10 mm along the tube, solid bossings Ø18 around the M8 screws.

Both versions are in the assembly (document "wheel supports fieldmind", inserted in the rover): **V1 on the left, V2 on the right**, for comparison; the other two examples exist, hidden (turn on `SR_IMP_Support_moteur_arriere_V2` on the left for an all-V2 rover). STL: `exports/stl/support_moteur_arriere_v2.stl`, printed like V1, standing on the wheel-side face.

![Motor supports V1 (left) and V2 (right), viewed from below](vues/14_support_moteur_v1_v2.png)

<a id="mâts-en-profilés-de-90-mm"></a>
### 90 mm profile masts

The two available profiles are 90 mm. The cameras are therefore 5 mm lower (optical centers at 265 mm from the ground). The lidar support is 60 mm higher, in one piece, to keep the lidar at the same height (laser plane at 322 mm). Nothing is screwed from underneath the board:
- an octagonal column of 56 mm (the width of the C1 plate) which ends at the bottom with a **31 mm socket** slipped onto the top of the profile, like the V1 support;
- two tabs in the front and rear grooves;
- **2 M8×16 side screws** (left and right), 23 mm below the top of the profile, in **M8 T-nuts** in the grooves;
- the C1 screws are tightened with a 2 Allen key, through the grooves of the four corners.

The profile protrudes only 5 mm above the hull: the M8 screws are therefore under the roof, 2,6 mm below the edge of the sides. The complete mast (support and lidar) is mounted before placing the hull. Its opening around the mast becomes **61 × 61 mm** (part C4), and the hull goes over the lidar. The support is 2,5 mm from the edge of the opening.

Verified: no collision, webcam field and lidar plane clear.

![90 mm profile masts](vues/08_mats_90.png)
![Lidar support socketed on the 90 mm profile, side M8 screw](vues/11_mat_lidar_manchon.png)
![Support viewed from below: socket, tabs, M8 holes, C1 screw grooves](vues/13_support_lidar_dessous.png)
![The hull goes over the lidar mast](vues/12_mat_lidar_coque.png)

Why this layout:
- the battery at the front left and the XL4015 mirrored at the front right: weight on the steering wheels and the center of gravity in the middle width-wise (see below);
- the controllers are as close as possible to their motors;
- the Pi is near the vents on its side;
- the A7 board is above the breadboard, and the screen and Knob remain on top.

<a id="sg90-à-la-place-du-dt996"></a>
### SG90 instead of DT996

The DT996 for steering burned out: an **SG90** takes its place, with two printed parts (Fusion document "servo SG90 steering fieldmind", `22_servo_sg90.py`):
- **`support_sg90.stl`** (20,4 × 55 × 9 mm): 3 mm plate on the board, screwed into the 4 holes of the DT996 ears (S1–S4) by 4 **M2,5×16 screws + nut under the board**, with a plug that fits into the DT996 cutout. The SG90 ears sit in the recess and are screwed by 2 **M2,5×10** screws into the Ø2,2 pilot holes. The SG90 axis aligns exactly with that of the DT996.
- **`palonnier_sg90.stl`** (37 × 55,4 × 18,9 mm): the rover's horn (same connecting rods), star imprint blocked, with a boss carrying the imprint of the **SG90 arm head 1** (no pins). An **M2,5 screw + large washer**, from below through the central access, passes through the horn and head and screws into the servo axis (approx. M2,5×18: measure).
- Checked: no contact with the board or support, at neutral and at ±31°.
- The SG90 is much less robust (approx. 1,8 kg·cm vs. about fifteen for the DT996): it may struggle to steer while stopped on a high-grip surface.


<a id="planche-de-10-mm"></a>
### 10 mm board

The board is now **10 mm** (two 5 mm plywood sheets glued). The underside remains 130 mm from the ground, so nothing moves under the board (steering, wheels, motor mounts, chassis). Everything placed on top rises by 5 mm (body, battery, electronics, masts: lidar laser plate at 327 mm).
- The servo is held by its ears on top: it rises by 5 mm, so the **horn must be reprinted** with a column 5 mm longer (`exports/stl/palonnier_planche_10mm.stl`, 37 × 53,5 × 14 mm), so that the connecting rods remain flat.
- The screws passing through the board are 5 mm longer (see [VISSERIE.md](VISSERIE.en.md)).
- Checked: no collision at neutral and at ±31°, front tires 11,7 mm from the body at maximum steering. Done by `21_planche_10mm.py`, in place.

<a id="équilibrage"></a>
### Balancing

All the weight was at the back: rear motors and wheels (1,8 kg) are on the rear axle, and the battery was just in front. It has been moved to the front left, and the XL4015 to the front right to compensate. `13_centre_de_gravite.py` calculates the center of gravity in Fusion. Printed parts and wood are weighed by volume, profiles by meter, and steel fasteners by weight. The rest is estimated in [masses.json](masses.json): battery 600 g, each motor 600 g, each rear wheel 300 g, Pi 150 g.

| Rover alone (5,8 kg) | Front | Rear | Left | Right |
| --- | ---: | ---: | ---: | ---: |
| Battery centered, ahead of the rear axle (front) | 34,8 % | 65,2 % | 49,6 % | 50,4 % |
| Battery at front left, XL4015 at front right | **42,0 %** | **58,0 %** | **50,6 %** | **49,4 %** |
| Same, with trailer attached (200 g on the tongue) | 39,2 % | 60,8 % | 50,5 % | 49,5 % |

- **Left-right**: the center of gravity is 2 mm from the axle, on a track width of 390 mm. The offset is smaller than the uncertainty in the estimated masses.
- **Front-rear**: 42/58. The motors and rear wheels are on the rear axle: approximately 650 g of ballast would be needed at the front to reach 50/50. For rear-wheel drive, 40/60 is a good split: grip on the drive wheels, and enough load on the steering wheels.
- **To do**: weigh the battery, a motor with its wheel, and the Pi. Enter the values in `masses.json` (with `"pese": true`), then re-run the script.

![Battery at front left, XL4015 at front right](vues/10_equilibrage_dessus.png)

<a id="coque"></a>
## Hull

![Hull top view](vues/02_coque_dessus.png)

It follows the look of the first FieldMind rendering: a white body with chamfered edges, with a blue stripe running from the nose to the rear.

- **Hidden board**:
  - the sides align with the edges of the board, leaving no visible wood rim;
  - the front and rear faces overhang it and descend 5 mm along its edge;
  - on the sides, only the 5 mm edge remains visible under the hull. It cannot be covered due to lack of space between the board and the wheels: the turned tires pass 8,6 mm from the side.
- **Template**: 300 × 506 mm, 85 mm high above the board, i.e., 220 mm from the ground; 20 mm chamfers on the top edges.
- **Sloped nose**: a front face of 40 mm with the grille, then a slope that goes under the camera bar.
- **Central blue stripe** (100 mm wide), from the nose to the rear:
  - the passage of the two profiles;
  - behind each, a **cable passage of 24 × 14 mm**, enough for a USB-A plug and for the lidar connector;
  - the screen window and the Knob hole, above the breadboard.
- **White sides**: ears opposite the Pi and controllers (the XL4015s, at the front right, are between the nose grille and the first ear); "FIELDMIND" inlaid in blue (see below).
- **No visible screws**: everything is screwed from the inside, and the shell sits on the board from underneath.

![Side and inscription](vues/04_flanc_gauche.png)

<a id="incrustation-fieldmind"></a>
### FIELDMIND Inlay

Letters in the opposite color, glued into a 1,2 mm pocket (half the thickness of the 2,4 mm wall): the wall remains solid, and the inside of the letters (D) sits at the bottom of the pocket.

| Where | Text | Insert | Size | STL |
| --- | --- | --- | --- | --- |
| Left side and right side, section 3 (D3, G3), between the ears and the bevel | FIELDMIND | blue on white, 9 letters per side | 109 × 13 mm | `incrustation_FIELDMIND_G.stl`, `_D.stl` |

- Print the letters flat, visible face against the build plate, with an **X-Y compensation of -0,1 mm** ("X-Y contour compensation" in Bambu Studio) so they fit into the pockets. Glue them with cyanoacrylate glue.
- With an AMS lite, you can also print the part and its letters in one go in two colors.
- An ASUS on the C2 strip was tried and then removed. The same script generates any text or logo.
- Made by `20_incrustations.py`, placed on the shell (after `02_coque.py`).

![FIELDMIND inlaid](vues/15_incrustations.png)
![FIELDMIND on the side](vues/16_incrustation_fieldmind.png)

<a id="couleurs--variantes-testées"></a>
### Colors: tested variants

![Four shell color combinations](vues/18_couleurs_coque.png)

A (current): white sides, blue stripe. B: blue sides, white stripe. C: blue nose and rear. D: all blue. If changed, the letters change color along with the part carrying them.

<a id="pièces-à-imprimer-stl-déjà-orientés-sans-support"></a>
### Parts to print (STLs already oriented, no support needed)

| File | Color | Placed on the build plate |
| --- | --- | --- |
| `coque_C1_nez` | blue | standing on its front face (grid) |
| `coque_C2_bande_avant` | blue | top side against the build plate |
| `coque_C3_bande_ecran_knob` | blue | top side against the build plate (bosses of card A7 facing up) |
| `coque_C4_bande_arriere_lidar` | blue | top side against the build plate |
| `coque_G1…G4` (left side), `coque_D1…D4` (right side) | white | lying on their outer wall: all walls are vertical |
| `berceau_batterie` | white | bottom against the build plate |
| `capuchon_knob` | white | standing, bore in D against the build plate |

Dimensions on the build plate (X × Y × height) and approximate filament mass: solid parts at 100 %, so an upper bound.

| Part | Color | Dimensions (mm) | Filament |
| --- | --- | --- | ---: |
| `coque_C1_nez` | blue | 100 × 90 × 103 | 36 g |
| `coque_C2_bande_avant` | blue | 100 × 133 × 7,9 | 41 g |
| `coque_C3_bande_ecran_knob` | blue | 100 × 133 × 16,4 | 43 g |
| `coque_C4_bande_arriere_lidar` | blue | 100 × 147 × 90 | 62 g |
| `berceau_batterie` | white | 75 × 89 × 30 | 66 g |
| `capuchon_knob` | white | 22 × 22 × 11 | 5 g |
| `coque_D1_nez_droit` | white | 90 × 93 × 114 | 85 g |
| `coque_D2_flanc_avant_droit` | white | 85 × 133 × 114 | 97 g |
| `coque_D3_flanc_milieu_droit` | white | 85 × 133 × 114 | 95 g |
| `coque_D4_flanc_arriere_droit` | white | 90 × 147 × 114 | 128 g |
| `coque_G1_nez_gauche` | white | 90 × 93 × 114 | 85 g |
| `coque_G2_flanc_avant_gauche` | white | 85 × 133 × 114 | 97 g |
| `coque_G3_flanc_milieu_gauche` | white | 85 × 133 × 114 | 95 g |
| `coque_G4_flanc_arriere_gauche` | white | 90 × 147 × 114 | 128 g |

- **Blue (PLA)**: approx. 182 g.
- **White (PETG)**: approx. 865 g.
- The largest part measures 147 mm: everything fits on the A1 mini's 180 mm build plate.

Settings: 0,2 mm layers, 3 walls, 20% infill. The recessed inscription is then against the build plate, which prints well.

<a id="comment-les-pièces-tiennent-ensemble-toutes-les-vis-à-lintérieur"></a>
### How the parts fit together (all screws inside)

Assemble the shell **upside down on the bed**, flip it over, then screw it onto the board from underneath.

- **Central strip on the sides**:
  - each side has a 6,5 mm rim under the strip's edge;
  - the strip's bosses fit into holes in this rim;
  - each boss receives an M3 insert; screw in from below an **M3×6 screw with a wide washer** (DIN 9021, Ø9) that presses under the rim;
  - at the front, the nose is screwed in the same way behind its front face, and a tab passes under band C2.
- **Sections between each other**: at each cut, two ribs of 8 mm face each other, one per section. Two **M3×12 bolts + nuts** per side and per cut.
- **Shell on the board**: 10 blocks at the base of the shell.
  - Each block has a **hexagonal slot** into which an **M6 nut** is inserted from the inside. Below the nut remains a base of 3 mm.
  - An **M6×16 screw and its washer** pass under the board and tighten into the nut, which the slot prevents from turning.
  - There are 3 per side, 2 at the front and 2 at the rear.
- **A7 board**: screwed under band C3 by 4 M3×8 screws, directly into the plastic (Ø2,6 holes).
- **Screen**: plugged into the A7 board and, additionally, screwed under band C3 by 4 M2×4 screws into Ø4 bosses (Ø1,6 holes). Its panel comes to 0,3 mm below the band. The window is the actual displayed area plus 0,6 mm, beveled at 30° towards the top.
![Screen window and Knob cap](vues/05_ecran_knob.png)

- **Knob**: screwed under band C3 by 4 M3×12 screws into Ø6 bosses (Ø2,6 holes). Its Ø6 shaft passes through the band via an Ø7 hole and protrudes by 5,5 mm. The **printed cap** slides onto the flat and remains 1,5 mm from the shell, to allow the push-button travel.

<a id="visserie"></a>
### Fasteners

Complete list of the rover and trailer, by size: [VISSERIE.md](VISSERIE.en.md).

| What | Quantity | Where |
| --- | ---: | --- |
| M3 heat-set insert (outer Ø 4,5 to 4,6, length 5,7 max) | 20 | Blue band bosses, Ø4 holes × 5,5 |
| M3×6 screw + large DIN 9021 washer (Ø9) | 20 | Bands on the edges of the sides, underneath |
| M3×12 bolt + nut + washer | 12 | Ribs between sections: 2 per side and per cut |
| M6×16 screw + washer | 10 | Shell on the board, underneath |
| M6 nut (10 mm across flats) | 10 | Slid into the hexagonal slots of the shell |
| M3×8 screw | 4 | A7 board under C3 strip, directly into the plastic |
| M2×4 screw | 4 | Screen under C3 strip, directly into the plastic |
| M3×12 screw | 4 | Modulino Knob under C3 strip, directly into the plastic |
| M3×10 screw + washer | 4 | Battery cradle, from below the board, into 4 M3 nuts recessed in its bottom |
| 20 mm self-adhesive strap | 2 | Battery in its cradle |
| Double-sided adhesive | — | Components on the board |

<a id="montage"></a>
### Assembly

1. **Board** :
   - drill the new holes (diagram below): Ø6,5 for the shell, Ø3,5 for the cradle;
   - slide the 4 M3 nuts into the bottom of the cradle, then screw it from below the board (M3×10 + washer).
2. **Components**: place them with double-sided adhesive or self-adhesive straps in the model's positions, then wire them.
3. **Inserts**: place them in the bosses of the strips (from below).
4. **Shell upside down on the table** :
   - bolt the segments of each side;
   - place the strips;
   - screw the strips from below into the flanges;
   - attach the A7 board, the screen (on its support, then its 4 M2 screws) and the Knob under C3;
   - push the cap onto the Knob shaft.
5. **Masts** :
   - remove the camera bar (2 M8 screws). The stereo calibration does not change, since both cameras remain on their bar;
   - the lidar mount is placed with the C1 already screwed onto it. Two M8 T-nuts are screwed two turns onto the lateral M8×16 screws, lengthwise. Slide the mount onto the top of the profile: the nuts enter the left and right grooves, the tabs into the front and rear grooves. Lower it until it hits the stop, then tighten the two screws.
6. **Hull** :
   - slide the 10 M6 nuts into their slots;
   - turn the hull over and lower it onto the two profiles: it goes over the lidar;
   - pass the cables through the passages;
   - screw the 10 M6 screws from underneath the board.
7. **Reassemble** the camera bar.

To access the interior: remove the camera bar, unplug the lidar cable from its adapter, then unscrew the 10 M6 screws under the board. The hull lifts off as a single piece, over the lidar. To charge the battery without opening everything, provide a charging extension up to a port on the rear face: you must first know its connector.

<a id="vérifications-dans-fusion"></a>
## Verifications in Fusion

Everything is checked by `03_verifier.py` (results in `verification_prototype_final.json`), with the steering at neutral then turned to ±31° :

- **Collisions**: none between the 12 hull parts, the cradle, the components, and the rover.
- **Front tires**: 40,7 mm from the sidewall at neutral, **8,6 mm** at the closest point in maximum steering (rims: 14,9 mm).
- **Rear wheels**: 3 mm from the sidewall and 3 mm from the motor mount (assumed wheel width, see below).
- **Camera bar**: at 7,8 mm from the nose. **Lidar mount**: at 2,5 mm from the edge of its opening in the hull; M8 screw heads at 2,6 mm below the side edges.
- **Field of view of the two webcams** (75° × 45°): no ray touches the rover or the hull.
- **Lidar laser plane** (360°, ±1,5°): clear.
- **Parts**: each one piece, all under 180 mm.

<a id="stéréo--lécartement-est-il-bon-"></a>
## Stereo: is the baseline correct?

Yes. In the model:
- the two optical centers are **120,0 mm** apart, at the same height (270 mm from the ground);
- the axes are **parallel** and tilted 10° towards the ground;
- nothing from the rover or the hull enters the field of view.

With the horizontal field of view of 75° from UGREEN:

| | 640 × 360 | 1280 × 720 |
| --- | ---: | ---: |
| Focal length | 417 px | 834 px |
| Measurable depth from (`numDisparities` = 96 / 128) | 0,52 / 0,39 m | 1,04 / 0,78 m (0,52 m with 192) |
| Error at 1 m (0,25 to 0,5 px disparity) | ±0,5 to 1 cm | ±0,2 to 0,5 cm |
| Error at 2 m | ±2 to 4 cm | ±1 to 2 cm |
| Error at 4 m | ±8 to 16 cm | ±4 to 8 cm |
| Error at 6 m | ±18 to 36 cm | ±9 to 18 cm |

- **Common field of view for both cameras**: 92 % of the image at 1 m, 98 % at 4 m.
- **Ground visible** from 0,42 m in front of the lenses; the horizon is at 29 % from the top of the image.
- **Why 120 mm**: it is the right compromise to see obstacles from 0,5 to 4 m.
  - At 60 mm (the simulated camera), the error at 4 m doubles.
  - At 200 mm, it drops by a third, but nothing is measured before 0,87 m. The bar would then be 250 mm: it can no longer be printed in one piece and loses its rigidity.
- **Starting advice**: 640 × 360, `numDisparities` = 128, at approximately 5 frames/s.
- **The real weak point** is elsewhere: the two webcams are not synchronized.
  - At 0,5 rad/s, a delay of 16 ms between the two images distorts the disparity by 3,3 px. This is 27 % of the disparity of an obstacle at 4 m.
  - Therefore, you must set the same frequency and exposure on both cameras, match the images by timestamp, and only rely on depth when moving straight or when stopped.
  - A wider base would hardly change anything.
- **Calibrate using a checkerboard** once everything is tightened, then recalibrate if a camera has moved on its screw.

<a id="plan-de-perçage"></a>
## Drilling plan

[plan_percages/plan_percages.pdf](plan_percages/plan_percages.pdf) includes all the holes in the board: direction, sensors and brackets, plus the new holes.

| Reference | Ø | For |
| --- | ---: | --- |
| K1–K10 | 6,5 | Hull mounting: M6×16 screws from below, M6 nuts inside the hull |
| B1–B4 | 3,5 | Battery cradle, at the front left: M3×10 screws from below |
| X1–X2 | 3,5 | XL4015 plate, at the front right: M3×12 screw + washer + nut |
| A1–A2 | 6,5 | Trailer hitch eye: M6×20 screws from above, nuts inside the eye |
| C1 | 8 | Routing for the trailer flap cables, towards the inside of the hull |

![Drilling plan](plan_percages/plan_percages.png)

<a id="remorque-porte-drone"></a>
## Drone carrier trailer

A two-wheeled trailer is hitched behind the rover via an eye and a single M6 screw tightened by hand:
- a wooden platform of 300 × 380 mm with blue edges and a landing cross;
- two side flaps, each driven by an SG90, which widen it to 530 mm to place the drone;
- the same wheels as the front, each free on two 5×16×5 bearings.

It has **its own Fusion document**, `remorque fieldmind`, inserted by reference in this document: we rebuild it without touching the rover. The hitch plate remains here. Everything is described in [remorque/README.md](remorque/README.en.md).

![Rover and trailer](remorque/vues/01_ensemble.png)

<a id="à-mesurer-ou-à-valider"></a>
## To measure or validate

- **Pi enclosure**: assumed 68 × 96 × 32 mm.
- **Rear wheels**: assumed 50 mm wide, just after the motor mount. They are then 3 mm from the side. If they are closer to the mount, say so: we move the rear sides inward.
- **Screen**: check with calipers the hole spacing (23,3 × 23,8 on the manufacturer's drawing) and the height of its female support (8,5 mm) before printing C3. Everything is adjustable at the top of `02_coque.py` (`OLED_TROUS`, `OLED_ZPCB`, `ECRAN`).
- **Inserts**: test on a scrap piece before the series.
- **Masses**: weigh the battery, a motor with its wheel, and the Pi (see "Balancing").

## Scripts

They go through the local Fusion MCP server (client `../mounts/prototype/direction_v04_servo_traversant/scripts/fusion_mcp.py`).

| Script | Role |
| --- | --- |
| `pf_helpers.py` | Common tools (document "prtotype final fieldmind", group `FM_Prototype_Final`) |
| `01_composants.py` | Embedded components (screen based on manufacturer's drawing, Knob imported from its official STEP), battery cradle, Knob cap; re-executable |
| `02_coque.py` | Full shell: volume, openings (beveled screen window, 61 × 61 around the lidar mast), cut into 12 pieces, edges, bosses (A7 card, M2 screen, M3 Knob), ribs, M6 slots, inscription; re-executable (about 10 minutes) |
| `03_verifier.py` | Collisions, clearances, camera field of view, lidar plane, single-piece parts → JSON |
| `04_exporter.py` | Oriented STLs for printing, STEP + F3D from the group, manifesto |
| `05_percages.py` | Holes in the hull, cradle, tow shackle, and trailer cables in the board, survey → `plan_percages.json` |
| `06_plan_percages.py` | Outside Fusion: PDF drilling plan (plan, coordinates, template 1:1) |
| `07_apparences.py` | Colors: printed parts in white, hull stripe in blue |
| `08_roues_arriere.py` | Rear wheels and motors only (wheels on the motor shafts), without redoing other components |
| `09_plans_atelier.py` | Outside Fusion: workshop plans in one PDF (trailer cutting and drilling, flaps and rover, full drill bits) |
| `10_support_xl4015.py` | Plate for the two XL4015s and the modules on top, without redoing other components; STL and board holes |
| `11_gabarit_xl4015.py` | Outside Fusion: template 1:1 to verify the XL4015 hole spacing |
| `12_mats_profiles_90.py` | 90 mm profiles (both available): cameras 5 mm lower; lidar support redone 60 mm higher, fitted onto the profile, 2 M8 side screws in T-nuts (lidar at the same height); STL of the support |
| `13_centre_de_gravite.py` | Center of gravity and front-rear and left-right distribution, based on `masses.json` → `centre_de_gravite.json` (read-only) |
| `14_batterie_avant.py` | Battery and cradle only (at the front left), without redoing other components; board holes, STL of the cradle |
| `15_ouverture_coque_lidar.py` | Hull opening enlarged to 61 × 61 around the lidar mast, in place on C4: the hull goes over the mounted mast |
| `16_supports_moteurs.py` | Rear motor mounts enlarged: motor held on 85 mm, clamped by 2 M6, base with 4 M8 screws; board holes, STL |
| `17_supports_pivot.py` | Reinforcement test for pivot supports (not selected: it is the axle that bends) |
| `18_document_supports.py` | Separate document "fieldmind wheel supports" (motor supports), saved and inserted by reference in the rover; STL |
| `19_fusees_renfort.py` | Front axles reinforced with a gusset under the bridge (L corner), in place in the rover; STL |
| `20_incrustations.py` | FIELDMIND inlay on the sides: 1,2 mm pockets and letters in the opposite color; STL |
| `21_planche_10mm.py` | 10 mm board: d4_planche_ep parameter, parts placed on it raised by 5 mm (Move function), without rebuilding |
| `20_support_moteur_v2.py` | V2 lightweight motor mount (−31 %, same interfaces), alongside V1 in the mounts document; updated rover reference; STL |
| `22_servo_sg90.py` | SG90 instead of DT996: mount on the board and horn matching the SG90 head footprint, separate document; control at ±31°; STL |
| `pf_helpers.py` → `importer_step` | Import of a real STEP model, copied in place within the component (Knob, SG90, horn) |

The Fusion document is not saved by the scripts.
