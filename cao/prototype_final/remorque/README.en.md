[Français](README.md) · **English**

<a id="fieldmind--remorque-porte-drone"></a>
# FieldMind — drone trailer

> **Status, 23 October 2026.** The trailer was built from these [printable parts](exports/stl/); its flaps are controlled by servos (ROVER_4 firmware). The images and controls below refer to the model. No actual drone transport or landing on this trailer has been validated.

The trailer has its own Fusion document, **`remorque fieldmind`**, inserted by reference into the rover document **`prtotype final fieldmind`** (see [Fusion Documents and scripts](#documents-fusion-et-scripts)). The hitch plate on the rover side remains in the rover document (`PF_IMP_Attelage`).

It is a wooden platform on two wheels, hitched behind the rover. The drone lands on the painted cross in the center. Two side flaps, each driven by an SG90, fold down to enlarge the landing area.

![Rover and hitched trailer, flaps folded (road position)](vues/01_ensemble.png)

<a id="ce-qui-la-compose"></a>
## What it is made of

- **Wooden board**: 300 × 380 × 5 mm, same width and wood as the rover's board.
  - The two front corners are cut at 45° over 50 mm: the trailer can steer further before touching the rover's rear wheels.
  - It is positioned 13,5 mm higher than the rover's board, so that the deployed flaps, flush with the platform, pass over the wheels.
- **Blue edges at the front and rear** (4 pieces):
  - they hide the wood edge and protrude 5,5 mm above the platform;
  - they are held by M3×10 screws placed from underneath, into nuts recessed in the lip: 1 per front edge (in the cut corner), 2 per rear edge. **Glue them additionally** (cyanoacrylate or wood glue on the lip): the front edges no longer carry the tow bar.
- **Blue cheeks on the sides** (4 pieces), where the flaps deploy:
  - **no lip**: they hide the wood edge **flush with the platform**, so that the platform and deployed flaps form a single flat surface;
  - their sole passes under the board, pinched by the servo housings, wheel supports, and rear blocks;
  - these parts are held by **domed head** screws placed from above, **without countersinking** in the wood: **M6×16 for the wheel supports**, which carry the trailer, M3×16 for the rest. The head rests on the platform (1,7 mm high for M3, 3,3 mm for M6), and the nut is recessed under the part's arm.
- **Landing cross**, to be painted in the center:
  - 200 × 200 mm, arms of 36 mm;
  - mate and dark (blue or black) on the light wood, for good contrast as seen by the drone camera.
- **Two wheels identical to the rover's front wheels** :
  - same rim, same 90A TPU tire, same screwed-on flange;
  - not connected to each other: each rotates freely on its screw, without a differential;
  - placed 20 mm behind the center of the plate: the trailer presses slightly on the hitch.
- **Wheel supports** (white), screwed under the board with **4 M6×16 countersunk head screws** and countersunk M6 nuts: 2 near the edge, through the sole of the cheek, and 2 towards the middle of the board, where the support arm extends (up to 75 mm from the axis).
- **Side flaps**: see below.
- **Hitch**: clevis under the rover, A-frame towbar, one M10 screw passing through everything, nut underneath. The towbar is held under the board by **4 M6×25 countersunk head screws**, from above (x ±20 and ±52, 25 mm behind the front edge), with countersunk M6 nuts under the towbar (`09_timon_m6.py`).

![Folded flaps, top view](vues/02_plateau.png)

<a id="volets-datterrissage"></a>
## Landing flaps

![Folded flaps](vues/06_volets_deplies.png)

![Unfolded flaps: side rails for transport](vues/07_volets_replies.png)

- **A plywood panel on each side**: 110 × 316,5 × 5 mm, in the same wood as the board.
- **Folded** :
  - the flap is upright, like a side rail;
  - it holds the drone during transport;
  - it rests against the side cheek, which serves as a stop along its entire length.
- **Unfolded** :
  - the flap is laid flat, **exactly level with the tray**: a single flat surface;
  - the landing zone goes from 300 to approximately 530 mm wide.
- **Unfolded stops, for stability**: the flap never rests on the servo. Its two brackets rest on two stops, one at each end, with no additional parts:
  - at the front, the stop is part of the servo housing;
  - at the rear, it is part of the hinge block.

  The drone's weight, and the impact upon landing, are transmitted through these stops to the board. The servo simply presses the flap against them. (The 4 brackets under the flaps, and their 8 screws, have been removed.)
- **Hinge**: the axis runs along the edge of the tray, just outside the cheek.
  - **At the front: the SG90**, in the cut corner of the board, shaft aligned with the hinge axis. Its 1 arm is screwed onto the front bracket of the flap. The cable is thus very close to the tow bar.
  - The front bracket carries the **standard SG90 servo horn 1 imprint**: a 1,5 mm plate in front of its face, shaped to the horn profile (4 % larger for clearance). The horn fits flush into it, without pins, and the shape prevents slipping. It is secured with **the M2,5 servo screw**, passed through the **Ø3 hole on axis**, through the bracket and the horn hub. `10_empreinte_palonnier.py`.
  - **At the rear: a hinge block** screwed under the board, with a cheek flush with the tray. An **M3×20 screw serves as the axis**: it passes freely through the block and the rear bracket (Ø3,4), with a washer on each side and a locknut. Nothing is screwed into the plastic.
- **Servo housing**, a single well-sealed printed piece:
  - the SG90 enters from the front and is screwed in via its ears (screws provided with the servo);
  - its foot passes under the board: 2 M3×16 pan-head screws from the top, countersunk nuts underneath;
  - outside, the same piece forms the stop for the front bracket of the unfolded flap;
  - the cable passes inside (see below).
- **Brackets under the panel**: 2 M3×16 pan-head screws from the top of the plywood (without counterboring), countersunk nuts under the bracket bosses.
- **Movement**: 90° between folded and unfolded, within the SG90's travel (180°). Command each servo from 2 to 3° beyond each position, so that it presses the flap against its stop.
- **Is the SG90 sufficient?** Yes, with a margin of approximately 3.
  - The flap (plywood of 110 × 316,5 × 5 mm and its fittings) weighs approximately 125 g, with its center of gravity at 55 mm from the axis: it requires 0,07 N·m to lift it from the horizontal.
  - The SG90 provides 0,18 N·m at 4,8 V and approximately 0,22 N·m at 6 V.
  - When unfolded, the servo never supports the drone: the flap rests on its stops.
  - The weak point is its plastic gears: a gust of wind on a folded flap, or impacts while rolling.
  - For more margin, the **MG90S** has the same size, the same mounting points, and metal gears, providing 0,22 N·m. One can also switch the flaps to 3 mm plywood, which doubles the margin.
- **Wiring**: see [Servo cables](#câbles-des-servos). The servos can be powered on the 6 V branch of the steering servo.
- **In Fusion**, each flap rotates on a pivot joint with stops at 0° and 90°: it can be tilted with the mouse (see [Joints](#liaisons-dans-fusion)).
- **While rolling**, keep the flaps folded. When unfolded, they protrude 110 mm on each side.

![SG90 housing at the front and front fitting with the horn](vues/08_servo.png)

![Underside: servo housing, recessed nuts, front rim, and tiller](vues/09_dessous_cables.png)

<a id="câbles-des-servos"></a>
## Servo cables

With the servos at the front, the cable of each SG90 takes a short, hidden path to the hitch:
1. **It exits under the servo body**, inside the housing, runs backward in a tunnel under the servo, then rises to the level of the board.
2. **It heads towards the tiller under the board**, in a groove of 4,5 × 4,5 mm carved into the sole of the housing; the board closes it.
3. **Between the housing and the tiller** (3 cm), it is held by an adhesive cable tie base.
4. **It enters the tiller** through the notch of the wedge, goes down through a Ø5 hole, then runs in a groove under the A-arm, up to the head.
5. **The two cables end in a 4-pin connector** (5 V, ground, left signal, right signal), housed in a pocket under the tiller head.

**Length**: approximately 30 cm from the servo to the connector, compared to 65 cm when the servos were at the rear. The original SG90 wire is 25 cm long:
- cut its connector and thread the wire through the tunnel before sliding the servo into its housing;
- extend it by approximately 8 cm (soldering and heat-shrink tubing, hidden in the tow bar groove), or crimp longer wires directly into the connector.

On the rover side, a 4-wire extension exits the hull through hole C1 (Ø8) in the board, passes through the notch in the shackle sole, and loops down flexibly to the connector. Unplug it to detach.

<a id="roulements--pas-de-608"></a>
## Bearings: no 608

Each wheel rotates on two **5×16×5 (625ZZ)** from the list, i.e., 4 out of the 10 available.

- The rim is exactly the same as the front one, with its Ø22 housings intended for the 608.
- In each housing, press in a **printed ring** (Ø22 × 7) that receives a 5×16×5 :
  - the 2 mm lip of the ring is turned towards the rim edge;
  - the rim edge is sandwiched between the two rings;
  - the wheel cannot come out of its bearings.
- The axle is an **M5×40 CHC screw**, which only tightens the inner rings :
  - head;
  - printed washer Ø7,5 × 2;
  - outer bearing;
  - printed spacer of 6,2 mm;
  - inner bearing;
  - printed shim of 3 mm;
  - support leg;
  - M5 washer and lock nut.

![Wheel and mount](vues/04_roue.png)

![Wheel mount from below: 4 M6 screw, 2 near the edge and 2 towards the center](vues/10_support_dessous.png)

<a id="attelage--une-vis-ça-tourne"></a>
## Hitch: one screw, it turns

![Hitch](vues/03_attelage.png)

- **Rover side: a C-shaped clevis**, side view.
  - It is screwed under the rear of the board by 2 M6 screws (holes A1 and A2 from the [rover drilling plan](../plan_percages/plan_percages.pdf)).
  - It exits from under the shell via a footplate, then an inclined upright carries a lower jaw and an upper jaw.
  - Its edges are rounded.
- **Trailer side: an A-shaped towbar** (blue, like the edges):
  - two arms starting from the corners of the board;
  - a triangular recess;
  - a chamfered top;
  - a round head of Ø36 that fits between the two jaws.
- **An H screw M10×55**, inserted from above with a washer, passes through:
  - the upper jaw;
  - the towbar head;
  - the lower jaw;

  then a washer and a **M10 lock nut** underneath. A standard hitch, and a large bolt that holds it. You hitch and unhitch with two 17 wrenches.
- **It rotates around the bolt**: the head rests on a support ring of the lower jaw.
- **The head hole is hourglass-shaped** (Ø10,6 in the middle, flared at 12°), with play under the upper jaw: the trailer tilts by ±10° in pitch and roll over bumps. The bolt clamps the shackle, not the tongue head: tighten the lock nut until contact, without crushing the jaws.

![Trailer turned at 40°](vues/05_braquee_40.png)

<a id="vérifications-dans-fusion"></a>
## Checks in Fusion

Checked in Fusion:
- `02_verifier.py`, in the trailer document → `verification_remorque.json`;
- `05_verifier_attelage.py`, in the rover document with the trailer inserted → `verification_attelage.json`.

- **Collisions**: none, with flaps deployed or folded, and none between the trailer and the rover.
- **Steering**, flaps folded: free up to **52°** to the left and **52°** to the right.
  - At 54°, the servo housing, in the front corner, touches a rear wheel of the rover (64° when the servos were at the rear).
  - When the rover steers to the maximum (31°), the trailer turns to approximately 43° in the turn: there is still about 9° of margin.
- **Pitch and roll of ±10° at the hitch**: the tongue head does not touch the M10 bolt, the shackle, or the washers.
- **Clearances**:
  - tire → deployed flap: 5,5 mm;
  - tire → folded flap: 8,69 mm;
  - tire → rims and supports: 5,2 mm;
  - rim → support: 3 mm;
  - flap → servo housing and rear block: 3 mm;
  - SG90 → board: 5,9 mm; SG90 → front lip: 3,5 mm;
  - tiller → rover, without hitching: 79 mm.
- **Parts**: all in one piece.

<a id="pièces-à-imprimer"></a>
## Parts to print

STLs already oriented, no supports, in `exports/stl/` (manifest: `exports/manifest.json`).

| Part | Qty | Color | Size (mm) | Placed on the build plate |
| --- | ---: | --- | --- | --- |
| `rebord_AV_G` | 1 | blue | 146,7 × 57,3 × 10,5 | flipped over, lip against the build plate |
| `rebord_AR_G` | 1 | blue | 150 × 16,4 × 10,5 | flipped over, lip against the build plate |
| `rebord_AR_D` | 1 | blue | 150 × 16,4 × 10,5 | flipped over, lip against the build plate |
| `rebord_AV_D` | 1 | blue | 146,7 × 57,3 × 10,5 | flipped over, lip against the build plate |
| `joue_M1_G` | 1 | blue | 7 × 172 × 16,4 | lying on its outer face |
| `joue_M2_G` | 1 | blue | 7 × 144 × 16,4 | lying on its outer face |
| `joue_M2_D` | 1 | blue | 7 × 144 × 16,4 | lying on its outer face |
| `joue_M1_D` | 1 | blue | 7 × 172 × 16,4 | lying on its outer face |
| `boitier_servo_G` | 1 | white | 39 × 69,5 × 50,9 | lying on its outer face (housing and stop) |
| `charniere_AR_G` | 1 | white | 50,9 × 17 × 34,4 | standing on its rear face |
| `boitier_servo_D` | 1 | white | 39 × 69,5 × 50,9 | lying on its outer face (housing and stop) |
| `charniere_AR_D` | 1 | white | 50,9 × 17 × 34,4 | standing on its rear face |
| `ferrure_AV_G` | 1 | white | 49,6 × 18 × 31,5 | standing on the face of the yoke |
| `ferrure_AR_G` | 1 | white | 49,6 × 18 × 26 | standing, vertical knot |
| `ferrure_AV_D` | 1 | white | 49,6 × 18 × 31,5 | standing on the face of the yoke |
| `ferrure_AR_D` | 1 | white | 49,6 × 18 × 26 | standing, vertical knot |
| `timon` | 1 | blue | 168 × 168 × 25,5 | flat, underside against the build plate |
| `support_roue_G` | 1 | white | 94,5 × 120 × 77,5 | lying on the outer face of the leg |
| `bague_625_dans_logement_608` | 4 | white | 22 × 22 × 7 | lip against the build plate |
| `entretoise_625_6.2mm` | 2 | white | 7,5 × 7,5 × 6,2 | flat |
| `rondelle_tete_M5_2mm` | 2 | white | 7,5 × 7,5 × 2 | flat |
| `cale_jambe_3mm` | 2 | white | 7,5 × 7,5 × 3 | flat |
| `support_roue_D` | 1 | white | 94,5 × 120 × 77,5 | lying on the outer face of the leg |
| `chape_attelage_rover` | 1 | white | 37 × 147 × 44 | lying on its side |

- **Filament** (counted solid parts, so an upper bound): blue (PLA) approx. 420 g, white (PETG) approx. 630 g, excluding wheels.
- The largest printed part measures 168 mm: everything fits on the A1 mini's 180 mm build plate.
- **Rings**: print one ring first and try a bearing: it should fit by hand or in a vise, without falling out.

The rims, tires, and flanges are the same as the front wheels: same STLs as the [steering V04](../../mounts/prototype/direction_v04_servo_traversant/exports/stl/) (`jante_608ZZ`, `pneu_TPU`, `flasque_pneu`), two more.

<a id="visserie-et-achats"></a>
## Screws and purchases

Complete list of the rover and trailer, by size: [../VISSERIE.md](../VISSERIE.en.md).

| What | Quantity | Where |
| --- | ---: | --- |
| SG90 servo (with its 1 horn and screws) | 2 | Flaps |
| 5×16×5 bearing (625ZZ) | 4 | 2 per wheel, inside the rings |
| M5×40 CHC screw + M5 washer + M5 locknut | 2 | Axis of each wheel |
| M3×8 screw + M3 heat-set insert | 12 | Tire flanges, 6 per wheel, as at the front |
| M6×25 dome-head screw (ISO 7380) + M6 nut | 4 | Tongue, from above through the board, nut recessed under the tongue (holes T1–T4, Ø6,5) |
| M6×16 dome-head screw (ISO 7380) + M6 nut | 8 | Wheel supports, 4 per support, from above, nut recessed under the arm |
| M3×16 dome-head screw (ISO 7380) | 14 | From above: servo housings (4), rear blocks (2), flap fittings (8) |
| M3×10 screw + washer | 6 | Front and rear edges, from underneath |
| M3 nut | 20 | Recessed in the lip of the edges (6), under the arms of the housings and rear blocks (6), under the fittings (8) |
| M3×20 screw + 2 washers + M3 locknut | 2 | Rear axis of each flap |
| H M10×55 screw + 2 M10 washers + M10 locknut | 1 | Hitch screw, from above, nut under the shackle |
| M6×20 screw + washer + M6 nut | 2 | Shackle on the rover board (nuts in the shackle) |
| 4-pin connector (JST-XH or Dupont) male + female, 4 flexible wires | 1 | Flap cables at the hitch, extension on the rover side |
| Adhesive base for clamp + clamp | 2 | Cables under the board, between the servo housing and the tongue |
| 5 mm plywood | 300 × 380 + 2 × (110 × 316,5) | Platform and flaps |
| Matte paint | — | Cross |

<a id="plan-de-découpe-et-de-perçage"></a>
## Cutting and drilling plan

[plan_planche/plan_planche.pdf](plan_planche/plan_planche.pdf) contains:
- the dimensioned board, with its corners cut off;
- its Ø3,4 holes;
- the impressions of the parts screwed underneath;
- the coordinates table;
- a scale template 1 on 4 A4 sheets.

No milling: all holes are straight (drill bit 4 for M3, drill bit 7 for M6). Workshop plans with drill bits and dimensions: [../plans_atelier/plans_atelier.pdf](../plans_atelier/plans_atelier.pdf).

**Flaps** (2 panels of 110 × 316,5 mm): 4 straight holes (drill bit 4) per panel, at 20 and 37 mm from the edge on the hinge side, at 13 mm from the front end and at 10 mm from the rear end.

![Board plan](plan_planche/plan_planche.png)

<a id="montage"></a>
## Assembly

1. **Board**: cut the front corners, drill the holes, paint the cross.
2. **Rover**:
   - drill A1 and A2;
   - screw the clevis under the board (M6×20 screws from above, nuts in the slots);
3. **Wheels**:
   - press the bushings into the rims, lip towards the edge, then the bearings;
   - mount the tire and the flange as at the front.
4. **Servos**: cut the connector off each SG90, thread the wire through the tunnel in the housing (it emerges at the top, in the sole groove), slide the servo in from the front, and screw on its ears.
5. **Assembly upside down on the table**:
   - place the front and rear flanges upside down, and place an M3 nut in each pocket;
   - place the board on top, cross-side down, then the side cheeks, with the sole on the board;
   - place the towbar, with an M6 nut in each of its 4 pockets; the M6×25 dome-head screws are inserted from the top, once the board is flipped over;
   - place a nut in each pocket under the arms: M6 under the wheel supports, M3 under the servo housings and the rear blocks;
   - place these parts on the sole of the cheeks, with each servo wire lying in the groove of the sole;
   - flip the trailer over and screw in the dome-head screws from the top: M6×16 for the wheel supports, M3×16 for the rest;
   - thread each wire through the towbar (notch, hole, groove) up to the connector under its head, and secure it with an adhesive base between the housing and the towbar.
6. **Wheels on the supports**, from outside to inside:
   - the M5 screw with its printed washer;
   - the wheel;
   - the 3 mm spacer;
   - the leg;
   - the washer and locknut. Tighten until the wheel has no play but still turns freely.
7. **Flaps**:
   - screw the hinges under each panel: M3×16 dome-head screws from the top of the plywood, nuts in the pockets of the bosses;
   - screw the control horn onto the front face of the front hinge;
   - center the servo in the middle of its travel, then engage the yoke hub onto the shaft, with the flap half-open;
   - pass the M3×20 shaft through the rear block and the rear bracket, with a washer on each side, then the lock nut, tightened just enough to eliminate any play. The rear block holds the flap in place: the yoke cannot come off the servo shaft.
8. **Coupling**: place the tongue head between the jaws, pass the M10 screw from above with its washer, then the washer and lock nut underneath; tighten until contact (two 17 wrenches).

<a id="à-valider"></a>
## To validate

- **E800 Drone**: I was unable to identify this model with certainty. The plate assumes a small drone of approximately 30 cm including propellers; with flaps deployed, the area is approximately 530 × 316 mm. For a larger drone, widen the flaps (`VOLET_W` in `rm_helpers.py`) or lengthen the board (`LONG`), then relaunch `01` to `04`.
- **SG90 Dimensions**: the housing is designed to Tower Pro dimensions (body 22,5 × 11,8 × 22,7 mm, pinion 4 mm, shaft 3,2 mm, ears 2,5 mm to 15,9 mm from the bottom, holes at 27,3 mm), with 0,3 mm of clearance on each side. Copies vary by a few tenths: measure the servo with calipers and print a housing first to test it. An MG90S (12,2 wide) also fits, but more snugly.
- **SG90 Travel**: set the two extreme positions during testing (hard stops) before locking them in the software.
- **Holding of M3 screws in the edges**: perform a tightening test on a first edge.

<a id="liaisons-dans-fusion"></a>
## Constraints in Fusion

The trailer parts are connected by real constraints, as on the real object:

| Constraint | Type | Between | Travel |
| --- | --- | --- | --- |
| `pivot_roue_G`, `pivot_roue_D` | pivot | wheel (rim, tire, flange, rings, bearings) / support | free |
| `charniere_volet_G`, `charniere_volet_D` | pivot | flap (panel, brackets, yoke) / hinges | 0° (deployed) to 90° (folded) |
| `attelage_remorque` (rover document) | ball joint, in the middle of the diabolo hole | tongue / clevis | yaw ±64°, pitch ±10° |

The M5 shaft, its washers, spacers, and nut remain fixed, in `RM_REF_Axe_roue_G/D`. All fixed parts (board, cross, edges, cheeks, hinges, housings, SG90s, tongue, supports, shafts) form the rigid group `chassis_remorque`: the ball joint holds the tongue, and the entire trailer follows. The Fusion ball joint provides only two free rotations (yaw and pitch); the roll of ±10°, verified by `05_verifier_attelage.py`, cannot be manipulated with the mouse. In the rover document, the two rover groups are anchored. Therefore, we steer and tilt the trailer, roll its wheels, and open its flaps with the mouse.

<a id="documents-fusion-et-scripts"></a>
## Fusion documents and scripts

The trailer has **its own Fusion document**, **`remorque fieldmind`**, in the same folder as the rover document, **`prtotype final fieldmind`**.

- It is **inserted by reference** in the rover document, at the identity. It is drawn in the rover's coordinate system, so it aligns hitched in the axis.
- Modify it and rebuild it in its own document: approximately 5 minutes instead of 30 when it was in the rover document. The rover is not affected.
- The trailer document is saved **with flaps folded**. Its two pivot joints (`charniere_volet_G/D`) are manipulated in Fusion, between their stops at 0° and 90°.
- The **towing shackle** (`PF_IMP_Attelage`) remains in the rover document: it is screwed under its board.

The scripts go through Fusion's local MCP server (client `../../mounts/prototype/direction_v04_servo_traversant/scripts/fusion_mcp.py`). Order to redo everything:

| Script | Active Document | Role |
| --- | --- | --- |
| `06_creer_document.py` | rover | Once only: creates and saves the trailer document, removes the old trailer from the rover |
| `01_remorque.py` | trailer | Board, cross, edges and cheeks, servo housings (front), hinge blocks (rear), flaps and SG90, tongue, supports, wheels (copied from the rover's front wheels, which must be open), wheel and flap linkages, rigid group → `remorque_bilan.json` |
| `02_verifier.py` | trailer | Collisions (flaps unfolded and folded), clearances, parts → `verification_remorque.json` ; leaves flaps folded |
| `03_exporter.py` | trailer | Oriented STL, STEP + F3D, manifest |
| `07_enregistrer_remorque.py` | trailer | Freezes flap position and saves a new version |
| `08_inserer_dans_rover.py` | → rover | Inserts the trailer into the rover, or updates the reference; creates the ball joint `attelage_remorque` |
| `00_chape_rover.py` | rover | Towing shackle (M10 hole), screw and nut as reference, its STL → `chape_bilan.json` |
| `05_verifier_attelage.py` | rover | Trailer / rover collisions, steering, tilting at hitch → `verification_attelage.json` |
| `04_plan_planche.py` | outside Fusion | Cutting and drilling plan for the board (PDF) |
| `rm_helpers.py` | — | Dimensions and common tools |

External 3D reference used in design (not redistributed in this edition, see [CAD sources](../../README.en.md)) :
- SG90 and its horn 1 arm: [FreeCAD library](https://github.com/FreeCAD/FreeCAD-library/tree/master/Electrical%20Parts/Servos/SG-90), with Tower Pro dimensions.

Holes A1 and A2 (shackle) and C1 (cables) on the rover are added by `../scripts/05_percages.py`, which reads `remorque_bilan.json`.

Only the trailer's document is saved by the scripts (`06` and `07`). The rover's document is not: you save it whenever you want.
