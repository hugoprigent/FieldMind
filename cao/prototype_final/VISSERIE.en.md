[Français](VISSERIE.md) · **English**

<a id="fieldmind--toute-la-visserie-du-prototype-final"></a>
# FieldMind — all fasteners for the final prototype

Document update and recalculation: **18 October 2026**. Net quantities as planned, **excluding margin and excluding deduction of existing stock**; this is not a validation of physical assemblies.

Complete rover (V04 steering, masts, hull, components) and drone carrier trailer. This document compiles the lists for each part:

- [V04 steering](../mounts/prototype/direction_v04_servo_traversant/README.en.md#quincaillerie) ;
- [hull and components](README.en.md#visserie) ;
- [trailer](remorque/README.en.md#visserie-et-achats).

The [CSV list](VISSERIE.csv) separates quantities for the **rover with clevis** and the **trailer** for each reference, including sizing reserves. The old trailer lists for M5/M8/M12 do not add to this version: the standard hitch is **M10×55** and the trailer wheel supports are **M6×16 countersunk head**. No countersunk screws: no countersinking in the wood.

| Assembly | Screws planned | M3 inserts |
| --- | ---: | ---: |
| Rover, hull, masts and hitch clevis | 126 | 32 |
| Trailer, excluding clevis already counted on the rover | 48 | 12 |
| **Total** | **174** | **44** |

The 174 include the **2 screws for the webcams with thread to be confirmed** and the **M3×16 for the tiller, whose length must be checked during assembly**. The **8 motor screws to be sized** are additional, outside this total.

<a id="liste-par-taille--quantités-nettes"></a>
## List by size: net quantities

M3×12 means nominal diameter 3 mm and length 12 mm. Countersunk heads are distinguished from non-countersunk heads; they must not be substituted without verifying the assembly. H = hexagonal head; CHC = cylindrical head with hex socket.

| Screws | Quantity | Nuts and washers |
| --- | ---: | --- |
| M2×4 | 4 | — |
| M2,5×8, cylindrical head | 4 | 4 M2,5 washers |
| M3×6 | 20 | 20 large M3 washers (DIN 9021, Ø9) |
| M3×8 | 32 | 4 M3 nuts (XL4015); the others screwed into inserts or into the plastic |
| M3×10 | 6 | 6 M3 nuts, 6 M3 washers (trailer edges) |
| M3×12, cylindrical head | 16 | 12 M3 nuts, 12 M3 washers |
| M3×12, domed head | 4 | 4 M3 nuts, 8 narrow M3 washers for tie rods |
| M3×16, cylindrical head | 11 | 10 M3 nuts, 10 M3 washers (board of 10 mm); 1 wide M3 washer (tiller) |
| **M3×16, domed head (ISO 7380)** | **14** | 14 M3 nuts |
| M3×20 | 2 | 2 M3 lock nuts, 4 M3 washers |
| CHC M5×40 | 2 | 2 M5 lock nuts, 2 M5 washers |
| **M6×16, domed head (ISO 7380)** | **8** | 8 M6 nuts |
| **M6×25, domed head (ISO 7380)** | **4** | 4 M6 nuts (trailer tongue) |
| M6×20 | 10 | 10 M6 nuts, 10 M6 washers (hull; 4 are sufficient) |
| M6×25 | 6 | 6 M6 nuts (motor mount clamps, clevis), 2 M6 washers |
| M8×16 | 8 | 8 T-nuts M8 for 10 groove |
| H M8×25 | 8 | 8 M8 nuts, 8 M8 washers (pivot mounts, brackets) |
| H M8×30 | 12 | 12 M8 nuts, 16 M8 washers (motor mounts); 2 mm printed washers (pivots, axles) |
| H M10×55 | 1 | 1 M10 lock nut, 2 M10 washers |
| Photo screw 1/4"-20 × 1/2", to be confirmed | 2 | — ; M6×12 is another hypothesis, not a reference to add |

<a id="écrous-rondelles-et-inserts"></a>
## Nuts, washers and inserts

These quantities are for the assembly. Add a margin separately if desired; no margin is already included in the totals.

| What | Quantity |
| --- | ---: |
| Screws, all sizes | 174 |
| M2,5 washers | 4 |
| M3 nuts | 50 |
| M3 lock nuts | 2 |
| Standard M3 washers | 32 |
| Narrow M3 washers, bearing on the inner ring of the 3×10×4 | 8, or 8 printed washers in their place |
| Wide M3 washers (DIN 9021, Ø9) | 21 |
| M3 heat-set inserts (outer diameter 4,5 to 4,6 mm, length up to 5,7 mm depending on housing) | 44 |
| M5 lock nuts + M5 washers | 2 + 2 |
| M6 nuts + M6 washers | 28 + 12 |
| M10 lock nut + M10 washers | 1 + 2 |
| M8 nuts | 20 |
| M8 washers | 24 |
| M8 T-nuts, groove 10 | 8 |

The **46 non-wide M3 washers** break down into 38 standard and 8 narrow. For the connecting rods, the plan proposes printed washers **outer diameter 5 mm, inner diameter 3,3 mm, thickness 0,6 mm**; a metal part must bear only on the inner ring. Do not order 8 additional washers if the 8 printed ones are used.

The printed washers/spacers for the pivots and hubs specified in the plans remain parts to be manufactured, not metal washers to be added to these quantities.

Screws are only used where plastic screws into plastic: shell band bosses and tire flanges. Wherever a printed part is fixed to wood, screws and nuts are used.

<a id="dimensions-à-confirmer-avant-achat-définitif"></a>
## Dimensions to confirm before final purchase

- **Rear motor mounting: 8 screws planned**, 4 per motor according to the mounts. The motors are now identified as NGP RM24A/RM18A, but the thread sizes, their depth, and the actual mounting on these supports still need to be determined. The Ø4 holes in the supports do not allow choosing between M3 or M4. These 8 screws are not included in the 174.
- **Webcams: 2 screws**, thread and engaged length to be confirmed. The plan selects 1/4"-20 × 1/2"; M6×12 is only suitable if the thread is truly M6. These are two exclusive options.
- **DT996 horn: 1 M3×16 screw planned**, M3 diameter confirmed in the V04 report. Check its length with the star fully inserted: the screw must tighten the stack without bottoming out in the shaft.
- **C1 Lidar: M2,5×8**, with a maximum of 4 mm penetration into the lidar according to the archived manufacturer plan and the current support.

The Pi in its case, the converters, and the controllers are mounted with adhesive/double-sided tape in the current version of the shell: no additional screwing for the plate or spacers is defined for them. The A7 board, screen, and Knob are well included in the table.

**Original hardware to reuse and check in the bags**:
- screws for the ears and horn of the 2 SG90s on the trailer;
- original servo steering screw: the one for the star is replaced by the M3×16.

<a id="détail-partie-par-partie"></a>
## Detail, part by part

<a id="direction-avant-v04-mâts-et-supports"></a>
### Front steering V04, masts and supports

| Screw | Quantity | Where |
| --- | ---: | --- |
| H M8×30 + M8 nut | 2 | Pivots: screw from below, captive nut in the support (threadlocker) |
| H M8×30 + M8 nut | 2 | Front wheel axles: captive nut in the fork leg (threadlocker) |
| H M8×25 + nut + M8 washer | 4 | Pivot supports through the 10 mm board |
| H M8×30 + nut + 2 M8 washers | 8 | Rear motor supports, 4 per support, through the 10 mm board |
| M6×25 + M6 nut | 4 | Motor support clamps, 2 per support (nut in the keel) |
| H M8×25 + nut + washer M8 | 4 | Brackets of both masts on the board |
| M8×16 + M8 T-nut | 8 | 4 for brackets on profiles, 2 for the stereo bar, 2 for the lidar mount (slotted onto the 90 profile) |
| M3×8 + M3 insert | 12 | Front tire flanges, 6 per wheel |
| M3×12 domed head + 2 narrow washers + M3 nut | 4 | Steering tie rods |
| M3×16 + nut + washer M3 | 4 | Servo ears for steering, through the board |
| M3×16 + wide washer M3 | 1 | Central screw of the horn, aligned with the servo (blue thread-locking) |
| M2,5×8 + washer | 4 | Lidar C1 on its mount |
| Photo screw 1/4"-20 × 1/2" | 2 | Webcams on the stereo bar (M6×12 if the threading is indeed M6) |

<a id="coque-et-composants"></a>
### Hull and components

| Screw | Quantity | Where |
| --- | ---: | --- |
| M3×6 + wide DIN 9021 washer | 20 | Blue strips on the side edges, into 20 M3 inserts |
| M3×12 + nut + washer M3 | 12 | Ribs between sections, 2 per side and per cut |
| M6×20 + washer M6 | 10 | Hull on the board, from underneath, into M6 nuts slid into slots (4 at the corners are sufficient) |
| M3×16 + washer M3, M3 nut recessed in the bottom of the cradle | 4 | Battery cradle (front left), screw from under the board |
| M3×8 | 4 | A7 board under strip C3, directly into the plastic |
| M3×12 | 4 | Modulino Knob under strip C3, directly into the plastic |
| M3×8 + M3 nut | 4 | The two XL4015 on their printed plate: screw from above, nut in the pocket under the plate |
| M3×16 + washer + M3 nut | 2 | XL4015 plate on the board, at the front right (holes X1 and X2), or double-sided |
| M2×4 | 4 | Screen under the C3 band, directly in the plastic |

<a id="attelage-côté-rover"></a>
### Hitch, rover side

| Screw | Quantity | Where |
| --- | ---: | --- |
| M6×25 + washer + M6 nut | 2 | Clevis under the rover board (holes A1 and A2), nuts in the clevis |
| H M10×55 + 2 washers + M10 locknut | 1 | Hitch screw from above, through both jaws and the tongue head, nut under the clevis |

<a id="remorque"></a>
### Trailer

| Screw | Quantity | Where |
| --- | ---: | --- |
| M6×16 dome head + M6 nut | 8 | Wheel supports, 4 per support (2 near the edge, 2 towards the middle), from above, countersunk nut |
| M3×16 dome head + M3 nut | 14 | From above, countersunk nut: servo housings (4), rear blocks (2), flap fittings (8) |
| M6×25 dome head + M6 nut | 4 | Tongue: from above, through the board, countersunk nut under the tongue |
| M3×10 + washer | 6 | Front and rear edges, from underneath, in the countersunk nuts of the lip |
| M3 nut | 12 | Countersunk in the lip of the edges (for the 6 M3×30 and the 6 M3×10) |
| M3×20 + 2 washers + M3 locknut | 2 | Rear axle of each flap |
| M3×8 + M3 insert | 12 | Trailer tire flanges, 6 per wheel, as at the front |
| CHC M5×40 + washer + M5 locknut | 2 | Axle of each trailer wheel |
