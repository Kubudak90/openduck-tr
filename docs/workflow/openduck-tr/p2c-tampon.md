# P2c: neck + head, chin/forehead bumper load path and MG90S shaft load check

Input: `spec.md` v3.1 (3.1 Mekanik, 3.2.2, A1), `p2-interface.md` (P2-0, frozen), `research/servo-mechanics.md`.
Code: `cad/head.py` (`build_tr35`), `configs/tr35_b2.json` `params.head`, `cad/export.py` (bumper geom only).
All coordinates: mm, trunk frame, zero pose (x forward, y left, z up). g = 9.81 m/s2.

## 1. What was built

| Link | Part (kind) | Function |
|---|---|---|
| neck | `neck_link` (bracket, PETG) | U-link: leg A embeds the DS aluminium 25T arm (pocket + 2 x M2.5 with nut traps + M3 centre screw), leg A thrust sleeve on the trunk_base cradle walls (0.2 mm), leg B carries the idler pin (M3 screw through the F623ZZ, nut in a leg B trap, brass spacer on the inner race); bridge under the head_pitch MG90S; MG90S cradle (side walls, upper tab plate on the horn side of the tab, lower tab in a bridge slot open to -y; servo slides in from -y, 2 x M2 tab screws from the rear); bumper stem + tongue |
| neck | `head_pitch_thrust_in`, `head_pitch_thrust_out` (bracket) | thrust faces on both sides of the head_tilt rotor disc, 0.3 mm running gap (inner: 4 x M2 to the walls; outer: foot on the bridge, 2 x M2) |
| neck | `neck_bumper` (pad, TPU 95A) | chin/forehead bumper, press-fit on the stem tongue; `collision_box` = pad box, exported as MJCF geom `neck_bumper` |
| head_tilt | `head_tilt_bracket` (bracket) | one-piece L: rotor disc (embeds the head_pitch horn, arms vertical) + upper arm + yaw table (embeds the head_yaw horn from below, arms along y) |
| head_tilt | `head_yaw_retainer_left/right` (bracket), `head_yaw_retainer_screws` | two PETG arcs on the yaw table (+-y, 50 deg each, r 12..17, outer top edge chamfered, 2 x M2 into the table, heads sunk): their lips hold the head_turn ring flange from above (0.3 mm), verification round 2 (D3). The arc width and the chamfer keep them clear of the head_frame deck under head_roll (0.85 mm at the +16 limit) |
| head_turn | `head_turn_bracket` (bracket) | yaw MG90S tab frame (servo drops in from above, tab screws from above), thrust ring on the yaw table (0.3 mm) with an outer flange (r 13.5, 1.2 thick; trimmed at +x to the head_turn envelope) under the retainer lips (0.3 mm), rear plate (embeds the roll horn, arms along y), front plate + roll pin d8 |
| head | `head_frame` (bracket) | ring deck around the head_tilt/head_turn passage, roll MG90S tab plate, SG90 tab plate, roll-pin front bushing (split cap at assembly), roll thrust ring (0.3 mm behind the head_turn rear plate), chin block (bumper contact face) |
| head | `head_hood` (shell, PLA), `upper_bill` (trim, PLA) | constant 1.8 mm wall (`params.wall`), open-bottom dome x -28..80, top z 199; bill shell to x 118.3 |
| head | `jaw_pin_bushing` (bracket) | right jaw pin bearing, screwed to the hood's right wall (2 x M2) |
| jaw | `jaw_hub` (bracket), `lower_bill` (trim) | SG90 horn captured inside the hood wall, hub through the wall hole; lower bill = two outer arms + chin bar + constant-wall tray; right arm pin in `jaw_pin_bushing`. Hub and left arm joined by 2 x M2 |
| all | `<joint>_harness_in/out` (harness) | frozen cross points and diameters; routing inside the head links by P2c (sec. 8) |

Joint origins, axes, servo faces/directions, IDs, names (`head`, `head_collision`, keyframe `stand`) are unchanged.
Head top at the stand keyframe: 347.7 mm (hood top z 199).

## 2. Bumper: placement and contact sequence

The bumper is the over-travel stop of the head on the neck. The TPU pad (x 51.5..58.8, y +-12, z 88.5..97.5, inside
the frozen bumper zone x 51..59, y +-16, z 88..102) sits on a PETG tongue in front of the neck bridge. When the head
pitches forward past its range, the **chin block of the head frame** (head link, x 54.5..58, y +-14, z 126.6..141.8)
lands on the pad.

Swept contact distances, other joints at the stand keyframe (FreeCAD `distToShape`):

| head_pitch | chin block (head) | head_tilt crossbar | yaw harness | hood | lower / upper bill |
|---|---|---|---|---|---|
| 72 (limit) | 0.99 | 2.90 | 11.7 | 21.7 | 25.1 / 27.8 |
| 75 (MJCF range = limit + 3, spec 3.2.3) | 0.24 | 1.72 | 10.9 | 22.3 | 25.9 / 27.9 |
| **76.0** | **0 (contact)** | 1.33 | 10.6 | 22.6 | 26.2 / 28.0 |

So the first and only part that touches the bumper is the head itself; the head_tilt (and with it the yaw and roll
MG90S) would need another 1.33 mm of TPU compression (about 18 % strain of the 7.3 mm pad, roughly 340 N on a 24 x 3
mm chin edge for TPU 95A, E ~ 26 MPa) before it shares any load. The pad top was lowered from z 101.5 to 97.5 for
this: at 101.5 the crossbar touched first (70 deg).

In MuJoCo the pad is a collision geom on the neck body (`<geom name="neck_bumper" class="collision">`, box, contype 0
/ conaffinity 1, pos (0.0218, 0, 0.0515) m and half sizes (3.65, 12, 4.5) mm in the neck frame). Ground contacts on it
go into the neck joint, not the MG90S chain. `neck` is not added to `collision_links` and no `neck_collision` is
exported (the RL getup variant adds its own).

## 3. Bumper load path

```
impact torque on the head about the pitch axis (M_i > MG90S stall 0.216 N.m back-drives the head)
  -> head reaches 76 deg -> chin block (head_frame, PETG) -> TPU pad -> PETG tongue / stem (root 18 x 4.4 mm)
  -> neck bridge (z 84.1..88.5) -> neck legs A and B
       leg A -> DS aluminium 25T arm (embedded, 2 x M2.5 + M3) -> DS32xx output (torque + radial)
       leg B -> M3 pin -> F623ZZ in the trunk's idler seat (radial; axial y load: sec. 4, neck_pitch row)
  -> trunk neck cradle
No head_pitch / head_yaw / head_roll MG90S is in this path (the head_tilt and head_turn are bypassed).
```

Design case: 10 g on the assembly beyond the pitch joint (150.2 g -> 14.7 N) at a 45 mm arm:
M_i = 0.66 N.m. Chin contact radius from the pitch axis r_c = 29.8 mm:

| Item | Value | Limit / margin |
|---|---|---|
| chin force F_c = M_i / r_c | 22 N | pad compression before the head_tilt touches: ~340 N |
| stem root bending (F_c x 7 mm) | 155 N.mm, sigma 2.7 MPa | PETG ~50 MPa |
| torque on the neck DS (F_c x 55 mm) | 1.2 N.m | DS32xx stall 2.88 N.m |
| over-travel beyond the limit | 4 deg (72 -> 76) | MG90S travel +-90, gear end stop never reached |

Static neck load (whole head module 199.4 g, COM (31.5, 1.0, 136.5)): 0.067 N.m at HOME (neck +20), 0.186 N.m at
neck -90; the DS output sees torque and radial load only (horn and idler 57.8 mm apart; reactions 0.92 N / 1.04 N).

## 4. MG90S / SG90 shaft load check (A1 "no bending or axial load on servo shafts")

Masses and COMs from the CAD model (bottom-up). Dynamic design factor 3 g (walking/getup); impacts above that go to
the bumper (sec. 3).

| Joint (servo) | Load beyond the joint | COM offset from the horn centre | Bending moment without support (static / 3 g) | Passive support taking it | Shaft keeps |
|---|---|---|---|---|---|
| head_pitch (MG90S) | 150.2 g, W 1.47 N | (-2.7, -14.6, 49.8) | 21.5 N.mm / 220 N.mm (lateral) | rotor disc r 12 between two thrust faces (gap 0.3 each): couple arm ~20 mm -> 11 N at 3 g, < 0.5 MPa on PETG; axial (y) load also on the faces | torque + radial shear <= 4.4 N |
| head_yaw (MG90S) | 141.8 g, W 1.39 N | (-2.9, 2.1, 27.9) | 5 N.mm / 116 N.mm; axial 1.39 N / 4.2 N (compression), the same in tension when the head is inverted (getup, falls) | down: head_turn thrust ring r 9..11.5 on the yaw table (gap 0.3): 11 N edge load at 3 g, 0.14 MPa; carries the head's weight. Up (D3): ring flange r 11.5..13.5 under the lips of two retainer arcs (gap 0.3, 1.5 mm radial overlap, 0.6 mm in the +x trim), 2 x M2 per arc into the table: 14 N at 10 g gives about 2 MPa lip bending (1.3 mm PETG). The tipping moment closes one side on the table and the other under a lip | torque + radial shear |
| head_roll (MG90S) | 122.0 g, W 1.20 N | COM x 30.3 between horn (x ~8) and front pin (x ~44.9) | two-point support: horn reaction 0.47 N, pin 0.72 N (1.4 / 2.2 N at 3 g) | roll pin d8 in the head-frame front bushing (radial gap 0.25); axial: backward -> front plate on the bushing wall (0.5), forward -> roll thrust ring (0.3) | torque + radial shear |
| mouth (SG90) | 10.3 g, W 0.10 N | COM between horn (y 54.5) and right pin (y -54.5) | two-point support: 0.06 / 0.04 N | right pin d5 in `jaw_pin_bushing`; axial (y): outer arms on the hood walls (0.5) | torque (3.8 N.mm static, stall 177) |
| neck_pitch (DS32xx) | head module 201.3 g, W 1.97 N | see sec. 3 | radial: U-link A (Al arm on the spline) and B (M3 pin in the F623ZZ) 57.8 mm apart (sec. 3). Axial (y, lateral head load: 5.9 N at 3 g, 19.7 N at 10 g): toward the servo (+y) the leg A thrust sleeve (r 8.2..11.5, 0.2 mm gap) bears on the end faces of the trunk_base neck-cradle side walls (y -21.0), about 20 mm2, 1.0 MPa at 10 g; away from it (-y) the leg B nut presses a brass spacer on the F623ZZ inner race (0.05 mm gap), outer race on the 1 mm seat lip (52 mm2, 0.4 MPa; F623ZZ C0r 180 N) | torque + radial |

Gap note (assumption, to confirm in Step 0): the faces and rings are modelled with a 0.3 mm print gap. They take the
moment once the rotor tilts by gap / radius (head_pitch 0.3 / 10 = 1.7 deg, yaw 0.3 / 10 = 1.7 deg, roll pin 0.25 / 37
= 0.4 deg). The MG90S output (bushing + gear play, plastic horn on a 21T spline) is much softer in tilt than the PETG
contact, so above a few N.mm the moment goes to the faces. To make the static moment (21.5 N.mm on head_pitch, the
head's weight on the yaw shaft) go there too, assemble each thrust face / ring with a 0.25 mm PTFE washer or tape
(shims the gap to ~0.05 mm). Add to Step 0: MG90S output tilt under a known moment (and SG90), PTFE shim fit.

## 5. Joint limits (derived)

Limit = (collision-free single-joint sweep of the CAD parts, other joints at stand) and (module-envelope sweep,
check.py section 6) and (servo travel), minus ~3 deg; required ranges (spec 3.1) inside. Sweeps in 1 deg steps:

| Joint | Required | Parts free | Envelopes free | Travel | **Limit** | First contact beyond |
|---|---|---|---|---|---|---|
| neck_pitch | -90..40 | -115..47 | -115..42 | -115..65 | **-110..40** | +43: head_tilt envelope vs trunk K_T (parts: +48 esp32 vs thrust plate) |
| head_pitch | -20..60 | -38..76 | -28..78 | -90..90 | **-25..72** | -29: head envelope vs K_T (parts -39 hood vs esp32); +76: chin on bumper (designed stop, sec. 2) |
| head_yaw | -34..34 | -90..90 | -90..90 | -90..90 | **-80..80** (spec) | none |
| head_roll | -10..10 | -23..19 | -24..20 | -90..90 | **-20..16** | -24: yaw harness / head_turn vs head frame; +20: head_tilt vs head frame |
| mouth | 0..30 | -1..35 | 0..36 | -90..90 | **0..32** | -2: lower vs upper bill (closed stop); +36: lower bill vs bumper |

head_pitch upper 72: the MJCF joint range (limit + 3 = 75) ends 1 deg before the physical bumper contact (76).

## 6. Mass against the budget (p2-interface.md sec. 8: head 202.0 g)

| Group | Budget g | CAD g | Parts |
|---|---|---|---|
| MG90S x 3 + SG90 | 49.2 | 49.2 | servo records |
| horns (DS arm 5 + 3 x MG90S + SG90) | 7.0 | 7.0 | |
| neck idler pin | 1.8 | 1.8 | M3 screw + nut + brass spacer (`servos.ds_idler_screw`, as in the legs; budget mass) |
| printed: neck U + bumper | 26 | 25.5 | neck_link 21.2 (with the leg A thrust sleeve), thrust_in 1.1, thrust_out 2.0, bumper (TPU) 1.2 |
| printed: tilt | 8 | 7.0 | head_tilt_bracket 6.6, head_yaw retainer arcs 2 x 0.2 (D3) |
| printed: turn | 8 | 5.0 | head_turn_bracket (with the D3 flange) |
| printed: frame | 12 | 14.5 | head_frame 13.1, jaw_pin_bushing 0.4, jaw_hub 1.0 |
| printed: shells | 68 | 68.8 | head_hood (constant wall 1.8) |
| printed: bill | 8 | 13.9 | upper_bill 5.6, lower_bill 8.3 |
| fasteners | 8 | 5.0 | M2.5 + nuts, M2 screws (modelled in their holes; + 4 x M2 retainer screws, D3) |
| wiring | 6 | 5.2 | 8 harness tubes (incl. neck_pitch in-tube in the trunk link, 0.8) |
| **Total** | **202.0** (203.5 after D3) | **202.7** | |

Printed 132.9 g vs 130 (+2.9, bills over, neck/tilt/turn under), fasteners and wiring under. Robot total with the
P2a/P2b stubs: 1845.4 g (x 1.05 = 1938 g <= 1950).

## 7. Collision results (`cad/check.py`, tr35_b2)

Full run (all modules, P2a/P2b still stubs, head limits of sec. 5; 511 s with other FreeCAD jobs running):

| Section | Result |
|---|---|
| 1 single joint (limits, 7 steps, 107 poses) | **0** (head joints vs everything, incl. neck -110..40 against trunk and leg stubs, harnesses included) |
| 3 same link (HOME) | **0** (all head-module parts incl. screws, horns, harnesses) |
| 4 adjacent pairs (5 x 5 required, module scope, 300 poses) | **0** counted; 10 modelled leg contacts (P2-0 baseline); module-external info: 3 x head_hood vs trunk stub at (neck +40, head_pitch -20) (spec v3.1 head <-> trunk pair), the rest legs vs trunk (P2a/P2b) |
| 5 leg-leg (32 poses) | 0 counted, 4 modelled (P2-0 baseline) |
| 6 envelopes | intrusion **0**, envelope-envelope **0**; containment 4 items = the two requests of sec. 9 (neck stem 1404 mm3, jaw parts) |
| 2 random (information) | head-module entries: lower bill vs trunk front electronics (bus_caps, main_switch, ina226, mp1584, fuse) and upper bill vs left hip_roll at combined neck-forward + head-down poses; neck_link vs lower bill (sec. 9 item 3) |

Head-module filter run (`OPENDUCK_CHECK_LINKS=head`): sections 1, 3, 4 at 0, section 6 as above.
`tests/cad/test_head_tr35_fc.py` repeats the limits, five required-range 4-joint combos (pitch, yaw, roll, mouth),
the bumper sequence, the passive-support gaps and the MJCF geom.

## 8. Harness routing (P2c)

Cross points and diameters are the frozen P2-0 values (`params.harness`). The inter-module neck crossing keeps the P2-0
stubs and is only extended inside the neck (`out_post`). For the three head-internal crossings the P2-0 short in/out
stub points are replaced by the link owner's routing (`params.head.harness_route.<joint>.in/out`): the P2-0 head_yaw
in-stub from (33.332, -10, 115) passes 2.0 mm from the head_pitch MG90S near-end corner and sweeps into it at head_pitch
38..54 deg (0.07 mm3 for the bare stub, 1.1 mm3 once any routing bend is added). The new head_yaw in-route rises along
y -24 to z 120 and reaches the cross point at r >= 12.6 mm from the pitch axis. Consecutive tubes of one bundle in the
same link end collinear (zero-volume contact). The yaw out-route stays on the -y side (r 20) so that at yaw +-80 it
never reaches the head_tilt rotor (+y).

## 9. Interface change requests and open points

1. **Neck envelope, bumper stem** (INTERFACE CHANGE REQUEST): the frozen bumper zone x 51..59 does not touch the bridge
   zone (x <= 45.332). Add `["box", 45.332, 51.5, -9.5, 9.5, 84.0, 98.0]` to `params.iface.envelopes.neck`.
   Now outside: `neck_link` 1403.7 mm3.
2. **Jaw envelope** (INTERFACE CHANGE REQUEST): the frozen jaw box (x 15..52, y 50..57, z 140..164) only holds the
   SG90 horn plate; no lower bill can be driven inside it. Replace `params.iface.envelopes.jaw` with
   `[["box", 15, 52, 50, 57, 140, 164], ["cyl", "y", 33.46, 152.045, 0, 7.0, 56.5, 62.4],
   ["box", 26, 92, 58.9, 62.4, 125, 159.5], ["box", 26, 92, -62.4, -58.9, 125, 159.5],
   ["cyl", "y", 33.46, 152.045, 0, 3.0, -58.9, -52.0], ["box", 81.5, 116.5, -62.4, 62.4, 125, 135]]`.
   Now outside: `lower_bill` 6940 mm3, `jaw_hub` 292 mm3, `jaw_hub_screws` 17 mm3.
   Verified with a scratch copy of the config carrying requests 1 and 2: `check.py` (head filter) gave sections 1, 3
   and 4 at 0 and section 6 intrusion 0, envelope-envelope 0, containment 35 mm3 (a first draft of request 1 ended at
   x 51.0, under the zone); with the stem box to x 51.5 as written above the containment test
   (`check.iface_envelopes`) gives 0 for every head-module part.
3. **New contact pair** (INTERFACE CHANGE REQUEST, spec 3.2.2): jaw <-> neck. head_pitch and mouth are not adjacent,
   so A1 does not count it, but with head_pitch >= ~55 and mouth >= ~28 the open lower bill reaches the neck link
   (head_pitch 60 / mouth 30 / yaw +-34 / roll +-10: up to 124 mm3; mouth 32: 317 mm3; at the widened head_pitch limit
   72 also the trunk's neck servo, up to 537 mm3). Options: MJCF pairs jaw <-> neck and jaw <-> trunk, or a policy /
   firmware interlock mouth <= 25 while head_pitch > 50.
4. Head <-> trunk at (neck +40, head_pitch -20) (hood vs the trunk stub's buck / servo hub / rear bracket) is the
   spec 3.2.2 v3.1 head <-> trunk pair (P2-0 open issue); listed as module-external information in section 4.
5. `cad/export.py`: `model.meta["collision_geoms"]` (Item names) are written as named collision boxes in their link's
   body; only the head generator sets it, so xl330_27 stays byte-identical. P3 owns export and may fold this in.
6. Joint limits widened (sec. 5): neck -110..40, head_pitch -25..72, head_roll -20..16, mouth 0..32 (p2-interface sec. 1:
   P2c may widen within travel with check.py clean). RL constants (rl/config.py) still carry 27 cm values (P5).
7. Head mass 200.2 g is inside the 202 g budget; the bills are 5.9 g over their 8 g line.

## 10. Printing

All parts < 256 mm. Flat-face orientations: `head_tilt_bracket` (rotor disc down, table vertical), thrust plates,
`jaw_hub` (plate down), `jaw_pin_bushing`, `neck_bumper` (TPU). Need supports: `neck_link` (cradle walls and legs on
opposite sides of the bridge), `head_turn_bracket` (ring below and plates above the tab frame), `head_frame` (chin block
below the deck), `head_hood` and `upper_bill` (flat roof when printed open side down; tree supports), `lower_bill`
(right pin overhang).
