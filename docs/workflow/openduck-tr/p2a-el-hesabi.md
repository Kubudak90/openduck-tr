# P2a: TR-35 legs, hand calculation (no bending or axial load on the servo shafts) and build notes

Input: `spec.md` v3.1 (3.1 Mekanik and the required-range table, 3.2.2, A1), `p2-interface.md` (P2-0, frozen),
`research/servo-mechanics.md` (D: cradle + U-link + rear idler). Code: `cad/legs.py` (`build_tr35`, `load_report`,
`print_report`, `PRINT_NOTES`), `cad/servos.py` (P2a section: `ds_stator`, `ds_rotor_cuts`, `ds_idler_screw`,
`ds_thrust_sleeve`), `configs/tr35_b2.json` (`params.legs`, leg rows of `joint_limits_deg`). Tests:
`tests/cad/test_legs_params.py`, `tests/cad/test_legs_tr35_fc.py`. The tables in sections 4 to 6 are generated from the
built model by `legs.load_markdown(legs.load_report(m))` (FreeCAD); the print table in section 7 comes from
`legs.print_report(m)`.

All coordinates: mm, trunk frame (x forward, y left, z up), zero pose, left leg; the right leg is `mirror_y` of the left
(27 cm rule). g = 9.81 m/s2. W = 1.95 kg x g = 19.13 N (hard mass limit; the leg weight is not subtracted, which is
conservative).

## 1. Result

| Done-when item | Result |
|---|---|
| Leg single-joint scan (limits, 7 steps, harnesses included) | **0** collisions (`check.py` section 1, `OPENDUCK_CHECK_LINKS=legs`: 72 poses) |
| Adjacent pairs (5 x 5, required ranges, module scope) | **0** counted (200 poses); 26 pose-pairs listed as modelled contacts (knee +65 / ankle -45 fold, `check.contact_pairs`), none involves a harness |
| Leg-leg (32 poses) | **0** counted; 4 pose-pairs modelled (both feet, pitch -70 / knee -37 toe-in), feet only |
| Same-link overlaps (leg links) | **0** |
| Module envelopes (containment / intrusion / envelope-envelope) | **0 / 0 / 0** |
| Full `check.py` (all links, P2-0 trunk/head stubs) | single joint 0 (107 poses), pairs 0 counted (300), leg-leg 0 (32), same link 0, envelopes 0/0/0; 30 modelled pose-pairs; 741 s |
| Leg mass vs interface budget | **912.9 g** (both legs) vs 913.2 g; printed 2 x 127.5 g vs 2 x 130 g; robot total with the P2-0 trunk/head stubs 1847.7 g x 1.05 = 1940.1 g <= 1950 g |
| Hand calculation | sections 4 to 6: on every DS joint the link moment becomes an A/B support couple and the axial load goes to the case and the idler, so the servo shaft sees only the radial spline reaction (its overhang moment is 7-10 % of the no-idler moment); the ES3054 carries only torque; lowest safety factor 1.4 (hip boss bearing pressure, 2 W drop) |
| Print parts within 256 mm, printable | 7 parts per leg, each one solid, largest 93.8 mm; orientation and supports in section 7 |
| Joint limits | section 3; required ranges inside the limits, limits inside servo travel (with the +-3 deg RL range margin) |
| Golden xl330_27 | 6/6 byte-identical (full run with check.py) |
| tests/cad | all pass (includes `test_legs_params.py` 6, `test_legs_tr35_fc.py` 7) |

## 2. What was built

| Link | Part (kind) | Function |
|---|---|---|
| hip | `hip_yaw_shaft` (metal, Al 6061 d10 x 35.8) | turntable shaft: z -12.0 .. 23.8, through both 6800ZZ inner races (lower z -3..2, upper 19..24). Bottom end in the hip boss with an M3 cross screw (torque + downward axial load); top end with an M3 blind tap (6 deep) and a 1.2 x 2 hacksaw slot |
| hip | `hip_yaw_coupler` (bracket, PETG) | pin/slot coupler: ring seated on the upper inner race (seat r 6.2, 0.3 high), key in the shaft slot, M3 x 8 countersunk screw into the shaft; fork around the ES3054 single arm with 0.3 mm side clearance, 1.0 mm radial float along the arm and 0.5 / 0.6 mm axial gaps: torque only |
| hip | `hip_yaw_screws` (metal) | M3 x 8 DIN 7991 (top) + M3 x 16 cross screw (head +x, tip tapped into the -x boss wall) |
| hip | `hip_bracket` (bracket) | yaw boss r 8.5 (z -12.2 .. -3.5, frozen flange limit) + shoulder r 6.2 under the lower inner race; roll DS cradle: top plate over the servo, inner rail carrying the far (tapped) shelf, near shelf with nut traps, rear wall with the F623ZZ seat; swept slot for the yaw harness (+-13 deg) |
| hip | `hip_roll_servo`, `hip_roll_tab_screws`, `hip_roll_idler_bearing` | roll DS (slides in from +y), 4 x M3, F623ZZ |
| hip_cradle | `hip_cradle_bracket` (bracket) | roll U-link (leg A embeds the roll Al arm, leg B on the roll idler pin, I-section bridge under the roll servo) + pitch U-link (leg A plate on the pitch Al arm, leg B on the pitch idler pin) joined by the front plate; thrust sleeves on both leg A; two lightening windows |
| hip_cradle | `hip_roll_horn`, `hip_pitch_horn` (Al 25T arm), `*_idler_pin` | arms embedded in pockets (2 x M2.5 + M3 centre screw); M3 idler pins with nut and brass spacer |
| thigh | `thigh_bracket` (bracket) | hip_pitch and knee DS cradles: ear-plane frames (shelves + side bars), idler seats on a common strut, posts frame -> strut |
| thigh | `hip_pitch_servo`, `knee_servo`, tab screws, idler bearings | body link of both (sign rule +1) |
| shin | `shin_bracket` (bracket) | knee U-link: leg A (knee arm pocket) + spine down to the ankle seat; ankle DS cradle (frame, seat, X- wall with a web to the seat); U-link bridge = hollow post d12/6 (spine -> knee leg B, beside the ankle servo X+ side, tied to the frame bar); knee leg B with an outer rib |
| shin | `knee_horn`, `knee_idler_pin`, `ankle_servo`, tab screws, idler bearing | |
| foot | `foot_bracket` (bracket) | ankle U-link legs (A embeds the ankle arm, B on the ankle idler pin) on the 5 deg canted foot plate (8 top pockets, floor 1.6 mm) |
| foot | `sole` (TPU 95A, `collision_box`) | name and collision box unchanged (`left_foot_collision`, site `left_foot_sole`) |
| all | `<joint>_harness_in/out` (harness) | frozen crossings + in-link routing (`params.legs.harness_route`); hip_yaw in-tube in the trunk link (P2-0 convention) |

Hip interface as built (left; right mirrored): mating plane z -3.0 (shoulder top), hip flange/boss top z -3.5, boss r 8.5,
shaft d10 z -12.0..23.8, coupler ring z 24.0..26.5 (r 7.5) with the fork up to z 29.6 inside the rotor zone (r <= 15),
ES3054 arm toward -x (`params.trunk.hip_yaw_horn_toward`). Nothing of the legs is above z -3.0 outside the rotor zone.

## 3. Joint limits

Method (`params.legs._limits`): single-joint scan with the other joints at HOME, 1 deg steps outward from the required
range, all leg parts and harnesses, against the P2-0 trunk/head stubs and the module envelopes. Limit = the tightest of:
(a) first counted collision or envelope intrusion minus 3 deg (RL keeps `range = ctrlrange +- 3 deg`,
`rl/config.py JOINT_RANGE_MARGIN_DEG`), (b) the last angle before a leg harness enters the trunk keep-in K_T (check.py
exempts harnesses from the envelope test, so this protects the integration against P2b parts anywhere in K_T),
(c) the ES3054 arm stays inside the frozen hip rotor zone (arm box y +-7 about the axis: up to 21 deg), and never inside
the required range.

| Joint | Required | First collision (1 deg) | Harness enters K_T | Limit | Servo travel |
|---|---|---|---|---|---|
| hip_yaw | -10..10 | -16 (yaw harness_in <-> hip bracket slot) / +27 (ES3054 horn into K_T) | -11 (hip_pitch_harness_out) | **-10..21** | -72.5..72.5 |
| hip_roll | -20..20 | -24 (hip_pitch_harness_out <-> roll servo) / +23 (hip_pitch_harness_in <-> hip bracket, stub PCA9685) | +21 (hip_pitch_harness_in, frozen in-point (5.1, 44.5, -10)) | **-20..20** | -90..90 |
| hip_pitch | -70..90 | none over the travel | none | **-70..90** (spec) | -75.6..104.4 |
| knee | -37..65 | -88 (thigh <-> shin) / +81 (foot <-> hip cradle) | none | **-84..77** | -90..90 |
| ankle | -45..57 | -69 / +68 (shin <-> foot) | none | **-65..64** | -90..90 |

hip_roll keeps the required range: its upper margin to the first collision is 2-3 deg and that collision is a harness
only (harnesses are not in the MJCF, so the +3 deg RL margin has no CAD collision of a modelled part).

## 4. Load cases and method

| Case | Vertical | Horizontal (x and y, each +- independently) | Centre of pressure |
|---|---|---|---|
| Stance (worst case) | W on one foot | 0.2 W | anywhere on the sole: centre and the 4 corners |
| Drop (~2 x robot weight) | 2 W on one foot | 0.3 x 2 W | sole centre |

The ground force acts at the sole underside in the stand (HOME) pose. For every leg joint the force and its moment about
the joint origin are split into the axial force (along the joint axis), the torque about the axis (taken by the servo
gear train) and the bending moment Mp perpendicular to the axis. Maxima over all sign combinations and pressure centres.

## 5. DS joints: idler load path

Every DS joint (hip_roll, hip_pitch, knee, ankle) is a cradle (stator: servo body by 4 x M3 tab screws + F623ZZ seat
behind the servo bottom) and a U-link (rotor) with two supports on the joint axis:

* A: the Al 25T arm on the spline, embedded in leg A (canonical y = -2.85, hub centre);
* B: leg B on the M3 idler pin running in the F623ZZ inner race (y = -51.3); span A-B = **48.45 mm**.

The link bending moment Mp is therefore carried as a force couple R_A / R_B across 48.45 mm instead of as a moment on the
spline. The servo shaft sees only the radial reaction R_A at the spline; the only shaft bending left is R_A times the
3.55 mm overhang between the spline centre and the servo's own output bearing (boss top), shown next to the moment the
spline would carry without the idler (single support):

| Joint | Case | Axial F (N) | Axis torque (N.m) | Mp (N.m) | R_A spline (N) | R_B idler (N) | Spline overhang moment (N.m) | Without idler (N.m) |
|---|---|---|---|---|---|---|---|---|
| hip_roll | stance | 3.8 | 1.89 | 1.76 | 35.1 | 35.1 | 0.124 | 1.70 |
| hip_roll | drop | 11.5 | 3.05 | 2.65 | 52.2 | 52.3 | 0.185 | 2.53 |
| hip_pitch | stance | 5.5 | 1.24 | 1.44 | 37.4 | 28.5 | 0.133 | 1.38 |
| hip_pitch | drop | 14.8 | 1.60 | 2.10 | 57.5 | 41.0 | 0.204 | 1.99 |
| knee | stance | 5.5 | 1.62 | 1.36 | 33.4 | 26.9 | 0.119 | 1.30 |
| knee | drop | 14.8 | 2.20 | 1.80 | 46.0 | 34.7 | 0.163 | 1.68 |
| ankle | stance | 5.5 | 0.98 | 1.37 | 24.7 | 27.2 | 0.088 | 1.32 |
| ankle | drop | 14.8 | 0.72 | 1.54 | 26.1 | 29.4 | 0.093 | 1.43 |

The idler removes about 90-93 % of the shaft bending moment; what is left is the unavoidable spline overhang of any servo
output (0.09-0.20 N.m), the same as a servo arm pushing a linkage. If the spline/clamp were taken as fully moment-rigid,
the moment splits by stiffness between the 48.45 mm A-B couple and the servo's internal bearing pair (a few mm apart),
so the couple still takes the large majority; the drop-test in Step 0 checks the real behaviour.

Axial load never reaches the shaft:

* toward the servo: the U-link leg A thrust sleeve (`servos.ds_thrust_sleeve`, r 8.2..10.7 round the hub, 0.2 mm gap)
  bears on the servo case top, i.e. the load goes into the servo case and the cradle;
* away from the servo: the leg B nut presses the brass spacer (OD 5, inner race only) on the F623ZZ inner race; the
  outer race sits on the 1.0 mm lip of the seat pocket (stator).

The U-link floats axially by at most these running gaps (0.2 + 0.05 mm). Step 0 must confirm that the DS output shaft
end play is larger than the sleeve gap; if not, a 0.1 mm PTFE shim under the sleeve sets it.

The axis torque is carried by the servo gear train (that is its job); in the drop case hip_roll sees 3.05 N.m, just over
the 2.88 N.m stall: the servo is back-driven, the arm and screws are sized for the larger value (section 6).

## 6. hip_yaw turntable, screws, arms and printed parts

Turntable: two 6800ZZ, centres at z -0.5 and 21.5 (spacing 22 >= 20). Upward load (stance, drop): hip boss shoulder ->
lower inner race -> housing. Downward load (leg hanging): cross screw -> shaft -> top countersunk screw -> coupler ring
-> upper inner race -> housing. Bending: the two bearing reactions. The ES3054 output gets only the yaw torque through
the fork (plus its own drive reaction T / r_arm <= 0.343 / 0.013 = 26 N at stall, normal servo-arm duty). Under the drop
moment the shaft tilts by the bearing play plus housing compliance (estimate <= 0.1 mm per bearing), moving the coupler
<= 0.15 mm at the arm: less than the 0.3 mm side and 1.0 mm radial float, so the turntable deflection does not load the
ES3054 shaft.

| Turntable | Axial F (N) | Yaw torque (N.m) | Mp at lower bearing (N.m) | R lower (N) | R upper (N) | M at boss top (N.m) | M at cross hole (N.m) |
|---|---|---|---|---|---|---|---|
| stance | 19.1 | 0.47 | 2.57 | 122.2 | 117.0 | 2.56 | 2.54 |
| drop | 38.3 | 0.64 | 4.24 | 207.9 | 192.6 | 4.19 | 4.15 |

Checks in the drop case (limit / value = safety factor). Catalogue values: 6800ZZ C0r 0.83 kN, F623ZZ C0r 0.18 kN;
Al 6061-T6 yield 240 MPa; steel 8.8 yield 640 MPa (shear 0.6 x); PETG bearing (crushing) 50 MPa, across-layer
tension 25 MPa. Torque for screws and arms: max(drop axis torque, DS stall) = 3.05 N.m; ES3054 0.64 N.m.

| Check (drop, 2 W) | Value | Limit | Safety |
|---|---|---|---|
| 6800ZZ static load, max(R) + 0.5 Fa (N) | 227.0 | 830 | 3.7 |
| shaft bending at the lower bearing (MPa) | 43.2 | 240 | 5.6 |
| shaft bending, cross-hole net section, full moment (MPa) | 92.4 | 240 | 2.6 |
| hip boss bearing pressure 6M/(dL^2) + F/(dL), L = 8.5 (MPa) | 35.0 | 50 | 1.4 |
| cross screw shear, T/d over the M3 core (MPa) | 14.3 | 384 | 26.8 |
| F623ZZ static load, R_B + 0.5 Fa (N) | 59.7 | 180 | 3.0 |
| idler M3 pin bending, R_B x 4.5 mm on the core (MPa) | 175.6 | 640 | 3.6 |
| tab screw PETG bearing, T/49.5/2 + (R_A + Fa)/4 on 3 x 6 (MPa) | 2.7 | 50 | 18.4 |
| arm M2.5 screw shear if all torque went through the screws (MPa) | 22.1 | 384 | 17.4 |
| arm M2.5 PETG bearing, 2.5 x 2.5 (MPa) | 12.0 | 50 | 4.2 |
| arm pocket side-wall pressure (arm form fit, triangular) (MPa) | 5.8 | 50 | 8.6 |
| leg B PETG bearing around the M3 pin (MPa) | 4.4 | 50 | 11.5 |
| idler seat PETG bearing around the F623ZZ (MPa) | 1.7 | 50 | 28.7 |
| hip_roll U-link leg B, worst section (in-plane R_B + out-of-plane axial) (MPa) | 10.8 | 25 | 2.3 |
| hip_pitch U-link leg B (MPa) | 8.7 | 25 | 2.9 |
| knee U-link leg B at the post (with the outer rib) (MPa) | 10.6 | 25 | 2.4 |
| ankle U-link leg B (MPa) | 6.8 | 25 | 3.7 |
| knee U-link post d12/6, bending + torsion, von Mises (MPa) | 13.3 | 25 | 1.9 |

The weakest item is the PETG hip boss that holds the shaft against the drop moment (SF 1.4; stance 21 MPa, SF 2.3). Its
engagement is limited to 8.5 mm by the roll servo top 0.2 mm below the boss (frozen `j_hip_roll`). Print it with
100 % infill around the bore (`PRINT_NOTES`). If the Step 0 drop test shows boss creep, the options are an Al sleeve
in the boss or a longer engagement (needs the roll servo lower: an interface change). The bracket members of the thigh
and shin frames carry the leg loads together with the clamped DS cases, as in commercial servo bracket kits; they are not
part of this note apart from the idler-side members above.

## 7. Print parts (left leg; right = mirror)

Bed 256 x 256 x 256 mm. "Up" = the body axis pointing up while printing (`print_report`: among axis-aligned
orientations, plus the canted sole plane for the foot, the one with bed contact >= 100 mm2 and the largest bed minus
overhang area; overhang = faces more than 45 deg from vertical facing down, including hole roofs and short bridges).

| Part | Size (mm) | Mass (g) | Up | Bed (mm2) | Overhang (mm2) | Note (`legs.PRINT_NOTES`) |
|---|---|---|---|---|---|---|
| hip_bracket | 32.4 x 32.5 x 58.1 | 12.2 | -y | 318 | 545 | outer (+y) side on the bed; supports under the roll-servo top plate and the yaw boss; 100 % infill around the yaw bore |
| hip_yaw_coupler | 20.5 x 15.0 x 7.7 | 0.5 | +x | 14 | 41 | fork end (-x) or tine tops on the bed, supports under the ring |
| hip_cradle_bracket | 69.8 x 93.8 x 62.6 | 32.0 | +z | 1012 | 2020 | U-bridge underside on the bed; supports under the roll legs' upper halves and the pitch U front plate |
| thigh_bracket | 85.9 x 32.1 x 64.6 | 17.7 | -y | 1543 | 1094 | idler-seat face (+y) on the bed; supports under the ear-plane shelves and posts |
| shin_bracket | 56.2 x 63.0 x 85.3 | 23.7 | +y | 645 | 2216 | knee leg A outer face (-y) on the bed; supports under the ankle cradle frame, the post web and the B leg |
| foot_bracket | 69.2 x 64.2 x 40.9 | 22.8 | plate | 4361 | 298 | plate underside on the bed (print the canted plate flat); pocket roofs only |
| sole (TPU 95A) | 69.2 x 64.1 x 10.4 | 18.4 | plate | 4009 | 271 | flat on the bed, no supports |

Every part is a single solid (`test_legs_tr35_fc`). Mass ladder step 2 (skeletal U-links and cradles) is applied where it
does not cost stiffness: hip_cradle front-plate windows, I-section roll bridge, hollow hip rail, hollow knee post, foot
plate pockets; the thigh is a frame (shelves, bars, strut) rather than a plate.

Leg mass (both legs, g; budget from `p2-interface.md` sec. 8): DS servos 520.0 (520), Al arms + ES horns 42.0 (42),
metal (idlers, idler pins, tab screws, shafts, shaft screws) 68.9 (67.2 = idlers 27.2 + shafts 16 + fasteners 24),
harness 27.1 (24), printed 254.9 incl. soles (260), **total 912.9 (913.2)**. The two hip_yaw in-tubes (2.8 g) sit in
the trunk link and count in the trunk module, as in P2-0.

## 8. Scans

`OPENDUCK_CHECK_LINKS=legs` (this worktree, P2-0 trunk/head stubs): single joint 0 (72 poses), pairs 0 counted
(200 poses; 26 modelled pose-pairs), leg-leg 0 counted (32; 4 modelled), same link 0, envelopes 0/0/0, 24 harness
items, 594 s. Full `check.py` (all links): single joint 0 (107 poses), random 25 / 40 collision-free (information),
same link 0, pairs 0 counted (300 poses; 26 modelled pose-pairs), leg-leg 0 counted (4 modelled), envelopes 0/0/0,
741 s.

Fold (knee +65 / ankle -45, modelled contact foot <-> hip_cradle / thigh): first contact on three paths from HOME is
always printed on printed: linear path at knee 50 / ankle -28.7 and knee-first at ankle -12, hip_cradle_bracket <->
foot_bracket; ankle-first at knee 43.8, hip_cradle_bracket <-> sole. Deeper in the fold the foot also overlaps the
hip_pitch horn, servo and tab screws (listed as modelled contacts, never a harness).

Module-external info (section 4 of check.py, leg folded into the body at hip_roll +20 / hip_pitch 50..90, against the
P2-0 stub electronics): harness <-> electronics contacts remain at the hip_pitch out/in and knee in/out tubes (PCA9685,
servo hub, main switch, LiPo alarm, fuse, latch, buck); they are MJCF body <-> limb pairs (spec 3.2.2) and depend on
P2b's real layout.

## 9. Trial integration against P2b / P2c (scratch copy, not merged)

A scratch tree with this branch plus `cad/trunk.py`, `cad/electronics.py`, `params.trunk` (P2b, 5b6695c) and
`cad/head.py`, `cad/export.py`, `params.head` + head limits (P2c, 53ca0ef), `OPENDUCK_CHECK_LINKS=legs`:

* pairs 0 counted (200 poses), leg-leg 0 counted, same link 0, envelopes 0/0/0;
* single joint: **2 collisions**, `left_hip_roll=20` and `right_hip_roll=-20` (required extremes): `trunk_base` <->
  `<side>_hip_pitch_harness_in`, 5.7 mm3 at x 2.4..7.9, y 36.0..38.4, z -3.75..-1.4. The cause is the frozen hip_pitch
  crossing in-point (5.128, 44.5, -10): at roll +20 the d 6.5 tube around it rises to z -1.4, 1.6 mm inside the trunk
  keep-in K_T (z >= -3), where P2b's base plate sits (the P2-0 envelope test exempts harnesses, so the stubs did not
  show it). No routing inside the frozen interface avoids it, because the tube must pass through that point.

**Interface change request**: `params.harness.crossings.hip_pitch.in` from `[[5.128, 44.5, -10.0]]` to
`[[5.128, 44.5, -13.5]]` (12.4 -> 8.9 mm above the pitch axis). With it, the tube top stays 1.5 mm below K_T at roll
+20 (0.7 mm at +23). The same trial with the change gives single joint 0 (left leg, 7 steps, roll +-20 included), pairs 0
counted, leg-leg 0, envelopes 0/0/0. Nothing else in P2a depends on the point (the in-link route joins it from
(3, 44.5, -17.5)).

Other module-external contacts in the trial (section 4 info, leg folded into the body at roll +20 / pitch 50..90):
harness and shin/foot parts against `trunk_base`, `torso_shell_*`, `belly_duct`, `buck_hood` and electronics; these are
the MJCF body <-> limb pairs of spec 3.2.2.
