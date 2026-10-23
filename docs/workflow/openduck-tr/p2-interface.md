# P2 interface contract (P2-0, frozen)

Input: `spec.md` v3 (3.1, 3.2.1-2, A1), `plan.md`, `review.md`, `research/servo-mechanics.md`, `research/power-electronics.md`.
Source of truth: `configs/tr35_b2.json` (numbers) + `cad/servos.py` (servo envelopes). This file explains them.
P2a (legs), P2b (trunk + electronics) and P2c (neck + head) run in parallel from this state. Anything listed as
FROZEN below is an interface: changing it needs a new P2-0 round (all three steps rebuild against it).

All coordinates: mm, trunk frame (x forward, y left, z up), **zero pose**, left side (right leg = mirror in y; Y-axis
joints have their axis y-component negated and limits `(-hi, -lo)`, as in the 27 cm model).

## 1. What is frozen where

| Item | Where | Owner after P2-0 |
|---|---|---|
| Joint names, order, axes, IDs, sign rule, keyframe `stand`, `root`, sites, `<link>_collision`, classes | RL contract (unchanged) | nobody |
| `j_*` joint origins, `servo_face`, `servo_dir`, `home_deg`, `joint_limits_deg`, `required_deg` | `configs/tr35_b2.json` | FROZEN (P2a/P2c may *widen* limits within servo travel if `check.py` stays clean, and report it) |
| `params.turntable`, `params.idler`, `params.harness`, `params.iface` | `configs/tr35_b2.json` | FROZEN |
| `params.legs` | config | P2a |
| `params.trunk` | config | P2b |
| `params.head` | config | P2c |
| Servo registry + envelopes | `cad/servos.py` | P2a may add (others read only) |
| Geometry helpers (`tube`, `harness_paths`, `envelope_solid`, ...) | `cad/geom.py` | FROZEN (read only) |
| Model core, export, check | `cad/openduck.py`, `cad/export.py`, `cad/check.py`, `cad/build_config.py` | FROZEN for P2 (P3 owns export) |
| Generators | `cad/legs.py` / `cad/trunk.py` + `cad/electronics.py` / `cad/head.py` | P2a / P2b / P2c; each replaces only its own `build_tr35_stub` (registered as `GENERATORS["tr35_b2"]`) |

## 2. Servo envelopes (`cad/servos.py`)

Canonical frame of every servo: +Y = shaft toward the horn, +Z = toward the far end, X = Y x Z (width).

* XL330: origin at mid stack (legacy, byte-identical 27 cm). horn_face_y = 14.5.
* PWM types (DS32XX, ES3054, MG90S, SG90): **origin on the shaft axis in the outer face of the horn actually used**
  (bracket datum). `horn_face_y = 0`; case bottom (rear face) at `Y = -horn.h[1]`.
* `m.servo_frame(key)` = (origin = `servo_face[key]`, horn dir, far dir). `m._servo(key, mirror)` -> (body, horn);
  `m.servo_parts(key, mirror, horn_toward)` -> body, horn (arm turned toward `horn_toward`), and for DS
  `idler_seat` (stator), `idler_bearing` (stator), `idler_pin` (rotor).
* All canonical solids are symmetric about X = 0 (holes, gland, arms on the X = 0 plane), so `mirror_y` gives a valid
  (rotated) right-side servo. Tested in `tests/cad/test_servo_geom_fc.py`.

| Type | Body l x w x h (case / tower) | Horn used (outer face h above bottom) | Tabs: span / under_h / t; holes | Rear | Mass g |
|---|---|---|---|---|---|
| DS32XX | 40 x 20 x 40.4, boss d11.8 to 41.5, 25T spline d6 to 45.0 | Al 25T clamp arm, hub d15 (42.2..47.9), single arm +Z, t 3, holes d2.6 at r 20/24 (2 x M2.5 into the link), M3 centre screw. **Embedded in the link** (`servos.horn_pocket`) | 54.5 / 27.7 / 3.2; 4 x d4.5 on 49.5 x 10, open slots | no boss: F623ZZ idler in the cradle (below) | 65 + 5 |
| ES3054 | 28.45 x 13 x 26.7, 23T d5.9 to 31.1 | plastic **single** arm (est.), hub d8 (27.4..32.4), arm r13 / tip r2.5, captured in a printed coupler (pin/slot) | 38.35 / 17.8 / 1.9; 2 x d4.1 pitch 35 | turntable (no idler) | 20 + 1 |
| MG90S | 22.8 x 12.2 x 22.5 / tower 28.4, 21T d4.8 to 32.5 | plastic double arm (est.), hub d7 (29.0..33.6), arms r14, captured in a printed pocket + 2 screws | 32.1 / 18.5 / 2.4; 2 x d2 pitch 27.8 | none | 13.4 + 0.5 |
| SG90 | 22.7 x 12.2 x 22.1 / tower 27.0, 20T d4.9 to 30.3 | plastic double arm (est.), hub d7 (27.5..31.6), arms r14 | 32.3 / 17.0 / 2.4; 2 x d2 pitch 27.85 | none | 9 + 0.5 |

Cable gland: near-end face, centred (DS: h 8.55, 6.8 x 4.5, sticks out 5.4). `servos.datums(t)` returns every plane and
point (horn face, case top, tower top, rear face, tab planes, tab holes, cable exit, idler axis). Fields marked in
`rec["estimated"]` are drawing/estimate values to be confirmed with calipers in Step 0.

**DS rear idler stack** (`servos.idler_stack`, planes from the rear face outward): 0.4 gap | cradle bottom plate
(stator) 4.0 with a d10.1 pocket 3.0 deep (1.0 lip) | F623ZZ, centre 3.4 below the rear face, flange outside |
0.5 running gap | U-link leg B (rotor) 4.0 with M3 nut trap | M3 head 3.0. Idler axis = `datums()["idler_axis"]`.

**Arm direction** is free (rotate about the shaft); the stub uses `params.legs.horn_toward` / `params.head.horn_toward`.

## 3. Frozen TR-35 layout

Sagittal (x, z) points = 27 cm x 1.282 (keeps the HOME flat-sole condition hp - kn + an = 0 and the crouch-family
angles). Only lateral offsets and stack positions along own axes were chosen (`p2-0` fit study, sec. 5).

| Joint | Type | Origin (left) | Axis | Parent -> child | Servo body in | Horn + idler pin in | Horn face | Case top / rear face | Idler axis | Cable exit (gland) | Owner |
|---|---|---|---|---|---|---|---|---|---|---|---|
| hip_yaw | ES3054 | (7.69, 12.00, -3.00) | 0 0 -1 | trunk -> hip | trunk | hip | z = 27.00 | z = 32.70 / 59.40 | turntable | (15.69, 12.0, 51.4) | P2b cage + bearings, P2a shaft/coupler/plate |
| hip_roll | DS32XX | (36.10, 26.00, -22.43) | 1 0 0 | hip -> hip_cradle | hip | hip_cradle | x = 36.10 | x = 28.60 / -11.80 | (-15.20, 26.0, -22.43) | (-3.25, 26.0, -7.03) | P2a |
| hip_pitch | DS32XX | (5.13, 51.00, -22.43) | 0 1 0 | hip_cradle -> thigh | thigh | hip_cradle | y = 51.00 | y = 58.50 / 98.90 | (5.13, 102.30, -22.43) | (20.53, 90.35, -22.43) | P2a |
| knee | DS32XX | (-40.77, 51.00, -50.64) | 0 -1 0 | thigh -> shin | thigh | shin | y = 51.00 | y = 58.50 / 98.90 | (-40.77, 102.30, -50.64) | (-36.78, 90.35, -65.52) | P2a |
| ankle | DS32XX | (-40.77, 104.80, -104.48) | 0 1 0 | shin -> foot | shin | foot | y = 104.80 | y = 97.30 / 56.90 | (-40.77, 53.50, -104.48) | (-30.87, 65.45, -116.28) | P2a |
| neck_pitch | DS32XX | (33.33, 0.00, 41.54) | 0 -1 0 | trunk -> neck | trunk | neck | y = -26.90 | y = -19.40 / 21.00 | (33.33, 24.40, 41.54) | (33.33, 12.45, 56.94) | P2b cradle + idler seat, P2c U-link + bumper |
| head_pitch | MG90S | (33.33, 0.00, 105.64) | 0 1 0 | neck -> head_tilt | neck | head_tilt | y = 16.80 | y = 5.70 (tower 11.6) / -16.80 | - | (33.33, -13.8, 113.0) | P2c |
| head_yaw | MG90S | (33.33, 0.00, 129.61) | 0 0 1 | head_tilt -> head_turn | head_turn | head_tilt | z = 129.61 | z = 140.71 / 163.21 | - | (33.33, 7.4, 160.2) | P2c |
| head_roll | MG90S | (10.38, 0.00, 148.20) | -1 0 0 | head_turn -> head | head | head_turn | x = 10.38 | x = -0.72 / -23.22 | - | (-20.22, 7.4, 148.2) | P2c |
| mouth | SG90 | (33.46, 35.90, 152.04) | 0 1 0 | head -> jaw | head | jaw | y = 54.48 | y = 44.98 / 22.88 | - | (26.06, 25.88, 152.04) | P2c |

`servo_dir` (horn, far): hip_yaw (-z, -x), hip_roll (+x, -z), hip_pitch (-y, -x), knee (-y, (-0.259, 0, 0.966): far end
15 deg back), ankle (+y, (-0.643, 0, 0.766): 40 deg back), neck (-y, -z), head_pitch (+y, -z), head_yaw (-z, -y),
head_roll (+x, -y), mouth (+y, +x). Sign rule check (`servo_signs`): +1 for all 15 joints.
Bracket datum faces = the horn-face plane (rotor side: link plate on/around the embedded horn) and the rear-face plane
(stator side: cradle bottom plate starts 0.4 below it); tab planes (`datums()["tab"]`) carry the M3 cradle screws.

hip_pitch horn offset +14.4 deg (one 25T tooth): servo travel -75.6..104.4 covers -70..90. Neck neutral -25 deg: travel
-115..65 covers -90..40. Limits (`joint_limits_deg`) = required ranges, except hip_pitch -70..90 and head_yaw +-80.

**HOME / stand** (27 cm angles): hip_roll -5, hip_pitch -26.237, knee -0.283, ankle 25.954 (right mirrored), neck 20,
head_pitch 20. Results with the P2-0 stubs:

| Quantity | Value |
|---|---|
| Head top at the stand keyframe | **350.45 mm** (target 350 +- 10) |
| Trunk origin z at HOME (`ground_offset`) | 152.53 mm |
| HOME foot lateral spacing (sole centres) | 133.4 mm (y = +-66.7; inner edges +-34.0) |
| Overall size at HOME (x, y, z) | 189.9 x 223.4 x 350.5 mm (`summary.json`; zero pose height 341.0) |
| Stance vs. 27 cm x 1.282 | +25 mm (feet centred under the DS stacks; "wide stance") |

## 4. Module boundaries

### 4.1 Hip (trunk <-> legs), turntable

Yaw axis (7.692, +-12.0). Turntable stack (`servos.turntable_stack(params.turntable)`), absolute z:

| z | Feature | Owner |
|---|---|---|
| -3.0 | **mating plane**: housing bottom face (trunk) / hip flange top at -3.5 (0.5 running gap) | interface |
| -3.0 .. 2.0 | lower 6800ZZ (10 x 19 x 5) | trunk housing (outer race), legs shaft (inner race, shoulder below) |
| 19.0 .. 24.0 | upper 6800ZZ; centre spacing 22 (>= 20) | idem; web between the bearings is trunk |
| 24.0 .. 26.5 | shaft nut / slotted driver (r <= 8) | legs |
| 27.0 | ES3054 horn outer face (horn down); horn hub 27..32, single arm toward -x (r 15.5) | legs (horn + coupler) |
| 32.7 .. 59.4 | ES3054 case (horn side .. bottom); tabs z 39.7..41.6 | trunk (cage) |

* Trunk provides: housing r <= 12.5 around each axis (two bores d19, the two housings meet at y = 0 - make one part),
  ES3054 cage (tabs + case, cage top <= 61.4), mating face z = -3.0, cable opening at the harness entry (sec. 6).
* Legs provide: shaft d10 (z -3.5..26.5) with shoulder under the lower inner race and nut on the upper inner race,
  pin/slot coupler capturing the ES3054 single arm (torque only, radial/axial float), hip flange/plate below z = -3.5,
  hip_roll cradle. **Rotor zone** the trunk must keep free: cyl r 5.0 (z -4..24); cyl r 15.0 (z 24..28.3) and
  annulus r 2.95..15.0 (z 28.3..32.2, the ES3054 spline stays trunk); arm box x -9.81..-2.31, y +-7 about the axis.
* 608ZZ does not fit this frozen stack (2 mm web between bores, 1 mm left for the nut): 6800ZZ is frozen
  (mass ladder step 4 is therefore already applied). Verify 6800ZZ stock in P3.
* Load path (hand calc for A1, P2a): leg weight and moments -> shaft -> two bearings 22 mm apart -> housing; the ES3054
  output carries only torque through the pin/slot (no bending/axial load on its shaft).

### 4.2 Neck (trunk <-> head)

* Trunk (P2b) owns the neck DS body + its cradle (tabs y -9.85..-6.7, z 4.3..58.8) and the **idler seat**: cradle plate
  y 21.4..25.4 with F623ZZ y 22.4..26.4 (flange 25.4..26.4), all within r 7.05 of the neck axis.
  Keep-in box for it: x 26.2..40.5, y 21.0..26.65, z 34.4..48.7. Spline (stator) y -24.0..-20.5, r 3.0.
* Head (P2c) owns from the horn/idler outward: horn hub (y -26.9..-21.2, embedded arm y -26.9..-23.9), U-link leg A
  (outer face y >= -31.4), leg B + M3 pin (y 26.9..33.9), bridge and the head_pitch MG90S cradle, and the
  **chin/forehead bumper**: an Item `neck_bumper` in link `neck` with a `collision_box` (P3 exports it as MJCF geom
  `neck_bumper`; do **not** add `neck` to `collision_links`: RL getup adds its own `neck_collision`).
* Neck harness passes outside leg A at y = -34.4 (sec. 6); trunk shell must leave |y| <= 36 open around the neck sweep.

### 4.3 Module envelopes and the swept-volume rule

`params.iface.envelopes` (boxes / axis cylinders, `geom.envelope_solid`, zero-pose trunk coordinates, moving with the
link):

* **Trunk keep-in K_T** (P2b): union of boxes minus the hip rotor zones. P2b parts must lie inside it.

  | Box | x | y | z |
  |---|---|---|---|
  | core | -60..46 | -21..21 | -3..62 |
  | low sides | -60..21 | +-(21..38) | -3..24 |
  | mid sides (behind the neck sweep) | -60..-14 | +-(21..40) | 24..62 |
  | front low sides | 21..46 | +-(21..38) | -3..20 |
  | back | -92..-60 | -30..30 | -3..64 |
  | belly | -66..-40 | -36..36 | -24..-3 |
  | deck | -42..-16 | -40..40 | 62..72 |
  | neck idler seat | 26.2..40.5 | 21..26.65 | 34.4..48.7 |
  | neck spline | cyl y, r 3.0 | -24.05..-21.0 | |

  Volume 625.6 cm3 (fixed contents 69.6 cm3, electronics boxes 314.0 cm3, 242 cm3 left for chassis/cages/shell/air).
* **Head-module keep-in K_H** (P2c), per link: `neck` (leg A: disc r 16 about the neck axis at y -31.4..-24.0 and
  annulus r 3..16 at y -24.0..-21.1 (spline stays trunk); leg B: disc r 16 at y 26.75..34.5; arm strips x 21.3..45.3,
  z 44.6..118 on both sides; bridge x 21.3..45.3, y -31.4..34.5, z 84..118; bumper zone x 51..59, y +-16, z 88..102),
  `head_tilt` x 21.3..45.3, y -27..26, z 88..136; `head_turn` x 3..46, y -24..20, z 118..168; `head` = the 27 cm head
  box x 1.282 (x -39.0..118.3, y +-58.85, z 126.5..201.8); `jaw` x 15..52, y 50..57, z 140..164.
* **Legs** (P2a): everything outside K_T and K_H.
* **Rule** (checked by `check.py` section 6): (a) every non-harness part of a link with an envelope lies inside it;
  (b) over single-joint scans (limits, 7 steps) + leg-leg (32 poses) no part of one module enters another module's
  envelope; (c) envelopes of different modules never intersect over the same scans. The P2-0 stubs give 0 / 0 / 0.
  So each P2 step only needs its own parts inside its own envelope (P2b, P2c) or outside the others (P2a) and
  `check.py` clean on its own module; cross-module collisions are then excluded by construction.

### 4.4 Pair-grid scope (decision needed)

A1 asks for adjacent pairs swept together with 0 collisions. Over the **full required ranges**, two pairs fold whole
chains into the body:

* (hip_roll +20 abduction, hip_pitch +90): the folded thigh/shin tilts inward to |y| ~ 21 at z 28..52 behind the hips;
* (neck_pitch +40, head_pitch -20) and (-90, +60): the head swings over the trunk top or under the chest.

With those combinations the trunk would be ~46 mm wide below z 52 (about 80 cm3 left after the hip-yaw stacks) for
314 cm3 of electronics, so no trunk is possible. The frozen config therefore uses `check.pair_scope = "module"`:
pair grids count collisions inside the moving module (leg pairs: all leg links incl. the other leg; head pairs: head
module), and body-vs-limb contacts are listed as information (they are the MJCF self-collision pairs of spec 3.2.2).
The trunk/head keep-ins (K_T, K_H) are verified against single-joint + leg-leg sweeps. `pair_scope = "all"` gives the
literal reading. **This needs a user decision** (see open issues in the P2-0 report).

## 5. Envelope fit study (`tools/fit_study.py`)

Envelopes only (bodies + ears + boss/spline + gland, horns, DS idler seat/bearing/pin, turntable bearings + housings
+ shaft), HOME + every joint over its required range (7 steps) + every parent-child pair (5 x 5, required) + leg-leg
(32): **438 poses, 0 cross-link collisions**. Joint-internal pairs (body/horn/idler/turntable of the same joint)
excluded from the clearance list. Minimum clearances (bracket rule: >= plate_t + 1 = 5 mm where a bracket passes):

| Min clearance | Pair (links) | Worst pose | Bracket passes? |
|---|---|---|---|
| 2.18 mm | hip_roll tabs (hip) <-> yaw housing / lower bearing (trunk) | hip_yaw -10 | no: hip flange stays r <= 8.5 around the yaw axis, the tabs are clamped from the side; 0.5 running gap at z = -3 |
| 3.79 mm | left ankle idler pin <-> right ankle idler pin | leg-leg toe-in, pitch -70, knee -37 | no (between the legs) |
| 5.53 mm | neck horn (neck) <-> right ES3054 (trunk) | neck +40 | yes: neck leg A at y <= -26.9 vs cage wall at |y| <= 20.5 |
| 7.66 mm | ankle servo (shin) <-> knee servo + gland (thigh) | knee extremes | yes (ankle cradle top) |
| 8.44 mm | hip_roll servo (hip) <-> knee horn (shin) | leg-leg | yes |
| 10.0 mm | ankle servo (shin) <-> hip_roll horn (hip_cradle) | pitch -70, knee 65 | yes |
| 10.9 mm | hip_pitch horn (hip_cradle) <-> hip_roll servo (hip) | yaw + roll extremes | yes (pitch leg A) |
| 11.6 mm | head_yaw servo (head_turn) <-> mouth servo (head) | yaw -17, roll 10 | yes |
| 13.1 mm | hip_pitch servo (thigh) <-> hip_roll servo (hip) | leg-leg | yes |
| 17.9 / 18.0 mm | hip_roll horn / knee servo (legs) <-> yaw housing (trunk) | roll 20 + pitch 90 | yes |

Full report: 438 poses in 305 s (`tools/fit_study.py`); every other cross-link pair >= 11.6 mm.

Iteration history (each run 438 poses): yaw axes y 22.4 -> 12 (neck legs and knee sweep clear the ES3054 stacks),
roll axes 22.4 -> 26 (leg-leg), ES3054 single arm toward -x (double arm reached the neck servo), ankle far end 40 deg
back (knee gland vs ankle tabs 0.9 -> 7.7 mm), turntable 608ZZ -> 6800ZZ (two bores at +-12), mating plane 0 -> -3
(neck at +40 above the ES3054 cages).

## 6. Cable harnesses

Convention (`geom.harness_paths`, `params.harness`): per joint crossing a bundle diameter `d` and minimum bend radius
`bend_r` (= 4 d, dynamic), a crossing point `cross` **on the joint axis** (exception: hip_yaw, where the turntable owns
the axis: the trunk floor opening 17.8 mm from the axis; the hip side moves 3.1 mm over +-10 deg), `in` points in the
parent link and `out` points in the child link. Each crossing gives Items of kind `harness`:
`<joint>_harness_in` (parent link) and `<joint>_harness_out` (child link), solid tubes of diameter d. They are in all
`check.py` scans; the two tubes of one crossing may touch each other, nothing else. Mass = length x area x 0.0017
g/mm3. P2-0 gives short entry/exit stubs; routing inside a link is the owner's.

| Crossing | Bundle (servos) | d / bend_r | Cross point | In (parent) | Out (child) |
|---|---|---|---|---|---|
| hip_yaw (trunk -> hip) | roll, pitch, knee, ankle | 7.5 / 30 | (-9.5, 8.0, -6.5) floor opening | (-9.5, 8.0, 12) | (-9.5, 8.0, -11.5) -> (-2.0, 10.5, -11.5) |
| hip_roll (hip -> hip_cradle) | pitch, knee, ankle | 6.5 / 26 | (-29.0, 26.0, -22.43) behind the roll idler | (-29.0, 26.0, -8) | (-29.0, 40.0, -22.43) |
| hip_pitch (hip_cradle -> thigh) | pitch, knee, ankle | 6.5 / 26 | (5.13, 44.5, -22.43) inboard of leg A | (5.13, 44.5, -10) | (-5.0, 44.5, -30) |
| knee (thigh -> shin) | ankle | 4.0 / 16 | (-40.77, 44.5, -50.64) inboard of leg A | (-30, 44.5, -42) | (-45, 44.5, -64) |
| neck_pitch (trunk -> neck) | 3 x MG90S, SG90 + 4 pots | 5.0 / 20 | (33.33, -34.4, 41.54) outside leg A | (33.33, -34.4, 18) | (33.33, -34.4, 70) |
| head_pitch (neck -> head_tilt) | yaw, roll, mouth | 4.5 / 18 | (33.33, -24.0, 105.64) | (33.33, -24, 90) | (33.33, -24, 118) |
| head_yaw (head_tilt -> head_turn) | roll, mouth (+ yaw servo) | 3.5 / 14 | (33.33, 0, 122) below the yaw horn | (33.33, -10, 115) | (33.33, -20, 122) -> (33.33, -20, 135) |
| head_roll (head_turn -> head) | roll, mouth | 3.5 / 14 | (18, 0, 148.2) in front of the roll horn | (18, -12, 140) | (18, 12, 156) |

The two inter-module crossings are the contract: hip_yaw (P2b opening at the floor point, P2a slot in the hip plate
and the loop), neck (P2b keeps the y = -34.4 channel outside the neck sweep; P2c routes up leg A).

## 7. Shared dict (`m.shared`, written by trunk, read by legs/head)

| Key | Written by | Read by | Content |
|---|---|---|---|
| `hip_yaw_horn` | trunk | legs | `{"left": solid, "right": solid}`: ES3054 horn at its frame (arm toward `params.trunk.hip_yaw_horn_toward`), added to `<side>_hip` |
| `neck_pitch_horn` | trunk | head | DS horn of the neck servo (arm toward `params.trunk.neck_horn_toward`), added to `neck` |
| `neck_pitch_idler_pin` | trunk | head | neck idler rotor stub (leg B boss + M3 head), added to `neck` |

Other cross-module data come from the frozen config (`j_*`, `servo_face/dir`, `turntable`, `idler`, `harness`, `iface`).
`m.meta["imu_item"]` is set by the trunk generator (IMU = `imu_mpu6500`). `m.meta["modules"]` and `m.link_module`
are filled by the model core (link -> trunk/legs/head).

## 8. Mass and COM budget

Hard limit: bottom-up CAD mass x 1.05 <= 1.95 kg, i.e. bottom-up <= 1857 g (target 1.85 kg -> 1762 g).

| Module | Bought / fixed (g) | Printed budget (g) | Fasteners + wiring (g) | Total (g) |
|---|---|---|---|---|
| Trunk (P2b) | 2 x ES3054 40, neck DS 65, F623ZZ 1.6, 4 x 6800ZZ 22, electronics 385 | 175 (chassis, housings, cages, neck cradle, constant-wall shell, deck) | 20 + 25 | 733.6 |
| Legs (P2a, both) | 8 x DS 520, 8 Al arms 40, 2 ES horns 2, 8 idlers (F623ZZ + M3 pin) 27.2, 2 shafts 16 | 2 x 130 (hip plate + roll cradle 20, hip_cradle 26, thigh 34, shin 22, foot plate 18, TPU sole 17 / leg) | 24 + 24 | 913.2 |
| Head (P2c) | 3 x MG90S 40.2, SG90 9, horns 7 (incl. neck DS arm), neck idler pin 1.8 | 130 (neck U + bumper 26, tilt 8, turn 8, frame 12, shells 68, bill 8) | 8 + 6 | 202.0 |
| **Total** | | | | **1848.8 g -> x 1.05 = 1941 g** |

Electronics (385 g, `cad/electronics.py`): 4S LiPo 200, buck 20 A 70 (est.), XL4015 16, hub 30, ESP32 10, 2 x PCA9685 18,
2 x ADS1115 4, MPU6500 2, MP1584 2, INA226 2, fuse 5, switch 4, latch 5, XT30 3, alarm 8, VBAT caps 6.
Target 1.85 kg needs the mass ladder (skeletal U-links ~-60 g, infill/walls); step 4 (68xx) is already in.

P2-0 stub (budget masses at plausible places): trunk 733.0 g, legs 913.0 g, head 201.9 g, **total 1847.9 g
(x 1.05 = 1940 g)**; HOME COM (trunk frame) x -10.9, y 1.2, z -6.0 (146.5 mm above ground); module COMs: trunk
(-26.3, 1.0, 31.6), legs (-2.5, 0, -67.2), head (7.2, 7.5, 133.7). Feet at HOME span x -26.4..44.3 (centre 9.0).

COM budget: the 27 cm model stands with the COM 7 mm behind the foot centre (x = 0.0, feet -20.7..34.7). Target for
TR-35: HOME COM x = foot centre -7 +- 10 mm (i.e. x in [-8, 12]), |y| <= 2 mm. The stub misses it by 3 mm
(19.9 mm behind the foot centre: battery and buck sit behind the hip-yaw stacks). P2b moves trunk mass forward where
K_T allows; whatever remains is closed in P5 by re-deriving HOME (spec G-ISIL allows a HOME change).

## 9. Trunk internal volume budget

| Part | Size (mm) | Qty | Volume cm3 | Mass g | Heat W | Stub placement (bottom centre, orientation) |
|---|---|---|---|---|---|---|
| battery_4s_lipo | 78 x 40 x 30 | 1 | 93.6 | 200 | 0 | (-53, 0, -2.5), flat, back core |
| buck_20a | 60 x 53 x 27 | 1 | 85.9 | 70 | 5 | (-58, 0, 31), flat above battery (PETG + 5 mm air to PLA) |
| pca9685 | 62.5 x 25.4 x 12 | 2 | 38.1 | 18 | 0 | (-26.75, +-31, -2.5), on edge, low sides |
| servo_hub | 46 x 36 x 12 | 1 | 19.9 | 30 | 0 | (-37, 33, 25), on edge, left mid side |
| xl4015 | 54 x 24 x 16 | 1 | 20.7 | 16 | 2 | (-53, 0, -21), belly |
| esp32_s3_devkitc | 63 x 25.4 x 10 | 1 | 16.0 | 10 | 0.5 | (-29, 0, 62), deck |
| lipo_alarm | 40 x 25 x 11 | 1 | 11.0 | 8 | 0 | (-38, -32.5, 26), on edge, right mid side |
| main_switch | 21 x 15 x 22 | 1 | 6.9 | 4 | 0 | (33.5, 29.5, -2.5), front low left |
| bus_caps | 25 x 13 x 20 | 1 | 6.5 | 6 | 0 | (33.5, 10, -3), lying, under the neck servo |
| ads1115 | 28 x 17.5 x 4 | 2 | 3.9 | 4 | 0 | (20.9, +-9.25, 33), upright between ES3054 and neck servo |
| fuse_15a, power_latch | 25 x 14 x 10, 25 x 15 x 8 | 1 + 1 | 6.5 | 10 | 0 | front low right, stacked |
| mp1584, ina226 | 22 x 17 x 4, 23 x 16 x 3 | 1 + 1 | 2.6 | 4 | 0.3 | under the neck servo, right |
| xt30_pair | 16 x 8 x 10 | 1 | 1.3 | 3 | 0 | deck, left |
| imu_mpu6500 | 25 x 15 x 3 | 1 | 1.1 | 2 | 0 | (-27.5, 0, 27.8) on the battery (MJCF site `imu`) |
| **Total** | | | **314.0** | **385** | | all inside K_T, 0 same-link overlaps |

K_T = 625.6 cm3, fixed contents 69.6 cm3 (ES3054 x 2, housings, bearings, neck DS + seat), electronics 314.0 cm3,
242 cm3 for structure, wiring and the buck air gap. The placement is a feasibility proof, not a design: P2b re-plans
(vent path for the buck, switch/alarm/XT30 access, servicing).

## 10. Config sections (each step edits only its own)

* `params.legs` (P2a): `horn_toward`, `hip_plate`, `foot` (yc, plate_t), `mass_g`, `mass_blocks` (stub placeholders,
  delete when real parts carry the mass) + anything new P2a needs. Frozen layout keys stay untouched.
* `params.trunk` (P2b): `hip_yaw_horn_toward`, `neck_horn_toward`, `imu_item`, `electronics` (placement), `mass_blocks`.
* `params.head` (P2c): `horn_toward`, `mass_g`, `neck_hole_r`, `shell_mass_g`, `bumper`, `jaw_block`, `jaw_mass_g`,
  `mass_blocks`.
* Shared top-level keys (`joint_limits_deg`, `targets`, `check`, `bom_buy`): only via the orchestrator.

## 11. Checks for the P2 steps

* Build: `OPENDUCK_CONFIG=tr35_b2 freecadcmd -c "import runpy; runpy.run_path('tools/build_gui.py')"` with
  `OPENDUCK_OUT=<scratch>`, `OPENDUCK_DOCS=0` (docs.py is P3).
* Collision: `cad/check.py` sections 1-6 (single joint over limits, random, same link, pair grid, leg-leg, module
  envelopes). Module filter: `OPENDUCK_CHECK_LINKS=legs|trunk|head|all` (or `link,link`): only pairs with at least
  one part in the selected links, poses of the joints that move them (trunk: all). Stub run times on this Mac (with
  other FreeCAD jobs running): full 177 s (single 35, random 16, pairs 99, leg-leg 12, envelopes 14), `legs` 106 s.
  Stub result: single 0, same link 0, envelopes 0/0/0; pair grid and leg-leg keep the kinematic items of sec. 12.
* Fit study: `OPENDUCK_CONFIG=tr35_b2 OPENDUCK_FIT_OUT=<file> freecadcmd -c "import runpy;
  runpy.run_path('tools/fit_study.py', run_name='__main__')"` (~6-7 min, envelopes only).
* Tests: `pytest tests/cad` (incl. `test_tr35_stub_fc.py`: build, MJCF in MuJoCo, names/order/classes, required within
  limits, head top 350 +- 10, mass budget) and `pytest tests/golden` (xl330_27 byte-identical).

## 12. Open decisions (orchestrator / user)

1. **Pair-grid scope** (sec. 4.4): literal "all links, full required ranges" leaves no trunk volume. Frozen:
   `pair_scope = "module"` + trunk/head keep-ins verified on single-joint + leg-leg sweeps.
2. **Leg-leg, 1 of 32 poses**: both legs pitch -70 / knee -37 (straight forward), toe-in +10, roll-in -20: the two
   soles overlap ~25 mm across the midline (toe-in swings each forward foot ~19 mm inward). Clearing it needs ~+15 mm
   stance per side (roll axes y 26 -> ~41, width 223 -> ~253 mm). Options: accept it as the leg<->leg MJCF
   self-collision pair (spec 3.2.2) or widen. Frozen: y = 26.
3. **(knee +65, ankle -45) pair**: the foot flips over the ankle into the hip front (foot plate vs hip_pitch servo and
   horns, ~300 mm3 in the stub). Kinematic (any foot plate under the ankle does this), not fixable by brackets.
   Options: a combined knee/ankle limit in getup, or a same-leg foot <-> hip/thigh self-collision pair in MJCF.
4. **HOME COM** 19.9 mm behind the foot centre (27 cm: 7 mm): P2b moves mass forward where possible, then P5 re-tunes
   HOME.
5. **Estimates to measure in Step 0**: plastic horn shapes (ES3054 single arm, MG90S/SG90 double arm), MG90S/SG90 tower
   heights, 300 W buck mass, servo hub size; 6800ZZ stock in Turkey (P3 BOM).
