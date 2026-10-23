# CAD CODE MAP: cad/openduck.py, cad/export.py, cad/check.py, cad/docs.py, tools/build_gui.py, build/sim/openduck.xml. Covers how the model is built, every XL330-specific assumption, a refactor for per-joint servo types and a ~35 cm build, and print-bed limits.

## Summary
The 27 cm model is only partly parametric. In cad/openduck.py, PARAMS supplies the joint points (Pollen Microduck positions) and the torso, head and foot boxes. The three geometry builders (_trunk, _leg_items, _neck_head) hold about 550 non-trivial hard-coded mm literals (124 + 175 + 250) and only 43 references to p[...]. As a result, `OpenDuck(params)` cannot resize the robot or change servos.

The servo model is single and global, and it is used in several places:
- `_servo_canon`: XL330 box 20 x 34 x 26 mm with 3.5 mm fillets, shaft 9.5 mm from the near end, horn r6.5 x 3 mm with 4 holes at r4.2 / 45°.
- `_horn_bolt_holes`, which asserts exactly 4 holes.
- `horn_bolts()`, which repeats 4.2 / 45° and the global servo_d/horn_t.
- `MASSES["servo"]`.
- One `ACTUATOR` dict (kp 1.1, forcerange 0.96, damping 0.053, armature 0.0018, frictionloss 0.0048, velocity 10).
- export.py, which writes one MJCF default class "openduck" for all 15 actuators.

Two invariants can be kept through any refactor:
1. **Horn-face datum.** For every servo, the horn outer face (the bracket attach plane) sits at SERVOS origin + 14.5 mm along the horn direction.
2. **Sign convention.** For all 15 joints, a positive joint angle is a positive rotation of the horn relative to the servo body about the horn direction (right-hand rule, counter-clockwise when facing the horn). I checked this by computing axis·horn and flipping it where the body sits on the child link. The PWM direction can therefore be derived automatically.

**The XL330 dual-sided idler is not used anywhere.** No joint has a rear bearing; every joint is cantilevered off a single horn. The only passive supports are printed pins (head_roll front pin, jaw pin).

**Fit study at s = 350/272.7 = 1.2835.** I measured approximate DS3230MG / ES3054 / MG90S / SG90 envelopes on the scaled horn planes, with brackets ignored, over 107 HOME + single-joint sweep poses.
- There are no cross-link clashes.
- DS3230MG case + 6 mm rear idler stub reach y = 102.9 mm, which pushes the thigh wing outward by about 14 mm per side. The DS3230MG case is 40.4 mm along the shaft, against 33.4 mm for the scaled XL330 stack.
- The hip_pitch rear mounting ear hits the knee case (38 mm³) and the knee ear (24 mm³).
- The hip_roll upper ear comes within 1.35 mm of the hip_yaw horn face, so it collides with the yaw-to-roll plate.

**Print bed.** Every current printed part scaled by 1.2835 fits a 220 x 220 x 250 mm bed. The largest is head_hood at 135.5 x 117.8 x 60.8 mm, and any part would only exceed the bed above s = 2.08. docs.py still checks a 180 mm cube.

**RL compatibility.** The RL code uses joint, actuator, site, geom and body names, the actuator order, axes, the "stand" keyframe and the default classes "visual"/"collision". It does not use servo IDs. It does hard-code many 27 cm lengths (H_HOME 0.115, EXTRA_COLLISION boxes, FEET_MIN_DISTANCE, and others) and the 0.96 N.m torque limit.

**Proposed refactor.**
- A servo-type library: cad/servos.py.
- A per-joint servo map.
- SERVOS rewritten as horn-face frames.
- Per-type MJCF default classes.
- Global scale applied to layout, with absolute fastener features.
- Redrawn leg brackets with a 693ZZ rear idler.
- A hardware-map export.

Estimated effort is 12–16.5 person-days. A related design exists in commit e206878 of the sibling repo ../openduck: an earlier 47 cm STS3215 build with a _layout() derived from servo dimensions, a generic dual-sided _u_bracket(L) with an idle-side shaft, and a _micro_servo(). It is a ready template for the 40 mm-class leg brackets.

## Facts
All line numbers refer to repo files at HEAD 29ea39d. "Measured" means I computed it in FreeCAD 1.1.1 headless or in Python from repo files. The scratch outputs are in /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s8/cad-codemap/: model_dump.json, fit_base.json, dump.py, fitstudy.py, literals.py, openduck_sts3215_e206878.py.

## 1. Build pipeline
| Step | Code | Output |
|---|---|---|
| tools/build.sh | Runs freecad GUI (or freecadcmd) on tools/build_gui.py; `--check` also runs cad/check.py (~3 min per the script comment) | build/build.log |
| build_gui.py L38-72 | `model = openduck.OpenDuck()` with no params or env config, then build_document, then export.summary/write_json/write_bom, save_fcstd, export_step, export_print_stls, export_robot, render_views (GUI only), docs.build_all | summary.json, BOM.md, openduck.FCStd/.step, stl/, sim/{openduck.urdf, openduck.xml, scene.xml, meshes/}, img/, print/, openduck_print_parts.zip, docs/, openduck_drawings.pdf |
| Model build time | Measured | 1.2 s headless, 16 links, 15 joints, 676.3 g |

## 2. openduck.py structure
**PARAMS (L31-73), all mm.**
- XL330 block: servo_w 20, servo_l 34, servo_d 26, servo_near 9.5, horn_r 6.5, horn_t 3.
- Print: wall 1.8, plate_t 2.5, bracket_r 9.5.
- Left-side joint points j_hip_yaw..j_mouth: Pollen microduck_rl robot_walk.xml positions.
- sole_below_ankle 25, sole_cant_deg 5.
- Torso box: torso_x (-46.4, 34.5), torso_w 64.4, torso_z (0.3, 42).
- battery (21.7, 38.6, 71.1): NP-F550.
- Head boxes: head_x/w/top/split/bot, face_x, eye_r/z, neck_hole_r.
- Foot: foot_x (-20, 34), foot_w 40.5, sole_t 5, foot_t 7.5.
- Densities: PLA 1.1e-6 kg/mm³, TPU 1.2e-6 kg/mm³.

**Other module constants.**
- MASSES (L75): servo 0.0175, horn 0.0005, battery 0.072, board/hat/camera/speaker/tof/imu/power.
- ACTUATOR (L84): kp 1.1, forcerange 0.96, damping 0.053, frictionloss 0.0048, armature 0.0018, velocity 10.
- JOINT_LIMITS (L90): per joint type, left side, in degrees.
- HOME_POSE (L98): stand pose.
- KINDS/PRINT_KINDS (L106-124): colour and print material per kind. Printed kinds are shell, trim, bracket, sole, pad, face.

**SERVOS (L128-139).** Each entry is key -> (canonical origin, horn dir, far-end dir). In the canonical frame, +Y is the shaft/horn, +Z the far end, and X = Y×Z.
- The origin is the midpoint of the 26+3 mm body+horn stack.
- Measured invariant: horn outer face = origin + 14.5·horn for all 10 keys, and each face lies on a bracket plate. Examples: hip_yaw face z = -3 (hip_bracket top plate z -6..-3); ankle face y = 64.5 (U-bracket horn arm 64.5..67); mouth face y = 42.5 (jaw hub).

**Servo canon and helpers.**
- `_servo_canon` (L354-369): body box from the XL330 params, Y-edge fillets 3.5, a hard-coded cable slot, horn disk r6.5 t3, centre hole r1.2, 4 holes r0.8 at radius 4.2 / 45°. The result is cached once, so there is a single servo type.
- `_servo(key, mirror)` (L371): applies frame() and mirror_y for the right side.
- `_horn_bolt_holes` (L463-487): finds the off-axis cylinder faces of the horn and asserts len == 4.
- `horn_bolts()` (L647-661): repeats the 4.2 mm / 45° / 4-hole pattern and derives y0 from the global servo_d/horn_t.

**Joint registration.** `_joint` (L344) stores name, parent, child, origin, axis, lower, upper, stand, id, actuator = "xl330", effort = 0.96, velocity = 10, and creates the child link at the joint origin.

**_legs (L609-638).**
- Chain: hip_yaw trunk->hip, hip_roll ->hip_cradle, hip_pitch ->thigh, knee ->shin, ankle ->foot.
- Axes: yaw (0,0,-1), roll (1,0,0), pitch (0,1,0), knee (0,-1,0), ankle (0,1,0).
- The left leg is built first (IDs 20-24), then the right (IDs 10-14). On the right, y is mirrored, the Y-axis joints have their axis Y-component negated, and the limits are swapped to (-hi, -lo).
- Item shapes are mirror_y'd. hip_yaw_horn is special-cased; its cbox angle is negated.

**Links and items (measured link masses, g).**
| Link | Mass |
|---|---|
| trunk | 166.95 |
| hip | 22.77 |
| hip_cradle | 3.75 |
| thigh | 47.98 |
| shin | 24.02 |
| foot | 31.96 |
| neck | 36.66 |
| head_tilt | 5.71 |
| head_turn | 26.52 |
| head | 148.33 |
| jaw | 31.15 |

Totals by kind: servo 15 x 17.5 = 262.5 g, brackets 96.0 g, shells 98.9 g, trim 48.5 g, battery 72 g, sole 25.2 g, pad 19.6 g, horns 6.5 g.

Where each servo body lives:
- trunk: hip_yaw x2
- hip: hip_roll
- thigh: hip_pitch and knee
- shin: ankle
- neck: neck_pitch and head_pitch
- head_turn: head_yaw
- head: head_roll and mouth

The servo body sits on the child link for hip_pitch, neck_pitch, head_yaw and head_roll, and on the parent link for the rest.

**Brackets and shells, by function.**
- `_trunk`: chassis (base plate with XL330-footprint pockets -18.7..15.7 x 7.3..27.7, horn clearance holes r7.5, servo cage walls y 5.5-7.5 / 27.5-29.5 z 1.5-20, neck fork r9.5); battery_tray (NP-F550, 3 x M2 + inserts); torso_shell L/R; battery, IMU and power-board solids.
- `_leg_items`: hip_bracket (yaw horn to roll servo), hip_cradle L-bracket (roll horn to pitch horn), thigh_inner_plate + thigh_wing (outline = hull of two hard-coded XL330 rectangles + 3 mm, wing at y 69.5-72), shin_bracket + shin_pad, ankle U-bracket (r_u 11, clearance r13.35 = XL330 corner-sweep radius 12.35 + 1, 4 x M2 countersunk into heat-set inserts), foot_body (trim) + sole (TPU, cbox).
- `_neck_head`: neck_splice, head_tilt_bracket, head_turn_bracket (roll pin), head hood, bottom shell, upper bill + pad, face plate, eye ring, camera/ToF/LED/board/HAT/speaker solids, head_frame (roll servo pocket, front bearing wall, mouth servo shelf), lower_bill (jaw) + pad.

**Mass properties.**
- Item.mass_kg: given mass, or volume × density.
- link_inertia: per solid, mass spread by volume, parallel-axis about the link COM.
- ground_offset: trunk z at which the lowest sole reaches z = 0; measured 118.3 mm at HOME.

**Collision boxes.** Only the sole carries an explicit cbox (center, half-extents, cant). Every other link falls back to its AABB in export.

**build_document (L929-987).** Creates a Pose spreadsheet (columns: Eklem/Aci/Min/Max/HOME/Servo ID/Aciklama, plus trunk_z) and nests App::Parts per link with expression-driven Placement, so the FCStd can be posed from the spreadsheet.

## 3. Joints (from the model dump, mm and degrees)
| Joint | Origin | Axis | Limits | HOME | ID |
|---|---|---|---|---|---|
| left_hip_yaw | (6, 17.5, -5) | 0 0 -1 | -20/30 | 0 | 20 |
| left_hip_roll | (22.5, 17.5, -17.5) | 1 0 0 | ±20 | -5 | 21 |
| left_hip_pitch | (4, 42.5, -17.5) | 0 1 0 | ±90 | -26.237 | 22 |
| left_knee | (-31.8, 38.5, -39.5) | 0 -1 0 | -90/70 | -0.283 | 23 |
| left_ankle | (-31.8, 64.5, -81.5) | 0 1 0 | ±80 | 25.954 | 24 |
| right leg | y mirrored | Y-axes negated, hip_yaw stays 0 0 -1 | (-hi, -lo) | negated | 10-14 |
| neck_pitch | (26, 14.5, 32.4) | 0 -1 0 | -90/40 | 20 | 30 |
| head_pitch | (26, 14.5, 82.4) | 0 1 0 | -30/90 | 20 | 31 |
| head_yaw | (26, 0, 101.1) | 0 0 1 | ±170 | 0 | 32 |
| head_roll | (8.1, 0, 115.6) | -1 0 0 | ±12 | 0 | 33 |
| mouth | (26.1, 28, 118.6) | 0 1 0 | 0/30 | 0 | 34 |

Measured: the servo sign (axis·horn, × -1 when the body is on the child) is **+1 for all 15 joints**.

## 4. export.py
**URDF (L117-155).** Per link: inertial; visual meshes in mm scaled by 0.001; one box collision. Per joint: revolute, with effort and velocity taken from the joint dict (0.96 and 10 for every joint).

**MJCF (L158-243).**
- `compiler angle=radian autolimits inertiafromgeom=false`; `option timestep 0.002 implicitfast`.
- A single `default class="openduck"` holding joint damping/armature/frictionloss and position kp/forcerange (all from ACTUATOR), plus child classes "visual" (mesh, group 2, contype/conaffinity 0) and "collision" (box, contype 0 conaffinity 1, group 3, friction 0.9 0.02 0.01).
- The trunk body has `childclass="openduck"` and `freejoint name="root"`. Joints carry no class attribute (they inherit).
- Collision geoms are written only for meta collision_links (trunk, head) and foot_links, named `<link>_collision`. Feet use the sole cbox; trunk and head use the AABB of all link items (trunk 0.0838 x 0.0660 x 0.0840 m).
- Sites: imu (centre of the imu_chip bbox) and `<foot>_sole` (bottom of the sole box).
- One `<position>` actuator per joint, in joint order, named the same as the joint; ctrlrange = joint range.
- Sensors: gyro, accelerometer, framequat orientation.
- `keyframe "stand"`: qpos = 0 0 base_z 1 0 0 0 + stand angles; ctrl = same angles; base_z = ground_offset + 1 mm = 0.119278 m.

**scene.xml.** Statistic center 0.14 / extent 0.35 are tied to the 27 cm size.

**BOM (L306-349).** Bought parts come from meta["bom_buy"], which is hard-coded with 15 XL330, Radxa/Pi, Robot HAT, IMX219, VL53L5CX, LSM6DSV16X, NP-F550, and M2 screws + heat-set inserts. Printed parts are grouped by Item.part, with material and colour from docs.PRINT_SETTINGS.

## 5. check.py
1. **Design check.** zero + stand + each joint swept over 7 steps (others at HOME), 107 poses. Cross-link pairs are compared with OCC common(); volume > 1 mm³ is a hit; an OCC failure counts as -1 and is reported.
2. **Random combined poses.** 40 poses, informational only.
3. **Same-link overlaps at HOME,** with a SAME_LINK_ALLOW list (camera_lens/holder).

Built with `OpenDuck()` defaults (no config). The current build gives 0 design hits, 14/40 random poses collision-free, and 0 same-link overlaps (build/interference.txt). JOINT_LIMITS were chosen as the collision-free single-joint ranges.

## 6. docs.py
- `print_pieces`: splits multi-solid parts into `_1`/`_2` pieces.
- Orientation: PRINT_DOWN manual orientations by part name; otherwise `best_print_rotation` scores candidates on bed contact, support overhang (>45°, ignoring <3 mm near bed and <10 mm bridges) and height. Mirror pairs are printed mirrored.
- Outputs: oriented STL + PRINT.md + zip, then exploded renders (EXPLODE_EXTRA mm offsets, EXPLODE_JOINT 1.9), a catalog (6x4 tiles per A4) and A4 sheets per piece (ISO E first-angle front/left/top, standard scale 5:1..1:5 at 150 dpi, envelope dimensions only). The PDF also contains hard-coded text: BOM, servo IDs, and assembly step 0 "XL330 / Dynamixel Wizard / set ID".
- Bed check: `bed = 180.0` cube, max(size) > bed.

## 7. XL330-specific assumptions (all must change)
1. PARAMS servo_w/l/d/near/horn_r/horn_t (L32-38); bracket_r 9.5 = horn r6.5 + 3.
2. `_servo_canon`: 3.5 mm fillets, cable slot position, horn with 4 holes at r4.2 / 45° and centre hole r1.2. The XL330 horn is fixed with 4 M2 screws; the PWM servos use a spline plus one centre screw.
3. `_horn_bolt_holes` assert == 4; horn_bolts() hard-codes 4.2 / 45° and the global stack.
4. SERVOS origins embed the 26 + 3 mm stack (origin = face - 14.5 h).
5. Literal XL330 footprints:
   - chassis pockets and cages L390-398 (34 x 20)
   - neck fork r9.5
   - hip_bracket (L497-500) and cradle plates (L509-510)
   - thigh rectangles (-20.5, 13.5, -27.5, -7.5) and (-41.8, -21.8, -49, -15) (L521, L535)
   - wing y 69.5 = servo back face
   - shin sx0/sx1 = knee ± 10 (20 mm width)
   - ankle U r_u 11, sweep r12.35 derived from the 3.5 fillet, yb0..yh1 32..67, foot centre fyc 49.95
   - neck_splice x 14-16 / 36-38
   - tilt arms y ±14.5-17
   - head_turn plates y -24.5..5 (34 mm length)
   - head_frame roll pocket and mouth shelf 16.6-50.6 x 13.5-39.5 (34 x 26)
6. Masses: servo 17.5 g, horn 0.5 g, every servo identical.
7. ACTUATOR: one XL330 BAM fit (0.96 N.m on 2S, above the 6 V rating); `"actuator": "xl330"` and effort/velocity on all joints.
8. JOINT_LIMITS assume ≥340° travel (head_yaw ±170) and do not model PWM travel or neutral position.
9. IDs 10-14 / 20-24 / 30-34 are DYNAMIXEL bus IDs. They appear in the FCStd sheet, bom_buy text, the PDF text and assembly_lines ("fabrikadan hepsi ID 1", Dynamixel Wizard).
10. **Dual-sided idler.** The XL330 supports a passive-side idler (ROBOTIS FPX330-H101 hinge frame + idler set), but the CAD never uses it. Every joint is single-horn cantilevered; L567 explicitly says "rulman yok" (no bearing) at the ankle.
11. Electronics and power are sized for the XL330 bus build: NP-F550 2S battery and tray, Robot HAT (DYNAMIXEL TTL), Radxa + IMX219 + ToF + speaker in the head, LSM6DSV16X board, "servolara dogrudan" (battery straight to servos).
12. Fasteners: M2 screws and heat-set insert holes (r1.1 / r1.6, countersinks).
13. meta "servo" string, used by summary, BOM and the PDF cover.

## 8. RL contract (what the CAD must keep)
**Names and order.**
- Joint names; actuator names equal joint names.
- Actuator order: 10 legs (left first), 4 head, mouth. qpos = 7 + 15 in that order.
- Keyframe "stand"; freejoint "root".
- Sites: imu, left/right_foot_sole. Sensors: gyro, accelerometer, orientation.
- Geoms: trunk_collision, head_collision, left/right_foot_collision. Bodies: trunk, head, and those in EXTRA_COLLISION (left/right_thigh, shin, hip, neck, jaw).
- Default classes "visual" and "collision": robot_model deletes group-2 geoms and uses find_default("collision").

**Already per-actuator.** Domain randomization multiplies nominal per-actuator kp/damping/armature/frictionloss. sim/stand_test reads actuator_forcerange.

**Not used.** RL never references servo IDs.

**Hard-coded 27 cm values in rl/config.py that need scaling or regeneration:**
- BASE_HEIGHT_TARGET / H_HOME 0.115, H_MIN_WALK 0.085, H_MIN_STAND 0.070, H_DEEP 0.065
- Z_SIT 0.044, SEAT_Z 0.042, the CROUCH_FAMILY h column
- SWING_HEIGHT 0.02, TERMINATION_MIN_HEIGHT 0.075, FEET_MIN_DISTANCE 0.06
- GETUP z values (pose_z, arrival_z, success z), standup_z 0.105
- trunk_com_offset, POSE_BANK drop
- EXTRA_COLLISION boxes (m)
- COMMAND_RANGES, GAIT_FREQ 2.5
- GETUP torque_limit 0.96; eval_tasks/play also assume 0.96

## 9. New servo data (sources)
**DS3230MG** ([direnc](https://www.direnc.net/ds3230mg-su-gecirmez-servo-motor)): 40 x 20 x 40.5 mm, 65 g, 27/32 kg·cm at 5/6.8 V (2.65/3.14 N.m), 0.16/0.12 s/60°, 4.8–7.2 V, 50–330 Hz. Spline, ear and hole data are not on that page.

**DS-family drawing** ([DS3235 datasheet](https://hajim.rochester.edu/me/sites/kelley/me240/DS3235-270_datasheet.pdf)): case 40 x 20 x 40.4 mm, ears 54.5 mm overall, holes 49.5 x 10 mm, ear top 27.7 mm above the bottom, double bearing, gear ratio 373, 500–2500 µs, 180° or 270° versions, deadband 2 µs. Shaft offset is about 10 mm from the near end, scaled off the drawing (my estimate). The kit photo shows plastic disc, cross and arm horns plus an aluminium arm.

**XL330-M288** ([ROBOTIS e-manual](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/)): 20 x 34 x 26 mm, 18 g, 0.52 N.m at 5 V and 0.60 N.m at 6 V, 3.7–6.0 V, default ID 1. Passive-side idler kit: [FPX330-H101](https://robotis.us/fpx330-h101-4pcs-set/) and [Generation Robots](https://www.generationrobots.com/en/403860-FPX330-H101-hinge-frame-and-idler-set-dynamixel-xl330.html).

**ES3054** (Robocombo listing per listings.json, https://robocombo.com/emax-es3054-3-5kg-metal-disli-dijital-mini-servo-4300): 28.5 x 13 x 27 mm, 20 g, 0.34 N.m at 6 V, 0.13 s/60°, ~145° travel, 23T. Other listings give 28.45 x 13 x 31.1 mm / 17 g ([bzbuas](https://bzbuas.com/en/shop/drone-parts/servos/emax-es3054/), [elektronicavoorjou](https://elektronicavoorjou.nl/en/product/emax-es3054-servo)).

**MG90S** (https://robocombo.com/mg90-rc-servo-motor): 22.8 x 12.2 x 28.5 mm, 0.22 N.m at 6 V, 0.08 s/60°. Another source gives a 23 x 12 x 26 mm housing and 32 mm including ears ([protosupplies](https://protosupplies.com/product/servo-motor-micro-mg90s/)).

**SG90** (https://robocombo.com/sg90-rc-servo-motor): 23.1 x 12.2 x 29 mm, 9 g, 0.18 N.m at 6 V, 500–2400 µs.

**693ZZ bearing** (https://www.rulmanal.com.tr/urun/693-zz-kwc-3-8-4-rulman): 3 x 8 x 4 mm.

None of the sources give ear-hole spacing, horn bolt circles or shaft offsets for ES3054, MG90S or SG90; they must be measured.

## 10. Fit study (measured, s = 1.2835, approximate envelopes, brackets ignored)
At HOME and over every single-joint sweep there are no cross-link clashes (DS3230MG / ES3054 / MG90S / SG90 against each other and against the scaled shells, trim, sole, pads, battery and PCBs).

Same-link clashes:
- knee case vs thigh_wing: 2567 mm³
- hip_pitch case vs thigh_wing: 2505 mm³
- hip_pitch rear ear vs knee case: 38.3 mm³
- hip_pitch ear vs knee ear: 23.6 mm³

HOME extents (left side, mm):
- hip_pitch y 50.8–102.9 (scaled wing sits at 89.2–92.4)
- knee y 46.8–98.9
- ankle y 23.3–75.4, z down to -139.8
- hip_roll x -24.6..26.3, z -59.7..-5.2

The hip_roll upper ear top is at z -5.2, against the scaled hip_yaw horn face at -3.85, so it collides with the scaled 3.85 mm yaw plate by about 2.5 mm.

## 11. Print bed (PRINT.md oriented envelopes × 1.2835, against 220 x 220 x 250)
| Part | Scaled envelope (mm) |
|---|---|
| head_hood | 135.5 x 117.8 x 60.8 |
| lower_bill | 84.3 x 126.8 x 34.5 |
| head_bottom_shell | 98.1 x 117.8 x 17.1 |
| upper_bill | 51.7 x 117.8 x 5.8 |
| face_plate | 61.3 x 112.4 x 2.6 |
| torso_shell L/R | 103.8 x 53.5 x 41.3 |
| head_frame | 35.3 x 100.8 x 94.3 |

All 27 piece types fit. The first part to exceed the bed would be head_hood, at s = 2.08. Printed mass goes from 299 g to about 632 g (× s³).

For the DS3230MG redesign I estimate thigh_wing at about 81 x 63 x 12 mm (hull of DS rectangles + ears + 3.85 mm) and the foot at about 69 x 52 mm; both fit.

## 12. Hard-coded literals (measured, AST count)
| Function | Numeric literals | Non-trivial | p[...] refs |
|---|---|---|---|
| _trunk | 193 | 124 | 7 |
| _leg_items | 278 | 175 | 17 |
| _neck_head | 433 | 250 | 19 |
| _servo_canon | 27 | 8 | 9 |

## 13. Related design
`git -C ../openduck show e206878:cad/openduck.py` is the earlier 47 cm, 14 x STS3215 (45.2 x 24.7 x 32.5 mm) design.
- PARAMS: servo_len/wid/hgt/shaft_off, femur/tibia_len, hip_y, and others.
- `_layout()` derives z/x stations from the servo dimensions.
- `_servo_canon` includes an idle-side shaft stub.
- `_u_bracket(L)` builds dual-sided outer (horn) + inner (idler) plates.
- `_micro_servo()`: SG90-like model.
- Its literal count is much lower (_leg_items 19 non-trivial).
- A copy is saved at /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s8/cad-codemap/openduck_sts3215_e206878.py

## Design inputs
## A. Keep unchanged (RL and tooling compatibility)
1. **Names, order and axes.**
   - Link, joint and actuator names; joint creation order (left leg, right leg, neck_pitch, head_pitch, head_yaw, head_roll, mouth); the 15-element qpos/ctrl order.
   - Axes and the right-leg mirroring rule in `_legs`; HOME_POSE angles.
   - Keyframe "stand"; freejoint "root"; sites imu, left/right_foot_sole; sensors gyro/accelerometer/orientation.
   - Geoms `<link>_collision` for trunk, head and feet; default classes "visual" and "collision".
   - Add an assertion in `_joint` that the axis equals +horn when the servo body is on the parent and -horn when it is on the child. This holds today for all 15 joints and prevents silent sign flips.
2. **Kinematics.** Scale all sagittal (x, z) coordinates of the j_* points uniformly by s = H_target / 272.7 mm (350 mm gives 1.2835). This keeps the HOME flat-sole condition (pitch + hp - kn + an = 0) and the CROUCH_FAMILY angles valid.
   - A joint origin can slide along its own axis without changing kinematics, so servo stack thickness only moves geometry along the axes.
   - Lateral (y) offsets may change for the servo stacks. Export the resulting HOME foot spacing so FEET_MIN_DISTANCE can follow it.
3. **Unchanged helpers.** Geometry helpers (box, *cyl, fillet*, rbox, shell_rbox, stadium_y, hull2d, prism_xz, bar_y, frame, moved, rotated, mirror_y), Item, Link, link_placements, posed_shapes, ground_offset, link_inertia, build_document, export._link_data, _write_urdf, check.py logic, and the docs.py orientation/drawing machinery.
4. **Logical IDs.** Keep `id` 10-14 / 20-24 / 30-34 as logical joint IDs for labels and the PDF. RL never reads them.

## B. Servo-type library (new cad/servos.py)
`SERVO_TYPES[name]` holds:
- case w (X), l (Z), h (Y along shaft); near (shaft to near end); corner_r
- spline_teeth; horn {kind: 'disc_bolts' | 'pocket', r, t, bolt_pcd, n_bolts, bolt_d, center_screw_d}
- ears {len, from_top, t, hole_pitch_l, hole_pitch_w, hole_d}
- idler {kind: none | 'printed_stub' | 'bearing_693zz', stub_r, stub_len}
- cable {exit_end, x_offset}; must be 0 so that mirror_y stays valid
- mass_g, horn_mass_g; travel_deg; pwm_us (lo, hi); pwm_dir (+1/-1, measured once per type)
- sim {kp, forcerange, damping, armature, frictionloss, velocity}

Functions:
- `canon(t)` returns case, horn, ears and idler shapes. Put the origin at the horn outer face, which is the bracket datum.
- `horn_cutters(t, depth, counterbore)` replaces both `_horn_bolt_holes` and `horn_bolts`.
- `ear_cutters(t)`.

Starting values:
| Type | Body (w x l x h, near) | Ears / holes | Mass | sim values |
|---|---|---|---|---|
| xl330 | today's numbers | — | 17.5 g | today's ACTUATOR |
| DS3230MG @6.5 V | 20 x 40 x 40.4, near ≈10 | ears 54.5, holes 49.5 x 10, ear top 12.7 below case top | 65 g | forcerange 3.06, damping 0.37 (= 3.06/8.3), velocity 8.3 |
| ES3054 @6 V | 13 x 28.5 x 27–31 (measure) | — | 20 g | forcerange 0.34, damping 0.042, velocity 8.1 |
| MG90S | 12.2 x 22.8 x 28.5 | ears ≈32.5 | 13.4 g | forcerange 0.22, damping 0.017, velocity 13.1 |
| SG90 | 11.8–12.2 x 22.2–23.1 x 29 | — | 9 g | forcerange 0.18, damping 0.017, velocity 10.5 |

Placeholder kp, armature and frictionloss until bench sysID:
- kp = forcerange / 0.87 rad. This is the same "100 % at 0.87 rad error" rule as the XL330 comment at L82, giving DS3230MG 3.5, ES3054 0.39, MG90S 0.25.
- armature and frictionloss = XL330 values × forcerange / 0.96 (DS3230MG 0.0057 / 0.015).

Use the DS3230MG travel (180° vs 270°) to validate limits.

## C. Changes in openduck.py
1. **Split PARAMS.**
   - LAYOUT (Pollen j_*, torso/head/foot boxes), multiplied by `scale`.
   - FIT (absolute, never scaled): wall, plate_t, clearances, bearing seat 8.05 x 4, and M3 hardware (clearance 3.4, self-tap 2.5, nut trap 5.7 AF x 2.6). Research found no heat-set inserts in stock, so replace the M2 insert holes.
   - `JOINT_SERVO` map. B2: hip_roll, hip_pitch, knee, ankle -> DS3230MG; hip_yaw, neck_pitch -> ES3054; head_pitch, head_yaw, head_roll -> MG90S; mouth -> SG90.
   - `OpenDuck(config)` takes a JSON or dict (configs/xl330_27.json, configs/tr35_b2.json). build_gui.py and check.py read `OPENDUCK_CONFIG`.
2. **SERVOS becomes SERVO_FRAMES:** key -> (27 cm horn-face point, horn dir, far dir). Face points:

   | Key | Face point (mm) |
   |---|---|
   | hip_yaw | (6, 17.5, -3) |
   | hip_roll | (20.5, 17.5, -17.5) |
   | hip_pitch | (4, 40.5, -17.5) |
   | knee | (-31.8, 40.5, -39.5) |
   | ankle | (-31.8, 64.5, -81.5) |
   | neck_pitch | (26, 14.5, 32.4) |
   | head_pitch | (26, 14.5, 82.4) |
   | head_yaw | (26, 0, 101.1) |
   | head_roll | (8.1, 0, 115.6) |
   | mouth | (26.1, 42.5, 118.6) |

   Placement = frame(face × s, h, f). The body depth follows from the type, so no origin needs editing per servo.
3. **Per-type servo code.** `_servo_canon` becomes a cache keyed by type; `_servo(key)` looks up JOINT_SERVO[key]. `_joint` stores actuator = type, effort/velocity from the type, neutral_deg, and hw. It asserts [lower, upper] ⊂ [neutral ± travel/2 ∓ 5°].
4. **MASSES:** servo and horn masses per type; replace the electronics list with the B2-ESP32 set: ESP32-S3-DevKitC-1, 2 x PCA9685, MPU6500, 6 x XL4015, 4 x 18650 + 2 holders + 2 x 2S BMS.
5. **Scaling strategy (fastest safe path).** Build "style" parts at 27 cm literals and scale them with `shape.transformGeometry(diag(s))`, then re-cut interface features at absolute size. Style parts: torso_shell, head hood/bottom/bills/pads/face/eye_ring, foot_body/sole, shin_pad, chassis outline. Interface features: horn and pin holes, screw holes, bearing seats. Do **not** scale fastener holes (M2 clearance 2.2 mm would become 2.8 mm). Walls grow 1.8 -> 2.3 mm, which matches the research s³ mass assumption (~632 g printed).
6. **Leg redesign around DS3230MG with rear idler** (servo-dependent, written parametrically; adapt `_layout` / `_u_bracket` from e206878):
   - Every DS joint gets a dual-sided bracket: the horn plate on the 25T horn, and the cheek on a 693ZZ (3 x 8 x 4) bearing over an M3 stub coaxial with the shaft on the case bottom. The case-bottom side needs a printed cradle around the case, because the DS case has no rear boss.
   - Move the thigh wing outward about 14 mm, from scaled y 89–92 to about 103–107.
   - Trim the hip_pitch rear ear and the hip_roll upper ear, or don't mount on ears and clamp the case instead.
   - Ankle U-bracket floor = axis - (corner sweep radius + 1). For DS (near 10, half-width 10, ~1 mm corner radius) the sweep radius is ≈13.7 mm.
   - Thigh/shin outlines come from type rectangles, not literals.
   - Foot pocket width follows the U-bracket span: scaled inner idler at y ≈23 to horn arm at y ≈86, against a scaled foot width of 52 mm. Either widen the foot or bias it inboard.
7. **Trunk.**
   - ES3054 hip_yaw cages from the type footprint.
   - 4S1P bay: 4 x 18650 cells (18 x 65 mm format) in a single row ≈74 x 18.5 x 65 mm plus holders. The scaled NP-F550 slot is only 27.9 x 49.5 x 91.3 mm, so widen it in y to about 78 mm.
   - Electronics bay for ESP32-S3, 2 x PCA9685, MPU6500 (SPI) and 6 bucks.
   - Wiper-wire channel per leg.
8. **Neck and head.**
   - ES3054 neck fork from its horn.
   - MG90S/SG90 horn pockets: capture the supplied plastic horn plus its self-tapping screws. A printed spline is not reliable.
   - head_yaw limit ±80° (MG90S 180°).
   - Neutrals at range midpoints: neck_pitch -25°, head_pitch +30°, knee -10°.
   - Drop the camera, ToF, speaker, board and HAT solids, which are not in B2-ESP32. Keep face_plate and eye_ring as cosmetic parts.
9. **Joint-limit flag.** hip_pitch ±90° uses the full 180° DS travel with zero margin, but SEAT_DEG needs 87.5°. Either buy the 270° variant or set neutral +5° and limits -80/+90 after the re-sweep.

## D. export.py
1. **Per-type MJCF classes.** Nest them under "openduck" and keep "visual" and "collision":
   `<default class="ds3230mg"><joint damping="0.37" armature="…" frictionloss="…"/><position kp="3.5" forcerange="-3.06 3.06"/></default>`, and the same for es3054, mg90s and sg90. Write `class="<type>"` on every `<joint>` and `<position>`. URDF effort/velocity come per joint, as today.
2. **New files beside openduck.xml.**
   - `collision_boxes.json`: AABB per link in m, for EXTRA_COLLISION.
   - `scale.json`: s, HOME trunk z (0.1183·s ≈ 0.152 m), HOME foot lateral spacing.
   - `hw_map.json`: joint, logical id, type, PCA addr/channel, ADC1 GPIO for the 10 leg wipers, neutral_deg, pwm_us, pwm_dir, servo_sign (+1 for all 15), lower/upper, home. Suggested layout: PCA 0x40 for the legs (two rails per leg); PCA 0x41 for neck, head and mouth; ADC1 GPIO1–10 for the leg wipers. The research doc lists GPIO1–10 = 10 ADC1 channels.
3. **Scene and BOM.** Scene statistic center/extent × s. BOM bom_buy generated from the JOINT_SERVO counts plus the config's electronics list.

## E. check.py and docs.py
- **check.py:** accept the config. Re-derive JOINT_LIMITS as the intersection of the collision-free sweep ranges and the servo travel. Clear SAME_LINK_ALLOW of the camera.
- **docs.py bed check:** BED = (220, 220, 250), checked per axis on the oriented envelope (XY may rotate).
- **docs.py exploded views:** EXPLODE_* offsets × s.
- **docs.py text:** assembly_lines and the build_pdf servo text come from config. Step 0 becomes: centre each PWM servo at its neutral_deg pulse, solder the wiper wire, calibrate the ADC, then fit the horn at the nearest spline tooth (25T 14.4°, 23T 15.7°, 20T 18°) and store the trim in hw_map.

## F. RL-side handoff (rl/config.py is lead-owned; recommend reading scale.json and collision_boxes.json)
- Multiply by s: H_HOME / BASE_HEIGHT_TARGET, H_MIN_*, H_DEEP, Z_SIT, SEAT_Z, the CROUCH_FAMILY h column, SWING_HEIGHT, TERMINATION_MIN_HEIGHT, standup/GETUP z values, POSE_BANK drop, trunk_com_offset.
- Set FEET_MIN_DISTANCE from scale.json.
- Regenerate EXTRA_COLLISION.
- Replace the 0.96 constant in GETUP, eval_tasks and play with per-actuator forcerange.
- Gait and command ranges: per the torque research, retrain at 1.6–1.8 Hz.

## G. Verification and effort (person-days)
| Phase | Work | Days |
|---|---|---|
| P0 | Golden snapshot of build/sim/openduck.xml, summary.json and interference.txt | 0.5 |
| P1 | Servo library + JOINT_SERVO + horn-face frames, with the xl330 config reproducing the snapshot byte for byte | 1.5–2 |
| P2 | Export classes and JSON files, docs bed/text, config plumbing | 1–1.5 |
| P3 | Scale path + absolute fasteners + M3 / no-insert conversion | 1.5–2 |
| P4 | DS3230MG leg brackets with 693ZZ idlers + ES3054 hip_yaw cages | 3–4 |
| P5 | Trunk battery/electronics bays + wire routing | 1.5–2 |
| P6 | Neck/head (ES3054, MG90S/SG90 pockets) | 2–3 |
| P7 | check.py limits, sim/collision_check, stand_test, RL constant handoff | 1–1.5 |
| **Total** | | **12–16.5** |

Check runtime: my 107-pose envelope sweep took 63 s; the full check.py takes about 3 min.

## Open questions
- Is the 35 cm target the HOME/stand height (s = 350/272.7 = 1.2835) or the zero-pose height (265.6 mm, s = 1.318)? Research and this report assume HOME.
- Is a DS3230MG 270° variant in stock in Turkey? With the 180° version, hip_pitch ±90° uses the whole travel, but SEAT_DEG needs +87.5° and CROUCH +81°.
- What are the measured DS3230MG details: shaft-to-near-end offset (about 10 mm estimated from the DS3235 drawing), ear thickness and position, cable exit position (centred, so mirror_y is valid?), and the horn bolt circle of the horn that will actually be used? Are 25T aluminium disc horns stocked in Turkey?
- Which ES3054 height is right: 27 mm case (Robocombo/Robotistan) or 31.1 mm (other listings)? Also needed: its ear-hole pitch, 23T horn set, and shaft offset. MG90S/SG90 ear pitch and shaft offset also differ between sources and need measuring.
- Stance width: accept the DS3230MG stack pushing the thighs about 14 mm outward per side (overall width about 187 -> about 215 mm), or move the hip joint y-offsets and redesign the trunk?
- Electronics placement for B2-ESP32 (no camera/ToF/speaker): put ESP32-S3, 2 x PCA9685, 6 x XL4015 and BMS in the trunk, and keep the head as shells plus MG90S/SG90 only?
- Fasteners: M3 self-tapping vs M3 nut traps throughout (heat-set inserts not found in stock)? Is the M3 shoulder-bolt + 693ZZ idler acceptable for all 8 DS3230MG joints?
- Should JOINT_LIMITS be re-derived from the new geometry plus servo travel (which forces a policy retrain anyway), or kept as close as possible to today's values (head_yaw must shrink from ±170 to about ±80 regardless)?
- Will the lead let rl/config.py read the CAD-exported scale.json and collision_boxes.json instead of hard-coded 27 cm constants?

## Risks
- **Shaft-direction mismatch.** The DS3230MG case is 40.4 mm along the shaft, against 33.4 mm for the scaled XL330 stack (26 + 3 mm × 1.2835). The pitch-servo stacks grow outward: hip_pitch and knee envelopes reach y ≈103 mm against the scaled thigh wing at 89–92 mm. That gives wider legs, more roll inertia and longer brackets.
- **Mounting-ear clashes (measured).** The hip_pitch rear ear hits the knee case (38 mm³) and the knee ear (24 mm³). The hip_roll upper ear sits 1.35 mm below the hip_yaw horn face, overlapping the yaw-to-roll plate by about 2.5 mm. Ears must be trimmed, or the case clamped instead.
- **No idler today.** Today's CAD hangs every joint off one horn and never uses the XL330's passive-side idler. Reusing that pattern with 3 N.m DS3230MG on a robot of about 1.65 kg would overload the horn screws, the PETG plates and the servo output bearing. Rear idlers (693ZZ) are needed, and the DS case has no rear boss to hang them on.
- **Servo travel.** DS3230MG 180° gives hip_pitch ±90° zero margin. MG90S 180° cannot do head_yaw ±170°. ES3054 at ~145° fits neck_pitch -90..40 only if the horn is indexed at -25°. Coarse spline indexing (14.4–18°) needs software trims.
- **Uniform scaling enlarges fasteners.** Scaling the 27 cm geometry with transformGeometry also scales screw holes, insert holes, pin and bearing clearances. Fastener and fit features must be kept as absolute parameters and re-cut after scaling.
- **Mirror validity.** The right leg reuses mirror_y'd servo shapes. This is valid only while the canonical servo model is symmetric about its X = 0 plane (cable exit and horn holes centred). An asymmetric real servo would need a rotated, not mirrored, placement.
- **Unverified dimensions.** Datasheet and listing sizes for ES3054, MG90S and SG90 conflict, and ear pitch, shaft offset and horn patterns are unpublished. Brackets designed before measuring the bought units are likely to misfit.
- **RL constants out of step.** RL constants (heights, EXTRA_COLLISION boxes, FEET_MIN_DISTANCE, 0.96 N.m torque limits in getup/eval/play) are hard-coded for 27 cm. Exporting a 35 cm MJCF without updating them gives wrong termination, reward targets and get-up success criteria, without raising any error.
- **Battery and electronics volume.** The B2 4S1P pack plus 6 XL4015 bucks, 2 PCA9685 boards and the ESP32 do not fit the scaled NP-F550 slot (27.9 x 49.5 x 91.3 mm) or the 42 x 36 mm scaled power-board area. The trunk needs real redesign, not just scaling, and its mass and inertia will move.
- **Snapshot regression is essential.** Without the P0 byte-for-byte snapshot, the per-type refactor can silently change today's XL330 model (MJCF inertias, keyframe base_z), which would invalidate the existing RL baselines used for comparison.
