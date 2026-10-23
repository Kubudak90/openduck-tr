# Can OpenDuck crouch, sit on the floor and stand back up within its servo limits?

Yes. Torque is not what limits these motions; balance and the robot's geometry are. Every path tested here, scripted with position control on the training model, stayed at or below 48% of the 0.96 N·m limit when stand-up took 2 s or more. Two results matter most for the spec:

1. **Getting up from a flat sit needs the head moved back.** If the robot sits with its trunk flat on the floor and the head stays at HOME, it cannot stand up. It always tips forward onto its head. Moving the head back fixes this.
2. **The current setup can't express these poses yet.** The policy's action range, the height termination and the height/orientation rewards all block them (§4).

Everything below comes from scripted position-control runs, not a trained policy. Low-height walking was checked kinematically only.

## 0. The model as measured (`rl.robot_model.build_model()`)

- **Timing and mass:** physics step 5 ms. Total mass 0.6746 kg; the neck subtree is 0.2454 kg (36.4%).
- **Actuators (all 15):** `openduck.xml:7-8` gives kp 1.100, no velocity gain, torque limit ±0.96 N·m. Joint damping is 0.053, armature 0.0018, friction 0.0048. There is no speed-dependent torque limit; damping is the only back-EMF model, so 4 rad/s costs 0.21 N·m. The torque limit is hit at 0.873 rad tracking error; steady sag is 5.2° per 0.1 N·m.
- **HOME:**
  - Geometry: the trunk origin is 0.1183 m above the sole and the trunk box clears the floor by 76.3 mm. The centre of mass (COM) sits 20 mm inside the foot support area (soles span x −20…+34 mm; COM x +0.2 mm).
  - Static torques: hip_roll 0.101 N·m (the largest static load in every standing pose) and knee 0.050.
  - In simulation, raw HOME targets settle at z 0.116, pitched 6–7° nose-down (knee sags about 4°).
- **Sign conventions (left leg):** positive hip_pitch moves the sole backward (−17 mm per +10°). Positive knee moves it forward and up (+10.5 / +5.3 mm). Feet stay flat when trunk pitch + hip_pitch − knee + ankle = −0.01°; positive trunk pitch is nose-down.
- **Servo naming mismatch:** `rl/TASK.md:85` says "STS3215", but `README.md:19` and `cad/openduck.py:84` say XL330-M288-T. From memory (not checked here), the XL330 datasheet stall torque is about 0.52 N·m at 5 V, much less than the simulated 0.96.

## 1. Crouching

### Height family
Built in `family3.py`. Constraints: knee ≤ 60°, hip_pitch ≤ 85°, |ankle| ≤ 70°, flat feet, trunk clearance ≥ 5 mm. For each height I took the pose with the largest COM margin.

| trunk height (m) | trunk pitch (°) | hip_pitch | knee | ankle | COM margin (mm) |
|---|---|---|---|---|---|
| 0.120 | 0 | −25.5 | −10.5 | 15.0 | 27.0 |
| 0.110 | 0 | −10.5 | 16.5 | 27.0 | 26.4 |
| 0.100 | 0 | 1.5 | 36.0 | 34.5 | 26.4 |
| 0.090 | 0 | 13.5 | 52.5 | 39.0 | 27.0 |
| 0.085 | 0 | 19.5 | 60.0 | 40.5 | 26.8 |
| 0.080 | −7.5 | 40.5 | 60.0 | 27.0 | 26.0 |
| 0.075 | −15 | 57.0 | 60.0 | 18.0 | 26.7 |
| 0.070 | −17.5 | 69.0 | 60.0 | 8.5 | 26.0 |
| 0.065 | −22.5 | 81.0 | 58.5 | 0.0 | 26.6 (8 mm trunk clearance) |
| 0.060 | −17.5 | 84.0 | 57.0 | −9.5 | 18.4 (6 mm trunk clearance) |

- **Static torques:** ≤ 0.116 N·m on every leg joint (hip_roll dominates) and ≤ 0.092 on the neck.
- **Below 0.085 m the trunk must pitch nose-up,** because knee travel runs out.
- **Trunk kept level** (`family2.py`, knee ≤ 60): margin is 17.8 mm at 0.080, 12.7 mm at 0.075 and 8.6 mm at 0.070.
- **Lowest balanced heights:** 0.0551 m with margin ≥ 8 mm, and 0.0591 m with margin ≥ 15 mm.
- **Maximum useful height is about 0.120 m.** Going to 0.125 needs the knee at its −90° limit.

### Checking the deep crouch you suggested
Your candidate (85, 70, −15) gives height 0.0520 m with 10 mm clearance but only 5.7 mm COM margin. It also has to be excluded on collision grounds: at knee 70° the thigh touches the shin, and the hip-roll servo touches the foot (6 mm³).

### Dynamic runs
**Straight joint-space ramps (`run_crouch.py`, 2 s):**
- Fine down to 0.080 m.
- At 0.075 m it survived the way down but fell on the way back.
- At 0.070 m and below it fell forward during the ramp (pitch 72°, head on the floor).
- Peak torque was only 12–25%. The cause is balance: a straight joint-space interpolation leaves the COM-centred family.

**Following the family path** (`run_sit.py crouch`), HOME → 0.060 m in 3 s, hold 2 s, back in 3 s:

| control | peak torque | worst COM margin | notes |
|---|---|---|---|
| raw targets | 0.25 N·m (26%), knee, on the way up | 6–7 mm | sag pitches the trunk +14–15° nose-down at 0.085–0.100 m |
| gravity feed-forward + iterative correction | 0.21 N·m (22%) | ≥ 15 mm | measured heights 0.110 / 0.101 / 0.091 / 0.080 / 0.071 / 0.065 / 0.060 m |

## 2. Sitting and standing up

Two candidate seated poses came out of the simulation, both reached with the head at HOME.

| seated pose | how it rests | load on bottom / feet | holding torque | joint state |
|---|---|---|---|---|
| **Flat sit** | trunk box flat on the floor, pitch 0.0°, z 0.042 m | 5.5 N (83%) / 1.1 N | ≤ 0.04 N·m (4%) | legs settle at (79.1, 14.7, −46.8) |
| **Lean-back tripod** | rear edge of the chassis plus both feet, pitch −48° to −50°, z 0.064 m | 2.9 N / 3.7 N | 0.17 N·m on the neck | hip_pitch ends pinned on its 90° limit even when commanded 70–85°; the joint limit carries the load |

- **Every seat target with pitch −15° to −30° rolled forward into the flat sit; targets at −35° rolled back into the tripod** (`seat_grid.py`). Under plain position control nothing settles in between. A policy could hold an intermediate seat, for example pitch −35° with 13–19 mm margin kinematically.
- **A seat with the bottom just touching floats in practice.** Kinematically, (85, 35, −25) at −25° touches with 14 mm margin, but in simulation the trunk hovered 1–2 mm up with no trunk load. To make the bottom actually carry weight, the target has to put the trunk 3–6 mm "below" the floor.
- **In the flat sit the COM is in front of the feet.** It is 4.5–9 mm ahead of the front edge of the soles; even the best tucked pose, (90, 52.5, −37.5), is still at −4.5 mm.
- **Your level-trunk seated candidates are not reachable.** At (80–90, 0–10, −75…−80) the chassis would be 14–32 mm below the floor.

### Standing up

| start | method | 3.0 s | 2.0 s | 1.5 s | 1.2 s |
|---|---|---|---|---|---|
| flat sit, head at HOME, feet tucked (87.5, 47.5, −40) | stand along the family path | fall | fall | fall | – |
| flat sit, head moved back + feet tucked | stand along the family path | 0.26–0.27 (27–28%) | 0.33–0.35 (35–36%) | 0.43–0.45 (44–47%) | – |
| tripod, head at HOME | stand along the family path | 0.30–0.34 (32–36%) | 0.41–0.46 (43–48%) | – | 0.63–0.96 (66–100%, saturates) |

Values are peak torque in N·m (% of 0.96).

- **Why the head-at-HOME flat sit fails:** the robot pivots on the front edge of the trunk box and pitches to +75° (head on the floor). Tucking the feet alone leaves the COM 6.7 mm ahead of the feet.
- **What "head back" means:** neck_pitch 20° → 40° and head_pitch 20° → 0°. This shifts the COM 13.0 mm backward (`head_com.py`). With the feet tucked, the COM ends 6.2 mm inside the feet.
- **The two successful methods hold up under variation.** Both passed with kp ×0.85 and ×1.15 and with +50 g on the trunk (`robust.py`).
- **Peak joints:** the ankle for the flat-sit method, the knee for the tripod method.
- **Moving the head forward makes it worse.** I tried neck −20° and −50°; the robot falls forward because the COM is already in front.
- **Most of the stand-up torque comes from speed, not gravity.** The static torque to stand from a seat with the COM over the feet is ≤ 0.12 N·m. The rest is damping (0.053 × joint speed) and acceleration.

### Getting down
HOME → deep crouch along the family path in 2 s, then → seat in 1 s: peak 0.17 N·m (18%).

## 3. Self-collision (visual meshes)

**Method:** I filled every mesh as a 0.8 mm voxel grid and also sampled surface points. I measured interpenetration volume and minimum surface distance for each pair of parts on different bodies, after removing the 24 joint-interface contacts that already exist at HOME (horns on servos). Touching surfaces show up as ≤ 1 mm or a few mm³.

**Single knee sweep** (thigh against shin):

| knee | 65° | 67.5° | 70° |
|---|---|---|---|
| clearance | 1.0 mm (the design gap, same as at HOME) | 0.47 mm | 0.06 mm and 9 mm³ (thigh_inner_plate on shin_pad) |

**Deep hip/knee grid** (`limit_scan.py`, level trunk, flat feet):
- Contact at (85, 65, −20): hip_roll_servo on foot_body, 3 mm³.
- Contact at (90, 50–65, −25…−40): thigh_wing on foot_body, 2–20 mm³.
- At (85, 60, −25) clearance is 1.2 mm.
- With the trunk pitched −20° the same grid is ≥ 6.4 mm everywhere.
- **The danger is the toe pitching up into the thigh,** i.e. a very negative ankle while hip_pitch ≥ 85° and knee ≥ 50°. Separately, (85, 5, −80) puts shin_pad into foot_body (53 mm³).

**Along the three simulated trajectories** (`traj_collide.py`; T1 crouch to 0.060 m, T2 flat sit + tuck + head back + stand, T3 tripod), every 0.1 s, minimum clearance:

| part pair | T1 | T2 | T3 | where |
|---|---|---|---|---|
| trunk–shin | 4.3 mm | 3.6 mm | 3.6 mm | torso_shell against shin bracket / ankle servo |
| trunk–foot | 4.3 mm | 4.0 mm | 4.0 mm | chassis against foot_body |
| thigh–foot | 9.2 mm | 2.5 mm | 9.2 mm | T2 minimum at the tuck pose (87.9, 46.9, −40.9) |
| hip–foot/shin | 7.7 mm | 4.7 mm | 7.6 mm | |
| thigh–shin | 1.0 mm | 1.0 mm | 1.0 mm | design gap; one touch of 1 mm³ at (84, 57, −8.9) |
| leg–leg | > 6 mm | > 6 mm | > 6 mm | |
| head–trunk | – | ≥ 14.3 mm | ≥ 23.7 mm | T2 with neck at 40° |

No interpenetration was found on any of the three trajectories.

**Trunk collision box against the real chassis** (`box_vs_mesh.py`):
- **Nose-up / sitting side (pitch 0° to −50°):** the box's lowest point matches the chassis exactly (0.0 mm), so the simulated sit is faithful.
- **Nose-down / falling forward (+10° and +20°):** the box sticks out 10.1 and 19.8 mm below the real shape.
- **Hip-roll servos in the flat sit:** their bottoms are coplanar with the chassis (within 0.2 mm), but they have no collision geometry in the model.

## 4. Numbers for the spec

**Height command**
- Standing still: 0.065–0.120 m.
- Walking: 0.080–0.120 m. Kinematically, lifting a foot 2 cm stops working below about 0.072 m even with the joints at their margins (`walk_reach.py`).
- Below 0.085 m the trunk needs a nose-up pitch reference: −7.5° at 0.080, −15° at 0.075, −17.5° at 0.070, −22.5° at 0.065. Without it, the level-trunk floor is 0.075 m.
- Slew no faster than about 0.018 m/s (HOME → 0.065 m in ≥ 3 s, which measured ≤ 26% torque).

**Seated target**
- Tucked flat sit: trunk z 0.042 ± 0.003 m, pitch 0 ± 5°, left leg (87.5, 47.5, −40), right mirrored.
- Holding torque ≤ 0.04 N·m.
- Head: neck_pitch 40°, head_pitch 0°. This can be scripted by the environment when sit is commanded; the policy doesn't have to control the head.
- If the head must stay at HOME, the only fallback is the tripod seat. It sits at pitch −48° with hip_pitch on its limit, and the termination tilt threshold would need relaxing to −0.5.

**Durations** (flat-sit method)

| phase | duration | peak torque |
|---|---|---|
| sit down: HOME → crouch along the family path → seat | ≥ 2 s + 1 s | ≤ 18% |
| head back + feet tuck | 1.5 s | – |
| stand up | ≥ 2.0 s (≥ 3 s if the real servo is ~0.52 N·m) | ≤ 36% |

Everything stays under 80% for stand-up ≥ 1.5 s (flat-sit method) or ≥ 2 s (tripod).

**Action space**
`ACTION_SCALE` × `ACTION_CLIP` (`config.py:41-42`) allows ±43.0° around HOME. That only covers heights ≥ ~0.095 m. Required changes from HOME:
- 0.090 m: knee +52.8°
- 0.080 m: hip_pitch +66.7°
- 0.065 m: hip_pitch +107°, knee +59°
- tucked seat: hip_pitch +114°, ankle −66°
- tripod: hip_pitch +111°, ankle −67°

Suggested fix: target = q_ref(height, sit) + scale × action, with q_ref taken from the table in §1 and the seat pose.

**Termination and rewards**
- `TERMINATION_MIN_HEIGHT` = 0.075 (`config.py:88`) ends episodes at crouches ≤ 0.075 m and at both seats. Replace it with "head box touches the floor" or a tilt check, and allow trunk–floor contact while sit is commanded.
- `TERMINATION_MAX_GRAVITY_Z` = −0.6 (`config.py:89`) leaves only 3–4° of margin for the tripod seat.
- Make `BASE_HEIGHT_TARGET` (`config.py:66`, used at `task_math.py:139`) follow the commanded height, and make the orientation term (`task_math.py:138`) follow the pitch reference.

**Joint limits and collision**
- Knee upper limit from 70° to 65° (left: 1.22173 → 1.13446 rad; right mirrored to −65°).
- Hip_pitch upper limit from 90° to 85°.
- Add a pose constraint: keep ankle ≥ −35° whenever hip_pitch ≥ 80° and knee ≥ 50°.
- Optionally give each thigh a box on a new contact bit (e.g. contype 4) and add that bit to the feet's conaffinity, so exploration can't put a foot into the thigh.
- The trunk box is fine for sitting and needs no change.

## Files

All in `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/sitstand/phys-sit/`:

- **Shared helpers:** `common.py`, `sim_traj.py`, `sim_path.py` (50 Hz control, gravity feed-forward, iterative correction)
- **Model inspection:** `inspect_model.py`
- **Kinematic scans:** `scan_kin.py`, `family2.py`, `family3.py`, `seat_search.py`, `seat_deep.py`, `head_com.py`, `walk_reach.py`
- **Dynamic runs:** `run_crouch.py`, `run_sit.py`, `run_sit2.py`, `seat_grid.py`, `flat_sit.py`, `flat_sit2.py`, `robust.py`
- **Collision checks:** `collide.py`, `collide_poses.py`, `test_collide.py`, `limit_scan.py`, `traj_collide.py`, `box_vs_mesh.py`
- **Outputs:** `sit_out.txt`, `sit2_out.txt`, `seat_grid_out.txt`, `family3.npy`, `scan_kin.npy`, `collide_poses.pkl`, `traj_collide.pkl`