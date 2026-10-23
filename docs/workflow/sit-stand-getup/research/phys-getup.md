# OpenDuck get-up feasibility (physics, real actuator model)

**Verdict: getting up is feasible within the 0.96 N·m limit for all three modes (front, back, side), but only on a model with extra collision geoms.** The front and side get-ups need the head/neck joints (0% success without them), so the get-up policy needs a 14-dim action. One blocker: the current action mapping can't reach the poses the get-up uses. All results below are open-loop keyframe sequences on flat floor, so the success rates are a lower bound for a feedback policy.

Scratch dir (all scripts and outputs): `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/sitstand/phys-getup/`. No repo edits.

## 0. Model facts (`rl.robot_model.build_model()`, mujoco 3.16)

**Actuators** (`build/sim/openduck.xml:7-8`, all 15):
- `position kp=1.1`, `forcerange ±0.96`, `forcelimited=1`; joint damping 0.053, armature 0.0018, frictionloss 0.0048.
- ctrlrange equals the joint range (`:255-269`).
- Hitting the torque limit takes a position error of at least 0.87 rad.
- Timestep: XML `:4` says 0.002, but `robot_model.py` sets 0.005 (`SIM_DT`).

**Mass:**
- Total 0.6746 kg. The neck subtree (head + neck + jaw) is 0.2454 kg (36.4%); the head body alone is 0.1456 kg and the jaw 0.0309 kg.
- Weight is 6.62 N.
- The largest static gravity torque on neck_pitch is 0.189 N·m (neck fully forward, trunk upright), about 20% of the limit.
- Half body weight times thigh length (0.042 m) is about 0.14 N·m per leg. Statically the robot is strong for its weight.

**Collision geoms: only 4 boxes, plus the floor:**
- `trunk_collision` (`:120`), `head_collision` (`:240`, half-sizes 0.062×0.047×0.029), and two foot boxes (`:155`, `:195`).
- Class `collision` has `contype=0, conaffinity=1` (`:10`), so only floor contact; `robot_model.py:28-30` gives the feet `2/3`.
- These bodies have **no** collision geom: hips, thighs, shins, neck, head_tilt, head_turn, **jaw/beak**.

**Visual meshes vs boxes** (`inspect_model.py`, HOME pose):

| Part | Visual mesh extent | Nearest collision box | Gap |
|---|---|---|---|
| Thigh wing | \|y\| up to 0.073 m | trunk 0.033, head 0.047, foot 0.062 | thigh sticks out sideways |
| Jaw | z_min 0.2108 | head box bottom 0.2150 | 4 mm below the box |
| Neck | z 0.140–0.210 | trunk top 0.161, head bottom 0.215 | the neck is not covered |

## 1. Fallen states (`falls.py` → `falls.json`, `penetration.py`)

Setup: 400 drops from uniform random orientations at 2–10 cm, plus 100 tips from HOME (15–35° tilt, 0–4 rad/s), each simulated until still or 4 s.

**Random drops by holding ctrl:**

| ctrl | back | front | left | right | inverted | upright |
|---|---|---|---|---|---|---|
| HOME | 113 (28%) | 102 (26%) | 80 (20%) | 72 (18%) | 21 (5%) | 12 (3%) |
| zero targets | 132 | 136 | 73 | 59 | 0 | 0 |
| limp | 124 | 108 | 84 | 73 | 0 | 11 |

- With limp servos, 378 of 500 runs were still sliding at 4 s.
- Tipping from HOME gives the expected mode: forward 25/25 front, back 25/25 back. Tipping left gives 19 left / 5 upright / 1 front; right gives 21 right / 2 upright / 2 front.

**Resting contacts with HOME ctrl:**

| Mode | Contacts | Trunk z (m) | Notes |
|---|---|---|---|
| back | head, trunk | 0.048 | |
| front | head, both feet | 0.047 | gravity in trunk frame (0.98, 0, −0.19); the trunk does not touch |
| left / right | head + lower foot only | 0.045 | |
| inverted | head only | 0.143 | |
| upright | feet | 0.116 | |

**Two of these poses are not realistic:**
- **Side poses:** the thigh mesh sinks **29.5 mm** into the floor (shin 4.5 mm, foot 5.0 mm, jaw 2.2 mm). The real robot would rest on its thigh wing with the body higher. Back, front and inverted have no penetration over 2 mm.
- **"Inverted":** the robot balances on the flat top of the head box. The real head hood is rounded, so this is an artifact of the box shape.

## 2. Scripted get-ups

**Search method** (`getup_lib.py`, `opt_getup.py`):
- PD-target keyframes for all 14 non-beak actuators (beak at HOME), smoothstep interpolation between them, then 0.6 s back to HOME and 1.5 s hold.
- Optimised with CMA-ES (population 24, 50–90 generations), with randomisation in the loop: kp ±15%, friction 0.6–1.3, mass ±5%, damping ±20%, frictionloss ±50%, joints ±0.05 rad.
- "Success" means gravity z below −0.9 (upright) and trunk height above 0.10 m for the whole last 0.5 s of the hold.
- Evaluation (`eval_getup.py`) uses 40–60 resting states of each mode.

**Proposed collision model ("extended"):** 8 boxes fitted to the visual mesh bounding boxes (`make_extra_boxes.py` → `extra_boxes.json`): thighs, shins, hips, neck, jaw, all floor-only.

**The current model can't be used for get-up:**
- Unconstrained CMA on the current model (`best_front_sym_K3.json`) found a 1.22 s flip with every target at its limit.
- It succeeded 45% nominally and 20% with randomisation, hit the head with 68 N, and **pushed shins and feet 15 mm through the floor**.
- So the optimiser exploits the missing leg collision.

**Results on the extended model:**

| Mode | Strategy (file) | Motion time, s (time to upright) | Nominal | Randomised | Peak torque, N·m | Without head joints |
|---|---|---|---|---|---|---|
| Front | slow, segments ≥0.6 s (`best_front_x_slow.json`) | 3.19 (2.99) | 100% | 85% | 0.71 (no saturation) | 0% |
| Front | fast (`best_front_x.json`) | 2.09 (1.93) | 100% | 55% | 0.96 (saturated 1–3% of time) | 0% |
| Back | legs only, head held at HOME: the slow front leg sequence rocks it straight up | ≈3.2 | 100% | 68% | 0.56 | (legs only) |
| Back | roll to front, then front (`combo_back.json`) | 4.94 (4.74) | 100% | 87.5% | 0.96 (head_yaw, neck, ankle) | 0%: ends on its side |
| Back | symmetric pitch-over to front, then front (`combo_back_sym.json`) | 6.02 | 100% | 82.5% | 0.96 | — |
| Left / right | roll to front, then front (`combo_left.json` / `combo_right.json`, mirrored) | 4.69 (4.49) | 100% | 82.5% | 0.96 (knee, head_yaw) | 0%: stays on its side |

**How each motion works:**
- **Front, slow sequence** (radians, left leg; the right is mirrored):
  1. 0.64 s: fold the legs fully (hip_pitch −1.57, knee +1.22, ankle +1.4, hip_roll −0.35), neck_pitch −1.57 and head_pitch +1.57, so the head is tucked with the beak planted on the floor.
  2. 0.71 s: swing hip_pitch to +1.57, knee to −1.57, neck to −0.08. The head acts as a pushing arm.
  3. 0.63 s: knee −0.13, ankle −1.4, neck +0.70, shifting the centre of mass over the feet.
  4. 0.60 s: settle, then 0.6 s to HOME.
- **Roll stages** reach the front pose 40/40, both nominal and randomised. They work by a fast swing of head_yaw (back: −1.07 → +2.97 → −1.65 rad; side: to ∓2.97 rad) in 0.25 s segments, which saturates head_yaw.
- A CMA run with the head frozen at HOME (`best_front_x_nohead.json`) also failed: the robot stayed face-down.

**Torque margin:**
- Every peak is capped at 0.96 by the force limit, so the useful measure is the sweep below.
- Slow front sequence, randomised, at lower torque limits: 0.75 → 83%, 0.60 → 83%, **0.50 → 82%**, 0.40 → 32%, 0.30 → 0%. That is roughly a 2× margin.
- Failures under randomisation are open-loop sensitivity, not lack of torque.

**Head impacts:** the peak head-floor normal force is 13.5 N on the slow front sequence (27 N randomised) and 22–51 N on the roll combos. The whole robot weighs 6.6 N.

## 3. Conclusions and recommendations

1. **Feasibility.** Getting up is feasible within 0.96 N·m for every mode.
   - Front: quasi-statically, with a peak of 0.71 N·m and about 2× margin.
   - Back: either a legs-only rock-up (0.56 N·m) or roll to front then front.
   - Side: roll to front then front.
   - Total motion time is about 3 s from front or back and about 4.5–5 s from the side or via a roll.

2. **The head joints are essential, so the get-up action must be 14-dim** (10 legs + 4 head, beak at HOME). The front get-up uses the head as an arm, and the rolls use the head swing (head_yaw up to ±2.97 rad).
   - Check that the real head_yaw cabling actually allows about ±170°.

3. **The current action mapping is a blocker.** `ACTION_SCALE` 0.5 × `ACTION_CLIP` 1.5 allows only ±0.75 rad around HOME. For example, left hip_pitch can only reach [−1.21, 0.29], but the solutions use ±1.57 on hip_pitch, ±1.57/1.22 on the knee, ±1.4 on the ankle, −1.57 on the neck and ±2.97 on head_yaw.
   - Map the get-up policy's actions to the full ctrlrange instead, e.g. `target = mid + half·tanh(a)` or a per-joint scale.
   - I did not test whether any get-up exists within ±0.75 rad.

4. **Initial-state distribution.**
   - Use a buffer of settled fallen states: uniform random orientation, 2–10 cm drop, random joint noise, HOME or random holding ctrl, settled for about 1–2 s.
   - The natural mix is back 28% / front 26% / left 20% / right 18% / upright 3%. Drop or re-settle the 5% "inverted" states.
   - Add 10–20% mid-fall states (tipping from HOME with 0–4 rad/s) and, later, states recorded when the walking policy terminates.
   - Use the same randomisation as walking.

5. **Episode length and handoff.**
   - Episodes of 8–10 s (400–500 steps at 50 Hz).
   - No fall termination, only timeout or success: gravity z below −0.9, trunk height above 0.10 m and near-HOME posture with low base velocity, held for at least 1 s. Then hand off to the main policy.
   - The main policy's existing termination test (`config.py` `TERMINATION_MIN_HEIGHT` 0.075, `TERMINATION_MAX_GRAVITY_Z` −0.6) works as the trigger to switch to get-up.

6. **Model changes needed.**
   - Add collision for the thighs (critical for side poses, 29.5 mm penetration) and the shins/knees (CMA exploited 15 mm penetration), plus the neck, jaw and optionally hips. A starting set is in `extra_boxes.json`; all are floor-only boxes and cheap in MJX.
   - Give the head top a rounded shape (capsule/ellipsoid) to remove the balance-on-head artifact.
   - The XML is generated (`openduck.xml:1`: generated by `cad/openduck.py`, not to be hand-edited), so add these in `cad/openduck.py` or in `rl/robot_model.build_spec`.
   - Add a head-safety cost: penalise head-floor normal force above about 10 N, or head impact speed, since the head carries the camera, ToF sensor and main board.
   - Add a cost on sustained torque saturation (above 0.9 N·m) for servo heat.

**Scripts** (all in the scratch dir above): `common.py`, `inspect_model.py`, `fk.py`, `falls.py`, `penetration.py`, `make_extra_boxes.py`, `getup_lib.py` (rollout + CMA-ES), `opt_getup.py`, `eval_getup.py`, `eval_roll.py`, `compose.py`, `legs_only_back.py`, `static_torque.py`, `batch1.sh`.

**Outputs** (same folder): `falls.json`, `extra_boxes.json`, `best_*.json`, `combo_*.json`, `eval_combo_*.log`, `batch1.log`.