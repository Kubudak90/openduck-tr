# OpenDuck sit/stand, height command and get-up: research digest

## 0. Summary

1. **The closest prior art is Pollen's `microduck_rl`, and its kinematics are the same as OpenDuck's.** I checked `build/sim/openduck.xml:277` against `microduck_constants.py`. The HOME pose matches exactly:
   - hip_roll ∓0.0873
   - hip_pitch ∓0.4579
   - knee ∓0.0049
   - ankle ±0.4530
   - neck_pitch = head_pitch = 0.349

   STAND_Z is 0.115 in both. Microduck already ships the behaviours you are specifying, so its reward stacks are the best starting point:
   - `Mjlab-SitStand`: a commanded sit↔stand policy.
   - `Mjlab-StandUp`: get-up from face-down, face-up or sitting, then a held stand with a body z/roll/pitch command.
   - `Mjlab-VelStand`: walking plus fall recovery in one policy.

   **No duck project has walking, a height command and sit in one policy.** Microduck uses a separate ONNX per skill and hot-swaps them over a shared 61-D obs contract. Open_Duck_Playground has only `joystick` and `standing` (head commands). Disney BDX has a separate "perpetual" standing policy with a torso-height command and no get-up.
2. **Hard blockers in the current OpenDuck task (repo facts):**
   - **Action range is too small.** The largest target offset is `ACTION_SCALE·ACTION_CLIP` = 0.5×1.5 = **0.75 rad** (`rl/config.py:41-42`). Any useful sit or deep crouch needs a knee change of 1.0–1.22 rad. Microduck uses `joint_pos_action.scale = 1.0` in every env (velocity cfg:273, sitstand:227, standup:197).
   - **Knee flexion limit.** It is **1.2217 rad** (`openduck.xml:141,181`). Microduck's verified SIT keyframe uses knee ±1.35, so it **cannot be copied as is**.
   - **Termination kills the sit.** `TERMINATION_MIN_HEIGHT = 0.075` (`config.py:88`) ends the episode at every sit (z ≈ 0.044–0.06) and at the deepest crouch (z ≈ 0.078).
   - **The height penalty has almost no effect.** `base_height` is −30·(z−0.115)² (`config.py:74`, `task_math.py:139`). At a 1 cm error that is −0.003/s, essentially nothing, and the fixed target fights a height command.
   - **Missing collision geometry.** There are only four collision boxes: trunk, head and two feet (`openduck.xml:120,155,195,240`). Thighs, shins and knees cannot touch the floor and there is no self-collision. For sit and get-up this is a sim2real exploit. Microduck trains standup, sitstand and velstand on an all-collisions model.
   - **Discount is short.** The MJX trainer defaults to `--discounting 0.97` (`rl/train_mjx.py:53`), a horizon of about 0.67 s. The CPU PPO uses 0.99 (`config.py:125`), and so does microduck. A 2 s sit ramp and a 3–6 s get-up need 0.99.
3. **Gentle sit/stand works by tracking a slewed internal target, not a binary one.** The observation stays the raw flag. Microduck's comment: *"with a binary target … crashing won ~7×"*. Use a 2 s ramp, about 0.0275 m/s. Speed-cap penalties and an |a_z| penalty are only backstops.
4. **Rules that were each learned from a failed run:**
   - Never gate a positive reward on being in a bad state. Use potential-based Δcos(tilt) and Δz instead.
   - Use multiplicative composites at goal states (height·upright·pose·head).
   - Introduce smoothness and speed-cap "attempt taxes" only after the skill is discovered (iteration 750–3000, not 0).
   - Give every command slot non-zero samples from step 0, and sample the exact-zero command explicitly.
   - With self-negating penalty functions, use positive weights, and check every logged penalty is ≤ 0.
5. **Get-up initial states:** use reset buckets with a reverse curriculum: stand, sit, face-down, then face-up with ±90° roll noise (which also produces side poses). Spawn just above the ground, not from a 20–25 cm drop. Episodes are 6 s, there is no fall termination (NaN guard only), and smoothing terms are phased in late. HumanUP and HoST show that "discover, then smooth or slow down" is required; doing it in one stage fails.

---

## 1. Height- and posture-conditioned locomotion

| Source | Command | Range | Reward (form · weight) | Notes |
|---|---|---|---|---|
| **Walk These Ways** (Go1) `scripts/train.py:122,156` | body_height Δ from `base_height_target=0.30` | **[−0.25, +0.15] m** | `jump`: −(z − (0.30+h))², **+10** | Also gait freq [2, 4] Hz, footswing [0.03, 0.35], pitch/roll. Rewards combined "ji22 style": r⁺·exp(r⁻/0.02) (`only_positive_rewards_ji22_style=True`, `sigma_rew_neg=0.02`). |
| **HOMIE** (G1 / GR-1) arXiv 2502.13013 | h_t torso height | G1 walk 0.74, squat [−0.24, 0.74] | Height tracking exp(−4‖h_t−h‖²) **2.0**; squat knee −‖(h_r−h_t)·(q̂_knee − ½)‖ **−0.75**; stand_still −0.15 | **1/3 of envs train squat, 2/3 stand/walk**, resampled every 4 s in the same env, so transitions get trained. No velocity-reward gating is described. |
| **HugWBC** (H1) arXiv 2502.03206 | body h | [−0.3, 0] m | Height tracking **−40**, the largest behaviour weight; pitch −10; foot swing −30 | Standing gait has constant phase 0.25 for both feet (stance). Height and pitch decouple worse at low h. Standing error E_h 0.035 vs walking 0.064. |
| **Microduck StandUp** body_pose (`microduck_standup_env_cfg.py:116-121`) | z, roll, pitch from STAND_Z | z **−0.04 / +0.03 m**, angles ±15°, x/y "alive" ±0.005, **zero-command prob 0.3** | `body_pose_tracking_locomotion`, mean of per-axis Gaussians (z_std 0.01–0.03), ramped **0 → 1.5 (iteration 2500) → 3 → 4 (iteration 4000)** | Only while standing still. The fixed-stand terms are "relaxed" (down-weighted) as tracking ramps in: composite 3.75→1.5, upright_sharp 1.5→0.5. |
| **Microduck Velocity** (`microduck_velocity_env_cfg.py:679-727`) | same 6-D slot | ±0.005 m | weight **0 (disabled)** | They deliberately do **not** train height while walking. `body_pose_tracking_locomotion` has a `vel_gate` exp(−(‖v_cmd,xy‖/0.1)²), added after a run where "tracking vs walking conflict … prevented … learning either well". |
| **Disney BDX** arXiv 2501.05204 | perpetual policy: torso height + orientation, head Δ | not given numerically (Fig. 4) | Imitation-based | Separate standing and walking policies, switched at the next double support. Observes a_{t−1} and a_{t−2} to keep setpoints continuous. Early termination on head or torso ground contact. 50 Hz, low-pass 37.5 Hz. |

**Sim measurement on OpenDuck.** Script: `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/sitstand/research/scripts/crouch_kin.py`. For each knee flexion it solves a flat-foot pose with the CoM over the feet, then holds the ctrl targets for 3 s on the raw `scene.xml`:

| knee Δ target | Δhip, Δankle | settled z (m) | tilt (°) | actual knee | max \|τ\|/0.96 |
|---|---|---|---|---|---|
| 0.0 | +0.11, −0.11 | 0.113 | 11.7 | 0.095 | 0.11 |
| 0.4 | +0.35, +0.05 | 0.102 | 13.7 | 0.516 | 0.14 |
| 0.6 | +0.48, +0.12 | 0.095 | 14.3 | 0.723 | 0.15 |
| 0.8 | +0.61, +0.19 | 0.089 | 14.3 | 0.927 | 0.15 |
| 1.0 | +0.75, +0.25 | 0.081 | 14.0 | 1.128 | 0.15 |
| 1.2 | +0.91, +0.29 | 0.078 | 3.7 | 1.223 (limit) | 0.03 |

**Conclusions from the sweep:**
- An upright flat-foot crouch spans only **0.115 → about 0.078 m (about 3.7 cm)**, and the trunk never reaches the floor.
- Under the current 0.75 rad action cap the crouch bottoms out near z ≈ 0.095.
- Gravity sag is about 0.1–0.13 rad at kp 1.1, so joint targets must overshoot the desired angle.
- Torque is not the limit: static torque is ≤ 15 % of 0.96 N·m.
- The open-loop tilt of 11–14° is an artefact of holding ctrl without a policy.

**Recommendation for the main policy's height command:** h ∈ **[−0.030, 0] m** while walking, extended to **[−0.035, 0]** at zero velocity. This is the microduck crouch range. Add no extension above STAND_Z: there is only about 1 cm of leg travel there. Feed h/0.035 to the observation so the input is O(1).

---

## 2. Sit and stand as a commanded mode in one policy

### 2a. Microduck SitStand recipe

Source: `microduck_sitstand_env_cfg.py`, mjlab 1.3.0, rsl_rl. Weights are "per second": mjlab, like Isaac Lab, multiplies by step_dt, which is the same convention as OpenDuck's `Σ w·r·CTRL_DT`. I did not verify this in mjlab's source.

**Command**
- `cmd = [sit_flag, 0, 0]` in the twist slot; "stand" is the all-zero command.
- Dwell **U(3.5, 6.5) s**, `SIT_PROB = 0.5`, episode **12 s**.
- The internal `alpha` slews toward the flag at 1/`ramp_s` per second, with **ramp_s = 2.0**.
- On the first step alpha is initialised from the actual trunk z, so a seated spawn is not dragged upward (`mdp.py:6234-6331`).

**Targets**
- STAND = HOME and z = 0.115.
- SIT = knee ±1.35, hip_pitch ∓0.4079, ankle 0, hip_roll 0, z = 0.060. **Not feasible on OpenDuck** (see below).
- Target pose = HOME + alpha·(SIT − HOME), target z = 0.115 + alpha·(0.060 − 0.115).

**Rewards** (σ = std; "+w on a self-negating function" means the function already returns ≤ 0):

| Term | Formula | Weight |
|---|---|---|
| posture_pose_legs | mean exp(−((q−q*)/0.5)²) over 10 leg joints | 4.0 |
| posture_pose_l1 | −mean\|q−q*\| | +1.0 |
| posture_height | exp(−((z−z*)/0.04)²) | 1.0 |
| posture_height_sharp | same, σ 0.015 | 1.0 |
| posture_height_l1 | −\|z−z*\| (makes "wrong posture" net-negative) | +6.0 |
| rise_bootstrap | clamp(v_z, 0, 0.08)·[z<0.125]·(1−sit) | 0.75 |
| descent_speed | −clamp(−v_z − 0.05, 0) | +10 → +20 (iteration 500) |
| rise_speed | −clamp(v_z − 0.08, 0) | 0 → 5 (iteration 1500) → 10 (iteration 2500) |
| gentle_motion | −\|a_z\| (finite difference of v_z, zeroed at reset) | +0.05 |
| upright_linear | cos tilt = 1 − 2(q_x² + q_y²) | 2.5 |
| upright_while_tall | cos tilt · smoothstep((z − 0.075)/(0.10 − 0.075)) | 1.5 |
| posture_stillness | exp(−(\|v\|/0.05)²) · z-band gate (full ≤ 12 mm, 0 at 30 mm) · tilt gate (full ≤ 25°, 0 at 60°) · [ramp done] | 2.0 |
| posture_composite | exp(−(Δz/0.03)²) · exp(−tilt²/0.4²) · exp(−pose_rms²/0.4²) · exp(−head_rms²/0.4²) | 3.0 |
| head_pose_tracking | — | 0.75 |
| action_rate_l2 | — | −0.1 → −1.0 (iterations 0–1500, steps every 250) |
| joint_torque_rate | — | 0 → −5e-4 (iteration 750) → −1e-3 (iteration 1250) |
| body_ang_vel | — | −0.05 |
| angular_momentum | — | −0.02 |
| self_collisions | — | −1 |

**Rest of the recipe**
- Terminations: **no fall termination**, NaN guard only.
- Reset: 50 % stand, 50 % seated keyframe (joint noise σ 0.10, tilt ±8°, z 0.06–0.075).
- Pushes are delayed: 0 until iteration 1000, then ±0.05 → ±0.3 by iteration 2500. Early pushes made the policy "unlearn sitting".
- Physics: `nconmax = 200`, solver iterations 30, ls_iterations 50, which fixed NaN in seated contact.
- PPO: 512-256-128 networks, γ 0.99, λ 0.95, entropy 0.01, adaptive KL 0.01, 15k iterations max.

**Failure modes they documented, with the fix for each:**

| Failure | Fix |
|---|---|
| Crash-sit when the descent cap was −5 | −10 from step 0, then −20 |
| Lying still on the back scored like a sit | Tilt gate on stillness |
| Plank or head-tripod rest | Composite term, plus a head factor |
| Head dangling to the floor at rest | head_std factor |
| Stall in a half-finished rise | Rise cap moved later, to iterations 1500/2500 |
| Double-negative sign bug, run 7ev90yd9 | Penalties became rewards and trained "butt-hopping" |

### 2b. Recommended adaptation for OpenDuck's single main policy

**Command and observation:** 49 → **51** dims, `command = [vx, vy, wz, h/0.035, sit]`.
- While sit = 1, zero vx, vy, wz in the obs and the rewards, and make that the runtime contract. This is my design choice, not taken from a source.

**Internal (not observed) slewed targets**
- z*_walk = 0.115 + h, slewed at ≤ 0.03 m/s.
- alpha_sit slewed over 2.0 s.
- z* = (1−α)·z*_walk + α·Z_SIT.
- q* = HOME + α·(SIT − HOME) on legs, plus the crouch-pose blend from the §1 sweep for the h component (optional).
- Put α, z* and time-since-flip in the **critic** obs. This is a suggestion; microduck does not do it.

**Gating the walking rewards with g_walk = (1−α)**
- `tracking_lin/ang` × g_walk.
- In the sit or transition phase, add a zero-velocity term: α·exp(−‖v_xy‖²/0.02) and α·exp(−ω_z²/0.1), or use stillness.
- `feet_air_time` × moving × g_walk.
- `feet_phase` × g_walk. It currently applies even at zero command (`task_math.py:122`).
- `hip_pose`: keep it, because SIT hip_roll = 0 vs HOME ±0.087, so the cost is about 0.015 rad², negligible.

**Phase handling when stationary or sitting**
- Booster Gym: 10 % "still" envs set `gait_frequency = 0`, the phase obs becomes cos·[f>0] and sin·[f>0] (so 0, 0), and swing rewards are disabled (`envs/t1.py:381-383,585-586,728-729`).
- HugWBC: constant stance phase.
- Disney: walk→stand switch at double support; when walking starts, the first step is chosen by turn direction.
- **Recommendation:** when α > 0 (or the sit flag is set), freeze the phase and output (0, 0). Replace feet_phase with "both feet in contact". On resuming the walk, reset the phase deterministically to 0, and do the same in the runtime.
- Keep OpenDuck's existing COMMAND_ZERO_PROB = 0.15 stepping-in-place behaviour only for sit = 0.

**Height reward:** replace `base_height` −30·Δz² with:
- the microduck pair: Gaussian σ 0.04 (w 1) + Gaussian σ 0.015 (w 1) on (z − z*), plus L1 (w +6, self-negating).
- Scale these against OpenDuck's positive task mass, about 6.5/s (tracking 2+2, air time 1, phase 1, alive 0.5). Microduck balances about 12 against the same regulariser set, and their rule is to compare reward *mass*, not raw weights.

**Gentleness:** add descent_speed (+10 → +20), rise_speed (0 → 5 → 10, late), |a_z| (+0.05), rise_bootstrap (0.75, only when sit = 0 and z < z* + 0.01), the stillness term and the posture composite, all as in 2a.

**Terminations**
- Drop the z < 0.075 rule.
- Keep tilt > 53° (`config.py:89`), since falls go to the get-up policy.
- Optionally terminate on trunk-floor contact when α < 0.05: an unplanned butt landing counts as a fall.
- Never terminate when α > 0.

**Mode mix per resample** (my synthesis; tune it):
- Walk (velocity random): ~50 %, of which turn-in-place ~15 % (microduck's bucket).
- Exact zero velocity, h = 0: 15 %.
- Stand still with h random: 10 %.
- Sit: 25 %.
- Height h is also random during walks. HOMIE uses a 1/3 squat share; microduck uses 50/50 sit/stand.
- Dwell: sit and stand U(3.5, 6.5) s, and keep 20 s episodes so each has ≥ 1 sit↔stand cycle.

**Reset mix:** HOME 60 %, seated keyframe 25 % (joint noise σ 0.1 rad, ±8° tilt), random crouch 15 %. Reset state and command are sampled independently, so "hold" and "transition" cases are all covered.

**SIT keyframe for OpenDuck: not yet established.** Candidates from a quick test (`pose_settle.py`, 3 s hold, no noise; Δ values in the left-leg sign convention, mirrored on the right):

| Candidate | Δhip_roll | Δhip_pitch | Δknee | Δankle | Result |
|---|---|---|---|---|---|
| A (microduck-like, knee at the limit) | +0.087 | +0.05 | 1.22 | −0.453 | z 0.052, tilt 14°, trunk and feet on the floor |
| B | — | −0.8 | +1.0 | +0.2 | z 0.044, tilt 2°, trunk and feet on the floor |

These signs come from my own convention, so verify them. Before training, re-run microduck's procedure: noisy inits, 3 s hold, and check **tilt**, not z. This needs ACTION_SCALE ≥ about 1.0, because knee Δ ≈ 1.0–1.22 plus about 0.1 rad of gravity sag must be reachable.

---

## 3. Learned fall recovery / get-up

### 3a. Source data

| Work | Key design facts |
|---|---|
| **Microduck StandUp** (`microduck_standup_env_cfg.py`) | Episode **6 s**, no fall termination (NaN guard incl. contact forces), action scale 1.0 HOME-relative, 14 joints incl. head. **Reset curriculum** (stand/sit/face-down/face-up): 0 iterations {0.40, 0.40, 0.20, 0.00} → 600 {0.25, 0.30, 0.35, 0.10} → 1500 {0.20, 0.25, 0.30, 0.25} → 2500 {0.15, 0.20, 0.30, 0.35}. Prone z 0.05–0.09 (rests at 0.044; the default 0.20–0.25 drop is avoided). **Face-up roll noise ±90° about the long axis**: the reward landscape between supine and prone is flat, so back recovery was "seed-lucky" (1 in 4) without it. Rewards: pose Gaussian σ 0.5 **2.0**; pose L1 **+1.25**; height Gaussian σ 0.04 **1** + σ 0.015 **1**; height L1 **+7.5**; com_upward_velocity clamp(v_z, 0)·[z<0.125] **0.75** (uncapped: capping at 0.15 or 0.30 broke prone discovery); \|a_z\| **+0.005** (ceiling; −0.01 froze face-up); upright_linear **1.5**; upright Gaussian σ 0.3 gated smoothstep(z: 0.060→0.115) **1.5**; standing_composite (σ 0.04 / 0.40 / 0.40) **3.75**; action_rate −0.1→−1.0 (iteration 1500); arrival_damping ω_xy² gated (z 0.09→0.11, tilt full ≤ 20°, 0 at 45°) **0 → −0.025 (iteration 3000) → −0.05 (iteration 4000)**; torque_rate 0 → −1e-3 (iteration 3000); self-collision −1. Task weights were all ÷4 after a "violent/shaky on real robot" transfer failure (task mass 49 → 12). |
| **Microduck VelStand** (walking + recovery, one policy) | Recovery rewards gated on **tilt > 40° only**: gating on low z let the robot sit and farm. Potential terms: `upright_progress` = Δcos(tilt) **×5**; `height_progress` = Δmin(z, 0.115). `fallen_state_penalty` with hysteresis (arms on fall, releases at tilt < 25° and z > 0.09). `recovery_success` one-shot bounty (fallen ≥ 0.5 s → tilt < 25°, z > 0.09). `fallen_too_long` termination after **8 s** (5 s cut episodes at the last mile). Phases: fell_over on until iteration 500; tax/bounty economics on at **1200** (a tax-free natural-fall window is needed); prone resets ramp 0 → 0.45 over iterations 1500–2500; a crouch reverse-curriculum slice of 0.15 from iteration 800. The walk was starved at 2/3 prone resets (25 % walking data). **This supports keeping get-up a separate policy.** |
| **HumanUP** (G1) arXiv 2502.12152 | Stage I (discovery): simplified collision, canonical pose, weak regularisation. Rewards: base and head height (e^h − 1) **5** each; Δheight indicator **1**; standing-on-feet **2.5**; feet height exp(−10h) **2.5**; upright **0.25**; soft symmetry **−1**; termination **−500**. Stage II: full URDF, **20K supine + 20K prone poses** (randomise joints, drop from 0.5 m, simulate 10 s), tracks the Stage I trajectory **slowed 8×** (4× gave large torques; 10× did not converge); joint tracking exp(−Δq²/4) **8**, torque −0.003, dof_vel −1e-3, action_rate −0.1. Held-out sim success: supine 95.3 / roll 94.4 / prone 92.1; without Stage II 24.8 / 43.5 / 27.6; without posture randomisation 65.4 / 37.3 / 30.3; hard symmetry 84.6 / 75.5 / 67.1. Real: 78.3 % vs vendor 41.7 %, about 6 s to stand. |
| **HoST** (G1) arXiv 2502.08378 | Four reward groups (task 2.5, style 1, regularisation 0.1, post-task 1) with **one critic per group**, normalised advantages summed. **Without multi-critic: 0 % success.** Upward assist force 200 N, −20 N per success, applied only when the trunk is near-vertical; **without it 0 %**. Action bound p_d = p_t + β·a, β 1 → 0.25 (−0.02 per success); without it, smoothness 9.52 vs 2.90. L2C2 λ_π = 1, λ_V = 0.1; without it, real success 11/20 vs 20/20. Post-task terms gated by h_base > 0.65 m. Rollout 500 steps (10 s). Initial joints offset U(±0.1), scale U(0.9, 1.1). |
| **Lee, Hwangbo, Hutter 2019** (ANYmal) arXiv 1901.07517 | Three behaviours (self-right, stand up, locomote) plus a learned selector. Self-righting drops from 0.5 m with random joints. Bounded logistic-kernel costs, because unbounded costs made early termination preferable. Smoothness cost curriculum from about 0, else "stopped moving". One policy for everything gave ungraceful, conservative motion. Joint wrap-around (≥ 2π) broke recovery. Height estimator needed after falls. > 97 % over 100 trials, < 5 s. |
| **Playground Go1Getup** (`mujoco_playground/_src/locomotion/go1/getup.py`, in the local venv) | 60 % reset from a **0.5 m drop with a uniform random quaternion and uniform random joints within limits**, 40 % HOME; root velocity U(±0.5); **0.5 s settle inside reset**. Actions are **relative to the current joint position** (`q + 0.5·a`; HOME-relative "didn't work as well"). Rewards: orientation exp(−2‖up − g‖²) **1**; height e^{min(z, z_des)} − 1 **1**; posture exp(−0.5‖q − q₀‖²) gated (upright and at height) **1**; stand_still exp(−0.5‖a‖²) gated **1**; action_rate −0.001; torques −1e-5; dof_vel −0.1; dof_acc −2.5e-7; dof_pos_limits −0.1. Episode 300 steps (6 s), no termination, 50 M steps, 512-256-128 networks. |
| **Tao et al. 2022** "Learning to Get Up" arXiv 2205.00307 | Discover with a strong character, then a weakness curriculum, then slow down. The result is quasi-static get-ups that can be paused. |

### 3b. Recommended getup.onnx design

**Observation:** gravity (3), gyro (3), q−q₀ (14), q̇·0.05 (14), last_action. No command and no phase.
- **Let the get-up policy drive the head and neck as well (14 actions).** The head is about 40 % of the mass, microduck explicitly allows the head as a support, and all microduck policies drive 14 joints. This is uncertain for OpenDuck's runtime contract.

**Action space:** HOME-relative with scale ≥ 1.0. This makes the handoff and the final pose identical to the main policy. The alternatives are Go1/HoST-style current-relative actions with a β schedule 1 → 0.25. Pick one deliberately.

**Initial states:** build an offline bank of settled poses with CPU MuJoCo, then sample from it in the MJX reset. This is cheaper than settling inside the jitted reset, and is HumanUP's approach.
- Bank construction: random quaternion (or face-down / face-up ± roll 90° for side poses), random joints within limits, drop from 0.10–0.20 m (the robot is 27 cm tall), settle about 1 s with the PD holding the random targets.
- Mix it with stand, sit-keyframe and crouch buckets on microduck's schedule above.

**Rewards:** start from the microduck StandUp stack, i.e. the table in 3a with the same stds; STAND_Z must be measured on OpenDuck. Add:
- VelStand's potential terms upright_progress (×5) and height_progress.
- Termination: NaN only. Episode 6–8 s.
- Smoothing curricula start at 0 and appear at roughly 60–75 % of the run.

**Sign check:** OpenDuck's `task_math` returns positive costs with negative weights, which is the base-mjlab convention. If you port microduck functions, keep their self-negating form with positive weights, or invert them. Then assert every per-term episode sum is ≤ 0 for penalties.

**Handoff to the main policy**
- Switch to main when tilt < 25°, z > 0.09–0.105, and ‖ω‖ < 0.5 rad/s for ≥ 0.3 s. The tilt/z part is microduck's "recovery complete" definition; the dwell and ω parts are my addition.
- Trigger the get-up when tilt > 53° (the current termination) or trunk/head floor contact persists > 0.3 s.
- On the switch, seed the main policy's `last_action` with (ctrl_target − q₀)/ACTION_SCALE, clipped, so action_rate does not spike. Disney relies on a_{t−1} and a_{t−2} for continuity.
- Train the get-up with a 15–40 % "already standing" bucket so it holds the stand instead of rushing.
- Add a slice of main-policy resets drawn from the get-up end-state distribution.

**Physics**
- Add capsule collision geoms on thighs and shins (class `collision`: contype 0, conaffinity 1), plus a self-collision penalty or contype pairs for legs vs trunk and head.
- Raise the MJX contact budgets: Go1Getup uses `naconmax = 30·8192` and `njmax = 250`; microduck sitstand uses `nconmax = 200` with solver iterations 30/50.

**Common pitfalls to expect:**
- Lying still beats trying, unless there is a fallen tax with hysteresis, introduced late.
- Parking in a deep crouch just past the gates (fix: a reachable success definition plus Δz potential).
- Head tripod.
- Violent, fast flips. Fix with late arrival damping, action rate and an action bound, not speed caps: microduck notes a 25 cm robot naturally tumbles at 3.5–5.5 rad/s.
- Sim exploits through missing knee/shin collision geoms.

---

## 4. Brax / MJX PPO tips for multi-mode training

- **Normalisation:**
  - Keep `normalize_observations=True` (Welford) and bake the normaliser into the ONNX (`train_mjx.py:361`).
  - `normalize_advantage` defaults to True in brax 0.14.2, so only reward *ratios* matter; `reward_scaling = 1` is fine. Balance task mass against regularisers per mode.
  - The sit-only terms must not dwarf walking tracking during walk episodes; the gating in §2b does this.
- **Curriculum inside jit:**
  - The repo's `info["steps"]` is episode-local: it is reset on done at `rl/env_mjx.py:506`, although it is listed in `_KEEP_INFO` at line 107.
  - Add a monotone per-env `global_step` to `_KEEP_INFO`, incremented every step (all envs step in lockstep), and implement stages as `jnp.where` step functions, as microduck's `reward_weight` does.
  - Brax's eval env starts at counter 0, so eval metrics run stage-0 settings. Either pass the stage explicitly or read the training metrics.
  - Alternative: staged runs with `ppo.train(restore_params=…/restore_checkpoint_path=…)` (brax 0.14.2, `train.py:237-240`) and a different static config per stage.
- **Resets:** the repo already does full fresh resets via `auto_reset` and `merge_reset`, so per-episode random fallen or seated states work. Playground's default `BraxAutoResetWrapper(full_reset=False)` would reuse a cached first state; avoid it for get-up.
- **Mode sampling:**
  - Sample the mode in `reset` and at resample with `jax.random`, using explicit buckets: exact-zero velocity, turn-in-place, sit-hold and sit-transition.
  - Log per-mode episode metrics in `info["episode_metrics"]`, plus success rates per reset bucket.
  - Watch for a main-task metric that steps *down* at curriculum boundaries: that means the pacing is wrong.
- **Hyperparameters:**
  - γ 0.99 for both new tasks (repo MJX default 0.97 at `train_mjx.py:53`); λ 0.95; entropy 0.005–0.01.
  - Networks: actor 512-256-128 for multi-mode. Microduck uses this for sitstand; the repo's CPU default is 256-128-64.
- **Budgets for reference:**
  - Microduck: simple episodic tasks about 1000 iterations × 4096 envs × 24 steps ≈ 0.1 B steps; curriculum-heavy recovery 4000–6000 iterations ≈ 0.4–0.6 B.
  - Go1Getup: 50 M. HumanUP Stage I: about 5 B.
- **Multi-critic** (HoST) would need a custom Brax PPO loss. Consider it only if additive multi-mode training fails; HoST's ablation went from 0 % to 99 %.

---

## 5. Uncertain or unverified

- Microduck results come from its code comments and documented runs, not from papers; there are no independent numbers.
- The mjlab reward-×dt convention is assumed, not checked in mjlab's source.
- The OpenDuck SIT candidates and the crouch heights in §1 and §2b come from open-loop 3 s holds without noise. Joint-sign conventions in `pose_settle.py` are my own. A 14° tilt is not yet "stable".
- I could not see HumanUP's termination triggers or episode length. HoST's critic weights wᵢ are not given.
- Not read beyond the abstract: UniReLo (arXiv 2606.08922) and arXiv 2511.06371. Both claim unified recovery-to-locomotion policies on G1.
- My own design choices, not taken from sources:
  - zeroing the velocity obs while sitting
  - critic access to α
  - the handoff dwell and ω thresholds
  - the main-policy mode-mix percentages
  - letting the get-up policy drive the head (14 actions)

---

**Files written (scratch only, repo untouched):**
- Scripts:
  - `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/sitstand/research/scripts/pose_settle.py`
  - `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/sitstand/research/scripts/crouch_kin.py`
- Downloaded reference sources (data only, not executed):
  - `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/sitstand/research/dl_microduck/`
  - `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/sitstand/research/dl_odp/`
  - `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/sitstand/research/dl_wtw/`
  - `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/sitstand/research/dl_booster/`

**Sources:**
- [microduck_rl repository](https://github.com/pollen-robotics/microduck_rl) and its files: [sitstand cfg](https://github.com/pollen-robotics/microduck_rl/blob/main/src/mjlab_microduck/tasks/microduck_sitstand_env_cfg.py), [standup cfg](https://github.com/pollen-robotics/microduck_rl/blob/main/src/mjlab_microduck/tasks/microduck_standup_env_cfg.py), [velstand cfg](https://github.com/pollen-robotics/microduck_rl/blob/main/src/mjlab_microduck/tasks/microduck_velstand_env_cfg.py), [mdp.py](https://github.com/pollen-robotics/microduck_rl/blob/main/src/mjlab_microduck/tasks/mdp.py), [CLAUDE.md](https://github.com/pollen-robotics/microduck_rl/blob/main/CLAUDE.md)
- [Open_Duck_Playground](https://github.com/apirrone/Open_Duck_Playground)
- [Disney BDX, arXiv 2501.05204](https://arxiv.org/abs/2501.05204)
- [Walk These Ways, arXiv 2212.03238](https://arxiv.org/abs/2212.03238) and [code](https://github.com/Improbable-AI/walk-these-ways)
- [HOMIE, arXiv 2502.13013](https://arxiv.org/abs/2502.13013)
- [HugWBC, arXiv 2502.03206](https://arxiv.org/abs/2502.03206)
- [Booster Gym](https://github.com/BoosterRobotics/booster_gym)
- [HumanUP, arXiv 2502.12152](https://arxiv.org/abs/2502.12152)
- [HoST, arXiv 2502.08378](https://arxiv.org/abs/2502.08378)
- [Lee et al. 2019, arXiv 1901.07517](https://arxiv.org/abs/1901.07517)
- [Learning to Get Up, arXiv 2205.00307](https://arxiv.org/abs/2205.00307)
- [MuJoCo Playground](https://github.com/google-deepmind/mujoco_playground) (Go1Getup, BraxAutoResetWrapper; read from the local venv)
- [UniReLo, arXiv 2606.08922](https://arxiv.org/abs/2606.08922)
- [arXiv 2511.06371](https://arxiv.org/abs/2511.06371)