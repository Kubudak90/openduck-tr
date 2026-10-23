# RL change map for OpenDuck-TR (35 cm, 1.65 kg, PWM hobby servos, B2-ESP32)

## Summary
I read every RL file on the task list (config, tasks, task_math, robot_model, env_mjx, env_cpu, pose_bank, train_mjx, play, eval_tasks, TASK.md, sit-stand-getup spec) plus policy_meta, the MJCF and the two research notes. I also ran 3 small scratch simulations. The repo was not touched.

Almost every number in rl/config.py is tied to the 27 cm, 0.676 kg robot with 15 identical XL330 servos (kp 1.1, ±0.96 N.m). The physics constants come from one place, cad/openduck.py:84 → cad/export.py:171, which writes the single `openduck` MJCF class. Everything else falls into two groups:
- **Re-derive from the new CAD:** heights, the crouch table F, SEAT, HEAD_SIT, joint caps, EXTRA_COLLISION, foot box, HOME.
- **Rescale:** use Froude scaling (s = 1.282, √s = 1.132, torque k_legs = 3.13 and k_head = 1.91), times a speed-reduction factor κ, because these servos are too slow for a Froude-similar gait.

**Key findings:**
1. **Walking is realistic only when slowed.**
   - With the speed spec Direnc lists for the DS3230MG (0.12 s/60° at 6.8 V), the knee utilisation u (from the parts-research fit) is 0.96 at Froude-similar speed, 0.72 at κ = 0.72 (gait 1.6 Hz, vx ≤ 0.245 m/s) and 0.62 at κ = 0.6 (gait 1.3 Hz).
   - servodatabase and aifitlab list 0.18 s/60° instead. If that is right, u is 1.41 at κ = 1 and 1.02 at κ = 0.72, so κ ≈ 0.5 would be needed.
   - So the servo's no-load speed must be measured on the bench before choosing GAIT_FREQ and command ranges.
2. **main (sit/stand/crouch) is the most realistic task.** At the 1× need, the margins are hip_pitch 3.9×, knee 2.0×, neck 1.8× and head 1.9×, and sit/stand tolerates slow servos.
   - Neck static load in a crouch is 0.56–0.67 of the ES3054 stall torque, so crouch head poses must be chosen to null the neck torque.
   - Random-head domain randomisation (DR) must shrink: head peaks would reach 1.5× MG90S stall.
3. **getup is not realistic with the current head-assisted strategy.**
   - Torque: neck 0.51×, head 0.32× and hip_yaw 0.73× of what is needed.
   - Speed: every servo has about 0.5× of the equivalent ω0 = 18 rad/s the get-up scripts needed.
   - It becomes a stretch goal: a leg-only RL get-up (unproven), or a hardware change such as a DS3230MG on neck_pitch.
4. **Actuator model.**
   - Per-class MuJoCo `position` actuators: kp = τs / P-band, forcerange = stall torque at the rail voltage, joint damping b = τs/ω0 (back-EMF, independent of voltage), plus armature and frictionloss from sysID.
   - Dead band and backlash are not available as MJX-JAX actuator types (no USER gain/bias/dyn), so the dead band is applied in env code as per-substep ctrl shaping. Backlash is an optional passive play joint.
   - My sim sweep shows SIM_DT = 5 ms stays stable and matches 2 ms for kp up to 175 N.m/rad. Stiff servos also remove the 7.8° trunk sag that the HOME/height constants were tuned around.
5. **Observation model.**
   - Leg joints: potentiometer readings with per-episode bias, noise, quantisation, 1-substep latency, and qd from a 200 Hz difference plus IIR filter. The firmware must implement the identical filter.
   - Head joints: commanded, slew-limited ctrl. The sim must put exactly this into obs, while rewards and critic keep the true q.
   - Actuation delay: 0–4 substeps at a 50 Hz PCA9685 frame, 0–1 at 330 Hz.
6. **Network.** Use a 256-128-64 actor (54k MAC, 214 KB float32, fits in the 512 KB SRAM): about 0.4 ms with ESP-DSP-class kernels, 1.2 ms in plain C. The current MJX default actor 512-256-128 (190k MAC, 747 KB) must be streamed from PSRAM, about 9–19 ms per step, which is not acceptable at 50 Hz. Only the critic stays large.

**Effort:** the full change list is about 27–38 person-days plus training runs.

## Facts
## F1. Measured or computed by me (scratch: /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s8/rl-change-map/)

**Current model facts** (CPU venv, `build_model`):

| quantity | value |
|---|---|
| total mass | 0.6763 kg |
| neck subtree | 0.2484 kg (37 %) |
| HOME trunk origin z | 0.1193 m |
| COM height | 0.137 m |
| head top | 0.2737 m |
| thigh (hip_pitch→knee) | 42.2 mm |
| shin (knee→ankle) | 49.4 mm |
| ankle→sole | 23.6 mm |
| foot box | 54 × 40.5 × 5 mm, x range −20..+34 mm |
| trunk box half-sizes | 41.9 × 33.0 × 42 mm |
| actuators | all 15: kp 1.1, ±0.96 N.m, damping 0.053, armature 0.0018, frictionloss 0.0048 (build/sim/openduck.xml:7-8, from cad/openduck.py:84) |
| dof_M0 diagonal | 0.0018–0.0037; the armature dominates distal joints |
| SIM_DT | 0.005 (robot_model overrides the 0.002 in the XML) |

**Stiff-servo stability sweep** (`scripts/stab3.py` → `stab3.json`, `stab3.log`):
- Setup: the 27 cm model with Froude-equivalent servo parameters (τe = τs/k, ωe = ω0√s, kpe = kp/k, armature_e = arm/(k·s)). DS3230MG on hip_roll, hip_pitch, knee and ankle; ES3054 on hip_yaw and neck; MG90S on the head joints.
- Sweep: P band 1, 2, 4, 8° (kp_new 175, 88, 44, 22 N.m/rad); armature_new 2e-3, 5e-3, 1e-2 kg·m²; dt 5 ms and 2 ms.
- Result: all 24 runs are stable. A 0.5 rad knee step on the welded trunk reaches 90 % in 55–75 ms (speed-limited) with 1–4° overshoot. dt 5 ms and 2 ms differ by ≤ 4 ms in t90 and ≤ 0.4° in overshoot. The robot stands open-loop on HOME targets with pitch ≤ 0.7°; the current kp 1.1 sags 7.8° (`scripts/dbg3.py`).
- Pitfall: calling `mj_setConst` after `mj_resetDataKeyframe` resets qpos to qpos0. My first sweep made this mistake.

**Derived numbers** (`scripts/derive.py` → `derive.log`):

| quantity | value |
|---|---|
| s = 350/273 | 1.282 |
| √s | 1.132 |
| mass ratio | 2.44 |
| k_legs = (m_new/m_old)·s | 3.13 |
| k_head = (0.370/0.248)·s | 1.91 (the 0.370 kg head uses the torque-needs head mass model) |
| Froude gait frequency | 2.5/√s = 2.21 Hz |

| κ (joint-speed factor) | 1.0 | 0.8 | 0.72 | 0.6 | 0.5 | 0.45 |
|---|---|---|---|---|---|---|
| GAIT_FREQ (Hz) | 2.21 | 1.77 | 1.59 | 1.32 | 1.10 | 0.99 |
| knee u, DS3230MG 3.06 N.m / 8.3 rad/s (Direnc speed) | 0.96 | 0.79 | 0.72 | 0.62 | 0.54 | 0.51 |
| knee u, 2.85 N.m / 5.6 rad/s (servodatabase 0.18 s/60°) | 1.41 | 1.13 | 1.02 | 0.87 | 0.75 | 0.69 |
| knee u, clone at 0.7× torque | 1.01 | 0.85 | 0.78 | 0.73 | 0.73 | 0.73 |

- u uses the parts-research fit `u = max(0.02/τe + 8.8κ/ωe, 0.13/τe + 7.7κ/ωe, 0.5/τe)`; u ≤ 0.5 means 2× margin.
- Command ranges at κ = 0.72: vx (−0.12, 0.245) m/s, vy ±0.12 m/s, wz ±0.64 rad/s, AIR_TIME_TARGET 0.236 s. At κ = 0.6: vx (−0.10, 0.20), vy ±0.10, wz ±0.53.

**Torque margins at 35 cm** (available stall ÷ 1× need; needs are the torque-needs.md group minimums × k):

| joint | walk + sit | get-up |
|---|---|---|
| hip_yaw | 5.5× | **0.73×** |
| hip_roll | 16× | 9.8× |
| hip_pitch | 3.9× | 2.2× |
| knee | 2.0× | 2.2× |
| ankle | 4.9× | 3.3× |
| neck | 1.8× | **0.51×** |
| head | 1.9× | **0.32×** |

- Get-up speed: equivalent ω0 needed is 18 rad/s. Available: DS3230MG 9.4 (0.52×), pessimistic DS3230MG 6.3 (0.35×), ES3054 9.1 (0.51×), MG90S 14.8 (0.82×).
- Peaks and holds scaled from torque-needs:
  - neck in walk with random-head DR: 0.31 N.m (0.89 × ES3054 stall)
  - head in walk: 0.33 N.m (1.5 × MG90S stall)
  - neck static hold at crouch h 0.070: 0.19 N.m (0.56 × stall); deep crouch with head at HOME: 0.67 × stall
  - hip_yaw peak in walk with DR (old policy): 4.4 × ES3054 stall
  - knee walk RMS 0.94 N.m against DS3230MG stall/3 = 1.02 N.m (thermal check is borderline)
- Back-EMF damping b = τs/ω0:

| servo | b (N.m·s/rad) |
|---|---|
| DS3230MG (6.5 V) | 0.369 (0.509 with servodatabase speed) |
| ES3054 | 0.043 |
| MG90S | 0.0165 |
| SG90 | 0.017 |

- PCA9685 resolution: 4.88 µs/LSB at 50 Hz (0.44° on a 180°/2000 µs servo), 1.22 µs at 200 Hz (0.11°), 0.74 µs at 330 Hz (0.07°).
- Dead band in degrees: DS3230MG 3 µs = 0.27° (180°) or 0.41° (270° version); MG90S 5 µs = 0.45–0.9°; SG90 10 µs = 0.9–1.8°.

**Networks:**

| actor | MAC | float32 params | weights |
|---|---|---|---|
| walk 49-256-128-64-10 | 54,144 | 54,602 | 214 KB |
| walk 49-512-256-128-10 | 190,208 | 191,114 | 747 KB |
| main 53-256-128-64-10 | 55,168 | — | 218 KB |
| main 53-512-256-128-10 | 192,256 | — | 755 KB |
| getup 48-x-14 | ≈ same as walk | — | — |

- Compute at 240 MHz, 1.69 cycles/MAC (ESP-DSP optimised dot product) or 5.12 cycles/MAC (ANSI):
  - small actor: 0.38 ms optimised, 1.16 ms ANSI
  - large actor: 1.34 ms optimised, 4.06 ms ANSI
- Large actor weights streamed from PSRAM at 40–84 MB/s: 9.1–19.1 ms per inference. Small actor from PSRAM: 2.6–5.5 ms.
- Activations: 448 hidden units (small) or 896 (large) need exp() for ELU/swish. My estimate is 0.1–0.3 ms for the small net; this is not sourced and must be measured.

## F2. Repo constants tied to the 27 cm XL330 robot (file:line)

| constant | current value | location |
|---|---|---|
| SIM_DT / CTRL_DT | 0.005 / 0.02 | config:15-16 |
| ACTION_SCALE / ACTION_CLIP | 0.5 / 1.5 | config:41-42 |
| OBS_SCALES | gyro 0.25, joint_vel 0.05 | config:45 |
| OBS_NOISE | gravity 0.05, gyro 0.1, joint_pos 0.01, joint_vel 0.5 | config:46 |
| COMMAND_RANGES | vx (−0.15, 0.30), vy ±0.15, wz ±1.0 | config:52 |
| GAIT_FREQ | 2.5 | config:57 |
| SWING_HEIGHT | 0.02 | config:58 |
| FOOT_HEIGHT_SIGMA | 2e-4 | config:59 |
| AIR_TIME_TARGET | 0.15 | config:60 |
| CONTACT_EPS | 0 | config:61 |
| TRACKING_SIGMA lin / ang | 0.02 / 0.1 | config:64-65 |
| BASE_HEIGHT_TARGET | 0.115 (includes about 3 mm kp-1.1 sag) | config:66 |
| FEET_MIN_DISTANCE | 0.06 | config:67 |
| REWARD_WEIGHTS | — | config:68-84 |
| TERMINATION_MIN_HEIGHT | 0.075 | config:88 |
| RESET noise | 0.05 rad / 0.05 m/s | config:92-93 |
| HEAD_RANDOM | ranges 0.25 / 0.3 / 0.6 / 0.1 rad, tau 0.3 s | config:94-97 |
| DOMAIN_RAND | friction 0.6–1.3, link mass ±5 %, trunk added mass −0.03..+0.05 kg, COM ±5 mm, kp ±15 %, damping ±20 %, frictionloss ×0.5–1.5, armature ±20 %, push 0.25 m/s every 3–6 s, delay (0, 0) | config:100-113 |
| H_HOME / H_MIN_WALK / H_MIN_STAND / H_DEEP / Z_SIT | 0.115 / 0.085 / 0.070 / 0.065 / 0.044 | config:153-157 |
| H_RATE | 0.02 | config:158 |
| SIT_RAMP_S / STAND_RAMP_S | 2.5 / 2.5 | config:159-160 |
| CROUCH_FAMILY | table | config:173-183 |
| SEAT_DEG / SEAT_Z / tolerance | — / 0.042 / ±0.003 | config:187-190 |
| HEAD_SIT_DEG | — | config:197 |
| MAIN | vz_cap 0.08, height_sigma 0.02, standup_z 0.105, windows 3.0 / 3.5 s, moving_eps 0.05 | config:202-215 |
| GETUP | torque_limit 0.96, height_sigma (0.04, 0.015), pose_z (0.06, 0.105), composite (0.03, 0.4, 0.4), arrival_z (0.09, 0.11), success z 0.10 / gyro 0.5 / hold 1 s, midfall angular velocity 0–4 rad/s, episode 8 s | config:241-252 |
| GETUP_REWARD | height_progress 50, height_l1 −6, … | config:253-273 |
| SUPERVISOR | fall_steps 5, ready_hold 0.5 s, handoff 1.0 s | config:276-280 |
| KNEE_LIMIT / NECK_PITCH_MAX / HIP_YAW_INWARD_MAX | 65° / 36° / 8° (all from 27 cm mesh collisions) | config:283-292 |
| EXTRA_COLLISION | 8 boxes fitted to old meshes | config:300-309 |
| POSE_BANK | settle 2.0 s, drop 0.02–0.10 m | config:312-322 |

Other places that hard-code the old robot:
- walk "moving" threshold `0.05**2`: task_math:374
- `torques` term on raw N.m: task_math:395/447
- scalar torque limit for saturation: task_math:509/654
- `sts_act` = "14 XL330" used for kp DR: env_mjx:324, env_cpu:199
- substep loop with no ctrl shaping: env_mjx:738 (`fori_loop`), env_cpu:678 (`mj_step(m, d, nsub)`)
- torque = actuator_force only: env_mjx:188, env_cpu:533
- MJX actor 512-256-128 swish: train_mjx:47; getup actor (512, 256, 128): tasks.py:126; CPU trainer [256, 128, 64] elu: config:131
- play TIP_W 10 rad/s: play:68; demo commands 0.15 / 0.5 / 0.08 m/s or rad/s and h 0.095 / 0.075: play:42-63; TORQUE_LIMIT: play:71
- eval criteria A2 0.01 m, vx 0.15, A3 windows, A4_S 6 s, head force 20 N: eval_tasks:61-66
- TASK.md says "14 STS3215 actuators"; it is stale.

## F3. Sourced numbers

**DS3230MG:**
- Direnc: 27 / 32 kg·cm at 5 / 6.8 V, 0.16 / 0.12 s/60°, "working frequency 50–330 Hz", 65 g — https://www.direnc.net/ds3230mg-su-gecirmez-servo-motor-en
- servodatabase: 30 kg·cm at 6.8 V, **0.18 s/60°**, dead band 3 µs, 500–2500 µs, stall current 3.5–3.8 A, 4.8–6.8 V — https://servodatabase.com/servo/dsservo/ds3230-mg (seen via a search snippet; the page returned 403)
- aifitlab: dead band 3 µs, 0.18 s/60°, 3.8 A — https://aifitlab.com/products/dsservo-ds3230-mg-servo-motor (search snippet)
- The parts-research values 3.06 N.m / 8.3 rad/s at 6.5 V use the Direnc speed.

**ES3054:** 3.5 kg·cm (0.343 N.m) and 0.13 s/60° at 6 V; dead band not published — https://www.readymaderc.com/products/details/emax-es3054-17g-metal-gear-digital-servo

**MG90S:** 2.2 kg·cm at 6 V, 0.08 s/60°, dead band 5 µs — https://components101.com/motors/mg90s-metal-gear-servo-motor. One shop says 1 µs: https://coolcomponents.co.uk/products/mg90s-analog-metal-gear-servo

**Analog servos:** their drive power depends on the pulse rate (lower rate means lower duty and lower torque) — https://www.pololu.com/blog/17/servo-control-interface-in-detail

**ESP32-S3 datasheet v2.2** (https://documentation.espressif.com/esp32-s3_datasheet_en.pdf):
- 240 MHz dual LX7, single-precision FPU, 512 KB SRAM
- ADC 12-bit, ≤ 100 kSPS, DNL ±4 LSB, INL ±8 LSB
- total error after calibration: ATTEN2 (0–1600 mV) ±10 mV; ATTEN3 (0–2900 mV) ±50 mV

**ESP32-S3 PIE SIMD:** 128-bit QR registers with 8/16/32-bit formats; no float32 SIMD is mentioned — https://developer.espressif.com/blog/2024/12/pie-introduction/

**ESP-DSP cycle counts on ESP32-S3** (https://docs.espressif.com/projects/esp-dsp/en/latest/esp-dsp-benchmarks.html):
- `dsps_dotprod_f32` N = 256: 432 cycles optimised, 1311 ANSI
- `dspm_mult_f32` 16×16×16: 6280 cycles optimised, 56,915 ANSI

**ESP-NN:** int8 kernels; fully-connected 271×3 takes 8439 → 1543 ticks on ESP32-S3 — https://github.com/espressif/esp-nn

**PSRAM bandwidth** (forum measurements, not official): about 84 MB/s once accesses exceed the cache, about 40 MB/s on an N16R8 — https://esp32.com/viewtopic.php?p=112250 ; 26.8 MB/s external→external — https://esp32.com/viewtopic.php?p=123419

**MJX-JAX feature parity** (https://mujoco.readthedocs.io/en/stable/mjx.html):
- dynamics NONE / INTEGRATOR / FILTER / FILTEREXACT / MUSCLE
- gain FIXED / AFFINE / MUSCLE; bias NONE / AFFINE / MUSCLE
- no USER callbacks
- LIMIT_JOINT and FRICTION_DOF supported; IMPLICITFAST supported

**Open Duck Mini v2 playground:** action_scale 0.25, motor-velocity clamp on targets 5.24 rad/s, action delay 0–3 steps, IMU delay 0–3 steps — https://raw.githubusercontent.com/apirrone/Open_Duck_Playground/main/playground/open_duck_mini_v2/joystick.py

**MPU-6500:** gyro noise 0.01 dps/√Hz, zero-rate offset ±5 dps, accelerometer 300 µg/√Hz, offset ±60 mg — https://universal-solder.ca/?p=81415

**Froude dynamic similarity:** Alexander & Jayes 1983 — https://doi.org/10.1111/j.1469-7998.1983.tb04266.x

**PCA9685:** 12-bit PWM — https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf (4.9 µs/step at 50 Hz per parts-research.md)

## Design inputs
## D1. Scaling rules (s = 1.282; κ = speed reduction, baseline κ = 0.72, final κ from the bench speed)

Let λ_L = s, λ_t = √s/κ = 1.57, λ_v = √s·κ = 0.815, λ_ω = κ/√s = 0.636. Sit/stand has speed margin, so it uses κ = 1 (λ_t = 1.13).

**Heights**
- Derive all heights from the new CAD keyframe, not from s.
- H_HOME = trunk-origin z at HOME minus ≤ 1 mm. Do not copy the old 3 mm sag: stiff servos sag < 1°.
- Keep the old ratios as starting points: H_MIN_WALK 0.739·H_HOME, H_MIN_STAND 0.609, H_DEEP 0.565, standup_z 0.913, GETUP success z 0.870, TERMINATION_MIN_HEIGHT 0.652, pose_z (0.52, 0.91), arrival_z (0.78, 0.96). At s = 1.282 that gives 0.147 / 0.109 / 0.090 / 0.083 / 0.135 / 0.128 / 0.096.
- Z_SIT and SEAT_Z = trunk-box bottom contact height from the new geometry. SEAT_Z_TOL = 0.004.

**Posture tables**
- Regenerate CROUCH_FAMILY, SEAT_DEG, HEAD_SIT_DEG and HEAD_HOME with a repo tool (a port of phys-sit family3.py). Constraints: COM over the soles, feet flat, knee ≤ new limit − 5°, and the neck static torque minimised (≤ 0.3 × ES3054 stall in every crouch row).
- Joint angles carry over only if the leg proportions are similar; DS3230MG bodies change the thigh/shin ratio.

**Timing**
- H_RATE 0.02 m/s (keep; 0.023 at Froude).
- SIT_RAMP_S and STAND_RAMP_S 2.5 → 3.0 s; standup and sitdown windows 3.4 / 4.0 s.
- GETUP_EPISODE_S 8 → 12 s; eval A4_S 6 → 10 s.
- SUPERVISOR fall_steps 5 → 6, ready_hold 0.6 s.
- COMMAND_RESAMPLE_S and EPISODE_LENGTH_S unchanged.

**Gait**
- GAIT_FREQ = 2.21·κ: 1.6 Hz baseline; sweep {1.3, 1.6, 1.8}.
- AIR_TIME_TARGET = 0.375/GAIT_FREQ (0.234 s at 1.6 Hz).
- SWING_HEIGHT 0.02·s = 0.026 m; FOOT_HEIGHT_SIGMA ×s² = 3.3e-4.
- FEET_MIN_DISTANCE = 0.72 × the new HOME lateral foot distance (old 0.06 / 0.083).

**Commands** (×λ_v and ×λ_ω): vx (−0.12, 0.24), vy ±0.12, wz ±0.6 (also limited by the weak ES3054 hip_yaw). moving_eps 0.04. Make the hard-coded 0.05 at task_math:374 a config constant.

**Action**
- ACTION_SCALE 0.5 → 0.3 rad. With a stiff P loop the action is effectively a position command; Open Duck Mini uses 0.25.
- ACTION_CLIP 1.5 (keep).
- Add a per-joint target slew limit of 0.8·ω0_j·CTRL_DT (DS3230MG 0.13 rad/step, MG90S head 2–3 rad/s). The limit also keeps the commanded head angle a valid observation.

**Rewards**
- Quadratic penalty in x: w/λ_x². Exp kernel defined on squared error: σ·λ_x². Linear term: w/λ_x.
- tracking σ_lin 0.0133, σ_ang 0.040; lin_vel_z −3.0; ang_vel_xy −0.12; base_height −18; joint_acc −1.5e-6; feet_slip −0.75; feet_distance −78. orientation, hip_pose, alive, feet_phase, feet_air_time and action_rate keep their weights.
- torques: switch to Σ(τ_net,j / τs,j)² at w = −2e-3 (old equivalent −9.2e-4; doubled for knee thermal), where τ_net = actuator_force − b_j·qd_j (true motor torque, proportional to current).
- MAIN: height_sigma 0.0256, height_l1 −7.8, vz_cap 0.09 m/s with w −17.7.
- GETUP: height_sigma (0.051, 0.019), height_l1 −4.7, height_progress 39, composite dz 0.038, joint_vel −2.5e-4, arrival_damping −0.12, midfall angular velocity (0, 3.5), success gyro 0.44, hold 1.13 s, torque_sat = Σ max(|τ_j|/τs_j − 0.9, 0) replacing the scalar 0.96.

**Domain randomisation** (before sysID → after sysID)
- kp ×U(0.5, 2.0) log-uniform → (0.8, 1.25)
- forcerange ×U(0.7, 1.05): new; covers clones, rail sag and heating
- damping (0.7, 1.4)
- armature (0.5, 2.0) → (0.8, 1.25)
- frictionloss (0.5, 2.0)
- dead band ×U(0.5, 2.0)
- link mass (0.9, 1.1)
- trunk added mass (−0.07, +0.12) kg
- trunk COM ±10 mm
- friction (0.6, 1.3) (keep)
- push 0.20 m/s with a curriculum to 0.3, every 3–6 s
- pot bias U(±0.02 rad) per joint per episode
- IMU mount misalignment ±2°, gyro bias ±0.01 rad/s
- actuation delay U{0..4} substeps at a 50 Hz PCA9685 frame, {0, 1} at 330 Hz

**Other**
- HEAD_RANDOM ranges: neck 0.15, head_pitch 0.2, head_yaw 0.5, head_roll 0.08 rad; filter_tau 0.5 s.
- RESET_BASE_VEL_NOISE 0.057.
- POSE_BANK settle 2.5 s, drop (0.026, 0.13).
- CONTACT_EPS 0 (keep). OBS_CLIP 100 (keep). OBS_SCALES: keep (normalisation is on); optionally joint_vel 0.08.
- Joint caps and EXTRA_COLLISION: re-measure on the new meshes with sim/collision_check.py and the extra_boxes method, and also clamp to servo travel (180° vs 270° DS3230MG version).

## D2. Actuator model (MuJoCo / MJX)

**Per-class `position` actuators.** Classes are ds3230 (hip_roll, hip_pitch, knee, ankle), es3054 (hip_yaw, neck_pitch), mg90s (head_pitch, head_yaw, head_roll) and sg90 (mouth). Put the parameters in a new rl/actuators.py applied in robot_model.build_spec via MjSpec, so sysID updates do not need a CAD rebuild. cad/openduck.py and export.py get the per-joint class map only for documentation and URDF effort limits.

| class | forcerange (N.m) | joint damping b (N.m·s/rad) | kp nominal, P band 3° (N.m/rad) |
|---|---|---|---|
| ds3230 | ±3.06 at 6.5 V | 0.369 | 58 |
| es3054 | ±0.343 | 0.043 | 6.6 |
| mg90s | ±0.216 | 0.0165 | 4.1 |
| sg90 | ±0.18 | 0.017 | 3.4 |

- forcerange = stall torque at the rail voltage.
- b = back-EMF damping = τs/ω0 = Kt·Ke/R, independent of voltage. Rail-voltage DR therefore scales forcerange only, and ω0 follows automatically.
- kp = τs / P band. The P band (error at 100 % duty) is unpublished. Use 3° as nominal with DR from 1–8°. This is an assumption; my sweep shows step responses are speed-limited and insensitive to the band.
- Armature = reflected rotor inertia; unpublished. Estimate for DS3230MG: 2e-3 to 1e-2 kg·m², assuming J_rotor 0.1–0.3 g·cm² and gear ratio 150–300. Use about 0.15× that for ES3054 and 0.05× for MG90S.
- frictionloss: start at 3 % of stall (assumption); measure with a backdrive test.
- SIM_DT stays 5 ms. It was measured stable and equal to 2 ms up to kp 175 N.m/rad with armature ≥ 2e-3. Keep `physics_step` c_semantics (forcelimited kv skip).

**Dead band.** MJX-JAX has no USER gain/bias, so apply it as per-substep ctrl shaping inside the substep loop:
- e = ctrl − q; e' = sign(e)·max(|e| − db/2, 0); data.ctrl = q + e'.
- Add PCA9685 quantisation of ctrl to 4.88 µs (50 Hz) or 0.74 µs (330 Hz) steps in degrees.
- db: DS3230MG 0.27° (180° version) or 0.41° (270°); MG90S 0.45–0.9°; SG90 0.9–1.8°.
- MJX: change the `fori_loop` body at env_mjx:738. CPU: replace `mj_step(m, d, nsub)` (env_cpu:678) with a Python loop of 4 single steps. This costs CPU speed, but CPU is used for parity, play and eval only.

**Backlash** (optional, only if sysID shows > 0.5°): a passive series hinge `<j>_play` between a "rotor" body and the link, range ±b/2, small damping. The actuator becomes a `motor` (gear 1) with τ = clip(kp·deadzone(target − q_link), ±τs) computed in env code, and back-EMF damping on the rotor dof. This is +10 DoF and +10 LIMIT_JOINT rows; budget +30–60 % MJX step time (estimate). The cheap default is dead band plus DR on the band width.

**Optional extras:** `timeconst` (FILTEREXACT is MJX-supported) for the servo's internal command filtering; reduced damping inside the dead band if the H-bridge coasts rather than brakes.

**Bench sysID** (B2 step 0, potentiometer logged at ≥ 500 Hz):
- no-load speed at 6.0 / 6.5 / 6.8 V → ω0, b
- stall with a lever and scale → τs
- hanging-mass deflection versus command → kp and dead band
- chirp → armature
- backdrive torque → frictionloss
- Fit with a single-joint MuJoCo model (BAM-like) in a new tools/sysid/.

## D3. Observation model (the sim must produce exactly what the firmware produces)

Add `s["q_obs"]` and `s["qd_obs"]` (actor). Rewards and critic keep the true q/qd. The tasks.obs_layout is unchanged in size (49 / 53 / 48).

**Leg joints (potentiometer):**
- q_obs = q(substep N−1) + bias_j (episode) + N(0, 0.003 rad), quantised to 0.001 rad.
- qd_obs = first-order IIR (α = 0.47, 20 Hz cutoff at 200 Hz) of the 200 Hz difference of q_obs. Carry the filter state through the substep loop; the firmware runs the same filter at 200 Hz. This yields σ_qd ≈ 0.47 rad/s (computed), matching today's OBS_NOISE of 0.5.
- Hardware: use ADC ATTEN2 (0–1.6 V, ±10 mV) with a wiper divider, not ATTEN3 (±50 mV), plus 16× oversampling and a 3–5-point per-joint calibration table.
- OBS_NOISE.joint_pos becomes per-joint: legs 0.005 uniform plus bias; head 0.

**Head joints:** q_obs = commanded ctrl after clip and slew limit (walk: default + random offset; main: + script offset; getup: policy targets). qd_obs = Δctrl/CTRL_DT. No noise.

**IMU:** gravity from quat rotated by the per-episode mount error, noise 0.03; gyro + bias + 0.1 noise. The MPU-6500 noise floor is negligible, but its ±5 dps zero-rate offset needs boot calibration.

**Latency:** actuation delay at substep resolution (D1 domain randomisation). Option: route one spare PCA9685 channel to an ESP32 GPIO interrupt to sync the 50 Hz loop to the PWM frame, which makes the latency deterministic.

Export every observation-model constant into policy_meta, so the ESP32 runtime has a single source of truth.

## D4. Policy network for the ESP32-S3

- Deploy actor 256-128-64 for walk, main and getup, with float32 weights in internal SRAM (214–218 KB) and normalisation folded in. Expected about 0.4 ms (ESP-DSP-style dot products) to 1.2 ms (plain C), plus activations.
- Keep the critic at 512-256-128 (training only).
- Set train_mjx DEFAULT policy hidden to 256,128,64 and tasks.GETUP.actor_hidden to (256, 128, 64).
- Two policies do not both fit in SRAM next to IDF. Keep the active one in SRAM and the other in PSRAM, and copy on handoff (2.6–5.5 ms, one cycle).
- Do not deploy 512-256-128: the 747 KB streamed from PSRAM costs 9–19 ms per step. If needed, use int8 via ESP-NN.
- New rl/export_c.py writes weights.h (row-major, 16-byte aligned) and obs_layout.h, plus a float32 numpy reference and a golden-vector parity test (< 1e-5).

## D5. File-by-file change list (effort in person-days; * = lead-owned shared file per TASK.md)

| file | change | effort (d) |
|---|---|---|
| rl/config.py* | every D1 constant; new DOMAIN_RAND keys; head-random limits; servo-free values (no 0.96) | 1.5 |
| rl/actuators.py (new, shared) | class table, joint→class map, dead band, PWM frequency per board, delays, DR ranges, apply_to_spec(); sysID values land here | 1.5 |
| rl/robot_model.py* | apply actuator classes; caps from new CAD; optional play joints; RobotIndex gains per-actuator forcerange, damping and class index; HEAD/EXTRA names check | 1.5 |
| rl/task_math.py* | observation uses q_obs/qd_obs; per-joint noise_vector; normalised torque terms using τ_net; per-joint torque_sat; config-driven moving threshold | 1.5 |
| rl/sensors.py (new, xp-generic) | potentiometer, IMU and commanded-head models, IIR, quantisation (numpy/jax like task_math) | 1.5 |
| rl/tasks.py* | getup actor (256, 128, 64); episode lengths; new obs keys documented | 0.5 |
| rl/env_mjx.py | substep loop: delay, dead band, quantisation, sensor history and filter; DR forcerange/damping per class (replace `sts_act`); τ_net extraction; info keys and _KEEP_INFO; parity | 4 |
| rl/env_cpu.py | mirror of env_mjx (4-substep Python loop, same RNG-stream rules) | 3 |
| rl/pose_bank.py* | new actuator model in settle; scaled drop/settle; regenerate and re-validate the head-only rejection | 0.5 |
| rl/policy_meta.py* | add actuator classes, observation-model params, delays, slew limits, network arch, PWM frequency | 0.5 |
| rl/train_mjx.py / train.py | default actor 256-128-64; call export_c | 0.5 |
| rl/export_c.py (new) | C-array export and parity test | 2 |
| rl/play.py | TIP_W 10→8.8; scaled demo commands and heights; per-joint saturation metric | 0.5 |
| rl/eval_tasks.py | A2 0.013 m; vx 0.12; A3 windows ×1.13; A4_S 10 s; head force 20→49 N (×mass); per-joint saturation | 1 |
| tools/crouch_family.py (new) | regenerate F, SEAT, HEAD_SIT and head poses from the new MJCF (replaces the scratch family3.py) | 2 |
| tools/sysid/ (new) | bench log → parameter fit | 3–5 |
| sim/collision_check.py + EXTRA_COLLISION | re-fit on new meshes; new caps | 1.5 |
| cad/openduck.py, cad/export.py | per-joint class names and placeholder parameters; keep FOOT, IMU, trunk and head geom and site names | 1 (CAD area) |
| rl/tests/* | update hard-coded layout and constants; new tests for dead band, delay, observation model and MJX↔C parity | 3 |
| rl/TASK.md, spec.md | contract: q_obs, substep shaping, τ_net, per-joint limits, C export; new acceptance numbers | 1 |

**Total:** about 32 d (range 27–38) before training. Training plan:
- walk: 3–5 runs sweeping GAIT_FREQ × command scale
- main: 1–2 runs
- getup: exploratory

## D6. Task verdicts

- **main sit/stand/crouch:** go first. Margins are 1.8–3.9× at 1× need and the speed is sufficient. Watch neck heating in crouch.
- **walk:** go, at κ 0.6–0.72 (1.3–1.6 Hz, vx ≤ 0.20–0.25 m/s, u 0.62–0.72). Use κ ≈ 0.5 if the bench confirms 0.18 s/60°.
- **getup:** defer. The current strategy needs about 2× neck and 3× head torque and 2× speed. Options:
  - (a) leg-only RL get-up, starting from front falls
  - (b) neck_pitch → DS3230MG: 4.6× margin; 10 of Direnc's 15 units are then used; +45 g in the head
  - (c) accept assisted recovery

## Open questions
- DS3230MG no-load speed: Direnc lists 0.12 s/60° at 6.8 V, while servodatabase and aifitlab list 0.18 s/60°. Is it a 180° or 270° version? These set κ, GAIT_FREQ, the command ranges, and whether walking is feasible. Measure on the bench before the first training run.
- Does the DS3230MG accept 200–333 Hz frames (Direnc says 50–330 Hz)? Does the ES3054? This decides the PCA9685 split (leg board fast, head board at 50 Hz for the analog MG90S/SG90) and the delay DR (0–4 vs 0–1 substeps).
- What are the internal P band, armature, frictionloss and in-dead-band braking or coasting of each servo class? All are unpublished; they need the pot-logged sysID in step 0.
- What is the potentiometer wiper voltage span and supply inside the DS3230MG and ES3054? This sets the divider, the ADC attenuation (ATTEN2 is recommended) and the real quantisation and noise.
- What are the new CAD's HOME height, leg segment ratio, foot box, trunk box and head subtree mass (assumed 0.370 kg)? Every height and the crouch table derive from them.
- Get-up path: leg-only RL get-up, a stronger neck_pitch servo (DS3230MG), or drop get-up for v1?
- Should the CPU (torch) backend keep exact parity with the new substep-level actuator and sensor models? This costs CPU-env speed, which is used only for play and eval.
- Who owns the edits to the lead-owned shared files (config, robot_model, task_math, tasks, pose_bank, policy_meta)? TASK.md forbids worker edits.

## Risks
- Speed-limited knee: at Froude-similar speed u ≈ 0.96, or 1.41 with the pessimistic speed spec. Even at κ 0.72, u stays between 0.72 and 1.02 across the speed specs, so there is no 2× margin unless κ ≤ 0.45 (about 1 Hz, vx ≤ 0.15 m/s).
- Knee thermal: walk RMS is about 0.94 N.m against DS3230MG stall/3 = 1.02 N.m. Long walks may overheat cheap metal-gear servos. The normalised τ_net penalty and duty monitoring on the robot are needed.
- Neck (ES3054) holds 0.56–0.67 of stall statically in crouches with the head at HOME. The head poses must be redesigned or the servo will cook during long crouches.
- Rail sag: each XL4015 5 A rail feeds two DS3230MG with 3.5–3.8 A stall current each. Peak demand exceeds the rail limit, and in sim this is only covered by forcerange DR (0.7×).
- A policy trained on soft kp 1.1 targets does not transfer. Stiff P loops turn actions into position commands, and the old HOME/height constants embed a 3 mm sag. Everything must be retrained; no warm start.
- The commanded-angle head observation is only valid while the head servos are not saturated. Random-head DR at today's ranges would drive the MG90S to 1.5× stall, so slew and range limits are mandatory.
- The pot mod is unverified (does the DS3230MG really have a pot, wiper range, noise from the motor PWM). If it fails, the build falls back to B1 with no leg feedback, which needs a different observation design (action history only).
- ESP32-S3 timing: the 512-wide actor does not fit SRAM (9–19 ms from PSRAM). The PSRAM bandwidth numbers are forum measurements, and the exp/activation cost is unmeasured. Benchmark on the board before freezing the network.
- Backlash modelled as a play joint adds about 30–60 % MJX cost (estimate). Dead-band shaping requires a Python substep loop in env_cpu, which is slower for eval and play.
- Pose banks, EXTRA_COLLISION, joint caps and the crouch table all depend on the new meshes. Tests that pin old numbers (test_task_math crouch validation, layout tests) will fail until regenerated.
