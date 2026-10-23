# OpenDuck tasks — backend contract

This file is the contract that every backend (CPU: MuJoCo C + PyTorch PPO, GPU: MJX + Brax PPO)
must implement. Constants live in `rl/config.py`; observation / reward / termination math lives
ONLY in `rl/task_math.py` (backend-agnostic, called with `xp=numpy` or `xp=jax.numpy`).
Backends must not re-implement that math — they gather raw state into the `s` dict documented
at the top of `rl/task_math.py` and call those functions.

Three tasks exist (`rl/tasks.py`, `get_task(name)` → `TaskSpec`): `walk` (the original joystick
walking task, the default; everything up to "Tasks main and getup" below describes it and is
unchanged), `main` (walk + body-height command + sit/stand) and `getup` (stand up after a fall).
New behaviour is opt-in through `task="main" | "getup"`; with `task="walk"` (default) every number,
layout, RNG stream and file output must stay bit-identical to before.

`rl/config.py`, `rl/robot_model.py`, `rl/task_math.py`, `rl/tasks.py`, `rl/pose_bank.py`,
`rl/policy_meta.py` are shared and owned by the lead.
Do not edit them; if you believe one is wrong, report it precisely in your final answer.

Two robot profiles exist (OpenDuck-TR): `xl330_27` (the 27 cm XL330 robot, the default, everything below as
written) and `tr35_b2` (the 35 cm PWM-servo robot). See "Robot profiles" at the end of this file; with the default
profile every number, layout, RNG stream, reward value, meta field and output file is unchanged.

## Model

`rl.robot_model.build_model()` builds the training model from `build/sim/scene.xml`
(visual meshes stripped, feet collide with each other and the floor, extra sensors, `SIM_DT`).
`rl.robot_model.RobotIndex(model)` holds all addresses (qpos/dof/actuator/sensor slices, foot geoms,
floor geom, trunk body) and the default pose (keyframe `stand`).

Actuator order (ctrl): 10 leg joints (`LEG_JOINTS`), 4 head joints (`HEAD_JOINTS`), then `OTHER_JOINTS`
(the beak, `mouth`). `qpos` = 7 (free joint: xyz + quat wxyz) + 15 joints in the same order.

## State extraction (`s` dict)

| key | how to get it |
|---|---|
| quat, gyro, linvel | sensordata slices `idx.s_quat`, `idx.s_gyro`, `idx.s_linvel` |
| trunk_z | `xpos[idx.trunk_body, 2]` |
| q, qd | `qpos[idx.obs_qpos_adr]`, `qvel[idx.obs_dof_adr]` |
| qdd_legs | `qacc[idx.leg_dof_adr]` |
| torque | `actuator_force[idx.obs_act_id]` |
| foot_pos, foot_vel | sensordata `idx.s_foot_pos[i]`, `idx.s_foot_vel[i]`, stacked → (N,2,3) |
| contact | foot i touches floor ⇔ ∃ contact between `idx.foot_geom[i]` and `idx.floor_geom` with `dist < CONTACT_EPS` |
| friction | sampled DR foot friction (1.0 nominal when DR is off) |
| default_q | `idx.default_obs_q` |

All values are read after the last physics substep of the control step (MuJoCo C: after
`mj_step`, sensors/contacts in `data` correspond to the state before that last substep's
integration — this is acceptable and identical in MJX).

## Control step (exact order)

Per environment, with policy output `action` (10,):

1. `targets, a = task_math.action_to_targets(xp, action, idx.default_action_q)` (`a` = clipped action).
   With action delay `k` (DR, default 0) use the `a` from `k` steps ago (keep a ring buffer).
2. Head: `head_offset ← head_offset + (head_target − head_offset) · CTRL_DT / HEAD_RANDOM.filter_tau`;
   `ctrl[head] = default_head + head_offset`. Other actuators (the beak): keyframe ctrl. `head_target` is resampled every
   `U(resample_s)` seconds: with prob `HEAD_RANDOM.prob` it is `U(−range, range)` per joint, else 0.
   When head randomization is disabled, `head_offset ≡ 0`.
3. `ctrl[legs] = targets`; clip all ctrl to `actuator_ctrlrange`.
4. Push (DR): when the per-env push timer (`U(push_interval_s)`) expires, add `U(−push_vel, push_vel)`
   to `qvel[0]` and `qvel[1]` (world xy of the free joint) and resample the timer.
5. Step physics `N_SUBSTEPS` times.
6. `phase ← mod(phase + GAIT_FREQ · CTRL_DT, 1)`; `step_count += 1`.
7. Gather `s` (with `action = a`, `last_action = a_prev`, current `command`, new `phase`).
8. `air_time_new, first_contact = update_air_time(xp, air_time, contact, CTRL_DT)`;
   put `s["air_time"] = air_time_new`, `s["first_contact"] = first_contact`.
9. `terms = reward_terms(xp, s)`, `term = terminated(xp, s)`, `timeout = step_count ≥ MAX_EPISODE_STEPS`,
   `reward = total_reward(xp, terms, term)`.
10. `air_time = reset_air_time(xp, air_time_new, contact)`; `a_prev = a`.
11. Command timer: if `COMMAND_RESAMPLE_S` elapsed, resample the command (see below).
12. `done = term or timeout`. Done envs are reset immediately (see Reset); the observation returned
    for them is the post-reset observation. Report `time_outs = timeout and not term` separately.
13. Actor obs = `observation(xp, s')` + `noise_vector(xp) · U(−1, 1)` (noise only if enabled), where
    `s'` is `s` with `last_action = a` (the action just applied), the possibly-resampled command, and
    for reset envs their fresh state. Critic obs = `critic_observation(xp, s')` without noise.

## Commands

`vx ~ U(COMMAND_RANGES.vx)`, `vy ~ U(...vy)`, `wz ~ U(...wz)`; with probability `COMMAND_ZERO_PROB`
the whole command is zero. Sampled at reset and every `COMMAND_RESAMPLE_S` seconds.

## Reset

- `qpos = idx.init_qpos`; the 14 obs joints get `+U(±RESET_JOINT_NOISE)`; base yaw `U(−π, π)`
  (rotate the free-joint quaternion about world z); base xy = 0; base z from keyframe.
- `qvel = 0`, except base linear x,y `U(±RESET_BASE_VEL_NOISE)`.
- `ctrl = idx.init_ctrl`, `phase ~ U[0,1)`, `a_prev = 0`, `air_time = 0`, `step_count = 0`,
  new command, head offset/target 0 with a fresh resample timer, fresh push timer.
- Domain randomization (if enabled), sampled from the NOMINAL model values each episode:
  - friction: one `f ~ U(friction)`; set `geom_friction[:,0]` of both foot geoms AND the floor to `f`
    (MuJoCo combines by max, so the floor must change too).
  - every body except world: `body_mass *= U(link_mass_scale)`, `body_inertia` scaled by the same factor.
  - trunk: `body_mass += U(trunk_added_mass)` (inertia scaled proportionally); `body_ipos += U(±trunk_com_offset)`.
  - each of the 14 policy/head servos (`idx.obs_act_id`; xl330_27: the XL330 position actuators):
    `kp' = kp · U(kp_scale)`; set `gainprm[0] = kp'`, `biasprm[1] = −kp'`. tr35_b2: these generic actuator factors
    are 1 (`actuators.DR_OVERRIDES`) and the per-class servo DR of `rl/actuators.py` applies instead (see
    "Robot profiles").
  - each actuated dof independently: `dof_damping`, `dof_frictionloss`, `dof_armature` scaled by
    `U(damping_scale)`, `U(frictionloss_scale)`, `U(armature_scale)`.
  - action delay `k ~ U{action_delay_steps}`.
  - CPU backend: call `mujoco.mj_setConst` after changing model fields.

## Observation / action layout (also exported with the policy)

Actor obs (49): `[gravity(3), gyro·0.25(3), q−q0(14), qd·0.05(14), last_action(10), command(3), cos/sin phase(2)]`.
Critic obs (58): actor obs (noise-free) + `[linvel(3), trunk_z(1), contact(2), foot_z(2), friction(1)]`.
Action (10): raw policy output; runtime does `target = q0_legs + ACTION_SCALE · clip(a, ±ACTION_CLIP)`.

## Policy export format (both trainers)

`<run>/policy.onnx`: input `obs` float32 `[batch, 49]` → output `action` float32 `[batch, 10]`
(deterministic mean action, observation normalization baked in, no clipping).
`<run>/policy_meta.json`: `{"obs_dim", "action_dim", "ctrl_dt", "action_scale", "action_clip",
"default_leg_q", "leg_joints", "obs_joints", "default_obs_q", "gait_freq", "obs_scales", "backend", "steps"}`.

## CPU backend API (`rl/env_cpu.py`)

```python
env = VecEnv(num_envs, num_workers=None, seed=0, randomize=True, noise=True, head_random=True,
             push=True, fixed_command=None)
obs, critic_obs = env.reset()                    # float32 (N,49), (N,58)
obs, critic_obs, rew, done, info = env.step(actions)   # actions float32 (N,10)
# rew float32 (N,), done bool (N,), info["time_outs"] bool (N,)
# info["episode"]: dict of float means over episodes finished this step (keys "ep_len" and
#                  "rew_<term>" = per-episode sum of w·r·CTRL_DT), only present if any finished.
env.set_commands(cmd)       # (N,3) fixed commands, disables resampling (for play/eval)
env.close()
```
`num_workers=0` runs everything in-process; then `env.mj_data(i)` / `env.mj_model(i)` give the
`MjData`/`MjModel` of env `i` (for rendering). `num_workers=None` → `os.cpu_count()`.
Throughput target on this 4-core machine: ≥ 12 000 env-steps/s with 512 envs.

Additions for main / getup (all optional; defaults keep the walk behaviour above):
```python
env = VecEnv(num_envs, ..., task="walk"|"main"|"getup", terminate=True, reset_probs=None,
             max_episode_steps=None, progress=0.0, episode_details=False, bank_n=None, model_variant=None)
# terminate=False: no fall termination (only bad physics / timeout), used by play
# reset_probs: reset-mix override, e.g. {"home": 1.0} (main), {"fallen": 1.0} (getup)
# progress: getup curriculum p (late-term weights; 1.0 in evaluation); env.set_progress(p)
# episode_details=True: info["episodes"] = {"env": ids, <episode_columns(task)>: (k,) arrays}
# bank_n: pose-bank size (int or {kind: n}); banks are built once in the parent and sent to workers
# model_variant: physics-model override (play runs the main policy on the getup model for handoff)
obs, critic = env.set_commands(cmd, refresh_obs=True)   # main: (N,3) or (N,5); getup: ValueError
obs, critic = env.observe()                             # recompute obs from the current state
s = env.task_state()                                    # num_workers=0: task_math "s" dict + task state
env.set_state(i, qpos, qvel, ctrl=None, warmstart=None) # handoff: last_action 0, alpha 0, h_ref H_HOME
```
`env_cpu.episode_columns(task)`: `ep_len`, `rew_<term>`, `spec.episode_metrics`, and for main/getup
`reset_kind`, `bank_label` (`pose_bank.LABELS` index, −1 without a bank).

## GPU backend (`rl/env_mjx.py`, `rl/train_mjx.py`)

Same semantics in JAX (MuJoCo Playground style env: `reset(rng) -> State`, `step(state, action) -> State`,
`obs = {"state": actor_obs, "privileged_state": critic_obs}`), domain randomization via a vmapped
`randomization_fn` over model fields, trained with Brax PPO (asymmetric actor-critic). Exports the same
`policy.onnx` / `policy_meta.json`.


---

# Tasks `main` and `getup`

Design: `docs/workflow/sit-stand-getup/spec.md`. Everything below is normative for both backends;
"recommended" marks implementation advice that may differ between backends without changing
semantics. All functions named `T.*` are in `rl/task_math.py`, `TK.*` in `rl/tasks.py`,
`PB.*` in `rl/pose_bank.py`. `spec = TK.get_task(task)`.

## TaskSpec (rl/tasks.py)

| field | walk | main | getup |
|---|---|---|---|
| `model_variant` | `walk` | `main` | `getup` |
| `action_joints` / `num_actions` | `LEG_JOINTS` / 10 | `LEG_JOINTS` / 10 | `LEG_JOINTS + HEAD_JOINTS` / 14 |
| `action_mode` | `home` | `ref` | `home_clip` |
| `action_scale`, `action_clip` | 0.5, 1.5 | 0.5, 1.5 | `None` (per joint, see below), 1.0 |
| `command_names` | vx, vy, wz | vx, vy, wz, h, sit | — |
| `num_obs` / `num_critic_obs` | 49 / 58 | 53 / 64 | 48 / 57 |
| `episode_length_s` / `max_episode_steps` | 20 / 1000 | 20 / 1000 | 8 / 400 |
| `terminate_on_fall` | yes | yes (mode-aware) | no (bad physics + timeout only) |
| `reset_probs` | home 1.0 | home 0.75, seated 0.25 | fallen 0.70, midfall 0.10, stand 0.20 |
| `head_mode` | `random` | `script` | `policy` |
| `gamma`, `actor_hidden`, `critic_hidden` | `None` (trainer defaults unchanged) | 0.97, None, None | 0.99, (512,256,128), (512,256,128) |
| `init_noise_std`, `solver_iterations` (MJX trainer defaults) | `None` (1.0, 4) | None, 8 | 0.5, 8 |
| `reward_weights` / `reward_late` / `late_p` | `C.REWARD_WEIGHTS` / {} / None | `C.MAIN_REWARD_WEIGHTS` / {} / None | `C.GETUP_REWARD_WEIGHTS` / `C.GETUP_REWARD_LATE` / (0.5, 0.75) |
| `episode_metrics` | () | see "Episode metrics" | see "Episode metrics" |

`spec.obs_layout` / `spec.critic_layout` are `((name, size), ...)`; `spec.obs_slices[name]` and
`spec.critic_slices[name]` give the slices (e.g. main: command 44:49, posture 49:51, phase 51:53).
Critic obs is always `actor obs (noise-free) + critic_layout`. Runtimes/tools must use these slices
(or `meta["obs_layout"]`), never hard-coded offsets.

| task | actor obs | critic extras (after the actor obs) |
|---|---|---|
| walk | gravity 3, gyro·0.25 3, q−q0 14, qd·0.05 14, last_action 10, command 3, phase 2 | linvel 3, trunk_z 1, foot_contact 2, foot_z 2, friction 1 |
| main | gravity 3, gyro·0.25 3, q−q0 14, qd·0.05 14, last_action 10, command 5, posture 2, phase 2 | linvel 3, trunk_z 1, foot_contact 2, foot_z 2, friction 1, trunk_floor 1, head_floor 1 |
| getup | gravity 3, gyro·0.25 3, q−q0 14, qd·0.05 14, last_action 14 | linvel 3, trunk_z 1, foot_contact 2, trunk_floor 1, head_floor 1, friction 1 |

`q0` is always HOME (`idx.default_obs_q`), also in `main` (not `q_ref`). main obs `command` is
`T.command_obs(command)` = `[vx, vy, wz, h_n, sit]` of the *effective* command (`T.main_command`:
velocities zeroed when `sit = 1` or `h < H_MIN_WALK`, `h` clipped to `[H_MIN_STAND, H_HOME]`,
`h_n = (h − H_HOME)/H_NORM ∈ [−1, 0]`); `posture = [(h_ref − H_HOME)/H_NORM, alpha]`.
Noise: `T.noise_vector(xp, task)` (zero for last_action/command/posture/phase).

## Models (rl/robot_model.py)

`build_model(variant=spec.model_variant)` (also `build_spec(..., variant=)`; `visual=True` keeps
meshes for rendering with the same qpos layout). `walk` is byte-identical to before. `main`: knee
flexion limit 65° (`left_knee` range upper = +1.13446, `right_knee` lower = −1.13446; ctrlrange
too), neck_pitch upper limit `C.NECK_PITCH_MAX_DEG` = 36° (joint and ctrlrange; from 38.3° the
neck_splice mesh enters the torso shells, and the soft limit overshoots ~2° on impact), inward hip_yaw limit
`C.HIP_YAW_INWARD_MAX_DEG` = 8° (`left_hip_yaw` lower −8°, `right_hip_yaw` upper +8°; at 20° the shin bracket enters
the battery tray in half of the walking poses, at 10° the thigh plate enters the torso shell when walking crouched) and a site `head_center` (centre of `head_collision`) with sensor `head_vel` (framelinvel,
world frame), appended after the existing sensors. `getup` = `main` + 8 floor-only boxes
`C.EXTRA_COLLISION` (`<body>_collision` on left/right thigh, shin, hip, neck, jaw; class
`collision`: contype 0, conaffinity 1, density 0). MJX contact slots: walk/main 20, getup 52.

`RobotIndex(model, action_joints=spec.action_joints)` — new fields besides the walk ones (which
are unchanged): `action_joints`, `action_act_id` (ctrl indices of the action joints, 10 or 14),
`action_qpos_adr`, `action_obs_idx`, `default_action_q` (keyframe value by joint name; getup:
14 = `default_obs_q`), `action_lo/hi` (ctrlrange of the action joints), `trunk_geom`,
`trunk_geoms`, `head_geoms` (`head_collision`, + `jaw_collision` in getup), `head_body`,
`s_head_vel` (sensordata slice; `None` in walk), `extra_geoms`, `variant`.

Per-joint getup action scale: `S = TK.action_scale_vector(spec, idx.default_action_q,
idx.action_lo, idx.action_hi)` = `max(|hi − HOME|, |lo − HOME|)`; walk/main: `ACTION_SCALE` × ones.

## Extra state (`s` dict) for main / getup

All walk keys keep their meaning (getup does not need `phase`, `air_time`, `first_contact`, and
`command` is absent). `action`/`last_action` are `num_actions` wide.

| key | task | how to get it |
|---|---|---|
| command (5) | main | raw stored command `[vx, vy, wz, h_cmd (m), sit (0/1)]` (the backend never pre-processes it; `T.*` applies `main_command`) |
| h_ref, alpha | main | posture filter state (env state, see step 6) |
| trunk_floor | main, getup | 1 ⇔ ∃ contact between `idx.trunk_geom` and `idx.floor_geom` with `dist < CONTACT_EPS` (same rule as foot contact) |
| head_floor | main, getup | 1 ⇔ ∃ such contact for any geom in `idx.head_geoms` |
| head_vel (3) | getup | `sensordata[idx.s_head_vel]` |
| head_floor_prev, phi_up_prev, phi_h_prev, head_v2_prev | getup | env memory: `T.getup_memory(xp, s)` of the previous step (of the reset state after a reset); keys are whatever `T.getup_memory` returns |
| default_q | all | `idx.default_obs_q` (HOME, 14) |

Contact flags are read after the last substep, exactly like the foot flags (MJX: contact slots of
the geom pair; C: `data.contact`). Bad-physics envs: state NaN → 0 like walk.

## Posture filter and reference pose (main)

```
ref = T.posture_ref(xp, h_ref, alpha, idx.default_obs_q)   # q_ref (10), z_ref, pitch_ref, head_offset (4)
h_ref, alpha = T.posture_step(xp, h_ref, alpha, command)   # rate-limited, reaches targets exactly
```
`posture_step` snaps `alpha` to `sit` when `|alpha − sit| < 1e-6` (float32 accumulation).
`q_ref(H_HOME, 0) == HOME` exactly. Left-leg absolute degrees, right mirrored (`q_right = −q_left`)
— see the sign comment in `config.py`. Initial values: `h_ref = H_HOME`; `alpha = 0` (home reset)
or `1` (seated reset).

## Control step, main (exact order)

State carried per env: walk state (phase, step_count, a_prev, air_time, head offset/target/timer,
push timer, delay buffer) + `command` (5), `cmd_lock`, `seg` (segment code, float), `seg_timer` (s),
`h_ref`, `alpha`, event state `ev` (`T.main_events_init`).

1. `ref = T.posture_ref(xp, h_ref, alpha, default_obs_q)` with the CURRENT posture (the one shown in
   the last observation). `targets, a = T.action_to_targets(xp, action, ref["q_ref"])`
   (walk scale/clip). Action delay: `targets` from the delayed `a` and the current `ref["q_ref"]`.
2. Head: `ctrl[head] = default_head + head_offset + ref["head_offset"]`, where `head_offset` is the
   walk random-head filter (only if `head_random`, else 0) whose INPUT target is gated:
   `head_offset ← head_offset + (head_target · [alpha < ALPHA_STAND_MAX] − head_offset)·CTRL_DT/filter_tau`.
   `head_target`/`head_timer` are resampled exactly as in walk (same uniforms) regardless of alpha.
   Beak: keyframe ctrl.
3. `ctrl[idx.action_act_id] = targets`; clip all ctrl to `actuator_ctrlrange`.
4. Push: the timer runs and is resampled as in walk; the velocity kick is applied only if
   `alpha < ALPHA_STAND_MAX` (uniforms consumed either way).
5. Physics `N_SUBSTEPS`.
6. `phase ← mod(phase + GAIT_FREQ·CTRL_DT, 1)` (always advances); `step_count += 1`;
   `h_ref, alpha ← T.posture_step(xp, h_ref, alpha, command)` with the command active during this step.
7. Gather `s` (walk keys + `command`, the NEW `h_ref`/`alpha`, `trunk_floor`, `head_floor`).
8. Air time as walk.
9. `terms = T.reward_terms(xp, s, "main")`; `term = T.terminated(xp, s, "main")` OR bad physics
   (bad → all terms 0, as walk); `w = T.reward_weights(xp, "main")`;
   `reward = T.total_reward(xp, terms, term, w)`; per-term metric `rew_k = terms[k]·w[k]·CTRL_DT`.
10. Air time reset; `a_prev = a`; `ev, inc = T.main_events(xp, s, ev, term)` (add `inc` to the
    episode sums).
11. Segment: `seg_timer −= CTRL_DT`; if `seg_timer ≤ 0` and the command is not locked:
    `seg, command, seg_timer = T.sample_segment(xp, u, command)` with `u` = `T.NUM_SEGMENT_U` (8)
    fresh U[0,1) numbers. (The walk 6 s `COMMAND_RESAMPLE_S` resample is NOT used in main.)
    Locked commands (`fixed_command` / `set_commands`) never resample; the timer may keep running.
12. Done / reset as walk (`time_outs = timeout ∧ ¬term`).
13. Obs from `s'` = `s` with `last_action = a`, the possibly new `command`, the new `h_ref`/`alpha`
    (reset envs: their fresh state): `T.observation(xp, s', "main") + T.noise_vector(xp, "main")·U(−1, 1)`
    (noise only if enabled). Critic `T.critic_observation(xp, s', "main")` without noise.

Segments (`T.sample_segment`, `C.MAIN`): kinds `walk 0.45 / stand 0.10 / crouch 0.15 / sit 0.30`,
never `sit` after `sit` (renormalised); duration `U(3.5, 6.5)` s; walk: velocities from
`COMMAND_RANGES` (zero with `COMMAND_ZERO_PROB`), `h = H_HOME` (50 %) else `U(H_MIN_WALK, H_HOME)`;
stand: `h = H_HOME`, v = 0; crouch: `h ~ U(H_MIN_STAND, H_HOME)`, v = 0; sit: `sit = 1`, v = 0,
`h` kept from the previous command.

## Reset, main

1. DR exactly as walk. `kind = T.sample_reset_kind(xp, u_kind, "main")` (0 home, 1 seated).
2. home: the walk reset (joint noise, yaw, base xy velocity noise, keyframe ctrl).
   seated: `b = PB.seated_bank()` (built once at env construction; `PB.check_variant(b, "main")`),
   `i = T.bank_index(xp, u_i, len(b["qpos"]))`: `qpos = b.qpos[i]` with
   `qpos[3:7] = T.quat_mul(xp, T.yaw_quat(xp, yaw), b.qpos[i, 3:7])` (`yaw` = the walk reset's yaw draw,
   `U(−π, π)`), `qpos[0:2] = 0`, z from the bank, `qvel = b.qvel[i]` (zeros), `ctrl = b.ctrl[i]`.
   No joint noise, no base-velocity noise. Never simulate inside reset (only the forward pass).
3. `alpha = kind` (1 seated, 0 home), `h_ref = H_HOME`, `ev = T.main_events_init(xp, alpha)`.
4. Command: always `seg, command, seg_timer = T.sample_segment(xp, u_seg, T.reset_command(xp, alpha),
   seated=alpha)` (seated start: 50 % sit / 50 % stand); a locked command then only overrides `command`
   (3-dim input padded by `TK.pad_command`: `h = H_HOME, sit = 0`).
5. Rest as walk (phase, a_prev = 0, air_time, head/push timers, step_count = 0).
6. Obs from the forward state (contacts → trunk_floor/head_floor).

## Control step, getup (exact order)

1. `targets, a = T.action_to_targets(xp, action, idx.default_action_q, scale=S, clip=1.0,
   lo=idx.action_lo, hi=idx.action_hi)` (14 wide; delay buffer 14 wide).
2. `ctrl = idx.init_ctrl` (beak at HOME); `ctrl[idx.action_act_id] = targets` (legs + head; no head
   randomisation or script, `head_random` is ignored); clip.
3. Push as walk (if enabled).
4. Physics; `step_count += 1` (no phase, no command, no air time).
5. Gather `s` (walk raw keys + `trunk_floor`, `head_floor`, `head_vel`, the memory keys
   `head_floor_prev`, `phi_up_prev`, `phi_h_prev`, `head_v2_prev`, and `default_q`).
6. `terms = T.reward_terms(xp, s, "getup")`; `term = bad` (`T.terminated(.., "getup")` is 0);
   `w = T.reward_weights(xp, "getup", progress)`; `reward = T.total_reward(xp, terms, term, w)`.
7. Memory: store `T.getup_memory(xp, s)`; `a_prev = a`; `ev, inc = T.getup_events(xp, s, ev)`; `inc = 0` on
   bad-physics steps.
8. `done = term ∨ step_count ≥ spec.max_episode_steps (400)`; success does NOT end the episode.
9. Obs `T.observation(xp, s', "getup")` with `last_action = a` (+ `T.noise_vector(xp, "getup")·U(−1, 1)`
   if enabled); critic `T.critic_observation(xp, s', "getup")`.

`progress` p ∈ [0, 1] = (env steps done so far in this run) / (planned total), the same for every env
(MJX: a global counter `info["gstep"]` kept across episodes; CPU: set by the trainer). Late terms
(`head_impact`, `action_rate`, `joint_vel`, `arrival_damping`) ramp linearly over `late_p`.
Evaluation envs use p = 1 (final weights). `head_impact` is an event term (value =
onset·`T.head_impact_v2`/CTRL_DT with `head_impact_v2 = max(head_v2_prev, |v_head|²)`, so it contributes
`w·v²` once per contact onset; the previous step's speed is the pre-impact speed, the onset step's speed is
mostly post-impact because contact can start in any of the 4 substeps).

## Reset, getup

DR exactly as walk (every episode, from the nominal getup model). The fallen/midfall banks are built
with the nominal model, `PB.check_variant(b, "getup")`.
`kind = T.sample_reset_kind(xp, u, "getup")`: 0 fallen (`PB.fallen_bank()`), 1 midfall
(`PB.midfall_bank()`), 2 stand (the walk HOME reset). Bank kinds as in main (yaw-rotated bank
quaternion, xy = 0, bank z, `qvel = bank qvel` — midfall carries the angular velocity of the free
joint in its body frame, which a yaw rotation does not change; `ctrl = bank ctrl`). Then
`memory = T.getup_memory(xp, s_reset)`, `ev = T.getup_events_init(xp, trunk_z)`, `a_prev = 0`.
Keep `reset_kind` and `bank_label` (`b["label"][i]`, `PB.LABELS`; −1 for non-bank resets, both tasks and
both backends) in the env state for per-mode evaluation.
MJX trainer: the getup training env uses `random_initial_steps=True` (first reset only: `steps ~ U{0, 399}`,
so timeouts are staggered; without fall termination all envs would otherwise time out on the same step and
every PPO batch would see one 0.4 s window), and the getup eval env uses `reset_probs` without `stand`
(so `eval/episode_getup_success` measures recovery only).

## Random numbers

Walk streams must not change. New draws for main/getup are taken so that the existing ones keep
their positions — recommended MJX: reset uses `jax.random.fold_in(k_n, 2)` split into
(kind, bank index, yaw) and the existing `k_cmd` for the 8 segment uniforms; step uses `k_cmd` for
the 8 segment uniforms (main has no walk command resample). Recommended CPU: draw the extra reset
uniforms from the env's generator after the existing `n_reset_u` block, and the 8 segment uniforms
on demand when a segment expires (like `_sample_command`). Banks never use the env RNG
(`PB.*` are seeded by `C.POSE_BANK["seed"]` and identical in every process).

## Commands API (main)

`fixed_command`, `set_commands` / `set_command` accept `(..., 3)` or `(..., 5)`;
`TK.pad_command("main", c)` pads 3 → `[vx, vy, wz, H_HOME, 0]`. walk accepts only 3; getup has no
command (raise). Backends may additionally accept `reset_probs` overrides (e.g. `{"home": 1.0}`)
for play/eval (recommended ctor argument name: `reset_probs`).

## Episode metrics

Per finished episode: walk keys (`ep_len`/`length`, `sum_reward`, `rew_<term>` for every key of
`spec.reward_weights`) plus the per-step increments returned by `T.main_events` / `T.getup_events`,
summed over the episode, under the names in `spec.episode_metrics`:
- main: `walk_steps, sit_steps, seated_ok_steps, standup_attempts, standup_success,
  sitdown_attempts, sitdown_success, falls_walk, falls_sit, falls_transition` (ratios offline:
  e.g. `standup_success / standup_attempts`; a stand-up attempt counts only if the post-sit `h` command is
  ≥ `standup_z` + 5 mm, otherwise the robot is not asked to reach `standup_z`).
- getup: `getup_success` (0/1), `head_impacts`, `head_impact_v2`, `torque_sat_steps`.
MJX: add them to `metrics` and `episode_metrics` with the same key set in reset and step; CPU:
`info["episode"][name]` means like the `rew_*` keys.

## Policy export (main / getup)

Same files as walk (`policy.onnx`, `policy_meta.json` in the run dir; role from `meta["task"]`).
`policy.onnx`: `obs [batch, spec.num_obs] → action [batch, spec.num_actions]` (main 53 → 10,
getup 48 → 14). Meta: `rl.policy_meta.build_meta(steps, backend, idx, task)` (torch-free; walk output
is exactly the walk meta above). Runtime action mapping: main `target = q_ref(h_ref, alpha) +
action_scale·clip(a, ±action_clip)` with the filter/reference constants in `meta["posture_filter"]`,
`meta["posture_ref"]`; getup `target = clip(default_action_q + action_scale·clip(a, ±1), action_lo,
action_hi)`. Supervisor thresholds: `meta["supervisor"]`, success: `meta["getup_success"]`.


---

# Robot profiles (OpenDuck-TR)

Design: `docs/workflow/openduck-tr/spec.md` (v3.1, 3.2.3-3.2.7). Two levels select a robot:

| level | how | selects |
|---|---|---|
| process | `OPENDUCK_ROBOT` (read once by `rl/config.py`) → `C.ROBOT_PROFILE` ∈ `C.ROBOT_PROFILES` = (`xl330_27` default, `tr35_b2`) | every geometry/task constant, every `TaskSpec`, the default scene, the derived file, the reward key sets, the model build rules below |
| instance | `robot=` on `build_spec` / `build_model` / `VecEnv` / `OpenDuckEnv` (P4 meaning) | only the actuator + sensor model (`rl/actuators.py`, `rl/sensors.py`) |

- `xl330_27` process: everything above this section, bit-identical (constants, layouts, RNG streams, rewards, meta,
  output files). `robot="tr35_b2"` still works there on whatever scene it is given (the P4 tests run the tr35 servo
  model on the 27 cm MJCF).
- `tr35_b2` process: `robot` must be `None` or `"tr35_b2"`; `VecEnv` / `OpenDuckEnv` raise `ValueError` otherwise
  (`actuators.env_profile`).
- `backlash=True` (tr35 servo model only): a passive series joint `<joint>_play` (±b/2) after every DS32xx joint.
  `s["q"]` / `s["qd"]` are then the servo-output (pot) angle / velocity of the MAIN joint (the actuated one, which the
  pot reads), not of the link after the play. Pose banks stay in the backlash-free layout; envs expand them with
  `robot_model.expand_state` (play joints 0).
- Names that never change with the profile: `LEG_JOINTS`, `HEAD_JOINTS`, `OTHER_JOINTS`, `OBS_JOINTS`, `ACTION_JOINTS`,
  `HIP_POSE_JOINTS`, `CROUCH_JOINTS`, `FOOT_GEOMS`, `FOOT_SITES`, `FLOOR_GEOM`, `IMU_SITE`, `TRUNK_BODY`, `TRUNK_GEOM`,
  `HEAD_BODY`, `DEFAULT_KEYFRAME`, `SIM_DT`, `CTRL_DT`, `N_SUBSTEPS`, `JOINT_RANGE_MARGIN_DEG`, `ROBOT_PROFILES`, the
  task names and the obs/critic/action sizes (walk 49/58/10, main 53/64/10, getup 48/57/14). No existing config name
  or dict key is removed in either profile (e.g. `C.GETUP["torque_limit"]` stays, unused by the tr35 rewards).

## Paths

| name | value |
|---|---|
| `C.SCENE_PATHS` | `{"xl330_27": build/sim/scene.xml, "tr35_b2": build/tr35_b2/sim/scene.xml}` |
| `C.SCENE_PATH` | `$OPENDUCK_SCENE` or `SCENE_PATHS[ROBOT_PROFILE]`; `build_spec` reads it at call time (assigning `rl.config.SCENE_PATH` works) |
| `C.DERIVED_PATHS` | `{"tr35_b2": rl/derived/tr35_b2.json}` |
| `C.DERIVED_PATH` | `$OPENDUCK_DERIVED` or `DERIVED_PATHS.get(ROBOT_PROFILE)` (None for xl330) |
| `C.DERIVED`, `C.DERIVED_SOURCE` | the loaded derived dict and its `source` (None for xl330) |

In a `tr35_b2` process `rl/config.py` loads `DERIVED_PATH` at import (`C.load_derived(path)` → dict, `ValueError` with
the field on a bad file); a missing or invalid file raises `RuntimeError` naming the regenerate command
(`tools/derive_posture.py derive --scene build/tr35_b2/sim/scene.xml --out rl/derived/tr35_b2.json`). The committed
`rl/derived/tr35_b2.json` is generated from the P2-0 stub (`source.stub = true`); until the real TR-35 MJCF exists
(P2/P3), every tr35 process needs `OPENDUCK_SCENE` (the P2-0 stub, built by `tools/golden.py build <dir> --config
tr35_b2 --no-check`). `OPENDUCK_DERIVED` is only needed for another file (`rl/tests/tr35_scene.bootstrap_derived` is a
TEST-ONLY fallback used when the committed file is missing).

Regeneration after G-CAD (one command writes every geometry-dependent tr35 constant; `check` exits 0 valid and fresh,
3 valid but the MJCF or mesh sha256 changed, 1 invalid):

    python tools/derive_posture.py derive --scene build/tr35_b2/sim/scene.xml --out rl/derived/tr35_b2.json
    python tools/derive_posture.py check  --scene build/tr35_b2/sim/scene.xml --derived rl/derived/tr35_b2.json
    python tools/derive_posture.py caps-scan --scene build/tr35_b2/sim/scene.xml      # optional A8 voxel scan (P5b)

## Derived file (schema 1; written by `tools/derive_posture.py`, read by `rl/config.py`)

Units m, kg, deg; leg angles are LEFT-leg absolute (right = −left for the 5 leg joints); deterministic JSON (sorted keys,
indent 1, trailing newline). Keys: `schema` (1), `profile` ("tr35_b2"), `tool`, `tool_version`, `source` {scene,
mjcf_sha256, config_sha256, stub}, `params`, `mass` {total_kg}, `home` {source "keyframe"|"family", qpos (7+15, MJCF
order), ctrl (15), legs_deg, head_deg, trunk_z_kin, com_offset_x, com_margin}, `heights` {H_HOME, H_MIN_WALK,
H_MIN_STAND, H_DEEP, Z_SIT, SEAT_Z, SEAT_Z_TOL, TERMINATION_MIN_HEIGHT, standup_z, getup_success_z, getup_pose_z [2],
getup_arrival_z [2]}, `crouch_family` [[h, pitch, hip_pitch, knee, ankle], ...], `seat` {deg, pitch_deg (+ = nose
down), seat_gz, feasible}, `head` {home_deg, sit_deg, tuck_deg}, `feet` {home_lateral, FEET_MIN_DISTANCE}, `caps_deg`
{KNEE_LIMIT_DEG, NECK_PITCH_MAX_DEG, HIP_YAW_INWARD_MAX_DEG}, `extra_collision` {body: ["box", centre, half sizes,
null]} (body frame), `self_collision_pairs` [[geomA, geomB], ...], `report` (free-form, never read).
`load_derived` checks schema/profile, every key and type, `H_HOME > H_MIN_WALK >= H_MIN_STAND >= H_DEEP` and
`SEAT_Z < Z_SIT` (the same order as `derive_posture.validate`; `Z_SIT < H_DEEP` is only a derive warning: on the stub
the seat is 0.5 mm higher than the deep crouch and `z_ref` interpolates linearly either way),
`crouch_family` ≥ 3 rows with h strictly decreasing, first h ≥ H_HOME, last h ≤ H_DEEP, and the flat-foot invariant
`pitch + hp − kn + an` = HOME's value within 1e-6 deg.

Config mapping (tr35 process): `H_HOME = BASE_HEIGHT_TARGET = heights.H_HOME`; H_MIN_WALK, H_MIN_STAND, H_DEEP, Z_SIT,
SEAT_Z, SEAT_Z_TOL, TERMINATION_MIN_HEIGHT copied; `H_NORM = H_HOME − H_MIN_STAND`; `MAIN["standup_z"]`,
`GETUP["success"]["z"]`, `GETUP["pose_z"]`, `GETUP["arrival_z"]` from heights; `CROUCH_FAMILY`, `SEAT_DEG`,
`SEAT_PITCH_DEG`, `MAIN["seat_gz"]`, `HEAD_HOME_DEG`, `HEAD_SIT_DEG`, `HEAD_TUCK_DEG`, `FEET_MIN_DISTANCE`, the three
caps, `EXTRA_COLLISION`, `SELF_COLLISION_PAIRS`; `HOME_OVERRIDE = home` if `home.source == "family"`; the DR trunk
added mass scales with `mass.total_kg / 0.6763`. Nothing geometry-dependent is typed in code for tr35.

## tr35 formula constants (`C.tr35_constants(derived, kappa=None)`)

Pure function (new objects; callable from an xl330 process). `S = C.FROUDE_S = 1.282`, `k = C.KAPPA` (`kappa` argument,
else `$OPENDUCK_KAPPA`, else 0.5). Walk: `λv = √S·k`, `λω = k/√S`, `λt = √S/k`; main sit/stand: `λt = λv = √S`; getup:
`λt = λv = √S`, `λω = 1/√S`.

| constant | tr35 value |
|---|---|
| `GAIT_FREQ`, `AIR_TIME_TARGET` | `2.21·k` Hz, `0.375/GAIT_FREQ` |
| `SWING_HEIGHT`, `FOOT_HEIGHT_SIGMA` | `0.02·S`, `2e-4·S²` |
| `COMMAND_RANGES` | vx, vy × λv; wz × λω |
| `MOVING_EPS = MAIN["moving_eps"]` | `0.05·λv` (walk `moving` threshold; xl330 0.05) |
| `TRACKING_SIGMA_LIN / _ANG` | `0.02·λv²` / `0.1·λω²` |
| `ACTION_SCALE` | 0.3 rad |
| `REWARD_WEIGHTS` | xl330 with lin_vel_z /λv², ang_vel_xy /λω², base_height /S², joint_acc ·λt⁴, feet_slip /λv², feet_distance /S; `torques` → `torque_ratio` −2e-3 (same position); + `limit_torque` −0.5, `rail_demand` −0.2 (initial, tuned in P7) |
| `MAIN_REWARD_WEIGHTS` | walk terms from the tr35 walk table; height_l1 /S; vz_cap /√S; same three torque/rail terms |
| `MAIN` | height_sigma ·S, vz_cap ·√S, standup_window_s 3.4, sitdown_window_s 4.0, `stand_gz` −0.9 (stood test), `seat_gz` from the derived file (seated test) |
| `SIT_RAMP_S = STAND_RAMP_S` | `2.5·√S` (2.83 s) |
| `GETUP_EPISODE_S` | 12.0 s |
| `GETUP` | height_sigma ·S; composite_sigma (0.03·S, 0.4, 0.4); success gyro 0.5/√S, hold_s √S; midfall ang_vel (0, 4/√S); `torque_limit` 2.88 (unused) |
| `GETUP_REWARD_WEIGHTS` / `_LATE` | height_l1 /S, height_progress /S, joint_vel and arrival_damping /λω² (early and late), late head_impact /λv²; `torque_sat` −1.0 on ratios; + `limit_torque` −0.5, `rail_demand` −0.2 (early only) |
| `HEAD_RANDOM` | range neck 0.15, head_pitch 0.2, head_yaw 0.5, head_roll 0.08 rad; filter_tau 0.5 s; `slew_max` {joint: 0.5·ω0(class)} (asserted, not clipped: `2·range/filter_tau ≤ slew_max` and the main head script rate ≤ slew_max) |
| `GETUP_HEAD_SCALE` | {head_pitch 0.2, head_yaw 0.5, head_roll 0.08} rad (getup action scale cap) |
| `SUPERVISOR` | fall_steps 6, ready_hold_s 0.6 |
| `RESET_BASE_VEL_NOISE` | `0.05·√S` |
| `POSE_BANK` | settle_s 2.5, drop (0.02·S, 0.10·S) |
| `DOMAIN_RAND` | trunk_added_mass × total_kg/0.6763, trunk_com_offset 0.010 m, push_vel 0.20 m/s (servo DR of `actuators.DR_OVERRIDES` / `actuators.DR` on top) |
| `HEAD_GEOMS` | (`head_collision`, `jaw_collision`, `neck_bumper`); RobotIndex drops missing geoms |

Other new names (xl330 values): `FROUDE_S` None, `KAPPA` None, `SEAT_PITCH_DEG` 0.0, `HEAD_TUCK_DEG` None,
`HOME_OVERRIDE` None, `SELF_COLLISION_PAIRS` (), `GETUP_HEAD_MODES` ("free", "fixed"), `GETUP_HEAD_DEFAULT` "free",
`GETUP_HEAD_SCALE` None, `MOVING_EPS` 0.05, `RAIL_SOFT_A` 15.0 A, `MAIN["stand_gz"]` −0.9.
`OPENDUCK_BANK_N=<int>` overrides all three `POSE_BANK` sizes (both profiles; smoke runs and tests).

## TaskSpecs, tr35

| field | walk | main | getup |
|---|---|---|---|
| `actor_hidden` / `critic_hidden` | (256,128,64) / (512,256,128) | same | same |
| `solver_iterations` (MJX; ls 8 = trainer default) | 8 | 8 | 16 |
| `episode_length_s` | 20 | 20 | 12 |
| `action_scale` | 0.3 | 0.3 | per joint, head_pitch/yaw/roll ≤ `GETUP_HEAD_SCALE` (neck_pitch full ctrlrange) |

Solver choice (P5a measurement on the P2-0 stub, CPU-JAX, DR + noise; `tools/mjx_solver_bench.py`): 4 Newton
iterations are not converged for tr35 friction (one-step joint-velocity error p99 0.2-1.5 rad/s vs 64/32); 8 gives p99
≤ 1e-6 with rare outliers (max 2.5e-3 main, 0.08 rad/s getup); 16 gives max ≤ 1.7e-4. 16 costs +2-3 % over 8 at 256
envs and +8-12 % at 2048 envs; frictionloss 1 % of stall saves no time, so it stays at 3 % until the Step-0 sysID.
P5b re-measures on the final MJCF and on GPU.

## Models, tr35 process (`rl/robot_model.py`)

- `C.HOME_OVERRIDE` (family HOME) overwrites keyframe `stand` qpos/ctrl before `actuators.apply_to_spec` (which
  inserts play-joint zeros).
- MJCF `<pair>`/`<exclude>` entries are deleted; the RL list `C.SELF_COLLISION_PAIRS` is authoritative (until P5b).
- `main` and `getup` get `C.EXTRA_COLLISION` boxes: main pair-only (contype 0, conaffinity 0), getup floor-only
  (contype 0, conaffinity 1). Then spec pairs for the `SELF_COLLISION_PAIRS` entries whose two geoms exist: getup the
  full list (30 pairs on the stub), main only `robot_model.main_pairs` (thigh/shin ↔ trunk, foot ↔ same-leg
  thigh/hip_cradle: 8 pairs), walk none. Cost (P5a, stub, real derived file, CPU-JAX 256 envs, shared machine): main
  +77-82 % over no pairs (8 pairs), getup +104-123 % (30 pairs; the 8-pair subset would be +17-28 %). P5b re-measures
  on the final MJCF and on GPU (8192 envs) and decides getup full list vs subset (+ head ↔ trunk) or condim 1.
  CPU/MJX step parity does not hold while a box-box self pair is in contact (MJX makes 4 contact points per pair,
  MuJoCo C 1-2); the tr35 lockstep test (`test_tr35_lockstep_cpu_mjx`) therefore runs with `SELF_COLLISION_PAIRS = ()`
  until P5b (capsule EXTRA geoms, smaller boxes, or a separately reported paired difference).
- A text marker `openduck_variant:<variant>`; `model_variant()` reads it (xl330 detection unchanged).
- `PB.bank(kind, n=None, seed=None, robot=None, scene_path=None)`: None = process default; the cache key includes
  robot and scene; envs pass their own. The seated bank places the trunk at `SEAT_PITCH_DEG + U(±tilt)` and accepts
  with `MAIN["seat_gz"]`. On the P2-0 stub the derived seat (pitch −50°, COM margin 6.9 mm) holds when placed
  exactly (settles at −49.2°, z 0.0833 m) but the ±5° / σ 0.05 rad bank placements tip forward onto the trunk
  (pitch +3..+12°, z 0.023-0.031 m) and are still accepted; `test_tr35_small_banks` asserts the bank pitch / z window
  only for a non-stub derived file (P5b).

## Control step, tr35 servo model (both backends, `robot="tr35_b2"`)

Steps 1-3 as the task's control step (targets, head, legs, clip), then the ctrl is rounded to the PCA9685 step once
per control step (`actuators.quantize_pwm`), and step 5 (physics) runs per substep `k`: delayed target (per-board
delay), dead band (`actuators.dead_band`), predicted LEG rail demand and CC scaling of the DS forceranges
(`actuators.shape_substep`), `mj_step`, then `actuators.after_substep` and one sensor tick (`sensors.pot_tick`,
`sensors.imu_tick`). The actor observation is built from the sensor model (`sensors.obs_state`: pot q/qd, measured
gyro, complementary-filter gravity); rewards and the critic use the true state; the generic `OBS_NOISE` is off.
Random numbers: env_cpu draws `actuators.n_uniform(nu) + sensors.n_episode(14)` per episode after the reset block and
`N_SUBSTEPS · sensors.n_tick(14)` per step after the U block; MJX uses `fold_in(k_n, 3)` and `fold_in(k_noise, 5)`.

Extra state (`s` dict and `task_state()`, OBS_JOINTS order, per env): `tau_net` (actuator_force − b·qd, motor torque),
`tau_s` (episode-sampled stall torque), `tau_ratio = tau_net/tau_s`, `limit_torque` (|joint-limit constraint torque|
of the main joints, max over substeps), `torque` (= actuator_force); `rail_demand` (substep mean of the pre-scaling
LEG demand, A), `rail_peak`, `cc_scale`; `task_state()` adds `q_obs`, `qd_obs`, `gyro_obs`, `gravity_obs`, `ctrl_prev`.

## Rewards, tr35 process (`rl/task_math.py`, static branch on `C.ROBOT_PROFILE`)

- walk, main: `torque_ratio = Σ tau_ratio²` (14 obs joints) in place of `torques`; then `limit_torque =
  Σ (limit_torque/tau_s)²` and `rail_demand = max(rail_demand − RAIL_SOFT_A, 0)` appended.
- getup: `torque_sat = Σ max(|tau_ratio| − GETUP["torque_sat_frac"], 0)`; `limit_torque`, `rail_demand` appended;
  `getup_events` `torque_sat_steps = [max |tau_ratio| > torque_sat_frac]`.
- main: `pitch_ref = lerp(pitch_F(hd), radians(SEAT_PITCH_DEG), s2)` when `SEAT_PITCH_DEG != 0`; `main_events` uses
  `MAIN["stand_gz"]` for "stood" and `MAIN["seat_gz"]` for "seated".
- `reward_terms` always returns exactly the keys of `TaskSpec.reward_weights`, in order.

## getup head mode (`getup_head=`; `VecEnv` and `OpenDuckEnv`)

`None` → `C.GETUP_HEAD_DEFAULT` ("free"); `"free"`: the head is policy-controlled (as above). `"fixed"`: after clipping
the action, the 4 `HEAD_JOINTS` action dims (getup action indices 10..13) are set to 0 and the head ctrl target is
`radians(C.HEAD_TUCK_DEG)` (absolute); obs/action layout and RNG are unchanged. Only task getup uses it (walk/main
store and ignore it); `"fixed"` without `HEAD_TUCK_DEG` (xl330) raises `ValueError`. The resolved value is
`env.getup_head`. (Spec 3.4 getup gate: runs (i) fixed and (ii) free.)

## Tools

- `tools/tr35_smoke.py --out <dir> [--scene <scene.xml>] [--derived <json>] [--lock <dir>]`: walk, main and getup CPU-JAX
  smoke trainings in a tr35 process (`OPENDUCK_BANK_N=64`, 32 envs, 2e4 steps); checks exit 0, finite log metrics,
  `policy.onnx` and `policy_meta.json`; getup fixed head via the trainer flag if present, else a direct env check.
- `tools/mjx_solver_bench.py --scene <scene.xml> --out <json> [--mode time|acc|all] [--tasks] [--iters] [--fric]
  [--pairs off,derived,spec,main-subset] [--n]`: MJX control-step cost and one-step accuracy (spec A3).
- `tools/derive_posture.py derive|check|caps-scan` (see "Paths"): the only producer of `rl/derived/tr35_b2.json`.

## Policy meta, C export, eval, play, trainers, launcher (tr35)

- `rl/policy_meta.build_meta(steps, backend, idx=None, task, network=None, scene_path=None, robot=None,
  getup_head=None)`: the xl330 meta is unchanged. When `idx.robot == "tr35_b2"` it adds `robot`, `actuators`
  (`actuators.actuator_table` columns in actuator order: names, cls, tau_s, omega0, pband, kp, damping, armature,
  frictionloss, deadband, pwm_res, i_stall, i_idle, board, on_leg, ctrl_lo, ctrl_hi; plus classes, rails, i_cc, boards
  {"0": 300 Hz PCA 0x40, "1": 50 Hz PCA 0x41}, units), `sensors` (POT, IMU, pot_table of the obs joints), `delays`,
  `derived_source` (`C.DERIVED_SOURCE`), `derived_sha256` (sha256 of the loaded derived JSON, tr35 process),
  `kappa` (`C.KAPPA`), `network` (when passed), getup: `getup_head` (+ `head_tuck_deg` = `C.HEAD_TUCK_DEG` when
  "fixed"), main: `posture_ref.seat_pitch_deg` (`C.SEAT_PITCH_DEG`). All geometry values in the meta (default_*_q =
  HOME, posture_ref, supervisor, getup_success) come from config, i.e. from the derived file in a tr35 process.
- `rl/export_c.py` (`python -m rl.export_c --onnx X.onnx [--meta M.json] --out DIR [--check N>=1000]`): C99 float32
  policy with the normalisation folded into layer 1; writes `policy_weights.h`, `policy.h`
  (`void openduck_policy(const float *obs, float *action)`), `policy.c`, `obs_layout.h`, `export_c.json`. `--check`
  compiles with `cc -O2 -std=c99 -ffp-contract=off` and compares with onnxruntime (exit 1 above 1e-5). For a tr35
  meta `obs_layout.h` also carries the robot constants: `OPENDUCK_NU`, `OPENDUCK_ACT_NAMES/TAU_S/OMEGA0/KP/DEADBAND/
  PWM_RES/CTRL_LO/CTRL_HI/BOARD[nu]`, `OPENDUCK_BOARD<k>_PWM_HZ/_PCA`, `OPENDUCK_DERIVED_MJCF_SHA256`,
  `OPENDUCK_DERIVED_STUB`, `OPENDUCK_DERIVED_SHA256`, `OPENDUCK_KAPPA`, getup fixed `OPENDUCK_HEAD_TUCK_RAD[4]`, main
  `OPENDUCK_SEAT_PITCH_DEG`; `export_c.json` repeats derived_source, derived_sha256 and kappa.
- Trainers (`rl.train`, `rl.train_mjx`): `--getup-head free|fixed` (absent → not in config.json; passed to the env
  only for getup and when not "free"); in a tr35 process every ONNX export also writes `<run>/c/` (export_c with a
  1000-observation check when a compiler exists). xl330 runs write the same files as before.
- `rl/eval_tasks.py`: tr35 mode (process profile tr35_b2, a meta robot tr35_b2, or `--criteria tr35`) runs spec 6
  A4-A8 with `--scene`, `--robot`, `--no-backlash`, `--getup-head`, `--t-knee` (default tau_s(ds32xx)/3 until
  G-BENCH), `--n-scale`, `--a7-onnx`, `--no-collision`; JSON version 3, `criteria_mode` "tr35_b2", every section with
  status / criteria / pass, `pass_all`. Torque criteria use NOMINAL tau_s; each `torque` summary also reports the
  training penalties `penalties` {torque_ratio, limit_torque, rail_demand (unweighted step means,
  `task_math.tr35_penalties`, episode tau_s), rail_soft_a = `C.RAIL_SOFT_A`, torque_sat_steps_frac}; the rail
  criterion threshold is `C.RAIL_SOFT_A`; A6 modes add the env's getup `episode_metrics` (getup_success,
  head_impacts, head_impact_v2, torque_sat_steps and its per-step fraction). The xl330 sections are unchanged.
- `rl/play.py`: in a tr35 process the demo programmes come from `C.COMMAND_RANGES` fractions and `C.H_*`
  (`tr35_programs`), `TIP_W = 10/√S`, per-joint saturation `|tau_net| / nominal tau_s` (`peak_tau_joints`); `--scene`,
  `--getup-head` (default: the getup meta's `getup_head`).
- `tools/runpod/launch.py launch --robot tr35_b2 [--kappa K]`: adds `OPENDUCK_ROBOT` (and `OPENDUCK_KAPPA`) to the pod
  env and the tr35 files (`build/tr35_b2/sim/openduck.xml`, `build/tr35_b2/sim/scene.xml`, `rl/derived/tr35_b2.json`)
  to the package; use `--source git` (the inline body is over the RunPod limit).
