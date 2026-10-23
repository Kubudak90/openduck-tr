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
  - each of the 14 STS3215 actuators: `kp' = kp · U(kp_scale)`; set `gainprm[0] = kp'`, `biasprm[1] = −kp'`.
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
