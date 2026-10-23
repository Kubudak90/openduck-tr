# GPU backend (MJX + Brax) implementation map: sit, body height and get-up

I read and measured only; nothing in the repo was changed. My scripts and their output are in `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/sitstand/map-mjx/scripts/` (`probe_model.py`, `bench_mjx.py`, `drop_settle.py`, `reach.py`, `reach2.py`). The raw benchmark output is `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/sitstand/map-mjx/bench_out.txt`.

## 0. Measured facts that shape the design

| Fact | Value |
|---|---|
| Collision geoms | 5 boxes: floor plane, `trunk_collision` half-size (0.0419, 0.033, 0.042), both feet, `head_collision`. Hips, thighs and shins have **no collision** (leg body origins go up to 6 mm below the floor in drop tests). |
| MJX contact slots | 20 = 4 each for floor–trunk, floor–head, floor–left foot, floor–right foot, and left foot–right foot. Trunk and head floor contacts already exist, so sitting and fallen poses change no static shapes. |
| Trunk box vs. termination | The box is centred on the trunk origin, so an upright trunk resting on the floor gives `trunk_z` ≈ 0.042–0.046. That is below `TERMINATION_MIN_HEIGHT = 0.075` (config.py:88), so **sitting currently counts as a fall**. At HOME the box bottom is at 0.0773 m and `trunk_z` is 0.1193. |
| Mass | Total 0.6746 kg. Trunk 0.1655 kg. Neck→head→jaw chain 0.2455 kg (36%). |
| **Sit is out of reach of the current action mapping** | Targets are `q0 + 0.5·clip(a, ±1.5)`, so each joint can move only ±0.75 rad from HOME (config.py:41-42, task_math.py:167-170). Sweep: mirrored hip_pitch/knee/ankle, feet within 20° of flat, trunk pitch −40…+40°. Smallest trunk-box-bottom minus foot-bottom gap: **+8.4 mm at ±0.75 rad (cannot sit)**, −6.2 mm at ±1.0, −17 mm at ±1.25, −33 mm at ±1.5. |
| Crouch range at ±0.75 rad | Trunk origin goes no lower than about 0.094 m above the feet, so crouch walking covers roughly h ∈ [0.094, 0.119] m. A per-joint action scale (or one ≥ 0.67) is needed for sit and for deeper crouch. |
| Cost, CPU JAX, 128 envs | vmapped `step_no_reset` 49.1 ms; vmapped `auto_reset` (DR + set_const + forward) 1.68 ms, about 3.4% of a step. One physics substep is about 12 ms. |
| Drop and settle (MuJoCo C, 60 drops from z = 0.30, random orientation, HOME ctrl, ±0.3 rad joint noise) | Settles in median 0.74 s and p90 1.29 s (148–258 substeps, i.e. 37–65 control steps). Final `trunk_z` median 0.046 (0.044–0.143). All 60 end with gz > −0.6. Body x-axis 18 pointing down (face-down if +x is forward), 16 pointing up (on the back), 23 on a side, 3 inverted. |

## 1. State / info layout (`rl/env_mjx.py`)

**`State`** is the Playground `State` (data, obs, reward, done, metrics, info), with a fallback copy at :62-69.

**`info` keys** are created in `_reset` (:450-489) and updated in `_step_no_reset` (:502-596). Dtypes were checked by `bench_mjx.py`.

| key | shape / dtype | written at | notes |
|---|---|---|---|
| `rng` | (2,) u32 | reset :449-450; step :505 → :590 | Each step splits it 6 ways: `rng, k_head, k_push, k_cmd, k_noise, k_reset`. |
| `reset_key` | (2,) u32 | reset :450 (`k_n`); step :590 (`k_reset`) | `auto_reset` resets from it (:445). |
| `dr` | dict | :451-452 (`sample_dr` :295-323) | friction (), body_mass (17), body_inertia (17,3), body_ipos (17,3), kp (15), dof_damping/frictionloss/armature (21), delay () i32, plus the set_const outputs body_invweight0 (17,2), dof_invweight0 (21), dof_M0 (21), body_subtreemass (17), meaninertia (). Each step rebuilds the model through `_model` (:402) → `_apply` (:325). |
| `steps` | i32 | :476; :506 (set to 0 if `prev_done`), :550 (+1) | |
| `truncation`, `time_out`, `episode_done` | f32 | :476-477, :590-591 | |
| `episode_metrics` | dict f32 | :478, :585-589 | `sum_reward`, `length`, `rew_<15>`. Zeroed lazily through `(1 - episode_done)`. |
| `command` | (3,) f32 | :465-472, :574-577, merge :606 | |
| `cmd_lock` | f32 | :466-472, `set_command` :618 | |
| `phase` | f32 | :473 (U[0,1)), :549 (+0.05 per step) | |
| `a_prev` | (10,) | :480, :571 | |
| `air_time` | (2,) | :480, :561 / :570 | |
| `head_offset`, `head_target` (4,), `head_timer` () | f32 | :483-485, :519-527 | Only when `head_random`. |
| `push_timer` | f32 | :487, :535-540 | Only when `push`. |
| `abuf` | (K+1, 10) | :489, :512-514 | Only when K > 0. Currently K = 0. |

- **`_KEEP_INFO`** (:107) lists the keys kept from the finished episode in `merge_reset` (:598-608): `rng, reset_key, steps, truncation, time_out, episode_done, episode_metrics, cmd_lock`. Every other key comes from the fresh reset. `merge_reset` loops over `stepped.info` and reads `fresh.info[k]`, so the key sets must be identical.
- **`metrics`** = `{"rew_"+k: terms[k]*w_dt[k]}` (:584), initialised to zero in reset (:493). `w_dt` (:272) uses the global `C.REWARD_WEIGHTS`. `TERMINATION_PENALTY` appears only in `sum_reward`.
- **The `s` dict for task_math** is built in `_task_state` (:419-424): `extract_state` (:132-149), then `nan_to_num`, then action, last_action, command, phase, friction, default_q.
  - Step adds `air_time` and `first_contact` (:561-562), then overwrites `last_action` and `command` for the next observation (:594).
  - Reset calls `_task_state(data, info, 0, 0, command, phase)` (:491).
  - Contact flags come from `foot_contacts` (:116-129).

## 2. Command sampling / resampling

- `_sample_command(key)` (:408-411) draws uniform `cmd_lo..cmd_hi` (:270-271, 3-dim). With probability `COMMAND_ZERO_PROB` = 0.15 the whole command is zero.
- **Reset** (:465-472): `fixed_command` (ctor :248, `reshape(3)`) sets lock = 1; otherwise the command is sampled. In `auto_reset` a locked command is kept (:470-472).
- **Step** (:574-577): resamples when `steps % CMD_STEPS == 0` (CMD_STEPS = 300, :103) and `cmd_lock < 0.5`. `merge_reset` keeps locked commands (:606). `set_command` (:615-619) is shape-generic.

**Adding `h` and `sit`** (recommendation): use one 5-dim command `[vx, vy, wz, h, sit]`. This reuses lock, `merge_reset` and `set_command` unchanged.

1. `__init__`: change `fixed_command.reshape(3)` to `reshape(NUM_CMD)`. Add constants `H_RANGE`, `SIT_PROB`, `SIT_S=(3,6)`, `WALK_S`, `UNSIT_GRACE_S`.
2. Split the command logic into two functions:
   - `_sample_walk_cmd(key)`: returns vx, vy, wz, h. Keep h in [0,1], normalised, with 1 = HOME; the metre mapping goes in config and in the meta file.
   - `_update_mode(info, key, steps)`: one function called from both reset and step.
3. New f32 info keys:
   - `mode_timer`: seconds until the next decision about the sit flag.
   - `mode_t`: seconds since the last change of the sit flag.
4. When `mode_timer` runs out, flip the flag:
   - sit 0 → 1 with probability `SIT_PROB`, then `timer = U(SIT_S)`.
   - sit 1 → 0, then `timer = U(WALK_S)`.
   - Set `mode_t = 0` on every flip.
5. Every step, `cmd = where(sit, [0, 0, 0, h_sit, 1], cmd)` so the observation shows zero velocities. `h` can keep the existing 300-step schedule.
6. Keep all of this as `jnp.where` and gate every change with `cmd_lock` (or a separate `mode_lock`).
7. Replace the step code at :574-577 and the reset code at :465-472 with calls to `_update_mode`.
8. If the initial state is seated, initialise `sit` and `mode_t` consistently with it (§3).

**task_math must change too:**
- `reward_terms` :119: `moving = sum(cmd**2)` over all dims would always be true once h is in the command. Restrict it to `cmd[..., :3]`.
- :117-118 already slice `:2` and `2`.
- `base_height` :139 uses the constant `BASE_HEIGHT_TARGET`. Make it a target taken from `s["command"]` (h → metres; `H_SIT` when sitting).
- `feet_phase` (:122-123, :144) is **not gated by `moving`**. It rewards stepping even at zero command and conflicts with sitting. Gate or weight it by mode.

## 3. Reset path and other initial-state distributions

**Current reset**, `_reset` (:447-494):
- Split the key 10 ways.
- DR: `sample_dr`, including `set_const` (:346-378), which takes a full mass-matrix inverse and a vmapped Jacobian per body.
- `qpos = init_qpos`, the 14 observed joints get ±0.05 noise (:456), random yaw rotation of the quaternion (:457-460), xy = 0, z from the keyframe.
- `qvel` is zero except base xy ±0.05 (:461-462).
- `_forward_obs` (:426-434): kinematics, com, collision and sensors only; qacc and actuator_force stay 0.
- Observation noise uses `fold_in(k_n, 1)`.

**`randomization_fn`** (:380-393) exists in Playground format but is **not used**: `wrap_for_brax_training` raises an error if it is given (:658-659). DR is drawn per episode, stored in `info["dr"]`, and turned into a per-env `mjx.Model` every step.

**`VmapAutoReset.step`** (:630-638) runs a vmapped `step_no_reset`, then a batch-level `lax.cond(any(done))`. When it fires, it runs vmapped `auto_reset` and `merge_reset` for all envs. With 8192 envs and episodes under about 2000 steps, `any(done)` is almost always true, so **reset runs for the whole batch on nearly every step**.

**Cheap alternative starts (pose bank):**
- **Do not settle physics inside reset.** It would cost 37–65 control steps (measured), multiplied across the batch, roughly a 40–65× slowdown.
- Instead build a fixed bank at env construction, `self.bank_qpos: (NB, 22) f32`, from either:
  - an offline `.npz` made with MuJoCo C (60 drops take a few seconds), or
  - a one-off jitted vmapped `fori_loop` of about 300 substeps on GPU at startup (NB = 8k–16k is about 2.8 MB).
- Bank variants:
  - Fallen: drop from random orientation, z about 0.3, random joint targets, settle 1.5 s.
  - Seated: a scripted crouch, or rollouts of a sit policy once one exists.
- In `_reset`:
  - `kind = categorical(k_kind, P_INIT)`, `i = randint(k_bank, (), 0, NB)`, `q_b = bank[kind_offset + i]`.
  - Rotate yaw with `_quat_mul(yaw_q, q_b[3:7])`; the current code rotates the keyframe quaternion, which is wrong for bank poses.
  - Set xy = 0, keep the bank's z.
  - `qpos = where(kind == STAND, qpos_stand, q_b)`.
  - The gather costs next to nothing compared with `set_const`.
- Keep the bank as a closure constant of a few MB. A much larger bank slows compilation and inflates the persistent cache.
- Derive new keys with `jax.random.fold_in(k_n, 2..)` so the existing split(10) streams stay the same.
- If a seated start is chosen, set the initial `sit` flag per a probability (sit = 0 teaches standing up from the floor). A locked command still overrides it.

## 4. Termination

- `T.terminated` (task_math.py:160-164): falls if `trunk_z < 0.075` or `gz > −0.6`.
- Used only at env_mjx.py:565 (`where(bad, 1, …)`). From there it feeds the reward penalty (:567, −10 once), `fell`/`done` (:580-581) and `time_out` (:582). The CPU backend uses it at env_cpu.py:411.

Changes needed:
1. Add to `s`:
   - `sit`, `mode_t`;
   - `trunk_floor` and `head_floor` contact flags. Generalise `foot_contacts` into `geom_floor_contacts(data, geom_ids)`; the slots already exist.
2. New `terminated(xp, s)`:
   - `low_ok = (sit > 0.5) | (mode_t < UNSIT_GRACE_S)`, where the grace window covers standing up after sit is cleared;
   - `fell = (gz > TILT_MAX) | head_floor | (~low_ok & (trunk_z < MIN_H))`;
   - optionally a stricter tilt threshold while sitting (needs lean: about 40° in the sweep, gz ≈ −0.77).
   - For crouch walking, MIN_H must sit below the lowest commanded h minus a margin. With the current action range the crouch is ≥ 0.094 m, so 0.075 does not bind.
3. Get-up env: no fall termination, only `bad` (:553-557) and timeout.
   - If you terminate on success, Brax treats it as terminal with no bootstrap, so add a success bonus. Otherwise do not terminate.
   - The success/hand-off test must use observable signals (gravity, joint error vs HOME, gyro). `trunk_z` is not available on the robot.

## 5. Hard-coded sizes 49 / 58 / 10, networks and export

**Hard-coded:**
- config.py:31, :48, :49 (`NUM_ACTIONS`, `NUM_OBS`, `NUM_CRITIC_OBS`).
- `task_math.noise_vector` :94-95 hard-codes 14 joints and `NUM_ACTIONS + 3 + 2`.
- env_mjx :248 (`reshape(3)`), :270-271 (3-dim command range), :283-284 (`action_size = C.NUM_ACTIONS`), :474 (`zero_a`), :489 (abuf).
- robot_model.py:52 (`action_act_id` from the global `C.ACTION_JOINTS`) and :75 (`default_action_q = default_obs_q[:NUM_ACTIONS]`). This only works because the joints come first in `OBS_JOINTS` order; 14 = LEG + HEAD happens to work.
- train_mjx.py:193 (`NUM_CRITIC_OBS` zeros in `jax_policy_cpu`), :159-160 and :164-167 (meta: dims, `default_leg_q`, `leg_joints`), file names at :224 and :237.
- export_onnx.py:114-115.
- play.py:28 (`CMD_SLICE = NUM_OBS-5:NUM_OBS-2` breaks once the command grows), :40, :52.
- Tests: test_env_mjx.py:26 (critic indices 52 / 53:55 / 57), :215 and :223 (`obs[44:47]`), :366-388 (meta key set).

**Already generic:**
- `observation_size` (eval_shape of reset).
- Brax networks: `network_factory(obs_shape, env.action_size)` with `obs_shape` taken from `env_state.obs` (brax train.py:430, :442-444).
- `NormalDistribution(event_size=action_size)`.
- `actor_arrays` / `actor_to_onnx` / `check_onnx` read dims from the arrays (:145-146, :203).

**So a 14-action get-up policy works through Brax and ONNX** once the env exposes `action_size = 14` and writes the targets to `idx.action_act_id` for legs and head. It also needs:
- `head_random = False`;
- a per-instance action-joint list instead of the global one;
- a vector action scale, because ±0.75 rad is not enough for getting up or sitting (§0);
- meta built from the params: `obs_dim = mean.shape[0]`, `action_dim = layers[-1][0].shape[1]`, plus `action_joints`, `default_action_q` and `action_scale[]`;
- an `export_policy(..., name="getup")` option that writes `getup.onnx` and `getup_meta.json` (matches play.py's `splitext + "_meta.json"`).

**Other blockers:**
- `--restore` across a change in obs size fails on shape mismatch. Add an `expand_params` helper that inserts zero rows into `MLP_0/hidden_0` kernels and pads the normalizer mean/std/summed_variance, to warm-start from the 49-dim walker.
- `wrap_for_brax_training` requires `episode_length == C.MAX_EPISODE_STEPS` (:662-663); the get-up env needs its own length.
- Add a layout to the meta file: `obs_layout` with named slices, `command_names` and `h_range`, so runtimes stop computing offsets.

## 6. Logging (log.jsonl)

`progress` (train_mjx.py:314-324) writes `{"step", "time", **metrics}` per call.

**Eval lines** (`acting.Evaluator` and `EvalWrapper`, envs/wrappers/training.py:177-220):
- `eval/episode_reward`, `eval/episode_rew_<15>`, all with `_std` versions;
- `eval/avg_episode_length`, `eval/std_episode_length`, `eval/epoch_eval_time`, `eval/sps`, `eval/walltime`;
- the training metrics: `training/{sps, walltime, total_loss, policy_loss, v_loss, entropy_loss, kl_mean, learning_rate, policy_dist_{mean,max,min}_{std,loc}}`.
- Eval sums `state.metrics` while an episode is active, over 128 envs × 1 episode × 1000 steps. Keys ending in `_per_step` are divided by episode length.
- The eval env is `make_env()` with the same DR, noise, head and push flags (:342).

**Training lines** (with `--log-training-metrics`, logger.py:44-57, :89): `episode/<key>` from `info["episode_metrics"]` (requires a `length` key) plus `episode/sps`.

**Adding per-mode metrics:**
- Add keys to `metrics` (:584), initialise them in reset `metrics` (:493) and `episode_metrics` (:478). Line :588 already accumulates them into `episode_metrics`.
- Use numerator/denominator pairs, e.g. `sit_steps` and `sit_ok_steps` (sit and trunk_floor and upright and low gyro), `walk_steps` and `h_track_ok`, `fall_sit` / `fall_walk` / `fall_transition`.
- Event counts: `standup_attempt` at the 1 → 0 flip and `standup_success`, which needs a pending-flag and a timer in info.
- For get-up: `getup_success` and `steps_to_getup`.
- Compute ratios offline; `_per_step` normalises by total length, not by time spent in a mode.
- For cleaner per-mode eval, give the eval env a higher `SIT_PROB` and pass it as a ctor flag to `make_env(eval=True)`.

## 7. JAX pitfalls

1. Inside the vmapped env, `lax.cond` becomes a select and both branches run. Use mode-blended weights with `jnp.where`: `W = W_WALK + sit·(W_SIT − W_WALK)` returned per env by `T.reward_weights(xp, s)`, then pass them to `total_reward(xp, terms, term, W)` and use them for the metrics as well (currently the global `w_dt`, :272/:584). The key set must be static: the union of all terms, with weight 0 where a term is unused.
2. The only real branch is the batch-level cond in `VmapAutoReset` (:638), and in practice it always fires. Keep reset cheap: no physics, a bank gather only.
3. The pytree must match between reset, step and both cond branches: same info keys and the **same dtypes**. Keep mode flags and timers as f32 (like `cmd_lock`); `steps` is i32. Decide `_KEEP_INFO` membership per key: per-episode accumulators are kept and zeroed through `prev_done`/`episode_done`; mode state comes from the fresh reset.
4. RNG: changing `split(…, 6)` (:505) or `split(…, 10)` (:449) changes every random stream. Use `fold_in` for new draws.
5. Brax normalizer: `std = clip(sqrt(var + std_eps), 1e-6, …)`, with `std_eps = 0`; train_mjx does not pass `normalize_observations_std_eps`. A channel that stays constant (sit always 0 in an early curriculum stage) ends up with std ≈ 1e-6, and its first change becomes a ±1e6 input. Sample every mode from step 0, or set std_eps.
6. Config constants are fixed at trace time. Mid-run curriculum changes need jnp scalars carried in state, or staged runs with `--restore` (same shapes).
7. Horizon: `--discounting 0.97` at 50 Hz is about 0.66 s. Sit/stand transitions and getting up take 1–3 s; consider γ = 0.99 for those.
8. Adding leg collision geoms for get-up realism adds contact slots and cost. Capsule–plane is cheap; avoid box–box. With 4/8 Newton iterations, check against MuJoCo C on fallen poses, in the style of `test_dynamics_match_c`.
9. New matmuls inside reset/step are fine: they inherit `_precision()`, and `test_env_matmul_precision` enforces it.

## 8. Where each change goes (function level)

- **config.py:** `NUM_CMD = 5`, `H_RANGE`, `H_SIT`, `SIT_PROB`, `SIT_S`, `WALK_S`, `UNSIT_GRACE_S`, `W_SIT` / `W_WALK`, per-joint `ACTION_SCALE` (≥ about 0.67 rad×1.5 on hip_pitch, knee and ankle), `INIT_PROBS`, and a get-up block. Recompute the obs sizes: actor 51; critic 58 + 2 + about 3 (trunk_floor, head_floor, mode_t).
- **task_math.py:**
  - `observation` / `noise_vector`: build from the layout.
  - `reward_terms`: `moving` over `[:3]`, height target from the command, mode-gated `feet_phase`, new sit/upright terms.
  - New `reward_weights`; `total_reward(…, weights)`.
  - `terminated` reads sit, grace and head contact.
  - `action_to_targets` takes a scale vector.
- **robot_model.py:** `RobotIndex(model, action_joints=…)`, with `default_action_q` indexed by name.
- **env_mjx.py:**
  - `__init__`: task spec, banks, NUM_CMD.
  - `_reset`: initial-state kinds, mode init.
  - `_update_mode` called from `_reset` and from `_step_no_reset` #11.
  - `_task_state`: add sit, mode_t and contacts.
  - :565 mode-aware termination; :567 / :584 per-env weights.
  - `merge_reset` / `_KEEP_INFO`: new counters.
  - `OpenDuckGetUpEnv` subclass: 14 actions, fallen bank, no fall termination, own episode length.
  - `wrap_for_brax_training`: take the length from the env.
- **train_mjx.py:** a `--task {walk,main,getup}` flag, meta built from params and task, `export_policy(name=)`, `expand_params` for warm starts, `--discounting` per task, an eval-env flag.
- **Kept in sync elsewhere:** env_cpu.py (shared TASK.md contract), play.py (`CMD_SLICE` → meta `obs_layout`, sit/h schedule), and the test indices listed in §5.