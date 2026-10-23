# CPU backend + tooling implementation map: walk + height + SIT (main policy) and get-up (getup.onnx)

Repo: `/Users/huseyinarslan/Desktop/adsız klasör 2/openduck`. I did not edit anything in the repo. One side effect: running the probes created `rl/__pycache__/`. That folder is gitignored and safe to delete. Its `env_mjx` .pyc came from another process, not from my runs.

Probe scripts and logs are in `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/sitstand/map-cpu/` (`probe_sit.py`, `probe_sit_dyn.py`, `probe_sit_grid.py`, `probe_fallen.py`, `test_env_cpu.log`, `test_ppo.log`).

Baseline tests on macOS (MUJOCO_GL=disabled, PYTHONDONTWRITEBYTECODE=1): `rl.tests.test_env_cpu` 22/22 pass, `rl.tests.test_ppo` 13/13 pass. The render part of `test_real_env_smoke` is skipped.

---

## 0. Measured physics constraints (these change the design)

### 0.1 SIT is not reachable inside the current action envelope

- The policy can only reach `target = q0 + 0.5·clip(a, ±1.5)`, i.e. **±0.75 rad from HOME** (`config.py:41-42`, `task_math.py:167-170`).
- Kinematic probe (symmetric legs, flat feet, level trunk): the lowest trunk-origin height inside ±0.75 rad is **0.0936 m** (dh=+0.75, dk=+0.75, da=0). HOME is **0.1193 m**.
- At 0.0936 m the trunk box bottom is still about 5 cm above the floor. The `trunk_collision` half-z is 0.042, so it only touches when `trunk_z ≤ 0.042`.
- Dynamic open-loop grid (432 combos, 3 s each): the deepest upright crouch with both feet down is `trunk_z` **0.054–0.062**, gz −0.85…−0.985, with the trunk-box corner **1–4 mm** above the floor.
  - It needs leg deltas of **[hip_pitch 2.03, knee 1.227, ankle −0.63] rad**.
  - Hip_pitch sits at its +1.571 limit and the knee at its 1.222 limit.
  - All leg joint anchors stay above the floor (lowest is 0.0236 m).
  - Peak torque is only 3–12% of 0.96 N·m, so torque is not the limit; the envelope and joint limits are.
- Any open-loop pose where the trunk touched the floor tipped over: 181 runs, 0 upright, gz ≥ −0.72.

Consequences:
- **(a)** SIT needs either `ACTION_CLIP ≥ ~4.1` at scale 0.5 (bad for walking exploration, since `init_noise_std` = 1.0) or a **command/mode-dependent nominal pose** `q0(c) = lerp(HOME, SIT_POSE, b)` with `target = q0(c) + 0.5·clip(a)`. The hook is `T.action_to_targets(np, a, default_action_q)` at `env_cpu.py:338` and `:343`; passing an `(n,10)` default already broadcasts. `b` must be a deterministic, filtered state the runtime can reproduce, and it must go into the obs and `policy_meta`.
- **(b)** A low-walk height command inside the current envelope only spans about **0.094–0.119 m**.
- **(c)** Seated `trunk_z` (~0.055) is below `TERMINATION_MIN_HEIGHT` 0.075 (`config.py:88`), so the current env terminates every sit. The gz check (−0.6) is fine.

### 0.2 Fallen poses

HOME ctrl, dropped from z = 0.15, settled for 2 s:

| start | trunk_z | gz | settle | floor contacts |
|---|---|---|---|---|
| back | 0.0482 | −0.002 | 0.38 s | head, trunk |
| front | 0.0493 | −0.192 | 1.3 s | head, both feet |
| side (either) | 0.0444 | −0.023 | 0.58 s | head + one foot |
| upside down | 0.143 | +0.910 | 0.5 s | head only (rests on its head) |

- Cost: 2 s of sim ≈ **4 ms per env** on one core.
- **Legs have no collision geoms** (only trunk, head and the two feet boxes; `openduck.xml:120,155,195,240`). On its side the trunk does not even touch the floor, and thighs and shins can pass through it. A get-up policy will exploit this.
- Recommendation: add leg capsules for the getup model and the sit model via `build_spec(..., leg_collision=True)` in `robot_model.py:18-35`, using a floor-only class (contype 0 / conaffinity 1).
- Note: feet are set to contype 2 / conaffinity 3 in `robot_model.py:27-29`, so feet do collide with each other.

---

## 1. `rl/env_cpu.py` VecEnv

### 1.1 Process structure
- `VecEnv.__init__` (`:550-598`):
  - `num_workers=0` runs one in-process `_EnvSlice(N,0,N)` over local numpy buffers (`:564-570`).
  - Otherwise the start method defaults to `forkserver` (`:510-516`, overridable with `OPENDUCK_MP_START`). A `ctx.RawArray` shared buffer is laid out by `_layout(n)` (`:34-50`): actions (n,10) f32, cmd_in (n,3) f64, obs (n,49), critic (n,58), rew, done, time_outs, ep_stats (n, 1+len(REW_KEYS)), each 64-byte aligned.
  - Contiguous slices come from `np.linspace(0,N,W+1)` (`:577`), with one `Process(_worker)` and one `Pipe` per slice (`:584-591`) and forkserver preload (`:581`).
  - `_main_hidden` (`:519-538`) stops workers from re-importing `__main__`.
- `_worker` (`:446-484`) speaks a one-byte protocol: `s` step, `r` reset_all, `c` set_commands(io["cmd_in"]), `n` clear commands, `q` quit. Replies are `k`, or `e`+traceback.
- `_run` broadcasts the command and `_wait_all` collects replies (`:625-654`). Any failure sets `_broken` (`:619-623`).
- `flags` (randomize/noise/head_random/push, `:560`) is pickled to the workers (`:586`). Add new options here.

### 1.2 Per-env state (all arrays inside `_EnvSlice`, vectorised over the slice)
- Models and data: `models` are `copy.copy(base)` per env (`:102`) with textures stripped (`:59-68`); `datas` (`:103`). RNG is `PCG64(SeedSequence(seed).spawn(num_envs)[lo:hi])` (`:104-105`), which is why worker count does not change results.
- Raw state: SD/QPOS/QVEL/QACC/AF/TZ/TIME/CONTACT (`:136-143`).
- Task state:
  - `phase` (`:146`), `step_count` (`:147`), `a_prev` (n,10) (`:148`), `air_time` (n,2) (`:149`).
  - `command` (n,3) and `fixed_cmd` (`:150-151`).
  - `head_offset`/`head_target`/`head_timer` (`:152-154`), `push_timer` (`:155`), `friction` (`:156`).
  - Delay: `delay` per env (`:157`). `abuf` is (n, K+1, 10) and `ptr` is a **single shared ring pointer** (`:158-159`). K = max delay (`:126`); currently `action_delay_steps=(0,0)` (`config.py:112`), so the delay path at `:339` is off.
  - `ep_sum` (n, nrew) (`:160`).
- Per-step uniforms `U`: [obs noise (NUM_OBS) | head 6 | push 3]; offsets `o_noise/o_head/o_push` (`:128-133`). They are drawn once per step (`:333-335`), and `test_head_push_noise_exact` reproduces them exactly.
- Step order (`:330-442`) follows the TASK.md step list:
  - action → targets plus optional delay (`:338-343`)
  - head filter (`:348-358`), then legs and clip (`:359-360`)
  - push into `qvel[0:2]` (`:363-372`)
  - per-env `mj_step(m,d,4)` loop and gather (`:378-393`)
  - phase advance (`:396`); bad-physics detection, counted as a fall (`:400-404, 413-416`)
  - `reward_terms` / `terminated` / `total_reward` (`:407-419`)
  - air_time and a_prev updates (`:422-423`)
  - command resample every `CMD_STEPS`=300 using `_sample_command(rng)` with `rng.random(4)` (`:426-428`, `:171-175`)
  - done → `ep_stats` + immediate `_reset_env` (`:431-439`), then obs (`:442`).

### 1.3 Reset and domain randomization
- `_reset_env(j)` (`:251-291`):
  1. `_randomize(j)` (`:201-238`): one `rng.random(len(dr_lo))` draw from the flattened lo/span built by `_setup_dr` (`:185-199`), always applied from the NOMINAL values in `self.nom` (`:118-120`), followed by `mj_setConst`.
  2. Exactly **`n_reset_u = 14+10 = 24`** uniforms (`:166`; layout comment `:255`): joint noise 14, yaw, base vel 2, phase, command 4, head timer, push timer.
  3. `mj_resetData`, then HOME qpos + noise, yaw quaternion, base velocity, `ctrl = init_ctrl`, `mj_forward`, `_gather`, contacts.
- `reset_all` (`:317-327`) adds per-env noise draws.

### 1.4 `set_commands()`
- `VecEnv.set_commands(cmd)` (`:678-689`) reshapes to (N,3) f64.
  - In-process: calls `core.set_commands`.
  - With workers: writes `io["cmd_in"]` and sends `c`, or `n` for None.
- `_EnvSlice.set_commands` (`:177-182`) sets `fixed_cmd` and overwrites `command`.
  - The lock survives resets (`:275-276`) and disables resampling (`:426`).
  - It does **not** refresh obs, which is why `play.py:241` patches `obs[:, CMD_SLICE]` by hand.
- The constructor's `fixed_command` broadcasts to (N,3) (`:597-598`).

### 1.5 Where each new piece goes (function level)

| Change | Exact place | Recommendation |
|---|---|---|
| Command width 3→5 `[vx,vy,wz,height,sit]` | `_layout` `:37`; `__init__` `:150,164-165`; `_sample_command` `:171-175`; `_EnvSlice.set_commands` `:181`; `_reset_env` `:274-280` + `n_reset_u` `:166` + comment `:255`; `VecEnv.set_commands` `:683,688`; `__init__` `:598` | Add `C.COMMAND_NAMES` and `C.NUM_COMMANDS`, and build `cmd_lo/hi` from the names. **Accept (N,3) and pad** height=nominal, sit=0 so play `--command`, `test_set_commands` and `test_matches_reference_step` keep working. With sit=1, force `command[:, :3]=0`. |
| Sit/height timers inside an episode | step 11 `:425-428` | Add per-env `sit_timer`/`height_timer` (with fields reset in `_reset_env`). Draw from `self.rngs[j]` the way `_sample_command` does (deterministic per env). Or extend `U` with an `o_cmd` block and update `test_head_push_noise_exact`. Sample sit hold times of a few seconds; clearing sit means "stand up". |
| Mode-dependent nominal pose (needed, §0.1) | `:338`, `:343` | Pass `q0 = lerp(default_action_q, sit_q, b)`, shape (n,10). Keep `b` as a filtered per-env state (e.g. `sit_level += (sit−sit_level)·dt/τ`) updated in step 11. Put `b` in the obs and in the meta. |
| Seated start | `_reset_env` between `:258` (`mj_resetData`) and `:267` (`d.qpos[:]=`) | Branch on `u_mode < p_seated`: `qpos = idx.sit_qpos` (new RobotIndex field from a config `SIT_POSE`, with base z solved so the soles and trunk box rest on the floor), `ctrl = sit ctrl`, command sit=1, `sit_level=1`. Bump `n_reset_u`. New flag `seated_start=0.0` in `VecEnv.__init__` (`:550`) and `flags` (`:560`). **Default 0** keeps `test_reset_state` valid. |
| Fallen start (getup task) | new `_EnvSlice._build_fall_bank(K)` called after `:168`; branch in `_reset_env` like the seated one | Pre-settle a **bank** of K≈256 states at init: random orientation (back/front/side/random) plus random joints, 1.5 s with HOME ctrl, about 0.8 s per worker. Seed it from `np.random.default_rng([seed, tag])`, not per-slice, so every worker builds the identical bank and `inprocess == workers` still holds. Per reset: index from u, yaw rotation, joint noise, qvel = 0, `mj_forward`. Settling inside every reset would add roughly 60% to physics cost. |
| Mode-dependent termination | `:411` (`term = T.terminated(np, s)`), `:416` | Put the logic in `task_math.terminated` (shared with MJX): `min_h = where(sit_level>0.5, SIT_MIN_HEIGHT≈0.035, 0.075)`, keeping the gz test. Add an env flag `terminate=True`; when False, `term = bad` only (getup training and the play get-up demo). |
| Getup actions = 14 (legs+head) | `_layout` `:36`; `a_prev` `:148`; `abuf` `:158`; `:338-343` default `idx.default_obs_q` (14); head block `:348-358` skipped; `:359` writes the `action_act_id+head_act_id` ctrl; `VecEnv.step` check `:663`; class attributes `:545-548` | Make `num_obs/num_critic_obs/num_actions/reward_keys` **instance** attributes driven by `task=`. `_layout(n, task)` must be called identically in the parent (`:566,574`) and the worker (`:450`). |
| Per-task reward keys | module-level `REW_KEYS` `:27` used at `:43,167,412,673-675` | Move these to the task spec. |
| Episode length | `:417` uses global `MAX_EPISODE_STEPS` | Add `max_episode_steps=` (getup ~6–8 s; play `None`). |
| Obs width | `:129` (`C.NUM_OBS` noise uniforms), `:313` | Fine once `C.NUM_OBS` and `T.noise_vector` agree (see §5). |

Task spec location: put the spec (name, dims, action joints, obs/critic/reward/terminate functions, reset probabilities) in a **flat** module such as `rl/tasks.py`. `tools/runpod/launch.py:327-345` `rl_imports` only resolves `rl/<name>.py` files, so a subpackage would be missing from the inline bundle.

---

## 2. `rl/play.py`

### 2.1 How it drives commands
- Scripted list `DEMO` (`:29-35`): stand 2 s, walk forward (0.15,0,0) 5 s, turn left wz 0.5 for 4 s, walk backward −0.08 for 4 s, sidestep vy 0.08 for 4 s; 19 s total.
- `build_schedule` (`:187-199`) truncates by `--seconds`. `--command` takes nargs=3 (`:174`); the default is (0.15,0,0) for 12 s.
- Env: `VecEnv(1, num_workers=0, randomize/noise/head_random/push=False, fixed_command=sched[0][1])` (`:213-214`).
- At segment boundaries it calls `env.set_commands(cmd[None])` and patches `obs[:, CMD_SLICE]` (`:236-241`), with `CMD_SLICE = slice(NUM_OBS-5, NUM_OBS-2)` (`:28`). That slice assumes the command is 3 wide and directly precedes the 2 phase features.
- On a fall the env auto-resets to HOME, `falls++`, and the camera `snap()`s (`:246-254`). Schedules over 20 s only print a note (`:211-212`); the env still times out at 1000 steps.

### 2.2 Camera
- `Recorder` (`:89-166`) renders `build_model(visual=True)` (same qpos layout, checked in `test_real_env_smoke`).
- Free camera: distance 2·extent, azimuth 135°, elevation −15°. Lookat follows trunk xy with smoothing 0.15 per frame and fixed z 0.22 (`:129-135`).
- Output: 30 fps mp4 (libx264) and 15 fps / 400 px GIF. MUJOCO_GL defaults to egl when rendering (`:205`); on macOS use cgl or glfw.

### 2.3 Metrics per segment
- Commanded vs achieved (vx, vy, wz), from base position deltas rotated into the yaw frame plus the yaw delta (`:256-263`); falls, timeouts, net displacement, path length; totals; optional `--json` (`:275-293`).

### 2.4 Obs and policy loading
- `play.py` does **not** build obs. It uses the env obs (`T.observation`, shared), and its only layout knowledge is `CMD_SLICE`.
- `OnnxPolicy` (`:43-56`) reads `<stem>_meta.json` and raises if `obs_dim != C.NUM_OBS` or `action_dim != C.NUM_ACTIONS` (`:52`).
- `TorchPolicy` loads a `.pt` checkpoint (`:59-69`). `ZeroPolicy` returns `C.NUM_ACTIONS` zeros (`:40`).

### 2.5 What breaks with a 5-dim command
- Format strings `"%s cmd %+.2f %+.2f %+.2f"` at `:232`, `:267` and the table at `:275,280-281` take `*cmd`; 5 values raise TypeError.
- `CMD_SLICE` (`:28`) would be wrong.

### 2.6 Recommendations
- Derive `CMD_SLICE` from `C.OBS_SLICES["command"]`. Better: add `VecEnv.set_commands(cmd, refresh_obs=True)` that re-runs `_write_obs` and returns obs, which removes the patch.
- Change `--demo` to `nargs="?", const="walk", choices=["walk","sit","getup"]` (bare `--demo` stays valid; `test_real_env_smoke` relies on it). Keep `DEMOS = {"walk": DEMO, "sit": [...], "getup": [...]}` with 5-tuple commands, padding 3-tuples.
- Sit demo, about 19 s: stand 2 → walk 0.15 for 4 → crouch-walk (0.10, h_low) 4 → sit (sit=1) 5 → stand (sit=0) 2.5 → walk 1.5. Or allow longer with `max_episode_steps=None`.
- New sit metrics: `t_to_seated` (first trunk_collision–floor contact after sit=1), `seated_frac`, `trunk_z` mean/min per segment, `t_to_stand` (trunk_z > 0.11 and both feet down after sit=0), height tracking RMSE, peak |τ|/0.96.
- Get-up demo:
  - Env with `terminate=False`. Tip the robot over through `env.mj_data(0)` (set `qpos[3:7]` / add `qvel[3:6]`, then `mj_forward`, as the tests do).
  - `Supervisor` state machine: `fallen(s)` (same thresholds as `T.terminated`: gz > −0.6 or trunk_z < 0.075) → getup policy; `recovered(s)` (gz < −0.9, trunk_z > 0.105, both feet in contact for 25 steps) → main policy with command 0, plus a new `VecEnv.reset_action_history()` (a_prev = 0, abuf = 0).
  - Build the getup obs from a new `VecEnv.task_state()` (num_workers=0 only; returns `core._state(a_prev, a_prev)`) through a `task_math.getup_observation`, so play never hard-codes layouts.
  - The env needs a per-env `head_from_policy` flag with a 14-wide action buffer: the main policy pads head actions with zeros (= HOME), the getup policy fills them.
  - Metrics: `t_to_upright`, success, handoff step, falls within 2 s after handoff.
- CLI: `--getup-policy path.onnx`. Validate each meta against its own role's dims, not `C.NUM_OBS`.

---

## 3. `rl/export_onnx.py` and `policy_meta`

- `actor_to_onnx` (`:47-79`), `check_onnx` (`:82-102`) and `export_policy` (`:136-163`) are **dimension-generic**: they use `ac.arch["num_obs"/"num_actions"]`, input "obs" and output "action".
  - `meta_path_for("…/getup.onnx")` already gives `getup_meta.json` (`:105-106`), so a 14-action getup export works unchanged.
- `policy_meta` (`:109-127`) hard-codes: `obs_dim=C.NUM_OBS`, `action_dim=C.NUM_ACTIONS`, `default_leg_q=idx.default_action_q` (10), `leg_joints`, `obs_joints` (14), `default_obs_q` (14), `gait_freq`, `obs_scales`, `ctrl_dt`, `action_scale`, `action_clip`, `backend`, `steps`.
  - It has no command or obs layout, which the robot runtime would need for height/sit.
- `main()` (`:174-184`) defaults to `policy.onnx` and takes `backend` from the checkpoint. The checkpoint stores no task: `Runner.save` (`train.py:221-222`) only stores `run_info`.
- `train_mjx.policy_meta` (`train_mjx.py:154-160`) is a **duplicate** builder. `export_onnx` imports torch at module top (`:20`), and MJX pods have no torch (`pod_entry.sh:31`), so the shared builder must move to a torch-free flat module (e.g. `rl/policy_meta.py`) used by both trainers.
- Recommendation: `policy_meta(steps, backend, idx, task)` adds:
  - `task` ("walk"/"getup"), `obs_layout` [[name,size],…], `command_names`, `command_ranges`
  - `action_joints` and `default_action_q` (14 for getup); keep `default_leg_q`/`leg_joints` for back-compat
  - `head_control` ("external"/"policy"), `sit_pose` and the `pose_blend` rule (for §0.1), `fallen`/`recovered` thresholds (from config, shared with the env reward and play)
  - `version`
- Store `task` in the checkpoint (`train.py:221`) so `export_onnx.main` can pick the right meta.
- File naming: `Runner.save` hard-codes `policy.onnx` (`train.py:235`), and the tools expect it (`launch.py:784,802,902`; `pod_entry.sh:209,616`). Recommendation: keep `policy.onnx`/`policy_meta.json` inside run dirs (no tools changes), mark the role through `meta["task"]`, and rename to `getup.onnx` only at bundle/deploy time.
- Warm start: `Runner` rejects an obs-size mismatch (`train.py:100-103`). Add `--init-from CKPT` weight surgery:
  - Insert zero columns into `actor[0].weight` and `critic[0].weight` at the command insert index.
  - Extend `actor/critic_obs_normalizer` `mean`/`var`/`std` (`ppo.py:51-54`) with analytic values for the new dims.
  - The shared scalar `count` (about 1e8 after training) would otherwise freeze adaptation of the new dims (`ppo.py:66-73`).
  - Reset the optimizer.

---

## 4. Tests

### 4.1 Current coverage
- **test_env_cpu.py**:
  - `shapes_dtypes` (literal 49/58, `:50-51,57`), `reproducible_seed`, `standing_zero_action` (5 s at 0 action; uses I_TRUNK_Z, I_GRAV_Z, I_CONTACT), `auto_reset_on_fall` (I_TRUNK_Z, I_LAST_ACT 34:44).
  - `time_outs` (episode keyset == REWARD_WEIGHTS), DR ranges / no drift (I_FRICTION), `no_randomization_is_nominal` (I_FRICTION), `action_delay`, `phase_advance` (I_PHASE 47:49), `contacts` (I_CONTACT).
  - `inprocess_equals_workers`, `set_commands` ((3,3) commands, I_CMD 44:47), `random_commands_resampled`, `head_and_push`, `close_and_gc`.
  - `matches_reference_step` (`_ref_state` `:373-382` builds a 3-dim command, `np.zeros(10)`, `default_action_q + 0.5·a`), `reset_state` (exact HOME reset), `head_push_noise_exact` (U layout; ctrl from `default_action_q`; `set_commands(zeros((N,3)))`).
  - `dr_set_const`, `worker_death`, `worker_error_cleanup`, `sigint_cleanup` (literal `(16,10)` at `:620`). Index constants are at `:24-25`.
- **test_ppo.py**: GAE and PPO math on a PointMass env (generic).
  - `onnx_export_matches_torch` asserts the meta keyset and literal `obs_dim==49`, `action_dim==10` (`:290-294`).
  - `export_validates_before_replace` asserts the dir contents are exactly `policy.onnx`/`policy_meta.json`.
  - `real_env_smoke`: 1 training iteration on the real env, then `play.run`; the render part asserts demo segment names `["stand","walk forward"]` (`:480`).
- **test_env_mjx.py**: I_TRUNK_Z 52, I_CONTACT 53:55, I_FRICTION 57 (`:26`); `obs[:,44:47]` (`:215,223`); `fixed_command=(0,0,0)` with reshape(3) in env_mjx `:248`; `set_command` 3-wide (`:199`); meta keyset (`:382-383`).

### 4.2 What breaks if obs grows 49→51 (command 5 wide at [44:49], phase moves to [49:51])

| Breaks | Why |
|---|---|
| `test_shapes_dtypes` | literal 49/58 |
| `test_standing_zero_action`, `test_auto_reset_on_fall`, `test_contacts` | critic index 52 → 54 |
| DR ranges test, `test_no_randomization_is_nominal` | I_FRICTION 57 → 59 |
| `test_phase_advance` | 47:49 now reads height/sit |
| `test_matches_reference_step` | `_ref_state` command is 3 wide; targets change if the pose blend is added |
| `test_head_push_noise_exact` | only if U layout or action mapping changes |
| `test_onnx_export_matches_torch` (ppo) | literals and keyset |
| mjx `:26`, keyset, 3-wide command/`fixed_command` | same reasons on the MJX side |

- `test_random_commands_resampled` breaks if sit toggles zero vx..wz mid-interval.
- `test_reset_state` breaks if seated/fallen start probabilities are non-zero by default.
- I_CMD 44:47 and I_LAST_ACT 34:44 stay valid.

### 4.3 New tests to add (names and assertions)
- `test_obs_layout_constants`: `C.NUM_OBS==51`, `NUM_CRITIC_OBS==60`, sum(OBS_LAYOUT sizes)==NUM_OBS, `len(T.noise_vector(np))==NUM_OBS`, `C.OBS_SLICES["command"]==slice(44,49)`, `play.CMD_SLICE` is the same slice.
- `test_set_commands_5d_and_3d_padding` (workers 0 and 2): a (N,3) input gives `obs[:,44:49]==[vx,vy,wz,h_nom,0]`; a (N,5) input is exact; the lock survives a fall reset.
- `test_sit_zeroes_velocity_and_freezes_resample`.
- `test_height_command_sampling`: values within range; own timer; locked by `fixed_cmd`.
- `test_sit_termination_relaxed`: upright with trunk_z = 0.05 and sit=1 → not done; sit=0 → done; sit=1 with a 90° tilt → done.
- `test_seated_start_reset` (`seated_start=1.0`): trunk box–floor contact or `|trunk_z − SIT_Z| < 5e-3`, both feet down, gz < −0.85, sit=1, 2 s of zero action without done.
- `test_fall_bank_reset` (task getup): over 64 resets, > 90% have gz > −0.6; bank identical across workers 0, 2 and 3 (`_same` rollouts); `|qvel|` small; no done for 100 steps with `terminate=False`.
- `test_getup_action_dim_14`: `env.num_actions==14`; head ctrl `== clip(default_obs_q[10:] + 0.5·clip(a[:,10:]))`; head_random ignored; (N,10) raises ValueError.
- `test_mode_pose_blend`: leg targets equal `lerp(HOME, SIT)` at `sit_level` 0 and 1 with a=0; the filter is exact against a reference.
- Update `test_matches_reference_step` and `test_head_push_noise_exact` for the new state and U blocks.
- ppo: `test_getup_export_meta` (14-action export to `getup.onnx` writes `getup_meta.json` with task "getup", `len(action_joints)==14`); `test_meta_role_mismatch_raises` (a 49-dim meta against a 51-dim env → ValueError).
- ppo: `test_play_sit_demo_smoke` (ZeroPolicy, `--demo sit --seconds 3`: segment names plus new metric keys, no format crash); `test_play_getup_supervisor` (unit test of `fallen`/`recovered`, and the handoff resets a_prev).
- ppo: `test_obs_expand_warmstart` (expanded network on old obs with new dims = 0 gives identical actions; normalizer extended).

---

## 5. Every hard-coded 49 / 58 / 10 / command layout (whole repo)

| File:line | Hard-code |
|---|---|
| `rl/config.py:48-49` | `NUM_OBS = 3+3+14+14+NUM_ACTIONS+3+2` (the `+3` is the command), `NUM_CRITIC_OBS` |
| `rl/config.py:52-54` | `COMMAND_RANGES` vx/vy/wz, ZERO_PROB, RESAMPLE |
| `rl/config.py:66` | `BASE_HEIGHT_TARGET 0.115` (fixed; needs to become the commanded height) |
| `rl/config.py:88-89` | termination thresholds (not mode-aware) |
| `rl/task_math.py:94-95` | `noise_vector` `[0.0]*(NUM_ACTIONS+3+2)`. **Will crash** with shape (49,) vs (51,) at `env_cpu.py:313` if not updated. |
| `rl/task_math.py:117-119` | `cmd[...,:2]`, `cmd[...,2]`, and `moving = sum(cmd²)` over **all** dims: height/sit would silently force moving=1. Must slice `[:3]`. |
| `rl/task_math.py:139,163,167-170` | base_height target, terminated, action_to_targets |
| `rl/task_math.py:15-22` | docstring dims |
| `rl/env_cpu.py` | `:37` cmd_in (n,3); `:150`; `:164-166` (names, `n_reset_u=14+10`); `:172` `random(4)`; `:181`, `:683` reshape(·,3); `:255,277-280` u layout; `:598` broadcast (N,3); `:36,148,158,663` NUM_ACTIONS |
| `rl/env_mjx.py` | `:10` docstring (49)/(58); `:248` reshape(3); `:270-271`; `:408`; `:449` split into 10 keys (new samples need keys); `:474,489` |
| `rl/train_mjx.py:154-160,193` | duplicate meta, critic zeros |
| `rl/play.py` | `:28` CMD_SLICE; `:29-35` DEMO 3-tuples; `:40`; `:52`; `:174` nargs=3; `:198`; `:232,267,275,280-281` 3-wide formats |
| `rl/export_onnx.py:5-6,114-121` | docstring and meta |
| `rl/robot_model.py:75` | `default_action_q = default_obs_q[:NUM_ACTIONS]` (fine for 14 only because `OBS_JOINTS = LEG + HEAD`) |
| `rl/tests/test_env_cpu.py` | `:24-25,50-51,56-57,280-282,308-309,373-382,399,407,411,438,478,487,620` |
| `rl/tests/test_ppo.py:290-294,480` | meta literals and keyset, demo segment names |
| `rl/tests/test_env_mjx.py:26,33,79,199,210,215,223,382-383` | indices, 3-wide command, keyset |
| `rl/TASK.md:93-94,99-102,109-111` | layouts and API, plus the Commands/Reset sections and "action (10,)" in the step list |
| `README.md:107,122` | "49 boyutlu", `obs [N,49]` / `action [N,10]` |
| `tools/runpod/launch.py:57,784,802,902` | `TRAIN_MODULE`, `policy.onnx`/`policy_meta.json` names |
| `tools/runpod/pod_entry.sh:209,616` | same names |
| `tools/runpod/launch.py:327-345` | flat-module-only import closure |
| `sim/stand_test.py` | no dims; `crouch_targets()` (`:30-54`) is a reusable bisection helper for building `SIT_POSE` |

Other modules:
- `rl/bench_env.py` is generic (uses `env.num_actions`, `:26`). Add `--task`, `--seated-start`, `--fallen-start` to measure reset and bank cost.
- `rl/train.py` `Runner` is generic (`:93-99`). Add `--task`, `--seated-start`, `--fallen-start` and `--init-from` to `parse_args` (`:246-270`) and pass them through in `main` (`:290-297`).