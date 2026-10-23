# -*- coding: utf-8 -*-
"""
rl/task_math.py, rl/tasks.py, rl/policy_meta.py ve config sabitlerinin testleri (S1 sozlesmesi).
Calistirma (repo kokunden):

  python3 -m rl.tests.test_task_math          # duz assert'ler
  python3 -m pytest rl/tests/test_task_math.py

jax kuruluysa numpy-jax esitligi de denenir (yoksa o testler atlanir).
"""
import json
import sys
import time

import mujoco
import numpy as np

from rl import config as C
from rl import policy_meta as PM
from rl import task_math as T
from rl import tasks as TK
from rl.robot_model import RobotIndex, build_model

try:
    import jax
    import jax.numpy as jnp
except ImportError:          # CPU venv'inde jax yok
    jax = jnp = None


class _Skip(Exception):
    pass


_STANDALONE = [False]


def _skip(msg):
    if not _STANDALONE[0]:
        try:
            import pytest
        except ImportError:
            pass
        else:
            pytest.skip(msg)
    raise _Skip(msg)


_IDX = {}


def _idx(variant="walk", action_joints=None):
    k = (variant, tuple(action_joints or ()))
    if k not in _IDX:
        _IDX[k] = RobotIndex(build_model(variant=variant), action_joints)
    return _IDX[k]


HOME = None


def _home():
    global HOME
    if HOME is None:
        HOME = _idx().default_obs_q.copy()
    return HOME


# --- eski (HEAD) walk formullerinin dondurulmus kopyasi: walk yolu bit-duzeyinde ayni kalmali -------
def _old_observation(s):
    g = T.projected_gravity(np, s["quat"])
    obs = np.concatenate([g, s["gyro"] * C.OBS_SCALES["gyro"], s["q"] - s["default_q"],
                          s["qd"] * C.OBS_SCALES["joint_vel"], s["last_action"], s["command"],
                          T.phase_features(np, s["phase"])], axis=-1)
    return np.clip(obs, -C.OBS_CLIP, C.OBS_CLIP)


def _old_noise():
    n = C.OBS_NOISE
    parts = ([n["gravity"]] * 3 + [n["gyro"] * C.OBS_SCALES["gyro"]] * 3 + [n["joint_pos"]] * 14
             + [n["joint_vel"] * C.OBS_SCALES["joint_vel"]] * 14 + [0.0] * (C.NUM_ACTIONS + 3 + 2))
    return np.asarray(parts)


def _old_critic(s):
    return np.concatenate([_old_observation(s), s["linvel"], s["trunk_z"][..., None], s["contact"],
                           s["foot_pos"][..., 2], s["friction"][..., None]], axis=-1)


def _old_terms(s):
    xp = np
    cmd = s["command"]
    g = T.projected_gravity(xp, s["quat"])
    lin_err = xp.sum(xp.square(cmd[..., :2] - s["linvel"][..., :2]), axis=-1)
    ang_err = xp.square(cmd[..., 2] - s["gyro"][..., 2])
    moving = xp.where(xp.sum(xp.square(cmd), axis=-1) > 0.05 ** 2, 1.0, 0.0)
    foot_z = s["foot_pos"][..., 2]
    feet_phase = xp.exp(-xp.sum(xp.square(foot_z - T.desired_foot_height(xp, s["phase"])), axis=-1)
                        / C.FOOT_HEIGHT_SIGMA)
    slip = xp.sum(s["contact"] * xp.sum(xp.square(s["foot_vel"][..., :2]), axis=-1), axis=-1)
    yaw = T.yaw_angle(xp, s["quat"])
    d = s["foot_pos"][..., 0, :] - s["foot_pos"][..., 1, :]
    lateral = -xp.sin(yaw) * d[..., 0] + xp.cos(yaw) * d[..., 1]
    hip_idx = [C.OBS_JOINTS.index(n) for n in C.HIP_POSE_JOINTS]
    dq = s["q"] - s["default_q"]
    hip = xp.sum(xp.square(xp.stack([dq[..., i] for i in hip_idx], axis=-1)), axis=-1)
    return {
        "tracking_lin_vel": xp.exp(-lin_err / C.TRACKING_SIGMA_LIN),
        "tracking_ang_vel": xp.exp(-ang_err / C.TRACKING_SIGMA_ANG),
        "lin_vel_z": xp.square(s["linvel"][..., 2]),
        "ang_vel_xy": xp.sum(xp.square(s["gyro"][..., :2]), axis=-1),
        "orientation": xp.sum(xp.square(g[..., :2]), axis=-1),
        "base_height": xp.square(s["trunk_z"] - C.BASE_HEIGHT_TARGET),
        "torques": xp.sum(xp.square(s["torque"]), axis=-1),
        "joint_acc": xp.sum(xp.square(s["qdd_legs"]), axis=-1),
        "action_rate": xp.sum(xp.square(s["action"] - s["last_action"]), axis=-1),
        "feet_air_time": moving * xp.sum((s["air_time"] - C.AIR_TIME_TARGET) * s["first_contact"], axis=-1),
        "feet_phase": feet_phase,
        "feet_slip": slip,
        "feet_distance": xp.maximum(C.FEET_MIN_DISTANCE - lateral, 0.0),
        "hip_pose": hip,
        "alive": xp.ones_like(s["trunk_z"]),
    }


def _old_total(terms, term):
    r = 0.0
    for k, w in C.REWARD_WEIGHTS.items():
        r = r + w * terms[k]
    return r * C.CTRL_DT + C.TERMINATION_PENALTY * term


def _old_terminated(s):
    g = T.projected_gravity(np, s["quat"])
    fell = (s["trunk_z"] < C.TERMINATION_MIN_HEIGHT) | (g[..., 2] > C.TERMINATION_MAX_GRAVITY_Z)
    return np.where(fell, 1.0, 0.0)


def _old_meta(steps, backend, idx):
    return {
        "obs_dim": C.NUM_OBS, "action_dim": C.NUM_ACTIONS, "ctrl_dt": C.CTRL_DT,
        "action_scale": C.ACTION_SCALE, "action_clip": C.ACTION_CLIP,
        "default_leg_q": [float(v) for v in idx.default_action_q], "leg_joints": list(C.LEG_JOINTS),
        "obs_joints": list(C.OBS_JOINTS), "default_obs_q": [float(v) for v in idx.default_obs_q],
        "gait_freq": C.GAIT_FREQ, "obs_scales": dict(C.OBS_SCALES), "backend": backend, "steps": int(steps),
    }


# --- rastgele durumlar ------------------------------------------------------------------------
def _rand_quat(rng, n, tilt=1.0):
    q = np.concatenate([np.ones((n, 1)), tilt * rng.standard_normal((n, 3))], axis=1)
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def _rand_state(rng, n, task="walk"):
    t = TK.get_task(task)
    A = t.num_actions
    s = {
        "quat": _rand_quat(rng, n, 0.6), "gyro": rng.standard_normal((n, 3)),
        "linvel": 0.3 * rng.standard_normal((n, 3)),
        "trunk_z": rng.uniform(0.03, 0.14, n), "q": _home() + 0.4 * rng.standard_normal((n, 14)),
        "qd": 3.0 * rng.standard_normal((n, 14)), "qdd_legs": 100.0 * rng.standard_normal((n, 10)),
        "torque": rng.uniform(-0.96, 0.96, (n, 14)), "foot_pos": 0.05 * rng.standard_normal((n, 2, 3)),
        "foot_vel": 0.2 * rng.standard_normal((n, 2, 3)), "contact": (rng.random((n, 2)) < 0.5).astype(float),
        "action": rng.uniform(-1.5, 1.5, (n, A)), "last_action": rng.uniform(-1.5, 1.5, (n, A)),
        "phase": rng.random(n), "air_time": rng.uniform(0, 0.4, (n, 2)),
        "first_contact": (rng.random((n, 2)) < 0.3).astype(float), "friction": rng.uniform(0.6, 1.3, n),
        "default_q": _home(),
    }
    if t.name == "walk":
        s["command"] = rng.uniform(-0.3, 0.3, (n, 3)) * (rng.random((n, 1)) < 0.8)
    if t.name == "main":
        vel = rng.uniform(-0.3, 0.3, (n, 3)) * (rng.random((n, 1)) < 0.8)
        h = rng.uniform(0.06, 0.12, n)
        sit = (rng.random(n) < 0.3).astype(float)
        s["command"] = np.concatenate([vel, h[:, None], sit[:, None]], axis=1)
        s["h_ref"] = rng.uniform(C.H_MIN_STAND, C.H_HOME, n)
        s["alpha"] = np.where(rng.random(n) < 0.2, 1.0, np.where(rng.random(n) < 0.3, 0.0, rng.random(n)))
    if t.name in ("main", "getup"):
        s["trunk_floor"] = (rng.random(n) < 0.4).astype(float)
        s["head_floor"] = (rng.random(n) < 0.3).astype(float)
    if t.name == "getup":
        s["head_vel"] = rng.standard_normal((n, 3))
        s["head_floor_prev"] = (rng.random(n) < 0.3).astype(float)
        s["phi_up_prev"] = rng.uniform(-1, 1, n)
        s["phi_h_prev"] = rng.uniform(0.03, C.H_HOME, n)
    return s


# --- duzen ve sabitler -------------------------------------------------------------------------
def test_task_layouts():
    w, m, g = TK.get_task("walk"), TK.get_task("main"), TK.get_task("getup")
    assert (w.num_obs, w.num_critic_obs, w.num_actions) == (C.NUM_OBS, C.NUM_CRITIC_OBS, C.NUM_ACTIONS) == (49, 58, 10)
    assert (m.num_obs, m.num_critic_obs, m.num_actions, m.num_commands) == (53, 64, 10, 5)
    assert (g.num_obs, g.num_critic_obs, g.num_actions, g.num_commands) == (48, 57, 14, 0)
    assert m.obs_slices["command"] == slice(44, 49) and m.obs_slices["posture"] == slice(49, 51)
    assert m.obs_slices["phase"] == slice(51, 53) and w.obs_slices["command"] == slice(44, 47)
    assert m.critic_slices["trunk_floor"] == slice(62, 63) and m.critic_slices["head_floor"] == slice(63, 64)
    assert g.critic_slices["trunk_z"] == slice(51, 52) and g.critic_slices["friction"] == slice(56, 57)
    assert (w.max_episode_steps, m.max_episode_steps, g.max_episode_steps) == (1000, 1000, 400)
    assert w.reward_weights == C.REWARD_WEIGHTS and not w.reward_late
    assert "base_height" not in m.reward_weights and set(g.reward_late) <= set(g.reward_weights)
    assert abs(sum(m.segment_probs) - 1) < 1e-12 and m.segment_names == ("walk", "stand", "crouch", "sit")
    for t in (w, m, g):
        assert abs(sum(p for _, p in t.reset_probs) - 1) < 1e-12
        assert TK.get_task(t.name) is t and TK.get_task(t) is t
    assert g.action_joints == tuple(C.LEG_JOINTS + C.HEAD_JOINTS) and g.gamma == 0.99 and w.gamma is None
    try:
        TK.get_task("run")
        raise AssertionError
    except ValueError:
        pass


def test_noise_vector_lengths():
    assert np.array_equal(T.noise_vector(np), _old_noise())
    for name in TK.TASK_NAMES:
        t = TK.get_task(name)
        nv = np.asarray(T.noise_vector(np, name))
        assert nv.shape == (t.num_obs,), (name, nv.shape)
        sl = t.obs_slices
        assert np.all(nv[:sl["joint_vel"].stop] > 0)
        assert np.all(nv[sl["last_action"].start:] == 0.0)
    n = np.asarray(T.noise_vector(np, "main"))
    w = np.asarray(T.noise_vector(np))
    assert np.array_equal(n[:34], w[:34])


def test_pad_command():
    c = TK.pad_command("main", [[0.1, 0.0, 0.2]])
    assert np.array_equal(c, [[0.1, 0.0, 0.2, C.H_HOME, 0.0]])
    c5 = np.array([[0.0, 0.0, 0.0, 0.09, 1.0]])
    assert np.array_equal(TK.pad_command("main", c5), c5)
    assert np.array_equal(TK.pad_command("walk", [0.1, 0, 0]), [0.1, 0, 0])
    for task, cmd in (("walk", [0, 0, 0, 0.1, 0]), ("main", [0, 0, 0, 1]), ("getup", [0, 0, 0])):
        try:
            TK.pad_command(task, cmd)
            raise AssertionError(task)
        except ValueError:
            pass


# --- walk regresyonu (eski formullerle bit-duzeyinde ayni) ------------------------------------
def test_walk_regression():
    rng = np.random.default_rng(0)
    for n in (1, 7, 64):
        s = _rand_state(rng, n)
        assert np.array_equal(T.observation(np, s), _old_observation(s))
        assert np.array_equal(T.observation(np, s, "walk"), _old_observation(s))
        assert np.array_equal(T.critic_observation(np, s), _old_critic(s))
        new, old = T.reward_terms(np, s), _old_terms(s)
        assert list(new) == list(old) == list(C.REWARD_WEIGHTS)
        for k in old:
            assert np.array_equal(new[k], old[k]), k
        term = _old_terminated(s)
        assert np.array_equal(T.terminated(np, s), term) and np.array_equal(T.terminated(np, s, "walk"), term)
        assert np.array_equal(T.total_reward(np, new, term), _old_total(old, term))
        assert np.array_equal(T.total_reward(np, new, term, T.reward_weights(np, "walk")), _old_total(old, term))
        a = rng.uniform(-3, 3, (n, 10))
        q0 = _idx().default_action_q
        t, ac = T.action_to_targets(np, a, q0)
        ac_old = np.clip(a, -C.ACTION_CLIP, C.ACTION_CLIP)
        assert np.array_equal(ac, ac_old) and np.array_equal(t, q0 + C.ACTION_SCALE * ac_old)
        sc = TK.action_scale_vector("walk", q0, None, None)
        t2, _ = T.action_to_targets(np, a, q0, scale=sc, clip=C.ACTION_CLIP)
        assert np.array_equal(t2, t)


def test_walk_index_unchanged():
    idx = _idx()
    assert np.array_equal(idx.default_action_q, idx.default_obs_q[:C.NUM_ACTIONS])
    assert idx.action_joints == list(C.ACTION_JOINTS) and idx.variant == "walk" and idx.s_head_vel is None
    assert len(idx.head_geoms) == 1 and len(idx.extra_geoms) == 0


def test_walk_meta_unchanged():
    idx = _idx()
    for backend in ("cpu_torch", "mjx_brax"):
        new, old = PM.build_meta(123, backend, idx), _old_meta(123, backend, idx)
        assert list(new) == list(old) and new == old
        assert json.dumps(new) == json.dumps(old)
    assert PM.build_meta(5, "cpu_torch") == _old_meta(5, "cpu_torch", idx)


# --- main: komut, filtre, referans durus ---------------------------------------------------------
def test_command_obs_and_gating():
    c = np.array([[0.2, -0.1, 0.5, C.H_HOME, 0.0],     # yurume
                  [0.2, -0.1, 0.5, C.H_HOME, 1.0],     # sit -> hiz 0
                  [0.2, 0.0, 0.0, 0.080, 0.0],         # derin comelme -> hiz 0
                  [0.1, 0.0, 0.0, C.H_MIN_WALK, 0.0],  # sinirda yurume serbest
                  [0.0, 0.0, 0.0, 0.03, 0.0],          # h kirpilir
                  [0.0, 0.0, 0.0, 0.2, 0.7]])          # h kirpilir, sit > 0.5
    o = np.asarray(T.command_obs(np, c))
    assert np.allclose(o[0], [0.2, -0.1, 0.5, 0.0, 0.0]) and np.allclose(o[1], [0, 0, 0, 0, 1])
    assert np.allclose(o[2, :3], 0) and np.isclose(o[2, 3], (0.080 - C.H_HOME) / C.H_NORM)
    assert np.allclose(o[3, :3], [0.1, 0, 0])
    assert np.isclose(o[4, 3], -1.0) and np.isclose(o[5, 3], 0.0) and o[5, 4] == 1.0
    assert np.all(o[:, 3] >= -1.0) and np.all(o[:, 3] <= 0.0)
    s = _rand_state(np.random.default_rng(1), 32, "main")
    obs = T.observation(np, s, "main")
    sl = TK.get_task("main").obs_slices
    assert np.array_equal(obs[:, sl["command"]], T.command_obs(np, s["command"]))
    assert np.allclose(obs[:, sl["posture"]], np.stack([(s["h_ref"] - C.H_HOME) / C.H_NORM, s["alpha"]], 1))
    sit = s["command"][:, 4] > 0.5
    assert np.all(obs[sit][:, sl["command"]][:, :3] == 0)


def test_posture_step_rates_and_convergence():
    dt = C.CTRL_DT
    h, a = np.array([C.H_HOME]), np.array([0.0])
    cmd = np.array([[0.0, 0.0, 0.0, C.H_HOME, 1.0]])
    n = 0
    while a[0] < 1.0:
        h2, a2 = T.posture_step(np, h, a, cmd)
        assert 0 < a2[0] - a[0] <= dt / C.SIT_RAMP_S + 1e-15 and h2[0] == h[0]
        h, a, n = h2, a2, n + 1
        assert n < 1000
    assert a[0] == 1.0 and n == int(np.ceil(C.SIT_RAMP_S / dt))
    cmd = np.array([[0.0, 0.0, 0.0, C.H_MIN_STAND, 0.0]])
    n = 0
    while a[0] > 0.0 or h[0] != C.H_MIN_STAND:
        h2, a2 = T.posture_step(np, h, a, cmd)
        assert a[0] - a2[0] <= dt / C.STAND_RAMP_S + 1e-15 and h[0] - h2[0] <= C.H_RATE * dt + 1e-15
        h, a, n = h2, a2, n + 1
        assert n < 1000
    assert a[0] == 0.0 and h[0] == C.H_MIN_STAND
    assert n == max(int(np.ceil((C.H_HOME - C.H_MIN_STAND) / (C.H_RATE * dt) - 1e-9)),
                    int(np.ceil(C.STAND_RAMP_S / dt - 1e-9)))
    # filtre etkin (kirpilmis) h'yi izler
    h3, _ = T.posture_step(np, np.array([C.H_HOME]), np.array([0.0]), np.array([[0, 0, 0, 0.0, 0]]))
    assert np.isclose(h3[0], C.H_HOME - C.H_RATE * dt)


def test_q_ref_home_exact():
    home = _home()
    r = T.posture_ref(np, np.array([C.H_HOME]), np.array([0.0]), home)
    assert np.array_equal(r["q_ref"][0], home[:10])
    assert r["z_ref"][0] == C.H_HOME and r["pitch_ref"][0] == 0.0 and np.all(r["head_offset"] == 0.0)
    # tek ortam (vmap bicimi, eksen yok)
    r1 = T.posture_ref(np, np.array(C.H_HOME), np.array(0.0), home)
    assert r1["q_ref"].shape == (10,) and np.array_equal(r1["q_ref"], home[:10])


def test_q_ref_family_and_seat():
    home = _home()
    F = np.array(C.CROUCH_FAMILY)
    f0 = np.array([np.interp(C.H_HOME, F[::-1, 0], F[::-1, k]) for k in (2, 3, 4)])
    for row in F:
        h = row[0]
        r = T.posture_ref(np, np.array([h]), np.array([0.0]), home)
        q = r["q_ref"][0]
        assert np.allclose(np.degrees(q[2:5]), np.degrees(home[2:5]) + row[2:5] - f0, atol=1e-9), h
        assert np.allclose(q[5:10], -q[0:5], atol=1e-12)               # sag = -sol
        assert np.array_equal(q[[0, 1, 5, 6]], home[[0, 1, 5, 6]])        # hip_yaw/hip_roll HOME
        assert np.isclose(np.degrees(r["pitch_ref"][0]), row[1]) and np.isclose(r["z_ref"][0], h)
        # ayak tabani duz: pitch + hp - kn + an sabit (HOME degeri)
        foot = np.degrees(r["pitch_ref"][0] + q[2] - q[3] + q[4])
        assert abs(foot - np.degrees(home[2] - home[3] + home[4])) < 1e-9
    r = T.posture_ref(np, np.array([0.1, C.H_HOME]), np.array([1.0, 1.0]), home)
    seat = np.radians([C.SEAT_DEG[j] for j in C.CROUCH_JOINTS])
    for q in r["q_ref"]:
        assert np.allclose(q[2:5], seat) and np.allclose(q[7:10], -seat)
        assert np.array_equal(q[[0, 1, 5, 6]], home[[0, 1, 5, 6]])
    assert np.allclose(r["z_ref"], C.Z_SIT) and np.allclose(r["pitch_ref"], 0.0)
    head_sit = np.radians([C.HEAD_SIT_DEG[j] for j in C.HEAD_JOINTS])
    assert np.allclose(home[10:14] + r["head_offset"], head_sit)
    # alpha = SIT_SPLIT: derin comelme (F_home(H_DEEP)), bas tam geride
    r = T.posture_ref(np, np.array([C.H_HOME]), np.array([C.SIT_SPLIT]), home)
    assert np.isclose(r["z_ref"][0], C.H_DEEP) and np.allclose(home[10:14] + r["head_offset"][0], head_sit)
    # kucuk yardimcilar posture_ref ile ayni
    hr, al = np.array([0.09, 0.1]), np.array([0.3, 0.8])
    r = T.posture_ref(np, hr, al, home)
    assert np.array_equal(T.q_ref(np, hr, al, home), r["q_ref"]) and np.array_equal(T.z_ref(np, hr, al), r["z_ref"])
    assert np.array_equal(T.pitch_ref(np, hr, al), r["pitch_ref"])
    assert np.array_equal(T.head_offset_ref(np, hr, al, home), r["head_offset"])


def test_sit_path_monotone():
    home = _home()
    for h0 in (C.H_HOME, 0.095, C.H_MIN_STAND):
        al = np.linspace(0.0, 1.0, 501)
        r = T.posture_ref(np, np.full_like(al, h0), al, home)
        z, hp, pr = r["z_ref"], r["q_ref"][:, 2], r["pitch_ref"]
        assert np.all(np.diff(z) <= 1e-15) and np.isclose(z[0], h0) and np.isclose(z[-1], C.Z_SIT)
        assert np.all(np.diff(hp) >= -1e-15)                        # kalca surekli one bukulur
        assert np.max(np.abs(np.diff(r["q_ref"], axis=0))) < 0.02   # surekli (alpha basina ~0.5 derece)
        first = al <= C.SIT_SPLIT
        assert np.all(np.diff(pr[first]) <= 1e-15) and np.all(np.diff(pr[~first]) >= -1e-15)
        assert np.all(np.diff(r["head_offset"][:, 0]) >= -1e-15)


def test_head_home_matches_keyframe():
    home = _home()
    assert np.allclose(home[10:14], np.radians([C.HEAD_HOME_DEG[j] for j in C.HEAD_JOINTS]), atol=1e-5)
    # HEAD_SIT boyun ust sinirinda, ctrlrange icinde
    idx = _idx("main")
    hs = np.radians([C.HEAD_SIT_DEG[j] for j in C.HEAD_JOINTS])
    lo, hi = idx.ctrl_lo[idx.head_act_id], idx.ctrl_hi[idx.head_act_id]
    assert np.all(hs >= lo - 1e-6) and np.all(hs <= hi + 1e-6)


# --- MuJoCo ile F tablosu ve SEAT dogrulamasi (phys-sit family3/scan_kin mantigi) --------------------
def _hull(pts):
    pts = sorted(set(map(tuple, np.round(pts, 7))))
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return np.array(lo[:-1] + up[:-1])


def _margin(poly, p):
    """>0: p cokgenin icinde (kenara en kisa mesafe); <0 disinda."""
    n, best, inside = len(poly), np.inf, True
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        e = b - a
        nrm = np.array([e[1], -e[0]]) / np.linalg.norm(e)
        dd = np.dot(p - a, nrm)
        inside &= dd <= 0
        best = min(best, abs(dd))
    return best if inside else -best


def _kin_pose(m, d, idx, pitch_deg, legs10, head4=None):
    """Govde orijini (0,0,0), pitch; eklemler; -> (h = -en alcak ayak noktasi, govde kutusu acikligi, COM payi)."""
    mujoco.mj_resetData(m, d)
    d.qpos[:] = idx.init_qpos
    d.qpos[0:3] = 0.0
    th = np.radians(pitch_deg)
    d.qpos[3:7] = [np.cos(th / 2), 0, np.sin(th / 2), 0]
    d.qpos[idx.obs_qpos_adr[:10]] = legs10
    if head4 is not None:
        d.qpos[idx.obs_qpos_adr[10:14]] = head4
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)

    def corners(g):
        s, R = m.geom_size[g], d.geom_xmat[g].reshape(3, 3)
        sg = [(a, b, c) for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]
        return np.array([d.geom_xpos[g] + R @ (s * np.array(x)) for x in sg])
    fc = np.vstack([corners(g) for g in idx.foot_geom])
    z0 = fc[:, 2].min()
    bottom = fc[fc[:, 2] < z0 + 1e-3]
    tc = corners(idx.trunk_geom)
    return -z0, tc[:, 2].min() - z0, _margin(_hull(bottom[:, :2]), d.subtree_com[idx.trunk_body][:2])


def test_crouch_family_mujoco():
    m = build_model(variant="main")
    d, idx, home = mujoco.MjData(m), _idx("main"), _home()
    for h, pitch, hp, kn, an in C.CROUCH_FAMILY:
        legs = home[:10].copy()
        legs[2:5] = np.radians([hp, kn, an])
        legs[7:10] = -legs[2:5]
        hk, clear, marg = _kin_pose(m, d, idx, pitch, legs)
        assert abs(hk - h) < 1.5e-3, (h, hk)                       # family3: 1 mm kutulari
        assert marg > 0.025 and clear > 0.005, (h, marg, clear)     # COM ayaklarin >= 25 mm icinde
        assert abs(pitch + hp - kn + an) < 1e-9                    # duz ayak
    # ofset formu (q_ref, alpha = 0): HOME'un +3.3 mm'si ile H_DEEP'teki +5.7 mm arasinda, COM payi >= 20 mm
    for h in (C.H_HOME, 0.1, C.H_MIN_WALK, C.H_MIN_STAND, C.H_DEEP):
        r = T.posture_ref(np, np.array(h), np.array(0.0), home)
        hk, clear, marg = _kin_pose(m, d, idx, np.degrees(r["pitch_ref"]), r["q_ref"])
        assert 0.0025 < hk - h < 0.0065 and marg > 0.019 and clear > 0.01, (h, hk, marg, clear)
    # SEAT (alpha = 1): govde kutusu yerde (acikligi ~0), z ~ SEAT_Z; kafa geride COM ayaklarin icinde
    r = T.posture_ref(np, np.array(C.H_HOME), np.array(1.0), home)
    hk, clear, marg_home_head = _kin_pose(m, d, idx, 0.0, r["q_ref"])
    assert abs(clear) < 0.002 and abs(hk - C.SEAT_Z) < C.SEAT_Z_TOL, (hk, clear)
    _, _, marg = _kin_pose(m, d, idx, 0.0, r["q_ref"], home[10:14] + r["head_offset"])
    assert marg_home_head < 0 < marg, (marg_home_head, marg)       # phys-sit: kafa HOME'da COM onde


# --- main: oduller, sonlandirma, ornekleme, metrikler -------------------------------------------
def test_reward_key_sets():
    rng = np.random.default_rng(2)
    for name in TK.TASK_NAMES:
        t = TK.get_task(name)
        s = _rand_state(rng, 16, name)
        terms = T.reward_terms(np, s, name)
        assert list(terms) == list(t.reward_weights), name
        for k, v in terms.items():
            assert np.shape(v) == (16,) and np.all(np.isfinite(v)), (name, k)
        w = T.reward_weights(np, name)
        assert list(w) == list(t.reward_weights)
        r = T.total_reward(np, terms, T.terminated(np, s, name), w)
        assert r.shape == (16,) and np.all(np.isfinite(r))
        assert T.observation(np, s, name).shape == (16, t.num_obs)
        assert T.critic_observation(np, s, name).shape == (16, t.num_critic_obs)
        assert np.array_equal(T.critic_observation(np, s, name)[:, :t.num_obs], T.observation(np, s, name))


def test_main_reward_terms():
    home = _home()
    n = 3
    s = _rand_state(np.random.default_rng(3), n, "main")
    s["quat"] = np.tile([1.0, 0, 0, 0], (n, 1))
    s["linvel"] = np.zeros((n, 3))
    s["gyro"] = np.zeros((n, 3))
    s["command"] = np.array([[0.2, 0, 0, C.H_HOME, 0], [0.2, 0, 0, C.H_HOME, 1], [0, 0, 0, 0.09, 0]])
    s["alpha"] = np.array([0.0, 1.0, 0.0])
    s["h_ref"] = np.array([C.H_HOME, C.H_HOME, 0.09])
    s["q"] = np.tile(home, (n, 1))
    s["trunk_z"] = np.array([C.H_HOME, C.Z_SIT, 0.09])
    s["contact"] = np.ones((n, 2))
    s["trunk_floor"] = np.array([0.0, 1.0, 0.0])
    t = T.reward_terms(np, s, "main")
    assert np.allclose(t["height_track"], 1.0) and np.allclose(t["height_l1"], 0.0)
    assert np.isclose(t["tracking_lin_vel"][1], 1.0) and t["tracking_lin_vel"][0] < 0.2   # sit: komut 0
    assert t["seat"][1] == 1.0 and t["seat"][0] == 0.0 and t["seat"][2] == 0.0
    assert t["stand_contact"][0] == 0.0 and t["stand_contact"][1] == 0.0 and t["stand_contact"][2] == 1.0
    assert t["posture"][0] == 0.0 and t["posture"][1] > 1.0 and t["posture"][2] > 0.0   # yururken posture yok
    assert t["feet_phase"][1] == 0.0 and t["feet_phase"][2] == 0.0 and t["feet_air_time"][2] == 0.0
    assert t["orientation"][0] == 0.0 and t["orientation"][1] == 0.0
    # pitch referansli egim: alpha = 0, h_ref = 0.07 -> pitch_ref -17.5 derece; tam o egimde orientation = 0
    s2 = {k: (v[:1] if isinstance(v, np.ndarray) and v.ndim and v.shape[0] == n else v) for k, v in s.items()}
    s2["h_ref"], s2["alpha"] = np.array([0.07]), np.array([0.0])
    th = np.radians(-17.5)
    s2["quat"] = np.array([[np.cos(th / 2), 0, np.sin(th / 2), 0]])
    assert abs(T.reward_terms(np, s2, "main")["orientation"][0]) < 1e-12
    # vz_cap: dunya dikey hizi
    s2["quat"] = np.array([[1.0, 0, 0, 0]])
    s2["linvel"] = np.array([[0.0, 0.0, -0.18]])
    assert np.isclose(T.reward_terms(np, s2, "main")["vz_cap"][0], 0.10)


def test_main_termination_truth_table():
    up = [1.0, 0, 0, 0]
    th = np.radians(60)
    tilted = [np.cos(th / 2), 0, np.sin(th / 2), 0]
    rows = [  # quat, trunk_floor, head_floor, alpha, trunk_z, beklenen
        (up, 0, 0, 0.0, 0.115, 0), (up, 0, 0, 0.0, 0.050, 0),       # yukseklik kurali yok
        (tilted, 0, 0, 0.0, 0.1, 1), (tilted, 0, 0, 1.0, 0.05, 1),  # egim her modda
        (up, 0, 1, 0.0, 0.1, 1), (up, 0, 1, 1.0, 0.05, 1),          # kafa yere degdi
        (up, 1, 0, 0.0, 0.05, 1), (up, 1, 0, 0.29, 0.05, 1),        # planlanmamis oturma
        (up, 1, 0, 0.3, 0.05, 0), (up, 1, 0, 0.6, 0.05, 0), (up, 1, 0, 1.0, 0.044, 0),
    ]
    s = _rand_state(np.random.default_rng(4), len(rows), "main")
    s["quat"] = np.array([r[0] for r in rows])
    s["trunk_floor"] = np.array([r[1] for r in rows], float)
    s["head_floor"] = np.array([r[2] for r in rows], float)
    s["alpha"] = np.array([r[3] for r in rows])
    s["trunk_z"] = np.array([r[4] for r in rows])
    assert np.array_equal(T.terminated(np, s, "main"), [r[5] for r in rows])
    g = _rand_state(np.random.default_rng(5), 8, "getup")
    g["quat"] = np.tile(tilted, (8, 1))
    assert np.all(T.terminated(np, g, "getup") == 0.0)


def test_sample_segment():
    rng = np.random.default_rng(6)
    n = 40000
    u = rng.random((n, T.NUM_SEGMENT_U))
    prev = np.tile([0.0, 0, 0, 0.1, 0], (n, 1))
    kind, cmd, dur = T.sample_segment(np, u, prev)
    frac = np.array([np.mean(kind == k) for k in range(4)])
    assert np.allclose(frac, C.MAIN["segment_probs"], atol=0.01), frac
    assert np.all((dur >= 3.5) & (dur <= 6.5))
    w, st, cr, si = (kind == k for k in range(4))
    assert np.all(cmd[~w, :3] == 0) and np.all(cmd[~si, 4] == 0) and np.all(cmd[si, 4] == 1)
    for i, k in enumerate(("vx", "vy", "wz")):
        lo, hi = C.COMMAND_RANGES[k]
        assert np.all((cmd[w, i] >= lo) & (cmd[w, i] <= hi))
    zero = np.all(cmd[w, :3] == 0, axis=1)
    assert abs(zero.mean() - C.COMMAND_ZERO_PROB) < 0.02
    hw = cmd[w, 3]
    assert abs(np.mean(hw == C.H_HOME) - 0.5) < 0.02 and np.all((hw >= C.H_MIN_WALK) & (hw <= C.H_HOME))
    assert np.all(cmd[st, 3] == C.H_HOME)
    hc = cmd[cr, 3]
    assert np.all((hc >= C.H_MIN_STAND) & (hc <= C.H_HOME)) and hc.min() < 0.075
    assert np.all(cmd[si, 3] == 0.1)                                # sit h'yi korur
    # sit'ten sonra sit yok
    prev_s = np.tile([0.0, 0, 0, 0.1, 1.0], (n, 1))
    kind2, _, _ = T.sample_segment(np, u, prev_s)
    f2 = np.array([np.mean(kind2 == k) for k in range(4)])
    assert f2[3] == 0.0 and np.allclose(f2[:3], np.array(C.MAIN["segment_probs"][:3]) / 0.7, atol=0.01)
    # oturustan baslangic: %50 sit, %50 stand
    k3, c3, _ = T.sample_segment(np, u, np.asarray(T.reset_command(np, np.ones(n))), seated=np.ones(n))
    assert set(np.unique(k3)) == {T.SEG_STAND, T.SEG_SIT} and abs(np.mean(k3 == T.SEG_SIT) - 0.5) < 0.01
    assert np.all(c3[:, 3] == C.H_HOME)
    k4, _, _ = T.sample_segment(np, u, prev, seated=np.zeros(n))
    assert np.array_equal(k4, kind)
    # sifirlama turu ve banka indeksi
    r = T.sample_reset_kind(np, rng.random(n), "main")
    assert abs(np.mean(r == 1) - 0.25) < 0.01 and set(np.unique(r)) == {0.0, 1.0}
    r = T.sample_reset_kind(np, rng.random(n), "getup")
    assert np.allclose([np.mean(r == k) for k in range(3)], [0.7, 0.1, 0.2], atol=0.01)
    i = T.bank_index(np, np.array([0.0, 0.5, 0.999999999, 1.0]), 10)
    assert list(i) == [0, 5, 9, 9]


def test_main_events():
    """Senaryo: oturustan baslar, kalk komutu (stand) -> 1 s sonra ayakta; sonra sit -> 2 s sonra oturdu;
    sonra kalk komutu ama 3 s icinde kalkamaz."""
    s = _rand_state(np.random.default_rng(7), 1, "main")
    s.update(quat=np.array([[1.0, 0, 0, 0]]), contact=np.ones((1, 2)), trunk_floor=np.array([1.0]))
    ev = T.main_events_init(np, np.array([1.0]))
    tot = {k: 0.0 for k in TK.get_task("main").episode_metrics}

    def run(steps, cmd, alpha, z, tf, term=0.0):
        nonlocal ev
        for _ in range(steps):
            s.update(command=np.array([cmd]), alpha=np.array([alpha]), trunk_z=np.array([z]),
                     trunk_floor=np.array([tf]))
            ev, inc = T.main_events(np, s, ev, np.array([term]))
            assert list(inc) == list(tot)
            for k, v in inc.items():
                tot[k] += float(v[0])
    stand = [0, 0, 0, C.H_HOME, 0]
    sit = [0, 0, 0, C.H_HOME, 1]
    run(50, stand, 0.5, 0.06, 1.0)            # 1 s: henuz kalkmadi
    run(10, stand, 0.0, 0.11, 0.0)            # ayakta (z > 0.105, iki ayak, dik)
    assert tot["standup_attempts"] == 1 and tot["standup_success"] == 1 and tot["sitdown_attempts"] == 0
    assert tot["walk_steps"] == 10
    run(100, sit, 0.8, 0.05, 1.0)             # 2 s oturma gecisi
    run(10, sit, 1.0, C.Z_SIT, 1.0)           # oturdu
    assert tot["sitdown_attempts"] == 1 and tot["sitdown_success"] == 1 and tot["seated_ok_steps"] == 10
    assert tot["sit_steps"] == 110
    run(160, stand, 0.2, 0.08, 0.0)           # 3.2 s: kalkamadi
    run(10, stand, 0.0, 0.11, 0.0)            # pencere disinda ayaga kalkti -> sayilmaz
    assert tot["standup_attempts"] == 2 and tot["standup_success"] == 1
    run(1, stand, 0.0, 0.11, 0.0, term=1.0)
    run(1, sit, 0.5, 0.08, 0.0, term=1.0)
    run(1, sit, 1.0, 0.044, 1.0, term=1.0)
    assert (tot["falls_walk"], tot["falls_transition"], tot["falls_sit"]) == (1, 1, 1)


# --- getup --------------------------------------------------------------------------------------
def _getup_home_state(n=1):
    s = _rand_state(np.random.default_rng(8), n, "getup")
    s.update(quat=np.tile([1.0, 0, 0, 0], (n, 1)), q=np.tile(_home(), (n, 1)), gyro=np.zeros((n, 3)),
             trunk_z=np.full(n, 0.116), head_floor=np.zeros(n), head_floor_prev=np.zeros(n))
    return s


def test_getup_success():
    s = _getup_home_state(6)
    th = np.radians(30)
    s["quat"][1] = [np.cos(th / 2), np.sin(th / 2), 0, 0]           # egik
    s["trunk_z"][2] = 0.09                                          # alcak
    s["q"][3, 3] += 1.5                                             # poz uzak (rms 0.4)
    s["gyro"][4] = [0.0, 0.6, 0.0]                                  # donuyor
    ok = T.getup_success_now(np, s)
    assert list(ok) == [1, 0, 0, 0, 0, 1]
    ok2 = T.getup_success_now(np, s, use_height=False)
    assert list(ok2) == [1, 0, 1, 0, 0, 1]
    t, held = np.zeros(1), 0.0
    for k in range(60):
        t, held = T.hold_timer(np, t, np.ones(1), C.GETUP["success"]["hold_s"])
        if k < 49:
            assert held[0] == 0.0
    assert held[0] == 1.0
    t, held = T.hold_timer(np, t, np.zeros(1), 1.0)
    assert t[0] == 0.0 and held[0] == 0.0
    # olay sayaci: bir kez
    ev = T.getup_events_init(np, np.zeros(1))
    s1 = _getup_home_state(1)
    tot = 0.0
    for k in range(120):
        ev, inc = T.getup_events(np, s1, ev)
        tot += inc["getup_success"][0]
    assert tot == 1.0 and ev["succeeded"][0] == 1.0


def test_getup_terms():
    s = _getup_home_state(2)
    s["phi_up_prev"] = np.array([0.98, 1.0])
    s["phi_h_prev"] = np.array([0.110, 0.116])
    s["head_floor"] = np.array([1.0, 1.0])
    s["head_floor_prev"] = np.array([0.0, 1.0])
    s["head_vel"] = np.array([[0.3, 0.0, -0.4], [1.0, 1.0, 1.0]])
    s["torque"] = np.zeros((2, 14))
    s["torque"][0, 3] = 0.96
    t = T.reward_terms(np, s, "getup")
    assert np.allclose(t["upright_progress"], [0.02 / C.CTRL_DT, 0.0])
    assert np.allclose(t["height_progress"], [(C.H_HOME - 0.110) / C.CTRL_DT, (C.H_HOME - 0.116) / C.CTRL_DT])
    assert np.isclose(t["head_impact"][0] * C.CTRL_DT, 0.25) and t["head_impact"][1] == 0.0   # yalniz baslangicta
    assert np.isclose(t["torque_sat"][0], 0.96 - 0.9 * 0.96) and t["torque_sat"][1] == 0.0
    assert np.allclose(t["pose"], 1.0) and np.allclose(t["upright"], 1.0)
    assert np.allclose(t["composite"], np.exp(-((0.116 - C.H_HOME) / 0.03) ** 2))
    mem = T.getup_memory(np, s)
    assert np.allclose(mem["phi_up_prev"], 1.0) and np.allclose(mem["phi_h_prev"], C.H_HOME)
    assert np.array_equal(mem["head_floor_prev"], s["head_floor"])
    # mufredat agirliklari
    g = TK.get_task("getup")
    w0, w1 = T.reward_weights(np, "getup", 0.0), T.reward_weights(np, "getup", 1.0)
    wm = T.reward_weights(np, "getup", 0.625)
    assert T.reward_weights(np, "getup") == g.reward_weights
    for k in g.reward_weights:
        lo, hi = g.reward_weights[k], g.reward_late.get(k, g.reward_weights[k])
        assert np.isclose(w0[k], lo) and np.isclose(w1[k], hi) and np.isclose(wm[k], 0.5 * (lo + hi)), k
    wv = T.reward_weights(np, "getup", np.array([0.0, 0.5, 0.75, 0.9]))
    assert np.allclose(wv["action_rate"], [-0.01, -0.01, -0.1, -0.1])


def test_getup_action_mapping():
    idx = _idx("getup", TK.GETUP_ACTION_JOINTS)
    assert idx.action_joints == list(C.LEG_JOINTS + C.HEAD_JOINTS)
    q0 = idx.default_action_q
    assert np.array_equal(q0, idx.default_obs_q)
    S = TK.action_scale_vector("getup", q0, idx.action_lo, idx.action_hi)
    assert S.shape == (14,) and np.allclose(S, np.maximum(idx.action_hi - q0, q0 - idx.action_lo))
    for sign in (-1.0, 1.0):
        t, a = T.action_to_targets(np, np.full((1, 14), 3.0 * sign), q0, scale=S, clip=1.0,
                                   lo=idx.action_lo, hi=idx.action_hi)
        assert np.all(np.abs(a) == 1.0)
        assert np.all((t >= idx.action_lo) & (t <= idx.action_hi))
    tp, _ = T.action_to_targets(np, np.ones((1, 14)), q0, S, 1.0, idx.action_lo, idx.action_hi)
    tn, _ = T.action_to_targets(np, -np.ones((1, 14)), q0, S, 1.0, idx.action_lo, idx.action_hi)
    hit = np.isclose(tp[0], idx.action_hi) | np.isclose(tn[0], idx.action_lo)
    assert np.all(hit)                                                # her eklem bir sinira ulasir
    t0, _ = T.action_to_targets(np, np.zeros((2, 14)), q0, S, 1.0, idx.action_lo, idx.action_hi)
    assert np.array_equal(t0, np.tile(q0, (2, 1)))
    # main: ortam basina nominal (q_ref)
    home = _home()
    qr = T.q_ref(np, np.array([C.H_HOME, 0.08]), np.array([0.0, 0.7]), home)
    t, _ = T.action_to_targets(np, np.zeros((2, 10)), qr)
    assert np.array_equal(t, qr)


def test_model_variants():
    w, m, g = _idx("walk"), _idx("main"), _idx("getup", TK.GETUP_ACTION_JOINTS)
    k = np.radians(C.KNEE_LIMIT_DEG)
    mm = build_model(variant="main")
    for side, i in (("left", 1), ("right", 0)):
        j = mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_JOINT, side + "_knee")
        a = mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_ACTUATOR, side + "_knee")
        assert np.isclose(abs(mm.jnt_range[j, i]), k) and mm.jnt_range[j, i] == mm.actuator_ctrlrange[a, i]
        assert np.isclose(abs(mm.jnt_range[j, i]), 1.13446, atol=1e-5)
    assert (w.variant, m.variant, g.variant) == ("walk", "main", "getup")
    assert m.s_head_vel is not None and len(m.extra_geoms) == 0 and len(g.extra_geoms) == len(C.EXTRA_COLLISION)
    gm = build_model(variant="getup")
    for gid in g.extra_geoms:
        assert gm.geom_contype[gid] == 0 and gm.geom_conaffinity[gid] == 1 and gm.geom_group[gid] == 3
    assert len(g.head_geoms) == 2 and len(m.head_geoms) == 1
    assert np.isclose(build_model().body_mass.sum(), gm.body_mass.sum())
    # yeni sensorler sona eklenir: walk sensor adresleri ayni
    assert (w.s_quat, w.s_gyro, w.s_linvel) == (m.s_quat, m.s_gyro, m.s_linvel)


def test_new_task_meta():
    for name in ("main", "getup"):
        t = TK.get_task(name)
        meta = PM.build_meta(10, "mjx_brax", task=name)
        json.dumps(meta)
        assert meta["task"] == name and meta["version"] == PM.META_VERSION
        assert meta["obs_dim"] == t.num_obs and meta["action_dim"] == t.num_actions
        assert sum(k for _, k in meta["obs_layout"]) == t.num_obs
        assert len(meta["action_scale"]) == len(meta["action_joints"]) == len(meta["default_action_q"]) == t.num_actions
        assert set(_old_meta(1, "x", _idx())) <= set(meta)
    m = PM.build_meta(1, "cpu_torch", task="main")
    assert m["command_names"] == ["vx", "vy", "wz", "h", "sit"] and m["posture_filter"]["h_rate"] == C.H_RATE
    assert len(m["posture_ref"]["crouch_family_deg"]) == len(C.CROUCH_FAMILY)
    g = PM.build_meta(1, "cpu_torch", task="getup")
    assert g["action_mode"] == "home_clip" and g["action_clip"] == 1.0 and g["head_control"] == "policy"
    try:
        PM.build_meta(1, "x", _idx(), task="getup")
        raise AssertionError
    except ValueError:
        pass


# --- numpy / jax esitligi -----------------------------------------------------------------------
def _to_jax(s):
    return {k: jnp.asarray(v, jnp.float32) for k, v in s.items()}


def _close(a, b, tol=2e-4):
    a, b = np.asarray(a, np.float64), np.asarray(b, np.float64)
    assert a.shape == b.shape, (a.shape, b.shape)
    assert np.allclose(a, b, rtol=tol, atol=tol), float(np.max(np.abs(a - b)))


def test_jax_parity():
    if jnp is None:
        _skip("jax yok")
    rng = np.random.default_rng(9)
    for name in TK.TASK_NAMES:
        s = _rand_state(rng, 32, name)
        js = _to_jax(s)
        _close(T.observation(jnp, js, name), T.observation(np, s, name))
        _close(T.critic_observation(jnp, js, name), T.critic_observation(np, s, name))
        _close(T.noise_vector(jnp, name), T.noise_vector(np, name))
        tn, tj = T.reward_terms(np, s, name), T.reward_terms(jnp, js, name)
        assert list(tn) == list(tj)
        for k in tn:
            sc = max(1.0, float(np.max(np.abs(tn[k]))))
            _close(np.asarray(tj[k]) / sc, tn[k] / sc, 1e-3)
        _close(T.terminated(jnp, js, name), T.terminated(np, s, name))
        wn, wj = T.reward_weights(np, name, 0.6), T.reward_weights(jnp, name, jnp.float32(0.6))
        for k in wn:
            _close(wj[k], wn[k], 1e-6)
    s = _rand_state(rng, 32, "main")
    js = _to_jax(s)
    home = _home()
    rn = T.posture_ref(np, s["h_ref"], s["alpha"], home)
    rj = T.posture_ref(jnp, js["h_ref"], js["alpha"], jnp.asarray(home))
    for k in rn:
        _close(rj[k], rn[k], 1e-5)
    rj0 = T.posture_ref(jnp, jnp.float32(C.H_HOME), jnp.float32(0.0), jnp.asarray(home, jnp.float32))
    assert np.array_equal(np.asarray(rj0["q_ref"]), np.asarray(home[:10], np.float32))   # float32'de de tam HOME
    for a, b in zip(T.posture_step(jnp, js["h_ref"], js["alpha"], js["command"]),
                    T.posture_step(np, s["h_ref"], s["alpha"], s["command"])):
        _close(a, b, 1e-6)
    u = rng.random((64, T.NUM_SEGMENT_U))
    prev = np.tile([0.1, 0, 0, 0.09, 0.0], (64, 1))
    prev[::3, 4] = 1.0
    for a, b in zip(T.sample_segment(jnp, jnp.asarray(u, jnp.float32), jnp.asarray(prev, jnp.float32)),
                    T.sample_segment(np, u.astype(np.float32).astype(np.float64), prev)):
        _close(a, b, 1e-5)
    ev = T.main_events_init(np, (rng.random(32) < 0.5).astype(float))
    term = (rng.random(32) < 0.1).astype(float)
    en, inn = T.main_events(np, s, ev, term)
    ej, inj = T.main_events(jnp, js, {k: jnp.asarray(v, jnp.float32) for k, v in ev.items()}, jnp.asarray(term))
    for k in en:
        _close(ej[k], en[k], 1e-5)
    for k in inn:
        _close(inj[k], inn[k], 1e-6)
    sg = _rand_state(rng, 16, "getup")
    gn = T.getup_events(np, sg, T.getup_events_init(np, sg["trunk_z"]))
    gj = T.getup_events(jnp, _to_jax(sg), T.getup_events_init(jnp, jnp.asarray(sg["trunk_z"])))
    for a, b in zip(gj, gn):
        for k in a:
            _close(a[k], b[k], 1e-4)


def test_jax_vmap_single_env():
    """MJX ortami tek ortami vmap'ler: tum fonksiyonlar eksensiz (skaler) girdiyle de calismali."""
    if jnp is None:
        _skip("jax yok")
    rng = np.random.default_rng(10)
    home = jnp.asarray(_home(), jnp.float32)
    for name in TK.TASK_NAMES:
        s = _to_jax(_rand_state(rng, 8, name))
        s["default_q"] = home
        batched = {k: v for k, v in s.items() if k != "default_q"}

        def one(b):
            x = dict(b, default_q=home)
            t = T.reward_terms(jnp, x, name)
            out = {"obs": T.observation(jnp, x, name), "critic": T.critic_observation(jnp, x, name),
                   "term": T.terminated(jnp, x, name),
                   "rew": T.total_reward(jnp, t, 0.0, T.reward_weights(jnp, name, 0.7))}
            if name == "main":
                out["q_ref"] = T.posture_ref(jnp, x["h_ref"], x["alpha"], home)["q_ref"]
                out["ps"] = jnp.stack(T.posture_step(jnp, x["h_ref"], x["alpha"], x["command"]))
                _, inc = T.main_events(jnp, x, T.main_events_init(jnp, x["alpha"]), 0.0)
                out["ev"] = jnp.stack([inc[k] for k in TK.get_task(name).episode_metrics])
                out["seg"] = T.sample_segment(jnp, jnp.full(T.NUM_SEGMENT_U, 0.3), x["command"], x["alpha"])[1]
            if name == "getup":
                _, inc = T.getup_events(jnp, x, T.getup_events_init(jnp, x["trunk_z"]))
                out["ev"] = jnp.stack([inc[k] for k in TK.get_task(name).episode_metrics])
            return out
        v = jax.jit(jax.vmap(one))(batched)
        t = T.reward_terms(jnp, s, name)
        _close(v["obs"], T.observation(jnp, s, name), 1e-5)
        _close(v["critic"], T.critic_observation(jnp, s, name), 1e-5)
        _close(v["rew"], T.total_reward(jnp, t, 0.0, T.reward_weights(jnp, name, 0.7)), 1e-4)


def main():
    _STANDALONE[0] = True
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    if len(sys.argv) > 1:
        tests = [(k, v) for k, v in tests if any(a in k for a in sys.argv[1:])]
    failed = 0
    for name, fn in tests:
        t = time.time()
        try:
            fn()
            print("PASS  %-40s %6.2fs" % (name, time.time() - t), flush=True)
        except _Skip as e:
            print("SKIP  %-40s %s" % (name, e), flush=True)
        except Exception:
            failed += 1
            import traceback
            print("FAIL  %-40s %6.2fs" % (name, time.time() - t), flush=True)
            traceback.print_exc()
    print("%d/%d passed" % (len(tests) - failed, len(tests)))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
