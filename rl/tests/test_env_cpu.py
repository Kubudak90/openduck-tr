# -*- coding: utf-8 -*-
"""
rl/env_cpu.py testleri. Calistirma (repo kokunden):

  python3 -m rl.tests.test_env_cpu          # duz assert'ler
  python3 -m pytest rl/tests/test_env_cpu.py   # (pytest kuruluysa)
"""
import gc
import os
import signal
import sys
import time

import mujoco
import numpy as np

from rl import config as C
from rl import task_math as T
from rl.env_cpu import VecEnv, data_foot_contacts, physics_model
from rl.robot_model import RobotIndex

OFF = dict(randomize=False, noise=False, head_random=False, push=False)
# kritik gozlem yerlesimi
I_GRAV_Z, I_LAST_ACT, I_CMD, I_PHASE = 2, slice(34, 44), slice(44, 47), slice(47, 49)
I_TRUNK_Z, I_CONTACT, I_FRICTION = 52, slice(53, 55), 57


def _rollout(env, steps, std=1.0, seed=123):
    rng = np.random.default_rng(seed)
    out = [env.reset()]
    for _ in range(steps):
        a = (std * rng.standard_normal((env.num_envs, env.num_actions))).astype(np.float32)
        o, c, r, d, info = env.step(a)
        out.append((o, c, r, d, info["time_outs"]))
    return out


def _same(x, y):
    return all(np.array_equal(a, b) for a, b in zip(x, y))


def test_shapes_dtypes():
    for w in (0, 2):
        env = VecEnv(5, num_workers=w, seed=0)
        try:
            obs, crit = env.reset()
            assert obs.shape == (5, C.NUM_OBS) and obs.dtype == np.float32
            assert crit.shape == (5, C.NUM_CRITIC_OBS) and crit.dtype == np.float32
            obs, crit, rew, done, info = env.step(np.zeros((5, C.NUM_ACTIONS), np.float32))
            assert obs.shape == (5, 49) and obs.dtype == np.float32
            assert crit.shape == (5, 58) and crit.dtype == np.float32
            assert rew.shape == (5,) and rew.dtype == np.float32
            assert done.shape == (5,) and done.dtype == np.bool_
            assert info["time_outs"].shape == (5,) and info["time_outs"].dtype == np.bool_
            assert np.all(np.isfinite(obs)) and np.all(np.isfinite(crit)) and np.all(np.isfinite(rew))
            # gurultusuz aktor gozlemi = kritigin ilk 49'u (gurultu acikken farkli olmali)
            assert not np.allclose(obs, crit[:, :49])
        finally:
            env.close()


def test_reproducible_seed():
    a = _rollout(VecEnv(4, num_workers=0, seed=7), 40)
    b = _rollout(VecEnv(4, num_workers=0, seed=7), 40)
    c = _rollout(VecEnv(4, num_workers=0, seed=8), 40)
    assert all(_same(x, y) for x, y in zip(a, b))
    assert not np.array_equal(a[-1][0], c[-1][0])


def test_standing_zero_action():
    env = VecEnv(4, num_workers=0, seed=1, **OFF)
    obs, crit = env.reset()
    z = np.zeros((4, C.NUM_ACTIONS), np.float32)
    for k in range(int(round(5.0 / C.CTRL_DT))):
        obs, crit, rew, done, info = env.step(z)
        assert not done.any(), "fell/terminated at step %d" % k
        assert np.all(crit[:, I_TRUNK_Z] > C.BASE_HEIGHT_TARGET - 0.02) and np.all(obs[:, I_GRAV_Z] < -0.95)
    assert np.all(crit[:, I_CONTACT] == 1.0)
    assert np.all(rew > 0.0)


def test_auto_reset_on_fall():
    env = VecEnv(3, num_workers=0, seed=2, **OFF)
    env.reset()
    z = np.zeros((3, C.NUM_ACTIONS), np.float32)
    for _ in range(5):
        env.step(z + 0.3)
    m, d = env.mj_model(1), env.mj_data(1)
    d.qpos[3:7] = [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0]     # x etrafinda 90 derece
    mujoco.mj_forward(m, d)
    obs, crit, rew, done, info = env.step(z + 0.3)
    assert done[1] and not info["time_outs"][1]
    assert not done[0] and not done[2]
    assert rew[1] < C.TERMINATION_PENALTY + 1.0
    # donen gozlem sifirlama sonrasi durumdur
    assert obs[1, I_GRAV_Z] < -0.99
    assert np.all(obs[1, I_LAST_ACT] == 0.0) and np.all(obs[0, I_LAST_ACT] != 0.0)
    assert abs(crit[1, I_TRUNK_Z] - env.robot_index.init_qpos[2]) < 1e-3
    assert env._core.step_count[1] == 0 and env._core.step_count[0] == 6
    assert "episode" in info and info["episode"]["ep_len"] == 6.0


def test_time_outs():
    env = VecEnv(2, num_workers=0, seed=3, **OFF)
    env.reset()
    z = np.zeros((2, C.NUM_ACTIONS), np.float32)
    for k in range(C.MAX_EPISODE_STEPS):
        obs, crit, rew, done, info = env.step(z)
        if k < C.MAX_EPISODE_STEPS - 1:
            assert not done.any() and not info["time_outs"].any() and "episode" not in info, k
    assert done.all() and info["time_outs"].all()
    ep = info["episode"]
    assert ep["ep_len"] == float(C.MAX_EPISODE_STEPS)
    assert set(ep) == {"ep_len"} | {"rew_" + k for k in C.REWARD_WEIGHTS}
    assert abs(ep["rew_alive"] - C.REWARD_WEIGHTS["alive"] * C.EPISODE_LENGTH_S) < 1e-6
    obs, crit, rew, done, info = env.step(z)
    assert not done.any() and env._core.step_count.tolist() == [1, 1]


def _nominal():
    m = physics_model()
    return m, RobotIndex(m)


def test_domain_randomization_ranges_no_drift():
    nom, idx = _nominal()
    env = VecEnv(3, num_workers=0, seed=4, randomize=True)
    DR = C.DOMAIN_RAND
    tb = idx.trunk_body
    sts = idx.obs_act_id
    other_act = np.setdiff1d(np.arange(nom.nu), sts)
    act_dof = nom.jnt_dofadr[nom.actuator_trnid[:, 0]]
    other_dof = np.setdiff1d(np.arange(nom.nv), act_dof)
    other_body = np.setdiff1d(np.arange(1, nom.nbody), [tb])
    eps = 1e-9
    mass_scales, fr = [], []
    for it in range(60):
        obs, crit = env.reset()
        for i in range(env.num_envs):
            m = env.mj_model(i)
            f = m.geom_friction[idx.foot_geom[0], 0]
            fr.append(f)
            # surtunme: iki ayak VE zemin ayni f; diger sutunlar nominal
            assert DR["friction"][0] - eps <= f <= DR["friction"][1] + eps
            assert m.geom_friction[idx.foot_geom[1], 0] == f and m.geom_friction[idx.floor_geom, 0] == f
            assert np.array_equal(m.geom_friction[:, 1:], nom.geom_friction[:, 1:])
            assert abs(crit[i, I_FRICTION] - f) < 1e-6
            # kutle/atalet
            s = m.body_mass[other_body] / nom.body_mass[other_body]
            mass_scales.append(s)
            assert np.all(s >= DR["link_mass_scale"][0] - eps) and np.all(s <= DR["link_mass_scale"][1] + eps)
            si = m.body_inertia[1:] / nom.body_inertia[1:]
            assert np.allclose(si, (m.body_mass[1:] / nom.body_mass[1:])[:, None], rtol=1e-12)
            assert m.body_mass[0] == nom.body_mass[0]
            tm = m.body_mass[tb]
            assert nom.body_mass[tb] * DR["link_mass_scale"][0] + DR["trunk_added_mass"][0] - eps <= tm
            assert tm <= nom.body_mass[tb] * DR["link_mass_scale"][1] + DR["trunk_added_mass"][1] + eps
            dcom = m.body_ipos[tb] - nom.body_ipos[tb]
            assert np.all(np.abs(dcom) <= DR["trunk_com_offset"] + eps)
            assert np.array_equal(np.delete(m.body_ipos, tb, 0), np.delete(nom.body_ipos, tb, 0))
            # kp
            ks = m.actuator_gainprm[sts, 0] / nom.actuator_gainprm[sts, 0]
            assert np.all(ks >= DR["kp_scale"][0] - eps) and np.all(ks <= DR["kp_scale"][1] + eps)
            assert np.array_equal(m.actuator_biasprm[sts, 1], -m.actuator_gainprm[sts, 0])
            assert np.array_equal(m.actuator_gainprm[other_act], nom.actuator_gainprm[other_act])
            assert np.array_equal(m.actuator_biasprm[other_act], nom.actuator_biasprm[other_act])
            # eklem parametreleri
            for field, key in (("dof_damping", "damping_scale"), ("dof_frictionloss", "frictionloss_scale"),
                               ("dof_armature", "armature_scale")):
                v, v0 = getattr(m, field), getattr(nom, field)
                r = v[act_dof] / v0[act_dof]
                assert np.all(r >= DR[key][0] - eps) and np.all(r <= DR[key][1] + eps), field
                assert np.array_equal(v[other_dof], v0[other_dof])
            assert env._core.delay[i] == 0
    # nominalden ornekleme: kayma yok, dagilim aralik ortasinda
    ms = np.concatenate(mass_scales)
    lo, hi = DR["link_mass_scale"]
    assert abs(ms.mean() - 0.5 * (lo + hi)) < 0.01 and ms.min() < lo + 0.02 and ms.max() > hi - 0.02
    assert abs(np.mean(fr) - 0.5 * sum(DR["friction"])) < 0.08
    # farkli bolumler farkli parametreler
    assert len(set(np.round(fr, 12))) == len(fr)


def test_no_randomization_is_nominal():
    nom, idx = _nominal()
    env = VecEnv(2, num_workers=0, seed=5, **OFF)
    obs, crit = env.reset()
    for _ in range(3):
        obs, crit, *_ = env.step(np.zeros((2, C.NUM_ACTIONS), np.float32))
    for i in range(2):
        m = env.mj_model(i)
        for f in ("geom_friction", "body_mass", "body_inertia", "body_ipos", "actuator_gainprm",
                  "actuator_biasprm", "dof_damping", "dof_frictionloss", "dof_armature"):
            assert np.array_equal(getattr(m, f), getattr(nom, f)), f
    assert np.all(crit[:, I_FRICTION] == 1.0)


def test_action_delay():
    """action_delay_steps=(2,2) iken uygulanan bacak hedefi 2 adim onceki aksiyondur (ilk adimlarda 0)."""
    old = C.DOMAIN_RAND["action_delay_steps"]
    C.DOMAIN_RAND["action_delay_steps"] = (2, 2)
    try:
        env = VecEnv(2, num_workers=0, seed=16, randomize=True, noise=False, head_random=False, push=False)
        env.reset()
        idx = env.robot_index
        assert env._core.delay.tolist() == [2, 2]
        rng = np.random.default_rng(0)
        acts = [(0.5 * rng.standard_normal((2, C.NUM_ACTIONS))).astype(np.float32) for _ in range(6)]
        for t, a in enumerate(acts):
            obs, crit, rew, done, info = env.step(a)
            assert not done.any()
            applied = acts[t - 2].astype(np.float64) if t >= 2 else np.zeros((2, C.NUM_ACTIONS))
            tgt = np.clip(idx.default_action_q + C.ACTION_SCALE * applied, idx.ctrl_lo[idx.action_act_id],
                          idx.ctrl_hi[idx.action_act_id])
            for i in range(2):
                assert np.allclose(env.mj_data(i).ctrl[idx.action_act_id], tgt[i], atol=1e-12), t
            assert np.allclose(obs[:, I_LAST_ACT], a, atol=1e-6)      # gozlem politikanin kendi aksiyonu
    finally:
        C.DOMAIN_RAND["action_delay_steps"] = old


def test_phase_advance():
    env = VecEnv(4, num_workers=0, seed=6, **OFF)
    obs, _ = env.reset()
    ph = lambda o: np.mod(np.arctan2(o[:, I_PHASE][:, 1], o[:, I_PHASE][:, 0]) / (2 * np.pi), 1.0)
    p0 = ph(obs)
    assert np.allclose(p0, env._core.phase, atol=1e-6)
    for _ in range(30):
        obs, crit, rew, done, info = env.step(np.zeros((4, C.NUM_ACTIONS), np.float32))
        p1 = ph(obs)
        dp = np.mod(p1 - p0, 1.0)
        assert np.allclose(dp[~done], C.GAIT_FREQ * C.CTRL_DT, atol=1e-5)
        p0 = p1


def test_contacts():
    env = VecEnv(3, num_workers=0, seed=9, **OFF)
    env.reset()
    z = np.zeros((3, C.NUM_ACTIONS), np.float32)
    for _ in range(50):
        obs, crit, *_ = env.step(z)
    assert np.all(crit[:, I_CONTACT] == 1.0)
    idx = env.robot_index
    m, d = env.mj_model(0), env.mj_data(0)
    assert np.all(data_foot_contacts(d, idx) == 1.0)
    # tum robot 5 cm yukarida: ortam adimi da iki ayagi havada gormeli
    d.qpos[2] += 0.05
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)
    assert np.all(data_foot_contacts(d, idx) == 0.0)
    obs, crit, *_ = env.step(z)
    assert np.all(crit[0, I_CONTACT] == 0.0) and np.all(crit[1:, I_CONTACT] == 1.0)
    # yalniz bir ayak kalkik: govdeyi x ekseni etrafinda yuvarla
    m, d = env.mj_model(1), env.mj_data(1)
    th = 0.15
    d.qpos[3:7] = [np.cos(th / 2), np.sin(th / 2), 0.0, 0.0]
    d.qpos[2] = idx.init_qpos[2] - 0.003
    mujoco.mj_forward(m, d)
    fz = np.array([d.sensordata[sl][2] for sl in idx.s_foot_pos])
    up = int(np.argmax(fz))
    assert fz[up] > 0.003 and fz[1 - up] < 0.0
    c = data_foot_contacts(d, idx)
    assert c[up] == 0.0 and c[1 - up] == 1.0


def test_inprocess_equals_workers():
    ref = _rollout(VecEnv(6, num_workers=0, seed=11), 80, std=2.0)
    assert any(x[3].any() for x in ref[1:]), "no reset happened; test would not cover the reset path"
    for w, sm in ((2, "forkserver"), (4, "fork"), (3, "spawn")):
        env = VecEnv(6, num_workers=w, seed=11, start_method=sm)
        try:
            got = _rollout(env, 80, std=2.0)
        finally:
            env.close()
        for k, (x, y) in enumerate(zip(ref, got)):
            assert _same(x, y), "workers=%d (%s) differs at step %d" % (w, sm, k)


def test_set_commands():
    cmd = np.array([[0.2, 0.0, 0.0], [0.0, 0.05, 0.5], [-0.1, 0.0, -0.3]])
    for w in (0, 2):
        env = VecEnv(3, num_workers=w, seed=12, fixed_command=[0.1, 0.0, 0.0])
        try:
            obs, _ = env.reset()
            assert np.allclose(obs[:, I_CMD], [0.1, 0.0, 0.0])
            env.set_commands(cmd)
            for _ in range(int(C.COMMAND_RESAMPLE_S / C.CTRL_DT) + 2):
                obs, crit, rew, done, info = env.step(np.zeros((3, C.NUM_ACTIONS), np.float32))
                assert np.allclose(obs[:, I_CMD], cmd, atol=1e-7)
            env.set_commands(None)
        finally:
            env.close()
    # sabit komut sifirlamadan sonra da korunur
    env = VecEnv(2, num_workers=0, seed=13, **OFF)
    env.reset()
    env.set_commands(cmd[:2])
    d = env.mj_data(0)
    d.qpos[2] = 0.05
    mujoco.mj_forward(env.mj_model(0), d)
    obs, crit, rew, done, info = env.step(np.zeros((2, C.NUM_ACTIONS), np.float32))
    assert done[0] and np.allclose(obs[:, I_CMD], cmd[:2], atol=1e-7)


def test_random_commands_resampled():
    env = VecEnv(8, num_workers=0, seed=14, **OFF)
    obs, _ = env.reset()
    c0 = obs[:, I_CMD].copy()
    lo = np.array([C.COMMAND_RANGES[k][0] for k in ("vx", "vy", "wz")])
    hi = np.array([C.COMMAND_RANGES[k][1] for k in ("vx", "vy", "wz")])
    assert np.all(c0 >= lo - 1e-6) and np.all(c0 <= hi + 1e-6)
    n = int(round(C.COMMAND_RESAMPLE_S / C.CTRL_DT))
    z = np.zeros((8, C.NUM_ACTIONS), np.float32)
    for k in range(n):
        obs, crit, rew, done, info = env.step(z)
        if k < n - 1:
            assert np.array_equal(obs[:, I_CMD], c0)
    assert not np.array_equal(obs[:, I_CMD], c0)


def test_head_and_push():
    env = VecEnv(6, num_workers=0, seed=15, randomize=False, noise=False, head_random=True, push=True)
    env.reset()
    idx = env.robot_index
    core = env._core
    z = np.zeros((6, C.NUM_ACTIONS), np.float32)
    head_dev, pushes, prev = 0.0, 0, core.push_timer.copy()
    for _ in range(int(8.0 / C.CTRL_DT)):
        env.step(z)
        for i in range(6):
            dh = env.mj_data(i).ctrl[idx.head_act_id] - idx.init_ctrl[idx.head_act_id]
            assert np.all(np.abs(dh) <= idx.head_range + 1e-9)
            head_dev = max(head_dev, np.abs(dh).max())
        pushes += int(np.sum(core.push_timer > prev + 1e-9))
        prev = core.push_timer.copy()
        assert np.all(core.push_timer <= C.DOMAIN_RAND["push_interval_s"][1] + 1e-9)
    assert head_dev > 0.01 and pushes >= 6


def test_close_and_gc():
    env = VecEnv(4, num_workers=2, seed=0)
    env.reset()
    procs = list(env._procs)
    env.close()
    env.close()
    assert not any(p.is_alive() for p in procs)
    try:
        env.step(np.zeros((4, C.NUM_ACTIONS), np.float32))
        raise AssertionError("step after close should fail")
    except RuntimeError:
        pass
    env = VecEnv(4, num_workers=2, seed=0)
    procs = list(env._procs)
    del env
    gc.collect()
    t = time.time()
    while any(p.is_alive() for p in procs) and time.time() - t < 5.0:
        time.sleep(0.05)
    assert not any(p.is_alive() for p in procs)


def _ref_contacts(d, idx):
    c = np.zeros(2)
    for k in range(d.ncon):
        con = d.contact[k]
        g = (int(con.geom1), int(con.geom2))
        if idx.floor_geom in g and con.dist < C.CONTACT_EPS:
            for f in range(2):
                if int(idx.foot_geom[f]) in g:
                    c[f] = 1.0
    return c


def _ref_state(d, idx, a, a_prev, cmd, phase):
    sd = d.sensordata
    return dict(quat=sd[idx.s_quat][None].copy(), gyro=sd[idx.s_gyro][None].copy(),
                linvel=sd[idx.s_linvel][None].copy(), trunk_z=np.array([d.xpos[idx.trunk_body, 2]]),
                q=d.qpos[idx.obs_qpos_adr][None].copy(), qd=d.qvel[idx.obs_dof_adr][None].copy(),
                qdd_legs=d.qacc[idx.leg_dof_adr][None].copy(), torque=d.actuator_force[idx.obs_act_id][None].copy(),
                foot_pos=np.stack([sd[sl] for sl in idx.s_foot_pos])[None].copy(),
                foot_vel=np.stack([sd[sl] for sl in idx.s_foot_vel])[None].copy(),
                contact=_ref_contacts(d, idx)[None], action=a[None], last_action=a_prev[None], command=cmd[None],
                phase=np.array([phase]), friction=np.array([1.0]), default_q=idx.default_obs_q)


def test_matches_reference_step():
    """TASK.md kontrol adiminin env_cpu'dan bagimsiz, tek ortamlik yeniden yazimi ile birebir ayni sonuc
    (odulde action/last_action, air_time sirasi, gozlemde last_action = uygulanan aksiyon, sifirlama sonrasi gozlem)."""
    N = 4
    env = VecEnv(N, num_workers=0, seed=3, **OFF)
    idx, core = env.robot_index, env._core
    cmd = np.array([[0.15, 0.0, 0.0], [0.0, 0.0, 0.0], [0.05, -0.05, 0.5], [-0.1, 0.1, -0.8]])
    env.set_commands(cmd)
    obs, crit = env.reset()

    def sync(i):
        m = env.mj_model(i)
        d = mujoco.MjData(m)
        mujoco.mj_copyData(d, m, env.mj_data(i))
        return dict(m=m, d=d, phase=float(core.phase[i]), air=np.zeros(2), a_prev=np.zeros(10), k=0)

    def check_obs(i, o, c, s):
        assert np.array_equal(o[i], T.observation(np, s)[0].astype(np.float32)), i
        assert np.array_equal(c[i], T.critic_observation(np, s)[0].astype(np.float32)), i

    ref = [sync(i) for i in range(N)]
    for i in range(N):
        check_obs(i, obs, crit, _ref_state(ref[i]["d"], idx, np.zeros(10), np.zeros(10), cmd[i], ref[i]["phase"]))
    rng = np.random.default_rng(3)
    resets = firsts = 0
    for t in range(200):
        act = (0.8 * rng.standard_normal((N, 10))).astype(np.float32)
        o, c, rw, dn, info = env.step(act)
        for i in range(N):
            r = ref[i]
            m, d = r["m"], r["d"]
            a = np.clip(act[i].astype(np.float64), -C.ACTION_CLIP, C.ACTION_CLIP)
            ctrl = idx.init_ctrl.copy()
            ctrl[idx.action_act_id] = idx.default_action_q + C.ACTION_SCALE * a
            d.ctrl[:] = np.clip(ctrl, idx.ctrl_lo, idx.ctrl_hi)
            for _ in range(C.N_SUBSTEPS):
                mujoco.mj_step(m, d)
            r["phase"] = (r["phase"] + C.GAIT_FREQ * C.CTRL_DT) % 1.0
            r["k"] += 1
            s = _ref_state(d, idx, a, r["a_prev"], cmd[i], r["phase"])
            contact = s["contact"][0]
            first = ((contact > 0.5) & (r["air"] > 0.0)).astype(float)
            firsts += int(first.sum())
            s["air_time"], s["first_contact"] = (r["air"] + C.CTRL_DT)[None], first[None]
            term = T.terminated(np, s)[0]
            rew = np.float32(T.total_reward(np, T.reward_terms(np, s), term)[0])
            assert rew == rw[i], (t, i, rew, rw[i])
            assert bool(dn[i]) == bool(term > 0.5 or r["k"] >= C.MAX_EPISODE_STEPS), (t, i)
            r["air"] = np.where(contact > 0.5, 0.0, r["air"] + C.CTRL_DT)
            r["a_prev"] = a
            if dn[i]:
                resets += 1
                ref[i] = r = sync(i)
                check_obs(i, o, c, _ref_state(r["d"], idx, np.zeros(10), np.zeros(10), cmd[i], r["phase"]))
            else:
                check_obs(i, o, c, dict(s, last_action=a[None]))
    assert resets >= 2 and firsts >= 20, (resets, firsts)


def test_reset_state():
    """Sifirlama: xy = 0, z ve yonelim keyframe'den (yalniz dunya z etrafinda yaw), gurultu yalniz 14 gozlem
    ekleminde, qvel yalniz taban vx/vy, ctrl = keyframe."""
    N = 16
    env = VecEnv(N, num_workers=0, seed=5)
    idx = env.robot_index
    other = np.setdiff1d(np.arange(7, len(idx.init_qpos)), idx.obs_qpos_adr)
    g0 = T.projected_gravity(np, idx.init_qpos[3:7])
    yaw0 = T.yaw_angle(np, idx.init_qpos[3:7])
    yaws = []
    for _ in range(6):
        env.reset()
        for i in range(N):
            q, d = env.mj_data(i).qpos, env.mj_data(i)
            assert q[0] == 0.0 and q[1] == 0.0 and q[2] == idx.init_qpos[2]
            assert abs(np.linalg.norm(q[3:7]) - 1.0) < 1e-12
            assert np.allclose(T.projected_gravity(np, q[3:7]), g0, atol=1e-12)
            yaws.append(T.yaw_angle(np, q[3:7]) - yaw0)
            dq = q[idx.obs_qpos_adr] - idx.init_qpos[idx.obs_qpos_adr]
            assert np.all(np.abs(dq) <= C.RESET_JOINT_NOISE) and np.all(dq != 0.0)
            assert np.array_equal(q[other], idx.init_qpos[other])
            assert np.all(d.qvel[2:] == 0.0) and np.all(np.abs(d.qvel[:2]) <= C.RESET_BASE_VEL_NOISE)
            assert np.all(d.qvel[:2] != 0.0)
            assert np.array_equal(d.ctrl, idx.init_ctrl) and d.time == 0.0
    yaws = np.mod(np.array(yaws) + np.pi, 2 * np.pi) - np.pi
    assert yaws.min() < -2.5 and yaws.max() > 2.5


def test_head_push_noise_exact():
    """Bas filtresi (zamanlayici -> yeniden ornekleme -> filtre), itme (fizikten once qvel[0:2]) ve gozlem
    gurultusu (yalniz aktor) adimin cektigi duzgun sayilarla (core.U) birebir yeniden uretilir."""
    N = 6
    env = VecEnv(N, num_workers=0, seed=21, randomize=False, noise=True, head_random=True, push=True)
    idx, core, HR = env.robot_index, env._core, C.HEAD_RANDOM
    env.set_commands(np.zeros((N, 3)))
    env.reset()
    nvec = np.asarray(T.noise_vector(np))
    lo_h, hi_h = HR["resample_s"]
    lo_p, hi_p = C.DOMAIN_RAND["push_interval_s"]
    pushes = heads = 0
    rng = np.random.default_rng(0)
    snaps = [mujoco.MjData(env.mj_model(i)) for i in range(N)]
    for t in range(300):
        act = (0.1 * rng.standard_normal((N, 10))).astype(np.float32)
        for i in range(N):
            mujoco.mj_copyData(snaps[i], env.mj_model(i), env.mj_data(i))
        off0, tgt0 = core.head_offset.copy(), core.head_target.copy()
        ht0, pt0 = core.head_timer.copy(), core.push_timer.copy()
        o, c, rw, dn, info = env.step(act)
        U = core.U
        uh, up = U[:, core.o_head:core.o_head + 6], U[:, core.o_push:core.o_push + 3]
        un = U[:, core.o_noise:core.o_noise + C.NUM_OBS]
        assert np.allclose(o - c[:, :C.NUM_OBS], nvec * (2.0 * un - 1.0), atol=2e-6, rtol=0)
        for i in np.flatnonzero(~dn):
            timer, tgt = ht0[i] - C.CTRL_DT, tgt0[i]
            if timer <= 0.0:
                tgt = np.where(uh[i, 0] < HR["prob"], (2.0 * uh[i, 1:5] - 1.0) * idx.head_range, 0.0)
                timer = lo_h + (hi_h - lo_h) * uh[i, 5]
                heads += 1
            off = off0[i] + (tgt - off0[i]) * C.CTRL_DT / HR["filter_tau"]
            assert np.allclose(core.head_offset[i], off, atol=1e-15, rtol=0) and np.isclose(core.head_timer[i], timer)
            ctrl = idx.init_ctrl.copy()
            a = np.clip(act[i].astype(np.float64), -C.ACTION_CLIP, C.ACTION_CLIP)
            ctrl[idx.action_act_id] = idx.default_action_q + C.ACTION_SCALE * a
            ctrl[idx.head_act_id] += off
            ctrl = np.clip(ctrl, idx.ctrl_lo, idx.ctrl_hi)
            d = env.mj_data(i)
            assert np.allclose(d.ctrl, ctrl, atol=1e-15, rtol=0)
            d2 = snaps[i]
            ptimer = pt0[i] - C.CTRL_DT
            if ptimer <= 0.0:
                d2.qvel[0:2] += (2.0 * up[i, :2] - 1.0) * C.DOMAIN_RAND["push_vel"]
                ptimer = lo_p + (hi_p - lo_p) * up[i, 2]
                pushes += 1
            assert np.isclose(core.push_timer[i], ptimer)
            d2.ctrl[:] = d.ctrl
            mujoco.mj_step(env.mj_model(i), d2, C.N_SUBSTEPS)
            assert np.array_equal(d2.qpos, d.qpos) and np.array_equal(d2.qvel, d.qvel), (t, i)
    assert pushes >= 3 and heads >= 6, (pushes, heads)


def test_dr_set_const():
    """DR sonrasi mj_setConst'un turettigi alanlar guncel (or. alt agac kutlesi = rastgele kutlelerin toplami)."""
    env = VecEnv(2, num_workers=0, seed=3)
    nom = physics_model()
    for _ in range(3):
        env.reset()
        for i in range(2):
            m = env.mj_model(i)
            assert np.isclose(m.body_subtreemass[1], m.body_mass[1:].sum())
            assert not np.isclose(m.body_subtreemass[1], nom.body_subtreemass[1])
            m2 = physics_model()
            for f in ("geom_friction", "body_mass", "body_inertia", "body_ipos", "actuator_gainprm",
                      "actuator_biasprm", "dof_damping", "dof_frictionloss", "dof_armature"):
                getattr(m2, f)[:] = getattr(m, f)
            mujoco.mj_setConst(m2, mujoco.MjData(m2))
            for f in ("body_subtreemass", "body_invweight0", "dof_invweight0", "actuator_acc0", "dof_M0"):
                assert np.allclose(getattr(m, f), getattr(m2, f)), f


def test_worker_death():
    """Bir isci olurse (or. OOM killer) step RuntimeError verir, kalan isciler kapanir."""
    env = VecEnv(6, num_workers=2, seed=0)
    try:
        env.reset()
        z = np.zeros((6, C.NUM_ACTIONS), np.float32)
        env.step(z)
        procs = list(env._procs)
        os.kill(procs[1].pid, signal.SIGKILL)
        procs[1].join(5.0)
        try:
            env.step(z)
            raise AssertionError("step with a dead worker should fail")
        except RuntimeError as e:
            assert "worker 1 died" in str(e), e
        assert not any(p.is_alive() for p in procs)
        try:
            env.reset()
            raise AssertionError("env should stay broken")
        except RuntimeError:
            pass
    finally:
        env.close()


def test_worker_error_cleanup():
    """Iscide Python hatasi (kurulumda ya da adimda): RuntimeError + iscinin traceback'i, hic isci kalmaz."""
    import multiprocessing as mp
    import rl.env_cpu as E
    orig_init, orig_step = E._EnvSlice.__init__, E._EnvSlice.step
    where = {}

    def init(self, num_envs, lo, *a, **k):
        if where.get("init") == lo:
            raise ValueError("injected init failure")
        orig_init(self, num_envs, lo, *a, **k)
        self._test_lo = lo

    def step(self):
        if where.get("step") == self._test_lo:
            raise ValueError("injected step failure")
        orig_step(self)

    E._EnvSlice.__init__, E._EnvSlice.step = init, step
    try:
        where["init"] = 3        # fork: iscilere yamali sinif gecer
        try:
            VecEnv(6, num_workers=2, seed=0, start_method="fork")
            raise AssertionError("init failure not reported")
        except RuntimeError as e:
            assert "injected init failure" in str(e)
        assert not mp.active_children()
        where.clear()
        where["step"] = 3
        env = VecEnv(6, num_workers=2, seed=0, start_method="fork")
        env.reset()
        try:
            env.step(np.zeros((6, C.NUM_ACTIONS), np.float32))
            raise AssertionError("step failure not reported")
        except RuntimeError as e:
            assert "injected step failure" in str(e)
        assert not mp.active_children()
        env.close()
    finally:
        E._EnvSlice.__init__, E._EnvSlice.step = orig_init, orig_step


def test_sigint_cleanup():
    """Ctrl-C tum surec grubuna gider: ebeveyn KeyboardInterrupt ile cikar, isciler kalmaz."""
    import subprocess
    code = ("import numpy as np\n"
            "from rl.env_cpu import VecEnv\n"
            "env = VecEnv(16, num_workers=2, seed=0)\n"
            "env.reset()\n"
            "print(' '.join(str(p.pid) for p in env._procs), flush=True)\n"
            "while True:\n"
            "    env.step(np.zeros((16, 10), np.float32))\n")
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    p = subprocess.Popen([sys.executable, "-c", code], cwd=root, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, start_new_session=True)
    try:
        pids = [int(x) for x in p.stdout.readline().split()]
        assert len(pids) == 2
        time.sleep(0.5)
        os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout=20)
        err = p.stderr.read()
        assert "KeyboardInterrupt" in err
        assert "env worker" not in err

        def alive(pid):
            try:
                with open("/proc/%d/stat" % pid) as f:
                    return f.read().split()[2] != "Z"
            except FileNotFoundError:
                return False
        t = time.time()
        while any(alive(q) for q in pids) and time.time() - t < 10.0:
            time.sleep(0.05)
        assert not any(alive(q) for q in pids)
    finally:
        if p.poll() is None:
            p.kill()


# =====================================================================================
# main / getup gorevleri (rl/TASK.md "Tasks main and getup")
# =====================================================================================
from rl import tasks as TK                      # noqa: E402

MAIN, GETUP = TK.get_task("main"), TK.get_task("getup")
BANK_N = {"seated": 64, "fallen": 256, "midfall": 64}      # testlerde banka oneki (hiz icin)
H_HOME = C.H_HOME


def _main_env(n, **kw):
    kw.setdefault("bank_n", BANK_N)
    return VecEnv(n, task="main", **kw)


def _getup_env(n, **kw):
    kw.setdefault("bank_n", BANK_N)
    return VecEnv(n, task="getup", **kw)


def test_task_layouts_cpu():
    """Gozlem/aksiyon boyutlari ve dilimler TaskSpec ile ayni; ortam ornek ozellikleri gorevden."""
    assert (MAIN.num_obs, MAIN.num_critic_obs, MAIN.num_actions) == (53, 64, 10)
    assert (GETUP.num_obs, GETUP.num_critic_obs, GETUP.num_actions) == (48, 57, 14)
    assert MAIN.obs_slices["command"] == slice(44, 49) and MAIN.obs_slices["posture"] == slice(49, 51)
    assert MAIN.obs_slices["phase"] == slice(51, 53) and GETUP.obs_slices["last_action"] == slice(34, 48)
    assert len(T.noise_vector(np, "main")) == 53 and len(T.noise_vector(np, "getup")) == 48
    for task, n_act in (("walk", 10), ("main", 10), ("getup", 14)):
        spec = TK.get_task(task)
        for w in (0, 2):
            env = VecEnv(3, num_workers=w, seed=0, task=task, bank_n=BANK_N)
            try:
                assert (env.num_obs, env.num_critic_obs, env.num_actions) == (spec.num_obs, spec.num_critic_obs, n_act)
                obs, crit = env.reset()
                assert obs.shape == (3, spec.num_obs) and crit.shape == (3, spec.num_critic_obs)
                obs, crit, rew, done, info = env.step(np.zeros((3, n_act), np.float32))
                assert obs.dtype == np.float32 and crit.dtype == np.float32 and rew.dtype == np.float32
                assert np.all(np.isfinite(obs)) and np.all(np.isfinite(crit)) and np.all(np.isfinite(rew))
                assert env.reward_keys == list(spec.reward_weights)
                if task != "walk":
                    try:
                        env.step(np.zeros((3, 24 - n_act), np.float32))
                        raise AssertionError("wrong action width accepted")
                    except ValueError:
                        pass
            finally:
                env.close()
    # VecEnv sinif degerleri walk'ta kalir (geri uyum)
    assert VecEnv.num_obs == C.NUM_OBS and VecEnv.num_actions == C.NUM_ACTIONS


def test_main_set_commands_padding():
    """main: (N,3) -> [vx,vy,wz,H_HOME,0]; (N,5) aynen; kilit sifirlamadan sonra da korunur; refresh_obs."""
    I_C = MAIN.obs_slices["command"]
    hn = lambda h: (h - H_HOME) / C.H_NORM
    c3 = np.array([[0.2, 0.0, 0.0], [0.0, 0.05, 0.5], [-0.1, 0.0, -0.3]])
    c5 = np.array([[0.1, 0.0, 0.0, 0.095, 0.0], [0.0, 0.0, 0.3, 0.075, 0.0], [0.0, 0.0, 0.0, 0.1, 1.0]])
    for w in (0, 2):
        env = _main_env(3, num_workers=w, seed=12, fixed_command=[0.1, 0.0, 0.0], reset_probs={"home": 1.0},
                        noise=False)
        try:
            obs, _ = env.reset()
            assert np.allclose(obs[:, I_C], [0.1, 0.0, 0.0, 0.0, 0.0])
            obs, crit = env.set_commands(c3, refresh_obs=True)
            assert np.allclose(obs[:, I_C], np.c_[c3, np.zeros((3, 2))], atol=1e-7)
            assert np.array_equal(crit[:, :MAIN.num_obs], obs)          # gurultu kapali
            obs, _ = env.set_commands(c5, refresh_obs=True)
            exp = np.c_[c5[:, :3], hn(c5[:, 3]), c5[:, 4]]
            exp[1, :3] = 0.0                    # h < H_MIN_WALK: hiz yok
            exp[2, :3] = 0.0                    # sit = 1: hiz yok
            assert np.allclose(obs[:, I_C], exp, atol=1e-6)
            for _ in range(int(7.0 / C.CTRL_DT)):        # segment suresinden uzun: kilitli komut degismez
                obs, crit, rew, done, info = env.step(np.zeros((3, 10), np.float32))
                assert np.allclose(obs[:, I_C], exp, atol=1e-6)
            env.set_commands(None)
        finally:
            env.close()
    env = _main_env(2, num_workers=0, seed=13, reset_probs={"home": 1.0}, **OFF)
    env.reset()
    env.set_commands(c5[:2])
    d = env.mj_data(0)
    d.qpos[3:7] = [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0]
    mujoco.mj_forward(env.mj_model(0), d)
    obs, crit, rew, done, info = env.step(np.zeros((2, 10), np.float32))
    assert done[0] and np.allclose(obs[:, I_C], np.c_[c5[:2, :3] * [[1], [0]], hn(c5[:2, 3]), c5[:2, 4]], atol=1e-6)
    for bad in (np.zeros((2, 4)), np.zeros((2, 6))):
        try:
            env.set_commands(bad)
            raise AssertionError("bad command width accepted")
        except ValueError:
            pass
    # walk yalniz 3, getup komutsuz
    try:
        VecEnv(2, num_workers=0, seed=0).set_commands(np.zeros((2, 5)))
        raise AssertionError("walk accepted 5-dim command")
    except ValueError:
        pass
    genv = _getup_env(2, num_workers=0, seed=0, reset_probs={"stand": 1.0})
    for f in (lambda: genv.set_commands(np.zeros((2, 3))),
              lambda: _getup_env(2, num_workers=0, fixed_command=[0.0, 0.0, 0.0])):
        try:
            f()
            raise AssertionError("getup accepted a command")
        except ValueError:
            pass


def test_main_segments_resampled():
    """Kilitsiz main: komut segment suresi U(3.5, 6.5) s'de bir yeniden orneklenir; sit'ten sonra sit gelmez;
    her kanal (h, sit) ilk adimdan itibaren degisken."""
    N = 24
    env = _main_env(N, num_workers=0, seed=14, reset_probs={"home": 1.0}, **OFF)
    env.reset()
    core = env._core
    lo, hi = C.MAIN["segment_s"]
    assert np.all(core.seg_timer >= lo - 1e-9) and np.all(core.seg_timer <= hi + 1e-9)
    c0 = core.command.copy()
    assert len(set(np.round(c0[:, 3], 6))) > 2 and 0.0 < c0[:, 4].mean() < 1.0
    changes, sits = 0, 0
    z = np.zeros((N, 10), np.float32)
    for k in range(int(round(7.0 / C.CTRL_DT))):
        prev, timer = core.command.copy(), core.seg_timer.copy()
        obs, crit, rew, done, info = env.step(z)
        ex = (timer - C.CTRL_DT <= 0.0) & ~done
        same = np.all(core.command == prev, axis=1)
        assert np.all(same[~ex & ~done])
        sits += int(np.sum(ex & (prev[:, 4] > 0.5) & (core.command[:, 4] > 0.5)))
        changes += int(ex.sum())
    assert changes >= N // 2 and sits == 0, (changes, sits)


def test_main_sit_zeroes_velocity_and_rewards():
    """sit = 1: gozlem ve odulde hizlar 0 (takip odulu durgunluk ister); yurume kapilari kapali."""
    env = _main_env(2, num_workers=0, seed=15, reset_probs={"home": 1.0}, **OFF)
    env.reset()
    env.set_commands([[0.25, 0.1, 0.8, H_HOME, 1.0], [0.25, 0.1, 0.8, H_HOME, 0.0]])
    obs, _ = env.observe()
    I_C = MAIN.obs_slices["command"]
    assert np.all(obs[0, I_C][:3] == 0.0) and obs[0, I_C][4] == 1.0
    assert np.allclose(obs[1, I_C][:3], [0.25, 0.1, 0.8])
    s = env.task_state()
    vel, h, sit = T.main_command(np, s["command"])
    assert np.all(vel[0] == 0.0) and sit.tolist() == [1.0, 0.0]
    moving, g_step, still = T.main_gates(np, s)
    assert moving.tolist() == [0.0, 1.0] and g_step[1] == 1.0 and still[0] == 1.0


def test_main_seated_start_and_relaxed_termination():
    """Oturus bankasindan sifirlama: govde kutusu yerde, iki ayak yerde, gz < -0.9, alpha = 1, h_ref = H_HOME,
    ilk segment %50 sit / %50 stand. sit=1 ve sifir aksiyonla 2 s dusme yok (walk'un yukseklik kurali yok);
    alpha < 0.3 iken govde yerde -> sonlanir; devrilme -> sonlanir."""
    N = 16
    env = _main_env(N, num_workers=0, seed=16, reset_probs={"seated": 1.0}, **OFF)
    obs, crit = env.reset()
    s = env.task_state()
    g = T.projected_gravity(np, s["quat"])
    assert np.all(np.abs(s["trunk_z"] - C.SEAT_Z) < 2 * C.SEAT_Z_TOL), s["trunk_z"]
    assert np.all(s["trunk_floor"] == 1.0) and np.all(s["contact"] == 1.0) and np.all(g[:, 2] < -0.9)
    assert np.all(s["alpha"] == 1.0) and np.all(s["h_ref"] == H_HOME) and np.all(s["reset_kind"] == 1.0)
    assert np.allclose(obs[:, MAIN.obs_slices["posture"]], [0.0, 1.0])
    assert 0 < s["command"][:, 4].sum() < N                    # %50 sit / %50 stand
    for i in range(N):
        d = env.mj_data(i)
        assert d.qpos[0] == 0.0 and d.qpos[1] == 0.0 and np.all(d.qvel == 0.0)
    yaws = T.yaw_angle(np, s["quat"])
    assert yaws.max() - yaws.min() > 2.0
    env.set_commands(np.tile([0.0, 0.0, 0.0, H_HOME, 1.0], (N, 1)))
    z = np.zeros((N, 10), np.float32)
    for k in range(int(2.0 / C.CTRL_DT)):
        obs, crit, rew, done, info = env.step(z)
        assert not done.any(), k
    s = env.task_state()
    assert np.all(s["trunk_floor"] == 1.0) and np.all(s["trunk_z"] < C.TERMINATION_MIN_HEIGHT)
    assert np.all(crit[:, MAIN.critic_slices["trunk_floor"]] == 1.0)
    # planlanmamis oturma: alpha < unplanned_sit_alpha iken govde yerde (servolar gevsek: govde yerde kalir)
    env._core.alpha[:2] = 0.0
    for i in range(2):
        m = env.mj_model(i)
        m.actuator_gainprm[:, 0] = 0.0
        m.actuator_biasprm[:, 1] = 0.0
    d = env.mj_data(3)
    d.qpos[3:7] = [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0]
    mujoco.mj_forward(env.mj_model(3), d)
    obs, crit, rew, done, info = env.step(z)
    assert done[:2].all() and done[3] and not done[2] and not done[4:].any()
    assert not info["time_outs"].any() and rew[0] < C.TERMINATION_PENALTY + 1.0
    ep = info["episode"]
    assert set(ep) == {"ep_len"} | {"rew_" + k for k in MAIN.reward_weights} | set(MAIN.episode_metrics)
    assert ep["falls_sit"] > 0.0 and ep["sit_steps"] > 0.0 and ep["seated_ok_steps"] > 0.0


def test_terminate_false():
    """terminate=False: devrilme bolumu bitirmez (yalniz bozuk fizik / zaman asimi)."""
    for task in ("walk", "main"):
        env = VecEnv(2, num_workers=0, seed=2, task=task, terminate=False, reset_probs=None if task == "walk"
                     else {"home": 1.0}, bank_n=BANK_N, **OFF)
        env.reset()
        d = env.mj_data(1)
        d.qpos[3:7] = [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0]
        mujoco.mj_forward(env.mj_model(1), d)
        for _ in range(20):
            obs, crit, rew, done, info = env.step(np.zeros((2, 10), np.float32))
            assert not done.any()
        assert obs[1, I_GRAV_Z] > -0.6


def test_main_action_mapping_and_posture_filter():
    """main: bacak hedefi = q_ref(h_ref, alpha) + 0.5*clip(a) (GUNCEL durus), bas = HOME + head_offset_ref;
    adim sonunda h_ref/alpha = posture_step(komut); oturus scripti HEAD_SIT'e gider."""
    N = 4
    env = _main_env(N, num_workers=0, seed=17, reset_probs={"home": 1.0}, **OFF)
    env.reset()
    idx, core = env.robot_index, env._core
    env.set_commands([[0.0, 0.0, 0.0, 0.08, 0.0], [0.0, 0.0, 0.0, H_HOME, 1.0], [0.1, 0.0, 0.0, 0.1, 0.0],
                      [0.0, 0.0, 0.0, 0.07, 1.0]])
    core.h_ref[:] = [0.09, 0.1, H_HOME, 0.075]
    core.alpha[:] = [0.0, 0.3, 0.0, 0.8]
    rng = np.random.default_rng(1)
    checked = 0
    for t in range(40):
        h0, a0 = core.h_ref.copy(), core.alpha.copy()
        act = (2.0 * rng.standard_normal((N, 10))).astype(np.float32)
        obs, crit, rew, done, info = env.step(act)
        ref = T.posture_ref(np, h0, a0, idx.default_obs_q)
        a = np.clip(act.astype(np.float64), -C.ACTION_CLIP, C.ACTION_CLIP)
        leg = np.clip(ref["q_ref"] + C.ACTION_SCALE * a, idx.ctrl_lo[idx.action_act_id], idx.ctrl_hi[idx.action_act_id])
        head = np.clip(idx.init_ctrl[idx.head_act_id] + ref["head_offset"], idx.ctrl_lo[idx.head_act_id],
                       idx.ctrl_hi[idx.head_act_id])
        h1, a1 = T.posture_step(np, h0, a0, core.command)
        for i in np.flatnonzero(~done):
            checked += 1
            assert np.allclose(env.mj_data(i).ctrl[idx.action_act_id], leg[i], atol=1e-12), (t, i)
            assert np.allclose(env.mj_data(i).ctrl[idx.head_act_id], head[i], atol=1e-12), (t, i)
            assert core.h_ref[i] == h1[i] and core.alpha[i] == a1[i]
            assert np.allclose(obs[i, MAIN.obs_slices["posture"]], [(h1[i] - H_HOME) / C.H_NORM, a1[i]], atol=1e-6)
            assert np.allclose(obs[i, MAIN.obs_slices["last_action"]], a[i], atol=1e-6)
    assert checked > 60, checked
    # oturus scripti: alpha >= SIT_SPLIT iken bas HEAD_SIT'te
    hs = np.deg2rad([C.HEAD_SIT_DEG[j] for j in C.HEAD_JOINTS])
    assert np.allclose(T.posture_ref(np, np.array([H_HOME]), np.array([0.7]), idx.default_obs_q)["head_offset"][0]
                       + idx.default_obs_q[10:], hs)
    # main modeli: diz siniri 65 derece (ctrlrange)
    k = np.deg2rad(C.KNEE_LIMIT_DEG)
    li, ri = idx.action_act_id[3], idx.action_act_id[8]
    assert np.isclose(idx.ctrl_hi[li], k) and np.isclose(idx.ctrl_lo[ri], -k)


def _ref_state_task(d, idx, task, a, a_prev, **extra):
    s = _ref_state(d, idx, a, a_prev, np.zeros(3), 0.0)
    f = np.zeros(4)
    for k in range(d.ncon):
        con = d.contact[k]
        g = (int(con.geom1), int(con.geom2))
        if idx.floor_geom in g and con.dist < C.CONTACT_EPS:
            other = g[1] if g[0] == idx.floor_geom else g[0]
            for j, gs in enumerate([idx.foot_geom[:1], idx.foot_geom[1:], idx.trunk_geoms, idx.head_geoms]):
                if other in gs:
                    f[j] = 1.0
    s["contact"] = f[None, :2]
    s["trunk_floor"], s["head_floor"] = f[None, 2], f[None, 3]
    if task == "getup":
        del s["command"], s["phase"]
        s["head_vel"] = d.sensordata[idx.s_head_vel][None].copy()
    s.update(extra)
    return s


def test_main_matches_reference_step():
    """TASK.md main kontrol adiminin env_cpu'dan bagimsiz, tek ortamlik yeniden yazimi ile birebir ayni sonuc
    (q_ref guncel durustan, bas scripti, adim sonunda durus filtresi, odul/sonlandirma/olaylar, gozlem)."""
    N = 3
    env = _main_env(N, num_workers=0, seed=3, reset_probs={"home": 1.0}, **OFF)
    idx, core = env.robot_index, env._core
    cmd = np.array([[0.15, 0.0, 0.0, H_HOME, 0.0], [0.0, 0.0, 0.3, 0.09, 0.0], [0.05, 0.0, 0.0, H_HOME, 1.0]])
    env.set_commands(cmd)
    obs, crit = env.reset()
    w = T.reward_weights(np, "main")

    def sync(i):
        m = env.mj_model(i)
        d = mujoco.MjData(m)
        mujoco.mj_copyData(d, m, env.mj_data(i))
        return dict(m=m, d=d, phase=float(core.phase[i]), air=np.zeros(2), a_prev=np.zeros(10), k=0,
                    h=np.array([H_HOME]), al=np.array([0.0]), ev=T.main_events_init(np, np.array([0.0])),
                    met=np.zeros(len(MAIN.episode_metrics)))

    def state(i, r, a, a_prev):
        return _ref_state_task(r["d"], idx, "main", a, a_prev, command=cmd[i][None],
                               phase=np.array([r["phase"]]), h_ref=r["h"], alpha=r["al"])

    def check_obs(i, o, c, s):
        assert np.array_equal(o[i], T.observation(np, s, "main")[0].astype(np.float32)), i
        assert np.array_equal(c[i], T.critic_observation(np, s, "main")[0].astype(np.float32)), i

    ref = [sync(i) for i in range(N)]
    for i in range(N):
        check_obs(i, obs, crit, state(i, ref[i], np.zeros(10), np.zeros(10)))
    rng = np.random.default_rng(3)
    resets = 0
    for t in range(250):
        act = (0.6 * rng.standard_normal((N, 10))).astype(np.float32)
        o, c, rw, dn, info = env.step(act)
        for i in range(N):
            r = ref[i]
            m, d = r["m"], r["d"]
            pr = T.posture_ref(np, r["h"], r["al"], idx.default_obs_q)
            a = np.clip(act[i].astype(np.float64), -C.ACTION_CLIP, C.ACTION_CLIP)
            ctrl = idx.init_ctrl.copy()
            ctrl[idx.action_act_id] = pr["q_ref"][0] + C.ACTION_SCALE * a
            ctrl[idx.head_act_id] = idx.init_ctrl[idx.head_act_id] + pr["head_offset"][0]
            d.ctrl[:] = np.clip(ctrl, idx.ctrl_lo, idx.ctrl_hi)
            mujoco.mj_step(m, d, C.N_SUBSTEPS)
            r["phase"] = (r["phase"] + C.GAIT_FREQ * C.CTRL_DT) % 1.0
            r["k"] += 1
            r["h"], r["al"] = T.posture_step(np, r["h"], r["al"], cmd[i][None])
            s = state(i, r, a, r["a_prev"])
            contact = s["contact"][0]
            first = ((contact > 0.5) & (r["air"] > 0.0)).astype(float)
            s["air_time"], s["first_contact"] = (r["air"] + C.CTRL_DT)[None], first[None]
            term = T.terminated(np, s, "main")
            rew = np.float32(T.total_reward(np, T.reward_terms(np, s, "main"), term, w)[0])
            assert rew == rw[i], (t, i, rew, rw[i])
            assert bool(dn[i]) == bool(term[0] > 0.5 or r["k"] >= MAIN.max_episode_steps), (t, i)
            r["ev"], inc = T.main_events(np, s, r["ev"], term)
            r["met"] += [inc[k][0] for k in MAIN.episode_metrics]
            r["air"] = np.where(contact > 0.5, 0.0, r["air"] + C.CTRL_DT)
            r["a_prev"] = a
            if dn[i]:
                resets += 1
                assert info["episode"]["ep_len"] > 0
                ref[i] = r = sync(i)
                check_obs(i, o, c, state(i, r, np.zeros(10), np.zeros(10)))
            else:
                check_obs(i, o, c, state(i, r, a, a))
                assert np.allclose(core.ep_met[i], r["met"]), (t, i)
    assert resets >= 1, resets


def test_getup_matches_reference_step_and_mapping():
    """getup: 14 aksiyon = clip(HOME + S*clip(a, +-1), ctrlrange), S = action_scale_vector; gaga HOME'da; bas
    rastgeleligi yok sayilir; bellek (phi/head_floor_prev) ve mufredat agirliklari (progress) ile odul, gozlem
    bagimsiz yeniden yazimla birebir ayni; dusme sonlandirmasi yok."""
    N = 3
    p = 0.6
    env = _getup_env(N, num_workers=0, seed=5, reset_probs={"fallen": 0.5, "stand": 0.5}, progress=p,
                     randomize=False, noise=False, head_random=True, push=False)
    idx, core = env.robot_index, env._core
    assert env.num_actions == 14 and core.n_uniform == 0 and not core.head_random
    S = TK.action_scale_vector("getup", idx.default_action_q, idx.action_lo, idx.action_hi)
    assert np.allclose(S, np.maximum(np.abs(idx.action_hi - idx.default_obs_q), np.abs(idx.action_lo - idx.default_obs_q)))
    obs, crit = env.reset()
    w = T.reward_weights(np, "getup", p)
    assert all(np.isclose(core.weights[k], w[k]) for k in w) and core.weights["head_impact"] != 0.0

    def sync(i):
        m = env.mj_model(i)
        d = mujoco.MjData(m)
        mujoco.mj_copyData(d, m, env.mj_data(i))
        s0 = _ref_state_task(d, idx, "getup", np.zeros(14), np.zeros(14))
        return dict(m=m, d=d, a_prev=np.zeros(14), k=0, mem=T.getup_memory(np, s0),
                    ev=T.getup_events_init(np, s0["trunk_z"]))

    def check_obs(i, o, c, s):
        assert np.array_equal(o[i], T.observation(np, s, "getup")[0].astype(np.float32)), i
        assert np.array_equal(c[i], T.critic_observation(np, s, "getup")[0].astype(np.float32)), i

    ref = [sync(i) for i in range(N)]
    for i in range(N):
        check_obs(i, obs, crit, _ref_state_task(ref[i]["d"], idx, "getup", np.zeros(14), np.zeros(14),
                                                **ref[i]["mem"]))
    rng = np.random.default_rng(4)
    for t in range(150):
        act = (1.5 * rng.standard_normal((N, 14))).astype(np.float32)
        o, c, rw, dn, info = env.step(act)
        assert not dn.any()
        for i in range(N):
            r = ref[i]
            m, d = r["m"], r["d"]
            a = np.clip(act[i].astype(np.float64), -1.0, 1.0)
            ctrl = idx.init_ctrl.copy()
            ctrl[idx.action_act_id] = np.clip(idx.default_obs_q + S * a, idx.action_lo, idx.action_hi)
            d.ctrl[:] = np.clip(ctrl, idx.ctrl_lo, idx.ctrl_hi)
            assert np.array_equal(env.mj_data(i).ctrl, d.ctrl), (t, i)
            mujoco.mj_step(m, d, C.N_SUBSTEPS)
            r["k"] += 1
            s = _ref_state_task(d, idx, "getup", a, r["a_prev"], **r["mem"])
            rew = np.float32(T.total_reward(np, T.reward_terms(np, s, "getup"), np.zeros(1), w)[0])
            assert rew == rw[i], (t, i, rew, rw[i])
            r["mem"] = T.getup_memory(np, s)
            r["a_prev"] = a
            check_obs(i, o, c, _ref_state_task(d, idx, "getup", a, a, **r["mem"]))
    # 10 genislikli aksiyon reddedilir
    try:
        env.step(np.zeros((N, 10), np.float32))
        raise AssertionError("10-wide action accepted")
    except ValueError:
        pass


def test_bank_resets_identical_across_workers():
    """Dusmus/dusus-ortasi/ayakta (getup) ve oturus (main) bankasindan sifirlamalar isci sayisindan bagimsiz
    (bankalar ebeveynde bir kez uretilir); dusmus sifirlamalarin cogu devrik, qvel = 0, xy = 0."""
    ref = _rollout(_getup_env(6, num_workers=0, seed=11, episode_details=True), 60, std=1.0)
    for w, sm in ((2, "forkserver"), (3, "spawn")):
        env = _getup_env(6, num_workers=w, seed=11, start_method=sm, episode_details=True)
        try:
            got = _rollout(env, 60, std=1.0)
        finally:
            env.close()
        for k, (x, y) in enumerate(zip(ref, got)):
            assert _same(x, y), "getup workers=%d (%s) differs at step %d" % (w, sm, k)
    ref = _rollout(_main_env(5, num_workers=0, seed=12), 120, std=1.0)
    assert any(x[3].any() for x in ref[1:])
    env = _main_env(5, num_workers=2, seed=12, start_method="fork")
    try:
        got = _rollout(env, 120, std=1.0)
    finally:
        env.close()
    assert all(_same(x, y) for x, y in zip(ref, got))
    # dusmus banka sifirlamasi
    env = _getup_env(16, num_workers=0, seed=7, reset_probs={"fallen": 1.0}, **OFF)
    gz, labels = [], []
    for _ in range(4):
        env.reset()
        s = env.task_state()
        gz += T.projected_gravity(np, s["quat"])[:, 2].tolist()
        labels += s["bank_label"].tolist()
        for i in range(16):
            d = env.mj_data(i)
            assert d.qpos[0] == 0.0 and d.qpos[1] == 0.0 and np.all(d.qvel == 0.0)
        assert np.all(s["reset_kind"] == 0.0)
    assert np.mean(np.array(gz) > -0.6) > 0.9 and len(set(labels)) >= 3
    for _ in range(100):
        obs, crit, rew, done, info = env.step(np.zeros((16, 14), np.float32))
        assert not done.any()
    # dusus ortasi: acisal hiz bankadan, egik
    env = _getup_env(8, num_workers=0, seed=8, reset_probs={"midfall": 1.0}, **OFF)
    env.reset()
    s = env.task_state()
    tilt = np.rad2deg(T.tilt_angle(np, s["quat"]))
    assert np.all(tilt > 10.0) and np.all(tilt < 40.0) and np.all(s["reset_kind"] == 1.0)
    assert any(np.linalg.norm(env.mj_data(i).qvel[3:6]) > 0.1 for i in range(8))


def test_reset_probs_validation_and_progress():
    for bad in ({"seated": 1.0, "fallen": 0.5}, {"home": 0.0}):
        try:
            VecEnv(2, num_workers=0, task="main", reset_probs=bad, bank_n=BANK_N)
            raise AssertionError("bad reset_probs accepted: %r" % (bad,))
        except ValueError:
            pass
    # set_progress: isci ve surec ici ayni agirliklar
    a = _rollout(_getup_env(4, num_workers=0, seed=3, progress=1.0), 30)
    env = _getup_env(4, num_workers=2, seed=3, progress=0.0)
    try:
        env.set_progress(1.0)
        assert env.progress == 1.0
        b = _rollout(env, 30)
    finally:
        env.close()
    assert all(_same(x, y) for x, y in zip(a, b))
    c = _rollout(_getup_env(4, num_workers=0, seed=3, progress=0.0), 30)
    assert not all(_same(x, y) for x, y in zip(a, c))


def test_set_state_handoff_and_episode_details():
    """set_state: fiziksel durum aktarilir, last_action = 0, alpha = 0, h_ref = H_HOME, sayaclar sifir; getup
    bellegi yeni durumdan. episode_details: info["episodes"] sutunlari."""
    genv = _getup_env(1, num_workers=0, seed=1, reset_probs={"fallen": 1.0}, episode_details=True,
                      max_episode_steps=50, **OFF)
    menv = _main_env(1, num_workers=0, seed=2, reset_probs={"home": 1.0}, model_variant="getup", **OFF)
    genv.reset()
    menv.reset()
    for _ in range(20):
        go, *_ = genv.step(np.ones((1, 14), np.float32))
        mo, *_ = menv.step(np.ones((1, 10), np.float32))
    assert np.all(mo[0, MAIN.obs_slices["last_action"]] == 1.0)
    d = genv.mj_data(0)
    obs, crit = menv.set_state(0, d.qpos.copy(), d.qvel.copy(), warmstart=d.qacc_warmstart.copy())
    s = menv.task_state()
    assert np.array_equal(menv.mj_data(0).qpos, d.qpos) and np.array_equal(menv.mj_data(0).qvel, d.qvel)
    assert np.all(obs[0, MAIN.obs_slices["last_action"]] == 0.0) and s["step_count"][0] == 0
    assert np.allclose(obs[0, MAIN.obs_slices["posture"]], [0.0, 0.0]) and s["h_ref"][0] == H_HOME
    assert np.allclose(obs[0, 6:34], go[0, 6:34], atol=1e-6)         # ayni eklem durumu
    assert np.allclose(obs[0, :3], go[0, :3], atol=0.02)             # sensorler son alt adimdan (yaklasik)
    d2 = menv.mj_data(0)
    obs, crit = genv.set_state(0, d2.qpos.copy(), d2.qvel.copy())
    g = genv.task_state()
    assert np.all(obs[0, GETUP.obs_slices["last_action"]] == 0.0)
    assert np.isclose(g["phi_up_prev"][0], -T.projected_gravity(np, g["quat"])[0, 2])
    for k in range(50):
        obs, crit, rew, done, info = genv.step(np.zeros((1, 14), np.float32))
    assert done[0] and info["time_outs"][0]
    ep = info["episodes"]
    assert set(ep) == set(env_cols("getup")) | {"env"} and ep["ep_len"][0] == 50 and ep["reset_kind"][0] == -1.0


def test_main_locked_reset_samples_segment():
    """Kilitli komutla (fixed_command / set_commands) main sifirlamasi: ilk segment (seg, seg_timer) yine zaten
    cekilmis sayilardan orneklenir (MJX ile ayni), yalniz komut kilitli degerle ezilir; ek RNG tuketilmez."""
    N, seed, fixed = 6, 21, [0.1, 0.0, 0.0, 0.1, 0.0]
    lk = _main_env(N, num_workers=0, seed=seed, fixed_command=fixed, **OFF)
    fr = _main_env(N, num_workers=0, seed=seed, **OFF)
    lk.reset()
    fr.reset()
    a, b = lk.task_state(), fr.task_state()
    assert a["locked"] and not b["locked"]
    assert np.array_equal(a["seg"], b["seg"]) and np.array_equal(a["seg_timer"], b["seg_timer"])
    assert np.allclose(a["command"], fixed) and not np.allclose(b["command"], fixed)
    lo, hi = C.MAIN["segment_s"]
    assert np.all(a["seg_timer"] >= lo) and np.all(a["seg_timer"] <= hi)
    for j in range(N):                  # RNG akislari ayni noktada
        assert lk._core.rngs[j].bit_generator.state == fr._core.rngs[j].bit_generator.state
    # bagimsiz yeniden hesap: sifirlama blogunun son 8 sayisi (tur, banka indeksi sonrasi) sample_segment'e
    core = lk._core
    for j in range(N):
        g = np.random.Generator(np.random.PCG64(np.random.SeedSequence(seed).spawn(N)[j]))
        u = g.random(core.n_reset_u)
        ux = u[len(core.idx.obs_qpos_adr) + 10:]
        al = np.array([1.0 if core.kinds[int(T.sample_reset_kind(np, ux[0], core.rspec))] == "seated" else 0.0])
        kd, cmd, dur = T.sample_segment(np, ux[None, 2:2 + T.NUM_SEGMENT_U], T.reset_command(np, al), seated=al)
        assert a["seg"][j] == kd[0] and a["seg_timer"][j] == dur[0] and np.array_equal(b["command"][j], cmd[0])
    # adimda kilitli komut degismez, zamanlayici isler
    z = np.zeros((N, 10), np.float32)
    for _ in range(3):
        lk.step(z)
    s = lk.task_state()
    assert np.allclose(s["command"][:, :3], fixed[:3]) and np.allclose(s["seg_timer"], a["seg_timer"] - 3 * C.CTRL_DT)


def test_getup_memory_keys_generic():
    """getup bellegi anahtarlari T.getup_memory'den gelir (env_cpu sabit liste tutmaz); sifirlamada bellek
    sifirlama durumundan hesaplanir."""
    env = _getup_env(3, num_workers=0, seed=4, reset_probs={"midfall": 1.0}, **OFF)
    env.reset()
    s = env.task_state()
    mem = T.getup_memory(np, s)
    assert set(env._core.mem) == set(mem) and set(mem) <= set(s)
    for k, v in mem.items():
        assert np.allclose(s[k], v), k
    env.step(np.zeros((3, 14), np.float32))
    s = env.task_state()
    assert set(env._core.mem) == set(mem) and all(np.all(np.isfinite(s[k])) for k in mem)


def test_head_force_peak_flag():
    """head_force_peak=True: fizik alt adim alt adim (dinamik/gozlem/odul bit-duzeyinde ayni), task_state()
    ["head_force_peak"] her kontrol adiminda >= kontrol adimi sonundaki tek ornek (eval_tasks.head_force) ve
    darbelerde daha buyuk."""
    from rl.eval_tasks import Evaluator
    kw = dict(num_workers=0, seed=3, reset_probs={"midfall": 1.0}, progress=1.0, **OFF)
    a, b = _getup_env(4, **kw), _getup_env(4, head_force_peak=True, **kw)
    a.reset()
    b.reset()
    assert "head_force_peak" not in a.task_state() and np.all(b.task_state()["head_force_peak"] == 0.0)
    rng = np.random.default_rng(0)
    more = pk_max = 0
    for k in range(150):
        act = (0.3 * rng.standard_normal((4, 14))).astype(np.float32)
        ra, rb = a.step(act), b.step(act)
        for x, y in zip(ra[:4], rb[:4]):
            assert np.array_equal(x, y), k
        for i in range(4):
            assert np.array_equal(a.mj_data(i).qpos, b.mj_data(i).qpos)
            assert np.array_equal(a.mj_data(i).qvel, b.mj_data(i).qvel)
        pk = b.task_state()["head_force_peak"]
        smp = Evaluator.head_force(b, 4)
        assert np.all(pk >= smp - 1e-12), (k, pk, smp)
        more += int(np.sum(pk > smp + 1e-6))
        pk_max = max(pk_max, float(pk.max()))
    assert pk_max > 1.0 and more > 0, (pk_max, more)
    # walk: bayrak acik da ayni dinamik
    w0 = _rollout(VecEnv(2, num_workers=0, seed=5), 40)
    w1 = _rollout(VecEnv(2, num_workers=0, seed=5, head_force_peak=True), 40)
    assert all(_same(x, y) for x, y in zip(w0, w1))


def env_cols(task):
    from rl.env_cpu import episode_columns
    return episode_columns(task)


def main():
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    if len(sys.argv) > 1:
        tests = [(k, v) for k, v in tests if any(a in k for a in sys.argv[1:])]
    failed = 0
    for name, fn in tests:
        t = time.time()
        try:
            fn()
            print("PASS  %-40s %6.2fs" % (name, time.time() - t), flush=True)
        except Exception:
            failed += 1
            import traceback
            print("FAIL  %-40s %6.2fs" % (name, time.time() - t), flush=True)
            traceback.print_exc()
    print("%d/%d passed" % (len(tests) - failed, len(tests)))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
