# -*- coding: utf-8 -*-
"""
rl/env_mjx.py ve rl/train_mjx.py (ONNX) testleri. Calistirma (repo kokunden):

  python3 -m rl.tests.test_env_mjx          # duz assert'ler
  python3 -m pytest rl/tests/test_env_mjx.py   # (pytest kuruluysa)

CPU'da JIT derlemeleri toplam birkac dakika surer; derlenen fonksiyonlar testler arasinda paylasilir.
"""
import os
import sys
import tempfile
import time

import jax
import jax.numpy as jnp
import mujoco
import numpy as np
from mujoco import mjx

from rl import config as C
from rl import policy_meta as PM
from rl import pose_bank as PB
from rl import task_math as T
from rl import tasks as TK
from rl.env_mjx import OpenDuckEnv, VmapAutoReset, extract_state, wrap_for_brax_training

# kritik gozlem yerlesimi
I_GRAV_Z, I_TRUNK_Z, I_CONTACT, I_FRICTION = 2, 52, slice(53, 55), 57
_CACHE = {}


def _plain():
    """DR/gurultu/bas/itme kapali, sabit sifir komut; 2 ortam."""
    if "plain" not in _CACHE:
        env = OpenDuckEnv(randomize=False, noise=False, head_random=False, push=False, fixed_command=(0.0, 0.0, 0.0))
        b = VmapAutoReset(env)
        _CACHE["plain"] = (env, jax.jit(b.reset), jax.jit(b.step))
    return _CACHE["plain"]


def _full():
    if "full" not in _CACHE:
        env = OpenDuckEnv()
        b = VmapAutoReset(env)
        _CACHE["full"] = (env, jax.jit(b.reset), jax.jit(b.step))
    return _CACHE["full"]


def _calm():
    """Itme ve rastgele bas hareketi kapali tam ortam. Kucuk robot (kafa+boyun kutlenin ~%40'i)
    sifir aksiyonla bunlar acikken 6 s ayakta kalamaz; komut testleri dusmeden calismali."""
    if "calm" not in _CACHE:
        env = OpenDuckEnv(head_random=False, push=False)
        b = VmapAutoReset(env)
        _CACHE["calm"] = (env, jax.jit(b.reset), jax.jit(b.step))
    return _CACHE["calm"]


def _keys(n, seed=0):
    return jax.random.split(jax.random.PRNGKey(seed), n)


def _set_qpos(st, i, qpos, qvel=None):
    d = st.data
    qp = d.qpos.at[i].set(qpos)
    qv = d.qvel.at[i].set(0.0 if qvel is None else qvel)
    return st.replace(data=d.replace(qpos=qp, qvel=qv))


def test_reset_step_shapes():
    env, reset, step = _full()
    n = 4
    st = reset(_keys(n))
    assert st.obs["state"].shape == (n, C.NUM_OBS) and st.obs["state"].dtype == jnp.float32
    assert st.obs["privileged_state"].shape == (n, C.NUM_CRITIC_OBS)
    assert env.observation_size == {"state": (C.NUM_OBS,), "privileged_state": (C.NUM_CRITIC_OBS,)}
    assert env.action_size == C.NUM_ACTIONS
    rng = np.random.default_rng(0)
    for _ in range(3):
        st = step(st, jnp.asarray(rng.standard_normal((n, C.NUM_ACTIONS)), jnp.float32))
        assert st.reward.shape == (n,) and st.done.shape == (n,)
        assert st.reward.dtype == jnp.float32 and st.done.dtype == jnp.float32
        for k in ("truncation", "time_out", "episode_done", "steps"):
            assert st.info[k].shape == (n,)
        assert np.all(np.isfinite(np.asarray(st.obs["state"])))
        assert np.all(np.isfinite(np.asarray(st.obs["privileged_state"])))
        assert np.all(np.isfinite(np.asarray(st.reward)))
    assert set(st.metrics) == {"rew_" + k for k in C.REWARD_WEIGHTS}
    # komut araliklari ve DR surtunmesi kritik gozlemde
    cmd = np.asarray(st.info["command"])
    for j, k in enumerate(("vx", "vy", "wz")):
        lo, hi = C.COMMAND_RANGES[k]
        assert np.all((cmd[:, j] >= lo - 1e-6) & (cmd[:, j] <= hi + 1e-6))
    fr = np.asarray(st.obs["privileged_state"][:, I_FRICTION])
    assert np.allclose(fr, np.asarray(st.info["dr"]["friction"]))
    assert np.all((fr >= C.DOMAIN_RAND["friction"][0]) & (fr <= C.DOMAIN_RAND["friction"][1]))


def test_stand_still_1s():
    """Sifir aksiyonla 1 s ayakta: dusmez, govde yuksekligi/egimi korunur; C ile ayni fizik."""
    env, reset, step = _plain()
    st = reset(_keys(2, 1))
    q0 = jnp.asarray(env.idx.init_qpos, jnp.float32)
    st = _set_qpos(st, 0, q0)
    st = _set_qpos(st, 1, q0)
    a = jnp.zeros((2, C.NUM_ACTIONS))
    n = int(round(1.0 / C.CTRL_DT))
    for _ in range(n):
        st = step(st, a)
        assert not np.any(np.asarray(st.done))
    crit = np.asarray(st.obs["privileged_state"])
    assert np.all(np.abs(crit[:, I_TRUNK_Z] - env.idx.init_qpos[2]) < 0.01), crit[:, I_TRUNK_Z]
    assert np.all(crit[:, I_GRAV_Z] < -0.99)
    assert np.all(crit[:, I_CONTACT] == 1.0)
    qpos = np.asarray(st.data.qpos)
    assert np.all(np.abs(qpos[:, :2]) < 0.01)
    assert np.all(np.asarray(st.info["steps"]) == n)

    # ayni baslangic + ayni ctrl ile MuJoCo C (float64)
    m = env.mj_model
    d = mujoco.MjData(m)
    d.qpos[:] = env.idx.init_qpos
    d.ctrl[:] = np.clip(env.idx.init_ctrl, env.idx.ctrl_lo, env.idx.ctrl_hi)
    mujoco.mj_step(m, d, n * C.N_SUBSTEPS)
    err = np.abs(qpos[0] - d.qpos).max()
    assert err < 2e-3, err


def test_dynamics_match_c():
    """Sert rastgele aksiyonlarla 10 kontrol adimi (50 alt adim) zeminde: ortamin fizigi MuJoCo C ile ayni
    (c_semantics: kirpilmis aktuator turevi + C duzlem-kutu temasi). Fark float32 kaynakli (~1e-4);
    c_semantics olmadan ~1e-2 olur."""
    env, reset, step = _plain()
    m, idx = env.mj_model, env.idx
    st = reset(_keys(2, 6))
    q0 = jnp.asarray(idx.init_qpos, jnp.float32)
    st = _set_qpos(_set_qpos(st, 0, q0), 1, q0)
    datas = [mujoco.MjData(m) for _ in range(2)]
    for d in datas:
        d.qpos[:] = idx.init_qpos
    rng = np.random.default_rng(1)
    foot_foot = [False, False]
    for _ in range(10):
        a = (0.5 * rng.standard_normal((2, C.NUM_ACTIONS))).astype(np.float32)
        st = step(st, jnp.asarray(a))
        for i, d in enumerate(datas):
            tg, _ = T.action_to_targets(np, a[i].astype(np.float64), idx.default_action_q)
            ctrl = idx.init_ctrl.copy()
            ctrl[idx.action_act_id] = tg
            d.ctrl[:] = np.clip(ctrl, idx.ctrl_lo, idx.ctrl_hi)
            for _ in range(C.N_SUBSTEPS):
                mujoco.mj_step(m, d)
                foot_foot[i] |= any(set(c.geom) == set(idx.foot_geom) for c in d.contact[:d.ncon])
    assert not np.any(np.asarray(st.done))
    for i, d in enumerate(datas):
        assert not foot_foot[i]        # ayak-ayak temasi MJX'in kendi kutu-kutu algoritmasini kullanir
        err = np.abs(np.asarray(st.data.qpos[i]) - d.qpos).max()
        assert err < 3e-3, (i, err)


def test_termination_on_tilt_and_autoreset():
    env, reset, step = _plain()
    st = reset(_keys(2, 2))
    q = np.array(env.idx.init_qpos, dtype=np.float32)
    q[3:7] = [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0]        # x ekseni etrafinda 90 derece
    q[2] += 0.05
    st = _set_qpos(st, 1, jnp.asarray(q))
    a = jnp.zeros((2, C.NUM_ACTIONS))
    st1 = step(st, a)
    done, rew = np.asarray(st1.done), np.asarray(st1.reward)
    assert done[0] == 0.0 and done[1] == 1.0
    assert np.asarray(st1.info["time_out"])[1] == 0.0 and np.asarray(st1.info["truncation"])[1] == 0.0
    assert rew[1] < C.TERMINATION_PENALTY + 0.1 and rew[0] > -1.0
    # biten bolum bilgisi korunur, gozlem/veri sifirlamadan gelir
    assert int(st1.info["steps"][1]) == 1 and float(st1.info["episode_metrics"]["length"][1]) == 1.0
    assert abs(float(st1.info["episode_metrics"]["sum_reward"][1]) - rew[1]) < 1e-5
    assert float(st1.obs["state"][1, I_GRAV_Z]) < -0.95
    assert float(st1.obs["privileged_state"][1, I_TRUNK_Z]) > C.BASE_HEIGHT_TARGET - 0.02
    qp = np.asarray(st1.data.qpos[1])
    assert abs(qp[4]) < 1e-6 and abs(qp[5]) < 1e-6           # yalnizca yaw donmesi
    assert np.allclose(np.asarray(st1.info["a_prev"][1]), 0.0)
    st2 = step(st1, a)
    assert int(st2.info["steps"][1]) == 1 and float(st2.done[1]) == 0.0
    assert float(st2.info["episode_metrics"]["length"][1]) == 1.0

    # VmapAutoReset (lax.cond) == vmap(env.step) (her ortamda kosulsuz sifirlama)
    vstep = jax.jit(jax.vmap(env.step))
    ref = vstep(st, a)
    for x, y in zip(jax.tree_util.tree_leaves(ref), jax.tree_util.tree_leaves(st1)):
        assert np.array_equal(np.asarray(x), np.asarray(y))


def test_timeout():
    env, reset, step = _plain()
    st = reset(_keys(2, 3))
    st = st.replace(info=dict(st.info, steps=st.info["steps"].at[0].set(C.MAX_EPISODE_STEPS - 1)))
    st = step(st, jnp.zeros((2, C.NUM_ACTIONS)))
    assert np.asarray(st.done).tolist() == [1.0, 0.0]
    assert np.asarray(st.info["time_out"]).tolist() == [1.0, 0.0]
    assert np.asarray(st.info["truncation"]).tolist() == [1.0, 0.0]
    assert float(st.reward[0]) > -1.0
    assert int(st.info["steps"][0]) == C.MAX_EPISODE_STEPS


def test_command_resample_and_lock():
    """Komut COMMAND_RESAMPLE_S'de (adim 300) yeniden orneklenir; set_command kilidi sifirlamada korunur.
    (Kurucuda fixed_command verilen ortamda yeniden ornekleme hic yoktur; burada itme ve bas hareketi
    kapali ama komut ornekleme ve DR acik ortam kullanilir.)"""
    env, reset, step = _calm()
    st = reset(_keys(2, 8))
    cmd0 = np.asarray(st.info["command"][0])
    st = env.set_command(st, jnp.asarray([cmd0, [0.1, 0.05, -0.2]], jnp.float32))
    st = st.replace(info=dict(st.info, cmd_lock=st.info["cmd_lock"].at[0].set(0.0)))
    a = jnp.zeros((2, C.NUM_ACTIONS))
    n = int(round(C.COMMAND_RESAMPLE_S / C.CTRL_DT))
    cmds = []
    for _ in range(n):
        st = step(st, a)
        assert not np.any(np.asarray(st.done))
        cmds.append(np.asarray(st.info["command"]))
    cmds = np.array(cmds)
    assert np.all(cmds[:-1, 0] == cmd0) and np.any(cmds[-1, 0] != cmd0), (cmd0, cmds[-1, 0])
    for j, k in enumerate(("vx", "vy", "wz")):
        lo, hi = C.COMMAND_RANGES[k]
        assert lo - 1e-6 <= cmds[-1, 0, j] <= hi + 1e-6
    assert np.allclose(cmds[:, 1], [0.1, 0.05, -0.2])
    # adim 13: gozlemdeki komut yeni komut
    assert np.allclose(np.asarray(st.obs["state"][:, 44:47]), cmds[-1])
    # kilitli ortam dusup sifirlaninca komutu korur
    q = np.array(env.idx.init_qpos, dtype=np.float32)
    q[3:7] = [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0]
    q[2] += 0.05
    st = step(_set_qpos(st, 1, jnp.asarray(q)), a)
    assert np.asarray(st.done).tolist() == [0.0, 1.0]
    assert np.allclose(np.asarray(st.info["command"][1]), [0.1, 0.05, -0.2])
    assert np.allclose(np.asarray(st.obs["state"][1, 44:47]), [0.1, 0.05, -0.2])


def test_dr_sampling_ranges():
    env = OpenDuckEnv()
    dr = C.DOMAIN_RAND
    n = 512
    p = jax.jit(jax.vmap(env.sample_dr))(_keys(n, 4))
    p = jax.tree_util.tree_map(np.asarray, p)
    nom = {k: np.asarray(v) for k, v in env.nominal.items()}
    tb = env.idx.trunk_body

    def in_range(x, lo, hi, cover=0.05):
        x = np.asarray(x)
        assert x.min() >= lo - 1e-6 and x.max() <= hi + 1e-6, (x.min(), x.max(), lo, hi)
        span = hi - lo
        assert x.min() < lo + cover * span and x.max() > hi - cover * span, (x.min(), x.max(), lo, hi)

    in_range(p["friction"], *dr["friction"])
    others = [b for b in range(1, env.mj_model.nbody) if b != tb]
    ratio = p["body_mass"][:, others] / nom["body_mass"][others]
    in_range(ratio, *dr["link_mass_scale"])
    assert np.allclose(p["body_inertia"][:, others] / nom["body_inertia"][others], ratio[..., None], rtol=1e-5)
    assert np.all(p["body_mass"][:, 0] == nom["body_mass"][0])
    tm = p["body_mass"][:, tb]
    lo = nom["body_mass"][tb] * dr["link_mass_scale"][0] + dr["trunk_added_mass"][0]
    hi = nom["body_mass"][tb] * dr["link_mass_scale"][1] + dr["trunk_added_mass"][1]
    assert tm.min() >= lo - 1e-6 and tm.max() <= hi + 1e-6
    assert np.allclose(p["body_inertia"][:, tb] / nom["body_inertia"][tb], (tm / nom["body_mass"][tb])[:, None], rtol=1e-5)
    off = p["body_ipos"][:, tb] - nom["body_ipos"][tb]
    in_range(off, -dr["trunk_com_offset"], dr["trunk_com_offset"])
    assert np.allclose(p["body_ipos"][:, others], nom["body_ipos"][others])
    sts = env.sts_act
    in_range(p["kp"][:, sts] / nom["actuator_gainprm"][sts, 0], *dr["kp_scale"])
    rest = np.setdiff1d(np.arange(env.mj_model.nu), sts)
    assert np.allclose(p["kp"][:, rest], nom["actuator_gainprm"][rest, 0])
    ad = env.act_dof
    free = np.setdiff1d(np.arange(env.mj_model.nv), ad)
    for field, rng_key in (("dof_damping", "damping_scale"), ("dof_frictionloss", "frictionloss_scale"),
                           ("dof_armature", "armature_scale")):
        in_range(p[field][:, ad] / nom[field][ad], *dr[rng_key])
        assert np.allclose(p[field][:, free], nom[field][free])
    lo, hi = dr["action_delay_steps"]
    assert p["delay"].min() >= lo and p["delay"].max() <= hi

    # modele yazilan alanlar
    one = jax.tree_util.tree_map(lambda x: x[0], p)
    m = env.model_from_dr(one)
    gf = np.asarray(m.geom_friction)
    fg = list(env.idx.foot_geom) + [env.idx.floor_geom]
    assert np.allclose(gf[fg, 0], one["friction"])
    assert np.allclose(np.delete(gf, fg, axis=0), np.delete(nom["geom_friction"], fg, axis=0))
    assert np.allclose(np.asarray(m.actuator_gainprm)[:, 0], one["kp"])
    assert np.allclose(np.asarray(m.actuator_biasprm)[:, 1], -one["kp"])

    # Playground/Brax bicimi vmap'li randomization_fn
    mv, axes = env.randomization_fn(env.mjx_model, _keys(8, 5))
    assert mv.body_mass.shape == (8, env.mj_model.nbody) and axes.body_mass == 0 and axes.opt is not None
    assert mv.geom_friction.shape == (8, env.mj_model.ngeom, 3) and axes.geom_friction == 0
    assert mv.dof_damping.shape == (8, env.mj_model.nv) and axes.body_pos is None


def test_setconst_matches_c():
    """mj_setConst ciktilari (MJX'in kullandiklari) JAX ile C ile ayni."""
    env = OpenDuckEnv()
    p = jax.tree_util.tree_map(np.asarray, jax.jit(env.sample_dr)(jax.random.PRNGKey(7)))
    m = env.mj_model.__copy__()
    fg = list(env.idx.foot_geom) + [env.idx.floor_geom]
    m.geom_friction[fg, 0] = p["friction"]
    m.body_mass[:] = p["body_mass"]
    m.body_inertia[:] = p["body_inertia"]
    m.body_ipos[:] = p["body_ipos"]
    m.actuator_gainprm[:, 0] = p["kp"]
    m.actuator_biasprm[:, 1] = -p["kp"]
    for k in ("dof_damping", "dof_frictionloss", "dof_armature"):
        getattr(m, k)[:] = p[k]
    mujoco.mj_setConst(m, mujoco.MjData(m))
    for k in ("body_invweight0", "dof_invweight0", "dof_M0", "body_subtreemass"):
        assert np.allclose(p[k], getattr(m, k), rtol=2e-4, atol=1e-6), (k, p[k], getattr(m, k))
    assert abs(float(p["meaninertia"]) - m.stat.meaninertia) < 2e-4 * m.stat.meaninertia
    # nominal model: setConst degisikligi yok
    nomp = env.set_const(env.mjx_model)
    assert np.allclose(np.asarray(nomp["body_invweight0"]), env.mj_model.body_invweight0, rtol=2e-4, atol=1e-6)


def test_extract_state_matches_c():
    """extract_state(mjx.forward) == MuJoCo C mj_forward'dan ayni alanlar; sifirlama icin kisaltilmis
    ileri hesap (_forward_obs) gozlem alanlarinda mjx.forward ile ayni."""
    env = OpenDuckEnv(randomize=False)
    m, idx = env.mj_model, env.idx
    rng = np.random.default_rng(0)
    fwd = jax.jit(mjx.forward)
    fwd_obs = jax.jit(env._forward_obs)
    for trial in range(3):
        d = mujoco.MjData(m)
        d.qpos[:] = idx.init_qpos
        d.qpos[2] -= 0.002
        if trial > 0:          # egik ayaklar, rastgele hizlar
            d.qpos[7:] += rng.uniform(-0.05, 0.05, m.nq - 7)
            d.qvel[:] = rng.uniform(-0.3, 0.3, m.nv)
        d.ctrl[:] = idx.init_ctrl + rng.uniform(-0.1, 0.1, m.nu)
        mujoco.mj_forward(m, d)
        dx0 = env._data0.replace(qpos=jnp.asarray(d.qpos, jnp.float32), qvel=jnp.asarray(d.qvel, jnp.float32),
                                 ctrl=jnp.asarray(d.ctrl, jnp.float32))
        sj = {k: np.asarray(v) for k, v in extract_state(fwd(env.mjx_model, dx0), idx).items()}
        so = {k: np.asarray(v) for k, v in extract_state(fwd_obs(env.mjx_model, dx0), idx).items()}
        con = np.zeros(2)
        for c in d.contact[:d.ncon]:
            for f in range(2):
                if c.dist < C.CONTACT_EPS and sorted(c.geom) == sorted([idx.floor_geom, idx.foot_geom[f]]):
                    con[f] = 1.0
        sc = {
            "quat": d.sensordata[idx.s_quat], "gyro": d.sensordata[idx.s_gyro], "linvel": d.sensordata[idx.s_linvel],
            "trunk_z": d.xpos[idx.trunk_body, 2], "q": d.qpos[idx.obs_qpos_adr], "qd": d.qvel[idx.obs_dof_adr],
            "qdd_legs": d.qacc[idx.leg_dof_adr], "torque": d.actuator_force[idx.obs_act_id],
            "foot_pos": np.stack([d.sensordata[s] for s in idx.s_foot_pos]),
            "foot_vel": np.stack([d.sensordata[s] for s in idx.s_foot_vel]),
            "contact": con,
        }
        assert set(sj) == set(sc)
        for k in sc:
            tol = 1e-3 + 1e-4 * np.abs(sc[k]).max() if k == "qdd_legs" else 2e-4
            assert np.abs(sj[k] - sc[k]).max() <= tol, (trial, k, sj[k], sc[k])
        # C ile ayni temas noktalari (c_semantics duzlem-kutu)
        cx = fwd(env.mjx_model, dx0)._impl.contact
        g, dist = np.asarray(cx.geom), np.asarray(cx.dist)
        for f in idx.foot_geom:
            pair = sorted([idx.floor_geom, f])
            n_mjx = sum(1 for gi, di in zip(g, dist) if di < 0 and sorted(gi) == pair)
            n_c = sum(1 for c in d.contact[:d.ncon] if sorted(c.geom) == pair)
            assert n_mjx == n_c, (trial, f, n_mjx, n_c)
        for k in ("quat", "gyro", "linvel", "trunk_z", "q", "qd", "foot_pos", "foot_vel", "contact"):
            assert np.abs(so[k] - sj[k]).max() <= 1e-5, (trial, k)
        assert sc["contact"].sum() >= 1


def test_onnx_export():
    from brax.training.acme import running_statistics, specs
    from brax.training.agents.ppo import networks as ppo_networks
    from rl import train_mjx
    import onnxruntime as ort
    import json
    args = train_mjx.parse_args(["--out", "x", "--num-envs", "8", "--policy-hidden", "32,16", "--value-hidden", "16"])
    obs_size = {"state": (C.NUM_OBS,), "privileged_state": (C.NUM_CRITIC_OBS,)}
    net = train_mjx.network_factory(args)(obs_size, C.NUM_ACTIONS,
                                          preprocess_observations_fn=running_statistics.normalize)
    k1, k2 = jax.random.split(jax.random.PRNGKey(0))
    pol, val = net.policy_network.init(k1), net.value_network.init(k2)
    norm = running_statistics.init_state({k: specs.Array(v, jnp.dtype("float32")) for k, v in obs_size.items()})
    rng = np.random.default_rng(1)
    batch = {k: jnp.asarray(rng.normal(0.5, 2.0, (64,) + v), jnp.float32) for k, v in obs_size.items()}
    norm = running_statistics.update(norm, batch)
    params = (norm, pol, val)
    make_policy = ppo_networks.make_inference_fn(net)
    with tempfile.TemporaryDirectory() as tmp:
        err = train_mjx.export_policy(params, tmp, 123, make_policy)
        assert err < 1e-4
        meta = json.load(open(os.path.join(tmp, "policy_meta.json")))
        assert meta["obs_dim"] == C.NUM_OBS and meta["action_dim"] == C.NUM_ACTIONS and meta["steps"] == 123
        assert set(meta) == {"obs_dim", "action_dim", "ctrl_dt", "action_scale", "action_clip", "default_leg_q",
                             "leg_joints", "obs_joints", "default_obs_q", "gait_freq", "obs_scales", "backend", "steps"}
        sess = ort.InferenceSession(os.path.join(tmp, "policy.onnx"), providers=["CPUExecutionProvider"])
        x = np.asarray(batch["state"][:5])
        out = sess.run(["action"], {"obs": x})[0]
        ref = np.asarray(make_policy(params, deterministic=True)(batch, jax.random.PRNGKey(0))[0][:5])
        assert out.shape == (5, C.NUM_ACTIONS) and np.abs(out - ref).max() < 1e-4
        assert np.abs(train_mjx.actor_numpy(params, x) - ref).max() < 1e-4
        train_mjx.save_params(params, tmp, 5, args)
        ps = train_mjx.load_params(os.path.join(tmp, "params_latest.pkl"))
        assert ps["steps"] == 5 and np.allclose(np.asarray(ps["params"][0].mean["state"]), np.asarray(norm.mean["state"]))

        # dogrulama gecmezse onceki policy.onnx/meta oldugu gibi kalir, gecici dosya silinir
        def bad_make_policy(p, deterministic=False):
            pol = make_policy(p, deterministic=deterministic)
            return lambda obs, key: (pol(obs, key)[0] + 0.01, {})
        before = open(os.path.join(tmp, "policy.onnx"), "rb").read()
        try:
            train_mjx.export_policy(params, tmp, 456, bad_make_policy)
            raise AssertionError("bozuk politika dogrulamadan gecti")
        except RuntimeError:
            pass
        assert open(os.path.join(tmp, "policy.onnx"), "rb").read() == before
        assert json.load(open(os.path.join(tmp, "policy_meta.json")))["steps"] == 123
        assert not os.path.exists(os.path.join(tmp, "policy.onnx.tmp"))


def _dot_precisions(jaxpr, out):
    from jax.extend import core as jcore
    for eqn in jaxpr.eqns:
        if eqn.primitive.name == "dot_general":
            out.append(eqn.params.get("precision"))
        for v in eqn.params.values():
            for sub in (v if isinstance(v, (list, tuple)) else [v]):
                if isinstance(sub, jcore.ClosedJaxpr):
                    _dot_precisions(sub.jaxpr, out)
                elif isinstance(sub, jcore.Jaxpr):
                    _dot_precisions(sub, out)
    return out


def test_env_matmul_precision():
    """Ortamin reset/step izlemesinde hassasiyeti belirsiz (GPU'da TF32 olacak) dot_general kalmaz."""
    env = _full()[0]
    b = VmapAutoReset(env)
    keys = jax.ShapeDtypeStruct((2, 2), jnp.uint32)
    st = jax.eval_shape(b.reset, keys)
    act = jax.ShapeDtypeStruct((2, C.NUM_ACTIONS), jnp.float32)
    for j in (jax.make_jaxpr(b.reset)(keys), jax.make_jaxpr(b.step)(st, act)):
        ps = _dot_precisions(j.jaxpr, [])
        assert len(ps) > 50 and all(p is not None for p in ps), (len(ps), sum(p is None for p in ps))


# =====================================================================================================
# main / getup gorevleri (rl/TASK.md "Tasks main and getup")
# =====================================================================================================
_PLAIN = dict(randomize=False, noise=False, head_random=False, push=False)
NT = 4          # gorev testlerinde toplu ortam sayisi (ayni jit derlemesi paylasilir)


def _task(name):
    """main_plain / main_full / getup_plain (mufredat 1000 adim) / getup_full: (env, reset, step) jit'li."""
    if name not in _CACHE:
        task, kind = name.split("_")
        kw = dict(_PLAIN) if kind == "plain" else {}
        if name == "getup_plain":
            kw["curriculum_steps"] = 1000
        env = OpenDuckEnv(task=task, **kw)
        b = VmapAutoReset(env)
        _CACHE[name] = (env, jax.jit(b.reset), jax.jit(b.step))
    return _CACHE[name]


def _avals(tree):
    return [(jax.tree_util.keystr(p), x.shape, x.dtype, bool(getattr(x, "weak_type", False)))
            for p, x in jax.tree_util.tree_flatten_with_path(tree)[0]]


def _bank_qpos(kind, j, yaw):
    """Banka durumu j, sifirlamadaki gibi: kuaterniyon yaw ile dondurulmus, xy = 0."""
    b = PB.bank(kind)
    q = np.array(b["qpos"][j], dtype=np.float64)
    q[3:7] = T.quat_mul(np, T.yaw_quat(np, np.float64(yaw)), q[3:7])
    q[:2] = 0.0
    return q, np.array(b["qvel"][j], dtype=np.float64)


def _put(st, i, qpos=None, qvel=None, **info):
    """Toplu durumda ortam i'nin qpos/qvel'ini ve info alanlarini degistirir."""
    d = st.data
    if qpos is not None:
        d = d.replace(qpos=d.qpos.at[i].set(jnp.asarray(qpos, jnp.float32)),
                      qvel=d.qvel.at[i].set(0.0 if qvel is None else jnp.asarray(qvel, jnp.float32)))
    inf = dict(st.info)
    for k, v in info.items():
        inf[k] = inf[k].at[i].set(jnp.asarray(v, inf[k].dtype))
    return st.replace(data=d, info=inf)


def _np_state(st, i, env, **extra):
    """Adim sonrasi veriden task_math durumu (ortam i, numpy) - ortamin kullandigi toplama ile ayni."""
    di = jax.tree_util.tree_map(lambda x: x[i], st.data)
    s = {k: np.nan_to_num(np.asarray(v, np.float64)) for k, v in extract_state(di, env.idx, env.task).items()}
    s.update(friction=np.float64(1.0), default_q=env.idx.default_obs_q)
    s.update({k: np.asarray(v, np.float64) for k, v in extra.items()})
    return s


def test_task_shapes_and_pytree():
    """main/getup: gozlem/aksiyon boyutlari, metrik anahtarlari, sonlu degerler; reset/step/merge pytree'leri ayni
    (anahtar kumesi, sekil, dtype, weak_type)."""
    for task in ("main", "getup"):
        spec = TK.get_task(task)
        env, reset, step = _task(task + "_full")
        assert env.action_size == spec.num_actions and env.max_episode_steps == spec.max_episode_steps
        assert env.observation_size == {"state": (spec.num_obs,), "privileged_state": (spec.num_critic_obs,)}
        keys = jax.ShapeDtypeStruct((NT, 2), jnp.uint32)
        r = jax.eval_shape(jax.vmap(env.reset), keys)
        act = jax.ShapeDtypeStruct((NT, spec.num_actions), jnp.float32)
        for f in (VmapAutoReset(env).step, jax.vmap(env.step_no_reset)):
            assert _avals(jax.eval_shape(f, r, act)) == _avals(r), task
        st = reset(_keys(NT, 11))
        assert st.obs["state"].shape == (NT, spec.num_obs) and st.obs["privileged_state"].shape == (NT, spec.num_critic_obs)
        rng = np.random.default_rng(0)
        for _ in range(3):
            st = step(st, jnp.asarray(rng.standard_normal((NT, spec.num_actions)), jnp.float32))
            for x in (st.obs["state"], st.obs["privileged_state"], st.reward):
                assert np.all(np.isfinite(np.asarray(x)))
        mk = {"rew_" + k for k in spec.reward_weights} | set(spec.episode_metrics)
        assert set(st.metrics) == mk, sorted(set(st.metrics) ^ mk)
        assert set(st.info["episode_metrics"]) == mk | {"sum_reward", "length"}
        assert np.all(np.asarray(st.info["steps"]) == 3)
        if task == "main":
            assert st.info["command"].shape == (NT, 5) and "gstep" not in st.info
        else:
            assert "command" not in st.info and "phase" not in st.info and "head_offset" not in st.info
            assert st.info["gstep"].dtype == jnp.int32 and np.all(np.asarray(st.info["gstep"]) == 3)
        wrap_for_brax_training(env, episode_length=spec.max_episode_steps)
        try:
            wrap_for_brax_training(env, episode_length=spec.max_episode_steps + 1)
            raise AssertionError("yanlis bolum uzunlugu kabul edildi")
        except ValueError:
            pass


def test_main_command_padding_and_obs():
    """fixed_command/set_command 3 -> [vx, vy, wz, H_HOME, 0], 5 aynen; getup komut kabul etmez. Gozlemdeki komut
    etkin komut: sit=1 ya da h < H_MIN_WALK iken hizlar 0, h [H_MIN_STAND, H_HOME]'a kirpilir."""
    e = OpenDuckEnv(task="main", fixed_command=(0.1, 0.0, 0.2), **_PLAIN)
    assert np.allclose(e.fixed_command, [0.1, 0.0, 0.2, C.H_HOME, 0.0])
    e = OpenDuckEnv(task="main", fixed_command=(0.1, 0.0, 0.2, 0.09, 1.0), **_PLAIN)
    assert np.allclose(e.fixed_command, [0.1, 0.0, 0.2, 0.09, 1.0])
    for bad in (dict(task="getup", fixed_command=(0, 0, 0)), dict(task="main", fixed_command=(0, 0, 0, 1))):
        try:
            OpenDuckEnv(**dict(_PLAIN, **bad))
            raise AssertionError(bad)
        except ValueError:
            pass
    env, reset, step = _task("main_plain")
    st = reset(_keys(NT, 12))
    st3 = env.set_command(st, jnp.asarray([0.1, -0.05, 0.3], jnp.float32))
    assert np.allclose(np.asarray(st3.info["command"]), [0.1, -0.05, 0.3, C.H_HOME, 0.0])
    assert np.all(np.asarray(st3.info["cmd_lock"]) == 1.0)
    cmds = np.array([[0.2, 0.1, 0.5, C.H_HOME, 1.0], [0.2, 0.1, 0.5, 0.075, 0.0],
                     [0.2, 0.1, 0.5, C.H_HOME, 0.0], [0.2, 0.1, 0.5, 0.05, 0.0]], np.float32)
    st = env.set_command(st, jnp.asarray(cmds))
    assert np.array_equal(np.asarray(st.info["command"]), cmds)
    genv = _task("getup_plain")[0]
    try:
        genv.set_command(_task("getup_plain")[1](_keys(NT, 1)), jnp.zeros(3))
        raise AssertionError("getup set_command kabul etti")
    except ValueError:
        pass
    q0 = env.idx.init_qpos
    for i in range(NT):     # hepsi ayakta, alpha 0, h_ref H_HOME; segment suresi dolmus (kilitli komut degismez)
        st = _put(st, i, q0, alpha=0.0, h_ref=C.H_HOME, seg_timer=0.001)
    st = step(st, jnp.zeros((NT, 10)))
    assert not np.any(np.asarray(st.done))
    assert np.array_equal(np.asarray(st.info["command"]), cmds)
    sl = TK.get_task("main").obs_slices
    cobs = np.asarray(st.obs["state"][:, sl["command"]])
    hn = lambda h: (h - C.H_HOME) / C.H_NORM
    want = [[0, 0, 0, 0, 1], [0, 0, 0, hn(0.075), 0], [0.2, 0.1, 0.5, 0, 0], [0, 0, 0, -1.0, 0]]
    assert np.allclose(cobs, want, atol=1e-6), cobs
    # durus filtresi bir adim: alpha += dt/SIT_RAMP_S (sit), h_ref -= H_RATE*dt (alcalma)
    pobs = np.asarray(st.obs["state"][:, sl["posture"]])
    dh = C.H_RATE * C.CTRL_DT
    assert np.allclose(pobs[:, 1], [C.CTRL_DT / C.SIT_RAMP_S, 0, 0, 0], atol=1e-6)
    assert np.allclose(pobs[:, 0], [0, hn(C.H_HOME - dh), 0, hn(C.H_HOME - dh)], atol=1e-5)
    assert np.allclose(np.asarray(st.info["alpha"]), pobs[:, 1]) and np.allclose(np.asarray(st.info["h_ref"]),
                                                                                  [C.H_HOME, C.H_HOME - dh] * 2)


def test_main_segment_resample():
    """Segment suresi dolunca (kilitsiz) yeni segment ornekelenir (sure U(segment_s)); dolmamissa zamanlayici
    azalir; kilitli komut hic degismez. Gozlemdeki komut yeni komuttur (adim 13)."""
    env, reset, step = _task("main_plain")
    st = reset(_keys(NT, 13))
    q0 = env.idx.init_qpos
    c_prev = np.array([0.1, 0.0, 0.2, C.H_HOME, 0.0], np.float32)
    for i in range(NT):
        st = _put(st, i, q0, alpha=0.0, h_ref=C.H_HOME, command=c_prev, cmd_lock=0.0,
                  seg_timer=[0.001, 2.0, 0.001, 0.001][i])
    st = st.replace(info=dict(st.info, cmd_lock=st.info["cmd_lock"].at[2].set(1.0)))
    seg_t0 = np.asarray(st.info["seg_timer"])
    st = step(st, jnp.zeros((NT, 10)))
    cmd, tm = np.asarray(st.info["command"]), np.asarray(st.info["seg_timer"])
    lo, hi = C.MAIN["segment_s"]
    for i in (0, 3):
        assert lo - 1e-6 <= tm[i] <= hi + 1e-6 and not np.array_equal(cmd[i], c_prev), (i, tm[i], cmd[i])
        assert float(st.info["seg"][i]) in (0.0, 1.0, 2.0, 3.0)
        k = int(st.info["seg"][i])
        if k != 0:
            assert np.all(cmd[i, :3] == 0.0)
        assert cmd[i, 4] == (1.0 if k == 3 else 0.0)
    assert abs(tm[1] - (seg_t0[1] - C.CTRL_DT)) < 1e-6 and np.array_equal(cmd[1], c_prev)
    assert np.array_equal(cmd[2], c_prev)
    sl = TK.get_task("main").obs_slices["command"]
    assert np.allclose(np.asarray(st.obs["state"][:, sl]), T.command_obs(np, cmd), atol=1e-6)


def test_main_mode_aware_termination():
    """main: govdenin uzerine inis (oturus pozu, 3 mm asagida, v_z = -0.4 m/s): alpha = 1 iken (planli oturus)
    sonlanmaz ve oturus 1 s korunur, alpha = 0 iken (planlanmamis oturma: govde yerde, alpha < 0.3) sonlanir;
    devrik govde sonlanir. walk ayni oturus pozunda (govde alcak) sonlanir."""
    env, reset, step = _task("main_plain")
    st = reset(_keys(NT, 14))
    seat_q, _ = _bank_qpos("seated", 0, 0.7)
    land_q, land_v = seat_q.copy(), np.zeros(env.mj_model.nv)
    land_q[2] -= 0.003
    land_v[2] = -0.4
    tip = np.array(env.idx.init_qpos)
    tip[3:7] = [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0]
    tip[2] += 0.05
    sit_cmd = [0.0, 0.0, 0.0, C.H_HOME, 1.0]
    stand_cmd = [0.0, 0.0, 0.0, C.H_HOME, 0.0]
    st = env.set_command(st, jnp.asarray([sit_cmd, stand_cmd, stand_cmd, stand_cmd], jnp.float32))
    st = _put(st, 0, land_q, land_v, alpha=1.0, h_ref=C.H_HOME)
    st = _put(st, 1, env.idx.init_qpos, alpha=0.0, h_ref=C.H_HOME)
    st = _put(st, 2, tip, alpha=0.0, h_ref=C.H_HOME)
    st = _put(st, 3, land_q, land_v, alpha=0.0, h_ref=C.H_HOME)
    a = jnp.zeros((NT, 10))
    st = step(st, a)
    assert np.asarray(st.done).tolist() == [0.0, 0.0, 1.0, 1.0], np.asarray(st.done)
    assert np.all(np.asarray(st.info["time_out"]) == 0.0)
    rew = np.asarray(st.reward)
    assert rew[2] < C.TERMINATION_PENALTY + 0.2 and rew[3] < C.TERMINATION_PENALTY + 0.2 and rew[0] > -1.0
    em = st.info["episode_metrics"]
    assert float(em["falls_walk"][2]) == 1.0 and float(em["falls_transition"][3]) == 0.0
    cs = TK.get_task("main").critic_slices
    for _ in range(int(round(1.0 / C.CTRL_DT))):
        st = step(st, a)
        assert not np.any(np.asarray(st.done)[:2])
    crit = np.asarray(st.obs["privileged_state"])
    assert crit[0, cs["trunk_floor"]][0] == 1.0 and crit[1, cs["trunk_floor"]][0] == 0.0
    assert crit[0, cs["head_floor"]][0] == 0.0 and abs(crit[0, cs["trunk_z"]][0] - C.SEAT_Z) < 0.006
    assert float(st.metrics["rew_seat"][0]) > 0.0 and float(st.metrics["seated_ok_steps"][0]) == 1.0
    assert float(st.info["alpha"][0]) == 1.0
    # walk ayni pozda: trunk_z < TERMINATION_MIN_HEIGHT -> dustu
    wenv, wreset, wstep = _plain()
    ws = _set_qpos(wreset(_keys(2, 14)), 1, jnp.asarray(seat_q, jnp.float32))
    assert np.asarray(wstep(ws, jnp.zeros((2, 10))).done).tolist() == [0.0, 1.0]


def test_main_bank_reset():
    """main sifirlamasi: %25 oturus bankasi (alpha = 1, govde yerde, banka eklem acilari, kuaterniyon = yaw *
    banka, xy = 0, qvel = 0, ilk segment sit/stand); %75 HOME (alpha = 0). Fizik yok. reset_probs degisikligi."""
    env = _task("main_plain")[0]
    n = 128
    st = jax.jit(jax.vmap(env.reset))(_keys(n, 15))
    b = PB.seated_bank()
    kind = np.asarray(st.info["reset_kind"])
    assert set(np.unique(kind)) <= {0.0, 1.0} and 0.1 < kind.mean() < 0.45, kind.mean()
    qpos, qvel = np.asarray(st.data.qpos, np.float64), np.asarray(st.data.qvel)
    assert np.all(np.asarray(st.data.time) == 0.0)
    cs, sl = TK.get_task("main").critic_slices, TK.get_task("main").obs_slices
    crit, obs = np.asarray(st.obs["privileged_state"]), np.asarray(st.obs["state"])
    seg, cmd = np.asarray(st.info["seg"]), np.asarray(st.info["command"])
    for i in range(n):
        assert np.all(qpos[i, :2] == 0.0)
        if kind[i] == 1.0:
            j = int(np.argmin(np.abs(b["qpos"][:, 7:] - qpos[i, 7:]).max(axis=1)))
            assert np.abs(b["qpos"][j, 7:] - qpos[i, 7:]).max() < 1e-6 and abs(b["qpos"][j, 2] - qpos[i, 2]) < 1e-6
            rel = T.quat_mul(np, qpos[i, 3:7], b["qpos"][j, 3:7] * [1, -1, -1, -1])     # q * conj(q_bank) = yaw
            assert abs(rel[1]) < 1e-5 and abs(rel[2]) < 1e-5, rel
            g = T.projected_gravity(np, qpos[i, 3:7])
            assert np.allclose(g, T.projected_gravity(np, b["qpos"][j, 3:7]), atol=1e-5)
            assert np.allclose(obs[i, sl["gravity"]], g, atol=1e-5)
            assert np.all(qvel[i] == 0.0) and float(st.info["alpha"][i]) == 1.0
            assert crit[i, cs["trunk_floor"]][0] == 1.0 and np.allclose(obs[i, sl["posture"]], [0.0, 1.0])
            assert seg[i] in (1.0, 3.0) and cmd[i, 4] == (1.0 if seg[i] == 3.0 else 0.0)
            assert float(st.info["ev_prev_sit"][i]) == 1.0
        else:
            assert abs(qpos[i, 2] - env.idx.init_qpos[2]) < 1e-6 and float(st.info["alpha"][i]) == 0.0
            assert crit[i, cs["trunk_floor"]][0] == 0.0 and obs[i, sl["gravity"]][2] < -0.99
            assert np.abs(qpos[i, 7:] - env.idx.init_qpos[7:]).max() <= C.RESET_JOINT_NOISE + 1e-6
    yaws = [T.yaw_angle(np, qpos[i, 3:7]) for i in range(n) if kind[i] == 1.0]
    assert np.std(yaws) > 0.5                              # yaw rastgele
    e = OpenDuckEnv(task="main", reset_probs={"seated": 1.0}, **_PLAIN)
    assert e.reset_probs == {"home": 0.0, "seated": 1.0}
    assert np.all(np.asarray(jax.jit(jax.vmap(e.reset))(_keys(8, 2)).info["reset_kind"]) == 1.0)
    for bad in ({"fallen": 1.0}, {"home": 0.0}):
        try:
            OpenDuckEnv(task="main", reset_probs=bad, **_PLAIN)
            raise AssertionError(bad)
        except ValueError:
            pass


def test_getup_bank_reset():
    """getup sifirlamasi: dusmus %70 / dusus ortasi %10 / ayakta %20; banka durumu yaw'la dondurulmus (yercekimi
    govde cercevesinde ayni), qvel bankadan (dusus ortasi acisal hiz), etiket; bellek ve olay durumu."""
    env = _task("getup_plain")[0]
    n = 256
    st = jax.jit(jax.vmap(env.reset))(_keys(n, 16))
    kind, lab = np.asarray(st.info["reset_kind"]), np.asarray(st.info["bank_label"])
    fr = [np.mean(kind == k) for k in (0.0, 1.0, 2.0)]
    assert 0.55 < fr[0] < 0.85 and 0.03 < fr[1] < 0.2 and 0.1 < fr[2] < 0.3, fr
    qpos, qvel = np.asarray(st.data.qpos, np.float64), np.asarray(st.data.qvel, np.float64)
    assert np.all(np.asarray(st.data.time) == 0.0) and np.all(qpos[:, :2] == 0.0)
    banks = {0.0: PB.fallen_bank(), 1.0: PB.midfall_bank()}
    sl, cs = TK.get_task("getup").obs_slices, TK.get_task("getup").critic_slices
    obs, crit = np.asarray(st.obs["state"]), np.asarray(st.obs["privileged_state"])
    for i in range(n):
        if kind[i] in banks:
            b = banks[kind[i]]
            j = int(np.argmin(np.abs(b["qpos"][:, 7:] - qpos[i, 7:]).max(axis=1)))
            assert np.abs(b["qpos"][j, 7:] - qpos[i, 7:]).max() < 1e-6 and abs(b["qpos"][j, 2] - qpos[i, 2]) < 1e-6
            assert np.allclose(T.projected_gravity(np, qpos[i, 3:7]), T.projected_gravity(np, b["qpos"][j, 3:7]),
                               atol=1e-5)
            assert np.allclose(qvel[i], b["qvel"][j], atol=1e-6) and lab[i] == b["label"][j]
        else:
            assert abs(qpos[i, 2] - env.idx.init_qpos[2]) < 1e-6 and lab[i] == -1.0
            assert np.all(np.abs(qvel[i, 3:]) == 0.0)
    assert np.any(np.abs(qvel[kind == 1.0, 3:6]).max(axis=1) > 0.5)        # dusus ortasi acisal hizli
    g = obs[:, sl["gravity"]]
    assert np.allclose(np.asarray(st.info["phi_up_prev"]), -g[:, 2], atol=1e-6)
    assert np.allclose(np.asarray(st.info["phi_h_prev"]), np.minimum(crit[:, cs["trunk_z"]][:, 0], C.H_HOME), atol=1e-6)
    assert np.array_equal(np.asarray(st.info["head_floor_prev"]), crit[:, cs["head_floor"]][:, 0])
    assert np.all(np.asarray(st.info["gstep"]) == 0) and np.all(np.asarray(st.info["ev_succeeded"]) == 0.0)
    assert np.mean(crit[kind == 0.0, cs["trunk_floor"]][:, 0] + crit[kind == 0.0, cs["head_floor"]][:, 0] > 0) > 0.9


def test_getup_action_mapping():
    """getup: 14 aksiyon -> clip(HOME + S_j clip(a, +-1), ctrlrange) (bacak + bas), gaga HOME'da; last_action =
    kirpilmis aksiyon."""
    env, reset, step = _task("getup_plain")
    idx = env.idx
    st = reset(_keys(NT, 17))
    S = TK.action_scale_vector("getup", idx.default_action_q, idx.action_lo, idx.action_hi)
    q0, lo, hi = idx.default_action_q, idx.action_lo, idx.action_hi
    assert S.shape == (14,) and np.all(S >= 0.5 * (hi - lo) - 1e-9)
    assert np.all((q0 + S >= hi - 1e-9) & (q0 - S <= lo + 1e-9))       # +-1 her iki siniri da kapsar
    rng = np.random.default_rng(3)
    act = np.stack([np.full(14, 5.0), np.full(14, -5.0), np.full(14, 0.5), rng.standard_normal(14)]).astype(np.float32)
    st = step(st, jnp.asarray(act))
    assert not np.any(np.asarray(st.done))
    ctrl = np.asarray(st.data.ctrl, np.float64)
    ac = np.clip(act, -1.0, 1.0)
    want = np.clip(idx.default_action_q + S * ac, idx.action_lo, idx.action_hi)
    assert np.allclose(ctrl[:, idx.action_act_id], want, atol=1e-6)
    assert np.allclose(ctrl[0, idx.action_act_id], idx.action_hi, atol=1e-6)
    assert np.allclose(ctrl[1, idx.action_act_id], idx.action_lo, atol=1e-6)
    beak = np.setdiff1d(np.arange(env.mj_model.nu), idx.action_act_id)
    assert np.allclose(ctrl[:, beak], idx.init_ctrl[beak])
    assert np.allclose(np.asarray(st.info["a_prev"]), ac)
    assert np.allclose(np.asarray(st.obs["state"][:, TK.get_task("getup").obs_slices["last_action"]]), ac)


def test_getup_curriculum_and_gstep():
    """getup: odul = paylasilan matematik (T.reward_terms + ilerlemeli agirliklar) ortamin topladigi durumda;
    p = gstep / curriculum_steps; gstep sifirlamalar arasinda korunur; zaman asimi 400 adim, dusme yok."""
    env, reset, step = _task("getup_plain")
    st = reset(_keys(NT, 18))
    st = st.replace(info=dict(st.info, gstep=jnp.asarray([0, 499, 624, 2000], jnp.int32),
                              steps=st.info["steps"].at[0].set(env.max_episode_steps - 1)))
    pre = jax.tree_util.tree_map(np.asarray, st.info)
    rng = np.random.default_rng(4)
    act = (0.3 * rng.standard_normal((NT, 14))).astype(np.float32)
    st1 = step(st, jnp.asarray(act))
    assert np.asarray(st1.done).tolist() == [1.0, 0.0, 0.0, 0.0] and float(st1.info["time_out"][0]) == 1.0
    assert np.asarray(st1.info["gstep"]).tolist() == [1, 500, 625, 2001]
    assert int(st1.info["steps"][0]) == env.max_episode_steps
    for i in (1, 2, 3):
        s = _np_state(st1, i, env, action=np.clip(act[i], -1, 1), last_action=pre["a_prev"][i],
                      **{k: pre[k][i] for k in ("phi_up_prev", "phi_h_prev", "head_floor_prev")})
        terms = T.reward_terms(np, s, "getup")
        p = min(pre["gstep"][i] + 1, 1000) / 1000.0
        w = T.reward_weights(np, "getup", p)
        for k in terms:
            got, ref = float(st1.metrics["rew_" + k][i]), float(terms[k] * w[k] * C.CTRL_DT)
            assert abs(got - ref) <= 1e-5 + 1e-4 * abs(ref), (i, k, got, ref, p)
        total = sum(float(st1.metrics["rew_" + k][i]) for k in terms)
        assert abs(total - float(st1.reward[i])) < 1e-4
        ev, inc = T.getup_events(np, s, {"success_t": pre["ev_success_t"][i], "succeeded": pre["ev_succeeded"][i]})
        for k in inc:
            assert abs(float(st1.metrics[k][i]) - float(inc[k])) < 1e-5, k
        mem = T.getup_memory(np, s)
        for k in mem:
            assert abs(float(st1.info[k][i]) - float(mem[k])) < 1e-6, k
    # gec agirliklar: p = 0.5 -> erken, p = 0.625 -> yari, p = 1 -> son (action_rate -0.01 -> -0.1)
    st2 = step(st1, jnp.asarray(act))
    assert int(st2.info["steps"][0]) == 1 and int(st2.info["gstep"][0]) == 2          # sifirlandi, sayac surer
    assert float(env._progress(jnp.int32(500))) == 0.5
    assert float(OpenDuckEnv(task="getup", final_weights=True, **_PLAIN)._progress(jnp.int32(0))) == 1.0
    assert float(OpenDuckEnv(task="getup", **_PLAIN)._progress(jnp.int32(10 ** 6))) == 0.0


def test_main_reward_and_control():
    """main: adim sirasi (TASK.md): hedefler q_ref(ONCEKI durus) + 0.5 clip(a), bas = HOME + s1 (HEAD_SIT - HOME);
    sonra durus filtresi; odul terimleri ve olay artislari paylasilan matematikle ayni."""
    env, reset, step = _task("main_plain")
    idx = env.idx
    st = reset(_keys(NT, 19))
    seat_q, _ = _bank_qpos("seated", 3, -1.1)
    cmds = np.array([[0.2, 0.0, 0.3, C.H_HOME, 0.0], [0.0, 0.0, 0.0, C.H_HOME, 1.0],
                     [0.0, 0.0, 0.0, 0.09, 0.0], [0.0, 0.0, 0.0, C.H_HOME, 1.0]], np.float32)
    st = env.set_command(st, jnp.asarray(cmds))
    for i, (q, al, hr) in enumerate([(idx.init_qpos, 0.0, C.H_HOME), (seat_q, 1.0, C.H_HOME),
                                     (idx.init_qpos, 0.0, 0.1), (idx.init_qpos, 0.5, C.H_HOME)]):
        st = _put(st, i, q, alpha=al, h_ref=hr)
    pre = jax.tree_util.tree_map(np.asarray, st.info)
    rng = np.random.default_rng(5)
    act = (0.4 * rng.standard_normal((NT, 10))).astype(np.float32)
    st1 = step(st, jnp.asarray(act))
    assert not np.any(np.asarray(st1.done))
    ctrl = np.asarray(st1.data.ctrl, np.float64)
    for i in range(NT):
        ref = T.posture_ref(np, pre["h_ref"][i].astype(np.float64), pre["alpha"][i].astype(np.float64),
                            idx.default_obs_q)
        tg, ac = T.action_to_targets(np, act[i].astype(np.float64), ref["q_ref"])
        want = idx.init_ctrl.copy()
        want[idx.head_act_id] += ref["head_offset"]
        want[idx.action_act_id] = tg
        assert np.allclose(ctrl[i], np.clip(want, idx.ctrl_lo, idx.ctrl_hi), atol=2e-6), (i, ctrl[i] - want)
        h1, a1 = T.posture_step(np, pre["h_ref"][i], pre["alpha"][i], pre["command"][i])
        assert abs(float(st1.info["h_ref"][i]) - h1) < 1e-6 and abs(float(st1.info["alpha"][i]) - a1) < 1e-6
        s = _np_state(st1, i, env, action=ac, last_action=pre["a_prev"][i], command=pre["command"][i],
                      phase=np.asarray(st1.info["phase"][i]), h_ref=h1, alpha=a1)
        s["air_time"], s["first_contact"] = T.update_air_time(np, pre["air_time"][i], s["contact"], C.CTRL_DT)
        terms = T.reward_terms(np, s, "main")
        w = T.reward_weights(np, "main")
        for k in terms:
            got, ref_v = float(st1.metrics["rew_" + k][i]), float(terms[k] * w[k] * C.CTRL_DT)
            assert abs(got - ref_v) <= 1e-5 + 1e-4 * abs(ref_v), (i, k, got, ref_v)
        assert abs(sum(float(st1.metrics["rew_" + k][i]) for k in terms) - float(st1.reward[i])) < 1e-4
        ev0 = {k: pre["ev_" + k][i] for k in ("prev_sit", "mode_t", "standup_pending", "sitdown_pending")}
        ev, inc = T.main_events(np, s, ev0, 0.0)
        for k in inc:
            assert abs(float(st1.metrics[k][i]) - float(inc[k])) < 1e-6, (i, k)
        for k in ev:
            assert abs(float(st1.info["ev_" + k][i]) - float(ev[k])) < 1e-6, (i, k)
        cobs = np.asarray(st1.obs["state"][i])
        assert np.allclose(cobs, T.observation(np, dict(s, last_action=ac), "main"), atol=1e-5)


def test_main_head_and_push_gating():
    """main (tam DR): alpha >= ALPHA_STAND_MAX iken itme darbesi ve rastgele bas hedefi uygulanmaz (zamanlayicilar
    yine isler); alpha = 0 iken walk gibi."""
    env, reset, step = _task("main_full")
    idx = env.idx
    st = reset(_keys(NT, 20))
    seat_q, _ = _bank_qpos("seated", 5, 0.3)
    st = env.set_command(st, jnp.asarray([[0.0, 0.0, 0.0, C.H_HOME, 1.0]] * 2 + [[0.0, 0.0, 0.0, C.H_HOME, 0.0]] * 2,
                                         jnp.float32))
    st = _put(st, 0, seat_q, alpha=1.0, h_ref=C.H_HOME, push_timer=0.001, head_timer=0.001)
    st = _put(st, 2, idx.init_qpos, alpha=0.0, h_ref=C.H_HOME, push_timer=0.001, head_timer=0.001)
    copy = lambda src, dst: jax.tree_util.tree_map(lambda x: x.at[dst].set(x[src]), st)
    st = copy(0, 1)
    st = copy(2, 3)
    st = _put(st, 1, push_timer=10.0)
    st = _put(st, 3, push_timer=10.0)
    st1 = step(st, jnp.zeros((NT, 10)))
    assert not np.any(np.asarray(st1.done))
    q = np.asarray(st1.data.qpos)
    assert np.array_equal(q[0], q[1])                                 # oturusta itme yok
    assert np.abs(q[2] - q[3]).max() > 1e-4                           # ayakta itme var
    pt = np.asarray(st1.info["push_timer"])
    lo, hi = C.DOMAIN_RAND["push_interval_s"]
    assert lo <= pt[0] <= hi and lo <= pt[2] <= hi                    # zamanlayici her iki durumda yenilendi
    ho, ht = np.asarray(st1.info["head_offset"]), np.asarray(st1.info["head_target"])
    assert np.all(ho[0] == 0.0) and np.allclose(ho[2], ht[2] * C.CTRL_DT / C.HEAD_RANDOM["filter_tau"], atol=1e-7)
    ref = T.posture_ref(np, np.float64(C.H_HOME), np.float64(1.0), idx.default_obs_q)
    head = np.asarray(st1.data.ctrl)[:, idx.head_act_id]
    assert np.allclose(head[0], np.clip(idx.init_ctrl[idx.head_act_id] + ref["head_offset"],
                                        idx.ctrl_lo[idx.head_act_id], idx.ctrl_hi[idx.head_act_id]), atol=1e-6)
    assert np.allclose(head[2], idx.init_ctrl[idx.head_act_id] + ho[2], atol=1e-6)


def _c_ctrl(env, a):
    """MuJoCo C tarafi icin ctrl (numpy): main oturus (h_ref = H_HOME, alpha = 1) ya da getup."""
    idx = env.idx
    ctrl = idx.init_ctrl.copy()
    if env.task == "main":
        ref = T.posture_ref(np, np.float64(C.H_HOME), np.float64(1.0), idx.default_obs_q)
        tg, _ = T.action_to_targets(np, a.astype(np.float64), ref["q_ref"])
        ctrl[idx.head_act_id] += ref["head_offset"]
    else:
        S = TK.action_scale_vector("getup", idx.default_action_q, idx.action_lo, idx.action_hi)
        tg, _ = T.action_to_targets(np, a.astype(np.float64), idx.default_action_q, scale=S, clip=1.0,
                                    lo=idx.action_lo, hi=idx.action_hi)
    ctrl[idx.action_act_id] = tg
    return np.clip(ctrl, idx.ctrl_lo, idx.ctrl_hi)


def _dynamics_vs_c(task, kind, amp, tol, seed):
    env, reset, step = _task(task + "_plain")
    m, idx = env.mj_model, env.idx
    st = reset(_keys(NT, seed))
    if task == "main":
        st = env.set_command(st, jnp.asarray([0.0, 0.0, 0.0, C.H_HOME, 1.0], jnp.float32))
    datas = []
    for i in range(NT):
        q, v = _bank_qpos(kind, 7 * i + 1, 0.9 * i - 1.3)
        st = _put(st, i, q, v, **(dict(alpha=1.0, h_ref=C.H_HOME) if task == "main" else {}))
        d = mujoco.MjData(m)
        d.qpos[:] = np.asarray(st.data.qpos[i], np.float64)
        d.qvel[:] = np.asarray(st.data.qvel[i], np.float64)
        datas.append(d)
    rng = np.random.default_rng(seed)
    foot_foot = [False] * NT
    for _ in range(10):
        a = (amp * rng.standard_normal((NT, env.action_size))).astype(np.float32)
        st = step(st, jnp.asarray(a))
        for i, d in enumerate(datas):
            d.ctrl[:] = _c_ctrl(env, a[i])
            assert np.abs(np.asarray(st.data.ctrl[i]) - d.ctrl).max() < 2e-6
            for _ in range(C.N_SUBSTEPS):
                mujoco.mj_step(m, d)
                foot_foot[i] |= any(set(c.geom) == set(idx.foot_geom) for c in d.contact[:d.ncon])
    assert not np.any(np.asarray(st.done))
    errs = [np.abs(np.asarray(st.data.qpos[i]) - d.qpos).max() for i, d in enumerate(datas) if not foot_foot[i]]
    assert len(errs) >= NT - 1 and max(errs) < tol, (errs, foot_foot)
    return errs


def test_dynamics_match_c_seated():
    """Oturus bankasi pozlarinda (govde kutusu + ayaklar yerde) 10 kontrol adimi rastgele aksiyon: MJX == MuJoCo C.
    Olculen: en buyuk qpos farki ~7e-3 (16 durumda; temas degisimleri), tutmada (a = 0) ~6e-4."""
    _dynamics_vs_c("main", "seated", 0.3, 1.5e-2, 21)


def test_dynamics_match_c_fallen():
    """Dusmus bankadaki pozlarda (getup modeli, ek zemin kutulari) 10 kontrol adimi: MJX == MuJoCo C
    (ayak-ayak kutu temasi olan durumlar haric). Olculen en buyuk qpos farki ~4e-4."""
    _dynamics_vs_c("getup", "fallen", 0.2, 1e-2, 22)


def test_floor_flags_match_c():
    """trunk_floor / head_floor (MJX temas yuvalari) ve head_vel sensoru == MuJoCo C, banka durumlarinda."""
    for task, kinds in (("main", ("seated",)), ("getup", ("fallen", "midfall"))):
        env = _task(task + "_plain")[0]
        m, idx = env.mj_model, env.idx
        f = jax.jit(lambda d: extract_state(mjx.forward(env.mjx_model, d), idx, task))
        seen = {"trunk_floor": set(), "head_floor": set()}
        for kind in kinds:
            b = PB.bank(kind)
            for j in range(0, len(b["qpos"]), max(1, len(b["qpos"]) // 48)):
                d = mujoco.MjData(m)
                d.qpos[:], d.qvel[:], d.ctrl[:] = b["qpos"][j], b["qvel"][j], b["ctrl"][j]
                mujoco.mj_forward(m, d)
                dx = env._data0.replace(qpos=jnp.asarray(d.qpos, jnp.float32), qvel=jnp.asarray(d.qvel, jnp.float32),
                                        ctrl=jnp.asarray(d.ctrl, jnp.float32))
                sj = {k: np.asarray(v) for k, v in f(dx).items()}
                for key, geoms in (("trunk_floor", idx.trunk_geoms), ("head_floor", idx.head_geoms)):
                    fc = float(any(c.dist < C.CONTACT_EPS and idx.floor_geom in c.geom
                                   and (c.geom[0] in geoms or c.geom[1] in geoms) for c in d.contact[:d.ncon]))
                    assert float(sj[key]) == fc, (task, kind, j, key)
                    seen[key].add(fc)
                if task == "getup":
                    assert np.abs(sj["head_vel"] - d.sensordata[idx.s_head_vel]).max() < 1e-4
        assert seen["trunk_floor"] == {0.0, 1.0} or task == "main", seen
        if task == "getup":
            assert seen["head_floor"] == {0.0, 1.0}, seen


def test_task_autoreset_equivalence():
    """main/getup: VmapAutoReset (lax.cond) == vmap(env.step); biten ortam taze sifirlamadan gelir, bolum
    bilgisi (steps, episode_metrics, gstep) bitmis bolumden kalir."""
    for task in ("main", "getup"):
        env, reset, step = _task(task + "_plain")
        st = reset(_keys(NT, 23))
        if task == "main":
            q = np.array(env.idx.init_qpos)
            q[3:7] = [np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0]
            q[2] += 0.05
            st = _put(st, 1, q)
        else:
            st = _put(st, 1, steps=env.max_episode_steps - 1)
        a = jnp.zeros((NT, env.action_size))
        st1 = step(st, a)
        assert np.asarray(st1.done).tolist() == [0.0, 1.0, 0.0, 0.0], (task, np.asarray(st1.done))
        ref = jax.jit(jax.vmap(env.step))(st, a)
        for x, y in zip(jax.tree_util.tree_leaves(ref), jax.tree_util.tree_leaves(st1)):
            assert np.array_equal(np.asarray(x), np.asarray(y))
        assert float(st1.info["episode_metrics"]["length"][1]) == 1.0
        assert int(st1.info["steps"][1]) == int(np.asarray(st.info["steps"])[1]) + 1
        assert np.all(np.asarray(st1.data.time)[1] == 0.0)               # taze sifirlama verisi
        if task == "getup":
            assert int(st1.info["gstep"][1]) == int(st.info["gstep"][1]) + 1


def _rand_params(task, hidden=(32, 16)):
    from brax.training.acme import running_statistics, specs
    from brax.training.agents.ppo import networks as ppo_networks
    from rl import train_mjx
    spec = TK.get_task(task)
    h = ",".join(map(str, hidden))
    args = train_mjx.parse_args(["--out", "x", "--task", task, "--num-envs", "8", "--policy-hidden", h,
                                 "--value-hidden", h])
    obs_size = {"state": (spec.num_obs,), "privileged_state": (spec.num_critic_obs,)}
    net = train_mjx.network_factory(args)(obs_size, spec.num_actions, preprocess_observations_fn=running_statistics.normalize)
    k1, k2 = jax.random.split(jax.random.PRNGKey(1))
    norm = running_statistics.init_state({k: specs.Array(v, jnp.dtype("float32")) for k, v in obs_size.items()})
    rng = np.random.default_rng(2)
    batch = {k: jnp.asarray(rng.normal(0.3, 1.5, (64,) + v), jnp.float32) for k, v in obs_size.items()}
    norm = running_statistics.update(norm, batch)
    return (norm, net.policy_network.init(k1), net.value_network.init(k2)), ppo_networks.make_inference_fn(net), batch


def test_onnx_export_tasks():
    """main [N,53] -> [N,10], getup [N,48] -> [N,14]; meta = policy_meta.build_meta; gorev/parametre boyutu
    uyusmazligi reddedilir; --task varsayilanlari TaskSpec'ten (walk degismez)."""
    from rl import train_mjx
    import onnxruntime as ort
    import json
    a = train_mjx.parse_args(["--out", "x"])
    assert (a.task, a.discounting, a.policy_hidden, a.value_hidden) == ("walk", 0.97, "512,256,128", "512,256,128")
    a = train_mjx.parse_args(["--out", "x", "--task", "getup"])
    assert (a.discounting, a.policy_hidden, a.value_hidden) == (0.99, "512,256,128", "512,256,128")
    assert train_mjx.parse_args(["--out", "x", "--task", "main", "--discounting", "0.99"]).discounting == 0.99
    for task in ("main", "getup"):
        spec = TK.get_task(task)
        params, make_policy, batch = _rand_params(task)
        assert train_mjx.params_dims(params) == (spec.num_obs, spec.num_critic_obs, spec.num_actions)
        env = _task(task + "_plain")[0]
        with tempfile.TemporaryDirectory() as tmp:
            err = train_mjx.export_policy(params, tmp, 77, make_policy, env.idx, task)
            assert err < 1e-4
            meta = json.load(open(os.path.join(tmp, "policy_meta.json")))
            assert meta == json.loads(json.dumps(PM.build_meta(77, "mjx_brax", env.idx, task)))
            assert meta["task"] == task and meta["obs_dim"] == spec.num_obs and meta["action_dim"] == spec.num_actions
            sess = ort.InferenceSession(os.path.join(tmp, "policy.onnx"), providers=["CPUExecutionProvider"])
            x = np.asarray(batch["state"][:5])
            out = sess.run(["action"], {"obs": x})[0]
            assert out.shape == (5, spec.num_actions) and np.all(np.isfinite(out))
            assert sess.get_inputs()[0].shape[1] == spec.num_obs
            other = "getup" if task == "main" else "main"
            try:
                train_mjx.export_policy(params, tmp, 78, make_policy, None, other)
                raise AssertionError("boyut uyusmazligi kabul edildi")
            except RuntimeError:
                pass
            assert json.load(open(os.path.join(tmp, "policy_meta.json")))["steps"] == 77


def test_env_matmul_precision_tasks():
    """main/getup reset/step izlemesinde de hassasiyeti belirsiz dot_general yok."""
    for task in ("main", "getup"):
        env = _task(task + "_full")[0]
        b = VmapAutoReset(env)
        keys = jax.ShapeDtypeStruct((2, 2), jnp.uint32)
        st = jax.eval_shape(b.reset, keys)
        act = jax.ShapeDtypeStruct((2, env.action_size), jnp.float32)
        for j in (jax.make_jaxpr(b.reset)(keys), jax.make_jaxpr(b.step)(st, act)):
            ps = _dot_precisions(j.jaxpr, [])
            assert len(ps) > 50 and all(p is not None for p in ps), (task, len(ps), sum(p is None for p in ps))


def _main():
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    order = ["test_reset_step_shapes", "test_stand_still_1s", "test_dynamics_match_c",
             "test_termination_on_tilt_and_autoreset",
             "test_timeout", "test_command_resample_and_lock", "test_dr_sampling_ranges", "test_setconst_matches_c", "test_extract_state_matches_c",
             "test_onnx_export", "test_env_matmul_precision"]
    tests.sort(key=lambda kv: order.index(kv[0]) if kv[0] in order else len(order))
    sel = sys.argv[1:]
    failed = 0
    for name, fn in tests:
        if sel and not any(s in name for s in sel):
            continue
        t = time.time()
        try:
            fn()
            print("OK   %-45s %6.1fs" % (name, time.time() - t), flush=True)
        except Exception:
            failed += 1
            import traceback
            traceback.print_exc()
            print("FAIL %-45s %6.1fs" % (name, time.time() - t), flush=True)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    _main()


# --- CPU (MuJoCo C) == MJX adim adim (S4 review parity) ----------------------------------------
class _ParityRNG(object):
    """np.random.Generator yerine: env_cpu'nun cektigi sayilari (boyuta gore kuyruk) MJX anahtarlarindan verir."""

    def __init__(self):
        self.q = {}

    def push(self, arr):
        arr = np.asarray(arr, np.float64).ravel()
        self.q.setdefault(arr.size, []).append(arr)

    def random(self, size=None, out=None):
        n = out.size if out is not None else int(np.prod(size))
        v = self.q[n].pop(0)
        if out is not None:
            out[...] = v.reshape(out.shape)
            return out
        return v.reshape(size).copy()


def _parity_draws(task):
    """(B,2) MJX anahtarlari -> env_cpu duzeninde ayni duzgun sayilar: sifirlama blogu [eklem 14, yaw, taban hizi 2,
    faz, komut 4 (kullanilmaz), bas/itme zamanlayicisi, tur, banka indeksi, (main) segment 8] ve adim basina
    (segment 8, o adimin sifirlama blogu). DR/gurultu/bas/itme kapali (_PLAIN)."""
    U = jax.random.uniform

    def reset_u(rk):
        _, _, k_q, k_yaw, k_v, k_ph, k_cmd, k_ht, k_pt, k_n = jax.random.split(rk, 10)
        k_kind, k_i = jax.random.split(jax.random.fold_in(k_n, 2))
        parts = [U(k_q, (14,)), U(k_yaw, (1,)), U(k_v, (2,)), U(k_ph, (1,)), jnp.full((4,), 0.5), U(k_ht, (1,)),
                 U(k_pt, (1,)), U(k_kind, (1,)), U(k_i, (1,))]
        return jnp.concatenate(parts + ([U(k_cmd, (T.NUM_SEGMENT_U,))] if task == "main" else []))

    def step_u(r):
        ks = jax.random.split(r, 6)
        return U(ks[3], (T.NUM_SEGMENT_U,)), reset_u(ks[5])

    return jax.jit(jax.vmap(reset_u)), jax.jit(jax.vmap(step_u))


def test_cpu_mjx_lockstep_parity():
    """main/getup: env_cpu.VecEnv (MuJoCo C) == OpenDuckEnv (MJX) adim adim, ayni durum + ayni aksiyon + ayni rastgele
    sayilar (MJX anahtarlarindan CPU'ya aktarilir: sifirlama turu, banka durumu, yaw, segmentler, oto-sifirlama).
    Fizik farki birikmesin diye her adimdan sonra C durumu (qpos/qvel/warmstart) MJX'e kopyalanir; MJX 8/8 Newton
    yinelemesi (4'te temas bayraklari C'den ayrisir). Karsilastirilan: aktor/kritik gozlem, odul, her odul terimi,
    olay artislari, done/time_out, gorev durumu (h_ref, alpha, seg, seg_timer, komut, adim sayaci, olaylar, getup
    bellegi, basari bayragi, reset_kind)."""
    from rl.env_cpu import VecEnv
    n, steps = 4, 30
    for task, rp in (("main", {"home": 0.5, "seated": 0.5}), ("getup", {"fallen": 0.5, "midfall": 0.25, "stand": 0.25})):
        spec = TK.get_task(task)
        menv = OpenDuckEnv(task=task, reset_probs=rp, solver_iterations=8, ls_iterations=8,
                           final_weights=task == "getup", **_PLAIN)
        cenv = VecEnv(n, num_workers=0, task=task, reset_probs=rp, progress=1.0, **_PLAIN)
        core = cenv._core
        core.rngs = rngs = [_ParityRNG() for _ in range(n)]
        reset_u, step_u = _parity_draws(task)
        keys = _keys(n, 31)
        for j, u in enumerate(np.asarray(reset_u(keys))):
            rngs[j].push(u)
        st = jax.jit(jax.vmap(menv.reset))(keys)
        obs_c, crit_c = cenv.reset()
        assert np.array_equal(core.reset_kind, np.asarray(st.info["reset_kind"])), task
        assert len(set(core.reset_kind.tolist())) >= 2, (task, core.reset_kind)
        # main: segment suresi 3. adimda dolar (yeni segment ayni sayilarla); getup: env 0 10 adim sonra zaman asimi
        info = dict(st.info)
        if task == "main":
            core.seg_timer[:] = 0.05
            info["seg_timer"] = jnp.full((n,), 0.05, jnp.float32)
        else:
            core.step_count[0] = spec.max_episode_steps - 10
            core.datas[0].time = core.step_count[0] * C.CTRL_DT        # env_cpu bozuk-fizik testi d.time'a bakar
            info["steps"] = info["steps"].at[0].set(spec.max_episode_steps - 10)
        st = st.replace(info=info)
        jstep = jax.jit(jax.vmap(menv.step))
        rng = np.random.default_rng(32)
        n_done = 0
        for t in range(-1, steps):
            if t >= 0:
                a = (0.3 * rng.standard_normal((n, spec.num_actions)) * (np.arange(n) % 2)[:, None]).astype(np.float32)
                a[n - 1] *= 6.0                                          # kirpma (main: 1.5, getup: 1.0)
                seg8, ru = (np.asarray(x) for x in step_u(st.info["rng"]))
                for j in range(n):
                    rngs[j].push(seg8[j])
                    rngs[j].push(ru[j])
                sum0, met0 = core.ep_sum.copy(), core.ep_met.copy()
                obs_c, crit_c, rew_c, done_c, info_c = cenv.step(a)
                for r_ in rngs:
                    r_.q = {}
                st = jstep(st, jnp.asarray(a))
                done_m = np.asarray(st.done) > 0.5
                assert np.array_equal(done_c, done_m), (task, t, done_c, done_m)
                assert np.array_equal(info_c["time_outs"], np.asarray(st.info["time_out"]) > 0.5), (task, t)
                assert np.allclose(rew_c, np.asarray(st.reward), atol=1e-3), (task, t, rew_c - np.asarray(st.reward))
                n_done += int(done_c.sum())
                live = ~done_c
                inc = core.ep_sum - sum0
                for i, k in enumerate(spec.reward_keys):
                    m = np.asarray(st.metrics["rew_" + k])
                    assert np.allclose(inc[live, i], m[live], atol=1e-4, rtol=1e-3), (task, t, k, inc[live, i], m[live])
                inc = core.ep_met - met0
                for i, k in enumerate(spec.episode_metrics):
                    m = np.asarray(st.metrics[k])
                    assert np.allclose(inc[live, i], m[live], atol=1e-3), (task, t, k, inc[live, i], m[live])
            om, cm = np.asarray(st.obs["state"]), np.asarray(st.obs["privileged_state"])
            assert np.allclose(obs_c, om, atol=2e-3), (task, t, np.abs(obs_c - om).max(axis=0))
            assert np.allclose(crit_c, cm, atol=2e-3), (task, t, np.abs(crit_c - cm).max(axis=0))
            for name in ("last_action", "command", "posture", "phase"):
                if name in spec.obs_slices:
                    sl = spec.obs_slices[name]
                    assert np.allclose(obs_c[:, sl], om[:, sl], atol=1e-5), (task, t, name)
            inf = jax.tree_util.tree_map(np.asarray, st.info)
            steps_m = np.where(np.asarray(st.done) > 0.5, 0, inf["steps"])
            assert np.array_equal(core.step_count, steps_m) and np.array_equal(core.reset_kind, inf["reset_kind"])
            assert np.allclose(core.a_prev, inf["a_prev"], atol=1e-6)
            ev_keys = ("prev_sit", "mode_t", "standup_pending", "sitdown_pending") if task == "main" else \
                ("success_t", "succeeded")
            for k in ev_keys:
                assert np.allclose(core.ev[k], inf["ev_" + k], atol=1e-5), (task, t, k)
            if task == "main":
                for k in ("h_ref", "alpha", "seg", "seg_timer", "command", "phase", "air_time"):
                    assert np.allclose(getattr(core, k), inf[k], atol=1e-5), (task, t, k, getattr(core, k), inf[k])
            else:
                for k in ("phi_up_prev", "phi_h_prev", "head_floor_prev", "head_v2_prev"):
                    assert np.allclose(core.mem[k], inf[k], atol=1e-3), (task, t, k)
            # fizik farki birikmesin: C durumunu MJX'e kopyala
            qp, qv, qw = (np.stack([getattr(d, f) for d in core.datas]) for f in ("qpos", "qvel", "qacc_warmstart"))
            st = st.replace(data=st.data.replace(qpos=jnp.asarray(qp, jnp.float32), qvel=jnp.asarray(qv, jnp.float32),
                                                 qacc_warmstart=jnp.asarray(qw, jnp.float32)))
        if task == "getup":
            assert n_done >= 1                                            # zaman asimi + oto-sifirlama denendi
        cenv.close()
