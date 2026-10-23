# -*- coding: utf-8 -*-
"""
rl/actuators.py ve robot_model'in tr35_b2 profili testleri (OpenDuck-TR spec 3.2.3, A3, A9). Calistirma (repo kokunden):

  python3 -m rl.tests.test_actuators          # duz assert'ler
  python3 -m pytest rl/tests/test_actuators.py

jax kuruluysa numpy-jax esitligi ve MJX limit metrigi de denenir (yoksa o testler atlanir).
"""
import copy
import os
import subprocess
import sys
import time

import mujoco
import numpy as np

from rl import actuators as A
from rl import config as C
from rl import task_math as T
from rl.robot_model import VARIANTS, RobotIndex, build_model, build_spec, expand_state, play_joints

try:
    import jax
    import jax.numpy as jnp
except ImportError:          # CPU venv'inde jax yok
    jax = jnp = None

TR = "tr35_b2"
MG = np.deg2rad(C.JOINT_RANGE_MARGIN_DEG)


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


_M = {}


def _model(variant="walk", backlash=False, robot=TR):
    k = (variant, backlash, robot)
    if k not in _M:
        _M[k] = build_model(variant=variant, robot=robot, backlash=backlash)
    return _M[k]


def _tab(variant="walk", backlash=False):
    return A.actuator_table(_model(variant, backlash))


# --- profil secimi ---------------------------------------------------------------------------------
def test_profile_selection():
    assert C.ROBOT_PROFILES == ("xl330_27", "tr35_b2") and C.ROBOT_PROFILE in C.ROBOT_PROFILES
    assert A.profile(None) == C.ROBOT_PROFILE and A.profile(TR) == TR
    try:
        A.profile("xl430")
        raise AssertionError("gecersiz profil kabul edildi")
    except ValueError:
        pass
    code = "from rl import config as C, actuators as A; print(C.ROBOT_PROFILE, A.profile(None))"
    env = dict(os.environ, OPENDUCK_ROBOT=TR, PYTHONDONTWRITEBYTECODE="1")
    if not os.path.isfile(C.DERIVED_PATHS[TR]):          # P5a: tr35 sureci turetilmis dosya ister (derive oncesi)
        from rl.tests.tr35_scene import bootstrap_for
        env["OPENDUCK_DERIVED"] = bootstrap_for(C.SCENE_PATH)
    r = subprocess.run([sys.executable, "-c", code], cwd=C.ROOT, env=env, capture_output=True, text=True, timeout=120)
    assert r.returncode == 0 and r.stdout.split() == [TR, TR], (r.stdout, r.stderr)
    env["OPENDUCK_ROBOT"] = "yok"
    r = subprocess.run([sys.executable, "-c", code], cwd=C.ROOT, env=env, capture_output=True, text=True, timeout=120)
    assert r.returncode != 0 and "OPENDUCK_ROBOT" in r.stderr
    # genel DR: xl330 nesnenin kendisi; tr35 aktuator carpanlari kapali, kutle spec 4 araligi
    assert A.domain_rand("xl330_27") is C.DOMAIN_RAND
    d = A.domain_rand(TR)
    assert set(d) == set(C.DOMAIN_RAND)
    for k in ("kp_scale", "damping_scale", "frictionloss_scale", "armature_scale"):
        assert d[k] == (1.0, 1.0)
    assert d["link_mass_scale"] == (0.9, 1.15) and d["action_delay_steps"] == (0, 0)
    assert d["friction"] == C.DOMAIN_RAND["friction"] and d["push_vel"] == C.DOMAIN_RAND["push_vel"]


def test_xl330_model_unchanged():
    """xl330_27: aktuator/eklem alanlari MJCF ile ayni, profil isareti ve bosluk eklemi yok, range = ctrlrange."""
    raw = mujoco.MjModel.from_xml_path(C.SCENE_PATH)
    m = build_model(robot="xl330_27")
    assert A.model_profile(m) == "xl330_27" and m.ntext == raw.ntext and len(play_joints(m)) == 0
    for k in ("actuator_gainprm", "actuator_biasprm", "actuator_forcerange", "actuator_ctrlrange", "jnt_range",
              "dof_damping", "dof_armature", "dof_frictionloss"):
        assert np.array_equal(getattr(m, k), getattr(raw, k)), k
    for v in VARIANTS:
        mv = build_model(variant=v, robot="xl330_27")
        jr = mv.jnt_range[mv.actuator_trnid[:, 0]]
        assert np.array_equal(jr, mv.actuator_ctrlrange), v
    try:
        build_model(robot="xl330_27", backlash=True)
        raise AssertionError("xl330'da bosluk eklemi kabul edildi")
    except ValueError:
        pass
    assert RobotIndex(m).act is None and RobotIndex(m).robot == "xl330_27"


# --- sinif parametreleri ve degismezler ----------------------------------------------------------
def test_tr35_class_params():
    cmap = A.JOINT_CLASS[TR]
    assert cmap["hip_yaw"] == "es3054" and cmap["neck_pitch"] == "ds32xx" and cmap["mouth"] == "sg90"
    assert all(cmap[j] == "ds32xx" for j in ("hip_roll", "hip_pitch", "knee", "ankle"))
    assert all(cmap[j] == "mg90s" for j in ("head_pitch", "head_yaw", "head_roll"))
    ds = A.CLASSES["ds32xx"]
    assert (ds["tau_s"], ds["omega0"], ds["pband_deg"]) == (2.88, 5.72, 3.0)        # spec 3.2.3 varsayilanlari
    for bl in (False, True):
        for v in VARIANTS:
            m = _model(v, bl)
            t = A.actuator_table(m)
            assert A.model_profile(m) == TR and RobotIndex(m).robot == TR
            assert int(t["on_leg"].sum()) == 9                                         # 8 bacak + boyun DS
            for i, name in enumerate(t["names"]):
                p = A.CLASSES[A.joint_class(TR, name)]
                ts, kp = p["tau_s"], p["tau_s"] / np.deg2rad(p["pband_deg"])
                assert np.isclose(m.actuator_gainprm[i, 0], kp) and np.isclose(m.actuator_biasprm[i, 1], -kp)
                assert m.actuator_biasprm[i, 0] == 0.0 and m.actuator_biasprm[i, 2] == 0.0
                assert np.allclose(m.actuator_forcerange[i], [-ts, ts]) and m.actuator_forcelimited[i]
                dof = t["act_dadr"][i]
                assert np.isclose(m.dof_damping[dof], ts / p["omega0"])
                assert np.isclose(m.dof_armature[dof], p["armature"])
                assert np.isclose(m.dof_frictionloss[dof], p["friction_frac"] * ts)
                assert t["board"][i] == (0 if p["rail"] == "leg" else 1)
                assert np.isclose(t["kp"][i], kp) and np.isclose(t["damping"][i], ts / p["omega0"])


def test_range_margin_invariant():
    """Degismez: butun varyantlarda (bosluklu / bosluksuz) aktuatorlu ana eklemlerde range = ctrlrange +- 3 derece,
    varyant sinirlayicilarinin ctrlrange'i korunur; bosluk eklemleri aktuatorsuz ve disarida."""
    for bl in (False, True):
        for v in VARIANTS:
            m = _model(v, bl)
            t = A.actuator_table(m)
            jr, cr = m.jnt_range[t["act_jnt"]], m.actuator_ctrlrange
            assert np.allclose(jr[:, 0], cr[:, 0] - MG, atol=1e-12) and np.allclose(jr[:, 1], cr[:, 1] + MG, atol=1e-12)
            assert np.all(m.jnt_limited[t["act_jnt"]])
            mx = build_model(variant=v, robot="xl330_27")
            assert np.allclose(cr, mx.actuator_ctrlrange), v                   # sinirlayicilar ayni
            pj = play_joints(m)
            assert len(pj) == (9 if bl else 0)
            assert not np.isin(pj, m.actuator_trnid[:, 0]).any()
            h = 0.5 * np.deg2rad(A.BACKLASH["nominal_deg"])
            for j in pj:
                assert np.allclose(m.jnt_range[j], [-h, h]) and m.jnt_limited[j]
                assert np.allclose(m.jnt_solref[j], A.BACKLASH["solref"])
                assert np.isclose(m.dof_armature[m.jnt_dofadr[j]], A.BACKLASH["armature"])
                assert m.dof_frictionloss[m.jnt_dofadr[j]] == 0.0 and m.dof_damping[m.jnt_dofadr[j]] == 0.0
    knee = _model("main").actuator("left_knee").id
    assert np.isclose(_model("main").actuator_ctrlrange[knee, 1], np.deg2rad(C.KNEE_LIMIT_DEG))
    # sinirlayicidan SONRA: ctrlrange daraltilip degismez yeniden uygulaninca range da izler
    spec = build_spec(variant="main", robot=TR)
    spec.actuator("left_knee").ctrlrange = np.array([-0.5, 0.4])
    A.enforce_range_margin(spec)
    m = spec.compile()
    j = m.actuator_trnid[m.actuator("left_knee").id, 0]
    assert np.allclose(m.jnt_range[j], [-0.5 - MG, 0.4 + MG])


def test_backlash_layout():
    m, m0 = _model("walk", True), _model("walk", False)
    t, t0 = A.actuator_table(m), A.actuator_table(m0)
    assert (m.nq, m.nv, m.nu) == (m0.nq + 9, m0.nv + 9, m0.nu)
    ds = t["on_leg"] > 0.5
    assert np.array_equal(t["play_jnt"] >= 0, ds) and np.all(t["backlash"][ds] > 0) and np.all(t["backlash"][~ds] == 0)
    for i in np.flatnonzero(ds):
        j, p = t["act_jnt"][i], t["play_jnt"][i]
        assert m.jnt_bodyid[j] == m.jnt_bodyid[p] and np.allclose(m.jnt_axis[j], m.jnt_axis[p])
        assert m.jnt_qposadr[p] == m.jnt_qposadr[j] + 1
    q, v = expand_state(m, m0.key_qpos[0], np.ones(m0.nv))
    assert np.array_equal(q, m.key_qpos[0]) and q[t["play_qadr"][ds]].max() == 0.0
    assert v.shape == (m.nv,) and np.all(v[t["play_dadr"][ds]] == 0.0) and v.sum() == m0.nv
    qb, _ = expand_state(m, np.tile(m0.key_qpos[0], (3, 1)))
    assert qb.shape == (3, m.nq)
    assert expand_state(m, m.key_qpos[0])[0] is not None and np.array_equal(expand_state(m0, m0.key_qpos[0])[0],
                                                                             m0.key_qpos[0])
    try:
        expand_state(m, np.zeros(m0.nq + 1))
        raise AssertionError("yanlis boyut kabul edildi")
    except ValueError:
        pass
    # ana eklem servo cikisi, bosluk eklemi link: link acisi = q + q_play; ayakta bosluk sinirlarinda kalir
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    h = 0.5 * np.deg2rad(A.BACKLASH["nominal_deg"])
    assert np.abs(d.qpos[t["play_qadr"][ds]]).max() < h + np.deg2rad(0.5)
    assert d.qpos[2] > 0.1


def test_home_stand_open_loop():
    """A3 (27 cm MJCF + tr35 aktuatorleri): HOME'da acik dongu 5 s ayakta, egim <= 2 derece (CPU, sifir aksiyon)."""
    from rl.env_cpu import VecEnv
    with VecEnv(1, num_workers=0, robot=TR, randomize=False, noise=False, head_random=False, push=False,
                fixed_command=(0.0, 0.0, 0.0), seed=0) as env:
        env.reset()
        core = env._core
        q0 = core.idx.init_qpos
        core.set_state(0, q0, np.zeros(core.models[0].nv), core.idx.init_ctrl)
        for _ in range(int(round(5.0 / C.CTRL_DT))):
            _, _, _, done, _ = env.step(np.zeros((1, env.num_actions)))
            assert not done.any()
        s = env.task_state()
        tilt = np.rad2deg(T.tilt_angle(np, s["quat"]))[0]
        assert tilt <= 2.0, tilt
        assert abs(s["trunk_z"][0] - q0[2]) < 0.01


# --- bolum parametreleri --------------------------------------------------------------------------
def test_sample_params():
    t = _tab("walk", True)
    nu, ds = len(t["tau_s"]), t["on_leg"] > 0.5
    rng = np.random.default_rng(0)
    p = A.sample_params(np, rng.random((4000, A.n_uniform(nu))), t, True)
    assert A.n_uniform(nu) == A.N_FIELDS * nu + 2

    def rng_ok(x, lo, hi, cover=0.03):
        assert x.min() >= lo - 1e-12 and x.max() <= hi + 1e-12, (x.min(), x.max(), lo, hi)
        assert x.min() < lo + cover * (hi - lo) and x.max() > hi - cover * (hi - lo), (x.min(), x.max(), lo, hi)

    D = A.DR
    rng_ok(p["tau_s"] / t["tau_s"], *D["tau_s"])
    rng_ok(p["tau_s"] / p["damping"] / t["omega0"], *D["omega0"])                  # b = tau_s / omega0
    pb = np.rad2deg(p["tau_s"] / p["kp"])                                         # kp = tau_s / P-bandi
    rng_ok(pb, *D["pband_deg"])
    assert abs(np.median(pb) - np.sqrt(D["pband_deg"][0] * D["pband_deg"][1])) < 0.15   # log-duzgun
    rng_ok(p["db_half"] / (0.5 * t["deadband"]), *D["deadband"])
    rng_ok(p["armature"] / t["armature"], *D["armature"])
    rng_ok(p["frictionloss"] / t["frictionloss"], *D["frictionloss"])
    rng_ok(np.rad2deg(p["backlash"][:, ds]), *D["backlash_deg"])
    assert np.all(p["backlash"][:, ~ds] == 0.0)
    (f0, f1), (s0, s1) = D["delay_substeps"]
    fast, slow = p["delay"][:, t["board"] == 0], p["delay"][:, t["board"] == 1]
    assert p["delay"].dtype == np.int32
    assert set(np.unique(fast)) == set(range(f0, f1 + 1)) and set(np.unique(slow)) == set(range(s0, s1 + 1))
    assert np.all(fast == fast[:, :1]) and np.all(slow == slow[:, :1])           # kart basina tek gecikme
    # nominal (randomize=False): u kullanilmaz
    q = A.sample_params(np, rng.random((3, A.n_uniform(nu))), t, False)
    for k, nk in (("tau_s", "tau_s"), ("kp", "kp"), ("damping", "damping"), ("armature", "armature"),
                  ("frictionloss", "frictionloss"), ("backlash", "backlash")):
        assert np.allclose(q[k], t[nk]), k
    assert np.allclose(q["db_half"], 0.5 * t["deadband"]) and np.all(q["delay"] == 0)
    # uclar
    lo = A.sample_params(np, np.zeros(A.n_uniform(nu)), t, True)
    hi = A.sample_params(np, np.full(A.n_uniform(nu), 1.0 - 1e-12), t, True)
    assert np.allclose(lo["tau_s"], t["tau_s"] * D["tau_s"][0]) and np.allclose(hi["tau_s"], t["tau_s"] * D["tau_s"][1])
    assert np.all(lo["delay"] == 0) and set(hi["delay"].tolist()) == {f1, s1}


# --- ctrl sekillendirmesi ve kuvvet ---------------------------------------------------------------------
def test_ctrl_shaping():
    q = np.full(7, 0.1)
    c = np.array([0.1, 0.105, 0.11, 0.13, 0.07, 0.09, 0.0])
    out = A.dead_band(np, c, q, 0.01)
    assert np.allclose(out, [0.1, 0.1, 0.1, 0.12, 0.08, 0.1, 0.01])
    e = np.linspace(-0.05, 0.05, 2001)
    y = A.dead_band(np, q[0] + e, np.full_like(e, q[0]), 0.01) - q[0]
    assert np.all(np.diff(y) >= -1e-15) and np.abs(np.diff(y)).max() <= 5e-5 + 1e-12          # surekli, monoton
    assert np.all(y[np.abs(e) <= 0.01] == 0.0)
    assert np.allclose(A.delayed(np, 1, np.array([0, 1, 2, 4]), np.zeros(4), np.ones(4)), [1, 1, 0, 0])
    assert np.allclose(A.delayed(np, 3, np.array([0, 1, 2, 4]), np.zeros(4), np.ones(4)), [1, 1, 1, 0])
    res, lo, hi = 0.01, np.full(5, -0.3), np.full(5, 0.3)
    x = np.array([0.0, 0.0149, -0.0251, 0.33, -0.5])
    qz = A.quantize_pwm(np, x, res, lo, hi)
    assert np.allclose(qz, [0.0, 0.01, -0.03, 0.3, -0.3])
    xr = np.random.default_rng(1).uniform(-0.29, 0.29, 1000)
    qr = A.quantize_pwm(np, xr, res, -0.3, 0.3)
    assert np.abs(qr - xr).max() <= 0.5 * res + 1e-12 and np.allclose(np.round(qr / res), qr / res)


def test_position_force_matches_mujoco():
    """position_force == MuJoCo actuator_force (ctrlrange kirpmasi ve forcerange doygunlugu dahil)."""
    m = _model("walk", True)
    t = A.actuator_table(m)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(2)
    for trial in range(20):
        mujoco.mj_resetDataKeyframe(m, d, 0)
        d.qpos[t["act_qadr"]] += rng.uniform(-0.3, 0.3, m.nu)
        d.qvel[:] = rng.uniform(-2.0, 2.0, m.nv)
        span = t["ctrl_hi"] - t["ctrl_lo"]
        d.ctrl[:] = t["ctrl_lo"] + span * rng.uniform(-0.2, 1.2, m.nu)              # ctrlrange disi da
        mujoco.mj_forward(m, d)
        f = A.position_force(np, d.ctrl, d.qpos[t["act_qadr"]], d.qvel[t["act_dadr"]], m.actuator_gainprm[:, 0],
                             m.actuator_biasprm[:, 0], m.actuator_biasprm[:, 1], m.actuator_biasprm[:, 2],
                             t["ctrl_lo"], t["ctrl_hi"], t["tau_s"])
        assert np.allclose(f, d.actuator_force, atol=1e-12), trial
        assert np.abs(f).max() <= t["tau_s"].max() + 1e-12


# --- besleme akimi ve CC olceklemesi ---------------------------------------------------------------------
def test_supply_current_and_cc():
    t = _tab()
    ts, b, i0, ist = t["tau_s"], t["damping"], t["i_idle"], t["i_stall"]
    w0 = t["omega0"]
    stall = A.supply_current(np, ts, A.tau_net(np, ts, 0.0 * ts, b), ts, ist, i0)
    assert np.allclose(stall, ist + i0)                                          # stall: I_stall + bosta
    noload = A.supply_current(np, ts, A.tau_net(np, ts, w0, b), ts, ist, i0)
    assert np.allclose(noload, i0, atol=1e-12)                                   # bosta hiz: tau_net = 0
    half = A.supply_current(np, 0.5 * ts, A.tau_net(np, 0.5 * ts, 0.0 * ts, b), ts, ist, i0)
    assert np.allclose(half - i0, 0.25 * ist)                                     # |tau_act| |tau_net| / tau_s^2
    regen = A.supply_current(np, ts, A.tau_net(np, ts, -w0, b), ts, ist, i0)      # geri surulen: |tau_net| = 2 tau_s
    assert np.allclose(regen - i0, 2.0 * ist)
    ds = t["on_leg"]
    assert np.isclose(A.rail_current(np, stall, ds), np.sum((ist + i0)[ds > 0.5]))
    assert A.cc_scale(np, np.array(10.0)) == 1.0 and np.isclose(A.cc_scale(np, np.array(32.0)), 0.5)
    x = np.linspace(15.0, 17.0, 401)
    sc = A.cc_scale(np, x)
    assert np.all(np.diff(sc) <= 0) and np.abs(np.diff(sc)).max() < 1e-2 and np.isclose(sc[200], 1.0)
    assert A.cc_scale(np, np.array(0.0)) == 1.0
    # shape_substep: tum servolar buyuk hatayla doygun -> talep > I_CC, DS forcerange = tau_s * I_CC / talep
    ep = A.sample_params(np, np.zeros(A.n_uniform(len(ts))), t, False)
    q = 0.5 * (t["ctrl_lo"] + t["ctrl_hi"])
    new = q + 0.3
    g, bp = t["kp"], -t["kp"]
    c, frc, dem, s = A.shape_substep(np, 0, q, new, q, 0.0 * q, ep, t, g, 0.0 * g, bp, 0.0 * g, b)
    assert np.allclose(c, new - ep["db_half"])                                    # gecikme 0, olu bant
    f = A.position_force(np, c, q, 0.0 * q, g, 0.0 * g, bp, 0.0 * g, t["ctrl_lo"], t["ctrl_hi"], ts)
    assert np.allclose(np.abs(f), ts)                                             # doygun
    dem_ref = A.rail_current(np, A.supply_current(np, f, f, ts, ist, i0), ds)
    assert np.isclose(dem, dem_ref) and dem > A.I_CC                               # olcekleme ONCESI talep
    assert np.isclose(s, A.I_CC / dem)
    assert np.allclose(frc[ds > 0.5], ts[ds > 0.5] * A.I_CC / dem) and np.allclose(frc[ds < 0.5], ts[ds < 0.5])
    # gecikme: k < delay -> onceki hedef
    ep2 = dict(ep, delay=np.where(t["board"] == 1, 4, 2).astype(np.int32))
    c2 = A.shape_substep(np, 1, q, new, q, 0.0 * q, ep2, t, g, 0.0 * g, bp, 0.0 * g, b)[0]
    assert np.allclose(c2, q)                                                     # hepsi onceki (bant icinde)
    c3 = A.shape_substep(np, 2, q, new, q, 0.0 * q, ep2, t, g, 0.0 * g, bp, 0.0 * g, b)[0]
    assert np.allclose(c3[t["board"] == 0], (new - ep["db_half"])[t["board"] == 0]) and np.allclose(c3[t["board"] == 1],
                                                                                                    q[t["board"] == 1])
    tn, cur, rail = A.after_substep(np, f, 0.0 * q, b, ep, t)
    assert np.allclose(tn, f) and np.isclose(rail, dem_ref)
    assert np.allclose(cur, A.supply_current(np, f, f, ts, ist, i0))
    met = A.metrics_init(np, np.zeros((2, len(ts))))
    met["limit"][0, 0] = 1.0
    assert met["tau_net"][0, 0] == 0.0 and np.all(met["cc"] == 1.0) and met["rail"].shape == (2,)


def _ref_targets(core, a):
    """env_cpu adim 1-3 (walk, bas hareketi kapali) + PWM kuantalama."""
    idx = core.idx
    tg, _ = T.action_to_targets(np, a.astype(np.float64), idx.default_action_q)
    ctrl = np.tile(idx.init_ctrl, (core.n, 1))
    ctrl[:, idx.action_act_id] = tg
    ctrl = np.clip(ctrl, idx.ctrl_lo, idx.ctrl_hi)
    return A.quantize_pwm(np, ctrl, idx.act["pwm_res"], idx.ctrl_lo, idx.ctrl_hi)


def test_cpu_env_substep_semantics():
    """env_cpu tr35 adimi == elle yazilmis MuJoCo C alt adim dongusu (kart gecikmesi, PWM kuantalama, olu bant, CC
    olceklemesi, forcerange), DR acik; olcutler (talep ort./tepe, cc, tau_net, limit) elle hesapla bit-duzeyinde
    ayni."""
    from rl.env_cpu import VecEnv
    env = VecEnv(3, num_workers=0, robot=TR, randomize=True, noise=True, head_random=False, push=False, seed=5,
                 fixed_command=(0.1, 0.0, 0.0))
    try:
        env.reset()
        core = env._core
        t = core.idx.act
        assert len(set(map(tuple, core.ap["delay"].tolist()))) >= 2              # farkli kart gecikmeleri
        rng = np.random.default_rng(3)
        LT = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
        for step in range(6):
            a = 0.8 * rng.standard_normal((3, env.num_actions))
            refs = [(copy.copy(m), copy.copy(d)) for m, d in zip(core.models, core.datas)]
            prev = core.ctrl_prev.copy()
            new = _ref_targets(core, a.astype(np.float32))
            dems, sc, lim = [], [], np.zeros((3, len(t["tau_s"])))
            for k in range(C.N_SUBSTEPS):
                q = np.stack([d.qpos[t["act_qadr"]] for _, d in refs])
                qd = np.stack([d.qvel[t["act_dadr"]] for _, d in refs])
                gp = np.stack([m.actuator_gainprm[:, 0] for m, _ in refs])
                bp = np.stack([m.actuator_biasprm[:, :3] for m, _ in refs])
                dmp = np.stack([m.dof_damping[t["act_dadr"]] for m, _ in refs])
                c, frc, dem, s = A.shape_substep(np, k, prev, new, q, qd, core.ap, t, gp, bp[..., 0], bp[..., 1],
                                                 bp[..., 2], dmp)
                dems.append(dem)
                sc.append(s)
                for j, (m, d) in enumerate(refs):
                    m.actuator_forcerange[:, 0], m.actuator_forcerange[:, 1] = -frc[j], frc[j]
                    d.ctrl[:] = c[j]
                    mujoco.mj_step(m, d)
                    sel = d.efc_type[:d.nefc] == LT
                    for jid, f in zip(d.efc_id[:d.nefc][sel], d.efc_force[:d.nefc][sel]):
                        i = t["jnt2act"][jid]
                        if i >= 0:
                            lim[j, i] = max(lim[j, i], abs(f))
                af = np.stack([d.actuator_force for _, d in refs])
                tn = A.tau_net(np, af, qd, dmp)
            env.step(a.astype(np.float32))
            for j, (m, d) in enumerate(refs):
                assert np.array_equal(core.datas[j].qpos, d.qpos), (step, j)
                assert np.array_equal(core.models[j].actuator_forcerange, m.actuator_forcerange)
            met = core.met
            assert np.allclose(met["rail"], np.mean(dems, axis=0), rtol=1e-12)
            assert np.array_equal(met["rail_peak"], np.max(dems, axis=0))
            assert np.array_equal(met["cc"], np.min(sc, axis=0))
            assert np.array_equal(met["tau_net"], tn) and np.array_equal(met["limit"], lim)
            assert np.array_equal(core.ctrl_prev, new)
            s = env.task_state()
            assert np.array_equal(s["tau_net"], tn[:, core.idx.obs_act_id])
            assert np.array_equal(s["tau_s"], core.ap["tau_s"][:, core.idx.obs_act_id])
            assert s["rail_demand"].shape == (3,) and s["limit_torque"].shape == (3, 14)
        assert min(np.min(sc), core.met["cc"].min()) < 1.0                         # CC olceklemesi devrede
    finally:
        env.close()


def test_tr35_workers_equal_inprocess():
    """tr35: isci surecleri == tek surec (ayni tohum); bolum/alt adim RNG yerlesimi isci sayisindan bagimsiz;
    tau_ratio = tau_net / tau_s (bolum ornegi)."""
    from rl.env_cpu import VecEnv
    kw = dict(robot=TR, randomize=True, noise=True, seed=7, task="main", bank_n={"seated": 16})
    e0, e2 = VecEnv(4, num_workers=0, **kw), VecEnv(4, num_workers=2, **kw)
    try:
        for x, y in zip(e0.reset(), e2.reset()):
            assert np.array_equal(x, y)
        rng = np.random.default_rng(8)
        for _ in range(6):
            a = 0.5 * rng.standard_normal((4, e0.num_actions)).astype(np.float32)
            for x, y in zip(e0.step(a)[:4], e2.step(a)[:4]):
                assert np.array_equal(x, y)
        s = e0.task_state()
        assert np.allclose(s["tau_ratio"], s["tau_net"] / s["tau_s"])        # geri surulurken |tau_net| > tau_s olur
        assert np.all(np.abs(s["torque"]) <= s["tau_s"] + 1e-9)                # CC sonrasi kuvvet <= tau_s
    finally:
        e0.close()
        e2.close()


def _limit_case(core, name, side):
    """Limit senaryosu: eklem sinirin hemen otesinde, sinira dogru 10 rad/s, servosu zayif (tau_s 1e-3; servo
    ctrlrange'i sinirin 3 derece icinde oldugundan kendisi sinira itemez, guclu servo da carpmayi kendisi durdurur).
    Bosluk eklemleri sinir disinda (surekli sinirda calisirlar). Dondurur (i, qpos, qvel)."""
    m, t = core.models[0], core.idx.act
    i = t["names"].index(name)
    qpos, qvel = core.idx.init_qpos.copy(), np.zeros(m.nv)
    s = 1.0 if side == 1 else -1.0
    qpos[t["act_qadr"][i]] = m.jnt_range[t["act_jnt"][i], side] + s * 0.005
    qvel[t["act_dadr"][i]] = s * 10.0
    ds = np.flatnonzero(t["play_jnt"] >= 0)
    qpos[t["play_qadr"][ds]] = 2.0 * m.jnt_range[t["play_jnt"][ds], 1]
    core.set_state(0, qpos, qvel, core.idx.init_ctrl)
    core.ap["tau_s"][0, i] = 1e-3
    return i, qpos, qvel


def test_limit_metric_cpu():
    """Limit metrigi: ana eklem sinira carpinca > 0 (efc LIMIT_JOINT kuvveti); surekli sinirda calisan bosluk
    eklemleri ve sinirda olmayan eklemler 0; task_state'te gozlem eklemleri sirasinda."""
    from rl.env_cpu import VecEnv
    env = VecEnv(1, num_workers=0, robot=TR, backlash=True, randomize=False, noise=False, head_random=False,
                 push=False, fixed_command=(0.0, 0.0, 0.0), seed=0)
    try:
        env.reset()
        core = env._core
        i, _, _ = _limit_case(core, "head_yaw", 1)
        env.step(np.zeros((1, env.num_actions)))
        lt = core.met["limit"][0]
        assert lt[i] > 1e-3, lt
        assert np.all(np.delete(lt, i) == 0.0), lt
        s = env.task_state()
        assert s["limit_torque"][0, C.OBS_JOINTS.index("head_yaw")] == lt[i]
    finally:
        env.close()


# --- numpy / jax ----------------------------------------------------------------------------------------
def test_jax_parity_actuators():
    if jax is None:
        _skip("jax yok")
    t = _tab("walk", True)
    nu = len(t["tau_s"])
    rng = np.random.default_rng(4)
    u = rng.random((16, A.n_uniform(nu))).astype(np.float32)
    for rnd in (True, False):
        pn = A.sample_params(np, u.astype(np.float64), t, rnd)
        pj = A.sample_params(jnp, jnp.asarray(u), t, rnd)
        for k in pn:
            assert np.allclose(pn[k], np.asarray(pj[k]), rtol=2e-6, atol=1e-7), (rnd, k)
        assert np.array_equal(pn["delay"], np.asarray(pj["delay"]))
    ep = A.sample_params(np, u[:, :].astype(np.float64), t, True)
    q = rng.uniform(-0.5, 0.5, (16, nu))
    qd = rng.uniform(-3, 3, (16, nu))
    prev, new = q + rng.uniform(-0.3, 0.3, (16, nu)), q + rng.uniform(-0.3, 0.3, (16, nu))
    g, z = ep["kp"], np.zeros((16, nu))
    for k in range(C.N_SUBSTEPS):
        a_n = A.shape_substep(np, k, prev, new, q, qd, ep, t, g, z, -g, z, ep["damping"])
        epj = {kk: jnp.asarray(v) for kk, v in ep.items()}
        f = lambda x: jnp.asarray(x, jnp.float32)
        a_j = A.shape_substep(jnp, k, f(prev), f(new), f(q), f(qd), epj, t, f(g), f(z), f(-g), f(z), f(ep["damping"]))
        for x, y in zip(a_n, a_j):
            assert np.allclose(x, np.asarray(y), rtol=1e-4, atol=1e-4), k
    tn_n = A.after_substep(np, new, qd, ep["damping"], ep, t)
    tn_j = A.after_substep(jnp, jnp.asarray(new, jnp.float32), jnp.asarray(qd, jnp.float32),
                           jnp.asarray(ep["damping"], jnp.float32), {kk: jnp.asarray(v) for kk, v in ep.items()}, t)
    for x, y in zip(tn_n, tn_j):
        assert np.allclose(x, np.asarray(y), rtol=1e-4, atol=1e-4)
    x = rng.uniform(-1, 1, (16, nu))
    assert np.allclose(A.quantize_pwm(np, x, t["pwm_res"], t["ctrl_lo"], t["ctrl_hi"]),
                       np.asarray(A.quantize_pwm(jnp, jnp.asarray(x), t["pwm_res"], t["ctrl_lo"], t["ctrl_hi"])),
                       atol=1e-6)
    # vmap'li tek ortam == toplu
    one = jax.vmap(lambda uu: A.sample_params(jnp, uu, t, True))(jnp.asarray(u))
    bat = A.sample_params(jnp, jnp.asarray(u), t, True)
    for k in bat:
        assert np.allclose(np.asarray(one[k]), np.asarray(bat[k]))


def test_mjx_limit_metric():
    """MJX limit metrigi == CPU (ayni sinir disi durumdan; efc LIMIT_JOINT satirlari, bosluk eklemleri disarida)."""
    if jax is None:
        _skip("jax yok")
    from rl.env_cpu import VecEnv
    from rl.env_mjx import OpenDuckEnv
    kw = dict(randomize=False, noise=False, head_random=False, push=False, robot=TR, backlash=True)
    menv = OpenDuckEnv(fixed_command=(0.0, 0.0, 0.0), solver_iterations=32, ls_iterations=16, **kw)
    cenv = VecEnv(1, num_workers=0, fixed_command=(0.0, 0.0, 0.0), **kw)
    try:
        cenv.reset()
        core = cenv._core
        i, qpos, qvel = _limit_case(core, "head_yaw", 0)
        cenv.step(np.zeros((1, cenv.num_actions)))
        st = jax.jit(menv.reset)(jax.random.PRNGKey(0))
        f32 = lambda x: jnp.asarray(x, jnp.float32)
        act = dict(st.info["tr"]["act"], tau_s=st.info["tr"]["act"]["tau_s"].at[i].set(1e-3))
        tr = dict(st.info["tr"], ctrl_prev=f32(core.idx.init_ctrl), act=act)
        st = st.replace(data=st.data.replace(qpos=f32(qpos), qvel=f32(qvel)), info=dict(st.info, tr=tr))
        st2 = jax.jit(menv.step)(st, jnp.zeros(menv.action_size))
        lm = np.asarray(st2.info["tr"]["met"]["limit"])
        lc = core.met["limit"][0]
        assert lc[i] > 1e-3 and np.all(np.delete(lc, i) == 0.0), lc
        assert abs(lm[i] - lc[i]) < 0.05 * lc[i] + 1e-4 and np.all(np.delete(lm, i) < 1e-6), (lm, lc)
    finally:
        cenv.close()


def _main():
    _STANDALONE[0] = True
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    sel = sys.argv[1:]
    failed = 0
    for name, fn in tests:
        if sel and not any(s in name for s in sel):
            continue
        t0 = time.time()
        try:
            fn()
            print("OK   %-42s %6.1fs" % (name, time.time() - t0), flush=True)
        except _Skip as e:
            print("SKIP %-42s (%s)" % (name, e), flush=True)
        except Exception:
            failed += 1
            import traceback
            traceback.print_exc()
            print("FAIL %-42s %6.1fs" % (name, time.time() - t0), flush=True)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    _main()
