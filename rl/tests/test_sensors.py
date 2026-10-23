# -*- coding: utf-8 -*-
"""
rl/sensors.py testleri (OpenDuck-TR spec 3.2.4, A9): pot yolu (RC, ADC1 gecikmesi, ADS1115 mux, kuantalama, biaslar,
akim/ortak-mod hatasi, IIR), IMU modeli ve tamamlayici filtre, gozlem durumu ve tr35 ortam gozlemi.
Calistirma (repo kokunden):

  python3 -m rl.tests.test_sensors
  python3 -m pytest rl/tests/test_sensors.py

jax kuruluysa numpy-jax esitligi de denenir (yoksa o test atlanir).
"""
import sys
import time

import numpy as np

from rl import actuators as A
from rl import config as C
from rl import sensors as S
from rl import task_math as T
from rl.robot_model import RobotIndex, build_model

try:
    import jax
    import jax.numpy as jnp
except ImportError:          # CPU venv'inde jax yok
    jax = jnp = None

TR = "tr35_b2"
DT = C.SIM_DT
N = len(C.OBS_JOINTS)


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


def _idx():
    if "i" not in _IDX:
        _IDX["i"] = RobotIndex(build_model(robot=TR))
    return _IDX["i"]


def _P(lsb=None):
    """Pot tablosu; lsb verilirse kuantalama adimi degistirilir (analitik karsilastirmalar icin)."""
    P = dict(_idx().pot)
    if lsb is not None:
        P["lsb"] = np.full(N, lsb)
    return P


def _ep0(P, batch=()):
    return S.episode(np, np.zeros(batch + (S.n_episode(N),)), P, noise=False)


def _rand_unit(rng, n):
    g = rng.normal(size=(n, 3))
    return g / np.linalg.norm(g, axis=-1, keepdims=True)


# --- tablolar ve yerlesim --------------------------------------------------------------------------
def test_pot_table_and_layout():
    P = _P()
    ads = [j for j, a in zip(C.OBS_JOINTS, P["ads"]) if a > 0.5]
    assert ads == list(S.POT["ads_joints"]) and int(P["ads"].sum()) == 4           # boyun + 3 kafa ADS1115'te
    assert P["ads_ch"][P["ads"] > 0.5].tolist() == [0.0, 1.0, 2.0, 3.0]
    adc = P["ads"] < 0.5
    assert np.allclose(P["lsb"][adc], np.deg2rad(S.POT["adc1"]["lsb_deg"]))
    assert np.allclose(P["lsb"][~adc], np.deg2rad(S.POT["ads"]["lsb_deg"]))
    leg = [j for j, r in zip(C.OBS_JOINTS, P["on_leg"]) if r > 0.5]                 # DS = LEG rayi
    assert len(leg) == 9 and "neck_pitch" in leg and not any("hip_yaw" in j for j in leg)
    assert np.isclose(P["a_rc"], 1.0 - np.exp(-DT / 3.2e-3)) and P["alpha"] == 0.47
    i = C.OBS_JOINTS.index("neck_pitch")
    assert np.isclose(P["lo"][i], np.deg2rad(-25.0 - 90.0))                         # pot merkezi -25, aciklik 180
    assert S.n_episode(N) == 46 and S.n_tick(N) == 20
    ep = _ep0(P)
    assert np.all(ep["bias"] == 0) and np.allclose(ep["R"], np.eye(3)) and np.isclose(ep["ads_tc"], 1.0 / 860.0)
    assert ep["ads_phase"] == 0.0 and ep["vib"] == 0.0
    # gurultu acik: araliklar
    u = np.random.default_rng(0).random((3000, S.n_episode(N)))
    e = S.episode(np, u, P, noise=True)
    b = np.rad2deg(np.abs(e["bias"]))
    assert b.max() <= S.POT["bias_deg"] + 1e-9 and b.max() > 0.97 * S.POT["bias_deg"]
    assert np.rad2deg(np.abs(e["gnd"])).max() <= S.POT["gnd_deg_per_a"] + 1e-9
    assert e["cm_r"].min() >= 0 and e["cm_r"].max() <= S.POT["cm_r_max"]
    assert np.abs(e["cm_eps"]).max() <= S.POT["cm_eps"]
    leg_i, aux_i = np.flatnonzero(P["rail"] < 0.5), np.flatnonzero(P["rail"] > 0.5)
    assert np.all(e["cm_eps"][:, leg_i] == e["cm_eps"][:, leg_i[:1]])              # ray basina tek hata
    assert np.all(e["cm_eps"][:, aux_i] == e["cm_eps"][:, aux_i[:1]])
    tc0, tol = 1.0 / 860.0, S.POT["ads"]["rate_tol"]
    assert e["ads_tc"].min() >= tc0 / (1 + tol) - 1e-12 and e["ads_tc"].max() <= tc0 / (1 - tol) + 1e-12
    assert np.all((e["ads_phase"] >= 0) & (e["ads_phase"] < 4 * e["ads_tc"]))
    R = e["R"]
    assert np.allclose(np.einsum("nij,nkj->nik", R, R), np.eye(3), atol=1e-12) and np.allclose(np.linalg.det(R), 1.0)
    ang = np.rad2deg(np.arccos(np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1, 1)))
    assert ang.max() <= np.sqrt(3) * S.IMU["mount_deg"] + 1e-6
    assert np.abs(e["gb"]).max() <= S.IMU["gyro_bias"] and np.abs(e["gs"]).max() <= S.IMU["gyro_scale"]


# --- birim fonksiyonlar --------------------------------------------------------------------------------
def test_quantize_rc_iir():
    rng = np.random.default_rng(1)
    lo, lsb = -1.3, 0.002
    x = rng.uniform(-1, 1, 5000)
    q = S.quantize(np, x, lo, lsb)
    assert np.abs(q - x).max() <= 0.5 * lsb + 1e-12
    assert np.allclose((q - lo) / lsb, np.round((q - lo) / lsb)) and np.allclose(S.quantize(np, q, lo, lsb), q)
    a = 0.79
    y = 0.0
    for k in range(1, 6):
        y = S.rc_step(np, y, 1.0, a)
        assert np.isclose(y, 1.0 - (1.0 - a) ** k)
    # IIR: sabit hizli rampa -> hiz; darbe yaniti alpha (1-alpha)^k
    w, al = 1.7, 0.47
    qdf, v = 0.0, 0.0
    for k in range(1, 80):
        vn = w * k * DT
        qdf = S.iir_qd(np, qdf, vn, v, al)
        v = vn
    assert abs(qdf - w) < 1e-9
    qdf = S.iir_qd(np, 0.0, 1e-3, 0.0, al)
    assert np.isclose(qdf, al * 1e-3 / DT)
    qdf2 = S.iir_qd(np, qdf, 1e-3, 1e-3, al)
    assert np.isclose(qdf2, (1 - al) * qdf)


def test_pot_latency_and_ads_lag():
    """Gurultusuz, akimsiz, ince kuantalama: rampa q = w t. ADC1: deger_k = y_{k-1} (1 alt adim gecikme; RC durgun
    rampa gecikmesi L = dt (1-a)/a). ADS1115: deger = y(t_k - lag_c), lag_c = mod(faz - (c+1) Tc, 4 Tc) + Tc/2."""
    P = _P(lsb=1e-9)
    ep = _ep0(P)
    w = 0.8
    q0 = np.zeros(N)
    st = S.pot_reset(np, q0, q0 + w, ep, P)
    a = P["a_rc"]
    L = DT * (1 - a) / a
    tc = float(ep["ads_tc"])
    ads = P["ads"] > 0.5
    lags = []
    for k in range(1, 400):
        st_prev = st
        q = q0 + w * k * DT
        st = S.pot_tick(np, st, q, 0.0 * q, np.zeros(()), np.zeros(N), ep, P, noise=False)
        if k > 60:                                   # RC oturdu
            assert np.allclose(st["y1"], w * (k * DT - L), atol=1e-9)
            assert np.allclose(st["v"][~ads], w * ((k - 1) * DT - L), atol=1e-8)          # ADC1: bir alt adim once
            lag_exp = np.mod(st["p"] - (P["ads_ch"] + 1) * tc, 4 * tc) + 0.5 * tc
            assert np.allclose(st["v"][ads], w * (k * DT - L - lag_exp[ads]), atol=1e-8)
            lags.append(lag_exp[ads])
            assert np.allclose(st["qdf"][~ads], w, atol=1e-6)                 # IIR durgun rampada hiz
    lags = np.array(lags)
    assert lags.min() >= 0.5 * tc - 1e-12 and lags.max() <= 4.5 * tc + 1e-12
    assert lags.max() - lags.min() > 3.0 * tc                                  # mux fazi kayiyor (215 vs 200 Hz)
    d = np.mod(lags[:, 1:] - lags[:, :-1], 4 * tc)                             # kanallar ardisik Tc
    assert np.allclose(np.minimum(d, 4 * tc - d), tc, atol=1e-9)


def test_pot_error_terms():
    """Durgun (RC oturmus) okumada: bias + gnd * I - (r I_rail / V [LEG] + eps) (q - lo); kuantalama izgarasi;
    beyaz gurultu standart sapmasi ~ sigma."""
    P = _P(lsb=1e-9)
    rng = np.random.default_rng(2)
    ep = S.episode(np, rng.random(S.n_episode(N)), P, noise=True)
    q = rng.uniform(-0.5, 0.5, N)
    i_sup = rng.uniform(0, 3, N)
    i_rail = np.array(12.0)
    st = S.pot_reset(np, q, 0.0 * q, ep, P)
    for _ in range(60):
        st = S.pot_tick(np, st, q, i_sup, i_rail, np.full(N, 0.5), ep, P, noise=True)   # u = 0.5: gurultu 0
    cm = ep["cm_r"] * i_rail / P["volt"] * P["on_leg"] + ep["cm_eps"]
    exp = q + ep["bias"] + ep["gnd"] * i_sup - cm * (q - P["lo"])
    assert np.allclose(st["v"], exp, atol=1e-8) and np.allclose(st["qdf"], 0.0, atol=1e-6)
    assert np.allclose(S.pot_analog(np, q, i_sup, i_rail, ep, P), exp)
    # isaret: ortak-mod hata potun alt ucundan uzaklastikca buyur, LEG akimi arttikca okuma duser (eps = 0, r > 0)
    ep2 = dict(ep, bias=0 * q, gnd=0 * q, cm_eps=0 * q, cm_r=np.array(0.01))
    e1 = S.pot_analog(np, q, 0 * q, np.array(10.0), ep2, P) - q
    leg = P["on_leg"] > 0.5
    assert np.all(e1[leg] < 0) and np.all(e1[~leg] == 0)
    # beyaz gurultu: gercek LSB ile sapma ~ sigma, sinirli
    P = _P()
    ep0 = _ep0(P)
    st = S.pot_reset(np, q, 0.0 * q, ep0, P)
    for _ in range(30):
        st = S.pot_tick(np, st, q, 0 * q, np.zeros(()), np.full(N, 0.5), ep0, P, noise=True)
    ref = st["v"]
    err = []
    for _ in range(3000):
        st = S.pot_tick(np, st, q, 0 * q, np.zeros(()), rng.random(N), ep0, P, noise=True)
        err.append(st["v"] - ref)
    err = np.array(err)
    assert np.all(np.abs(err) <= P["sigma"] * np.sqrt(3) + P["lsb"] + 1e-12)
    sd = err.std(axis=0)
    assert np.all(np.abs(sd / np.sqrt(P["sigma"] ** 2 + P["lsb"] ** 2 / 12) - 1) < 0.15), sd / P["sigma"]
    q_off = S.pot_tick(np, st, q, 0 * q, np.zeros(()), rng.random(N), ep0, P, noise=False)["v"]
    assert np.allclose(q_off, ref)                                             # noise=False: gurultu yok


def test_imu_and_complementary_filter():
    rng = np.random.default_rng(3)
    P = _P()
    ep = S.episode(np, rng.random(S.n_episode(N)), P, noise=True)
    R = ep["R"]
    g_true = np.array([0.0, 0.0, -1.0])
    acc = np.array([0.0, 0.0, 9.81])
    # gyro modeli (gurultu yok)
    w = rng.normal(size=3)
    st = S.imu_tick(np, {"g": g_true, "w": np.zeros(3)}, w, acc, np.zeros(()), np.full(6, 0.5), ep, noise=True)
    assert np.allclose(st["w"], (1 + ep["gs"]) * (R @ w) + ep["gb"])
    # yakinsama: hareketsiz, yanlis baslangic -> R g (ivme biasi 0)
    ep0 = dict(ep, ab=np.zeros(3), gb=np.zeros(3), gs=np.zeros(3))
    g = np.array([0.4, -0.2, -0.9])
    st = {"g": g / np.linalg.norm(g), "w": np.zeros(3)}
    for _ in range(int(5.0 / DT)):
        st = S.imu_tick(np, st, np.zeros(3), acc, np.zeros(()), np.full(6, 0.5), ep0, noise=True)
    assert np.abs(st["g"] - R @ g_true).max() < 1e-3
    assert np.isclose(np.linalg.norm(st["g"]), 1.0)
    # gyro entegrasyonu: ivme kapisi kapali (|a| = 0 ya da 2 g) -> g sabit dunya yonunu govdede izler
    for a_gate in (np.zeros(3), np.array([0.0, 0.0, 2 * 9.81])):
        wx = 1.5
        g = np.array([0.0, 0.0, -1.0])
        for _ in range(40):
            g = S.complementary(np, g, np.array([wx, 0.0, 0.0]), a_gate)
        th = wx * 40 * DT                                                    # x etrafinda donen govde
        exp = np.array([0.0, -np.sin(th), -np.cos(th)])
        assert np.abs(g - exp).max() < 0.02, (g, exp)
    # kapi: |a| = g0 tam kazanc, 1.3 g0 sifir
    k = lambda an: np.linalg.norm(S.complementary(np, np.array([1.0, 0, 0]), np.zeros(3), np.array([0, 0, an])) -
                                  np.array([1.0, 0, 0]))
    assert k(9.81) > 0 and k(9.81 * (1 + S.IMU["cf_gate"])) < 1e-12 and k(0.0) < 1e-12
    # gurultu genligi
    ws = np.array([S.imu_tick(np, {"g": g_true, "w": np.zeros(3)}, np.zeros(3), acc, np.array(10.0), rng.random(6), ep0,
                              noise=True)["w"] for _ in range(4000)])
    assert np.abs(ws.std(0) / S.IMU["gyro_sigma"] - 1).max() < 0.1
    # sifirlama: R * izdusumlu yercekimi, olculen gyro
    quat = np.array([np.cos(0.2), np.sin(0.2), 0.0, 0.0])
    st0 = S.imu_reset(np, quat, w, ep)
    assert np.allclose(st0["g"], R @ T.projected_gravity(np, quat))
    assert np.allclose(st0["w"], (1 + ep["gs"]) * (R @ w) + ep["gb"])


def test_quat_from_gravity():
    rng = np.random.default_rng(4)
    g = np.concatenate([_rand_unit(rng, 2000), [[0, 0, -1.0], [0, 0, 1.0], [1e-7, 0, 1.0]], [[1.0, 0, 0]]])
    g = g / np.linalg.norm(g, axis=-1, keepdims=True)
    q = S.quat_from_gravity(np, g)
    assert np.allclose(np.linalg.norm(q, axis=-1), 1.0)
    assert np.allclose(T.projected_gravity(np, q), g, atol=1e-6)
    assert np.allclose(q[2000], [1, 0, 0, 0]) and np.allclose(q[2001], [0, 1, 0, 0])


def test_obs_state_and_cpu_env():
    """tr35 CPU ortami: aktor gozlemi = sensor modeli (q_obs/qd_obs/gyro_obs/gravity_obs), kritik = gercek durum;
    gurultusuz sifirlamada q_obs = kuantalanmis gercek aci, qd_obs = gercek hiz; hareketsiz ayakta okumalar 1 LSB
    icinde."""
    from rl.env_cpu import VecEnv
    with VecEnv(2, num_workers=0, robot=TR, randomize=False, noise=False, head_random=False, push=False,
                fixed_command=(0.0, 0.0, 0.0), seed=1) as env:
        obs, crit = env.reset()
        core, spec = env._core, env.spec
        s = env.task_state()
        P = core.idx.pot
        assert np.allclose(s["q_obs"], S.quantize(np, s["q"], P["lo"], P["lsb"]))
        assert np.allclose(s["qd_obs"], s["qd"])
        sl = spec.obs_slices
        assert np.allclose(obs[:, sl["joint_pos"]], s["q_obs"] - s["default_q"], atol=1e-6)
        assert np.allclose(crit[:, sl["joint_pos"]], s["q"] - s["default_q"], atol=1e-6)        # kritik gercek
        assert np.allclose(obs[:, sl["gravity"]], s["gravity_obs"], atol=1e-6)
        assert np.allclose(obs[:, sl["gyro"]], s["gyro_obs"] * C.OBS_SCALES["gyro"], atol=1e-6)
        so = S.obs_state(np, s, s["q_obs"], s["qd_obs"], s["gyro_obs"], s["gravity_obs"])
        assert np.allclose(obs, T.observation(np, so, "walk"), atol=1e-6)
        assert np.allclose(crit[:, :spec.num_obs], T.observation(np, s, "walk"), atol=1e-6)
        for _ in range(100):
            obs, crit, _, done, _ = env.step(np.zeros((2, env.num_actions)))
        assert not done.any()
        s = env.task_state()
        assert np.abs(s["q_obs"] - s["q"]).max() <= P["lsb"].max() + 1e-4
        assert np.abs(s["gravity_obs"] - T.projected_gravity(np, s["quat"])).max() < 2e-3
        assert np.allclose(obs[:, sl["joint_vel"]], s["qd_obs"] * C.OBS_SCALES["joint_vel"], atol=1e-6)
        assert np.allclose(crit[:, sl["joint_vel"]], s["qd"] * C.OBS_SCALES["joint_vel"], atol=1e-6)
        assert np.all(np.isfinite(obs)) and A.TR35 == core.robot


def test_noise_flag_and_rng_layout():
    """noise=True: bolum biaslari ve beyaz gurultu (genel OBS_NOISE kapali); her adim sabit sayida U[0,1) cekilir."""
    from rl.env_cpu import VecEnv
    with VecEnv(2, num_workers=0, robot=TR, randomize=True, noise=True, head_random=False, push=False,
                fixed_command=(0.0, 0.0, 0.0), seed=2) as env:
        env.reset()
        core = env._core
        assert core.noise and not core.noise_obs and core.n_uniform == 0
        assert core.n_us == C.N_SUBSTEPS * S.n_tick(N) and core.n_ue == A.n_uniform(15) + S.n_episode(N)
        assert np.abs(core.sp["bias"]).max() > 0 and not np.allclose(core.sp["R"], np.eye(3))
        s0 = env.task_state()
        env.step(np.zeros((2, env.num_actions)))
        s1 = env.task_state()
        assert np.abs(s1["q_obs"] - s1["q"]).max() > 0.0 and np.isfinite(s1["gyro_obs"]).all()
        assert s0["q_obs"].shape == (2, N)


def test_jax_parity_sensors():
    if jax is None:
        _skip("jax yok")
    rng = np.random.default_rng(5)
    P = _P()
    B = 8
    f = lambda x: jnp.asarray(x, jnp.float32)
    u = rng.random((B, S.n_episode(N)))
    for noise in (True, False):
        en, ej = S.episode(np, u, P, noise), S.episode(jnp, f(u), P, noise)
        for k in en:
            assert np.allclose(en[k], np.asarray(ej[k]), rtol=1e-5, atol=1e-7), (noise, k)
    ep_n = S.episode(np, u, P, True)
    ep_j = {k: f(v) for k, v in ep_n.items()}
    q, qd = rng.uniform(-1, 1, (B, N)), rng.uniform(-2, 2, (B, N))
    sn, sj = S.pot_reset(np, q, qd, ep_n, P), S.pot_reset(jnp, f(q), f(qd), ep_j, P)
    for k in sn:
        assert np.allclose(sn[k], np.asarray(sj[k]), atol=1e-5), k
    sj = {k: f(v) for k, v in sn.items()}
    for it in range(6):
        q = q + 0.01 * rng.normal(size=(B, N))
        isup, irail, un = rng.uniform(0, 3, (B, N)), rng.uniform(0, 20, B), rng.random((B, N))
        sn = S.pot_tick(np, sn, q, isup, irail, un, ep_n, P, noise=True)
        sj = S.pot_tick(jnp, sj, f(q), f(isup), f(irail), f(un), ep_j, P, noise=True)
        for k in sn:
            d = np.abs(sn[k] - np.asarray(sj[k]))
            tol = (P["lsb"] + 1e-5) if k == "v" else (1e-4 if k != "qdf" else 2 * P["alpha"] * P["lsb"] / DT + 1e-3)
            assert np.all(d <= tol), (it, k, d.max())
        sj = {k: f(v) for k, v in sn.items()}                                   # kuantalama esigi birikmesin
    quat = _rand_unit(rng, B)
    quat = np.concatenate([np.cos(0.3) * np.ones((B, 1)), np.sin(0.3) * quat], -1)
    gy, acc = rng.normal(size=(B, 3)), rng.normal(size=(B, 3)) + [0, 0, 9.81]
    imn, imj = S.imu_reset(np, quat, gy, ep_n), S.imu_reset(jnp, f(quat), f(gy), ep_j)
    for k in imn:
        assert np.allclose(imn[k], np.asarray(imj[k]), atol=1e-5)
    imj = {k: f(v) for k, v in imn.items()}
    for it in range(10):
        un, irail = rng.random((B, 6)), rng.uniform(0, 20, B)
        imn = S.imu_tick(np, imn, gy, acc, irail, un, ep_n, noise=True)
        imj = S.imu_tick(jnp, imj, f(gy), f(acc), f(irail), f(un), ep_j, noise=True)
        for k in imn:
            assert np.allclose(imn[k], np.asarray(imj[k]), atol=1e-4), (it, k)
    g = _rand_unit(rng, 64)
    assert np.allclose(S.quat_from_gravity(np, g), np.asarray(S.quat_from_gravity(jnp, f(g))), atol=1e-5)
    # vmap'li tek ortam == toplu
    one = jax.vmap(lambda st, qq, ii, rr, uu, ee: S.pot_tick(jnp, st, qq, ii, rr, uu, ee, P, True))(
        sj, f(q), f(isup), f(irail), f(un[:, :1].repeat(N, 1)), ep_j)
    bat = S.pot_tick(jnp, sj, f(q), f(isup), f(irail), f(un[:, :1].repeat(N, 1)), ep_j, P, True)
    for k in bat:
        assert np.allclose(np.asarray(one[k]), np.asarray(bat[k]), atol=1e-6), k


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
