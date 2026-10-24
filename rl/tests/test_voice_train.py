# -*- coding: utf-8 -*-
"""
Sesli komut katmaninin egitim ve degerlendirme tarafi (spec eki K6, K7): komut atomlari ve oturus oncesi "tut"
(task_math.sample_segment; xp-genel), egitim jest kanali (env_cpu; tr35 main), xl330 degismezligi, CPU == MJX adim adim
(jest kanali ve atomlar acik), A10 duzenegi dumani (eval_tasks) ve play --demo voice.

  python3 -m pytest rl/tests/test_voice_train.py

tr35 parcalari alt surecte (OPENDUCK_ROBOT=tr35_b2, P2-0 stub; rl/tests/tr35_scene.py). MJX parcasi jax yoksa atlanir.
"""
import json
import os
import subprocess
import sys
import tempfile

import numpy as np
import pytest

from rl import config as C
from rl import task_math as T
from rl.tests.tr35_scene import tr35_env, tr35_scene

ROOT = C.ROOT


def _run(code, env, timeout=1800):
    pre = ("import sys, json\nsys.path.insert(0, %r)\nimport rl.config as C\nC.SCENE_PATH = %r\n"
           % (ROOT, env["OPENDUCK_SCENE"]))
    r = subprocess.run([sys.executable, "-c", pre + code], cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout)
    assert r.returncode == 0, "alt surec hatasi:\n%s\n%s" % (r.stdout[-3000:], r.stderr[-6000:])
    return json.loads(r.stdout.strip().splitlines()[-1])


def _has(mod):
    import importlib.util
    return importlib.util.find_spec(mod) is not None


def _env(**kw):
    scene = tr35_scene()
    return tr35_env(scene, None, MUJOCO_GL="disabled", **kw)


# --- xl330: degismez ---------------------------------------------------------------------------------------------------
def test_xl330_unchanged_no_voice_training():
    """xl330 surecinde jest kanali ve atomlar kapali: sample_segment eski turler / komutlar (prev_kind yok sayilir),
    main ortaminda gestures False ve bas/gaga ctrl'u eski formul."""
    if C.ROBOT_PROFILE != "xl330_27":
        pytest.skip("xl330 sureci degil")
    assert C.VOICE == {"gestures": False, "atoms": False} and C.MAIN_SEGMENTS == ("walk", "stand", "crouch", "sit")
    u = np.random.default_rng(0).random((5000, T.NUM_SEGMENT_U))
    prev = np.tile([0.1, 0, 0, 0.1, 0.0], (5000, 1))
    a = T.sample_segment(np, u, prev)
    b = T.sample_segment(np, u, prev, prev_kind=np.full(5000, T.SEG_HOLD))
    assert all(np.array_equal(x, y) for x, y in zip(a, b)) and set(np.unique(a[0])) == {0.0, 1.0, 2.0, 3.0}
    from rl.env_cpu import VecEnv
    e = VecEnv(2, num_workers=0, task="main", seed=1, bank_n=8)
    try:
        assert e._core.gestures is False and e._core.slew_lim is None
        e.reset()
        for _ in range(5):
            e.step(np.zeros((2, 10), np.float32))
        s = e.task_state()
        assert np.all(s["g_kind"] == 0)
    finally:
        e.close()


# --- tr35: atomlar ve tut ----------------------------------------------------------------------------------------
_SEG = r"""
import numpy as np
from rl import task_math as T
R, A = C.COMMAND_RANGES, C.ATOM
rng = np.random.default_rng(3)
n = 200000
u = rng.random((n, T.NUM_SEGMENT_U))
prev = np.tile([0.0, 0.0, 0.0, C.H_HOME, 0.0], (n, 1))
kind, cmd, dur = T.sample_segment(np, u, prev, prev_kind=np.zeros(n))
out = {"voice": C.VOICE, "names": list(C.MAIN_SEGMENTS)}
w = (kind == T.SEG_WALK) | (kind == T.SEG_ATOM)
at = kind == T.SEG_ATOM
out["atom_frac"] = float(at.sum() / w.sum())
av = T.atom_velocities()
out["atom_vel_ok"] = bool(all(np.any(np.all(np.isclose(c, av), axis=1)) for c in cmd[at][:2000, :3]))
out["atom_types"] = [float(np.mean(np.all(np.isclose(cmd[at][:, :3], v), axis=1))) for v in av]
out["atom_dur"] = [float(dur[at].min()), float(dur[at].max())]
out["av"] = av.tolist(), [0.8 * R["vx"][1], 0.8 * R["vx"][0], 0.8 * R["wz"][1]]
hold = kind == T.SEG_HOLD
out["hold_from_sit"] = float(hold.mean())
out["no_direct_sit"] = bool(not np.any(kind == T.SEG_SIT))
out["hold_vel0"] = bool(np.all(cmd[hold, :3] == 0) and np.all(cmd[hold, 4] == 0) and np.all(cmd[hold, 3] == prev[hold, 3]))
out["hold_dur"] = [float(dur[hold].min()), float(dur[hold].max())]
# tut'tan sonra hep sit, h korunur; oturustan baslangicin ilk sit'i tut istemez
h = rng.uniform(C.H_MIN_STAND, C.H_HOME, n)
ph = np.c_[np.zeros((n, 3)), h, np.zeros(n)]
k2, c2, d2 = T.sample_segment(np, u, ph, prev_kind=np.full(n, T.SEG_HOLD))
out["after_hold_sit"] = bool(np.all(k2 == T.SEG_SIT) and np.all(c2[:, 4] == 1) and np.all(c2[:, 3] == h) and np.all(c2[:, :3] == 0))
k3, _, _ = T.sample_segment(np, u, T.reset_command(np, np.ones(n)), seated=np.ones(n))
out["seated_first"] = sorted(set(np.unique(k3).tolist()))
# zincir: sit oncesi hep tut ve oturus baslangicindaki h [H_MIN_STAND, H_HOME]'u kapsar (comel -> tut -> otur)
cm, kd, hs, bad = np.array([[0.0, 0, 0, C.H_HOME, 0]]), np.zeros(1), [], 0
for i in range(30000):
    k, c, _ = T.sample_segment(np, rng.random((1, T.NUM_SEGMENT_U)), cm, prev_kind=kd)
    if k[0] == T.SEG_SIT:
        bad += int(kd[0] != T.SEG_HOLD)
        hs.append(float(c[0, 3]))
    cm, kd = c, k
out["chain_bad"] = bad
out["sit_h"] = [min(hs), max(hs), len(hs)]
try:
    import jax, jax.numpy as jnp
    uj = u[:4096].astype(np.float32)
    pj = np.tile([0.05, 0, 0.1, 0.12, 0.0], (4096, 1)).astype(np.float32)
    pk = (np.arange(4096) % 6).astype(np.float32)
    a = T.sample_segment(jnp, jnp.asarray(uj), jnp.asarray(pj), prev_kind=jnp.asarray(pk))
    b = T.sample_segment(np, uj.astype(np.float64), pj.astype(np.float64), prev_kind=pk.astype(np.float64))
    out["jnp_equal"] = bool(np.array_equal(np.asarray(a[0]), b[0]) and np.allclose(np.asarray(a[1]), b[1], atol=1e-6)
                            and np.allclose(np.asarray(a[2]), b[2], atol=1e-5))
except ImportError:
    out["jnp_equal"] = None
print(json.dumps(out))
"""


def test_tr35_atoms_and_hold_segments():
    """spec eki K6.2 (tr35): walk segmentlerinin %20'si atom {(+0.8 vx_max), (0.8 vx_min), (+-0.8 wz_max)}, esit tur, sure
    U(3.5, 15); her sit'ten once tut (sifir hiz, onceki h, 1-2 s), tut'tan sonra sit; oturustan baslangicta ilk sit tut
    istemez; zincirde oturus baslangic h'si [H_MIN_STAND, H_HOME]'u kapsar; numpy == jax.numpy (varsa)."""
    o = _run(_SEG, _env())
    assert o["voice"] == {"gestures": True, "atoms": True} and o["names"][4:] == ["hold", "atom"]
    assert abs(o["atom_frac"] - 0.2) < 0.01 and o["atom_vel_ok"] and all(abs(x - 0.25) < 0.02 for x in o["atom_types"])
    av, want = o["av"]
    assert np.allclose([av[0][0], av[1][0], av[2][2]], want) and av[3][2] == -av[2][2]
    assert o["atom_dur"][0] >= 3.5 and o["atom_dur"][1] <= 15.0 and o["atom_dur"][1] > 14.0
    assert o["no_direct_sit"] and o["hold_vel0"] and 1.0 <= o["hold_dur"][0] and o["hold_dur"][1] <= 2.0
    assert abs(o["hold_from_sit"] - 0.3) < 0.01 and o["after_hold_sit"] and o["seated_first"] == [1.0, 3.0]
    d = C.load_derived(C.DERIVED_PATHS["tr35_b2"])["heights"]
    lo, hi, n_sit = o["sit_h"]
    assert o["chain_bad"] == 0 and n_sit > 1000
    assert lo < d["H_MIN_STAND"] + 0.002 and hi == d["H_HOME"]
    if o["jnp_equal"] is not None:
        assert o["jnp_equal"]


# --- tr35: CPU ortami jest kanali -----------------------------------------------------------------------------------
_GEST_ENV = r"""
import numpy as np
from rl import task_math as T
from rl.env_cpu import VecEnv
n = 16                      # tut (4) ve atom (5) segmentlerinin ikisi de gorulsun (atom: yurume segmentlerinin %20'si)
e = VecEnv(n, num_workers=0, task="main", seed=3, randomize=True, noise=True, head_random=True, push=True, bank_n=16)
core = e._core
out = {"gestures": core.gestures, "slew": core.slew_lim.tolist()}
e.reset()
K, G, OFF, TGT, HC, AL, SEG, TM, SC = [], [], [], [], [], [], [], [], []
for k in range(1500):
    if k % 40 == 0:
        core.head_timer[:] = np.minimum(core.head_timer, 0.03)
    tm0 = core.head_timer.copy()
    e.step(np.zeros((n, 10), np.float32))
    s = e.task_state()
    K.append(s["g_kind"].copy()); HC.append(s["head_ctrl"].copy()); AL.append(s["alpha"].copy()); SEG.append(s["seg"].copy())
    SC.append(s["step_count"].copy())
    OFF.append(core.head_offset.copy()); TGT.append(core.head_target.copy()); TM.append((tm0, core.head_timer.copy()))
K, HC, AL, SEG, OFF, SC = map(np.array, (K, HC, AL, SEG, OFF, SC))
out["kinds"] = {T.GEST_NAMES[int(k)]: int((K == k).sum()) for k in range(6)}
out["segs"] = sorted(set(np.unique(SEG).tolist()))
seated = AL >= 1.0 - 1e-6
out["head_in_seated"] = int(np.sum(seated & ((K == 1) | (K == 2))))
out["mouth_seated_max_deg"] = float(np.degrees(HC[:, :, 4][seated & (K >= 3)].max())) if np.any(seated & (K >= 3)) else None
out["mouth_max_deg"] = float(np.degrees(HC[:, :, 4].max()))
act = K > 0.5
# jest suresince rastgele ofset 0'a dogru (|ofset| azalir) ve zamanlayici durur
dec = [bool(np.all(np.abs(OFF[k + 1][act[k + 1]]) <= np.abs(OFF[k][act[k + 1]]) + 1e-12)) for k in range(len(K) - 1)]
out["offset_decays"] = bool(all(dec))
# bolum sifirlanan adimlar haric (sure dolumu / dusme: zamanlayici yeniden kurulur; adim sayaci geri gider)
cont = [act[k - 1] & (SC[k] > SC[k - 1]) for k in range(1, len(K))]
frozen = [bool(np.all(TM[k][1][cont[k - 1]] == TM[k][0][cont[k - 1]])) for k in range(1, len(K))]
out["timer_frozen"] = bool(all(frozen))
rate = np.abs(np.diff(HC, axis=0)) / C.CTRL_DT
same = SC[1:] >= 2                                    # ayni bolum icindeki ardisik adimlar (sifirlama haric)
out["rate_max"] = rate[same].max(axis=0).tolist()
out["resets"] = int(np.sum(SC[1:] < 2))
try:
    e.set_gesture(np.zeros((n, 5)))
    out["ext_guard"] = False
except ValueError:
    out["ext_guard"] = True
e.close()
w = VecEnv(2, num_workers=0, task="walk", seed=1, head_random=True)
g = VecEnv(2, num_workers=0, task="getup", seed=1, head_random=True, bank_n=8)
out["walk_getup_gestures"] = [w._core.gestures, g._core.gestures]
w.close(); g.close()
print(json.dumps(out))
"""


def test_tr35_cpu_env_gesture_channel():
    """spec eki K6.1 (tr35 main, CPU): yeniden ornekleme 0.3 jest (4 tur) / 0.35 rastgele / 0.35 sifir; jestler gorulur,
    oturusta (alpha = 1) bas jesti yok, gaga <= 15 derece (ayakta 25); jest suresince rastgele ofset 0'a doner ve
    zamanlayici durur; toplam bas/gaga ctrl hizi <= 0.5 omega0 (ortam assert'i hic tetiklenmez); tut ve atom segmentleri;
    walk ve getup'ta jest kanali yok; dis jest egitim kanaliyla birlikte reddedilir."""
    o = _run(_GEST_ENV, _env(OPENDUCK_BANK_N=16))
    assert o["gestures"] is True and o["walk_getup_gestures"] == [False, False] and o["ext_guard"]
    k = o["kinds"]
    assert all(k[x] > 50 for x in ("nod", "shake", "beak", "click")) and k["click2"] == 0
    assert o["head_in_seated"] == 0 and o["mouth_max_deg"] <= 25.0 + 1e-6 and o["mouth_max_deg"] > 20.0
    if o["mouth_seated_max_deg"] is not None:
        assert o["mouth_seated_max_deg"] <= 15.0 + 1e-6
    assert o["offset_decays"] and o["timer_frozen"]
    assert all(r <= lim + 1e-6 for r, lim in zip(o["rate_max"], o["slew"]))
    assert {4.0, 5.0} <= set(o["segs"])


# --- tr35: CPU == MJX (jest kanali ve atomlar acik) -----------------------------------------------------------------
_LOCK = r"""
import numpy as np, jax, jax.numpy as jnp
from rl import tasks as TK, task_math as T
from rl.env_cpu import VecEnv
from rl.env_mjx import OpenDuckEnv
from rl.tests import test_env_mjx as TM
C.SELF_COLLISION_PAIRS = ()          # kutu-kutu oz-carpisma MJX'te farkli (test_tr35_lockstep_cpu_mjx notu)
n, steps = 4, 70
kw = dict(randomize=True, noise=True, head_random=True, push=False)
task, rp = "main", {"home": 0.5, "seated": 0.5}
spec = TK.get_task(task)
menv = OpenDuckEnv(task=task, reset_probs=rp, solver_iterations=32, ls_iterations=16, **kw)
cenv = VecEnv(n, num_workers=0, task=task, reset_probs=rp, **kw)
core = cenv._core
core.rngs = rngs = [TM._ParityRNG() for _ in range(n)]
reset_u, step_u = TM._parity_draws_tr35(menv, task)
head_u = jax.jit(jax.vmap(lambda r: jax.random.uniform(jax.random.split(r, 6)[1], (6,))))
keys = TM._keys(n, %(seed)d)
for j, arrs in enumerate(zip(*(np.asarray(x) for x in reset_u(keys)))):
    for x in arrs:
        rngs[j].push(x)
st = jax.jit(jax.vmap(menv.reset))(keys)
obs_c, crit_c = cenv.reset()
jstep = jax.jit(jax.vmap(menv.step))
rng = np.random.default_rng(%(seed)d)
P = core.idx.pot
sl_q, sl_v = spec.obs_slices["joint_pos"], spec.obs_slices["joint_vel"]
lsb = float(P["lsb"].max())
tol_q, tol_v = 2e-3 + lsb, 2e-3 + 2.0 * C.OBS_SCALES["joint_vel"] * P["alpha"] * lsb / C.SIM_DT
rec = {"rew": 0.0, "crit": 0.0, "obs_rest": 0.0, "dq": 0.0, "dv": 0.0, "g": 0.0, "ctrl": 0.0, "seg": 0.0,
       "kinds": [], "segs": [], "gest_steps": 0, "mouth": 0.0, "ties": 0, "contact_steps": 0}
# PWM nicemleme (actuators.quantize_pwm, round(ctrl / res)): nicemleme oncesi deger yuvarlama sinirina float32 hassasiyeti
# icinde dustugunde CPU (float64) ve MJX (float32) bir adim farkli yuvarlar. Tam bir adimlik fark "tie" sayilir; o ortamin
# o adimdaki odul / gozlem karsilastirmasi atlanir (durum her adim CPU'dan yeniden esitlenir).
res_pwm = np.asarray(core.idx.act["pwm_res"])
flip = np.zeros(n, bool)
# Temas uyusmazligi: adim sonunda MJX ile MuJoCo C'nin aktif temas sayisi (dist < 0) farkli olan ortam-adimlari. Iki
# motor temas acilip kapanirken (gercek CAD'de oturusta govde kutusu zeminden ~0.3 mm yukarida) farkli cozer; bu
# adimlarin odul / gozlem karsilastirmasi atlanir, sayisi raporlanir. Jest, segment ve ctrl her adim kesin.
active_c = lambda: np.array([int(np.sum(d.contact.dist[:d.ncon] < 0.0)) for d in core.datas])
def active_m(s_):
    c_ = getattr(getattr(s_.data, "_impl", s_.data), "contact")
    return np.sum(np.asarray(c_.dist) < 0.0, axis=1)
f32 = lambda x: jnp.asarray(x, jnp.float32)
for t in range(-1, steps):
    if t >= 0:
        info = dict(st.info)
        if t %% 6 == 0:                      # bas yeniden orneklemesi (iki arka uc ayni anda)
            core.head_timer[:] = np.minimum(core.head_timer, 0.01)
            info["head_timer"] = jnp.minimum(info["head_timer"], 0.01)
        if t %% 14 == 0:                     # segment degisimi (atom / tut / jest kesmesi)
            core.seg_timer[:] = 0.01
            info["seg_timer"] = jnp.full((n,), 0.01, jnp.float32)
        st = st.replace(info=info)
        a = (0.3 * rng.standard_normal((n, spec.num_actions)) * (np.arange(n) %% 2)[:, None]).astype(np.float32)
        draws = [np.asarray(x) for x in step_u(st.info["rng"])] + [np.asarray(head_u(st.info["rng"]))]
        for j in range(n):
            for x in draws:
                rngs[j].push(x[j])
        obs_c, crit_c, rew_c, done_c, _ = cenv.step(a)
        for r_ in rngs:
            r_.q = {}
        st = jstep(st, jnp.asarray(a))
        assert np.array_equal(done_c, np.asarray(st.done) > 0.5), t
        inf = jax.tree_util.tree_map(np.asarray, st.info)
        live = ~done_c
        dd = np.abs(core.ctrl_prev - inf["tr"]["ctrl_prev"])
        tie = (dd > 1e-5) & (np.abs(dd - res_pwm[None, :]) < 1e-6)
        nc1 = active_c()
        trans = live & (np.asarray(active_m(st)) != nc1)
        rec["contact_steps"] += int(trans.sum())
        flip = tie.any(axis=1) | trans
        rec["ties"] += int(tie.sum())
        rec["rew"] = max(rec["rew"], float(np.abs(rew_c - np.asarray(st.reward))[~flip].max(initial=0.0)))
        for k in ("kind", "n", "amp", "cut_n"):
            rec["g"] = max(rec["g"], float(np.abs(core.gst[k][live] - inf["g_" + k][live]).max(initial=0.0)))
        rec["seg"] = max(rec["seg"], float(np.abs(core.seg[live] - inf["seg"][live]).max(initial=0.0)))
        rec["ctrl"] = max(rec["ctrl"], float(np.where(tie, 0.0, dd)[live].max(initial=0.0)))
        rec["kinds"] = sorted(set(rec["kinds"]) | set(core.gst["kind"].tolist()))
        rec["segs"] = sorted(set(rec["segs"]) | set(core.seg.tolist()))
        rec["gest_steps"] += int(np.sum(core.gst["kind"] > 0.5))
        rec["mouth"] = max(rec["mouth"], float(np.abs(core.ctrl_prev[:, core.idx.other_act_id]).max()))
    om, cm = np.asarray(st.obs["state"]), np.asarray(st.obs["privileged_state"])
    k_ = ~flip
    rec["crit"] = max(rec["crit"], float(np.abs(crit_c - cm)[k_].max(initial=0.0)))
    dq = np.abs(obs_c[:, sl_q] - om[:, sl_q])[k_].max(initial=0.0)
    dv = np.abs(obs_c[:, sl_v] - om[:, sl_v])[k_].max(initial=0.0)
    rec["dq"], rec["dv"] = max(rec["dq"], float(dq / tol_q)), max(rec["dv"], float(dv / tol_v))
    rest = np.ones(spec.num_obs, bool)
    rest[sl_q] = rest[sl_v] = False
    rec["obs_rest"] = max(rec["obs_rest"], float(np.abs(obs_c[:, rest] - om[:, rest])[k_].max(initial=0.0)))
    qp, qv, qw = (np.stack([getattr(d, f) for d in core.datas]) for f in ("qpos", "qvel", "qacc_warmstart"))
    tr = dict(st.info["tr"], ctrl_prev=f32(core.ctrl_prev), pot={k: f32(v) for k, v in core.pot_st.items()},
              imu={k: f32(v) for k, v in core.imu_st.items()})
    st = st.replace(data=st.data.replace(qpos=f32(qp), qvel=f32(qv), qacc_warmstart=f32(qw)), info=dict(st.info, tr=tr))
cenv.close()
print(json.dumps(rec))
"""


def test_tr35_lockstep_cpu_mjx_gestures_atoms():
    """tr35 main, jest kanali ve atomlar acik (head_random=True; zorlanmis bas yeniden orneklemesi ve segment
    degisimleri): CPU == MJX adim adim; jest durumu, segment turu, ctrl (bas + gaga dahil), odul, kritik, aktor gozlemi.
    Yeni rastgele sayi yok (mevcut 6 bas ve 8 segment uniform'u)."""
    if not _has("jax"):
        pytest.skip("jax yok (MJX takimi)")
    o = _run(_LOCK % {"seed": 71}, _env(OPENDUCK_BANK_N=32, JAX_PLATFORMS="cpu"), timeout=2400)
    print("lockstep jest/atom", o)
    assert o["g"] < 1e-5 and o["seg"] == 0.0 and o["ctrl"] < 1e-5, o
    assert o["ties"] <= 4, o                  # tek PWM adimi yuvarlama farklari seyrek (280 ortam-adimi x 15 kanal)
    assert o["contact_steps"] <= 28, o        # temas uyusmazligi seyrek (280 ortam-adiminin <= %10'u)
    assert o["rew"] < 1e-3 and o["crit"] < 2e-3 and o["obs_rest"] < 2e-3 and o["dq"] < 1.0 and o["dv"] < 1.0, o
    assert o["gest_steps"] > 20 and len([k for k in o["kinds"] if k > 0]) >= 2 and o["mouth"] >= 0.0
    assert {4.0} <= set(o["segs"]) or {5.0} <= set(o["segs"]), o["segs"]


# --- tr35: A10 duzenegi dumani ve play --demo voice -----------------------------------------------------------------
def test_tr35_a10_smoke():
    """eval_tasks --a10-only (zero politikalar, P2-0 stub, kucuk N): S1, S2, A6 devri (A10 duzeni) ve gyro bias
    duyarliligi kosar; her olcut pass / fail / n/a; S2 hucre tablosu 10 satir x 9 durum; degismez ihlali yok; kopya
    girisimleri kaydedilir."""
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "a10.json")
        code = ("from rl import eval_tasks as E\n"
                "rc = E.main(['--main-policy', 'zero', '--getup-policy', 'zero', '--a10-only', '--n-scale', '0.012',"
                " '--bank-n', '16', '--scene', C.SCENE_PATH, '--no-collision', '--out', %r])\n"
                "print(json.dumps({'rc': rc}))\n" % out)
        rc = _run(code, _env(OPENDUCK_BANK_N=16), timeout=2400)["rc"]
        with open(out) as f:
            r = json.load(f)
    assert rc == 0 and not r["errors"] and r["A10"]["status"] == "ok", r.get("A10", {}).get("traceback")
    a = r["A10"]
    assert r["N"]["A10_S1"] == 3 and r["N"]["A10_S2"] == 3 and a["N"]["handover"] == 2
    assert list(r["pass"]) == ["A10"] and r["pass_all"] is False and r["A4"]["status"] == "not_run"
    for sec in ("S1", "S2", "handover"):
        cs = a[sec]["criteria"]
        assert cs and all(v["status"] in ("pass", "fail", "n/a") for v in _flat(cs)), sec
    s2 = a["S2"]
    assert len(s2["cells"]) == 10 and all(len(v) == 9 for v in s2["cells"].values()) and s2["invariant_violations"] == 0
    assert s2["start_seated"] == 1 and "falls_exposure" in s2 and "w_upper" in s2["transition_falls"]
    assert a["S1"]["invariant_violations"] == 0 and "penetration" in a["S1"] and "turns_with_bias_update" in a["S2"]
    assert a["bias_sensitivity"]["report_only"] and "S1" in a["bias_sensitivity"]
    assert {"ds_ratio_p99", "rail_p99_a", "limit_main_p99_tau_frac"} <= set(a["criteria"]) and a["torque"]["steps"] > 0


def _flat(d, out=None):
    out = [] if out is None else out
    for v in d.values():
        if isinstance(v, dict) and "op" in v and "status" in v:
            out.append(v)
        elif isinstance(v, dict):
            _flat(v, out)
    return out


def test_tr35_play_voice_demo():
    """play --demo voice (zero politikalar): A10 S1 bir kosu, kelime listesi ve rapor alanlari; --voice-seat exact
    (stub tanisi) ile 12 kelimenin hepsi uygulanir."""
    code = ("from rl import play\n"
            "o = play.run(play.parse_args(['--policy', 'zero', '--getup-policy', 'zero', '--demo', 'voice',"
            " '--voice-seat', 'exact', '--scene', C.SCENE_PATH]))\n"
            "print(json.dumps({'words': [w['word'] for w in o['words']], 'ok': [w['ok'] for w in o['words']],"
            " 'ended': o['ended'], 'keys': sorted(o['report'])}))\n")
    o = _run(code, _env(OPENDUCK_BANK_N=16))
    assert o["words"] == ["kalk", "gel", "dur", "saga", "sola", "geri", "egil", "yuksel", "evet", "hayir", "vak", "otur"]
    assert o["ended"] == "done" and o["keys"] == ["completion", "invariant_violations", "metrics", "penetration"]
