# -*- coding: utf-8 -*-
"""
OpenDuck-TR degerlendirme / oynatma / meta / egitici / baslatici testleri (P5a deploy; spec v3.1 bolum 6 A4-A8).

tr35 sahnesi: $OPENDUCK_TR35_SCENE ya da P2-0 yer tutucu (tools/golden.py, freecadcmd; yoksa atlanir). tr35 surecleri
alt surecte: OPENDUCK_ROBOT=tr35_b2, OPENDUCK_SCENE=<sahne> ve cocukta `import rl.config as C; C.SCENE_PATH = <sahne>`
(cekirdek birlesmeden once de calisir). xl330 surecinde robot="tr35_b2" ile ornek duzeyinde tr35 modeli (P4 anlami).

  python3 -m pytest rl/tests/test_eval_tr35.py
"""
import contextlib
import importlib.util
import io
import json
import os
import subprocess
import sys
import tempfile
import types

import numpy as np
import pytest

from rl import config as C
from rl import eval_tasks as E
from rl import policy_meta as PM
from rl import tasks as TK

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SCENE = {}


def tr35_scene():
    """$OPENDUCK_TR35_SCENE if set; otherwise build the P2-0 stub once per process into a temp dir with tools/golden.py
    (freecadcmd, no check); no freecadcmd -> pytest.skip."""
    if "path" in _SCENE:
        return _SCENE["path"]
    p = os.environ.get("OPENDUCK_TR35_SCENE")
    if not p:
        spec = importlib.util.spec_from_file_location("openduck_golden", os.path.join(ROOT, "tools", "golden.py"))
        golden = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(golden)
        if golden.find_freecadcmd() is None:
            pytest.skip("freecadcmd yok: tr35 yer tutucu sahnesi kurulamiyor")
        out = tempfile.mkdtemp(prefix="tr35_stub_")
        golden.build(out, config="tr35_b2", check=False)
        p = os.path.join(out, "sim", "scene.xml")
    _SCENE["path"] = p
    return p


def run_tr35(code, timeout=1200, extra_env=None):
    """code'u tr35 alt surecinde calistirir (kok dizinde); stdout dondurur."""
    scene = tr35_scene()
    env = dict(os.environ, OPENDUCK_ROBOT="tr35_b2", OPENDUCK_SCENE=scene, PYTHONDONTWRITEBYTECODE="1",
               MUJOCO_GL="disabled", **(extra_env or {}))
    pre = "import sys\nsys.path.insert(0, %r)\nimport rl.config as C\nC.SCENE_PATH = %r\n" % (ROOT, scene)
    r = subprocess.run([sys.executable, "-c", pre + code], cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout)
    assert r.returncode == 0, "cocuk surec %d\n--- stdout\n%s\n--- stderr\n%s" % (
        r.returncode, r.stdout[-4000:], r.stderr[-6000:])
    return r.stdout


def brax_params(n_obs, n_act, hidden=(256, 128, 64), seed=0):
    """rl/train_mjx.actor_arrays yapisi (jax'siz, numpy)."""
    rng = np.random.default_rng(seed)
    f = lambda *s: rng.standard_normal(s).astype(np.float32)
    nc = n_obs + 9
    norm = types.SimpleNamespace(mean={"state": f(n_obs) * 0.5, "privileged_state": np.zeros(nc, np.float32)},
                                 std={"state": np.exp(f(n_obs) * 0.5) * np.float32(0.3),
                                      "privileged_state": np.ones(nc, np.float32)})
    sizes = [n_obs] + list(hidden)
    k = lambda a, b: (f(a, b) / np.float32(np.sqrt(a))).astype(np.float32)
    mlp = {"hidden_%d" % i: {"kernel": k(a, b), "bias": f(b) * np.float32(0.1)}
           for i, (a, b) in enumerate(zip(sizes[:-1], sizes[1:]))}
    return (norm, {"params": {"MLP_0": mlp, "Dense_0": {"kernel": k(sizes[-1], n_act), "bias": f(n_act) * np.float32(0.1)}}},
            None)


def _criteria(d, out=None):
    """Ic ice sozluklerdeki butun olcutler (op + status anahtarli)."""
    out = [] if out is None else out
    for v in d.values():
        if isinstance(v, dict) and "op" in v and "status" in v:
            out.append(v)
        elif isinstance(v, dict):
            _criteria(v, out)
    return out


# --- kural yardimcilari ------------------------------------------------------------------------------------
def test_rule_helpers():
    """nokta = k/n, W-alt / W-ust Wilson %95 (spec 6 dili); olcut islecleri ve ic ice gecme karari."""
    assert E.point(90, 100) == 0.9 and E.point(0, 0) is None
    assert abs(E.w_lower(90, 100) - 0.8256) < 1e-4 and abs(E.w_upper(90, 100) - 0.9448) < 1e-4
    z2 = E.Z95 ** 2
    assert abs(E.w_upper(0, 512) - z2 / (512 + z2)) < 1e-12          # 0 dusme: ust sinir z^2 / (n + z^2)
    # A4 dusme W-ust <= %3 (N 512): 0 ve 5 dusme gecer, 12 gecmez
    for k, ok in ((0, True), (5, True), (12, False)):
        r = E.rate_rule(k, 512, w_upper_max=0.03)
        assert r["criteria"]["w_upper"]["pass"] is ok and r["w_upper"] == E.wilson(k, 512)[1], (k, r)
    # A5 nokta >= %95 ve W-alt >= %90 (N 256)
    r = E.rate_rule(250, 256, 0.95, 0.90)
    assert E.all_pass(r["criteria"]) and r["point"] == 250 / 256
    r = E.rate_rule(243, 256, 0.95, 0.90)
    assert r["criteria"]["point"]["pass"] is False and r["criteria"]["w_lower"]["pass"] is True
    # A6 yuz: taban %80, W-alt >= %72 (N 128)
    assert E.all_pass(E.rate_rule(110, 128, 0.80, 0.72)["criteria"])
    assert not E.all_pass(E.rate_rule(102, 128, 0.80, 0.72)["criteria"])
    assert E.crit(0.85, 0.85)["pass"] and not E.crit(0.8501, 0.85)["pass"] and E.crit(1.99, 2.0, "<")["pass"]
    assert E.crit(1.1, [0.8, 1.2], "in")["pass"] and not E.crit(-0.1, [0.8, 1.2], "in")["pass"]
    assert E.crit([256, 128, 64], [256, 128, 64], "==")["pass"] and E.crit(None, 1.0)["status"] == "n/a"
    assert E.crit(1.0, None)["pass"] is None
    assert E.all_pass({"a": E.crit(1, 2), "b": {"c": E.crit(3, 4)}}) and not E.all_pass({"a": E.crit(None, 1)})
    assert not E.all_pass({})


def _fake_idx():
    from rl import actuators as ACT
    names = list(C.OBS_JOINTS) + ["mouth"]
    cls = [ACT.joint_class("tr35_b2", n) for n in names]
    act = {"tau_s": np.array([ACT.CLASSES[c]["tau_s"] for c in cls]), "cls": cls}
    return types.SimpleNamespace(act=act, obs_act_id=np.arange(14), robot="tr35_b2")


def test_torque_acc_and_criteria():
    """TorqueAcc: NOMINAL tau_s ile oranlar, RMS, limit ve ray yuzdelikleri; olcut esikleri ve en kotu eklem."""
    idx = _fake_idx()
    acc = E.TorqueAcc(idx)
    tau = acc.tau
    j = {n: i for i, n in enumerate(C.OBS_JOINTS)}
    n = 1000
    tn = np.zeros((n, 14))
    tn[:, j["left_knee"]] = 1.2                        # diz RMS 1.2 > 0.96 (T_diz yer tutucu)
    tn[:, j["right_ankle"]] = np.linspace(0, 0.9, n) * tau[j["right_ankle"]]
    tn[:, j["left_hip_yaw"]] = 0.05
    lim = np.zeros((n, 14))
    lim[-5:, j["left_hip_pitch"]] = 0.5               # p99 disinda kalan 5 tepe
    lim[:, j["head_yaw"]] = 0.1
    rail = np.full(n, 3.0)
    rail[:15] = 15.5                                  # %1.5 > 15 A
    live = np.ones(n, bool)
    live[0] = False                                   # canli olmayan satir sayilmaz
    acc.add({"tau_net": tn, "limit_torque": lim, "rail_demand": rail}, live)
    s = acc.summary()
    assert s["steps"] == n - 1 and s["basis"] == "nominal tau_s"
    assert abs(s["joints"]["left_knee"]["rms_nm"] - 1.2) < 1e-6
    assert abs(s["joints"]["right_ankle"]["ratio_p99"] - np.percentile(np.linspace(0, 0.9, n)[1:], 99)) < 1e-6
    cr = E.torque_criteria(s, 0.96)
    assert cr["knee_rms_nm"]["pass"] is False and cr["knee_rms_nm"]["joint"] == "left_knee"
    assert cr["ds_ratio_p99"]["pass"] is False and cr["ds_ratio_p99"]["joint"] == "right_ankle"
    assert cr["hip_yaw_rms_nm"]["pass"] and abs(cr["es3054_rms_nm"]["limit"] - tau[j["left_hip_yaw"]] / 3) < 1e-12
    assert cr["limit_main_p99_tau_frac"]["pass"] and cr["limit_head_p99_nm"]["pass"]
    assert cr["limit_head_p99_nm"]["joint"] == "head_yaw" and abs(cr["limit_head_p99_nm"]["value"] - 0.1) < 1e-6
    assert cr["rail_gt15_frac"]["pass"] and abs(cr["rail_gt15_frac"]["value"] - 14 / 999) < 1e-9
    assert cr["rail_p99_a"]["pass"]
    assert set(E.torque_criteria(s, 0.96, parts=("head",))) == {"head_ratio_p95", "head_sat_frac"}
    assert E.torque_criteria({"steps": 0}, 0.96)["knee_rms_nm"]["status"] == "n/a"
    with pytest.raises(ValueError):
        E.TorqueAcc(types.SimpleNamespace(act=None, robot="xl330_27"))
    assert "penalties" not in s                         # tau_ratio / tau_s verilmedi


def test_torque_acc_penalties():
    """TorqueAcc tr35 egitim cezalari (P5a entegrasyonu): task_math.tr35_penalties adim ortalamalari (bolum tau_s'i)
    ve torque_sat_steps orani (max |tau_ratio| > GETUP torque_sat_frac); ray esigi = C.RAIL_SOFT_A."""
    from rl import task_math as T
    acc = E.TorqueAcc(_fake_idx())
    rng = np.random.default_rng(4)
    n = 400
    ts = np.tile(acc.tau * rng.uniform(0.8, 1.2, 14), (n, 1))
    tr = rng.uniform(-1.1, 1.1, (n, 14))
    s = {"tau_net": tr * ts, "tau_ratio": tr, "tau_s": ts, "limit_torque": np.abs(rng.normal(0, 0.05, (n, 14))),
         "rail_demand": rng.uniform(0.0, 20.0, n)}
    live = rng.random(n) > 0.2
    acc.add(s, live)
    p = acc.summary()["penalties"]
    ref = T.tr35_penalties(np, {k: v[live] for k, v in s.items() if k != "tau_net"})
    for k in ("torque_ratio", "limit_torque", "rail_demand"):
        assert abs(p[k] - ref[k].mean()) < 1e-9 * max(1.0, abs(p[k])), k
    sat = np.mean(np.abs(tr[live]).max(-1) > C.GETUP["torque_sat_frac"])
    assert abs(p["torque_sat_steps_frac"] - sat) < 1e-12 and p["rail_soft_a"] == C.RAIL_SOFT_A
    assert E.CRIT_TR35["rail_hi_a"] == C.RAIL_SOFT_A == 15.0


def test_stratified_counts_exact():
    """Mod basina tabakali secim: tam N, etiket dogru; side = left + right (n // 2 + kalan, n // 2); az girdide basa
    sararak tekrar; etiket yoksa ValueError."""
    from rl.pose_bank import LABELS
    lab = np.array([LABELS.index(x) for x in ("back", "front", "left", "front", "right", "upright", "front", "back")])
    for mode, n in (("front", 7), ("side", 7), ("back", 3), ("side", 1)):
        ids = E.Evaluator.stratified(lab, mode, n)
        assert len(ids) == n, (mode, ids)
        names = [LABELS[lab[i]] for i in ids]
        want = dict(E.A6_MODES)[mode]
        assert set(names) <= set(want)
        if mode == "side":
            assert names.count("left") == n // 2 + n % 2 and names.count("right") == n // 2
    assert list(E.Evaluator.stratified(lab, "front", 5)) == [1, 3, 6, 1, 3]
    with pytest.raises(ValueError):
        E.Evaluator.stratified(np.array([0, 0, 1]), "side", 4)


def test_criteria_mode():
    class P(object):
        def __init__(self, robot):
            self.meta = {"robot": robot} if robot else None
    assert E.criteria_mode("tr35", {"main": None}) == "tr35_b2"
    assert E.criteria_mode("xl330", {"main": P("tr35_b2")}) == "xl330_27"
    want = "tr35_b2" if C.ROBOT_PROFILE == "tr35_b2" else "xl330_27"
    assert E.criteria_mode("auto", {"main": P(None), "walk": None}) == want
    assert E.criteria_mode("auto", {"main": P(None), "getup": P("tr35_b2")}) == "tr35_b2"


# --- policy_meta -----------------------------------------------------------------------------------------
def test_tr35_meta_on_stub_and_xl330_unchanged():
    """tr35 modeli (robot="tr35_b2", yer tutucu sahne): aktuator, sensor, gecikme, ag, kappa, derived_source,
    getup_head, seat_pitch_deg alanlari, JSON'a yazilabilir; xl330 meta'si network / getup_head'den etkilenmez."""
    from rl import actuators as ACT
    from rl import sensors as SENS
    from rl.robot_model import RobotIndex, build_model
    scene = tr35_scene()
    net = {"actor_hidden": [256, 128, 64], "critic_hidden": [512, 256, 128], "activation": "swish",
           "obs_normalization": True}
    for task in TK.TASK_NAMES:
        t = TK.get_task(task)
        idx = RobotIndex(build_model(scene, variant=t.model_variant, robot="tr35_b2"), list(t.action_joints))
        m = PM.build_meta(9, "mjx_brax", idx, task, network=net)
        assert json.loads(json.dumps(m)) == m
        for k in ("robot", "actuators", "sensors", "delays", "derived_source", "kappa", "network"):
            assert k in m, (task, k)
        a = m["actuators"]
        assert m["robot"] == "tr35_b2" and set(C.OBS_JOINTS) <= set(a["names"]) and a["names"] == idx.act["names"]
        assert all(len(a[c]) == idx.act["tau_s"].shape[0] == 15 for c in PM.ACT_COLUMNS), task
        assert a["cls"][a["names"].index("left_knee")] == "ds32xx" and a["boards"]["1"]["pwm_hz"] == 50
        assert a["i_cc"] == ACT.I_CC and set(a["classes"]) == set(ACT.CLASSES) and "rails" in a
        assert abs(a["kp"][0] - a["tau_s"][0] / a["pband"][0]) < 1e-9
        pt = m["sensors"]["pot_table"]
        assert pt["joints"] == list(C.OBS_JOINTS) and len(pt["lo"]) == 14 and m["sensors"]["imu"] == SENS.IMU
        assert m["sensors"]["pot"]["ads_joints"] == list(SENS.POT["ads_joints"])
        assert m["delays"]["delay_substeps"] == [list(x) for x in ACT.DR["delay_substeps"]]
        assert m["delays"]["substep_dt"] == C.SIM_DT and m["network"] == net
        assert m["kappa"] == getattr(C, "KAPPA", None) and m["derived_source"] == getattr(C, "DERIVED_SOURCE", None)
        assert ("getup_head" in m) == (task == "getup")
        if task == "getup":
            assert m["getup_head"] == getattr(C, "GETUP_HEAD_DEFAULT", "free") and "head_tuck_deg" not in m
        if task == "main":
            assert m["posture_ref"]["seat_pitch_deg"] == float(getattr(C, "SEAT_PITCH_DEG", 0.0))
        if task == "walk":
            assert "task" not in m and m["obs_dim"] == 49
    g = PM.build_meta(1, "cpu_torch", task="getup", scene_path=scene, robot="tr35_b2", getup_head="fixed")
    assert g["getup_head"] == "fixed" and "head_tuck_deg" in g and g["robot"] == "tr35_b2"
    # xl330: ek argumanlar yok sayilir, alanlar ve sira bugunku
    for task in TK.TASK_NAMES:
        base = PM.build_meta(3, "cpu_torch", task=task)
        assert PM.build_meta(3, "cpu_torch", task=task, network=net, getup_head="fixed") == base
        assert "robot" not in base and "actuators" not in base and "network" not in base
        assert json.dumps(PM.build_meta(3, "cpu_torch", task=task, network=net)) == json.dumps(base)


# --- tr35 degerlendirme dumani ----------------------------------------------------------------------------
def test_eval_tr35_zero_policy_smoke():
    """Alt surecte (tr35_b2, yer tutucu): zero politikalar, --n-scale 0.01, --bank-n 32, A7 icin brax ONNX; her
    bolum (A4-A8) "ok" durumla ve her olcut pass / fail / n/a ile; mod basina tabakali sayilar tam; devir calisir."""
    with tempfile.TemporaryDirectory() as d:
        onnx_path, out = os.path.join(d, "getup_x.onnx"), os.path.join(d, "eval.json")
        import onnx
        from rl.train_mjx import actor_to_onnx
        onnx.save(actor_to_onnx(brax_params(48, 14)), onnx_path)
        code = ("from rl import eval_tasks as E\n"
                "rc = E.main(['--main-policy', 'zero', '--walk-policy', 'zero', '--getup-policy', 'zero',"
                " '--n-scale', '0.01', '--bank-n', '32', '--scene', C.SCENE_PATH, '--out', %r, '--a7-onnx', %r])\n"
                "sys.exit(rc)\n" % (out, onnx_path))
        run_tr35(code)
        with open(out) as f:
            r = json.load(f)
    assert r["criteria_mode"] == "tr35_b2" and r["robot"] == "tr35_b2" and r["backlash"] is True and not r["errors"]
    assert r["N"] == {"A4": 5, "A5": 3, "A6_per_mode": 2, "handover": 1}
    for k in ("A4", "A5", "A6", "A7", "A8"):
        assert r[k]["status"] == "ok", (k, r[k].get("reason"), r[k].get("traceback"))
    assert list(r["pass"]) == ["A4", "A5", "A6", "A7", "A8"] and r["pass_all"] is False
    for k in ("A4", "A5", "A6", "A7"):
        cs = _criteria(r[k])
        assert cs and all(c["status"] in ("pass", "fail", "n/a") for c in cs), k
    a4 = r["A4"]
    for t in ("walk", "main"):
        w = a4[t]
        assert w["episodes"] == 5 and sum(w["groups"].values()) == 5 and w["torque"]["steps"] > 0
        assert set(w["criteria"]) >= {"falls_w_upper", "vx_fwd_frac", "wz_pos_frac", "ds_ratio_p99", "knee_rms_nm",
                                      "es3054_rms_nm", "mg90s_ratio_p99", "hip_yaw_rms_nm", "limit_main_p99_tau_frac",
                                      "limit_head_p99_nm", "rail_p99_a", "rail_gt15_frac"}
        assert w["criteria"]["knee_rms_nm"]["limit"] == r["t_knee_nm"]
    assert "main_tracking_ratio" in a4["criteria"]
    a5 = r["A5"]
    assert a5["from_home"]["episodes"] == 3 and a5["from_seated"]["episodes"] == 3
    assert {"sit_from_home", "stand_from_home", "stand_from_seated", "transition_falls_w_upper",
            "height_rmse_standing", "height_rmse_walking", "knee_rms_nm", "rail_p99_a"} <= set(a5["criteria"])
    a6 = r["A6"]
    assert [m for m in a6["modes"]] == ["front", "side", "back"]
    assert a6["modes"]["front"]["labels"] == {"front": 2} and a6["modes"]["back"]["labels"] == {"back": 2}
    assert a6["modes"]["side"]["labels"] == {"left": 1, "right": 1}
    assert all(m["episodes"] == 2 and m["rate"]["n"] == 2 for m in a6["modes"].values())
    assert {"head_ratio_p95", "head_sat_frac", "limit_main_p99_tau_frac", "rail_p99_a"} <= set(a6["criteria"])
    assert a6["head_force_peak_median_n"] is not None
    # P5a entegrasyonu: cekirdegin tr35 cezalari (TorqueAcc) ve getup bolum metrikleri (ortam)
    for sec in (a4["walk"], a4["main"], a5, a6):
        pen = sec["torque"]["penalties"]
        assert all(pen[k] is not None and pen[k] >= 0 for k in ("torque_ratio", "limit_torque", "rail_demand"))
    for m in a6["modes"].values():
        em = m["episode_metrics"]
        assert em["episodes"] == 2 and em["torque_sat_steps_frac"] is not None and "head_impacts" in em
    assert r["derived_source"]["mjcf_sha256"] and r["kappa"] == 0.5
    ho = a6["handover"]
    assert ho["episodes"] == 1 and ho["successes"] + sum(ho["failures"].values()) == 1 and "point" in ho["criteria"]
    a7 = r["A7"]["files"]["extra_0"]
    assert a7["status"] == "ok" and a7["check"]["equal"] and a7["check"]["n"] == 1000 and a7["sizes"][0] == 48
    assert r["A7"]["criteria"]["extra_0"]["actor_hidden"]["pass"] is True
    g = r["A8"]["groups"]
    assert {"A4_walk", "A4_main", "A5_standing", "A5_walking", "A5_home", "A5_seated", "A6_front", "A6_side",
            "A6_back"} <= set(g) and all(x["in_verdict"] for x in g.values())


def test_tr35_handover_scripted():
    """Devir mantigi (xl330 modelinde, betikli supervizor): devirme 1 s'de; dusme -> getup -> main -> 2 s ayakta =
    basari; donusten sonra 2 s icinde dusme = refall; TIP_MAX devirmede dusmeyen = no_fall."""
    from rl import play
    plan = {0: {"getup": 60, "main": 100}, 1: {"getup": 60, "main": 100, "getup2": 150}, 2: {}}

    class Scripted(object):
        n = 0

        def __init__(self, *a, **k):
            self.i = Scripted.n
            Scripted.n += 1
            self.k = 0
            self.mode = "main"

        def update(self, s):
            self.k += 1
            p = plan[self.i]
            if self.k in (p.get("getup"), p.get("getup2")):
                self.mode = "getup"
                return "getup"
            if self.k == p.get("main"):
                self.mode = "main"
                return "main"
            return None

    zero_m, zero_g = play.load_policy("zero", task="main"), play.load_policy("zero", task="getup")
    ev = E.Evaluator(seed=3, randomize=False, noise=False, push=False, head_random=False, bank_n=16,
                     n_scale=3.0 / E.N_TR35["handover"])
    assert ev.n_ho == 3
    orig = play.Supervisor
    play.Supervisor = Scripted
    try:
        r = ev.tr35_handover(zero_m, zero_g)
    finally:
        play.Supervisor = orig
    assert r["episodes"] == 3 and r["successes"] == 1, r
    assert r["failures"] == {"refall": 1, "no_fall": 1}, r["failures"]
    assert abs(r["t_fall_to_main"]["max"] - 40 * C.CTRL_DT) < 1e-9 and r["tips_used"]["max"] == play.TIP_MAX
    assert r["criteria"]["point"]["value"] == 1 / 3 and r["pass"] is False


# --- play (tr35) -----------------------------------------------------------------------------------------
def test_play_tr35_demos_smoke():
    """tr35 surecinde play: sit demosu (komutlar COMMAND_RANGES kesirleri, yukseklikler C.H_*), eklem basina
    nominal tau_s doygunlugu; getup demosu (devirme TIP_W = 10 / sqrt(1.282)), getup_head meta'dan."""
    with tempfile.TemporaryDirectory() as d:
        code = ("import json, numpy as np, onnx\n"
                "from rl import play, policy_meta as PM\n"
                "from rl.train_mjx import actor_to_onnx\n"
                "sys.path.insert(0, %r)\n"
                "from test_eval_tr35 import brax_params\n"
                "g = %r\n"
                "onnx.save(actor_to_onnx(brax_params(48, 14)), g)\n"
                "json.dump(PM.build_meta(1, 'mjx_brax', task='getup'), open(g[:-5] + '_meta.json', 'w'))\n"
                "a = play.run(play.parse_args(['--policy', 'zero', '--demo', 'sit', '--scene', C.SCENE_PATH]))\n"
                "b = play.run(play.parse_args(['--policy', 'zero', '--getup-policy', g, '--demo', 'getup',"
                " '--seconds', '6', '--tip', 'left', '--scene', C.SCENE_PATH]))\n"
                "R = C.COMMAND_RANGES\n"
                "print('JSON' + json.dumps({'a': a, 'b': b, 'tip_w': play.TIP_W, 'vx': R['vx'], 'wz': R['wz'],"
                " 'vy': R['vy'], 'h': [C.H_HOME, C.H_MIN_WALK, C.H_MIN_STAND],"
                " 'demo': [list(x[1]) for x in play.DEMO]}))\n" % (os.path.dirname(os.path.abspath(__file__)),
                                                                     os.path.join(d, "getup.onnx")))
        out = run_tr35(code)
    r = json.loads(out[out.index("JSON") + 4:].strip().splitlines()[0])
    a, b = r["a"], r["b"]
    assert abs(r["tip_w"] - 10.0 / np.sqrt(1.282)) < 1e-12
    vx, wz, vy = r["vx"], r["wz"], r["vy"]
    assert r["demo"] == [[0, 0, 0], [0.5 * vx[1], 0, 0], [0, 0, 0.5 * wz[1]], [0.5 * vx[0], 0, 0], [0, 0.5 * vy[1], 0]]
    h_home, h_walk, h_stand = r["h"]
    seg = {s["name"]: s for s in a["segments"]}
    assert a["task"] == "main" and a["demo"] == "sit" and a["robot"] == "tr35_b2"
    assert seg["walk forward"]["command"] == [0.5 * vx[1], 0.0, 0.0, h_home, 0.0]
    assert abs(seg["crouch walk"]["command"][3] - 0.5 * (h_home + h_walk)) < 1e-12
    assert abs(seg["crouch"]["command"][3] - 0.5 * (h_stand + h_walk)) < 1e-12 and seg["sit"]["command"][4] == 1.0
    for s in a["segments"]:
        assert len(s["peak_tau_joints"]) == 14 and s["peak_tau"] == max(s["peak_tau_joints"])
        assert s["peak_tau_basis"] == "|tau_net| / nominal tau_s"
    assert b["getup_head"] == "free" and b["supervisor"]["tips"] >= 1 and b["robot"] == "tr35_b2"


# --- egiticiler ------------------------------------------------------------------------------------------
def test_trainer_args_and_xl330_outputs_unchanged():
    """--getup-head: verilmezse args'ta yok (config.json / run_info bugunku); yalniz getup'ta ve "free" degilse ortama
    gecer. xl330 CPU egitimi c/ yazmaz, dosya kumesi bugunku."""
    from rl import train_mjx as TM
    a = TM.parse_args(["--out", "x"])
    assert "getup_head" not in vars(a) and TM.parse_args(["--out", "x", "--getup-head", "fixed"]).getup_head == "fixed"
    assert TM.env_task_kwargs("getup", "fixed") == {"getup_head": "fixed"}
    assert TM.env_task_kwargs("getup", "free") == {} and TM.env_task_kwargs("walk", "fixed") == {}
    n = TM.network_info(TM.parse_args(["--out", "x", "--task", "getup"]))
    assert n["activation"] == "swish" and n["actor_hidden"] == list(TK.get_task("getup").actor_hidden or (512, 256, 128))
    pytest.importorskip("torch")
    from rl import train as TR
    a = TR.parse_args(["--out", "x"])
    assert "getup_head" not in vars(a) and TR.env_task_kwargs("getup", "fixed") == {"getup_head": "fixed"}
    if C.ROBOT_PROFILE != "xl330_27":
        return
    import signal
    handlers = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    with tempfile.TemporaryDirectory() as d:
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                TR.main(["--out", d, "--num-envs", "4", "--workers", "0", "--max-steps", "1"])
        finally:
            for s, h in handlers.items():
                signal.signal(s, h)
        assert sorted(os.listdir(d)) == ["config.json", "latest.pt", "log.jsonl", "model_1.pt", "policy.onnx",
                                         "policy_meta.json"]
        with open(os.path.join(d, "policy_meta.json")) as f:
            assert json.load(f) == PM.build_meta(96, "cpu_torch")
        with open(os.path.join(d, "config.json")) as f:
            assert "getup_head" not in json.load(f)["args"]


def test_trainers_write_c_for_tr35():
    """tr35 alt sureci: train_mjx.export_all (brax params) ve rl.train (CPU, 1 iterasyon) <run>/c/ yazar; meta'da
    network ve aktuator alanlari; xl330 surecinde export_all c/ yazmaz."""
    from rl import train_mjx as TM
    with tempfile.TemporaryDirectory() as d:
        TM.export_all(brax_params(49, 10), d, 5, None, None, "walk")
        assert sorted(os.listdir(d)) == ["policy.onnx", "policy_meta.json"]          # xl330 (bu surec)
    torch_ok = importlib.util.find_spec("torch") is not None
    with tempfile.TemporaryDirectory() as d:
        code = ("import json, os\n"
                "sys.path.insert(0, %r)\n"
                "from test_eval_tr35 import brax_params\n"
                "from rl import train_mjx as TM\n"
                "from rl.robot_model import RobotIndex, build_model\n"
                "d = %r\n"
                "idx = RobotIndex(build_model(variant='walk'))\n"
                "net = TM.network_info(TM.parse_args(['--out', d, '--policy-hidden', '256,128,64']))\n"
                "TM.export_all(brax_params(49, 10), os.path.join(d, 'mjx'), 5, None, idx, 'walk', net, log=print)\n"
                "if %r:\n"
                "    from rl import train as TR\n"
                "    C.POSE_BANK.update(fallen_n=16, midfall_n=8, seated_n=8)   # kucuk bankalar (duman)\n"
                "    TR.main(['--out', os.path.join(d, 'cpu'), '--num-envs', '4', '--workers', '0', '--max-steps', '1',"
                " '--task', 'getup', '--getup-head', 'free'])\n" % (os.path.dirname(os.path.abspath(__file__)), d,
                                                                    torch_ok))
        run_tr35(code, extra_env={"OPENDUCK_BANK_N": "16"})
        mj = os.path.join(d, "mjx")
        assert sorted(os.listdir(os.path.join(mj, "c"))) == ["export_c.json", "obs_layout.h", "policy.c", "policy.h",
                                                            "policy_weights.h"]
        with open(os.path.join(mj, "c", "export_c.json")) as f:
            rep = json.load(f)
        assert rep["sizes"] == [49, 256, 128, 64, 10] and rep["robot"] == "tr35_b2"
        assert rep["check"] is None or rep["check"]["equal"]
        with open(os.path.join(mj, "policy_meta.json")) as f:
            m = json.load(f)
        assert m["robot"] == "tr35_b2" and m["network"]["activation"] == "swish" and "actuators" in m
        # P5a entegrasyonu: meta ve C basligi turetilmis dosyayi (rl/derived/tr35_b2.json) ve aktuator tablosunu okur
        import hashlib
        with open(C.DERIVED_PATHS["tr35_b2"], "rb") as f:
            raw = f.read()
        dsrc = json.loads(raw)["source"]
        assert m["derived_source"] == dsrc and m["derived_sha256"] == hashlib.sha256(raw).hexdigest()
        assert m["kappa"] == 0.5 and rep["derived_sha256"] == m["derived_sha256"]
        with open(os.path.join(mj, "c", "obs_layout.h")) as f:
            hdr = f.read()
        assert '#define OPENDUCK_DERIVED_MJCF_SHA256 "%s"' % dsrc["mjcf_sha256"] in hdr
        assert "OPENDUCK_ACT_TAU_S[15]" in hdr and "OPENDUCK_ACT_PWM_RES[15]" in hdr and "#define OPENDUCK_KAPPA 0.5f" in hdr
        if torch_ok:
            cp = os.path.join(d, "cpu")
            assert os.path.exists(os.path.join(cp, "c", "export_c.json")), sorted(os.listdir(cp))
            with open(os.path.join(cp, "policy_meta.json")) as f:
                m = json.load(f)
            assert m["task"] == "getup" and m["getup_head"] == "free" and m["network"]["activation"] == "elu"
            with open(os.path.join(cp, "config.json")) as f:
                assert json.load(f)["args"]["getup_head"] == "free"


# --- RunPod baslatici ------------------------------------------------------------------------------------
def _launch_module(src=None, name="openduck_launch"):
    path = os.path.join(ROOT, "tools", "runpod", "launch.py")
    mod = types.ModuleType(name)
    mod.__file__ = path
    if src is None:
        with open(path) as f:
            src = f.read()
    exec(compile(src, path, "exec"), mod.__dict__)
    return mod


def _run_launch(L, argv):
    so, se = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(so), contextlib.redirect_stderr(se):
        try:
            rc = L.main(argv)
        except SystemExit as e:
            rc = e.code
    return rc, so.getvalue(), se.getvalue()


def test_launcher_tr35_dry_run_and_xl330_unchanged():
    """--robot tr35_b2: pod ortaminda OPENDUCK_ROBOT (ve --kappa -> OPENDUCK_KAPPA), tr35 dosya listesi; xl330 istek
    govdesi (paket, ortam, adaylar) degisiklik oncesi baslaticiyla (673f94c) bayt bayt ayni."""
    L = _launch_module()
    assert L.needed("tr35_b2") == ["rl/__init__.py", "rl/config.py", "rl/robot_model.py", "rl/task_math.py",
                                   "build/tr35_b2/sim/openduck.xml", "build/tr35_b2/sim/scene.xml",
                                   "rl/derived/tr35_b2.json"]
    assert L.needed() == L.NEEDED and L.inline_extra("xl330_27") == L.INLINE_EXTRA
    assert "build/sim/scene.xml" not in L.inline_extra("tr35_b2") and "rl/derived/tr35_b2.json" in L.inline_extra("tr35_b2")
    rc, out, err = _run_launch(L, ["launch", "--dry-run", "--robot", "tr35_b2", "--kappa", "0.5", "--name", "t35",
                                   "--", "--task", "walk"])
    assert rc in (0, 1), (rc, err[-2000:])
    assert '"OPENDUCK_ROBOT": "tr35_b2"' in out and '"OPENDUCK_KAPPA": "0.5"' in out, out[:3000]
    for f in L.TR35_FILES:
        assert f in out, f
    assert '"name": "openduck-t35"' in out and "build/sim/openduck.xml" not in out
    rc, out, err = _run_launch(L, ["launch", "--dry-run", "--kappa", "0.5", "--name", "x"])
    assert rc == 2 and "--kappa" in err
    rc, out, err = _run_launch(L, ["launch", "--dry-run", "--name", "x"])
    assert rc in (0, 1) and "OPENDUCK_ROBOT" not in out and "OPENDUCK_KAPPA" not in out
    # degisiklik oncesi baslatici
    try:
        src = subprocess.run(["git", "-C", ROOT, "show", "673f94c:tools/runpod/launch.py"], capture_output=True,
                             text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        pytest.skip("git yok")
    if src.returncode:
        pytest.skip("673f94c yok: %s" % src.stderr[:200])
    old = _launch_module(src.stdout, "openduck_launch_old")

    def body(M, extra):
        a = M.build_parser().parse_args(["launch", "--name", "x"] + extra)
        a.compute = a.compute or ("gpu" if a.backend == "mjx" else "cpu")
        code, files, raw = M.build_inline(a.backend, strict=False)
        env = M.pod_env(a, "x", "0123456789abcdef", ["--task", "walk"], 2.0, 1.8e9, code)
        c = M.create_bodies(a, "openduck-x", env, list(M.DEFAULT_GPUS), ["SECURE", "COMMUNITY"],
                            list(M.DEFAULT_CPU_FLAVORS))
        return code, files, json.dumps(c, sort_keys=False)
    for extra in ([], ["--backend", "cpu"], ["--backend", "cpu", "--compute", "gpu", "--ssh", "--api", "v1"]):
        o, n = body(old, extra), body(L, extra)
        assert o[1] == n[1] and o[0] == n[0] and o[2] == n[2], extra


def test_launcher_datacenter_and_network():
    """--datacenter / --min-download-mbps: REST v1'e gecer, govdede dataCenterIds (sirali) + minDownloadMbps."""
    L = _launch_module()
    rc, out, err = _run_launch(L, ["launch", "--dry-run", "--name", "dc", "--gpu", "4090", "--cloud", "SECURE",
                                   "--datacenter", "eu-ro-1,EU-CZ-1", "--datacenter", "EUR-IS-1",
                                   "--min-download-mbps", "500", "--", "--task", "walk"])
    assert rc in (0, 1), (rc, err[-2000:])
    assert "rest.runpod.io/v1" in out and "(--api v1)" in out
    body = json.loads(out[out.index("{"):out.index("\n}\n") + 2])
    assert body["dataCenterIds"] == ["EU-RO-1", "EU-CZ-1", "EUR-IS-1"] and body["dataCenterPriority"] == "custom"
    assert body["minDownloadMbps"] == 500 and body["cloudType"] == "SECURE"
