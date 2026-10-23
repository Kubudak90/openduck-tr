# -*- coding: utf-8 -*-
"""
tr35_b2 gorev profili testleri (OpenDuck-TR P5a, cekirdek): config.load_derived / tr35_constants formulleri (xl330
surecinden), tr35_b2 SURECI (alt surec; P2-0 stub + derive paketinin rl/derived/tr35_b2.json'u; dosya yoksa test-yalniz
bootstrap) icinde sabitler, model varyantlari, kucuk poz bankalari, robot korumasi, getup_head="fixed" (CPU ve MJX) ve
kisa CPU == MJX adim adim esitligi.

  python3 -m pytest rl/tests/test_tr35_core.py

Stub, freecadcmd varsa surec basina bir kez derlenir (rl/tests/tr35_scene.py; ~3 s); yoksa stub testleri atlanir
($OPENDUCK_TR35_SCENE ile hazir sahne verilebilir). MJX parcalari jax yoksa atlanir.
"""
import copy
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

from rl import config as C
from rl import tasks as TK
from rl.tests.tr35_scene import bootstrap_for, tr35_env, tr35_scene

ROOT = C.ROOT
S = 1.282
TR = "tr35_b2"


def _boot():
    """(sahne, turetilmis dosya): rl/derived/tr35_b2.json (derive paketi, varsayilan yol; P5a entegrasyonu); yoksa
    test-yalniz bootstrap (birlesme oncesi)."""
    scene = tr35_scene()
    p = C.DERIVED_PATHS[TR]
    return scene, (p if os.path.isfile(p) else bootstrap_for(scene))


def _run(code, env, timeout=1200):
    """tr35 alt sureci: once rl.config.SCENE_PATH (arayuz kurali), sonra code. stdout'un son satiri JSON."""
    pre = ("import sys, json\nsys.path.insert(0, %r)\nimport rl.config as C\nC.SCENE_PATH = %r\n"
           % (ROOT, env["OPENDUCK_SCENE"]))
    r = subprocess.run([sys.executable, "-c", pre + code], cwd=ROOT, env=env, capture_output=True, text=True,
                       timeout=timeout)
    assert r.returncode == 0, "alt surec hatasi:\n%s\n%s" % (r.stdout[-3000:], r.stderr[-6000:])
    return json.loads(r.stdout.strip().splitlines()[-1])


def _has(mod):
    import importlib.util
    return importlib.util.find_spec(mod) is not None


# --- xl330 surecinde: yeni adlar, formuller, dogrulama --------------------------------------------------------
def test_xl330_new_names_and_snapshot():
    """xl330: yeni adlarin xl330 degerleri; tr35_constants xl330 sozluklerini degistirmez."""
    assert C.ROBOT_PROFILE == "xl330_27"
    assert C.SCENE_PATHS == {"xl330_27": os.path.join(ROOT, "build", "sim", "scene.xml"),
                             TR: os.path.join(ROOT, "build", TR, "sim", "scene.xml")}
    assert C.DERIVED_PATHS == {TR: os.path.join(ROOT, "rl", "derived", TR + ".json")}
    if not os.environ.get("OPENDUCK_SCENE"):
        assert C.SCENE_PATH == C.SCENE_PATHS["xl330_27"]
    assert C.DERIVED is None and C.DERIVED_SOURCE is None and (C.DERIVED_PATH == os.environ.get("OPENDUCK_DERIVED"))
    assert (C.FROUDE_S, C.KAPPA, C.SEAT_PITCH_DEG, C.HEAD_TUCK_DEG, C.HOME_OVERRIDE) == (None, None, 0.0, None, None)
    assert C.SELF_COLLISION_PAIRS == () and C.GETUP_HEAD_MODES == ("free", "fixed") and C.GETUP_HEAD_DEFAULT == "free"
    assert C.GETUP_HEAD_SCALE is None and C.MOVING_EPS == 0.05 and C.RAIL_SOFT_A == 15.0
    assert C.MAIN["stand_gz"] == C.MAIN["seat_gz"] == -0.9 and C.GETUP["torque_limit"] == 0.96
    assert "slew_max" not in C.HEAD_RANDOM and C.HEAD_GEOMS == ("head_collision", "jaw_collision")
    assert TK.WALK.solver_iterations is None and TK.WALK.actor_hidden is None and TK.GETUP.episode_length_s == 8.0
    assert TK.GETUP.actor_hidden == (512, 256, 128) and TK.MAIN.solver_iterations == 8 == TK.GETUP.solver_iterations
    assert list(TK.WALK.reward_weights)[6] == "torques" and "torque_ratio" not in TK.MAIN.reward_weights


def test_tr35_constants_formulas():
    """tr35_constants(turetilmis dosya): Froude/kappa formulleri, odul anahtar listeleri, bas, rampalar, getup, devir, DR,
    geometri = JSON; saf (iki cagri esit, xl330 degismez); kappa ve OPENDUCK_KAPPA."""
    _, path = _boot()
    d = C.load_derived(path)
    before = copy.deepcopy(C._xl330)
    k = C.tr35_constants(d)
    live = {n: copy.deepcopy(getattr(C, n)) for n in k}                      # bu surecin (xl330) nesneleri
    k2 = C.tr35_constants(d)
    assert k == k2 and C._xl330 == before and all(getattr(C, n) == v for n, v in live.items())
    assert all(k[n] is not getattr(C, n) for n in k if isinstance(k[n], dict))
    sq, kap = math.sqrt(S), 0.5
    lv, lw, lt = sq * kap, kap / sq, sq / kap
    close = lambda a, b: abs(a - b) <= 1e-12 * max(1.0, abs(b))
    assert k["FROUDE_S"] == S and k["KAPPA"] == kap and close(k["GAIT_FREQ"], 2.21 * kap)
    assert close(k["AIR_TIME_TARGET"], 0.375 / (2.21 * kap)) and close(k["SWING_HEIGHT"], 0.02 * S)
    assert close(k["FOOT_HEIGHT_SIGMA"], 2e-4 * S ** 2) and k["ACTION_SCALE"] == 0.3
    cr = k["COMMAND_RANGES"]
    assert np.allclose(cr["vx"], (-0.15 * lv, 0.30 * lv)) and np.allclose(cr["vy"], (-0.15 * lv, 0.15 * lv))
    assert np.allclose(cr["wz"], (-lw, lw)) and close(k["MOVING_EPS"], 0.05 * lv) and k["MAIN"]["moving_eps"] == k["MOVING_EPS"]
    assert close(k["TRACKING_SIGMA_LIN"], 0.02 * lv ** 2) and close(k["TRACKING_SIGMA_ANG"], 0.1 * lw ** 2)
    # oduller: anahtarlar (xl330 sirasi, torques yerinde torque_ratio, sona limit/ray) ve olcekler
    sub = lambda keys: [("torque_ratio" if n == "torques" else n) for n in keys] + ["limit_torque", "rail_demand"]
    w, wx = k["REWARD_WEIGHTS"], C.REWARD_WEIGHTS
    assert list(w) == sub(wx) and list(k["MAIN_REWARD_WEIGHTS"]) == sub(C.MAIN_REWARD_WEIGHTS)
    assert list(k["GETUP_REWARD_WEIGHTS"]) == list(C.GETUP_REWARD_WEIGHTS) + ["limit_torque", "rail_demand"]
    assert list(k["GETUP_REWARD_LATE"]) == list(C.GETUP_REWARD_LATE)
    assert (w["torque_ratio"], w["limit_torque"], w["rail_demand"]) == (-2e-3, -0.5, -0.2)
    for n, f in (("lin_vel_z", 1 / lv ** 2), ("ang_vel_xy", 1 / lw ** 2), ("base_height", 1 / S ** 2),
                 ("joint_acc", lt ** 4), ("feet_slip", 1 / lv ** 2), ("feet_distance", 1 / S)):
        assert close(w[n], wx[n] * f), n
    for n in ("tracking_lin_vel", "orientation", "action_rate", "feet_air_time", "feet_phase", "hip_pose", "alive"):
        assert w[n] == wx[n], n
    wm = k["MAIN_REWARD_WEIGHTS"]
    for n in wm:
        if n in w and n not in ("limit_torque", "rail_demand", "torque_ratio"):
            assert wm[n] == w[n], n
    assert close(wm["height_l1"], C.MAIN_REWARD_WEIGHTS["height_l1"] / S) and close(wm["vz_cap"], -20.0 / sq)
    assert wm["posture"] == -1.0 and wm["seat"] == 1.0
    M = k["MAIN"]
    assert close(M["height_sigma"], 0.02 * S) and close(M["vz_cap"], 0.08 * sq) and M["stand_gz"] == -0.9
    assert (M["standup_window_s"], M["sitdown_window_s"]) == (3.4, 4.0)
    assert close(k["SIT_RAMP_S"], 2.5 * sq) and k["STAND_RAMP_S"] == k["SIT_RAMP_S"] and abs(k["SIT_RAMP_S"] - 2.83) < 0.01
    wg, wl, G = k["GETUP_REWARD_WEIGHTS"], k["GETUP_REWARD_LATE"], k["GETUP"]
    assert k["GETUP_EPISODE_S"] == 12.0 and G["torque_limit"] == 2.88 and wg["torque_sat"] == -1.0
    assert close(wg["height_l1"], -6.0 / S) and close(wg["height_progress"], 50.0 / S) and wg["upright_progress"] == 5.0
    assert wg["joint_vel"] == 0.0 and (wg["limit_torque"], wg["rail_demand"]) == (-0.5, -0.2)
    assert close(wl["joint_vel"], -1e-4 * S) and close(wl["arrival_damping"], -0.05 * S) and close(wl["head_impact"], -1.0 / S)
    assert np.allclose(G["height_sigma"], (0.04 * S, 0.015 * S)) and np.allclose(G["composite_sigma"], (0.03 * S, 0.4, 0.4))
    assert close(G["success"]["gyro"], 0.5 / sq) and close(G["success"]["hold_s"], sq)
    assert np.allclose(G["midfall"]["ang_vel"], (0.0, 4.0 / sq)) and G["midfall"]["tilt_deg"] == (15.0, 35.0)
    # bas (spec 3.2.5): araliklar, filtre, egim siniri; rastgele bas ve oturus scripti hizi <= slew_max
    hr = k["HEAD_RANDOM"]
    assert hr["range"] == {"neck_pitch": 0.15, "head_pitch": 0.2, "head_yaw": 0.5, "head_roll": 0.08}
    assert hr["filter_tau"] == 0.5 and hr["prob"] == 0.5 and hr["resample_s"] == (2.0, 4.0)
    from rl import actuators as A
    for j in C.HEAD_JOINTS:
        assert close(hr["slew_max"][j], 0.5 * A.CLASSES[A.JOINT_CLASS[TR][j]]["omega0"]), j
        assert 2.0 * hr["range"][j] / hr["filter_tau"] <= hr["slew_max"][j], j
        ds1 = 1.0 / (k["SIT_RAMP_S"] * C.SIT_SPLIT)                         # s1 hizi (1/s)
        assert ds1 * abs(math.radians(k["HEAD_SIT_DEG"][j] - k["HEAD_HOME_DEG"][j])) <= hr["slew_max"][j], j
    assert k["GETUP_HEAD_SCALE"] == {"head_pitch": 0.2, "head_yaw": 0.5, "head_roll": 0.08}
    assert k["SUPERVISOR"]["fall_steps"] == 6 and k["SUPERVISOR"]["ready_hold_s"] == 0.6
    assert k["SUPERVISOR"]["fall_gz"] == C.SUPERVISOR["fall_gz"] and close(k["RESET_BASE_VEL_NOISE"], 0.05 * sq)
    pb, dr = k["POSE_BANK"], k["DOMAIN_RAND"]
    assert pb["settle_s"] == 2.5 and np.allclose(pb["drop"], (0.02 * S, 0.10 * S)) and pb["fallen_n"] == C.POSE_BANK["fallen_n"]
    f = d["mass"]["total_kg"] / 0.6763
    assert np.allclose(dr["trunk_added_mass"], (-0.03 * f, 0.05 * f)) and dr["trunk_com_offset"] == 0.010
    assert dr["push_vel"] == 0.20 and dr["friction"] == C.DOMAIN_RAND["friction"]
    # geometri = JSON
    hz = d["heights"]
    for n in ("H_HOME", "H_MIN_WALK", "H_MIN_STAND", "H_DEEP", "Z_SIT", "SEAT_Z", "SEAT_Z_TOL", "TERMINATION_MIN_HEIGHT"):
        assert k[n] == hz[n], n
    assert k["BASE_HEIGHT_TARGET"] == hz["H_HOME"] and k["H_NORM"] == hz["H_HOME"] - hz["H_MIN_STAND"]
    assert M["standup_z"] == hz["standup_z"] and G["success"]["z"] == hz["getup_success_z"] and M["seat_gz"] == d["seat"]["seat_gz"]
    assert G["pose_z"] == tuple(hz["getup_pose_z"]) and G["arrival_z"] == tuple(hz["getup_arrival_z"])
    assert k["CROUCH_FAMILY"] == tuple(tuple(r) for r in d["crouch_family"]) and k["SEAT_DEG"] == d["seat"]["deg"]
    assert k["SEAT_PITCH_DEG"] == d["seat"]["pitch_deg"] and k["HEAD_TUCK_DEG"] == d["head"]["tuck_deg"]
    assert k["HEAD_SIT_DEG"] == d["head"]["sit_deg"] and k["HEAD_HOME_DEG"] == d["head"]["home_deg"]
    assert k["FEET_MIN_DISTANCE"] == d["feet"]["FEET_MIN_DISTANCE"]
    assert all(k[n] == d["caps_deg"][n] for n in d["caps_deg"])
    assert k["HOME_OVERRIDE"] == (d["home"] if d["home"]["source"] == "family" else None)
    assert k["EXTRA_COLLISION"] == {b: (g[0], tuple(g[1]), tuple(g[2]), g[3]) for b, g in d["extra_collision"].items()}
    assert k["SELF_COLLISION_PAIRS"] == tuple(map(tuple, d["self_collision_pairs"]))
    assert k["HEAD_GEOMS"] == ("head_collision", "jaw_collision", "neck_bumper")
    # kappa
    k6 = C.tr35_constants(d, kappa=0.6)
    assert close(k6["GAIT_FREQ"], 2.21 * 0.6) and close(k6["COMMAND_RANGES"]["vx"][1], 0.30 * sq * 0.6)
    old = os.environ.get("OPENDUCK_KAPPA")
    os.environ["OPENDUCK_KAPPA"] = "0.7"
    try:
        assert close(C.tr35_constants(d)["GAIT_FREQ"], 2.21 * 0.7)
    finally:
        os.environ.pop("OPENDUCK_KAPPA") if old is None else os.environ.__setitem__("OPENDUCK_KAPPA", old)
    # family HOME -> HOME_OVERRIDE
    d2 = copy.deepcopy(d)
    d2["home"]["source"] = "family"
    assert C.tr35_constants(d2)["HOME_OVERRIDE"] == d2["home"]


def _broken(d, fn):
    d = copy.deepcopy(d)
    fn(d)
    p = os.path.join(tempfile.mkdtemp(prefix="tr35_bad_"), "bad.json")
    with open(p, "w") as f:
        json.dump(d, f)
    return p


def test_load_derived_validation():
    """load_derived: sema, profil, anahtarlar, tipler ve iliskiler; bozuk dosya ValueError (alan adiyla)."""
    _, path = _boot()
    d = C.load_derived(path)
    assert d["schema"] == 1 and d["profile"] == TR and d["source"]["stub"] is True
    assert d["tool"] in ("tools/derive_posture.py", "bootstrap")
    with open(path) as f:
        txt = f.read()
    assert txt.endswith("\n") and txt == json.dumps(json.loads(txt), indent=1, sort_keys=True) + "\n"   # deterministik
    fam = d["crouch_family"]
    assert len(fam) >= 3 and fam[0][0] >= d["heights"]["H_HOME"] and fam[-1][0] <= d["heights"]["H_DEEP"]
    cases = {
        "schema": lambda x: x.update(schema=2),
        "profile": lambda x: x.update(profile="xl330_27"),
        "eksik": lambda x: x.pop("seat"),
        "H_HOME": lambda x: x["heights"].update(H_MIN_WALK=x["heights"]["H_HOME"] + 0.01),
        "Z_SIT": lambda x: x["heights"].update(Z_SIT=x["heights"]["SEAT_Z"]),
        "azalan": lambda x: x["crouch_family"].__setitem__(1, [x["crouch_family"][0][0]] + x["crouch_family"][1][1:]),
        "duz taban": lambda x: x["crouch_family"][2].__setitem__(4, x["crouch_family"][2][4] + 1.0),
        "ilk h": lambda x: x["crouch_family"][0].__setitem__(0, x["heights"]["H_HOME"] - 1e-4),
        "qpos": lambda x: x["home"].update(qpos=x["home"]["qpos"][:-1]),
        "legs_deg": lambda x: x["home"]["legs_deg"].pop("knee"),
        "head": lambda x: x["head"]["tuck_deg"].update(neck_pitch="36"),
        "seat_gz": lambda x: x["seat"].update(seat_gz=0.5),
        "extra": lambda x: x["extra_collision"].update(jaw=["box", [0, 0, 0], [0.01, 0.01], None]),
        "pairs": lambda x: x.update(self_collision_pairs=[["a", "a"]]),
        "bool": lambda x: x["mass"].update(total_kg=True),
    }
    for name, fn in cases.items():
        try:
            C.load_derived(_broken(d, fn))
        except ValueError:
            continue
        raise AssertionError("bozuk dosya kabul edildi: %s" % name)


def test_tr35_missing_or_invalid_derived_raises():
    """tr35_b2 sureci: turetilmis dosya yok/bozuk -> RuntimeError (yeniden uretme komutu ile)."""
    scene, path = _boot()
    for bad in (os.path.join(tempfile.mkdtemp(), "yok.json"),
                _broken(C.load_derived(path), lambda x: x.update(schema=9))):
        r = subprocess.run([sys.executable, "-c", "import rl.config"], cwd=ROOT, env=tr35_env(scene, bad),
                           capture_output=True, text=True, timeout=120)
        assert r.returncode != 0 and "RuntimeError" in r.stderr and "derive_posture" in r.stderr, r.stderr[-2000:]


def test_tr35_import_orders():
    """tr35_b2: config <-> actuators dongusel importu (bas egim siniri CLASSES'tan) hangi modul once yuklenirse
    yuklensin calisir."""
    scene, path = _boot()
    for m in ("actuators", "sensors", "robot_model", "task_math"):
        code = "import rl.%s, rl.config as C; print(C.HEAD_RANDOM['slew_max']['head_yaw'], C.GAIT_FREQ)" % m
        r = subprocess.run([sys.executable, "-c", code], cwd=ROOT, env=tr35_env(scene, path), capture_output=True,
                           text=True, timeout=120)
        assert r.returncode == 0 and r.stdout.split() == ["6.55", "1.105"], (m, r.stdout, r.stderr[-2000:])


def test_real_derived_file_if_present():
    """Birlesmeden sonra: rl/derived/tr35_b2.json (derive paketi) load_derived + tr35_constants'tan gecer."""
    p = C.DERIVED_PATHS[TR]
    if not os.path.isfile(p):
        import pytest
        pytest.skip("rl/derived/tr35_b2.json henuz yok (derive birlesmedi)")
    d = C.load_derived(p)
    k = C.tr35_constants(d)
    assert k["H_HOME"] == d["heights"]["H_HOME"] and k["SELF_COLLISION_PAIRS"] == tuple(map(tuple, d["self_collision_pairs"]))


def test_getup_head_and_robot_guard_xl330():
    """xl330: getup_head="fixed" -> ValueError (HEAD_TUCK_DEG yok); bilinmeyen mod ValueError; walk'ta etkisiz;
    robot="tr35_b2" xl330 surecinde izinli (P4: 27 cm MJCF)."""
    from rl import actuators as A
    from rl.env_cpu import VecEnv
    for bad in ("fixed", "tucked"):
        try:
            VecEnv(2, num_workers=0, task="getup", getup_head=bad)
            raise AssertionError(bad)
        except ValueError:
            pass
    assert TK.getup_head_mode(None, "getup") == "free" and TK.getup_head_mode("fixed", "walk") == "fixed"
    assert A.env_profile("tr35_b2") == "tr35_b2" and A.env_profile(None) == "xl330_27"
    with VecEnv(2, num_workers=0, task="walk") as e:
        assert e.getup_head == "free"


def test_spec_pair_list_and_add_pairs():
    """tools/mjx_solver_bench.spec_pairs (spec 3.2.2 aday listesi) ve robot_model.add_self_pairs (eksik geom atlanir)."""
    import importlib.util
    import mujoco
    from rl import robot_model as RM
    sp = importlib.util.spec_from_file_location("_bench", os.path.join(ROOT, "tools", "mjx_solver_bench.py"))
    B = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(B)
    names = ["trunk_collision", "head_collision", "jaw_collision"] + ["%s_%s_collision" % (s, b) for s in ("left", "right")
                                                                     for b in ("hip", "hip_cradle", "thigh", "shin", "foot")]
    full, sub = B.spec_pairs(names), B.spec_pairs(names, subset=True)
    assert len(full) == len(set(full)) == 12 + 4 + 2 + 24 + 1, len(full)
    assert ("left_foot_collision", "right_foot_collision") not in full and len(sub) == 8
    assert set(sub) <= set(full) and all("trunk" in a + b or "foot" in a for a, b in sub)
    assert set(RM.main_pairs(full)) == set(sub) and RM.main_pairs([(b, a) for a, b in sub]) == tuple((b, a) for a, b in sub)
    spec = RM.build_spec(variant="getup")
    n = RM.add_self_pairs(spec, [("trunk_collision", "left_thigh_collision"), ("trunk_collision", "yok_collision")])
    assert n == 1 and spec.compile().npair == 1
    assert RM.build_model(variant="getup").npair == 0 and mujoco.MjSpec is not None


# --- tr35_b2 SURECI (alt surec; CPU) ----------------------------------------------------------------------
_PROC = r"""
import importlib, mujoco, numpy as np
from rl import actuators as A, tasks as TK, task_math as T, pose_bank as PB, robot_model as RM
from rl.env_cpu import VecEnv
d = C.DERIVED
out = {"profile": C.ROBOT_PROFILE, "scene": C.SCENE_PATH}
mods = ["config", "tasks", "task_math", "pose_bank", "robot_model", "actuators", "sensors", "env_cpu", "policy_meta",
        "eval_tasks", "play", "bench_env", "env_mjx", "train_mjx", "ppo", "train", "export_onnx"]
out["imports"] = {}
for mname in mods:
    try:
        importlib.import_module("rl." + mname)
        out["imports"][mname] = "ok"
    except ModuleNotFoundError as e:          # jax / torch yalniz kendi venv'inde
        assert e.name.split(".")[0] in ("jax", "jaxlib", "brax", "flax", "torch", "mujoco_playground", "ml_collections", "onnx", "onnxruntime"), e
        out["imports"][mname] = "skip:" + e.name
hz = d["heights"]
geo = {n: getattr(C, n) == hz[n] for n in ("H_HOME", "H_MIN_WALK", "H_MIN_STAND", "H_DEEP", "Z_SIT", "SEAT_Z",
                                          "SEAT_Z_TOL", "TERMINATION_MIN_HEIGHT")}
geo.update(BASE=C.BASE_HEIGHT_TARGET == hz["H_HOME"], H_NORM=C.H_NORM == hz["H_HOME"] - hz["H_MIN_STAND"],
           standup=C.MAIN["standup_z"] == hz["standup_z"], succ=C.GETUP["success"]["z"] == hz["getup_success_z"],
           pose_z=C.GETUP["pose_z"] == tuple(hz["getup_pose_z"]), arr=C.GETUP["arrival_z"] == tuple(hz["getup_arrival_z"]),
           fam=C.CROUCH_FAMILY == tuple(map(tuple, d["crouch_family"])), seat=C.SEAT_DEG == d["seat"]["deg"],
           pitch=C.SEAT_PITCH_DEG == d["seat"]["pitch_deg"], seat_gz=C.MAIN["seat_gz"] == d["seat"]["seat_gz"],
           head=(C.HEAD_HOME_DEG, C.HEAD_SIT_DEG, C.HEAD_TUCK_DEG) == (d["head"]["home_deg"], d["head"]["sit_deg"], d["head"]["tuck_deg"]),
           feet=C.FEET_MIN_DISTANCE == d["feet"]["FEET_MIN_DISTANCE"], caps=all(getattr(C, n) == v for n, v in d["caps_deg"].items()),
           extra=sorted(C.EXTRA_COLLISION) == sorted(d["extra_collision"]), src=C.DERIVED_SOURCE == d["source"])
out["geo"] = geo
out["tasks"] = {n: [t.num_obs, t.num_critic_obs, t.num_actions, t.solver_iterations, list(t.actor_hidden),
                    list(t.critic_hidden), t.episode_length_s, list(t.reward_weights)] for n, t in TK.TASKS.items()}
# getup aksiyon olcegi: kafa eklemleri GETUP_HEAD_SCALE ile sinirli, boyun tam
mg = RM.build_model(variant="getup")
ig = RM.RobotIndex(mg, TK.GETUP.action_joints)
sc = TK.action_scale_vector(TK.GETUP, ig.default_action_q, ig.action_lo, ig.action_hi)
out["getup_scale"] = sc.tolist()
# robot korumasi
guard = []
for r in ("xl330_27", "tr35_b2", None):
    try:
        VecEnv(1, num_workers=0, task="walk", robot=r).close()
        guard.append("ok")
    except ValueError:
        guard.append("ValueError")
try:
    from rl.env_mjx import OpenDuckEnv
    try:
        OpenDuckEnv(robot="xl330_27")
        guard.append("ok")
    except ValueError:
        guard.append("ValueError")
except ModuleNotFoundError:
    pass
out["guard"] = guard
# varyantlar, isaretler, sinirlar, EXTRA ve ciftler
mdl = {}
for v in RM.VARIANTS:
    m = RM.build_model(variant=v)
    gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    ex = [gid(b + "_collision") for b in C.EXTRA_COLLISION]
    jr = lambda n: np.degrees(m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]).tolist()
    cr = lambda n: np.degrees(m.actuator_ctrlrange[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)]).tolist()
    mdl[v] = {"variant": RM.model_variant(m), "profile": A.model_profile(m), "npair": int(m.npair),
               "extra": [[int(m.geom_contype[g]), int(m.geom_conaffinity[g])] if g >= 0 else None for g in ex],
               "knee": cr("left_knee"), "knee_r": cr("right_knee"), "knee_j": jr("left_knee"), "neck": cr("neck_pitch"),
               "hy": cr("left_hip_yaw"), "hy_r": cr("right_hip_yaw"),
               "head_geoms": [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) for g in RM.RobotIndex(m).head_geoms]}
out["models"] = mdl
# beklenen cift sayisi: iki geomu da modelde olan ciftler (main: main_pairs alt kumesi, walk: yok)
want = {}
for v in RM.VARIANTS:
    m = RM.build_model(variant=v)
    have = set(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) for g in range(m.ngeom))
    pr = RM.main_pairs(C.SELF_COLLISION_PAIRS) if v == "main" else C.SELF_COLLISION_PAIRS
    want[v] = 0 if v == "walk" else sum(int(a in have and b in have) for a, b in pr)
out["npair_want"] = want
out["npair_cfg"] = [len(C.SELF_COLLISION_PAIRS), len(RM.main_pairs(C.SELF_COLLISION_PAIRS))]
# oz-carpisma ciftleri: geomlari olanlar eklenir (main + getup), walk yok
C.SELF_COLLISION_PAIRS = (("trunk_collision", "left_thigh_collision"), ("head_collision", "trunk_collision"),
                          ("trunk_collision", "neck_bumper"))
out["pairs"] = {v: int(RM.build_model(variant=v).npair) for v in RM.VARIANTS}
C.SELF_COLLISION_PAIRS = ()
# HOME_OVERRIDE (family): keyframe qpos/ctrl ezilir, bosluk eklemli modelde de (sifirlar sonradan)
h = dict(d["home"])
q = list(h["qpos"]); q[7 + 3] += 0.05; c = list(h["ctrl"]); c[3] += 0.05
C.HOME_OVERRIDE = dict(h, source="family", qpos=q, ctrl=c)
mh = RM.build_model(variant="main")
mb = RM.build_model(variant="getup", backlash=True)
ib = RM.RobotIndex(mb)
out["home"] = [float(mh.key_qpos[0][10] - q[10]), float(mh.key_ctrl[0][3] - c[3]), len(ib.play_jnt),
               float(ib.init_qpos[ib.obs_qpos_adr[3]] - q[10]), float(np.abs(ib.init_qpos[mb.jnt_qposadr[ib.play_jnt]]).max())]
C.HOME_OVERRIDE = None
print(json.dumps(out))
"""


def test_tr35_process_constants_models_guard():
    """tr35_b2 sureci: butun rl modulleri yuklenir, geometri sabitleri = JSON, gorev duzenleri ayni, cozucu ve aglar,
    robot korumasi, varyant isaretleri, sinirlar, EXTRA (main yalniz cift / getup yalniz zemin), ciftler, HOME_OVERRIDE."""
    scene, path = _boot()
    o = _run(_PROC, tr35_env(scene, path))
    assert o["profile"] == TR and o["scene"] == scene
    assert all(v == "ok" or v.startswith("skip:") for v in o["imports"].values()) and o["imports"]["env_cpu"] == "ok"
    assert all(o["geo"].values()), o["geo"]
    t = o["tasks"]
    assert [t[n][:3] for n in ("walk", "main", "getup")] == [[49, 58, 10], [53, 64, 10], [48, 57, 14]]
    assert [t[n][3] for n in ("walk", "main", "getup")] == [8, 8, 16]
    assert all(t[n][4] == [256, 128, 64] and t[n][5] == [512, 256, 128] for n in t) and t["getup"][6] == 12.0
    assert t["walk"][7][6] == "torque_ratio" and t["walk"][7][-2:] == ["limit_torque", "rail_demand"]
    sc = o["getup_scale"]
    assert np.allclose(sc[11:14], [0.2, 0.5, 0.08]) and sc[10] > 0.5            # kafa sinirli, boyun tam ctrlrange
    assert o["guard"][:3] == ["ValueError", "ok", "ok"] and o["guard"][3:] in ([], ["ValueError"])
    md = o["models"]
    assert {v: md[v]["variant"] for v in md} == {"walk": "walk", "main": "main", "getup": "getup"}
    assert all(md[v]["profile"] == TR and md[v]["npair"] == o["npair_want"][v] for v in md), (o["npair_want"], md)
    pairs = C.load_derived(path)["self_collision_pairs"]
    assert o["npair_cfg"][0] == len(pairs) and o["npair_want"]["getup"] == len(pairs)     # stub'da hepsinin geomu var
    assert o["npair_want"]["main"] == o["npair_cfg"][1] and (not pairs or o["npair_cfg"][1] == 8)
    assert md["walk"]["extra"] == [None] * len(md["walk"]["extra"])
    assert md["main"]["extra"] == [[0, 0]] * len(md["main"]["extra"]) and md["getup"]["extra"] == [[0, 1]] * len(md["getup"]["extra"])
    caps = C.load_derived(path)["caps_deg"]
    for v in ("main", "getup"):
        assert abs(md[v]["knee"][1] - caps["KNEE_LIMIT_DEG"]) < 1e-9 and abs(md[v]["knee_r"][0] + caps["KNEE_LIMIT_DEG"]) < 1e-9
        assert abs(md[v]["knee_j"][1] - caps["KNEE_LIMIT_DEG"] - C.JOINT_RANGE_MARGIN_DEG) < 1e-9
        assert md[v]["neck"][1] <= caps["NECK_PITCH_MAX_DEG"] + 1e-9
        assert md[v]["hy"][0] >= -caps["HIP_YAW_INWARD_MAX_DEG"] - 1e-9 and md[v]["hy_r"][1] <= caps["HIP_YAW_INWARD_MAX_DEG"] + 1e-9
        assert md[v]["head_geoms"] == ["head_collision", "jaw_collision"]             # neck_bumper stub'da yok
    assert md["walk"]["head_geoms"] == ["head_collision"]
    assert o["pairs"] == {"walk": 0, "main": 1, "getup": 2}               # main: main_pairs alt kumesi
    dq, dc, nplay, dqb, play = o["home"]
    assert abs(dq) < 1e-12 and abs(dc) < 1e-12 and nplay == 9 and abs(dqb) < 1e-12 and play == 0.0


_BANKS = r"""
import collections, mujoco, numpy as np
from rl import pose_bank as PB, task_math as T
from rl.robot_model import RobotIndex, build_model
out = {"det": {}}
for kind, n in (("fallen", 16), ("seated", 8), ("midfall", 8)):
    PB.clear_cache()
    a = PB.bank(kind, n, seed=5)
    PB.clear_cache()
    b = PB.bank(kind, n, seed=5)
    c = PB.bank(kind, n // 2, seed=5)
    same = all(np.array_equal(a[k], b[k]) for k in ("qpos", "qvel", "ctrl", "label"))
    pre = all(np.array_equal(c[k], b[k][:n // 2]) for k in ("qpos", "qvel", "ctrl", "label"))
    out["det"][kind] = [same, pre, b["robot"], b["variant"], PB.bank(kind, n, seed=5) is b]
fb = PB.fallen_bank(64)
mv = build_model(variant="getup", visual=True)
dv = mujoco.MjData(mv)
meshg = [g for g in range(mv.ngeom) if mv.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
verts = {g: mv.mesh_vert[mv.mesh_vertadr[mv.geom_dataid[g]]:mv.mesh_vertadr[mv.geom_dataid[g]] + mv.mesh_vertnum[mv.geom_dataid[g]]]
         for g in meshg}
pen = []
for q in fb["qpos"]:
    dv.qpos[:] = q
    mujoco.mj_kinematics(mv, dv)
    pen.append(-min((verts[g] @ dv.geom_xmat[g].reshape(3, 3)[2] + dv.geom_xpos[g][2]).min() for g in meshg))
out["pen"] = [float(max(pen)), len(meshg)]
out["labels"] = collections.Counter(PB.LABELS[k] for k in fb["label"].tolist())
# kafa-yalniz (tepe ustu) reddi: kabul edilenlerde yok; zorla ters yerlestirme kurali tetikler mi
m = build_model(variant="getup")
idx = RobotIndex(m)
head = set(int(g) for g in idx.head_geoms)
dd = mujoco.MjData(m)
bad = 0
for q, u in zip(fb["qpos"], fb["ctrl"]):
    dd.qpos[:] = q
    dd.ctrl[:] = u
    mujoco.mj_forward(m, dd)
    g = T.projected_gravity(np, q[3:7])
    t = PB.floor_contact_geoms(m, dd, idx.floor_geom)
    bad += int(g[2] > C.POSE_BANK["inverted_gz"] and bool(t) and t <= head)
bld = PB._Builder("getup")
inv = []
for k in range(6):
    ang = np.pi + np.deg2rad(4.0 * (k - 2.5))
    bld.place(PB.axis_angle_quat((1, 0, 0), ang), idx.init_ctrl.copy(), 0.002)
    ok = bld.settle()
    g = bld.gravity()
    t = PB.floor_contact_geoms(bld.m, bld.d, idx.floor_geom)
    inv.append([bool(ok), float(g[2]), bool(t) and t <= head, sorted(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, x) for x in t)])
out["head_only"] = [bad, inv]
# oturus bankasi: govde + ayaklar yerde, pitch ~ SEAT_PITCH_DEG, z ~ SEAT_Z
sb = PB.seated_bank(32)
g = T.projected_gravity(np, sb["qpos"][:, 3:7])
out["seated"] = [float(np.degrees(np.arcsin(g[:, 0])).min()), float(np.degrees(np.arcsin(g[:, 0])).max()),
                 float(g[:, 2].max()), float(sb["qpos"][:, 2].min()), float(sb["qpos"][:, 2].max()), sb["candidates"]]
# turetilmis oturus pozunun kendisi (gurultusuz, egimsiz; seated bankasiyla ayni hedef ve yerlestirme): isaret kurali
# (SEAT_PITCH_DEG + = burun asagi) ve SEAT_Z derive ile ayni mi
sbd = PB._Builder("main")
ref = T.posture_ref(np, np.array(C.H_HOME), np.array(1.0), sbd.idx.default_obs_q)
tg = sbd.idx.init_ctrl.copy()
tg[sbd.idx.obs_act_id[:10]] = ref["q_ref"]
tg[sbd.idx.head_act_id] = sbd.idx.default_obs_q[10:14] + ref["head_offset"]
tg = np.clip(tg, sbd.m.actuator_ctrlrange[:, 0], sbd.m.actuator_ctrlrange[:, 1])
sbd.place(PB.axis_angle_quat((0, 1, 0), np.deg2rad(C.SEAT_PITCH_DEG)), tg, 0.001)
sbd.settle()
ge = sbd.gravity()
te = PB.floor_contact_geoms(sbd.m, sbd.d, sbd.idx.floor_geom)
out["seat_exact"] = [float(np.degrees(np.arcsin(ge[0]))), float(sbd.d.qpos[2]),
                     bool(int(sbd.idx.trunk_geom) in te and set(int(x) for x in sbd.idx.foot_geom) <= te)]
# getup modelinde HOME'da EXTRA kutulari zemine degmez
d0 = mujoco.MjData(m)
d0.qpos[:] = idx.init_qpos
mujoco.mj_forward(m, d0)
out["home_extra_touch"] = sorted(PB.floor_contact_geoms(m, d0, idx.floor_geom, eps=1e-3) & set(int(x) for x in idx.extra_geoms))
print(json.dumps(out))
"""


def test_tr35_small_banks():
    """tr35_b2 kucuk bankalar (tr35 aktuatorleri, stub): belirlenimci ve onek kararli, mesh-zemin batmasi <= 3 mm x S,
    kafa-yalniz red kurali, oturus (egik: SEAT_PITCH_DEG) ve HOME'da EXTRA kutulari zeminde degil."""
    scene, path = _boot()
    o = _run(_BANKS, tr35_env(scene, path))
    for kind in ("fallen", "seated", "midfall"):
        assert o["det"][kind] == [True, True, TR, {"seated": "main"}.get(kind, "getup"), True], (kind, o["det"][kind])
    assert o["pen"][1] > 10 and o["pen"][0] <= 0.003 * S, o["pen"]
    assert sum(o["labels"].values()) == 64
    bad, inv = o["head_only"]
    assert bad == 0 and all(x[0] for x in inv), inv
    d = C.load_derived(path)
    p0, p1, gzmax, z0, z1, cand = o["seated"]
    tilt = C.POSE_BANK["seated_tilt_deg"]
    sz, tol = d["heights"]["SEAT_Z"], d["heights"]["SEAT_Z_TOL"]
    pe, ze, touch = o["seat_exact"]
    print("tr35 oturus: turetilmis pitch %.1f z %.4f | tam poz oturtmasi pitch %.1f z %.4f | banka pitch %.1f..%.1f "
          "z %.4f..%.4f (%d aday)" % (d["seat"]["pitch_deg"], sz, pe, ze, p0, p1, z0, z1, cand))
    # turetilmis pozun kendisi: ayni isaret kurali ve SEAT_Z (cekirdek <-> derive arayuzu)
    assert touch and abs(pe - d["seat"]["pitch_deg"]) < 2.0 and abs(ze - sz) <= tol, o["seat_exact"]
    assert gzmax < d["seat"]["seat_gz"] and cand < 10 * 32
    if not d["source"]["stub"]:
        # P2-0 stub'da oturus marjinal (COM payi 6.9 mm): +-5 derece / 0.05 rad gurultulu yerlestirmeler one devrilip
        # govde ustune yatiyor (P5a notlari); pencere yalniz son MJCF'de (P5b) zorunlu
        assert d["seat"]["pitch_deg"] - 2 * tilt < p0 <= p1 < d["seat"]["pitch_deg"] + 2 * tilt, o["seated"]
        assert sz - 2 * tol < z0 <= z1 < sz + 2 * tol, (o["seated"], sz)
    assert o["home_extra_touch"] == []


_HEADFIX_CPU = r"""
import numpy as np
from rl import tasks as TK
from rl.env_cpu import VecEnv
out = {}
for gh in (None, "fixed"):
    e = VecEnv(3, num_workers=0, task="getup", getup_head=gh, bank_n=16, seed=3)
    o, _ = e.reset()
    idx, spec = e.robot_index, e.spec
    rng = np.random.default_rng(0)
    for k in range(5):
        o, _, r, d, _ = e.step(rng.uniform(-2, 2, (3, 14)).astype(np.float32))
    s = e.task_state()
    la = o[:, spec.obs_slices["last_action"]]
    out[str(gh)] = {"mode": e.getup_head, "head_ctrl": s["ctrl_prev"][:, idx.head_act_id].tolist(),
                    "la_head": np.abs(la[:, 10:14]).max().item(), "la_legs": np.abs(la[:, :10]).max().item(),
                    "finite": bool(np.isfinite(o).all() and np.isfinite(r).all()),
                    "res": e._core.idx.act["pwm_res"][idx.head_act_id].tolist()}
    e.close()
print(json.dumps(out))
"""


def test_tr35_getup_head_fixed_cpu():
    """getup_head="fixed" (CPU, tr35_b2): bas hedefi = radians(HEAD_TUCK_DEG) (PWM adimina yuvarli), bas aksiyonlari 0
    (last_action gozlemi); "free"/None: bas politikada."""
    scene, path = _boot()
    o = _run(_HEADFIX_CPU, tr35_env(scene, path))
    tuck = np.deg2rad([C.load_derived(path)["head"]["tuck_deg"][j] for j in C.HEAD_JOINTS])
    fx, fr = o["fixed"], o["None"]
    assert fx["mode"] == "fixed" and fr["mode"] == "free" and fx["finite"] and fr["finite"]
    res = np.array(fx["res"])
    assert np.allclose(fx["head_ctrl"], np.round(tuck / res) * res, atol=1e-9) and fx["la_head"] == 0.0
    assert fx["la_legs"] > 0.5 and fr["la_head"] > 0.5


_TERMS = r"""
import numpy as np
from rl import tasks as TK, task_math as T
from rl.env_cpu import VecEnv
out = {}
rng = np.random.default_rng(1)
for task in ("walk", "main", "getup"):
    e = VecEnv(4, num_workers=0, task=task, bank_n=16, seed=2)
    e.reset()
    for k in range(6):
        e.step(rng.uniform(-1.5, 1.5, (4, e.num_actions)).astype(np.float32))
    s = e.task_state()
    if task != "getup":
        s.update(air_time=np.zeros((4, 2)), first_contact=np.zeros((4, 2)))
    terms = T.reward_terms(np, s, task)
    w = TK.get_task(task).reward_weights
    r = {"keys": list(terms) == list(w)}
    lim = np.sum(np.square(s["limit_torque"] / s["tau_s"]), -1)
    rail = np.maximum(s["rail_demand"] - C.RAIL_SOFT_A, 0.0)
    r["limit"] = bool(np.allclose(terms["limit_torque"], lim)) and r["keys"]
    r["rail"] = bool(np.allclose(terms["rail_demand"], rail))
    if task == "getup":
        f = C.GETUP["torque_sat_frac"]
        r["sat"] = bool(np.allclose(terms["torque_sat"], np.sum(np.maximum(np.abs(s["tau_ratio"]) - f, 0.0), -1)))
        ev, inc = T.getup_events(np, s, {"success_t": np.zeros(4), "succeeded": np.zeros(4)})
        r["sat_steps"] = bool(np.array_equal(inc["torque_sat_steps"], (np.abs(s["tau_ratio"]).max(-1) > f).astype(float)))
        r["tau_max"] = float(np.abs(s["tau_ratio"]).max())
    else:
        r["ratio"] = bool(np.allclose(terms["torque_ratio"], np.sum(np.square(s["tau_ratio"]), -1)))
    out[task] = r
# main: pitch_ref oturusta SEAT_PITCH_DEG; stood (stand_gz) / seated (seat_gz) ayri esikler
h = np.full(3, C.H_HOME)
pr = T.pitch_ref(np, h, np.array([0.0, 0.6, 1.0]))
ref = T.posture_ref(np, h, np.array([0.0, 0.6, 1.0]), np.zeros(14))
out["pitch"] = [float(v) for v in pr] + [bool(np.allclose(pr, ref["pitch_ref"]))]
print(json.dumps(out))
"""


def test_tr35_task_math_terms():
    """tr35_b2 odul terimleri (CPU ortam durumu uzerinde): anahtarlar = TaskSpec sirasi, torque_ratio, limit_torque,
    rail_demand, getup torque_sat ve torque_sat_steps tau_ratio ile; pitch_ref oturusta radians(SEAT_PITCH_DEG)."""
    scene, path = _boot()
    o = _run(_TERMS, tr35_env(scene, path))
    for task in ("walk", "main", "getup"):
        assert all(v for k, v in o[task].items() if k != "tau_max"), (task, o[task])
    d = C.load_derived(path)
    p0, p6, p1, same = o["pitch"]
    assert same and abs(p1 - math.radians(d["seat"]["pitch_deg"])) < 1e-12 and abs(p6 - p0) < 0.5


_STAND = r"""
import numpy as np
from rl import task_math as T
from rl.env_cpu import VecEnv
out = {}
for bl in (False, True):
    e = VecEnv(1, num_workers=0, task="walk", randomize=False, noise=False, head_random=False, push=False,
               fixed_command=(0.0, 0.0, 0.0), backlash=bl)
    e.reset()
    tilt, fell = 0.0, False
    for k in range(250):
        _, _, _, d, _ = e.step(np.zeros((1, 10), np.float32))
        tilt = max(tilt, float(np.degrees(T.tilt_angle(np, e.task_state()["quat"]))[0]))
        fell = fell or bool(d[0])
    out[str(bl)] = [tilt, fell]
    e.close()
print(json.dumps(out))
"""


def test_tr35_home_stands_5s():
    """A3 (tr35 stub, tr35 servolari): HOME'da acik dongu 5 s ayakta, egim <= 2 derece (bosluksuz); bosluk (nominal
    0.5 derece, 9 DS eklemi) ile dusmez (egim raporlanir; stub'da COM ayak merkezinin 19.9 mm gerisinde)."""
    scene, path = _boot()
    o = _run(_STAND, tr35_env(scene, path))
    print("tr35 HOME 5 s: egim %.2f derece (bosluksuz), %.2f derece (bosluk)" % (o["False"][0], o["True"][0]))
    assert o["False"][0] <= 2.0 and not o["False"][1] and not o["True"][1], o


# --- MJX (alt surec; jax gerekli) ----------------------------------------------------------------------------
_HEADFIX_MJX = r"""
import numpy as np, jax, jax.numpy as jnp
from rl.env_mjx import OpenDuckEnv
out = {}
for gh in (None, "fixed"):
    e = OpenDuckEnv(task="getup", getup_head=gh, solver_iterations=16)
    st = jax.jit(jax.vmap(e.reset))(jax.random.split(jax.random.PRNGKey(0), 3))
    stp = jax.jit(jax.vmap(e.step))
    rng = np.random.default_rng(0)
    for k in range(4):
        st = stp(st, jnp.asarray(rng.uniform(-2, 2, (3, 14)), jnp.float32))
    la = np.asarray(st.info["a_prev"])
    out[str(gh)] = {"mode": e.getup_head, "head_ctrl": np.asarray(st.info["tr"]["ctrl_prev"])[:, e.idx.head_act_id].tolist(),
                    "la_head": float(np.abs(la[:, 10:14]).max()), "finite": bool(np.isfinite(np.asarray(st.obs["state"])).all()),
                    "res": e._act["pwm_res"][e.idx.head_act_id].tolist()}
print(json.dumps(out))
"""


def test_tr35_getup_head_fixed_mjx():
    """getup_head="fixed" (MJX, tr35_b2): CPU ile ayni anlam."""
    if not _has("jax"):
        import pytest
        pytest.skip("jax yok (MJX takimi)")
    scene, path = _boot()
    o = _run(_HEADFIX_MJX, tr35_env(scene, path, OPENDUCK_BANK_N=16, JAX_PLATFORMS="cpu"))
    tuck = np.deg2rad([C.load_derived(path)["head"]["tuck_deg"][j] for j in C.HEAD_JOINTS])
    fx, fr = o["fixed"], o["None"]
    res = np.array(fx["res"])
    assert fx["mode"] == "fixed" and fr["mode"] == "free" and fx["finite"] and fr["finite"]
    assert np.allclose(fx["head_ctrl"], np.round(tuck / res) * res, atol=1e-6) and fx["la_head"] == 0.0 and fr["la_head"] > 0.5


_LOCKSTEP = r"""
import numpy as np, jax, jax.numpy as jnp
from rl import tasks as TK, task_math as T
from rl.env_cpu import VecEnv
from rl.env_mjx import OpenDuckEnv
from rl.tests import test_env_mjx as TM
# MJX kutu-kutu temasi cift basina 4 nokta uretir, MuJoCo C 1-2 (stub bankalarinda uyluk/incik <-> govde ciftleri
# oturusta 12/32, dusmuste 17/32 durumda temasta): oz-carpisma ciftleriyle adim adim esitlik olmaz (P5a notlari, P5b).
# Bu test ciftsiz modelde geri kalan her seyi dogrular (turetilmis dosya, HOME, bankalar, tr35 odulleri, sensorler).
out = {"pairs_cfg": len(C.SELF_COLLISION_PAIRS)}
C.SELF_COLLISION_PAIRS = ()
n, steps = 4, 15
kw = dict(randomize=True, noise=True, head_random=False, push=False)
for task, rp in (("main", {"home": 0.5, "seated": 0.5}), ("getup", {"fallen": 0.5, "midfall": 0.25, "stand": 0.25})):
    spec = TK.get_task(task)
    menv = OpenDuckEnv(task=task, reset_probs=rp, solver_iterations=32, ls_iterations=16, final_weights=task == "getup", **kw)
    cenv = VecEnv(n, num_workers=0, task=task, reset_probs=rp, progress=1.0, **kw)
    core = cenv._core
    core.rngs = rngs = [TM._ParityRNG() for _ in range(n)]
    reset_u, step_u = TM._parity_draws_tr35(menv, task)
    keys = TM._keys(n, 51)
    for j, arrs in enumerate(zip(*(np.asarray(x) for x in reset_u(keys)))):
        for x in arrs:
            rngs[j].push(x)
    st = jax.jit(jax.vmap(menv.reset))(keys)
    obs_c, crit_c = cenv.reset()
    jstep = jax.jit(jax.vmap(menv.step))
    rng = np.random.default_rng(52)
    P = core.idx.pot
    sl_q, sl_v = spec.obs_slices["joint_pos"], spec.obs_slices["joint_vel"]
    lsb = float(P["lsb"].max())
    tol_q, tol_v = 2e-3 + lsb, 2e-3 + 2.0 * C.OBS_SCALES["joint_vel"] * P["alpha"] * lsb / C.SIM_DT
    rec = {"kinds": sorted(set(core.reset_kind.tolist())), "rew": 0.0, "terms": 0.0, "crit": 0.0, "obs_rest": 0.0,
           "dq": 0.0, "dv": 0.0, "done": 0, "tau_ratio": 0.0, "keys": list(spec.reward_keys)}
    for t in range(-1, steps):
        if t >= 0:
            a = (0.3 * rng.standard_normal((n, spec.num_actions)) * (np.arange(n) % 2)[:, None]).astype(np.float32)
            a[n - 1] *= 6.0
            draws = [np.asarray(x) for x in step_u(st.info["rng"])]
            for j in range(n):
                for x in draws:
                    rngs[j].push(x[j])
            sum0 = core.ep_sum.copy()
            obs_c, crit_c, rew_c, done_c, _ = cenv.step(a)
            for r_ in rngs:
                r_.q = {}
            st = jstep(st, jnp.asarray(a))
            assert np.array_equal(done_c, np.asarray(st.done) > 0.5), (task, t)
            rec["done"] += int(done_c.sum())
            rec["rew"] = max(rec["rew"], float(np.abs(rew_c - np.asarray(st.reward)).max()))
            live = ~done_c
            inc = core.ep_sum - sum0
            for i, k in enumerate(spec.reward_keys):
                m = np.asarray(st.metrics["rew_" + k])
                rec["terms"] = max(rec["terms"], float((np.abs(inc[live, i] - m[live]) / (1e-4 + 1e-3 * np.abs(m[live]))).max(initial=0.0)))
            s = cenv.task_state()
            tr = jax.tree_util.tree_map(np.asarray, st.info["tr"])
            rec["tau_ratio"] = max(rec["tau_ratio"], float(np.abs(s["tau_ratio"]).max()))
        om, cm = np.asarray(st.obs["state"]), np.asarray(st.obs["privileged_state"])
        rec["crit"] = max(rec["crit"], float(np.abs(crit_c - cm).max()))
        dq, dv = np.abs(obs_c[:, sl_q] - om[:, sl_q]).max(), np.abs(obs_c[:, sl_v] - om[:, sl_v]).max()
        rec["dq"], rec["dv"] = max(rec["dq"], float(dq / tol_q)), max(rec["dv"], float(dv / tol_v))
        rest = np.ones(spec.num_obs, bool)
        rest[sl_q] = rest[sl_v] = False
        rec["obs_rest"] = max(rec["obs_rest"], float(np.abs(obs_c[:, rest] - om[:, rest]).max()))
        qp, qv, qw = (np.stack([getattr(d, f) for d in core.datas]) for f in ("qpos", "qvel", "qacc_warmstart"))
        f32 = lambda x: jnp.asarray(x, jnp.float32)
        tr = dict(st.info["tr"], ctrl_prev=f32(core.ctrl_prev), pot={k: f32(v) for k, v in core.pot_st.items()},
                  imu={k: f32(v) for k, v in core.imu_st.items()})
        st = st.replace(data=st.data.replace(qpos=f32(qp), qvel=f32(qv), qacc_warmstart=f32(qw)), info=dict(st.info, tr=tr))
    cenv.close()
    out[task] = rec
print(json.dumps(out))
"""


def test_tr35_lockstep_cpu_mjx():
    """tr35_b2 surecinde kisa CPU == MJX adim adim (main, getup; n = 4, 15 adim; DR + gurultu, tr35 odulleri dahil):
    odul, her odul terimi, kritik, aktor gozlemi (pot dilimleri LSB toleransli). Oz-carpisma ciftleri kapali (kutu-kutu
    temas manifoldu MJX'te farkli; _LOCKSTEP notu)."""
    if not _has("jax"):
        import pytest
        pytest.skip("jax yok (MJX takimi)")
    scene, path = _boot()
    o = _run(_LOCKSTEP, tr35_env(scene, path, OPENDUCK_BANK_N=32, JAX_PLATFORMS="cpu"), timeout=2400)
    assert o["pairs_cfg"] == len(C.load_derived(path)["self_collision_pairs"])
    for task in ("main", "getup"):
        r = o[task]
        assert len(r["kinds"]) >= 2 and r["keys"][-2:] == ["limit_torque", "rail_demand"]
        assert ("torque_ratio" if task == "main" else "torque_sat") in r["keys"] and "torques" not in r["keys"]
        print("lockstep", task, {k: v for k, v in r.items() if k != "keys"})
        assert r["rew"] < 1e-3 and r["terms"] <= 1.0 and r["crit"] < 2e-3 and r["obs_rest"] < 2e-3, (task, r)
        assert r["dq"] < 1.0 and r["dv"] < 1.0 and r["tau_ratio"] > 0.1, (task, r)


def main():
    import time
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    sel = sys.argv[1:]
    failed = 0
    for name, fn in tests:
        if sel and not any(s in name for s in sel):
            continue
        t = time.time()
        try:
            fn()
            print("PASS  %-45s %6.1fs" % (name, time.time() - t), flush=True)
        except Exception:
            failed += 1
            import traceback
            traceback.print_exc()
            print("FAIL  %-45s %6.1fs" % (name, time.time() - t), flush=True)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
