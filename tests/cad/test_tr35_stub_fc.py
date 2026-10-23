# -*- coding: utf-8 -*-
"""
TR-35 yer tutucu derlemesi (P2-0; freecadcmd yoksa atlanir): OpenDuck(config="tr35_b2") kurulur, URDF + MJCF
disa aktarilir ve MuJoCo'da yuklenir (mujoco yoksa o testler atlanir). RL sozlesmesi: eklem/aktuator adlari ve
sirasi, sinif basina aktuator, freejoint "root", keyframe "stand", imu ve taban siteleri, <link>_collision
geomlari, "visual"/"collision" siniflari; A1: gerekli araliklar <= eklem sinirlari, isaret kurali +1, kafa ustu
stand keyframe'inde 350 +- 10 mm; kutle butcesi (1.95 kg / 1.05).

  pytest tests/cad -q -p no:cacheprovider
"""
import json
import math
import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import build_config  # noqa: E402
import golden  # noqa: E402
import servos  # noqa: E402

pytestmark = pytest.mark.slow

JOINTS = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
          "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
          "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth"]

SCRIPT = r'''
import json, os, sys
ROOT, OUT = %(root)r, %(out)r
sys.path.insert(0, os.path.join(ROOT, "cad"))
import openduck, export
m = openduck.OpenDuck(config="tr35_b2")
stand = m.pose("stand")
gz = m.ground_offset(stand)
sh = m.posed_shapes(stand)
head_top = max(s.BoundBox.ZMax for ln, it, s in sh if ln == "head") + gz
sole_z = min(s.BoundBox.ZMin for ln, it, s in sh if it.kind == "sole") + gz
data = export.export_robot(m, os.path.join(OUT, "sim"))
res = {"joints": [{k: j[k] for k in ("name", "parent", "child", "servo", "actuator", "lower", "upper", "stand", "id")}
                  for j in m.joints],
       "signs": dict(m.servo_signs()), "head_top": head_top, "sole_z": sole_z, "trunk_z": gz,
       "mass": m.total_mass(), "modules": m.meta["modules"], "imu_item": m.meta["imu_item"],
       "kinds": sorted({it.kind for l in m.links.values() for it in l.items}),
       "harness": sorted(it.name for l in m.links.values() for it in l.items if it.kind == "harness"),
       "link_mass": {ln: data[ln]["mass"] for ln in m.links}}
with open(os.path.join(OUT, "res.json"), "w") as f:
    json.dump(res, f, indent=1)
'''


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    fc = golden.find_freecadcmd()
    if fc is None:
        pytest.skip("freecadcmd bulunamadi (FREECADCMD=/yol/freecadcmd ile belirtin)")
    out = str(tmp_path_factory.mktemp("tr35_stub"))
    script = os.path.join(out, "run.py")
    with open(script, "w") as f:
        f.write(SCRIPT % {"root": ROOT, "out": out})
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    env.pop("OPENDUCK_CONFIG", None)
    log = os.path.join(out, "run.log")
    with open(log, "w") as lf:
        subprocess.run([fc, script], cwd=ROOT, env=env, stdout=lf, stderr=subprocess.STDOUT, timeout=900)
    path = os.path.join(out, "res.json")
    assert os.path.exists(path), open(log).read()[-4000:]
    with open(path) as f:
        return out, json.load(f)


def test_joints_order_ids_signs(built):
    _, r = built
    assert [j["name"] for j in r["joints"]] == JOINTS
    assert [j["id"] for j in r["joints"]] == [20, 21, 22, 23, 24, 10, 11, 12, 13, 14, 30, 31, 32, 33, 34]
    assert set(r["signs"].values()) == {1} and len(r["signs"]) == 15
    cfg = build_config.load("tr35_b2")
    for j in r["joints"]:
        assert j["servo"] == cfg["servo_map"][j["name"].replace("left_", "").replace("right_", "")]


def test_required_within_limits(built):
    """A1: spec 3.1 gerekli araliklar <= eklem sinirlari (sag bacak aynali), sinirlar servo hareketi icinde."""
    _, r = built
    cfg = build_config.load("tr35_b2")
    for j in r["joints"]:
        key = j["name"].replace("left_", "").replace("right_", "")
        lo, hi = cfg["required_deg"][key]
        if j["name"].startswith("right_"):
            lo, hi = -hi, -lo
        assert j["lower"] <= lo + 1e-9 and hi <= j["upper"] + 1e-9, j["name"]
    assert build_config.travel_violations(cfg, cfg["joint_limits_deg"]) == []


def test_head_top_and_stance(built):
    """A1: kafa ustu nominal stand keyframe'inde 350 +- 10 mm; tabanlar z = 0'da."""
    _, r = built
    lo, hi = build_config.load("tr35_b2")["targets"]["head_top_stand_mm"]
    assert lo <= r["head_top"] <= hi, r["head_top"]
    assert abs(r["sole_z"]) < 1e-6


def test_mass_budget_and_modules(built):
    _, r = built
    t = build_config.load("tr35_b2")["targets"]
    assert r["mass"] * t["mass_factor"] <= t["mass_max_kg"] + 1e-9, r["mass"]
    assert r["modules"]["trunk"] == ["trunk"]
    assert r["modules"]["head"] == ["neck", "head_tilt", "head_turn", "head", "jaw"]
    assert len(r["modules"]["legs"]) == 10
    assert r["imu_item"] == "imu_mpu6500"
    assert "harness" in r["kinds"] and r["harness"]


def test_mjcf_loads_rl_contract(built):
    mujoco = pytest.importorskip("mujoco")
    out, r = built
    path = os.path.join(out, "sim", "openduck.xml")
    xml = open(path).read()
    mm = mujoco.MjModel.from_xml_path(path)
    name = lambda t, i: mujoco.mj_id2name(mm, t, i)
    J = mujoco.mjtObj
    assert name(J.mjOBJ_JOINT, 0) == "root" and mm.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert [name(J.mjOBJ_JOINT, i) for i in range(1, mm.njnt)] == JOINTS
    assert [name(J.mjOBJ_ACTUATOR, i) for i in range(mm.nu)] == JOINTS
    for i, jn in enumerate(JOINTS):
        jid = mujoco.mj_name2id(mm, J.mjOBJ_JOINT, jn)
        j = r["joints"][i]
        assert tuple(mm.jnt_range[jid]) == pytest.approx((math.radians(j["lower"]), math.radians(j["upper"])), abs=1e-5)
        a = servos.actuator(servos.get(j["servo"]))
        assert mm.actuator_gainprm[i, 0] == pytest.approx(a["kp"])
        assert tuple(mm.actuator_forcerange[i]) == pytest.approx((-a["forcerange"], a["forcerange"]))
        assert tuple(mm.actuator_ctrlrange[i]) == pytest.approx(tuple(mm.jnt_range[jid]), abs=1e-5)
    for c in ("visual", "collision", "ds32xx", "es3054", "mg90s", "sg90"):
        assert '<default class="%s">' % c in xml
    for s in ("imu", "left_foot_sole", "right_foot_sole"):
        assert mujoco.mj_name2id(mm, J.mjOBJ_SITE, s) >= 0, s
    for g in ("trunk_collision", "head_collision", "left_foot_collision", "right_foot_collision"):
        assert mujoco.mj_name2id(mm, J.mjOBJ_GEOM, g) >= 0, g
    assert mujoco.mj_name2id(mm, J.mjOBJ_GEOM, "neck_collision") < 0   # RL getup varyanti ekler
    assert mm.nkey == 1 and name(J.mjOBJ_KEY, 0) == "stand"
    d = mujoco.MjData(mm)
    mujoco.mj_resetDataKeyframe(mm, d, 0)
    mujoco.mj_forward(mm, d)
    # stand keyframe'inde kafa carpisma kutusunun ustu = kafa ustu (350 +- 10 mm)
    gid = mujoco.mj_name2id(mm, J.mjOBJ_GEOM, "head_collision")
    top = d.geom_xpos[gid][2] + sum(abs(d.geom_xmat[gid].reshape(3, 3)[2, k]) * mm.geom_size[gid][k] for k in range(3))
    assert 0.340 <= top <= 0.360, top
    for _ in range(50):
        mujoco.mj_step(mm, d)
    assert all(math.isfinite(v) for v in d.qpos)
