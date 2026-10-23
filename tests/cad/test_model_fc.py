# -*- coding: utf-8 -*-
"""
Model + disa aktarma (freecadcmd ile; yoksa atlanir): isaret kurali (+1, spec 3.2.1), eklem servo/aktuator
alanlari, eksik uretecleri adlandiran NotImplementedError, xl330/tr35 ureteclerinin olcek/servo korumasi ve
sinif basina MJCF aktuator varsayilanlari (eklemlerin servo turu degistirilmis xl330 modeli MuJoCo'da yuklenir).

  pytest tests/cad -q -p no:cacheprovider          (~15 s)
"""
import json
import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import golden  # noqa: E402
import servos  # noqa: E402

pytestmark = pytest.mark.slow

SCRIPT = r'''
import json, os, sys, traceback
ROOT, OUT = %(root)r, %(out)r
sys.path.insert(0, os.path.join(ROOT, "cad"))
import openduck, export
res = {}
m = openduck.OpenDuck()
res["config"] = m.cfg["name"]
res["signs"] = dict(m.servo_signs())
res["joints"] = [{k: j[k] for k in ("name", "servo", "actuator", "effort", "velocity", "neutral", "horn_offset", "id")}
                 for j in m.joints]
res["servo_label"] = m.meta["servo"]
res["scale"] = m.p["scale"]
tr = openduck.build_config.load("tr35_b2")
for tag, cfg in (("tr35", "tr35_b2"), ("xl_scaled", dict(openduck.build_config.load("xl330_27"), scale=1.282)),
                 ("missing", dict(tr, generators={"trunk": "yok", "legs": "yok", "head": "yok"})),
                 ("tr35_on_xl", dict(openduck.build_config.load("xl330_27"),
                                     generators={"trunk": "tr35_b2", "legs": "tr35_b2", "head": "tr35_b2"}))):
    try:
        openduck.OpenDuck(config=cfg)
        res[tag] = "hata yok"
    except NotImplementedError as e:
        res[tag] = "NotImplementedError: %%s" %% e
    except Exception:
        res[tag] = traceback.format_exc()
TR = {"hip_yaw": "ES3054", "hip_roll": "DS32XX", "hip_pitch": "DS32XX", "knee": "DS32XX", "ankle": "DS32XX",
      "neck_pitch": "DS32XX", "head_pitch": "MG90S", "head_yaw": "MG90S", "head_roll": "MG90S", "mouth": "SG90"}
for tag, remap in (("mixed", {"knee": "DS32XX", "mouth": "SG90"}), ("tr35like", TR)):
    for j in m.joints:   # yalniz disa aktarma sinif yolu: geometri XL330 kalir
        j["servo"] = remap.get(openduck.joint_key(j["name"]), "XL330")
    export.export_robot(m, os.path.join(OUT, tag))
with open(os.path.join(OUT, "res.json"), "w") as f:
    json.dump(res, f, indent=1)
'''


@pytest.fixture(scope="module")
def fc_run(tmp_path_factory):
    fc = golden.find_freecadcmd()
    if fc is None:
        pytest.skip("freecadcmd bulunamadi (FREECADCMD=/yol/freecadcmd ile belirtin)")
    out = str(tmp_path_factory.mktemp("model_fc"))
    script = os.path.join(out, "run.py")
    with open(script, "w") as f:
        f.write(SCRIPT % {"root": ROOT, "out": out})
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    env.pop("OPENDUCK_CONFIG", None)
    log = os.path.join(out, "run.log")
    with open(log, "w") as lf:
        subprocess.run([fc, script], cwd=ROOT, env=env, stdout=lf, stderr=subprocess.STDOUT, timeout=600)
    res = os.path.join(out, "res.json")
    assert os.path.exists(res), open(log).read()[-4000:]
    with open(res) as f:
        return out, json.load(f)


def test_sign_rule_and_joint_fields(fc_run):
    _, r = fc_run
    assert r["config"] == "xl330_27" and r["scale"] == 1.0
    assert len(r["signs"]) == 15 and set(r["signs"].values()) == {1}
    for j in r["joints"]:
        assert (j["servo"], j["actuator"], j["effort"], j["velocity"]) == ("XL330", "xl330", 0.96, 10.0)
        assert j["neutral"] == 0.0 and j["horn_offset"] == 0.0
    assert [j["id"] for j in r["joints"]] == [20, 21, 22, 23, 24, 10, 11, 12, 13, 14, 30, 31, 32, 33, 34]
    assert r["servo_label"] == "DYNAMIXEL XL330-M288-T"


def test_generator_guards(fc_run):
    """tr35_b2 P2-0 yer tutucularla kurulur; eksik uretec adi NotImplementedError'da adlandirilir; xl330 ureteci
    olcek korumali, tr35 ureteci servo turu korumali."""
    _, r = fc_run
    assert r["tr35"] == "hata yok", r["tr35"]
    msg = r["missing"]
    assert msg.startswith("NotImplementedError: konfig tr35_b2"), msg
    for stage in ("trunk", "legs", "head"):
        assert "%s.GENERATORS['yok']" % stage in msg
    assert r["xl_scaled"].startswith("NotImplementedError: trunk.xl330_27 1 olcekli"), r["xl_scaled"]
    assert r["tr35_on_xl"].startswith("NotImplementedError: trunk.tr35_b2 yalniz ES3054"), r["tr35_on_xl"]


@pytest.mark.parametrize("tag", ["mixed", "tr35like"])
def test_per_class_mjcf(fc_run, tag):
    mujoco = pytest.importorskip("mujoco")
    out, r = fc_run
    path = os.path.join(out, tag, "openduck.xml")
    xml = open(path).read()
    mm = mujoco.MjModel.from_xml_path(path)
    TR = {"hip_yaw": "ES3054", "hip_roll": "DS32XX", "hip_pitch": "DS32XX", "knee": "DS32XX", "ankle": "DS32XX",
          "neck_pitch": "DS32XX", "head_pitch": "MG90S", "head_yaw": "MG90S", "head_roll": "MG90S", "mouth": "SG90"}
    remap = {"knee": "DS32XX", "mouth": "SG90"} if tag == "mixed" else TR
    assert mm.nu == 15
    for j in r["joints"]:
        key = j["name"].replace("left_", "").replace("right_", "")
        a = servos.actuator(servos.get(remap.get(key, "XL330")))
        aid = mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_ACTUATOR, j["name"])
        jid = mujoco.mj_name2id(mm, mujoco.mjtObj.mjOBJ_JOINT, j["name"])
        dof = mm.jnt_dofadr[jid]
        assert mm.actuator_gainprm[aid, 0] == pytest.approx(a["kp"]), j["name"]
        assert tuple(mm.actuator_forcerange[aid]) == pytest.approx((-a["forcerange"], a["forcerange"]))
        assert mm.dof_damping[dof] == pytest.approx(a["damping"])
        assert mm.dof_armature[dof] == pytest.approx(a["armature"])
        assert mm.dof_frictionloss[dof] == pytest.approx(a["frictionloss"])
    used = {servos.actuator(servos.get(v))["mjcf_class"] for v in remap.values()}
    for c in used:
        assert '<default class="%s">' % c in xml
    if tag == "tr35like":     # kok sinifta aktuator degeri yok; 4 sinif
        assert used == {"ds32xx", "es3054", "mg90s", "sg90"}
        assert '<default class="openduck">\n      <default class="visual">' in xml
