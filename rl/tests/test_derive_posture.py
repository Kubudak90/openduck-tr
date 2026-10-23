# -*- coding: utf-8 -*-
"""
tools/derive_posture.py ve rl/derived/tr35_b2.json testleri (OpenDuck-TR P5a; ortak arayuz 3, sema 1).
Calistirma (repo kokunden):

  python3 -m pytest rl/tests/test_derive_posture.py

tr35 sahnesi: $OPENDUCK_TR35_SCENE, yoksa P2-0 yer tutucusu surec basina bir kez tools/golden.py (freecadcmd, check
yok) ile gecici dizine derlenir; freecadcmd yoksa tr35 testleri atlanir.
  (a) stub'da derive (CLI, alt surec, OPENDUCK_ROBOT yok) + check (OPENDUCK_ROBOT=tr35_b2) cikis 0 / 3 / 1, sema ve iliskiler
  (b) aractan bagimsiz MuJoCo denetimi (test_crouch_family_mujoco tarzi): satirlar, ofset formu, oturus, HOME, H_HOME
  (c) xl330 regresyonu: family3 parametreleriyle aile portu build/sim/scene.xml'de C.CROUCH_FAMILY'yi 3 derece icinde
      (COM payi > 25 mm) uretir; EXTRA kutu yontemi C.EXTRA_COLLISION'u verir
  (d) depodaki JSON yuklenir, gecerli ve belirlenimci bicimde; stub degismediyse taze turetmeyle bayt bayt ayni
"""
import importlib.util
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

import mujoco
import numpy as np
import pytest

from rl import actuators as A
from rl import config as C

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TOOL = os.path.join(ROOT, "tools", "derive_posture.py")
COMMITTED = os.path.join(ROOT, "rl", "derived", "tr35_b2.json")
XL_SCENE = os.path.join(ROOT, "build", "sim", "scene.xml")
S = 1.282
HEAD = ("neck_pitch", "head_pitch", "head_yaw", "head_roll")
_SGN = np.array([(a, b, c) for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], dtype=np.float64)


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_MOD, _SCENE, _RUN = {}, {}, {}


def _dp():
    if "dp" not in _MOD:
        _MOD["dp"] = _load("derive_posture_t", TOOL)
    return _MOD["dp"]


def tr35_scene():
    """$OPENDUCK_TR35_SCENE if set; otherwise build the P2-0 stub once per process into a temp dir with tools/golden.py
    (freecadcmd, no check); no freecadcmd -> pytest.skip."""
    if "scene" not in _SCENE:
        env = os.environ.get("OPENDUCK_TR35_SCENE")
        if env:
            _SCENE.update(scene=env, stub=False)
        else:
            golden = _load("golden_tr35_t", os.path.join(ROOT, "tools", "golden.py"))
            if golden.find_freecadcmd() is None:
                pytest.skip("freecadcmd yok: tr35 stub derlenemiyor")
            out = tempfile.mkdtemp(prefix="tr35_stub_")
            golden.build(out, config="tr35_b2", check=False)
            _SCENE.update(scene=os.path.join(out, "sim", "scene.xml"), stub=True)
    return _SCENE["scene"]


def _env(robot=None):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    env.pop("OPENDUCK_ROBOT", None)
    if robot:
        env["OPENDUCK_ROBOT"] = robot
    return env


def _derived():
    """tr35 sahnesinden bir kez (CLI) gecici dizine turetilmis JSON."""
    if "doc" not in _RUN:
        scene = tr35_scene()
        out = os.path.join(tempfile.mkdtemp(prefix="tr35_derived_"), "tr35_b2.json")
        cmd = [sys.executable, TOOL, "derive", "--scene", scene, "--out", out] + (["--stub"] if _SCENE["stub"] else [])
        r = subprocess.run(cmd, cwd=ROOT, env=_env(), capture_output=True, text=True, timeout=600)
        assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-3000:]
        with open(out) as f:
            text = f.read()
        _RUN.update(scene=scene, out=out, text=text, doc=json.loads(text), log=r.stdout)
    return _RUN


def _check(scene, derived, robot=None):
    r = subprocess.run([sys.executable, TOOL, "check", "--scene", scene, "--derived", derived], cwd=ROOT,
                       env=_env(robot), capture_output=True, text=True, timeout=300)
    return r.returncode, r.stdout + r.stderr


# --- aractan bagimsiz kinematik (test_task_math._kin_pose tarzi) ---------------------------------------
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
    n, best, inside = len(poly), np.inf, True
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        e = b - a
        dd = np.dot(p - a, np.array([e[1], -e[0]]) / np.linalg.norm(e))
        inside &= dd <= 0
        best = min(best, abs(dd))
    return best if inside else -best


class _Rig(object):
    """RL tarzi carpisma modeli (gorsel geomlar silinir, EXTRA kutulari + JSON ciftleri, zemin kapali)."""

    def __init__(self, scene, doc, actuators=False, floor=False):
        spec = mujoco.MjSpec.from_file(scene)
        for g in list(spec.geoms):
            if g.group == 2:
                spec.delete(g)
        for me in list(spec.meshes):
            spec.delete(me)
        spec.option.timestep = C.SIM_DT
        for g in spec.geoms:
            if g.name in C.FOOT_GEOMS:
                g.contype, g.conaffinity = 2, 3
            elif g.name == C.FLOOR_GEOM and not floor:
                g.contype = g.conaffinity = 0
        dc = spec.find_default("collision")
        for b, (kind, pos, size, _ax) in doc["extra_collision"].items():
            assert kind == "box"
            spec.body(b).add_geom(dc, name=b + "_collision", type=mujoco.mjtGeom.mjGEOM_BOX, size=size, pos=pos)
        for a, b in doc["self_collision_pairs"]:
            spec.add_pair(geomname1=a, geomname2=b)
        if actuators:
            A.apply_to_spec(spec, "tr35_b2")
            A.enforce_range_margin(spec)
        self.m = m = spec.compile()
        self.d = mujoco.MjData(m)
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        self.gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
        self.qa = {n: m.jnt_qposadr[jid(n)] for n in C.OBS_JOINTS}
        self.da = np.array([m.jnt_dofadr[jid(n)] for n in HEAD])
        self.foot = [self.gid(n) for n in C.FOOT_GEOMS]
        self.trunk = self.gid(C.TRUNK_GEOM)
        self.tb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, C.TRUNK_BODY)
        key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, C.DEFAULT_KEYFRAME)
        self.key_qpos = m.key_qpos[key].copy()
        self.tau_s = np.array([A.CLASSES[A.JOINT_CLASS["tr35_b2"][j]]["tau_s"] for j in HEAD])

    def corners(self, g):
        return self.d.geom_xpos[g] + (_SGN * self.m.geom_size[g]) @ self.d.geom_xmat[g].reshape(3, 3).T

    def kin(self, base, pitch, legs3, head4=None):
        """-> dict h, clear (govde kutusu - ayak duzlemi), trunk_low, margin, tilt (derece), com_off, contacts, static."""
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        d.qpos[:] = base
        d.qpos[0:3] = 0.0
        th = math.radians(pitch)
        d.qpos[3:7] = [math.cos(th / 2), 0, math.sin(th / 2), 0]
        for j, v in zip(("hip_pitch", "knee", "ankle"), legs3):
            d.qpos[self.qa["left_" + j]], d.qpos[self.qa["right_" + j]] = math.radians(v), -math.radians(v)
        if head4 is not None:
            for j, v in zip(HEAD, head4):
                d.qpos[self.qa[j]] = math.radians(v)
        mujoco.mj_forward(m, d)
        fc = np.vstack([self.corners(g) for g in self.foot])
        z0 = fc[:, 2].min()
        bottom = fc[fc[:, 2] < z0 + 1e-3]
        tz = self.corners(self.trunk)[:, 2].min()
        com = d.subtree_com[self.tb][:2]
        con = [(m.geom(d.contact[i].geom1).name, m.geom(d.contact[i].geom2).name) for i in range(d.ncon)
               if d.contact[i].dist < 0]
        return {"h": -z0, "clear": tz - z0, "trunk_low": tz, "margin": _margin(_hull(bottom[:, :2]), com),
                "tilt": max(math.degrees(math.acos(min(1.0, d.geom_xmat[g][8]))) for g in self.foot),
                "com_off": com[0] - 0.5 * (bottom[:, 0].min() + bottom[:, 0].max()), "contacts": con,
                "static": np.abs(d.qfrc_bias[self.da]) / self.tau_s}


# --- (a) derive + check -------------------------------------------------------------------------------
def test_derive_and_check_stub():
    r = _derived()
    d, dp = r["doc"], _dp()
    assert dp.validate(d) == [] and d["report"]["errors"] == []
    assert r["text"] == dp.dumps(d) and r["text"].endswith("}\n")          # sirali anahtarlar, indent 1, son satir
    assert d["schema"] == 1 and d["profile"] == "tr35_b2" and d["tool"] == "tools/derive_posture.py"
    assert d["source"]["scene"] == r["scene"] and d["source"]["stub"] is _SCENE["stub"]
    mj = os.path.join(os.path.dirname(r["scene"]), "openduck.xml")
    assert d["source"]["mjcf_sha256"] == dp.sha256_file(mj)
    # HOME kurali (p2-interface 8 butcesi)
    lo, hi = d["params"]["home_com_x_budget"]
    off = d["report"]["keyframe"]["com_offset_x"]
    assert d["home"]["source"] == ("keyframe" if lo <= off <= hi else "family")
    if d["home"]["source"] == "family":
        assert lo <= d["home"]["com_offset_x"] <= hi
    # yukseklik eslemesi (arayuz 3) ve oranlar
    H, P, rows = d["heights"], d["params"], d["crouch_family"]
    hh, k = H["H_HOME"], P["ratios"]
    assert abs(hh * 1e4 - round(hh * 1e4)) < 1e-6
    assert H["H_DEEP"] == pytest.approx(max(k["H_DEEP"] * hh, rows[-1][0]), abs=1e-6)
    assert H["H_MIN_STAND"] == pytest.approx(max(k["H_MIN_STAND"] * hh, H["H_DEEP"]), abs=1e-6)
    assert H["TERMINATION_MIN_HEIGHT"] == pytest.approx(k["TERMINATION_MIN_HEIGHT"] * hh, abs=1e-6)
    assert H["getup_pose_z"] == pytest.approx([x * hh for x in k["getup_pose_z"]], abs=1e-6)
    assert H["Z_SIT"] == pytest.approx(H["SEAT_Z"] + 0.002 * S, abs=1e-6)
    assert H["SEAT_Z_TOL"] == pytest.approx(0.003 * S, abs=1e-9)
    assert d["seat"]["seat_gz"] == pytest.approx(-math.cos(math.radians(abs(d["seat"]["pitch_deg"]) + 25.84)), abs=1e-6)
    # satir yukseklikleri: round_mm(z_kin) + 0.005*S*(1-k), en az 3, ilk >= H_HOME
    h0 = round(d["home"]["trunk_z_kin"], 3)
    assert [x[0] for x in rows] == pytest.approx([h0 + 0.005 * S * (1 - i) for i in range(len(rows))], abs=1e-6)
    assert len(rows) >= 3 and rows[0][0] >= hh and rows[-1][0] <= H["H_DEEP"]
    assert d["head"]["tuck_deg"] == d["head"]["sit_deg"] and d["head"]["home_deg"] == d["home"]["head_deg"]
    assert d["feet"]["FEET_MIN_DISTANCE"] == pytest.approx(0.72 * d["feet"]["home_lateral"], abs=1e-6)
    # sinirlar ve kutle ham MJCF'den
    vis = mujoco.MjSpec.from_file(r["scene"]).compile()
    cr = lambda n: np.degrees(vis.actuator_ctrlrange[mujoco.mj_name2id(vis, mujoco.mjtObj.mjOBJ_ACTUATOR, n)])
    cp = d["caps_deg"]
    assert cp["KNEE_LIMIT_DEG"] == pytest.approx(cr("left_knee")[1], abs=1e-3)
    assert cp["NECK_PITCH_MAX_DEG"] == pytest.approx(cr("neck_pitch")[1], abs=1e-3)
    assert cp["HIP_YAW_INWARD_MAX_DEG"] == pytest.approx(-cr("left_hip_yaw")[0], abs=1e-3)
    assert d["mass"]["total_kg"] == pytest.approx(float(vis.body_mass.sum()), abs=1e-6)
    # check: taze -> 0 (OPENDUCK_ROBOT=tr35_b2 ortaminda da: arac profilden bagimsiz)
    code, out = _check(r["scene"], r["out"], robot="tr35_b2")
    assert code == 0, out
    tmp = tempfile.mkdtemp(prefix="tr35_check_")
    shutil.copy(r["scene"], os.path.join(tmp, "scene.xml"))
    with open(mj) as f, open(os.path.join(tmp, "openduck.xml"), "w") as g:
        g.write(f.read() + "<!-- degisti -->\n")
    code, out = _check(os.path.join(tmp, "scene.xml"), r["out"])
    assert code == 3, out
    bad = dict(d)
    del bad["seat"]
    with open(os.path.join(tmp, "bad.json"), "w") as f:
        json.dump(bad, f)
    code, out = _check(r["scene"], os.path.join(tmp, "bad.json"))
    assert code == 1, out
    # ikinci (surec ici) turetme bayt bayt ayni
    doc2, errs = dp.derive(r["scene"], stub=_SCENE["stub"])
    assert errs == [] and dp.dumps(doc2) == r["text"]


# --- (b) bagimsiz MuJoCo denetimi -------------------------------------------------------------------------
def test_independent_mujoco_recheck():
    r = _derived()
    d = r["doc"]
    rig = _Rig(r["scene"], d)
    P, H, hm = d["params"], d["heights"], d["home"]
    L_f = float(rig.m.geom_size[rig.foot[0]][0])
    base = np.array(hm["qpos"])
    lg = hm["legs_deg"]
    head0 = [hm["head_deg"][j] for j in HEAD]
    f0 = lg["hip_pitch"] - lg["knee"] + lg["ankle"]
    # HOME: qpos/ctrl/derece tutarli, z kurali, COM
    for side, sg in (("left", 1.0), ("right", -1.0)):
        for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle"):
            assert math.degrees(base[rig.qa[side + "_" + j]]) == pytest.approx(sg * lg[j], abs=1e-9)
    assert np.allclose(hm["ctrl"][:14], [base[rig.qa[n]] for n in C.OBS_JOINTS], atol=1e-12)
    key = rig.key_qpos
    kd = {j: math.degrees(key[rig.qa["left_" + j]]) for j in ("hip_pitch", "knee", "ankle")}
    kk = rig.kin(key, 0.0, (kd["hip_pitch"], kd["knee"], kd["ankle"]))
    hk = rig.kin(base, 0.0, (lg["hip_pitch"], lg["knee"], lg["ankle"]), head0)
    assert base[2] - hk["h"] == pytest.approx(key[2] - kk["h"], abs=1e-6)
    assert hk["h"] == pytest.approx(hm["trunk_z_kin"], abs=1e-6)
    assert hk["com_off"] == pytest.approx(hm["com_offset_x"], abs=1e-6) and not hk["contacts"]
    assert kk["com_off"] == pytest.approx(d["report"]["keyframe"]["com_offset_x"], abs=1e-6)
    # satirlar
    for h, th, hp, kn, an in d["crouch_family"]:
        x = rig.kin(base, th, (hp, kn, an), head0)
        assert abs(x["h"] - h) <= 0.0015 * S, (h, x["h"])
        assert x["margin"] >= 0.75 * L_f - 1e-9 and x["clear"] >= 0.005 * S - 1e-9, (h, x)
        assert abs(th + hp - kn + an - f0) < 1e-6 and x["tilt"] < 0.5 and not x["contacts"], (h, x)
    # ofset formu q_ref = HOME + F(h) - F(H_HOME) (task_math.crouch_offset)
    F = np.array(d["crouch_family"])[::-1]
    fi = lambda z, c: float(np.interp(z, F[:, 0], F[:, c]))
    for z in (H["H_HOME"], 0.8696 * H["H_HOME"], H["H_MIN_WALK"], H["H_MIN_STAND"], H["H_DEEP"]):
        q = [lg[j] + fi(z, c) - fi(H["H_HOME"], c) for j, c in (("hip_pitch", 2), ("knee", 3), ("ankle", 4))]
        x = rig.kin(base, fi(z, 1), q, head0)
        assert -1e-6 <= x["h"] - z <= 0.01 * S and x["margin"] >= 0.6 * L_f, (z, x)
    # oturus
    se, sh = d["seat"], [d["head"]["sit_deg"][j] for j in HEAD]
    x = rig.kin(base, se["pitch_deg"], (se["deg"]["hip_pitch"], se["deg"]["knee"], se["deg"]["ankle"]), sh)
    assert abs(se["pitch_deg"] + se["deg"]["hip_pitch"] - se["deg"]["knee"] + se["deg"]["ankle"] - f0) < 1e-6
    assert not x["contacts"] and x["static"].max() <= 0.3 + 1e-9, x
    if d["report"]["seat"]["mode"] == "trunk_floor":
        assert abs(x["clear"]) <= 0.001 * S + 1e-9 and -x["trunk_low"] == pytest.approx(H["SEAT_Z"], abs=1e-6)
        assert se["feasible"] == (x["margin"] > 0)
    else:
        assert not se["feasible"] and x["h"] == pytest.approx(H["SEAT_Z"], abs=1e-6)
    # H_HOME: 2 s oturtma (nominal tr35 aktuatorleri, zemin acik)
    sim = _Rig(r["scene"], d, actuators=True, floor=True)
    sd = sim.d
    mujoco.mj_resetData(sim.m, sd)
    sd.qpos[:] = base
    sd.ctrl[:] = hm["ctrl"]
    for _ in range(int(round(2.0 / C.SIM_DT))):
        mujoco.mj_step(sim.m, sd)
    assert round(sd.qpos[2], 4) == pytest.approx(H["H_HOME"], abs=1.5e-4)
    assert hm["trunk_z_kin"] - H["H_HOME"] >= -1e-6                      # oturtma kinematik yuksekligi asmaz


# --- (c) xl330 regresyonu ------------------------------------------------------------------------------------
def test_family3_port_xl330():
    if C.ROBOT_PROFILE != "xl330_27":
        pytest.skip("xl330 sabitleri yalniz xl330_27 surecinde")
    if not os.path.exists(XL_SCENE):
        pytest.skip("build/sim/scene.xml yok")
    dp = _dp()
    R = dp.Model(XL_SCENE, actuators=False, floor=False)
    kd = R.deg(R.key_qpos)
    foot0 = kd["hip_pitch"] - kd["knee"] + kd["ankle"]
    # family3: kn <= 60, hp <= 85, |an| <= 70, pitch -30..10, h 0.120..0.065, 1 mm, 5 mm aciklik
    recs = dp.scan(R, dp.grid(-30, 10, 2.5), dp.grid(-90, 85, 1.5), dp.grid(-90, 60, 1.5), (-70, 70), foot0,
                   keep=lambda g: g["clear"] >= 0.005)
    hs = [round(0.120 - 0.005 * i, 3) for i in range(12)]
    rows = dict(dp.family(R, recs, hs, 0.001, 0.005, collide=False, stop=False, tie="first"))
    for h, th, hp, kn, an in C.CROUCH_FAMILY:
        g = rows[h]
        assert np.abs(np.array([g["th"], g["hp"], g["kn"], g["an"]]) - [th, hp, kn, an]).max() <= 3.0 + 1e-9, (h, g)
        assert g["margin"] > 0.025, (h, g["margin"])
    ex = dp.extra_boxes(dp.visual_model(XL_SCENE))
    for b, (kind, pos, size, ax) in C.EXTRA_COLLISION.items():
        assert ex[b][0] == kind and ex[b][3] is ax
        assert np.allclose(ex[b][1], pos, atol=1.5e-5) and np.allclose(ex[b][2], size, atol=1.5e-5), (b, ex[b])


def test_tool_profile_independent():
    src = open(TOOL).read()
    allowed_c = {"SIM_DT", "LEG_JOINTS", "HEAD_JOINTS", "OTHER_JOINTS", "OBS_JOINTS", "ACTION_JOINTS", "FOOT_GEOMS",
                 "FOOT_SITES", "FLOOR_GEOM", "IMU_SITE", "TRUNK_BODY", "TRUNK_GEOM", "HEAD_BODY", "DEFAULT_KEYFRAME",
                 "JOINT_RANGE_MARGIN_DEG", "ROBOT_PROFILES", "CROUCH_JOINTS"}
    assert set(re.findall(r"\bC\.([A-Za-z_]\w*)", src)) <= allowed_c
    assert set(re.findall(r"\bA\.([A-Za-z_]\w*)", src)) <= {"apply_to_spec", "enforce_range_margin", "CLASSES",
                                                            "JOINT_CLASS"}
    assert not re.search(r"(from|import)\s+[\w.]*robot_model", src)


# --- (d) depodaki JSON -------------------------------------------------------------------------------------------
def test_committed_json():
    dp = _dp()
    with open(COMMITTED) as f:
        text = f.read()
    doc = json.loads(text)
    assert dp.validate(doc) == [] and doc["report"]["errors"] == []
    assert text == dp.dumps(doc)
    ld = getattr(C, "load_derived", None)                 # core (P5a) birlesince: config'in kendi dogrulamasi
    if ld is not None:
        ld(COMMITTED)
    r = _derived()
    fresh = r["doc"]
    if (fresh["source"]["mjcf_sha256"], fresh["report"]["meshes_sha256"]) != \
            (doc["source"]["mjcf_sha256"], doc["report"]["meshes_sha256"]):
        pytest.skip("stub/MJCF yukari akista degisti (sha farkli): rl/derived/tr35_b2.json yeniden uretilmeli: "
                    "python tools/derive_posture.py derive --scene <sahne> --out rl/derived/tr35_b2.json [--stub]")
    sc = lambda s: '"scene": %s' % json.dumps(s)
    assert r["text"].count(sc(r["scene"])) == 1
    assert r["text"].replace(sc(r["scene"]), sc(doc["source"]["scene"])) == text
