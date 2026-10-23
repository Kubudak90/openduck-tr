# -*- coding: utf-8 -*-
"""
tr35_b2 test yardimcilari (OpenDuck-TR P5a; rl/TASK.md "Robot profiles").

  scene = tr35_scene()                          # $OPENDUCK_TR35_SCENE ya da P2-0 stub'i (surec basina bir kez)
  path = bootstrap_derived(scene, out_path)     # YALNIZ TEST: MJCF'den sema-1 turetilmis dosya
  env = tr35_env(scene, derived)                # tr35 alt sureci icin ortam degiskenleri

bootstrap_derived, tools/derive_posture.py (derive paketi) birlesmeden once cekirdegin tr35 yolunu sinamak icindir:
source.tool = "bootstrap", stub = true; varsayilan yolda HIC yuklenmez. Kurallar: H_HOME = keyframe govde z - 1 mm,
diger yukseklikler xl330 oranlariyla, comelme ailesi xl330 acilari (h * trunk_z_kin / 0.1183, ayak bilegi HOME'un duz
taban degismezine oturtulur), oturus xl330 SEAT acilari + MuJoCo C oturtmasi (pitch, SEAT_Z), bas ve sinirlar xl330
tablolari ile MJCF araliklari, ayaklar taban sitelerinden, EXTRA kutulari govde cercevesinde gorsel mesh sinir
kutulari (xl330 govdeleri + hip_cradle), oz-carpisma ciftleri yok.
"""
import hashlib
import importlib.util
import json
import math
import os
import tempfile

import numpy as np

_CACHE = {}
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
S = 1.282                       # Froude boy orani (spec 3.2.5)
XL330_KIN_H = 0.1183            # xl330 HOME kinematik govde yuksekligi (config.CROUCH_FAMILY notu)
# EXTRA kutulari: xl330 govde kumesi + hip_cradle (TR-35'te ayri govde; xl330 kutulari x S stub'da kalca besigi
# mesh'ini 20 mm'ye kadar zemine sokuyordu), sinir kutusu yontemiyle
EXTRA_BODIES = tuple("%s_%s" % (s, b) for b in ("thigh", "shin", "hip", "hip_cradle") for s in ("left", "right")) + \
    ("neck", "jaw")


def _golden():
    if "golden" not in _CACHE:
        spec = importlib.util.spec_from_file_location("_openduck_golden", os.path.join(ROOT, "tools", "golden.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _CACHE["golden"] = mod
    return _CACHE["golden"]


def tr35_scene():
    """$OPENDUCK_TR35_SCENE if set; otherwise build the P2-0 stub once per process into a temp dir with tools/golden.py
    (freecadcmd, no check); no freecadcmd -> pytest.skip."""
    env = os.environ.get("OPENDUCK_TR35_SCENE")
    if env:
        return env
    if "scene" not in _CACHE:
        g = _golden()
        if g.find_freecadcmd() is None:
            import pytest
            pytest.skip("freecadcmd yok (tr35 stub derlenemez; OPENDUCK_TR35_SCENE ile verin)")
        out = tempfile.mkdtemp(prefix="tr35_stub_")
        g.build(out, config="tr35_b2", check=False)
        _CACHE["scene"] = os.path.join(out, "sim", "scene.xml")
    return _CACHE["scene"]


def tr35_env(scene, derived=None, **extra):
    """tr35_b2 alt sureci icin ortam: OPENDUCK_ROBOT, OPENDUCK_SCENE, (OPENDUCK_DERIVED), PYTHONDONTWRITEBYTECODE."""
    env = dict(os.environ, OPENDUCK_ROBOT="tr35_b2", OPENDUCK_SCENE=scene, PYTHONDONTWRITEBYTECODE="1")
    if derived:
        env["OPENDUCK_DERIVED"] = derived
    env.update({k: str(v) for k, v in extra.items()})
    return env


def _sha(path):
    if not os.path.isfile(path):
        return None
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def _corners(m, d, g):
    s, R = m.geom_size[g], d.geom_xmat[g].reshape(3, 3)
    return np.array([d.geom_xpos[g] + R @ (s * np.array([a, b, c])) for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])


def _mesh_boxes(m, d, bodies, clear=0.001):
    """Govde cercevesinde gorsel mesh (grup 2) koselerinin eksen hizali sinir kutulari {govde: ("box", merkez, yari,
    None)} (S5 extra_boxes yontemi); mesh'i olmayan govde atlanir. d: HOME kinematigi. Egik govdede (incik) kutu kosesi
    HOME'da mesh'ten asagi sarkarsa, en alttaki yuz govdenin o mesh'inin en alt noktasinin clear altina cekilir (getup
    modelinde HOME'da EXTRA kutulari zemine degmesin)."""
    import mujoco
    out = {}
    for b in bodies:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
        pts = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != bid or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or bid < 0:
                continue
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, m.geom_quat[g])
            pts.append(v @ R.reshape(3, 3).T + m.geom_pos[g])
        if not pts:
            continue
        p = np.vstack(pts)
        lo, hi = p.min(0), p.max(0)
        c, s = 0.5 * (lo + hi), 0.5 * (hi - lo)
        Rb = d.xmat[bid].reshape(3, 3)
        zmin = float((p @ Rb[2]).min() + d.xpos[bid, 2]) - clear          # mesh'in HOME'daki en alt noktasi
        for _ in range(3):
            low = d.xpos[bid, 2] + Rb[2] @ c - np.abs(Rb[2]) @ s
            if low >= zmin - 1e-9:
                break
            i = int(np.argmax(np.abs(Rb[2]) * s))
            t = min((zmin - low) / abs(Rb[2, i]), 1.8 * s[i])                 # alt yuzu t kadar iceri al
            s[i] -= 0.5 * t
            c[i] += np.sign(Rb[2, i]) * 0.5 * t
        out[b] = ["box", [float(v) for v in c], [float(v) for v in s], None]
    return out


def _seat_settle(scene, seat_deg, head_deg):
    """xl330 SEAT acilariyla MuJoCo C oturtmasi (tr35 aktuatorleri, main modeli, 2.5 s): ilk temasli (govde + iki ayak)
    sonuc -> (pitch_deg (+ burun asagi), govde z) ya da None."""
    import mujoco
    from rl import task_math as T
    from rl.pose_bank import axis_angle_quat, floor_contact_geoms, lowest_point
    from rl.robot_model import RobotIndex, build_model
    m = build_model(scene, variant="main", robot="tr35_b2")
    d, idx = mujoco.MjData(m), RobotIndex(m)
    aq = np.array([m.jnt_qposadr[m.actuator_trnid[a, 0]] for a in range(m.nu)])
    sl = np.deg2rad([seat_deg[j] for j in ("hip_pitch", "knee", "ankle")])
    ctrl = idx.init_ctrl.copy()
    ctrl[idx.obs_act_id[[2, 3, 4]]], ctrl[idx.obs_act_id[[7, 8, 9]]] = sl, -sl
    ctrl[idx.head_act_id] = np.deg2rad([head_deg[j] for j in ("neck_pitch", "head_pitch", "head_yaw", "head_roll")])
    ctrl = np.clip(ctrl, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
    for p0 in (0.0, -10.0, 10.0, -20.0, 20.0):
        mujoco.mj_resetDataKeyframe(m, d, 0)
        d.qpos[aq], d.qpos[0:2] = ctrl, 0.0
        d.qpos[3:7] = axis_angle_quat((0, 1, 0), np.deg2rad(p0))
        mujoco.mj_kinematics(m, d)
        d.qpos[2] += 0.001 - lowest_point(m, d)
        d.ctrl[:] = ctrl
        mujoco.mj_forward(m, d)
        mujoco.mj_step(m, d, int(round(2.5 / m.opt.timestep)))
        mujoco.mj_forward(m, d)
        touch = floor_contact_geoms(m, d, idx.floor_geom)
        if idx.trunk_geom in touch and set(int(g) for g in idx.foot_geom) <= touch:
            g = T.projected_gravity(np, d.qpos[3:7])
            return float(np.rad2deg(np.arcsin(np.clip(g[0], -1.0, 1.0)))), float(d.xpos[idx.trunk_body, 2])
    return None


def bootstrap_derived(scene, out_path):
    """TEST-ONLY schema-1 derived file computed from the MJCF (source.tool 'bootstrap', stub true). Writes out_path
    (deterministic JSON) and returns it. Never loaded by default."""
    import mujoco
    from rl import config as C
    x = C._xl330                                               # xl330 tablolari (profilden bagimsiz)
    m = mujoco.MjModel.from_xml_path(scene)
    d = mujoco.MjData(m)
    name = lambda t, n: mujoco.mj_name2id(m, t, n)
    J, A_, G, B = mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco.mjtObj.mjOBJ_GEOM, mujoco.mjtObj.mjOBJ_BODY
    key = name(mujoco.mjtObj.mjOBJ_KEY, C.DEFAULT_KEYFRAME)
    qpos, ctrl = m.key_qpos[key].copy(), m.key_ctrl[key].copy()
    deg = lambda j: float(np.rad2deg(qpos[m.jnt_qposadr[name(J, j)]]))
    legs = {j: deg("left_" + j) for j in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")}
    head = {j: deg(j) for j in C.HEAD_JOINTS}
    d.qpos[:] = qpos
    d.qpos[0:2] = 0.0
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)
    feet = np.vstack([_corners(m, d, name(G, g)) for g in C.FOOT_GEOMS])
    z0 = feet[:, 2].min()
    bottom = feet[feet[:, 2] < z0 + 1e-3]
    tb = name(B, C.TRUNK_BODY)
    trunk_z_kin = float(d.xpos[tb, 2] - z0)
    com_x, (fx0, fx1) = float(d.subtree_com[tb, 0]), (float(bottom[:, 0].min()), float(bottom[:, 0].max()))
    sites = [d.site_xpos[name(mujoco.mjtObj.mjOBJ_SITE, s)] for s in C.FOOT_SITES]
    lateral = float(abs(sites[0][1] - sites[1][1]))

    H = float(qpos[2]) - 0.001
    r = lambda n: x[n] / x["H_HOME"] * H                       # xl330 oranlari
    hz = {"H_HOME": H, "H_MIN_WALK": r("H_MIN_WALK"), "H_MIN_STAND": r("H_MIN_STAND"), "H_DEEP": r("H_DEEP"),
          "TERMINATION_MIN_HEIGHT": r("TERMINATION_MIN_HEIGHT"), "SEAT_Z_TOL": 0.004,
          "standup_z": x["MAIN"]["standup_z"] / x["H_HOME"] * H, "getup_success_z": x["GETUP"]["success"]["z"] / x["H_HOME"] * H,
          "getup_pose_z": [v / x["H_HOME"] * H for v in x["GETUP"]["pose_z"]],
          "getup_arrival_z": [v / x["H_HOME"] * H for v in x["GETUP"]["arrival_z"]]}
    cr = lambda a, i: round(float(np.rad2deg(m.actuator_ctrlrange[name(A_, a), i])), 3)
    head_sit = {j: float(np.clip(v, cr(j, 0), cr(j, 1))) for j, v in x["HEAD_SIT_DEG"].items()}
    seat = _seat_settle(scene, x["SEAT_DEG"], head_sit)
    pitch, seat_z = seat if seat is not None else (0.0, x["SEAT_Z"] / x["H_HOME"] * H)
    hz["SEAT_Z"], hz["Z_SIT"] = seat_z, seat_z + (x["Z_SIT"] - x["SEAT_Z"])
    flat = legs["hip_pitch"] - legs["knee"] + legs["ankle"]
    fam = [[h * trunk_z_kin / XL330_KIN_H, p, hp, kn, flat - p - hp + kn] for h, p, hp, kn, _ in x["CROUCH_FAMILY"]]
    tilt = x["POSE_BANK"]["seated_tilt_deg"]
    out = {
        "schema": 1, "profile": "tr35_b2", "tool": "bootstrap", "tool_version": 1,
        "source": {"scene": scene, "mjcf_sha256": _sha(os.path.join(os.path.dirname(scene), "openduck.xml")),
                   "config_sha256": _sha(os.path.join(ROOT, "configs", "tr35_b2.json")), "stub": True},
        "params": {"height_ratios": "xl330", "crouch_scale": trunk_z_kin / XL330_KIN_H, "seat_from": "xl330 SEAT_DEG"},
        "mass": {"total_kg": float(m.body_subtreemass[tb])},
        "home": {"source": "keyframe", "qpos": [float(v) for v in qpos], "ctrl": [float(v) for v in ctrl],
                 "legs_deg": legs, "head_deg": head, "trunk_z_kin": trunk_z_kin,
                 "com_offset_x": com_x - 0.5 * (fx0 + fx1), "com_margin": min(com_x - fx0, fx1 - com_x)},
        "heights": hz,
        "crouch_family": fam,
        "seat": {"deg": dict(x["SEAT_DEG"]), "pitch_deg": round(pitch, 1),
                 "seat_gz": -math.cos(math.radians(abs(pitch) + 2.0 * tilt)), "feasible": seat is not None},
        "head": {"home_deg": head, "sit_deg": head_sit, "tuck_deg": dict(head_sit)},
        "feet": {"home_lateral": lateral, "FEET_MIN_DISTANCE": lateral * x["FEET_MIN_DISTANCE"] / 0.083},
        "caps_deg": {"KNEE_LIMIT_DEG": min(x["KNEE_LIMIT_DEG"], cr("left_knee", 1)),
                     "NECK_PITCH_MAX_DEG": min(x["NECK_PITCH_MAX_DEG"], cr("neck_pitch", 1)),
                     "HIP_YAW_INWARD_MAX_DEG": min(x["HIP_YAW_INWARD_MAX_DEG"], -cr("left_hip_yaw", 0))},
        "extra_collision": _mesh_boxes(m, d, EXTRA_BODIES),
        "self_collision_pairs": [],
        "report": {"com_x": com_x, "feet_x": [fx0, fx1], "seat_settle": seat},
    }
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(out, f, indent=1, sort_keys=True)
        f.write("\n")
    return out_path


def bootstrap_for(scene, out_dir=None):
    """bootstrap_derived'in surec ici onbellekli hali: (scene, json yolu)."""
    k = ("boot", scene)
    if k not in _CACHE:
        out = os.path.join(out_dir or tempfile.mkdtemp(prefix="tr35_boot_"), "tr35_b2.bootstrap.json")
        _CACHE[k] = bootstrap_derived(scene, out)
    return _CACHE[k]
