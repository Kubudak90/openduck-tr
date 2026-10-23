# -*- coding: utf-8 -*-
"""
rl/pose_bank.py ve getup/main model varyantlarinin testleri. Calistirma (repo kokunden):

  python3 -m rl.tests.test_pose_bank
  python3 -m pytest rl/tests/test_pose_bank.py

Tam dusmus banka (4096) bir kez uretilir (~15 s) ve surec icinde onbellekte kalir.
"""
import collections
import sys
import time

import mujoco
import numpy as np

from rl import config as C
from rl import pose_bank as PB
from rl import task_math as T
from rl.robot_model import RobotIndex, build_model

_M = {}


def _model(variant, visual=False):
    k = (variant, visual)
    if k not in _M:
        _M[k] = build_model(visual=visual, variant=variant)
    return _M[k]


def _forward(m, qpos, ctrl=None):
    d = mujoco.MjData(m)
    d.qpos[:] = qpos
    if ctrl is not None:
        d.ctrl[:] = ctrl
    mujoco.mj_forward(m, d)
    return d


def _same(a, b):
    return all(np.array_equal(a[k], b[k]) for k in ("qpos", "qvel", "ctrl", "label"))


def test_determinism_and_prefix():
    for kind, n in (("fallen", 24), ("seated", 8), ("midfall", 16)):
        PB.clear_cache()
        a = PB.bank(kind, n, seed=7)
        PB.clear_cache()
        b = PB.bank(kind, n, seed=7)
        assert _same(a, b), kind
        assert PB.bank(kind, n, seed=7) is b                       # onbellek
        c = PB.bank(kind, n // 2, seed=7)
        assert all(np.array_equal(c[k], b[k][:n // 2]) for k in ("qpos", "qvel", "ctrl", "label")), kind
        d = PB.bank(kind, n, seed=8)
        assert not np.array_equal(d["qpos"], b["qpos"]), kind
        assert not b["qpos"].flags.writeable and b["variant"] == PB.KINDS[kind] and b["kind"] == kind
        m = _model(b["variant"])
        assert b["qpos"].shape == (n, m.nq) and b["qvel"].shape == (n, m.nv) and b["ctrl"].shape == (n, m.nu)
    PB.clear_cache()


def _digest(n_fallen=32, n_seated=8, n_mid=8):
    import hashlib
    h = hashlib.sha256()
    for kind, n in (("fallen", n_fallen), ("seated", n_seated), ("midfall", n_mid)):
        b = PB.bank(kind, n)
        for f in ("qpos", "qvel", "ctrl", "label"):
            h.update(np.ascontiguousarray(b[f]).tobytes())
    return h.hexdigest()


def test_same_bank_in_other_process():
    """CPU iscileri / MJX ayri sureclerde ayni bankayi uretir."""
    import os
    import subprocess
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    code = "from rl.tests.test_pose_bank import _digest; print(_digest())"
    env = dict(os.environ, MUJOCO_GL="disabled", PYTHONDONTWRITEBYTECODE="1")
    out = subprocess.run([sys.executable, "-c", code], cwd=root, env=env, capture_output=True, text=True,
                         timeout=300)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip().splitlines()[-1] == _digest()


def test_fallen_bank():
    b = PB.fallen_bank()
    n = C.POSE_BANK["fallen_n"]
    m = _model("getup")
    idx = RobotIndex(m)
    assert b["qpos"].shape == (n, m.nq) and np.all(b["qvel"] == 0) and b["variant"] == "getup"
    assert b["build_s"] < 60.0, b["build_s"]                     # olculen ~14 s (tek cekirdek)
    frac = {PB.LABELS[k]: v / n for k, v in collections.Counter(b["label"].tolist()).items()}
    print("fallen bank: %.1f s, %d aday, %s" % (b["build_s"], b["candidates"],
                                                 {k: round(v, 3) for k, v in sorted(frac.items())}))
    for k in ("back", "front", "left", "right"):
        assert 0.12 < frac.get(k, 0.0) < 0.42, frac                 # beklenen ~ 28/26/20/18 %
    assert sum(frac.get(k, 0.0) for k in ("back", "front", "left", "right")) > 0.97
    head = set(int(g) for g in idx.head_geoms)
    for i in range(n):
        q = b["qpos"][i]
        g = T.projected_gravity(np, q[3:7])
        if g[2] > C.POSE_BANK["inverted_gz"]:
            d = _forward(m, q, b["ctrl"][i])
            touch = PB.floor_contact_geoms(m, d, idx.floor_geom)
            assert touch and not touch <= head, (i, touch)          # "tepe ustu" atildi
    z = b["qpos"][:, 2]
    assert z.min() > 0.03 and np.median(z) < 0.07, (z.min(), np.median(z))
    lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
    assert np.all(b["ctrl"] >= lo - 1e-12) and np.all(b["ctrl"] <= hi + 1e-12)


def test_fallen_bank_penetration():
    """Kabul (spec 3.6): yatik/dusmus pozlarda gorsel mesh-zemin batmasi < 3 mm (ek geomlar sayesinde)."""
    b = PB.fallen_bank()
    mv = _model("getup", visual=True)
    dv = mujoco.MjData(mv)
    meshg = [g for g in range(mv.ngeom) if mv.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    assert len(meshg) > 10
    verts = {}
    for g in meshg:
        mid = mv.geom_dataid[g]
        verts[g] = mv.mesh_vert[mv.mesh_vertadr[mid]:mv.mesh_vertadr[mid] + mv.mesh_vertnum[mid]]
    pen = np.empty(len(b["qpos"]))
    for i, q in enumerate(b["qpos"]):
        dv.qpos[:] = q
        mujoco.mj_kinematics(mv, dv)
        pen[i] = -min((verts[g] @ dv.geom_xmat[g].reshape(3, 3)[2] + dv.geom_xpos[g][2]).min() for g in meshg)
    side = np.isin(b["label"], [PB.LABELS.index("left"), PB.LABELS.index("right")])
    print("mesh penetration mm: all max %.2f p99 %.2f | side max %.2f" % (
        1e3 * pen.max(), 1e3 * np.percentile(pen, 99), 1e3 * pen[side].max()))
    assert side.sum() > 0.3 * len(pen) and pen[side].max() < 0.003
    assert pen.max() < 0.003


def test_getup_model_floor_only():
    m = _model("getup")
    idx = RobotIndex(m)
    assert len(idx.extra_geoms) == len(C.EXTRA_COLLISION) == 8
    coll = [g for g in range(m.ngeom) if g != idx.floor_geom]
    feet = set(int(g) for g in idx.foot_geom)
    for i in coll:
        fl = idx.floor_geom
        assert m.geom_contype[fl] & m.geom_conaffinity[i] or m.geom_contype[i] & m.geom_conaffinity[fl]
        for j in coll:
            if i < j and not (i in feet and j in feet):
                pair = (m.geom_contype[i] & m.geom_conaffinity[j]) | (m.geom_contype[j] & m.geom_conaffinity[i])
                assert pair == 0, (m.geom(i).name, m.geom(j).name)
    for g in idx.extra_geoms:
        assert m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and m.geom_margin[g] == 0 and m.geom_gap[g] == 0
    # HOME'da ek geomlar zemine degmez (ayakta durus walk/main ile ayni)
    d = _forward(m, RobotIndex(m).init_qpos)
    touch = PB.floor_contact_geoms(m, d, idx.floor_geom, eps=1e-3)
    assert not touch & set(int(g) for g in idx.extra_geoms)


def test_seated_bank():
    b = PB.seated_bank()
    n = C.POSE_BANK["seated_n"]
    m = _model("main")
    idx = RobotIndex(m)
    assert b["qpos"].shape == (n, m.nq) and np.all(b["qvel"] == 0) and b["variant"] == "main"
    print("seated bank: %.1f s, %d aday" % (b["build_s"], b["candidates"]))
    feet = set(int(g) for g in idx.foot_geom)
    for i in range(0, n, 4):
        d = _forward(m, b["qpos"][i], b["ctrl"][i])
        touch = PB.floor_contact_geoms(m, d, idx.floor_geom)
        assert int(idx.trunk_geom) in touch and feet <= touch, (i, touch)       # govde yerde, ayaklar yerde
        assert not touch & set(int(g) for g in idx.head_geoms)
    g = T.projected_gravity(np, b["qpos"][:, 3:7])
    assert np.all(g[:, 2] < -0.9)                                              # dik
    pitch = np.degrees(np.arcsin(np.clip(g[:, 0], -1, 1)))
    assert np.all(np.abs(pitch) < C.SEAT_PITCH_TOL_DEG)
    z = b["qpos"][:, 2]
    assert abs(z.mean() - C.SEAT_Z) < C.SEAT_Z_TOL and np.all(np.abs(z - C.SEAT_Z) < 2 * C.SEAT_Z_TOL)
    # alpha = 1 hedefleriyle (sifir aksiyon) 1 s tutunca oturus korunur
    home = idx.default_obs_q
    ref = T.posture_ref(np, np.array(C.H_HOME), np.array(1.0), home)
    ctrl = idx.init_ctrl.copy()
    ctrl[idx.obs_act_id[:10]] = ref["q_ref"]
    ctrl[idx.head_act_id] = home[10:14] + ref["head_offset"]
    ctrl = np.clip(ctrl, idx.ctrl_lo, idx.ctrl_hi)
    for i in range(0, n, 64):
        d = _forward(m, b["qpos"][i], ctrl)
        p0 = d.xpos[idx.trunk_body].copy()
        mujoco.mj_step(m, d, int(1.0 / m.opt.timestep))
        mujoco.mj_forward(m, d)
        assert int(idx.trunk_geom) in PB.floor_contact_geoms(m, d, idx.floor_geom), i
        assert T.projected_gravity(np, d.qpos[3:7])[2] < -0.9
        assert np.linalg.norm(d.xpos[idx.trunk_body] - p0) < 0.01


def test_midfall_bank():
    b = PB.midfall_bank()
    m = _model("getup")
    idx = RobotIndex(m)
    n = C.POSE_BANK["midfall_n"]
    g = T.projected_gravity(np, b["qpos"][:, 3:7])
    tilt = np.degrees(np.arccos(np.clip(-g[:, 2], -1, 1)))
    lo, hi = C.GETUP["midfall"]["tilt_deg"]
    assert np.all((tilt > lo - 1e-6) & (tilt < hi + 1e-6))
    w = b["qvel"][:, 3:6]
    wn = np.linalg.norm(w, axis=1)
    assert np.all(wn <= C.GETUP["midfall"]["ang_vel"][1] + 1e-9) and wn.max() > 3.0
    assert np.all(np.abs(w[:, 2]) < 1e-12) and np.all(b["qvel"][:, :3] == 0) and np.all(b["qvel"][:, 6:] == 0)
    assert np.all(b["ctrl"] == idx.init_ctrl)
    frac = np.bincount(b["label"], minlength=6) / n
    assert np.all(np.abs(frac[:4] - 0.25) < 0.05) and frac[4:].sum() == 0
    for i in range(0, n, 32):                                          # zeminin hemen ustunde, temas yok
        d = _forward(m, b["qpos"][i])
        assert abs(PB.lowest_point(m, d) - 0.0005) < 1e-9
        assert not PB.floor_contact_geoms(m, d, idx.floor_geom)
    # devrilme yonune dogru donuyor: acisal hiz ekseni egimi artirir
    s = np.sign(np.einsum("ij,ij->i", w, np.stack([-g[:, 1], g[:, 0], np.zeros(n)], axis=1)))
    assert np.all(s[wn > 1e-3] > 0)


def test_check_variant():
    b = PB.bank("seated", 4, seed=3)
    PB.check_variant(b, "main")
    try:
        PB.check_variant(b, "getup")
        raise AssertionError
    except ValueError:
        pass
    try:
        PB.bank("standing")
        raise AssertionError
    except ValueError:
        pass


def main():
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    if len(sys.argv) > 1:
        tests = [(k, v) for k, v in tests if any(a in k for a in sys.argv[1:])]
    failed = 0
    for name, fn in tests:
        t = time.time()
        try:
            fn()
            print("PASS  %-40s %6.2fs" % (name, time.time() - t), flush=True)
        except Exception:
            failed += 1
            import traceback
            print("FAIL  %-40s %6.2fs" % (name, time.time() - t), flush=True)
            traceback.print_exc()
    print("%d/%d passed" % (len(tests) - failed, len(tests)))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
