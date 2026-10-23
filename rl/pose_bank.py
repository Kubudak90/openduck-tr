# -*- coding: utf-8 -*-
"""
Deterministik baslangic poz bankalari (spec 3.5). Yalniz MuJoCo C + numpy; sabit tohumla, ortam
kurulurken bir kez uretilir ve surec icinde onbellege alinir. Ayni (tur, n, tohum) her cagirana
(CPU iscileri, MJX) ayni diziyi verir; adaylar tek bir RNG akisindan SIRAYLA uretilir ve her aday
mj_resetData ile bagimsiz baslar -> n'den bagimsiz onek kararliligi: bank(n=64) == bank(n=4096)[:64].

  b = seated_bank()     # "main" modeli: SEAT/HEAD_SIT hedefleriyle oturtulmus (alpha = 1) durumlar
  b = fallen_bank()     # "getup" modeli: rastgele yonelimden dusurulup oturtulmus durumlar
  b = midfall_bank()    # "getup" modeli: HOME'dan 15-35 derece devrik, 0-4 rad/s, oturtmasiz
  b = bank("fallen")    # ada gore

Donen sozluk (diziler salt okunur): qpos (N,nq), qvel (N,nv), ctrl (N,nu), label (N,) int32
(LABELS indeksi), labels, kind, variant, seed, build_s, candidates (denenen aday sayisi).
Sifirlamada (TASK.md): yaw rastgele quat_mul(yaw_q, qpos[3:7]), xy = 0, z = qpos[2], qvel = bank qvel
(seated/fallen: 0), ctrl = bank ctrl.
"""
import functools
import time

import mujoco
import numpy as np

from . import config as C
from . import task_math as T
from .robot_model import RobotIndex, build_model

LABELS = ("back", "front", "left", "right", "upright", "inverted")
KINDS = {"seated": "main", "fallen": "getup", "midfall": "getup"}     # tur -> model varyanti
_TAGS = {"seated": 1, "fallen": 2, "midfall": 3}
_P = C.POSE_BANK


def classify(g):
    """Govde cercevesi yercekimi (3,) -> LABELS indeksi: baskin eksen (back: govde -x asagi, front: +x,
    left: +y, right: -y, upright: -z, inverted: +z)."""
    k = int(np.argmax(np.abs(g)))
    if k == 2:
        return 4 if g[2] < 0 else 5
    if k == 0:
        return 0 if g[0] < 0 else 1
    return 2 if g[1] > 0 else 3


def tilt_label(g):
    """Yatay bilesene gore (back/front/left/right) - devrik ama dik duran durumlar icin."""
    return (0 if g[0] < 0 else 1) if abs(g[0]) >= abs(g[1]) else (2 if g[1] > 0 else 3)


def random_quat(rng):
    """Duzgun rastgele birim kuaterniyon [w,x,y,z] (Shoemake)."""
    u1, u2, u3 = rng.random(3)
    a, b = np.sqrt(1.0 - u1), np.sqrt(u1)
    return np.array([b * np.cos(2 * np.pi * u3), a * np.sin(2 * np.pi * u2), a * np.cos(2 * np.pi * u2),
                     b * np.sin(2 * np.pi * u3)])


def axis_angle_quat(axis, ang):
    axis = np.asarray(axis, dtype=np.float64)
    return np.r_[np.cos(0.5 * ang), np.sin(0.5 * ang) * axis / np.linalg.norm(axis)]


def lowest_point(m, d):
    """Carpisan (zemin disi) kutu/kapsul geomlarin en alcak dunya z'si (kinematik guncel olmali)."""
    z = np.inf
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == 0 or (m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0):
            continue
        R, s, t = d.geom_xmat[g].reshape(3, 3), m.geom_size[g], m.geom_type[g]
        if t == mujoco.mjtGeom.mjGEOM_BOX:
            z = min(z, d.geom_xpos[g][2] - np.abs(R[2]) @ s)
        elif t == mujoco.mjtGeom.mjGEOM_CAPSULE:
            z = min(z, d.geom_xpos[g][2] - abs(R[2, 2]) * s[1] - s[0])
        elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
            z = min(z, d.geom_xpos[g][2] - s[0])
    return z


def floor_contact_geoms(m, d, floor_geom, eps=C.CONTACT_EPS):
    """Zemine degen geom kimlikleri (dist < eps)."""
    out = set()
    for i in range(d.ncon):
        c = d.contact[i]
        if c.dist >= eps:
            continue
        g0, g1 = int(c.geom[0]), int(c.geom[1])
        if g0 == floor_geom:
            out.add(g1)
        elif g1 == floor_geom:
            out.add(g0)
    return out


class _Builder(object):
    def __init__(self, variant):
        self.m = m = build_model(variant=variant)
        self.d = mujoco.MjData(m)
        self.idx = RobotIndex(m)
        self.key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, C.DEFAULT_KEYFRAME)
        self.act_qadr = np.array([m.jnt_qposadr[m.actuator_trnid[a, 0]] for a in range(m.nu)])
        self.n_settle = int(round(_P["settle_s"] / m.opt.timestep))

    def place(self, quat, ctrl, clearance):
        """Eklemler = ctrl, yonelim quat, en alcak carpisma noktasi zeminden clearance yukarida; qvel = 0."""
        m, d = self.m, self.d
        mujoco.mj_resetDataKeyframe(m, d, self.key)
        d.qpos[self.act_qadr] = ctrl
        d.qpos[0:2] = 0.0
        d.qpos[3:7] = quat / np.linalg.norm(quat)
        mujoco.mj_kinematics(m, d)
        d.qpos[2] += clearance - lowest_point(m, d)
        d.qvel[:] = 0.0
        d.ctrl[:] = ctrl
        mujoco.mj_forward(m, d)

    def settle(self):
        m, d = self.m, self.d
        mujoco.mj_step(m, d, self.n_settle)
        mujoco.mj_forward(m, d)
        return bool(np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))
                    and d.time > (self.n_settle - 0.5) * m.opt.timestep)

    def gravity(self):
        return np.asarray(T.projected_gravity(np, self.d.qpos[3:7]))


def _finish(kind, variant, seed, qpos, qvel, ctrl, label, t0, tried):
    out = {"qpos": np.array(qpos), "qvel": np.array(qvel), "ctrl": np.array(ctrl),
           "label": np.array(label, dtype=np.int32)}
    for v in out.values():
        v.setflags(write=False)
    out.update(kind=kind, variant=variant, seed=int(seed), labels=LABELS, build_s=time.time() - t0,
               candidates=int(tried))
    return out


@functools.lru_cache(maxsize=None)
def _fallen(n, seed):
    t0 = time.time()
    b = _Builder("getup")
    m, d, idx = b.m, b.d, b.idx
    rng = np.random.default_rng([seed, _TAGS["fallen"]])
    lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
    head = set(int(g) for g in idx.head_geoms)
    nz = _P["fallen_joint_noise"]
    Q, V, U, L = [], [], [], []
    tried = 0
    while len(Q) < n:
        tried += 1
        ctrl = idx.init_ctrl.copy()
        ctrl[idx.obs_act_id] += rng.uniform(-nz, nz, len(idx.obs_act_id))
        ctrl = np.clip(ctrl, lo, hi)
        quat = random_quat(rng)
        drop = rng.uniform(*_P["drop"])
        b.place(quat, ctrl, drop)
        if not b.settle():
            continue
        g = b.gravity()
        touch = floor_contact_geoms(m, d, idx.floor_geom)
        if g[2] > _P["inverted_gz"] and touch and touch <= head:
            continue                                  # kafa ustunde dengede ("tepe ustu", kutu artefakti)
        Q.append(d.qpos.copy())
        V.append(np.zeros(m.nv))
        U.append(ctrl)
        L.append(classify(g))
    return _finish("fallen", "getup", seed, Q, V, U, L, t0, tried)


@functools.lru_cache(maxsize=None)
def _seated(n, seed):
    t0 = time.time()
    b = _Builder("main")
    m, d, idx = b.m, b.d, b.idx
    rng = np.random.default_rng([seed, _TAGS["seated"]])
    lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
    home = idx.default_obs_q
    ref = T.posture_ref(np, np.array(C.H_HOME), np.array(1.0), home)       # alpha = 1: SEAT + HEAD_SIT
    target = idx.init_ctrl.copy()
    target[idx.obs_act_id[:10]] = ref["q_ref"]
    target[idx.head_act_id] = home[10:14] + ref["head_offset"]
    sig, tilt = _P["seated_joint_sigma"], np.deg2rad(_P["seated_tilt_deg"])
    trunk, feet = int(idx.trunk_geom), set(int(g) for g in idx.foot_geom)
    Q, V, U, L = [], [], [], []
    tried = 0
    while len(Q) < n:
        tried += 1
        ctrl = target.copy()
        ctrl[idx.obs_act_id] += sig * rng.standard_normal(len(idx.obs_act_id))
        ctrl = np.clip(ctrl, lo, hi)
        pitch, roll = rng.uniform(-tilt, tilt, 2)
        quat = T.quat_mul(np, axis_angle_quat((0, 1, 0), pitch), axis_angle_quat((1, 0, 0), roll))
        b.place(quat, ctrl, 0.001)
        if not b.settle():
            continue
        g = b.gravity()
        touch = floor_contact_geoms(m, d, idx.floor_geom)
        if not (trunk in touch and feet <= touch and g[2] < C.MAIN["seat_gz"]):
            continue
        Q.append(d.qpos.copy())
        V.append(np.zeros(m.nv))
        U.append(ctrl)
        L.append(4)
    return _finish("seated", "main", seed, Q, V, U, L, t0, tried)


@functools.lru_cache(maxsize=None)
def _midfall(n, seed):
    t0 = time.time()
    b = _Builder("getup")
    m, d, idx = b.m, b.d, b.idx
    rng = np.random.default_rng([seed, _TAGS["midfall"]])
    lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
    mf = C.GETUP["midfall"]
    fj = int(np.flatnonzero(m.jnt_type == mujoco.mjtJoint.mjJNT_FREE)[0])
    dadr = int(m.jnt_dofadr[fj])
    Q, V, U, L = [], [], [], []
    tried = 0
    while len(Q) < n:
        tried += 1
        q = idx.init_ctrl.copy()
        q[idx.obs_act_id] += rng.uniform(-C.RESET_JOINT_NOISE, C.RESET_JOINT_NOISE, len(idx.obs_act_id))
        q = np.clip(q, lo, hi)
        psi = rng.uniform(0.0, 2.0 * np.pi)
        ang = np.deg2rad(rng.uniform(*mf["tilt_deg"]))
        w = rng.uniform(*mf["ang_vel"])
        axis = np.array([np.cos(psi), np.sin(psi), 0.0])
        b.place(axis_angle_quat(axis, ang), q, 0.0005)
        qv = np.zeros(m.nv)
        qv[dadr + 3:dadr + 6] = w * axis          # serbest eklem acisal hizi govde cercevesinde; eksen donuste sabit
        Q.append(d.qpos.copy())
        V.append(qv)
        U.append(idx.init_ctrl.copy())
        L.append(tilt_label(b.gravity()))
    return _finish("midfall", "getup", seed, Q, V, U, L, t0, tried)


_BUILD = {"seated": _seated, "fallen": _fallen, "midfall": _midfall}


def bank(kind, n=None, seed=None):
    """kind: "seated" | "fallen" | "midfall". n/seed varsayilanlari C.POSE_BANK."""
    if kind not in _BUILD:
        raise ValueError("bilinmeyen banka %r (%s)" % (kind, ", ".join(_BUILD)))
    n = int(_P[kind + "_n"] if n is None else n)
    return _BUILD[kind](n, int(_P["seed"] if seed is None else seed))


def seated_bank(n=None, seed=None):
    return bank("seated", n, seed)


def fallen_bank(n=None, seed=None):
    return bank("fallen", n, seed)


def midfall_bank(n=None, seed=None):
    return bank("midfall", n, seed)


def clear_cache():
    for f in _BUILD.values():
        f.cache_clear()


def check_variant(b, variant):
    """Banka/model uyusmazligi (spec 5.10): banka hangi varyantla uretildiyse o modelde kullanilir."""
    if b["variant"] != variant:
        raise ValueError("%s bankasi %s modeliyle uretildi, %s ile kullanilamaz" % (b["kind"], b["variant"], variant))
