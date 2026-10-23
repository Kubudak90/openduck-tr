# -*- coding: utf-8 -*-
"""
Egitim modeli: build/sim/scene.xml'den (CAD'den uretilen MJCF) turetilir.

- Gorsel mesh geomlar silinir (sadece kutu carpisma geomlari kalir) -> hizli fizik.
- Ayak kutulari birbirleriyle ve zeminle, govde/kafa kutulari yalnizca zeminle carpisir.
- Odul/gozlem icin sensorler eklenir: yerel dogrusal hiz (IMU), ayak tabani konum/hizlari.
- Fizik adimi config.SIM_DT olur.

Model varyantlari (variant=, gorev -> rl/tasks.py model_variant):
  "walk"  : yukaridaki model, hic degismeden (mevcut yurume gorevi).
  "main"  : walk + diz ust siniri KNEE_LIMIT_DEG (sol ust sinir, sag aynali alt sinir; eklem
            range'i ve aktuator ctrlrange'i) + neck_pitch ust siniri NECK_PITCH_MAX_DEG + ice kalca donmesi
            siniri HIP_YAW_INWARD_MAX_DEG + kafa hiz sensoru ("head_vel", kafa kutusu merkezi).
  "getup" : main + EXTRA_COLLISION geomlari (yalnizca zemin: contype 0, conaffinity 1; sinif
            "collision"). Sensor/eklem/aktuator sirasi her varyantta ayni; yeni sensorler sona eklenir.

Robot profili (robot=, varsayilan config.ROBOT_PROFILE / OPENDUCK_ROBOT; rl/actuators.py):
  "xl330_27": yukaridaki model aynen.
  "tr35_b2" : + servo sinifi parametreleri (actuators.apply_to_spec; kp, forcerange, sonum, armature, frictionloss),
              backlash=True ise DS eklemlerine seri "<eklem>_play" bosluk mafsali, ve EN SONDA degismez: aktuatorlu
              eklemlerde range = ctrlrange +- C.JOINT_RANGE_MARGIN_DEG (varyant sinirlayicilarindan sonra).
              Derlenmis modelde profil isareti (actuators.model_profile).

RobotIndex, her iki arka ucun (CPU/MJX) ham durumdan dizi cekmek icin kullandigi indeksleri toplar.
"""
import mujoco
import numpy as np

from . import actuators as A
from . import config as C
from . import sensors as SENS

VARIANTS = ("walk", "main", "getup")
HEAD_SITE = "head_center"
HEAD_VEL_SENSOR = "head_vel"


def _limit_knees(spec):
    """Diz bukulme siniri: sol knee ust siniri +K, sag knee alt siniri -K (aynali eksen)."""
    k = float(np.deg2rad(C.KNEE_LIMIT_DEG))
    for side, i in (("left", 1), ("right", 0)):
        name = side + "_knee"
        j, a = spec.joint(name), spec.actuator(name)
        r = np.array(j.range, dtype=float)
        r[i] = k if i == 1 else -k
        j.range = r
        cr = np.array(a.ctrlrange, dtype=float)
        cr[i] = r[i]
        a.ctrlrange = cr


def _limit_neck(spec):
    """neck_pitch ust siniri NECK_PITCH_MAX_DEG (eklem range'i ve aktuator ctrlrange'i)."""
    k = float(np.deg2rad(C.NECK_PITCH_MAX_DEG))
    j, a = spec.joint("neck_pitch"), spec.actuator("neck_pitch")
    r = np.array(j.range, dtype=float)
    r[1] = min(r[1], k)
    j.range = r
    cr = np.array(a.ctrlrange, dtype=float)
    cr[1] = min(cr[1], k)
    a.ctrlrange = cr


def _limit_hip_yaw(spec):
    """Ice kalca donmesi siniri HIP_YAW_INWARD_MAX_DEG: sol hip_yaw alt siniri -K, sag ust siniri +K (eklem
    range'i ve aktuator ctrlrange'i; dis yon degismez)."""
    k = float(np.deg2rad(C.HIP_YAW_INWARD_MAX_DEG))
    for side, i in (("left", 0), ("right", 1)):
        name = side + "_hip_yaw"
        j, a = spec.joint(name), spec.actuator(name)
        r = np.array(j.range, dtype=float)
        r[i] = max(r[i], -k) if i == 0 else min(r[i], k)
        j.range = r
        cr = np.array(a.ctrlrange, dtype=float)
        cr[i] = r[i]
        a.ctrlrange = cr


def _add_head_sensor(spec):
    g = spec.geom("head_collision")
    spec.body("head").add_site(name=HEAD_SITE, pos=np.array(g.pos, dtype=float))
    spec.add_sensor(name=HEAD_VEL_SENSOR, type=mujoco.mjtSensor.mjSENS_FRAMELINVEL,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname=HEAD_SITE)


def _quat_z_to(axis):
    """Kapsul ekseni (yerel z) -> govde ekseni 'x' | 'y' | 'z'."""
    h = np.sqrt(0.5)
    return {"x": [h, 0.0, h, 0.0], "y": [h, -h, 0.0, 0.0], "z": [1.0, 0.0, 0.0, 0.0]}[axis]


def add_extra_collision(spec, geoms=None):
    """Zemin-yalniz ek carpisma geomlari (getup modeli). geoms: {govde: (tip, merkez, boyut, eksen)},
    tip "box" (boyut = yari kenarlar) ya da "capsule" (boyut = (yaricap, yari uzunluk), eksen x/y/z)."""
    d = spec.find_default("collision")
    for body, (kind, pos, size, axis) in (C.EXTRA_COLLISION if geoms is None else geoms).items():
        if kind == "box":
            t, sz, q = mujoco.mjtGeom.mjGEOM_BOX, list(size), [1.0, 0.0, 0.0, 0.0]
        elif kind == "capsule":
            t, sz, q = mujoco.mjtGeom.mjGEOM_CAPSULE, [size[0], size[1], 0.0], _quat_z_to(axis)
        else:
            raise ValueError(kind)
        spec.body(body).add_geom(d, name=body + "_collision", type=t, size=sz, pos=list(pos), quat=q)


def build_spec(scene_path=None, visual=False, variant="walk", robot=None, backlash=None):
    if variant not in VARIANTS:
        raise ValueError("variant %r (%s)" % (variant, ", ".join(VARIANTS)))
    robot = A.profile(robot)
    spec = mujoco.MjSpec.from_file(scene_path or C.SCENE_PATH)
    if not visual:
        for g in list(spec.geoms):
            if g.group == 2:
                spec.delete(g)
        for m in list(spec.meshes):
            spec.delete(m)
    spec.option.timestep = C.SIM_DT
    for g in spec.geoms:
        if g.name in C.FOOT_GEOMS:
            g.contype, g.conaffinity = 2, 3
    S, O = mujoco.mjtSensor, mujoco.mjtObj
    spec.add_sensor(name="local_linvel", type=S.mjSENS_VELOCIMETER, objtype=O.mjOBJ_SITE, objname=C.IMU_SITE)
    for side, site in zip(("left", "right"), C.FOOT_SITES):
        spec.add_sensor(name=side + "_foot_pos", type=S.mjSENS_FRAMEPOS, objtype=O.mjOBJ_SITE, objname=site)
        spec.add_sensor(name=side + "_foot_vel", type=S.mjSENS_FRAMELINVEL, objtype=O.mjOBJ_SITE, objname=site)
    if variant != "walk":
        _limit_knees(spec)
        _limit_neck(spec)
        _limit_hip_yaw(spec)
        _add_head_sensor(spec)
    if variant == "getup":
        add_extra_collision(spec)
    if robot == A.TR35:
        A.apply_to_spec(spec, robot, bool(backlash))
        A.enforce_range_margin(spec)         # degismez: butun sinirlayicilardan sonra
    elif backlash:
        raise ValueError("bosluk eklemi yalniz %s profilinde" % A.TR35)
    return spec


def build_model(scene_path=None, visual=False, variant="walk", robot=None, backlash=None):
    return build_spec(scene_path, visual, variant, robot, backlash).compile()


def play_joints(model):
    """Bosluk eklemi kimlikleri ("<eklem>_play")."""
    return np.array([j for j in range(model.njnt) if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or "")
                     .endswith(A.PLAY_SUFFIX)], dtype=np.int64)


def expand_state(model, qpos=None, qvel=None):
    """Bosluk eklemsiz duzendeki qpos/qvel (or. poz bankalari) -> bu modelin duzeni (bosluk eklemleri 0). Boyut
    zaten uyuyorsa aynen. Dondurur (qpos, qvel)."""
    pj = play_joints(model)
    out = []
    for x, n, adr in ((qpos, model.nq, model.jnt_qposadr), (qvel, model.nv, model.jnt_dofadr)):
        if x is None:
            out.append(None)
            continue
        x = np.asarray(x)
        if x.shape[-1] == n or len(pj) == 0:
            out.append(x)
            continue
        keep = np.setdiff1d(np.arange(n), adr[pj])
        if x.shape[-1] != len(keep):
            raise ValueError("durum boyutu %d, model %d (bosluksuz %d)" % (x.shape[-1], n, len(keep)))
        y = np.zeros(x.shape[:-1] + (n,), dtype=x.dtype)
        y[..., keep] = x
        out.append(y)
    return tuple(out)


def model_variant(model):
    """Derlenmis modelin varyanti (ek geom / kafa sensoru / diz sinirindan)."""
    has = lambda t, n: mujoco.mj_name2id(model, t, n) >= 0
    if has(mujoco.mjtObj.mjOBJ_GEOM, next(iter(C.EXTRA_COLLISION)) + "_collision"):
        return "getup"
    if has(mujoco.mjtObj.mjOBJ_SENSOR, HEAD_VEL_SENSOR):
        return "main"
    return "walk"


class RobotIndex(object):
    """Model icindeki adresler ve varsayilan poz (numpy).

    action_joints: politikanin surdugu eklemler (varsayilan C.ACTION_JOINTS = 10 bacak; getup icin
    C.LEG_JOINTS + C.HEAD_JOINTS). default_action_q bu eklemlerin keyframe degerleri (ada gore)."""

    def __init__(self, model, action_joints=None):
        m = model
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        aid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
        self.obs_qpos_adr = np.array([m.jnt_qposadr[jid(n)] for n in C.OBS_JOINTS])
        self.obs_dof_adr = np.array([m.jnt_dofadr[jid(n)] for n in C.OBS_JOINTS])
        self.leg_dof_adr = np.array([m.jnt_dofadr[jid(n)] for n in C.LEG_JOINTS])
        self.action_joints = list(C.ACTION_JOINTS if action_joints is None else action_joints)
        for n in self.action_joints:
            if aid(n) < 0 or n not in C.OBS_JOINTS:
                raise ValueError("aksiyon eklemi %r gozlem eklemi/aktuator degil" % n)
        self.action_act_id = np.array([aid(n) for n in self.action_joints])
        self.action_qpos_adr = np.array([m.jnt_qposadr[jid(n)] for n in self.action_joints])
        self.action_obs_idx = np.array([C.OBS_JOINTS.index(n) for n in self.action_joints])
        self.head_act_id = np.array([aid(n) for n in C.HEAD_JOINTS])
        self.obs_act_id = np.array([aid(n) for n in C.OBS_JOINTS])
        self.hip_pose_idx = np.array([C.OBS_JOINTS.index(n) for n in C.HIP_POSE_JOINTS])

        def sensor(name):
            s = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if s < 0:
                return None
            return slice(m.sensor_adr[s], m.sensor_adr[s] + m.sensor_dim[s])
        self.s_quat = sensor("orientation")
        self.s_gyro = sensor("gyro")
        self.s_linvel = sensor("local_linvel")
        self.s_foot_pos = [sensor("left_foot_pos"), sensor("right_foot_pos")]
        self.s_foot_vel = [sensor("left_foot_vel"), sensor("right_foot_vel")]
        self.s_head_vel = sensor(HEAD_VEL_SENSOR)          # yalniz main/getup modelinde (yoksa None)
        self.s_acc = sensor("accelerometer")

        self.foot_geom = np.array([gid(n) for n in C.FOOT_GEOMS])
        self.floor_geom = gid(C.FLOOR_GEOM)
        self.trunk_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, C.TRUNK_BODY)
        self.foot_body = np.array([m.geom_bodyid[g] for g in self.foot_geom])
        # govde/kafa-zemin temas bayraklari (trunk_floor / head_floor) icin geom kumeleri
        self.trunk_geom = gid(C.TRUNK_GEOM)
        self.trunk_geoms = np.array([self.trunk_geom])
        self.head_geoms = np.array([g for g in (gid(n) for n in C.HEAD_GEOMS) if g >= 0])
        self.head_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, C.HEAD_BODY)
        self.extra_geoms = np.array([g for g in (gid(b + "_collision") for b in C.EXTRA_COLLISION) if g >= 0],
                                    dtype=np.int64)
        self.variant = model_variant(m)

        key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, C.DEFAULT_KEYFRAME)
        self.init_qpos = m.key_qpos[key].copy()
        self.init_ctrl = m.key_ctrl[key].copy()
        self.default_obs_q = self.init_qpos[self.obs_qpos_adr].copy()          # (14,)
        self.default_action_q = self.init_qpos[self.action_qpos_adr].copy()    # (10,) ya da (14,)
        self.ctrl_lo = m.actuator_ctrlrange[:, 0].copy()
        self.ctrl_hi = m.actuator_ctrlrange[:, 1].copy()
        self.action_lo = self.ctrl_lo[self.action_act_id].copy()
        self.action_hi = self.ctrl_hi[self.action_act_id].copy()
        self.head_range = np.array([C.HEAD_RANDOM["range"][n] for n in C.HEAD_JOINTS])

        # robot profili; tr35: aktuator tablosu (aktuator sirasi) ve gozlem eklemlerinin pot tablosu
        self.robot = A.model_profile(m)
        self.act = self.pot = None
        self.play_jnt = play_joints(m)
        if self.robot == A.TR35:
            self.act = A.actuator_table(m)
            cls = [A.joint_class(self.robot, n) for n in C.OBS_JOINTS]
            rails = [A.CLASSES[c]["rail"] for c in cls]
            self.pot = SENS.pot_table(C.OBS_JOINTS, cls, rails, [A.RAILS[r]["volt"] for r in rails])
