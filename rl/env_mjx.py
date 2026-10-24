# -*- coding: utf-8 -*-
"""
GPU arka ucu: MJX (JAX) ortami, MuJoCo Playground tarzinda.

Sozlesme: rl/TASK.md. Gozlem/odul/sonlandirma rl/task_math.py'den (xp=jax.numpy) gelir.

  env = OpenDuckEnv(randomize=True, noise=True, head_random=True, push=True, fixed_command=None)
  state = env.reset(rng)              # tek ortam; toplu kullanim icin jax.vmap ya da VmapAutoReset
  state = env.step(state, action)     # action (10,); biten ortam ayni adimda sifirlanir
  state.obs = {"state": (49,), "privileged_state": (58,)}

Gorevler (task=, rl/tasks.py; sozlesme rl/TASK.md "Tasks main and getup"):
  "walk"  (varsayilan) yukaridaki davranis, BIT-DUZEYINDE degismeden (gozlem, RNG akislari, info alanlari).
  "main"  yurume + govde yuksekligi + otur/kalk: komut 5 ([vx, vy, wz, h (m), sit]), gozlem 53 / kritik 64,
          aksiyon 10 (q_ref(h_ref, alpha) + 0.5 clip(a)). info: command (5), cmd_lock, seg, seg_timer, h_ref,
          alpha, ev_* (main_events), reset_kind (0 HOME, 1 oturus bankasi). set_command 3 ya da 5 boyut.
  "getup" dusunce kalkma: aksiyon 14 (bacak + bas, clip(HOME + S_j clip(a, +-1), ctrlrange)), gozlem 48 /
          kritik 57, 400 adim, dusme sonlandirmasi yok (yalniz bozuk fizik). Sifirlama karisimi: dusmus banka /
          dusus ortasi bankasi / HOME (reset_kind 0/1/2, bank_label PB.LABELS). info["gstep"] (int32) ortam
          basina toplam adim, bolumler arasi korunur -> mufredat ilerlemesi p = clip(gstep / curriculum_steps,
          0, 1) (curriculum_steps = toplam egitim adimi / ortam sayisi); final_weights=True -> p = 1 (eval).
  Ortak: reset_probs={"home": 1.0} gibi sifirlama karisimi degisikligi (oynatma/degerlendirme). Poz bankalari
  (rl/pose_bank.py) kurucuda MuJoCo C ile bir kez uretilir ve sabit olarak gomulur; sifirlamada yalniz indeks
  secilir (fizik yok), bankanin kuaterniyonu walk'un yaw cekilisiyle dondurulur. Yeni rastgele sayilar
  fold_in ile alinir (walk akislari ayni kalir).

Notlar:
- Alan rastgelelestirme HER BOLUMDE nominal degerlerden yeniden cekilir; ornekler state.info["dr"]
  icinde tasinir ve adimda mjx.Model alanlarina yazilir. mj_setConst'un MJX'te kullanilan ciktilari
  (body_invweight0, dof_invweight0, dof_M0, body_subtreemass, stat.meaninertia) qpos0'da JAX ile
  ayni formullerle yeniden hesaplanir (C ile makine hassasiyetinde ayni, bkz. testler).
- MJX (impl="jax") modelin tum ozelliklerini destekliyor (frictionloss, eklem limitleri, forcerange,
  implicitfast, gyro/framequat/velocimeter/framepos/framelinvel). MuJoCo C'den iki fark bulundu ve
  burada C ile ayni olacak sekilde duzeltildi (c_semantics=True, varsayilan):
  (1) implicitfast: C, forcerange'e dayanmis aktuatorun hiz turevini (kv) atlar
      (engine_derivative.c mjd_actuatorDerivSkip), MJX atlamaz -> physics_step() bu aktuatorlerin
      biasprm[2]/gainprm[2]'sini yalnizca integrasyon icin sifirlar.
  (2) duzlem-kutu temasi: MJX en derin kosenin 1 mm yakinindaki koseleri alir, C alt yaridaki giren
      ilk 4 koseyi alir (mjc_PlaneBox) -> MJX carpisma tablosuna C'nin algoritmasi yazilir
      (install_c_plane_box; surec genelinde gecerli, margin/gap 0 olmali).
  Bunlarla float64'te zeminde sert aksiyonlarla 50 alt adim C ile 1e-14 icinde ayni. Kalan farklar:
  float32; ayak-ayak (kutu-kutu) temasi MJX'in kendi algoritmasiyla; Newton cozucu yineleme ust
  siniri 100/50 yerine 4/8 (C ayakta ortalama 1.1, sert hareketlerde en cok ~5 yineleme yapiyor;
  MJX'te 100/50 ayni sonucu ~2.5 kat yavas veriyor).
- Temas BAYRAGI (ayak-zemin temas noktasi, dist < CONTACT_EPS) MJX temas yuvalarindan okunur;
  c_semantics=False iken de C ile ayni (giren kose varsa MJX de en az birini secer).
- Brax ile egitim: wrap_for_brax_training() (ppo.train(wrap_env_fn=...)). EpisodeWrapper/
  AutoResetWrapper kullanilmaz; bolum sayaci, truncation ve episode_metrics ortamin icindedir.
- GPU: JAX, Ampere ve sonrasi GPU'larda float32 matris carpimlarini varsayilan olarak TF32 ile yapar
  (~1e-3 bagil hata). Ortamin reset/step izlemesi matmul_precision="highest" (varsayilan) altinda
  yapilir: fizik CPU'daki gibi tam float32; politika agi JAX varsayilaninda kalir.
- Robot profili (robot=, varsayilan config.ROBOT_PROFILE / OPENDUCK_ROBOT): "xl330_27" yukaridaki her sey aynen.
  "tr35_b2" (rl/actuators.py, rl/sensors.py; env_cpu ile ayni sira ve ayni rastgele sayi yerlesimi): alt adim
  dongusunde gecikmeli hedef + olu bant + LEG ray CC olceklemesi (forcerange alt adim basina), her alt adimda pot/IMU
  algilamasi; aktor gozlemi sensor modelinden, oduller/kritik gercek durumdan; genel OBS_NOISE kapali. Bolum
  parametreleri info["tr"] (act/sens/pot/imu/met/ctrl_prev); yeni sayilar fold_in(k_n, 3) (bolum, aktuator +
  sensor) ve fold_in(k_noise, 5) (alt adim gurultusu). backlash=True: DS bosluk eklemleri. tr35_b2 SURECINDE
  (OPENDUCK_ROBOT) robot None ya da "tr35_b2" olmali; poz bankalari ortamin robot'u ve sahnesiyle.
- getup_head (yalniz getup; env_cpu ile ayni): None/"free" bas politikada; "fixed" kirpmadan sonra 4 bas aksiyonu 0,
  bas hedefi radians(C.HEAD_TUCK_DEG) (mutlak). Gozlem/aksiyon duzeni ve RNG ayni.
"""
import contextlib
import copy

import numpy as np
import jax
import jax.numpy as jnp
import mujoco
from mujoco import mjx

from . import actuators as ACT
from . import config as C
from . import pose_bank as PB
from . import sensors as SENS
from . import task_math as T
from . import tasks as TK
from .robot_model import RobotIndex, build_model, expand_state

try:
    from ml_collections import config_dict as _config_dict
    from mujoco_playground._src import mjx_env as _pg
except ImportError:          # Playground yoksa ayni arayuzun kucuk bir kopyasi
    _pg = None

if _pg is not None:
    State = _pg.State
    _EnvBase = _pg.MjxEnv
else:
    from flax import struct

    @struct.dataclass
    class State(object):
        data: mjx.Data
        obs: dict
        reward: jax.Array
        done: jax.Array
        metrics: dict
        info: dict

    class _EnvBase(object):
        def __init__(self, config, config_overrides=None):
            self._ctrl_dt = config["ctrl_dt"]
            self._sim_dt = config["sim_dt"]

        @property
        def dt(self):
            return self._ctrl_dt

        @property
        def sim_dt(self):
            return self._sim_dt

        @property
        def n_substeps(self):
            return int(round(self.dt / self.sim_dt))

        @property
        def observation_size(self):
            obs = jax.eval_shape(self.reset, jax.random.PRNGKey(0)).obs
            return jax.tree_util.tree_map(lambda x: x.shape, obs)

        @property
        def unwrapped(self):
            return self

try:
    from brax.envs.base import Wrapper as _BraxWrapper
except ImportError:
    _BraxWrapper = None

REW_KEYS = list(C.REWARD_WEIGHTS)
CMD_STEPS = int(round(C.COMMAND_RESAMPLE_S / C.CTRL_DT))
_DR = C.DOMAIN_RAND
_HR = C.HEAD_RANDOM
# merge_reset'te bitmis bolumden korunan info alanlari (Brax EpisodeWrapper anlami); gstep: getup mufredat sayaci
_KEEP_INFO = ("rng", "reset_key", "steps", "truncation", "time_out", "episode_done", "episode_metrics", "cmd_lock",
              "gstep")
_MEM_KEYS = ("phi_up_prev", "phi_h_prev", "head_floor_prev", "head_v2_prev")   # getup: onceki adimin T.getup_memory'si
_EV_KEYS = {"main": ("prev_sit", "mode_t", "standup_pending", "sitdown_pending"),
            "getup": ("success_t", "succeeded")}                   # info["ev_" + k]
_NO_BANK = -1.0                     # bankasiz sifirlamanin bank_label'i (env_cpu ile ayni)


def _f32(x):
    """Guclu (weak_type olmayan) float32: reset/step/merge pytree'leri ayni tipte kalsin."""
    return jnp.asarray(x, jnp.float32)


def _contact(data):
    impl = getattr(data, "_impl", None)
    c = getattr(impl, "contact", None)
    return c if c is not None else data.contact


def foot_contacts(data, idx):
    """(...,2) 0/1: ayak i zemine degiyor (ayak-zemin temas noktasi, dist < CONTACT_EPS).
    Temas yuvalarinin geom ciftleri veriden okunur; mjx.put_data ile tasinmis C temaslari da calisir."""
    c = _contact(data)
    g = c.geom
    g0, g1 = g[..., 0], g[..., 1]
    ok = c.dist < C.CONTACT_EPS
    fl = int(idx.floor_geom)
    out = []
    for f in idx.foot_geom:
        f = int(f)
        pair = ((g0 == fl) & (g1 == f)) | ((g1 == fl) & (g0 == f))
        out.append(jnp.any(ok & pair, axis=-1))
    return jnp.stack(out, axis=-1).astype(jnp.float32)


def floor_contact(data, idx, geoms):
    """(...) 0/1: geoms'dan herhangi biri zemine degiyor (foot_contacts ile ayni kural: dist < CONTACT_EPS).
    trunk_floor = floor_contact(data, idx, idx.trunk_geoms), head_floor = (..., idx.head_geoms)."""
    c = _contact(data)
    g = c.geom
    g0, g1 = g[..., 0], g[..., 1]
    fl = int(idx.floor_geom)
    hit = jnp.zeros(g0.shape, bool)
    for x in geoms:
        x = int(x)
        hit = hit | ((g0 == fl) & (g1 == x)) | ((g1 == fl) & (g0 == x))
    return jnp.any((c.dist < C.CONTACT_EPS) & hit, axis=-1).astype(jnp.float32)


def extract_state(data, idx, task="walk"):
    """mjx.Data (tek ortam ya da toplu, ilk eksenler serbest) -> task_math "s" sozlugunun ham alanlari
    (quat, gyro, linvel, trunk_z, q, qd, qdd_legs, torque, foot_pos, foot_vel, contact). idx: RobotIndex.
    main/getup: + trunk_floor, head_floor; getup: + head_vel (rl/TASK.md "Extra state").
    Esdeger C karsiligi icin MjData'nin ayni qpos/qvel/ctrl'u ile mjx.forward cagrilmis veri kullanin."""
    sd = data.sensordata
    out = {
        "quat": sd[..., idx.s_quat],
        "gyro": sd[..., idx.s_gyro],
        "linvel": sd[..., idx.s_linvel],
        "trunk_z": data.xpos[..., idx.trunk_body, 2],
        "q": data.qpos[..., idx.obs_qpos_adr],
        "qd": data.qvel[..., idx.obs_dof_adr],
        "qdd_legs": data.qacc[..., idx.leg_dof_adr],
        "torque": data.actuator_force[..., idx.obs_act_id],
        "foot_pos": jnp.stack([sd[..., sl] for sl in idx.s_foot_pos], axis=-2),
        "foot_vel": jnp.stack([sd[..., sl] for sl in idx.s_foot_vel], axis=-2),
        "contact": foot_contacts(data, idx),
    }
    name = TK.get_task(task).name
    if name != "walk":
        out["trunk_floor"] = floor_contact(data, idx, idx.trunk_geoms)
        out["head_floor"] = floor_contact(data, idx, idx.head_geoms)
    if name == "getup":
        out["head_vel"] = sd[..., idx.s_head_vel]
    return out


_CORNER_SIGNS = np.array([[1.0 if i & 1 else -1.0, 1.0 if i & 2 else -1.0, 1.0 if i & 4 else -1.0]
                          for i in range(8)])
_PLANE_BOX_ORIG = None


def install_c_plane_box(enable=True):
    """MJX'in duzlem-kutu carpismasini MuJoCo C'ninkiyle (mjc_PlaneBox) degistirir: kutunun normal
    yonunde alt yarisindaki, dist <= 0 olan koselerden indeks sirasina gore ilk 4'u. Surec geneli;
    enable=False orijinali geri koyar. Etki, bundan sonra izlenen (trace) fonksiyonlarda gorulur."""
    global _PLANE_BOX_ORIG
    from mujoco.mjx._src import collision_driver as cd
    from mujoco.mjx._src import math as mmath
    from mujoco.mjx._src.collision_convex import collider
    from mujoco.mjx._src.types import GeomType
    key = (GeomType.PLANE, GeomType.BOX)
    if _PLANE_BOX_ORIG is None:
        _PLANE_BOX_ORIG = cd._COLLISION_FUNC[key]
    if not enable:
        cd._COLLISION_FUNC[key] = _PLANE_BOX_ORIG
        return

    @collider(ncon=4)
    def plane_box_c(plane, box):
        n = plane.mat[:, 2]
        dist0 = jnp.dot(box.pos - plane.pos, n)
        corners = (_CORNER_SIGNS * box.size) @ box.mat.T
        ldist = corners @ n
        d = dist0 + ldist
        ok = (d <= 0.0) & (ldist <= 0.0)
        rank = jnp.cumsum(ok) - 1
        sel = ok[None, :] & (rank[None, :] == jnp.arange(4)[:, None])
        k = jnp.argmax(sel, axis=1)
        dist = jnp.where(jnp.any(sel, axis=1), d[k], 1.0)
        pos = box.pos + corners[k] - 0.5 * dist[:, None] * n
        frame = jnp.stack([mmath.make_frame(n)] * 4, axis=0)
        return dist, pos, frame

    cd._COLLISION_FUNC[key] = plane_box_c


def physics_step(m, d):
    """mjx.step; implicitfast'ta forcerange'e dayanmis aktuatorlerin hiz turevi C'deki gibi atlanir."""
    if m.opt.integrator != mjx.IntegratorType.IMPLICITFAST or not np.any(np.asarray(m.actuator_forcelimited)):
        return mjx.step(m, d)
    d = mjx.forward(m, d)
    f = d.actuator_force
    lim = jnp.asarray(np.asarray(m.actuator_forcelimited, bool))
    clamped = lim & ((f <= m.actuator_forcerange[:, 0]) | (f >= m.actuator_forcerange[:, 1]))
    mi = m.replace(actuator_biasprm=m.actuator_biasprm.at[:, 2].set(jnp.where(clamped, 0.0, m.actuator_biasprm[:, 2])),
                   actuator_gainprm=m.actuator_gainprm.at[:, 2].set(jnp.where(clamped, 0.0, m.actuator_gainprm[:, 2])))
    return mjx.implicit(mi, d)


def _quat_mul(a, b):
    w1, x1, y1, z1 = a[0], a[1], a[2], a[3]
    w2, x2, y2, z2 = b[0], b[1], b[2], b[3]
    return jnp.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                      w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                      w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _uniform(key, lo, hi, shape=()):
    return jax.random.uniform(key, shape, minval=lo, maxval=hi)


class OpenDuckEnv(_EnvBase):
    """Tek ortam (Playground MjxEnv arayuzu). Bayraklar rl/env_cpu.VecEnv ile ayni anlamdadir.

    task: "walk" | "main" | "getup" (rl/tasks.py). reset_probs: sifirlama karisimi degisikligi, or.
    {"home": 1.0} ya da {"seated": 1.0} (main), {"fallen": 1.0} (getup); verilmeyen turler 0, toplam 1'e
    olceklenir. curriculum_steps / final_weights: getup gec odul terimlerinin ilerlemesi (sinif notu).
    random_initial_steps: ilk (otomatik olmayan) sifirlamada info["steps"] ~ U{0, max_episode_steps-1}; getup
    egitiminde dusme sonlandirmasi olmadigi icin tum ortamlar ayni adimda zaman asimina ugrar ve her PPO
    yigini bolumun tek bir 0.4 s'lik penceresini gorur (legged_gym init_at_random_ep_len)."""

    def __init__(self, randomize=True, noise=True, head_random=True, push=True, fixed_command=None,
                 solver_iterations=4, ls_iterations=8, exact_setconst=True, c_semantics=True, scene_path=None,
                 matmul_precision="highest", task="walk", reset_probs=None, curriculum_steps=None,
                 final_weights=False, random_initial_steps=False, robot=None, backlash=None, getup_head=None):
        cfg = dict(ctrl_dt=C.CTRL_DT, sim_dt=C.SIM_DT)
        self.matmul_precision = matmul_precision or None
        super().__init__(_config_dict.create(**cfg) if _pg is not None else cfg)
        self.spec = spec = TK.get_task(task)
        self.task = spec.name
        self.robot = ACT.env_profile(robot)
        self.getup_head = TK.getup_head_mode(getup_head, spec)
        self.tr35 = self.robot == ACT.TR35
        self.backlash = bool(backlash)
        self.dr = ACT.domain_rand(self.robot)          # xl330: config.DOMAIN_RAND
        mj_model = build_model(scene_path, variant=spec.model_variant, robot=self.robot, backlash=self.backlash)
        if solver_iterations:
            mj_model.opt.iterations = int(solver_iterations)
        if ls_iterations:
            mj_model.opt.ls_iterations = int(ls_iterations)
        self.c_semantics = bool(c_semantics)
        if self.c_semantics:
            if np.any(mj_model.geom_margin != 0) or np.any(mj_model.geom_gap != 0):
                raise ValueError("c_semantics: duzlem-kutu C carpismasi margin/gap = 0 varsayar")
            install_c_plane_box()
        self._mj_model = mj_model
        self._mjx_model = mjx.put_model(mj_model, impl="jax")
        self._data0 = mjx.make_data(mj_model, impl="jax")
        self.idx = idx = RobotIndex(mj_model, spec.action_joints)
        self._xml_path = scene_path or C.SCENE_PATH
        self._scene_path = scene_path

        self.randomize = bool(randomize) and self.dr["enabled"]
        self.push = bool(push) and self.dr["enabled"]
        self.noise = bool(noise)
        self.head_random = bool(head_random) and spec.head_mode != "policy"     # getup: bas politikada
        # jest kanali (tr35 main, spec eki K6.1; env_cpu ile ayni: mevcut 6 bas uniform'u, yeni sayi yok)
        self.gestures = self.head_random and self.task == "main" and bool(C.VOICE["gestures"])
        if self.gestures:
            T.check_head_slew(True)
        self.exact_setconst = bool(exact_setconst)
        if fixed_command is None or self.task == "walk":
            self.fixed_command = None if fixed_command is None else np.asarray(fixed_command, np.float32).reshape(3)
        else:                                  # main: 3 -> [vx, vy, wz, H_HOME, 0]; getup: ValueError
            self.fixed_command = np.asarray(TK.pad_command(spec, np.ravel(fixed_command)), np.float32)
        self.K = int(self.dr["action_delay_steps"][1]) if self.randomize else 0
        self.max_episode_steps = spec.max_episode_steps
        self.rew_keys = list(spec.reward_weights)
        self.final_weights = bool(final_weights)
        self.curriculum_steps = float(curriculum_steps) if curriculum_steps else None
        self.random_initial_steps = bool(random_initial_steps)

        m = mj_model
        assert np.all(m.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT)
        fj = int(np.flatnonzero(m.jnt_type == mujoco.mjtJoint.mjJNT_FREE)[0])
        self.qadr, self.dadr = int(m.jnt_qposadr[fj]), int(m.jnt_dofadr[fj])
        self.act_dof = np.asarray(m.jnt_dofadr[m.actuator_trnid[:, 0]])       # aktuatorlu dof'lar (16)
        self.sts_act = np.asarray(idx.obs_act_id)                              # 14 politika/bas servosu (XL330)
        self.fric_geoms = np.array(list(idx.foot_geom) + [idx.floor_geom])
        self.body_weldid = np.asarray(m.body_weldid)
        self.body_parentid = np.asarray(m.body_parentid)

        f32 = lambda x: jnp.asarray(np.asarray(x, dtype=np.float32))
        self.init_qpos = f32(idx.init_qpos)
        self.init_ctrl = f32(idx.init_ctrl)
        self.default_obs_q = f32(idx.default_obs_q)
        self.default_action_q = f32(idx.default_action_q)
        self.default_head = f32(idx.init_ctrl[idx.head_act_id])
        self.other_act_id = np.asarray(idx.other_act_id)
        self.ctrl_lo, self.ctrl_hi = f32(idx.ctrl_lo), f32(idx.ctrl_hi)
        self.head_range = f32(idx.head_range)
        self.noise_vec = f32(T.noise_vector(np, self.task))
        self.cmd_lo = f32([C.COMMAND_RANGES[k][0] for k in ("vx", "vy", "wz")])
        self.cmd_hi = f32([C.COMMAND_RANGES[k][1] for k in ("vx", "vy", "wz")])
        if self.task == "walk":
            self.w_dt = {k: float(C.REWARD_WEIGHTS[k] * C.CTRL_DT) for k in REW_KEYS}
        else:                                  # main sabit; getup: gec terimler adimda ilerlemeyle (_progress)
            self.weights = T.reward_weights(np, self.task)
            self.w_dt = {k: float(self.weights[k] * C.CTRL_DT) for k in self.rew_keys}
        self._act_kw = {}                      # T.action_to_targets ek argumanlari (walk/main: varsayilan)
        if spec.action_mode == "home_clip":
            self._act_kw = dict(scale=f32(TK.action_scale_vector(spec, idx.default_action_q, idx.action_lo,
                                                                 idx.action_hi)),
                                clip=float(spec.action_clip), lo=f32(idx.action_lo), hi=f32(idx.action_hi))
        self.head_fixed = self.task == "getup" and self.getup_head == "fixed"
        if self.head_fixed:                    # bas aksiyon boyutlari ve toplanmis bas hedefi (mutlak)
            self._head_fix = tuple(f32(v) for v in T.head_fix_arrays(np, spec.action_joints))
        self.nominal = {k: getattr(self._mjx_model, k) for k in (
            "geom_friction", "body_mass", "body_inertia", "body_ipos", "actuator_gainprm",
            "actuator_biasprm", "dof_damping", "dof_frictionloss", "dof_armature")}
        if self.tr35:
            self._tr35_init()
        self._init_banks(reset_probs)

    def _tr35_init(self):
        """tr35 statik tablolari: aktuator/pot tablolari, RNG yerlesimi, ana eklemlerin limit satirlari (MJX efc)."""
        idx, m = self.idx, self._mj_model
        self.noise_vec = jnp.zeros_like(self.noise_vec)          # genel gozlem gurultusu yok (sensor modeli)
        self._act, self._pot = idx.act, idx.pot
        no = len(idx.obs_qpos_adr)
        self.n_ue_act = ACT.n_uniform(m.nu)
        self.n_ue = self.n_ue_act + SENS.n_episode(no)
        self.n_tick = SENS.n_tick(no)
        self.nominal["actuator_forcerange"] = self._mjx_model.actuator_forcerange
        self.nominal["jnt_range"] = self._mjx_model.jnt_range
        from mujoco.mjx._src.types import ConstraintType, JointType
        et = np.asarray(self._data0._impl.efc_type)
        rows = np.flatnonzero(et == ConstraintType.LIMIT_JOINT)
        jt = np.asarray(m.jnt_type)
        if np.any((jt == JointType.BALL) & (m.jnt_limited > 0)):
            raise ValueError("tr35 limit metrigi sinirli ball eklemi varsaymaz")
        jl = np.flatnonzero(np.isin(jt, (JointType.SLIDE, JointType.HINGE)) & (m.jnt_limited > 0))
        assert len(rows) == len(jl), (len(rows), len(jl))
        a = idx.act["jnt2act"][jl]
        keep = a >= 0                                                  # bosluk eklemleri disarida
        self._lim_rows, self._lim_dof, self._lim_act = rows[keep], np.asarray(m.jnt_dofadr[jl[keep]]), a[keep]

    def _limit_torque(self, d):
        """(nu,) ana aktuatorlu eklemlerin |limit kisit torku| (efc LIMIT_JOINT satirlari, J = +-1 aktifken)."""
        f = d._impl.efc_force[self._lim_rows] * d._impl.efc_J[self._lim_rows, self._lim_dof]
        return jnp.zeros(self._mj_model.nu, jnp.float32).at[self._lim_act].set(jnp.abs(f))

    def _tr35_reset(self, key):
        """Bolum parametreleri (fold_in(k_n, 3)): (aktuator ep {tau_s, db_half, delay}, tam ap, sensor ep)."""
        ue = jax.random.uniform(key, (self.n_ue,))
        ap = ACT.sample_params(jnp, ue[:self.n_ue_act], self._act, self.randomize)
        ap = {k: (v.astype(jnp.int32) if k == "delay" else _f32(v)) for k, v in ap.items()}
        sp = {k: _f32(v) for k, v in SENS.episode(jnp, ue[self.n_ue_act:], self._pot, self.noise).items()}
        return {k: ap[k] for k in ("tau_s", "db_half", "delay")}, ap, sp

    def _tr35_substeps(self, m, data, ctrl_new, tr, us):
        """N_SUBSTEPS alt adim (actuators/sensors sirasi; env_cpu._tr35_substeps ile ayni). us (N_SUBSTEPS, n_tick)."""
        idx, t, P = self.idx, self._act, self._pot
        aq, ad, oq, oa = t["act_qadr"], t["act_dadr"], idx.obs_qpos_adr, idx.obs_act_id
        no = len(oq)
        gain, bp, damp = m.actuator_gainprm[:, 0], m.actuator_biasprm, m.dof_damping[ad]
        ap, sp, prev = tr["act"], tr["sens"], tr["ctrl_prev"]
        sub = physics_step if self.c_semantics else mjx.step
        sg, sa = idx.s_gyro, idx.s_acc

        def body(k, carry):
            d, pot, imu, met = carry
            q, qd = d.qpos[aq], d.qvel[ad]
            c, frc, dem, sc = ACT.shape_substep(jnp, k, prev, ctrl_new, q, qd, ap, t, gain, bp[:, 0], bp[:, 1],
                                                bp[:, 2], damp)
            d = sub(m.replace(actuator_forcerange=jnp.stack([-frc, frc], axis=-1)), d.replace(ctrl=c))
            tn, cur, rail = ACT.after_substep(jnp, d.actuator_force, qd, damp, ap, t)
            u = us[k]
            pot = SENS.pot_tick(jnp, pot, d.qpos[oq], cur[oa], rail, u[:no], sp, P, self.noise)
            imu = SENS.imu_tick(jnp, imu, d.sensordata[sg], d.sensordata[sa], rail, u[no:], sp, self.noise)
            met = {"tau_net": tn, "limit": jnp.maximum(met["limit"], self._limit_torque(d)),
                   "rail": met["rail"] + dem, "rail_peak": jnp.maximum(met["rail_peak"], dem),
                   "cc": jnp.minimum(met["cc"], sc)}
            return d, pot, imu, met

        met0 = ACT.metrics_init(jnp, jnp.zeros(self._mj_model.nu, jnp.float32))
        d, pot, imu, met = jax.lax.fori_loop(0, C.N_SUBSTEPS, body, (data, tr["pot"], tr["imu"], met0))
        met["rail"] = met["rail"] / C.N_SUBSTEPS
        return d, dict(tr, pot=pot, imu=imu, met=met, ctrl_prev=_f32(ctrl_new))

    def _tr35_state(self, tr):
        """s'ye eklenen olcut kancalari (gozlem eklemleri sirasi; env_cpu._state ile ayni anahtarlar)."""
        oa, met = self.idx.obs_act_id, tr["met"]
        tn, ts = met["tau_net"][oa], tr["act"]["tau_s"][oa]
        return {"tau_net": tn, "tau_s": ts, "tau_ratio": tn / ts, "limit_torque": met["limit"][oa],
                "rail_demand": met["rail"], "rail_peak": met["rail_peak"], "cc_scale": met["cc"]}

    def _init_banks(self, reset_probs):
        """Sifirlama turleri ve poz bankalari (kurucuda bir kez, MuJoCo C; sifirlamada yalniz indeks secilir).
        main: [oturus]; getup: [dusmus, dusus ortasi] tek dizide (indeks ofsetli). Olasiligi 0 olan bankanin
        yerine tek satirlik HOME yer tutucu konur (uretim suresi harcanmaz)."""
        spec, idx = self.spec, self.idx
        kinds = [k for k, _ in spec.reset_probs]
        if reset_probs is None:
            probs = [p for _, p in spec.reset_probs]
        else:
            bad = set(reset_probs) - set(kinds)
            if bad:
                raise ValueError("%s: bilinmeyen sifirlama turu %s (%s)" % (self.task, sorted(bad), kinds))
            probs = [float(reset_probs.get(k, 0.0)) for k in kinds]
            tot = sum(probs)
            if not tot > 0.0 or min(probs) < 0.0:
                raise ValueError("reset_probs: %r" % (reset_probs,))
            probs = [p / tot for p in probs]
        self.reset_probs = dict(zip(kinds, probs))
        self._kind_spec = copy.copy(spec)
        self._kind_spec.reset_probs = tuple(zip(kinds, probs))
        self.bank, self.bank_n = None, ()
        names = [k for k in kinds if k in PB.KINDS]
        if not names:
            return
        nq, nv, nu = self._mj_model.nq, self._mj_model.nv, self._mj_model.nu
        parts = []
        for k in names:
            if self.reset_probs[k] > 0.0:
                b = PB.bank(k, robot=self.robot, scene_path=self._scene_path)
                PB.check_variant(b, spec.model_variant)
                bq, bv = expand_state(self._mj_model, b["qpos"], b["qvel"])        # bosluk eklemleri 0
                if bq.shape[1] != nq or bv.shape[1] != nv or b["ctrl"].shape[1] != nu:
                    raise ValueError("%s bankasi bu modelle uyusmuyor (nq/nv/nu)" % k)
                parts.append((bq, bv, b["ctrl"], b["label"]))
            else:
                parts.append((idx.init_qpos[None], np.zeros((1, nv)), idx.init_ctrl[None], np.array([_NO_BANK])))
        self.bank_n = tuple(len(p[0]) for p in parts)
        f32 = lambda x: jnp.asarray(np.asarray(x, dtype=np.float32))
        self.bank = {key: f32(np.concatenate([p[j] for p in parts])) for j, key in
                     enumerate(("qpos", "qvel", "ctrl", "label"))}

    # -- Playground arayuzu
    @property
    def xml_path(self):
        return self._xml_path

    @property
    def action_size(self):
        return self.spec.num_actions

    @property
    def mj_model(self):
        return self._mj_model

    @property
    def mjx_model(self):
        return self._mjx_model

    # -- alan rastgelelestirme
    def sample_dr(self, key, ap=None):
        """Bir bolumun rastgele model parametreleri (NOMINAL degerlerden), mj_setConst ciktilari dahil.
        tr35: aktuator parametreleri ap'den (actuators.sample_params; None ise fold_in(key, 3)'ten cekilir)."""
        m, nom, tb = self._mjx_model, self.nominal, self.idx.trunk_body
        ks = jax.random.split(key, 9)
        nb, nad, nk = m.nbody - 1, len(self.act_dof), len(self.sts_act)
        f = _uniform(ks[0], *self.dr["friction"])
        scale = jnp.ones(m.nbody).at[1:].set(_uniform(ks[1], *self.dr["link_mass_scale"], (nb,)))
        mass = nom["body_mass"] * scale
        inertia = nom["body_inertia"] * scale[:, None]
        new = mass[tb] + _uniform(ks[2], *self.dr["trunk_added_mass"])
        inertia = inertia.at[tb].multiply(new / mass[tb])
        mass = mass.at[tb].set(new)
        o = self.dr["trunk_com_offset"]
        ipos = nom["body_ipos"].at[tb].add(_uniform(ks[3], -o, o, (3,)))
        gain0 = nom["actuator_gainprm"][:, 0]
        kp = gain0.at[self.sts_act].multiply(_uniform(ks[4], *self.dr["kp_scale"], (nk,)))
        ad = self.act_dof
        p = {
            "friction": f, "body_mass": mass, "body_inertia": inertia, "body_ipos": ipos, "kp": kp,
            "dof_damping": nom["dof_damping"].at[ad].multiply(_uniform(ks[5], *self.dr["damping_scale"], (nad,))),
            "dof_frictionloss": nom["dof_frictionloss"].at[ad].multiply(
                _uniform(ks[6], *self.dr["frictionloss_scale"], (nad,))),
            "dof_armature": nom["dof_armature"].at[ad].multiply(_uniform(ks[7], *self.dr["armature_scale"], (nad,))),
        }
        lo, hi = self.dr["action_delay_steps"]
        p["delay"] = jax.random.randint(ks[8], (), lo, hi + 1)
        if self.tr35:                      # genel aktuator carpanlari 1; servo modeli DR'i (spec 4)
            if ap is None:
                u = jax.random.uniform(jax.random.fold_in(key, 3), (self.n_ue_act,))
                ap = ACT.sample_params(jnp, u, self._act, True)
            p["kp"] = ap["kp"]
            p["forcerange"] = ap["tau_s"]
            for fld, k in (("dof_damping", "damping"), ("dof_armature", "armature"),
                           ("dof_frictionloss", "frictionloss")):
                p[fld] = p[fld].at[ad].set(ap[k])
            pj = self._act["play_jnt"] >= 0
            if np.any(pj):
                h = 0.5 * ap["backlash"][pj]
                p["jnt_range"] = self.nominal["jnt_range"].at[self._act["play_jnt"][pj]].set(jnp.stack([-h, h], -1))
        if self.exact_setconst:
            p.update(self.set_const(self._apply(m, p)))
        return p

    def _apply(self, m, p):
        gf = self.nominal["geom_friction"].at[self.fric_geoms, 0].set(p["friction"])
        rep = {
            "geom_friction": gf, "body_mass": p["body_mass"], "body_inertia": p["body_inertia"],
            "body_ipos": p["body_ipos"],
            "actuator_gainprm": self.nominal["actuator_gainprm"].at[:, 0].set(p["kp"]),
            "actuator_biasprm": self.nominal["actuator_biasprm"].at[:, 1].set(-p["kp"]),
            "dof_damping": p["dof_damping"], "dof_frictionloss": p["dof_frictionloss"],
            "dof_armature": p["dof_armature"],
        }
        if "forcerange" in p:
            rep["actuator_forcerange"] = jnp.stack([-p["forcerange"], p["forcerange"]], axis=-1)
        if "jnt_range" in p:
            rep["jnt_range"] = p["jnt_range"]
        for k in ("body_invweight0", "dof_invweight0", "dof_M0", "body_subtreemass"):
            if k in p:
                rep[k] = p[k]
        if "meaninertia" in p:
            rep["stat.meaninertia"] = p["meaninertia"]
        return m.tree_replace(rep)

    def model_from_dr(self, p):
        """sample_dr ciktisindan mjx.Model."""
        return self._apply(self._mjx_model, p)

    def set_const(self, m):
        """mj_setConst'un MJX'in kullandigi ciktilari, qpos0'da (C ile ayni formuller)."""
        d = self._data0.replace(qpos=m.qpos0)
        d = mjx.kinematics(m, d)
        d = mjx.com_pos(m, d)
        d = mjx.crb(m, d)
        M = mjx.full_m(m, d)
        Minv = jnp.linalg.inv(M)

        def body_iw(b):
            jp_, jr = mjx.jac(m, d, d.xipos[b], b)
            J = jnp.concatenate([jp_.T, jr.T], axis=0)
            A = J @ Minv @ J.T
            return jnp.stack([jnp.trace(A[:3, :3]), jnp.trace(A[3:, 3:])]) / 3.0

        biw = jax.vmap(body_iw)(jnp.arange(m.nbody))
        biw = jnp.where((self.body_weldid == 0)[:, None], 0.0, biw)
        dinv = jnp.diag(Minv)
        mm = self._mj_model
        for j in range(mm.njnt):
            a = int(mm.jnt_dofadr[j])
            t = mm.jnt_type[j]
            if t == mujoco.mjtJoint.mjJNT_FREE:
                dinv = dinv.at[a:a + 3].set(jnp.mean(dinv[a:a + 3])).at[a + 3:a + 6].set(jnp.mean(dinv[a + 3:a + 6]))
            elif t == mujoco.mjtJoint.mjJNT_BALL:
                dinv = dinv.at[a:a + 3].set(jnp.mean(dinv[a:a + 3]))
        mass = m.body_mass
        sub = mass
        for b in range(mm.nbody - 1, 0, -1):
            sub = sub.at[int(self.body_parentid[b])].add(sub[b])
        dM = jnp.diag(M)
        return {"body_invweight0": biw, "dof_invweight0": dinv, "dof_M0": dM, "body_subtreemass": sub,
                "meaninertia": jnp.sum(dM) / mm.nv}

    def randomization_fn(self, model, rng):
        """Playground/Brax bicimi: rng (B,2) -> (B ortamli mjx.Model, in_axes). Ayni dagilim (sample_dr),
        ama bolum basina degil ortam basina sabit; egitimde KULLANILMAZ (orada bolum basina cekilir)."""
        with self._precision():
            p = jax.vmap(self.sample_dr)(rng)
        p.pop("delay")
        mv = jax.vmap(lambda q: self._apply(model, q))(p)
        names = ["geom_friction", "body_mass", "body_inertia", "body_ipos", "actuator_gainprm",
                 "actuator_biasprm", "dof_damping", "dof_frictionloss", "dof_armature"]
        if self.exact_setconst:
            names += ["body_invweight0", "dof_invweight0", "dof_M0", "body_subtreemass", "stat.meaninertia"]
        if self.tr35:
            names += ["actuator_forcerange"] + (["jnt_range"] if "jnt_range" in p else [])
        in_axes = jax.tree_util.tree_map(lambda x: None, model)
        in_axes = in_axes.tree_replace({k: 0 for k in names})
        return mv, in_axes

    # -- yardimcilar
    def _precision(self):
        """Izleme (trace) sirasinda gecerli matmul hassasiyeti (precision=None olan dot_general'lar icin)."""
        if self.matmul_precision is None:
            return contextlib.nullcontext()
        return jax.default_matmul_precision(self.matmul_precision)

    def _model(self, info):
        return self.model_from_dr(info["dr"]) if self.randomize else self._mjx_model

    def _friction(self, info):
        return info["dr"]["friction"] if self.randomize else jnp.float32(1.0)

    def _sample_command(self, key):
        k1, k2 = jax.random.split(key)
        cmd = self.cmd_lo + (self.cmd_hi - self.cmd_lo) * jax.random.uniform(k2, (3,))
        return jnp.where(jax.random.uniform(k1) < C.COMMAND_ZERO_PROB, 0.0, cmd)

    def _progress(self, gstep):
        """getup mufredat ilerlemesi p (skaler, tum ortamlarda ayni): final_weights -> 1; curriculum_steps
        (ortam basina adim; = toplam egitim adimi / ortam sayisi) -> clip(gstep / curriculum_steps, 0, 1);
        hicbiri -> 0 (erken agirliklar)."""
        if self.final_weights:
            return jnp.float32(1.0)
        if self.curriculum_steps:
            return jnp.clip(gstep.astype(jnp.float32) / jnp.float32(self.curriculum_steps), 0.0, 1.0)
        return jnp.float32(0.0)

    def _obs(self, s, key, tr=None):
        if self.tr35:                      # aktor: sensor modeli ciktisi; kritik: gercek durum
            so = SENS.obs_state(jnp, s, tr["pot"]["v"], tr["pot"]["qdf"], tr["imu"]["w"], tr["imu"]["g"])
            obs = T.observation(jnp, so, self.task)
        else:
            obs = T.observation(jnp, s, self.task)
            if self.noise:
                obs = obs + self.noise_vec * jax.random.uniform(key, obs.shape, minval=-1.0, maxval=1.0)
        return {"state": obs, "privileged_state": T.critic_observation(jnp, s, self.task)}

    def _task_state(self, data, info, action, last_action, command=None, phase=None):
        s = extract_state(data, self.idx, self.task)
        s = {k: jnp.nan_to_num(v, nan=0.0, posinf=0.0, neginf=0.0) for k, v in s.items()}
        if self.tr35:
            s.update(self._tr35_state(info["tr"]))
        if self.task == "walk":
            s.update(action=action, last_action=last_action, command=command, phase=phase,
                     friction=self._friction(info), default_q=self.default_obs_q)
            return s
        s.update(action=action, last_action=last_action, friction=self._friction(info), default_q=self.default_obs_q)
        if self.task == "main":         # h_ref/alpha: info'daki (adimda posture_step SONRASI) deger
            s.update(command=command, phase=phase, h_ref=info["h_ref"], alpha=info["alpha"])
        else:                           # getup: onceki adimin bellegi (sifirlamada henuz yok)
            s.update({k: info[k] for k in _MEM_KEYS if k in info})
        return s

    def _targets(self, action, base_q):
        """T.action_to_targets (gorev eslemesi) + getup_head="fixed" (bas aksiyonlari 0, bas hedefi toplanmis)."""
        targets, a = T.action_to_targets(jnp, action, base_q, **self._act_kw)
        if self.head_fixed:
            targets, a = T.fixed_head(jnp, targets, a, *self._head_fix)
        return targets, a

    def _bank_reset(self, key, yaw, qpos, qvel):
        """Sifirlama turu ve banka durumu (fizik yok, yalniz indeks). Dondurur (tur, etiket, qpos, qvel, ctrl);
        HOME/ayakta turunde verilen qpos/qvel (walk sifirlamasi) ve keyframe ctrl kalir."""
        k_kind, k_i = jax.random.split(key)
        kind = T.sample_reset_kind(jnp, jax.random.uniform(k_kind), self._kind_spec)
        u = jax.random.uniform(k_i)
        n0 = self.bank_n[0]
        if self.task == "main":            # 0 home, 1 seated
            use = kind > 0.5
            i = T.bank_index(jnp, u, n0)
        else:                              # 0 fallen, 1 midfall, 2 stand
            use = kind < 1.5
            i = jnp.where(kind < 0.5, T.bank_index(jnp, u, n0), n0 + T.bank_index(jnp, u, self.bank_n[1]))
        b, a = self.bank, self.qadr
        qb = b["qpos"][i]
        qb = qb.at[a:a + 2].set(0.0).at[a + 3:a + 7].set(T.quat_mul(jnp, T.yaw_quat(jnp, yaw), qb[a + 3:a + 7]))
        return (_f32(kind), _f32(jnp.where(use, b["label"][i], _NO_BANK)), jnp.where(use, qb, qpos),
                jnp.where(use, b["qvel"][i], qvel), jnp.where(use, b["ctrl"][i], self.init_ctrl))

    def _forward_obs(self, m, d):
        """Sifirlamada gozlem icin gereken asamalar (konum/hiz sensorleri, temaslar); qacc ve
        actuator_force sifir kalir (sifirlama gozleminde kullanilmaz, sonraki mjx.step bastan hesaplar)."""
        d = mjx.kinematics(m, d)
        d = mjx.com_pos(m, d)
        d = mjx.com_vel(m, d)
        d = mjx.collision(m, d)
        d = mjx.sensor_pos(m, d)
        return mjx.sensor_vel(m, d)

    # -- sifirlama
    def reset(self, rng):
        with self._precision():
            st = self._reset(rng)
            if self.random_initial_steps:       # yalniz ilk sifirlama; auto_reset _reset'i cagirir (steps = 0)
                s0 = jax.random.randint(jax.random.fold_in(rng, 7), (), 0, self.max_episode_steps, jnp.int32)
                st = st.replace(info=dict(st.info, steps=s0))
            return st

    def auto_reset(self, stepped):
        """Biten ortam icin taze sifirlama (stepped.info["reset_key"]); set_command ile kilitli komut
        korunur, sifirlama gozlemi de bu komutla kurulur (CPU'daki set_commands ile ayni)."""
        with self._precision():
            if "cmd_lock" not in stepped.info:          # getup: komut yok
                return self._reset(stepped.info["reset_key"])
            return self._reset(stepped.info["reset_key"], stepped.info["command"], stepped.info["cmd_lock"])

    def _reset(self, rng, locked_command=None, cmd_lock=None):
        idx, task, spec = self.idx, self.task, self.spec
        rng, k_dr, k_q, k_yaw, k_v, k_ph, k_cmd, k_ht, k_pt, k_n = jax.random.split(rng, 10)
        info = {"rng": rng, "reset_key": k_n}
        if self.tr35:
            ep, ap, sp = self._tr35_reset(jax.random.fold_in(k_n, 3))
        if self.randomize:
            info["dr"] = self.sample_dr(k_dr, ap) if self.tr35 else self.sample_dr(k_dr)
        m = self._model(info)

        nj = len(idx.obs_qpos_adr)
        qpos = self.init_qpos.at[idx.obs_qpos_adr].add(_uniform(k_q, -C.RESET_JOINT_NOISE, C.RESET_JOINT_NOISE, (nj,)))
        yaw = _uniform(k_yaw, -jnp.pi, jnp.pi)
        a = self.qadr
        quat = _quat_mul(jnp.stack([jnp.cos(0.5 * yaw), 0.0, 0.0, jnp.sin(0.5 * yaw)]), self.init_qpos[a + 3:a + 7])
        qpos = qpos.at[a:a + 2].set(0.0).at[a + 3:a + 7].set(quat)
        v = C.RESET_BASE_VEL_NOISE
        qvel = jnp.zeros(self._mj_model.nv).at[self.dadr:self.dadr + 2].set(_uniform(k_v, -v, v, (2,)))
        ctrl = self.init_ctrl
        if self.bank is not None:        # main/getup: banka turleri (yeni sayilar fold_in(k_n, 2) ile)
            kind, label, qpos, qvel, ctrl = self._bank_reset(jax.random.fold_in(k_n, 2), yaw, qpos, qvel)
        data = self._forward_obs(m, self._data0.replace(qpos=qpos, qvel=qvel, ctrl=ctrl))
        if self.tr35:                      # sensor filtreleri oturmus, olcutler 0
            sd = data.sensordata
            pot = SENS.pot_reset(jnp, data.qpos[idx.obs_qpos_adr], data.qvel[idx.obs_dof_adr], sp, self._pot)
            imu = SENS.imu_reset(jnp, sd[idx.s_quat], sd[idx.s_gyro], sp)
            info["tr"] = {"act": ep, "sens": sp, "ctrl_prev": _f32(ctrl), "pot": {k: _f32(v) for k, v in pot.items()},
                          "imu": {k: _f32(v) for k, v in imu.items()},
                          "met": ACT.metrics_init(jnp, jnp.zeros(self._mj_model.nu, jnp.float32))}

        zero_a = jnp.zeros(spec.num_actions)
        info.update({
            "steps": jnp.int32(0), "truncation": jnp.float32(0.0), "time_out": jnp.float32(0.0),
            "episode_done": jnp.float32(0.0),
            "episode_metrics": {k: jnp.float32(0.0) for k in ["sum_reward", "length"] + ["rew_" + r for r in self.rew_keys]
                                + list(spec.episode_metrics)},
            "a_prev": zero_a,
        })
        command = phase = None
        if task != "getup":
            if task == "main":            # ilk segment (oturustan: %50 sit / %50 stand), k_cmd'den 8 sayi
                alpha0 = kind
                seg, cmd_s, seg_t = T.sample_segment(jnp, jax.random.uniform(k_cmd, (T.NUM_SEGMENT_U,)),
                                                     T.reset_command(jnp, alpha0), seated=alpha0)
                info.update(seg=_f32(seg), seg_timer=_f32(seg_t), h_ref=jnp.float32(C.H_HOME), alpha=alpha0,
                            reset_kind=kind)
                info.update({"ev_" + k: _f32(v) for k, v in T.main_events_init(jnp, alpha0).items()})
            if self.fixed_command is not None:
                command, lock = jnp.asarray(self.fixed_command), jnp.float32(1.0)
            else:
                command, lock = (self._sample_command(k_cmd) if task == "walk" else _f32(cmd_s)), jnp.float32(0.0)
            if locked_command is not None:
                keep = cmd_lock > 0.5
                command = jnp.where(keep, locked_command, command)
                lock = jnp.where(keep, jnp.float32(1.0), lock)
            phase = jax.random.uniform(k_ph)
            info.update({"command": command, "cmd_lock": lock, "phase": phase, "air_time": jnp.zeros(2)})
        if self.head_random:
            info["head_offset"] = jnp.zeros(len(idx.head_act_id))
            info["head_target"] = jnp.zeros(len(idx.head_act_id))
            info["head_timer"] = _uniform(k_ht, *_HR["resample_s"])
        if self.gestures:                  # jest durumu (T.gesture_init) + segment degisimi kesme istegi
            info.update({"g_" + k: _f32(v) for k, v in T.gesture_init(jnp, jnp.float32(0.0)).items()})
            info["g_cutreq"] = jnp.float32(0.0)
        if self.push:
            info["push_timer"] = _uniform(k_pt, *self.dr["push_interval_s"])
        if self.K > 0:
            info["abuf"] = jnp.zeros((self.K + 1, spec.num_actions))

        s = self._task_state(data, info, zero_a, zero_a, command, phase)
        if task == "getup":
            info.update({k: _f32(v) for k, v in T.getup_memory(jnp, s).items()})
            info.update({"ev_" + k: _f32(v) for k, v in T.getup_events_init(jnp, s["trunk_z"]).items()})
            info.update(reset_kind=kind, bank_label=label, gstep=jnp.int32(0))
        obs = self._obs(s, jax.random.fold_in(k_n, 1), info.get("tr"))
        metrics = {"rew_" + k: jnp.float32(0.0) for k in self.rew_keys}
        metrics.update({k: jnp.float32(0.0) for k in spec.episode_metrics})
        return State(data=data, obs=obs, reward=jnp.float32(0.0), done=jnp.float32(0.0), metrics=metrics, info=info)

    # -- kontrol adimi (TASK.md sirasiyla), sifirlamasiz
    def step_no_reset(self, state, action):
        """Adim 1-11 ve 13 (biten ortamlar icin de adim sonrasi gozlem). Sifirlama: merge_reset."""
        with self._precision():
            return self._step_no_reset(state, action)

    def _step_no_reset(self, state, action):
        idx, info, task = self.idx, dict(state.info), self.task
        prev_done = state.done > 0.5
        rng, k_head, k_push, k_cmd, k_noise, k_reset = jax.random.split(info["rng"], 6)
        steps = jnp.where(prev_done, 0, info["steps"])
        m = self._model(info)

        # 1. aksiyon -> hedefler (gecikmeli). main: q_ref SON gozlemdeki durustan; getup: 14 eklem, S_j, ctrlrange
        if task == "main":
            ref = T.posture_ref(jnp, info["h_ref"], info["alpha"], self.default_obs_q)
            base_q = ref["q_ref"]
        else:
            base_q = self.default_action_q
        targets, a = self._targets(action, base_q)
        if self.K > 0:
            abuf = jnp.concatenate([a[None], info["abuf"][:-1]], axis=0)
            info["abuf"] = abuf
            targets, _ = self._targets(abuf[info["dr"]["delay"]], base_q)

        # 2-3. bas + bacak kontrolleri (main: bas scripti + alpha'ya bagli rastgele hedef; getup: bas aksiyonda)
        ctrl = self.init_ctrl
        if self.head_random:
            u = jax.random.uniform(k_head, (6,))
            gest = self.gestures
            if gest:                       # jest suresince zamanlayici durur (spec eki K6.1)
                timer = info["head_timer"] - C.CTRL_DT * jnp.where(info["g_kind"] > 0.5, 0.0, 1.0)
            else:
                timer = info["head_timer"] - C.CTRL_DT
            rs = timer <= 0.0
            if gest:                       # u0: jest / rastgele / sifir (C.GESTURE_SAMPLE), u1: jest turu
                greq, rnd = T.gesture_sample(jnp, u[0], u[1])
                tgt = jnp.where(rnd > 0.5, (2.0 * u[1:5] - 1.0) * self.head_range, 0.0)
            else:
                tgt = jnp.where(u[0] < _HR["prob"], (2.0 * u[1:5] - 1.0) * self.head_range, 0.0)
            lo, hi = _HR["resample_s"]
            head_target = jnp.where(rs, tgt, info["head_target"])
            info["head_timer"] = jnp.where(rs, lo + (hi - lo) * u[5], timer)
            gated = head_target
            if task == "main":
                gated = head_target * jnp.where(info["alpha"] < C.ALPHA_STAND_MAX, 1.0, 0.0)
            if gest:                       # jest (sonumleme dahil) suresince rastgele hedef 0
                gst = {k: info["g_" + k] for k in ("kind", "n", "amp", "cut_n")}
                gst, gout, gact, _ = T.gesture_step(jnp, gst, jnp.where(rs, greq, 0.0), info["alpha"], info["g_cutreq"])
                info.update({"g_" + k: _f32(v) for k, v in gst.items()})
                info["g_cutreq"] = jnp.float32(0.0)
                gated = gated * (1.0 - gact)
            head_offset = info["head_offset"] + (gated - info["head_offset"]) * (C.CTRL_DT / _HR["filter_tau"])
            info["head_target"], info["head_offset"] = head_target, head_offset
            head_q = self.default_head + head_offset
            if task == "main":
                head_q = head_q + ref["head_offset"]
            if gest:
                head_q = head_q + gout[:4]
                ctrl = ctrl.at[self.other_act_id].add(gout[4:])
            ctrl = ctrl.at[idx.head_act_id].set(head_q)
        elif task == "main":
            ctrl = ctrl.at[idx.head_act_id].set(self.default_head + ref["head_offset"])
        ctrl = ctrl.at[idx.action_act_id].set(targets)
        ctrl = jnp.clip(ctrl, self.ctrl_lo, self.ctrl_hi)
        if self.tr35:                      # PCA9685 adimi (kontrol adiminda bir kez)
            ctrl = ACT.quantize_pwm(jnp, ctrl, self._act["pwm_res"], self.ctrl_lo, self.ctrl_hi)

        # 4. itme (main: zamanlayici walk gibi isler, hiz darbesi yalniz alpha < ALPHA_STAND_MAX iken)
        data = state.data
        if self.push:
            u = jax.random.uniform(k_push, (3,))
            timer = info["push_timer"] - C.CTRL_DT
            pm = timer <= 0.0
            kick = (pm & (info["alpha"] < C.ALPHA_STAND_MAX)) if task == "main" else pm
            dv = jnp.where(kick, (2.0 * u[:2] - 1.0) * self.dr["push_vel"], 0.0)
            lo, hi = self.dr["push_interval_s"]
            info["push_timer"] = jnp.where(pm, lo + (hi - lo) * u[2], timer)
            data = data.replace(qvel=data.qvel.at[self.dadr:self.dadr + 2].add(dv))

        # 5. fizik (tr35: alt adim ctrl sekillendirmesi + algilama; gurultu fold_in(k_noise, 5))
        if self.tr35:
            us = jax.random.uniform(jax.random.fold_in(k_noise, 5), (C.N_SUBSTEPS, self.n_tick))
            data, info["tr"] = self._tr35_substeps(m, data, ctrl, info["tr"], us)
        else:
            data = data.replace(ctrl=ctrl)
            sub = physics_step if self.c_semantics else mjx.step
            data = jax.lax.fori_loop(0, C.N_SUBSTEPS, lambda i, d: sub(m, d), data)

        # 6. (main: durus filtresi bu adimin komutuyla; getup: faz yok, mufredat sayaci)
        phase = None
        if task != "getup":
            phase = jnp.mod(info["phase"] + C.GAIT_FREQ * C.CTRL_DT, 1.0)
        steps = steps + 1
        if task == "main":
            h_ref, alpha = T.posture_step(jnp, info["h_ref"], info["alpha"], info["command"])
            info["h_ref"], info["alpha"] = _f32(h_ref), _f32(alpha)
        if task == "getup":
            info["gstep"] = info["gstep"] + 1

        # MJX kararsizlikta NaN uretir (C veriyi sifirlar); bu ortamlar dusmus sayilir (env_cpu ile ayni)
        bad = jnp.zeros((), bool)
        for x in (data.qpos, data.qvel, data.qacc, data.sensordata, data.actuator_force):
            bad = bad | ~jnp.all(jnp.isfinite(x))
        for x in (data.qpos, data.qvel, data.qacc):
            bad = bad | jnp.any(jnp.abs(x) > mujoco.mjMAXVAL)

        # 7-9.
        if task == "getup":
            s = self._task_state(data, info, a, info["a_prev"])
        else:
            s = self._task_state(data, info, a, info["a_prev"], info["command"], phase)
            air_new, first = T.update_air_time(jnp, info["air_time"], s["contact"], C.CTRL_DT)
            s["air_time"], s["first_contact"] = air_new, first
        terms = T.reward_terms(jnp, s, task)
        terms = {k: jnp.where(bad, 0.0, jnp.broadcast_to(v, ())) for k, v in terms.items()}
        term = jnp.where(bad, 1.0, T.terminated(jnp, s, task))
        timeout = steps >= self.max_episode_steps
        w_dt = self.w_dt
        if task == "walk":
            reward = T.total_reward(jnp, terms, term)
        elif task == "main":
            reward = T.total_reward(jnp, terms, term, self.weights)
        else:                              # gec terimler mufredatla (eval: final_weights -> p = 1)
            w = T.reward_weights(jnp, task, self._progress(info["gstep"]))
            reward = T.total_reward(jnp, terms, term, w)
            w_dt = {k: w[k] * C.CTRL_DT for k in self.rew_keys}

        # 10.
        if task != "getup":
            info["air_time"] = T.reset_air_time(jnp, air_new, s["contact"])
        info["a_prev"] = a
        inc = {}
        if task == "main":
            ev, inc = T.main_events(jnp, s, {k: info["ev_" + k] for k in _EV_KEYS[task]}, term)
            info.update({"ev_" + k: _f32(ev[k]) for k in _EV_KEYS[task]})
        elif task == "getup":
            info.update({k: _f32(v) for k, v in T.getup_memory(jnp, s).items()})
            ev, inc = T.getup_events(jnp, s, {k: info["ev_" + k] for k in _EV_KEYS[task]})
            inc = {k: jnp.where(bad, 0.0, v) for k, v in inc.items()}     # bozuk fizik adimi sayilmaz (env_cpu)
            info.update({"ev_" + k: _f32(ev[k]) for k in _EV_KEYS[task]})

        # 11. komut (walk: 6 s yeniden ornekleme; main: segment zamanlayicisi, kilitli komut hic degismez)
        command = info.get("command")
        if task == "walk" and self.fixed_command is None:
            resample = (jnp.mod(steps, CMD_STEPS) == 0) & (info["cmd_lock"] < 0.5)
            command = jnp.where(resample, self._sample_command(k_cmd), command)
        elif task == "main":
            timer = info["seg_timer"] - C.CTRL_DT
            if self.fixed_command is None:
                rs = (timer <= 0.0) & (info["cmd_lock"] < 0.5)
                seg, cmd_new, dur = T.sample_segment(jnp, jax.random.uniform(k_cmd, (T.NUM_SEGMENT_U,)), command,
                                                     prev_kind=info["seg"])
                command = jnp.where(rs, _f32(cmd_new), command)
                info["seg"] = jnp.where(rs, _f32(seg), info["seg"])
                timer = jnp.where(rs, _f32(dur), timer)
                if self.gestures:          # segment degisimi jesti keser (spec eki K6.1)
                    info["g_cutreq"] = jnp.where(rs, jnp.float32(1.0), info["g_cutreq"])
            info["seg_timer"] = timer

        # 12. (sifirlama merge_reset'te)
        fell = term > 0.5
        done = (fell | timeout).astype(jnp.float32)
        time_out = (timeout & ~fell).astype(jnp.float32)

        metrics = {"rew_" + k: terms[k] * w_dt[k] for k in self.rew_keys}
        metrics.update({k: _f32(inc[k]) for k in self.spec.episode_metrics})
        em = {k: v * (1.0 - info["episode_done"]) for k, v in info["episode_metrics"].items()}
        em["sum_reward"] = em["sum_reward"] + reward
        em["length"] = em["length"] + 1.0
        for k, v in metrics.items():
            em[k] = em[k] + v
        info.update(rng=rng, reset_key=k_reset, steps=steps, truncation=time_out, time_out=time_out,
                    episode_done=done, episode_metrics=em)
        if task != "getup":
            info.update(phase=phase, command=command)

        # 13. (s': last_action = a, yeni komut; main: yeni h_ref/alpha zaten s'de)
        s["last_action"] = a
        if task != "getup":
            s["command"] = command
        obs = self._obs(s, k_noise, info.get("tr"))
        return State(data=data, obs=obs, reward=reward, done=done, metrics=metrics, info=info)

    def merge_reset(self, stepped, fresh):
        """Biten ortami (stepped.done) taze sifirlama durumuyla (auto_reset(stepped)) degistirir. Odul, done,
        metrikler, bolum sayaci/truncation/episode_metrics bitmis bolumden kalir (Brax EpisodeWrapper anlami)."""
        done = stepped.done > 0.5
        sel = lambda x, y: jnp.where(done, x, y)
        info = {}
        for k, v in stepped.info.items():
            info[k] = v if k in _KEEP_INFO else jax.tree_util.tree_map(sel, fresh.info[k], v)
        if "cmd_lock" in stepped.info:
            info["command"] = jnp.where(stepped.info["cmd_lock"] > 0.5, stepped.info["command"], info["command"])
        return stepped.replace(data=jax.tree_util.tree_map(sel, fresh.data, stepped.data),
                               obs=jax.tree_util.tree_map(sel, fresh.obs, stepped.obs), info=info)

    def step(self, state, action):
        stepped = self.step_no_reset(state, action)
        return self.merge_reset(stepped, self.auto_reset(stepped))

    # -- oynatma/degerlendirme
    def set_command(self, state, command):
        """Sabit komut (yeniden ornekleme kapanir, sifirlamada korunur). Tek ya da toplu durum.
        main: (..., 3) -> [vx, vy, wz, H_HOME, 0] ya da (..., 5) [vx, vy, wz, h (m), sit]; getup: komut yok."""
        if self.task == "walk":
            cmd = jnp.broadcast_to(jnp.asarray(command, jnp.float32), state.info["command"].shape)
        elif self.task == "main":
            cmd = jnp.asarray(command, jnp.float32)
            if cmd.shape[-1] == 3:
                pad = jnp.asarray([C.H_HOME, 0.0], jnp.float32)
                cmd = jnp.concatenate([cmd, jnp.broadcast_to(pad, cmd.shape[:-1] + (2,))], axis=-1)
            if cmd.shape[-1] != self.spec.num_commands:
                raise ValueError("main komutu 3 ya da 5 boyutlu olmali (verilen %d)" % cmd.shape[-1])
            cmd = jnp.broadcast_to(cmd, state.info["command"].shape)
        else:
            raise ValueError("%s gorevinin komutu yok" % self.task)
        info = dict(state.info, command=cmd, cmd_lock=jnp.ones_like(state.info["cmd_lock"]))
        return state.replace(info=info)


if _BraxWrapper is not None:
    class VmapAutoReset(_BraxWrapper):
        """Toplu ortam: vmap'li adim; taze sifirlama yalnizca biten ortam varken hesaplanir (lax.cond).
        env.step'i vmap'lemekle ayni sonucu verir."""

        def reset(self, rng):
            return jax.vmap(self.env.reset)(rng)

        def step(self, state, action):
            env = self.env.unwrapped
            st = jax.vmap(env.step_no_reset)(state, action)

            def with_reset(st):
                fresh = jax.vmap(env.auto_reset)(st)
                return jax.vmap(env.merge_reset)(st, fresh)

            return jax.lax.cond(jnp.any(st.done > 0.5), with_reset, lambda st: st, st)

    class _NoTruncation(_BraxWrapper):
        """bootstrap_on_timeout icin: zaman asimi truncation olarak maskelenmez."""

        def reset(self, rng):
            return self.env.reset(rng)

        def step(self, state, action):
            st = self.env.step(state, action)
            return st.replace(info=dict(st.info, truncation=jnp.zeros_like(st.info["truncation"])))
else:
    VmapAutoReset = None


def wrap_for_brax_training(env, episode_length=None, action_repeat=1, randomization_fn=None,
                           bootstrap_on_timeout=False):
    """brax.training.agents.ppo.train(wrap_env_fn=...) icin. Bolum sonu ve sifirlama ortamin icinde;
    episode_length ortamin bolum uzunlugu (env.max_episode_steps: walk/main 1000, getup 400) olmali."""
    if VmapAutoReset is None:
        raise ImportError("brax gerekli: pip install brax")
    if randomization_fn is not None:
        raise ValueError("alan rastgelelestirme ortamin icinde bolum basina yapilir; randomization_fn verilmemeli")
    if action_repeat != 1:
        raise ValueError("action_repeat=1 olmali")
    n = int(getattr(getattr(env, "unwrapped", env), "max_episode_steps", C.MAX_EPISODE_STEPS))
    if episode_length is not None and int(episode_length) != n:
        raise ValueError("episode_length ortamin bolum uzunlugu (%d) olmali" % n)
    env = VmapAutoReset(env)
    if bootstrap_on_timeout:
        env = _NoTruncation(env)
    return env
