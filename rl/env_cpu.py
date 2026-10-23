# -*- coding: utf-8 -*-
"""
CPU arka ucu: MuJoCo C (python baglantilari) + numpy ile vektorlestirilmis gorev matematigi.

Sozlesme: rl/TASK.md ("CPU backend API", "Control step", "Commands", "Reset" ve "Tasks main and getup").
Her ortamin kendi MjModel kopyasi vardir (bolum basinda NOMINAL degerlerden alan rastgelelestirme).
Ortamlar isci sureclere bitisik dilimler halinde dagitilir; aksiyon/gozlem/odul paylasimli bellekten,
senkronizasyon her isci icin bir Pipe uzerinden tek baytlik mesajlarla yapilir.
num_workers=0 her seyi ayni surecte calistirir (mj_data(i)/mj_model(i) ile goruntuleme icin).

Gorevler (task=, rl/tasks.py): "walk" (varsayilan; davranis, RNG akislari ve ciktilar bugunku ile
bit-duzeyinde ayni), "main" (yurume + govde yuksekligi + otur/kalk; 5 boyutlu komut, durus filtresi,
oturus bankasindan sifirlama) ve "getup" (14 aksiyon, dusmus/dusus-ortasi bankalari, dusme
sonlandirmasi yok, 8 s). Poz bankalari (rl/pose_bank.py) ebeveyn surecte BIR KEZ uretilir ve iscilere
gecirilir: her iscide birebir ayni banka, sonuclar isci sayisindan bagimsiz.

Robot profili (robot=, varsayilan config.ROBOT_PROFILE / OPENDUCK_ROBOT): "xl330_27" yukaridaki davranis aynen.
"tr35_b2" (rl/actuators.py, rl/sensors.py): fizik alt adim alt adim (Python dongusu; MJX ile ayni sira): gecikmeli
hedef, PWM kuantalama, olu bant, LEG ray CC olceklemesi; her alt adimda pot/IMU algilamasi. Aktor gozlemi sensor
modelinden (q/qd pot, gyro, tamamlayici filtre gravity), oduller/kritik gercek durumdan; genel OBS_NOISE kapali.
RNG: bolum basina sifirlama blogundan SONRA rng.random(actuators.n_uniform(nu) + sensors.n_episode(14)), her adimda
U blogundan SONRA rng.random(N_SUBSTEPS * sensors.n_tick(14)). backlash=True: DS bosluk eklemleri (bankalar
robot_model.expand_state ile genisletilir).
"""
import copy
import multiprocessing as mp
import os
import signal
import sys
import time
import traceback
import weakref

import mujoco
import numpy as np

from . import actuators as ACT
from . import config as C
from . import pose_bank as PB
from . import sensors as SENS
from . import task_math as T
from . import tasks as TK
from .robot_model import RobotIndex, build_spec, expand_state

REW_KEYS = list(C.REWARD_WEIGHTS)       # walk odul terimleri (geri uyum; gorev bazli: VecEnv.reward_keys)
CMD_STEPS = int(round(C.COMMAND_RESAMPLE_S / C.CTRL_DT))
_DR = C.DOMAIN_RAND
_HR = C.HEAD_RANDOM
_N_EXTRA_U = {"walk": 0, "main": 2 + T.NUM_SEGMENT_U, "getup": 2}   # walk blogundan SONRA cekilen sifirlama sayilari


def episode_columns(task="walk"):
    """info["episodes"] / ep_stats sutunlari: ep_len, rew_<terim>..., bolum metrikleri (main/getup),
    reset_kind, bank_label (main/getup; bank_label = pose_bank.LABELS indeksi, bankasiz sifirlamada -1)."""
    t = TK.get_task(task)
    cols = ["ep_len"] + ["rew_" + k for k in t.reward_keys] + list(t.episode_metrics)
    return cols + (["reset_kind", "bank_label"] if t.name != "walk" else [])


# --- paylasimli tampon duzeni ------------------------------------------------------
def _layout(n, task="walk"):
    t = TK.get_task(task)
    fields = [
        ("actions", np.float32, (n, t.num_actions)),
        ("cmd_in", np.float64, (n, max(t.num_commands, 1))),
        ("obs", np.float32, (n, t.num_obs)),
        ("critic", np.float32, (n, t.num_critic_obs)),
        ("rew", np.float32, (n,)),
        ("done", np.bool_, (n,)),
        ("time_outs", np.bool_, (n,)),
        ("ep_stats", np.float64, (n, len(episode_columns(t)))),   # biten bolumler (episode_columns sirasi)
        ("ctl", np.float64, (1,)),                                 # [mufredat ilerlemesi p] (dilimlenmez)
    ]
    out, off = [], 0
    for name, dt, shape in fields:
        nbytes = int(np.prod(shape)) * np.dtype(dt).itemsize
        out.append((name, dt, shape, off, nbytes))
        off += (nbytes + 63) // 64 * 64
    return out, off


def _views(buf, n, task="walk"):
    lay, size = _layout(n, task)
    raw = np.frombuffer(buf, dtype=np.uint8, count=size)
    return {name: raw[off:off + nb].view(dt).reshape(shape) for name, dt, shape, off, nb in lay}


def physics_model(scene_path=None, textures=False, variant="walk", robot=None, backlash=None):
    """robot_model.build_model(variant=, robot=, backlash=) ile ayni fizik; textures=False ise gok/zemin dokulari
    atilir (modeli ~5 MB'tan ~40 KB'a indirir, her ortamin kendi kopyasi oldugu icin onemli)."""
    spec = build_spec(scene_path, variant=variant, robot=robot, backlash=backlash)
    if not textures:
        for mt in spec.materials:
            mt.textures = [""] * len(mt.textures)
        for t in list(spec.textures):
            spec.delete(t)
    return spec.compile()


def reset_spec(task, reset_probs=None):
    """TaskSpec'in sifirlama karisimi degistirilmis kopyasi (oynatma/degerlendirme; or. {"home": 1.0}).
    Verilmeyen turler 0; olasiliklar normalize edilir; tur sirasi gorevinki."""
    t = TK.get_task(task)
    if reset_probs is None:
        return t
    kinds = [k for k, _ in t.reset_probs]
    bad = sorted(set(reset_probs) - set(kinds))
    if bad:
        raise ValueError("%s gorevinde sifirlama turu yok: %s (gecerli: %s)" % (t.name, bad, kinds))
    p = np.array([float(reset_probs.get(k, 0.0)) for k in kinds])
    if np.any(p < 0) or p.sum() <= 0:
        raise ValueError("gecersiz reset_probs %r" % (reset_probs,))
    t2 = copy.copy(t)
    t2.reset_probs = tuple(zip(kinds, (p / p.sum()).tolist()))
    return t2


def load_banks(task, reset_probs=None, bank_n=None, variant=None):
    """Gorevin olasiligi > 0 olan bankali sifirlama turleri icin poz bankalari {tur: {qpos, qvel, ctrl, label}}.
    bank_n: None (config boyutlari), int ya da {tur: n} (testler / hizli oynatma icin onek)."""
    t = reset_spec(task, reset_probs)
    variant = variant or t.model_variant
    out = {}
    for kind, p in t.reset_probs:
        if p <= 0 or kind not in PB.KINDS:
            continue
        n = bank_n.get(kind) if isinstance(bank_n, dict) else bank_n
        b = PB.bank(kind, n)
        PB.check_variant(b, variant)
        out[kind] = {k: b[k] for k in ("qpos", "qvel", "ctrl", "label")}
    return out


# --- temas -----------------------------------------------------------------------
def foot_contacts(geom, dist, env_id, n, foot_geom, floor_geom):
    """(n,2) 0/1: ayak i zemine degiyor (dist < CONTACT_EPS). geom (K,2), dist (K,), env_id (K,)
    tum ortamlarin temaslari art arda eklenmis halde."""
    out = np.zeros((n, 2))
    if len(dist):
        ok = dist < C.CONTACT_EPS
        g0, g1 = geom[:, 0], geom[:, 1]
        other = np.where(g0 == floor_geom, g1, np.where(g1 == floor_geom, g0, -1))
        for f in range(2):
            hit = ok & (other == foot_geom[f])
            out[env_id[hit], f] = 1.0
    return out


def floor_flags(geom, dist, env_id, n, groups, floor_geom):
    """(n, len(groups)) 0/1: gruptaki geomlardan biri zemine degiyor (dist < CONTACT_EPS; ayak kuraliyla ayni)."""
    out = np.zeros((n, len(groups)))
    if len(dist):
        ok = dist < C.CONTACT_EPS
        g0, g1 = geom[:, 0], geom[:, 1]
        other = np.where(g0 == floor_geom, g1, np.where(g1 == floor_geom, g0, -1))
        for k, gs in enumerate(groups):
            hit = ok & np.isin(other, gs)
            out[env_id[hit], k] = 1.0
    return out


def data_foot_contacts(data, idx):
    """Tek bir MjData icin (2,) ayak-zemin temasi."""
    c = data.contact
    return foot_contacts(c.geom, c.dist, np.zeros(data.ncon, dtype=np.int64), 1,
                         idx.foot_geom, idx.floor_geom)[0]


def contact_groups(idx):
    """floor_flags gruplari: [sol ayak, sag ayak, govde (trunk_floor), kafa (head_floor)]."""
    return [idx.foot_geom[:1], idx.foot_geom[1:], idx.trunk_geoms, idx.head_geoms]


def data_floor_flags(data, idx):
    """Tek bir MjData icin (4,): [sol ayak, sag ayak, trunk_floor, head_floor]."""
    c = data.contact
    return floor_flags(c.geom, c.dist, np.zeros(data.ncon, dtype=np.int64), 1, contact_groups(idx),
                       idx.floor_geom)[0]


# --- bir isci dilimi ----------------------------------------------------------------
class _EnvSlice(object):
    """VecEnv'in [lo, hi) ortamlari. io: tam boy diziler (paylasimli ya da yerel)."""

    def __init__(self, num_envs, lo, hi, seed, flags, io, scene_path=None, textures=False, banks=None):
        self.n = n = hi - lo
        self.io = {k: (v if k == "ctl" else v[lo:hi]) for k, v in io.items()}
        self.spec = spec = TK.get_task(flags.get("task", "walk"))
        self.task = spec.name
        variant = flags.get("model_variant") or spec.model_variant
        self.robot = ACT.profile(flags.get("robot"))
        self.tr35 = self.robot == ACT.TR35
        self.dr = ACT.domain_rand(self.robot)           # xl330: config.DOMAIN_RAND
        base = physics_model(scene_path, textures, variant, self.robot, flags.get("backlash"))
        self.idx = idx = RobotIndex(base, list(spec.action_joints))
        self.models = [copy.copy(base) for _ in range(n)]
        self.datas = [mujoco.MjData(m) for m in self.models]
        seqs = np.random.SeedSequence(seed).spawn(num_envs)[lo:hi]
        self.rngs = [np.random.Generator(np.random.PCG64(s)) for s in seqs]

        self.randomize = bool(flags["randomize"]) and self.dr["enabled"]
        self.push = bool(flags["push"]) and self.dr["enabled"]
        self.noise = bool(flags["noise"])
        self.noise_obs = self.noise and not self.tr35    # genel OBS_NOISE (tr35: sensor modeli)
        self.head_random = bool(flags["head_random"]) and spec.head_mode != "policy"   # getup: bas politikada
        self.terminate = bool(flags.get("terminate", True))
        self.max_steps = int(flags.get("max_episode_steps") or spec.max_episode_steps)

        assert np.all(base.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT)
        fj = int(np.flatnonzero(base.jnt_type == mujoco.mjtJoint.mjJNT_FREE)[0])
        self.qadr, self.dadr = int(base.jnt_qposadr[fj]), int(base.jnt_dofadr[fj])
        self.act_dof = base.jnt_dofadr[base.actuator_trnid[:, 0]]      # aktuatorlu dof'lar (16)
        self.sts_act = idx.obs_act_id                                   # 14 politika/bas servosu (XL330)
        self.fric_geoms = np.array(list(idx.foot_geom) + [idx.floor_geom])
        self.nom = {k: getattr(base, k).copy() for k in (
            "geom_friction", "body_mass", "body_inertia", "body_ipos", "actuator_gainprm",
            "actuator_biasprm", "dof_damping", "dof_frictionloss", "dof_armature")}

        self.ar = np.arange(n)
        self.ctrl0 = idx.init_ctrl.copy()
        self.default_head = idx.init_ctrl[idx.head_act_id].copy()
        self.num_obs, A = spec.num_obs, spec.num_actions
        self.noise_vec = np.asarray(T.noise_vector(np, self.task), dtype=np.float64)
        self.K = int(self.dr["action_delay_steps"][1]) if self.randomize else 0
        if self.task == "getup":
            self.act_scale = TK.action_scale_vector(spec, idx.default_action_q, idx.action_lo, idx.action_hi)

        # adim basina cekilen duzgun sayilarin yerlesimi (her ortam her adimda ayni miktari tuketir)
        self.o_noise, nu = 0, (self.num_obs if self.noise_obs else 0)
        self.o_head, nu = nu, nu + (6 if self.head_random else 0)
        self.o_push, nu = nu, nu + (3 if self.push else 0)
        self.n_uniform = nu
        self.U = np.zeros((n, max(nu, 1)))
        self._last_noise = np.zeros((n, self.num_obs))

        # ham durum (toplu)
        self.SD = np.zeros((n, base.nsensordata))
        self.QPOS = np.zeros((n, base.nq))
        self.QVEL = np.zeros((n, base.nv))
        self.QACC = np.zeros((n, base.nv))
        self.AF = np.zeros((n, base.nu))
        self.TZ = np.zeros(n)
        self.TIME = np.zeros(n)
        self.CONTACT = np.zeros((n, 2))
        self.TRUNKF = np.zeros(n)          # main/getup: govde kutusu-zemin
        self.HEADF = np.zeros(n)           # main/getup: kafa geomlari-zemin
        self.groups = contact_groups(idx)
        # head_force_peak (istege bagli, degerlendirme): fizik tek tek alt adimlarla; son kontrol adimindaki alt
        # adimlarin kafa geomlari-zemin normal kuvvet toplaminin en buyugu (N). Dinamik ayni (mj_step(m, d, k) ==
        # k kez mj_step(m, d)); kapaliyken bu yol hic calismaz (walk/egitim bit-duzeyinde ayni).
        self.head_force_peak = bool(flags.get("head_force_peak", False))
        self.HFPEAK = np.zeros(n)
        self._is_head = np.zeros(base.ngeom, bool)
        self._is_head[idx.head_geoms] = True
        self._f6 = np.zeros(6)
        if self.tr35:
            self._tr35_init(base)

        # ortam durumu
        self.phase = np.zeros(n)
        self.step_count = np.zeros(n, dtype=np.int64)
        self.a_prev = np.zeros((n, A))
        self.air_time = np.zeros((n, 2))
        self.command = np.zeros((n, max(spec.num_commands, 1)))
        self.fixed_cmd = None
        self.head_offset = np.zeros((n, len(idx.head_act_id)))
        self.head_target = np.zeros_like(self.head_offset)
        self.head_timer = np.zeros(n)
        self.push_timer = np.zeros(n)
        self.friction = np.ones(n)
        self.delay = np.zeros(n, dtype=np.int64)
        self.abuf = np.zeros((n, self.K + 1, A))
        self.ptr = 0
        self.rkeys = list(spec.reward_keys)
        self.mkeys = list(spec.episode_metrics)
        self.ep_sum = np.zeros((n, len(self.rkeys)))
        self.ep_met = np.zeros((n, len(self.mkeys)))
        # main: durus filtresi, segment, olaylar; getup: onceki adim bellegi, olaylar
        self.h_ref = np.full(n, C.H_HOME)
        self.alpha = np.zeros(n)
        self.seg = np.zeros(n)
        self.seg_timer = np.zeros(n)
        z = np.zeros(n)
        self.ev = T.main_events_init(np, z) if self.task == "main" else T.getup_events_init(np, z)
        self.ev = {k: np.array(v) for k, v in self.ev.items()}
        self.mem = {}                      # getup bellegi: anahtarlar T.getup_memory'den (sabit liste yok)
        if self.task == "getup":
            za = np.zeros((n, spec.num_actions))
            with np.errstate(all="ignore"):
                self.mem = {k: z.copy() for k in T.getup_memory(np, self._state(za, za))}
        self.reset_kind = np.zeros(n)
        self.bank_label = np.full(n, -1.0)

        # sifirlama karisimi ve bankalar
        self.rspec = reset_spec(spec, flags.get("reset_probs"))
        self.kinds = [k for k, _ in self.rspec.reset_probs]
        self.banks = banks or {}
        if len(idx.play_jnt):                 # bosluk eklemli model: bankalar bosluksuz duzende
            self.banks = {k: dict(b, **dict(zip(("qpos", "qvel"), expand_state(base, b["qpos"], b["qvel"]))))
                          for k, b in self.banks.items()}
        for kind, p in self.rspec.reset_probs:
            if p > 0 and kind in PB.KINDS and kind not in self.banks:
                raise ValueError("%s sifirlamasi icin poz bankasi verilmedi" % kind)

        self._dv = [(m, d, d.ctrl, d.qvel, d.sensordata, d.qpos, d.qacc, d.actuator_force, d.xpos)
                    for m, d in zip(self.models, self.datas)]
        self.cmd_lo = np.array([C.COMMAND_RANGES[k][0] for k in ("vx", "vy", "wz")])
        self.cmd_hi = np.array([C.COMMAND_RANGES[k][1] for k in ("vx", "vy", "wz")])
        self.n_reset_u = len(idx.obs_qpos_adr) + 10 + _N_EXTRA_U[self.task]
        if self.task == "walk":
            self.weights = None                 # total_reward varsayilani (C.REWARD_WEIGHTS)
            self.w_dt = np.array([C.REWARD_WEIGHTS[k] for k in REW_KEYS]) * C.CTRL_DT
        else:
            self.set_progress(flags.get("progress", 0.0))
        self._setup_dr(base)

    # -- tr35: aktuator + sensor modeli (rl/actuators.py, rl/sensors.py)
    def _tr35_init(self, base):
        idx, n = self.idx, self.n
        t, nu, no = idx.act, base.nu, len(idx.obs_qpos_adr)
        self.n_ue_act = ACT.n_uniform(nu)
        self.n_ue = self.n_ue_act + SENS.n_episode(no)
        self.n_tick = SENS.n_tick(no)
        self.n_us = C.N_SUBSTEPS * self.n_tick
        self.US = np.zeros((n, self.n_us))
        self.ap = ACT.sample_params(np, np.zeros((n, self.n_ue_act)), t, False)     # bolum: tau_s, db_half, ...
        self.sp = SENS.episode(np, np.zeros((n, self.n_ue - self.n_ue_act)), idx.pot, False)
        self.mprm = {k: np.zeros((n, nu)) for k in ("gain", "b0", "b1", "b2", "damping")}
        self.ctrl_prev = np.zeros((n, nu))
        z = np.zeros((n, no))
        self.pot_st = SENS.pot_reset(np, z, z, self.sp, idx.pot)
        self.imu_st = {"g": np.zeros((n, 3)), "w": np.zeros((n, 3))}
        self.met = ACT.metrics_init(np, np.zeros((n, nu)))
        self._lim_type = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
        for j in range(n):
            self._tr35_model_prm(j)

    def _tr35_model_prm(self, j):
        m, ad = self.models[j], self.idx.act["act_dadr"]
        mp = self.mprm
        mp["gain"][j] = m.actuator_gainprm[:, 0]
        mp["b0"][j], mp["b1"][j], mp["b2"][j] = (m.actuator_biasprm[:, i] for i in range(3))
        mp["damping"][j] = m.dof_damping[ad]

    def _tr35_episode(self, j, ue):
        """Bolum parametreleri (DR randomize'a, sensor biaslari noise'a bagli); randomize ise model alanlari +
        mj_setConst (qpos ayarlanmadan ONCE)."""
        t, m = self.idx.act, self.models[j]
        ap = ACT.sample_params(np, ue[:self.n_ue_act], t, self.randomize)
        for k, v in ap.items():
            self.ap[k][j] = v
        for k, v in SENS.episode(np, ue[self.n_ue_act:], self.idx.pot, self.noise).items():
            self.sp[k][j] = v
        m.actuator_forcerange[:, 0] = -ap["tau_s"]
        m.actuator_forcerange[:, 1] = ap["tau_s"]
        if self.randomize:
            ad = t["act_dadr"]
            m.actuator_gainprm[:, 0] = ap["kp"]
            m.actuator_biasprm[:, 1] = -ap["kp"]
            m.dof_damping[ad] = ap["damping"]
            m.dof_armature[ad] = ap["armature"]
            m.dof_frictionloss[ad] = ap["frictionloss"]
            pj = t["play_jnt"] >= 0
            if pj.any():
                h = 0.5 * ap["backlash"][pj]
                m.jnt_range[t["play_jnt"][pj]] = np.stack([-h, h], axis=-1)
            mujoco.mj_setConst(m, self.datas[j])
        self._tr35_model_prm(j)

    def _tr35_sensor_reset(self, j, ctrl):
        """Sifirlama / set_state sonrasi (fiziksel durum toplandiktan sonra) sensor ve aktuator durumu."""
        idx, sl = self.idx, slice(j, j + 1)
        sp = {k: v[sl] for k, v in self.sp.items()}
        q, qd = self.QPOS[sl][:, idx.obs_qpos_adr], self.QVEL[sl][:, idx.obs_dof_adr]
        for k, v in SENS.pot_reset(np, q, qd, sp, idx.pot).items():
            self.pot_st[k][j] = v[0]
        for k, v in SENS.imu_reset(np, self.SD[sl][:, idx.s_quat], self.SD[sl][:, idx.s_gyro], sp).items():
            self.imu_st[k][j] = v[0]
        self.ctrl_prev[j] = ctrl
        for k, v in self.met.items():
            v[j] = 1.0 if k == "cc" else 0.0

    def _head_force_now(self, m, d):
        """Kafa geomlari - zemin temaslarinin mj_contactForce normal kuvvet toplami (N)."""
        if not d.ncon:
            return 0.0
        fl, hd, f6 = self.idx.floor_geom, self._is_head, self._f6
        g = d.contact.geom
        hit = np.flatnonzero(((g[:, 0] == fl) & hd[g[:, 1]]) | ((g[:, 1] == fl) & hd[g[:, 0]]))
        f = 0.0
        for c in hit:
            mujoco.mj_contactForce(m, d, int(c), f6)
            f += abs(f6[0])
        return f

    def _tr35_substeps(self, ctrl_new):
        """N_SUBSTEPS alt adim, ortamlar alt adim basina topluca (actuators / sensors sirasi; MJX ile ayni)."""
        idx, n, nsub = self.idx, self.n, C.N_SUBSTEPS
        t, P, mp, ap = idx.act, idx.pot, self.mprm, self.ap
        aq, ad, oq, oa = t["act_qadr"], t["act_dadr"], idx.obs_qpos_adr, idx.obs_act_id
        nu, no = len(aq), len(oq)
        QA, VA, AF = np.empty((n, nu)), np.empty((n, nu)), np.empty((n, nu))
        QO, GY, AC = np.empty((n, no)), np.empty((n, 3)), np.empty((n, 3))
        met = ACT.metrics_init(np, np.zeros((n, nu)))
        lim, j2a, LT = met["limit"], t["jnt2act"], self._lim_type
        us = self.US.reshape(n, nsub, self.n_tick)
        hfp, sg, sa = self.head_force_peak, idx.s_gyro, idx.s_acc
        if hfp:
            self.HFPEAK[:] = 0.0
        tn = None
        for k in range(nsub):
            for j, (m, d, cv, qv, sd, qp, qa, af, xp) in enumerate(self._dv):
                QA[j] = qp[aq]
                VA[j] = qv[ad]
            c, frc, dem, sc = ACT.shape_substep(np, k, self.ctrl_prev, ctrl_new, QA, VA, ap, t, mp["gain"],
                                                mp["b0"], mp["b1"], mp["b2"], mp["damping"])
            met["rail"] = met["rail"] + dem
            met["rail_peak"] = np.maximum(met["rail_peak"], dem)
            met["cc"] = np.minimum(met["cc"], sc)
            for j, (m, d, cv, qv, sd, qp, qa, af, xp) in enumerate(self._dv):
                fr = m.actuator_forcerange
                fr[:, 0] = -frc[j]
                fr[:, 1] = frc[j]
                cv[:] = c[j]
                mujoco.mj_step(m, d)
                if hfp:
                    self.HFPEAK[j] = max(self.HFPEAK[j], self._head_force_now(m, d))
                QO[j] = qp[oq]
                AF[j] = af
                GY[j] = sd[sg]
                AC[j] = sd[sa]
                ne = d.nefc
                if ne:
                    sel = d.efc_type[:ne] == LT
                    if sel.any():
                        a = j2a[d.efc_id[:ne][sel]]
                        ok = a >= 0
                        np.maximum.at(lim[j], a[ok], np.abs(d.efc_force[:ne][sel][ok]))
            tn, cur, rail = ACT.after_substep(np, AF, VA, mp["damping"], ap, t)
            self.pot_st = SENS.pot_tick(np, self.pot_st, QO, cur[:, oa], rail, us[:, k, :no], self.sp, P, self.noise)
            self.imu_st = SENS.imu_tick(np, self.imu_st, GY, AC, rail, us[:, k, no:], self.sp, self.noise)
        met["rail"] = met["rail"] / nsub
        met["tau_net"] = tn
        self.met = met
        self.ctrl_prev = np.array(ctrl_new, dtype=np.float64)

    # -- mufredat (getup gec terimleri; walk/main agirliklari sabit)
    def set_progress(self, p):
        if self.task == "walk":
            return
        self.progress = float(p)
        w = T.reward_weights(np, self.task, self.progress if self.task == "getup" else None)
        self.weights = {k: float(v) for k, v in w.items()}
        self.w_dt = np.array([self.weights[k] for k in self.rkeys]) * C.CTRL_DT

    # -- komutlar
    def _sample_command(self, rng):
        r = rng.random(4)
        if r[0] < C.COMMAND_ZERO_PROB:
            return np.zeros(3)
        return self.cmd_lo + (self.cmd_hi - self.cmd_lo) * r[1:]

    def set_commands(self, cmd):
        if cmd is None:
            self.fixed_cmd = None
        elif self.task == "walk":
            self.fixed_cmd = np.array(cmd, dtype=np.float64).reshape(self.n, 3)
            self.command[:] = self.fixed_cmd
        else:
            c = np.asarray(cmd, dtype=np.float64).reshape(self.n, -1)
            self.fixed_cmd = np.array(TK.pad_command(self.task, c))
            self.command[:] = self.fixed_cmd

    # -- alan rastgelelestirme (her zaman nominalden; tek bir rng cagrisi)
    def _setup_dr(self, base):
        nb, nad, nk = base.nbody - 1, len(self.act_dof), len(self.sts_act)
        parts = [("friction", 1, self.dr["friction"]), ("mass", nb, self.dr["link_mass_scale"]),
                 ("trunk_add", 1, self.dr["trunk_added_mass"]),
                 ("com", 3, (-self.dr["trunk_com_offset"], self.dr["trunk_com_offset"])),
                 ("kp", nk, self.dr["kp_scale"]), ("damping", nad, self.dr["damping_scale"]),
                 ("frictionloss", nad, self.dr["frictionloss_scale"]), ("armature", nad, self.dr["armature_scale"]),
                 ("delay", 1, (0.0, 1.0))]
        self.dr_sl, lo, hi, o = {}, [], [], 0
        for name, k, (a, b) in parts:
            self.dr_sl[name] = slice(o, o + k)
            lo += [a] * k
            hi += [b] * k
            o += k
        self.dr_lo, self.dr_span = np.array(lo), np.array(hi) - np.array(lo)

    def _randomize(self, j):
        m, nom, sl, tb = self.models[j], self.nom, self.dr_sl, self.idx.trunk_body
        v = self.dr_lo + self.dr_span * self.rngs[j].random(len(self.dr_lo))
        f = v[0]
        gf = nom["geom_friction"].copy()
        gf[self.fric_geoms, 0] = f
        m.geom_friction[:] = gf

        scale = np.ones(len(nom["body_mass"]))
        scale[1:] = v[sl["mass"]]
        mass = nom["body_mass"] * scale
        inertia = nom["body_inertia"] * scale[:, None]
        new = mass[tb] + v[sl["trunk_add"]][0]
        inertia[tb] *= new / mass[tb]
        mass[tb] = new
        m.body_mass[:] = mass
        m.body_inertia[:] = inertia
        ipos = nom["body_ipos"].copy()
        ipos[tb] += v[sl["com"]]
        m.body_ipos[:] = ipos

        kp = nom["actuator_gainprm"][self.sts_act, 0] * v[sl["kp"]]
        gp, bp = nom["actuator_gainprm"].copy(), nom["actuator_biasprm"].copy()
        gp[self.sts_act, 0] = kp
        bp[self.sts_act, 1] = -kp
        m.actuator_gainprm[:] = gp
        m.actuator_biasprm[:] = bp

        ad = self.act_dof
        for field in ("dof_damping", "dof_frictionloss", "dof_armature"):
            x = nom[field].copy()
            x[ad] *= v[sl[field[4:]]]
            getattr(m, field)[:] = x

        lo, hi = self.dr["action_delay_steps"]
        self.delay[j] = min(lo + int(v[sl["delay"]][0] * (hi - lo + 1)), hi)
        self.friction[j] = f
        mujoco.mj_setConst(m, self.datas[j])

    # -- sifirlama
    def _gather(self, j):
        m, d, _, qv, sd, qp, qa, af, xp = self._dv[j]
        self.SD[j] = sd
        self.QPOS[j] = qp
        self.QVEL[j] = qv
        self.QACC[j] = qa
        self.AF[j] = af
        self.TZ[j] = xp[self.idx.trunk_body, 2]
        self.TIME[j] = d.time

    def _contacts_one(self, j):
        d, idx = self.datas[j], self.idx
        if self.task == "walk":
            self.CONTACT[j] = data_foot_contacts(d, idx)
            return
        f = data_floor_flags(d, idx)
        self.CONTACT[j] = f[:2]
        self.TRUNKF[j], self.HEADF[j] = f[2], f[3]

    def _task_reset(self, j, alpha0):
        """Gorev durumunun bolum basi degerleri (fiziksel durum _gather ile toplandiktan sonra)."""
        if self.task == "main":
            self.alpha[j], self.h_ref[j] = alpha0, C.H_HOME
            for k, v in T.main_events_init(np, np.array([alpha0])).items():
                self.ev[k][j] = v[0]
        elif self.task == "getup":
            za = np.zeros((1, self.spec.num_actions))
            sj = self._state(za, za, rows=slice(j, j + 1))          # ortam j'nin tam durumu (MJX: _task_state)
            for k, v in T.getup_memory(np, sj).items():
                self.mem.setdefault(k, np.zeros(self.n))[j] = np.reshape(v, -1)[0]
            for k, v in T.getup_events_init(np, self.TZ[j:j + 1]).items():
                self.ev[k][j] = v[0]

    def _reset_env(self, j):
        rng, m, d, idx = self.rngs[j], self.models[j], self.datas[j], self.idx
        if self.randomize:
            self._randomize(j)
        # [eklem gurultusu(14), yaw, taban hizi(2), faz, komut(4), bas zamanlayici, itme zamanlayici]
        # + main: [tur, banka indeksi, segment(8)]; getup: [tur, banka indeksi]
        u = rng.random(self.n_reset_u)
        if self.tr35:
            self._tr35_episode(j, rng.random(self.n_ue))
        nj = len(idx.obs_qpos_adr)
        mujoco.mj_resetData(m, d)
        kind, name, bank, label = 0, "home", None, -1.0
        if self.task != "walk":
            ux = u[nj + 10:]
            kind = int(T.sample_reset_kind(np, ux[0], self.rspec))
            name = self.kinds[kind]
            bank = self.banks.get(name)
        yaw = np.pi * (2.0 * u[nj] - 1.0)
        a = self.qadr
        if bank is None:
            qpos = idx.init_qpos.copy()
            qpos[idx.obs_qpos_adr] += C.RESET_JOINT_NOISE * (2.0 * u[:nj] - 1.0)
            qpos[a:a + 2] = 0.0
            q = np.empty(4)
            mujoco.mju_mulQuat(q, np.array([np.cos(0.5 * yaw), 0.0, 0.0, np.sin(0.5 * yaw)]),
                               idx.init_qpos[a + 3:a + 7])
            qpos[a + 3:a + 7] = q
            d.qpos[:] = qpos
            d.qvel[self.dadr:self.dadr + 2] = C.RESET_BASE_VEL_NOISE * (2.0 * u[nj + 1:nj + 3] - 1.0)
            d.ctrl[:] = idx.init_ctrl
        else:
            i = int(T.bank_index(np, ux[1], len(bank["qpos"])))
            qpos = np.array(bank["qpos"][i], dtype=np.float64)
            qpos[a:a + 2] = 0.0
            qpos[a + 3:a + 7] = T.quat_mul(np, T.yaw_quat(np, np.asarray(yaw)), bank["qpos"][i, a + 3:a + 7])
            d.qpos[:] = qpos
            d.qvel[:] = bank["qvel"][i]
            d.ctrl[:] = bank["ctrl"][i]
            label = float(bank["label"][i])
        mujoco.mj_forward(m, d)
        self._gather(j)
        self._contacts_one(j)
        if self.tr35:
            self._tr35_sensor_reset(j, d.ctrl)

        self.phase[j] = u[nj + 3]
        if self.task == "walk":
            if self.fixed_cmd is not None:
                self.command[j] = self.fixed_cmd[j]
            elif u[nj + 4] < C.COMMAND_ZERO_PROB:
                self.command[j] = 0.0
            else:
                self.command[j] = self.cmd_lo + (self.cmd_hi - self.cmd_lo) * u[nj + 5:nj + 8]
        elif self.task == "main":
            # ilk segment her zaman zaten cekilmis sayilardan (MJX ile ayni: seg/seg_timer kilitliyken de
            # orneklenir, ek RNG yok); kilitli komut yalniz komutu ezer
            alpha0 = 1.0 if name == "seated" else 0.0
            al = np.array([alpha0])
            kd, cmd, dur = T.sample_segment(np, ux[None, 2:2 + T.NUM_SEGMENT_U], T.reset_command(np, al), seated=al)
            self.seg[j], self.command[j], self.seg_timer[j] = kd[0], cmd[0], dur[0]
            if self.fixed_cmd is not None:
                self.command[j] = self.fixed_cmd[j]
            self._task_reset(j, alpha0)
        else:
            self._task_reset(j, 0.0)
        self.reset_kind[j], self.bank_label[j] = kind, label
        self.step_count[j] = 0
        self.a_prev[j] = 0.0
        self.air_time[j] = 0.0
        self.ep_sum[j] = 0.0
        self.ep_met[j] = 0.0
        self.abuf[j] = 0.0
        self.head_offset[j] = 0.0
        self.head_target[j] = 0.0
        lo, hi = _HR["resample_s"]
        self.head_timer[j] = lo + (hi - lo) * u[nj + 8]
        lo, hi = self.dr["push_interval_s"]
        self.push_timer[j] = lo + (hi - lo) * u[nj + 9]

    def set_state(self, j, qpos, qvel, ctrl=None, warmstart=None):
        """Ortam j'yi verilen fiziksel duruma koyar (oynatma: politika devri). Model (DR) korunur; bolum
        sayaclari, aksiyon gecmisi (last_action = 0), durus filtresi (alpha = 0, h_ref = H_HOME), getup bellegi
        ve olaylar yeni durumdan baslar. Gozlem icin ardindan refresh_obs()."""
        m, d = self.models[j], self.datas[j]
        mujoco.mj_resetData(m, d)
        d.qpos[:] = qpos
        d.qvel[:] = qvel
        d.ctrl[:] = self.ctrl0 if ctrl is None else ctrl
        if warmstart is not None:
            d.qacc_warmstart[:] = warmstart
        mujoco.mj_forward(m, d)
        self._gather(j)
        self._contacts_one(j)
        if self.tr35:
            self._tr35_sensor_reset(j, d.ctrl)
        self._task_reset(j, 0.0)
        self.reset_kind[j], self.bank_label[j] = -1.0, -1.0
        self.step_count[j] = 0
        self.a_prev[j] = 0.0
        self.air_time[j] = 0.0
        self.ep_sum[j] = 0.0
        self.ep_met[j] = 0.0
        self.abuf[j] = 0.0
        self.head_offset[j] = 0.0
        self.head_target[j] = 0.0

    # -- gozlem
    def _state(self, action, last_action, rows=None):
        """task_math "s" sozlugu (toplu); rows (or. slice(j, j + 1)): yalniz bu satirlar (getup sifirlama bellegi)."""
        idx = self.idx
        if rows is None:
            SD, TZ, QP, QV, QA, AF, CT, CM, PH, FR = (self.SD, self.TZ, self.QPOS, self.QVEL, self.QACC, self.AF,
                                                      self.CONTACT, self.command, self.phase, self.friction)
            HR, AL, TF, HF, MEM = self.h_ref, self.alpha, self.TRUNKF, self.HEADF, self.mem
        else:
            SD, TZ, QP, QV, QA, AF, CT, CM, PH, FR, HR, AL, TF, HF = (
                X[rows] for X in (self.SD, self.TZ, self.QPOS, self.QVEL, self.QACC, self.AF, self.CONTACT,
                                  self.command, self.phase, self.friction, self.h_ref, self.alpha, self.TRUNKF,
                                  self.HEADF))
            MEM = {k: v[rows] for k, v in self.mem.items()}
        s = {
            "quat": SD[:, idx.s_quat], "gyro": SD[:, idx.s_gyro], "linvel": SD[:, idx.s_linvel],
            "trunk_z": TZ,
            "q": QP[:, idx.obs_qpos_adr], "qd": QV[:, idx.obs_dof_adr],
            "qdd_legs": QA[:, idx.leg_dof_adr], "torque": AF[:, idx.obs_act_id],
            "foot_pos": np.stack([SD[:, sl] for sl in idx.s_foot_pos], axis=1),
            "foot_vel": np.stack([SD[:, sl] for sl in idx.s_foot_vel], axis=1),
            "contact": CT,
            "action": action, "last_action": last_action,
            "command": CM, "phase": PH,
            "friction": FR, "default_q": idx.default_obs_q,
        }
        if self.task == "main":
            s.update(h_ref=HR, alpha=AL, trunk_floor=TF, head_floor=HF)
        elif self.task == "getup":
            del s["command"], s["phase"]
            s.update(trunk_floor=TF, head_floor=HF, head_vel=SD[:, idx.s_head_vel], **MEM)
        if self.tr35:                      # olcut kancalari (gozlem eklemleri sirasi; tau_s bolum ornegi)
            r = slice(None) if rows is None else rows
            oa, met = idx.obs_act_id, self.met
            tn, ts = met["tau_net"][r][:, oa], self.ap["tau_s"][r][:, oa]
            s.update(tau_net=tn, tau_s=ts, tau_ratio=tn / ts, limit_torque=met["limit"][r][:, oa],
                     rail_demand=met["rail"][r], rail_peak=met["rail_peak"][r], cc_scale=met["cc"][r])
        return s

    def _obs_state(self, s):
        """tr35 aktor gozlemi durumu: q/qd pot, gyro olculen, gravity filtre ciktisi (sensors.obs_state)."""
        return SENS.obs_state(np, s, self.pot_st["v"], self.pot_st["qdf"], self.imu_st["w"], self.imu_st["g"])

    def _write_obs(self, noise_u):
        s = self._state(self.a_prev, self.a_prev)
        obs = T.observation(np, self._obs_state(s) if self.tr35 else s, self.task)
        if self.noise_obs:
            obs = obs + self.noise_vec * (2.0 * noise_u - 1.0)
            self._last_noise[:] = noise_u
        self.io["obs"][:] = obs
        self.io["critic"][:] = T.critic_observation(np, s, self.task)

    def refresh_obs(self):
        """Gozlemi mevcut durumdan yeniden yazar (set_commands / set_state sonrasi); gurultu acikken son
        adimin gurultu ornegi kullanilir (RNG tuketilmez)."""
        self._write_obs(self._last_noise)

    def task_state(self):
        """Gozlemin gordugu durumun kopyasi (last_action = action = son uygulanan aksiyon) + gorev durumu."""
        s = {k: np.array(v) for k, v in self._state(self.a_prev, self.a_prev).items()}
        s.update(step_count=self.step_count.copy(), reset_kind=self.reset_kind.copy(),
                 bank_label=self.bank_label.copy())
        if self.head_force_peak:            # son kontrol adiminin alt adim tepe kuvveti (sifirlamada korunur)
            s["head_force_peak"] = self.HFPEAK.copy()
        if self.tr35:                       # aktorun gordugu olcumler
            s.update(q_obs=self.pot_st["v"].copy(), qd_obs=self.pot_st["qdf"].copy(), gyro_obs=self.imu_st["w"].copy(),
                     gravity_obs=self.imu_st["g"].copy(), ctrl_prev=self.ctrl_prev.copy())
        if self.task == "main":
            s.update(seg=self.seg.copy(), seg_timer=self.seg_timer.copy(), locked=self.fixed_cmd is not None,
                     **{"ev_" + k: np.array(v) for k, v in self.ev.items()})
        elif self.task == "getup":
            s.update(**{"ev_" + k: np.array(v) for k, v in self.ev.items()})
        return s

    def reset_all(self):
        for j in range(self.n):
            self._reset_env(j)
        noise_u = np.zeros((self.n, self.num_obs))
        if self.noise_obs:
            for j, rng in enumerate(self.rngs):
                rng.random(out=noise_u[j])
        self._write_obs(noise_u)
        self.io["rew"][:] = 0.0
        self.io["done"][:] = False
        self.io["time_outs"][:] = False

    # -- aksiyon eslemesi (gorev bazli; TASK.md adim 1)
    def _targets(self, a, ref):
        idx = self.idx
        if self.task == "walk":
            return T.action_to_targets(np, a, idx.default_action_q)
        if self.task == "main":
            return T.action_to_targets(np, a, ref["q_ref"])
        return T.action_to_targets(np, a, idx.default_action_q, scale=self.act_scale, clip=self.spec.action_clip,
                                   lo=idx.action_lo, hi=idx.action_hi)

    def _substeps_head_peak(self, m, d, nsub):
        """nsub kez mj_step(m, d) (mj_step(m, d, nsub) ile bit-duzeyinde ayni); dondurur: alt adimlarin her birinden
        sonra kafa geomlari (idx.head_geoms) - zemin temaslarinin mj_contactForce normal kuvvet toplaminin (N) en
        buyugu (eval_tasks.head_force ile ayni tanim, kontrol adimi yerine her alt adimda)."""
        pk = 0.0
        for _ in range(nsub):
            mujoco.mj_step(m, d)
            pk = max(pk, self._head_force_now(m, d))
        return pk

    # -- kontrol adimi (TASK.md sirasiyla)
    def step(self):
        io, idx, n, task = self.io, self.idx, self.n, self.task
        U = self.U
        if self.n_uniform:
            for j, rng in enumerate(self.rngs):
                rng.random(out=U[j])
        if self.tr35:                       # alt adim algilama gurultusu (sabit yerlesim)
            for j, rng in enumerate(self.rngs):
                self.US[j] = rng.random(self.n_us)

        # 1. aksiyon -> hedefler (gecikmeli); main: guncel durustan q_ref
        ref = T.posture_ref(np, self.h_ref, self.alpha, idx.default_obs_q) if task == "main" else None
        targets, a = self._targets(io["actions"].astype(np.float64), ref)
        if self.K > 0:
            self.ptr = (self.ptr + 1) % (self.K + 1)
            self.abuf[:, self.ptr] = a
            a_del = self.abuf[self.ar, (self.ptr - self.delay) % (self.K + 1)]
            targets, _ = self._targets(a_del, ref)

        # 2-3. bas + bacak kontrolleri
        ctrl = np.empty((n, len(self.ctrl0)))
        ctrl[:] = self.ctrl0
        if self.head_random:
            uh = U[:, self.o_head:self.o_head + 6]
            self.head_timer -= C.CTRL_DT
            rs = self.head_timer <= 0.0
            if rs.any():
                tgt = np.where(uh[:, :1] < _HR["prob"], (2.0 * uh[:, 1:5] - 1.0) * idx.head_range, 0.0)
                lo, hi = _HR["resample_s"]
                self.head_target[rs] = tgt[rs]
                self.head_timer[rs] = lo + (hi - lo) * uh[rs, 5]
            if task == "main":      # rastgele bas hedefi yalniz ayaktayken (alpha < ALPHA_STAND_MAX)
                gate = np.where(self.alpha < C.ALPHA_STAND_MAX, 1.0, 0.0)[:, None]
                self.head_offset += (self.head_target * gate - self.head_offset) * (C.CTRL_DT / _HR["filter_tau"])
            else:
                self.head_offset += (self.head_target - self.head_offset) * (C.CTRL_DT / _HR["filter_tau"])
            ctrl[:, idx.head_act_id] = self.default_head + self.head_offset
        if task == "main":          # oturus scripti (HEAD_SIT) + rastgele bas
            ctrl[:, idx.head_act_id] = self.default_head + self.head_offset + ref["head_offset"]
        ctrl[:, idx.action_act_id] = targets
        np.clip(ctrl, idx.ctrl_lo, idx.ctrl_hi, out=ctrl)
        if self.tr35:                       # PCA9685 adimi (kontrol adiminda bir kez)
            ctrl = ACT.quantize_pwm(np, ctrl, idx.act["pwm_res"], idx.ctrl_lo, idx.ctrl_hi)

        # 4. itme (main: yalniz alpha < ALPHA_STAND_MAX iken uygulanir; sayilar her durumda tuketilir)
        if self.push:
            up = U[:, self.o_push:self.o_push + 3]
            self.push_timer -= C.CTRL_DT
            pm = np.flatnonzero(self.push_timer <= 0.0)
            if len(pm):
                dv = (2.0 * up[pm, :2] - 1.0) * self.dr["push_vel"]
                lo, hi = self.dr["push_interval_s"]
                self.push_timer[pm] = lo + (hi - lo) * up[pm, 2]
                for k, j in enumerate(pm):
                    if task != "main" or self.alpha[j] < C.ALPHA_STAND_MAX:
                        self._dv[j][3][self.dadr:self.dadr + 2] += dv[k]

        # 5. fizik + ham durum
        SD, QPOS, QVEL, QACC, AF, TZ, TIME = self.SD, self.QPOS, self.QVEL, self.QACC, self.AF, self.TZ, self.TIME
        tb, nsub, mj_step, hfp = idx.trunk_body, C.N_SUBSTEPS, mujoco.mj_step, self.head_force_peak
        gl, dl = [], []
        if self.tr35:
            self._tr35_substeps(ctrl)
        for j, (m, d, cv, qv, sd, qp, qa, af, xp) in enumerate(self._dv):
            if self.tr35:
                pass
            elif hfp:
                cv[:] = ctrl[j]
                self.HFPEAK[j] = self._substeps_head_peak(m, d, nsub)
            else:
                cv[:] = ctrl[j]
                mj_step(m, d, nsub)
            SD[j] = sd
            QPOS[j] = qp
            QVEL[j] = qv
            QACC[j] = qa
            AF[j] = af
            TZ[j] = xp[tb, 2]
            TIME[j] = d.time
            c = d.contact
            gl.append(c.geom)
            dl.append(c.dist)
        env_id = np.repeat(self.ar, [len(x) for x in dl])
        if task == "walk":
            self.CONTACT[:] = foot_contacts(np.concatenate(gl), np.concatenate(dl), env_id, n,
                                            idx.foot_geom, idx.floor_geom)
        else:
            f = floor_flags(np.concatenate(gl), np.concatenate(dl), env_id, n, self.groups, idx.floor_geom)
            self.CONTACT[:] = f[:, :2]
            self.TRUNKF[:] = f[:, 2]
            self.HEADF[:] = f[:, 3]

        # 6. faz (walk/main), sayac, durus filtresi (main: bu adimin komutuyla)
        if task != "getup":
            self.phase = np.mod(self.phase + C.GAIT_FREQ * C.CTRL_DT, 1.0)
        self.step_count += 1
        if task == "main":
            self.h_ref, self.alpha = T.posture_step(np, self.h_ref, self.alpha, self.command)

        # MuJoCo kararsizlikta veriyi kendisi sifirlar (zaman geri gider); bu ortamlar dusmus sayilir
        bad = (TIME < self.step_count * C.CTRL_DT - 0.5 * C.SIM_DT) | ~np.isfinite(SD).all(1) \
            | ~np.isfinite(QVEL).all(1) | ~np.isfinite(QACC).all(1) | ~np.isfinite(AF).all(1)
        if bad.any():
            for X in (SD, QPOS, QVEL, QACC, AF, TZ):
                X[bad] = np.nan_to_num(X[bad], nan=0.0, posinf=0.0, neginf=0.0)

        # 7-9.
        s = self._state(a, self.a_prev)
        if task != "getup":
            air_new, first = T.update_air_time(np, self.air_time, s["contact"], C.CTRL_DT)
            s["air_time"], s["first_contact"] = air_new, first
        if task == "walk":
            terms = T.reward_terms(np, s)
            term = T.terminated(np, s) if self.terminate else np.zeros(n)
        else:
            terms = T.reward_terms(np, s, task)
            term = T.terminated(np, s, task) if self.terminate else np.zeros(n)
        TM = np.stack([np.broadcast_to(terms[k], (n,)) for k in self.rkeys], axis=1)
        if bad.any():
            TM[bad] = 0.0
            terms = {k: TM[:, i] for i, k in enumerate(self.rkeys)}
            term = np.where(bad, 1.0, term)
        timeout = self.step_count >= self.max_steps
        rew = T.total_reward(np, terms, term, self.weights)
        self.ep_sum += TM * self.w_dt

        # 10.
        if task != "getup":
            self.air_time = T.reset_air_time(np, air_new, s["contact"])
        if task == "main":
            self.ev, inc = T.main_events(np, s, self.ev, term)
        elif task == "getup":
            self.mem = {k: np.array(v, dtype=np.float64) for k, v in T.getup_memory(np, s).items()}
            self.ev, inc = T.getup_events(np, s, self.ev)
            inc = {k: v * np.where(bad, 0.0, 1.0) for k, v in inc.items()}
        if task != "walk":
            self.ev = {k: np.array(v, dtype=np.float64) for k, v in self.ev.items()}
            self.ep_met += np.stack([np.broadcast_to(inc[k], (n,)) for k in self.mkeys], axis=1)
        self.a_prev = a

        # 11. komut: walk 6 s'de bir yeniden ornekleme; main segment zamanlayicisi (kilitli komut degismez)
        if task == "walk":
            if self.fixed_cmd is None:
                for j in np.flatnonzero(self.step_count % CMD_STEPS == 0):
                    self.command[j] = self._sample_command(self.rngs[j])
        elif task == "main":
            self.seg_timer -= C.CTRL_DT
            if self.fixed_cmd is None:
                ex = np.flatnonzero(self.seg_timer <= 0.0)
                if len(ex):
                    uu = np.stack([self.rngs[j].random(T.NUM_SEGMENT_U) for j in ex])
                    kd, cmd, dur = T.sample_segment(np, uu, self.command[ex])
                    self.seg[ex], self.command[ex], self.seg_timer[ex] = kd, cmd, dur

        # 12.
        fell = term > 0.5
        done = fell | timeout
        io["rew"][:] = rew
        io["done"][:] = done
        io["time_outs"][:] = timeout & ~fell
        nr, nm = len(self.rkeys), len(self.mkeys)
        for j in np.flatnonzero(done):
            io["ep_stats"][j, 0] = self.step_count[j]
            if task == "walk":
                io["ep_stats"][j, 1:] = self.ep_sum[j]
            else:
                io["ep_stats"][j, 1:1 + nr] = self.ep_sum[j]
                io["ep_stats"][j, 1 + nr:1 + nr + nm] = self.ep_met[j]
                io["ep_stats"][j, -2:] = (self.reset_kind[j], self.bank_label[j])
            self._reset_env(j)

        # 13.
        self._write_obs(U[:, self.o_noise:self.o_noise + self.num_obs])


# --- isci sureci -----------------------------------------------------------------
def _worker(conn, raw, num_envs, lo, hi, seed, flags, scene_path, banks=None):
    signal.signal(signal.SIGINT, signal.SIG_IGN)      # Ctrl-C ebeveynde ele alinir
    ppid = os.getppid()
    try:
        core = _EnvSlice(num_envs, lo, hi, seed, flags, _views(raw, num_envs, flags.get("task", "walk")),
                         scene_path, banks=banks)
    except BaseException:
        conn.send_bytes(b"e" + traceback.format_exc().encode())
        return
    conn.send_bytes(b"k")
    while True:
        try:
            if not conn.poll(2.0):
                if os.getppid() != ppid:                # ebeveyn oldu
                    break
                continue
            cmd = conn.recv_bytes()
        except (EOFError, OSError):
            break
        if cmd == b"q":
            break
        try:
            if cmd == b"s":
                core.step()
            elif cmd == b"r":
                core.reset_all()
            elif cmd == b"c":
                core.set_commands(core.io["cmd_in"])
            elif cmd == b"n":
                core.set_commands(None)
            elif cmd == b"o":
                core.refresh_obs()
            elif cmd == b"p":
                core.set_progress(core.io["ctl"][0])
            else:
                raise ValueError("unknown command %r" % cmd)
            reply = b"k"
        except Exception:
            reply = b"e" + traceback.format_exc().encode()
        try:
            conn.send_bytes(reply)
        except (EOFError, OSError):
            break
    conn.close()


def _shutdown(conns, procs):
    for c in conns:
        try:
            c.send_bytes(b"q")
        except Exception:
            pass
    deadline = time.monotonic() + 3.0
    for p in procs:
        p.join(max(0.0, deadline - time.monotonic()))
    for p in procs:
        if p.is_alive():
            p.terminate()
            p.join(1.0)
        if p.is_alive():
            p.kill()
            p.join(1.0)
    for c in conns:
        try:
            c.close()
        except Exception:
            pass


def _default_start_method():
    # fork, cok is parcacikli bir ebeveynde (numpy/OpenBLAS, torch, jax) kilitlenebilir -> forkserver.
    # forkserver/spawn ile calistirilan betigin `if __name__ == "__main__":` korumasi olmali.
    m = os.environ.get("OPENDUCK_MP_START")
    if m:
        return m
    return "forkserver" if "forkserver" in mp.get_all_start_methods() else "spawn"


class _main_hidden(object):
    """spawn/forkserver iscileri __main__'i (or. torch yukleyen egitim betigi) yeniden import etmesin:
    isciler yalnizca bu modulu kullanir. Process.start() suresince __main__'in spec/file bilgisi gizlenir."""

    def __init__(self, active):
        self.main = sys.modules.get("__main__") if active else None
        self.saved = {}

    def __enter__(self):
        if self.main is not None:
            d = self.main.__dict__
            self.saved = {k: d[k] for k in ("__spec__", "__file__") if k in d}
            d["__spec__"] = None
            d.pop("__file__", None)

    def __exit__(self, *exc):
        if self.main is not None:
            d = self.main.__dict__
            d.pop("__spec__", None)
            d.update(self.saved)


# --- genel API --------------------------------------------------------------------
class VecEnv(object):
    """Bkz. rl/TASK.md "CPU backend API". Gorev parametreleri (walk varsayilanlari bugunku davranis):

    task               "walk" | "main" | "getup"
    terminate          False: dusme sonlandirmasi yok (yalniz bozuk fizik ve zaman asimi; oynatma/getup demosu)
    reset_probs        sifirlama karisimi degisikligi, or. {"home": 1.0} (main), {"fallen": 1.0} (getup)
    max_episode_steps  None: gorevin bolum uzunlugu
    progress           getup mufredat ilerlemesi p (gec terim agirliklari; degerlendirmede 1.0); set_progress(p)
    episode_details    True: info["episodes"] = {"env": ids, <episode_columns(task)>: (k,) diziler}
    bank_n             poz bankasi boyutu (None: config; int ya da {tur: n}, onek)
    model_variant      fizik modeli degisikligi (oynatma: main politikasi getup modelinde, devir icin ayni geomlar)
    head_force_peak    True: fizik alt adim alt adim (dinamik ayni); task_state()["head_force_peak"] (N,) = son kontrol
                       adimindaki alt adimlarin kafa-zemin normal kuvveti en buyugu (N; degerlendirme A4)
    robot              None (config.ROBOT_PROFILE) | "xl330_27" | "tr35_b2" (aktuator + sensor modeli; modul notu)
    backlash           tr35: DS eklemlerinde seri bosluk eklemi (None/False: yok)
    """

    num_obs = C.NUM_OBS                 # sinif degerleri walk icin; ornek degerleri gorevden
    num_critic_obs = C.NUM_CRITIC_OBS
    num_actions = C.NUM_ACTIONS
    reward_keys = REW_KEYS

    def __init__(self, num_envs, num_workers=None, seed=0, randomize=True, noise=True, head_random=True,
                 push=True, fixed_command=None, start_method=None, scene_path=None, textures=None,
                 task="walk", terminate=True, reset_probs=None, max_episode_steps=None, progress=0.0,
                 episode_details=False, bank_n=None, model_variant=None, head_force_peak=False, robot=None,
                 backlash=None):
        self.spec = spec = TK.get_task(task)
        self.robot = ACT.profile(robot)
        self.backlash = bool(backlash)
        self.task = spec.name
        self.num_obs, self.num_critic_obs, self.num_actions = spec.num_obs, spec.num_critic_obs, spec.num_actions
        self.reward_keys = list(spec.reward_keys)
        self.episode_metrics = list(spec.episode_metrics)
        self.episode_columns = episode_columns(spec)
        self.episode_details = bool(episode_details)
        self.model_variant = model_variant or spec.model_variant
        self.num_envs = N = int(num_envs)
        if num_workers is None:
            num_workers = os.cpu_count() or 1
        self.num_workers = W = max(0, min(int(num_workers), N))
        if seed is None:
            seed = int(np.random.SeedSequence().entropy % (2 ** 63))
        self.seed = seed
        self._scene_path = scene_path
        if fixed_command is not None and spec.num_commands == 0:
            raise ValueError("%s gorevinin komutu yok (fixed_command verilemez)" % self.task)
        reset_spec(spec, reset_probs)                   # erken dogrulama (iscilerden once)
        flags = dict(randomize=randomize, noise=noise, head_random=head_random, push=push, task=self.task,
                     terminate=terminate, reset_probs=None if reset_probs is None else dict(reset_probs),
                     max_episode_steps=max_episode_steps, progress=float(progress), model_variant=model_variant,
                     head_force_peak=bool(head_force_peak), robot=self.robot, backlash=self.backlash)
        banks = load_banks(spec, reset_probs, bank_n, self.model_variant) if self.task != "walk" else {}
        self._progress = float(progress)
        self._core, self._conns, self._procs, self._broken, self._idx = None, [], [], False, None
        self._finalizer = None
        self.start_method = None
        if W == 0:
            _, size = _layout(N, spec)
            self._raw = np.zeros(size, dtype=np.uint8)
            self._io = _views(self._raw, N, spec)
            if textures is None:
                textures = N <= 16
            self._core = _EnvSlice(N, 0, N, seed, flags, self._io, scene_path, textures, banks=banks)
        else:
            self.start_method = start_method or _default_start_method()
            ctx = mp.get_context(self.start_method)
            _, size = _layout(N, spec)
            self._raw = ctx.RawArray("B", size)
            self._io = _views(self._raw, N, spec)
            bounds = [int(b) for b in np.linspace(0, N, W + 1).round()]
            self._slices = list(zip(bounds[:-1], bounds[1:]))
            self._finalizer = weakref.finalize(self, _shutdown, self._conns, self._procs)
            if self.start_method == "forkserver":
                ctx.set_forkserver_preload([__name__])      # sunucu mujoco/numpy'yi bir kez yukler
            try:
                with _main_hidden(self.start_method != "fork"):
                    for lo, hi in self._slices:
                        parent, child = ctx.Pipe()
                        p = ctx.Process(target=_worker, args=(child, self._raw, N, lo, hi, seed, flags, scene_path,
                                                              banks),
                                        daemon=True, name="openduck-env-%d-%d" % (lo, hi))
                        p.start()
                        child.close()
                        self._conns.append(parent)
                        self._procs.append(p)
                self._wait_all()
            except BaseException:
                self._broken = True
                self._finalizer()       # baslamis iscileri hemen kapat (GC'yi bekleme)
                raise
        if fixed_command is not None:
            if self.task == "walk":
                self.set_commands(np.broadcast_to(np.asarray(fixed_command, dtype=np.float64), (N, 3)))
            else:
                c = TK.pad_command(self.task, np.asarray(fixed_command, dtype=np.float64))
                self.set_commands(np.broadcast_to(c, (N, c.shape[-1])))

    # -- isci iletisimi
    def _died(self, w):
        proc = self._procs[w]
        proc.join(1.0)
        return RuntimeError("env worker %d died (exitcode %s)" % (w, proc.exitcode))

    def _wait(self, w):
        conn, proc = self._conns[w], self._procs[w]
        try:
            while not conn.poll(1.0):
                if not proc.is_alive() and not conn.poll(0):
                    raise EOFError
            msg = conn.recv_bytes()
        except (EOFError, OSError):
            raise self._died(w) from None
        if msg[:1] == b"e":
            return "env worker %d failed:\n%s" % (w, msg[1:].decode(errors="replace"))
        return None

    def _fail(self):
        # bir isci coktuyse ortam kullanilamaz: kalan iscileri de kapat
        self._broken = True
        if self._finalizer is not None:
            self._finalizer()

    def _wait_all(self):
        try:
            errs = [e for e in (self._wait(w) for w in range(len(self._conns))) if e]
        except RuntimeError:
            self._fail()
            raise
        except BaseException:
            self._broken = True
            raise
        if errs:
            self._fail()
            raise RuntimeError("\n".join(errs))

    def _run(self, cmd):
        if self._broken:
            raise RuntimeError("VecEnv is closed or a worker failed")
        if self._core is not None:
            core = self._core
            {b"s": core.step, b"r": core.reset_all, b"o": core.refresh_obs,
             b"p": lambda: core.set_progress(self._io["ctl"][0])}[cmd]()
            return
        for w, c in enumerate(self._conns):
            try:
                c.send_bytes(cmd)
            except OSError:
                err = self._died(w)
                self._fail()
                raise err from None
            except BaseException:
                self._broken = True
                raise
        self._wait_all()

    def _in_process(self, what):
        if self._broken:
            raise RuntimeError("VecEnv is closed or a worker failed")
        if self._core is None:
            raise RuntimeError("%s needs num_workers=0" % what)
        return self._core

    # -- API
    def reset(self):
        self._run(b"r")
        return self._io["obs"].copy(), self._io["critic"].copy()

    def step(self, actions):
        a = np.asarray(actions, dtype=np.float32)
        if a.shape != (self.num_envs, self.num_actions):
            raise ValueError("actions must have shape (%d, %d), got %s" % (self.num_envs, self.num_actions, a.shape))
        self._io["actions"][:] = a
        self._run(b"s")
        io = self._io
        done = io["done"].copy()
        info = {"time_outs": io["time_outs"].copy()}
        if done.any():
            st = io["ep_stats"][done]
            ep = {"ep_len": float(st[:, 0].mean())}
            for k, key in enumerate(self.reward_keys):
                ep["rew_" + key] = float(st[:, 1 + k].mean())
            for k, key in enumerate(self.episode_metrics):
                ep[key] = float(st[:, 1 + len(self.reward_keys) + k].mean())
            info["episode"] = ep
            if self.episode_details:
                info["episodes"] = dict({c: st[:, i].copy() for i, c in enumerate(self.episode_columns)},
                                        env=np.flatnonzero(done))
        return io["obs"].copy(), io["critic"].copy(), io["rew"].copy(), done, info

    def set_commands(self, cmd, refresh_obs=False):
        """Sabit komutlar (yeniden ornekleme / segmentler kapanir); None rastgele komutlara doner.
        walk: (N,3). main: (N,3) ya da (N,5) (ya da tek komut (3,)/(5,)); 3 boyut [vx,vy,wz,H_HOME,0] ile
        doldurulur. getup: komut yok (ValueError). refresh_obs=True: gozlem yeni komutla yeniden yazilir ve
        (obs, critic) dondurulur."""
        if self._broken:
            raise RuntimeError("VecEnv is closed or a worker failed")
        if cmd is not None:
            if self.task == "walk":
                cmd = np.asarray(cmd, dtype=np.float64).reshape(self.num_envs, 3)
            elif self.spec.num_commands == 0:
                raise ValueError("%s gorevinin komutu yok" % self.task)
            else:
                c = np.asarray(cmd, dtype=np.float64)
                if c.ndim == 1:
                    c = np.broadcast_to(c, (self.num_envs, c.shape[0]))
                cmd = TK.pad_command(self.task, c.reshape(self.num_envs, -1))
        if self._core is not None:
            self._core.set_commands(cmd)
        else:
            if cmd is not None:
                self._io["cmd_in"][:] = cmd
            self._run(b"n" if cmd is None else b"c")
        if refresh_obs:
            return self.observe()
        return None

    def observe(self):
        """Gozlemi mevcut durumdan yeniden hesaplar (adim atmadan) ve (obs, critic) dondurur."""
        self._run(b"o")
        return self._io["obs"].copy(), self._io["critic"].copy()

    def set_progress(self, p):
        """getup mufredat ilerlemesi p in [0,1] (egitici her iterasyonda ayarlar; walk/main'de etkisiz)."""
        self._progress = float(min(max(p, 0.0), 1.0))
        if self.task != "getup":
            return
        self._io["ctl"][0] = self._progress
        self._run(b"p")

    @property
    def progress(self):
        return self._progress

    def task_state(self):
        """(num_workers=0) gozlemin gordugu durum sozlugunun kopyasi (rl/task_math.py "s" alanlari, toplu) +
        gorev durumu (step_count, reset_kind, bank_label, main: seg/seg_timer/locked/ev_*, getup: ev_*;
        head_force_peak=True ise head_force_peak)."""
        return self._in_process("task_state()").task_state()

    def set_state(self, i, qpos, qvel, ctrl=None, warmstart=None):
        """(num_workers=0) ortam i'yi verilen fiziksel duruma koyar (politika devri; last_action = 0,
        alpha = 0, h_ref = H_HOME, bolum sayaclari sifir). Yeni (obs, critic) dondurur."""
        core = self._in_process("set_state()")
        core.set_state(i, qpos, qvel, ctrl, warmstart)
        core.refresh_obs()
        return self._io["obs"].copy(), self._io["critic"].copy()

    def mj_model(self, i):
        if self._core is None:
            raise RuntimeError("mj_model() needs num_workers=0")
        return self._core.models[i]

    def mj_data(self, i):
        if self._core is None:
            raise RuntimeError("mj_data() needs num_workers=0")
        return self._core.datas[i]

    @property
    def robot_index(self):
        if self._core is not None:
            return self._core.idx
        if self._idx is None:
            self._idx = RobotIndex(physics_model(self._scene_path, variant=self.model_variant, robot=self.robot,
                                                 backlash=self.backlash), list(self.spec.action_joints))
        return self._idx

    def close(self):
        if self._finalizer is not None:
            self._finalizer()
        self._broken = True

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
