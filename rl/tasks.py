# -*- coding: utf-8 -*-
"""
Gorev tanimlari (TaskSpec): "walk" (mevcut yurume, degismez), "main" (yurume + govde yuksekligi +
otur/kalk), "getup" (dusunce kalkma). Sayisal sabitler rl/config.py'de; matematik rl/task_math.py'de;
arka uc sozlesmesi rl/TASK.md. Duz modul (tools/runpod/launch.py yalniz rl/<ad>.py paketler).

  spec = get_task("main")
  spec.num_obs, spec.num_critic_obs, spec.num_actions        # 53, 64, 10
  spec.obs_slices["command"]                                  # slice(44, 49)
  spec.reward_weights                                         # erken agirliklar (getup: p <= late_p[0])

Gozlem duzeni: obs_layout = ((ad, boyut), ...) aktor sirasi; critic_layout = aktorden SONRA eklenen
ayricalikli parcalar (kritik = aktor (gurultusuz) + critic_layout). Ad -> anlam (task_math.observation):
  gravity 3 (izdusumlu yercekimi), gyro 3 (x OBS_SCALES.gyro), joint_pos 14 (q - HOME), joint_vel 14
  (x OBS_SCALES.joint_vel), last_action num_actions, command (walk: [vx,vy,wz]; main: [vx,vy,wz,h_n,sit]),
  posture 2 ([h_ref_n, alpha]), phase 2 (cos/sin), linvel 3, trunk_z 1, foot_contact 2, foot_z 2,
  friction 1, trunk_floor 1, head_floor 1.
"""
import numpy as np

from . import config as C

TASK_NAMES = ("walk", "main", "getup")
GETUP_ACTION_JOINTS = tuple(C.LEG_JOINTS) + tuple(C.HEAD_JOINTS)


class TaskSpec(object):
    """Bir gorevin statik tanimi (salt okunur kullanin)."""

    FIELDS = (
        "name",              # "walk" | "main" | "getup"
        "model_variant",     # robot_model.build_model(variant=...)
        "action_joints",     # politikanin surdugu eklemler (aktuator sirasi)
        "action_mode",       # "home": HOME + s*clip(a)  | "ref": q_ref(h_ref, alpha) + s*clip(a)
                             # "home_clip": clip(HOME + S_j*clip(a, +-1), ctrlrange)
        "action_scale",      # float (her eklem ayni) ya da None (getup: S_j modelden, action_scale_vector)
        "action_clip",       # aksiyon kirpma siniri
        "command_names",     # durumdaki komut alanlari (getup: bos)
        "obs_layout",        # aktor gozlemi ((ad, boyut), ...)
        "critic_layout",     # kritik ek parcalari ((ad, boyut), ...)
        "reward_weights",    # {terim: w} (saniye basina; toplam = sum(w*r)*CTRL_DT)
        "reward_late",       # {terim: w_son}: p in late_p arasinda reward_weights'ten w_son'a lineer
        "late_p",            # (p0, p1) ya da None
        "termination_penalty",
        "terminate_on_fall",  # False: yalnizca bozuk fizik ve zaman asimi (getup)
        "episode_length_s",
        "reset_probs",       # ((tur, olasilik), ...) sifirlama karisimi
        "segment_names",     # main: MAIN_SEGMENTS; digerleri ()
        "segment_probs",     # main: MAIN["segment_probs"]
        "head_mode",         # "random" (walk) | "script" (main: oturus scripti + alpha'ya bagli rastgele) | "policy"
        "gamma",             # varsayilan indirim (None: egiticinin mevcut varsayilani)
        "actor_hidden",      # None: egiticinin mevcut varsayilani
        "critic_hidden",
        "init_noise_std",    # PPO baslangic aksiyon gurultusu (None: egiticinin mevcut varsayilani)
        "solver_iterations",  # MJX Newton iterasyonu (None: egiticinin varsayilani 4); oturus/dusmus temaslari
                             # 4'te MuJoCo C'den sapiyor, 8 C ile ayni sonucu veriyor (S4 parity)
        "episode_metrics",   # rew_* disindaki bolum metrikleri (task_math.*_events artislari)
    )

    def __init__(self, **kw):
        for k in self.FIELDS:
            setattr(self, k, kw.pop(k))
        if kw:
            raise TypeError("bilinmeyen alanlar: %s" % sorted(kw))
        self.num_actions = len(self.action_joints)
        self.num_commands = len(self.command_names)
        self.num_obs = sum(n for _, n in self.obs_layout)
        self.num_critic_obs = self.num_obs + sum(n for _, n in self.critic_layout)
        self.max_episode_steps = int(round(self.episode_length_s / C.CTRL_DT))
        self.obs_slices = _slices(self.obs_layout)
        self.critic_slices = _slices(tuple(self.obs_layout) + tuple(self.critic_layout))
        self.reward_keys = tuple(self.reward_weights)

    def __repr__(self):
        return "TaskSpec(%s: obs %d, critic %d, actions %d)" % (self.name, self.num_obs, self.num_critic_obs,
                                                                self.num_actions)


def _slices(layout):
    out, o = {}, 0
    for name, n in layout:
        out[name] = slice(o, o + n)
        o += n
    return out


_BASE_OBS = (("gravity", 3), ("gyro", 3), ("joint_pos", 14), ("joint_vel", 14))

WALK = TaskSpec(
    name="walk", model_variant="walk", action_joints=tuple(C.ACTION_JOINTS), action_mode="home",
    action_scale=C.ACTION_SCALE, action_clip=C.ACTION_CLIP, command_names=("vx", "vy", "wz"),
    obs_layout=_BASE_OBS + (("last_action", C.NUM_ACTIONS), ("command", 3), ("phase", 2)),
    critic_layout=(("linvel", 3), ("trunk_z", 1), ("foot_contact", 2), ("foot_z", 2), ("friction", 1)),
    reward_weights=dict(C.REWARD_WEIGHTS), reward_late={}, late_p=None,
    termination_penalty=C.TERMINATION_PENALTY, terminate_on_fall=True, episode_length_s=C.EPISODE_LENGTH_S,
    reset_probs=(("home", 1.0),), segment_names=(), segment_probs=(), head_mode="random",
    gamma=None, actor_hidden=None, critic_hidden=None, init_noise_std=None, solver_iterations=None,
    episode_metrics=(),
)

MAIN = TaskSpec(
    name="main", model_variant="main", action_joints=tuple(C.LEG_JOINTS), action_mode="ref",
    action_scale=C.ACTION_SCALE, action_clip=C.ACTION_CLIP, command_names=tuple(C.MAIN_COMMAND_NAMES),
    obs_layout=_BASE_OBS + (("last_action", 10), ("command", 5), ("posture", 2), ("phase", 2)),
    critic_layout=(("linvel", 3), ("trunk_z", 1), ("foot_contact", 2), ("foot_z", 2), ("friction", 1),
                   ("trunk_floor", 1), ("head_floor", 1)),
    reward_weights=dict(C.MAIN_REWARD_WEIGHTS), reward_late={}, late_p=None,
    termination_penalty=C.TERMINATION_PENALTY, terminate_on_fall=True, episode_length_s=C.MAIN_EPISODE_S,
    reset_probs=tuple(C.MAIN["reset_probs"].items()), segment_names=tuple(C.MAIN_SEGMENTS),
    segment_probs=tuple(C.MAIN["segment_probs"]), head_mode="script",
    gamma=0.97, actor_hidden=None, critic_hidden=None, init_noise_std=None, solver_iterations=8,
    episode_metrics=("walk_steps", "sit_steps", "seated_ok_steps", "standup_attempts", "standup_success",
                     "sitdown_attempts", "sitdown_success", "falls_walk", "falls_sit", "falls_transition"),
)

GETUP = TaskSpec(
    name="getup", model_variant="getup", action_joints=GETUP_ACTION_JOINTS, action_mode="home_clip",
    action_scale=None, action_clip=1.0, command_names=(),
    obs_layout=_BASE_OBS + (("last_action", 14),),
    critic_layout=(("linvel", 3), ("trunk_z", 1), ("foot_contact", 2), ("trunk_floor", 1), ("head_floor", 1),
                   ("friction", 1)),
    reward_weights=dict(C.GETUP_REWARD_WEIGHTS), reward_late=dict(C.GETUP_REWARD_LATE),
    late_p=tuple(C.GETUP["late_p"]),
    termination_penalty=C.TERMINATION_PENALTY, terminate_on_fall=False, episode_length_s=C.GETUP_EPISODE_S,
    reset_probs=tuple(C.GETUP["reset_probs"].items()), segment_names=(), segment_probs=(), head_mode="policy",
    gamma=0.99, actor_hidden=(512, 256, 128), critic_hidden=(512, 256, 128),
    # tam ctrlrange eslemesi (S_j 25-170 derece): 1.0 gurultu %32 kirpilan bang-bang verir; 0.5 ~ microduck
    init_noise_std=0.5, solver_iterations=8,
    episode_metrics=("getup_success", "head_impacts", "head_impact_v2", "torque_sat_steps"),
)

TASKS = {"walk": WALK, "main": MAIN, "getup": GETUP}


def get_task(task="walk"):
    """Ad ya da TaskSpec -> TaskSpec."""
    if isinstance(task, TaskSpec):
        return task
    try:
        return TASKS[task]
    except KeyError:
        raise ValueError("bilinmeyen gorev %r (%s)" % (task, ", ".join(TASK_NAMES)))


def action_scale_vector(task, default_action_q, action_lo, action_hi):
    """(num_actions,) eklem basina aksiyon olcegi. walk/main: ACTION_SCALE; getup:
    S_j = max(|hi_j - HOME_j|, |lo_j - HOME_j|) (RobotIndex.action_lo/hi, default_action_q)."""
    t = get_task(task)
    if t.action_mode == "home_clip":
        q0 = np.asarray(default_action_q, dtype=np.float64)
        return np.maximum(np.abs(np.asarray(action_hi) - q0), np.abs(np.asarray(action_lo) - q0))
    return np.full(t.num_actions, float(t.action_scale))


def pad_command(task, command):
    """set_commands / fixed_command dolgusu (numpy). walk: (...,3). main: (...,3) -> [vx,vy,wz,H_HOME,0],
    (...,5) aynen. getup: komut yok (ValueError)."""
    t = get_task(task)
    c = np.asarray(command, dtype=np.float64)
    if t.num_commands == 0:
        raise ValueError("%s gorevinin komutu yok" % t.name)
    if c.shape[-1] == t.num_commands:
        return c.copy()
    if t.name == "main" and c.shape[-1] == 3:
        pad = np.broadcast_to(np.array([C.H_HOME, 0.0]), c.shape[:-1] + (2,))
        return np.concatenate([c, pad], axis=-1)
    raise ValueError("%s komutu %s boyutlu olmali (verilen %d)" % (
        t.name, "3 ya da 5" if t.name == "main" else str(t.num_commands), c.shape[-1]))
