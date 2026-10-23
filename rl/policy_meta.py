# -*- coding: utf-8 -*-
"""
Politika meta verisi (<run>/policy_meta.json) - torch'suz; iki egitici de (train.py, train_mjx.py) bunu kullanir.

  meta = build_meta(steps, backend, idx=None, task="walk", network=None, scene_path=None, robot=None,
                    getup_head=None)

walk: bugunku alanlar, ayni sira ve degerlerle (obs_dim, action_dim, ctrl_dt, action_scale, action_clip,
default_leg_q, leg_joints, obs_joints, default_obs_q, gait_freq, obs_scales, backend, steps).
main/getup: ayni alanlar (action_scale vektor) + gorev alanlari ("task", "version", "obs_layout", komut,
aksiyon esleme, durus filtresi ve referans durus sabitleri, devir esikleri). Robot calisma zamani gozlemi ve
aksiyon eslemesini yalniz bu meta ile yeniden uretebilmelidir (rl/TASK.md "Policy export format").

idx verilmezse RobotIndex sahneden kurulur: build_model(scene_path, variant=gorev varyanti, robot=robot) (None:
surec varsayilanlari; xl330 ciktisi bugunkuyle ayni). tr35_b2 modeli (idx.robot == "tr35_b2") icin ek ust duzey
alanlar (firmware'in tek dogruluk kaynagi, OpenDuck-TR spec 3.2.3-3.2.4, rl-map D3):
  robot, actuators (actuators.actuator_table sutunlari aktuator sirasiyla + classes, rails, i_cc, boards),
  sensors (sensors.POT, sensors.IMU, gozlem eklemlerinin pot tablosu), delays (alt adim gecikme araliklari),
  derived_source (config.DERIVED_SOURCE), kappa (config.KAPPA), network (verilirse: actor_hidden, critic_hidden,
  activation, obs_normalization), getup: getup_head ("free" | "fixed"; fixed'te head_tuck_deg), main:
  posture_ref.seat_pitch_deg. xl330'da network/getup_head yok sayilir. Cekirdek adlari getattr ile okunur.
"""
from . import config as C
from . import tasks as TK

META_VERSION = 2
TR35 = "tr35_b2"
ACT_COLUMNS = ("names", "cls", "tau_s", "omega0", "pband", "kp", "damping", "armature", "frictionloss", "deadband",
               "pwm_res", "i_stall", "i_idle", "board", "on_leg", "ctrl_lo", "ctrl_hi")
# PCA9685 kartlari (rl/actuators.py: kart 0 300 Hz DS, kart 1 50 Hz ES3054/MG90S/SG90); JSON anahtari kart no (str)
BOARDS = {"0": {"pwm_hz": 300, "pca": "0x40"}, "1": {"pwm_hz": 50, "pca": "0x41"}}
NETWORK_KEYS = ("actor_hidden", "critic_hidden", "activation", "obs_normalization")


def _f(x):
    return [float(v) for v in x]


def _j(x):
    """JSON'a uygun kopya: numpy dizileri/sayilari listeye/Python sayisina, tuple listeye, anahtarlar str."""
    import numpy as np
    if isinstance(x, dict):
        return {str(k): _j(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_j(v) for v in x]
    if isinstance(x, np.ndarray):
        return _j(x.tolist())
    if isinstance(x, np.generic):
        return x.item()
    return x


def _tr35_fields(meta, idx, t, network, getup_head):
    from . import actuators as ACT
    from . import sensors as SENS
    tab = idx.act
    act = {k: _j(tab[k]) for k in ACT_COLUMNS}
    act.update(classes=_j(ACT.CLASSES), rails=_j(ACT.RAILS), i_cc=float(ACT.I_CC),
               boards=_j(getattr(ACT, "BOARDS", BOARDS)),
               units="tau_s N.m; omega0 rad/s; pband, deadband, pwm_res, ctrl_lo/hi rad; kp N.m/rad; damping N.m.s/rad; "
                     "armature kg.m2; frictionloss N.m; i_stall, i_idle A; board PCA9685 karti")
    pot = dict(_j(idx.pot), joints=list(C.OBS_JOINTS))
    meta["robot"] = idx.robot
    meta["actuators"] = act
    meta["sensors"] = {"pot": _j(SENS.POT), "imu": _j(SENS.IMU), "pot_table": pot}
    meta["delays"] = {"delay_substeps": _j(ACT.DR["delay_substeps"]), "delay_boards": [0, 1],
                      "substep_dt": C.SIM_DT, "n_substeps": C.N_SUBSTEPS}
    meta["derived_source"] = _j(getattr(C, "DERIVED_SOURCE", None))
    meta["kappa"] = _j(getattr(C, "KAPPA", None))
    if network is not None:
        meta["network"] = {k: _j(network.get(k)) for k in NETWORK_KEYS}
    if t.name == "getup":
        mode = getup_head or getattr(C, "GETUP_HEAD_DEFAULT", "free")
        meta["getup_head"] = mode
        if mode == "fixed":
            meta["head_tuck_deg"] = _j(getattr(C, "HEAD_TUCK_DEG", None))
    return meta


def build_meta(steps, backend, idx=None, task="walk", network=None, scene_path=None, robot=None, getup_head=None):
    t = TK.get_task(task)
    if idx is None:
        from .robot_model import RobotIndex, build_model
        idx = RobotIndex(build_model(scene_path, variant=t.model_variant, robot=robot), t.action_joints)
    tr35 = getattr(idx, "robot", None) == TR35
    if t.name != "walk" and (idx.variant != t.model_variant or list(idx.action_joints) != list(t.action_joints)):
        raise ValueError("RobotIndex gorevle uyusmuyor: %s/%s (gorev %s: %s)" % (
            idx.variant, len(idx.action_joints), t.name, t.model_variant))
    scale = TK.action_scale_vector(t, idx.default_action_q, idx.action_lo, idx.action_hi)
    meta = {
        "obs_dim": t.num_obs,
        "action_dim": t.num_actions,
        "ctrl_dt": C.CTRL_DT,
        "action_scale": C.ACTION_SCALE if t.name == "walk" else _f(scale),
        "action_clip": t.action_clip,
        "default_leg_q": _f(idx.default_obs_q[:len(C.LEG_JOINTS)]),
        "leg_joints": list(C.LEG_JOINTS),
        "obs_joints": list(C.OBS_JOINTS),
        "default_obs_q": _f(idx.default_obs_q),
        "gait_freq": C.GAIT_FREQ,
        "obs_scales": dict(C.OBS_SCALES),
        "backend": backend,
        "steps": int(steps),
    }
    if t.name == "walk":
        return _tr35_fields(meta, idx, t, network, getup_head) if tr35 else meta
    meta.update({
        "task": t.name,
        "version": META_VERSION,
        "obs_layout": [[n, k] for n, k in t.obs_layout],
        "obs_clip": C.OBS_CLIP,
        "action_joints": list(t.action_joints),
        "default_action_q": _f(idx.default_action_q),
        "action_mode": t.action_mode,
        "action_lo": _f(idx.action_lo),
        "action_hi": _f(idx.action_hi),
        "episode_length_s": t.episode_length_s,
        "supervisor": dict(C.SUPERVISOR),
        "getup_success": dict(C.GETUP["success"]),
    })
    if t.name == "main":
        meta.update({
            "head_control": "script",
            "command_names": list(t.command_names),
            "command_ranges": {"vx": _f(C.COMMAND_RANGES["vx"]), "vy": _f(C.COMMAND_RANGES["vy"]),
                               "wz": _f(C.COMMAND_RANGES["wz"]), "h": [C.H_MIN_STAND, C.H_HOME], "sit": [0.0, 1.0]},
            # gozlemdeki komut: [vx, vy, wz, (h - h_home)/h_norm, sit]; sit=1 ya da h < h_min_walk iken hizlar 0
            "command_obs": {"h_home": C.H_HOME, "h_norm": C.H_NORM, "h_min_walk": C.H_MIN_WALK,
                            "h_min_stand": C.H_MIN_STAND},
            # gozlemdeki durus: [(h_ref - h_home)/h_norm, alpha]; her kontrol adiminda aksiyon uygulandiktan sonra
            # h_ref += clip(h - h_ref, +-h_rate*dt); alpha += clip(sit - alpha, -dt/stand_ramp_s, +dt/sit_ramp_s)
            "posture_filter": {"h_rate": C.H_RATE, "sit_ramp_s": C.SIT_RAMP_S, "stand_ramp_s": C.STAND_RAMP_S,
                               "h_init": C.H_HOME, "alpha_init": 0.0, "alpha_stand_max": C.ALPHA_STAND_MAX},
            # target = q_ref(h_ref, alpha) + action_scale * clip(a, +-action_clip);
            # s1 = clip(alpha/sit_split, 0, 1), s2 = clip((alpha - sit_split)/(1 - sit_split), 0, 1),
            # hd = lerp(h_ref, h_deep, s1), F_home(h) = HOME + F(h) - F(h_home) (F: tablo, dogrusal ara deger,
            # uclarda kirpilir), q_ref = lerp(F_home(hd), SEAT, s2) (sag bacak = -sol; yaw/roll HOME),
            # bas ctrl = HOME_head + s1 * (HEAD_SIT - HOME_head)
            "posture_ref": {
                "crouch_joints": list(C.CROUCH_JOINTS),
                "crouch_family_deg": [list(r) for r in C.CROUCH_FAMILY],
                "seat_deg": dict(C.SEAT_DEG), "head_sit_deg": dict(C.HEAD_SIT_DEG),
                "head_home_deg": dict(C.HEAD_HOME_DEG),
                "sit_split": C.SIT_SPLIT, "h_deep": C.H_DEEP, "z_sit": C.Z_SIT, "h_home": C.H_HOME,
                "sign": "left leg absolute; right = -left",
            },
        })
        if tr35:      # oturus govde pitch'i (+ = burun asagi, 0 = duz oturus)
            meta["posture_ref"]["seat_pitch_deg"] = float(getattr(C, "SEAT_PITCH_DEG", 0.0))
    else:
        meta.update({"head_control": "policy", "command_names": []})
    return _tr35_fields(meta, idx, t, network, getup_head) if tr35 else meta
