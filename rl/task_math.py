# -*- coding: utf-8 -*-
"""
Gozlem, odul ve sonlandirma matematigi - arka uctan bagimsiz.

Her fonksiyon `xp` (numpy ya da jax.numpy) ve toplu (batched, ilk eksen N) dizilerden
olusan bir durum sozlugu `s` alir. Yerinde (in-place) degisiklik yapilmaz, boylece ayni kod
CPU'da numpy ile, GPU'da jax ile calisir (vmap'li tek ortamda ilk eksen yoktur: tum indeksleme
`[..., i]`, birlestirme `axis=-1`). Dizi degerlerine gore python dallanmasi yoktur; gorev adina
(statik) gore dallanma vardir. Gorev parametresi `task`: ad ("walk" | "main" | "getup") ya da
tasks.TaskSpec; varsayilan "walk" bugunku davranisin BIT-DUZEYINDE aynisidir.

Durum sozlugu alanlari (bkz. rl/TASK.md):

  -- tum gorevler (walk'ta bugunku alanlar)
  quat        (N,4)   IMU yonelimi [w,x,y,z] (dunya <- govde)
  gyro        (N,3)   govde acisal hizi (rad/s, govde cercevesi)
  linvel      (N,3)   IMU yerel dogrusal hizi (m/s, govde cercevesi)
  trunk_z     (N,)    govde orijini yuksekligi (m)
  q           (N,14)  OBS_JOINTS eklem acilari (rad)
  qd          (N,14)  OBS_JOINTS eklem hizlari (rad/s)
  qdd_legs    (N,10)  LEG_JOINTS eklem ivmeleri (rad/s^2)
  torque      (N,14)  OBS_JOINTS aktuator kuvvetleri (N.m)
  foot_pos    (N,2,3) ayak tabani konumu (dunya, m)   [sol, sag]
  foot_vel    (N,2,3) ayak tabani hizi (dunya, m/s)
  contact     (N,2)   ayak-zemin temasi (0/1)
  action      (N,A)   bu adimin kirpilmis aksiyonu (A = gorevin num_actions'i: 10 ya da 14)
  last_action (N,A)   bir onceki adimin kirpilmis aksiyonu
  command     (N,3)   walk: [vx, vy, wz]
              (N,5)   main: [vx, vy, wz, h_cmd (m), sit (0/1)] HAM komut (gozlem/odul main_command ile
                      etkin komuta cevirir: sit=1 ya da h_cmd < H_MIN_WALK iken hizlar 0, h kirpilir)
  phase       (N,)    yuruyus fazi [0,1) (walk, main)
  air_time    (N,2)   ayaklarin havada gecirdigi sure (bu adim dahil) (walk, main)
  first_contact (N,2) bu adimda yere ilk degen ayak (0/1) (walk, main)
  friction    (N,)    ayak surtunmesi (yalnizca kritik)
  default_q   (14,)   varsayilan (ayakta, HOME) eklem acilari

  -- main
  h_ref       (N,)    durus filtresi: referans yukseklik (m) (posture_step sonrasi degeri, TASK.md)
  alpha       (N,)    durus filtresi: oturma ilerlemesi [0,1] (0 ayakta, 1 oturmus)
  trunk_floor (N,)    govde kutusu-zemin temasi (0/1, dist < CONTACT_EPS)
  head_floor  (N,)    kafa geomlari (kafa kutusu; getup'ta + gaga) - zemin temasi (0/1)

  -- getup
  trunk_floor, head_floor (main ile ayni)
  head_vel    (N,3)   kafa kutusu merkezinin dunya dogrusal hizi (m/s, sensor "head_vel")
  head_floor_prev (N,) onceki adimin head_floor'u (temas baslangici = head_floor * (1 - prev))
  phi_up_prev (N,)    onceki adimin potansiyeli -gz          (getup_memory)
  phi_h_prev  (N,)    onceki adimin potansiyeli min(z, H_HOME) (getup_memory)
  progress    ()/(N,) mufredat ilerlemesi p in [0,1] (reward_weights'e verilir; s'de zorunlu degil)

  -- tr35_b2 (aktuator modeli; tum gorevler, OBS_JOINTS sirasi; rl/TASK.md "Robot profiles")
  tau_net, tau_s, tau_ratio, limit_torque (N,14): motor torku, bolum tau_s'i, tau_net/tau_s, |limit kisit torku|
  rail_demand (N,)    LEG ray talebi (A; olcekleme oncesi, alt adim ortalamasi)
  tr35 SURECINDE (config.ROBOT_PROFILE, statik dal) tork cezalari bunlarla: "torques" yerine "torque_ratio",
  sona "limit_torque", "rail_demand"; getup torque_sat |tau_ratio| ile. xl330 ciktilari bit-duzeyinde ayni.
"""
import numpy as np

from . import config as C
from . import tasks as TK

_TR35 = C.ROBOT_PROFILE == "tr35_b2"     # statik: tork/ray cezalari tau_ratio ile (dosya notu)


def _name(task):
    return task if isinstance(task, str) else task.name


def _ratio_term(xp, s):
    return xp.sum(xp.square(s["tau_ratio"]), axis=-1)


def _limit_rail(xp, s):
    return {"limit_torque": xp.sum(xp.square(s["limit_torque"] / s["tau_s"]), axis=-1),
            "rail_demand": xp.maximum(s["rail_demand"] - C.RAIL_SOFT_A, 0.0)}


def _effort_terms(xp, s):
    """Tork cezasi terimi: xl330 {"torques": sum tau^2}; tr35 {"torque_ratio": sum (tau_net/tau_s)^2}."""
    if _TR35:
        return {"torque_ratio": _ratio_term(xp, s)}
    return {"torques": xp.sum(xp.square(s["torque"]), axis=-1)}


def _limit_rail_terms(xp, s):
    """tr35 ek cezalari (odul sozlugunun sonunda): limit_torque = sum (limit/tau_s)^2, rail_demand =
    max(talep - RAIL_SOFT_A, 0) (A). xl330: bos."""
    if not _TR35:
        return {}
    return _limit_rail(xp, s)


def tr35_penalties(xp, s):
    """tr35 tork cezalari, agirliksiz ve odul sozlugundeki formullerle (profilden bagimsiz; tr35 aktuator durumu
    ister): {"torque_ratio", "limit_torque", "rail_demand"} (N,). rl/eval_tasks.py TorqueAcc raporu."""
    out = {"torque_ratio": _ratio_term(xp, s)}
    out.update(_limit_rail(xp, s))
    return out


# --- yardimcilar -----------------------------------------------------------------
def projected_gravity(xp, quat):
    """Govde cercevesinde birim yercekimi vektoru: R^T [0,0,-1]."""
    w, x, y, z = quat[..., 0], quat[..., 1], quat[..., 2], quat[..., 3]
    gx = -2.0 * (x * z - w * y)
    gy = -2.0 * (y * z + w * x)
    gz = -(1.0 - 2.0 * (x * x + y * y))
    return xp.stack([gx, gy, gz], axis=-1)


def yaw_angle(xp, quat):
    w, x, y, z = quat[..., 0], quat[..., 1], quat[..., 2], quat[..., 3]
    return xp.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_mul(xp, a, b):
    """Hamilton carpimi a*b, [w,x,y,z] (mju_mulQuat ile ayni)."""
    w1, x1, y1, z1 = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    w2, x2, y2, z2 = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return xp.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], axis=-1)


def yaw_quat(xp, yaw):
    """Dunya z ekseni etrafinda yaw donusu [w,0,0,z]."""
    z = xp.zeros_like(yaw)
    return xp.stack([xp.cos(0.5 * yaw), z, z, xp.sin(0.5 * yaw)], axis=-1)


def world_vz(xp, quat, linvel):
    """Govde cercevesindeki hizin dunya z bileseni: (R v)_z = -g . v."""
    return -xp.sum(projected_gravity(xp, quat) * linvel, axis=-1)


def smoothstep(xp, x):
    x = xp.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def phase_features(xp, phase):
    a = 2.0 * xp.pi * phase
    return xp.stack([xp.cos(a), xp.sin(a)], axis=-1)


def foot_phases(xp, phase):
    """(N,2): sol ayak phi, sag ayak phi+0.5 (mod 1)."""
    return xp.stack([phase, xp.mod(phase + 0.5, 1.0)], axis=-1)


def desired_foot_height(xp, phase):
    """Sallanma fazinda (sin > 0) SWING_HEIGHT*sin, durus fazinda 0."""
    return C.SWING_HEIGHT * xp.maximum(xp.sin(2.0 * xp.pi * foot_phases(xp, phase)), 0.0)


def update_air_time(xp, air_time, contact, dt):
    """Ayak havadaysa sureyi artir. first_contact: bu adim yere ilk degen ayaklar.
    Dondurulen air_time, odul hesabi icin temas anindaki sureyi de icerir; ortam,
    temas olan ayaklarin suresini odulden SONRA sifirlamalidir (reset_air_time)."""
    new_air = air_time + dt
    first_contact = xp.where((contact > 0.5) & (air_time > 0.0), 1.0, 0.0)
    return new_air, first_contact


def reset_air_time(xp, air_time, contact):
    return xp.where(contact > 0.5, 0.0, air_time)


# --- main: komut, durus filtresi ve referans durus -----------------------------------
# Comelme ailesi (h artan sirada) ve oturus pozu; acilar radyan, SOL bacak, mutlak (config isaret kurali).
_FAM = np.array(C.CROUCH_FAMILY, dtype=np.float64)[::-1]
_FAM_H = _FAM[:, 0].copy()
_FAM_PITCH = np.deg2rad(_FAM[:, 1])
_FAM_Q = np.deg2rad(_FAM[:, 2:5])                               # (9,3) hip_pitch, knee, ankle
_LEG_MASK = np.array([0, 0, 1, 1, 1, 0, 0, 1, 1, 1], dtype=np.float64)   # aile/oturus eklemleri (10 bacak)
_SEAT_L = np.deg2rad([C.SEAT_DEG[j] for j in C.CROUCH_JOINTS])
_SEAT_LEGS = np.concatenate([[0.0, 0.0], _SEAT_L, [0.0, 0.0], -_SEAT_L])     # maskeli eklemlerde mutlak
_HEAD_SIT = np.deg2rad([C.HEAD_SIT_DEG[j] for j in C.HEAD_JOINTS])


def main_command(xp, command):
    """Ham main komutu (N,5) [vx,vy,wz,h_cmd,sit] -> (vel (N,3), h (N,), sit (N,)) etkin komut:
    sit = [command[4] > 0.5]; h = clip(h_cmd, H_MIN_STAND, H_HOME); sit=1 ya da h < H_MIN_WALK iken vel = 0."""
    sit = xp.where(command[..., 4] > 0.5, 1.0, 0.0)
    h = xp.clip(command[..., 3], C.H_MIN_STAND, C.H_HOME)
    ok = xp.where((sit < 0.5) & (h >= C.H_MIN_WALK - 1e-6), 1.0, 0.0)
    return command[..., :3] * ok[..., None], h, sit


def command_obs(xp, command):
    """main gozlemindeki komut (N,5): [vx, vy, wz, h_n, sit], h_n = (h - H_HOME) / H_NORM in [-1, 0]."""
    vel, h, sit = main_command(xp, command)
    return xp.concatenate([vel, ((h - C.H_HOME) / C.H_NORM)[..., None], sit[..., None]], axis=-1)


def posture_obs(xp, h_ref, alpha):
    """(N,2) [h_ref_n, alpha]."""
    return xp.stack([(h_ref - C.H_HOME) / C.H_NORM, alpha], axis=-1)


def posture_step(xp, h_ref, alpha, command, dt=C.CTRL_DT):
    """Deterministik durus filtresi (robot calisma zamaninda birebir ayni; meta'da). command: ham (N,5).
      h_ref <- h_ref + clip(h - h_ref, +-H_RATE*dt)
      alpha <- alpha + clip(sit - alpha, -dt/STAND_RAMP_S, +dt/SIT_RAMP_S); |alpha - sit| < 1e-6 ise alpha = sit
    Hedefe tam olarak ulasir (son adimda fark tam toplanir; float32 birikim artigi 1e-6 icinde hedefe oturtulur)."""
    _, h, sit = main_command(xp, command)
    r = C.H_RATE * dt
    h_new = h_ref + xp.clip(h - h_ref, -r, r)
    a_new = alpha + xp.clip(sit - alpha, -dt / C.STAND_RAMP_S, dt / C.SIT_RAMP_S)
    a_new = xp.where(xp.abs(a_new - sit) < 1e-6, sit, a_new)
    return h_new, a_new


def sit_split(xp, alpha):
    """(s1, s2): s1 = clip(alpha/SIT_SPLIT, 0, 1) derin comelme,
    s2 = clip((alpha - SIT_SPLIT)/(1 - SIT_SPLIT), 0, 1) oturus."""
    s1 = xp.clip(alpha / C.SIT_SPLIT, 0.0, 1.0)
    s2 = xp.clip((alpha - C.SIT_SPLIT) / (1.0 - C.SIT_SPLIT), 0.0, 1.0)
    return s1, s2


def crouch_offset(xp, h):
    """(N,3) F(h) - F(H_HOME) (rad; hip_pitch, knee, ankle, SOL). h tablo disinda kirpilir (interp)."""
    h0 = xp.zeros_like(h) + C.H_HOME
    return xp.stack([xp.interp(h, _FAM_H, _FAM_Q[:, k]) - xp.interp(h0, _FAM_H, _FAM_Q[:, k])
                     for k in range(3)], axis=-1)


def crouch_pitch(xp, h):
    """(N,) ailenin govde pitch referansi (rad, + = burun asagi)."""
    return xp.interp(h, _FAM_H, _FAM_PITCH)


def posture_ref(xp, h_ref, alpha, home_q):
    """Referans durus (spec 3.2). home_q: (14,) HOME (idx.default_obs_q; ilk 10 bacak, son 4 bas). Dondurur:
      q_ref (N,10)      bacak hedefleri: lerp(F_home(hd), SEAT, s2), F_home(h) = HOME + F(h) - F(H_HOME)
      z_ref (N,)        lerp(hd, Z_SIT, s2)
      pitch_ref (N,)    lerp(pitch_F(hd), SEAT_PITCH (0: duz oturus), s2)
      head_offset (N,4) s1 * (HEAD_SIT - HEAD_HOME) (bas ctrl'una eklenir; HEAD_JOINTS sirasi)
    burada hd = lerp(h_ref, H_DEEP, s1). q_ref(H_HOME, 0) = HOME tam olarak."""
    s1, s2 = sit_split(xp, alpha)
    hd = h_ref + (C.H_DEEP - h_ref) * s1
    off = crouch_offset(xp, hd)
    z = xp.zeros_like(hd)
    delta = xp.stack([z, z, off[..., 0], off[..., 1], off[..., 2], z, z, -off[..., 0], -off[..., 1], -off[..., 2]],
                     axis=-1)
    home_l = home_q[..., :10]
    q_crouch = home_l + delta
    q_seat = home_l * (1.0 - _LEG_MASK) + _SEAT_LEGS
    q_ref = q_crouch + (q_seat - q_crouch) * s2[..., None]
    z_ref = hd + (C.Z_SIT - hd) * s2
    pitch_ref = _pitch_mix(xp, crouch_pitch(xp, hd), s2)
    head_offset = s1[..., None] * (_HEAD_SIT - home_q[..., 10:14])
    return {"q_ref": q_ref, "z_ref": z_ref, "pitch_ref": pitch_ref, "head_offset": head_offset}


def q_ref(xp, h_ref, alpha, home_q):
    return posture_ref(xp, h_ref, alpha, home_q)["q_ref"]


def z_ref(xp, h_ref, alpha, home_q=None):
    s1, s2 = sit_split(xp, alpha)
    hd = h_ref + (C.H_DEEP - h_ref) * s1
    return hd + (C.Z_SIT - hd) * s2


def _pitch_mix(xp, p_crouch, s2):
    """pitch_ref = lerp(pitch_F(hd), SEAT_PITCH, s2); duz oturusta (SEAT_PITCH_DEG = 0, xl330) eski ifade aynen."""
    if C.SEAT_PITCH_DEG:
        return p_crouch * (1.0 - s2) + float(np.deg2rad(C.SEAT_PITCH_DEG)) * s2
    return p_crouch * (1.0 - s2)


def pitch_ref(xp, h_ref, alpha, home_q=None):
    s1, s2 = sit_split(xp, alpha)
    return _pitch_mix(xp, crouch_pitch(xp, h_ref + (C.H_DEEP - h_ref) * s1), s2)


def head_offset_ref(xp, h_ref, alpha, home_q):
    s1, _ = sit_split(xp, alpha)
    return s1[..., None] * (_HEAD_SIT - home_q[..., 10:14])


def is_seated(xp, alpha):
    """alpha tam 1'e ulasti (filtre tam toplar; float32 icin 1e-6 tolerans)."""
    return xp.where(alpha >= 1.0 - 1e-6, 1.0, 0.0)


# --- main: segment ve sifirlama orneklemesi (rastgele sayi kaynagi arka ucta) ------------------
SEG_WALK, SEG_STAND, SEG_CROUCH, SEG_SIT = 0.0, 1.0, 2.0, 3.0
SEG_HOLD, SEG_ATOM = 4.0, 5.0       # tr35 (C.VOICE["atoms"]): oturus oncesi "tut" ve komut atomu (spec eki K6.2)
NUM_SEGMENT_U = 8      # sample_segment'in tukettigi U[0,1) sayisi (ortam basina)


def atom_velocities():
    """Komut atomlari (spec eki K6.2) (4,3) [vx, vy, wz]: (+k vx_max, 0, 0), (k vx_min, 0, 0), (0, 0, +k wz_max),
    (0, 0, -k wz_max); k = C.ATOM["speed"] (0.8), sinirlar C.COMMAND_RANGES'tan."""
    k, R = C.ATOM["speed"], C.COMMAND_RANGES
    return np.array([[k * R["vx"][1], 0.0, 0.0], [k * R["vx"][0], 0.0, 0.0], [0.0, 0.0, k * R["wz"][1]],
                     [0.0, 0.0, -k * R["wz"][1]]])


def sample_segment(xp, u, prev_command, seated=None, prev_kind=None):
    """Yeni main segmenti. u (N,8) U[0,1): [tur, sure, sifir-hiz, vx, vy, wz, h-modu, h].
    prev_command (N,5): onceki ham komut (sit=1 ise yeni segment sit olamaz; sit segmenti h'yi korur).
    seated (N,) 0/1 ya da None: oturus bankasindan sifirlamada ilk segment (u[0] < seated_first_sit_prob -> sit,
    yoksa stand). Dondurur (tur (N,) float SEG_*, komut (N,5), sure (N,) s).
    C.VOICE["atoms"] (tr35; spec eki K6.2; yeni sayi yok, ayni 8 uniform): walk segmentlerinin C.ATOM["frac"]'i atom
    (u[0] walk araliginin alt kesri): eksen u[4] (< 0.5 vx, yoksa wz), yon u[3] (vx: < 0.5 ileri) / u[5] (wz: < 0.5 sola),
    hiz atom_velocities(), sure U(C.ATOM["segment_s"]) (u[1]), h walk kurali. Her sit'ten once "tut" (SEG_HOLD): sifir hiz,
    onceki h, sure U(C.HOLD_S) (u[1]); prev_kind (N,) onceki segment turu SEG_HOLD ise bu segment sit olur (sifirlamada
    None: onceki segment yok). Oturustan baslangicin ilk sit'i tut istemez."""
    M = C.MAIN
    p = np.asarray(M["segment_probs"], dtype=np.float64)
    c = np.cumsum(p)
    prev_sit = prev_command[..., 4] > 0.5
    u0 = xp.where(prev_sit, u[..., 0] * (1.0 - p[3]), u[..., 0])
    kind = xp.where(u0 >= c[0], 1.0, 0.0) + xp.where(u0 >= c[1], 1.0, 0.0) + xp.where(u0 >= c[2], 1.0, 0.0)
    if seated is not None:
        first = xp.where(u[..., 0] < M["seated_first_sit_prob"], SEG_SIT, SEG_STAND)
        kind = xp.where(seated > 0.5, first, kind)
    lo, hi = M["segment_s"]
    dur = lo + (hi - lo) * u[..., 1]
    clo = np.array([C.COMMAND_RANGES[k][0] for k in ("vx", "vy", "wz")])
    chi = np.array([C.COMMAND_RANGES[k][1] for k in ("vx", "vy", "wz")])
    vel = clo + (chi - clo) * u[..., 3:6]
    is_w = xp.where(kind == SEG_WALK, 1.0, 0.0)
    vel = vel * (is_w * xp.where(u[..., 2] < C.COMMAND_ZERO_PROB, 0.0, 1.0))[..., None]
    h_walk = xp.where(u[..., 6] < M["walk_h_home_prob"], C.H_HOME, C.H_MIN_WALK + (C.H_HOME - C.H_MIN_WALK) * u[..., 7])
    h_crouch = C.H_MIN_STAND + (C.H_HOME - C.H_MIN_STAND) * u[..., 7]
    h = xp.where(kind == SEG_WALK, h_walk, xp.where(kind == SEG_CROUCH, h_crouch,
                                                     xp.where(kind == SEG_SIT, prev_command[..., 3], C.H_HOME)))
    sit = xp.where(kind == SEG_SIT, 1.0, 0.0)
    if not C.VOICE["atoms"]:
        return kind, xp.concatenate([vel, h[..., None], sit[..., None]], axis=-1), dur
    # --- tr35: komut atomlari ve oturus oncesi tut (spec eki K6.2) ---
    atom = (kind == SEG_WALK) & (u0 < C.ATOM["frac"] * c[0])
    av = atom_velocities()
    lin = u[..., 4] < 0.5
    v_lin = xp.where((u[..., 3] < 0.5)[..., None], av[0], av[1])
    v_rot = xp.where((u[..., 5] < 0.5)[..., None], av[2], av[3])
    vel = xp.where(atom[..., None], xp.where(lin[..., None], v_lin, v_rot), vel)
    alo, ahi = C.ATOM["segment_s"]
    dur = xp.where(atom, alo + (ahi - alo) * u[..., 1], dur)
    kind = xp.where(atom, SEG_ATOM, kind)
    after_hold = (prev_kind > SEG_HOLD - 0.5) & (prev_kind < SEG_HOLD + 0.5) if prev_kind is not None else \
        xp.zeros_like(prev_sit)
    first_sit = (seated > 0.5) & (kind == SEG_SIT) if seated is not None else xp.zeros_like(prev_sit)
    hold = (kind == SEG_SIT) & ~after_hold & ~first_sit
    to_sit = after_hold                                  # tut'tan sonra sit (onceki komut sit degil)
    hlo, hhi = C.HOLD_S
    kind = xp.where(to_sit, SEG_SIT, xp.where(hold, SEG_HOLD, kind))
    vel = xp.where((hold | to_sit)[..., None], 0.0, vel)
    h = xp.where(hold | to_sit, prev_command[..., 3], h)
    dur = xp.where(hold, hlo + (hhi - hlo) * u[..., 1], xp.where(to_sit, lo + (hi - lo) * u[..., 1], dur))
    sit = xp.where(kind == SEG_SIT, 1.0, 0.0)
    return kind, xp.concatenate([vel, h[..., None], sit[..., None]], axis=-1), dur


def reset_command(xp, alpha0):
    """Sifirlamada sample_segment'e verilecek "onceki" komut: [0, 0, 0, H_HOME, alpha0]."""
    z = xp.zeros_like(alpha0)
    return xp.stack([z, z, z, z + C.H_HOME, alpha0], axis=-1)


def sample_reset_kind(xp, u, task):
    """u (N,) U[0,1) -> sifirlama turu indeksi (N,) float, TaskSpec.reset_probs sirasiyla
    (main: 0 home, 1 seated; getup: 0 fallen, 1 midfall, 2 stand)."""
    c = np.cumsum([p for _, p in TK.get_task(task).reset_probs])
    k = xp.zeros_like(u)
    for b in c[:-1]:
        k = k + xp.where(u >= b, 1.0, 0.0)
    return k


def bank_index(xp, u, n):
    """u U[0,1) -> banka indeksi floor(u*n) (n-1'e kirpilir; int32)."""
    return xp.minimum(xp.floor(u * n), n - 1).astype(xp.int32)


# --- jest kanali (spec eki K6.1): env_cpu, env_mjx ve rl/commands.py AYNI fonksiyonu cagirir ---------------------
# Cikti (N,5) ofset [rad], GEST_CHANNELS sirasi (4 bas eklemi + gaga); bas kontrolu = varsayilan + filtreli rastgele
# ofset + jest (jest filtreyi atlar). Durum {"kind", "n", "amp", "cut_n"} (N,) float: tur (GEST_*), baslangictan beri
# adim, genlik carpani (oturusta gaga siniri), kesmeden beri adim (-1: kesilmedi). Zaman = adim * CTRL_DT (birikmez).
GEST_NONE, GEST_NOD, GEST_SHAKE, GEST_BEAK, GEST_CLICK, GEST_CLICK2 = 0.0, 1.0, 2.0, 3.0, 4.0, 5.0
GEST_NAMES = ("none", "nod", "shake", "beak", "click", "click2")
GEST_TRAIN = (GEST_NOD, GEST_SHAKE, GEST_BEAK, GEST_CLICK)          # egitimde orneklenen turler (spec K6.1 tablosu)
GEST_CHANNELS = tuple(C.HEAD_JOINTS) + tuple(C.OTHER_JOINTS)


def _gesture_table():
    """Tur basina sabitler (numpy, GEST_NAMES sirasi): kanal, genlik, periyot, sure, Tukey yari cevrimi, sekil (0 yok,
    1 sin, 2 acik (0 <-> genlik), 3 darbe), bas jesti mi, oturus genlik carpani."""
    rows = [(0, 0.0, 1.0, 0.0, 0.0, 0, 0.0, 1.0)]
    for name in GEST_NAMES[1:]:
        g = C.GESTURES[name]
        ch = GEST_CHANNELS.index(g["joint"])
        if g["shape"] == "pulse":
            per, dur, taper, shape = g["pulse_s"], g["pulse_s"] * g["pulses"], 0.0, 3
        else:
            per = 1.0 / g["freq"]
            dur, taper, shape = per * g["cycles"], 0.5 * per, (1 if g["shape"] == "sin" else 2)
        head = 1.0 if g["joint"] in C.HEAD_JOINTS else 0.0
        cap = 1.0 if head else min(1.0, float(np.deg2rad(C.GESTURE_SEATED_MAX_DEG)) / g["amp"])
        rows.append((ch, float(g["amp"]), per, dur, taper, shape, head, cap))
    t = np.array(rows, dtype=np.float64)
    return {"ch": t[:, 0].astype(np.int64), "amp": t[:, 1], "per": t[:, 2], "dur": t[:, 3], "taper": t[:, 4],
            "shape": t[:, 5], "head": t[:, 6], "cap": t[:, 7],
            "onehot": np.eye(len(GEST_CHANNELS))[t[:, 0].astype(np.int64)] * (t[:, 5:6] > 0)}


GEST_TAB = _gesture_table()


def gesture_init(xp, like):
    """Bos jest durumu (like (N,) sablonu)."""
    z = xp.zeros_like(like)
    return {"kind": z, "n": z + 0.0, "amp": z + 1.0, "cut_n": z - 1.0}


def _gtake(xp, key, k, like):
    """GEST_TAB[key][k], like'in dtype'inda (commands.py float32, env_cpu float64, MJX float32)."""
    return xp.asarray(GEST_TAB[key], dtype=like.dtype)[k]


def gesture_raw(xp, kind, t, amp):
    """Kesmesiz jest sinyali (N,5): sin/acik sekilde Tukey zarfi (yarim cevrim acilis ve kapanis, ortadaki tam cevrim
    tam genlik; t = 0 ve t = sure'de konum ve hiz 0), darbe 0.5 (1 - cos) (tik). t (N,) s, amp (N,) genlik carpani."""
    k = xp.clip(kind, 0.0, len(GEST_NAMES) - 1.0).astype(xp.int32)
    A, P, D = _gtake(xp, "amp", k, t) * amp, _gtake(xp, "per", k, t), _gtake(xp, "dur", k, t)
    tt, sh = xp.maximum(_gtake(xp, "taper", k, t), 1e-9), _gtake(xp, "shape", k, t)
    inside = xp.where((t >= 0.0) & (t <= D), t * 0.0 + 1.0, t * 0.0)      # t'nin dtype'i (float32 korunur)
    env = xp.minimum(0.5 - 0.5 * xp.cos(np.pi * xp.clip(t / tt, 0.0, 1.0)),
                     0.5 - 0.5 * xp.cos(np.pi * xp.clip((D - t) / tt, 0.0, 1.0)))
    ph = 2.0 * np.pi * t / P
    cyc = 0.5 - 0.5 * xp.cos(ph)
    v = xp.where(sh == 1.0, env * xp.sin(ph), xp.where(sh == 2.0, env * cyc, xp.where(sh == 3.0, cyc, 0.0)))
    return (A * v * inside)[..., None] * _gtake(xp, "onehot", k, t)


def gesture_fade(xp, cut_n, dt=C.CTRL_DT):
    """Kesme sonumleme carpani: kesilmedi 1; kesmeden tau s sonra 0.5 (1 + cos(pi tau / GESTURE_FADE_S))."""
    tau = cut_n * dt
    w = 0.5 + 0.5 * xp.cos(np.pi * xp.clip(tau / C.GESTURE_FADE_S, 0.0, 1.0))
    return xp.where(cut_n < 0.0, 1.0, w)


def gesture_gate(xp, kind, alpha):
    """Baslatma kapisi (0/1): bas jestleri alpha < ALPHA_STAND_MAX iken; gaga (gaga, tik) alpha < ALPHA_STAND_MAX ya da
    alpha = 1 iken. Dondurur (kapi, genlik carpani: alpha = 1 ve ayakta degilse oturus siniri)."""
    k = xp.clip(kind, 0.0, len(GEST_NAMES) - 1.0).astype(xp.int32)
    stand = alpha < C.ALPHA_STAND_MAX
    seated = is_seated(xp, alpha) > 0.5
    ok = xp.where(_gtake(xp, "head", k, alpha) > 0.5, stand, stand | seated)
    return xp.where(ok & (kind > 0.5), 1.0, 0.0), xp.where(stand, 1.0, _gtake(xp, "cap", k, alpha))


def gesture_step(xp, st, req, alpha, cut, dt=C.CTRL_DT):
    """Jest kanalinin bir kontrol adimi (spec eki K6.1). st: gesture_init durumu; req (N,) baslatma istegi (GEST_*, 0
    yok); alpha (N,) durus filtresi (kapi yalniz baslangicta); cut (N,) 0/1 kesme (dur, otur, dusme, segment degisimi).
    Sira: kesme (calisan, kesilmemis jest -> sonumleme baslar), baslatma (kanal bos, istek, kapi; jestler kuyruga girmez:
    kanal doluyken istek yok sayilir), cikti (bu adimin zamaninda; kesildiyse x gesture_fade), ilerletme ve bitis (sure
    ya da GESTURE_FADE_S doldu). Dondurur (st', out (N,5) ofset rad, active (N,) 0/1 bu adimda jest var (sonumleme dahil),
    started (N,) 0/1 bu adimda basladi)."""
    kind, n, amp, cn = st["kind"], st["n"], st["amp"], st["cut_n"]
    on = kind > 0.5
    cn = xp.where(on & (cut > 0.5) & (cn < 0.0), 0.0, cn)
    gate, cap = gesture_gate(xp, req, alpha)
    go = (~on) & (gate > 0.5)
    kind = xp.where(go, req, kind)
    n = xp.where(go, 0.0, n)
    amp = xp.where(go, cap, amp)
    cn = xp.where(go, -1.0, cn)
    on = kind > 0.5
    out = gesture_raw(xp, kind, n * dt, amp) * gesture_fade(xp, cn, dt)[..., None]
    n2 = n + 1.0
    cn2 = xp.where(cn >= 0.0, cn + 1.0, cn)
    D = _gtake(xp, "dur", xp.clip(kind, 0.0, len(GEST_NAMES) - 1.0).astype(xp.int32), n)
    end = on & ((n2 * dt > D + 1e-6) | (cn2 * dt > C.GESTURE_FADE_S + 1e-6))    # son ornek t = sure (cikti 0)
    st2 = {"kind": xp.where(end, 0.0, kind), "n": xp.where(on & ~end, n2, 0.0), "amp": xp.where(end, 1.0, amp),
           "cut_n": xp.where(on & ~end, cn2, -1.0)}
    return st2, out, xp.where(on, 1.0, 0.0), xp.where(go, 1.0, 0.0)


def gesture_sample(xp, u0, u1):
    """Egitim bas yeniden orneklemesi (C.GESTURE_SAMPLE, mevcut 6 bas uniform'undan): u0 < p_jest -> jest (tur
    GEST_TRAIN[floor(4 u1)]), < p_jest + p_rastgele -> rastgele hedef, yoksa sifir. Dondurur (istek (N,) GEST_*, rastgele
    (N,) 0/1)."""
    pg, pr, _ = C.GESTURE_SAMPLE
    g = u0 < pg
    k = xp.minimum(xp.floor(u1 * len(GEST_TRAIN)), len(GEST_TRAIN) - 1.0)
    return xp.where(g, 1.0 + k, 0.0), xp.where((~g) & (u0 < pg + pr), 1.0, 0.0)


_RATE_CACHE = {}


def gesture_rate_bounds(fine_dt=5e-4):
    """Kanal basina (5,) jest ciktisinin |d/dt| ust siniri (rad/s): tam genlik, her baslangic ve kesme aninda (sonumleme
    dahil; ince izgarada sayisal). Durumdan bagimsiz sabit; head_slew_bounds ve testler kullanir."""
    if fine_dt in _RATE_CACHE:
        return _RATE_CACHE[fine_dt].copy()
    out = np.zeros(len(GEST_CHANNELS))
    F = C.GESTURE_FADE_S
    tf = np.arange(0.0, F + fine_dt, fine_dt)
    w = gesture_fade(np, np.round(tf / fine_dt), fine_dt)
    wd = np.gradient(w, fine_dt)
    for k in range(1, len(GEST_NAMES)):
        D = GEST_TAB["dur"][k]
        t = np.arange(-F, D + F + fine_dt, fine_dt)
        g = gesture_raw(np, np.full(len(t), float(k)), t, np.ones(len(t)))[:, GEST_TAB["ch"][k]]
        gd = np.gradient(g, fine_dt)
        m = len(tf)
        best = np.max(np.abs(gd))
        for i0 in range(0, len(t) - m, 4):                        # kesme aninin her konumu (2 ms adim)
            best = max(best, float(np.max(np.abs(gd[i0:i0 + m] * w + g[i0:i0 + m] * wd))))
        ch = GEST_TAB["ch"][k]
        out[ch] = max(out[ch], best)
    _RATE_CACHE[fine_dt] = out
    return out.copy()


def slew_limits():
    """(5,) GEST_CHANNELS sirasinda toplam bas/gaga ctrl hiz siniri 0.5 omega0 (MG90S, SG90; tr35 servo siniflari)."""
    from . import actuators as A
    return np.array([0.5 * A.CLASSES[A.JOINT_CLASS[A.TR35][j]]["omega0"] for j in GEST_CHANNELS])


def head_slew_bounds(random_head=True):
    """Toplam bas ctrl'unun (varsayilan + filtreli rastgele ofset + oturus scripti + jest) |d/dt| ust siniri (5,)
    rad/s, GEST_CHANNELS sirasi. Rastgele ofset: jest yokken 2 r / tau, jest suresince (hedef 0) r / tau; script:
    |HEAD_SIT - HEAD_HOME| / (SIT_SPLIT min(SIT_RAMP_S, STAND_RAMP_S)); gaga ofsetsiz. random_head=False: komut katmani
    (rastgele ofset yok)."""
    g = gesture_rate_bounds()
    hr = C.HEAD_RANDOM
    r = np.array([hr["range"][j] for j in C.HEAD_JOINTS] + [0.0])
    off = r / hr["filter_tau"] if random_head else 0.0 * r
    script = np.array([abs(np.deg2rad(C.HEAD_SIT_DEG[j] - C.HEAD_HOME_DEG[j])) for j in C.HEAD_JOINTS] + [0.0]) \
        / (C.SIT_SPLIT * min(C.SIT_RAMP_S, C.STAND_RAMP_S))
    return np.maximum(2.0 * off, off + g) + script


def check_head_slew(random_head=True):
    """spec eki K6.1 hiz siniri: head_slew_bounds <= slew_limits (assert; tr35 jest kanali acik ortamlar ve
    rl/commands.py kurulumda cagirir). Dondurur (sinir, limit)."""
    b, lim = head_slew_bounds(random_head), slew_limits()
    assert np.all(b <= lim), "jest + bas ctrl hizi siniri asiliyor: %s > %s" % (b.round(3).tolist(), lim.tolist())
    return b, lim


# --- gozlemler ----------------------------------------------------------------
def observation(xp, s, task="walk"):
    """Aktor gozlemi (N, num_obs), gurultusuz. Gurultu: obs + noise_vector() * U(-1,1)."""
    name = _name(task)
    g = projected_gravity(xp, s["quat"])
    if name == "walk":
        obs = xp.concatenate([
            g,
            s["gyro"] * C.OBS_SCALES["gyro"],
            s["q"] - s["default_q"],
            s["qd"] * C.OBS_SCALES["joint_vel"],
            s["last_action"],
            s["command"],
            phase_features(xp, s["phase"]),
        ], axis=-1)
        return xp.clip(obs, -C.OBS_CLIP, C.OBS_CLIP)
    parts = [g, s["gyro"] * C.OBS_SCALES["gyro"], s["q"] - s["default_q"], s["qd"] * C.OBS_SCALES["joint_vel"],
             s["last_action"]]
    if name == "main":
        parts += [command_obs(xp, s["command"]), posture_obs(xp, s["h_ref"], s["alpha"]),
                  phase_features(xp, s["phase"])]
    elif name != "getup":
        raise ValueError(name)
    return xp.clip(xp.concatenate(parts, axis=-1), -C.OBS_CLIP, C.OBS_CLIP)


def noise_vector(xp, task="walk"):
    """(num_obs,) gozlem gurultusu genlikleri (olceklenmis birimlerde)."""
    n = C.OBS_NOISE
    if _name(task) == "walk":
        parts = ([n["gravity"]] * 3 + [n["gyro"] * C.OBS_SCALES["gyro"]] * 3 + [n["joint_pos"]] * 14
                 + [n["joint_vel"] * C.OBS_SCALES["joint_vel"]] * 14 + [0.0] * (C.NUM_ACTIONS + 3 + 2))
        return xp.asarray(parts)
    amp = {"gravity": n["gravity"], "gyro": n["gyro"] * C.OBS_SCALES["gyro"], "joint_pos": n["joint_pos"],
           "joint_vel": n["joint_vel"] * C.OBS_SCALES["joint_vel"]}
    parts = []
    for k, size in TK.get_task(task).obs_layout:
        parts += [amp.get(k, 0.0)] * size
    return xp.asarray(parts)


def critic_observation(xp, s, task="walk"):
    """Kritik (ayricalikli) gozlem (N, num_critic_obs) = aktor (gurultusuz) + TaskSpec.critic_layout."""
    name = _name(task)
    obs = observation(xp, s, task)
    if name == "walk":
        return xp.concatenate([
            obs,
            s["linvel"],
            s["trunk_z"][..., None],
            s["contact"],
            s["foot_pos"][..., 2],
            s["friction"][..., None],
        ], axis=-1)
    feat = {"linvel": lambda: s["linvel"], "trunk_z": lambda: s["trunk_z"][..., None],
            "foot_contact": lambda: s["contact"], "foot_z": lambda: s["foot_pos"][..., 2],
            "friction": lambda: s["friction"][..., None], "trunk_floor": lambda: s["trunk_floor"][..., None],
            "head_floor": lambda: s["head_floor"][..., None]}
    return xp.concatenate([obs] + [feat[k]() for k, _ in TK.get_task(task).critic_layout], axis=-1)


# --- oduller ------------------------------------------------------------------
def reward_terms(xp, s, task="walk"):
    """Agirliksiz odul terimleri, her biri (N,). Anahtar kumesi = TaskSpec.reward_weights (statik).
    main/getup'ta mod kapilari terimlerin ICINDEDIR (agirliklar sabit/mufredat)."""
    name = _name(task)
    if name == "main":
        return _main_terms(xp, s)
    if name == "getup":
        return _getup_terms(xp, s)
    if name != "walk":
        raise ValueError(name)
    cmd = s["command"]
    g = projected_gravity(xp, s["quat"])
    lin_err = xp.sum(xp.square(cmd[..., :2] - s["linvel"][..., :2]), axis=-1)
    ang_err = xp.square(cmd[..., 2] - s["gyro"][..., 2])
    moving = xp.where(xp.sum(xp.square(cmd), axis=-1) > C.MOVING_EPS ** 2, 1.0, 0.0)

    foot_z = s["foot_pos"][..., 2]
    feet_phase = xp.exp(-xp.sum(xp.square(foot_z - desired_foot_height(xp, s["phase"])), axis=-1)
                        / C.FOOT_HEIGHT_SIGMA)
    slip = xp.sum(s["contact"] * xp.sum(xp.square(s["foot_vel"][..., :2]), axis=-1), axis=-1)

    yaw = yaw_angle(xp, s["quat"])
    d = s["foot_pos"][..., 0, :] - s["foot_pos"][..., 1, :]
    lateral = -xp.sin(yaw) * d[..., 0] + xp.cos(yaw) * d[..., 1]
    hip_idx = [C.OBS_JOINTS.index(n) for n in C.HIP_POSE_JOINTS]
    dq = s["q"] - s["default_q"]
    hip = xp.sum(xp.square(xp.stack([dq[..., i] for i in hip_idx], axis=-1)), axis=-1)

    out = {
        "tracking_lin_vel": xp.exp(-lin_err / C.TRACKING_SIGMA_LIN),
        "tracking_ang_vel": xp.exp(-ang_err / C.TRACKING_SIGMA_ANG),
        "lin_vel_z": xp.square(s["linvel"][..., 2]),
        "ang_vel_xy": xp.sum(xp.square(s["gyro"][..., :2]), axis=-1),
        "orientation": xp.sum(xp.square(g[..., :2]), axis=-1),
        "base_height": xp.square(s["trunk_z"] - C.BASE_HEIGHT_TARGET),
    }
    out.update(_effort_terms(xp, s))
    out.update({
        "joint_acc": xp.sum(xp.square(s["qdd_legs"]), axis=-1),
        "action_rate": xp.sum(xp.square(s["action"] - s["last_action"]), axis=-1),
        "feet_air_time": moving * xp.sum((s["air_time"] - C.AIR_TIME_TARGET) * s["first_contact"], axis=-1),
        "feet_phase": feet_phase,
        "feet_slip": slip,
        "feet_distance": xp.maximum(C.FEET_MIN_DISTANCE - lateral, 0.0),
        "hip_pose": hip,
        "alive": xp.ones_like(s["trunk_z"]),
    })
    out.update(_limit_rail_terms(xp, s))
    return out


def main_gates(xp, s):
    """(moving, g_step, still): moving = [|vel| > moving_eps] (etkin komut), g_step = moving*(1-alpha),
    still = 1 - g_step."""
    vel, _, _ = main_command(xp, s["command"])
    moving = xp.where(xp.sum(xp.square(vel), axis=-1) > C.MAIN["moving_eps"] ** 2, 1.0, 0.0)
    g_step = moving * (1.0 - s["alpha"])
    return moving, g_step, 1.0 - g_step


def _main_terms(xp, s):
    M = C.MAIN
    vel, _, sit = main_command(xp, s["command"])
    ref = posture_ref(xp, s["h_ref"], s["alpha"], s["default_q"])
    g = projected_gravity(xp, s["quat"])
    lin_err = xp.sum(xp.square(vel[..., :2] - s["linvel"][..., :2]), axis=-1)
    ang_err = xp.square(vel[..., 2] - s["gyro"][..., 2])
    _, g_step, still = main_gates(xp, s)

    foot_z = s["foot_pos"][..., 2]
    feet_phase = xp.exp(-xp.sum(xp.square(foot_z - desired_foot_height(xp, s["phase"])), axis=-1)
                        / C.FOOT_HEIGHT_SIGMA)
    slip = xp.sum(s["contact"] * xp.sum(xp.square(s["foot_vel"][..., :2]), axis=-1), axis=-1)
    yaw = yaw_angle(xp, s["quat"])
    d = s["foot_pos"][..., 0, :] - s["foot_pos"][..., 1, :]
    lateral = -xp.sin(yaw) * d[..., 0] + xp.cos(yaw) * d[..., 1]
    dq = s["q"] - s["default_q"]
    hip = xp.sum(xp.square(xp.stack([dq[..., C.OBS_JOINTS.index(n)] for n in C.HIP_POSE_JOINTS], axis=-1)),
                 axis=-1)

    dz = s["trunk_z"] - ref["z_ref"]
    pr = ref["pitch_ref"]
    both = s["contact"][..., 0] * s["contact"][..., 1]
    seat = sit * is_seated(xp, s["alpha"]) * s["trunk_floor"] * xp.where(g[..., 2] < M["seat_gz"], 1.0, 0.0)
    vz = world_vz(xp, s["quat"], s["linvel"])
    out = {
        "tracking_lin_vel": xp.exp(-lin_err / C.TRACKING_SIGMA_LIN),
        "tracking_ang_vel": xp.exp(-ang_err / C.TRACKING_SIGMA_ANG),
        "lin_vel_z": xp.square(s["linvel"][..., 2]),
        "ang_vel_xy": xp.sum(xp.square(s["gyro"][..., :2]), axis=-1),
        "orientation": xp.square(g[..., 0] - xp.sin(pr)) + xp.square(g[..., 1]),
    }
    out.update(_effort_terms(xp, s))
    out.update({
        "joint_acc": xp.sum(xp.square(s["qdd_legs"]), axis=-1),
        "action_rate": xp.sum(xp.square(s["action"] - s["last_action"]), axis=-1),
        "feet_air_time": g_step * xp.sum((s["air_time"] - C.AIR_TIME_TARGET) * s["first_contact"], axis=-1),
        "feet_phase": g_step * feet_phase,
        "feet_slip": slip,
        "feet_distance": xp.maximum(C.FEET_MIN_DISTANCE - lateral, 0.0),
        "hip_pose": hip,
        "alive": xp.ones_like(s["trunk_z"]),
        "height_track": xp.exp(-xp.square(dz / M["height_sigma"])),
        "height_l1": xp.abs(dz),
        "posture": still * xp.sum(xp.square(s["q"][..., :10] - ref["q_ref"]), axis=-1),
        "stand_contact": still * (1.0 - s["alpha"]) * both,
        "seat": seat,
        "vz_cap": xp.maximum(xp.abs(vz) - M["vz_cap"], 0.0),
    })
    out.update(_limit_rail_terms(xp, s))
    return out


def getup_potentials(xp, s):
    """(phi_up, phi_h) = (-gz, min(z, H_HOME))."""
    g = projected_gravity(xp, s["quat"])
    return -g[..., 2], xp.minimum(s["trunk_z"], C.H_HOME)


def getup_memory(xp, s):
    """Bir sonraki adimin "onceki" alanlari: {phi_up_prev, phi_h_prev, head_floor_prev, head_v2_prev} (odulden
    SONRA ve sifirlamada bu durumdan yazilir)."""
    up, h = getup_potentials(xp, s)
    return {"phi_up_prev": up, "phi_h_prev": h, "head_floor_prev": s["head_floor"],
            "head_v2_prev": xp.sum(xp.square(s["head_vel"]), axis=-1)}


def head_impact_v2(xp, s):
    """Kafa temasi baslangicinda carpma hizi^2: max(onceki adim, bu adim) |v_head|^2. Temas, kontrol adiminin
    4 alt adimindan herhangi birinde baslamis olabilir; adim sonu hizi cogu zaman carpmadan SONRAKI hizdir
    (S4 review: medyan 0.2x). Onceki adimin hizi carpma oncesi hizdir."""
    v2 = xp.sum(xp.square(s["head_vel"]), axis=-1)
    return xp.maximum(s.get("head_v2_prev", v2), v2)


def pose_rms(xp, s):
    return xp.sqrt(xp.mean(xp.square(s["q"] - s["default_q"]), axis=-1))


def tilt_angle(xp, quat):
    """Govde z ekseninin dikeyle acisi (rad) = arccos(-gz)."""
    return xp.arccos(xp.clip(-projected_gravity(xp, quat)[..., 2], -1.0, 1.0))


def _getup_terms(xp, s):
    G = C.GETUP
    z = s["trunk_z"]
    dq = s["q"] - s["default_q"]
    rms = xp.sqrt(xp.mean(xp.square(dq), axis=-1))
    phi_up, phi_h = getup_potentials(xp, s)
    sg, ss = G["height_sigma"]
    z0, z1 = G["pose_z"]
    a0, a1 = G["arrival_z"]
    cz, ct, cp = G["composite_sigma"]
    dz = z - C.H_HOME
    tilt = tilt_angle(xp, s["quat"])
    onset = s["head_floor"] * (1.0 - s["head_floor_prev"])
    if _TR35:                          # doygunluk |tau_net/tau_s| > torque_sat_frac (bolum tau_s'i)
        sat = xp.sum(xp.maximum(xp.abs(s["tau_ratio"]) - G["torque_sat_frac"], 0.0), axis=-1)
    else:
        sat = xp.sum(xp.maximum(xp.abs(s["torque"]) - G["torque_sat_frac"] * G["torque_limit"], 0.0), axis=-1)
    out = {
        "upright": phi_up,
        "height_gauss": xp.exp(-xp.square(dz / sg)),
        "height_sharp": xp.exp(-xp.square(dz / ss)),
        "height_l1": xp.abs(dz),
        "upright_progress": (phi_up - s["phi_up_prev"]) / C.CTRL_DT,
        "height_progress": (phi_h - s["phi_h_prev"]) / C.CTRL_DT,
        "pose": xp.mean(xp.exp(-xp.square(dq / G["pose_sigma"])), axis=-1) * smoothstep(xp, (z - z0) / (z1 - z0)),
        "composite": xp.exp(-xp.square(dz / cz)) * xp.exp(-xp.square(tilt / ct)) * xp.exp(-xp.square(rms / cp)),
        "torque_sat": sat,
        # olay terimi: temas basladigi adimda carpma hizi^2 (head_impact_v2); total_reward'in CTRL_DT carpimini
        # iptal etmek icin / CTRL_DT
        "head_impact": onset * head_impact_v2(xp, s) / C.CTRL_DT,
        "action_rate": xp.sum(xp.square(s["action"] - s["last_action"]), axis=-1),
        "joint_vel": xp.sum(xp.square(s["qd"]), axis=-1),
        "arrival_damping": xp.sum(xp.square(s["gyro"][..., :2]), axis=-1) * smoothstep(xp, (z - a0) / (a1 - a0)),
    }
    out.update(_limit_rail_terms(xp, s))
    return out


def late_ramp(xp, progress, task="getup"):
    """Gec terimlerin karisim katsayisi: clip((p - p0) / (p1 - p0), 0, 1) (late_p yoksa 0)."""
    lp = TK.get_task(task).late_p
    if lp is None:
        return xp.zeros_like(progress)
    p0, p1 = lp
    return xp.clip((progress - p0) / (p1 - p0), 0.0, 1.0)


def reward_weights(xp, task="walk", progress=None):
    """{terim: w} (saniye basina). walk/main: sabit (python float). getup: gec terimler (reward_late)
    ilerleme p'ye gore reward_weights[k] -> reward_late[k] arasinda lineer; progress None ise p = 0
    (erken agirliklar). progress skaler ya da (N,) olabilir; donen degerler ona yayilir."""
    t = TK.get_task(task)
    w = dict(t.reward_weights)
    if t.reward_late and progress is not None:
        r = late_ramp(xp, xp.asarray(progress), t)
        for k, w1 in t.reward_late.items():
            w[k] = w[k] + (w1 - w[k]) * r
    return w


def total_reward(xp, terms, terminated, weights=None):
    """sum(w_i * r_i) * CTRL_DT + sonlandirma cezasi. weights None: walk (C.REWARD_WEIGHTS)."""
    r = 0.0
    for k, w in (C.REWARD_WEIGHTS if weights is None else weights).items():
        r = r + w * terms[k]
    return r * C.CTRL_DT + C.TERMINATION_PENALTY * terminated


def terminated(xp, s, task="walk"):
    """Dusme (N,) 0/1 (bozuk fizik arka ucta ayrica OR'lanir).
    walk: govde cok alcak ya da cok egik. main: gz > TERMINATION_MAX_GRAVITY_Z, ya da head_floor, ya da
    planlanmamis oturma (trunk_floor ve alpha < unplanned_sit_alpha); yukseklik kurali yok. getup: hic."""
    name = _name(task)
    g = projected_gravity(xp, s["quat"])
    if name == "walk":
        fell = (s["trunk_z"] < C.TERMINATION_MIN_HEIGHT) | (g[..., 2] > C.TERMINATION_MAX_GRAVITY_Z)
    elif name == "main":
        fell = ((g[..., 2] > C.TERMINATION_MAX_GRAVITY_Z) | (s["head_floor"] > 0.5)
                | ((s["trunk_floor"] > 0.5) & (s["alpha"] < C.MAIN["unplanned_sit_alpha"])))
    elif name == "getup":
        return xp.zeros_like(s["trunk_z"])
    else:
        raise ValueError(name)
    return xp.where(fell, 1.0, 0.0)


def getup_success_now(xp, s, use_height=True):
    """Anlik basari kosulu (N,) 0/1: gz < -0.9, (z > 0.10), pose_rms_14 < 0.3 rad, |gyro| < 0.5 rad/s.
    use_height=False: robot calisma zamani devri (z olculmez)."""
    S = C.GETUP["success"]
    g = projected_gravity(xp, s["quat"])
    ok = (g[..., 2] < S["gz"]) & (pose_rms(xp, s) < S["pose_rms"]) \
        & (xp.sqrt(xp.sum(xp.square(s["gyro"]), axis=-1)) < S["gyro"])
    if use_height:
        ok = ok & (s["trunk_z"] > S["z"])
    return xp.where(ok, 1.0, 0.0)


def hold_timer(xp, timer, ok, hold_s, dt=C.CTRL_DT):
    """Kosulun kesintisiz suresi: (timer', held) ; timer' = (timer + dt) ok iken, yoksa 0;
    held = [timer' >= hold_s - dt/2] (0/1)."""
    t = xp.where(ok > 0.5, timer + dt, 0.0)
    return t, xp.where(t >= hold_s - 0.5 * dt, 1.0, 0.0)


# --- bolum metrikleri (olay sayaclari; per-step artislar episode toplamina eklenir) -------------
def main_events_init(xp, alpha0):
    """Sifirlamada olay durumu: prev_sit = alpha0 (oturustan baslayan bolumde 'oturuyordu')."""
    z = xp.zeros_like(alpha0)
    return {"prev_sit": alpha0 + z, "mode_t": z, "standup_pending": z, "sitdown_pending": z}


def main_events(xp, s, ev, term, dt=C.CTRL_DT):
    """main metrikleri. s: bu adimin durumu (komut c_t, alpha/h_ref guncel), ev: olay durumu,
    term: (N,) 0/1 sonlandirma (bozuk fizik dahil). Dondurur (ev', artislar {TaskSpec.episode_metrics}).
      walk_steps = sit=0 ve alpha < ALPHA_STAND_MAX; sit_steps = sit=1; seated_ok_steps = seat odulu kosulu
      sitdown_attempts/standup_attempts: sit 0->1 / 1->0 donusu (prev_sit'e gore); kalkis yalniz h komutu
        >= standup_z + 5 mm iken sayilir (alcak comelmeye kalkis z > standup_z'ye hic ulasmaz); mode_t donusten beri sure
      sitdown_success: donusten sonra sitdown_window_s icinde seat kosulu (bir kez)
      standup_success: donusten sonra standup_window_s icinde z > standup_z, iki ayak yerde, gz < stand_gz (bir kez)
      falls_walk / falls_sit / falls_transition: term ve (walk modu / alpha=1 ve sit=1 / digerleri)."""
    M = C.MAIN
    _, h_cmd, sit = main_command(xp, s["command"])
    g = projected_gravity(xp, s["quat"])
    a = s["alpha"]
    up = xp.where((ev["prev_sit"] > 0.5) & (sit < 0.5), 1.0, 0.0)
    up_ok = up * xp.where(h_cmd >= M["standup_z"] + 0.005, 1.0, 0.0)      # sayilan kalkis denemesi
    down = xp.where((ev["prev_sit"] < 0.5) & (sit > 0.5), 1.0, 0.0)
    flip = xp.maximum(up, down)
    mode_t = ev["mode_t"] * (1.0 - flip) + dt
    up_seat = xp.where(g[..., 2] < M["seat_gz"], 1.0, 0.0)          # oturus dikligi (tr35: egik oturus)
    up_stand = xp.where(g[..., 2] < M["stand_gz"], 1.0, 0.0)        # ayakta dikligi (xl330: ikisi -0.9)
    seated_ok = sit * is_seated(xp, a) * s["trunk_floor"] * up_seat
    stood = xp.where(s["trunk_z"] > M["standup_z"], 1.0, 0.0) * s["contact"][..., 0] * s["contact"][..., 1] * up_stand
    su_pend = xp.maximum(ev["standup_pending"] * (1.0 - down), up_ok)
    sd_pend = xp.maximum(ev["sitdown_pending"] * (1.0 - up), down)
    su_win = xp.where(mode_t <= M["standup_window_s"] + 0.5 * dt, 1.0, 0.0)
    sd_win = xp.where(mode_t <= M["sitdown_window_s"] + 0.5 * dt, 1.0, 0.0)
    su = su_pend * su_win * stood
    sd = sd_pend * sd_win * seated_ok
    walk_mode = (1.0 - sit) * xp.where(a < C.ALPHA_STAND_MAX, 1.0, 0.0)
    sit_mode = sit * is_seated(xp, a)
    ev_new = {"prev_sit": sit, "mode_t": mode_t,
              "standup_pending": su_pend * su_win * (1.0 - su), "sitdown_pending": sd_pend * sd_win * (1.0 - sd)}
    inc = {"walk_steps": walk_mode, "sit_steps": sit, "seated_ok_steps": seated_ok,
           "standup_attempts": up_ok, "standup_success": su, "sitdown_attempts": down, "sitdown_success": sd,
           "falls_walk": term * walk_mode, "falls_sit": term * sit_mode,
           "falls_transition": term * (1.0 - walk_mode) * (1.0 - sit_mode)}
    return ev_new, inc


def getup_events_init(xp, z_like):
    z = xp.zeros_like(z_like)
    return {"success_t": z, "succeeded": z}


def getup_events(xp, s, ev, dt=C.CTRL_DT):
    """getup metrikleri. Dondurur (ev', artislar): getup_success (basari kosulu hold_s boyunca ilk saglandigi
    adimda 1, bolumde bir kez), head_impacts (kafa temas baslangici sayisi), head_impact_v2 (baslangiclarda
    carpma hizi^2 toplami, head_impact_v2()), torque_sat_steps (|tau| > torque_sat_frac*limit olan adim; tr35:
    max |tau_ratio| > torque_sat_frac)."""
    G = C.GETUP
    t, held = hold_timer(xp, ev["success_t"], getup_success_now(xp, s), G["success"]["hold_s"], dt)
    first = held * (1.0 - ev["succeeded"])
    onset = s["head_floor"] * (1.0 - s["head_floor_prev"])
    if _TR35:
        sat = xp.where(xp.max(xp.abs(s["tau_ratio"]), axis=-1) > G["torque_sat_frac"], 1.0, 0.0)
    else:
        lim = G["torque_sat_frac"] * G["torque_limit"]
        sat = xp.where(xp.max(xp.abs(s["torque"]), axis=-1) > lim, 1.0, 0.0)
    return ({"success_t": t, "succeeded": xp.maximum(ev["succeeded"], held)},
            {"getup_success": first, "head_impacts": onset,
             "head_impact_v2": onset * head_impact_v2(xp, s), "torque_sat_steps": sat})


# --- aksiyon ------------------------------------------------------------------
def head_fix_arrays(xp, action_joints):
    """getup_head="fixed": (mask (A,) 1 = HEAD_JOINTS aksiyon boyutlari, tuck (A,) radians(HEAD_TUCK_DEG) bas
    boyutlarinda, digerleri 0)."""
    if C.HEAD_TUCK_DEG is None:
        raise ValueError("getup_head='fixed' icin C.HEAD_TUCK_DEG gerekli")
    aj = list(action_joints)
    mask = np.array([1.0 if j in C.HEAD_JOINTS else 0.0 for j in aj])
    tuck = np.array([np.deg2rad(C.HEAD_TUCK_DEG[j]) if j in C.HEAD_JOINTS else 0.0 for j in aj])
    return xp.asarray(mask), xp.asarray(tuck)


def fixed_head(xp, targets, action, mask, tuck):
    """Kirpmadan sonra: bas aksiyonlari 0, bas hedefleri tuck (mutlak; ctrlrange kirpmasi ortamda)."""
    on = mask > 0.5
    return xp.where(on, tuck, targets), xp.where(on, 0.0, action)


def action_to_targets(xp, action, default_action_q, scale=None, clip=None, lo=None, hi=None):
    """Politika aksiyonu (N,A) -> hedef acilar (N,A). Kirpma burada yapilir.
    target = default_action_q + scale * clip(action, +-clip); lo/hi verilirse hedef [lo, hi]'ye kirpilir.
    walk: varsayilanlar (ACTION_SCALE, ACTION_CLIP), default_action_q = HOME bacaklari (10,).
    main: default_action_q = posture_ref(...)["q_ref"] (N,10), scale/clip walk ile ayni.
    getup: default_action_q = HOME (14,), scale = tasks.action_scale_vector (14,), clip = 1.0,
           lo/hi = RobotIndex.action_lo/hi. Dondurur (targets, kirpilmis aksiyon)."""
    c = C.ACTION_CLIP if clip is None else clip
    a = xp.clip(action, -c, c)
    t = default_action_q + (C.ACTION_SCALE if scale is None else scale) * a
    if lo is not None:
        t = xp.clip(t, lo, hi)
    return t, a
