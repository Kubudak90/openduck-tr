# -*- coding: utf-8 -*-
"""
OpenDuck yurume (joystick) gorevinin ve yeni main/getup gorevlerinin (dosyanin sonu) TEK dogruluk kaynagi.

CPU (MuJoCo C + PyTorch PPO) ve GPU (MJX + Brax PPO) arka uclari ayni sabitleri
ve rl/task_math.py'deki ayni gozlem/odul fonksiyonlarini kullanir.
Ayrintili formuller: rl/TASK.md; gorev tanimlari: rl/tasks.py
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_PATH = os.path.join(ROOT, "build", "sim", "scene.xml")

# --- Zamanlama -----------------------------------------------------------------
SIM_DT = 0.005               # fizik adimi (s)
CTRL_DT = 0.02               # politika adimi (s) -> 50 Hz
N_SUBSTEPS = int(round(CTRL_DT / SIM_DT))
EPISODE_LENGTH_S = 20.0
MAX_EPISODE_STEPS = int(round(EPISODE_LENGTH_S / CTRL_DT))

# --- Eklemler (MJCF aktuator sirasiyla ayni) ----------------------------------
LEG_JOINTS = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]
HEAD_JOINTS = ["neck_pitch", "head_pitch", "head_yaw", "head_roll"]
OTHER_JOINTS = ["mouth"]                        # gaga: politika surmez, HOME'da tutulur
OBS_JOINTS = LEG_JOINTS + HEAD_JOINTS          # gozlemdeki 14 eklem
ACTION_JOINTS = LEG_JOINTS                      # politikanin surdugu 10 eklem
HIP_POSE_JOINTS = ["left_hip_yaw", "left_hip_roll", "right_hip_yaw", "right_hip_roll"]
NUM_ACTIONS = len(ACTION_JOINTS)

FOOT_GEOMS = ["left_foot_collision", "right_foot_collision"]
FOOT_SITES = ["left_foot_sole", "right_foot_sole"]
FLOOR_GEOM = "floor"
IMU_SITE = "imu"
TRUNK_BODY = "trunk"
DEFAULT_KEYFRAME = "stand"

# --- Aksiyon -------------------------------------------------------------------
ACTION_SCALE = 0.5           # rad; hedef = varsayilan + ACTION_SCALE * clip(a, -ACTION_CLIP, ACTION_CLIP)
ACTION_CLIP = 1.5

# --- Gozlem olcekleri ve gurultu (egitimde, uniform +-) -----------------------
OBS_SCALES = {"gyro": 0.25, "joint_vel": 0.05}
OBS_NOISE = {"gravity": 0.05, "gyro": 0.1, "joint_pos": 0.01, "joint_vel": 0.5}   # ham birimlerde
OBS_CLIP = 100.0
NUM_OBS = 3 + 3 + 14 + 14 + NUM_ACTIONS + 3 + 2            # 49
NUM_CRITIC_OBS = NUM_OBS + 3 + 1 + 2 + 2 + 1                # 58

# --- Komutlar ------------------------------------------------------------------
COMMAND_RANGES = {"vx": (-0.15, 0.30), "vy": (-0.15, 0.15), "wz": (-1.0, 1.0)}   # m/s, m/s, rad/s
COMMAND_ZERO_PROB = 0.15
COMMAND_RESAMPLE_S = 6.0

# --- Yuruyus fazi --------------------------------------------------------------
GAIT_FREQ = 2.5              # Hz; sol ayak faz phi, sag ayak phi + 0.5
SWING_HEIGHT = 0.02          # m, sallanma fazinda tepe ayak yuksekligi
FOOT_HEIGHT_SIGMA = 0.0002   # m^2
AIR_TIME_TARGET = 0.15       # s
CONTACT_EPS = 0.0            # ayak-zemin temasi: temas mesafesi < CONTACT_EPS

# --- Oduller (agirliklar saniye basina; toplam = sum(w * r) * CTRL_DT) ---------
TRACKING_SIGMA_LIN = 0.02    # (m/s)^2
TRACKING_SIGMA_ANG = 0.1     # (rad/s)^2
BASE_HEIGHT_TARGET = 0.115   # m (govde orijini; HOME'da 0.118)
FEET_MIN_DISTANCE = 0.06     # m, ayaklarin yanal (govde yonunde) en kucuk mesafesi (HOME'da 0.083)
REWARD_WEIGHTS = {
    "tracking_lin_vel": 2.0,
    "tracking_ang_vel": 2.0,
    "lin_vel_z": -2.0,
    "ang_vel_xy": -0.05,
    "orientation": -5.0,
    "base_height": -30.0,
    "torques": -1e-3,
    "joint_acc": -2.5e-7,
    "action_rate": -0.02,
    "feet_air_time": 1.0,
    "feet_phase": 1.0,
    "feet_slip": -0.5,
    "feet_distance": -100.0,
    "hip_pose": -0.2,
    "alive": 0.5,
}
TERMINATION_PENALTY = -10.0  # dusmede bir kez (CTRL_DT ile carpilmaz)

# --- Sonlandirma ---------------------------------------------------------------
TERMINATION_MIN_HEIGHT = 0.075    # govde z < bu -> dustu
TERMINATION_MAX_GRAVITY_Z = -0.6  # izdusumlu yercekimi z > bu (egim > ~53 derece) -> dustu

# --- Baslangic ve bas hareketi ------------------------------------------------
RESET_JOINT_NOISE = 0.05          # rad
RESET_BASE_VEL_NOISE = 0.05       # m/s
HEAD_RANDOM = {                   # egitimde bas eklemleri bu araliklarda rastgele hedeflere gider
    "prob": 0.5, "resample_s": (2.0, 4.0), "filter_tau": 0.3,
    "range": {"neck_pitch": 0.25, "head_pitch": 0.3, "head_yaw": 0.6, "head_roll": 0.1},
}

# --- Alan rastgelelestirme (her bolum basinda) ---------------------------------
DOMAIN_RAND = {
    "enabled": True,
    "friction": (0.6, 1.3),           # ayak surtunmesi
    "link_mass_scale": (0.95, 1.05),
    "trunk_added_mass": (-0.03, 0.05), # kg
    "trunk_com_offset": 0.005,         # m, +- her eksende
    "kp_scale": (0.85, 1.15),
    "damping_scale": (0.8, 1.2),
    "frictionloss_scale": (0.5, 1.5),
    "armature_scale": (0.8, 1.2),
    "push_interval_s": (3.0, 6.0),
    "push_vel": 0.25,                  # m/s, govde xy hizina eklenen
    "action_delay_steps": (0, 0),      # (min, max) kontrol adimi gecikmesi
}

# --- PPO (CPU / PyTorch) ---------------------------------------------------------
PPO = {
    "num_envs": 512,
    "num_steps_per_env": 24,
    "learning_rate": 1e-3,
    "schedule": "adaptive",
    "desired_kl": 0.01,
    "num_learning_epochs": 5,
    "num_mini_batches": 4,
    "clip_param": 0.2,
    "gamma": 0.99,
    "lam": 0.95,
    "value_loss_coef": 1.0,
    "entropy_coef": 0.005,
    "max_grad_norm": 1.0,
    "init_noise_std": 1.0,
    "actor_hidden": [256, 128, 64],
    "critic_hidden": [512, 256, 128],
    "activation": "elu",
    "obs_normalization": True,
}


# =====================================================================================
# Yeni gorevler: "main" (yurume + govde yuksekligi + otur/kalk) ve "getup" (dusunce kalkma).
# Spec: docs/workflow/sit-stand-getup/spec.md; gorev tanimlari rl/tasks.py; sozlesme rl/TASK.md.
# Yukaridaki "walk" sabitlerinin hicbiri degismez.
#
# ISARET KURALI (bacak eklemleri): tablolardaki degerler SOL bacagin MUTLAK eklem acilaridir (derece);
# sag bacak aynalidir: q_sag = -q_sol (5 bacak ekleminin hepsi icin). Dayanak (build/sim/openduck.xml):
# hip_pitch/knee/ankle eksenleri sol/sagda zit ((0,1,0)/(0,-1,0), (0,-1,0)/(0,1,0), (0,1,0)/(0,-1,0));
# hip_roll (1,0,0) ve hip_yaw (0,0,-1) iki yanda ayni eksen -> aynali hareket zit isaret. HOME
# keyframe'i de boyle: sol (yaw, roll, pitch, knee, ankle) = (0, -5.00, -26.24, -0.28, +25.95) derece,
# sag = (0, +5.00, +26.24, +0.28, -25.95). Sol bacakta + hip_pitch tabani geriye, + knee bukulme
# (taban one/yukari) yonudur; govde pitch'i + = burun asagi. Ayak tabani duz: pitch + hp - kn + an = 0.
# =====================================================================================

# --- Govde yuksekligi komutu ve durus filtresi (main) ---------------------------------
H_HOME = 0.115               # m, ayakta referans govde yuksekligi (= BASE_HEIGHT_TARGET)
H_MIN_WALK = 0.085           # m, hiz komutu yalnizca h_cmd >= H_MIN_WALK iken (derin comelmede yurume yok)
H_MIN_STAND = 0.070          # m, ayakta en alcak komut
H_DEEP = 0.065               # m, oturmadan once inilen derin comelme
Z_SIT = 0.044                # m, oturusta govde orijini (govde kutusu yerde)
H_RATE = 0.02                # m/s, h_ref'in h_cmd'ye en buyuk yaklasma hizi
SIT_RAMP_S = 2.5             # s, alpha 0 -> 1 (otur)
STAND_RAMP_S = 2.5           # s, alpha 1 -> 0 (kalk). 2.0'da referans rampa sonunda geriye devriliyordu (sifir
                             # politika DR'de %42 -> %88); 0.008/adim esikleri (0.01, 0.3) float kafesinde tutturmaz
SIT_SPLIT = 0.6              # alpha <= SIT_SPLIT: derin comelme (s1), sonra oturus pozu (s2)
H_NORM = H_HOME - H_MIN_STAND  # gozlem: h_n = (h - H_HOME) / H_NORM  in [-1, 0]
ALPHA_STAND_MAX = 0.01       # alpha < bu iken: rastgele bas hedefleri ve itmeler acik

# Comelme ailesi F(h) (phys-sit family3.py; COM ayak tabani ustunde, ayaklar duz, diz <= 60).
# (h [m], pitch_ref [deg, + = burun asagi], hip_pitch, knee, ankle [deg, SOL, mutlak]); h azalan.
# MuJoCo ile dogrulandi (rl/tests/test_task_math.py): kinematik yukseklik h'nin 1 mm icinde, COM payi
# >= 26 mm. Kullanim ofset formunda: F_home(h) = HOME + (F(h) - F(H_HOME)), boylece
# q_ref(H_HOME, 0) = HOME tam olarak; F(H_HOME) tablo satirlarinin dogrusal ara degeridir.
# Not: ofset formunun kinematik yuksekligi h'den 3.3 (H_HOME) .. 5.7 mm (H_DEEP) fazla (HOME'un
# kendisi 0.1183 m); politika artigi bunu kapatir.
CROUCH_FAMILY = (
    (0.120, 0.0, -25.5, -10.5, 15.0),
    (0.110, 0.0, -10.5, 16.5, 27.0),
    (0.100, 0.0, 1.5, 36.0, 34.5),
    (0.090, 0.0, 13.5, 52.5, 39.0),
    (0.085, 0.0, 19.5, 60.0, 40.5),
    (0.080, -7.5, 40.5, 60.0, 27.0),
    (0.075, -15.0, 57.0, 60.0, 18.0),
    (0.070, -17.5, 69.0, 60.0, 8.5),
    (0.065, -22.5, 81.0, 58.5, 0.0),
)
CROUCH_JOINTS = ("hip_pitch", "knee", "ankle")     # tablonun eklem sutunlari; hip_yaw/hip_roll HOME'da
# Oturus pozu (tucked flat sit): SOL mutlak derece; hip_yaw/hip_roll HOME'da. Govde z 0.042 +- 0.003,
# pitch 0 +- 5 derece (kafa geride, HEAD_SIT, iken COM ayaklarin 6 mm icinde).
SEAT_DEG = {"hip_pitch": 87.5, "knee": 47.5, "ankle": -40.0}
SEAT_Z = 0.042
SEAT_Z_TOL = 0.003
SEAT_PITCH_TOL_DEG = 5.0
# Oturusta bas (politika surmez, ortam scripti): mutlak derece. HOME: neck_pitch 20, head_pitch 20.
HEAD_HOME_DEG = {"neck_pitch": 20.0, "head_pitch": 20.0, "head_yaw": 0.0, "head_roll": 0.0}
# neck_pitch 38.3+ derecede neck_splice gorsel mesh'i torso_shell'lere girer (A8); boyun hedefi NECK_PITCH_MAX_DEG'e
# esit (kp 1.1 ile kafa ~5 derece sarkar, boyun sinirda durur). Kalkis rampasinin sonundaki geriye devrilme kafa
# onde azalir: sifir politika, fizik DR, 1024x2 eslestirilmis bolum: head_pitch 0 -> %90, +10 -> %99.2 (S5, yeni CAD);
# +12.5'ten sonra one devrilme baslar. -10 (S4 ilk secim) rampa sonunda geriye devirir.
HEAD_SIT_DEG = {"neck_pitch": 36.0, "head_pitch": 10.0, "head_yaw": 0.0, "head_roll": 0.0}

# --- main gorevi ------------------------------------------------------------------------
MAIN_COMMAND_NAMES = ("vx", "vy", "wz", "h", "sit")   # durum/komut: h metre, sit 0/1
MAIN_SEGMENTS = ("walk", "stand", "crouch", "sit")     # segment kodu = bu listedeki indeks (float)
MAIN = {
    "segment_s": (3.5, 6.5),                           # segment suresi U(.)
    "segment_probs": (0.45, 0.10, 0.15, 0.30),         # MAIN_SEGMENTS sirasiyla; sit'ten sonra sit gelmez
    "walk_h_home_prob": 0.5,                           # walk: %50 h = H_HOME, %50 U(H_MIN_WALK, H_HOME)
    "reset_probs": {"home": 0.75, "seated": 0.25},
    "seated_first_sit_prob": 0.5,                      # oturustan baslayinca ilk segment: %50 sit, %50 stand
    "moving_eps": 0.05,                                # moving = |cmd[:3]| > bu
    "unplanned_sit_alpha": 0.3,                        # trunk_floor ve alpha < bu -> dustu
    "seat_gz": -0.9,                                   # seat odulu/kosulu: gz < bu
    "vz_cap": 0.08,                                    # m/s
    "height_sigma": 0.02,                              # m
    "standup_window_s": 3.0, "standup_z": 0.105,       # metrik: kalkis basarisi
    "sitdown_window_s": 3.5,                           # metrik: oturus basarisi
}
MAIN_EPISODE_S = 20.0
MAIN_REWARD_WEIGHTS = {
    "tracking_lin_vel": 2.0,
    "tracking_ang_vel": 2.0,
    "lin_vel_z": -2.0,
    "ang_vel_xy": -0.05,
    "orientation": -5.0,
    "torques": -1e-3,
    "joint_acc": -2.5e-7,
    "action_rate": -0.02,
    "feet_air_time": 1.0,
    "feet_phase": 1.0,
    "feet_slip": -0.5,
    "feet_distance": -100.0,
    "hip_pose": -0.2,
    "alive": 0.5,
    "height_track": 2.0,
    "height_l1": -10.0,
    "posture": -1.0,
    "stand_contact": 0.5,
    "seat": 1.0,
    "vz_cap": -20.0,
}

# --- getup gorevi -----------------------------------------------------------------------
GETUP_EPISODE_S = 8.0
GETUP = {
    "reset_probs": {"fallen": 0.70, "midfall": 0.10, "stand": 0.20},
    "height_sigma": (0.04, 0.015),                     # height_gauss / height_sharp
    "pose_sigma": 0.5, "pose_z": (0.06, 0.105),        # pose: smoothstep((z - z0) / (z1 - z0))
    "composite_sigma": (0.03, 0.4, 0.4),               # (dz, tilt, pose_rms)
    "torque_limit": 0.96, "torque_sat_frac": 0.9,      # N.m (MJCF forcerange)
    "arrival_z": (0.09, 0.11),                         # arrival_damping: smoothstep((z - z0) / (z1 - z0))
    "late_p": (0.5, 0.75),                             # gec terimler bu ilerleme araliginda lineer
    "success": {"gz": -0.9, "z": 0.10, "pose_rms": 0.3, "gyro": 0.5, "hold_s": 1.0},
    "midfall": {"tilt_deg": (15.0, 35.0), "ang_vel": (0.0, 4.0)},
}
GETUP_REWARD_WEIGHTS = {             # erken (p <= late_p[0]) agirliklar
    "upright": 1.5,
    "height_gauss": 1.0,
    "height_sharp": 1.0,
    "height_l1": -6.0,
    "upright_progress": 5.0,
    "height_progress": 50.0,
    "pose": 2.0,
    "composite": 3.0,
    "torque_sat": -1.0,
    "head_impact": 0.0,
    "action_rate": -0.01,
    "joint_vel": 0.0,
    "arrival_damping": 0.0,
}
GETUP_REWARD_LATE = {                # p >= late_p[1] agirliklar (arada lineer)
    "head_impact": -1.0,
    "action_rate": -0.1,
    "joint_vel": -1e-4,
    "arrival_damping": -0.05,
}

# --- Gorev degisimi (oynatma / robot calisma zamani) ---------------------------------
SUPERVISOR = {
    "fall_gz": -0.6, "fall_steps": 5,      # main -> getup: gz > fall_gz art arda fall_steps adim
    "ready_hold_s": 0.5,                   # getup -> main: basari kosulu (z haric) bu sure boyunca
    "handoff_zero_cmd_s": 1.0,             # devirden sonra sifir komut; last_action = 0, alpha = 0, h_ref = H_HOME
}

# --- Model varyantlari (rl/robot_model.py) -------------------------------------------
KNEE_LIMIT_DEG = 65.0                # main/getup: diz bukulme siniri (70'te uyluk-incik temasi)
# main/getup: neck_pitch ust siniri. A8 ilk hata 38.3 derece (neck_splice-torso_shell). Yumusak eklem siniri
# (solref 0.02) carpmada ~2.3 derece asar ve oturusta kafa sinira yaslanir: 37'de getup rastgele kontrolunde 48000
# karede 25 A8 hatasi, 36'da 0 (S5).
NECK_PITCH_MAX_DEG = 36.0
# main/getup: ice kalca donmesi siniri (sol alt -20 -> -8, sag ust +20 -> +8). Comelerek yurume (h 0.085..0.115,
# yuruyus sapmalari, roll +5'e kadar) 8 derecede 0/1200 poz, 10'da 135-150/1200 (uyluk ic plakasi govde kabuguna,
# 33 mm3'e kadar); 20'de ayakta yurume pozlarinin yarisinda incik braketi batarya yuvasina giriyor (458 mm3;
# walk-mjx-v2 sinira dayali yuruyor). Dis yon (30) degismez (S4/S5).
HIP_YAW_INWARD_MAX_DEG = 8.0
TRUNK_GEOM = "trunk_collision"
HEAD_GEOMS = ("head_collision", "jaw_collision")    # head_floor: bunlardan biri zemine degiyor (jaw yalniz getup)
HEAD_BODY = "head"
# getup modelinin zemin-yalniz ek carpisma geomlari (govde cercevesinde; sinif "collision").
# (tip, merkez, boyut, eksen): box -> boyut yari kenarlar; capsule -> (yaricap, yari uzunluk), eksen x/y/z.
# Baslangic: docs/workflow/sit-stand-getup/research/extra_boxes.json (gorsel mesh sinir kutulari). S5: yeni CAD'de
# alt gaga 3.7 mm yukari tasindi, jaw kutusu ayni yontemle yeniden oturtuldu (digerleri degismedi).
EXTRA_COLLISION = {
    "left_thigh": ("box", (-0.01815, 0.01425, -0.01075), (0.03065, 0.01525, 0.02375), None),
    "right_thigh": ("box", (-0.01815, -0.01425, -0.01075), (0.03065, 0.01525, 0.02375), None),
    "left_shin": ("box", (0.0005, 0.0100, -0.02075), (0.0125, 0.0130, 0.03075), None),
    "right_shin": ("box", (0.0005, -0.0100, -0.02075), (0.0125, 0.0130, 0.03075), None),
    "left_hip": ("box", (-0.0025, 0.0, -0.0160), (0.0150, 0.0120, 0.0210), None),
    "right_hip": ("box", (-0.0025, 0.0, -0.0160), (0.0150, 0.0120, 0.0210), None),
    "neck": ("box", (0.0, -0.0160, 0.0250), (0.0120, 0.0130, 0.0345), None),
    "jaw": ("box", (0.02635, -0.0280, -0.00695), (0.03285, 0.0494, 0.01345), None),
}

# --- Poz bankalari (rl/pose_bank.py) --------------------------------------------------
POSE_BANK = {
    "seed": 20261022,
    "seated_n": 1024, "fallen_n": 4096, "midfall_n": 1024,
    "settle_s": 2.0,                       # oturtma suresi (fizik, MuJoCo C); 1.5'te carpma ortasi ornekler batma
                                           # sinirini (3 mm) 11 tohumun birinde asiyordu (S5)
    "drop": (0.02, 0.10),                  # m, dusmus banka: en alcak carpisma noktasi zeminden bu kadar yukarida
    "fallen_joint_noise": 0.3,             # rad, HOME +- U (eklem baslangici = tutma hedefi)
    "inverted_gz": 0.85,                   # gz > bu ve yalniz kafa temasi -> atilir (kutu artefakti)
    "seated_joint_sigma": 0.05,            # rad, SEAT/HEAD_SIT etrafinda N(0, s) (baslangic = hedef)
    "seated_tilt_deg": 5.0,                # pitch/roll U(+-)
}
