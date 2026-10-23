# -*- coding: utf-8 -*-
"""
OpenDuck yurume (joystick) gorevinin ve yeni main/getup gorevlerinin (dosyanin sonu) TEK dogruluk kaynagi.

CPU (MuJoCo C + PyTorch PPO) ve GPU (MJX + Brax PPO) arka uclari ayni sabitleri
ve rl/task_math.py'deki ayni gozlem/odul fonksiyonlarini kullanir.
Ayrintili formuller: rl/TASK.md; gorev tanimlari: rl/tasks.py

Profil (OPENDUCK_ROBOT, surec basina): "xl330_27" (varsayilan) asagidaki degerler aynen. "tr35_b2": dosyanin sonunda
geometriye bagli her sabit turetilmis dosyadan (DERIVED_PATH, sema 1; load_derived), digerleri tr35_constants()
formullerinden (Froude S = 1.282, hiz azaltma KAPPA). Hicbir tr35 geometri sayisi burada elle yazilmaz.
"""
import copy
import json
import math
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_PATH = os.path.join(ROOT, "build", "sim", "scene.xml")

# --- Robot profili (OpenDuck-TR spec 3.2.3-3.2.4; rl/actuators.py, rl/sensors.py) ------------------
# "xl330_27": 27 cm XL330 robotu, MJCF aktuatorleri aynen (walk bit-duzeyinde ayni; varsayilan).
# "tr35_b2" : PWM servo aktuator modeli + sensor modeli, yuklenen MJCF'ye eklem -> servo sinifi haritasiyla.
# Ortam degiskeni OPENDUCK_ROBOT varsayilani secer; build_spec/VecEnv/OpenDuckEnv robot= ile ornek basina.
ROBOT_PROFILES = ("xl330_27", "tr35_b2")
ROBOT_PROFILE = os.environ.get("OPENDUCK_ROBOT", "xl330_27")
if ROBOT_PROFILE not in ROBOT_PROFILES:
    raise ValueError("OPENDUCK_ROBOT=%r (gecerli: %s)" % (ROBOT_PROFILE, ", ".join(ROBOT_PROFILES)))
JOINT_RANGE_MARGIN_DEG = 3.0       # tr35: aktuatorlu eklem range = ctrlrange +- bu (varyant sinirlarindan SONRA)
# Sahne ve turetilmis dosya (profil basina). OPENDUCK_SCENE / OPENDUCK_DERIVED ezer; build_spec SCENE_PATH'i cagri
# aninda okur (rl.config.SCENE_PATH atamasi da calisir).
SCENE_PATHS = {"xl330_27": os.path.join(ROOT, "build", "sim", "scene.xml"),
               "tr35_b2": os.path.join(ROOT, "build", "tr35_b2", "sim", "scene.xml")}
SCENE_PATH = os.environ.get("OPENDUCK_SCENE") or SCENE_PATHS[ROBOT_PROFILE]
DERIVED_PATHS = {"tr35_b2": os.path.join(ROOT, "rl", "derived", "tr35_b2.json")}
DERIVED_PATH = os.environ.get("OPENDUCK_DERIVED") or DERIVED_PATHS.get(ROBOT_PROFILE)
DERIVED = None                     # tr35: load_derived(DERIVED_PATH) (dosyanin sonu)
DERIVED_SOURCE = None              # tr35: DERIVED["source"] (sahne, MJCF/konfig sha256, stub)

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


# =====================================================================================
# OpenDuck-TR (P5a) ek adlari. Asagidaki degerler xl330_27 degerleridir (davranis degismez); tr35_b2 surecinde
# dosyanin sonunda tr35_constants() ile degisirler. Mevcut adlar/anahtarlar hicbir profilde silinmez.
# =====================================================================================
FROUDE_S = None                    # tr35: 1.282 (boy orani; Froude olcegi, spec 3.2.5)
KAPPA = None                       # tr35: hiz azaltma (varsayilan 0.5; OPENDUCK_KAPPA)
SEAT_PITCH_DEG = 0.0               # oturus govde pitch'i (+ = burun asagi; 0 = duz oturus)
HEAD_TUCK_DEG = None               # getup_head="fixed" bas pozu (mutlak derece; tr35 turetilmis)
HOME_OVERRIDE = None               # tr35: yeniden optimize HOME (DERIVED["home"], source "family"); keyframe'i ezer
SELF_COLLISION_PAIRS = ()          # tr35 main/getup: oz-carpisma geom ciftleri (spec 3.2.2)
GETUP_HEAD_MODES = ("free", "fixed")
GETUP_HEAD_DEFAULT = "free"        # getup_head=None: bas politikada; "fixed": HEAD_TUCK_DEG, bas aksiyonlari 0
GETUP_HEAD_SCALE = None            # tr35: getup bas aksiyon olcegi ust siniri (rad; head_pitch/yaw/roll)
MOVING_EPS = 0.05                  # walk: moving = |komut| > bu
RAIL_SOFT_A = 15.0                 # A, LEG ray talebi cezasinin yumusak esigi (A4: > 15 A suresi <= %2)
MAIN["stand_gz"] = -0.9            # kalkis/ayakta dikligi (MAIN["seat_gz"] oturus kosulundan ayri)
# POSE_BANK boyutlari: OPENDUCK_BANK_N=<int> hepsini ezer (duman kosulari, testler); yoksa degismez
_bank_n = os.environ.get("OPENDUCK_BANK_N")
if _bank_n:
    POSE_BANK.update({k + "_n": int(_bank_n) for k in ("seated", "fallen", "midfall")})

# xl330 degerlerinin kopyasi: tr35_constants() hep bundan uretir (saf; tr35 surecinde de tekrar cagrilabilir)
_xl330 = copy.deepcopy({k: v for k, v in globals().items() if k.isupper()})

# --- turetilmis dosya (sema 1; tools/derive_posture.py yazar) ------------------------------------
_schema = 1
_regen = ("python tools/derive_posture.py derive --scene build/tr35_b2/sim/scene.xml --out rl/derived/tr35_b2.json "
          "(MJCF'den yeniden uretir) ya da OPENDUCK_DERIVED=<json>")
_leg5 = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")
_heights = ("H_HOME", "H_MIN_WALK", "H_MIN_STAND", "H_DEEP", "Z_SIT", "SEAT_Z", "SEAT_Z_TOL",
            "TERMINATION_MIN_HEIGHT", "standup_z", "getup_success_z")
_dkeys = {
    "source": ("scene", "mjcf_sha256", "config_sha256", "stub"),
    "mass": ("total_kg",),
    "home": ("source", "qpos", "ctrl", "legs_deg", "head_deg", "trunk_z_kin", "com_offset_x", "com_margin"),
    "heights": _heights + ("getup_pose_z", "getup_arrival_z"),
    "seat": ("deg", "pitch_deg", "seat_gz", "feasible"),
    "head": ("home_deg", "sit_deg", "tuck_deg"),
    "feet": ("home_lateral", "FEET_MIN_DISTANCE"),
    "caps_deg": ("KNEE_LIMIT_DEG", "NECK_PITCH_MAX_DEG", "HIP_YAW_INWARD_MAX_DEG"),
}


def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def load_derived(path):
    """Turetilmis JSON'u (sema 1, rl/TASK.md "Derived file") okur ve dogrular; dict dondurur. Bozuksa ValueError
    (alan adiyla). Yalniz json (mujoco yok)."""
    with open(path) as f:
        d = json.load(f)

    def need(ok, msg, *a):
        if not ok:
            raise ValueError("%s: %s" % (path, msg % a))

    def nums(x, n, what):
        need(isinstance(x, (list, tuple)) and len(x) == n and all(_num(v) for v in x), "%s: %d sayi olmali", what, n)

    def keyed(x, keys, what):
        need(isinstance(x, dict) and set(x) == set(keys) and all(_num(v) for v in x.values()),
             "%s anahtarlari %s (sayi) olmali", what, sorted(keys))

    need(isinstance(d, dict), "JSON nesnesi degil")
    need(d.get("schema") == _schema, "schema %r (beklenen %d)", d.get("schema"), _schema)
    need(d.get("profile") == "tr35_b2", "profile %r (beklenen tr35_b2)", d.get("profile"))
    top = ("tool", "tool_version", "params", "crouch_family", "extra_collision", "self_collision_pairs") + tuple(_dkeys)
    need(not [k for k in top if k not in d], "eksik anahtarlar %s", [k for k in top if k not in d])
    need(isinstance(d["tool"], str) and isinstance(d["tool_version"], int), "tool (str) / tool_version (int)")
    need(isinstance(d["params"], dict), "params dict olmali")
    for sec, keys in _dkeys.items():
        miss = [k for k in keys if not isinstance(d[sec], dict) or k not in d[sec]]
        need(not miss, "%s: eksik %s", sec, miss)
    src = d["source"]
    need(isinstance(src["scene"], str) and isinstance(src["mjcf_sha256"], str) and isinstance(src["stub"], bool)
         and (src["config_sha256"] is None or isinstance(src["config_sha256"], str)), "source alan tipleri")
    need(_num(d["mass"]["total_kg"]) and d["mass"]["total_kg"] > 0, "mass.total_kg > 0 olmali")
    h = d["home"]
    nj = len(LEG_JOINTS) + len(HEAD_JOINTS) + len(OTHER_JOINTS)
    need(h["source"] in ("keyframe", "family"), "home.source %r", h["source"])
    nums(h["qpos"], 7 + nj, "home.qpos")
    nums(h["ctrl"], nj, "home.ctrl")
    keyed(h["legs_deg"], _leg5, "home.legs_deg")
    keyed(h["head_deg"], HEAD_JOINTS, "home.head_deg")
    for k in ("trunk_z_kin", "com_offset_x", "com_margin"):
        need(_num(h[k]), "home.%s sayi olmali", k)
    hz = d["heights"]
    for k in _heights:
        need(_num(hz[k]) and hz[k] > 0, "heights.%s > 0 olmali", k)
    for k in ("getup_pose_z", "getup_arrival_z"):
        nums(hz[k], 2, "heights." + k)
        need(0 < hz[k][0] < hz[k][1], "heights.%s: 0 < z0 < z1 olmali", k)
    # derive_posture.validate ile ayni sira: Z_SIT < H_DEEP yalniz uyari (stub'da oturus derin comelmeden 0.5 mm
    # yuksek; z_ref dogrusal ara deger oldugu icin calisir), SEAT_Z < Z_SIT zorunlu
    need(hz["H_HOME"] > hz["H_MIN_WALK"] >= hz["H_MIN_STAND"] >= hz["H_DEEP"] and hz["SEAT_Z"] < hz["Z_SIT"],
         "H_HOME > H_MIN_WALK >= H_MIN_STAND >= H_DEEP ve SEAT_Z < Z_SIT olmali (%s)",
         [hz[k] for k in ("H_HOME", "H_MIN_WALK", "H_MIN_STAND", "H_DEEP", "Z_SIT", "SEAT_Z")])
    fam = d["crouch_family"]
    need(isinstance(fam, list) and len(fam) >= 3, "crouch_family en az 3 satir olmali")
    for i, r in enumerate(fam):
        nums(r, 5, "crouch_family[%d]" % i)
    hs = [r[0] for r in fam]
    need(all(a > b for a, b in zip(hs, hs[1:])), "crouch_family h kesin azalan olmali")
    need(hs[0] >= hz["H_HOME"] and hs[-1] <= hz["H_DEEP"], "crouch_family: ilk h >= H_HOME, son h <= H_DEEP olmali")
    lg = h["legs_deg"]
    flat = lg["hip_pitch"] - lg["knee"] + lg["ankle"]                      # HOME (pitch 0) duz taban degismezi
    bad = [i for i, r in enumerate(fam) if abs(r[1] + r[2] - r[3] + r[4] - flat) > 1e-6]
    need(not bad, "crouch_family satirlari %s duz taban degil (pitch + hp - kn + an != HOME %.6g)", bad, flat)
    s = d["seat"]
    keyed(s["deg"], CROUCH_JOINTS, "seat.deg")
    need(_num(s["pitch_deg"]) and _num(s["seat_gz"]) and -1.0 <= s["seat_gz"] < 0.0 and isinstance(s["feasible"], bool),
         "seat.pitch_deg / seat_gz (-1..0) / feasible (bool)")
    for k in _dkeys["head"]:
        keyed(d["head"][k], HEAD_JOINTS, "head." + k)
    for sec in ("feet", "caps_deg"):
        for k in _dkeys[sec]:
            need(_num(d[sec][k]) and d[sec][k] > 0, "%s.%s > 0 olmali", sec, k)
    ec = d["extra_collision"]
    need(isinstance(ec, dict), "extra_collision dict olmali")
    for b, g in ec.items():
        need(isinstance(g, (list, tuple)) and len(g) == 4 and g[0] in ("box", "capsule"), "extra_collision.%s bicimi", b)
        nums(g[1], 3, "extra_collision.%s merkez" % b)
        nums(g[2], 3 if g[0] == "box" else 2, "extra_collision.%s boyut" % b)
        need(all(v > 0 for v in g[2]) and (g[3] is None if g[0] == "box" else g[3] in ("x", "y", "z")),
             "extra_collision.%s boyut > 0 / eksen", b)
    pr = d["self_collision_pairs"]
    need(isinstance(pr, list) and all(isinstance(p, (list, tuple)) and len(p) == 2 and all(isinstance(n, str) for n in p)
                                      and p[0] != p[1] for p in pr), "self_collision_pairs [[geomA, geomB], ...] olmali")
    return d


def tr35_constants(derived, kappa=None):
    """tr35_b2 sabitleri {AD: deger}: geometri turetilmis dosyadan (load_derived ciktisi), digerleri formullerle
    (S = 1.282, k = kappa | OPENDUCK_KAPPA | 0.5; yurume lv = sqrt(S) k, lw = k / sqrt(S), lt = sqrt(S) / k; main
    otur/kalk lv = lt = sqrt(S); getup lv = sqrt(S), lw = 1 / sqrt(S)). Yeni nesneler dondurur, xl330 sozluklerini
    degistirmez (xl330 surecinden de cagrilabilir). Sayisal sabitler: Froude S, KAPPA, 2.21 Hz, 2.88 N.m (spec)."""
    from . import actuators as A          # servo siniflari (slew siniri); dongusel import modulun sonunda guvenli
    x = copy.deepcopy(_xl330)
    d = derived
    S = 1.282
    k = float(os.environ.get("OPENDUCK_KAPPA", "0.5") if kappa is None else kappa)
    sq = math.sqrt(S)
    lv, lw, lt = sq * k, k / sq, sq / k                  # yurume
    lv_m = sq                                            # main otur/kalk (kappa yok; lt_m = sqrt(S))
    lv_g, lw_g = sq, 1.0 / sq                            # getup
    hz, fl = d["heights"], (lambda v: float(v))
    o = {"FROUDE_S": S, "KAPPA": k}

    # --- geometri (turetilmis dosya) -----------------------------------------------------------------
    for n in ("H_HOME", "H_MIN_WALK", "H_MIN_STAND", "H_DEEP", "Z_SIT", "SEAT_Z", "SEAT_Z_TOL", "TERMINATION_MIN_HEIGHT"):
        o[n] = fl(hz[n])
    o["BASE_HEIGHT_TARGET"] = o["H_HOME"]
    o["H_NORM"] = o["H_HOME"] - o["H_MIN_STAND"]
    o["CROUCH_FAMILY"] = tuple(tuple(fl(v) for v in r) for r in d["crouch_family"])
    o["SEAT_DEG"] = {j: fl(d["seat"]["deg"][j]) for j in CROUCH_JOINTS}
    o["SEAT_PITCH_DEG"] = fl(d["seat"]["pitch_deg"])
    for n, key in (("HEAD_HOME_DEG", "home_deg"), ("HEAD_SIT_DEG", "sit_deg"), ("HEAD_TUCK_DEG", "tuck_deg")):
        o[n] = {j: fl(d["head"][key][j]) for j in HEAD_JOINTS}
    o["FEET_MIN_DISTANCE"] = fl(d["feet"]["FEET_MIN_DISTANCE"])
    for n in ("KNEE_LIMIT_DEG", "NECK_PITCH_MAX_DEG", "HIP_YAW_INWARD_MAX_DEG"):
        o[n] = fl(d["caps_deg"][n])
    o["EXTRA_COLLISION"] = {b: (g[0], tuple(fl(v) for v in g[1]), tuple(fl(v) for v in g[2]), g[3])
                            for b, g in d["extra_collision"].items()}
    o["SELF_COLLISION_PAIRS"] = tuple(tuple(p) for p in d["self_collision_pairs"])
    o["HOME_OVERRIDE"] = copy.deepcopy(d["home"]) if d["home"]["source"] == "family" else None
    o["HEAD_GEOMS"] = ("head_collision", "jaw_collision", "neck_bumper")     # RobotIndex olmayanlari atar

    # --- yurume (Froude + kappa) ---------------------------------------------------------------------
    o["GAIT_FREQ"] = 2.21 * k
    o["AIR_TIME_TARGET"] = 0.375 / o["GAIT_FREQ"]                       # xl330: 0.15 s @ 2.5 Hz
    o["SWING_HEIGHT"] = x["SWING_HEIGHT"] * S
    o["FOOT_HEIGHT_SIGMA"] = x["FOOT_HEIGHT_SIGMA"] * S ** 2
    cr = x["COMMAND_RANGES"]
    o["COMMAND_RANGES"] = {"vx": tuple(v * lv for v in cr["vx"]), "vy": tuple(v * lv for v in cr["vy"]),
                           "wz": tuple(v * lw for v in cr["wz"])}
    o["MOVING_EPS"] = x["MOVING_EPS"] * lv
    o["TRACKING_SIGMA_LIN"] = x["TRACKING_SIGMA_LIN"] * lv ** 2
    o["TRACKING_SIGMA_ANG"] = x["TRACKING_SIGMA_ANG"] * lw ** 2
    o["ACTION_SCALE"] = 0.3                                             # rl-map D1 (sert P dongusu)

    def effort(w):
        """"torques" yerinde "torque_ratio" (sum (tau_net/tau_s)^2) + sona limit_torque, rail_demand. Agirliklar
        baslangic degerleri; P7'de ayarlanir."""
        out = {("torque_ratio" if n == "torques" else n): (-2e-3 if n == "torques" else v) for n, v in w.items()}
        out.update(limit_torque=-0.5, rail_demand=-0.2)
        return out
    w = x["REWARD_WEIGHTS"]
    w.update(lin_vel_z=w["lin_vel_z"] / lv ** 2, ang_vel_xy=w["ang_vel_xy"] / lw ** 2,
             base_height=w["base_height"] / S ** 2, joint_acc=w["joint_acc"] * lt ** 4,
             feet_slip=w["feet_slip"] / lv ** 2, feet_distance=w["feet_distance"] / S)
    o["REWARD_WEIGHTS"] = effort(w)

    # --- main ---------------------------------------------------------------------------------------
    wm = {n: (w[n] if n in w and n != "torques" else v) for n, v in x["MAIN_REWARD_WEIGHTS"].items()}
    wm.update(height_l1=wm["height_l1"] / S, vz_cap=wm["vz_cap"] / lv_m)
    o["MAIN_REWARD_WEIGHTS"] = effort(wm)
    M = x["MAIN"]
    M.update(moving_eps=o["MOVING_EPS"], height_sigma=M["height_sigma"] * S, vz_cap=M["vz_cap"] * lv_m,
             standup_z=fl(hz["standup_z"]), seat_gz=fl(d["seat"]["seat_gz"]), stand_gz=-0.9,
             standup_window_s=round(x["MAIN"]["standup_window_s"] * sq, 1),
             sitdown_window_s=round(x["MAIN"]["sitdown_window_s"] * sq, 1))
    o["MAIN"] = M
    o["SIT_RAMP_S"] = x["SIT_RAMP_S"] * sq
    o["STAND_RAMP_S"] = x["STAND_RAMP_S"] * sq

    # --- getup --------------------------------------------------------------------------------------
    o["GETUP_EPISODE_S"] = 12.0
    G = x["GETUP"]
    G.update(height_sigma=tuple(v * S for v in G["height_sigma"]),
             composite_sigma=(G["composite_sigma"][0] * S,) + tuple(G["composite_sigma"][1:]),
             torque_limit=2.88,      # KULLANILMAZ (tr35 odulleri tau_ratio ile); eval/play import'ta okur
             pose_z=tuple(fl(v) for v in hz["getup_pose_z"]), arrival_z=tuple(fl(v) for v in hz["getup_arrival_z"]))
    G["success"].update(z=fl(hz["getup_success_z"]), gyro=G["success"]["gyro"] / sq, hold_s=G["success"]["hold_s"] * sq)
    G["midfall"].update(ang_vel=(G["midfall"]["ang_vel"][0], G["midfall"]["ang_vel"][1] / sq))
    o["GETUP"] = G
    wg, wl = x["GETUP_REWARD_WEIGHTS"], x["GETUP_REWARD_LATE"]
    wg.update(height_l1=wg["height_l1"] / S, height_progress=wg["height_progress"] / S,
              joint_vel=wg["joint_vel"] / lw_g ** 2, arrival_damping=wg["arrival_damping"] / lw_g ** 2)
    wg.update(limit_torque=-0.5, rail_demand=-0.2)                     # erken agirliklar; gec deger yok
    wl.update(head_impact=wl["head_impact"] / lv_g ** 2, joint_vel=wl["joint_vel"] / lw_g ** 2,
              arrival_damping=wl["arrival_damping"] / lw_g ** 2)
    o["GETUP_REWARD_WEIGHTS"], o["GETUP_REWARD_LATE"] = wg, wl

    # --- bas, devir, sifirlama, bankalar, DR ----------------------------------------------------------
    rng = {"neck_pitch": 0.15, "head_pitch": 0.2, "head_yaw": 0.5, "head_roll": 0.08}      # rad (spec 3.2.5, MG90S)
    hr = x["HEAD_RANDOM"]
    hr.update(filter_tau=0.5, range=rng,
              slew_max={j: 0.5 * A.CLASSES[A.JOINT_CLASS[A.TR35][j]]["omega0"] for j in HEAD_JOINTS})   # rad/s
    o["HEAD_RANDOM"] = hr
    o["GETUP_HEAD_SCALE"] = {j: rng[j] for j in ("head_pitch", "head_yaw", "head_roll")}
    sv = x["SUPERVISOR"]
    sv.update(fall_steps=6, ready_hold_s=0.6)
    o["SUPERVISOR"] = sv
    o["RESET_BASE_VEL_NOISE"] = x["RESET_BASE_VEL_NOISE"] * sq
    pb = x["POSE_BANK"]
    pb.update(settle_s=2.5, drop=tuple(v * S for v in pb["drop"]))
    o["POSE_BANK"] = pb
    dr = x["DOMAIN_RAND"]
    f = fl(d["mass"]["total_kg"]) / 0.6763                              # xl330 toplam kutlesi 0.6763 kg
    dr.update(trunk_added_mass=tuple(v * f for v in dr["trunk_added_mass"]), trunk_com_offset=0.010, push_vel=0.20)
    o["DOMAIN_RAND"] = dr                                               # actuators.DR_OVERRIDES bunun ustune
    return o


if ROBOT_PROFILE == "tr35_b2":
    if not DERIVED_PATH or not os.path.isfile(DERIVED_PATH):
        raise RuntimeError("tr35_b2 turetilmis dosyasi yok: %s. Uretmek icin: %s" % (DERIVED_PATH, _regen))
    try:
        DERIVED = load_derived(DERIVED_PATH)
    except (ValueError, OSError) as _e:
        raise RuntimeError("tr35_b2 turetilmis dosyasi gecersiz (%s). Yeniden uretin: %s" % (_e, _regen))
    DERIVED_SOURCE = DERIVED["source"]
    globals().update(tr35_constants(DERIVED))
    H_NORM = H_HOME - H_MIN_STAND
