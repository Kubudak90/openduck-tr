# -*- coding: utf-8 -*-
"""
Servo turu kayit defteri (spec 3.2.1): govde, kulak (tab), delik, mil ofseti, spline, horn, kablo cikisi, kutle,
hareket araligi, elektrik ve MJCF aktuator sinifi. Veri kismi FreeCAD gerektirmez (tests/cad/test_servos.py);
geometri fonksiyonlari (canon, horn_bolt_*) FreeCAD'i ilk cagrida yukler.

Kaynaklar: XL330 -> mevcut 27 cm model (cad/openduck.py, birebir); DS32XX, ES3054, MG90S, SG90 ->
docs/workflow/openduck-tr/research/servo-mechanics.md (uretici cizimi/STEP, A-F tablolari). "est" ile isaretli
degerler cizimden olculmus ya da tahmindir; Adim 0'da kumpasla dogrulanir (rec["estimated"]).

Servo yerel (kanonik) cercevesi: +Y mil (horn) yonu, +Z uzak uc (mil tarafindaki "yakin" uctan uzaga), X = Y x Z
(genislik). Olculer mm, kutle g, tork N.m, hiz rad/s, akim A.
  body.l : uzunluk (Z), body.w : genislik (X), body.h : mil yonunde govde yuksekligi (Y; tabandan govde/disli
           kulesi ustune, horn ve spline haric)
  shaft.near : mil ekseni - yakin uc yuzu; shaft.x : genislikte kayma (hepsi 0 -> mirror_y gecerli)
  *_h : govde tabanindan (alt yuz) mil yonunde yukseklik

Kanonik orijin (P2-0, docs/workflow/openduck-tr/p2-interface.md):
  XL330 : mil ekseninde, govde+horn yiginin (26 + 3 mm) ortasinda; horn dis yuzu Y = +14.5 (eski model, bayt-esit)
  PWM   : mil ekseninde, KULLANILAN horn'un dis yuzu duzleminde (braket datumu): horn_face_y = 0, govde tabani
          (arka yuz) Y = -horn.h[1]. DS32XX: aluminyum 25T kol (dis yuz 47.9) -> taban Y = -47.9.
Butun kanonik katilar X = 0 duzlemine gore simetriktir (kablo rakoru, delikler, horn kollari ortada): sag taraf
mirror_y ile gecerli bir (donmus) yerlesimdir. Horn kolu kanonik +Z (uzak uc) yonundedir; baska yone bakan kol
icin uretec horn'u mil ekseni etrafinda dondurur (geom.rotated).
"""
import math
from collections import OrderedDict

# Eklem anahtarlari (sol/sag ayni tur). Eklem adi = [left_|right_] + anahtar.
JOINT_KEYS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle",
              "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth")

# MJCF kok varsayilan sinifi (trunk childclass). XL330 aktuatorleri tarihsel olarak dogrudan bu siniftadir
# (27 cm build/sim/openduck.xml bayt-esit); diger turler bunun altinda kendi siniflarini alir.
ROOT_CLASS = "openduck"

# XL330 BAM uydurmasindan placeholder olcekleme (cad-map B): armature, frictionloss = XL330 x stall / 0.96.
_XL_ARM, _XL_FRIC, _XL_TAU = 0.0018, 0.0048, 0.96

SERVO_TYPES = OrderedDict()

# ---------------------------------------------------------------------------
# DYNAMIXEL XL330-M288-T (27 cm OpenDuck; degerler eski PARAMS/_servo_canon/ACTUATOR'dan birebir)
# ---------------------------------------------------------------------------
SERVO_TYPES["XL330"] = {
    "name": "XL330", "label": "DYNAMIXEL XL330-M288-T", "models": ("ROBOTIS DYNAMIXEL XL330-M288-T",),
    "interface": "ttl",                 # DYNAMIXEL TTL yolu (ID 10-14, 20-24, 30-34)
    "geometry": "xl330",                # canon() ureteci
    "body": {"l": 34.0, "w": 20.0, "h": 26.0, "corner_r": 3.5},     # Y kenarlari 3.5 yuvarlatilmis
    "shaft": {"near": 9.5, "x": 0.0},
    "boss": None,
    "spline": None,                     # horn fabrikada takili; braket horn'a 4 x M2 ile baglanir
    # horn diski: r6.5 x 3, merkez delik r1.2, 4 x r0.8 (M2) delik 4.2 yaricapta, 45 dereceden baslayip 90'ar
    "horn": {"kind": "disc_bolts", "r": 6.5, "t": 3.0, "center_hole_r": 1.2, "bolt_n": 4, "bolt_r": 4.2,
             "bolt_a0_deg": 45.0, "bolt_hole_r": 0.8, "bolt": "M2", "mass_g": 0.5},
    "tabs": None, "holes": None,        # kulak yok; govde vida delikleri modellenmedi
    # uzak uctaki konnektorler: tabandan 6..7 mm, uzak uctan 6 mm derin kanal (tum genislik)
    "cable": {"face": "far_end", "slot_h": (6.0, 7.0), "slot_depth": 6.0},
    "mass_g": 17.5,
    "travel_deg": 360.0, "pwm_us": None, "neutral_us": None, "deadband_us": None, "frame_hz": None,
    # 2S (6.6-8.2 V) ile dogrudan beslenir (Pollen ile ayni), etiket 3.7-6.0 V; 0.52 N.m @5 V, 0.60 @6 V
    "electrical": {"v_nom": 7.4, "v_min": 3.7, "v_max": 6.0, "stall_torque": 0.96, "omega0": 10.0,
                   "stall_current": None, "idle_current": None},
    # kp: firmware P=200'un ~0.87 rad hatada %100 PWM'e doymasindan; sinir tork 0.96 N.m ve sonumleme (geri EMK)
    # Pollen/BAM XL330 modelinden
    "actuator": {"class": "xl330", "mjcf_class": ROOT_CLASS, "kp": 1.1, "forcerange": 0.96, "damping": 0.053,
                 "frictionloss": 0.0048, "armature": 0.0018, "velocity": 10.0, "pband_deg": 49.8,
                 "deadband_deg": 0.0, "source": "BAM (Pollen microduck) uydurmasi, 2S"},
    "estimated": (),
}

# ---------------------------------------------------------------------------
# DSSERVO DS32xx (DS3218MG/DS3225MG/DS3230MG/DS3235 ayni 40 x 20 x 40.4 govde; STEP + cizim)
# ---------------------------------------------------------------------------
SERVO_TYPES["DS32XX"] = {
    "name": "DS32XX", "label": "DS32xx (DS3225MG / DS3230MG / DS3235)",
    "models": ("DS3225MG", "DS3230MG", "DS3235"),
    "interface": "pwm", "geometry": "pwm",
    "body": {"l": 40.0, "w": 20.0, "h": 40.4, "corner_r": 1.0, "bottom_protrusion": 0.3},
    "shaft": {"near": 10.0, "x": 0.0},                      # uzak uca 30.0; delik deseni merkezinden 10.0
    "boss": {"d": 11.8, "top_h": 41.5},
    "spline": {"teeth": 25, "od": 6.0, "top_h": 45.0, "screw": "M3", "screw_depth": 8.0},
    "tabs": {"span": 54.5, "w": 20.0, "t": 3.2, "under_h": 27.7, "top_h": 30.85},
    "holes": {"n": 4, "d": 4.5, "pitch_l": 49.5, "pitch_w": 10.0, "from_tab_end": 2.45, "open_slot": True},
    "case_screws": {"n": 4, "d": 2.2, "pitch_l": 35.0, "pitch_w": 15.0},    # taban duz, arka mil/cikinti yok
    "motor": {"d": 17.4, "axis_from_shaft": 20.0},
    # kablo: mil tarafi (yakin) uc yuzu, rakor merkezi tabandan 8.55; 6.8 G x 4.5 Y, 5.4 tasar; 3 x d1.5, 300 mm
    "cable": {"face": "near_end", "h": 8.55, "gland_w": 6.8, "gland_h": 4.5, "gland_out": 5.4, "wires": 3,
              "wire_d": 1.5, "lead": 300.0},
    # spec 3.1: aluminyum 25T kol linke gomulu (kutuyla gelir). Disk horn ayri alinir (TR stogu dogrulanmadi).
    # Kol: gobek d15 (h 42.2..47.9), tek kol +Z yonunde, dis yuzle ayni duzlemde arm_t kalinlik; uc yaricapi
    # arm_tip_r (en dis delik merkezinde); 2 x d2.6 delik r 20 / 24 (link'e M2.5 ile). Gobek deligi spline'i alir.
    "horn": {"kind": "al_arm_25t", "hub_d": 15.0, "h": (42.2, 47.9), "arm_t": 3.0, "arm_tip_r": 3.5,
             "hole_d": 2.6, "hole_r": (20.0, 24.0), "bolt_n": 2, "bolt": "M2.5", "clamp": "M2",
             "center_screw": "M3", "screw_d": 3.2, "mass_g": 5.0},
    "horn_alt": {"al_disc_25t": {"d": 20.0, "hub_d": 9.0, "h": (42.0, 46.5), "bolt_n": 4, "bolt_pcd": 14.0,
                                 "bolt": "M3"}},
    # arka idler (spec 3.1): kasada arka mil/cikinti yok; besik tabanin altinda F623ZZ tasir, U-link'in B kolu
    # M3 mil ile eksenel doner (servo-mechanics D2/D5). Duzlemler arka yuzden (govde tabani) disari dogru mm.
    "idler": {"bearing": "F623ZZ", "seat_gap": 0.4, "plate_t": 4.0, "run_gap": 0.5, "leg_t": 4.0, "head_t": 3.0,
              "head_d": 5.5, "boss_r": 5.0, "seat_wall": 2.0},
    "mass_g": 65.0,                                         # Direnc; DSSERVO 60, Probots 58 +- 2
    "travel_deg": 180.0, "pwm_us": (500, 2500), "neutral_us": 1500, "deadband_us": 3.0, "frame_hz": (50, 330),
    # spec 3.2.3 varsayilani (Adim 0'a kadar): DS3230MG Blue @6.5 V (DSSERVO 5/6.8 V tablosundan dogrusal)
    "electrical": {"v_nom": 6.5, "v_min": 4.8, "v_max": 6.8, "stall_torque": 2.88, "omega0": 5.72,
                   "stall_current": 3.8, "idle_current": 0.005},
    "variants": {                                           # @6.5 V: (stall N.m, omega0 rad/s, stall A @6.8 V)
        "DS3230MG_blue": (2.88, 5.72, 3.8), "DS3230MG_red": (3.30, 6.01, 2.9),
        "DS3230MG_direnc_text": (3.06, 8.36, None), "DS3225MG": (2.69, 7.32, 2.9),
    },
    # kp = stall / P-bandi (3 derece), damping = stall / omega0; armature/frictionloss XL330 x stall/0.96
    "actuator": {"class": "ds32xx", "mjcf_class": "ds32xx", "kp": 55.0, "forcerange": 2.88, "damping": 0.5035,
                 "frictionloss": 0.0144, "armature": 0.0054, "velocity": 5.72, "pband_deg": 3.0,
                 "deadband_deg": 0.27, "source": "spec 3.2.3 varsayilan; Adim 0 sysID ile guncellenir"},
    "estimated": ("body.corner_r", "horn.mass_g", "horn.arm_t", "horn.arm_tip_r"),
}

# ---------------------------------------------------------------------------
# EMAX ES3054 (EMAX olculu cizim; Robocombo ilani ayni)
# ---------------------------------------------------------------------------
SERVO_TYPES["ES3054"] = {
    "name": "ES3054", "label": "EMAX ES3054", "models": ("EMAX ES3054",),
    "interface": "pwm", "geometry": "pwm",
    "body": {"l": 28.45, "w": 13.0, "h": 26.7, "corner_r": 0.5},
    "shaft": {"near": 6.0, "x": 0.0},                       # 6.1 turetilmis, cizimden 5.7-6.0
    "boss": None,
    "spline": {"teeth": 23, "od": 5.9, "top_h": 31.1, "screw": "M3", "screw_depth": None},
    "tabs": {"span": 38.35, "w": 13.0, "t": 1.9, "under_h": 17.8, "top_h": 19.7},
    "holes": {"n": 2, "d": 4.1, "pitch_l": 35.0, "pitch_w": 0.0, "from_tab_end": 1.675, "open_slot": True},
    # yakin uc, alcak (fotograf); h olculecek. Rakor zarfi tahmin (kablo cikis kutusu)
    "cable": {"face": "near_end", "h": 8.0, "gland_w": 4.0, "gland_h": 3.0, "gland_out": 2.0, "lead": 240.0},
    # kutudan cikan plastik TEK kol (olculeri yayimli degil, tahmin; 3 horn'dan biri): spec 3.1 / P2-0 -> horn
    # baskili kaplin cebinde tutulur (kalca yaw pim/yarik kaplini: kol ucundaki pim mil ust ucundaki yariga girer),
    # merkez vidasi M3 + kol ucundan 1 vida. Tek kol: cift kolun on ucu boyun servosuna 0.1 mm yaklasiyordu (P2-0)
    "horn": {"kind": "plastic_arm1", "n_supplied": 3, "hub_d": 8.0, "h": (27.4, 32.4), "arm_t": 2.0,
             "arm_r": 13.0, "arm_tip_r": 2.5, "hole_d": 1.5, "hole_r": (13.0,), "bolt_n": 1, "bolt": "M1.7 kendinden",
             "screw": "M3", "screw_d": 3.2, "mass_g": 1.0},
    "idler": None,                                          # kalca yaw: doner tabla tasir (spec 3.1)
    "mass_g": 20.0,                                         # EMAX 17; olculen 20.1
    "travel_deg": 145.0, "pwm_us": (700, 2300), "neutral_us": 1500, "deadband_us": None, "frame_hz": (50, None),
    "electrical": {"v_nom": 6.0, "v_min": 4.8, "v_max": 6.0, "stall_torque": 0.343, "omega0": 8.06,
                   "stall_current": None, "idle_current": None},
    "actuator": {"class": "es3054", "mjcf_class": "es3054", "kp": 6.551, "forcerange": 0.343, "damping": 0.04256,
                 "frictionloss": 0.001715, "armature": 0.000643, "velocity": 8.06, "pband_deg": 3.0,
                 "deadband_deg": 0.5, "source": "EMAX @6.0 V; P-bandi/olu bant tahmini, Adim 0"},
    "estimated": ("body.corner_r", "shaft.near", "cable.h", "cable.gland_w", "horn.mass_g", "horn.hub_d", "horn.h",
                  "horn.arm_r", "horn.arm_t", "actuator.deadband_deg"),
}

# ---------------------------------------------------------------------------
# TowerPro MG90S (A-F tablosu; delik araligi ve mil ofseti DSSERVO MG90S cizimindan olculdu)
# ---------------------------------------------------------------------------
SERVO_TYPES["MG90S"] = {
    "name": "MG90S", "label": "TowerPro MG90S", "models": ("TowerPro MG90S",),
    "interface": "pwm", "geometry": "pwm",
    # D 12.4 (A-F) / 12.2 (olcu cizgisi). main_h: ana kasa ustu; ustunde mil cevresinde disli kulesi (tower: mil
    # merkezli w/2 yaricapli, +Z'de tower_l'ye uzanan stadyum) h'ye kadar
    "body": {"l": 22.8, "w": 12.2, "h": 28.4, "main_h": 22.5, "tower_l": 9.0, "corner_r": 0.5},
    "shaft": {"near": 5.9, "x": 0.0},
    "boss": None,
    "spline": {"teeth": 21, "od": 4.8, "top_h": 32.5, "screw": "M2 self-tapping", "screw_depth": None},
    "tabs": {"span": 32.1, "w": 12.2, "t": 2.4, "under_h": 18.5, "top_h": 20.9},
    "holes": {"n": 2, "d": 2.0, "pitch_l": 27.8, "pitch_w": 0.0, "from_tab_end": 2.15, "open_slot": True},
    "cable": {"face": "near_end", "h": 3.0, "gland_w": 3.6, "gland_h": 2.0, "gland_out": 1.5, "lead": 250.0},
    # kutudan cikan plastik cift kol (tahmin): baskili cepte tutulur + merkez vidasi + 2 kol vidasi (spec 3.1)
    "horn": {"kind": "plastic_arm2", "n_supplied": 3, "hub_d": 7.0, "h": (29.0, 33.6), "arm_t": 1.6,
             "arm_r": 14.0, "arm_tip_r": 2.2, "hole_d": 1.2, "hole_r": (14.0,), "bolt_n": 2, "bolt": "M1.7 kendinden",
             "screw": "M2 self-tapping", "screw_d": 2.2, "mass_g": 0.5},
    "idler": None,
    "mass_g": 13.4,
    "travel_deg": 180.0, "pwm_us": (500, 2400), "neutral_us": 1500, "deadband_us": 5.0, "frame_hz": (50, 50),
    "electrical": {"v_nom": 6.0, "v_min": 4.8, "v_max": 6.0, "stall_torque": 0.216, "omega0": 13.1,
                   "stall_current": 0.9, "idle_current": 0.25},
    "actuator": {"class": "mg90s", "mjcf_class": "mg90s", "kp": 4.125, "forcerange": 0.216, "damping": 0.01649,
                 "frictionloss": 0.00108, "armature": 0.000405, "velocity": 13.1, "pband_deg": 3.0,
                 "deadband_deg": 0.9, "source": "TowerPro @6.0 V; P-bandi tahmini, Adim 0"},
    "estimated": ("body.corner_r", "body.main_h", "body.tower_l", "shaft.near", "spline.teeth", "tabs.t",
                  "holes.pitch_l", "cable.h", "cable.gland_w", "horn.mass_g", "horn.hub_d", "horn.h", "horn.arm_r",
                  "horn.arm_t"),
}

# ---------------------------------------------------------------------------
# SG90 (TowerPro A-F; tab kalinligi, delik araligi ve mil ofseti klon cizimden)
# ---------------------------------------------------------------------------
SERVO_TYPES["SG90"] = {
    "name": "SG90", "label": "TowerPro SG90", "models": ("TowerPro SG90",),
    "interface": "pwm", "geometry": "pwm",
    # main_h: ana govde ustu; disli kulesi (tower) MG90S'teki gibi h'ye kadar
    "body": {"l": 22.7, "w": 12.2, "h": 27.0, "main_h": 22.1, "tower_l": 9.0, "corner_r": 0.5},
    "shaft": {"near": 5.9, "x": 0.0},
    "boss": {"d": None, "top_h": 26.5},
    "spline": {"teeth": 20, "od": 4.9, "top_h": 30.3, "screw": "M2 self-tapping", "screw_depth": None},
    "tabs": {"span": 32.3, "w": 12.2, "t": 2.4, "under_h": 17.0, "top_h": 19.4},
    "holes": {"n": 2, "d": 2.0, "pitch_l": 27.85, "pitch_w": 0.0, "from_tab_end": 2.225, "open_slot": True},
    "cable": {"face": "near_end", "h": 3.0, "gland_w": 3.6, "gland_h": 2.0, "gland_out": 1.5, "lead": 250.0},
    "horn": {"kind": "plastic_arm2", "n_supplied": 3, "hub_d": 7.0, "h": (27.5, 31.6), "arm_t": 1.5,
             "arm_r": 14.0, "arm_tip_r": 2.2, "hole_d": 1.2, "hole_r": (14.0,), "bolt_n": 2, "bolt": "M1.7 kendinden",
             "screw": "M2 self-tapping", "screw_d": 2.2, "mass_g": 0.5},
    "idler": None,
    "mass_g": 9.0,
    "travel_deg": 180.0, "pwm_us": (500, 2400), "neutral_us": 1500, "deadband_us": 5.0, "frame_hz": (50, 50),
    # tork/hiz yalniz 4.8 V'ta yayimli (6.0 V'ta calisir; Adim 0'da olculur)
    "electrical": {"v_nom": 6.0, "v_min": 4.8, "v_max": 6.0, "stall_torque": 0.177, "omega0": 10.5,
                   "stall_current": 0.75, "idle_current": 0.09},
    "actuator": {"class": "sg90", "mjcf_class": "sg90", "kp": 3.38, "forcerange": 0.177, "damping": 0.01686,
                 "frictionloss": 0.000885, "armature": 0.000332, "velocity": 10.5, "pband_deg": 3.0,
                 "deadband_deg": 0.9, "source": "TowerPro @4.8 V; P-bandi tahmini, Adim 0"},
    "estimated": ("body.corner_r", "body.tower_l", "tabs.under_h", "shaft.near", "spline.teeth", "cable.h",
                  "cable.gland_w", "horn.mass_g", "horn.hub_d", "horn.h", "horn.arm_r", "horn.arm_t",
                  "electrical.stall_torque"),
}

# XL330: arka idler yok (eski model; FPX330 idler seti kullanilmiyor)
SERVO_TYPES["XL330"]["idler"] = None

# PWM horn gobek deligi ile spline arasi radyal bosluk (zarf modeli; esit yaricapli es eksenli yuzeyler OCC'de
# carpisma hacmi gurultusu verir)
SPLINE_CLEAR = 0.05

# Rulman katalogu (d x D x B, mm; flans d x t; kutle g). Idler: F623ZZ (spec 3.1); doner tabla: 608ZZ (taban) ya da
# 6800ZZ (kutle merdiveni 4, ince kesit). Kaynaklar: servo-mechanics.md 4, uretici standart olculeri.
BEARINGS = {
    "F623ZZ": {"d": 3.0, "D": 10.0, "B": 4.0, "flange_d": 11.5, "flange_t": 1.0, "mass_g": 1.6},
    "623ZZ": {"d": 3.0, "D": 10.0, "B": 4.0, "flange_d": None, "flange_t": 0.0, "mass_g": 1.5},
    "693ZZ": {"d": 3.0, "D": 8.0, "B": 4.0, "flange_d": None, "flange_t": 0.0, "mass_g": 1.0},
    "608ZZ": {"d": 8.0, "D": 22.0, "B": 7.0, "flange_d": None, "flange_t": 0.0, "mass_g": 12.0},
    "6800ZZ": {"d": 10.0, "D": 19.0, "B": 5.0, "flange_d": None, "flange_t": 0.0, "mass_g": 5.5},
}


# ---------------------------------------------------------------------------
# Erisim
# ---------------------------------------------------------------------------
def get(name):
    """Kayit (buyuk/kucuk harf duyarsiz). Bilinmeyen ad -> KeyError (bilinenlerle)."""
    key = str(name).upper()
    if key not in SERVO_TYPES:
        raise KeyError("bilinmeyen servo turu %r (bilinen: %s)" % (name, ", ".join(SERVO_TYPES)))
    return SERVO_TYPES[key]


def mass_kg(t):
    return t["mass_g"] / 1000.0


def horn_mass_kg(t):
    return t["horn"]["mass_g"] / 1000.0


def actuator(t):
    return t["actuator"]


def spline_step_deg(t):
    """Horn'u bir dis kaydirmanin acisi (25T 14.4, 23T 15.65, 21T 17.14, 20T 18); spline yoksa None."""
    sp = t.get("spline")
    return 360.0 / sp["teeth"] if sp and sp.get("teeth") else None


def travel_range(t, center=0.0):
    """Servonun eklem acisi araligi (derece): orta darbedeki eklem acisi `center` +- hareket/2."""
    h = t["travel_deg"] / 2.0
    return center - h, center + h


def horn_face_y(t):
    """Kanonik cercevede horn dis yuzunun (braket datumu) Y konumu. XL330: orijin govde+horn yiginin
    (26+3 mm) ortasinda -> 14.5. PWM turleri: orijin horn dis yuzunde -> 0."""
    if t["geometry"] == "xl330":
        return (t["body"]["h"] + t["horn"]["t"]) / 2
    if t["geometry"] == "pwm":
        return 0.0
    raise NotImplementedError("servos.horn_face_y: %s kanonik zarfi yok" % t["name"])


def face_h(t):
    """Horn dis yuzunun govde tabanindan (arka yuz) yuksekligi (mm). Kanonik taban duzlemi Y = horn_face_y - face_h."""
    if t["geometry"] == "xl330":
        return t["body"]["h"] + t["horn"]["t"]
    return t["horn"]["h"][1]


def rear_face_y(t):
    """Kanonik cercevede govde tabaninin (arka yuz; idler tarafi) Y konumu."""
    return horn_face_y(t) - face_h(t)


def body_center_z(t):
    """Govde uzunluk ortasinin (kulak/delik deseni merkezi) kanonik Z konumu: milden uzak uca dogru."""
    return t["body"]["l"] / 2.0 - t["shaft"]["near"]


def idler_stack(t, bearing=None):
    """DS arka idler duzlemleri (kanonik Y, mm) ve rulman; idler'siz turde None. Arka yuzden disari (-Y):
    seat_gap | besik taban plakasi (stator, plate_t; rulman cebi D+0.1, derinlik B - flans) | rulman flansi |
    run_gap | U-link B kolu (rotor, leg_t, M3 somun yuvasi) | M3 vida basi (head_t).
      axis: rulman merkezi (idler ekseni noktasi, kanonik (0, axis, 0))."""
    idl = t.get("idler")
    if not idl:
        return None
    b = BEARINGS[bearing or idl["bearing"]]
    yr = rear_face_y(t)
    seat0 = yr - idl["seat_gap"]
    seat1 = seat0 - idl["plate_t"]
    b_in = seat1 + (b["B"] - b["flange_t"])          # rulmanin servo tarafi yuzu
    b_out = b_in - b["B"]
    leg0 = b_out - idl["run_gap"]
    leg1 = leg0 - idl["leg_t"]
    return {"rear_face": yr, "seat": (seat1, seat0), "bearing": (b_out, b_in), "axis": (b_in + b_out) / 2.0,
            "leg": (leg1, leg0), "head": (leg1 - idl["head_t"], leg1), "bearing_rec": b,
            "seat_r": b["D"] / 2.0 + idl["seat_wall"], "boss_r": idl["boss_r"], "head_r": idl["head_d"] / 2.0}


def datums(t, bearing=None):
    """Braket tasarimi icin kanonik datumlar (FreeCAD gerektirmez; mm). Y duzlemleri, kulak delikleri (x, z),
    kablo cikisi (nokta, yon), idler ekseni (nokta | None)."""
    b, tb, ho = t["body"], t["tabs"], t["holes"]
    yb = rear_face_y(t)
    zc = body_center_z(t)
    out = {"horn_face": horn_face_y(t), "rear_face": yb, "case_top": yb + b.get("main_h", b["h"]),
           "tower_top": yb + b["h"], "near_end_z": -t["shaft"]["near"], "far_end_z": b["l"] - t["shaft"]["near"],
           "half_w": b["w"] / 2.0, "body_center_z": zc}
    if t.get("spline"):
        out["spline_top"] = yb + t["spline"]["top_h"]
    if t.get("boss") and t["boss"].get("top_h"):
        out["boss_top"] = yb + t["boss"]["top_h"]
    if tb:
        out["tab"] = (yb + tb["under_h"], yb + tb["under_h"] + tb["t"])
        out["tab_z"] = (zc - tb["span"] / 2.0, zc + tb["span"] / 2.0)
        out["tab_holes"] = [(sx * ho["pitch_w"] / 2.0, zc + sz * ho["pitch_l"] / 2.0)
                            for sz in (-1, 1) for sx in ((-1, 1) if ho["pitch_w"] else (1,))]
    c = t["cable"]
    if c["face"] == "near_end" and c.get("h") is not None:
        out["cable_exit"] = ((0.0, yb + c["h"], -t["shaft"]["near"] - c.get("gland_out", 0.0)), (0.0, 0.0, -1.0))
    st = idler_stack(t, bearing)
    out["idler_axis"] = (0.0, st["axis"], 0.0) if st else None
    out["idler"] = st
    return out


def arm_angle_deg(horn_dir, far_dir, toward):
    """Kanonik +Z'deki (uzak uc) horn kolunu dunya `toward` yonune ceviren aci (derece), horn ekseni etrafinda
    (sag el). toward horn eksenine dik olmali (yakin degilse izdusumu kullanilir)."""
    h, f, u = [list(map(float, v)) for v in (horn_dir, far_dir, toward)]
    d = sum(a * b for a, b in zip(u, h))
    u = [a - d * b for a, b in zip(u, h)]
    cx = (f[1] * u[2] - f[2] * u[1], f[2] * u[0] - f[0] * u[2], f[0] * u[1] - f[1] * u[0])
    return math.degrees(math.atan2(sum(a * b for a, b in zip(cx, h)), sum(a * b for a, b in zip(f, u))))


def turntable_stack(tt, bearing=None):
    """hip_yaw doner tablasi (spec 3.1) z duzlemleri, kalca-govde birlesme duzlemine (tt["z_mate"]) gore mutlak
    (govde cercevesi, mm). Alt rulman birlesme duzleminden baslar, rulman merkezleri arasi tt["spacing"] (>= 20). Ust
    rulmanin ustunde somun/kaplin surucusu (nut_t), kaplin boslugu ve ES3054 horn dis yuzu (horn_face: konfigin
    rulmaniyla (tt["bearing"], P2-0'da 6800ZZ) dondurulur; bearing argumani yalniz yatak duzlemlerini degistirir).
    Doner: {"lower": (z0, z1), "upper": (z0, z1), "shaft_top", "horn_face", "housing_r", "bearing_rec", "z_mate",
    "spacing"}."""
    b = BEARINGS[bearing or tt["bearing"]]
    b0 = BEARINGS[tt["bearing"]]
    zm = tt["z_mate"]
    lo = (zm, zm + b["B"])
    up = (lo[0] + tt["spacing"], lo[0] + tt["spacing"] + b["B"])
    top0 = zm + tt["spacing"] + b0["B"] + tt["nut_t"]          # taban rulmaniyla dondurulmus mil ust ucu
    return {"lower": lo, "upper": up, "shaft_top": top0, "horn_face": top0 + tt["coupler_gap"],
            "housing_r": b["D"] / 2.0 + tt["housing_wall"], "bearing_rec": b, "z_mate": zm,
            "spacing": up[0] - lo[0]}


def require(m, sv, keys, tname, who, scale=None):
    """Uretec on kosulu: `keys` eklemleri `tname` turunde (ve scale verilmisse model olcegi o) olmali; degilse
    NotImplementedError (hangi uretecin hangi zarf/olcekle cizildigini soyler)."""
    bad = ["%s=%s" % (k, sv[k]["name"]) for k in keys if sv[k]["name"] != tname]
    if bad:
        raise NotImplementedError("%s yalniz %s servo zarfiyla cizildi: %s" % (who, tname, ", ".join(bad)))
    if scale is not None and m.p["scale"] != scale:
        raise NotImplementedError("%s %g olcekli literallerle cizildi (scale=%g)" % (who, scale, m.p["scale"]))


# ---------------------------------------------------------------------------
# Geometri (FreeCAD)
# ---------------------------------------------------------------------------
def horn_bolt_xz(t):
    """Horn civata deliklerinin kanonik (x, z) konumlari. XL330: 4 delik r4.2 / 45 derece; DS aluminyum kol: kol
    uzerinde (+Z) r 20 ve 24; plastik cift kol: kol uclarinda +-Z."""
    hn = t["horn"]
    if t["geometry"] == "xl330":
        a0, n = math.radians(hn["bolt_a0_deg"]), hn["bolt_n"]
        return [(hn["bolt_r"] * math.cos(a0 + k * (2 * math.pi / n)),
                 hn["bolt_r"] * math.sin(a0 + k * (2 * math.pi / n))) for k in range(n)]
    if hn["kind"] in ("al_arm_25t", "plastic_arm1"):
        return [(0.0, r) for r in hn["hole_r"]]
    return [(0.0, s * r) for r in hn["hole_r"] for s in (1, -1)]


def horn_arm_ends(t):
    """Horn kol uclari (kanonik (z, uc yaricapi)): DS aluminyum ve plastik tek kol +Z, plastik cift kol +-Z."""
    hn = t["horn"]
    if hn["kind"] == "al_arm_25t":
        return [(hn["hole_r"][-1], hn["arm_tip_r"])]
    if hn["kind"] == "plastic_arm1":
        return [(hn["arm_r"], hn["arm_tip_r"])]
    return [(hn["arm_r"], hn["arm_tip_r"]), (-hn["arm_r"], hn["arm_tip_r"])]


def _arm_xz(r0, z1, r1):
    """XZ'de (0, 0) r0 ve (0, z1) r1 dairelerinin dis teget dortgeni (kol govdesi; daireler ayrica eklenir)."""
    L = abs(z1)
    s = 1.0 if z1 > 0 else -1.0
    nz = (r0 - r1) / L
    nx = math.sqrt(max(1.0 - nz * nz, 0.0))
    return [(r0 * nx, s * r0 * nz), (r1 * nx, z1 + s * r1 * nz), (-r1 * nx, z1 + s * r1 * nz), (-r0 * nx, s * r0 * nz)]


def _horn_outline(t, y0, y1, grow=0.0):
    """Horn kol(lar)i + gobek dis hatti (grow kadar buyutulmus), kanonik Y'de y0..y1 kati."""
    from geom import prism_xz, ycyl
    hn = t["horn"]
    r0 = hn["hub_d"] / 2.0 + grow
    parts = [ycyl(r0, 0, 0, y0, y1)]
    for z1, rt in horn_arm_ends(t):
        rt += grow
        parts += [prism_xz(_arm_xz(r0, z1, rt), y0, y1), ycyl(rt, 0, z1, y0, y1)]
    return parts[0].fuse(parts[1:]).removeSplitter()


def _canon_pwm(t):
    """PWM turu (govde, horn), kanonik cerceve: orijin horn dis yuzunde (mil ekseni), govde tabani Y = -face_h."""
    from geom import box, fillet_axis, ycyl, zcyl
    b, sp, tb, ho, c, hn = t["body"], t["spline"], t["tabs"], t["holes"], t["cable"], t["horn"]
    yb = rear_face_y(t)
    w2, near = b["w"] / 2.0, t["shaft"]["near"]
    far = b["l"] - near
    zc = body_center_z(t)
    main_h = b.get("main_h", b["h"])
    body = box(-w2, w2, yb, yb + main_h, -near, far)
    body = fillet_axis(body, b["corner_r"], (0, 1, 0))
    parts = []
    if main_h < b["h"] - 1e-9:                     # disli kulesi: mil merkezli stadyum (+Z)
        tl = b.get("tower_l", 0.0)
        tower = ycyl(w2, 0, 0, yb + main_h, yb + b["h"])
        if tl > 0:
            tower = tower.fuse([box(-w2, w2, yb + main_h, yb + b["h"], 0, tl),
                                ycyl(w2, 0, tl, yb + main_h, yb + b["h"])])
        parts.append(tower.common(box(-w2, w2, yb + main_h - 1, yb + b["h"] + 1, -near, far)))
    y_top = yb + b["h"]
    bs = t.get("boss")
    if bs and bs.get("d"):                          # cikis gobegi (DS d11.8)
        parts.append(ycyl(bs["d"] / 2.0, 0, 0, y_top - 0.01, yb + bs["top_h"]))
        y_top = yb + bs["top_h"]
    elif bs and bs.get("top_h"):
        y_top = yb + bs["top_h"]
    parts.append(ycyl(sp["od"] / 2.0, 0, 0, y_top - 0.01, yb + sp["top_h"]))     # spline
    # kulaklar: tam genislik, govde ortasinda; delikler acik yarik (uca dogru)
    tab = box(-tb["w"] / 2.0, tb["w"] / 2.0, yb + tb["under_h"], yb + tb["under_h"] + tb["t"],
              zc - tb["span"] / 2.0, zc + tb["span"] / 2.0)
    r = ho["d"] / 2.0
    xs = (-ho["pitch_w"] / 2.0, ho["pitch_w"] / 2.0) if ho["pitch_w"] else (0.0,)
    for sz in (-1, 1):
        z = zc + sz * ho["pitch_l"] / 2.0
        zend = zc + sz * (tb["span"] / 2.0 + 1.0)
        for x in xs:
            tab = tab.cut(ycyl(r, x, z, yb + tb["under_h"] - 1, yb + tb["under_h"] + tb["t"] + 1))
            if ho.get("open_slot"):
                tab = tab.cut(box(x - r, x + r, yb + tb["under_h"] - 1, yb + tb["under_h"] + tb["t"] + 1,
                                  min(z, zend), max(z, zend)))
    parts.append(tab)
    if c.get("gland_out"):                          # kablo rakoru: yakin uc yuzunde, genislikte ortada
        gh2 = c["gland_h"] / 2.0
        parts.append(box(-c["gland_w"] / 2.0, c["gland_w"] / 2.0, yb + c["h"] - gh2, yb + c["h"] + gh2,
                         -near - c["gland_out"], -near + 0.01))
    body = body.fuse(parts).removeSplitter()
    # horn: gobek + kol(lar) dis yuzle ayni duzlemde; gobekte spline deligi ve vida deligi; kol delikleri
    y0, y1 = yb + hn["h"][0], yb + hn["h"][1]
    hub = ycyl(hn["hub_d"] / 2.0, 0, 0, y0, y1)
    arms = _horn_outline(t, y1 - hn["arm_t"], y1)
    horn = hub.fuse(arms).removeSplitter()
    horn = horn.cut(ycyl(sp["od"] / 2.0 + SPLINE_CLEAR, 0, 0, y0 - 1, yb + sp["top_h"]))   # spline'a 0.05 bosluk
    horn = horn.cut(ycyl(hn["screw_d"] / 2.0, 0, 0, y0 - 1, y1 + 1))
    for x, z in horn_bolt_xz(t):
        horn = horn.cut(ycyl(hn["hole_d"] / 2.0, x, z, y0 - 1, y1 + 1))
    return body, horn


def idler_parts(t, bearing=None):
    """DS arka idler zarflari, kanonik cerceve (idler'siz turde None): (stator, rotor, rulman). stator: besik
    taban plakasi rulman yuvasi (servo govdesiyle ayni link), rotor: U-link B kolu gobegi + M3 vida basi (horn ile
    ayni link), rulman: halka (stator link'te; ic bilezik rotorla doner)."""
    st = idler_stack(t, bearing)
    if st is None:
        return None
    from geom import ycyl
    b = st["bearing_rec"]
    stator = ycyl(st["seat_r"], 0, 0, st["seat"][0], st["seat"][1])
    stator = stator.cut(ycyl(b["D"] / 2.0 + 0.05, 0, 0, st["seat"][0] - 1, st["bearing"][1]))
    stator = stator.cut(ycyl(b["d"] / 2.0 + 1.5, 0, 0, st["seat"][0] - 1, st["seat"][1] + 1))
    rotor = ycyl(st["boss_r"], 0, 0, st["leg"][0], st["leg"][1]).fuse(
        ycyl(st["head_r"], 0, 0, st["head"][0], st["head"][1] + 0.01)).removeSplitter()
    ring = ycyl(b["D"] / 2.0, 0, 0, st["bearing"][0], st["bearing"][1])
    if b.get("flange_d"):
        ring = ring.fuse(ycyl(b["flange_d"] / 2.0, 0, 0, st["bearing"][0], st["bearing"][0] + b["flange_t"]))
    ring = ring.cut(ycyl(b["d"] / 2.0, 0, 0, st["bearing"][0] - 1, st["bearing"][1] + 1))
    return stator, rotor, ring


def horn_pocket(t, clearance=0.2, depth=None, over=0.0):
    """Horn'u linke gommek icin kesici (kanonik): kol(lar) + gobek dis hatti clearance kadar buyuk, Y'de
    -(depth | arm_t) - clearance .. over (horn dis yuzunden disari). Gobegin kol altinda kalan kismi icin
    depth = gobek boyu verilir."""
    hn = t["horn"]
    d = hn["arm_t"] if depth is None else depth
    return _horn_outline(t, horn_face_y(t) - d - clearance, horn_face_y(t) + over, clearance)


def canon(t):
    """Kanonik cercevede (govde, horn) katilari. XL330: orijin mil ekseninde, govde+horn yiginin ortasinda;
    PWM: orijin horn dis yuzunde (kulaklar, delikler, gobek/spline, kablo rakoru dahil)."""
    if t["geometry"] == "pwm":
        return _canon_pwm(t)
    if t["geometry"] != "xl330":
        raise NotImplementedError("servos.canon: %s zarf geometrisi yok" % t["name"])
    from geom import box, ycyl, fillet_axis
    b, hn, c = t["body"], t["horn"], t["cable"]
    w2, d, near = b["w"] / 2, b["h"], t["shaft"]["near"]
    far = b["l"] - near
    y_face = (d + hn["t"]) / 2 - hn["t"]       # govdenin horn tarafi yuzu
    body = box(-w2, w2, y_face - d, y_face, -near, far)
    body = fillet_axis(body, b["corner_r"], (0, 1, 0))
    body = body.cut(box(-w2 - 1, w2 + 1, y_face - d + c["slot_h"][0], y_face - d + c["slot_h"][1],
                        far - c["slot_depth"], far + 1))      # kablo yuvasi
    horn = ycyl(hn["r"], 0, 0, y_face, y_face + hn["t"])
    horn = horn.cut(ycyl(hn["center_hole_r"], 0, 0, y_face - 1, y_face + hn["t"] + 1))
    for cx, cz in horn_bolt_xz(t):
        horn = horn.cut(ycyl(hn["bolt_hole_r"], cx, cz, y_face - 1, y_face + hn["t"] + 1))
    return body, horn


def horn_bolt_holes(t, horn, lo, hi, r=1.0):
    """Modellenen (yerlestirilmis) horn'un civata delikleri (eksen disi silindirik yuzeyler) hizasinda, horn
    ekseni boyunca [lo, hi] (eksenin dunya koordinati) araliginda r yaricapli bosluk delikleri."""
    import Part
    from geom import V
    cyls = [f.Surface for f in horn.Faces if isinstance(f.Surface, Part.Cylinder)]
    main = max(cyls, key=lambda s: s.Radius)
    a, c = main.Axis, main.Center
    k = max(range(3), key=lambda i: abs(a[i]))
    d = [0.0, 0.0, 0.0]
    d[k] = 1.0
    holes, seen = [], set()
    for s in cyls:
        off = s.Center - c
        off = off - a * off.dot(a)
        if off.Length < 1.0:
            continue
        q = c + off
        key = tuple(round(q[i], 3) for i in range(3) if i != k)
        if key in seen:
            continue
        seen.add(key)
        o = [q.x, q.y, q.z]
        o[k] = lo
        holes.append(Part.makeCylinder(r, hi - lo, V(*o), V(*d)))
    assert len(holes) == t["horn"]["bolt_n"], "horn vida deligi sayisi: %d" % len(holes)
    return holes


def horn_bolt_cutters(t, length, cb=None, r=1.0):
    """Horn'a oturan braket icin kanonik cercevede bolt_n x r civata deligi (horn'daki delik dairesiyle ayni).
    length: horn dis yuzunden itibaren delik boyu; cb = (yaricap, bas, son) istege bagli havsa (horn dis
    yuzunden uzaklik). Tek kati (fuse) doner; cagiran servo cercevesine tasir. PWM: delik kolun arka yuzunden
    (Y = -arm_t) baslar."""
    from geom import ycyl
    if t["geometry"] == "xl330":
        y0 = (t["body"]["h"] + t["horn"]["t"]) / 2 - t["horn"]["t"]
        y1 = y0 + t["horn"]["t"]
    else:
        y1 = horn_face_y(t)
        y0 = y1 - t["horn"]["arm_t"]
    cut = []
    for cx, cz in horn_bolt_xz(t):
        cut.append(ycyl(r, cx, cz, y0, y1 + length))
        if cb:
            cut.append(ycyl(cb[0], cx, cz, y1 + cb[1], y1 + cb[2]))
    return cut[0].fuse(cut[1:])
