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
Yalniz XL330'un geometri ureteci (canon) var; PWM turlerinin zarflari P2a'da eklenir.
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
    "interface": "pwm", "geometry": None,
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
    "horn": {"kind": "al_arm_25t", "hub_d": 15.0, "h": (42.2, 47.9), "hole_d": 2.6, "hole_r": (20.0, 24.0),
             "clamp": "M2", "center_screw": "M3", "mass_g": 5.0},
    "horn_alt": {"al_disc_25t": {"d": 20.0, "hub_d": 9.0, "h": (42.0, 46.5), "bolt_n": 4, "bolt_pcd": 14.0,
                                 "bolt": "M3"}},
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
    "estimated": ("body.corner_r", "horn.mass_g"),
}

# ---------------------------------------------------------------------------
# EMAX ES3054 (EMAX olculu cizim; Robocombo ilani ayni)
# ---------------------------------------------------------------------------
SERVO_TYPES["ES3054"] = {
    "name": "ES3054", "label": "EMAX ES3054", "models": ("EMAX ES3054",),
    "interface": "pwm", "geometry": None,
    "body": {"l": 28.45, "w": 13.0, "h": 26.7, "corner_r": 0.5},
    "shaft": {"near": 6.0, "x": 0.0},                       # 6.1 turetilmis, cizimden 5.7-6.0
    "boss": None,
    "spline": {"teeth": 23, "od": 5.9, "top_h": 31.1, "screw": "M3", "screw_depth": None},
    "tabs": {"span": 38.35, "w": 13.0, "t": 1.9, "under_h": 17.8, "top_h": 19.7},
    "holes": {"n": 2, "d": 4.1, "pitch_l": 35.0, "pitch_w": 0.0, "from_tab_end": 1.675, "open_slot": True},
    "cable": {"face": "near_end", "h": 8.0, "lead": 240.0},  # yakin uc, alcak (fotograf); h olculecek
    "horn": {"kind": "plastic_set", "n_supplied": 3, "screw": "M3", "mass_g": 1.0},
    "mass_g": 20.0,                                         # EMAX 17; olculen 20.1
    "travel_deg": 145.0, "pwm_us": (700, 2300), "neutral_us": 1500, "deadband_us": None, "frame_hz": (50, None),
    "electrical": {"v_nom": 6.0, "v_min": 4.8, "v_max": 6.0, "stall_torque": 0.343, "omega0": 8.06,
                   "stall_current": None, "idle_current": None},
    "actuator": {"class": "es3054", "mjcf_class": "es3054", "kp": 6.551, "forcerange": 0.343, "damping": 0.04256,
                 "frictionloss": 0.001715, "armature": 0.000643, "velocity": 8.06, "pband_deg": 3.0,
                 "deadband_deg": 0.5, "source": "EMAX @6.0 V; P-bandi/olu bant tahmini, Adim 0"},
    "estimated": ("body.corner_r", "shaft.near", "cable.h", "horn.mass_g", "actuator.deadband_deg"),
}

# ---------------------------------------------------------------------------
# TowerPro MG90S (A-F tablosu; delik araligi ve mil ofseti DSSERVO MG90S cizimindan olculdu)
# ---------------------------------------------------------------------------
SERVO_TYPES["MG90S"] = {
    "name": "MG90S", "label": "TowerPro MG90S", "models": ("TowerPro MG90S",),
    "interface": "pwm", "geometry": None,
    "body": {"l": 22.8, "w": 12.2, "h": 28.4, "corner_r": 0.5},    # D 12.4 (A-F) / 12.2 (olcu cizgisi)
    "shaft": {"near": 5.9, "x": 0.0},
    "boss": None,
    "spline": {"teeth": 21, "od": 4.8, "top_h": 32.5, "screw": "M2 self-tapping", "screw_depth": None},
    "tabs": {"span": 32.1, "w": 12.2, "t": 2.4, "under_h": 18.5, "top_h": 20.9},
    "holes": {"n": 2, "d": 2.0, "pitch_l": 27.8, "pitch_w": 0.0, "from_tab_end": 2.15, "open_slot": True},
    "cable": {"face": "near_end", "h": 3.0, "lead": 250.0},
    "horn": {"kind": "plastic_set", "n_supplied": 3, "screw": "M2 self-tapping", "mass_g": 0.5},
    "mass_g": 13.4,
    "travel_deg": 180.0, "pwm_us": (500, 2400), "neutral_us": 1500, "deadband_us": 5.0, "frame_hz": (50, 50),
    "electrical": {"v_nom": 6.0, "v_min": 4.8, "v_max": 6.0, "stall_torque": 0.216, "omega0": 13.1,
                   "stall_current": 0.9, "idle_current": 0.25},
    "actuator": {"class": "mg90s", "mjcf_class": "mg90s", "kp": 4.125, "forcerange": 0.216, "damping": 0.01649,
                 "frictionloss": 0.00108, "armature": 0.000405, "velocity": 13.1, "pband_deg": 3.0,
                 "deadband_deg": 0.9, "source": "TowerPro @6.0 V; P-bandi tahmini, Adim 0"},
    "estimated": ("body.corner_r", "shaft.near", "spline.teeth", "tabs.t", "holes.pitch_l", "cable.h",
                  "horn.mass_g"),
}

# ---------------------------------------------------------------------------
# SG90 (TowerPro A-F; tab kalinligi, delik araligi ve mil ofseti klon cizimden)
# ---------------------------------------------------------------------------
SERVO_TYPES["SG90"] = {
    "name": "SG90", "label": "TowerPro SG90", "models": ("TowerPro SG90",),
    "interface": "pwm", "geometry": None,
    "body": {"l": 22.7, "w": 12.2, "h": 27.0, "main_h": 22.1, "corner_r": 0.5},  # main_h: ana govde ustu
    "shaft": {"near": 5.9, "x": 0.0},
    "boss": {"d": None, "top_h": 26.5},
    "spline": {"teeth": 20, "od": 4.9, "top_h": 30.3, "screw": "M2 self-tapping", "screw_depth": None},
    "tabs": {"span": 32.3, "w": 12.2, "t": 2.4, "under_h": 17.0, "top_h": 19.4},
    "holes": {"n": 2, "d": 2.0, "pitch_l": 27.85, "pitch_w": 0.0, "from_tab_end": 2.225, "open_slot": True},
    "cable": {"face": "near_end", "h": 3.0, "lead": 250.0},
    "horn": {"kind": "plastic_set", "n_supplied": 3, "screw": "M2 self-tapping", "mass_g": 0.5},
    "mass_g": 9.0,
    "travel_deg": 180.0, "pwm_us": (500, 2400), "neutral_us": 1500, "deadband_us": 5.0, "frame_hz": (50, 50),
    # tork/hiz yalniz 4.8 V'ta yayimli (6.0 V'ta calisir; Adim 0'da olculur)
    "electrical": {"v_nom": 6.0, "v_min": 4.8, "v_max": 6.0, "stall_torque": 0.177, "omega0": 10.5,
                   "stall_current": 0.75, "idle_current": 0.09},
    "actuator": {"class": "sg90", "mjcf_class": "sg90", "kp": 3.38, "forcerange": 0.177, "damping": 0.01686,
                 "frictionloss": 0.000885, "armature": 0.000332, "velocity": 10.5, "pband_deg": 3.0,
                 "deadband_deg": 0.9, "source": "TowerPro @4.8 V; P-bandi tahmini, Adim 0"},
    "estimated": ("body.corner_r", "tabs.under_h", "shaft.near", "spline.teeth", "cable.h", "horn.mass_g",
                  "electrical.stall_torque"),
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
    (26+3 mm) ortasinda -> 14.5."""
    if t["geometry"] == "xl330":
        return (t["body"]["h"] + t["horn"]["t"]) / 2
    raise NotImplementedError("servos.horn_face_y: %s kanonik zarfi henuz yok (P2a)" % t["name"])


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
    """Horn civata deliklerinin kanonik (x, z) konumlari."""
    hn = t["horn"]
    a0, n = math.radians(hn["bolt_a0_deg"]), hn["bolt_n"]
    return [(hn["bolt_r"] * math.cos(a0 + k * (2 * math.pi / n)), hn["bolt_r"] * math.sin(a0 + k * (2 * math.pi / n)))
            for k in range(n)]


def canon(t):
    """Kanonik cercevede (govde, horn) katilari. XL330: orijin mil ekseninde, govde+horn yiginin ortasinda."""
    if t["geometry"] != "xl330":
        raise NotImplementedError("servos.canon: %s zarf geometrisi henuz yok (P2a)" % t["name"])
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
    yuzunden uzaklik). Tek kati (fuse) doner; cagiran servo cercevesine tasir."""
    from geom import ycyl
    y0 = (t["body"]["h"] + t["horn"]["t"]) / 2 - t["horn"]["t"]
    y1 = y0 + t["horn"]["t"]
    cut = []
    for cx, cz in horn_bolt_xz(t):
        cut.append(ycyl(r, cx, cz, y0, y1 + length))
        if cb:
            cut.append(ycyl(cb[0], cx, cz, y1 + cb[1], y1 + cb[2]))
    return cut[0].fuse(cut[1:])
