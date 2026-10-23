# -*- coding: utf-8 -*-
"""
OpenDuck - Microduck (Pollen Robotics / Hugging Face) benzeri, ~27 cm, 15 servolu
(DYNAMIXEL XL330-M288-T) cift ayakli ordek robot. Tamamen parametrik FreeCAD modeli.

Eklem yerlesimi, adlari ve eksen yonleri Pollen'in Apache-2.0 lisansli MJCF'i
(pollen-robotics/microduck_rl, robot_walk.xml) ile ayni; boylece ayni yazilim ve
egitim yigini kullanilabilir. Kabuk ve braket geometrisi bu dosyada sifirdan cizildi
(Pollen'in CC BY-NC-SA STL'leri kullanilmadi).

Kullanim
  * FreeCAD GUI : Makro > Makrolar... > bu dosyayi sec > Calistir
  * Tam derleme : tools/build.sh   (FCStd, STEP, STL, URDF, MJCF, BOM, gorseller)
  * Python'dan  : import openduck; m = openduck.OpenDuck()

Koordinatlar: mm; govde (trunk) cercevesi: X ileri, Y sol, Z yukari. Sifir pozda butun
eklem cerceveleri dunya eksenlerine paraleldir (uyluk 58 derece geriye egik, incik dik).
Robotun pozu "Pose" hesap tablosundaki acilarla (derece) suruluyor.
"""
import math
from collections import OrderedDict

import FreeCAD as App
import Part

V = App.Vector

# ---------------------------------------------------------------------------
# PARAMETRELER (mm, kg)
# ---------------------------------------------------------------------------
PARAMS = {
    # DYNAMIXEL XL330-M288-T
    "servo_w": 20.0,            # genislik
    "servo_l": 34.0,            # uzunluk
    "servo_d": 26.0,            # mil yonunde kalinlik (horn haric)
    "servo_near": 9.5,          # mil ekseni ile yakin uc arasi
    "horn_r": 6.5,
    "horn_t": 3.0,
    # Baski
    "wall": 1.8,                # kabuk et kalinligi
    "plate_t": 2.5,             # braket plaka kalinligi
    "bracket_r": 9.5,           # eklem etrafindaki braket yaricapi
    # Kinematik: sol taraf eklem noktalari (govde cercevesi, sifir poz).
    # Pollen microduck_rl robot_walk.xml (Apache-2.0) ile ayni.
    "j_hip_yaw": (6.0, 17.5, -5.0),
    "j_hip_roll": (22.5, 17.5, -17.5),
    "j_hip_pitch": (4.0, 42.5, -17.5),
    "j_knee": (-31.8, 38.5, -39.5),
    "j_ankle": (-31.8, 64.5, -81.5),
    "j_neck_pitch": (26.0, 14.5, 32.4),
    "j_head_pitch": (26.0, 14.5, 82.4),
    "j_head_yaw": (26.0, 0.0, 101.1),
    "j_head_roll": (8.1, 0.0, 115.6),
    "j_mouth": (26.1, 28.0, 118.6),
    "sole_below_ankle": 25.0,
    "sole_cant_deg": 5.0,       # taban, ayakta pozda (kalca roll 5 derece) duz basacak sekilde egik
    # Govde
    "torso_x": (-46.4, 34.5), "torso_w": 64.4, "torso_z": (0.3, 42.0),
    "battery": (21.7, 38.6, 71.1),      # NP-F550 (x, y, z)
    # Kafa
    "head_x": (-30.4, 92.3), "head_w": 91.8, "head_top": 157.4, "head_split": 111.0, "head_bot": 98.7,
    "face_x": 72.5,             # yuz plakasinin on yuzu (kaputun ici)
    "eye_r": 15.0, "eye_z": 131.1,
    "neck_hole_r": 28.0,        # kafa tabanindaki boyun deligi
    # Ayak
    "foot_x": (-20.0, 34.0),    # ayak bilegi eksenine gore (topuk, burun)
    "foot_w": 40.5,
    "sole_t": 5.0,
    "foot_t": 7.5,
    # Malzeme
    "pla_density": 1.1e-6,      # kg/mm^3, efektif
    "tpu_density": 1.2e-6,
}

MASSES = {           # satin alinan parcalar (kg)
    "servo": 0.0175, "horn": 0.0005,
    "battery": 0.072, "board": 0.010, "hat": 0.008, "camera": 0.004, "speaker": 0.006,
    "tof": 0.001, "imu": 0.002, "power": 0.008,
}

# DYNAMIXEL XL330-M288-T, 2S (6.6-8.2 V) ile dogrudan beslenir (Pollen ile ayni).
# Simulasyon: kp, firmware P=200'un ~0.87 rad hatada %100 PWM'e doymasindan;
# sinir tork 0.96 N.m ve sonumleme (geri EMK) Pollen/BAM XL330 modelinden.
ACTUATOR = {"kp": 1.1, "forcerange": 0.96, "damping": 0.053, "frictionloss": 0.0048,
            "armature": 0.0018, "velocity": 10.0}

# Eklem sinirlari (derece; sol bacak, sag bacakta aynalanir). Pollen'in mekanik sinirlarindan
# (diz/bilek/kafa pitch +-90, kafa roll +-25, boyun -90..60) dar: cad/check.py ile tek eklem
# taramasinda carpismasiz kalan araliklar.
JOINT_LIMITS = {
    "hip_yaw": (-20.0, 30.0), "hip_roll": (-20.0, 20.0), "hip_pitch": (-90.0, 90.0),
    "knee": (-90.0, 70.0), "ankle": (-80.0, 80.0),
    "neck_pitch": (-90.0, 40.0), "head_pitch": (-30.0, 90.0), "head_yaw": (-170.0, 170.0),
    "head_roll": (-12.0, 12.0), "mouth": (0.0, 30.0),
}

# HOME ("stand") pozu, derece. Sag bacak eksenleri aynali oldugu icin isaretler ters.
HOME_POSE = {
    "left_hip_roll": -5.0, "left_hip_pitch": -26.237, "left_knee": -0.283, "left_ankle": 25.954,
    "right_hip_roll": 5.0, "right_hip_pitch": 26.237, "right_knee": 0.283, "right_ankle": -25.954,
    "neck_pitch": 20.0, "head_pitch": 20.0,
}
STAND_POSE = HOME_POSE

# Parca turleri: renk (GUI) ve baski malzemesi
KINDS = {
    "shell":   {"color": (0.93, 0.90, 0.84), "print": "PLA (krem)"},
    "trim":    {"color": (1.00, 0.50, 0.16), "print": "PLA (turuncu)"},
    "bracket": {"color": (0.62, 0.64, 0.67), "print": "PLA/PETG (gri)"},
    "sole":    {"color": (0.22, 0.26, 0.32), "print": "TPU 95A (koyu)"},
    "pad":     {"color": (0.30, 0.34, 0.42), "print": "TPU 85A (koyu)"},
    "face":    {"color": (0.80, 0.79, 0.76), "print": "PLA (acik gri)"},
    "lens":    {"color": (0.03, 0.03, 0.04)},
    "glass":   {"color": (0.35, 0.55, 0.85)},
    "led":     {"color": (0.95, 0.10, 0.10)},
    "servo":   {"color": (0.12, 0.12, 0.13)},
    "horn":    {"color": (0.35, 0.35, 0.37)},
    "pcb":     {"color": (0.10, 0.45, 0.20)},
    "pcb2":    {"color": (0.10, 0.25, 0.65)},
    "chip":    {"color": (0.08, 0.08, 0.08)},
    "battery": {"color": (0.18, 0.18, 0.20)},
    "metal":   {"color": (0.70, 0.70, 0.72)},
}
PRINT_KINDS = tuple(k for k, v in KINDS.items() if "print" in v)

# Kanonik servo cercevesinden dunyaya: mil (horn) +Y, uzak uc +Z, X = Y x Z.
# Kanonik orijin: mil ekseni uzerinde, govde+horn yiginin (26+3 mm) ortasi.
SERVOS = {   # ad: (orijin, horn yonu, uzak uc yonu)   - sol taraf + orta
    "hip_yaw":    ((6.0, 17.5, 11.5), (0, 0, -1), (-1, 0, 0)),
    "hip_roll":   ((6.0, 17.5, -17.5), (1, 0, 0), (0, 0, -1)),
    "hip_pitch":  ((4.0, 55.0, -17.5), (0, -1, 0), (-1, 0, 0)),
    "knee":       ((-31.8, 55.0, -39.5), (0, -1, 0), (0, 0, 1)),
    "ankle":      ((-31.8, 50.0, -81.5), (0, 1, 0), (0, 0, 1)),
    "neck_pitch": ((26.0, 0.0, 32.4), (0, 1, 0), (0, 0, 1)),
    "head_pitch": ((26.0, 0.0, 82.4), (0, 1, 0), (0, 0, -1)),
    "head_yaw":   ((26.0, 0.0, 115.6), (0, 0, -1), (0, -1, 0)),
    "head_roll":  ((-6.4, 0.0, 115.6), (1, 0, 0), (0, -1, 0)),
    "mouth":      ((26.1, 28.0, 118.6), (0, 1, 0), (1, 0, 0)),
}


# ---------------------------------------------------------------------------
# Geometri yardimcilari
# ---------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def xcyl(r, y, z, x0, x1):
    return Part.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def ycyl(r, x, z, y0, y1):
    return Part.makeCylinder(r, y1 - y0, V(x, y0, z), V(0, 1, 0))


def zcyl(r, x, y, z0, z1):
    return Part.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def _parallel(edge, axis):
    if not isinstance(edge.Curve, Part.Line):
        return False
    d = edge.Vertexes[-1].Point - edge.Vertexes[0].Point
    return d.Length > 1e-9 and abs(abs(d.normalize().dot(V(*axis))) - 1) < 1e-6


def _at_z(edge, z):
    bb = edge.BoundBox
    return abs(bb.ZMin - z) < 1e-6 and abs(bb.ZMax - z) < 1e-6


def fillet(shape, r, edges):
    """Kenar yuvarlatma; OCC tumunu yapamazsa kenar kenar dener, olmayanlari atlar."""
    if r <= 0 or not edges:
        return shape
    try:
        return shape.makeFillet(r, edges)
    except Exception:
        pass
    out = shape
    for e in edges:
        try:
            match = [f for f in out.Edges if f.isSame(e) or (f.CenterOfMass - e.CenterOfMass).Length < 1e-6]
            if match:
                out = out.makeFillet(r, match[:1])
        except Exception:
            continue
    return out


def fillet_axis(shape, r, axis):
    return fillet(shape, r, [e for e in shape.Edges if _parallel(e, axis)])


def rbox(x0, x1, y0, y1, z0, z1, r_side=0.0, r_top=0.0, r_bot=0.0):
    """Dikey kenarlari r_side, ust kenarlari r_top, alt kenarlari r_bot ile yuvarlatilmis kutu."""
    b = box(x0, x1, y0, y1, z0, z1)
    b = fillet_axis(b, r_side, (0, 0, 1))
    b = fillet(b, r_top, [e for e in b.Edges if _at_z(e, z1)])
    b = fillet(b, r_bot, [e for e in b.Edges if _at_z(e, z0)])
    return b


def shell_rbox(x0, x1, y0, y1, z0, z1, wall, r_side, r_top, r_bot):
    outer = rbox(x0, x1, y0, y1, z0, z1, r_side, r_top, r_bot)
    inner = rbox(x0 + wall, x1 - wall, y0 + wall, y1 - wall, z0 + wall, z1 - wall,
                 max(r_side - wall, 0.5), max(r_top - wall, 0.0), max(r_bot - wall, 0.0))
    return outer.cut(inner)


def stadium_y(cx, cz, r, z_end, y0, y1, x_half=None):
    """XZ duzleminde (cx,cz) etrafinda r yaricapli disk + z_end'e uzanan dikdortgen, Y'de ekstrude."""
    lo, hi = sorted((cz, z_end))
    w = r if x_half is None else x_half
    if hi - lo < 1e-6:
        return ycyl(r, cx, cz, y0, y1)
    return ycyl(r, cx, cz, y0, y1).fuse(box(cx - w, cx + w, y0, y1, lo, hi)).removeSplitter()


def hull2d(points):
    pts = sorted(set(points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def prism_xz(pts, y0, y1, r_round=0.0):
    """XZ duzleminde (x, z) cokgen, Y yonunde y0..y1 ekstrude; kose yuvarlatma r_round."""
    poly = Part.makePolygon([V(x, y0, z) for x, z in pts] + [V(pts[0][0], y0, pts[0][1])])
    solid = Part.Face(poly).extrude(V(0, y1 - y0, 0))
    return fillet_axis(solid, r_round, (0, 1, 0)) if r_round > 0 else solid


def bar_y(p0, p1, r, y0, y1):
    """XZ duzleminde iki daire (p0, p1; yaricap r) arasi kapsul, Y'de ekstrude."""
    (x0, z0), (x1, z1) = p0, p1
    dx, dz = x1 - x0, z1 - z0
    n = math.hypot(dx, dz)
    ox, oz = -dz / n * r, dx / n * r
    pts = [(x0 + ox, z0 + oz), (x1 + ox, z1 + oz), (x1 - ox, z1 - oz), (x0 - ox, z0 - oz)]
    return prism_xz(pts, y0, y1).fuse([ycyl(r, x0, z0, y0, y1), ycyl(r, x1, z1, y0, y1)]).removeSplitter()


def frame(origin, horn, far):
    """Kanonik servo cercevesi -> dunya yerlesimi."""
    Y, Z = V(*horn), V(*far)
    X = Y.cross(Z)
    m = App.Matrix(X.x, Y.x, Z.x, origin[0],
                   X.y, Y.y, Z.y, origin[1],
                   X.z, Y.z, Z.z, origin[2],
                   0, 0, 0, 1)
    return App.Placement(m)


def moved(shape, placement):
    s = shape.copy()
    s.Placement = placement.multiply(s.Placement)
    return s


def rotated(shape, center, axis, angle_deg):
    s = shape.copy()
    s.rotate(V(*center), V(*axis), angle_deg)
    return s


_MY = App.Matrix(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)


def mirror_y(shape):
    """XZ duzlemine gore ayna. Shape.mirror() yerlesimli sekillerde yanlis sonuc
    verdigi icin yerlesim ayrica aynalanir: M*P*S = (M*P*M) * (M*S)."""
    s = shape.copy()
    pm = s.Placement.toMatrix()
    s.Placement = App.Placement()
    m = s.mirror(V(0, 0, 0), V(0, 1, 0))
    m.Placement = App.Placement(_MY.multiply(pm).multiply(_MY)).multiply(m.Placement)
    return m


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
class Item(object):
    """Bir link'e bagli tek bir kati parca (dunya koordinatlarinda, sifir pozda)."""

    def __init__(self, name, shape, kind, mass=None, collision_box=None):
        self.name = name
        self.shape = shape
        self.kind = kind
        self.mass = mass
        self.collision_box = collision_box   # (merkez V, yari olculer, App.Rotation) dunya, sifir poz

    @property
    def part(self):
        for pre in ("left_", "right_"):
            if self.name.startswith(pre):
                return self.name[len(pre):]
        return self.name

    def mass_kg(self, p):
        if self.mass is not None:
            return self.mass
        dens = p["tpu_density"] if self.kind in ("sole", "pad") else p["pla_density"]
        return self.shape.Volume * dens


class Link(object):
    def __init__(self, name, origin):
        self.name = name
        self.origin = origin
        self.items = []


class OpenDuck(object):
    def __init__(self, params=None):
        self.p = dict(PARAMS)
        self.p.update(params or {})
        self.links = OrderedDict()
        self.joints = []
        self._servo_cache = None
        self.meta = {}
        self._build()

    # -- kayit -----------------------------------------------------------------
    def _link(self, name, origin):
        self.links[name] = Link(name, V(*origin))
        return self.links[name]

    def _add(self, link, name, shape, kind, mass=None, collision_box=None):
        self.links[link].items.append(Item(name, shape, kind, mass, collision_box))

    def _joint(self, name, parent, child, origin, axis, limits, home=0.0, servo_id=None):
        lo, hi = limits
        self.joints.append({
            "name": name, "parent": parent, "child": child, "origin": V(*origin), "axis": axis,
            "lower": lo, "upper": hi, "stand": home, "id": servo_id, "actuator": "xl330",
            "effort": ACTUATOR["forcerange"], "velocity": ACTUATOR["velocity"],
        })
        self._link(child, origin)

    # -- XL330 ------------------------------------------------------------------
    def _servo_canon(self):
        if self._servo_cache is None:
            p = self.p
            w2, d, near = p["servo_w"] / 2, p["servo_d"], p["servo_near"]
            far = p["servo_l"] - near
            y_face = (d + p["horn_t"]) / 2 - p["horn_t"]       # govdenin horn tarafi yuzu
            body = box(-w2, w2, y_face - d, y_face, -near, far)
            body = fillet_axis(body, 3.5, (0, 1, 0))
            body = body.cut(box(-w2 - 1, w2 + 1, y_face - d + 6, y_face - d + 7, far - 6, far + 1))  # kablo yuvasi
            horn = ycyl(p["horn_r"], 0, 0, y_face, y_face + p["horn_t"])
            horn = horn.cut(ycyl(1.2, 0, 0, y_face - 1, y_face + p["horn_t"] + 1))
            for k in range(4):
                a = math.pi / 4 + k * math.pi / 2
                horn = horn.cut(ycyl(0.8, 4.2 * math.cos(a), 4.2 * math.sin(a), y_face - 1, y_face + 4))
            self._servo_cache = (body, horn)
        return self._servo_cache

    def _servo(self, key, mirror=False):
        body, horn = self._servo_canon()
        o, h, f = SERVOS[key]
        pl = frame(o, h, f)
        b, hn = moved(body, pl), moved(horn, pl)
        return (mirror_y(b), mirror_y(hn)) if mirror else (b, hn)

    # -- govde -----------------------------------------------------------------
    def _trunk(self):
        p = self.p
        t, w = p["plate_t"], p["wall"]
        tx0, tx1 = p["torso_x"]
        tw2 = p["torso_w"] / 2
        tz0, tz1 = p["torso_z"]
        nx, _, nz = p["j_neck_pitch"]

        # sasi: taban plakasi + kalca yaw servo kafesleri + boyun catali. Batarya yuvasi ayri parca
        # (battery_tray): tek parcada taban plakasinin altina sarkiyor ve ~85 cm3 destek istiyordu.
        # Sasi boylece taban plakasi tablada (link -Z) desteksiz basilir.
        base = rbox(-25.0, 17.5, -30.0, 30.0, -1.0, 1.5, 4.0)
        for sy in (1, -1):
            base = base.cut(box(-18.7, 15.7, sy * 7.3 if sy > 0 else -27.7, 27.7 if sy > 0 else -7.3, 0.0, 2.0))
            base = base.cut(zcyl(7.5, 6.0, sy * 17.5, -2.0, 2.0))
        parts = []
        for sy in (1, -1):
            for y0, y1 in ((5.5, 7.5), (27.5, 29.5)):
                a, b = (y0, y1) if sy > 0 else (-y1, -y0)
                parts.append(box(-18.5, 15.5, a, b, 1.5, 20.0))
        # boyun catali: horn diskinden tabana inen dis bukey profil (on kenari ~74 derece, sarkma yok;
        # kabugun on duvarindan x<=31.8'de kalir) + taban plakasinin onune catal ayagi dili
        fr, (fpx, fpz) = 9.5, (27.0, 1.5)
        fd = math.hypot(nx - fpx, nz - fpz)
        fa = math.atan2(nz - fpz, nx - fpx) - math.asin(fr / fd)              # on teget dogrultusu
        ft = (fpx + math.cos(fa) * math.sqrt(fd * fd - fr * fr), fpz + math.sin(fa) * math.sqrt(fd * fd - fr * fr))
        fork_xz = [(nx - fr, fpz), (fpx, fpz), ft, (nx, nz), (nx - fr, nz)]
        for a, b in ((14.5, 17.0), (-17.0, -14.5)):
            parts.append(prism_xz(fork_xz, a, b).fuse(ycyl(fr, nx, nz, a, b)))
            parts.append(rbox(16.0, 27.0, a - 1.0, b + 1.0, -1.0, 1.5, 1.0))
        parts.append(box(-25.0, -22.5, -21.0, 21.0, -1.0, 30.0))       # arka duvar (taban ustu)
        chassis = base.fuse(parts).removeSplitter()
        chassis = chassis.cut(ycyl(1.6, nx, nz, -18.0, 18.0))
        chassis = chassis.cut(self._horn_bolt_holes(self._servo("neck_pitch")[1], 14.0, 17.5))
        # batarya yuvasi vidalari: 3x M2, ustten taban plakasindan yuvanin flansindaki isil eklere
        tray_screws = [(-21.5, 24.25), (-21.5, -24.25), (-19.5, 0.0)]
        chassis = chassis.cut([zcyl(1.1, sx, sy, -2.0, 2.0) for sx, sy in tray_screws])
        self._add("trunk", "chassis", chassis, "bracket")

        # batarya yuvasi: arka duvarin taban alti kismi + taban + yan dudaklar + taban plakasina
        # vidalanan flans (alti 50 derece pahli: taban tablada desteksiz basilir)
        bx0 = tx0 - 0.4
        tray = [box(-25.0, -22.5, -21.0, 21.0, -42.0, -1.0),
                box(bx0 - 1.5, -22.5, -21.0, 21.0, -42.0, -39.6),
                prism_xz([(-25.0, -1.0), (-18.5, -1.0), (-18.5, -5.0), (-25.0, -12.75)], -27.5, 27.5),
                prism_xz([(-22.5, -1.0), (-16.0, -1.0), (-16.0, -5.0), (-22.5, -12.75)], -4.0, 4.0)]   # orta vida dili
        for a, b in ((-21.0, -19.5), (19.5, 21.0)):
            tray.append(box(bx0 - 1.5, -22.5, a, b, -42.0, -30.0))
        tray = tray[0].fuse(tray[1:]).removeSplitter()
        tray = tray.cut([zcyl(1.6, sx, sy, -5.5, 0.0) for sx, sy in tray_screws])
        self._add("trunk", "battery_tray", tray, "bracket")

        # kabuk: iki yarim (sol/sag), alt acik, arkada batarya yuvasi, onde boyun acikligi
        shell = shell_rbox(tx0, tx1, -tw2, tw2, tz0, tz1, w, 10.0, 9.0, 2.0)
        shell = shell.cut(box(tx0 + w, tx1 - w, -tw2 + w, tw2 - w, tz0 - 1, tz0 + w + 0.01))
        shell = shell.cut(box(tx0 - 2, -24.5, -20.0, 20.0, tz0 - 1, 33.0))
        shell = shell.cut(rbox(2.0, tx1 + 2, -18.0, 18.0, 18.0, tz1 + 2, 4.0))
        shell = shell.cut(xcyl(3.0, 22.0, 24.0, tx0 - 1, tx0 + w + 1))        # USB-C / anahtar
        big = (tx0 - 1, tx1 + 1)
        self._add("trunk", "torso_shell_left", shell.common(box(big[0], big[1], 0.0, tw2 + 1, tz0 - 1, tz1 + 1)), "shell")
        self._add("trunk", "torso_shell_right", shell.common(box(big[0], big[1], -tw2 - 1, 0.0, tz0 - 1, tz1 + 1)), "shell")

        # batarya, IMU, guc karti
        bx, by, bz = p["battery"]
        bat = rbox(bx0, bx0 + bx, -by / 2, by / 2, -39.4, -39.4 + bz, 2.0)
        self._add("trunk", "battery_np_f550", bat, "battery", MASSES["battery"])
        imu = rbox(-4.0, 12.0, -5.0, 5.0, 3.5, 5.1, 1.0).fuse(
            [zcyl(1.0, px, py, 1.5, 3.5) for px in (-2.5, 10.5) for py in (-3.5, 3.5)])
        self._add("trunk", "imu_board", imu, "pcb", MASSES["imu"])
        self._add("trunk", "imu_chip", box(2.0, 6.0, -2.0, 2.0, 5.1, 6.0), "chip", 0.0)
        pw = rbox(-21.0, 12.0, -14.0, 14.0, 27.0, 28.6, 2.0).fuse(
            [zcyl(1.5, -15.0, py, 1.5, 27.0) for py in (-3.5, 3.5)])   # servolarin arasinda, tabana
        self._add("trunk", "power_board", pw, "pcb2", MASSES["power"])
        self.meta["imu_item"] = "imu_chip"

        # kalca yaw servolari ve boyun horn'u
        for side, m in (("left", False), ("right", True)):
            b, h = self._servo("hip_yaw", m)
            self._add("trunk", side + "_hip_yaw_servo", b, "servo", MASSES["servo"])
            self._hip_yaw_horn = getattr(self, "_hip_yaw_horn", {})
            self._hip_yaw_horn[side] = h

    # -- bacak (sol kurulur, sag aynalanir) --------------------------------------
    @staticmethod
    def _horn_bolt_holes(horn, lo, hi, r=1.0):
        """Modellenen horn'un 4 vida deligi (eksen disi silindirik yuzeyler) hizasinda, horn ekseni
        boyunca [lo, hi] (eksenin dunya koordinati) araliginda r yaricapli bosluk delikleri."""
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
        assert len(holes) == 4, "horn vida deligi sayisi: %d" % len(holes)
        return holes

    def _leg_items(self):
        p = self.p
        t = p["plate_t"]
        hp, kn, an = p["j_hip_pitch"], p["j_knee"], p["j_ankle"]
        out = []

        # hip (yaw2roll): yaw horn'una takili, roll servosunu tasir
        roll_b, roll_h = self._servo("hip_roll")
        hb = rbox(-11.5, 18.5, 6.5, 28.5, -6.0, -3.0, 3.0)
        hb = hb.fuse([box(-11.5, -8.5, 6.5, 28.5, -30.0, -6.0),
                      box(-11.5, 12.0, 5.5, 7.5, -14.0, -3.0),
                      box(-11.5, 12.0, 27.5, 29.5, -14.0, -3.0)]).removeSplitter()
        hb = hb.cut(zcyl(1.4, 6.0, 17.5, -7.0, -2.0))
        hb = hb.cut(self._horn_bolt_holes(self._servo("hip_yaw")[1], -7.0, -2.0))
        out += [("hip", "hip_yaw_horn", None, "horn", MASSES["horn"]),
                ("hip", "hip_bracket", hb, "bracket", None),
                ("hip", "hip_roll_servo", roll_b, "servo", MASSES["servo"])]

        # hip_cradle (hip_l): roll horn'u -> pitch horn'u (L braket)
        pitch_b, pitch_h = self._servo("hip_pitch")
        front = box(20.5, 23.5, 8.5, 40.5, -27.0, -8.0)
        side = stadium_y(hp[0], hp[2], 9.5, -17.5, 37.5, 40.5).fuse(box(hp[0], 23.5, 37.5, 40.5, -27.0, -8.0))
        cr = front.fuse(side).removeSplitter()
        cr = fillet_axis(cr, 3.0, (0, 1, 0))
        cr = cr.cut(xcyl(1.4, 17.5, -17.5, 19.0, 25.0)).cut(ycyl(1.4, hp[0], hp[2], 36.0, 42.0))
        cr = cr.cut(self._horn_bolt_holes(roll_h, 19.0, 25.0) + self._horn_bolt_holes(pitch_h, 36.0, 42.0))
        out += [("hip_cradle", "hip_roll_horn", roll_h, "horn", MASSES["horn"]),
                ("hip_cradle", "hip_cradle_bracket", cr, "bracket", None)]

        # uyluk: pitch + diz servolari, ic plaka ve dis "kanat" kabuk
        knee_b, knee_h = self._servo("knee")
        corners = []
        for (x0, x1, z0, z1) in ((-20.5, 13.5, -27.5, -7.5), (-41.8, -21.8, -49.0, -15.0)):
            corners += [(x0 - 3, z0 - 3), (x1 + 3, z0 - 3), (x1 + 3, z1 + 3), (x0 - 3, z1 + 3)]
        outline = hull2d(corners)
        wing = prism_xz(outline, 69.5, 72.0, 6.0)
        try:
            wing = fillet(wing, 1.2, [e for e in wing.Edges if abs(e.BoundBox.YMax - 72.0) < 1e-6
                                      and abs(e.BoundBox.YMin - 72.0) < 1e-6])
        except Exception:
            pass
        wing = wing.cut(ycyl(4.0, -9.0, -26.0, 70.5, 73.0))
        cx = sum(x for x, _ in outline) / len(outline)
        cz = sum(z for _, z in outline) / len(outline)
        inset = [(cx + (x - cx) * 0.9, cz + (z - cz) * 0.9) for x, z in outline]
        rim = prism_xz(outline, 62.5, 69.6, 6.0).cut(prism_xz(inset, 62.0, 70.0, 4.0))
        rim = rim.cut([box(x0 - 0.3, x1 + 0.3, 62.0, 69.5, z0 - 0.3, z1 + 0.3)     # servo sirtlarina girmesin
                       for (x0, x1, z0, z1) in ((-20.5, 13.5, -27.5, -7.5), (-41.8, -21.8, -49.0, -15.0))])
        wing = wing.fuse(rim).removeSplitter()
        inner = prism_xz(outline, 41.5, 43.5, 6.0).common(box(-60.0, 30.0, 40.0, 45.0, kn[2] + 7.5, 0.0))
        inner = inner.cut(ycyl(7.5, hp[0], hp[2], 40.0, 45.0)).cut(ycyl(7.5, kn[0], kn[2], 40.0, 45.0))
        out += [("thigh", "hip_pitch_servo", pitch_b, "servo", MASSES["servo"]),
                ("thigh", "knee_servo", knee_b, "servo", MASSES["servo"]),
                ("thigh", "thigh_inner_plate", inner, "bracket", None),
                ("thigh", "thigh_wing", wing, "shell", None)]

        # incik: diz horn'u -> bilek servosu. Diz plakasinin arka kenari arka plaka yuzune (x=sx0-2)
        # kadar uzar ve dairesel ucuna 45 derece teget iner: arka plaka tablada desteksiz basilir.
        ank_b, ank_h = self._servo("ankle")
        sx0, sx1 = kn[0] - 10.0, kn[0] + 10.0
        rx = sx0 - 2.0                                  # arka plakanin arka yuzu
        c45 = math.cos(math.radians(45.0))
        tk = (kn[0] - 10.0 * c45, kn[2] + 10.0 * c45)   # 135 derecedeki teget noktasi
        knee_ext = prism_xz([(rx, -54.0), (sx0, -54.0), (sx0, kn[2]), tk, (rx, tk[1] - (tk[0] - rx))], 37.5, 40.5)
        shin = stadium_y(kn[0], kn[2], 10.0, -54.0, 37.5, 40.5)
        shin = shin.fuse([knee_ext, box(sx0, sx1, 35.5, 61.5, -57.0, -54.0),
                          box(rx, sx0, 35.5, 61.5, -70.0, -54.0)]).removeSplitter()
        shin = shin.cut(ycyl(1.4, kn[0], kn[2], 36.0, 42.0))
        shin = shin.cut(self._horn_bolt_holes(knee_h, 36.0, 42.0))
        pad = rbox(sx1, sx1 + 3.0, 36.5, 60.5, -76.0, -52.0, 0.0)
        pad = fillet_axis(pad, 4.0, (1, 0, 0))
        out += [("shin", "knee_horn", knee_h, "horn", MASSES["horn"]),
                ("shin", "shin_bracket", shin, "bracket", None),
                ("shin", "shin_pad", pad, "shell", None),
                ("shin", "ankle_servo", ank_b, "servo", MASSES["servo"])]

        # ayak: bilek horn'u, tek parca U braket, ayak govdesi ve taban (egik).
        # U braket: horn kolu (y 64.5..67) + ic yanak/koruma kolu (y 32..35, servo sirtindan 0.5 mm
        # acik; rulman yok: servo arkasinda mil tasiyacak yer yok, incikten yanakla mil vermek
        # kalca pitch +-90 / diz 70 taramasinda govde kabuguna ve roll servosuna carpiyor) +
        # servonun altinda 2.5 mm taban. Servo bilek ekseni etrafinda r=12.35 supurur (z>=-93.85):
        # taban ustu z=-95, ayak govdesindeki duz tabanli cebe (taban -97.5, link cercevesinde duz)
        # oturur; 4x M2 havsa basli vida ile govdedeki isil eklere baglanir (servo takilmadan once).
        z_sole = an[2] - p["sole_below_ankle"]
        z_ft = z_sole + p["sole_t"] + p["foot_t"]
        fx0, fx1 = an[0] + p["foot_x"][0], an[0] + p["foot_x"][1]
        fyc = 49.95
        fy0, fy1 = fyc - p["foot_w"] / 2, fyc + p["foot_w"] / 2
        r_u, ub0, ub1 = 11.0, -97.5, -95.0              # kol yaricapi, taban alt/ust
        ux0, ux1 = an[0] - r_u, an[0] + r_u
        yb0, yb1, yh0, yh1 = 32.0, 35.0, 64.5, 67.0     # ic kol, horn kolu
        u = stadium_y(an[0], an[2], r_u, ub1, yh0, yh1).fuse(
            [stadium_y(an[0], an[2], r_u, ub1, yb0, yb1), box(ux0, ux1, yb0, yh1, ub0, ub1)]).removeSplitter()
        u = fillet(u, 2.5, [e for e in u.Edges if _parallel(e, (1, 0, 0)) and abs(e.BoundBox.ZMin - ub1) < 1e-6
                            and min(abs(e.BoundBox.YMin - yb1), abs(e.BoundBox.YMin - yh0)) < 1e-6])
        u = u.cut(ycyl(13.35, an[0], an[2], yb1, 62.0))           # servo supurme zarfi + 1 mm
        u = u.cut(ycyl(1.4, an[0], an[2], 63.0, 68.0))
        u = u.cut(self._horn_bolt_holes(ank_h, 63.5, 68.0))
        u_screws = [(an[0] + dx, sy) for dx in (-7.5, 7.5) for sy in (40.0, 57.0)]
        for sx, sy in u_screws:
            u = u.cut(zcyl(1.1, sx, sy, ub0 - 1.0, ub1 + 1.0))
            u = u.cut(Part.makeCone(1.1, 1.95 + 0.5, 0.85 + 0.5, V(sx, sy, ub1 - 0.85), V(0, 0, 1)))  # 90 derece havsa
        body = rbox(fx0, fx1, fy0, fy1, z_sole + p["sole_t"], z_ft, 9.0, 3.0, 0.0)
        for k, hx in enumerate((fx1 - 9.0, fx1 - 17.0)):
            body = body.cut(zcyl(2.2, hx, fyc + (6.0 if k else -6.0), z_ft - 2.0, z_ft + 1.0))
        sole = rbox(fx0, fx1, fy0, fy1, z_sole, z_sole + p["sole_t"], 9.0, 0.0, 1.5)
        c = (0.5 * (fx0 + fx1), fyc, z_sole)
        cant = p["sole_cant_deg"]
        body, sole = rotated(body, c, (1, 0, 0), cant), rotated(sole, c, (1, 0, 0), cant)
        body = body.cut(box(ux0 - 0.2, ux1 + 0.2, yb0 - 0.2, yh1 + 0.2, ub0, z_ft + 5.0))   # U braket cebi
        body = body.cut([zcyl(1.6, sx, sy, z_sole - 1.0, ub0 + 0.5) for sx, sy in u_screws])  # M2 isil ek
        ca = math.radians(cant)
        cc = V(c[0], c[1] - 0.5 * p["sole_t"] * math.sin(ca), z_sole + 0.5 * p["sole_t"] * math.cos(ca))
        cbox = (cc, ((fx1 - fx0) / 2, (fy1 - fy0) / 2, p["sole_t"] / 2), cant)
        out += [("foot", "ankle_horn", ank_h, "horn", MASSES["horn"]),
                ("foot", "ankle_bracket", u, "bracket", None),
                ("foot", "foot_body", body, "trim", None),
                ("foot", "sole", sole, "sole", None, cbox)]
        return out

    def _legs(self):
        p = self.p
        lim = JOINT_LIMITS
        axes = {"hip_yaw": (0, 0, -1), "hip_roll": (1, 0, 0), "hip_pitch": (0, 1, 0),
                "knee": (0, -1, 0), "ankle": (0, 1, 0)}
        chain = [("hip_yaw", "trunk", "hip"), ("hip_roll", "hip", "hip_cradle"),
                 ("hip_pitch", "hip_cradle", "thigh"), ("knee", "thigh", "shin"), ("ankle", "shin", "foot")]
        items = self._leg_items()
        for side, sy, ids in (("left", 1, 20), ("right", -1, 10)):
            S = side + "_"
            for k, (jn, parent, child) in enumerate(chain):
                x, y, z = p["j_" + jn]
                ax = axes[jn]
                lo, hi = lim[jn]
                if sy < 0:   # sag bacak: Y'ye gore ayna; Y eksenli eklemlerin ekseni ters (Pollen ile ayni)
                    ax = (ax[0], -ax[1], ax[2])
                    lo, hi = -hi, -lo
                pa = parent if parent == "trunk" else S + parent
                self._joint(S + jn, pa, S + child, (x, sy * y, z), ax, (lo, hi),
                            HOME_POSE.get(S + jn, 0.0), ids + k)
            for entry in items:
                link, name, shape, kind, mass = entry[:5]
                cbox = entry[5] if len(entry) > 5 else None
                if name == "hip_yaw_horn":
                    shape = self._hip_yaw_horn[side]
                elif sy < 0:
                    shape = mirror_y(shape)
                if cbox is not None and sy < 0:
                    cbox = (V(cbox[0].x, -cbox[0].y, cbox[0].z), cbox[1], -cbox[2])
                self._add(S + link, S + name, shape, kind, mass, cbox)

    # -- boyun ve kafa -----------------------------------------------------------
    def _neck_head(self):
        p = self.p
        w = p["wall"]
        np_, hp, hy, hr, mo = (p["j_neck_pitch"], p["j_head_pitch"], p["j_head_yaw"],
                               p["j_head_roll"], p["j_mouth"])

        def horn_bolts(key, length, cb=None):
            """key servosunun horn'una oturan braket icin 4 x 2.0 mm civata deligi; horn'daki
            delik dairesiyle ayni (_servo_canon: 4.2 mm yaricap, 45 derece). length: horn dis
            yuzunden itibaren delik boyu; cb = (yaricap, bas, son) istege bagli havsa (horn dis
            yuzunden uzaklik)."""
            y0 = (p["servo_d"] + p["horn_t"]) / 2 - p["horn_t"]
            y1 = y0 + p["horn_t"]
            cut = []
            for k in range(4):
                a = math.pi / 4 + k * math.pi / 2
                cx, cz = 4.2 * math.cos(a), 4.2 * math.sin(a)
                cut.append(ycyl(1.0, cx, cz, y0, y1 + length))
                if cb:
                    cut.append(ycyl(cb[0], cx, cz, y1 + cb[1], y1 + cb[2]))
            return moved(cut[0].fuse(cut[1:]), frame(*SERVOS[key]))

        # boyun: iki servo sirt sirta + ek plakalari
        self._joint("neck_pitch", "trunk", "neck", np_, (0, -1, 0), JOINT_LIMITS["neck_pitch"], HOME_POSE["neck_pitch"], 30)
        nb, nh = self._servo("neck_pitch")
        hpb, hph = self._servo("head_pitch")
        self._add("trunk", "neck_pitch_horn", nh, "horn", MASSES["horn"])
        self._add("neck", "neck_pitch_servo", nb, "servo", MASSES["servo"])
        self._add("neck", "head_pitch_servo", hpb, "servo", MASSES["servo"])
        splice = box(14.0, 16.0, -14.5, 11.5, 50.0, 64.5).fuse(box(36.0, 38.0, -14.5, 11.5, 50.0, 64.5))
        self._add("neck", "neck_splice", splice, "bracket")

        # kafa egim cercevesi (neck_pitch govdesi): kafa pitch horn'u -> kafa yaw horn'u
        self._joint("head_pitch", "neck", "head_tilt", hp, (0, 1, 0), JOINT_LIMITS["head_pitch"], HOME_POSE["head_pitch"], 31)
        hyb, hyh = self._servo("head_yaw")
        arms = stadium_y(hp[0], hp[2], 10.0, hy[2] - 1.5, 14.5, 17.0).fuse(
            stadium_y(hp[0], hp[2], 10.0, hy[2] - 1.5, -17.0, -14.5))
        tilt = arms.fuse(rbox(16.0, 36.0, -17.0, 17.0, hy[2] - 3.0, hy[2], 3.0)).removeSplitter()
        tilt = tilt.cut(ycyl(1.4, hp[0], hp[2], 13.0, 18.0)).cut(zcyl(1.4, hy[0], 0, hy[2] - 4, hy[2] + 1))
        tilt = tilt.cut(horn_bolts("head_pitch", 3.5)).cut(horn_bolts("head_yaw", 4.0))
        self._add("head_tilt", "head_pitch_horn", hph, "horn", MASSES["horn"])
        self._add("head_tilt", "head_tilt_bracket", tilt, "bracket")
        self._add("head_tilt", "head_yaw_horn", hyh, "horn", MASSES["horn"])

        # kafa donus cercevesi (yaw_roll_motion): yaw servosu + roll ekseni yataklari
        self._joint("head_yaw", "head_tilt", "head_turn", hy, (0, 0, 1), JOINT_LIMITS["head_yaw"], 0.0, 32)
        hrb, hrh = self._servo("head_roll")
        turn = box(8.1, 16.0, -24.5, 5.0, 104.5, 127.0).fuse(
            [box(36.0, 39.0, -24.5, 5.0, 104.5, 127.0), xcyl(5.0, 0.0, hr[2], 39.0, 45.0),
             box(8.1, 16.0, 4.0, 7.5, 109.0, 122.0)]).removeSplitter()          # havsa et payi (+y)
        # arka plaka roll horn'una civatali (baslar yaw servosu tarafindaki havsalarda), on plaka
        # (roll pimi) yaw servosu govdesine; iki plaka yaw servosu uzerinden birlesir
        turn = turn.cut(xcyl(1.4, 0.0, hr[2], 7.0, 12.0)).cut(horn_bolts("head_roll", 9.0, (2.1, 5.7, 9.0)))
        self._add("head_turn", "head_yaw_servo", hyb, "servo", MASSES["servo"])
        self._add("head_turn", "head_turn_bracket", turn, "bracket")
        self._add("head_turn", "head_roll_horn", hrh, "horn", MASSES["horn"])

        # kafa
        self._joint("head_roll", "head_turn", "head", hr, (-1, 0, 0), JOINT_LIMITS["head_roll"], 0.0, 33)
        hx0, hx1 = p["head_x"]
        W2 = p["head_w"] / 2
        zt, zs, zb = p["head_top"], p["head_split"], p["head_bot"]
        fx = p["face_x"]
        R, c = 22.0, 0.2                    # kaput kose yaricapi, gecme boslugu
        zl, zlip = zs - 1.0, zs + 1.0       # dikis duzlemi (kaput alt kenari), alt kabuk dudaginin ustu

        def hood_body(d, z0, z1, top=True):
            """Kaput dis formunun d kadar iceri ofseti (top: ust yuz ve ust kenarlar da)."""
            return rbox(hx0 + d, hx1 - d, -W2 + d, W2 - d, z0, z1 - (d if top else 0.0), R - d,
                        (R - d) if top else 0.0)

        # gaga ekseni delikleri: alt gaga gobegi ve gaga horn'u (ikisi de r6.5) 0.8 mm boslukla gecer (FDM'de
        # donen gobek icin 0.4 mm dar: ilk katman tasmasi/asiri ekstruzyon gagayi sikistirir). Delik altinda
        # kaput duvari dikis duzlemine 1.3 mm kalir.
        mouth_holes = [ycyl(7.3, mo[0], mo[2], *sorted((sy * (W2 - 4.5), sy * (W2 + 3.0)))) for sy in (1, -1)]
        # ust kaput: alt acik, onde yuz icin acik. Yuz acikligi ic bosluk profiliyle (ust koseler
        # R20.2) kesilir; on kenar, dis kose yuvarlaginda dudak >= 1.25 mm kalan yerde dik duzlemle biter
        x_trim = hx1 - R + math.sqrt(R ** 2 - (R - w + 1.25) ** 2)
        hood = hood_body(0.0, zl, zt).cut(hood_body(w, zl - 1.0, zt))
        fo = box(fx, hx1 + 2.0, -W2 + w, W2 - w, zl - 1.0, zt - w)
        fo = fillet(fo, R - w, [e for e in fo.Edges if _parallel(e, (1, 0, 0)) and _at_z(e, zt - w)])
        hood = hood.cut(fo).cut(box(x_trim, hx1 + 2.0, -W2 - 1.0, W2 + 1.0, zl - 1.0, zt + 1.0))
        for mh_ in mouth_holes:
            hood = hood.cut(mh_)
        self._add("head", "head_hood", hood, "shell")
        # alt kabuk (turuncu): taban + ust gaga. Arka koseler kaputla ayni (R22), gaga ucu R18.
        # Kaputla bindirme: kaputun altinda (x < xb) dis duvar dikis duzleminde (zl) biter; kaputun ic
        # yuzune c bosluklu, w kalin dudak zlip'e kadar kaputun icine girer, altindaki 45 derece pahli
        # kalinlastirma onu duvara baglar. Kaputun onunde gaga tam duvarla zlip'e cikar.
        xf, xb = hx1 - 6.0, x_trim + c

        def tray(d, z0, z1, r_bot):
            b = box(hx0 + d, xf - d, -W2 + d, W2 - d, z0, z1)
            b = fillet(b, R - d, [e for e in b.Edges if _parallel(e, (0, 0, 1)) and e.BoundBox.XMax < hx0 + d + 1e-6])
            b = fillet(b, 18.0 - d, [e for e in b.Edges if _parallel(e, (0, 0, 1)) and e.BoundBox.XMin > xf - d - 1e-6])
            return fillet(b, r_bot, [e for e in b.Edges if _at_z(e, z0)])
        outer = tray(0.0, zb, zlip, 6.0)
        low = outer.cut(tray(w, zb + w, zlip + 1.0, 6.0 - w))
        keep = box(hx0 - 1.0, xb, -W2 - 1.0, W2 + 1.0, zb, zlip + 1.0)
        band = box(hx0 - 1.0, xb, -W2 - 1.0, W2 + 1.0, zl, zlip + 1.0)
        low = low.cut(band.cut(hood_body(w + c, zl - 1.0, zlip + 2.0, False)))
        d_in = 2 * w + c
        lip_in = hood_body(d_in, zl - 5.0, zlip + 1.0, False)
        lip = hood_body(w + c, zl - 1.0, zlip, False).cut(lip_in)
        rib = hood_body(1.0, zl - 4.0, zl, False).cut(lip_in)
        lo_x, hi_x, lo_y, hi_y = hx0 + d_in - 0.05, hx1 - d_in + 0.05, -W2 + d_in - 0.05, W2 - d_in + 0.05
        rib = rib.makeChamfer(2.0, [e for e in rib.Edges if _at_z(e, zl - 4.0) and e.BoundBox.XMin > lo_x
                                    and e.BoundBox.XMax < hi_x and e.BoundBox.YMin > lo_y and e.BoundBox.YMax < hi_y])
        low = low.fuse([lip.common(keep), rib.common(keep).common(outer)]).removeSplitter()
        low = low.cut(zcyl(p["neck_hole_r"], hy[0], 0.0, zb - 1.0, zb + w + 1.0))
        low = low.cut(box(46.0, hx1 + 1, -W2 - 1, W2 + 1, zb - 1.0, 107.5))
        for mh_ in mouth_holes:
            low = low.cut(mh_)
        bill = box(46.0, hx1 + 1, -W2 - 1, W2 + 1, zb - 1.0, zlip + 1.0)
        self._add("head", "head_bottom_shell", low.cut(bill), "shell")
        self._add("head", "upper_bill", low.common(bill), "trim")
        self._add("head", "upper_bill_pad", rbox(52.0, hx1 - 7.0, -W2 + w + 0.5, W2 - w - 0.5, 106.4, 107.5, 4.0),
                  "pad")
        # yuz plakasi, goz (kamera), ToF penceresi, kayit LED'i
        ez = p["eye_z"]
        # kaputun ic yuzunu (ust koseler R20.2) 0.3 mm, alt kabuk dudagini 0.2 mm bosluk birakarak izler
        face = box(fx - 2.0, fx, -W2, W2, 107.5, zt).common(hood_body(w + 0.3, zlip + c, zt).fuse(
            hood_body(d_in + c, 107.0, zlip + c + 0.01, False)))
        face = face.cut(xcyl(9.6, 0.0, ez, fx - 3, fx + 1)).cut(rbox(fx - 3, fx + 1, 19.0, 26.0, 126.0, 132.0, 0.0))
        face = face.cut(xcyl(1.4, -20.0, 145.0, fx - 3, fx + 1))       # kayit LED'i deligi
        self._add("head", "face_plate", face, "face")
        ring = xcyl(p["eye_r"], 0.0, ez, fx, fx + 9.5).cut(xcyl(9.6, 0.0, ez, fx - 1, fx + 11))
        ring = fillet(ring, 1.5, [e for e in ring.Edges if abs(e.BoundBox.XMin - fx - 9.5) < 1e-6
                                  and abs(e.BoundBox.XMax - fx - 9.5) < 1e-6])
        self._add("head", "eye_ring", ring, "trim")
        self._add("head", "camera_lens", xcyl(9.5, 0.0, ez, fx - 2.0, fx + 6.0), "lens", MASSES["camera"])
        self._add("head", "camera_glass", xcyl(4.0, 1.5, ez + 2.0, fx + 6.0, fx + 6.3), "glass", 0.0)
        self._add("head", "camera_holder", box(fx - 4.5, fx - 2.0, -7.4, 7.4, ez - 8.0, ez + 8.0), "chip", 0.0)
        tof = rbox(fx - 2.5, fx + 0.8, 19.2, 25.8, 126.2, 131.8, 0.0)
        self._add("head", "tof_sensor", fillet_axis(tof, 2.5, (1, 0, 0)), "lens", MASSES["tof"])
        self._add("head", "rec_led", xcyl(1.3, -20.0, 145.0, fx - 1.0, fx + 0.8), "led", 0.0)
        # elektronik: kart + HAT dik, hoparlor arkada
        self._add("head", "main_board", box(fx - 7.0, fx - 5.4, -32.5, 32.5, 114.5, 144.5), "pcb", MASSES["board"])
        self._add("head", "robot_hat", box(fx - 14.0, fx - 12.4, -32.5, 32.5, 114.5, 144.5), "pcb2", MASSES["hat"])
        self._add("head", "speaker", rbox(-17.9, 7.1, -16.8, 18.2, 127.0, 133.0, 3.0), "chip", MASSES["speaker"])
        # ic cerceve (tek parca): roll servosu yuvasi, on yatak duvari, gaga servosu rafi. Yuva ile duvar
        # sagda (-y) bir rayla, solda yuva duvari -> raf -> duvar uzerinden baglanir; kapali halka
        # kafa donus/egim parcalarinin supurme alaninin (|y| < 27, eksenden r < 20) disinda kalir.
        frame_parts = [box(-22.9, 5.1, -26.5, 11.5, 103.6, 105.6),
                       box(-22.9, 3.0, -26.5, -24.5, 105.6, 125.6),
                       box(-22.9, 3.0, 9.5, 11.5, 105.6, 125.6),
                       box(-22.9, -20.9, -24.5, 9.5, 105.6, 125.6)]
        frame_parts += [zcyl(1.6, px, py, zb + w, 103.6) for px in (-19.0, -4.0) for py in (-22.0, 8.0)]
        frame_parts += [box(43.0, 46.0, -39.0, 12.0, 104.0, 128.0)]                  # on yatak duvari
        frame_parts += [box(-12.0, 5.1, -29.5, -26.0, 103.6, 105.6),                  # sag ray
                        box(-12.0, 46.0, -39.0, -29.5, 103.6, 105.6), zcyl(1.6, 40.0, -34.0, zb + w, 103.6)]
        frame_parts += [box(16.6, 50.6, 13.5, 39.5, 106.6, 108.6)]                   # gaga servosu rafi
        frame_parts += [zcyl(1.6, px, 34.0, zb + w, 106.6) for px in (20.0, 46.0)]
        frame_parts += [box(-4.0, 15.0, 10.0, 22.0, 106.6, 108.6), box(14.0, 17.0, 13.5, 22.0, 106.6, 108.6),
                        box(43.0, 46.0, 11.0, 15.0, 104.0, 108.6)]                    # yuva -> raf -> duvar
        hframe = frame_parts[0].fuse(frame_parts[1:]).removeSplitter()
        hframe = hframe.cut(xcyl(5.2, 0.0, hr[2], 42.0, 47.0))
        self._add("head", "head_frame", hframe, "bracket")
        self._add("head", "head_roll_servo", hrb, "servo", MASSES["servo"])
        mb, mh = self._servo("mouth")
        self._add("head", "mouth_servo", mb, "servo", MASSES["servo"])

        # gaga (alt cene): gaga servosu ekseninde doner, karsi tarafta pim
        self._joint("mouth", "head", "jaw", mo, (0, 1, 0), JOINT_LIMITS["mouth"], 0.0, 34)
        tip = (hx1 - 7.0, 100.0)
        jaw = rbox(52.0, tip[0], -W2 - 0.5, W2 + 0.5, 98.2, 101.6, 6.0)
        jaw = jaw.fuse(box(52.0, 62.0, -W2 - 3.5, W2 + 3.5, 98.2, 101.6))
        for sy in (1, -1):
            y0, y1 = (W2 + 1.0, W2 + 3.5) if sy > 0 else (-W2 - 3.5, -W2 - 1.0)
            jaw = jaw.fuse(bar_y((mo[0], mo[2]), (57.0, 98.2 + 5.5), 5.5, y0, y1))   # kol alt gaga altina tasmaz
            jaw = jaw.fuse(ycyl(6.5, mo[0], mo[2], y0, y1))                    # kol gobegi (civata dairesi)
        jaw = jaw.fuse(ycyl(6.5, mo[0], mo[2], mo[1] + 14.5, W2 + 1.0))       # sol: horn'a gecen gobek
        jaw = jaw.fuse(ycyl(2.5, mo[0], mo[2], -W2 - 1.0, -W2 + 3.0))         # sag: pim
        jaw = jaw.removeSplitter()
        # disaridan horn'a 4 x M2 (kol + gobek boyunca) ve merkez vidasi erisim deligi
        jaw = jaw.cut(horn_bolts("mouth", 8.0)).cut(ycyl(1.4, mo[0], mo[2], mo[1] + 13.0, W2 + 4.5))
        self._add("jaw", "mouth_horn", mh, "horn", MASSES["horn"])
        self._add("jaw", "lower_bill", jaw, "trim")
        self._add("jaw", "lower_bill_pad", rbox(53.0, tip[0] - 1.0, -W2 + 2.0, W2 - 2.0, 101.6, 106.4, 4.0), "pad")


    def _build(self):
        self._link("trunk", (0, 0, 0))
        self._trunk()
        self._legs()
        self._neck_head()
        names = list(self.links) + [it.name for l in self.links.values() for it in l.items]
        assert len(names) == len(set(names)), "link/parca adlari benzersiz olmali"
        self.meta.update({
            "robot": "OpenDuck (Microduck benzeri)",
            "servo": "DYNAMIXEL XL330-M288-T",
            "foot_links": [l for l in self.links if l.endswith("foot")],
            "action_pose": {"left_hip_pitch": 0.0, "left_knee": 49.2, "left_ankle": 49.2, "neck_pitch": 10.0,
                            "head_pitch": 35.0, "head_yaw": -30.0, "head_roll": -8.0, "mouth": 25.0},
            "collision_links": ["trunk", "head"],
            "bom_buy": [
                (15, "ROBOTIS DYNAMIXEL XL330-M288-T", "bacaklar 2x5, boyun+kafa 4, gaga 1 (ID 10-14, 20-24, 30-34)"),
                (1, "Radxa Zero 3W (RK3566) veya Raspberry Pi Zero 2 W", "kafada, dik"),
                (1, "Robot HAT (Pollen elec_RPI_Robot_HAT, Apache-2.0) veya esdegeri", "Dynamixel TTL, ses, 5 V"),
                (1, "IMX219 kamera + M12 lens", "goz"),
                (1, "VL53L5CX 8x8 ToF", "'lidar'"),
                (1, "LSM6DSV16X IMU karti", "govde"),
                (1, "NP-F550 2S Li-ion (7.4 V, 2600 mAh) + yuva konnektoru", "servolara dogrudan"),
                (1, "Hoparlor 25x35 mm, MEMS mikrofon", ""),
                (1, "M2 vida seti, isil ek somun, kablolar",
                 "bacak horn baglantilari 2x20 + boyun catali 4 vida; ayak bilegi U braketi 2x4 havsa basli vida"
                 " + 8 isil ek; batarya yuvasi 3 vida + 3 isil ek"),
            ],
        })

    # -- kinematik --------------------------------------------------------------
    def joint(self, name):
        for j in self.joints:
            if j["name"] == name:
                return j
        raise KeyError(name)

    def pose(self, name="stand"):
        if name == "zero":
            return {j["name"]: 0.0 for j in self.joints}
        return {j["name"]: float(j["stand"]) for j in self.joints}

    def link_placements(self, pose, base=None):
        base = base or App.Placement(self.links["trunk"].origin, App.Rotation())
        out = {"trunk": base}
        for j in self.joints:
            parent = self.links[j["parent"]]
            rel = App.Placement(j["origin"] - parent.origin, App.Rotation(V(*j["axis"]), pose.get(j["name"], 0.0)))
            out[j["child"]] = out[j["parent"]].multiply(rel)
        return out

    def posed_shapes(self, pose, base=None):
        pls = self.link_placements(pose, base)
        res = []
        for ln, link in self.links.items():
            for it in link.items:
                s = it.shape.copy()
                s.translate(-link.origin)
                s.Placement = pls[ln].multiply(s.Placement)
                res.append((ln, it, s))
        return res

    def ground_offset(self, pose):
        """Pozda tabanlar z=0'a gelsin diye govde orijininin dunya z konumu."""
        shapes = self.posed_shapes(pose)
        zmin = min(s.BoundBox.ZMin for ln, it, s in shapes if it.kind == "sole")
        return self.links["trunk"].origin.z - zmin

    # -- kutle ozellikleri ------------------------------------------------------
    def link_inertia(self, name):
        link = self.links[name]
        parts = []
        for it in link.items:
            m = it.mass_kg(self.p)
            if m <= 0:
                continue
            solids = it.shape.Solids or [it.shape]
            vol = sum(s.Volume for s in solids)
            for s in solids:
                ms = m * s.Volume / vol
                c = s.CenterOfMass - link.origin
                I = s.MatrixOfInertia
                k = ms / s.Volume
                Ic = [[I.A11 * k, I.A12 * k, I.A13 * k],
                      [I.A21 * k, I.A22 * k, I.A23 * k],
                      [I.A31 * k, I.A32 * k, I.A33 * k]]
                parts.append((ms, c, Ic))
        mt = sum(pm for pm, _, _ in parts)
        com = V(0, 0, 0)
        for pm, c, _ in parts:
            com += c * (pm / mt)
        It = [[0.0] * 3 for _ in range(3)]
        for pm, c, Ic in parts:
            d = c - com
            dv = (d.x, d.y, d.z)
            dd = d.x * d.x + d.y * d.y + d.z * d.z
            for a in range(3):
                for b in range(3):
                    It[a][b] += Ic[a][b] + pm * ((dd if a == b else 0.0) - dv[a] * dv[b])
        return mt, com, It

    def total_mass(self):
        return sum(it.mass_kg(self.p) for l in self.links.values() for it in l.items)


# ---------------------------------------------------------------------------
# FreeCAD belgesi
# ---------------------------------------------------------------------------
def build_document(model, pose=None, name="OpenDuck"):
    pose = pose or model.pose("stand")
    if App.GuiUp:
        import PartGui  # noqa: F401  (Part::Feature renk ozellikleri icin view provider)
    doc = App.newDocument(name)

    sheet = doc.addObject("Spreadsheet::Sheet", "Pose")
    sheet.Label = "Poz (derece)"
    for col, txt in zip("ABCDEFG", ("Eklem", "Aci", "Min", "Max", "HOME", "Servo ID", "Aciklama")):
        sheet.set(col + "1", txt)
    notes = {"hip_yaw": "kalca donus", "hip_roll": "kalca yana", "hip_pitch": "kalca one/arkaya",
             "knee": "diz (kus dizi, geriye)", "ankle": "ayak bilegi", "neck_pitch": "boyun",
             "head_pitch": "kafa egim", "head_yaw": "kafa donus", "head_roll": "kafa yatma",
             "mouth": "gaga (0 kapali, 30 acik)"}
    for i, j in enumerate(model.joints):
        row = str(i + 2)
        key = j["name"].replace("left_", "").replace("right_", "")
        sheet.set("A" + row, j["name"])
        sheet.set("B" + row, "%g" % pose.get(j["name"], 0.0))
        sheet.setAlias("B" + row, j["name"])
        sheet.set("C" + row, "%g" % j["lower"])
        sheet.set("D" + row, "%g" % j["upper"])
        sheet.set("E" + row, "%g" % j["stand"])
        sheet.set("F" + row, "%s" % (j["id"] if j["id"] is not None else ""))
        sheet.set("G" + row, notes.get(key, ""))
    row = str(len(model.joints) + 3)
    sheet.set("A" + row, "trunk_z")
    sheet.set("B" + row, "%.3f" % model.ground_offset(pose))
    sheet.setAlias("B" + row, "trunk_z")
    sheet.set("G" + row, "govde yuksekligi (mm); ayaklar zemine degsin diye")
    sheet.setColumnWidth("A", 150)
    sheet.setColumnWidth("G", 260)

    root = doc.addObject("App::Part", "OpenDuck")
    parts = {}
    for ln, link in model.links.items():
        prt = doc.addObject("App::Part", ln)
        parts[ln] = prt
        for it in link.items:
            f = doc.addObject("Part::Feature", it.name)
            s = it.shape.copy()
            s.translate(-link.origin)
            f.Shape = s
            prt.addObject(f)
    root.addObject(parts["trunk"])
    parts["trunk"].setExpression("Placement", "placement(vector(0; 0; Pose.trunk_z); rotation(vector(0; 0; 1); 0))")
    for j in model.joints:
        parent, child = parts[j["parent"]], parts[j["child"]]
        parent.addObject(child)
        d = j["origin"] - model.links[j["parent"]].origin
        child.setExpression("Placement", "placement(vector(%.4f; %.4f; %.4f); rotation(vector(%g; %g; %g); Pose.%s))"
                            % (d.x, d.y, d.z, j["axis"][0], j["axis"][1], j["axis"][2], j["name"]))
    doc.recompute()

    if App.GuiUp:
        for ln, link in model.links.items():
            for it in link.items:
                set_color(doc.getObject(it.name).ViewObject, KINDS[it.kind]["color"])
    return doc


def set_color(vo, color):
    """Mat (az parlak) malzeme; FreeCAD 1.0 ShapeAppearance, eski surumlerde ShapeColor."""
    try:
        mat = App.Material()
        mat.DiffuseColor = color
        mat.AmbientColor = tuple(0.35 * c for c in color)
        mat.SpecularColor = (0.08, 0.08, 0.08)
        mat.Shininess = 0.12
        vo.ShapeAppearance = [mat]
    except Exception:
        vo.ShapeColor = color
    vo.DisplayMode = "Shaded"


if __name__ == "__main__":
    _model = OpenDuck()
    _doc = build_document(_model)
    if App.GuiUp:
        import FreeCADGui as Gui
        Gui.activeDocument().activeView().viewIsometric()
        Gui.SendMsgToActiveView("ViewFit")
    App.Console.PrintMessage("OpenDuck hazir: %d link, %d eklem, %.3f kg\n"
                             % (len(_model.links), len(_model.joints), _model.total_mass()))
