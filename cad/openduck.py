# -*- coding: utf-8 -*-
"""
OpenDuck - Microduck (Pollen Robotics / Hugging Face) benzeri cift ayakli ordek robot. Tamamen parametrik
FreeCAD modeli; robot bir konfigden kurulur (cad/build_config.py, spec 3.2.1):
  configs/xl330_27.json : ~27 cm, 15 x DYNAMIXEL XL330-M288-T (refactor oncesi model; bayt-esit regresyon referansi)
  configs/tr35_b2.json  : OpenDuck-TR ~35 cm (scale 1.282), DS32xx / ES3054 / MG90S / SG90 PWM servolar
Secim: OPENDUCK_CONFIG (yol ya da ad) ya da OpenDuck(config=...); varsayilan xl330_27.

Eklem yerlesimi, adlari ve eksen yonleri Pollen'in Apache-2.0 lisansli MJCF'i
(pollen-robotics/microduck_rl, robot_walk.xml) ile ayni; boylece ayni yazilim ve
egitim yigini kullanilabilir. Kabuk ve braket geometrisi bu depoda sifirdan cizildi
(Pollen'in CC BY-NC-SA STL'leri kullanilmadi).

Moduller
  openduck.py     : parametreler, Item/Link/OpenDuck (eklem/parca kaydi, kinematik, kutle), FreeCAD belgesi
  servos.py       : servo turu kayit defteri (zarf, horn, kutle, aktuator sinifi) ve servo geometrisi
  build_config.py : konfig okuma/dogrulama;  geom.py : geometri yardimcilari
  trunk.py, legs.py, head.py : geometri uretecleri (konfig "generators" -> GENERATORS[ad](model, servolar))

Kullanim
  * FreeCAD GUI : Makro > Makrolar... > bu dosyayi sec > Calistir
  * Tam derleme : tools/build.sh   (FCStd, STEP, STL, URDF, MJCF, BOM, gorseller)
  * Python'dan  : import openduck; m = openduck.OpenDuck()

Koordinatlar: mm; govde (trunk) cercevesi: X ileri, Y sol, Z yukari. Sifir pozda butun
eklem cerceveleri dunya eksenlerine paraleldir (uyluk 58 derece geriye egik, incik dik).
Robotun pozu "Pose" hesap tablosundaki acilarla (derece) suruluyor. Isaret kurali (spec 3.2.1): pozitif eklem
acisi = servo govdesine gore horn'un, horn'a bakarken saat yonunun tersine donmesi (servo_signs ile denetlenir).
"""
from collections import OrderedDict

import FreeCAD as App

import build_config
import servos
from geom import (V, _at_z, _parallel, bar_y, box, fillet, fillet_axis, frame, hull2d, mirror_y,  # noqa: F401
                  moved, prism_xz, rbox, rotated, shell_rbox, stadium_y, xcyl, ycyl, zcyl)
import head
import legs
import trunk

# ---------------------------------------------------------------------------
# PARAMETRELER (mm, kg) - 27 cm temel yerlesim. LAYOUT_KEYS konfigdeki scale ile carpilir; konfig "params"
# sonra mutlak deger olarak yazilir. Servo olculeri servos.py'de (tur basina).
# ---------------------------------------------------------------------------
PARAMS = {
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
    # Servo cerceveleri (sol taraf + orta). servo_face: horn dis yuzunun mil eksenindeki noktasi (braket
    # datumu; servo turunden bagimsiz). servo_dir: (horn yonu, uzak uc yonu). Kanonik servo cercevesi (servos.py):
    # orijin = yuz - horn yonu x servos.horn_face_y(tur); XL330'da 14.5 mm (26 + 3 mm yiginin ortasi).
    "servo_face": {
        "hip_yaw": (6.0, 17.5, -3.0), "hip_roll": (20.5, 17.5, -17.5), "hip_pitch": (4.0, 40.5, -17.5),
        "knee": (-31.8, 40.5, -39.5), "ankle": (-31.8, 64.5, -81.5), "neck_pitch": (26.0, 14.5, 32.4),
        "head_pitch": (26.0, 14.5, 82.4), "head_yaw": (26.0, 0.0, 101.1), "head_roll": (8.1, 0.0, 115.6),
        "mouth": (26.1, 42.5, 118.6),
    },
    "servo_dir": {
        "hip_yaw": ((0, 0, -1), (-1, 0, 0)), "hip_roll": ((1, 0, 0), (0, 0, -1)),
        "hip_pitch": ((0, -1, 0), (-1, 0, 0)), "knee": ((0, -1, 0), (0, 0, 1)), "ankle": ((0, 1, 0), (0, 0, 1)),
        "neck_pitch": ((0, 1, 0), (0, 0, 1)), "head_pitch": ((0, 1, 0), (0, 0, -1)),
        "head_yaw": ((0, 0, -1), (0, -1, 0)), "head_roll": ((1, 0, 0), (0, -1, 0)), "mouth": ((0, 1, 0), (1, 0, 0)),
    },
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
# Olcekle buyuyen yerlesim anahtarlari (kinematik + stil kutulari). Digerleri (et/plaka kalinligi, taban
# kalinligi, satin alinan parca olculeri, yogunluk, aci) mutlaktir.
LAYOUT_KEYS = ("j_hip_yaw", "j_hip_roll", "j_hip_pitch", "j_knee", "j_ankle", "j_neck_pitch", "j_head_pitch",
               "j_head_yaw", "j_head_roll", "j_mouth", "servo_face", "sole_below_ankle", "torso_x", "torso_w",
               "torso_z", "head_x", "head_w", "head_top", "head_split", "head_bot", "face_x", "eye_r", "eye_z",
               "neck_hole_r", "foot_x", "foot_w")

MASSES = {           # satin alinan parcalar (kg); servo ve horn kutleleri tur basina servos.py'de
    "battery": 0.072, "board": 0.010, "hat": 0.008, "camera": 0.004, "speaker": 0.006,
    "tof": 0.001, "imu": 0.002, "power": 0.008,
}

# Eklem sinirlari (derece; sol bacak, sag bacakta aynalanir). Pollen'in mekanik sinirlarindan
# (diz/bilek/kafa pitch +-90, kafa roll +-25, boyun -90..60) dar: cad/check.py ile tek eklem
# taramasinda carpismasiz kalan araliklar. Konfig "joint_limits_deg" ustune yazar.
JOINT_LIMITS = {
    "hip_yaw": (-20.0, 30.0), "hip_roll": (-20.0, 20.0), "hip_pitch": (-90.0, 90.0),
    "knee": (-90.0, 70.0), "ankle": (-80.0, 80.0),
    "neck_pitch": (-90.0, 40.0), "head_pitch": (-30.0, 90.0), "head_yaw": (-170.0, 170.0),
    "head_roll": (-12.0, 12.0), "mouth": (0.0, 30.0),
}

# HOME ("stand") pozu, derece. Sag bacak eksenleri aynali oldugu icin isaretler ters. Konfig "home_deg" ustune yazar.
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

# Uretec asamalari (sira: trunk -> legs -> head; eklem sirasi: sol bacak, sag bacak, boyun, kafa x3, gaga)
STAGES = (("trunk", trunk), ("legs", legs), ("head", head))
_STAGE_PLAN = {"trunk": "P2b", "legs": "P2a", "head": "P2c"}


def joint_key(name):
    """Eklem adi -> servo/eklem anahtari (left_/right_ oneki atilir)."""
    for pre in ("left_", "right_"):
        if name.startswith(pre):
            return name[len(pre):]
    return name


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
    """Konfigden kurulan robot. config: ad, yol ya da sozluk (None -> $OPENDUCK_CONFIG ya da xl330_27);
    params: en son yazilan mutlak PARAMS degerleri. Konfigin uretecleri yoksa NotImplementedError."""

    def __init__(self, params=None, config=None):
        self.cfg = build_config.load(config)
        s = float(self.cfg["scale"])
        p = build_config.merge(PARAMS, {k: build_config.scaled(PARAMS[k], s) for k in LAYOUT_KEYS})
        p = build_config.merge(build_config.merge(p, self.cfg["params"]), params or {})
        p["scale"] = s
        self.p = p
        self.servo_map = OrderedDict((k, servos.get(self.cfg["servo_map"][k])) for k in servos.JOINT_KEYS)
        self.limits = dict(JOINT_LIMITS)
        self.limits.update({k: tuple(v) for k, v in self.cfg["joint_limits_deg"].items()})
        self.home = dict(HOME_POSE)
        self.home.update(self.cfg["home_deg"])
        self.masses = dict(MASSES)
        self.links = OrderedDict()
        self.joints = []
        self.meta = {}
        self.shared = {}             # uretecler arasi paylasilan sekiller (or. trunk -> legs kalca yaw horn'u)
        self._servo_cache = {}
        self._build()

    # -- kayit -----------------------------------------------------------------
    def _link(self, name, origin):
        self.links[name] = Link(name, V(*origin))
        return self.links[name]

    def _add(self, link, name, shape, kind, mass=None, collision_box=None):
        self.links[link].items.append(Item(name, shape, kind, mass, collision_box))

    def _joint(self, name, parent, child, origin, axis, limits, home=0.0, servo_id=None):
        """Eklem + cocuk link. Servo turu, aktuator sinifi, effort/velocity eklem anahtarinin turunden. Sinirlar
        servo hareket araligi icinde olmali: merkez = neutral + horn ofseti (konfig, sol taraf; sagda ters)."""
        key = joint_key(name)
        t = self.servo_map[key]
        a = servos.actuator(t)
        lo, hi = limits
        sgn = -1.0 if name.startswith("right_") else 1.0
        neutral = sgn * float(self.cfg["neutral_deg"].get(key, 0.0))
        horn_off = sgn * float(self.cfg["horn_offset_deg"].get(key, 0.0))
        tlo, thi = servos.travel_range(t, neutral + horn_off)
        assert tlo - 1e-9 <= lo <= hi <= thi + 1e-9, "%s: sinir [%g, %g] %s hareket araligi [%g, %g] disinda" % (
            name, lo, hi, t["name"], tlo, thi)
        self.joints.append({
            "name": name, "parent": parent, "child": child, "origin": V(*origin), "axis": axis,
            "lower": lo, "upper": hi, "stand": home, "id": servo_id, "actuator": a["class"],
            "effort": a["forcerange"], "velocity": a["velocity"],
            "servo": t["name"], "neutral": neutral, "horn_offset": horn_off,
        })
        self._link(child, origin)

    # -- servolar ----------------------------------------------------------------
    def servo_type(self, key):
        return self.servo_map[key]

    def servo_mass(self, key):
        return servos.mass_kg(self.servo_map[key])

    def horn_mass(self, key):
        return servos.horn_mass_kg(self.servo_map[key])

    def servo_frame(self, key):
        """(orijin, horn yonu, uzak uc yonu): sol taraf + orta kanonik servo cercevesi. Orijin, horn dis yuzu
        noktasindan (p["servo_face"]) turun kanonik horn yuzu ofseti kadar horn yonunun tersinde."""
        face = self.p["servo_face"][key]
        h, f = self.p["servo_dir"][key]
        fy = servos.horn_face_y(self.servo_map[key])
        return tuple(face[i] - h[i] * fy for i in range(3)), h, f

    def _servo(self, key, mirror=False):
        """key servosunun (govde, horn) katilari dunya cercevesinde (sifir poz); mirror: sag taraf."""
        t = self.servo_map[key]
        if t["name"] not in self._servo_cache:
            self._servo_cache[t["name"]] = servos.canon(t)
        body, horn = self._servo_cache[t["name"]]
        pl = frame(*self.servo_frame(key))
        b, hn = moved(body, pl), moved(horn, pl)
        return (mirror_y(b), mirror_y(hn)) if mirror else (b, hn)

    def servo_signs(self):
        """Eklem basina servo isareti: eksen . horn yonu (sagda horn aynali), servo govdesi cocuk link'teyse -1 ile
        carpilir. Spec 3.2.1 kurali: hepsi +1 (pozitif aci = horn'a bakarken CCW). Govde parcasi
        "[left_|right_]<anahtar>_servo" adini tasimali."""
        where = {it.name: ln for ln, link in self.links.items() for it in link.items}
        out = OrderedDict()
        for j in self.joints:
            key = joint_key(j["name"])
            h = self.p["servo_dir"][key][0]
            if j["name"].startswith("right_"):
                h = (h[0], -h[1], h[2])
            body = where.get(j["name"] + "_servo")
            s = sum(a * b for a, b in zip(j["axis"], h))
            out[j["name"]] = None if body is None else (-s if body == j["child"] else s)
        return out

    def _servo_label(self):
        """Ozet/BOM servo metni: tek tur -> etiketi, karisik -> "n x etiket" listesi."""
        used = OrderedDict()
        for j in self.joints:
            used[j["servo"]] = used.get(j["servo"], 0) + 1
        if len(used) == 1:
            return servos.get(next(iter(used)))["label"]
        return ", ".join("%d x %s" % (n, servos.get(k)["label"]) for k, n in used.items())

    def _build(self):
        cfg = self.cfg
        gens, missing = [], []
        for stage, mod in STAGES:
            gname = cfg["generators"][stage]
            fn = mod.GENERATORS.get(gname)
            gens.append(fn)
            if fn is None:
                missing.append("%s.GENERATORS[%r] (plan %s; mevcut: %s)"
                               % (stage, gname, _STAGE_PLAN[stage], ", ".join(sorted(mod.GENERATORS))))
        if missing:
            raise NotImplementedError("konfig %s: CAD geometri uretecleri henuz yok: %s"
                                      % (cfg["name"], "; ".join(missing)))
        self._link("trunk", (0, 0, 0))
        for fn in gens:
            fn(self, self.servo_map)
        names = list(self.links) + [it.name for l in self.links.values() for it in l.items]
        assert len(names) == len(set(names)), "link/parca adlari benzersiz olmali"
        bad = {n: s for n, s in self.servo_signs().items() if s != 1}
        assert not bad, "servo isaret kurali (+1: horn'a bakarken CCW) bozuk: %s" % bad
        self.meta.update({
            "robot": cfg["robot"],
            "servo": self._servo_label(),
            "config": cfg["name"],
            "foot_links": [l for l in self.links if l.endswith("foot")],
            "action_pose": {"left_hip_pitch": 0.0, "left_knee": 49.2, "left_ankle": 49.2, "neck_pitch": 10.0,
                            "head_pitch": 35.0, "head_yaw": -30.0, "head_roll": -8.0, "mouth": 25.0},
            "collision_links": ["trunk", "head"],
            "bom_buy": [tuple(r) for r in cfg["bom_buy"]],
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
