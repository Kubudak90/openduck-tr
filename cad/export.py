# -*- coding: utf-8 -*-
"""
OpenDuck disa aktarma: FCStd, STEP, baski STL'leri, URDF + MuJoCo MJCF, BOM, ozet ve gorseller.
tools/build_gui.py tarafindan cagrilir.
"""
import json
import math
import os
from collections import OrderedDict

import FreeCAD as App
import MeshPart

import openduck
import servos
from openduck import PRINT_KINDS, V


def _local(link, item):
    s = item.shape.copy()
    s.translate(-link.origin)
    return s


def _write_stl(shape, path, linear, angular):
    mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=linear, AngularDeflection=angular, Relative=False)
    mesh.write(path)
    return mesh.CountFacets


# ---------------------------------------------------------------------------
def save_fcstd(doc, path):
    doc.saveAs(path)


def export_step(doc, path):
    root = doc.getObject("OpenDuck")
    if App.GuiUp:
        import ImportGui
        ImportGui.export([root], path)
    else:
        import Import
        Import.export([root], path)


def export_print_stls(model, out_dir):
    """Baski parcalari: link cercevesinde, mm. Sol/sag ayri dosyalar."""
    os.makedirs(out_dir, exist_ok=True)
    files = []
    for link in model.links.values():
        for it in link.items:
            if it.kind in PRINT_KINDS:
                fn = it.name + ".stl"
                _write_stl(_local(link, it), os.path.join(out_dir, fn), 0.05, 0.2)
                files.append(fn)
    return files


# ---------------------------------------------------------------------------
# URDF / MJCF
# ---------------------------------------------------------------------------
def _f(v):
    return "%.6g" % v


def _vec_m(v):
    return " ".join(_f(c / 1000.0) for c in (v.x, v.y, v.z))


def _quat_x(angle_deg):
    a = math.radians(angle_deg) / 2.0
    return (math.cos(a), math.sin(a), 0.0, 0.0)


def _link_data(model):
    """Her link icin kutle, COM (m), eylemsizlik (kg m^2) ve carpisma kutusu (m)."""
    data = OrderedDict()
    for name, link in model.links.items():
        m, com, I = model.link_inertia(name)
        boxed = [it for it in link.items if it.collision_box is not None]
        if boxed:
            c, half, ang = boxed[0].collision_box
            c = c - link.origin
            center, halfm, angle = V(c.x / 1000.0, c.y / 1000.0, c.z / 1000.0), tuple(h / 1000.0 for h in half), ang
        else:
            bb = None
            for it in link.items:
                b = _local(link, it).BoundBox
                if bb is None:
                    bb = b
                else:
                    bb.add(b)
            center = V(bb.Center.x / 1000.0, bb.Center.y / 1000.0, bb.Center.z / 1000.0)
            halfm, angle = (bb.XLength / 2000.0, bb.YLength / 2000.0, bb.ZLength / 2000.0), 0.0
        data[name] = {"mass": m, "com": V(com.x / 1000.0, com.y / 1000.0, com.z / 1000.0),
                      "I": [[c * 1e-6 for c in row] for row in I],
                      "box_center": center, "box_half": halfm, "box_angle_x": angle}
    return data


def export_robot(model, out_dir):
    """sim/ altina meshes/*.stl, openduck.urdf, openduck.xml (MJCF) ve scene.xml yazar."""
    mesh_dir = os.path.join(out_dir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    for link in model.links.values():
        for it in link.items:
            if it.shape.Volume > 0:
                _write_stl(_local(link, it), os.path.join(mesh_dir, it.name + ".stl"), 0.15, 0.4)
    data = _link_data(model)
    stand = model.pose("stand")
    base_z = model.ground_offset(stand) / 1000.0 + 0.001
    _write_urdf(model, data, os.path.join(out_dir, "openduck.urdf"))
    _write_mjcf(model, data, stand, base_z, os.path.join(out_dir, "openduck.xml"))
    _write_scene(os.path.join(out_dir, "scene.xml"))
    return data


def _write_urdf(model, data, path):
    L = ['<?xml version="1.0"?>',
         '<!-- OpenDuck - cad/openduck.py tarafindan uretildi; elle duzenlemeyin. -->',
         '<robot name="openduck">',
         '  <mujoco><compiler meshdir="meshes" strippath="true" discardvisual="false" balanceinertia="true"/></mujoco>']
    for kind, k in openduck.material_kinds(model):
        L.append('  <material name="%s"><color rgba="%s 1"/></material>' % (kind, " ".join(_f(c) for c in k["color"])))
    for name, link in model.links.items():
        d = data[name]
        I = d["I"]
        L.append('  <link name="%s">' % name)
        L.append('    <inertial>')
        L.append('      <origin xyz="%s" rpy="0 0 0"/>' % " ".join(_f(c) for c in (d["com"].x, d["com"].y, d["com"].z)))
        L.append('      <mass value="%s"/>' % _f(d["mass"]))
        L.append('      <inertia ixx="%s" ixy="%s" ixz="%s" iyy="%s" iyz="%s" izz="%s"/>'
                 % tuple(_f(x) for x in (I[0][0], I[0][1], I[0][2], I[1][1], I[1][2], I[2][2])))
        L.append('    </inertial>')
        for it in link.items:
            if it.shape.Volume <= 0:
                continue
            L.append('    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry>'
                     '<mesh filename="meshes/%s.stl" scale="0.001 0.001 0.001"/></geometry>'
                     '<material name="%s"/></visual>' % (it.name, it.kind))
        c, h = d["box_center"], d["box_half"]
        L.append('    <collision><origin xyz="%s %s %s" rpy="%s 0 0"/><geometry><box size="%s %s %s"/></geometry></collision>'
                 % (_f(c.x), _f(c.y), _f(c.z), _f(math.radians(d["box_angle_x"])), _f(2 * h[0]), _f(2 * h[1]), _f(2 * h[2])))
        L.append('  </link>')
    for j in model.joints:
        d = j["origin"] - model.links[j["parent"]].origin
        L.append('  <joint name="%s" type="revolute">' % j["name"])
        L.append('    <origin xyz="%s" rpy="0 0 0"/>' % _vec_m(d))
        L.append('    <parent link="%s"/><child link="%s"/>' % (j["parent"], j["child"]))
        L.append('    <axis xyz="%d %d %d"/>' % j["axis"])
        L.append('    <limit lower="%s" upper="%s" effort="%s" velocity="%s"/>'
                 % (_f(math.radians(j["lower"])), _f(math.radians(j["upper"])), _f(j["effort"]), _f(j["velocity"])))
        L.append('  </joint>')
    L.append('</robot>')
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


def actuator_classes(model):
    """MJCF aktuator siniflari {sinif: servos aktuator kaydi}, eklem sirasinda ilk gorulus sirasiyla (servo turu
    kayit defterinden: XL330 -> kok "openduck", DS32XX -> "ds32xx", ES3054, MG90S, SG90)."""
    out = OrderedDict()
    for j in model.joints:
        a = servos.actuator(servos.get(j["servo"]))
        prev = out.setdefault(a["mjcf_class"], a)
        assert prev is a, "MJCF sinifi %s iki farkli aktuator kaydiyla" % a["mjcf_class"]
    return out


def _class_lines(a, sp):
    return ['%s<joint damping="%s" armature="%s" frictionloss="%s"/>' % (sp, _f(a["damping"]), _f(a["armature"]),
                                                                        _f(a["frictionloss"])),
            '%s<position kp="%s" forcerange="-%s %s"/>' % (sp, _f(a["kp"]), _f(a["forcerange"]), _f(a["forcerange"]))]


def _default_lines(classes):
    """<default> blogu: kok "openduck" (trunk childclass) + visual/collision; kok sinifi kullanan aktuator varsa
    degerleri dogrudan kokte (27 cm XL330 duzeni, bayt-esit), digerleri kokun altinda kendi siniflarinda."""
    root = servos.ROOT_CLASS
    L = ['  <default>', '    <default class="%s">' % root]
    if root in classes:
        L += _class_lines(classes[root], '      ')
    L += ['      <default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/>'
          '</default>',
          '      <default class="collision"><geom type="box" contype="0" conaffinity="1" group="3" density="0"'
          ' friction="0.9 0.02 0.01" rgba="1 0 0 0.3"/></default>']
    for name, a in classes.items():
        if name != root:
            L += ['      <default class="%s">' % name] + _class_lines(a, '        ') + ['      </default>']
    return L + ['    </default>', '  </default>']


def _write_mjcf(model, data, stand, base_z, path):
    kids = {}
    for j in model.joints:
        kids.setdefault(j["parent"], []).append(j)
    classes = actuator_classes(model)
    jclass = {j["name"]: servos.actuator(servos.get(j["servo"]))["mjcf_class"] for j in model.joints}
    collide = set(model.meta.get("collision_links", [])) | set(model.meta.get("foot_links", []))

    L = ['<!-- OpenDuck - cad/openduck.py tarafindan uretildi; elle duzenlemeyin. -->',
         '<mujoco model="openduck">',
         '  <compiler angle="radian" meshdir="meshes" autolimits="true" inertiafromgeom="false"/>',
         '  <option timestep="0.002" integrator="implicitfast"/>']
    L += _default_lines(classes)
    L.append('  <asset>')
    for kind, k in openduck.material_kinds(model):
        L.append('    <material name="%s" rgba="%s 1"/>' % (kind, " ".join(_f(c) for c in k["color"])))
    for link in model.links.values():
        for it in link.items:
            if it.shape.Volume > 0:
                L.append('    <mesh name="%s" file="%s.stl" scale="0.001 0.001 0.001"/>' % (it.name, it.name))
    L.append('  </asset>')
    L.append('  <worldbody>')

    def body(name, pos, joint, indent):
        sp = "  " * indent
        d = data[name]
        I = d["I"]
        attrs = ' childclass="openduck"' if joint is None else ""
        L.append('%s<body name="%s" pos="%s"%s>' % (sp, name, pos, attrs))
        if joint is None:
            L.append('%s  <freejoint name="root"/>' % sp)
        else:   # kok sinifi childclass'tan miras alinir; diger aktuator siniflari eklemde acikca
            jc = jclass[joint["name"]]
            cls = "" if jc == servos.ROOT_CLASS else ' class="%s"' % jc
            L.append('%s  <joint name="%s"%s axis="%d %d %d" range="%s %s"/>'
                     % ((sp, joint["name"], cls) + tuple(joint["axis"]) + (_f(math.radians(joint["lower"])),
                                                                          _f(math.radians(joint["upper"])))))
        L.append('%s  <inertial pos="%s %s %s" mass="%s" fullinertia="%s %s %s %s %s %s"/>'
                 % (sp, _f(d["com"].x), _f(d["com"].y), _f(d["com"].z), _f(d["mass"]),
                    _f(I[0][0]), _f(I[1][1]), _f(I[2][2]), _f(I[0][1]), _f(I[0][2]), _f(I[1][2])))
        for it in model.links[name].items:
            if it.shape.Volume > 0:
                L.append('%s  <geom class="visual" mesh="%s" material="%s"/>' % (sp, it.name, it.kind))
        c, h = d["box_center"], d["box_half"]
        if name in collide:
            q = _quat_x(d["box_angle_x"])
            L.append('%s  <geom name="%s_collision" class="collision" pos="%s %s %s" quat="%s" size="%s %s %s"/>'
                     % (sp, name, _f(c.x), _f(c.y), _f(c.z), " ".join(_f(x) for x in q), _f(h[0]), _f(h[1]), _f(h[2])))
        # parca adli carpisma kutulari (meta["collision_geoms"]: Item adlari; TR-35 cene/alin tamponu "neck_bumper",
        # spec 3.1); meta'da yoksa (27 cm) hicbir sey yazilmaz
        for it in model.links[name].items:
            if it.name in model.meta.get("collision_geoms", ()):
                gc, gh, ga = it.collision_box
                gc = gc - model.links[name].origin
                L.append('%s  <geom name="%s" class="collision" pos="%s" quat="%s" size="%s"/>'
                         % (sp, it.name, _vec_m(gc), " ".join(_f(x) for x in _quat_x(ga)),
                            " ".join(_f(v / 1000.0) for v in gh)))
        if name == "trunk":
            imu = [it for it in model.links["trunk"].items if it.name == model.meta["imu_item"]][0]
            ic = imu.shape.BoundBox.Center - model.links["trunk"].origin
            L.append('%s  <site name="imu" pos="%s"/>' % (sp, _vec_m(ic)))
        if name in model.meta.get("foot_links", []):
            ang = math.radians(d["box_angle_x"])
            bottom = (c.x, c.y + h[2] * math.sin(ang), c.z - h[2] * math.cos(ang))
            L.append('%s  <site name="%s_sole" pos="%s" quat="%s"/>'
                     % (sp, name, " ".join(_f(x) for x in bottom), " ".join(_f(x) for x in _quat_x(d["box_angle_x"]))))
        for j in kids.get(name, []):
            dp = j["origin"] - model.links[name].origin
            body(j["child"], _vec_m(dp), j, indent + 1)
        L.append('%s</body>' % sp)

    body("trunk", "0 0 %s" % _f(base_z), None, 2)
    L.append('  </worldbody>')
    L.append('  <actuator>')
    for j in model.joints:
        L.append('    <position name="%s" joint="%s" class="%s" ctrlrange="%s %s"/>'
                 % (j["name"], j["name"], jclass[j["name"]], _f(math.radians(j["lower"])),
                    _f(math.radians(j["upper"]))))
    L.append('  </actuator>')
    L.append('  <sensor>')
    L.append('    <gyro name="gyro" site="imu"/>')
    L.append('    <accelerometer name="accelerometer" site="imu"/>')
    L.append('    <framequat name="orientation" objtype="site" objname="imu"/>')
    L.append('  </sensor>')
    q = " ".join(_f(math.radians(stand[j["name"]])) for j in model.joints)
    L.append('  <keyframe>')
    L.append('    <key name="stand" qpos="0 0 %s 1 0 0 0 %s" ctrl="%s"/>' % (_f(base_z), q, q))
    L.append('  </keyframe>')
    L.append('</mujoco>')
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


def _write_scene(path):
    xml = """<mujoco model="openduck scene">
  <include file="openduck.xml"/>
  <statistic center="0 0 0.14" extent="0.35"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <quality shadowsize="4096" offsamples="8"/>
    <global azimuth="150" elevation="-20" offwidth="1600" offheight="1200"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.85 0.88 0.92" rgb2="0.98 0.98 0.98" width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.84 0.86" rgb2="0.9 0.91 0.92" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="20 20" reflectance="0"/>
  </asset>
  <worldbody>
    <light pos="0.25 -0.4 1.2" dir="-0.18 0.32 -1" cutoff="65" exponent="1" diffuse="0.7 0.7 0.7"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="grid" contype="1" conaffinity="1"/>
  </worldbody>
</mujoco>
"""
    with open(path, "w") as f:
        f.write(xml)


# ---------------------------------------------------------------------------
# BOM ve ozet
# ---------------------------------------------------------------------------
def summary(model):
    stand, zero = model.pose("stand"), model.pose("zero")

    def extents(pose):
        sh = model.posed_shapes(pose)
        bb = None
        for _, _, s in sh:
            if bb is None:
                bb = s.BoundBox
            else:
                bb.add(s.BoundBox)
        return bb

    bb = extents(stand)
    bz = extents(zero)
    printed = [it for l in model.links.values() for it in l.items if it.kind in PRINT_KINDS]
    servos = sum(1 for l in model.links.values() for it in l.items if it.kind == "servo")
    return OrderedDict([
        ("robot", model.meta["robot"]),
        ("mass_kg", round(model.total_mass(), 3)),
        ("printed_mass_kg", round(sum(it.mass_kg(model.p) for it in printed), 3)),
        ("height_stand_mm", round(bb.ZLength, 1)),
        ("height_zero_pose_mm", round(bz.ZLength, 1)),
        ("size_stand_mm", [round(bb.XLength, 1), round(bb.YLength, 1), round(bb.ZLength, 1)]),
        ("dof_total", len(model.joints)),
        ("servo_count", servos),
        ("servo_model", model.meta["servo"]),
        ("printed_parts", len(printed)),
        ("links", len(model.links)),
        ("trunk_height_stand_mm", round(model.ground_offset(stand), 1)),
    ])


def write_bom(model, path, summ):
    from docs import describe, material, color_name    # baski malzemesi tek kaynaktan: docs.PRINT_SETTINGS
    p = model.p
    groups = OrderedDict()
    for link in model.links.values():
        for it in link.items:
            if it.kind in PRINT_KINDS:
                g = groups.setdefault(it.part, {"n": 0, "kind": it.kind, "vol": 0.0, "mass": 0.0, "files": [],
                                                "pieces": 0})
                g["n"] += 1
                g["pieces"] += max(len(it.shape.Solids), 1)
                g["vol"] += it.shape.Volume
                g["mass"] += it.mass_kg(p)
                g["files"].append(it.name + ".stl")
    L = ["# OpenDuck - Malzeme Listesi (BOM)", "",
         "> `tools/build.sh` tarafindan modelden otomatik uretildi.", "",
         "## Ozet", "",
         "| | |", "|---|---|",
         "| Robot | %s |" % summ["robot"],
         "| Toplam kutle (tahmini, kablo/vida haric) | %.0f g |" % (summ["mass_kg"] * 1000),
         "| Baski parcalarinin kutlesi | %.0f g |" % (summ["printed_mass_kg"] * 1000),
         "| Boy (HOME pozu) | %.0f mm |" % summ["height_stand_mm"],
         "| Serbestlik derecesi | %d (%d x %s) |" % (summ["dof_total"], summ["servo_count"], summ["servo_model"]), "",
         "## Satin alinacak parcalar", "",
         "| Adet | Parca | Not |", "|---:|---|---|"]
    for n, part, note in model.meta["bom_buy"]:
        L.append("| %d | %s | %s |" % (n, part, note))
    L += ["", "## 3B baski parcalari", "",
          "Malzeme ve renk onerileri `print/PRINT.md` ile aynidir. Bir model parcasi birden fazla ayrik katidan",
          "olusuyorsa (or. iki plaka) `Basilacak parca` sutunu ayri basilan parca sayisini verir; baskiya hazir",
          "STL'ler `print/stl/` altindadir. Buradaki STL sutunu montaj (link) cercevesindeki `stl/` dosyalaridir.", "",
          "| Parca | Aciklama | Model adedi | Basilacak parca | Malzeme | Renk | Toplam hacim (cm3) | "
          "Toplam kutle (g) | STL (montaj cercevesi) |",
          "|---|---|---:|---:|---|---|---:|---:|---|"]
    for name, g in groups.items():
        L.append("| %s | %s | %d | %d | %s | %s | %.1f | %.0f | %s |" % (
            name, describe(name), g["n"], g["pieces"], material(g["kind"]), color_name(g["kind"]), g["vol"] / 1000.0,
            g["mass"] * 1000, ", ".join("`%s`" % f for f in g["files"])))
    L.append("| **Toplam** | | %d | %d | | | %.1f | %.0f | |" % (
        sum(g["n"] for g in groups.values()), sum(g["pieces"] for g in groups.values()),
        sum(g["vol"] for g in groups.values()) / 1000.0, sum(g["mass"] for g in groups.values()) * 1000))
    L += ["", "Yogunluk varsayimi: PLA %.2f g/cm3, TPU %.2f g/cm3." % (p["pla_density"] * 1e6, p["tpu_density"] * 1e6), ""]
    with open(path, "w") as f:
        f.write("\n".join(L))


# ---------------------------------------------------------------------------
# Gorseller (yalnizca GUI)
# ---------------------------------------------------------------------------
def _camera_rotation(direction):
    d = V(*direction).normalize()
    zc = d * -1
    xc = V(0, 0, 1).cross(zc).normalize()
    yc = zc.cross(xc)
    m = App.Matrix(xc.x, yc.x, zc.x, 0, xc.y, yc.y, zc.y, 0, xc.z, yc.z, zc.z, 0, 0, 0, 0, 1)
    return App.Rotation(m)


def _set_pose(doc, model, pose):
    sheet = doc.getObject("Pose")
    for k, v in pose.items():
        sheet.set(k, "%g" % v)
    sheet.set("trunk_z", "%.3f" % model.ground_offset(pose))
    doc.recompute()


def render_views(doc, model, out_dir, size=(1200, 900), ss=2):
    import FreeCADGui as Gui
    from PIL import Image

    os.makedirs(out_dir, exist_ok=True)
    gdoc = Gui.getDocument(doc.Name)
    view = gdoc.ActiveView if gdoc.ActiveView else Gui.ActiveDocument.ActiveView
    view.setCameraType("Orthographic")
    shells = [it.name for l in model.links.values() for it in l.items if it.kind in ("shell", "trim", "face")]
    stand = model.pose("stand")
    action = dict(stand)
    action.update(model.meta.get("action_pose", {}))
    hero = (-1.0, 0.85, -0.38)
    shots = [
        ("hero", stand, hero, False),
        ("front", stand, (-1.0, 0.08, -0.06), False),
        ("side", stand, (-0.06, 1.0, -0.06), False),
        ("back", stand, (1.0, 0.85, -0.45), False),
        ("xray", stand, hero, True),
        ("action", action, (-0.9, 0.95, -0.35), False),
    ]
    files = []
    for name, pose, direction, xray in shots:
        _set_pose(doc, model, pose)
        for n in shells:
            doc.getObject(n).ViewObject.Transparency = 75 if xray else 0
        view.setCameraOrientation(_camera_rotation(direction).Q)
        view.fitAll()
        Gui.updateGui()
        raw = os.path.join(out_dir, "_raw_%s.png" % name)
        view.saveImage(raw, size[0] * ss, size[1] * ss, "Transparent")
        img = Image.open(raw).convert("RGBA").resize(size, Image.LANCZOS)
        bg = Image.new("RGBA", size, (244, 245, 247, 255))
        bg.alpha_composite(img)
        fn = os.path.join(out_dir, "%s.png" % name)
        bg.convert("RGB").save(fn, optimize=True)
        os.remove(raw)
        files.append(fn)
    for n in shells:
        doc.getObject(n).ViewObject.Transparency = 0
    _set_pose(doc, model, stand)
    view.setCameraOrientation(_camera_rotation(hero).Q)
    view.fitAll()
    return files


def write_json(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)
        f.write("\n")
