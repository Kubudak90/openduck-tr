# -*- coding: utf-8 -*-
"""
TR-35 bacaklar (P2a), freecadcmd ile (yoksa atlanir): bacak link'lerinde ayni link ici cakisma yok, her baski parcasi
tek kati ve 256 mm kupe sigar, sag bacak sol bacagin aynasi, servo govdesi / horn / idler parcalari dogru link'te
(isaret kurali), idler mili ve rulmani eklem ekseniyle es eksenli, doner tabla mili yaw ekseninde iki rulmani da
kavrar, kaplin rotor bolgesinde, kablo demetleri her gecis icin dogru link'te, bacak kutlesi butcede, el hesabi
(legs.load_report) emniyet katsayilari >= 1.3 ve idler U-link egilmeyi kuvvet ciftine cevirir (servo miline giden
spline tasma momenti idler'siz momentin <= 1/5'i), sol bacak eklem sinir uclarinda carpisma yok.

  pytest tests/cad -q -p no:cacheprovider
"""
import json
import math
import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import build_config  # noqa: E402
import golden  # noqa: E402

pytestmark = pytest.mark.slow
LEG_LINKS = ["hip", "hip_cradle", "thigh", "shin", "foot"]
DS = ("hip_roll", "hip_pitch", "knee", "ankle")
BODY = {"hip_roll": "hip", "hip_pitch": "thigh", "knee": "thigh", "ankle": "shin"}
HORN = {"hip_yaw": "hip", "hip_roll": "hip_cradle", "hip_pitch": "hip_cradle", "knee": "shin", "ankle": "foot"}

SCRIPT = r'''
import json, math, os, sys
ROOT, OUT = %(root)r, %(out)r
sys.path.insert(0, os.path.join(ROOT, "cad"))
import FreeCAD as App
import openduck, check, legs, servos
m = openduck.OpenDuck(config="tr35_b2")
mods = m.meta["modules"]["legs"]
links = set(mods)
res = {"modules": mods, "items": {ln: [[it.name, it.kind] for it in m.links[ln].items] for ln in mods},
       "trunk_leg_items": [it.name for it in m.links["trunk"].items if it.module == "legs"],
       "legs_g": sum(it.mass_kg(m.p) for ln in mods for it in m.links[ln].items) * 1000.0,
       "total_g": m.total_mass() * 1000.0, "kind_g": {}, "valid": [], "mirror": []}
for ln in mods:
    for it in m.links[ln].items:
        res["kind_g"][it.kind] = res["kind_g"].get(it.kind, 0.0) + it.mass_kg(m.p) * 1000.0
        if not it.shape.isValid():
            res["valid"].append(it.name)
for ln in mods:
    if not ln.startswith("left_"):
        continue
    rl = {it.name: it for it in m.links["right_" + ln[5:]].items}
    for it in m.links[ln].items:
        r = rl.get("right_" + it.name[5:])
        if r is None:
            res["mirror"].append([it.name, "eksik"])
            continue
        a, b = it.shape.BoundBox, r.shape.BoundBox
        err = max(abs(it.shape.Volume - r.shape.Volume) / max(it.shape.Volume, 1.0),
                  abs(a.XMin - b.XMin), abs(a.XMax - b.XMax), abs(a.YMin + b.YMax), abs(a.YMax + b.YMin),
                  abs(a.ZMin - b.ZMin), abs(a.ZMax - b.ZMax))
        if err > 1e-3:
            res["mirror"].append([it.name, err])
res["same_link"] = [list(h[:4]) for h in check.same_link_overlaps(m, links)]
res["print"] = [{k: r[k] for k in ("name", "kind", "dims", "solids", "best")} for r in legs.print_report(m)]
res["print_notes"] = sorted(legs.PRINT_NOTES)
# idler es eksenlilik: idler mili ve rulmani (eksenel simetrik) kutu merkezinin eklem eksenine uzakligi (sifir poz)
J = {j["name"]: j for j in m.joints}
item = {it.name: it for l in m.links.values() for it in l.items}
coax = {}
for key in %(ds)r:
    j = J["left_" + key]
    o, a = App.Vector(*j["origin"]), App.Vector(*j["axis"])
    for part in ("idler_pin", "idler_bearing"):
        c = item["left_%%s_%%s" %% (key, part)].shape.BoundBox.Center - o        # eksenel simetrik parca
        coax["%%s_%%s" %% (key, part)] = (c - a * c.dot(a)).Length
res["coax"] = coax
sh = item["left_hip_yaw_shaft"].shape
tt = servos.turntable_stack(m.p["turntable"])
yx, yy = m.p["j_hip_yaw"][0], m.p["j_hip_yaw"][1]
bb = sh.BoundBox
res["shaft"] = {"cx": (bb.XMin + bb.XMax) / 2 - yx, "cy": (bb.YMin + bb.YMax) / 2 - yy, "d": bb.XLength,
                "z0": bb.ZMin, "z1": bb.ZMax, "lower": tt["lower"], "upper": tt["upper"]}
cp = item["left_hip_yaw_coupler"].shape
pts = [v.Point for v in cp.Vertexes]
res["coupler"] = {"r": max(math.hypot(q.x - yx, q.y - yy) for q in pts), "z0": cp.BoundBox.ZMin,
                  "z1": cp.BoundBox.ZMax, "solids": len(cp.Solids)}
# eksenel yuk yolu (A1 el hesabi): DS baski kovani -> servo ust yuzu (THRUST_GAP, kovanin yakin yuzu tam), ES3054
# kolu kaplinde eksenel serbest (kol altinda top_gap; horn asagi kaydirilinca temas yok)
F = {key: legs._Frame(m, key, m.p["legs"]["horn_toward"].get(key)) for key in legs.DS_KEYS}
rotor = {"hip_roll": "hip_cradle_bracket", "hip_pitch": "hip_cradle_bracket", "knee": "shin_bracket",
         "ankle": "foot_bracket"}
from geom import ycyl
th = {}
for key, fr in F.items():
    d = servos.datums(fr.t)
    r0 = fr.t["horn"]["hub_d"] / 2.0 + m.p["fit"]["clearance"] + 0.5
    face = fr.put(ycyl(r0 + 2.5, 0, 0, d["case_top"] + servos.THRUST_GAP, d["case_top"] + 0.7)
                  .cut(ycyl(r0, 0, 0, d["case_top"] - 1.0, 1.0)))
    br, sv = item["left_" + rotor[key]].shape, item["left_%%s_servo" %% key].shape
    th[key] = [br.distToShape(sv)[0], br.common(face).Volume / face.Volume]
res["thrust"] = th
hn = item["left_hip_yaw_horn"].shape
cg = {}
for dz in (0.3, 0.49):
    h = hn.copy(); h.translate(App.Vector(0, 0, -dz))
    cg[str(dz)] = cp.common(h).Volume
res["coupler_axial"] = cg
rep = legs.load_report(m)
res["checks"] = rep["checks"]
res["ds"] = rep["ds"]
res["turntable"] = rep["turntable"]
# sol bacak eklem sinir uclari (digerleri HOME) + sifir + HOME: carpisma yok (bacak link'leri, kablolar dahil)
stand = m.pose("stand")
poses = [("zero", m.pose("zero")), ("stand", stand)]
for key in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle"):
    j = J["left_" + key]
    for a_ in (j["lower"], j["upper"]):
        p = dict(stand); p[j["name"]] = a_
        poses.append(("%%s=%%g" %% (j["name"], a_), p))
res["hits"] = {n: [[v, a, b] for v, a, b in check.check_pose(m, p, links)] for n, p in poses}
with open(os.path.join(OUT, "res.json"), "w") as f:
    json.dump(res, f, indent=1)
'''


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    fc = golden.find_freecadcmd()
    if fc is None:
        pytest.skip("freecadcmd bulunamadi (FREECADCMD=/yol/freecadcmd ile belirtin)")
    out = str(tmp_path_factory.mktemp("tr35_legs"))
    script = os.path.join(out, "run.py")
    with open(script, "w") as f:
        f.write(SCRIPT % {"root": ROOT, "out": out, "ds": DS})
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    env.pop("OPENDUCK_CONFIG", None)
    log = os.path.join(out, "run.log")
    with open(log, "w") as lf:
        subprocess.run([fc, script], cwd=ROOT, env=env, stdout=lf, stderr=subprocess.STDOUT, timeout=900)
    path = os.path.join(out, "res.json")
    assert os.path.exists(path), open(log).read()[-4000:]
    with open(path) as f:
        return json.load(f)


def test_parts_in_their_links(built):
    r = built
    assert r["modules"] == ["%s_%s" % (s, ln) for s in ("left", "right") for ln in LEG_LINKS]
    for side in ("left", "right"):
        names = {ln: {n for n, k in r["items"]["%s_%s" % (side, ln)]} for ln in LEG_LINKS}
        for key in DS:
            assert "%s_%s_servo" % (side, key) in names[BODY[key]], key
            assert "%s_%s_idler_bearing" % (side, key) in names[BODY[key]], key
            assert "%s_%s_horn" % (side, key) in names[HORN[key]], key
            assert "%s_%s_idler_pin" % (side, key) in names[HORN[key]], key
        assert {"%s_hip_yaw_horn" % side, "%s_hip_yaw_shaft" % side, "%s_hip_yaw_coupler" % side,
                "%s_hip_yaw_screws" % side} <= names["hip"]
        for ln in LEG_LINKS:
            assert "%s_%s_bracket" % (side, ln) in names[ln]
        assert "%s_sole" % side in names["foot"]
        # kablo demetleri: <eklem>_harness_in ust link'te, _out alt link'te (hip_yaw girisi govdede)
        for key, (pa, ch) in {"hip_yaw": ("trunk", "hip"), "hip_roll": ("hip", "hip_cradle"),
                              "hip_pitch": ("hip_cradle", "thigh"), "knee": ("thigh", "shin")}.items():
            if pa == "trunk":
                assert "%s_%s_harness_in" % (side, key) in r["trunk_leg_items"]
            else:
                assert "%s_%s_harness_in" % (side, key) in names[pa]
            assert "%s_%s_harness_out" % (side, key) in names[ch]


def test_same_link_zero_valid_and_mirrored(built):
    assert built["same_link"] == []
    assert built["valid"] == []
    assert built["mirror"] == []


def test_print_parts_one_solid_fit_bed(built):
    bed = build_config.load("tr35_b2")["targets"]["bed_mm"]
    names = set()
    for p in built["print"]:
        names.add(p["name"])
        assert p["solids"] == 1, p
        assert max(p["dims"]) <= min(bed), p
        assert p["best"][1] >= (10.0 if p["name"] == "hip_yaw_coupler" else 100.0), p    # tablada duz yuz
    assert names == {"hip_bracket", "hip_yaw_coupler", "hip_cradle_bracket", "thigh_bracket", "shin_bracket",
                     "foot_bracket", "sole"}
    assert names <= set(built["print_notes"])


def test_leg_mass_budget(built):
    """p2-interface.md sec. 8: bacaklar (iki bacak) 913.2 g; baski 2 x 130 g; toplam x 1.05 <= 1.95 kg."""
    r = built
    t = build_config.load("tr35_b2")["targets"]
    assert r["legs_g"] <= 913.2 + 1e-6, r["legs_g"]
    printed = r["kind_g"]["bracket"] + r["kind_g"]["sole"]
    assert printed <= 2 * 130.0, printed
    assert r["total_g"] * t["mass_factor"] <= t["mass_max_kg"] * 1000.0


def test_idler_coaxial_and_turntable(built):
    for k, d in built["coax"].items():
        assert d < 0.05, (k, d)
    s = built["shaft"]
    assert abs(s["cx"]) < 1e-6 and abs(s["cy"]) < 1e-6 and s["d"] == pytest.approx(10.0)
    # mil alt rulmanin altindan ust rulmanin icine kadar (iki ic bilezigi de kavrar)
    assert s["z0"] < s["lower"][0] - 5.0 and s["upper"][0] + 4.0 < s["z1"] <= s["upper"][1]
    c = built["coupler"]
    assert c["solids"] == 1 and c["r"] <= 15.0 and c["z1"] <= 32.2 and c["z0"] >= s["upper"][0]


def test_hand_calc_load_path(built):
    """A1 el hesabi: emniyet >= 1.3 (2 W dusme); U-link'in iki mesnedi egilmeyi kuvvet ciftine cevirir: servo milinin
    gordugu spline tasma momenti, idler'siz (tek mesnet) spline momentinin en cok 1/5'i."""
    for n, v, lim, sf in built["checks"]:
        assert sf >= 1.3, (n, v, lim, sf)
    for key, rec in built["ds"].items():
        for case, r in rec.items():
            assert r["M_shaft"] * 5.0 <= r["M_cant"], (key, case, r)
    for case, r in built["turntable"].items():
        assert r["Rl"] > 0 and r["Ru"] > 0


def test_axial_load_paths(built):
    """Servo miline eksenel yuk gitmez (p2a-el-hesabi.md 5-6): her DS ekleminde U-link baski kovani servo ust yuzune
    0.2 mm (kovanin servoya bakan yuzu kesicilerle kirpilmamis), ES3054 kolu kaplin yariginda eksenel serbest (kol
    0.49 mm asagi kaydirilinca kaplinla kesisim yok)."""
    for key, (gap, face) in built["thrust"].items():
        assert gap == pytest.approx(0.2, abs=0.005), (key, gap)
        assert face > 0.999, (key, face)
    for dz, v in built["coupler_axial"].items():
        assert v < 1e-6, (dz, v)


def test_no_collision_at_limit_ends(built):
    bad = {n: h for n, h in built["hits"].items() if h}
    assert bad == {}, bad
