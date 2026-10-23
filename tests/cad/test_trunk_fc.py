# -*- coding: utf-8 -*-
"""
TR-35 govdesi (P2b; freecadcmd yoksa atlanir): OpenDuck(config="tr35_b2") kurulur ve trunk link'i denetlenir.
A1 / plan P2b: ayni link icinde 0 cakisma (elektronik ve govde ici kablolar dahil), P2b parcalari K_T icinde, isi
kaynaklari PETG uzerinde ve PLA'ya >= 5 mm, butun govde baski parcalari 256 mm kupe sigar, kutle butcesi, IMU govde
COM'una yakin ve duz, paylasilan horn/idler anahtarlari, kalca/boyun kablo kanallari donmus giris tuplerine bagli.
P2 entegrasyonu: baski parcalari tek kati, idler yuvasi bagi (ICR-1) ve batarya kapagi (ICR-2).

  pytest tests/cad -q -p no:cacheprovider
"""
import json
import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import build_config  # noqa: E402
import golden  # noqa: E402

pytestmark = pytest.mark.slow

SCRIPT = r'''
import itertools, json, math, os, sys
ROOT, OUT = %(root)r, %(out)r
sys.path.insert(0, os.path.join(ROOT, "cad"))
import openduck, electronics as E
from geom import envelope_solid
m = openduck.OpenDuck(config="tr35_b2")
tr = m.links["trunk"]
items = tr.items
res = {"invalid": [it.name for it in items if not it.shape.isValid()]}
hits = []
for a, b in itertools.combinations(items, 2):
    if not a.shape.BoundBox.intersect(b.shape.BoundBox):
        continue
    v = a.shape.common(b.shape).Volume
    if v > 1.0:
        hits.append([a.name, b.name, v])
res["same_link"] = hits
env = m.p["iface"]["envelopes"]["trunk"]
K = envelope_solid(env["add"]).cut(envelope_solid(env["cut"]))
res["outside"] = {it.name: it.shape.cut(K).Volume for it in items if it.module == "trunk" and it.kind != "harness"}
by = {it.name: it for it in items}
pla = [it for it in items if it.kind in ("shell", "trim", "face")]
heat = {}
for name in m.p["trunk"]["electronics"]:
    if E.is_heat_source(name):
        it = by[name.replace("#", "_")]
        touch = sorted(o.kind for o in items if o is not it and o.module == "trunk" and o.kind != "harness"
                       and it.shape.distToShape(o.shape)[0] < 0.3)
        heat[name] = {"pla": min(it.shape.distToShape(q.shape)[0] for q in pla), "touch": touch}
res["heat"] = heat
res["print"] = {it.name: sorted((it.shape.BoundBox.XLength, it.shape.BoundBox.YLength, it.shape.BoundBox.ZLength))
                for it in items if it.kind in openduck.PRINT_KINDS}
mt, com, _ = m.link_inertia("trunk")
res["trunk_mass_g"] = mt * 1000.0
res["trunk_com"] = [com.x, com.y, com.z]
res["total_g"] = m.total_mass() * 1000.0
imu = by[m.meta["imu_item"]]
ib = imu.shape.BoundBox
res["imu"] = {"z_extent": ib.ZLength, "center": [ib.Center.x, ib.Center.y, ib.Center.z]}
res["shared"] = {"hip_yaw_horn": sorted(m.shared.get("hip_yaw_horn", {})),
                 "neck": [k for k in ("neck_pitch_horn", "neck_pitch_idler_pin") if m.shared.get(k) is not None]}
vols = {}
for name, pl in m.p["trunk"]["electronics"].items():
    rec = E.PARTS[E.part_of(name)]
    sx, sy, sz = rec["size"]
    exp = math.pi * sy * sy / 4.0 * sx if rec["form"] == "cyl" else (sx * sy * sz if rec["form"] == "box" else None)
    vols[name] = [by[name.replace("#", "_")].shape.Volume, exp]
res["el_vol"] = vols
links = {}
for a, b in (("left_hip_trunk_wiring", "left_hip_yaw_harness_in"),
             ("right_hip_trunk_wiring", "right_hip_yaw_harness_in"),
             ("neck_trunk_wiring", "neck_pitch_harness_in")):
    links[a] = by[a].shape.distToShape(by[b].shape)[0]
res["wire_links"] = links
res["names"] = [it.name for it in items]
res["solids"] = {it.name: len(it.shape.Solids) for it in items if it.kind in openduck.PRINT_KINDS}
seat = by["neck_pitch_idler_seat"].shape
res["tie"] = {n: seat.distToShape(by[n].shape)[0] for n in ("left_hip_yaw_servo", "trunk_fasteners",
                                                             "neck_pitch_idler_bearing", "neck_pitch_servo")}
db = by["battery_door"].shape.BoundBox if "battery_door" in by else None
res["door_bb"] = db and [db.XMin, db.XMax, db.YMin, db.YMax, db.ZMin, db.ZMax]
with open(os.path.join(OUT, "res.json"), "w") as f:
    json.dump(res, f, indent=1)
'''


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    fc = golden.find_freecadcmd()
    if fc is None:
        pytest.skip("freecadcmd bulunamadi (FREECADCMD=/yol/freecadcmd ile belirtin)")
    out = str(tmp_path_factory.mktemp("tr35_trunk"))
    script = os.path.join(out, "run.py")
    with open(script, "w") as f:
        f.write(SCRIPT % {"root": ROOT, "out": out})
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    env.pop("OPENDUCK_CONFIG", None)
    log = os.path.join(out, "run.log")
    with open(log, "w") as lf:
        subprocess.run([fc, script], cwd=ROOT, env=env, stdout=lf, stderr=subprocess.STDOUT, timeout=900)
    path = os.path.join(out, "res.json")
    assert os.path.exists(path), open(log).read()[-4000:]
    with open(path) as f:
        return json.load(f)


CFG = build_config.load("tr35_b2")


def test_valid_and_no_same_link_overlap(built):
    """A1: ayni link ici 0 cakisma (elektronik, kablo kanallari, bacak/kafa giris tupleri dahil)."""
    assert built["invalid"] == []
    assert built["same_link"] == []


def test_p2b_parts_inside_trunk_keep_in(built):
    """p2-interface 4.3 (a): P2b parcalari K_T icinde (kablo demeti haric)."""
    bad = {k: v for k, v in built["outside"].items() if v > 1.0}
    assert bad == {}


def test_heat_sources_petg_and_gap(built):
    """spec 3.1: isi kaynagina PETG (temas eden basili parcalar bracket = PETG), PLA'ya >= 5 mm hava."""
    gap = CFG["targets"]["pla_heat_gap_mm"]
    assert set(built["heat"]) == {"buck_20a", "xl4015"}
    for name, h in built["heat"].items():
        assert h["pla"] >= gap - 1e-6, (name, h)
        assert "shell" not in h["touch"] and "bracket" in h["touch"], (name, h)


def test_print_parts_fit_bed(built):
    bed = min(CFG["targets"]["bed_mm"])
    trunk_print = [n for n in built["print"] if n in built["names"]]
    assert {"trunk_base", "buck_hood", "belly_duct", "torso_shell_left", "torso_shell_right", "torso_top",
            "deck_cover", "battery_strap", "battery_door", "neck_pitch_idler_seat"} <= set(trunk_print)
    for n in trunk_print:
        assert built["print"][n][-1] <= bed, (n, built["print"][n])


def test_print_parts_single_solid(built):
    """P2 entegrasyonu: her govde baski parcasi tek kati (kabuk cekirdek duvari ve sag cene seridi ayrik kalmaz)."""
    bad = {n: k for n, k in built["solids"].items() if n in built["names"] and k != 1}
    assert bad == {}


def test_seat_tie_and_battery_door(built):
    """ICR-1: idler yuvasi PETG bagla sol on ES3054 tab'ina oturur, tab vidasi ayagi sikar, VHB yok, DS ve rulmanla
    cakismaz. ICR-2: batarya kapagi K_T 'door' kutusunda (K_T icerme testi), batarya ucunu orter."""
    tie = built["tie"]
    assert tie["left_hip_yaw_servo"] < 1e-6 and tie["trunk_fasteners"] < 1e-6, tie
    assert tie["neck_pitch_servo"] >= 0.3 - 1e-6, tie
    assert "neck_seat_vhb" not in built["names"]
    add = CFG["params"]["iface"]["envelopes"]["trunk"]["add"]
    assert ["box", 19.9, 26.2, 21.0, 26.65, 34.4, 39.7] in add and ["box", -95.0, -92.0, -24.0, 24.0, -3.0, 31.0] in add
    x0, x1, y0, y1, z0, z1 = built["door_bb"]
    bat = CFG["params"]["trunk"]["electronics"]["battery_4s_lipo"]
    assert x0 >= -95.0 - 1e-6 and y0 <= -20.0 and y1 >= 20.0 and z0 <= bat[2] and z1 >= bat[2] + 30.0


def test_mass_budget(built):
    """Govde <= p2-interface butcesi (733.6 g); toplam (P2a/P2c yer tutuculariyla) x 1.05 <= 1.95 kg."""
    T = CFG["params"]["trunk"]
    t = CFG["targets"]
    assert built["trunk_mass_g"] <= T["mass_budget_g"] + 1e-6, built["trunk_mass_g"]
    assert built["total_g"] * t["mass_factor"] <= t["mass_max_kg"] * 1000.0 + 1e-6


def test_imu_flat_near_trunk_com(built):
    """MPU6500 duz (eksenler govdeye paralel, MJCF sitesi donmesiz), govde COM'una 15 mm icinde, tam ortada (y)."""
    imu = built["imu"]
    assert abs(imu["z_extent"] - 3.0) < 1e-6
    c, g = imu["center"], built["trunk_com"]
    assert sum((a - b) ** 2 for a, b in zip(c, g)) ** 0.5 <= 15.0, (c, g)
    assert abs(c[1]) < 1e-6


def test_shared_keys(built):
    assert built["shared"]["hip_yaw_horn"] == ["left", "right"]
    assert built["shared"]["neck"] == ["neck_pitch_horn", "neck_pitch_idler_pin"]


def test_electronics_real_envelopes(built):
    """Her yerlesim kayit zarfini (kutu / silindir) tam hacimle uretir."""
    for name, (v, exp) in built["el_vol"].items():
        if exp is not None:
            assert abs(v - exp) <= 1e-3 * exp + 1e-6, (name, v, exp)


def test_wire_channels_connect_frozen_crossings(built):
    """Kalca ve boyun kanallari donmus params.harness giris tuplerine degiyor (yuz temasi, cakismasiz)."""
    for k, dd in built["wire_links"].items():
        assert dd < 1e-6, (k, dd)
