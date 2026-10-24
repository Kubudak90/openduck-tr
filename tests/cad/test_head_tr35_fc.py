# -*- coding: utf-8 -*-
"""
TR-35 boyun + kafa (P2c), freecadcmd ile (yoksa atlanir): kafa link'lerinde ayni link ici cakisma yok, parcalar
modul zarfi icinde (arayuz degisiklik istegi: tampon govde dili ve gaga, p2c-tampon.md), kutle butcesi, MG90S pasif
destekleri (head_pitch baski yuzleri, head_yaw baski halkasi, head_roll on pimi) bosluklari, eklem sinirlarinda ve
birlesik uclarda carpisma yok, tamponun ilk temasi kafa (cene blogu, head link) ve MG90S zincirinden once, MJCF'de
boyun govdesinde "neck_bumper" carpisma geomu.

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
HEAD_LINKS = ["neck", "head_tilt", "head_turn", "head", "jaw"]

SCRIPT = r'''
import json, os, sys
ROOT, OUT = %(root)r, %(out)r
sys.path.insert(0, os.path.join(ROOT, "cad"))
import openduck, export, check
m = openduck.OpenDuck(config="tr35_b2")
head = m.meta["modules"]["head"]
links = set(head)
env = check.iface_envelopes(m)
items = {it.name: (ln, it) for ln, l in m.links.items() for it in l.items}
res = {"items": {ln: [[it.name, it.kind] for it in m.links[ln].items] for ln in head},
       "link_mass_g": {ln: sum(it.mass_kg(m.p) for it in m.links[ln].items) * 1000 for ln in head},
       "head_trunk_link_g": sum(it.mass_kg(m.p) for it in m.links["trunk"].items if it.module == "head") * 1000,
       "kind_mass_g": {}, "outside": {}, "valid": [], "bed": []}
for ln in head:
    for it in m.links[ln].items:
        res["kind_mass_g"][it.kind] = res["kind_mass_g"].get(it.kind, 0.0) + it.mass_kg(m.p) * 1000
        if not it.shape.isValid():
            res["valid"].append(it.name)
        if it.kind in openduck.PRINT_KINDS:
            bb = it.shape.BoundBox
            res["bed"].append([it.name, max(bb.XLength, bb.YLength, bb.ZLength)])
        if it.kind != "harness":
            v = it.shape.cut(env[ln]).Volume
            if v > check.TOL:
                res["outside"][it.name] = v
res["same_link"] = [list(h[:4]) for h in check.same_link_overlaps(m, links)]
d = lambda a, b: items[a][1].shape.distToShape(items[b][1].shape)[0]
res["gaps"] = {"thrust_in": d("head_tilt_bracket", "head_pitch_thrust_in"),
               "thrust_out": d("head_tilt_bracket", "head_pitch_thrust_out"),
               "yaw_ring": d("head_turn_bracket", "head_tilt_bracket"),
               "roll_pin": d("head_turn_bracket", "head_frame"),
               "jaw_pin": d("lower_bill", "jaw_pin_bushing"),
               "neck_thrust": d("neck_link", "trunk_base"),
               "neck_idler": d("neck_pitch_idler_pin", "neck_pitch_idler_bearing"),
               "neck_servo": d("neck_link", "neck_pitch_servo")}
# head_yaw eksenel tutucusu (D3): flans tabla ile yaylarin dudagi arasinda; head_turn z'de kaydirilinca once 0.3'te
# tutucuya (yukari) / tablaya (asagi) dayanir; yaw sinir + 3 boyunca yaylar head_turn parcalarina degmez
import FreeCAD as App
yx, yy, yz = m.p["j_head_yaw"]
ret = [n for n in items if n.startswith("head_yaw_retainer_") and n != "head_yaw_retainer_screws"]
turn = items["head_turn_bracket"][1].shape
def lift(dz):
    t = turn.copy(); t.translate(App.Vector(0, 0, dz))
    return {"up": sum(t.common(items[n][1].shape).Volume for n in ret),
            "down": t.common(items["head_tilt_bracket"][1].shape).Volume}
res["yaw_lift"] = {str(dz): lift(dz) for dz in (0.25, 0.4, -0.25, -0.4)}
res["gaps"]["yaw_retainer"] = min(d(n, "head_turn_bracket") for n in ret)
sweep = {}
lim = [j for j in m.joints if j["name"] == "head_yaw"][0]
for q in range(int(lim["lower"]) - 3, int(lim["upper"]) + 4, 4):
    for it in [it for ln, it in items.values() if ln == "head_turn"]:
        sh = it.shape.copy(); sh.rotate(App.Vector(yx, yy, yz), App.Vector(0, 0, 1), q)
        for n in ret + ["head_yaw_retainer_screws"]:
            v = sh.distToShape(items[n][1].shape)[0]
            k = n + " | " + it.name
            sweep[k] = min(sweep.get(k, 1e9), v)
res["yaw_retainer_sweep"] = sweep
# head_roll: head_frame guvertesinin ic kenari yaylarin dis ustune iner (pah + +-25 derece; sinirda > 0.5 mm)
roll = {}
jr = [j for j in m.joints if j["name"] == "head_roll"][0]
for q in (jr["lower"], jr["upper"]):
    p = dict(m.pose("stand")); p["head_roll"] = q
    sh = {i.name: s for ln, i, s in m.posed_shapes(p)}
    roll[str(q)] = min(sh[n].distToShape(sh["head_frame"])[0] for n in ret + ["head_yaw_retainer_screws"])
res["roll_retainer"] = roll
stand = m.pose("stand")
J = {j["name"]: j for j in m.joints}
poses = []
for jn in ("neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth"):
    for a in (J[jn]["lower"], J[jn]["upper"]):
        p = dict(stand); p[jn] = a
        poses.append(("%%s=%%g" %% (jn, a), p))
# gerekli araliklarda birlesik uclar (head_pitch ile mouth komsu degil; head_pitch >= ~55 ve mouth >= ~28'de gaga
# boyuna deger: p2c-tampon.md "gaga <-> boyun", MJCF cifti onerisi)
for hp, hy, hr, mo in ((60, 34, 10, 20), (50, -34, -10, 30), (-20, -34, -10, 30), (60, -34, -10, 0), (-20, 34, 10, 0)):
    p = dict(stand); p.update(head_pitch=hp, head_yaw=hy, head_roll=hr, mouth=mo)
    poses.append(("combo %%d/%%d/%%d/%%d" %% (hp, hy, hr, mo), p))
res["hits"] = {n: [[v, a, b] for v, a, b in check.check_pose(m, p, links)] for n, p in poses}
top = J["head_pitch"]["upper"]
seq = {}
for da in (3.0, 4.5):
    p = dict(stand); p["head_pitch"] = top + da
    sh = {it.name: s for ln, it, s in m.posed_shapes(p)}
    seq[str(da)] = {n: sh["neck_bumper"].distToShape(sh[n])[0] for n in
                    ("head_frame", "head_tilt_bracket", "head_hood", "lower_bill", "head_yaw_harness_out")}
res["bumper_seq"] = seq
res["collision_geoms"] = m.meta.get("collision_geoms")
res["bumper_box"] = [list(items["neck_bumper"][1].collision_box[0]), list(items["neck_bumper"][1].collision_box[1])]
res["neck_origin"] = list(m.links["neck"].origin)
export.export_robot(m, os.path.join(OUT, "sim"))
with open(os.path.join(OUT, "res.json"), "w") as f:
    json.dump(res, f, indent=1)
'''


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    fc = golden.find_freecadcmd()
    if fc is None:
        pytest.skip("freecadcmd bulunamadi (FREECADCMD=/yol/freecadcmd ile belirtin)")
    out = str(tmp_path_factory.mktemp("tr35_head"))
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
        return out, json.load(f)


def test_parts_valid_same_link_and_bed(built):
    _, r = built
    assert r["valid"] == []
    assert r["same_link"] == []
    bed = min(build_config.load("tr35_b2")["targets"]["bed_mm"])
    assert all(size < bed for _, size in r["bed"]), r["bed"]


def test_envelopes_only_requested_changes(built):
    """Zarf disi yalniz arayuz degisiklik istegindeki parcalar: tampon govde dili (neck_link, x 45.3..51) ve gaga
    (jaw: alt gaga, gobek, vidalar)."""
    _, r = built
    assert set(r["outside"]) <= {"neck_link", "jaw_hub", "lower_bill", "jaw_hub_screws"}, r["outside"]
    B = build_config.load("tr35_b2")["params"]["head"]["bumper"]
    stem = (B["box"][0] - (33.332 + 12.0)) * 2 * B["stem_half_y"] * (B["box"][5] - 84.0)
    assert r["outside"].get("neck_link", 0.0) <= stem


# kafa yapisi butcesi (asagidan yukari): p2-interface.md 8'deki 202.0 g + head_yaw eksenel tutucusu 1.5 g (dogrulama 2,
# D3; trunk butcesinden, p2-entegrasyon.md 10). Ses modulunun 30 g'i (spec eki K3.1) P2d'nindir, burada yok.
HEAD_BUDGET_G = 203.5


def test_head_mass_budget(built):
    """Kafa link'leri + trunk link'indeki neck_pitch kablo girisi <= HEAD_BUDGET_G."""
    _, r = built
    total = sum(r["link_mass_g"].values()) + r["head_trunk_link_g"]
    assert total <= HEAD_BUDGET_G + 1e-6, (total, r["link_mass_g"])
    k = r["kind_mass_g"]
    assert k["servo"] == pytest.approx(3 * 13.4 + 9.0)
    assert k["horn"] == pytest.approx(7.0)


def test_passive_supports(built):
    """MG90S miline egilme/eksenel yuk gitmesin: head_pitch diski iki baski yuzu arasinda, yaw halkasi tablanin
    uzerinde ve flansi tutucu yaylarin altinda, roll pimi on yatakta, gaga pimi yatakta (bosluklar params.head.fit)."""
    _, r = built
    F = build_config.load("tr35_b2")["params"]["head"]["fit"]
    g = r["gaps"]
    assert g["thrust_in"] == pytest.approx(F["gap"], abs=0.02)
    assert g["thrust_out"] == pytest.approx(F["gap"], abs=0.02)
    assert g["yaw_ring"] == pytest.approx(F["gap"], abs=0.02)
    # head_yaw eksenel tutma (dogrulama 2, D3): kafa yukari cekilince flans 0.3'ten sonra yaylarin dudagina, asagi
    # bastirilinca halka tablaya dayanir; MG90S mili eksenel yuk almaz
    assert g["yaw_retainer"] == pytest.approx(F["gap"], abs=0.02)
    lift = r["yaw_lift"]
    assert lift["0.25"]["up"] == 0.0 and lift["0.4"]["up"] > 1.0
    assert lift["-0.25"]["down"] == 0.0 and lift["-0.4"]["down"] > 1.0
    sw = r["yaw_retainer_sweep"]
    assert min(sw.values()) >= 0.25, sorted(sw.items(), key=lambda kv: kv[1])[:3]
    assert min(r["roll_retainer"].values()) > 0.5, r["roll_retainer"]
    assert 0.0 < g["roll_pin"] <= 0.5
    assert 0.0 < g["jaw_pin"] <= 0.5
    # boyun DS32xx eksenel yuku milde degil (p2c-tampon.md 4): +y A kolu baski kovani trunk_base besik duvarlarina
    # 0.2, -y B kolu somun + ara halka F623ZZ ic bilezigine 0.05; servo govdesine daha uzak
    assert g["neck_thrust"] == pytest.approx(0.2, abs=0.005)
    assert g["neck_idler"] == pytest.approx(0.05, abs=0.005)
    assert g["neck_servo"] > g["neck_thrust"] + 1.0


# Bilinen kusur (dogrulama 2, D5; P2d'ye devredildi, spec eki K3.6 "head_yaw +-80 burulma taramasi"): head_yaw
# gecisinin giris ve cikis tupleri yaw 0'da gecis noktasindan 2 d uzakta da ust uste biner (kablo kendi icinden gecer).
# Donmus geometride gecerli guzergah yok (p2-entegrasyon.md 10.3); check.py bunu sayar. P2d duzeltince bu kume bosalir.
KNOWN = {("head_yaw_harness_in", "head_yaw_harness_out")}


def test_no_collisions_at_limits_and_combos(built):
    _, r = built
    bad = {n: [x for x in h if (x[1], x[2]) not in KNOWN] for n, h in r["hits"].items()}
    bad = {n: h for n, h in bad.items() if h}
    assert not bad, bad


def test_known_head_yaw_crossing_defect_tracked(built):
    """KNOWN hala mevcut: P2d head_yaw gecisini duzelttiginde bu test kirilir ve KNOWN bosaltilir."""
    _, r = built
    seen = {(x[1], x[2]) for h in r["hits"].values() for x in h}
    assert KNOWN <= seen, "head_yaw gecis kusuru duzelmis: KNOWN'u ve p2-entegrasyon.md 10.3'u guncelleyin"


def test_bumper_takes_head_first(built):
    """head_pitch ust siniri + 3 (MJCF range = ctrlrange + 3, spec 3.2.3) temassiz; tampona ilk degen kafa link'i
    (cene blogu): head_tilt / kablo / gaga MG90S zinciri tampondan once yuk almaz."""
    _, r = built
    s = r["bumper_seq"]
    assert min(s["3.0"].values()) > 0.0
    hit = s["4.5"]
    assert hit["head_frame"] == pytest.approx(0.0, abs=1e-6)
    assert min(v for k, v in hit.items() if k != "head_frame") > 0.5, hit


def test_mjcf_neck_bumper_geom(built):
    mujoco = pytest.importorskip("mujoco")
    out, r = built
    assert r["collision_geoms"] == ["neck_bumper"]
    mm = mujoco.MjModel.from_xml_path(os.path.join(out, "sim", "openduck.xml"))
    G, B = mujoco.mjtObj.mjOBJ_GEOM, mujoco.mjtObj.mjOBJ_BODY
    gid = mujoco.mj_name2id(mm, G, "neck_bumper")
    assert gid >= 0
    assert mm.geom_bodyid[gid] == mujoco.mj_name2id(mm, B, "neck")
    assert mm.geom_type[gid] == mujoco.mjtGeom.mjGEOM_BOX
    assert (mm.geom_contype[gid], mm.geom_conaffinity[gid]) == (0, 1)
    c, h = r["bumper_box"]
    assert list(mm.geom_size[gid]) == pytest.approx([v / 1000.0 for v in h])
    assert list(mm.geom_pos[gid]) == pytest.approx([(a - b) / 1000.0 for a, b in zip(c, r["neck_origin"])])
    assert mujoco.mj_name2id(mm, G, "neck_collision") < 0
    assert math.isfinite(mm.body_mass[mujoco.mj_name2id(mm, B, "head")])
