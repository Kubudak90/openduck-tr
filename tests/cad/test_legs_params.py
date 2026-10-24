# -*- coding: utf-8 -*-
"""
TR-35 bacak konfigu (P2a, params.legs + bacak eklem sinirlari), FreeCAD gerektirmez: gerekli araliklar sinirlarin
icinde, sinirlar servo hareketi icinde, hip_pitch spec sinirinda (-70..90), horn kol yonleri horn eksenine dik,
baglanti elemanlari mutlak M3 (olceklenmez), doner tabla mili / kaplin olculeri birbiriyle ve donmus tabla
yiginiyla tutarli, kablo guzergahi birlesimleri gecerli, U-link idler mesnet acikligi.

  pytest tests/cad -q -p no:cacheprovider
"""
import math

import pytest

import build_config
import servos

LEG = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")
DS = ("hip_roll", "hip_pitch", "knee", "ankle")
# P2a sinirlari (configs/tr35_b2.json params.legs._limits; p2a-el-hesabi.md)
LIMITS = {"hip_yaw": (-10.0, 21.0), "hip_roll": (-20.0, 20.0), "hip_pitch": (-70.0, 90.0), "knee": (-84.0, 77.0),
          "ankle": (-65.0, 64.0)}


@pytest.fixture(scope="module")
def cfg():
    return build_config.load("tr35_b2")


def test_leg_limits_contain_required_within_travel(cfg):
    lim, req = cfg["joint_limits_deg"], cfg["required_deg"]
    for key in LEG:
        assert tuple(lim[key]) == LIMITS[key], key
        assert lim[key][0] <= req[key][0] and req[key][1] <= lim[key][1], key
    assert tuple(lim["hip_pitch"]) == (-70.0, 90.0)
    assert build_config.travel_violations(cfg, lim) == []
    # RL degismezi (range = sinir +- 3): sinir + 3 de servo hareketi icinde
    for key in LEG:
        t = servos.get(cfg["servo_map"][key])
        c = cfg.get("neutral_deg", {}).get(key, 0.0) + cfg.get("horn_offset_deg", {}).get(key, 0.0)
        lo, hi = servos.travel_range(t, c)
        assert lo <= lim[key][0] - 3.0 and lim[key][1] + 3.0 <= hi, key


def test_horn_toward_perpendicular_to_horn_axis(cfg):
    legs = cfg["params"]["legs"]
    for key in DS:
        h = cfg["params"]["servo_dir"][key][0]
        u = legs["horn_toward"][key]
        n = math.sqrt(sum(c * c for c in u))
        assert n > 0.5
        assert abs(sum(a * b for a, b in zip(h, u))) / n < 1e-6, key
        assert -180.0 <= servos.arm_angle_deg(h, cfg["params"]["servo_dir"][key][1], u) <= 180.0


def test_fasteners_absolute_m3(cfg):
    fit = cfg["params"]["fit"]
    assert (fit["clear_d"], fit["tap_d"], fit["nut_af"], fit["nut_t"]) == (3.4, 2.5, 5.7, 2.6)
    assert servos.M3_HEAD == {"d": 5.5, "h": 3.0} and servos.M3_NUT["af"] == 5.5
    assert servos.THRUST_GAP > 0 and servos.IDLER_SPACER_R * 2 < 5.3     # ara halka yalniz ic bilezige basar


def test_turntable_shaft_and_coupler_consistent(cfg):
    p = cfg["params"]
    H, C, tt = p["legs"]["hip"], p["legs"]["coupler"], p["turntable"]
    st = servos.turntable_stack(tt)
    assert st["spacing"] >= tt["spacing_min"] >= 20.0
    ztop = st["z_mate"] - tt["gap"]
    # mil gobekte en az 8 mm; capraz vida deligi gobek icinde, alt ucta >= 2 mm et
    assert ztop - H["shaft_bot"] >= 8.0
    assert H["shaft_bot"] + H["pin_hole_d"] / 2.0 + 2.0 <= H["pin_z"] <= ztop - H["pin_hole_d"] / 2.0 - 1.0
    assert H["boss_bot"] < H["shaft_bot"]
    # vida basi birlesme duzleminin altinda (gobek flansi r <= 8.5 bolgesi govde yuvasina girmez)
    assert H["pin_z"] + servos.M3_HEAD["d"] / 2.0 < st["z_mate"] - tt["gap"]
    assert H["r"] <= 8.5
    # kaplin: mil ucu ust ic bilezigin altinda, kama yarigi kilavuzdan sig, havsa halka icinde
    assert 0.0 < C["shaft_gap"] < 1.0
    z1 = st["horn_face"] - C["top_gap"]                   # kaplin halkasi ustu (havsa yuzu)
    sh_top = st["upper"][1] - C["shaft_gap"]
    assert C["key_depth"] < C["tap_depth"] < sh_top - st["lower"][1]       # kor delik iki rulman arasinda biter
    assert C["csk_h"] < z1 - st["upper"][1]
    assert C["screw_len"] - C["csk_h"] <= (z1 - C["csk_h"] - sh_top) + C["tap_depth"] - 0.3   # vida kor delige
    assert C["r"] <= 15.0 and C["seat_r"] < st["bearing_rec"]["D"] / 2.0       # rotor bolgesi, ic bilezik


def test_harness_routes_and_joins(cfg):
    legs, hc = cfg["params"]["legs"], cfg["params"]["harness"]["crossings"]
    rt = legs["harness_route"]
    for a, b, mode in rt["joins"]:
        assert a in hc and b in hc and mode in ("mid", "a", "b")
    for key, r in rt.items():
        if key.startswith("_") or key == "joins":
            continue
        assert key in hc
        for q in r.get("in", []) + r.get("out", []):
            # [x, y, z] ya da [x, y, z, r]: r kose bukulme yaricapi (route.fillet; dogrulama 2, D1)
            assert len(q) == 3 or (len(q) == 4 and q[3] > 0.0)
        if "out_d" in r:
            assert 0 < r["out_d"] <= hc[key]["d"]
        if r.get("gland"):
            assert r["gland"] in DS


def test_idler_support_span():
    """U-link iki mesnedi (spline kavrama merkezi A ve F623ZZ merkezi B) arasindaki aciklik: egilme momenti bu kolla
    kuvvet ciftine doner (p2a-el-hesabi.md)."""
    t = servos.get("DS32XX")
    st = servos.idler_stack(t, "F623ZZ")
    hn = t["horn"]
    yA = -(hn["h"][1] - hn["h"][0]) / 2.0
    assert yA - st["axis"] == pytest.approx(48.45)
    # servo ic yatagi (boss ustu) ile spline kavrama merkezi arasi kol (spline tasma momenti)
    assert yA - servos.datums(t)["boss_top"] == pytest.approx(3.55)
    # rulman merkezinden B kolu orta duzlemine (idler M3 mili egilme kolu)
    assert st["axis"] - sum(st["leg"]) / 2.0 == pytest.approx(4.5)
