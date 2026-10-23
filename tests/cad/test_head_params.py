# -*- coding: utf-8 -*-
"""
TR-35 kafa modulu (P2c) konfig denetimi, FreeCAD gerektirmez: donmus kablo gecisleri (nokta + cap) degismemis,
moduller arasi gecis (neck_pitch) yalniz uzatilmis, link ici guzergah birlesmeleri es dogrusal (sifir hacimli temas),
gerekli araliklar <= eklem sinirlari <= servo hareketi, tampon yastigi donmus tampon bolgesinde, horn yonleri horn
eksenine dik.

  pytest tests/cad -q -p no:cacheprovider
"""
import math

import pytest

import build_config
import servos

CFG = build_config.load("tr35_b2")
P = CFG["params"]
H = P["head"]
# P2-0 (p2-interface.md, kablo demetleri) gecis noktalari ve demet caplari: donmus
P2_0 = {"neck_pitch": ((33.332, -34.4, 41.537), 5.0), "head_pitch": ((33.332, -24.0, 105.637), 4.5),
        "head_yaw": ((33.332, 0.0, 122.0), 3.5), "head_roll": ((18.0, 0.0, 148.199), 3.5)}
# ayni link'te ardisik demet tuplari (onceki gecisin out'u -> sonrakinin in'i)
JUNCTIONS = (("neck_pitch", "head_pitch"), ("head_pitch", "head_yaw"), ("head_yaw", "head_roll"))


def _paths(jn):
    """head._harness ile ayni kural (FreeCAD'siz): (in noktalari, out noktalari)."""
    hc, r = P["harness"]["crossings"][jn], H["harness_route"].get(jn, {})
    cross = tuple(hc["cross"])
    pin = [tuple(q) for q in r["in"]] if "in" in r else [tuple(q) for q in r.get("in_pre", []) + hc["in"]]
    pout = [tuple(q) for q in r["out"]] if "out" in r else [tuple(q) for q in hc["out"] + r.get("out_post", [])]
    return pin + [cross], [cross] + pout


def _unit(a, b):
    d = [y - x for x, y in zip(a, b)]
    n = math.sqrt(sum(c * c for c in d))
    return [c / n for c in d]


@pytest.mark.parametrize("jn", sorted(P2_0))
def test_frozen_crossings_unchanged(jn):
    hc = P["harness"]["crossings"][jn]
    cross, d = P2_0[jn]
    assert tuple(hc["cross"]) == pytest.approx(cross) and hc["d"] == d
    pin, pout = _paths(jn)
    assert pin[-1] == pytest.approx(cross) and pout[0] == pytest.approx(cross)


def test_inter_module_crossing_only_extended():
    """neck_pitch (trunk -> neck) P2-0 parcalari aynen kalir; P2c yalniz neck link'inde uzatir."""
    r = H["harness_route"]["neck_pitch"]
    assert not ({"in", "out"} & set(r)) and not r.get("in_pre")
    pin, pout = _paths("neck_pitch")
    hc = P["harness"]["crossings"]["neck_pitch"]
    assert pin == [tuple(q) for q in hc["in"]] + [tuple(hc["cross"])]
    assert pout[:1 + len(hc["out"])] == [tuple(hc["cross"])] + [tuple(q) for q in hc["out"]]


@pytest.mark.parametrize("a,b", JUNCTIONS)
def test_route_junctions_collinear(a, b):
    """Onceki gecisin out ucu = sonrakinin in basi ve son/ilk parcalar ayni yonde (tupler yalniz uc diskte degisir,
    ayni link'te cakisma hacmi yok)."""
    out_a = _paths(a)[1]
    in_b = _paths(b)[0]
    assert out_a[-1] == pytest.approx(in_b[0])
    ua, ub = _unit(out_a[-2], out_a[-1]), _unit(in_b[0], in_b[1])
    assert sum(x * y for x, y in zip(ua, ub)) == pytest.approx(1.0, abs=1e-6)
    # ilk ara noktadaki kure (yaricap = sonraki tup yaricapi) onceki tupun bitisinden uzakta kalir
    d_b = P["harness"]["crossings"][b]["d"]
    assert math.dist(in_b[0], in_b[1]) >= d_b / 2.0 + 0.1


def test_limits_cover_required_within_travel():
    lim, req = CFG["joint_limits_deg"], CFG["required_deg"]
    for k in ("neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth"):
        assert lim[k][0] <= req[k][0] and req[k][1] <= lim[k][1], k
    assert lim["head_yaw"] == [-80.0, 80.0]
    assert build_config.travel_violations(CFG, lim) == []


def test_bumper_pad_inside_frozen_zone():
    zone = [e for e in P["iface"]["envelopes"]["neck"] if e[0] == "box" and e[1] > P["j_head_pitch"][0] + 12.0]
    assert len(zone) == 1
    _, x0, x1, y0, y1, z0, z1 = zone[0]
    bx0, bx1, by0, by1, bz0, bz1 = H["bumper"]["box"]
    assert x0 <= bx0 < bx1 <= x1 and y0 <= by0 < by1 <= y1 and z0 <= bz0 < bz1 <= z1
    tg = H["bumper"]["tongue"]
    assert by0 < -tg[1] and tg[1] < by1 and bz0 < tg[2] < tg[3] < bz1


@pytest.mark.parametrize("key", ["head_pitch", "head_yaw", "head_roll", "mouth"])
def test_horn_toward_perpendicular(key):
    h = P["servo_dir"][key][0]
    u = H["horn_toward"][key]
    assert abs(sum(a * b for a, b in zip(h, u))) < 1e-9


def test_mass_budget_keys():
    assert H["mass_g"]["idler_pin"] == 1.8
    assert servos.get(CFG["servo_map"]["mouth"])["name"] == "SG90"
