# -*- coding: utf-8 -*-
"""
cad/route.py (FreeCAD gerektirmez): kablo demeti kose yuvarlatma, polyline bukulme yaricapi ve eklem gecisindeki
dinamik bukulme (p2-interface.md 6: bend_r = 4 d, dinamik). Diz gecisi (dogrulama 2, D1) icin konfig degerleri.

  pytest tests/cad/test_route.py -q
"""
import math
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import build_config  # noqa: E402
import route  # noqa: E402


def _arc_points_radius(pts):
    """Ardisik ucluden gecen cemberlerin en kucuk yaricapi (keskin kose yoksa ~ fillet yaricapi)."""
    rr = []
    for a, b, c in zip(pts, pts[1:], pts[2:]):
        ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
        u = [b[i] - a[i] for i in range(3)]
        w = [c[i] - a[i] for i in range(3)]
        cr = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
        area2 = math.sqrt(sum(x * x for x in cr))
        if area2 > 1e-9:
            rr.append(ab * bc * ca / (2.0 * area2))
    return min(rr) if rr else float("inf")


def test_fillet_right_angle():
    pts = [(0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 20.0, 0.0)]
    out = route.fillet(pts, [0.0, 5.0, 0.0])
    assert out[0] == pts[0] and out[-1] == pts[-1]
    # yay 10 derecelik dilimlerle yazilir: 90 derece -> 9 dilim, 10 yay noktasi + iki uc; teget noktalari koseden
    # r / cos(5) * tan(45) uzakta (dilim koseleri yaricapin disinda: ayrik yayin bukulme yaricapi r'nin altina dusmez)
    assert len(out) == 1 + 10 + 1
    t = 5.0 / math.cos(math.radians(5.0))
    assert out[1] == pytest.approx((20.0 - t, 0.0, 0.0), abs=1e-9)
    assert out[-2] == pytest.approx((20.0, t, 0.0), abs=1e-9)
    # ayrik polyline'in kabul ettigi bukulme yaricapi istenenin altina dusmez
    assert route.min_bend_radius(out) >= 5.0 - 1e-6
    assert _arc_points_radius(out) == pytest.approx(5.0 / math.cos(math.radians(5.0)), rel=1e-3)


def test_fillet_keeps_sharp_and_straight_corners():
    pts = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0)]
    assert route.fillet(pts, [0.0, 3.0, 0.0, 0.0]) == pts              # duz kose: degismez, r = 0: keskin
    with pytest.raises(ValueError):
        route.fillet([(0, 0, 0), (2, 0, 0), (2, 2, 0)], [0.0, 5.0, 0.0])  # teget boyu parcaya sigmaz


def test_min_bend_radius():
    assert route.min_bend_radius([(0, 0, 0), (10, 0, 0)]) == float("inf")
    # 90 derece kose, kisa kol 4 mm: r tan(45) <= 4
    assert route.min_bend_radius([(0, 0, 0), (4, 0, 0), (4, 30, 0)]) == pytest.approx(4.0)


def test_crossing_bend_planar():
    # y ekseni etrafinda donen cocuk: gecisten +x'e giden in kolu, -x'e cikan out; q = 0'da duz (beta 0)
    pin = [(-20.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    pout = [(0.0, 0.0, 0.0), (15.0, 0.0, 0.0)]
    b = route.crossing_bend(pin, pout, (0, 1, 0), -60.0, 30.0)
    assert b["L_in"] == pytest.approx(20.0) and b["L_out"] == pytest.approx(15.0)
    assert b["beta_max"] == pytest.approx(60.0, abs=0.5) and b["q0"] == pytest.approx(0.0, abs=0.5)
    assert b["R"] == pytest.approx(15.0 / math.tan(math.radians(30.0)), rel=1e-2)
    # kol ekseni etrafinda donus (burulma): beta sabit 90, kollar dogrultusu degismez
    t = route.crossing_bend([(0, 0, -10.0), (0, 0, 0)], [(0, 0, 0), (0, -12.0, 0)], (0, 0, 1), -80.0, 80.0)
    assert t["beta_max"] == pytest.approx(90.0) and t["R"] == pytest.approx(10.0)


def test_knee_crossing_meets_bend_radius():
    """Diz gecisi (D1, uylugun dis tarafi): diz sinirlarinda dinamik gecis yayi >= bend_r (4 d)."""
    cfg = build_config.load("tr35_b2")
    c = cfg["params"]["harness"]["crossings"]["knee"]
    rt = cfg["params"]["legs"]["harness_route"]["knee"]
    pts = lambda qs: [tuple(q[:3]) for q in qs]
    pin = pts(rt["in"]) + pts(c["in"]) + [tuple(c["cross"])]
    pout = [tuple(c["cross"])] + pts(c["out"]) + pts(rt["out"])
    lo, hi = cfg["joint_limits_deg"]["knee"]
    b = route.crossing_bend(pin, pout, (0, -1, 0), lo, hi)
    assert b["R"] >= c["bend_r"], b
    # gecis noktasi diz ekseninde, idler M3 basinin disinda (y > 111.8), in/out radyal (gecis duzleminde)
    assert c["cross"][1] > 111.8 and c["in"][0][1] == c["cross"][1] == c["out"][0][1]
