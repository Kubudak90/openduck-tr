# -*- coding: utf-8 -*-
"""
TR-35 govde verisi (P2b; FreeCAD gerektirmez): cad/electronics.py kayit defteri ve params.trunk yerlesimi. Her parca
kaynakli ve kutleli, adetler yerlesimle tutarli, yerlesim kutulari K_T kutularinin birlesiminde (rotor bolgelerinin
disinda), isi kaynaklari buck + XL4015, batarya kalca demeti giris tupune carpmiyor, hava yolu acikliklari var.

  pytest tests/cad -q -p no:cacheprovider
"""
import collections
import math

import build_config
import electronics as E

CFG = build_config.load("tr35_b2")
P = CFG["params"]
T = P["trunk"]
ENV = P["iface"]["envelopes"]["trunk"]


def _in_box(q, b, eps=1e-6):
    return all(b[2 * i] - eps <= q[i] <= b[2 * i + 1] + eps for i in range(3))


def _in_kt(q):
    """Nokta K_T'de mi: kutulardan birinde (silindir ilkelleri disinda) ve rotor kesicilerinin disinda."""
    inside = any(_in_box(q, prim[1:7]) for prim in ENV["add"] if prim[0] == "box")
    for prim in ENV["cut"]:
        if prim[0] == "box" and _in_box(q, prim[1:7], -1e-6):
            return False
        if prim[0] == "cyl" and prim[1] == "z":
            _, _, cx, cy, ri, ro, a0, a1 = prim
            r = math.hypot(q[0] - cx, q[1] - cy)
            if a0 + 1e-6 < q[2] < a1 - 1e-6 and ri + 1e-6 < r < ro - 1e-6:
                return False
    return inside


def test_parts_registry():
    for name, rec in E.PARTS.items():
        assert len(rec["size"]) == 3 and min(rec["size"]) > 0, name
        assert rec["mass_g"] > 0 and rec["qty"] >= 1 and rec["source"], name
        assert rec["form"] in ("box", "cyl", "switch"), name
        assert set(rec["est"]) <= {"size", "mass_g"}, (name, rec["est"])
        if rec["form"] == "cyl":
            assert rec["size"][1] == rec["size"][2], name
    assert {n for n in E.PARTS if E.is_heat_source(n)} == {"buck_20a", "xl4015"}
    assert E.PARTS["battery_4s_lipo"]["size"] == (78.0, 40.0, 30.0) and E.PARTS["battery_4s_lipo"]["mass_g"] == 200.0
    assert E.PARTS["buck_20a"]["size"] == (60.0, 53.0, 27.0)
    # P2-0 butcesi 385 g; P2b toplam (adetlerle) bunun altinda
    assert E.total_mass_g() <= 385.0


def test_placement_counts_match_registry():
    cnt = collections.Counter(E.part_of(n) for n in T["electronics"])
    assert set(cnt) == set(E.PARTS), set(E.PARTS) ^ set(cnt)
    for part, n in cnt.items():
        assert n == E.PARTS[part]["qty"], part


def test_placed_box_rotation_and_perm():
    assert E.placed_size("pca9685", "yzx") == (25.4, 7.0, 62.5)
    bx = E.placed_box("xl4015", [0.0, 0.0, 0.0, 90])
    assert (bx[1] - bx[0], bx[3] - bx[2], bx[5] - bx[4]) == (24.0, 54.0, 16.0)


def test_placements_inside_trunk_keep_in():
    """Her yerlesim kutusunun 27 orneklem noktasi (koseler, kenar/yuz ortalari, merkez) K_T'de (P2b parcalari K_T icinde
    olmali; tam hacim kontrolu cad/check.py bolum 6 ve test_trunk_fc)."""
    for name, pl in T["electronics"].items():
        b = E.placed_box(name, pl)
        for fx in (0.0, 0.5, 1.0):
            for fy in (0.0, 0.5, 1.0):
                for fz in (0.0, 0.5, 1.0):
                    q = (b[0] + fx * (b[1] - b[0]), b[2] + fy * (b[3] - b[2]), b[4] + fz * (b[5] - b[4]))
                    assert _in_kt(q), (name, q)


def test_battery_clears_hip_harness_and_keep_in():
    bat = E.placed_box("battery_4s_lipo", T["electronics"]["battery_4s_lipo"])
    hc = P["harness"]["crossings"]["hip_yaw"]
    x_in = hc["in"][0][0] - hc["d"] / 2.0
    assert bat[1] < x_in, "batarya on yuzu kalca demeti giris tupunden once bitmeli"
    assert bat[0] >= ENV["add"][7][1] - 1e-6, "batarya K_T arka sinirinda"


def test_heat_sources_away_from_shell_planes():
    """Isi kaynagi kutulari ile kabuk duzlemleri arasinda >= 5 mm (yan duvar ic yuzleri, arka yuz). Kati kontrolu
    test_trunk_fc."""
    gap = CFG["targets"]["pla_heat_gap_mm"]
    w = P["wall"]
    S = T["struct"]
    y_low_in = ENV["add"][1][4] - w           # low_l: ["box", x0, x1, y0, y1, z0, z1]
    y_back = S["shell_back_y0"]
    z_floor = ENV["add"][0][5] + S["floor_t"]
    for name in ("buck_20a", "xl4015"):
        b = E.placed_box(name, T["electronics"][name])
        ym = max(abs(b[2]), abs(b[3]))
        if b[4] >= z_floor:          # taban ustu (buck): kabuk duzlemlerine yatay aciklik
            assert y_low_in - ym >= gap - 1e-6 and y_back - ym >= gap - 1e-6, name
        else:                        # taban alti (XL4015): kabugun alt kenarlarina capraz aciklik
            dz = z_floor - b[5]
            assert math.hypot(max(y_low_in - ym, 0.0), dz) >= gap - 1e-6, name
            assert math.hypot(max(y_back - ym, 0.0), dz + S["shell_back_z0"]) >= gap - 1e-6, name


def test_airflow_openings():
    """Buck kaputu: ust izgara ve arka/yan girisler, XL4015 kanali uclari acik (K_T gobek kutusu)."""
    H = T["struct"]["hood"]
    gx0, gx1, gy, gw, gp = H["grille"]
    n = int((gx1 - gx0 - gw) // gp) + 1
    # 5 W dogal tasinim: Q / (rho cp dT) ~ 2.1e-4 m3/s, baca hizi ~0.2 m/s -> ~1000 mm2 cikis ve giris
    assert n * gw * 2 * gy >= 1000.0, "izgara acikligi (mm2)"
    win = H["ports"][-1]                     # arka duvar buck penceresi (buck arka ucu dogrudan disariya)
    bk = E.placed_box("buck_20a", T["electronics"]["buck_20a"])
    # pencereden giren hava buck modulunun acik PCB yuzunden gecer (acik oran ~%50 tahmin)
    back = 0.5 * (win[3] - win[2]) * (win[5] - win[4])
    assert win[2] < bk[2] and win[3] > bk[3] and win[4] < bk[4] and win[5] > bk[5], "pencere buck kesitini cevreler"
    assert bk[0] < win[1], "buck arka ucu pencerede"
    sx0, sx1, sz0, sz1, sw, sp = H["intake_side"]
    side = 2 * (sz1 - sz0) * sw * (int((sx1 - sx0 - sw) // sp) + 1)
    assert back + side >= 1000.0, "giris acikligi (mm2)"
    buck = E.placed_box("buck_20a", T["electronics"]["buck_20a"])
    assert gx0 >= buck[0] and gx1 <= buck[1], "izgara buck'in ustunde"


def test_wiring_starts_at_frozen_harness_points():
    W = T["wiring"]["bundles"]
    hip = P["harness"]["crossings"]["hip_yaw"]["in"][0]
    neck = P["harness"]["crossings"]["neck_pitch"]["in"][0]
    assert W["left_hip_trunk_wiring"]["pts"][0] == hip
    assert W["right_hip_trunk_wiring"]["pts"][0] == [hip[0], -hip[1], hip[2]]
    assert W["neck_trunk_wiring"]["pts"][0] == neck
    assert W["left_hip_trunk_wiring"]["d"] == P["harness"]["crossings"]["hip_yaw"]["d"]
    assert W["neck_trunk_wiring"]["d"] == P["harness"]["crossings"]["neck_pitch"]["d"]
