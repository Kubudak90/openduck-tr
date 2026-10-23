# -*- coding: utf-8 -*-
"""
P2-0 servo zarflari (servos.canon / idler_parts / horn_pocket), freecadcmd ile (yoksa atlanir): olculer kayitla
ayni (govde, kulak acikligi, spline ustu, horn yuzu), kanonik orijin, X = 0 simetrisi (mirror_y gecerliligi),
govde/horn/idler parcalari birbirine girmez, horn civata delikleri bulunur, XL330 eski zarf hacmi degismez.

  pytest tests/cad -q -p no:cacheprovider          (~10 s)
"""
import json
import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import golden  # noqa: E402
import servos  # noqa: E402

pytestmark = pytest.mark.slow
PWM = ("DS32XX", "ES3054", "MG90S", "SG90")

SCRIPT = r'''
import json, os, sys
ROOT, OUT = %(root)r, %(out)r
sys.path.insert(0, os.path.join(ROOT, "cad"))
import FreeCAD as App
import servos
bb6 = lambda s: [s.BoundBox.XMin, s.BoundBox.XMax, s.BoundBox.YMin, s.BoundBox.YMax, s.BoundBox.ZMin, s.BoundBox.ZMax]
mx = App.Matrix(); mx.A11 = -1
res = {}
for name in ("XL330", "DS32XX", "ES3054", "MG90S", "SG90"):
    t = servos.get(name)
    b, h = servos.canon(t)
    r = {"body_bb": bb6(b), "horn_bb": bb6(h), "body_vol": b.Volume, "horn_vol": h.Volume,
         "valid": b.isValid() and h.isValid(), "solids": [len(b.Solids), len(h.Solids)],
         "body_horn": b.common(h).Volume,
         "sym": [b.common(b.transformGeometry(mx)).Volume / b.Volume,
                 h.common(h.transformGeometry(mx)).Volume / h.Volume],
         "holes": len(servos.horn_bolt_holes(t, h, -60, 10))}
    ip = servos.idler_parts(t)
    if ip:
        st, ro, ring = ip
        r["idler"] = {"bb": [bb6(s) for s in ip], "body": [s.common(b).Volume for s in ip],
                      "horn": [s.common(h).Volume for s in ip], "st_ro": st.common(ro).Volume,
                      "st_ring": st.common(ring).Volume, "ro_ring": ro.common(ring).Volume}
    if t["geometry"] == "pwm":
        pk = servos.horn_pocket(t, 0.2, depth=t["horn"]["h"][1] - t["horn"]["h"][0])
        r["pocket"] = {"outside": h.cut(pk).Volume, "bb": bb6(pk), "body": pk.common(b).Volume}
    res[name] = r
with open(os.path.join(OUT, "geom.json"), "w") as f:
    json.dump(res, f)
'''


@pytest.fixture(scope="module")
def geom(tmp_path_factory):
    fc = golden.find_freecadcmd()
    if fc is None:
        pytest.skip("freecadcmd bulunamadi (FREECADCMD=/yol/freecadcmd ile belirtin)")
    out = str(tmp_path_factory.mktemp("servo_geom"))
    script = os.path.join(out, "run.py")
    with open(script, "w") as f:
        f.write(SCRIPT % {"root": ROOT, "out": out})
    log = os.path.join(out, "run.log")
    with open(log, "w") as lf:
        subprocess.run([fc, script], cwd=ROOT, env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"), stdout=lf,
                       stderr=subprocess.STDOUT, timeout=600)
    path = os.path.join(out, "geom.json")
    assert os.path.exists(path), open(log).read()[-4000:]
    with open(path) as f:
        return json.load(f)


def test_xl330_legacy_envelope(geom):
    """XL330 kanonik zarfi eski modelle ayni (orijin yigin ortasinda: govde -14.5..11.5, horn 11.5..14.5)."""
    r = geom["XL330"]
    assert r["body_bb"] == pytest.approx([-10, 10, -14.5, 11.5, -9.5, 24.5])
    assert r["horn_bb"] == pytest.approx([-6.5, 6.5, 11.5, 14.5, -6.5, 6.5])
    assert r["holes"] == 4 and r["valid"]


@pytest.mark.parametrize("name", PWM)
def test_pwm_envelope_dimensions(geom, name):
    t = servos.get(name)
    r = geom[name]
    b, tb, sp, hn = t["body"], t["tabs"], t["spline"], t["horn"]
    x0, x1, y0, y1, z0, z1 = r["body_bb"]
    assert r["valid"] and r["solids"] == [1, 1]
    assert (x0, x1) == pytest.approx((-b["w"] / 2, b["w"] / 2))                     # genislik (kulak = govde eni)
    assert y0 == pytest.approx(-hn["h"][1]) and y1 == pytest.approx(sp["top_h"] - hn["h"][1])   # taban, spline ustu
    zc = b["l"] / 2 - t["shaft"]["near"]
    assert (z0, z1) == pytest.approx((zc - tb["span"] / 2, zc + tb["span"] / 2))   # kulak acikligi
    hx0, hx1, hy0, hy1, hz0, hz1 = r["horn_bb"]
    assert hy1 == pytest.approx(0.0) and hy0 == pytest.approx(hn["h"][0] - hn["h"][1])   # orijin horn dis yuzunde
    assert (hx0, hx1) == pytest.approx((-hn["hub_d"] / 2, hn["hub_d"] / 2))
    reach = max(abs(z) + rt for z, rt in servos.horn_arm_ends(t))
    assert max(abs(hz0), abs(hz1)) == pytest.approx(reach)
    # hacim: govde kutusu + kulaklar mertebesinde (kose yuvarlatma ve delikler dusuk)
    assert 0.9 * b["l"] * b["w"] * b.get("main_h", b["h"]) < r["body_vol"] < 1.25 * b["l"] * b["w"] * b["h"]


@pytest.mark.parametrize("name", PWM)
def test_pwm_envelope_symmetry_and_contacts(geom, name):
    """X = 0 duzlemine gore simetri (sag taraf mirror_y = donmus gercek servo), govde ile horn ust uste binmez,
    horn civata delikleri (bolt_n) bulunur, horn cebi kolu ve gobegi kapsar."""
    t = servos.get(name)
    r = geom[name]
    assert r["sym"][0] == pytest.approx(1.0, abs=1e-6) and r["sym"][1] == pytest.approx(1.0, abs=1e-6)
    assert r["body_horn"] < 1e-6
    assert r["holes"] == t["horn"]["bolt_n"]
    assert r["pocket"]["outside"] < 1e-6                     # gobek boyu derinlikte cep horn'u tamamen kapsar
    assert r["pocket"]["bb"][3] == pytest.approx(0.0)        # cep horn dis yuzunde biter (over = 0)


def test_ds_idler_parts(geom):
    """DS idler: stator yuvasi, rotor pimi ve rulman govdeye/horna/birbirine girmez; hepsi mil ekseninde (X, Z
    simetrik) ve tabanin altinda (Y < arka yuz)."""
    r = geom["DS32XX"]["idler"]
    yr = servos.rear_face_y(servos.get("DS32XX"))
    for bb in r["bb"]:
        assert bb[0] == pytest.approx(-bb[1]) and bb[4] == pytest.approx(-bb[5])
        assert bb[3] <= yr + 1e-6
    assert max(r["body"]) < 1e-6 and max(r["horn"]) < 1e-6
    assert r["st_ro"] < 1e-6 and r["st_ring"] < 1e-6 and r["ro_ring"] < 1e-6
    for n in ("ES3054", "MG90S", "SG90", "XL330"):
        assert "idler" not in geom[n]
