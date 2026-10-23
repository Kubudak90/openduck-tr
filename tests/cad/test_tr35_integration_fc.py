# -*- coding: utf-8 -*-
"""
TR-35 P2 entegrasyonu (P2a + P2b + P2c birlikte; docs/workflow/openduck-tr/p2-entegrasyon.md): orkestrator kararlariyla
degisen arayuz degerleri (K_T ICR-1/ICR-2 kutulari, boyun tampon govdesi kutusu, yeni jaw zarfi, hip_pitch kablo
giris noktasi) ve butun modullerin baski parcalari tek kati + 256 mm kupte (freecadcmd yoksa FreeCAD testi atlanir).

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

CFG = build_config.load("tr35_b2")
ENV = CFG["params"]["iface"]["envelopes"]


def test_accepted_interface_changes():
    """p2-entegrasyon.md 2: kabul edilen degisiklik talepleri konfigde."""
    assert ["box", 19.9, 26.2, 21.0, 26.65, 34.4, 39.7] in ENV["trunk"]["add"]          # ICR-1
    assert ["box", -95.0, -92.0, -24.0, 24.0, -3.0, 31.0] in ENV["trunk"]["add"]        # ICR-2
    assert ["box", 45.332, 51.5, -9.5, 9.5, 84.0, 98.0] in ENV["neck"]                  # tampon govdesi
    assert len(ENV["jaw"]) == 6 and ENV["jaw"][0] == ["box", 15.0, 52.0, 50.0, 57.0, 140.0, 164.0]
    assert CFG["params"]["harness"]["crossings"]["hip_pitch"]["in"] == [[5.128, 44.5, -13.5]]   # P2a ICR
    # gerekli araliklar genisletilmis sinirlarin icinde
    for k, (lo, hi) in CFG["required_deg"].items():
        a, b = CFG["joint_limits_deg"][k]
        assert a <= lo and hi <= b, k


SCRIPT = r'''
import json, os, sys
ROOT, OUT = %(root)r, %(out)r
sys.path.insert(0, os.path.join(ROOT, "cad"))
import openduck
m = openduck.OpenDuck(config="tr35_b2")
res = {}
for ln, l in m.links.items():
    for it in l.items:
        if it.kind in openduck.PRINT_KINDS:
            b = it.shape.BoundBox
            res[it.name] = [len(it.shape.Solids), max(b.XLength, b.YLength, b.ZLength), it.module]
with open(os.path.join(OUT, "res.json"), "w") as f:
    json.dump(res, f, indent=1)
'''


@pytest.mark.slow
def test_print_parts_single_solid_on_bed(tmp_path):
    fc = golden.find_freecadcmd()
    if fc is None:
        pytest.skip("freecadcmd bulunamadi (FREECADCMD=/yol/freecadcmd ile belirtin)")
    script = tmp_path / "run.py"
    script.write_text(SCRIPT % {"root": ROOT, "out": str(tmp_path)})
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    env.pop("OPENDUCK_CONFIG", None)
    log = tmp_path / "run.log"
    with open(log, "w") as lf:
        subprocess.run([fc, str(script)], cwd=ROOT, env=env, stdout=lf, stderr=subprocess.STDOUT, timeout=900)
    path = tmp_path / "res.json"
    assert path.exists(), log.read_text()[-4000:]
    res = json.loads(path.read_text())
    assert {md for _, _, md in res.values()} == {"trunk", "legs", "head"}
    bed = min(CFG["targets"]["bed_mm"])
    assert {n: v for n, v in res.items() if v[0] != 1} == {}
    assert {n: v for n, v in res.items() if v[1] > bed} == {}
