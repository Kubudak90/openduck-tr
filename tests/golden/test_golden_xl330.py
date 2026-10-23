# -*- coding: utf-8 -*-
"""
xl330_27 altin regresyonu (spec 3.2.1, A1/A9): CAD'i freecadcmd ile gecici dizine derler (tools/golden.py)
ve tests/golden/xl330_27/ ile karsilastirir. Metin dosyalari bayt bayt; sim mesh STL'leri meshes.json ile
(once sha256, yalniz tessellation sirasi farkliysa ucgen sayisi + hacim/bbox 1e-9 bagil).

  pytest tests/golden -q -p no:cacheprovider                    # mevcut kod (~70 s: derleme ~4 s + check ~65 s)
  OPENDUCK_CONFIG=configs/xl330_27.json pytest tests/golden -q  # refactor sonrasi config ile
  OPENDUCK_GOLDEN_CHECK=0 pytest tests/golden -q                # cad/check.py'yi atla (interference.txt skip)

Altin bu makinede (meshes.json["meta"]) refactor oncesi kodla uretildi; baska platformda sayisal artik farklari
beklenir (hata mesaji "yalniz sayisal; max |fark|" ile gosterir). freecadcmd yoksa test atlanir.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "tools"))

import golden  # noqa: E402

GOLDEN = os.path.join(HERE, "xl330_27")
CHECK = os.environ.get("OPENDUCK_GOLDEN_CHECK", "1") == "1"
pytestmark = pytest.mark.slow


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    fc = golden.find_freecadcmd()
    if fc is None:
        pytest.skip("freecadcmd bulunamadi (FREECADCMD=/yol/freecadcmd ile belirtin)")
    out = str(tmp_path_factory.mktemp("xl330_27"))
    times = golden.build(out, config=os.environ.get("OPENDUCK_CONFIG") or None, check=CHECK, fc=fc)
    sys.stdout.write("\nderleme %s -> %s\n" % (times, out))
    return out


@pytest.mark.parametrize("rel", golden.GOLDEN_FILES)
def test_file_bytes(built, rel):
    if rel == "interference.txt" and not CHECK:
        pytest.skip("OPENDUCK_GOLDEN_CHECK=0")
    err = golden.compare_file(GOLDEN, built, rel)
    assert err is None, err


def test_sim_meshes(built):
    ref = golden.load_manifest(GOLDEN)
    errs, notes = golden.compare_meshes(ref["meshes"], golden.mesh_manifest(built))
    for n in notes:
        sys.stdout.write(n + "\n")
    assert not errs, "altin: %s\n%s" % (ref["meta"], "\n".join(errs))
