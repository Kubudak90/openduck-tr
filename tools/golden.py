# -*- coding: utf-8 -*-
"""
Altin referans (golden) yardimcisi: CAD'i freecadcmd ile bir dizine derler (tools/build.sh gibi, GUI'siz,
OPENDUCK_DOCS=0), sim mesh'lerinin manifestini cikarir ve ciktiyi tests/golden/<ad>/ ile karsilastirir.
tests/golden/test_golden_xl330.py bunu kullanir. Normal Python ile calisir (FreeCAD modulu gerekmez).

  python tools/golden.py build <out> [--config X] [--no-check]
  python tools/golden.py compare <altin_dizin> <out>
  python tools/golden.py freeze <altin_dizin> [--from <out>] [--config X]   # altini yeniden yazar!

Altin, uretildigi platforma ve FreeCAD surumune ozgudur: Linux ve macOS derlemeleri sifira yakin terimlerde
(~1e-18, or. eylemsizlik capraz terimleri) farkli kayan nokta artiklari verir. meshes.json["meta"] nerede
uretildigini yazar. freeze yalniz refactor ONCESI kodla calistirilir (spec 3.2.1).
"""
import argparse
import difflib
import hashlib
import json
import os
import platform
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GOLDEN_FILES = ("sim/openduck.xml", "sim/openduck.urdf", "summary.json", "BOM.md", "interference.txt")
MESH_DIR = "sim/meshes"
MANIFEST = "meshes.json"
REL_TOL = 1e-9
FC_CANDIDATES = ("freecadcmd", "FreeCADCmd", "/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd",
                 "/opt/freecad-dl/squashfs-root/usr/bin/freecadcmd")
NUM = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?")


def find_freecadcmd():
    """FREECADCMD, PATH ve bilinen kurulum yerleri; yoksa None."""
    for c in (os.environ.get("FREECADCMD"),) + FC_CANDIDATES:
        p = c and shutil.which(c)
        if p:
            return p
    return None


def freecad_version(fc):
    try:
        out = subprocess.run([fc, "--version"], capture_output=True, text=True, timeout=120).stdout
        return out.strip().splitlines()[0]
    except Exception:
        return "?"


def _run_fc(fc, script, env, log_path, timeout, main=False):
    code = "import runpy; runpy.run_path(%r%s)" % (script, ", run_name='__main__'" if main else "")
    with open(log_path, "w") as log:
        return subprocess.run([fc, "-c", code], cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                              timeout=timeout).returncode


def build(out, config=None, check=True, fc=None):
    """out/ altina derler (+ check=True ise out/interference.txt). Sureleri (s) dondurur; hata -> RuntimeError."""
    fc = fc or find_freecadcmd()
    if fc is None:
        raise RuntimeError("freecadcmd bulunamadi (FREECADCMD=/yol/freecadcmd)")
    out = os.path.abspath(out)
    os.makedirs(out, exist_ok=True)
    env = dict(os.environ, OPENDUCK_ROOT=ROOT, OPENDUCK_OUT=out, OPENDUCK_DOCS="0", OPENDUCK_RENDER="0",
               OPENDUCK_RANDOM_POSES="40", PYTHONDONTWRITEBYTECODE="1")
    env.pop("OPENDUCK_CONFIG", None)
    if config:
        env["OPENDUCK_CONFIG"] = config
    times = {}
    t0 = time.time()
    rc = _run_fc(fc, os.path.join(ROOT, "tools", "build_gui.py"), env, os.path.join(out, "build.stdout"), 1800)
    times["build_s"] = round(time.time() - t0, 1)
    blog = os.path.join(out, "build.log")
    text = open(blog).read() if os.path.exists(blog) else ""
    if "bitti" not in text:
        raise RuntimeError("derleme basarisiz (rc=%d, %s):\n%s" % (rc, blog, text[-4000:]))
    if check:
        env["OPENDUCK_CHECK_OUT"] = os.path.join(out, "interference.txt")
        t0 = time.time()
        rc = _run_fc(fc, os.path.join(ROOT, "cad", "check.py"), env, os.path.join(out, "check.stdout"), 3600,
                     main=True)
        times["check_s"] = round(time.time() - t0, 1)
        if not os.path.exists(env["OPENDUCK_CHECK_OUT"]):
            raise RuntimeError("cad/check.py basarisiz (rc=%d), bkz. %s" % (rc, os.path.join(out, "check.stdout")))
    return times


# ---------------------------------------------------------------------------
# STL manifesti
# ---------------------------------------------------------------------------
def _triangles(data):
    n = struct.unpack("<I", data[80:84])[0] if len(data) >= 84 else -1
    if len(data) == 84 + 50 * n:     # ikili STL
        for t in struct.iter_unpack("<12fH", data[84:]):
            yield t[3:6], t[6:9], t[9:12]
        return
    v = [tuple(float(c) for c in ln.split()[1:4]) for ln in data.decode("ascii", "replace").splitlines()
         if ln.strip().startswith("vertex")]
    for i in range(0, len(v) - 2, 3):
        yield v[i], v[i + 1], v[i + 2]


def stl_stats(path):
    """sha256, ucgen sayisi, isaretli hacim (mm3) ve bbox [xmin, ymin, zmin, xmax, ymax, zmax] (mm)."""
    data = open(path, "rb").read()
    n, vol = 0, 0.0
    lo, hi = [float("inf")] * 3, [float("-inf")] * 3
    for a, b, c in _triangles(data):
        n += 1
        vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2])
                + a[2] * (b[0] * c[1] - b[1] * c[0]))
        for p in (a, b, c):
            for k in range(3):
                lo[k] = min(lo[k], p[k])
                hi[k] = max(hi[k], p[k])
    return {"sha256": hashlib.sha256(data).hexdigest(), "triangles": n, "volume": vol / 6.0, "bbox": lo + hi}


def mesh_manifest(out):
    d = os.path.join(out, MESH_DIR)
    return {fn: stl_stats(os.path.join(d, fn)) for fn in sorted(os.listdir(d)) if fn.endswith(".stl")}


def load_manifest(golden_dir):
    with open(os.path.join(golden_dir, MANIFEST)) as f:
        return json.load(f)


def compare_meshes(golden_meshes, meshes):
    """(hatalar, notlar). Once sha256; farkliysa ucgen sayisi esit ve hacim/bbox REL_TOL bagil icinde olmali."""
    errs, notes = [], []
    for fn in sorted(set(golden_meshes) - set(meshes)):
        errs.append("%s: eksik mesh" % fn)
    for fn in sorted(set(meshes) - set(golden_meshes)):
        errs.append("%s: fazla mesh" % fn)
    for fn in sorted(set(golden_meshes) & set(meshes)):
        a, b = golden_meshes[fn], meshes[fn]
        if a["sha256"] == b["sha256"]:
            continue
        if a["triangles"] != b["triangles"]:
            errs.append("%s: ucgen sayisi %d -> %d" % (fn, a["triangles"], b["triangles"]))
            continue
        scale = max(abs(c) for c in a["bbox"]) or 1.0    # sifira yakin koordinatlar mesh boyutuna gore
        dv = abs(a["volume"] - b["volume"])
        db = max(abs(x - y) for x, y in zip(a["bbox"], b["bbox"]))
        if dv <= REL_TOL * max(abs(a["volume"]), abs(b["volume"])) and db <= REL_TOL * scale:
            notes.append("%s: sha256 farkli, ucgen/hacim/bbox esit (yalniz tessellation sirasi)" % fn)
        else:
            errs.append("%s: hacim %.9g -> %.9g, bbox max fark %.3g mm" % (fn, a["volume"], b["volume"], db))
    return errs, notes


# ---------------------------------------------------------------------------
# Metin dosyalari
# ---------------------------------------------------------------------------
def describe_text_diff(a_bytes, b_bytes, max_lines=12):
    """Bayt farkinin ozeti: farkli satir sayisi, yalniz sayisal mi (max mutlak fark) ve unified diff basi."""
    a = a_bytes.decode("utf-8", "replace").splitlines()
    b = b_bytes.decode("utf-8", "replace").splitlines()
    head = ["%d -> %d satir" % (len(a), len(b))]
    if len(a) == len(b):
        rows = [i for i in range(len(a)) if a[i] != b[i]]
        numeric, worst = True, (0.0, 0)
        for i in rows:
            if NUM.sub("#", a[i]) != NUM.sub("#", b[i]):
                numeric = False
                break
            for x, y in zip(NUM.findall(a[i]), NUM.findall(b[i])):
                worst = max(worst, (abs(float(x) - float(y)), i + 1))
        head.append("%d satir farkli" % len(rows))
        if rows and numeric:
            head.append("yalniz sayisal; max |fark| = %.3g (satir %d)" % worst)
        elif not rows:
            head.append("satir sonu/kodlama farki")
    diff = list(difflib.unified_diff(a, b, "altin", "yeni", n=0, lineterm=""))
    return ", ".join(head) + "\n" + "\n".join(diff[:max_lines]) + ("\n..." if len(diff) > max_lines else "")


def compare_file(golden_dir, out, rel):
    """Bayt esitse None, degilse aciklama."""
    g, n = os.path.join(golden_dir, rel), os.path.join(out, rel)
    if not os.path.exists(g):
        return "%s: altin dosya yok (%s)" % (rel, g)
    if not os.path.exists(n):
        return "%s: derleme ciktisi yok (%s)" % (rel, n)
    a, b = open(g, "rb").read(), open(n, "rb").read()
    return None if a == b else "%s: bayt farkli: %s" % (rel, describe_text_diff(a, b))


def compare(golden_dir, out, files=GOLDEN_FILES):
    """Tum altin dosyalar + mesh manifesti: (hatalar, notlar)."""
    errs = [e for e in (compare_file(golden_dir, out, rel) for rel in files) if e]
    me, notes = compare_meshes(load_manifest(golden_dir)["meshes"], mesh_manifest(out))
    return errs + me, notes


def freeze(golden_dir, src=None, config=None):
    """Altini yeniden yazar: src (hazir derleme) ya da gecici derlemeden dosyalari + meshes.json kopyalar."""
    fc = find_freecadcmd()
    tmp = None
    times = {}
    if src is None:
        tmp = src = tempfile.mkdtemp(prefix="golden_")
        times = build(src, config=config, fc=fc)
    for rel in GOLDEN_FILES:
        dst = os.path.join(golden_dir, rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copyfile(os.path.join(src, rel), dst)
    meta = {"freecad": freecad_version(fc) if fc else "?", "platform": "%s %s" % (platform.system(),
            platform.machine()), "config": config, "mesh_dir": MESH_DIR, "units": "mm, mm3", "rel_tol": REL_TOL}
    meshes = mesh_manifest(src)
    rows = [' %s: %s' % (json.dumps(fn), json.dumps(meshes[fn], sort_keys=True)) for fn in sorted(meshes)]
    with open(os.path.join(golden_dir, MANIFEST), "w") as f:    # mesh basina bir satir: okunur git diff'i
        f.write('{"meta": %s,\n"meshes": {\n%s\n}}\n' % (json.dumps(meta, sort_keys=True), ",\n".join(rows)))
    if tmp:
        shutil.rmtree(tmp, ignore_errors=True)
    return times


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("out")
    b.add_argument("--config")
    b.add_argument("--no-check", action="store_true")
    c = sub.add_parser("compare")
    c.add_argument("golden")
    c.add_argument("out")
    z = sub.add_parser("freeze")
    z.add_argument("golden")
    z.add_argument("--from", dest="src")
    z.add_argument("--config")
    a = ap.parse_args(argv)
    if a.cmd == "build":
        print(json.dumps(build(a.out, config=a.config, check=not a.no_check)))
    elif a.cmd == "compare":
        errs, notes = compare(a.golden, a.out)
        for line in notes + errs:
            print(line)
        print("ESIT" if not errs else "%d FARK" % len(errs))
        return 1 if errs else 0
    else:
        print(json.dumps(freeze(a.golden, src=a.src, config=a.config)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
