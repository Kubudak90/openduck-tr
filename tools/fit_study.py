# -*- coding: utf-8 -*-
"""
Zarf yerlesim calismasi (P2-0, A1 hazirligi): yalniz servo zarflari (govde + kulak + gobek/spline + rakor, horn,
DS idler yuvasi/rulmani/pimi, doner tabla rulmanlari ve mili) ile farkli link'ler arasindaki EN KUCUK BOSLUK ve
carpismalar. Pozlar (cad/check.py ureteclerinden, gerekli araliklar): HOME + tek eklem (7 adim) + komsu cift
izgarasi (5 x 5) + bacak-bacak (32). Rapor: carpismalar, braket payindan (targets.bracket_clearance_mm, varsayilan
plate_t + 1) kucuk bosluklu ciftler ve her link cifti icin en kucuk bosluk.

  OPENDUCK_CONFIG=tr35_b2 OPENDUCK_FIT_OUT=<dosya> freecadcmd -c "import runpy; runpy.run_path('tools/fit_study.py',
      run_name='__main__')"
  OPENDUCK_FIT_SETS=single,pairs,legleg   (varsayilan hepsi)   OPENDUCK_FIT_MARGIN=12 (mm, bbox on eleme)
"""
import collections
import os
import sys
import time

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = os.path.join(os.getcwd(), "tools")
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "cad"))

import check  # noqa: E402
import openduck  # noqa: E402

ENVELOPE_KINDS = ("servo", "horn", "metal")
JOINT_KEYS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle", "neck_pitch", "head_pitch", "head_yaw",
              "head_roll", "mouth")


def is_envelope(it):
    """Zarf parcasi: servo govdesi, horn, metal (rulman, idler pimi, doner tabla mili) ve idler yuvasi / doner
    tabla yatagi (arayuz keep-out'u)."""
    return it.kind in ENVELOPE_KINDS or "idler" in it.name or "housing" in it.name


def joint_group(name):
    """Parcanin ait oldugu eklem (taraf onekli; or. left_knee) ya da None: ayni eklemin govde/horn/idler/doner tabla
    parcalari eklem ici kabul edilir (es eksenli temas; bosluk raporuna girmez, hacimli carpisma yine sayilir)."""
    side = ""
    for pre in ("left_", "right_"):
        if name.startswith(pre):
            side, name = pre, name[len(pre):]
    for k in JOINT_KEYS:
        if name.startswith(k + "_"):
            return side + k
    return None


def bb_gap(a, b):
    dx = max(0.0, a.XMin - b.XMax, b.XMin - a.XMax)
    dy = max(0.0, a.YMin - b.YMax, b.YMin - a.YMax)
    dz = max(0.0, a.ZMin - b.ZMax, b.ZMin - a.ZMax)
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def study(model, poses, margin, keep=None):
    """[(poz adi, poz)] -> (en kucuk bosluk {(a, b): (mm, poz)}, carpismalar [(poz, hacim, a, b)])."""
    best, hits = {}, []
    for pname, pose in poses:
        sh = [(ln, it, s) for ln, it, s in model.posed_shapes(pose) if is_envelope(it)]
        for i in range(len(sh)):
            la, ia, sa = sh[i]
            for lb, ib, sb in sh[i + 1:]:
                if la == lb:
                    continue
                if keep is not None and la not in keep and lb not in keep:
                    continue
                g = bb_gap(sa.BoundBox, sb.BoundBox)
                if g > margin:
                    continue
                d = sa.distToShape(sb)[0]
                if d < 1e-6:
                    v = sa.common(sb).Volume
                    if v > check.TOL:
                        hits.append((pname, v, ia.name, ib.name))
                ga = joint_group(ia.name)
                if ga is not None and ga == joint_group(ib.name):
                    continue
                key = tuple(sorted((ia.name, ib.name)))
                if key not in best or d < best[key][0]:
                    best[key] = (d, pname)
    return best, hits


def run():
    model = openduck.OpenDuck()
    req = "required"
    sets = os.environ.get("OPENDUCK_FIT_SETS", "single,pairs,legleg").split(",")
    margin = float(os.environ.get("OPENDUCK_FIT_MARGIN", "12"))
    need = float(model.cfg.get("targets", {}).get("bracket_clearance_mm", model.p["plate_t"] + 1.0))
    poses = []
    if "single" in sets:
        poses += check.single_poses(model, req, 7)
    if "pairs" in sets:
        poses += check.pair_poses(model, 5, req)
    if "legleg" in sets:
        poses += check.leg_leg_poses(model, req)
    t0 = time.time()
    best, hits = study(model, poses, margin)
    dt = time.time() - t0
    link_of = {it.name: ln for ln, l in model.links.items() for it in l.items}
    L = ["# Zarf yerlesim calismasi (%s)" % model.cfg["name"], "",
         "Pozlar: %d (%s; gerekli araliklar), sure %.0f s; zarf parcasi %d; bbox on eleme %.0f mm; braket payi %.1f mm"
         % (len(poses), ", ".join(sets), dt, sum(1 for l in model.links.values() for it in l.items if is_envelope(it)),
            margin, need), ""]
    L.append("## Carpismalar (farkli link zarflari, > %.0f mm3)" % check.TOL)
    agg = collections.OrderedDict()
    for pname, v, a, b in hits:
        k = (a, b)
        if k not in agg or v > agg[k][0]:
            agg[k] = (v, pname)
    for (a, b), (v, pname) in sorted(agg.items(), key=lambda kv: -kv[1][0]):
        L.append("  %8.1f mm3  %s <-> %s  (en kotu: %s; %d poz)" % (v, a, b, pname,
                                                                  sum(1 for h in hits if (h[2], h[3]) == (a, b))))
    L.append("  -> %d carpisma (%d cift)" % (len(hits), len(agg)))
    L += ["", "## Braket payindan kucuk bosluklar (< %.1f mm)" % need]
    tight = sorted((d, k, pn) for k, (d, pn) in best.items() if d < need)
    for d, (a, b), pn in tight:
        L.append("  %6.2f mm  %s [%s] <-> %s [%s]  (%s)" % (d, a, link_of[a], b, link_of[b], pn))
    L.append("  -> %d cift" % len(tight))
    L += ["", "## Link cifti basina en kucuk bosluk (zarflar, < %.0f mm)" % margin]
    per = {}
    for (a, b), (d, pn) in best.items():
        k = tuple(sorted((link_of[a], link_of[b])))
        if k not in per or d < per[k][0]:
            per[k] = (d, a, b, pn)
    for k, (d, a, b, pn) in sorted(per.items(), key=lambda kv: kv[1][0]):
        L.append("  %6.2f mm  %-24s %s <-> %s  (%s)" % (d, "%s/%s" % k, a, b, pn))
    return len(hits), tight, "\n".join(L)


if __name__ == "__main__":
    n_hit, tight, txt = run()
    out = os.environ.get("OPENDUCK_FIT_OUT")
    if out:
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        with open(out, "w") as f:
            f.write(txt + "\n")
    sys.stdout.write(txt + "\n")
