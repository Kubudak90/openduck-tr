# -*- coding: utf-8 -*-
"""
Carpisma (interference) kontrolu.

1) Tasarim kontrolu: sifir ve ayakta pozlari + her eklemin, digerleri ayakta
   pozundayken, kendi siniri boyunca taranmasi. Burada HIC carpisma olmamali.
2) Bilgi amacli: tum eklemlerin rastgele birlikte surulmesi. Uc kombinasyonlarda
   (or. bacak 80 derece one + 25 derece yana) govde kendine carpabilir; bunlar
   tasarim hatasi degil, kontrolcunun kacinmasi gereken pozlardir.

3) Ayni link'teki parcalar (HOME pozu; birlikte hareket ettikleri icin pozdan bagimsiz): basilan
   ve satin alinan tum parca ciftleri, kesisim hacmi > TOL olanlar raporlanir. Bilerek ic ice
   modellenen ciftler SAME_LINK_ALLOW listesindedir (her biri gerekcesiyle). 1) ve 2) farkli
   link'leri karsilastirir.

  freecadcmd -c "import runpy; runpy.run_path('cad/check.py', run_name='__main__')"
"""
import collections
import itertools
import os
import random
import sys

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = os.path.join(os.getcwd(), "cad")
sys.path.insert(0, HERE)

import openduck  # noqa: E402

TOL = 1.0   # mm^3 - degen yuzeyler (horn/servo temasi) bunun altinda kalir
SWEEP_STEPS = 7

# Ayni link'te bilerek ust uste modellenen parca ciftleri (Item.part adlari; sol/sag ayni).
# Servo/horn, horn/braket ve braket/servo temaslari yuz temasidir (0 mm^3), listede olmalari gerekmez.
SAME_LINK_ALLOW = {
    # M12 objektif kamera tutucusuna vidalanir; tutucu disli deligi olmadan dolu blok olarak
    # modellenmis (ikisi de satin alinan parca, baski geometrisini etkilemez).
    frozenset(("camera_lens", "camera_holder")): "M12 objektif tutucuya vidali (tutucu dolu blok)",
}


def design_poses(model):
    stand = model.pose("stand")
    poses = [("zero", model.pose("zero")), ("stand", stand)]
    for j in model.joints:
        for k in range(SWEEP_STEPS):
            a = j["lower"] + (j["upper"] - j["lower"]) * k / (SWEEP_STEPS - 1.0)
            p = dict(stand)
            p[j["name"]] = a
            poses.append(("%s=%.0f" % (j["name"], a), p))
    return poses


def random_poses(model, n, seed=1):
    rnd = random.Random(seed)
    return [("random_%02d" % i, {j["name"]: rnd.uniform(j["lower"], j["upper"]) for j in model.joints})
            for i in range(n)]


def check_pose(model, pose):
    shapes = model.posed_shapes(pose)
    hits = []
    for (la, ia, sa), (lb, ib, sb) in itertools.combinations(shapes, 2):
        if la == lb or not sa.BoundBox.intersect(sb.BoundBox):
            continue
        try:
            vol = sa.common(sb).Volume
        except Exception:   # OCC nadiren basarisiz olur; raporla
            vol = -1.0
        if vol > TOL or vol < 0:
            hits.append((vol, ia.name, ib.name))
    return hits


def same_link_overlaps(model):
    """HOME pozunda ayni link'teki parca ciftlerinin kesisimleri: [(link, hacim, a, b, izin_gerekcesi|None)]."""
    hits = []
    for ln, link in model.links.items():
        for ia, ib in itertools.combinations(link.items, 2):
            if not ia.shape.BoundBox.intersect(ib.shape.BoundBox):
                continue
            try:
                vol = ia.shape.common(ib.shape).Volume
            except Exception:
                vol = -1.0
            if vol > TOL or vol < 0:
                hits.append((ln, vol, ia.name, ib.name, SAME_LINK_ALLOW.get(frozenset((ia.part, ib.part)))))
    return hits


def run(n_random=40):
    model = openduck.OpenDuck()
    lines = ["# OpenDuck carpisma raporu", "", "## 1) Tasarim kontrolu (tek eklem taramasi)"]
    design_hits = 0
    for name, pose in design_poses(model):
        hits = check_pose(model, pose)
        design_hits += len(hits)
        for vol, a, b in sorted(hits, reverse=True):
            lines.append("  %-26s %8.1f mm3  %s <-> %s" % (name, vol, a, b))
    lines.append("  -> %d poz, %d carpisma" % (len(design_poses(model)), design_hits))

    lines += ["", "## 2) Rastgele birlesik pozlar (bilgi amacli)"]
    free, pairs = 0, collections.Counter()
    poses = random_poses(model, n_random)
    for name, pose in poses:
        hits = check_pose(model, pose)
        free += not hits
        for vol, a, b in hits:
            pairs[(a, b)] += 1
    lines.append("  carpismasiz: %d / %d" % (free, len(poses)))
    for (a, b), c in pairs.most_common():
        lines.append("  %3dx  %s <-> %s" % (c, a, b))

    lines += ["", "## 3) Ayni link ici cakismalar (HOME pozu, > %.0f mm3)" % TOL]
    same = same_link_overlaps(model)
    bad = [h for h in same if h[4] is None]
    for ln, vol, a, b, why in sorted(same, key=lambda h: (h[4] is not None, h[0], -h[1])):
        lines.append("  %-18s %8.1f mm3  %s <-> %s%s" % (ln, vol, a, b, "   [izinli: %s]" % why if why else ""))
    lines.append("  -> %d cakisma (%d izinli)" % (len(bad), len(same) - len(bad)))
    return design_hits, free, len(poses), len(bad), "\n".join(lines)


if __name__ == "__main__":
    out = os.environ.get("OPENDUCK_CHECK_OUT", os.path.join(
        os.environ.get("OPENDUCK_OUT", os.path.join(HERE, "..", "build")), "interference.txt"))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    n_design, n_free, n_rand, n_same, txt = run(int(os.environ.get("OPENDUCK_RANDOM_POSES", "40")))
    with open(out, "w") as f:
        f.write(txt + "\n")
    sys.stdout.write(txt + "\n")
