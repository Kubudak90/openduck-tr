# -*- coding: utf-8 -*-
"""
Baski yonu inceleme araci. Bir baski parcasinin otomatik yonunu, secilen yonu (docs.PRINT_DOWN) ve
istenen alternatifleri docs.score ile (build'in kullandigi ayni puanlama) olcer.

  ORIENT_PART=head_hood [ORIENT_DOWN="1,0,0;0,0,-1"] \
    freecadcmd -c "import runpy; runpy.run_path('tools/orient_tool.py', run_name='__main__')"

ORIENT_PART: model parcasi (or. neck_splice -> tum ayrik parcalari) ya da baski parcasi (neck_splice_1);
"all" tum baski parcalarini tek satirla ozetler.
ORIENT_DOWN: tablaya bakacak yuz normali (parcanin montaj/link cercevesinde, build/stl/ ile ayni),
';' ile birden fazla. Cikti: temas alani, 45 dereceden dik sarkma alani (toplam / destek gerektiren /
destek gerektirmeyen: tablaya yakin + kisa kopru), yukseklik, destek bayragi.
"""
import os
import sys

import FreeCAD as App

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "cad"))
import openduck  # noqa: E402
import docs  # noqa: E402

V = App.Vector


def fmt(label, r, inf):
    d = docs.down_of(r)
    return ("%s asagi=(%.3f, %.3f, %.3f): temas %.0f mm2, sarkma %.0f mm2 (destekli %.0f, kucuk %.0f), "
            "yukseklik %.1f mm, destek %s" % (label, d.x, d.y, d.z, inf["contact_mm2"], inf["overhang_mm2"],
                                              inf["support_mm2"], inf["minor_mm2"], inf["height_mm"],
                                              "evet" if inf["support"] else "hayir"))


def main():
    name = os.environ.get("ORIENT_PART", "")
    model = openduck.OpenDuck()
    pieces = docs.print_pieces(model)
    sel = [k for k, pc in pieces.items() if name in (k, pc["part"], "all")]
    if not sel:
        print("parca yok. secenekler: all, %s" % ", ".join(pieces))
        return 1
    for pname in sel:
        pc = pieces[pname]
        shapes = [s for _, _, s in pc["items"]]
        down, why = docs._down_entry(pname, pc["part"])
        rots, info = docs.group_rotations(shapes, down)
        if name == "all":
            print(fmt("%-22s %s" % (pname, "ELLE " if info["manual"] else "OTO  "), rots[0], info))
            continue
        s = shapes[0]
        md = docs.mesh_data(s)
        bb = s.optimalBoundingBox(False, False)
        print("parca %s (%s, %d adet, ayrik parca %d/%d), link cercevesinde zarf %.1f x %.1f x %.1f mm, hacim %.0f mm3"
              % (pname, pc["kind"], len(shapes), pc["idx"] + 1, pc["n"], bb.XLength, bb.YLength, bb.ZLength,
                 s.Volume))
        print("ogeler: %s" % ", ".join(it.name for _, it, _ in pc["items"]))
        ra, ia = docs.best_print_rotation(s, md)
        print(fmt("OTOMATIK", ra, ia))
        print(fmt("SECILEN (%s)" % ("elle: " + why if info["manual"] else "otomatik"), rots[0], docs.score(md, rots[0])))
        planar = sorted([f for f in s.Faces if f.Surface.__class__.__name__ == "Plane"], key=lambda f: -f.Area)
        print("en buyuk duz yuzler (normal, alan):")
        for f in planar[:6]:
            n = f.normalAt(0, 0)
            print("   (%.3f, %.3f, %.3f)  %.0f mm2" % (n.x, n.y, n.z, f.Area))
        for txt in filter(None, os.environ.get("ORIENT_DOWN", "").split(";")):
            n = V(*[float(x) for x in txt.split(",")]).normalize()
            r = App.Rotation(n, V(0, 0, -1))
            print(fmt("ONERI", r, docs.score(md, r)))
    return 0


if __name__ == "__main__":
    main()
