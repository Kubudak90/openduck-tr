# -*- coding: utf-8 -*-
"""
OpenDuck dokumanlari: baskiya yonlendirilmis STL seti + baski rehberi + zip, patlatilmis montaj
gorselleri, parca katalogu, olculu teknik resim sayfalari ve hepsini toplayan PDF kitapcik.
tools/build_gui.py tarafindan cagrilir (gorseller icin FreeCAD GUI gerekir).
"""
import datetime
import math
import os
import zipfile
from collections import OrderedDict

import FreeCAD as App
import MeshPart

import openduck
from openduck import KINDS, PRINT_KINDS, V

FONT_DIRS = ["/usr/share/fonts/truetype/dejavu", "/usr/share/fonts/TTF", "/Library/Fonts", "C:/Windows/Fonts"]

# Baski ayarlari onerisi (tur -> metin)
PRINT_SETTINGS = {
    "shell":   ("PLA", "0.2 mm katman, 3 duvar, %15 dolgu"),
    "trim":    ("PLA", "0.2 mm katman, 3 duvar, %20 dolgu"),
    "face":    ("PLA", "0.2 mm katman, 3 duvar, %20 dolgu"),
    "bracket": ("PETG (veya PLA+)", "0.2 mm katman, 4 duvar, %40 dolgu"),
    "sole":    ("TPU 95A", "0.2 mm katman, 3 duvar, %30 dolgu, yavas"),
    "pad":     ("TPU 85-95A", "0.2 mm katman, %100 dolgu, yavas"),
}
TURKISH = {   # parca -> aciklama; eksik parca adlari describe() ile addan turetilir
    "chassis": "sasi (taban + servo kafesi + boyun catali)", "battery_tray": "batarya yuvasi",
    "torso_shell_left": "govde kabugu sol yarim", "torso_shell_right": "govde kabugu sag yarim",
    "hip_bracket": "kalca yaw braketi", "hip_cradle_bracket": "kalca roll-pitch braketi",
    "thigh_inner_plate": "uyluk ic plakasi", "thigh_wing": "uyluk kanat kabugu",
    "shin_bracket": "incik braketi", "shin_pad": "diz/incik pedi", "ankle_bracket": "ayak bilegi U braketi",
    "foot_body": "ayak govdesi", "sole": "taban (TPU)", "neck_splice": "boyun ek plakalari",
    "head_tilt_bracket": "kafa egim braketi", "head_turn_bracket": "kafa donus braketi",
    "head_hood": "kafa ust kaput", "head_bottom_shell": "kafa alt kabugu", "upper_bill": "ust gaga",
    "upper_bill_pad": "ust gaga pedi (TPU)", "face_plate": "yuz plakasi", "eye_ring": "goz halkasi",
    "head_frame": "kafa ic cercevesi", "lower_bill": "alt gaga (cene)", "lower_bill_pad": "alt gaga pedi (TPU)",
}


def _font(size, bold=False):
    from PIL import ImageFont
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    for d in FONT_DIRS:
        p = os.path.join(d, name)
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def _local(link, item):
    s = item.shape.copy()
    s.translate(-link.origin)
    return s


def describe(part):
    """Parcanin Turkce aciklamasi; sozlukte yoksa adindan turetilir."""
    return TURKISH.get(part) or part.replace("_", " ")


def material(kind):
    """Baski malzemesi (BOM, PRINT.md, katalog ve sayfalar hep buradan okur)."""
    return PRINT_SETTINGS[kind][0] if kind in PRINT_SETTINGS else KINDS[kind]["print"].split(" (")[0]


def color_name(kind):
    return KINDS[kind]["print"].split("(")[-1].rstrip(")")


def print_parts(model):
    """Benzersiz baski parcalari: ad -> {"items": [(link, item)], "kind"}. Sol/sag ayni parca."""
    parts = OrderedDict()
    for link in model.links.values():
        for it in link.items:
            if it.kind in PRINT_KINDS:
                g = parts.setdefault(it.part, {"items": [], "kind": it.kind})
                g["items"].append((link, it))
    return parts


def _solid_key(s):
    c = s.CenterOfMass
    return (round(c.x, 1), round(c.z, 1), -round(c.y, 1), -round(s.Volume))


def _match_solids(base, shape, mirror):
    """shape'in katilarini base (ilk oge) sirasina dizer; ayna esinde y isareti cevrilerek eslenir."""
    pool = list(shape.Solids) or [shape]
    if len(pool) != len(base):
        raise ValueError("ayna/kopya ogelerin kati sayisi farkli (%d != %d)" % (len(pool), len(base)))
    out = []
    for b in base:
        c = b.CenterOfMass
        t = V(c.x, -c.y if mirror else c.y, c.z)
        out.append(pool.pop(min(range(len(pool)), key=lambda i: (pool[i].CenterOfMass - t).Length)))
    return out


def print_pieces(model):
    """Ayri ayri basilan parcalar. Bir model parcasi birden fazla ayrik katidan olusuyorsa (or. iki plaka)
    her kati ayri baski parcasidir: ad "<parca>_1", "<parca>_2"...; tek katiysa ad parca adidir.
    Doner: ad -> {"part", "kind", "idx", "n", "items": [(link, item, kati (link cercevesinde))]}."""
    pieces = OrderedDict()
    for part, g in print_parts(model).items():
        locs = [_local(l, it) for l, it in g["items"]]
        base = sorted(list(locs[0].Solids) or [locs[0]], key=_solid_key)
        per_item = [base] + [_match_solids(base, s, _is_mirror(locs[0], s)) for s in locs[1:]]
        n = len(base)
        for k in range(n):
            pieces[part if n == 1 else "%s_%d" % (part, k + 1)] = {
                "part": part, "kind": g["kind"], "idx": k, "n": n,
                "items": [(l, it, per_item[j][k]) for j, (l, it) in enumerate(g["items"])]}
    return pieces


def piece_file(item, pc):
    return (item.name if pc["n"] == 1 else "%s_%d" % (item.name, pc["idx"] + 1)) + ".stl"


# ---------------------------------------------------------------------------
# Baski yonu
# ---------------------------------------------------------------------------
# Elle secilen baski yonleri (dayanim / gorunus). Anahtar: parca adi ya da ayrik parcalarda "<parca>_N".
# Deger: (tablaya bakan yuz normali, link cercevesinde - sol/ilk oge; ayna esi aynalanmis yonu alir, gerekce).
# Her deger tools/orient_tool.py ile olculdu (temas / sarkma / destek); otomatik secime gore kazanci yazili.
PRINT_DOWN = {
    "hip_cradle_bracket": ((0, 0, -1), "L profili katman duzleminde, kollar bukumden ayrilmaz; 5 mm kenar (brim)"),
    "hip_bracket": ((-1, 0, 0), "servo arka plakasi katman duzleminde, desteksiz"),
    "torso_shell_left": ((0, 1, 0), "dis yan yuz tablada: desteksiz, alcak, gorunen yuz tabla yuzeyi"),
    "torso_shell_right": ((0, -1, 0), "dis yan yuz tablada: desteksiz, alcak, gorunen yuz tabla yuzeyi"),
    "head_tilt_bracket": ((-1, 0, 0), "U profili katman duzleminde (kafa egim torku katman arasina binmez)"),
    "head_turn_bracket": ((-1, 0, 0), "-X yuzu tablada, 10 mm pim yukari bakar ve desteksiz yuvarlak basilir"),
    "neck_splice": ((1, 0, 0), "duz plaka, tablada"),
    "head_hood": ((0, 0, -1), "acik alt tablada: dis yuzeyde destek yok, 47 mm (sirtustu 109 mm); ic tavana "
                              "tabladan agac (tree) destek"),
}

NEAR_BED_MM = 3.0     # en ust noktasi tablaya bu kadar yakin sarkmalar destek istemez (r<=10 tabla teget
                      # yuvarlatmanin 45 derece noktasi 0.29 r = 2.9 mm yukseklikte)
BRIDGE_MM = 10.0      # bu boydan kucuk sarkma adalari (yatay delik tepeleri, kisa kopruler) destek istemez
COS45 = math.cos(math.radians(45))


def mesh_data(shape, lin=0.4, ang=0.5):
    """Puanlama icin kaba ucgen ag (numpy): koseler, ucgenler, normaller, alanlar, kaynakli kose kimlikleri."""
    import numpy as np
    mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=lin, AngularDeflection=ang, Relative=False)
    pts, tris = mesh.Topology
    P = np.array([(p.x, p.y, p.z) for p in pts], float)
    T = np.array(tris, int).reshape(-1, 3)
    cr = np.cross(P[T[:, 1]] - P[T[:, 0]], P[T[:, 2]] - P[T[:, 0]])
    nl = np.linalg.norm(cr, axis=1)
    ok = nl > 1e-12
    T, cr, nl = T[ok], cr[ok], nl[ok]
    weld = np.unique(np.round(P / 0.01).astype(np.int64), axis=0, return_inverse=True)[1].reshape(-1)
    return {"P": P, "T": T, "N": cr / nl[:, None], "A": 0.5 * nl, "W": weld[T]}


def _islands(ids, W):
    """Kose paylasan ucgen adalari (union-find); ids: ucgen indeksleri."""
    parent = {int(i): int(i) for i in ids}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    owner = {}
    for i in parent:
        for v in W[i]:
            j = owner.setdefault(int(v), i)
            ri, rj = find(i), find(j)
            if ri != rj:
                parent[ri] = rj
    groups = {}
    for i in parent:
        groups.setdefault(find(i), []).append(i)
    return list(groups.values())


def score(md, rot):
    """Yonelim puani. contact: tablaya oturan alt yuz alani; overhang: 45 dereceden dik tum alt yuzeyler;
    support_mm2: bunlardan destek gerektirenler (tablaya NEAR_BED_MM'den yakin olanlar ve BRIDGE_MM'den
    kucuk adalar - yatay delik tepeleri, kisa kopruler - haric); minor_mm2: haric tutulan kisim."""
    import numpy as np
    m = rot.toMatrix()
    R = np.array([[m.A11, m.A12, m.A13], [m.A21, m.A22, m.A23], [m.A31, m.A32, m.A33]])
    P = md["P"].dot(R.T)
    nz = md["N"].dot(R[2])
    z = P[:, 2]
    zmin = z.min()
    ztop = z[md["T"]].max(axis=1) - zmin
    A = md["A"]
    low = ztop < 0.05
    contact = float(A[(nz < -0.999) & low].sum())
    ov = (nz < -COS45 - 0.01) & ~low          # tam 45 derece (pah) basilabilir; kaba agdaki sinir ucgenleri disarida
    over = float(A[ov].sum())
    sup = 0.0
    for isl in _islands(np.nonzero(ov & (ztop >= NEAR_BED_MM))[0], md["W"]):
        vs = P[md["T"][isl].reshape(-1)]
        if max(np.ptp(vs[:, 0]), np.ptp(vs[:, 1])) >= BRIDGE_MM:
            sup += float(A[isl].sum())
    inf = {"contact_mm2": contact, "overhang_mm2": over, "support_mm2": sup, "minor_mm2": over - sup,
           "height_mm": float(z.max() - zmin), "support": sup > 0.05 * max(contact, 1.0) + 20.0}
    inf["key"] = (round(contact - 2.0 * sup - 0.5 * (over - sup), 1), -round(inf["height_mm"], 1))
    return inf


def down_of(rot):
    """Tablaya bakan yon (link cercevesinde): R^-1 * (0,0,-1)."""
    return rot.inverted().multVec(V(0, 0, -1))


def best_print_rotation(shape, md=None):
    """Otomatik yon: tablaya temas alani buyuk, destek gerektiren sarkmasi kucuk, esitlikte alcak yonelim.
    Doner: (App.Rotation, bilgi sozlugu)."""
    md = md or mesh_data(shape)
    cands = [App.Rotation()]
    for d in (V(1, 0, 0), V(-1, 0, 0), V(0, 1, 0), V(0, -1, 0), V(0, 0, 1), V(0, 0, -1)):
        cands.append(App.Rotation(d, V(0, 0, -1)))
    planar = sorted([f for f in shape.Faces if f.Surface.__class__.__name__ == "Plane"], key=lambda f: -f.Area)
    for f in planar[:12]:
        n = f.normalAt(0, 0)          # yuz yonu dahil; aynalanmis parcalarda bayraklar ters olabilir -> iki yon de aday
        cands += [App.Rotation(n, V(0, 0, -1)), App.Rotation(n * -1, V(0, 0, -1))]
    best, best_info = None, None
    for r in cands:
        inf = score(md, r)
        if best_info is None or inf["key"] > best_info["key"]:
            best, best_info = r, inf
    return best, best_info


def _mirror_rot(r):
    """Y aynasindaki parca icin esdeger yonelim: M.R.M (M = diag(1,-1,1))."""
    M = App.Matrix()
    M.A22 = -1.0
    return App.Rotation(M * r.toMatrix() * M)


def _is_mirror(a, b, tol=0.05):
    ba, bb = a.optimalBoundingBox(False, False), b.optimalBoundingBox(False, False)
    return (abs(a.Volume - b.Volume) < 1e-3 * a.Volume + 1e-6 and abs(ba.YMin + bb.YMax) < tol
            and abs(ba.YMax + bb.YMin) < tol and abs(ba.XMin - bb.XMin) < tol and abs(ba.ZMin - bb.ZMin) < tol)


def group_rotations(shapes, down=None):
    """Bir baski parcasi grubunun (sol/sag ayna cifti ya da tekil) yonelimleri. Ayna ciftleri ayni sekilde
    (aynali) basilir. down verilirse (link cercevesinde tablaya bakan normal) ilk ogeye o uygulanir;
    yoksa ikisinden hangisi daha iyi yonelim bulursa o kullanilir. Doner: (rotasyonlar, bilgi)."""
    mirror = len(shapes) == 2 and _is_mirror(shapes[0], shapes[1])
    if down is not None:
        r = App.Rotation(V(*down).normalize(), V(0, 0, -1))
        inf = score(mesh_data(shapes[0]), r)
        rots = [r, _mirror_rot(r)] if mirror else [r] * len(shapes)
    elif mirror:
        res = [best_print_rotation(s) for s in shapes]
        k = 0 if res[0][1]["key"] >= res[1][1]["key"] else 1
        r, inf = res[k]
        rots = [r, _mirror_rot(r)] if k == 0 else [_mirror_rot(r), r]
    else:
        r, inf = best_print_rotation(shapes[0])
        rots = [r] * len(shapes)
    inf["mirror"] = mirror
    inf["manual"] = down is not None
    return rots, inf


def oriented(shape, rot):
    """Yonelimi uygula, tablaya (z=0) oturt, xy'de ortala. Zarf kesin geometriden (ucgenlemeden degil)."""
    s = shape.copy()
    s.Placement = App.Placement(V(0, 0, 0), rot).multiply(s.Placement)
    bb = s.optimalBoundingBox(False, False)
    s.translate(V(-bb.Center.x, -bb.Center.y, -bb.ZMin))
    return s


def _down_entry(pname, part):
    e = PRINT_DOWN.get(pname, PRINT_DOWN.get(part))
    return e if e else (None, "")


def export_print_set(model, out_dir, zip_path, summ):
    """Yonlendirilmis STL'ler (build/print/stl), PRINT.md rehberi ve zip. Her ayrik kati ayri parca/STL."""
    stl_dir = os.path.join(out_dir, "stl")
    os.makedirs(stl_dir, exist_ok=True)
    for old in os.listdir(stl_dir):               # eski derlemelerden kalan (yeniden adlanmis) dosyalar
        if old.endswith(".stl"):
            os.remove(os.path.join(stl_dir, old))
    pieces = print_pieces(model)
    rows, files, info = [], [], OrderedDict()
    for pname, pc in pieces.items():
        shapes = [s for _, _, s in pc["items"]]
        down, why = _down_entry(pname, pc["part"])
        rots, inf = group_rotations(shapes, down)
        fns = []
        for (link, it, s), r in zip(pc["items"], rots):
            mesh = MeshPart.meshFromShape(Shape=oriented(s, r), LinearDeflection=0.04, AngularDeflection=0.15,
                                          Relative=False)
            mesh.translate(0, 0, -mesh.BoundBox.ZMin)
            fns.append(piece_file(it, pc))
            mesh.write(os.path.join(stl_dir, fns[-1]))
        files += fns
        link0, it0, s0l = pc["items"][0]
        s0 = oriented(s0l, rots[0])
        bb = s0.optimalBoundingBox(False, False)      # kesin zarf: GUI ucgenlemesinden once olculur
        kind = pc["kind"]
        info[pname] = {"rot": rots[0], "shape": s0, "kind": kind, "part": pc["part"], "piece": (pc["idx"], pc["n"]),
                       "qty": len(pc["items"]), "mirror": inf["mirror"], "support": inf["support"],
                       "mass_g": it0.mass_kg(model.p) * 1000.0 * s0l.Volume / it0.shape.Volume,
                       "size": (bb.XLength, bb.YLength, bb.ZLength), "center": bb.Center,
                       "down": down_of(rots[0]), "why": why if inf["manual"] else "", "score": inf, "files": fns}
        desc = describe(pc["part"]) + (" - ayri parca %d/%d" % (pc["idx"] + 1, pc["n"]) if pc["n"] > 1 else "")
        rows.append("| %s | %s | %d | %s | %s | %.1f x %.1f x %.1f | %.0f | %s | %s | %s |" % (
            pname, desc, len(pc["items"]), material(kind), color_name(kind), bb.XLength, bb.YLength, bb.ZLength,
            info[pname]["mass_g"], "evet" if inf["support"] else "hayir", why if inf["manual"] else "otomatik",
            ", ".join("`%s`" % f for f in fns)))
    total_g = sum(v["mass_g"] * v["qty"] for v in info.values())
    n_parts = len(set(v["part"] for v in info.values()))
    n_split = sorted(set(v["part"] for v in info.values() if v["piece"][1] > 1))
    biggest = max(info, key=lambda k: max(info[k]["size"]))
    bed = 180.0
    L = ["# OpenDuck - 3B baski rehberi", "",
         "Bu klasordeki her STL **tek bir kati parcadir**, baski yonune cevrilmis, tablada ortalanmis ve z=0'a",
         "oturtulmustur. Olcu birimi mm. **Dilimleyicide dondurmeyin**; dosyayi oldugu gibi tablaya yerlestirin.",
         "", "Yon nasil secildi:",
         "- Cogu parcada otomatik: tablaya oturan duz alan buyuk, destek gerektiren sarkma (45 dereceden dik alt",
         "  yuzey) kucuk, esitlikte daha alcak olan yon. Tablaya %.0f mm'den yakin yuvarlatmalar ve %.0f mm'den kisa"
         % (NEAR_BED_MM, BRIDGE_MM), "  kopruler/yatay delik tepeleri destek sayilmaz.",
         "- Yuk tasiyan braketlerde ve gorunen kabuklarda yon dayanim ya da gorunus icin elle secildi (tabloda",
         "  `Yon` sutunu): katman cizgileri yuk altindaki bukumlere paralel kalir, gorunen yuz tabla yuzeyi alir.",
         "  Bu yuzden her parcada en genis duz yuz tablaya bakmaz.",
         "- Sol/sag parcalar ayna simetriktir ve ayna yonde basilir; ikisini de basin.", ""]
    if n_split:
        L += ["Birden fazla ayrik katidan olusan model parcalari (%s) ayri STL'lere bolundu" % ", ".join(n_split),
              "(`_1`, `_2`...); her biri kendi yonunde tablaya oturur.", ""]
    L += ["| | |", "|---|---|",
          "| Model parcasi cesidi | %d |" % n_parts,
          "| Baski parcasi cesidi | %d |" % len(info),
          "| Basilacak toplam parca (= STL dosyasi) | %d |" % sum(v["qty"] for v in info.values()),
          "| Tahmini toplam filament | %.0f g |" % total_g,
          "| En buyuk parca | %s (%.0f x %.0f x %.0f mm) |" % ((biggest,) + info[biggest]["size"]), ""]
    too_big = [k for k, v in info.items() if max(v["size"]) > bed]
    L += ["Tum parcalar %.0f x %.0f x %.0f mm'lik bir tablaya sigar." % (bed, bed, bed) if not too_big else
          "%.0f mm tablaya sigmayanlar: %s" % (bed, ", ".join(too_big)), "",
          "## Ayar onerileri", "",
          "| Tur | Malzeme | Renk | Ayar |", "|---|---|---|---|"]
    for kind, (mat, setting) in PRINT_SETTINGS.items():
        L.append("| %s | %s | %s | %s |" % (kind, mat, color_name(kind), setting))
    L += ["", "- Braketler servo yuklerini tasir: PETG ya da PLA+ ve 4 duvar onerilir.",
          "- Vida delikleri sadelestirilmistir. Basili braketin servoya ya da horn'a sikildigi yerlerde M2 icin",
          "  ~2.2-2.4 mm bosluk deligi olmali (vida servonun/horn'un disine tutunur, braket yalnizca sikisir);",
          "  1.6 mm pilot delik yalnizca vidanin plastige tutundugu yerlerde; isil ek somun yalnizca modelde",
          "  yuvasi olan yerlerde kullanilir.",
          "- `Destek: evet` olan parcalarda yalnizca tabla uzerinden destek (support on build plate) yeterlidir.",
          "- Tabladan kalkma riskine karsi `Yon` sutununda kenar (brim) yazan parcalarda 5 mm brim kullanin.", "",
          "## Parca listesi", "",
          "| Parca | Aciklama | Adet | Malzeme | Renk | Boyut (mm, baski yonunde) | Kutle (g, adet basi) | Destek | "
          "Yon | STL |",
          "|---|---|---:|---|---|---|---:|---|---|---|"] + rows
    L += ["", "Robot: %s, %d servo (%s), toplam ~%.0f g." % (summ["robot"], summ["servo_count"], summ["servo_model"],
                                                            summ["mass_kg"] * 1000), ""]
    md = os.path.join(out_dir, "PRINT.md")
    with open(md, "w") as f:
        f.write("\n".join(L))
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
        z.write(md, "openduck_print/PRINT.md")
        for fn in files:
            z.write(os.path.join(stl_dir, fn), "openduck_print/stl/" + fn)
    return info


# ---------------------------------------------------------------------------
# Patlatilmis gorunum
# ---------------------------------------------------------------------------
EXPLODE_EXTRA = {   # parca adina gore ek kaydirma (mm, dunya), sol taraf icin; sagda y aynalanir
    "torso_shell_left": (0, 38, 0), "torso_shell_right": (0, -38, 0), "battery_np_f550": (-50, 0, -15),
    "power_board": (-15, 0, 40), "imu_board": (0, 0, 22), "imu_chip": (0, 0, 22),
    "head_hood": (0, 0, 60), "head_bottom_shell": (0, 0, -10), "face_plate": (28, 0, 0), "eye_ring": (55, 0, 0),
    "camera_lens": (42, 0, 0), "camera_glass": (42, 0, 0), "camera_holder": (34, 0, 0), "tof_sensor": (34, 0, 0),
    "rec_led": (34, 0, 0), "main_board": (18, 0, 28), "robot_hat": (6, 0, 28), "speaker": (-25, 0, 30),
    "upper_bill": (32, 0, -4), "upper_bill_pad": (32, 0, -10), "lower_bill": (30, 0, -26),
    "lower_bill_pad": (30, 0, -18), "mouth_horn": (0, 18, 0),
    "thigh_wing": (0, 28, 0), "thigh_inner_plate": (0, -8, 0), "shin_pad": (16, 0, 0),
    "sole": (0, 0, -14), "neck_splice": (0, 0, 0),
}
EXPLODE_JOINT = 1.9   # eklem ofsetleri bu katsayiyla buyutulur
# Kafa alt montaji gorunusu icin: yuz yigini (yuz plakasi -> kamera -> goz halkasi) goz ekseni boyunca ve
# gaga parcalari (ust gaga, ust ped, alt ped, alt gaga) ileri-asagi merdiven seklinde acilir; bakis dogrultusunda
# ust uste binmezler (izdusum kutulari ayrik).
EXPLODE_EXTRA_HEAD = {
    "face_plate": (60, 0, 8), "camera_holder": (125, 0, 8), "tof_sensor": (125, 0, 8), "rec_led": (125, 0, 8),
    "camera_lens": (133, 0, 8), "camera_glass": (133, 0, 8), "eye_ring": (180, 0, 8),
    "upper_bill": (99, 0, -5), "upper_bill_pad": (104, 0, -30), "lower_bill_pad": (54, 0, -93),
    "lower_bill": (97, 0, -151),
}


def exploded_shapes(model, pose, links=None, scale=EXPLODE_JOINT, radial=0.0, extra=None):
    """Patlatilmis parcalar: eklem ofsetleri `scale` ile buyutulur, link icindeki parcalar link merkezinden
    `radial` oraninda disari itilir, EXPLODE_EXTRA (+ `extra`) ek kaydirmalari uygulanir."""
    extra = dict(EXPLODE_EXTRA, **(extra or {}))
    pls = model.link_placements(pose)
    # hiyerarsik: cocuk link, ebeveyne gore ofsetinin scale katina tasinir
    shift = {"trunk": V(0, 0, 0)}
    for j in model.joints:
        p, c = j["parent"], j["child"]
        d = pls[c].Base - pls[p].Base
        shift[c] = shift[p] + d * (scale - 1.0)
    out = []
    for ln, link in model.links.items():
        if links is not None and ln not in links:
            continue
        posed = []
        for it in link.items:
            s = it.shape.copy()
            s.translate(-link.origin)
            s.Placement = pls[ln].multiply(s.Placement)
            posed.append((it, s))
        bb = None
        for _, s in posed:
            if bb is None:
                bb = s.BoundBox
            else:
                bb.add(s.BoundBox)
        for it, s in posed:
            if radial > 0 and len(posed) > 1:
                s.translate((s.BoundBox.Center - bb.Center) * radial)
            e = extra.get(it.part)
            if e is not None:
                sy = -1.0 if it.name.startswith("right_") else 1.0
                s.translate(V(e[0], sy * e[1], e[2]))
            s.translate(shift[ln])
            out.append((it, s))
    return out


# ---------------------------------------------------------------------------
# GUI yardimcilari
# ---------------------------------------------------------------------------
def _new_doc(name):
    import FreeCADGui as Gui
    # kamera gecis animasyonlari, ardisik kamera ayarlarini ezer
    App.ParamGet("User parameter:BaseApp/Preferences/View").SetBool("UseNavigationAnimations", False)
    doc = App.newDocument(name)
    Gui.activateView("Gui::View3DInventor", True)
    return doc, Gui.getDocument(doc.Name).ActiveView


def _add(doc, name, shape, color, mode="Shaded"):
    f = doc.addObject("Part::Feature", name)
    f.Shape = shape
    openduck.set_color(f.ViewObject, color)
    f.ViewObject.DisplayMode = mode
    return f


def _shot(view, path, direction, size, ss=2, bg=(255, 255, 255)):
    import FreeCADGui as Gui
    from PIL import Image
    from export import _camera_rotation
    view.setCameraType("Orthographic")
    view.setCameraOrientation(_camera_rotation(direction).Q)
    view.fitAll()
    Gui.updateGui()
    raw = (path or "/tmp/_openduck_shot") + ".raw.png"
    view.saveImage(raw, size[0] * ss, size[1] * ss, "Transparent")
    img = Image.open(raw).convert("RGBA").resize(size, Image.LANCZOS)
    out = Image.new("RGBA", size, bg + (255,))
    out.alpha_composite(img)
    out = out.convert("RGB")
    os.remove(raw)
    if path:
        out.save(path, optimize=True)
    return out


def _ortho_shot(view, center, look, up_hint, height_mm, size, ss=2, bg=(255, 255, 255)):
    """Kesin olcekli ortografik goruntu: goruntu yuksekligi height_mm'e karsilik gelir."""
    import FreeCADGui as Gui
    from PIL import Image
    look = V(*look).normalize()
    zc = look * -1
    yc = V(*up_hint)
    xc = yc.cross(zc).normalize()
    yc = zc.cross(xc)
    m = App.Matrix(xc.x, yc.x, zc.x, 0, xc.y, yc.y, zc.y, 0, xc.z, yc.z, zc.z, 0, 0, 0, 0, 1)
    r = App.Rotation(m)
    pos = center - look * 1000.0
    # ADJUST_CAMERA: dar (en < boy) goruntude gorunen yukseklik height/aspect olur; telafi et
    cam_h = height_mm * min(1.0, float(size[0]) / size[1])
    iv = ("#Inventor V2.1 ascii\nOrthographicCamera {\n viewportMapping ADJUST_CAMERA\n position %f %f %f\n"
          " orientation %f %f %f %f\n nearDistance 1\n farDistance 3000\n aspectRatio 1\n focalDistance 1000\n"
          " height %f\n}\n" % (pos.x, pos.y, pos.z, r.Axis.x, r.Axis.y, r.Axis.z, r.Angle, cam_h))
    view.setCamera(iv)
    Gui.updateGui()
    view.setCamera(iv)
    raw = "/tmp/_openduck_ortho.png"
    view.saveImage(raw, size[0] * ss, size[1] * ss, "Transparent")
    img = Image.open(raw).convert("RGBA").resize(size, Image.LANCZOS)
    out = Image.new("RGBA", size, tuple(bg) + (255,))
    out.alpha_composite(img)
    os.remove(raw)
    return out.convert("RGB"), (xc, yc)


def _fit_shot(view, shapes, path, direction, size, bg, margin=0.04):
    """Sahneye siki oturan ortografik goruntu. fitAll sinir kuresine gore oturtur ve patlatilmis (dagitilmis)
    sahnelerde genis bos alan birakir; burada izdusum kutusu kullanilir."""
    look = V(*direction).normalize()
    zc = look * -1
    xc = V(0, 0, 1).cross(zc).normalize()
    yc = zc.cross(xc)
    pts = [p for s in shapes for p in s.tessellate(2.0)[0]]
    us, vs, ws = [p.dot(xc) for p in pts], [p.dot(yc) for p in pts], [p.dot(zc) for p in pts]
    c = (xc * ((min(us) + max(us)) / 2) + yc * ((min(vs) + max(vs)) / 2) + zc * ((min(ws) + max(ws)) / 2))
    h = max(max(vs) - min(vs), (max(us) - min(us)) * size[1] / float(size[0])) * (1 + 2 * margin)
    img, _ = _ortho_shot(view, c, look, (0, 0, 1), h, size, bg=bg)
    img.save(path, optimize=True)
    return img


# ---------------------------------------------------------------------------
# Patlatilmis gorseller
# ---------------------------------------------------------------------------
def render_exploded(model, out_dir):
    from PIL import Image, ImageDraw
    os.makedirs(out_dir, exist_ok=True)
    stand = model.pose("stand")
    sets = [
        ("exploded", None, [("iso", (-1.0, 0.85, -0.40)), ("side", (-0.05, 1.0, -0.12))], (1600, 1400), None),
        ("exploded_head", [l for l in model.links if l in ("neck", "head_tilt", "head_turn", "head", "jaw")],
         [("iso", (-1.0, 0.9, -0.45))], (1400, 1100), EXPLODE_EXTRA_HEAD),
        ("exploded_leg", [l for l in model.links if l.startswith("left_")],
         [("iso", (-0.9, 1.0, -0.35))], (1100, 1300), None),
    ]
    files = []
    for name, links, views, size, extra in sets:
        doc, view = _new_doc("Exp_" + name)
        scale, radial = (EXPLODE_JOINT, 0.2) if links is None else (EXPLODE_JOINT + 0.7, 0.6)
        shapes = []
        for k, (it, s) in enumerate(exploded_shapes(model, stand, links, scale, radial, extra)):
            _add(doc, "p%03d" % k, s, KINDS[it.kind]["color"])
            shapes.append(s)
        doc.recompute()
        for vn, d in views:
            fn = os.path.join(out_dir, "%s_%s.png" % (name, vn))
            _fit_shot(view, shapes, fn, d, size, (244, 245, 247))
            files.append(fn)
        App.closeDocument(doc.Name)
    return files


# ---------------------------------------------------------------------------
# Parca katalogu ve teknik resim sayfalari
# ---------------------------------------------------------------------------
A4 = (1754, 1240)        # yatay A4, 150 dpi
PX_PER_MM_1TO1 = 150 / 25.4
STD_SCALES = [5.0, 2.0, 1.0, 0.5, 0.2]          # ISO 5455


def _scale_label(s):
    return ("%g:1" % s) if s >= 1 else ("1:%g" % (1 / s))


DIM_COL = (20, 60, 140)
DIM_SHORT_PX = 30         # bundan kisa olculerde oklar disaridan


def _dim_h(draw, x0, x1, y, text, font, ext_from=None):
    """Yatay olcu cizgisi (gorunusun ustunde): oklar, uzatma cizgileri (ext_from: parca kenarinin y'si) ve
    cizginin ustunde ortalanmis metin. Kisa olculerde oklar disaridan iceri bakar."""
    if ext_from is not None:
        for x in (x0, x1):
            draw.line([(x, ext_from - 2), (x, y - 6)], fill=DIM_COL, width=1)
    short = x1 - x0 < DIM_SHORT_PX
    draw.line([(x0 - 22, y), (x1 + 22, y)] if short else [(x0, y), (x1, y)], fill=DIM_COL, width=2)
    for x, d in ((x0, 1), (x1, -1)):
        d = -d if short else d
        draw.polygon([(x, y), (x + 12 * d, y - 5), (x + 12 * d, y + 5)], fill=DIM_COL)
    tw = draw.textlength(text, font=font)
    xm = (x0 + x1) / 2
    draw.rectangle([(xm - tw / 2 - 4, y - 26), (xm + tw / 2 + 4, y - 5)], fill=(255, 255, 255))
    draw.text((xm - tw / 2, y - 26), text, fill=DIM_COL, font=font)


def _dim_v(draw, y0, y1, x, text, font, ext_from=None):
    """Dikey olcu cizgisi (gorunusun solunda): metin cizginin SOLUNDA (gorunusun disinda) durur.
    ext_from: parca kenarinin x'i (uzatma cizgileri). Kisa olculerde oklar disaridan iceri bakar."""
    if ext_from is not None:
        for y in (y0, y1):
            draw.line([(ext_from + 2, y), (x - 6, y)], fill=DIM_COL, width=1)
    short = y1 - y0 < DIM_SHORT_PX
    draw.line([(x, y0 - 22), (x, y1 + 22)] if short else [(x, y0), (x, y1)], fill=DIM_COL, width=2)
    for y, d in ((y0, 1), (y1, -1)):
        d = -d if short else d
        draw.polygon([(x, y), (x - 5, y + 12 * d), (x + 5, y + 12 * d)], fill=DIM_COL)
    tw = draw.textlength(text, font=font)
    ym = (y0 + y1) / 2
    draw.rectangle([(x - 12 - tw, ym - 12), (x - 6, ym + 12)], fill=(255, 255, 255))
    draw.text((x - 9 - tw, ym - 12), text, fill=DIM_COL, font=font)


def _view_px(mm, ppm, margin_mm=2.0):
    """Gorunus goruntusu piksel boyu: parca + iki yanda margin_mm; en az 10 px."""
    return max(int(math.ceil((mm + 2 * margin_mm) * ppm)), 10)


def _wrap(dr, text, font, width):
    words, line, out = str(text).split(" "), "", []
    for w in words:
        t = (line + " " + w).strip()
        if line and dr.textlength(t, font=font) > width:
            out.append(line)
            line = w
        else:
            line = t
    return out + [line]


def _piece_desc(d):
    i, n = d["piece"]
    return describe(d["part"]) + (" (ayri parca %d/%d)" % (i + 1, n) if n > 1 else "")


def render_parts(model, out_dir, info, summ):
    """Her baski parcasi icin: katalog karesi (renkli izometrik) ve olculu A4 teknik resim sayfasi.
    Olculer export_print_set'in kesin zarfindan (d["size"]) gelir; gorunusler tam olcekte cizilir."""
    from PIL import Image, ImageDraw
    os.makedirs(out_dir, exist_ok=True)
    for old in os.listdir(out_dir):                   # eski derlemelerden kalan sayfa/katalog dosyalari
        if old.startswith(("sheet_", "parts_catalog")) and old.endswith(".png"):
            os.remove(os.path.join(out_dir, old))
    doc, view = _new_doc("Parts")
    feats = {}
    for k, (name, d) in enumerate(info.items()):
        f = _add(doc, "x%03d" % k, d["shape"], KINDS[d["kind"]]["color"], "Flat Lines")
        f.ViewObject.LineColor = (0.1, 0.1, 0.1)
        f.ViewObject.LineWidth = 1.5
        f.ViewObject.Visibility = False
        feats[name] = f
    doc.recompute()

    tiles, sheets = OrderedDict(), []
    f_small, f_med, f_title = _font(18), _font(22), _font(40, True)
    today = datetime.date.today().isoformat()
    n_sheets = len(info)
    for k, (name, d) in enumerate(info.items()):
        for f in feats.values():
            f.ViewObject.Visibility = False
        feats[name].ViewObject.Visibility = True
        feats[name].ViewObject.DisplayMode = "Shaded"
        tiles[name] = _shot(view, None, (-1.0, 0.8, -0.6), TILE)
        feats[name].ViewObject.DisplayMode = "Flat Lines"

        # teknik resim: on (+Y'ye bakar), ustten (-Z), soldan (+X'e bakar) gorunus; ISO E (1. aci)
        X, Y, Z = d["size"]                              # kesin zarf (GUI ucgenlemesinden once olculdu)
        c = d["center"]
        area_w, area_h = 1080, 1080                      # cizim alani (px)
        gap = 40 * 2
        need_w, need_h = X + Y, Z + Y                    # mm (on + yan genislik, on + ust yukseklik)
        fit = min((area_w - gap - 120) / need_w, (area_h - gap - 120) / need_h)
        scale = next((s for s in STD_SCALES if s * PX_PER_MM_1TO1 <= fit), STD_SCALES[-1])
        ppm = scale * PX_PER_MM_1TO1
        page = Image.new("RGB", A4, (255, 255, 255))
        dr = ImageDraw.Draw(page)
        dr.rectangle([(20, 20), (A4[0] - 20, A4[1] - 20)], outline=(0, 0, 0), width=3)
        ox, oy = 90, 110                                  # cizim alani sol ust
        wf, hf, ws, hs = _view_px(X, ppm), _view_px(Z, ppm), _view_px(Y, ppm), _view_px(Y, ppm)
        # kamera yuksekligi goruntunun gercek piksel boyundan: olcek tam ppm olur (int yuvarlama hatasi yok)
        # on gorunus: +Y'ye bakar, yukari +Z
        img_f, _ = _ortho_shot(view, c, (0, 1, 0), (0, 0, 1), hf / ppm, (wf, hf))
        # yandan (soldan) gorunus: +X'e bakar, yukari +Z -> 1. aci izdusumunde on gorunusun SAGINA
        img_s, _ = _ortho_shot(view, c, (1, 0, 0), (0, 0, 1), hf / ppm, (ws, hf))
        # ustten gorunus: -Z'ye bakar, yukari +Y -> 1. aci izdusumunde on gorunusun ALTINA
        img_t, _ = _ortho_shot(view, c, (0, 0, -1), (0, 1, 0), hs / ppm, (wf, hs))
        fx, fy = ox + 60, oy + 40
        sx, sy = fx + wf + gap, fy
        tx, ty = fx, fy + hf + gap
        page.paste(img_f, (fx, fy))
        page.paste(img_s, (sx, sy))
        page.paste(img_t, (tx, ty))
        for (lx, ly, lab) in ((fx, fy + hf + 4, "ON GORUNUS"), (sx, sy + hf + 4, "SOLDAN GORUNUS"),
                              (tx, ty + hs + 4, "USTTEN GORUNUS")):
            dr.text((lx + 4, ly), lab, fill=(60, 60, 60), font=f_small)
        # olculer: parca kenarlari goruntu merkezine gore tam olcekte; cizgiler gorunusun disinda
        ex0, ex1 = fx + (wf - X * ppm) / 2, fx + (wf + X * ppm) / 2
        ez0, ez1 = fy + (hf - Z * ppm) / 2, fy + (hf + Z * ppm) / 2
        ey0, ey1 = sx + (ws - Y * ppm) / 2, sx + (ws + Y * ppm) / 2
        et0, et1 = ty + (hs - Y * ppm) / 2, ty + (hs + Y * ppm) / 2
        _dim_h(dr, ex0, ex1, fy - 14, "%.1f" % X, f_med, ext_from=ez0)
        _dim_v(dr, ez0, ez1, fx - 14, "%.1f" % Z, f_med, ext_from=ex0)
        _dim_h(dr, ey0, ey1, sy - 14, "%.1f" % Y, f_med, ext_from=ez0)
        _dim_v(dr, et0, et1, tx - 14, "%.1f" % Y, f_med, ext_from=ex0)
        # sag sutun: izometrik + baslik blogu
        rx = 1220
        iso = tiles[name].resize((480, int(480 * TILE[1] / TILE[0])))
        page.paste(iso, (rx, 60))
        dr.rectangle([(rx, 60), (rx + 480, 60 + iso.height)], outline=(180, 180, 180), width=1)
        setting = PRINT_SETTINGS.get(d["kind"], ("", "-"))[1]
        lines = [
            ("PARCA", name), ("ACIKLAMA", _piece_desc(d)),
            ("ADET", "%d%s" % (d["qty"], " (sol + sag, ayna)" if d["mirror"] else "")),
            ("MALZEME", "%s, renk: %s" % (material(d["kind"]), color_name(d["kind"]))), ("BASKI", setting),
            ("KUTLE", "%.1f g (adet basi, tahmini)" % d["mass_g"]),
            ("ZARF", "%.1f x %.1f x %.1f mm" % (X, Y, Z)),
            ("YON", d["why"] or "otomatik (genis temas, az destek)"),
            ("DESTEK", "gerekli (tabladan)" if d["support"] else "gerekmez"),
            ("OLCEK", "%s (A4, 150 dpi)" % _scale_label(scale)), ("IZDUSUM", "ISO E - 1. aci, baski yonunde"),
            ("DOSYA", ", ".join(d["files"])),
        ]
        yy = 60 + iso.height + 16
        dr.rectangle([(rx, yy), (rx + 480, A4[1] - 40)], outline=(0, 0, 0), width=2)
        dr.text((rx + 14, yy + 8), "OpenDuck", fill=(0, 0, 0), font=f_title)
        dr.text((rx + 14, yy + 56), "Microduck benzeri ordek robot", fill=(80, 80, 80), font=f_small)
        yy += 86
        for key, val in lines:
            dr.line([(rx, yy), (rx + 480, yy)], fill=(160, 160, 160), width=1)
            dr.text((rx + 10, yy + 5), key, fill=(110, 110, 110), font=f_small)
            wl = _wrap(dr, val, f_small, 330)
            for j, ln in enumerate(wl):
                dr.text((rx + 140, yy + 5 + 22 * j), ln, fill=(0, 0, 0), font=f_small)
            yy += 22 * len(wl) + 9
        dr.text((rx + 10, A4[1] - 70), "Sayfa %d / %d    %s" % (k + 1, n_sheets, today), fill=(80, 80, 80), font=f_small)
        dr.text((ox, 40), "Olculer mm. Genel zarf olculeri verilmistir; delikler ve detaylar icin STEP/STL kullanin.",
                fill=(90, 90, 90), font=f_small)
        fn = os.path.join(out_dir, "sheet_%02d_%s.png" % (k + 1, name))
        page.save(fn, optimize=True)
        sheets.append(fn)
    App.closeDocument(doc.Name)
    return render_catalog(info, tiles, out_dir), sheets


TILE = (420, 300)          # katalog karesi (px)


def render_catalog(info, tiles, out_dir, cols=6, rows=4):
    """Yatay A4 katalog sayfalari (6 x 4 kare/sayfa, gerekirse birden fazla sayfa). Doner: dosya listesi."""
    from PIL import Image, ImageDraw
    m, top = 40, 118
    tw = (A4[0] - 2 * m) // cols
    iw = tw - 12
    ih = int(iw * TILE[1] / TILE[0])
    rh = (A4[1] - top - 30) // rows
    f_name, f_txt, f_small, f_big, f_sub = _font(18, True), _font(17), _font(14), _font(34, True), _font(20)
    names = list(info)
    per = cols * rows
    n_pages = int(math.ceil(len(names) / float(per)))
    total = sum(d["qty"] for d in info.values())
    files = []
    for pg in range(n_pages):
        page = Image.new("RGB", A4, (255, 255, 255))
        dr = ImageDraw.Draw(page)
        dr.text((m, 26), "OpenDuck - 3B baski parcalari (%d cesit, %d adet)%s" % (
            len(info), total, "  - sayfa %d/%d" % (pg + 1, n_pages) if n_pages > 1 else ""), fill=(0, 0, 0), font=f_big)
        dr.text((m, 74), "baski yonunde, izometrik; renkler oneridir; adet sol + sag dahil; her kare bir STL cesidi",
                fill=(90, 90, 90), font=f_sub)
        for j, name in enumerate(names[pg * per:(pg + 1) * per]):
            d = info[name]
            x, y = m + (j % cols) * tw, top + (j // cols) * rh
            page.paste(tiles[name].resize((iw, ih)), (x, y))
            dr.rectangle([(x, y), (x + iw, y + ih)], outline=(215, 215, 215), width=1)
            lab = "%s  x%d" % (name, d["qty"])
            f = f_name if dr.textlength(lab, font=f_name) <= iw else _font(15, True)
            dr.text((x + 2, y + ih + 3), lab, fill=(0, 0, 0), font=f)
            for li, t in enumerate(("%s, %.0f g%s" % (material(d["kind"]), d["mass_g"], ", destekli" if d["support"] else ""),
                                    "%.1f x %.1f x %.1f mm" % d["size"])):
                f = f_txt if dr.textlength(t, font=f_txt) <= iw else f_small
                dr.text((x + 2, y + ih + 25 + 21 * li), t, fill=(60, 60, 60), font=f)
        fn = os.path.join(out_dir, "parts_catalog%s.png" % ("" if pg == 0 else "_%d" % (pg + 1)))
        page.save(fn, optimize=True)
        files.append(fn)
    return files


# ---------------------------------------------------------------------------
# PDF kitapcik
# ---------------------------------------------------------------------------
def _fit_page(img, title, subtitle=""):
    from PIL import Image, ImageDraw
    page = Image.new("RGB", A4, (255, 255, 255))
    dr = ImageDraw.Draw(page)
    dr.text((60, 40), title, fill=(0, 0, 0), font=_font(40, True))
    if subtitle:
        dr.text((60, 95), subtitle, fill=(90, 90, 90), font=_font(22))
    box = (A4[0] - 120, A4[1] - 190)
    im = img.copy()
    im.thumbnail(box, Image.LANCZOS)
    page.paste(im, ((A4[0] - im.width) // 2, 150 + (box[1] - im.height) // 2))
    return page


def _text_pages(title, lines):
    """Metin sayfalari; tasan satirlar yeni sayfaya gecer. '# ' ile baslayan satir alt basliktir."""
    from PIL import Image, ImageDraw
    pages, dr, y = [], None, 0
    for ln in lines:
        bold = ln.startswith("# ")
        if dr is None or y + (40 if bold else 31) > A4[1] - 50:
            page = Image.new("RGB", A4, (255, 255, 255))
            dr = ImageDraw.Draw(page)
            dr.text((60, 40), title + (" (devam)" if pages else ""), fill=(0, 0, 0), font=_font(40, True))
            pages.append(page)
            y = 120
        dr.text((70, y), ln[2:] if bold else ln, fill=(0, 0, 0), font=_font(24, True) if bold else _font(21))
        y += 40 if bold else 31
    return pages


def assembly_lines(model):
    """Montaj sirasi metni (PDF). Parca adlari modelden kontrol edilir."""
    parts = print_parts(model)
    tray = ("batarya yuvasini (battery_tray) sasiye tak" if "battery_tray" in parts
            else "sasideki batarya yuvasini kontrol et")
    return [
        "0) Servolar, montajdan ONCE: her XL330'u TEK BASINA bagla (fabrikadan hepsi ID 1 gelir), Dynamixel Wizard ile",
        "   ID'sini yukaridaki tabloya gore ata ve etiketle, servoyu HOME acisina sur (FCStd 'Poz' tablosu, B sutunu).",
        "   Horn'u ve braketi ANCAK bundan sonra, servo HOME'dayken tak.",
        "1) Govde: sasiye kalca yaw servolarini ve boyun servosunu (ID 30) tak; IMU kartini ve guc kartini sasiye",
        "   vidala; %s." % tray,
        "2) Her bacakta: hip_bracket + roll servosu -> hip_cradle_bracket -> uyluk (pitch + diz servosu, ic plaka,",
        "   kanat) -> incik (bilek servosu, braket, ped) -> ayak (U braketi 4x M2 ile ayak govdesine, taban).",
        "3) Boyun: iki servoyu ek plakalariyla (neck_splice) birlestir, kafa egim braketini (head_tilt_bracket) tak;",
        "   yaw servosunu kafa donus braketinin (head_turn_bracket) iki parcasi arasina al: roll horn'u arka parcaya,",
        "   on parcadaki 10 mm pim roll ekseninin karsi yatagina.",
        "4) Kafa: ic cerceve + roll servosu + gaga servosu, kart/HAT, kamera, ToF, hoparlor; alt kabuk, yuz plakasi,",
        "   goz halkasi, ust gaga + ust gaga pedi (TPU, ust gaganin altina); alt gaga (cene) servoya, alt gaga pedi",
        "   (TPU) alt gaganin ustune; en son ust kaput.",
        "5) Govdeyi kapat: kablolari topla, bataryayi yuvasina yerlestir, govde kabugunun sol ve sag yarilarini",
        "   (torso_shell_left / torso_shell_right) sasiye tak.",
    ]


def build_pdf(model, img_dir, docs_dir, pdf_path, summ, catalog, sheets, exploded, info):
    from PIL import Image
    hero = Image.open(os.path.join(img_dir, "hero.png"))
    cover = _fit_page(hero, "OpenDuck - montaj ve baski kitapcigi",
                      "%s | %.0f mm | ~%.0f g | %d x %s | %s"
                      % (summ["robot"], summ["height_stand_mm"], summ["mass_kg"] * 1000, summ["servo_count"],
                         summ["servo_model"], datetime.date.today().isoformat()))
    pages = [cover]
    titles = {"exploded_iso": ("Patlatilmis montaj", "tum robot, HOME pozunda; eklem ofsetleri buyutuldu"),
              "exploded_side": ("Patlatilmis montaj - yandan", ""),
              "exploded_head_iso": ("Kafa alt montaji", "boyun, kafa egim/donus braketleri, kafa ici; yuz yigini goz "
                                    "ekseni boyunca, gaga parcalari (ust gaga, ust ped, alt ped, alt gaga) asagi dogru"),
              "exploded_leg_iso": ("Sol bacak alt montaji", "kalca yaw -> roll -> pitch -> diz -> ayak bilegi; sag bacak ayna")}
    for fn in exploded:
        key = os.path.splitext(os.path.basename(fn))[0]
        t, s = titles.get(key, (key, ""))
        pages.append(_fit_page(Image.open(fn), t, s))
    buy = ["# Satin alinacak parcalar"] + ["%2d x  %s  %s" % (n, p, ("- " + note) if note else "")
                                         for n, p, note in model.meta["bom_buy"]]
    buy += ["", "# 3B baski", "%d baski parcasi cesidi, toplam %d parca (sol + sag dahil); liste: katalog sayfalari ve "
            "print/PRINT.md." % (len(info), sum(d["qty"] for d in info.values())),
            "", "# Servo kimlikleri (ID)", "sol bacak 20-24, sag bacak 10-14 (hip_yaw, hip_roll, hip_pitch, knee, ankle)",
            "neck_pitch 30, head_pitch 31, head_yaw 32, head_roll 33, mouth (gaga) 34", ""]
    pages += _text_pages("Malzeme listesi", buy)
    pages += _text_pages("Montaj sirasi (oneri)", assembly_lines(model))
    pages += [Image.open(fn).convert("RGB") for fn in catalog]
    pages += [Image.open(fn).convert("RGB") for fn in sheets]
    pages[0].save(pdf_path, "PDF", resolution=150.0, save_all=True, append_images=pages[1:])
    return pdf_path


def build_all(model, doc, out_dir, summ, gui=True):
    """build/ altina: print/ (STL + PRINT.md), openduck_print_parts.zip, docs/ (gorseller, sayfalar), PDF."""
    print_dir = os.path.join(out_dir, "print")
    info = export_print_set(model, print_dir, os.path.join(out_dir, "openduck_print_parts.zip"), summ)
    res = {"print_dir": print_dir, "parts": len(info), "pieces": sum(d["qty"] for d in info.values())}
    if gui:
        docs_dir = os.path.join(out_dir, "docs")
        exploded = render_exploded(model, docs_dir)
        catalog, sheets = render_parts(model, docs_dir, info, summ)
        pdf = build_pdf(model, os.path.join(out_dir, "img"), docs_dir, os.path.join(out_dir, "openduck_drawings.pdf"),
                        summ, catalog, sheets, exploded, info)
        res.update({"exploded": exploded, "catalog": catalog, "sheets": len(sheets), "pdf": pdf})
        App.setActiveDocument(doc.Name)
    return res
