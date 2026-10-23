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

Konfigde "check" bolumu varsa (TR-35, spec A1; P2-0) ayrica:
4) Komsu eklem ciftleri pair_steps x pair_steps izgarada birlikte (pair_range: required | limits).
5) Bacak-bacak: iki bacak hip_roll ve hip_yaw ic uclarinda, hip_pitch / knee uclari (leg_leg_range).
Kablo demetleri (Item kind "harness") butun taramalara dahildir; ayni eklem gecisinin iki tupu
("<eklem>_harness_in/out") gecis noktasinda bilerek birlesir ve birbirine karsi sayilmaz. Rapor sonunda sureler.
Modellenen temaslar (check.contact_pairs, spec 3.2.2): 4) ve 5)'te birlesik uc pozlarda beklenen, MJCF'ye oz-carpisma
cifti olarak giren link ciftleri sayilmaz, ayri listelenir; kablo demeti (contact_count_kinds) yine sayilir.

Modul suzgeci (P2a/b/c hizli dongu): OPENDUCK_CHECK_LINKS=legs | trunk | head | all | <link>,<link>... Yalniz en az
bir parcasi secili link'lerde olan ciftler; pozlar secili modulun eklemleri (trunk: hepsi). 27 cm raporu suzgecsiz
bayt-esittir.

  freecadcmd -c "import runpy; runpy.run_path('cad/check.py', run_name='__main__')"
"""
import collections
import itertools
import os
import random
import sys
import time

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


def joint_range(model, j, which="limits"):
    """Eklemin tarama araligi (derece): "limits" -> eklem siniri, "required" -> konfig required_deg (sag bacakta
    (-ust, -alt), sinirlarla ayni ayna kurali). required'da olmayan eklem -> sinir."""
    if which == "required":
        key = openduck.joint_key(j["name"])
        req = model.cfg.get("required_deg", {}).get(key)
        if req:
            lo, hi = req
            return (-hi, -lo) if j["name"].startswith("right_") else (lo, hi)
    return j["lower"], j["upper"]


def _steps(lo, hi, n):
    return [lo + (hi - lo) * k / (n - 1.0) for k in range(n)]


def single_poses(model, which="limits", steps=SWEEP_STEPS, joints=None):
    """HOME + her eklemin (joints: ad kumesi) kendi araliginda tek basina taranmasi (digerleri HOME)."""
    stand = model.pose("stand")
    out = [("stand", stand)]
    for j in model.joints:
        if joints is not None and j["name"] not in joints:
            continue
        for a in _steps(*(joint_range(model, j, which) + (steps,))):
            p = dict(stand)
            p[j["name"]] = a
            out.append(("%s=%.0f" % (j["name"], a), p))
    return out


def adjacent_pairs(model):
    """Komsu (ebeveyn-cocuk) eklem ciftleri: (ust eklem, alt eklem); alt eklemin ebeveyn link'i ust eklemin
    cocugu."""
    by_child = {j["child"]: j for j in model.joints}
    return [(by_child[j["parent"]], j) for j in model.joints if j["parent"] in by_child]


def pair_poses(model, steps=5, which="required", joints=None):
    """Komsu eklem ciftleri steps x steps izgarada birlikte (digerleri HOME)."""
    stand = model.pose("stand")
    out = []
    for ja, jb in adjacent_pairs(model):
        if joints is not None and not ({ja["name"], jb["name"]} & joints):
            continue
        for a in _steps(*(joint_range(model, ja, which) + (steps,))):
            for b in _steps(*(joint_range(model, jb, which) + (steps,))):
                p = dict(stand)
                p[ja["name"]], p[jb["name"]] = a, b
                out.append(("%s=%.0f,%s=%.0f" % (ja["name"], a, jb["name"], b), p))
    return out


def leg_leg_poses(model, which="required"):
    """Bacak-bacak: iki bacakta hip_roll iceri uc (sol alt, sag ust), hip_yaw iki ic uc modu (ikisi de burun ice /
    ikisi de burun disa), her bacakta hip_pitch ve knee uclari bagimsiz (4 x 4 x 2 = 32 poz)."""
    stand = model.pose("stand")
    J = {j["name"]: j for j in model.joints}
    if "left_hip_roll" not in J:
        return []
    rng = lambda n: joint_range(model, J[n], which)
    out = []
    for mode, (ly, ry) in (("ice", (rng("left_hip_yaw")[1], rng("right_hip_yaw")[0])),
                           ("disa", (rng("left_hip_yaw")[0], rng("right_hip_yaw")[1]))):
        for lp in rng("left_hip_pitch"):
            for lk in rng("left_knee"):
                for rp in rng("right_hip_pitch"):
                    for rk in rng("right_knee"):
                        p = dict(stand)
                        p.update({"left_hip_roll": rng("left_hip_roll")[0], "right_hip_roll": rng("right_hip_roll")[1],
                                  "left_hip_yaw": ly, "right_hip_yaw": ry, "left_hip_pitch": lp, "left_knee": lk,
                                  "right_hip_pitch": rp, "right_knee": rk})
                        out.append(("legleg %s L%.0f/%.0f R%.0f/%.0f" % (mode, lp, lk, rp, rk), p))
    return out


def random_poses(model, n, seed=1):
    rnd = random.Random(seed)
    return [("random_%02d" % i, {j["name"]: rnd.uniform(j["lower"], j["upper"]) for j in model.joints})
            for i in range(n)]


def harness_crossing(it):
    """Kablo demeti parcasinin eklem gecisi ("left_knee_harness_in" -> "left_knee") ya da None."""
    if it.kind != "harness":
        return None
    for suf in ("_harness_in", "_harness_out"):
        if it.name.endswith(suf):
            return it.name[:-len(suf)]
    return None


def allowed_pair(ia, ib):
    """Bilerek temas eden farkli link parcalari: ayni eklem gecisinin iki kablo tupu."""
    ca = harness_crossing(ia)
    return ca is not None and ca == harness_crossing(ib)


def contact_rules(model, section):
    """check.contact_pairs (spec 3.2.2): MJCF oz-carpisma cifti olarak modellenen temaslar. Kural: {"links": [a, b]
    (sol/sag oneksiz), "side": same | cross | any, "sections": [pairs | leg_leg], "why": ...}. contact_count_kinds
    (varsayilan kablo demeti) turundeki parcalar yine sayilir. Doner f(parca_a, parca_b) -> gerekce | None, ya da
    kural yoksa None."""
    ext = model.cfg.get("check") or {}
    rules = [r for r in ext.get("contact_pairs", []) if section in r.get("sections", ())]
    if not rules:
        return None
    count = set(ext.get("contact_count_kinds", ["harness"]))
    where = {it.name: (ln, it) for ln, l in model.links.items() for it in l.items}
    side = lambda ln: ln.split("_")[0] if ln.startswith(("left_", "right_")) else ""

    def allow(na, nb):
        (la, ia), (lb, ib) = where[na], where[nb]
        if ia.kind in count or ib.kind in count:
            return None
        ks = sorted((openduck.joint_key(la), openduck.joint_key(lb)))
        for r in rules:
            sd = r.get("side", "any")
            if ks == sorted(r["links"]) and (sd == "any" or (sd == "same") == (side(la) == side(lb))):
                return r.get("why", "")
        return None
    return allow


def selected_links(model, spec=None):
    """OPENDUCK_CHECK_LINKS -> link kumesi (None: hepsi). legs/trunk/head: modul link'leri (model.meta["modules"])."""
    spec = os.environ.get("OPENDUCK_CHECK_LINKS", "") if spec is None else spec
    if spec in ("", "all"):
        return None
    mods = model.meta.get("modules", {})
    out = set()
    for tok in spec.split(","):
        tok = tok.strip()
        if tok in mods:
            out.update(mods[tok])
        elif tok in model.links:
            out.add(tok)
        else:
            raise ValueError("OPENDUCK_CHECK_LINKS: bilinmeyen modul/link %r (moduller: %s)" % (tok, ", ".join(mods)))
    return out


def selected_joints(model, links):
    """Secili link'leri hareket ettiren eklemler (cocugu secili link olanlar ve onlarin atalari); trunk seciliyse
    ya da suzgec yoksa None (hepsi)."""
    if links is None or "trunk" in links:
        return None
    by_child = {j["child"]: j for j in model.joints}
    out = set()
    for ln in links:
        while ln in by_child:
            out.add(by_child[ln]["name"])
            ln = by_child[ln]["parent"]
    return out


def check_pose(model, pose, links=None):
    shapes = model.posed_shapes(pose)
    hits = []
    for (la, ia, sa), (lb, ib, sb) in itertools.combinations(shapes, 2):
        if la == lb or not sa.BoundBox.intersect(sb.BoundBox):
            continue
        if links is not None and la not in links and lb not in links:
            continue
        if allowed_pair(ia, ib):
            continue
        try:
            vol = sa.common(sb).Volume
        except Exception:   # OCC nadiren basarisiz olur; raporla
            vol = -1.0
        if vol > TOL or vol < 0:
            hits.append((vol, ia.name, ib.name))
    return hits


def same_link_overlaps(model, links=None):
    """HOME pozunda ayni link'teki parca ciftlerinin kesisimleri: [(link, hacim, a, b, izin_gerekcesi|None)]."""
    hits = []
    for ln, link in model.links.items():
        if links is not None and ln not in links:
            continue
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


def _scan(model, poses, links, lines, label_w=26, scope=None, info=None, allow=None, modeled=None):
    """Pozlari tara, carpismalari satir satir yaz; carpisma sayisi. scope(poz adi) -> link kumesi verilirse yalniz iki
    parcasi da o kumede olan carpismalar sayilir, digerleri info sayacina (bilgi amacli) gider. allow(a, b) gerekce
    verirse (contact_rules) carpisma sayilmaz, modeled[(a, b)] = [poz sayisi, en buyuk hacim] olur."""
    n = 0
    item_link = {it.name: ln for ln, l in model.links.items() for it in l.items}
    for name, pose in poses:
        hits = check_pose(model, pose, links)
        if scope is not None:
            sc = scope(name)
            out = [h for h in hits if not (item_link[h[1]] in sc and item_link[h[2]] in sc)]
            hits = [h for h in hits if h not in out]
            for vol, a, b in out:
                info[(a, b)] += 1
        if allow is not None:
            ok = [h for h in hits if allow(h[1], h[2]) is not None]
            hits = [h for h in hits if h not in ok]
            for vol, a, b in ok:
                c = modeled.setdefault((a, b), [0, 0.0])
                c[0], c[1] = c[0] + 1, max(c[1], vol)
        n += len(hits)
        for vol, a, b in sorted(hits, reverse=True):
            lines.append("  %-*s %8.1f mm3  %s <-> %s" % (label_w, name, vol, a, b))
    return n


def iface_envelopes(model):
    """P2-0 modul zarflari (params.iface.envelopes) -> {link: kati, sifir poz}. Bacak anahtarlari (hip, thigh, ...)
    iki tarafa (sag: mirror_y); {"add": [...], "cut": [...]} bicimi = birlesim eksi kesiciler (geom.envelope_solid)."""
    from geom import envelope_solid, mirror_y
    env = (model.p.get("iface") or {}).get("envelopes") or {}
    out = collections.OrderedDict()
    for key, spec in env.items():
        add, cut = (spec["add"], spec.get("cut", [])) if isinstance(spec, dict) else (spec, [])
        s = envelope_solid(add)
        if cut:
            s = s.cut(envelope_solid(cut))
        if key in model.links:
            out[key] = s
        for side in ("left_", "right_"):
            if side + key in model.links:
                out[side + key] = mirror_y(s) if side == "right_" else s
    return out


def _posed(model, ln, shape, pls):
    s = shape.copy()
    s.translate(-model.links[ln].origin)
    s.Placement = pls[ln].multiply(s.Placement)
    return s


def iface_report(model, links, joints, lines):
    """Bolum 6: (a) icerme: zarfli link'lerin parcalari (kablo demeti haric) kendi zarfi icinde; (b) giris: tek eklem
    (sinir) + bacak-bacak taramasinda bir modulun parcalari baska modulun zarfina girmez; (c) zarf-zarf: farkli
    modul zarflari kesismez. Doner (icerme, giris, zarf-zarf) sayilari."""
    env = iface_envelopes(model)
    mod = model.link_module
    n_in = 0
    lines.append("  (a) icerme (kendi zarfinin disinda kalan hacim > %.0f mm3)" % TOL)
    for ln, e in env.items():
        if links is not None and ln not in links:
            continue
        for it in model.links[ln].items:
            if it.kind == "harness":
                continue
            v = it.shape.cut(e).Volume
            if v > TOL:
                n_in += 1
                lines.append("      %-18s %8.1f mm3 disarida  %s" % (ln, v, it.name))
    poses = single_poses(model, "limits", SWEEP_STEPS, joints)
    legs = set(model.meta.get("modules", {}).get("legs", []))
    if links is None or links & legs:
        poses += leg_leg_poses(model, "required")
    n_x = n_ee = 0
    agg = collections.OrderedDict()
    eagg = collections.OrderedDict()
    for pname, pose in poses:
        pls = model.link_placements(pose)
        pe = {ln: _posed(model, ln, e, pls) for ln, e in env.items()}
        for ln, l in model.links.items():
            for it in l.items:
                if it.kind == "harness":
                    continue
                s = None
                for el, e in pe.items():
                    if mod[el] == mod[ln] or (links is not None and ln not in links and el not in links):
                        continue
                    s = s or _posed(model, ln, it.shape, pls)
                    if not s.BoundBox.intersect(e.BoundBox):
                        continue
                    v = s.common(e).Volume
                    if v > TOL:
                        k = (it.name, el)
                        if k not in agg or v > agg[k][0]:
                            agg[k] = (v, pname)
        els = list(pe)
        for i, a in enumerate(els):
            for b in els[i + 1:]:
                if mod[a] == mod[b] or not pe[a].BoundBox.intersect(pe[b].BoundBox):
                    continue
                v = pe[a].common(pe[b]).Volume
                if v > TOL:
                    k = (a, b)
                    if k not in eagg or v > eagg[k][0]:
                        eagg[k] = (v, pname)
    lines.append("  (b) baska modulun zarfina giris (%d poz: tek eklem sinir + bacak-bacak)" % len(poses))
    for (a, el), (v, pname) in sorted(agg.items(), key=lambda kv: -kv[1][0]):
        lines.append("      %8.1f mm3  %s -> %s zarfi  (%s)" % (v, a, el, pname))
    n_x = len(agg)
    lines.append("  (c) zarf-zarf kesisimi (farkli moduller)")
    for (a, b), (v, pname) in sorted(eagg.items(), key=lambda kv: -kv[1][0]):
        lines.append("      %8.1f mm3  %s <-> %s  (%s)" % (v, a, b, pname))
    n_ee = len(eagg)
    lines.append("  -> icerme %d, giris %d, zarf-zarf %d" % (n_in, n_x, n_ee))
    return n_in, n_x, n_ee


def _modeled_lines(model, section, modeled, lines):
    """Modellenen temaslar bolumu (yalniz kural varsa yazilir); poz-cift sayisi."""
    if contact_rules(model, section) is None:
        return 0
    lines.append("  modellenen temaslar (sayilmaz; MJCF oz-carpisma cifti, check.contact_pairs): %d poz-cift"
                 % sum(c for c, v in modeled.values()))
    for (a, b), (c, v) in sorted(modeled.items(), key=lambda kv: (-kv[1][0], -kv[1][1])):
        lines.append("  %3dx  en cok %8.1f mm3  %s <-> %s" % (c, v, a, b))
    return sum(c for c, v in modeled.values())


def run(n_random=40):
    model = openduck.OpenDuck()
    links = selected_links(model)
    joints = selected_joints(model, links)
    ext = model.cfg.get("check") or {}
    timing = collections.OrderedDict()
    lines = ["# OpenDuck carpisma raporu", "", "## 1) Tasarim kontrolu (tek eklem taramasi)"]
    if links is not None:     # suzgec yalniz istenince basliga yazilir (27 cm raporu bayt-esit)
        lines.insert(1, "suzgec: %s (%d link)" % (os.environ.get("OPENDUCK_CHECK_LINKS"), len(links)))
    t0 = time.time()
    dposes = [(n, p) for n, p in design_poses(model)
              if joints is None or n in ("zero", "stand") or n.split("=")[0] in joints]
    design_hits = _scan(model, dposes, links, lines)
    lines.append("  -> %d poz, %d carpisma" % (len(dposes), design_hits))
    timing["tek eklem"] = time.time() - t0

    lines += ["", "## 2) Rastgele birlesik pozlar (bilgi amacli)"]
    t0 = time.time()
    free, pairs = 0, collections.Counter()
    poses = random_poses(model, n_random)
    for name, pose in poses:
        hits = check_pose(model, pose, links)
        free += not hits
        for vol, a, b in hits:
            pairs[(a, b)] += 1
    lines.append("  carpismasiz: %d / %d" % (free, len(poses)))
    for (a, b), c in pairs.most_common():
        lines.append("  %3dx  %s <-> %s" % (c, a, b))
    timing["rastgele"] = time.time() - t0

    lines += ["", "## 3) Ayni link ici cakismalar (HOME pozu, > %.0f mm3)" % TOL]
    t0 = time.time()
    same = same_link_overlaps(model, links)
    bad = [h for h in same if h[4] is None]
    for ln, vol, a, b, why in sorted(same, key=lambda h: (h[4] is not None, h[0], -h[1])):
        lines.append("  %-18s %8.1f mm3  %s <-> %s%s" % (ln, vol, a, b, "   [izinli: %s]" % why if why else ""))
    lines.append("  -> %d cakisma (%d izinli)" % (len(bad), len(same) - len(bad)))
    timing["ayni link"] = time.time() - t0

    ext_hits = 0
    if ext:
        steps = int(ext.get("pair_steps", 5))
        prange = ext.get("pair_range", "required")
        pscope = ext.get("pair_scope", "all")
        lines += ["", "## 4) Komsu eklem ciftleri (%d x %d, %s araliklari; kapsam: %s)"
                  % (steps, steps, prange, pscope)]
        t0 = time.time()
        pposes = pair_poses(model, steps, prange, joints)
        scope, info = None, collections.Counter()
        if pscope == "module":      # ciftin modulu icindeki carpismalar sayilir; govde/kafa <-> uzuv bilgi amacli
            jmod = {j["name"]: model.link_module[j["child"]] for j in model.joints}
            mlinks = {md: set(lns) for md, lns in model.meta["modules"].items()}
            scope = lambda pname: mlinks[jmod[pname.split(",")[0].split("=")[0]]]
        pmod = {}
        n = _scan(model, pposes, links, lines, 40, scope, info, contact_rules(model, "pairs"), pmod)
        lines.append("  -> %d poz, %d carpisma" % (len(pposes), n))
        n_mod = _modeled_lines(model, "pairs", pmod, lines)
        if scope is not None:
            lines.append("  modul disi temaslar (bilgi amacli; MJCF oz-carpisma ciftleri, spec 3.2.2): %d poz-cift"
                         % sum(info.values()))
            for (a, b), c in info.most_common():
                lines.append("  %3dx  %s <-> %s" % (c, a, b))
        ext_hits += n
        timing["komsu cift"] = time.time() - t0
        if ext.get("leg_leg"):
            lrange = ext.get("leg_leg_range", "required")
            lines += ["", "## 5) Bacak-bacak (hip_roll + hip_yaw ic uclar, hip_pitch/knee uclari, %s)" % lrange]
            t0 = time.time()
            legs = set(model.meta.get("modules", {}).get("legs", []))
            lposes = leg_leg_poses(model, lrange) if links is None or links & legs else []
            lmod = {}
            n = _scan(model, lposes, links, lines, 40, allow=contact_rules(model, "leg_leg"), modeled=lmod)
            lines.append("  -> %d poz, %d carpisma" % (len(lposes), n))
            n_mod += _modeled_lines(model, "leg_leg", lmod, lines)
            ext_hits += n
            timing["bacak-bacak"] = time.time() - t0
        n_if = (0, 0, 0)
        if (model.p.get("iface") or {}).get("envelopes"):
            lines += ["", "## 6) Modul zarflari (P2-0 arayuzu, params.iface.envelopes)"]
            t0 = time.time()
            n_if = iface_report(model, links, joints, lines)
            ext_hits += sum(n_if)
            timing["zarflar"] = time.time() - t0
        nh = sum(1 for l in model.links.values() for it in l.items if it.kind == "harness")
        lines += ["", "## Ozet", "  kablo demeti parcasi: %d (taramalara dahil)" % nh,
                  "  carpisma: tek eklem %d, komsu cift + bacak-bacak %d, ayni link %d; zarf: icerme %d, giris %d, "
                  "zarf-zarf %d" % ((design_hits, ext_hits - sum(n_if), len(bad)) + tuple(n_if)),
                  "  modellenen temas (sayilmaz, MJCF cifti): %d poz-cift" % n_mod,
                  "  sure (s): " + ", ".join("%s %.0f" % kv for kv in timing.items())
                  + ", toplam %.0f" % sum(timing.values())]
    run.timing = timing
    return design_hits + ext_hits, free, len(poses), len(bad), "\n".join(lines)


if __name__ == "__main__":
    out = os.environ.get("OPENDUCK_CHECK_OUT", os.path.join(
        os.environ.get("OPENDUCK_OUT", os.path.join(HERE, "..", "build")), "interference.txt"))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    n_design, n_free, n_rand, n_same, txt = run(int(os.environ.get("OPENDUCK_RANDOM_POSES", "40")))
    with open(out, "w") as f:
        f.write(txt + "\n")
    sys.stdout.write(txt + "\n")
    sys.stdout.write("sure (s): %s\n" % ", ".join("%s %.1f" % kv for kv in getattr(run, "timing", {}).items()))
