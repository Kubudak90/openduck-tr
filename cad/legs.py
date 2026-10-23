# -*- coding: utf-8 -*-
"""
Bacak geometri uretecleri: kalca yaw/roll/pitch, diz, ayak bilegi eklemleri ve braketleri, ayak + taban.
Sol bacak kurulur, sag bacak XZ duzlemine gore aynalanir (Y eksenli eklemlerin ekseni ters, sinirlar (-ust, -alt)).

Uretec arayuzu (openduck.OpenDuck._build): GENERATORS[konfig "generators.legs"](m, sv); m model (m.p parametreler,
m._add / m._joint kayit, m._servo(anahtar, ayna) yerlestirilmis (govde, horn), m.servo_frame, m.servo_mass /
m.horn_mass, m.masses, m.limits, m.home, m.meta, m.shared uretecler arasi sekiller), sv eklem anahtari -> servo
turu kaydi (servos.SERVO_TYPES). Servo govdesi parcasinin adi "[left_|right_]<anahtar>_servo" olmali
(openduck.servo_signs isaret kontrolu).
"""
import math

import Part

import servos
from geom import (V, _parallel, box, fillet, fillet_axis, frame, hull2d, mirror_y, moved, prism_xz, rbox, rotated,
                  stadium_y, xcyl, ycyl, zcyl)

LEG_KEYS = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")


def _leg_items_xl330(m, sv):
    """Sol bacagin parcalari [(link, ad, sekil, tur, kutle[, carpisma kutusu])] (refactor oncesi
    OpenDuck._leg_items, birebir)."""
    p = m.p
    t = p["plate_t"]
    hp, kn, an = p["j_hip_pitch"], p["j_knee"], p["j_ankle"]
    out = []

    # hip (yaw2roll): yaw horn'una takili, roll servosunu tasir
    roll_b, roll_h = m._servo("hip_roll")
    hb = rbox(-11.5, 18.5, 6.5, 28.5, -6.0, -3.0, 3.0)
    hb = hb.fuse([box(-11.5, -8.5, 6.5, 28.5, -30.0, -6.0),
                  box(-11.5, 12.0, 5.5, 7.5, -14.0, -3.0),
                  box(-11.5, 12.0, 27.5, 29.5, -14.0, -3.0)]).removeSplitter()
    hb = hb.cut(zcyl(1.4, 6.0, 17.5, -7.0, -2.0))
    hb = hb.cut(servos.horn_bolt_holes(sv["hip_yaw"], m._servo("hip_yaw")[1], -7.0, -2.0))
    out += [("hip", "hip_yaw_horn", None, "horn", m.horn_mass("hip_yaw")),
            ("hip", "hip_bracket", hb, "bracket", None),
            ("hip", "hip_roll_servo", roll_b, "servo", m.servo_mass("hip_roll"))]

    # hip_cradle (hip_l): roll horn'u -> pitch horn'u (L braket)
    pitch_b, pitch_h = m._servo("hip_pitch")
    front = box(20.5, 23.5, 8.5, 40.5, -27.0, -8.0)
    side = stadium_y(hp[0], hp[2], 9.5, -17.5, 37.5, 40.5).fuse(box(hp[0], 23.5, 37.5, 40.5, -27.0, -8.0))
    cr = front.fuse(side).removeSplitter()
    cr = fillet_axis(cr, 3.0, (0, 1, 0))
    cr = cr.cut(xcyl(1.4, 17.5, -17.5, 19.0, 25.0)).cut(ycyl(1.4, hp[0], hp[2], 36.0, 42.0))
    cr = cr.cut(servos.horn_bolt_holes(sv["hip_roll"], roll_h, 19.0, 25.0)
                + servos.horn_bolt_holes(sv["hip_pitch"], pitch_h, 36.0, 42.0))
    out += [("hip_cradle", "hip_roll_horn", roll_h, "horn", m.horn_mass("hip_roll")),
            ("hip_cradle", "hip_cradle_bracket", cr, "bracket", None)]

    # uyluk: pitch + diz servolari, ic plaka ve dis "kanat" kabuk
    knee_b, knee_h = m._servo("knee")
    corners = []
    for (x0, x1, z0, z1) in ((-20.5, 13.5, -27.5, -7.5), (-41.8, -21.8, -49.0, -15.0)):
        corners += [(x0 - 3, z0 - 3), (x1 + 3, z0 - 3), (x1 + 3, z1 + 3), (x0 - 3, z1 + 3)]
    outline = hull2d(corners)
    wing = prism_xz(outline, 69.5, 72.0, 6.0)
    try:
        wing = fillet(wing, 1.2, [e for e in wing.Edges if abs(e.BoundBox.YMax - 72.0) < 1e-6
                                  and abs(e.BoundBox.YMin - 72.0) < 1e-6])
    except Exception:
        pass
    wing = wing.cut(ycyl(4.0, -9.0, -26.0, 70.5, 73.0))
    cx = sum(x for x, _ in outline) / len(outline)
    cz = sum(z for _, z in outline) / len(outline)
    inset = [(cx + (x - cx) * 0.9, cz + (z - cz) * 0.9) for x, z in outline]
    rim = prism_xz(outline, 62.5, 69.6, 6.0).cut(prism_xz(inset, 62.0, 70.0, 4.0))
    rim = rim.cut([box(x0 - 0.3, x1 + 0.3, 62.0, 69.5, z0 - 0.3, z1 + 0.3)     # servo sirtlarina girmesin
                   for (x0, x1, z0, z1) in ((-20.5, 13.5, -27.5, -7.5), (-41.8, -21.8, -49.0, -15.0))])
    wing = wing.fuse(rim).removeSplitter()
    inner = prism_xz(outline, 41.5, 43.5, 6.0).common(box(-60.0, 30.0, 40.0, 45.0, kn[2] + 7.5, 0.0))
    inner = inner.cut(ycyl(7.5, hp[0], hp[2], 40.0, 45.0)).cut(ycyl(7.5, kn[0], kn[2], 40.0, 45.0))
    out += [("thigh", "hip_pitch_servo", pitch_b, "servo", m.servo_mass("hip_pitch")),
            ("thigh", "knee_servo", knee_b, "servo", m.servo_mass("knee")),
            ("thigh", "thigh_inner_plate", inner, "bracket", None),
            ("thigh", "thigh_wing", wing, "shell", None)]

    # incik: diz horn'u -> bilek servosu. Diz plakasinin arka kenari arka plaka yuzune (x=sx0-2)
    # kadar uzar ve dairesel ucuna 45 derece teget iner: arka plaka tablada desteksiz basilir.
    ank_b, ank_h = m._servo("ankle")
    sx0, sx1 = kn[0] - 10.0, kn[0] + 10.0
    rx = sx0 - 2.0                                  # arka plakanin arka yuzu
    c45 = math.cos(math.radians(45.0))
    tk = (kn[0] - 10.0 * c45, kn[2] + 10.0 * c45)   # 135 derecedeki teget noktasi
    knee_ext = prism_xz([(rx, -54.0), (sx0, -54.0), (sx0, kn[2]), tk, (rx, tk[1] - (tk[0] - rx))], 37.5, 40.5)
    shin = stadium_y(kn[0], kn[2], 10.0, -54.0, 37.5, 40.5)
    shin = shin.fuse([knee_ext, box(sx0, sx1, 35.5, 61.5, -57.0, -54.0),
                      box(rx, sx0, 35.5, 61.5, -70.0, -54.0)]).removeSplitter()
    shin = shin.cut(ycyl(1.4, kn[0], kn[2], 36.0, 42.0))
    shin = shin.cut(servos.horn_bolt_holes(sv["knee"], knee_h, 36.0, 42.0))
    pad = rbox(sx1, sx1 + 3.0, 36.5, 60.5, -76.0, -52.0, 0.0)
    pad = fillet_axis(pad, 4.0, (1, 0, 0))
    out += [("shin", "knee_horn", knee_h, "horn", m.horn_mass("knee")),
            ("shin", "shin_bracket", shin, "bracket", None),
            ("shin", "shin_pad", pad, "shell", None),
            ("shin", "ankle_servo", ank_b, "servo", m.servo_mass("ankle"))]

    # ayak: bilek horn'u, tek parca U braket, ayak govdesi ve taban (egik).
    # U braket: horn kolu (y 64.5..67) + ic yanak/koruma kolu (y 32..35, servo sirtindan 0.5 mm
    # acik; rulman yok: servo arkasinda mil tasiyacak yer yok, incikten yanakla mil vermek
    # kalca pitch +-90 / diz 70 taramasinda govde kabuguna ve roll servosuna carpiyor) +
    # servonun altinda 2.5 mm taban. Servo bilek ekseni etrafinda r=12.35 supurur (z>=-93.85):
    # taban ustu z=-95, ayak govdesindeki duz tabanli cebe (taban -97.5, link cercevesinde duz)
    # oturur; 4x M2 havsa basli vida ile govdedeki isil eklere baglanir (servo takilmadan once).
    z_sole = an[2] - p["sole_below_ankle"]
    z_ft = z_sole + p["sole_t"] + p["foot_t"]
    fx0, fx1 = an[0] + p["foot_x"][0], an[0] + p["foot_x"][1]
    fyc = 49.95
    fy0, fy1 = fyc - p["foot_w"] / 2, fyc + p["foot_w"] / 2
    r_u, ub0, ub1 = 11.0, -97.5, -95.0              # kol yaricapi, taban alt/ust
    ux0, ux1 = an[0] - r_u, an[0] + r_u
    yb0, yb1, yh0, yh1 = 32.0, 35.0, 64.5, 67.0     # ic kol, horn kolu
    u = stadium_y(an[0], an[2], r_u, ub1, yh0, yh1).fuse(
        [stadium_y(an[0], an[2], r_u, ub1, yb0, yb1), box(ux0, ux1, yb0, yh1, ub0, ub1)]).removeSplitter()
    u = fillet(u, 2.5, [e for e in u.Edges if _parallel(e, (1, 0, 0)) and abs(e.BoundBox.ZMin - ub1) < 1e-6
                        and min(abs(e.BoundBox.YMin - yb1), abs(e.BoundBox.YMin - yh0)) < 1e-6])
    u = u.cut(ycyl(13.35, an[0], an[2], yb1, 62.0))           # servo supurme zarfi + 1 mm
    u = u.cut(ycyl(1.4, an[0], an[2], 63.0, 68.0))
    u = u.cut(servos.horn_bolt_holes(sv["ankle"], ank_h, 63.5, 68.0))
    u_screws = [(an[0] + dx, sy) for dx in (-7.5, 7.5) for sy in (40.0, 57.0)]
    for sx, sy in u_screws:
        u = u.cut(zcyl(1.1, sx, sy, ub0 - 1.0, ub1 + 1.0))
        u = u.cut(Part.makeCone(1.1, 1.95 + 0.5, 0.85 + 0.5, V(sx, sy, ub1 - 0.85), V(0, 0, 1)))  # 90 derece havsa
    body = rbox(fx0, fx1, fy0, fy1, z_sole + p["sole_t"], z_ft, 9.0, 3.0, 0.0)
    for k, hx in enumerate((fx1 - 9.0, fx1 - 17.0)):
        body = body.cut(zcyl(2.2, hx, fyc + (6.0 if k else -6.0), z_ft - 2.0, z_ft + 1.0))
    sole = rbox(fx0, fx1, fy0, fy1, z_sole, z_sole + p["sole_t"], 9.0, 0.0, 1.5)
    c = (0.5 * (fx0 + fx1), fyc, z_sole)
    cant = p["sole_cant_deg"]
    body, sole = rotated(body, c, (1, 0, 0), cant), rotated(sole, c, (1, 0, 0), cant)
    body = body.cut(box(ux0 - 0.2, ux1 + 0.2, yb0 - 0.2, yh1 + 0.2, ub0, z_ft + 5.0))   # U braket cebi
    body = body.cut([zcyl(1.6, sx, sy, z_sole - 1.0, ub0 + 0.5) for sx, sy in u_screws])  # M2 isil ek
    ca = math.radians(cant)
    cc = V(c[0], c[1] - 0.5 * p["sole_t"] * math.sin(ca), z_sole + 0.5 * p["sole_t"] * math.cos(ca))
    cbox = (cc, ((fx1 - fx0) / 2, (fy1 - fy0) / 2, p["sole_t"] / 2), cant)
    out += [("foot", "ankle_horn", ank_h, "horn", m.horn_mass("ankle")),
            ("foot", "ankle_bracket", u, "bracket", None),
            ("foot", "foot_body", body, "trim", None),
            ("foot", "sole", sole, "sole", None, cbox)]
    return out


def build_xl330(m, sv):
    """27 cm XL330 bacaklari: eklemler (sol ID 20-24, sag 10-14) ve parcalar (refactor oncesi OpenDuck._legs)."""
    servos.require(m, sv, LEG_KEYS, "XL330", "legs.xl330_27", scale=1.0)
    p = m.p
    lim = m.limits
    axes = {"hip_yaw": (0, 0, -1), "hip_roll": (1, 0, 0), "hip_pitch": (0, 1, 0),
            "knee": (0, -1, 0), "ankle": (0, 1, 0)}
    chain = [("hip_yaw", "trunk", "hip"), ("hip_roll", "hip", "hip_cradle"),
             ("hip_pitch", "hip_cradle", "thigh"), ("knee", "thigh", "shin"), ("ankle", "shin", "foot")]
    items = _leg_items_xl330(m, sv)
    for side, sy, ids in (("left", 1, 20), ("right", -1, 10)):
        S = side + "_"
        for k, (jn, parent, child) in enumerate(chain):
            x, y, z = p["j_" + jn]
            ax = axes[jn]
            lo, hi = lim[jn]
            if sy < 0:   # sag bacak: Y'ye gore ayna; Y eksenli eklemlerin ekseni ters (Pollen ile ayni)
                ax = (ax[0], -ax[1], ax[2])
                lo, hi = -hi, -lo
            pa = parent if parent == "trunk" else S + parent
            m._joint(S + jn, pa, S + child, (x, sy * y, z), ax, (lo, hi), m.home.get(S + jn, 0.0), ids + k)
        for entry in items:
            link, name, shape, kind, mass = entry[:5]
            cbox = entry[5] if len(entry) > 5 else None
            if name == "hip_yaw_horn":
                shape = m.shared["hip_yaw_horn"][side]
            elif sy < 0:
                shape = mirror_y(shape)
            if cbox is not None and sy < 0:
                cbox = (V(cbox[0].x, -cbox[0].y, cbox[0].z), cbox[1], -cbox[2])
            m._add(S + link, S + name, shape, kind, mass, cbox)


# ---------------------------------------------------------------------------
# TR-35 (P2a): DS besik (stator) + U-link (rotor) + F623ZZ arka idler (M3 mil), hip_yaw doner tabla mili + pim/yarik
# kaplin (yalniz tork), ayak (U-link + plaka) + TPU taban, eklem gecisi kablo demetleri. Sol bacak sifir pozda govde
# cercevesinde kurulur; olculer servo zarfindan (servos.datums / ds_stator / ds_rotor_cuts) ve konfigden
# (params.legs) turetilir, baglanti elemanlari mutlak M3 (params.fit). Sag bacak = mirror_y (27 cm kurali).
# Agirlik: braket hacmi x params.print.bracket.density_eff (+ modellenmeyen vida/kol vidasi payi params.legs.mass_g).
# ---------------------------------------------------------------------------
LEG_AXES = {"hip_yaw": (0, 0, -1), "hip_roll": (1, 0, 0), "hip_pitch": (0, 1, 0), "knee": (0, -1, 0),
            "ankle": (0, 1, 0)}
LEG_CHAIN = (("hip_yaw", "trunk", "hip"), ("hip_roll", "hip", "hip_cradle"), ("hip_pitch", "hip_cradle", "thigh"),
             ("knee", "thigh", "shin"), ("ankle", "shin", "foot"))
# servo govdesi hangi link'te (spec 3.2.1 isaret kurali +1; p2-interface.md): hip_yaw trunk, hip_roll hip,
# hip_pitch thigh (cocuk), knee thigh, ankle shin. Horn ve idler pimi (rotor) diger link'te.
LEG_BODY_LINK = {"hip_roll": "hip", "hip_pitch": "thigh", "knee": "thigh", "ankle": "shin"}
LEG_HORN_LINK = {"hip_yaw": "hip", "hip_roll": "hip_cradle", "hip_pitch": "hip_cradle", "knee": "shin",
                 "ankle": "foot"}
DS_KEYS = ("hip_roll", "hip_pitch", "knee", "ankle")


def harness_items(m, key, parent, child):
    """Eklem gecisi kablo demeti (P2-0 sozlesmesi, geom.harness_paths): params.harness.crossings[key] -> parent
    link'te "<anahtar>_harness_in" (giris -> gecis noktasi) ve child link'te "<anahtar>_harness_out" (gecis noktasi
    -> cikis) tupleri (Item kind "harness", cap d). Doner [(link, ad, sekil, kutle_kg)] (sol taraf)."""
    from geom import harness_paths, tube
    out = []
    for part, pts, d, mass in harness_paths(m.p["harness"], key):
        out.append((parent if part == "in" else child, "%s_harness_%s" % (key, part), tube(pts, d / 2.0), mass))
    return out


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def _circ(c, r, n=28):
    """(u, v) duzleminde c merkezli r yaricapli cokgen noktalari (dis bukey zarf icin)."""
    return [(c[0] + r * math.cos(2.0 * math.pi * k / n), c[1] + r * math.sin(2.0 * math.pi * k / n)) for k in range(n)]


def _hull_xz(groups, y0, y1):
    """XZ'de nokta gruplarinin dis bukey zarfi, Y'de y0..y1."""
    return prism_xz(hull2d([q for g in groups for q in g]), y0, y1)


def prism_yz(pts, x0, x1):
    """YZ duzleminde (y, z) cokgen, X yonunde x0..x1 ekstrude."""
    poly = Part.makePolygon([V(x0, y, z) for y, z in pts] + [V(x0, pts[0][0], pts[0][1])])
    return Part.Face(poly).extrude(V(x1 - x0, 0, 0))


def _hull_yz(groups, x0, x1):
    return prism_yz(hull2d([q for g in groups for q in g]), x0, x1)


def _fuse(parts):
    s = parts[0]
    for q in parts[1:]:
        s = s.fuse(q)
    return s.removeSplitter()


def _cut(s, cuts):
    for c in cuts:
        s = s.cut(c)
    return s


def _sector_xz(c, r0, a0, a1, y0, y1, r1=400.0, step=10.0):
    """XZ'de c merkezli, r0..r1 yaricapli, a0..a1 (derece, +x'ten +z'ye) halka dilimi; Y'de y0..y1 (kesici)."""
    n = max(2, int(math.ceil(abs(a1 - a0) / step)) + 1)
    angs = [math.radians(a0 + (a1 - a0) * k / (n - 1.0)) for k in range(n)]
    outer = [(c[0] + r1 * math.cos(a), c[1] + r1 * math.sin(a)) for a in angs]
    inner = [(c[0] + r0 * math.cos(a), c[1] + r0 * math.sin(a)) for a in reversed(angs)]
    return prism_xz(outer + inner, y0, y1)


def _halfplanes_xz(c, dist, a0, a1, y0, y1, step=7.5, size=300.0):
    """XZ'de c'den normal yonu a (a0..a1, derece) ve uzakligi dist olan yari duzlemlerin birlesimi (kesici)."""
    n = max(2, int(math.ceil(abs(a1 - a0) / step)) + 1)
    out = []
    for kk in range(n):
        a = math.radians(a0 + (a1 - a0) * kk / (n - 1.0))
        u, v = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
        pts = [(c[0] + u[0] * s + v[0] * t, c[1] + u[1] * s + v[1] * t)
               for s, t in ((dist, -size), (dist + size, -size), (dist + size, size), (dist, size))]
        out.append(prism_xz(pts, y0, y1))
    return _fuse(out)


class _Frame(object):
    """Servo kanonik cercevesi (X genislik, Y mil -> horn, Z uzak uc) -> govde cercevesi (sol taraf, sifir poz).
    arm: horn kolunun konfig yonune (params.legs.horn_toward) donus acisi (kanonik Y etrafinda)."""

    def __init__(self, m, key, toward=None):
        self.key = key
        self.t = m.servo_type(key)
        self.o, self.h, self.f = m.servo_frame(key)
        self.g = _cross(self.h, self.f)
        self.pl = frame(self.o, self.h, self.f)
        self.d = servos.datums(self.t, m.p["idler"]["bearing"])
        self.toward = toward
        self.arm = servos.arm_angle_deg(self.h, self.f, toward) if toward is not None else 0.0

    def w(self, X, Y, Z):
        return tuple(self.o[i] + X * self.g[i] + Y * self.h[i] + Z * self.f[i] for i in range(3))

    def xz(self, X, Y, Z):
        q = self.w(X, Y, Z)
        return (q[0], q[2])

    def put(self, s, arm=False):
        if arm and abs(self.arm) > 1e-9:
            s = rotated(s, (0, 0, 0), (0, 1, 0), self.arm)
        return moved(s, self.pl)

    def arm_tip(self, r):
        """Kol ekseni uzerinde (horn yuzunde) eksenden r uzaktaki nokta (dunya)."""
        a = math.radians(self.arm)
        return self.w(r * math.sin(a), 0.0, r * math.cos(a))


def _routes(m, L, F):
    """Kablo demeti guzergahlari (sol taraf): donmus gecis (params.harness) + link ici guzergah
    (params.legs.harness_route[anahtar] = {"in": [...] gecis girisinden once, "out": [...] cikistan sonra, "gland":
    demetin bittigi servo (rakor ucuna kanonik +Z yonunde es eksenli, 0.2 mm aralik)}). Ayni link'teki iki gecis
    tupunun birlesimi (joins = [[A, B], ...]: A'nin cikisi -> B'nin girisi) duz bir parca uzerinde 0.2 mm aralikla es
    eksenli kesilir: ayni link ici cakisma olmaz, demet surekli. Doner {anahtar: {"in", "out": noktalar, "d": cap}}."""
    hc = m.p["harness"]["crossings"]
    rt = L["harness_route"]
    out = {}
    for key in ("hip_yaw", "hip_roll", "hip_pitch", "knee"):
        c, r = hc[key], rt.get(key, {})
        out[key] = {"d": float(c["d"]),
                    "in": [tuple(q) for q in r.get("in", [])] + [tuple(q) for q in c["in"]] + [tuple(c["cross"])],
                    "out": [tuple(c["cross"])] + [tuple(q) for q in c["out"]] + [tuple(q) for q in r.get("out", [])],
                    # donmus cikis parcasi (gecis + out noktalari) ve link ici uzantinin capi (kollar ayrildiktan sonra
                    # demet incelir: out_d; yoksa d)
                    "n_out": 1 + len(c["out"]), "out_d": float(r.get("out_d", c["d"]))}
        if r.get("gland"):
            fr = F[r["gland"]]
            cb = fr.t["cable"]
            gy = fr.d["rear_face"] + cb["h"]
            zt = fr.d["near_end_z"] - cb["gland_out"]
            out[key]["out"] += [fr.w(0, gy, zt - c["d"] / 2.0 - 0.4), fr.w(0, gy, zt - 0.2)]
    for a, b, mode in rt["joins"]:
        A, B = out[a], out[b]
        ra, rb = A["out_d"] / 2.0, B["d"] / 2.0
        P, Q = A["out"][-1], B["in"][0]
        if mode == "mid":                      # PQ duz parcasinin ortasinda kes
            u = _unit(tuple(Q[i] - P[i] for i in range(3)))
            Lpq = math.dist(P, Q)
            assert Lpq / 2.0 >= max(ra, rb) + 0.3, "kablo birlesimi %s->%s: PQ cok kisa (%.1f)" % (a, b, Lpq)
            M = tuple((P[i] + Q[i]) / 2.0 for i in range(3))
            A["out"].append(tuple(M[i] - 0.1 * u[i] for i in range(3)))
            B["in"].insert(0, tuple(M[i] + 0.1 * u[i] for i in range(3)))
        elif mode == "a":                      # A'nin ucunda: B, A'nin son parcasi dogrultusunda baslar
            u = _unit(tuple(P[i] - A["out"][-2][i] for i in range(3)))
            B["in"][0:0] = [tuple(P[i] + 0.2 * u[i] for i in range(3)), tuple(P[i] + (rb + 0.4) * u[i] for i in range(3))]
        else:                                  # B'nin basinda: A, B'nin ilk parcasi dogrultusunda yaklasir
            u = _unit(tuple(B["in"][1][i] - Q[i] for i in range(3)))
            A["out"] += [tuple(Q[i] - (ra + 0.4) * u[i] for i in range(3)), tuple(Q[i] - 0.2 * u[i] for i in range(3))]
    return out


def _tube_item(link, name, pts, d, gmm3, n0=None, d1=None):
    """Kablo demeti Item'i: pts boyunca cap d; n0 verilirse ilk n0 nokta d, kalani (uzanti) d1 capinda."""
    from geom import tube
    mass = lambda q, dd: sum(math.dist(a, b) for a, b in zip(q, q[1:])) * gmm3 * math.pi * dd * dd / 4.0 / 1000.0
    if n0 is None or d1 is None or abs(d1 - d) < 1e-9 or n0 >= len(pts):
        return (link, name, tube(pts, d / 2.0), "harness", mass(pts, d))
    a, b = pts[:n0], pts[n0 - 1:]
    return (link, name, tube(a, d / 2.0).fuse(tube(b, d1 / 2.0)).removeSplitter(), "harness",
            mass(a, d) + mass(b, d1))


def _tube_clear(pts, d, grow):
    from geom import tube
    return tube(pts, d / 2.0 + grow)


def _bracket_mass(m, shape, extra_g):
    return shape.Volume * m.p["print"]["bracket"]["density_eff"] + extra_g / 1000.0


# --------------------------------------------------------------------------- hip (yaw rotoru, roll statoru)
def _hip(m, F, L, fit, R):
    p, H, C = m.p, L["hip"], L["coupler"]
    cl = fit["clearance"]
    tt = servos.turntable_stack(p["turntable"])
    shaft_r = tt["bearing_rec"]["d"] / 2.0
    yx, yy = p["j_hip_yaw"][0], p["j_hip_yaw"][1]
    ztop = tt["z_mate"] - p["turntable"]["gap"]               # kalca flansi ustu (birlesme duzlemi - 0.5)
    r = F["hip_roll"]
    d, st = r.d, r.d["idler"]
    # roll servosu +y (dis) yanindan kayar: plaka ustte, ic ray (kulak duzlemi, iceride kalin blok) uzak rafi tasir,
    # arka duvar yalniz idler yuvasina iner. Servonun uzak ucu altinda/disinda parca yok: hip_cradle U-link'i roll +-20
    # boyunca orada doner, uyluk demeti (y 41..48) roll -20'de servo uzak kosesinin supurdugu bandi gorur.
    sh = servos.ds_stator(r.t, fit, L["shelf_t"], bearing=p["idler"]["bearing"], insert="X+", far="tap")
    x_tab = r.w(0, d["tab"][0], 0)[0]                         # kulak alt yuzu = raf ustu
    x_r0, x_r1 = sorted((r.w(0, st["seat"][0], 0)[0], r.w(0, st["seat"][1], 0)[0]))   # arka duvar (idler yuvasi)
    z_top = r.w(0, 0, d["tab_z"][0])[2]                       # yakin kulak ucu = plaka ustu
    z_pl = r.w(0, 0, d["near_end_z"])[2] + cl                 # servo yakin yuzu + bosluk = plaka alti
    z_far = r.w(0, 0, d["tab_z"][1])[2]                       # uzak raf alti (kilavuzlu: kulak ucunda)
    z_rw = r.w(0, 0, st["seat_r"] + H["rear_below"])[2]       # arka duvar alti (yuva halkasinin altinda)
    y0, y1 = r.w(-d["half_w"], 0, 0)[1], r.w(d["half_w"], 0, 0)[1]
    parts = [r.put(sh["near"]), r.put(sh["far"]), r.put(sh["seat"]),
             box(x_r0, x_tab, H["y_in"], y1, z_pl, z_top),                             # servo ustu plaka
             box(yx - H["conn"][0], x_tab, yy - H["conn"][1], H["y_in"] + 1.0, z_pl, z_top),   # flans -> plaka
             zcyl(H["r"], yx, yy, H["boss_bot"], ztop),                                   # flans + mil gobegi
             zcyl(H["shoulder_r"], yx, yy, ztop - 0.01, tt["z_mate"]),                  # alt ic bilezik omzu
             box(x_tab - H["rail_t"], x_tab, H["rail_y0"], y0 + 0.01, z_far, z_pl + 0.01),   # ic ray
             box(x_r0, x_r1, y0, y1, z_rw, z_pl + 0.01)]                                # arka duvar (yuva)
    hb = _fuse(parts)
    cuts = [r.put(c) for c in sh["cuts"]]
    cuts.append(zcyl(shaft_r + 0.02, yx, yy, H["shaft_bot"] - 0.01, tt["z_mate"] + 0.01))
    cuts += _hip_cross_cuts(m, H, fit, shaft_r)
    cuts.append(_tube_clear(R["hip_yaw"]["out"], R["hip_yaw"]["d"], L["harness_clear"]))
    cuts.append(_tube_clear(R["hip_roll"]["in"], R["hip_roll"]["d"], L["harness_clear"]))
    for nb in H["notches"]:                                    # plaka kenar centikleri [x0, x1, y0, y1]
        cuts.append(box(nb[0], nb[1], nb[2], nb[3], z_pl - 1.0, z_top + 1.0))
    # yaw kablo yarigi: govdedeki (sabit) giris tupu kalcaya gore yaw araligi (+ pay) boyunca supurur
    ya = H["yaw_slot_deg"]
    tin = _tube_clear(R["hip_yaw"]["in"], R["hip_yaw"]["d"], L["harness_clear"])
    cuts += [rotated(tin, (yx, yy, 0.0), (0, 0, 1), -ya + 2.0 * ya * k / 4.0) for k in range(5)]
    rw = H["rail_window"]                                      # ic ray: kulak duzlemine dik pencere (I kesit)
    cuts.append(box(x_tab - H["rail_t"] + rw[0], x_tab - rw[0], H["rail_y0"] - 1.0, y0 + 1.0,
                    z_far + rw[1], z_pl - rw[1]))
    hb = _cut(hb, cuts)
    items = [("hip", "hip_bracket", hb, "bracket", _bracket_mass(m, hb, L["mass_g"]["fasteners"]["hip"])),
             ("hip", "hip_roll_servo", r.put(servos.canon(r.t)[0]), "servo", m.servo_mass("hip_roll")),
             ("hip", "hip_roll_tab_screws", r.put(sh["screws"]), "metal", sh["screw_g"] / 1000.0)]
    ip = m.servo_parts("hip_roll")
    items.append(("hip", "hip_roll_idler_bearing", ip["idler_bearing"], "metal",
                  servos.BEARINGS[p["idler"]["bearing"]]["mass_g"] / 1000.0))
    # doner tabla mili (Al 6061 d10): alt ucu kalca gobeginde M3 capraz vida (tork + asagi eksenel yuk), ust ucu ust ic
    # bilezigin shaft_gap altinda; kaplin halkasi (PETG) bilezige basar, M3 havsa vida (mil ucunda kilavuz) + kama
    # (mil ucundaki testere yarigi) ile mile bagli. Yukari yuk: gobek omzu -> alt ic bilezik; asagi yuk: capraz vida ->
    # mil -> ust vida -> kaplin -> ust ic bilezik. ES3054 tek kolu kaplin yariginda: yalniz tork (eksenel/radyal serbest).
    sh_top = tt["upper"][1] - C["shaft_gap"]
    zp, kw = H["pin_z"], C["key_w"]
    shaft = zcyl(shaft_r, yx, yy, H["shaft_bot"], sh_top)
    shaft = _cut(shaft, [xcyl(H["pin_hole_d"] / 2.0, yy, zp, yx - shaft_r - 1.0, yx + shaft_r + 1.0),
                         zcyl(fit["tap_d"] / 2.0, yx, yy, sh_top - C["tap_depth"], sh_top + 1.0),
                         box(yx - shaft_r - 1.0, yx + shaft_r + 1.0, yy - kw / 2.0 - 0.05, yy + kw / 2.0 + 0.05,
                             sh_top - C["key_depth"], sh_top + 1.0)])
    items.append(("hip", "hip_yaw_shaft", shaft, "metal", shaft.Volume * servos.AL_G_MM3 / 1000.0))
    items.append(("hip", "hip_yaw_coupler", _coupler(m, C, tt, fit), "bracket", None))
    scr = _hip_screws(m, H, C, tt, fit)
    items.append(("hip", "hip_yaw_screws", scr, "metal", scr.Volume * servos.STEEL_G_MM3 / 1000.0))
    return items


def _hip_pin_x(m, H):
    """Capraz vida basinin oturma yuzu (x) ve ucu (x): gobek +x kenarindan spot yuzey, M3 x pin_len."""
    yx = m.p["j_hip_yaw"][0]
    xh = yx + H["r"] - H["pin_spot"]
    return xh, xh - H["pin_len"]


def _hip_cross_cuts(m, H, fit, shaft_r):
    """Kalca gobegi capraz vida kesicileri: bas spot yuzeyi (+x), bosluk deligi (+x duvari), kilavuz (-x duvari, kor)."""
    yx, yy = m.p["j_hip_yaw"][0], m.p["j_hip_yaw"][1]
    zp = H["pin_z"]
    xh, xt = _hip_pin_x(m, H)
    hr = servos.M3_HEAD["d"] / 2.0 + 0.3
    return [xcyl(hr, yy, zp, xh, xh + 8.0),
            xcyl(fit["clear_d"] / 2.0, yy, zp, yx + shaft_r - 0.5, xh + 0.01),
            xcyl(fit["tap_d"] / 2.0, yy, zp, xt - 0.2, yx - shaft_r + 0.5)]


def _hip_screws(m, H, C, tt, fit):
    """hip_yaw mil vidalari (celik, tek kati): ust M3 x 8 havsa bas (DIN 7991; bas kaplin halkasinda gomulu, mil
    ucundaki kilavuza) + alt M3 x pin_len capraz vida (bas +x, ucu -x gobek duvarinda kilavuz)."""
    yx, yy = m.p["j_hip_yaw"][0], m.p["j_hip_yaw"][1]
    z1 = tt["horn_face"] - C["top_gap"]                     # kaplin halkasi ustu
    kh = C["csk_h"]
    head = Part.makeCone(1.45, C["csk_d"] / 2.0 - 0.1, kh, V(yx, yy, z1 - kh), V(0, 0, 1))
    top = head.fuse(zcyl(1.2, yx, yy, z1 - C["screw_len"], z1 - kh + 0.01))
    xh, xt = _hip_pin_x(m, H)
    zp = H["pin_z"]
    cross = xcyl(servos.M3_HEAD["d"] / 2.0, yy, zp, xh, xh + servos.M3_HEAD["h"]).fuse(xcyl(1.2, yy, zp, xt, xh + 0.01))
    return top.fuse(cross).removeSplitter()


def _coupler(m, C, tt, fit):
    """hip_yaw pim/yarik kaplini (PETG): ust rulman ic bileziginde oturan halka + ES3054 tek kolunu kol yonunde
    yanlardan saran catal. Halka mile merkez M3 havsa vida (mil ucunda kilavuz) ve mil ucundaki yariga giren kama ile
    bagli (tork pozitif); kol yarikta radyal ve eksenel serbest (yalniz tork). Horn vidasi basi horn gobeginde gomulu
    (Adim 0'da dogrulanir). Hepsi doner tabla rotor bolgesinde (r <= 15, z <= 28.3 + halka)."""
    ft = _Frame(m, "hip_yaw", m.p["trunk"]["hip_yaw_horn_toward"])
    t = ft.t
    hn = t["horn"]
    cl = C["slot"]
    # kanonik (hip_yaw): horn dis yuzu Y = 0 (z = horn_face), +Y asagi; kol Y -arm_t..0; kaplin Y >= top_gap
    y_ring0 = C["top_gap"]
    y_ring1 = tt["horn_face"] - tt["upper"][1]                   # ust rulman ust yuzu (kanonik Y)
    ring = ycyl(C["r"], 0, 0, y_ring0, y_ring1 - C["seat_h"]).fuse(
        ycyl(C["seat_r"], 0, 0, y_ring1 - C["seat_h"] - 0.01, y_ring1))
    outer = servos._horn_outline(t, -hn["arm_t"] - C["top_up"], y_ring0 + 0.01, cl + C["wall"])
    lim = box(-50, 50, -10, 10, hn["hub_d"] / 2.0 + cl + 0.3, hn["hole_r"][-1])   # catal: gobek disi .. kol ucu
    tines = outer.common(lim)
    slot = servos._horn_outline(t, -hn["arm_t"] - C["top_up"] - 1.0, 0.0, cl)
    slot2 = slot.copy()
    slot2.translate(V(0, 0, C["float"]))                         # radyal serbestlik (kol yonunde)
    cp = ring.fuse(tines).removeSplitter()
    cp = ft.put(cp.cut(slot.fuse(slot2)), arm=True)
    # kama (dunya cercevesi, x yonunde): mil ucundaki yariga (genislik key_w + 0.1) iner; merkez vida deligi + havsa
    yx, yy = m.p["j_hip_yaw"][0], m.p["j_hip_yaw"][1]
    sh_top = tt["upper"][1] - C["shaft_gap"]
    rk = tt["bearing_rec"]["d"] / 2.0 - 0.3
    key = box(yx - rk, yx + rk, yy - C["key_w"] / 2.0, yy + C["key_w"] / 2.0, sh_top - C["key_depth"] + 0.1,
              tt["upper"][1] + 0.01)
    z1 = tt["horn_face"] - C["top_gap"]
    cp = cp.fuse(key).removeSplitter()
    cp = cp.cut(zcyl(fit["clear_d"] / 2.0, yx, yy, sh_top - C["key_depth"] - 1.0, z1 + 1.0))
    cp = cp.cut(Part.makeCone(fit["clear_d"] / 2.0, C["csk_d"] / 2.0 + 0.2, C["csk_h"] + 0.2,
                              V(yx, yy, z1 - C["csk_h"] - 0.2 + 0.01), V(0, 0, 1)))
    return cp


# --------------------------------------------------------------------------- hip_cradle (roll + pitch rotoru)
def _hip_cradle(m, F, L, fit, R):
    p, K = m.p, L["hip_cradle"]
    r, q = F["hip_roll"], F["hip_pitch"]
    ra = L["plate_a"]["hip_roll"]
    rc, qc = r.o, q.o                                    # roll ekseni (y, z) / pitch ekseni (x, z)
    x_a0, x_a1 = r.w(0, 0, 0)[0], r.w(0, ra, 0)[0]       # roll A kolu (horn yuzu disinda)
    stR = r.d["idler"]
    x_b0, x_b1 = sorted((r.w(0, stR["leg"][0], 0)[0], r.w(0, stR["leg"][1], 0)[0]))   # roll B kolu
    y_b0, y_b1 = sorted((q.w(0, q.d["idler"]["leg"][0], 0)[1], q.w(0, q.d["idler"]["leg"][1], 0)[1]))
    y_bm0 = q.w(0, L["plate_a"]["hip_pitch"], 0)[1]      # pitch A kolu dis yuzu (horn yuzu - plaka)
    y_bm1 = q.w(0, q.d["case_top"], 0)[1] - L["rotor_gap"]   # pitch servo ust yuzu - bosluk
    z0, z1 = rc[2] - K["boss_r"], rc[2] + K["boss_r"]    # on plaka yuksekligi = roll gobegi
    tipR = r.arm_tip(r.t["horn"]["hole_r"][-1])
    # roll U-link: A kolu (horn) ve B kolu (idler) roll servosunun uzak ucunun altinda alt kopruyle birlesir (hip
    # besiginin roll +-20 supurmesinin altinda; kopru yalniz eksenin dis tarafinda: roll -20'de ic kose orta
    # duzleme yaklasmaz). Pitch ekseninin arkasinda, uyluk demeti bandinda (y 41..48) parca yok.
    ub1 = rc[2] - K["u_depth"]
    ub0 = ub1 - K["u_t"]                                 # kopru derinligi: idler payi (dikey yukun %58'i) U'dan gecer
    uy0, uy1 = rc[1] + K["u_y"][0], rc[1] + K["u_y"][1]
    ubot = [(uy0, ub0), (uy1, ub0), (uy1, ub1), (uy0, ub1)]
    lega = _hull_yz([_circ((rc[1], rc[2]), K["boss_r"]), ubot], x_a0, x_a1)
    lega = lega.fuse(_hull_yz([_circ((rc[1], rc[2]), K["boss_r"]), _circ((tipR[1], tipR[2]), K["tip_r"])], x_a0, x_a1))
    front = lega.fuse(box(x_a0, x_a1, K["front_y0"], y_b1, z0, z1))                 # on plaka (pitch U koprusu)
    legb = _hull_yz([_circ((rc[1], rc[2]), K["legb_r"]), ubot], x_b0, x_b1)
    ut = K["u_thick"]                                    # U kollarinin alt yarisi kalin (kose egilmesi)
    zt0 = rc[2] - K["boss_r"] - ut
    ubr = _fuse([box(x_b0 - ut, x_a1 + ut, uy0, uy1, ub0, ub1),
                 box(x_a1 - 0.01, x_a1 + ut, uy0, uy1, ub0, zt0), box(x_b0 - ut, x_b0 + 0.01, uy0, uy1, ub0, zt0)])
    fl, wb = K["u_flange"], K["u_web"] / 2.0             # kopru I kesit: ust/alt flans + orta govde
    ym = (uy0 + uy1) / 2.0
    for ya, yb in ((uy0 - 1.0, ym - wb), (ym + wb, uy1 + 1.0)):
        ubr = ubr.cut(box(x_b0 + ut, x_a1 - ut, ya, yb, ub0 + fl, ub1 - fl))
    ring = r.put(servos._horn_outline(r.t, -r.t["horn"]["arm_t"], 0.0, fit["clearance"] + L["ring_w"]), arm=True)
    # pitch A kolu (y_bm0..y_bm1, XZ): pitch gobegi -> on plaka; pitch B kolu (XZ): idler -> on plaka
    bz0, bz1 = qc[2] - K["beam_hz"], qc[2] + K["beam_hz"]
    beam = _hull_xz([_circ((qc[0], qc[2]), K["boss_r"]), [(x_a1, bz0), (x_a1, bz1)]], y_bm0, y_bm1)
    legbp = _hull_xz([_circ((qc[0], qc[2]), K["legb_r"]),
                      [(x_a0 - K["legbp_w"], z0), (x_a1, z0), (x_a1, z1), (x_a0 - K["legbp_w"], z1)]], y_b0, y_b1)
    sleeves = [fr.put(servos.ds_thrust_sleeve(fr.t, fit)) for fr in (r, q)]   # eksenel yuk -> servo govdesi
    hc = _fuse([front, ring, beam, legb, ubr, legbp] + sleeves)
    cuts = [r.put(c, arm=True) for c in servos.ds_rotor_cuts(r.t, fit, ra, p["idler"]["bearing"])]
    cuts += [q.put(c, arm=True) for c in servos.ds_rotor_cuts(q.t, fit, L["plate_a"]["hip_pitch"],
                                                              p["idler"]["bearing"])]
    cuts.append(_tube_clear(R["hip_roll"]["out"], R["hip_roll"]["d"], L["harness_clear"]))
    cuts.append(_tube_clear(R["hip_pitch"]["in"], R["hip_pitch"]["d"], L["harness_clear"]))
    ch = K["chamfer"]                                     # on plaka alt dis kenari pahi (incik pitch -70/diz 65)
    cuts.append(prism_xz([(x_a1 + 1.0, z0 - 1.0), (x_a1 - ch - 1.0, z0 - 1.0), (x_a1 + 1.0, z0 + ch + 1.0)],
                         K["front_y0"], y_b1 + 1.0))
    # hafifletme: on plakada pitch A ve B kollari arasinda n pencere; roll U kollarinda birer pencere
    wn, wr, ww = K["window"]["n"], K["window"]["rib"], K["window"]["wall"]
    wy0, wy1 = y_bm1 + wr, y_b0 - wr
    wl = (wy1 - wy0 - (wn - 1) * wr) / wn
    for i in range(wn):
        cuts.append(box(x_a0 - 1.0, x_a1 + 1.0, wy0 + i * (wl + wr), wy0 + i * (wl + wr) + wl, z0 + ww, z1 - ww))
    hc = _cut(hc, cuts)
    items = [("hip_cradle", "hip_cradle_bracket", hc, "bracket",
              _bracket_mass(m, hc, L["mass_g"]["fasteners"]["hip_cradle"]))]
    for key, fr in (("hip_roll", r), ("hip_pitch", q)):
        sp = m.servo_parts(key, horn_toward=L["horn_toward"][key])
        scr, g = servos.ds_idler_screw(fr.t, p["idler"]["bearing"])
        items += [("hip_cradle", key + "_horn", sp["horn"], "horn", m.horn_mass(key)),
                  ("hip_cradle", key + "_idler_pin", fr.put(scr, arm=True), "metal", g / 1000.0)]
    return items


# --------------------------------------------------------------------------- thigh (pitch + diz statoru)
def _ds_frame(fr, fit, L):
    """Kulak duzlemi cercevesi (kanonik): raflar + iki yan cubuk (X +-, raf kalinliginda)."""
    d = fr.d
    sh = servos.ds_stator(fr.t, fit, L["shelf_t"], bearing=None)
    yt0 = d["tab"][0]
    w = d["half_w"] + fit["clearance"]
    z0, z1 = d["tab_z"][0], d["tab_z"][1] + servos.DS_FAR_EXT
    bars = [box(s0, s1, yt0 - L["shelf_t"], yt0, z0, z1) for s0, s1 in ((w, w + L["bar_w"]), (-w - L["bar_w"], -w))]
    # raf <-> yan cubuk baglantisi (raflar govde genisliginde, cubuklar govde + bosluk disinda): yalniz govde boyu
    # disindaki raf Z araliklarinda (govde kesicisine girmez)
    near, far = fr.t["shaft"]["near"], fr.t["body"]["l"] - fr.t["shaft"]["near"]
    cl = fit["clearance"]
    for za, zb in ((z0, -near - cl), (far + cl, z1)):
        bars += [box(d["half_w"] - 0.01, w + 0.01, yt0 - L["shelf_t"], yt0, za, zb),
                 box(-w - 0.01, -d["half_w"] + 0.01, yt0 - L["shelf_t"], yt0, za, zb)]
    return sh, _fuse([sh["near"], sh["far"]] + bars)


def _ds_post(fr, fit, L, side, za, zb):
    """Kulak duzlemi (raf alti) -> idler taban plakasi direk (kanonik X tarafi side = +-1, Z za..zb)."""
    d = fr.d
    st = d["idler"]
    w = d["half_w"] + fit["clearance"]
    x0, x1 = (w, w + L["bar_w"]) if side > 0 else (-w - L["bar_w"], -w)
    return box(x0, x1, st["seat"][0], d["tab"][0] - L["shelf_t"] + 0.01, za, zb)


def _thigh(m, F, L, fit, R):
    p, T = m.p, L["thigh"]
    q, k = F["hip_pitch"], F["knee"]
    shq, frq = _ds_frame(q, fit, L)
    shk, frk = _ds_frame(k, fit, L)
    parts = [q.put(frq), k.put(frk), q.put(shq["seat"]), k.put(shk["seat"])]
    st = q.d["idler"]
    ys0, ys1 = sorted((q.w(0, st["seat"][0], 0)[1], q.w(0, st["seat"][1], 0)[1]))
    qa, ka = (q.o[0], q.o[2]), (k.o[0], k.o[2])
    parts.append(_hull_xz([_circ(qa, T["strut_r"]), _circ(ka, T["strut_r"])], ys0, ys1))    # taban dikmesi
    for key, side, za, zb in T["posts"]:
        fr = q if key == "hip_pitch" else k
        post = fr.put(_ds_post(fr, fit, L, side, za, zb))
        parts.append(post)
        bb = post.BoundBox
        c = ((bb.XMin + bb.XMax) / 2.0, (bb.ZMin + bb.ZMax) / 2.0)
        parts.append(_hull_xz([_circ(qa if key == "hip_pitch" else ka, T["strut_r"]), _circ(c, T["strut_r"])],
                              ys0, ys1))
    th = _fuse(parts)
    cuts = [q.put(c) for c in shq["cuts"]] + [k.put(c) for c in shk["cuts"]]
    # incik supurmesi: diz ekseni etrafinda r >= keep_r, aci araligi (diz cevresi, govde Y bandinda)
    kr = T["shin_keep"]
    cuts.append(_sector_xz(ka, kr["r"], kr["a"][0], kr["a"][1], kr["y"][0], kr["y"][1]))
    cuts.append(_tube_clear(R["hip_pitch"]["out"], R["hip_pitch"]["d"], L["harness_clear"]))
    cuts.append(_tube_clear(R["knee"]["in"], R["knee"]["d"], L["harness_clear"]))
    th = _cut(th, cuts)
    items = [("thigh", "thigh_bracket", th, "bracket", _bracket_mass(m, th, L["mass_g"]["fasteners"]["thigh"]))]
    for key, fr, sh in (("hip_pitch", q, shq), ("knee", k, shk)):
        ip = m.servo_parts(key)
        items += [("thigh", key + "_servo", fr.put(servos.canon(fr.t)[0]), "servo", m.servo_mass(key)),
                  ("thigh", key + "_tab_screws", fr.put(sh["screws"]), "metal", sh["screw_g"] / 1000.0),
                  ("thigh", key + "_idler_bearing", ip["idler_bearing"], "metal",
                   servos.BEARINGS[p["idler"]["bearing"]]["mass_g"] / 1000.0)]
    return items


# --------------------------------------------------------------------------- shin (diz rotoru, bilek statoru)
def _shin(m, F, L, fit, R):
    p, S = m.p, L["shin"]
    k, a = F["knee"], F["ankle"]
    ka, aa = (k.o[0], k.o[2]), (a.o[0], a.o[2])
    tipK = k.arm_tip(k.t["horn"]["hole_r"][-1])
    ya0 = k.w(0, L["plate_a"]["knee"], 0)[1]                  # diz A kolu dis yuzu
    ya1 = k.w(0, k.d["case_top"], 0)[1] - L["rotor_gap"]
    stK = k.d["idler"]
    yb0, yb1 = sorted((k.w(0, stK["leg"][0], 0)[1], k.w(0, stK["leg"][1], 0)[1]))
    sha, fra = _ds_frame(a, fit, L)
    # A kolu (XZ): diz gobegi + kol ucu + bilek yuvasina inen omurga
    lega = _hull_xz([_circ(ka, S["boss_r"]), _circ((tipK[0], tipK[2]), S["tip_r"])], ya0, ya1)
    spine = _hull_xz([_circ(ka, S["spine_r"][0]), _circ(aa, S["spine_r"][1])], ya0 + S["spine_dy"], ya1)
    # bilek besigi: kulak cercevesi + yuva + X- yan duvar (yuva -> kulak duzlemi; yuva duzleminde yuva halkasina
    # perde) + U-link koprusu: omurgadan diz B koluna uzanan direk (bilek servosunun X+ yaninda, diz kablosunun
    # disinda; kulak rafi yan cubuguna baglanti). Diz U-link yuk yolu: A kolu -> omurga -> direk -> B kolu.
    dw = a.d
    wst = dw["idler"]
    ww = dw["half_w"] + fit["clearance"]
    wall = a.put(_ds_post(a, fit, L, -1, S["wall_z"][0], S["wall_z"][1]).fuse(
        box(-ww - L["bar_w"], -wst["seat_r"] + 1.0, wst["seat"][0], wst["seat"][1], S["web_z"][0], S["web_z"][1])))
    pa = S["post"]                                            # kanonik [X, Z, r] (bilek cercevesi)
    pc = a.xz(pa[0], 0, pa[1])
    y_sp0 = ya0 + S["spine_dy"]
    post = ycyl(pa[2], pc[0], pc[1], y_sp0, yb0 + 0.01)
    post = post.fuse(_hull_xz([_circ(pc, pa[2]), _circ((aa[0], pc[1]), pa[2])], y_sp0, ya1))   # omurgaya kok
    post = post.fuse(a.put(box(ww + L["bar_w"] - 0.01, pa[0], dw["tab"][0] - L["shelf_t"], dw["tab"][0],
                               pa[1] - pa[2] + 1.0, pa[1] + pa[2] - 1.0)))                   # raf cubuguna
    legb = _hull_xz([_circ(ka, S["legb_r"]), _circ(pc, pa[2])], yb0, yb1)
    # B kolu dis yuz nervuru (diz idler M3 basinin disinda baslar): direk -> idler kolu egilme kesiti (el hesabi)
    rt, r0, rr = S["legb_rib"]
    u = _unit((pc[0] - ka[0], pc[1] - ka[1], 0.0))
    rib = _hull_xz([_circ((ka[0] + u[0] * r0, ka[1] + u[1] * r0), rr), _circ(pc, pa[2])], yb1 - 0.01, yb1 + rt)
    parts = [lega, spine, a.put(fra), a.put(sha["seat"]), wall, post, legb, rib,
             k.put(servos.ds_thrust_sleeve(k.t, fit))]
    sn = _fuse(parts)
    cuts = [k.put(c, arm=True) for c in servos.ds_rotor_cuts(k.t, fit, L["plate_a"]["knee"], p["idler"]["bearing"])]
    cuts += [a.put(c) for c in sha["cuts"]]
    # ayak supurmesi: bilek ekseni etrafinda ayak plakasinin (ust yuzu, eksenden ~22 mm) bilek araligi boyunca
    # tarayacagi yari duzlemler (normal acisi a0..a1) -> incik orada fk.r'den yakin kalir
    fk = S["foot_keep"]
    cuts.append(_halfplanes_xz(aa, fk["r"], fk["a"][0], fk["a"][1], fk["y"][0], fk["y"][1]))
    lb = S["legb_keep"]                                       # ayak B kolu supurmesi (bilek ic tarafi, Y bandi)
    cuts.append(ycyl(lb["r"], aa[0], aa[1], lb["y"][0], lb["y"][1]))
    cuts.append(_sector_xz(aa, 0.0, lb["a"][0], lb["a"][1], lb["y"][0], lb["y"][1]))
    # diz cevresi: uyluk diz besigi bolgesi (govde Y bandinda r < keep)
    tk = S["thigh_keep"]
    cuts.append(ycyl(tk["r"], ka[0], ka[1], tk["y"][0], tk["y"][1]))
    cuts.append(_tube_clear(R["knee"]["out"], R["knee"]["d"], L["harness_clear"]))
    cuts.append(ycyl(S["post_bore"], pc[0], pc[1], ya1 + 1.0, yb0 - 2.0))     # direk ici bos (boru, kutle)
    sn = _cut(sn, cuts)
    items = [("shin", "shin_bracket", sn, "bracket", _bracket_mass(m, sn, L["mass_g"]["fasteners"]["shin"]))]
    sp = m.servo_parts("knee", horn_toward=L["horn_toward"]["knee"])
    scr, g = servos.ds_idler_screw(k.t, p["idler"]["bearing"])
    ip = m.servo_parts("ankle")
    items += [("shin", "knee_horn", sp["horn"], "horn", m.horn_mass("knee")),
              ("shin", "knee_idler_pin", k.put(scr, arm=True), "metal", g / 1000.0),
              ("shin", "ankle_servo", a.put(servos.canon(a.t)[0]), "servo", m.servo_mass("ankle")),
              ("shin", "ankle_tab_screws", a.put(sha["screws"]), "metal", sha["screw_g"] / 1000.0),
              ("shin", "ankle_idler_bearing", ip["idler_bearing"], "metal",
               servos.BEARINGS[p["idler"]["bearing"]]["mass_g"] / 1000.0)]
    return items


# --------------------------------------------------------------------------- foot (bilek rotoru + taban)
def _foot(m, F, L, fit):
    p, FT = m.p, L["foot"]
    a = F["ankle"]
    aa = (a.o[0], a.o[2])
    an = p["j_ankle"]
    z_sole = an[2] - p["sole_below_ankle"]
    z_pl0 = z_sole + p["sole_t"]
    z_pl1 = z_pl0 + FT["plate_t"]
    fx0, fx1 = an[0] + p["foot_x"][0], an[0] + p["foot_x"][1]
    fy0, fy1 = FT["yc"] - p["foot_w"] / 2.0, FT["yc"] + p["foot_w"] / 2.0
    plate = rbox(fx0, fx1, fy0, fy1, z_pl0, z_pl1, FT["corner_r"])
    sole = rbox(fx0, fx1, fy0, fy1, z_sole, z_pl0, FT["corner_r"], 0.0, 1.5)
    c = (0.5 * (fx0 + fx1), FT["yc"], z_sole)
    cant = p["sole_cant_deg"]
    plate, sole = rotated(plate, c, (1, 0, 0), cant), rotated(sole, c, (1, 0, 0), cant)
    st = a.d["idler"]
    ya0, ya1 = sorted((a.w(0, 0, 0)[1], a.w(0, L["plate_a"]["ankle"], 0)[1]))      # A kolu (horn yuzu disinda)
    yb0, yb1 = sorted((a.w(0, st["leg"][0], 0)[1], a.w(0, st["leg"][1], 0)[1]))    # B kolu
    tip = a.arm_tip(a.t["horn"]["hole_r"][-1])
    ca = math.radians(cant)
    top = lambda y: z_sole + (y - FT["yc"]) * math.sin(ca) + (z_pl1 - z_sole) * math.cos(ca)   # egik plaka ustu
    zba = top(ya0) - FT["leg_dip"]                             # kollar plakaya gomulur (alt yuzune ulasmaz)
    zbb = top(yb0) - FT["leg_dip"]
    lega = _hull_xz([_circ(aa, FT["boss_r"]), _circ((tip[0], tip[2]), FT["tip_r"]),
                     [(aa[0] + FT["lega_x"][0], zba), (aa[0] + FT["lega_x"][1], zba)]], ya0, ya1)
    ring = a.put(servos._horn_outline(a.t, -a.t["horn"]["arm_t"], 0.0, fit["clearance"] + L["ring_w"]), arm=True)
    legb = _hull_xz([_circ(aa, FT["legb_r"]), [(aa[0] + FT["legb_x"][0], zbb), (aa[0] + FT["legb_x"][1], zbb)]],
                    yb0, yb1)
    fb = _fuse([plate, lega, ring, legb, a.put(servos.ds_thrust_sleeve(a.t, fit))])
    cuts = [a.put(c, arm=True) for c in servos.ds_rotor_cuts(a.t, fit, L["plate_a"]["ankle"], p["idler"]["bearing"])]
    # plaka ustu hafifletme cepleri (taban tarafi tam, ust 'floor' kalir), kollar arasinda; egimle birlikte
    pk = FT["pockets"]
    nx, ny = pk["n"]
    px0, px1 = fx0 + pk["rim"], fx1 - pk["rim"]
    py0, py1 = yb1 + pk["rim"], ya0 - L["ring_w"] - pk["rim"] - a.t["horn"]["arm_t"]
    wx = (px1 - px0 - (nx - 1) * pk["rib"]) / nx
    wy = (py1 - py0 - (ny - 1) * pk["rib"]) / ny
    for i in range(nx):
        for j in range(ny):
            x0 = px0 + i * (wx + pk["rib"])
            y0 = py0 + j * (wy + pk["rib"])
            cuts.append(rotated(box(x0, x0 + wx, y0, y0 + wy, z_pl0 + pk["floor"], z_pl1 + 1.0), c, (1, 0, 0), cant))
    fb = _cut(fb, cuts)
    cc = V(c[0], c[1] - 0.5 * p["sole_t"] * math.sin(ca), z_sole + 0.5 * p["sole_t"] * math.cos(ca))
    sp = m.servo_parts("ankle", horn_toward=L["horn_toward"]["ankle"])
    scr, g = servos.ds_idler_screw(a.t, p["idler"]["bearing"])
    return [("foot", "foot_bracket", fb, "bracket", _bracket_mass(m, fb, L["mass_g"]["fasteners"]["foot"])),
            ("foot", "ankle_horn", sp["horn"], "horn", m.horn_mass("ankle")),
            ("foot", "ankle_idler_pin", a.put(scr, arm=True), "metal", g / 1000.0),
            ("foot", "sole", sole, "sole", None, (cc, ((fx1 - fx0) / 2, (fy1 - fy0) / 2, p["sole_t"] / 2), cant))]


def _harness_items(m, L, R):
    """Kablo demeti Item'lari: her gecis icin <anahtar>_harness_in (parent) ve _out (child) (link ici guzergah
    dahil, sol taraf)."""
    gmm3 = m.p["harness"]["g_per_mm3"]
    out = []
    for key, parent, child in LEG_CHAIN:
        if key not in R:          # bilek: gecis yok (bilek servosu incikte, kablosu diz gecisinde)
            continue
        hr = R[key]
        out.append(_tube_item(parent, key + "_harness_in", hr["in"], hr["d"], gmm3))
        out.append(_tube_item(child, key + "_harness_out", hr["out"], hr["d"], gmm3, hr["n_out"], hr["out_d"]))
    return out


def build_tr35(m, sv):
    """TR-35 bacaklari (P2a): eklemler (sol ID 20-24, sag 10-14; eksen/isaret 27 cm ile ayni) ve parcalar:
    hip (doner tabla mili + kaplin + roll besigi), hip_cradle (roll/pitch U-link), thigh (pitch + diz besigi), shin
    (diz U-link + bilek besigi), foot (bilek U-link + plaka + TPU taban), kablo demetleri."""
    servos.require(m, sv, DS_KEYS, "DS32XX", "legs.tr35_b2")
    servos.require(m, sv, ("hip_yaw",), "ES3054", "legs.tr35_b2")
    p, L = m.p, m.p["legs"]
    fit = p["fit"]
    lim = m.limits
    F = {key: _Frame(m, key, L["horn_toward"].get(key)) for key in DS_KEYS}
    R = _routes(m, L, F)
    items = []
    items += _hip(m, F, L, fit, R)
    items += _hip_cradle(m, F, L, fit, R)
    items += _thigh(m, F, L, fit, R)
    items += _shin(m, F, L, fit, R)
    items += _foot(m, F, L, fit)
    for link, name, shape, kind, mass in _harness_items(m, L, R):
        items.append((link, name, shape, kind, mass))

    for side, sy, ids in (("left", 1, 20), ("right", -1, 10)):
        S = side + "_"
        for k, (jn, parent, child) in enumerate(LEG_CHAIN):
            x, y, z = p["j_" + jn]
            ax = LEG_AXES[jn]
            lo, hi = lim[jn]
            if sy < 0:   # sag bacak: Y'ye gore ayna; Y eksenli eklemlerin ekseni ters (27 cm ile ayni)
                ax = (ax[0], -ax[1], ax[2])
                lo, hi = -hi, -lo
            pa = parent if parent == "trunk" else S + parent
            m._joint(S + jn, pa, S + child, (x, sy * y, z), ax, (lo, hi), m.home.get(S + jn, 0.0), ids + k)
        m._add(S + "hip", S + "hip_yaw_horn", m.shared["hip_yaw_horn"][side], "horn", m.horn_mass("hip_yaw"))
        for entry in items:
            link, name, shape, kind, mass = entry[:5]
            cbox = entry[5] if len(entry) > 5 else None
            lk = link if link == "trunk" else S + link
            if sy < 0:
                shape = mirror_y(shape)
                if cbox is not None:
                    cbox = (V(cbox[0].x, -cbox[0].y, cbox[0].z), cbox[1], -cbox[2])
            m._add(lk, S + name, shape, kind, mass, cbox)


GENERATORS = {"xl330_27": build_xl330, "tr35_b2": build_tr35}
