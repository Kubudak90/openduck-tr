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
from geom import (V, _parallel, box, fillet, fillet_axis, hull2d, mirror_y, prism_xz, rbox, rotated, stadium_y,
                  xcyl, ycyl, zcyl)

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


GENERATORS = {"xl330_27": build_xl330}
