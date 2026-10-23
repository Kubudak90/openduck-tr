# -*- coding: utf-8 -*-
"""
Govde (trunk) geometri uretecleri: sasi, batarya yuvasi, kabuk, govde elektronigi, kalca yaw servolari.

Uretec arayuzu (openduck.OpenDuck._build): GENERATORS[konfig "generators.trunk"](m, sv); m model (m.p parametreler,
m._add / m._joint kayit, m._servo(anahtar, ayna) yerlestirilmis (govde, horn), m.servo_frame, m.servo_mass /
m.horn_mass, m.masses, m.limits, m.home, m.meta, m.shared uretecler arasi sekiller), sv eklem anahtari -> servo
turu kaydi (servos.SERVO_TYPES). Servo govdesi parcasinin adi "[left_|right_]<anahtar>_servo" olmali
(openduck.servo_signs isaret kontrolu).
"""
import math

import servos
from geom import box, prism_xz, rbox, shell_rbox, xcyl, ycyl, zcyl


def build_xl330(m, sv):
    """27 cm XL330 govdesi (refactor oncesi OpenDuck._trunk, birebir)."""
    servos.require(m, sv, ("hip_yaw", "neck_pitch"), "XL330", "trunk.xl330_27", scale=1.0)
    p = m.p
    t, w = p["plate_t"], p["wall"]
    tx0, tx1 = p["torso_x"]
    tw2 = p["torso_w"] / 2
    tz0, tz1 = p["torso_z"]
    nx, _, nz = p["j_neck_pitch"]

    # sasi: taban plakasi + kalca yaw servo kafesleri + boyun catali. Batarya yuvasi ayri parca
    # (battery_tray): tek parcada taban plakasinin altina sarkiyor ve ~85 cm3 destek istiyordu.
    # Sasi boylece taban plakasi tablada (link -Z) desteksiz basilir.
    base = rbox(-25.0, 17.5, -30.0, 30.0, -1.0, 1.5, 4.0)
    for sy in (1, -1):
        base = base.cut(box(-18.7, 15.7, sy * 7.3 if sy > 0 else -27.7, 27.7 if sy > 0 else -7.3, 0.0, 2.0))
        base = base.cut(zcyl(7.5, 6.0, sy * 17.5, -2.0, 2.0))
    parts = []
    for sy in (1, -1):
        for y0, y1 in ((5.5, 7.5), (27.5, 29.5)):
            a, b = (y0, y1) if sy > 0 else (-y1, -y0)
            parts.append(box(-18.5, 15.5, a, b, 1.5, 20.0))
    # boyun catali: horn diskinden tabana inen dis bukey profil (on kenari ~74 derece, sarkma yok;
    # kabugun on duvarindan x<=31.8'de kalir) + taban plakasinin onune catal ayagi dili
    fr, (fpx, fpz) = 9.5, (27.0, 1.5)
    fd = math.hypot(nx - fpx, nz - fpz)
    fa = math.atan2(nz - fpz, nx - fpx) - math.asin(fr / fd)              # on teget dogrultusu
    ft = (fpx + math.cos(fa) * math.sqrt(fd * fd - fr * fr), fpz + math.sin(fa) * math.sqrt(fd * fd - fr * fr))
    fork_xz = [(nx - fr, fpz), (fpx, fpz), ft, (nx, nz), (nx - fr, nz)]
    for a, b in ((14.5, 17.0), (-17.0, -14.5)):
        parts.append(prism_xz(fork_xz, a, b).fuse(ycyl(fr, nx, nz, a, b)))
        parts.append(rbox(16.0, 27.0, a - 1.0, b + 1.0, -1.0, 1.5, 1.0))
    parts.append(box(-25.0, -22.5, -21.0, 21.0, -1.0, 30.0))       # arka duvar (taban ustu)
    chassis = base.fuse(parts).removeSplitter()
    chassis = chassis.cut(ycyl(1.6, nx, nz, -18.0, 18.0))
    chassis = chassis.cut(servos.horn_bolt_holes(sv["neck_pitch"], m._servo("neck_pitch")[1], 14.0, 17.5))
    # batarya yuvasi vidalari: 3x M2, ustten taban plakasindan yuvanin flansindaki isil eklere
    tray_screws = [(-21.5, 24.25), (-21.5, -24.25), (-19.5, 0.0)]
    chassis = chassis.cut([zcyl(1.1, sx, sy, -2.0, 2.0) for sx, sy in tray_screws])
    m._add("trunk", "chassis", chassis, "bracket")

    # batarya yuvasi: arka duvarin taban alti kismi + taban + yan dudaklar + taban plakasina
    # vidalanan flans (alti 50 derece pahli: taban tablada desteksiz basilir)
    bx0 = tx0 - 0.4
    tray = [box(-25.0, -22.5, -21.0, 21.0, -42.0, -1.0),
            box(bx0 - 1.5, -22.5, -21.0, 21.0, -42.0, -39.6),
            prism_xz([(-25.0, -1.0), (-18.5, -1.0), (-18.5, -5.0), (-25.0, -12.75)], -27.5, 27.5),
            prism_xz([(-22.5, -1.0), (-16.0, -1.0), (-16.0, -5.0), (-22.5, -12.75)], -4.0, 4.0)]   # orta vida dili
    for a, b in ((-21.0, -19.5), (19.5, 21.0)):
        tray.append(box(bx0 - 1.5, -22.5, a, b, -42.0, -30.0))
    tray = tray[0].fuse(tray[1:]).removeSplitter()
    tray = tray.cut([zcyl(1.6, sx, sy, -5.5, 0.0) for sx, sy in tray_screws])
    m._add("trunk", "battery_tray", tray, "bracket")

    # kabuk: iki yarim (sol/sag), alt acik, arkada batarya yuvasi, onde boyun acikligi
    shell = shell_rbox(tx0, tx1, -tw2, tw2, tz0, tz1, w, 10.0, 9.0, 2.0)
    shell = shell.cut(box(tx0 + w, tx1 - w, -tw2 + w, tw2 - w, tz0 - 1, tz0 + w + 0.01))
    shell = shell.cut(box(tx0 - 2, -24.5, -20.0, 20.0, tz0 - 1, 33.0))
    shell = shell.cut(rbox(2.0, tx1 + 2, -18.0, 18.0, 18.0, tz1 + 2, 4.0))
    shell = shell.cut(xcyl(3.0, 22.0, 24.0, tx0 - 1, tx0 + w + 1))        # USB-C / anahtar
    big = (tx0 - 1, tx1 + 1)
    m._add("trunk", "torso_shell_left", shell.common(box(big[0], big[1], 0.0, tw2 + 1, tz0 - 1, tz1 + 1)), "shell")
    m._add("trunk", "torso_shell_right", shell.common(box(big[0], big[1], -tw2 - 1, 0.0, tz0 - 1, tz1 + 1)), "shell")

    # batarya, IMU, guc karti
    bx, by, bz = p["battery"]
    bat = rbox(bx0, bx0 + bx, -by / 2, by / 2, -39.4, -39.4 + bz, 2.0)
    m._add("trunk", "battery_np_f550", bat, "battery", m.masses["battery"])
    imu = rbox(-4.0, 12.0, -5.0, 5.0, 3.5, 5.1, 1.0).fuse(
        [zcyl(1.0, px, py, 1.5, 3.5) for px in (-2.5, 10.5) for py in (-3.5, 3.5)])
    m._add("trunk", "imu_board", imu, "pcb", m.masses["imu"])
    m._add("trunk", "imu_chip", box(2.0, 6.0, -2.0, 2.0, 5.1, 6.0), "chip", 0.0)
    pw = rbox(-21.0, 12.0, -14.0, 14.0, 27.0, 28.6, 2.0).fuse(
        [zcyl(1.5, -15.0, py, 1.5, 27.0) for py in (-3.5, 3.5)])   # servolarin arasinda, tabana
    m._add("trunk", "power_board", pw, "pcb2", m.masses["power"])
    m.meta["imu_item"] = "imu_chip"

    # kalca yaw servolari ve boyun horn'u
    for side, mir in (("left", False), ("right", True)):
        b, h = m._servo("hip_yaw", mir)
        m._add("trunk", side + "_hip_yaw_servo", b, "servo", m.servo_mass("hip_yaw"))
        m.shared.setdefault("hip_yaw_horn", {})[side] = h


# ---------------------------------------------------------------------------
# TR-35 yer tutucu (P2-0 arayuz dondurma; P2b bu bolumu sasi, kabuk, elektronik yerlesimi, havalandirma ile
# degistirir: cad/trunk.py + cad/electronics.py)
# ---------------------------------------------------------------------------
def build_tr35_stub(m, sv):
    """TR-35 govdesi, P2-0 yer tutucu: kalca yaw ES3054 + doner tabla yataklari (2 rulman/bacak, yuva tupu),
    boyun DS govdesi + idler yuvasi (horn ve idler pimi -> shared, P2c boyun link'ine ekler), elektronik zarflari
    (cad/electronics.py, params.trunk.electronics yerlesimi), sasi kutle bloklari. Kabuk ve gercek sasi P2b."""
    import electronics
    from geom import box, zcyl
    servos.require(m, sv, ("hip_yaw",), "ES3054", "trunk.tr35_b2")
    servos.require(m, sv, ("neck_pitch",), "DS32XX", "trunk.tr35_b2")
    p, T = m.p, m.p["trunk"]
    tt = servos.turntable_stack(p["turntable"])
    b = tt["bearing_rec"]
    yx, yy, _ = p["j_hip_yaw"]
    for side, mir in (("left", False), ("right", True)):
        sp = m.servo_parts("hip_yaw", mir, horn_toward=T["hip_yaw_horn_toward"])
        m._add("trunk", side + "_hip_yaw_servo", sp["body"], "servo", m.servo_mass("hip_yaw"))
        m.shared.setdefault("hip_yaw_horn", {})[side] = sp["horn"]
        sy = -1.0 if mir else 1.0
        hz0, hz1 = tt["lower"][0], tt["upper"][1]
        hous = zcyl(tt["housing_r"], yx, sy * yy, hz0, hz1).cut(zcyl(b["D"] / 2.0, yx, sy * yy, hz0 - 1, hz1 + 1))
        r = tt["housing_r"]       # iki yuva ortada birlesir (P2b tek parca yapar): yer tutucuda y = 0'da ayrik
        hous = hous.common(box(yx - r - 1, yx + r + 1, 0.0 if sy > 0 else -yy - r - 1, yy + r + 1 if sy > 0 else 0.0,
                               hz0 - 1, hz1 + 1))
        m._add("trunk", side + "_hip_yaw_housing", hous, "bracket")
        for k, (z0, z1) in (("lo", tt["lower"]), ("hi", tt["upper"])):
            ring = zcyl(b["D"] / 2.0, yx, sy * yy, z0, z1).cut(zcyl(b["d"] / 2.0, yx, sy * yy, z0 - 1, z1 + 1))
            m._add("trunk", "%s_hip_yaw_bearing_%s" % (side, k), ring, "metal", b["mass_g"] / 1000.0)
    # boyun: DS govdesi trunk'ta sabit (spec 3.1, horn -y); besik idler yuvasi + rulman trunk'ta
    sp = m.servo_parts("neck_pitch", horn_toward=T["neck_horn_toward"])
    m._add("trunk", "neck_pitch_servo", sp["body"], "servo", m.servo_mass("neck_pitch"))
    m._add("trunk", "neck_pitch_idler_seat", sp["idler_seat"], "bracket")
    m._add("trunk", "neck_pitch_idler_bearing", sp["idler_bearing"], "metal",
           servos.BEARINGS[p["idler"]["bearing"]]["mass_g"] / 1000.0)
    m.shared["neck_pitch_horn"] = sp["horn"]
    m.shared["neck_pitch_idler_pin"] = sp["idler_pin"]
    # elektronik (gercek zarf kutulari) ve sasi kutle bloklari
    for name, pl in T["electronics"].items():
        part = name.rsplit("#", 1)[0]
        rec = electronics.PARTS[part]
        shape = electronics.solid(part, pl[:3], pl[3] if len(pl) > 3 else 0.0, pl[4] if len(pl) > 4 else "xyz")
        m._add("trunk", name.replace("#", "_"), shape, rec["kind"], rec["mass_g"] / 1000.0)
    for name, (c, half, mass_g) in T["mass_blocks"].items():
        blk = box(c[0] - half[0], c[0] + half[0], c[1] - half[1], c[1] + half[1], c[2] - half[2], c[2] + half[2])
        m._add("trunk", name, blk, "bracket", mass_g / 1000.0)
    m.meta["imu_item"] = T["imu_item"]


GENERATORS = {"xl330_27": build_xl330, "tr35_b2": build_tr35_stub}
