# -*- coding: utf-8 -*-
"""
Boyun ve kafa geometri uretecleri: boyun, kafa egim/donus/yatma cerceveleri, kafa kabuklari, gaga.

Uretec arayuzu (openduck.OpenDuck._build): GENERATORS[konfig "generators.head"](m, sv); m model (m.p parametreler,
m._add / m._joint kayit, m._servo(anahtar, ayna) yerlestirilmis (govde, horn), m.servo_frame, m.servo_mass /
m.horn_mass, m.masses, m.limits, m.home, m.meta, m.shared uretecler arasi sekiller), sv eklem anahtari -> servo
turu kaydi (servos.SERVO_TYPES). Servo govdesi parcasinin adi "[left_|right_]<anahtar>_servo" olmali
(openduck.servo_signs isaret kontrolu).
"""
import math

import servos
from geom import (V, _at_z, _parallel, bar_y, box, fillet, fillet_axis, frame, moved, rbox, shell_rbox, stadium_y,
                  xcyl, ycyl, zcyl)

HEAD_KEYS = ("neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth")


def build_xl330(m, sv):
    """27 cm XL330 boyun + kafa (refactor oncesi OpenDuck._neck_head, birebir): eklemler ID 30-34."""
    servos.require(m, sv, HEAD_KEYS, "XL330", "head.xl330_27", scale=1.0)
    p = m.p
    w = p["wall"]
    np_, hp, hy, hr, mo = (p["j_neck_pitch"], p["j_head_pitch"], p["j_head_yaw"],
                           p["j_head_roll"], p["j_mouth"])

    def horn_bolts(key, length, cb=None):
        """key servosunun horn'una oturan braket icin civata delikleri (servos.horn_bolt_cutters), servo
        cercevesinde: horn'daki delik dairesiyle ayni (XL330: 4 x 2.0 mm, 4.2 mm yaricap, 45 derece). length:
        horn dis yuzunden itibaren delik boyu; cb = (yaricap, bas, son) istege bagli havsa."""
        return moved(servos.horn_bolt_cutters(sv[key], length, cb), frame(*m.servo_frame(key)))

    # boyun: iki servo sirt sirta + ek plakalari
    m._joint("neck_pitch", "trunk", "neck", np_, (0, -1, 0), m.limits["neck_pitch"], m.home["neck_pitch"], 30)
    nb, nh = m._servo("neck_pitch")
    hpb, hph = m._servo("head_pitch")
    m._add("trunk", "neck_pitch_horn", nh, "horn", m.horn_mass("neck_pitch"))
    m._add("neck", "neck_pitch_servo", nb, "servo", m.servo_mass("neck_pitch"))
    m._add("neck", "head_pitch_servo", hpb, "servo", m.servo_mass("head_pitch"))
    splice = box(14.0, 16.0, -14.5, 11.5, 50.0, 64.5).fuse(box(36.0, 38.0, -14.5, 11.5, 50.0, 64.5))
    m._add("neck", "neck_splice", splice, "bracket")

    # kafa egim cercevesi (neck_pitch govdesi): kafa pitch horn'u -> kafa yaw horn'u
    m._joint("head_pitch", "neck", "head_tilt", hp, (0, 1, 0), m.limits["head_pitch"], m.home["head_pitch"], 31)
    hyb, hyh = m._servo("head_yaw")
    arms = stadium_y(hp[0], hp[2], 10.0, hy[2] - 1.5, 14.5, 17.0).fuse(
        stadium_y(hp[0], hp[2], 10.0, hy[2] - 1.5, -17.0, -14.5))
    tilt = arms.fuse(rbox(16.0, 36.0, -17.0, 17.0, hy[2] - 3.0, hy[2], 3.0)).removeSplitter()
    tilt = tilt.cut(ycyl(1.4, hp[0], hp[2], 13.0, 18.0)).cut(zcyl(1.4, hy[0], 0, hy[2] - 4, hy[2] + 1))
    tilt = tilt.cut(horn_bolts("head_pitch", 3.5)).cut(horn_bolts("head_yaw", 4.0))
    m._add("head_tilt", "head_pitch_horn", hph, "horn", m.horn_mass("head_pitch"))
    m._add("head_tilt", "head_tilt_bracket", tilt, "bracket")
    m._add("head_tilt", "head_yaw_horn", hyh, "horn", m.horn_mass("head_yaw"))

    # kafa donus cercevesi (yaw_roll_motion): yaw servosu + roll ekseni yataklari
    m._joint("head_yaw", "head_tilt", "head_turn", hy, (0, 0, 1), m.limits["head_yaw"], 0.0, 32)
    hrb, hrh = m._servo("head_roll")
    turn = box(8.1, 16.0, -24.5, 5.0, 104.5, 127.0).fuse(
        [box(36.0, 39.0, -24.5, 5.0, 104.5, 127.0), xcyl(5.0, 0.0, hr[2], 39.0, 45.0),
         box(8.1, 16.0, 4.0, 7.5, 109.0, 122.0)]).removeSplitter()          # havsa et payi (+y)
    # arka plaka roll horn'una civatali (baslar yaw servosu tarafindaki havsalarda), on plaka
    # (roll pimi) yaw servosu govdesine; iki plaka yaw servosu uzerinden birlesir
    turn = turn.cut(xcyl(1.4, 0.0, hr[2], 7.0, 12.0)).cut(horn_bolts("head_roll", 9.0, (2.1, 5.7, 9.0)))
    m._add("head_turn", "head_yaw_servo", hyb, "servo", m.servo_mass("head_yaw"))
    m._add("head_turn", "head_turn_bracket", turn, "bracket")
    m._add("head_turn", "head_roll_horn", hrh, "horn", m.horn_mass("head_roll"))

    # kafa
    m._joint("head_roll", "head_turn", "head", hr, (-1, 0, 0), m.limits["head_roll"], 0.0, 33)
    hx0, hx1 = p["head_x"]
    W2 = p["head_w"] / 2
    zt, zs, zb = p["head_top"], p["head_split"], p["head_bot"]
    fx = p["face_x"]
    R, c = 22.0, 0.2                    # kaput kose yaricapi, gecme boslugu
    zl, zlip = zs - 1.0, zs + 1.0       # dikis duzlemi (kaput alt kenari), alt kabuk dudaginin ustu

    def hood_body(d, z0, z1, top=True):
        """Kaput dis formunun d kadar iceri ofseti (top: ust yuz ve ust kenarlar da)."""
        return rbox(hx0 + d, hx1 - d, -W2 + d, W2 - d, z0, z1 - (d if top else 0.0), R - d,
                    (R - d) if top else 0.0)

    # gaga ekseni delikleri: alt gaga gobegi ve gaga horn'u (ikisi de r6.5) 0.8 mm boslukla gecer (FDM'de
    # donen gobek icin 0.4 mm dar: ilk katman tasmasi/asiri ekstruzyon gagayi sikistirir). Delik altinda
    # kaput duvari dikis duzlemine 1.3 mm kalir.
    mouth_holes = [ycyl(7.3, mo[0], mo[2], *sorted((sy * (W2 - 4.5), sy * (W2 + 3.0)))) for sy in (1, -1)]
    # ust kaput: alt acik, onde yuz icin acik. Yuz acikligi ic bosluk profiliyle (ust koseler
    # R20.2) kesilir; on kenar, dis kose yuvarlaginda dudak >= 1.25 mm kalan yerde dik duzlemle biter
    x_trim = hx1 - R + math.sqrt(R ** 2 - (R - w + 1.25) ** 2)
    hood = hood_body(0.0, zl, zt).cut(hood_body(w, zl - 1.0, zt))
    fo = box(fx, hx1 + 2.0, -W2 + w, W2 - w, zl - 1.0, zt - w)
    fo = fillet(fo, R - w, [e for e in fo.Edges if _parallel(e, (1, 0, 0)) and _at_z(e, zt - w)])
    hood = hood.cut(fo).cut(box(x_trim, hx1 + 2.0, -W2 - 1.0, W2 + 1.0, zl - 1.0, zt + 1.0))
    for mh_ in mouth_holes:
        hood = hood.cut(mh_)
    m._add("head", "head_hood", hood, "shell")
    # alt kabuk (turuncu): taban + ust gaga. Arka koseler kaputla ayni (R22), gaga ucu R18.
    # Kaputla bindirme: kaputun altinda (x < xb) dis duvar dikis duzleminde (zl) biter; kaputun ic
    # yuzune c bosluklu, w kalin dudak zlip'e kadar kaputun icine girer, altindaki 45 derece pahli
    # kalinlastirma onu duvara baglar. Kaputun onunde gaga tam duvarla zlip'e cikar.
    xf, xb = hx1 - 6.0, x_trim + c

    def tray(d, z0, z1, r_bot):
        b = box(hx0 + d, xf - d, -W2 + d, W2 - d, z0, z1)
        b = fillet(b, R - d, [e for e in b.Edges if _parallel(e, (0, 0, 1)) and e.BoundBox.XMax < hx0 + d + 1e-6])
        b = fillet(b, 18.0 - d, [e for e in b.Edges if _parallel(e, (0, 0, 1)) and e.BoundBox.XMin > xf - d - 1e-6])
        return fillet(b, r_bot, [e for e in b.Edges if _at_z(e, z0)])
    outer = tray(0.0, zb, zlip, 6.0)
    low = outer.cut(tray(w, zb + w, zlip + 1.0, 6.0 - w))
    keep = box(hx0 - 1.0, xb, -W2 - 1.0, W2 + 1.0, zb, zlip + 1.0)
    band = box(hx0 - 1.0, xb, -W2 - 1.0, W2 + 1.0, zl, zlip + 1.0)
    low = low.cut(band.cut(hood_body(w + c, zl - 1.0, zlip + 2.0, False)))
    d_in = 2 * w + c
    lip_in = hood_body(d_in, zl - 5.0, zlip + 1.0, False)
    lip = hood_body(w + c, zl - 1.0, zlip, False).cut(lip_in)
    rib = hood_body(1.0, zl - 4.0, zl, False).cut(lip_in)
    lo_x, hi_x, lo_y, hi_y = hx0 + d_in - 0.05, hx1 - d_in + 0.05, -W2 + d_in - 0.05, W2 - d_in + 0.05
    rib = rib.makeChamfer(2.0, [e for e in rib.Edges if _at_z(e, zl - 4.0) and e.BoundBox.XMin > lo_x
                                and e.BoundBox.XMax < hi_x and e.BoundBox.YMin > lo_y and e.BoundBox.YMax < hi_y])
    low = low.fuse([lip.common(keep), rib.common(keep).common(outer)]).removeSplitter()
    low = low.cut(zcyl(p["neck_hole_r"], hy[0], 0.0, zb - 1.0, zb + w + 1.0))
    low = low.cut(box(46.0, hx1 + 1, -W2 - 1, W2 + 1, zb - 1.0, 107.5))
    for mh_ in mouth_holes:
        low = low.cut(mh_)
    bill = box(46.0, hx1 + 1, -W2 - 1, W2 + 1, zb - 1.0, zlip + 1.0)
    m._add("head", "head_bottom_shell", low.cut(bill), "shell")
    m._add("head", "upper_bill", low.common(bill), "trim")
    m._add("head", "upper_bill_pad", rbox(52.0, hx1 - 7.0, -W2 + w + 0.5, W2 - w - 0.5, 106.4, 107.5, 4.0),
              "pad")
    # yuz plakasi, goz (kamera), ToF penceresi, kayit LED'i
    ez = p["eye_z"]
    # kaputun ic yuzunu (ust koseler R20.2) 0.3 mm, alt kabuk dudagini 0.2 mm bosluk birakarak izler
    face = box(fx - 2.0, fx, -W2, W2, 107.5, zt).common(hood_body(w + 0.3, zlip + c, zt).fuse(
        hood_body(d_in + c, 107.0, zlip + c + 0.01, False)))
    face = face.cut(xcyl(9.6, 0.0, ez, fx - 3, fx + 1)).cut(rbox(fx - 3, fx + 1, 19.0, 26.0, 126.0, 132.0, 0.0))
    face = face.cut(xcyl(1.4, -20.0, 145.0, fx - 3, fx + 1))       # kayit LED'i deligi
    m._add("head", "face_plate", face, "face")
    ring = xcyl(p["eye_r"], 0.0, ez, fx, fx + 9.5).cut(xcyl(9.6, 0.0, ez, fx - 1, fx + 11))
    ring = fillet(ring, 1.5, [e for e in ring.Edges if abs(e.BoundBox.XMin - fx - 9.5) < 1e-6
                              and abs(e.BoundBox.XMax - fx - 9.5) < 1e-6])
    m._add("head", "eye_ring", ring, "trim")
    m._add("head", "camera_lens", xcyl(9.5, 0.0, ez, fx - 2.0, fx + 6.0), "lens", m.masses["camera"])
    m._add("head", "camera_glass", xcyl(4.0, 1.5, ez + 2.0, fx + 6.0, fx + 6.3), "glass", 0.0)
    m._add("head", "camera_holder", box(fx - 4.5, fx - 2.0, -7.4, 7.4, ez - 8.0, ez + 8.0), "chip", 0.0)
    tof = rbox(fx - 2.5, fx + 0.8, 19.2, 25.8, 126.2, 131.8, 0.0)
    m._add("head", "tof_sensor", fillet_axis(tof, 2.5, (1, 0, 0)), "lens", m.masses["tof"])
    m._add("head", "rec_led", xcyl(1.3, -20.0, 145.0, fx - 1.0, fx + 0.8), "led", 0.0)
    # elektronik: kart + HAT dik, hoparlor arkada
    m._add("head", "main_board", box(fx - 7.0, fx - 5.4, -32.5, 32.5, 114.5, 144.5), "pcb", m.masses["board"])
    m._add("head", "robot_hat", box(fx - 14.0, fx - 12.4, -32.5, 32.5, 114.5, 144.5), "pcb2", m.masses["hat"])
    m._add("head", "speaker", rbox(-17.9, 7.1, -16.8, 18.2, 127.0, 133.0, 3.0), "chip", m.masses["speaker"])
    # ic cerceve (tek parca): roll servosu yuvasi, on yatak duvari, gaga servosu rafi. Yuva ile duvar
    # sagda (-y) bir rayla, solda yuva duvari -> raf -> duvar uzerinden baglanir; kapali halka
    # kafa donus/egim parcalarinin supurme alaninin (|y| < 27, eksenden r < 20) disinda kalir.
    frame_parts = [box(-22.9, 5.1, -26.5, 11.5, 103.6, 105.6),
                   box(-22.9, 3.0, -26.5, -24.5, 105.6, 125.6),
                   box(-22.9, 3.0, 9.5, 11.5, 105.6, 125.6),
                   box(-22.9, -20.9, -24.5, 9.5, 105.6, 125.6)]
    frame_parts += [zcyl(1.6, px, py, zb + w, 103.6) for px in (-19.0, -4.0) for py in (-22.0, 8.0)]
    frame_parts += [box(43.0, 46.0, -39.0, 12.0, 104.0, 128.0)]                  # on yatak duvari
    frame_parts += [box(-12.0, 5.1, -29.5, -26.0, 103.6, 105.6),                  # sag ray
                    box(-12.0, 46.0, -39.0, -29.5, 103.6, 105.6), zcyl(1.6, 40.0, -34.0, zb + w, 103.6)]
    frame_parts += [box(16.6, 50.6, 13.5, 39.5, 106.6, 108.6)]                   # gaga servosu rafi
    frame_parts += [zcyl(1.6, px, 34.0, zb + w, 106.6) for px in (20.0, 46.0)]
    frame_parts += [box(-4.0, 15.0, 10.0, 22.0, 106.6, 108.6), box(14.0, 17.0, 13.5, 22.0, 106.6, 108.6),
                    box(43.0, 46.0, 11.0, 15.0, 104.0, 108.6)]                    # yuva -> raf -> duvar
    hframe = frame_parts[0].fuse(frame_parts[1:]).removeSplitter()
    hframe = hframe.cut(xcyl(5.2, 0.0, hr[2], 42.0, 47.0))
    m._add("head", "head_frame", hframe, "bracket")
    m._add("head", "head_roll_servo", hrb, "servo", m.servo_mass("head_roll"))
    mb, mh = m._servo("mouth")
    m._add("head", "mouth_servo", mb, "servo", m.servo_mass("mouth"))

    # gaga (alt cene): gaga servosu ekseninde doner, karsi tarafta pim
    m._joint("mouth", "head", "jaw", mo, (0, 1, 0), m.limits["mouth"], 0.0, 34)
    tip = (hx1 - 7.0, 100.0)
    jaw = rbox(52.0, tip[0], -W2 - 0.5, W2 + 0.5, 98.2, 101.6, 6.0)
    jaw = jaw.fuse(box(52.0, 62.0, -W2 - 3.5, W2 + 3.5, 98.2, 101.6))
    for sy in (1, -1):
        y0, y1 = (W2 + 1.0, W2 + 3.5) if sy > 0 else (-W2 - 3.5, -W2 - 1.0)
        jaw = jaw.fuse(bar_y((mo[0], mo[2]), (57.0, 98.2 + 5.5), 5.5, y0, y1))   # kol alt gaga altina tasmaz
        jaw = jaw.fuse(ycyl(6.5, mo[0], mo[2], y0, y1))                    # kol gobegi (civata dairesi)
    jaw = jaw.fuse(ycyl(6.5, mo[0], mo[2], mo[1] + 14.5, W2 + 1.0))       # sol: horn'a gecen gobek
    jaw = jaw.fuse(ycyl(2.5, mo[0], mo[2], -W2 - 1.0, -W2 + 3.0))         # sag: pim
    jaw = jaw.removeSplitter()
    # disaridan horn'a 4 x M2 (kol + gobek boyunca) ve merkez vidasi erisim deligi
    jaw = jaw.cut(horn_bolts("mouth", 8.0)).cut(ycyl(1.4, mo[0], mo[2], mo[1] + 13.0, W2 + 4.5))
    m._add("jaw", "mouth_horn", mh, "horn", m.horn_mass("mouth"))
    m._add("jaw", "lower_bill", jaw, "trim")
    m._add("jaw", "lower_bill_pad", rbox(53.0, tip[0] - 1.0, -W2 + 2.0, W2 - 2.0, 101.6, 106.4, 4.0), "pad")


# ---------------------------------------------------------------------------
# TR-35 yer tutucu (P2-0 arayuz dondurma; P2c bu bolumu boyun U-link'i, MG90S/SG90 cepleri, cene/alin tamponu ve
# sabit etli kabuklarla degistirir)
# ---------------------------------------------------------------------------
HEAD_CHAIN = (("neck_pitch", "trunk", "neck", (0, -1, 0), 30), ("head_pitch", "neck", "head_tilt", (0, 1, 0), 31),
              ("head_yaw", "head_tilt", "head_turn", (0, 0, 1), 32), ("head_roll", "head_turn", "head", (-1, 0, 0), 33),
              ("mouth", "head", "jaw", (0, 1, 0), 34))
# servo govdesi / horn link'i (isaret kurali +1): neck_pitch govde trunk (horn -y, neck'i surer), head_pitch govde
# neck, head_yaw govde head_turn (cocuk), head_roll govde head (cocuk), mouth govde head
HEAD_BODY_LINK = {"head_pitch": "neck", "head_yaw": "head_turn", "head_roll": "head", "mouth": "head"}
HEAD_HORN_LINK = {"neck_pitch": "neck", "head_pitch": "head_tilt", "head_yaw": "head_tilt", "head_roll": "head_turn",
                  "mouth": "jaw"}


def build_tr35_stub(m, sv):
    """TR-35 boyun + kafa, P2-0 yer tutucu: eklemler (ID 30-34), boyun DS horn'u + idler pimi (trunk'tan shared),
    MG90S/SG90 govde ve horn zarflari, sabit etli kafa kabugu (kutu, boyun deligi), cene/alin tamponu (neck link'te,
    carpisma kutulu "neck_bumper"), butce kutleli braket bloklari ve kablo demetleri."""
    from geom import harness_paths, tube
    servos.require(m, sv, ("neck_pitch",), "DS32XX", "head.tr35_b2")
    servos.require(m, sv, ("head_pitch", "head_yaw", "head_roll"), "MG90S", "head.tr35_b2")
    servos.require(m, sv, ("mouth",), "SG90", "head.tr35_b2")
    p, H = m.p, m.p["head"]
    for jn, parent, child, axis, sid in HEAD_CHAIN:
        m._joint(jn, parent, child, p["j_" + jn], axis, m.limits[jn], m.home.get(jn, 0.0), sid)
    m._add("neck", "neck_pitch_horn", m.shared["neck_pitch_horn"], "horn", m.horn_mass("neck_pitch"))
    m._add("neck", "neck_pitch_idler_pin", m.shared["neck_pitch_idler_pin"], "metal", H["mass_g"]["idler_pin"] / 1000.0)
    for key in ("head_pitch", "head_yaw", "head_roll", "mouth"):
        sp = m.servo_parts(key, horn_toward=H["horn_toward"].get(key))
        m._add(HEAD_BODY_LINK[key], key + "_servo", sp["body"], "servo", m.servo_mass(key))
        m._add(HEAD_HORN_LINK[key], key + "_horn", sp["horn"], "horn", m.horn_mass(key))
    # kafa kabugu: sabit etli kutu (PLA), altta boyun deligi; P2c gercek kaput/alt kabuk/gaga
    hx0, hx1 = p["head_x"]
    W2 = p["head_w"] / 2.0
    zt, zb, w = p["head_top"], p["head_bot"], p["wall"]
    shell = shell_rbox(hx0, hx1, -W2, W2, zb, zt, w, 18.0, 14.0, 6.0)
    hy = p["j_head_yaw"]
    shell = shell.cut(zcyl(H["neck_hole_r"], hy[0], hy[1], zb - 1.0, zb + w + 1.0))
    m._add("head", "head_shell", shell, "shell", H["shell_mass_g"] / 1000.0)
    # tampon (spec 3.1): kafa darbesini boyun U-link'ine aktarir; MJCF'de boyun govdesine bagli carpisma geomu
    # (ad "neck_bumper"; "neck_collision" RL getup varyantinin ek geomudur, kullanilmaz). Disa aktarma P3.
    bc, bh = H["bumper"]["center"], H["bumper"]["half"]
    bump = box(bc[0] - bh[0], bc[0] + bh[0], bc[1] - bh[1], bc[1] + bh[1], bc[2] - bh[2], bc[2] + bh[2])
    m._add("neck", "neck_bumper", bump, "pad", None, (V(*bc), tuple(bh), 0.0))
    # alt gaga yer tutucusu (jaw) ve braket kutle bloklari
    jb = H["jaw_block"]
    m._add("jaw", "lower_bill", box(*jb), "trim", H["jaw_mass_g"] / 1000.0)
    for link, (c, half, mass_g) in H["mass_blocks"].items():
        blk = box(c[0] - half[0], c[0] + half[0], c[1] - half[1], c[1] + half[1], c[2] - half[2], c[2] + half[2])
        m._add(link, link + "_bracket", blk, "bracket", mass_g / 1000.0)
    for jn, parent, child, _, _ in HEAD_CHAIN:
        for part, pts, d, mass in harness_paths(p["harness"], jn):
            m._add(parent if part == "in" else child, "%s_harness_%s" % (jn, part), tube(pts, d / 2.0), "harness", mass)


GENERATORS = {"xl330_27": build_xl330, "tr35_b2": build_tr35_stub}
