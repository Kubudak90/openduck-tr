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
from geom import (V, _at_z, _parallel, bar_y, box, fillet, fillet_axis, frame, moved, prism_xz, rbox, rotated,
                  stadium_y, xcyl, ycyl, zcyl)

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
# TR-35 boyun + kafa (P2c). Arayuz: docs/workflow/openduck-tr/p2-interface.md (P2-0, donmus); yuk yolu ve MG90S mil
# yuk hesabi: docs/workflow/openduck-tr/p2c-tampon.md. Olculer servo zarflarindan (servos.datums, horn cepleri),
# donmus eklem noktalarindan ve params.head'den turetilir; vida olculeri mutlaktir (olceklenmez).
# ---------------------------------------------------------------------------
HEAD_CHAIN = (("neck_pitch", "trunk", "neck", (0, -1, 0), 30), ("head_pitch", "neck", "head_tilt", (0, 1, 0), 31),
              ("head_yaw", "head_tilt", "head_turn", (0, 0, 1), 32), ("head_roll", "head_turn", "head", (-1, 0, 0), 33),
              ("mouth", "head", "jaw", (0, 1, 0), 34))
# servo govdesi / horn link'i (isaret kurali +1): neck_pitch govde trunk (horn -y, neck'i surer), head_pitch govde
# neck, head_yaw govde head_turn (cocuk), head_roll govde head (cocuk), mouth govde head
HEAD_BODY_LINK = {"head_pitch": "neck", "head_yaw": "head_turn", "head_roll": "head", "mouth": "head"}
HEAD_HORN_LINK = {"neck_pitch": "neck", "head_pitch": "head_tilt", "head_yaw": "head_tilt", "head_roll": "head_turn",
                  "mouth": "jaw"}


def _horn_xf(m, key, toward):
    """key servosunun kanonik cercevesindeki katiyi dunyaya (sifir poz) tasiyan f(sekil, kol=False). kol=True: horn
    kolu `toward` yonune dondurulur (servo_parts'taki aciyla ayni; cep, kol vidalari)."""
    o, h, f = m.servo_frame(key)
    pl = frame(o, h, f)
    a = servos.arm_angle_deg(h, f, toward) if toward is not None else 0.0

    def xf(shape, arm=False):
        s = moved(shape, pl)
        return rotated(s, o, h, a) if arm and abs(a) > 1e-9 else s
    return xf


def _capture(t, xf, y0, y1, grow):
    """Horn kol(lar)i + gobek dis hatti `grow` kadar buyuk, kanonik Y y0..y1 (horn'u saran braket govdesi)."""
    return xf(servos.horn_pocket(t, grow, depth=-y0 - grow, over=y1), arm=True)


def _horn_seat(t, xf, F, y1, from_inside=False):
    """Horn'u braket plakasina gomen kesiciler (plaka kanonik Y: horn dis yuzu 0 .. y1): cep (kol + gobek,
    pocket_clear), kol vidalari ve merkez vidasi erisim deligi. from_inside: vidalar horn tarafindan plakada kor
    kilavuz delige (plakanin disi kapali); degilse plakadan gecip horn koluna, baslari havsada."""
    hn = t["horn"]
    cut = [xf(servos.horn_pocket(t, F["pocket_clear"], depth=hn["arm_t"], over=0.0), arm=True)]
    if from_inside:
        cut.append(xf(servos.horn_bolt_cutters(t, y1 - F["skin"], r=F["m17_pilot_d"] / 2.0), arm=True))
    else:
        cb = (F["m17_head_d"] / 2.0, y1 - F["m17_head_h"], y1 + 1.0)
        cut.append(xf(servos.horn_bolt_cutters(t, y1 + 1.0, cb, r=F["m17_clear_d"] / 2.0), arm=True))
    cut.append(xf(ycyl(F["center_screw_d"] / 2.0, 0, 0, -0.01, y1 + 1.0)))
    return cut


def _prism_y(profile, y0, y1):
    """Y'de uzun bir XZ profil katisinin y0..y1 dilimi."""
    return profile.common(box(-1e3, 1e3, y0, y1, -1e3, 1e3))


def _hex_y(cx, cz, af, y0, y1):
    """Somun yuvasi: XZ'de anahtar agzi af olan altigen, Y'de y0..y1."""
    import Part
    r = af / math.sqrt(3.0)
    pts = [V(cx + r * math.cos(math.radians(30 + 60 * k)), y0, cz + r * math.sin(math.radians(30 + 60 * k)))
           for k in range(7)]
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, y1 - y0, 0))


def _slot_y(cx, z0, z1, r, y0, y1):
    """XZ'de dikey oval delik (yaricap r, merkezler z0+r .. z1-r), Y'de y0..y1 (hafifletme)."""
    if z1 - z0 < 2 * r + 1e-6:
        return None
    return stadium_y(cx, z0 + r, r, z1 - r, y0, y1).fuse(ycyl(r, cx, z1 - r, y0, y1)).removeSplitter()


def _harness(m, H):
    """Kafa modulu kablo demetleri: donmus gecisler (params.harness: gecis noktasi + cap) ve link ici guzergah
    (params.head.harness_route). Moduller arasi gecis (neck_pitch: trunk -> neck) P2-0 giris/cikis parcalarini aynen
    tutar, yalniz uzatilir (in_pre / out_post); kafa ici gecislerde (parent ve child kafa modulunde) P2-0'in kisa
    yer tutucu in/out noktalari sahibin guzergahiyla degisir ("in" / "out"; arayuz: link ici guzergah sahibindir),
    gecis noktasi ve cap degismez. Ayni demetin ardisik tuplari birlesme noktasinda es dogrusal biter (sifir hacimli
    temas). Doner {link: [(ad, noktalar, cap, kutle_kg)]}."""
    from geom import harness_paths
    hc, rt = m.p["harness"], m.p["head"]["harness_route"]
    out = {}
    for jn, parent, child, _, _ in HEAD_CHAIN:
        r = rt.get(jn, {})
        own = parent != "trunk"
        assert own or not ({"in", "out"} & set(r)), "%s: moduller arasi gecisin parcalari degistirilemez" % jn
        for part, pts, d, _ in harness_paths(hc, jn):
            cross = tuple(hc["crossings"][jn]["cross"])
            if part == "in":
                pts = ([tuple(q) for q in r["in"]] + [cross] if "in" in r else
                       [tuple(q) for q in r.get("in_pre", [])] + pts)
            else:
                pts = ([cross] + [tuple(q) for q in r["out"]] if "out" in r else
                       pts + [tuple(q) for q in r.get("out_post", [])])
            L = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
            mass = L * hc["g_per_mm3"] * math.pi * d * d / 4.0 / 1000.0
            out.setdefault(parent if part == "in" else child, []).append(("%s_harness_%s" % (jn, part), pts, d, mass))
    return out


def _harness_cut(hz, link, grow):
    """link'teki kablo demetlerinin grow kadar kalin hali (ayni link'teki braketlerde kablo kanali)."""
    from geom import tube
    return [tube(pts, d / 2.0 + grow) for _, pts, d, _ in hz.get(link, [])]


def _cut(shape, cutters):
    cutters = [c for c in cutters if c is not None]
    return shape.cut(cutters[0].fuse(cutters[1:]) if len(cutters) > 1 else cutters[0]) if cutters else shape


def _metal(m, link, name, axis, pts, r, a0, a1, mass_g):
    """Vida/pim zarfi (kind metal, aciklik deliklerinin icinde): eksen 'x'|'y'|'z' boyunca a0..a1, pts diger iki
    koordinat (x: (y, z), y: (x, z), z: (x, y)); kutle adet basina mass_g."""
    mk = {"x": lambda a, b: xcyl(r, a, b, a0, a1), "y": lambda a, b: ycyl(r, a, b, a0, a1),
          "z": lambda a, b: zcyl(r, a, b, a0, a1)}[axis]
    s = [mk(a, b) for a, b in pts]
    m._add(link, name, s[0].fuse(s[1:]) if len(s) > 1 else s[0], "metal", mass_g * len(pts) / 1000.0)


def _neck(m, sv, H, hz):
    """Boyun U-link'i (neck, PETG): A kolu DS aluminyum kolu gomer (cep + 2 x M2.5 somunlu + M3 merkez vidasi), B kolu
    trunk'in idler pimini (F623ZZ ic bileziginde M3) tasir; kopru head_pitch MG90S'in uzak ucunun altindan gecer.
    head_pitch besigi: kopru (servo altinda -y'ye acik yarik, alt kulak yarikta), yan duvarlar ve ust kulak plakasi
    kulaklarin horn tarafinda; servo arkadan (-y) takilir, 2 x M2 kulak vidasi arkadan. Duvarlar ve plaka yaw kablo
    demetinin pitch ile donme bandinin (y <= 1) onunde kalir. head_pitch devrilme/eksenel yuku: MG90S horn'unun iki
    yaninda baski yuzleri (ic: duvarlara vidali, dis: ayakli, kopruye vidali), aradaki head_tilt diski 0.3 mm
    bosluklu doner (p2c-tampon.md). Cene/alin tamponu kopru onundeki govde dilinde TPU yastik (carpisma kutusu
    "neck_bumper")."""
    p, N, F, B, G = m.p, H["neck"], H["fit"], H["bumper"], H["mass_g"]
    t, tp = sv["neck_pitch"], sv["head_pitch"]
    nx, _, nz = p["j_neck_pitch"]
    px, _, pz = p["j_head_pitch"]
    hw, w = N["half_w"], N["wall_t"]
    toward = p["trunk"]["neck_horn_toward"]
    assert toward[2] >= math.cos(math.radians(30.0)) * math.sqrt(sum(c * c for c in toward)), \
        "head.tr35_b2: boyun A kolu DS horn kolunu +z yonunde gomer (trunk.neck_horn_toward=%r)" % (toward,)
    horn = m.shared["neck_pitch_horn"]
    m._add("neck", "neck_pitch_horn", horn, "horn", m.horn_mass("neck_pitch"))
    m._add("neck", "neck_pitch_idler_pin", m.shared["neck_pitch_idler_pin"], "metal", G["idler_pin"] / 1000.0)
    xf = _horn_xf(m, "neck_pitch", toward)
    fy = p["servo_face"]["neck_pitch"][1]                     # DS horn -y: dunya y = fy - Y (kanonik)
    st = servos.idler_stack(t, p["idler"]["bearing"])
    yb0, yb1 = sorted((fy - st["leg"][0], fy - st["leg"][1]))  # B kolu (idler pimi tarafi)
    arm_t = t["horn"]["arm_t"]
    ya0 = fy - N["leg_a_t"]                                   # A kolu dis yuzu (arayuz: >= -31.4)
    ya_in = fy + arm_t + F["pocket_clear"]                    # DS kol cebinin ic yuzu
    ya2 = ya0 + N["leg_t"]                                    # kol ustunde ince A kolu ic yuzu
    # head_pitch MG90S (horn +y, uzak uc -z): dunya y = hf + Y, z = pz - Z
    dp = servos.datums(tp)
    hf = p["servo_face"]["head_pitch"][1]
    w2 = dp["half_w"]
    z_far = pz - dp["far_end_z"]
    ytab0, ytab1 = hf + dp["tab"][0], hf + dp["tab"][1]
    z_hole_near = pz - dp["tab_holes"][0][1]
    zb1 = z_far - F["clear"]                                  # kopru ustu: servo uzak ucunun altinda
    zb0 = zb1 - N["bridge_t"]
    ztop = N["top_z"]
    yr0 = hf - tp["horn"]["arm_t"] - F["pocket_clear"]         # head_tilt rotor diski bandi
    yr1 = hf + H["tilt"]["rotor_t"]
    yti1 = yr0 - F["gap"]                                     # ic baski yuzu
    yti0 = yti1 - N["thrust_in_t"]
    yto0 = yr1 + F["gap"]                                     # dis baski plakasi (+ kopruye oturan ayak)
    yto1 = yto0 + N["thrust_out_t"]
    yfo1 = yb0 - F["clear"]
    lobe = tp["horn"]["arm_r"] + tp["horn"]["arm_tip_r"] + F["pocket_clear"] + F["pocket_wall"]
    z_relief = min(zb1, pz - lobe - 2.0 * F["clear"])           # rotor alt kol ucu icin kopru cukuru
    xw = w2 + F["clear"]
    bx0, bx1, by0, by1, bz0, bz1 = B["box"]
    tg = B["tongue"]                                          # govde dili yastiga gecer (x, yari y, z0, z1)

    prof = ycyl(N["disc_r"], nx, nz, -1e3, 1e3).fuse(box(nx - hw, nx + hw, -1e3, 1e3, nz, zb1)).removeSplitter()
    leg_a = _prism_y(prof, ya0, ya2).fuse(_capture(t, xf, fy - ya_in, fy - ya2,
                                                   F["pocket_clear"] + F["pocket_wall"]))
    leg_b = _prism_y(prof, yb0, yb1)
    bridge = box(nx - hw, nx + hw, ya0, yb1, zb0, zb1)
    walls = [box(px + xw, px + xw + w, N["wall_y0"], yti0, zb1 - 0.01, ztop),
             box(px - xw - w, px - xw, N["wall_y0"], yti0, zb1 - 0.01, ztop)]
    ytp0 = ytab1 + F["pocket_clear"]                          # ust kulak plakasi: kulagin horn tarafinda
    ytp1 = ytp0 + N["tab_plate_t"]
    tabp = box(px - xw - w, px + xw + w, ytp0, ytp1, pz + w2 + F["clear"], ztop)
    z_hole_far = pz - dp["tab_holes"][1][1]
    stem = box(nx + hw - 0.01, bx0, -B["stem_half_y"], B["stem_half_y"], zb0, bz1)
    tongue = box(bx0 - 0.01, bx0 + tg[0], -tg[1], tg[1], tg[2], tg[3])
    link = leg_a.fuse([leg_b, bridge, stem, tongue, tabp] + walls).removeSplitter()
    arm_holes = [(h.BoundBox.Center.x, h.BoundBox.Center.z)
                 for h in servos.horn_bolt_holes(t, horn, ya0 - 1.0, ya0 + 1.0, 1.0)]
    wscr = [(px + s * (xw + w / 2.0), z) for s in (1, -1) for z in (zb1 + N["thrust_in_screw_dz"],
                                                                   ztop - N["thrust_in_screw_dz"])]
    yfc = (yto1 + yfo1) / 2.0
    oscr = [(px + s * N["thrust_out_screw_dx"], yfc) for s in (1, -1)]
    r_sl = N["slot_r"]
    arm_top = nz + t["horn"]["hole_r"][-1] + t["horn"]["arm_tip_r"] + F["pocket_clear"] + F["pocket_wall"]
    cut = [xf(servos.horn_pocket(t, F["pocket_clear"], depth=arm_t, over=0.0), arm=True),
           xf(servos.horn_bolt_cutters(t, fy - ya0 + 1.0, r=F["m25_clear_d"] / 2.0), arm=True),
           ycyl(N["center_hole_r"], nx, nz, ya0 - 1.0, fy + 0.01),
           ycyl(st["boss_r"] + 0.02, nx, nz, yb0 - 1.0, yb1 + 1.0),
           box(px - w2 - F["pocket_clear"], px + w2 + F["pocket_clear"], ya2 + 0.01, ytp0, zb0 - 1.0, zb1 + 1.0),
           box(nx - hw - 1.0, nx + hw + 1.0, yti1 - 0.01, yto0 + 0.01, z_relief, zb1 + 1.0),
           ycyl(F["m2_pilot_d"] / 2.0, px, z_hole_near, ytp0 - 0.01, ytp1 + 0.01),
           ycyl(F["m2_pilot_d"] / 2.0, px, z_hole_far, ytp0 - 0.01, ytp0 + N["tab_screw_depth"]),
           _slot_y(nx, arm_top + w, zb0 - w, r_sl, ya0 - 1.0, ya2 + 0.01),
           _slot_y(nx, nz + st["boss_r"] + w + 2.0, zb0 - w, r_sl, yb0 - 1.0, yb1 + 1.0)]
    cut += [_hex_y(cx, cz, F["m25_nut_af"], ya0 - 1.0, ya0 + F["m25_nut_t"]) for cx, cz in arm_holes]
    cut += [ycyl(F["m2_pilot_d"] / 2.0, x, z, yti0 - N["thrust_in_screw_depth"], yti0 + 0.01) for x, z in wscr]
    cut += [zcyl(F["m2_pilot_d"] / 2.0, x, y, zb1 - N["thrust_out_screw_depth"], zb1 + 0.01) for x, y in oscr]
    cut += [box(px - xh, px + xh, y0_, y1_, zb0 - 1.0, zb1 + 1.0) for xh, y0_, y1_ in N["bridge_holes"]]
    cut += _harness_cut(hz, "neck", F["harness_clear"])
    m._add("neck", "neck_link", _cut(link, cut), "bracket")
    _metal(m, "neck", "head_pitch_tab_screws", "y", [(px, z_hole_near), (px, z_hole_far)],
           F["m2_pilot_d"] / 2.0 - 0.05, ytab0, ytp0 + N["tab_screw_depth"] - 0.5, G["m2_screw"])
    _metal(m, "neck", "neck_arm_screws", "y", arm_holes, F["m25_clear_d"] / 2.0 - 0.15, ya0 + F["m25_nut_t"] + 0.01,
           ya_in - 0.01, G["m25_screw_nut"])
    # baski yuzleri (ayri parcalar): ic yuz yan duvarlarin +y ucuna (4 x M2), dis plaka ayagiyla kopruye (2 x M2)
    hub_r = tp["horn"]["hub_d"] / 2.0 + 2.0 * F["clear"]
    tin = box(px - xw - w, px + xw + w, yti0, yti1, zb1, ztop)
    tin = _cut(tin, [ycyl(hub_r, px, pz, yti0 - 1.0, yti1 + 1.0)] +
               [ycyl(F["m2_clear_d"] / 2.0, x, z, yti0 - 1.0, yti1 + 1.0) for x, z in wscr])
    m._add("neck", "head_pitch_thrust_in", tin, "bracket")
    tout = box(px - hw, px + hw, yto0, yto1, zb1, ztop).fuse(
        box(px - hw, px + hw, yto1 - 0.01, yfo1, zb1, zb1 + N["thrust_out_foot_t"]))
    tout = _cut(tout, [ycyl(N["thrust_hole_r"], px, pz, yto0 - 1.0, yto1 + 1.0)] +
                [zcyl(F["m2_clear_d"] / 2.0, x, y, zb1 - 1.0, zb1 + N["thrust_out_foot_t"] + 1.0) for x, y in oscr])
    m._add("neck", "head_pitch_thrust_out", tout, "bracket")
    _metal(m, "neck", "head_pitch_thrust_screws", "y", wscr, F["m2_pilot_d"] / 2.0 - 0.05,
           yti0 - N["thrust_in_screw_depth"] + 0.5, yti1 - 0.01, G["m2_screw"])
    _metal(m, "neck", "head_pitch_thrust_out_screws", "z", oscr, F["m2_pilot_d"] / 2.0 - 0.05,
           zb1 - N["thrust_out_screw_depth"] + 0.5, zb1 + N["thrust_out_foot_t"] - 0.01, G["m2_screw"])
    # cene/alin tamponu (TPU 95A): govde diline gecme; carpisma kutusu = yastik kutusu (MJCF geom "neck_bumper")
    pad = box(bx0, bx1, by0, by1, bz0, bz1).cut(box(bx0 - 1.0, bx0 + tg[0], -tg[1], tg[1], tg[2], tg[3]))
    m._add("neck", "neck_bumper", pad, "pad", None,
           (V((bx0 + bx1) / 2.0, (by0 + by1) / 2.0, (bz0 + bz1) / 2.0),
            ((bx1 - bx0) / 2.0, (by1 - by0) / 2.0, (bz1 - bz0) / 2.0), 0.0))
    sp = m.servo_parts("head_pitch", horn_toward=H["horn_toward"]["head_pitch"])
    m._add("neck", "head_pitch_servo", sp["body"], "servo", m.servo_mass("head_pitch"))


def _tilt(m, sv, H, hz):
    """Kafa egim cercevesi (head_tilt, PETG, tek parca L): head_pitch horn'unu gomen rotor diski (horn tarafi, baski
    yuzleri arasinda doner), ondan yukari kol ve head_yaw horn'unu ustten gomen yatay tabla (yaw horn kollari y
    boyunca; tablanin -y kismi yaw kablo demetinin donme yolunun (r 18.25) icinde kalir)."""
    p, T, F = m.p, H["tilt"], H["fit"]
    tp, ty = sv["head_pitch"], sv["head_yaw"]
    px, _, pz = p["j_head_pitch"]
    yx, yy, _ = p["j_head_yaw"]
    hf = p["servo_face"]["head_pitch"][1]
    yzf = p["servo_face"]["head_yaw"][2]                       # yaw horn dis yuzu (asagi bakar)
    yr0 = hf - tp["horn"]["arm_t"] - F["pocket_clear"]
    yr1 = hf + T["rotor_t"]
    zc1 = yzf + ty["horn"]["arm_t"] + F["pocket_clear"]
    zc0 = yzf - T["cross_t"]
    xfp = _horn_xf(m, "head_pitch", H["horn_toward"]["head_pitch"])
    xfy = _horn_xf(m, "head_yaw", H["horn_toward"]["head_yaw"])
    R = T["rotor_r"]
    grow = F["pocket_clear"] + F["pocket_wall"]
    rotor = ycyl(R, px, pz, yr0, yr1).fuse(_capture(tp, xfp, yr0 - hf, yr1 - hf, grow))
    hx = T["cross_neck_half_x"]
    up = prism_xz([(px - R, pz), (px + R, pz), (px + hx, zc0 + 0.01), (px - hx, zc0 + 0.01)], yr0, yr1)
    rcb = ty["horn"]["arm_r"] + ty["horn"]["arm_tip_r"] + grow
    cross = zcyl(rcb, yx, yy, zc0, zc1).common(box(px - R, px + R, yy - rcb, yy + rcb, zc0, zc1))
    cross = cross.fuse(box(yx - hx, yx + hx, yy, yr1, zc0, zc1))
    tilt = rotor.fuse([up, cross]).removeSplitter()
    cut = _horn_seat(tp, xfp, F, yr1 - hf) + _horn_seat(ty, xfy, F, yzf - zc0)
    cut += _harness_cut(hz, "head_tilt", F["harness_clear"])
    m._add("head_tilt", "head_tilt_bracket", _cut(tilt, cut), "bracket")
    for key in ("head_pitch", "head_yaw"):
        sp = m.servo_parts(key, horn_toward=H["horn_toward"][key])
        m._add("head_tilt", key + "_horn", sp["horn"], "horn", m.horn_mass(key))


def _turn(m, sv, H, hz):
    """Kafa donus cercevesi (head_turn, PETG): yaw MG90S kulak cercevesi (kulaklar ustten vidali), head_tilt tablasi
    uzerinde 0.3 mm bosluklu baski halkasi (yaw eksenel/devrilme yuku), arka plaka (roll horn'unu gomer, kollar y
    boyunca) ve on plaka + roll pimi (kafa cercevesinin on yatagina; roll mili egilme almaz)."""
    p, U, F = m.p, H["turn"], H["fit"]
    ty, tr = sv["head_yaw"], sv["head_roll"]
    yx, yy, _ = p["j_head_yaw"]
    rz = p["j_head_roll"][2]
    dy = servos.datums(ty)
    yzf = p["servo_face"]["head_yaw"][2]                       # yaw: dunya z = yzf - Y, y = yy - Z, x = yx - X
    rxf = p["servo_face"]["head_roll"][0]                      # roll: dunya x = rxf + Y, y = -Z, z = rz - X
    w2 = dy["half_w"]
    zc1 = yzf + ty["horn"]["arm_t"] + F["pocket_clear"]        # head_tilt tablasi ustu
    z_tower = yzf - dy["tower_top"]
    ztf1 = yzf - dy["tab"][1]                                  # kulaklarin horn tarafi yuzu
    ztf0 = ztf1 - U["frame_t"]
    yt0, yt1 = yy - dy["tab_z"][1] - U["frame_margin"], yy - dy["tab_z"][0] + U["frame_margin"]
    yw0, yw1 = yy - dy["far_end_z"] - F["pocket_clear"], yy - dy["near_end_z"] + F["pocket_clear"]
    xfr0, xfr1 = yx - w2 - F["pocket_clear"] - U["frame_side"], yx + w2 + F["pocket_clear"] + U["frame_side"]
    ring = zcyl(U["ring_r"][1], yx, yy, zc1 + F["gap"], z_tower - F["clear"]).cut(
        zcyl(U["ring_r"][0], yx, yy, zc1, z_tower))
    rm = sum(U["ring_r"]) / 2.0
    posts = [zcyl(U["post_r"], yx + rm * math.cos(math.radians(a)), yy + rm * math.sin(math.radians(a)),
                  z_tower - F["clear"] - 0.01, ztf0 + 0.01) for a in U["post_deg"]]
    tframe = box(xfr0, xfr1, yt0, yt1, ztf0, ztf1)
    xr0, xr1 = rxf - tr["horn"]["arm_t"] - F["pocket_clear"], rxf + U["rear_t"]
    xfrr = _horn_xf(m, "head_roll", H["horn_toward"]["head_roll"])
    grow = F["pocket_clear"] + F["pocket_wall"]
    rear = _capture(tr, xfrr, xr0 - rxf, xr1 - rxf, grow).fuse(box(xr0, xr1, U["bar_y"][0], U["bar_y"][1], ztf0, rz))
    bar = box(xr1 - 0.01, xfr0 + 0.01, U["bar_y"][0], U["bar_y"][1], ztf0, ztf1)
    xf0 = yx + w2 + U["front_gap"]
    xf1 = xf0 + U["front_t"]
    front = xcyl(U["front_r"], 0.0, rz, xf0, xf1).fuse(box(xf0, xf1, -U["front_r"], U["front_r"], ztf0, rz))
    pin = xcyl(U["pin_r"], 0.0, rz, xf1 - 0.01, U["pin_x1"])
    turn = tframe.fuse([ring, rear, bar, front, pin] + posts).removeSplitter()
    holes = [(yx, yy - z) for _, z in dy["tab_holes"]]
    cut = [box(yx - w2 - F["pocket_clear"], yx + w2 + F["pocket_clear"], yw0, yw1, ztf0 - 1.0, ztf1 + 1.0)]
    cut += [zcyl(F["m2_pilot_d"] / 2.0, x, y, ztf0 - 0.01, ztf1 + 0.01) for x, y in holes]
    cut += _horn_seat(tr, xfrr, F, xr1 - rxf)
    cut += _harness_cut(hz, "head_turn", F["harness_clear"])
    m._add("head_turn", "head_turn_bracket", _cut(turn, cut), "bracket")
    sp = m.servo_parts("head_yaw", horn_toward=H["horn_toward"]["head_yaw"])
    m._add("head_turn", "head_yaw_servo", sp["body"], "servo", m.servo_mass("head_yaw"))
    sp = m.servo_parts("head_roll", horn_toward=H["horn_toward"]["head_roll"])
    m._add("head_turn", "head_roll_horn", sp["horn"], "horn", m.horn_mass("head_roll"))
    return {"pin_x": (xf1, U["pin_x1"]), "pin_r": U["pin_r"]}


def _head(m, sv, H, hz, turn):
    """Kafa (head): sabit etli PLA kaput (alti acik kubbe) + turuncu ust gaga; PETG kafa cercevesi: halka tabla, roll
    MG90S kulak plakasi, gaga SG90 kulak plakasi, roll pimi on yatagi ve cene blogu (tampon temas yuzu); gaga sag
    pim yatagi (ayri parca, kaput duvarina vidali)."""
    p, S, R_, F = m.p, H["shell"], H["frame"], H["fit"]
    tr, ts = sv["head_roll"], sv["mouth"]
    rz = p["j_head_roll"][2]
    mx, _, mz = p["j_mouth"]
    dr, dm = servos.datums(tr), servos.datums(ts)
    rxf = p["servo_face"]["head_roll"][0]                      # roll: x = rxf + Y, y = -Z, z = rz - X
    myf = p["servo_face"]["mouth"][1]                          # gaga: y = myf + Y, x = mx + Z, z = mz - X
    w, W2 = p["wall"], p["head_w"] / 2.0
    hx0, hx1 = S["x"]
    hz0, hz1 = S["z"]
    Rs = S["r"]
    # kaput: sabit et (dis/ic yuvarlatilmis kutu esmerkezli), alti acik
    outer = rbox(hx0, hx1, -W2, W2, hz0, hz1, Rs, Rs)
    hood = outer.cut(rbox(hx0 + w, hx1 - w, -W2 + w, W2 - w, hz0 - 1.0, hz1 - w, Rs - w, Rs - w))
    J = H["jaw"]
    hood_cut = [ycyl(J["hub_r"] + J["wall_hole_gap"], mx, mz, W2 - w - 1.0, W2 + 1.0),
                ycyl(J["pin_r"] + J["wall_hole_gap"], mx, mz, -W2 - 1.0, -W2 + w + 1.0)]
    # cerceve vidalari (kaput duvarindan halka tablanin kenarina, 4 x M2) ve sag pim yatagi vidalari
    dz = (R_["deck_z"][0] + R_["deck_z"][1]) / 2.0
    fscr_y = [(x, sy) for x in R_["screw_x"] for sy in (1, -1)]
    span = lambda sy, a, b: tuple(sorted((sy * a, sy * b)))
    hood_cut += [ycyl(F["m2_clear_d"] / 2.0, x, dz, *span(sy, W2 - w - 1.0, W2 + 1.0)) for x, sy in fscr_y]
    bscr = [(mx + s * J["bushing_screw_dx"], mz) for s in (1, -1)]
    hood_cut += [ycyl(F["m2_clear_d"] / 2.0, x, z, -W2 - 1.0, -W2 + w + 1.0) for x, z in bscr]
    m._add("head", "head_hood", _cut(hood, hood_cut), "shell")
    # ust gaga (turuncu): ust + yanlar + uc, alti ve arkasi acik; arka ucu kaputun dis yuzeyine oturur
    bx0, bx1, bw, bz0, bz1, br, brt = (S["bill"][k] for k in ("x0", "x1", "half_w", "z0", "z1", "r", "r_top"))
    bill = rbox(bx0, bx1, -bw, bw, bz0, bz1, br, brt).cut(
        rbox(bx0 - 1.0, bx1 - w, -bw + w, bw - w, bz0 - 1.0, bz1 - w, br - w, max(brt - w, 0.0)))
    m._add("head", "upper_bill", bill.cut(outer), "trim")

    # kafa cercevesi (PETG): halka tabla (head_turn + head_tilt gecis acikligi), roll kulak plakasi, gaga kulak
    # plakasi, on yatak duvari (roll pimi), cene blogu
    d0, d1 = R_["deck_z"]
    gap = R_["wall_gap"]
    yin = W2 - w - gap                                         # kaput ic duvari (duz yan bolge)
    ox0, ox1, oy0, oy1 = R_["opening"]
    qx0, qx1, qy0, qy1 = R_["deck_outer"]
    b = R_["arm_w"] / 2.0
    deck = box(qx0, qx1, qy0, qy1, d0, d1).cut(box(ox0, ox1, oy0, oy1, d0 - 1.0, d1 + 1.0))
    ym_out = myf + dm["tab"][1] + R_["cradle_t"]               # gaga kulak plakasinin dis yuzu
    deck = deck.fuse([box(x - b, x + b, qy1 - 0.01, yin, d0, d1) for x in R_["screw_x"]] +
                     [box(x - b, x + b, qy1 - 0.01, ym_out, d0, d1) for x in R_["mouth_arm_x"]] +
                     [box(x - b, x + b, -yin, qy0 + 0.01, d0, d1) for x in R_["screw_x"]])
    # roll MG90S: kulaklarin horn tarafinda plaka (servo arkadan takilir, vidalar arkadan)
    rw2 = dr["half_w"]
    xr0 = rxf + dr["tab"][1]
    xr1 = xr0 + R_["cradle_t"]
    ry0, ry1 = -dr["tab_z"][1] - R_["cradle_margin"], -dr["tab_z"][0] + R_["cradle_margin"]
    rcz1 = rz + rw2 + F["pocket_clear"] + R_["cradle_side"]
    rcr = box(xr0, xr1, ry0, ry1, d0, rcz1)
    rcr = _cut(rcr, [box(xr0 - 1.0, xr1 + 1.0, -dr["far_end_z"] - F["pocket_clear"], -dr["near_end_z"] +
                         F["pocket_clear"], rz - rw2 - F["pocket_clear"], rz + rw2 + F["pocket_clear"])] +
               [xcyl(F["m2_pilot_d"] / 2.0, -z, rz, xr0 - 0.01, xr1 + 0.01) for _, z in dr["tab_holes"]])
    # gaga SG90: kulaklarin horn tarafinda plaka
    mw2 = dm["half_w"]
    ym0 = myf + dm["tab"][1]
    ym1 = ym0 + R_["cradle_t"]
    mx0, mx1 = mx + dm["tab_z"][0] - R_["cradle_margin"], mx + dm["tab_z"][1] + R_["cradle_margin"]
    mcz1 = mz + mw2 + F["pocket_clear"] + R_["cradle_side"]
    mcr = box(mx0, mx1, ym0, ym1, d0, mcz1)
    mcr = _cut(mcr, [box(mx + dm["near_end_z"] - F["pocket_clear"], mx + dm["far_end_z"] + F["pocket_clear"],
                         ym0 - 1.0, ym1 + 1.0, mz - mw2 - F["pocket_clear"], mz + mw2 + F["pocket_clear"])] +
               [ycyl(F["m2_pilot_d"] / 2.0, mx + z, mz, ym0 - 0.01, ym1 + 0.01) for _, z in dm["tab_holes"]])
    # on yatak duvari: head_turn roll piminin yatagi (ust yarisi vidali kapak, montaj icin)
    fw0 = turn["pin_x"][0] + F["clear"] * 2.0
    fw1 = fw0 + R_["bush_t"]
    fwall = xcyl(R_["bush_r"], 0.0, rz, fw0, fw1).fuse(box(fw0, fw1, -R_["bush_r"], R_["bush_r"], d0, rz))
    fwall = fwall.cut(xcyl(turn["pin_r"] + R_["bush_gap"], 0.0, rz, fw0 - 1.0, fw1 + 1.0))
    # cene blogu (tampon temas yuzu): halka tablanin onunden kafa tabanina
    cx0, cx1, chy = R_["chin"]
    chin = box(cx0, cx1, -chy, chy, p["head_bot"] + R_["chin_bot_gap"], d1).fuse(
        box(qx1 - 0.01, cx0 + 0.01, -chy, chy, d0, d1))
    # roll eksenel baski halkasi: head_turn arka plakasinin arkasinda 0.3 mm (kafa ileri itilince roll mili basi
    # almaz; geri yonde on plaka / on yatak duvari dayanir), roll kulak plakasindan iki dikmeyle
    RT = R_["roll_thrust"]
    xa1 = rxf - tr["horn"]["arm_t"] - F["pocket_clear"] - F["gap"]
    xa0 = xa1 - RT["t"]
    rring = xcyl(RT["r"][1], 0.0, rz, xa0, xa1).cut(xcyl(RT["r"][0], 0.0, rz, xa0 - 1.0, xa1 + 1.0))
    ph = RT["post"][0] / 2.0
    rposts = [box(xr1 - 0.01, xa0 + 0.01, -ph, ph, rz + s * RT["post"][1] - ph, rz + s * RT["post"][1] + ph)
              for s in (1, -1)]
    hframe = deck.fuse([rcr, mcr, fwall, chin, rring] + rposts).removeSplitter()
    hframe = _cut(hframe, [ycyl(F["m2_pilot_d"] / 2.0, x, dz, *span(sy, W2 - w - gap - R_["screw_depth"],
                                                                    W2 - w - gap + 0.01)) for x, sy in fscr_y] +
                  _harness_cut(hz, "head", F["harness_clear"]))
    m._add("head", "head_frame", hframe, "bracket")
    # gaga sag pim yatagi (ayri, kaput sag duvarina 2 x M2)
    yb0 = -W2 + w + J["bushing_wall_gap"]
    bush = box(mx - J["bushing_half"], mx + J["bushing_half"], yb0, yb0 + J["bushing_t"], mz - J["bushing_half"],
               mz + J["bushing_half"])
    bush = _cut(bush, [ycyl(J["pin_r"] + J["pin_gap"], mx, mz, yb0 - 1.0, yb0 + J["bushing_t"] + 1.0)] +
                [ycyl(F["m2_pilot_d"] / 2.0, x, z, yb0 - 0.01, yb0 + J["bushing_t"] - 1.0) for x, z in bscr])
    m._add("head", "jaw_pin_bushing", bush, "bracket")
    G = H["mass_g"]
    rs = F["m2_pilot_d"] / 2.0 - 0.05                          # vida zarfi kilavuz deligin icinde
    for sy, tag in ((1, "left"), (-1, "right")):
        _metal(m, "head", "head_frame_screws_" + tag, "y", [(x, dz) for x, s_ in fscr_y if s_ == sy], rs,
               *(span(sy, W2 - w - gap - R_["screw_depth"] + 0.5, W2 - 0.3) + (G["m2_screw"],)))
    _metal(m, "head", "jaw_pin_bushing_screws", "y", bscr, rs, -W2 + 0.3, yb0 + J["bushing_t"] - 1.5, G["m2_screw"])
    for key in ("head_roll", "mouth"):
        sp = m.servo_parts(key, horn_toward=H["horn_toward"][key])
        m._add("head", key + "_servo", sp["body"], "servo", m.servo_mass(key))


def _jaw(m, sv, H, hz):
    """Gaga (jaw): ic gobek parcasi (PETG; SG90 horn'unu iceride gomer, gobegi kaput sol duvar deliginden cikar) +
    turuncu alt gaga (PLA; disaridan iki kol, kollari birlestiren cene cubugu, sabit etli alt gaga tepsisi; sag kolda
    pim kafanin pim yatagina). Gobek ile sol kol 2 x M2 eksenel vidayla birlesir."""
    p, J, F = m.p, H["jaw"], H["fit"]
    ts = sv["mouth"]
    mx, _, mz = p["j_mouth"]
    myf = p["servo_face"]["mouth"][1]
    w, W2 = p["wall"], p["head_w"] / 2.0
    xfm = _horn_xf(m, "mouth", H["horn_toward"]["mouth"])
    yp0 = myf - ts["horn"]["arm_t"] - F["pocket_clear"]
    yp1 = W2 - w - J["plate_wall_gap"]
    yh1 = W2 + J["arm_gap"]
    grow = F["pocket_clear"] + F["pocket_wall"]
    hub = _capture(ts, xfm, yp0 - myf, yp1 - myf, grow).fuse(ycyl(J["hub_r"], mx, mz, yp1 - 0.01, yh1))
    jscr = [(mx + s * J["hub_screw_dx"], mz) for s in (1, -1)]
    hub = _cut(hub, _horn_seat(ts, xfm, F, yp1 - myf, from_inside=True) +
               [ycyl(F["m2_pilot_d"] / 2.0, x, z, yh1 - J["hub_screw_depth"], yh1 + 0.01) for x, z in jscr])
    m._add("jaw", "jaw_hub", hub, "bracket")
    # alt gaga: kollar (|y| W2+gap .. +arm_t) mil ekseninden cene cubuguna, cubuk, tepsi
    at = J["arm_t"]
    lx0, lx1, lw, lz0, lz1, lr = (J["bill"][k] for k in ("x0", "x1", "half_w", "z0", "z1", "r"))
    bar_x = lx0 + J["bar_w"] / 2.0
    bar_z = lz0 + J["bar_h"] / 2.0
    parts = []
    for sy in (1, -1):
        y0, y1 = (yh1, yh1 + at) if sy > 0 else (-yh1 - at, -yh1)
        parts.append(bar_y((mx, mz), (bar_x, bar_z), J["arm_w"] / 2.0, y0, y1))
        parts.append(ycyl(J["hub_r"], mx, mz, y0, y1))
    parts.append(box(lx0, lx0 + J["bar_w"], -yh1 - at, yh1 + at, lz0, lz0 + J["bar_h"]))
    tray = rbox(lx0, lx1, -lw, lw, lz0, lz1, lr, 0.0, J["bill"]["r_bot"])
    tray = tray.cut(rbox(lx0 + w, lx1 - w, -lw + w, lw - w, lz0 + w, lz1 + 1.0, lr - w, 0.0,
                         max(J["bill"]["r_bot"] - w, 0.0)))
    parts.append(tray)
    parts.append(ycyl(J["pin_r"], mx, mz, -yh1 - 0.01, -W2 + w + J["bushing_wall_gap"] + J["bushing_t"] -
                      J["pin_tip_gap"]))
    low = parts[0].fuse(parts[1:]).removeSplitter()
    low = _cut(low, [ycyl(F["m2_clear_d"] / 2.0, x, z, yh1 - 0.01, yh1 + at + 0.01) for x, z in jscr] +
               [ycyl(F["center_screw_d"] / 2.0, mx, mz, yh1 - 0.01, yh1 + at + 0.01)])
    m._add("jaw", "lower_bill", low, "trim")
    _metal(m, "jaw", "jaw_hub_screws", "y", jscr, F["m2_pilot_d"] / 2.0 - 0.05, yh1 - J["hub_screw_depth"] + 0.5,
           yh1 + at - 0.01, H["mass_g"]["m2_screw"])
    sp = m.servo_parts("mouth", horn_toward=H["horn_toward"]["mouth"])
    m._add("jaw", "mouth_horn", sp["horn"], "horn", m.horn_mass("mouth"))


def build_tr35(m, sv):
    """TR-35 boyun + kafa (P2c): eklemler (ID 30-34), boyun U-link'i + head_pitch besigi + baski yuzleri + tampon
    (neck), egim cercevesi (head_tilt), donus cercevesi + roll pimi (head_turn), kafa cercevesi + sabit etli kaput +
    ust gaga (head), gaga (jaw) ve link ici guzergahli kablo demetleri. Ayrintilar ve yuk hesabi: p2c-tampon.md."""
    from geom import tube
    servos.require(m, sv, ("neck_pitch",), "DS32XX", "head.tr35_b2")
    servos.require(m, sv, ("head_pitch", "head_yaw", "head_roll"), "MG90S", "head.tr35_b2")
    servos.require(m, sv, ("mouth",), "SG90", "head.tr35_b2")
    p, H = m.p, m.p["head"]
    for jn, parent, child, axis, sid in HEAD_CHAIN:
        m._joint(jn, parent, child, p["j_" + jn], axis, m.limits[jn], m.home.get(jn, 0.0), sid)
    hz = _harness(m, H)
    _neck(m, sv, H, hz)
    _tilt(m, sv, H, hz)
    turn = _turn(m, sv, H, hz)
    _head(m, sv, H, hz, turn)
    _jaw(m, sv, H, hz)
    for link, rows in hz.items():
        for name, pts, d, mass in rows:
            m._add(link, name, tube(pts, d / 2.0), "harness", mass)
    # tampon MJCF'de boyun govdesine bagli carpisma geomu (export: meta["collision_geoms"]; "neck" collision_links'e
    # eklenmez, RL getup varyanti kendi neck_collision'ini ekler)
    m.meta["collision_geoms"] = list(m.meta.get("collision_geoms", [])) + ["neck_bumper"]


GENERATORS = {"xl330_27": build_xl330, "tr35_b2": build_tr35}
