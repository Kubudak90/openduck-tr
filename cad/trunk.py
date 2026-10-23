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
# TR-35 govdesi (P2b, docs/workflow/openduck-tr/p2-interface.md + p2b-hacim.md). Butun olculer donmus arayuzden
# (params.iface.envelopes.trunk = K_T, servo zarflari, doner tabla, harness) ve params.trunk'tan turetilir.
#   sasi (PETG): trunk_base = taban + doner tabla yuvalari (6800ZZ x 4) + ES3054 kafesi (plaka + arka duvar, destek
#                uzerinde) + boyun besigi (DS cerceve) + batarya tuneli (duvarlar + tavan = buck tablasi) + raflar;
#                buck_hood = buck kaputu (izgara, buck penceresi, yan hava girisleri, batarya ve XT30 erisim agzi);
#                belly_duct = XL4015 hava kanali (uclari acik)
#   kabuk (PLA, sabit et p["wall"]): torso_shell_left/right, torso_top, deck_cover
#   elektronik: cad/electronics.py zarflari, yerlesim params.trunk.electronics
#   kablo (harness): kalca ve boyun demetlerinin govde ici kanallari, batarya ana kablosu
# ---------------------------------------------------------------------------
KT_NAMES = ("core", "low_l", "low_r", "mid_l", "mid_r", "flow_l", "flow_r", "back", "belly", "deck", "seat")


class _D(dict):
    """Nokta erisimli sozluk (datum kumesi)."""
    __getattr__ = dict.__getitem__
    __setattr__ = dict.__setitem__


def _kt(p):
    """K_T kutulari (p2-interface.md 4.3 sirasi) -> {ad: (x0, x1, y0, y1, z0, z1)}."""
    add = p["iface"]["envelopes"]["trunk"]["add"]
    out = {}
    for name, prim in zip(KT_NAMES, add):
        assert prim[0] == "box", (name, prim)
        out[name] = tuple(float(v) for v in prim[1:7])
    assert out["low_l"][2] > 0 > out["low_r"][3] and out["mid_l"][2] > 0 and out["flow_l"][2] > 0, "K_T sirasi"
    return out


def _to_world(m, key):
    """Kanonik servo cercevesi (servos.py) -> dunya noktasi (sol / orta)."""
    from geom import V, frame
    pl = frame(*m.servo_frame(key))
    return lambda cx, cy, cz: tuple(pl.multVec(V(cx, cy, cz)))


def _hexz(af, x, y, z0, z1):
    """z ekseninde altigen somun yuvasi (anahtar agzi af, duz kenarlar x'e dik)."""
    import Part
    from geom import V
    r = af / math.sqrt(3.0)
    pts = [V(x + r * math.cos(math.radians(30 + 60 * k)), y + r * math.sin(math.radians(30 + 60 * k)), z0)
           for k in range(6)]
    return Part.Face(Part.makePolygon(pts + pts[:1])).extrude(V(0, 0, z1 - z0))


def _hexy(af, x, z, y0, y1):
    """y ekseninde altigen somun yuvasi."""
    import Part
    from geom import V
    r = af / math.sqrt(3.0)
    pts = [V(x + r * math.cos(math.radians(60 * k)), y0, z + r * math.sin(math.radians(60 * k))) for k in range(6)]
    return Part.Face(Part.makePolygon(pts + pts[:1])).extrude(V(0, y1 - y0, 0))


def _fuse(parts):
    parts = [s for s in parts if s is not None]
    return parts[0].fuse(parts[1:]).removeSplitter() if len(parts) > 1 else parts[0]


def _datums(m):
    """Donmus arayuzden ve servo zarflarindan turetilen butun govde olculeri."""
    from collections import OrderedDict
    import electronics as E
    p, T = m.p, m.p["trunk"]
    S = T["struct"]
    kt = _kt(p)
    d = _D(kt=kt, S=S, fit=p["fit"], w=p["wall"])
    core, back, low, mid, flow = kt["core"], kt["back"], kt["low_l"], kt["mid_l"], kt["flow_l"]
    d.xb, d.xf, d.xm0, d.xm1, d.xfl = back[0], core[1], mid[0], mid[1], flow[0]
    d.yc, d.ylo, d.ymid, d.ybk = core[3], low[3], mid[3], back[3]
    d.zf, d.zlo, d.zfl, d.ztop = core[4], low[5], flow[5], core[5]
    d.zft = d.zf + S["floor_t"]
    # kalca yaw doner tablasi (servos.turntable_stack)
    tt = servos.turntable_stack(p["turntable"])
    d.tt, d.brg = tt, tt["bearing_rec"]
    d.yx, d.yy = p["j_hip_yaw"][0], p["j_hip_yaw"][1]
    d.hr = tt["housing_r"]
    # ES3054 (kalca yaw), kanonik datumlar -> dunya (sol)
    es = m.servo_type("hip_yaw")
    de, we = servos.datums(es), _to_world(m, "hip_yaw")
    zt = sorted(we(0, y, 0)[2] for y in de["tab"])
    d.es_tab_z = (zt[0], zt[1])
    d.es_case_z = tuple(sorted((we(0, de["case_top"], 0)[2], we(0, de["rear_face"], 0)[2])))
    d.es_x = tuple(sorted((we(0, 0, de["near_end_z"])[0], we(0, 0, de["far_end_z"])[0])))
    d.es_tab_x = tuple(sorted(we(0, 0, z)[0] for z in de["tab_z"]))
    d.es_holes = [we(hx, 0, hz)[:2] for hx, hz in de["tab_holes"]]
    d.es_hw = de["half_w"]
    # boyun DS (govde trunk'ta, horn -y)
    ds = m.servo_type("neck_pitch")
    dn, wn = servos.datums(ds), _to_world(m, "neck_pitch")
    d.nb_x = tuple(sorted((wn(-dn["half_w"], 0, 0)[0], wn(dn["half_w"], 0, 0)[0])))
    d.nb_y = tuple(sorted((wn(0, dn["rear_face"], 0)[1], wn(0, dn["case_top"], 0)[1])))
    d.nb_z = tuple(sorted((wn(0, 0, dn["near_end_z"])[2], wn(0, 0, dn["far_end_z"])[2])))
    d.n_tab_y = tuple(sorted(wn(0, y, 0)[1] for y in dn["tab"]))
    d.n_tab_z = tuple(sorted(wn(0, 0, z)[2] for z in dn["tab_z"]))
    d.n_holes = [(wn(hx, 0, hz)[0], wn(hx, 0, hz)[2]) for hx, hz in dn["tab_holes"]]
    d.n_axis = (p["j_neck_pitch"][0], p["j_neck_pitch"][2])
    # kablo demeti giris noktalari (donmus params.harness)
    hc = p["harness"]["crossings"]
    d.hip_in, d.hip_d = tuple(hc["hip_yaw"]["in"][0]), hc["hip_yaw"]["d"]
    d.neck_in, d.neck_d = tuple(hc["neck_pitch"]["in"][0]), hc["neck_pitch"]["d"]
    # elektronik yerlesim kutulari
    d.el = OrderedDict((n, E.placed_box(n, pl)) for n, pl in T["electronics"].items())
    bat = d.el["battery_4s_lipo"]
    d.bat = bat
    d.by_in = bat[3] + S["bay_gap"]
    d.by_out = d.by_in + S["bay_wall_t"]
    d.z_roof0 = bat[5] + S["strap_t"]
    d.z_roof1 = d.z_roof0 + S["roof_t"]
    d.x_tun0 = d.xb + S["hood_t"]                   # tunel/raf arka ucu = kaput arka duvari ic yuzu
    # kafes (ES3054) ve destek
    d.ycage = d.yc - d.w - S["cage_side_gap"]
    d.z_br1 = tt["upper"][1]
    d.z_br0 = d.z_br1 - S["bracket_t"]
    d.cr = tuple(S["cage_rear_x"])
    d.cp_z = (d.es_tab_z[0] - S["cage_plate_t"], d.es_tab_z[0])
    hx_sorted = sorted(h[0] for h in d.es_holes)
    d.cp_x = (hx_sorted[0] - d.fit["nut_af"] / 2.0 - S["cage_nut_edge"],
              hx_sorted[-1] + d.fit["nut_af"] / 2.0 + S["cage_nut_edge"])
    # boyun besigi: tab'larin horn tarafinda plaka
    d.fy = (d.n_tab_y[0] - p["plate_t"], d.n_tab_y[0])
    d.fx = (d.nb_x[0] - S["frame_bar"], d.nb_x[1] + S["frame_bar"])
    d.ftop = d.ztop - d.w
    return d


def _base(m, d, sv):
    """trunk_base (PETG): taban, doner tabla yuvalari, kafes destegi, boyun besigi, batarya tuneli, raflar, dikmeler."""
    from geom import box, ycyl, zcyl
    S, f = d.S, d.fit
    b = d.brg
    parts = [box(d.xb, d.xm0, -d.ybk, d.ybk, d.zf, d.zft), box(d.xm0, d.xf, -d.ylo, d.ylo, d.zf, d.zft)]
    for sy in (1, -1):
        parts.append(zcyl(d.hr, d.yx, sy * d.yy, d.zf, d.z_br1))
    parts.append(box(d.cr[0], d.yx, -d.ycage, d.ycage, d.z_br0, d.z_br1))          # kafes destegi
    # batarya on dayanaklari: orta (kafes destegine kadar) ve sag yan (kablo demeti altinda)
    bx1 = d.bat[1]
    hw = S["stop_center_hw"]
    parts.append(box(bx1, bx1 + S["stop_t"], -hw, hw, d.zft, d.z_br0))
    for y0, y1, z1 in S["side_stops"]:
        parts.append(box(bx1, bx1 + S["stop_t"], y0, y1, d.zft, z1))
    # batarya tuneli: duvarlar + tavan (buck tablasi, 40 mm kopru baski)
    for sy in (1, -1):
        y0, y1 = sorted((sy * d.by_in, sy * d.by_out))
        parts.append(box(d.x_tun0, bx1, y0, y1, d.zft, d.z_roof0))
    parts.append(box(d.x_tun0, bx1, -S["roof_hw"], S["roof_hw"], d.z_roof0, d.z_roof1))
    for bxs in S["shelves"]:                                    # yan bolme raflari (alarm, kirpici)
        parts.append(box(*bxs))
    for bxs in S["posts"]:                                      # bolucu karti dikmeleri (tavanda)
        parts.append(box(*bxs))
    # boyun besigi (DS govdesi trunk'ta): tab'larin horn tarafinda cerceve plaka + yan duvarlar + yuvaya kopru
    fx0, fx1 = d.fx
    fy0, fy1 = d.fy
    parts.append(box(fx0, fx1, fy0, fy1, d.zft, d.ftop))
    c = S["fit_servo"]
    parts.append(box(fx0, d.nb_x[0] - c, -d.yc, fy0, d.zft, d.ftop))
    parts.append(box(d.nb_x[1] + c, fx1, -d.yc, fy0, d.zft, d.ftop))
    parts.append(box(S["web_x0"], fx0, fy0, fy1, d.zf, d.z_br1))
    # XL4015 aski dikmeleri (taban altinda, gobek kanalinda)
    xl = d.el["xl4015"]
    for hx, hy in S["xl4015_posts"]:
        parts.append(zcyl(S["post_r"], hx, hy, xl[5], d.zf))
    base = _fuse(parts)
    cut = []
    for sy in (1, -1):                                          # 6800ZZ yuvalari + ag + rotor bolgesi
        y = sy * d.yy
        cut.append(zcyl(b["D"] / 2.0, d.yx, y, d.zf - 1, d.tt["lower"][1]))
        cut.append(zcyl(b["D"] / 2.0 - S["bearing_shoulder"], d.yx, y, d.zf - 1, d.z_br1 + 1))
        cut.append(zcyl(b["D"] / 2.0, d.yx, y, d.tt["upper"][0], d.z_br1 + 1))
        hx, hy, _ = d.hip_in                                     # kalca demeti taban agzi
        cut.append(zcyl(d.hip_d / 2.0 + S["harness_gap"], hx, sy * hy, d.zf - 1, d.zft + 1))
        sx0, sx1 = S["strap_x"]
        y0, y1 = sorted((sy * d.by_out, sy * (d.by_out + S["strap_t"])))
        cut.append(box(sx0, sx1, y0, y1, d.zf - 1, d.zft + 1))   # kayis yuvalari (taban) ve duvar ustu centigi
        y0, y1 = sorted((sy * (d.by_in - 0.01), sy * (d.by_out + 0.01)))
        cut.append(box(sx0, sx1, y0, y1, d.bat[5], d.z_roof0 + 0.01))
    cut.append(box(d.nb_x[0] - c, d.nb_x[1] + c, fy0 - 1, fy1 + 1, d.nb_z[0] - c, d.nb_z[1] + c))   # govde penceresi
    for hx, hz in d.n_holes:                                     # boyun tab vidalari: delik + somun yuvasi (-y yuzu)
        cut.append(ycyl(f["clear_d"] / 2.0, hx, hz, fy0 - 1, fy1 + 1))
        cut.append(_hexy(f["nut_af"], hx, hz, fy0 - 0.01, fy0 + f["nut_t"]))
    for hx, hz, r in S["frame_wire_holes"]:                      # besik altindan gecen demetler
        cut.append(ycyl(r, hx, hz, fy0 - 1, fy1 + 1))
    for n, bx in d.el.items():                                   # taban cebi: kenar ustu kartlar / anahtar cercevesi
        if bx[4] < d.zft - 1e-6:
            g = S["pocket_gap"]
            cut.append(box(bx[0] - g, bx[1] + g, bx[2] - g, bx[3] + g, bx[4] - g, d.zft + 0.01))
    for bxs in S["floor_notches"]:
        cut.append(box(*bxs))
    cut += d.kt_cut                                              # rotor bolgeleri (K_T kesicileri, donmus)
    return base.cut(_fuse(cut))


def _cage(m, d):
    """ES3054 kafesi (trunk_base'in parcasi): iki servo icin tab plakasi (4 x M3 somun yuvasi, alt yuz) + kafes
    destegine oturan arka duvar (rotor bolgesinin arkasinda). Plaka 30 mm konsol: baskida plaka altina destek."""
    from geom import box, zcyl
    S, f = d.S, d.fit
    g = S["fit_es"]
    plate = box(d.cp_x[0], d.cp_x[1], -d.ycage, d.ycage, d.cp_z[0], d.cp_z[1])
    rear = box(d.cr[0], d.cr[1], -d.ycage, d.ycage, d.z_br1, d.cp_z[0])
    cage = plate.fuse(rear).removeSplitter()
    cut = []
    for sy in (1, -1):
        y0, y1 = sorted((sy * (d.yy - d.es_hw - g), sy * (d.yy + d.es_hw + g)))
        cut.append(box(d.es_x[0] - g, d.es_x[1] + g, y0, y1, d.es_case_z[0] - g, d.cp_z[1] + 1))
        for hx, hy in d.es_holes:
            cut.append(zcyl(f["clear_d"] / 2.0, hx, sy * hy, d.cp_z[0] - 1, d.cp_z[1] + 1))
            cut.append(_hexz(f["nut_af"], hx, sy * hy, d.cp_z[0] - 0.01, d.cp_z[0] + f["nut_t"]))
    return cage.cut(_fuse(cut))


def _hood(m, d):
    """buck_hood (PETG): ust izgara, arka duvar (batarya + XT30 agzi, buck penceresi), arka kutu yan duvarlari (hava
    girisleri), on dikmeler. Buck'a en yakin PLA >= 5 mm (spec 3.1)."""
    from geom import box
    S = d.S
    t = S["hood_t"]
    H = S["hood"]
    x1 = H["x1"]
    yh = d.ybk - t
    yt = H["top_hw"]
    parts = [box(d.xb, d.xm0, -d.ybk, d.ybk, d.ztop - t, d.ztop), box(d.xm0, x1, -yt, yt, d.ztop - t, d.ztop),
             box(d.xb, d.xb + t, -d.ybk, d.ybk, d.zft, d.ztop - t)]
    for sy in (1, -1):
        y0, y1 = sorted((sy * yh, sy * d.ybk))
        parts.append(box(d.xb + t, d.xm0, y0, y1, d.zft, d.ztop - t))
    for px0, px1, py0, py1 in H["posts"]:
        for sy in (1, -1):
            y0, y1 = sorted((sy * py0, sy * py1))
            parts.append(box(px0, px1, y0, y1, d.z_roof1, d.ztop - t))
    hood = _fuse(parts)
    cut = [box(*bx) for bx in H["ports"]]                       # batarya + XT30 erisim agzi
    gx0, gx1, gy, gw, gp = H["grille"]                           # ust izgara (y boyunca yariklar)
    x = gx0
    while x + gw <= gx1 + 1e-6:
        cut.append(box(x, x + gw, -gy, gy, d.ztop - t - 1, d.ztop + 1))
        x += gp
    if H.get("intake_back"):                                     # arka duvar yariklari (pencere yoksa)
        iz0, iz1, iy, iw, ip = H["intake_back"]
        y = -iy
        while y + iw <= iy + 1e-6:
            cut.append(box(d.xb - 1, d.xb + t + 1, y, y + iw, iz0, iz1))
            y += ip
    sx0, sx1, sz0, sz1, sw, sp = H["intake_side"]               # yan duvar hava girisi
    x = sx0
    while x + sw <= sx1 + 1e-6:
        for sy in (1, -1):
            y0, y1 = sorted((sy * (yh - 1), sy * (d.ybk + 1)))
            cut.append(box(x, x + sw, y0, y1, sz0, sz1))
        x += sp
    return hood.cut(_fuse(cut))


def _belly(m, d):
    """belly_duct (PETG): XL4015 hava kanali (alt + yan duvarlar, on/arka uclar acik)."""
    from geom import box
    x0, x1, y0, y1, z0, z1 = d.kt["belly"]
    t = d.S["hood_t"]
    parts = [box(x0, x1, y0, y1, z0, z0 + t)]
    for a, b_ in ((y0, y0 + t), (y1 - t, y1)):
        parts.append(box(x0, x1, a, b_, z0 + t, z1))
    return _fuse(parts)


def _shell(m, d, sy):
    """torso_shell_left/right (PLA, sabit et w): alt yan duvar, on yuz + cene, omuzlar, cekirdek yan duvari (rotor
    bolgesi acik), orta yan duvar + basamak + ust + arka yuz. sy = +1 sol, -1 sag."""
    from geom import box, zcyl
    S, w = d.S, d.w
    yh = S["hood"]["top_hw"]                 # orta ust panel kaputun kenarindan baslar (buck'a >= 5 mm)
    yback = S["shell_back_y0"]               # arka yuz: buck yan yuzune >= 5 mm

    def yb(a, b_):
        return tuple(sorted((sy * a, sy * b_)))

    zchin = d.nb_z[0] - S["chin_gap"]
    P = [box(d.xm0, d.xfl, *(yb(d.ylo - w, d.ylo) + (d.zft, d.zlo))),          # alt yan duvar
         box(d.xfl, d.xf - w, *(yb(d.ylo - w, d.ylo) + (d.zft, d.zfl))),       # on alt yan duvar
         box(d.xf - w, d.xf, *(yb(d.yc, d.ylo) + (d.zft, d.zfl))),             # on yuz
         box(d.xf - w, d.xf, *(yb(0.0, d.yc) + (d.zft, zchin))),               # cene yuzu
         box(d.xm1, d.xfl, *(yb(d.yc, d.ylo) + (d.zlo - w, d.zlo))),            # omuz
         box(d.xfl, d.xf - w, *(yb(d.yc, d.ylo - w) + (d.zfl - w, d.zfl))),     # on omuz
         box(d.xm1, d.xfl, *(yb(d.yc - w, d.yc) + (d.zlo, d.ztop - w))),        # cekirdek yan duvari
         box(d.xm0, d.xm1, *(yb(d.ymid - w, d.ymid) + (d.zlo, d.ztop - w))),    # orta yan duvar
         box(d.xm0, d.xm1, *(yb(d.ylo - w, d.ymid) + (d.zlo, d.zlo + w))),     # basamak (alt duvar -> orta duvar)
         box(d.xm0, d.xm1, *(yb(yh, d.ymid) + (d.ztop - w, d.ztop))),          # orta ust
         box(d.xm0, d.xm0 + w, *(yb(yback, d.ylo) + (d.zft + S["shell_back_z0"], d.zlo))),   # arka yuz (alt)
         box(d.xm0, d.xm0 + w, *(yb(yback, d.ymid) + (d.zlo, d.ztop - w)))]     # arka yuz (orta)
    sh = _fuse(P)
    cut = list(d.kt_cut)                                           # rotor bolgeleri (K_T kesicileri)
    for yy in (d.yy, -d.yy):                                       # doner tabla yuvalari omuzdan gecer
        cut.append(zcyl(d.hr + S["housing_gap"], d.yx, yy, d.zft - 1, d.zlo + 1))
    for bx in d.S["shell_cuts"]["left" if sy > 0 else "right"]:
        cut.append(box(*bx))
    for hx, hy, r, z0, z1 in d.S["shell_holes"]["left" if sy > 0 else "right"]:
        cut.append(zcyl(r, hx, hy, z0, z1))
    vx0, vx1, vz0, vz1, vw, vp = S["shell_vents"]                  # yan bolme havalandirma yariklari
    x = vx0
    while x + vw <= vx1 + 1e-6:
        cut.append(box(x, x + vw, *(yb(d.ylo - w - 1, d.ylo + 1) + (vz0, vz1))))
        x += vp
    sh = sh.cut(_fuse(cut))
    if sy > 0:                                                     # anahtar paneli (girintili, tabana kadar)
        px0, px1, py0, py1, pz0, pz1 = S["switch_panel"]
        sw_ = d.el.get("main_switch")
        panel = box(px0, px1, py0, py1, pz0, pz1)
        if sw_ is not None:
            panel = panel.cut(box(px0 - 1, px1 + 1, sw_[2] + 1.0 - 0.1, sw_[3] - 1.0 + 0.1,
                                  sw_[4] + 1.0 - 0.1, sw_[5] - 1.0 + 0.1))
        sh = sh.fuse(panel).removeSplitter()
    return sh


def _top(m, d):
    """torso_top (PLA): kafesler, boyun servosu ve bolucu kart uzerinde ust kapak."""
    from geom import box
    S, w = d.S, d.w
    x0 = S["hood"]["x1"]
    yh = S["hood"]["top_hw"]
    return _fuse([box(x0, d.xf - w, -d.yc, d.yc, d.ztop - w, d.ztop), box(x0, d.xm1, -yh, yh, d.ztop - w, d.ztop)])


def _deck(m, d):
    """deck_cover (PLA): ESP32 ustunde kapak; on/arka uclari acik (kart 25.4, K_T 26), sol uc duvarinda USB agzi."""
    from geom import box
    x0, x1, y0, y1, z0, z1 = d.kt["deck"]
    w = d.w
    dk = _fuse([box(x0, x1, y0, y1, z1 - w, z1), box(x0, x1, y0, y0 + w, z0, z1 - w),
                box(x0, x1, y1 - w, y1, z0, z1 - w)])
    ux, uz, uw, uh = d.S["usb_port"]
    return dk.cut(box(ux - uw / 2.0, ux + uw / 2.0, y1 - w - 1, y1 + 1, uz - uh / 2.0, uz + uh / 2.0))


def _screw(d, axis, x, y, z0, length, head_dir):
    """M3 SHCS (bas d 5.5 x 3) + somun (5.5 AF x 2.4): axis 'z'/'y', govde z0'dan head_dir'in tersine length;
    bas z0'dan head_dir yonunde."""
    from geom import ycyl, zcyl
    hr, ht = 2.75, 3.0
    if axis == "z":
        a, b_ = sorted((z0, z0 - head_dir * length))
        sh = zcyl(1.5, x, y, a, b_)
        h0, h1 = sorted((z0, z0 + head_dir * ht))
        return sh.fuse(zcyl(hr, x, y, h0, h1)).removeSplitter()
    a, b_ = sorted((z0, z0 - head_dir * length))
    sh = ycyl(1.5, x, y, a, b_)
    h0, h1 = sorted((z0, z0 + head_dir * ht))
    return sh.fuse(ycyl(hr, x, y, h0, h1)).removeSplitter()


def _nut(d, axis, x, y, a0, a1):
    af = 5.5
    return _hexz(af, x, y, a0, a1) if axis == "z" else _hexy(af, x, y, a0, a1)


def _fasteners(m, d):
    """Modellenen M3 baglantilari: ES3054 tab vidalari (4 x M3 x es_screw_len), boyun DS tab vidalari (4 x M3 x
    neck_screw_len) + somun yuvalarindaki somunlar. Kutle: params.trunk.struct.fastener_mass_g (govde vida butcesi;
    modellenmeyen kabuk/kaput baglantilari dahil)."""
    out = []
    for sy in (1, -1):
        for hx, hy in d.es_holes:
            out.append(_screw(d, "z", hx, sy * hy, d.es_tab_z[1], d.S["es_screw_len"], 1))
            out.append(_nut(d, "z", hx, sy * hy, d.cp_z[0] + 0.1, d.cp_z[0] + 0.1 + 2.4))
    fy0, fy1 = d.fy
    for hx, hz in d.n_holes:
        out.append(_screw(d, "y", hx, hz, d.n_tab_y[1], d.S["neck_screw_len"], 1))
        out.append(_nut(d, "y", hx, hz, fy0 + 0.1, fy0 + 0.1 + 2.4))
    import Part
    return Part.makeCompound(out)


def _wiring(m, d):
    """Govde ici kablo demetleri (kind harness, params.trunk.wiring): ad -> (noktalar, cap). Kutle: boy x alan x
    params.harness.g_per_mm3; toplam govde kablo butcesine (electronics.WIRING_G['trunk']) olceklenir (modellenmeyen
    yerel kablolar dahil)."""
    from geom import tube
    W = m.p["trunk"]["wiring"]
    g = m.p["harness"]["g_per_mm3"]
    out = []
    for name, spec in W["bundles"].items():
        pts = [tuple(q) for q in spec["pts"]]
        dd = float(spec["d"])
        L = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        out.append((name, tube(pts, dd / 2.0), L * math.pi * dd * dd / 4.0 * g / 1000.0))
    return out


def build_tr35_b2(m, sv):
    """TR-35 govdesi (P2b): sasi + ES3054 kafesi + boyun besigi (PETG), sabit etli kabuklar (PLA), butun elektronik
    gercek zarflariyla (cad/electronics.py, params.trunk.electronics), buck kaputu ve hava yolu, XL4015 kanali,
    kablo kanallari, M3 baglantilari. Paylasilan: hip_yaw_horn (bacaklar), neck_pitch_horn + idler pimi (kafa)."""
    import electronics as E
    from geom import box, envelope_solid, ycyl, zcyl
    servos.require(m, sv, ("hip_yaw",), "ES3054", "trunk.tr35_b2")
    servos.require(m, sv, ("neck_pitch",), "DS32XX", "trunk.tr35_b2")
    p, T = m.p, m.p["trunk"]
    d = _datums(m)
    d.kt_cut = [envelope_solid([c]) for c in p["iface"]["envelopes"]["trunk"].get("cut", [])]
    S = d.S
    tt, b = d.tt, d.brg
    # sabit icerik: ES3054 x 2 + 6800ZZ x 4 (yuvalar trunk_base'de), boyun DS + idler yuvasi + F623ZZ
    for side, mir in (("left", False), ("right", True)):
        sp = m.servo_parts("hip_yaw", mir, horn_toward=T["hip_yaw_horn_toward"])
        m._add("trunk", side + "_hip_yaw_servo", sp["body"], "servo", m.servo_mass("hip_yaw"))
        m.shared.setdefault("hip_yaw_horn", {})[side] = sp["horn"]
        sy = -1.0 if mir else 1.0
        for k, (z0, z1) in (("lo", tt["lower"]), ("hi", tt["upper"])):
            ring = zcyl(b["D"] / 2.0, d.yx, sy * d.yy, z0, z1).cut(zcyl(b["d"] / 2.0, d.yx, sy * d.yy, z0 - 1, z1 + 1))
            m._add("trunk", "%s_hip_yaw_bearing_%s" % (side, k), ring, "metal", b["mass_g"] / 1000.0)
    sp = m.servo_parts("neck_pitch", horn_toward=T["neck_horn_toward"])
    m._add("trunk", "neck_pitch_servo", sp["body"], "servo", m.servo_mass("neck_pitch"))
    m._add("trunk", "neck_pitch_idler_seat", sp["idler_seat"], "bracket")
    m._add("trunk", "neck_pitch_idler_bearing", sp["idler_bearing"], "metal",
           servos.BEARINGS[p["idler"]["bearing"]]["mass_g"] / 1000.0)
    m.shared["neck_pitch_horn"] = sp["horn"]
    m.shared["neck_pitch_idler_pin"] = sp["idler_pin"]
    # idler yuvasi DS arka yuzune VHB ile yapisik (donmus K_T'de yuvayi govdeye baglayan kol yok; ICR p2b-hacim.md)
    st = servos.datums(m.servo_type("neck_pitch"))["idler"]
    nx, nz = d.n_axis
    y_rear = d.nb_y[1]
    pad = ycyl(st["seat_r"], nx, nz, y_rear, y_rear + S["seat_pad_t"]).cut(
        ycyl(st["bearing_rec"]["d"] / 2.0 + 1.5, nx, nz, y_rear - 1, y_rear + 1))
    m._add("trunk", "neck_seat_vhb", pad, "chip", S["seat_pad_g"] / 1000.0)
    # yapi
    m._add("trunk", "trunk_base", _base(m, d, sv).fuse(_cage(m, d)).removeSplitter(), "bracket")
    m._add("trunk", "buck_hood", _hood(m, d), "bracket")
    m._add("trunk", "belly_duct", _belly(m, d), "bracket")
    m._add("trunk", "torso_shell_left", _shell(m, d, 1), "shell")
    m._add("trunk", "torso_shell_right", _shell(m, d, -1), "shell")
    m._add("trunk", "torso_top", _top(m, d), "shell")
    m._add("trunk", "deck_cover", _deck(m, d), "shell")
    # batarya kayisi (TPU, baskili): batarya ustu (tunel tavani alti), yanlar, taban alti (gobek bolgesi)
    sx0, sx1 = S["strap_x"]
    t = S["strap_t"]
    bat = d.bat
    strap = _fuse([box(sx0, sx1, -d.by_out - t, d.by_out + t, bat[5], bat[5] + t),
                   box(sx0, sx1, d.by_out, d.by_out + t, d.zf, bat[5]),
                   box(sx0, sx1, -d.by_out - t, -d.by_out, d.zf, bat[5]),
                   box(sx0, sx1, -d.by_out - t, d.by_out + t, d.zf - t, d.zf)])
    strap = strap.cut(box(sx0 - 1, sx1 + 1, -d.by_out, d.by_out, d.zf - 0.01, bat[5]))
    m._add("trunk", "battery_strap", strap, "pad")
    # elektronik
    for name, pl in T["electronics"].items():
        rec = E.PARTS[E.part_of(name)]
        shape = E.solid(name, pl[:3], pl[3] if len(pl) > 3 else 0.0, pl[4] if len(pl) > 4 else "xyz")
        m._add("trunk", name.replace("#", "_"), shape, rec["kind"], rec["mass_g"] / 1000.0)
    # kablo kanallari
    wires = _wiring(m, d)
    tot = sum(w_[2] for w_ in wires)
    budget = E.WIRING_G["trunk"] / 1000.0
    k = max(1.0, budget / tot) if tot > 0 else 1.0
    for name, shape, mass in wires:
        m._add("trunk", name, shape, "harness", mass * k)
    # baglanti elemanlari
    m._add("trunk", "trunk_fasteners", _fasteners(m, d), "metal", S["fastener_mass_g"] / 1000.0)
    m.meta["imu_item"] = T["imu_item"]


# baski yonu / destek notlari (TR-35 govde baski parcalari; p2b-hacim.md tablosuna yazilir, Ingilizce)
PRINT_NOTES = {
    "trunk_base": "floor on the bed; battery-tunnel roof is a 40 mm bridge; supports under the ES3054 cage plate "
                  "(z 35.7) and over the neck-frame window",
    "buck_hood": "grille face on the bed (upside down), no supports",
    "belly_duct": "bottom plate on the bed, no supports",
    "torso_shell_left": "outer side face on the bed; shoulder/top panels bridge < 20 mm; switch panel vertical",
    "torso_shell_right": "outer side face on the bed; shoulder/top panels bridge < 20 mm",
    "torso_top": "flat plate on the bed",
    "deck_cover": "top face on the bed (upside down), no supports",
    "battery_strap": "TPU, printed flat as a strip and wrapped",
    "neck_pitch_idler_seat": "flat disc on the bed",
}


def volume_report(m):
    """P2b hacim/aciklik/kutle tablolari (markdown, FreeCAD): K_T butcesi, parca zarflari, her elektronik parcanin en
    yakin komsuya acikligi, isi kaynagi -> PLA acikligi, baski parcalari (256 mm kup), govde kutlesi ve COM'lar."""
    from collections import OrderedDict
    import electronics as E
    import openduck
    from geom import envelope_solid
    p, T = m.p, m.p["trunk"]
    env = p["iface"]["envelopes"]["trunk"]
    K = envelope_solid(env["add"]).cut(envelope_solid(env["cut"]))
    items = m.links["trunk"].items
    el_names = {n.replace("#", "_"): n for n in T["electronics"]}
    fixed_kinds = ("servo",)

    def cat(it):
        if it.module != "trunk":
            return "foreign harness"
        if it.name in el_names:
            return "electronics"
        if it.kind == "harness":
            return "wiring"
        if it.kind == "shell":
            return "shell PLA"
        if it.kind == "pad":
            return "strap TPU"
        if it.name == "trunk_fasteners":
            return "fasteners"
        if it.kind in fixed_kinds or it.name.startswith(("left_hip_yaw_bearing", "right_hip_yaw_bearing",
                                                         "neck_pitch_idler", "neck_seat")):
            return "fixed"
        return "structure PETG"

    vin = {it.name: it.shape.common(K).Volume for it in items}
    groups = OrderedDict()
    for it in items:
        g = groups.setdefault(cat(it), [0.0, 0.0, 0])
        g[0] += vin[it.name] / 1000.0
        g[1] += it.mass_kg(p) * 1000.0
        g[2] += 1
    VK = K.Volume / 1000.0
    used = sum(v for v, _, _ in groups.values())
    L = ["| Group | Items | Volume in K_T (cm3) | Mass (g) |", "|---|---|---|---|"]
    for k, (v, ms, n) in groups.items():
        L.append("| %s | %d | %.1f | %.1f |" % (k, n, v, ms))
    L.append("| **K_T** | | **%.1f** | |" % VK)
    L.append("| **free (K_T - all parts)** | | **%.1f** (%.0f %%) | |" % (VK - used, 100.0 * (VK - used) / VK))
    out = ["### Volume budget", ""] + L + [""]
    # elektronik tablosu + en yakin komsu
    L = ["| Part | Envelope (mm, placed x y z) | Vol (cm3) | Mass (g) | Heat (W) | Box x / y / z (mm) | Nearest "
         "neighbour (gap mm) | Est. |", "|---|---|---|---|---|---|---|---|"]
    by = {it.name: it for it in items}
    for iname, cname in el_names.items():
        it = by[iname]
        rec = E.PARTS[E.part_of(cname)]
        bb = it.shape.BoundBox
        best = (1e9, "-")
        for o in items:
            if o is it:
                continue
            ob = o.shape.BoundBox
            if (ob.XMin > bb.XMax + 15 or ob.XMax < bb.XMin - 15 or ob.YMin > bb.YMax + 15 or ob.YMax < bb.YMin - 15
                    or ob.ZMin > bb.ZMax + 15 or ob.ZMax < bb.ZMin - 15):
                continue
            dd = it.shape.distToShape(o.shape)[0]
            if dd < best[0]:
                best = (dd, o.name)
        gap = "%s (%.1f)" % (best[1], best[0]) if best[0] < 1e8 else "> 15"
        L.append("| %s | %.1f x %.1f x %.1f | %.1f | %.1f | %.1f | %.1f..%.1f / %.1f..%.1f / %.1f..%.1f | %s | %s |" % (
            iname, bb.XLength, bb.YLength, bb.ZLength, it.shape.Volume / 1000.0, rec["mass_g"], rec["heat_w"],
            bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax, gap, ", ".join(rec["est"]) or "-"))
    out += ["### Electronics envelopes and clearances", ""] + L + [""]
    # isi kaynaklari
    pla = [it for it in items if it.kind in ("shell", "trim", "face")]
    L = ["| Heat source | Loss (W) | Min gap to PLA (mm) | Nearest PLA part | Touching parts (gap < 0.5 mm) |",
         "|---|---|---|---|---|"]
    for iname, cname in el_names.items():
        if not E.is_heat_source(cname):
            continue
        it = by[iname]
        ds = sorted((it.shape.distToShape(q.shape)[0], q.name) for q in pla)
        touch = sorted(o.name + " (" + o.kind + ")" for o in items if o is not it and o.module == "trunk"
                       and it.shape.distToShape(o.shape)[0] < 0.5)
        L.append("| %s | %.1f | %.2f | %s | %s |" % (iname, E.PARTS[E.part_of(cname)]["heat_w"], ds[0][0], ds[0][1],
                                                    ", ".join(touch) or "-"))
    out += ["### Heat sources (spec 3.1: PETG at the source, >= %.0f mm air to PLA)" % m.cfg["targets"].get(
        "pla_heat_gap_mm", 5.0), ""] + L + [""]
    # baski parcalari
    bed = m.cfg["targets"].get("bed_mm", [256.0] * 3)
    L = ["| Print part | Material | Volume (cm3) | Mass (g) | Extents (mm) | Fits %g mm cube | Orientation / support |"
         % min(bed), "|---|---|---|---|---|---|---|"]
    for it in items:
        if it.kind not in openduck.PRINT_KINDS or it.module != "trunk":
            continue
        bb = it.shape.BoundBox
        ext = sorted((bb.XLength, bb.YLength, bb.ZLength))
        mat = (p.get("print", {}).get(it.kind) or {}).get("material", it.kind)
        L.append("| %s | %s | %.1f | %.1f | %.1f x %.1f x %.1f | %s | %s |" % (
            it.name, mat, it.shape.Volume / 1000.0, it.mass_kg(p) * 1000.0, ext[2], ext[1], ext[0],
            "yes" if ext[2] <= min(bed) else "NO", PRINT_NOTES.get(it.name, "-")))
    out += ["### Trunk print parts", ""] + L + [""]
    # kutle ve COM
    mt, com, _ = m.link_inertia("trunk")
    stand = m.pose("stand")
    pls = m.link_placements(stand)
    tot, acc = 0.0, None
    for ln in m.links:
        lm, lc, _ = m.link_inertia(ln)
        w = pls[ln].multVec(lc) * lm
        acc = w if acc is None else acc + w
        tot += lm
    hc = acc * (1.0 / tot)
    soles = [s.BoundBox for ln, it, s in m.posed_shapes(stand) if it.kind == "sole"]
    fx0, fx1 = min(b_.XMin for b_ in soles), max(b_.XMax for b_ in soles)
    budget = T.get("mass_budget_g")
    out += ["### Mass and COM", "",
            "| Quantity | Value |", "|---|---|",
            "| Trunk link mass (bottom-up) | %.1f g%s |" % (mt * 1000.0, " (budget %.1f g)" % budget if budget else ""),
            "| Trunk COM (trunk frame) | (%.1f, %.1f, %.1f) mm |" % (com.x, com.y, com.z),
            "| Robot mass (bottom-up, with P2a/P2c stubs) | %.1f g; x %.2f = %.1f g (limit %.0f g) |" % (
                tot * 1000.0, m.cfg["targets"].get("mass_factor", 1.05), tot * 1000.0 * m.cfg["targets"].get(
                    "mass_factor", 1.05), m.cfg["targets"].get("mass_max_kg", 1.95) * 1000.0),
            "| HOME COM (trunk frame) | (%.1f, %.1f, %.1f) mm |" % (hc.x, hc.y, hc.z),
            "| Feet x span at HOME / centre | %.1f..%.1f / %.1f mm; COM - centre %.1f mm |" % (
                fx0, fx1, (fx0 + fx1) / 2.0, hc.x - (fx0 + fx1) / 2.0), ""]
    return "\n".join(out)


GENERATORS = {"xl330_27": build_xl330, "tr35_b2": build_tr35_b2}
