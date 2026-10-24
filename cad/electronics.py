# -*- coding: utf-8 -*-
"""
TR-35 B2-ESP32 elektronik zarflari (spec 3.1 "Butun elektronik CAD'de gercek zarfiyla"): satin alinan her kartin /
parcanin dis zarfi (konnektor, lehimli kablo cikisi ve en yuksek bilesen dahil), kutlesi, isi kaybi ve kaynagi. Veri
kismi FreeCAD gerektirmez (hacim/kutle butcesi, tests/cad); geometri (solid) FreeCAD'i ilk cagrida yukler.

Kaynaklar: docs/workflow/openduck-tr/research/power-electronics.md (F1-F5, 1, 5, 6 BOM), parts-research.md,
listings.json. Yayimlanmamis ya da kumpasla dogrulanmamis degerler rec["est"] alaninda isaretlidir (P3 BOM / Adim 0).
P2b kararlari (rec["note"]):
  * PCA9685: servo basliklari, vida terminali ve V+ kondansatoru sokulur, yalniz sinyal kablolari lehimlenir (V+
    kullanilmaz, power-electronics 2; kutle merdiveni 3 "dogrudan lehim") -> 7 mm zarf.
  * ESP32-S3-DevKitC-1: pin basliklari sokulur, kablolar lehimlenir -> 8 mm zarf; USB-C/mikro-USB ucu disari bakar.
  * LiPo alarmi: ekransiz buzzer turu (30 x 18 x 9, tahmin). Ekranli 1-8S olcer (40 x 25 x 11) donmus K_T'de hicbir
    bosluga sigmiyor (p2b-hacim.md).
  * VBAT: 1 x 1000 uF 35 V buck girisinde (power-electronics 1 "1000 uF on the battery bus"); BOM'daki ikincisi yedek.
  * Isi kaynagi (spec 3.1 "isi kaynagina PETG, PLA'ya >= 5 mm hava"): surekli kayip >= HEAT_SOURCE_W (power-electronics
    1 Thermal: buck 4-5 W, XL4015 < 2 W). ESP32 (0.5 W) ve MP1584 (0.4 W) ilik parcadir, kural uygulanmaz.

Kayit: ad -> {"size": (x, y, z) mm kayit cercevesinde (yatay yerlesim: x uzun kenar, z yukseklik), "form": "box" |
"cyl" (kayit x ekseninde silindir, cap = size[1] = size[2]) | "switch" (KCD1: govde + on cerceve, cerceve +x ucunda),
"mass_g", "kind" (openduck.KINDS), "heat_w" (surekli kayip, W), "qty", "source", "est", "note"}.
Yerlesim (params.trunk.electronics): [x, y, z_alt, rot_z, perm]: kutu alt orta noktasi; perm dunya x/y/z'ye giden
kayit ekseni ("yzx": kayit y -> dunya x, z -> y, x -> z).
"""
from collections import OrderedDict

PARTS = OrderedDict()
HEAT_SOURCE_W = 1.0


def _add(name, size, mass_g, kind="pcb", heat_w=0.0, qty=1, source="", est=(), form="box", note=""):
    PARTS[name] = {"size": tuple(size), "mass_g": mass_g, "kind": kind, "heat_w": heat_w, "qty": qty,
                   "source": source, "est": tuple(est), "form": form, "note": note}


# guc zinciri (spec 3.1): 4S LiPo -> XT30 -> 15 A sigorta -> anahtar -> VBAT -> 20 A buck / XL4015 / MP1584
_add("battery_4s_lipo", (78.0, 40.0, 30.0), 200.0, "battery", 0.0, 1,
     "Profuse 4S 1500 mAh 95C (motorobit): 78 x 40 x 30 mm, 200 g; XT60 -> XT30, JST-XH 5 pin denge "
     "(power-electronics F4, 6)", note="ana ve denge kablolari on (+x) yuzden cikar")
_add("lipo_alarm", (30.0, 18.0, 9.0), 5.0, "pcb2", 0.0, 1,
     "1-8S LiPo dusuk voltaj alarmi, denge soketinde (spec 3.1)", ("size", "mass_g"),
     note="ekransiz buzzer turu; ekranli 40 x 25 x 11 surum K_T'ye sigmiyor")
_add("xt30_pair", (22.0, 10.5, 5.5), 3.0, "chip", 0.0, 1,
     "XT30 erkek + disi takili (komponentci), ana kesme (power-electronics 6 #5)", ("size",))
_add("fuse_15a", (25.0, 14.0, 10.0), 5.0, "chip", 0.0, 1,
     "kablolu mini bicak sigorta yuvasi (direnc) + 15 A mini low-profile 10.9 x 8.73 (motorobit) (F4)", ("size",))
_add("main_switch", (20.5, 15.0, 21.0), 4.0, "chip", 0.0, 1,
     "KCD1 kulbutor (motorobit) 'DC 12V 20A', 16.8 V'ta <= 10 A'e dusurulur (power-electronics Risks); "
     "on cerceve 21 x 15 x 2.5, panel kesimi 19 x 13, govde + terminal 18", ("size",), form="switch",
     note="spec 3.1: 16.8 V DC'ye uygun anahtar stokta yoksa yalniz XT30")
_add("power_latch", (20.0, 12.0, 6.0), 3.0, "pcb", 0.0, 1,
     "guc kesme MOSFET/latch icin ayrilan yer: 2 x P-MOSFET + surucu, delikli kart (spec 3.1)", ("size", "mass_g"),
     note="ayrilmis hacim; v1'de bos kalabilir")
_add("vbat_cap", (20.0, 13.0, 13.0), 3.0, "chip", 0.0, 1,
     "1000 uF 35 V elektrolitik 13 x 20 (motorobit), buck girisi (power-electronics 1, 6 #14)", ("mass_g",),
     form="cyl")
_add("ina226", (23.0, 16.0, 4.0), 2.0, "pcb", 0.0, 1, "INA226 modulu (direnc), yalniz VBUS 0x45 (spec 3.1)",
     ("size", "mass_g"))
_add("buck_20a", (60.0, 53.0, 27.0), 70.0, "pcb2", 5.0, 1,
     "300 W / 20 A CV/CC buck (robocombo): 60 x 53 x 27 mm, 6.50 V bacak rayi, kayip 4-5 W @6 A, 15 A uzeri fan "
     "(power-electronics F4, F5, 1 Thermal)", ("mass_g",), note="kutle yayimlanmamis (70 g tahmin, Adim 0'da tart)")
_add("leg_bus_cap", (20.0, 10.0, 10.0), 3.0, "chip", 0.0, 2,
     "2200 uF 16 V elektrolitik 10 x 20 (motorobit), LEG barasi (power-electronics 1, 6 #15)", ("mass_g",), form="cyl")
_add("overshoot_clamp", (20.0, 12.0, 10.0), 3.0, "pcb", 0.0, 1,
     "LEG rayi asim kirpicisi yeri: zener + MOSFET + bosaltma direnci (spec 3.1; uygun TVS stokta yok, "
     "power-electronics F6)", ("size", "mass_g"), note="darbeli kayip; surekli isi yok")
_add("servo_hub", (24.0, 22.0, 8.0), 12.0, "pcb", 0.0, 1,
     "servo guc dagitimi + yildiz toprak: 7 x 9 perfboard parcasi, LEG (+6.5 V), AUX (+6.0 V) ve toprak barasi "
     "(1.5 mm2 bakir), 15 servo lehim noktasi (power-electronics 1, 5)", ("size", "mass_g"))
_add("xl4015", (54.0, 24.0, 16.0), 16.0, "pcb2", 2.0, 1,
     "XL4015 5 A (komponentci) 54 x 24 x 16 sogutuculu, 6.0 V AUX rayi; ~16 g (F4)")
_add("aux_bus_cap", (20.0, 10.0, 10.0), 3.0, "chip", 0.0, 1,
     "2200 uF 16 V elektrolitik 10 x 20, AUX barasi (power-electronics 1)", ("mass_g",), form="cyl")
_add("mp1584", (22.0, 17.0, 4.0), 2.0, "pcb2", 0.4, 1, "MP1584EN 5 V 3 A modul (komponentci), 1N5819 ile 5V pini (F4)",
     ("size", "mass_g"))
# lojik
_add("esp32_s3_devkitc", (63.0, 25.4, 8.0), 10.0, "pcb", 0.5, 1,
     "ESP32-S3-DevKitC-1 N16R8 (komponentci): PCB 63 x 25.4 (Espressif cizimi, F1)", ("size", "mass_g"),
     note="pin basliklari sokulur, kablolar lehimli; USB ucu disari bakar")
_add("divider_board", (40.0, 20.0, 6.0), 4.0, "pcb", 0.0, 1,
     "pot bolucu karti: 10 x (100k / 47k + 100 nF), 1N5819, I2C/SPI dagitimi; ESP32 yaninda (power-electronics 3)",
     ("size", "mass_g"))
_add("pca9685", (62.5, 25.4, 7.0), 6.5, "pcb2", 0.0, 2,
     "PCA9685 16 kanal (robocombo; Adafruit 815 ile ayni 62.5 x 25.4, ciplak 5.5 g); 0x40 bacak 305 Hz, "
     "0x41 kafa/yaw 50 Hz (F2, 2)", ("size", "mass_g"),
     note="servo basliklari, terminal ve V+ kondansatoru sokulur; sinyal kablolari lehimli")
_add("ads1115", (28.0, 17.5, 5.0), 2.0, "pcb", 0.0, 2,
     "ADS1115 modulu: 0x48 boyun + kafa potlari, 0x49 diz NTC (spec 3.1)", ("size", "mass_g"))
_add("imu_mpu6500", (25.0, 15.0, 3.0), 2.0, "pcb", 0.0, 1,
     "GY-6500 15 x 25 mm (motorobit, F3), SPI GPIO39-42; MJCF imu sitesi, eksenleri govdeye paralel",
     ("mass_g",))

# kablo kutlesi (power-electronics 5: ~55 g; dogrudan lehim -20 g, kutle merdiveni 3): link basina dagitim
WIRING_G = {"trunk": 25.0, "leg_each": 12.0, "head": 6.0}


def part_of(name):
    """Yerlesim anahtari ("pca9685#1") -> kayit adi."""
    return name.rsplit("#", 1)[0]


def total_mass_g(names=None):
    names = names or list(PARTS)
    return sum(PARTS[n]["mass_g"] * PARTS[n]["qty"] for n in names)


def volume_cm3(names=None):
    """Zarf kutularinin hacmi (cm3, adet dahil)."""
    names = names or list(PARTS)
    out = 0.0
    for n in names:
        x, y, z = PARTS[n]["size"]
        out += x * y * z * PARTS[n]["qty"] / 1000.0
    return out


def is_heat_source(name):
    return PARTS[part_of(name)]["heat_w"] >= HEAT_SOURCE_W


def placed_size(name, perm="xyz"):
    """Yerlestirilmis kutu olculeri: perm, dunya x/y/z'ye giden kayit ekseni (or. "xzy": kart kenar ustu, kalinlik
    y'de; "yzx": kayit y -> dunya x, z -> y, x -> z)."""
    s = dict(zip("xyz", PARTS[part_of(name)]["size"]))
    return tuple(s[a] for a in perm)


def placed_box(name, place):
    """Yerlesim [x, y, z_alt, rot_z, perm] -> dunya kutusu (x0, x1, y0, y1, z0, z1); rot_z 0/90/180/270."""
    x, y, z = place[:3]
    rot = place[3] if len(place) > 3 else 0.0
    sx, sy, sz = placed_size(name, place[4] if len(place) > 4 else "xyz")
    if int(round(rot / 90.0)) % 2:
        sx, sy = sy, sx
    return (x - sx / 2.0, x + sx / 2.0, y - sy / 2.0, y + sy / 2.0, z, z + sz)


def _canon(rec):
    """Kayit cercevesinde, orijin merkezli kati (x +-sx/2, y +-sy/2, z +-sz/2)."""
    from geom import box, xcyl
    sx, sy, sz = rec["size"]
    if rec["form"] == "cyl":
        return xcyl(sy / 2.0, 0.0, 0.0, -sx / 2.0, sx / 2.0)
    if rec["form"] == "switch":     # KCD1: govde 19 x 13 (panel kesimi), on cerceve 2.5 kalinlik (+x ucu)
        bezel = 2.5
        body = box(-sx / 2.0, sx / 2.0 - bezel, -6.5, 6.5, -9.5, 9.5)
        return body.fuse(box(sx / 2.0 - bezel, sx / 2.0, -sy / 2.0, sy / 2.0, -sz / 2.0, sz / 2.0)).removeSplitter()
    return box(-sx / 2.0, sx / 2.0, -sy / 2.0, sy / 2.0, -sz / 2.0, sz / 2.0)


def solid(name, origin, rot_z=0.0, perm="xyz"):
    """Parcanin zarf katisi (FreeCAD): origin = kutu alt orta noktasi (x, y, z), perm eksen sirasi (placed_size), rot_z:
    z ekseni etrafinda derece."""
    import FreeCAD as App
    rec = PARTS[part_of(name)]
    s = _canon(rec)
    ax = "xyz"
    # tek permutasyon (yansima) yerine kayit y ekseni ters cevrilmis donme: butun zarflar y'de simetrik
    odd = perm in ("xzy", "yxz", "zyx")
    rows = []
    for i in range(3):
        for j in range(3):
            v = 1.0 if perm[i] == ax[j] else 0.0
            rows.append(-v if (odd and j == 1) else v)
        rows.append(0.0)
    m = App.Matrix(*(rows + [0.0, 0.0, 0.0, 1.0]))
    if perm != "xyz":
        s = s.copy()
        s.transformShape(m)
    if rot_z:
        s.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), rot_z)
    sz = placed_size(name, perm)[2]
    s.translate(App.Vector(origin[0], origin[1], origin[2] + sz / 2.0))
    return s
