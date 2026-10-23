# -*- coding: utf-8 -*-
"""
TR-35 B2-ESP32 elektronik zarflari (spec 3.1 "Butun elektronik CAD'de gercek zarfiyla"): satin alinan kartlarin
dis olculeri (konnektor/pin basliklari dahil kutu), kutleleri, isi kaynagi bilgisi ve kaynaklari. Veri kismi
FreeCAD gerektirmez (P2-0 hacim/kutle butcesi, tests/cad); geometri (solid) FreeCAD'i ilk cagrida yukler.

P2-0 yer tutucu: P2b (cad/trunk.py + bu dosya) gercek yerlesimi, montaj deliklerini, havalandirmayi ve kablo
kanallarini cizer. Olculer docs/workflow/openduck-tr/research/power-electronics.md (F2-F4, 5) ve
parts-research.md'den; "est" isaretliler tahmindir (Adim 0'da kumpasla dogrulanir).

Kayit: ad -> {"size": (x, y, z) mm (kartin yatay yerlesiminde: x uzun kenar), "mass_g", "kind" (openduck.KINDS),
"heat_w" (surekli isi, W; > 0 ise PETG + >= 5 mm hava, spec 3.1), "qty", "source", "est"}.
"""
from collections import OrderedDict

PARTS = OrderedDict()


def _add(name, size, mass_g, kind="pcb", heat_w=0.0, qty=1, source="", est=()):
    PARTS[name] = {"size": tuple(size), "mass_g": mass_g, "kind": kind, "heat_w": heat_w, "qty": qty,
                   "source": source, "est": tuple(est)}


# guc (spec 3.1 Guc): 4S LiPo -> XT30 -> 15 A sigorta -> anahtar -> raylar
_add("battery_4s_lipo", (78.0, 40.0, 30.0), 200.0, "battery", 0.0, 1,
     "Profuse 4S 1500 mAh 95C, motorobit (power-electronics F4)")
_add("lipo_alarm", (40.0, 25.0, 11.0), 8.0, "pcb2", 0.0, 1, "denge soketi voltaj alarmi (tahmin)", ("size", "mass_g"))
_add("buck_20a", (60.0, 53.0, 27.0), 70.0, "pcb2", 5.0, 1,
     "300 W / 20 A CV/CC buck, 6.50 V bacak rayi; isi 4-5 W @6 A (F4, F5)", ("mass_g",))
_add("xl4015", (54.0, 24.0, 16.0), 16.0, "pcb2", 2.0, 1, "XL4015 5 A, 6.0 V AUX rayi (F4)")
_add("mp1584", (22.0, 17.0, 4.0), 2.0, "pcb2", 0.3, 1, "MP1584EN 5 V lojik (F4)", ("size", "mass_g"))
_add("ina226", (23.0, 16.0, 3.0), 2.0, "pcb", 0.0, 1, "INA226 VBUS (0x45)", ("size", "mass_g"))
_add("fuse_15a", (25.0, 14.0, 10.0), 5.0, "chip", 0.0, 1, "mini bicak sigorta + yuva (F4)", ("size",))
_add("main_switch", (21.0, 15.0, 22.0), 4.0, "chip", 0.0, 1, "KCD1 kulbutor, 16.8 V DC'ye uygun (stokta dogrula)",
     ("size",))
_add("power_latch", (25.0, 15.0, 8.0), 5.0, "pcb", 0.0, 1, "guc kesme MOSFET/latch yeri (spec 3.1, ayrilan hacim)",
     ("size", "mass_g"))
_add("xt30_pair", (16.0, 8.0, 10.0), 3.0, "chip", 0.0, 1, "XT30 ana kesme (F4)", ("size",))
# lojik
_add("esp32_s3_devkitc", (63.0, 25.4, 10.0), 10.0, "pcb", 0.5, 1, "ESP32-S3-DevKitC-1 N16R8 (F1; pinler dahil)",
     ("mass_g",))
_add("pca9685", (62.5, 25.4, 12.0), 9.0, "pcb2", 0.0, 2,
     "PCA9685 0x40 bacak 300 Hz, 0x41 kafa 50 Hz (F2; terminal+baslik)")
_add("ads1115", (28.0, 17.5, 4.0), 2.0, "pcb", 0.0, 2, "ADS1115 0x48 boyun+kafa pot, 0x49 diz NTC (spec 3.1)",
     ("size", "mass_g"))
_add("imu_mpu6500", (25.0, 15.0, 3.0), 2.0, "pcb", 0.0, 1, "GY-6500 SPI (F3); MJCF imu sitesi", ("mass_g",))
_add("servo_hub", (46.0, 36.0, 12.0), 30.0, "pcb", 0.0, 1,
     "servo dagitim/yildiz toprak perfboard (7x9 cm'den kesilmis) + 3 x 2200 uF yatik + bolucu direncler "
     "(power-electronics 5); P2-0'da kenar ustu yerlesim icin 46 x 36 x 12", ("size",))
_add("bus_caps", (25.0, 13.0, 20.0), 6.0, "chip", 0.0, 1, "2 x 1000 uF 35 V VBAT (yatik)", ("size",))

# kablo kutlesi (power-electronics 5: ~55 g; dogrudan lehimle -20 g, kutle merdiveni 3): link basina dagitim
WIRING_G = {"trunk": 25.0, "leg_each": 12.0, "head": 6.0}


def total_mass_g(names=None):
    names = names or list(PARTS)
    return sum(PARTS[n]["mass_g"] * PARTS[n]["qty"] for n in names)


def volume_cm3(names=None):
    names = names or list(PARTS)
    out = 0.0
    for n in names:
        x, y, z = PARTS[n]["size"]
        out += x * y * z * PARTS[n]["qty"] / 1000.0
    return out


def placed_size(name, perm="xyz"):
    """Yerlestirilmis kutu olculeri: perm, dunya x/y/z'ye giden kayit ekseni (or. "xzy": kart kenar ustu, kalinlik
    y'de; "zyx": dik)."""
    s = dict(zip("xyz", PARTS[name]["size"]))
    return tuple(s[a] for a in perm)


def solid(name, origin, rot_z=0.0, perm="xyz"):
    """Kartin zarf kutusu (FreeCAD): origin = kutu alt orta noktasi (x, y, z), perm eksen sirasi (placed_size), rot_z:
    z ekseni etrafinda derece."""
    from geom import box, rotated
    x, y, z = placed_size(name, perm)
    s = box(-x / 2.0, x / 2.0, -y / 2.0, y / 2.0, 0.0, z)
    if rot_z:
        s = rotated(s, (0, 0, 0), (0, 0, 1), rot_z)
    s.translate(__import__("FreeCAD").Vector(*origin))
    return s
