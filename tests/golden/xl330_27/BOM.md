# OpenDuck - Malzeme Listesi (BOM)

> `tools/build.sh` tarafindan modelden otomatik uretildi.

## Ozet

| | |
|---|---|
| Robot | OpenDuck (Microduck benzeri) |
| Toplam kutle (tahmini, kablo/vida haric) | 676 g |
| Baski parcalarinin kutlesi | 296 g |
| Boy (HOME pozu) | 273 mm |
| Serbestlik derecesi | 15 (15 x DYNAMIXEL XL330-M288-T) |

## Satin alinacak parcalar

| Adet | Parca | Not |
|---:|---|---|
| 15 | ROBOTIS DYNAMIXEL XL330-M288-T | bacaklar 2x5, boyun+kafa 4, gaga 1 (ID 10-14, 20-24, 30-34) |
| 1 | Radxa Zero 3W (RK3566) veya Raspberry Pi Zero 2 W | kafada, dik |
| 1 | Robot HAT (Pollen elec_RPI_Robot_HAT, Apache-2.0) veya esdegeri | Dynamixel TTL, ses, 5 V |
| 1 | IMX219 kamera + M12 lens | goz |
| 1 | VL53L5CX 8x8 ToF | 'lidar' |
| 1 | LSM6DSV16X IMU karti | govde |
| 1 | NP-F550 2S Li-ion (7.4 V, 2600 mAh) + yuva konnektoru | servolara dogrudan |
| 1 | Hoparlor 25x35 mm, MEMS mikrofon |  |
| 1 | M2 vida seti, isil ek somun, kablolar | bacak horn baglantilari 2x20 + boyun catali 4 vida; ayak bilegi U braketi 2x4 havsa basli vida + 8 isil ek; batarya yuvasi 3 vida + 3 isil ek |

## 3B baski parcalari

Malzeme ve renk onerileri `print/PRINT.md` ile aynidir. Bir model parcasi birden fazla ayrik katidan
olusuyorsa (or. iki plaka) `Basilacak parca` sutunu ayri basilan parca sayisini verir; baskiya hazir
STL'ler `print/stl/` altindadir. Buradaki STL sutunu montaj (link) cercevesindeki `stl/` dosyalaridir.

| Parca | Aciklama | Model adedi | Basilacak parca | Malzeme | Renk | Toplam hacim (cm3) | Toplam kutle (g) | STL (montaj cercevesi) |
|---|---|---:|---:|---|---|---:|---:|---|
| chassis | sasi (taban + servo kafesi + boyun catali) | 1 | 1 | PETG (veya PLA+) | gri | 15.0 | 17 | `chassis.stl` |
| battery_tray | batarya yuvasi | 1 | 1 | PETG (veya PLA+) | gri | 9.2 | 10 | `battery_tray.stl` |
| torso_shell_left | govde kabugu sol yarim | 1 | 1 | PLA | krem | 10.4 | 11 | `torso_shell_left.stl` |
| torso_shell_right | govde kabugu sag yarim | 1 | 1 | PLA | krem | 10.4 | 11 | `torso_shell_right.stl` |
| hip_bracket | kalca yaw braketi | 2 | 2 | PETG (veya PLA+) | gri | 8.7 | 10 | `left_hip_bracket.stl`, `right_hip_bracket.stl` |
| hip_cradle_bracket | kalca roll-pitch braketi | 2 | 2 | PETG (veya PLA+) | gri | 5.9 | 6 | `left_hip_cradle_bracket.stl`, `right_hip_cradle_bracket.stl` |
| thigh_inner_plate | uyluk ic plakasi | 2 | 2 | PETG (veya PLA+) | gri | 5.7 | 6 | `left_thigh_inner_plate.stl`, `right_thigh_inner_plate.stl` |
| thigh_wing | uyluk kanat kabugu | 2 | 2 | PLA | krem | 17.9 | 20 | `left_thigh_wing.stl`, `right_thigh_wing.stl` |
| shin_bracket | incik braketi | 2 | 2 | PETG (veya PLA+) | gri | 7.6 | 8 | `left_shin_bracket.stl`, `right_shin_bracket.stl` |
| shin_pad | diz/incik pedi | 2 | 2 | PLA | krem | 3.4 | 4 | `left_shin_pad.stl`, `right_shin_pad.stl` |
| ankle_bracket | ayak bilegi U braketi | 2 | 2 | PETG (veya PLA+) | gri | 9.1 | 10 | `left_ankle_bracket.stl`, `right_ankle_bracket.stl` |
| foot_body | ayak govdesi | 2 | 2 | PLA | turuncu | 25.2 | 28 | `left_foot_body.stl`, `right_foot_body.stl` |
| sole | taban (TPU) | 2 | 2 | TPU 95A | koyu | 21.0 | 25 | `left_sole.stl`, `right_sole.stl` |
| neck_splice | boyun ek plakalari | 1 | 2 | PETG (veya PLA+) | gri | 1.5 | 2 | `neck_splice.stl` |
| head_tilt_bracket | kafa egim braketi | 1 | 1 | PETG (veya PLA+) | gri | 4.3 | 5 | `head_tilt_bracket.stl` |
| head_turn_bracket | kafa donus braketi | 1 | 2 | PETG (veya PLA+) | gri | 7.7 | 9 | `head_turn_bracket.stl` |
| head_hood | kafa ust kaput | 1 | 1 | PLA | krem | 34.7 | 38 | `head_hood.stl` |
| head_bottom_shell | kafa alt kabugu | 1 | 1 | PLA | krem | 13.2 | 15 | `head_bottom_shell.stl` |
| upper_bill | ust gaga | 1 | 1 | PLA | turuncu | 1.5 | 2 | `upper_bill.stl` |
| upper_bill_pad | ust gaga pedi (TPU) | 1 | 1 | TPU 85-95A | koyu | 3.2 | 4 | `upper_bill_pad.stl` |
| face_plate | yuz plakasi | 1 | 1 | PLA | acik gri | 7.3 | 8 | `face_plate.stl` |
| eye_ring | goz halkasi | 1 | 1 | PLA | turuncu | 3.9 | 4 | `eye_ring.stl` |
| head_frame | kafa ic cercevesi | 1 | 1 | PETG (veya PLA+) | gri | 12.6 | 14 | `head_frame.stl` |
| lower_bill | alt gaga (cene) | 1 | 1 | PLA | turuncu | 13.5 | 15 | `lower_bill.stl` |
| lower_bill_pad | alt gaga pedi (TPU) | 1 | 1 | TPU 85-95A | koyu | 13.1 | 16 | `lower_bill_pad.stl` |
| **Toplam** | | 34 | 36 | | | 266.0 | 296 | |

Yogunluk varsayimi: PLA 1.10 g/cm3, TPU 1.20 g/cm3.
