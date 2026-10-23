# OpenDuck - 3B baski rehberi

Bu klasordeki her STL **tek bir kati parcadir**, baski yonune cevrilmis, tablada ortalanmis ve z=0'a
oturtulmustur. Olcu birimi mm. **Dilimleyicide dondurmeyin**; dosyayi oldugu gibi tablaya yerlestirin.

Yon nasil secildi:
- Cogu parcada otomatik: tablaya oturan duz alan buyuk, destek gerektiren sarkma (45 dereceden dik alt
  yuzey) kucuk, esitlikte daha alcak olan yon. Tablaya 3 mm'den yakin yuvarlatmalar ve 10 mm'den kisa
  kopruler/yatay delik tepeleri destek sayilmaz.
- Yuk tasiyan braketlerde ve gorunen kabuklarda yon dayanim ya da gorunus icin elle secildi (tabloda
  `Yon` sutunu): katman cizgileri yuk altindaki bukumlere paralel kalir, gorunen yuz tabla yuzeyi alir.
  Bu yuzden her parcada en genis duz yuz tablaya bakmaz.
- Sol/sag parcalar ayna simetriktir ve ayna yonde basilir; ikisini de basin.

Birden fazla ayrik katidan olusan model parcalari (head_turn_bracket, neck_splice) ayri STL'lere bolundu
(`_1`, `_2`...); her biri kendi yonunde tablaya oturur.

| | |
|---|---|
| Model parcasi cesidi | 25 |
| Baski parcasi cesidi | 27 |
| Basilacak toplam parca (= STL dosyasi) | 36 |
| Tahmini toplam filament | 296 g |
| En buyuk parca | head_hood (106 x 92 x 47 mm) |

Tum parcalar 180 x 180 x 180 mm'lik bir tablaya sigar.

## Ayar onerileri

| Tur | Malzeme | Renk | Ayar |
|---|---|---|---|
| shell | PLA | krem | 0.2 mm katman, 3 duvar, %15 dolgu |
| trim | PLA | turuncu | 0.2 mm katman, 3 duvar, %20 dolgu |
| face | PLA | acik gri | 0.2 mm katman, 3 duvar, %20 dolgu |
| bracket | PETG (veya PLA+) | gri | 0.2 mm katman, 4 duvar, %40 dolgu |
| sole | TPU 95A | koyu | 0.2 mm katman, 3 duvar, %30 dolgu, yavas |
| pad | TPU 85-95A | koyu | 0.2 mm katman, %100 dolgu, yavas |

- Braketler servo yuklerini tasir: PETG ya da PLA+ ve 4 duvar onerilir.
- Vida delikleri sadelestirilmistir. Basili braketin servoya ya da horn'a sikildigi yerlerde M2 icin
  ~2.2-2.4 mm bosluk deligi olmali (vida servonun/horn'un disine tutunur, braket yalnizca sikisir);
  1.6 mm pilot delik yalnizca vidanin plastige tutundugu yerlerde; isil ek somun yalnizca modelde
  yuvasi olan yerlerde kullanilir.
- `Destek: evet` olan parcalarda yalnizca tabla uzerinden destek (support on build plate) yeterlidir.
- Tabladan kalkma riskine karsi `Yon` sutununda kenar (brim) yazan parcalarda 5 mm brim kullanin.

## Parca listesi

| Parca | Aciklama | Adet | Malzeme | Renk | Boyut (mm, baski yonunde) | Kutle (g, adet basi) | Destek | Yon | STL |
|---|---|---:|---|---|---|---:|---|---|---|
| chassis | sasi (taban + servo kafesi + boyun catali) | 1 | PETG (veya PLA+) | gri | 60.5 x 60.0 x 42.9 | 17 | hayir | otomatik | `chassis.stl` |
| battery_tray | batarya yuvasi | 1 | PETG (veya PLA+) | gri | 32.3 x 55.0 x 41.0 | 10 | hayir | otomatik | `battery_tray.stl` |
| torso_shell_left | govde kabugu sol yarim | 1 | PLA | krem | 80.9 x 41.7 x 32.2 | 11 | hayir | dis yan yuz tablada: desteksiz, alcak, gorunen yuz tabla yuzeyi | `torso_shell_left.stl` |
| torso_shell_right | govde kabugu sag yarim | 1 | PLA | krem | 80.9 x 41.7 x 32.2 | 11 | hayir | dis yan yuz tablada: desteksiz, alcak, gorunen yuz tabla yuzeyi | `torso_shell_right.stl` |
| hip_bracket | kalca yaw braketi | 2 | PETG (veya PLA+) | gri | 27.0 x 24.0 x 30.0 | 5 | hayir | servo arka plakasi katman duzleminde, desteksiz | `left_hip_bracket.stl`, `right_hip_bracket.stl` |
| hip_cradle_bracket | kalca roll-pitch braketi | 2 | PETG (veya PLA+) | gri | 29.0 x 32.0 x 19.0 | 3 | hayir | L profili katman duzleminde, kollar bukumden ayrilmaz; 5 mm kenar (brim) | `left_hip_cradle_bracket.stl`, `right_hip_cradle_bracket.stl` |
| thigh_inner_plate | uyluk ic plakasi | 2 | PETG (veya PLA+) | gri | 61.3 x 27.5 x 2.0 | 3 | hayir | otomatik | `left_thigh_inner_plate.stl`, `right_thigh_inner_plate.stl` |
| thigh_wing | uyluk kanat kabugu | 2 | PLA | krem | 61.3 x 47.5 x 9.5 | 10 | hayir | otomatik | `left_thigh_wing.stl`, `right_thigh_wing.stl` |
| shin_bracket | incik braketi | 2 | PETG (veya PLA+) | gri | 40.5 x 26.0 x 22.0 | 4 | hayir | otomatik | `left_shin_bracket.stl`, `right_shin_bracket.stl` |
| shin_pad | diz/incik pedi | 2 | PLA | krem | 24.0 x 24.0 x 3.0 | 2 | hayir | otomatik | `left_shin_pad.stl`, `right_shin_pad.stl` |
| ankle_bracket | ayak bilegi U braketi | 2 | PETG (veya PLA+) | gri | 22.0 x 35.0 x 27.0 | 5 | hayir | otomatik | `left_ankle_bracket.stl`, `right_ankle_bracket.stl` |
| foot_body | ayak govdesi | 2 | PLA | turuncu | 54.0 x 40.5 x 7.5 | 14 | hayir | otomatik | `left_foot_body.stl`, `right_foot_body.stl` |
| sole | taban (TPU) | 2 | TPU 95A | koyu | 54.0 x 40.5 x 5.0 | 13 | hayir | otomatik | `left_sole.stl`, `right_sole.stl` |
| neck_splice_1 | boyun ek plakalari - ayri parca 1/2 | 1 | PETG (veya PLA+) | gri | 14.5 x 26.0 x 2.0 | 1 | hayir | duz plaka, tablada | `neck_splice_1.stl` |
| neck_splice_2 | boyun ek plakalari - ayri parca 2/2 | 1 | PETG (veya PLA+) | gri | 14.5 x 26.0 x 2.0 | 1 | hayir | duz plaka, tablada | `neck_splice_2.stl` |
| head_tilt_bracket | kafa egim braketi | 1 | PETG (veya PLA+) | gri | 28.7 x 34.0 x 20.0 | 5 | hayir | U profili katman duzleminde (kafa egim torku katman arasina binmez) | `head_tilt_bracket.stl` |
| head_turn_bracket_1 | kafa donus braketi - ayri parca 1/2 | 1 | PETG (veya PLA+) | gri | 22.5 x 32.0 x 7.9 | 6 | hayir | -X yuzu tablada, 10 mm pim yukari bakar ve desteksiz yuvarlak basilir | `head_turn_bracket_1.stl` |
| head_turn_bracket_2 | kafa donus braketi - ayri parca 2/2 | 1 | PETG (veya PLA+) | gri | 22.5 x 29.5 x 9.0 | 3 | hayir | -X yuzu tablada, 10 mm pim yukari bakar ve desteksiz yuvarlak basilir | `head_turn_bracket_2.stl` |
| head_hood | kafa ust kaput | 1 | PLA | krem | 105.6 x 91.8 x 47.4 | 38 | evet | acik alt tablada: dis yuzeyde destek yok, 47 mm (sirtustu 109 mm); ic tavana tabladan agac (tree) destek | `head_hood.stl` |
| head_bottom_shell | kafa alt kabugu | 1 | PLA | krem | 76.4 x 91.8 x 13.3 | 15 | hayir | otomatik | `head_bottom_shell.stl` |
| upper_bill | ust gaga | 1 | PLA | turuncu | 40.3 x 91.8 x 4.5 | 2 | hayir | otomatik | `upper_bill.stl` |
| upper_bill_pad | ust gaga pedi (TPU) | 1 | TPU 85-95A | koyu | 33.3 x 87.2 x 1.1 | 4 | hayir | otomatik | `upper_bill_pad.stl` |
| face_plate | yuz plakasi | 1 | PLA | acik gri | 47.8 x 87.6 x 2.0 | 8 | hayir | otomatik | `face_plate.stl` |
| eye_ring | goz halkasi | 1 | PLA | turuncu | 30.0 x 30.0 x 9.5 | 4 | hayir | otomatik | `eye_ring.stl` |
| head_frame | kafa ic cercevesi | 1 | PETG (veya PLA+) | gri | 27.5 x 78.5 x 73.5 | 14 | evet | otomatik | `head_frame.stl` |
| lower_bill | alt gaga (cene) | 1 | PLA | turuncu | 65.7 x 98.8 x 26.9 | 15 | evet | otomatik | `lower_bill.stl` |
| lower_bill_pad | alt gaga pedi (TPU) | 1 | TPU 85-95A | koyu | 31.3 x 87.8 x 4.8 | 16 | hayir | otomatik | `lower_bill_pad.stl` |

Robot: OpenDuck (Microduck benzeri), 15 servo (DYNAMIXEL XL330-M288-T), toplam ~676 g.
