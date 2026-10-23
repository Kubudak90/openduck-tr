# Spec: OpenDuck-TR (v1, CAD + simülasyon + RL)

Girdi: `research/parts-research.md` (Türkiye stok/fiyat, 2026-10-23), `research/torque-needs.md`,
`research/servo-mechanics.md`, `research/cad-map.md`, `research/power-electronics.md`, `research/rl-map.md`,
`research/discovery-critic.md` (R1–R19, G1–G9, D1–D15). Sayıların kaynakları oradadır; bu belge kararları verir.

## 1. Problem ve hedef

Türkiye'de bugün stokta olan, ucuz parçalarla basılabilen ~35 cm'lik bir BDX/Microduck tarzı ördek robotun
**parametrik CAD'i, baskı seti, BOM'u, servo modelli MuJoCo modeli ve simülasyonda eğitilmiş politikaları**
(yürüme, boy ayarı + oturma/kalkma, düşünce kalkma). Donanım "B2-ESP32": PWM servolar, bacaklarda potansiyometre
geri bildirimi, politika ESP32-S3'te koşacak (firmware bu sürümde yok, ama politika boyutu ona göre seçilir).

## 2. Kapsam dışı (non-goals)

- ESP32-S3 firmware'i, kumanda (gamepad/Wi-Fi), tutum filtresinin gömülü uygulaması (bir sonraki tur).
- Satın alma, tezgâh testi (Adım 0) ve gerçek robot denemeleri (kullanıcı yapar; sonuçlar sysID olarak geri gelir).
- 27 cm XL330 profili: bu repo yalnız TR-35'i hedefler; micro robot `Kubudak90/openduck`'ta kalır. Bu repoda
  `walk` için "bit-düzeyinde aynı" sözleşmesi (rl/TASK.md) kaldırılır.
- Kafa elektroniği (kamera, hoparlör, ToF): kafa elektroniksiz, göz kozmetik baskı.
- Yazılım tabanlı servo sıcaklık modeli; yalnız simülasyon tork/ısıl ölçütleri (§6).

## 3. Mimari

### 3.1 Donanım (kararlar)

| Eklem (sol/sağ) | Servo | Adet | Not |
|---|---|---|---|
| hip_roll, hip_pitch, knee, ankle | DS32xx (DS3230MG varsayılan; DS3225MG/DS3218MG aynı gövde) | 8 | 180° (500–2500 µs), 6.5 V |
| neck_pitch | DS32xx | 1 | **Kullanıcı kararı**: kalkma için güçlü boyun. Gövde trunk'ta sabit, horn boynu sürer |
| hip_yaw | EMAX ES3054 | 2 | 6.0 V, idler + baskı halkası (G3) |
| head_pitch, head_yaw, head_roll | MG90S | 3 | 6.0 V; head_yaw ±80° (180° servo) |
| mouth | SG90 | 1 | 6.0 V |

- **Geri bildirim**: 9 DS + 2 ES3054 = 11 potansiyometre. 10'u ESP32-S3 ADC1 (GPIO1–10, 100k/47k bölücü + 100 nF,
  12 dB). 11. (neck_pitch) ve 2 diz NTC'si harici **ADS1115** (I2C) üzerinden. Kafa eklemleri komutla gözlenir.
- **Elektronik**: ESP32-S3-DevKitC-1 N16R8; 2 × PCA9685 (0x40: 9 DS, prescale 19/20 ≈ 300 Hz; 0x41: ES3054 +
  MG90S + SG90, 50 Hz); MPU6500 (SPI GPIO39–42); INA226 (pil); ADS1115.
- **Güç** (**kullanıcı kararı**: hazır LiPo): Profuse 4S 1500 mAh 95C → 15 A sigorta → anahtar →
  (a) 300 W / 20 A buck 6.50 V → 9 DS; (b) XL4015 6.0 V → ES3054 + MG90S + SG90; (c) MP1584 5 V → lojik
  (1N5819 ile DevKitC 5V). Her rayda 2200 µF, batarya barasında 1000 µF; 16/20 AWG silikon, XT30. Şarj: dengeli
  şarj cihazı (iMAX B6AC sınıfı). Düşük gerilim kesmesi 3.5 V/hücre (firmware turunda).
- **Mekanik**: ölçek s = 1.282 (stand keyframe'inde kafa üstü 350 ± 5 mm, micro'nun eklem noktaları düzgün
  ölçeklenir). **Kullanıcı kararı**: geniş duruş kalır (uyluklar taraf başına ~14–22 mm dışarı). Yazıcı
  **256 × 256 × 256 mm**, TPU var: yapısal PETG, kabuklar PLA, taban TPU 95A. Isıl ek somun yok → M3 somun
  yuvası. DS eklemleri: baskı beşik + U-link, eksenle aynı hizada arka idler (F623ZZ, M3 mil); 25T alüminyum/plastik
  kol baskı linke gömülür (Motorobit 25T kol). hip_pitch horn'u bir diş kaydırılır (+14.4°) → aralık −75…+90°.
  Kabuklar sabit et kalınlığıyla ölçeklenir (kafa kütlesi s² ile büyür; boyun/kafa yükü azalır).

### 3.2 Yazılım bileşenleri

1. **CAD** (`cad/openduck.py` + `cad/servos.py` yeni): servo türü kayıt defteri (gövde/tab/delik/çıkış mili/spline/
   kablo/kütle/aktüatör sınıfı, `research/servo-mechanics.md`'deki zarflar) ve eklem → servo türü haritası; tek
   `scale` parametresi; braket/beşik/idler üreteçleri servo zarfından türetilir (sabit mm literalleri yerine);
   `configs/tr35_b2.json` (ölçek, servo haritası, horn ofsetleri, et kalınlıkları). Eklem adları, eksenleri, sıra ve
   işaret kuralı (pozitif açı = horn'a bakarken CCW) micro ile aynı kalır (RL uyumluluğu).
2. **Dışa aktarma** (`cad/export.py`): sınıf başına MJCF `position` aktüatörü (`ds32xx`, `es3054`, `mg90s`, `sg90`),
   eklem sınırları = servo hareket aralığı ∩ çarpışmasız tarama, BOM (Türkiye mağaza/fiyat/URL/stok tarihi),
   baskı seti ve teknik resimler (`docs.py`, 256 mm tabla kontrolü).
3. **Aktüatör modeli** (`rl/actuators.py` yeni, `robot_model.build_spec` içinde MjSpec ile uygulanır, CAD'siz
   güncellenebilir): kp = τs / P-bandı, forcerange = τs (ray geriliminde), eklem sönümü b = τs/ω0 (geri-EMK),
   armature/frictionloss. Varsayılan (Adım 0'a kadar, D1): DS3230MG Blue 2.88 N·m, 5.72 rad/s, P-bandı 3°.
   Ölü bant (3–10 µs) env'de alt-adım ctrl şekillendirmesiyle; boşluk (0–1.5°) DR'de konum gürültüsü olarak.
4. **Sensör modeli** (`rl/sensors.py` yeni, firmware'in birebir kopyalayacağı tek tanım): bacak + boyun q =
   potansiyometre (bölüm başına bias ±1.5°, yük akımına bağlı toprak-düşümü biası, 12-bit kuantalama, RC τ 3.2 ms,
   1 alt-adım gecikme); qd = 200 Hz fark + IIR (α 0.47); kafa q = komut edilen (kırpılmış, eğim sınırlı) ctrl,
   qd = Δctrl/CTRL_DT. Ödüller ve kritik gerçek q'yu görür. Aksiyon gecikmesi U{0..2} alt-adım (DS 300 Hz),
   U{0..4} (50 Hz servolar).
5. **Görevler** (`rl/config.py`, `rl/tasks.py`, `rl/task_math.py`): sabitler TR-35 için yeniden türetilir
   (`research/rl-map.md` tablosu): yükseklikler/F tablosu/SEAT/HEAD_SIT CAD'den bir araçla (phys-sit family3 portu)
   üretilir; zaman ölçekleri Froude (√s) × hız katsayısı κ. **GAIT_FREQ = 2.21·κ Hz**; eğitim κ = 0.6 (1.32 Hz)
   ile başlar, Adım 0 hız kapısı (critic R2) κ'yı belirler. Tork cezası servo stall'ına göre normalize edilir.
   `getup`: kafa-yaw savurmalı eski strateji kaldırılır; güçlü boyun + bacaklarla yeni strateji RL ile öğrenilir
   (bölüm 12 s), poz bankaları yeni modelden.
6. **Politika boyutu**: aktör 256-128-64 (her görev), kritik büyük kalabilir. Dışa aktarma: ONNX + C dizisi
   (normalizasyon katlanmış, float32, ~214 KB); çıkarım maliyeti MAC sayısıyla raporlanır (hedef < 2 ms tahmini).

### 3.3 Veri akışı

`configs/tr35_b2.json` → FreeCAD build (`tools/build.sh`) → `build/` (STEP, STL, print set, drawings, BOM,
`build/sim/openduck.xml`) → `rl/robot_model` + `rl/actuators` + `rl/sensors` → MJX eğitimi (RunPod, onaylı) →
`runs/<ad>/policy.onnx` + `policy_c.h` → `rl/eval_tasks` (kabul) → (sonraki tur) firmware.

## 4. Kısıtlar ve varsayımlar

- Bütün satın alınan parçalar Türkiye'de stokta (doğrulama tarihi BOM'da). DS servolar için Direnc'te 15 adet
  (marka/sürüm belirsiz), Motorobit'te 48 (DSSERVO, 180°). 9 + 2 yedek = 11 adet → Motorobit varsayılan kaynak.
- Servo hızları üç farklı yayımlanmış değerde (0.12/0.14/0.18 s/60°); simülasyon en kötüsünü (DSSERVO Blue) ve
  geniş DR'yi kullanır: tork ×(0.7–1.15), ω0 ×(0.75–1.4), P-bandı 1–8°, ölü bant 3–10 µs, kütle ×(0.9–1.15).
- Kütle bütçesi ≤ 1.90 kg (servo 674 g, baskı ~500 g, elektronik + LiPo ~420 g, idler/horn/vida ~210 g).
- 180° DS servo; eklem sınırları servo hareketinin içinde 5° pay bırakır.
- Mevcut RL hattı (MJX + Brax PPO, CPU torch, eval/play araçları, RunPod başlatıcı) korunur; değişiklikler sabit,
  model, aktüatör ve sensör katmanında.

## 5. Riskli senaryolar (edge case'ler)

1. **Diz ısınması**: 1.9 kg'da diz RMS ~1.0 N·m, stall/3 0.96 N·m (Blue). → Simde RMS ölçütü (§6 A4) ve
   stall-normalize tork cezası; tutmazsa κ düşer ya da DS3230 Red/DS3225 kararı Adım 0'a bırakılır.
2. **hip_yaw (ES3054) aşırı yük**: eski politika gereğinden 10× fazla yaw torku kullanıyordu → ceza + ölçüt.
3. **Potansiyometre toprak düşümü**: yük altında ~0.9°/A bias → sensör modelinde akıma bağlı bias, CAD'de kablo
   kanalları 20 AWG + yıldız toprak için yer.
4. **Servo hareket aralığı dışına politika aksiyonu**: ctrlrange = eklem sınırı; aksiyon kırpılır, sınıra
   çarpma cezası.
5. **Düşüşte kırılma**: MG90S iç dişlileri plastik; kafa kabuğu darbe emici bölgelerle, kalkma politikasına
   kafa darbe cezası.
6. **Kablolar eklemlerden geçiş**: her DS 4 tel (3 + wiper); hip_yaw/hip_pitch/knee geçişlerinde kanal ve gevşeklik
   (G5) CAD'de modellenir; çarpışma taramasında kablo kanalları dahil.
7. **PCA9685 osilatör sapması**: prescale 19 yalnız ölçüm ≤ 27 MHz ise; simülasyonda DS komut çözünürlüğü 0.8 µs.
8. **Stok tükenmesi**: BOM her satır için yedek kaynak listeler; DS32xx ailesi gövde uyumlu olduğundan CAD değişmez.

## 6. Kabul kriterleri

- **A1 CAD**: `tools/build.sh --check` hatasız; tek eklem taramasında eklem sınırları içinde 0 çarpışma; aynı link
  içi 0 çakışma; tüm baskı parçaları 256 × 256 × 256 mm'ye sığar; stand'da kafa üstü 350 ± 5 mm; toplam kütle
  (vida/horn/idler tahminleri dahil) ≤ 1.90 kg.
- **A2 BOM**: her satın alınan parça için mağaza, TL fiyat, URL, stok ve doğrulama tarihi; zorunlu toplam
  ≤ 20.000 TL, yedeklerle ≤ 24.000 TL; her satırda yedek kaynak ya da "yedek yok" notu.
- **A3 Model**: MJCF MuJoCo ve MJX'te yüklenir; 4 aktüatör sınıfı; HOME hedefleriyle açık döngü 5 s ayakta
  (gövde eğimi ≤ 2°); CPU (C) ve MJX adım adım tutarlılık testi geçer.
- **A4 Yürüme** (sim, DR + itme açık, 64 bölüm × 20 s): düşme ≤ %3; vx 0.15 m/s komutunda ölçülen ≥ 0.12 m/s;
  dönüş komutunu ±%20 içinde izler; diz |τ|/stall p99 ≤ 0.85 ve RMS ≤ stall/3; hip_yaw RMS ≤ 0.11 N·m.
- **A5 Boy + otur/kalk** (sim, DR): boy RMS hatası < 1.5 cm; otur ≤ 4.5 s, kalk ≤ 4.0 s, her biri ≥ %95 başarı.
- **A6 Düşünce kalkma** (sim, DR, düşmüş bankadan): 12 s içinde ≥ %80 genel başarı; kafa-zemin tepe kuvveti
  medyanı raporlanır. Tutmazsa nedeni ve önerilen tek donanım/strateji değişikliği raporlanır (spec'e dönüş).
- **A7 Konuşlandırılabilirlik**: aktör 256-128-64; ONNX + C dizisi dışa aktarımı; C ağırlıklarıyla host'ta
  ONNX'e eşit çıktı (≤ 1e-5); MAC sayısı ve tahmini ESP32-S3 süresi raporlanır.
- **A8 Öz-çarpışma**: değerlendirme rollout'larında görsel mesh girişimi ≤ 1 mm³ (eklem arayüzü toleransı ile).
- **A9 Testler**: CPU ve MJX test takımları geçer; yeni modüller (`servos`, `actuators`, `sensors`) birim testli.

## 7. Adım 0 (kullanıcı, paralel): tezgâh testi

1 × DS3230MG (Motorobit, DSSERVO) + 1 × DS3225MG (Robocombo) + 1 × ES3054 + PCA9685 + ESP32-S3 (~3.5k TL).
Ölç: 6.5 V boşta hız (60° süresi), stall akımı, hareket aralığı/µs, ölü bant, potansiyometre teli (gerilim aralığı ≥ 1 V,
monoton), basamak tepkisi. Kapı (critic R2): ω0 ≥ 7.04 rad/s → κ 0.72; ≥ 5.87 → κ 0.6; ≥ 4.89 → κ 0.5.
Sonuçlar `rl/actuators.py` parametrelerine işlenir ve eğitim yenilenir.
