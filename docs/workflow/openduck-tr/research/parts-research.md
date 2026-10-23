**Özet:** Türkiye'de stokta olan hiçbir servo, mevcut 2.5 Hz yürüyüşte istenen 2× tork payını vermiyor. Sınırı hız koyuyor. En iyi seçenekler (DS3230MG ve ST3025) 33–40 cm boyda yalnızca 1× pay veriyor. Bu yüzden yürüyüş daha yavaş bir frekansla yeniden eğitilmeli. Önerim B2: yaklaşık 20.7k TL, bütün parçaların stoğu doğrulandı. Üç adayın zorunlu toplamları: B1 15,308 TL, B2 20,683 TL, B3 59,874 TL.

# Türkiye stoğundan RL ördek robot – sentez (stok/fiyat tarihi 2026-10-23, fiyatlar KDV dahil TL)

## 0. Önce bilinmesi gerekenler

**Hesap yöntemi.** u değerleri benim tahminim, simülasyonda koşulmadı. Tork ajanının ölçeklendirme tablosunun 17 noktasına ±0.06 içinde uyan basit bir model kullandım:
`u ≈ max(0.02/τe + 8.8/ωe, 0.13/τe + 7.7/ωe, 0.5/τe)`
- τe = stall / k_bacak, ωe = ω0·√s, s = H / 273 mm
- k_bacak = (m / 0.676) · s, m robotun kütlesi
- Okuma: u ≤ 0.5 ise 2× pay var, u ≤ 1 ise pay olmadan çalışıyor.

**1. 2× pay şu anki yürüyüşle mümkün değil.** Formüldeki hız terimi (8.8/ωe) tek başına u ≤ 0.5 için ω0·√s ≥ 17.6 rad/s istiyor. Stoktaki en hızlı servo 13.1 rad/s (MG90S ve ES09MD) ve bunların torku zaten yetmiyor. Tork ajanı da aynı sonuca vardı: 22–47 cm arasında hiçbir sınıf 2× vermiyor.

**2. Micro (9 g) PWM servo ile ördek her boyda başarısız.** MG90S 22 cm'de u 1.26, SG90 1.31, ES08MA 1.31, FS90-FB yaklaşık 1.78 çıkıyor. Bu yüzden "en ucuz micro PWM" yapıyı reddettim.

**3. Ne çalışır?** Diz, hip_roll, hip_pitch ve ayak bileğinde 40 mm sınıfı güçlü bir servo, düşük yüklü eklemlerde hafif servo. Boy yaklaşık 35 cm. Bu hibrit yapı mevcut yürüyüşle u ≈ 0.91–0.95 veriyor, yani payı yok.
- Yürüyüş frekansı 2.5'ten 1.8 Hz'e düşürülüp yeniden eğitilirse u ≈ 0.7 bekliyorum. Bu da kaba bir tahmin: eklem hızlarının yaklaşık 0.72 katına ineceğini varsaydım.
- Bacak servoları en yüksek gerilimde sürülmeli, çünkü hız bağlayıcı. DS3230MG 6.0 V'ta u 1.04 iken 6.5 V'ta 0.95.

**4. Politika yapısı.** Politika 10 bacak eklemini sürüyor, 14 eklemi gözlüyor (bacaklar + 4 kafa eklemi), gaga politikanın dışında.
- Konum geri beslemesi en çok bacaklarda lazım.
- Kafa eklemleri az yüklü. Bunlarda komut edilen açıyı gözlem olarak vermek kabul edilebilir.
- Gagayı çıkarmanın RL'ye etkisi yok. head_roll'u çıkarmak gözlem boyutunu 49'dan 47'ye düşürür, ama yeniden eğitim zaten gerekiyor.

---

## A. Türkiye'de stoğu doğrulanmış servolar

Bu tabloda yalnızca iki kontrolün uyuştuğu ya da doğrulayıcının teyit ettiği servolar var. ω0 boşta hız (rad/s).

Son üç sütun:
- **2× (mevcut yürüyüş):** 2× payla desteklenen en büyük boy.
- **Tork-yalnız 2×:** yalnız diz torku hesabıyla en büyük boy (ölçeklendirme tablosunun kütle modeliyle).
- **1× aralığı:** u ≤ 1 olan boy aralığı (tahmin).

| Servo | TL/adet | En ucuz doğrulanmış mağaza (stok) | Kullanılacak V → stall | ω0 | Kütle | Geri besleme | Arayüz | 2× (mevcut yürüyüş) | Tork-yalnız 2× | 1× aralığı |
|---|---:|---|---|---:|---:|---|---|---|---|---|
| SG90 | 54.00 | Robocombo (2491) | 6 V → 0.18 N·m | 10.5 | 9 g | yok | PWM | yok | yok | yok (22 cm'de 1.31) |
| MG90S (klon) | 108.00 | Robocombo (141); Robotistan 125.73 (542) | 6 V → 0.22 | 13.1 | 13.4 g | yok | PWM analog | yok | yok | yok (22 cm'de 1.26) |
| SG92R | 115.21 | Komponentci ("Var", adet yazmıyor); Robolink 190.50 (21) | 4.8 V → 0.25 | 10.5 | 9 g | yok | PWM | yok | yok | yok (~1.06) |
| ES08MA II metal | 424.20 | Motorobit (133) | 6 V → 0.18–0.20 | 10.5 | 12 g | yok | PWM analog | yok | yok | yok (22 cm'de 1.31) |
| ES08MD II | 606.00 | Motorobit (164) | 6 V → 0.20 | 10.5 | 13.4 g | yok | PWM dijital | yok | yok | yok (~1.36) |
| ES09MD | 896.50 | Robocombo (31) | 6 V → 0.25 | 13.1 | 14.8 g | yok | PWM dijital | yok | yok | yok (22 cm'de ~1.13) |
| ES3054 | 735.50 | Robocombo (76) | 6 V → 0.34 | 8.1 | 20 g | yok | PWM dijital | yok | yok | yok (~1.32), yardımcı eklemler için |
| FS90-FB | 358.45 | Robotistan (23) | 6 V → 0.15 | 10.5 | ~13 g | analog (4. kablo, ADC gerekir) | PWM | yok | yok | yok (~1.78) |
| MG996R | 200.50 | Robocombo (46); Robotistan 234.53 (775) | 6 V → 1.08 | 7.5 | 55 g | yok | PWM | yok | yok | yok (27–33 cm'de 1.24–1.28) |
| MG995 | 200.50 | Robocombo (53); Robotistan 210.36 (1145) | 6 V → 1.08 (iddia) | 6.5 | 55 g | yok | PWM | yok | yok | yok (~1.40) |
| DS3218MG | 971.50 | Robocombo (53) | 6.8 V → 2.11 | 7.5 | 60 g | yok | PWM, 333 Hz | yok | 27.5 cm (gövde ≥32.5 cm ister) | yok (33–40 cm'de 1.10–1.12) |
| TD-8120MG | 969.60 | Motorobit (165) | 6 V → 2.14 | 7.5 | 65 g | yok | PWM | yok | 27 cm | yok (~1.11) |
| DS3225 | 1031.00 | Robocombo (99) | 6.8 V → 2.40 | 8.1 | 60 g | yok | PWM, 50–333 Hz | yok | 29.5 cm | sınırda (36 cm'de ~1.02) |
| **DS3230MG** | **1046.12** | **Direnc (15)**; Motorobit 1939.20 (48) | 6.8 V → 3.14 (6.5 V'ta ~3.06, 6.0 V'ta ~2.92) | 8.7 (6.5 V'ta 8.3) | 65 g | yok (potansiyometre modu mümkün, bkz. B2) | PWM | yok | 33.5 cm | **29–45 cm** (38 cm'de en iyi 0.91; gövde ≥32.5 cm) |
| DS3235 | 1171.50 | Robocombo (134) | "35 kg" sınıfı, spec doğrulanmadı | ? | ? | yok | PWM | – | hesaplanmadı | hesaplanmadı |
| FT5335M-FB | 2064.87 | Robotistan (16) | 7.4 V → 3.9 | 5.8 | 180 g | analog | PWM | yok | 23 cm | yok (~1.37) |
| SC15 | 1318.85 | SAMM (**yalnız 10**) | 7.4 V → ~1.59 (6 V'ta 1.47, 8.4 V'ta 1.67) | 6.5 (6 V'ta; 7.4 V hızı doğrulanmadı) | 70 g | **evet**: potansiyometre 1024/210°, yük/hız/gerilim | TTL yarı-dupleks, SCS | yok | 21 cm | yok (~1.19–1.33), yardımcı eklemler için |
| **ST3025** | **6564.41** | **SAMM (44)** | 12 V → 3.9 | 8.9 | 124 g | **evet**: 12-bit manyetik enkoder (0.088°), akım/sıcaklık/gerilim | TTL, STS | yok | 29.5 cm (sığmaz) | **28–43.5 cm** (38 cm'de en iyi 0.93; gövde ≥32 cm) |
| AX-12A | 4627.50 (6'lı paket 27,765.15) | robotsepeti (1 paket = 6 adet) | 12 V → 1.5 | 6.2 | 54.6 g | potansiyometre 1024/300° | TTL, Protocol 1.0 | yok | 23 cm | yok (~1.37); 6 adet yetmez |
| XL430-W250-T / 2XL430 | kit başına yaklaşık 5.7–8.5k TL/DOF | robotsepeti, Engineer Kit 1 (1 adet) / Kit 2 (4 adet) | 12 V → 1.5 | 6.4 | 57 g | 12-bit | TTL, Protocol 2.0 | yok | – | yok (~1.34); 17 DOF ≈ 111k TL |

**Elenenler:**
- **Stokta yok:** bütün ST3215 sürümleri, ST3020, ST3235, CF35-12, XL330 (M288 ve M077), XL430 tek satış, XL-320.
- **Türkiye'de satılmıyor:** STS3032, SCS0009, MG92B.
- **Adet yetersiz:** SER0046 (4), Goteck/SER0011 (5), ES09MA (6), FS5115M (7), RSBL45 (3), XM430 (2).
- **Fiyat absürt:** MX-28T 26,759 TL/adet. Görünen stok 997, büyük ihtimalle sanal.
- **Pazaryerleri** (Trendyol, Hepsiburada, n11, Akakçe): erişilemedi, stok bilinmiyor.

---

## B. Aday yapılar

Üç yapı da 35 cm (s = 1.282) ve 15 DOF. Hepsi aynı ölçekte bir 3B gövdeye oturuyor, yalnız servo yuvaları farklı. Mevcut CAD XL330'a (34 mm gövde) göre yapıldığı için yeni bir CAD gerekiyor.

**Baskı miktarı (her üç yapı):** 296 g × s³ = yaklaşık 624 g baskı.
- Sert parçalar yaklaşık 530 g, TPU yaklaşık 95 g.
- Hurda ve destek için %30 eklersem: PETG yaklaşık 700 g, TPU yaklaşık 120 g.
- Bu yüzden 2 × 1 kg PETG (biri yeniden basım yedeği) ve 1 × 1 kg TPU listede.

**Vida ve ısıl ek somun:** Ek somun hiçbir dükkânda stokta değil (doğrulanmadı). Gövdeyi M3 somun yuvası ya da kendinden kılavuzlu vida ile tasarlayın.

### B1 – "Ekonomik": PWM hibrit, geri beslemesiz, tek denetleyici ESP32-S3

- **Kütle:** yaklaşık 1.62 kg (tahmini). Servolar 629 g, baskı 624 g, elektronik yaklaşık 150 g, batarya yaklaşık 215 g.
- **Eklem ataması:**
  - DS3230MG ×8: hip_roll, hip_pitch, knee, ankle (her iki bacak)
  - ES3054 ×3: hip_yaw ×2, neck_pitch
  - MG90S ×3: head_pitch, head_yaw, head_roll
  - SG90 ×1: gaga
- **Diz:** τe 1.00 N·m. Bu tork olarak 2.0× pay, ama hız yüzünden u ≈ 0.95 (6.5 V'ta). Isınma kontrolü geçiyor: diz RMS yaklaşık 0.86 N·m, stall/3 yaklaşık 1.02 N·m.
- **Kalça:**
  - hip_roll (DS3230MG): u ≈ 0.37–0.70.
  - hip_yaw (ES3054): u ≈ 0.76.
- **Kalkma (get-up):** Boyun ve kafa servoları kalkma için gereken torkun yaklaşık 0.5 katında. Kalkma script'i yavaşlatılıp yeniden optimize edilmeli.
- **Kafa/gaga:** 15 DOF'u koruyun. head_roll ve gagayı çıkarmak yalnızca 162 TL kazandırır.

| # | Kalem | Adet | Birim | Tutar | Mağaza / URL | Stok |
|---|---|---:|---:|---:|---|---|
| 1 | DSSERVO DS3230MG | 8 | 1046.12 | 8368.96 | Direnc, https://www.direnc.net/ds3230mg-su-gecirmez-servo-motor | ✓ doğrulayıcı, **yalnız 15 adet** |
| 2 | EMAX ES3054 | 3 | 735.50 | 2206.50 | Robocombo, https://robocombo.com/emax-es3054-3-5kg-metal-disli-dijital-mini-servo-4300 | ✓ 76 |
| 3 | MG90S | 3 | 108.00 | 324.00 | Robocombo, https://robocombo.com/mg90-rc-servo-motor | ✓ 141 |
| 4 | SG90 | 1 | 54.00 | 54.00 | Robocombo, https://robocombo.com/sg90-rc-servo-motor | ✓ 2491 |
| 5 | ESP32-S3-DevKitC-1 N16R8 (politika + IO) | 1 | 537.62 | 537.62 | Komponentci, https://www.komponentci.net/esp32-s3-devkitc-1-n16r8-wifi-ve-bluetooth-gelistirme-karti-pmu12351 | ✓ "Var", adet yok (yedek kaynak Direnc 610.23) |
| 6 | PCA9685 16 kanal | 2 | 179.50 | 359.00 | Robocombo, https://robocombo.com/pca9685-16-kanal-12-bit-pwm-servo-motor-surucusu | ✓ 99 |
| 7 | MPU6500 IMU (SPI) | 1 | 136.35 | 136.35 | Motorobit, https://www.motorobit.com/mpu6500-6-axis-acceleration-and-gyro-sensor-gy-6500 | ✓ (sayfa verisinde STOCK 75) |
| 8 | Aspilsan 18650 2800 mAh, 14 A (4S1P) | 4 | 124.26 | 497.04 | pilpaketi, https://www.pilpaketi.com/aspilsan-pil | ✓, adet yok |
| 9 | 18650 2'li yuva | 2 | 19.50 | 39.00 | Robocombo, https://www.robocombo.com/18650-2li-pil-yuvasi-kablolu-3353 | ✓ 1046 |
| 10 | 2S 13 A BMS (her 2S pakete bir tane) | 2 | 70.10 | 140.20 | Komponentci, https://www.komponentci.net/2s-13a-bms-18650-lityum-pil-sarj-koruma-devresi-pmu6859 | ✓ |
| 11 | XL4015 5 A buck (4 bacak rayı 6.5 V + 1 mikro rayı 6.0 V) | 5 | 83.39 | 416.95 | Komponentci, https://www.komponentci.net/xl4015-5a-ayarlanabilir-dc-dc-voltaj-dusurucu-pmu3119 | ✓ |
| 12 | MP1584 5 V (lojik besleme) | 1 | 51.87 | 51.87 | Komponentci, https://www.komponentci.net/mp1584en-5v-3a-ultra-kucuk-boyutlu-dc-dc-step-down-voltaj-regulator-modulu-pmu61596 | ✓ |
| 13 | XT30 erkek-dişi çift | 2 | 17.33 | 34.66 | Komponentci, https://www.komponentci.net/xt30-li-po-pil-erkek-disi-soket-takim-konnektor-pmu6847 | ✓ |
| 14 | Elas renk geçişli PETG 1 kg (renk rastgele) | 2 | 300.00 | 600.00 | elas3d, https://elas3d.com/elas-3d-renk-gecisi-petg-1-kg-koyu-gri-tonlarda | ✓ doğrulayıcı, stockCount 10 |
| 15 | Creality CR-TPU 1 kg (ayak tabanı) | 1 | 1072.50 | 1072.50 | Robolink, https://www.robolinkmarket.com/creality-cr-tpu-siyah-filament-175mm-1000gr | ✓ (95A sertliği doğrulanmadı) |
| 16 | M3 YHB vida-somun-pul seti, 415 parça | 1 | 469.50 | 469.50 | Robolink, https://www.robolinkmarket.com/m3-yhb-vida-somun-ve-pul-seti-415-parca | ✓ |
| | **ZORUNLU TOPLAM** | | | **15,308.15** | | |

Önerilen ekler (toplam 3,528.26 TL, B1 dahil **18,836.41 TL**):
- Yedek servolar: DS3230MG 1046.12 + ES3054 735.50 + MG90S 108 + SG90 54.
- 693ZZ rulman ×8, 89.33 TL/adet, servonun karşı tarafındaki pivot için. Rulmanal, https://www.rulmanal.com.tr/urun/693-zz-kwc-3-8-4-rulman (55 stok).
- M2 YSB vida seti 870 TL. Robocombo, https://www.robocombo.com/m2-ysb-vida-seti-1375-parca-kutulu-4643

**Doğrulanmadı (fiyat yok):**
- 2S şarj cihazı (iki paket ayrı ayrı şarj edilecek)
- Silikon kablo (16–22 AWG), servo uzatma kabloları
- Her raya 1000–2200 µF kondansatör, sigorta ve anahtar
- İki 2S BMS'in seri bağlanması (üretici onayı yok)
- Yaylı yuvaların akım sınırı
- DS3230MG'nin 200–333 Hz PWM desteği (doğrulanana kadar 50 Hz)
- Politikanın ESP32-S3'te çalışma süresi. Ağ 49→256→128→64→10, adım başına yaklaşık 54k MAC; tahminim <3 ms. onnxruntime yok, ağırlıkları C dizisine ya da TFLite-Micro'ya çevirmek gerekiyor.
- Servo vidalarının kutudan çıkıp çıkmadığı

**Bağlantı notları:**
- PCA9685 kartının V+ hattını kullanmayın. Kart yalnız sinyal ve ortak GND versin; servoların kırmızı kablosu ilgili XL4015 rayına gitsin.
- Rail dağılımı: her bacakta iki ray (hip_roll + hip_pitch, knee + ankle).

### B2 – "Orta" (önerilen): B1 + 10 bacak ekleminde potansiyometre geri beslemesi + Pi 4

- **Mekanik:** B1 ile aynı. Kütle yaklaşık 1.69 kg, diz u ≈ 0.94.
- **Geri besleme:** 10 bacak servosunda potansiyometrenin orta ucundan bir kablo çıkıyor (8 DS3230MG + 2 ES3054). Bu kablolar ESP32-S3'ün ADC1 kanallarına gidiyor; GPIO1–10, tam 10 kanal.
- **Görev dağılımı:**
  - ESP32-S3 IO hub olarak çalışır: ADC oversampling, SPI IMU, I2C PCA9685, 200–500 Hz.
  - Pi 4 USB-CDC üzerinden durumu alır, ONNX politikayı değiştirmeden 50 Hz koşar, log tutar ve sysID yapar.

| Satır | B1'e göre değişiklik | Tutar |
|---|---|---:|
| + | Raspberry Pi 4 3GB, Robotistan, https://www.robotistan.com/raspberry-pi-4-3gb-versiyon (✓ doğrulayıcı, sipariş başına en fazla 2, adet yok) | 5342.91 |
| ± | XL4015 ×6 (beşinci yerine altıncı: Pi rayı 5.1 V), MP1584 çıkar | +83.39 / −51.87 |
| | **ZORUNLU TOPLAM** | **20,682.58** |

- **Önerilen ekler:** B1'deki ekler + OV5647 kamera 326.41 TL (Robotistan, https://www.robotistan.com/raspberry-pi-3-ve-4-uyumlu-kamera-modulu). Toplam 3,854.67, B2 dahil **24,537.25 TL**.
- **Doğrulanmadı:**
  - DS3230MG'de gerçekten potansiyometre olduğu (önce 1 servoyu açın)
  - Orta uç gerilim aralığı (3.1 V'u geçerse bölücü direnç gerekir)
  - İnce kablo ve dirençler
  - microSD kart
- **Pi'siz varyant:** Politika ESP32'de koşar, maliyet B1 ile aynı: 15,308.15 TL.

### B3 – "Tam geri beslemeli bus-servo hibrit"

- **Kütle:** yaklaşık 2.25 kg. Servolar 1163 g, baskı 624 g, elektronik yaklaşık 245 g, batarya yaklaşık 215 g.
- **Eklem ataması:**
  - ST3025 ×6: hip_pitch, knee, ankle
  - SC15 ×5: hip_yaw ×2, hip_roll ×2, neck_pitch
  - ES3054 ×3 (PWM): head_pitch, head_yaw, head_roll
  - SG90: gaga
- **Diz:** τe 0.92 N·m (1.83×), u ≈ 0.91 (12 V regüleli ray). Isınma kontrolü geçiyor: RMS yaklaşık 1.19 N·m, stall/3 yaklaşık 1.30 N·m.
- **Kalça:** SC15 hip_roll u ≈ 0.57, ama DR + itme altında 0.98. Sınırda; SC15'in 7.4 V hızı doğrulanmadı.
- **Kalkma:** SC15 boyunda kalkma için yaklaşık 2× pay var. ES3054 kafa servolarında kalkma yaklaşık 0.45×, başarısız.

| # | Kalem | Adet | Birim | Tutar | Mağaza / URL | Stok |
|---|---|---:|---:|---:|---|---|
| 1 | Waveshare ST3025 40 kg | 6 | 6564.41 | 39386.46 | SAMM, https://market.samm.com/serial-bus-yuksek-hassasiyetli-programlanabilir-servo-motor-st3025-40kg-1 | ✓ 44 |
| 2 | Waveshare SC15 17 kg | 5 | 1318.85 | 6594.25 | SAMM, https://market.samm.com/sc15-17kg-yuksek-torklu-programlanabilir-seri-haberlesmeli-servo-motor | ✓ **yalnız 10** |
| 3 | EMAX ES3054 | 3 | 735.50 | 2206.50 | Robocombo (yukarıdaki URL) | ✓ 76 |
| 4 | SG90 | 1 | 54.00 | 54.00 | Robocombo | ✓ |
| 5 | Raspberry Pi 4 3GB | 1 | 5342.91 | 5342.91 | Robotistan (yukarıdaki URL) | ✓ doğrulayıcı |
| 6 | Waveshare Bus Servo Adapter (A) (ST bus ve SC bus için ayrı ayrı) | 2 | 328.22 | 656.44 | SAMM, https://market.samm.com/seri-bus-servo-surucu-ve-kontrol-karti | ✓ 18 (SC bus için gerilim notu aşağıda) |
| 7 | PCA9685 (4 PWM kanalı, 50 Hz) | 1 | 179.50 | 179.50 | Robocombo | ✓ 99 |
| 8 | GY-BNO085 IMU | 1 | 1999.80 | 1999.80 | Motorobit, https://www.motorobit.com/gy-bno080-bno085-9-axis-high-precision-acceleration-and-gyro-sensor | ✓ (STOCK 49) |
| 9–11 | Aspilsan ×4 + 2'li yuva ×2 + 2S BMS ×2 (4S1P) | – | – | 676.24 | B1 ile aynı | ✓ |
| 12 | XL4015 ×7: 4 × 12.0 V bacak rayı, 1 × 7.4 V SC rayı, 1 × 6.0 V PWM rayı, 1 × 5.1 V Pi | 7 | 83.39 | 583.73 | Komponentci | ✓ |
| 13 | XT30 çift | 3 | 17.33 | 51.99 | Komponentci | ✓ |
| 14–16 | PETG ×2 + CR-TPU + M3 seti | – | – | 2142.00 | B1 ile aynı | ✓ |
| | **ZORUNLU TOPLAM** | | | **59,873.82** | | |

- **Önerilen ekler:** SC15 yedek 1318.85, ES3054 yedek 735.50, SG90 yedek 54, 693ZZ ×6 535.98, M2 seti 870, OV5647 326.41. Toplam 3,840.74, B3 dahil **63,714.56 TL**. ST3025 yedeği (6,564.41) bu toplama dahil değil.
- **SC bus adaptör riski:** Adapter (A) 9–12.6 V girişi bildiriyor; 7.4 V'luk SC bus bunun altında, spec dışı. Önce adaptörü yalnız veri yolu olarak, servo gücünü ayrı vererek test edin. Olmazsa doğrulanmış yedek: U2D2 3555.52 (5 stok) + U2D2 Power Hub 2051.76 (3 stok, giriş aralığı doğrulanmadı), robotsepeti, https://www.robotsepeti.com/robotis-u2d2 ve https://www.robotsepeti.com/u2d2-power-hub-board
- **ST bus alternatifi:** Waveshare ESP32 Bus Servo Driver HAT, Robocombo 1255 TL (18 stok), 9–25 V giriş.
- **Varyantlar:**
  - Kafa da bus'ta (9 × SC15): +2,835.40 TL, kafa yaklaşık 210 g ağırlaşır.
  - Kalçalar da ST3025 (10 × ST3025): +20,982.24 TL, +216 g, SC bus'a bacakta gerek kalmaz.
- **Doğrulanmadı:**
  - 3S doğrudan besleme seçeneği (3S yuva ve BMS bulunamadı). Bunun yerine 4S → 12 V regüleli raylar seçildi.
  - XL4015'in 3 × ST3025 tepe akımına (5 A sınırı) yetip yetmediği
  - ST3025 ve SC15 vida ölçüleri
  - microSD, şarj cihazı, kablolar

---

## C. RL sim-to-real açısından her yapının artısı ve eksisi

**Üç yapıda da gereken iş:**
- Yeni 35 cm MJCF'de her eklem sınıfı için ayrı aktüatör class tanımlanmalı. Mevcut tek `openduck` default class yetmez. Gerçek değerler (damping = stall/ω0):
  - DS3230MG (6.5 V): ±3.06 N·m, damping ≈ 0.37
  - ST3025: ±3.9 N·m, damping ≈ 0.44
  - SC15: ±1.59 N·m, damping ≈ 0.24
  - ES3054: ±0.34 N·m, damping ≈ 0.042
  - MG90S: ±0.22 N·m, damping ≈ 0.017
- Yeniden eğitim: GAIT_FREQ yaklaşık 1.6–1.8 Hz, daha dar komut aralığı, tork cezası.
- Kalkma keyframe'leri yavaşlatılıp yeniden optimize edilmeli.
- Ardından tork script'leri (`walk_torque.py`, `scaling.py`) yeni modelle tekrar koşulup u doğrulanmalı.
- Tork ajanının minimum tork değerleri üst sınır, çünkü 0.96 N·m servoyla eğitilmiş bir politikadan çıktılar.

**B1 (geri beslemesiz):**
- **En büyük açık bu.** Politika q ve qd gözlüyor. Bunların yerine "komut + gecikme modeli" ya da yalnız aksiyon geçmişi vermek gerekir.
- Hobi servoların iç denetleyicisi sert bir P kontrolcü ve kazancı bilinmiyor. Simdeki yumuşak kp 1.1 N·m/rad ile uyuşmaz, ve geri besleme olmadan sysID yapılamaz.
- **Deadband:** DS3218 için 3 µs (≈0.27°) yazıyor, DS3230MG için yazmıyor. MG90S 5 µs, SG90 10 µs.
- **Boşluk (backlash):** hiçbir listelemede yok. Robotistan, MG90S'te iç dişlilerin plastik olduğunu söylüyor. Simde DR ile en az ±1° modelleyin.
- **PCA9685 çözünürlüğü:** 50 Hz'de 4.9 µs/adım ≈ 0.44°. 200–333 Hz desteklenirse adım yaklaşık 1 µs'ye iner.
- **Gerilim:** Raylar regüleli, yani düşen batarya hızı değiştirmez. Ama XL4015'in 5 A sınırında tepe akımında ray çöker ya da tork düşer. Lojik ayrı rayda olduğu için yeniden başlatma olmaz.

**B2 (potansiyometre modu):**
- Bacaklarda konum ölçümü gelir. ESP32-S3'ün 12-bit ADC'si doğrusal değil ve gürültülü; kalibrasyon ve oversampling ile ±0.2–0.5° bekliyorum (tahmin). Eğitimdeki eklem konum gürültüsü 0.01 rad (0.57°), uyumlu.
- qd türevle hesaplanır ve gürültülü olur (eğitimdeki gürültü 0.5 rad/s). 20–30 Hz alçak geçiren filtre gerekir, bu da gecikme ekler.
- **Asıl kazanç sysID:** Basamak ve chirp testleriyle kp, forcerange, damping ve deadband'i ölçüp MJCF'e işleyebilirsiniz, repodaki BAM fit'e benzer şekilde.
- Kafa servoları komutla gözlenir.
- **Riskler:**
  - Elle mod gerektiriyor ve su geçirmezliği bozuyor.
  - DS3230MG'de potansiyometre olduğu doğrulanmadı.
  - Akım ve tork ölçümü yok.

**B3 (bus-servo):**
- **Konum ölçümü:** Büyük 6 eklemde gerçek 12-bit enkoder (0.088°). SC15'te potansiyometre (0.2°).
- **Ek telemetri:** yük, akım, sıcaklık ve gerilim. Bu hem sysID hem güvenlik için değerli.
- **Ayarlanabilir kontrolcü:** P/D kazancı, tork sınırı ve dead-zone register'ı ayarlanabiliyor. P düşürülerek gerçek servo simdeki yumuşak PD'ye yaklaştırılabilir. Sim-to-real farkı en küçük olan yapı bu.
- **Güncelleme hızı:** 1 Mbps'de sync-write + sıralı okuma ile 6 servo için 2–4 ms bekliyorum (tahmin). İki bus paralel çalıştığı için 50 Hz rahat, 100–200 Hz mümkün.
- **Protokol:** SCS ve STS'nin byte sırası farklı; Feetech SCServo SDK ikisini de destekliyor.
- **Gerilim:** 12 V regüleli raylarla sabit. 3S'ten doğrudan beslenseydi 12.6'dan yaklaşık 10.5 V'a düşerdi; bu durumda bus'tan gelen gerilim değerini gözleme ekleyin ya da DR ile tork ±%20 oynatın.
- **Riskler:**
  - Maliyet B2'nin yaklaşık 3 katı.
  - 2.25 kg; düşüşler ağır olur.
  - SC15 stoğu yalnız 10 adet.
  - Adaptörün gerilim spec'i.
  - 7 buck modülü.
  - Not: Tahmini u değeri B2 ile hemen hemen aynı (0.91 ve 0.94). Ağır servolar pay kazandırmıyor; avantaj ölçüm ve kontrol.

**Tavsiye: B2.**
1. Bütün parçaların stoğu doğrulandı, adetler en az ihtiyaç kadar. En dar kalem Direnc'teki 15 DS3230MG. Yedek kaynaklar: Robocombo DS3225, 1031 TL, 99 adet (u ≈ 1.02, sınırda) ve Motorobit DS3230MG, 1939.20 TL, 48 adet.
2. Diz torkunda 2× pay var, ısınma kontrolü geçiyor, u ≈ 0.94. Yavaş yürüyüşle yaklaşık 0.7 bekleniyor.
3. Politikanın ihtiyaç duyduğu bacak q/qd ölçümü ve sysID, B3'ün yaklaşık üçte bir maliyetine geliyor.
4. Pi 4'te ONNX'i değiştirmeden çalıştırabilirsiniz.
5. Mod tutmazsa aynı donanım B1 olarak kullanılır.

**Uygulama sırası:**
- **Adım 0 – tezgâh testi:** 1 DS3230MG + 1 ES3054 + PCA9685 + ESP32-S3 = 2,498.74 TL. Servoyu açın, potansiyometreyi ve orta uç gerilimini kontrol edin, basamak tepkisini loglayın. Devam/vazgeç kararını burada verin.
- **Adım 1:** DS3230MG siparişini hemen verin.
- **Adım 2:** Ölçülen parametrelerle MJCF'i doldurun, yeniden eğitin, u'yu tekrar ölçün.

Bütçe yaklaşık 60–64k TL'ye çıkabiliyorsa ve orijinal pipeline'a en yakın donanım isteniyorsa (enkoder, akım ölçümü, ayarlanabilir P) B3'ü seçin.

Hesap dosyaları /private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s6/research/synthesis/ klasöründe:
- est.py
- tableA.py
- hybrid.py
- final.py
- cost.py
- cost.txt

Repoda hiçbir şey değiştirilmedi.