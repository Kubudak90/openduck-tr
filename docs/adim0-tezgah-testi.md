# Adım 0: servo tezgâh testi (kullanım kılavuzu)

Bu test, robotun servo seçimini ve simülasyon parametrelerini gerçek ölçümle sabitler (spec §7). Çıktısı
`tools/sysid` ile `rl/actuators.py` parametrelerine ve §7 karar tablosuna dönüşür: κ (yürüyüş hızı), diz servosu,
300 Hz, 3.3 V darbe, pot okuma yöntemi, ray aşımı ve T_diz.

Bir DS servo için tahmini süre: kurulum 1–2 saat (pot kablolama dahil), ölçüm ~45 dk, ısınma testi ~1.5 saat.

- Betik: `tools/bench/` (ESP32-S3 + PCA9685 + INA226 + ADS1115 + NTC).
- PC kaydedici: `tools/bench/logger.py`.
- Uydurucu: `python -m tools.sysid.fit`.
- Satır biçimi: `tools/sysid/logio.py`.

---

## 1. Alışveriş listesi

Fiyat ve stoklar 23.10.2026 tarihli research taramasından (`docs/workflow/openduck-tr/research/`), KDV dahildir.
Almadan önce sayfayı açıp stoğu kontrol edin. Adım 0 alımları robota ya da yedeğe gider ve bütçe tavanına dahildir.

### Servolar (her biri 1 adet; tezgâhta kaybeden yedeğe gider)

| Parça | Mağaza | Fiyat | Stok | Bağlantı |
|---|---|---|---|---|
| DS3225MG | Robocombo | 1.031 TL | 99 | https://robocombo.com/dsservo-ds3225-25kg-servo-motor-180-derece-su-4150 |
| DS3230(MG) | Direnc | 1.046,12 TL | 15 | https://www.direnc.net/ds3230mg-su-gecirmez-servo-motor |
| DS3235 (yalnız diz adayı) | Robocombo | 1.171,50 TL | 134 | https://robocombo.com/dsservo-ds3235-35kg-servo-motor-180-derece-su-4148 |
| DS3230MG (yedek kaynak) | Motorobit | 1.939,20 TL | 48 | https://www.motorobit.com/ds3230mg-30kg-su-gecirmez-dijital-servo-motor |
| EMAX ES3054 | Robocombo | 735,50 TL | 76 | https://robocombo.com/emax-es3054-3-5kg-metal-disli-dijital-mini-servo-4300 |
| MG90S | Robocombo | 108 TL | 141 | https://robocombo.com/mg90-rc-servo-motor |
| SG90 | Robocombo | 54 TL | 2491 | https://robocombo.com/sg90-rc-servo-motor |

### Elektronik (robotta da kullanılır; robot adetlerini şimdi almak mantıklı)

| Parça | Adet (tezgâh / robot) | Mağaza, fiyat | Bağlantı |
|---|---|---|---|
| ESP32-S3-DevKitC-1 N16R8 | 1 / 1 | Komponentci 537,62 TL (yedek: Direnc 610,23) | https://www.komponentci.net/esp32-s3-devkitc-1-n16r8-wifi-ve-bluetooth-gelistirme-karti-pmu12351 · https://www.direnc.net/esp32-s3-n16r8-wifi-bluetooth-modul |
| PCA9685 16 kanal | 1 / 2 | Robocombo 179,50 TL | https://robocombo.com/pca9685-16-kanal-12-bit-pwm-servo-motor-surucusu |
| INA226 modülü | 1 / 1 | Direnc 104,61 TL (yedek: Robotistan 149,91; Motorobit 193,92) | https://www.direnc.net/ina226-iki-yonlu-akim-guc-izleme-sensor-modulu-en |
| ADS1115 modülü (16 bit, I2C) | 1 / 2 | research'te stok/URL **yok**: Robocombo, Direnc ya da Komponentci'de "ADS1115" arayın | — |
| Şönt 2–10 mΩ (örnek: 5 mΩ, 2512, ≥ 1 W) | 1 | research'te stok **doğrulanmadı** | yedek: 20 cm 16 AWG kablo ≈ 2.6 mΩ (kalibre edin, §6.1) ya da akım göstergeli laboratuvar kaynağı |
| 10 kΩ NTC (B3950) + 10 kΩ direnç | 2 + 2 | genel parça (stok doğrulanmadı) | biri servo kasasına, biri ortama |

### Besleme (laboratuvar kaynağınız yoksa; parçalar robota gider)

| Parça | Mağaza, fiyat | Bağlantı |
|---|---|---|
| Profuse 4S 1500 mAh 95C LiPo | Motorobit 2.528,23 TL | https://www.motorobit.com/148v-4s-1500mah-95c-lipo-battery |
| XT60 erkek → XT30 dişi adaptör | Robocombo 54 TL | https://www.robocombo.com/xt60-erkek-xt30-disi-konnektor-4188 |
| XT30 erkek-dişi çift (×2) | Komponentci 17,33 TL | https://www.komponentci.net/xt30-li-po-pil-erkek-disi-soket-takim-konnektor-pmu6847 |
| 300 W / 20 A buck (DS rayı, 6.50 V) | Robocombo 329 TL | https://www.robocombo.com/300w-20a-dcdc-ayarlanabilir-voltaj-dusurucu-modul-2987 |
| XL4015 5 A buck (ES/MG/SG rayı, 6.00 V) | Komponentci 83,39 TL | https://www.komponentci.net/xl4015-5a-ayarlanabilir-dc-dc-voltaj-dusurucu-pmu3119 |
| Bıçak sigorta yuvası + 15 A sigorta | Direnc 30,92 + Motorobit 7,27 TL | https://www.direnc.net/sigota-yuvasi-kablolu-30cm · https://www.motorobit.com/15a-mini-low-profile-car-fuse-bicak-sigorta |
| iMAX B6AC şarj cihazı (klon) | Robotistan 1.797,09 TL | https://www.robotistan.com/imax-b6ac-professionel-li-po-with-adaptor-ni-mh-charger-balancer-50w |
| LiPo voltaj alarmı (denge soketine) | stok doğrulanmadı | spec §3.1 |
| 2200 µF 16 V kondansatör (×2) | Motorobit 4,24 TL | https://www.motorobit.com/2200uf-16v-electrolytic-capacitor-10x20mm |

### Küçük parçalar

| Parça | Mağaza | Bağlantı |
|---|---|---|
| 1/4 W metal film direnç seti (100k, 47k, 220k dahil) | Direnc 145,29 TL | https://www.direnc.net/1/4w-metal-film-direnc-paketi-600-adet |
| 100 nF 50 V MLCC (×10) | Motorobit 1,21 TL | https://www.motorobit.com/100nf-50v-5mm-multilayer-capacitor |
| 1 nF seramik (×2, ray algısı için) | stok doğrulanmadı | — |
| 30 AWG wire-wrap teli (pot uçları) | Direnc 435,88 TL | https://www.direnc.net/30awg-250-metre-yesil-kablo-sarma-teli |
| 20 AWG silikon kablo (kırmızı + siyah) | Robocombo 24 TL/m | https://www.robocombo.com/20wh-od1.8-kirmizi-silikon-kablo-1m · https://www.robocombo.com/20awg-siyah-silikon-kablo-1m |
| 7×9 delikli plaket, pin header | Motorobit 45,45 TL | https://www.motorobit.com/7x9cm-epoxy-double-sided-perforated-plaque |

### Mekanik ve aletler

Bunlar research'te yok; elinizde olanı kullanın.

- 25T alüminyum servo kolu (ya da 3D baskı/alüminyum şerit kol). Kol ucunda delik olsun.
  Eksen–delik mesafesi L ≈ 100 mm; kumpasla ölçün.
- Mutfak terazisi: ≥ 5 kg kapasite, 1 g çözünürlük.
- Ağırlık: 1.5 L dolu su şişesi (≈ 1.5 kg) ve ip.
- Açıölçer: kâğıda basılı, 5° işaretli.
- Mengene ya da sabitleme aparatı. Servo çıkış mili **yatay** olmalı ki kol dikey düzlemde dönsün.
- Multimetre: DC 10 A akım kademesi; varsa Hz kademesi.
- İnce uçlu havya, flux, kapton bant, sıcak silikon/RTV, kumpas, küçük tornavida seti.
- Dizüstü bilgisayarı **pilden** çalıştırın (toprak döngüsü olmasın).

---

## 2. Bağlantı

### 2.1 Pin haritası (tezgâh)

Robot pin haritasıyla uyumludur (power-electronics §2). Tezgâhta tek servo (DUT) vardır.

| ESP32-S3 | Bağlantı | Not |
|---|---|---|
| GPIO1 (ADC1_CH0) | Pot orta ucu (wiper), 100k/47k + 100 nF bölücüden | Zorunlu. 12 dB zayıflatma |
| GPIO2 | Pot üst ucu (pot beslemesi), aynı bölücüden | Önerilir: oranlı okuma ve besleme ölçümü |
| GPIO4 | Servo rayı (+), 100k/47k + **1 nF** (100 nF değil) | Aşım yakalama; servo konektörüne yakın |
| GPIO5 | Ortam NTC'si (3V3 – 10k – düğüm – NTC – GND) | İsteğe bağlı; yoksa betik ortam sıcaklığını sorar |
| GPIO15 | PCA ch0 sinyali, 1 kΩ seri | İsteğe bağlı: gerçek darbe genişliği ölçümü |
| GPIO16 | PCA **ch15** çıkışı, 1 kΩ seri | Osilatör kalibrasyonu (`osc auto`) |
| GPIO17 / GPIO18 | I2C SDA / SCL (400 kHz) | PCA9685, INA226, ADS1115 |
| GPIO21 | PCA9685 OE | Yüksek: bütün PWM kapalı (yazılım E-stop) |
| GPIO0 (BOOT düğmesi) | — | Program sırasında basılırsa E-stop |
| 3V3, GND | PCA9685 VCC, ADS1115 VDD, INA226 VCC, NTC bölücüsü | |

ADS1115 (ADDR → GND, adres 0x48):

| ADS1115 | Bağlantı |
|---|---|
| AIN0 | Kasa NTC'si (3V3 – 10k – düğüm – NTC – GND) |
| AIN1 | Pot orta ucu, ikinci bölücü üzerinden (ADS yolu; isteğe bağlı) |
| AIN2 | Pot **alt** ucu (servonun içindeki pot GND padı), doğrudan |
| AIN3 | Yıldız toprak noktasına giden **ayrı** ince tel |

AIN2–AIN3 diferansiyel okunur (±256 mV). Bu, servo toprağının akımla kaymasını ölçer (ortak-mod hatası).

### 2.2 Adresler

- PCA9685: 0x40 (varsayılan).
- ADS1115: 0x48 (ADDR → GND).
- INA226: **0x40 olmasın**, çünkü PCA9685 ile çakışır.
  - Modüldeki A0 (ve/veya A1) köprüsünü VS'ye lehimleyin: 0x41 ya da 0x45.
  - Betik INA226'yı üretici kimliğinden kendisi bulur.
  - `--cmd scan` ile kontrol edin.

### 2.3 Bölücü (bütün pot girişleri aynı, spec §3.1)

```
pot ucu (30 AWG) ── R1 100k ──┬── ADC pini (GPIO1 / GPIO2 / ADS AIN1)
                              ├── R2 47k ── yıldız GND
                              └── C 100 nF ── yıldız GND      (ray algısında C = 1 nF)
```

- Oran 0.32, kaynak empedansı 32 kΩ, τ = 3.2 ms.
- 6.5 V → 2.08 V; 3.3 V → 1.06 V.
- R ve C'yi ESP32'nin yanına, plakete koyun.
- Pot ~3.3 V'tan besleniyorsa (§4'te ölçeceksiniz) R2'yi 220k yapın. Oran 0.69 olur;
  `set div=0.6875` komutuyla betiğe bildirin.

### 2.4 Akım ölçümü (INA226)

```
buck (+) ── INA IN+ ──[ şönt 2–10 mΩ ]── INA IN- ── servo (+)
                                         └── INA VBS (VBUS) bu uca
```

- **Modüldeki 0.1 Ω (R100) şöntü sökün** ya da dış şöntü onun yerine bağlayın.
  0.1 Ω ile ölçüm 0.8 A'de doyar. 3.6 A'de 0.36 V düşer ve servo yavaşlar.
- Şönt değerini betiğe bildirin: `set shunt_mohm=5`.
- Laboratuvar kaynağı kullanıyorsanız INA226 yine bağlı olsun. Kaynağın göstergesi stall testinde
  "ikinci yöntem" olur.

### 2.5 Toprak ve besleme (spec §3.1, power-electronics §1)

```
LiPo/kaynak ─ XT30 (ana kesme) ─ sigorta ─ buck IN
buck OUT+ ─ INA şönt ─ servo kırmızı (20 AWG, kısa)           2200 µF servo konektörüne yakın
buck OUT- ─────────── YILDIZ NOKTA ─── servo siyah (20 AWG, kısa)
                        ├── ESP32 GND (tek ince tel, bölücülerin R2 uçlarının yanına)
                        └── ADS AIN3 (ayrı ince tel)
```

- Servo akımı **asla** ESP32, PCA9685 ya da breadboard topraklarından geçmesin.
- PCA9685'in V+ terminali boş kalır. Servonun yalnız sinyal kablosu (sarı/turuncu) PCA ch0'a gider.
- PCA9685 VCC = 3.3 V. **5 V yapmayın**: kart I2C hatlarını 5 V'a çeker ve ESP32 bunu kaldırmaz.
  Servo 3.3 V darbeyi kabul etmezse bunu §7'deki `accept33` testi söyler.
- ESP32'yi bilgisayarın USB'sinden besleyin. Kartın **UART/COM** portunu kullanın.

### 2.6 NTC'ler

- Kasa NTC'sini servo gövdesinin motor tarafındaki geniş yüzüne yerleştirin (gövdenin uzak yarısı, alt kapağa yakın).
  Termal macun ve kapton bantla yapıştırın, üstünü bir parça köpükle örtün.
- Ortam NTC'si 20–30 cm uzakta, açıkta dursun.

---

## 3. Yazılım kurulumu

1. **Firmware**:
   - PlatformIO ile:
     - `pipx install platformio` (ya da VS Code PlatformIO eklentisi).
     - `pio run -d tools/bench -t upload`.
   - Arduino IDE 2.x ile: "esp32 by Espressif" 3.x kurun ve `tools/bench/bench/bench.ino` dosyasını açın.
     Kart ayarları dosyanın başında yazar: ESP32S3 Dev Module, USB CDC On Boot: Disabled, 16MB, OPI PSRAM.
2. **PC tarafı**: `pip install pyserial numpy`. Komutları repo kökünden çalıştırın.
3. **Port adı**:
   - macOS: `/dev/cu.usbserial-*` ya da `/dev/cu.wchusbserial-*`.
   - Windows: `COM5` gibi.
   - Linux: `/dev/ttyUSB0`.
4. **İlk kontrol** (servo bağlı değilken):
   ```
   python tools/bench/logger.py --port PORT --cmd scan
   python tools/bench/logger.py --port PORT --cmd "osc auto"
   ```
   - `scan` I2C cihazlarını, ADC pin gerilimlerini, INA/ADS/NTC okumalarını basar.
   - `osc auto` PCA osilatörünü ölçer: `R,osc_hz,...` 23–27 MHz arasında çıkmalı.
   - Betik bu değeri kullanır; `cfg`'de `osc=` olarak görünür.
   - Ayarları kalıcı yapmak için: `--cmd save`.
   - GPIO16 bağlı değilse multimetrenin Hz kademesiyle ch15'i ölçün ve `--cmd "osc 305.2"` girin
     (ölçülen frekans, Hz).

Betik açılışta PWM'i kapalı tutar. İlk `us`/program komutunda servo merkeze (1500 µs) sıçrar. Kol serbest olsun.

---

## 4. DS32xx'i açmak ve pot orta ucunu almak

> **Uyarı**: DS32xx için bilinen bir söküm (teardown) yok (servo-mechanics §6).
> - İçeride erişilebilir 3 uçlu bir potansiyometre olduğunu varsayıyoruz, ama doğrulanmadı.
> - Manyetik enkoder çıkarsa ya da orta uç bulunamazsa B2 donanımı geçersizdir: **Dur, spec'e dön** (§7).
>   Fotoğraf çekip gönderin.
> - Açmak IP66 su geçirmezliğini bozar.
>
> Önce **tek bir servoyla** (DS3230) deneyin. Sonra diğerlerine geçin.

1. **Açmadan önce**:
   - Etiketin fotoğrafını çekin: HV mi, 180° mi 270° mi, model.
   - Servoyu tartın, kumpasla tab yüksekliğini ve mil ofsetini ölçün.
2. **Açma**:
   - Alt kapaktaki 4 vidayı (M2) sökün. Kapağı contayı yırtmadan kaldırın.
   - PCB'yi zorlamayın, kabloları çekmeyin.
   - Çıkış miline bağlı potansiyometreyi ve onun PCB'ye giden 3 ucunu/padını bulun. Fotoğraflayın.
3. **Lehimlemeden pot kontrolü** (`potprobe` programı):
   - Servoyu açık hâlde rayına bağlayın: 6.5 V, buck CC ≈ 2 A, sinyal PCA ch0.
   - Çalıştırın:
     ```
     python tools/bench/logger.py --port PORT --servo DS3230-1 --model DS3230 --cls ds --test potprobe
     ```
   - Betik servoyu 900, 1200, 1500, 1800 ve 2100 µs'ye götürür. Her konumda sorar.
   - Multimetrenin siyah ucunu servonun siyah kablosuna (GND) koyun. Kırmızı uçla 3 padın gerilimini ölçün
     (iğne uçla; kısa devre yapmayın). Yanıtı üç sayı olarak yazın: `0.01 1.65 3.29`.
   - Konuma göre **monoton** değişen pad orta uçtur. Sabit kalanlar üst (≈3.3 V ya da ≈6.5 V) ve alt (≈0 V) uçlardır.
   - Üst uç gerilimi pot beslemesidir (3.3 V / 5 V / ray). Bölücü seçimini bu belirler (§2.3).
   - Hiçbir pad değişmiyorsa ya da orta uç aralığı < 1 V ise durun: spec §7 "Pot yok / < 1 V".
4. **Lehim**:
   - 30 AWG wire-wrap telini orta uca lehimleyin (zorunlu).
   - Alt uca da lehimleyin (şiddetle önerilir; ortak-mod testi için ADS AIN2).
   - Üst uca da lehimleyin (önerilir; oranlı okuma için GPIO2).
   - Telleri kapton ve bir damla sıcak silikonla gerin. Kablo rakorundan ya da Ø1.5 mm bir delikten dışarı alın.
   - Servonun kendi kablolarıyla birlikte burun.
   - Kapağı kapatın, rakoru/deliği silikonla mühürleyin.
5. **ES3054, MG90S, SG90**:
   - Aynı yöntem; padlar çok küçük.
   - ES3054'te pot ADC1'e gider (hip_yaw).
   - MG90S'te pot ADS1115'e gider (kafa eklemleri; spec §3.1).
   - SG90'da pot okunmaz.

---

## 5. Güvenlik

- **LiPo**:
  - Yalnız dengeli şarj edin (iMAX B6AC, LiPo Balance, 1 A). Şarj ederken başında durun, yanmaz torba kullanın.
  - Denge soketine voltaj alarmı takın. Hücre başı 3.5 V'un altına indirmeyin (yük altında 13.2 V).
  - Saklama gerilimi hücre başı 3.8 V.
  - Şişen ya da ezilen paketi kullanmayın.
- **Asıl E-stop fiziksel kesmedir**: XT30'u elinizin altında tutun.
- **Yazılım E-stop'ları**:
  - Logger'da `!` + Enter.
  - Ctrl-C (logger `!` gönderir).
  - DevKit'in BOOT düğmesi.
  - Bu üçü PCA çıkışlarını kapatır. Dijital servo PWM gelmeyince konumunu **tutabilir**; `nopwm` testi bunu ölçer.
- **Buck ayarı**:
  - Servoyu bağlamadan önce gerilimi multimetreyle ayarlayın: DS için 6.50 V; ES3054, MG90S ve SG90 için 6.00 V.
  - CC (akım sınırı) potunu DS için ≈ 4 A, küçük servolar için ≈ 1.5 A yapın.
  - Kondansatörün kutbuna dikkat edin.
- **Betik sınırları** (`cfg`'de görünür, `set` ile değişir):
  - µs yumuşak sınırları: DS 500–2500, ES 700–2300, MG/SG 600–2400.
  - Sürekli akım: DS 3.0 A × 1.5 s. Tepe akım: 6.0 A × 0.3 s.
  - Kasa sıcaklığı: 75 °C.
  - Ray: 7.6 V.
  - Program başına zaman aşımı.
  - Kalp atışı: logger kapanırsa 5 s içinde durur.
- **Stall**:
  - Her baskı en çok 3 s sürer, aralarda soğuma beklenir.
  - Kol ve terazi sağlam sabitlensin; DS servo 100 mm'de ~3 kg kuvvet basar.
  - Parmaklarınızı kol yolundan uzak tutun.
- **Kütleli testler** (`curpts`, `overshoot atalet`, `heat`): kolun ve kütlenin döndüğü alanı boşaltın.
  Mengene masaya sağlam bağlı olsun.
- **Isınma testi**:
  - Kasa 70 °C'yi geçebilir; dokunmayın.
  - Betik kasa 75 °C'yi geçince o seviyeyi bitirir.
  - Testi gözetimsiz bırakmayın.
- **Analog servolar** (MG90S, SG90) 300 Hz'de sürülmez. Betik reddeder.

---

## 6. Test prosedürü (her servo için)

### 6.1 Kurulum

1. Servoyu çıkış mili yatay olacak şekilde mengeneye sıkın. Alüminyum kolu takın.
2. Açıölçeri kolun arkasına, 0° yatay olacak şekilde koyun.
3. Kolun ağırlığını tartın (`lever_g`). Eksen–kol ucu deliği mesafesini ölçün (`lever_mm`).
4. Ayarları girin:
   ```
   python tools/bench/logger.py --port PORT --cls ds --cmd "set lever_mm=100 lever_g=12 mass_g=1500 shunt_mohm=5"
   python tools/bench/logger.py --port PORT --cmd "set pin_ph=2 pin_rail=4 pin_amb=5 ads_mask=7"
   python tools/bench/logger.py --port PORT --cmd save
   ```
   - İsteğe bağlı kanallar varsayılan olarak **kapalıdır**, çünkü boş bir pin rastgele okur.
   - Yalnız bağladıklarınızı açın:
     - `pin_ph`: pot üst ucu;
     - `pin_rail`: ray algısı;
     - `pin_amb`: ortam NTC'si;
     - `ads_mask`: 1 = kasa NTC'si, 2 = ADS orta uç, 4 = pot alt ucu (toplayın; hepsi = 7).
   - Kapalı kanallar log'da `nan` görünür.
5. **Şönt kalibrasyonu** (bir kez): stall testinin son adımı multimetreyle karşılaştırır.
   Fark %10'dan büyükse `shunt_mohm` değerini `şönt × INA/multimetre` olarak düzeltip stall'u tekrarlayın.

### 6.2 Sıra

En kolayı plan kullanmaktır. Plan her testten önce Enter bekler: `s` atlar, `q` çıkar.

```
python tools/bench/logger.py --port PORT --servo DS3230-1 --model DS3230 --cls ds --plan ds
python tools/bench/logger.py --port PORT --servo DS3230-1 --model DS3230 --cls ds --plan heat
```

| # | Test | Kol durumu | Ne yapar | Süre |
|---|---|---|---|---|
| 1 | `scan`, `osc` | — | Bağlantı kontrolü, PCA osilatörü | 10 s |
| 2 | `sweep` | serbest | 550→2450 µs basamaklar: µs→açı, aralık, monotonluk, orta uç gerilimi | 1.5 dk |
| 3 | `potcal` | serbest, açıölçer | Kolu −60/−30/0/30/60° çizgilerine `+5`/`-5` ile getirin, `ok`: pot ofset/ölçek/doğrusallık | 5 dk |
| 4 | `deadband` | serbest | 1 sayımlık basamaklar: ölü bant (play) | 1 dk |
| 5 | `speed` | serbest | 100° adımlar: boşta hız, 60° süresi | 30 s |
| 6 | `step`, `chirp` | serbest | Basamak ve frekans taraması: P-bandı, armature, gecikme | 1.5 dk |
| 7 | `ramp` | serbest | 5…160 °/s sabit hızlar: sürtünme (hareket akımı), takip gecikmesi | 1.5 dk |
| 8 | `stall` | kol ucu terazide | Teraziyi darala; betik sırayla 0.5…20° bastırır, her baskıdan sonra en yüksek değeri (g) sorar; sonda multimetreyle 3 s ölçüm | 5 dk |
| 9 | `curpts` | 1.5 kg kol ucunda asılı | Kolu yatay yapın (`+N`/`-N`, `ok`); 6 açıda iki yönden tutma + yüklü hareket: besleme akımı modeli, ortak-mod hata | 3 dk |
| 10 | `accept300`, `accept33` | serbest | 50/300 Hz ve 3.3 V darbe takip testi. Sonda "düzgün takip etti mi? (e/h)" | 1 dk |
| 11 | `overshoot` | önce serbest, sonra kütleli | 90° ani dönüşler, ray 20 kS/s örneklenir: yavaşlamada aşım | 1 dk |
| 12 | `noise`, `nopwm` | serbest | ADC gürültüsü (Wi-Fi kapalı/açık); PWM kesilince servo tutuyor mu (`t`/`b`) | 1 dk |
| 13 | `heat` | 1.5 kg asılı | Diz benzeri döngü (1.1 Hz, ±15°), 0.9 / 1.05 / 1.15 N·m RMS, her biri 10 dk; aralarda kasa ortam+3 °C'ye soğur | 60–90 dk |

**Stall yönü**: Betik µs'yi artırarak bastırır (`dir=1`). Sizin kurulumunuzda µs artınca kol yukarı kalkıyorsa
şöyle çalıştırın:

```
--test stall --arg dir=-1
```

**Isınma kütlesi**:
- Seviyeler için m·g·L ≥ 1.25 × 1.15 N·m olmalı. L = 100 mm'de bu ≈ 1.5 kg eder.
- Kütle yetmezse betik o seviyeyi atlar (`E,skip`).
- Ortam NTC'si yoksa ortam sıcaklığını sorar. Odayı sabit tutun; ölçüm 25 °C'ye düzeltilir.

**Servo sırası**:

1. DS3230 (Direnc): tam plan + `heat`.
2. DS3225MG: tam plan + `heat`.
3. DS3235: tam plan + `heat`. Diz adayıdır; ısınma testi en önemli sonuçtur.
4. ES3054:
   - 6.0 V rayda `--cls es --plan es`.
   - `accept300` sonucu hip_yaw'ın hangi karta gideceğini belirler.
5. MG90S ve SG90: `--cls mg --plan mg`. SG90'da pot yoksa `sweep` ve `potcal`'ı atlayın.

Her servoya ayrı `--servo` adı verin (`DS3225-1`, `DS3230-1`, `DS3235-1`, `ES3054-1`, …). Ölçülen ray gerilimini
`--vrail 6.50` ile, kendi notlarınızı `--note "..."` ile ekleyin.

---

## 7. Sonuç: spec §7 karar tablosu

Logları topladıktan sonra:

```
python -m tools.sysid.fit --logs bench_logs --mass 1.95
```

Bu komut `bench_logs/sysid_params.json` ve `bench_logs/sysid_rapor.md` üretir. Kütle G-CAD'den gelir
(CAD × 1.05); bilinmiyorsa 1.95 kg sert sınırı kullanılır.

| Ölçüm | Hangi test | Eşik → eylem |
|---|---|---|
| u(τs, ω0, kütle) | stall (τs), speed (ω0) | ≤ 0.85 olan en büyük κ ∈ {0.72, 0.6, 0.5}; hiçbiri değilse spec'e dön |
| Pot | sweep, potcal, potprobe | Pot yok, monoton değil ya da < 1 V: Dur |
| Pot ortak-mod hata | curpts + stall (AIN2–AIN3 bağlıysa) | > 1°: oranlı okuma (boş ADS kanalı referans) |
| DS 300 Hz | accept300 | Red: bacak kartı 50 Hz, gecikme U{0..4}, yeniden eğit |
| ES3054 300 Hz | accept300 (`--cls es`) | Kabul: hip_yaw 0x40'a |
| 3.3 V darbe | accept33 | Red: seviye çevirici (BOM + CAD) |
| Ray aşımı | overshoot | > 6.8 V: kırpıcı ya da fren direnci, kondansatör artır |
| Isınma | heat | T_diz = 10 dk sonunda kasa ≤ 60 °C kalan en yüksek RMS (25 °C'ye düzeltilmiş) |
| Diz servosu | heat | Yalnız T_diz(aday) ≥ tahmini diz RMS olan aday. Tahmin G-ISIL'den gelir (`--knee-rms`); verilmezse 0.30·k(m) |
| Stall akımı | stall | INA ile multimetre farkı ≤ %10 |

`sysid_params.json` içindeki `actuator` ve `mjcf` blokları `rl/actuators.py`'e girer:
kp = τs/P-bandı, forcerange = ±τs, damping = τs/ω0, armature, frictionloss, ölü bant, gecikme.
Besleme akımı modeli `supply_current` bloğundadır.

---

## 8. Logları geri gönderme

1. Logları zipleyin:
   ```
   python tools/bench/logger.py --pack bench_logs
   ```
   Bu komut `bench_logs.zip` dosyasını oluşturur. İçinde her koşu için CSV, olay, meta JSON ve ham satırlar bulunur.
2. Zip'in yanına şunları ekleyin:
   - etiket ve PCB fotoğrafları;
   - pot pad fotoğrafı;
   - servo ağırlıkları;
   - kumpas ölçüleri;
   - kurulum fotoğrafı (kol, terazi, NTC yeri).
3. Varsa `sysid_rapor.md` dosyasını da ekleyin.
4. Gönderme yolu: repoda `adim0-logs` dalına `bench_logs/` olarak commit edin ya da zip'i bu sohbete/paylaşılan
   klasöre koyun.

Bir test yarıda kesildiyse (`END,...,abort:<neden>`), o kaydı silmeyin; aynı testi yeniden koşun.
Uydurucu her programın son başarılı koşusunu kullanır.

---

## 9. Sorun giderme

| Belirti | Çözüm |
|---|---|
| `W,PCA9685 0x40'ta yok` | SDA=17, SCL=18, PCA VCC=3V3 ve GND bağlarını kontrol edin. `scan` çıktısında 0x40 görünmeli |
| INA226 yok / akım `nan` | A0/A1 köprüsü (0x40 olmasın). VBS bağlı mı? R100 söküldü mü? |
| Akım çok küçük ya da hep 0 | Modül şöntü dış şöntü kısa devre ediyor olabilir; `shunt_mohm` doğru mu? |
| Pot okuması gürültülü | Bölücü ESP32'nin yanında mı? 100 nF var mı? ESP32 GND yıldız noktaya tek telle mi bağlı? Wi-Fi kapalı mı? |
| `abort:host_yok` | Logger kapandı ya da port koptu. Kabloyu kontrol edip testi tekrarlayın |
| `abort:asiri_akim` / `tepe_akim` | Kol bir yere takılıyor, ya da buck CC çok yüksek ve servo zorlanıyor. Kurulumu kontrol edin |
| Servo 300 Hz'de titriyor ya da çok ısınıyor | `accept300` sorusuna `h` yazın: red kaydedilir (karar tablosu) |
| Port açılınca kart resetleniyor | Logger DTR/RTS'yi kapalı açar. Başka bir seri monitör açık mı? |
| Ray okuması yanlış | GPIO4 bölücüsünde C = 1 nF. Ray ölçeği INA VBUS ile otomatik eşlenir |
