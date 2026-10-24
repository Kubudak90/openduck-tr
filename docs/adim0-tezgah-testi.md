# Adım 0: servo tezgâh testi (kullanım kılavuzu)

Bu test, robotun servo seçimini ve simülasyon parametrelerini gerçek ölçümle sabitler (spec §7). Çıktısı
`tools/sysid` ile `rl/actuators.py` parametrelerine ve §7 karar tablosuna dönüşür: κ (yürüyüş hızı), diz servosu,
300 Hz, 3.3 V darbe, pot okuma yöntemi, ray aşımı ve T_diz.

Bir DS servo için tahmini süre: kurulum 1–2 saat (pot kablolama dahil), ölçüm ~45 dk, ısınma testi ~1.5 saat.

- Betik: `tools/bench/` (ESP32-S3 + PCA9685 + INA226 + ADS1115 + NTC).
- PC kaydedici: `tools/bench/logger.py`.
- Uydurucu: `python -m tools.sysid.fit`.
- Satır biçimi: `tools/sysid/logio.py`.
- Ses modülü eki (G-SES-a, spec eki K8): §10; analiz `tools/bench/voice_report.py`.

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
| GPIO21 | PCA9685 OE (iki kartta da) | Yüksek: bütün PWM kapalı (yazılım E-stop) |
| GPIO0 (BOOT düğmesi) | — | Program sırasında basılırsa E-stop. 2 s basılı tutmak ses modülünün öğrenme modunu açar/kapatır |
| GPIO12 (UART1 RX) | ← **1 kΩ** ← DF2301Q D/T (modülün TX'i) | Ses modülü (§10). Robotla aynı pin |
| GPIO11 (UART1 TX) | → 1 kΩ → DF2301Q C/R (modülün RX'i) | Ses modülü. Robotla aynı pin |
| GPIO13 | → 330 Ω → LED anodu, katot GND | Kafa LED'i (uyanma/red göstergesi). Robotla aynı pin |
| GPIO7 | → 100 Ω → piezo | Yalnız tezgâh: G-SES-a senkron tıkı |
| GPIO6 (ADC1_CH5) | Fototransistör düğümü (modülün mavi LED'ine bakar) | Yalnız tezgâh. `set pin_ldr=6` |
| 3V3, GND | PCA9685 VCC, ADS1115 VDD, INA226 VCC, NTC bölücüsü | |
| 5V | DF2301Q VCC (ayrı dal; modülde 47 µF + 100 nF) | |

Ses modülü pinleri robot pin haritasında da aynıdır (ek K3 Bağlantı; power-electronics §2). Ses modülü için kayış
pinleri (0, 3, 45, 46) ve UART0 (43/44) kullanılmadı. GPIO11–13, haritada isteğe bağlı ADC2 toprak algısına ayrılmış 11–14 aralığından alındı;
toprak algısı için GPIO14 kalır. Piezo ve fototransistör, tezgâhta boş olan sağ bacak pot pinlerinde (6, 7) durur;
robota gitmezler.

İkinci PCA9685 (isteğe bağlı, önerilir; robotun PCA#2'si gibi): A0 köprülü (0x41), 50 Hz. SDA/SCL ortak, OE → GPIO21,
VCC 3V3. MG90S → ch4, SG90 → ch6. Bu durumda INA226 0x45'te olmalı (A0 = A1 = VS).

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
- İkinci PCA9685 (§10, isteğe bağlı): 0x41 (A0 köprülü).
- INA226: **0x40 olmasın**, çünkü PCA9685 ile çakışır.
  - Modüldeki A0 (ve/veya A1) köprüsünü VS'ye lehimleyin: 0x41 ya da 0x45. İkinci PCA kullanacaksanız 0x45.
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
| 4 | `deadband` | serbest | 1 sayımlık basamaklar: ölü bant + sürtünme histerezisi | 1 dk |
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
Ölü bant (`deadband_rad`) tam genişliktir ve `rl/actuators.py` içindeki `dead_band` ile aynı kuraldır:
e = ctrl − q, ctrl' = q + sign(e)·max(|e| − ölü bant/2, 0). Kural her alt adımda o anki q ile uygulanır;
bant içinde P kuvveti 0'dır. Gecikme ölü banttan önce komuta uygulanır.
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

---

## 10. G-SES-a: ses modülü testi

Bu bölüm spec ekinin K8 protokolüdür (`docs/workflow/openduck-tr/spec-ek-komut.md`). DF2301Q ses modülünün
robotta kalıp kalmayacağına bu kapı karar verir. Çıktıları şunlardır:
- modülün ölçüleri ve kütlesi (P2d kafa yerleşimi);
- kimlik tablosu `configs/komut_kimlik.json` (P5k ve firmware okur);
- tanıma gecikmesi dağılımı `configs/ses_gecikme.json` (A10 değerlendirme DR'ı);
- kapı sonucu.

Süre: kurulum ve bağlantı yaklaşık 20 dk, protokol yaklaşık 1.5–2 saat.

| Adım | Test | Süre |
|---|---|---|
| Ölçümler (§10.4) | `ses-olcu` | 15 dk |
| Öğrenme ve kimlik tablosu (§10.5) | `ses-ogren` | 15 dk |
| Deneme 1: gürültü, 1.3 m, 12 × 20 | `ses-d1` | 25 dk |
| Deneme 2: sessiz, 1.8 m, 12 × 10 + 10 aralıksız | `ses-d2` | 15 dk |
| Deneme 3: uyanık pencerede yanlış komut | `ses-d3` | 40–45 dk |
| Deneme 4: modül davranışı | `ses-d4` | 10 dk |

Hepsini sırayla koşturmak için (her testten önce Enter bekler; `s` atlar, `q` çıkar):

```
python tools/bench/logger.py --port PORT --servo DF2301Q-1 --plan ses
```

Planın ilk adımı `gest`tir: gürültü programını 60 s koşturup servoların doğru kanallarda olduğunu gösterir.

### 10.1 Alışveriş listesi

Modül Adım 0 alımıyla birlikte alınır ve robota gider. Fiyat ve stok 24.10.2026 tarihli research taramasından
(`research/voice-research.md`). Diğer satırların stoğu doğrulanmadı.

| Parça | Adet | Not |
|---|---|---|
| DFRobot DF2301Q (Gravity SEN0539) | 1 | SAMM 1.171,86 TL, stok 6 (https://market.samm.com/gravity-cevrimdisi-ses-tanima-sensoru-i2c-ve-uart). Yedek: Direnc 1.817,05 TL. Tercihen V1.1 (dört delik); gelince kart yazısına bakın |
| Seramik piezo disk (pasif, 20–27 mm, kendi osilatörü olmayan) | 1 | Yalnız tezgâh. 100 Ω seri direnç |
| Görünür ışık fototransistörü (ör. TEMT6000 modülü) ya da LDR (GL5528) | 1 | Yalnız tezgâh. 10 kΩ ve bir parça siyah makaron |
| 3 mm LED (yüksek parlaklık) + 330 Ω | 1 + 1 | Kafa LED'i; robota da gider |
| 1 kΩ direnç | 2 | RX ve TX hatlarına seri |
| 2 kΩ direnç | 1 | Yalnız TX boşta gerilimi > 3.6 V çıkarsa (bölücü) |
| 47 µF 16 V elektrolitik + 100 nF MLCC | 1 + 1 | Modülün besleme ucunda |
| İkinci PCA9685 | 1 | Robot adedinden. Önerilir; yoksa tek kart 50 Hz'e alınır |
| Mezura, 0.35 m'lik kutu/sehpa, kumpas, 0.1 g terazi (yoksa 1 g), multimetre (mA) | — | |
| Ses kaydedici (telefon) ve kayıtları çalacak bir hoparlör (bilgisayar ya da ikinci telefon) | — | |

Modülün kutusundaki Gravity 4P kablosu (PH2.0) tezgâhta kullanılabilir. Adım 0'dan MG90S, SG90 ve bir DS servo gerekir.

### 10.2 Bağlantı

Pinler §2.1'deki tabloda ve robot pin haritasında aynıdır.

```
DF2301Q (Gravity 4P)                         ESP32-S3-DevKitC-1
  D/T  (modül TX) ── 1 kΩ ──────────────────── GPIO12  (UART1 RX)
  C/R  (modül RX) ── 1 kΩ ──────────────────── GPIO11  (UART1 TX)
  GND ──────────────────────────────────────── GND
  VCC ──────────────────┬───────────────────── 5V   (ayrı dal: servo raylarına bağlanmaz)
              47 µF + 100 nF (modülün yanında, GND'ye)

GPIO13 ── 330 Ω ── LED (+) ; LED (−) ── GND          kafa LED'i: konuşmacı görebilmeli
GPIO7  ── 100 Ω ── piezo (+) ; piezo (−) ── GND       modülün ve ses kaydedicinin yanında
3V3 ── fototransistör C ; E ── düğüm ── 10 kΩ ── GND ; düğüm ── GPIO6
   (LDR ile: 3V3 ── LDR ── düğüm ── 10 kΩ ── GND. TEMT6000 modülü: VCC 3V3, SIG → GPIO6)
```

- Modülün anahtarları: iletişim **UART** (I2C değil), hoparlör **SPK1** (kart üstü). I2C kullanılmaz: modülün I2C
  hattını kilitlediğine dair kullanıcı raporu var (ek K3).
- **TX'i bağlamadan önce gerilimini ölçün** (§10.4). Ölçülen boşta gerilim ≤ 3.6 V ise yalnız 1 kΩ seri direnç
  yeterlidir. > 3.6 V ise bölücü kurun: D/T ── 1 kΩ ── GPIO12, ayrıca GPIO12 ── 2 kΩ ── GND (5 V → 3.33 V).
- D/T ve C/R'nin TX/RX eşlemesi doğrulanmadı. Hiç kare gelmezse iki teli yer değiştirin. İki uçta da 1 kΩ olduğu
  için bu zararsızdır.
- Fototransistörü modülün mavi durum LED'ine dayayın ve üstüne siyah makaron geçirin ki oda ışığı girmesin.
  Kontrol: `--cmd scan` çıktısındaki `R,ldr_mv` değeri, modül uyanıkken (ördek dedikten sonra) uykudakinden en az
  100 mV yüksek olmalı.
- Servolar: MG90S → PCA#2 ch4, SG90 → PCA#2 ch6, DS → PCA#1 ch0.
  - MG90S ve SG90 6.0 V rayda, DS 6.5 V rayda (§5).
  - İkinci PCA yoksa MG90S → PCA#1 ch4, SG90 → PCA#1 ch6. Betik bu durumda PCA#1'i oturum boyunca 50 Hz'e alır; DS
    de 50 Hz'de salınır (DS3230 50–330 Hz kabul eder). Analog servolar hiçbir zaman 300 Hz'de sürülmez.
- Kanalları değiştirmek için: `--arg mg=2:4 sg=2:6 ds=1:0` (kart:kanal). İkinci bir MG90S'i ayrı head_yaw olarak
  eklemek için: `mg2=2:3`.

**Düzenek (deneme 1 ve 3):**
- Çıplak modül 0.35 m yükseklikte, mikrofonlar konuşmacıya bakar.
- MG90S ve SG90 modülden yaklaşık 4 cm uzakta, kolları serbest. DS servo yanında, mengenede.
- Ses kaydedici (telefon) ve piezo modülün hemen yanında (≤ 10 cm).
- Konuşmacı ayaktadır. Ağız–modül mesafesi deneme 1'de 1.3 m, deneme 2'de 1.8 m. Mesafeyi mezurayla işaretleyin.

### 10.3 Yazılım

1. Firmware'i yükleyin (`bench-2`): `pio run -d tools/bench -t upload`.
   - Açılışta betik modüle iki kez (1.5 s ve 4 s'de) `setMuteMode(1)` ve `setWakeTime(15)` gönderir (ek K3.7).
   - Modül `acilis` NOTIFY'ı gönderirse aynısı tekrarlanır.
2. Kontrol:
   ```
   python tools/bench/logger.py --port PORT --cmd scan
   python tools/bench/logger.py --port PORT --cmd "set pin_ldr=6"
   python tools/bench/logger.py --port PORT --cmd save
   ```
   `scan` çıktısında `R,pca2,1` (ikinci PCA bulundu) ve `R,ldr_mv,...` görünür.
3. Gürültü programı: `python tools/bench/logger.py --port PORT --servo DF2301Q-1 --test gest`. 60 s boyunca:
   - MG90S sırayla baş sallar (head_pitch şablonu: ±0.2 rad, 1.5 Hz, 2 çevrim) ve baş çevirir (head_yaw: ±0.4 rad,
     1.2 Hz, 2 çevrim);
   - SG90 gaga açıp kapar (0 ↔ 25°, 2 Hz, 2 çevrim) ve çift tık yapar (0 → 12° → 0, 0.3 s);
   - DS ±15° ve 1.1 Hz'de salınır.

   Şablonlar ek K6.1'dekilerdir: Tukey zarfı, konum ya da hız basamağı yok.
4. Modülü izlemek için: `--test ses-izle` (Ctrl-C ile biter). Tanınan her kimlik ekrana yazılır.
   Elle komut: `--cmd "v play 23"`, `--cmd "v mute 0"`, `--cmd "v wake 20"`.

Bütün oturumlar `voice` programını koşturur. Firmware şunları kendi saatiyle (µs) kaydeder:
- modülden gelen her kare: tanıma kimliği, NOTIFY, ACK, bilinmeyen ya da bozuk kareler ve ham baytlar;
- piezo tıkları ve deneme pencereleri;
- fototransistör, kafa LED'i ve INA226 (50 Hz).

Logger protokolü yürütür, ekranda ne söyleyeceğinizi yazar ve kimlik gelince bir sonrakine geçer. Deneme sırasında
Ctrl-C duraklatır: Enter o denemeyi tekrarlar (kayıt geçersiz sayılır), `s` olduğu gibi sayıp geçer, `q` bitirir.

### 10.4 Ölçümler: `ses-olcu`

Önce, TX'i ESP32'ye bağlamadan:
1. Modülü yalnız VCC (5 V) ve GND ile besleyin; anahtar UART'ta olsun.
2. Multimetreyle D/T ile GND arasını ölçün (TX boşta gerilimi). Değeri not edin; bölücü kararı buna bağlıdır (§10.2).
3. Sonra RX/TX tellerini bağlayın.

Kumpas ve terazi ölçümlerini yapın, fotoğraflayın:
- kart yazısı ve revizyon (V1.0 / V1.1);
- montaj delikleri: sayı, eksenler arası mesafe ve çap;
- iki mikrofonun merkezleri (konnektör kenarından ve uzun kenardan);
- mikrofon portunun yönü;
- en yüksek parçayla toplam yükseklik;
- konnektörün yeri ve fiş takılıyken yükseklik;
- kütle (kablosuz).

Akım için multimetreyi (mA kademesi) modülün 5 V dalına seri bağlayın. İsterseniz INA226'yı da bu dala alın; o zaman
oturum her ölçümün ortalamasını ve tepesini de kaydeder.

```
python tools/bench/logger.py --port PORT --servo DF2301Q-1 --test ses-olcu
```

Oturum her değeri sorar, siz yazarsınız (bilinmiyorsa `-`). Sırası:
1. ölçüler, kütle ve TX boşta gerilimi;
2. akım: boşta (uyku, sessiz oda), dinlerken (uyku, siz konuşurken; "ördek" demeden), uyanık ("ördek"ten sonra, LED
   yanarken);
3. açılış ani akımı: multimetre MAX/tepe kipinde, modülün beslemesini çıkarıp takın;
4. RX 3.3 V testi: betik modüle `setMuteMode(0)`, ses 7 ve `playByCMDID(23)` gönderir.
   - Yanıt sesini duyduysanız modülün RX'i ESP32'nin 3.3 V sürüşüyle çalışıyor.
   - Duymadıysanız D/T–C/R'yi değiştirip tekrarlayın. Yine olmazsa seviye çevirici gerekir; rapora yazın.
   - Çalarken akım da sorulur. Sonda modül yeniden susturulur.

### 10.5 Kelimeleri öğretme ve kimlik tablosu: `ses-ogren`

Modül çıplak, servolar kapalı, oda sessiz; modüle yaklaşık 1 m uzakta durun.

```
python tools/bench/logger.py --port PORT --servo DF2301Q-1 --test ses-ogren
```

Betik öğrenme modunu açar: hoparlör açılır ve kafa LED'i hızlı yanıp söner. Aynı şey BOOT düğmesine 2 s basarak da
yapılır (robotta da öyle). Sonra adım adım yönlendirir. Modülün istemleri İngilizcedir; bazı birimler başka dilde
konuşabilir.
1. **Hepsini sil:** "Hello robot" → "I want to delete" → "Delete all". Ekranda 2, 204 ve 208 kimlikleri görünmeli.
2. **Uyanma kelimesi:** "Hello robot" → "Learning wake word". Her istemden sonra "ördek" deyin (3 kez).
3. **Komutlar:** "ördek" → "Learning command word". Sonra şu sırayla, her birini istem sordukça 3 kez söyleyin:
   gel, dur, geri, sağa dön, sola dön, otur, kalk, eğil, yüksel, evet, hayır, vak. Kimlikler öğrenme sırasıyla 5–16
   olur.
4. "Exit learning" deyin. Betik öğrenme modunu kapatır ve modülü yeniden susturur.
5. **Doğrulama:** her kelime için ekranda yazanı söyleyin: "ördek" → LED → kelime. Gelen kimlik kaydedilir.

Sonra kimlik tablosunu yazın:

```
python tools/bench/voice_report.py tablo bench_logs/DF2301Q-1
```

- Komut `configs/komut_kimlik.json` dosyasını yazar: `{"version": 1, "ids": {"5": "gel", ...}, "wake": [1, 2]}`.
- Bir kelime beklenen kimliği almadıysa ya da doğrulamada kimlik gelmediyse uyarı verir. O kelimeyi yeniden öğretin.
- Zayıf bir kelimeye boş kimliklerde (17–21) ikinci şablon vermek için (ek K3 yedek yolu):
  ```
  python tools/bench/logger.py --port PORT --servo DF2301Q-1 --test ses-ogren --arg kelimeler=geri,sola
  python tools/bench/voice_report.py tablo bench_logs/DF2301Q-1 --ekle
  ```
  Bu sırada silme ve uyanma adımları atlanır.

### 10.6 Denemeler

Deneme 1 ve 2 boyunca telefonla ses kaydı alın; telefon modülün yanında dursun. Kaydı WAV'a çevirin (Audacity:
Dışa aktar > WAV). Dosyaları `ses-d1.wav` ve `ses-d2.wav` adlarıyla `bench_logs/DF2301Q-1/` klasörüne koyun. Gecikme
bu kayıttan ölçülür: her denemenin başında piezo bir tık çalar. Analiz tıkları kayıtta bulup log saatine hizalar,
sonra kelimenin bitişinden kimlik karesine kadar geçen süreyi hesaplar.

**Deneme 1 (gürültülü, 1.3 m):**
```
python tools/bench/logger.py --port PORT --servo DF2301Q-1 --test ses-d1
```
- Düzenek §10.2'deki gibidir. Enter'dan sonra servolar oynamaya başlar.
- Her kelime 20 kez, karışık sırayla gelir (240 deneme).
- Her denemede: "ördek" deyin, kafa LED'inin yanmasını bekleyin, sonra ekrandaki kelimeyi söyleyin.

**Deneme 2 (sessiz, 1.8 m):**
```
python tools/bench/logger.py --port PORT --servo DF2301Q-1 --test ses-d2
```
- Servolar kapalı. Her kelime 10 kez söylenir (120 deneme).
- Sonra 10 kez aralıksız "ördek gel": tek nefeste, LED'i beklemeden.

**Deneme 3 (uyanık pencerede yanlış komut; servolar çalışırken, konuşmadan):**

Önce iki kısa kayıt hazırlayın: kendi sesinizle "gel" (`gel.wav`) ve "ördek" (`ordek.wav`). Bilgisayar bunları
kendi hoparlöründen çalar. Dosya vermezseniz betik her seferinde sizden çalmanızı ister.

```
python tools/bench/logger.py --port PORT --servo DF2301Q-1 --test ses-d3 --arg canli_wav=gel.wav ordek_wav=ordek.wav
```

Akış:
1. Servolar çalışır, 20 s uyku tabanı kaydedilir. Sonra "ördek" deyin ve susun.
2. Pencere `setWakeTime(255)` ve 200 s'de bir yenilemeyle açık tutulur (A yöntemi).
3. Her 5 dakikada bir canlılık kontrolü yapılır: kayıtlı "gel" çalınır, tanınması beklenir.
4. 10. dakikada yöntem doğrulanır.
   - Doğrulanmış uyanık süre geçen sürenin %90'ından azsa B yöntemine geçilir: pencere 15 s olur ve kayıtlı "ördek"
     yaklaşık 14 s'de bir çalınır. Her çalmadan sonraki 2 s maruziyetten düşülür.
5. Doğrulanmış uyanık süre 30 dakikaya ulaşınca (en çok 60 dk) koşu biter.
6. Ardından pencere kapanır ve 5 dk'lık uyku bölümü gelir. Buradaki yanlış uyanmalar ayrıca sayılır.

Uyanık durum, modül NOTIFY kareleri gönderiyorsa onlardan okunur. Göndermiyorsa modülün LED'ine bakan
fototransistörden okunur.

"Doğrulanmış uyanık süre", göstergenin uyanık dediği ve iki geçen canlılık kontrolü arasında kalan süredir. Bir
kontrol kalırsa ondan önceki aralık sayılmaz. Bu süre 30 dakikanın altında kalırsa sonuç "geçti" değil
"ölçülemedi"dir.

**Deneme 4 (modül davranışı):**
```
python tools/bench/logger.py --port PORT --servo DF2301Q-1 --test ses-d4
```
- **A, pencere süresi:** "ördek"; 17 s sonra ekranda `SIMDI` çıkınca "evet".
- **B, geçerli komut pencereyi yeniliyor mu:** "ördek"; 8. saniyede "evet", 17. saniyede "hayır".
- **C, kalıcılık:** betik modüle mute = 1 ve pencere = 30 s gönderir.
  - Modülün 5 V'unu 5 s çıkarıp geri takın; ESP32 açık kalsın.
  - "ördek"e sesli yanıt geldi mi? Bu, susturmanın kalıp kalmadığını gösterir.
  - 22. saniyede "evet" tanınırsa 30 s ayarı kalmıştır.
- **D, istemler kafa kesiti içinden duyuluyor mu:** modülü kafa kesitine koyun (P2d çıktısı yoksa kapalı bir kutu ya
  da 3D baskı kapak). 1 m'den "ördek" → "Learning command word" deyin, istemi dinleyin, hemen "Exit learning" deyin.

NOTIFY karelerinin gelip gelmediği bütün oturumlardan otomatik sayılır.

### 10.7 Kapı ve rapor

```
python tools/bench/voice_report.py rapor bench_logs/DF2301Q-1
```

Komut `bench_logs/DF2301Q-1/ses_rapor.md` ve `ses_rapor.json` dosyalarını yazar. Ses kaydı varsa
`configs/ses_gecikme.json` dosyasını da yazar.

| Ölçüt | Hesap | Eşik |
|---|---|---|
| Kelime başına doğruluk | Deneme 1, kelime başına 20 deneme | ≥ 18/20; dur için ≥ 19/20 |
| Havuz | Deneme 1 + 2'nin LED biçimli denemeleri | ≥ %90 ve Wilson alt sınırı ≥ %85 |
| Tehlikeli çiftler | gel/geri, sağa/sola, otur/kalk; iki yönde, bütün denemelerde | İkame çift başına ≤ 1 |
| Uyanık pencerede yanlış komut | Deneme 3, doğrulanmış uyanık süre | 30 dk'da 0 yanlış komut; Poisson %95 üst sınırı ≤ 1/10 dk |
| "ördek → komut" zinciri | LED biçimli denemelerde uyanma kimliği ve ardından doğru komut | ≥ %90 |

- Bir denemenin sonucu, penceresindeki ilk uyanma dışı kimliktir. Bu kimlik tablodan kelimeye çevrilir. Sonuç dört
  türden biridir: doğru, ikame (başka bir kelime), bilinmeyen (tabloda yok) ya da yok.
- Deneme 2'nin kelime başına sonuçları ve aralıksız biçim raporda ayrıca yazılır.
- Karışıklık matrisi rapordadır.
- Ölçülen gecikme dağılımı yalnız değerlendirme DR'ına gider (`configs/ses_gecikme.json`, A10). Eğitim yeniden
  yapılmaz.
- Kapı tutmazsa ek K3'teki yedek yol uygulanır:
  1. şablonlar yeniden öğretilir, zayıf kelimelere 17–21'de ikinci şablon verilir;
  2. conta ve köpük düzeltilir;
  3. hâlâ tutmazsa kullanıcı karar verir (Elechouse V3 tipi modül ya da telefon uygulaması).

### 10.8 Geri gönderilecek dosyalar

1. Logları zipleyin: `python tools/bench/logger.py --pack bench_logs/DF2301Q-1`. Zip içinde bütün `ses-*` koşuları
   (CSV, olay, meta JSON, ham satırlar), `ses-d1.wav`, `ses-d2.wav`, `ses_rapor.md` ve `ses_rapor.json` bulunur.
2. Ayrıca şunları gönderin:
   - `configs/komut_kimlik.json` ve (varsa) `configs/ses_gecikme.json`;
   - `gel.wav` ve `ordek.wav`;
   - kartın iki yüzünün fotoğrafı (revizyon yazısı, delikler, mikrofonlar);
   - kumpas ölçülerinin fotoğrafı;
   - düzenek fotoğrafı: modül yüksekliği, servo mesafeleri, fototransistör ve LED yeri;
   - notlar: istemlerin dili ve gözünüze çarpan her şey.
3. Gönderme yolu §8'deki gibidir.

### 10.9 Sorun giderme

| Belirti | Çözüm |
|---|---|
| `voice programi baslamadi` | Firmware `bench-2` yüklü mü? `--cmd cfg` çıktısında `fw=bench-2` görünmeli |
| Hiç kimlik gelmiyor (`<-` satırı yok) | Anahtar UART'ta mı? D/T–C/R'yi değiştirin. GND ortak mı? `--test ses-izle` ile "Hello robot" deyin |
| `vbad` / `vjunk` olayları | Hat gürültülü ya da baud yanlış. Teller kısa olsun ve servo kablolarından ayrı dursun. Raporda sayılır; ham baytlar logdadır |
| `fototransistor ... kontrast yetersiz` | Makaronu LED'e dayayın, oda ışığını kısın. TEMT6000 ya da LDR deneyin. `scan` ile uyanık ve uyku değerlerini karşılaştırın |
| `W,ses: ogrenme kimligi 20x (ogrenme modu kapali)` | Modül bir öğrenme komutunu duydu. Raporda sayılır; öğrenme modunu açmadıysanız not edin |
| `W,BOOT: servolar kapatildi` | BOOT'a kısa basıldı (E-stop). Ctrl-C ile duraklatıp `q` deyin ve oturumu yeniden başlatın |
| `W,gurultu: tepe akim` | INA226 servo rayındaysa akım sınırı aşıldı. Kollar bir yere takılıyor mu? |
| `UYARI: uyanik gosterge yok` (deneme 3) | NOTIFY yok ve fototransistör geçersiz. Süre doğrulanamaz; fototransistörü düzeltip deneme 3'ü yeniden koşturun |
| `tikler hizalanamadi` (rapor) | Kayıtta piezo tıkı duyulmuyor. Piezoyu telefona yaklaştırın, `--arg tick_ms=80` ile tıkı uzatın |
