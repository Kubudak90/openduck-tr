# Spec: OpenDuck-TR (v2, CAD + simülasyon + RL)

Girdi: `research/` (parts-research, torque-needs, servo-mechanics, cad-map, power-electronics, rl-map,
discovery-critic R1–R19/G1–G9/D1–D15). İnceleme kararları: `review.md` (tur 1: 34 itiraz). Sayıların kaynakları
research'tedir; bu belge kararları verir.

## 1. Problem ve hedef

Türkiye'de bugün stokta olan, ucuz parçalarla basılabilen ~35 cm'lik BDX/Microduck tarzı bir ördek robotun
**parametrik CAD'i, baskı seti, BOM'u, servo ve sensör modelli MuJoCo modeli ve simülasyonda eğitilmiş politikaları**
(yürüme, boy ayarı + oturma/kalkma, düşünce kalkma). Donanım "B2-ESP32": PWM servolar, potansiyometre geri bildirimi,
politika ESP32-S3'te (firmware sonraki tur; politika ve sensör modeli ona göre). Bütçe tavanı ~25k TL (kullanıcı).

## 2. Kapsam dışı

- Robot firmware'i (görevler, süpervizör, kumanda, kalibrasyon depolama). **İstisna**: Adım 0 için tezgâh betiği.
- Satın alma, tezgâh testinin yürütülmesi (kullanıcı) ve gerçek robot denemeleri.
- 27 cm XL330 profilinin kalıcı bakımı. Yalnız refactor regresyon referansı olarak `configs/xl330_27.json` kalır
  (§3.2.1); RL tarafında walk için "bit-düzeyinde aynı" sözleşmesi kalkar.
- Kafa elektroniği (kamera, hoparlör, ToF); göz kozmetik baskı.

## 3. Mimari

### 3.1 Donanım

| Eklem (sol/sağ) | Servo | Adet | Not |
|---|---|---|---|
| hip_roll, hip_pitch, knee, ankle | DS32xx | 8 | 180° (500–2500 µs), 6.5 V, alüminyum 25T kol |
| neck_pitch | DS32xx | 1 | Gövde trunk'ta sabit, horn boynu sürer |
| hip_yaw | EMAX ES3054 | 2 | 6.0 V, yalnız tork iletir (döner tabla, aşağıda) |
| head_pitch, head_yaw, head_roll | MG90S | 3 | 6.0 V; head_yaw ±80° |
| mouth | SG90 | 1 | 6.0 V |

- **DS32xx seçimi Adım 0'da**: Robocombo DS3225MG (1.031 TL, 99 stok) veya Direnc DS3230 (1.046 TL, 15 stok).
  DS3218/3225/3230 aynı gövdede, CAD değişmez. Motorobit DSSERVO DS3230 (1.939 TL, 48 stok) yalnız yedek kaynak.
- **Geri bildirim (15 eklemin 14'ü ölçülür)**:
  - 9 DS + 2 ES3054 potansiyometresi → 10'u ESP32-S3 ADC1 (GPIO1–10; 100k/47k + 100 nF; 12 dB).
  - neck_pitch potu ve 3 kafa servosunun potları → ADS1115 #1 (0x48).
  - 2 diz NTC'si → ADS1115 #2 (0x49); 2 kanal boş.
  - ADS1115'ler 860 SPS mux'lı çalışır (kanal başı ~215 Hz). Gaga ölçülmez.
- **Elektronik**: ESP32-S3-DevKitC-1 N16R8; PCA9685 0x40 (9 DS, prescale 19/20 ≈ 300 Hz) ve 0x41 (ES3054 + MG90S +
  SG90, 50 Hz); MPU6500 (SPI GPIO39–42); INA226 yalnız VBUS ölçümü (seri şönt yok).
- **Güç** (hazır LiPo, kullanıcı kararı):
  - Profuse 4S 1500 mAh 95C, denge soketinde LiPo voltaj alarmı.
  - Zincir: XT30 ana kesme → 15 A sigorta → 16.8 V DC'ye uygun anahtar → raylar.
  - Raylar: (a) 20 A buck 6.50 V, CC ≈ 16 A → 9 DS (TVS + 2200 µF); (b) XL4015 6.0 V → ES3054 + MG90S + SG90;
    (c) MP1584 5 V → lojik (1N5819).
  - CAD'de güç kesme MOSFET/latch için yer ayrılır. Dengeli şarj cihazı kullanılır.
- **Mekanik**:
  - Ölçek s = 1.282 (stand keyframe'inde kafa üstü 350 ± 5 mm). Micro eklem noktaları düzgün ölçeklenir.
  - Geniş duruş kabul (kullanıcı). Yazıcı 256 × 256 × 256 mm; yapısal PETG, kabuklar PLA (sabit et kalınlığı),
    taban TPU 95A. M3 somun yuvası; ısıl ek somun yok.
  - DS eklemleri: baskı beşik + U-link; eksenle hizalı arka idler (F623ZZ, M3 mil); alüminyum 25T kol linke gömülü.
  - hip_pitch horn'u bir diş kaydırılır (+14.4°).
  - hip_yaw: kalça braketi gövdeye ≥ 20 mm aralıklı iki rulmanlı döner tablayla oturur; ES3054'e pim/yarık
    kaplinle yalnız tork gider.
  - Kafa: çene/alın tamponu kafa darbesini boyun U-link'ine aktarır, kafa eklemleri bu yük yolunun dışında kalır.
  - Satın alınan her elektronik parça CAD'de gerçek zarfıyla katı olarak bulunur. Trunk iç hacim bütçesi ve buck
    havalandırması tasarlanır; ısı kaynaklarına PETG kullanılır, PLA'ya ≥ 5 mm hava bırakılır.
  - Eklem geçişlerinde kablo demeti (çap, bükülme yarıçapı) CAD parametresidir ve çarpışma taramasına girer.
- **Görevlerin gerektirdiği eklem aralıkları** (sol bacak işareti; sınırlar bunları kapsamalı, 5° servo payı):
  hip_pitch −52…+90 (sınır −70…+90), knee −37…+62, ankle −45…+57, hip_yaw −21…+4, neck 1…40 (getup için ileri
  −90'a kadar istenir, servo aralığı izin verdiği kadar), head_yaw ±34 (sınır ±80), head_pitch 1…38.

### 3.2 Yazılım bileşenleri

1. **CAD refactor** (`cad/servos.py` yeni, `cad/openduck.py`):
   - Servo türü kayıt defteri (gövde/tab/delik/mil/spline/kablo/kütle/aktüatör sınıfı; `research/servo-mechanics.md`),
     eklem → servo türü haritası, tek `scale`, braket/beşik/idler/döner tabla üreteçleri servo zarfından türetilir.
   - Konfigler: `configs/xl330_27.json` ve `configs/tr35_b2.json`.
   - **İlk aşama regresyon**: xl330_27 konfigi, refactor'dan önceki micro `build/sim/openduck.xml`'i bayt-eşit üretir.
   - Eklem adları, eksenleri, sırası ve işaret kuralı (pozitif açı = horn'a bakarken CCW) korunur.
2. **Dışa aktarma** (`cad/export.py`, `cad/docs.py`):
   - MJCF: sınıf başına `position` aktüatörü (`ds32xx`, `es3054`, `mg90s`, `sg90`); ctrlrange = eklem sınırı; eklem
     range = ctrlrange + 3° (politika sınırı bedava destek olarak kullanamaz).
   - BOM: Türkiye mağaza/TL/URL/stok/doğrulama tarihi ve yedek kaynak. Baskı seti 256 mm tabla kontrolüyle;
     teknik resimler. Kütle aşağıdan yukarı hesaplanır (braket hacmi × efektif yoğunluk; vida/horn/idler satırları).
3. **Aktüatör modeli** (`rl/actuators.py` yeni; `robot_model.build_spec` içinde MjSpec ile uygulanır):
   - Parametreler: kp = τs / P-bandı, forcerange = τs (ray geriliminde), eklem sönümü b = τs/ω0, armature,
     frictionloss, sınıf başına ölü bant.
   - **Tork tanımı**: τ_act = actuator_force, τ_net = τ_act − b·qd (motor akımıyla orantılı). Isıl ve doygunluk
     ölçütleri τ_net / τs üzerinden; ceza bölüm başına örneklenen τs ile, kabul nominal τs ile.
   - Ölü bant env'de alt-adım ctrl şekillendirmesiyle. Boşluk, DS eklemlerinde bayrakla açılan seri boşluk eklemi
     (±b/2). Komut→açı hatası aktüatör tarafında ctrl biası.
   - **Paylaşımlı ray modeli**: I_rail = Σ I_stall,i·|τ_net,i|/τs,i + boşta akım. I_rail > I_CC ise DS
     forcerange'leri I_CC/I_rail ile ölçeklenir.
   - Varsayılanlar (Adım 0'a kadar): DS3230MG Blue 2.88 N·m, 5.72 rad/s, P-bandı 3°.
4. **Sensör modeli** (`rl/sensors.py` yeni; firmware'in birebir kopyalayacağı tek tanım):
   - Pot q: bölüm başına bias ±1.5°, yük akımına bağlı toprak-düşümü biası, ortak-mod ray hatası, ADC beyaz
     gürültü, 12-bit (ADC1) / 16-bit (ADS1115) kuantalama, RC τ 3.2 ms, yol başına gecikme (ADC1 1 alt-adım,
     ADS1115 örnekleme dinamiği).
   - qd = 200 Hz fark + IIR (α 0.47). Gaga komutla gözlenir.
   - IMU: gyro gürültü/bias/ölçek, ivmeölçer gürültü/titreşim. Gravity gözlemini aynı tamamlayıcı filtre üretir.
   - Aksiyon gecikmesi U{0..2} alt-adım (DS 300 Hz), U{0..4} (50 Hz servolar).
   - Ödüller ve kritik gerçek durumu görür.
5. **Görevler** (`rl/config.py`, `rl/tasks.py`, `rl/task_math.py`):
   - Sabitler TR-35 için yeniden türetilir (rl-map tablosu). Yükseklikler, F tablosu, SEAT ve HEAD_SIT, donmuş
     CAD'den bir araçla (phys-sit family3 portu) üretilir; boyun ve kafa statik torku kısıt olarak girer.
   - Yürüyüş: **GAIT_FREQ = 2.21·κ Hz**, komut aralıkları ve AIR_TIME κ ile ölçeklenir. Başlangıç κ = 0.5.
     Kabul eğitiminde κ, Adım 0 kapısından gelir.
   - main: rampa ve pencereler λ_t = √s ile (κ uygulanmaz). Rampalar 2.83 s; kalkış penceresi 3.7 s, oturuş 4.0 s.
   - Cezalar: τ_net/τs ile normalize tork cezası, limit kısıt torku (efc) cezası, ray akımı aşım cezası. getup'ta
     ayrıca kafa darbe cezası.
   - getup: kafa-yaw savurmalı eski strateji kaldırılır. Güçlü boyun + bacaklarla yeni strateji RL ile öğrenilir
     (bölüm 12 s), poz bankaları yeni modelden. Önce yapılabilirlik kapısı (§3.4).
   - Eğitim sırası: walk → main → getup.
6. **Politika ve dışa aktarma**: aktör 256-128-64; kritik büyük kalabilir. ONNX + C dizisi (normalizasyon katlanmış,
   float32). Küçük aktör A4–A6'yı tutmazsa 512-256-128 + int8/PSRAM tahmini raporlanır.
7. **Değerlendirme** (`rl/eval_tasks.py`, `rl/play.py` güncellemesi kapsamda): tork/ray/limit metrikleri, mod başına
   getup, main yürüme, main → getup → main devir senaryosu, Wilson %95 aralıkları.
8. **Tezgâh ve sysID** (`tools/bench/`, `tools/sysid/` yeni): ESP32 tezgâh betiği (PWM süpürme, ADC/INA kaydı,
   basamak/chirp), PC kaydedici. Log'dan τs, ω0, P-bandı, armature, frictionloss, ölü bant, gecikme ve pot doğrusallığı
   uydurulur; sonuç `rl/actuators.py` parametrelerine yazılır.

### 3.3 Veri akışı ve sıralama kapıları

`configs/tr35_b2.json` → FreeCAD build → `build/` (STEP, STL, print set, drawings, BOM, `build/sim/openduck.xml`)
→ **G-CAD kapısı** (trunk/bacak geometrisi, kütle ve COM donar; A1 geçer) → RL sabitleri, F tablosu, poz bankaları
→ boru hattı denemesi eğitimi (varsayılan parametreler, kabul değil) → **G-BENCH kapısı** (Adım 0 sonuçları + sysID)
→ kabul eğitimleri (RunPod, onaylı) → `rl/eval_tasks` (A4–A8).

### 3.4 Kalkma yapılabilirlik kapısı

Tam getup eğitiminden önce iki kısa koşu: (i) kafa sabit/toplanmış (aksiyon dışı), (ii) kafa serbest. İkisinde de
kafa torku sınırlı. Mod başına (yüz/yan/sırt) başarı raporlanır. Sırt koşusu tabanın altında kalırsa spec'e dönülür
(seçenekler: sırttan kalkmayı v1'den çıkarmak ya da donanım değişikliği).

## 4. Kısıtlar ve varsayımlar

- Bütün satın alınan parçalar Türkiye'de stokta; BOM her satırda doğrulama tarihi ve yedek kaynak taşır.
- Servo hızı için üç farklı yayımlanmış değer var (0.12, 0.14 ve 0.18 s/60°). Kesin değer Adım 0'da ölçülür.
  - Adım 0'a kadar en kötü değer (DSSERVO Blue) kullanılır, DR geniş: tork ×(0.7–1.15), ω0 ×(0.75–1.4),
    P-bandı 1–8°, ölü bant sınıf başına, kütle ×(0.9–1.15).
  - Kabul eğitiminde DR, ölçülen değerlerin etrafında ±%15'e daralır.
- Kütle: sert sınır 1.95 kg, hedef 1.85 kg, aşağıdan yukarı hesap. Isıl ölçütler ölçülen kütleyle değerlendirilir.
- Mevcut RL hattı (MJX + Brax PPO, CPU torch, eval/play, RunPod başlatıcı) korunur.

## 5. Riskli senaryolar

1. **Diz ısınması** (~1.9 kg'da diz τ_net RMS ≈ 1.0 N·m ≥ Blue stall/3 0.96). Isıl eşik Adım 0 ısınma testiyle
   kalibre edilir. Çıkış yolları: dizi daha az büken HOME/H_HOME, diz tork cezası, DS3230 Red. DS3225 ısıl yedek
   değildir.
2. **hip_yaw aşırı yükü**: eski politika gereğinden 10× fazla yaw torku kullanıyordu. Tork cezası ve ölçütle
   sınırlanır; döner tabla servo milini eğilmeden korur.
3. **Ray çökmesi**: kalkmada 5+ DS aynı anda doyarsa > 18 A çekilir. Paylaşımlı ray modeli ve cezası; yedek olarak
   bacak başı ayrı buck.
4. **Eklem sınırına yaslanma**: sim sınırı bedava destek olarak kullanırsa gerçekte servo geri sürülür. Range +3°,
   limit torku ölçütü ve cezası.
5. **Kafa zinciri yükü**: MG90S plastik iç dişli. Tampon yük yolu, kafa tork ölçütleri, getup kapısı.
6. **Pot toprak düşümü / ray hatası**: sensör modelinde akıma bağlı bias ve ortak-mod hata; 20 AWG yıldız toprak.
7. **LiPo derin deşarj** (firmware yokken): voltaj alarmı ve ana kesme.
8. **Adım 0 varsayılanları çürütürse**: §7 karar tablosu.
9. **Stok tükenmesi**: DS32xx gövde uyumlu; her BOM satırında yedek kaynak.

## 6. Kabul kriterleri

- **A1 CAD**:
  - `tools/build.sh --check` hatasız.
  - §3.1 gerekli aralıklar ⊆ eklem sınırları. Komşu eklem çiftleri birlikte taranır, kablo demetleri dahil 0 çarpışma.
  - Aynı link içi 0 çakışma; elektronik zarfları dahil.
  - Baskı parçaları 256 mm küpe sığar. Stand'da kafa üstü 350 ± 5 mm.
  - Kütle ≤ 1.95 kg (hedef 1.85).
  - hip_yaw ve DS idler'ları için "servo miline eğilme/eksenel yük yok" el hesabı.
  - xl330_27 regresyonu bayt-eşit.
- **A2 BOM**:
  - Her parçada mağaza, TL, URL, stok, doğrulama tarihi ve yedek kaynak.
  - Zorunlu toplam ≤ 23.500 TL, yedeklerle ≤ 25.000 TL. Her iki DS adayı için ayrı toplam.
- **A3 Model**:
  - MuJoCo ve MJX'te yüklenir; 4 aktüatör sınıfı.
  - HOME hedefleriyle açık döngü 5 s ayakta (gövde eğimi ≤ 2°).
  - CPU ile MJX tutarlılığı: aynı durumdan tek adım atol 1e-4; 50 adımlık eşli koşu mevcut parity testiyle.
- **A4 Yürüme** (walk ve main h = H_HOME, sit = 0; Adım 0 parametreleri; DR + itme + boşluk açık; ≥ 256 bölüm × 20 s;
  Wilson %95):
  - Düşme ≤ %3. vx = 0.5·vx_max komutunda ölçülen ≥ %80; dönüş ±%20.
  - main takip ödülü ≥ walk'ın %90'ı.
  - DS: τ_net/τs p99 ≤ 0.85. Diz τ_net RMS ≤ ısıl eşik (Adım 0; varsayılan stall/3).
  - ES3054 ve MG90S: τ_net RMS ≤ stall/3, p99 ≤ 1.0. hip_yaw RMS ≤ 0.11 N·m.
  - Limit torku p99 ≤ 0.1·τs. Ray p99 ≤ 16 A; > 15 A süre ≤ %2.
- **A5 Boy + otur/kalk** (DR):
  - Boy RMS ≤ 1.3 cm.
  - Otur ≤ 4.0 s, kalk ≤ 3.7 s; her biri ≥ %95. Geçişlerde düşme < %5.
  - A4 tork, ray ve limit ölçütleri burada da geçer.
- **A6 Düşünce kalkma** (DR, düşmüş bankadan, 12 s):
  - Mod başına taban: yüz ≥ %80, yan ≥ %70, sırt ≥ %50 (Wilson).
  - Kafa servoları |τ_net|/stall p95 ≤ 1.0 ve doygun adım oranı ≤ %5.
  - Kafa-zemin tepe kuvveti medyanı raporlanır.
  - Ray ve limit ölçütleri geçer.
  - Sim devir senaryosu main → düşme → getup → main en az %80 başarılı.
- **A7 Konuşlandırılabilirlik**:
  - Aktör 256-128-64; ONNX ve C dizisi çıktıları eşit (≤ 1e-5).
  - MAC sayısı ve tahmini ESP32-S3 süresi raporlanır (hedef < 2 ms).
  - Gerekirse büyük aktör + int8 tahmini.
- **A8 Öz-çarpışma**: değerlendirme rollout'larında görsel mesh girişimi ≤ 1 mm³ (eklem arayüzü toleransıyla).
- **A9 Testler**: CPU ve MJX takımları geçer. `servos`, `actuators`, `sensors`, sysID için birim testler; xl330
  regresyonu; parity.

## 7. Adım 0 (kullanıcı, paralel): tezgâh testi ve karar tablosu

**Alınacaklar** (~3.6k TL): 1 × Robocombo DS3225MG, 1 × Direnc DS3230, 1 × ES3054, 1 × MG90S, PCA9685,
ESP32-S3, INA226. Betik: `tools/bench/`.

**Ölçümler** (6.5 V; MG90S 6.0 V):
- boşta hız (60° süresi), stall akımı ve torku (kol + yay terazisi);
- hareket aralığı/µs, ölü bant, 300 Hz ve 3.3 V darbe kabulü;
- pot teli: besleme kaynağı (ray mı iç regülatör mü), gerilim aralığı, monotonluk;
- basamak/chirp tepkisi, yavaşlamada ray aşımı;
- 10 dakikalık diz benzeri döngüsel yükte kasa sıcaklığı.

**Karar tablosu**:

| Ölçüm | Sonuç | Eylem |
|---|---|---|
| u(τs, ω0, kütle) | ≤ 0.85 (κ 0.72) | κ = 0.72 |
| | ≤ 0.85 (κ 0.6) | κ = 0.6 |
| | ≤ 0.85 (κ 0.5) | κ = 0.5 |
| | hiçbiri | Dur, spec'e dön (servo/ölçek) |
| Pot | yok, monoton değil ya da < 1 V | Dur, spec'e dön (B1 geri bildirimsiz ya da B3) |
| 300 Hz | reddedildi | Bacak kartı 50 Hz; gecikme U{0..4}, çözünürlük 4.9 µs; yeniden eğit |
| Isınma | stall/3'te 60 °C üstü | Isıl eşiği düşür; diz cezasını artır ya da DS3230 Red |
| Kazanan DS | iki aday karşılaştırılır | Hız/ısıl/stok dengesiyle seçilir. Sonuç parametreleri `tools/sysid` ile `rl/actuators.py`'e yazılır, kabul eğitimleri bunlarla koşar |
