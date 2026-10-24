# Spec: OpenDuck-TR (v3.2, CAD + simülasyon + RL)

Girdi: `research/` (parts-research, torque-needs, servo-mechanics, cad-map, power-electronics, rl-map,
discovery-critic). İnceleme: `review.md` (tur 1: 34 itiraz, tur 2: 20 itiraz; tur 2'deki iki kritik itiraz kullanıcı
kararıyla çözüldü). Sayıların kaynakları research'tedir; bu belge kararları verir.
v3.1 (2026-10-23, P2-0 bulguları): öz-çarpışma çiftlerine kafa ↔ trunk ve ayak ↔ aynı bacağın uyluk/kalça beşiği
eklendi; A1 komşu çift taraması modül içinde sayılır, modellenen temaslar ayrı listelenir (`p2-interface.md` §12).
v3.2 (2026-10-24, P2 entegrasyonu ve doğrulama tur 2, `p2-entegrasyon.md` §2 ve §10): öz-çarpışma çiftlerine
çene ↔ boyun/trunk ve kafa ↔ kalça/kalça beşiği eklendi; A1'de aynı geçişin iki kablo tüpü yalnız geçiş noktasının 2d
çevresinde birbirine değebilir, komşu çift ızgarasında "yapı önce" kuralı, `check.py` sayılan çarpışmada çıkış kodu 1.

## 1. Problem ve hedef

Türkiye'de bugün stokta olan, ucuz parçalarla basılabilen ~35 cm'lik BDX/Microduck tarzı bir ördek robotun
**parametrik CAD'i, baskı seti, BOM'u, servo ve sensör modelli MuJoCo modeli ve simülasyonda eğitilmiş politikaları**
(yürüme, boy ayarı + oturma/kalkma, düşünce kalkma). Donanım "B2-ESP32": PWM servolar, potansiyometre geri bildirimi,
politika ESP32-S3'te (firmware sonraki tur; politika ve sensör modeli ona göre). Bütçe tavanı: zorunlu ≤ 25.8k TL,
yedeklerle ≤ 28.8k TL (kullanıcı; sesli komut eki K10 ile yükseltildi). Sesli komut katmanı: `spec-ek-komut.md` (ek K).

## 2. Kapsam dışı

- Robot firmware'i (görevler, süpervizör, kumanda, kalibrasyon depolama). **İstisna**: Adım 0 tezgâh betiği.
- Satın alma, tezgâh testinin yürütülmesi (kullanıcı) ve gerçek robot denemeleri.
- 27 cm XL330 profilinin kalıcı bakımı. Yalnız refactor regresyon referansı olarak `configs/xl330_27.json` kalır
  (§3.2.1). Walk için "bit-düzeyinde aynı" sözleşmesi kalkar.
- Kafa elektroniği (kamera, hoparlör, ToF); göz kozmetik baskı. İstisna: DF2301Q ses modülü ve gösterge LED'i (ek K3,
  kullanıcı kararı 2026-10-24).

## 3. Mimari

### 3.1 Donanım

| Eklem (sol/sağ) | Servo | Adet | Not |
|---|---|---|---|
| hip_roll, hip_pitch, ankle | DS32xx | 6 | 180° (500–2500 µs), 6.5 V, alüminyum 25T kol |
| knee | DS32xx (DS3235 adayı dahil) | 2 | Isıl seçim Adım 0'da |
| neck_pitch | DS32xx | 1 | Gövde trunk'ta sabit, horn boynu sürer (horn yönü −y, sabit) |
| hip_yaw | EMAX ES3054 | 2 | 6.0 V; yalnız tork iletir (döner tabla) |
| head_pitch, head_yaw, head_roll | MG90S | 3 | 6.0 V |
| mouth | SG90 | 1 | 6.0 V |

- **DS32xx adayları** (aynı 40 × 20 × 40.4 gövde, CAD değişmez). Seçim Adım 0'da yapılır.
  - DS3225MG: Robocombo, 1.031 TL, 99 stok.
  - DS3230: Direnc, 1.046 TL, 15 stok.
  - DS3235 (yalnız diz için): Robocombo, 1.171,50 TL, 134 stok; spesifikasyonu doğrulanacak.
  - Yedek kaynak: Motorobit DSSERVO DS3230.
- **Geri bildirim** (15 eklemin 14'ü ölçülür):
  - 9 DS + 2 ES3054 → 10'u ESP32-S3 ADC1'de (GPIO1–10).
  - Boyun ve 3 kafa potu → ADS1115 #1 (0x48).
  - 2 diz NTC'si → ADS1115 #2 (0x49); 2 kanalı boş (biri ortak-mod referansı için ayrılabilir).
  - Bütün pot girişlerinde aynı bölücü: 100k/47k + 100 nF (τ 3.2 ms).
  - ADC1: 12 dB. ADS1115: VDD 3.3 V, PGA ±4.096 V, 860 SPS mux (kanal başı ~215 Hz).
  - Gaga ölçülmez.
- **Elektronik**:
  - ESP32-S3-DevKitC-1 N16R8.
  - PCA9685 0x40: 9 DS, prescale 19 ya da 20 (≈ 300 Hz).
  - PCA9685 0x41: ES3054 + MG90S + SG90, 50 Hz. ES3054 300 Hz'i kabul ederse 0x40'a taşınır.
  - MPU6500 (SPI, GPIO39–42). INA226 yalnız VBUS ölçer.
- **Güç** (hazır LiPo):
  - Profuse 4S 1500 mAh 95C. Denge soketinde LiPo voltaj alarmı.
  - Zincir: XT30 ana kesme → 15 A sigorta → 16.8 V DC'ye uygun anahtar (stok planda doğrulanır; yoksa yalnız XT30)
    → raylar:
    - 20 A buck, 6.50 V, CC ≈ 16 A → 9 DS. 2200 µF ve aşım kırpıcı: TVS ya da zener + MOSFET; stokta yoksa
      büyük kapasitör + Adım 0 ölçümü.
    - XL4015, 6.0 V → ES3054 + MG90S + SG90.
    - MP1584, 5 V → lojik (1N5819).
  - CAD'de güç kesme MOSFET/latch yeri ayrılır. Dengeli şarj cihazı BOM'da (iMAX B6AC sınıfı).
- **Mekanik**:
  - Ölçek s = 1.282. Nominal stand keyframe'inde kafa üstü 350 ± 10 mm.
  - Geniş duruş. Yazıcı 256 × 256 × 256 mm. Yapısal PETG; kabuklar PLA, sabit et kalınlığı; taban TPU 95A.
    M3 somun yuvası kullanılır.
  - DS eklemleri: baskı beşik + U-link; eksenle hizalı arka idler (F623ZZ, M3 mil); alüminyum 25T kol linke gömülü.
    hip_pitch horn'u +14.4° ofsetli (bir diş). Boyun nötrü −25°.
  - hip_yaw: iki rulmanlı döner tabla (≥ 20 mm aralık; 608ZZ ya da 6800ZZ, stok planda). ES3054'e pim/yarık
    kaplinle yalnız tork gider.
  - Çene/alın tamponu kafa darbesini boyun U-link'ine aktarır. MJCF'de boyun gövdesine bağlı çarpışma geomudur.
  - Bütün elektronik CAD'de gerçek zarfıyla bulunur. Trunk iç hacim bütçesi ve buck havalandırması tasarlanır.
    Isı kaynağına PETG kullanılır, PLA'ya ≥ 5 mm hava bırakılır.
  - Eklem geçişlerindeki kablo demetleri (çap, bükülme yarıçapı) CAD parametresidir.
  - **Kütle merdiveni**, sırayla uygulanır:
    1. sabit etli kabuklar
    2. iskeletli U-link ve beşik (~−60 g)
    3. servo uzatması yerine doğrudan lehim (~−20 g)
    4. döner tablada ince kesitli 68xx (~−25 g)
    5. dolgu ve et parametreleri
- **Gerekli eklem aralıkları** (derece, sol bacak işareti; birleşik ihtiyaç):

| Eklem | walk | main | getup | Birleşik gerekli | Not |
|---|---|---|---|---|---|
| hip_yaw | −10…+4 | −10…+4 | −10…+10 | −10…+10 | İç sınır CAD'den. < 10° çıkarsa wz aralığı daralır |
| hip_roll | ±12 | ±12 | ±20 | ±20 | |
| hip_pitch | −52…+16 | −31…+90 | −70…+90 | −70…+90 | Sınır −70…+90 |
| knee | −37…+40 | −3…+62 | −37…+65 | −37…+65 | |
| ankle | −26…+57 | −45…+49 | −45…+57 | −45…+57 | |
| neck_pitch | 1…40 | 1…40 | −90…+40 | −90…+40 | Nötr −25 |
| head_pitch | 1…38 | 1…38 | −20…+60 | −20…+60 | |
| head_yaw | ±34 | ±34 | ±34 | ±34 | Sınır ±80 |
| head_roll | ±6 | ±6 | ±10 | ±10 | |
| mouth | 0…30 | 0…30 | 0…30 | 0…30 | |

### 3.2 Yazılım bileşenleri

1. **CAD refactor** (`cad/servos.py` yeni, `cad/openduck.py`):
   - Servo türü kayıt defteri (gövde/tab/delik/mil/spline/kablo/kütle/aktüatör sınıfı) ve eklem → servo türü haritası.
   - Tek `scale` parametresi. Braket, beşik, idler, döner tabla ve tampon üreteçleri servo zarfından türetilir.
   - Konfigler: `configs/xl330_27.json`, `configs/tr35_b2.json`.
   - **İlk aşama regresyon**: xl330_27, refactor öncesi micro `build/sim/openduck.xml`'i bayt-eşit üretir.
   - Eklem adları, eksenler, sıra ve işaret kuralı (pozitif açı = horn'a bakarken CCW) korunur.
2. **Dışa aktarma** (`cad/export.py`, `cad/docs.py`):
   - Sınıf başına MJCF `position` aktüatörü (`ds32xx`, `es3054`, `mg90s`, `sg90`); ctrlrange = eklem sınırı.
   - Öz-çarpışma çiftleri: uyluk/incik ↔ trunk/kafa/çene, ayak ↔ kafa, bacak ↔ bacak, kafa ↔ trunk,
     ayak ↔ aynı bacağın uyluk/kalça beşiği, çene ↔ boyun/trunk, kafa ↔ kalça/kalça beşiği. Tampon geomu.
   - BOM: Türkiye mağaza, TL, URL, stok, doğrulama tarihi ve yedek kaynak.
   - Baskı seti 256 mm kontrolüyle; teknik resimler.
   - Aşağıdan yukarı kütle (braket hacmi × efektif yoğunluk; vida, horn ve idler satırları).
3. **Aktüatör modeli** (`rl/actuators.py` yeni; `robot_model.build_spec` içinde MjSpec):
   - Parametreler: kp = τs / P-bandı, forcerange = τs, eklem sönümü b = τs/ω0, armature, frictionloss, sınıf başına
     ölü bant. Ölü bant env'de alt-adım ctrl şekillendirmesiyle uygulanır.
   - **Tork tanımı**: τ_act = actuator_force, τ_net = τ_act − b·qd. Isıl ve doygunluk ölçütleri τ_net/τs ile.
     Ceza bölüm başına örneklenen τs ile, kabul nominal τs ile.
   - **Eklem sınırı**: bütün varyant sınırlayıcılarından sonra `range = ctrlrange + 3°` değişmezi uygulanır
     (birim testli). KNEE, NECK ve HIP_YAW sınırları TR-35 CAD'inden yeniden türetilir.
   - **Limit metriği ve cezası** yalnız aktüatörlü ana eklemlerin LIMIT_JOINT satırlarından hesaplanır; boşluk
     eklemleri hariç.
   - **Boşluk**: DS eklemlerinde servo çıkışı ile link arasında, bayrakla açılan seri eklem (±b/2, DR 0–1°).
     Ara gövde armature 1e-5 kg·m² ve sert solref taşır. Pot servo çıkış milini okur. Dişli içi boşluk ölü bant ve
     P-bandı DR'ıyla temsil edilir.
   - **Besleme akımı**: I_supply,i ≈ I_stall,i·|τ_act,i|·|τ_net,i|/τs,i² + boşta akım.
     - Ray: I_rail = Σ I_supply,i. Talep I_CC'yi aşarsa DS forcerange'leri I_CC/I_rail ile ölçeklenir.
     - Metrik, ölçekleme öncesi talep akımıdır.
   - Varsayılanlar (Adım 0'a kadar): DS3230MG Blue 2.88 N·m, 5.72 rad/s, P-bandı 3°.
4. **Sensör modeli** (`rl/sensors.py` yeni; firmware'in birebir kopyalayacağı tek tanım):
   - Pot q:
     - bölüm başına bias ±1.5°;
     - akıma bağlı toprak-düşümü biası ve ortak-mod ray hatası;
     - ADC beyaz gürültü; kuantalama 12-bit (ADC1) / 16-bit (ADS1115);
     - RC τ 3.2 ms; yol başına gecikme (ADC1 1 alt-adım, ADS1115 mux örnekleme dinamiği).
   - qd: 200 Hz fark + IIR (α 0.47). Gaga komutla gözlenir.
   - IMU: gyro gürültü/bias/ölçek, ivmeölçer gürültü/titreşim. Gravity gözlemini aynı tamamlayıcı filtre üretir.
   - Aksiyon gecikmesi: U{0..2} alt-adım (DS 300 Hz), U{0..4} (50 Hz servolar).
   - Ödüller ve kritik gerçek durumu görür.
5. **Görevler** (`rl/config.py`, `rl/tasks.py`, `rl/task_math.py`):
   - Sabitler TR-35 için yeniden türetilir. Yükseklikler, F tablosu, SEAT ve HEAD_SIT donmuş CAD'den bir araçla
     üretilir; boyun ve kafa statik torku bu araçta kısıttır.
   - **HEAD_RANDOM** MG90S'e göre: neck 0.15, head_pitch 0.2, head_yaw 0.5, head_roll 0.08 rad; filtre 0.5 s;
     eğim sınırı ≤ 0.5·ω0(MG90S).
   - Yürüyüş: **GAIT_FREQ = 2.21·κ Hz**; komut aralıkları ve AIR_TIME κ ile. Başlangıç κ = 0.5. Kabulde κ Adım 0
     kapısından gelir.
   - main: λ_t = √s, κ uygulanmaz. Rampalar 2.83 s; kalkış penceresi 3.4 s, oturuş 4.0 s.
   - Cezalar: τ_net/τs ile normalize tork, limit torku, ray talebi, getup'ta kafa darbesi.
   - getup: kafa-yaw savurmalı eski strateji kaldırılır. Güçlü boyun + bacaklarla yeni strateji RL ile öğrenilir
     (bölüm 12 s); poz bankaları yeni modelden. Önce kapı (§3.4).
   - Eğitim sırası: walk → main → getup.
6. **Politika ve dışa aktarma**: aktör 256-128-64; ONNX + C dizisi (normalizasyon katlanmış). Gerekirse büyük
   aktör ya da damıtılmış öğrenci + int8.
7. **Değerlendirme** (`rl/eval_tasks.py`, `rl/play.py`): tork, ray, limit ve ısıl metrikler; mod başına getup;
   main yürüme; main → getup → main devri; N ve Wilson kuralları §6'daki gibi.
8. **Tezgâh ve sysID** (`tools/bench/`, `tools/sysid/` yeni):
   - ESP32 tezgâh betiği: PWM süpürme, ADC/INA kaydı, basamak/chirp, çok seviyeli ısınma döngüsü. PC kaydedici.
   - Log'dan uydurulan parametreler: τs, ω0, P-bandı, armature, frictionloss, ölü bant, gecikme, pot doğrusallığı,
     besleme akımı modeli ve T_diz. Sonuç `rl/actuators.py`'e yazılır.

### 3.3 Veri akışı ve kapılar

`configs/tr35_b2.json` → FreeCAD build → `build/` → **G-CAD** → RL sabitleri/banka → boru hattı eğitimi
(varsayılanlar) → **G-BENCH** → **G-ISIL** → kabul eğitimleri (RunPod, onaylı) → eval (A4–A8).

| Kapı | Koşul | Tutmazsa |
|---|---|---|
| G-CAD | A1 geçer; kütle = CAD aşağıdan yukarı × 1.05 ≤ 1.95 kg (merdiven sonrası); trunk ve bacak geometrisi ile COM yazılır | Kütle hâlâ > 1.95 kg ise RL'den önce spec'e dönülür |
| G-BENCH | Adım 0 ölçümleri + sysID parametreleri hazır | §7 karar tablosu |
| G-ISIL | Boru hattı walk/main politikalarının diz τ_net RMS'i, tezgâh τs ve G-CAD kütlesiyle ölçeklenir; ≤ T_diz | Önce stand keyframe'i (H_HOME/HOME) dizi daha az bükecek şekilde değiştirilir (A1 yükseklik toleransı içinde), RL sabitleri yeniden türetilir; ya da diz servosu DS3235'e geçer |

### 3.4 Kalkma yapılabilirlik kapısı

- G-BENCH parametreleriyle, tam getup bütçesinin %30'u kadar iki koşu yapılır:
  - (i) kafa sabit/toplanmış (aksiyon dışı);
  - (ii) kafa serbest.
- Kafa torku sınırlı. Mod başına (yüz, yan, sırt) başarı, öğrenme eğrisinin eğimi ve A8 raporlanır.
- Geçme koşulu: her modda başarı ≥ 0.6 × A6 tabanı (yüz 48, yan 42, sırt %30) ve eğim > 0.
- Tam eğitim (i) ile (ii)'den hangisi iyiyse onunla yapılır.
- Düşen bir mod varsa spec'e dönülür. Seçenekler: o modu v1'den çıkarmak ya da donanım değişikliği.

## 4. Kısıtlar ve varsayımlar

- Bütün satın alınan parçalar Türkiye'de stokta. BOM her satırda doğrulama tarihi ve yedek kaynak taşır.
- **Bütçe**:
  - Zorunlu ≤ 25.8k TL, yedeklerle ≤ 28.8k TL (ek K10; ses katmanı dahil).
  - Yedek seti: tezgâhta kaybeden DS, 1 × ES3054, 1 × MG90S, 1 × SG90, 2 × alüminyum 25T kol, 2 × F623ZZ.
  - Adım 0 alımları robota ya da yedeğe gider ve tavana dahildir.
- **Servo hızı**: üç yayımlanmış değer var (0.12, 0.14, 0.18 s/60°).
  - Adım 0'a kadar en kötüsü (DSSERVO Blue) kullanılır, DR geniş: tork ×(0.7–1.15), ω0 ×(0.75–1.4),
    P-bandı 1–8°, ölü bant sınıf başına, kütle ×(0.9–1.15), boşluk 0–1°.
  - Kabulde DR ölçülen değerlerin ±%15'ine daralır.
- Kütle sert sınırı 1.95 kg, hedef 1.85 kg.
- Mevcut RL hattı (MJX + Brax PPO, CPU torch, eval/play, RunPod başlatıcı) korunur.

## 5. Riskli senaryolar

1. **Diz ısınması**: G-ISIL kapısı, iki yönlü kalibre edilen T_diz ve DS3235 adayıyla yönetilir.
2. **hip_yaw yükü**: döner tabla, tork cezası ve ölçüt.
3. **Ray çökmesi**: besleme akımı modeli, CC ölçeklemesi ve talep ölçütü. Yedek olarak bacak başı ayrı buck.
4. **Sınıra yaslanma**: range +3° değişmezi ve ana eklem limit metriği.
5. **Kafa zinciri yükü**: tampon geomu, kafa tork ve limit ölçütleri, getup kapısı.
6. **Öz-çarpışma** (simde bacağın gövdeden geçmesi): çarpışma çiftleri, A8 ve kapı raporu.
7. **Pot hataları**: toprak düşümü, ortak-mod hata ve gürültü modeli; yıldız toprak.
8. **LiPo derin deşarj**: voltaj alarmı ve ana kesme.
9. **Adım 0 varsayılanları çürütürse**: §7 karar tablosu.
10. **Stok tükenmesi**: DS32xx gövde uyumlu; yedek kaynaklar.

## 6. Kabul kriterleri

Oran ölçütlerinde "nokta" örnek oranıdır. "W-alt" ve "W-üst" Wilson %95 sınırlarıdır.

- **A1 CAD**:
  - `tools/build.sh --check` hatasız.
  - §3.1 gerekli aralıklar ⊆ eklem sınırları. Komşu eklem çiftleri, bacak–bacak ve kablo demetleri birlikte
    taranır: 0 çarpışma. Tek eklem taramasında istisna yoktur. Komşu çift ızgarası hareket eden modülün içinde sayılır.
    Gövde/kafa ↔ uzuv ve `check.contact_pairs` temasları (iki ayak; ayak ↔ aynı bacağın uyluk/kalça beşiği) MJCF
    öz-çarpışma çifti olarak modellenir ve ayrı listelenir.
  - Kablo demeti temasları sayılır. İki istisna vardır (v3.2):
    - Aynı eklem geçişinin iki tüpü, geçiş noktasının 2d çevresinde (d: demet çapı) birbirine değebilir. Kesişimin
      ayrık parçalarından kütle merkezi bu kürenin dışında kalanlar sayılır.
    - Komşu çift ızgarasında, uzvun kendi yapısının da gövdeye/kafaya girdiği bir pozda, HOME → poz doğrusal eklem
      yolunda (en büyük eklem değişiminin 1° adımlarıyla) ilk modül dışı temas yalnız yapıysa, o pozun kablo temasları
      sayılmaz. Bunlar MJCF gövde ↔ uzuv çiftiyle birlikte ilk temas adımıyla listelenir ("yapı önce"). Yalnız kablonun
      değdiği pozlar ve yolda kablonun önce değdiği pozlar sayılır.
  - `cad/check.py`, sayılan herhangi bir çarpışmada çıkış kodu 1 verir; `tools/build.sh --check` başarısız olur.
  - Aynı link içi 0 çakışma (elektronik dahil). Baskı parçaları 256 mm küpe sığar.
  - Kafa üstü 350 ± 10 mm. Kütle ≤ 1.95 kg.
  - "Servo miline eğilme/eksenel yük yok" el hesabı.
  - xl330_27 regresyonu bayt-eşit.
- **A2 BOM**:
  - Her satırda mağaza, TL, URL, stok, tarih ve yedek kaynak.
  - Her DS adayı kombinasyonu için zorunlu ≤ 25.800 TL ve yedekli ≤ 28.800 TL (ek K10; ses katmanı dahil).
- **A3 Model**:
  - MuJoCo ve MJX'te yüklenir; 4 aktüatör sınıfı; öz-çarpışma çiftlerinin MJX adım maliyeti raporlanır.
  - Varyant sınırları ⊇ gerekli aralıklar ve +3° değişmezi testli.
  - HOME'da açık döngü 5 s ayakta (eğim ≤ 2°).
  - CPU ile MJX: tek adım atol 1e-4; 50 adımlık eşli koşu.
- **A4 Yürüme** (walk ve main h = H_HOME, sit = 0; G-BENCH parametreleri; DR + itme + boşluk açık; N = 512 × 20 s):
  - Düşme: W-üst ≤ %3.
  - vx = 0.5·vx_max'ta [5, 20] s ortalaması ≥ %80.
  - wz ∈ {±0.5·wz_max}'te [5, 20] s ortalaması ±%20 içinde.
  - main takip ödülü ≥ walk'ın %90'ı.
  - DS: τ_net/τs p99 ≤ 0.85. Diz τ_net RMS ≤ T_diz.
  - ES3054 ve MG90S: τ_net RMS ≤ stall/3, p99 ≤ 1.0. hip_yaw RMS ≤ 0.11 N·m.
  - Ana eklem limit torku p99 ≤ 0.1·τs; kafa eklemlerinde ≤ MG90S stall.
  - Ray talebi p99 ≤ 16 A; > 15 A süre ≤ %2.
- **A5 Boy + otur/kalk** (DR; home ve oturuş başlangıcı için ayrı ayrı N = 256):
  - Boy RMS ≤ 1.3 cm.
  - Otur ≤ 4.0 s ve kalk ≤ 3.4 s: her biri nokta ≥ %95 ve W-alt ≥ %90.
  - Geçişlerde düşme W-üst ≤ %5.
  - A4'ün tork, ray ve limit ölçütleri burada da geçer.
- **A6 Düşünce kalkma** (DR, düşmüş bankadan tabakalı örnekleme, mod başına N = 128, 12 s):
  - Mod başına nokta ≥ taban ve W-alt ≥ taban − 8 puan. Tabanlar: yüz %80, yan %70, sırt %50.
  - Kafa servoları |τ_net|/stall p95 ≤ 1.0; doygun adım oranı ≤ %5.
  - Ray ve limit ölçütleri geçer. Kafa-zemin tepe kuvveti medyanı raporlanır.
  - Devir senaryosu main → düşme → getup → main: N = 128, nokta ≥ %80.
- **A7 Konuşlandırılabilirlik**: iki daldan biri.
  - (a) Aktör 256-128-64 float32; C ve ONNX çıktıları ≤ 1e-5 eşit; tahmini süre < 2 ms.
  - (b) Büyük aktör ya da damıtılmış öğrenci int8; A4–A6'nın float öğretmene farkı ≤ 2 puan; tahmini süre < 2 ms.
- **A8 Öz-çarpışma**: değerlendirme rollout'larında görsel mesh girişimi ≤ 1 mm³ (eklem arayüzü toleransıyla).
- **A9 Testler**:
  - CPU ve MJX takımları geçer.
  - Birim testler: `servos`, `actuators` (+3° değişmezi, besleme akımı), `sensors`, sysID, boyun işareti.
  - xl330 regresyonu ve parity testi.
- **A10 Sesli komut** (simülasyon): `spec-ek-komut.md` K7.

## 7. Adım 0 (kullanıcı, paralel): tezgâh testi ve karar tablosu

**Alınacaklar** (tavana dahil; robota ya da yedeğe gider):
- DS3225MG (Robocombo), DS3230 (Direnc), DS3235 (Robocombo);
- ES3054, MG90S, SG90;
- PCA9685, ESP32-S3, ADS1115, INA226 + 2–10 mΩ şönt (ya da laboratuvar kaynağı).

Betik: `tools/bench/`.

**Ölçümler** (DS 6.5 V; ES3054, MG90S ve SG90 6.0 V):
- Boşta hız, stall akımı (iki yöntem, fark ≤ %10) ve stall torku (kol + terazi).
- Besleme akımı: farklı duty ve hız noktalarında.
- Hareket aralığı/µs, ölü bant, 300 Hz kabulü (DS ve ES3054), 3.3 V darbe kabulü.
- Pot: besleme kaynağı, gerilim aralığı, monotonluk, ortak-mod hata.
- Basamak/chirp; yavaşlamada ray aşımı.
- Isınma: diz benzeri döngüsel yük, 3 seviye (0.9 / 1.05 / 1.15 N·m RMS), 10'ar dakika.
  T_diz = kasa ≤ 60 °C kalan en yüksek RMS (25 °C ortam). Eşik yukarı da aşağı da çıkabilir.

**Karar tablosu**:

| Ölçüm | Sonuç | Eylem |
|---|---|---|
| u(τs, ω0, G-CAD kütlesi) | ≤ 0.85 (κ 0.72) | κ = 0.72 |
| | ≤ 0.85 (κ 0.6) | κ = 0.6 |
| | ≤ 0.85 (κ 0.5) | κ = 0.5 |
| | hiçbiri | Dur, spec'e dön |
| Pot | yok, monoton değil ya da < 1 V | Dur, spec'e dön |
| Pot ortak-mod hata | > 1° | Oranlı okuma (boş ADS kanalı referans) |
| DS 300 Hz | reddedildi | Bacak kartı 50 Hz, gecikme U{0..4}, çözünürlük 4.9 µs; yeniden eğit |
| ES3054 300 Hz | kabul | hip_yaw 0x40'a taşınır, gecikme U{0..2} |
| 3.3 V darbe | reddedildi | Seviye çevirici (BOM + CAD) |
| Ray aşımı | > 6.8 V | Kırpıcı ya da fren direnci; kapasitör artır |
| Isınma | T_diz ölçüldü | A4 ve G-ISIL bu değerle |
| Diz servosu | DS adayları | Yalnız T_diz(aday) ≥ G-ISIL tahmini diz RMS olan aday; aralarından hız (u) ve stok |
| Diğer DS ekleri | DS3225 ya da DS3230 | u ve stok dengesi; parametreler `tools/sysid` ile `rl/actuators.py`'e |
