# P2 entegrasyonu: bacaklar (P2a) + gövde (P2b) + boyun/kafa (P2c)

Tarih: 2026-10-24. Girdi: `p2-interface.md` (§12 kararları), P2a/P2b/P2c raporları ve değişiklik talepleri, orkestratör
kararları. Bu belge birleştirmeyi, kararları, entegrasyon düzeltmelerini, tam `check.py` sonuçlarını, boy/kütle/COM'u
ve sonraki adımlara devredilen açık konuları verir. `acik-konular.md` başka bir iz tarafından tutulur; bu adımın
açık konuları §8'dedir. Bağımsız doğrulamanın engelleyici bulguları ve düzeltmeleri §9'dadır; §4 ve §5 doğrulama
öncesi durumdur, güncel sonuçlar §9.6'da.

## 1. Birleştirilenler

Dal `main` (755e2ea) üzerinden, `--no-ff`, sırayla:

| Adım | Dal | Commit'ler | Birleştirme | Çakışma |
|---|---|---|---|---|
| P2b gövde + elektronik | `worktree-wf_760bffe2-b4a-2` | 5b6695c | ca39a67 | yok |
| P2c boyun + kafa | `worktree-wf_760bffe2-b4a-3` | a118ffb, a4bfd92, 53ca0ef | 5f754fa | yok (`configs/tr35_b2.json` otomatik) |
| P2a bacaklar | `worktree-wf_760bffe2-b4a-1` | 33a140c, 29ea4bf, b2f3aad, a41e25c, 8d0a87f | 67e2d22 | `joint_limits_deg`: bacak satırları P2a'dan, kafa satırları P2c'den |

Her modül kendi dosyalarına ve `params.legs` / `params.trunk` / `params.head` bölümlerine yazdığı için başka çakışma
çıkmadı. Entegrasyon değişiklikleri ba3b0a1'de (kod, konfig, testler), bu belge ve spec sonraki commit'tedir.

## 2. Değişiklik talepleri ve kararlar

| Talep | Kaynak | Karar | Uygulama |
|---|---|---|---|
| ICR-1: K_T kutusu [19.9, 26.2, 21.0, 26.65, 34.4, 39.7] (boyun idler yuvası bağı) | P2b | KABUL | Kutu `params.iface.envelopes.trunk.add` sonuna eklendi. VHB pedi (`neck_seat_vhb`) kaldırıldı; yerine baskılı PETG bağ var (§3.1). |
| ICR-2: K_T kutusu [-95, -92, -24, 24, -3, 31] (batarya kapağı) | P2b | KABUL | Kutu eklendi; 2 mm PETG `battery_door` modellendi (§3.2). |
| Boyun zarfı: tampon gövdesi kutusu [45.332, 51.5, -9.5, 9.5, 84, 98] | P2c | KABUL | `params.iface.envelopes.neck`'e eklendi. |
| Jaw zarfının değiştirilmesi (6 parça: SG90 horn plakası, göbek silindiri, iki kol, sağ pim, çene çubuğu) | P2c | KABUL, talep edildiği gibi | `params.iface.envelopes.jaw` değişti. |
| Yeni temas: çene ↔ boyun ve çene ↔ trunk | P2c | KABUL, MJCF öz-çarpışma çifti olarak | Spec §3.2.2 listesine "çene ↔ boyun/trunk" eklendi (v3.2). head_pitch ve mouth komşu eklem değildir; A1 bunu saymaz ve `check.contact_pairs` değişmedi. P3 dışa aktarımı ve P5b RL çift listesi için kayıt §8'de. Firmware/politika kilidi (head_pitch > 50 iken mouth <= 25) reddedildi. |
| Genişletilmiş kafa sınırları: neck -110..40, head_pitch -25..72, head_roll -20..16, mouth 0..32 | P2c | KABUL (entegre tam `check.py` temiz, §4) | Gerekli aralıklar sınırların içinde. P5b kafa sabitlerini MJCF'den yeniden türetir. |
| `cad/export.py`: `meta['collision_geoms']` uzantısı (`neck_bumper` geomu) | P2c | KORUNDU | xl330_27 bayt-eşit kaldı (altın 6/6). Dışa aktarım P3'ündür; bu uzantıyı içine alır. |
| KCD1 ana anahtar yalnız DC 12 V | P2b | P3 BOM karar verir | Stokta 16.8 V'luk anahtar yoksa ayırıcı XT30'dur. |
| `params.harness.crossings.hip_pitch.in`: z -10 → -13.5 | P2a | KABUL | Gerekli: donmuş noktayla hip_roll ±20'de tüp K_T'ye girer ve P2b'nin `trunk_base`'ine değer (P2a deneme entegrasyonunda 2 sayılan tek eklem çarpışması). Spec'le tutarlı: kablo demeti spec 3.1'de bir CAD parametresidir; spec değişmez. Uygulandı; tek eklem taraması 0. |
| Bacak sınırlarının genişletilmesi (hip_yaw -10..21, knee -84..77, ankle -65..64) | P2a | Kabul (`p2-interface.md` §1 izin verir) | Birleştirme çakışmasında alındı. |
| Kafa içi kablo güzergâhı (P2-0 giriş/çıkış yer tutucuları yerine sahibin güzergâhı) | P2c | Kabul (link içi güzergâh sahibindir) | Değişiklik yok. |
| İsteğe bağlı: rotor bölgesi kol kutusu y ±7 → ±8.6 (hip_yaw +23 için) | P2a (öneri, talep değil) | Uygulanmadı | Gerek yok; hip_yaw 21 gerekli 10'u karşılar. |

P2a'nın yeni talebi (hip_pitch giriş noktası) bir sayım istisnası, temas çifti ya da duruş/HOME değişikliği değildir.
Bu yüzden "orkestratör kararı gerekli" işareti almadı. Entegrasyonda ortaya çıkan iki yorum sorusu §8'de işaretlidir.

## 3. Entegrasyon düzeltmeleri

İlk entegre tam `check.py` (§4) sayılan her bölümde 0 verdi: P2a'nın talebi uygulandıktan sonra modüller arası
çarpışma kalmadı. Aşağıdaki değişiklikler kabul edilen taleplerin uygulanması ve parça kalitesi hatalarıdır. Hepsi
sahibi olan modülde, en küçük değişiklikle yapıldı.

### 3.1 ICR-1: idler yuvası bağı (`cad/trunk.py` `_seat_tie`, `params.trunk.struct.seat_tie`)

`neck_pitch_idler_seat` artık tek PETG baskıdır (0.75 g). Parçalar:

- Yuva diski.
- Yuva plakası düzleminde -x'e uzanan kol (y 21.4..25.4, z 34.6..39.7). Kol, yeni kutu ile donmuş yuva kutusunun
  içindedir. F623ZZ cebi ve M3 deliği kol eklendikten sonra yeniden kesilir.
- DS gövdesinin -x yanında çekirdeğe inen gövde (x 19.9..23.0; DS'ye 0.33 mm).
- Sol ön ES3054 tab'ının üstüne oturan 2 mm ayak.

Ayağı, ES3054'ün kendi ön tab vidası sıkar. Vida M3 x 6 → M3 x 8 olur; kafes plakasındaki somun aynı kalır. Bu
yüzden yeni somun yuvası gerekmedi: kutuda M2 somun bile sığmıyordu. Montaj sırası: önce U-link pimi takılır, yuva
pimle hizalanır, sonra vida sıkılır (delik boşluklu). Kabuk çekirdek yan duvarının ön ucunda gövde için bir çentik
açıldı (`shell_cuts.left` [19.7, 21.1, 18.9, 21.1, 34.4, 43.8]).

Açıklıklar: DS 0.33 mm, kabuk 0.19 mm, boyun zarfı 1.35 mm. ES3054 tab'ı ve vida başı yüz temasındadır.

### 3.2 ICR-2: batarya kapağı (`cad/trunk.py` `_door`, `params.trunk.struct.door`)

`battery_door`: 2 mm PETG plaka, x -94.2..-92.2, y -24..22, z -3..30.6, 3.16 g.

- Kaput arka duvarına ve batarya ucuna 0.2 mm boşluk bırakır, böylece darbe batarya ucuna değil çerçeveye gider.
- +y kenarı XT30 ağzının kenarındadır; XT30 ağzı açık kalır.
- Üst kenar buck penceresinin altındadır; hava girişi açık kalır.
- Batarya ağzına, batarya yanı ile ağız kenarı arasına giren 4 konum dili vardır (y ±20.3..21.8, z 2..8 ve 20..26):
  sürtünmeli geçme.

Kabul edilen kutu, ağız kenarının iki yanında yalnız 2 mm şerit bırakır. Arkada ADS1115 kartları duvara 1.2 mm'dir,
batarya ucu da K_T arka düzlemindedir. Bu yüzden kutuda vida ya da somun için yer yok. Tutma ayrıntısı (kilit
çıkıntısı / çekme dili) P3 çiziminde verilir (§8). En yakın bacak parçası P2b ölçümünde 24.95 mm uzaktaydı; entegre
taramada kapak hiçbir şeye değmedi.

### 3.3 Gövde kabukları tek katı (`cad/trunk.py` `_shell`, `chin_y0`)

`torso_shell_left` 2, `torso_shell_right` 3 ayrık katıydı. Bu bir P2b hatasıydı; ayrık parçalar baskıda gevşek plaka
olur.

- Çekirdek yan duvarı (x -14..21, y ±19.2..21, 1886 mm3) omuza yalnız bir kenarla değiyordu. Omuz duvarın altına
  uzatıldı (y ±19.2'den başlar).
- Sağ çene yüzünün y -9.7..0 şeridi (219 mm3), boyun çerçevesi çentiğiyle ayrık kalıyordu. Sol çene yüzü bu şeridi de
  kapsar (`chin_y0` -9.7); sağ kabukta o şerit yok.

Sol kabuk 15.27 → 15.59 g oldu, sağ kabuk 15.26 g.

### 3.4 Gaga pimi (`cad/head.py` `_jaw`)

`lower_bill`'in sağ pimi sağ koldan 0.01 mm ayrık başlıyordu: `-yh1 + 0.01` yazılmıştı, `-yh1 - 0.01` olmalıydı.
Sonuç iki katı olarak çıkıyordu. Düzeltildi; parça tek katı.

### 3.5 Diğer

`trunk._kt`, K_T kutularını silindirleri atlayarak adlandırır (yeni adlar `seat_tie`, `door`). Testler:

- `tests/cad/test_trunk_fc.py`: baskı parçaları tek katı, idler yuvası bağı ve kapak.
- `tests/cad/test_tr35_integration_fc.py`: kabul edilen arayüz değerleri; bütün modüllerde baskı parçaları tek katı
  ve 256 mm küpe sığar.

Entegre modelde 36 baskı parçasının hepsi tek katıdır.

## 4. Tam `check.py` sonuçları (tr35_b2, bütün modüller, entegre geometri)

Doğrulama öncesi koşu (o zamanki `check.py` modül dışı kablo temaslarını saymıyordu). Güncel sonuç: §9.6.

Rapor: `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s16/p2-integ/snap1/out/interference.txt`.
Süre 1077 s: tek eklem 230, rastgele 100, aynı link 3, komşu çift 616, bacak-bacak 89, zarflar 39.

| Bölüm | Sonuç |
|---|---|
| 1) Tek eklem (sınırlar, 7 adım, HOME) | 107 poz, **0 çarpışma** |
| 2) Rastgele 40 birleşik poz (bilgi) | 19 / 40 çarpışmasız |
| 3) Aynı link (HOME, > 1 mm3; elektronik dahil) | **0** (0 izinli) |
| 4) Komşu çiftler (5 x 5, gerekli, kapsam `module`) | 300 poz, **0 sayılan**; 26 modellenen temas; 111 modül dışı poz-çift (bilgi) |
| 5) Bacak-bacak | 32 poz, **0 sayılan**; 4 modellenen temas |
| 6) Modül zarfları (138 poz) | içerme **0**, giriş **0**, zarf-zarf **0** (kabul edilen zarf değişiklikleriyle) |
| Kablo demetleri | 34 parça, bütün taramalarda |

Modellenen temasların (`check.contact_pairs`, sayılmaz, MJCF çifti) hiçbiri kablo demeti içermez:

| Bölüm | Çift (sol ve sağ ayrı) | Poz | En çok mm3 |
|---|---|---|---|
| 4) ayak ↔ kalça beşiği | hip_cradle_bracket ↔ sole | 2 + 2 | 1813.8 |
| | hip_cradle_bracket ↔ foot_bracket | 2 + 2 | 768.1 |
| | hip_pitch_horn ↔ foot_bracket / sole | 1 + 1 / 1 + 1 | 152.9 / 112.1 |
| | hip_roll_horn ↔ sole | 1 + 1 | 52.1 |
| 4) ayak ↔ uyluk | hip_pitch_servo ↔ foot_bracket / sole | 1 + 1 / 1 + 1 | 221.4 / 62.2 |
| | thigh_bracket ↔ foot_bracket / sole | 1 + 1 / 1 + 1 | 219.7 / 78.2 |
| | hip_pitch_tab_screws ↔ foot_bracket / sole | 1 + 1 / 1 + 1 | 55.9 / 15.8 |
| 5) iki ayak | left_sole ↔ right_sole | 1 | 2956.9 |
| | left_foot_bracket ↔ right_foot_bracket | 1 | 1629.1 |
| | foot_bracket ↔ sole (çapraz) | 1 + 1 | 780.3 |

Modül dışı temaslar (bölüm 4, `pair_scope: module` ile sayılmaz; gövde/kafa ↔ uzuv, spec 3.2.2): 20 pozda 111
poz-çift. Ayrıntılı tarama: `extscan.py` / `extsum.py`, aynı scratch dizini.

| Poz (sol; sağ ayna) | Temas |
|---|---|
| hip_roll +20, hip_pitch +90 | shin/foot/ankle ↔ PCA9685, kabuk, buck (en çok 1498 mm3) + kablo demetleri (yapı ve kablo) |
| hip_roll +10, hip_pitch +90 | shin_bracket ↔ kabuk 122 mm3 + kablo demetleri |
| hip_roll +20, hip_pitch +10 / +50 | yalnız kablo: hip_pitch_harness_out ↔ PCA9685#1 224 mm3, kabuk 173; knee_harness_in ↔ PCA 121 |
| hip_roll +10, hip_pitch +10 / +50 | yalnız kablo: hip_pitch_harness_out ↔ kabuk 22 / 277 mm3 (pitch 10'da ilk temas roll +9) |
| hip_yaw -10, hip_roll 0 / 10 / 20 | yalnız kablo: hip_pitch_harness_out ↔ belly_duct ön-alt dış kenarı 3.1 / 12 / 22 mm3 |
| neck +40, head_pitch -20 | head_hood ↔ buck_20a 1300 mm3, buck_hood 322 (kafa ↔ trunk çifti, v3.1) |
| neck -90, head_pitch +60 | upper_bill ↔ hip_cradle_bracket 3.1 mm3 (iki taraf); 3.2.2 listesinde yok (§8) |

111 poz-çiftin 73'ünde bacak kablo demeti tüpü var. 14 pozda gövdeye yalnız kablo değiyor, bacak yapısı değmiyor.

## 5. Boy, kütle, COM

Doğrulama öncesi değerler; düzeltmelerden sonra kütle 1768.2 g (x 1.05 = 1856.6 g), kafa üstü ve COM aynı (§9.6).

- **Kafa üstü (stand keyframe)**: CAD 347.7 mm (hedef 350 ± 10). MJCF `head_collision` kutusunun üstü 348.7 mm.
  Sıfır pozda 338.2 mm. Stand'da boyut 190.6 x 223.4 x 347.7 mm; trunk kökü yerden 152.5 mm.
- **Baskı parçaları**: 36 parça, hepsi 256 mm küpe sığar ve tek katıdır. En büyükleri: trunk_base 138.0,
  lower_bill 123.7, head_hood 117.7 mm.
- **Gerekli aralıklar ⊆ sınırlar**: 10 eklemin hepsi.
- **Kütle (aşağıdan yukarı, `print.density_eff`)**: 1769.0 g x 1.05 = **1857.5 g** (sınır 1950 g, pay 92.5 g). Hedef
  1.85 kg (aşağıdan yukarı 1762 g) 7 g aşılıyor.

| Modül | Sahip modül (g) | Link kuralı (g, P2-0 §8) | Bütçe (g) | Kalemler (sahip modül, g) |
|---|---|---|---|---|
| Gövde | 653.9 | 657.5 | 733.6 | batarya 200, buck/XL4015/hub kartları 106.0, servolar 105 (2 x ES3054 + boyun DS), PETG 66.7, metal (rulman, vida) 43.6, PLA 42.9, kartlar 40.0, kablo 25.0, küçük elektronik 24.0, TPU 0.7 |
| Bacaklar | 914.9 | 912.1 | 913.2 | servolar 520, PETG 218.0, metal 68.9, horn 42.0, TPU taban 36.9, kablo 29.2 |
| Boyun + kafa | 200.2 | 199.4 | 202.0 | PLA 82.6, servolar 49.2, PETG 49.1, horn 7.0, metal 6.0, kablo 5.1, TPU 1.2 |
| **Toplam** | **1769.0** | **1769.0** | 1848.8 | |

"Link kuralı" P2-0'daki gibidir: trunk link'indeki `hip_yaw_harness_in` tüpleri (2 x 1.39 g) ile
`neck_pitch_harness_in` (0.79 g) gövdeye sayılır. Bacaklar bu kuralla bütçenin içindedir.

- **HOME COM** (trunk çerçevesi): (-10.8, 0.3, -10.0) mm, yerden 142.6 mm.
  - HOME'da ayaklar x -26.4..44.3 (merkez 9.0); COM, ayak merkezinin **19.8 mm gerisinde**. Hedef x ∈ [-8, 12]
    2.8 mm kaçırıldı (P2-0 yer tutucularıyla -10.9, P2b yer tutucularıyla -10.2). |y| 0.3 ≤ 2.
  - Modül COM'ları: gövde (-26.5, 0.4, 23.7), bacaklar (-4.4, 0.0, -65.5), kafa (11.8, 1.0, 133.2).
  - MuJoCo keyframe'inde dünya COM'u (-10.8, 0.3, 143.6) mm.

## 6. MJCF (MuJoCo, CPU venv)

`snap1/build/sim/scene.xml` (`tools/build_gui.py`, `OPENDUCK_DOCS=0`; FCStd, STEP, 36 STL, URDF, MJCF hatasız):

- 15 aktüatör, RL sözleşmesi sırasıyla (sol bacak, sağ bacak, neck_pitch, head_pitch, head_yaw, head_roll, mouth).
  4 sınıf: es3054 (2), ds32xx (9), mg90s (3), sg90 (1). ctrlrange = eklem sınırı.
- freejoint `root`, tek keyframe `stand`, siteler `imu`, `left_foot_sole`, `right_foot_sole`. Collision geomları:
  `trunk_collision` (kapakla birlikte x -94.2..46), `head_collision`, `left/right_foot_collision`.
- `neck_bumper` geomu `neck` gövdesinde: collision sınıfı, contype 0 / conaffinity 1, yarı boyutlar
  (3.65, 12, 4.5) mm. `neck_collision` yok.
- Toplam kütle 1.769 kg.
- Yerçekiminde 2 s, ctrl = stand: gövde eğimi **0.33°** (pitch -0.33, roll 0.00; en çok 0.37). Trunk z 153.5 → 152.1
  mm, xy kayması 0.7 mm, eklem izleme hatası en çok 0.12°. Temaslar yalnız iki ayak ↔ zemin.

## 7. Testler

| Test | Sonuç |
|---|---|
| `pytest tests/cad` | 122 geçti (yeni: `test_tr35_integration_fc.py` 2, `test_trunk_fc.py` +2) |
| `pytest tests/golden` (check.py ile tam koşu) | 6 / 6, 71 s; xl330_27 bayt-eşit |
| RL CPU paketi (kilit altında) | 119 geçti / 5 atlandı (taban ile aynı) |
| RL MJX paketi (kilit altında, `JAX_PLATFORMS=cpu`) | 135 geçti / 2 atlandı (taban ile aynı) |

## 8. Açık konular

### P3 (dışa aktarma, BOM, baskı seti)

- MJCF öz-çarpışma çiftleri:
  - Spec §3.2.2 v3.2 listesi, yeni **çene ↔ boyun/trunk** dahil. P2c ölçümü: head_pitch ≥ ~55 ve mouth ≥ ~28'de alt
    gaga neck link'ine değer (60/30'da en çok 124 mm3, mouth 32'de 317 mm3). head_pitch 72'de boyun servosuna (trunk)
    da değer, 537 mm3'e kadar.
  - Ayrıca `check.contact_pairs` (iki ayak; ayak ↔ aynı bacağın uyluk/kalça beşiği).
- Uyluk/incik collision geomları bacak kablo demeti tüplerini (`hip_pitch_harness_out` y 53 katmanı,
  `knee_harness_in/out`) kapsamalı. Kapsamazsa §9.1'de kalan kablo temaslarını (roll ±20 × pitch ±50 yalnız kablo;
  roll ±10/±20 × pitch ±90) MJCF çiftleri görmez.
- `export.py`'deki `meta['collision_geoms']` uzantısı P3'ün dışa aktarımına katılır.
- KCD1 yalnız DC 12 V: BOM'da 16.8 V'luk anahtar yoksa ayırıcı XT30 olur.
- Batarya kapağı tutma ayrıntısı (§3.2): konum dillerinde kilit çıkıntısı, çekme dili ve Adım 0'da geçme kontrolü.
  Kutuda vida için yer yok.
- İdler yuvası bağının montaj sırası (§3.1), baskı notları (`trunk.PRINT_NOTES`, `legs.PRINT_NOTES` /
  `print_report`). `docs.py`'de `harness` türünün baskı satırı yok.
- `p2b-hacim.md` tabloları entegrasyondan önceki durumdur. Kapak (+3.16 g), bağ (+0.45 g) ve birleşen kabuklar
  eklenmeli; `trunk.volume_report(m)` ile yeniden üretilir. Robot satırı artık gerçek modüllerle hesaplanır.
- Devralınanlar:
  - P2b tahmin boyutları: alarm, INA226, MP1584, hub, bölücü, kırpıcı, mandal, sigorta, XT30.
  - PCA9685/DevKitC başlık sökümü.
  - Küçük kart ve kabuk bağlantıları.
  - P2c gaga pim yatağının bölünmüş kapağı (2 x M2).
  - 6800ZZ stoğu.

### G-CAD

- A1 özeti (doğrulama düzeltmelerinden sonra, §9.6):
  - tek eklem 0, aynı link 0, bacak-bacak 0, zarflar 0/0/0;
  - komşu çift ızgarası: modül içi 0; **modül dışı kablo demeti 35 poz-çift sayılır** (aşağıdaki karar);
  - gerekli ⊆ sınırlar;
  - baskı ≤ 256 mm;
  - kafa üstü 347.7;
  - kütle 1856.6 ≤ 1950 g;
  - el hesapları: `p2a-el-hesabi.md`, `p2c-tampon.md` (dört DS bacak eklemi, ES3054 ve boyun eksenel yük yolu §9.2-9.4);
  - altın bayt-eşit.
- **ORCHESTRATOR DECISION NEEDED (kalan kablo temasları, §9.1):** `check.py` artık spec A1'in harfiyle sayar. P2a
  güzergâhı 14 yalnız-kablo pozun 12'sini temizledi; donmuş arayüzle temizlenemeyen 6 poz kaldı (35 poz-çift):
  - roll ±20 × pitch ±50, yalnız kablo (2 poz, 10 poz-çift): donmuş diz giriş noktası ve diz geçişi trunk'ın alt dış
    arka kenarına girer.
  - roll ±10 / ±20 × pitch ±90, uzvun yapısı da gövdede (4 poz, 25 poz-çift).
  - Seçenekler: (a) spec A1'e "uzvun kendi yapısının da gövde/kafaya girdiği pozlarda kablo teması o MJCF gövde ↔ uzuv
    çiftiyle birlikte listelenir" eklemek (P3 collision geomları kabloyu kapsar); kalan 2 yalnız-kablo poz için (b).
    (b) P2b: alt dış arka kenarda x −58..−28 boyunca ~4 x 8 mm kırpma + PCA9685'i ~2 mm yukarı/içeri almak, ya da diz
    giriş noktası için ICR + x −58..−40'ta daha küçük kırpma. (c) A1'in harfi tüm pozlarda: hip_pitch ve diz
    geçişlerini bacağın dış (idler) tarafına almak (arayüz değişikliği, P2a yeniden tasarımı).
- **ORCHESTRATOR DECISION NEEDED (yeni çift):** neck -90 + head_pitch 60'ta `upper_bill` ↔ `hip_cradle_bracket`
  (kafa ↔ kalça beşiği) 3.1 mm3, iki taraf. §3.2.2 listesinde kafa ↔ ayak ve kafa ↔ uyluk/incik var, kafa ↔ kalça
  beşiği yok.
  - Seçenekler: listeye eklemek (spec değişikliği) ya da P2c'nin üst gaga altını ~1 mm kırpması.
  - Şimdiki çözüm: bilgi olarak bırakıldı; sayılmıyor.
- HOME COM ayak merkezinin 19.7 mm gerisinde (hedef 7 ± 10 mm geride). Karar 4 gereği P5 HOME'u yeniden türetir.
- Kütle hedefi 1.85 kg 6 g aşılıyor (sert sınır tutuyor). Kütle merdiveni isteğe bağlıdır.

### P5b (RL)

- tr35 sabitleri MJCF'den yeniden türetilir:
  - genişletilmiş sınırlar: bacak hip_yaw -10..21, knee -84..77, ankle -65..64; kafa neck -110..40, head_pitch
    -25..72, head_roll -20..16, mouth 0..32;
  - HOME COM;
  - kafa sabitleri;
  - `trunk_collision` kutusu (kapak nedeniyle x -94.2'ye uzadı);
  - `neck_bumper`.
- RL çift listesine **çene ↔ boyun** ve **çene ↔ trunk** girer. Kilit reddedildiği için politika teması simde öğrenir.
  Ayrıca v3.1 çiftleri: kafa ↔ trunk; ayak ↔ aynı bacağın uyluk/kalça beşiği.
- Bacak kablo demetlerinin gövdeye değdiği birleşik pozlar (§9.1: roll ±20 × pitch ±50, roll ±10/±20 × pitch ±90),
  P3'ün kabloyu kapsayan collision geomlarıyla simde görünür olmalı.

## 9. Dogrulama duzeltmeleri

Bağımsız doğrulama (tur 1) dört engelleyici bulgu verdi. Düzeltmeler dal `worktree-wf_6b702ebd-130-6`'dadır
(`worktree-wf_6b702ebd-130-4` üstüne hızlı ileri). Her biri sahibi olan modülde, en küçük değişiklikle yapıldı.
Ölçüm betikleri: `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s14/p2-dogrulama/`
(`p6.py` eksenel boşluklar, `route_eval.py` + `cands.json` kablo güzergâhı adayları, `p7.py` temas yerleri).

### 9.1 Kablo demeti temasları komşu çift ızgarasında da sayılır

**Bulgu.** `check._scan`, `pair_scope: module` ile modül dışı temasları `contact_count_kinds` uygulanmadan bilgi
listesine taşıyordu. Spec A1 "Kablo demeti temasları her zaman sayılır" der. Izgarada 18 pozda 73 poz-çift kablo teması
sayılmıyordu; 14 pozda gövdeye yalnız kablo değiyordu, yani MJCF bu temasları hiç görmez.

**`cad/check.py`:**

- Modül dışı temaslardan kablo demeti parçası olanlar (`check.count_kinds`, varsayılan `harness`) sayılır. Gövde/kafa
  ↔ uzuv yapı temasları eskisi gibi bilgi amaçlıdır (spec 3.2.2 MJCF çiftleri).
- Sayılan her modül dışı kablo satırı işaretlenir: "yalnız kablo" (pozda modül dışı yapı teması yok) ya da "yapı
  temaslı poz" (uzvun kendi yapısı da gövdeye giriyor). Bölüm 4 özetinde iki sayı ayrı verilir.
- Test: `tests/cad/test_check_contacts.py::test_pair_scope_still_counts_harness`. xl330_27 raporu değişmez (bölüm 4
  yok; altın 6/6).

**P2a güzergâhı** (`params.legs.harness_route.hip_pitch.out` ve `knee.in`; yalnız konfig, donmuş geçiş/giriş/çıkış
noktaları aynı):

- Neden: uyluk kablosu y 44.5 bandında (iki servonun horn iç yüzü), pitch ekseninden r 15..40 arasında yer bulamaz.
  roll −20 / pitch −70'te hip roll servosu (y 16..36), roll +20 / pitch 10..50'de trunk'ın alt yan kenarı engeldir.
  Eski yay yukarıdan dolaşarak (r 42) birincisini çözüyor, ikincisinde gövdeye giriyordu.
- Yeni güzergâh: donmuş çıkıştan (−5, 44.5, −30) sonra pitch ekseninden r 15'te y 46'ya, r 19'da y 53'e çıkar. y 53,
  horn dış yüzü (51) ile servo üst yüzü (58.5) arasındaki boş katmandır; hip_cradle pitch göbeği (r 11) dışındadır. Kablo
  y 53'te diz yönüne gider ve dizin üstünde (diz ekseninden r 16.5, 63°) y 44.5'e iner.
  - İniş noktası, incik A kolunun diz +77'deki süpürmesinin ~8° dışındadır.
  - İniş noktası pitch ekseninden r 40.7'dedir. hip_cradle'daki pitch kablosu pitch ile r ≤ 37.4'ü süpürür.
- Aynı geçişin iki tüpü (izinli çift) geçiş noktasından > 8 mm uzakta 12 pitch pozunda kesişiyordu (36 mm'ye kadar).
  Şimdi 3 pozda, en çok 9.3 mm uzakta kesişir.
- Kütle: uyluk kablosu bacak başına 3.35 → 2.29 g.

Komşu çift ızgarası (5 x 5, required), modül dışı kablo temasları (sol; sağ ayna):

| Poz | Önce (130-4, sayılmıyordu) | Sonra (sayılır) |
|---|---|---|
| hip_yaw −10 × roll 0 / 10 / 20 | belly_duct 3.1 / 12 / 22 mm3 (yalnız kablo) | 0 |
| roll 10 × pitch 10 / 50 | kabuk 22 / 277 (+ diz girişi 10) (yalnız kablo) | 0 |
| roll 20 × pitch 10 | PCA9685 224, kabuk 173, taban 24, güç kablosu 3 (yalnız kablo) | 0 |
| roll 20 × pitch 50 | 10 poz-çift, en çok PCA9685 170 (yalnız kablo) | 5 poz-çift: kabuk 93, taban 35 + 21, kabuk 23, PCA9685 9 (yalnız kablo) |
| roll 10 × pitch 90 (yapı: incik A kolu ↔ kabuk) | 6 poz-çift, en çok kabuk 230 | 5 poz-çift (sağda 4), en çok kabuk 143 |
| roll 20 × pitch 90 (yapı: diz servosu, incik, ayak gövdede) | 10 poz-çift, en çok kabuk 213 | 8 poz-çift, en çok PCA9685 130 |
| **Toplam (iki taraf)** | **73 poz-çift, 18 poz (14 yalnız kablo)** | **35 poz-çift, 6 poz: 10 yalnız kablo (2 poz), 25 yapı temaslı (4 poz)** |

9 x 9 ızgarada (bilgi; doğrulama bu ızgarada 203 kablo poz-çifti saymıştı) sol uyluk kablosunun kalan temasları
yalnız roll ≥ 5 ve pitch ≥ 50 köşesindedir: 10 pozda 28 poz-çift (incik kablosu bu taramada yok). Modül içi temas yok.

**Kalan temaslar (ORCHESTRATOR DECISION NEEDED, §8 G-CAD):**

- roll ±20 × pitch ±50 (yalnız kablo): donmuş diz giriş noktası (−30, 44.5, −42) ve diz geçişi (−40.768, 44.5,
  −50.639) bu pozda trunk'ın alt dış arka kenarına girer (x −58..−28, y ≥ 34.3, z ≤ 6). Orada kabuk alt yan duvarı
  (y 36.2..38), taban plakasının kenarı ve PCA9685'in alt köşesi vardır. Bağlantı içi güzergâh bunu çözemez: geçiş
  noktasının kendisi gövdenin içindedir.
- roll ±10 / ±20 × pitch ±90 (yapı temaslı): uzvun yapısı gövdeye girer (roll 10'da incik A kolu kabuğa 122 mm3;
  roll 20'de diz servosu 287, incik 1498 mm3). Donmuş diz geçişi de bu pozlarda gövde parçalarının içindedir. Geçişi iç
  tarafta (y 44.5) olan hiçbir güzergâh burada temassız olamaz.

### 9.2 hip_yaw kaplini: kolun altında eksenel boşluk (`cad/legs.py` `_coupler`)

**Bulgu.** Yarık kesicisi horn yüzünde (kanonik Y 0) bitiyordu. Çatal tabanı (0.51 mm, 43 mm3) ES3054 kolunun altında
sıfır boşlukla kalıyordu; rulman eksenel boşluğu ve düşmede mil eğikliği ES3054 çıkışını eksenel itebilirdi.

**Düzeltme.** Yarık halka üstünün 0.02 mm altına kadar (`top_gap` + 0.02) kesilir; kolun altında çatal tabanı kalmaz.

- Horn 0.30 / 0.49 / 0.51 mm aşağı kaydırılınca kaplinle kesişim 0; 0.60 mm'de halkaya 3.7 mm3.
- ±1.4° dönmede temas yok, 2°'de tork teması (0.3 mm yan boşluk).
- Kaplin 0.49 g, tek katı. `p2a-el-hesabi.md` §2 güncellendi; test `test_legs_tr35_fc.py::test_axial_load_paths`.

### 9.3 Diz baskı kovanı (`cad/legs.py` `_shin`)

**Bulgu.** İncik `thigh_keep` kesicisi (diz ekseni etrafında r 23, y 57.5..104.5), `servos.ds_thrust_sleeve`'in servoya
bakan son 0.8 mm'sini kesiyordu: diz servosu üst yüzüne 1.0 mm (tasarım 0.2).

**Düzeltme.** Kesici kovanı dışarıda bırakır. Kovan eksenel simetriktir; uylukta r ≤ 10.7, y 57.5..58.3 bölgesinde
parça yoktur.

- Dört DS ekleminde (hip_roll, hip_pitch, knee, ankle) kovan ↔ servo üst yüzü 0.200 mm; kovanın yakın yüzü tam
  (74.2 / 74.2 mm3).
- `p2a-el-hesabi.md` §5'teki "en çok 0.2 + 0.05 mm" artık dizde de doğrudur (not eklendi). Test:
  `test_axial_load_paths`.

### 9.4 Boyun eksenel yük yolu (`cad/head.py` `_neck`, `params.head.neck.thrust_r` / `thrust_y1`)

**Bulgu.** Boyun B kolunda P2-0 yer tutucu pimi duruyordu: göbek + baş; F623ZZ'den geçen M3 ve ara halka yoktu. Baskı
kovanı da yoktu (neck_link ↔ servo 2.9 mm). Yanal kafa yükü DS spline'ına eksenel gidiyordu; `p2c-tampon.md` boynu
yalnız "tork + radyal" diye geçiyordu.

**Düzeltme:**

- B kolu: bacaklarla aynı `servos.ds_idler_screw`. M3 imbus F623ZZ iç bileziğinden geçer; somun B kolunun rulmana bakan
  altıgen yuvasındadır; pirinç ara halka iç bileziğe 0.05 mm. B kolu deliği r 5.02'den M3 boşluk deliği + somun yuvasına
  döndü. −y yükü: somun → ara halka → iç bilezik → yuva dudağı (trunk).
- A kolu: baskı kovanı r 8.2..11.5, horn yüzünden y −21.2'ye (donmuş boyun zarfı −21.1 içinde). Servo üst yüzü
  (y −19.4) zarfın dışındadır. Bu yüzden kovan, trunk_base boyun beşiği yan duvarlarının uç yüzlerine dayanır (y −21.0 =
  K_T çekirdek sınırı, |dx| ≥ 10.15), 0.2 mm boşlukla, ~20 mm². +y yükü trunk_base'e gider.
- Arayüz değişikliği (ICR-3, boyun idler mili yolu): mil F623ZZ'nin içinden, ara halka da rulman ile B kolu arasındaki
  0.1 mm'lik zarf aralığından geçer. Boyun zarfına iki silindir eklendi, K_T'den aynıları çıkarıldı (`["cyl", "y",
  33.332, 41.537, 0, 1.5, 22.5, 26.45]` rulman deliği, `[..., 0, 2.6, 26.45, 26.75]` ara halka). Bu bölgede trunk
  parçası yok; trunk geometrisi değişmedi. Test: `test_tr35_integration_fc.py::test_accepted_interface_changes`.
- Ölçüm: boyun zinciri y boyunca kaydırıldı. +0.25 mm'de trunk_base'e, −0.10 mm'de rulmana dayanır. Servoya 1.8,
  kabuğa 0.86, torso_top'a 2.7 mm.
- El hesabı (`p2c-tampon.md` §4 boyun satırı): 10 g yanal yükte (19.7 N) kovanda 1.0 MPa, yuva dudağında 0.4 MPa;
  F623ZZ C0r 180 N.
- Kütle: neck_link 20.1 → 21.2 g. Pim bütçe değeri 1.8 g kaldı. Kafa modülü 201.3 g (bütçe 202.0).
- Test: `test_head_tr35_fc.py::test_passive_supports` (kovan 0.2, idler 0.05).

### 9.5 Bu bulgular dışında

- `check.py` raporunda bölüm 4'e sayılan modül dışı kablo satırları ve özeti eklendi (§9.1). Bölüm 6'nın davranışı
  değişmedi.
- Adım 0'a: DS çıkış mili eksenel boşluğu > 0.2 mm olmalı. Bu, bacaklardaki koşulun boyuna da uygulanmasıdır.

### 9.6 Güncel sonuçlar (tr35_b2, entegre, düzeltmelerden sonra)

Rapor: `/private/tmp/claude-501/-Users-huseyinarslan-Desktop-ads-z-klas-r-2/8217cd75-2c54-4fa4-b47f-57c38ee96c2b/scratchpad/s14/p2-dogrulama/full2/out/interference.txt`.

Süre 1076 s: tek eklem 227, rastgele 106, aynı link 3, komşu çift 627, bacak-bacak 74, zarflar 39.

| Bölüm | Önce (§4) | Sonra |
|---|---|---|
| 1) Tek eklem (sınırlar, 7 adım, HOME) | 107 poz, 0 | 107 poz, **0** |
| 2) Rastgele 40 birleşik poz (bilgi) | 19 / 40 çarpışmasız | 26 / 40 çarpışmasız |
| 3) Aynı link (HOME, > 1 mm3; elektronik dahil) | 0 | **0** |
| 4) Komşu çiftler (5 x 5, gerekli, kapsam `module`) | 0 sayılan (kablo sayılmıyordu) | modül içi **0**; modül dışı kablo **35 poz-çift sayılır** (10 yalnız kablo, 25 yapı temaslı pozda; §9.1) |
| 4) modellenen temaslar / modül dışı yapı (bilgi) | 26 / 111 poz-çift (73'ü kablo) | 26 / 38 poz-çift (hiçbiri kablo değil) |
| 5) Bacak-bacak | 32 poz, 0 sayılan; 4 modellenen | 32 poz, **0** sayılan; 4 modellenen |
| 6) Modül zarfları (138 poz) | 0 / 0 / 0 | içerme **0**, giriş **0**, zarf-zarf **0** (ICR-3 ile; öncesinde boyun mili 30.9 mm3 dışarıda) |

- Kütle (aşağıdan yukarı): 1768.2 g x 1.05 = **1856.6 g** (sınır 1950). Gövde 653.9, bacaklar 912.9 (bütçe 913.2),
  kafa 201.3 (bütçe 202.0).
- Kafa üstü (stand) 347.7 mm. HOME COM (−10.8, 0.2, −9.9), ayak merkezinin 19.7 mm gerisinde. 36 baskı parçası tek
  katı ve 256 mm küpe sığar. Gerekli ⊆ sınırlar (10 eklem).
- MJCF: 15 aktüatör, 4 sınıf, 1.768 kg. 2 s ayakta eğim 0.33°; temaslar yalnız ayak ↔ zemin.

| Test | Sonuç |
|---|---|
| `pytest tests/cad` | 124 geçti (yeni: `test_pair_scope_still_counts_harness`, `test_axial_load_paths`) |
| `pytest tests/golden` (check.py ile tam koşu) | 6 / 6, xl330_27 bayt-eşit |
