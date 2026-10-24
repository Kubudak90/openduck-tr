# Spec eki K: Sesli komut katmanı (OpenDuck-TR, ek K v3)

Ana belge `spec.md`'dir. Bu ek ana belgeyi yalnız açıkça yazdığı yerlerde değiştirir:
- K3: §2 istisnası ve §4 kütle dağılımı;
- K10: §4 bütçe tavanı.

Başka bir çelişki çıkarsa ana spec geçerlidir ve ek güncellenir.

Kaynaklar:
- Kullanıcı kararları (2026-10-24): tanıma robot üstünde, DF2301Q ile; "gel" düz ileri yürür; 12 komut; bu tur yalnız
  simülasyon; bütçe tavanı yükseltildi.
- Araştırma: `research/voice-research.md`.
- İnceleme: `review-komut.md`. Tur 1'de 43 itiraz geldi (2 kritik), tur 2'de 27 (kritik yok); v3 ikisini de işler.

## K1. Problem ve hedef

Kullanıcı Türkçe kısa komutlarla robotu yönetir: "ördek" → LED → "gel". Politikalar zaten komutla çalışıyor:
- main, `[vx, vy, wz, h, sit]` komutunu alır.
- Kafa hedefleri betikle sürülür.
- getup ayrı bir politikadır.

Bu katman, kelime kimliğini bu girdilere çeviren denetleyiciyi tanımlar (`rl/commands.py`). Denetleyici simülasyonda
test edilir. Eğitim, katmanın ürettiği komut biçimlerini ve jestleri kapsayacak biçimde genişletilir. Katman yalnız main
ve getup'ı kullanır; walk değişmez.

## K2. Kapsam dışı

- Firmware. Bu tur Python referansı ve adım adım altın izler yazılır; firmware ikisini birebir kopyalar.
- Konuşmacıyı bulma (ses yönü, kamera, takip). "Gel", robotun baktığı yöne yürür.
- Serbest konuşma ve robotun konuşması. Hoparlör normal çalışmada susturulur; yalnız öğrenme modunda açılır.
- Tanıma modeli. Modül yalnız kimlik (ID) verir.
- G-SES-b. Robot kurulunca yapılır. İçeriği:
  - son yerleşimde ve kabuk içinde şablonlar yeniden öğretilir;
  - K8 protokolü robotun kendi servolarıyla tekrarlanır;
  - pil süresi kadar (20–30 dk) bir koşu yapılır, sonunda 12 kelime ve canlılık yeniden kontrol edilir.

## K3. Donanım ve yerleşim

### Modül

- DFRobot DF2301Q (Gravity SEN0539), tercihen V1.1.
- Kart 50 × 32 mm. İki mikrofon sol köşelerde, aralarında yaklaşık 25 mm.
- Besleme 3.3–5 V, en çok 370 mA.
- Bilinmeyenler: kütle, yükseklik, delikler, revizyon, konnektör yeri. Modül Adım 0 alımıyla şimdi alınır ve
  G-SES-a hazırlığında ölçülür (K8).
- Ölçü gelene kadar muhafazakâr zarf: 52 × 34 × 14 mm, 14 g, artı fiş ve büküm için 10 × 12 × 10 mm'lik bir yer ayırma
  kutusu. Fiş yerine teller pedlere doğrudan lehimlenirse beşikte gerilme tutucusu olur.

### Ana spec değişikliği (§2)

"Kafa elektroniği kapsam dışı" maddesine bir istisna eklenir: DF2301Q ses modülü ve onun gösterge LED'i. Kullanıcı
kararı, 2026-10-24.

### Yerleşim: yalnız kafa kubbesi

Trunk seçeneği yoktur. Donmuş K_T'de bu kart için yer yok (P2b voksel taraması), dışarısı da kafanın süpürme alanında.

1. **Kütle dağılımı (ana spec §4):**
   - Trunk bütçesinden kafaya 30 g aktarılır. Kafa 202 → 232 g, trunk 733.6 → 703.6 g olur; trunk ölçülen 654 g.
   - Modül, LED, beşik, contalar, pasifler, köpük ve teller aşağıdan yukarı kütlede ayrı satırlardır.
   - G-CAD (×1.05 ≤ 1950 g) bunlarla birlikte sağlanır. Aşılırsa önce kütle merdiveni uygulanır.
2. **Yük:** MG90S statik momenti, `p2c-tampon.md` yöntemiyle, ≤ 0.5·stall kalır. Tampon tasarım durumu yeniden
   hesaplanır.
3. **Akustik.** En yakın kaynaklar kafanın kendi servolarıdır: head_yaw MG90S yaklaşık 35 mm, SG90 yaklaşık 36 mm,
   head_roll yaklaşık 45 mm.
   - Her mikrofon portu kendi kabuk deliğine (≥ 3 mm) yumuşak bir contayla, %20–30 sıkıştırmayla, sızdırmaz bağlanır.
     Conta kapalı hücreli köpük (PORON tipi) ya da Shore ≤ 40A silikon halkadır; gerekirse ağzına akustik file konur.
     İki yol eş uzunluktadır.
   - Kart, f_n ≤ 100 Hz hedefiyle TPU 95A yaylı beşiğe ya da silikon grometlere asılır. Conta bu rijitliğe katkısıyla
     birlikte raporlanır.
   - Köpük kartın çevresine konur, port deliklerinin üstüne konmaz.
   - Mikrofon ile en yakın servo arasındaki mesafe CAD raporuna yazılır.
4. **Montaj.** Kart kenarını tutan bir beşik kullanılır: iki revizyonda ortak delik çifti ve kenar klipsi. Delikler
   ±2 mm yarıktır.
5. **Gösterge.** Kafaya, ESP32'nin sürdüğü 3 mm'lik bir LED ve direnç konur (yaklaşık 1 g). Kabukta bunun için bir
   pencere açılır. Bütün görsel geri bildirim bu LED'dir: uyanma, red ve pencere bitişi (K5). Modülün kendi LED'i ayrı
   bir pencerede ikincil göstergedir.
6. **Kablo demeti (ICR; P2d'nin değiştirme yetkisi var).**
   - Kafaya 5 iletken gider: VCC, GND, TX, RX ve LED. Her biri 30 AWG silikon kaplı teldir. TX/RX, GND ile bükülü çift
     yapılır ve servo güç tellerinden ayrı yerleşir.
   - Dört geçişin yeni çapı gerçek iletken listesinden hesaplanır: neck, head_pitch, head_yaw ve head_roll. Komşu
     parçaya ≥ 0.4 mm açıklık kalır; gerekirse geçiş noktası kaydırılır (ör. neck y −34.4 → −34.9). bend_r = 4·d.
   - P2d'nin değiştirebileceği dosyalar: P2b'nin neck_trunk/power/signal_wiring kanalları, sağ gövde kabuğu çentiği ve
     P2c'nin harness_route'u. Bitti ölçütü: tam check.py, tests/cad ve golden geçer; head_yaw ±80° burulma taraması
     temiz.
7. **Hoparlör.**
   - Hoparlör kartta kalır, SPK anahtarı SPK1'de sabittir.
   - Susturma yalnız yazılımla yapılır: firmware her açılışta `setMuteMode(1)` gönderir. Ayarın kalıcılığı G-SES-a'da
     ölçülür.
   - Öğrenme modu: BOOT tuşuna uzun basılınca açılır. `setMuteMode(0)` gönderir, 200–208 ve yeni kimlikleri loglar,
     kimlik tablosunu günceller, çıkışta modülü yeniden susturur.
   - İstemin kafa içinden duyulup duyulmadığı G-SES-a'da denenir. Duyulmuyorsa kabuğa küçük bir hoparlör açıklığı
     konur.

### Bağlantı

- **Arayüz:** UART1, 9600 8N1. I2C kullanılmaz; SDA/SCL'yi kilitlediğine dair kullanıcı raporu var.
- **Pinler:** robot pin haritasında P6 ekiyle seçilir; tezgâh aynı pinleri kullanır. Kayış pinleri (0, 3, 45, 46) ve
  43/44 yasaktır. RX'e 1 kΩ seri direnç konur. LED için bir GPIO ayrılır.
- **Besleme:** 5 V raydan ayrı bir dal; yerelde 47 µF ve 100 nF.
- **Seviye:** G-SES-a'da ölçülen TX boşta gerilimine göre seçilir. ≤ 3.6 V ise yalnız 1 kΩ seri direnç, > 3.6 V ise
  bölücü. Modülün RX'inin 3.3 V sürüşle çalıştığı doğrulanır.

### BOM (P3 mağaza, fiyat, stok ve tarihle yazar)

| Kalem | Not |
|---|---|
| DF2301Q | SAMM 1.171,86 TL (stok 6, 2026-10-24); yedek kaynak Direnc 1.817,05 TL |
| Kargo | |
| 30 AWG silikon tel | Kynar wire-wrap uygun değil |
| Pasifler | 47 µF, 100 nF, 1 kΩ ×2, gerekirse bölücü |
| LED ve direnç | |
| Conta, köpük, gromet malzemesi | |
| Piezo ve fototransistör | yalnız tezgâh (G-SES-a) |

Ses katmanının tam farkı modül, kargo ve yan kalemlerin toplamıdır. K10 tavanına sığmalıdır.

### Yedek yol

G-SES-a tutmazsa sırasıyla:
1. şablonlar yeniden öğretilir; zayıf kelimelere boş kimliklerde ikinci şablon verilir;
2. conta ve köpük düzeltilir;
3. hâlâ tutmazsa kullanıcı karar verir: Elechouse V3 tipi modül ya da telefon uygulaması.

Yer, kütle ve COM G-CAD'de donar; sonradan yer değişmez.

## K4. Kelime dağarcığı ve kimlik tablosu

**Kimlik tablosu.** Kimlikler öğrenme sırasıyla atanır, bu yüzden kod sabiti değildir. Tablo
`configs/komut_kimlik.json` dosyasındadır, çoktan bire eşler ve öğrenme adımında yazılır. commands.py ve firmware bu
tabloyu okur.
- Beyaz liste: yalnız tablodaki kimlikler komut üretir.
- 1 ve 2 uyanmadır.
- Tabloda olmayan kimlikler yok sayılır ve loglanır.
- 200–208 gelirse uyarı verilir (öğrenme modu).
- 17–21 numaralı boş kimlikler ikinci şablonlar içindir.

**Uyanma.** Uyanma kelimesi "ördek"tir. Kullanıcı protokolü: ördek → LED yanar → komut. `setWakeTime` 15 s'dir.
- Denetleyici pencereye göre filtre yapmaz. Modül hangi beyaz liste kimliğini gönderirse kabul edilir. "dur" asla
  düşürülmez.
- 15 s'lik, yenilenmeyen pencere yalnız A10 düzeneğindeki modül modelinde bulunur (K7).
- LED, uyanma kimliğinden 15 s sonra söner. NOTIFY çerçeveleri G-SES-a'da doğrulanırsa LED onlara bağlanır.

**Sabitler.** vx_max, vx_min ve wz_max tr35 COMMAND_RANGES'tan türetilir; κ = 0.5'te vx −0.085..0.17 m/s, wz
±0.44 rad/s. ΔH = (H_HOME − H_MIN_WALK)/2. h_ref, h_cmd'yi H_RATE hızıyla izler. Bütün sayılar commands.py'de
türetilir.

| Kelime | Uygulama (main komutu) | Bitiş |
|---|---|---|
| gel | vx = 0.8·vx_max | dur, yeni hareket komutu ya da 12 s |
| dur | vx = vy = wz = 0; jest yumuşak sönümlenir | hemen |
| geri | vx = 0.8·vx_min | 0.5 m'lik süre, en çok 8 s |
| sağa dön / sola dön | wz = ∓0.8·wz_max | bias düzeltilmiş gyro integrali 90° ya da 8 s |
| otur | sit = 1 | T2 |
| kalk | sit = 0, h = H_HOME | T1 |
| eğil / yüksel | h_cmd ∓ ΔH, [H_MIN_WALK, H_HOME] içinde | \|h_ref − h_cmd\| < 1e-6 |
| evet / hayır | jest: head_pitch / head_yaw | jest biter |
| vak | jest: mouth | jest biter |

## K5. Denetleyici (`rl/commands.py`)

commands.py tek tanımdır. Numpy ile ve baştan sona float32 yazılır. Eval ve play onu çağırır, firmware kopyalar;
eğitimde çalışmaz (K6). Geçişler bir veri tablosudur.

**Altın izler.** Her satır bir kontrol adımıdır:
- Girdiler: t, kimlik olayları, gravity_obs, gyro_obs, bacak qd_obs, alpha, h_ref.
- Çıktılar: durum, main komutu, politika seçimi, jest hedefleri, LED.
- İzler S1/S2 rollout'larından ve sentetik tablo testlerinden üretilir.
- Karşılaştırma: ayrık çıktılar birebir, sürekli çıktılar 1e-6 toleransla.

**Girdiler.** Yalnız firmware'in ölçebildiği sinyaller kullanılır: gravity_obs, gyro_obs, bacak qd_obs (sensör
modeli), posture_step'ten alpha ve h_ref, zaman ve kimlik olayları.

**Çıktılar.** Politika (main, getup ya da yok), main komutu, jest hedefleri, LED.

**Düşme algısı.** play.Supervisor'ın mantığı commands.py'ye taşınır; play ve eval onu çağırır. Parametreler
C.SUPERVISOR'dan gelir: fall_gz, fall_steps (debounce), ready_hold_s, handoff_zero_cmd_s.
- A, Ö, Y ve D'de mevcut ölçüt geçerlidir.
- O, M ve K'de düşme, eğim hem pitch_ref(alpha)'dan hem oturuş pozundan (SEAT_PITCH_DEG) > 35° saparsa ilan edilir.
  Oturuşla uyumlu bir duruş düşme sayılmaz. Bu duruşun ölçütü: eğim SEAT_PITCH_DEG'den en çok 15° sapar ve
  gz_obs < seat_gz.

**Tamamlanma (IMU ile):**
- **T1 kalkış:**
  - Koşul: alpha = 0, gz_obs < stand_gz ve \|gyro_obs\| < getup başarı eşiği.
  - Koşul uygulamadan sonraki 3.4 s içinde ilk kez sağlanır, ardından ready_hold_s boyunca kesintisiz tutar.
  - Sert sınır: STAND_RAMP_S + ready_hold_s + 0.5 s ≈ 3.9 s.
- **T2 oturuş:**
  - Koşul: alpha = 1, gz_obs < seat_gz, \|gyro_obs\| < aynı eşik, eğim SEAT_PITCH_DEG'den en çok 15° sapıyor.
  - Koşul ready_hold_s boyunca tutar. Sınır 4.0 s + ready_hold_s.
- **Başarısız kalkış** (T1 süresi doldu ama düşme yok): sit = 1 ile oturuşa dönülür, T2 beklenir, sonra bir kez daha
  kalk. İkinci başarısızlıkta red işareti verilir ve B'ye geçilir.
- **Başarısız oturuş:** T2'nin süresi dolarsa red işareti verilir ve B'ye geçilir.
- Geçiş hataları getup başlatmaz; getup yalnız gerçek düşmede devreye girer.

**Gyro bias.** Ölçüt büyüklüğe değil dağılıma bakar:
- Koşul: 1 s'lik pencerede eksen başına std(gyro_obs) < 2·σ_gyro, bütün bacak \|qd_obs\| < 0.2 rad/s ve hız komutu
  sıfır. Durum A ya da M olmalı.
- Bias pencere ortalamasıdır, \|b\| ≤ 0.06 rad/s ile kırpılır. Dönüş integrali bias'tan arındırılır.
- Birim test gerçek sensör modeliyle yapılır: b = 0.04 iken 2 s içinde güncellenir ve hata < 0.005 rad/s olur.

**Kuyruk.**
- Derinlik 1'dir. Sonra gelen komut bekleyenin yerine geçer. Jestler kuyruğa girmez.
- Kuyruk düşmede, B'ye girişte ve dur ile temizlenir.
- Değişmez: bekleyen bir komut, tamamlanan tek bir geçiş içinde ya uygulanır ya da temizlenir.

**Durumlar:**
- A: AYAKTA
- Ö: OTURMA ÖNCESİ (dur sonrası 1.0 s yerleşme)
- Y: YÜRÜYOR
- D: DÖNÜYOR
- O: OTURUYOR
- M: OTURMUŞ
- K: KALKIYOR
- F: DÜŞMÜŞ (getup)
- B: BEKLE

O ve K geçişleri tamamlanır; kesilmez.

**Giriş ve çıkış eylemleri:**
- **F → A (devir):** h_cmd = H_HOME, sit = 0, kuyruk temizlenir. handoff_zero_cmd_s boyunca gelen kelimeler kuyruğa
  girer ve süre bitince uygulanır.
- **B:** son komuttan 0.5 s'de ölçülen poza rampalanır. Ardından bacak PWM'i kapatılır (servolar gevşer), kafa HOME'da
  tutulur. LED yavaş yanıp söner.

**Hücre gösterimi:**
- "→X": durum X'e geçer.
- "kuyruk": komut bekletilir.
- "red": komut uygulanmaz ve red işareti verilir.
- "—": etkisiz, işaret yok.

| olay \ durum | A | Ö | Y | D | O | M | K | F | B |
|---|---|---|---|---|---|---|---|---|---|
| gel / geri | →Y | →Y (otur iptal) | →Y (yön yenilenir) | →Y | kuyruk | kalk, bekleyen = komut, →K | kuyruk | red | red |
| sağa / sola | →D | →D (otur iptal) | →D | →D (yeni yön) | kuyruk | kalk, bekleyen, →K | kuyruk | red | red |
| dur | jest sönümlenir | →A (otur iptal) | →A | →A | kuyruk temizlenir | kuyruk temizlenir | kuyruk temizlenir | getup kesilir, →B | — |
| otur | son hızdan 1.0 s geçtiyse →O, yoksa →Ö | — | dur, →Ö | dur, →Ö | — | — | kuyruk | red | red |
| kalk | — | →A (otur iptal) | — | — | kuyruk | →K | — | red | düşmüşse →F, oturuşta →K |
| eğil / yüksel | uygula | red | uygula | red | red | red | red | red | red |
| evet / hayır | jest | red | jest | jest | red | red | red | red | red |
| vak | jest | jest | jest | jest | red | jest (≤ 15°) | red | red | red |
| düşme | →F | →F | →F | →F | →F (K5 ölçütü) | →F (K5 ölçütü) | →F (K5 ölçütü) | — | — |
| tamamlandı / süre | — | 1.0 s doldu →O | →A | →A | T2 →M; bekleyen varsa kalk ve bekleyen, →K | — | T1 →A, bekleyen uygulanır; başarısız kalkış kuralı | başarı →A (devir); 12 s'de başarısız: 2. deneme, sonra →B | — |

- **Red işareti:** LED 3 kez yanıp söner. A, Ö, Y ve D'de ayrıca çift gaga tıkı yapılır. Tık, tanıma bitip dinleme
  durduktan sonra çalar.
- **Testler:** birim testler tabloyu satır satır gezer. Değişmezler:
  - F ve B'de main komutu çıkmaz.
  - Ö, M, O ve K'de hız komutu çıkmaz.
  - h, [H_MIN_WALK, H_HOME] içinde kalır.
  - Tabloda olmayan kimlik etkisizdir.
  - dur hiçbir durumda düşürülmez.
  - Bekleyen komutun ömrü, kuyruk kuralındaki değişmeze uyar.

## K6. Eğitime etkisi (P5k ve P5b; P7'den önce)

1. **Jest fonksiyonu.** Tek bir xp-genel fonksiyon task_math'te tanımlanır. env_cpu, env_mjx ve commands.py aynı
   fonksiyonu çağırır.
   - **Kafa kontrolü:** ctrl[kafa] = varsayılan + filtrelenmiş rastgele ofset + jest(t). Jest filtreyi atlar.
   - **Zarf:** Tukey; yarım çevrimlik açılış ve kapanış. Jest bir tam çevrim boyunca tam genliğe ulaşır, konum ya da hız
     basamağı yoktur.
   - **Hız sınırı:** toplam kafa ctrl çıktısında (varsayılan + ofset + jest) assert edilir: ≤ 0.5·ω0 (MG90S, SG90).
   - **Başlatma kapısı:** kafa jestleri alpha < ALPHA_STAND_MAX iken başlar. Gaga alpha < ALPHA_STAND_MAX iken ya da
     alpha = 1 iken başlar; alpha = 1'de en çok 15°.
   - **Kesme:** jest dur, otur, düşme ya da segment değişimiyle kesilirse 0.25 s'lik yükseltilmiş kosinüsle sönümlenir.
   - **Rastgele ofset:** jest sürerken rastgele hedef 0'da tutulur ve zamanlayıcı durur.
   - **Örnekleme:** main ayakta ve yürüme segmentlerinde her kafa yeniden örneklemesi 0.3 olasılıkla jest, 0.35
     rastgele, 0.35 sıfırdır. Gaga ve tık oturuş segmentlerinde de örneklenir, oradaki genlik ≤ 15°'dir.
   - walk ve getup değişmez.
   - **Gözlem değişmez.** Kafa jestleri yalnız pot q/qd ve IMU ile hissedilir. Gaga gözlenmez, bozucu olarak bulunur.

   | Jest | Eklem | Genlik | Frekans | Çevrim |
   |---|---|---|---|---|
   | baş sallama | head_pitch | ±0.2 rad | 1.5 Hz | 2 |
   | baş çevirme | head_yaw | ±0.4 rad | 1.2 Hz | 2 |
   | gaga | mouth | 0 ↔ 25° (oturuşta 15°) | 2 Hz | 2 |
   | tık | mouth | 0 → 12° → 0 | — | 0.3 s |

2. **Komut atomları** (task_math `sample_segment`, xp-genel):
   - main yürüme segmentlerinin %20'si atomdur: {(+0.8·vx_max, 0, 0), (0.8·vx_min, 0, 0), (0, 0, ±0.8·wz_max)}.
   - Her atom segmentinin süresi U(3.5, 15) s'dir.
   - Oturuştan önce ≥ 1 s'lik "tut" segmenti gelir: sıfır hız, önceki h korunur. Böylece oturuş başlangıcındaki h,
     [H_MIN_STAND, H_HOME] aralığını kapsamaya devam eder (çömel → otur korunur).
   - Atom seçimi mevcut uniformlardan yapılır: tür uniformu yeniden kullanılır, yön ve hız vx/wz uniformlarından
     türetilir. Ek uniform gerekirse fold_in kullanılır. CPU/MJX lockstep testi geçer.
3. **Gecikme.** Tanıma gecikmesi yalnız değerlendirmede rastgeledir. Yeni bir G-SES ölçümü yeniden eğitim gerektirmez.
4. **P5b'nin P7'den önce vermesi gereken kararlar:**
   - **EXTRA kutularının main eğitimindeki temas modeli.** Kutular zemine etkin olursa A6, A10 ve devir tek varyant
     kullanır, durum kopyası kalkar.
   - **main eğitim sonlandırması.** Sabit gz > −0.6 kuralı oturuşta yanlış sonlandırıyor: stub oturuşu gz −0.643, banka
     ±5°. Kural duruşa göre olmalı; ölçüt K5'teki düşme algısıyla aynıdır.
   - **Çift listesi.** Şu çiftler RL çift listesine girer: jaw ↔ neck/trunk, kafa ↔ kalça/kalça beşiği, ve spec
     3.2.2'nin bütün listesi.
   - **HEAD_SIT ve HEAD_TUCK.** Mesh girişim sınırıyla yeniden türetilir; stub'da head_pitch 55.5° iken ilk girişim
     33°'de başlıyor.

## K7. Kabul: A10 sesli komut senaryoları (simülasyon)

**Koşullar:**
- A4 koşulları: G-BENCH parametreleri, DR, itme, boşluk.
- Tanıma gecikmesi U(0.3, 1.5) s.
- Değerlendirme denetleyiciyle yapılır.
- Uyanma modeli: 15 s'lik, yenilenmeyen pencere. Bir kelimeden önce kalan pencere 1.5 s artı kelime süresinden kısaysa
  önce "ördek" söylenir: gecikme U(0.3, 1.5) s, artı 0.5 s tepki.
- Bölüm main varyantında başlar. Düşme algılanınca A6 mekanizmasıyla (tr35_handover durum kopyası) getup varyantına
  geçilir. Kopya anındaki en büyük EXTRA–zemin girişimi kaydedilir; 2 mm'nin üstündeki kopyalar ayrı raporlanır.
- A6 devri aynı düzenekle yeniden koşulur.

**S1 sabit dizi** (oturmuş başlangıç, N = 256 bölüm):
- Dizi: kalk, gel, dur, sağa dön, sola dön, geri, eğil, yüksel, evet, hayır, vak, otur.
- Zaman çizelgesi:
  - Her kelime, önceki komut tamamlandıktan 1.0 s sonra söylenir.
  - dur, gel uygulandıktan 8 s sonra söylenir.
  - dur'dan hemen önce yeniden uyandırılır.
- Ölçütler:

  | Ölçüt | Tanım | Eşik |
  |---|---|---|
  | Tamamlanma | Her adım tamamlanır, düşme olmaz. otur ve kalk A5'in gerçek durum başarı kurallarıyla, diğerleri K5 ölçütleriyle puanlanır. | nokta ≥ %85, W-alt ≥ %80 |
  | gel | gel başlarkenki yön boyunca yer değiştirme / (0.8·vx_max × uygulama süresi) | p10 ≥ 0.7, p90 ≤ 1.3 |
  | Dönüş | Dönüş bitişinden 1 s sonra \|gerçek yaw değişimi − 90°\| | p90 ≤ 20° |
  | eğil / yüksel | h_ref hedefe ulaştıktan 1 s sonra ölçülen gövde z değişimi | p10 ≥ 0.5·ΔH |
  | evet / hayır | Ortadaki tam çevrimde, bias çıkarılmış pot q tepeden tepeye ≥ 0.8 × aynı çevrimin komut tepeden tepesi | p10 |
  | dur | Kelimenin bitişinden (gecikme dahil) \|v\| < 0.02 m/s olana kadar gidilen mesafe | p90 ≤ 0.4 m |

**S2 rastgele dizi** (20 kelime, N = 256, başlangıç %50 HOME ve %50 oturmuş):
- Kelimeler 12 komut üzerinde düzgün dağılır.
- Her kelime, önceki kelimenin uygulanmasından U(1, 6) s sonra söylenir. Böylece kelimeler geçişlerin ve hareketlerin
  ortasına da düşer. Red, "—" ve kuyruk kelimelerinin uygulanması, kelimenin işlendiği andır.
- Ölçütler:
  - Düşme oranı 20 s'lik maruziyete normalize edilir: W-üst ≤ %3 (A4 ile aynı).
  - Geçiş düşmeleri geçiş başına W-üst ≤ %5 (A5 ile aynı).
  - K5 değişmezlerinde 0 ihlal.
  - Tablonun her hücresi 256 bölümde en az 20 kez tetiklenir. Fizikte erişilemeyen hücreler listelenir ve gerekçesi
    yazılır.
  - "Düşme + başarılı getup + devir" ayrı kategoride raporlanır.

**Her iki dizide de:**
- A4'ün tork, ray ve limit ölçütleri ile A8 geçer.
- Duyarlılık (yalnız rapor): gyro bias U(±0.04) rad/s. Bias güncellemesi almış dönüşlerin oranı da raporlanır.
- Video: S1'den bir koşu (play.py).
- Sürüm: G-BENCH'ten önce varsayılan parametrelerle "ön sürüm" olarak koşulur. Kabul G-BENCH sonrasıdır.

## K8. Adım 0 eki: G-SES-a (kullanıcı, tezgâhta; yaklaşık 1.5–2 saat)

**Ön koşul.** P6 ekinin tezgâh betiği hazır olmalı. Betik şunları yapar:
- çok kanallı jest ve salınım programı; analog MG90S ve SG90 için 50 Hz'lik ayrı bir PCA ya da kanal grubu;
- DF2301Q UART okuma ve kimlik zaman damgası;
- piezo ile senkron tık;
- uyanık durum kaydı: NOTIFY çerçeveleri, doğrulanmazsa modül LED'ine bakan fototransistörden ADC ile;
- öğrenme modu.

**Hazırlık:**
- Kumpasla ölçülür: revizyon, delikler, mikrofon merkezleri, port yönü, yükseklik, konnektör yeri ve takılı fiş
  yüksekliği. Kütle tartılır.
- Akım ölçülür: boşta, dinlerken, uyanık, açılış ani akımı.
- 5 V'ta TX boşta gerilimi ölçülür. RX'in 3.3 V sürüşle çalıştığı doğrulanır.
- "ördek" ve 12 kelime öğretilir; kimlik tablosu yazılır.

**Düzenek:**
- Çıplak modül 0.35 m yükseklikte durur.
- Konuşmacı ayaktadır; ağız-modül mesafesi 1.3 m ve 1.8 m.
- Gürültü: Adım 0'ın MG90S ve SG90'ı modülden yaklaşık 4 cm uzakta jest programını, bir DS salınım programını koşar.

**Denemeler:**
1. Gürültülü düzenekte, 1.3 m'den, her kelime 20 kez "ördek → LED → komut" sırasıyla söylenir.
2. Sessiz ortamda, 1.8 m'den, her kelime 10 kez söylenir. Ayrıca aralıksız "ördek gel" biçimi 10 kez denenir.
3. **Uyanık pencerede yanlış komut testi** (servolar çalışırken, konuşmadan):
   - Pencere setWakeTime(255) ve 200 s'de bir yenilemeyle açık tutulur. Bu yöntem önce 10 dakikalık kısa bir koşuyla
     doğrulanır. Tutmazsa kayıtlı "ördek" hoparlörden yaklaşık 14 s'de bir çalınır ve her çalmadan sonraki 2 s
     maruziyetten düşülür.
   - Koşu, doğrulanmış uyanık süre 30 dakikaya ulaşana kadar sürer.
   - Canlılık: her 5 dakikada ve sonda kayıtlı bilinen bir kelime çalınır ve tanınması doğrulanır.
   - Doğrulanmış uyanık süre 30 dakikanın altında kalırsa sonuç "geçti" değil "ölçülemedi"dir.
   - Uyku durumundaki yanlış uyanmalar ayrıca sayılır.
4. **Modül davranışı:** şunlar ölçülür:
   - pencerenin geçerli komutla yenilenip yenilenmediği;
   - NOTIFY çerçevelerinin gelip gelmediği;
   - setWakeTime ve setMuteMode'un güç kesilince kalıp kalmadığı;
   - öğrenme istemlerinin kafa kesiti içinden duyulup duyulmadığı.
5. **Firmware sözleşmesi için canlılık:** modül sessizleşirse fark edilebilmeli. Uygun yol, NOTIFY ya da LED kaydıyla
   belirlenir.

**Kapı:**
- Kelime başına doğruluk ≥ 18/20; dur için ≥ 19/20.
- Havuzda ≥ %90, Wilson alt sınırı ≥ %85.
- Tehlikeli çiftlerde (gel/geri, sağa/sola, otur/kalk) ikame çift başına ≤ 1.
- Doğrulanmış 30 uyanık dakikada 0 yanlış komut; Poisson %95 üst sınırı 1/10 dk.
- "ördek → komut" zinciri ≥ %90.

Ölçülen gecikme dağılımı yalnız değerlendirme DR'ına yazılır. Kapı tutmazsa K3'teki yedek yol uygulanır.

**Sıra.** Mümkünse G-CAD'den önce yapılır. Gecikirse yerleşim yine kafa olarak donar.

## K9. Plan etkisi ve sıra

- **P5k** (şimdi, yer tutucu modelde, P5a üstünde):
  - `rl/commands.py` ve tablo testleri;
  - altın izler;
  - jest fonksiyonu ve komut atomları (CPU ve MJX, lockstep);
  - A10 düzeneği (modül modeli, uyanma, gecikme) ve değerlendirme iskeleti (eval_tasks, play).

  P5b'ye birleşir.
- **P6 eki** (şimdi; G-SES-a'nın ön koşulu): K8'deki tezgâh betiği eklemeleri, pin haritası ve kullanım kılavuzu eki
  (`docs/adim0-tezgah-testi.md`).
- **Adım 0 alımı:** modül, piezo, fototransistör.
- **P2d** (dalga 2 entegrasyonundan sonra, P3 ve G-CAD'den önce):
  - kafa yerleşimi, conta, beşik, LED ve pencereler;
  - demet ICR'si, K3.6'daki dosya yetkileriyle;
  - 30 g kütle aktarımı;
  - MG90S kontrolü;
  - ölçü yoksa muhafazakâr zarf kullanılır.
- **P3:** K3'teki BOM satırları. G-CAD modülü içerir.
- **P5b:** P5k son MJCF'de; K6.4 kararları; A10 son modelde.
- **P7/P9 eğitimleri:** jest fonksiyonu ve komut atomlarıyla eğitilmiş main.
- **P10:** A10 ve video.
- **G-SES-b:** robot kurulunca (K2).

## K10. Bütçe (kullanıcı kararı, 2026-10-24)

Tavan yükseltildi:
- zorunlu ≤ 25.800 TL (önceki 24.500);
- yedeklerle ≤ 28.800 TL (önceki 27.500).

Ses katmanının tam farkı (modül, kargo ve K3'teki yan kalemler) bu tavana sığmalıdır. P3, A2'yi bu tavanla hesaplar.
Sığmazsa kullanıcıya dönülür.
