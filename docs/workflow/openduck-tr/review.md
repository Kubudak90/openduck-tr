# Review: OpenDuck-TR spec

Codex MCP bu oturumda bağlı değildi; inceleme, aynı bağlam paketiyle (spec + research + repo özeti +
değiştirilemez kısıtlar) üç bağımsız inceleyiciyle yapıldı: mekanik/elektrik/BOM, RL/sim, kapsam/kabul.
Format zorunluydu: `[SEV] itiraz + somut senaryo + öneri`. Ham itirazlar: `research/review-r1-objections.md`.

## Tur 1: 34 itiraz (4 critical, 22 major, 8 minor) → kararlar

| İtiraz | SEV | Karar | Spec'e etkisi / gerekçe |
|---|---|---|---|
| O1 / SC-1 bütçe tutmuyor | critical | **Kabul** | Kullanıcı kararı (2026-10-23): tavan ~25k TL, DS en ucuz kaynaktan (Robocombo DS3225MG 1.031 TL / Direnc DS3230 1.046 TL; tezgâh seçer). A2: zorunlu ≤ 23.5k, yedeklerle ≤ 25k; BOM satır satır yeniden hesaplanır. Motorobit yalnız yedek kaynak. |
| O2 / SC-5 eklem sınırı sonsuz güçlü dayanak; kafa zinciri | critical | **Kabul** | §3.2.3: MJCF eklem range = ctrlrange + 3°; limit kısıt torku (efc) metriği ve cezası; §6 A4–A6 limit torku p99 ≤ 0.1·τs (kafa: ≤ MG90S stall). CAD: kafa darbesini boyun U-link'ine aktaran çene/alın tamponu. |
| O3 9 DS'lik ray akımı | major | **Kabul** | Env'de paylaşımlı ray modeli (I = Σ I_stall·\|τ_net\|/τs + boşta); I > I_CC (16 A) ise DS forcerange'leri ölçeklenir. A4–A6: p99 I_rail ve > 15 A süre oranı. Yedek: bacak başı ayrı buck. |
| O4 hip_yaw yapısal yük | major | **Kabul** | Kalça braketi iki rulmanlı döner tablaya (≥ 20 mm aralık) oturur; ES3054 yalnız tork iletir (pim/yarık kaplin). A1'e "servo miline eğilme/eksenel yük yok" + el hesabı. Rulman stoğu planda doğrulanır. |
| O5 gövde paketleme/ısı | major | **Kabul** | Satın alınan her elektronik CAD'de gerçek zarfla; trunk iç hacim bütçesi; buck havalandırması; ısı kaynağına PETG + PLA'ya ≥ 5 mm hava. Trunk dondurulmadan MJCF/banka üretilmez (sıralama kapısı). |
| O6 kütle tahmini düşük | major | **Kabul** | Braket kütlesi servo zarfından aşağıdan yukarı; et/dolgu config parametresi. A1 sert sınır 1.95 kg, hedef 1.85 kg (ısıl ölçüt kütleye bağlı, SC-4). |
| O7 BMS'siz LiPo | major | **Kabul** | BOM: denge soketine LiPo voltaj alarmı; INA226 yalnız VBUS (seri şönt yok); CAD'de MOSFET/latch yeri; ana kesme XT30. Anahtar 16.8 V DC'ye uygun seçilir. |
| O8 hip_pitch −75 pay kuralını çiğniyor | minor | **Kabul** | Sınır −70…+90° (servo −75.6…+104.4; gerekli −52…+90). |
| O9 boyun DS entegrasyonu | minor | **Kabul** | ADS1115 yolu, işaret kuralı ve BOM satırı spec'te. |
| O10 rejeneratif aşım | minor | **Kabul** | Bacak rayına TVS + kapasitör; Adım 0'da yavaşlama aşımı ölçülür. |
| O11 iletken planı | minor | **Kabul** | Eklem geçişlerinde demet çapı ve bükülme yarıçapı CAD parametresi; taramaya dahil. |
| O12 plastik horn | minor | **Kabul** | DS eklemlerinde yalnız alüminyum 25T kol. |
| RL-1 / SC-3 A4 varsayılanlarla çelişiyor; κ | critical | **Kısmen kabul** | (a) Kapı u(τs, ω0, ölçülen kütle) ≤ 0.85; Blue varsayılanla κ = 0.5 (1.10 Hz). (c) Kabul eğitimleri Adım 0 parametreleri ve daraltılmış DR ile; öncesindeki eğitim yalnız boru hattı denemesi. (b) Bölüm başına gait frekansı örnekleme **red**: gözlem boyutu ve karmaşıklık artar; Adım 0 sonrası tek yeniden eğitim ucuz (~2–5 $). |
| RL-2 / SC-2 tork tanımı | major | **Kabul** | τ_act = actuator_force, τ_net = τ_act − b·qd (akımla orantılı). Isıl/doygunluk ölçütleri τ_net / τs üzerinden; ceza bölüm başına örneklenen τs ile, kabul nominal τs ile. Tek tanım `rl/actuators.py`. |
| RL-3 kalkma yapılabilirlik kapısı; mod başına | major | **Kabul** | Tam eğitimden önce kapı: kafa sabit/toplanmış ve kafa serbest kısa koşular, mod başına sonuç. A6 mod başına (Wilson %95) taban: yüz ≥ %80, yan ≥ %70, sırt ≥ %50; kapı sırtın imkânsız olduğunu gösterirse spec'e dönülür. Kafa eklemleri pot ile ölçülür (ikinci ADS1115). main, getup'tan önce eğitilir. |
| RL-4 boşluk dinamik | major | **Kısmen kabul** | DS eklemlerinde bayrakla açılan seri boşluk eklemi (±b/2, DR 0–1°); kabul koşusu boşluk açıkken. Komut→açı hatası aktüatör tarafında ctrl biası. Boşluğu yalnız gürültü olarak modelleme kaldırıldı. |
| RL-5 / SC-10 IMU ve tutum filtresi | major | **Kabul** | `rl/sensors.py`: IMU modeli (gyro gürültü/bias/ölçek, ivmeölçer gürültü/titreşim) ve aynı tamamlayıcı filtre gravity gözlemini üretir. Firmware birebir kopyalar. |
| RL-6 pot modeli eksik | major | **Kabul** | Ortak-mod ray hatası, ADC beyaz gürültü, ADS1115 yolunun ayrı gecikme/örnekleme dinamiği. |
| RL-7 main zaman ölçekleri | major | **Kabul** | κ yalnız yürüyüş gaiti için; main rampaları/pencereleri λ_t = √s: rampa 2.83 s; A5 otur ≤ 4.0 s, kalk ≤ 3.7 s. |
| RL-8 / SC-11 eval kapsamı, eksik kriterler | major | **Kabul** | eval_tasks güncellemesi kapsamda (tork/ray/limit metrikleri, mod başına, main yürüme, devir senaryosu). A4 ≥ 256 bölüm + Wilson. main için yürüme takip ≥ walk'ın %90'ı; ES3054/MG90S RMS ≤ stall/3 ve p99 ≤ 1.0; main→getup→main sim senaryo testi; boy RMS ≤ 1.3 cm. |
| RL-9 256-128-64 kanıtsız | minor | **Kabul** | A7: küçük aktör A4–A6'yı tutmazsa 512-256-128 + int8/PSRAM tahmini raporlanır. |
| RL-10 ölü bant tek aralık | minor | **Kabul** | Sınıf başına ölü bant; MG90S/SG90 50 Hz davranışı Adım 0'da test edilir. |
| SC-4 kütle ↔ diz ısıl | major | **Kabul** | Isıl eşik Adım 0 ısınma testiyle kalibre (varsayılan stall/3); çıkış yolları: dizi daha az büken HOME/H_HOME, diz tork cezası, DS3230 Red. DS3225 ısıl yedek değil (yalnız hız/kaynak adayı). |
| SC-6 Adım 0 karar tablosu, sysID | major | **Kabul** | §7 karar tablosu (eşik → parametre güncelle / yeniden eğit / spec'e dön). `tools/bench/` (ESP32 tezgâh betiği + PC kaydedici) ve `tools/sysid/` kapsamda. |
| SC-7 A1 sınır kriteri totolojik | major | **Kabul** | Görev başına gerekli eklem aralıkları tablosu; sınırlar ⊇ gerekli; komşu eklem çiftleri birlikte taranır. |
| SC-8 CAD → RL sıralama kapısı | major | **Kabul** | Planda G-CAD kapısı: trunk/bacak donar, kütle/COM yazılır → RL sabitleri, poz bankaları. |
| SC-9 XL330 regresyon referansı | major | **Kabul** | Refactor'un ilk aşaması: `configs/xl330_27.json` eski micro MJCF'yi bayt-eşit (float biçimi dahil) üretir; sonra TR-35'e geçilir. A3 parity toleransı: aynı durumdan tek adım atol 1e-4, 50 adımlık eşli koşu (mevcut parity testi). |
| SC-12 ADS1115 zamanlaması | minor | **Kabul** | İki ADS1115 (0x48 boyun + 3 kafa; 0x49 2 NTC + 2 boş), 860 SPS mux (kanal başı ~215 Hz); yol başına gecikme modeli. |

## Tur 2 (son): spec v2 üzerine 2 inceleyici (taze göz + çözüm denetçisi), 20 itiraz

Ham itirazlar: `research/review-r2-objections.md`. Tur 2'de 2 kritik itiraz kaldı. İş akışı kuralı gereği
otomatik devam edilmedi, uyuşmazlık kullanıcıya özetlendi ve kullanıcı karar verdi (2026-10-23).

| İtiraz | SEV | Karar | Spec v3'e etkisi |
|---|---|---|---|
| R2-1 / F3 BOM tavanı tutmuyor | critical | **Kullanıcı kararı** | Tavan ~27.5k TL: zorunlu ≤ 24.5k, yedeklerle ≤ 27.5k. Şarj cihazı BOM'da. Yedek seti adet adet yazıldı. Adım 0 alımları robota/yedeğe gider ve tavana dahildir. TVS, anahtar ve döner tabla rulmanı için stok doğrulaması ve alternatif yazıldı. |
| F1 / R2-4 diz ısıl ölçütü her durumda düşüyor | critical | **Kullanıcı kararı** | Dize DS3235 de aday (aynı gövde). Isınma testi 2–3 yük seviyesinde; eşik iki yönde kalibre. G-ISIL kapısı (boru hattı politikalarından ölçülen diz RMS × tezgâh τs × CAD kütlesi) kabul eğitiminden önce. Diz servosu yalnız T_diz ≥ tahmini RMS ise seçilebilir. DS3230 Red çıkış listesinden çıkarıldı (Türkiye stoğu yok). |
| F2 kütle sınırı aşımı tanımsız | major | **Kabul** | Gramlı kütle merdiveni. Kütle = CAD aşağıdan yukarı × 1.05. Merdivenden sonra > 1.95 kg ise RL'den önce spec'e dönülür. |
| F4 / R2-7 Wilson yönü, N | major | **Kabul** | Her oran ölçütünde N ve kural yazıldı (nokta + Wilson sınırı). Dönüş için sabit wz komutları ve kararlı durum penceresi tanımlandı. |
| F5 getup kapısı sayısız | major | **Kabul** | Kapı: tam bütçenin %30'u, G-BENCH parametreleriyle. Mod başına eşik ≥ 0.6 × A6 tabanı ve pozitif öğrenme eğimi. (i) ile (ii)'den iyisi tam eğitime gider. Düşen mod için spec'e dönülür. |
| F6 / R2-2 limit metriği ↔ boşluk eklemi | major | **Kabul** | Limit metriği ve cezası yalnız aktüatörlü ana eklemlerin LIMIT_JOINT satırlarından; boşluk eklemleri hariç. Pot servo çıkış milini okur. Boşluk ara gövdesine armature ve solref verildi. Boşluk DR'ı §4'te. |
| R2-3 +3° kuralı varyant sınırlayıcılarıyla eziliyor | major | **Kabul** | +3° kuralı `robot_model.build_spec`'te bütün sınırlayıcılardan sonra uygulanan değişmez (testli). KNEE/NECK/HIP_YAW sınırları TR-35 için CAD'den yeniden türetilir. A3: varyant sınırları ⊇ gerekli aralıklar. |
| F7 öz-çarpışma yok | major | **Kabul** | main/getup'a seçilmiş öz-çarpışma çiftleri (uyluk/incik ↔ trunk/kafa/çene, ayak ↔ kafa). MJX maliyeti A3'te ölçülür. Getup kapısında A8 de raporlanır. |
| F8 ray akımı modeli besleme akımı değil | major | **Kabul** | I_supply ≈ I_stall·\|τ_act\|·\|τ_net\|/τs² + boşta. Tezgâhta INA226 + uygun şöntle uydurulur. Metrik ölçekleme öncesi talep akımı. |
| R2-5 tezgâh akım ölçümü | major | **Kabul** | Tezgâhta 2–10 mΩ şönt ya da laboratuvar kaynağı. sysID'de iki yöntem arasındaki fark ≤ %10. |
| R2-6 HEAD_RANDOM, kafa limit eşiği, tampon geomu | major | **Kabul** | HEAD_RANDOM MG90S'e göre yeniden türetildi (aralık + eğim sınırı). Kafa eklemlerinde limit eşiği ≤ MG90S stall. Tampon MJCF'de boyun gövdesine bağlı çarpışma geomu. |
| F9 A7 iki dallı değil | minor | **Kabul** | A7 iki dal: küçük aktör float32, ya da büyük/damıtılmış öğrenci int8 (A4–A6 farkı ≤ 2 puan). Damıtmaya izin var. |
| F10 / R2-10 gerekli aralık tablosu eksik; 3.7 s | minor | **Kabul** | 15 eklemin hepsi için görev başına sayısal tablo; boyun −90…+40 (nötr −25°). Kalkış penceresi 3.4 s (= 3.0·√s). hip_yaw iç sınırı CAD'den; < 10° çıkarsa wz komut aralığı daralır. Bacak–bacak çifti taramada. |
| R2-8 ADS1115 bölücü/VDD, boyun işareti | minor | **Kabul** | ADS VDD 3.3 V, ADC1 ile aynı bölücü (100k/47k + 100 nF, τ 3.2 ms), PGA ±4.096 V. Boyun horn yönü sabit; işaret testi A9'da. |
| R2-9 Adım 0 eksik satırlar | minor | **Kabul** | Satırlar eklendi: ray aşımı, 3.3 V darbe, ortak-mod hata, ES3054 300 Hz. ES3054/MG90S/SG90 6.0 V'ta ölçülür; SG90 alınacaklarda. |

İnceleme aşaması kapandı. Tur 2'deki kritik itirazlar kullanıcı kararıyla çözüldü, kalan bütün itirazlar spec v3'e işlendi.
