# P5a notları: geometriden bağımsız P5 (P2-0 yer tutucusuna karşı)

P5 asıl olarak G-CAD'den sonra başlar. P5a, son geometriye bağlı olmayan her şeyi şimdi, P2-0 yer tutucu (stub)
MJCF'sine karşı yaptı. Kural şu: tr35'in geometriye bağlı her sabiti bir araçla MJCF'den üretilir, elle yazılmaz.
G-CAD sonrasında tek komut hepsini yeniden üretir (bölüm 3). xl330_27 profili bit düzeyinde değişmedi.

Kaynaklar: `spec.md` v3.1 (3.2.3-3.2.7, 3.3, 3.4, 4, 6), `plan.md` (P5, P7, G-ISIL), `research/rl-map.md`,
`p2-interface.md`, `p2c-tampon.md` (P2c dalı), `rl/TASK.md` "Robot profiles".

## 1. Birleştirilenler

Entegrasyon dalı main 755e2ea'dan açıldı (`git merge --ff-only main`: zaten günceldi). Üç paket `--no-ff` ile bu
sırayla birleştirildi:

| Paket | Dal | Commit'ler | İçerik |
|---|---|---|---|
| core | `worktree-wf_30db4214-cf8-2` | b019bcc, 5d7420b, e131819 | tr35 görev profili (config, tasks, task_math, pose_bank, robot_model, iki ortam), çözücü ayarı, `rl/tests/test_tr35_core.py`, `tools/tr35_smoke.py`, `tools/mjx_solver_bench.py`, `rl/TASK.md` |
| derive | `worktree-wf_30db4214-cf8-3` | 7ddb42f, f361c13 | `tools/derive_posture.py`, `rl/derived/tr35_b2.json` (stub'dan), `rl/tests/test_derive_posture.py` |
| deploy | `worktree-wf_30db4214-cf8-4` | 99d6c0f | tr35 eval (spec 6 A4-A8), `rl/export_c.py`, tr35 `policy_meta`, play, eğiticiler, RunPod başlatıcısı |

Dosya sahiplikleri ayrık olduğu için birleştirmede çakışma çıkmadı. Ardından bir entegrasyon commit'i paketleri
birlikte çalışır hâle getirdi (bölüm 2).

## 2. Kararlar

Paket raporlarında hiç INTERFACE CHANGE REQUEST yoktu (derive ve deploy: "yok"; core raporu oturum sınırında
kaybolmuştu, commit gövdelerinde de istek yok). Birleştirmeden sonra çıkan uyumsuzluklar ve verilen kararlar:

1. **Gerçek türetilmiş dosya yüklenmiyordu.** `config.load_derived`, `H_DEEP > Z_SIT` istiyordu. Stub'da
   Z_SIT 86.5 mm, H_DEEP 86.0 mm. derive bu durumu yalnızca uyarı sayıyor. Karar: çekirdek, `derive_posture.validate`
   ile aynı sırayı ister: `H_HOME > H_MIN_WALK >= H_MIN_STAND >= H_DEEP` ve `SEAT_Z < Z_SIT`. `Z_SIT < H_DEEP` uyarı
   olarak kalır. `z_ref` doğrusal ara değer olduğundan oturuş yolu çalışır (0.5 mm yükselir). Dosya eksik ya da
   geçersizse çıkan `RuntimeError` artık tam yeniden üretme komutunu yazıyor.
2. **Çekirdek testleri gerçek JSON'u kullanıyor.** `rl/tests/test_tr35_core.py` içindeki `_boot()` artık
   `rl/derived/tr35_b2.json` döndürüyor. Test-yalnız bootstrap dosyası yalnızca bu dosya yoksa kullanılır. Bootstrap'a
   özgü beklentiler veriden okunur hâle geldi: `HOME_OVERRIDE` (family HOME), 30 öz-çarpışma çifti, model başına çift
   sayısı (walk 0, main 8, getup 30), `tool` adı. Bozuk dosya örneği "Z_SIT" artık `Z_SIT = SEAT_Z` kullanıyor.
3. **Oturuş bankası stub'da marjinal.** Tam türetilmiş oturuş pozu (pitch −50°, COM payı 6.9 mm) gürültüsüz
   yerleştirilince −49.2° / z 83.3 mm'de durur (SEAT_Z 84.0 mm). Banka yerleştirmeleri (±5°, σ 0.05 rad) ise öne
   devrilip gövde üstüne yatıyor (pitch +3..+12°, z 23-31 mm) ve `seat_gz` = −0.245 eşiği yüzünden kabul ediliyor.
   Karar: test, tam pozun oturtmasını (işaret kuralı + SEAT_Z, çekirdek ↔ derive arayüzü) her zaman doğrular. Banka
   pitch/z penceresini yalnız `source.stub = false` olan dosyada zorunlu tutar (P5b). Çekirdeğin kabul kuralı
   değişmedi.
4. **`rl/tests/test_actuators.py` main'deki hâline döndü.** core, türetilmiş dosya yokken alt süreçte bootstrap
   kullanan bir dal eklemişti. Dosya artık commit'li olduğu için o dal etkisizdi. Geri alındı; mevcut test
   değişmeden geçiyor.
5. **Çekirdek hatası (deploy'un bildirdiği) düzeltildi.** `actuators.apply_to_spec`, boşluk eklemi adreslerini
   bulmak için spec'i derliyordu. Bu derleme yüzünden sonradan doku silinen modeller derlenmiyordu
   (`physics_model(textures=False)` + `backlash=True`: "texture 'grid' not found"). Artık ara derleme
   `spec.copy()` üzerinde yapılıyor. xl330 etkilenmez (boşluk yalnız tr35'te). eval'deki geçici çözüm kendiliğinden
   kapanır, artık 16'dan fazla ortamlı bir backlash VecEnv kurulabiliyor.
6. **eval yeni cezaları ve metrikleri okuyor.**
   - `TorqueAcc` her bölümde eğitim cezalarını raporluyor: `penalties` {torque_ratio, limit_torque, rail_demand}.
     Bunlar ağırlıksız adım ortalamalarıdır, bölümün tau_s'iyle `task_math.tr35_penalties` üzerinden hesaplanır.
   - Raporda ayrıca `torque_sat_steps_frac` ve `rail_soft_a` var.
   - Ray ölçütünün eşiği artık `C.RAIL_SOFT_A`; değer aynı, 15 A.
   - A6 mod başına ortamın getup bölüm metriklerini ekliyor: getup_success, head_impacts, head_impact_v2,
     torque_sat_steps ve adım oranı.
   - Çekirdek adları (`DERIVED_SOURCE`, `KAPPA`, `GETUP_HEAD_DEFAULT`) artık `getattr` yerine doğrudan okunuyor.
   - Ölçütler değişmedi.
7. **`policy_meta` ve `export_c` türetilmiş ve aktüatör sabitlerini okuyor.**
   - `policy_meta` çekirdek adlarını doğrudan okuyor: `DERIVED_SOURCE`, `KAPPA`, `GETUP_HEAD_DEFAULT`,
     `HEAD_TUCK_DEG`, `SEAT_PITCH_DEG`.
   - Meta'ya `derived_sha256` eklendi (yüklenen JSON'un sha256'sı).
   - `export_c`, tr35 meta'sı için `obs_layout.h` içine robot sabitlerini yazıyor: `OPENDUCK_NU`,
     `OPENDUCK_ACT_NAMES/TAU_S/OMEGA0/KP/DEADBAND/PWM_RES/CTRL_LO/CTRL_HI/BOARD`, kart PWM frekansı ve PCA adresi,
     `OPENDUCK_DERIVED_MJCF_SHA256/STUB/SHA256`, `OPENDUCK_KAPPA`. getup fixed'te `OPENDUCK_HEAD_TUCK_RAD[4]`,
     main'de `OPENDUCK_SEAT_PITCH_DEG` de yazılıyor.
   - `export_c.json` derived_source, derived_sha256 ve kappa'yı tekrarlıyor.
   - xl330 meta'sı bayt düzeyinde aynı; xl330 meta'sında robot bloğu yok.
8. **play tr35 koşuyor.** Kod değişmedi. tr35 sürecinde, gerçek JSON ve stub ile sit ve getup demoları
   `test_play_tr35_demos_smoke` içinde koşuyor.
9. **Öz-çarpışma çiftleri.** core'un seçimi korundu: main alt kümesi (8 çift), getup türetilmiş tam liste (30 çift),
   walk çiftsiz. Gerçek JSON ile maliyet ölçüldü (bölüm 5). getup'ta maliyet yaklaşık iki katı; karar P5b'de GPU
   ölçümüyle verilecek.
10. **CPU ile MJX eşitliği öz-çarpışma çiftleriyle bozuluyor (yeni bulgu).** Gerçek JSON ile tr35 sürecindeki adım
    adım test (`test_tr35_lockstep_cpu_mjx`) main'de başarısız oldu:
    - ödül farkı 0.02, kritikte `foot_contact` 0/1 çevriliyor;
    - core'un dalında bootstrap dosyasında çift olmadığından bu görünmemişti;
    - çiftler kapatılınca eşitlik tam: main ödül farkı 1.5e-6, getup 1.5e-6, kritik ≤ 1.1e-4.
    Neden: MJX kutu-kutu teması çift başına 4 temas noktası üretiyor, MuJoCo C aynı durumda 1-2 nokta. Stub
    bankalarında uyluk/incik ↔ gövde çiftleri oturuşta 32 durumun 12'sinde, düşmüşte 17'sinde temasta: C 17 / 43
    nokta, MJX 60 / 92 nokta. Stub'ın EXTRA uyluk kutusu çok büyük (82 × 63 × 58 mm).
    Karar: test çiftsiz modelde koşuyor (türetilmiş dosya, HOME, bankalar, tr35 ödülleri, sensörler doğrulanır).
    Eğitim ve eval çiftlerle çalışıyor. P5b'de son EXTRA geometrisiyle ele alınır (bölüm 7 madde 5).
11. **`rl/TASK.md`.** Eklenenler: yeniden üretme komutları, `load_derived` sırası, main/getup çift kuralı ve maliyeti,
    stub'daki oturuş bankası notu. deploy'un istediği bölüm de eklendi (meta tr35 alanları, `<run>/c/` dosyaları,
    `rl.export_c` CLI, eval tr35 CLI ve JSON, `--getup-head`, play `--scene` / `--getup-head`, launch `--robot` /
    `--kappa`).

Değişen yeni testler, yani bu P5a paketlerinin kendi dosyaları:
- `test_tr35_core.py`: madde 2 ve 3.
- `test_eval_tr35.py`: cezalar, A6 metrikleri, meta/C başlığında türetilmiş dosya kimliği.
- `test_export_c.py`: robot bloğu derlenir ve meta ile aynıdır.

Baseline'daki (main 755e2ea) hiçbir mevcut test düzenlenmedi.

## 3. Türetme komutu ve yazdığı dosyalar

```
python tools/derive_posture.py derive --scene build/tr35_b2/sim/scene.xml --out rl/derived/tr35_b2.json
python tools/derive_posture.py check  --scene build/tr35_b2/sim/scene.xml --derived rl/derived/tr35_b2.json
python tools/derive_posture.py caps-scan --scene build/tr35_b2/sim/scene.xml      # isteğe bağlı A8 voksel taraması
```

Stub için `--stub` eklenir. `check` çıkış kodları: 0 geçerli ve taze, 3 geçerli ama openduck.xml ya da mesh sha256'sı
farklı, 1 geçersiz. Araç `build/` ve `configs/` içine yazmaz, yalnız `--out` dosyasını yazar. Doğrulama başarısızsa
hiçbir şey yazmaz (`--force` hariç).

Yazdığı tek dosya `rl/derived/tr35_b2.json`. Şema 1, belirlenimci: zaman damgası yok, sıralı anahtarlar, indent 1.
Bu dosya tr35 sürecinde geometriye bağlı her sabitin tek kaynağıdır. Okuyan `rl/config.py`'dir, import sırasında
`load_derived` + `tr35_constants` ile:

| JSON | config |
|---|---|
| `heights` | `H_HOME` (= `BASE_HEIGHT_TARGET`), `H_MIN_WALK`, `H_MIN_STAND`, `H_DEEP`, `Z_SIT`, `SEAT_Z`, `SEAT_Z_TOL`, `TERMINATION_MIN_HEIGHT`, `H_NORM`, `MAIN["standup_z"]`, `GETUP["success"]["z"]`, `GETUP["pose_z"]`, `GETUP["arrival_z"]` |
| `crouch_family` | `CROUCH_FAMILY` (F tablosu) |
| `seat` | `SEAT_DEG`, `SEAT_PITCH_DEG`, `MAIN["seat_gz"]` |
| `head` | `HEAD_HOME_DEG`, `HEAD_SIT_DEG`, `HEAD_TUCK_DEG` |
| `feet`, `caps_deg` | `FEET_MIN_DISTANCE`, `KNEE_LIMIT_DEG`, `NECK_PITCH_MAX_DEG`, `HIP_YAW_INWARD_MAX_DEG` |
| `extra_collision`, `self_collision_pairs` | `EXTRA_COLLISION`, `SELF_COLLISION_PAIRS` |
| `home` (source "family") | `HOME_OVERRIDE` (keyframe `stand` qpos/ctrl'u ezer → bankalar, `default_*_q`, meta) |
| `mass.total_kg` | DR `trunk_added_mass` ölçeği |

Dosyayı ayrıca şunlar okur:
- `policy_meta` (`derived_source`, `derived_sha256`);
- `export_c` (`obs_layout.h`);
- eval (`derived_source`);
- RunPod başlatıcısı (`TR35_FILES`).

Kanıt: taze bir stub derlemesi üzerinde (`s16/p5a-integ/stub`) `derive --stub` 3.1 s sürdü. Çıktı commit'li dosyayla
yalnız `source.scene` satırında farklı; bu satır verilen yoldur. Commit'li dosya için `check` 0 döndü. Stub
derlemesi belirlenimci: openduck.xml sha256 `4e9ab5b8...`.

Stub'da türetilen değerler (derive raporundan, taze koşuda aynı):
- **HOME:** keyframe COM ayak merkezinin 19.9 mm gerisinde, bütçenin ([−17, +3] mm) dışında. Bu yüzden family HOME
  seçildi: hip_pitch −17.5, knee −10.5, ankle 7.0; COM −0.1 mm.
- **Yükseklikler:** H_HOME 0.1522, H_MIN_WALK 0.1125, H_MIN_STAND 0.0926, H_DEEP 0.0860, SEAT_Z 0.0840, Z_SIT 0.0865.
- **F tablosu:** 13 satır, 0.1594 → 0.0825.
- **Oturuş:** pitch −50 (hip_pitch 85, knee −5, ankle −40), uygun, pay 6.9 mm. HEAD_SIT = HEAD_TUCK: neck 23,
  head_pitch 55.5.
- **Ayak ve sınırlar:** FEET_MIN 0.0961 m; sınırlar KNEE 65, NECK 40, HIP_YAW_INWARD 10.
- **EXTRA ve çiftler:** 10 EXTRA kutusu, 30 çift.

## 4. Çözücü kararı

**Karar:** tr35 MJX Newton yinelemesi walk 8, main 8, getup 16; `ls_iterations` 8 (eğitici varsayılanı).
frictionloss, Adım 0 sysID'ye kadar spec'teki %3 stall değerinde kalır. xl330 değerleri değişmedi (walk 4, main 8,
getup 8).

Ölçümler P5a kapsam aşamasında stub üzerinde yapıldı (CPU-JAX, mujoco/mjx 3.16, DR ve gürültü açık). Kontrol adımı
süresi:

| Ortam | Görev | 4 yineleme | 8 yineleme | 16 yineleme |
|---|---|---|---|---|
| 256 | walk | 67 ms | 113 ms | 116 ms |
| 256 | main | 68 ms | 116 ms | 118 ms |
| 256 | getup | 99 ms | 171 ms | 177 ms |
| 2048 | walk | - | 632 ms | 686 ms (+%8.4) |
| 2048 | getup | - | 1231 ms | 1375 ms (+%11.7) |

Tek kontrol adımı hatası, 64/32 yinelemeli referansa göre (eklem qvel, frictionloss %3, DR):
- 4 yineleme yakınsamıyor: main p99 0.22 / en büyük 1.2 rad/s; getup 1.36 / 3.2 rad/s.
- 8 yinelemede p99 ≤ 1e-6, en büyük 2.5e-3 (main) ve 0.082 rad/s (getup; seyrek yapış-kay).
- 16 yinelemede en büyük ≤ 1.7e-4 rad/s.
- frictionloss %1 hız kazandırmıyor (walk 627 ms, getup 1227 ms; ikisi de 8 yinelemede).

Entegrasyonda gerçek JSON ile 256 ortamda yeniden ölçüldü (paylaşımlı makine, yük ~10, gürültü ±%12): 16'nın 8'e
göre maliyeti main'de ölçüm gürültüsü içinde (290 → 269 ms), getup'ta (30 çift ile) +%3.4 (570 → 589 ms). Yani getup
16'nın bedeli küçük. 4 yineleme tr35 için yakınsamadığından walk xl330'un 4'ünü kullanamaz.

## 5. Öz-çarpışma çifti maliyeti (spec A3)

Ölçüm `tools/mjx_solver_bench.py --mode time --tasks main,getup --iters 8,16 --pairs off,derived,main-subset
--n 256` ile yapıldı. Koşul: taze stub, gerçek `rl/derived/tr35_b2.json`, CPU-JAX, RL kilidi altında. Kontrol
adımı süresi (8 / 16 yineleme):

| Görev | Çiftsiz | Yapılandırılan (derived) | Fark | 8 çiftlik alt küme | nefc (çiftsiz → derived) |
|---|---|---|---|---|---|
| main | 164 / 148 ms | 290 / 269 ms (8 çift: main_pairs) | +%77 / +%82 | 257 / 286 ms | 110 → 238 |
| getup | 279 / 264 ms | 570 / 589 ms (30 çift: tam liste) | +%104 / +%123 | 327 / 338 ms (+%17 / +%28) | 270 → 750 |

core'un kendi ölçümü (bootstrap dosyası ve spec 3.2.2 aday listesi, 43 çift) şunu verdi:
- tam liste main'i +%310..330, getup'ı +%180..264 pahalılaştırıyor;
- 8 çiftlik alt küme main'de +%40..78;
- condim 1, tam listenin ek maliyetini yaklaşık yarıya indiriyor (main 170 → 429 ms, getup 326 → 449 ms).

Karar (P5a): main alt kümesi (uyluk/incik ↔ gövde, ayak ↔ aynı bacağın uyluk/kalça beşiği) kullanılıyor, çünkü tam
liste +%25 eşiğini çok aşıyor. getup tam listeyi kullanıyor, çünkü yuvarlanmada kafa/gaga ↔ gövde ve bacak ↔ bacak
temasları gerekli. getup'ın yaklaşık 2 katı maliyeti P5b'de GPU'da (8192 ortam) yeniden ölçülür. Seçenekler: tam
liste, alt küme + kafa/gaga ↔ gövde, condim 1.

## 6. Duman sonuçları

`tools/tr35_smoke.py` taze stub derlemesi ve gerçek `rl/derived/tr35_b2.json` ile koştu (`OPENDUCK_DERIVED`
verilmedi, varsayılan yol kullanıldı). Ayarlar: CPU-JAX MJX + Brax PPO, 32 ortam, 2e4 adım, 2 eval, `OPENDUCK_BANK_N`
= 64. Her koşu RL kilidini ayrı aldı (`--lock`). Komut:

```
JAX_PLATFORMS=cpu <mjx py> tools/tr35_smoke.py --out <scratch>/smoke --scene <scratch>/stub/sim/scene.xml \
    --lock <scratch>/locks/rl --label p5a-integ-smoke
```

| Koşu | rc | Süre | Eval ödülü | Ort. bölüm uzunluğu | sps | C ↔ ORT | ESP32-S3 tahmini |
|---|---|---|---|---|---|---|---|
| walk | 0 | 77.5 s | −14.82 | 42.4 | 611 | 2.9e-6 | 1.47 ms |
| main | 0 | 147.2 s | −14.41 | 85.8 | 310 | 4.8e-6 | 1.49 ms |
| getup (free) | 0 | 172.6 s | −3.92 | 600 | 256 | 1.9e-6 | 1.47 ms |
| getup `--getup-head fixed` | 0 | 187.2 s | −6.92 | 600 | 233 | 2.9e-6 | 1.47 ms |

Doğrulananlar:
- Dört koşunun hepsinde log metrikleri sonlu, `policy.onnx` ve `policy_meta.json` yazılmış, meta boyutları doğru
  (49/10, 53/10, 48/14).
- Her koşuda `<run>/c/` yazılmış ve 1000 gözlemde onnxruntime ile eşit (≤ 1e-5).
- Meta'da `robot` tr35_b2, `derived_sha256` commit'li dosyanın sha256'sı (`b7e6901c...`), kappa 0.5.
- main meta'sında `seat_pitch_deg` −50; getup fixed'te `head_tuck_deg` (neck 23, head_pitch 55.5) ve
  `OPENDUCK_HEAD_TUCK_RAD`.
- `default_obs_q` family HOME'u gösteriyor (hip_roll −5°, hip_pitch −17.5°, knee −10.5°, ankle 7°).

core'un bootstrap dosyasıyla yaptığı önceki duman koşusu walk 99 s, main 91 s, getup 100 s sürmüştü. O dosyada
öz-çarpışma çifti yoktu. Bu koşuda main ve getup'ın daha yavaş olması 8 / 30 çiftin maliyetiyle (bölüm 5) ve
paylaşımlı makinenin yüküyle uyumlu.

Test takımları (entegrasyon dalı, her biri ayrı ve RL kilidi altında):
- **CPU:** 156 geçti / 7 atlandı. Baseline 119 / 5. Yeniler: core 13 (+2 atlanan MJX), derive 5, export_c 7,
  eval_tr35 12.
- **MJX:** 172 geçti / 4 atlandı. Baseline 135 / 2; +37 geçti, +2 torch'suz atlandı.
- **`tests/cad` + `tests/golden`:** 72 geçti; kilit gerekmedi.

Ek ölçümler:
- **HOME 5 s (A3):** açık döngü, stub üzerinde eğim boşluksuz 1.16°, boşlukla 2.92°; düşme yok.
- **Eşli koşu (çiftsiz):** main ödül farkı 1.5e-6, getup 1.5e-6.
- **Sıfır politikalı tr35 eval dumanı** (`test_eval_tr35_zero_policy_smoke`, `--n-scale 0.01`): A4-A8 bölümlerinin
  hepsi "ok", yaklaşık 35 s.

## 7. P5b kontrol listesi (G-CAD'den sonra)

Sıra önemlidir; her madde bir öncekinin çıktısını kullanır.

1. **P3 tr35 derlemesini `build/tr35_b2/sim/` altına yazar.** Gerekenler: `openduck.xml`, `scene.xml`, `meshes/`.
   Bu yol `C.SCENE_PATHS["tr35_b2"]` ve başlatıcının `TR35_FILES` listesindeki varsayılandır. P3 başka bir yer
   seçerse yalnız bu iki giriş değişir. Ondan sonra hiçbir tr35 sürecinde `OPENDUCK_SCENE` gerekmez. Gerekli
   tr35 MJCF dosyaları commit'lenir (RunPod `--source git`).
2. **Son MJCF'de yeniden türetme.**
   - `python tools/derive_posture.py derive --scene build/tr35_b2/sim/scene.xml --out rl/derived/tr35_b2.json`
     (`--stub` olmadan) çalıştırılır, ardından `check` 0 dönmelidir. Dosya commit'lenir.
   - HOME kaynağı (keyframe / family) son COM'a göre kendiliğinden seçilir. P2b COM'u bütçeye getirirse keyframe
     HOME'a döner ve bütün sabitler değişir. G-ISIL tutmazsa HOME değişir ve bu adım tekrarlanır.
   - `caps-scan` son meshlerde koşulur. KNEE / NECK / HIP_YAW sınırları ham ctrlrange'den dar olmalı mı, karar
     verilir. EXTRA kutuları gerçek CAD'den yeniden çıkar.
   - Ardından `rl/tests/test_derive_posture.py`, `rl/tests/test_tr35_core.py` (bu kez oturuş bankası penceresi
     zorunlu, `stub = false`) ve `rl/tests/test_eval_tr35.py` koşulur.
3. **P2c'nin jaw ↔ neck / trunk çiftini ve genişletilmiş kafa sınırlarını işle** (`p2c-tampon.md` §5, §9).
   - Genişletilmiş sınırlar: neck −110..40, head_pitch −25..72 (MJCF aralığı 75, çene tampona 76'da dayanır),
     head_roll −20..16, mouth 0..32.
   - `derive_posture.candidate_pairs` listesine jaw ↔ neck eklenir (`neck` EXTRA kutusu zaten var; jaw ↔ trunk
     listede). neck_bumper ile P3'ün dışa aktardığı MJCF çiftleri uzlaştırılır; RL listesi o zamana dek yetkilidir.
   - Yeni sınırlarla yeniden türetilir: NECK_PITCH_MAX, oturuş baş ızgarası ve HEAD_SIT/TUCK. Stub'da head_pitch
     55.5, 33°'deki ilk görsel girişimin ötesinde.
   - HEAD_RANDOM aralıklarının ve `slew_max` iddiasının yeni sınırlar içinde kaldığı kontrol edilir.
   - Gerekirse "head_pitch > 50 iken mouth ≤ 25" kilidi politika ya da firmware tarafında kararlaştırılır.
4. **Çözücü GPU'da 8192 ortamla yeniden ölçülür.** `tools/mjx_solver_bench.py --mode all --iters 8,16 --n 8192`
   son MJCF ve yeni JSON üzerinde, walk/main/getup için koşulur.
   - 16'nın 8'e maliyeti %25'i aşarsa getup için 12 denenir.
   - Aynı koşuda öz-çarpışma çifti maliyeti de ölçülür (`--pairs off,derived,main-subset,spec`, `--pair-condim 1`).
     getup için tam liste / alt küme + kafa ↔ gövde / condim 1 kararı verilir.
   - G-BENCH sysID frictionloss'u belirleyince ölçüm tekrarlanır.
5. **A3, son MJCF'de** (spec 6):
   - **HOME 5 s:** açık döngü 5 s ayakta, eğim ≤ 2° (`test_tr35_home_stands_5s`). Stub'da boşluksuz 1.16°,
     boşlukla 2.92° ölçüldü (boşlukta düşmüyor; ölçüt boşluksuz).
   - Model MuJoCo'da ve MJX'te yüklenir; 4 aktüatör sınıfı var; varyant sınırları gerekli aralıkları kapsar ve +3°
     değişmezi test edilir.
   - CPU ile MJX karşılaştırması: tek adım atol 1e-4 ve 50 adımlık eşli koşu
     (`test_cpu_mjx_lockstep_parity_tr35`, `test_tr35_lockstep_cpu_mjx`).
   - Öz-çarpışma çiftleriyle eşitlik ele alınır: MJX kutu-kutu çift başına 4 nokta, C 1-2 (bölüm 2 madde 10).
     Seçenekler:
     - EXTRA uyluk/incik geomlarını kapsüle çevirmek (şema `capsule`'ü destekliyor; derive `extra_boxes`);
     - kutuları son CAD'de HOME, oturuş ve düşmüş durumlarda gövdeye değmeyecek kadar küçültmek;
     - A3 eşitliğini çiftsiz modelde tutup çiftli farkı ayrıca raporlamak.
     Sonra `test_tr35_lockstep_cpu_mjx` çiftler açıkken koşulur.
   - Öz-çarpışma MJX maliyeti raporlanır (madde 4).
6. **Poz bankaları son MJCF'de yeniden doğrulanır.**
   - Bakılacaklar: zemine batma, kafa-yalnız reddi, oturuş kabul oranı ve oturuş pitch/z penceresi (stub'da
     marjinaldi, bölüm 2 madde 3). Kurulum süreleri kaydedilir.
   - Oturuş hâlâ devriliyorsa iki yol var: derive oturuş aramasına ±tilt fiziksel oturtma ölçütü eklenir, ya da
     `seated_tilt_deg` daraltılır.
7. **Duman ve küçük eğitimler, son model ve yeni sabitlerle.**
   - `tools/tr35_smoke.py` yeniden koşulur.
   - Ardından küçük CPU-JAX ve boru hattı boyutunda walk, main, getup eğitimleri yapılır (P7 girdisi).
   - Orta N'li bir tr35 eval koşulur (örneğin `--n-scale 0.125`, kilit altında); A4-A6'nın tam ölçekli CPU süresi
     tahmin edilir.
8. **Başlatıcı.** `launch.py launch --dry-run --robot tr35_b2 --source git` gerçek dosyalarla (YOK işareti olmadan)
   koşulur. Satır içi gövde RunPod sınırının üstünde olduğundan P7 `--source git` kullanır.
9. **Ödül ağırlıkları ve G-BENCH sonrası** (P5b'nin içinde değil, P7 / G-BENCH):
   - limit_torque, rail_demand ve torque_ratio ilk ağırlıkları ayarlanır.
   - Adım 0 tablosundan KAPPA (`OPENDUCK_KAPPA`) ve T_diz (`--t-knee`) alınır.
   - DR ölçülen değerin ±%15'ine daraltılır; `actuators.CLASSES` sysID'den güncellenir.
   - rl-map D1'deki eklem başına hedef eğim sınırı kararlaştırılır.
   - A7 ESP32-S3 aktivasyon çevrimleri kartta ölçülür.
