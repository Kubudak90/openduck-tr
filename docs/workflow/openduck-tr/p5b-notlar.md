# P5b-ön: RL sabitleri dalga-2 CAD'inden (P7 ön sürüm eğitimleri için hızlı yol)

Kullanıcı kararı (2026-10-24): RunPod ön sürüm eğitimleri (P7: walk, main, getup) P2d, P3 ve G-CAD beklenmeden,
güncel CAD üzerinde başlar. P5b kontrol listesinin (`p5a-notlar.md` §7) geometriden bağımsız olmayan ilk maddeleri
burada yapıldı. Kafaya +30 g (ses modülü, P2d) ve P3'ün MJCF çiftleri gelince türetme tekrarlanır; kabul eğitimleri
(P9) zaten G-BENCH'i bekler.

Dal: `p5b-on` (main 3455046 üstünde).

## 1. Yapılanlar (kontrol listesi maddeleri)

| Madde | Durum |
|---|---|
| 1. `build/tr35_b2/sim/` (openduck.xml, scene.xml, meshes) | Main'in CAD'inden derlendi ve commit'lendi (6.8 MB). `C.SCENE_PATHS` ve başlatıcının `TR35_FILES` varsayılanı |
| 2. Son MJCF'de yeniden türetme | `derive` + `check` 0. Tablo §2 |
| 2. `caps-scan` | §3 |
| 3. jaw ↔ neck çifti, genişletilmiş kafa sınırları | `derive_posture.candidate_pairs` spec 3.2.2 v3.2'ye göre: jaw ↔ neck, kafa ↔ kalça/kalça beşiği. Sınırlar MJCF'den okunur (neck −110..40, head_pitch −25..72, head_roll −20..16, mouth 0..32) |
| 4. GPU çözücü / çift maliyeti (8192) | RunPod'un ilk koşusunda (`tools/mjx_solver_bench.py`) |
| 5. A3 | HOME 5 s açık döngü: eğim **1.17°** (boşluksuz, ≤ 2°); boşlukla 2.88°, düşme yok. CPU/MJX eşliği §4 |
| 6. Poz bankaları | `test_tr35_small_banks` gerçek sahnede, oturuş penceresi zorunlu (`stub = false`): banka pitch −53.7..−44.6, z 84.4..90.6 mm, 52 aday |
| 7. Duman eğitimleri | §5 |
| 8. Başlatıcı | `launch --dry-run --robot tr35_b2 --source git`: gövde 13.1 KB. Satır içi paket 140.8 KB, sınırın üstünde; `git` kullanılır |

## 2. Türetilmiş sabitler (`rl/derived/tr35_b2.json`, gerçek CAD)

| Sabit | P2-0 stub | Dalga-2 CAD |
|---|---|---|
| Kütle | 1.848 kg | 1.775 kg |
| HOME | family | family. Keyframe COM ayak merkezinin 19.8 mm gerisinde; family HOME: hip_pitch −17, knee −10, ankle 7, COM payı 34.6 mm |
| H_HOME / H_MIN_WALK / H_DEEP | 0.1522 / 0.1125 / 0.0860 | 0.1519 / 0.1123 / 0.0879 |
| Oturuş | pitch −50, knee −5, ankle −40 | pitch **−47.5**: hip_pitch 85, knee 71, ankle 33.5; seat_gz −0.287 |
| HEAD_SIT / TUCK | neck 23, head_pitch 55.5 | neck **−102**, head_pitch −22 |
| KNEE cap | 65 | 77 (CAD sınırı) |
| Öz-çarpışma çifti | 30 | 46 (HOME temaslı 0, eksik geom 0) |

Uyarılar:
- `Z_SIT` 88.2 mm ≥ `H_DEEP` 87.9 mm: oturuş gövde yüksekliği derin çömelmeden alçak değil.
- Oturuşta ayak kutusu eğimi 1.63°.
- Keyframe HOME COM bütçe dışında; bu yüzden family HOME seçildi.

Gerçek CAD'de oturuşta COM ayakların üstüne düşer: gövde kutusu yük taşımaz, zeminden ~0.3 mm yukarıda durur. Poz
kararlıdır (üç oturtmada pitch −47.47°). Bankalar ve A5 bunu kullanır. P5b-son'da oturuş aramasına "gövde yük taşısın"
ölçütü eklenip eklenmeyeceğine karar verilir.

## 3. caps-scan (görsel mesh voksel, 0.8 mm)

Tek eklem süpürmeleri ve aile/oturuş pozları:
- Aile satırları 0.0943 ve 0.0879 (knee 71): `hip_pitch_harness_in` ↔ `shin_bracket` 2.0 / 0.5 mm³. A1 ızgarası knee'yi gerekli
  aralıkta (≤ 65) tarar; bu iki derin satır onun dışındadır. P3'te uyluk kablosu derin dizde inciğe değmeyecek
  biçimde düzeltilir ya da aile diz 68 ile sınırlanır (A8 eşiği 1 mm³).
- head_roll −14 / +13 (tutucu yay ↔ head_frame) voksel artefaktıydı: kesin BRep açıklığı sınırda 0.85 mm
  (`p2-entegrasyon.md` §10.4). Yaylar düzeltildikten sonra kalan tek nokta MJCF aralık ucudur (+19: 0.28 mm³).
- Diz −84 / +77, hip_yaw −10 / +20: ilk girişim sınırda. CAD sınırları tek eklem taramasından gelir.
- Oturuş (HEAD_SIT dahil): girişim yok.

## 4. Testler (gerçek sahne)

Değişen beklentiler (stub'a özgüydü):
- Testler commit'li sahneyi kullanır (`rl/tests/tr35_scene.py`, `test_derive_posture.py`). `OPENDUCK_TR35_SCENE=stub`
  stub derlemesini zorlar.
- Türetilmiş dosyanın `stub` bayrağı artık false. `neck_bumper` geomu var; getup çift sayısı +1.
- Tam oturuş: gövde kutusunun en alçak köşesi zemine 1 mm içinde (margin 0'da temas kaydı yok).
- Jest kanalı: bölüm sıfırlanan adımlar zamanlayıcı kontrolünden çıkarılır. Koşu 16 ortamla yapılır; tut ve atom
  segmentleri görülür.
- CPU/MJX jest+atom eşliği (`test_tr35_lockstep_cpu_mjx_gestures_atoms`): aktüatör komutu, jest durumu ve segment
  her adımda kesin karşılaştırılır. İki sapma kaynağı ayrı sayılır ve o ortam-adımın ödül/gözlem karşılaştırması
  atlanır:
  - **PWM yuvarlama:** `quantize_pwm`'de nicemleme öncesi değer x.5 sınırına float32 hassasiyetinden yakın düşerse
    CPU (float64) ve MJX (float32) tam bir adım farklı yuvarlar. Örnek: DS'de 0.072° (23 ↔ 24 sayım). 280
    ortam-adımında 3 kez görüldü, sınır ≤ 4.
  - **Temas uyuşmazlığı:** adım sonunda iki motorun aktif temas sayısı farklıdır (temas açılıp kapanırken). 280'de 1
    kez görüldü, sınır ≤ 28. Kalan adımlarda ödül 3.5e-5, kritik 1.4e-4.

Sonuçlar:
- CPU venv `rl/tests` (`test_env_mjx` hariç): **185 geçti**, 8 atlandı (jax).
- MJX venv (`test_env_mjx`, `test_tr35_core`, `test_voice_train`, aktüatör, sensör, task_math): **99 geçti**
  (eşlik düzeltmesinden sonra).
- Komut altın izleri yeni türetilmiş dosyayla yeniden üretildi (`tools/commands_golden.py`).

## 5. Duman eğitimleri (`tools/tr35_smoke.py`, CPU-JAX, 32 ortam, 2e4 adım)

| Koşu | rc | Süre | Eval ödülü | Bölüm uzunluğu | sps |
|---|---|---|---|---|---|
| walk | 0 | 295 s | −22.0 | 143.5 | 196 |
| main | 0 | 377 s | −11.4 | 93.0 | 150 |
| getup | 0 | 505 s | 6.37 | 600 | 89 |
| getup `--getup-head fixed` | 0 | 460 s | −13.3 | 600 | 90 |

Hepsinde metrikler sonlu, `policy.onnx` + `policy_meta.json` yazıldı ve meta boyutları doğru (49/10, 53/10, 48/14).

## 6. Kalan P5b maddeleri (P5b-son, P2d/P3 sonrası)

- Kafa +30 g (ses modülü) ve P3'ün MJCF çiftleri / kabloyu kapsayan çarpışma geomları gelince: derleme, `derive`,
  `check`, `caps-scan`, altın izler ve testler tekrarlanır.
- Öz-çarpışma çiftleri açıkken CPU/MJX eşliği: kutu-kutu temas noktası farkı (`p5a-notlar.md` §2 madde 10).
- Derin aile satırlarında uyluk kablosu ↔ incik (§3).
- Orta N'li tr35 eval ve A4–A6 tam ölçekli CPU süresi.
