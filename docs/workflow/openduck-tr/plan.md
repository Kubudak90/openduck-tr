# Plan: OpenDuck-TR v1

Girdi: `spec.md` (v3.x), `review.md`, `spec-ek-komut.md` (ek K), `review-komut.md`. Plan spec dışında tasarım kararı içermez; yeni ihtiyaç çıkarsa spec güncellenir.
Repo: `Kubudak90/openduck-tr`, dal `main`. Her adım kendi dalında/worktree'sinde çalışır; ben birleştiririm.
Model: tasarım, geometri, RL modelleme ve inceleme Opus; arama, stok doğrulama ve mekanik düzenleme Sonnet.

## Bağımlılık grafiği

```
P0 ─► P1 ─► P2a ┐
            P2b ├─► P3 ─► [G-CAD] ─► P5 ─► P7* ─► [G-BENCH]◄── Adım 0 (kullanıcı) ◄── P6
            P2c ┘                    ▲                │
P0 ─► P4 ────────────────────────────┘                ▼
                                             [G-ISIL] ─► P8* ─► P9* ─► P10 ─► P11
P0 ─► P6 (öncelikli: Adım 0'ı açar)
```
`*` RunPod: kullanıcı ön onaylı (2026-10-23/24, ön koşullar ve testler tamamsa). Canlı fiyat okunur, saat üst sınırı konur, iş bitince pod durdurulur, harcama bildirilir.

## Adımlar

| # | Adım | Sahip dosyalar | Girdi | Çıktı | Test / bitti |
|---|---|---|---|---|---|
| P0 | Altın referans | `tests/golden/` (yeni) | mevcut micro `build/` | MJCF, summary, interference ve URDF sağlamaları + kopyaları | `pytest tests/golden` geçer |
| P1 | CAD refactor I: servo kayıt defteri, ölçek, konfigler, modüllere bölme (`cad/trunk.py`, `cad/legs.py`, `cad/head.py`) | `cad/servos.py`, `cad/openduck.py`, `cad/{trunk,legs,head}.py`, `cad/export.py` (sınıf başına aktüatör), `configs/*.json` | P0, research/cad-map | xl330_27 → bayt-eşit MJCF/summary/interference | Altın test bayt-eşit; `build.sh --check` (xl330) aynı rapor |
| P2-0 | Arayuz dondurma: PWM servo zarflari (DS32xx/ES3054/MG90S/SG90 kanonik, idler, doner tabla), TR-35 yerlesimi (j_*, servo_face/dir, HOME), zarf yerlesim calismasi, modul sozlesmesi, yer tutucu uretecler, genisletilmis carpisma taramasi | `cad/servos.py`, `cad/geom.py`, `cad/openduck.py`, `cad/check.py`, `cad/build_config.py`, `cad/export.py` (malzeme listesi), `cad/electronics.py` (yeni, sonra P2b), `cad/{trunk,legs,head}.py` (yer tutucular), `configs/tr35_b2.json`, `tools/fit_study.py`, `tests/cad/`, `p2-interface.md` | P1, servo-mechanics, power-electronics | `p2-interface.md` + donmus konfig | tr35_b2 yer tutucularla derlenir, MJCF MuJoCo'da yuklenir, `check.py` calisir; zarf calismasi 0 capraz-link carpisma; kafa ustu 350 +- 10; altin 6/6; `tests/cad` gecer |
| P2a | TR-35 bacaklar: DS beşik/U-link/idler, hip_yaw döner tabla + kaplin, hip_pitch ofset, ayak, kablo demetleri | `cad/legs.py`, `cad/servos.py` (DS/ES zarfları) | P1, servo-mechanics | Birleşti (`p2a-el-hesabi.md`, `p2-entegrasyon.md`) | Tamam; tur 2'de diz geçişi dış tarafa (D1) |
| P2b | TR-35 gövde: trunk, elektronik zarfları, LiPo, buck havalandırması, boyun DS yuvası (trunk'ta), güç kesme yeri | `cad/trunk.py`, `cad/electronics.py` (yeni) | P1, power-electronics | Birleşti (`p2b-hacim.md`, `p2-entegrasyon.md`) | Tamam |
| P2d | Ses modülü yerleşimi (ek K3): kafa kubbesi, conta/beşik, LED ve pencereler, demet ICR'si (5 iletken), 30 g kütle aktarımı, MG90S kontrolü | `cad/head.py`, `configs/tr35_b2.json` (params.head, harness), K3.6'daki P2b/P2c demet dosyaları | Dalga 2 entegrasyonu, ek K, (varsa) G-SES-a ölçüleri | Kafada modül | Tam check.py 0 sayılan (head_yaw geçiş kusuru dahil, `p2-entegrasyon.md` §10.3), tests/cad, altın 6/6, head_yaw ±80 burulma temiz; P3'ten önce |
| P2c | TR-35 boyun + kafa: boyun U-link (nötr −25°), MG90S zinciri, SG90 gaga, çene/alın tamponu, sabit etli kabuklar | `cad/head.py` | P1 | Birleşti (`p2c-tampon.md`, `p2-entegrasyon.md`) | Tamam; tur 2'de head_yaw tutucusu (D3); head_yaw kablo geçişi P2d'ye (§10.3) |
| P3 | Montaj + dışa aktarma: tr35_b2 build; öz-çarpışma çiftleri; tampon geomu; BOM (stok yeniden doğrulama, Sonnet); baskı seti/teknik resim; aşağıdan yukarı kütle + kütle merdiveni | `cad/export.py`, `cad/docs.py`, `cad/bom_tr.json` (yeni), `build/` | P2a–c | `build/` (STEP, STL, print, drawings, BOM, MJCF) | A1, A2; `build.sh --check` |
| G-CAD | Kapı: A1 + kütle ≤ 1.95 kg; trunk/bacak/COM donar | — | P3 | `docs/workflow/openduck-tr/gcad.md` | Tutmazsa kütle merdiveni; hâlâ tutmazsa spec'e dön |
| P4 | RL aktüatör + sensör modeli (CPU ve MJX): `rl/actuators.py`, `rl/sensors.py`, build_spec +3° değişmezi, ölü bant, boşluk eklemi, besleme akımı/ray modeli, IMU + tamamlayıcı filtre, kafa potları; parity testi | `rl/actuators.py`, `rl/sensors.py`, `rl/robot_model.py`, `rl/env_cpu.py`, `rl/env_mjx.py`, testler | P0 (geçici TR modeli ya da xl330 ile geliştirilir) | Modeller + birim testler | A3 (parity, değişmez), A9 birim testleri |
| P5a | P5'in geometriden bağımsız kısmı (yer tutucuya karşı): türetme aracı, tr35 görev profili, cezalar, eval/export/meta | rl/, `tools/derive_posture.py`, `rl/derived/` | P4, P2-0 | Birleşti (`p5a-notlar.md`) | Tamam (CPU 156, MJX 172) |
| P5k | Sesli komut katmanı (ek K5–K7): `rl/commands.py`, tablo testleri, altın izler, jest fonksiyonu, komut atomları, A10 düzeneği ve eval iskeleti | `rl/commands.py` (yeni), `rl/task_math.py`, `rl/env_cpu.py`, `rl/env_mjx.py`, `rl/eval_tasks.py`, `rl/play.py`, `configs/komut_kimlik.json` (örnek), testler | P5a, ek K | Birleşti (`p5k-notlar.md`) | Tamam (CPU 184, MJX 221; lockstep ve A10 dumanı stub'da) |
| P6e | Tezgâh eki (ek K8): DF2301Q UART okuma, kimlik zaman damgası, çok kanallı jest/salınım (MG90S/SG90 50 Hz), piezo senkron tık, uyanık durum kaydı, öğrenme modu, pin haritası; kılavuz eki | `tools/bench/`, `docs/adim0-tezgah-testi.md` | P6, ek K | Birleşti | Tamam (testler 20/20; bench ve bench_usb derlenir) |
| P5 | TR-35 görev sabitleri ve görevler: türetme aracı (family3 portu), H_*, F, SEAT, HEAD_SIT, HEAD_RANDOM, κ/gait/komutlar, cezalar, getup stratejisi + bankalar; eval/play güncellemesi (metrikler, mod başına, devir, Wilson); C dizisi dışa aktarma + host eşitlik kontrolü | `rl/config.py`, `rl/tasks.py`, `rl/task_math.py`, `rl/pose_bank.py`, `rl/eval_tasks.py`, `rl/play.py`, `rl/export_c.py` (yeni), `tools/derive_posture.py` (yeni) | G-CAD, P4 | Eğitilebilir görevler | Küçük CPU-JAX eğitimleri 3 görevde uçtan uca; A3 HOME 5 s ayakta; testler |
| P6 | Tezgâh + sysID araçları: ESP32 betiği (PWM süpürme, ADC/INA226/ADS1115 kaydı, basamak/chirp, ısınma döngüsü), PC kaydedici, sysID uydurma (sentetik veriyle testli), Adım 0 kullanım kılavuzu (Türkçe) | `tools/bench/`, `tools/sysid/` | spec §7 | Betik + kılavuz + uydurucu | Sentetik log'dan parametreleri ±%5 geri bulur; kılavuz |
| P7* | Boru hattı eğitimi (ön sürüm): walk + main (jest + komut atomlu) + getup, varsayılan parametreler, κ 0.5 (kabul değil); A10 ön sürüm | — | P5b, P5k | `runs/pipe-*` | Eğitim tamamlanır; G-ISIL için diz RMS ölçülür |
| G-SES-a | Adım 0 eki (kullanıcı): modül ölçü/akım, tanıma kapısı (ek K8) | — | P6e, Adım 0 alımı | `gses.md` | Tutmazsa yedek yol (ek K3) |
| G-BENCH | Adım 0 verisi → P6 sysID → `rl/actuators.py`; κ ve servo seçimi | — | Adım 0 + P6 | `gbench.md` | §7 karar tablosu |
| G-ISIL | Diz RMS (P7) × tezgâh τs × G-CAD kütlesi ≤ T_diz | — | P7, G-BENCH | `gisil.md` | Tutmazsa HOME değişikliği + P5 yeniden türetme ya da DS3235 |
| P8* | Getup yapılabilirlik kapısı (§3.4) | — | G-BENCH | `gate-getup.md` | Mod eşikleri; tutmazsa spec'e dön |
| P9* | Kabul eğitimleri: walk, main, getup (G-BENCH parametreleri, daraltılmış DR) | — | G-ISIL, P8 | `runs/accept-*` | — |
| P10 | Kabul değerlendirmesi A4–A8, A10, A7 dışa aktarma, videolar | `runs/*/eval.json` | P9 | Rapor | Spec §6 |
| P11 | Bağımsız kod incelemesi (bulgu → doğrulama) + düzeltmeler; README | — | P10 | — | Doğrulanmış kritik bulgu yok |

## Paralellik ve worktree'ler

- Önce P0. Ardından P1, P4 ve P6 paralel çalışır: dosya kümeleri ayrık (cad/ – rl/ – tools/).
- P2a, P2b ve P2c, P1'deki modül bölmesinden sonra paralel ve ayrık dosyalarda yürür. Ortak `cad/servos.py`'ye yalnız
  P2a ekleme yapar; diğerleri okur.
- P2a, P2b ve P2c, P2-0'ın dondurduğu arayüzden başlar (`p2-interface.md`, `configs/tr35_b2.json`).
- P5b (P5'in geometriye bağlı kısmı), G-CAD ve P5a'dan sonra başlar. P5k ve P6e şimdi, P5a üstünde paralel yürür.
- P2d, dalga 2 entegrasyonundan sonra ve P3'ten önce gelir. G-SES-b robot kurulunca yapılır (ek K2).
- Hızlı yol (kullanıcı kararı, 2026-10-24): P7 ön sürüm eğitimleri G-CAD beklenmeden dalga-2 CAD'i üzerinde başlar.
  P5b'nin geometriye bağlı ilk maddeleri bunun için yapıldı (P5b-ön, `p5b-notlar.md`). P2d ve P3'ten sonra P5b-son
  sabitleri yeniden türetir; P9 kabul eğitimleri G-BENCH'i bekler.

## Riskli adımlar (insan onayı)

- P7, P8, P9: para harcar. Kullanıcı ön onay verdi (2026-10-23/24). Canlı fiyat, saat üst sınırı ve harcama bildirilir; P8/P9 için G-BENCH beklenir.
- G-CAD, G-ISIL ve P8'de "spec'e dön" sonucu çıkarsa kullanıcıya özet ve seçenekler sunulur.
