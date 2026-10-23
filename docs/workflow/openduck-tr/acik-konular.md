# Açık konular (sonraki adımlara devredilen)

Kaynak: P1/P4/P6 birleştirme notları, P2-0 raporu (`p2-interface.md` §12). Her madde, sahibi olan adım
tamamlanınca kapatılır.

## P3: dışa aktarma, BOM, baskı seti

- `cad/docs.py`:
  - XL330 metinleri duruyor.
  - Yatak kontrolü 180 mm; 256 mm olmalı.
  - `harness` türünün baskı satırı yok.
- BOM servo satırının ifadesi tr35'e göre yazılmalı.
- `build/` içindeki eski `left/right_ankle_bearing.stl` dosyaları düşmeli.
- MJCF tarafı:
  - Öz-çarpışma çiftleri: spec §3.2.2 listesi (v3.1) ile `check.contact_pairs`.
  - `neck_bumper` geomu. `neck`, `collision_links`'e eklenmez.
  - `scene.xml` statistic değeri yeniden ölçeklenmeli.
- 6800ZZ'nin Türkiye stoğu doğrulanmadı. Stok yeniden doğrulaması Sonnet'le yapılacak.

## G-CAD

- Yer tutucu kütle aşağıdan yukarı 1847.9 g; ×1.05 ile 1940 g, sınırın 10 g altında.
- Gerçek geometriyle aşılırsa kütle merdiveni uygulanır (spec §3.1).

## P5: TR-35 görevleri

P5a (geometriden bağımsız kısım, P2-0 stub'ına karşı) birleşti; ayrıntı ve P5b kontrol listesi: `p5a-notlar.md`.

- ~~MJX çözücü iterasyonu: tr35 sürtünmesi yaklaşık 16 iterasyon istiyor, bugün 4/8; maliyet yaklaşık 1.9 katı.
  Seçenek: frictionloss'u stall'un %3'ünün altına çekmek.~~ (P5a: done) Karar walk/main 8, getup 16, ls 8,
  frictionloss %3. 16'nın 8'e maliyeti 256 ortamda +%2-3, 2048'de +%8-12. frictionloss %1 hız kazandırmıyor.
- ~~Poz bankaları tr35 için yeniden üretilecek.~~ (P5a: done) Bankalar ortamın robot'u ve sahnesiyle kuruluyor,
  oturuş `SEAT_PITCH_DEG`'de, aday sınırı var.
- ~~Görev matematiği `tau_ratio`, `limit_torque` ve `rail_demand` kullanmalı; ham `actuator_force` ve sabit 0.96 N·m
  değil.~~ (P5a: done) tr35 ödülleri: `torque_ratio`, `limit_torque`, `rail_demand`; getup `torque_sat` oranlarla.
  eval aynı cezaları raporluyor.
- ~~`policy_meta` sensör ve aktüatör sabitlerini dışa aktarmalı.~~ (P5a: done) Meta'da actuators, sensors, delays,
  derived_source/sha256 ve kappa var. `export_c` bunları `obs_layout.h`'ye de yazıyor.
- ~~`rl/TASK.md` dosyasına `robot=` ve `backlash=` belgeleri eklenecek. Boşluk açıkken `s['q']` servo açısıdır.~~
  (P5a: done)
- ~~`rl/config.py` tr35 sabitleri (`H_HOME`, `EXTRA_COLLISION`, `FEET_MIN_DISTANCE`, ...) hâlâ 27 cm değerlerinde.
  Yeni değerler: HOME ayak aralığı 133.4 mm; trunk z 152.5 mm; taban kutusu genişliği `foot_w` 64 mm.~~ (P5a: done)
  Hepsi `tools/derive_posture.py`'nin yazdığı `rl/derived/tr35_b2.json`'dan geliyor; tr35 için elle yazılmış
  geometri sayısı yok.
- ~~HOME kütle merkezi ayak merkezinin 19.9 mm gerisinde (yer tutucuyla; P2b iyileştirir). Kalan fark HOME yeniden
  türetmesiyle kapatılır.~~ (P5a: done) Stub'da family HOME seçiliyor (diz −10.5°, COM −0.1 mm).
- ~~Yeni öz-çarpışma çiftleri (kafa ↔ trunk; ayak ↔ aynı bacağın uyluk/kalça beşiği) RL çarpışma listesine girer.~~
  (P5a: done) Türetilmiş listede 30 çift var. main 8 çiftlik alt kümeyi kullanıyor (+%77-82), getup tam listeyi
  (+%104-123, CPU-JAX 256 ortam).
- P5b: P3 tr35 derlemesini `build/tr35_b2/sim/` altına yazar (`C.SCENE_PATHS`, başlatıcı `TR35_FILES`); tr35 MJCF
  commit'lenir.
- P5b: Son MJCF'de yeniden türetme yapılır (`derive` + `check` 0, `caps-scan`). HOME kaynağı son COM'a göre
  seçilir. EXTRA kutuları gerçek CAD'den yeniden çıkar.
- P5b: P2c'nin jaw ↔ neck / trunk çifti ve genişletilmiş kafa sınırları işlenir: neck −110..40, head_pitch −25..72,
  head_roll −20..16, mouth 0..32. Ardından NECK cap ve HEAD_SIT/TUCK yeniden türetilir. RL çift listesi P3 /
  neck_bumper ile uzlaştırılır.
- P5b: Çözücü ve çift maliyeti GPU'da 8192 ortamla yeniden ölçülür (`tools/mjx_solver_bench.py`). getup için tam
  liste / alt küme / condim 1 kararı verilir. G-BENCH frictionloss'undan sonra ölçüm tekrarlanır.
- P5b: A3 son MJCF'de koşulur: HOME 5 s (eğim ≤ 2°; stub'da 1.16°), CPU/MJX tek adım ve 50 adımlık eşli koşu, 4
  sınıf, +3° değişmezi.
- P5b: CPU/MJX eşitliği öz-çarpışma çiftleriyle bozuluyor. MJX kutu-kutu teması çift başına 4 nokta, C 1-2 nokta
  üretiyor; stub bankalarında uyluk/incik ↔ gövde temasları var. tr35 eşli testi şimdilik çiftsiz koşuyor. Seçenekler:
  EXTRA geomlarını kapsüle çevirmek, kutuları küçültmek ya da çiftli farkı ayrıca raporlamak.
- P5b: Oturuş stub'da marjinal (pay 6.9 mm, pitch −50°); ±5° yerleştirmeler gövde üstüne devriliyor. Oturuş bankası
  penceresi son MJCF'de doğrulanır. Gerekirse derive'a fiziksel oturtma ölçütü eklenir ya da `seated_tilt_deg`
  daraltılır.
- P5b: Son modelle `tools/tr35_smoke.py` ve küçük uçtan uca eğitimler koşulur (P7 girdisi). Orta N'li bir tr35 eval
  ile süre ölçülür. `launch.py --dry-run --robot tr35_b2 --source git` gerçek dosyalarla koşulur.

## G-BENCH ve Adım 0

- Pot ortak mod hatası 16 A'da varsayılanla yaklaşık 4°'ye çıkabilir.
- Sürtünme aktarımında, analitik uydurma ile MuJoCo frictionloss (yumuşak kısıt) arasında bant içi durma noktası
  0.1–0.18° farklı. Frictionloss'un solref/solimp ayarı gözden geçirilecek.
- Ölçülecek tahminler:
  - plastik horn biçimleri (ES3054 tek kol, MG90S/SG90 çift kol);
  - MG90S/SG90 kule yükseklikleri;
  - DS kol kalınlığı ve uç yarıçapı;
  - 300 W buck kütlesi;
  - servo hub boyutu.
- İlk tezgâh oturumu `--cmd scan` ve `--cmd "osc auto"` ile başlamalı. Firmware donanımda hiç koşmadı.
