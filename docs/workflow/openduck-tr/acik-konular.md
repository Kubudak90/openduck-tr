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

- MJX çözücü iterasyonu: tr35 sürtünmesi yaklaşık 16 iterasyon istiyor, bugün 4/8; maliyet yaklaşık 1.9 katı. Seçenek:
  frictionloss'u stall'un %3'ünün altına çekmek.
- Poz bankaları tr35 için yeniden üretilecek.
- Görev matematiği `tau_ratio`, `limit_torque` ve `rail_demand` kullanmalı; ham `actuator_force` ve sabit 0.96 N·m değil.
- `policy_meta` sensör ve aktüatör sabitlerini dışa aktarmalı.
- `rl/TASK.md` dosyasına `robot=` ve `backlash=` belgeleri eklenecek. Boşluk açıkken `s['q']` servo açısıdır.
- `rl/config.py` tr35 sabitleri (`H_HOME`, `EXTRA_COLLISION`, `FEET_MIN_DISTANCE`, ...) hâlâ 27 cm değerlerinde. Yeni
  değerler:
  - HOME ayak aralığı 133.4 mm;
  - trunk z 152.5 mm;
  - taban kutusu genişliği `foot_w` 64 mm.
- HOME kütle merkezi ayak merkezinin 19.9 mm gerisinde (yer tutucuyla; P2b iyileştirir). Kalan fark HOME yeniden
  türetmesiyle kapatılır.
- Yeni öz-çarpışma çiftleri (kafa ↔ trunk; ayak ↔ aynı bacağın uyluk/kalça beşiği) RL çarpışma listesine girer.

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
