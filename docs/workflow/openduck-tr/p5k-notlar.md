# P5k: sesli komut katmanı (spec eki K5–K7) — notlar

Girdi: `spec-ek-komut.md` (ek K v3), `p5a-notlar.md`, P2-0 stub'ı (`rl/derived/tr35_b2.json`). Kaynak dal
`worktree-wf_a65822de-6b5-1`: iş akışı oturum sınırında yarıda kaldı, değişiklikler commit'lenmemişti. Kullanıcı worktree'yi
WIP commit'i olarak kurtardı. Bu oturumda bu commit düzgün mesajla yeniden commit'lendi, test edildi ve main ile
entegre edildi (§3).

## 1. Teslim edilenler

| Parça | Dosya | Not |
|---|---|---|
| K5 denetleyicisi | `rl/commands.py` | Durum × olay tablosu bir veri yapısıdır (`TABLE`). Tamamı float32 numpy. Eval, play ve firmware aynı tanımı kullanır; eğitimde çalışmaz. |
| Kimlik tablosu | `configs/komut_kimlik.json` | Şema 1, **örnek** (`"ornek": true`). Gerçek tabloyu G-SES-a öğrenme adımı yazar (§3). |
| Jest kanalı (K6.1) | `rl/task_math.py` | Tek xp-genel fonksiyon: `gesture_init/step/sample/gate/fade/raw`. Tukey zarfı, 0.25 s kesme sönümü, başlatma kapısı, oturuşta gaga ≤ 15°. Toplam kafa ctrl hızı ≤ 0.5·ω0 assert'i: `gesture_rate_bounds`. |
| Komut atomları ve "tut" (K6.2) | `rl/task_math.py` `sample_segment` | main yürüme segmentlerinin %20'si atom (`C.ATOM`), oturuştan önce ≥ 1 s "tut" (`C.HOLD_S`). Aynı uniform'lar kullanılır. |
| Eğitim ortamları | `rl/env_cpu.py`, `rl/env_mjx.py` | tr35 main'de jest kanalı ve atomlar açık (`C.VOICE`; tr35_constants). xl330 değişmedi. Jest sırasında rastgele kafa hedefi 0'dır. |
| A10 düzeneği (K7) | `rl/eval_tasks.py` | Modül modeli: 15 s yenilenmeyen pencere, gecikme, uyandırma kuralı. Konuşmacı: S1 sabit dizi, S2 20 rastgele kelime. A6 devri A10 düzeninde (HO). Ölçütler, hücre kapsaması, gyro bias duyarlılığı (yalnız rapor). |
| Video / demo | `rl/play.py --demo voice` | A10 S1'den bir koşu. |
| Altın izler | `tools/commands_golden.py` → `rl/tests/data/komut_*.csv.gz`, `komut_golden.json` | Sentetik senaryolar (`rl/tests/komut_plant.py`) ve S1/S2 rollout'ları (stub, zero politikalar). Firmware bu izleri birebir üretmelidir: ayrık alanlar birebir, sürekli alanlar 1e-6. |
| Testler | `rl/tests/test_commands.py`, `rl/tests/test_voice_train.py` | K5 tablosu satır satır (spec tablosunun bağımsız kopyasına karşı), değişmezler, gyro bias (gerçek sensör modeli), CPU == MJX adım adım (jest ve atomlar açık), A10 dumanı, `play --demo voice`. |

## 2. Spec'in sessiz kaldığı yerlerde verilen kararlar

- Jest kanalı doluyken (jest ya da sönümü sürüyor) gelen yeni jest kelimesi reddedilir. Jestler kuyruğa girmez (K5).
- F → A devrinin sıfır komut süresinde (`handoff_zero_cmd_s`):
  - jest kelimeleri reddedilir;
  - "dur", A hücresi gibi işlenir;
  - diğer kelimeler kuyruğa girer (`HANDOFF`).
- Red işaretinin çift gaga tıkı, kanal doluysa atlanır; LED yine 3 kez yanıp söner.
- Gyro bias penceresi:
  - kesintisiz durağanlık süresidir: ≥ 1 s, en çok `BIAS_RUN_MAX_S` 5 s;
  - bias bu sürenin ortalamasıdır;
  - |b| ≤ 0.06 rad/s ile kırpılır.
- BEKLE'de robotta PWM kapalıdır, servolar gevşer. Simülasyon ölçülen pozu tutar.
- Fizikte erişilemeyen hücreler (`commands.UNREACHABLE`, A10 kapsama raporunda gerekçesiyle):
  - düşme/F ve düşme/B: düşme algısı bu durumlarda çalışmaz, spec "—" der;
  - tamam/B: BEKLE'de tamamlanma olayı yoktur.

## 3. Entegrasyon (bu oturum, P6e ile)

İki paralel dalın ortak dosyaları uyuşmuyordu:

- **Kimlik tablosu şeması.**
  - P6e'nin `tools/bench/voice_report.py` aracı `{"version", "ids", "wake"}` yazıyordu. `rl/commands.load_id_table` ise
    `{"schema": 1, "uyanma", "komutlar", ...}` okur.
  - G-SES-a'nın yazacağı tablo denetleyiciye yüklenemezdi; ayrıca `voice_report` depodaki örnek tabloyu boş okurdu.
  - Düzeltme: `voice_report` artık şema 1'i yazar ve okur. Örnek tablo (`"ornek": true`) ölçüm yerine kullanılmaz.
  - Test: `tools/bench/tests/test_voice.py`. `tablo` komutunun yazdığı dosya `load_id_table` ile yüklenir, 12 kelime
    eksiksizdir.
- **Gecikme DR'ı (K7, K8).**
  - A10 modül modeli sabit U(0.3, 1.5) s kullanıyordu.
  - Artık `eval_tasks.a10_latency()` kullanılıyor: `configs/ses_gecikme.json` varsa onun `uniform_s` alanı ([p05, p95],
    `voice_report rapor` yazar), yoksa varsayılan.
  - Kaynak, A10 raporunda `module.lat_source` olarak yazılır. Yeni ölçüm yeniden eğitim gerektirmez (K6.3).
  - Gidiş-dönüş testi: `test_shared_files_with_controller`.

## 4. Test sonuçları (bu oturum, Linux bulut konteyneri)

| Paket | Sonuç |
|---|---|
| CPU venv (mujoco 3.16.0, torch), `rl/tests` (`test_env_mjx` hariç) | 184 geçti, 9 atlandı (jax yok) |
| MJX venv (jax 0.12.0.1 CPU, brax, torch), `rl/tests` tamamı | 221 geçti, 1 atlandı |
| `tools/bench/tests` (entegrasyon sonrası) | 10 geçti |
| `test_voice_train.py` + `test_commands.py` (entegrasyon sonrası) | 29 geçti, 1 atlandı (jax) |

Atlanan tek MJX testi `test_derive_posture.py::test_committed_json`'dır: stub MJCF/mesh sha256'sı farklıdır. Aynı atlama
main'de de vardır (P5a), P5k'yla ilgisi yoktur. Stub mesh'i platforma bağlıdır; P5b son MJCF ile yeniden türetir.

## 5. Komutlar

```
python3 -m pytest rl/tests/test_commands.py rl/tests/test_voice_train.py
OPENDUCK_ROBOT=tr35_b2 OPENDUCK_SCENE=<tr35 scene.xml> python tools/commands_golden.py --out rl/tests/data
OPENDUCK_ROBOT=tr35_b2 python -m rl.eval_tasks --main-policy <main> --getup-policy <getup> --a10 [--a10-only] [--n-scale 0.05]
OPENDUCK_ROBOT=tr35_b2 python -m rl.play --demo voice --policy <main> --getup-policy <getup> [--voice-seat exact]
```

## 6. P5b'ye ve sonrasına devredilenler

- K6.4 kararları P7'den önce verilir:
  - EXTRA kutularının main eğitimindeki zemin teması;
  - main sonlandırması duruşa göre olmalı (oturuşta sabit gz > −0.6 kuralı yanlış sonlandırıyor);
  - RL çift listesi: jaw ↔ neck/trunk, kafa ↔ kalça/kalça beşiği ve spec 3.2.2'nin bütün listesi;
  - HEAD_SIT ve HEAD_TUCK mesh girişim sınırıyla yeniden türetilir.
- Türetilmiş dosya, kimlik tablosu ya da `commands.py` değişirse altın izler yeniden üretilir
  (`tools/commands_golden.py`).
- A10, son MJCF üzerinde koşulur. Stub'da oturuş bankası DR ve boşluk altında öne devriliyor (p5a-notlar 2.3); A10 S1
  duman koşuları ve altın S1 izi bu yüzden türetilmiş oturuş pozuyla kuruldu (`a10_seat = "exact"`,
  `play --voice-seat exact`). Son modelde varsayılan banka (`bank`) kullanılır.
- Gerçek kimlik tablosu ve gecikme dosyası G-SES-a'dan gelir (`voice_report.py tablo` / `rapor`).
