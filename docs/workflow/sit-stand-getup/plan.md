# Plan: sit-stand-getup

Girdi: `spec.md` (v1). Plan spec dışında tasarım kararı içermez; yeni ihtiyaç çıkarsa spec güncellenir.
Dal: `feat/sit-stand-getup`. Push yok (kullanıcı istemedikçe). Her adım sonunda commit (yerel).

## Dosya sahipliği (paralel adımlarda çakışmayı önler)

| Adım | Sahip olduğu dosyalar |
|---|---|
| S1 | `rl/config.py`, `rl/tasks.py`(yeni), `rl/task_math.py`, `rl/robot_model.py`, `rl/pose_bank.py`(yeni), `rl/policy_meta.py`(yeni), `rl/TASK.md`, `rl/tests/test_task_math.py`(yeni), `rl/tests/test_pose_bank.py`(yeni) |
| S2 | `rl/env_mjx.py`, `rl/train_mjx.py`, `rl/tests/test_env_mjx.py` |
| S3 | `rl/env_cpu.py`, `rl/train.py`, `rl/export_onnx.py`, `rl/play.py`, `rl/eval_tasks.py`(yeni), `sim/collision_check.py`(yeni), `rl/tests/test_env_cpu.py`, `rl/tests/test_ppo.py`, `rl/tests/test_play_tasks.py`(yeni) |
| S4 | entegrasyon: yalnız düzeltmeler + `README.md`, `tools/runpod/*` (gerekirse) |

## Adımlar

**S1 — Ortak sözleşme (sıralı, tek ajan, ana çalışma ağacı).** Girdi: spec §3.1–3.6, §4.
- `tasks.py` TaskSpec (`walk`, `main`, `getup`); `config.py` sabitleri (H_*, rampalar, F tablosu, SEAT,
  HEAD_SIT, segment/reset olasılıkları, ağırlık tabloları, getup bloğu, ek çarpışma geometrileri).
- `task_math.py`: `posture_step`, `q_ref`, `z_ref`, `pitch_ref`, `head_offset_ref`, görev-parametreli
  `observation/critic_observation/noise_vector/reward_terms/reward_weights/total_reward/terminated`,
  getup başarı fonksiyonu, vektör ölçekli `action_to_targets`. `walk` yolu bit-düzeyinde aynı.
- `robot_model.py`: `build_spec(variant)` (`walk` aynı; `main`: diz 65°; `getup`: + ek geometriler),
  `RobotIndex(model, action_joints)`. `pose_bank.py`: deterministik oturuş/düşmüş bankaları.
  `policy_meta.py`: torch'suz ortak meta.
- `TASK.md`: yeni görevlerin arka uç sözleşmesi (S2/S3 bunu uygular): state dict alanları, adım sırası,
  segment/filtre güncelleme sırası, reset karışımı, info alanları.
- Bitti: `pytest rl/tests/test_task_math.py rl/tests/test_pose_bank.py` geçer; mevcut
  `rl/tests/test_env_cpu.py` ve `test_env_mjx.py` **değişmeden** geçer (walk bozulmadı).

**S2 — MJX arka ucu + eğitici (S1 sonrası, worktree A, S3 ile paralel).**
- `env_mjx.py`: görev parametresi; segment/filtre durumu info'da; banka reset'i; mod-duyarlı ödül
  ağırlıkları ve sonlandırma; getup (14 aksiyon, 8 s, sonlandırmasız); `gstep` müfredatı; metrikler.
- `train_mjx.py`: `--task`, görev varsayılanları, meta `policy_meta.py`'den.
- Bitti: `test_env_mjx.py` (walk testleri aynen + yeni görev testleri: şekiller, sit sonlandırması, banka
  reset'i, 14 aksiyon, MJX–C uyumu oturuş/düşmüş pozda) geçer; CPU JAX'ta
  `main` ve `getup` için küçük uçtan uca eğitim `policy.onnx` üretir.

**S3 — CPU arka ucu + araçlar (S1 sonrası, worktree B, S2 ile paralel).**
- `env_cpu.py` görev parametresi (S1 TASK.md sözleşmesi), 3/5 komut dolgusu, banka reset'i, 14 aksiyon,
  `terminate=False` seçeneği. `train.py`/`export_onnx.py`: görev geçişi, meta `policy_meta.py`'den.
- `play.py`: `--demo {walk,sit,getup}`, `--getup-policy`, süpervizör, yeni metrikler.
  `eval_tasks.py`: A1–A4 metrikleri JSON. `sim/collision_check.py`: phys-sit voxel aracı (A8).
- Bitti: `test_env_cpu.py`, `test_ppo.py`, `test_play_tasks.py` geçer; ZeroPolicy ile
  `play --demo sit/getup` ve `eval_tasks` çökmeden çalışır.

**S4 — Entegrasyon (ana ağaç, ben).** A ve B dallarını birleştir; çapraz parity testleri (aynı tohumla
CPU ve MJX `main`/`getup` reset+adım gözlemleri tutarlı); tüm testler; `launch.py --dry-run --task main/getup`
paket boyutu < 100 KB; README. Ardından bağımsız kod review'u (workflow: bulgu → doğrulama) ve düzeltmeler.
- Bitti: tüm testler geçer, review'de doğrulanmış kritik bulgu kalmaz.

**S5 — Eğitim (RunPod, insan onayı: fiyat bildirilir).** İki pod paralel:
- `main`: `--task main --num-timesteps 1e9` (sıfırdan).
- `getup`: `--task getup --num-timesteps 1e9`.
- Bitti: her iki koşu `policy.onnx` üretir, indirilir, pod'lar silinir.

**S6 — Kabul (yerel).** `eval_tasks.py` ile A1–A4, A8; `play` videoları (A6); sonuçlar `runs/<ad>/eval.json`
ve kullanıcıya özet + videolar. Kriter tutmazsa: neden + önerilen tek değişiklik (spec'e geri dönüş).

## Riskli adımlar (insan onayı)
- S5 (para harcar) — saatlik fiyat ve üst sınır bildirilerek başlatılır.
