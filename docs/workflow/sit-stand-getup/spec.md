# Spec: OpenDuck — boy ayarı, oturma/kalkma ve düşünce kalkma

Durum: taslak v1 (2026-10-22). Codex review'u kullanıcı isteğiyle atlandı.
Kaynak bulgular (deney/araştırma raporları): `docs/workflow/sit-stand-getup/research/*.md` (deney scriptleri: oturumun scratchpad `sitstand/` klasörü) → özetleri bu dosyada,
sayılar `phys-sit`, `phys-getup`, `map-mjx`, `map-cpu-play`, `research` raporlarından.

## 1. Problem ve hedef

OpenDuck şu an yalnızca joystick ile yürüyor (`walk` görevi, 49 boyutlu gözlem, 10 bacak aksiyonu).
Hedef iki yeni politika:

1. **`main` politikası (`policy.onnx`)**: yürüme + sürekli **gövde yüksekliği komutu** (çömelme / alçak
   yürüme) + **otur komutu** (durur, çömelir, gövdeyi yere koyup oturur; komut kalkınca ayağa kalkar).
2. **`getup` politikası (`getup.onnx`)**: sırtüstü / yüzüstü / yan düşmeden kalkıp HOME duruşuna gelir,
   sonra `main`'e devreder.

Mevcut `walk` görevi **olduğu gibi kalır** (varsayılan; mevcut testler ve `walk-mjx-v2` baseline'ı bozulmaz).
Yeni görevler `--task main|getup` ile seçilir.

## 2. Kapsam dışı (non-goals)

- Gerçek robota aktarma: motor tanımlama (BAM), XL330 tork modeli kalibrasyonu, gecikme ölçümü.
  (Not: simülasyon tork sınırı 0.96 N·m; XL330 datasheet'i ~0.52 N·m @5 V diyor — risk olarak §7'de.)
- CAD/FreeCAD değişikliği (`cad/openduck.py`, `build/`): model değişiklikleri yalnızca eğitim modelinde
  (`rl/robot_model.build_spec`) yapılır.
- Kafa/gaga ifadeleri, kamera/ToF algısı, engebeli zemin.
- CPU (torch PPO) ile yeni görevlerin **performanslı eğitimi** (CPU env parity sadece play/test/eval için).
- Multi-critic PPO, motion imitation, tek politikada yürüme+getup birleşimi.

## 3. Mimari

### 3.1 Görev tanımları (`rl/tasks.py`, yeni düz modül)

| | `walk` (mevcut) | `main` (yeni) | `getup` (yeni) |
|---|---|---|---|
| Aksiyon | 10 bacak | 10 bacak | 14 (10 bacak + neck_pitch, head_pitch, head_yaw, head_roll) |
| Aksiyon eşleme | `HOME + 0.5·clip(a,±1.5)` | `q_ref(h_ref, α) + 0.5·clip(a,±1.5)` | `clip(HOME + S_j·clip(a,±1), ctrlrange)`, `S_j = max(|hi_j−HOME_j|, |lo_j−HOME_j|)` |
| Aktör gözlemi | 49 | **53** | **48** |
| Kritik gözlemi | 58 | **64** | **57** |
| Bölüm | 20 s | 20 s | 8 s, düşme sonlandırması yok |
| Çarpışma modeli | mevcut | mevcut + diz üst sınırı 65° | genişletilmiş (§3.6) + diz 65° |
| Kafa | rastgele hedef (DR) | rastgele hedef × (α≈0), oturuşta script | politika |
| γ (discount) | 0.97 | 0.97 (CLI ile 0.99 denenebilir) | 0.99 |

`tasks.py` her görev için: boyutlar, aksiyon eklemleri ve ölçekleri, gözlem düzeni (`obs_layout`: isim+boyut
listesi), ödül ağırlıkları, bölüm uzunluğu, reset karışımı, model varyantı. Paket düz modül olmalı
(`tools/runpod/launch.py` `rl_imports` yalnız `rl/<ad>.py` çözer). Veri dosyası yok; bankalar çalışma
anında üretilir (§3.5).

### 3.2 `main`: komutlar ve duruş filtresi

Komut (gözlemde, 5): `[vx, vy, wz, h_n, sit]`, `h_n = (h_cmd − H_HOME)/(H_HOME − H_MIN_STAND) ∈ [−1, 0]`.
Duruş durumu (gözlemde, 2): `[h_ref_n, α]`. Her ikisi de **robot runtime'ında birebir yeniden üretilebilir**
deterministik filtrelerdir (meta'ya yazılır):

```
h_ref ← h_ref + clip(h_cmd − h_ref, ±H_RATE·dt)                 H_RATE = 0.02 m/s
α     ← α + clip(sit − α, +dt/SIT_RAMP_S, −dt/STAND_RAMP_S)     SIT_RAMP_S = 2.5 s, STAND_RAMP_S = 2.5 s (S4: 2.0'da referans kalkış sonunda geriye devriliyordu)
sit = 1 iken vx = vy = wz = 0 (gözlemde ve ödülde)
```

Sabitler: `H_HOME = 0.115`, `H_MIN_WALK = 0.085`, `H_MIN_STAND = 0.070`, `Z_SIT = 0.044` (gövde kutusu yerde),
`H_DEEP = 0.065`. Hız komutu ≠ 0 yalnızca `h_cmd ≥ H_MIN_WALK` iken örneklenir.

**Referans duruş** `q_ref(h_ref, α)` (10 bacak; sol bacak değerleri, sağ aynalı; mutlak eklem açıları, derece):

- Çömelme ailesi `F(h)` (phys-sit §1, COM ayak üstünde, ayaklar düz). `q_ref(H_HOME, 0) = HOME` tam olarak
  sağlanmalı: `F_home(h) = HOME + (F(h) − F(H_HOME))`, aradaki değerler lineer interpolasyon.

  | h (m) | pitch_ref (°, + = burun aşağı) | hip_pitch | knee | ankle |
  |---|---|---|---|---|
  | 0.120 | 0 | −25.5 | −10.5 | 15.0 |
  | 0.110 | 0 | −10.5 | 16.5 | 27.0 |
  | 0.100 | 0 | 1.5 | 36.0 | 34.5 |
  | 0.090 | 0 | 13.5 | 52.5 | 39.0 |
  | 0.085 | 0 | 19.5 | 60.0 | 40.5 |
  | 0.080 | −7.5 | 40.5 | 60.0 | 27.0 |
  | 0.075 | −15 | 57.0 | 60.0 | 18.0 |
  | 0.070 | −17.5 | 69.0 | 60.0 | 8.5 |
  | 0.065 | −22.5 | 81.0 | 58.5 | 0.0 |

  hip_yaw ve hip_roll HOME'da kalır. Uygulayıcı tabloyu `phys-sit/family3.py` ile yeniden üretip doğrular.
- Oturuş pozu `SEAT` (tucked flat sit): hip_pitch 87.5, knee 47.5, ankle −40; gövde z 0.042 ± 0.003,
  pitch 0 ± 5°. Bu pozun HOME'dan farkı hip_pitch +113.7°, ankle −66° → aksiyon aralığının dışında, bu yüzden
  referans duruş zorunlu.
- Oturma yolu α ile parametrik (phys-sit'in başarılı yolu: önce aile boyunca derin çömelme, sonra oturuş):
  `s1 = clip(α/0.6, 0, 1)`, `s2 = clip((α−0.6)/0.4, 0, 1)`;
  `q_ref = lerp(F_home(lerp(h_ref, H_DEEP, s1)), SEAT, s2)`;
  `z* = lerp(lerp(h_ref, H_DEEP, s1), Z_SIT, s2)`; `pitch* = lerp(pitch_ref(lerp(h_ref,H_DEEP,s1)), 0, s2)`.
- Kafa (politika sürmez): `head_offset = s1 · (HEAD_SIT − HEAD_HOME)`, `HEAD_SIT`: neck_pitch 36°,
  head_pitch +10° (HOME 20°, 20°). S4: 40°/0° neck_splice'ı gövdeye 13.8 mm³ sokuyordu, 36°/−10° rampa sonunda geriye
  devriliyordu. S5 (yeni CAD): kafa öndeyken (+10°) sıfır politikayla kalkış %90 → %99; +12.5°'ten sonra öne devrilme. Kafa geride olmadan oturuştan kalkılamıyor (phys-sit §2).
  Rastgele kafa hedefleri `× (α < 0.01)`; itmeler yalnız `α < 0.01` iken.

**Segment örnekleme** (bölüm içinde, süre `U(3.5, 6.5)` s; mevcut 6 s komut yenilemesinin yerini alır):

| Segment | Olasılık | Komut |
|---|---|---|
| WALK | 0.45 | hızlar `COMMAND_RANGES`'ten (%15 sıfır), `h_cmd`: %50 `H_HOME`, %50 `U(H_MIN_WALK, H_HOME)` |
| STAND | 0.10 | hız 0, `h_cmd = H_HOME` |
| CROUCH | 0.15 | hız 0, `h_cmd ~ U(H_MIN_STAND, H_HOME)` |
| SIT | 0.30 | `sit = 1`, hız 0 |

SIT'ten sonra SIT gelmez (geçişler garanti). Her kanal ilk adımdan itibaren değişken olmalı (Brax normalizer
`std_eps = 0` tuzağı, map-mjx §7.5). `fixed_command` / `set_command` 3 veya 5 boyut kabul eder; 3 boyut
`h = H_HOME, sit = 0` ile doldurulur.

**Reset karışımı** (`main`): %75 HOME (+ mevcut gürültü), %25 oturuş bankasından (§3.5). Oturuştan
başlayan bölümde ilk segment %50 SIT (otur kalmaya devam), %50 STAND (kalk). Başlangıçta `α`, `h_ref`
gerçek duruma göre başlatılır (oturuşta `α = 1`).

### 3.3 `main`: gözlem, ödül, sonlandırma

Aktör (53): `gravity 3, gyro·0.25 3, q−q0 14, qd·0.05 14, last_action 10, command 5, posture 2, phase 2`.
Kritik (64): aktör (gürültüsüz) + `linvel 3, trunk_z 1, foot_contact 2, foot_z 2, friction 1,
trunk_floor 1, head_floor 1`. Gürültü vektörü command/posture/phase için 0. Faz her zaman ilerler.

Kapılar: `moving = [‖cmd[:3]‖ > 0.05]`, `g_step = moving · (1 − α)`, `still = 1 − g_step`.

Ödül değişiklikleri (ağırlıklar saniye başına; `total = Σ w·r·CTRL_DT`, mevcut kural):

| Terim | Tanım | w |
|---|---|---|
| tracking_lin/ang_vel | mevcut (sit'te komut 0 → durgunluk ödülü) | 2.0 / 2.0 |
| feet_air_time, feet_phase | mevcut × `g_step` | 1.0 / 1.0 |
| orientation | `‖g_xy − g_xy*(pitch*)‖²` | −5.0 |
| base_height | **kaldırılır** | — |
| height_track | `exp(−((z − z*)/0.02)²)` | +2.0 |
| height_l1 | `|z − z*|` | −10.0 |
| posture | `still · Σ_legs (q − q_ref)²` | −1.0 |
| stand_contact | `still · (1−α) · [iki ayak yerde]` | +0.5 |
| seat | `[sit=1 ∧ α=1] · trunk_floor · [gz < −0.9]` | +1.0 |
| vz_cap | `max(|v_z| − 0.08, 0)` (yumuşak geçiş) | −20.0 |
| diğerleri | lin_vel_z, ang_vel_xy, torques, joint_acc, action_rate, feet_slip, feet_distance, hip_pose, alive — değişmez | mevcut |

Sonlandırma (`main`): `gz > −0.6` **veya** `head_floor` **veya** `(trunk_floor ∧ α < 0.3)` (planlanmamış
oturma) **veya** bozuk fizik. Mevcut `trunk_z < 0.075` kuralı `main`'de kaldırılır. Ceza −10 (mevcut).

Episode metrikleri (mod bazında pay/payda): `walk_steps`, `sit_steps`, `seated_ok_steps`,
`standup_attempts`, `standup_success` (α 1→0 başladıktan sonra 3 s içinde `z > 0.105`, iki ayak yerde,
`gz < −0.9`), `sitdown_success` (sit=1'den sonra 3.5 s içinde seat koşulu), `falls_by_mode`.

### 3.4 `getup`

Gözlem (48): `gravity 3, gyro·0.25 3, q−q0 14, qd·0.05 14, last_action 14`. Kritik (57): + `linvel 3,
trunk_z 1, foot_contact 2, trunk_floor 1, head_floor 1, friction 1`. Komut ve faz yok.

Reset karışımı: %70 düşmüş banka, %10 düşüş ortası (HOME'dan 15–35° devrik, 0–4 rad/s açısal hız,
oturtmadan), %20 ayakta (HOME + gürültü; dik durmayı korumayı öğrenir). DR `walk` ile aynı.

Ödüller (ilerleme durumu `p = global_step / toplam`; "geç" terimler `p ∈ [0.5, 0.75]` arasında 0'dan
lineer artar):

| Terim | Tanım | w |
|---|---|---|
| upright | `−gz` | +1.5 |
| height_gauss / height_sharp | `exp(−((z−H_HOME)/σ)²)`, σ 0.04 / 0.015 | +1.0 / +1.0 |
| height_l1 | `|z − H_HOME|` | −6.0 |
| upright_progress | `(Φ(s')−Φ(s))/dt`, `Φ = −gz` (potansiyel tabanlı) | +5.0 |
| height_progress | `(Φ(s')−Φ(s))/dt`, `Φ = min(z, H_HOME)` | +50.0 |
| pose | `mean_14 exp(−((q−q0)/0.5)²) · smoothstep((z−0.06)/(0.105−0.06))` | +2.0 |
| composite | `exp(−(Δz/0.03)²)·exp(−tilt²/0.4²)·exp(−pose_rms²/0.4²)` | +3.0 |
| torque_sat | `Σ max(|τ| − 0.9·0.96, 0)` | −1.0 |
| head_impact (geç) | kafa-zemin temasının başladığı adımda `‖v_head‖²` (olay başı, ×dt yok) | −1.0 |
| action_rate (geç) | mevcut tanım (14 boyut) | −0.01 → −0.1 |
| joint_vel (geç) | `Σ qd²` | 0 → −1e-4 |
| arrival_damping (geç) | `‖gyro_xy‖² · smoothstep(z)` | 0 → −0.05 |

Başarı (metrik ve runtime devri): `gz < −0.9 ∧ z > 0.10 ∧ pose_rms_14 < 0.3 rad ∧ ‖gyro‖ < 0.5 rad/s`,
≥ 1 s boyunca. Başarıda bölüm bitmez (dik durmayı sürdürmek ödüllü).

### 3.5 Poz bankaları (`rl/pose_bank.py`, yeni)

MuJoCo C ile env kurulurken deterministik (sabit tohum) üretilir; MJX reset'i yalnızca banka indeksi seçer
(reset içinde fizik oturtma yasak: 40–65× yavaşlama, map-mjx §3).
- **Düşmüş banka** (getup modeli): rastgele yönelim, 2–10 cm düşürme, eklem hedefleri HOME ± rastgele,
  1.5 s oturtma; `gz > 0.85 ∧ yalnız kafa teması` ("tepe üstü", kutu artefaktı) atılır. N = 4096.
  Beklenen dağılım ≈ sırt %28 / yüz %26 / sol %20 / sağ %18.
- **Oturuş bankası** (`main` modeli): `SEAT` hedefleriyle α=1 duruşundan küçük gürültüyle (eklem σ 0.05 rad,
  ±5° eğim) oturtulmuş durumlar. N = 1024.
- Reset'te: yaw rastgele (`quat_mul(yaw_q, q_bank)`, keyframe quaternion'u değil), xy = 0, z bankadan,
  qvel = 0. Yeni rastgele sayılar mevcut RNG akışlarını bozmayacak şekilde `fold_in` ile.
- CPU env aynı bankayı aynı tohumla üretir (worker sayısından bağımsız).

### 3.6 Model değişiklikleri (`rl/robot_model.build_spec`, yalnız yeni görevler)

- Diz üst sınırı 70° → 65° (sol `1.13446` rad, sağ aynalı; ctrlrange da). 70°'de uyluk-incik teması var.
- neck_pitch üst sınırı 40° → 36° (eklem + ctrlrange; S4/S5). 38.3°'den itibaren neck_splice gövde kabuğuna giriyor;
  yumuşak eklem sınırı çarpmada ~2.3° aşıyor ve oturuşta kafa sınıra yaslanıyor.
- İçe kalça dönmesi 20° → 8° (sol hip_yaw alt sınırı, sağ üst sınırı; eklem + ctrlrange; S4/S5). Ayakta yürürken 20°'de
  pozların yarısında incik braketi gövdeye giriyor (458 mm³'e kadar; walk-mjx-v2 bu sınıra dayanarak yürüyor); çömelerek
  yürürken 10°'de de uyluk plakası gövde kabuğuna giriyor, 8°'de hiç.
- `getup` için zemin-yalnız ek çarpışma geometrileri (sınıf `collision`: contype 0, conaffinity 1):
  uyluklar, incikler, kalçalar, boyun, gaga — başlangıç kutuları `research/extra_boxes.json`
  (görsel mesh sınırlarına oturtulmuş). Kabul: yan yatışta mesh-zemin batması < 3 mm. Kapsül tercih edilir
  (MJX'te ucuz); MJX kontak sayısı artışı ölçülüp raporlanır.
- `main` mevcut 4 kutuyla kalır (oturuşta gövde kutusu gerçek şasiyle 0.0 mm örtüşüyor, phys-sit §3).

### 3.7 Eğitim, dışa aktarma, oynatma

- `train_mjx.py`: `--task {walk,main,getup}` (varsayılan walk); görev bazlı varsayılanlar (γ, bölüm,
  ağ 512-256-128 getup için). Warm-start (`walk` → `main` ağırlık genişletme) **kapsam dışı**: params
  dosyası satır içi pod paketine sığmıyor; `main` sıfırdan eğitilir (`walk` 2e8 adım 17.5 dk sürdü).
- Global adım sayacı `info["gstep"]` (bölümler arası korunur) → getup müfredatı. Eval env son aşama
  ağırlıklarıyla çalışır.
- Ortak meta modülü `rl/policy_meta.py` (torch'suz): mevcut alanlar + `task`, `obs_layout`, `command_names`,
  `command_ranges`, `action_joints`, `default_action_q`, `action_scale` (vektör), duruş filtresi sabitleri,
  `F` tablosu, `SEAT`, `HEAD_SIT`, devir eşikleri, `version`. Run klasöründe dosya adı yine `policy.onnx`
  (araçlar bunu bekliyor); rol `meta["task"]` ile belirlenir.
- `rl/play.py`: `--demo {walk,sit,getup}` (çıplak `--demo` = walk), `--policy` + `--getup-policy`;
  süpervizör: `main→getup` `gz > −0.6` 5 adım; `getup→main` başarı koşulu (z hariç, robotta yok) 0.5 s;
  devirde `last_action = 0`, `α = 0`, `h_ref = H_HOME`, 1 s sıfır komut. Metrikler: t_to_seated, t_to_stand,
  yükseklik RMSE, tepe |τ|/0.96, t_to_upright, devirden sonra 2 s içinde düşme.
- `rl/eval_tasks.py` (yeni): ONNX politikaları DR açık değerlendirip §6 kabul metriklerini JSON'a yazar
  (walk baseline karşılaştırması dahil).
- `sim/collision_check.py`: phys-sit voxel öz-çarpışma aracının repo'ya taşınmış hali (rollout'ları denetler).

## 4. Kısıtlar ve varsayımlar

- Paylaşılan matematik yalnız `rl/task_math.py`'de (numpy/jax); arka uçlar yalnız durum toplar
  (`rl/TASK.md` sözleşmesi; yeni görevler için güncellenir).
- JAX: statik şekiller; mod kapıları `jnp.where`/çarpım (vmapped `lax.cond` iki dalı da çalıştırır);
  info anahtar kümesi ve dtype'lar reset/step/merge'de aynı; `_KEEP_INFO` güncellenir.
- Sürümler: jax 0.12.0.1, mujoco/mjx 3.16.0, brax 0.14.2, playground 0.2.0 (pod pin'leri değişmez).
- Mevcut `walk` davranışı, gözlemi, testleri ve `walk-mjx-v2` politikası bit-düzeyinde aynı kalır.
- Eğitim RunPod'da (`tools/runpod/launch.py`, `--source inline`); paket boyutu < 100 KB gövde sınırı.
  Ölçülen hız: `walk` 239k adım/s (RTX 4090).

## 5. Edge case'ler

1. Yürürken otur komutu: hızlar anında 0, α rampası başlar; politika önce durup sonra çökmeli
   (vz_cap + height_track). Hızlı yürüyüşten otur segmenti eğitimde örneklenir (WALK→SIT geçişi).
2. Oturma yarıda iptal (sit 1→0, α≈0.5): filtre geri döner; yol tersine izlenir.
3. Oturuşta itme yok; ama DR'li ayak sürtünmesi/kütle farkları oturuşta devrilmeye yol açmamalı.
4. Derin çömelmede (`h < H_MIN_WALK`) hız komutu gelmez; runtime da bunu zorlar (meta).
5. Getup'tan devir anında aksiyon sıçraması: `last_action` sıfırlanır; `q_ref = HOME` olduğundan süreklilik var.
6. Kafa ile itme (getup) gerekli; kafa çarpma hızı geç cezalandırılır, kafa teması yasaklanmaz.
7. Normalizer: sabit kalan kanal yok (her mod ilk adımdan örneklenir).
8. Eval env'in müfredat sayacı 0'dan başlar → son aşama ağırlıkları bayrakla verilir.
9. Oturuş temasında MJX çözücü (iter 4 / ls 8) NaN üretirse: bozuk-fizik sayılır; oran > %0.1 ise iterasyon artırılır.
10. Banka/model uyuşmazlığı: banka hangi model varyantıyla üretildiyse o görevde kullanılır (assert).
11. Konteyner yeniden başlarsa eğitim baştan başlar (mevcut davranış; kapsam dışı).

## 6. Kabul kriterleri (simülasyon, DR açık, `rl/eval_tasks.py` ile ölçülür)

- **A1 Yürüme:** `main`, `h = H_HOME, sit = 0`'da hız takibi ödülü (lin+ang) `walk-mjx-v2`'nin ≥ %90'ı;
  düşme oranı ≤ baseline + 2 puan.
- **A2 Yükseklik:** kararlı durumda `|trunk_z − h_ref|` RMS < 1 cm (ayakta tüm aralık; yürürken
  `[H_MIN_WALK, H_HOME]`).
- **A3 Oturma/kalkma:** sit komutundan sonra ≤ 3.5 s içinde oturuş koşulu (gövde yerde, `gz < −0.9`)
  ve segment boyunca düşmeme ≥ %95; sit kalkınca ≤ 3.0 s içinde ayakta (`z > 0.105`, iki ayak, `gz < −0.9`)
  ≥ %95; geçişlerde düşme < %5; tork satürasyonu adımların < %1'i.
- **A4 Getup:** banka durumlarından 6 s içinde başarı ≥ %90 genel, her mod (yüz/sırt/yan) ≥ %85;
  kafa-zemin tepe kuvveti medyanı < 20 N.
- **A5 Dışa aktarma:** `policy.onnx` [N,53]→[N,10], `getup` [N,48]→[N,14]; ONNX–JAX farkı < 1e-4;
  meta'da `obs_layout` ve filtre sabitleri.
- **A6 Demo:** `rl.play --demo sit` ve `--demo getup` video üretir ve metrikleri yazar.
- **A7 Testler:** `pytest rl/tests` (CPU) ve MJX testleri geçer; mevcut `walk` testleri değişmeden geçer;
  yeni testler: düzen sabitleri, 3/5 boyut komut dolgusu, sit'te hız sıfırlama, mod-duyarlı sonlandırma,
  oturuş/düşmüş banka reset'i (worker sayısından bağımsız), 14 aksiyon eşleme, filtre/q_ref referans testi,
  getup meta, MJX–C dinamik uyumu (oturuş ve düşmüş pozlarda).
- **A8 Öz-çarpışma:** eval rollout'larında `sim/collision_check.py` ile HOME'da zaten var olan arayüz
  temasları hariç > 1 mm³ girişim yok.

## 7. Riskler

- XL330 gerçek torku simülasyonun ~yarısı olabilir → kalkış süresi gerçek robotta ≥ 3 s gerekebilir.
- `main`'de γ 0.97 ile 2–2.5 s'lik geçişler zayıf öğrenilirse γ 0.99 ile yeniden eğitim.
- Getup açık-döngü deneylerde DR altında %55–87; RL politikasının ≥ %85 hedefi müfredat ayarı gerektirebilir.
- Ek çarpışma geometrileri MJX hızını düşürür (getup bütçesi buna göre).
