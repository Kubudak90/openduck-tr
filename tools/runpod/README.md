# OpenDuck eğitimini RunPod'da çalıştırma

`tools/runpod/launch.py` RunPod'da bir pod açar, yürüme eğitimini başlatır, ilerlemeyi izler,
sonuçları `runs/<ad>/` altına indirir ve podu siler. Yalnızca Python standart kütüphanesini
kullanır, ek kurulum gerekmez. Pod içinde `tools/runpod/pod_entry.sh` çalışır.

Varsayılan API RunPod REST **v2**'dir (`https://api.runpod.io/v2`). Eski v1
(`rest.runpod.io/v1`) 15.11.2026'da kapanıyor; gerekirse `--api v1` ile seçilebilir.

## 1. Hazırlık

**API anahtarı**

1. https://console.runpod.io → *Settings → API Keys → Create API Key*. Pod açıp silebilmesi için
   yetkisi *All* (Read/Write) olmalı. Hesapta bakiye de olmalı (*Billing*).
2. Anahtarı ortam değişkenine koyun. Shell geçmişine düşmemesi için:
   ```bash
   read -rs RUNPOD_API_KEY && export RUNPOD_API_KEY
   ```
   `launch.py` anahtarı hiçbir yere yazdırmaz (`--dry-run` çıktısında da gizlidir) ve poda göndermez.

**GitHub token secret'ı** (yalnızca `--source git` için)

Repo (`Kubudak90/openduck`) private olduğundan pod klonlamak için token ister.

1. GitHub → *Settings → Developer settings → Fine-grained tokens → Generate new token*:
   yalnızca `Kubudak90/openduck`, izin *Contents: Read-only*, kısa süreli.
2. RunPod console → *Secrets → Create Secret*: ad `github_token`, değer token.
3. `launch.py` poda `GITHUB_TOKEN={{ RUNPOD_SECRET_github_token }}` verir. RunPod değeri pod açılırken
   yerine koyar. Token loglara ve HTTP'ye çıkmaz. Başka ad için `--secret AD` kullanın.

## 2. Kod poda nasıl gider

| `--source` | Ne olur | Ne zaman |
|---|---|---|
| `inline` (varsayılan) | Eğitim modülünün (`rl/train_mjx.py` ya da `rl/train.py`) `rl/` içinde import ettiği dosyalar, `rl/robot_model.py`, model dosyaları (`--robot`'a göre, aşağıda) ve `pod_entry.sh` tar.gz + base64 olarak bir ortam değişkeninde gider | Commit/push gerekmez, yerel dosyalar gider |
| `git` | Pod `origin/<dal>` commit'ini token ile klonlar | Kod push edilmiş olmalı (launch.py kontrol eder) |

RunPod istek gövdesi sınırı yaklaşık 100 KB'tır (v2: 102400 bayt). `launch.py` paket ve gövde
boyutunu yazdırır. P4 (aktüatör + sensör modeli) ve P5a (`rl/export_c.py`) sonrası `mjx` satır içi
gövdesi sınırı aşar (≈ 107–112 KB): gerçek `launch` açık bir hata verir, `--dry-run` isteği yine
gösterir ve 1 ile çıkar. O durumda `--source git` kullanın.

### Robot profili (`--robot`)

| `--robot` | Pod ortamı | Paketlenen / git'te denetlenen model dosyaları |
|---|---|---|
| `xl330_27` (varsayılan) | değişmez (`OPENDUCK_ROBOT` eklenmez; istek gövdesi bugünküyle aynı) | `build/sim/openduck.xml`, `build/sim/scene.xml` |
| `tr35_b2` | `OPENDUCK_ROBOT=tr35_b2` (`--kappa K` → `OPENDUCK_KAPPA=K`) | `build/tr35_b2/sim/openduck.xml`, `build/tr35_b2/sim/scene.xml`, `rl/derived/tr35_b2.json` |

`tr35_b2` podunda `rl/config.py` sahneyi ve türetilmiş sabitleri bu varsayılan yollardan okur
(`OPENDUCK_SCENE` / `OPENDUCK_DERIVED` verilmez). Dosyalar yoksa `launch` (inline) ya da git denetimi
eksikleri adlarıyla bildirir; `--dry-run` "(YOK)" ile işaretler. tr35 eğitimleri her ONNX dışa
aktarmasından sonra `run/c/` (C dizileri, `rl/export_c.py`) de yazar; `fetch` bunları da indirir.

```bash
python3 tools/runpod/launch.py launch --robot tr35_b2 --kappa 0.5 --source git --dry-run -- --task walk
```

## 3. Komutlar

```bash
cd ~/openduck                                   # repo kökü
python3 tools/runpod/launch.py gpus             # GPU tipleri, güncel fiyat ve stok (--cpu: CPU podlar)
python3 tools/runpod/launch.py launch --dry-run # gönderilecek isteği gösterir (anahtar gizli), pod açmaz

# MJX (GPU) eğitimi: bitene kadar izle, sonuçları indir, podu sil
python3 tools/runpod/launch.py launch --backend mjx --max-hours 3 --wait -- --num-timesteps 200000000

# CPU eğitimi (CPU pod, 32 vCPU): python3 -m rl.train --workers <çekirdek sayısı>
python3 tools/runpod/launch.py launch --backend cpu --vcpu 32 --max-hours 6 --wait -- --max-steps 1e8

# Beklemeden başlatıp sonra bakmak
python3 tools/runpod/launch.py launch --backend mjx --name deneme1
python3 tools/runpod/launch.py status deneme1   # faz, ilerleme, kalan süre, log sonu, maliyet
python3 tools/runpod/launch.py fetch deneme1    # runs/deneme1/ (policy.onnx, policy_meta.json, son checkpoint)
python3 tools/runpod/launch.py logs deneme1     # RunPod konteyner logu (HTTP açılmadan önceki hatalar için)
python3 tools/runpod/launch.py terminate deneme1
python3 tools/runpod/launch.py list             # hesaptaki tüm podlar
```

`--` sonrasındaki argümanlar olduğu gibi `rl.train_mjx` / `rl.train`'e gider. `--out` verilmez, onu pod
belirler. Ad verilmezse `status`/`fetch`/`terminate` son kaydı (`runs/*/runpod/pod.json`) kullanır.
Kayıt yoksa `--pod-id` ile çalışır. RunPod'daki pod adı hep `openduck` ile başlar (`--name deneme1` →
pod `openduck-deneme1`, yerel klasör `runs/deneme1/`).

Önemli seçenekler:

| Seçenek | Varsayılan | Açıklama |
|---|---|---|
| `--robot` | `xl330_27` | `tr35_b2`: `OPENDUCK_ROBOT` + TR-35 model dosyaları (yukarıda) |
| `--kappa` | | Yalnız `tr35_b2`: hız çarpanı κ (`OPENDUCK_KAPPA`) |
| `--gpu` | 4090 → 5090 → L40S → RTX 6000 Ada → 3090 → A6000 → A40 | Sırayla denenir. Kısa ad olur (`--gpu 4090`) |
| `--cloud` | `SECURE,COMMUNITY` | Her GPU için önce Secure, sonra Community |
| `--datacenter` | | Veri merkezleri, sırayla (ör. `EU-RO-1,EU-CZ-1`). REST v1 `dataCenterIds`; verilirse `--api v1` seçilir |
| `--min-download-mbps` | | Makinenin en düşük indirme hızı (REST v1 `minDownloadMbps`). Yavaş ağ kurulumu (pip, JAX) uzatır |
| `--max-hours` | 4 | Eğitimin sert süre sınırı. Dolunca SIGTERM gider (`rl.train` checkpoint + ONNX yazar; `rl.train_mjx` son değerlendirmedeki dosyalarla kalır), 5 dk sonra SIGKILL |
| `--setup-hours` | 1 | Kod + kurulum için süre sınırı |
| `--linger-hours` | 2 (`--keep` ile 0) | Eğitim bitince pod sonuçları bu kadar sunar, sonra kendini siler. 0 = asla |
| `--wait` / `--keep` | | İzle + indir + sil / sonunda silme |
| `--min-cuda` | 12.4 | Host CUDA alt sınırı. "JAX GPU görmüyor" hatasında `12.8` deneyin |
| `--pip PAKET` | | Poda ek pip paketi |
| `--compute gpu` | | CPU backend'i GPU podda çalıştırır |
| `--image` | `runpod/base:1.0.7-ubuntu2404` | Python 3.12, uv ve git içerir. CUDA'yı jax pip paketleri getirir |
| `--ssh` | | 22/tcp + imajın sshd'si (RunPod hesabındaki SSH anahtarlarıyla) |

## 4. Pod içinde ne olur

1. Önce HTTP sunucu açılır: `https://<podId>-8765.proxy.runpod.net/<token>/`. Token rastgeledir ve
   `runs/<ad>/runpod/pod.json` içinde saklanır. Bu adres `status.json`, `manifest.json`, `entry.log`
   (kurulum), `train.log` (eğitim) ve `run/` (eğitim çıktıları) dosyalarını sunar.
2. Kod alınır (inline paket ya da git).
3. Kurulum yapılır: uv ile Python 3.12 venv kurulur. `mjx` için `jax[cuda12]==0.12.0.1 mujoco==3.16.0
   mujoco-mjx==3.16.0 brax==0.14.2 playground==0.2.0 onnx onnxruntime` kurulur, `cpu` için
   `mujoco==3.16.0 onnx onnxruntime` ile CPU torch kurulur.
   `rl/requirements-<backend>.txt` varsa varsayılan liste yerine o kullanılır.
4. Kontrol: model yüklenir ve eğitimin import ettiği her modül (brax, torch, onnxruntime...) bir kez
   import edilir. Eksik paket ya da `mjx` için GPU'yu görmeyen JAX varsa pod hemen `failed` olur.
5. Eğitim `--max-hours` sınırıyla çalışır. `cpu` backend `--workers`'ı podun gerçek çekirdek
   sayısından alır (cgroup / `RUNPOD_CPU_COUNT`).
6. Eğitim bitince (ya da hata olunca) pod sunmaya devam eder. `--linger-hours` dolunca kendini siler.
   Bunun için her podda hazır gelen `runpodctl` ve pod'a özel `RUNPOD_API_KEY` kullanılır; sizin
   anahtarınız poda gitmez.

**Pod süresi:** `launch`, poda kesin bir silinme anı verir (`OPENDUCK_EXPIRES_AT`, `launch` bunu
yazdırır): oluşturma anı + `--setup-hours` + `--max-hours` + `--linger-hours` + 0,5 saat. Konteyner
yeniden başlasa ya da bir şey takılsa bile pod o anda kendini siler. `--keep` ya da
`--linger-hours 0` ile bu sınır yoktur; podu siz silmelisiniz.

`status.json` içinde şunlar bulunur: `phase` (`server` → `code` → `install` → `check` → `training` →
`done`, `failed`, `timeout` ya da `stopped`), `progress`, `steps`/`total_steps`, `eta_s`, `last_record`
(log.jsonl'ın son satırı), `last_log_line`, `log_tail`, `error`, `env` (paket sürümleri, GPU),
`has_policy` ve `expires_at`.

## 5. Maliyet

- Pod çalıştığı (ve silinmediği) sürece saniye bazında ücretlendirilir. Kurulum (≈ 5–10 dk; jax CUDA
  paketleri ≈ 3 GB) da ücretlidir.
- Yaklaşık fiyatlar saat başınadır ve değişir, güncelini `launch.py gpus` gösterir: RTX 4090 ≈ 0.35–0.70 $,
  RTX 5090 / L40S ≈ 0.9–1.0 $, CPU pod vCPU başına ≈ 0.04–0.06 $ (32 vCPU ≈ 1.3–2 $).
- Tipik maliyet *(max-hours + ~0.25) × saatlik ücret* olur; `--wait` kullanılmazsa buna
  *+ linger-hours* eklenir. Kesin üst sınır pod süresidir (yukarıda). `launch` ikisini de yazdırır.

## 6. Podu durdurma / silme

- `python3 tools/runpod/launch.py terminate <ad>` (ya da `--pod-id ID`) podu siler.
  `terminate --all-openduck` adı `openduck` ile başlayan ve `runs/*/runpod/pod.json` kayıtlarındaki
  bütün podları siler.
- Console → *Pods* → pod → *Terminate*. *Stop* GPU'yu bırakır ama konteyner diskini (sonuçları)
  siler; varsa hacim (volume) diski ücretlenmeye devam eder. İşiniz bittiyse *Terminate* kullanın.
- `launch --wait` sırasında Ctrl-C yalnızca beklemeyi bırakır. Pod çalışmaya devam eder, sonra
  `status`/`fetch`/`terminate` kullanın.
- `--wait` şu durumlarda podu kendisi siler: eğitim bitti/hata verdi ve sonuçlar indirildi; HTTP
  25 dk (`--boot-timeout-min`) içinde hiç açılmadı; RunPod podu `ERROR`/`EXITED` gösteriyor.
  Sonuçlar indirilemezse podu silmez, uyarı verir; pod, pod süresi dolunca kendini siler.

## 7. Sorun giderme

- **"yerleştirilemedi (400)" / hiçbir seçenekte pod açılamadı:** `gpus` ile stoğa bakın,
  `--gpu`/`--cloud` değiştirin. Bütün seçenekler 400 veriyorsa son hata ayrıntısına bakın (istek
  sorunu olabilir); hepsi 403 ise anahtarın pod yazma yetkisi yoktur. 402 = bakiye yetersiz.
- **HTTP hiç açılmıyor:** `logs <ad>` ile bakın. `--wait` 25 dk (`--boot-timeout-min`) içinde HTTP
  açılmazsa logları yazdırır ve podu siler.
- **`failed`:** `status <ad>` hatayı ve log sonunu gösterir. `fetch <ad>` ile
  `runs/<ad>/runpod/entry.log` ve `train.log` indirilir.
- **git modu:** "origin/<dal> içinde eksik" uyarısı alırsanız commit + push yapın ya da `--source inline` kullanın.
