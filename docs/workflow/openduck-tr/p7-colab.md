# P7: Google Colab'da GPU eğitimi ve ilk getup koşusu (2026-11-06)

RunPod bakiyesi bitince GPU eğitimleri Google Colab'a taşındı (`colab` CLI, Gemini üyeliğiyle gelen compute unit).
Araçlar ve kullanım: `tools/colab/README.md`.

## 1. Altyapı

- VM: `--gpu G4` = RTX PRO 6000 Blackwell Server Edition (96 GB), 48 vCPU, 176 GB RAM; ~10 CU/sa.
- Kurulum (`tools/colab/setup.py`): git clone + uv venv, `jax[cuda12]==0.12.0.1 mujoco==3.16.0 mujoco-mjx==3.16.0
  brax==0.14.2 playground==0.2.0 onnx onnxruntime`; ~25 sn. VM'in Python'u 3.13, venv 3.12.
- Hız (8192 ortam, `tr35_b2`): walk **197k SPS** (RunPod 4090: 147k), getup **25.3k SPS** (4090: < 28k). walk'ta
  G4 1.34× hızlı; getup'ta (solver 8 iterasyon, çarpışma kutuları) fark yok.
- VM ömrü: bağlı kernel istemcisi yoksa ~18–20 dk'da geri alınıyor (kernel meşgul olsa da, çıktı bassa da). İlk üç
  koşu bu yüzden ilk değerlendirmeden önce kayboldu (~17 CU). Çözüm `tools/colab/keeper.sh`: dördüncü koşu 11 sa
  ayakta kaldı; ana bağlantı 3.5 sa sonra sessizce takıldı, VM'i kalan ~7.5 sa 4 dakikalık kısa bağlantılar tuttu.

## 2. getup koşusu (colab-getup, 1e9 adım, varsayılan ayarlar)

Komut: `OPENDUCK_ROBOT=tr35_b2 OPENDUCK_KAPPA=0.5 python -m rl.train_mjx --task getup --num-envs 8192
--num-timesteps 1e9` (TUNE yok; getup varsayılanı gürültü 0.5 + `torque_sat` cezası). 39 860 s, ~108 CU.

| Adım | getup_success | eval ödül | torque_sat_steps (/600) | ONNX farkı |
|---|---|---|---|---|
| 0 | 0.070 | 11.1 | 90 | 2.1e-6 |
| 53M | 0 | 29.6 | 573 | 4.1e-6 |
| 106M | 0 | 48.6 | 595 | 5.4e-6 |
| 528M | 0 | 56.6 | 593 | 6.4e-3 |
| 791M | 0 | 49.4 | 598 | 1.1e-1 |
| 1002M | 0 | 53.4 | 598 | 4.8e-1 |

Sonuç: **kalkma öğrenilmedi.** Başarı ilk değerlendirmeden sonra hep 0 (rastgele başlangıç politikası 0.07);
ödül 106M'den sonra düz; bölümün %99'unda en az bir servo doygun (`max |tau_ratio| > torque_sat_frac`).

Kök neden (son `policy.onnx`, rastgele normal gözlemle): ham aksiyon medyanı |a| ≈ 100–600, en büyükler 1e5–1e6;
`action_clip` 1.0. Her aksiyon kırpılıyor → bang-bang, servolar sürekli tork sınırında. Dağılım tanh'sız "normal"
ve kırpma ortamda olduğu için kırpma ötesine itilen ortalamanın cezası yok; ortalama sınırsız büyüyor. ONNX/JAX
farkının adım adım büyümesi (2e-6 → 0.48; `export_c` C/ORT farkı da 0.5) aynı büyüklüklerin float32 hatası.
p7-notlar §2'de main'de görülen "aksiyonlar ±1.5 kırpmanın çok dışında (2.8–4.0)" bunun hafif hâli.
Bu koşunun `policy.onnx`'i kullanılamaz.

## 3. Açık

- Aksiyon patlaması: kırpma öncesi ham aksiyona ceza (en küçük müdahale), tanh'lı dağılım ya da ağ çıktısında
  ortalama sınırı. Karar verilmedi.
- Düzeltmeden sonra önce kısa tanı koşusu (~2e8, ~2 sa, ~20 CU): success > 0 ve |a| makul mü; sonra 1e9.
- main/walk'ta da aynı eğilim var mı: mevcut koşuların `policy.onnx`'lerinde ham |a| ölçülmeli.
