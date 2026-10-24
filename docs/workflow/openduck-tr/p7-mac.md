# P7: Mac'te (yerel, CPU) eğitim

RunPod bakiyesi yetmediğinde eğitimler Mac'te CPU ile sürer: `rl.train` (PyTorch PPO + MuJoCo C, çok süreçli). MJX/JAX
GPU yolu Mac'te kullanılmaz (Apple GPU için JAX desteği deneysel, MJX koşmuyor). Ortam, ödüller ve gözlem GPU
eğitimiyle aynıdır (CPU/MJX eşlik testleri); PPO kurulumu farklıdır (512 ortam × 24 adım, rsl-rl tarzı).

## 1. Kurulum (bir kez)

```bash
cd ~/.../openduck-tr            # repo kökü
git pull origin main
python3.12 -m venv .venv-cpu && source .venv-cpu/bin/activate
pip install mujoco==3.16.0 torch onnx onnxruntime numpy scipy imageio pillow
```

Bulut konteynerinde doğrulanan sürümler: Python 3.12, mujoco 3.16.0, torch 2.14, onnx 1.23, onnxruntime 1.31, numpy 2.5.

## 2. Hız ölçümü (1–2 dk)

```bash
export OPENDUCK_ROBOT=tr35_b2 OPENDUCK_KAPPA=0.5
python -m rl.train --task walk --out runs/mac-bench --max-steps 3e5 --save-interval 1000
```

Satırlardaki `fps` değeri SPS'tir. Bulut konteynerinde 4 çekirdekte ~6k (çekirdek başına ~1.5k). `--workers`
varsayılanı kullanılabilir çekirdek sayısıdır.

## 3. Eğitimler

Uzun koşularda Mac uyumasın: komutun başına `caffeinate -dimsu` ekleyin. Ödül ayarları GPU taramasından
(`p7-notlar.md` §3); `OPENDUCK_TUNE` değerleri run `config.json`'una `TUNE` olarak yazılır.

```bash
export OPENDUCK_ROBOT=tr35_b2 OPENDUCK_KAPPA=0.5
L='"torque_ratio": -0.1, "joint_acc": -2.5e-7, "feet_phase": 3.0'

# walk (taban çizgisi)
OPENDUCK_TUNE="{\"walk\": {$L}}" caffeinate -dimsu \
  python -m rl.train --task walk --out runs/mac-walk --max-steps 3e8 --init-noise-std 0.5

# main (yürüme + otur/kalk + jest)
OPENDUCK_TUNE="{\"main\": {$L}}" caffeinate -dimsu \
  python -m rl.train --task main --out runs/mac-main --max-steps 1e9 --init-noise-std 0.5

# getup (görev tanımı zaten gürültü 0.5 ve tork doygunluğu cezası kullanır)
caffeinate -dimsu python -m rl.train --task getup --out runs/mac-getup --max-steps 1e9
```

Yarıda kalan koşu devam eder: aynı komuta `--resume runs/<ad>/latest.pt` eklenir.

## 4. Kontrol

- İlerleme: terminal satırları ve `runs/<ad>/log.jsonl`. Her checkpoint'te `policy.onnx` ve `c/` (firmware C dizileri)
  güncellenir.
- Yürüyüşü görmek: `python -m rl.play --policy runs/<ad>/policy.onnx --demo walk --gif walk.gif --no-shadows`
  (main için `--demo sit`; getup için `--demo getup --getup-policy ...`).
- Titreme / tork kontrolü (`p7-notlar.md` §3 ölçütleri): aksiyon adım farkı küçük (< 0.1), bacak servoları stall'da
  değil (doygunluk < %5).
