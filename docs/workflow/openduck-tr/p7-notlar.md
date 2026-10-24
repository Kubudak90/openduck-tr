# P7: ön sürüm eğitimleri (RunPod) ve ödül ayarı — notlar

Girdi: `p5b-notlar.md` (dalga-2 CAD, türetilmiş sabitler), kullanıcı kararı (2026-10-24): RTX 4090, yakın veri merkezi,
yavaş ağlı pod hemen kapatılır.

## 1. Altyapı

- Podlar: RTX 4090 Secure, EU-RO-1 (Romanya), 0.89 $/sa. Ağ 3.7–7 Gbps; kurulum (git + pip) 1–2 dk.
- `launch.py --datacenter` ve `--min-download-mbps` (REST v1). `--min-cuda` (≥ 12.4) filtresiyle EU'da 4090 kapasitesi
  sıfır döndü; `--min-cuda ''` ile açıldı (pod kendi kontrolünde JAX GPU'yu doğrular). `minDownloadMbps` filtresi ayrıca
  denenmedi; açılan makinelerin hızı GraphQL `pod.machine.maxDownloadSpeedMbps` ile kontrol edildi.
- `OPENDUCK_TUNE` (JSON) ödül ağırlıklarını görev başına değiştirir; değerler run `config.json`'unda `TUNE` olarak kalır.
  `launch.py --env KEY=DEGER` poda iletir.
- Hız (8192 ortam): walk 147k SPS (2e8 ≈ 27 dk), main 71k SPS, getup < 28k SPS (1e9 ≈ 10+ sa).

## 2. İlk koşu: varsayılan ayarlar (pipe-walk 2e8, pipe-main ve pipe-getup durduruldu)

walk düşmüyor ama yürümüyor: ileri komutun %45'i, dönüş ters yönde, 3.0 Hz titreme (aksiyon spektrumunun %95'i 2–10 Hz),
aksiyon adım farkı ortalaması 0.40 (× 0.3 rad), bacak servoları sürenin %33'ünde stall torkunda.

main 1e8'de: hareket yok; iki kalçanın yaw ve roll servoları sürekli stall'da, bacaklar birbirine karşı geriliyor
(roll hedefi ~15°, yaw ~11° ulaşılamaz; ayaklar sürtünmeyle sabit). Aksiyonlar ±1.5 kırpmanın çok dışında (2.8–4.0).

Kök nedenler (ölçümle):

1. Keşif gürültüsü `init_noise_std` 1.0 = 50 Hz'de 0.3 rad beyaz gürültü. P-bandı 3° servolarda her adım tam tork; politika
   bang-bang sürmeyi öğrenir. getup tanımı bunu zaten not edip 0.5 kullanıyordu; walk/main 1.0'da kalmıştı.
2. `torque_ratio` −2e-3: bölüm başına etkisi ~0.1, izleme ödülünün ~1/250'si. Stall'da kas kasma (co-contraction) bedava.
3. `joint_acc` × lt⁴ (= 26): κ ile yavaşlayan hareket için doğru, ama keşif gürültüsünün ivmesi κ ile ölçeklenmez. Gürültü
   0.5'te eğitim başında bölüm başına −45 (izleme kazancı kadar) ve std 0.04'e çöker: politika hareketsiz durur.

## 3. Ayar taraması (walk; puanlama: sabit komutla 6 s, ilk 1 s hariç)

Ölçütler: izleme oranı (ulaşılan / komut; ½ vmax ileri, vmax ileri, ½ wzmax dönüş, ½ vmin geri, ½ vymax yana), aksiyon adım
farkı |da|, 2–10 Hz payı, tork / tau_s ortalaması, doygunluk (> 0.95) süresi.

| Koşu | Ayar (gürültü 0.5, aksi belirtilmedikçe) | İleri ½ / max | Dönüş | Geri | Yana | \|da\| | Doygun |
|---|---|---|---|---|---|---|---|
| pipe-walk | varsayılan (gürültü 1.0) | 0.45 / 0.39 | −0.19 | −0.04 | 0.32 | 0.40 | %33 |
| A | — | 0 / 0 | 0.15 | 0 | 0 | 0.004 | 0 |
| B | action_rate −0.1 | 0 / 0 | 0.19 | 0 | 0 | 0.003 | 0 |
| C | B + torque −0.3 | düşüyor (6 s'de 8–11) | | | | 0.82 | %17 |
| D | B + torque −1.0 | 0 / 0 | 0.11 | 0 | 0 | 0.002 | 0 |
| E | torque −0.3, joint_acc × lt² | 0 / 0 | 0.18 | 0 | 0 | 0.003 | 0 |
| F | torque −0.3, joint_acc × 1 (xl330) | 0 / **1.02** | 0.06 | 0 | 0 | 0.012 | 0 |
| G | E + entropi 1e-2 | 0.78 / 0.40 | 0.13 | −0.02 | 0.18 | 0.18 | %16 |
| H | E, gürültü 1.0 | 0.18 / 0.02 | 0.16 | −0.02 | −0.03 | 0.21 | %23 |
| I | F + feet_phase 3 (3e8) | 0.60 / 0.13 | −0.03 | 0.27 | 0.54 | 0.06 | %1 |
| J | F + izleme lin/ang 4 (3e8) | 0.77 / 0.74 | 0.33 | 0.05 | 0.01 | 0.02 | 0 |
| L | I, torque −0.1 (3e8) | **0.86 / 0.85** | −0.27 | 0.29 | 0.41 | 0.08 | %2 |
| M | I + J (3e8) | 0.64 / 0.48 | 0.29 | 0.04 | −0.01 | 0.02 | %1 |
| N | L + izleme ang 4 (3e8) | 0.67 / 0.61 | −0.02 | 0.01 | 0.10 | 0.06 | %1 |
| O | N + hip_pose −0.05 (3e8) | **0.93** / 0.65 | −0.11 | 0.03 | 0.40 | 0.06 | %1 |
| P | L + izleme lin/ang 4 (3e8) | ~0 / 0.08 | ~0 | ~0 | ~0 | 0.04 | %2 |

Gözlemler:

- Titreme ve doygunluk gürültü 0.5 + anlamlı tork cezası ile kalkar; gürültü 1.0'da (H) tork cezası tek başına yetmez.
- Keşfe tepki veren her ceza (joint_acc × lt⁴, torque −0.3/−1.0) eğitim başında std'yi çökertir. Küçük komutta durmak
  izleme ödülünün ~%30'unu verir; politika yalnızca durmanın açıkça kötü olduğu yerde (F: vmax) yürür.
- feet_phase × 3 adım ritmini öğretir (L: 57/60), ama yürürken gövde yaw'da salınır ve dönüş izlemesi hareketsiz
  durmaktan kötüdür (L: 7.6 / durma 15.8).
- Dönüş hiçbir ayarda öğrenilmedi (izleme ağırlığı × 2, hip_pose cezası ¼ dahil). Ağırlık dışı bir neden olabilir
  (hip_yaw sınırı / servo hızı, taban sürtünmesi); açık madde.
- Her ayar tek tohumla koşuldu; L ile O arasındaki fark gürültü içinde olabilir.

## 4. Seçilen ayar ve sonraki adımlar

Ön sürüm ayarı L: gürültü 0.5, `torque_ratio` −0.1, `joint_acc` −2.5e-7 (xl330 değeri, κ ölçeklemesi yok),
`feet_phase` 3.0 (`OPENDUCK_TUNE`). main ön denemesi (t7-mainL, 3e8) bu ayarla koşuldu; getup varsayılanında (zaten 0.5
gürültü ve `torque_sat` cezası) bırakıldı.

Açık:
- Dönüş (yukarıda).
- Ayar, `tr35_constants` varsayılanına alınmadı (TUNE ile verilir); main/getup sonuçlarından sonra karar.
- `--env-precision default` (TF32) hızı ölçülmedi; getup < 28k SPS (1e9 ≈ 10+ sa).
- Mac'te CPU eğitimi: `p7-mac.md`.

## 5. Harcama (RunPod, 0.89 $/sa)

İlk koşu $1.40, tur 1 $1.42, tur 2 $1.76, tur 3 $2.40, tur 4 + main/getup ön ~$2.5 (main ön denemesi bakiye
bitince checkpoint'le kesildi/kesilecek). Hesap bakiyesi $10'du.
