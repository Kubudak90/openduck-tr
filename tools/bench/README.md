# tools/bench: Adım 0 tezgâh betiği ve kaydedici

Kullanım kılavuzu: [`docs/adim0-tezgah-testi.md`](../../docs/adim0-tezgah-testi.md).

| Dosya | Ne |
|---|---|
| `platformio.ini` | ESP32-S3-DevKitC-1 N16R8 derleme ayarları. Ortamlar: `bench` (UART portu), `bench_usb` (yerel USB CDC) |
| `bench/bench.ino` | Arduino IDE giriş dosyası; kart ayarları başlıkta yazılı |
| `bench/bench_main.cpp` | Programlar, güvenlik, komutlar, satır protokolü. Ses eki: `voice` ve `gest` programları, `v ...` komutları |
| `bench/drivers.h` | PCA9685, INA226 ve ADS1115 doğrudan yazmaç sürücüleri |
| `bench/df2301q.h` | DF2301Q UART çerçevesi (kur/çöz; DFRobot_DF2301Q ile aynı biçim), Arduino'suz |
| `logger.py` | PC kaydedici (pyserial). Adlı testleri ya da planları koşturur, CSV + meta JSON yazar, `--pack` ile zip'ler. `ses-*` testleri G-SES-a protokolünü yürütür |
| `voice_report.py` | G-SES-a analizi: kapı büyüklükleri, `configs/komut_kimlik.json`, `configs/ses_gecikme.json` |
| `tests/` | Kaydedici testi (sahte seri port, `tools/sysid/synth.py`), ses eki testi (sahte `voice` firmware'i, sentetik WAV, çerçeve çözücü) |

Ses modülü (G-SES-a, spec eki K8): kılavuz §10. Pinler: GPIO12 UART1 RX (1 kΩ seri), GPIO11 UART1 TX, GPIO13 kafa
LED'i; yalnız tezgâhta GPIO7 piezo, GPIO6 fototransistör (`set pin_ldr=6`). İsteğe bağlı ikinci PCA9685 0x41'de, 50 Hz.

```
python tools/bench/logger.py --port PORT --servo DF2301Q-1 --plan ses
python tools/bench/voice_report.py tablo bench_logs/DF2301Q-1
python tools/bench/voice_report.py rapor bench_logs/DF2301Q-1
```

Derleme ve yükleme:

```
pio run -d tools/bench -t upload
```

Derlenip denenen ortam: platformio/espressif32 8.0.0, Arduino-ESP32 3.3.12.

Satır biçimi `tools/sysid/logio.py` başlığında. Uydurma:

```
python -m tools.sysid.fit --logs bench_logs --mass 1.95
```
