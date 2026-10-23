# tools/bench: Adım 0 tezgâh betiği ve kaydedici

Kullanım kılavuzu: [`docs/adim0-tezgah-testi.md`](../../docs/adim0-tezgah-testi.md).

| Dosya | Ne |
|---|---|
| `platformio.ini` | ESP32-S3-DevKitC-1 N16R8 derleme ayarları. Ortamlar: `bench` (UART portu), `bench_usb` (yerel USB CDC) |
| `bench/bench.ino` | Arduino IDE giriş dosyası; kart ayarları başlıkta yazılı |
| `bench/bench_main.cpp` | Programlar, güvenlik, komutlar, satır protokolü |
| `bench/drivers.h` | PCA9685, INA226 ve ADS1115 doğrudan yazmaç sürücüleri |
| `logger.py` | PC kaydedici (pyserial). Adlı testleri ya da planları koşturur, CSV + meta JSON yazar, `--pack` ile zip'ler |
| `tests/` | Kaydedici testi. Sahte seri port, `tools/sysid/synth.py` akışını oynatır |

Derleme ve yükleme:

```
pio run -d tools/bench -t upload
```

Derlenip denenen ortam: platformio/espressif32 8.0.0, Arduino-ESP32 3.3.12.

Satır biçimi `tools/sysid/logio.py` başlığında. Uydurma:

```
python -m tools.sysid.fit --logs bench_logs --mass 1.95
```
