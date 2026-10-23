// OpenDuck-TR tezgah: PCA9685, INA226, ADS1115 icin kucuk dogrudan-yazmac surucu (kutuphane yok).
#pragma once
#include <Arduino.h>
#include <Wire.h>

static bool i2cWrite8(uint8_t a, uint8_t reg, uint8_t v) {
  Wire.beginTransmission(a); Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}
static bool i2cWrite16(uint8_t a, uint8_t reg, uint16_t v) {
  Wire.beginTransmission(a); Wire.write(reg); Wire.write(v >> 8); Wire.write(v & 0xFF);
  return Wire.endTransmission() == 0;
}
static bool i2cRead16(uint8_t a, uint8_t reg, uint16_t &v) {
  Wire.beginTransmission(a); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)a, 2) != 2) return false;
  v = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}
static bool i2cRead8(uint8_t a, uint8_t reg, uint8_t &v) {
  Wire.beginTransmission(a); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)a, 1) != 1) return false;
  v = Wire.read();
  return true;
}
static bool i2cPresent(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }

// ---------------------------------------------------------------- PCA9685
// prescale = round(osc / (4096 f)) - 1 (yalniz SLEEP'te yazilir). Cikis guncellemesi PWM periyodu sonunda.
struct Pca {
  uint8_t addr = 0x40;
  bool ok = false;
  int prescale = 19;
  float osc = 25.0e6f;            // kalibre edilmis osilator (Hz)
  float freq() const { return osc / (4096.0f * (prescale + 1)); }
  float tickUs() const { return 1.0e6f / (4096.0f * freq()); }
  bool begin(uint8_t a) {
    addr = a;
    ok = i2cPresent(a);
    if (!ok) return false;
    i2cWrite8(addr, 0x00, 0x30);  // MODE1: SLEEP + AI
    i2cWrite8(addr, 0x01, 0x04);  // MODE2: OUTDRV totem-pole
    setPrescale(prescale);
    allOff();
    return true;
  }
  void setPrescale(int p) {
    if (p < 3) p = 3;
    if (p > 255) p = 255;
    prescale = p;
    uint8_t m1 = 0;
    i2cRead8(addr, 0x00, m1);
    i2cWrite8(addr, 0x00, (m1 & 0x7F) | 0x10);  // sleep
    i2cWrite8(addr, 0xFE, (uint8_t)p);
    i2cWrite8(addr, 0x00, 0x20);                // uyandir, AI
    delayMicroseconds(600);
    i2cWrite8(addr, 0x00, 0xA0);                // RESTART + AI
  }
  bool setCounts(int ch, int on, int off) {
    Wire.beginTransmission(addr);
    Wire.write(0x06 + 4 * ch);
    Wire.write(on & 0xFF); Wire.write((on >> 8) & 0x0F);
    Wire.write(off & 0xFF); Wire.write((off >> 8) & 0x1F);  // bit4 = tam KAPALI
    return Wire.endTransmission() == 0;
  }
  void chOff(int ch) { setCounts(ch, 0, 0x1000); }
  void allOff() {
    Wire.beginTransmission(addr);
    Wire.write(0xFA); Wire.write(0); Wire.write(0); Wire.write(0); Wire.write(0x10);
    Wire.endTransmission();
  }
};

// ---------------------------------------------------------------- INA226
// Yalniz sont gerilimi (2.5 uV/LSB) ve VBUS (1.25 mV/LSB) okunur; akim = Vsont / R (kalibrasyon yazmaci yok).
struct Ina {
  uint8_t addr = 0;
  bool ok = false;
  bool find() {
    for (uint8_t a = 0x40; a <= 0x4F; a++) {
      uint16_t id = 0, die = 0;
      if (!i2cPresent(a)) continue;
      if (i2cRead16(a, 0xFE, id) && id == 0x5449 && i2cRead16(a, 0xFF, die) && (die & 0xFFF0) == 0x2260) {
        addr = a; ok = true; return true;
      }
    }
    ok = false;
    return false;
  }
  // avg: 0=1,1=4,2=16,3=64 ; ct: 0=140us,1=204,2=332,3=588,4=1100us
  void config(int avg, int ct) {
    if (!ok) return;
    uint16_t c = 0x4000 | ((avg & 7) << 9) | ((ct & 7) << 6) | ((ct & 7) << 3) | 7;
    i2cWrite16(addr, 0x00, c);
  }
  bool shuntUv(float &uv) {
    uint16_t r;
    if (!ok || !i2cRead16(addr, 0x01, r)) return false;
    uv = (int16_t)r * 2.5f;
    return true;
  }
  bool busMv(float &mv) {
    uint16_t r;
    if (!ok || !i2cRead16(addr, 0x02, r)) return false;
    mv = r * 1.25f;
    return true;
  }
};

// ---------------------------------------------------------------- ADS1115
// Tek atis, 860 SPS, sirali kanal (engellemesiz: baslat, >= 1.25 ms sonra oku).
struct Ads {
  uint8_t addr = 0;
  bool ok = false;
  static const int NS = 3;
  // slot: 0 AIN0 (NTC kasa, +-4.096 V), 1 AIN1 (pot orta uc ADS yolu, +-4.096), 2 AIN2-AIN3 (pot alt uc, +-0.256)
  const uint8_t mux[NS] = {4, 5, 3};
  const uint8_t pga[NS] = {1, 1, 5};
  const float lsb[NS] = {0.125f, 0.125f, 0.0078125f};  // mV
  float val[NS] = {NAN, NAN, NAN};                     // mV
  uint8_t mask = 7;
  int cur = -1;
  uint32_t t0 = 0;
  bool find() {
    for (uint8_t a = 0x48; a <= 0x4B; a++) {
      uint16_t c;
      if (i2cPresent(a) && i2cRead16(a, 0x01, c)) { addr = a; ok = true; return true; }
    }
    ok = false;
    return false;
  }
  void start(int s) {
    uint16_t c = 0x8000 | (mux[s] << 12) | (pga[s] << 9) | 0x0100 | (7 << 5) | 0x0003;
    i2cWrite16(addr, 0x01, c);
    cur = s;
    t0 = micros();
  }
  int nextSlot(int s) const {
    for (int k = 1; k <= NS; k++) {
      int n = (s + k) % NS;
      if (mask & (1 << n)) return n;
    }
    return -1;
  }
  void poll() {
    if (!ok || !mask) return;
    if (cur < 0) { int s = nextSlot(NS - 1); if (s >= 0) start(s); return; }
    if ((uint32_t)(micros() - t0) < 1250) return;
    uint16_t r;
    if (i2cRead16(addr, 0x00, r)) val[cur] = (int16_t)r * lsb[cur];
    int s = nextSlot(cur);
    if (s >= 0) start(s);
  }
};
