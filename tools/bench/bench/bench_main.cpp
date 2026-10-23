// OpenDuck-TR Adim 0 tezgah betigi (spec 7, plan P6). Tek servo (DUT) testleri, CSV akisi USB seriden.
//
// Donanim: ESP32-S3-DevKitC-1 N16R8 + PCA9685 (0x40, VCC 3.3 V) + INA226 (0x41..0x45, dis sont 2-10 mOhm)
//          + ADS1115 (0x48) + 10k NTC (B3950). Baglanti: docs/adim0-tezgah-testi.md.
// Derleme:  PlatformIO:  pio run -d tools/bench -t upload      (platformio.ini: UART portu, 921600 baud)
//           Arduino IDE 2.x + esp32 core 3.x: kart "ESP32S3 Dev Module", USB CDC On Boot: Disabled (UART portu)
//           ya da Enabled (USB portu), Flash 16MB (QIO 80MHz), PSRAM: OPI PSRAM, Partition: 16M Flash (3MB APP),
//           Upload 921600, CPU 240 MHz. Seri monitor 921600 baud, satir sonu: LF.
// Protokol (satirlar, kolonlar): tools/sysid/logio.py. PC kaydedici: tools/bench/logger.py.
// Programlar tools/sysid/synth.py ile ayni komut dizisini, olaylari ve kolonlari uretir (sysID testleri onu kullanir).
// Guvenlik: us yumusak sinirlari, akim (surekli + tepe), kasa sicakligi, ray asimi, zaman asimi, host kalp atisi,
//           '!' karakteri ya da BOOT dugmesi = E-stop (PCA cikislari kapali + OE yuksek). Fiziksel anahtar asil E-stop.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <math.h>
#include "driver/gpio.h"
#include "drivers.h"

#define FW_VER "bench-1"
#ifndef BENCH_BAUD
#define BENCH_BAUD 921600
#endif

static const int PIN_SDA = 17, PIN_SCL = 18, PIN_OE = 21, PIN_CAP_OSC = 16, PIN_CAP_DUT = 15, PIN_BOOT = 0;
static const float G = 9.80665f;

// ---------------------------------------------------------------- ayarlar (NVS'te saklanir)
struct Cfg {
  uint32_t magic;
  char cls[4];
  float div, rail_div, rc_ms, shunt_mohm, lever_mm, mass_g, lever_g, vcc;
  int freq, prescale, rate, os, ch;
  float osc;
  int pin_pot, pin_ph, pin_rail, pin_amb;
  float us_lo, us_hi, us_c, dpu, vrail, ilim, ilim_peak, tlim, vmax, hbto;
  int ads_mask;
  float ntc_r25, ntc_b, ntc_rfix, amb_c;
};
static Cfg C;
static const uint32_t MAGIC = 0x0D0C0003;

static void setClass(const char* c) {
  strncpy(C.cls, c, 3); C.cls[3] = 0;
  if (!strcmp(c, "es"))      { C.us_lo = 700; C.us_hi = 2300; C.dpu = 0.0906f; C.freq = 50; C.vrail = 6.0f; C.ilim = 1.2f; C.ilim_peak = 2.0f; }
  else if (!strcmp(c, "mg")) { C.us_lo = 600; C.us_hi = 2400; C.dpu = 0.095f;  C.freq = 50; C.vrail = 6.0f; C.ilim = 0.8f; C.ilim_peak = 1.5f; }
  else if (!strcmp(c, "sg")) { C.us_lo = 600; C.us_hi = 2400; C.dpu = 0.095f;  C.freq = 50; C.vrail = 6.0f; C.ilim = 0.7f; C.ilim_peak = 1.2f; }
  else { strcpy(C.cls, "ds"); C.us_lo = 500; C.us_hi = 2500; C.dpu = 0.090f; C.freq = 300; C.vrail = 6.5f; C.ilim = 3.0f; C.ilim_peak = 6.0f; }
  C.us_c = 1500;
}
static bool isAnalog() { return !strcmp(C.cls, "mg") || !strcmp(C.cls, "sg"); }

static void cfgDefaults() {
  memset(&C, 0, sizeof C);
  C.magic = MAGIC;
  setClass("ds");
  C.div = 47.0f / 147.0f; C.rail_div = 47.0f / 147.0f; C.rc_ms = 3.2f; C.shunt_mohm = 5.0f;
  C.lever_mm = 100; C.mass_g = 0; C.lever_g = 0; C.vcc = 3.30f;
  C.prescale = 19; C.rate = 500; C.os = 8; C.ch = 0; C.osc = 25.0e6f;
  // istege bagli kanallar kapali baslar (bos pin rastgele okur): baglayinca  set pin_ph=2 pin_rail=4 pin_amb=5 ads_mask=7
  C.pin_pot = 1; C.pin_ph = -1; C.pin_rail = -1; C.pin_amb = -1;
  C.tlim = 75; C.vmax = 7.6f; C.hbto = 0; C.ads_mask = 1;
  C.ntc_r25 = 10000; C.ntc_b = 3950; C.ntc_rfix = 10000; C.amb_c = NAN;
}

struct KeyDef { const char* k; char t; void* p; };
static const KeyDef KEYS[] = {
  {"div", 'f', &C.div}, {"rail_div", 'f', &C.rail_div}, {"rc_ms", 'f', &C.rc_ms}, {"shunt_mohm", 'f', &C.shunt_mohm},
  {"lever_mm", 'f', &C.lever_mm}, {"mass_g", 'f', &C.mass_g}, {"lever_g", 'f', &C.lever_g}, {"vcc", 'f', &C.vcc},
  {"rate", 'i', &C.rate}, {"os", 'i', &C.os}, {"ch", 'i', &C.ch}, {"osc", 'f', &C.osc},
  {"pin_pot", 'i', &C.pin_pot}, {"pin_ph", 'i', &C.pin_ph}, {"pin_rail", 'i', &C.pin_rail}, {"pin_amb", 'i', &C.pin_amb},
  {"us_lo", 'f', &C.us_lo}, {"us_hi", 'f', &C.us_hi}, {"us_c", 'f', &C.us_c}, {"dpu", 'f', &C.dpu},
  {"vrail", 'f', &C.vrail}, {"ilim", 'f', &C.ilim}, {"ilim_peak", 'f', &C.ilim_peak}, {"tlim", 'f', &C.tlim},
  {"vmax", 'f', &C.vmax}, {"hbto", 'f', &C.hbto}, {"ads_mask", 'i', &C.ads_mask},
  {"ntc_r25", 'f', &C.ntc_r25}, {"ntc_b", 'f', &C.ntc_b}, {"ntc_rfix", 'f', &C.ntc_rfix}, {"amb_c", 'f', &C.amb_c},
};

// ---------------------------------------------------------------- donanim
static Pca pca;
static Ina ina;
static Ads ads;
static Preferences prefs;
static uint16_t* g_buf = nullptr;
static uint32_t* g_bt = nullptr;
static const int BURST_MAX = 12000;

static volatile int64_t capRise = 0;
static volatile float capW = NAN;
static volatile int64_t oscFirst = 0, oscLast = 0;
static volatile uint32_t oscN = 0;
static volatile bool oscOn = false;
static void IRAM_ATTR isrCapDut() {
  int64_t t = esp_timer_get_time();
  if (gpio_get_level((gpio_num_t)PIN_CAP_DUT)) capRise = t;
  else if (capRise) { float w = (float)(t - capRise); if (w > 100 && w < 3000) capW = w; }
}
static void IRAM_ATTR isrOsc() {
  if (!oscOn) return;
  int64_t t = esp_timer_get_time();
  if (!oscN) oscFirst = t;
  oscLast = t;
  oscN = oscN + 1;
}

// ---------------------------------------------------------------- calisma durumu
struct Smp { int64_t t; float pot, ph, rail, ima, vbus, adsw, adslo, ntc, amb, cap; };
static Smp S;
static bool g_running = false, g_abort = false, g_mute = false, g_pwm = false, g_tsoft = false, g_mon = false;
static char g_why[32] = "";
static int64_t g_t0 = 0, g_next = 0, g_lastHb = 0;
static uint32_t g_per = 2000;
static float g_tmax = 0, g_cmd = NAN, g_ilim = 3.0f, g_jog = 0;
static int g_seg = -1, g_ocnt = 0, g_pcnt = 0, g_vcnt = 0, g_rate = 500;
static uint32_t g_tick = 0;
static char g_line[192];
static int g_ll = 0;
static char g_cmdq[192];
static bool g_cmdReady = false;
static char g_ans[96];
static volatile bool g_ansReady = false;
// program parametreleri
static char g_ak[20][16], g_av[20][40];
static int g_na = 0;
static String g_used;

static float adcMv(int pin, int n) {
  if (pin < 0 || n <= 0) return NAN;
  uint32_t s = 0;
  for (int i = 0; i < n; i++) s += analogReadMilliVolts(pin);
  return (float)s / n;
}
static float ntcC(float mv) {
  float vcc = C.vcc * 1000.0f;
  if (!(mv > 20.0f && mv < vcc - 20.0f)) return NAN;
  float r = C.ntc_rfix * mv / (vcc - mv);
  float inv = 1.0f / 298.15f + logf(r / C.ntc_r25) / C.ntc_b;
  return 1.0f / inv - 273.15f;
}

static void sample(int64_t t) {
  S.t = t;
  S.pot = adcMv(C.pin_pot, C.os);
  S.ph = adcMv(C.pin_ph, C.os);
  S.rail = adcMv(C.pin_rail, 2) / 1.0f;
  if (g_tick % 25 == 0 || isnan(S.amb)) S.amb = ntcC(adcMv(C.pin_amb, 4));
  float uv, mv;
  S.ima = (ina.ok && ina.shuntUv(uv)) ? uv / C.shunt_mohm : NAN;   // uV / mOhm = mA
  S.vbus = (ina.ok && ina.busMv(mv)) ? mv : NAN;
  ads.mask = C.ads_mask;
  ads.poll();
  S.ntc = ads.ok && (ads.mask & 1) ? ntcC(ads.val[0]) : NAN;
  S.adsw = ads.ok && (ads.mask & 2) ? ads.val[1] : NAN;
  S.adslo = ads.ok && (ads.mask & 4) ? ads.val[2] * 1000.0f : NAN;
  S.cap = capW;
  if (!isnan(C.amb_c)) S.amb = C.amb_c;
}

// ---------------------------------------------------------------- PWM / E-stop
static void pwmOff() { digitalWrite(PIN_OE, HIGH); g_pwm = false; }
static void setUs(float us) {
  if (us < C.us_lo) us = C.us_lo;
  if (us > C.us_hi) us = C.us_hi;
  float tick = pca.tickUs();
  int cnt = lroundf(us / tick);
  if (cnt > 4000) cnt = 4000;
  float q = cnt * tick;
  if (q != g_cmd || !g_pwm) { pca.setCounts(C.ch, 0, cnt); g_cmd = q; }
  if (!g_pwm) { digitalWrite(PIN_OE, LOW); g_pwm = true; }
}
static void killOutputs() { pca.allOff(); digitalWrite(PIN_OE, HIGH); g_pwm = false; }
static void abortRun(const char* why) {
  killOutputs();
  if (!g_abort) Serial.printf("W,DUR %s\n", why);
  g_abort = true;
  strncpy(g_why, why, sizeof g_why - 1);
}

static void printCfg() {
  Serial.printf("CFG,cls=%s,div=%.5f,rail_div=%.5f,rc_ms=%.2f,shunt_mohm=%.3f,lever_mm=%.1f,mass_g=%.1f,lever_g=%.1f,"
                "vcc=%.3f,freq=%d,prescale=%d,pwm_hz=%.2f,tick_us=%.4f,osc=%.0f,rate=%d,os=%d,ch=%d,pin_pot=%d,pin_ph=%d,"
                "pin_rail=%d,pin_amb=%d,us_lo=%.0f,us_hi=%.0f,us_c=%.0f,dpu=%.4f,vrail=%.2f,ilim=%.2f,ilim_peak=%.2f,"
                "tlim=%.1f,vmax=%.2f,hbto=%.1f,ads_mask=%d,ntc_b=%.0f,amb_c=%.2f,ina=0x%02X,ads=0x%02X,pca=%d,fw=%s\n",
                C.cls, C.div, C.rail_div, C.rc_ms, C.shunt_mohm, C.lever_mm, C.mass_g, C.lever_g, C.vcc, C.freq,
                pca.prescale, pca.freq(), pca.tickUs(), pca.osc, C.rate, C.os, C.ch, C.pin_pot, C.pin_ph, C.pin_rail,
                C.pin_amb, C.us_lo, C.us_hi, C.us_c, C.dpu, C.vrail, C.ilim, C.ilim_peak, C.tlim, C.vmax, C.hbto,
                C.ads_mask, C.ntc_b, C.amb_c, ina.ok ? ina.addr : 0, ads.ok ? ads.addr : 0, pca.ok ? 1 : 0, FW_VER);
}

// ---------------------------------------------------------------- seri giris
static void trim(char* s) {
  int n = strlen(s);
  while (n && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = 0;
  char* p = s;
  while (*p == ' ') p++;
  if (p != s) memmove(s, p, strlen(p) + 1);
}
static void lineIn(char* s) {
  trim(s);
  if (!*s) return;
  if (!strcmp(s, "hb")) { g_lastHb = esp_timer_get_time(); return; }
  if (g_running) {
    if (!strcmp(s, "stop") || !strcmp(s, "x")) { abortRun("kullanici"); return; }
    if (!strncmp(s, "ans", 3)) {
      const char* v = s + 3;
      while (*v == ' ') v++;
      strncpy(g_ans, v, sizeof g_ans - 1); g_ans[sizeof g_ans - 1] = 0;
      g_ansReady = true; return;
    }
    if (!strcmp(s, "ok")) { strcpy(g_ans, "ok"); g_ansReady = true; return; }
    if (s[0] == '+' || s[0] == '-') { g_jog += atof(s); return; }
    Serial.println("W,mesgul: program calisiyor (stop | ! | ans <yanit> | +N/-N)");
    return;
  }
  strncpy(g_cmdq, s, sizeof g_cmdq - 1); g_cmdq[sizeof g_cmdq - 1] = 0;
  g_cmdReady = true;
}
static void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '!') { abortRun("estop"); g_ll = 0; continue; }
    if (c == '\r') continue;
    if (c == '\n') { g_line[g_ll] = 0; g_ll = 0; lineIn(g_line); continue; }
    if (g_ll < (int)sizeof g_line - 1) g_line[g_ll++] = c;
  }
}

// ---------------------------------------------------------------- program cercevesi
static void addUsed(const char* k, const char* v) {
  if (g_used.length()) g_used += ",";
  String s(v);
  s.replace(",", ";");                   // liste degerleri (points=-60;-30;...) E satirini bolmesin
  g_used += k; g_used += "="; g_used += s;
}
static const char* argv_(const char* k) {
  for (int i = 0; i < g_na; i++) if (!strcmp(g_ak[i], k)) return g_av[i];
  return nullptr;
}
static float P(const char* k, float def) {
  const char* v = argv_(k);
  char b[24];
  if (v) { addUsed(k, v); return atof(v); }
  snprintf(b, sizeof b, "%g", def); addUsed(k, b);
  return def;
}
static const char* PStr(const char* k, const char* def) {
  const char* v = argv_(k);
  addUsed(k, v ? v : def);
  return v ? v : def;
}
static int64_t evT() { return g_next; }
static void ev(const char* name, const char* fmt = nullptr, ...) {
  char b[200] = "";
  if (fmt) { va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap); }
  Serial.printf("E,%lld,%s%s%s\n", (long long)evT(), name, fmt ? "," : "", b);
}
static void seg(const char* kind, const char* fmt = nullptr, ...) {
  char b[160] = "";
  if (fmt) { va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap); }
  g_seg++;
  Serial.printf("E,%lld,seg,i=%d,kind=%s%s%s\n", (long long)evT(), g_seg, kind, fmt ? "," : "", b);
}
static void begin(const char* name, float tmax_s) {
  g_rate = (int)P("rate", (float)C.rate);
  if (g_rate < 10) g_rate = 10;
  if (g_rate > 1000) g_rate = 1000;
  g_per = 1000000UL / g_rate;
  g_tmax = P("tmax", tmax_s);
  PStr("tag", "");
  ina.config(g_rate >= 150 ? 1 : 2, 2);   // AVG 4 (ya da 16) x 332 us: okuma ~ tik araligi ortalamasi
  printCfg();
  Serial.printf("S,%s,%s\n", name, FW_VER);
  Serial.printf("E,%lld,params,%s\n", (long long)esp_timer_get_time(), g_used.c_str());
  Serial.println("H,t_us,seg,cmd_us,pot_mv,ph_mv,rail_mv,i_ma,vbus_mv,adsw_mv,adslo_uv,ntc_c,amb_c,cap_us");
  g_t0 = esp_timer_get_time();
  g_next = g_t0 + g_per;
  g_seg = -1; g_tick = 0; g_ocnt = g_pcnt = g_vcnt = 0;
  g_ilim = C.ilim;
}

static void safety(int64_t now) {
  if (digitalRead(PIN_BOOT) == LOW) abortRun("boot_dugmesi");
  if (g_tmax > 0 && (now - g_t0) * 1e-6f > g_tmax) abortRun("zaman_asimi");
  if (C.hbto > 0 && (now - g_lastHb) * 1e-6f > C.hbto) abortRun("host_yok");
  if (!isnan(S.ima)) {
    float a = fabsf(S.ima) * 1e-3f;
    g_ocnt = a > g_ilim ? g_ocnt + 1 : 0;                       // surekli (1.5 s)
    g_pcnt = a > C.ilim_peak ? g_pcnt + 1 : 0;                  // tepe (0.3 s)
    if (g_ocnt > g_rate * 3 / 2) abortRun("asiri_akim");
    if (g_pcnt > g_rate * 3 / 10) abortRun("tepe_akim");
  }
  if (!g_tsoft && !isnan(S.ntc) && S.ntc > C.tlim) abortRun("sicaklik");
  if (!isnan(S.vbus) && S.vbus * 1e-3f > C.vmax) { if (++g_vcnt > 3) abortRun("ray_asimi"); } else g_vcnt = 0;
}

static void printD() {
  char b[220];
  int n = snprintf(b, sizeof b, "D,%lld,%d,%.2f,%.1f,%.1f,%.0f,%.1f,%.0f,%.2f,%.0f,%.2f,%.2f,%.1f\n",
                   (long long)S.t, g_seg, g_pwm ? g_cmd : 0.0f, S.pot, S.ph, S.rail, S.ima, S.vbus, S.adsw,
                   S.adslo, S.ntc, S.amb, S.cap);
  Serial.write((const uint8_t*)b, n);
}

// Bir tik: zamani bekle, olc, komutu yaz (NAN: PWM kapali), guvenlik, D satiri.
static bool tk(float us) {
  for (;;) {
    int64_t now = esp_timer_get_time();
    if (now >= g_next) break;
    pollSerial();
    if (g_next - now > 400) ads.poll();   // ADS donusumleri bekleme sirasinda da ilerlesin
    if (g_abort) return false;
  }
  int64_t now = esp_timer_get_time();
  if (now - g_next > 20LL * g_per) g_next = now;
  g_next += g_per;
  pollSerial();
  sample(now);
  if (g_abort) return false;
  if (isnan(us)) pwmOff(); else setUs(us);
  safety(now);
  if (!g_mute && !g_abort) printD();
  g_tick++;
  return !g_abort;
}
static float cur() { return isnan(g_cmd) ? C.us_c : g_cmd; }
static bool hold(float us, float ms) {
  int n = max(1, (int)lroundf(ms * g_rate / 1000.0f));
  for (int i = 0; i < n; i++) if (!tk(us)) return false;
  return true;
}
static bool ramp(float u0, float u1, float us_per_s) {
  int n = max(1, (int)ceilf(fabsf(u1 - u0) / us_per_s * g_rate));
  for (int i = 1; i <= n; i++) if (!tk(u0 + (u1 - u0) * i / n)) return false;
  return true;
}
static bool moveTo(float u1, float dps = 60.0f) { return ramp(cur(), u1, dps / C.dpu); }
static float quant(float us) { float t = pca.tickUs(); return lroundf(us / t) * t; }

struct Acc {
  double pot = 0, ph = 0, adsw = 0, lo = 0, pot2 = 0, i = 0, vb = 0, rail = 0;
  int n = 0, nlo = 0;
  void add() {
    pot += S.pot; pot2 += (double)S.pot * S.pot; ph += S.ph; adsw += S.adsw; i += S.ima; vb += S.vbus; rail += S.rail;
    if (!isnan(S.adslo)) { lo += S.adslo; nlo++; }
    n++;
  }
  float m(double v) const { return n ? (float)(v / n) : NAN; }
  float sd() const { return n > 1 ? sqrtf(fmaxf(0.0f, (float)(pot2 / n - (pot / n) * (pot / n)))) : NAN; }
  float mlo() const { return nlo ? (float)(lo / nlo) : NAN; }
};
static bool holdAcc(float us, int n, Acc& a) {
  for (int i = 0; i < n; i++) { if (!tk(us)) return false; a.add(); }
  return true;
}

// Kullanicidan yanit (P satiri). Beklerken servo tutar (D satiri yok); jog: +N / -N us.
static bool ask(const char* id, const char* text, bool jog, char* out = nullptr, int n = 0) {
  Serial.printf("P,%s,%s\n", id, text);
  g_ansReady = false; g_jog = 0;
  bool m = g_mute;
  g_mute = true;
  float h = cur();
  bool on = g_pwm;
  while (!g_ansReady) {
    if (jog && g_jog != 0) { h += g_jog; g_jog = 0; if (h < C.us_lo) h = C.us_lo; if (h > C.us_hi) h = C.us_hi;
                             Serial.printf("#jog us=%.2f\n", quant(h)); }
    if (!tk(on ? h : NAN)) { g_mute = m; return false; }
  }
  g_mute = m;
  for (char* p = g_ans; *p; p++) if (*p == ',') *p = ';';
  ev("ans", "id=%s,val=%s", id, g_ans);
  if (out) { strncpy(out, g_ans, n - 1); out[n - 1] = 0; }
  return true;
}

static int parseList(const char* s, float* out, int nmax) {
  int n = 0;
  char b[128];
  strncpy(b, s, sizeof b - 1); b[sizeof b - 1] = 0;
  for (char* p = strtok(b, ",;"); p && n < nmax; p = strtok(nullptr, ",;")) out[n++] = atof(p);
  return n;
}

// ================================================================ programlar (tools/sysid/synth.py ile ayni)
static bool p_sweep() {
  float lo = P("lo_us", C.us_lo + 50), hi = P("hi_us", C.us_hi - 50), st = P("step_us", 10), dw = P("dwell_ms", 200);
  if (!argv_("rate")) { strcpy(g_ak[g_na], "rate"); strcpy(g_av[g_na++], "200"); }
  begin("sweep", 900);
  seg("move");
  if (!moveTo(lo) || !hold(lo, 1000)) return false;
  float u = lo;
  int k = 0;
  for (; u <= hi + 1e-3f; k++, u = lo + k * st) { seg("stair", "dir=1,us=%.1f", u); if (!hold(u, dw)) return false; }
  for (k--; k >= 0; k--) { u = lo + k * st; seg("stair", "dir=-1,us=%.1f", u); if (!hold(u, dw)) return false; }
  seg("move");
  return moveTo(C.us_c) && hold(C.us_c, 500);
}

static bool p_potcal() {
  const char* pts = PStr("points", "-90,-45,0,45,90");
  int n = (int)P("n", 256);
  if (!argv_("rate")) { strcpy(g_ak[g_na], "rate"); strcpy(g_av[g_na++], "200"); }
  begin("potcal", 3600);
  float d[12];
  int m = parseList(pts, d, 12);
  if (!g_pwm) setUs(cur());
  for (int i = 0; i < m; i++) {
    char id[8], txt[96];
    snprintf(id, sizeof id, "pc%d", i);
    snprintf(txt, sizeof txt, "Kolu %g derece cizgisine getirin (+N/-N us), sonra ok", d[i]);
    if (!ask(id, txt, true)) return false;
    seg("potcal", "deg=%g", d[i]);
    float u = cur();
    Acc a;
    if (!hold(u, 500) || !holdAcc(u, n, a)) return false;
    ev("potcal", "deg=%g,us=%.2f,mv=%.2f,sd=%.2f,ph_mv=%.2f,adsw_mv=%.2f,adslo_uv=%.0f,n=%d", d[i], u, a.m(a.pot),
       a.sd(), a.m(a.ph), a.m(a.adsw), a.mlo(), a.n);
  }
  seg("move");
  return moveTo(C.us_c) && hold(C.us_c, 300);
}

static bool p_potprobe() {
  const char* pos = PStr("positions_us", "900,1200,1500,1800,2100");
  begin("potprobe", 3600);
  float u[12];
  int m = parseList(pos, u, 12);
  for (int i = 0; i < m; i++) {
    char id[8];
    snprintf(id, sizeof id, "pp%d", i);
    seg("probe", "us=%.1f", u[i]);
    if (!moveTo(u[i], 30) || !hold(u[i], 500)) return false;
    if (!ask(id, "Pad gerilimleri (V, GND'ye gore)? ornek: ans 0.01 1.65 3.29", false)) return false;
  }
  seg("move");
  return moveTo(C.us_c);
}

static bool p_deadband() {
  float c = P("c_us", C.us_c), sp = P("span_us", 12), cyc = P("cycles", 3), dw = P("dwell_ms", 150);
  if (!argv_("rate")) { strcpy(g_ak[g_na], "rate"); strcpy(g_av[g_na++], "200"); }
  begin("deadband", 900);
  float tk_ = pca.tickUs();
  seg("move");
  if (!moveTo(c - 3 * sp) || !hold(c - 3 * sp, 500) || !ramp(c - 3 * sp, c - sp, 100.0f) || !hold(c - sp, 1000)) return false;
  int nst = (int)lroundf(2 * sp / tk_);
  float u = c - sp;
  for (int k = 0; k < (int)cyc; k++)
    for (int d = 1; d >= -1; d -= 2)
      for (int s = 0; s < nst; s++) {
        u += d * tk_;
        seg("stair", "dir=%d,us=%.2f", d, quant(u));
        if (!hold(u, dw)) return false;
      }
  seg("move");
  return moveTo(c) && hold(c, 300);
}

static bool p_step() {
  float c = P("c_us", C.us_c), hm = P("hold_ms", 500), reps = P("reps", 2);
  const char* amps = PStr("amps_deg", "1,3,6,20,40");
  begin("step", 900);
  float a[12];
  int m = parseList(amps, a, 12);
  seg("hold");
  if (!moveTo(c) || !hold(c, 500)) return false;
  for (int i = 0; i < m; i++)
    for (int r = 0; r < (int)reps; r++)
      for (int s = 1; s >= -1; s -= 2) {
        float u = c + s * 0.5f * a[i] / C.dpu;
        seg("step", "amp_deg=%g,us=%.2f", a[i], quant(u));
        if (!hold(u, hm)) return false;
      }
  seg("hold");
  return moveTo(c) && hold(c, 300);
}

static bool p_chirp() {
  float c = P("c_us", C.us_c), amp = P("amp_deg", 1.5f), f0 = P("f0", 0.5f);
  float f1 = P("f1", pca.freq() > 100 ? 25.0f : 8.0f), T = P("dur_s", 20);
  begin("chirp", 300);
  float A = amp / C.dpu, k = logf(f1 / f0);
  seg("hold");
  if (!moveTo(c) || !hold(c, 1000)) return false;
  seg("chirp", "f0=%g,f1=%g,dur_s=%g,amp_deg=%g", f0, f1, T, amp);
  int n = (int)lroundf(T * g_rate);
  for (int i = 1; i <= n; i++) {
    float t = (float)i / g_rate;
    float ph = 2.0f * PI * f0 * T / k * (expf(t * k / T) - 1.0f);
    if (!tk(c + A * sinf(ph))) return false;
  }
  seg("hold");
  return hold(c, 1000);
}

static bool p_speed() {
  float c = P("c_us", C.us_c), amp = P("amp_deg", 100), reps = P("reps", 3), hm = P("hold_ms", 1000);
  begin("speed", 300);
  float h = 0.5f * amp / C.dpu;
  seg("move");
  if (!moveTo(c - h) || !hold(c - h, hm)) return false;
  for (int r = 0; r < (int)reps; r++)
    for (int s = 1; s >= -1; s -= 2) {
      seg("step", "amp_deg=%g,dir=%d,us=%.2f", amp, s, quant(c + s * h));
      if (!hold(c + s * h, hm)) return false;
    }
  seg("move");
  return moveTo(c) && hold(c, 300);
}

static bool p_ramp() {
  float c = P("c_us", C.us_c), amp = P("amp_deg", 40), reps = P("reps", 1), hm = P("hold_ms", 800);
  const char* sp = PStr("speeds_dps", "5,10,20,40,80,160");
  begin("ramp", 900);
  float v[12];
  int m = parseList(sp, v, 12);
  float h = 0.5f * amp / C.dpu;
  seg("move");
  if (!moveTo(c - h) || !hold(c - h, hm)) return false;
  for (int i = 0; i < m; i++)
    for (int r = 0; r < (int)reps; r++)
      for (int s = 1; s >= -1; s -= 2) {
        seg("ramp", "dir=%d,speed_dps=%g", s, v[i]);
        if (!ramp(c - s * h, c + s * h, v[i] / C.dpu)) return false;
        seg("hold");
        if (!hold(c + s * h, hm)) return false;
      }
  seg("move");
  return moveTo(c) && hold(c, 300);
}

static bool p_stall() {
  float dir = P("dir", 1), hm = P("hold_ms", 2500), cool = P("cool_ms", 5000), meter = P("meter", 1);
  const char* ds = PStr("deltas_deg", "0.5,1,2,3,4,6,8,12,20");
  if (!argv_("rate")) { strcpy(g_ak[g_na], "rate"); strcpy(g_av[g_na++], "200"); }
  begin("stall", 1800);
  if (hm > 3000) hm = 3000;
  float d[14];
  int m = parseList(ds, d, 13);
  float dmax = 0;
  for (int i = 0; i < m; i++) dmax = fmaxf(dmax, d[i]);
  if (meter > 0) d[m++] = dmax;
  seg("move");
  if (!g_pwm) setUs(cur());
  if (!ask("touch", "Kol ucunu teraziye degdirin (jog +N/-N us), servo tutarken teraziyi darali sifirlayin; hazirsa ok", true))
    return false;
  float ut = cur();
  Acc a;
  if (!holdAcc(ut, max(1, g_rate / 4), a) || !hold(ut, 250)) return false;
  ev("ref", "name=touch,us=%.2f,mv=%.2f", ut, a.m(a.pot));
  for (int k = 0; k < m; k++) {
    bool mt = meter > 0 && k == m - 1;
    float u = ut + dir * d[k] / C.dpu;
    if (mt && !ask("meter_ready", "Multimetreyi/kaynak gostergesini izleyin; ok deyince 3 s stall", false)) return false;
    char ik[8];
    if (mt) strcpy(ik, "meter"); else snprintf(ik, sizeof ik, "%d", k);
    seg("stall", "i=%s,delta_deg=%g,us=%.2f", ik, d[k], quant(u));
    g_ilim = C.ilim_peak;
    bool ok = hold(u, mt ? 3000 : hm);
    g_ilim = C.ilim;
    if (!ok) return false;
    seg("rest");
    float ur = ut - dir * 1.0f / C.dpu;
    if (!hold(ur, 200)) return false;
    char id[12];
    if (mt) strcpy(id, "meter_a"); else snprintf(id, sizeof id, "g_%d", k);
    if (!ask(id, mt ? "Olcu aletinde okunan stall akimi (A)?" : "Terazide okunan en yuksek deger (g)?", false)) return false;
    if (!hold(ur, cool)) return false;
  }
  seg("move");
  return moveTo(C.us_c);
}

static float mglNow() { float L = C.lever_mm / 1000.0f; return G * (C.mass_g / 1000.0f * L + C.lever_g / 1000.0f * L / 2.0f); }

static bool askHoriz() {
  if (!g_pwm) setUs(cur());
  if (!ask("horiz", "Kolu yatay konuma getirin (jog +N/-N us; su terazisi/aciolcer), ok", true)) return false;
  return true;
}

static bool p_curpts() {
  const char* angs = PStr("angles_deg", "-60,-30,0,30,60,90");
  float hm = P("hold_ms", 2000), amp = P("amp_deg", 30);
  const char* sp = PStr("speeds_dps", "30,90");
  if (!argv_("rate")) { strcpy(g_ak[g_na], "rate"); strcpy(g_av[g_na++], "250"); }
  begin("curpts", 1800);
  if (C.mass_g <= 0) { Serial.println("W,mass_g ayarlanmamis (set mass_g=...)"); return false; }
  float a[12], v[8];
  int na = parseList(angs, a, 12), nv = parseList(sp, v, 8);
  seg("move");
  if (!askHoriz()) return false;
  float uh = cur();
  Acc ac;
  if (!hold(uh, 200) || !holdAcc(uh, max(1, g_rate * 2 / 5), ac)) return false;
  ev("ref", "name=horiz,us=%.2f,mv=%.2f", uh, ac.m(ac.pot));
  for (int i = 0; i < na; i++) {
    float u = uh + a[i] / C.dpu;
    for (int app = 1; app >= -1; app -= 2) {
      seg("move");
      if (!moveTo(u - app * 3.0f / C.dpu, 60) || !hold(cur(), 200)) return false;
      seg("static", "ang_deg=%g,app=%d", a[i], app);
      if (!moveTo(u, 20) || !hold(u, hm)) return false;
    }
  }
  float A = amp / C.dpu;
  for (int i = 0; i < nv; i++) {
    seg("move");
    if (!moveTo(uh - A, 60) || !hold(uh - A, 300)) return false;
    for (int r = 0; r < 2; r++)
      for (int s = 1; s >= -1; s -= 2) {
        seg("ramp_ld", "dir=%d,speed_dps=%g", s, v[i]);
        if (!ramp(uh - s * A, uh + s * A, v[i] / C.dpu)) return false;
      }
  }
  seg("move");
  return moveTo(uh, 60) && hold(uh, 300);
}

static bool p_track() {
  float fq = P("freq", (float)C.freq), amp = P("amp_deg", 10), reps = P("reps", 3), sm = P("step_ms", 800);
  float fs = P("f_sin", 1.0f), ss = P("sin_s", 5), hs = P("hold_s", 2), force = P("force", 0);
  if (!argv_("rate")) { strcpy(g_ak[g_na], "rate"); strcpy(g_av[g_na++], "250"); }
  begin("track", 300);
  if (isAnalog() && fq > 60 && force <= 0) { Serial.println("W,analog servo 50 Hz disinda surulmez (force=1)"); return false; }
  int pre0 = pca.prescale;
  int pre = (int)lroundf(pca.osc / (4096.0f * fq)) - 1;
  if (fq > 200 && pca.osc / (4096.0f * (pre + 1)) > 330.0f) pre++;
  pca.setPrescale(pre);
  if (g_pwm) setUs(cur());
  ev("pca", "freq=%g,prescale=%d,pwm_hz=%.2f,tick_us=%.4f", fq, pre, pca.freq(), pca.tickUs());
  float c = C.us_c, A = amp / C.dpu;
  bool ok = true;
  seg("hold");
  ok = moveTo(c) && hold(c, 1000);
  for (int r = 0; ok && r < (int)reps; r++)
    for (int s = 1; ok && s >= -1; s -= 2) {
      seg("step", "amp_deg=%g,us=%.2f", 2 * amp, quant(c + s * A));
      ok = hold(c + s * A, sm);
    }
  if (ok) {
    seg("sin", "f=%g,amp_deg=%g", fs, amp);
    int n = (int)lroundf(ss * g_rate);
    for (int i = 1; ok && i <= n; i++) ok = tk(c + A * sinf(2.0f * PI * fs * i / g_rate));
  }
  if (ok) { seg("hold"); ok = hold(c, hs * 1000); }
  if (ok) ok = ask("obs", "Servo duzgun takip etti mi (titreme/ses/isinma yok)? (e/h)", false);
  pca.setPrescale(pre0);
  if (g_pwm) setUs(cur());
  return ok;
}

static bool p_overshoot() {
  float amp = P("amp_deg", 90), reps = P("reps", 4), hm = P("hold_ms", 800);
  int n = (int)P("n", 8000), dtu = (int)P("dt_us", 50), dec = (int)P("dec", 4);
  if (!argv_("rate")) { strcpy(g_ak[g_na], "rate"); strcpy(g_av[g_na++], "200"); }
  begin("overshoot", 600);
  if (n > BURST_MAX) n = BURST_MAX;
  if (!g_buf || C.pin_rail < 0) { Serial.println("W,burst tamponu yok ya da pin_rail < 0"); return false; }
  float c = C.us_c, h = 0.5f * amp / C.dpu, vmax = 0;
  seg("move");
  if (!moveTo(c - h) || !hold(c - h, 500)) return false;
  for (int k = 0; k < (int)reps; k++) {
    int s = (k % 2 == 0) ? 1 : -1;
    seg("burst", "i=%d", k);
    Acc a;
    if (!holdAcc(c - s * h, max(1, g_rate / 10), a)) return false;
    float v0 = a.m(a.rail), vb0 = a.m(a.vb);
    int64_t t0 = esp_timer_get_time();
    Serial.printf("E,%lld,burst,i=%d,n=%d,dt_us=%d,v0_mv=%.1f,vbus0_mv=%.1f\n", (long long)t0, k, n, dtu, v0, vb0);
    setUs(c + s * h);
    int kmax = 0;
    for (int j = 0; j < n; j++) {
      int64_t tt = t0 + (int64_t)j * dtu;
      while (esp_timer_get_time() < tt) {}
      g_bt[j] = (uint32_t)(esp_timer_get_time() - t0);
      g_buf[j] = analogReadMilliVolts(C.pin_rail);
      if (g_buf[j] > g_buf[kmax]) kmax = j;
    }
    for (int j = 0; j < n; j++)
      if (j % dec == 0 || abs(j - kmax) <= 50) Serial.printf("B,%lld,%u\n", (long long)(t0 + g_bt[j]), (unsigned)g_buf[j]);
    float vm = isnan(vb0) || v0 <= 0 ? g_buf[kmax] / C.rail_div / 1000.0f : g_buf[kmax] * vb0 / v0 / 1000.0f;
    vmax = fmaxf(vmax, vm);
    Serial.printf("R,ovs_%d_v,%.3f\n", k, vm);
    g_next = esp_timer_get_time() + g_per;
    pollSerial();
    if (g_abort || !hold(c + s * h, hm)) return false;
  }
  Serial.printf("R,ovs_max_v,%.3f\n", vmax);
  seg("move");
  return moveTo(c);
}

static double bessel0(double x) {
  double s = 1, t = 1;
  for (int k = 1; k < 40; k++) { t *= -(x * x / 4.0) / ((double)k * k); s += t; if (fabs(t) < 1e-14) break; }
  return s;
}

static bool p_heat() {
  const char* lv = PStr("levels_nm", "0.9,1.05,1.15");
  float ls = P("level_s", 600), fh = P("f_hz", 1.1f), amp = P("amp_deg", 15), coolc = P("cool_c", 3);
  float coolm = P("cool_max_s", 1200), rest = P("rest_deg", -80);
  if (!argv_("rate")) { strcpy(g_ak[g_na], "rate"); strcpy(g_av[g_na++], "50"); }
  begin("heat", 4 * 3600);
  if (C.mass_g <= 0) { Serial.println("W,mass_g ayarlanmamis (set mass_g=...)"); return false; }
  float L[6];
  int nl = parseList(lv, L, 6);
  float mgl = mglNow(), A = amp * PI / 180.0f;
  seg("move");
  if (!askHoriz()) return false;
  float uh = cur();
  Acc ac;
  if (!hold(uh, 100) || !holdAcc(uh, max(1, g_rate / 2), ac)) return false;
  ev("ref", "name=horiz,us=%.2f,mv=%.2f", uh, ac.m(ac.pot));
  float amb = isnan(S.amb) ? NAN : S.amb;
  if (isnan(amb)) {
    char b[16];
    if (!ask("amb", "Ortam sicakligi (C)? (ortam NTC'si bagli degil)", false, b, sizeof b)) return false;
    amb = atof(b);
  }
  g_tsoft = true;
  bool ok = true;
  for (int li = 0; ok && li < nl; li++) {
    double r = (2.0 * pow(L[li] / mgl, 2) - 1.0) / bessel0(2.0 * A);
    if (r > 1 || r < -1) { ev("skip", "lvl=%d,tau_nm=%g,neden=kutle_yetersiz", li, L[li]); continue; }
    float qc = -0.5f * acosf((float)r) * 180.0f / PI;
    seg("heat", "lvl=%d,tau_nm=%g,qc_deg=%.3f,amp_deg=%g,f_hz=%g,mgl=%.4f", li, L[li], qc, amp, fh, mgl);
    ok = moveTo(uh + qc / C.dpu, 30);
    int n = (int)lroundf(ls * g_rate);
    for (int i = 0; ok && i < n; i++) {
      ok = tk(uh + (qc + amp * sinf(2.0f * PI * fh * i / g_rate)) / C.dpu);
      if (ok && !isnan(S.ntc) && S.ntc > C.tlim) { ev("level_end", "lvl=%d,neden=tlim,ntc_c=%.2f", li, S.ntc); break; }
    }
    if (!ok) break;
    seg("cool", "lvl=%d", li);
    float ur = uh + rest / C.dpu;
    ok = moveTo(ur, 30);
    if (!ok) break;
    ev("pwm", "on=0");
    float tc = 0;
    while (ok && tc < coolm) {
      ok = tk(NAN);
      tc += 1.0f / g_rate;
      float a2 = isnan(S.amb) ? amb : S.amb;
      if (!isnan(S.ntc) && S.ntc < a2 + coolc) break;
    }
    ev("pwm", "on=1");
    if (ok) ok = hold(ur, 200);
  }
  g_tsoft = false;
  if (!ok) return false;
  seg("move");
  return moveTo(uh, 30) && hold(uh, 300);
}

static bool p_noise() {
  float d = P("dur_s", 10), wf = P("wifi", 0);
  begin("noise", 120);
  if (wf > 0) { WiFi.mode(WIFI_AP); WiFi.softAP("duck-bench", "duckbench1"); }
  seg("hold", "wifi=%d", (int)wf);
  bool ok = hold(cur(), d * 1000);
  if (wf > 0) WiFi.mode(WIFI_OFF);
  return ok;
}

static bool p_nopwm() {
  begin("nopwm", 600);
  seg("hold");
  if (!hold(cur(), 1000)) return false;
  ev("pwm", "on=0");
  pwmOff();
  seg("off");
  if (!ask("nopwm", "PWM kesildi. Kolu elle cevirin: servo tutuyor mu? (t: tutuyor / b: bosta)", false)) return false;
  ev("pwm", "on=1");
  seg("hold");
  return hold(cur(), 500);
}

static bool p_hold() {
  float u = P("c_us", cur()), d = P("dur_s", 5);
  begin("hold", 3600);
  seg("hold");
  return moveTo(u, 30) && hold(u, d * 1000);
}

typedef bool (*ProgFn)();
struct Prog { const char* name; ProgFn fn; };
static const Prog PROGS[] = {
  {"sweep", p_sweep}, {"potcal", p_potcal}, {"potprobe", p_potprobe}, {"deadband", p_deadband}, {"step", p_step},
  {"chirp", p_chirp}, {"speed", p_speed}, {"ramp", p_ramp}, {"stall", p_stall}, {"curpts", p_curpts},
  {"track", p_track}, {"overshoot", p_overshoot}, {"heat", p_heat}, {"noise", p_noise}, {"nopwm", p_nopwm},
  {"hold", p_hold},
};

static void runProg(char* args) {
  char* name = strtok(args, " ");
  if (!name) { Serial.println("ERR program adi"); return; }
  const Prog* pg = nullptr;
  for (auto& p : PROGS) if (!strcmp(p.name, name)) pg = &p;
  if (!pg) { Serial.printf("ERR bilinmeyen program %s\n", name); return; }
  if (!pca.ok) { Serial.println("ERR PCA9685 yok (0x40); kablolari kontrol edin"); return; }
  g_na = 0;
  for (char* kv = strtok(nullptr, " "); kv && g_na < 20; kv = strtok(nullptr, " ")) {
    char* e = strchr(kv, '=');
    if (!e) continue;
    *e = 0;
    strncpy(g_ak[g_na], kv, 15); g_ak[g_na][15] = 0;
    strncpy(g_av[g_na], e + 1, 39); g_av[g_na][39] = 0;
    g_na++;
  }
  g_used = "";
  g_abort = false; g_why[0] = 0; g_running = true; g_mute = false; g_tsoft = false;
  g_lastHb = esp_timer_get_time();
  bool ok = pg->fn();
  g_running = false; g_tsoft = false; g_mute = false;
  if (!ok && !g_abort) { killOutputs(); strcpy(g_why, "program"); }
  Serial.printf("END,%s,%s%s\n", pg->name, ok && !g_abort ? "ok" : "abort:", ok && !g_abort ? "" : g_why);
  g_abort = false;
}

// ---------------------------------------------------------------- komutlar
static void oscAuto() {
  pca.setCounts(15, 0, 2048);
  digitalWrite(PIN_OE, LOW);
  delay(50);
  oscN = 0; oscOn = true;
  delay(1000);
  oscOn = false;
  uint32_t n = oscN;
  if (!g_pwm) digitalWrite(PIN_OE, HIGH);
  pca.chOff(15);
  if (n < 10) { Serial.printf("W,osc: GPIO%d'de darbe yok (PCA ch15 -> 1k -> GPIO%d baglayin) n=%u\n", PIN_CAP_OSC, PIN_CAP_OSC, (unsigned)n); return; }
  float f = (n - 1) * 1.0e6f / (float)(oscLast - oscFirst);
  pca.osc = f * 4096.0f * (pca.prescale + 1);
  C.osc = pca.osc;
  Serial.printf("R,osc_pwm_hz,%.3f\nR,osc_hz,%.0f\n", f, pca.osc);
}

static void setFreq(float hz, bool force) {
  if (isAnalog() && hz > 60 && !force) { Serial.println("ERR analog servo (mg/sg) 50 Hz; zorlamak icin: freq <hz> force"); return; }
  int pre = (int)lroundf(pca.osc / (4096.0f * hz)) - 1;
  if (hz > 200 && pca.osc / (4096.0f * (pre + 1)) > 330.0f) pre++;   // DS en fazla 330 Hz
  C.freq = (int)hz;
  C.prescale = pre;
  pca.setPrescale(pre);
  if (g_pwm) setUs(cur());
  Serial.printf("OK freq prescale=%d pwm_hz=%.2f tick_us=%.4f\n", pre, pca.freq(), pca.tickUs());
}

static void scan() {
  Serial.print("R,i2c,");
  for (uint8_t a = 1; a < 127; a++) if (i2cPresent(a)) Serial.printf("0x%02X;", a);
  Serial.println();
  for (int p = 1; p <= 10; p++) Serial.printf("R,adc_gpio%d_mv,%.1f\n", p, adcMv(p, 16));
  float uv = NAN, mv = NAN;
  if (ina.ok) { ina.shuntUv(uv); ina.busMv(mv); }
  Serial.printf("R,ina_ma,%.1f\nR,ina_vbus_mv,%.0f\n", uv / C.shunt_mohm, mv);
  for (int i = 0; i < 30; i++) { ads.poll(); delay(2); }
  Serial.printf("R,ads_ain0_mv,%.2f\nR,ads_ain1_mv,%.2f\nR,ads_ain23_uv,%.0f\n", ads.val[0], ads.val[1], ads.val[2] * 1000.0f);
  Serial.printf("R,ntc_kasa_c,%.2f\nR,ntc_ortam_c,%.2f\nR,cap_us,%.1f\n", ntcC(ads.val[0]), ntcC(adcMv(C.pin_amb, 8)), capW);
  Serial.printf("R,pwm_hz,%.2f\nR,boot,%d\n", pca.freq(), digitalRead(PIN_BOOT));
}

static void help() {
  Serial.println("# komutlar: help | cfg | scan | cls ds|es|mg|sg | freq 50|300 [force] | prescale N | osc auto|<olculen_Hz>");
  Serial.println("#   set k=v ... (cfg anahtarlari) | us <darbe> | off | on | save | load | reset | wifi 0|1 | mon 0|1 | hb");
  Serial.println("#   run <program> k=v ... : sweep potcal potprobe deadband step chirp speed ramp stall curpts track");
  Serial.println("#                           overshoot heat noise nopwm hold");
  Serial.println("# program sirasinda: ! (E-stop) | stop | ans <yanit> | ok | +N/-N (jog, us)");
}

static void command(char* s) {
  static char empty[1] = {0};
  char* sp = strchr(s, ' ');
  char* rest = sp ? sp + 1 : empty;
  if (sp) *sp = 0;
  if (!strcmp(s, "help")) help();
  else if (!strcmp(s, "cfg")) printCfg();
  else if (!strcmp(s, "scan")) scan();
  else if (!strcmp(s, "run")) runProg(rest);
  else if (!strcmp(s, "cls")) { setClass(rest); pca.setPrescale(C.prescale); setFreq(C.freq, false); printCfg(); }
  else if (!strcmp(s, "freq")) setFreq(atof(rest), strstr(rest, "force") != nullptr);
  else if (!strcmp(s, "prescale")) { C.prescale = atoi(rest); pca.setPrescale(C.prescale); if (g_pwm) setUs(cur());
                                     Serial.printf("OK prescale pwm_hz=%.2f\n", pca.freq()); }
  else if (!strcmp(s, "osc")) {
    if (!strcmp(rest, "auto")) oscAuto();
    else { float f = atof(rest); if (f > 20 && f < 2000) { pca.osc = f * 4096.0f * (pca.prescale + 1); C.osc = pca.osc;
           Serial.printf("OK osc=%.0f (olculen %.3f Hz)\n", pca.osc, f); } else Serial.println("ERR osc <Hz> (olculen PWM frekansi)"); }
    if (g_pwm) setUs(cur());
  }
  else if (!strcmp(s, "set")) {
    for (char* kv = strtok(rest, " "); kv; kv = strtok(nullptr, " ")) {
      char* e = strchr(kv, '=');
      if (!e) continue;
      *e = 0;
      bool found = false;
      for (auto& k : KEYS) if (!strcmp(k.k, kv)) {
        found = true;
        if (k.t == 'f') *(float*)k.p = (!strcmp(e + 1, "nan") ? NAN : atof(e + 1)); else *(int*)k.p = atoi(e + 1);
      }
      Serial.printf(found ? "OK %s\n" : "ERR bilinmeyen anahtar %s\n", kv);
    }
    pca.osc = C.osc;
  }
  else if (!strcmp(s, "us")) {
    if (g_abort) { Serial.println("ERR once 'on'"); return; }
    float u = atof(rest);
    if (u < C.us_lo || u > C.us_hi) Serial.printf("W,sinir [%.0f, %.0f] uygulandi\n", C.us_lo, C.us_hi);
    setUs(u);
    Serial.printf("OK us=%.2f\n", g_cmd);
  }
  else if (!strcmp(s, "off")) { killOutputs(); Serial.println("OK off"); }
  else if (!strcmp(s, "on")) { g_abort = false; Serial.println("OK on (PWM 'us' ya da program ile acilir)"); }
  else if (!strcmp(s, "save")) {
    Cfg t = C;
    t.hbto = 0;                          // kalp atisi siniri oturuma ozel (logger acar/kapatir), kaydedilmez
    prefs.putBytes("cfg", &t, sizeof t);
    Serial.println("OK save");
  }
  else if (!strcmp(s, "load")) { Cfg t; if (prefs.getBytes("cfg", &t, sizeof t) == sizeof t && t.magic == MAGIC) { C = t; pca.osc = C.osc;
                                  pca.setPrescale(C.prescale); Serial.println("OK load"); } else Serial.println("ERR kayit yok"); }
  else if (!strcmp(s, "reset")) { cfgDefaults(); pca.osc = C.osc; pca.setPrescale(C.prescale); Serial.println("OK reset"); }
  else if (!strcmp(s, "wifi")) {
    if (atoi(rest)) { WiFi.mode(WIFI_AP); WiFi.softAP("duck-bench", "duckbench1"); } else { WiFi.mode(WIFI_OFF); }
    Serial.println("OK wifi");
  }
  else if (!strcmp(s, "mon")) { g_mon = atoi(rest) != 0; Serial.println("OK mon"); }
  else Serial.printf("ERR bilinmeyen komut %s (help)\n", s);
}

// ---------------------------------------------------------------- kurulum
void setup() {
  pinMode(PIN_OE, OUTPUT);
  digitalWrite(PIN_OE, HIGH);            // acilista PWM cikislari kapali
  pinMode(PIN_BOOT, INPUT_PULLUP);
  Serial.setTxBufferSize(16384);
  Serial.setRxBufferSize(1024);
  Serial.begin(BENCH_BAUD);
  delay(300);
  cfgDefaults();
  prefs.begin("bench", false);
  Cfg t;
  if (prefs.getBytes("cfg", &t, sizeof t) == sizeof t && t.magic == MAGIC) C = t;
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.setTimeOut(5);
  analogReadResolution(12);
  for (int p = 1; p <= 10; p++) analogSetPinAttenuation(p, ADC_11db);   // ADC1, 12 dB (0..~2.9 V)
  pca.osc = C.osc;
  pca.prescale = C.prescale;
  if (!pca.begin(0x40)) Serial.println("W,PCA9685 0x40'ta yok");
  if (!ina.find()) Serial.println("W,INA226 yok (akim/VBUS kolonlari nan)");
  else ina.config(1, 2);
  if (!ads.find()) Serial.println("W,ADS1115 yok (ntc/adsw/adslo kolonlari nan)");
  pinMode(PIN_CAP_DUT, INPUT_PULLDOWN);
  pinMode(PIN_CAP_OSC, INPUT_PULLDOWN);
  attachInterrupt(PIN_CAP_DUT, isrCapDut, CHANGE);
  attachInterrupt(PIN_CAP_OSC, isrOsc, RISING);
  g_buf = (uint16_t*)ps_malloc(BURST_MAX * sizeof(uint16_t));
  g_bt = (uint32_t*)ps_malloc(BURST_MAX * sizeof(uint32_t));
  if (!g_buf) g_buf = (uint16_t*)malloc(BURST_MAX * sizeof(uint16_t));
  if (!g_bt) g_bt = (uint32_t*)malloc(BURST_MAX * sizeof(uint32_t));
  S.amb = NAN;
  Serial.printf("#OpenDuck-TR tezgah %s hazir. 'help' yazin. PWM kapali; once 'scan' ve 'osc auto'.\n", FW_VER);
  printCfg();
}

void loop() {
  pollSerial();
  if (g_cmdReady) { g_cmdReady = false; command(g_cmdq); }
  if (digitalRead(PIN_BOOT) == LOW && g_pwm) { killOutputs(); Serial.println("W,DUR boot_dugmesi"); delay(300); }
  ads.poll();
  static uint32_t lm = 0;
  if (g_mon && millis() - lm > 200) {
    lm = millis();
    g_tick = 0;
    sample(esp_timer_get_time());
    Serial.printf("M,cmd=%.1f,pot=%.1f,ph=%.1f,rail=%.0f,i=%.1f,vbus=%.0f,ntc=%.2f,amb=%.2f,cap=%.1f\n",
                  g_pwm ? g_cmd : 0.0f, S.pot, S.ph, S.rail, S.ima, S.vbus, S.ntc, S.amb, S.cap);
  }
}
