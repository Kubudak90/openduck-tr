// OpenDuck-TR: DF2301Q (DFRobot SEN0539) ses modulu UART cercevesi (ek K3 Baglanti, K8). Kutuphane yok.
// Arduino'ya bagli degil: masaustunde de derlenir (tools/bench/tests/test_voice.py cozucuyu boyle sinar).
//
// Cerceve, DFRobot_DF2301Q.cpp v1.0.1 sendPacket/recvPacket ile ayni (UART 9600 8N1):
//   F4 F5 | uzunluk (2 bayt, LE) | tip | komut | sira | veri[uzunluk] | toplam (2 bayt, LE) | FB
//   toplam = tip + komut + sira + veri baytlari (16 bit; kutuphanede yapinin 4. baytindan baslar).
// Tipler: A0 yukari (tanima: kutuphane her A0'da veri[0]'i kimlik sayar), A1 asagi (host -> modul), A2 ACK,
//         A3 NOTIFY. NOTIFY (komut 9A) veri[0]: B0 acilis, B1 uyanma basladi, B2 uyanma bitti, B3/B4 calma.
// Belgelenmeyenler (modul NOTIFY/ACK yolluyor mu, A0'da baska veri var mi): cozucu savunmaci; bilinmeyen ve
// bozuk cerceveler ham baytlariyla dondurulur, cagiran kaydeder (G-SES-a bunlari olcer).
#pragma once
#include <stdint.h>
#include <string.h>

namespace df {
static const uint8_t H0 = 0xF4, H1 = 0xF5, TAIL = 0xFB;
static const uint8_t T_UP = 0xA0, T_DOWN = 0xA1, T_ACK = 0xA2, T_NOTIFY = 0xA3;
static const uint8_t C_ASR = 0x91, C_PLAY = 0x92, C_VER = 0x94, C_RESET = 0x95, C_SET = 0x96, C_NOTIFY = 0x9A;
static const uint8_t S_VOLUME = 0x80, S_ENTERWAKE = 0x81, S_MUTE = 0x83, S_WAKETIME = 0x84;
static const uint8_t N_POWERON = 0xB0, N_WAKE_ENTER = 0xB1, N_WAKE_EXIT = 0xB2, N_PLAY_START = 0xB3, N_PLAY_END = 0xB4;
static const int MAXD = 32;              // kutuphane 8'den uzun veriyi atar; biz 32'ye kadar alip kaydederiz
static const int MAXRAW = MAXD + 10;

// Cerceve kur; out en az n + 10 bayt. Doner: bayt sayisi.
static inline int build(uint8_t type, uint8_t cmd, uint8_t seq, const uint8_t* d, int n, uint8_t* out) {
  int k = 0;
  uint16_t s = (uint16_t)(type + cmd + seq);
  out[k++] = H0; out[k++] = H1; out[k++] = (uint8_t)(n & 0xFF); out[k++] = (uint8_t)((n >> 8) & 0xFF);
  out[k++] = type; out[k++] = cmd; out[k++] = seq;
  for (int i = 0; i < n; i++) { out[k++] = d[i]; s = (uint16_t)(s + d[i]); }
  out[k++] = (uint8_t)(s & 0xFF); out[k++] = (uint8_t)(s >> 8); out[k++] = TAIL;
  return k;
}
// settingCMD(tur, deger): veri = [tur, deger (4 bayt LE)]
static inline int buildSet(uint8_t seq, uint8_t what, uint32_t v, uint8_t* out) {
  const uint8_t d[5] = {what, (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
  return build(T_DOWN, C_SET, seq, d, 5, out);
}
// playByCMDID(id): veri = [80 (calma basla), 92 (kimlikle), id (4 bayt LE)]
static inline int buildPlay(uint8_t seq, uint32_t id, uint8_t* out) {
  const uint8_t d[6] = {0x80, 0x92, (uint8_t)id, (uint8_t)(id >> 8), (uint8_t)(id >> 16), (uint8_t)(id >> 24)};
  return build(T_DOWN, C_PLAY, seq, d, 6, out);
}
static inline int buildReset(uint8_t seq, uint8_t* out) {
  const uint8_t d[5] = {'r', 'e', 's', 'e', 't'};
  return build(T_DOWN, C_RESET, seq, d, 5, out);
}

// Bayt bayt cozucu. feed(b): 0 devam, 1 gecerli cerceve (type/cmd/seq/d/len), -1 bozuk cerceve (raw[0..nraw),
// err: 'l' uzunluk, 'k' toplam, 't' kuyruk), 2 cerceve disi bayt (cop). Bozuk cerceveden sonra F4 aranir.
struct Parser {
  enum { S_H0, S_H1, S_L0, S_L1, S_T, S_C, S_Q, S_D, S_K0, S_K1, S_E };
  int st = S_H0;
  uint8_t type = 0, cmd = 0, seq = 0, d[MAXD];
  uint16_t len = 0, nd = 0, chk = 0;
  uint8_t raw[MAXRAW];
  int nraw = 0;
  char err = 0;
  void push(uint8_t b) { if (nraw < MAXRAW) raw[nraw++] = b; }
  int feed(uint8_t b) {
    switch (st) {
      case S_H0:
        if (b != H0) return 2;
        nraw = 0; push(b); st = S_H1; return 0;
      case S_H1:
        if (b == H1) { push(b); st = S_L0; return 0; }
        if (b == H0) { nraw = 0; push(b); return 2; }      // onceki F4 copmus; bu yeni baslangic olabilir
        st = S_H0; return 2;
      case S_L0: push(b); len = b; st = S_L1; return 0;
      case S_L1:
        push(b); len = (uint16_t)(len | (b << 8));
        if (len > MAXD) { err = 'l'; st = S_H0; return -1; }
        nd = 0; st = S_T; return 0;
      case S_T: push(b); type = b; st = S_C; return 0;
      case S_C: push(b); cmd = b; st = S_Q; return 0;
      case S_Q: push(b); seq = b; st = len ? S_D : S_K0; return 0;
      case S_D: push(b); d[nd++] = b; if (nd >= len) st = S_K0; return 0;
      case S_K0: push(b); chk = b; st = S_K1; return 0;
      case S_K1: {
        push(b); chk = (uint16_t)(chk | (b << 8));
        uint16_t s = (uint16_t)(type + cmd + seq);
        for (int i = 0; i < len; i++) s = (uint16_t)(s + d[i]);
        if (s != chk) { err = 'k'; st = S_H0; return -1; }
        st = S_E; return 0;
      }
      case S_E:
        push(b); st = S_H0;
        if (b != TAIL) { err = 't'; return -1; }
        err = 0; return 1;
    }
    st = S_H0;
    return 2;
  }
};

// Ham baytlari onaltilik yaz (bosluksuz); out en az 2n + 1.
static inline void hex(const uint8_t* b, int n, char* out) {
  static const char* H = "0123456789ABCDEF";
  for (int i = 0; i < n; i++) { out[2 * i] = H[b[i] >> 4]; out[2 * i + 1] = H[b[i] & 15]; }
  out[2 * n] = 0;
}
}  // namespace df
