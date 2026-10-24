# -*- coding: utf-8 -*-
"""
Sesli komut denetleyicisi (OpenDuck-TR spec eki K5, docs/workflow/openduck-tr/spec-ek-komut.md): DF2301Q kimlik (ID)
olaylarini main / getup politikalarinin girdilerine ceviren TEK tanim. numpy, baştan sona float32; eval (rl/eval_tasks.py
A10), play ve firmware (birebir kopya, adim adim altin izlerle: rl/tests/data/komut_*.csv.gz) cagirir; egitimde CALISMAZ
(egitim jest kanali ve komut atomlari task_math'te, K6). Gecisler veri tablosudur (TABLE: olay satiri x durum -> eylem).

  tab = load_id_table()                          # configs/komut_kimlik.json (beyaz liste, coktan bire; K4)
  sup = Supervisor(tab, home_q, state="A")       # ya da "M" (oturmus baslangic)
  out = sup.step(Inputs(t, ids, gravity_obs, gyro_obs, qd_obs, q_obs, alpha, h_ref))   # her kontrol adiminda

Girdiler (yalniz firmware'in olcebildigi; sensor modeli rl/sensors.py): gravity_obs (3, tamamlayici filtre), gyro_obs
(3), qd_obs / q_obs (14, pot; bacak qd ilk 10), posture_step'in alpha ve h_ref'i, zaman ve bu adimda gelen kimlikler
(gelis sirasiyla). q_obs yalniz getup -> main hazirlik olcutu (play.Supervisor'dan gelen poz RMS) icin.
Ciktilar: durum, politika ("main" | "getup" | "none"), main komutu (5; F ve B'de None), jest hedefleri (5 ofset,
task_math.GEST_CHANNELS), LED (0/1 ve kip), BEKLE rampasi / PWM kapali, getup yeniden baslatma ve devir bayraklari,
isaretler (uygulama / kuyruk / red / yok sayma / uyari) ve tamamlanmalar (zaman cizelgesi icin).

Durumlar: A AYAKTA, OE OTURMA ONCESI (dur sonrasi 1.0 s yerlesme), Y YURUYOR, D DONUYOR, O OTURUYOR, M OTURMUS, K
KALKIYOR, F DUSMUS (getup), B BEKLE. O ve K gecisleri tamamlanir, kesilmez. Kuyruk derinligi 1 (sonraki bekleyenin
yerine gecer; jestler kuyruga girmez); dusmede, B'ye giriste ve dur ile temizlenir.

Yorumlar (spec sessiz; p5k-notlar.md): kanal doluyken (jest ya da sonumlemesi suruyor) yeni jest kelimesi red;
F -> A devrinin sifir komut suresinde jest kelimeleri red (kuyruga giremez), digerleri kuyruga; red isaretinin cift
tiki kanal doluysa atlanir (LED yine yanip soner); bias penceresi kesintisiz duraganlik suresidir (>= 1 s, en cok
BIAS_RUN_MAX_S), ortalamasi bias'tir. BEKLE'de PWM kapali, robotta servolar gevser; simulasyon olculen pozu tutar.
"""
import json
import math
import os

import numpy as np

from . import config as C
from . import sensors as SENS
from . import task_math as T

F32 = np.float32
DT = C.CTRL_DT
STATES = ("A", "OE", "Y", "D", "O", "M", "K", "F", "B")
WORDS = ("gel", "dur", "geri", "saga", "sola", "otur", "kalk", "egil", "yuksel", "evet", "hayir", "vak")
SPOKEN = {"gel": "gel", "dur": "dur", "geri": "geri", "saga": "sağa dön", "sola": "sola dön", "otur": "otur",
          "kalk": "kalk", "egil": "eğil", "yuksel": "yüksel", "evet": "evet", "hayir": "hayır", "vak": "vak"}
WORD_ROW = {"gel": "hareket", "geri": "hareket", "saga": "donus", "sola": "donus", "dur": "dur", "otur": "otur",
            "kalk": "kalk", "egil": "yukseklik", "yuksel": "yukseklik", "evet": "bas_jest", "hayir": "bas_jest",
            "vak": "vak"}
ROWS = ("hareket", "donus", "dur", "otur", "kalk", "yukseklik", "bas_jest", "vak", "dusme", "tamam")
WORD_GESTURE = {"evet": T.GEST_NOD, "hayir": T.GEST_SHAKE, "vak": T.GEST_BEAK}
POLICIES = ("none", "main", "getup")

# --- K5 durum x olay tablosu (veri). Eylem adlari Supervisor._act_<ad> yontemleridir ("-": etkisiz, isaret yok).
# Hucre anlamlari (spec K5 tablosu): git / don = ->Y / ->D (OE'den otur iptal, Y'de yon yenilenir, D'de yeni yon);
# kuyruk = bekletilir; kalk_bekle = kalk, bekleyen = komut, ->K; red = uygulanmaz + red isareti; jest_son = jest
# sonumlenir (A'da dur); dur = ->A (dur, jest sonumlenir); iptal = ->A (otur iptal); kuyruk_sil = kuyruk temizlenir (O, M,
# K'de dur; gecis surer); getup_kes = getup kesilir, ->B; otur = son hizdan 1.0 s gectiyse ->O, yoksa ->OE; dur_otur =
# dur, ->OE; kalk = ->K; b_kalk = dusmusse ->F, oturusta ->K; yukseklik = h_cmd -+ dH (uygula); jest = jest (M'de gaga
# <= 15 derece, task_math.gesture_gate); dus = ->F (O, M, K'de K5 duruşa gore olcut); oe_bitti = 1.0 s doldu ->O;
# hareket_bitti = ->A; t2 = T2 ->M, bekleyen varsa kalk ve bekleyen ->K, suresi dolarsa red ->B; t1 = T1 ->A, bekleyen
# uygulanir, basarisiz kalkis kurali; getup_bitti = basari ->A (devir), 12 s'de basarisiz: 2. deneme, sonra ->B.
TABLE = {
    "hareket":   {"A": "git", "OE": "git", "Y": "git", "D": "git", "O": "kuyruk", "M": "kalk_bekle", "K": "kuyruk",
                  "F": "red", "B": "red"},
    "donus":     {"A": "don", "OE": "don", "Y": "don", "D": "don", "O": "kuyruk", "M": "kalk_bekle", "K": "kuyruk",
                  "F": "red", "B": "red"},
    "dur":       {"A": "jest_son", "OE": "dur", "Y": "dur", "D": "dur", "O": "kuyruk_sil", "M": "kuyruk_sil",
                  "K": "kuyruk_sil", "F": "getup_kes", "B": "-"},
    "otur":      {"A": "otur", "OE": "-", "Y": "dur_otur", "D": "dur_otur", "O": "-", "M": "-", "K": "kuyruk",
                  "F": "red", "B": "red"},
    "kalk":      {"A": "-", "OE": "iptal", "Y": "-", "D": "-", "O": "kuyruk", "M": "kalk", "K": "-", "F": "red",
                  "B": "b_kalk"},
    "yukseklik": {"A": "yukseklik", "OE": "red", "Y": "yukseklik", "D": "red", "O": "red", "M": "red", "K": "red",
                  "F": "red", "B": "red"},
    "bas_jest":  {"A": "jest", "OE": "red", "Y": "jest", "D": "jest", "O": "red", "M": "red", "K": "red", "F": "red",
                  "B": "red"},
    "vak":       {"A": "jest", "OE": "jest", "Y": "jest", "D": "jest", "O": "red", "M": "jest", "K": "red", "F": "red",
                  "B": "red"},
    "dusme":     {"A": "dus", "OE": "dus", "Y": "dus", "D": "dus", "O": "dus", "M": "dus", "K": "dus", "F": "-",
                  "B": "-"},
    "tamam":     {"A": "-", "OE": "oe_bitti", "Y": "hareket_bitti", "D": "hareket_bitti", "O": "t2", "M": "-", "K": "t1",
                  "F": "getup_bitti", "B": "-"},
}
# F -> A devrinin sifir komut suresi (handoff_zero_cmd_s; K5 giris eylemi): kelimeler kuyruga, dur A hucresi, jestler red
HANDOFF = {"dur": "jest_son", "bas_jest": "red", "vak": "red"}
HANDOFF_DEFAULT = "kuyruk"
# fizikte erisilemeyen hucreler (A10 kapsama raporu; gerekce)
UNREACHABLE = {
    ("dusme", "F"): "dusme algisi F'de calismaz (getup politikasi aktif; K5 '—')",
    ("dusme", "B"): "dusme algisi B'de calismaz (politika yok; K5 '—')",
    ("tamam", "B"): "B'de tamamlanma olayi yok (PWM kapali bekleme)",
}
# "tamam" satirinin A ve M hucreleri ('—'): A'da yukseklik / jest bitisi, M'de gaga jesti bitisi (durum degismez)

# --- turetilmis sabitler (tr35: COMMAND_RANGES, H_*, SUPERVISOR, MAIN, GETUP; geometri elle yazilmaz) -------------
# K4 / K5 spec sayilari (zaman, aci, mesafe; geometriden bagimsiz)
GEL_S, GERI_M, GERI_MAX_S, TURN_DEG, TURN_MAX_S = 12.0, 0.5, 8.0, 90.0, 8.0
SETTLE_S, B_RAMP_S, WAKE_LED_S = 1.0, 0.5, 15.0
FALL_POSE_DEG, SEAT_TOL_DEG, STAND_TOL_S = 35.0, 15.0, 0.5
BIAS_WIN_S, BIAS_QD, BIAS_MAX, BIAS_STD_K, BIAS_RUN_MAX_S = 1.0, 0.2, 0.06, 2.0, 5.0
LED_RED_N, LED_RED_HALF_S, LED_B_HALF_S = 3, 0.15, 0.5
GETUP_TRIES, STAND_TRIES = 2, 2
SPEED_FRAC = 0.8


def steps(sec):
    """Sure (s) -> tam kontrol adimi (firmware ile ayni tamsayi esikler)."""
    return int(round(float(sec) / DT))


def derived():
    """K4 / K5 sabitleri config'ten (tr35 surecinde tr35 degerleri): hizlar, dH, T1/T2 pencereleri, devir, getup."""
    R, S, M, G = C.COMMAND_RANGES, C.SUPERVISOR, C.MAIN, C.GETUP
    vx_gel, vx_geri, wz = SPEED_FRAC * R["vx"][1], SPEED_FRAC * R["vx"][0], SPEED_FRAC * R["wz"][1]
    d = {"vx_gel": vx_gel, "vx_geri": vx_geri, "wz_turn": wz, "dh": 0.5 * (C.H_HOME - C.H_MIN_WALK),
         "h_lo": C.H_MIN_WALK, "h_hi": C.H_HOME, "h_home": C.H_HOME,
         "geri_s": min(GERI_M / abs(vx_geri), GERI_MAX_S) if vx_geri else GERI_MAX_S,
         "t1_first_s": M["standup_window_s"], "t1_hard_s": C.STAND_RAMP_S + S["ready_hold_s"] + STAND_TOL_S,
         "t2_limit_s": M["sitdown_window_s"] + S["ready_hold_s"], "ready_hold_s": S["ready_hold_s"],
         "fall_gz": S["fall_gz"], "fall_steps": int(S["fall_steps"]), "handoff_s": S["handoff_zero_cmd_s"],
         "stand_gz": M["stand_gz"], "seat_gz": M["seat_gz"], "gyro_ok": G["success"]["gyro"],
         "seat_pitch": math.radians(C.SEAT_PITCH_DEG), "getup_s": C.GETUP_EPISODE_S,
         "ok_gz": G["success"]["gz"], "ok_pose_rms": G["success"]["pose_rms"],
         "sigma_gyro": SENS.IMU["gyro_sigma"]}
    return d


K = derived()


# --- kimlik tablosu (K4) -------------------------------------------------------------------------------------------
ID_TABLE_PATH = os.path.join(C.ROOT, "configs", "komut_kimlik.json")
LEARNING_IDS = tuple(range(200, 209))


class IdTable(object):
    """Beyaz liste: wake (uyanma kimlikleri), cmd {kimlik: kelime} (coktan bire), learning (200-208: uyari)."""

    def __init__(self, wake, cmd, learning=LEARNING_IDS, example=False, path=None):
        self.wake, self.cmd = frozenset(int(i) for i in wake), {int(k): str(v) for k, v in cmd.items()}
        self.learning, self.example, self.path = frozenset(int(i) for i in learning), bool(example), path
        bad = sorted(set(self.cmd.values()) - set(WORDS))
        if bad or self.wake & set(self.cmd) or not self.wake:
            raise ValueError("kimlik tablosu gecersiz: bilinmeyen kelime %s / uyanma-komut cakismasi / uyanma yok" % bad)

    def ids_of(self, word):
        return sorted(k for k, v in self.cmd.items() if v == word)

    def missing(self):
        return [w for w in WORDS if not self.ids_of(w)]

    def classify(self, i):
        """Kimlik -> ("wake", None) | ("cmd", kelime) | ("warn", None) (ogrenme kipi) | ("ignore", None)."""
        i = int(i)
        if i in self.wake:
            return "wake", None
        if i in self.cmd:
            return "cmd", self.cmd[i]
        return ("warn", None) if i in self.learning else ("ignore", None)


def load_id_table(path=None):
    """configs/komut_kimlik.json (sema 1; G-SES-a ogrenme adimi yazar, depodaki ornektir: "ornek": true)."""
    p = path or ID_TABLE_PATH
    with open(p) as f:
        d = json.load(f)
    if d.get("schema") != 1:
        raise ValueError("%s: schema %r (beklenen 1)" % (p, d.get("schema")))
    return IdTable(d["uyanma"], d["komutlar"], d.get("ogrenme_kimlikleri", LEARNING_IDS), d.get("ornek", False), p)


# --- duruş geometrisi (K5 dusme / T2): gravity_obs ile referans egim arasindaki aci ------------------------------
def _f32(x):
    return np.asarray(x, dtype=F32)


def tilt_dev(g, pitch):
    """Olculen izdusumlu yercekimi g (3,) ile pitch (rad, + = burun asagi) referansinin [sin p, 0, -cos p] arasindaki
    aci (rad, float32): egim sapmasi (pitch ve roll birlikte)."""
    g = _f32(g)
    g = g / np.maximum(np.sqrt(np.sum(g * g)), F32(1e-9))
    p = _f32(pitch)
    c = g[0] * np.sin(p) - g[2] * np.cos(p)
    return np.arccos(np.clip(c, F32(-1.0), F32(1.0))).astype(F32)


def seat_compatible(g):
    """Oturusla uyumlu durus (K5): egim SEAT_PITCH_DEG'den en cok 15 derece ve gz < seat_gz."""
    return bool(tilt_dev(g, K["seat_pitch"]) <= F32(math.radians(SEAT_TOL_DEG)) and _f32(g)[2] < F32(K["seat_gz"]))


class Inputs(object):
    """Bir kontrol adiminin girdileri (float32'ye cevrilir). ids: bu adimda gelen kimlikler (gelis sirasi)."""
    __slots__ = ("t", "ids", "gravity", "gyro", "qd", "q", "alpha", "h_ref")

    def __init__(self, t, ids, gravity_obs, gyro_obs, qd_obs, q_obs, alpha, h_ref):
        self.t, self.ids = float(t), [int(i) for i in (ids or ())]
        self.gravity, self.gyro = _f32(gravity_obs).reshape(3), _f32(gyro_obs).reshape(3)
        self.qd, self.q = _f32(qd_obs).reshape(-1), _f32(q_obs).reshape(-1)
        self.alpha, self.h_ref = F32(alpha), F32(h_ref)


class Supervisor(object):
    """K5 denetleyicisi (tek robot). table: IdTable; home_q (14,) HOME (getup -> main poz RMS); state "A" | "M"."""

    def __init__(self, table, home_q, state="A"):
        if state not in ("A", "M"):
            raise ValueError("baslangic durumu A ya da M olmali")
        self.tab, self.home = table, _f32(home_q).reshape(-1)
        self.n = 0                                             # kontrol adimi sayaci
        self.state, self.st_n = state, 0
        self.h_cmd = F32(K["h_home"])
        self.vel = np.zeros(3, F32)
        self.sit = F32(1.0 if state == "M" else 0.0)
        self.n_since_vel = 10 ** 6                              # son sifir olmayan hiz komutundan beri adim
        self.pending = None
        self.motion = None                                      # Y: ("gel" | "geri", adim siniri)
        self.turn_dir, self.turn_int = F32(0.0), F32(0.0)
        self.hold, self.first_ok = 0, False                     # T1 / T2 / getup hazirlik tutma sayaci
        self.stand_try, self.recovery = 0, False                # basarisiz kalkis kurali
        self.getup_try = 0
        self.handoff_n = 0                                      # F -> A sifir komut suresi (adim)
        self.fall_n = 0
        self.h_target_pending = False                           # yukseklik komutu tamamlanma bekleniyor
        self.gst = T.gesture_init(np, np.zeros(1, F32))
        self.g_req, self.g_cut = np.zeros(1, F32), np.zeros(1, F32)
        self.g_active, self.g_word = False, None
        self.led_wake_until, self.red_start = -1, -10 ** 6
        w = steps(BIAS_WIN_S)
        self.ring, self.ring_i, self.ring_fill = np.zeros((w, 3), F32), 0, 0
        self.still_n, self.run_sum, self.run_n = 0, np.zeros(3, F32), 0
        self.bias, self.bias_updates = np.zeros(3, F32), 0
        self.turn_bias_updated = False
        self.b_n = 0
        self.main_alpha = F32(0.0)
        self.policy = "main"
        self._sig, self._done, self._ev, self._fired, self._inp = [], [], [], False, None
        self._getup_restart, self._handover = False, False

    # -- yardimcilar
    def _signal(self, kind, word=None, cell=None, extra=None):
        self._sig.append((kind, word, cell, extra))

    def _to(self, state):
        if state != self.state:
            self.state, self.st_n = state, 0
            self.hold, self.first_ok = 0, False

    def _set_vel(self, v):
        self.vel = _f32(v).reshape(3)

    def _cut(self):
        self.g_cut[:] = 1.0

    def _clear_queue(self, why):
        if self.pending is not None:
            self._signal("clear", self.pending, None, why)
        self.pending = None

    def _red(self, word, cell):
        """Red isareti: LED 3 kez yanip soner; A, OE, Y, D'de cift gaga tiki (kanal bossa; tanima bittikten sonra)."""
        self.red_start = self.n
        self._signal("red", word, cell)
        if self.state in ("A", "OE", "Y", "D") and not self.g_active and self.g_req[0] < 0.5:
            self.g_req[:] = T.GEST_CLICK2
            self.g_word = "red"

    # -- K5 eylemleri (TABLE hucreleri) -------------------------------------------------------------------------
    def _act_noop(self, word, cell):
        self._signal("noop", word, cell)

    def _act_red(self, word, cell):
        self._red(word, cell)

    def _act_kuyruk(self, word, cell):
        if self.pending is not None:
            self._signal("replace", self.pending, cell, word)
        self.pending = word
        self._signal("queue", word, cell)

    def _act_git(self, word, cell):
        vx = K["vx_gel"] if word == "gel" else K["vx_geri"]
        self._to("Y")
        self.st_n = 0                                           # yon yenilenir: sure bastan
        self._set_vel((vx, 0.0, 0.0))
        self.sit = F32(0.0)
        self.motion = (word, steps(GEL_S if word == "gel" else K["geri_s"]))
        self._signal("apply", word, cell)

    def _act_don(self, word, cell):
        wz = K["wz_turn"] if word == "sola" else -K["wz_turn"]   # saga = -wz (saat yonu), sola = +wz
        self._to("D")
        self.st_n = 0
        self._set_vel((0.0, 0.0, wz))
        self.sit = F32(0.0)
        self.turn_dir, self.turn_int = F32(1.0 if wz > 0 else -1.0), F32(0.0)
        self.turn_bias_updated = self.bias_updates > 0
        self._signal("apply", word, cell)

    def _stop(self):
        if np.any(self.vel != 0.0):
            self.n_since_vel = 0
        self._set_vel((0.0, 0.0, 0.0))
        self.motion = None

    def _act_dur(self, word, cell):
        self._stop()
        self._cut()
        self._clear_queue("dur")
        self._to("A")
        self._signal("apply", word, cell)
        self._done.append(("dur", None))

    def _act_jest_son(self, word, cell):
        self._stop()
        self._cut()
        self._clear_queue("dur")                               # devir suresi (sifir komut) surer
        self._signal("apply", word, cell)
        self._done.append(("dur", None))

    def _act_iptal(self, word, cell):
        self._to("A")
        self._signal("apply", word, cell)

    def _act_kuyruk_sil(self, word, cell):
        self._cut()
        self._clear_queue("dur")
        self._signal("apply", word, cell)
        self._done.append(("dur", None))

    def _act_getup_kes(self, word, cell):
        self._clear_queue("B")
        self._signal("apply", word, cell)
        self._enter_b("dur")
        self._done.append(("dur", None))

    def _act_otur(self, word, cell):
        self._cut()
        if self.n_since_vel >= steps(SETTLE_S):
            self._enter_o()
        else:
            self._to("OE")
        self._signal("apply", word, cell)

    def _act_dur_otur(self, word, cell):
        self._stop()
        self._cut()
        self._to("OE")
        self._signal("apply", word, cell)

    def _act_kalk(self, word, cell):
        self._enter_k()
        self._signal("apply", word, cell)

    def _act_kalk_bekle(self, word, cell):
        self._enter_k()
        self.pending = word
        self._signal("apply", "kalk", cell, word)
        self._signal("queue", word, cell)

    def _act_b_kalk(self, word, cell):
        if seat_compatible(self._inp.gravity):
            self._enter_k(from_b=True)
            self._signal("apply", word, cell, "K")
        else:
            self._enter_f(restart=True)
            self._signal("apply", word, cell, "F")

    def _act_yukseklik(self, word, cell):
        d = -K["dh"] if word == "egil" else K["dh"]
        self.h_cmd = F32(np.clip(F32(self.h_cmd + F32(d)), F32(K["h_lo"]), F32(K["h_hi"])))
        self.h_target_pending = True
        self._signal("apply", word, cell)

    def _act_jest(self, word, cell):
        if self.g_active or self.g_req[0] > 0.5:               # kanal dolu: jestler kuyruga girmez -> red
            self._red(word, cell + ":dolu")
            return
        self.g_req[:] = WORD_GESTURE[word]
        self.g_word = word
        self._signal("apply", word, cell)

    def _act_dus(self, word, cell):
        self._signal("fall", None, cell)
        self._enter_f(restart=True)

    # -- tamamlanmalar ("tamam" satiri)
    def _act_oe_bitti(self, word, cell):
        if self.n_since_vel >= steps(SETTLE_S):
            self._signal("done", "OE", cell)
            self._fired = True
            self._enter_o()

    def _act_hareket_bitti(self, word, cell):
        what = "turn" if self.state == "D" else "motion"
        self._stop()
        self._to("A")
        self._fired = True
        self._signal("done", what, cell)
        self._done.append((what, None))

    def _act_t2(self, word, cell):
        """O: T2 (alpha = 1, gz < seat_gz, |gyro| < esik, oturus egimine <= 15 derece) ready_hold_s boyunca; sure
        4.0 s + ready_hold_s. Basari -> M; bekleyen ya da basarisiz kalkis kurtarmasi -> kalk (->K, bekleyen korunur)."""
        i = self._inp
        ok = (i.alpha >= F32(1.0 - 1e-6) and i.gravity[2] < F32(K["seat_gz"]) and self._gyro_ok()
              and tilt_dev(i.gravity, K["seat_pitch"]) <= F32(math.radians(SEAT_TOL_DEG)))
        self.hold = self.hold + 1 if ok else 0
        if self.hold >= steps(K["ready_hold_s"]):
            self._signal("done", "T2", cell)
            self._fired = True
            self._done.append(("T2", None))
            self._to("M")
            if self.recovery or self.pending is not None:
                self._signal("apply", "kalk", cell, "kurtarma" if self.recovery else self.pending)
                self._enter_k(keep_try=self.recovery)
        elif self.st_n * DT > K["t2_limit_s"] + 1e-6:
            self._signal("timeout", "T2", cell)
            self._fired = True
            self._red("otur", cell)
            self._enter_b("T2")

    def _act_t1(self, word, cell):
        """K: T1 (alpha = 0, gz < stand_gz, |gyro| < esik) uygulamadan sonraki t1_first_s icinde ilk kez saglanir,
        ready_hold_s kesintisiz tutar; sert sinir t1_hard_s. Basari -> A + bekleyen; sure doldu (dusme yok): 1. kez
        oturusa donus (O, kurtarma) ve tekrar kalk, 2. kez red ->B."""
        i = self._inp
        ok = i.alpha <= F32(1e-6) and i.gravity[2] < F32(K["stand_gz"]) and self._gyro_ok()
        self.hold = self.hold + 1 if ok else 0
        start = (self.st_n - self.hold + 1) * DT
        if self.hold >= steps(K["ready_hold_s"]) and start <= K["t1_first_s"] + 1e-6 \
                and self.st_n * DT <= K["t1_hard_s"] + 1e-6:
            self._signal("done", "T1", cell)
            self._fired = True
            self._done.append(("T1", None))
            self.stand_try, self.recovery = 0, False
            self._to("A")
            p, self.pending = self.pending, None
            if p is not None:
                self._signal("apply_pending", p, cell)
                self._dispatch(p)
        elif self.st_n * DT > K["t1_hard_s"] + 1e-6:
            self._signal("timeout", "T1", cell, self.stand_try)
            self._fired = True
            if self.stand_try >= STAND_TRIES:
                self._red("kalk", cell)
                self._enter_b("T1")
            else:
                self.recovery = True
                self._enter_o()

    def _act_getup_bitti(self, word, cell):
        """F: getup basarisi (play.Supervisor olcutu, olculen sinyallerle: gz < -0.9, poz RMS < 0.3, |gyro| <
        esik, z haric) ready_hold_s -> A (devir); getup_s'de basarisiz: 2. deneme, sonra ->B."""
        i = self._inp
        rms = np.sqrt(np.mean(np.square(i.q[:len(self.home)] - self.home))).astype(F32)
        ok = i.gravity[2] < F32(K["ok_gz"]) and rms < F32(K["ok_pose_rms"]) and self._gyro_ok()
        self.hold = self.hold + 1 if ok else 0
        if self.hold >= steps(K["ready_hold_s"]):
            self._signal("done", "getup", cell)
            self._fired = True
            self._done.append(("getup", None))
            self._enter_a_handover()
        elif self.st_n >= steps(K["getup_s"]):
            self._signal("timeout", "getup", cell, self.getup_try)
            self._fired = True
            if self.getup_try >= GETUP_TRIES:
                self._red(None, cell)
                self._enter_b("getup")
            else:
                self._enter_f(restart=False)

    # -- giris eylemleri
    def _enter_o(self):
        self._stop()
        self._cut()
        self.sit = F32(1.0)
        self._to("O")

    def _enter_k(self, from_b=False, keep_try=False):
        self._cut()
        self._stop()
        self.sit, self.h_cmd = F32(0.0), F32(K["h_home"])
        self.stand_try = (self.stand_try if keep_try else 0) + 1
        self.main_alpha = F32(1.0 if from_b else 0.0)
        self._to("K")
        self.st_n = 0

    def _enter_f(self, restart):
        """->F: kuyruk temizlenir, jest kesilir, getup denemesi (restart: 1. deneme; degilse 2. deneme)."""
        self._clear_queue("dusme")
        self._cut()
        self._stop()
        self.getup_try = 1 if restart else self.getup_try + 1
        self.stand_try, self.recovery, self.fall_n = 0, False, 0
        self._to("F")
        self.st_n, self.hold = 0, 0
        self._getup_restart = True

    def _enter_b(self, why):
        self._clear_queue("B")
        self._cut()
        self._stop()
        self.stand_try, self.recovery, self.b_n = 0, False, 0
        self._to("B")
        self._signal("bekle", None, None, why)

    def _enter_a_handover(self):
        """F -> A (devir): h_cmd = H_HOME, sit = 0, kuyruk temizlenir; handoff_zero_cmd_s sifir komut (kelimeler
        kuyruga, sure bitince uygulanir)."""
        self._clear_queue("devir")
        self.h_cmd, self.sit = F32(K["h_home"]), F32(0.0)
        self._stop()
        self.getup_try = 0
        self.handoff_n = steps(K["handoff_s"])
        self.main_alpha = F32(0.0)
        self._to("A")
        self._handover = True

    # -- olcutler
    def _gyro_ok(self):
        g = self._inp.gyro
        return bool(np.sqrt(np.sum(g * g)) < F32(K["gyro_ok"]))

    def _fall_cond(self):
        """Dusme kosulu (debounce'suz): A, OE, Y, D'de gz_obs > fall_gz; O, M, K'de egim hem pitch_ref(alpha)'dan hem
        oturus pozundan > 35 derece (oturusla uyumlu durus dusme sayilmaz)."""
        i = self._inp
        if self.state in ("A", "OE", "Y", "D"):
            return bool(i.gravity[2] > F32(K["fall_gz"]))
        pr = F32(T.pitch_ref(np, np.asarray([i.h_ref], F32), np.asarray([i.alpha], F32))[0])
        lim = F32(math.radians(FALL_POSE_DEG))
        return bool(tilt_dev(i.gravity, pr) > lim and tilt_dev(i.gravity, K["seat_pitch"]) > lim)

    def _bias_update(self):
        """Gyro bias (K5): 1 s pencerede eksen basina std(gyro_obs) < 2 sigma_gyro, butun bacak |qd_obs| < 0.2 rad/s,
        hiz komutu sifir, durum A ya da M. Bias kesintisiz duraganlik suresinin (pencere; en cok BIAS_RUN_MAX_S)
        ortalamasidir, |b| <= 0.06 rad/s (eksen basina) kirpilir."""
        i, W = self._inp, len(self.ring)
        self.ring[self.ring_i] = i.gyro
        self.ring_i, self.ring_fill = (self.ring_i + 1) % W, min(self.ring_fill + 1, W)
        ok = (self.state in ("A", "M") and not np.any(self.vel != 0.0)
              and bool(np.all(np.abs(i.qd[:10]) < F32(BIAS_QD))))
        self.still_n = self.still_n + 1 if ok else 0
        still = False
        if self.still_n >= W and self.ring_fill == W:
            sd = np.std(self.ring, axis=0).astype(F32)
            still = bool(np.all(sd < F32(BIAS_STD_K * K["sigma_gyro"])))
        if not still:
            self.run_n = 0
            return
        if self.run_n == 0:
            self.run_sum, self.run_n = np.sum(self.ring, axis=0).astype(F32), W
        elif self.run_n < steps(BIAS_RUN_MAX_S):
            self.run_sum, self.run_n = (self.run_sum + i.gyro).astype(F32), self.run_n + 1
        else:
            return
        self.bias = np.clip(self.run_sum / F32(self.run_n), F32(-BIAS_MAX), F32(BIAS_MAX)).astype(F32)
        self.bias_updates += 1

    # -- olay dagitimi
    def _cell(self, row):
        if row != "dusme" and row != "tamam" and self.state == "A" and self.handoff_n > 0:
            return HANDOFF.get(row, HANDOFF_DEFAULT), "%s/A_devir" % row
        return TABLE[row][self.state], "%s/%s" % (row, self.state)

    def _dispatch(self, word):
        """Kelime olayi: durumun hucresi (TABLE ya da devir kurali) -> eylem; hucre kapsama listesine."""
        act, cell = self._cell(WORD_ROW[word])
        self._ev.append(cell)
        getattr(self, "_act_" + ("noop" if act == "-" else act))(word, cell)
        return cell

    def _event(self, row):
        """Ic olay (dusme / tamam): eylem calisir; tamamlanma ya da sure gerceklesince (dusme her zaman) kapsamaya."""
        act, cell = self._cell(row)
        self._fired = row == "dusme"
        if act != "-":
            getattr(self, "_act_" + act)(None, cell)
        if self._fired:
            self._ev.append(cell)

    def step(self, inp):
        """Bir kontrol adimi: (1) sayaclar, devir suresi, bias; (2) dusme (debounce fall_steps); (3) tamamlanma / sure
        (gecis adiminda degil); (4) kimlik olaylari gelis sirasiyla; (5) jest kanali (task_math.gesture_step), LED,
        ciktilar. Dondurur dict (ciktilar + "events": bu adimda tetiklenen tablo hucreleri "satir/durum")."""
        self._inp, self._sig, self._done, self._ev = inp, [], [], []
        self._getup_restart, self._handover = False, False
        self.st_n += 1
        self.n_since_vel = min(self.n_since_vel + 1, 10 ** 6)
        if self.state == "B":
            self.b_n += 1
        if self.handoff_n > 0:
            self.handoff_n -= 1
            if self.handoff_n == 0 and self.pending is not None and self.state == "A":
                p, self.pending = self.pending, None
                self._signal("apply_pending", p, "devir")
                self._dispatch(p)
        self._bias_update()
        s0 = self.state
        # 2. dusme
        if self.state not in ("F", "B"):
            self.fall_n = self.fall_n + 1 if self._fall_cond() else 0
            if self.fall_n >= K["fall_steps"]:
                self._event("dusme")
        # 3. tamamlanma / sure (bu adimda durum degismediyse)
        if self.state == "D":
            g = inp.gravity / np.maximum(np.sqrt(np.sum(inp.gravity * inp.gravity)), F32(1e-9))
            self.turn_int = F32(self.turn_int + F32(-np.dot(g, inp.gyro - self.bias)) * F32(DT))
        if self.state == s0:
            s = self.state
            due = {"OE": self.n_since_vel >= steps(SETTLE_S),
                   "Y": self.motion is not None and self.st_n >= self.motion[1],
                   "D": bool(self.turn_dir * self.turn_int >= F32(math.radians(TURN_DEG))) or self.st_n >= steps(TURN_MAX_S),
                   "O": True, "K": True, "F": True}.get(s, False)
            if due:
                self._event("tamam")
            if self.h_target_pending and abs(float(inp.h_ref) - float(self.h_cmd)) < 1e-6:
                self.h_target_pending = False
                self._signal("done", "height", "tamam/" + self.state)
                self._done.append(("height", None))
                if self.state == "A":
                    self._ev.append("tamam/A")
        # 4. kimlikler
        for i in inp.ids:
            kind, word = self.tab.classify(i)
            if kind == "wake":
                self.led_wake_until = self.n + steps(WAKE_LED_S)
                self._signal("wake", None, None, i)
            elif kind == "cmd":
                self._dispatch(word)
            else:
                self._signal(kind, None, None, i)               # ignore (beyaz liste disi) / warn (200-208)
        # 5. jest kanali
        was = self.g_active
        if was or self.g_req[0] > 0.5:
            self.gst, gout, gact, gstart = T.gesture_step(np, self.gst, self.g_req, np.asarray([inp.alpha], F32),
                                                          self.g_cut)
            self.gst = {k: np.asarray(v, F32) for k, v in self.gst.items()}
        else:                                  # bos kanal, istek yok: gesture_step'in sonucu ayni (sifir), hizli yol
            gout, gact, gstart = np.zeros((1, len(T.GEST_CHANNELS)), F32), np.zeros(1), np.zeros(1)
        self.g_active = bool(gact[0] > 0.5)
        if self.g_req[0] > 0.5 and gstart[0] < 0.5:
            self._signal("gesture_refused", self.g_word, None)
        if gstart[0] > 0.5:
            self._signal("gesture", self.g_word, None, T.GEST_NAMES[int(self.g_req[0])])
        if was and not self.g_active:
            self._done.append(("gesture", self.g_word))
            if self.state in ("A", "M"):
                self._ev.append("tamam/" + self.state)
        self.g_req[:], self.g_cut[:] = 0.0, 0.0
        out = self._outputs(np.asarray(gout[0], F32))
        out["events"] = list(self._ev)
        self.n += 1
        return out

    def _outputs(self, gout):
        s = self.state
        pol = "getup" if s == "F" else ("none" if s == "B" else "main")
        self.policy = pol
        cmd = None
        if pol == "main":
            if self.handoff_n > 0:
                cmd = np.array([0.0, 0.0, 0.0, K["h_home"], 0.0], F32)
            else:
                v = self.vel if s in ("Y", "D") else np.zeros(3, F32)
                cmd = np.concatenate([v, [self.h_cmd, self.sit]]).astype(F32)
        led, mode = 0, "off"
        if self.n <= self.led_wake_until:
            led, mode = 1, "wake"
        if s == "B":
            led, mode = int((self.b_n // steps(LED_B_HALF_S)) % 2 == 0), "bekle"
        k = self.n - self.red_start
        if 0 <= k < 2 * LED_RED_N * steps(LED_RED_HALF_S):
            led, mode = int((k // steps(LED_RED_HALF_S)) % 2 == 0), "red"
        ramp = F32(min(self.b_n / float(steps(B_RAMP_S)), 1.0)) if s == "B" else F32(0.0)
        return {"state": s, "policy": pol, "command": cmd, "gesture": gout if pol == "main" else np.zeros(5, F32),
                "led": led, "led_mode": mode, "ramp": ramp, "pwm_off": bool(s == "B" and self.b_n >= steps(B_RAMP_S)),
                "getup_restart": self._getup_restart, "handover": self._handover, "main_alpha": self.main_alpha,
                "pending": self.pending, "bias": self.bias.copy(), "turn_int": F32(self.turn_int),
                "signals": list(self._sig), "done": list(self._done)}


# --- main <-> getup devri (play.Supervisor'in mantigi; K5 "dusme algisi", C.SUPERVISOR) -----------------------------
def measured_state(s):
    """task_state satiri -> firmware'in olcebildigi durum: tr35 (gravity_obs varsa) q / qd / gyro olculen, quat
    tamamlayici filtre ciktisindan (sensors.obs_state); yoksa (xl330) s aynen (eski play.Supervisor davranisi)."""
    if "gravity_obs" not in s:
        return s
    return SENS.obs_state(np, s, s["q_obs"], s["qd_obs"], s["gyro_obs"], s["gravity_obs"])


class FallMonitor(object):
    """main <-> getup devri (spec 3.7, C.SUPERVISOR; eski play.Supervisor, play ve eval cagirir). update(s) her kontrol
    adiminda AKTIF politikanin ortam durumuyla (VecEnv.task_state(), tek ortam satiri) cagrilir; devir olursa yeni modu
    ("getup" | "main") dondurur, yoksa None.
      main -> getup: gz > fall_gz art arda fall_steps adim (K5: A, OE, Y, D olcutu; tr35'te olculen gz_obs)
      getup -> main: task_math.getup_success_now(use_height=False) ready_hold_s boyunca (z olculmez; tr35'te olculen
                     gravity / gyro / pot q)
    Devirde cagiran: last_action = 0, alpha = 0, h_ref = H_HOME (VecEnv.set_state) ve handoff_zero_cmd_s sifir komut.
    Sesli komut katmaninda ayni olcutler Supervisor icinde (O, M, K'de duruşa gore dusme ve F durumu)."""

    def __init__(self, cfg=None, dt=C.CTRL_DT, mode="main"):
        self.cfg = dict(C.SUPERVISOR if cfg is None else cfg)
        self.dt = dt
        self.mode = mode
        self.n_fall = 0
        self.t_ready = np.zeros(1)

    def update(self, s):
        m = measured_state(s)
        if self.mode == "main":
            gz = float(np.max(T.projected_gravity(np, m["quat"])[..., 2]))
            self.n_fall = self.n_fall + 1 if gz > self.cfg["fall_gz"] else 0
            if self.n_fall >= self.cfg["fall_steps"]:
                self.mode, self.n_fall, self.t_ready = "getup", 0, np.zeros(1)
                return "getup"
            return None
        ok = T.getup_success_now(np, m, use_height=False)
        self.t_ready, held = T.hold_timer(np, self.t_ready, ok.reshape(-1)[:1], self.cfg["ready_hold_s"], self.dt)
        if held[0] > 0.5:
            self.mode, self.t_ready = "main", np.zeros(1)
            return "main"
        return None


# --- altin iz satiri (firmware karsilastirmasi; rl/tests/data/komut_*.csv.gz) ---------------------------------------
TRACE_IN = ("t", "ids", "g0", "g1", "g2", "w0", "w1", "w2") + tuple("qd%d" % i for i in range(10)) + \
    tuple("q%d" % i for i in range(14)) + ("alpha", "h_ref")
TRACE_OUT_D = ("state", "policy", "has_cmd", "led", "led_mode", "pwm_off", "getup_restart", "handover", "pending",
               "signals")
TRACE_OUT_C = ("c0", "c1", "c2", "c3", "c4", "j0", "j1", "j2", "j3", "j4", "ramp", "b0", "b1", "b2", "turn_int")
LED_MODES = ("off", "wake", "red", "bekle")


def trace_row(inp, out):
    """Girdi + cikti -> {sutun: deger} (ayrik alanlar str/int, surekli float32)."""
    r = {"t": "%.2f" % inp.t, "ids": " ".join(str(i) for i in inp.ids)}
    for k, v in zip(("g0", "g1", "g2"), inp.gravity):
        r[k] = float(v)
    for k, v in zip(("w0", "w1", "w2"), inp.gyro):
        r[k] = float(v)
    for i in range(10):
        r["qd%d" % i] = float(inp.qd[i])
    for i in range(14):
        r["q%d" % i] = float(inp.q[i])
    r["alpha"], r["h_ref"] = float(inp.alpha), float(inp.h_ref)
    c = out["command"]
    r.update(state=out["state"], policy=out["policy"], has_cmd=int(c is not None), led=int(out["led"]),
             led_mode=out["led_mode"], pwm_off=int(out["pwm_off"]), getup_restart=int(out["getup_restart"]),
             handover=int(out["handover"]), pending=out["pending"] or "",
             signals=";".join("%s:%s:%s" % (k, w or "", cell or "") for k, w, cell, _ in out["signals"]))
    for i in range(5):
        r["c%d" % i] = float(c[i]) if c is not None else 0.0
        r["j%d" % i] = float(out["gesture"][i])
    r["ramp"] = float(out["ramp"])
    for i in range(3):
        r["b%d" % i] = float(out["bias"][i])
    r["turn_int"] = float(out["turn_int"])
    return r


def inputs_from_row(r):
    """trace_row girdilerinden Inputs (yeniden oynatma)."""
    ids = [int(x) for x in str(r["ids"]).split()] if str(r["ids"]).strip() else []
    f = lambda k: float(r[k])
    return Inputs(float(r["t"]), ids, [f("g0"), f("g1"), f("g2")], [f("w0"), f("w1"), f("w2")],
                  [f("qd%d" % i) for i in range(10)] + [0.0] * 4, [f("q%d" % i) for i in range(14)], f("alpha"),
                  f("h_ref"))


def compare_rows(a, b, atol=1e-6):
    """Iki iz satirinin ciktilari: ayrik birebir, surekli atol. Dondurur farkli sutunlar listesi."""
    bad = [k for k in TRACE_OUT_D if str(a[k]) != str(b[k])]
    bad += [k for k in TRACE_OUT_C if abs(float(a[k]) - float(b[k])) > atol]
    return bad
