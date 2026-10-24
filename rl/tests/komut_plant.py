# -*- coding: utf-8 -*-
"""
rl/commands.py test yardimcilari (spec eki K5): kinematik bitki, durum kurulumu ve sentetik senaryolar (birim testler,
degismez testleri ve tools/commands_golden.py altin izleri). Fizik yok: durus filtresi komutu izler (task_math
.posture_step), gravity referans egimi (pitch_ref) ya da olay pozlari, gyro z = komut wz (+ bias + istege bagli
sensor modeli gurultusu), bacak qd = 0, q = HOME.

  sup, pl = to_state("K")                         # denetleyici + bitki istenen durumda
  outs = run(sup, pl, n, script={adim: [kelime | kimlik]})
"""
import math

import numpy as np

from rl import commands as CM
from rl import config as C
from rl import sensors as SENS
from rl import task_math as T

TAB = None


def table():
    global TAB
    if TAB is None:
        TAB = CM.load_id_table()
    return TAB


def wid(word):
    """Kelime -> ilk kimlik (ornek tablo); 'wake' -> uyanma kimligi; int aynen."""
    if isinstance(word, int):
        return word
    if word == "wake":
        return min(table().wake)
    return table().ids_of(word)[0]


def home_q():
    """HOME (14) yer tutucu: getup -> main poz RMS'i icin bitki q'su ile ayni (sifir)."""
    return np.zeros(14, np.float32)


def g_of(pitch, roll=0.0):
    """pitch (+ burun asagi) / roll -> izdusumlu yercekimi (govde)."""
    return np.array([math.sin(pitch) * math.cos(roll), -math.sin(roll), -math.cos(pitch) * math.cos(roll)])


class Plant(object):
    """Kinematik bitki. Olay bayraklari: fall_at (adim; yan yatar, q HOME'dan uzak), stand_fail (sayi: o kadar K
    denemesi oturus egiminde kalir: T1 dolar, dusme yok), sit_fail (O'da oturus pozundan 25 derece sapar: T2 dolar, dusme
    yok), getup_after (getup politikasindan bu kadar adim sonra HOME'da ayakta; None: hic), bias (3), noise
    (np.random.Generator: sensors.IMU beyaz gurultusu, kontrol adimi basina bir ornek). Politika yokken (B) son durus
    korunur."""

    def __init__(self, seated=False, fall_at=None, stand_fail=0, sit_fail=False, getup_after=30, bias=(0, 0, 0),
                 noise=None, qd=0.0):
        self.alpha, self.h = (1.0 if seated else 0.0), C.H_HOME
        self.cmd = np.array([0.0, 0.0, 0.0, C.H_HOME, 1.0 if seated else 0.0])
        self.fall_at, self.stand_fail, self.sit_fail, self.getup_after = fall_at, int(stand_fail), sit_fail, getup_after
        self.bias, self.noise, self.qd = np.asarray(bias, float), noise, float(qd)
        self.mode, self.policy, self.k, self.n_getup = "ok", "main", 0, 0
        self.state = "M" if seated else "A"
        self.g_last = None

    def inputs(self, t, ids):
        q = np.zeros(14)
        if self.fall_at is not None and self.k >= self.fall_at and self.mode == "ok":
            self.mode, self.fall_at = "fallen", None
        if self.mode == "fallen":
            g, q = np.array([0.0, 1.0, 0.0]), q + 0.6
        elif self.policy == "getup" or (self.policy == "none" and self.g_last is None):
            g = g_of(0.0)
        elif self.policy == "none":
            g = self.g_last
        else:
            p = float(T.pitch_ref(np, np.array([self.h]), np.array([self.alpha]))[0])
            seat = math.radians(C.SEAT_PITCH_DEG)
            if self.stand_fail > 0 and self.state == "K":       # oturusla uyumlu kalir; gz >= stand_gz (T1 dolar)
                p = seat if abs(seat) >= math.radians(30.0) else seat + math.radians(30.0)
            if self.sit_fail and self.state == "O":             # oturustan 25 derece sapar: T2 dolar, dusme yok
                p = seat + math.radians(25.0)
            g = g_of(p)
        self.g_last = g
        w = np.array([0.0, 0.0, self.cmd[2] if (self.policy == "main" and self.mode == "ok") else 0.0]) + self.bias
        if self.noise is not None:
            w = w + SENS.IMU["gyro_sigma"] * SENS.SQ3 * (2.0 * self.noise.random(3) - 1.0)
        qd = np.full(14, self.qd)
        a, h = (self.alpha, self.h) if self.policy == "main" else (0.0, C.H_HOME)
        return CM.Inputs(t, [wid(x) for x in ids], g, w, qd, q, a, h)

    def apply(self, out):
        self.k += 1
        if self.state == "K" and out["state"] not in ("K", "A") and self.stand_fail > 0:
            self.stand_fail -= 1                         # basarisiz kalkis denemesi sayildi
        self.state = out["state"]
        pol = out["policy"]
        if out["handover"] or (pol == "main" and self.policy != "main"):
            self.mode = "ok"
            self.alpha, self.h = float(out["main_alpha"]), C.H_HOME
        if pol == "getup":
            self.n_getup = 0 if (self.policy != "getup" or out["getup_restart"]) else self.n_getup + 1
            if self.getup_after is not None and self.n_getup >= self.getup_after:
                self.mode = "standing"
        self.policy = pol
        if pol == "main" and out["command"] is not None:
            self.cmd = out["command"].astype(np.float64)
            h, a = T.posture_step(np, np.array([self.h]), np.array([self.alpha]), self.cmd[None])
            self.h, self.alpha = float(h[0]), float(a[0])


def run(sup, pl, n, script=None, t0=0.0, rows=None):
    """n adim; script {adim: [kelime | kimlik | 'wake']} (adim sup.n'e gore). Dondurur ciktilar listesi; rows verilirse
    commands.trace_row satirlari eklenir."""
    script = script or {}
    outs = []
    for _ in range(n):
        k = sup.n
        inp = pl.inputs(t0 + k * C.CTRL_DT, script.get(k, []))
        out = sup.step(inp)
        pl.apply(out)
        outs.append(out)
        if rows is not None:
            rows.append(CM.trace_row(inp, out))
    return outs


def to_state(state, **plant_kw):
    """Denetleyici ve bitkiyi verilen K5 durumuna mesru kelimelerle getirir: A, OE (gel -> otur), Y (gel), D (saga),
    O (otur), M (oturmus baslangic), K (M -> kalk), F (dusme), B (F -> dur). Dondurur (sup, bitki)."""
    seated = state in ("M", "K")
    pl = Plant(seated=seated, **plant_kw)
    sup = CM.Supervisor(table(), home_q(), "M" if seated else "A")
    run(sup, pl, 3)
    plan = {"A": [], "Y": ["gel"], "D": ["saga"], "O": ["otur"], "OE": ["gel", "otur"], "M": [], "K": ["kalk"],
            "F": [], "B": ["dur"]}[state]
    if state in ("F", "B"):
        pl.fall_at = pl.k
        run(sup, pl, CM.K["fall_steps"] + 1)
    for w in plan:
        run(sup, pl, 1, {sup.n: [w]})
        run(sup, pl, 2)
    assert sup.state == state, (state, sup.state)
    return sup, pl


# --- sentetik altin iz senaryolari (tools/commands_golden.py): ad -> (baslangic, adim, betik, bitki ayarlari) ----------
def _s1_script():
    """S1 dizisi kinematik bitkide (uyanma + kelimeler sabit adimlarda; K5 tamamlanmalarina yetecek araliklarla)."""
    s, k = {}, 20
    for w, gap in (("wake", 30), ("kalk", 230), ("gel", 400), ("wake", 30), ("dur", 60), ("saga", 300),
                   ("wake", 30), ("sola", 300), ("geri", 420), ("wake", 30), ("egil", 110), ("yuksel", 110),
                   ("evet", 100), ("hayir", 120), ("wake", 30), ("vak", 80), ("otur", 260)):
        s.setdefault(k, []).append(w)
        k += gap
    return s, k


SCENARIOS = {
    "s1_kinematik": ("M", None, None, {}),
    "kuyruk": ("M", 1500, {10: ["kalk"], 40: ["gel"], 60: ["otur"], 70: ["saga"], 300: ["otur"], 500: ["dur"],
                          520: ["otur"], 560: ["gel"], 600: ["kalk"], 900: ["egil"], 950: ["evet"], 1300: ["kalk"]}, {}),
    "kalkis_basarisiz": ("M", 1600, {10: ["kalk"], 30: ["gel"], 900: ["dur"], 1000: ["kalk"]}, {"stand_fail": 2}),
    "oturus_basarisiz": ("A", 600, {10: ["otur"], 400: ["vak"], 450: ["kalk"]}, {"sit_fail": True}),
    "dusme_getup_devir": ("A", 900, {10: ["gel"], 100: ["evet"], 120: ["wake"], 125: ["hayir"], 160: ["gel"],
                                     175: ["evet"], 180: ["saga"], 205: ["gel"], 210: ["evet"], 215: ["dur"],
                                     220: ["saga"], 225: ["egil"], 400: ["dur"], 420: ["vak"]},
                          {"fall_at": 150, "getup_after": 10}),
    "getup_basarisiz": ("A", 1500, {700: ["gel"], 1300: ["dur"], 1350: ["kalk"], 1400: ["otur"]},
                        {"fall_at": 20, "getup_after": None}),
    "kimlik_led_jest": ("A", 800, {5: ["wake"], 10: [99], 15: [203], 20: ["evet"], 30: ["hayir"], 35: ["egil"],
                                   40: [17, "vak"], 100: ["yuksel"], 200: ["gel"], 230: ["egil"], 260: ["otur"],
                                   262: ["evet"], 330: ["vak"], 400: ["kalk"], 600: [1, 2, 207]}, {}),
    "bias": ("A", 600, {300: ["sola"]}, {"bias": (0.01, -0.02, 0.04), "noise": "seed:7"}),
}


def scenario(name, rows=None):
    """Senaryoyu calistirir. Dondurur (sup, ciktilar)."""
    start, n, script, kw = SCENARIOS[name]
    if name == "s1_kinematik":
        script, n = _s1_script()
        n += 100
    kw = dict(kw)
    if isinstance(kw.get("noise"), str):
        kw["noise"] = np.random.default_rng(int(kw["noise"].split(":")[1]))
    pl = Plant(seated=start == "M", **kw)
    sup = CM.Supervisor(table(), home_q(), start)
    return sup, run(sup, pl, n, script, rows=rows)
