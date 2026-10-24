# -*- coding: utf-8 -*-
"""Ses eki (G-SES-a) testleri: sahte 'voice' firmware'iyle logger oturumlari, voice_report kapi buyuklukleri,
kimlik tablosu, gecikme (sentetik WAV), Poisson/Wilson, DF2301Q kare cozucusu (masaustunde derlenir).

  python -m pytest tools/bench/tests -q -p no:cacheprovider
"""
import json
import math
import queue
import shutil
import subprocess
import sys
import threading
import time
import wave
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.bench import logger  # noqa: E402
from tools.bench import voice_report as vr  # noqa: E402

IDS = {w: 5 + k for k, w in enumerate(vr.ANAHTAR)}          # bos modulde ogrenme sirasi
WORD = {v: k for k, v in IDS.items()}
HDR = "H,t_us,seg,ldr_mv,led,learn,i_ma,vbus_mv,mg_us,mg2_us,sg_us,ds_us"


# ---------------------------------------------------------------- sahte firmware ('voice' programi)
class FakeVoice:
    """'run voice' ve 'v ...' komutlarina firmware gibi yanit verir; D satirlari cihaz saatini ilerletir;
    sahte konusmaci (speaker) her pencere acilisinda [(gecikme s, kimlik)] dondurur."""

    def __init__(self, speaker, dt=0.25):
        self.speaker, self.dt0 = speaker, dt
        self.out = queue.Queue()
        self.lock = threading.Lock()
        self.t = 1_000_000
        self.dt = int(dt * 1e6)
        self.run = False
        self.pending = []
        self.wake_s = 15
        self.awake_until = 0
        self.refresh = 0.0
        self.t_ref = 0
        self.tick = 0
        self.writes = []
        self.test = ""
        self.link = None

    def awake(self, t=None):
        return (self.t if t is None else t) < self.awake_until

    def _e(self, name, kv="", t=None):
        self.out.put("E,%d,%s%s" % (self.t if t is None else t, name, ("," + kv) if kv else ""))

    def write(self, data):
        with self.lock:
            for ln in data.decode().splitlines():
                self._cmd(ln.strip())

    def _cmd(self, ln):
        self.writes.append(ln)
        if ln == "hb":
            return
        if ln == "cfg":
            self.out.put("CFG,cls=ds,pin_ldr=6,pca2=1,fw=bench-2")
        elif ln.startswith("set"):
            self.out.put("OK " + ln)
        elif ln.startswith("run voice"):
            kv = dict(p.split("=", 1) for p in ln.split()[2:])
            self.test = kv.get("tag", "")
            self.dt = int((1.0 if self.test == "ses-d3" else self.dt0) * 1e6)
            self.wake_s = int(kv.get("wake_s", 15))
            self.refresh = 0.0
            self.out.put("CFG,cls=ds,pin_ldr=6,pca2=1,fw=bench-2")
            self.out.put("S,voice,bench-2")
            self._e("params", ",".join("%s=%s" % x for x in kv.items()))
            self.out.put(HDR)
            self.run = True
        elif ln.startswith("v "):
            p = ln[2:].split()
            c, args = p[0], p[1:]
            if c == "win":
                op, kv = args[0], dict(a.split("=", 1) for a in args[1:])
                if op == "open":
                    self.tick += 1
                    self._e("tick", "n=%d,hz=4000,ms=40" % self.tick)
                rest = ",".join("%s=%s" % x for x in kv.items())
                self._e("win", "op=%s,tick=%d,%s" % (op, self.tick if op == "open" else 0, rest))
                if op == "open":
                    for d, i in self.speaker(kv, self):
                        self.pending.append((self.t + int(d * 1e6), i))
            elif c == "mark":
                self._e("mark", "ad=%s%s" % (args[0], "".join("," + a for a in args[1:])))
            elif c == "end":
                self.out.put("END,voice,ok")
                self.run = False
            else:
                if c == "wake":
                    self.wake_s = int(args[0])
                    self._e("vtx", "ad=wake,val=%d,seq=0,hex=0xF4F5" % self.wake_s)
                if c == "refresh":
                    self.refresh, self.t_ref = float(args[0]), self.t
                self._e("vcmd", "k=%s,v=%s" % (c, ",".join(args)))
        elif ln in ("stop", "!"):
            self.out.put("END,voice,abort:kullanici")
            self.run = False

    def readline(self):
        try:
            return (self.out.get_nowait() + "\n").encode()
        except queue.Empty:
            pass
        if self.link is not None and self.link.q.qsize() > 8:   # kaydedici geride: cihaz saati onden kacmasin
            time.sleep(0.0005)
            return b""
        with self.lock:
            if not self.run:
                time.sleep(0.002)
                return b""
            self.t += self.dt
            due = sorted(x for x in self.pending if x[0] <= self.t)
            self.pending = [x for x in self.pending if x[0] > self.t]
            for tp, i in due:
                self._e("asr", "id=%d,cmd=0x91,seq=0,len=1,ogr=0,yeni=0,hex=0xF4F5" % i, tp)
                if i in vr.WAKE:
                    self.awake_until = max(self.awake_until, tp + self.wake_s * 1_000_000)
            if self.refresh > 0 and self.t - self.t_ref >= self.refresh * 1e6:     # firmware: setWakeTime yenileme
                self.t_ref = self.t
                self._e("vtx", "ad=yenile,val=%d,seq=0,hex=0xF4F5" % self.wake_s)
                if self.awake():
                    self.awake_until = self.t + self.wake_s * 1_000_000
            a = self.awake()
            self.out.put("D,%d,0,%.1f,%d,0,nan,nan,nan,nan,nan,nan" % (self.t, 800.0 if a else 100.0, int(a)))
        time.sleep(0.0002)
        return (self.out.get() + "\n").encode()


class Speaker:
    """Sahte kullanici: d1'de ilk 'geri' 'gel' sanilir, ilk 'vak' kacar; digerleri dogru."""

    def __init__(self):
        self.n = {}

    def __call__(self, kv, f):
        s, w, form = kv.get("set"), kv.get("word"), kv.get("form")
        self.n[(s, w)] = self.n.get((s, w), 0) + 1
        first = self.n[(s, w)] == 1
        if s in ("d1", "d2", "dogrula"):
            if s == "d1" and w == "vak" and first:
                return [(0.8, 1)]
            i = IDS["gel"] if (s == "d1" and w == "geri" and first) else IDS[w]
            return [(0.8, 1), (2.6, i)]
        if s == "d2a":
            return [(1.2, 1), (1.6, IDS["gel"])]
        if s == "uyan":
            return [(1.0, 1)]
        if s == "canli":
            return [(1.5, IDS["gel"])] if f.awake() else []
        if s == "d4":
            if form in ("a_uyan", "b_uyan", "p_uyan"):
                return [(1.0, 1)]
            return [(0.6, IDS[w])] if f.awake() else []
        return []


def _ask(text):
    t = text.lower()
    if "tx" in t:
        return "3.31"
    if "kutle" in t:
        return "12.4"
    if "revizyon" in t:
        return "DF2301QG V1.1"
    if "akim" in t:
        return "45"
    if "duyuldu" in t and "yanit" in t and "ordek" in t:
        return "h"
    if "duyuldu" in t:
        return "e"
    return ""


def test_voice_sessions_end_to_end(tmp_path):
    fake = FakeVoice(Speaker())
    link = logger.Link(fake, hb=False)
    fake.link = link
    quiet = dict(echo=lambda *a, **k: None)
    tab_path = tmp_path / "komut_kimlik.json"
    try:
        out = tmp_path / "logs"
        p = logger.run_voice(link, "ses-ogren", "DF2301Q-1", out, {}, {"trial_s": 6, "grace_s": 0.5, "gap_s": 0.3},
                             ask=_ask, table=None, **quiet)
        assert p[0].endswith("_ses-ogren_voice.json")
        assert vr.main(["tablo", str(out / "DF2301Q-1"), "--tablo", str(tab_path)]) == 0
        tab = vr.load_table(tab_path)
        assert tab["ids"] == {v: k for k, v in IDS.items()} and tab["wake"] == (1, 2)
        # sema rl/commands.load_id_table ile ortak (spec eki K4): denetleyici bu dosyayi okur
        doc = json.loads(tab_path.read_text())
        assert doc["schema"] == 1 and doc["ornek"] is False and doc["uyanma"] == [1, 2]
        commands = pytest.importorskip("rl.commands")
        it = commands.load_id_table(str(tab_path))
        assert it.wake == {1, 2} and not it.example and not it.missing()
        assert {i: w for i, w in it.cmd.items()} == {v: k for k, v in IDS.items()}
        o = {"trial_s": 6, "grace_s": 0.5, "gap_s": 0.3, "uyku_dk": 2}
        for t in ("ses-olcu", "ses-d1", "ses-d2", "ses-d3", "ses-d4"):          # kilavuzdaki sira
            logger.run_voice(link, t, "DF2301Q-1", out, {}, dict(o), ask=_ask, table=tab, **quiet)
    finally:
        link.close()
    assert any(w.startswith("v win open i=0 word=") for w in fake.writes) and "v noise all" in fake.writes
    assert "v learn 1" in fake.writes and "v learn 0" in fake.writes and "v refresh 200" in fake.writes
    gec = tmp_path / "gecikme.json"
    R = vr.analyze(out / "DF2301Q-1", tab, gecikme_out=gec)
    kw = R["d1"]["kelime"]
    assert R["d1"]["n"] == 240 and kw["geri"]["k"] == 19 and kw["vak"]["k"] == 19 and kw["dur"]["k"] == 20
    assert all(v["ok"] for v in kw.values())
    assert R["d2"]["n"] == 120 and R["havuz"]["k"] == 358 and R["havuz"]["n"] == 360
    assert R["karisiklik"]["geri"]["gel"] == 1 and R["karisiklik"]["vak"]["yok"] == 1
    assert R["ciftler"]["gel/geri"] == 1
    assert R["zincir"]["n"] == 360 and R["zincir"]["k"] == 358 and R["zincir"]["araliksiz"]["k"] == 10
    d3 = R["d3"]
    assert d3["sonuc"] == "gecti", d3
    assert d3["maruziyet_dk"] >= 30.0 and d3["k"] == 0 and d3["ust_10dk"] <= 1.0
    assert d3["gosterge"]["tur"] == "ldr" and d3["kontrol_gecen"] == d3["kontrol_n"] >= 6
    assert d3["uyku"]["yanlis_uyanma"] == 0 and d3["uyku"]["dk"] > 1.5
    d4 = R["d4"]
    assert d4["yenileme"] == "gecerli komut pencereyi yenilemiyor"
    assert d4["kalicilik"]["mute"].startswith("kaldi") and d4["kalicilik"]["wake"].startswith("kaldi")
    assert d4["led_uyanikligi_izliyor"] is True and d4["istem_duyuldu"] == "e"
    ol = R["olcu"]
    assert ol["yanit"]["tx_bosta_v"] == 3.31 and ol["seviye"] == "yalniz 1 kOhm seri"
    assert ol["yanit"]["revizyon"] == "DF2301QG_V1.1"
    g = {o["olcut"]: o["sonuc"] for o in R["kapi"]["satirlar"]}
    assert R["kapi"]["genel"] == "GECTI", g
    assert R["gecikme"]["ozet"]["n"] == 0 and not gec.exists()          # ses kaydi yok
    md = vr.markdown(R)
    assert "**Kapi: GECTI.**" in md and "| geri | 19 | 20 | 18 | gecti |" in md
    # rapor CLI
    assert vr.main(["rapor", str(out / "DF2301Q-1"), "--tablo", str(tab_path), "--gecikme", str(gec)]) == 0
    assert (out / "DF2301Q-1" / "ses_rapor.md").exists()


# ---------------------------------------------------------------- sentetik log: uyanik test sinirlari
def _ses_d3(minutes, false_at=(), ldr=True, live_fail=()):
    """Dogrudan olaylardan bir ses-d3 oturumu: t = 0'da uyanma, 5 dk'da bir canlilik."""
    ev = [(0.5, "win", dict(op="open", i=0, set="uyan", word="ordek")), (1.0, "asr", dict(id=1)),
          (1.5, "win", dict(op="close", i=0, set="uyan"))]
    k, t = 0, 300.0
    while t <= minutes * 60 + 1e-6:
        ev.append((t, "win", dict(op="open", i=k, set="canli", word="gel")))
        if k not in live_fail:
            ev.append((t + 1.2, "asr", dict(id=IDS["gel"])))
        ev.append((t + 3.0, "win", dict(op="close", i=k, set="canli")))
        k, t = k + 1, t + 300.0
    for tf in false_at:
        ev.append((tf, "asr", dict(id=IDS["otur"])))
    tt = np.arange(0.0, minutes * 60 + 60, 0.5)
    cols = dict(t_us=tt * 1e6, ldr_mv=np.where(tt >= 1.0, 800.0, 100.0) if ldr else np.full(len(tt), np.nan))
    return vr.Ses("ses-d3", ev, cols, dict(wake_s=255))


TAB = dict(ids={v: k for k, v in IDS.items()}, wake=(1, 2))


def test_awake_gate_cases():
    r = vr.uyanik(_ses_d3(35), TAB)
    assert r["sonuc"] == "gecti" and r["k"] == 0 and 33.0 < r["maruziyet_dk"] < 35.0
    r = vr.uyanik(_ses_d3(20), TAB)
    assert r["sonuc"] == "olculemedi" and r["maruziyet_dk"] < 30
    r = vr.uyanik(_ses_d3(35, false_at=[700.0]), TAB)
    assert r["sonuc"] == "kaldi" and r["k"] == 1 and r["yanlis"][0]["komut"] == "otur"
    r = vr.uyanik(_ses_d3(35, ldr=False), TAB)                       # gosterge yok -> dogrulanamaz
    assert r["sonuc"] == "olculemedi" and r["dogrulanmis_dk"] == 0
    r = vr.uyanik(_ses_d3(35, live_fail=(2,)), TAB)                  # 15. dk kontrolu kaldi: 10-20 dk dusulur
    assert 23.0 < r["maruziyet_dk"] < 25.5 and r["sonuc"] == "olculemedi"
    r = vr.uyanik(_ses_d3(35, false_at=[1000.0]), TAB, gecerli=False)  # LED uyanikligi izlemiyor (d4)
    assert r["sonuc"] == "olculemedi"


def test_notify_indicator_preferred():
    s = _ses_d3(35, ldr=False)
    s.ev.append((1.05, "notify", dict(code="0xB1", ad="uyanma_bas")))
    s.ev.sort(key=lambda e: e[0])
    r = vr.uyanik(s, TAB)
    assert r["gosterge"]["tur"] == "notify" and r["sonuc"] == "gecti"


def test_stats():
    assert abs(vr.poisson_upper(0) - 2.9957) < 1e-3 and abs(vr.poisson_upper(1) - 4.7439) < 1e-3
    assert 3.0 / 30.0 * 10 < 1.0 + 1e-3                              # 0 / 30 dk -> 1 / 10 dk sinirda
    lo, hi = vr.wilson(216, 240)
    assert abs(lo - 0.8558) < 2e-3 and abs(hi - 0.9327) < 2e-3
    lo, _ = vr.wilson(0, 0)
    assert math.isnan(lo)
    ws = vr.word_stats([dict(word="dur", ok=i < 18) for i in range(20)] +
                       [dict(word="gel", ok=i < 18) for i in range(20)], n_req=20)
    assert ws["gel"]["ok"] and not ws["dur"]["ok"] and ws["dur"]["gerekli"] == 19 and ws["egil"]["eksik"]
    assert vr.iv_sub([(0, 10)], [(2, 3), (5, 6)]) == [(0, 2), (3, 5), (6, 10)]
    assert vr.anahtar("Sağa dön") == "saga" and vr.anahtar("hayır") == "hayir"


def test_table_learning_order_and_warnings():
    ev = []
    for n, w in enumerate(vr.ANAHTAR):
        ev.append((n * 10.0, "mark", dict(ad="ogren", word=w, sira=n)))
    t = 500.0
    for n, w in enumerate(vr.ANAHTAR):
        ev += [(t, "win", dict(op="open", i=n, set="dogrula", word=w)), (t + 1, "asr", dict(id=1)),
               (t + 2, "asr", dict(id=IDS[w] if w != "vak" else 203)),
               (t + 4, "win", dict(op="close", i=n, set="dogrula"))]
        t += 10
    ids, warn = vr.build_table(vr.Ses("ses-ogren", ev, dict(t_us=np.zeros(0))))
    assert ids[5] == "gel" and ids[16 - 1] == "hayir" and 203 not in ids
    assert any("vak" in w for w in warn)


# ---------------------------------------------------------------- gecikme: sentetik WAV (tik + konusma)
def _wav(path, sr, events, dur):
    rng = np.random.default_rng(3)
    x = 0.002 * rng.standard_normal(int(dur * sr))
    for kind, t0, t1 in events:
        i0, i1 = int(t0 * sr), int(t1 * sr)
        tt = np.arange(i1 - i0) / sr
        if kind == "tick":
            x[i0:i1] += 0.3 * np.sin(2 * np.pi * 4000 * tt)
        else:                                            # konusma benzeri: 300-3000 Hz bant gurultusu
            n = rng.standard_normal(i1 - i0)
            F = np.fft.rfft(n)
            f = np.fft.rfftfreq(len(n), 1 / sr)
            F[(f < 300) | (f > 3000)] = 0
            x[i0:i1] += 0.2 * np.fft.irfft(F, len(n)) / (np.std(np.fft.irfft(F, len(n))) + 1e-9)
    y = np.clip(x * 32767, -32768, 32767).astype("<i2")
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(y.tobytes())


def test_latency_from_wav(tmp_path):
    off, sr = 7.3, 16000                     # ses kaydi log'dan 7.3 s once basladi
    lat_true = [0.42, 0.65, 0.9, 1.21, 0.55, 0.77]
    ev, aud, t = [], [], 20.0
    for k, L in enumerate(lat_true):
        w = vr.ANAHTAR[k]
        ev += [(t, "tick", dict(n=k + 1)), (t, "win", dict(op="open", i=k, set="d2", word=w, tick=k + 1))]
        a = t + off
        aud += [("tick", a, a + 0.04), ("sp", a + 0.9, a + 1.4), ("sp", a + 2.4, a + 2.9)]
        we = t + 2.9
        ev += [(t + 1.6, "asr", dict(id=1)), (we + L, "asr", dict(id=IDS[w])),
               (we + L + 1.5, "win", dict(op="close", i=k, set="d2"))]
        t += 9.0
    ev.append((t, "tick", dict(n=99)))                  # pencere disi fazladan tik da hizalamaya girer
    aud.append(("tick", t + off, t + off + 0.04))
    wav = tmp_path / "ses-d2.wav"
    _wav(wav, sr, aud, t + off + 3)
    s = vr.Ses("ses-d2", sorted(ev, key=lambda e: e[0]), dict(t_us=np.zeros(0)), dict(tick_hz=4000))
    trs = vr.trials(s, TAB)
    assert all(r["ok"] for r in trs) and len(trs) == 6
    lat, info = vr.latency_wav(s, trs, wav)
    assert info["eslesen"] == 7 and abs(info["kayma_s"] - off) < 0.01, info
    got = [r["s"] for r in lat]
    assert len(got) == 6 and max(abs(a - b) for a, b in zip(got, lat_true)) < 0.03, got
    d = vr.gecikme_json(lat, tmp_path / "g.json", [str(wav)])
    j = json.loads((tmp_path / "g.json").read_text())
    assert j["version"] == 1 and j["n"] == 6 and len(j["uniform_s"]) == 2 and j["kosul"] == {"d2": 6}
    assert d["ozet"]["p50"] == pytest.approx(np.median(got))


# ---------------------------------------------------------------- DF2301Q kare cozucu (C++, masaustu derleme)
def lib_frame(type_, cmd, seq, data):
    """DFRobot_DF2301Q.cpp sendPacket: F4 F5 len(LE) tip komut sira veri toplam(LE) FB; toplam = tip..veri."""
    body = bytes([type_, cmd, seq]) + bytes(data)
    s = sum(body) & 0xFFFF
    return bytes([0xF4, 0xF5, len(data) & 0xFF, len(data) >> 8]) + body + bytes([s & 0xFF, s >> 8, 0xFB])


HARNESS = r"""
#include <cstdio>
#include <cstdlib>
#include "df2301q.h"
int main(int argc, char** argv) {
  uint8_t b[64];
  int n = df::buildSet(0, df::S_MUTE, 1, b); for (int i = 0; i < n; i++) printf("%02X", b[i]); printf("\n");
  n = df::buildSet(3, df::S_WAKETIME, 255, b); for (int i = 0; i < n; i++) printf("%02X", b[i]); printf("\n");
  n = df::buildPlay(7, 23, b); for (int i = 0; i < n; i++) printf("%02X", b[i]); printf("\n");
  df::Parser p;
  const char* h = argv[1];
  for (int i = 0; h[i] && h[i + 1]; i += 2) {
    char t[3] = {h[i], h[i + 1], 0};
    int r = p.feed((uint8_t)strtoul(t, 0, 16));
    if (r == 1) printf("F %02X %02X %d %d\n", p.type, p.cmd, p.len, p.len ? p.d[0] : -1);
    if (r == -1) printf("B %c %d\n", p.err, p.nraw);
    if (r == 2) printf("J\n");
  }
  return 0;
}
"""


def test_df2301q_cpp_parser(tmp_path):
    cxx = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if not cxx:
        pytest.skip("C++ derleyici yok")
    (tmp_path / "h.cpp").write_text(HARNESS)
    exe = tmp_path / "h"
    inc = ROOT / "tools" / "bench" / "bench"
    subprocess.run([cxx, "-std=c++11", "-Wall", "-Werror", "-I", str(inc), str(tmp_path / "h.cpp"), "-o", str(exe)],
                   check=True, capture_output=True)
    asr = lib_frame(0xA0, 0x91, 4, [5])
    nfy = lib_frame(0xA3, 0x9A, 1, [0xB1])
    bad = bytearray(lib_frame(0xA0, 0x91, 5, [6]))
    bad[-2] ^= 0xFF                                      # bozuk toplam
    long_ = bytes([0xF4, 0xF5, 0x40, 0x00])              # uzunluk 64 > 32
    stream = b"\x00\x12" + asr + bytes(bad) + b"\xF4" + nfy + long_ + asr
    out = subprocess.run([str(exe), stream.hex().upper()], check=True, capture_output=True,
                         text=True).stdout.split("\n")
    assert out[0] == lib_frame(0xA1, 0x96, 0, [0x83, 1, 0, 0, 0]).hex().upper()
    assert out[1] == lib_frame(0xA1, 0x96, 3, [0x84, 255, 0, 0, 0]).hex().upper()
    assert out[2] == lib_frame(0xA1, 0x92, 7, [0x80, 0x92, 23, 0, 0, 0]).hex().upper()
    ev = [ln for ln in out[3:] if ln]
    assert ev[:2] == ["J", "J"] and ev[2] == "F A0 91 1 5"
    assert "B k 10" in ev and "F A3 9A 1 177" in ev and "B l 4" in ev and ev[-1] == "F A0 91 1 5"


def test_shared_files_with_controller(tmp_path):
    """configs/komut_kimlik.json ve configs/ses_gecikme.json: G-SES-a yazar, denetleyici ve A10 okur (spec eki K4, K7,
    K8). Depodaki P5k ornek tablosu ("ornek": true) olcum yerine kullanilmaz."""
    assert vr.load_table(vr.TABLO) is None
    commands = pytest.importorskip("rl.commands")
    assert commands.load_id_table(str(vr.TABLO)).example
    ev = pytest.importorskip("rl.eval_tasks")
    gec = tmp_path / "ses_gecikme.json"
    d = vr.gecikme_json([dict(set="ses-d1", i=i, word="gel", s=0.4 + 0.01 * i) for i in range(41)], gec, ["d1.wav"])
    lat, src = ev.a10_latency(str(gec))
    assert lat == tuple(d["uniform_s"]) and src.startswith("G-SES-a")
    assert ev.a10_latency(str(tmp_path / "yok.json")) == (tuple(ev.A10["lat"]), "varsayilan (spec eki K7)")
