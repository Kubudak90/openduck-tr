# -*- coding: utf-8 -*-
"""
Adim 0 PC kaydedici: tezgah betigine (tools/bench/bench) adli bir testi kosturur, satirlari kaydeder.

  pip install pyserial
  python tools/bench/logger.py --list
  python tools/bench/logger.py --port /dev/cu.usbserial-0001 --servo DS3230-1 --model DS3230 --cls ds --test sweep
  python tools/bench/logger.py --port COM5 --servo DS3230-1 --model DS3230 --cls ds --plan ds
  python tools/bench/logger.py --port COM5 --servo DS3230-1 --cls ds --test curpts --set mass_g=1500 lever_mm=100
  python tools/bench/logger.py --port COM5 --cmd "scan"
  python tools/bench/logger.py --pack bench_logs          # gondermek icin zip
  python tools/bench/logger.py --port COM5 --servo DF2301Q-1 --plan ses     # G-SES-a (ek K8), kilavuz bolum 10
  python tools/bench/logger.py --port COM5 --servo DF2301Q-1 --test ses-d1 --arg n=20 desen=all

Her kosu: <out>/<servo>/<zaman>_<test>_<program>[_<tag>].csv + .events.csv + .json (+ .burst.csv, .raw.txt).
Bicim: tools/sysid/logio.py. Istemler (P): jog icin '+10' / '-5' yazin, bitince 'ok'; diger sorulara deger yazin.
'!' + Enter: E-stop. Ctrl-C: E-stop + kaydi kapatir. Kalp atisi (hb) 1 s'de bir; kesilirse betik 5 s'de durur.
Ses oturumlari (ses-*): firmware 'voice' programi kayit tutar, bu betik protokolu yurutur ('v ...' komutlari);
deneme sirasinda Ctrl-C duraklatir (tekrar / atla / bitir). Analiz: tools/bench/voice_report.py.
"""
import argparse
import json
import queue
import random
import subprocess
import sys
import threading
import time
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.sysid.logio import RunRecorder, _num  # noqa: E402
from tools.bench import voice_report as vr  # noqa: E402

# test adi -> [(firmware programi, parametreler)]; ayni test birden cok kosu olabilir
TESTS = {
    "sweep": [("sweep", {})],
    "potcal": [("potcal", {"points": "-60,-30,0,30,60"})],
    "potprobe": [("potprobe", {})],
    "deadband": [("deadband", {})],
    "speed": [("speed", {})],
    "step": [("step", {})],
    "chirp": [("chirp", {})],
    "ramp": [("ramp", {})],
    "stall": [("stall", {})],
    "curpts": [("curpts", {})],
    "accept300": [("track", {"freq": 50, "tag": "f50"}), ("track", {"freq": 300, "tag": "f300"})],
    "accept33": [("track", {"tag": "v33"})],
    "overshoot": [("overshoot", {"tag": "bos"}),
                  ("overshoot", {"tag": "atalet", "_ask": "Agir kolu (kutleli) takin, kolun serbestce donebildigini "
                                                       "kontrol edin; hazirsa Enter"})],
    "heat": [("heat", {})],
    "noise": [("noise", {"wifi": 0, "tag": "wifi0"}), ("noise", {"wifi": 1, "tag": "wifi1"})],
    "nopwm": [("nopwm", {})],
    "hold": [("hold", {})],
    "gest": [("gest", {"dur_s": 60})],       # ses eki: yalniz gurultu programi (servo baglantisi kontrolu)
}
CMD_TESTS = {"scan": "scan", "osc": "osc auto"}
PLANS = {
    "ds": ["scan", "osc", "sweep", "potcal", "deadband", "speed", "step", "chirp", "ramp", "stall", "curpts",
           "accept300", "accept33", "overshoot", "noise", "nopwm"],
    "es": ["scan", "osc", "sweep", "potcal", "deadband", "speed", "step", "chirp", "ramp", "stall", "curpts",
           "accept300", "accept33", "noise"],
    "mg": ["scan", "osc", "sweep", "potcal", "speed", "step", "stall", "curpts", "accept33"],
    "heat": ["heat"],
    "ses": ["gest", "ses-olcu", "ses-ogren", "ses-d1", "ses-d2", "ses-d3", "ses-d4"],
}
PLANS["sg"] = PLANS["mg"]
JOG_PROMPTS = ("touch", "horiz", "pc")


class Link:
    """Seri hat: okuma ipligi + kalp atisi; testte sahte port verilebilir (write(bytes), readline() -> bytes)."""

    def __init__(self, ser, hb=True):
        self.ser = ser
        self.q = queue.Queue()
        self.lock = threading.Lock()
        self.alive = True
        self.t = threading.Thread(target=self._rd, daemon=True)
        self.t.start()
        self.hb = None
        if hb:
            self.hb = threading.Thread(target=self._hb, daemon=True)
            self.hb.start()

    def _rd(self):
        while self.alive:
            try:
                ln = self.ser.readline()
            except Exception as e:  # port kapandi
                self.q.put(("#ERR", str(e)))
                break
            if ln:
                self.q.put(("L", ln.decode("utf-8", "replace").rstrip("\r\n")))

    def _hb(self):
        while self.alive:
            self.send("hb")
            time.sleep(1.0)

    def send(self, s):
        with self.lock:
            self.ser.write((s + "\n").encode())

    def get(self, timeout=0.2):
        try:
            return self.q.get(timeout=timeout)
        except queue.Empty:
            return None

    def close(self):
        self.alive = False


def open_serial(port, baud):
    import serial  # pyserial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, 0.1
    s.dtr = False      # DevKitC otomatik reset/boot devresi: portu acarken karti sifirlama
    s.rts = False
    s.open()
    return s


def ask_user(pid, text, answer_fn=None, first=True):
    """Istem yaniti: answer_fn (test) ya da klavye. Liste ya da tek satir."""
    if answer_fn:
        return answer_fn(pid, text)
    if first:
        print("\n>>> %s" % text)
    return input("[%s] %s> " % (pid, "+N/-N (us), bitince ok" if pid.startswith(JOG_PROMPTS) else "yanit"))


def _isnum(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def answer_prompt(link, pid, text, answer_fn=None):
    """Jog istemlerinde +N/-N satirlari dogrudan gider; ilk baska satir yanittir ('ans ...')."""
    jog = pid.startswith(JOG_PROMPTS)
    first = True
    while True:
        a = ask_user(pid, text, answer_fn, first)
        first = False
        for s in (a if isinstance(a, list) else [a]):
            s = (s or "").strip()
            if not s:
                continue
            if s == "!":
                link.send("!")
                return
            if s in ("stop", "x"):
                link.send(s)
                return
            if jog and s[0] in "+-" and _isnum(s):
                link.send(s)
                continue
            link.send("ans " + s)
            return


def wait_ready(link, timeout=3.0):
    t0 = time.time()
    lines = []
    while time.time() - t0 < timeout:
        m = link.get(0.1)
        if m and m[0] == "L":
            lines.append(m[1])
    return lines


def command(link, cmd, timeout=3.0):
    link.send(cmd)
    out, t0 = [], time.time()
    while time.time() - t0 < timeout:
        m = link.get(0.1)
        if m and m[0] == "L":
            out.append(m[1])
            if m[1].startswith(("OK", "ERR")) and not cmd.startswith(("scan", "osc")):
                break
            if cmd == "cfg" and m[1].startswith("CFG"):
                break
    return out


def run_program(link, prog, params, answer_fn=None, echo=print, max_s=6 * 3600, silent_s=30.0):
    """Bir firmware programini kostur, RunRecorder dondur. silent_s boyunca satir gelmezse (istem disinda) kesilir."""
    rec = RunRecorder()
    for ln in command(link, "cfg", 1.5):
        rec.feed(ln)
    args = " ".join("%s=%s" % (k, v) for k, v in params.items() if not k.startswith("_"))
    link.send(("run %s %s" % (prog, args)).strip())
    t0, nD, last = time.time(), 0, time.time()
    rx = time.time()
    try:
        while time.time() - t0 < max_s:
            m = link.get(0.2)
            if m is None:
                if time.time() - rx > silent_s:
                    echo("  betikten %d s satir yok: kesildi (kart resetlendi ya da kablo koptu?)" % silent_s)
                    link.send("!")
                    rec.status = "abort:sessiz"
                    break
                continue
            rx = time.time()
            if m[0] == "#ERR":
                echo("seri hata: " + m[1])
                rec.status = "abort:seri"
                break
            ln = m[1]
            tag = rec.feed(ln)
            if tag == "D":
                nD += 1
                if time.time() - last > 2.0:
                    last = time.time()
                    echo("  ... %d satir, %.0f s" % (nD, time.time() - t0))
            elif tag == "P":
                pid, text = rec.prompts[-1]
                answer_prompt(link, pid, text, answer_fn)
                rx = time.time()
            elif tag in ("W", "R", "S", "#"):
                echo("  " + ln)
            elif tag == "END":
                echo("  " + ln)
                break
            elif ln.startswith("ERR"):
                echo("  " + ln)
                rec.status = "abort:" + ln
                break
    except KeyboardInterrupt:
        link.send("!")
        echo("E-stop gonderildi (Ctrl-C)")
        rec.status = "abort:host"
    return rec


def save(rec, out, servo, test, prog, params, extra):
    ts = time.strftime("%Y%m%d-%H%M%S")
    tag = str(params.get("tag", "") or "")
    stem = Path(out) / servo / ("%s_%s_%s%s" % (ts, test, prog, ("_" + tag) if tag else ""))
    meta = dict(servo_id=servo, test=test, host_time=time.strftime("%Y-%m-%dT%H:%M:%S"), **extra)
    return rec.write(stem, meta)


def save_cmd(lines, out, servo, test, extra):
    p = Path(out) / servo / ("%s_%s.cmd.json" % (time.strftime("%Y%m%d-%H%M%S"), test))
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(dict(servo_id=servo, test=test, lines=lines, **extra), indent=1, ensure_ascii=False))
    return str(p)


# ================================================================ ses modulu oturumlari (G-SES-a, ek K8)
# Firmware 'voice' programi kareleri, tikleri ve pencereleri kendi saatiyle (us) kaydeder; bu betik protokolu
# yurutur. Bekleme cihaz saatine gore (D satirlari 50 Hz). Secenekler --arg k=v (VOICE_DEF; firmware
# parametreleri VOICE_FW dogrudan 'run voice'a gider).
VOICE_FW = ("wake_s", "refresh_s", "mg", "mg2", "sg", "ds", "pause_s", "ds_amp", "ds_f", "tick_hz", "tick_ms",
            "raw", "rate", "tmax")
VOICE_DEF = dict(n=0, seed=1, trial_s=12.0, grace_s=1.5, gap_s=1.0, kelimeler="", dk=30.0, max_dk=60.0,
                 kontrol_s=300.0, dogrula_dk=10.0, canli="gel", canli_wav="", ordek_wav="", ordek_s=14.0,
                 uyku_dk=5.0, tablo="", desen="all")


def play_wav(path):
    """Kayitli bir sesi PC hoparlorunden cal (engellemesiz). Ek bagimlilik yok: isletim sistemi oynaticisi."""
    p = str(path)
    if sys.platform == "darwin":
        cmd = ["afplay", p]
    elif sys.platform.startswith("win"):
        cmd = ["powershell", "-c", "(New-Object Media.SoundPlayer '%s').PlaySync()" % p]
    else:
        cmd = ["aplay", "-q", p]
    try:
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except OSError:
        return False


def _san(s):
    """Yanit E satirina girsin: bosluk -> _, virgul -> ;, = -> :"""
    return (str(s).strip() or "-").replace(" ", "_").replace(",", ";").replace("=", ":")


class Voice:
    """Bir ses oturumu: 'run voice', 'v ...' komutlari ve satirlarin RunRecorder'a kaydi."""

    def __init__(self, link, test, opts, echo=print, ask=None, table=None):
        self.link, self.test, self.o, self.echo = link, test, opts, echo
        self.ask = ask or input
        self.table = table
        self.rec = RunRecorder()
        self.ids = []                 # (t_us, id)
        self.now = 0                  # son cihaz zamani (us)
        self.ended = False
        self.closing = False
        self.params = {}

    # -- hat
    def fw(self, **kw):
        p = dict(kw, tag=self.test)
        p.update({k: v for k, v in self.o.items() if k in VOICE_FW})
        return p

    def start(self, params):
        self.params = params
        for ln in command(self.link, "cfg", 1.5):
            self.rec.feed(ln)
        self.link.send(("run voice " + " ".join("%s=%s" % kv for kv in params.items())).strip())
        if not self.pump(lambda: self.rec.program == "voice" and self.rec.header is not None, 5.0):
            raise RuntimeError("voice programi baslamadi (firmware bench-2 yuklu mu?)")

    def v(self, s):
        self.link.send("v " + s)

    def _line(self, ln):
        n0 = len(self.rec.events)
        tag = self.rec.feed(ln)
        if tag == "D":
            try:
                self.now = max(self.now, int(ln.split(",", 2)[1]))
            except (ValueError, IndexError):
                pass
        elif tag == "E" and len(self.rec.events) > n0:
            t, name, kv = self.rec.events[-1]
            self.now = max(self.now, t)
            if name == "asr":
                self.ids.append((t, int(kv.get("id", 0))))
                self.echo("    <- %s" % self.fmt(int(kv.get("id", 0))))
            elif name in ("notify", "ack", "vframe", "vbad", "learn"):
                self.echo("    " + ln)
        elif tag in ("W", "END") or ln.startswith("ERR"):
            self.echo("  " + ln)
        if tag == "END":
            self.ended = True

    def pump(self, until=None, timeout=0.0):
        """Satirlari kayda al; until() dogru olunca True, PC saatiyle timeout dolunca False."""
        t_end = time.time() + timeout
        while True:
            m = self.link.get(0.02)
            if m:
                if m[0] == "#ERR":
                    raise RuntimeError("seri hata: " + m[1])
                self._line(m[1])
            if until is not None and until():
                return True
            if self.ended:
                if self.closing:
                    return False
                raise RuntimeError("firmware programi bitti (%s)" % self.rec.status)
            if time.time() >= t_end:
                return False

    def wait_dev(self, s, tick=None):
        t1 = self.now + s * 1e6
        return self.wait_until(t1, tick)

    def wait_until(self, t1, tick=None):
        """Cihaz saati t1'e (us) gelene kadar; tick(now) her satirda cagrilir (ornek: B yontemi calmalari)."""
        def f():
            if tick:
                tick(self.now)
            return self.now >= t1
        return self.pump(f, (t1 - self.now) * 1e-6 + 30.0)

    def send_wait(self, cmd, name, pred, timeout=3.0):
        n0 = len(self.rec.events)
        self.v(cmd)
        hit = []

        def f():
            for t, nm, kv in self.rec.events[n0:]:
                if nm == name and pred(kv):
                    hit.append(t)
                    return True
            return False
        self.pump(f, timeout)
        return hit[0] if hit else self.now

    # -- protokol yardimcilari
    def fmt(self, i):
        w = vr.classify(i, self.table) if self.table else None
        return "%d (uyanma)" % i if i in vr.WAKE else ("%d %s" % (i, w) if w else str(i))

    def word(self, i):
        return vr.classify(i, self.table) if self.table else str(i)

    def step(self, text):
        self.echo("\n>>> " + text)
        return self.ask("[Enter] > ")

    def win(self, i, word, set_, form="led"):
        return self.send_wait("win open i=%s word=%s set=%s form=%s" % (i, word, set_, form), "win",
                              lambda kv: kv.get("op") == "open" and str(kv.get("i")) == str(i)
                              and str(kv.get("set")) == set_)

    def close(self, i, set_, op="close"):
        self.v("win %s i=%s set=%s" % (op, i, set_))

    def wait_cmd(self, t0, timeout, grace=None, tick=None):
        """t0'dan sonra ilk uyanma disi kimlik (+ grace s) ya da timeout; doner [(t, id)]."""
        grace = self.o["grace_s"] if grace is None else grace
        wk = set(self.table["wake"]) if self.table else set(vr.WAKE)
        got = lambda: [x for x in self.ids if x[0] >= t0 and x[1] not in wk]  # noqa: E731

        def f():
            if tick:
                tick(self.now)
            return bool(got()) or self.now - t0 >= timeout * 1e6
        self.pump(f, timeout + 30.0)
        c = got()
        if c:
            self.wait_until(c[0][0] + grace * 1e6, tick)
        return c

    def wait_wake(self, t0, timeout):
        wk = set(self.table["wake"]) if self.table else set(vr.WAKE)
        got = lambda: [t for t, i in self.ids if t >= t0 and i in wk]  # noqa: E731
        self.pump(lambda: bool(got()) or self.now - t0 >= timeout * 1e6, timeout + 30.0)
        return (got() or [None])[0]

    def block(self, words, n, set_, form, say):
        """n tur; her turda kelimeler karisik. Ctrl-C: duraklat (Enter tekrar, s atla, q bitir)."""
        rng = random.Random("%s-%s" % (self.o["seed"], set_))
        order = []
        for _ in range(n):
            w = list(words)
            rng.shuffle(w)
            order += w
        self.echo("  %d deneme. Ctrl-C: duraklat." % len(order))
        i = 0
        while i < len(order):
            w = order[i]
            try:
                self.echo("[%s %d/%d] %s" % (set_, i + 1, len(order), say(w)))
                t0 = self.win(i, w, set_, form)
                c = self.wait_cmd(t0, self.o["trial_s"])
                self.close(i, set_)
                got = self.word(c[0][1]) if c else None
                self.echo("    %s" % ("dogru" if got == w else ("yok" if not c else "YANLIS: " + str(got))))
                self.wait_dev(self.o["gap_s"])
                i += 1
            except KeyboardInterrupt:
                a = self.ask("\nDuraklatildi. Enter = bu denemeyi tekrarla, s = say ve gec, q = bitir > ").strip()
                if a.lower() == "s":
                    self.close(i, set_)
                    i += 1
                    continue
                self.close(i, set_, "void")
                if a.lower() == "q":
                    return False
        return True

    def live(self, k):
        """Canlilik: kayitli bilinen kelime calinir (canli_wav varsa PC calar), kimligi beklenir."""
        t0 = self.win(k, self.o["canli"], "canli", "kayit")
        if self.o["canli_wav"] and play_wav(self.o["canli_wav"]):
            self.echo("  canlilik %d: '%s' calindi" % (k, self.o["canli"]))
        else:
            self.echo("\a  CANLILIK %d: kayitli '%s' sesini simdi calin" % (k, self.o["canli"]))
        c = self.wait_cmd(t0, max(20.0, self.o["trial_s"]), 1.0)
        self.close(k, "canli")
        return bool(c)

    def wake_win(self, form, word="ordek"):
        t0 = self.win(form, word, "d4", form)
        self.echo("  'ordek' deyin")
        tw = self.wait_wake(t0, 15.0)
        self.close(form, "d4")
        return tw

    def cmd_win(self, form, word, t_at):
        self.wait_until(t_at)
        self.echo("  SIMDI: '%s'" % vr.SOYLE.get(word, word))
        t0 = self.win(form, word, "d4", form)
        c = self.wait_cmd(t0, 6.0, 0.5)
        self.close(form, "d4")
        self.echo("    %s" % ("taninda" if c else "taninmadi"))
        return c

    def mark(self, ad, **kv):
        self.v("mark %s %s" % (ad, " ".join("%s=%s" % (k, _san(v)) for k, v in kv.items())))

    def finish(self):
        self.closing = True
        if not self.ended:
            self.v("end")
            self.pump(lambda: self.ended, 5.0)
        if not self.ended:
            self.link.send("stop")
            self.pump(lambda: self.ended, 3.0)
        return self.rec


def _words(V):
    return [vr.anahtar(w) for w in V.o["kelimeler"].split(",")] if V.o["kelimeler"] else list(vr.ANAHTAR)


def _say_led(w):
    return "'ordek' -> LED -> '%s'" % vr.SOYLE.get(w, w)


def ses_olcu(V):
    V.start(V.fw(noise="0"))
    V.echo("Hazirlik olcumleri (ek K8). Yanitlari yazin; bilmiyorsaniz '-'.")
    for k, q in (("revizyon", "Kart yazisi / revizyon (ornek DF2301QG V1.1)"),
                 ("delik", "Montaj delikleri: sayi, eksenler arasi x*y (mm), cap (mm)"),
                 ("mik1_mm", "Mikrofon 1 merkezi: konnektor kenarindan x, uzun kenardan y (mm)"),
                 ("mik2_mm", "Mikrofon 2 merkezi (mm)"), ("port_yonu", "Mikrofon portu yonu (ust / alt / yan)"),
                 ("yukseklik_mm", "En yuksek parcayla toplam yukseklik (mm)"),
                 ("konnektor", "Konnektor yeri ve yonu"), ("fis_mm", "Fis takiliyken yukseklik (mm)"),
                 ("kutle_g", "Kutle, kablosuz (g)"),
                 ("tx_bosta_v", "TX (D/T) bosta gerilimi, ESP32'den ayrikken, 5 V beslemede (V)")):
        V.mark("olcu", id=k, val=V.ask("%s > " % q))
    V.step("Akim: multimetre (mA) ya da INA226 modulun 5 V dalinda. Hazirsa Enter.")
    for k, q in (("bosta", "Sessiz oda, modul uykuda"), ("dinlerken", "Uykuda, siz konusurken ('ordek' demeden)"),
                 ("uyanik", "'ordek' deyin; LED yanarken")):
        V.step("%s: Enter ile 5 s olcum" % q)
        V.mark("akim", id=k, op="bas")
        V.wait_dev(5.0)
        V.mark("akim", id=k, op="son")
        V.mark("olcu", id="akim_%s_ma" % k, val=V.ask("Multimetrede okunan akim (mA) > "))
    V.step("Acilis ani akimi: multimetre MAX/tepe modunda; Enter'dan sonra modul beslemesini cikarip takin")
    V.mark("akim", id="acilis", op="bas")
    V.mark("olcu", id="akim_acilis_ma", val=V.ask("Okunan tepe akim (mA) > "))
    V.mark("akim", id="acilis", op="son")
    V.step("RX 3.3 V ve hoparlor: modul bir yanit sesi calacak (ID 23). Enter")
    V.v("mute 0")
    V.v("vol 7")
    V.mark("akim", id="hoparlor", op="bas")
    V.v("play 23")
    V.wait_dev(3.0)
    V.mark("akim", id="hoparlor", op="son")
    V.mark("olcu", id="rx_33v", val=V.ask("Yanit sesi duyuldu mu (modul RX'i 3.3 V surusle calisiyor)? (e/h) > "))
    V.mark("olcu", id="akim_hoparlor_ma", val=V.ask("Calarken akim (mA, okunabildiyse) > "))
    V.v("mute 1")
    V.finish()


def ses_ogren(V):
    words = _words(V)
    V.start(V.fw(noise="0"))
    V.v("learn 1")
    V.step("Ogrenme modu acik (LED hizli yanip soner, hoparlor acik). Modul ciplak, servolar kapali.")
    if not V.o["kelimeler"]:
        V.step("1) Hepsini sil: 'Hello robot' -> 'I want to delete' -> 'Delete all'. Kimlikler 2, 204, 208 gorunmeli.")
        V.step("2) Uyanma kelimesi: 'Hello robot' -> 'Learning wake word'; her istemden sonra 'ordek' (3 kez).")
    V.step("3) Komutlar: 'ordek' -> 'Learning command word'. Her kelimeyi istem sorunca 3 kez soyleyin.")
    for n, w in enumerate(words):
        V.mark("ogren", word=w, sira=n)
        V.step("   %d/%d: '%s' (3 kez; modul siradakini isteyince Enter)" % (n + 1, len(words), vr.SOYLE.get(w, w)))
    V.step("4) 'Exit learning' deyin.")
    V.v("learn 0")
    V.echo("Dogrulama: her kelime icin 'ordek' -> LED -> kelime (3 deneme hakki).")
    for n, w in enumerate(words):
        for k in range(3):
            V.echo("[dogrula %d/%d] %s" % (n + 1, len(words), _say_led(w)))
            t0 = V.win(n, w, "dogrula")
            c = V.wait_cmd(t0, V.o["trial_s"])
            V.close(n, "dogrula")
            if c:
                V.echo("    %s -> kimlik %d" % (w, c[0][1]))
                break
            if k < 2:
                V.close(n, "dogrula", "void")
        V.wait_dev(V.o["gap_s"])
    V.finish()
    V.echo("Tablo: python tools/bench/voice_report.py tablo <klasor>  (configs/komut_kimlik.json)")


def ses_d1(V):
    V.start(V.fw(noise="0"))
    V.step("Deneme 1 (gurultu): modul 0.35 m yukseklikte, MG90S/SG90 modulden ~4 cm, DS yanda; siz ayakta, "
           "agiz-modul 1.3 m. Ses kaydini baslatin (telefon modulun yaninda; dosya adi ses-d1.wav). "
           "Enter: servolar oynamaya baslar.")
    V.v("noise %s" % V.o["desen"])
    V.wait_dev(2.0)
    V.block(_words(V), int(V.o["n"] or 20), "d1", "led", _say_led)
    V.v("noise 0")
    V.finish()


def ses_d2(V):
    V.start(V.fw(noise="0"))
    V.step("Deneme 2 (sessiz): servolar kapali, ortam sessiz, agiz-modul 1.8 m. Ses kaydini baslatin (ses-d2.wav).")
    if V.block(_words(V), int(V.o["n"] or 10), "d2", "led", _say_led):
        V.step("Araliksiz bicim: 'ordek gel' tek nefeste, LED beklemeden (10 kez).")
        V.block(["gel"], 10, "d2a", "araliksiz", lambda w: "'ordek %s' (araliksiz)" % vr.SOYLE.get(w, w))
    V.finish()


def ses_d3(V):
    o = V.o
    tab = V.table or dict(ids={}, wake=vr.WAKE)
    V.start(V.fw(noise="0"))
    V.step("Deneme 3 (uyanik pencerede yanlis komut, %g dk dogrulanmis): duzenek deneme 1 gibi, servolar calisir, "
           "kimse konusmaz. Canlilik icin kayitli '%s' (%s). Enter: basla." % (
               o["dk"], o["canli"], o["canli_wav"] or "her istemde siz calarsiniz"))
    V.v("noise %s" % o["desen"])
    V.echo("  20 s uyku tabani (fototransistor kapali seviyesi); konusmayin")
    V.mark("taban", op="bas")
    V.wait_dev(20.0)
    V.mark("taban", op="son")
    V.mark("yontem", val="a")
    V.v("wake 255")
    V.v("refresh 200")
    tw = None
    for k in range(3):
        t0 = V.win(k, "ordek", "uyan")
        V.echo("  'ordek' deyin (LED yanar), sonra susun")
        tw = V.wait_wake(t0, o["trial_s"])
        V.close(k, "uyan")
        if tw:
            break
    if tw is None:
        V.echo("  uyanma kimligi gelmedi; oturum bitiyor")
        V.v("noise 0")
        return V.finish()
    meth, nk, t_next, checked = "a", 0, tw + o["kontrol_s"] * 1e6, False
    last = [V.now]

    def ordek(now):                                   # B yontemi: PC 'ordek' kaydini ~14 s'de bir calar
        if meth == "b" and o["ordek_wav"] and now - last[0] >= o["ordek_s"] * 1e6:
            last[0] = now
            if play_wav(o["ordek_wav"]):
                V.mark("oynat", ad2="ordek")
    while True:
        V.wait_until(t_next, ordek)
        ok = V.live(nk)
        nk += 1
        r = vr.uyanik(vr.Ses.from_recorder(V.rec, V.test), tab)
        el = (V.now - tw) / 60e6
        V.echo("  canlilik %d %s; gecen %.1f dk, dogrulanmis uyanik %.1f dk, maruziyet %.1f dk" % (
            nk, "gecti" if ok else "KALDI", el, r.get("dogrulanmis_dk", 0.0), r.get("maruziyet_dk", 0.0)))
        if not r.get("gosterge", {}).get("tur"):
            V.echo("  UYARI: uyanik gosterge yok (%s): sure dogrulanamaz" % r.get("gosterge", {}).get("neden"))
        elif meth == "a" and not checked and el >= o["dogrula_dk"] - 0.01:
            checked = True
            if r.get("dogrulanmis_dk", 0.0) < 0.9 * el:
                meth = "b"
                V.mark("yontem", val="b")
                V.v("refresh 0")
                V.v("wake 15")
                V.step("A yontemi tutmadi. Kayitli 'ordek'i ~%g s'de bir calin (%s). Enter" % (
                    o["ordek_s"], "PC calacak" if o["ordek_wav"] else "kendi oynaticinizda dongu"))
        if r.get("maruziyet_dk", 0.0) >= o["dk"] or el >= o["max_dk"]:
            break
        t_next += o["kontrol_s"] * 1e6
    V.v("refresh 0")
    V.v("wake 15")
    if meth == "b" and not o["ordek_wav"]:
        V.step("Uyku bolumu (%g dk): kayitli 'ordek' calmayi durdurun. Enter" % o["uyku_dk"])
    if meth == "a":                                   # 255 s pencere kapanana kadar (son yenileme + 255 s)
        t_r = max([t for t, n, kv in V.rec.events if n == "vtx" and str(kv.get("val")) == "255"] + [tw])
        V.echo("  uyku bolumu icin pencerenin kapanmasi bekleniyor (en cok 255 s)")
        V.wait_until(max(V.now, t_r + 260e6))
    meth = "a"
    V.wait_dev(20.0)
    V.mark("uyku", op="bas")
    V.wait_dev(o["uyku_dk"] * 60.0)
    V.mark("uyku", op="son")
    V.v("noise 0")
    V.finish()


def ses_d4(V):
    V.start(V.fw(noise="0", wake_s=15))
    V.step("Modul davranisi A: pencere uzunlugu. 'ordek' deyin; 'SIMDI' cikinca 'evet' deyin. Enter")
    tw = V.wake_win("a_uyan")
    if tw:
        V.cmd_win("a_gec", "evet", tw + 17e6)
    V.wait_dev(5.0)
    V.step("B: gecerli komut pencereyi yeniliyor mu? 'ordek'; 1. SIMDI'de 'evet', 2. SIMDI'de 'hayir'. Enter")
    tw = V.wake_win("b_uyan")
    if tw:
        V.cmd_win("b_ara", "evet", tw + 8e6)
        V.cmd_win("b_gec", "hayir", tw + 17e6)
    V.wait_dev(20.0)
    V.v("auto 0")
    V.v("mute 1")
    V.v("wake 30")
    V.wait_dev(1.0)
    V.step("C: kalicilik. Modulun 5 V'unu 5 s cikarip geri takin (ESP32 acik kalsin); 3 s bekleyip Enter")
    V.mark("guc", op="ver")
    tw = V.wake_win("p_uyan")
    V.mark("soru", id="mute_duyuldu", val=V.ask("'ordek'e sesli yanit duyuldu mu? (e/h) > "))
    if tw:
        V.cmd_win("p_gec", "evet", tw + 22e6)
    V.v("auto 1")
    V.v("wake 15")
    V.v("mute 1")
    V.step("D: istem duyulabilirligi. Modulu kafa kesitine (yoksa kapali bir kutuya) koyun; 1 m uzakta durun. "
           "Enter: ogrenme modu acilir; 'ordek' -> 'Learning command word' deyin, istemi dinleyin, hemen "
           "'Exit learning' deyin")
    V.v("learn 1")
    V.mark("soru", id="istem_duyuldu", val=V.ask("Istem kesit icinden duyuldu mu? (e/h) > "))
    V.v("learn 0")
    V.finish()


def ses_izle(V):
    V.start(V.fw())
    V.echo("Izleme: kimlikler ekrana yazilir. Ctrl-C: bitir.")
    try:
        while not V.ended:
            V.pump(None, 1.0)
    except KeyboardInterrupt:
        pass
    V.finish()


VOICE_TESTS = {"ses-olcu": ses_olcu, "ses-ogren": ses_ogren, "ses-d1": ses_d1, "ses-d2": ses_d2, "ses-d3": ses_d3,
               "ses-d4": ses_d4, "ses-izle": ses_izle}


def run_voice(link, test, servo, out, extra, args=None, echo=print, ask=None, table=None):
    o = dict(VOICE_DEF)
    o.update({k: _num(str(v)) if k not in ("kelimeler", "canli_wav", "ordek_wav", "tablo", "canli", "desen") else v
              for k, v in (args or {}).items()})
    if table is None:
        table = vr.load_table(o["tablo"] or vr.TABLO)
    if table is None and test not in ("ses-olcu", "ses-ogren", "ses-izle"):
        echo("  UYARI: kimlik tablosu yok (configs/komut_kimlik.json); kimlikler kelimesiz gosterilir")
    V = Voice(link, test, o, echo, ask, table)
    echo("== %s" % test)
    try:
        VOICE_TESTS[test](V)
    except KeyboardInterrupt:
        link.send("!")
        echo("E-stop gonderildi (Ctrl-C)")
        V.rec.status = "abort:host"
    except RuntimeError as e:
        echo("  " + str(e))
        V.rec.status = V.rec.status or "abort:" + str(e)[:40]
    finally:
        V.finish()
    p = save(V.rec, out, servo, test, "voice", {}, extra)
    echo("   kayit: %s (%s, %d satir, %d kimlik)" % (p, V.rec.status, len(V.rec.rows), len(V.ids)))
    return [p]


def run_test(link, test, servo, out, extra, answer_fn=None, args=None, echo=print, confirm=None):
    paths = []
    if test in VOICE_TESTS:
        return run_voice(link, test, servo, out, extra, args, echo, confirm)
    if test in CMD_TESTS:
        lines = command(link, CMD_TESTS[test], 4.0)
        for ln in lines:
            echo("  " + ln)
        paths.append(save_cmd(lines, out, servo, test, extra))
        return paths
    for prog, params in TESTS[test]:
        p = dict(params)
        p.update(args or {})
        if p.get("_ask"):
            (confirm or input)("\n>>> " + p["_ask"] + " ")
        echo("== %s / %s %s" % (test, prog, " ".join("%s=%s" % kv for kv in p.items() if not kv[0].startswith("_"))))
        rec = run_program(link, prog, p, answer_fn, echo)
        paths.append(save(rec, out, servo, test, prog, p, extra))
        echo("   kayit: %s (%s, %d satir)" % (paths[-1], rec.status, len(rec.rows)))
        if not (rec.status or "").startswith("ok"):
            break
    return paths


def pack(folder):
    folder = Path(folder)
    z = folder.with_suffix(".zip") if folder.is_dir() else Path(str(folder) + ".zip")
    with zipfile.ZipFile(z, "w", zipfile.ZIP_DEFLATED) as zf:
        for p in sorted(folder.rglob("*")):
            if p.is_file() and p.suffix != ".zip":
                zf.write(p, p.relative_to(folder.parent))
    return str(z)


def main(argv=None):
    ap = argparse.ArgumentParser(description="OpenDuck-TR Adim 0 tezgah kaydedici")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--servo", default="servo1", help="servo kimligi (klasor adi), ornek DS3230-1")
    ap.add_argument("--model", default=None, help="DS3225MG | DS3230 | DS3235 | ES3054 | MG90S | SG90")
    ap.add_argument("--cls", default=None, choices=["ds", "es", "mg", "sg"])
    ap.add_argument("--test", default=None)
    ap.add_argument("--plan", default=None, choices=sorted(PLANS))
    ap.add_argument("--set", nargs="*", default=[], help="betik ayarlari k=v (mass_g, lever_mm, shunt_mohm ...)")
    ap.add_argument("--arg", nargs="*", default=[], help="program parametreleri k=v")
    ap.add_argument("--out", default="bench_logs")
    ap.add_argument("--note", default="")
    ap.add_argument("--vrail", type=float, default=None, help="servo rayi (V), multimetreyle olculen")
    ap.add_argument("--cmd", default=None)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--pack", default=None)
    a = ap.parse_args(argv)
    if a.list:
        print("testler:", ", ".join(list(CMD_TESTS) + list(TESTS)))
        print("ses testleri (G-SES-a):", ", ".join(VOICE_TESTS))
        for k, v in PLANS.items():
            print("plan %-5s: %s" % (k, " ".join(v)))
        return 0
    if a.pack:
        print("zip:", pack(a.pack))
        return 0
    if not a.port:
        ap.error("--port gerekli")
    link = Link(open_serial(a.port, a.baud))
    try:
        for ln in wait_ready(link, 1.0):
            print("  " + ln)
        command(link, "set hbto=5")
        if a.cls:
            for ln in command(link, "cls %s" % a.cls):
                print("  " + ln)
        if a.set:
            for ln in command(link, "set " + " ".join(a.set)):
                print("  " + ln)
        if a.cmd:
            for ln in command(link, a.cmd, 5.0):
                print(ln)
            return 0
        args = dict(kv.split("=", 1) for kv in a.arg)
        extra = dict(servo_model=a.model, servo_class=a.cls, note=a.note, vrail_meas=a.vrail)
        tests = PLANS[a.plan] if a.plan else [a.test]
        for t in tests:
            if t not in TESTS and t not in CMD_TESTS and t not in VOICE_TESTS:
                print("bilinmeyen test:", t)
                return 2
            if a.plan:
                s = input("\n=== %s: Enter = kostur, s = atla, q = cik > " % t).strip().lower()
                if s == "q":
                    break
                if s == "s":
                    continue
            run_test(link, t, a.servo, a.out, extra, args=args)
    finally:
        command(link, "set hbto=0", 0.5)
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
