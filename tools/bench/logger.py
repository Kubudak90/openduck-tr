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

Her kosu: <out>/<servo>/<zaman>_<test>_<program>[_<tag>].csv + .events.csv + .json (+ .burst.csv, .raw.txt).
Bicim: tools/sysid/logio.py. Istemler (P): jog icin '+10' / '-5' yazin, bitince 'ok'; diger sorulara deger yazin.
'!' + Enter: E-stop. Ctrl-C: E-stop + kaydi kapatir. Kalp atisi (hb) 1 s'de bir; kesilirse betik 5 s'de durur.
"""
import argparse
import json
import queue
import sys
import threading
import time
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.sysid.logio import RunRecorder  # noqa: E402

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
}
CMD_TESTS = {"scan": "scan", "osc": "osc auto"}
PLANS = {
    "ds": ["scan", "osc", "sweep", "potcal", "deadband", "speed", "step", "chirp", "ramp", "stall", "curpts",
           "accept300", "accept33", "overshoot", "noise", "nopwm"],
    "es": ["scan", "osc", "sweep", "potcal", "deadband", "speed", "step", "chirp", "ramp", "stall", "curpts",
           "accept300", "accept33", "noise"],
    "mg": ["scan", "osc", "sweep", "potcal", "speed", "step", "stall", "curpts", "accept33"],
    "heat": ["heat"],
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


def run_program(link, prog, params, answer_fn=None, echo=print, max_s=6 * 3600):
    """Bir firmware programini kostur, RunRecorder dondur."""
    rec = RunRecorder()
    for ln in command(link, "cfg", 1.5):
        rec.feed(ln)
    args = " ".join("%s=%s" % (k, v) for k, v in params.items() if not k.startswith("_"))
    link.send(("run %s %s" % (prog, args)).strip())
    t0, nD, last = time.time(), 0, time.time()
    try:
        while time.time() - t0 < max_s:
            m = link.get(0.2)
            if m is None:
                continue
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


def run_test(link, test, servo, out, extra, answer_fn=None, args=None, echo=print, confirm=None):
    paths = []
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
            if t not in TESTS and t not in CMD_TESTS:
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
