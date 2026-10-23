# -*- coding: utf-8 -*-
"""
Tezgah log bicimi: firmware satir protokolu, kayit (logger + synth ortak) ve okuma (fit).

Firmware (tools/bench/bench/bench_main.cpp) USB seriden su satirlari yollar:
  #<metin>                 bilgi
  CFG,k=v,...              ayar dokumu ('cfg' komutuna yanit)
  S,<program>,<fw>         program basladi
  E,<t_us>,<ad>[,k=v]*     olay (params, seg, ans, potcal, stall, level, burst, ref, pca ...)
  H,<kolon>,...            D satirlarinin basligi (COLUMNS)
  D,<deger>,...            veri satiri (sabit kolonlar)
  B,<t_us>,<rail_mv>       hizli ray ornegi (overshoot burst)
  P,<id>,<metin>           kullanicidan yanit ister; host 'ans <metin>' yollar
  R,<anahtar>,<deger>      cihaz ustu hizli sonuc
  W,<metin>                uyari
  END,<program>,<durum>    bitti: ok | abort:<neden>
Kolonlar (D):
  t_us     esp_timer zamani (us)          seg      program parca indeksi
  cmd_us   PCA'ya yazilan darbe (us, sayim x tick)
  pot_mv   pot orta ucu, ADC1 pininde (mV, bolucu sonrasi, ortalama)
  ph_mv    pot ust ucu (besleme) ADC1 pininde (mV) | nan
  rail_mv  servo rayi ADC1 pininde (mV, bolucu sonrasi) | nan
  i_ma     INA226 akimi (mA, sont gerilimi / R)  vbus_mv  INA226 VBUS (mV)
  adsw_mv  ADS1115 AIN1 (pot orta ucu, ADS yolu, mV) | nan
  adslo_uv ADS1115 AIN2-AIN3 (pot alt ucu - yildiz GND, uV) | nan
  ntc_c    kasa NTC (ADS AIN0, C) | nan      amb_c  ortam NTC (ADC1, C) | nan
  cap_us   GPIO15'te olculen DUT darbe genisligi (us) | nan
Kayit (bir program kosusu): <kok>.csv (H + D), <kok>.events.csv, <kok>.burst.csv (varsa), <kok>.json (meta),
<kok>.raw.txt (logger: tum satirlar).
"""
import json
import math
from pathlib import Path

import numpy as np

FW_PROTO = 1
COLUMNS = ("t_us", "seg", "cmd_us", "pot_mv", "ph_mv", "rail_mv", "i_ma", "vbus_mv", "adsw_mv", "adslo_uv",
           "ntc_c", "amb_c", "cap_us")

# Sinif varsayilanlari (firmware ile ayni; servo-mechanics C, power-electronics F5)
CLASS_DEFAULTS = {   # ilim: surekli (1.5 s), ilim_peak: tepe (0.3 s; stall programinda surekli sinir da bu)
    "ds": dict(us_lo=500.0, us_hi=2500.0, us_c=1500.0, deg_per_us=0.090, freq=300, vrail=6.5,
               ilim=3.0, ilim_peak=6.0, analog=False),
    "es": dict(us_lo=700.0, us_hi=2300.0, us_c=1500.0, deg_per_us=0.0906, freq=50, vrail=6.0,
               ilim=1.2, ilim_peak=2.0, analog=False),
    "mg": dict(us_lo=600.0, us_hi=2400.0, us_c=1500.0, deg_per_us=0.095, freq=50, vrail=6.0,
               ilim=0.8, ilim_peak=1.5, analog=True),
    "sg": dict(us_lo=600.0, us_hi=2400.0, us_c=1500.0, deg_per_us=0.095, freq=50, vrail=6.0,
               ilim=0.7, ilim_peak=1.2, analog=True),
}
# Ayar varsayilanlari (CFG): bolucu 100k/47k + 100 nF (spec 3.1), sont mOhm, kol mm
CFG_DEFAULTS = dict(div=47.0 / 147.0, rail_div=47.0 / 147.0, rc_ms=3.2, shunt_mohm=5.0, lever_mm=100.0, mass_g=0.0,
                    lever_g=0.0, vcc=3.30, cls="ds", freq=300, rate=500, os=8)
G = 9.80665


def _num(s):
    s = s.strip()
    if s == "":
        return ""
    try:
        if s.lower() == "nan":
            return float("nan")
        if any(c in s for c in ".eE") or s.lower() in ("inf", "-inf"):
            return float(s)
        return int(s)
    except ValueError:
        try:
            return float(s)
        except ValueError:
            return s


def parse_kv(fields):
    out = {}
    for f in fields:
        if "=" in f:
            k, v = f.split("=", 1)
            out[k.strip()] = _num(v)
    return out


class RunRecorder:
    """Firmware satirlarini toplar; logger ve synth ayni kaydi uretir."""

    def __init__(self):
        self.header = None
        self.rows = []
        self.events = []        # (t_us, ad, {k: v})
        self.burst = []         # (t_us, rail_mv)
        self.results = {}
        self.prompts = []
        self.answers = {}
        self.warnings = []
        self.cfg = {}
        self.program = None
        self.fw = None
        self.status = None
        self.raw = []

    def feed(self, line):
        line = line.rstrip("\r\n")
        self.raw.append(line)
        if not line:
            return ""
        if line.startswith("#"):
            return "#"
        p = line.split(",")
        tag = p[0]
        if tag == "D" and self.header is not None:
            self.rows.append(",".join(p[1:]))
        elif tag == "H":
            self.header = p[1:]
        elif tag == "E" and len(p) >= 3:
            kv = parse_kv(p[3:])
            ev = (int(_num(p[1])), p[2], kv)
            self.events.append(ev)
            if p[2] == "ans":
                self.answers[str(kv.get("id"))] = kv.get("val")
        elif tag == "B" and len(p) >= 3:
            self.burst.append((int(_num(p[1])), float(_num(p[2]))))
        elif tag == "P":
            self.prompts.append((p[1] if len(p) > 1 else "", ",".join(p[2:])))
        elif tag == "R" and len(p) >= 3:
            self.results[p[1]] = _num(",".join(p[2:]))
        elif tag == "W":
            self.warnings.append(",".join(p[1:]))
        elif tag == "CFG":
            self.cfg.update(parse_kv(p[1:]))
        elif tag == "S":
            self.program = p[1] if len(p) > 1 else None
            self.fw = p[2] if len(p) > 2 else None
        elif tag == "END":
            self.status = p[2] if len(p) > 2 else "?"
        return tag

    def params(self):
        for t, name, kv in self.events:
            if name == "params":
                return dict(kv)
        return {}

    def write(self, stem, meta_extra=None):
        stem = Path(stem)
        stem.parent.mkdir(parents=True, exist_ok=True)
        hdr = self.header or list(COLUMNS)
        with open(str(stem) + ".csv", "w") as f:
            f.write(",".join(hdr) + "\n")
            for r in self.rows:
                f.write(r + "\n")
        with open(str(stem) + ".events.csv", "w") as f:
            f.write("t_us,name,args\n")
            for t, name, kv in self.events:
                f.write("%d,%s,%s\n" % (t, name, ";".join("%s=%s" % (k, v) for k, v in kv.items())))
        if self.burst:
            with open(str(stem) + ".burst.csv", "w") as f:
                f.write("t_us,rail_mv\n")
                for t, v in self.burst:
                    f.write("%d,%.1f\n" % (t, v))
        if self.raw:
            with open(str(stem) + ".raw.txt", "w") as f:
                f.write("\n".join(self.raw) + "\n")
        meta = dict(proto=FW_PROTO, program=self.program, fw=self.fw, status=self.status, cfg=self.cfg,
                    params=self.params(), answers=self.answers, results=self.results, prompts=self.prompts,
                    warnings=self.warnings, rows=len(self.rows), files=dict(csv=stem.name + ".csv",
                    events=stem.name + ".events.csv", burst=(stem.name + ".burst.csv") if self.burst else None))
        meta.update(meta_extra or {})
        with open(str(stem) + ".json", "w") as f:
            json.dump(meta, f, indent=1, ensure_ascii=False, default=_json_default)
        return str(stem) + ".json"


def _json_default(o):
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, float) and math.isnan(o):
        return None
    return str(o)


class Run:
    """Bir kosunun log'u: meta, kolon dizileri (d), olaylar, burst."""

    def __init__(self, json_path):
        p = Path(json_path)
        self.path = p
        with open(p) as f:
            self.meta = json.load(f)
        base = p.parent
        csv = base / self.meta["files"]["csv"]
        with open(csv) as f:
            hdr = f.readline().strip().split(",")
        arr = np.loadtxt(csv, delimiter=",", skiprows=1, ndmin=2) if self.meta.get("rows", 1) else \
            np.zeros((0, len(hdr)))
        self.d = {h: arr[:, i] for i, h in enumerate(hdr)}
        self.t = self.d["t_us"] * 1e-6 if len(arr) else np.zeros(0)
        self.events = []
        evp = base / self.meta["files"]["events"]
        if evp.exists():
            with open(evp) as f:
                f.readline()
                for ln in f:
                    t, name, args = (ln.rstrip("\n").split(",", 2) + ["", ""])[:3]
                    kv = parse_kv(args.split(";")) if args else {}
                    self.events.append((int(t) * 1e-6, name, kv))
        self.burst = None
        bp = self.meta["files"].get("burst")
        if bp and (base / bp).exists():
            self.burst = np.loadtxt(base / bp, delimiter=",", skiprows=1, ndmin=2)

    # kisayollar
    @property
    def program(self):
        return self.meta.get("program")

    @property
    def test(self):
        return self.meta.get("test") or self.program

    @property
    def tag(self):
        return str(self.params.get("tag", ""))

    @property
    def params(self):
        return self.meta.get("params", {})

    @property
    def cfg(self):
        c = dict(CFG_DEFAULTS)
        c.update(self.meta.get("cfg", {}))
        return c

    def ev(self, name):
        return [(t, kv) for t, n, kv in self.events if n == name]

    def segs(self):
        """[(t0, t1, kv)] 'seg' olaylarindan; t1 = sonraki seg ya da log sonu."""
        s = self.ev("seg")
        tend = float(self.t[-1]) if len(self.t) else 0.0
        return [(t, (s[i + 1][0] if i + 1 < len(s) else tend), kv) for i, (t, kv) in enumerate(s)]

    def mask(self, t0, t1):
        return (self.t >= t0) & (self.t < t1)


def load_dir(path):
    """Bir servo klasorundeki tum kosular (zaman sirali)."""
    runs = []
    for p in sorted(Path(path).glob("*.json")):
        try:
            with open(p) as f:
                m = json.load(f)
        except (OSError, ValueError):
            continue
        if isinstance(m, dict) and m.get("proto") == FW_PROTO and "files" in m:   # komut/params dosyalari atlanir
            runs.append(Run(p))
    return sorted(runs, key=lambda r: (r.meta.get("host_time") or "", r.path.name))
