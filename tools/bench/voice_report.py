# -*- coding: utf-8 -*-
"""
G-SES-a analizi (spec-ek-komut.md K8): ses modulu oturum loglarindan kapi buyuklukleri, kimlik tablosu, gecikme.

  python tools/bench/voice_report.py tablo bench_logs/DF2301Q-1        # ses-ogren -> configs/komut_kimlik.json
  python tools/bench/voice_report.py rapor bench_logs/DF2301Q-1        # ses_rapor.md/.json + configs/ses_gecikme.json
  python tools/bench/voice_report.py rapor bench_logs/DF2301Q-1 --wav ses-d1=d1.wav --wav ses-d2=d2.wav

Girdi: tools/bench/logger.py ses-* kayitlari (tools/sysid/logio.py bicimi). Firmware olaylari: asr (id), notify,
ack, vframe/vbad/vjunk/vrx (ham), vtx, tick (piezo), win (deneme penceresi), mark (host isareti, yanitlar).
D kolonlari: t_us, seg, ldr_mv, led, learn, i_ma, vbus_mv, mg_us, mg2_us, sg_us, ds_us.
Ses kaydi (istege bagli, gecikme icin): WAV (PCM 8/16/24/32 bit); klasorde <test>.wav ya da <kayit koku>.wav
aranir. Telefonun m4a kaydini once WAV'a cevirin (Audacity: Disa aktar > WAV).

Tanimlar:
  deneme   'win open' .. 'win close' (void edilen atilir). Penceredeki ilk uyanma disi kimlik tablodan kelimeye
           cevrilir: dogru | ikame (baska kelime) | bilinmeyen (tabloda yok) | ogrenme (200-208) | yok.
  kelime   ses-d1 (gurultu, 1.3 m, 20/kelime): >= 18/20, dur >= 19/20. ses-d2 (sessiz, 1.8 m): ayni oranlar, bilgi.
  havuz    ses-d1 + ses-d2 LED bicimli denemeler: >= %90 ve Wilson (z = 1.96) alt siniri >= %85.
  ciftler  gel/geri, saga/sola, otur/kalk ikameleri (iki yon, butun denemeler): cift basina <= 1.
  zincir   LED bicimli denemede uyanma kimligi ve ardindan dogru komut: >= %90 (araliksiz bicim ayrica yazilir).
  uyanik   ses-d3. Dogrulanmis uyanik sure: gosterge (NOTIFY, yoksa fototransistor) uyanik derken ve iki gecen
           canlilik kontrolu arasinda kalan sure. Maruziyet = bu sure - pencereler - kayitli 'ordek' calmalarinin
           2 s'si (B yontemi). Gecer: 0 yanlis komut ve Poisson %95 ust siniri <= 1/10 dk. Sure < 30 dk: olculemedi.
  gecikme  kelime sonu (ses kaydinda; piezo tikleri log zamanina hizalanir) -> kimlik karesi (UART), s.
Kimlik tablosu: {"version": 1, "ids": {"<id>": "<komut>"}, "wake": [1, 2]} (komut anahtarlari KELIMELER).
"""
import argparse
import json
import math
import sys
import time
import wave
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.sysid import logio  # noqa: E402

# (tablo anahtari, soylenen bicim); ogrenme sirasi: bos modulde kimlik 5, 6, ... (voice-research 1.3)
KELIMELER = (("gel", "gel"), ("dur", "dur"), ("geri", "geri"), ("saga", "saga don"), ("sola", "sola don"),
             ("otur", "otur"), ("kalk", "kalk"), ("egil", "egil"), ("yuksel", "yuksel"), ("evet", "evet"),
             ("hayir", "hayir"), ("vak", "vak"))
ANAHTAR = tuple(k for k, _ in KELIMELER)
SOYLE = dict(KELIMELER)
WAKE = (1, 2)
OGRENME = tuple(range(200, 209))
TEHLIKELI = (("gel", "geri"), ("saga", "sola"), ("otur", "kalk"))
TABLO = ROOT / "configs" / "komut_kimlik.json"
GECIKME = ROOT / "configs" / "ses_gecikme.json"
ESIK = dict(kelime=18 / 20, dur=19 / 20, n_kelime=20, havuz=0.90, wilson=0.85, cift=1, zincir=0.90,
            uyanik_dk=30.0, ust_10dk=1.0)
_ALIAS = {"saga don": "saga", "sağa dön": "saga", "sağa": "saga", "sola don": "sola", "sola dön": "sola",
          "eğil": "egil", "yüksel": "yuksel", "hayır": "hayir", "ördek": "ordek"}
NAN = float("nan")


def anahtar(w):
    w = str(w if w is not None else "").strip().lower().replace("_", " ")
    return _ALIAS.get(w, w)


# ---------------------------------------------------------------- kimlik tablosu
# Tablo semasi rl/commands.load_id_table ile ortaktir (spec eki K4; sema 1): uyanma, komutlar {kimlik: kelime},
# ogrenme_kimlikleri; depodaki dosya P5k'nin ORNEK tablosudur ("ornek": true) ve olcum icin yok sayilir.
YAZIM = {"saga": "sağa dön", "sola": "sola dön", "egil": "eğil", "yuksel": "yüksel", "hayir": "hayır"}
IKINCI = tuple(range(17, 22))          # ikinci sablonlar icin bos kimlikler


def load_table(path=TABLO):
    """Olculmus kimlik tablosu ya da None (dosya yok ya da P5k ornegi)."""
    p = Path(path)
    if not p.exists():
        return None
    d = json.loads(p.read_text(encoding="utf-8"))
    if d.get("schema") != 1:
        raise SystemExit("%s: kimlik tablosu semasi %r (beklenen 1, rl/commands.load_id_table)" % (p, d.get("schema")))
    if d.get("ornek"):
        return None
    return dict(ids={int(k): anahtar(v) for k, v in d.get("komutlar", {}).items()},
                wake=tuple(int(x) for x in d.get("uyanma", WAKE)))


def write_table(path, ids, wake=WAKE):
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    d = {"schema": 1, "ornek": False,
         "not": "G-SES-a ogrenme adimindan (tools/bench/voice_report.py tablo, %s). Coktan bire; tabloda olmayan kimlik "
                "yok sayilir; 200-208 ogrenme kipi uyarisi." % time.strftime("%Y-%m-%d"),
         "uyanma": [int(x) for x in wake],
         "komutlar": {str(k): ids[k] for k in sorted(ids)},
         "kelimeler": dict([("uyanma", "ördek")] + [(k, YAZIM.get(k, k)) for k in ANAHTAR]),
         "bos_kimlikler": [i for i in IKINCI if i not in ids],
         "ogrenme_kimlikleri": list(OGRENME)}
    p.write_text(json.dumps(d, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    return str(p)


# ---------------------------------------------------------------- istatistik
def wilson(k, n, z=1.96):
    if n <= 0:
        return NAN, NAN
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    r = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (c - r) / d, (c + r) / d


def poisson_cdf(k, lam):
    s, term = 0.0, math.exp(-lam)
    for i in range(k + 1):
        s += term
        term *= lam / (i + 1)
    return s


def poisson_upper(k, conf=0.95):
    """Tek yonlu ust sinir: P(X <= k; lam) = 1 - conf. k = 0 -> -ln(0.05) = 3.00."""
    lo, hi = 0.0, 10.0 + 3.0 * k
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if poisson_cdf(k, mid) > 1 - conf:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# ---------------------------------------------------------------- aralik islemleri [(a, b)]
def iv_merge(iv, gap=0.0):
    out = []
    for a, b in sorted(iv):
        if b <= a:
            continue
        if out and a - out[-1][1] <= gap:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out


def iv_clip(iv, a, b):
    return [(max(x, a), min(y, b)) for x, y in iv if y > a and x < b]


def iv_len(iv):
    return float(sum(b - a for a, b in iv))


def iv_sub(iv, cut):
    cut = iv_merge(cut)
    out = []
    for a, b in iv:
        cur = a
        for x, y in cut:
            if y <= cur or x >= b:
                continue
            if x > cur:
                out.append((cur, x))
            cur = max(cur, y)
        if cur < b:
            out.append((cur, b))
    return out


def iv_has(iv, t):
    return any(a <= t <= b for a, b in iv)


# ---------------------------------------------------------------- oturum verisi
class Ses:
    """Bir ses oturumu: olaylar [(t s, ad, kv)], D kolonlari, parametreler."""

    def __init__(self, test, events, cols, params=None, path=None):
        self.test = test
        self.ev = sorted(events, key=lambda e: e[0])
        self.c = cols
        self.params = params or {}
        self.path = path
        tu = cols.get("t_us")
        self.t = np.asarray(tu, float) * 1e-6 if tu is not None else np.zeros(0)

    @classmethod
    def from_run(cls, run):
        return cls(run.test, run.events, run.d, run.params, run.path)

    @classmethod
    def from_recorder(cls, rec, test):
        """logger'in bellekteki kaydindan (cevrimici kontrol icin)."""
        ev = [(t * 1e-6, n, kv) for t, n, kv in rec.events]
        hdr = rec.header or []
        rows = [r.split(",") for r in rec.rows]
        rows = [r for r in rows if len(r) == len(hdr)]
        arr = np.array([[_f(x) for x in r] for r in rows], float).reshape(-1, len(hdr)) if hdr else np.zeros((0, 0))
        return cls(test, ev, {h: arr[:, i] for i, h in enumerate(hdr)}, rec.params())

    def named(self, name):
        return [(t, kv) for t, n, kv in self.ev if n == name]

    @property
    def t_end(self):
        te = [e[0] for e in self.ev]
        if len(self.t):
            te.append(float(self.t[-1]))
        return max(te) if te else 0.0


def _f(x):
    try:
        return float(x)
    except ValueError:
        return NAN


def load_sessions(folder):
    """Klasordeki ses oturumlari: {test: [Ses, ...]} (zaman sirali)."""
    out = {}
    for r in logio.load_dir(folder):
        if r.program not in ("voice", "gest"):
            continue
        out.setdefault(r.test, []).append(Ses.from_run(r))
    return out


def asr(s):
    return [(t, int(kv.get("id", 0))) for t, kv in s.named("asr")]


def windows(s):
    """Deneme pencereleri; ayni (set, i) icin 'void' oncekileri gecersiz kilar."""
    out, cur = [], {}
    for t, n, kv in s.ev:
        if n != "win":
            continue
        op = kv.get("op")
        key = (str(kv.get("set", "")), str(kv.get("i", "")))
        if op == "open":
            w = dict(kv, t0=t, t1=None, void=False)
            out.append(w)
            cur[key] = w
        elif op == "close" and key in cur and cur[key]["t1"] is None:
            cur[key]["t1"] = t
        elif op == "void":
            for w in out:
                if (str(w.get("set", "")), str(w.get("i", ""))) == key:
                    w["void"] = True
    for k, w in enumerate(out):
        if w["t1"] is None:
            nxt = [x["t0"] for x in out[k + 1:]]
            w["t1"] = min([w["t0"] + 20.0] + nxt)
    return out


def classify(i, table):
    if i in OGRENME:
        return "ogrenme"
    if table and i in table["ids"]:
        return table["ids"][i]
    return "bilinmeyen"


def trials(s, table, sets=None):
    ids = asr(s)
    wk = set(table["wake"]) if table else set(WAKE)
    out = []
    for w in windows(s):
        if w["void"] or (sets and str(w.get("set")) not in sets):
            continue
        seq = [(t, i) for t, i in ids if w["t0"] <= t <= w["t1"]]
        wake = [t for t, i in seq if i in wk]
        cmd = [(t, i) for t, i in seq if i not in wk]
        exp = anahtar(w.get("word"))
        got, idc, tc = "yok", None, None
        if cmd:
            tc, idc = cmd[0]
            got = classify(idc, table)
        out.append(dict(set=str(w.get("set")), i=w.get("i"), word=exp, form=str(w.get("form", "led")), got=got,
                        id=idc, t0=w["t0"], t1=w["t1"], t_cmd=tc, tick=w.get("tick"), wake_n=len(wake),
                        extra=max(0, len(cmd) - 1), ok=got == exp,
                        zincir=bool(cmd) and got == exp and any(t < tc for t in wake)))
    return out


# ---------------------------------------------------------------- dogruluk, karisiklik, zincir
def word_stats(tr, rate=None, rate_dur=None, n_req=None):
    rate = ESIK["kelime"] if rate is None else rate
    rate_dur = ESIK["dur"] if rate_dur is None else rate_dur
    out = {}
    for k in ANAHTAR:
        rs = [r for r in tr if r["word"] == k]
        n, c = len(rs), sum(r["ok"] for r in rs)
        thr = rate_dur if k == "dur" else rate
        need = math.ceil(thr * n - 1e-9)
        eksik = n_req is not None and n < n_req
        out[k] = dict(k=c, n=n, gerekli=need, ok=(n > 0 and c >= need and not eksik), eksik=eksik)
    return out


def confusion(tr):
    cols = list(ANAHTAR) + ["bilinmeyen", "ogrenme", "yok"]
    m = {k: {c: 0 for c in cols} for k in ANAHTAR}
    for r in tr:
        if r["word"] in m:
            m[r["word"]][r["got"] if r["got"] in cols else "bilinmeyen"] += 1
    return m


def pair_subs(m):
    return {"%s/%s" % (a, b): m[a][b] + m[b][a] for a, b in TEHLIKELI}


# ---------------------------------------------------------------- uyanik gosterge: NOTIFY ya da fototransistor
def notify_iv(s):
    on, iv, seen = None, [], False
    for t, kv in s.named("notify"):
        c = str(kv.get("code", "")).upper()
        if c == "0XB1":
            seen = True
            if on is None:
                on = t
        elif c == "0XB2":
            seen = True
            if on is not None:
                iv.append((on, t))
                on = None
    if on is not None:
        iv.append((on, s.t_end))
    return iv if seen else None


def otsu(x, bins=256):
    h, e = np.histogram(x, bins=bins)
    c = 0.5 * (e[:-1] + e[1:])
    w0 = np.cumsum(h)
    w1 = w0[-1] - w0
    m0 = np.cumsum(h * c)
    mt = m0[-1]
    with np.errstate(divide="ignore", invalid="ignore"):
        var = w0 * w1 * (m0 / w0 - (mt - m0) / w1) ** 2
    var[~np.isfinite(var)] = -1
    return float(c[int(np.argmax(var))])


def runs(t, b, dt):
    iv, start = [], None
    for k in range(len(b)):
        if b[k] and start is None:
            start = t[k]
        if start is not None and (not b[k] or k == len(b) - 1):
            iv.append((start, (t[k] + dt) if b[k] else t[k]))
            start = None
    return iv


def ldr_iv(s, thr=None):
    x = np.asarray(s.c.get("ldr_mv", np.zeros(0)), float)
    good = np.isfinite(x)
    info = dict(n=int(good.sum()))
    if good.sum() < 50:
        info["neden"] = "fototransistor verisi yok (set pin_ldr=6)"
        return None, info
    t, x = s.t[good], x[good]
    thr = otsu(x) if thr is None else float(thr)
    hi = x > thr
    info["esik_mv"] = thr
    if hi.all() or not hi.any():
        info["neden"] = "tek seviye (LED hic degismedi)"
        return None, info
    m1, m0 = float(x[hi].mean()), float(x[~hi].mean())
    sd = max(float(x[hi].std()), float(x[~hi].std()), 1.0)
    info.update(acik_mv=m1, kapali_mv=m0, gurultu_mv=sd)
    if m1 - m0 < max(30.0, 6 * sd):
        info["neden"] = "kontrast yetersiz"
        return None, info
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.02
    iv = [(a, b) for a, b in iv_merge(runs(t, hi, dt), gap=0.3) if b - a >= 0.2]
    return iv, info


def gosterge(s, table=None, thr=None):
    """Uyanik araliklari ve dogrulama: (iv | None, bilgi)."""
    wk = set(table["wake"]) if table else set(WAKE)
    wakes = [t for t, i in asr(s) if i in wk]
    iv = notify_iv(s)
    info = dict(tur="notify") if iv is not None else {}
    if iv is None:
        iv, li = ldr_iv(s, thr)
        info = dict(tur="ldr" if iv is not None else None, **li)
    if iv is None:
        return None, info
    hit = [any(a <= t + 1.5 and b >= t - 0.5 for a, b in iv) for t in wakes]
    info["uyanma_n"] = len(wakes)
    info["isabet"] = float(np.mean(hit)) if hit else NAN
    info["gecerli"] = (not hit) or info["isabet"] >= 0.8
    # yalniz uyanma (sonraki wake_s + 3 s icinde baska kimlik yok): gosterge suresi ~ wake_s olmali
    ids = asr(s)
    wake_s = float(s.params.get("wake_s", 15) or 15)
    durs = []
    for t in wakes:
        if any(t < u <= t + wake_s + 3 for u, _ in ids):
            continue
        cont = [b - max(a, t - 0.5) for a, b in iv if a <= t + 1.5 and b >= t - 0.5]
        if cont:
            durs.append(max(cont))
    info["yalniz_sure_s"] = durs
    return iv, info


# ---------------------------------------------------------------- uyanik pencerede yanlis komut (ses-d3)
def uyanik(s, table, gecerli=None):
    """Dogrulanmis uyanik sure, maruziyet, yanlis komut ve Poisson ust siniri."""
    wk = set(table["wake"]) if table else set(WAKE)
    ids = asr(s)
    ws = [w for w in windows(s) if not w["void"]]
    ind, info = gosterge(s, table)
    res = dict(gosterge=info)
    starts = []
    for w in ws:
        if str(w.get("set")) == "uyan":
            tw = [t for t, i in ids if w["t0"] <= t <= w["t1"] and i in wk]
            if tw:
                starts.append(tw[0])
    live = []
    for w in ws:
        if str(w.get("set")) != "canli":
            continue
        exp = anahtar(w.get("word"))
        got = [classify(i, table) for t, i in ids if w["t0"] <= t <= w["t1"] and i not in wk]
        live.append((w["t0"], exp in got or (bool(got) and not (table and table["ids"]))))
    res["canlilik"] = [dict(t=t, gecti=ok) for t, ok in live]
    if not starts:
        res.update(sonuc="olculemedi", neden="uyandirma kimligi yok (set=uyan)", dogrulanmis_dk=0.0, maruziyet_dk=0.0)
        return res
    t_a = starts[0]
    chk = [(t_a, True)] + [x for x in live if x[0] > t_a]
    segs = [(a[0], b[0]) for a, b in zip(chk, chk[1:]) if a[1] and b[1]]
    if ind is None:
        verified = []
        res["neden"] = "uyanik gosterge yok (NOTIFY yok, fototransistor gecersiz)"
    else:
        verified = iv_merge([x for a, b in segs for x in iv_clip(ind, a, b)])
    # B yontemi: kayitli 'ordek' calmalari (mark oynat) ve o bolumdeki uyanma kimlikleri dusulur
    ym = sorted((t, str(kv.get("val", "a"))) for t, kv in s.named("mark") if kv.get("ad") == "yontem")
    def yontem(t):
        y = "a"
        for tm, v in ym:
            if tm <= t:
                y = v
        return y
    cut = [(w["t0"], w["t1"]) for w in ws]
    cut += [(t, t + 3.0) for t, kv in s.named("mark") if kv.get("ad") == "oynat"]
    cut += [(t - 1.5, t + 2.0) for t, i in ids if i in wk and yontem(t) == "b"]
    expo = iv_sub(verified, cut)
    T = iv_len(expo) / 60.0
    inx = [(t, i) for t, i in ids if iv_has(expo, t)]
    fc = [(t, i) for t, i in inx if i not in wk and i not in OGRENME and table and i in table["ids"]]
    other = dict(bilinmeyen=[i for t, i in inx if i not in wk and classify(i, table) == "bilinmeyen"],
                 ogrenme=[i for t, i in inx if i in OGRENME],
                 uyanma=len([1 for t, i in inx if i in wk and yontem(t) == "a"]))
    k = len(fc)
    U = poisson_upper(k)
    up10 = U / T * 10.0 if T > 0 else float("inf")
    ok_ind = bool(ind is not None and info.get("gecerli", False)
                  and (info.get("tur") == "notify" or gecerli is not False))
    if not ok_ind or T < ESIK["uyanik_dk"]:
        sonuc = "olculemedi"
    else:
        sonuc = "gecti" if k == 0 and up10 <= ESIK["ust_10dk"] + 1e-9 else "kaldi"
    res.update(dogrulanmis_dk=iv_len(verified) / 60.0, maruziyet_dk=T, yanlis=[dict(t=t, id=i, komut=table["ids"][i])
               for t, i in fc], k=k, poisson_ust=U, ust_10dk=up10, diger=other, sonuc=sonuc,
               kontrol_n=len(live), kontrol_gecen=sum(1 for _, ok in live if ok))
    # A yontemi ilk 10 dk dogrulamasi
    ta_end = min([t for t, v in ym if v == "b"] + [t_a + 600.0])
    if ta_end >= t_a + 300.0:
        va = iv_len(iv_clip(verified, t_a, ta_end)) / 60.0
        res["a_dogrulama"] = dict(dk=(ta_end - t_a) / 60.0, dogrulanmis_dk=va, tuttu=va >= 0.9 * (ta_end - t_a) / 60.0)
    # uyku: mark uyku op=bas/son; yanlis uyanma = uyku sirasinda (gosterge kapali) gelen uyanma kimligi
    ub = [t for t, kv in s.named("mark") if kv.get("ad") == "uyku" and kv.get("op") == "bas"]
    us_ = [t for t, kv in s.named("mark") if kv.get("ad") == "uyku" and kv.get("op") == "son"]
    if ub:
        a, b = ub[0], (us_[0] if us_ else s.t_end)
        sleep = iv_sub([(a, b)], ind or []) if ind is not None else [(a, b)]
        fw = [t for t, i in ids if i in wk and a <= t <= b and (ind is None or not iv_has(ind, t - 0.6))]
        res["uyku"] = dict(dk=iv_len(sleep) / 60.0, yanlis_uyanma=len(fw),
                           komut=[i for t, i in ids if i not in wk and a <= t <= b])
    return res


# ---------------------------------------------------------------- gecikme: WAV + piezo tikleri
def _decode(raw, sw, ch):
    if sw == 2:
        x = np.frombuffer(raw, "<i2").astype(np.float32) / 32768.0
    elif sw == 4:
        x = np.frombuffer(raw, "<i4").astype(np.float32) / 2147483648.0
    elif sw == 3:
        b = np.frombuffer(raw, np.uint8).reshape(-1, 3).astype(np.int32)
        v = b[:, 0] | (b[:, 1] << 8) | (b[:, 2] << 16)
        x = (np.where(v >= 1 << 23, v - (1 << 24), v)).astype(np.float32) / 8388608.0
    else:
        x = (np.frombuffer(raw, np.uint8).astype(np.float32) - 128.0) / 128.0
    return x.reshape(-1, ch).mean(axis=1) if ch > 1 else x


def band_env(path, bands, hop_s=0.005, win_s=0.02):
    """Bant gucleri (kare zamani s, [bant dizileri]); dosya parca parca okunur."""
    with wave.open(str(path), "rb") as w:
        sr, ch, sw = w.getframerate(), w.getnchannels(), w.getsampwidth()
        hop, win = max(1, int(round(hop_s * sr))), max(8, int(round(win_s * sr)))
        nfft = 1 << (win - 1).bit_length()
        f = np.fft.rfftfreq(nfft, 1.0 / sr)
        masks = [(f >= lo) & (f < hi) for lo, hi in bands]
        hann = np.hanning(win).astype(np.float32)
        buf, out, nf = np.zeros(0, np.float32), [[] for _ in bands], 0
        while True:
            raw = w.readframes(hop * 2000)
            if raw:
                buf = np.concatenate([buf, _decode(raw, sw, ch)])
            n = (len(buf) - win) // hop + 1 if len(buf) >= win else 0
            if n > 0:
                idx = np.arange(win)[None, :] + hop * np.arange(n)[:, None]
                P = np.abs(np.fft.rfft(buf[idx] * hann, nfft, axis=1)) ** 2
                for k, m in enumerate(masks):
                    out[k].append(P[:, m].sum(axis=1))
                buf = buf[n * hop:]
                nf += n
            if not raw:
                break
    t = (np.arange(nf) * hop + win / 2.0) / sr
    return t, [np.concatenate(o) if o else np.zeros(0) for o in out]


def find_ticks(t, e_n, e_side, bw_n, bw_side, min_gap=0.5):
    """Dar bant (tik frekansi) / yan bant yogunluk orani yuksek ve tabanin 15 dB ustu: tik basi."""
    d = 10 * np.log10(e_n + 1e-20)
    base = float(np.median(d))
    ratio = (e_n / bw_n) / (e_side / bw_side + 1e-20)
    on = (d > base + 15.0) & (ratio > 20.0)
    out, last = [], -1e9
    for k in np.flatnonzero(on & ~np.concatenate([[False], on[:-1]])):
        if t[k] - last >= min_gap:
            out.append(float(t[k]))
            last = t[k]
    return np.array(out)


def align(tl, ta, tol=0.04):
    """Log tik zamanlari -> ses zamani: en cok eslesen kayma, sonra dogrusal uydurma. Doner (a, b, n) | None."""
    tl, ta = np.asarray(tl, float), np.asarray(ta, float)
    if len(tl) < 2 or len(ta) < 2:
        return None
    best = (0, 0.0)
    for i in range(min(len(tl), 40)):
        for j in range(len(ta)):
            p = tl + (ta[j] - tl[i])
            k = np.clip(np.searchsorted(ta, p), 1, len(ta) - 1)
            near = np.minimum(np.abs(ta[k] - p), np.abs(ta[k - 1] - p))
            n = int((near < tol).sum())
            if n > best[0]:
                best = (n, ta[j] - tl[i])
    n, b0 = best
    if n < max(3, len(tl) // 2):
        return None
    p = tl + b0
    k = np.clip(np.searchsorted(ta, p), 1, len(ta) - 1)
    m = np.where(np.abs(ta[k] - p) < np.abs(ta[k - 1] - p), ta[k], ta[k - 1])
    sel = np.abs(m - p) < tol
    a, b = np.polyfit(tl[sel], m[sel], 1) if sel.sum() >= 2 else (1.0, b0)
    return float(a), float(b), int(sel.sum())


def word_end(t, e_sp, a0, a1, hop=0.005):
    """[a0, a1] icindeki son konusma kosusunun sonu (taban + 10 dB, >= 3 kare)."""
    m = (t >= a0) & (t <= a1)
    if m.sum() < 10:
        return None
    d = 10 * np.log10(e_sp[m] + 1e-20)
    ts = t[m]
    act = d > np.percentile(d, 20) + 10.0
    k = len(act) - 1
    while k >= 0:
        if act[k]:
            j = k
            while j >= 0 and act[j]:
                j -= 1
            if k - j >= 3:
                return float(ts[k] + hop / 2)
            k = j
        k -= 1
    return None


def latency_wav(s, trs, wav):
    """Dogru tanimali denemeler icin gecikme (s) listesi ve hizalama bilgisi."""
    hz = float(s.params.get("tick_hz", 4000) or 4000)
    bands = [(hz - 150, hz + 150), (hz - 1200, hz - 400), (hz + 400, hz + 1200), (250, 3400)]
    t, (en, s1, s2, sp) = band_env(wav, bands)
    ta = find_ticks(t, en, s1 + s2, 300.0, 1600.0)
    tl = np.array([tt for tt, kv in s.named("tick")])
    al = align(tl, ta)
    info = dict(wav=str(wav), tik_log=len(tl), tik_ses=len(ta))
    if al is None:
        info["neden"] = "tikler hizalanamadi"
        return [], info
    a, b, n = al
    info.update(eslesen=n, olcek=a, kayma_s=b)
    lat = []
    for r in trs:
        if not r["ok"] or r["t_cmd"] is None:
            continue
        ti = [tt for tt in tl if r["t0"] - 0.05 <= tt <= r["t0"] + 0.05]
        if not ti:
            continue
        a0, a1 = a * ti[0] + b + 0.15, a * r["t_cmd"] + b - 0.03
        we = word_end(t, sp, a0, a1)
        if we is None:
            continue
        L = a * r["t_cmd"] + b - we
        if 0.05 <= L <= 3.0:
            lat.append(dict(set=r["set"], i=r["i"], word=r["word"], s=float(L)))
    return lat, info


def gecikme_json(lat, path, kaynak):
    x = np.sort(np.array([r["s"] for r in lat], float))
    q = (lambda p: float(np.percentile(x, p))) if len(x) else (lambda p: NAN)
    d = {"version": 1, "kaynak": "G-SES-a", "olcum": "kelime sonu -> modul kimlik karesi (UART), s",
         "kayitlar": kaynak, "n": int(len(x)), "gecikme_s": [round(float(v), 4) for v in x],
         "ozet": {"min": q(0), "p05": q(5), "p50": q(50), "p95": q(95), "max": q(100),
                  "ort": float(x.mean()) if len(x) else NAN},
         "uniform_s": [round(q(5), 3), round(q(95), 3)] if len(x) else None,
         "kosul": {k: int(sum(1 for r in lat if r["set"] == k)) for k in sorted({r["set"] for r in lat})},
         "tarih": time.strftime("%Y-%m-%d")}
    if path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(d, indent=1, default=_jd) + "\n", encoding="utf-8")
    return d


def _jd(o):
    if isinstance(o, float) and math.isnan(o):
        return None
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return str(o)


# ---------------------------------------------------------------- ogrenme oturumu -> kimlik tablosu
def build_table(s, base=None):
    """ses-ogren 'dogrula' pencerelerinden kimlik -> komut. Doner (ids, uyarilar)."""
    ids = dict(base["ids"]) if base else {}
    warn = []
    seen = {}
    for w in windows(s):
        if w["void"] or str(w.get("set")) != "dogrula":
            continue
        k = anahtar(w.get("word"))
        got = [i for t, i in asr(s) if w["t0"] <= t <= w["t1"] and i not in WAKE]
        if not got:
            warn.append("%s: dogrulamada kimlik gelmedi" % k)
            continue
        i = got[0]
        if i in OGRENME or not (3 <= i <= 255):
            warn.append("%s: %d ogrenme/gecersiz kimlik, atlandi" % (k, i))
            continue
        if i in ids and ids[i] != k:
            warn.append("kimlik %d hem %s hem %s: yeni deger %s" % (i, ids[i], k, k))
        if not (5 <= i <= 21):
            warn.append("%s: kimlik %d ogrenilmis aralik (5-21) disinda (sabit komut?)" % (k, i))
        ids[i] = k
        seen.setdefault(k, set()).add(i)
    for k in ANAHTAR:
        if k not in set(ids.values()):
            warn.append("%s: tabloda kimlik yok" % k)
    sira = [anahtar(kv.get("word")) for t, kv in s.named("mark") if kv.get("ad") == "ogren"]
    if sira and not base:
        for n, k in enumerate(sira):
            if k in seen and (5 + n) not in seen[k]:
                warn.append("%s: ogrenme sirasina gore beklenen %d, dogrulanan %s" % (k, 5 + n, sorted(seen[k])))
    return ids, warn


# ---------------------------------------------------------------- olcumler (ses-olcu) ve davranis (ses-d4)
def answers(s, ad):
    return {str(kv.get("id")): kv.get("val") for t, kv in s.named("mark") if kv.get("ad") == ad}


def olcu(s):
    res = dict(yanit=answers(s, "olcu"))
    seg = {}
    for t, kv in s.named("mark"):
        if kv.get("ad") == "akim":
            seg.setdefault(str(kv.get("id")), {})[str(kv.get("op"))] = t
    ima = np.asarray(s.c.get("i_ma", np.zeros(0)), float)
    ina = {}
    for k, d in seg.items():
        if "bas" in d and "son" in d and len(ima):
            m = (s.t >= d["bas"]) & (s.t <= d["son"]) & np.isfinite(ima)
            if m.sum():
                ina[k] = dict(ort_ma=float(ima[m].mean()), tepe_ma=float(ima[m].max()), n=int(m.sum()))
    res["ina"] = ina
    tx = _f(res["yanit"].get("tx_bosta_v", NAN))
    res["seviye"] = (None if math.isnan(tx) else ("yalniz 1 kOhm seri" if tx <= 3.6 else
                     "bolucu: 1 kOhm seri + 2 kOhm GND'ye (5 V -> 3.33 V)"))
    return res


def davranis(s, table):
    ids = asr(s)
    wk = set(table["wake"]) if table else set(WAKE)
    rec = {}
    for w in windows(s):
        if w["void"] or str(w.get("set")) != "d4":
            continue
        got = [classify(i, table) for t, i in ids if w["t0"] <= t <= w["t1"] and i not in wk]
        wak = [t for t, i in ids if w["t0"] <= t <= w["t1"] and i in wk]
        rec[str(w.get("form"))] = dict(taninan=got, uyanma=len(wak), t_uyanma=wak[0] if wak else None,
                                       beklenen=anahtar(w.get("word")), ok=anahtar(w.get("word")) in got)
    g = lambda f: rec.get(f, {}).get("ok")  # noqa: E731
    if g("a_gec") is None or g("b_gec") is None:
        yen = None
    elif g("a_gec"):
        yen = "pencere beklenenden uzun (wake_s uygulanmadi ya da varsayilan > 17 s)"
    else:
        yen = "gecerli komut pencereyi yeniliyor" if g("b_gec") else "gecerli komut pencereyi yenilemiyor"
    ans = answers(s, "soru")
    pw = g("p_gec")
    kal = dict(mute=(None if "mute_duyuldu" not in ans else ("kalmadi (yanit duyuldu)" if str(ans["mute_duyuldu"])
                     .startswith("e") else "kaldi (sessiz)")),
               wake=(None if pw is None else ("kaldi (30 s)" if pw else "sifirlandi")))
    ind, info = gosterge(s, table)
    # A testinde (ordek, 17 s'de taninmayan 'evet') gosterge suresi ~ wake_s ise modul LED'i uyanikligi izler
    led, wa = None, rec.get("a_uyan", {}).get("t_uyanma")
    wake_s = float(s.params.get("wake_s", 15) or 15)
    if ind is not None and wa is not None and g("a_gec") is False:
        cont = [b - wa for a, b in ind if a <= wa + 1.5 and b >= wa - 0.5]
        led = bool(cont and abs(float(max(cont)) - wake_s) <= 0.3 * wake_s)
        info["a_sure_s"] = max(cont) if cont else None
    return dict(pencere=rec, yenileme=yen, kalicilik=kal, istem_duyuldu=ans.get("istem_duyuldu"), gosterge=info,
                led_uyanikligi_izliyor=led)


def kesifler(sess):
    """Butun oturumlar: NOTIFY/ACK/bilinmeyen/bozuk kare sayilari, pencere disi kimlikler."""
    c = dict(notify={}, ack=0, vframe=0, vbad=0, vjunk=0)
    for ss in sess.values():
        for s in ss:
            for t, kv in s.named("notify"):
                k = str(kv.get("ad"))
                c["notify"][k] = c["notify"].get(k, 0) + 1
            for n in ("ack", "vframe", "vbad", "vjunk"):
                c[n] += len(s.named(n))
    return c


# ---------------------------------------------------------------- rapor
def _pc(x):
    return "nan" if x is None or (isinstance(x, float) and math.isnan(x)) else "%.1f%%" % (100 * x)


def analyze(folder, table=None, wavs=None, gecikme_out=GECIKME):
    folder = Path(folder)
    sess = load_sessions(folder)
    first = lambda k: (sess.get(k) or [None])[-1]  # noqa: E731 (son kayit)
    notes = []
    if table is None and first("ses-ogren") is not None:
        ids, w = build_table(first("ses-ogren"))
        table = dict(ids=ids, wake=WAKE)
        notes.append("kimlik tablosu ses-ogren kaydindan kuruldu (tablo yazilmadi; 'tablo' komutunu kosun)")
        notes += w
    if table is None:
        raise SystemExit("kimlik tablosu yok: once 'voice_report.py tablo <klasor>' (ses-ogren kaydi gerekir)")
    R = dict(klasor=str(folder), tablo={str(k): v for k, v in table["ids"].items()}, notlar=notes)
    d1 = trials(first("ses-d1"), table, {"d1"}) if first("ses-d1") else []
    d2s = first("ses-d2")
    d2 = trials(d2s, table, {"d2"}) if d2s else []
    d2a = trials(d2s, table, {"d2a"}) if d2s else []
    R["d1"] = dict(n=len(d1), kelime=word_stats(d1, n_req=ESIK["n_kelime"]))
    R["d2"] = dict(n=len(d2), kelime=word_stats(d2))
    pool = d1 + d2
    k, n = sum(r["ok"] for r in pool), len(pool)
    lo, hi = wilson(k, n)
    R["havuz"] = dict(k=k, n=n, oran=k / n if n else NAN, wilson_alt=lo, wilson_ust=hi)
    allt = d1 + d2 + d2a
    conf = confusion(allt)
    R["karisiklik"] = conf
    R["ciftler"] = pair_subs(conf)
    led = [r for r in pool if r["form"] == "led"]
    R["zincir"] = dict(k=sum(r["zincir"] for r in led), n=len(led),
                       araliksiz=dict(k=sum(r["zincir"] for r in d2a), n=len(d2a),
                                      dogru=sum(r["ok"] for r in d2a)))
    R["zincir"]["oran"] = R["zincir"]["k"] / R["zincir"]["n"] if R["zincir"]["n"] else NAN
    R["fazla_kimlik"] = sum(r["extra"] for r in allt)
    d4 = first("ses-d4")
    R["d4"] = davranis(d4, table) if d4 else None
    gec = R["d4"]["led_uyanikligi_izliyor"] if R["d4"] else None
    d3 = first("ses-d3")
    R["d3"] = uyanik(d3, table, gecerli=gec) if d3 else None
    ol = first("ses-olcu")
    R["olcu"] = olcu(ol) if ol else None
    R["kesif"] = kesifler(sess)
    # gecikme
    lat, linfo = [], []
    wavs = dict(wavs or {})
    for name, trs in (("ses-d1", d1), ("ses-d2", d2 + d2a)):
        s = first(name)
        if s is None:
            continue
        cands = [wavs.get(name)] + [folder / (name + ".wav")] + ([Path(str(s.path)[:-5] + ".wav")] if s.path else [])
        wv = next((Path(c) for c in cands if c and Path(c).exists()), None)
        if wv is None:
            linfo.append(dict(test=name, neden="ses kaydi yok"))
            continue
        L, info = latency_wav(s, trs, wv)
        info["test"] = name
        lat += L
        linfo.append(info)
    R["gecikme"] = dict(bilgi=linfo, ozet=gecikme_json(lat, gecikme_out if lat else None,
                                                        [i.get("wav") for i in linfo if i.get("wav")]))
    R["kapi"] = gate(R)
    return R


def gate(R):
    out = []
    def add(ad, deger, esik, ok):
        out.append(dict(olcut=ad, deger=deger, esik=esik, sonuc=("gecti" if ok else "kaldi") if ok is not None
                        else "olculemedi"))
    kw = R["d1"]["kelime"]
    if R["d1"]["n"]:
        bad = [k for k, v in kw.items() if not v["ok"] and not v["eksik"]]
        eks = [k for k, v in kw.items() if v["eksik"]]
        low = min(kw.items(), key=lambda kv: (kv[1]["k"] / kv[1]["n"]) if kv[1]["n"] else -1)
        add("Kelime basina (ses-d1: gurultu, 1.3 m)", "en dusuk %s %d/%d%s%s" % (
            low[0], low[1]["k"], low[1]["n"], (", kalan: " + " ".join(bad)) if bad else "",
            (", eksik deneme: " + " ".join(eks)) if eks else ""), ">= 18/20, dur >= 19/20",
            False if bad else (None if eks else True))
    else:
        add("Kelime basina (ses-d1)", "kayit yok", ">= 18/20, dur >= 19/20", None)
    h = R["havuz"]
    add("Havuz (ses-d1 + ses-d2)", "%d/%d = %s, Wilson alt %s" % (h["k"], h["n"], _pc(h["oran"]), _pc(h["wilson_alt"])),
        ">= %90, W-alt >= %85", (h["oran"] >= ESIK["havuz"] and h["wilson_alt"] >= ESIK["wilson"]) if h["n"] else None)
    c = R["ciftler"]
    add("Tehlikeli ciftler (ikame, iki yon)", ", ".join("%s %d" % kv for kv in c.items()), "cift basina <= 1",
        all(v <= ESIK["cift"] for v in c.values()) if h["n"] else None)
    d3 = R["d3"]
    if d3:
        add("Uyanik pencerede yanlis komut (ses-d3)", "%d / %.1f dk; Poisson %%95 ust %.2f / 10 dk" % (
            d3.get("k", 0), d3.get("maruziyet_dk", 0.0), d3.get("ust_10dk", float("inf"))),
            "0 / >= 30 dk; <= 1 / 10 dk", None if d3["sonuc"] == "olculemedi" else d3["sonuc"] == "gecti")
    else:
        add("Uyanik pencerede yanlis komut (ses-d3)", "kayit yok", "0 / >= 30 dk; <= 1 / 10 dk", None)
    z = R["zincir"]
    add("Zincir ordek -> komut", "%d/%d = %s (araliksiz %d/%d)" % (z["k"], z["n"], _pc(z["oran"]), z["araliksiz"]["k"],
        z["araliksiz"]["n"]), ">= %90", (z["oran"] >= ESIK["zincir"]) if z["n"] else None)
    res = [o["sonuc"] for o in out]
    genel = "KALDI" if "kaldi" in res else ("OLCULEMEDI" if "olculemedi" in res else "GECTI")
    return dict(satirlar=out, genel=genel)


def markdown(R):
    L = ["# G-SES-a raporu: ses modulu (spec-ek-komut.md K8)", "",
         "Kayit: `%s`. Tarih: %s." % (R["klasor"], time.strftime("%Y-%m-%d")), "", "## Kapi", "",
         "| Olcut | Deger | Esik | Sonuc |", "|---|---|---|---|"]
    for o in R["kapi"]["satirlar"]:
        L.append("| %s | %s | %s | %s |" % (o["olcut"], o["deger"], o["esik"], o["sonuc"]))
    L += ["", "**Kapi: %s.** Tutmazsa ek K3 yedek yolu: yeniden ogretme ve ikinci sablonlar (17-21), conta/kopuk, "
          "sonra kullanici karari." % R["kapi"]["genel"], ""]
    if R["notlar"]:
        L += ["Notlar:"] + ["- " + n for n in R["notlar"]] + [""]
    for name in ("d1", "d2"):
        kw = R[name]["kelime"]
        L += ["## Kelime basina dogruluk (ses-%s)" % name, "", "| Kelime | Dogru | n | Gerekli | Sonuc |",
              "|---|---|---|---|---|"]
        for k, v in kw.items():
            L.append("| %s | %d | %d | %d | %s |" % (SOYLE[k], v["k"], v["n"], v["gerekli"],
                     "eksik" if v["eksik"] else ("gecti" if v["ok"] else "kaldi")))
        L.append("")
    cols = list(ANAHTAR) + ["bilinmeyen", "ogrenme", "yok"]
    L += ["## Karisiklik matrisi (butun denemeler; satir: soylenen, sutun: taninan)", "",
          "| | " + " | ".join(cols) + " |", "|---" * (len(cols) + 1) + "|"]
    for k in ANAHTAR:
        L.append("| %s | %s |" % (k, " | ".join(str(R["karisiklik"][k][c]) for c in cols)))
    z = R["zincir"]
    L += ["", "Tehlikeli cift ikameleri: " + ", ".join("%s %d" % kv for kv in R["ciftler"].items()) + ".",
          "Zincir (LED bicimi): %d/%d. Araliksiz 'ordek gel': zincir %d/%d, dogru komut %d/%d. Pencerede fazladan "
          "komut kimligi: %d." % (z["k"], z["n"], z["araliksiz"]["k"], z["araliksiz"]["n"], z["araliksiz"]["dogru"],
                                  z["araliksiz"]["n"], R["fazla_kimlik"]), ""]
    g = R["gecikme"]
    oz = g["ozet"]
    L += ["## Gecikme (kelime sonu -> kimlik karesi)", ""]
    if oz["n"]:
        o = oz["ozet"]
        L.append("n = %d; min %.2f, p05 %.2f, p50 %.2f, p95 %.2f, max %.2f s. A10 DR dosyasi: "
                 "`configs/ses_gecikme.json` (uniform_s = [p05, p95] = %s)." % (
                     oz["n"], o["min"], o["p05"], o["p50"], o["p95"], o["max"], oz["uniform_s"]))
    for i in g["bilgi"]:
        L.append("- %s: %s" % (i.get("test"), ", ".join("%s=%s" % (k, v) for k, v in i.items() if k != "test")))
    L.append("")
    d3 = R["d3"]
    L += ["## Uyanik pencerede yanlis komut (ses-d3)", ""]
    if d3:
        L.append("Sonuc: **%s**. Dogrulanmis uyanik %.1f dk, maruziyet %.1f dk, yanlis komut %d (Poisson %%95 ust %.2f "
                 "/ 10 dk). Canlilik %s/%s. Gosterge: %s." % (d3["sonuc"], d3.get("dogrulanmis_dk", 0),
                 d3.get("maruziyet_dk", 0), d3.get("k", 0), d3.get("ust_10dk", float("inf")),
                 d3.get("kontrol_gecen", 0), d3.get("kontrol_n", 0), _kv(d3.get("gosterge", {}))))
        if d3.get("neden"):
            L.append("Neden: %s." % d3["neden"])
        if d3.get("a_dogrulama"):
            a = d3["a_dogrulama"]
            L.append("A yontemi (pencere 255 s + 200 s yenileme) ilk %.0f dk: dogrulanmis %.1f dk, %s." % (
                a["dk"], a["dogrulanmis_dk"], "tuttu" if a["tuttu"] else "tutmadi"))
        if d3.get("diger"):
            L.append("Maruziyette diger kimlikler: %s." % _kv(d3["diger"]))
        if d3.get("uyku"):
            u = d3["uyku"]
            L.append("Uykuda %.1f dk: yanlis uyanma %d, komut kimlikleri %s." % (
                u["dk"], u["yanlis_uyanma"], u["komut"]))
    else:
        L.append("Kayit yok.")
    L += ["", "## Modul davranisi (ses-d4)", ""]
    d4 = R["d4"]
    if d4:
        L.append("- Pencere yenileme: %s." % d4["yenileme"])
        L.append("- Kalicilik (guc kesilince): setMuteMode %s; setWakeTime %s." % (d4["kalicilik"]["mute"],
                                                                                  d4["kalicilik"]["wake"]))
        L.append("- Ogrenme istemi kafa kesiti icinden duyuldu mu: %s." % d4["istem_duyuldu"])
        L.append("- Modul LED'i uyanik durumu izliyor mu (fototransistor): %s. %s" % (
            d4.get("led_uyanikligi_izliyor"), _kv(d4["gosterge"])))
    else:
        L.append("Kayit yok.")
    k = R["kesif"]
    L += ["- NOTIFY kareleri: %s; ACK: %d; bilinmeyen kare: %d; bozuk kare: %d; kare disi bayt obegi: %d." % (
        k["notify"] or "yok", k["ack"], k["vframe"], k["vbad"], k["vjunk"]),
        "- Firmware canlilik sozlesmesi (K8.5): %s." % canlilik_onerisi(R), "", "## Olcumler (ses-olcu)", ""]
    ol = R["olcu"]
    if ol:
        for kk, v in ol["yanit"].items():
            L.append("- %s: %s" % (kk, v))
        for kk, v in ol["ina"].items():
            L.append("- INA226 %s: ortalama %.1f mA, tepe %.1f mA (n = %d)" % (kk, v["ort_ma"], v["tepe_ma"], v["n"]))
        L.append("- Seviye (ek K3): %s." % (ol["seviye"] or "TX gerilimi girilmedi"))
    else:
        L.append("Kayit yok.")
    L += ["", "## Kimlik tablosu", "", ", ".join("%s: %s" % kv for kv in sorted(R["tablo"].items(),
                                                                                  key=lambda kv: int(kv[0]))), ""]
    return "\n".join(L)


def canlilik_onerisi(R):
    k = R["kesif"]["notify"]
    if k.get("uyanma_bas") and k.get("uyanma_son"):
        return "NOTIFY uyanma kareleri geliyor: firmware sessizligi bunlarla ve periyodik canlilikla izleyebilir"
    if R.get("d4") and R["d4"].get("led_uyanikligi_izliyor"):
        return "NOTIFY yok; modul LED'i uyanikligi izliyor (robotta LED'i okuyacak bir giris yok: yalniz tezgah)"
    return "NOTIFY yok; sessiz modul yalniz beklenen kimlik gelmemesiyle (canlilik kelimesi) anlasilir"


def _kv(d):
    return ", ".join("%s=%s" % (k, ("%.3g" % v) if isinstance(v, float) else v) for k, v in d.items())


def main(argv=None):
    ap = argparse.ArgumentParser(description="G-SES-a analizi (ek K8)")
    ap.add_argument("komut", choices=["tablo", "rapor"])
    ap.add_argument("klasor", help="ses oturumlarinin klasoru, ornek bench_logs/DF2301Q-1")
    ap.add_argument("--tablo", default=str(TABLO), help="kimlik tablosu (okunur / yazilir)")
    ap.add_argument("--ekle", action="store_true", help="tablo: var olan tabloya ekle (ikinci sablonlar 17-21)")
    ap.add_argument("--wav", action="append", default=[], help="test=dosya.wav (gecikme icin ses kaydi)")
    ap.add_argument("--gecikme", default=str(GECIKME), help="A10 gecikme DR dosyasi")
    ap.add_argument("--cikti", default=None, help="rapor klasoru (varsayilan: kayit klasoru)")
    a = ap.parse_args(argv)
    if a.komut == "tablo":
        ss = load_sessions(a.klasor).get("ses-ogren")
        if not ss:
            print("ses-ogren kaydi yok:", a.klasor)
            return 2
        base = load_table(a.tablo) if a.ekle else None
        ids, warn = build_table(ss[-1], base)
        for w in warn:
            print("UYARI:", w)
        print("yazildi:", write_table(a.tablo, ids))
        print(" ".join("%d:%s" % kv for kv in sorted(ids.items())))
        return 0
    R = analyze(a.klasor, load_table(a.tablo), dict(w.split("=", 1) for w in a.wav), a.gecikme)
    out = Path(a.cikti or a.klasor)
    out.mkdir(parents=True, exist_ok=True)
    (out / "ses_rapor.json").write_text(json.dumps(R, indent=1, ensure_ascii=False, default=_jd), encoding="utf-8")
    (out / "ses_rapor.md").write_text(markdown(R), encoding="utf-8")
    for o in R["kapi"]["satirlar"]:
        print("%-45s %-12s %s" % (o["olcut"], o["sonuc"], o["deger"]))
    print("KAPI:", R["kapi"]["genel"], "->", out / "ses_rapor.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
