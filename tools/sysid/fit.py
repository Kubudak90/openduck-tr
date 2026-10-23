# -*- coding: utf-8 -*-
"""
Adim 0 sysID uydurma hatti: tezgah log'lari -> aktuator/algilayici parametreleri (rl/actuators.py icin JSON)
+ spec 7 karar tablosu raporu (report.py).

  python -m tools.sysid.fit --logs bench_logs/DS3230-1 --mass 1.95
  python -m tools.sysid.fit --logs bench_logs --mass 1.95 --knee-rms 1.05   # her alt klasor bir servo
Cikti: <logs>/sysid_params.json + <logs>/sysid_rapor.md (ya da --out/--report).

Siralama (her adim oncekilerin sonucunu kullanir; eksik test -> ilgili alanlar None):
  1 pot haritasi (potcal; yoksa sweep + nominal derece/us)       2 sweep: varlik, monotonluk, aralik, komut haritasi
  3 speed: yuksuz plato hizi                                       4 stall: tau_s (terazi), stall akimi (INA + multimetre)
  5 curpts: besleme akimi modeli (i_stall, i_idle), ortak-mod hata 6 ramp: frictionloss ilk tahmini (akim egimi)
  7 dinamik LM: p_band, armature, latency, deadband (step + chirp + ramp + deadband basamaklari; tau_s, b,
    frictionloss sabit; Latin hiperkup + LM + merkezi fark cilasi), sonra model tabanli frictionloss + i_idle
    (yavas ramp parcalarinin tik-ortalamali akimi) ve dinamik cila   8 track: 300 Hz / 3.3 V kabul
  9 overshoot: ray asimi   10 heat: T_diz   11 kapi: u(kappa), karar tablosu
Olu bant: model.simulate'teki hata olu bolgesi (rl/actuators.py dead_band, half = deadband/2; e = c - q her alt
adimda). Neden bu ayrim: yavas/duragan testlerde olu bant ile surtunme ayni izi birakir (histerezis genisligi
db + 2*tau_f/kp) ve plato hizinda tau_f, tau_s'ten ayrilamaz; surtunmeyi sabit hizdaki besleme akimi
(I ~ tau_act*tau_net) ayirir.
Aci: pot alt/ust ucu (ADS AIN2-3, ADC1 ph) bagliysa oranli/duzeltmeli okuma kullanilir; tek uclu okuma hizli
adimlarda akim kadar kayar (ortak-mod hata, ayrica raporlanir).
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from . import model as M
from .logio import CLASS_DEFAULTS, G, load_dir

MJCF_CLASS = dict(ds="ds32xx", es="es3054", mg="mg90s", sg="sg90")
D2R = math.pi / 180.0


# ---------------------------------------------------------------- yardimcilar
def _tail(run, t0, t1, frac=0.5, col=None):
    """[t0, t1) parcasinin son frac kismindaki ornek maskesi (ya da kolon ortalamasi)."""
    m = run.mask(t0 + (1.0 - frac) * (t1 - t0), t1)
    if col is None:
        return m
    v = run.d[col][m]
    v = v[np.isfinite(v)]
    return float(np.mean(v)) if len(v) else float("nan")


def _q_rest(c, q):
    """Pencere basindaki durgun konum: komut ilk kez degisene kadarki orneklerin ortalamasi (pencereler onceki
    tutmada baslar; gecikme + RC yuzunden bu ornekler hareketten oncedir). Hata olu bolgesinde durgun servo bant
    icinde herhangi bir yerde durabilir (yay geri cekmez): tek ornegin gurultusu hareketin baslangicini kaydirirdi."""
    c, q = np.asarray(c, float), np.asarray(q, float)
    ch = np.abs(c - c[0]) > 1e-12
    n = int(np.argmax(ch)) if ch.any() else len(c)
    return float(np.mean(q[:max(n, 1)]))


def _linfit(x, y):
    A = np.vstack([np.ones_like(x), x]).T
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    return float(c[0]), float(c[1])


def _f(v, sig=6):
    if v is None:
        return None
    if isinstance(v, (bool, np.bool_)):
        return bool(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    v = float(v)
    return None if not math.isfinite(v) else float("%.*g" % (sig, v))


class PotMap:
    """Pot pini mV -> aci (derece). Istege bagli oranli harita (x = (Vw - Vlo)/(Vhi - Vlo); pot ust ucu yoksa
    x = Vw - Vlo). Aktuator uydurmasi icin aci (deg_d) varsa oranli/alt-uc duzeltmeli okumadan gelir: tek uclu
    okuma, akimla kayan servo topragi yuzunden hizli adimlarda derecelerce sapar (ortak-mod hata)."""

    def __init__(self, a, b, src, ar=None, br=None, div=47.0 / 147.0, uses_ph=False, rng=None):
        self.a, self.b, self.src, self.ar, self.br, self.div = a, b, src, ar, br, div
        self.uses_ph, self.rng = uses_ph, rng

    def deg(self, mv):
        return self.a + self.b * np.asarray(mv, float)

    def rad(self, mv):
        return self.deg(mv) * D2R

    def x(self, pot_mv, ph_mv, lo_uv):
        vw = np.asarray(pot_mv, float) / self.div * 1e-3
        vlo = np.asarray(lo_uv, float) * 1e-6
        if not self.uses_ph:
            return vw - vlo
        vhi = np.asarray(ph_mv, float) / self.div * 1e-3
        return (vw - vlo) / (vhi - vlo)

    def deg_ratio(self, pot_mv, ph_mv, lo_uv):
        if self.ar is None:
            return None
        return self.ar + self.br * self.x(pot_mv, ph_mv, lo_uv)

    def corrected(self, d, m):
        if self.ar is None:
            return False
        ok = np.isfinite(d["adslo_uv"][m]).all()
        return bool(ok and (not self.uses_ph or np.isfinite(d["ph_mv"][m]).all()))

    def deg_d(self, d, m=slice(None)):
        """Log kolonlarindan en iyi aci tahmini (derece)."""
        if self.corrected(d, m):
            return self.deg_ratio(d["pot_mv"][m], d["ph_mv"][m], d["adslo_uv"][m])
        return self.deg(d["pot_mv"][m])

    def rad_d(self, d, m=slice(None)):
        return self.deg_d(d, m) * D2R


class CmdMap:
    """Komut us -> servonun ic hedef acisi (pot cercevesinde, derece)."""

    def __init__(self, c0, k, src):
        self.c0, self.k, self.src = c0, k, src

    def deg(self, us):
        return self.c0 + self.k * (np.asarray(us, float) - 1500.0)

    def rad(self, us):
        return self.deg(us) * D2R


# ---------------------------------------------------------------- 1 pot haritasi
def fit_potcal(run):
    ev = run.ev("potcal")
    if len(ev) < 2:
        return None
    deg = np.array([kv["deg"] for _, kv in ev], float)
    mv = np.array([kv["mv"] for _, kv in ev], float)
    a, b = _linfit(mv, deg)
    res = deg - (a + b * mv)
    cub = np.polyfit(mv, deg, 3).tolist() if len(ev) >= 5 else None
    div = run.cfg["div"]
    pm = PotMap(a, b, "potcal", div=div, rng=(float(deg.min()), float(deg.max())))
    out = dict(points=[dict(deg=float(d), mv=float(m)) for d, m in zip(deg, mv)], a_deg=a, b_deg_per_mv=b,
               scale_deg_per_v=b * 1000.0 * div, offset_v=(-a / b) / 1000.0 / div, nonlin_max_deg=float(np.abs(res).max()),
               cubic=cub, monotonic=bool(np.all(np.diff(mv[np.argsort(deg)]) * np.sign(b) > 0)))
    ph = np.array([kv.get("ph_mv", float("nan")) for _, kv in ev], float)
    lo = np.array([kv.get("adslo_uv", float("nan")) for _, kv in ev], float)
    if np.all(np.isfinite(lo)) and np.all(np.abs(lo) < 3e5):          # alt uc: birkac yuz mV'tan az
        vhi = ph / div * 1e-3
        pm.uses_ph = bool(np.all(np.isfinite(ph)) and np.all((vhi > 1.0) & (vhi < 9.0))
                          and np.ptp(vhi) < 0.05 * np.median(vhi))       # ust uc sabit besleme olmali
        x = pm.x(mv, ph, lo)
        pm.ar, pm.br = _linfit(x, deg)
        out["ratio_map"] = dict(a=pm.ar, b=pm.br, uses_ph=pm.uses_ph)
    return pm, out


# ---------------------------------------------------------------- 2 sweep
def fit_sweep(run, pm=None):
    cfg = run.cfg
    div = cfg["div"]
    cls = cfg.get("cls", "ds")
    st = [(t0, t1, kv) for t0, t1, kv in run.segs() if kv.get("kind") == "stair"]
    if len(st) < 6:
        return None, None, None
    us = np.array([float(kv["us"]) for _, _, kv in st])
    dr = np.array([int(kv["dir"]) for _, _, kv in st])
    mv = np.array([_tail(run, t0, t1, 0.5, "pot_mv") for t0, t1, _ in st])
    ph = np.array([_tail(run, t0, t1, 0.5, "ph_mv") for t0, t1, _ in st])
    sd = np.array([np.std(run.d["pot_mv"][_tail(run, t0, t1, 0.5)]) for t0, t1, _ in st])
    noise = float(np.median(sd)) if len(sd) else 1.0
    out = dict(noise_mv=noise)
    rng_mv = float(np.nanmax(mv) - np.nanmin(mv))
    out["present"] = bool(rng_mv > max(30.0, 20 * noise))
    out["wiper_v_min"] = float(np.nanmin(mv)) / div * 1e-3
    out["wiper_v_max"] = float(np.nanmax(mv)) / div * 1e-3
    out["span_v"] = rng_mv / div * 1e-3
    if np.any(np.isfinite(ph)) and 1.0 < float(np.nanmedian(ph)) / div * 1e-3 < 9.0:
        vs = float(np.nanmedian(ph)) / div * 1e-3
        out["pot_supply_v"] = vs
        vr = CLASS_DEFAULTS[cls]["vrail"]
        out["pot_supply"] = ("ray" if abs(vs - vr) < 0.35 else "5V" if abs(vs - 5.0) < 0.3 else
                             "3V3" if abs(vs - 3.3) < 0.25 else "bilinmiyor")
    # monotonluk ve hareketli bolge (yon basina)
    mono, moving = True, np.zeros(len(st), bool)
    for d in (1, -1):
        idx = np.where(dr == d)[0]
        if len(idx) < 3:
            continue
        o = idx[np.argsort(us[idx])]
        dm = np.diff(mv[o])
        s = np.sign(np.median(dm)) or 1.0
        bad = dm * s < -max(4.0 * noise, 2.0)
        if np.any(bad):
            mono = False
        mv_ok = dm * s > 0.3 * abs(np.median(dm))
        moving[o[1:]] |= mv_ok
        moving[o[:-1]] |= mv_ok
    out["monotonic"] = bool(mono and out["present"])
    src = "potcal"
    if pm is None:
        # aciolcer yok: nominal derece/us ile olcek (uyari)
        dpu = CLASS_DEFAULTS[cls]["deg_per_us"]
        m = moving
        a, b = _linfit(mv[m], (us[m] - 1500.0) * dpu)
        pm = PotMap(a, b, "nominal", div=div)
        src = "nominal"
    q = np.array([float(np.mean(pm.deg_d(run.d, _tail(run, t0, t1, 0.5)))) for t0, t1, _ in st])
    # aciolcer araligi disinda dogrusal harita guvenilmez: komut haritasi ve play blogu aralik icinden
    inr = moving.copy()
    if pm.rng:
        inr &= (q >= pm.rng[0] - 2.0) & (q <= pm.rng[1] + 2.0)
    # komut haritasi: ayni us'de yukari/asagi ortasi
    ups = {round(u, 2): (qq, mm) for u, qq, mm, d, mo in zip(us, q, mv, dr, inr) if d == 1 and mo}
    dns = {round(u, 2): (qq, mm) for u, qq, mm, d, mo in zip(us, q, mv, dr, inr) if d == -1 and mo}
    common = sorted(set(ups) & set(dns))
    if len(common) >= 3:
        uc = np.array(common)
        mid = np.array([(ups[u][0] + dns[u][0]) / 2 for u in common])
        hy = np.array([dns[u][0] - ups[u][0] for u in common])
        c0, k = _linfit(uc - 1500.0, mid)
        res = mid - (c0 + k * (uc - 1500.0))
        cm = CmdMap(c0, k, src)
        out.update(deg_per_us=k, us_zero=1500.0 - c0 / k, hyst_deg=float(np.median(hy)),
                   cmd_nonlin_max_deg=float(np.abs(res).max()), travel_deg=float(np.ptp(q[moving])),
                   travel_us=float(np.ptp(us[moving])))
    else:
        cm = None
    out["scale_src"] = src
    out["stairs"] = dict(us=us, dir=dr, mv=mv, q=q, inrange=inr)
    return pm, cm, out


# ---------------------------------------------------------------- 3 speed
def fit_speed(run, pm):
    w = []
    segs = [(t0, t1, kv) for t0, t1, kv in run.segs() if kv.get("kind") == "step"]
    for t0, t1, kv in segs:
        m = run.mask(t0 - 0.01, t1)
        t, q = run.t[m], pm.rad_d(run.d, m)
        if len(t) < 20:
            continue
        qs = float(np.mean(q[:3]))
        qe = float(np.mean(q[-max(3, len(q) // 5):]))
        if abs(qe - qs) < 30 * D2R:
            continue
        f = (q - qs) / (qe - qs)
        s = (f > 0.2) & (f < 0.8)
        if s.sum() < 5:
            continue
        _, sl = _linfit(t[s], q[s])
        w.append(abs(sl))
    if not w:
        return None
    w = np.array(w)
    om = float(np.median(w))
    return dict(omega_noload=om, t60_s=(math.pi / 3) / om, n=len(w), spread=float(np.ptp(w) / om))


# ---------------------------------------------------------------- 4 stall
def fit_stall(run, pm, cm):
    cfg = run.cfg
    L = cfg["lever_mm"] / 1000.0
    ans = run.meta.get("answers", {})
    pts, meter = [], None
    for t0, t1, kv in run.segs():
        if kv.get("kind") != "stall":
            continue
        m = _tail(run, t0, t1, 0.4)
        I = float(np.mean(run.d["i_ma"][m])) * 1e-3
        mv = float(np.mean(run.d["pot_mv"][m]))
        dq = float(cm.deg(float(kv["us"])) - np.mean(pm.deg_d(run.d, m))) if cm is not None else \
            float(kv.get("delta_deg"))
        if str(kv.get("i")) == "meter":
            meter = dict(i_ina=I, i_meter=_num(ans.get("meter_a")))
            continue
        g = _num(ans.get("g_%s" % kv.get("i")))
        if g is None:
            continue
        pts.append(dict(delta_cmd_deg=float(kv.get("delta_deg")), delta_deg=dq, tau=g / 1000.0 * G * L, i=I,
                        mv=mv, ph_mv=_tail(run, t0, t1, 0.4, "ph_mv"), lo_uv=_tail(run, t0, t1, 0.4, "adslo_uv")))
    if not pts:
        return None
    tau = np.array([p["tau"] for p in pts])
    sat = tau >= 0.97 * tau.max()
    out = dict(tau_s=float(np.median(tau[sat])), points=pts, i_sat_ina=float(np.median([p["i"] for p, s in
                                                                                         zip(pts, sat) if s])))
    un = (~sat) & (tau > 0.05 * tau.max())
    if un.sum() >= 2:
        d = np.array([p["delta_deg"] for p in pts])
        c0, k = _linfit(d[un], tau[un])
        out["kp_check"] = k / D2R
        out["p_band_check_deg"] = out["tau_s"] / k
    if meter:
        out["meter"] = meter
        if meter["i_meter"]:
            out["meter"]["diff_frac"] = abs(meter["i_ina"] - meter["i_meter"]) / meter["i_meter"]
    return out


def _num(v):
    try:
        f = float(v)
        return f if math.isfinite(f) else None
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------- 5 besleme akimi + ortak mod
def _lever(cfg):
    L = cfg["lever_mm"] / 1000.0
    m, ml = cfg.get("mass_g", 0.0) / 1000.0, cfg.get("lever_g", 0.0) / 1000.0
    return G * (m * L + ml * L / 2.0), m * L * L + ml * L * L / 3.0


def _ref(run, pm, name):
    """Referans konumu (kol yatay vb.): olaydan onceki 0.4 s'lik satirlardan (duzeltmeli aci)."""
    for t, kv in run.ev("ref"):
        if kv.get("name") == name:
            m = run.mask(t - 0.4, t)
            if m.sum() >= 3:
                return float(np.mean(pm.rad_d(run.d, m)))
            return float(pm.rad(float(kv["mv"])))
    return None


def fit_idle(runs):
    """Bosta akim: yuksuz tutma parcalari (ramp/speed 'hold'); tutma torku ~ surtunme kalintisi."""
    v = []
    for r in runs:
        for t0, t1, kv in r.segs():
            if kv.get("kind") == "hold" and t1 - t0 > 0.2:
                v.append(_tail(r, t0, t1, 0.6, "i_ma") * 1e-3)
    v = [x for x in v if math.isfinite(x)]
    return float(np.median(v)) if v else None


def fit_current(run, pm, tau_s, i_idle=None):
    mgl, _ = _lever(run.cfg)
    qh = _ref(run, pm, "horiz")
    if qh is None or mgl <= 0:
        return None
    pts = []
    for t0, t1, kv in run.segs():
        if kv.get("kind") != "static":
            continue
        m = _tail(run, t0, t1, 0.6)
        q = float(np.mean(pm.rad_d(run.d, m)))
        pts.append(dict(ang_deg=(q - qh) / D2R, tau=abs(mgl * math.cos(q - qh)), i=float(np.mean(run.d["i_ma"][m])) * 1e-3,
                        mv=float(np.mean(run.d["pot_mv"][m])), ph_mv=_tail(run, t0, t1, 0.6, "ph_mv"),
                        lo_uv=_tail(run, t0, t1, 0.6, "adslo_uv")))
    if len(pts) < 2:
        return None
    x = np.array([p["tau"] ** 2 for p in pts])
    y = np.array([p["i"] for p in pts])
    if not np.all(np.isfinite(y)) or not np.all(np.isfinite(x)):
        return None                       # INA226 yok ya da pot okunmuyor
    if i_idle is not None:
        i0, c2 = i_idle, float(np.sum((y - i_idle) * x) / np.sum(x * x))
    else:
        i0, c2 = _linfit(x, y)
    yp = i0 + c2 * x
    r2 = 1.0 - float(np.sum((y - yp) ** 2) / max(np.sum((y - y.mean()) ** 2), 1e-12))
    out = dict(c2_a_per_nm2=c2, i_idle=i0, r2=r2, points=pts, mgl=mgl)
    if tau_s:
        out["i_stall"] = c2 * tau_s ** 2
    return out


def fit_common_mode(pm, pts, i_ref):
    """Ortak-mod hata: tek uclu aci - oranli aci, akima karsi (derece/A)."""
    if pm is None or pm.ar is None:
        return None
    e, I = [], []
    for p in pts:
        dr = pm.deg_ratio(p["mv"], p.get("ph_mv", float("nan")), p.get("lo_uv", float("nan")))
        if dr is None or not np.isfinite(dr):
            continue
        e.append(float(pm.deg(p["mv"]) - dr))
        I.append(p["i"])
    if len(e) < 3:
        return None
    I, e = np.array(I), np.array(e)
    c0, k = _linfit(I, e)
    return dict(slope_deg_per_a=k, offset_deg=c0, i_ref=i_ref, err_at_ref_deg=abs(k * i_ref + c0),
                max_abs_deg=float(np.abs(e).max()), n=len(e))


# ---------------------------------------------------------------- 6 surtunme (ramp akimi)
def fit_friction(run, pm, tau_s, omega_p, i_stall, i_idle):
    segs = []
    for t0, t1, kv in run.segs():
        if kv.get("kind") != "ramp":
            continue
        a, b_ = t0 + 0.2 * (t1 - t0), t0 + 0.8 * (t1 - t0)
        m = run.mask(a, b_)
        if m.sum() < 5:
            continue
        _, sl = _linfit(run.t[m], pm.rad_d(run.d, m))
        segs.append((abs(sl), float(np.mean(run.d["i_ma"][m])) * 1e-3, float(kv.get("speed_dps", 0))))
    if not segs or not (tau_s and omega_p and i_stall):
        return dict(segs=segs) if segs else None
    # Komut basamakli (tik ya da PCA cercevesi, hangisi seyrekse): tau_act testere disi genligi A = kp*w*dt/2
    # akim carpimini A^2/3 kadar sisirir. Yalniz sapmasi < %2 olan (yavas) parcalar kullanilir.
    dtc = max(1.0 / float(run.params.get("rate") or run.cfg.get("rate", 500)), 1.0 / float(run.cfg.get("freq", 300)))
    kp0 = tau_s / (3.0 * D2R)
    # Sabit hizda I = [i_idle + i_stall*tf^2/ts^2] + [i_stall*b*tf/ts^2]*w: egim tf'yi bosta akimdan bagimsiz verir
    # (tutma akimi, takilan surtunme torku yuzunden bosta akimi yukari kaydirir); kesisim i_idle'i verir.
    ws = sorted(set(round(s[0], 3) for s in segs))
    used = [s for s in segs if round(s[0], 3) in ws[:3]]
    tf, i0 = None, None
    for _ in range(30):
        w = np.array([s[0] for s in used])
        if len(set(np.round(w, 3))) < 2:
            return dict(segs=segs)
        s0, s1 = _linfit(w, np.array([s[1] for s in used]))
        b = (tau_s - (tf or 0.0)) / omega_p
        tn = s1 * tau_s ** 2 / (i_stall * b)
        if tn <= 0:
            return dict(segs=segs)
        i0 = s0 - i_stall * tn ** 2 / tau_s ** 2
        nu = [s for s in segs if (kp0 * s[0] * dtc / 2) ** 2 / 3 <= 0.02 * (b * s[0] + tn) * tn]
        if len(set(round(s[0], 3) for s in nu)) >= 2:
            used = nu
        if tf is not None and abs(tn - tf) < 1e-8:
            tf = tn
            break
        tf = tn
    b = (tau_s - tf) / omega_p
    return dict(frictionloss=tf, damping=b, omega0=tau_s / b, i_idle=i0, segs=segs, n_used=len(used),
                speeds_used=sorted(set(round(s[2], 2) for s in used)), method="egim")


def fit_friction_model(run, pm, cm, p, omega_p, i_stall, tf0, rc_tau, dt=2.5e-4):
    """Model tabanli surtunme: her ramp parcasinin olculen ortalama akimini, ayni modelin (dinamik uydurmadan
    p_band/armature/latency/deadband) tik-ortalamali besleme akimiyla eslestir. Bilinmeyenler tf ve i_idle;
    b = (tau_s - tf)/plato hizi baglantisi korunur. Altin oran arama (log tf), i_idle dogrusal."""
    segs, wins = [], []
    rs = run.segs()
    dtc = max(1.0 / float(run.params.get("rate") or run.cfg.get("rate", 500)), 1.0 / float(run.cfg.get("freq", 300)))
    b0 = (p.tau_s - tf0) / omega_p
    cand = []
    for k, (t0, t1, kv) in enumerate(rs):
        if kv.get("kind") != "ramp":
            continue
        a, b_ = t0 + 0.2 * (t1 - t0), t0 + 0.8 * (t1 - t0)
        m = run.mask(a, b_)
        if m.sum() < 5:
            continue
        w = abs(float(kv.get("speed_dps", 0))) * D2R
        # testere payi (A^2/3) surtunme payina gore kucuk olan parcalar: dinamik parametre hatasina duyarsiz
        share = (p.kp * w * dtc / 2) ** 2 / 3 / max((b0 * w + tf0) * tf0, 1e-12)
        cand.append((share, t0, t1, kv, a, b_, m))
    use = [c for c in cand if c[0] <= 0.10]
    if len(set(c[3].get("speed_dps") for c in use)) < 2:
        use = sorted(cand, key=lambda c: c[0])[:6]
    for share, t0, t1, kv, a, b_, m in use:
        wmask = run.mask(t0 - 0.05, t1)
        c = cm.rad(run.d["cmd_us"][wmask])
        wins.append((run.t[wmask], c, _q_rest(c, pm.rad_d(run.d, wmask)), a, b_))
        segs.append(float(np.mean(run.d["i_ma"][m])) * 1e-3)
    if len(segs) < 2:
        return None
    I_meas = np.array(segs)

    def model_I(tf):
        q = M.ServoParams(**{**p.dict(), "frictionloss": tf, "i_idle": 0.0, "i_stall": i_stall})
        b = (q.tau_s - tf) / omega_p
        q.omega0 = q.tau_s / b
        out = []
        for t, c, q0, a, b_ in wins:
            # pencere onceki tutmada baslar: onceki komut c[0]; olu bant durumu yalniz durgun q0'dan (_q_rest)
            s = M.simulate(q, t, c, t, q0=q0, c_prev=float(c[0]), dt=dt, rc_tau=rc_tau)
            sel = (t >= a) & (t < b_)
            out.append(float(np.mean(s["i_avg"][sel])))
        return np.array(out)

    def cost(ltf):
        mi = model_I(math.exp(ltf))
        i0 = float(np.mean(I_meas - mi))
        return float(np.sum((I_meas - mi - i0) ** 2)), i0

    lo, hi = math.log(max(tf0, 1e-4) / 4), math.log(max(tf0, 1e-4) * 4)
    g = (math.sqrt(5) - 1) / 2
    for _ in range(40):
        c1, c2 = hi - g * (hi - lo), lo + g * (hi - lo)
        if cost(c1)[0] < cost(c2)[0]:
            hi = c2
        else:
            lo = c1
    ltf = 0.5 * (lo + hi)
    c, i0 = cost(ltf)
    tf = math.exp(ltf)
    b = (p.tau_s - tf) / omega_p
    return dict(frictionloss=tf, damping=b, omega0=p.tau_s / b, i_idle=i0, n_segs=len(segs), method="model",
                rms_ma=1e3 * math.sqrt(c / len(segs)))


# ---------------------------------------------------------------- 7 dinamik LM
class Dyn:
    """Dinamik ve yari-duragan artik bloklari; parametreler log uzayinda [p_band, armature, latency, deadband]."""

    def __init__(self, fixed, rc_tau, dt=2.5e-4):
        self.fx = fixed            # tau_s, damping, frictionloss
        self.rc = rc_tau
        self.dt = dt
        self.blocks = []           # (tur, veri)

    def add_dyn(self, t, c, q, sigma, w=1.0, dec=1):
        self.blocks.append(("dyn", dict(t=t, c=c, q=q, q0=_q_rest(c, q), sig=sigma, w=w, dec=dec)))

    def add_play(self, c, q, sigma):
        self.blocks.append(("play", dict(c=c, q=q, sig=sigma)))

    def params(self, x):
        pb, J, L, db = np.exp(x)
        p = M.ServoParams(tau_s=self.fx["tau_s"], omega0=self.fx["tau_s"] / self.fx["damping"], p_band=pb,
                          deadband=db, armature=J, frictionloss=self.fx["frictionloss"], latency=L)
        return p

    def resid(self, x, kinds=("dyn", "play")):
        p = self.params(x)
        out = []
        # yari-duragan blok: hata olu bolgesi + Coulomb surtunmesi statikte play(c, 2h) ile esdegerdir. Servo
        # kp*(|e| - db/2) < frictionloss iken takili kalir, yani |c - q| < h = db/2 + frictionloss/kp; komut
        # 1 sayim (asimsiz) ilerledikce q, |c - q| = h sinirinda komutun arkasindan gelir -> genislik 2h.
        h = 0.5 * p.deadband + p.frictionloss / p.kp
        for kind, b in self.blocks:
            if kind not in kinds:
                continue
            if kind == "dyn":
                # pencereler onceki tutmada baslar (onceki komut c[0]); olu bant gecmisi yok; durum: durgun q0
                s = M.simulate(p, b["t"], b["c"], b["t"], q0=b["q0"], c_prev=float(b["c"][0]),
                               dt=self.dt, rc_tau=self.rc)
                r = (s["q_rc"] - b["q"])[::b["dec"]] / b["sig"] * math.sqrt(b["w"])
            else:
                y = M.play(b["c"], 2.0 * h, y0=float(b["q"][0]))
                d = b["q"] - y
                r = (d - np.mean(d)) / b["sig"]
            out.append(r)
        return np.concatenate(out) if out else np.zeros(0)


def lm(fun, x0, it=40, lo=None, hi=None, h=1e-2, central=False, h_min=2.5e-4):
    """Levenberg-Marquardt (sayisal Jacobian). Maliyet simulasyondan geldigi icin hafif puruzlu: adim bulunamazsa
    fark adimi h kuculur (h_min'e kadar); central=True merkezi fark (cilalama)."""
    x = np.array(x0, float)
    r = fun(x)
    cost = float(r @ r)
    lam = 1e-2
    Jm = None
    for _ in range(it):
        Jm = np.empty((len(r), len(x)))
        for j in range(len(x)):
            xp = x.copy()
            xp[j] += h
            if central:
                xm = x.copy()
                xm[j] -= h
                Jm[:, j] = (fun(xp) - fun(xm)) / (2 * h)
            else:
                Jm[:, j] = (fun(xp) - r) / h
        A, g = Jm.T @ Jm, Jm.T @ r
        improved, rel = False, 0.0
        while lam < 1e9:
            dx = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
            xn = x + dx
            if lo is not None:
                xn = np.clip(xn, lo, hi)
            rn = fun(xn)
            cn = float(rn @ rn)
            if cn < cost:
                rel = (cost - cn) / max(cost, 1e-30)
                x, r, cost = xn, rn, cn
                lam = max(lam / 3.0, 1e-7)
                improved = True
                break
            lam *= 4.0
        if not improved:
            if h / 4.0 < h_min:
                break
            h, lam = h / 4.0, 1e-2
            continue
        if rel < 1e-9:
            break
    try:
        cov = np.linalg.inv(Jm.T @ Jm) * cost / max(len(r) - len(x), 1)
        se = np.sqrt(np.clip(np.diag(cov), 0, None))
    except np.linalg.LinAlgError:
        se = np.full(len(x), np.nan)
    return x, cost, se


def build_dyn(runs, pm, cm, fixed, rc_tau, noise_deg):
    dyn = Dyn(fixed, rc_tau)
    sig = max(noise_deg, 0.02) * D2R
    for r in runs:
        segs = r.segs()
        if r.program == "step":
            for k, (t0, t1, kv) in enumerate(segs):
                if kv.get("kind") != "step":
                    continue
                a, b = t0 - 0.03, min(t1, t0 + 0.35)
                m = r.mask(a, b)
                if m.sum() < 20:
                    continue
                dyn.add_dyn(r.t[m], cm.rad(r.d["cmd_us"][m]), pm.rad_d(r.d, m), sig)
        elif r.program == "chirp":
            for t0, t1, kv in segs:
                if kv.get("kind") != "chirp":
                    continue
                m = r.mask(t0 - 0.2, t1)
                dyn.add_dyn(r.t[m], cm.rad(r.d["cmd_us"][m]), pm.rad_d(r.d, m), sig, dec=2)
        elif r.program == "ramp":
            for k, (t0, t1, kv) in enumerate(segs):
                if kv.get("kind") != "ramp":
                    continue
                t2 = segs[k + 1][1] if k + 1 < len(segs) else t1
                m = r.mask(t0 - 0.05, min(t2, t1 + 0.3))
                # yon degisimi: komut bant (db/2 + tf/kp) disina cikana kadar servo durur (baslangic: durgun q0)
                dyn.add_dyn(r.t[m], cm.rad(r.d["cmd_us"][m]), pm.rad_d(r.d, m), sig, dec=2)
        elif r.program == "deadband":
            # 1 sayimlik basamaklar asim yapmaz: yari-duragan play esdegeri gecerli (sweep'in 10 us basamaklari degil)
            st = [(t0, t1, kv) for t0, t1, kv in segs if kv.get("kind") == "stair"]
            if len(st) < 6:
                continue
            c = np.array([cm.rad(float(kv["us"])) for _, _, kv in st])
            q = np.array([float(np.mean(pm.rad_d(r.d, _tail(r, t0, t1, 0.5)))) for t0, t1, _ in st])
            if pm.rng:
                ok = (q >= (pm.rng[0] - 2.0) * D2R) & (q <= (pm.rng[1] + 2.0) * D2R)
                c, q = c[ok], q[ok]
            if len(q) < 6:
                continue
            n = max(1, int(np.median([_tail(r, t0, t1, 0.5).sum() for t0, t1, _ in st])))
            dyn.add_play(c, q, max(sig / math.sqrt(n), 0.02 * D2R))
    return dyn


def fit_dynamic(dyn, init):
    if not any(k == "dyn" for k, _ in dyn.blocks):
        return None
    lo = np.log([0.1 * D2R, 1e-5, 2e-4, 0.005 * D2R])
    hi = np.log([30 * D2R, 0.1, 0.08, 5.0 * D2R])
    # vadi duz ve egik (p_band - armature - latency birbirini telafi eder): once kaba arama (sabit tohumlu
    # Latin hiperkup, makul aralik), en iyi 3 noktadan LM; maliyet ucuz (~20 ms/degerlendirme)
    rng = np.random.default_rng(0)
    box_lo = np.log([0.8 * D2R, 3e-4, 1e-3, 0.03 * D2R])
    box_hi = np.log([10 * D2R, 2e-2, 0.03, 1.5 * D2R])
    n = 64
    u = (np.argsort(rng.random((4, n)), axis=1).T + rng.random((n, 4))) / n
    cand = [np.log([init["p_band"], 3e-3, 6e-3, init["deadband"]])] + list(box_lo + u * (box_hi - box_lo))
    sc = []
    for x0 in cand:
        r = dyn.resid(x0)
        sc.append((float(r @ r), x0))
    sc.sort(key=lambda s: s[0])
    best = None
    for _, x0 in sc[:3]:
        x, c, se = lm(dyn.resid, x0, it=40, lo=lo, hi=hi)
        if best is None or c < best[1]:
            best = (x, c, se)
    x, cost, se = lm(dyn.resid, best[0], it=30, lo=lo, hi=hi, h=2e-3, central=True)
    pb, J, L, db = np.exp(x)
    n = len(dyn.resid(x))
    return dict(p_band=pb, armature=J, latency=L, deadband=db, rel_se=dict(zip(
        ("p_band", "armature", "latency", "deadband"), [float(s) for s in se])), rms_resid=math.sqrt(cost / max(n, 1)))


# ---------------------------------------------------------------- 8 track (300 Hz / 3.3 V)
def fit_track(run, pm, cm):
    segs = run.segs()
    out = dict(freq=run.params.get("freq"), tag=run.tag)
    rat, prev = [], None
    for t0, t1, kv in segs:
        k = kv.get("kind")
        if k == "step":
            q = float(np.mean(pm.deg_d(run.d, _tail(run, t0, t1, 0.3))))
            c = float(cm.deg(float(kv["us"])))
            if prev is not None and abs(c - prev[0]) > 1.0:
                rat.append((q - prev[1]) / (c - prev[0]))
            prev = (c, q)
        elif k == "sin":
            m = run.mask(t0 + 0.5, t1)
            t, q, c = run.t[m], pm.deg_d(run.d, m), cm.deg(run.d["cmd_us"][m])
            best = None
            for lag in np.arange(0.0, 0.15, 0.002):
                ci = np.interp(t - lag, t, c)
                e = q - ci
                rms = float(np.sqrt(np.mean((e - e.mean()) ** 2)))
                if best is None or rms < best[0]:
                    best = (rms, lag)
            out["sin_rms_deg"], out["sin_lag_s"] = best
    if rat:
        out["step_ratio"] = float(np.median(rat))
    hs = [s for s in segs if s[2].get("kind") == "hold"]
    if hs:
        t0, t1, _ = hs[-1]
        m = _tail(run, t0, t1, 0.7)
        out["hold_std_deg"] = float(np.std(pm.deg_d(run.d, m)))
        out["hold_i"] = float(np.mean(run.d["i_ma"][m])) * 1e-3
    ans = {k: v for k, v in run.meta.get("answers", {}).items()}
    out["user_ok"] = str(ans.get("obs", "e")).lower().startswith(("e", "y", "ok", "1"))
    out["follows"] = bool(out.get("step_ratio", 0) >= 0.8 and out.get("sin_rms_deg", 99) <= 2.0 and out["user_ok"])
    return out


def accept_300(t50, t300):
    if not (t50 and t300):
        return None
    ok = (t300["follows"] and t300.get("sin_rms_deg", 99) <= max(1.5 * t50.get("sin_rms_deg", 0), t50.get("sin_rms_deg", 0) + 0.5)
          and t300.get("hold_std_deg", 0) <= 2.0 * t50.get("hold_std_deg", 0) + 0.1
          and t300.get("hold_i", 0) <= 1.5 * t50.get("hold_i", 0) + 0.05)
    return bool(ok)


# ---------------------------------------------------------------- 9 overshoot
def fit_overshoot(run):
    if run.burst is None:
        return None
    vmax, vmin, per = -1.0, 99.0, []
    tb, mv = run.burst[:, 0] * 1e-6, run.burst[:, 1]
    evs = run.ev("burst")
    for k, (t0, kv) in enumerate(evs):
        t1 = t0 + kv["n"] * kv["dt_us"] * 1e-6
        m = (tb >= t0 - 1e-6) & (tb <= t1)
        if not m.any():
            continue
        g = kv["vbus0_mv"] / kv["v0_mv"]
        v = mv[m] * g * 1e-3
        per.append(dict(i=kv["i"], v_max=float(v.max()), v_min=float(v.min()), v0=kv["vbus0_mv"] * 1e-3))
        vmax, vmin = max(vmax, float(v.max())), min(vmin, float(v.min()))
    return dict(v_max=vmax, v_min=vmin, bursts=per) if per else None


# ---------------------------------------------------------------- 10 isinma (T_diz)
def _harm(t, q, f, nh=3):
    cols = [np.ones_like(t)]
    for h in range(1, nh + 1):
        w = 2 * math.pi * f * h
        cols += [np.sin(w * t), np.cos(w * t)]
    A = np.vstack(cols).T
    c, *_ = np.linalg.lstsq(A, q, rcond=None)
    qq, qd, qdd = np.full_like(t, c[0]), np.zeros_like(t), np.zeros_like(t)
    for h in range(1, nh + 1):
        w = 2 * math.pi * f * h
        a, b = c[2 * h - 1], c[2 * h]
        s, co = np.sin(w * t), np.cos(w * t)
        qq += a * s + b * co
        qd += w * (a * co - b * s)
        qdd += -w * w * (a * s + b * co)
    return qq, qd, qdd


def fit_heat(run, pm, armature, frictionloss, level_ref_s=600.0, t_ref=25.0, t_lim=60.0):
    mgl, JL = _lever(run.cfg)
    qh = _ref(run, pm, "horiz")
    levels = []
    amb_ans = _num(run.meta.get("answers", {}).get("amb"))
    for t0, t1, kv in run.segs():
        if kv.get("kind") != "heat":
            continue
        m = run.mask(t0 + 1.0, t1)
        t = run.t[m] - t0
        T = run.d["ntc_c"][m]
        amb = run.d["amb_c"][m]
        Ta = float(np.nanmean(amb)) if np.any(np.isfinite(amb)) else (amb_ans or t_ref)
        qq, qd, qdd = _harm(t, pm.rad_d(run.d, m), float(kv["f_hz"]))
        J = (armature or 0.0) + JL
        tn = J * qdd + (frictionloss or 0.0) * np.sign(qd) + mgl * np.cos(qq - (qh or 0.0))
        good = np.isfinite(T)
        levels.append(dict(lvl=kv.get("lvl"), tau_target=float(kv.get("tau_nm")), rms=float(np.sqrt(np.mean(tn ** 2))),
                           t=t[good], T=T[good], t_amb=Ta, t_max=float(np.nanmax(T)), dur_s=float(t[-1])))
    if not levels:
        return None

    def cost_tau(tau):
        c, sol = 0.0, []
        for lv in levels:
            e = np.exp(-lv["t"] / tau)
            A = np.vstack([1 - e, e]).T
            y = lv["T"] - lv["t_amb"]
            x, *_ = np.linalg.lstsq(A, y, rcond=None)
            c += float(np.sum((A @ x - y) ** 2))
            sol.append(x)
        return c, sol

    lo, hi = math.log(20.0), math.log(5000.0)
    g = (math.sqrt(5) - 1) / 2
    a, b = lo, hi
    for _ in range(60):
        c1, c2 = b - g * (b - a), a + g * (b - a)
        if cost_tau(math.exp(c1))[0] < cost_tau(math.exp(c2))[0]:
            b = c2
        else:
            a = c1
    tau = math.exp(0.5 * (a + b))
    _, sol = cost_tau(tau)
    k10 = 1.0 - math.exp(-level_ref_s / tau)
    for lv, x in zip(levels, sol):
        lv["dT_ss"] = float(x[0])
        lv["T10_25"] = t_ref + float(x[0]) * k10
        lv["pass_60"] = bool(lv["T10_25"] <= t_lim)
    rms = np.array([lv["rms"] for lv in levels])
    d10 = np.array([lv["T10_25"] - t_ref for lv in levels])
    need = t_lim - t_ref
    if len(levels) >= 2:
        beta, alpha = _linfit(rms ** 2, d10)
    else:
        alpha, beta = float(d10[0] / rms[0] ** 2), 0.0
    tk = math.sqrt((need - beta) / alpha) if alpha > 0 and need > beta else None
    out = dict(tau_th_s=tau, alpha=alpha, beta=beta, t_knee=tk,
               extrapolated=bool(tk is None or tk < rms.min() or tk > rms.max()),
               levels=[{k: v for k, v in lv.items() if k not in ("t", "T")} for lv in levels])
    if alpha > 0 and need > beta:
        # kararli durum (10 dk yerine sonsuz) tutucu bilgi
        a_ss, b_ss = alpha / k10, beta / k10
        out["t_knee_ss"] = math.sqrt((need - b_ss) / a_ss) if need > b_ss else None
    return out


# ---------------------------------------------------------------- 11 kapi
def gate(tau_s, omega_noload, mass, kappas=(0.72, 0.6, 0.5), u_max=0.85):
    if not (tau_s and omega_noload):
        return None
    u = {str(k): M.u_fit(tau_s, omega_noload, mass, k) for k in kappas}
    kap = next((k for k in kappas if u[str(k)] <= u_max), None)
    return dict(mass_kg=mass, u=u, kappa=kap, u_max=u_max, gait_freq_hz=(2.21 * kap) if kap else None)


# ---------------------------------------------------------------- hat
def fit_servo(path, mass=1.95, knee_rms=None, verbose=False):
    allr = load_dir(path)
    runs = [r for r in allr if str(r.meta.get("status")) == "ok"]     # yarida kesilen kosular kullanilmaz
    if not runs:
        raise SystemExit("basarili kosu yok: %s" % path)
    meta0 = runs[0].meta
    cls = runs[0].cfg.get("cls", "ds")
    by = {}
    for r in runs:
        by.setdefault(r.program, []).append(r)
    res = dict(schema="openduck-tr/sysid/v1", servo=dict(id=meta0.get("servo_id", Path(path).name), cls=cls,
               mjcf_class=MJCF_CLASS.get(cls), model=meta0.get("servo_model"), files=[r.path.name for r in runs],
               skipped=[r.path.name for r in allr if r not in runs]))
    log = (lambda *a: print(*a, file=sys.stderr)) if verbose else (lambda *a: None)
    # 1-2
    pm, pc = None, None
    if "potcal" in by:
        got = fit_potcal(by["potcal"][-1])
        if got:
            pm, pc = got
    cm, sw = None, None
    if "sweep" in by:
        pm2, cm, sw = fit_sweep(by["sweep"][-1], pm)
        pm = pm or pm2
    res["pot"] = dict(potcal=pc, sweep={k: v for k, v in (sw or {}).items() if k != "stairs"} or None)
    if pm is None or cm is None:
        res["error"] = "pot haritasi kurulamadi (sweep yok ya da pot okunmuyor)"
        return res
    log("pot", pm.src, cm.k)
    # 3-6
    sp = fit_speed(by["speed"][-1], pm) if "speed" in by else None
    stl = fit_stall(by["stall"][-1], pm, cm) if "stall" in by else None
    tau_s = stl["tau_s"] if stl else None
    om_p = sp["omega_noload"] if sp else None
    # akim modeli <-> surtunme: once serbest LS, sonra ramp egiminden tf ve bosta akim, sonra sabit bosta akimla
    cur = fit_current(by["curpts"][-1], pm, tau_s) if "curpts" in by else None
    fr = None
    for _ in range(2):
        i_stall = cur.get("i_stall") if cur else None
        fr = fit_friction(by["ramp"][-1], pm, tau_s, om_p, i_stall, None) if "ramp" in by else None
        idle = (fr or {}).get("i_idle") or fit_idle(by.get("ramp", []) + by.get("speed", []))
        if "curpts" in by:
            cur = fit_current(by["curpts"][-1], pm, tau_s, idle)
    i_stall = cur.get("i_stall") if cur else None
    i_idle = cur["i_idle"] if cur else None
    tf = fr.get("frictionloss") if fr else None
    b = fr.get("damping") if fr and tf is not None else ((tau_s / om_p) if (tau_s and om_p) else None)
    log("tau_s", tau_s, "om_p", om_p, "tf0", tf, "b", b)
    # 7 dinamik uydurma; sonra model tabanli surtunme (ramp akimlari) ve dinamik cilalama
    dynres = None
    rc_tau = runs[0].cfg["rc_ms"] * 1e-3
    if tau_s and b:
        noise_deg = abs(sw["noise_mv"] * pm.b) if sw else 0.1
        dyn = build_dyn(runs, pm, cm, dict(tau_s=tau_s, damping=b, frictionloss=tf or 0.0), rc_tau, noise_deg)
        w = (sw or {}).get("hyst_deg") or 0.4
        kp0 = tau_s / (3 * D2R)
        db0 = max((w - 2 * (tf or 0.0) / kp0 / D2R) * D2R, 0.05 * D2R)
        dynres = fit_dynamic(dyn, dict(p_band=3 * D2R, deadband=db0))
        if dynres and tf and i_stall and "ramp" in by:
            p_dyn = M.ServoParams(tau_s=tau_s, omega0=tau_s / b, p_band=dynres["p_band"], deadband=dynres["deadband"],
                                  armature=dynres["armature"], frictionloss=tf, latency=dynres["latency"])
            frm = fit_friction_model(by["ramp"][-1], pm, cm, p_dyn, om_p, i_stall, tf, rc_tau)
            if frm:
                fr = dict(fr, **frm, slope_frictionloss=tf)
                tf, b = frm["frictionloss"], frm["damping"]
                if "curpts" in by:
                    cur = fit_current(by["curpts"][-1], pm, tau_s, frm["i_idle"])
                    i_stall = cur.get("i_stall") if cur else i_stall
                    i_idle = cur["i_idle"] if cur else i_idle
                dyn.fx = dict(tau_s=tau_s, damping=b, frictionloss=tf)
                x0 = np.log([dynres["p_band"], dynres["armature"], dynres["latency"], dynres["deadband"]])
                x, cost, se = lm(dyn.resid, x0, it=20, h=2e-3, central=True)
                n = len(dyn.resid(x))
                pb_, J_, L_, db_ = np.exp(x)
                dynres = dict(p_band=pb_, armature=J_, latency=L_, deadband=db_, rel_se=dict(zip(
                    ("p_band", "armature", "latency", "deadband"), [float(s) for s in se])),
                    rms_resid=math.sqrt(cost / max(n, 1)))
    om0 = (tau_s / b) if (tau_s and b) else None
    log("tf", tf, "b", b, "dyn", dynres)
    act = dict(tau_s=tau_s, omega0=om0, omega_noload=om_p, damping=b, frictionloss=tf,
               p_band_deg=dynres["p_band"] / D2R if dynres else (stl or {}).get("p_band_check_deg"),
               armature=dynres["armature"] if dynres else None, latency_s=dynres["latency"] if dynres else None,
               deadband_deg=dynres["deadband"] / D2R if dynres else None)
    act["kp"] = (tau_s / (act["p_band_deg"] * D2R)) if (tau_s and act["p_band_deg"]) else None
    act["deadband_us"] = act["deadband_deg"] / cm.k if act["deadband_deg"] is not None else None
    act["t60_s"] = sp["t60_s"] if sp else None
    res["actuator"] = act
    # rl/actuators.py / MJCF icin hazir blok (spec 3.2.3); kabul DR'i olculenin +-%15'i (spec 4)
    if tau_s and act["kp"] and b:
        res["mjcf"] = dict(cls=MJCF_CLASS.get(cls), position=dict(kp=act["kp"], forcerange=[-tau_s, tau_s]),
                           joint=dict(damping=b, armature=act["armature"], frictionloss=tf),
                           deadband_rad=(act["deadband_deg"] or 0.0) * D2R,
                           deadband_model="hata olu bolgesi, tam genislik: rl/actuators.dead_band(ctrl, q, half = "
                                          "deadband_rad/2), e = ctrl - q her alt adimda, gecikmeden sonra",
                           latency_s=act["latency_s"], dr_accept_frac=0.15)
    res["fit"] = dict(speed=sp, friction={k: v for k, v in (fr or {}).items() if k != "segs"} or None,
                      dynamic=dynres, stall={k: v for k, v in (stl or {}).items() if k != "points"} or None,
                      stall_points=(stl or {}).get("points"))
    # besleme akimi
    sc = dict(model="I = i_stall*|tau_act|*|tau_net|/tau_s^2 + i_idle", i_stall=i_stall, i_idle=i_idle,
              r2=cur["r2"] if cur else None, c2_a_per_nm2=cur["c2_a_per_nm2"] if cur else None)
    if stl and stl.get("meter"):
        mt = stl["meter"]
        sc["stall_a_ina"], sc["stall_a_meter"], sc["stall_two_method_diff"] = mt["i_ina"], mt["i_meter"], mt.get("diff_frac")
        sc["stall_two_method_pass"] = (mt.get("diff_frac") is not None and mt["diff_frac"] <= 0.10)
    if stl:
        sc["stall_a_ina_sat"] = stl["i_sat_ina"]
    res["supply_current"] = sc
    # ortak mod
    i_ref = (i_stall * 0.85 ** 2 + (i_idle or 0.0)) if i_stall else 1.0
    cm_pts = (cur["points"] if cur else []) + ((stl or {}).get("points") or [])
    cmode = fit_common_mode(pm, cm_pts, i_ref)
    res["pot"]["common_mode"] = cmode
    res["pot"]["map"] = dict(src=pm.src, a_deg=pm.a, b_deg_per_mv=pm.b, ratio_a=pm.ar, ratio_b=pm.br,
                             cmd_c0_deg=cm.c0, cmd_deg_per_us=cm.k)
    # 8
    tr = {}
    for r in by.get("track", []):
        tr[r.tag or ("f%d" % int(r.params.get("freq", 0)))] = fit_track(r, pm, cm)
    t50 = tr.get("f50")
    t300 = tr.get("f300")
    res["pwm"] = dict(track=tr, accept_300hz=accept_300(t50, t300),
                      accept_3v3=(tr["v33"]["follows"] if "v33" in tr else None))
    # 9
    ovs = [(r, o) for r, o in ((r, fit_overshoot(r)) for r in by.get("overshoot", [])) if o]
    ov = None
    if ovs:
        ov = dict(v_max=max(o["v_max"] for _, o in ovs), v_min=min(o["v_min"] for _, o in ovs),
                  runs=[dict(tag=r.tag, **o) for r, o in ovs])
    res["rail"] = ov
    # 10
    ht = fit_heat(by["heat"][-1], pm, act["armature"], tf) if "heat" in by else None
    res["thermal"] = ht
    # 11
    res["gate"] = gate(tau_s, om_p, mass)
    kr = knee_rms if knee_rms is not None else M.knee_rms_estimate(mass)
    res["knee"] = dict(knee_rms_pred=kr, knee_rms_src="--knee-rms" if knee_rms is not None else
                       "tahmin 0.30*k(m) (P7 olcumuyle guncelleyin)",
                       eligible=(ht["t_knee"] >= kr) if (ht and ht.get("t_knee")) else None)
    from .report import verdicts
    res["verdicts"] = verdicts(res)
    return res


def _clean(o):
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return [_clean(v) for v in o.tolist()]
    if isinstance(o, (float, int, np.floating, np.integer, bool, np.bool_)):
        return _f(o)
    return o


def main(argv=None):
    ap = argparse.ArgumentParser(description="Adim 0 sysID: tezgah log'larindan parametreler + karar tablosu")
    ap.add_argument("--logs", required=True, help="servo klasoru ya da servo klasorlerinin ust klasoru")
    ap.add_argument("--mass", type=float, default=1.95, help="G-CAD kutlesi kg (CAD x 1.05); varsayilan sert sinir")
    ap.add_argument("--knee-rms", type=float, default=None, help="G-ISIL tahmini diz tau_net RMS (N.m)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--report", default=None)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    from .report import write_report
    root = Path(a.logs)
    dirs = [root] if load_dir(root) else sorted(d for d in root.iterdir() if d.is_dir() and load_dir(d))
    if not dirs:
        raise SystemExit("log bulunamadi: %s" % root)
    results = []
    for d in dirs:
        try:
            r = fit_servo(d, a.mass, a.knee_rms, a.verbose)
        except SystemExit as e:
            print("atlandi:", d, e)
            continue
        results.append(_clean(r))
    if not results:
        raise SystemExit("uydurulacak servo yok")
    out = Path(a.out) if a.out else root / "sysid_params.json"
    payload = results[0] if len(results) == 1 else dict(schema="openduck-tr/sysid/v1", servos=results)
    out.write_text(json.dumps(payload, indent=1, ensure_ascii=False))
    rp = Path(a.report) if a.report else root / "sysid_rapor.md"
    rp.write_text(write_report(results, a.mass))
    print("params:", out)
    print("rapor :", rp)
    return payload


if __name__ == "__main__":
    main()
