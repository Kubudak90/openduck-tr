# -*- coding: utf-8 -*-
"""
Bilinen (gercek parametreleri belli) bir servodan sentetik tezgah log'u.

Firmware programlarinin (tools/bench/bench/bench_main.cpp) Python esidir: ayni komut dizisi, ayni olaylar (E),
ayni D kolonlari, ayni istemler (P) + yanitlar. Satirlar logio.RunRecorder'dan gecer; boylece fit, logger'in
yazdigi dosyalarin aynisini okur. Servo model.simulate ile, algilayicilar (pot bolucusu + RC, ADS, INA226,
NTC, ray) asagidaki gercek degerlerle uretilir.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from . import model as M
from .logio import CFG_DEFAULTS, CLASS_DEFAULTS, COLUMNS, G, RunRecorder

FW = "synth"


@dataclass
class Truth:
    p: M.ServoParams = field(default_factory=M.ServoParams)
    cls: str = "ds"
    us_c: float = 1512.0          # servo ic sifiri (us)
    dpu: float = 0.0935           # gercek derece/us
    pc_off: float = 1.3           # aciolcer sifiri - servo sifiri (derece)
    pot_span: float = 290.0       # pot elektriksel acisi (derece)
    pot_nl: float = 0.004         # kubik egrilik (x biriminde, aralik ucunda)
    pot_reg: float = 3.3          # pot ic regulatorden (V); 0: servo rayindan
    v_rail: float = 6.5
    r_gnd: float = 0.026          # servo toprak kablosu (Ohm)
    r_src: float = 0.12           # ray kaynak direnci (fren geri beslemesi dahil)
    noise_mv: float = 0.6         # pot pininde (ortalama sonrasi)
    noise_ma: float = 1.5
    rc_ms: float = 3.2
    accept_300: bool = True
    accept_3v3: bool = True
    tau_th: float = 240.0         # kasa isil zaman sabiti (s)
    k_heat: float = 37.0          # C / (N.m)^2 (RMS tau_net^2 basina kararli artis)
    dT0: float = 1.0
    t_amb: float = 24.0
    holds: bool = True            # PWM kesilince servo tutar (dijital)

    def us_to_rad(self, us):
        return np.radians((np.asarray(us, float) - self.us_c) * self.dpu)

    def rad_to_us(self, q):
        return self.us_c + np.degrees(q) / self.dpu


class Avg:
    """Olay alani: olay anindan onceki n tikin ortalamasi (sim sonrasi cozulur)."""

    def __init__(self, col, n, fmt="%.2f"):
        self.col, self.n, self.fmt = col, n, fmt


class PB:
    """Program kurucu: firmware'deki tk/hold/ramp/seg/ask yardimcilarinin esi."""

    def __init__(self, bench, rate):
        self.b = bench
        self.rate = float(rate)
        self.us = []
        self.seg_i = []
        self.mute = []
        self.events = []   # (tik, ad, kv)
        self.seg_n = -1
        self.cur = bench.cur_us
        self.muted = False

    def tk(self, us):
        self.us.append(float("nan") if us is None else float(self.b.quant(us)))
        self.seg_i.append(self.seg_n)
        self.mute.append(self.muted)
        if us is not None:
            self.cur = us

    def hold(self, us, ms):
        for _ in range(max(1, int(round(ms * self.rate / 1000.0)))):
            self.tk(us)

    def ramp(self, u0, u1, us_per_s):
        n = max(1, int(math.ceil(abs(u1 - u0) / us_per_s * self.rate)))
        for i in range(1, n + 1):
            self.tk(u0 + (u1 - u0) * i / n)

    def move(self, u1, dps=60.0):
        self.ramp(self.cur, u1, dps / self.b.dpu_nom)

    def seg(self, kind, **kv):
        self.seg_n += 1
        d = dict(i=self.seg_n, kind=kind)
        d.update(kv)
        self.events.append((len(self.us), "seg", d))

    def ev(self, _name, **kv):
        self.events.append((len(self.us), _name, kv))

    def ask(self, pid, text, ans, hold_s=2.0):
        self.events.append((len(self.us), "P", dict(id=pid, text=text)))
        self.muted = True
        self.hold(self.cur, hold_s * 1000)
        self.muted = False
        self.events.append((len(self.us), "ans", dict(id=pid, val=ans)))


class Bench:
    """Sentetik tezgah: Truth servosu + firmware ayarlari."""

    def __init__(self, truth=None, cfg=None, seed=0, freq=None, dt=2.5e-4):
        self.tr = truth or Truth()
        self.cfg = dict(CFG_DEFAULTS)
        self.cfg.update(dict(cls=self.tr.cls))
        self.cfg.update(cfg or {})
        cd = CLASS_DEFAULTS[self.cfg["cls"]]
        self.dpu_nom = cd["deg_per_us"]
        self.us_c = cd["us_c"]
        self.freq = freq or cd["freq"]
        self.rng = np.random.default_rng(seed)
        self.dt = dt
        self.t0 = 1.0
        self.cur_us = self.us_c
        self.state = None          # (q, qd, ce) son kosu sonu
        self.temp = self.tr.t_amb
        self.mass_g = float(self.cfg.get("mass_g", 0.0))

    # --- PCA ---
    @property
    def tick_us(self):
        pre = 19 if self.freq > 100 else 121
        f = 25.0e6 / (4096.0 * (pre + 1))
        return 1e6 / (4096.0 * f)

    def quant(self, us):
        tk = self.tick_us
        return round(float(us) / tk) * tk

    def cfg_line(self):
        c = dict(self.cfg)
        c.update(freq=self.freq, tick_us=round(self.tick_us, 4), fw=FW)
        return "CFG," + ",".join("%s=%s" % (k, v) for k, v in c.items())

    # --- yuk ---
    def load(self, kind):
        L = self.cfg["lever_mm"] / 1000.0
        m = self.cfg.get("mass_g", 0.0) / 1000.0 if kind in ("mass",) else 0.0
        ml = self.cfg.get("lever_g", 0.0) / 1000.0
        return M.Load(inertia=m * L * L + ml * L * L / 3.0, mgl=G * (m * L + ml * L / 2.0), q_h=self.q_h)

    @property
    def q_h(self):
        # kol yatayi: servo cercevesinde 0 rad (kullanici horn'u 1500 us civarinda yatay takar)
        return math.radians(-2.0)

    def jog_u(self, q_target, load=None, u_from=None):
        """Kullanici kolu hedef aciya jog'lar (yuk altinda gercek aci): statik denge iterasyonu."""
        u = float(self.tr.rad_to_us(q_target))
        u0 = self.cur_us if u_from is None else u_from
        st = self.state
        for _ in range(5):
            tt = np.arange(0.0, 1.2, 0.01)
            n0 = 40
            c = np.concatenate([np.linspace(float(self.tr.us_to_rad(u0)), float(self.tr.us_to_rad(u)), n0),
                                np.full(len(tt) - n0, float(self.tr.us_to_rad(u)))])
            q0, qd0, ce0 = st if st else (float(c[0]), 0.0, float(c[0]))
            sm = M.simulate(self.tr.p, tt, c, tt, load=load, q0=q0, qd0=qd0, ceff0=ce0, dt=self.dt)
            err = q_target - float(sm["q"][-1])
            if abs(err) < 2e-5:
                break
            u += math.degrees(err) / self.tr.dpu
        return u

    # --- algilayicilar ---
    def sensors(self, q_rc, q, i_sup, i_signed, temp):
        tr = self.tr
        div = self.cfg["div"]
        x = 0.5 + np.degrees(q_rc) / tr.pot_span
        x = x + tr.pot_nl * ((x - 0.5) / 0.5) ** 3
        v_rail = tr.v_rail - tr.r_src * i_signed
        v_lo = i_sup * tr.r_gnd
        v_hi = (v_lo + tr.pot_reg) if tr.pot_reg > 0 else (v_rail - i_sup * tr.r_gnd)
        vw = v_lo + x * (v_hi - v_lo)
        n = len(q)
        rn = self.rng.normal
        pot = vw * div * 1000.0 + rn(0, tr.noise_mv, n)
        ph = v_hi * div * 1000.0 + rn(0, tr.noise_mv, n)
        rail = v_rail * self.cfg["rail_div"] * 1000.0 + rn(0, 1.5, n)
        ima = np.round((i_sup * 1000.0 + rn(0, tr.noise_ma, n)) * 2.0) / 2.0
        vbus = np.round((v_rail - 0.005 * i_sup) * 1000.0 / 1.25) * 1.25
        adsw = vw * div * 1000.0 + rn(0, 0.15, n)
        adslo = v_lo * 1e6 + rn(0, 4.0, n)
        ntc = temp + rn(0, 0.04, n)
        amb = np.full(n, tr.t_amb) + rn(0, 0.15, n)
        return dict(pot_mv=pot, ph_mv=ph, rail_mv=rail, i_ma=ima, vbus_mv=vbus, adsw_mv=adsw, adslo_uv=adslo,
                    ntc_c=ntc, amb_c=amb)

    # --- kosu ---
    def finish(self, pb, program, params, load=None, temp=None, extra_lines=None, results=None, sim=None):
        """Komut dizisini simule et, satirlari uret, RunRecorder'a besle."""
        n = len(pb.us)
        t = self.t0 + np.arange(n) / pb.rate + self.rng.uniform(-15e-6, 15e-6, n)
        us = np.array(pb.us)
        off = np.isnan(us)
        us_f = us.copy()
        # PWM kesik: dijital servo son komutu tutar (holds) -> ayni komutla simule
        last = self.cur_us
        for i in range(n):
            if off[i]:
                us_f[i] = last
            else:
                last = us_f[i]
        c = self.tr.us_to_rad(us_f)
        if sim is None:
            q0, qd0, ce0 = self.state if self.state else (float(c[0]), 0.0, float(c[0]))
            # INA226 akimi tik araligi boyunca ortalama okur (AVG x donusum ~ tik): i_avg
            sim = M.simulate(self.tr.p, t, c, t, load=load, q0=q0, qd0=qd0, ceff0=ce0, dt=self.dt,
                             rc_tau=self.tr.rc_ms * 1e-3)
            ce_end = float(sim.get("ce_end", sim["q"][-1]))
            self.state = (float(sim["q"][-1]), float(sim["qd"][-1]), ce_end)
        i_sup = sim.get("i_avg", sim["i"])
        i_signed = sim.get("i_signed_avg", self.tr.p.i_stall * sim["tau_act"] * sim["tau_net"] / self.tr.p.tau_s ** 2
                           + self.tr.p.i_idle)
        temp = np.full(n, self.temp) if temp is None else temp
        s = self.sensors(sim["q_rc"], sim["q"], i_sup, i_signed, temp)
        s["cmd_us"] = np.where(off, 0.0, us)
        s["cap_us"] = np.where(off, 0.0, us)
        s["t_us"] = np.round(t * 1e6)
        s["seg"] = np.array(pb.seg_i, float)
        lines = [self.cfg_line(), "S,%s,%s" % (program, FW),
                 "E,%d,params,%s" % (round(t[0] * 1e6),
                                     ",".join("%s=%s" % (k, str(v).replace(",", ";")) for k, v in params.items())),
                 "H," + ",".join(COLUMNS)]
        evs = sorted(pb.events, key=lambda e: e[0])
        ei = 0
        fmt = dict(t_us="%d", seg="%d", cmd_us="%.2f", pot_mv="%.1f", ph_mv="%.1f", rail_mv="%.0f", i_ma="%.1f",
                   vbus_mv="%.0f", adsw_mv="%.2f", adslo_uv="%.0f", ntc_c="%.2f", amb_c="%.2f", cap_us="%.1f")
        for i in range(n + 1):
            while ei < len(evs) and evs[ei][0] <= i:
                k, name, kv = evs[ei]
                tt = round(t[min(k, n - 1)] * 1e6)
                if name == "P":
                    lines.append("P,%s,%s" % (kv["id"], kv["text"]))
                else:
                    kv2 = {}
                    for a, v in kv.items():
                        if isinstance(v, Avg):
                            lo = max(0, k - v.n)
                            v = v.fmt % float(np.mean(s[v.col][lo:max(k, lo + 1)]))
                        kv2[a] = v
                    lines.append("E,%d,%s,%s" % (tt, name, ",".join("%s=%s" % x for x in kv2.items())))
                ei += 1
            if i < n and not pb.mute[i]:
                lines.append("D," + ",".join(fmt[cname] % s[cname][i] for cname in COLUMNS))
        for ln in (extra_lines or []):
            lines.append(ln)
        for k, v in (results or {}).items():
            lines.append("R,%s,%s" % (k, v))
        lines.append("END,%s,ok" % program)
        rec = RunRecorder()
        for ln in lines:
            rec.feed(ln)
        self.t0 = float(t[-1]) + 5.0
        self.cur_us = pb.cur
        return rec, dict(t=t, sim=sim, sens=s, us=us)

    # ================= programlar (firmware ile ayni) =================
    def run(self, program, **kw):
        return getattr(self, "p_" + program)(**kw)

    def _par(self, defaults, kw):
        d = dict(defaults)
        d.update(kw)
        return d

    def p_sweep(self, **kw):
        cd = CLASS_DEFAULTS[self.cfg["cls"]]
        P = self._par(dict(lo_us=cd["us_lo"] + 50, hi_us=cd["us_hi"] - 50, step_us=10, dwell_ms=200, rate=200,
                           tag=""), kw)
        pb = PB(self, P["rate"])
        pb.seg("move")
        pb.move(P["lo_us"])
        pb.hold(P["lo_us"], 1000)
        u = P["lo_us"]
        while u <= P["hi_us"] + 1e-9:
            pb.seg("stair", dir=1, us="%.1f" % u)
            pb.hold(u, P["dwell_ms"])
            u += P["step_us"]
        u -= P["step_us"]
        while u >= P["lo_us"] - 1e-9:
            pb.seg("stair", dir=-1, us="%.1f" % u)
            pb.hold(u, P["dwell_ms"])
            u -= P["step_us"]
        pb.seg("move")
        pb.move(self.us_c)
        pb.hold(self.us_c, 500)
        return self.finish(pb, "sweep", P)[0]

    def p_potcal(self, points="-90,-45,0,45,90", n=256, rate=200, align_sd=0.15, tag=""):
        P = dict(points=points, n=n, rate=rate, tag=tag)
        pb = PB(self, rate)
        for i, d in enumerate([float(x) for x in str(points).split(";" if ";" in str(points) else ",")]):
            # kullanici kolu aciolcer cizgisine jog'lar: servo cercevesinde (d - pc_off) + hizalama hatasi
            q_true = math.radians(d - self.tr.pc_off + self.rng.normal(0, align_sd))
            u = self.jog_u(q_true, None, pb.cur)
            pb.ask("pc%d" % i, "Kolu %g derece cizgisine getirin (+N/-N us), sonra ok" % d, "ok")
            pb.seg("potcal", deg=d)
            pb.move(u, 30.0)
            pb.hold(u, 500)
            pb.hold(u, n * 1000.0 / rate)
            pb.ev("potcal", deg=d, us="%.2f" % self.quant(u), mv=Avg("pot_mv", n), ph_mv=Avg("ph_mv", n),
                  adsw_mv=Avg("adsw_mv", n), adslo_uv=Avg("adslo_uv", n, "%.0f"), n=n)
        pb.seg("move")
        pb.move(self.us_c)
        pb.hold(self.us_c, 300)
        return self.finish(pb, "potcal", P)[0]

    def p_deadband(self, **kw):
        P = self._par(dict(c_us=self.us_c, span_us=12.0, cycles=3, dwell_ms=150, rate=200, tag=""), kw)
        pb = PB(self, P["rate"])
        c, sp, tk = P["c_us"], P["span_us"], self.tick_us
        pb.seg("move")
        pb.move(c - 3 * sp)
        pb.hold(c - 3 * sp, 500)
        pb.ramp(c - 3 * sp, c - sp, 100.0)
        pb.hold(c - sp, 1000)
        nst = int(round(2 * sp / tk))
        u = c - sp
        for _ in range(int(P["cycles"])):
            for d in (1, -1):
                for _ in range(nst):
                    u += d * tk
                    pb.seg("stair", dir=d, us="%.2f" % self.quant(u))
                    pb.hold(u, P["dwell_ms"])
        pb.seg("move")
        pb.move(c)
        pb.hold(c, 300)
        return self.finish(pb, "deadband", P)[0]

    def p_step(self, **kw):
        P = self._par(dict(c_us=self.us_c, amps_deg="1,3,6,20,40", hold_ms=500, reps=2, rate=500, tag=""), kw)
        pb = PB(self, P["rate"])
        c = P["c_us"]
        pb.seg("hold")
        pb.move(c)
        pb.hold(c, 500)
        for a in [float(x) for x in str(P["amps_deg"]).replace(";", ",").split(",")]:
            for _ in range(int(P["reps"])):
                for s in (1, -1):
                    u = c + s * 0.5 * a / self.dpu_nom
                    pb.seg("step", amp_deg=a, us="%.2f" % self.quant(u))
                    pb.hold(u, P["hold_ms"])
        pb.seg("hold")
        pb.move(c)
        pb.hold(c, 300)
        return self.finish(pb, "step", P)[0]

    def p_chirp(self, **kw):
        P = self._par(dict(c_us=self.us_c, amp_deg=1.5, f0=0.5, f1=25.0, dur_s=20.0, rate=500, tag=""), kw)
        pb = PB(self, P["rate"])
        c, A = P["c_us"], P["amp_deg"] / self.dpu_nom
        pb.seg("hold")
        pb.move(c)
        pb.hold(c, 1000)
        pb.seg("chirp", f0=P["f0"], f1=P["f1"], dur_s=P["dur_s"], amp_deg=P["amp_deg"])
        T, k = P["dur_s"], math.log(P["f1"] / P["f0"])
        nn = int(round(T * P["rate"]))
        for i in range(1, nn + 1):
            tt = i / P["rate"]
            ph = 2 * math.pi * P["f0"] * T / k * (math.exp(tt * k / T) - 1.0)
            pb.tk(c + A * math.sin(ph))
        pb.seg("hold")
        pb.hold(c, 1000)
        return self.finish(pb, "chirp", P)[0]

    def p_speed(self, **kw):
        P = self._par(dict(c_us=self.us_c, amp_deg=100.0, reps=3, hold_ms=1000, rate=500, tag=""), kw)
        pb = PB(self, P["rate"])
        c, h = P["c_us"], 0.5 * P["amp_deg"] / self.dpu_nom
        pb.seg("move")
        pb.move(c - h)
        pb.hold(c - h, P["hold_ms"])
        for _ in range(int(P["reps"])):
            for s in (1, -1):
                pb.seg("step", amp_deg=P["amp_deg"], dir=s, us="%.2f" % self.quant(c + s * h))
                pb.hold(c + s * h, P["hold_ms"])
        pb.seg("move")
        pb.move(c)
        pb.hold(c, 300)
        return self.finish(pb, "speed", P)[0]

    def p_ramp(self, **kw):
        P = self._par(dict(c_us=self.us_c, amp_deg=40.0, speeds_dps="5,10,20,40,80,160", reps=1, hold_ms=800,
                           rate=500, tag=""), kw)
        pb = PB(self, P["rate"])
        c, h = P["c_us"], 0.5 * P["amp_deg"] / self.dpu_nom
        pb.seg("move")
        pb.move(c - h)
        pb.hold(c - h, P["hold_ms"])
        for v in [float(x) for x in str(P["speeds_dps"]).replace(";", ",").split(",")]:
            for _ in range(int(P["reps"])):
                for s in (1, -1):
                    pb.seg("ramp", dir=s, speed_dps=v)
                    pb.ramp(c - s * h, c + s * h, v / self.dpu_nom)
                    pb.seg("hold")
                    pb.hold(c + s * h, P["hold_ms"])
        pb.seg("move")
        pb.move(c)
        pb.hold(c, 300)
        return self.finish(pb, "ramp", P)[0]

    def p_stall(self, **kw):
        P = self._par(dict(dir=1, deltas_deg="0.5,1,2,3,4,6,8,12,20", hold_ms=2500, cool_ms=5000, meter=1,
                           rate=200, tag=""), kw)
        tr, p = self.tr, self.tr.p
        L = self.cfg["lever_mm"] / 1000.0
        q_touch = math.radians(-1.0)
        load = M.Load(wall=q_touch)
        u_t = float(self.tr.rad_to_us(q_touch))
        pb = PB(self, P["rate"])
        pb.seg("move")
        pb.move(u_t - 2.0 / self.dpu_nom)
        pb.ask("touch", "Kol ucunu teraziye degdirin (jog), teraziyi sifirlayin, ok", "ok")
        pb.move(u_t, 20.0)
        pb.hold(u_t, 500)
        pb.ev("ref", name="touch", us="%.2f" % self.quant(u_t), mv=Avg("pot_mv", 50))
        deltas = [float(x) for x in str(P["deltas_deg"]).replace(";", ",").split(",")]
        if int(P["meter"]):
            deltas = deltas + [max(deltas)]
        marks = []
        for k, d in enumerate(deltas):
            u = u_t + P["dir"] * d / self.dpu_nom
            idk = "meter" if (int(P["meter"]) and k == len(deltas) - 1) else str(k)
            if idk == "meter":
                pb.ask("meter_ready", "Multimetreyi izleyin; ok deyince 3 s stall", "ok")
            pb.seg("stall", i=idk, delta_deg=d, us="%.2f" % self.quant(u))
            pb.hold(u, P["hold_ms"] if idk != "meter" else 3000)
            marks.append((idk, len(pb.us)))
            pb.seg("rest")
            pb.hold(u_t - 1.0 / self.dpu_nom, 200)
            pb.ask("g_" + idk if idk != "meter" else "meter_a", "deger?", "__%s__" % idk, hold_s=1.0)
            pb.hold(u_t - 1.0 / self.dpu_nom, P["cool_ms"])
        pb.seg("move")
        pb.move(self.us_c)
        rec, out = self.finish(pb, "stall", P, load=load)
        # yanitlar: terazi (g) ve multimetre (A)
        ans = {}
        for idk, k in marks:
            ta = float(out["sim"]["tau_act"][k - 1])
            if idk == "meter":
                ans["meter_a"] = "%.2f" % (float(out["sim"]["i"][k - 1]) * (1 + self.rng.normal(0, 0.01)))
            else:
                ans["g_" + idk] = "%d" % round(ta / (G * L) * 1000.0 + self.rng.normal(0, 0.5))
        _fix_answers(rec, ans)
        return rec

    def p_curpts(self, **kw):
        P = self._par(dict(angles_deg="-60,-30,0,30,60,90", hold_ms=2000, speeds_dps="30,90", amp_deg=30.0,
                           rate=250, tag=""), kw)
        pb = PB(self, P["rate"])
        u_h = self.jog_u(self.q_h, self.load("mass"))
        pb.seg("move")
        pb.ask("horiz", "Kolu yatay konuma getirin (jog), ok", "ok")
        pb.move(u_h, 30.0)
        pb.hold(u_h, 600)
        pb.ev("ref", name="horiz", us="%.2f" % self.quant(u_h), mv=Avg("pot_mv", 100))
        for a in [float(x) for x in str(P["angles_deg"]).replace(";", ",").split(",")]:
            u = u_h + a / self.dpu_nom
            for app in (1, -1):
                # iki yonden yaklasma: surtunme tutma payi (+-tau_f) ortalamada sadelesir
                pb.seg("move")
                pb.move(u - app * 3.0 / self.dpu_nom, 60.0)
                pb.hold(pb.cur, 200)
                pb.seg("static", ang_deg=a, app=app)
                pb.move(u, 20.0)
                pb.hold(u, P["hold_ms"])
        A = P["amp_deg"] / self.dpu_nom
        for v in [float(x) for x in str(P["speeds_dps"]).replace(";", ",").split(",")]:
            pb.seg("move")
            pb.move(u_h - A, 60.0)
            pb.hold(u_h - A, 300)
            for _ in range(2):
                for s in (1, -1):
                    pb.seg("ramp_ld", dir=s, speed_dps=v)
                    pb.ramp(u_h - s * A, u_h + s * A, v / self.dpu_nom)
        pb.seg("move")
        pb.move(u_h, 60.0)
        pb.hold(u_h, 300)
        return self.finish(pb, "curpts", P, load=self.load("mass"))[0]

    def p_track(self, **kw):
        cd = CLASS_DEFAULTS[self.cfg["cls"]]
        P = self._par(dict(freq=cd["freq"], amp_deg=10.0, reps=3, step_ms=800, f_sin=1.0, sin_s=5.0, hold_s=2.0,
                           rate=250, tag=""), kw)
        f_old = self.freq
        self.freq = P["freq"]
        ok = (self.tr.accept_300 or P["freq"] < 100) and (self.tr.accept_3v3 or "v33" not in str(P["tag"]))
        pb = PB(self, P["rate"])
        c, A = self.us_c, P["amp_deg"] / self.dpu_nom
        pb.ev("pca", freq=P["freq"], tick_us="%.4f" % self.tick_us)
        pb.seg("hold")
        pb.move(c)
        pb.hold(c, 1000)
        for _ in range(int(P["reps"])):
            for s in (1, -1):
                pb.seg("step", amp_deg=2 * P["amp_deg"], us="%.2f" % self.quant(c + s * A))
                pb.hold(c + s * A, P["step_ms"])
        pb.seg("sin", f=P["f_sin"], amp_deg=P["amp_deg"])
        nn = int(round(P["sin_s"] * P["rate"]))
        for i in range(1, nn + 1):
            pb.tk(c + A * math.sin(2 * math.pi * P["f_sin"] * i / P["rate"]))
        pb.seg("hold")
        pb.hold(c, P["hold_s"] * 1000)
        pb.ask("obs", "Servo duzgun takip etti mi (titreme/ses/isinma yok)? (e/h)", "e" if ok else "h", hold_s=1.0)
        if not ok:
            # servo darbeyi kabul etmiyor: yerinde kalir (komut yok sayilir)
            n = len(pb.us)
            t = self.t0 + np.arange(n) / pb.rate
            q = np.full(n, float(self.tr.us_to_rad(self.cur_us)))
            sim = dict(q=q, qd=np.zeros(n), q_rc=q, tau_act=np.zeros(n), tau_net=np.zeros(n),
                       i=np.full(n, self.tr.p.i_idle))
            rec = self.finish(pb, "track", P, sim=sim)[0]
        else:
            rec = self.finish(pb, "track", P)[0]
        self.freq = f_old
        return rec

    def p_overshoot(self, **kw):
        P = self._par(dict(amp_deg=90.0, reps=4, hold_ms=800, n=8000, dt_us=50, dec=4, rate=200, tag=""), kw)
        pb = PB(self, P["rate"])
        c, h = self.us_c, 0.5 * P["amp_deg"] / self.dpu_nom
        pb.seg("move")
        pb.move(c - h)
        pb.hold(c - h, 500)
        bursts = []
        for k in range(int(P["reps"])):
            s = 1 if k % 2 == 0 else -1
            pb.seg("burst", i=k)
            pb.hold(c - s * h, 100)
            bursts.append((k, len(pb.us), c + s * h))
            pb.muted = True
            pb.hold(c + s * h, P["n"] * P["dt_us"] / 1000.0)   # yakalama suresi (D satiri yok)
            pb.muted = False
            pb.hold(c + s * h, P["hold_ms"])
        pb.seg("move")
        pb.move(c)
        rec, out = self.finish(pb, "overshoot", P)
        # burst: hizli ray ornekleri (komut aninin ardindan n x dt_us); ayrintili sim
        tr = self.tr
        lines, vmax = [], 0.0
        for k, i0, u in bursts:
            t0 = out["t"][i0]
            tb = t0 + np.arange(int(P["n"])) * P["dt_us"] * 1e-6
            q0 = float(out["sim"]["q"][i0 - 1])
            c_prev = float(tr.us_to_rad(out["us"][i0 - 1]))
            sim = M.simulate(tr.p, [t0 - 1.0, t0], [c_prev, float(tr.us_to_rad(self.quant(u)))], tb,
                             q0=q0, ceff0=q0, dt=self.dt, rc_tau=0.0)
            isg = tr.p.i_stall * sim["tau_act"] * sim["tau_net"] / tr.p.tau_s ** 2 + tr.p.i_idle
            v = tr.v_rail - tr.r_src * isg
            vmax = max(vmax, float(v.max()))
            mv = v * self.cfg["rail_div"] * 1000.0 + self.rng.normal(0, 1.5, len(v))
            v0 = tr.v_rail - tr.r_src * tr.p.i_idle
            lines.append("E,%d,burst,i=%d,n=%d,dt_us=%d,v0_mv=%.1f,vbus0_mv=%.1f" % (
                round(t0 * 1e6), k, len(v), P["dt_us"], v0 * self.cfg["rail_div"] * 1000.0, v0 * 1000.0))
            for j in range(0, len(v), int(P["dec"])):
                lines.append("B,%d,%.1f" % (round(tb[j] * 1e6), mv[j]))
        self.ovs_true = vmax
        rec2 = RunRecorder()
        for ln in rec.raw[:-1] + lines + rec.raw[-1:]:
            rec2.feed(ln)
        return rec2

    def p_heat(self, **kw):
        P = self._par(dict(levels_nm="0.9,1.05,1.15", level_s=600, f_hz=1.1, amp_deg=15.0, cool_c=3.0,
                           cool_max_s=1200, rest_deg=-80.0, rate=50, tag=""), kw)
        tr = self.tr
        ld = self.load("mass")
        L = self.cfg["lever_mm"] / 1000.0
        mgl_fw = G * (self.cfg["mass_g"] / 1000.0 * L + self.cfg.get("lever_g", 0.0) / 1000.0 * L / 2.0)
        u_h = float(tr.rad_to_us(self.q_h))
        rate = float(P["rate"])
        pb = PB(self, rate)
        pb.seg("move")
        pb.ask("horiz", "Kolu yatay konuma getirin (jog), ok", "ok")
        pb.move(u_h, 30.0)
        pb.hold(u_h, 600)
        pb.ev("ref", name="horiz", us="%.2f" % self.quant(u_h), mv=Avg("pot_mv", 25))
        lvl_T = []          # (k0, k1, sicaklik dizisi)
        sims = []           # (baslangic tik, bitis tik, periyodik cozum)
        T = self.temp
        A = math.radians(P["amp_deg"])
        for li, tau in enumerate([float(x) for x in str(P["levels_nm"]).replace(";", ",").split(",")]):
            qc = M.heat_center(tau, mgl_fw, A)
            qc_deg = math.degrees(qc)
            pb.seg("heat", lvl=li, tau_nm=tau, qc_deg="%.3f" % qc_deg, amp_deg=P["amp_deg"], f_hz=P["f_hz"],
                   mgl="%.4f" % mgl_fw)
            pb.move(u_h + qc_deg / self.dpu_nom, 30.0)
            k0 = len(pb.us)
            nn = int(round(P["level_s"] * rate))
            for i in range(nn):
                pb.tk(u_h + (qc_deg + P["amp_deg"] * math.sin(2 * math.pi * P["f_hz"] * i / rate)) / self.dpu_nom)
            k1 = len(pb.us)
            # periyodik kararli cozum: 4 periyot simule, son periyot
            per = 1.0 / P["f_hz"]
            tf = np.arange(0.0, 4 * per, 1e-3)
            cf = tr.us_to_rad(u_h + (qc_deg + P["amp_deg"] * np.sin(2 * np.pi * P["f_hz"] * tf)) / self.dpu_nom)
            sm = M.simulate(tr.p, tf, cf, tf, load=ld, q0=float(cf[0]), dt=self.dt, rc_tau=tr.rc_ms * 1e-3)
            sel = tf >= 3 * per
            sims.append((k0, k1, tf[sel] - 3 * per, {k: v[sel] for k, v in sm.items() if k != "ce_end"}, per))
            msq = float(np.mean(sm["tau_net"][sel] ** 2))
            dTss = tr.k_heat * msq + tr.dT0
            tt = np.arange(nn) / rate
            Tl = M.thermal_step(tt, tr.t_amb, dTss, T, tr.tau_th)
            lvl_T.append((k0, k1, Tl))
            T = float(Tl[-1])
            # sogutma: dinlenme acisi, PWM kes, ntc < amb + cool_c
            pb.seg("cool", lvl=li)
            pb.move(u_h + P["rest_deg"] / self.dpu_nom, 30.0)
            pb.ev("pwm", on=0)
            tc = 0.0
            while tc < P["cool_max_s"]:
                pb.tk(None)
                tc += 1.0 / rate
                if tr.t_amb + (T - tr.t_amb) * math.exp(-tc / tr.tau_th) < tr.t_amb + P["cool_c"]:
                    break
            T = tr.t_amb + (T - tr.t_amb) * math.exp(-tc / tr.tau_th)
            pb.ev("pwm", on=1)
        pb.seg("move")
        pb.move(u_h, 30.0)
        pb.hold(u_h, 300)
        # sicaklik dizisi (sogutma + hareketler: ustel bozunma)
        n = len(pb.us)
        temps = [None] * n
        for k0, k1, Tl in lvl_T:
            temps[k0:k1] = list(Tl)
        Tc = self.temp
        out_T = np.empty(n)
        for i in range(n):
            if temps[i] is not None:
                Tc = temps[i]
            else:
                Tc = tr.t_amb + (Tc - tr.t_amb) * math.exp(-1.0 / rate / tr.tau_th)
            out_T[i] = Tc
        self.temp = float(out_T[-1])
        # hareket: duraganlar icin model.simulate (hizli degil ama kisa); seviye icin periyodik cozum
        us = np.array(pb.us)
        tt = self.t0 + np.arange(n) / rate
        usf = us.copy()
        last = self.cur_us
        for i in range(n):
            if np.isnan(usf[i]):
                usf[i] = last
            else:
                last = usf[i]
        c = tr.us_to_rad(usf)
        sim = dict(q=c.copy(), qd=np.zeros(n), q_rc=c.copy(), tau_act=np.zeros(n), tau_net=np.zeros(n))
        # duragan: yercekimi torku (tutma)
        tg = ld.mgl * np.cos(c - ld.q_h)
        sim["tau_act"], sim["tau_net"] = tg.copy(), tg.copy()
        for k0, k1, tper, sm, per in sims:
            ph = (np.arange(k1 - k0) / rate) % per
            for key in ("q", "qd", "q_rc", "tau_act", "tau_net"):
                sim[key][k0:k1] = np.interp(ph, tper, sm[key], period=per)
        sim["i"] = M.supply_current(tr.p, sim["tau_act"], sim["tau_net"])
        self.heat_truth = sims
        rec = self.finish(pb, "heat", P, sim=sim, temp=out_T)[0]
        return rec


def _fix_answers(rec, ans):
    """Sim sonrasi bilinen yanitlari (terazi g, multimetre A) kayda yaz."""
    raw = []
    for ln in rec.raw:
        if ln.startswith("E,") and ",ans," in ln and "val=__" in ln:
            pid = ln.split("id=")[1].split(",")[0]
            ln = ln.split(",val=")[0] + ",val=" + ans.get(pid, "")
        raw.append(ln)
    rec2 = RunRecorder()
    for ln in raw:
        rec2.feed(ln)
    rec.__dict__.update(rec2.__dict__)


def true_t_knee(tr, level_s=600.0):
    """Gercek T_diz: 25 C ortam, ortamdan baslayan 10 dk sonunda kasa = 60 C olan RMS tau_net."""
    a = 1.0 - math.exp(-level_s / tr.tau_th)
    return math.sqrt((35.0 / a - tr.dT0) / tr.k_heat)


def full_session(bench, out_dir, servo_id="DS-T1", quick=True):
    """Adim 0 oturumu: tum programlar (quick: kisaltilmis), dosyalar out_dir'e."""
    from pathlib import Path
    out = Path(out_dir)
    q = quick
    plan = [
        ("sweep", "sweep", dict(step_us=40 if q else 10, dwell_ms=150 if q else 200)),
        ("potcal", "potcal", dict(points="-60,-30,0,30,60")),
        ("deadband", "deadband", dict(cycles=2 if q else 3, dwell_ms=100 if q else 150)),
        ("speed", "speed", dict(reps=2 if q else 3, hold_ms=600)),
        ("step", "step", dict(amps_deg="1,3,6,20" if q else "1,3,6,20,40", reps=2, hold_ms=400)),
        ("chirp", "chirp", dict(dur_s=20)),
        ("ramp", "ramp", dict(speeds_dps="5,10,20,80,160" if q else "5,10,20,40,80,160", hold_ms=400)),
        ("stall", "stall", dict(deltas_deg="0.5,1,1.5,2,3,5,8,12", hold_ms=1500, cool_ms=1000 if q else 5000)),
        ("curpts", "curpts", dict(angles_deg="-60,-30,0,30,60,90", hold_ms=1200 if q else 2000,
                                  speeds_dps="30,90")),
        ("accept300", "track", dict(freq=50, tag="f50")),
        ("accept300", "track", dict(freq=300, tag="f300")),
        ("accept33", "track", dict(tag="v33")),
        ("overshoot", "overshoot", dict(reps=2 if q else 6)),
        ("heat", "heat", dict(level_s=300 if q else 600, cool_max_s=600)),
    ]
    paths = []
    for k, (test, prog, kw) in enumerate(plan):
        if prog in ("curpts", "heat"):
            bench.cfg["mass_g"] = bench.cfg.get("mass_g") or 1500.0
        rec = bench.run(prog, **kw)
        stem = out / servo_id / ("%02d_%s_%s" % (k, test, prog))
        paths.append(rec.write(stem, dict(servo_id=servo_id, servo_class=bench.cfg["cls"], test=test,
                                          host_time="2026-10-23T12:%02d:00" % k)))
    return paths
