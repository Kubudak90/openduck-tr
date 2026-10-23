# -*- coding: utf-8 -*-
"""
Tek eklemli PWM servo modeli (spec 3.2.3 anlami; MuJoCo position aktuatoru + eklem parametreleri):

  c_d     = c(t - latency)                          gecikme olu banttan once, komuta uygulanir
  e       = c_d - q;  e' = sign(e)*max(|e| - deadband/2, 0);  ctrl' = q + e'
                                                    olu bant = hata olu bolgesi: her entegrasyon adiminda o anki
                                                    q'ya karsi; bant icinde P kuvveti 0 (hobi servo |komut - konum|
                                                    < db/2 iken motoru surmez). rl/actuators.py dead_band(ctrl, q,
                                                    half) ile ayni kural, half = deadband/2 (deadband tam genislik)
  tau_act = clip(kp*(ctrl' - q), -tau_s, tau_s)     kp = tau_s / p_band  (actuator_force)
  tau_net = tau_act - b*qd                          b = tau_s / omega0   (eklem sonumu, spec tork tanimi)
  (armature + J_yuk)*qdd = tau_net - frictionloss*sgn(qd) + tau_yuk(q)
  I_besleme = i_stall*|tau_act|*|tau_net|/tau_s^2 + i_idle            (spec 3.2.3)
Duragan: surtunme servoyu kp*(|e| - db/2) < frictionloss iken tutar, yani |e| < db/2 + frictionloss/kp; yari-duragan
histerezis genisligi db + 2*frictionloss/kp (play(c, db + 2*frictionloss/kp) esdegeri; fit.Dyn "play" blogu).
Dinamikte play'den farkli: bant icinde sertlik 0 (play kaydirilmis komut etrafinda tam kp tutardi).
Pot: aci q'nun dogrusal (+ kucuk egrilik) fonksiyonu; ADC pininde bolucu + RC (tau_rc).
Bosta hiz (yuksuz plato) = (tau_s - frictionloss)/b; omega0 model parametresidir (tau_s/b).
Ic dongu duz Python (numpy dizi yukunden hizli; ~1.5 us/adim). Birimler SI (rad, N.m, s, A).
"""
import math
from dataclasses import dataclass, asdict

import numpy as np

S_SCALE = 1.282          # spec 3.1 olcek
M_REF = 0.676            # 27 cm robot kutlesi (torque-needs)


@dataclass
class ServoParams:
    tau_s: float = 2.88                  # N.m
    omega0: float = 5.72                 # rad/s (tau_s/b)
    p_band: float = math.radians(3.0)    # rad
    deadband: float = math.radians(0.27)  # rad, toplam genislik
    armature: float = 3.0e-3             # kg.m^2
    frictionloss: float = 0.05           # N.m
    latency: float = 0.006               # s
    i_stall: float = 3.6                 # A
    i_idle: float = 0.02                 # A

    @property
    def kp(self):
        return self.tau_s / self.p_band

    @property
    def damping(self):
        return self.tau_s / self.omega0

    @property
    def omega_noload(self):
        return (self.tau_s - self.frictionloss) / self.damping

    def dict(self):
        return asdict(self)


@dataclass
class Load:
    """Kol yuku: ek atalet + yercekimi (kol yatayi q = q_h, +q yukari) + istege bagli duvar (terazi, q <= wall)."""
    inertia: float = 0.0     # kg.m^2 (kol + kutle)
    mgl: float = 0.0         # N.m (m*g*L)
    q_h: float = 0.0         # rad
    wall: float = None       # rad; kol bu acinin ustune cikamaz (stall: terazi)


def supply_current(p, tau_act, tau_net):
    return p.i_stall * np.abs(tau_act) * np.abs(tau_net) / p.tau_s ** 2 + p.i_idle


def simulate(p, t_cmd, c_cmd, t_out, load=None, q0=None, qd0=0.0, c_prev=None, dt=2.5e-4, rc_tau=3.2e-3):
    """Komut dizisi (t_cmd[k]'dan itibaren c_cmd[k], rad) ile t_out anlarinda durum.
    c_prev: ilk komut + gecikmeden once etkin olan (onceki) komut, rad; None: q0 (hata 0, motor bant icinde).
    Olu bant her alt adimda o anki q'ya karsi uygulanir (hata olu bolgesi, ustteki formul); gecmis durumu yoktur.
    Donus: dict q, qd, q_rc (RC suzulmus), tau_act, tau_net, i (besleme), c_end (sondaki etkin komut)."""
    load = load or Load()
    tc = [float(x) for x in t_cmd]
    cc = [float(x) for x in c_cmd]
    to = [float(x) for x in t_out]
    n, nc = len(to), len(tc)
    ts, tf, b = float(p.tau_s), float(p.frictionloss), float(p.damping)
    kp, hw, L = float(p.kp), 0.5 * float(p.deadband), float(p.latency)
    J = float(p.armature) + float(load.inertia)
    mgl, qh = float(load.mgl), float(load.q_h)
    wall = None if load.wall is None else float(load.wall)
    t = to[0]
    q = cc[0] if q0 is None else float(q0)
    qd = float(qd0)
    cp = q if c_prev is None else float(c_prev)
    qf = q
    # k: etkin komut indeksi (-1: ilk komut + gecikmeden once, c_prev etkin). Alt adimlar komut gecis anlarinda
    # (tc + L) bolunur: gecikme alt adim izgarasina kuantalanmaz, maliyet L'de surekli olur.
    k = -1
    while k + 1 < nc and tc[k + 1] + L <= t + 1e-12:
        k += 1
    oq, oqd, oqf, oa, on = [0.0] * n, [0.0] * n, [0.0] * n, [0.0] * n, [0.0] * n
    oia, ois = [0.0] * n, [0.0] * n
    ist, iid, its2 = float(p.i_stall), float(p.i_idle), 1.0 / (ts * ts)
    veps = 1e-4
    cos, exp = math.cos, math.exp
    ta = 0.0
    for i in range(n):
        tend = to[i]
        acc, accs, acct = 0.0, 0.0, 0.0
        while t < tend - 1e-12:
            h = dt if t + dt <= tend else tend - t
            while k + 1 < nc and tc[k + 1] + L <= t + 1e-12:
                k += 1
            if k + 1 < nc:
                tsw = tc[k + 1] + L
                if tsw < t + h:
                    h = tsw - t
            # olu bant (hata olu bolgesi): e = c - q; kp*(ctrl' - q) = kp*sign(e)*max(|e| - hw, 0)
            e = (cc[k] if k >= 0 else cp) - q
            if e > hw:
                ta = kp * (e - hw)
            elif e < -hw:
                ta = kp * (e + hw)
            else:
                ta = 0.0
            if ta > ts:
                ta = ts
            elif ta < -ts:
                ta = -ts
            f = ta - mgl * cos(q - qh) if mgl else ta
            if -veps < qd < veps and -tf <= f <= tf:
                qd = 0.0
            else:
                s = (1.0 if qd > 0 else -1.0) if (qd >= veps or qd <= -veps) else (1.0 if f > 0 else -1.0)
                qn = (qd + h * (f - tf * s) / J) / (1.0 + h * b / J)
                if qn * s < 0.0 and -tf <= f <= tf:
                    qn = 0.0
                qd = qn
            q += h * qd
            if wall is not None and q > wall:
                q, qd = wall, (qd if qd < 0.0 else 0.0)
            qf += (q - qf) * (1.0 - exp(-h / rc_tau)) if rc_tau > 0 else (q - qf)
            tn = ta - b * qd
            pr = ta * tn * its2
            acc += h * (pr if pr > 0 else -pr)
            accs += h * pr
            acct += h
            t += h
        oq[i], oqd[i], oqf[i], oa[i], on[i] = q, qd, qf, ta, ta - b * qd
        if acct > 0:
            oia[i], ois[i] = ist * acc / acct + iid, ist * accs / acct + iid
        else:
            pr = oa[i] * on[i] * its2
            oia[i], ois[i] = ist * abs(pr) + iid, ist * pr + iid
    out = dict(q=np.array(oq), qd=np.array(oqd), q_rc=np.array(oqf), tau_act=np.array(oa), tau_net=np.array(on))
    out["i"] = supply_current(p, out["tau_act"], out["tau_net"])
    # aralik ortalamali besleme akimi (INA226 ortalama okur); isaretli hali ray modeli icin (fren geri beslemesi)
    out["i_avg"], out["i_signed_avg"] = np.array(oia), np.array(ois)
    while k + 1 < nc and tc[k + 1] + L <= t + 1e-12:
        k += 1
    out["c_end"] = cc[k] if k >= 0 else cp
    return out


def play(c, width, y0=None):
    """Play (bosluk) operatoru: cikis, komutun +-width/2 bandinda kalir. Simulasyonda kullanilmaz; yalniz
    yari-duragan olu bant basamaklarinin statik esdegeri (fit.Dyn "play" blogu, width = db + 2*frictionloss/kp)."""
    hw = 0.5 * width
    y = c[0] if y0 is None else y0
    out = np.empty(len(c))
    for i, x in enumerate(c):
        if x - hw > y:
            y = x - hw
        elif x + hw < y:
            y = x + hw
        out[i] = y
    return out


# --- spec 7 kapisi: parts-research u-fit'i (kappa ile; rl-map D1) ---
def u_fit(tau_s, omega_noload, mass, kappa, s=S_SCALE):
    """u = max(0.02/te + 8.8k/we, 0.13/te + 7.7k/we, 0.5/te); te = tau_s/k_bacak, we = w0*sqrt(s),
    k_bacak = (m/0.676)*s. omega: yayimlanmis (bosta) hiz anlaminda olculen plato hizi."""
    kb = (mass / M_REF) * s
    te = tau_s / kb
    we = omega_noload * math.sqrt(s)
    return max(0.02 / te + 8.8 * kappa / we, 0.13 / te + 7.7 * kappa / we, 0.5 / te)


def knee_rms_estimate(mass, s=S_SCALE):
    """G-ISIL oncesi tahmin: 27 cm walk-DR diz RMS 0.30 N.m x k (review R2-4); P7 olcumuyle degistirin."""
    return 0.30 * (mass / M_REF) * s


# --- isil model (kasa NTC) ---
def thermal_step(t, t_amb, dT_ss, T0, tau_th):
    """Birinci derece: T(t) = Tamb + dTss(1-e^-t/tau) + (T0-Tamb) e^-t/tau."""
    e = np.exp(-np.asarray(t) / tau_th)
    return t_amb + dT_ss * (1.0 - e) + (T0 - t_amb) * e


def j0(x):
    """Bessel J0 (seri; |x| < 3 yeterli)."""
    s, term, k = 1.0, 1.0, 1
    while k < 40:
        term *= -(x * x / 4.0) / (k * k)
        s += term
        if abs(term) < 1e-14:
            break
        k += 1
    return s


def heat_center(tau_rms, mgl, amp):
    """q = qc + A sin: RMS(mgl cos q) = tau_rms icin qc (rad, <= 0); None: ulasilamaz."""
    if mgl <= 0:
        return None
    r = (2.0 * (tau_rms / mgl) ** 2 - 1.0) / j0(2.0 * amp)
    if r > 1.0 or r < -1.0:
        return None
    return -0.5 * math.acos(r)
