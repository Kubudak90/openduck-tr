# -*- coding: utf-8 -*-
"""
Sensor modeli (OpenDuck-TR spec 3.2.4, A9): firmware'in birebir kopyalayacagi TEK tanim (tr35_b2 profili).

Algilama 200 Hz = her fizik alt adimi (SIM_DT); politika 50 Hz kontrol adiminda son algilama degerlerini okur.
Aktor gozlemi bu modelin ciktisini gorur; oduller ve kritik GERCEK durumu gorur (env_cpu / env_mjx).

Pot (OBS_JOINTS'in 14'u; servo cikis mili, bosluk eklemi varsa ana eklem):
  analog  x = q + bias_j + gnd_j * I_j - (r_cm * I_rail / V_ray * [LEG] + eps_ray) * (q - lo_j)
          bias_j bolum basina U(+-1.5 derece); gnd_j toprak-dusumu egimi U(+-k) * servo besleme akimi I_j;
          ortak-mod: ray dusumu (yalniz LEG, gercek ray akimi) + bolum basina kalibrasyon kazanc hatasi; lo_j potun
          alt ucu (eklem acisi; pot merkezi - acikligin yarisi).
  RC      y_k = y_{k-1} + a_rc (x_k - y_{k-1}),  a_rc = 1 - exp(-dt / 3.2 ms)  (100k/47k + 100 nF)
  yol     ADC1 (bacaklar, 12-bit): deger_k = y_{k-1} (1 alt adim gecikme).
          ADS1115 #1 (boyun + 3 kafa; 16-bit, 860 SPS, 4 kanal mux, ~215 Hz/kanal): deger_k = y(t_k - lag_c),
          lag_c = (kanal c'nin son donusumunun yasi) + T_conv/2, y_{k-2}, y_{k-1}, y_k arasinda dogrusal ara deger;
          mux fazi bolum basina rastgele, donusum hizi +-%10.
  sayisal v_k = lo_j + LSB_j * round((deger_k + beyaz gurultu - lo_j) / LSB_j)
  hiz     qd_k = qd_{k-1} + alpha ((v_k - v_{k-1}) / dt - qd_{k-1}), alpha 0.47 (200 Hz fark + IIR)
  gozlem  q_obs = v, qd_obs = qd (son algilama). Gaga olculmez (gozlemde de yok).
IMU (MPU6500, govde imu sitesi; montaj hatasi R bolum basina +-2 derece):
  gyro    w = (1 + s) * (R w_true) + b + gurultu
  ivme    a = R a_true + b_a + (sigma_a + k_vib * I_rail) * gurultu      (a_true: MuJoCo accelerometer, ozgul kuvvet)
  filtre  g' = g - dt (w x g);  d = -a/|a|;  k = k_cf * max(0, 1 - ((|a|/g0 - 1) / gate)^2)
          g = normalize(g' + k (d - g'))        (tamamlayici filtre; gravity gozlemi bu)
Gurultu bayragi (noise=False): bolum biaslari 0, montaj birim, mux fazi 0 / hiz nominal, beyaz gurultu 0; RC, gecikme,
mux, kuantalama, IIR ve filtre her zaman calisir.

Rastgele sayilar: episode() n_episode(n) adet, her alt adim tick n_tick(n) adet U[0,1) (beyaz gurultu duzgun,
genlik sigma*sqrt(3)). Fonksiyonlar xp (numpy | jax.numpy) ilk arguman, toplu ya da tek ortam.
"""
import numpy as np

from . import config as C
from . import task_math as T

SQ3 = float(np.sqrt(3.0))

POT = {
    "rc_tau": 3.2e-3,                    # s
    "bias_deg": 1.5,                     # bolum basina U(+-)
    "gnd_deg_per_a": 1.0,                # toprak-dusumu egimi U(+-) derece/A (20 AWG yeniden kablolama, pot rayda)
    "cm_r_max": 0.01,                    # ohm, LEG ortak-mod ray dusumu U(0, r) (gercek ray akimiyla)
    "cm_eps": 0.005,                     # ray basina kalibrasyon kazanc hatasi U(+-)
    "iir_alpha": 0.47,
    "adc1": {"sigma_deg": 0.15, "lsb_deg": 0.12},          # 12 dB, 100k/47k, pot 6.5 V: ~0.12 derece/LSB
    "ads": {"sigma_deg": 0.05, "lsb_deg": 0.02, "sps": 860.0, "rate_tol": 0.10, "nch": 4},
    "ads_joints": ("neck_pitch", "head_pitch", "head_yaw", "head_roll"),   # ADS1115 #1 (0x48) kanal sirasi
    "span_deg": {"ds32xx": 180.0, "es3054": 145.0, "mg90s": 180.0, "sg90": 180.0},
    "center_deg": {"left_hip_pitch": 14.4, "right_hip_pitch": -14.4, "neck_pitch": -25.0},   # pot merkezi (eklem)
}
IMU = {
    "gyro_sigma": 0.02, "gyro_bias": 0.01, "gyro_scale": 0.02,       # rad/s, rad/s (acilis kalibrasyonu sonrasi), -
    "mount_deg": 2.0,
    "acc_sigma": 0.03, "acc_bias": 0.2, "vib_per_a": 0.05,           # m/s^2, m/s^2, (m/s^2)/A (ust sinir; U(0, .))
    "cf_gain": 0.01, "cf_gate": 0.3, "g": 9.81,                      # 200 Hz'de tau ~0.5 s
}


def n_episode(n):
    """episode()'un tukettigi U[0,1) sayisi (n: pot sayisi)."""
    return 2 * n + 18


def n_tick(n):
    """Bir algilama tick'inin beyaz gurultu sayisi: n pot + 3 gyro + 3 ivme."""
    return n + 6


def pot_table(joint_names, classes, rails, volts):
    """Gozlem eklemleri icin statik pot tablosu (numpy). classes: eklem basina servo sinifi; rails: "leg" | "aux";
    volts: ray gerilimi. Anahtarlar: ads (0/1), ads_ch, lsb, sigma (rad), lo (rad), on_leg (0/1), rail (0 leg/1 aux),
    volt, a_rc, alpha."""
    n = len(joint_names)
    ads = np.array([1.0 if j in POT["ads_joints"] else 0.0 for j in joint_names])
    ch = np.array([float(POT["ads_joints"].index(j)) if j in POT["ads_joints"] else 0.0 for j in joint_names])
    path = lambda k: np.array([POT["ads" if a > 0.5 else "adc1"][k] for a in ads])
    span = np.deg2rad([POT["span_deg"][c] for c in classes])
    center = np.deg2rad([POT["center_deg"].get(j, 0.0) for j in joint_names])
    return {"ads": ads, "ads_ch": ch, "lsb": np.deg2rad(path("lsb_deg")), "sigma": np.deg2rad(path("sigma_deg")),
            "lo": center - 0.5 * span, "on_leg": np.array([1.0 if r == "leg" else 0.0 for r in rails]),
            "rail": np.array([0.0 if r == "leg" else 1.0 for r in rails]), "volt": np.asarray(volts, dtype=float),
            "a_rc": float(1.0 - np.exp(-C.SIM_DT / POT["rc_tau"])), "alpha": float(POT["iir_alpha"]), "n": n}


def rot_xyz(xp, a):
    """Kucuk montaj donusu R = Rz(a2) Ry(a1) Rx(a0), a (..., 3) -> (..., 3, 3)."""
    ca, sa = xp.cos(a[..., 0]), xp.sin(a[..., 0])
    cb, sb = xp.cos(a[..., 1]), xp.sin(a[..., 1])
    cc, sc = xp.cos(a[..., 2]), xp.sin(a[..., 2])
    r0 = xp.stack([cb * cc, sa * sb * cc - ca * sc, ca * sb * cc + sa * sc], axis=-1)
    r1 = xp.stack([cb * sc, sa * sb * sc + ca * cc, ca * sb * sc - sa * cc], axis=-1)
    r2 = xp.stack([-sb, sa * cb, ca * cb], axis=-1)
    return xp.stack([r0, r1, r2], axis=-2)


def _mv(xp, R, v):
    return xp.sum(R * v[..., None, :], axis=-1)


def episode(xp, u, P, noise=True):
    """u (..., n_episode(n)) U[0,1) -> bolum basina sensor parametreleri: bias, gnd (rad/A), cm_r (ohm), cm_eps (ray
    basina, eklemlere dagitilmis (..., n)), ads_tc (donusum suresi, s), ads_phase (mux fazi, s), gb, gs, R, ab, vib."""
    n = P["n"]
    U = lambda o, k: u[..., o:o + k]
    sym = lambda o, k, a: a * (2.0 * U(o, k) - 1.0)
    tc0 = 1.0 / POT["ads"]["sps"]
    if not noise:                      # her alan ayri dizi (numpy'de yerinde satir yazimi icin)
        z = lambda k: U(0, k) * 0.0
        zs = lambda: u[..., 0] * 0.0
        return {"bias": z(n), "gnd": z(n), "cm_r": zs(), "cm_eps": z(n), "ads_tc": zs() + tc0, "ads_phase": zs(),
                "gb": z(3), "gs": z(3), "R": rot_xyz(xp, z(3)), "ab": z(3), "vib": zs()}
    o = 2 * n
    eps = sym(o + 1, 2, POT["cm_eps"])                  # [LEG, AUX]
    tol = POT["ads"]["rate_tol"]
    tc = tc0 / (1.0 - tol + 2.0 * tol * u[..., o + 3])
    return {"bias": sym(0, n, np.deg2rad(POT["bias_deg"])),
            "gnd": sym(n, n, np.deg2rad(POT["gnd_deg_per_a"])),
            "cm_r": POT["cm_r_max"] * u[..., o],
            "cm_eps": eps[..., 0:1] * (1.0 - P["rail"]) + eps[..., 1:2] * P["rail"],
            "ads_tc": tc, "ads_phase": u[..., o + 4] * POT["ads"]["nch"] * tc,
            "gb": sym(o + 5, 3, IMU["gyro_bias"]), "gs": sym(o + 8, 3, IMU["gyro_scale"]),
            "R": rot_xyz(xp, sym(o + 11, 3, np.deg2rad(IMU["mount_deg"]))),
            "ab": sym(o + 14, 3, IMU["acc_bias"]), "vib": IMU["vib_per_a"] * u[..., o + 17]}


# --- pot yolu ----------------------------------------------------------------------------------
def quantize(xp, x, lo, lsb):
    """ADC kodu izgarasi: lo + lsb * round((x - lo) / lsb)."""
    return lo + lsb * xp.round((x - lo) / lsb)


def rc_step(xp, y, x, a):
    return y + a * (x - y)


def iir_qd(xp, qdf, v_new, v_old, alpha, dt=C.SIM_DT):
    """200 Hz fark + birinci derece IIR."""
    return qdf + alpha * ((v_new - v_old) / dt - qdf)


def pot_analog(xp, q, i_sup, i_rail, ep, P):
    """Potun bolucu girisindeki aci karsiligi (RC oncesi): gercek aci + bias + toprak dusumu + ortak-mod hata."""
    g = ep["cm_r"][..., None] * i_rail[..., None] / P["volt"] * P["on_leg"] + ep["cm_eps"]
    return q + ep["bias"] + ep["gnd"] * i_sup - g * (q - P["lo"])


def ads_lag(xp, phase, tc, ch):
    """ADS1115 mux: tick anindaki faz (s, mux cevrimi icinde) -> kanal basina gecikme (s) = son donusumun yasi +
    T_conv/2 (donusum ortalamasi). phase (...), tc (...), ch (n,) -> (..., n)."""
    tch = 4.0 * tc[..., None]
    age = xp.mod(phase[..., None] - (ch + 1.0) * tc[..., None], tch)
    return age + 0.5 * tc[..., None]


def pot_reset(xp, q, qd, ep, P):
    """Sifirlamada pot durumu (filtreler oturmus; akim 0, gurultu yok): {y1, y2, v, qdf, p}."""
    x = pot_analog(xp, q, q * 0.0, q[..., 0] * 0.0, ep, P)
    return {"y1": x, "y2": x + 0.0, "v": quantize(xp, x, P["lo"], P["lsb"]), "qdf": qd + q * 0.0,
            "p": ep["ads_phase"] + 0.0}


def pot_tick(xp, st, q, i_sup, i_rail, un, ep, P, noise=True, dt=C.SIM_DT):
    """Bir algilama tick'i (alt adim sonu). q (..., n) servo cikis acisi, i_sup (..., n) servo besleme akimi, i_rail
    (...) LEG ray akimi, un (..., n) U[0,1). Dondurur yeni durum {y1, y2, v, qdf, p}."""
    x = pot_analog(xp, q, i_sup, i_rail, ep, P)
    y = rc_step(xp, st["y1"], x, P["a_rc"])
    p = xp.mod(st["p"] + dt, 4.0 * ep["ads_tc"])
    w = xp.minimum(ads_lag(xp, p, ep["ads_tc"], P["ads_ch"]) / dt, 2.0)
    y_ads = xp.where(w <= 1.0, y + w * (st["y1"] - y), st["y1"] + (w - 1.0) * (st["y2"] - st["y1"]))
    raw = xp.where(P["ads"] > 0.5, y_ads, st["y1"])
    if noise:
        raw = raw + P["sigma"] * SQ3 * (2.0 * un - 1.0)
    v = quantize(xp, raw, P["lo"], P["lsb"])
    return {"y1": y, "y2": st["y1"], "v": v, "qdf": iir_qd(xp, st["qdf"], v, st["v"], P["alpha"], dt), "p": p}


# --- IMU ve tamamlayici filtre -------------------------------------------------------------------
def imu_reset(xp, quat, gyro, ep):
    """Sifirlamada {g: R * gercek izdusumlu yercekimi (filtre oturmus), w: olculen gyro (gurultusuz)}."""
    return {"g": _mv(xp, ep["R"], T.projected_gravity(xp, quat)),
            "w": (1.0 + ep["gs"]) * _mv(xp, ep["R"], gyro) + ep["gb"]}


def complementary(xp, g, w, a, dt=C.SIM_DT):
    """Tamamlayici filtre adimi: gyro ile on kestirim, ivme yonune k ile duzeltme (|a| ~ g0 iken), normalize."""
    gp = g - dt * xp.cross(w, g)
    an = xp.sqrt(xp.sum(xp.square(a), axis=-1))
    d = -a / xp.maximum(an, 1e-6)[..., None]
    r = (an / IMU["g"] - 1.0) / IMU["cf_gate"]
    k = IMU["cf_gain"] * xp.maximum(1.0 - xp.square(r), 0.0)
    gn = gp + k[..., None] * (d - gp)
    return gn / xp.maximum(xp.sqrt(xp.sum(xp.square(gn), axis=-1)), 1e-9)[..., None]


def imu_tick(xp, st, gyro, acc, i_rail, un, ep, noise=True, dt=C.SIM_DT):
    """gyro/acc (..., 3) gercek (govde cercevesi), i_rail (...) LEG ray akimi, un (..., 6) U[0,1)."""
    w = (1.0 + ep["gs"]) * _mv(xp, ep["R"], gyro) + ep["gb"]
    a = _mv(xp, ep["R"], acc) + ep["ab"]
    if noise:
        w = w + IMU["gyro_sigma"] * SQ3 * (2.0 * un[..., 0:3] - 1.0)
        amp = IMU["acc_sigma"] + ep["vib"] * i_rail
        a = a + amp[..., None] * SQ3 * (2.0 * un[..., 3:6] - 1.0)
    return {"g": complementary(xp, st["g"], w, a, dt), "w": w}


def quat_from_gravity(xp, g):
    """Izdusumlu yercekimi g (birim, govde) -> projected_gravity(quat) == g olan en kucuk donus kuaterniyonu
    [w,x,y,z] (g'yi [0,0,-1]'e dondurur). Tam ters (g = [0,0,1]) icin x etrafinda 180 derece."""
    c = 1.0 - g[..., 2]
    vx, vy = -g[..., 1], g[..., 0]
    n = xp.sqrt(c * c + vx * vx + vy * vy)
    ok = n > 1e-9
    s = 1.0 / xp.where(ok, n, 1.0)
    z = g[..., 0] * 0.0
    q = xp.stack([c * s, vx * s, vy * s, z], axis=-1)
    flip = xp.stack([z, z + 1.0, z, z], axis=-1)
    return xp.where(ok[..., None], q, flip)


def obs_state(xp, s, q_obs, qd_obs, gyro_obs, g_obs):
    """Aktor gozlemi icin durum: s'nin kopyasi, q/qd/gyro olculen, quat filtre ciktisindan (task_math.observation
    yalniz projected_gravity(quat) kullanir). Oduller ve kritik s'yi (gercek) kullanir."""
    o = dict(s)
    o.update(q=q_obs, qd=qd_obs, gyro=gyro_obs, quat=quat_from_gravity(xp, g_obs))
    return o
