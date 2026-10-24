# -*- coding: utf-8 -*-
"""
Servo aktuator modeli (OpenDuck-TR spec 3.2.3, A3, A9): robot profili, servo sinifi parametreleri, bolum basina
alan rastgelelestirme (spec 4), alt adim ctrl sekillendirmesi, besleme akimi / ray CC olceklemesi, olcut kancalari.
Tezgah sysID sonuclari (tools/sysid) bu dosyadaki CLASSES tablosuna yazilir.

Profiller (config.ROBOT_PROFILE, ortam degiskeni OPENDUCK_ROBOT):
  "xl330_27"  27 cm XL330 robotu: MJCF aktuatorleri aynen; bu modulun hicbir modeli calismaz (walk bit-duzeyinde ayni).
  "tr35_b2"   PWM servolar. Parametreler Adim 0'a kadar spec varsayilanlari (DS3230MG Blue 2.88 N.m, 5.72 rad/s,
              P-bandi 3 derece). Yuklenen MJCF'ye eklem -> sinif haritasiyla (JOINT_CLASS) uygulanir; TR-35 MJCF'si
              yokken 27 cm MJCF'de de calisir (olcek bu modelin testlerini etkilemez).

Tanimlar (spec 3.2.3):
  kp = tau_s / P-bandi, forcerange = +-tau_s, eklem sonumu b = tau_s / omega0 (geri-EMK), armature, frictionloss.
  tau_act = actuator_force; tau_net = tau_act - b*qd (motor torku, akimla orantili).
  I_i = I_stall,i * |tau_act,i| * |tau_net,i| / tau_s,i^2 + I_idle,i ; LEG rayi I_rail = sum_DS I_i.
  Talep I_CC'yi asarsa DS forcerange'leri I_CC / I_rail ile olceklenir; talep metrigi olcekleme ONCESI degerdir.
  Eklem siniri degismezi (tr35): aktuatorlu ana eklemlerde range = ctrlrange +- JOINT_RANGE_MARGIN_DEG, butun varyant
  sinirlayicilarindan SONRA (robot_model.build_spec). Bosluk eklemleri ("<eklem>_play") disinda.

Alt adim k = 0..N_SUBSTEPS-1 (iki arka uc ayni sirayla; ortam kodunda):
  1. c = k < gecikme_j ? ctrl_onceki_j : ctrl_yeni_j     gecikme bolum basina kart basina: U{0..2} (300 Hz PCA 0x40,
                                                          DS), U{0..4} (50 Hz PCA 0x41: ES3054, MG90S, SG90)
     (ctrl_yeni kontrol adiminda PWM kuantalanir: quantize_pwm)
  2. c = q + olu_bant(c - q)                              |e| <= db/2 -> 0, disarida db/2 kaydirma (q: servo cikisi)
  3. tau_pred = MuJoCo position kuvveti (ctrlrange kirpmali, +-tau_s); talep = sum_DS I(tau_pred, tau_pred - b*qd)
  4. forcerange_DS = +-tau_s * min(1, I_CC / talep); fizik alt adimi
  5. tau_act = actuator_force (alt adim sonrasi); tau_net = tau_act - b*qd_onceki; gercek akim -> sensor modeli
Bosluk (istege bagli, backlash=True): DS eklemlerinin govdesine ayni eksende seri pasif "<eklem>_play" mafsali
(+-b/2, armature 1e-5, sert solref). Ara govde kutlesizdir: servo cikisinin ataleti ana eklemin armature'u, link
ataleti govdede; pot servo cikis milini (ana eklem) okur. Limit metrigi bosluk eklemlerini saymaz.

Fonksiyonlar xp (numpy | jax.numpy) ilk arguman, toplu ya da tek ortam (task_math kurali: [..., i], axis=-1).
"""
import mujoco
import numpy as np

TR35 = "tr35_b2"
MARK = "openduck_robot:"           # derlenmis modelde profil isareti (mjOBJ_TEXT adi)
PLAY_SUFFIX = "_play"

# --- servo siniflari (spec 3.1; research servo-mechanics, power-electronics). Adim 0'a kadar varsayilan. --------
# tau_s N.m (ray gerilimindeki stall), omega0 rad/s (bosta hiz), pband_deg (P-bandi: %100 doluluktaki hata),
# armature kg.m2 (yansiyan rotor ataleti), friction_frac (frictionloss / tau_s), deadband_deg (tam genislik),
# pwm_res_deg (PCA9685 adimi), i_stall / i_idle A, board 0: 300 Hz PCA 0x40 / 1: 50 Hz PCA 0x41, rail "leg" | "aux"
CLASSES = {
    "ds32xx": dict(tau_s=2.88, omega0=5.72, pband_deg=3.0, armature=5e-3, friction_frac=0.03, deadband_deg=0.27,
                   pwm_res_deg=0.072, i_stall=3.63, i_idle=0.005, board=0, rail="leg"),
    "es3054": dict(tau_s=0.343, omega0=8.06, pband_deg=3.0, armature=7.5e-4, friction_frac=0.03, deadband_deg=0.5,
                   pwm_res_deg=0.442, i_stall=1.0, i_idle=0.01, board=1, rail="aux"),
    "mg90s": dict(tau_s=0.216, omega0=13.1, pband_deg=3.0, armature=2.5e-4, friction_frac=0.03, deadband_deg=0.45,
                  pwm_res_deg=0.44, i_stall=0.9, i_idle=0.01, board=1, rail="aux"),
    "sg90": dict(tau_s=0.177, omega0=10.5, pband_deg=3.0, armature=2.5e-4, friction_frac=0.03, deadband_deg=0.9,
                 pwm_res_deg=0.44, i_stall=0.75, i_idle=0.005, board=1, rail="aux"),
}
CLASS_NAMES = tuple(CLASSES)
# eklem (sol/sag oneki olmadan) -> sinif
JOINT_CLASS = {TR35: {
    "hip_yaw": "es3054", "hip_roll": "ds32xx", "hip_pitch": "ds32xx", "knee": "ds32xx", "ankle": "ds32xx",
    "neck_pitch": "ds32xx", "head_pitch": "mg90s", "head_yaw": "mg90s", "head_roll": "mg90s", "mouth": "sg90",
}}
RAILS = {"leg": dict(volt=6.5, i_cc=16.0), "aux": dict(volt=6.0, i_cc=None)}   # 20 A buck CC ~16 A; XL4015
I_CC = RAILS["leg"]["i_cc"]

# config buradan sonra: tr35_b2 surecinde config import'unun sonunda CLASSES/JOINT_CLASS okunur (bas egim siniri);
# hangi modul once yuklenirse yuklensin dongusel import bu tablolar tanimliyken olur
from . import config as C  # noqa: E402

PROFILES = tuple(C.ROBOT_PROFILES)

# bolum basina DR (spec 4; Adim 0 sonrasi kabulde olculenin +-%15'ine daralir). Carpanlar nominale; P-bandi
# log-duzgun (derece); bosluk tam genislik (derece, yalniz DS); gecikme alt adim (kart 0, kart 1).
DR = {
    "tau_s": (0.7, 1.15), "omega0": (0.75, 1.4), "pband_deg": (1.0, 8.0), "deadband": (0.5, 2.0),
    "armature": (0.5, 2.0), "frictionloss": (0.5, 2.0), "backlash_deg": (0.0, 1.0),
    "delay_substeps": ((0, 2), (0, 4)),
}
BACKLASH = dict(nominal_deg=0.5, armature=1e-5, solref=(2.0 * C.SIM_DT, 1.0))   # solref: timestep alt siniri 2*dt
# tr35'te genel DR'in aktuator carpanlari kapanir (yerine yukaridaki DR); kutle spec 4 araligina genisler.
DR_OVERRIDES = {TR35: {"kp_scale": (1.0, 1.0), "damping_scale": (1.0, 1.0), "frictionloss_scale": (1.0, 1.0),
                       "armature_scale": (1.0, 1.0), "link_mass_scale": (0.9, 1.15), "action_delay_steps": (0, 0)}}
N_FIELDS = 7                       # sample_params alanlari (nu): tau_s, omega0, pband, deadband, armature, fl, bosluk


def profile(robot=None):
    """None -> config.ROBOT_PROFILE; gecersiz ad -> ValueError."""
    r = C.ROBOT_PROFILE if robot is None else str(robot)
    if r not in PROFILES:
        raise ValueError("robot profili %r (gecerli: %s)" % (r, ", ".join(PROFILES)))
    return r


def env_profile(robot=None):
    """Ortamlarin robot= dogrulamasi: profile(robot); tr35_b2 SURECINDE (config.ROBOT_PROFILE, geometri/gorev profili)
    yalniz None | "tr35_b2" (xl330 surecinde robot="tr35_b2" 27 cm MJCF'de de calisir)."""
    r = profile(robot)
    if C.ROBOT_PROFILE == TR35 and r != TR35:
        raise ValueError("tr35_b2 surecinde robot=%r gecersiz (None ya da %r; OPENDUCK_ROBOT surec profilini secer)"
                         % (robot, TR35))
    return r


def model_profile(model):
    """Derlenmis modelin profili (build_spec isareti; isaretsiz model xl330_27)."""
    for r in PROFILES:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, MARK + r) >= 0:
            return r
    return PROFILES[0]


def base_joint(name):
    for p in ("left_", "right_"):
        if name.startswith(p):
            return name[len(p):]
    return name


def joint_class(robot, joint):
    """Eklem adi (sol/sag onekli) -> servo sinifi."""
    return JOINT_CLASS[profile(robot)][base_joint(joint)]


def domain_rand(robot=None):
    """Ortamlarin genel DR sozlugu (xl330: config.DOMAIN_RAND nesnesinin kendisi; tr35: DR_OVERRIDES'li kopya)."""
    r = profile(robot)
    if r not in DR_OVERRIDES:
        return C.DOMAIN_RAND
    d = dict(C.DOMAIN_RAND)
    d.update(DR_OVERRIDES[r])
    return d


# --- MjSpec (robot_model.build_spec) -------------------------------------------------------------
def _set_damping(joint, b):
    d = np.array(joint.damping, dtype=float).reshape(-1)
    d[:] = 0.0
    d[0] = b
    joint.damping = d if d.size > 1 else float(b)


def apply_to_spec(spec, robot, backlash=False):
    """Profilin aktuator parametrelerini MjSpec'e uygular (xl330: hicbir sey). backlash=True: DS eklemlerine seri
    bosluk mafsali (nominal genislik BACKLASH["nominal_deg"]) + keyframe qpos'una sifir eklenir."""
    robot = profile(robot)
    if robot not in JOINT_CLASS:
        if backlash:
            raise ValueError("bosluk eklemi yalniz %s profilinde" % TR35)
        return spec
    play = []
    for a in spec.actuators:
        j = spec.joint(a.target)
        p = CLASSES[joint_class(robot, j.name)]
        ts = float(p["tau_s"])
        kp = ts / np.deg2rad(p["pband_deg"])
        gp, bp = np.array(a.gainprm, dtype=float), np.array(a.biasprm, dtype=float)
        gp[0], bp[0], bp[1], bp[2] = kp, 0.0, -kp, 0.0
        a.gainprm, a.biasprm = gp, bp
        a.forcerange = np.array([-ts, ts])
        a.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        _set_damping(j, ts / p["omega0"])
        j.armature = float(p["armature"])
        j.frictionloss = float(p["friction_frac"] * ts)
        if backlash and p["rail"] == "leg":
            play.append(j)
    if play:
        m0 = spec.copy().compile()      # kopya: ozgun spec derlenmez (sonraki doku silme vb. bozulmasin)
        jid = lambda n: mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_JOINT, n)
        q_ins = sorted(int(m0.jnt_qposadr[jid(j.name)]) + 1 for j in play)
        v_ins = sorted(int(m0.jnt_dofadr[jid(j.name)]) + 1 for j in play)
        h = 0.5 * np.deg2rad(BACKLASH["nominal_deg"])
        for j in play:
            jp = j.parent.add_joint(name=j.name + PLAY_SUFFIX, type=mujoco.mjtJoint.mjJNT_HINGE,
                                    axis=np.array(j.axis, dtype=float), pos=np.array(j.pos, dtype=float))
            jp.range = np.array([-h, h])
            jp.limited = mujoco.mjtLimited.mjLIMITED_TRUE
            jp.armature = float(BACKLASH["armature"])
            _set_damping(jp, 0.0)
            jp.frictionloss = 0.0
            jp.solref_limit = np.array(BACKLASH["solref"], dtype=float)
        for k in spec.keys:
            if len(k.qpos):
                k.qpos = np.insert(np.array(k.qpos, dtype=float), q_ins, 0.0)
            if len(k.qvel):
                k.qvel = np.insert(np.array(k.qvel, dtype=float), v_ins, 0.0)
    spec.add_text(name=MARK + robot, data=robot)
    return spec


def enforce_range_margin(spec, margin_deg=None):
    """Degismez: aktuatorlu ana eklemlerde range = ctrlrange +- margin (bosluk eklemleri aktuatorsuz, disarida).
    build_spec'te butun varyant sinirlayicilarindan SONRA cagrilir."""
    mg = np.deg2rad(C.JOINT_RANGE_MARGIN_DEG if margin_deg is None else margin_deg)
    for a in spec.actuators:
        cr = np.array(a.ctrlrange, dtype=float)
        j = spec.joint(a.target)
        j.range = np.array([cr[0] - mg, cr[1] + mg])
        j.limited = mujoco.mjtLimited.mjLIMITED_TRUE
    return spec


def actuator_table(model):
    """Derlenmis tr35 modeli icin aktuator sirasinda nominal parametreler (numpy) ve adresler.
    Anahtarlar: names, cls, tau_s, omega0, pband, kp, damping, armature, frictionloss, deadband (tam genislik, rad),
    pwm_res (rad), i_stall, i_idle, board (int), on_leg (0/1, DS = LEG rayi), ctrl_lo/hi, act_jnt, act_qadr,
    act_dadr, play_jnt / play_qadr / play_dadr (aktuator basina, yoksa -1), backlash (nominal tam genislik, rad;
    bosluksuz 0), jnt2act (njnt,) ana aktuatorlu eklem -> aktuator indeksi (yoksa -1)."""
    m = model
    robot = model_profile(m)
    if robot not in JOINT_CLASS:
        raise ValueError("aktuator tablosu yalniz %s modelinde (model %s)" % (TR35, robot))
    jname = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, int(i))
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    act_jnt = np.array([m.actuator_trnid[a, 0] for a in range(m.nu)], dtype=np.int64)
    names = [jname(j) for j in act_jnt]
    cls = [joint_class(robot, n) for n in names]
    col = lambda k: np.array([float(CLASSES[c][k]) for c in cls])
    t = {"names": names, "cls": cls, "tau_s": col("tau_s"), "omega0": col("omega0"),
         "pband": np.deg2rad(col("pband_deg")), "armature": col("armature"),
         "deadband": np.deg2rad(col("deadband_deg")), "pwm_res": np.deg2rad(col("pwm_res_deg")),
         "i_stall": col("i_stall"), "i_idle": col("i_idle"),
         "board": np.array([int(CLASSES[c]["board"]) for c in cls], dtype=np.int64),
         "on_leg": np.array([1.0 if CLASSES[c]["rail"] == "leg" else 0.0 for c in cls])}
    t["kp"] = t["tau_s"] / t["pband"]
    t["damping"] = t["tau_s"] / t["omega0"]
    t["frictionloss"] = col("friction_frac") * t["tau_s"]
    t["ctrl_lo"] = m.actuator_ctrlrange[:, 0].copy()
    t["ctrl_hi"] = m.actuator_ctrlrange[:, 1].copy()
    t["act_jnt"] = act_jnt
    t["act_qadr"] = np.array(m.jnt_qposadr[act_jnt], dtype=np.int64)
    t["act_dadr"] = np.array(m.jnt_dofadr[act_jnt], dtype=np.int64)
    pj = np.array([jid(n + PLAY_SUFFIX) for n in names], dtype=np.int64)
    t["play_jnt"] = pj
    t["play_qadr"] = np.where(pj >= 0, m.jnt_qposadr[np.maximum(pj, 0)], -1)
    t["play_dadr"] = np.where(pj >= 0, m.jnt_dofadr[np.maximum(pj, 0)], -1)
    t["backlash"] = np.where(pj >= 0, np.deg2rad(BACKLASH["nominal_deg"]), 0.0)
    j2a = np.full(m.njnt, -1, dtype=np.int64)
    j2a[act_jnt] = np.arange(m.nu)
    t["jnt2act"] = j2a
    return t


# --- bolum basina parametreler (xp) ----------------------------------------------------------------
def n_uniform(nu):
    """sample_params'in tukettigi U[0,1) sayisi."""
    return N_FIELDS * nu + 2


def sample_params(xp, u, nom, randomize=True):
    """u (..., n_uniform(nu)) U[0,1) -> bolum parametreleri (aktuator sirasi, (..., nu)):
    tau_s, kp, damping (b = tau_s/omega0), armature, frictionloss, db_half (olu bant yarisi), backlash (tam genislik;
    yalniz bosluk eklemli aktuatorlerde > 0), delay (int32 alt adim). nom: actuator_table(...).
    randomize=False: nominal degerler, gecikme 0 (u yalniz sekil icin)."""
    nu = len(nom["tau_s"])
    z = u[..., :nu] * 0.0
    if not randomize:
        ts = nom["tau_s"] + z
        return {"tau_s": ts, "kp": nom["kp"] + z, "damping": nom["damping"] + z, "armature": nom["armature"] + z,
                "frictionloss": nom["frictionloss"] + z, "db_half": 0.5 * nom["deadband"] + z,
                "backlash": nom["backlash"] + z, "delay": (z + 0.0).astype(xp.int32)}
    U = lambda i: u[..., i * nu:(i + 1) * nu]
    lerp = lambda i, r: r[0] + (r[1] - r[0]) * U(i)
    ts = nom["tau_s"] * lerp(0, DR["tau_s"])
    w0 = nom["omega0"] * lerp(1, DR["omega0"])
    l0, l1 = np.log(DR["pband_deg"][0]), np.log(DR["pband_deg"][1])
    pb = np.deg2rad(1.0) * xp.exp(l0 + (l1 - l0) * U(2))
    has_play = (nom["play_jnt"] >= 0).astype(np.float64)
    out = {"tau_s": ts, "kp": ts / pb, "damping": ts / w0,
           "armature": nom["armature"] * lerp(4, DR["armature"]),
           "frictionloss": nom["frictionloss"] * lerp(5, DR["frictionloss"]),
           "db_half": 0.5 * nom["deadband"] * lerp(3, DR["deadband"]),
           "backlash": has_play * np.deg2rad(1.0) * lerp(6, DR["backlash_deg"])}
    (f0, f1), (s0, s1) = DR["delay_substeps"]
    o = N_FIELDS * nu
    fast = xp.minimum(f0 + xp.floor(u[..., o] * (f1 - f0 + 1)), f1)
    slow = xp.minimum(s0 + xp.floor(u[..., o + 1] * (s1 - s0 + 1)), s1)
    board = nom["board"]
    out["delay"] = xp.where(board == 0, fast[..., None], slow[..., None]).astype(xp.int32)
    return out


# --- alt adim ctrl sekillendirmesi ve akim modeli (xp) -------------------------------------------------
def quantize_pwm(xp, ctrl, res, lo, hi):
    """PCA9685 adimina yuvarlama (eklem sifirina hizali izgara), ctrlrange'e kirpilir. Kontrol adiminda bir kez."""
    return xp.clip(xp.round(ctrl / res) * res, lo, hi)


def delayed(xp, k, delay, prev, new):
    """Alt adim k'da servonun gordugu hedef: k < gecikme ise onceki kontrol adiminin hedefi."""
    return xp.where(k < delay, prev, new)


def dead_band(xp, ctrl, q, half):
    """Olu bant: e = ctrl - q; e' = sign(e) * max(|e| - half, 0); ctrl' = q + e' (P kuvveti bant icinde 0)."""
    e = ctrl - q
    return q + xp.sign(e) * xp.maximum(xp.abs(e) - half, 0.0)


def position_force(xp, ctrl, q, qd, gain, b0, b1, b2, lo, hi, frc):
    """MuJoCo position aktuator kuvveti (gear 1): gain*clip(ctrl, lo, hi) + b0 + b1*q + b2*qd, +-frc'ye kirpilir."""
    f = gain * xp.clip(ctrl, lo, hi) + b0 + b1 * q + b2 * qd
    return xp.clip(f, -frc, frc)


def tau_net(xp, tau_act, qd, damping):
    """tau_net = tau_act - b*qd (motor torku; isil/doygunluk olcutleri tau_net/tau_s)."""
    return tau_act - damping * qd


def supply_current(xp, tau_act, tnet, tau_s, i_stall, i_idle):
    """I = I_stall * |tau_act| * |tau_net| / tau_s^2 + I_idle (A)."""
    return i_stall * xp.abs(tau_act) * xp.abs(tnet) / xp.square(tau_s) + i_idle


def rail_current(xp, cur, on_leg):
    """LEG rayi akimi: DS servolarin toplami (..., nu) -> (...)."""
    return xp.sum(cur * on_leg, axis=-1)


def cc_scale(xp, demand, i_cc=I_CC):
    """DS forcerange carpani min(1, I_CC / talep)."""
    return xp.minimum(1.0, i_cc / xp.maximum(demand, 1e-9))


def shape_substep(xp, k, prev, new, q, qd, ep, nom, gain, b0, b1, b2, damping):
    """Alt adim k'nin servo girdisi. q/qd (..., nu) aktuatorlu eklemlerin mevcut (adim oncesi) durumu; ep:
    sample_params ciktisi (tau_s, db_half, delay); gain/b0/b1/b2: modelin aktuator prm'leri; damping: b.
    Dondurur (ctrl', forcerange ust siniri (..., nu), olcekleme oncesi LEG talebi (...), CC carpani (...))."""
    c = dead_band(xp, delayed(xp, k, ep["delay"], prev, new), q, ep["db_half"])
    f = position_force(xp, c, q, qd, gain, b0, b1, b2, nom["ctrl_lo"], nom["ctrl_hi"], ep["tau_s"])
    dem = rail_current(xp, supply_current(xp, f, tau_net(xp, f, qd, damping), ep["tau_s"], nom["i_stall"],
                                          nom["i_idle"]), nom["on_leg"])
    s = cc_scale(xp, dem)
    frc = ep["tau_s"] * (1.0 + nom["on_leg"] * (s[..., None] - 1.0))
    return c, frc, dem, s


def after_substep(xp, tau_act, qd_pre, damping, ep, nom):
    """Alt adim sonrasi: (tau_net (..., nu), gercek besleme akimi (..., nu), gercek LEG ray akimi (...))."""
    tn = tau_net(xp, tau_act, qd_pre, damping)
    cur = supply_current(xp, tau_act, tn, ep["tau_s"], nom["i_stall"], nom["i_idle"])
    return tn, cur, rail_current(xp, cur, nom["on_leg"])


def metrics_init(xp, like_nu):
    """Kontrol adimi olcutleri: tau_net (son alt adim), limit (alt adimlarin en buyugu |limit kisit torku|),
    rail (alt adim ortalamasi, olcekleme oncesi talep), rail_peak (en buyugu), cc (en kucuk CC carpani)."""
    z = lambda: like_nu * 0.0                    # her alan ayri dizi (numpy'de yerinde yazim)
    zs = lambda: like_nu[..., 0] * 0.0
    return {"tau_net": z(), "limit": z(), "rail": zs(), "rail_peak": zs(), "cc": zs() + 1.0}
