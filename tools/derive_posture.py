# -*- coding: utf-8 -*-
"""
TR-35 geometri/durus sabitleri turetme araci (OpenDuck-TR P5a; spec 3.2.5, research/rl-map D1). Geometriye bagli
her tr35 sabiti (H_* yukseklikleri, comelme ailesi F, SEAT, HEAD_SIT/TUCK, ayak mesafesi, eklem sinirlari,
EXTRA_COLLISION kutulari, oz-carpisma ciftleri, HOME) bir MJCF'den uretilir; rl/config.py tr35 surecinde
rl/derived/tr35_b2.json'u (sema 1, P5a ortak arayuz 3) okur. G-CAD sonrasi tek komut yeniden uretir:

  python tools/derive_posture.py derive --scene build/tr35_b2/sim/scene.xml --out rl/derived/tr35_b2.json
  python tools/derive_posture.py derive --scene <stub>/sim/scene.xml --out rl/derived/tr35_b2.json --stub
  python tools/derive_posture.py check --scene S [--derived rl/derived/tr35_b2.json]
         # 0 gecerli + taze, 3 gecerli ama MJCF (ya da mesh) sha'si farkli, 1 gecersiz
  python tools/derive_posture.py caps-scan --scene S [--derived P] [--json out.json]
         # gorsel mesh voxel taramasi (sim/collision_check): tek eklem suprumeleri + aile/oturus pozlari (P5b)

Profilden bagimsiz: rl'den yalniz profil bagimsiz adlar (C.SIM_DT, eklem/geom/site adlari, JOINT_RANGE_MARGIN_DEG)
ve rl.actuators (apply_to_spec, enforce_range_margin, CLASSES, JOINT_CLASS) kullanilir; rl.robot_model kullanilmaz.
Modeller dogrudan MjSpec.from_file(scene)'den kurulur: carpisma modeli (grup 2 geomlari ve mesh'ler silinir,
timestep SIM_DT, ayak kutulari contype 2 / conaffinity 3, EXTRA kutulari yalniz zemin, oz-carpisma <pair>'leri,
tr35 aktuatorleri + range = ctrlrange +- marj) ve kutu oturtma icin gorsel model. Yalniz numpy + mujoco.
build/ ve configs/'a yazmaz. Cikti belirlenimci (zaman damgasi yok, sirali anahtarlar, indent 1).

Adimlar (sinirlar PARAMS'ta; JSON "params"). Acilar SOL bacak mutlak (config isaret kurali: sag = -sol), derece.
  9  EXTRA kutulari: thigh/shin/hip/hip_cradle/neck/jaw govdelerinin gorsel mesh koselerinin govde cercevesindeki AABB'si.
  10 oz-carpisma ciftleri (spec 3.2.2 v3.2); eksik geomlu ve keyframe HOME'da temas eden ciftler atilir (raporlanir).
  1  HOME: keyframe "stand". auto: COM - ayak merkezi x, [-17, +3] mm butcesinin disindaysa aile HOME'u (pitch 0,
     keyframe hip_yaw/roll ve bas; keyframe yuksekliginde duz ayakli en buyuk COM payli (hp, kn)).
  2  H_HOME: MuJoCo C oturtma (HOME qpos/ctrl, nominal tr35 aktuatorleri, DR yok, 2 s) govde z'si (0.1 mm).
  3  comelme ailesi (family3 portu): duz ayak an = FOOT0 - th - hp + kn; satir h_k = round_mm(z_kin) + 0.005*S*(1-k);
     her satirda en buyuk pay, 2 mm icindekilerden en kucuk |th|; COM payi < 0.75*L_f olan ilk satirda durur.
  4  yukseklikler: xl330 oranlari (H_DEEP en az son satir, H_MIN_WALK en az en alcak pitch 0 satiri).
  5  oturus: govde kutusu yerde, duz ayak, pitch -50..0; 1. asama bas HOME'da, 2. asama ilk 20 icin bas izgarasi.
  6  ayaklar, 7 sinirlar (ham MJCF ctrlrange), 8 kutle, 11 dogrulama (basarisizsa cikis 1), 12 cikti.
"""
import argparse
import hashlib
import json
import math
import os
import re
import sys

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
# profilden bagimsiz: tr35 config'i bu aracin urettigi JSON'u yukler; rl xl330 profiliyle iceri alinir (ortam geri konur)
_PREV_ROBOT = os.environ.get("OPENDUCK_ROBOT")
os.environ["OPENDUCK_ROBOT"] = "xl330_27"
try:
    from rl import actuators as A   # noqa: E402
    from rl import config as C      # noqa: E402
finally:
    if _PREV_ROBOT is None:
        os.environ.pop("OPENDUCK_ROBOT", None)
    else:
        os.environ["OPENDUCK_ROBOT"] = _PREV_ROBOT

TOOL = "tools/derive_posture.py"
TOOL_VERSION = 1
SCHEMA = 1
PROFILE = "tr35_b2"
DEFAULT_OUT = os.path.join(ROOT, "rl", "derived", "tr35_b2.json")
DEFAULT_CONFIG = os.path.join(ROOT, "configs", "tr35_b2.json")
S = 1.282                                    # Froude olcegi (tasarim sabiti)
LEG5 = ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")
FAM_J = ("hip_pitch", "knee", "ankle")       # = C.CROUCH_JOINTS
HEAD_GEOM, JAW_GEOM, BUMPER, NECK_GEOM = "head_collision", "jaw_collision", "neck_bumper", "neck_collision"
EXTRA_BODIES = ("left_thigh", "right_thigh", "left_shin", "right_shin", "left_hip", "right_hip",
                "left_hip_cradle", "right_hip_cradle", "neck", "jaw")
HEIGHT_KEYS = ("H_HOME", "H_MIN_WALK", "H_MIN_STAND", "H_DEEP", "Z_SIT", "SEAT_Z", "SEAT_Z_TOL",
               "TERMINATION_MIN_HEIGHT", "standup_z", "getup_success_z", "getup_pose_z", "getup_arrival_z")
CAP_KEYS = ("KNEE_LIMIT_DEG", "NECK_PITCH_MAX_DEG", "HIP_YAW_INWARD_MAX_DEG")
TOP_KEYS = ("schema", "profile", "tool", "tool_version", "source", "params", "mass", "home", "heights",
            "crouch_family", "seat", "head", "feet", "caps_deg", "extra_collision", "self_collision_pairs", "report")
_q9 = lambda x: round(float(x), 9)

PARAMS = {
    "S": S,
    "home_com_x_budget": [-0.017, 0.003],        # m; p2-interface 8: HOME COM x = ayak merkezi -7 +- 10 mm
    "home_step_deg": 0.5,                        # aile HOME izgarasi (hp, kn)
    "home_h_tol": 0.0005,                        # m; aile HOME'u keyframe kinematik yuksekliginin bu kadar icinde
    "settle_s": 2.0, "h_home_round": 0.0001,
    "pitch_grid_deg": [-30.0, 10.0, 2.5],        # comelme ailesi govde pitch'i (+ = burun asagi)
    "joint_step_deg": 1.5,                       # hp, kn izgarasi (alt sinirdan baslar; family3 np.arange)
    "joint_margin_deg": 5.0,                     # hp/kn/an: CAD ctrlrange'in bu kadar ici; diz ust = KNEE cap - bu
    "clearance": _q9(0.005 * S),                 # m; govde kutusu (ve kafa/gaga kutulari) ayak tabani duzleminin ustunde
    "row_step": _q9(0.005 * S), "row_tol": _q9(0.001 * S),
    "row_margin_lf": 0.75,                       # satir COM payi >= bu * L_f (L_f = ayak kutusu x yari boyu)
    "near_margin": 0.002,                        # m; family3: en iyi payin bu kadar icindekilerden en kucuk |th|
    "row_tie": "height",                         # esit |th|: en kucuk |h - h_k| (family3: izgara sirasinda ilk)
    "flat_tilt_max_deg": 0.5,                    # duz ayak: ayak kutusu z ekseni egimi
    "bottom_eps": 0.001,                         # m; destek cokgeni: en alcak noktanin bu kadar icindeki ayak koseleri
    "head_static_max": 0.3,                      # statik bas tork orani |qfrc_bias| / nominal tau_s (spec 3.2.5)
    "ratios": {"H_DEEP": 0.5652, "H_MIN_STAND": 0.6087, "H_MIN_WALK": 0.7391, "TERMINATION_MIN_HEIGHT": 0.6522,
               "standup_z": 0.9130, "getup_success_z": 0.8696, "getup_pose_z": [0.5217, 0.9130],
               "getup_arrival_z": [0.7826, 0.9565]},
    "seat_pitch_grid_deg": [-50.0, 0.0, 2.5],
    "seat_contact_tol": _q9(0.001 * S),          # m; govde kutusu en alt noktasi ayak tabani duzleminin +- bu icinde
    "seat_top_n": 20, "seat_head_step_deg": 2.5, "seat_head_margin_deg": 3.0, "seat_near_margin": 0.002,
    "seat_fallback_window": 0.02,                # m; govde yere degemiyorsa en alcak govdeli poz (feasible = false)
    "z_sit_offset": _q9(0.002 * S), "seat_z_tol": _q9(0.003 * S), "seat_gz_base_deg": 25.84,
    "feet_min_frac": 0.72,
    "val_h_tol": _q9(0.0015 * S),
    "val_offset_dh": [0.0, _q9(0.01 * S)],       # ofset formu: kinematik h - h bu aralikta
    "val_offset_margin_lf": 0.6,
    "val_offset_points": ["H_HOME", 0.8696, "H_MIN_WALK", "H_MIN_STAND", "H_DEEP"],   # 0.8696*H_HOME = xl330 0.1
    "extra_bodies": list(EXTRA_BODIES), "extra_round": 1e-05,
    "angle_round_deg": 0.001,                    # CAD sinirlari
}


# --- kucuk yardimcilar ----------------------------------------------------------------------------
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def mjcf_path(scene):
    return os.path.join(os.path.dirname(os.path.abspath(scene)), "openduck.xml")


def meshes_sha256(scene):
    """MJCF'nin referans verdigi mesh dosyalarinin (ad + icerik, sirali) ozeti; yoksa None."""
    p = mjcf_path(scene)
    if not os.path.exists(p):
        return None
    text = open(p).read()
    md = re.search(r'meshdir="([^"]*)"', text)
    base = os.path.join(os.path.dirname(p), md.group(1) if md else "")
    h = hashlib.sha256()
    for f in sorted(set(re.findall(r'<mesh [^>]*file="([^"]+)"', text))):
        h.update(f.encode() + b"\0")
        fp = os.path.join(base, f)
        h.update(sha256_file(fp).encode() if os.path.exists(fp) else b"missing")
    return h.hexdigest()


def grid(lo, hi, step):
    """lo, lo + step, ... <= hi (family3: np.arange(lo, hi + 0.01, step); alt sinir izgaranin icinde)."""
    n = int(math.floor((hi - lo) / step + 1e-9))
    return [round(lo + k * step, 9) + 0.0 for k in range(n + 1)]


def hull(pts):
    """Konveks zarf (CCW; phys-sit common.hull)."""
    pts = sorted(set(map(tuple, np.round(pts, 6))))
    if len(pts) < 3:
        return np.array(pts)
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return np.array(lo[:-1] + up[:-1])


def signed_margin(poly, p):
    """>0 icinde: kenarlara en kisa mesafe; <0 disinda: cokgene mesafe (phys-sit common.signed_margin)."""
    n = len(poly)
    if n < 3:
        return -1.0
    best, inside = 1e9, True
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        e = b - a
        dist = np.dot(p - a, np.array([e[1], -e[0]]) / (np.linalg.norm(e) + 1e-12))
        inside &= dist <= 0
        best = min(best, abs(dist))
    if inside:
        return float(best)
    dmin = 1e9
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        t = np.clip(np.dot(p - a, b - a) / (np.dot(b - a, b - a) + 1e-12), 0, 1)
        dmin = min(dmin, np.linalg.norm(p - (a + t * (b - a))))
    return -float(dmin)


def pitch_deg(quat):
    w, x, y, z = quat
    return math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))


def _py(x):
    """JSON icin yerel tipler (numpy -> float/int/list)."""
    if isinstance(x, dict):
        return {str(k): _py(v) for k, v in x.items()}
    if isinstance(x, (list, tuple, np.ndarray)):
        return [_py(v) for v in x]
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, (int, np.integer)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        return float(x)
    return x


def _rd(x, nd=6):
    """Rapor icin yuvarlama (ozyinelemeli)."""
    if isinstance(x, dict):
        return {k: _rd(v, nd) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_rd(v, nd) for v in x]
    if isinstance(x, float):
        return round(x, nd)
    return x


def dumps(doc):
    return json.dumps(_py(doc), sort_keys=True, indent=1) + "\n"


_SGN = np.array([(a, b, c) for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], dtype=np.float64)


# --- modeller -----------------------------------------------------------------------------------
def visual_model(scene):
    return mujoco.MjSpec.from_file(scene).compile()


class Model(object):
    """Turetme modeli: kutu carpisma geomlari (+ EXTRA, yalniz zemin), oz-carpisma <pair>'leri, istege bagli zemin ve
    tr35 aktuatorleri. Kinematik pozlar: govde orijini (0,0,0), pitch th; eklemler taban qpos'tan (HOME)."""

    def __init__(self, scene, extra=None, pairs=(), floor=True, actuators=True):
        spec = mujoco.MjSpec.from_file(scene)
        for g in list(spec.geoms):
            if g.group == 2:
                spec.delete(g)
        for me in list(spec.meshes):
            spec.delete(me)
        left = {g.name for g in spec.geoms}
        for pr in list(spec.pairs):                  # silinen (gorsel) geomlara ait MJCF ciftleri
            if pr.geomname1 not in left or pr.geomname2 not in left:
                spec.delete(pr)
        spec.option.timestep = C.SIM_DT
        for g in spec.geoms:
            if g.name in C.FOOT_GEOMS:
                g.contype, g.conaffinity = 2, 3
            elif g.name == C.FLOOR_GEOM and not floor:
                g.contype, g.conaffinity = 0, 0
        if extra:
            dc = spec.find_default("collision")
            for body, (kind, pos, size, _ax) in extra.items():
                if kind != "box":
                    raise ValueError(kind)
                spec.body(body).add_geom(dc, name=body + "_collision", type=mujoco.mjtGeom.mjGEOM_BOX,
                                         size=list(size), pos=list(pos), quat=[1.0, 0.0, 0.0, 0.0])
        for a, b in pairs:
            spec.add_pair(geomname1=a, geomname2=b)
        if actuators:
            A.apply_to_spec(spec, PROFILE)
            A.enforce_range_margin(spec)
        self.m = m = spec.compile()
        self.d = mujoco.MjData(m)
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
        self.gid = gid
        names = [s + "_" + j for s in ("left", "right") for j in LEG5] + list(C.HEAD_JOINTS) + list(C.OTHER_JOINTS)
        self.qa = {n: int(m.jnt_qposadr[jid(n)]) for n in names}
        self.da = {n: int(m.jnt_dofadr[jid(n)]) for n in names}
        self.foot = [gid(n) for n in C.FOOT_GEOMS]
        self.trunk = gid(C.TRUNK_GEOM)
        self.headg = [g for g in (gid(HEAD_GEOM), gid(JAW_GEOM)) if g >= 0]
        self.floor = gid(C.FLOOR_GEOM)
        self.trunk_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, C.TRUNK_BODY)
        self.sites = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, n) for n in C.FOOT_SITES]
        key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, C.DEFAULT_KEYFRAME)
        self.key_qpos, self.key_ctrl = m.key_qpos[key].copy(), m.key_ctrl[key].copy()
        self.base = self.key_qpos.copy()
        self.act_qadr = np.array([m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in range(m.nu)])
        self.head_dof = np.array([self.da[j] for j in C.HEAD_JOINTS])
        self.head_tau_s = np.array([A.CLASSES[A.JOINT_CLASS[PROFILE][j]]["tau_s"] for j in C.HEAD_JOINTS])
        self._bias = np.zeros(m.nv)

    def deg(self, qpos, side="left"):
        """qpos -> {eklem: derece} (SOL bacak ve bas)."""
        out = {j: math.degrees(qpos[self.qa[side + "_" + j]]) for j in LEG5}
        out.update({j: math.degrees(qpos[self.qa[j]]) for j in C.HEAD_JOINTS})
        return out

    def pose(self, th, hp, kn, an, head=None, base=None):
        """Kinematik poz: govde orijini 0, pitch th; sol (hp, kn, an), sag aynali; bas (4 derece) ya da tabandan."""
        m, d = self.m, self.d
        d.qpos[:] = self.base if base is None else base
        d.qpos[0:3] = 0.0
        t = math.radians(th)
        d.qpos[3:7] = [math.cos(t / 2), 0.0, math.sin(t / 2), 0.0]
        for j, v in zip(FAM_J, (hp, kn, an)):
            r = math.radians(v)
            d.qpos[self.qa["left_" + j]] = r
            d.qpos[self.qa["right_" + j]] = -r
        if head is not None:
            for j, v in zip(C.HEAD_JOINTS, head):
                d.qpos[self.qa[j]] = math.radians(v)
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)

    def corners(self, g):
        d = self.d
        return d.geom_xpos[g] + (_SGN * self.m.geom_size[g]) @ d.geom_xmat[g].reshape(3, 3).T

    def geo(self, eps=PARAMS["bottom_eps"]):
        """Mevcut poz: h (orijin - en alcak ayak noktasi), govde/kafa kutusu acikligi (ayak duzlemine gore), ayak
        egimi (derece), COM xy, destek koseleri (xy), govde kutusu en alt z'si (orijine gore, mutlak)."""
        d = self.d
        fc = np.vstack([self.corners(g) for g in self.foot])
        z0 = fc[:, 2].min()
        tz = self.corners(self.trunk)[:, 2].min()
        hz = min(self.corners(g)[:, 2].min() for g in self.headg) if self.headg else np.inf
        tilt = max(math.degrees(math.acos(max(-1.0, min(1.0, d.geom_xmat[g][8])))) for g in self.foot)
        return {"h": -z0, "clear": tz - z0, "head_clear": hz - z0, "trunk_low": tz, "tilt": tilt,
                "com": d.subtree_com[self.trunk_body][:2].copy(), "bottom": fc[fc[:, 2] < z0 + eps][:, :2]}

    def contacts(self):
        """Zemin disi temaslar (dist < 0): [(geom_a, geom_b, dist)] (adlar sirali)."""
        m, d = self.m, self.d
        mujoco.mj_collision(m, d)
        out = []
        for i in range(d.ncon):
            c = d.contact[i]
            if c.dist < 0 and self.floor not in (c.geom1, c.geom2):
                a, b = sorted((m.geom(c.geom1).name, m.geom(c.geom2).name))
                out.append((a, b, float(c.dist)))
        return out

    def head_static(self):
        """Mevcut kinematik pozda (qvel = 0) bas eklemlerinin statik torku / nominal tau_s (HEAD_JOINTS sirasi)."""
        m, d = self.m, self.d
        d.qvel[:] = 0.0
        mujoco.mj_comVel(m, d)
        mujoco.mj_rne(m, d, 0, self._bias)
        return np.abs(self._bias[self.head_dof]) / self.head_tau_s


def margin_of(g):
    if "margin" not in g:
        g["margin"] = signed_margin(hull(g["bottom"]), g["com"])
    return g["margin"]


def com_offset_x(g):
    b = g["bottom"][:, 0]
    return float(g["com"][0] - 0.5 * (b.min() + b.max()))


def scan(R, ths, hps, kns, an_lim, foot0, head=None, keep=None):
    """Duz ayakli simetrik poz izgarasi (th dis, hp, kn ic dongu: family3 sirasi). keep(g) -> bool on eleme."""
    out = []
    for th in ths:
        for hp in hps:
            for kn in kns:
                an = foot0 - th - hp + kn
                if an < an_lim[0] - 1e-9 or an > an_lim[1] + 1e-9:
                    continue
                R.pose(th, hp, kn, an, head)
                g = R.geo()
                g.update(th=th, hp=hp, kn=kn, an=an)
                if keep is None or keep(g):
                    out.append(g)
    return out


def collides(R, g, head=None):
    R.pose(g["th"], g["hp"], g["kn"], g["an"], head)
    return R.contacts()


def pick_family3(cands, near, hk=None, tie="first"):
    """En buyuk pay; ondan `near` icindekilerden en kucuk |th| (family3 kurali). Esitlik: tie "first" izgara
    sirasinda ilk (family3), "height" en kucuk |h - hk| (sonra izgara sirasi)."""
    best = max(cands, key=lambda g: g["margin"])
    near_set = [g for g in cands if g["margin"] >= best["margin"] - near]
    mth = min(abs(g["th"]) for g in near_set)
    near_set = [g for g in near_set if abs(g["th"]) == mth]
    return near_set[0] if tie == "first" else min(near_set, key=lambda g: abs(g["h"] - hk))


def family(R, recs, heights, tol, clear, head_clear=None, min_margin=None, near=0.002, tilt_max=None,
           collide=True, stop=True, tie="height"):
    """Comelme ailesi satirlari. heights: h_k listesi ya da uretec. min_margin: aday COM payi alt siniri; stop: aday
    kalmayan ilk satirda dur (yoksa atla). Dondurur [(h_k, poz kaydi)]."""
    H = np.array([g["h"] for g in recs])
    ok = np.array([g["clear"] >= clear and (head_clear is None or g["head_clear"] >= head_clear) and
                   (tilt_max is None or g["tilt"] < tilt_max) for g in recs], dtype=bool)
    rows = []
    for hk in heights:
        sel = [recs[i] for i in np.nonzero(ok & (np.abs(H - hk) < tol))[0]]
        sel = [g for g in sel if min_margin is None or margin_of(g) >= min_margin]
        if collide:
            sel = [g for g in sel if not collides(R, g)]
        if not sel:
            if stop:
                break
            continue
        for g in sel:
            margin_of(g)
        rows.append((hk, pick_family3(sel, near, hk, tie)))
    return rows


# --- adim 9-10: EXTRA kutulari ve oz-carpisma ciftleri -----------------------------------------------------
def extra_boxes(vis, bodies=EXTRA_BODIES, rnd=PARAMS["extra_round"]):
    """{govde: ("box", merkez, yari boyut, None)}: govdenin gorsel mesh geomlarinin govde cercevesindeki AABB'si."""
    m = vis
    out = {}
    for b in bodies:
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
        if bid < 0 or mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, b + "_collision") >= 0:
            continue                         # govde yok ya da MJCF'nin kendi <govde>_collision geomu var
        pts = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != bid or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mid = m.geom_dataid[g]
            V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].astype(np.float64)
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, m.geom_quat[g])
            pts.append(V @ R.reshape(3, 3).T + m.geom_pos[g])
        if not pts:
            continue
        P = np.vstack(pts)
        lo, hi = P.min(0), P.max(0)
        q = lambda v: [round(round(float(x) / rnd) * rnd, 9) + 0.0 for x in v]
        out[b] = ("box", q(0.5 * (lo + hi)), q(0.5 * (hi - lo)), None)
    return out


def candidate_pairs(geoms, has_bumper=False):
    """Spec 3.2.2 v3.2 oz-carpisma kurali (belirlenimci sira); geoms: mevcut RL geom adlari kumesi. Dondurur
    (ciftler, eksik geomlu ciftler). v3.2 (p2-entegrasyon.md 2, 10): cene <-> boyun ve kafa <-> kalca / kalca besigi
    (spec eki K6.4 RL cift listesi)."""
    T, H, J = C.TRUNK_GEOM, HEAD_GEOM, JAW_GEOM
    g = lambda side, part: "%s_%s_collision" % (side, part)
    P = []
    for side in ("left", "right"):
        for limb in ("thigh", "shin"):
            P += [(g(side, limb), t) for t in (T, H, J)]
    for side in ("left", "right"):
        P += [(g(side, "foot"), t) for t in (H, J)]
    P += [(H, T), (J, T), (J, NECK_GEOM)]
    for side in ("left", "right"):
        P += [(H, g(side, b)) for b in ("hip", "hip_cradle")]
    for a in ("thigh", "shin", "foot"):
        for b in ("thigh", "shin", "foot"):
            if not a == b == "foot":
                P.append((g("left", a), g("right", b)))
    for side in ("left", "right"):
        P += [(g(side, "foot"), g(side, b)) for b in ("thigh", "hip_cradle")]
    if has_bumper:
        P += [(BUMPER if a == H else a, BUMPER if b == H else b) for a, b in list(P) if H in (a, b)]
    out, missing, seen = [], [], set()
    for p in P:
        if frozenset(p) in seen:
            continue
        seen.add(frozenset(p))
        (out if p[0] in geoms and p[1] in geoms else missing).append(list(p))
    return out, missing


# --- adim 1-2: HOME ve H_HOME -----------------------------------------------------------------------------
def home_stats(R, qpos):
    R.base = qpos.copy()
    lg = R.deg(qpos)
    R.pose(0.0, lg["hip_pitch"], lg["knee"], lg["ankle"])
    g = R.geo()
    sl = [R.d.site_xpos[s].copy() for s in R.sites]
    return {"trunk_z_kin": g["h"], "com_offset_x": com_offset_x(g), "com_margin": margin_of(g),
            "foot_x": [float(g["bottom"][:, 0].min()), float(g["bottom"][:, 0].max())], "clear": g["clear"],
            "tilt": g["tilt"], "home_lateral": float(abs(sl[0][1] - sl[1][1])), "contacts": R.contacts()}


def family_home(R, foot0, z_target, hp_rng, kn_rng, an_lim, P):
    """Pitch 0, keyframe hip_yaw/roll ve bas: duz ayakli, kinematik yuksekligi z_target'in home_h_tol icinde, temassiz
    ve acikligi yeterli (hp, kn) iclerinden en buyuk COM payli olan (esitlikte izgara sirasinda ilk)."""
    st = P["home_step_deg"]
    keep = lambda g: (abs(g["h"] - z_target) < P["home_h_tol"] and g["clear"] >= P["clearance"] and
                      g["head_clear"] >= P["clearance"] and g["tilt"] < P["flat_tilt_max_deg"])
    recs = scan(R, [0.0], grid(*hp_rng, st), grid(*kn_rng, st), an_lim, foot0, keep=keep)
    recs = [g for g in recs if not collides(R, g)]
    if not recs:
        return None, 0
    for g in recs:
        margin_of(g)
    return max(recs, key=lambda g: g["margin"]), len(recs)


def settle(Rs, qpos, ctrl, seconds):
    """MuJoCo C: qpos/ctrl'dan seconds boyunca (nominal aktuatorler). -> (govde z, pitch derece, max |qvel|)."""
    m, d = Rs.m, Rs.d
    mujoco.mj_resetData(m, d)
    d.qpos[:] = qpos
    d.ctrl[:] = ctrl
    for _ in range(int(round(seconds / m.opt.timestep))):
        mujoco.mj_step(m, d)
    return float(d.qpos[2]), pitch_deg(d.qpos[3:7]), float(np.abs(d.qvel).max())


# --- offset formu (task_math.posture_ref, alpha = 0) ---------------------------------------------------
def offset_pose(rows, home_legs, h_home, h):
    """F_home(h) = HOME + F(h) - F(H_HOME) (lineer ara deger, tablo disi kirpilir) -> (pitch, hp, kn, an) derece."""
    F = np.array(rows, dtype=np.float64)[::-1]
    f = lambda x, k: float(np.interp(x, F[:, 0], F[:, k]))
    q = [home_legs[j] + f(h, k) - f(h_home, k) for j, k in zip(FAM_J, (2, 3, 4))]
    return (f(h, 1),) + tuple(q)


# --- ana turetme -----------------------------------------------------------------------------------------
def _ctrl_deg(vis, name, rnd=PARAMS["angle_round_deg"]):
    """Ham MJCF aktuator ctrlrange'i (derece, rnd'ye yuvarlanmis)."""
    a = mujoco.mj_name2id(vis, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    return [round(round(math.degrees(x) / rnd) * rnd, 6) + 0.0 for x in vis.actuator_ctrlrange[a]]


def derive(scene, stub=False, home="auto", config=None, log=None):
    """Sema 1 sozlugu ve dogrulama hatalari listesi dondurur: (doc, errors)."""
    P = PARAMS
    say = log or (lambda *a: None)
    if home not in ("auto", "keyframe", "family"):
        raise ValueError("--home %r" % home)
    cfg = DEFAULT_CONFIG if config is None else config
    vis = visual_model(scene)
    cr = lambda n: _ctrl_deg(vis, n)
    caps = {"KNEE_LIMIT_DEG": cr("left_knee")[1], "NECK_PITCH_MAX_DEG": cr("neck_pitch")[1],
            "HIP_YAW_INWARD_MAX_DEG": -cr("left_hip_yaw")[0] + 0.0}
    mg = P["joint_margin_deg"]
    hp_rng = (cr("left_hip_pitch")[0] + mg, cr("left_hip_pitch")[1] - mg)
    kn_rng = (cr("left_knee")[0] + mg, caps["KNEE_LIMIT_DEG"] - mg)
    an_lim = (cr("left_ankle")[0] + mg, cr("left_ankle")[1] - mg)
    mass = float(sum(vis.body_mass))

    # 9-10: EXTRA kutulari, ciftler; keyframe HOME'da temas edenler atilir
    extra = extra_boxes(vis)
    R0 = Model(scene, extra=extra, floor=False)
    geoms = {R0.m.geom(i).name for i in range(R0.m.ngeom)}
    pairs, missing = candidate_pairs(geoms, BUMPER in geoms)
    R0 = Model(scene, extra=extra, pairs=[tuple(p) for p in pairs], floor=False)
    kst = home_stats(R0, R0.key_qpos)
    touch = {(a, b) for a, b, _ in kst["contacts"]}
    dropped = [p for p in pairs if tuple(sorted(p)) in touch]
    pairs = [p for p in pairs if tuple(sorted(p)) not in touch]
    R = Model(scene, extra=extra, pairs=[tuple(p) for p in pairs], floor=False)
    Rs = Model(scene, extra=extra, pairs=[tuple(p) for p in pairs], floor=True)
    L_f = float(R.m.geom_size[R.foot[0]][0])
    say("EXTRA %d kutu, %d cift (%d HOME temasi, %d eksik geom)" % (len(extra), len(pairs), len(dropped), len(missing)))

    # 1: HOME
    kq = R.key_qpos.copy()
    kd = R.deg(kq)
    foot0 = kd["hip_pitch"] - kd["knee"] + kd["ankle"]
    kst = home_stats(R, kq)
    lo, hi = P["home_com_x_budget"]
    in_budget = lo <= kst["com_offset_x"] <= hi
    src = {"auto": "keyframe" if in_budget else "family", "keyframe": "keyframe", "family": "family"}[home]
    say("keyframe HOME: z_kin %.4f, COM - ayak merkezi %+.4f m (butce %s) -> %s" % (
        kst["trunk_z_kin"], kst["com_offset_x"], "icinde" if in_budget else "DISINDA", src))
    hq, n_home = kq.copy(), None
    if src == "family":
        g, n_home = family_home(R, foot0, kst["trunk_z_kin"], hp_rng, kn_rng, an_lim, P)
        if g is None:
            raise RuntimeError("aile HOME'u bulunamadi (keyframe yuksekliginde duz ayakli temassiz poz yok)")
        for j, v in zip(FAM_J, (g["hp"], g["kn"], g["an"])):
            hq[R.qa["left_" + j]], hq[R.qa["right_" + j]] = math.radians(v), -math.radians(v)
        hq[2] = g["h"] + (kq[2] - kst["trunk_z_kin"])
    hst = home_stats(R, hq)
    R.base = hq.copy()
    Rs.base = hq.copy()
    hctrl = hq[R.act_qadr].copy()
    hd = R.deg(hq)
    home_legs = {j: hd[j] for j in LEG5}
    home_head = {j: hd[j] for j in C.HEAD_JOINTS}

    # 2: H_HOME
    z_set, p_set, v_set = settle(Rs, hq, hctrl, P["settle_s"])
    H_HOME = round(round(z_set / P["h_home_round"]) * P["h_home_round"], 6)
    say("H_HOME %.4f (oturtma z %.5f, pitch %+.2f derece, max |qvel| %.2e)" % (H_HOME, z_set, p_set, v_set))

    # 3: comelme ailesi (+ oturus taramasi ayni izgarada)
    js = P["joint_step_deg"]
    hps, kns = grid(*hp_rng, js), grid(*kn_rng, js)
    pg, sg = P["pitch_grid_deg"], P["seat_pitch_grid_deg"]
    ths = grid(min(pg[0], sg[0]), max(pg[1], sg[1]), pg[2])
    stol = P["seat_contact_tol"]
    in_fam = lambda g: (pg[0] - 1e-9 <= g["th"] <= pg[1] + 1e-9 and g["tilt"] < P["flat_tilt_max_deg"] and
                        g["clear"] >= P["clearance"])
    in_seat = lambda g: sg[0] - 1e-9 <= g["th"] <= sg[1] + 1e-9 and g["clear"] <= P["seat_fallback_window"]
    recs = scan(R, ths, hps, kns, an_lim, foot0, keep=lambda g: in_fam(g) or in_seat(g))
    fam_recs = [g for g in recs if in_fam(g)]
    seat_recs = [g for g in recs if in_seat(g)]
    h0 = round(hst["trunk_z_kin"], 3)

    def heights():
        k = 0
        while True:
            hk = round(h0 + P["row_step"] * (1 - k), 6)
            if hk <= 0:
                return
            yield hk
            k += 1
    fam = family(R, fam_recs, heights(), P["row_tol"], P["clearance"], head_clear=P["clearance"],
                 min_margin=P["row_margin_lf"] * L_f, near=P["near_margin"], tilt_max=P["flat_tilt_max_deg"],
                 tie=P["row_tie"])
    rows, row_rep = [], []
    for hk, g in fam:
        rows.append([hk, g["th"], g["hp"], g["kn"], g["an"]])
        R.pose(g["th"], g["hp"], g["kn"], g["an"])
        st = R.head_static()
        row_rep.append({"h": hk, "h_kin": g["h"], "margin": g["margin"], "clear": g["clear"],
                        "head_clear": g["head_clear"], "tilt_deg": g["tilt"], "com_offset_x": com_offset_x(g),
                        "head_static_ratio": dict(zip(C.HEAD_JOINTS, st.tolist())),
                        "head_static_flag": bool(st.max() > P["head_static_max"])})
    say("aile: %d satir, h %.4f .. %.4f" % (len(rows), rows[0][0] if rows else 0, rows[-1][0] if rows else 0))
    if len(rows) < 3:
        raise RuntimeError("comelme ailesi %d satir (en az 3 gerekli)" % len(rows))

    # 4: yukseklikler
    r = P["ratios"]
    q6 = lambda x: round(float(x), 6)
    pitch0 = [row[0] for row in rows[:next((i for i, row in enumerate(rows) if abs(row[1]) > 1e-9), len(rows))]]
    H = {"H_HOME": H_HOME}
    H["H_DEEP"] = q6(max(r["H_DEEP"] * H_HOME, rows[-1][0]))
    H["H_MIN_STAND"] = q6(max(r["H_MIN_STAND"] * H_HOME, H["H_DEEP"]))
    H["H_MIN_WALK"] = q6(max(r["H_MIN_WALK"] * H_HOME, min(pitch0) if pitch0 else H_HOME))
    for k in ("TERMINATION_MIN_HEIGHT", "standup_z", "getup_success_z"):
        H[k] = q6(r[k] * H_HOME)
    H["getup_pose_z"] = [q6(x * H_HOME) for x in r["getup_pose_z"]]
    H["getup_arrival_z"] = [q6(x * H_HOME) for x in r["getup_arrival_z"]]

    # 5: oturus
    seat, seat_rep = derive_seat(R, seat_recs, home_head, vis, P)
    H["SEAT_Z"] = seat["SEAT_Z"]
    H["Z_SIT"] = q6(seat["SEAT_Z"] + P["z_sit_offset"])
    H["SEAT_Z_TOL"] = P["seat_z_tol"]

    # 6: ayaklar
    feet = {"home_lateral": q6(hst["home_lateral"]), "FEET_MIN_DISTANCE": q6(P["feet_min_frac"] * hst["home_lateral"])}

    home_d = {"source": src, "qpos": hq.tolist(), "ctrl": hctrl.tolist(), "legs_deg": home_legs, "head_deg": home_head,
              "trunk_z_kin": q6(hst["trunk_z_kin"]), "com_offset_x": q6(hst["com_offset_x"]),
              "com_margin": q6(hst["com_margin"])}
    doc = {
        "schema": SCHEMA, "profile": PROFILE, "tool": TOOL, "tool_version": TOOL_VERSION,
        "source": {"scene": scene, "mjcf_sha256": sha256_file(mjcf_path(scene)),
                   "config_sha256": sha256_file(cfg) if cfg and os.path.exists(cfg) else None, "stub": bool(stub)},
        "params": P,
        "mass": {"total_kg": q6(mass)},
        "home": home_d,
        "heights": H,
        "crouch_family": rows,
        "seat": {"deg": seat["deg"], "pitch_deg": seat["pitch_deg"], "seat_gz": seat["seat_gz"],
                 "feasible": seat["feasible"]},
        "head": {"home_deg": home_head, "sit_deg": seat["head_deg"], "tuck_deg": dict(seat["head_deg"])},
        "feet": feet,
        "caps_deg": caps,
        "extra_collision": {b: list(v) for b, v in extra.items()},
        "self_collision_pairs": pairs,
    }
    rep = {
        "foot_half_length_L_f": L_f, "foot0_deg": foot0,
        "grid": {"hip_pitch": list(hp_rng), "knee": list(kn_rng), "ankle": list(an_lim), "poses": len(recs),
                 "family_poses": len(fam_recs), "seat_poses": len(seat_recs)},
        "keyframe": {k: kst[k] for k in ("trunk_z_kin", "com_offset_x", "com_margin", "foot_x")},
        "keyframe_com_in_budget": bool(in_budget),
        "home": {"source": src, "candidates": n_home, "foot_x": hst["foot_x"], "clear": hst["clear"],
                 "tilt_deg": hst["tilt"], "settle": {"z": z_set, "pitch_deg": p_set, "max_abs_qvel": v_set}},
        "rows": row_rep,
        "seat": seat_rep,
        "pairs": {"kept": len(pairs), "dropped_home_contact": dropped, "dropped_missing_geom": missing},
        "extra_not_fitted": [b for b in EXTRA_BODIES if b not in extra],
        "meshes_sha256": meshes_sha256(scene),
    }
    doc["report"] = rep
    errs, val = validate_physics(R, doc, L_f, foot0)
    rep["validation"] = val
    doc["report"] = _rd(_py(rep))
    errs = validate(doc) + errs
    doc["report"]["errors"] = errs
    doc["report"]["warnings"] = warnings(doc)
    return doc, errs


def derive_seat(R, seat_recs, home_head, vis, P):
    """Adim 5. 1. asama: bas HOME'da govde kutusu yerde, duz ayakli, temassiz adaylar (COM payina gore ilk N). 2. asama:
    bas izgarasi (neck_pitch, head_pitch; yaw = roll = 0), temassiz, kafa/gaga zeminden yukarida, statik oran <= sinir.
    Amac: en buyuk ayak-yalniz COM payi; payin seat_near_margin icinde en kucuk |pitch|, sonra en kucuk statik oran.
    Govde kutusu hicbir duz ayakli pozda yere degmiyorsa (mode "no_trunk_contact", feasible = false): en alcak govdeli
    temassiz poz (esitlikte buyuk pay); SEAT_Z o pozun kinematik yuksekligi (z_ref = q_ref tutarli)."""
    stol = P["seat_contact_tol"]
    ok = lambda g: g["head_clear"] - min(0.0, g["clear"]) >= P["clearance"] and g["clear"] >= -stol - 1e-12
    touch = [g for g in seat_recs if abs(g["clear"]) <= stol + 1e-12 and ok(g)]
    st1 = []
    for g in touch:
        if not collides(R, g):
            margin_of(g)
            st1.append(g)
    rep = {"stage1": len(st1), "mode": "trunk_floor"}
    if not st1:
        low = sorted((g for g in seat_recs if ok(g)), key=lambda g: g["clear"])
        for g in low:
            if not collides(R, g):
                cl = g["clear"]
                st1 = [x for x in low if x["clear"] <= cl + 1e-9 and not collides(R, x)]
                break
        if not st1:
            raise RuntimeError("oturus: duz ayakli temassiz poz yok")
        for g in st1:
            margin_of(g)
        rep.update(mode="no_trunk_contact", min_trunk_clear=st1[0]["clear"],
                   note="govde kutusu hicbir duz ayakli pozda yere degmiyor (en kucuk aciklik %.1f mm, pitch %.1f)" % (
                       1000 * st1[0]["clear"], st1[0]["th"]))
    top = sorted(st1, key=lambda g: -g["margin"])[:P["seat_top_n"]]
    rep["stage1_top"] = [{"pitch": g["th"], "hp": g["hp"], "kn": g["kn"], "an": g["an"], "margin": g["margin"]}
                         for g in top[:5]]
    hs, hm = P["seat_head_step_deg"], P["seat_head_margin_deg"]
    nk = _ctrl_deg(vis, "neck_pitch")
    hk = _ctrl_deg(vis, "head_pitch")
    necks, hpts = grid(nk[0] + hm, nk[1] - hm, hs), grid(hk[0] + hm, hk[1] - hm, hs)
    st2, n_eval = [], 0
    for ci, g in enumerate(top):
        for nv in necks:
            for hv in hpts:
                head = (nv, hv, 0.0, 0.0)
                R.pose(g["th"], g["hp"], g["kn"], g["an"], head)
                n_eval += 1
                x = R.geo()
                if x["head_clear"] - min(0.0, x["clear"]) < P["clearance"]:
                    continue
                ratio = R.head_static()
                if ratio.max() > P["head_static_max"]:
                    continue
                if R.contacts():
                    continue
                st2.append({"ci": ci, "g": g, "head": head, "margin": margin_of(x), "ratio": float(ratio.max()),
                            "ratios": ratio.tolist(), "geo": x})
    rep["stage2_evaluated"], rep["stage2_ok"] = n_eval, len(st2)
    if st2:
        best = max(st2, key=lambda c: c["margin"])
        near = [c for c in st2 if c["margin"] >= best["margin"] - P["seat_near_margin"]]
        mp = min(abs(c["g"]["th"]) for c in near)
        near = [c for c in near if abs(c["g"]["th"]) == mp]
        w = min(near, key=lambda c: c["ratio"])
        g, head, margin, x, ratios = w["g"], w["head"], w["margin"], w["geo"], w["ratios"]
    else:                                    # bas izgarasinda gecerli poz yok: 1. asama en iyisi, bas HOME'da
        g = top[0]
        head = tuple(home_head[j] for j in C.HEAD_JOINTS)
        R.pose(g["th"], g["hp"], g["kn"], g["an"], head)
        x = R.geo()
        margin, ratios = margin_of(x), R.head_static().tolist()
        rep["stage2_fallback"] = "bas izgarasinda gecerli poz yok; bas HOME'da"
    seat_z = round(-x["trunk_low"] if rep["mode"] == "trunk_floor" else x["h"], 6)
    th = g["th"]
    out = {"deg": {"hip_pitch": g["hp"], "knee": g["kn"], "ankle": g["an"]}, "pitch_deg": th,
           "seat_gz": round(-math.cos(math.radians(abs(th) + P["seat_gz_base_deg"])), 6),
           "feasible": bool(margin > 0 and rep["mode"] == "trunk_floor"), "SEAT_Z": seat_z,
           "head_deg": dict(zip(C.HEAD_JOINTS, head))}
    rep.update({"pitch_deg": th, "margin": margin, "com_offset_x": com_offset_x(x), "h_kin": x["h"],
                "trunk_contact": x["clear"], "head_clear": x["head_clear"], "tilt_deg": x["tilt"],
                "head_static_ratio": dict(zip(C.HEAD_JOINTS, ratios)), "head": list(head),
                "stage1_margin_head_home": g["margin"], "feasible": out["feasible"]})
    return out, rep


# --- dogrulama ------------------------------------------------------------------------------------------
def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def validate(doc):
    """Sema 1 ve iliskiler (MuJoCo'suz). Hata metinleri listesi (bos = gecerli)."""
    E = []
    try:
        _validate(doc, E)
    except (KeyError, TypeError, IndexError, ValueError, AttributeError) as e:
        E.append("yapi hatasi: %r" % (e,))
    return E


def _validate(d, E):
    err = E.append
    if not isinstance(d, dict):
        err("kok sozluk degil")
        return
    if set(d) != set(TOP_KEYS):
        err("ust anahtarlar: eksik %s fazla %s" % (sorted(set(TOP_KEYS) - set(d)), sorted(set(d) - set(TOP_KEYS))))
        return
    if d["schema"] != SCHEMA or d["profile"] != PROFILE or d["tool"] != TOOL or not isinstance(d["tool_version"], int):
        err("schema/profile/tool/tool_version")
    s = d["source"]
    if set(s) != {"scene", "mjcf_sha256", "config_sha256", "stub"} or not isinstance(s["scene"], str) or \
            not re.fullmatch(r"[0-9a-f]{64}", str(s["mjcf_sha256"])) or not isinstance(s["stub"], bool) or \
            not (s["config_sha256"] is None or re.fullmatch(r"[0-9a-f]{64}", str(s["config_sha256"]))):
        err("source")
    if not isinstance(d["params"], dict) or not isinstance(d["report"], dict):
        err("params/report sozluk degil")
    if not (_num(d["mass"]["total_kg"]) and d["mass"]["total_kg"] > 0) or set(d["mass"]) != {"total_kg"}:
        err("mass")
    hm = d["home"]
    hk = {"source", "qpos", "ctrl", "legs_deg", "head_deg", "trunk_z_kin", "com_offset_x", "com_margin"}
    if set(hm) != hk:
        err("home anahtarlari")
        return
    if hm["source"] not in ("keyframe", "family"):
        err("home.source")
    if len(hm["qpos"]) != 22 or len(hm["ctrl"]) != 15 or not all(_num(x) for x in hm["qpos"] + hm["ctrl"]):
        err("home.qpos (22) / ctrl (15)")
    if set(hm["legs_deg"]) != set(LEG5) or set(hm["head_deg"]) != set(C.HEAD_JOINTS):
        err("home.legs_deg/head_deg anahtarlari")
    for k in ("trunk_z_kin", "com_offset_x", "com_margin"):
        if not _num(hm[k]):
            err("home." + k)
    H = d["heights"]
    if set(H) != set(HEIGHT_KEYS):
        err("heights anahtarlari")
        return
    for k in HEIGHT_KEYS:
        v = H[k] if isinstance(H[k], list) else [H[k]]
        if (len(v) != (2 if k in ("getup_pose_z", "getup_arrival_z") else 1)) or not all(_num(x) and x > 0 for x in v):
            err("heights." + k)
            return
    if not H["SEAT_Z"] < H["Z_SIT"]:
        err("sira: SEAT_Z < Z_SIT degil")
    chain = [("H_DEEP", H["H_DEEP"]), ("H_MIN_STAND", H["H_MIN_STAND"]), ("H_MIN_WALK", H["H_MIN_WALK"]),
             ("H_HOME", H["H_HOME"])]
    for (a, x), (b, y) in zip(chain, chain[1:]):
        if not (x < y or (x == y and a in ("H_DEEP", "H_MIN_STAND"))):
            err("sira: %s %.4f < %s %.4f degil" % (a, x, b, y))
    gz = [("getup_pose_z[0]", H["getup_pose_z"][0]), ("getup_arrival_z[0]", H["getup_arrival_z"][0]),
          ("getup_success_z", H["getup_success_z"]), ("standup_z", H["standup_z"]),
          ("getup_pose_z[1]", H["getup_pose_z"][1]), ("getup_arrival_z[1]", H["getup_arrival_z"][1]),
          ("H_HOME", H["H_HOME"])]
    for (a, x), (b, y) in zip(gz, gz[1:]):
        if not (x < y or (x == y and (a, b) == ("standup_z", "getup_pose_z[1]"))):
            err("sira: %s %.4f < %s %.4f degil" % (a, x, b, y))
    if not H["TERMINATION_MIN_HEIGHT"] < H["H_MIN_WALK"]:
        err("sira: TERMINATION_MIN_HEIGHT < H_MIN_WALK degil")
    F = d["crouch_family"]
    if len(F) < 3 or not all(len(r) == 5 and all(_num(x) for x in r) for r in F):
        err("crouch_family: en az 3 satir x 5 sayi")
        return
    hs = [r[0] for r in F]
    if not all(a > b for a, b in zip(hs, hs[1:])):
        err("crouch_family: h kesin azalan degil")
    if not hs[0] >= H["H_HOME"] or not hs[-1] <= H["H_DEEP"]:
        err("crouch_family: ilk h >= H_HOME ve son h <= H_DEEP olmali")
    lg = hm["legs_deg"]
    f0 = lg["hip_pitch"] - lg["knee"] + lg["ankle"]
    for r in F:
        if abs(r[1] + r[2] - r[3] + r[4] - f0) > 1e-6:
            err("crouch_family duz ayak: h %.4f" % r[0])
    se = d["seat"]
    if set(se) != {"deg", "pitch_deg", "seat_gz", "feasible"} or set(se["deg"]) != set(FAM_J) or \
            not _num(se["pitch_deg"]) or not _num(se["seat_gz"]) or not isinstance(se["feasible"], bool) or \
            not all(_num(x) for x in se["deg"].values()):
        err("seat")
    elif abs(se["pitch_deg"] + se["deg"]["hip_pitch"] - se["deg"]["knee"] + se["deg"]["ankle"] - f0) > 1e-6:
        err("seat duz ayak")
    hd = d["head"]
    if set(hd) != {"home_deg", "sit_deg", "tuck_deg"} or any(set(hd[k]) != set(C.HEAD_JOINTS) or
                                                         not all(_num(x) for x in hd[k].values()) for k in hd):
        err("head")
    ft = d["feet"]
    if set(ft) != {"home_lateral", "FEET_MIN_DISTANCE"} or not 0 < ft["FEET_MIN_DISTANCE"] < ft["home_lateral"]:
        err("feet")
    if set(d["caps_deg"]) != set(CAP_KEYS) or not all(_num(x) and x > 0 for x in d["caps_deg"].values()):
        err("caps_deg")
    ex = d["extra_collision"]
    for b, v in ex.items():
        if not (isinstance(v, list) and len(v) == 4 and v[0] == "box" and v[3] is None and len(v[1]) == 3 and
                len(v[2]) == 3 and all(_num(x) for x in v[1]) and all(_num(x) and x > 0 for x in v[2])):
            err("extra_collision.%s" % b)
    names = {C.TRUNK_GEOM, HEAD_GEOM, BUMPER} | set(C.FOOT_GEOMS) | {b + "_collision" for b in ex}
    seen = set()
    for p in d["self_collision_pairs"]:
        if not (isinstance(p, list) and len(p) == 2 and p[0] != p[1] and set(p) <= names):
            err("self_collision_pairs: %r" % (p,))
        elif frozenset(p) in seen:
            err("self_collision_pairs: tekrar %r" % (p,))
        else:
            seen.add(frozenset(p))


def warnings(doc):
    """Hata olmayan ama geometri icin kirmizi bayrak olan durumlar (rapora yazilir)."""
    W, H, rp, P = [], doc["heights"], doc["report"], doc["params"]
    if not H["Z_SIT"] < H["H_DEEP"]:
        W.append("Z_SIT %.4f >= H_DEEP %.4f: oturus govde yuksekligi derin comelmeden alcak degil" % (
            H["Z_SIT"], H["H_DEEP"]))
    sr = rp["seat"]
    if sr["mode"] != "trunk_floor":
        W.append("oturus: " + sr["note"])
    if not doc["seat"]["feasible"]:
        W.append("oturus uygun degil (COM payi %.1f mm, mod %s)" % (1000 * sr["margin"], sr["mode"]))
    if sr["tilt_deg"] >= P["flat_tilt_max_deg"]:
        W.append("oturus ayak kutusu egimi %.2f derece (pitch %.1f; hip_roll -5 / taban +5 derece karisimi)" % (
            sr["tilt_deg"], sr["pitch_deg"]))
    if "stage2_fallback" in sr:
        W.append("oturus: " + sr["stage2_fallback"])
    for x in rp["rows"]:
        if x["head_static_flag"]:
            W.append("satir h %.4f: bas statik orani %.2f > %.2f" % (
                x["h"], max(x["head_static_ratio"].values()), P["head_static_max"]))
    if not rp["keyframe_com_in_budget"]:
        W.append("keyframe HOME COM - ayak merkezi %+.1f mm, butce [%+.0f, %+.0f] mm disinda (HOME %s)" % (
            1000 * rp["keyframe"]["com_offset_x"], 1000 * P["home_com_x_budget"][0], 1000 * P["home_com_x_budget"][1],
            doc["home"]["source"]))
    if rp["pairs"]["dropped_home_contact"]:
        W.append("HOME'da temas eden ciftler atildi: %s" % rp["pairs"]["dropped_home_contact"])
    return W


def validate_physics(R, doc, L_f, foot0):
    """Adim 11 (MuJoCo): satirlar, ofset formu, oturus ve HOME. R: HOME tabanli kinematik model (ciftler dahil).
    Dondurur (hatalar, rapor)."""
    P = doc["params"]
    E, rep = [], {}
    H = doc["heights"]
    hl = doc["home"]["legs_deg"]
    head0 = [doc["home"]["head_deg"][j] for j in C.HEAD_JOINTS]
    out = []
    for h, th, hp, kn, an in doc["crouch_family"]:
        R.pose(th, hp, kn, an, head0)
        g = R.geo()
        c = R.contacts()
        m = margin_of(g)
        ok = (abs(g["h"] - h) <= P["val_h_tol"] and m >= P["row_margin_lf"] * L_f - 1e-9 and
              g["clear"] >= P["clearance"] - 1e-9 and g["tilt"] < P["flat_tilt_max_deg"] and not c and
              abs(th + hp - kn + an - foot0) < 1e-6)
        out.append({"h": h, "dh": g["h"] - h, "margin": m, "ok": ok})
        if not ok:
            E.append("satir h %.4f: dh %.4f pay %.4f aciklik %.4f egim %.2f temas %s" % (
                h, g["h"] - h, m, g["clear"], g["tilt"], c))
    rep["rows"] = out
    pts = []
    for p in P["val_offset_points"]:
        h = H[p] if isinstance(p, str) else round(p * H["H_HOME"], 6)
        th, hp, kn, an = offset_pose(doc["crouch_family"], hl, H["H_HOME"], h)
        R.pose(th, hp, kn, an, head0)
        g = R.geo()
        c = R.contacts()
        m = margin_of(g)
        dh = g["h"] - h
        ok = (P["val_offset_dh"][0] - 1e-9 <= dh <= P["val_offset_dh"][1] and m >= P["val_offset_margin_lf"] * L_f and
              g["clear"] >= P["clearance"] and not c)
        pts.append({"point": p, "h": h, "dh": dh, "margin": m, "clear": g["clear"], "contacts": len(c), "ok": ok})
        if not ok:
            E.append("ofset formu h %.4f (%s): dh %.4f pay %.4f aciklik %.4f temas %s" % (h, p, dh, m, g["clear"], c))
    rep["offset_form"] = pts
    se = doc["seat"]
    sh = [doc["head"]["sit_deg"][j] for j in C.HEAD_JOINTS]
    R.pose(se["pitch_deg"], se["deg"]["hip_pitch"], se["deg"]["knee"], se["deg"]["ankle"], sh)
    g = R.geo()
    c = R.contacts()
    ratio = R.head_static()
    srep = doc.get("report", {}).get("seat", {})
    mode = srep.get("mode", "trunk_floor")
    if mode == "trunk_floor":
        z_ok = (abs(g["clear"]) <= P["seat_contact_tol"] + 1e-9 and abs(-g["trunk_low"] - H["SEAT_Z"]) < 1e-6 and
                abs(g["h"] - H["SEAT_Z"]) <= H["SEAT_Z_TOL"])
    else:
        z_ok = abs(g["h"] - H["SEAT_Z"]) < 1e-6 and not se["feasible"]
    seat_ok = (z_ok and not c and g["head_clear"] - min(0.0, g["clear"]) >= P["clearance"] - 1e-9 and
               abs(se["pitch_deg"] + se["deg"]["hip_pitch"] - se["deg"]["knee"] + se["deg"]["ankle"] - foot0) < 1e-6 and
               (ratio.max() <= P["head_static_max"] or "stage2_fallback" in srep))
    rep["seat"] = {"mode": mode, "trunk_contact": g["clear"], "h_kin": g["h"], "margin": margin_of(g),
                   "contacts": len(c), "tilt_deg": g["tilt"], "head_static_max": float(ratio.max()), "ok": seat_ok}
    if not seat_ok:
        E.append("oturus pozu: govde temasi %.4f h %.4f temas %s statik %.2f" % (g["clear"], g["h"], c, ratio.max()))
    hq = np.array(doc["home"]["qpos"])
    R.pose(0.0, hl["hip_pitch"], hl["knee"], hl["ankle"], head0, base=hq)
    g = R.geo()
    c = R.contacts()
    home_ok = not c and g["clear"] >= P["clearance"] and abs(g["h"] - doc["home"]["trunk_z_kin"]) < 1e-6
    rep["home"] = {"contacts": len(c), "ok": home_ok}
    if not home_ok:
        E.append("HOME: temas %s aciklik %.4f" % (c, g["clear"]))
    return E, _rd(rep)


# --- check ve caps-scan --------------------------------------------------------------------------------------
def check(scene, derived=DEFAULT_OUT):
    """(cikis kodu, mesajlar): 0 gecerli + taze, 3 gecerli ama MJCF/mesh sha'si farkli, 1 gecersiz."""
    try:
        doc = json.load(open(derived))
    except (OSError, ValueError) as e:
        return 1, ["okunamadi: %s (%s)" % (derived, e)]
    errs = validate(doc)
    if errs:
        return 1, errs
    msgs = []
    if not os.path.exists(scene) or not os.path.exists(mjcf_path(scene)):
        return 1, ["sahne ya da yanindaki openduck.xml yok: %s" % scene]
    sha = sha256_file(mjcf_path(scene))
    msha = meshes_sha256(scene)
    rsha = doc["report"].get("meshes_sha256")
    if sha != doc["source"]["mjcf_sha256"] or (rsha is not None and msha != rsha):
        return 3, ["gecerli ama eski: MJCF sha %s, dosyada %s (mesh %s)" % (
            sha, doc["source"]["mjcf_sha256"], "ayni" if msha == rsha else "farkli")]
    hq = np.array(doc["home"]["qpos"])
    pairs = [tuple(p) for p in doc["self_collision_pairs"]]
    extra = {b: tuple(v) for b, v in doc["extra_collision"].items()}
    R = Model(scene, extra=extra, pairs=pairs, floor=False)
    R.base = hq.copy()
    L_f = float(R.m.geom_size[R.foot[0]][0])
    lg = doc["home"]["legs_deg"]
    errs, _ = validate_physics(R, doc, L_f, lg["hip_pitch"] - lg["knee"] + lg["ankle"])
    if errs:
        return 1, errs
    msgs.append("gecerli ve taze (%d satir, HOME %s, H_HOME %.4f)" % (
        len(doc["crouch_family"]), doc["home"]["source"], doc["heights"]["H_HOME"]))
    return 0, msgs


def caps_scan(scene, derived=DEFAULT_OUT, step_deg=1.0, pitch=None):
    """Gorsel mesh voxel denetimi (sim/collision_check, gorsel model = sahnenin kendisi): HOME'dan tek eklem
    suprumeleri (sol bacak + bas; ctrlrange sinirlarina dogru) ilk girisim acisi, aile satirlari ve oturus pozu."""
    from sim import collision_check as CC
    doc = json.load(open(derived)) if derived and os.path.exists(derived) else None
    vis = visual_model(scene)
    ms = CC.MeshSet(model=vis, pitch=pitch or CC.PITCH)
    jid = lambda n: mujoco.mj_name2id(vis, mujoco.mjtObj.mjOBJ_JOINT, n)
    qa = lambda n: int(vis.jnt_qposadr[jid(n)])
    key = mujoco.mj_name2id(vis, mujoco.mjtObj.mjOBJ_KEY, C.DEFAULT_KEYFRAME)
    home = np.array(doc["home"]["qpos"]) if doc else vis.key_qpos[key].copy()
    out = {"pitch_mm": 1000.0 * ms.pitch, "limit_mm3": CC.LIMIT_MM3, "sweeps": {}, "poses": {}}
    for n in ["left_" + j for j in LEG5] + list(C.HEAD_JOINTS):
        lo, hi = _ctrl_deg(vis, n)
        q0 = math.degrees(home[qa(n)])
        res = {}
        for name, end in (("lower", lo), ("upper", hi)):
            first = None
            for v in np.arange(q0, end + (1e-9 if end > q0 else -1e-9), step_deg if end > q0 else -step_deg):
                q = home.copy()
                q[qa(n)] = math.radians(v)
                r = CC.check_pose(q, ms)
                if r["pen_mm3"] > CC.LIMIT_MM3:
                    first = {"deg": round(float(v), 3), "mm3": round(r["pen_mm3"], 3), "parts": r["pairs"][0]["parts"]}
                    break
            res[name] = {"limit_deg": round(end, 3), "first_interference": first}
        out["sweeps"][n] = res
    if doc:
        poses = [("row_%.4f" % r[0], r[1:], None) for r in doc["crouch_family"]]
        se = doc["seat"]
        poses.append(("seat", (se["pitch_deg"], se["deg"]["hip_pitch"], se["deg"]["knee"], se["deg"]["ankle"]),
                      doc["head"]["sit_deg"]))
        for name, (th, hp, kn, an), head in poses:
            q = home.copy()
            q[0:3] = [0.0, 0.0, 1.0]
            t = math.radians(th)
            q[3:7] = [math.cos(t / 2), 0.0, math.sin(t / 2), 0.0]
            for j, v in zip(FAM_J, (hp, kn, an)):
                q[qa("left_" + j)], q[qa("right_" + j)] = math.radians(v), -math.radians(v)
            for j in (C.HEAD_JOINTS if head else ()):
                q[qa(j)] = math.radians(head[j])
            r = CC.check_pose(q, ms)
            out["poses"][name] = {"pen_mm3": round(r["pen_mm3"], 3), "pairs": r["pairs"][:3]}
    return out


# --- komut satiri ---------------------------------------------------------------------------------------------
def main(argv=None):
    p = argparse.ArgumentParser(description="TR-35 durus/geometri sabitleri (rl/derived/tr35_b2.json)")
    sp = p.add_subparsers(dest="cmd", required=True)
    a = sp.add_parser("derive", help="MJCF'den sema 1 JSON uret")
    a.add_argument("--scene", required=True)
    a.add_argument("--out", default=DEFAULT_OUT)
    a.add_argument("--stub", action="store_true", help="P2-0 yer tutucu geometri (source.stub = true)")
    a.add_argument("--home", default="auto", choices=("auto", "keyframe", "family"))
    a.add_argument("--config", default=None, help="config_sha256 icin (varsayilan configs/tr35_b2.json)")
    a.add_argument("--force", action="store_true", help="dogrulama basarisizken de yaz")
    c = sp.add_parser("check", help="JSON gecerli ve sahneye gore taze mi")
    c.add_argument("--scene", required=True)
    c.add_argument("--derived", default=DEFAULT_OUT)
    s = sp.add_parser("caps-scan", help="gorsel mesh voxel taramasi (sim/collision_check)")
    s.add_argument("--scene", required=True)
    s.add_argument("--derived", default=DEFAULT_OUT)
    s.add_argument("--step", type=float, default=1.0)
    s.add_argument("--json", default=None)
    o = p.parse_args(argv)
    if o.cmd == "derive":
        import time
        t0 = time.time()
        try:
            doc, errs = derive(o.scene, o.stub, o.home, o.config, log=lambda *x: print(*x))
        except RuntimeError as e:
            print("HATA: %s" % e, file=sys.stderr)
            return 1
        print(summary(doc))
        if errs:
            print("DOGRULAMA BASARISIZ:\n  " + "\n  ".join(errs), file=sys.stderr)
            if not o.force:
                return 1
        os.makedirs(os.path.dirname(os.path.abspath(o.out)) or ".", exist_ok=True)
        with open(o.out, "w") as f:
            f.write(dumps(doc))
        print("yazildi %s (%.1f s)" % (o.out, time.time() - t0))
        return 1 if errs else 0
    if o.cmd == "check":
        code, msgs = check(o.scene, o.derived)
        print("\n".join(msgs))
        return code
    r = caps_scan(o.scene, o.derived, o.step)
    txt = json.dumps(r, indent=1, sort_keys=True)
    if o.json:
        with open(o.json, "w") as f:
            f.write(txt + "\n")
    print(txt)
    return 0


def summary(doc):
    """Insan icin ozet tablo."""
    H, hm, se, rp = doc["heights"], doc["home"], doc["seat"], doc["report"]
    L = ["HOME %s: z_kin %.4f, COM - ayak merkezi %+.1f mm, pay %.1f mm; keyframe COM %+.1f mm" % (
        hm["source"], hm["trunk_z_kin"], 1000 * hm["com_offset_x"], 1000 * hm["com_margin"],
        1000 * rp["keyframe"]["com_offset_x"]),
        "  bacak %s" % {k: round(v, 3) for k, v in hm["legs_deg"].items()},
        "  oturtma pitch %+.2f derece" % rp["home"]["settle"]["pitch_deg"],
        "H: " + ", ".join("%s %s" % (k, H[k]) for k in HEIGHT_KEYS),
        "F (h, pitch, hp, kn, an | pay mm, bas statik orani max):"]
    for r, x in zip(doc["crouch_family"], rp["rows"]):
        L.append("  %.4f %6.1f %6.1f %6.1f %7.2f | %5.1f %.2f%s" % (
            r[0], r[1], r[2], r[3], r[4], 1000 * x["margin"], max(x["head_static_ratio"].values()),
            " !" if x["head_static_flag"] else ""))
    L.append("SEAT %s pitch %.1f, gz %.3f, uygun %s, pay %.1f mm; HEAD_SIT %s" % (
        se["deg"], se["pitch_deg"], se["seat_gz"], se["feasible"], 1000 * rp["seat"]["margin"],
        doc["head"]["sit_deg"]))
    L.append("ayak araligi %.4f, FEET_MIN %.4f; sinirlar %s; kutle %.3f kg" % (
        doc["feet"]["home_lateral"], doc["feet"]["FEET_MIN_DISTANCE"], doc["caps_deg"], doc["mass"]["total_kg"]))
    L.append("EXTRA %s" % sorted(doc["extra_collision"]))
    L.append("ciftler %d; HOME temasi ile atilan %s" % (len(doc["self_collision_pairs"]),
                                                         rp["pairs"]["dropped_home_contact"]))
    L += ["UYARI: " + w for w in rp.get("warnings", [])]
    return "\n".join(L)


if __name__ == "__main__":
    sys.exit(main())
