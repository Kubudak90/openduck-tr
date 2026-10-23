# -*- coding: utf-8 -*-
"""
Kabul olcumleri (docs/workflow/sit-stand-getup/spec.md 6, A1-A4 ve istege bagli A8): ONNX politikalarini
CPU ortaminda (rl/env_cpu.py, surec ici) DR acik degerlendirir ve sonuclari JSON'a yazar.

  python3 -m rl.eval_tasks --main-policy runs/M/policy.onnx --walk-policy runs/walk-mjx-v2/policy.onnx
                           --getup-policy runs/G/policy.onnx [--episodes 64] [--a4-episodes 512] [--seed 0]
                           [--out eval.json] [--no-randomize] [--no-noise] [--no-push] [--no-head] [--bank-n N]
                           [--collision]
                           [--criteria auto|xl330|tr35] [--scene S.xml] [--robot R] [--no-backlash]
                           [--getup-head free|fixed] [--t-knee NM] [--n-scale F] [--a7-onnx X.onnx] [--no-collision]

Politika: .onnx (yaninda <ad>_meta.json), .pt ya da "zero" (rolun gorevinde sifir aksiyon). Her politikanin
gorevi meta'dan okunur ve rolune uymali (main -> "main", walk -> "walk", getup -> "getup"); uymazsa ilgili
bolumler "error" durumuyla raporlanir (diger bolumler yine olculur), cikis kodu 2.

  A1 yurume: main politikasi h = H_HOME, sit = 0 ve walk politikasi AYNI rastgele hiz komutlariyla (bolum basina
     sabit, COMMAND_RANGES, %15 sifir) birer tam bolum (20 s): bolum basina takip odulu (rew_tracking_lin_vel +
     rew_tracking_ang_vel toplami) ve dusme orani. Kabul: oran >= 0.90, dusme farki <= 2 puan.
  A2 yukseklik: sabit h komutlari (ayakta [H_MIN_STAND, H_HOME], yururken vx 0.15 ile [H_MIN_WALK, H_HOME]),
     h_ref oturduktan sonraki pencerede |trunk_z - z_ref| RMS (ortam basina). Yururken ayni pencerede canli
     ortamlarin govde cercevesi ileri hizi (IMU linvel x) ortalamasi vx_mean da olculur (yerinde durmak yurume
     sayilmaz). Kabul: hepsi < 1 cm, dusme yok, yururken her h icin vx_mean >= 0.8 * vx komutu.
  A3 otur/kalk: HOME'dan [dur 1.5 s | sit 5 s | kalk 4.5 s] ve oturus bankasindan [sit 1 s | kalk 4.5 s].
     sit basarisi: komuttan sonra <= 3.5 s icinde oturus kosulu (sit, alpha = 1, govde yerde, gz < -0.9) ve
     segment boyunca dusmeme; kalkis basarisi: sit kalktiktan sonra <= 3.0 s icinde z > 0.105, iki ayak yerde,
     gz < -0.9; gecis dusmeleri; tork doygunlugu (max|tau| > 0.9 * 0.96 olan adim orani).
     Kabul: >= %95, >= %95, < %5, < %1.
  A4 getup (--a4-episodes, varsayilan 512, bolum turu basina): dusmus ve dusus-ortasi bankalarindan 6 s'lik
     bolumler (mufredat p = 1): basari (gz < -0.9, z > 0.10, poz RMS < 0.3, |gyro| < 0.5 kesintisiz 1 s) mod basina
     (back/front/left/right; yan = left + right), her oranin yaninda Wilson %95 araligi; kafa-zemin tepe normal
     kuvveti: bolum basina HER FIZIK ALT ADIMINDAKI temaslarin en buyugu (VecEnv(head_force_peak=True); kontrol
     adimi sonunda tek ornek darbe tepelerini kacirir), medyani. Kabul: genel >= %90, her mod >= %85, medyan < 20 N.
  A8 (--collision): her rollout grubunda (A1_main, A1_walk, A2_standing, A2_walking, A3_home, A3_seated,
     A4_fallen, A4_midfall) ilk min(8, N) ortamin qpos'u her 10 kontrol adiminda (0.2 s), yalniz ortamin ilk bolumu
     surerken (ilk dusme/bitiste durur) kaydedilir ve sim/collision_check.py ile denetlenir (HOME'daki arayuz
     temaslari ve eklemli ebeveyn/cocuk ciftlerde <= 4 voxel haric). Grup basina en kotu cift, mm^3, ortam, kare
     sayisi. Kabul: main/getup gruplarinin hepsinde girisim <= 1 mm^3 (A1_walk = walk taban cizgisi politikasi,
     raporlanir ama kabule dahil degil). --collision verilmezse A8 olculmez ve genel kabule (pass, pass_all)
     dahil edilmez (JSON'da "A8": status not_run).

OpenDuck-TR kipi (docs/workflow/openduck-tr/spec.md v3.1 bolum 6, A4-A8): config.ROBOT_PROFILE == "tr35_b2", bir
politikanin meta'sinda robot "tr35_b2" ya da --criteria tr35 ise. Yukaridaki xl330 bolumleri, olcutleri ve JSON'u
degismez. tr35 kipinde N = spec N x --n-scale (--episodes / --a4-episodes kullanilmaz), varsayilan robot tr35_b2,
bosluk (--no-backlash ile kapali) ve A8 (--no-collision ile kapali) acik; --scene, --getup-head, --t-knee
(varsayilan tau_s(ds32xx) / 3, G-BENCH'e kadar yer tutucu), --a7-onnx (A7'ye ek ONNX). Butun tork olcutleri NOMINAL
tau_s ile (actuator_table; spec 3.2.3 "kabul nominal tau_s ile"); tork / ray / limit ortam-adimlarinda (canli
ortamlar, bolumun ilk sonuna dek) tau_net = tau_act - b qd, limit = LIMIT_JOINT kisit torku (alt adim en buyugu),
ray = LEG talebi (olcekleme oncesi alt adim ortalamasi). Oran kurallari: nokta = k/n, W-alt / W-ust = Wilson %95.
  A4 walk ve main (h = H_HOME, sit 0; DR + itme + bosluk; N 512 x 20 s; ortamlarin 1/4'u vx = 0.5 vx_max, 1/8'i
     wz = +0.5 wz_max, 1/8'i -0.5 wz_max, yarisi A1 rastgele komutlari): dusme W-ust <= %3; [5, 20] s ileri hiz
     ortalamasi (IMU linvel x) >= %80 komut; wz ortalamasi (gyro z) komutun +-%20'si; main takip >= %90 walk
     (rastgele grup); DS |tau_net|/tau_s p99 <= 0.85 (eklem basina en kotusu); diz tau_net RMS <= T_diz; ES3054 ve
     MG90S RMS <= stall/3 ve p99 <= 1.0; hip_yaw RMS <= 0.11 N.m; ana eklem limit p99 <= 0.1 tau_s (MG90S kafa
     eklemlerinde <= MG90S stall); ray p99 <= 16 A, > 15 A sure orani <= %2.
  A5 main, N 256: boy RMS (A2 duzeni, ayakta ve 0.5 vx_max ile yururken) <= 1.3 cm; HOME'dan otur (<= sitdown_window_s)
     ve kalk (<= standup_window_s), oturus bankasindan kalk: her biri nokta >= %95, W-alt >= %90; gecis dusmeleri
     W-ust <= %5; A4 tork / ray / limit olcutleri.
  A6 getup: dusmus bankadan etikete gore tabakali (yuz = front, yan = left + right, sirt = back), mod basina N 128,
     12 s (VecEnv.set_state): nokta >= taban, W-alt >= taban - 8 puan (80 / 70 / 50); kafa servolari
     |tau_net|/stall p95 <= 1.0, doygun adim (herhangi bir HEAD_JOINTS orani > torque_sat_frac) <= %5; ray ve limit;
     kafa-zemin tepe kuvveti medyani raporlanir. Devir: main -> devirme (play.tip_over, 4 yon sirayla) -> getup ->
     main (play.Supervisor; donusten sonra AFTER_HANDOFF_S yeniden dusmeden), N 128, nokta >= %80.
  A7 rl/export_c.py --check 1000 her ONNX politikada: max |C - ORT| <= 1e-5, ESP32-S3 tahmini < 2 ms, aktor 256-128-64.
  A8 sim/collision_check.py, tr35 gorsel sahnesinde (ayni robot / bosluk duzeni) her A4-A6 grubu.
"""
import argparse
import functools
import importlib.util
import inspect
import json
import math
import os
import shutil
import sys
import tempfile
import time
import traceback

import mujoco
import numpy as np

from . import config as C
from . import pose_bank as PB
from . import task_math as T
from . import tasks as TK
from .env_cpu import VecEnv
from .play import AFTER_HANDOFF_S, load_policy

DT = C.CTRL_DT
SAT = C.GETUP["torque_sat_frac"] * C.GETUP["torque_limit"]
CRIT = {"A1_ratio": 0.90, "A1_fall_pts": 2.0, "A2_rmse": 0.01, "A2_vx_frac": 0.8, "A3_sit": 0.95, "A3_stand": 0.95,
        "A3_trans_fall": 0.05, "A3_torque_sat": 0.01, "A4_all": 0.90, "A4_mode": 0.85, "A4_head_n": 20.0,
        "A8_mm3": 1.0}
A3_HOME = (1.5, 5.0, 4.5)       # dur, sit, kalk (s)
A3_SEATED = (1.0, 4.5)          # sit, kalk (s)
A4_S = 6.0
A4_EPISODES = 512               # bolum turu (fallen / midfall) basina; mod basina ~130-200 bolum
A8_K, A8_STRIDE = 8, 10         # grup basina kaydedilen ortam sayisi (ilk K), kare araligi (kontrol adimi)
A8_GROUPS = ("A1_main", "A1_walk", "A2_standing", "A2_walking", "A3_home", "A3_seated", "A4_fallen", "A4_midfall")
A8_BASELINE = ("A1_walk",)      # walk taban cizgisi politikasi: raporlanir, kabule dahil degil
Z95 = 1.959963984540054

# --- OpenDuck-TR (spec v3.1 bolum 6) -------------------------------------------------------------------
TR35 = "tr35_b2"
CRIT_TR35 = {
    "A4_fall_w_upper": 0.03, "A4_vx_frac": 0.80, "A4_wz_tol": 0.20, "A4_track_ratio": 0.90,
    "ds_ratio_p99": 0.85, "es_mg_rms_stall_frac": 1.0 / 3.0, "es_mg_ratio_p99": 1.0, "hip_yaw_rms_nm": 0.11,
    "limit_p99_tau_frac": 0.10, "rail_p99_a": 16.0, "rail_hi_a": C.RAIL_SOFT_A, "rail_hi_frac": 0.02,
    "A5_height_rmse": 0.013, "A5_point": 0.95, "A5_w_lower": 0.90, "A5_trans_fall_w_upper": 0.05,
    "A6_base": {"front": 0.80, "side": 0.70, "back": 0.50}, "A6_w_lower_margin": 0.08,
    "A6_head_ratio_p95": 1.0, "A6_head_sat_frac": 0.05, "A6_handover_point": 0.80,
    "A7_atol": 1e-5, "A7_ms": 2.0, "A7_hidden": [256, 128, 64], "A8_mm3": 1.0,
}
N_TR35 = {"A4": 512, "A5": 256, "A6": 128, "handover": 128}     # spec 6 orneklem buyuklukleri (x --n-scale)
TR_A4_S, TR_A4_WINDOW = 20.0, (5.0, 20.0)
TR_A4_GROUPS = (("fwd", 0.25), ("turn_pos", 0.125), ("turn_neg", 0.125), ("random", 0.5))
TR_A6_S = 12.0
TR_HO = {"stand_s": 1.0, "after_s": AFTER_HANDOFF_S}   # devir: devirmeden once dur; donusten sonra dusmeme suresi
A6_MODES = (("front", ("front",)), ("side", ("left", "right")), ("back", ("back",)))
A7_CHECK_N = 1000


def _num(x):
    if x is None:
        return None
    x = float(x)
    return x if math.isfinite(x) else None


def _stats(v):
    v = [x for x in v if x is not None]
    if not v:
        return {"n": 0, "mean": None, "median": None, "max": None}
    return {"n": len(v), "mean": _num(np.mean(v)), "median": _num(np.median(v)), "max": _num(np.max(v))}


def _rate(k, n):
    return _num(k / n) if n else None


def wilson(k, n, z=Z95):
    """Basari orani k/n icin Wilson skor araligi [alt, ust] (n = 0: [None, None])."""
    if not n:
        return [None, None]
    p, z2 = k / n, z * z
    d = 1.0 + z2 / n
    c = (p + z2 / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z2 / (4 * n * n)) / d
    return [_num(round(max(0.0, c - h), 12)), _num(round(min(1.0, c + h), 12))]


def _rate_ci(succ):
    """succ (n,) 0/1 -> {"n", "successes", "success_rate", "ci95"}."""
    n, k = int(len(succ)), int(np.sum(np.asarray(succ) > 0.5))
    return {"n": n, "successes": k, "success_rate": _rate(k, n), "ci95": wilson(k, n)}


# --- spec 6 kurallari (OpenDuck-TR): "nokta" ornek orani, "W-alt" / "W-ust" Wilson %95 sinirlari ----------------
def point(k, n):
    """Nokta: ornek orani k/n (n = 0: None)."""
    return _rate(k, n)


def w_lower(k, n):
    """W-alt: Wilson %95 alt siniri."""
    return wilson(k, n)[0]


def w_upper(k, n):
    """W-ust: Wilson %95 ust siniri."""
    return wilson(k, n)[1]


def crit(value, limit, op="<="):
    """Tek olcut: {"value", "limit", "op", "pass" (None: deger yok), "status" pass | fail | n/a}. op: "<=", ">=",
    "<", "in" (limit = [alt, ust]), "==" ."""
    v = value if op == "==" else _num(value)
    if v is None or limit is None:
        ok = None
    elif op == "<=":
        ok = v <= limit
    elif op == ">=":
        ok = v >= limit
    elif op == "<":
        ok = v < limit
    elif op == "in":
        ok = limit[0] <= v <= limit[1]
    elif op == "==":
        ok = v == limit
    else:
        raise ValueError(op)
    ok = None if ok is None else bool(ok)
    return {"value": v, "limit": limit, "op": op, "pass": ok, "status": "n/a" if ok is None else ("pass" if ok else "fail")}


def rate_rule(k, n, point_min=None, w_lower_min=None, w_upper_max=None):
    """Oran kurali (spec 6 dili): {"k", "n", "point", "w_lower", "w_upper", "criteria": {point / w_lower / w_upper}}."""
    lo, hi = wilson(k, n)
    c = {}
    if point_min is not None:
        c["point"] = crit(point(k, n), point_min, ">=")
    if w_lower_min is not None:
        c["w_lower"] = crit(lo, w_lower_min, ">=")
    if w_upper_max is not None:
        c["w_upper"] = crit(hi, w_upper_max, "<=")
    return {"k": int(k), "n": int(n), "point": point(k, n), "w_lower": lo, "w_upper": hi, "criteria": c}


def all_pass(criteria):
    """Butun olcutler (ic ice sozlukler dahil) gecti mi; deger yok (n/a) gecmez."""
    flat = []

    def walk(d):
        for v in d.values():
            if isinstance(v, dict) and "status" in v and "op" in v:
                flat.append(v["pass"] is True)
            elif isinstance(v, dict):
                walk(v)
    walk(criteria)
    return bool(flat) and all(flat)


class TorqueAcc(object):
    """tr35 tork / ray / limit toplayicisi (spec 6 A4-A6). NOMINAL tau_s (idx.act, actuator_table), gozlem eklemleri
    sirasi. add(s, live): task_state() satirlarindan canli olanlar: |tau_net| / tau_s, tau_net^2 toplami, limit_torque,
    rail_demand. s'de tau_ratio / tau_s (bolum tau_s'i) varsa ayrica egitim cezalari task_math.tr35_penalties
    (agirliksiz adim ortalamasi: torque_ratio, limit_torque, rail_demand) ve ortamin torque_sat_steps metrigi (max
    |tau_ratio| > GETUP torque_sat_frac olan adim orani) raporlanir (summary()["penalties"]; olcut degil)."""

    def __init__(self, idx):
        if getattr(idx, "act", None) is None:
            raise ValueError("tr35 olcutleri tr35_b2 modeli ister (model %s)" % getattr(idx, "robot", None))
        oa = np.asarray(idx.obs_act_id, dtype=np.int64)
        self.joints = list(C.OBS_JOINTS)
        self.tau = np.asarray(idx.act["tau_s"], dtype=np.float64)[oa]
        self.cls = [idx.act["cls"][int(a)] for a in oa]
        self.ratio, self.limit, self.rail = [], [], []
        self.sq = np.zeros(len(oa))
        self.n = 0
        self.pen, self.pen_n, self.sat_steps = {}, 0, 0

    def add(self, s, live):
        live = np.asarray(live, bool)
        if not live.any():
            return
        tn = np.asarray(s["tau_net"], dtype=np.float64)[live]
        self.ratio.append((np.abs(tn) / self.tau).astype(np.float32))
        self.limit.append(np.asarray(s["limit_torque"], dtype=np.float64)[live].astype(np.float32))
        self.rail.append(np.asarray(s["rail_demand"], dtype=np.float64)[live].astype(np.float32))
        self.sq += np.sum(tn * tn, axis=0)
        self.n += int(live.sum())
        if "tau_ratio" in s and "tau_s" in s:
            row = lambda v: np.asarray(v, dtype=np.float64)[live]
            p = T.tr35_penalties(np, {k: row(s[k]) for k in ("tau_ratio", "tau_s", "limit_torque", "rail_demand")})
            for k, v in p.items():
                self.pen[k] = self.pen.get(k, 0.0) + float(np.sum(v))
            self.pen_n += int(live.sum())
            self.sat_steps += int(np.sum(np.max(np.abs(row(s["tau_ratio"])), axis=-1) > C.GETUP["torque_sat_frac"]))

    def summary(self):
        if not self.n:
            return {"steps": 0, "basis": "nominal tau_s"}
        R, L = np.concatenate(self.ratio), np.concatenate(self.limit)
        rail = np.concatenate(self.rail).astype(np.float64)
        pc = lambda x, q: _num(np.percentile(x.astype(np.float64), q))
        rms = np.sqrt(self.sq / self.n)
        joints = {}
        for j, name in enumerate(self.joints):
            joints[name] = {"cls": self.cls[j], "tau_s": _num(self.tau[j]), "ratio_p95": pc(R[:, j], 95),
                            "ratio_p99": pc(R[:, j], 99), "ratio_max": _num(R[:, j].max()), "rms_nm": _num(rms[j]),
                            "limit_p99_nm": pc(L[:, j], 99), "limit_max_nm": _num(L[:, j].max())}
        head = [self.joints.index(n) for n in C.HEAD_JOINTS]
        sat = C.GETUP["torque_sat_frac"]
        out = {"steps": self.n, "basis": "nominal tau_s", "joints": joints,
               "rail": {"p99_a": pc(rail, 99), "max_a": _num(rail.max()), "mean_a": _num(rail.mean()),
                        "frac_gt_hi": _num(np.mean(rail > CRIT_TR35["rail_hi_a"])), "hi_a": CRIT_TR35["rail_hi_a"]},
               "head_sat_frac": _num(np.mean(np.any(R[:, head] > sat, axis=1))), "head_sat_threshold": sat}
        if self.pen_n:
            out["penalties"] = dict({k: _num(v / self.pen_n) for k, v in sorted(self.pen.items())},
                                    basis="bolum tau_s (odul terimleri, agirliksiz adim ortalamasi)",
                                    rail_soft_a=C.RAIL_SOFT_A, torque_sat_steps_frac=_num(self.sat_steps / self.pen_n))
        return out


def _ep_metrics(em):
    """A6 ortam bolum metrikleri: getup_success / head_impacts / head_impact_v2 / torque_sat_steps ortalamasi ve
    torque_sat_steps / ep_len (adim orani; tr35'te max |tau_ratio| > torque_sat_frac). Bitmeyen bolum yok sayilir."""
    if not em:
        return {}
    ok = ~np.isnan(em["ep_len"])
    out = {k: _num(np.mean(v[ok])) if ok.any() else None for k, v in em.items() if k != "ep_len"}
    out["torque_sat_steps_frac"] = (_num(np.sum(em["torque_sat_steps"][ok]) / np.sum(em["ep_len"][ok]))
                                    if ok.any() and "torque_sat_steps" in em else None)
    out["episodes"] = int(ok.sum())
    return out


def _worst(summ, key, names, div=None):
    """(en buyuk deger, eklem) summ["joints"][ad][key] (/ div(ad)) uzerinden; veri yoksa (None, None)."""
    J = summ.get("joints") or {}
    vals = [(J[n][key] / (div(n) if div else 1.0), n) for n in names if n in J and J[n][key] is not None]
    return max(vals) if vals else (None, None)


def torque_criteria(summ, t_knee, parts=("torque", "limit", "rail")):
    """spec 6 A4 tork / limit / ray olcutleri (+ "head": A6 kafa servolari) TorqueAcc.summary()'den."""
    J = summ.get("joints") or {}
    by = lambda c: [n for n in C.OBS_JOINTS if J.get(n, {}).get("cls") == c]
    tau = lambda n: J[n]["tau_s"]
    out = {}

    def add(name, v_j, limit, op="<="):
        v, j = v_j
        out[name] = dict(crit(v, limit, op), joint=j)
    if "torque" in parts:
        add("ds_ratio_p99", _worst(summ, "ratio_p99", by("ds32xx")), CRIT_TR35["ds_ratio_p99"])
        add("knee_rms_nm", _worst(summ, "rms_nm", ["left_knee", "right_knee"]), float(t_knee))
        for c in ("es3054", "mg90s"):
            names = by(c)
            stall = tau(names[0]) if names else None
            add(c + "_rms_nm", _worst(summ, "rms_nm", names),
                None if stall is None else stall * CRIT_TR35["es_mg_rms_stall_frac"])
            add(c + "_ratio_p99", _worst(summ, "ratio_p99", names), CRIT_TR35["es_mg_ratio_p99"])
        add("hip_yaw_rms_nm", _worst(summ, "rms_nm", ["left_hip_yaw", "right_hip_yaw"]), CRIT_TR35["hip_yaw_rms_nm"])
    if "limit" in parts:
        mg = by("mg90s")
        add("limit_main_p99_tau_frac", _worst(summ, "limit_p99_nm", [n for n in J if n not in mg], tau),
            CRIT_TR35["limit_p99_tau_frac"])
        add("limit_head_p99_nm", _worst(summ, "limit_p99_nm", mg), tau(mg[0]) if mg else None)
    if "rail" in parts:
        r = summ.get("rail") or {}
        out["rail_p99_a"] = crit(r.get("p99_a"), CRIT_TR35["rail_p99_a"])
        out["rail_gt15_frac"] = crit(r.get("frac_gt_hi"), CRIT_TR35["rail_hi_frac"])
    if "head" in parts:
        add("head_ratio_p95", _worst(summ, "ratio_p95", list(C.HEAD_JOINTS)), CRIT_TR35["A6_head_ratio_p95"])
        out["head_sat_frac"] = crit(summ.get("head_sat_frac"), CRIT_TR35["A6_head_sat_frac"])
    return out


def _row(s, i, n):
    """task_state() sozlugunun i. ortam satiri (play.Supervisor tek ortam ister); default_q ortak."""
    return {k: (v[i:i + 1] if isinstance(v, np.ndarray) and v.ndim >= 1 and v.shape[0] == n and k != "default_q"
                else v) for k, v in s.items()}


class _Traj(object):
    """A8 kaydi: ilk K ortamin qpos'u her A8_STRIDE kontrol adiminda, yalniz ortamin ilk bolumu surerken
    (bu adimda biten -dusme ya da sure- ortam o adimdan itibaren kaydedilmez; otomatik sifirlanan durum alinmaz)."""

    def __init__(self, on, n):
        self.k = min(A8_K, n) if on else 0
        self.alive = np.ones(self.k, bool)
        self.q = [[] for _ in range(self.k)]
        self.steps = [[] for _ in range(self.k)]

    def step(self, env, k, done):
        """k: biten kontrol adiminin indeksi (0'dan); done: env.step'in done dizisi."""
        if not self.k:
            return
        self.alive &= ~np.asarray(done[:self.k], bool)
        if (k + 1) % A8_STRIDE == 0:
            for j in np.flatnonzero(self.alive):
                self.q[j].append(env.mj_data(int(j)).qpos.copy())
                self.steps[j].append(k + 1)


class Evaluator(object):
    """xl330 (A1-A4, A8) ve OpenDuck-TR (tr35_*: A4-A8) olcumleri. robot / backlash / scene_path / getup_head
    verilirse her ortama gecer (None: surec varsayilani; getup_head yalniz getup'ta ve "free" degilse). tr35: t_knee
    (N.m; None = tau_s(ds32xx) / 3), n_scale (spec N carpani)."""

    def __init__(self, episodes=64, seed=0, randomize=True, noise=True, push=True, head_random=True, bank_n=None,
                 collision=False, log=print, a4_episodes=A4_EPISODES, robot=None, backlash=None, scene_path=None,
                 getup_head=None, t_knee=None, n_scale=1.0):
        self.N = int(episodes)
        self.N4 = int(a4_episodes)
        self.seed = int(seed)
        self.flags = dict(randomize=randomize, noise=noise, push=push, head_random=head_random)
        self.bank_n = bank_n
        self.collision = collision
        self.log = log or (lambda *a, **k: None)
        self.traj = {}
        self.robot, self.backlash, self.scene_path, self.getup_head = robot, backlash, scene_path, getup_head
        self.n_scale = float(n_scale)
        sc = lambda k, lo: max(lo, int(round(N_TR35[k] * self.n_scale)))
        self.n_a4, self.n_a5, self.n_a6, self.n_ho = sc("A4", 4), sc("A5", 1), sc("A6", 2), sc("handover", 1)
        if t_knee is None:
            from . import actuators as ACT
            t_knee = ACT.CLASSES["ds32xx"]["tau_s"] / 3.0
        self.t_knee = float(t_knee)
        self._tex, self._pidx, self._ms = None, None, None

    def _env(self, task, n, **kw):
        kw.setdefault("seed", self.seed)
        for k, v in (("robot", self.robot), ("scene_path", self.scene_path)):
            if v is not None:
                kw.setdefault(k, v)
        if self.backlash:
            kw.setdefault("backlash", True)
            kw.update(self._textures_kw(n))
        if task == "getup" and self.getup_head not in (None, "free"):
            kw.setdefault("getup_head", self.getup_head)
        return VecEnv(n, num_workers=0, task=task, episode_details=True, bank_n=self.bank_n, **self.flags, **kw)

    def _textures_kw(self, n):
        """Gecici: env_cpu.physics_model(textures=False) + bosluk eklemi derlenmiyor (apply_to_spec'in ara
        compile'indan sonra doku silme; 'texture grid not found'). Bu durumda > 16 ortamda dokulu modeller."""
        if n <= 16:
            return {}
        if self._tex is None:
            from .env_cpu import physics_model
            try:
                physics_model(self.scene_path, textures=False, variant="main", robot=self.robot, backlash=True)
                self._tex = True
            except ValueError as e:
                if "texture" not in str(e):
                    raise
                self._tex = False
                self.log("UYARI: textures=False + backlash derlenmiyor (env_cpu.physics_model); dokulu modeller")
        return {} if self._tex else {"textures": True}

    def _probe_idx(self):
        """Olcut tablolari icin RobotIndex (ortamlarla ayni sahne / robot / bosluk)."""
        if self._pidx is None:
            from .robot_model import RobotIndex, build_model
            self._pidx = RobotIndex(build_model(self.scene_path, variant="main", robot=self.robot,
                                                backlash=bool(self.backlash)))
        return self._pidx

    def _rec(self, n):
        return _Traj(self.collision, n)

    def _keep(self, name, rec):
        if rec.k:
            self.traj[name] = {"qpos": [np.array(q) for q in rec.q], "steps": [list(s) for s in rec.steps]}

    # --- A1 -------------------------------------------------------------------------
    def commands(self, n):
        rng = np.random.default_rng([self.seed, 1])
        lo = np.array([C.COMMAND_RANGES[k][0] for k in ("vx", "vy", "wz")])
        hi = np.array([C.COMMAND_RANGES[k][1] for k in ("vx", "vy", "wz")])
        c = lo + (hi - lo) * rng.random((n, 3))
        c[rng.random(n) < C.COMMAND_ZERO_PROB] = 0.0
        return c

    def tracking(self, policy, task):
        """Bir tam bolum, ortam basina sabit hiz komutu (main: h = H_HOME, sit = 0)."""
        spec = TK.get_task(task)
        cmd = self.commands(self.N)
        kw = {"reset_probs": {"home": 1.0}} if task == "main" else {}
        env = self._env(task, self.N, fixed_command=TK.pad_command(task, cmd) if task == "main" else cmd, **kw)
        n_steps = spec.max_episode_steps
        trk, ln, fell = np.full(self.N, np.nan), np.zeros(self.N), np.zeros(self.N, bool)
        alive = np.ones(self.N, bool)
        rec = self._rec(self.N)
        try:
            obs, _ = env.reset()
            for k in range(n_steps):
                obs, _, _, done, info = env.step(policy(obs))
                rec.step(env, k, done)
                if done.any():
                    ep = info["episodes"]
                    for r, j in enumerate(ep["env"]):
                        if alive[j]:
                            trk[j] = ep["rew_tracking_lin_vel"][r] + ep["rew_tracking_ang_vel"][r]
                            ln[j] = ep["ep_len"][r]
                            fell[j] = not info["time_outs"][j]
                            alive[j] = False
                if not alive.any():
                    break
        finally:
            env.close()
        self._keep("A1_" + task, rec)
        return {"episodes": self.N, "tracking_per_episode": _num(np.nanmean(trk)),
                "tracking_per_second": _num(np.nanmean(trk / np.maximum(ln, 1) / DT)),
                "fall_rate": _num(fell.mean()), "ep_len_mean_s": _num(ln.mean() * DT)}

    def a1(self, main, walk):
        out = {"walk": self.tracking(walk, "walk")}
        if main is None:
            return out
        out["main"] = self.tracking(main, "main")
        w, m = out["walk"], out["main"]
        ratio = m["tracking_per_episode"] / w["tracking_per_episode"] if w["tracking_per_episode"] else None
        dpts = 100.0 * (m["fall_rate"] - w["fall_rate"])
        out.update(tracking_ratio=_num(ratio), fall_rate_delta_pts=_num(dpts),
                   **{"pass": bool(ratio is not None and ratio >= CRIT["A1_ratio"] and dpts <= CRIT["A1_fall_pts"])})
        return out

    # --- A2 -------------------------------------------------------------------------
    def height(self, policy, walking, n=None, vx=None, acc=None, name=None, rmse_lim=None):
        """A2 (xl330) / A5 boy (tr35: n, vx = 0.5 vx_max, acc = TorqueAcc, name = A8 grubu, rmse_lim)."""
        N = self.N if n is None else int(n)
        lo = C.H_MIN_WALK if walking else C.H_MIN_STAND
        hs = np.linspace(lo, C.H_HOME, N)
        vx = (0.15 if vx is None else float(vx)) if walking else 0.0
        cmd = np.c_[np.full(N, vx), np.zeros((N, 2)), hs, np.zeros(N)]
        t0 = (C.H_HOME - C.H_MIN_STAND) / C.H_RATE + 1.5            # h_ref oturur + 1.5 s
        n_steps = int(round((t0 + 4.0) / DT))
        env = self._env("main", N, fixed_command=cmd, reset_probs={"home": 1.0}, max_episode_steps=n_steps + 1)
        e2, ne, fell = np.zeros(N), np.zeros(N), np.zeros(N, bool)
        vxs = np.zeros(N)
        rec = self._rec(N)
        try:
            obs, _ = env.reset()
            for k in range(n_steps):
                obs, _, _, done, info = env.step(policy(obs))
                rec.step(env, k, done)
                fell |= done
                s = None
                if acc is not None:
                    s = env.task_state()
                    acc.add(s, ~fell)
                if (k + 1) * DT >= t0 - 1e-9:
                    s = env.task_state() if s is None else s
                    e = s["trunk_z"] - T.z_ref(np, s["h_ref"], s["alpha"])
                    ok = ~fell
                    e2[ok] += e[ok] ** 2
                    vxs[ok] += s["linvel"][ok, 0]               # govde cercevesi ileri hiz
                    ne[ok] += 1
        finally:
            env.close()
        self._keep(name or ("A2_walking" if walking else "A2_standing"), rec)
        rmse = np.where(ne > 0, np.sqrt(e2 / np.maximum(ne, 1)), np.nan)
        vxm = np.where(ne > 0, vxs / np.maximum(ne, 1), np.nan)
        vx_min = CRIT["A2_vx_frac"] * vx
        with np.errstate(invalid="ignore"):
            moving = bool(np.all(vxm >= vx_min - 1e-12)) if walking else True
        lim = CRIT["A2_rmse"] if rmse_lim is None else rmse_lim
        return {"h": hs.tolist(), "vx": vx, "rmse": [_num(x) for x in rmse], "vx_mean": [_num(x) for x in vxm],
                "vx_min_required": vx_min if walking else None, "fell": fell.tolist(),
                "max_rmse": _num(np.nanmax(rmse)) if np.any(ne > 0) else None,
                "min_vx_mean": _num(np.nanmin(vxm)) if np.any(ne > 0) else None, "moving": moving,
                "pass": bool(np.all(~fell) and np.all(rmse < lim) and moving)}

    def a2(self, main):
        st, wk = self.height(main, False), self.height(main, True)
        return {"standing": st, "walking": wk, "pass": bool(st["pass"] and wk["pass"])}

    # --- A3 -------------------------------------------------------------------------
    def _sit_stand_run(self, policy, seated, n=None, sched=None, acc=None, name=None, stand_gz=None):
        """Otur / kalk rollout'u (A3 xl330, A5 tr35). sched: (HOME (dur, sit, kalk), oturus (sit, kalk)) s; stand_gz:
        kalkis dik testi (None: MAIN seat_gz, xl330). Ham diziler dondurur."""
        cmd_sit = np.array([0.0, 0.0, 0.0, C.H_HOME, 1.0])
        cmd_up = np.array([0.0, 0.0, 0.0, C.H_HOME, 0.0])
        home_s, seat_s = (A3_HOME, A3_SEATED) if sched is None else sched
        if seated:
            t_sit, t_up = 0.0, seat_s[0]
            total = sum(seat_s)
            first = cmd_sit
        else:
            t_sit, t_up = home_s[0], home_s[0] + home_s[1]
            total = sum(home_s)
            first = cmd_up
        stand_gz = C.MAIN["seat_gz"] if stand_gz is None else stand_gz
        N = self.N if n is None else int(n)
        n_steps = int(round(total / DT))
        env = self._env("main", N, fixed_command=first, max_episode_steps=n_steps + 1,
                        reset_probs={"seated": 1.0} if seated else {"home": 1.0})
        t_seated, t_stand = np.full(N, np.nan), np.full(N, np.nan)
        fell_t = np.full(N, np.nan)
        sat = tot = 0
        rec = self._rec(N)
        try:
            obs, _ = env.reset()
            phase = 0
            for k in range(n_steps):
                t = k * DT
                if phase == 0 and not seated and t >= t_sit - 1e-9:
                    obs, _ = env.set_commands(cmd_sit, refresh_obs=True)
                    phase = 1
                if phase <= 1 and t >= t_up - 1e-9:
                    obs, _ = env.set_commands(cmd_up, refresh_obs=True)
                    phase = 2
                obs, _, _, done, info = env.step(policy(obs))
                rec.step(env, k, done)
                t1 = (k + 1) * DT
                newly = done & np.isnan(fell_t) & ~info["time_outs"]
                fell_t[newly] = t1
                live = np.isnan(fell_t) & ~done
                s = env.task_state()
                if acc is not None:
                    acc.add(s, live)
                g = T.projected_gravity(np, s["quat"])[:, 2]
                up = g < C.MAIN["seat_gz"]
                seated_ok = (s["command"][:, 4] > 0.5) & (T.is_seated(np, s["alpha"]) > 0.5) & (s["trunk_floor"] > 0.5) & up
                stood = (s["command"][:, 4] < 0.5) & (s["trunk_z"] > C.MAIN["standup_z"]) & np.all(s["contact"] > 0.5, 1) \
                    & (g < stand_gz)
                m = live & seated_ok & np.isnan(t_seated) & (t1 > t_sit)
                t_seated[m] = t1 - t_sit
                m = live & stood & np.isnan(t_stand) & (t1 > t_up)
                t_stand[m] = t1 - t_up
                tau = np.max(np.abs(s["torque"]), axis=1)
                sat += int(np.sum(live & (tau > SAT)))
                tot += int(np.sum(live))
        finally:
            env.close()
        self._keep(name or ("A3_seated" if seated else "A3_home"), rec)
        return {"N": N, "t_sit": t_sit, "t_up": t_up, "t_seated": t_seated, "t_stand": t_stand, "fell_t": fell_t,
                "sat": sat, "tot": tot}

    def sit_stand(self, policy, seated):
        r = self._sit_stand_run(policy, seated)
        N, t_up, t_seated, t_stand, fell_t = r["N"], r["t_up"], r["t_seated"], r["t_stand"], r["fell_t"]
        sat, tot = r["sat"], r["tot"]
        ok_sit_seg = np.isnan(fell_t) | (fell_t > t_up)              # sit segmenti boyunca dusmedi
        with np.errstate(invalid="ignore"):
            sit_ok = (t_seated <= C.MAIN["sitdown_window_s"] + 1e-9) & ok_sit_seg
            stand_ok = (t_stand <= C.MAIN["standup_window_s"] + 1e-9) & np.isnan(fell_t)
        out = {"episodes": N, "fall_rate": _num(np.mean(~np.isnan(fell_t))),
               "stand_success_rate": _num(stand_ok.mean()), "t_to_stand": _stats(t_stand[~np.isnan(t_stand)].tolist()),
               "torque_sat_frac": _rate(sat, tot), "torque_sat_threshold_nm": SAT}
        if not seated:
            out.update(sit_success_rate=_num(sit_ok.mean()), t_to_seated=_stats(t_seated[~np.isnan(t_seated)].tolist()))
        return out

    def a3(self, main):
        home, seat = self.sit_stand(main, False), self.sit_stand(main, True)
        n = home["episodes"] + seat["episodes"]
        fall = (home["fall_rate"] * home["episodes"] + seat["fall_rate"] * seat["episodes"]) / n
        sat = max(home["torque_sat_frac"] or 0.0, seat["torque_sat_frac"] or 0.0)
        ok = (home["sit_success_rate"] >= CRIT["A3_sit"] and home["stand_success_rate"] >= CRIT["A3_stand"]
              and seat["stand_success_rate"] >= CRIT["A3_stand"] and fall < CRIT["A3_trans_fall"]
              and sat < CRIT["A3_torque_sat"])
        return {"from_home": home, "from_seated": seat, "transition_fall_rate": _num(fall), "torque_sat_frac": sat,
                "pass": bool(ok)}

    # --- A4 -------------------------------------------------------------------------
    @staticmethod
    def head_force(env, n):
        """(n,) kafa geomlari-zemin temaslarinin normal kuvvet toplami (N), kontrol adimi sonundaki temaslar
        (tek ornek; A4 tepe kuvveti VecEnv(head_force_peak=True) ile her alt adimdan olculur)."""
        idx = env.robot_index
        head = set(int(g) for g in idx.head_geoms)
        f6 = np.zeros(6)
        out = np.zeros(n)
        for i in range(n):
            m, d = env.mj_model(i), env.mj_data(i)
            for c in range(d.ncon):
                con = d.contact[c]
                g = (int(con.geom1), int(con.geom2))
                if idx.floor_geom in g and (g[0] in head or g[1] in head):
                    mujoco.mj_contactForce(m, d, c, f6)
                    out[i] += abs(f6[0])
        return out

    def getup(self, policy, kind):
        n_steps = int(round(A4_S / DT))
        N = self.N4
        env = self._env("getup", N, reset_probs={kind: 1.0}, progress=1.0, max_episode_steps=n_steps,
                        head_force_peak=True)
        peak, t_succ = np.zeros(N), np.full(N, np.nan)
        alive = np.ones(N, bool)
        rec = self._rec(N)
        try:
            obs, _ = env.reset()
            s = env.task_state()
            label = s["bank_label"].astype(int)
            for k in range(n_steps):
                obs, _, _, done, info = env.step(policy(obs))
                t1 = (k + 1) * DT
                s = env.task_state()
                # bu adimin alt adim tepe kuvveti (sifirlamada korunur: son adimda da bolume ait)
                peak = np.where(alive, np.maximum(peak, s["head_force_peak"]), peak)
                rec.step(env, k, done)
                if done.all():          # son adim: ortamlar sifirlandi, olcumler bolumden (info) gelir
                    ep = info["episodes"]
                    succ = np.zeros(N)
                    succ[ep["env"]] = ep["getup_success"]
                    break
                alive &= ~done          # bozuk fizik: bolum bitti, yeni bolum sayilmaz
                m = alive & (s["ev_succeeded"] > 0.5) & np.isnan(t_succ)
                t_succ[m] = t1
            else:
                succ = (~np.isnan(t_succ)).astype(float)
        finally:
            env.close()
        self._keep("A4_" + kind, rec)
        return label, succ, peak, t_succ

    def a4(self, policy):
        res = {"episodes_per_kind": self.N4, "head_force_sampling": "per_substep_max"}
        allsucc, allpeak = [], []
        for kind in ("fallen", "midfall"):
            label, succ, peak, t_succ = self.getup(policy, kind)
            modes = {}
            for li, name in enumerate(PB.LABELS):
                m = label == li
                if m.any():
                    modes[name] = _rate_ci(succ[m])
            side = np.isin(label, [PB.LABELS.index("left"), PB.LABELS.index("right")])
            if side.any():
                modes["side"] = _rate_ci(succ[side])
            r = _rate_ci(succ)
            res[kind] = {"episodes": r["n"], "success_rate": r["success_rate"], "success_ci95": r["ci95"],
                         "modes": modes, "t_to_success": _stats(t_succ[~np.isnan(t_succ)].tolist()),
                         "head_force_peak_n": _stats(peak.tolist())}
            allsucc.append(succ)
            allpeak.append(peak)
        succ, peak = np.concatenate(allsucc), np.concatenate(allpeak)
        fm = res["fallen"]["modes"]
        mode_ok = all(fm.get(k, {"success_rate": 0.0})["success_rate"] >= CRIT["A4_mode"] for k in ("front", "back", "side"))
        med = float(np.median(peak))
        r = _rate_ci(succ)
        res.update(success_rate=r["success_rate"], success_ci95=r["ci95"], head_force_peak_median_n=med,
                   **{"pass": bool(r["success_rate"] >= CRIT["A4_all"] and mode_ok and med < CRIT["A4_head_n"])})
        return res

    # --- A8 -------------------------------------------------------------------------
    def a8(self, ms=None):
        """ms: collision_check.MeshSet (None: surec varsayilan gorsel modeli, xl330)."""
        t0 = time.time()
        cc = load_collision_check()
        ms = cc.mesh_set() if ms is None else ms
        lim = CRIT["A8_mm3"]
        groups = {}
        for name in [g for g in A8_GROUPS if g in self.traj] + sorted(set(self.traj) - set(A8_GROUPS)):
            tr = self.traj[name]
            g = {"envs": len(tr["qpos"]), "frames": 0, "frames_per_env": [], "max_pen_mm3": 0.0, "worst": None,
                 "pairs": {}, "joint_tolerated": {}, "min_nonfoot_mesh_z_mm": None,
                 "in_verdict": name not in A8_BASELINE}
            for j, (q, st) in enumerate(zip(tr["qpos"], tr["steps"])):
                g["frames_per_env"].append(len(q))
                if not len(q):
                    continue
                r = cc.check_trajectory(q, stride=1, ms=ms, limit=lim)
                g["frames"] += r["frames"]
                for key, dst in (("pairs", g["pairs"]), ("joint_tolerated", g["joint_tolerated"])):
                    for k, v in r[key].items():
                        dst[k] = max(dst.get(k, 0.0), v)
                w = r["worst"]
                if w is not None and (g["worst"] is None or w["mm3"] > g["worst"]["mm3"]):
                    g["worst"] = {"env": j, "step": st[w["frame"]], "t_s": _num(st[w["frame"]] * DT),
                                  "parts": w["parts"], "bodies": w["bodies"], "mm3": w["mm3"]}
                z = r["min_nonfoot_mesh_z_mm"]
                if z is not None and (g["min_nonfoot_mesh_z_mm"] is None or z < g["min_nonfoot_mesh_z_mm"]):
                    g["min_nonfoot_mesh_z_mm"] = z
            g["max_pen_mm3"] = g["worst"]["mm3"] if g["worst"] else 0.0
            g["pairs"] = dict(sorted(g["pairs"].items(), key=lambda kv: -kv[1])[:8])
            g["joint_tolerated"] = dict(sorted(g["joint_tolerated"].items(), key=lambda kv: -kv[1])[:4])
            g["pass"] = bool(g["max_pen_mm3"] <= lim)
            groups[name] = g
        ver = [k for k, g in groups.items() if g["in_verdict"]]
        worst = max(ver, key=lambda k: groups[k]["max_pen_mm3"]) if ver else None
        out = {"groups": groups, "verdict_groups": ver, "baseline_groups": [k for k in groups if k not in ver],
               "envs_per_group": A8_K, "stride_steps": A8_STRIDE, "frames": int(sum(g["frames"] for g in groups.values())),
               "max_pen_mm3": groups[worst]["max_pen_mm3"] if worst else None, "worst_group": worst,
               "limit_mm3": lim}
        out.update(ms.info())
        out["check_seconds"] = round(time.time() - t0, 1)
        out["pass"] = bool(all(groups[k]["pass"] for k in ver)) if ver else None
        if not ver:
            out["note"] = "main/getup yorungesi yok: yalniz taban cizgisi gruplari olculdu, kabul karari yok"
        return out

    # ===================================================================================================
    # OpenDuck-TR (spec v3.1 bolum 6, A4-A8)
    # ===================================================================================================
    def _acc(self):
        return TorqueAcc(self._probe_idx())

    def _bank(self, kind, n):
        """Poz bankasi (cekirdek imzasi robot= / scene_path= destekliyorsa ortamlarla ayni model)."""
        kw = {}
        par = inspect.signature(PB.bank).parameters
        if "robot" in par and self.robot is not None:
            kw["robot"] = self.robot
        if "scene_path" in par and self.scene_path:
            kw["scene_path"] = self.scene_path
        return PB.bank(kind, n, **kw)

    # --- A4 ------------------------------------------------------------------------------------------
    def tr35_locomotion(self, policy, task):
        """A4 tek politika (walk / main): N ortam x 20 s, gruplar TR_A4_GROUPS; dusme, hiz, takip, tork."""
        N = self.n_a4
        n_steps = int(round(TR_A4_S / DT))
        k0 = int(round(TR_A4_WINDOW[0] / DT))
        R = C.COMMAND_RANGES
        vx, wz = 0.5 * R["vx"][1], 0.5 * R["wz"][1]
        cnt = [int(round(f * N)) for _, f in TR_A4_GROUPS[:-1]]
        cnt.append(N - sum(cnt))
        names = [g for g, _ in TR_A4_GROUPS]
        b = np.cumsum([0] + cnt)
        grp = {g: np.arange(b[i], b[i + 1]) for i, g in enumerate(names)}
        cmd = np.zeros((N, 3))
        cmd[grp["fwd"], 0] = vx
        cmd[grp["turn_pos"], 2] = wz
        cmd[grp["turn_neg"], 2] = -wz
        cmd[grp["random"]] = self.commands(len(grp["random"]))
        kw = {"reset_probs": {"home": 1.0}} if task == "main" else {}
        env = self._env(task, N, fixed_command=TK.pad_command(task, cmd) if task == "main" else cmd,
                        max_episode_steps=n_steps, **kw)
        acc = self._acc()
        trk, fell, alive = np.full(N, np.nan), np.zeros(N, bool), np.ones(N, bool)
        vxs, wzs, nw = np.zeros(N), np.zeros(N), np.zeros(N)
        rec = self._rec(N)
        try:
            obs, _ = env.reset()
            for k in range(n_steps):
                obs, _, _, done, info = env.step(policy(obs))
                rec.step(env, k, done)
                if done.any():
                    ep = info["episodes"]
                    for r, j in enumerate(ep["env"]):
                        if alive[j]:
                            trk[j] = ep["rew_tracking_lin_vel"][r] + ep["rew_tracking_ang_vel"][r]
                            fell[j] = not info["time_outs"][j]
                            alive[j] = False
                if not alive.any():
                    break
                s = env.task_state()
                acc.add(s, alive)
                if k + 1 >= k0:
                    vxs[alive] += s["linvel"][alive, 0]
                    wzs[alive] += s["gyro"][alive, 2]
                    nw[alive] += 1
        finally:
            env.close()
        self._keep("A4_" + task, rec)

        def gmean(g, x, c):
            sel = grp[g][nw[grp[g]] > 0]
            m = _num(np.mean(x[sel] / nw[sel])) if len(sel) else None
            return {"cmd": c, "envs": int(len(grp[g])), "envs_in_window": int(len(sel)), "mean": m,
                    "frac": _num(m / c) if m is not None and c else None}
        sp = {"vx_fwd": gmean("fwd", vxs, vx), "wz_pos": gmean("turn_pos", wzs, wz), "wz_neg": gmean("turn_neg", wzs, -wz)}
        falls = rate_rule(int(fell.sum()), N, w_upper_max=CRIT_TR35["A4_fall_w_upper"])
        summ = acc.summary()
        tol = CRIT_TR35["A4_wz_tol"]
        cr = {"falls_w_upper": falls["criteria"]["w_upper"],
              "vx_fwd_frac": crit(sp["vx_fwd"]["frac"], CRIT_TR35["A4_vx_frac"], ">="),
              "wz_pos_frac": crit(sp["wz_pos"]["frac"], [1.0 - tol, 1.0 + tol], "in"),
              "wz_neg_frac": crit(sp["wz_neg"]["frac"], [1.0 - tol, 1.0 + tol], "in")}
        cr.update(torque_criteria(summ, self.t_knee))
        rnd = grp["random"]
        return {"episodes": N, "seconds": TR_A4_S, "window_s": list(TR_A4_WINDOW),
                "groups": {g: int(len(v)) for g, v in grp.items()}, "falls": falls, "speed": sp,
                "tracking_per_episode": _num(np.nanmean(trk[rnd])) if len(rnd) and np.any(~np.isnan(trk[rnd])) else None,
                "torque": summ, "criteria": cr, "pass": all_pass(cr)}

    def tr35_a4(self, walk, main):
        out = {}
        for task, pol in (("walk", walk), ("main", main)):
            if pol is not None:
                out[task] = self.tr35_locomotion(pol, task)
        cr = {}
        if "walk" in out and "main" in out:
            w, m = out["walk"]["tracking_per_episode"], out["main"]["tracking_per_episode"]
            out["tracking_ratio"] = _num(m / w) if (w and m is not None) else None
            cr["main_tracking_ratio"] = crit(out["tracking_ratio"], CRIT_TR35["A4_track_ratio"], ">=")
        out["criteria"] = cr
        out["missing"] = [t for t in ("walk", "main") if t not in out]
        out["pass"] = bool(not out["missing"] and all(out[t]["pass"] for t in ("walk", "main")) and all_pass(cr))
        return out

    # --- A5 ------------------------------------------------------------------------------------------
    def tr35_a5(self, main):
        acc = self._acc()
        N = self.n_a5
        lim = CRIT_TR35["A5_height_rmse"]
        st = self.height(main, False, n=N, acc=acc, name="A5_standing", rmse_lim=lim)
        wk = self.height(main, True, n=N, vx=0.5 * C.COMMAND_RANGES["vx"][1], acc=acc, name="A5_walking",
                         rmse_lim=lim)
        sw, uw = C.MAIN["sitdown_window_s"], C.MAIN["standup_window_s"]
        sched = ((1.5, sw + 1.5, uw + 1.5), (1.0, uw + 1.5))
        stand_gz = C.MAIN.get("stand_gz", C.MAIN["seat_gz"])
        runs = {"home": self._sit_stand_run(main, False, N, sched, acc, "A5_home", stand_gz),
                "seated": self._sit_stand_run(main, True, N, sched, acc, "A5_seated", stand_gz)}
        P, WL = CRIT_TR35["A5_point"], CRIT_TR35["A5_w_lower"]
        res, cr, falls, n_all = {}, {}, 0, 0
        for kind, r in runs.items():
            ft = r["fell_t"]
            with np.errstate(invalid="ignore"):
                stand_ok = (r["t_stand"] <= uw + 1e-9) & np.isnan(ft)
                sit_ok = (r["t_seated"] <= sw + 1e-9) & (np.isnan(ft) | (ft > r["t_up"]))
            d = {"episodes": r["N"], "schedule_s": list(sched[0] if kind == "home" else sched[1]),
                 "falls": int(np.sum(~np.isnan(ft))), "stand": rate_rule(int(stand_ok.sum()), r["N"], P, WL),
                 "t_to_stand": _stats(r["t_stand"][~np.isnan(r["t_stand"])].tolist())}
            cr["stand_from_" + kind] = d["stand"]["criteria"]
            if kind == "home":
                d["sit"] = rate_rule(int(sit_ok.sum()), r["N"], P, WL)
                d["t_to_seated"] = _stats(r["t_seated"][~np.isnan(r["t_seated"])].tolist())
                cr["sit_from_home"] = d["sit"]["criteria"]
            falls += d["falls"]
            n_all += r["N"]
            res[kind] = d
        tf = rate_rule(falls, n_all, w_upper_max=CRIT_TR35["A5_trans_fall_w_upper"])
        cr["transition_falls_w_upper"] = tf["criteria"]["w_upper"]
        cr["height_rmse_standing"] = crit(st["max_rmse"], lim)
        cr["height_rmse_walking"] = crit(wk["max_rmse"], lim)
        summ = acc.summary()
        cr.update(torque_criteria(summ, self.t_knee))
        return {"height": {"standing": st, "walking": wk}, "from_home": res["home"], "from_seated": res["seated"],
                "windows_s": {"sitdown": sw, "standup": uw}, "stand_gz": stand_gz, "transition_falls": tf,
                "torque": summ, "criteria": cr, "pass": all_pass(cr)}

    # --- A6 ------------------------------------------------------------------------------------------
    def _fallen_bank(self):
        """A6 icin dusmus banka: bank_n (yoksa config) boyutunda; front/back/left/right'tan biri yoksa config boyutuna
        dek buyutulur (onek kararli)."""
        n = self.bank_n.get("fallen") if isinstance(self.bank_n, dict) else self.bank_n
        full = int(C.POSE_BANK["fallen_n"])
        n = int(n or full)
        while True:
            b = self._bank("fallen", n)
            if all(np.any(b["label"] == PB.LABELS.index(l)) for l in ("front", "back", "left", "right")) or n >= full:
                return b
            n = min(4 * n, full)

    @staticmethod
    def stratified(labels, mode, n):
        """Mod basina tabakali secim: banka sirasiyla ilk girdiler (azsa basa sararak tekrar); side = left + right
        (n // 2 + kalan left, n // 2 right). Dondurur banka indeksleri (n,)."""
        labs = dict(A6_MODES)[mode]
        per = [n // len(labs) + (1 if i < n % len(labs) else 0) for i in range(len(labs))]
        out = []
        for l, k in zip(labs, per):
            ids = np.flatnonzero(np.asarray(labels) == PB.LABELS.index(l))
            if k and not len(ids):
                raise ValueError("dusmus bankada '%s' etiketli girdi yok" % l)
            out.append(np.resize(ids, k).astype(np.int64))
        return np.concatenate(out)

    def _getup_from(self, policy, bank, ids, name, acc, metrics=None):
        """Banka girdilerinden (VecEnv.set_state) TR_A6_S getup bolumu: (basari (N,), tepe kuvvet, t_basari). metrics
        (dict) verilirse ortamin bolum metrikleri (TaskSpec.episode_metrics, ep_len) ortam basina doldurulur."""
        from .robot_model import expand_state
        N = len(ids)
        n_steps = int(round(TR_A6_S / DT))
        env = self._env("getup", N, reset_probs={"fallen": 1.0}, progress=1.0, max_episode_steps=n_steps,
                        head_force_peak=True)
        peak, t_succ = np.zeros(N), np.full(N, np.nan)
        alive = np.ones(N, bool)
        succ = None
        rec = self._rec(N)
        try:
            env.reset()
            Q, V = expand_state(env.mj_model(0), bank["qpos"][ids], bank["qvel"][ids])
            for i in range(N):
                obs, _ = env.set_state(i, Q[i], V[i], ctrl=np.array(bank["ctrl"][ids[i]]))
            for k in range(n_steps):
                obs, _, _, done, info = env.step(policy(obs))
                t1 = (k + 1) * DT
                s = env.task_state()
                peak = np.where(alive, np.maximum(peak, s["head_force_peak"]), peak)
                rec.step(env, k, done)
                acc.add(s, alive & ~done)
                if done.all():
                    ep = info["episodes"]
                    succ = np.zeros(N)
                    succ[ep["env"]] = ep["getup_success"]
                    succ[~alive] = 0.0
                    if metrics is not None:
                        for k in ["ep_len"] + list(TK.get_task("getup").episode_metrics):
                            v = np.full(N, np.nan)
                            v[ep["env"]] = ep[k]
                            v[~alive] = np.nan
                            metrics[k] = v
                    break
                alive &= ~done
                m = alive & (s["ev_succeeded"] > 0.5) & np.isnan(t_succ)
                t_succ[m] = t1
            if succ is None:
                succ = (~np.isnan(t_succ)).astype(float)
        finally:
            env.close()
        self._keep(name, rec)
        return succ, peak, t_succ

    def tr35_handover(self, main, getup):
        """main -> devirme -> getup -> main (play.Supervisor, ortam basina), N ortam. Basari: main'e donus ve
        TR_HO["after_s"] boyunca yeniden dusmeme. Basarisizlik nedenleri: no_fall (TIP_MAX devirmede dusmedi),
        getup_timeout (TR_A6_S), refall, physics (bozuk fizik)."""
        from . import play as PL
        N = self.n_ho
        tips = sorted(PL.TIP_AXES)
        t_tip0, after = TR_HO["stand_s"], TR_HO["after_s"]
        n_steps = int(round((t_tip0 + PL.TIP_MAX * PL.TIP_RETRY_S + TR_A6_S + after) / DT))
        zero = TK.pad_command("main", np.zeros(3))
        env = self._env("main", N, fixed_command=zero, reset_probs={"home": 1.0}, terminate=False,
                        model_variant="getup", max_episode_steps=n_steps + 1)
        genv = self._env("getup", N, seed=self.seed + 1, reset_probs={"stand": 1.0}, progress=1.0,
                         max_episode_steps=n_steps + 1)
        sups = [PL.Supervisor() for _ in range(N)]
        mode = np.zeros(N, np.int64)                      # 0 main, 1 getup
        n_tip, t_tip = np.zeros(N, np.int64), np.full(N, np.nan)
        t_fall, t_up = np.full(N, np.nan), np.full(N, np.nan)
        result, why = np.full(N, -1), [""] * N
        dirs = [tips[i % len(tips)] for i in range(N)]

        def fail(i, r):
            result[i], why[i] = 0, r
        try:
            obs, _ = env.reset()
            gobs, _ = genv.reset()
            for k in range(n_steps):
                t = k * DT
                for i in np.flatnonzero((mode == 0) & np.isnan(t_fall) & (result < 0)):
                    if (n_tip[i] == 0 and t >= t_tip0 - 1e-9) or \
                            (0 < n_tip[i] < PL.TIP_MAX and t - t_tip[i] >= PL.TIP_RETRY_S - 1e-9):
                        PL.tip_over(env.mj_data(int(i)), dirs[i])
                        n_tip[i] += 1
                        t_tip[i] = t
                    elif n_tip[i] >= PL.TIP_MAX and t - t_tip[i] >= PL.TIP_RETRY_S - 1e-9:
                        fail(i, "no_fall")
                obs, _, _, done, _ = env.step(main(obs))
                gobs, _, _, gdone, _ = genv.step(getup(gobs))
                t1 = (k + 1) * DT
                sm, sg = env.task_state(), genv.task_state()
                cm = cg = False
                for i in range(N):
                    if result[i] >= 0:
                        continue
                    if (done if mode[i] == 0 else gdone)[i]:
                        fail(i, "physics")
                        continue
                    if mode[i] == 0:
                        new = sups[i].update(_row(sm, i, N))
                        if not np.isnan(t_up[i]) and t1 - t_up[i] >= after - 1e-9 and new is None:
                            result[i] = 1
                        elif new == "getup":
                            if not np.isnan(t_up[i]):
                                fail(i, "refall")
                                continue
                            d = env.mj_data(i)
                            genv.set_state(i, d.qpos.copy(), d.qvel.copy(), warmstart=d.qacc_warmstart.copy())
                            mode[i], t_fall[i], cg = 1, t1, True
                    else:
                        new = sups[i].update(_row(sg, i, N))
                        if new == "main":
                            d = genv.mj_data(i)
                            env.set_state(i, d.qpos.copy(), d.qvel.copy(), warmstart=d.qacc_warmstart.copy())
                            mode[i], t_up[i], cm = 0, t1, True
                        elif t1 - t_fall[i] >= TR_A6_S - 1e-9:
                            fail(i, "getup_timeout")
                if cm:
                    obs, _ = env.observe()
                if cg:
                    gobs, _ = genv.observe()
                if np.all(result >= 0):
                    break
        finally:
            env.close()
            genv.close()
        for i in np.flatnonzero(result < 0):
            fail(i, "incomplete")
        ok = result == 1
        reasons = {}
        for i in np.flatnonzero(~ok):
            reasons[why[i]] = reasons.get(why[i], 0) + 1
        up = t_up - t_fall
        rr = rate_rule(int(ok.sum()), N, point_min=CRIT_TR35["A6_handover_point"])
        return {"episodes": N, "tip_w": float(PL.TIP_W), "tips": {d: dirs.count(d) for d in tips},
                "successes": int(ok.sum()), "rate": rr, "failures": reasons,
                "t_fall_to_main": _stats(up[~np.isnan(up)].tolist()), "tips_used": _stats(n_tip.tolist()),
                "criteria": rr["criteria"], "pass": all_pass(rr["criteria"])}

    def tr35_a6(self, getup, main=None):
        acc = self._acc()
        bank = self._fallen_bank()
        N = self.n_a6
        modes, cr, peaks = {}, {}, []
        for mode, labs in A6_MODES:
            ids = self.stratified(bank["label"], mode, N)
            em = {}
            succ, peak, t_succ = self._getup_from(getup, bank, ids, "A6_" + mode, acc, em)
            base = CRIT_TR35["A6_base"][mode]
            rr = rate_rule(int(np.sum(succ > 0.5)), len(ids), base, base - CRIT_TR35["A6_w_lower_margin"])
            lab = bank["label"][ids]
            modes[mode] = {"episodes": int(len(ids)), "labels": {l: int(np.sum(lab == PB.LABELS.index(l))) for l in labs},
                           "unique_bank_entries": int(len(np.unique(ids))), "rate": rr,
                           "t_to_success": _stats(t_succ[~np.isnan(t_succ)].tolist()),
                           "head_force_peak_n": _stats(peak.tolist()), "criteria": rr["criteria"],
                           "episode_metrics": _ep_metrics(em)}
            cr[mode] = rr["criteria"]
            peaks.append(peak)
        summ = acc.summary()
        cr.update(torque_criteria(summ, self.t_knee, parts=("head", "limit", "rail")))
        out = {"episodes_per_mode": N, "seconds": TR_A6_S, "bank_n": int(len(bank["label"])), "modes": modes,
               "head_force_peak_median_n": _num(np.median(np.concatenate(peaks))), "torque": summ}
        if main is not None:
            out["handover"] = self.tr35_handover(main, getup)
            cr["handover"] = out["handover"]["criteria"]
        else:
            out["handover"] = {"status": "skipped", "reason": "main politikasi verilmedi"}
        out["criteria"] = cr
        out["pass"] = bool(all_pass(cr) and main is not None)
        return out

    # --- A7 / A8 -------------------------------------------------------------------------------------
    @staticmethod
    def tr35_a7(paths):
        """paths [(rol, onnx)]: rl/export_c.py --check A7_CHECK_N (gecici klasorde)."""
        from . import export_c as EC
        files, cr = {}, {}
        for role, p in paths:
            d = tempfile.mkdtemp(prefix="openduck_a7_")
            try:
                rep = EC.export(p, d, check_n=A7_CHECK_N)
            except Exception as e:
                files[role] = {"path": p, "status": "error", "reason": repr(e)}
                cr[role] = {"export": crit(None, True, "==")}
                continue
            finally:
                shutil.rmtree(d, ignore_errors=True)
            c, e = rep["check"], rep["esp32s3"]
            cr[role] = {"c_onnx_max_abs_diff": crit(c["max_abs_diff"], CRIT_TR35["A7_atol"], "<="),
                        "esp32s3_estimate_ms": crit(e["estimate_ms"], CRIT_TR35["A7_ms"], "<"),
                        "actor_hidden": crit(rep["hidden"], CRIT_TR35["A7_hidden"], "==")}
            files[role] = {"path": p, "status": "ok", "sizes": rep["sizes"], "activation": rep["activation"],
                           "macs": rep["macs"], "params": rep["params"], "weight_bytes": rep["weight_bytes"],
                           "esp32s3": e, "check": c, "onnx_sha256": rep["onnx_sha256"], "criteria": cr[role]}
        return {"files": files, "criteria": cr, "pass": all_pass(cr)}

    def tr35_mesh_set(self):
        """A8 icin gorsel model: ortamlarla ayni sahne / robot / bosluk (qpos duzeni ayni)."""
        if self._ms is None:
            from .robot_model import build_model
            cc = load_collision_check()
            self._ms = cc.MeshSet(model=build_model(self.scene_path, visual=True, robot=self.robot,
                                                    backlash=bool(self.backlash)))
        return self._ms


@functools.lru_cache(maxsize=1)
def load_collision_check():
    """sim/collision_check.py modulu (surec ici tek kopya: mesh_set onbellegi korunur)."""
    path = os.path.join(C.ROOT, "sim", "collision_check.py")
    spec = importlib.util.spec_from_file_location("openduck_collision_check", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _load(path, role):
    """(politika, hata). "zero" rolun gorevinde sifir aksiyon."""
    if not path:
        return None, None
    try:
        return load_policy(path, task=role if path == "zero" else None), None
    except Exception as e:
        return None, "%s politikasi yuklenemedi: %s" % (role, e)


def criteria_mode(criteria, pol):
    """"xl330_27" | "tr35_b2": --criteria (auto: surec profili ya da bir politika meta'sinin robotu)."""
    if criteria in ("xl330", "xl330_27"):
        return "xl330_27"
    if criteria in ("tr35", TR35):
        return TR35
    metas = [getattr(p, "meta", None) or {} for p in pol.values() if p is not None]
    if C.ROBOT_PROFILE == TR35 or any(m.get("robot") == TR35 for m in metas):
        return TR35
    return "xl330_27"


def _evaluate_tr35(base, pol, paths, seed, flags, bank_n, collision, log, robot, backlash, scene_path, getup_head,
                   t_knee, n_scale, a7_onnx, t0):
    """OpenDuck-TR spec 6 A4-A8 (modul notu). base: politika yukleme sonucu (policies, errors)."""
    robot = robot or TR35
    backlash = True if backlash is None else bool(backlash)
    collision = True if collision is None else bool(collision)
    ev = Evaluator(seed=seed, bank_n=bank_n, collision=collision, log=log, robot=robot, backlash=backlash,
                   scene_path=scene_path, getup_head=getup_head, t_knee=t_knee, n_scale=n_scale, **flags)
    out = {"version": 3, "criteria_mode": TR35, "spec": "docs/workflow/openduck-tr/spec.md v3.1 bolum 6 (A4-A8)",
           "robot": robot, "process_profile": C.ROBOT_PROFILE, "scene": scene_path or C.SCENE_PATH,
           "backlash": backlash, "getup_head": getup_head or C.GETUP_HEAD_DEFAULT, "seed": seed,
           "n_scale": n_scale, "N": {"A4": ev.n_a4, "A5": ev.n_a5, "A6_per_mode": ev.n_a6, "handover": ev.n_ho},
           "dr": dict(flags), "t_knee_nm": ev.t_knee,
           "t_knee_source": "--t-knee" if t_knee is not None else "tau_s(ds32xx) / 3 (G-BENCH'e kadar yer tutucu)",
           "torque_basis": "nominal tau_s (rl/actuators.actuator_table)",
           "derived_source": C.DERIVED_SOURCE, "kappa": C.KAPPA,
           "criteria": CRIT_TR35, "policies": base["policies"], "errors": base["errors"]}
    log("tr35 kabul (spec 6 A4-A8): robot %s, sahne %s, bosluk %s, N %s" % (robot, out["scene"], backlash, out["N"]))

    def section(name, fn, need=(), any_of=()):
        miss = [r for r in need if pol[r] is None]
        if any_of and all(pol[r] is None for r in any_of):
            miss = list(any_of)
        if miss:
            bad = [r for r in miss if out["policies"][r]["status"] == "error"]
            out[name] = {"status": "error" if bad else "skipped",
                         "reason": "gecersiz politika: %s" % ", ".join(bad) if bad else
                         "politika verilmedi: %s" % ", ".join(miss)}
            return
        t = time.time()
        log("%s ..." % name)
        try:
            r = fn()
        except Exception as e:          # bir bolumun cokmesi digerlerini durdurmasin
            out[name] = {"status": "error", "reason": repr(e), "traceback": traceback.format_exc()[-4000:]}
            out["errors"].append("%s: %r" % (name, e))
            log("HATA %s: %r" % (name, e))
            return
        r["status"] = "ok"
        r["seconds"] = round(time.time() - t, 1)
        out[name] = r
        log("%s: %s (%.1f s)" % (name, "GECTI" if r.get("pass") else "gecmedi", r["seconds"]))

    section("A4", lambda: ev.tr35_a4(pol["walk"], pol["main"]), any_of=("walk", "main"))
    section("A5", lambda: ev.tr35_a5(pol["main"]), need=("main",))
    section("A6", lambda: ev.tr35_a6(pol["getup"], pol["main"]), need=("getup",))
    onnx = [(r, paths[r]) for r in ("main", "walk", "getup")
            if pol[r] is not None and str(paths[r] or "").endswith(".onnx")]
    onnx += [("extra_%d" % i, p) for i, p in enumerate(a7_onnx or [])]
    if onnx:
        section("A7", lambda: ev.tr35_a7(onnx))
    else:
        out["A7"] = {"status": "skipped", "reason": "ONNX politika yok (zero / .pt): --a7-onnx ile verilebilir"}
    if collision:
        if ev.traj:
            section("A8", lambda: ev.a8(ms=ev.tr35_mesh_set()))
        else:
            out["A8"] = {"status": "skipped", "reason": "yorunge yok (hicbir rollout calismadi)"}
    else:
        out["A8"] = {"status": "not_run", "reason": "--no-collision: A8 olculmedi ve genel kabule dahil degil"}
    keys = ("A4", "A5", "A6", "A7") + (("A8",) if collision else ())
    out["pass"] = {k: out[k].get("pass") for k in keys}
    out["pass_all"] = all(v is True for v in out["pass"].values())
    out["pass_all_includes"] = list(keys)
    out["seconds"] = round(time.time() - t0, 1)
    return out


def evaluate(main=None, walk=None, getup=None, episodes=64, seed=0, randomize=True, noise=True, push=True,
             head_random=True, bank_n=None, collision=False, log=print, a4_episodes=A4_EPISODES, criteria="auto",
             robot=None, backlash=None, scene_path=None, getup_head=None, t_knee=None, n_scale=1.0, a7_onnx=None):
    """Politika yollari ("zero" dahil) -> sonuc sozlugu (JSON'a yazilabilir). criteria "auto" | "xl330" | "tr35"
    (criteria_mode); tr35 kipinde robot ... a7_onnx kullanilir (collision None: tr35'te acik, xl330'da kapali)."""
    t0 = time.time()
    log = log or (lambda *a, **k: None)
    ev = Evaluator(episodes, seed, randomize, noise, push, head_random, bank_n, collision, log, a4_episodes,
                   robot=robot, backlash=backlash, scene_path=scene_path, getup_head=getup_head)
    custom_model = bool(robot or backlash or scene_path)      # xl330 kipinde istege bagli; varsayilan yol aynen
    out = {"version": 2, "seed": seed, "episodes": episodes, "a4_episodes": a4_episodes,
           "dr": dict(randomize=randomize, noise=noise, push=push, head_random=head_random),
           "criteria": CRIT, "policies": {}, "errors": []}
    pol = {}
    for role, path in (("main", main), ("walk", walk), ("getup", getup)):
        p, err = _load(path, role)
        info = {"path": path}
        if path and p is not None:
            info["task"] = p.task
            if p.task != role:
                have, want = TK.get_task(p.task), TK.get_task(role)
                err = "%s rolune %s politikasi verildi (%s; obs %d / aksiyon %d), %s rolu obs %d / aksiyon %d bekler" % (
                    role, p.task, path, have.num_obs, have.num_actions, role, want.num_obs, want.num_actions)
                p = None
        info["status"] = "missing" if not path else ("error" if err else "ok")
        if err:
            info["error"] = err
            out["errors"].append(err)
            log("HATA: " + err)
        out["policies"][role] = info
        pol[role] = p
    if criteria_mode(criteria, pol) == TR35:
        flags = dict(randomize=randomize, noise=noise, push=push, head_random=head_random)
        return _evaluate_tr35(out, pol, {"main": main, "walk": walk, "getup": getup}, seed, flags, bank_n, collision,
                              log, robot, backlash, scene_path, getup_head, t_knee, n_scale, a7_onnx, t0)

    def section(name, need, fn):
        miss = [r for r in need if pol[r] is None]
        if miss:
            bad = [r for r in miss if out["policies"][r]["status"] == "error"]
            out[name] = {"status": "error" if bad else "skipped",
                         "reason": "gecersiz politika: %s" % ", ".join(bad) if bad else
                         "politika verilmedi: %s" % ", ".join(miss)}
            return
        t = time.time()
        log("%s ..." % name)
        r = fn()
        r["status"] = "ok"
        r["seconds"] = round(time.time() - t, 1)
        out[name] = r
        res = r.get("pass", "x")
        log("%s: %s (%.1f s)" % (name, "olculdu" if res in ("x", None) else ("GECTI" if res else "gecmedi"),
                                 r["seconds"]))

    if pol["walk"] is not None and pol["main"] is None:
        # A1 yalniz walk taban cizgisiyle (main gecersiz/yok): taban yine olculur
        section("A1", ["walk"], lambda: ev.a1(None, pol["walk"]))
        out["A1"]["status"] = out["policies"]["main"]["status"] if main else "skipped"
        if main:
            out["A1"]["reason"] = "gecersiz politika: main"
    else:
        section("A1", ["main", "walk"], lambda: ev.a1(pol["main"], pol["walk"]))
    section("A2", ["main"], lambda: ev.a2(pol["main"]))
    section("A3", ["main"], lambda: ev.a3(pol["main"]))
    section("A4", ["getup"], lambda: ev.a4(pol["getup"]))
    if collision:
        if ev.traj:
            section("A8", [], lambda: ev.a8(ms=ev.tr35_mesh_set() if custom_model else None))
            for k, g in out["A8"]["groups"].items():
                w = g["worst"]
                log("  A8 %-12s %s %d kare, en kotu %.3f mm^3%s" % (
                    k, "      " if g["in_verdict"] else "(taban)", g["frames"], g["max_pen_mm3"],
                    " (%s, ortam %d, t %.1f s)" % (" ~ ".join(w["parts"]), w["env"], w["t_s"]) if w else ""))
        else:
            out["A8"] = {"status": "skipped", "reason": "yorunge yok (hicbir rollout calismadi)"}
    else:
        out["A8"] = {"status": "not_run",
                     "reason": "--collision verilmedi: A8 olculmedi ve genel kabule (pass / pass_all) dahil degil"}
    keys = ("A1", "A2", "A3", "A4") + (("A8",) if collision else ())
    out["pass"] = {k: out[k].get("pass") for k in keys if k in out}
    out["pass_all"] = bool(out["pass"]) and all(v is True for v in out["pass"].values())
    out["pass_all_includes"] = list(out["pass"])
    if not collision:
        out["pass_note"] = "A8 (oz-carpisma) dahil degil: --collision verilmedi"
    out["seconds"] = round(time.time() - t0, 1)
    return out


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="OpenDuck kabul olcumleri (spec 6 A1-A4, A8) -> JSON")
    p.add_argument("--main-policy", default=None, help="main politikasi (.onnx/.pt/'zero')")
    p.add_argument("--walk-policy", default=None, help="walk taban cizgisi (or. runs/walk-mjx-v2/policy.onnx)")
    p.add_argument("--getup-policy", default=None, help="getup politikasi")
    p.add_argument("--episodes", type=int, default=64, help="A1-A3 olcum gruplarinda ortam (bolum) sayisi")
    p.add_argument("--a4-episodes", type=int, default=A4_EPISODES,
                   help="A4: bolum turu (fallen / midfall) basina bolum sayisi (varsayilan %d)" % A4_EPISODES)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", default=None, help="JSON cikti (varsayilan: stdout)")
    p.add_argument("--no-randomize", action="store_true", help="alan rastgelelestirmeyi kapat")
    p.add_argument("--no-noise", action="store_true", help="gozlem gurultusunu kapat")
    p.add_argument("--no-push", action="store_true", help="itmeleri kapat")
    p.add_argument("--no-head", action="store_true", help="rastgele bas hareketini kapat")
    p.add_argument("--bank-n", type=int, default=None, help="poz bankasi boyutu (varsayilan config; hizli deneme)")
    p.add_argument("--collision", action="store_true",
                   help="A8: tum rollout gruplarinda (ilk 8 ortam, 0.2 s'de bir) mesh oz-carpisma denetimi")
    g = p.add_argument_group("OpenDuck-TR (spec v3.1 bolum 6, A4-A8)")
    g.add_argument("--criteria", choices=("auto", "xl330", "tr35"), default="auto",
                   help="auto: config.ROBOT_PROFILE ya da politika meta'sinin robotu tr35_b2 ise tr35")
    g.add_argument("--scene", default=None, help="sahne MJCF'si (varsayilan config.SCENE_PATH)")
    g.add_argument("--robot", choices=("xl330_27", TR35), default=None, help="ortam robotu (tr35 kipi: tr35_b2)")
    g.add_argument("--backlash", dest="backlash", action="store_true", default=None,
                   help="DS bosluk eklemi (tr35 kipinde varsayilan acik)")
    g.add_argument("--no-backlash", dest="backlash", action="store_false")
    g.add_argument("--getup-head", choices=("free", "fixed"), default=None,
                   help="getup kafa modu (varsayilan ortaminki, config.GETUP_HEAD_DEFAULT)")
    g.add_argument("--t-knee", type=float, default=None,
                   help="T_diz, N.m (varsayilan tau_s(ds32xx) / 3: G-BENCH'e kadar yer tutucu)")
    g.add_argument("--n-scale", type=float, default=1.0, help="spec N carpani (duman kosulari)")
    g.add_argument("--a7-onnx", action="append", default=None, help="A7'ye eklenecek ONNX (tekrarlanabilir)")
    g.add_argument("--no-collision", action="store_true", help="tr35: A8'i atla (varsayilan olculur)")
    return p.parse_args(argv)


def main(argv=None):
    a = parse_args(argv)
    if not (a.main_policy or a.walk_policy or a.getup_policy):
        print("en az bir politika verin (--main-policy / --walk-policy / --getup-policy)", file=sys.stderr)
        return 2
    collision = True if a.collision else (False if a.no_collision else None)
    out = evaluate(a.main_policy, a.walk_policy, a.getup_policy, a.episodes, a.seed, not a.no_randomize,
                   not a.no_noise, not a.no_push, not a.no_head, a.bank_n, collision,
                   log=lambda *x: print(*x, file=sys.stderr, flush=True), a4_episodes=a.a4_episodes,
                   criteria=a.criteria, robot=a.robot, backlash=a.backlash, scene_path=a.scene,
                   getup_head=a.getup_head, t_knee=a.t_knee, n_scale=a.n_scale, a7_onnx=a.a7_onnx)
    txt = json.dumps(out, indent=1)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
        print("yazildi: %s" % a.out, file=sys.stderr)
    else:
        print(txt)
    if out["errors"]:
        print("HATA: " + "; ".join(out["errors"]), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
