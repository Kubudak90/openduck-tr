# -*- coding: utf-8 -*-
"""
Kabul olcumleri (docs/workflow/sit-stand-getup/spec.md 6, A1-A4 ve istege bagli A8): ONNX politikalarini
CPU ortaminda (rl/env_cpu.py, surec ici) DR acik degerlendirir ve sonuclari JSON'a yazar.

  python3 -m rl.eval_tasks --main-policy runs/M/policy.onnx --walk-policy runs/walk-mjx-v2/policy.onnx
                           --getup-policy runs/G/policy.onnx [--episodes 64] [--a4-episodes 512] [--seed 0]
                           [--out eval.json] [--no-randomize] [--no-noise] [--no-push] [--no-head] [--bank-n N]
                           [--collision]

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
"""
import argparse
import functools
import importlib.util
import json
import math
import os
import sys
import time

import mujoco
import numpy as np

from . import config as C
from . import pose_bank as PB
from . import task_math as T
from . import tasks as TK
from .env_cpu import VecEnv
from .play import load_policy

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
    def __init__(self, episodes=64, seed=0, randomize=True, noise=True, push=True, head_random=True, bank_n=None,
                 collision=False, log=print, a4_episodes=A4_EPISODES):
        self.N = int(episodes)
        self.N4 = int(a4_episodes)
        self.seed = int(seed)
        self.flags = dict(randomize=randomize, noise=noise, push=push, head_random=head_random)
        self.bank_n = bank_n
        self.collision = collision
        self.log = log or (lambda *a, **k: None)
        self.traj = {}

    def _env(self, task, n, **kw):
        kw.setdefault("seed", self.seed)
        return VecEnv(n, num_workers=0, task=task, episode_details=True, bank_n=self.bank_n, **self.flags, **kw)

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
    def height(self, policy, walking):
        lo = C.H_MIN_WALK if walking else C.H_MIN_STAND
        hs = np.linspace(lo, C.H_HOME, self.N)
        vx = 0.15 if walking else 0.0
        cmd = np.c_[np.full(self.N, vx), np.zeros((self.N, 2)), hs, np.zeros(self.N)]
        t0 = (C.H_HOME - C.H_MIN_STAND) / C.H_RATE + 1.5            # h_ref oturur + 1.5 s
        n_steps = int(round((t0 + 4.0) / DT))
        env = self._env("main", self.N, fixed_command=cmd, reset_probs={"home": 1.0}, max_episode_steps=n_steps + 1)
        e2, ne, fell = np.zeros(self.N), np.zeros(self.N), np.zeros(self.N, bool)
        vxs = np.zeros(self.N)
        rec = self._rec(self.N)
        try:
            obs, _ = env.reset()
            for k in range(n_steps):
                obs, _, _, done, info = env.step(policy(obs))
                rec.step(env, k, done)
                fell |= done
                if (k + 1) * DT >= t0 - 1e-9:
                    s = env.task_state()
                    e = s["trunk_z"] - T.z_ref(np, s["h_ref"], s["alpha"])
                    ok = ~fell
                    e2[ok] += e[ok] ** 2
                    vxs[ok] += s["linvel"][ok, 0]               # govde cercevesi ileri hiz
                    ne[ok] += 1
        finally:
            env.close()
        self._keep("A2_walking" if walking else "A2_standing", rec)
        rmse = np.where(ne > 0, np.sqrt(e2 / np.maximum(ne, 1)), np.nan)
        vxm = np.where(ne > 0, vxs / np.maximum(ne, 1), np.nan)
        vx_min = CRIT["A2_vx_frac"] * vx
        with np.errstate(invalid="ignore"):
            moving = bool(np.all(vxm >= vx_min - 1e-12)) if walking else True
        return {"h": hs.tolist(), "vx": vx, "rmse": [_num(x) for x in rmse], "vx_mean": [_num(x) for x in vxm],
                "vx_min_required": vx_min if walking else None, "fell": fell.tolist(),
                "max_rmse": _num(np.nanmax(rmse)) if np.any(ne > 0) else None,
                "min_vx_mean": _num(np.nanmin(vxm)) if np.any(ne > 0) else None, "moving": moving,
                "pass": bool(np.all(~fell) and np.all(rmse < CRIT["A2_rmse"]) and moving)}

    def a2(self, main):
        st, wk = self.height(main, False), self.height(main, True)
        return {"standing": st, "walking": wk, "pass": bool(st["pass"] and wk["pass"])}

    # --- A3 -------------------------------------------------------------------------
    def sit_stand(self, policy, seated):
        cmd_sit = np.array([0.0, 0.0, 0.0, C.H_HOME, 1.0])
        cmd_up = np.array([0.0, 0.0, 0.0, C.H_HOME, 0.0])
        if seated:
            t_sit, t_up = 0.0, A3_SEATED[0]
            total = sum(A3_SEATED)
            first = cmd_sit
        else:
            t_sit, t_up = A3_HOME[0], A3_HOME[0] + A3_HOME[1]
            total = sum(A3_HOME)
            first = cmd_up
        n_steps = int(round(total / DT))
        env = self._env("main", self.N, fixed_command=first, max_episode_steps=n_steps + 1,
                        reset_probs={"seated": 1.0} if seated else {"home": 1.0})
        N = self.N
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
                g = T.projected_gravity(np, s["quat"])[:, 2]
                up = g < C.MAIN["seat_gz"]
                seated_ok = (s["command"][:, 4] > 0.5) & (T.is_seated(np, s["alpha"]) > 0.5) & (s["trunk_floor"] > 0.5) & up
                stood = (s["command"][:, 4] < 0.5) & (s["trunk_z"] > C.MAIN["standup_z"]) & np.all(s["contact"] > 0.5, 1) & up
                m = live & seated_ok & np.isnan(t_seated) & (t1 > t_sit)
                t_seated[m] = t1 - t_sit
                m = live & stood & np.isnan(t_stand) & (t1 > t_up)
                t_stand[m] = t1 - t_up
                tau = np.max(np.abs(s["torque"]), axis=1)
                sat += int(np.sum(live & (tau > SAT)))
                tot += int(np.sum(live))
        finally:
            env.close()
        self._keep("A3_seated" if seated else "A3_home", rec)
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
    def a8(self):
        t0 = time.time()
        cc = load_collision_check()
        ms = cc.mesh_set()
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


def evaluate(main=None, walk=None, getup=None, episodes=64, seed=0, randomize=True, noise=True, push=True,
             head_random=True, bank_n=None, collision=False, log=print, a4_episodes=A4_EPISODES):
    """Politika yollari ("zero" dahil) -> sonuc sozlugu (JSON'a yazilabilir)."""
    t0 = time.time()
    log = log or (lambda *a, **k: None)
    ev = Evaluator(episodes, seed, randomize, noise, push, head_random, bank_n, collision, log, a4_episodes)
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
            section("A8", [], ev.a8)
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
    return p.parse_args(argv)


def main(argv=None):
    a = parse_args(argv)
    if not (a.main_policy or a.walk_policy or a.getup_policy):
        print("en az bir politika verin (--main-policy / --walk-policy / --getup-policy)", file=sys.stderr)
        return 2
    out = evaluate(a.main_policy, a.walk_policy, a.getup_policy, a.episodes, a.seed, not a.no_randomize,
                   not a.no_noise, not a.no_push, not a.no_head, a.bank_n, a.collision,
                   log=lambda *x: print(*x, file=sys.stderr, flush=True), a4_episodes=a.a4_episodes)
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
