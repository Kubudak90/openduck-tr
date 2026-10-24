# -*- coding: utf-8 -*-
"""
CPU PPO egitimi (MuJoCo C ortami rl.env_cpu.VecEnv + rl.ppo).

  python3 -m rl.train --out runs/NAME [--task walk|main|getup] [--num-envs N] [--workers W] [--max-steps 5e7]
                      [--seed S] [--resume PATH] [--no-randomize] [--getup-head free|fixed]

--task: rl/tasks.py gorevi (varsayilan walk, davranis degismez). main/getup icin TaskSpec varsayilanlari
(gamma, ag boyutlari) kullanilir; getup'ta ortamin mufredat ilerlemesi her iterasyonda adim/hedef olarak
ayarlanir (VecEnv.set_progress). Not: CPU'da main/getup egitimi yalniz duman/parite icindir (spec 2).
--getup-head (getup): "fixed" ortama getup_head="fixed" olarak verilir (kafa toplanmis, aksiyon disi; spec 3.4);
verilmezse / "free" ise ortamin varsayilani. tr35_b2 surecinde (OPENDUCK_ROBOT) policy_meta ag/aktuator/sensor
alanlarini tasir ve her ONNX disa aktarmasindan sonra rl/export_c.py <run>/c/ yazar (hata olumcul degil).

runs/NAME/ altina yazilanlar:
  log.jsonl          her iterasyonda bir JSON satiri
  model_<iter>.pt    periyodik checkpoint (model + normalizer + optimizer + lr + adim sayisi)
  latest.pt          son checkpoint (ayni icerik)
  policy.onnx, policy_meta.json   her checkpoint'ta ve sonda (rl/export_onnx.py)
  c/                 yalniz tr35_b2: policy_weights.h, policy.h, policy.c, obs_layout.h, export_c.json
  config.json        egitimin basindaki rl/config.py sabitleri ve komut satiri (devamda config.<zaman>.json)

SIGINT/SIGTERM: o anki toplama yarida birakilir, checkpoint + ONNX yazilir, cikilir.
Ikinci sinyal aninda keser.
"""
import argparse
import collections
import json
import math
import os
import signal
import sys
import time

import numpy as np
import torch

from . import config as C
from . import tasks as TK
from .export_onnx import export_policy, policy_meta
from .ppo import PPO, ActorCritic, set_torch_threads


def _f32(x):
    return torch.from_numpy(np.ascontiguousarray(x, dtype=np.float32))


def _num(x):
    x = float(x)
    return x if math.isfinite(x) else None


def effective_cpus():
    """Kullanilabilir cekirdek sayisi: affinity ve cgroup CPU kotasinin kucugu. Konteynerde (or. RunPod)
    os.cpu_count() makinenin tum cekirdeklerini verir; o kadar isci/is parcacigi asiri yukleme yapar."""
    n = [len(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else (os.cpu_count() or 1)]
    try:
        with open("/sys/fs/cgroup/cpu.max") as f:
            q, p = f.read().split()[:2]
        if q != "max":
            n.append(int(q) // int(p))
    except (OSError, ValueError):
        pass
    try:
        with open("/sys/fs/cgroup/cpu/cpu.cfs_quota_us") as f:
            q = int(f.read())
        with open("/sys/fs/cgroup/cpu/cpu.cfs_period_us") as f:
            p = int(f.read())
        if q > 0:
            n.append(q // p)
    except (OSError, ValueError):
        pass
    return max(1, min(n))


class Runner(object):
    """PPO egitim dongusu; VecEnv arayuzunu (rl/TASK.md) uygulayan her ortamla calisir."""

    def __init__(self, env, out_dir, num_actions=None, cfg=None, seed=0, save_interval=50, threads=1,
                 update_threads=2, resume=None, meta_fn=None, export=True, print_fn=print, run_info=None,
                 post_export=None):
        self.env = env
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.save_interval = save_interval
        self.threads, self.update_threads = threads, update_threads
        self.meta_fn = meta_fn
        self.export = export
        self.post_export = post_export      # basarili ONNX disa aktarmasindan sonra f(onnx_yolu) (tr35: export_c)
        self.print = print_fn or (lambda *a, **k: None)
        self.run_info = run_info or {}
        self.stop_requested = False
        torch.manual_seed(seed)
        np.random.seed(seed)
        set_torch_threads(threads)

        ckpt = torch.load(resume, map_location="cpu", weights_only=False) if resume else None
        obs, critic_obs = env.reset()
        self.obs, self.critic_obs = _f32(obs), _f32(critic_obs)
        self.num_envs = self.obs.shape[0]
        if ckpt is not None:
            ac = ActorCritic.from_arch(ckpt["arch"])
        else:
            if num_actions is None:
                num_actions = getattr(env, "num_actions", C.NUM_ACTIONS)
            P = dict(C.PPO)
            P.update(cfg or {})
            ac = ActorCritic(self.obs.shape[1], self.critic_obs.shape[1], num_actions,
                             P["actor_hidden"], P["critic_hidden"], P["activation"], P["init_noise_std"],
                             P["obs_normalization"])
        a = ac.arch
        if self.obs.shape[1] != a["num_obs"] or self.critic_obs.shape[1] != a["num_critic_obs"]:
            raise ValueError("gozlem boyutu modele uymuyor: env %s/%s, model %d/%d" % (
                self.obs.shape[1], self.critic_obs.shape[1], a["num_obs"], a["num_critic_obs"]))
        self.alg = PPO(ac, self.num_envs, cfg=cfg)
        self.iter, self.steps, self.wall_offset = 0, 0, 0.0
        if ckpt is not None:
            self.alg.load_state_dict(ckpt)
            self.iter, self.steps = int(ckpt["iter"]), int(ckpt["steps"])
            self.wall_offset = float(ckpt.get("wall_time", 0.0))
            if "torch_rng" in ckpt:
                torch.set_rng_state(ckpt["torch_rng"])
            self.print("devam: %s (iter %d, %d adim, lr %.2e)" % (resume, self.iter, self.steps, self.alg.lr))
        self.last_saved_iter = self.iter if ckpt is not None else -1
        self.log_path = os.path.join(out_dir, "log.jsonl")
        self.t_start = time.time()

    @property
    def batch_size(self):
        return self.num_envs * self.alg.num_steps

    def request_stop(self):
        self.stop_requested = True

    # --- dongu -------------------------------------------------------------------
    def learn(self, max_steps):
        N, T = self.num_envs, self.alg.num_steps
        ep_ret = np.zeros(N, np.float64)
        ep_len = np.zeros(N, np.int64)
        ret_buf = collections.deque(maxlen=100)
        len_buf = collections.deque(maxlen=100)
        self.t_start = time.time()
        set_progress = getattr(self.env, "set_progress", None)      # getup mufredati (walk/main: etkisiz)
        try:
            while self.steps < max_steps and not self.stop_requested:
                t0 = time.time()
                if set_progress is not None:
                    set_progress(min(1.0, self.steps / float(max_steps)))
                torch.set_num_threads(self.threads)
                ep_acc = {}
                rew_sum, n_eps, n_falls = 0.0, 0, 0
                self.alg.ac.train()
                for _ in range(T):
                    actions = self.alg.act(self.obs, self.critic_obs)
                    obs, critic_obs, rew, done, info = self.env.step(actions.numpy())
                    rew = np.asarray(rew, np.float32)
                    done = np.asarray(done, bool)
                    time_outs = np.asarray(info.get("time_outs", np.zeros(N, bool)), bool)
                    self.alg.process_env_step(rew, done, time_outs)
                    self.obs, self.critic_obs = _f32(obs), _f32(critic_obs)

                    rew_sum += float(rew.sum())
                    ep_ret += rew
                    ep_len += 1
                    if done.any():
                        ids = np.flatnonzero(done)
                        ret_buf.extend(ep_ret[ids].tolist())
                        len_buf.extend(ep_len[ids].tolist())
                        ep_ret[ids] = 0.0
                        ep_len[ids] = 0
                        n_eps += len(ids)
                        n_falls += int(np.sum(done & ~time_outs))
                        for k, v in (info.get("episode") or {}).items():
                            s = ep_acc.setdefault(k, [0.0, 0])
                            s[0] += float(v) * len(ids)
                            s[1] += len(ids)
                    if self.stop_requested:
                        break
                if self.alg.storage.step < T:
                    self.alg.storage.clear()
                    break
                t1 = time.time()
                torch.set_num_threads(self.update_threads)
                self.alg.compute_returns(self.critic_obs)
                stats = self.alg.update()
                t2 = time.time()
                self.iter += 1
                self.steps += N * T

                rec = {
                    "iter": self.iter, "steps": self.steps,
                    "fps": round(N * T / (t2 - t0), 1),
                    "wall_time": round(self.wall_offset + t2 - self.t_start, 2),
                    "collect_time": round(t1 - t0, 3), "learn_time": round(t2 - t1, 3),
                    "reward_per_step": _num(rew_sum / (N * T)),
                    "ep_len": _num(np.mean(len_buf)) if len_buf else None,
                    "ep_return": _num(np.mean(ret_buf)) if ret_buf else None,
                    "n_episodes": n_eps, "n_falls": n_falls,
                    "episode": {k: _num(s[0] / s[1]) for k, s in ep_acc.items() if s[1] > 0},
                }
                rec.update({k: _num(v) for k, v in stats.items()})
                self._log(rec)
                if not all(math.isfinite(float(v)) for v in stats.values()):
                    raise FloatingPointError("PPO istatistiklerinde NaN/inf: %r" % stats)
                if self.save_interval and self.iter % self.save_interval == 0:
                    self.save()
        except Exception:
            if not self.stop_requested:
                raise
            self.print("durdurma sirasinda ortam hatasi yok sayildi", file=sys.stderr)
        finally:
            if self.iter != self.last_saved_iter:
                self.save()
        return self.iter

    def _log(self, rec):
        with open(self.log_path, "a") as f:
            f.write(json.dumps(rec) + "\n")
        ep = rec["episode"]
        g = lambda v, fmt: (fmt % v) if v is not None else "-"
        self.print("it %5d %7.2fM %6.0f fps | r/step %s len %s ret %s lin %s ang %s falls %3d | "
                   "kl %.4f lr %.1e (ort %.1e) std %.3f vl %.3g | %.1f+%.1fs" % (
                       rec["iter"], rec["steps"] / 1e6, rec["fps"], g(rec["reward_per_step"], "%+.4f"),
                       g(rec["ep_len"], "%5.0f"), g(rec["ep_return"], "%+6.2f"),
                       g(ep.get("rew_tracking_lin_vel"), "%5.2f"), g(ep.get("rew_tracking_ang_vel"), "%5.2f"),
                       rec["n_falls"], rec["kl"] or 0.0, rec["lr"], rec.get("lr_mean") or 0.0, rec["action_std"],
                       rec["value_loss"] or 0.0, rec["collect_time"], rec["learn_time"]), flush=True)

    # --- kayit -------------------------------------------------------------------
    def save(self):
        if not all(torch.isfinite(p).all() for p in self.alg.ac.parameters()):
            self.print("UYARI: model parametrelerinde NaN/inf; checkpoint yazilmadi", file=sys.stderr)
            return
        ckpt = self.alg.state_dict()
        ckpt.update(iter=self.iter, steps=self.steps, wall_time=self.wall_offset + time.time() - self.t_start,
                    backend="cpu_torch", torch_rng=torch.get_rng_state(), run_info=self.run_info)
        task = getattr(self.env, "task", "walk")
        if task != "walk":
            ckpt["task"] = task          # export_onnx.main meta'yi buna gore yazar
        path = os.path.join(self.out_dir, "model_%d.pt" % self.iter)
        tmp = path + ".tmp"
        torch.save(ckpt, tmp)
        os.replace(tmp, path)
        latest = os.path.join(self.out_dir, "latest.pt")
        torch.save(ckpt, latest + ".tmp")
        os.replace(latest + ".tmp", latest)
        self.last_saved_iter = self.iter
        msg = "checkpoint: %s" % path
        if self.export:
            try:
                meta = self.meta_fn(self.steps) if self.meta_fn else None
                onnx_path = os.path.join(self.out_dir, "policy.onnx")
                err = export_policy(self.alg.ac, onnx_path, meta=meta)
                msg += " + policy.onnx (%s)" % ("dogrulanmadi" if err is None else "onnx/torch %.1e" % err)
                if self.post_export is not None:
                    self.post_export(onnx_path)
            except Exception as e:      # egitim disa aktarma yuzunden durmasin; checkpoint yazildi
                msg += " (UYARI: ONNX disa aktarma basarisiz: %r)" % (e,)
        self.print(msg, flush=True)


def _config_snapshot():
    return {k: getattr(C, k) for k in dir(C) if k.isupper() and not k.startswith("_")}


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="OpenDuck yurume politikasi: CPU PPO egitimi")
    p.add_argument("--out", required=True, help="cikti klasoru, or. runs/NAME")
    p.add_argument("--task", choices=TK.TASK_NAMES, default="walk", help="gorev (rl/tasks.py), varsayilan walk")
    p.add_argument("--num-envs", type=int, default=C.PPO["num_envs"])
    p.add_argument("--workers", type=int, default=None,
                   help="ortam isci surecleri (varsayilan: kullanilabilir cekirdek, cgroup kotasi dahil)")
    p.add_argument("--max-steps", type=float, default=5e7, help="toplam ortam adimi (devamda dahil)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--resume", default=None, help="checkpoint (.pt) yolu")
    p.add_argument("--no-randomize", action="store_true", help="alan rastgelelestirmeyi kapat")
    p.add_argument("--no-noise", action="store_true", help="gozlem gurultusunu kapat")
    p.add_argument("--no-push", action="store_true", help="itmeleri kapat")
    p.add_argument("--no-head", action="store_true", help="rastgele bas hareketini kapat")
    p.add_argument("--save-interval", type=int, default=50, help="kac iterasyonda bir checkpoint")
    p.add_argument("--threads", type=int, default=1, help="toplama sirasinda torch is parcacigi")
    p.add_argument("--update-threads", type=int, default=None,
                   help="PPO guncellemesinde torch is parcacigi (varsayilan: cekirdeklerin yarisi, 1..16; "
                        "isciler bu sirada bekler)")
    # SUPPRESS: verilmezse args'ta yok (config.json / checkpoint run_info bugunkuyle ayni)
    p.add_argument("--getup-head", choices=("free", "fixed"), default=argparse.SUPPRESS,
                   help="getup: kafa serbest (politika) ya da sabit/toplanmis (HEAD_TUCK_DEG; spec 3.4); "
                        "varsayilan ortaminki (config.GETUP_HEAD_DEFAULT)")
    a = p.parse_args(argv)
    cpus = effective_cpus()
    if a.workers is None:
        a.workers = cpus
    if a.update_threads is None:
        a.update_threads = max(1, min(16, cpus // 2))
    return a


def env_task_kwargs(task, getup_head=None):
    """Ortama gorev ek argumanlari: getup_head yalniz getup'ta ve "free" degilse verilir (varsayilan ortaminki)."""
    if TK.get_task(task).name == "getup" and getup_head not in (None, "free"):
        return {"getup_head": getup_head}
    return {}


def task_ppo_cfg(task):
    """Gorevin PPO varsayilanlari (TaskSpec gamma / ag boyutlari; walk: None = C.PPO aynen)."""
    t = TK.get_task(task)
    cfg = {}
    if t.gamma is not None:
        cfg["gamma"] = t.gamma
    if t.actor_hidden is not None:
        cfg["actor_hidden"] = list(t.actor_hidden)
    if t.critic_hidden is not None:
        cfg["critic_hidden"] = list(t.critic_hidden)
    return cfg or None


def main(argv=None):
    args = parse_args(argv)
    from .env_cpu import VecEnv
    from .robot_model import RobotIndex, build_model

    os.makedirs(args.out, exist_ok=True)
    spec = TK.get_task(args.task)
    idx = RobotIndex(build_model(variant=spec.model_variant), list(spec.action_joints))
    cfg_path = os.path.join(args.out, "config.json")
    if os.path.exists(cfg_path):
        cfg_path = os.path.join(args.out, "config.%d.json" % int(time.time()))
    with open(cfg_path, "w") as f:
        json.dump({"args": vars(args), "argv": sys.argv, "config": _config_snapshot()}, f, indent=1, default=str)

    # isci surecleri terminaldeki Ctrl-C'yi gormesin (yok sayilan sinyal fork/exec'te miras kalir);
    # durdurmayi ana surec yonetir
    old_int = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        env = VecEnv(args.num_envs, num_workers=args.workers, seed=args.seed, randomize=not args.no_randomize,
                     noise=not args.no_noise, head_random=not args.no_head, push=not args.no_push, task=spec.name,
                     **env_task_kwargs(spec.name, getattr(args, "getup_head", None)))
    finally:
        signal.signal(signal.SIGINT, old_int)
    net = {}
    post = None
    if C.ROBOT_PROFILE == "tr35_b2":       # tr35: her ONNX'ten sonra C dizileri <run>/c (rl/export_c.py)
        from . import export_c as EC
        post = lambda p: EC.safe_export(p, os.path.join(args.out, "c"))
    try:
        runner = Runner(env, args.out, cfg=task_ppo_cfg(spec), seed=args.seed, save_interval=args.save_interval,
                        threads=args.threads, update_threads=args.update_threads, resume=args.resume,
                        meta_fn=lambda steps: policy_meta(steps, "cpu_torch", idx, spec.name, network=net.get("v"),
                                                          getup_head=getattr(args, "getup_head", None)),
                        run_info=vars(args), post_export=post)
        net["v"] = runner.alg.ac.network()

        def on_signal(signum, frame):
            runner.request_stop()
            signal.signal(signal.SIGINT, signal.default_int_handler)
            signal.signal(signal.SIGTERM, signal.SIG_DFL)
            print("\nsinyal %d: iterasyon kesiliyor, checkpoint yaziliyor..." % signum, flush=True)

        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)
        print("egitim: %d ortam x %d adim = %d ornek/iterasyon, hedef %.3g adim, cikti %s" % (
            runner.num_envs, runner.alg.num_steps, runner.batch_size, args.max_steps, args.out), flush=True)
        runner.learn(args.max_steps)
    finally:
        try:
            env.close()
        except Exception as e:
            print("env.close() hatasi: %r" % (e,), file=sys.stderr)
    print("bitti: %s" % os.path.join(args.out, "policy.onnx"))


if __name__ == "__main__":
    main()
