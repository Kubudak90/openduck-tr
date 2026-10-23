# -*- coding: utf-8 -*-
"""
CPU ortam hiz olcumu (env-steps/s).

  python3 -m rl.bench_env                      # 512 ortam, 0..4 isci
  python3 -m rl.bench_env --num-envs 512 --workers 4 --seconds 5

Aksiyonlar N(0, std) (PPO baslangic gurultusu std=1.0): robotlar sik duser, sifirlama + DR maliyeti dahildir.
"""
import argparse
import os
import time

import numpy as np

from .env_cpu import VecEnv


def bench(num_envs, num_workers, seconds, warmup=5, std=1.0, seed=0, **flags):
    t0 = time.perf_counter()
    env = VecEnv(num_envs, num_workers=num_workers, seed=seed, **flags)
    try:
        t_init = time.perf_counter() - t0
        env.reset()
        rng = np.random.default_rng(seed)
        acts = (std * rng.standard_normal((16, num_envs, env.num_actions))).astype(np.float32)
        for k in range(warmup):
            env.step(acts[k % 16])
        steps, resets = 0, 0
        t = time.perf_counter()
        while True:
            _, _, _, done, _ = env.step(acts[steps % 16])
            resets += int(done.sum())
            steps += 1
            dt = time.perf_counter() - t
            if dt >= seconds and steps >= 3:
                break
    finally:
        env.close()
    return {"workers": num_workers, "init_s": t_init, "steps": steps, "seconds": dt,
            "env_steps_per_s": steps * num_envs / dt, "resets_per_env_step": resets / float(steps * num_envs)}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--num-envs", type=int, default=512)
    p.add_argument("--workers", default="0,1,2,3,4", help="virgulle ayrilmis isci sayilari")
    p.add_argument("--seconds", type=float, default=3.0, help="her ayar icin olcum suresi")
    p.add_argument("--std", type=float, default=1.0, help="aksiyon gurultusu std")
    p.add_argument("--no-dr", action="store_true", help="DR/itme/bas/gurultu kapali")
    a = p.parse_args()
    flags = {}
    if a.no_dr:
        flags = dict(randomize=False, noise=False, head_random=False, push=False)
    print("cpu_count=%s  num_envs=%d  load_avg=%s" % (os.cpu_count(), a.num_envs, os.getloadavg()))
    for w in [int(x) for x in a.workers.split(",")]:
        r = bench(a.num_envs, w, a.seconds, std=a.std, **flags)
        print("workers=%d  %8.0f env-steps/s  (%d steps in %.2fs, init %.2fs, resets/env-step %.4f)" % (
            w, r["env_steps_per_s"], r["steps"], r["seconds"], r["init_s"], r["resets_per_env_step"]), flush=True)


if __name__ == "__main__":
    main()
