# -*- coding: utf-8 -*-
"""
tr35_b2 duman egitimleri (OpenDuck-TR P5a): walk, main ve getup icin kisa CPU-JAX MJX + Brax PPO egitimi, tr35_b2
surecinde (OPENDUCK_ROBOT/SCENE/DERIVED, OPENDUCK_BANK_N=64). Her kosu: cikis 0, log.jsonl'daki tum metrikler sonlu,
policy.onnx + policy_meta.json yazilmis (meta boyutlari gorevle ayni). getup ayrica getup_head="fixed" ile: egiticide
--getup-head bayragi varsa egitim, yoksa dogrudan ortam kontrolu (sifirlama + adimlar: bas hedefi toplanmis, bas
aksiyonlari 0, sonlu).

  <mjx py> tools/tr35_smoke.py --out <scratch>/smoke [--scene <scene.xml> | (stub derlenir)] [--derived <json>]
      [--tasks walk,main,getup] [--lock <dizin>] [--num-envs 32 --num-timesteps 2e4 --num-evals 2 --num-eval-envs 8
      --unroll-length 10] [--bank-n 64]

--derived: varsayilan rl/derived/tr35_b2.json (derive paketi); birlesmeden once test-yalniz bootstrap dosyasi verilir
(rl/tests/tr35_scene.bootstrap_derived). --lock: her kosudan once mkdir kilidi alinir, sonra birakilir (bir kilitte bir
kosu). Ozet: <out>/tr35_smoke.json; basarisizlikta cikis 1.
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TASK_DIMS = {"walk": (49, 10), "main": (53, 10), "getup": (48, 14)}

_HEAD_CHECK = r"""
import json, sys
sys.path.insert(0, %(root)r)
import numpy as np, jax, jax.numpy as jnp
import rl.config as C
from rl.env_mjx import OpenDuckEnv
e = OpenDuckEnv(task="getup", getup_head="fixed", solver_iterations=16)
st = jax.jit(jax.vmap(e.reset))(jax.random.split(jax.random.PRNGKey(0), 8))
step = jax.jit(jax.vmap(e.step))
rng = np.random.default_rng(0)
for k in range(20):
    st = step(st, jnp.asarray(rng.uniform(-2, 2, (8, 14)), jnp.float32))
res = e._act["pwm_res"][e.idx.head_act_id]
tuck = np.round(np.deg2rad([C.HEAD_TUCK_DEG[j] for j in C.HEAD_JOINTS]) / res) * res
hc = np.asarray(st.info["tr"]["ctrl_prev"])[:, e.idx.head_act_id]
la = np.asarray(st.info["a_prev"])
ok = bool(np.allclose(hc, tuck, atol=1e-6) and np.all(la[:, 10:14] == 0) and np.isfinite(np.asarray(st.obs["state"])).all()
          and np.isfinite(np.asarray(st.reward)).all())
print(json.dumps({"ok": ok, "mode": e.getup_head, "head_ctrl_err": float(np.abs(hc - tuck).max()),
                  "head_action_max": float(np.abs(la[:, 10:14]).max()), "reward_mean": float(np.asarray(st.reward).mean())}))
"""


def parse(argv=None):
    p = argparse.ArgumentParser(description="tr35_b2 duman egitimleri (walk, main, getup)")
    p.add_argument("--out", required=True, help="cikti dizini (karalama)")
    p.add_argument("--scene", default=os.environ.get("OPENDUCK_SCENE"), help="tr35 scene.xml (yoksa stub derlenir)")
    p.add_argument("--derived", default=os.environ.get("OPENDUCK_DERIVED"), help="turetilmis JSON")
    p.add_argument("--tasks", default="walk,main,getup")
    p.add_argument("--python", default=sys.executable, help="egitici yorumlayicisi (MJX venv)")
    p.add_argument("--lock", default=None, help="kosu basina mkdir kilit dizini")
    p.add_argument("--label", default="tr35-smoke")
    p.add_argument("--num-envs", type=int, default=32)
    p.add_argument("--num-timesteps", type=float, default=2e4)
    p.add_argument("--num-evals", type=int, default=2)
    p.add_argument("--num-eval-envs", type=int, default=8)
    p.add_argument("--unroll-length", type=int, default=10)
    p.add_argument("--bank-n", type=int, default=64)
    p.add_argument("--timeout", type=float, default=3600.0, help="kosu basina saniye")
    return p.parse_args(argv)


class _Lock(object):
    def __init__(self, path, label):
        self.path, self.label = path, label

    def __enter__(self):
        if self.path:
            while True:
                try:
                    os.mkdir(self.path)
                    break
                except FileExistsError:
                    time.sleep(20)
            with open(os.path.join(self.path, "owner"), "w") as f:
                f.write("%s %s\n" % (self.label, time.ctime()))

    def __exit__(self, *exc):
        if self.path:
            shutil.rmtree(self.path, ignore_errors=True)


def _stub_scene(out):
    import importlib.util
    sp = importlib.util.spec_from_file_location("_openduck_golden", os.path.join(ROOT, "tools", "golden.py"))
    g = importlib.util.module_from_spec(sp)
    sp.loader.exec_module(g)
    d = os.path.join(out, "stub")
    g.build(d, config="tr35_b2", check=False)
    return os.path.join(d, "sim", "scene.xml")


def _check_log(path):
    """log.jsonl: her satirdaki her sayisal alan sonlu (egitici sonlu olmayani null yazar)."""
    bad, n, last = [], 0, {}
    with open(path) as f:
        for ln in f:
            rec = json.loads(ln)
            n += 1
            for k, v in rec.items():
                if v is None or (isinstance(v, float) and not math.isfinite(v)):
                    bad.append(k)
            last = rec
    return n, sorted(set(bad)), last


def run_task(a, env, task, extra=(), tag=None):
    tag = tag or task
    out = os.path.join(a.out, "smoke-" + tag)
    shutil.rmtree(out, ignore_errors=True)
    cmd = [a.python, "-m", "rl.train_mjx", "--task", task, "--out", out, "--num-envs", str(a.num_envs),
           "--num-timesteps", "%g" % a.num_timesteps, "--num-evals", str(a.num_evals),
           "--num-eval-envs", str(a.num_eval_envs), "--unroll-length", str(a.unroll_length)] + list(extra)
    t0 = time.time()
    with _Lock(a.lock, "%s %s" % (a.label, tag)):
        with open(os.path.join(a.out, "smoke-%s.log" % tag), "w") as f:
            try:
                rc = subprocess.run(cmd, cwd=ROOT, env=env, stdout=f, stderr=subprocess.STDOUT, timeout=a.timeout).returncode
            except subprocess.TimeoutExpired:
                rc = "timeout"
    r = {"task": task, "cmd": " ".join(cmd), "rc": rc, "seconds": round(time.time() - t0, 1)}
    logp = os.path.join(out, "log.jsonl")
    r["log_lines"], r["nonfinite"], last = _check_log(logp) if os.path.exists(logp) else (0, ["<log yok>"], {})
    r["last"] = {k: last.get(k) for k in ("step", "eval/episode_reward", "eval/avg_episode_length", "training/sps")}
    r["onnx"] = os.path.isfile(os.path.join(out, "policy.onnx"))
    mp = os.path.join(out, "policy_meta.json")
    meta = json.load(open(mp)) if os.path.isfile(mp) else {}
    r["meta_dims"] = [meta.get("obs_dim"), meta.get("action_dim")]
    r["ok"] = (rc == 0 and r["log_lines"] > 0 and not r["nonfinite"] and r["onnx"]
               and tuple(r["meta_dims"]) == TASK_DIMS[task])
    return r


def main(argv=None):
    a = parse(argv)
    os.makedirs(a.out, exist_ok=True)
    scene = os.path.abspath(a.scene) if a.scene else _stub_scene(a.out)
    env = dict(os.environ, OPENDUCK_ROBOT="tr35_b2", OPENDUCK_SCENE=scene, OPENDUCK_BANK_N=str(a.bank_n),
               PYTHONDONTWRITEBYTECODE="1", MUJOCO_GL=os.environ.get("MUJOCO_GL", "disabled"))
    env.setdefault("JAX_PLATFORMS", "cpu")
    if a.derived:
        env["OPENDUCK_DERIVED"] = os.path.abspath(a.derived)
    summary = {"scene": scene, "derived": env.get("OPENDUCK_DERIVED") or os.path.join(ROOT, "rl", "derived", "tr35_b2.json"),
               "runs": [], "getup_head_fixed": None}
    help_txt = subprocess.run([a.python, "-m", "rl.train_mjx", "--help"], cwd=ROOT, env=env, capture_output=True,
                              text=True, timeout=300).stdout
    for task in [t for t in a.tasks.split(",") if t]:
        r = run_task(a, env, task)
        summary["runs"].append(r)
        print(json.dumps(r), flush=True)
        if task == "getup":
            if "--getup-head" in help_txt:
                r2 = run_task(a, env, "getup", ("--getup-head", "fixed"), tag="getup-fixed")
                summary["runs"].append(r2)
                summary["getup_head_fixed"] = {"via": "trainer", "ok": r2["ok"]}
                print(json.dumps(r2), flush=True)
            else:
                t0 = time.time()
                with _Lock(a.lock, "%s getup-head-check" % a.label):
                    p = subprocess.run([a.python, "-c", _HEAD_CHECK % {"root": ROOT}], cwd=ROOT, env=env,
                                       capture_output=True, text=True, timeout=a.timeout)
                try:
                    res = json.loads(p.stdout.strip().splitlines()[-1])
                except (ValueError, IndexError):
                    res = {"ok": False, "stderr": p.stderr[-2000:]}
                res.update(via="env", rc=p.returncode, seconds=round(time.time() - t0, 1))
                summary["getup_head_fixed"] = res
                print(json.dumps(res), flush=True)
    ok = all(r["ok"] for r in summary["runs"]) and (summary["getup_head_fixed"] or {"ok": True})["ok"]
    summary["ok"] = bool(ok)
    with open(os.path.join(a.out, "tr35_smoke.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print("tr35_smoke: %s (%s)" % ("TAMAM" if ok else "HATA", os.path.join(a.out, "tr35_smoke.json")), flush=True)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
