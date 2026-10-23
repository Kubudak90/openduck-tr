# -*- coding: utf-8 -*-
"""
GPU (MJX + Brax PPO) egitimi, asimetrik aktor-kritik ("state" -> aktor, "privileged_state" -> kritik).

  python3 -m rl.train_mjx --out runs/NAME [--task walk|main|getup] [--num-envs 8192] [--num-timesteps 2e8]
                          [--seed S] [--num-evals 20] [--num-eval-envs 128] [--restore runs/X/params_latest.pkl]
                          [--no-randomize] [--no-noise] [--no-head] [--no-push] [--env-precision highest]

Varsayilanlar MuJoCo Playground'un iki ayakli joystick ayarlarina yakindir (8192 ortam, unroll 20,
32 minibatch x 4 guncelleme, gamma 0.97, lr 3e-4, entropi 5e-3, 512-256-128 swish agi).
--task (rl/tasks.py): walk (varsayilan, degismez), main (yurume + yukseklik + otur/kalk; gozlem 53, aksiyon 10),
getup (dusunce kalkma; gozlem 48, aksiyon 14, 400 adimlik bolum). Gorev varsayilanlari TaskSpec'ten:
--discounting (gamma; walk/main 0.97, getup 0.99), --policy-hidden / --value-hidden, bolum uzunlugu.
getup: egitim ortami gec odul terimlerini ilerlemeyle (p = ortam basina adim * num_envs / num_timesteps,
info["gstep"]) acar, degerlendirme ortami son asama agirliklariyla (p = 1) calisir. --restore'da sayac 0'dan
baslar (mufredat bastan).
Dagilim "normal" (tanh yok): politika ciktisi ham aksiyondur, ortam +-ACTION_CLIP ile kirpar (CPU ile ayni).
Ortam fizigi tam float32 matmul ile izlenir (--env-precision; GPU'daki TF32 varsayilani yalnizca agda kalir).
ONNX dogrulamasi float64 numpy aktor ve CPU'daki JAX politikasina gore yapilir; egitim sirasindaki
disa aktarma hatasi yalnizca uyari verir.

runs/NAME/ altina yazilanlar:
  config.json                 komut satiri + rl/config.py sabitleri
  log.jsonl                   her degerlendirmede bir JSON satiri (eval/*, training/*)
  params_<adim>.pkl, params_latest.pkl   (normalizer, politika, deger) parametreleri, numpy
  policy.onnx, policy_meta.json          her degerlendirmede ve sonda (rl/TASK.md bicimi; meta rl/policy_meta.py,
                                         gorev meta["task"] ile, walk'ta bugunku alanlar aynen)
"""
import argparse
import functools
import json
import math
import os
import pickle
import sys
import time

import numpy as np

from . import config as C
from . import policy_meta as PM
from . import tasks as TK

ONNX_OPSET = 13
ONNX_IR_VERSION = 7
DEFAULT_GAMMA = 0.97
DEFAULT_HIDDEN = "512,256,128"


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="OpenDuck MJX + Brax PPO egitimi")
    p.add_argument("--out", required=True, help="cikti klasoru, or. runs/NAME")
    p.add_argument("--task", default="walk", choices=list(TK.TASK_NAMES),
                   help="walk (varsayilan), main (yurume + yukseklik + otur/kalk), getup (dusunce kalkma)")
    p.add_argument("--num-envs", type=int, default=8192)
    p.add_argument("--num-timesteps", type=float, default=2e8)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--num-evals", type=int, default=20)
    p.add_argument("--num-eval-envs", type=int, default=128)
    p.add_argument("--unroll-length", type=int, default=20)
    p.add_argument("--num-minibatches", type=int, default=None, help="varsayilan min(32, num_envs)")
    p.add_argument("--batch-size", type=int, default=None, help="varsayilan num_envs / num_minibatches")
    p.add_argument("--updates-per-batch", type=int, default=4)
    p.add_argument("--learning-rate", type=float, default=3e-4)
    p.add_argument("--entropy-cost", type=float, default=5e-3)
    p.add_argument("--discounting", type=float, default=None,
                   help="gamma; varsayilan gorevden (walk/main %.2f, getup 0.99)" % DEFAULT_GAMMA)
    p.add_argument("--gae-lambda", type=float, default=0.95)
    p.add_argument("--clipping-epsilon", type=float, default=0.2)
    p.add_argument("--max-grad-norm", type=float, default=1.0)
    p.add_argument("--init-noise-std", type=float, default=None,
                   help="varsayilan gorevden (walk/main %.1f, getup 0.5)" % C.PPO["init_noise_std"])
    p.add_argument("--policy-hidden", type=str, default=None, help="varsayilan gorevden (%s)" % DEFAULT_HIDDEN)
    p.add_argument("--value-hidden", type=str, default=None, help="varsayilan gorevden (%s)" % DEFAULT_HIDDEN)
    p.add_argument("--bootstrap-on-timeout", action="store_true",
                   help="zaman asiminda r += gamma*V(s) (varsayilan: Playground gibi truncation maskesi)")
    p.add_argument("--log-training-metrics", action="store_true",
                   help="egitim bolumlerinin metriklerini de log.jsonl'a yaz (episode/*)")
    p.add_argument("--solver-iterations", type=int, default=None, help="varsayilan gorevden (walk 4, main/getup 8)")
    p.add_argument("--ls-iterations", type=int, default=8)
    p.add_argument("--env-precision", default="highest", choices=["highest", "high", "default"],
                   help="ortam (fizik) matmul hassasiyeti; 'default' = JAX varsayilani (GPU'da TF32)")
    p.add_argument("--restore", default=None, help="params_*.pkl (normalizer + politika + deger)")
    p.add_argument("--no-randomize", action="store_true")
    p.add_argument("--no-noise", action="store_true")
    p.add_argument("--no-head", action="store_true")
    p.add_argument("--no-push", action="store_true")
    p.add_argument("--jax-cache", default=os.path.expanduser("~/.cache/openduck_jax"),
                   help="JAX kalici derleme onbellegi ('' kapatir)")
    a = p.parse_args(argv)
    spec = TK.get_task(a.task)                 # gorev varsayilanlari (walk: bugunku degerler)
    if a.discounting is None:
        a.discounting = DEFAULT_GAMMA if spec.gamma is None else float(spec.gamma)
    if a.policy_hidden is None:
        a.policy_hidden = DEFAULT_HIDDEN if spec.actor_hidden is None else ",".join(map(str, spec.actor_hidden))
    if a.value_hidden is None:
        a.value_hidden = DEFAULT_HIDDEN if spec.critic_hidden is None else ",".join(map(str, spec.critic_hidden))
    if a.init_noise_std is None:
        a.init_noise_std = C.PPO["init_noise_std"] if spec.init_noise_std is None else float(spec.init_noise_std)
    if a.solver_iterations is None:
        a.solver_iterations = 4 if spec.solver_iterations is None else int(spec.solver_iterations)
    if a.num_minibatches is None:
        a.num_minibatches = min(32, a.num_envs)
    if a.batch_size is None:
        a.batch_size = max(1, a.num_envs // a.num_minibatches)
    if (a.batch_size * a.num_minibatches) % a.num_envs:
        p.error("batch_size * num_minibatches, num_envs'in kati olmali")
    return a


def _hidden(s):
    return tuple(int(x) for x in s.split(",") if x.strip())


def network_factory(args):
    from brax.training.agents.ppo import networks as ppo_networks
    return functools.partial(
        ppo_networks.make_ppo_networks,
        policy_hidden_layer_sizes=_hidden(args.policy_hidden),
        value_hidden_layer_sizes=_hidden(args.value_hidden),
        policy_obs_key="state", value_obs_key="privileged_state",
        distribution_type="normal", noise_std_type="log", init_noise_std=args.init_noise_std)


# --- ONNX -------------------------------------------------------------------------
def actor_arrays(params):
    """(normalizer, politika, deger) -> aktor icin numpy: obs ortalama/std ve [(W, b), ...] katmanlari.
    Brax PolicyModuleWithStd: MLP_0 (her katmandan sonra swish) + Dense_0 (ortalama)."""
    norm, pol = params[0], params[1]
    mean = np.asarray(norm.mean["state"], np.float32)
    std = np.asarray(norm.std["state"], np.float32)
    p = pol["params"]
    mlp = p["MLP_0"]
    layers = [(np.asarray(mlp["hidden_%d" % i]["kernel"], np.float32), np.asarray(mlp["hidden_%d" % i]["bias"], np.float32))
              for i in range(len(mlp))]
    layers.append((np.asarray(p["Dense_0"]["kernel"], np.float32), np.asarray(p["Dense_0"]["bias"], np.float32)))
    return mean, std, layers


def actor_numpy(params, obs):
    """Deterministik aktor ciktisi, float64 numpy (ONNX dogrulamasinin referansi)."""
    mean, std, layers = actor_arrays(params)
    x = (np.asarray(obs, np.float64) - mean) / std
    for i, (w, b) in enumerate(layers):
        x = x @ w.astype(np.float64) + b
        if i < len(layers) - 1:
            x = x / (1.0 + np.exp(-x))
    return x


def actor_to_onnx(params):
    import onnx
    from onnx import TensorProto, helper, numpy_helper
    mean, std, layers = actor_arrays(params)
    inits = [numpy_helper.from_array(mean, "obs_mean"), numpy_helper.from_array(std, "obs_std")]
    nodes = [helper.make_node("Sub", ["obs", "obs_mean"], ["obs_centered"]),
             helper.make_node("Div", ["obs_centered", "obs_std"], ["x0"])]
    x = "x0"
    for i, (w, b) in enumerate(layers):
        inits += [numpy_helper.from_array(w, "W%d" % i), numpy_helper.from_array(b, "b%d" % i)]
        last = i == len(layers) - 1
        y = "action" if last else "h%d" % i
        nodes.append(helper.make_node("Gemm", [x, "W%d" % i, "b%d" % i], [y]))
        if not last:      # swish = x * sigmoid(x)
            nodes.append(helper.make_node("Sigmoid", [y], [y + "_sig"]))
            nodes.append(helper.make_node("Mul", [y, y + "_sig"], [y + "_act"]))
            y = y + "_act"
        x = y
    graph = helper.make_graph(
        nodes, "openduck_policy",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, ["batch", int(mean.shape[0])])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, ["batch", int(layers[-1][0].shape[1])])],
        inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", ONNX_OPSET)], producer_name="openduck-rl-mjx")
    model.ir_version = ONNX_IR_VERSION
    onnx.checker.check_model(model)
    return model


def policy_meta(steps, idx=None, task="walk"):
    """rl/policy_meta.build_meta (walk: bugunku alanlar aynen; main/getup: + gorev alanlari)."""
    return PM.build_meta(steps, "mjx_brax", idx, task)


def params_dims(params):
    """(aktor gozlemi, kritik gozlemi, aksiyon) boyutlari, parametrelerden (normalizer + son katman)."""
    norm = params[0]
    mean, _, layers = actor_arrays(params)
    return int(mean.shape[0]), int(np.asarray(norm.mean["privileged_state"]).shape[-1]), int(layers[-1][0].shape[1])


def _atomic(path, write_fn):
    tmp = path + ".tmp"
    write_fn(tmp)
    os.replace(tmp, path)


def jax_policy_cpu(params, make_policy, obs):
    """Brax deterministik politikasi CPU'da, tam float32 matmul ile (GPU'da varsayilan TF32 ~1e-3 hata verir)."""
    import jax
    import jax.numpy as jnp
    try:
        dev = jax.devices("cpu")[0]
    except RuntimeError:        # JAX_PLATFORMS=cuda gibi: CPU arka ucu yok, "highest" yine tam float32
        dev = jax.devices()[0]
    host = jax.device_get(params)
    n_critic = params_dims(host)[1]
    with jax.default_device(dev), jax.default_matmul_precision("highest"):
        pol = make_policy(host, deterministic=True)
        o = {"state": jnp.asarray(obs, jnp.float32),
             "privileged_state": jnp.zeros((len(obs), n_critic), jnp.float32)}
        return np.asarray(pol(o, jax.random.PRNGKey(0))[0])


def check_onnx(onnx_path, params, make_policy=None, n=257, seed=0, atol=1e-4):
    """onnxruntime ciktisini float64 numpy aktorle karsilastirir; make_policy verilirse numpy aktor de
    Brax'in JAX politikasiyla karsilastirilir (ag yorumunun dogrulugu). En buyuk ONNX farkini dondurur."""
    import onnxruntime as ort
    mean, std, _ = actor_arrays(params)
    rng = np.random.default_rng(seed)
    x = (mean + 3.0 * std * rng.standard_normal((n, mean.shape[0]))).astype(np.float32)
    ref = actor_numpy(params, x)
    tol = atol * max(1.0, float(np.max(np.abs(ref))))
    if make_policy is not None:
        jerr = float(np.max(np.abs(jax_policy_cpu(params, make_policy, x) - ref)))
        if not jerr <= tol:
            raise RuntimeError("numpy aktor / JAX politikasi farki %.3g (%s)" % (jerr, onnx_path))
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    out = sess.run(["action"], {"obs": x})[0]
    out1 = sess.run(["action"], {"obs": x[:1]})[0]
    err = max(float(np.max(np.abs(out - ref))), float(np.max(np.abs(out1 - ref[:1]))))
    if not err <= tol:
        raise RuntimeError("ONNX / numpy aktor farki %.3g (%s)" % (err, onnx_path))
    return err


def export_policy(params, out_dir, steps, make_policy=None, idx=None, task="walk"):
    """policy.onnx + policy_meta.json yazar. ONNX gecici dosyada onnxruntime ile dogrulanir, ancak gecerse
    yerine konur (basarisizsa onceki policy.onnx/meta kalir, RuntimeError). En buyuk farki dondurur.
    ONNX boyutlari parametrelerden; gorevin boyutlariyla (walk 49->10, main 53->10, getup 48->14) uyusmalidir."""
    import onnx
    spec = TK.get_task(task)
    dims = params_dims(params)
    if dims != (spec.num_obs, spec.num_critic_obs, spec.num_actions):
        raise RuntimeError("parametre boyutlari %s, %s gorevi %s bekliyor" % (
            dims, spec.name, (spec.num_obs, spec.num_critic_obs, spec.num_actions)))
    meta = policy_meta(steps, idx, spec.name)
    os.makedirs(out_dir, exist_ok=True)
    onnx_path = os.path.join(out_dir, "policy.onnx")
    tmp = onnx_path + ".tmp"
    try:
        onnx.save(actor_to_onnx(params), tmp)
        err = check_onnx(tmp, params, make_policy)
        os.replace(tmp, onnx_path)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)

    def w(p):
        with open(p, "w") as f:
            json.dump(meta, f, indent=1)
    _atomic(os.path.join(out_dir, "policy_meta.json"), w)
    return err


def save_params(params, out_dir, steps, args=None):
    import jax
    rec = {"params": jax.device_get(params), "steps": int(steps), "backend": "mjx_brax",
           "network": {"policy_hidden": args.policy_hidden, "value_hidden": args.value_hidden} if args else None}
    data = pickle.dumps(rec)
    for name in ("params_%d.pkl" % steps, "params_latest.pkl"):
        def w(p):
            with open(p, "wb") as f:
                f.write(data)
        _atomic(os.path.join(out_dir, name), w)


def load_params(path):
    with open(path, "rb") as f:
        return pickle.load(f)


def _num(x):
    try:
        x = float(np.asarray(x))
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def _config_snapshot():
    return {k: getattr(C, k) for k in dir(C) if k.isupper()}


def jax_compat():
    """brax 0.14.2, jax>=0.10'da kaldirilan jax.device_put_replicated/_sharded'i cagiriyor;
    jax'in ic uygulamasi hala var, modul ozelligi olarak geri baglanir."""
    import jax
    from jax._src import api as _api
    for name in ("device_put_replicated", "device_put_sharded"):
        try:
            getattr(jax, name)
        except AttributeError:
            if hasattr(_api, name):
                setattr(jax, name, getattr(_api, name))


def main(argv=None):
    args = parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    if args.jax_cache:
        os.environ.setdefault("JAX_COMPILATION_CACHE_DIR", args.jax_cache)
    import jax
    if args.jax_cache:
        jax.config.update("jax_compilation_cache_dir", os.environ["JAX_COMPILATION_CACHE_DIR"])
        jax.config.update("jax_persistent_cache_min_compile_time_secs", 5.0)
    jax_compat()
    from brax.training.agents.ppo import train as ppo
    from .env_mjx import OpenDuckEnv, wrap_for_brax_training

    cfg_path = os.path.join(args.out, "config.json")
    if os.path.exists(cfg_path):
        cfg_path = os.path.join(args.out, "config.%d.json" % int(time.time()))
    with open(cfg_path, "w") as f:
        json.dump({"args": vars(args), "argv": sys.argv, "config": _config_snapshot(),
                   "devices": [str(d) for d in jax.devices()]}, f, indent=1, default=str)

    spec = TK.get_task(args.task)

    def make_env(eval_env=False):
        kw = {}
        if spec.name != "walk":
            kw["task"] = spec.name
        if spec.reward_late and eval_env:     # getup: degerlendirme son asama agirliklariyla (p = 1)
            kw["final_weights"] = True
        elif spec.reward_late:                # egitim: p = gstep / (num_timesteps / num_envs)
            kw["curriculum_steps"] = args.num_timesteps / args.num_envs
        if not spec.terminate_on_fall:
            if eval_env:                      # eval/episode_getup_success: yalniz dusmus/dusus-ortasi baslangiclar
                kw["reset_probs"] = {k: p for k, p in C.GETUP["reset_probs"].items() if k != "stand"}
            else:                             # zaman asimlari dagilsin (her PPO yigini bolumun tum evrelerini gorsun)
                kw["random_initial_steps"] = True
        return OpenDuckEnv(randomize=not args.no_randomize, noise=not args.no_noise,
                           head_random=not args.no_head, push=not args.no_push,
                           solver_iterations=args.solver_iterations, ls_iterations=args.ls_iterations,
                           matmul_precision=None if args.env_precision == "default" else args.env_precision, **kw)
    env = make_env()
    idx = env.idx
    log_path = os.path.join(args.out, "log.jsonl")
    t0 = time.time()
    state = {"steps": 0}

    def progress(num_steps, metrics):
        rec = {"step": int(num_steps), "time": round(time.time() - t0, 2)}
        for k, v in metrics.items():
            rec[k] = _num(v)
        with open(log_path, "a") as f:
            f.write(json.dumps(rec) + "\n")
        if "eval/episode_reward" in metrics:
            print("adim %11d | %7.0fs | eval odul %8.3f | bolum %6.1f | sps %8.0f" % (
                num_steps, rec["time"], rec["eval/episode_reward"] or float("nan"),
                rec.get("eval/avg_episode_length") or float("nan"), rec.get("training/sps") or float("nan")),
                flush=True)

    def on_params(current_step, make_policy, params):
        state["steps"] = int(current_step)
        save_params(params, args.out, current_step, args)
        try:       # ara disa aktarma hatasi egitimi durdurmasin (params zaten kaydedildi)
            err = export_policy(params, args.out, current_step, make_policy, idx, spec.name)
        except Exception as e:
            print("UYARI adim %d: policy.onnx yazilamadi: %r" % (current_step, e), file=sys.stderr, flush=True)
            return
        print("adim %d: params + policy.onnx yazildi (onnx fark %.1e)" % (current_step, err), flush=True)

    restore = load_params(args.restore)["params"] if args.restore else None
    wrap = functools.partial(wrap_for_brax_training, bootstrap_on_timeout=args.bootstrap_on_timeout)
    print("egitim: %s gorevi, %d ortam, %.3g adim, cihazlar %s, cikti %s" % (
        spec.name, args.num_envs, args.num_timesteps, jax.devices(), args.out), flush=True)
    make_policy, params, _ = ppo.train(
        environment=env,
        eval_env=make_env(eval_env=True),
        num_timesteps=int(args.num_timesteps),
        num_evals=args.num_evals,
        episode_length=env.max_episode_steps,
        action_repeat=1,
        wrap_env_fn=wrap,
        num_envs=args.num_envs,
        num_eval_envs=args.num_eval_envs,
        deterministic_eval=True,
        unroll_length=args.unroll_length,
        batch_size=args.batch_size,
        num_minibatches=args.num_minibatches,
        num_updates_per_batch=args.updates_per_batch,
        learning_rate=args.learning_rate,
        entropy_cost=args.entropy_cost,
        discounting=args.discounting,
        gae_lambda=args.gae_lambda,
        clipping_epsilon=args.clipping_epsilon,
        max_grad_norm=args.max_grad_norm,
        normalize_observations=True,
        reward_scaling=1.0,
        bootstrap_on_timeout=args.bootstrap_on_timeout,
        network_factory=network_factory(args),
        seed=args.seed,
        log_training_metrics=args.log_training_metrics,
        progress_fn=progress,
        policy_params_fn=on_params,
        restore_params=restore,
    )
    steps = state["steps"]
    save_params(params, args.out, steps, args)
    err = export_policy(params, args.out, steps, make_policy, idx, spec.name)
    print("bitti: %s (adim %d, onnx fark %.1e, %.0f s)" % (
        os.path.join(args.out, "policy.onnx"), steps, err, time.time() - t0), flush=True)


if __name__ == "__main__":
    main()
