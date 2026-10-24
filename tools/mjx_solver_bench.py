# -*- coding: utf-8 -*-
"""
MJX cozucu olcumu (OpenDuck-TR P5a; spec A3): tr35_b2 kontrol adimi maliyeti ve tek adim dogrulugu, Newton yinelemesi,
frictionloss ve oz-carpisma ciftleri (acik/kapali) ile. Kendi tr35_b2 surecini kurar (OPENDUCK_ROBOT/SCENE/DERIVED).

  JAX_PLATFORMS=cpu <mjx py> tools/mjx_solver_bench.py --scene <scene.xml> [--derived <json>] --out bench.json \\
      [--mode time|acc|all] [--tasks walk,main,getup] [--iters 4,8,16] [--ls 8] [--n 256] [--fric 0.03] \\
      [--pairs off,derived,spec,main-subset] [--pair-condim 3] [--steps 20] [--bank-n 128]

  --pairs  off: cift yok; derived: C.SELF_COLLISION_PAIRS (turetilmis dosya; main'de robot_model.main_pairs alt kumesi,
           ortamlardaki gibi); spec: spec 3.2.2 aday listesi modelde var olan geomlardan (uyluk/incik <-> govde/kafa/cene,
           ayak <-> kafa, bacak <-> bacak, kafa <-> govde, ayak <-> ayni bacagin uyluk/kalca besigi), main'de de TAM
           liste; main-subset: yalniz alt kume (uyluk/incik <-> govde, ayak <-> ayni bacagin uyluk/kalca besigi).
           walk her zaman ciftsiz.
  time: vmap'li adim suresi (ctrl_step_ms, us_per_env_step), derleme suresi, temas yuvasi/kisit sayisi.
  acc:  ayni durumdan tek kontrol adimi, referans 64/32 yinelemeye gore eklem q / qd hatasi (p50/p99/en buyuk).
Sonuc JSON (her olcumden sonra yeniden yazilir): {"config", "time": {"<gorev>/<cift>/it<k>": ...}, "acc": {...}}.
"""
import argparse
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEG_BODIES = ("hip", "hip_cradle", "thigh", "shin", "foot")


def spec_pairs(geoms, subset=False):
    """spec 3.2.2 aday oz-carpisma ciftleri (geom adlari; yalniz modelde var olanlar). subset: main alt kumesi."""
    have = set(geoms)
    g = lambda side, b: "%s_%s_collision" % (side, b)
    out = []
    add = lambda a, b: out.append((a, b)) if a in have and b in have and (a, b) not in out and (b, a) not in out \
        else None
    for s in ("left", "right"):
        for b in ("thigh", "shin"):
            add(g(s, b), "trunk_collision")
            if not subset:
                add(g(s, b), "head_collision")
                add(g(s, b), "jaw_collision")
        for b in ("thigh", "hip_cradle"):
            add(g(s, "foot"), g(s, b))
        if not subset:
            add(g(s, "foot"), "head_collision")
    if not subset:
        for a in LEG_BODIES:
            for b in LEG_BODIES:
                if not (a == "foot" and b == "foot"):           # ayak-ayak zaten contype ile carpisir
                    add(g("left", a), g("right", b))
        add("head_collision", "trunk_collision")
    return out


def parse(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    p.add_argument("--scene", default=os.environ.get("OPENDUCK_SCENE"), help="tr35 scene.xml (OPENDUCK_SCENE)")
    p.add_argument("--derived", default=os.environ.get("OPENDUCK_DERIVED"),
                   help="turetilmis JSON (varsayilan rl/derived/tr35_b2.json)")
    p.add_argument("--out", required=True)
    p.add_argument("--mode", default="all", choices=("time", "acc", "all"))
    p.add_argument("--tasks", default="walk,main,getup")
    p.add_argument("--iters", default="4,8,16")
    p.add_argument("--ls", type=int, default=8)
    p.add_argument("--n", type=int, default=256, help="ortam sayisi")
    p.add_argument("--fric", type=float, default=None, help="tum siniflarda friction_frac (varsayilan: actuators)")
    p.add_argument("--pairs", default="off,derived")
    p.add_argument("--pair-condim", type=int, default=None, help="oz-carpisma cifti condim (varsayilan robot_model)")
    p.add_argument("--steps", type=int, default=20, help="zamanlanan adim sayisi")
    p.add_argument("--acc-steps", type=int, default=30)
    p.add_argument("--bank-n", type=int, default=128)
    a = p.parse_args(argv)
    if not a.scene:
        p.error("--scene (ya da OPENDUCK_SCENE) gerekli")
    return a


def main(argv=None):
    a = parse(argv)
    os.environ.update(OPENDUCK_ROBOT="tr35_b2", OPENDUCK_SCENE=os.path.abspath(a.scene), OPENDUCK_BANK_N=str(a.bank_n))
    if a.derived:
        os.environ["OPENDUCK_DERIVED"] = os.path.abspath(a.derived)
    sys.path.insert(0, ROOT)
    import numpy as np
    import jax
    import jax.numpy as jnp
    import mujoco
    from rl import actuators as A
    from rl import config as C
    from rl import robot_model as RM
    from rl.env_mjx import OpenDuckEnv
    if a.fric is not None:
        for c in A.CLASSES.values():
            c["friction_frac"] = float(a.fric)
    if a.pair_condim is not None:
        RM.SELF_PAIR_CONDIM = int(a.pair_condim)
    derived_pairs = tuple(C.SELF_COLLISION_PAIRS)
    res = {"config": {"argv": sys.argv, "args": vars(a), "derived_source": C.DERIVED_SOURCE,
                      "derived_pairs": len(derived_pairs), "mujoco": mujoco.__version__, "jax": jax.__version__,
                      "devices": [str(d) for d in jax.devices()], "pair_condim": RM.SELF_PAIR_CONDIM,
                      "friction_frac": {k: v["friction_frac"] for k, v in A.CLASSES.items()}},
           "time": {}, "acc": {}}

    def save():
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, "w") as f:
            json.dump(res, f, indent=1)

    def set_pairs(mode, task):
        RM.MAIN_PAIR_SUBSET = mode != "spec"           # spec: main'de de tam aday listesi (olcum)
        if task == "walk" or mode == "off":
            C.SELF_COLLISION_PAIRS = ()
        elif mode == "derived":
            C.SELF_COLLISION_PAIRS = derived_pairs
        else:
            names = [mujoco.mj_id2name(m0, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(m0.ngeom)]
            C.SELF_COLLISION_PAIRS = tuple(spec_pairs(names, subset=mode == "main-subset"))
        return len(C.SELF_COLLISION_PAIRS)

    def env(task, it, ls):
        return OpenDuckEnv(task=task, solver_iterations=it, ls_iterations=ls, randomize=True, noise=True,
                           head_random=False, push=False)

    keys = lambda n, seed: jax.random.split(jax.random.PRNGKey(seed), n)
    act = lambda rng, n, na: jnp.asarray(0.5 * rng.standard_normal((n, na)), jnp.float32)
    tasks = [t for t in a.tasks.split(",") if t]
    iters = [int(x) for x in a.iters.split(",") if x]
    modes = [x for x in a.pairs.split(",") if x]
    if a.mode in ("time", "all"):
        for task in tasks:
            C.SELF_COLLISION_PAIRS = ()
            m0 = RM.build_model(variant=task)
            for mode in (["off"] if task == "walk" else modes):
                npair = set_pairs(mode, task)
                for it in iters:
                    e = env(task, it, a.ls)
                    na = e.spec.num_actions
                    t0 = time.time()
                    rs, st_ = jax.jit(jax.vmap(e.reset)), jax.jit(jax.vmap(e.step))
                    rng = np.random.default_rng(0)
                    st = st_(rs(keys(a.n, 0)), act(rng, a.n, na))
                    jax.block_until_ready(st.obs)
                    tc = time.time() - t0
                    for _ in range(5):
                        st = st_(st, act(rng, a.n, na))
                    jax.block_until_ready(st.obs)
                    t0 = time.time()
                    for _ in range(a.steps):
                        st = st_(st, act(rng, a.n, na))
                    jax.block_until_ready(st.obs)
                    dt = (time.time() - t0) / a.steps
                    d0 = e._data0._impl
                    r = {"ctrl_step_ms": 1e3 * dt, "us_per_env_step": 1e6 * dt / a.n, "compile_s": tc,
                         "nonfinite_frac": float(np.mean(~np.isfinite(np.asarray(st.obs["state"])).all(-1))),
                         "npair": int(e.mj_model.npair), "pairs_requested": npair,
                         "ncon_slots": int(d0.contact.dist.shape[0]), "nefc": int(d0.efc_type.shape[0])}
                    res["time"]["%s/%s/it%d" % (task, mode, it)] = r
                    print("time", task, mode, it, {k: round(v, 3) if isinstance(v, float) else v for k, v in r.items()},
                          flush=True)
                    save()
    if a.mode in ("acc", "all"):
        for task in [t for t in tasks if t != "walk"] or tasks:
            C.SELF_COLLISION_PAIRS = ()
            m0 = RM.build_model(variant=task)
            for mode in ([modes[0]] if task == "walk" else modes):
                set_pairs(mode, task)
                ref = env(task, 64, 32)
                na = ref.spec.num_actions
                sref = jax.jit(jax.vmap(ref.step_no_reset))
                cands = {it: jax.jit(jax.vmap(env(task, it, a.ls).step_no_reset)) for it in iters}
                st = jax.jit(jax.vmap(ref.reset))(keys(a.n, 1))
                rng = np.random.default_rng(1)
                errs = {it: {"dq": [], "dv": []} for it in cands}
                for t in range(a.acc_steps):
                    u = act(rng, a.n, na)
                    nxt = sref(st, u)
                    if t % 3 == 0:
                        for it, f in cands.items():
                            o = f(st, u)
                            errs[it]["dq"].append(np.abs(np.asarray(o.data.qpos[:, 7:]) - np.asarray(nxt.data.qpos[:, 7:])).max(-1))
                            errs[it]["dv"].append(np.abs(np.asarray(o.data.qvel[:, 6:]) - np.asarray(nxt.data.qvel[:, 6:])).max(-1))
                    st = nxt
                for it, e_ in errs.items():
                    dq, dv = np.concatenate(e_["dq"]), np.concatenate(e_["dv"])
                    r = {"dq_p50": float(np.median(dq)), "dq_p99": float(np.quantile(dq, 0.99)), "dq_max": float(dq.max()),
                         "dv_p50": float(np.median(dv)), "dv_p99": float(np.quantile(dv, 0.99)), "dv_max": float(dv.max())}
                    res["acc"]["%s/%s/it%d" % (task, mode, it)] = r
                    print("acc", task, mode, it, {k: "%.2e" % v for k, v in r.items()}, flush=True)
                save()
    save()
    print("yazildi:", a.out)


if __name__ == "__main__":
    main()
