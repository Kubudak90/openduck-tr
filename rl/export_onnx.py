# -*- coding: utf-8 -*-
"""
Politika disa aktarma: aktor ortalamasi + gomulu gozlem normalizasyonu -> ONNX.

  girdi  "obs"    float32 [batch, num_obs]       (walk 49, main 53, getup 48)
  cikti  "action" float32 [batch, num_actions]   (walk/main 10, getup 14; deterministik, kirpmasiz)

Graf torch.onnx yerine dogrudan onnx.helper ile kurulur (Sub, Div, Gemm, aktivasyon);
opset 13 / IR 7, robottaki eski onnxruntime surumleri de acabilsin diye.
Ayrica <ad>_meta.json (rl/TASK.md'deki bicim, rl/policy_meta.build_meta) yazilir: policy.onnx -> policy_meta.json.

Komut satiri:  python3 -m rl.export_onnx --checkpoint runs/NAME/latest.pt [--out runs/NAME/policy.onnx]
                                         [--task walk|main|getup]   (varsayilan: checkpoint'taki gorev)
"""
import argparse
import json
import os

import numpy as np
import onnx
import torch
import torch.nn as nn
from onnx import TensorProto, helper, numpy_helper

from . import policy_meta as PM
from . import tasks as TK
from .ppo import ActorCritic

OPSET = 13
IR_VERSION = 7


def _act_node(mod, x, y):
    if isinstance(mod, nn.ELU):
        return helper.make_node("Elu", [x], [y], alpha=float(mod.alpha))
    if isinstance(mod, nn.ReLU):
        return helper.make_node("Relu", [x], [y])
    if isinstance(mod, nn.Tanh):
        return helper.make_node("Tanh", [x], [y])
    if isinstance(mod, nn.Sigmoid):
        return helper.make_node("Sigmoid", [x], [y])
    if isinstance(mod, nn.SELU):
        return helper.make_node("Selu", [x], [y])
    if isinstance(mod, nn.LeakyReLU):
        return helper.make_node("LeakyRelu", [x], [y], alpha=float(mod.negative_slope))
    raise ValueError("ONNX'e cevrilemeyen katman: %r" % mod)


def actor_to_onnx(actor_critic):
    ac = actor_critic
    num_obs, num_actions = ac.arch["num_obs"], ac.arch["num_actions"]
    f32 = lambda t: t.detach().cpu().numpy().astype(np.float32)
    nodes, inits, x = [], [], "obs"
    if ac.obs_normalization:
        n = ac.actor_obs_normalizer
        inits.append(numpy_helper.from_array(f32(n.mean), "obs_mean"))
        inits.append(numpy_helper.from_array(f32(n.std + n.eps), "obs_denom"))
        nodes.append(helper.make_node("Sub", ["obs", "obs_mean"], ["obs_centered"]))
        nodes.append(helper.make_node("Div", ["obs_centered", "obs_denom"], ["obs_norm"]))
        x = "obs_norm"
    layers = list(ac.actor)
    for i, mod in enumerate(layers):
        y = "action" if i == len(layers) - 1 else "h%d" % i
        if isinstance(mod, nn.Linear):
            w, b = "W%d" % i, "b%d" % i
            inits.append(numpy_helper.from_array(f32(mod.weight), w))
            inits.append(numpy_helper.from_array(f32(mod.bias), b))
            nodes.append(helper.make_node("Gemm", [x, w, b], [y], transB=1))
        else:
            nodes.append(_act_node(mod, x, y))
        x = y
    graph = helper.make_graph(
        nodes, "openduck_policy",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, ["batch", num_obs])],
        [helper.make_tensor_value_info("action", TensorProto.FLOAT, ["batch", num_actions])],
        inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", OPSET)],
                              producer_name="openduck-rl")
    model.ir_version = IR_VERSION
    onnx.checker.check_model(model)
    return model


def check_onnx(actor_critic, onnx_path, n=257, atol=1e-5, seed=0):
    """onnxruntime ciktisini torch ile karsilastirir (1 ve n satirlik iki parti); en buyuk mutlak hatayi
    dondurur. Sinir: atol * max(1, max|aksiyon|) (buyuk cikislarda float32 toplama sirasi farki)."""
    import onnxruntime as ort
    ac = actor_critic
    g = torch.Generator().manual_seed(seed)
    num_obs = ac.arch["num_obs"]
    x = torch.randn(n, num_obs, generator=g)
    if ac.obs_normalization:
        nrm = ac.actor_obs_normalizer
        x = nrm.mean + 3.0 * nrm.std * x
    with torch.no_grad():
        ref = ac.act_inference(x).numpy()
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    out = sess.run(["action"], {"obs": x.numpy().astype(np.float32)})[0]
    out1 = sess.run(["action"], {"obs": x[:1].numpy().astype(np.float32)})[0]
    err = max(float(np.max(np.abs(out - ref))), float(np.max(np.abs(out1 - ref[:1]))))
    tol = atol * max(1.0, float(np.max(np.abs(ref))))
    if not err <= tol:
        raise RuntimeError("ONNX/torch farki %.3g > %.3g (%s)" % (err, tol, onnx_path))
    return err


def meta_path_for(onnx_path):
    return os.path.splitext(onnx_path)[0] + "_meta.json"


def policy_meta(steps, backend="cpu_torch", idx=None, task="walk"):
    """rl/policy_meta.build_meta (torch'suz ortak meta; walk ciktisi bugunkuyle ayni)."""
    return PM.build_meta(steps, backend, idx, task)


def checkpoint_task(ckpt):
    """Checkpoint'in gorevi: ckpt["task"] (main/getup), yoksa run_info["task"], yoksa "walk"."""
    return ckpt.get("task") or (ckpt.get("run_info") or {}).get("task") or "walk"


def _atomic_write(path, write_fn):
    tmp = path + ".tmp"
    write_fn(tmp)
    os.replace(tmp, path)


def export_policy(actor_critic, onnx_path, meta=None, check=True, atol=1e-5):
    """policy.onnx (+ meta verilirse policy_meta.json) yazar. check: gecici dosya onnxruntime ile
    dogrulanir, ancak gecerse yerine konur (basarisizsa onceki policy.onnx/meta oldugu gibi kalir).
    onnxruntime kurulu degilse dogrulama atlanir. Dondurur: en buyuk |onnx - torch| (dogrulanmadiysa None)."""
    d = os.path.dirname(os.path.abspath(onnx_path))
    os.makedirs(d, exist_ok=True)
    if check:
        try:
            import onnxruntime  # noqa: F401
        except ImportError:
            check = False
    was_training = actor_critic.training
    actor_critic.eval()
    tmp = onnx_path + ".tmp"
    try:
        onnx.save(actor_to_onnx(actor_critic), tmp)
        err = check_onnx(actor_critic, tmp, atol=atol) if check else None
        os.replace(tmp, onnx_path)
    finally:
        actor_critic.train(was_training)
        if os.path.exists(tmp):
            os.remove(tmp)
    if meta is not None:
        def w(p):
            with open(p, "w") as f:
                json.dump(meta, f, indent=1)
        _atomic_write(meta_path_for(onnx_path), w)
    return err


def load_actor_critic(checkpoint_path):
    ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    ac = ActorCritic.from_arch(ckpt["arch"])
    ac.load_state_dict(ckpt["model"])
    ac.eval()
    return ac, ckpt


def main(argv=None):
    p = argparse.ArgumentParser(description="Checkpoint'tan policy.onnx + policy_meta.json uret")
    p.add_argument("--checkpoint", required=True, help="model_<iter>.pt ya da latest.pt")
    p.add_argument("--out", default=None, help="varsayilan: checkpoint klasorunde policy.onnx")
    p.add_argument("--task", choices=TK.TASK_NAMES, default=None, help="varsayilan: checkpoint'taki gorev")
    a = p.parse_args(argv)
    ac, ckpt = load_actor_critic(a.checkpoint)
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.checkpoint)), "policy.onnx")
    spec = TK.get_task(a.task or checkpoint_task(ckpt))
    if (ac.arch["num_obs"], ac.arch["num_actions"]) != (spec.num_obs, spec.num_actions):
        raise SystemExit("checkpoint %d/%d boyutlu, %s gorevi %d/%d bekler" % (
            ac.arch["num_obs"], ac.arch["num_actions"], spec.name, spec.num_obs, spec.num_actions))
    meta = policy_meta(ckpt.get("steps", 0), backend=ckpt.get("backend", "cpu_torch"), task=spec.name)
    err = export_policy(ac, out, meta=meta)
    print("%s yazildi (iter %s, %d adim), onnx/torch max fark %s" % (
        out, ckpt.get("iter"), meta["steps"], "dogrulanmadi" if err is None else "%.2e" % err))


if __name__ == "__main__":
    main()
