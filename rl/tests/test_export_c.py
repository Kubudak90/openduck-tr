# -*- coding: utf-8 -*-
"""
rl/export_c.py testleri (spec 3.2.6, A7a): torch ELU (rl/export_onnx.py) ve brax swish (rl/train_mjx.py) graflari,
normalizasyon katlama esitligi, C derleme + onnxruntime esitligi (<= 1e-5), 256-128-64 aktorlerin boyut / MAC /
ESP32-S3 raporu. Torch yalniz ELU testinde gerekir (MJX venv'inde atlanir).

  python3 -m pytest rl/tests/test_export_c.py -s      (-s: boyut raporu yazdirilir)
"""
import json
import os
import subprocess
import tempfile
import types

import numpy as np
import pytest

from rl import export_c as EC
from rl import tasks as TK

CC = EC.find_cc()
needs_cc = pytest.mark.skipif(CC is None, reason="C derleyicisi yok")


def brax_params(n_obs, n_act, hidden=(256, 128, 64), n_critic=None, seed=0, scale=1.0):
    """rl/train_mjx.actor_arrays'in bekledigi (normalizer, politika, deger) yapisi, numpy ile (jax'siz)."""
    rng = np.random.default_rng(seed)
    f = lambda *s: rng.standard_normal(s).astype(np.float32)
    mean = f(n_obs) * 0.5
    std = np.exp(f(n_obs) * 0.5).astype(np.float32) * 0.3
    nc = n_critic or n_obs + 9
    norm = types.SimpleNamespace(mean={"state": mean, "privileged_state": np.zeros(nc, np.float32)},
                                 std={"state": std, "privileged_state": np.ones(nc, np.float32)})
    sizes = [n_obs] + list(hidden)
    k32 = lambda a, b: (f(a, b) * (scale / np.sqrt(a))).astype(np.float32)
    mlp = {"hidden_%d" % i: {"kernel": k32(a, b), "bias": f(b) * np.float32(0.1)}
           for i, (a, b) in enumerate(zip(sizes[:-1], sizes[1:]))}
    pol = {"params": {"MLP_0": mlp, "Dense_0": {"kernel": k32(sizes[-1], n_act), "bias": f(n_act) * np.float32(0.1)}}}
    return (norm, pol, None)


def brax_onnx(path, n_obs, n_act, hidden=(256, 128, 64), seed=0):
    from rl.train_mjx import actor_to_onnx
    import onnx
    onnx.save(actor_to_onnx(brax_params(n_obs, n_act, hidden, seed=seed)), path)
    return path


def torch_onnx(d, task="walk", hidden=(256, 128, 64), seed=0, name="policy.onnx"):
    torch = pytest.importorskip("torch")
    from rl.export_onnx import export_policy, policy_meta
    from rl.ppo import ActorCritic
    spec = TK.get_task(task)
    torch.manual_seed(seed)
    ac = ActorCritic(spec.num_obs, spec.num_critic_obs, spec.num_actions, list(hidden), [64])
    ac.actor_obs_normalizer.update(torch.randn(4000, spec.num_obs) * torch.rand(spec.num_obs) * 2
                                   + torch.randn(spec.num_obs))
    with torch.no_grad():
        for p in ac.actor.parameters():
            p.add_(0.05 * torch.randn_like(p))
    path = os.path.join(d, name)
    export_policy(ac, path, meta=policy_meta(11, "cpu_torch", task=task))
    return path, ac


# --- graf ayrisma ve katlama -----------------------------------------------------------------------
def test_parse_torch_and_brax_graphs():
    """torch Gemm transB=1 ve brax Gemm (W [in, out]) ayni (cikis, girdi) yerlesimine iner; Sub/Div ve
    aktivasyonlar taninir."""
    with tempfile.TemporaryDirectory() as d:
        g = EC.load_graph(brax_onnx(os.path.join(d, "b.onnx"), 53, 10))
        assert g["obs_dim"] == 53 and g["act_dim"] == 10 and g["mean"] is not None and g["denom"] is not None
        assert [L["W"].shape for L in g["layers"]] == [(256, 53), (128, 256), (64, 128), (10, 64)]
        assert [L["act"] for L in g["layers"]] == ["swish", "swish", "swish", None]
        p = brax_params(53, 10)
        assert np.array_equal(g["layers"][0]["W"], p[1]["params"]["MLP_0"]["hidden_0"]["kernel"].T.astype(np.float64))
        try:
            import torch  # noqa: F401
        except ImportError:
            return
        path, ac = torch_onnx(d, "getup")
        g = EC.load_graph(path)
        assert g["obs_dim"] == 48 and g["act_dim"] == 14
        assert [L["act"] for L in g["layers"]] == ["elu", "elu", "elu", None]
        assert all(L["alpha"] == 1.0 for L in g["layers"][:-1])
        assert np.allclose(g["layers"][0]["W"], ac.actor[0].weight.detach().numpy())
        n = ac.actor_obs_normalizer
        assert np.allclose(g["denom"], (n.std + n.eps).numpy(), rtol=1e-6)


def test_fold_equality():
    """W1' = W1 / s (girdi sutunu basina), b1' = b1 - W1' m; katlanmis ag katlanmamisla ayni sonucu verir."""
    with tempfile.TemporaryDirectory() as d:
        g = EC.load_graph(brax_onnx(os.path.join(d, "b.onnx"), 49, 10, seed=3))
    W, b = g["layers"][0]["W"], g["layers"][0]["b"]
    f = EC.fold(g)
    W1 = W / g["denom"][None, :]
    assert np.allclose(f[0][0], W1.astype(np.float32), rtol=0, atol=0)
    assert np.allclose(f[0][1], (b - W1 @ g["mean"]).astype(np.float32), rtol=0, atol=0)
    for k in range(1, len(f)):
        assert np.array_equal(f[k][0], g["layers"][k]["W"].astype(np.float32))
    x = EC.check_obs(g, 1000, seed=1).astype(np.float64)
    ref = EC.forward(g, x)
    assert np.max(np.abs(EC.forward_folded(f, x) - ref)) < 1e-5
    # tam float64 katlama: matematiksel esitlik
    g64 = dict(g, layers=[dict(L) for L in g["layers"]])
    g64["layers"][0] = dict(g["layers"][0], W=W1, b=b - W1 @ g["mean"])
    g64["mean"] = g64["denom"] = None
    assert np.max(np.abs(EC.forward(g64, x) - ref)) < 1e-10


def test_unsupported_graph_rejected():
    import onnx
    from onnx import TensorProto, helper, numpy_helper
    with tempfile.TemporaryDirectory() as d:
        w = numpy_helper.from_array(np.ones((3, 2), np.float32), "W")
        nodes = [helper.make_node("MatMul", ["obs", "W"], ["action"])]
        gr = helper.make_graph(nodes, "x", [helper.make_tensor_value_info("obs", TensorProto.FLOAT, ["b", 3])],
                               [helper.make_tensor_value_info("action", TensorProto.FLOAT, ["b", 2])], [w])
        p = os.path.join(d, "x.onnx")
        onnx.save(helper.make_model(gr), p)
        with pytest.raises(ValueError):
            EC.load_graph(p)
        p2 = brax_onnx(os.path.join(d, "b.onnx"), 49, 10, hidden=(8,))
        with pytest.raises(ValueError):
            EC.check(d, p2, n=10)
        with pytest.raises(SystemExit):
            EC.main(["--onnx", p2, "--out", d, "--check", "10"])


# --- C derleme + esitlik ---------------------------------------------------------------------------
@needs_cc
def test_c_matches_onnxruntime_brax_swish():
    """brax swish grafi (getup 48 -> 14): C ciktisi onnxruntime ile <= 1e-5; dosyalar, meta'siz obs_layout.h, CLI."""
    with tempfile.TemporaryDirectory() as d:
        p = brax_onnx(os.path.join(d, "policy.onnx"), 48, 14, seed=5)
        out = os.path.join(d, "c")
        rep = EC.export(p, out, check_n=1000)
        c = rep["check"]
        assert c["equal"] and c["max_abs_diff"] <= 1e-5 and c["n"] == 1000, c
        assert c["max_abs_action"] > 0.1 and c["max_abs_diff_ref64"] < 1e-4 and not c["warnings"], c
        assert sorted(os.listdir(out)) == sorted(EC.FILES + ("export_c.json",))
        assert rep["layers"][0]["activation"] == "swish" and rep["sizes"] == [48, 256, 128, 64, 14]
        with open(os.path.join(out, "export_c.json")) as f:
            assert json.load(f)["onnx_sha256"] == EC.sha256_file(p)
        with open(os.path.join(out, "policy.c")) as f:
            src = f.read()
        assert "expf(-x[i])" in src and "expm1f" not in src
        with open(os.path.join(out, "policy_weights.h")) as f:
            assert "__attribute__((aligned(16)))" in f.read()
        with open(os.path.join(out, "obs_layout.h")) as f:
            assert "#define OBS_DIM 48" in f.read()
        # ayni ONNX -> ayni dosyalar (zaman damgasi yok)
        h0 = dict(rep["files"])
        assert EC.export(p, out)["files"] == h0
        assert EC.main(["--onnx", p, "--out", out, "--check", "1000"]) == 0


@needs_cc
def test_c_matches_onnxruntime_torch_elu():
    """torch ELU grafi (export_onnx, normalizator eps'li): katlanmis C <= 1e-5; meta'dan obs_layout.h derlenir."""
    pytest.importorskip("torch")
    with tempfile.TemporaryDirectory() as d:
        path, _ = torch_onnx(d, "main")
        out = os.path.join(d, "c")
        rep = EC.export(path, out, check_n=1200, seed=2)
        c = rep["check"]
        assert c["equal"] and c["max_abs_diff"] <= 1e-5 and c["n"] == 1200, c
        assert rep["task"] == "main" and rep["layers"][0]["activation"] == "elu" and rep["meta"].endswith("_meta.json")
        with open(os.path.join(out, "policy.c")) as f:
            assert "expm1f" in f.read()
        with open(os.path.join(out, "obs_layout.h")) as f:
            hdr = f.read()
        lay = TK.get_task("main").obs_slices
        assert "#define OBS_COMMAND_OFFSET %d" % lay["command"].start in hdr and "#define ACT_DIM 10" in hdr
        assert "OPENDUCK_ACTION_SCALE[10]" in hdr and "#define CTRL_DT 0.02f" in hdr
        prog = os.path.join(d, "t.c")
        with open(prog, "w") as f:
            f.write('#include <stdio.h>\n#include "obs_layout.h"\nint main(void) {\n float s = 0; int i;\n'
                    ' for (i = 0; i < ACT_DIM; ++i) s += OPENDUCK_ACTION_SCALE[i] + OPENDUCK_DEFAULT_ACTION_Q[i];\n'
                    ' for (i = 0; i < 14; ++i) s += OPENDUCK_DEFAULT_OBS_Q[i];\n'
                    ' printf("%d %d %d %g\\n", OBS_DIM, OBS_COMMAND_OFFSET, OBS_COMMAND_SIZE, s);\n return 0;\n}\n')
        exe = os.path.join(d, "t")
        r = subprocess.run([CC, "-std=c99", "-I", out, "-o", exe, prog], capture_output=True, text=True)
        assert r.returncode == 0, r.stderr
        o = subprocess.run([exe], capture_output=True, text=True).stdout.split()
        assert o[:3] == ["53", str(lay["command"].start), "5"]


@needs_cc
def test_actor_size_report_all_tasks():
    """256-128-64 aktorleri (walk 49 -> 10, main 53 -> 10, getup 48 -> 14): MAC / bayt / ESP32-S3 tahmini (< 2 ms) ve
    tam boy C <= 1e-5 (brax swish)."""
    rows = []
    with tempfile.TemporaryDirectory() as d:
        for task in TK.TASK_NAMES:
            spec = TK.get_task(task)
            p = brax_onnx(os.path.join(d, task + ".onnx"), spec.num_obs, spec.num_actions, seed=7)
            rep = EC.export(p, os.path.join(d, task), check_n=1000)
            e = rep["esp32s3"]
            rows.append((task, rep, e))
            assert rep["hidden"] == [256, 128, 64] and rep["check"]["equal"], rep["check"]
            assert rep["params"] == rep["macs"] + 256 + 128 + 64 + spec.num_actions
            assert rep["weight_bytes"] == 4 * rep["params"] and e["pass"] and e["estimate_ms"] < 2.0
    print("\nESP32-S3 (240 MHz) aktor raporu:")
    for task, rep, e in rows:
        print("  %-5s %-18s MAC %6d  param %6d  %6.1f KB  ESP-DSP %.2f ms  ANSI %.2f ms (aktivasyon %.2f ms)  "
              "C/ORT %.1e" % (task, "-".join(str(s) for s in rep["sizes"]), rep["macs"], rep["params"],
                              rep["weight_bytes"] / 1024.0, e["total_ms"]["esp_dsp"], e["total_ms"]["ansi_c"],
                              e["activation_ms"], rep["check"]["max_abs_diff"]))
    walk = EC.size_report([49, 256, 128, 64, 10])
    assert walk["macs"] == 54144 and walk["params"] == 54602            # rl-map F1 tablosu
    assert EC.size_report([53, 256, 128, 64, 10])["macs"] == 55168
    big = EC.size_report([49, 512, 256, 128, 10], "swish")
    assert big["macs"] == 190208 and not big["esp32s3"]["pass"]


@needs_cc
def test_robot_block_tr35_meta_compiles():
    """tr35 meta'si (policy_meta; xl330 surecinde robot="tr35_b2", P4 anlami): obs_layout.h robot sabitleri (aktuator
    sirasi tau_s / pwm_res / ctrl araligi / kart, turetilmis dosya kimligi, kappa, bas tuck, oturus pitch'i) meta ile
    ayni ve derlenir; xl330 meta'sinda blok yok (P5a entegrasyonu)."""
    from rl import policy_meta as PM
    x = EC.obs_layout_header(PM.build_meta(1, "mjx_brax", task="getup"), {"obs_dim": 48, "act_dim": 14}, "0" * 64)
    assert "OPENDUCK_ACT_" not in x and "OPENDUCK_NU" not in x
    meta = PM.build_meta(1, "mjx_brax", task="getup", robot="tr35_b2")
    meta.update(derived_source={"mjcf_sha256": "ab" * 32, "stub": True, "scene": "s", "config_sha256": None},
                derived_sha256="cd" * 32, kappa=0.5, getup_head="fixed",
                head_tuck_deg={"neck_pitch": 23.0, "head_pitch": 55.5, "head_yaw": 0.0, "head_roll": 0.0})
    meta.setdefault("posture_ref", {})["seat_pitch_deg"] = -50.0
    a = meta["actuators"]
    with tempfile.TemporaryDirectory() as d:
        p = brax_onnx(os.path.join(d, "g.onnx"), 48, 14, hidden=(16, 8), seed=3)
        out = os.path.join(d, "c")
        rep = EC.export(p, out, meta=meta)
        assert rep["robot"] == "tr35_b2" and rep["kappa"] == 0.5 and rep["derived_sha256"] == "cd" * 32
        with open(os.path.join(out, "obs_layout.h")) as f:
            hdr = f.read()
        assert '#define OPENDUCK_DERIVED_MJCF_SHA256 "%s"' % ("ab" * 32) in hdr and "#define OPENDUCK_DERIVED_STUB 1" in hdr
        assert "#define OPENDUCK_NU 15" in hdr and "#define OPENDUCK_BOARD0_PWM_HZ 300" in hdr
        prog = os.path.join(d, "t.c")
        with open(prog, "w") as f:
            f.write('#include <stdio.h>\n#include "obs_layout.h"\nint main(void) {\n int i;\n'
                    ' for (i = 0; i < OPENDUCK_NU; ++i) printf("%s %.9g %.9g %.9g %.9g %d\\n", OPENDUCK_ACT_NAMES[i],'
                    ' OPENDUCK_ACT_TAU_S[i], OPENDUCK_ACT_PWM_RES[i], OPENDUCK_ACT_CTRL_LO[i], OPENDUCK_ACT_CTRL_HI[i],'
                    ' OPENDUCK_ACT_BOARD[i]);\n'
                    ' printf("%.9g %.9g %.9g\\n", OPENDUCK_HEAD_TUCK_RAD[1], OPENDUCK_SEAT_PITCH_DEG, OPENDUCK_KAPPA);\n'
                    ' return 0;\n}\n')
        exe = os.path.join(d, "t")
        r = subprocess.run([CC, "-std=c99", "-I", out, "-o", exe, prog], capture_output=True, text=True)
        assert r.returncode == 0, r.stderr
        rows = [ln.split() for ln in subprocess.run([exe], capture_output=True, text=True).stdout.splitlines()]
    assert [x[0] for x in rows[:15]] == a["names"]
    for i, x in enumerate(rows[:15]):
        v = [float(t) for t in x[1:5]]
        ref = [a["tau_s"][i], a["pwm_res"][i], a["ctrl_lo"][i], a["ctrl_hi"][i]]
        assert np.allclose(v, ref, rtol=1e-6, atol=1e-9) and int(x[5]) == a["board"][i], (i, x)
    assert np.allclose([float(t) for t in rows[15]], [np.deg2rad(55.5), -50.0, 0.5], rtol=1e-6)
