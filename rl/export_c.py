# -*- coding: utf-8 -*-
"""
Politika ONNX -> C99 float32 (OpenDuck-TR spec 3.2.6, A7a): ESP32-S3 calisma zamani icin agirlik dizileri ve tek
fonksiyon; gozlem normalizasyonu birinci katmana katlanir. Yalniz numpy + onnx (torch'suz; iki venv'de de calisir).

  python3 -m rl.export_c --onnx runs/X/policy.onnx [--meta runs/X/policy_meta.json] --out runs/X/c [--check N]

Girdi graflari (Sub(ortalama), Div(std), sonra Gemm + aktivasyon zinciri, son Gemm):
  rl/export_onnx.py (torch): Sub/Div (obs_denom = std + eps), Gemm transB=1, Elu (Relu/Tanh da)
  rl/train_mjx.py   (brax) : Sub/Div (obs_std), Gemm (W [in, out]), swish = Mul(h, Sigmoid(h))
Katlama (Gemm transB'ye gore W satir = cikis): W1' = W1 / s (girdi sutunu basina), b1' = b1 - W1' m; float64'te
hesaplanip float32'ye yuvarlanir. Sonraki katmanlar aynen.

Cikti (--out klasoru):
  policy_weights.h  static const float OPENDUCK_W<i>[out*in] (satir oncelikli: W[o*in + j]), OPENDUCK_B<i>[out],
                    __attribute__((aligned(16)))
  policy.h          void openduck_policy(const float *obs, float *action)  (ham aksiyon, kirpmasiz; statik ara
                    bellek: yeniden girisli degil)
  policy.c          C99: dense (acc = sum w*x, sonra + b), ELU expm1f ile, swish x / (1 + expf(-x))
  obs_layout.h      meta'dan #define'lar: OBS_DIM, ACT_DIM, OBS_<AD>_OFFSET/_SIZE, ACTION_CLIP, CTRL_DT, ... ve
                    OPENDUCK_ACTION_SCALE / OPENDUCK_DEFAULT_* dizileri; tr35 meta'sinda (actuators alani) ayrica robot
                    sabitleri: OPENDUCK_NU, OPENDUCK_ACT_* (aktuator sirasi: ad, tau_s, omega0, kp, deadband, pwm_res,
                    ctrl_lo/hi, board), OPENDUCK_BOARD<k>_PWM_HZ/_PCA, OPENDUCK_DERIVED_* (turetilmis dosya kimligi),
                    OPENDUCK_KAPPA, getup fixed'te OPENDUCK_HEAD_TUCK_RAD[4], main'de OPENDUCK_SEAT_PITCH_DEG
  export_c.json     onnx sha256, katmanlar, parametre / bayt / MAC, ESP32-S3 tahmini, dosya sha256'lari, --check sonucu
                    (zaman damgasi yok: ayni ONNX -> ayni dosyalar)
--check N (N >= 1000, spec A7a): test programi `cc -O2 -std=c99 -ffp-contract=off ... -lm` ile derlenir; N gozlemde
  (normalizator olcekli Gauss: m + 3 s z, ve uc degerler: 0, m, m +- 4 s, isaret degisimli +-4 s, tek eksen +-6 s) C
  ciktisi onnxruntime ile karsilastirilir. max |C - ORT| > atol (1e-5) -> cikis kodu 1.
ESP32-S3 tahmini (240 MHz): MAC basina 1.69 (ESP-DSP nokta carpimi) ve 5.12 (ANSI C) cevrim (rl-map F1) + aktivasyon
basina ACT_CYCLES cevrim (expf/expm1f; OLCULMEDI, varsayim). Kabul (A7a) tahmini ANSI toplamiyla: < 2 ms.
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

import numpy as np

TOOL_VERSION = 1
ESP32 = {"mhz": 240.0, "cycles_per_mac": {"esp_dsp": 1.69, "ansi_c": 5.12}}
# aktivasyon basina cevrim (ESP32-S3 FPU, newlib expf/expm1f ~150 cevrim varsayimi; olculmedi, rl-map F1 0.1-0.3 ms)
ACT_CYCLES = {"elu": 150.0, "swish": 170.0, "tanh": 200.0, "relu": 4.0}
A7_MS = 2.0
ATOL = 1e-5
MIN_CHECK = 1000
FILES = ("policy_weights.h", "policy.h", "policy.c", "obs_layout.h")
CFLAGS = ["-O2", "-std=c99", "-ffp-contract=off", "-Wall", "-Wextra"]


# --- ONNX grafi ---------------------------------------------------------------------------------------
def _attrs(node):
    from onnx import helper
    return {a.name: helper.get_attribute_value(a) for a in node.attribute}


def load_graph(onnx_path):
    """ONNX -> {"obs_dim", "act_dim", "mean" (obs_dim,) | None, "denom" | None, "layers": [{"W" (out,in), "b",
    "act" (None | "elu" | "swish" | "relu" | "tanh"), "alpha"}], "input", "output"} (float64). Desteklenmeyen
    dugumde ValueError."""
    import onnx
    from onnx import numpy_helper
    m = onnx.load(onnx_path)
    g = m.graph
    init = {t.name: numpy_helper.to_array(t).astype(np.float64) for t in g.initializer}
    inputs = [i.name for i in g.input if i.name not in init]
    if len(inputs) != 1 or len(g.output) != 1:
        raise ValueError("%s: tek girdi / tek cikti bekleniyor (%s / %d)" % (onnx_path, inputs, len(g.output)))
    x, out = inputs[0], g.output[0].name
    mean = denom = None
    layers = []
    nodes = list(g.node)
    i = 0
    while i < len(nodes):
        n = nodes[i]
        op, ins = n.op_type, list(n.input)
        const = [k for k in ins if k in init]
        if op == "Sub" and not layers and mean is None and denom is None and ins[0] == x and ins[1] in init:
            mean, x = init[ins[1]].reshape(-1), n.output[0]
        elif op == "Div" and not layers and denom is None and ins[0] == x and ins[1] in init:
            denom, x = init[ins[1]].reshape(-1), n.output[0]
        elif op == "Gemm" and ins[0] == x and len(const) == len(ins) - 1:
            a = _attrs(n)
            if a.get("transA", 0):
                raise ValueError("%s: Gemm transA desteklenmiyor" % onnx_path)
            if layers and layers[-1]["act"] is None:
                raise ValueError("%s: aktivasyonsuz ardisik Gemm" % onnx_path)
            W = init[ins[1]]
            W = W if a.get("transB", 0) else W.T                   # (out, in)
            b = init[ins[2]].reshape(-1) if len(ins) > 2 else np.zeros(W.shape[0])
            b = np.broadcast_to(b, (W.shape[0],)).copy()
            layers.append({"W": float(a.get("alpha", 1.0)) * W, "b": float(a.get("beta", 1.0)) * b, "act": None,
                           "alpha": None})
            x = n.output[0]
        elif op in ("Elu", "Relu", "Tanh") and layers and layers[-1]["act"] is None and ins[0] == x:
            layers[-1]["act"] = op.lower()
            if op == "Elu":
                layers[-1]["alpha"] = float(_attrs(n).get("alpha", 1.0))
            x = n.output[0]
        elif op == "Sigmoid" and layers and layers[-1]["act"] is None and ins[0] == x and i + 1 < len(nodes) \
                and nodes[i + 1].op_type == "Mul" and sorted(nodes[i + 1].input) == sorted([x, n.output[0]]):
            layers[-1]["act"] = "swish"
            x = nodes[i + 1].output[0]
            i += 1
        else:
            raise ValueError("%s: desteklenmeyen dugum %d: %s(%s)" % (onnx_path, i, op, ", ".join(ins)))
        i += 1
    if x != out or not layers:
        raise ValueError("%s: zincir '%s' ciktisina ulasmiyor (son %s)" % (onnx_path, out, x))
    if layers[-1]["act"] is not None or any(L["act"] is None for L in layers[:-1]):
        raise ValueError("%s: gizli katmanlarda aktivasyon, son katmanda yok bekleniyor" % onnx_path)
    d = layers[0]["W"].shape[1]
    for k, v in (("mean", mean), ("denom", denom)):
        if v is not None and v.shape != (d,):
            raise ValueError("%s: %s boyutu %s, gozlem %d" % (onnx_path, k, v.shape, d))
    for a, b in zip(layers[:-1], layers[1:]):
        if b["W"].shape[1] != a["W"].shape[0]:
            raise ValueError("%s: katman boyutlari uyusmuyor" % onnx_path)
    return {"obs_dim": int(d), "act_dim": int(layers[-1]["W"].shape[0]), "mean": mean, "denom": denom,
            "layers": layers, "input": inputs[0], "output": out}


def fold(graph):
    """Normalizasyonu ilk katmana katlar: [(W float32 (out,in), b float32, act, alpha)]. W1' = W1 / s,
    b1' = b1 - W1' m (float64'te)."""
    out = []
    for k, L in enumerate(graph["layers"]):
        W, b = L["W"], L["b"]
        if k == 0:
            s = graph["denom"] if graph["denom"] is not None else np.ones(W.shape[1])
            m = graph["mean"] if graph["mean"] is not None else np.zeros(W.shape[1])
            W = W / s[None, :]
            b = b - W @ m
        out.append((np.ascontiguousarray(W, np.float32), np.ascontiguousarray(b, np.float32), L["act"], L["alpha"]))
    return out


def _act(x, act, alpha):
    if act == "elu":
        return np.where(x > 0, x, alpha * np.expm1(np.minimum(x, 0.0)))
    if act == "swish":
        return x / (1.0 + np.exp(-x))
    if act == "relu":
        return np.maximum(x, 0.0)
    if act == "tanh":
        return np.tanh(x)
    return x


def forward(graph, obs):
    """Katlanmamis graf, float64 numpy referansi (obs (n, d))."""
    x = np.asarray(obs, np.float64)
    if graph["mean"] is not None:
        x = x - graph["mean"]
    if graph["denom"] is not None:
        x = x / graph["denom"]
    for L in graph["layers"]:
        x = _act(x @ L["W"].T + L["b"], L["act"], L["alpha"])
    return x


def forward_folded(folded, obs, dtype=np.float64):
    x = np.asarray(obs, dtype)
    for W, b, act, alpha in folded:
        x = _act(x @ W.astype(dtype).T + b.astype(dtype), act, alpha)
    return x


# --- boyut / sure tahmini -----------------------------------------------------------------------------
def size_report(sizes, act="elu"):
    """sizes [in, h1, ..., out] -> MAC, parametre, bayt (float32), aktivasyon sayisi, ESP32-S3 tahmini (ms)."""
    sizes = [int(s) for s in sizes]
    macs = int(sum(a * b for a, b in zip(sizes[:-1], sizes[1:])))
    params = macs + int(sum(sizes[1:]))
    n_act = int(sum(sizes[1:-1]))
    hz = ESP32["mhz"] * 1e6
    act_ms = 1e3 * n_act * ACT_CYCLES.get(act, max(ACT_CYCLES.values())) / hz
    ms = {k: 1e3 * macs * c / hz for k, c in ESP32["cycles_per_mac"].items()}
    tot = {k: v + act_ms for k, v in ms.items()}
    return {"sizes": sizes, "activation": act, "macs": macs, "params": params, "weight_bytes": 4 * params,
            "activations": n_act,
            "esp32s3": {"mhz": ESP32["mhz"], "cycles_per_mac": dict(ESP32["cycles_per_mac"]),
                        "act_cycles": ACT_CYCLES.get(act), "mac_ms": ms, "activation_ms": act_ms, "total_ms": tot,
                        "estimate_ms": tot["ansi_c"], "limit_ms": A7_MS, "pass": bool(tot["ansi_c"] < A7_MS),
                        "note": "aktivasyon cevrimi olculmedi (varsayim); kabul tahmini ANSI C toplami"}}


# --- C uretimi ------------------------------------------------------------------------------------------
def _cf(v):
    v = float(v)
    if not np.isfinite(v):
        raise ValueError("sonlu olmayan agirlik %r" % v)
    s = "%.9g" % v
    if "e" not in s and "." not in s:
        s += ".0"
    return s + "f"


def _carray(name, arr, per_line=8):
    flat = np.asarray(arr, np.float32).reshape(-1)
    rows = [", ".join(_cf(v) for v in flat[i:i + per_line]) for i in range(0, len(flat), per_line)]
    return "static const float %s[%d] __attribute__((aligned(16))) = {\n    %s\n};\n" % (
        name, len(flat), ",\n    ".join(rows))


def _header(sha):
    return "/* Otomatik uretildi: rl/export_c.py v%d, ONNX sha256 %s. Elle duzenlemeyin. */\n" % (TOOL_VERSION, sha)


def c_sources(folded, sha):
    """{dosya: metin} policy_weights.h, policy.h, policy.c."""
    sizes = [folded[0][0].shape[1]] + [W.shape[0] for W, _, _, _ in folded]
    w = [_header(sha), "#ifndef OPENDUCK_POLICY_WEIGHTS_H\n#define OPENDUCK_POLICY_WEIGHTS_H\n\n"]
    for k, (W, b, act, alpha) in enumerate(folded):
        w.append("/* katman %d: %d -> %d, %s%s; W satir oncelikli [cikis][girdi] */\n" % (
            k, W.shape[1], W.shape[0], act or "dogrusal", " (normalizasyon katlanmis)" if k == 0 else ""))
        w.append(_carray("OPENDUCK_W%d" % k, W))
        w.append(_carray("OPENDUCK_B%d" % k, b))
        w.append("\n")
    w.append("#endif\n")
    h = [_header(sha), "#ifndef OPENDUCK_POLICY_H\n#define OPENDUCK_POLICY_H\n\n",
         "#define OPENDUCK_POLICY_OBS_DIM %d\n#define OPENDUCK_POLICY_ACT_DIM %d\n#define OPENDUCK_POLICY_N_LAYERS %d\n\n"
         % (sizes[0], sizes[-1], len(folded)),
         "#ifdef __cplusplus\nextern \"C\" {\n#endif\n\n",
         "/* obs: OBS_DIM ham gozlem (normalizasyon agirliklarda), action: ACT_DIM ham aksiyon (kirpmasiz; kirpma\n"
         "   ACTION_CLIP ile calisma zamaninda). Statik ara bellek kullanir: yeniden girisli degil. */\n",
         "void openduck_policy(const float *obs, float *action);\n\n",
         "#ifdef __cplusplus\n}\n#endif\n\n#endif\n"]
    acts = sorted({a for _, _, a, _ in folded if a})
    c = [_header(sha), "#include <math.h>\n\n#include \"policy.h\"\n#include \"policy_weights.h\"\n\n"]
    for k, n in enumerate(sizes[1:-1]):
        c.append("static float openduck_h%d[%d];\n" % (k, n))
    c.append("\n/* y = W x + b; W satir oncelikli (n_out x n_in); toplama sirasi sabit, FMA yok (-ffp-contract=off) */\n"
             "static void openduck_dense(const float *w, const float *b, const float *x, float *y, int n_out, int n_in)\n"
             "{\n    int o, i;\n    for (o = 0; o < n_out; ++o) {\n        const float *wr = w + o * n_in;\n"
             "        float acc = 0.0f;\n        for (i = 0; i < n_in; ++i)\n            acc += wr[i] * x[i];\n"
             "        y[o] = acc + b[o];\n    }\n}\n")
    if "elu" in acts:
        c.append("\n/* ELU: x > 0 ? x : alpha * (exp(x) - 1) */\nstatic void openduck_elu(float *x, int n, float alpha)\n"
                 "{\n    int i;\n    for (i = 0; i < n; ++i)\n        if (!(x[i] > 0.0f))\n"
                 "            x[i] = alpha * expm1f(x[i]);\n}\n")
    if "swish" in acts:
        c.append("\n/* swish: x * sigmoid(x) = x / (1 + exp(-x)) */\nstatic void openduck_swish(float *x, int n)\n"
                 "{\n    int i;\n    for (i = 0; i < n; ++i)\n        x[i] = x[i] / (1.0f + expf(-x[i]));\n}\n")
    if "relu" in acts:
        c.append("\nstatic void openduck_relu(float *x, int n)\n{\n    int i;\n    for (i = 0; i < n; ++i)\n"
                 "        if (x[i] < 0.0f)\n            x[i] = 0.0f;\n}\n")
    if "tanh" in acts:
        c.append("\nstatic void openduck_tanh(float *x, int n)\n{\n    int i;\n    for (i = 0; i < n; ++i)\n"
                 "        x[i] = tanhf(x[i]);\n}\n")
    c.append("\nvoid openduck_policy(const float *obs, float *action)\n{\n")
    for k, (W, b, act, alpha) in enumerate(folded):
        src = "obs" if k == 0 else "openduck_h%d" % (k - 1)
        dst = "action" if k == len(folded) - 1 else "openduck_h%d" % k
        c.append("    openduck_dense(OPENDUCK_W%d, OPENDUCK_B%d, %s, %s, %d, %d);\n" % (
            k, k, src, dst, W.shape[0], W.shape[1]))
        if act == "elu":
            c.append("    openduck_elu(%s, %d, %s);\n" % (dst, W.shape[0], _cf(alpha)))
        elif act:
            c.append("    openduck_%s(%s, %d);\n" % (act, dst, W.shape[0]))
    c.append("}\n")
    return {"policy_weights.h": "".join(w), "policy.h": "".join(h), "policy.c": "".join(c)}


def _cname(s):
    return "".join(ch if ch.isalnum() else "_" for ch in str(s)).upper()


def obs_layout_header(meta, graph, sha):
    """obs_layout.h metni (meta yoksa yalniz boyutlar)."""
    L = [_header(sha), "#ifndef OPENDUCK_OBS_LAYOUT_H\n#define OPENDUCK_OBS_LAYOUT_H\n\n"]
    L.append("#define OBS_DIM %d\n#define ACT_DIM %d\n" % (graph["obs_dim"], graph["act_dim"]))
    if meta is None:
        L.append("/* meta verilmedi: yalniz boyutlar */\n\n#endif\n")
        return "".join(L)
    task = meta.get("task", "walk")
    L.append("#define OPENDUCK_TASK \"%s\"\n" % task)
    if meta.get("robot"):
        L.append("#define OPENDUCK_ROBOT \"%s\"\n" % meta["robot"])
    L.append("#define CTRL_DT %s\n#define ACTION_CLIP %s\n" % (_cf(meta["ctrl_dt"]), _cf(meta["action_clip"])))
    if meta.get("obs_clip") is not None:
        L.append("#define OBS_CLIP %s\n" % _cf(meta["obs_clip"]))
    for k, v in sorted((meta.get("obs_scales") or {}).items()):
        L.append("#define OBS_SCALE_%s %s\n" % (_cname(k), _cf(v)))
    lay = meta.get("obs_layout")
    if lay is None:
        from . import tasks as TK
        lay = TK.get_task(task).obs_layout
    L.append("\n/* gozlem duzeni (rl/tasks.py obs_layout) */\n")
    o = 0
    for name, n in lay:
        L.append("#define OBS_%s_OFFSET %d\n#define OBS_%s_SIZE %d\n" % (_cname(name), o, _cname(name), int(n)))
        o += int(n)
    if o != graph["obs_dim"]:
        raise ValueError("meta obs_layout toplami %d, ONNX gozlemi %d" % (o, graph["obs_dim"]))
    sc = meta["action_scale"]
    sc = [float(sc)] * graph["act_dim"] if np.isscalar(sc) else [float(v) for v in sc]
    L.append("\n#define ACTION_SCALE_UNIFORM %d\n" % int(len(set(sc)) == 1))
    arrs = [("OPENDUCK_ACTION_SCALE", sc), ("OPENDUCK_DEFAULT_ACTION_Q", meta.get("default_action_q")
                                            or meta.get("default_leg_q")),
            ("OPENDUCK_DEFAULT_OBS_Q", meta.get("default_obs_q"))]
    for name, v in arrs:
        if v is not None:
            L.append("static const float %s[%d] = {%s};\n" % (name, len(v), ", ".join(_cf(x) for x in v)))
    if meta.get("action_lo") is not None:
        for name, k in (("OPENDUCK_ACTION_LO", "action_lo"), ("OPENDUCK_ACTION_HI", "action_hi")):
            L.append("static const float %s[%d] = {%s};\n" % (name, len(meta[k]), ", ".join(_cf(x) for x in meta[k])))
    if meta.get("action_mode"):
        L.append("#define OPENDUCK_ACTION_MODE \"%s\"\n" % meta["action_mode"])
    if meta.get("actuators"):
        L.append(robot_block(meta))
    L.append("\n#endif\n")
    return "".join(L)


ROBOT_ACT_COLUMNS = ("tau_s", "omega0", "kp", "deadband", "pwm_res", "ctrl_lo", "ctrl_hi")


def robot_block(meta):
    """tr35 meta'si (rl/policy_meta.py: actuators = actuators.actuator_table, derived_source, derived_sha256, kappa,
    head_tuck_deg, posture_ref.seat_pitch_deg) -> obs_layout.h robot sabitleri (aktuator sirasi; birimler meta
    actuators.units: rad, N.m)."""
    a = meta["actuators"]
    n = len(a["names"])
    L = ["\n/* robot sabitleri (policy_meta: %s; aktuator sirasi) */\n" % meta.get("robot"),
         "#define OPENDUCK_NU %d\n" % n]
    src = meta.get("derived_source") or {}
    if src:
        L.append("#define OPENDUCK_DERIVED_MJCF_SHA256 \"%s\"\n#define OPENDUCK_DERIVED_STUB %d\n"
                 % (src.get("mjcf_sha256"), int(bool(src.get("stub")))))
    if meta.get("derived_sha256"):
        L.append("#define OPENDUCK_DERIVED_SHA256 \"%s\"\n" % meta["derived_sha256"])
    if meta.get("kappa") is not None:
        L.append("#define OPENDUCK_KAPPA %s\n" % _cf(meta["kappa"]))
    names = ", ".join('"%s"' % x for x in a["names"])
    L.append("static const char *const OPENDUCK_ACT_NAMES[%d] = {%s};\n" % (n, names))
    for k in ROBOT_ACT_COLUMNS:
        if len(a[k]) != n:
            raise ValueError("meta actuators.%s %d deger, %d aktuator" % (k, len(a[k]), n))
        L.append("static const float OPENDUCK_ACT_%s[%d] = {%s};\n" % (_cname(k), n, ", ".join(_cf(x) for x in a[k])))
    L.append("static const unsigned char OPENDUCK_ACT_BOARD[%d] = {%s};\n"
             % (n, ", ".join(str(int(b)) for b in a["board"])))
    for k, b in sorted((a.get("boards") or {}).items()):
        L.append("#define OPENDUCK_BOARD%s_PWM_HZ %d\n#define OPENDUCK_BOARD%s_PCA %s\n"
                 % (k, int(b["pwm_hz"]), k, b["pca"]))
    tuck = meta.get("head_tuck_deg")
    if tuck:
        hj = [j for j in meta["obs_joints"] if j in tuck]
        L.append("static const float OPENDUCK_HEAD_TUCK_RAD[%d] = {%s};  /* %s */\n"
                 % (len(hj), ", ".join(_cf(np.deg2rad(tuck[j])) for j in hj), ", ".join(hj)))
    pr = meta.get("posture_ref") or {}
    if "seat_pitch_deg" in pr:
        L.append("#define OPENDUCK_SEAT_PITCH_DEG %s\n" % _cf(pr["seat_pitch_deg"]))
    return "".join(L)


# --- disa aktarma ve dogrulama --------------------------------------------------------------------------
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for blk in iter(lambda: f.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()


def default_meta_path(onnx_path):
    """export_onnx kurali: <ad>.onnx -> <ad>_meta.json."""
    return os.path.splitext(onnx_path)[0] + "_meta.json"


def find_cc(cc=None):
    for c in (cc, os.environ.get("CC"), shutil.which("cc"), "/usr/bin/cc"):
        if c and (os.path.isfile(c) or shutil.which(c)):
            return c
    return None


def check_obs(graph, n, seed=0):
    """(n, d) float32 dogrulama gozlemleri: uc degerler + normalizator olcekli Gauss (m + 3 s z)."""
    d = graph["obs_dim"]
    m = graph["mean"] if graph["mean"] is not None else np.zeros(d)
    s = graph["denom"] if graph["denom"] is not None else np.ones(d)
    alt = np.where(np.arange(d) % 2 == 0, 1.0, -1.0)
    edge = [np.zeros(d), m, m + 4 * s, m - 4 * s, m + 4 * s * alt, m - 4 * s * alt]
    for j in range(d):
        for sg in (6.0, -6.0):
            e = m.copy()
            e[j] += sg * s[j]
            edge.append(e)
    edge = np.array(edge)[:n]
    rng = np.random.default_rng(seed)
    g = m + 3.0 * s * rng.standard_normal((n - len(edge), d))
    return np.concatenate([edge, g]).astype(np.float32)


HARNESS = """#include <stdio.h>
#include <stdlib.h>
#include "policy.h"

int main(int argc, char **argv)
{
    float obs[OPENDUCK_POLICY_OBS_DIM], act[OPENDUCK_POLICY_ACT_DIM];
    long k, n;
    FILE *fi, *fo;
    if (argc != 4)
        return 2;
    n = atol(argv[3]);
    fi = fopen(argv[1], "rb");
    fo = fopen(argv[2], "wb");
    if (!fi || !fo)
        return 3;
    for (k = 0; k < n; ++k) {
        if (fread(obs, sizeof(float), OPENDUCK_POLICY_OBS_DIM, fi) != OPENDUCK_POLICY_OBS_DIM)
            return 4;
        openduck_policy(obs, act);
        if (fwrite(act, sizeof(float), OPENDUCK_POLICY_ACT_DIM, fo) != OPENDUCK_POLICY_ACT_DIM)
            return 5;
    }
    fclose(fi);
    fclose(fo);
    return 0;
}
"""


def check(out_dir, onnx_path, graph=None, n=MIN_CHECK, seed=0, atol=ATOL, cc=None):
    """C kodunu derleyip n gozlemde onnxruntime ile karsilastirir. Dondurur {"n", "max_abs_diff", "atol", "equal",
    "max_abs_action", "max_abs_diff_ref64" (C - float64 numpy), "compiler", "cflags", "warnings", "host_us"}."""
    import onnxruntime as ort
    n = int(n)
    if n < MIN_CHECK:
        raise ValueError("--check %d: spec A7a en az %d gozlem ister" % (n, MIN_CHECK))
    graph = graph or load_graph(onnx_path)
    cc = find_cc(cc)
    if cc is None:
        raise RuntimeError("C derleyicisi yok (cc / $CC)")
    x = check_obs(graph, n, seed)
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    ref = sess.run([graph["output"]], {graph["input"]: x})[0].astype(np.float64)
    tmp = tempfile.mkdtemp(prefix="openduck_c_")
    try:
        with open(os.path.join(tmp, "harness.c"), "w") as f:
            f.write(HARNESS)
        exe = os.path.join(tmp, "harness")
        cmd = [cc] + CFLAGS + ["-I", out_dir, "-o", exe, os.path.join(tmp, "harness.c"),
                               os.path.join(out_dir, "policy.c"), "-lm"]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        if r.returncode:
            raise RuntimeError("derleme basarisiz (%d): %s" % (r.returncode, (r.stderr or r.stdout)[-2000:]))
        fi, fo = os.path.join(tmp, "obs.bin"), os.path.join(tmp, "act.bin")
        x.tofile(fi)
        t0 = time.time()
        r2 = subprocess.run([exe, fi, fo, str(n)], capture_output=True, text=True, timeout=600)
        dt = time.time() - t0
        if r2.returncode:
            raise RuntimeError("test programi kod %d ile cikti" % r2.returncode)
        y = np.fromfile(fo, dtype=np.float32).reshape(n, graph["act_dim"]).astype(np.float64)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    diff = float(np.max(np.abs(y - ref)))
    d64 = float(np.max(np.abs(y - forward(graph, x))))
    warn = [l for l in (r.stderr or "").splitlines() if "warning" in l]
    return {"n": n, "seed": int(seed), "max_abs_diff": diff, "atol": float(atol), "equal": bool(diff <= atol),
            "max_abs_action": float(np.max(np.abs(ref))), "max_abs_diff_ref64": d64, "compiler": cc,
            "cflags": list(CFLAGS) + ["-lm"], "warnings": warn, "host_us_per_call": 1e6 * dt / n}


def export(onnx_path, out_dir, meta=None, check_n=0, seed=0, atol=ATOL, cc=None):
    """C dosyalarini + export_c.json'u yazar; check_n > 0 ise derleyip dogrular. meta: sozluk, yol ya da None
    (None: <ad>_meta.json varsa o). Dondurur rapor sozlugu (export_c.json icerigi)."""
    onnx_path = os.path.abspath(onnx_path)
    meta_path = None
    if meta is None and os.path.exists(default_meta_path(onnx_path)):
        meta = default_meta_path(onnx_path)
    if isinstance(meta, str):
        meta_path = os.path.abspath(meta)
        with open(meta_path) as f:
            meta = json.load(f)
    graph = load_graph(onnx_path)
    if meta is not None and (meta.get("obs_dim"), meta.get("action_dim")) != (graph["obs_dim"], graph["act_dim"]):
        raise ValueError("meta %s/%s, ONNX %d/%d" % (meta.get("obs_dim"), meta.get("action_dim"), graph["obs_dim"],
                                                   graph["act_dim"]))
    sha = sha256_file(onnx_path)
    folded = fold(graph)
    src = c_sources(folded, sha)
    src["obs_layout.h"] = obs_layout_header(meta, graph, sha)
    os.makedirs(out_dir, exist_ok=True)
    for name in FILES:
        p = os.path.join(out_dir, name)
        with open(p + ".tmp", "w") as f:
            f.write(src[name])
        os.replace(p + ".tmp", p)
    sizes = [graph["obs_dim"]] + [L["W"].shape[0] for L in graph["layers"]]
    acts = sorted({L["act"] for L in graph["layers"] if L["act"]})
    rep = {"tool": "rl/export_c.py", "tool_version": TOOL_VERSION, "onnx": onnx_path, "onnx_sha256": sha,
           "meta": meta_path, "task": (meta or {}).get("task", "walk" if meta else None),
           "robot": (meta or {}).get("robot"), "normalization": "folded" if (graph["mean"] is not None or
                                                                            graph["denom"] is not None) else "none",
           "layers": [{"in": int(L["W"].shape[1]), "out": int(L["W"].shape[0]), "activation": L["act"],
                       "alpha": L["alpha"]} for L in graph["layers"]],
           "hidden": sizes[1:-1]}
    if (meta or {}).get("robot"):
        rep.update(derived_source=meta.get("derived_source"), derived_sha256=meta.get("derived_sha256"),
                   kappa=meta.get("kappa"))
    rep.update(size_report(sizes, acts[0] if len(acts) == 1 else (acts[0] if acts else "relu")))
    rep["files"] = {name: hashlib.sha256(src[name].encode()).hexdigest() for name in FILES}
    rep["check"] = check(out_dir, onnx_path, graph, check_n, seed, atol, cc) if check_n else None
    with open(os.path.join(out_dir, "export_c.json.tmp"), "w") as f:
        json.dump(rep, f, indent=1, sort_keys=True)
        f.write("\n")
    os.replace(os.path.join(out_dir, "export_c.json.tmp"), os.path.join(out_dir, "export_c.json"))
    return rep


def safe_export(onnx_path, out_dir, check_n=MIN_CHECK, log=print):
    """Egiticiler icin (tr35): export + (derleyici varsa) dogrulama; hata egitimi durdurmaz, yalniz yazdirilir.
    Dondurur rapor ya da None."""
    try:
        n = check_n if (check_n and find_cc() is not None) else 0
        rep = export(onnx_path, out_dir, check_n=n)
        c = rep["check"]
        log("export_c: %s (%s, %.2f ms ESP32-S3 tahmini%s)" % (
            out_dir, "-".join(str(s) for s in rep["sizes"]), rep["esp32s3"]["estimate_ms"],
            ", C/ORT %.1e" % c["max_abs_diff"] if c else ", dogrulanmadi"))
        if c and not c["equal"]:
            log("UYARI export_c: C/ORT farki %.3g > %.1g" % (c["max_abs_diff"], c["atol"]))
        return rep
    except Exception as e:      # egitim C disa aktarma yuzunden durmasin
        log("UYARI export_c basarisiz (%s): %r" % (onnx_path, e))
        return None


def main(argv=None):
    p = argparse.ArgumentParser(description="policy.onnx -> C99 float32 (normalizasyon katlanmis) + host esitlik kontrolu")
    p.add_argument("--onnx", required=True)
    p.add_argument("--meta", default=None, help="policy_meta.json (varsayilan: <ad>_meta.json varsa)")
    p.add_argument("--out", required=True, help="cikti klasoru, or. runs/X/c")
    p.add_argument("--check", type=int, default=0, metavar="N",
                   help="derle + N (>= %d) gozlemde onnxruntime ile karsilastir; fark > --atol ise cikis 1" % MIN_CHECK)
    p.add_argument("--atol", type=float, default=ATOL)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--cc", default=None, help="C derleyicisi (varsayilan $CC, cc, /usr/bin/cc)")
    a = p.parse_args(argv)
    if a.check and a.check < MIN_CHECK:
        p.error("--check en az %d (spec A7a)" % MIN_CHECK)
    rep = export(a.onnx, a.out, a.meta, a.check, a.seed, a.atol, a.cc)
    e = rep["esp32s3"]
    print("%s -> %s: %s, %d MAC, %d parametre (%.1f KB), ESP32-S3 ~%.2f ms (ESP-DSP %.2f ms)" % (
        a.onnx, a.out, "-".join(str(s) for s in rep["sizes"]), rep["macs"], rep["params"],
        rep["weight_bytes"] / 1024.0, e["estimate_ms"], e["total_ms"]["esp_dsp"]))
    c = rep["check"]
    if c is None:
        return 0
    print("check: %d gozlem, max |C - ORT| %.3g (sinir %.1g) -> %s" % (
        c["n"], c["max_abs_diff"], c["atol"], "ESIT" if c["equal"] else "FARKLI"))
    return 0 if c["equal"] else 1


if __name__ == "__main__":
    sys.exit(main())
