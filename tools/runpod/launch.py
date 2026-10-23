#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenDuck egitimini RunPod'da calistirir (yalnizca stdlib). API anahtari: RUNPOD_API_KEY ortam degiskeni.

  python3 tools/runpod/launch.py gpus [--cpu] [--all]
  python3 tools/runpod/launch.py launch [--backend mjx|cpu] [--source inline|git] [--gpu ID ...]
          [--robot xl330_27|tr35_b2 [--kappa K]]
          [--max-hours H] [--wait [--keep]] [--dry-run [--full]] [-- EGITIM_ARGUMANLARI ...]
  python3 tools/runpod/launch.py status    [NAME | --pod-id ID] [--json]
  python3 tools/runpod/launch.py fetch     [NAME | --pod-id ID] [--all]
  python3 tools/runpod/launch.py logs      [NAME | --pod-id ID] [--tail N]
  python3 tools/runpod/launch.py terminate [NAME | --pod-id ID | --all-openduck]
  python3 tools/runpod/launch.py list

Pod tarafi tools/runpod/pod_entry.sh: once https://<pod>-<port>.proxy.runpod.net/<token>/ adresinde
/workspace/out'u (status.json, manifest.json, entry.log, train.log, run/) sunar, sonra kodu alir,
bagimliliklari kurar, egitimi sure siniriyla calistirir ve bitince sunmaya devam eder.
Yerel kayit runs/<name>/runpod/pod.json; fetch egitim ciktilarini (pod'daki run/) runs/<name>/ altina,
pod dosyalarini (status.json, loglar) runs/<name>/runpod/ altina indirir.
API: varsayilan REST v2 (https://api.runpod.io/v2); --api v1 eski REST (rest.runpod.io/v1, 15.11.2026'da kapaniyor).
--robot tr35_b2: pod ortamina OPENDUCK_ROBOT=tr35_b2 (--kappa -> OPENDUCK_KAPPA) eklenir; satir ici paket / git
denetimi build/sim yerine build/tr35_b2/sim/{openduck,scene}.xml ve rl/derived/tr35_b2.json ister. Varsayilan
xl330_27 istek govdesi bugunkuyle ayni.
"""
import argparse
import base64
import glob
import gzip
import hashlib
import io
import json
import os
import re
import secrets
import shutil
import socket
import subprocess
import sys
import tarfile
import time
import urllib.error
import urllib.parse
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUNS = os.path.join(ROOT, "runs")
ENTRY = os.path.join(ROOT, "tools", "runpod", "pod_entry.sh")
API = {"v2": "https://api.runpod.io/v2", "v1": "https://rest.runpod.io/v1"}
UA = "openduck-runpod-launch/1.0"   # rest.runpod.io varsayilan Python-urllib UA'sini reddediyor (Cloudflare 1010)
DEFAULT_IMAGE = "runpod/base:1.0.7-ubuntu2404"   # python3.12 + uv + git, CUDA'siz (jax CUDA'yi pip getirir)
DEFAULT_GPUS = ["NVIDIA GeForce RTX 4090", "NVIDIA GeForce RTX 5090", "NVIDIA L40S",
                "NVIDIA RTX 6000 Ada Generation", "NVIDIA GeForce RTX 3090", "NVIDIA RTX A6000", "NVIDIA A40"]
DEFAULT_CPU_FLAVORS = ["cpu5c", "cpu3c"]
DEFAULT_PORT = 8765
BODY_LIMIT = 100000      # v2 POST /pods 102400 bayttan buyuk govdeyi 413 ile reddeder; biraz pay
BODY_WARN = 80000
MAX_ENV = 50             # RunPod: pod basina en fazla 50 ortam degiskeni
NEEDED = ["rl/__init__.py", "rl/config.py", "rl/robot_model.py", "rl/task_math.py",
          "build/sim/openduck.xml", "build/sim/scene.xml"]
TRAIN_MODULE = {"mjx": "rl/train_mjx.py", "cpu": "rl/train.py"}
# satir ici paket: egitim modulunun rl/ icindeki import kapanisi + bunlar (rl/play.py, testler gitmez)
INLINE_EXTRA = ["rl/__init__.py", "rl/robot_model.py", "build/sim/openduck.xml", "build/sim/scene.xml",
                "tools/runpod/pod_entry.sh"]
# robot profili (--robot -> pod'da OPENDUCK_ROBOT). tr35_b2: xl330 build dosyalari yerine TR-35 MJCF + sahne + turetilmis
# sabitler (rl/config.py SCENE_PATHS / DERIVED_PATHS varsayilanlari; pod'da OPENDUCK_SCENE / OPENDUCK_DERIVED yok)
ROBOTS = ("xl330_27", "tr35_b2")
XL330_FILES = ["build/sim/openduck.xml", "build/sim/scene.xml"]
TR35_FILES = ["build/tr35_b2/sim/openduck.xml", "build/tr35_b2/sim/scene.xml", "rl/derived/tr35_b2.json"]
SIM_DIRS = {"xl330_27": "build/sim", "tr35_b2": "build/tr35_b2/sim"}


def _robot_files(files, robot):
    """xl330 listesi aynen; tr35_b2: xl330 build dosyalari yerine TR35_FILES."""
    if robot in (None, ROBOTS[0]):
        return list(files)
    return [f for f in files if f not in XL330_FILES] + TR35_FILES


def needed(robot=None):
    return _robot_files(NEEDED, robot)


def inline_extra(robot=None):
    return _robot_files(INLINE_EXTRA, robot)
TERMINAL = ("done", "failed", "timeout", "stopped")
CUDA_V1 = ["13.0", "12.9", "12.8", "12.7", "12.6", "12.5", "12.4", "12.3", "12.2", "12.1", "12.0", "11.8"]
CKPT_EXT = (".pt", ".pth", ".ckpt", ".pkl", ".npz", ".msgpack", ".safetensors")
POD_PREFIX = "openduck"  # RunPod'daki pod adi hep bununla baslar (terminate --all-openduck)
# konteyner komutu (bash -c): pod_entry.sh'i ortam degiskeninden cikarip calistirir. Cikaramazsa
# yeniden baslama dongusune girmemek icin bekler, 30 dk sonra podu silmeyi dener.
BOOT = ('D=/workspace/.openduck; mkdir -p "$D" && '
        'if [ -n "${OPENDUCK_CODE_B64:-}" ]; then '
        'printf %s "$OPENDUCK_CODE_B64" | base64 -d | tar -xzO tools/runpod/pod_entry.sh > "$D/pod_entry.sh"; '
        'else printf %s "${OPENDUCK_ENTRY_B64:-}" | base64 -d | gzip -dc > "$D/pod_entry.sh"; fi '
        '&& [ -s "$D/pod_entry.sh" ] && exec bash "$D/pod_entry.sh"; '
        'echo "openduck: pod_entry.sh acilamadi, 30 dk sonra pod silinecek"; sleep 1800; '
        'runpodctl pod delete "${RUNPOD_POD_ID:-}" || runpodctl remove pod "${RUNPOD_POD_ID:-}"; sleep infinity')


def die(msg, code=2):
    print("HATA: " + msg, file=sys.stderr)
    sys.exit(code)


def warn(msg):
    print("UYARI: " + msg, file=sys.stderr)


def fmt_dur(s):
    if s is None:
        return "-"
    s = int(s)
    if s >= 3600:
        return "%dsa%02ddk" % (s // 3600, s % 3600 // 60)
    if s >= 60:
        return "%ddk%02dsn" % (s // 60, s % 60)
    return "%dsn" % s


def si(x):
    if x is None:
        return "?"
    x = float(x)
    for d, u in ((1e9, "G"), (1e6, "M"), (1e3, "k")):
        if abs(x) >= d:
            return "%.2f%s" % (x / d, u)
    return "%.0f" % x


def fnum(x):
    return ("%g" % x) if isinstance(x, float) else str(x)


# --- HTTP ----------------------------------------------------------------------------------------
class HttpError(Exception):
    def __init__(self, status, text, url, headers=None):
        Exception.__init__(self, "HTTP %d %s" % (status, url))
        self.status, self.text, self.url = status, text or "", url
        self.headers = headers or {}

    def retry_after(self, default):
        try:
            return max(1.0, min(120.0, float(self.headers.get("Retry-After"))))
        except (TypeError, ValueError):
            return default

    def detail(self):
        try:
            d = json.loads(self.text)
        except ValueError:
            return self.text.strip()[:500] or "(bos yanit)"
        if isinstance(d, dict):
            parts = [str(d[k]) for k in ("title", "detail", "error", "message") if d.get(k)]
            parts += [str(e) for e in d.get("errors") or []]
            return "; ".join(parts) or json.dumps(d)[:500]
        return json.dumps(d)[:500]


def http(method, url, body=None, headers=None, timeout=60):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header("User-Agent", UA)
    req.add_header("Accept", "application/json")
    if data is not None:
        req.add_header("Content-Type", "application/json")
    for k, v in (headers or {}).items():
        req.add_header(k, v)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            status, raw = r.status, r.read()
    except urllib.error.HTTPError as e:
        raise HttpError(e.code, e.read().decode("utf-8", "replace"), url, dict(e.headers or {}))
    if not raw.strip():
        return status, None
    try:
        return status, json.loads(raw)
    except ValueError:
        return status, raw.decode("utf-8", "replace")


def short_strings(obj, limit=1000):
    if isinstance(obj, dict):
        return {k: short_strings(v, limit) for k, v in obj.items()}
    if isinstance(obj, list):
        return [short_strings(v, limit) for v in obj]
    if isinstance(obj, str) and len(obj) > limit:
        return "<%d karakter, sha256 %s..., tamami icin --full>" % (
            len(obj), hashlib.sha256(obj.encode()).hexdigest()[:16])
    return obj


def show_request(method, url, body=None, full=False):
    print("%s %s" % (method, url))
    print("Authorization: Bearer <RUNPOD_API_KEY gizlendi>")
    print("User-Agent: %s" % UA)
    if body is not None:
        print("Content-Type: application/json\n")
        print(json.dumps(body if full else short_strings(body), indent=2, ensure_ascii=False))
    print()


class Api(object):
    def __init__(self, version, key=None, dry=False, full=False):
        self.v, self.key, self.dry, self.full = version, key, dry, full
        self.base = API[version]

    def call(self, method, path, body=None, query=None, base=None, timeout=60, retries=None):
        url = (base or self.base) + path
        if query:
            url += "?" + urllib.parse.urlencode(query)
        if self.dry:
            show_request(method, url, body, self.full)
            return None
        if not self.key:
            die("RUNPOD_API_KEY tanimli degil (README: API anahtari)")
        tries = retries if retries is not None else (3 if method == "GET" else 1)
        for i in range(tries):
            try:
                return http(method, url, body, {"Authorization": "Bearer " + self.key}, timeout)[1]
            except HttpError as e:
                if i + 1 < tries and (e.status == 429 or e.status >= 500):
                    time.sleep(e.retry_after(5 * (i + 1)))
                    continue
                raise
            except (urllib.error.URLError, socket.timeout, ConnectionError):
                if i + 1 < tries:
                    time.sleep(5 * (i + 1))
                    continue
                raise

    def get_pod(self, pid):
        return norm_pod(self.call("GET", "/pods/" + pid), self.v)

    def list_pods(self):
        r = self.call("GET", "/pods")
        items = (r or {}).get("pods", []) if self.v == "v2" else (r or [])
        return [norm_pod(p, self.v) for p in items]

    def delete_pod(self, pid):
        self.call("DELETE", "/pods/" + pid, retries=3)


def parse_time(s):
    if not s:
        return None
    s = str(s).replace(" +0000 UTC", "Z").replace(" ", "T")
    for f in ("%Y-%m-%dT%H:%M:%S.%fZ", "%Y-%m-%dT%H:%M:%SZ", "%Y-%m-%dT%H:%M:%S.%f", "%Y-%m-%dT%H:%M:%S"):
        try:
            return time.mktime(time.strptime(s, f)) - time.timezone
        except ValueError:
            pass
    return None


def norm_pod(p, v):
    p = p or {}
    if v == "v2":
        hw = p.get("gpu") or p.get("cpu") or {}
        return {"id": p.get("id"), "name": p.get("name"), "status": p.get("status"), "cost": p.get("cost"),
                "hw": hw.get("id"), "uptime": (p.get("runtime") or {}).get("uptime"), "env": p.get("env") or {},
                "dc": p.get("dataCenterId"), "image": p.get("image")}
    m = p.get("machine") or {}
    t = parse_time(p.get("lastStartedAt"))
    up = time.time() - t if (t and p.get("desiredStatus") == "RUNNING") else None
    return {"id": p.get("id"), "name": p.get("name"), "status": p.get("desiredStatus"),
            "cost": p.get("adjustedCostPerHr") or p.get("costPerHr"),
            "hw": (p.get("gpu") or {}).get("id") or m.get("gpuTypeId") or p.get("cpuFlavorId"),
            "uptime": up, "env": p.get("env") or {}, "dc": m.get("dataCenterId"), "image": p.get("image")}


def get_key(a, required=True):
    k = os.environ.get("RUNPOD_API_KEY", "").strip()
    if not k and getattr(a, "api_key_file", None):
        with open(a.api_key_file) as f:
            k = f.read().strip()
    if not k and required and not a.dry_run:
        die("RUNPOD_API_KEY tanimli degil: export RUNPOD_API_KEY=... (README)")
    return k


# --- yerel kayit ---------------------------------------------------------------------------------
def rec_path(name):
    return os.path.join(RUNS, name, "runpod", "pod.json")


def save_rec(rec):
    p = rec_path(rec["name"])
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p + ".tmp", "w") as f:
        json.dump(rec, f, indent=1, sort_keys=True)
    os.replace(p + ".tmp", p)


def all_recs():
    out = []
    for p in glob.glob(os.path.join(RUNS, "*", "runpod", "pod.json")):
        try:
            with open(p) as f:
                out.append(json.load(f))
        except (OSError, ValueError):
            pass
    return sorted(out, key=lambda r: r.get("created_at", 0))


def find_rec(a, api):
    if getattr(a, "pod_id", None):
        rec = next((r for r in all_recs() if r.get("pod_id") == a.pod_id), None) or {"pod_id": a.pod_id}
    elif getattr(a, "name", None):
        if not os.path.exists(rec_path(a.name)):
            die("kayit yok: %s (--pod-id ile deneyin)" % rec_path(a.name))
        with open(rec_path(a.name)) as f:
            rec = json.load(f)
    else:
        recs = all_recs()
        if not recs:
            die("runs/*/runpod/pod.json yok; NAME ya da --pod-id verin")
        rec = recs[-1]
        print("son kayit: %s (%s)" % (rec["name"], rec["pod_id"]))
    if ("http_token" not in rec or "name" not in rec) and not api.dry and api.key:
        try:
            pod = api.get_pod(rec["pod_id"])
            env = pod["env"]
            rec.setdefault("name", env.get("OPENDUCK_RUN_NAME") or pod["name"] or rec["pod_id"])
            rec.setdefault("port", int(env.get("OPENDUCK_PORT") or DEFAULT_PORT))
            rec.setdefault("http_token", env.get("OPENDUCK_HTTP_TOKEN", ""))
        except (HttpError, urllib.error.URLError, socket.timeout) as e:
            warn("pod bilgisi alinamadi: %s" % (e.detail() if isinstance(e, HttpError) else e))
    rec.setdefault("name", rec["pod_id"])
    rec.setdefault("port", DEFAULT_PORT)
    rec.setdefault("http_token", "")
    return rec


def base_url(rec):
    tok = rec.get("http_token") or ""
    return "https://%s-%d.proxy.runpod.net/%s" % (rec["pod_id"], int(rec.get("port") or DEFAULT_PORT),
                                                    tok + "/" if tok else "")


def http_status(rec, timeout=30):
    try:
        _, st = http("GET", base_url(rec) + "status.json", timeout=timeout)
    except HttpError as e:
        return None, "HTTP %d" % e.status
    except (urllib.error.URLError, socket.timeout, ConnectionError, OSError) as e:
        return None, str(getattr(e, "reason", e))
    return (st, None) if isinstance(st, dict) else (None, "status.json okunamadi")


# --- kod paketi ----------------------------------------------------------------------------------
def rl_imports(start):
    """start dosyalarindan rl/ icindeki goreli importlarin (from .x, from . import x, rl.x) gecisli kapanisi."""
    seen, todo = set(), list(start)
    pats = (r"^\s*from\s+\.(\w*)\s+import\s+([^\n#]+)", r"^\s*from\s+rl(?:\.(\w+))?\s+import\s+([^\n#]+)",
            r"^\s*import\s+rl\.(\w+)()")
    while todo:
        f = todo.pop()
        if f in seen or not os.path.isfile(os.path.join(ROOT, f)):
            continue
        seen.add(f)
        if not f.endswith(".py"):
            continue
        with open(os.path.join(ROOT, f), encoding="utf-8") as fh:
            src = fh.read()
        for pat in pats:
            for mod, names in re.findall(pat, src, re.M):
                mods = [mod] if mod else [n.split()[0] for n in names.strip("() \\").split(",") if n.strip()]
                todo += ["rl/%s.py" % m for m in mods]
    return seen


def build_inline(backend, strict=True, robot=None):
    files = rl_imports([TRAIN_MODULE[backend]] + inline_extra(robot))
    files.update(os.path.relpath(p, ROOT).replace(os.sep, "/")
                 for p in glob.glob(os.path.join(ROOT, "rl", "requirements-*.txt")))
    missing = [f for f in needed(robot) + ["tools/runpod/pod_entry.sh", TRAIN_MODULE[backend]] if f not in files]
    if missing:
        (die if strict else warn)("satir ici paket icin eksik dosya: %s" % ", ".join(missing))
    buf = io.BytesIO()
    raw = 0
    with gzip.GzipFile(fileobj=buf, mode="wb", compresslevel=9, mtime=0) as gz:
        with tarfile.open(fileobj=gz, mode="w", format=tarfile.GNU_FORMAT) as tar:
            for rel in sorted(files):
                path = os.path.join(ROOT, rel)
                ti = tar.gettarinfo(path, arcname=rel)
                ti.uid = ti.gid = 0
                ti.uname = ti.gname = ""
                ti.mode = 0o755 if rel.endswith(".sh") else 0o644
                raw += ti.size
                with open(path, "rb") as f:
                    tar.addfile(ti, f)
    return buf.getvalue(), sorted(files), raw


def git(*args, timeout=120):
    return subprocess.run(["git", "-C", ROOT] + list(args), capture_output=True, text=True, timeout=timeout)


def git_info(a):
    repo = a.git_repo
    if not repo:
        url = git("remote", "get-url", "origin").stdout.strip()
        m = re.search(r"github\.com[:/]+([^/]+/[^/]+?)(?:\.git)?/?$", url)
        repo = m.group(1) if m else "Kubudak90/openduck"
    ref = a.git_ref or git("rev-parse", "--abbrev-ref", "HEAD").stdout.strip()
    if not ref or ref == "HEAD":
        die("dal bulunamadi (detached HEAD?): --git-ref verin")
    try:
        r = git("fetch", "--quiet", "origin", ref, timeout=90)
        if r.returncode:
            warn("git fetch origin %s basarisiz: %s" % (ref, r.stderr.strip()[:200]))
    except subprocess.TimeoutExpired:
        warn("git fetch zaman asimi; yerel origin/%s kullaniliyor" % ref)
    r = git("rev-parse", "--verify", "origin/%s^{commit}" % ref)
    if r.returncode:
        (warn if a.dry_run else die)("origin/%s yok: dali push edin ya da --source inline kullanin" % ref)
        return {"repo": repo, "ref": ref, "commit": ""}
    commit = r.stdout.strip()
    robot = getattr(a, "robot", None)
    missing = [f for f in needed(robot) + [TRAIN_MODULE[a.backend]]
               if git("cat-file", "-e", "%s:%s" % (commit, f)).returncode]
    if missing:
        (warn if a.dry_run else die)("origin/%s (%s) icinde eksik: %s -> commit+push ya da --source inline" % (
            ref, commit[:10], ", ".join(missing)))
    if git("rev-parse", "HEAD").stdout.strip() != commit:
        warn("yerel HEAD origin/%s ile ayni degil; pod %s commit'ini calistirir" % (ref, commit[:10]))
    sim = SIM_DIRS.get(robot or ROBOTS[0], "build/sim")
    dirty = git("status", "--porcelain", "--", "rl", sim).stdout.rstrip()
    if dirty:
        warn("rl/ ya da %s'de commit edilmemis degisiklik var; pod bunlari gormez:\n  " % sim +
             dirty.replace("\n", "\n  "))
    return {"repo": repo, "ref": ref, "commit": commit}


# --- istek govdeleri -----------------------------------------------------------------------------
def pod_env(a, name, token, train_args, linger, expires, code=None, gitd=None):
    env = {"OPENDUCK_BACKEND": a.backend, "OPENDUCK_SOURCE": a.source, "OPENDUCK_RUN_NAME": name,
           "OPENDUCK_PORT": str(a.port), "OPENDUCK_HTTP_TOKEN": token, "OPENDUCK_MAX_HOURS": fnum(a.max_hours),
           "OPENDUCK_SETUP_HOURS": fnum(a.setup_hours), "OPENDUCK_LINGER_HOURS": fnum(linger),
           "OPENDUCK_TRAIN_ARGS": json.dumps(train_args)}
    if expires:
        env["OPENDUCK_EXPIRES_AT"] = str(int(expires))
    if a.pip:
        env["OPENDUCK_PIP_EXTRA"] = " ".join(a.pip)
    if a.python:
        env["OPENDUCK_PYTHON"] = a.python
    if a.ssh:
        env["OPENDUCK_START_SH"] = "1"
    if code is not None:
        env["OPENDUCK_CODE_B64"] = base64.b64encode(code).decode()
        env["OPENDUCK_CODE_SHA256"] = hashlib.sha256(code).hexdigest()
    else:
        with open(ENTRY, "rb") as f:
            env["OPENDUCK_ENTRY_B64"] = base64.b64encode(gzip.compress(f.read(), 9, mtime=0)).decode()
        env.update(OPENDUCK_GIT_REPO=gitd["repo"], OPENDUCK_GIT_REF=gitd["ref"], OPENDUCK_GIT_COMMIT=gitd["commit"],
                   OPENDUCK_SECRET_NAME=a.secret, GITHUB_TOKEN="{{ RUNPOD_SECRET_%s }}" % a.secret)
    robot = getattr(a, "robot", None)
    if robot not in (None, ROBOTS[0]):        # xl330 govdesi bugunkuyle ayni (anahtar eklenmez)
        env["OPENDUCK_ROBOT"] = robot
        if getattr(a, "kappa", None) is not None:
            env["OPENDUCK_KAPPA"] = fnum(float(a.kappa))
    return env


def vtuple(v):
    return tuple(int(x) for x in v.split("."))


def create_bodies(a, name, env, gpus, clouds, flavors):
    ports = ["%d/http" % a.port] + (["22/tcp"] if a.ssh else [])
    entry, cmd = ["bash", "-c"], [BOOT]      # imajin ENTRYPOINT'i ne olursa olsun bash -c BOOT
    out = []
    if a.api == "v2":
        base = {"name": name, "image": a.image, "disk": a.disk_gb, "ports": ports, "env": env,
                "entrypoint": entry, "cmd": cmd}
        if a.ssh:
            base["startSsh"] = True
        if a.compute == "gpu":
            for g in gpus:
                for c in clouds:
                    gpu = {"id": g, "count": a.gpu_count}
                    if a.min_cuda:
                        gpu["minCudaVersion"] = a.min_cuda
                    b = dict(base, cloud=c, gpu=gpu)
                    if a.volume_gb:
                        b["mounts"] = {"persistent": {"size": a.volume_gb, "path": "/workspace"}}
                    out.append(("%s / %s" % (g, c), b))
        else:
            for f in flavors:
                out.append(("%s x %d vCPU / SECURE" % (f, a.vcpu), dict(base, cloud="SECURE", cpu={"id": f, "vcpuCount": a.vcpu})))
    else:
        base = {"name": name, "imageName": a.image, "containerDiskInGb": a.disk_gb,
                "volumeInGb": a.volume_gb if a.compute == "gpu" else 0, "volumeMountPath": "/workspace",
                "ports": ports, "env": env, "dockerEntrypoint": entry, "dockerStartCmd": cmd}
        if a.compute == "gpu":
            for c in clouds:
                b = dict(base, cloudType=c, computeType="GPU", gpuCount=a.gpu_count, gpuTypeIds=list(gpus),
                         gpuTypePriority="custom")
                if a.min_cuda:
                    b["allowedCudaVersions"] = [v for v in CUDA_V1 if vtuple(v) >= vtuple(a.min_cuda)]
                out.append(("%s / %s" % (" > ".join(gpus), c), b))
        else:
            out.append(("%s x %d vCPU / SECURE" % (" > ".join(flavors), a.vcpu),
                        dict(base, cloudType="SECURE", computeType="CPU", cpuFlavorIds=list(flavors),
                             cpuFlavorPriority="custom", vcpuCount=a.vcpu)))
    return out


def gpu_catalog(api, cloud, min_cuda=None):
    q = {"include": "AVAILABILITY", "product": "POD", "cloud": cloud}
    if min_cuda:
        q["minCudaVersion"] = min_cuda
    r = api.call("GET", "/catalog/gpus", query=q)
    return {g["id"]: g for g in (r or {}).get("gpus", [])}


def resolve_names(names, catalog_ids, catalog_names, defaults):
    out = []
    for n in names:
        lo = n.lower()
        hit = [i for i in catalog_ids + defaults if i.lower() == lo]
        hit = hit or [i for i, nm in catalog_names.items() if nm.lower() == lo]
        if not hit:
            hit = sorted({i for i in catalog_ids + defaults if lo in i.lower()}, key=len)
            if len(hit) > 1:
                warn("%r birden cok tipe uyuyor (%s); %r secildi" % (n, ", ".join(hit), hit[0]))
        out.append(hit[0] if hit else n)
    return list(dict.fromkeys(out))


# --- komutlar ------------------------------------------------------------------------------------
def cmd_launch(a, train_args):
    if a.max_hours <= 0:
        die("--max-hours > 0 olmali")
    if any(x == "--out" or x.startswith("--out=") for x in train_args):
        die("--out egitim argumani launch tarafindan verilir (pod'da /workspace/out/run)")
    tr35 = a.robot != ROBOTS[0]
    if a.kappa is not None and not tr35:
        die("--kappa yalniz --robot tr35_b2 ile (OPENDUCK_KAPPA)")
    if a.kappa is not None and not a.kappa > 0:
        die("--kappa > 0 olmali")
    name = a.name or "%s-%s-%s" % (POD_PREFIX, a.backend + ("-tr35" if tr35 else ""), time.strftime("%Y%m%d-%H%M%S"))
    if not re.match(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,90}$", name):
        die("gecersiz --name: %r" % name)
    pod_name = name if name.startswith(POD_PREFIX) else "%s-%s" % (POD_PREFIX, name)
    if not a.dry_run and os.path.exists(rec_path(name)):
        die("%s zaten var; baska --name verin" % rec_path(name))
    a.compute = a.compute or ("gpu" if a.backend == "mjx" else "cpu")
    if a.backend == "mjx" and a.compute == "cpu":
        warn("mjx backend CPU podda cok yavas olur")
    if a.vcpu < 2 or a.vcpu & (a.vcpu - 1):
        die("--vcpu 2'nin kuvveti olmali (RunPod CPU pod kurali)")
    if a.min_cuda and not re.match(r"^\d+\.\d+$", a.min_cuda):
        die("--min-cuda bicimi major.minor olmali, or. 12.4")
    if a.volume_gb and (a.volume_gb < 10 or a.compute == "cpu"):
        die("--volume-gb: en az 10 ve yalniz GPU podda")
    if a.setup_hours <= 0:
        die("--setup-hours > 0 olmali")
    linger = a.linger_hours if a.linger_hours is not None else (0.0 if a.keep else 2.0)
    # pod bu andan sonra kendini siler (konteyner yeniden baslasa da); linger 0 ise sinir yok
    expires = time.time() + (a.setup_hours + a.max_hours + linger + 0.5) * 3600 if linger > 0 else None
    clouds = [c.strip().upper() for c in a.cloud.split(",") if c.strip()]
    if any(c not in ("SECURE", "COMMUNITY") for c in clouds):
        die("--cloud: SECURE, COMMUNITY ya da SECURE,COMMUNITY")
    gpus = [g.strip() for x in (a.gpu or DEFAULT_GPUS) for g in x.split(",") if g.strip()]
    flavors = [f.strip() for x in (a.cpu_flavor or DEFAULT_CPU_FLAVORS) for f in x.split(",") if f.strip()]
    key = get_key(a)
    api = Api(a.api, key, dry=a.dry_run, full=a.full)
    cat_api = Api("v2", key)
    code = gitd = None
    if a.source == "inline":
        code, files, raw = build_inline(a.backend, strict=not a.dry_run, robot=a.robot)
    else:
        gitd = git_info(a)
    if tr35:
        print("robot: %s (pod OPENDUCK_ROBOT%s); gerekli dosyalar: %s" % (
            a.robot, ", OPENDUCK_KAPPA=%s" % fnum(float(a.kappa)) if a.kappa is not None else "",
            ", ".join("%s%s" % (f, "" if os.path.isfile(os.path.join(ROOT, f)) else " (YOK)") for f in TR35_FILES)))

    catalogs = {}
    if a.compute == "gpu" and not a.dry_run:
        for c in clouds:
            try:
                catalogs[c] = gpu_catalog(cat_api, c, a.min_cuda)
            except (HttpError, urllib.error.URLError, socket.timeout) as e:
                warn("GPU katalogu alinamadi (%s): %s" % (c, e.detail() if isinstance(e, HttpError) else e))
    ids = sorted({i for cat in catalogs.values() for i in cat})
    names = {i: g.get("name", "") for cat in catalogs.values() for i, g in cat.items()}
    gpus = resolve_names(gpus, ids, names, DEFAULT_GPUS)
    if a.compute == "cpu" and not a.dry_run:
        try:
            r = cat_api.call("GET", "/catalog/cpus", query={"include": "AVAILABILITY", "product": "POD"})
            cids = [c["id"] for c in (r or {}).get("cpus", [])]
            flavors = resolve_names(flavors, cids, {}, DEFAULT_CPU_FLAVORS)
            for c in (r or {}).get("cpus", []):
                if c["id"] in flavors and not (c["vcpu"]["min"] <= a.vcpu <= c["vcpu"]["max"]):
                    warn("%s icin vCPU araligi %d..%d (istenen %d)" % (c["id"], c["vcpu"]["min"], c["vcpu"]["max"], a.vcpu))
        except (HttpError, urllib.error.URLError, socket.timeout, KeyError, TypeError) as e:
            warn("CPU katalogu alinamadi: %r" % (e,))

    token = secrets.token_hex(8)
    env = pod_env(a, name, token, train_args, linger, expires, code, gitd)
    if len(env) > MAX_ENV:
        die("%d ortam degiskeni > %d" % (len(env), MAX_ENV))
    cands = create_bodies(a, pod_name, env, gpus, clouds, flavors)
    size = max(len(json.dumps(b).encode()) for _, b in cands)
    if code is not None:
        b64 = len(env["OPENDUCK_CODE_B64"])
        print("satir ici paket: %d dosya, %.1f KB ham -> %.1f KB tar.gz -> %.1f KB base64 (OPENDUCK_CODE_B64)" % (
            len(files), raw / 1024.0, len(code) / 1024.0, b64 / 1024.0))
    print("istek govdesi: %.1f KB (RunPod siniri ~100 KB)" % (size / 1024.0))
    if expires:
        print("pod en gec %s'de kendini siler (OPENDUCK_EXPIRES_AT)" % time.strftime("%d.%m %H:%M", time.localtime(expires)))
    oversize = size > BODY_LIMIT
    if oversize:
        # dry-run istegi yine gosterir (cikis kodu 1); gercek launch reddedilir
        (warn if a.dry_run else die)("istek govdesi %d bayt > %d: satir ici paket cok buyuk; --source git kullanin"
                                     % (size, BODY_LIMIT))
    elif size > BODY_WARN:
        warn("istek govdesi sinira yakin (%d bayt)" % size)

    if a.dry_run:
        print("\n=== DRY RUN: gonderilecek istek (1/%d) ===" % len(cands))
        show_request("POST", api.base + "/pods", cands[0][1], a.full)
        if len(cands) > 1:
            print("kapasite yoksa sirayla ayni govde su degisikliklerle denenir:")
            for label, _ in cands[1:]:
                print("  - " + label)
        if code is not None:
            with tarfile.open(fileobj=io.BytesIO(code), mode="r:gz") as t:
                names_in = t.getnames()
            print("\npaket icerigi (%d): %s" % (len(names_in), " ".join(names_in)))
        print("\nHTTP: https://<podId>-%d.proxy.runpod.net/%s/status.json" % (a.port, token))
        if oversize:
            print("\nUYARI: govde %d bayt > %d; gercek launch bu govdeyi gondermez (--source git)" % (size, BODY_LIMIT))
        return 1 if oversize else 0

    if a.source == "git":
        try:
            r = cat_api.call("GET", "/account/secrets", query={"name": a.secret})
            if isinstance(r, dict) and not any(s.get("name", "").lower() == a.secret.lower() for s in r.get("secrets", [])):
                die("RunPod secret '%s' yok: console > Secrets'ta olusturun (README)" % a.secret)
        except HttpError as e:
            warn("secret listesi okunamadi (%s); '%s' secret'inin var oldugu varsayiliyor" % (e.status, a.secret))
        except (urllib.error.URLError, socket.timeout) as e:
            warn("secret listesi okunamadi: %s" % e)

    def in_stock(body):
        if a.api != "v2" or a.compute != "gpu" or body["cloud"] not in catalogs:
            return True
        g = catalogs[body["cloud"]].get(body["gpu"]["id"])
        return g is not None and g.get("availability") != "NONE" and g.get(body["cloud"].lower(), True)
    stocked = [c for c in cands if in_stock(c[1])]
    if len(stocked) < len(cands):
        print("katalogda stok yok (en sona birakildi): " + ", ".join(l for l, b in cands if not in_stock(b)))
    def by_name():
        # sunucu hatasi/zaman asiminda pod yine de olusmus olabilir; ad bu calismaya ozgu
        time.sleep(5)
        try:
            return next((q for q in api.list_pods() if q["name"] == pod_name and q["status"] != "TERMINATED"), None)
        except (HttpError, urllib.error.URLError, socket.timeout):
            return None

    # RunPod v2 belgesi: 400 (kural ihlali ya da yer yok) ve 403 (havuza erisim yok) -> sonraki aday;
    # 401/402/413/422 -> dur; 429 -> Retry-After; 5xx -> ayni adayi tekrar dene
    p = label = last = None
    for label, body in stocked + [c for c in cands if not in_stock(c[1])]:
        for attempt in range(4):
            try:
                print("pod olusturuluyor: %s ..." % label, flush=True)
                p = norm_pod(api.call("POST", "/pods", body, timeout=120), a.api)
                if not p["id"]:
                    p = by_name() or die("pod yaniti beklenmedik; launch.py list ile kontrol edin", 1)
                break
            except HttpError as e:
                last = e
                if e.status == 429 and attempt < 3:
                    time.sleep(e.retry_after(10))
                    continue
                if e.status >= 500 or e.status == 429:
                    p = by_name()
                    if p or attempt >= 2:
                        if not p:
                            print("  sunucu hatasi (%d): %s" % (e.status, e.detail()[:200]))
                        break
                    time.sleep(5 * 2 ** attempt)
                    continue
                if e.status == 401:
                    die("yetki hatasi (401): %s -- RUNPOD_API_KEY dogru mu?" % e.detail(), 1)
                if e.status == 402:
                    die("bakiye yetersiz (402): %s" % e.detail(), 1)
                if e.status in (413, 422):
                    die("istek reddedildi (%d): %s" % (e.status, e.detail()), 1)
                if e.status in (400, 403, 404, 409):
                    print("  yerlestirilemedi (%d): %s" % (e.status, e.detail()[:200]))
                    break
                die("pod olusturulamadi (%d): %s" % (e.status, e.detail()), 1)
            except (urllib.error.URLError, socket.timeout, ConnectionError) as e:
                p = by_name() or die("RunPod API'ye ulasilamadi: %s (pod olusmus olabilir: launch.py list)" % e, 1)
                break
        if p:
            break
    if not p:
        hint = ""
        if last is not None and last.status == 403:
            hint = " 403: anahtarin pod yazma yetkisi (All / pods:write) var mi?"
        elif last is not None and last.status == 400:
            hint = " Hepsi 400 ise sorun istekte de olabilir (son ayrinti yukarida)."
        die("hicbir secenekte pod acilamadi;%s --gpu/--cloud degistirin ya da sonra deneyin (launch.py gpus)" % hint, 1)

    rec = {"name": name, "pod_name": pod_name, "pod_id": p["id"], "api": a.api, "backend": a.backend,
           "compute": a.compute, "source": a.source, "choice": label, "hw": p["hw"], "cost_per_hr": p["cost"],
           "port": a.port, "http_token": token, "created_at": time.time(), "max_hours": a.max_hours,
           "setup_hours": a.setup_hours, "linger_hours": linger, "expires_at": expires, "train_args": train_args,
           "image": a.image, "code_sha256": env.get("OPENDUCK_CODE_SHA256"), "git": gitd}
    if tr35:
        rec.update(robot=a.robot, kappa=a.kappa)
    save_rec(rec)
    cost = p["cost"]
    print("pod: %s  (%s, %s $/saat)" % (p["id"], label, cost if cost is not None else "?"))
    if cost:
        print("maliyet: tipik ~%.2f $ (kurulum + %g saat egitim%s); ust sinir %s" % (
            float(cost) * (a.max_hours + 0.25 + (0 if a.wait else linger)), a.max_hours,
            "" if a.wait else " + %g saat bekleme" % linger,
            "~%.2f $ (pod suresi)" % (float(cost) * (expires - time.time()) / 3600.0) if expires else
            "YOK (--linger-hours 0 / --keep: podu siz silmelisiniz)"))
    print("durum: %sstatus.json" % base_url(rec))
    print("kayit: %s" % rec_path(name))
    print("komutlar: launch.py status %s | fetch %s | terminate %s" % (name, name, name))
    if a.wait:
        return wait_loop(api, rec, a)
    return 0


def progress_line(st):
    parts = [str(st.get("phase"))]
    if st.get("progress") is not None:
        parts.append("%%%.1f" % (100.0 * st["progress"]))
    if st.get("steps") is not None:
        parts.append("%s/%s adim" % (si(st["steps"]), si(st.get("total_steps"))))
    if st.get("eta_s"):
        parts.append("kalan ~" + fmt_dur(st["eta_s"]))
    if st.get("error"):
        parts.append("HATA: " + str(st["error"]))
    line = st.get("last_log_line") or ""
    return " ".join(parts) + (" | " + line[:140] if line else "")


def wait_loop(api, rec, a):
    t0 = time.time()
    hard = (a.max_hours + a.setup_hours + 0.75) * 3600.0
    boot = a.boot_timeout_min * 60.0
    last_print, last_key, down_since, outcome, pod, missing = 0.0, None, t0, None, None, 0
    seen_http = bad_status = 0
    print("bekleniyor (her %ds; Ctrl-C beklemeyi birakir, pod calismaya devam eder)" % a.poll)
    try:
        while True:
            now = time.time()
            try:
                pod = api.get_pod(rec["pod_id"])
                missing = 0
            except HttpError as e:
                missing += e.status == 404
                if missing >= 3:
                    outcome = "pod_gone"
                    break
                warn("API: %s" % e.detail()[:200])
            except (urllib.error.URLError, socket.timeout, ConnectionError) as e:
                warn("API: %s" % e)
            if pod and pod["status"] == "TERMINATED":
                outcome = "pod_gone"
                break
            st, err = http_status(rec)
            el = fmt_dur(now - t0)
            if st:
                down_since, seen_http = None, 1
                key = (st.get("phase"), st.get("steps"), st.get("error"))
                if key != last_key or now - last_print > 600:
                    print("[%s] %s" % (el, progress_line(st)), flush=True)
                    last_key, last_print = key, now
                if st.get("phase") in TERMINAL:
                    outcome = st["phase"]
                    break
            else:
                down_since = down_since or now
                # ERROR: kurtarilamaz; EXITED: durdurulmus (konteyner diski silinir)
                bad_status = bad_status + 1 if pod and pod["status"] in ("ERROR", "EXITED") else 0
                if now - last_print > 120:
                    print("[%s] pod %s, HTTP yok (%s)" % (el, pod["status"] if pod else "?", err), flush=True)
                    last_print = now
                if bad_status >= 3:
                    outcome = "pod_" + pod["status"].lower()
                    break
                if now - down_since > boot * (2 if seen_http else 1):
                    outcome = "http_lost" if seen_http else "boot_timeout"
                    break
            if now - t0 > hard:
                outcome = "deadline"
                break
            time.sleep(a.poll)
    except KeyboardInterrupt:
        print("\nbekleme birakildi; pod calisiyor: launch.py status/fetch/terminate %s" % rec["name"])
        return 130

    print("sonuc: %s" % outcome)
    if outcome == "pod_gone":
        print("pod artik yok (silinmis)")
        return 1
    try:
        return finish(api, rec, a, outcome)
    except KeyboardInterrupt:
        print("\nyarida kesildi; pod SILINMEMIS olabilir: launch.py terminate %s" % rec["name"], file=sys.stderr)
        return 130


def finish(api, rec, a, outcome):
    # pod HTTP'ye hic cikmadiysa ya da bozuksa indirilecek bir sey yok: logu goster, sil.
    # Aksi halde once indir; indirilemezse silme (pod OPENDUCK_EXPIRES_AT'te kendini siler).
    if outcome in ("boot_timeout", "pod_error", "pod_exited"):
        print("pod HTTP'ye cikmadi (%s); RunPod loglari:" % outcome)
        for line in pod_logs(api.key, rec["pod_id"], 60):
            print("  " + line)
        fetched = True
    else:
        st = None
        for i in range(4):
            st, _ = http_status(rec)
            if do_fetch(rec, everything=a.fetch_all) is not None:
                break
            if i < 3:
                time.sleep(30)
        policy = os.path.exists(os.path.join(RUNS, rec["name"], "policy.onnx"))
        fetched = st is not None and (policy or not st.get("has_policy"))
        if st and st.get("phase") == "failed":
            print("egitim logunun sonu:")
            for line in st.get("log_tail") or []:
                print("  " + line)
    if a.keep:
        print("--keep: pod calismaya devam ediyor ve UCRETLENIYOR (%s); silmek icin: launch.py terminate %s" % (
            rec["pod_id"], rec["name"]))
    elif not fetched:
        exp = rec.get("expires_at")
        print("UYARI: ciktilar indirilemedi; pod SILINMEDI. Tekrar: launch.py fetch %s && launch.py terminate %s%s" % (
            rec["name"], rec["name"], " (pod %s'de kendini siler)" % time.strftime("%d.%m %H:%M", time.localtime(exp))
            if exp else ""), file=sys.stderr)
    else:
        terminate(api, rec)
    if outcome in ("boot_timeout", "pod_error", "pod_exited", "http_lost"):
        return 1
    ok = os.path.exists(os.path.join(RUNS, rec["name"], "policy.onnx"))
    return 0 if outcome in ("done", "timeout", "deadline") and ok else 1


def terminate(api, rec):
    try:
        api.delete_pod(rec["pod_id"])
        print("pod silindi: %s" % rec["pod_id"])
    except HttpError as e:
        if e.status == 404:
            print("pod zaten yok: %s" % rec["pod_id"])
        else:
            print("SILINEMEDI (%d): %s -- RunPod console'dan silin!" % (e.status, e.detail()), file=sys.stderr)
            return False
    except (urllib.error.URLError, socket.timeout) as e:
        print("SILINEMEDI: %s -- tekrar deneyin: launch.py terminate --pod-id %s" % (e, rec["pod_id"]), file=sys.stderr)
        return False
    if os.path.exists(rec_path(rec.get("name", ""))):
        rec["terminated_at"] = time.time()
        save_rec(rec)
    return True


def select_files(files, everything=False):
    if everything:
        return list(files)
    keep, groups = [], {}
    for f in files:
        parts = f["path"].split("/")
        key = None
        for i, part in enumerate(parts[:-1]):
            if part.isdigit():
                key = ("d", "/".join(parts[:i]), int(part))
                break
        if key is None:
            m = re.match(r"^(.*?)(\d+)(\.[A-Za-z0-9]+)$", parts[-1])
            if m and m.group(3).lower() in CKPT_EXT:
                key = ("f", "/".join(parts[:-1] + [m.group(1) + "#" + m.group(3)]), int(m.group(2)))
        if key is None:
            keep.append(f)
        else:
            groups.setdefault(key[:2], []).append((key[2], f))
    for items in groups.values():
        top = max(n for n, _ in items)
        keep += [f for n, f in items if n == top]
    return sorted(keep, key=lambda f: f["path"])


def download(url, dest):
    tmp = dest + ".part"
    err = None
    for i in range(3):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            with urllib.request.urlopen(req, timeout=120) as r, open(tmp, "wb") as f:
                shutil.copyfileobj(r, f, 1 << 20)
            os.replace(tmp, dest)
            return
        except (urllib.error.URLError, socket.timeout, ConnectionError, OSError) as e:
            err = e
            time.sleep(3 * (i + 1))
    raise err


def do_fetch(rec, everything=False):
    base = base_url(rec)
    try:
        _, man = http("GET", base + "manifest.json", timeout=60)
    except (HttpError, urllib.error.URLError, socket.timeout, OSError) as e:
        print("manifest.json alinamadi (%s): pod hazir degil ya da silinmis" % e, file=sys.stderr)
        return None
    if not isinstance(man, dict) or not isinstance(man.get("files"), list):
        print("manifest.json gecersiz", file=sys.stderr)
        return None
    dest = os.path.realpath(os.path.join(RUNS, rec["name"]))
    files = select_files(man["files"], everything)
    n = skipped = nbytes = 0
    for f in files:
        rel = str(f.get("path", ""))
        local = os.path.realpath(os.path.join(dest, rel[4:]) if rel.startswith("run/") else
                                 os.path.join(dest, "runpod", rel))
        if rel.startswith("/") or not local.startswith(dest + os.sep) or local == os.path.realpath(rec_path(rec["name"])):
            warn("atlandi: %r" % rel)
            continue
        if os.path.exists(local) and os.path.getsize(local) == f.get("size") and \
                abs(os.path.getmtime(local) - float(f.get("mtime", 0))) < 2:
            skipped += 1
            continue
        os.makedirs(os.path.dirname(local), exist_ok=True)
        try:
            download(base + urllib.parse.quote(rel), local)
        except Exception as e:
            warn("indirilemedi %s: %s" % (rel, e))
            continue
        if f.get("mtime"):
            os.utime(local, (float(f["mtime"]), float(f["mtime"])))
        n += 1
        nbytes += os.path.getsize(local)
    print("indirildi: %d dosya (%.1f MB), degismeyen %d, secilmeyen %d -> %s" % (
        n, nbytes / 1e6, skipped, len(man["files"]) - len(files), os.path.relpath(dest)))
    for fn in ("policy.onnx", "policy_meta.json"):
        p = os.path.join(dest, fn)
        print("  %s %s" % (os.path.relpath(p), "OK" if os.path.exists(p) else "yok"))
    return dest


def pod_logs(key, pid, tail=100, idle=8, total=30):
    url = API["v2"] + "/pods/%s/logs?%s" % (pid, urllib.parse.urlencode({"tail": tail}))
    req = urllib.request.Request(url, headers={"Authorization": "Bearer " + (key or ""), "User-Agent": UA,
                                               "Accept": "text/event-stream"})
    lines = []
    try:
        with urllib.request.urlopen(req, timeout=idle) as r:
            t0 = time.time()
            while time.time() - t0 < total and len(lines) < tail:
                try:
                    raw = r.readline()
                except (socket.timeout, TimeoutError, OSError):
                    break
                if not raw:
                    break
                s = raw.decode("utf-8", "replace").strip()
                if s.startswith("data:"):
                    try:
                        d = json.loads(s[5:].strip())
                        lines.append("%-9s %s" % (d.get("source", ""), d.get("line", "")))
                    except ValueError:
                        lines.append(s[5:].strip())
    except urllib.error.HTTPError as e:
        lines.append("(log alinamadi: HTTP %d %s)" % (e.code, e.read().decode("utf-8", "replace")[:200]))
    except (urllib.error.URLError, socket.timeout, OSError) as e:
        lines.append("(log alinamadi: %s)" % e)
    return lines


def cmd_status(a, _):
    api = Api(a.api, get_key(a, required=False), dry=a.dry_run)
    rec = find_rec(a, api)
    if a.dry_run:
        api.get_pod(rec["pod_id"])
        print("GET %sstatus.json" % base_url(rec))
        return 0
    pod, err_api = None, "RUNPOD_API_KEY yok"
    try:
        if api.key:
            pod = api.get_pod(rec["pod_id"])
    except HttpError as e:
        err_api = "pod bulunamadi (silinmis?)" if e.status == 404 else "HTTP %d %s" % (e.status, e.detail()[:200])
    except (urllib.error.URLError, socket.timeout) as e:
        err_api = str(e)
    st, err = http_status(rec)
    if a.json:
        if pod:
            pod = {k: v for k, v in pod.items() if k != "env"}
        print(json.dumps({"record": rec, "pod": pod, "pod_error": err_api, "status": st, "http_error": err},
                         indent=1, default=str))
        return 0 if st or pod else 1
    print("%s  pod %s%s" % (rec["name"], rec["pod_id"], "  (silindi)" if rec.get("terminated_at") else ""))
    if pod:
        cost = pod["cost"]
        spent = (" ~%.2f $" % (float(cost) * pod["uptime"] / 3600.0)) if (cost and pod["uptime"]) else ""
        print("  RunPod: %s  %s  %s $/saat  uptime %s%s" % (pod["status"], pod["hw"], cost, fmt_dur(pod["uptime"]), spent))
    else:
        print("  RunPod: %s" % err_api)
    print("  HTTP:   %sstatus.json" % base_url(rec))
    if not st:
        print("  durum:  alinamadi (%s)" % err)
        return 1
    print("  faz:    %s" % progress_line(st))
    if st.get("linger_until"):
        print("  bekleme: pod %s icinde kendini silecek" % fmt_dur(float(st["linger_until"]) - time.time()))
    if st.get("expires_at"):
        print("  pod suresi: en gec %s icinde silinir" % fmt_dur(float(st["expires_at"]) - time.time()))
    rec_ = st.get("last_record")
    if isinstance(rec_, dict):
        keys = [k for k in ("iter", "steps", "fps", "reward_per_step", "ep_len", "ep_return", "n_falls") if k in rec_]
        if keys:
            print("  son kayit: " + "  ".join("%s=%s" % (k, rec_[k]) for k in keys))
    for line in (st.get("log_tail") or [])[-a.lines:]:
        print("  | " + line)
    return 0


def cmd_fetch(a, _):
    api = Api(a.api, get_key(a, required=False), dry=a.dry_run)
    rec = find_rec(a, api)
    if a.dry_run:
        print("GET %smanifest.json  (+ secilen dosyalar -> runs/%s/)" % (base_url(rec), rec["name"]))
        return 0
    return 0 if do_fetch(rec, a.all) else 1


def cmd_logs(a, _):
    api = Api(a.api, get_key(a), dry=a.dry_run)
    rec = find_rec(a, api)
    if a.dry_run:
        api.call("GET", "/pods/%s/logs" % rec["pod_id"], query={"tail": a.tail}, base=API["v2"])
        return 0
    for line in pod_logs(api.key, rec["pod_id"], a.tail):
        print(line)
    return 0


def cmd_terminate(a, _):
    api = Api(a.api, get_key(a), dry=a.dry_run)
    if a.all_openduck:
        if a.dry_run:
            api.call("GET", "/pods")
            print("(adi openduck ile baslayan her pod icin) DELETE %s/pods/<id>" % api.base)
            return 0
        mine = {r.get("pod_id") for r in all_recs() if not r.get("terminated_at")}
        pods = [p for p in api.list_pods() if ((p["name"] or "").startswith(POD_PREFIX) or p["id"] in mine)
                and p["status"] != "TERMINATED"]
        if not pods:
            print("silinecek openduck podu yok")
        ok = True
        for p in pods:
            print("%s %s %s" % (p["id"], p["name"], p["status"]))
            ok = terminate(api, next((r for r in all_recs() if r.get("pod_id") == p["id"]), {"pod_id": p["id"]})) and ok
        return 0 if ok else 1
    rec = find_rec(a, api)
    if a.dry_run:
        api.delete_pod(rec["pod_id"])
        return 0
    return 0 if terminate(api, rec) else 1


def cmd_list(a, _):
    api = Api(a.api, get_key(a), dry=a.dry_run)
    pods = api.list_pods()
    if a.dry_run:
        return 0
    print("%-16s %-36s %-12s %-30s %8s %10s" % ("id", "ad", "durum", "donanim", "$/saat", "uptime"))
    for p in pods:
        print("%-16s %-36s %-12s %-30s %8s %10s" % (p["id"], (p["name"] or "")[:36], p["status"], (p["hw"] or "")[:30],
                                                   p["cost"], fmt_dur(p["uptime"])))
    if not pods:
        print("(pod yok)")
    return 0


def cmd_gpus(a, _):
    api = Api("v2", get_key(a), dry=a.dry_run)
    if a.cpu:
        r = api.call("GET", "/catalog/cpus", query={"include": "AVAILABILITY", "product": "POD"})
        if a.dry_run:
            return 0
        print("%-10s %-22s %-8s %-9s %-8s %-10s %s" % ("id", "ad", "grup", "vCPU", "GB/vCPU", "$/vCPU/sa", "stok"))
        for c in sorted((r or {}).get("cpus", []), key=lambda c: (c.get("price") or {}).get("securePerVcpu") or 9):
            print("%-10s %-22s %-8s %-9s %-8s %-10s %s" % (c["id"], c["name"][:22], c.get("group", "")[:8],
                  "%d-%d" % (c["vcpu"]["min"], c["vcpu"]["max"]), c.get("ramGbPerVcpu"),
                  c["price"].get("securePerVcpu"), c.get("availability", "?")))
        return 0
    cats = {c: gpu_catalog(api, c, a.min_cuda) for c in ("SECURE", "COMMUNITY")}
    if a.dry_run:
        return 0
    rows = {}
    for c, cat in cats.items():
        for i, g in cat.items():
            r = rows.setdefault(i, {"g": g, "SECURE": "-", "COMMUNITY": "-"})
            r[c] = g.get("availability", "?") if g.get(c.lower(), True) else "-"
    order = {g: n + 1 for n, g in enumerate(DEFAULT_GPUS)}
    print("%-3s %-42s %5s %9s %9s %-8s %-8s" % ("#", "id (--gpu)", "VRAM", "secure$", "comm.$", "secure", "comm."))
    price = lambda g, c: (g.get("price") or {}).get(c)
    for i, r in sorted(rows.items(), key=lambda kv: (price(kv[1]["g"], "secure") or 99, kv[0])):
        if not a.all and r["SECURE"] in ("-", "NONE") and r["COMMUNITY"] in ("-", "NONE"):
            continue
        g = r["g"]
        print("%-3s %-42s %4sG %9s %9s %-8s %-8s" % (order.get(i, ""), i[:42], g.get("memory", "?"),
              price(g, "secure"), price(g, "community"), r["SECURE"], r["COMMUNITY"]))
    print("# = varsayilan --gpu sirasi; fiyatlar GPU basina $/saat (liste fiyati)")
    return 0


def build_parser():
    p = argparse.ArgumentParser(prog="launch.py", description="OpenDuck egitimini RunPod'da calistir",
                                epilog="Egitim argumanlari '--' sonrasina: launch.py launch -- --num-timesteps 2e8")
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp):
        sp.add_argument("--api", choices=("v2", "v1"), default="v2", help="RunPod REST surumu (v1 15.11.2026'da kapaniyor)")
        sp.add_argument("--api-key-file", help="RUNPOD_API_KEY yerine anahtari bu dosyadan oku")
        sp.add_argument("--dry-run", action="store_true", help="istekleri (anahtar gizli) yazdir ve cik")
        return sp

    def target(sp):
        sp.add_argument("name", nargs="?", help="calisma adi (runs/NAME); bos: en son kayit")
        sp.add_argument("--pod-id")
        return sp

    s = common(sub.add_parser("launch", help="pod olustur ve egitimi baslat"))
    s.add_argument("--backend", choices=("mjx", "cpu"), default="mjx", help="mjx: rl.train_mjx (GPU), cpu: rl.train")
    s.add_argument("--robot", choices=ROBOTS, default=ROBOTS[0],
                   help="robot profili: pod'da OPENDUCK_ROBOT (tr35_b2: TR-35 MJCF + rl/derived/tr35_b2.json paketlenir)")
    s.add_argument("--kappa", type=float, default=None, help="tr35_b2: hiz carpani kappa (pod'da OPENDUCK_KAPPA)")
    s.add_argument("--source", choices=("inline", "git"), default="inline",
                   help="inline: kod ortam degiskeninde tar.gz; git: pod repo'yu GitHub token secret'i ile klonlar")
    s.add_argument("--name", help="calisma/pod adi (varsayilan openduck-<backend>-<zaman>)")
    s.add_argument("--compute", choices=("gpu", "cpu"), help="pod tipi (varsayilan: mjx->gpu, cpu->cpu)")
    s.add_argument("--gpu", action="append", help="GPU tipi, sirayla denenir (tekrarlanabilir / virgullu)")
    s.add_argument("--cloud", default="SECURE,COMMUNITY", help="denenecek bulutlar, sirayla")
    s.add_argument("--gpu-count", type=int, default=1)
    s.add_argument("--min-cuda", default="12.4", help="host CUDA surumu alt siniri ('' = yok)")
    s.add_argument("--cpu-flavor", action="append", help="CPU pod tipi, sirayla (varsayilan cpu5c,cpu3c)")
    s.add_argument("--vcpu", type=int, default=32, help="CPU pod vCPU sayisi (2'nin kuvveti)")
    s.add_argument("--image", default=DEFAULT_IMAGE)
    s.add_argument("--disk-gb", type=int, default=40, help="konteyner diski")
    s.add_argument("--volume-gb", type=int, default=0, help="/workspace kalici hacim (yalniz GPU, 0 = yok)")
    s.add_argument("--port", type=int, default=DEFAULT_PORT)
    s.add_argument("--max-hours", type=float, default=4.0, help="egitim icin sert sure siniri")
    s.add_argument("--setup-hours", type=float, default=1.0, help="kurulum icin sure siniri")
    s.add_argument("--linger-hours", type=float, default=None,
                   help="bitince pod bu kadar sunar sonra kendini siler (0 = asla; varsayilan 2, --keep ile 0)")
    s.add_argument("--pip", action="append", help="ek pip paketi (tekrarlanabilir)")
    s.add_argument("--python", help="pod python surumu (varsayilan 3.12)")
    s.add_argument("--secret", default="github_token", help="--source git: GitHub token'li RunPod secret adi")
    s.add_argument("--git-repo", help="owner/repo (varsayilan origin)")
    s.add_argument("--git-ref", help="dal (varsayilan: mevcut dal, origin'e push edilmis olmali)")
    s.add_argument("--ssh", action="store_true", help="22/tcp ac, startSsh ve imajin /start.sh'i (sshd)")
    s.add_argument("--wait", action="store_true", help="bitene kadar izle, indir, podu sil")
    s.add_argument("--keep", action="store_true", help="--wait sonunda podu silme")
    s.add_argument("--fetch-all", action="store_true", help="--wait: tum checkpoint'leri indir")
    s.add_argument("--poll", type=int, default=30, help="--wait yoklama araligi (sn)")
    s.add_argument("--boot-timeout-min", type=float, default=25.0,
                   help="HTTP bu kadar dakika hic gelmezse podu sil (sonradan kesilirse 2 kati beklenir)")
    s.add_argument("--full", action="store_true", help="--dry-run: uzun degerleri kisaltma")
    s.set_defaults(func=cmd_launch)

    s = target(common(sub.add_parser("status", help="pod ve egitim durumu")))
    s.add_argument("--json", action="store_true")
    s.add_argument("--lines", type=int, default=8, help="gosterilecek log satiri")
    s.set_defaults(func=cmd_status)

    s = target(common(sub.add_parser("fetch", help="ciktilari runs/NAME/ altina indir")))
    s.add_argument("--all", action="store_true", help="tum checkpoint'ler (varsayilan: yalniz en sonuncusu)")
    s.set_defaults(func=cmd_fetch)

    s = target(common(sub.add_parser("logs", help="RunPod konteyner loglari (v2 API)")))
    s.add_argument("--tail", type=int, default=100)
    s.set_defaults(func=cmd_logs)

    s = target(common(sub.add_parser("terminate", help="podu sil (geri alinamaz)")))
    s.add_argument("--all-openduck", action="store_true",
                   help="adi openduck ile baslayan ve runs/*/runpod kayitlarindaki tum podlari sil")
    s.set_defaults(func=cmd_terminate)

    common(sub.add_parser("list", help="hesaptaki podlar")).set_defaults(func=cmd_list)

    s = common(sub.add_parser("gpus", help="GPU (ya da --cpu) tipleri, fiyat ve stok"))
    s.add_argument("--cpu", action="store_true")
    s.add_argument("--all", action="store_true", help="stokta olmayanlari da goster")
    s.add_argument("--min-cuda", default=None)
    s.set_defaults(func=cmd_gpus)
    return p


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    train_args = []
    if "--" in argv:
        i = argv.index("--")
        argv, train_args = argv[:i], argv[i + 1:]
    a = build_parser().parse_args(argv)
    if train_args and a.cmd != "launch":
        die("'--' sonrasi egitim argumanlari yalniz launch icin")
    try:
        return a.func(a, train_args)
    except HttpError as e:
        die("RunPod API hatasi %d: %s" % (e.status, e.detail()), 1)
    except (urllib.error.URLError, socket.timeout, ConnectionError) as e:
        die("ag hatasi: %s" % (getattr(e, "reason", None) or e), 1)


if __name__ == "__main__":
    sys.exit(main())
