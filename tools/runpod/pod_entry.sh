#!/usr/bin/env bash
# OpenDuck egitimi icin RunPod pod giris betigi (tools/runpod/launch.py baslatir).
#
# Sira: once $OUT klasorunu sunan HTTP sunucu (status.json, manifest.json, loglar, run/),
# sonra kod (satir ici tar.gz ya da git clone), python ortami + bagimliliklar, kontrol,
# sert sure sinirli egitim. Bitince sunmaya devam eder; OPENDUCK_LINGER_HOURS sonra podu
# kendisi silmeyi dener (0 = asla). OPENDUCK_EXPIRES_AT (epoch sn) varsa pod o anda her
# durumda kendini siler; konteyner yeniden baslasa da gecerlidir. Betik kendiliginden cikmaz
# (beklenmedik hata dahil): cikarsa RunPod konteyneri yeniden baslatir ve egitim bastan baslar.
#
# Ayarlar ortam degiskenleriyle gelir (launch.py pod_env()). Yerelde ag/kurulum olmadan deneme:
#   OPENDUCK_WORKSPACE=/tmp/x OPENDUCK_CODE_DIR=$PWD OPENDUCK_SKIP_INSTALL=1 \
#   OPENDUCK_TRAIN_CMD='python3 sahte_egitim.py' OPENDUCK_LINGER_HOURS=0 bash tools/runpod/pod_entry.sh
set -uo pipefail

WS=${OPENDUCK_WORKSPACE:-/workspace}
D=$WS/.openduck
OUT=$WS/out
RUN=$OUT/run
CODE=${OPENDUCK_CODE_DIR:-$WS/openduck}
PORT=${OPENDUCK_PORT:-8765}
BACKEND=${OPENDUCK_BACKEND:-mjx}
SOURCE=${OPENDUCK_SOURCE:-inline}
MAX_HOURS=${OPENDUCK_MAX_HOURS:-4}
SETUP_HOURS=${OPENDUCK_SETUP_HOURS:-1}
LINGER_HOURS=${OPENDUCK_LINGER_HOURS:-2}
GRACE_S=${OPENDUCK_GRACE_S:-300}
EXPIRES_AT=${OPENDUCK_EXPIRES_AT:-0}
case "$EXPIRES_AT" in ''|*[!0-9]*) EXPIRES_AT=0 ;; esac
PYVER=${OPENDUCK_PYTHON:-3.12}
# torch (CPU) yalniz cpu backend'e gerekir (rl/ppo.py, rl/export_onnx.py); mjx ONNX'i onnx ile yazar
TORCH_INDEX=${OPENDUCK_TORCH_INDEX:-}
[ -n "$TORCH_INDEX" ] || { [ "$BACKEND" = cpu ] && TORCH_INDEX=https://download.pytorch.org/whl/cpu || TORCH_INDEX=none; }
# rl/requirements-<backend>.txt varsa bu listelerin yerine o kullanilir
PKG_MJX="jax[cuda12]==0.12.0.1 mujoco==3.16.0 mujoco-mjx==3.16.0 brax==0.14.2 playground==0.2.0 onnx onnxruntime"
PKG_CPU="mujoco==3.16.0 onnx onnxruntime"

export OPENDUCK_OUT=$OUT OPENDUCK_D=$D PYTHONUNBUFFERED=1 PIP_DISABLE_PIP_VERSION_CHECK=1 \
       PIP_ROOT_USER_ACTION=ignore PIP_BREAK_SYSTEM_PACKAGES=1 UV_HTTP_TIMEOUT=300
mkdir -p "$D" "$RUN"
exec 3>&1
exec > >(tee -a "$OUT/entry.log") 2>&1

SYSPY=$(command -v python3 || command -v python || true)
TRAIN_PID=""
ERR=""
FINISHED=0

log() { printf '[openduck %s] %s\n' "$(date -u +%H:%M:%S)" "$*"; }
setst() { "$SYSPY" "$D/helper.py" set "$@" >/dev/null || log "UYARI: durum yazilamadi ($*)"; }
hours_to_s() { awk -v h="$1" 'BEGIN { s = int(h * 3600); if (s < 0) s = 0; print s }'; }
snooze() { sleep "$1" & wait $!; }

# --- pod tarafi yardimci (yalnizca stdlib, python>=3.7): HTTP sunucu + status.json -------------
write_helper() {
cat > "$D/helper.py" <<'PYEOF'
import fcntl
import functools
import json
import os
import shlex
import sys
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

OUT = os.environ["OPENDUCK_OUT"]
D = os.environ["OPENDUCK_D"]
RUN = os.path.join(OUT, "run")
STATE = os.path.join(D, "state.json")
STEP_KEYS = ("steps", "num_steps", "env_steps", "total_env_steps", "timesteps", "num_timesteps", "step")
TOTAL_FLAGS = ("--max-steps", "--num-timesteps", "--total-steps", "--timesteps")
TOTAL_ARGS = ("max_steps", "num_timesteps", "total_steps", "timesteps")
DEFAULT_TOTAL = {"cpu": 5e7}


def num(x):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return v if v == v and abs(v) != float("inf") else None


def load(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def dump(path, obj):
    tmp = "%s.%d.tmp" % (path, os.getpid())
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=1, sort_keys=True)
    os.replace(tmp, path)


class Lock(object):
    def __enter__(self):
        self.f = open(STATE + ".lock", "a")
        fcntl.flock(self.f, fcntl.LOCK_EX)
        return self

    def __exit__(self, *exc):
        fcntl.flock(self.f, fcntl.LOCK_UN)
        self.f.close()


def tail(path, n, nbytes=65536):
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - nbytes))
            data = f.read().decode("utf-8", "replace")
    except OSError:
        return []
    lines = [l.rsplit("\r", 1)[-1].rstrip() for l in data.splitlines()]   # ilerleme cubuklari \r kullanir
    return [l for l in lines if l.strip()][-n:]


def last_record():
    best = None
    for root, dirs, files in os.walk(RUN):
        if root[len(RUN):].count(os.sep) >= 2:
            dirs[:] = []
        for fn in files:
            if fn.endswith(".jsonl"):
                p = os.path.join(root, fn)
                try:
                    m = os.path.getmtime(p)
                except OSError:
                    continue
                if best is None or m > best[0]:
                    best = (m, p)
    if best is None:
        return None
    for line in reversed(tail(best[1], 5, 262144)):
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        if isinstance(rec, dict):
            return rec
    return None


def total_steps(st):
    args = st.get("train_args") or []
    for i, a in enumerate(args):
        for fl in TOTAL_FLAGS:
            v = None
            if a == fl and i + 1 < len(args):
                v = num(args[i + 1])
            elif a.startswith(fl + "="):
                v = num(a.split("=", 1)[1])
            if v:
                return v
    cfg = load(os.path.join(RUN, "config.json"), {})
    a = cfg.get("args") if isinstance(cfg, dict) else None
    if isinstance(a, dict):
        for k in TOTAL_ARGS:
            if num(a.get(k)):
                return num(a.get(k))
    return DEFAULT_TOTAL.get(st.get("backend"))


def scan():
    files = []
    for root, dirs, fns in os.walk(OUT):
        dirs.sort()
        for fn in sorted(fns):
            if fn.endswith((".tmp", ".part")) or ".tmp." in fn:
                continue
            p = os.path.join(root, fn)
            try:
                s = os.stat(p)
            except OSError:
                continue
            files.append({"path": os.path.relpath(p, OUT).replace(os.sep, "/"), "size": s.st_size,
                          "mtime": round(s.st_mtime, 1)})
    return files


def render():
    with Lock():
        st = load(STATE, {})
        now = time.time()
        rec = last_record()
        steps = prog = eta = None
        if rec:
            for k in STEP_KEYS:
                if num(rec.get(k)) is not None:
                    steps = num(rec[k])
                    break
            prog = num(rec.get("progress"))
        total = total_steps(st)
        if prog is None and steps is not None and total:
            prog = min(1.0, steps / total)
        t_train = (st.get("phase_times") or {}).get("training")
        if st.get("phase") == "training" and prog and 0 < prog < 1 and t_train:
            eta = (now - t_train) * (1 - prog) / prog
        log = tail(os.path.join(OUT, "train.log"), 30) or tail(os.path.join(OUT, "entry.log"), 30)
        files = scan()
        s = dict(st)
        s.update(updated_at=now, uptime_s=round(now - st.get("started_at", now)), steps=steps, total_steps=total,
                 progress=prog, eta_s=eta, last_record=rec, last_log_line=log[-1] if log else None, log_tail=log,
                 has_policy=os.path.exists(os.path.join(RUN, "policy.onnx")),
                 n_files=len(files), total_bytes=sum(f["size"] for f in files))
        dump(os.path.join(OUT, "status.json"), s)
        dump(os.path.join(OUT, "manifest.json"), {"updated_at": now, "files": files})


def cmd_set(argv):
    with Lock():
        st = load(STATE, {})
        now = time.time()
        st.setdefault("started_at", now)
        for a in argv:
            if ":=" in a:
                k, v = a.split(":=", 1)
                v = json.loads(v)
            else:
                k, v = a.split("=", 1)
            if k == "phase" and v != st.get("phase"):
                st.setdefault("phase_times", {})[v] = now
            st[k] = v
        dump(STATE, st)
    render()


class Handler(SimpleHTTPRequestHandler):
    token = ""

    def translate_path(self, path):
        if self.token:
            p = path.split("?", 1)[0].split("#", 1)[0]
            pre = "/" + self.token
            if p != pre and not p.startswith(pre + "/"):
                return os.path.join(OUT, ".yok", "yok")
            path = path[len(pre):] or "/"
        return SimpleHTTPRequestHandler.translate_path(self, path)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        SimpleHTTPRequestHandler.end_headers(self)

    def log_message(self, *args):
        pass


def cmd_serve():
    def loop():
        while True:
            try:
                render()
            except Exception as e:
                print("render hatasi: %r" % (e,), file=sys.stderr)
            time.sleep(10)
    threading.Thread(target=loop, daemon=True).start()
    Handler.token = os.environ.get("OPENDUCK_HTTP_TOKEN", "")
    srv = ThreadingHTTPServer(("0.0.0.0", int(os.environ.get("OPENDUCK_PORT", "8765"))),
                              functools.partial(Handler, directory=OUT))
    srv.daemon_threads = True
    srv.serve_forever()


def cmd_args():
    raw = os.environ.get("OPENDUCK_TRAIN_ARGS", "").strip()
    args = json.loads(raw) if raw.startswith("[") else shlex.split(raw)
    sys.stdout.write("".join(str(a) + "\0" for a in args))


def cmd_cpus():
    n = [len(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else (os.cpu_count() or 1)]
    try:
        q, p = open("/sys/fs/cgroup/cpu.max").read().split()[:2]
        if q != "max":
            n.append(max(1, int(q) // int(p)))
    except (OSError, ValueError):
        pass
    try:
        q = int(open("/sys/fs/cgroup/cpu/cpu.cfs_quota_us").read())
        p = int(open("/sys/fs/cgroup/cpu/cpu.cfs_period_us").read())
        if q > 0:
            n.append(max(1, q // p))
    except (OSError, ValueError):
        pass
    e = num(os.environ.get("RUNPOD_CPU_COUNT"))
    if e and e >= 1:
        n.append(int(e))
    print(max(1, min(n)))


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "set":
        cmd_set(sys.argv[2:])
    elif cmd == "render":
        render()
    elif cmd == "serve":
        cmd_serve()
    elif cmd == "args":
        cmd_args()
    elif cmd == "cpus":
        cmd_cpus()
    else:
        sys.exit("bilinmeyen komut: " + cmd)
PYEOF
}

# --- bekleme ve kendini silme --------------------------------------------------------------------
# her podda runpodctl ve pod'a ozel RUNPOD_API_KEY hazir gelir (docs.runpod.io/runpodctl/overview).
# Silme olmazsa durdurur: GPU birakilir, konteyner diski silinir.
self_terminate() {
    local id=${RUNPOD_POD_ID:-} url code
    [ -n "$id" ] || { log "RUNPOD_POD_ID yok, kendini silme atlandi"; return 1; }
    if command -v runpodctl >/dev/null 2>&1; then
        timeout 120 runpodctl pod delete "$id" && return 0
        timeout 120 runpodctl remove pod "$id" && return 0
    fi
    if [ -n "${RUNPOD_API_KEY:-}" ] && command -v curl >/dev/null 2>&1; then
        for url in "https://api.runpod.io/v2/pods/$id" "https://rest.runpod.io/v1/pods/$id"; do
            code=$(curl -s -m 60 -o /dev/null -w '%{http_code}' -X DELETE -A openduck-pod/1.0 \
                   -H "Authorization: Bearer $RUNPOD_API_KEY" "$url")
            case $code in 200|202|204) return 0 ;; esac
            log "kendini silme: $url -> HTTP $code"
        done
    fi
    if command -v runpodctl >/dev/null 2>&1; then
        timeout 120 runpodctl pod stop "$id" && return 0
        timeout 120 runpodctl stop pod "$id" && return 0
    fi
    return 1
}

expiry_watchdog() {
    local left
    [ "$EXPIRES_AT" -gt 0 ] || return 0
    ( trap - TERM INT EXIT
      left=$(( EXPIRES_AT - $(date +%s) ))
      [ "$left" -le 0 ] || sleep "$left"
      log "pod suresi doldu (OPENDUCK_EXPIRES_AT): pod siliniyor"
      setst "expired:=true"
      self_terminate || log "kendini silme basarisiz; launch.py terminate ile silin" ) &
}

park() {
    local secs
    FINISHED=1
    secs=$(hours_to_s "$LINGER_HOURS")
    if [ "$secs" -gt 0 ]; then
        setst "linger_until:=$(( $(date +%s) + secs ))"
        log "sonuclar sunuluyor; $LINGER_HOURS saat sonra pod kendini silecek (launch.py fetch ile indirin)"
        snooze "$secs"
        setst "self_terminate:=true"
        log "bekleme suresi doldu, pod siliniyor"
        self_terminate || log "kendini silme basarisiz; sunmaya devam ediliyor (launch.py terminate ile silin)"
    else
        log "sonuclar sunuluyor (OPENDUCK_LINGER_HOURS=0: pod kendini silmez)"
    fi
    while true; do snooze 3600; done
}

fail() {
    log "HATA: $*"
    setst phase=failed "error=$*"
    park
}

stop_trainer() {
    if [ -n "$TRAIN_PID" ] && kill -0 "$TRAIN_PID" 2>/dev/null; then
        kill -TERM "$TRAIN_PID" 2>/dev/null
        for _ in $(seq 1 "$1"); do kill -0 "$TRAIN_PID" 2>/dev/null || break; sleep 1; done
        kill -KILL -- "-$TRAIN_PID" 2>/dev/null
    fi
    return 0
}

on_term() {
    log "durdurma sinyali alindi"
    stop_trainer 20
    [ "$FINISHED" = 1 ] || setst phase=stopped
    trap - TERM INT EXIT
    kill -TERM 0 2>/dev/null   # HTTP sunucu ve tee dahil tum grup
    exit 0
}

# beklenmedik cikis (or. set -u hatasi): konteyner yeniden baslamasin, failed olarak bekle
on_exit() {
    local rc=$?
    trap - EXIT
    log "HATA: pod_entry.sh beklenmedik sekilde cikti (kod $rc)"
    stop_trainer 60     # denetimsiz (sure siniri olmadan) egitim kalmasin
    [ "$FINISHED" = 1 ] || setst phase=failed "error=pod_entry.sh beklenmedik cikis (kod $rc), entry.log"
    park
}
trap on_term TERM INT

# --- adimlar -------------------------------------------------------------------------------------
get_code() {
    if [ -n "${OPENDUCK_CODE_DIR:-}" ]; then
        log "kod: $CODE (OPENDUCK_CODE_DIR, indirme yok)"
        return 0
    fi
    { rm -rf "$CODE" && mkdir -p "$CODE"; } || { ERR="$CODE olusturulamadi"; return 1; }
    case "$SOURCE" in
    inline)
        local want=${OPENDUCK_CODE_SHA256:-} sum
        [ -n "${OPENDUCK_CODE_B64:-}" ] || { ERR="OPENDUCK_CODE_B64 bos"; return 1; }
        if [ -n "$want" ]; then
            sum=$(printf %s "$OPENDUCK_CODE_B64" | base64 -d | sha256sum | cut -d' ' -f1)
            [ "$sum" = "$want" ] || { ERR="satir ici paketin sha256'si tutmuyor"; return 1; }
        fi
        printf %s "$OPENDUCK_CODE_B64" | base64 -d | tar -xz -C "$CODE" || { ERR="satir ici paket acilamadi"; return 1; }
        log "kod: satir ici paket acildi ($(find "$CODE" -type f | wc -l) dosya, sha256 ${want:0:12})"
        ;;
    git)
        local repo=${OPENDUCK_GIT_REPO:-} ref=${OPENDUCK_GIT_REF:-main} commit=${OPENDUCK_GIT_COMMIT:-}
        local tok=${GITHUB_TOKEN:-} auth
        case "$tok" in
            ""|*RUNPOD_SECRET*|*"{{"*)
                ERR="GitHub token yok: RunPod secret '${OPENDUCK_SECRET_NAME:-github_token}' tanimli mi?"; return 1 ;;
        esac
        [ -n "$repo" ] || { ERR="OPENDUCK_GIT_REPO bos"; return 1; }
        command -v git >/dev/null 2>&1 || { ERR="imajda git yok"; return 1; }
        auth=$(printf 'x-access-token:%s' "$tok" | base64 -w0)
        export GIT_TERMINAL_PROMPT=0 GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0="http.https://github.com/.extraheader" \
               GIT_CONFIG_VALUE_0="AUTHORIZATION: basic $auth"
        if ! timeout 600 git clone --quiet --depth 50 --branch "$ref" "https://github.com/$repo.git" "$CODE"; then
            ERR="git clone basarisiz ($repo @ $ref): token yetkisi ya da dal adi?"
        elif [ -n "$commit" ] && ! (cd "$CODE" && { git cat-file -e "$commit^{commit}" 2>/dev/null ||
                timeout 300 git fetch --quiet --depth 1 origin "$commit"; } && git checkout --quiet "$commit"); then
            ERR="commit $commit alinamadi (push edildi mi?)"
        fi
        unset GIT_CONFIG_COUNT GIT_CONFIG_KEY_0 GIT_CONFIG_VALUE_0 GITHUB_TOKEN
        auth=""
        [ -z "$ERR" ] || return 1
        log "kod: git $repo @ $ref -> $(git -C "$CODE" log -1 --format='%h %s')"
        setst "git_commit=$(git -C "$CODE" rev-parse HEAD)"
        ;;
    *)
        ERR="bilinmeyen OPENDUCK_SOURCE: $SOURCE"; return 1 ;;
    esac
    unset OPENDUCK_CODE_B64 OPENDUCK_ENTRY_B64
    return 0
}

install_deps() {
    if [ "${OPENDUCK_SKIP_INSTALL:-0}" = 1 ]; then
        PY=${OPENDUCK_PY:-$SYSPY}
        log "kurulum atlandi (OPENDUCK_SKIP_INSTALL=1), python: $PY"
        return 0
    fi
    local venv=$D/venv pyb=""
    local -a pipi pkgs extra
    if ! command -v uv >/dev/null 2>&1; then
        log "uv yok, pip ile kuruluyor"
        "$SYSPY" -m pip install -q uv >/dev/null 2>&1 || true
    fi
    # konteyner yeniden baslarsa /workspace kalir; uv var olan venv'i ezmeyi reddeder (--clear ister)
    rm -rf "$venv"
    if command -v uv >/dev/null 2>&1; then
        timeout 900 uv venv -q --python "$PYVER" "$venv" || { ERR="uv venv (python $PYVER) basarisiz"; return 1; }
        pipi=(uv pip install --python "$venv/bin/python")
    else
        for c in "python$PYVER" python3.13 python3.12; do command -v "$c" >/dev/null 2>&1 && { pyb=$c; break; }; done
        [ -n "$pyb" ] || { ERR="python>=3.12 ve uv yok"; return 1; }
        "$pyb" -m venv "$venv" || { ERR="$pyb -m venv basarisiz"; return 1; }
        pipi=("$venv/bin/python" -m pip install -q)
    fi
    PY=$venv/bin/python
    if [ "$TORCH_INDEX" != none ]; then
        log "torch kuruluyor ($TORCH_INDEX)"
        timeout 1800 "${pipi[@]}" --index-url "$TORCH_INDEX" torch || { ERR="torch kurulamadi"; return 1; }
    fi
    local req=$CODE/rl/requirements-$BACKEND.txt
    if [ -n "${OPENDUCK_PIP:-}" ]; then
        read -r -a pkgs <<< "$OPENDUCK_PIP"
    elif [ -f "$req" ]; then
        pkgs=(-r "$req")
    elif [ "$BACKEND" = mjx ]; then
        read -r -a pkgs <<< "$PKG_MJX"
    else
        read -r -a pkgs <<< "$PKG_CPU"
    fi
    read -r -a extra <<< "${OPENDUCK_PIP_EXTRA:-}"
    log "paketler: ${pkgs[*]} ${extra[*]}"
    timeout 2400 "${pipi[@]}" "${pkgs[@]}" "${extra[@]}" || { ERR="paket kurulumu basarisiz (entry.log)"; return 1; }
    return 0
}

check_env() {
    local mod=rl.train
    [ "$BACKEND" = mjx ] && mod=rl.train_mjx
    if [ -z "${OPENDUCK_TRAIN_CMD:-}" ] && [ ! -f "$CODE/${mod//.//}.py" ]; then
        ERR="$CODE/${mod//.//}.py yok"; return 1
    fi
    command -v nvidia-smi >/dev/null 2>&1 && timeout 60 nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
    local info
    info=$(cd "$CODE" && timeout 900 "$PY" - "$BACKEND" "${OPENDUCK_TRAIN_CMD:+custom}" <<'PYEOF'
import importlib, importlib.metadata as md, json, sys
backend, custom = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else ""
v = {}
for p in ("mujoco", "mujoco-mjx", "jax", "jaxlib", "brax", "playground", "torch", "onnx", "onnxruntime", "numpy"):
    try:
        v[p] = md.version(p)
    except md.PackageNotFoundError:
        pass
out = {"python": sys.version.split()[0], "versions": v}
try:
    from rl.robot_model import RobotIndex, build_model
    m = build_model()
    RobotIndex(m)
    out["model"] = "nq=%d nu=%d" % (m.nq, m.nu)
except Exception as e:
    out["model_error"] = repr(e)
# egitimin import ettigi her sey simdi yuklensin: eksik paket kurulumdan hemen sonra gorulur
mods = {"mjx": ["rl.env_mjx", "rl.train_mjx", "brax.training.agents.ppo.train", "onnx", "onnxruntime"],
        "cpu": ["rl.env_cpu", "rl.train", "rl.export_onnx", "torch", "onnx", "onnxruntime"]}
for name in ([] if custom else mods.get(backend, [])):
    try:
        importlib.import_module(name)
    except Exception as e:
        out.setdefault("import_error", []).append("%s: %r" % (name, e))
if backend == "mjx":
    try:
        import jax
        out["jax_devices"] = [str(d) for d in jax.devices()]
        out["jax_gpu"] = jax.devices()[0].platform == "gpu"
    except Exception as e:
        out["jax_error"] = repr(e)
print(json.dumps(out))
PYEOF
)
    log "ortam: $info"
    [ -n "$info" ] || { ERR="python ortam kontrolu calismadi (zaman asimi?)"; return 1; }
    setst "env:=$info"
    case "$info" in
        *'"model_error"'*) ERR="model yuklenemedi (build/sim, rl/ eksik mi?)"; return 1 ;;
        *'"import_error"'*) ERR="import hatasi (paket eksik?): $(printf %s "$info" | "$SYSPY" -c 'import json,sys; print("; ".join(json.load(sys.stdin)["import_error"])[:300])')"; return 1 ;;
    esac
    if [ "$BACKEND" = mjx ] && [ -z "${OPENDUCK_TRAIN_CMD:-}" ] && [ "${OPENDUCK_ALLOW_CPU_JAX:-0}" != 1 ]; then
        case "$info" in
            *'"jax_gpu": true'*) ;;
            *) ERR="JAX GPU gormuyor (surucu/CUDA uyumsuz?)"; return 1 ;;
        esac
    fi
    return 0
}

run_training() {
    local -a args cmd
    local secs cap start now rc tpid timed_out=0 term_at=0
    mapfile -d '' -t args < <("$SYSPY" "$D/helper.py" args) || { ERR="OPENDUCK_TRAIN_ARGS okunamadi"; return 1; }
    if [ -n "${OPENDUCK_TRAIN_CMD:-}" ]; then
        cmd=(bash -c "$OPENDUCK_TRAIN_CMD" train "$RUN" "${args[@]}")
    elif [ "$BACKEND" = mjx ]; then
        cmd=("$PY" -m rl.train_mjx --out "$RUN" "${args[@]}")
    else
        cmd=("$PY" -m rl.train --out "$RUN")
        case " ${args[*]} " in
            *" --workers "*|*" --workers="*) ;;
            *) cmd+=(--workers "$("$SYSPY" "$D/helper.py" cpus)") ;;
        esac
        cmd+=("${args[@]}")
    fi
    secs=$(hours_to_s "$MAX_HOURS")
    [ "$secs" -gt 0 ] || secs=14400
    if [ "$EXPIRES_AT" -gt 0 ]; then
        # konteyner yeniden basladiysa kalan pod suresine sigdir (bitince sunma suresi + 10 dk pay)
        cap=$(( EXPIRES_AT - $(date +%s) - $(hours_to_s "$LINGER_HOURS") - 600 ))
        if [ "$cap" -lt "$secs" ]; then
            [ "$cap" -ge 600 ] || { ERR="pod suresi (OPENDUCK_EXPIRES_AT) egitime yetmiyor"; return 1; }
            log "egitim siniri kalan pod suresine gore $cap sn"
            secs=$cap
        fi
    fi
    log "egitim (sinir $secs sn): ${cmd[*]}"
    setst phase=training "train_cmd:=$("$SYSPY" -c 'import json,sys; print(json.dumps(sys.argv[1:]))' "${cmd[@]}")"
    cd "$CODE" || { ERR="cd $CODE"; return 1; }
    setsid "${cmd[@]}" > >(tee -a "$OUT/train.log" >&3) 2>&1 &
    TRAIN_PID=$!
    tpid=$TRAIN_PID
    start=$(date +%s)
    while kill -0 "$TRAIN_PID" 2>/dev/null; do
        now=$(date +%s)
        if [ "$timed_out" = 0 ] && [ $(( now - start )) -ge "$secs" ]; then
            timed_out=1
            term_at=$now
            log "sure siniri ($secs sn) doldu: SIGTERM (egitim checkpoint yazip cikmali)"
            setst "time_limit_hit:=true"
            kill -TERM "$TRAIN_PID" 2>/dev/null
        elif [ "$timed_out" = 1 ] && [ $(( now - term_at )) -ge "$GRACE_S" ]; then
            log "egitim $GRACE_S sn icinde cikmadi: SIGKILL"
            kill -KILL -- "-$TRAIN_PID" 2>/dev/null
        fi
        snooze 5
    done
    wait "$TRAIN_PID"
    rc=$?
    TRAIN_PID=""
    kill -KILL -- "-$tpid" 2>/dev/null   # artakalan isci surecleri
    setst "exit_code:=$rc"
    if [ "$timed_out" = 1 ]; then
        log "egitim sure siniriyla durdu (kod $rc)"
        setst phase=timeout
    elif [ "$rc" = 0 ]; then
        log "egitim bitti"
        setst phase=done
    else
        ERR="egitim kod $rc ile cikti: $(tail -n 1 "$OUT/train.log" 2>/dev/null | cut -c1-300)"
        return 1
    fi
    [ -f "$RUN/policy.onnx" ] || { log "UYARI: $RUN/policy.onnx yok"; setst "warning=policy.onnx yok"; }
    return 0
}

# --- ana akis ------------------------------------------------------------------------------------
log "OpenDuck pod basliyor: backend=$BACKEND source=$SOURCE port=$PORT pod=${RUNPOD_POD_ID:-?}"
if [ -z "$SYSPY" ]; then
    log "HATA: imajda python3 yok; HTTP sunucu baslatilamiyor"
    expiry_watchdog
    while true; do snooze 3600; done
fi
write_helper
rm -f "$D/state.json"
trap on_exit EXIT
setst phase=server "backend=$BACKEND" "source=$SOURCE" "run_name=${OPENDUCK_RUN_NAME:-run}" \
      "pod_id=${RUNPOD_POD_ID:-}" "max_hours=$MAX_HOURS" "linger_hours=$LINGER_HOURS" \
      "expires_at:=$EXPIRES_AT" \
      "train_args:=$("$SYSPY" -c 'import json,os,shlex; r=os.environ.get("OPENDUCK_TRAIN_ARGS","").strip(); print(json.dumps(json.loads(r) if r.startswith("[") else shlex.split(r)))' || echo '[]')" \
      "error="
expiry_watchdog
( trap - TERM INT EXIT
  while true; do
      "$SYSPY" "$D/helper.py" serve
      log "HTTP sunucu durdu (kod $?), 10 sn sonra yeniden"
      sleep 10
  done ) &
log "HTTP sunucu: 0.0.0.0:$PORT -> $OUT"

if [ "${OPENDUCK_START_SH:-0}" = 1 ] && [ -x /start.sh ]; then
    log "/start.sh arka planda (ssh)"
    ( trap - TERM INT EXIT; /start.sh >/dev/null 2>&1 ) &
fi

SETUP_DEADLINE=$(( $(date +%s) + $(hours_to_s "$SETUP_HOURS") ))
setst phase=code
get_code || fail "kod alinamadi: $ERR"
setst phase=install
install_deps || fail "kurulum: $ERR"
[ "$(date +%s)" -le "$SETUP_DEADLINE" ] || fail "kurulum $SETUP_HOURS saati asti"
setst phase=check
check_env || fail "kontrol: $ERR"
run_training || fail "$ERR"
park
