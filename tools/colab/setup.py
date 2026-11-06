# Colab VM kurulumu: repo (public, git clone) + izole venv /content/venv (tools/runpod/pod_entry.sh PKG_MJX ile ayni
# sabit surumler). Calistirma: colab exec -s <oturum> --timeout 900 -f tools/colab/setup.py  (~25 sn)
import subprocess, sys, os, time
def sh(cmd):
    t = time.time()
    print("$", cmd, flush=True)
    r = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = r.stdout.strip().splitlines()
    print("\n".join(out[-25:]), flush=True)
    print("-> rc=%d (%.0fs)" % (r.returncode, time.time() - t), flush=True)
    if r.returncode: raise SystemExit(r.returncode)
sh("nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv; nproc; free -g | head -2; python3 --version")
if not os.path.isdir("/content/openduck-tr"):
    sh("git clone -q --depth 1 https://github.com/Kubudak90/openduck-tr /content/openduck-tr")
sh("cd /content/openduck-tr && git log --oneline -1")
sh("command -v uv || pip install -q uv")
sh("uv venv -q --python 3.12 /content/venv 2>&1 || uv venv --python 3.12 /content/venv")
sh("uv pip install -q --python /content/venv/bin/python 'jax[cuda12]==0.12.0.1' mujoco==3.16.0 mujoco-mjx==3.16.0 brax==0.14.2 playground==0.2.0 onnx onnxruntime")
sh("/content/venv/bin/python -c 'import jax, mujoco, brax; print(jax.__version__, mujoco.__version__, jax.devices())'")
