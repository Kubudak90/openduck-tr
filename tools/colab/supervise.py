# Kernel icinde BLOKLAYAN, tekrar calistirilabilir gozetmen.
# - Ilk calismada egitimi baslatir; sonraki calismalarda (yeniden baglanma) yalnizca bekler.
# - Her HB_S saniyede bir satir yazar (kernel etkinligi; Colab bosta sanip VM'i geri almasin).
# - Her SNAP_S saniyede run klasorunu atomik olarak /content/sync_<ad>.tgz'ye paketler.
# - Egitim sureci bitince "CIKIS" yazar. Onceden tanimli: RUN_NAME, RUN_ARGS, RUN_ENV, (ops.) SNAP_S, HB_S
import subprocess, os, time
R = "/content/openduck-tr/runs"; N = RUN_NAME
SNAP = globals().get("SNAP_S", 600); HB = globals().get("HB_S", 60)
pidf, logp = "%s/%s.pid" % (R, N), "%s/%s.log" % (R, N)

def alive(pid):
    try:
        return open("/proc/%d/stat" % pid).read().rsplit(")", 1)[-1].split()[0] != "Z"
    except FileNotFoundError:
        return False

def snap():
    files = [f for f in ("config.json", "log.jsonl", "params_latest.pkl", "policy.onnx", "policy_meta.json", "c")
             if os.path.exists("%s/%s/%s" % (R, N, f))]
    tmp = "/content/sync_%s.tgz.tmp" % N
    subprocess.run(["tar", "czf", tmp, "-C", R, "%s.log" % N] + ["%s/%s" % (N, f) for f in files])
    os.replace(tmp, "/content/sync_%s.tgz" % N)

os.makedirs(R, exist_ok=True)
child = None
if os.path.exists(pidf):
    pid = int(open(pidf).read())
    print(time.strftime("%H:%M"), "yeniden baglanildi, pid", pid, "calisiyor" if alive(pid) else "BITMIS", flush=True)
else:
    env = dict(os.environ, PYTHONUNBUFFERED="1", OPENDUCK_ROBOT="tr35_b2", OPENDUCK_KAPPA="0.5")
    env.update(RUN_ENV)                    # RUN_ENV varsayilanlari ezebilir
    cmd = ["/content/venv/bin/python", "-m", "rl.train_mjx", "--out", "runs/" + N] + RUN_ARGS
    child = subprocess.Popen(cmd, cwd="/content/openduck-tr", env=env, stdout=open(logp, "ab"),
                             stderr=subprocess.STDOUT, start_new_session=True)
    pid = child.pid
    open(pidf, "w").write(str(pid))
    print(time.strftime("%H:%M"), "baslatildi pid", pid, " ".join(cmd), flush=True)

pos = 0
last_snap = last_hb = 0.0
while True:
    done = (child.poll() is not None) if child else not alive(pid)
    if os.path.exists(logp):
        with open(logp, "rb") as f:
            f.seek(pos); new = f.read(); pos += len(new)
        for ln in new.decode("utf-8", "replace").splitlines():
            if ln.startswith(("adim ", "bitti", "egitim", "Traceback")) or "Error" in ln:
                print(time.strftime("%H:%M"), ln, flush=True)
    now = time.time()
    if done or now - last_snap > SNAP:
        snap(); last_snap = now
    if done:
        print(time.strftime("%H:%M"), "CIKIS", ("rc=%s" % child.returncode) if child else "", flush=True)
        break
    if now - last_hb > HB:
        print(time.strftime("%H:%M"), "hb", flush=True); last_hb = now
    time.sleep(15)
