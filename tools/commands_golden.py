# -*- coding: utf-8 -*-
"""
rl/commands.py altin izleri (OpenDuck-TR spec eki K5): kontrol adimi basina girdiler (t, kimlikler, gravity_obs, gyro_obs,
bacak qd_obs, q_obs, alpha, h_ref) ve ciktilar (durum, politika, main komutu, jest hedefleri, LED, BEKLE rampasi, bekleyen,
isaretler, bias, donus integrali). Firmware bu izleri birebir tekrar uretmelidir: ayrik alanlar birebir, surekli 1e-6.

  OPENDUCK_ROBOT=tr35_b2 OPENDUCK_SCENE=<tr35 scene.xml> python tools/commands_golden.py --out rl/tests/data
      [--no-rollouts] [--rollout-rows 3000]

Yazar: <out>/komut_<ad>.csv.gz (gzip CSV, sutunlar commands.TRACE_IN + TRACE_OUT_D + TRACE_OUT_C) ve
<out>/komut_golden.json (meta: profil, turetilmis dosya sha256, kimlik tablosu (izin uretildigi), baslangic durumu,
home_q, satir sayisi, kaynak). Kaynaklar:
  - sentetik: rl/tests/komut_plant.SCENARIOS (kinematik bitki; fizik yok, sahne gerekmez);
  - rollout (P2-0 stub, zero politikalar, rl/eval_tasks A10): S1 (oturmus baslangic; stub'da oturus bankasi DR / bosluk
    altinda one devrildigi icin tani kurulumu: turetilmis oturus pozu, DR / bosluk / itme kapali, sensor gurultusu
    acik) ve S2 (A4 kosullari: DR, itme, bosluk; bolum 0 = HOME baslangici; ilk --rollout-rows satir).
Yeniden oynatma: rl/tests/test_commands.py (tr35 alt sureci). Turetilmis dosya ya da kimlik tablosu degisirse izler
yeniden uretilir (P5b kontrol listesi).
"""
import argparse
import csv
import gzip
import hashlib
import io
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

COLUMNS = None


def _cols():
    from rl import commands as CM
    return list(CM.TRACE_IN) + list(CM.TRACE_OUT_D) + list(CM.TRACE_OUT_C)


def fmt(v):
    return "%.9g" % v if isinstance(v, float) else str(v)


def write_trace(path, rows):
    cols = _cols()
    buf = io.StringIO()
    w = csv.writer(buf, lineterminator="\n")
    w.writerow(cols)
    for r in rows:
        w.writerow([fmt(r[c]) for c in cols])
    with gzip.GzipFile(path, "wb", mtime=0) as f:                 # belirlenimci (zaman damgasiz)
        f.write(buf.getvalue().encode())


def read_trace(path):
    with gzip.open(path, "rt") as f:
        return list(csv.DictReader(f))


def sha(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def main(argv=None):
    p = argparse.ArgumentParser(description="rl/commands.py altin izleri")
    p.add_argument("--out", default=os.path.join(ROOT, "rl", "tests", "data"))
    p.add_argument("--no-rollouts", action="store_true", help="yalniz sentetik senaryolar (sahne gerekmez)")
    p.add_argument("--rollout-rows", type=int, default=3000, help="S2 rollout izinin satir siniri")
    a = p.parse_args(argv)
    import numpy as np
    from rl import commands as CM
    from rl import config as C
    from rl.tests import komut_plant as KP
    os.makedirs(a.out, exist_ok=True)
    with open(CM.ID_TABLE_PATH) as f:
        id_table = json.load(f)
    meta = {"schema": 1, "tool": "tools/commands_golden.py", "profile": C.ROBOT_PROFILE,
            "derived_sha256": sha(C.DERIVED_PATH) if C.DERIVED_PATH else None, "id_table": id_table,
            "atol": 1e-6, "columns": _cols(), "traces": {}}
    for name, (start, _, _, kw) in KP.SCENARIOS.items():
        rows = []
        KP.scenario(name, rows=rows)
        fn = "komut_%s.csv.gz" % name
        write_trace(os.path.join(a.out, fn), rows)
        meta["traces"][name] = {"file": fn, "start": start, "home_q": [float(v) for v in KP.home_q()], "rows": len(rows),
                                "source": "sentetik (rl/tests/komut_plant.SCENARIOS, kinematik bitki)"}
        print("%-20s %5d satir" % (name, len(rows)))
    if not a.no_rollouts:
        from rl import eval_tasks as E
        from rl import play
        zm, zg = play.load_policy("zero", task="main"), play.load_policy("zero", task="getup")
        for name, mode, kw, seat, lim in (
                ("rollout_S1", "S1", dict(randomize=False, push=False, backlash=False), "exact", None),
                ("rollout_S2", "S2", dict(backlash=True), "bank", a.rollout_rows)):
            ev = E.Evaluator(seed=0, bank_n=16, collision=False, robot="tr35_b2", scene_path=C.SCENE_PATH,
                             n_scale=0.01, **kw)
            ev.a10_seat = seat
            rows = []
            ev.a10_rollout(zm, zg, mode, 1 if mode == "S1" else 2, trace=rows)
            rows = rows[:lim] if lim else rows
            fn = "komut_%s.csv.gz" % name
            write_trace(os.path.join(a.out, fn), rows)
            home = ev._probe_idx().default_obs_q
            meta["traces"][name] = {
                "file": fn, "start": "M" if mode == "S1" else "A", "home_q": [float(v) for v in np.asarray(home, np.float32)],
                "rows": len(rows), "source": "A10 %s rollout, P2-0 stub, zero politikalar (%s, oturus %s)" % (
                    mode, ", ".join("%s=%s" % kv for kv in sorted(kw.items())), seat)}
            print("%-20s %5d satir" % (name, len(rows)))
    with open(os.path.join(a.out, "komut_golden.json"), "w") as f:
        json.dump(meta, f, indent=1, sort_keys=True, ensure_ascii=False)
        f.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
