# -*- coding: utf-8 -*-
"""
rl/play.py (sit / getup demolari, supervizor), rl/eval_tasks.py ve sim/collision_check.py testleri.

  python3 -m rl.tests.test_play_tasks
  python3 -m pytest rl/tests/test_play_tasks.py
"""
import json
import os
import sys
import tempfile
import time

import mujoco
import numpy as np

from rl import config as C
from rl import play
from rl import pose_bank as PB
from rl import task_math as T
from rl import tasks as TK

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _cc():
    from rl.eval_tasks import load_collision_check
    return load_collision_check()


def _export(d, task, name="policy.onnx", meta_task=None, seed=0):
    """Rastgele aktor -> ONNX + meta (gorev boyutlariyla)."""
    try:
        import torch
    except ImportError:          # MJX venv'inde torch yok
        import pytest
        pytest.skip("torch yok")
    from rl.export_onnx import export_policy, policy_meta
    from rl.ppo import ActorCritic
    spec = TK.get_task(task)
    torch.manual_seed(seed)
    ac = ActorCritic(spec.num_obs, spec.num_critic_obs, spec.num_actions, [32], [32])
    with torch.no_grad():
        for p in ac.actor.parameters():
            p.mul_(0.1)
    path = os.path.join(d, name)
    export_policy(ac, path, meta=policy_meta(7, "cpu_torch", task=meta_task or task))
    return path


# --- supervizor ----------------------------------------------------------------------
def _sup_state(tilt=0.0, gyro=0.0, dq=0.0):
    from rl.robot_model import RobotIndex, build_model
    home = RobotIndex(build_model()).default_obs_q
    quat = np.array([np.cos(0.5 * tilt), np.sin(0.5 * tilt), 0.0, 0.0])
    return {"quat": quat[None], "gyro": np.array([[gyro, 0.0, 0.0]]), "q": (home + dq)[None], "default_q": home,
            "trunk_z": np.array([0.05])}


def test_supervisor_unit():
    """main -> getup: gz > fall_gz art arda fall_steps adim (kesinti sayaci sifirlar); getup -> main: basari kosulu
    (z haric) ready_hold_s boyunca, kosul bozulunca zamanlayici sifirlanir."""
    S = C.SUPERVISOR
    sup = play.Supervisor()
    up, down = _sup_state(0.0), _sup_state(np.deg2rad(80.0))
    assert T.projected_gravity(np, down["quat"])[0, 2] > S["fall_gz"]
    for _ in range(S["fall_steps"] - 1):
        assert sup.update(down) is None
    assert sup.update(up) is None                          # kesinti: sayac sifir
    for _ in range(S["fall_steps"] - 1):
        assert sup.update(down) is None
    assert sup.update(down) == "getup" and sup.mode == "getup"
    n_hold = int(round(S["ready_hold_s"] / C.CTRL_DT))
    ok = _sup_state(0.0)                                    # z = 0.05: yukseklik bakilmaz (robotta yok)
    assert T.getup_success_now(np, ok, use_height=False)[0] == 1.0 and T.getup_success_now(np, ok)[0] == 0.0
    for _ in range(n_hold - 1):
        assert sup.update(ok) is None
    assert sup.update(_sup_state(0.0, gyro=1.0)) is None    # hizli donus: zamanlayici sifir
    assert sup.update(_sup_state(0.0, dq=0.5)) is None      # poz HOME'dan uzak
    for _ in range(n_hold - 1):
        assert sup.update(ok) is None
    assert sup.update(ok) == "main" and sup.mode == "main"


# --- play demolari ---------------------------------------------------------------------
def test_play_sit_demo_smoke():
    """ZeroPolicy (referans durus) ile --demo sit: segment adlari, main olcumleri, JSON, qpos yorungesi."""
    with tempfile.TemporaryDirectory() as d:
        j, q = os.path.join(d, "m.json"), os.path.join(d, "q.npy")
        out = play.run(play.parse_args(["--policy", "zero", "--demo", "sit", "--json", j, "--qpos-out", q]))
        assert [s["name"] for s in out["segments"]] == [n for n, _, _ in play.SIT_DEMO]
        assert out["task"] == "main" and out["demo"] == "sit" and out["falls"] == 0
        for s in out["segments"]:
            assert len(s["command"]) == 5 and all(k in s for k in ("height_rmse", "trunk_z_mean", "peak_tau"))
            assert s["height_rmse"] < 0.02 and 0.0 < s["peak_tau"] < 1.0
        sit = out["segments"][[n for n, _, _ in play.SIT_DEMO].index("sit")]
        up = out["segments"][[n for n, _, _ in play.SIT_DEMO].index("stand up")]
        assert sit["t_to_seated"] is not None and sit["t_to_seated"] <= C.MAIN["sitdown_window_s"] + 1e-9
        assert up["t_to_stand"] is not None and up["t_to_stand"] <= C.MAIN["standup_window_s"] + 1e-9
        assert sit["trunk_z_min"] < C.SEAT_Z + C.SEAT_Z_TOL
        with open(j) as f:
            assert json.load(f)["segments"][0]["name"] == "stand"
        traj = np.load(q)
        n = int(round(sum(dd for _, _, dd in play.SIT_DEMO) / C.CTRL_DT))
        assert traj.shape == (n + 1, 22)


def test_play_walk_demo_unchanged_names():
    """Cplak --demo == walk; walk segment adlari ve 3 boyutlu komutlar (main politikasinda 5'e doldurulur)."""
    a = play.parse_args(["--policy", "zero", "--demo"])
    assert a.demo == "walk"
    out = play.run(play.parse_args(["--policy", "zero", "--demo", "--seconds", "2.2"]))
    assert [s["name"] for s in out["segments"]] == ["stand", "walk forward"] and "task" not in out
    assert all(len(s["command"]) == 3 for s in out["segments"])
    out = play.run(play.parse_args(["--policy", "zero", "--task", "main", "--demo", "--seconds", "2.2"]))
    assert out["task"] == "main" and [s["command"] for s in out["segments"]][1] == [0.15, 0.0, 0.0, C.H_HOME, 0.0]
    out = play.run(play.parse_args(["--policy", "zero", "--task", "main", "--command", "0.1", "0", "0", "0.09", "0",
                                    "--seconds", "1"]))
    assert out["segments"][0]["command"] == [0.1, 0.0, 0.0, 0.09, 0.0]
    for argv in (["--policy", "zero", "--command", "0.1", "0", "0", "0.09", "0", "--seconds", "1"],
                 ["--policy", "zero", "--task", "walk", "--demo", "sit"]):
        try:
            play.run(play.parse_args(argv))
            raise AssertionError("accepted: %r" % (argv,))
        except ValueError:
            pass


def test_play_getup_demo_smoke():
    """--demo getup (zero/zero): devirme itmesi supervizoru getup'a gecirir; olcum anahtarlari."""
    out = play.run(play.parse_args(["--policy", "zero", "--getup-policy", "zero", "--demo", "getup",
                                    "--seconds", "8", "--tip", "back"]))
    sup = out["supervisor"]
    assert out["task"] == "main" and out["demo"] == "getup"
    assert [s["name"] for s in out["segments"]] == ["stand", "walk forward", play.TIP_SEGMENT]
    assert sup["events"] and sup["events"][0]["to"] == "getup" and sup["events"][0]["t"] > 5.0
    assert sup["tips"] >= 1 and sup["final_mode"] == "getup" and sup["t_to_upright"] is None and not sup["success"]
    try:
        play.run(play.parse_args(["--policy", "zero", "--demo", "getup"]))
        raise AssertionError("getup demo without --getup-policy accepted")
    except ValueError:
        pass


def test_play_getup_handoff_path():
    """Supervizor devir yolu (zorlanmis olaylarla): getup'a devirde getup ortami ayni fiziksel durumdan, main'e
    donuste last_action = 0, alpha = 0, h_ref = H_HOME ve handoff_zero_cmd_s sifir komut; 2 s icinde tekrar
    devir 'fell_within_2s' sayilir."""
    seen = {}

    class Scripted(play.Supervisor):
        k = 0

        def update(self, s):
            Scripted.k += 1
            k = Scripted.k
            if k in (150, 250):
                self.mode = "getup"
                return "getup"
            if k == 200:
                self.mode = "main"
                return "main"
            if k == 201:
                seen["state"] = s
            return None

    orig_sup, orig_set = play.Supervisor, None
    from rl import env_cpu
    orig_set = env_cpu.VecEnv.set_state

    def spy(self, i, qpos, qvel, ctrl=None, warmstart=None):
        res = orig_set(self, i, qpos, qvel, ctrl, warmstart)
        seen.setdefault("handoffs", []).append((self.task, res[0][i].copy(), self.task_state()))
        return res

    play.Supervisor = Scripted
    env_cpu.VecEnv.set_state = spy
    try:
        out = play.run(play.parse_args(["--policy", "zero", "--getup-policy", "zero", "--demo", "getup",
                                        "--seconds", "7", "--tip", "front"]))
    finally:
        play.Supervisor = orig_sup
        env_cpu.VecEnv.set_state = orig_set
    sup = out["supervisor"]
    assert [e["to"] for e in sup["events"]] == ["getup", "main", "getup"]
    assert [e["t"] for e in sup["events"]] == [150 * C.CTRL_DT, 200 * C.CTRL_DT, 250 * C.CTRL_DT]
    assert sup["t_to_upright"] is not None and abs(sup["t_to_upright"] - 50 * C.CTRL_DT) < 1e-6
    # ikinci devir ilk donusten 1 s sonra: donusten sonra 2 s icinde tekrar dusme
    assert sup["getups"][0]["fell_within_2s"] is True and sup["falls_after_handoff"] == 1
    tasks = [h[0] for h in seen["handoffs"]]
    assert tasks == ["getup", "main", "getup"]
    _, obs_main, st = seen["handoffs"][1]
    lay = TK.get_task("main").obs_slices
    assert np.all(obs_main[lay["last_action"]] == 0.0) and np.allclose(obs_main[lay["posture"]], [0.0, 0.0])
    assert st["h_ref"][0] == C.H_HOME and st["alpha"][0] == 0.0 and st["step_count"][0] == 0
    assert np.allclose(seen["state"]["command"][0], [0, 0, 0, C.H_HOME, 0])    # 1 s sifir komut
    _, obs_g, _ = seen["handoffs"][0]
    assert np.all(obs_g[TK.get_task("getup").obs_slices["last_action"]] == 0.0)


def test_policy_roles_and_meta_checks():
    """Gorev meta'dan; ONNX/meta uyusmazligi ve yanlis rol ValueError."""
    with tempfile.TemporaryDirectory() as d:
        g = _export(d, "getup", "getup.onnx")
        m = _export(d, "main", "main.onnx")
        w = _export(d, "walk", "walk.onnx")
        assert play.load_policy(g).task == "getup" and play.load_policy(m).task == "main"
        assert play.load_policy(w).task == "walk"
        p = play.load_policy(g)
        assert p(np.zeros((3, 48), np.float32)).shape == (3, 14)
        with open(os.path.join(d, "getup_meta.json")) as f:
            meta = json.load(f)
        assert meta["task"] == "getup" and len(meta["action_joints"]) == 14 and meta["obs_dim"] == 48
        for f_, args in ((lambda: play.load_policy(w, task="main"), None),
                         (lambda: play.run(play.parse_args(["--policy", g, "--seconds", "0.1"])), None),
                         (lambda: play.run(play.parse_args(["--policy", w, "--demo", "sit"])), None)):
            try:
                f_()
                raise AssertionError("role mismatch accepted")
            except ValueError:
                pass
        # meta ile ONNX boyutu celisirse
        bad = _export(d, "main", "bad.onnx", meta_task="walk")
        try:
            play.load_policy(bad)
            raise AssertionError("meta/onnx mismatch accepted")
        except ValueError:
            pass
        # meta'siz ONNX: boyutlardan
        os.remove(os.path.join(d, "main_meta.json"))
        assert play.load_policy(m).task == "main"
        assert play.infer_task(49, 10) == "walk" and play.infer_task(53, 10) == "main"


# --- eval_tasks ------------------------------------------------------------------------
def test_eval_tasks_smoke():
    """Kucuk ayarlarla (zero politikalar, 2 bolum, A4 3 bolum, kucuk banka) A1-A4 calisir ve JSON yazilir;
    --collision yokken A8 olculmez ve genel kabule dahil degil; --a4-episodes yalniz A4'u etkiler; Wilson araliklari."""
    from rl import eval_tasks as E
    assert E.parse_args(["--main-policy", "zero"]).a4_episodes == 512 == E.A4_EPISODES
    with tempfile.TemporaryDirectory() as d:
        out_path = os.path.join(d, "eval.json")
        rc = E.main(["--main-policy", "zero", "--walk-policy", "zero", "--getup-policy", "zero", "--episodes", "2",
                     "--a4-episodes", "3", "--bank-n", "32", "--seed", "1", "--out", out_path])
        assert rc == 0
        with open(out_path) as f:
            r = json.load(f)
    assert not r["errors"] and set(r["pass"]) == {"A1", "A2", "A3", "A4"} and r["pass_all_includes"] == list(r["pass"])
    assert r["A8"]["status"] == "not_run" and "--collision" in r["pass_note"] and r["pass_all"] is False
    for k in ("A1", "A2", "A3", "A4"):
        assert r[k]["status"] == "ok", (k, r[k])
    a1 = r["A1"]
    assert a1["walk"]["episodes"] == 2 and a1["main"]["episodes"] == 2 and a1["tracking_ratio"] is not None
    assert len(r["A2"]["standing"]["rmse"]) == 2 and r["A2"]["standing"]["h"][0] == C.H_MIN_STAND
    wk = r["A2"]["walking"]
    assert len(wk["vx_mean"]) == 2 and abs(wk["vx_min_required"] - 0.8 * 0.15) < 1e-12
    assert r["A2"]["standing"]["vx_min_required"] is None and r["A2"]["standing"]["moving"] is True
    # sifir politika yerinde durur: yururken hiz sarti tutmaz, A2 walking gecemez
    assert max(abs(v) for v in wk["vx_mean"]) < 0.05 and wk["moving"] is False and wk["pass"] is False
    for k in ("sit_success_rate", "stand_success_rate", "t_to_seated", "t_to_stand", "torque_sat_frac"):
        assert k in r["A3"]["from_home"], k
    assert "stand_success_rate" in r["A3"]["from_seated"]
    a4 = r["A4"]
    assert r["a4_episodes"] == 3 and a4["episodes_per_kind"] == 3 and a4["head_force_sampling"] == "per_substep_max"
    assert a4["fallen"]["episodes"] == 3 and a4["midfall"]["episodes"] == 3 and a4["head_force_peak_median_n"] >= 0.0
    assert a4["fallen"]["head_force_peak_n"]["n"] == 3
    assert set(a4["fallen"]["modes"]) <= set(PB.LABELS) | {"side"}
    for m in list(a4["fallen"]["modes"].values()) + [a4["fallen"], a4]:
        lo, hi = m.get("ci95") or m["success_ci95"]
        assert 0.0 <= lo <= m["success_rate"] <= hi <= 1.0
    assert sum(m["n"] for k, m in a4["fallen"]["modes"].items() if k != "side") == 3


def test_wilson_interval():
    """Wilson %95 araligi: bilinen degerler, uclar, n = 0."""
    from rl.eval_tasks import wilson
    lo, hi = wilson(90, 100)
    assert abs(lo - 0.8256) < 1e-4 and abs(hi - 0.9448) < 1e-4
    assert wilson(0, 10)[0] == 0.0 and wilson(10, 10)[1] == 1.0 and wilson(0, 0) == [None, None]
    lo, hi = wilson(461, 512)                       # 512 bolumde %90: alt sinir %85'in ustunde
    assert lo > 0.87 and hi < 0.93


def test_eval_a2_walking_requires_motion():
    """A2 yururken: pencerede canli ortamlarin govde cercevesi ileri hizi ortalamasi >= 0.8 vx olmali (yerinde
    duran robot gecemez). Hiz olcumu task_state()['linvel'][:, 0] uzerinden (burada kaydirilarak sinanir)."""
    from rl import eval_tasks as E
    zero = play.load_policy("zero", task="main")
    orig = E.VecEnv.task_state
    res = {}
    try:
        for add in (0.0, 0.11, 0.13):
            def ts(self, add=add):
                s = orig(self)
                s["linvel"] = s["linvel"].copy()
                s["linvel"][:, 0] += add
                return s
            E.VecEnv.task_state = ts
            ev = E.Evaluator(episodes=2, seed=2, randomize=False, noise=False, push=False, head_random=False)
            res[add] = ev.height(zero, True)
    finally:
        E.VecEnv.task_state = orig
    for add, r in res.items():
        assert not any(r["fell"]) and max(r["rmse"]) < 0.01, r
        assert all(abs(v - add) < 0.02 for v in r["vx_mean"]), (add, r["vx_mean"])
    assert res[0.0]["moving"] is False and res[0.11]["moving"] is False and res[0.13]["moving"] is True
    assert res[0.13]["pass"] and not res[0.11]["pass"] and not res[0.0]["pass"]


def test_eval_a8_groups_multi_env():
    """--collision: her rollout grubu (A1_main, A1_walk, A2_*, A3_*, A4_*) ilk min(8, N) ortamdan kaydedilir; grup
    basina kare/ortam/en kotu cift raporlanir; A1_walk (taban cizgisi) kabule dahil degil; A8 genel kabulde."""
    from rl import eval_tasks as E
    stride = E.A8_STRIDE
    E.A8_STRIDE = 50                    # test hizi: 1 s'de bir kare
    try:
        r = E.evaluate(main="zero", walk="zero", getup="zero", episodes=2, a4_episodes=3, bank_n=32, seed=1,
                       collision=True, log=None, randomize=False, noise=False, push=False, head_random=False)
    finally:
        E.A8_STRIDE = stride
    a8 = r["A8"]
    assert a8["status"] == "ok" and set(a8["groups"]) == set(E.A8_GROUPS), sorted(a8["groups"])
    assert a8["baseline_groups"] == ["A1_walk"] and "A1_walk" not in a8["verdict_groups"]
    assert "A8" in r["pass"] and r["pass"]["A8"] == a8["pass"] and r["pass_all_includes"][-1] == "A8"
    for name, g in a8["groups"].items():
        k = 3 if name.startswith("A4") else 2
        assert g["envs"] == k and len(g["frames_per_env"]) == k, (name, g)
        assert g["frames"] == sum(g["frames_per_env"]) and g["frames_per_env"][0] > 0, (name, g)
        assert g["in_verdict"] == (name != "A1_walk")
    # A4: 6 s, kare 1 s'de bir, bolum sonuna dek canli -> ortam basina 5 kare (son adim sifirlanan durum alinmaz)
    assert a8["groups"]["A4_fallen"]["frames_per_env"] == [5, 5, 5]
    assert a8["joint_tol_voxels"] == 4 and abs(a8["joint_tol_mm3"] - 4 * 0.512) < 1e-9 and a8["stride_steps"] == 50


def test_eval_a8_verdict_aggregation():
    """a8(): grup basina en kotu cift / ortam / zaman; taban cizgisi grubu (A1_walk) karari etkilemez; eklem
    toleransi icindeki diz titremesi kaldirmaz, gercek girisim (diz 70 derece) kaldirir."""
    from rl import eval_tasks as E
    cc = _cc()
    ms = cc.mesh_set()
    m = ms.m
    k = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "left_knee")]
    home, q70 = ms.home_qpos.copy(), ms.home_qpos.copy()
    q70[k] = np.deg2rad(70.0)
    ev = E.Evaluator(episodes=2, collision=True)
    ev.traj = {"A1_walk": {"qpos": [np.stack([home, q70])], "steps": [[10, 20]]},
               "A3_home": {"qpos": [np.stack([home]), np.stack([home, home, home])], "steps": [[10], [10, 20, 30]]}}
    r = ev.a8()
    assert r["pass"] is True and r["groups"]["A1_walk"]["pass"] is False and r["worst_group"] == "A3_home"
    w = r["groups"]["A1_walk"]["worst"]
    assert w["env"] == 0 and w["step"] == 20 and abs(w["t_s"] - 20 * C.CTRL_DT) < 1e-12 and w["mm3"] > 1.0
    assert set(w["parts"]) == {"left_thigh_inner_plate", "left_shin_pad"}
    assert r["groups"]["A3_home"]["frames"] == 4 and r["groups"]["A3_home"]["frames_per_env"] == [1, 3]
    ev.traj["A3_home"]["qpos"][1] = np.stack([home, q70, home])
    r = ev.a8()
    assert r["pass"] is False and r["groups"]["A3_home"]["worst"]["env"] == 1
    assert r["groups"]["A3_home"]["worst"]["step"] == 20 and r["max_pen_mm3"] > 1.0


def test_eval_tasks_role_mismatch():
    """Walk politikasi main rolune verilirse: walk taban cizgisi olculur, A1-A3 'error', cikis kodu 2 (traceback yok)."""
    from rl import eval_tasks as E
    with tempfile.TemporaryDirectory() as d:
        w = _export(d, "walk", "policy.onnx")
        r = E.evaluate(main=w, walk=w, episodes=2, log=None)
        assert r["policies"]["main"]["status"] == "error" and r["policies"]["main"]["task"] == "walk"
        assert r["errors"] and "main" in r["errors"][0]
        assert r["A1"]["status"] == "error" and r["A1"]["walk"]["episodes"] == 2
        assert r["A2"]["status"] == "error" and r["A3"]["status"] == "error" and r["A4"]["status"] == "skipped"
        out_path = os.path.join(d, "e.json")
        assert E.main(["--main-policy", w, "--episodes", "1", "--out", out_path]) == 2
        with open(out_path) as f:
            assert json.load(f)["A2"]["status"] == "error"
        r = E.evaluate(main=os.path.join(d, "missing.onnx"), episodes=1, log=None)
        assert r["policies"]["main"]["status"] == "error" and r["A2"]["status"] == "error"


# --- collision_check -----------------------------------------------------------------
def test_collision_check_home_vs_knee70():
    """HOME'da taban disi girisim yok; diz 70 derece (eski sinir) uyluk ic plakasi - incik pedi girisimi > 1 mm^3;
    diz 65 derece (yeni sinir) temiz; yorunge API'si ve CLI."""
    cc = _cc()
    ms = cc.mesh_set()
    base = ms.baseline()
    assert 15 <= len(base) <= 40 and ("left_knee_horn", "left_knee_servo") in base
    q = ms.home_qpos.copy()
    assert cc.check_pose(q, ms)["pen_mm3"] == 0.0
    m = ms.m
    k = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "left_knee")]
    q70, q65 = q.copy(), q.copy()
    q70[k], q65[k] = np.deg2rad(70.0), np.deg2rad(65.0)
    r = cc.check_pose(q70, ms)
    assert r["pen_mm3"] > 1.0 and set(r["pairs"][0]["parts"]) == {"left_thigh_inner_plate", "left_shin_pad"}, r
    assert set(r["pairs"][0]["bodies"]) == {"left_thigh", "left_shin"}
    assert cc.check_pose(q65, ms)["pen_mm3"] <= 1.0
    tr = cc.check_trajectory(np.stack([q, q65, q70, q]), stride=1, ms=ms)
    assert tr["frames"] == 4 and tr["worst"]["frame"] == 2 and not tr["pass"] and tr["max_pen_mm3"] > 1.0
    assert "left_shin_pad~left_thigh_inner_plate" in tr["pairs"]
    assert cc.check_trajectory(np.stack([q, q65]), stride=1, ms=ms)["pass"]
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "t.npy")
        np.save(p, np.stack([q, q70]))
        j = os.path.join(d, "r.json")
        assert cc.main([p, "--stride", "1", "--json", j]) == 1
        with open(j) as f:
            assert json.load(f)["max_pen_mm3"] > 1.0
        np.save(p, np.stack([q, q]))
        assert cc.main([p]) == 0
    try:
        cc.check_trajectory(np.zeros((3, 5)), ms=ms)
        raise AssertionError("wrong qpos width accepted")
    except ValueError:
        pass


def test_collision_check_joint_tolerance():
    """Eklemle dogrudan bagli (ebeveyn/cocuk) govde ciftlerinde <= 4 voxel girisim yok sayilir (diz arayuzu
    titremesi), diger ciftlerde 1 mm^3 siniri aynen; sonuc JSON'unda raporlanir."""
    cc = _cc()
    ms = cc.mesh_set()
    vox = (ms.pitch * 1000.0) ** 3
    assert cc.JOINT_TOL_VOX == 4 and abs(ms.joint_tol_mm3 - 4 * vox) < 1e-12
    assert frozenset(("left_thigh", "left_shin")) in ms.joint_pairs and frozenset(("trunk", "left_hip")) in ms.joint_pairs
    assert frozenset(("trunk", "left_shin")) not in ms.joint_pairs and frozenset(("left_hip", "left_thigh")) not in ms.joint_pairs
    assert ms.ignored("left_thigh_inner_plate", "left_shin_bracket", "left_thigh", "left_shin", 4 * vox) == "joint"
    assert ms.ignored("left_thigh_inner_plate", "left_shin_bracket", "left_thigh", "left_shin", 5 * vox) is None
    assert ms.ignored("chassis", "left_shin_bracket", "trunk", "left_shin", 2 * vox) is None
    a, b = next(iter(ms.baseline()))
    assert ms.ignored(a, b, "x", "y", 1e3) == "baseline"
    # check_pose / check_trajectory yolu, geometriden bagimsiz: ms.pose sarilip ebeveyn/cocuk ciftine (diz arayuzu)
    # ham girisim eklenir. <= 4 voxel denetimde 0 (joint_tolerated'da raporlanir), 5 voxel basarisiz.
    pair = ("left_thigh_inner_plate", "left_shin_bracket")
    assert tuple(sorted(pair)) not in ms.baseline()

    def fake_pose(n_vox):
        def pose(q, *a, **k):
            recs, minz = cc.MeshSet.pose(ms, q, *a, **k)
            return recs + [pair + ("left_thigh", "left_shin", n_vox * vox)], minz
        return pose
    try:
        ms.pose = fake_pose(4)
        r = cc.check_pose(ms.home_qpos, ms)
        assert r["pen_mm3"] == 0.0 and r["joint_tolerated"] and r["joint_tolerated"][0]["mm3"] <= ms.joint_tol_mm3
        tr = cc.check_trajectory(np.stack([ms.home_qpos, ms.home_qpos]), stride=1, ms=ms)
        assert tr["pass"] and tr["max_pen_mm3"] == 0.0 and tr["joint_tolerated"] and tr["joint_tol_voxels"] == 4
        ms.pose = fake_pose(5)
        r = cc.check_pose(ms.home_qpos, ms)
        assert abs(r["pen_mm3"] - 5 * vox) < 1e-9 and not r["joint_tolerated"]
    finally:
        del ms.pose                      # ornek ozniteligi: sinif metoduna don (mesh_set onbellekli)


def main():
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    if len(sys.argv) > 1:
        tests = [(k, v) for k, v in tests if any(a in k for a in sys.argv[1:])]
    failed = 0
    for name, fn in tests:
        t = time.time()
        try:
            fn()
            print("PASS  %-40s %6.2fs" % (name, time.time() - t), flush=True)
        except Exception:
            failed += 1
            import traceback
            print("FAIL  %-40s %6.2fs" % (name, time.time() - t), flush=True)
            traceback.print_exc()
    print("%d/%d passed" % (len(tests) - failed, len(tests)))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
