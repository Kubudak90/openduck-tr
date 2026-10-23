# -*- coding: utf-8 -*-
"""
sysID testleri: bilinen servodan (synth) uretilen Adim 0 log'larindan parametreler +-%5 icinde geri bulunur.

  python -m pytest tools/sysid/tests -q -p no:cacheprovider
Sentetik servo: model.simulate (MuJoCo position aktuatoru + eklem sonumu/armature/frictionloss + play olu bant +
gecikme); algilayicilar gercekci gurultulu (pot 0.6 mV pin, INA 1.5 mA, aciolcer hizalama 0.15 derece, pot egriligi).
Ayrica analitik modelin MuJoCo tek eklemiyle ayni anlamda oldugu kontrol edilir.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.sysid import fit, model as M, synth  # noqa: E402
from tools.sysid.report import write_report  # noqa: E402

TOL = 0.05


@pytest.fixture(scope="module")
def session(tmp_path_factory):
    out = tmp_path_factory.mktemp("bench")
    tr = synth.Truth()
    b = synth.Bench(truth=tr, seed=3, cfg=dict(mass_g=1500.0))
    synth.full_session(b, out, servo_id="DS-T1", quick=True)
    res = fit.fit_servo(out / "DS-T1", mass=1.95)
    return tr, b, res, out


def _rel(a, b):
    return abs(a / b - 1.0)


def test_actuator_params_within_5pct(session):
    tr, b, res, _ = session
    p, a = tr.p, res["actuator"]
    truth = dict(tau_s=p.tau_s, omega0=p.omega0, omega_noload=p.omega_noload, damping=p.damping, kp=p.kp,
                 p_band_deg=math.degrees(p.p_band), armature=p.armature, frictionloss=p.frictionloss,
                 deadband_deg=math.degrees(p.deadband), latency_s=p.latency)
    bad = {k: (a[k], v) for k, v in truth.items() if a.get(k) is None or _rel(a[k], v) > TOL}
    assert not bad, bad
    # 60 derece suresi (bosta plato) ve us cinsinden olu bant
    assert _rel(a["t60_s"], (math.pi / 3) / p.omega_noload) < TOL
    assert _rel(a["deadband_us"], math.degrees(p.deadband) / tr.dpu) < TOL


def test_supply_current_and_stall_methods(session):
    tr, b, res, _ = session
    sc = res["supply_current"]
    assert _rel(sc["i_stall"], tr.p.i_stall) < TOL
    assert _rel(sc["i_idle"], tr.p.i_idle) < TOL
    assert sc["r2"] > 0.99
    assert sc["stall_two_method_pass"] is True
    assert _rel(sc["stall_a_ina_sat"], tr.p.i_stall + tr.p.i_idle) < TOL


def test_pot_maps_and_common_mode(session):
    tr, b, res, _ = session
    pot = res["pot"]
    # pot ic regulatorden 3.3 V: orta uc olcegi span/3.3 derece/V; komut olcegi gercek derece/us
    assert _rel(pot["potcal"]["scale_deg_per_v"], tr.pot_span / tr.pot_reg) < TOL
    assert _rel(pot["map"]["cmd_deg_per_us"], tr.dpu) < TOL
    off_true = tr.pot_reg * (0.5 - tr.pc_off / tr.pot_span)   # 0 derecede orta uc gerilimi (V)
    assert abs(pot["potcal"]["offset_v"] - off_true) < TOL * tr.pot_reg
    assert pot["potcal"]["nonlin_max_deg"] < 0.5
    sw = pot["sweep"]
    assert sw["present"] and sw["monotonic"] and sw["span_v"] > 1.0
    assert sw["pot_supply"] == "3V3" and abs(sw["pot_supply_v"] - tr.pot_reg) < 0.05
    cm = pot["common_mode"]
    slope_true = tr.r_gnd / (tr.pot_reg / tr.pot_span)          # derece/A (alt uc topragi kayar)
    assert _rel(cm["slope_deg_per_a"], slope_true) < TOL


def test_rail_thermal_gate(session):
    tr, b, res, _ = session
    assert abs(res["rail"]["v_max"] - b.ovs_true) < 0.02
    th = res["thermal"]
    assert _rel(th["t_knee"], synth.true_t_knee(tr)) < TOL
    assert _rel(th["tau_th_s"], tr.tau_th) < TOL
    assert not th["extrapolated"]
    g = res["gate"]
    want = {str(k): M.u_fit(tr.p.tau_s, tr.p.omega_noload, 1.95, k) for k in (0.72, 0.6, 0.5)}
    for k, v in want.items():
        assert abs(g["u"][k] - v) < 0.02
    assert g["kappa"] == 0.5            # DSSERVO Blue + 1.95 kg: review tur 1 karari (kappa 0.5)


def test_verdicts_and_report(session):
    tr, b, res, _ = session
    v = {x["item"]: x for x in res["verdicts"]}
    assert v["u(tau_s, omega0, kutle) <= 0.85"]["ok"] is True
    assert v["Pot (var, monoton, >= 1 V)"]["ok"] is True
    assert v["Pot ortak-mod hata"]["ok"] is False                     # stok kablo + 3.3 V pot: ~6 derece
    assert "Oranli" in v["Pot ortak-mod hata"]["action"]
    assert v["DS 300 Hz"]["ok"] is True and v["3.3 V darbe"]["ok"] is True
    assert v["Ray asimi (yavaslamada)"]["ok"] is True
    assert v["Isinma: T_diz"]["ok"] is True
    assert v["Diz uygunlugu"]["ok"] is False                          # T_diz ~1.0 < tahmini 1.11 N.m
    md = write_report([res], 1.95)
    assert "Karar tablosu" in md and "kappa = 0.5" in md
    js = fit._clean(res)
    import json
    json.dumps(js)                                                    # JSON'a yazilabilir


def _small_session(root, servo, truth, seed):
    b = synth.Bench(truth=truth, seed=seed)
    plan = [("sweep", dict(step_us=40, dwell_ms=150)), ("potcal", dict(points="-60,-30,0,30,60")),
            ("speed", dict(reps=1, hold_ms=500)), ("stall", dict(deltas_deg="1,3,8", hold_ms=1500, cool_ms=500))]
    for k, (prog, kw) in enumerate(plan):
        b.run(prog, **kw).write(root / servo / ("%02d_%s" % (k, prog)),
                                dict(servo_id=servo, servo_model=servo.split("-")[0], test=prog,
                                     host_time="2026-10-23T11:%02d" % k))


def test_cli_writes_params(tmp_path):
    """CLI: ust klasorde birden cok servo -> servos listesi + karsilastirma tablosu."""
    _small_session(tmp_path / "logs", "DS3230-1", synth.Truth(), 5)
    _small_session(tmp_path / "logs", "DS3225-1", synth.Truth(p=M.ServoParams(tau_s=2.69, omega0=7.32)), 6)
    p, r = tmp_path / "p.json", tmp_path / "r.md"
    fit.main(["--logs", str(tmp_path / "logs"), "--mass", "1.95", "--out", str(p), "--report", str(r)])
    import json
    d = json.loads(p.read_text())
    assert d["schema"] == "openduck-tr/sysid/v1" and len(d["servos"]) == 2
    s = {x["servo"]["id"]: x for x in d["servos"]}
    assert s["DS3230-1"]["servo"]["mjcf_class"] == "ds32xx"
    assert abs(s["DS3225-1"]["actuator"]["tau_s"] / 2.69 - 1) < TOL
    # DSSERVO DS3225MG (2.69 N.m, 7.32 rad/s) 1.95 kg'da u(0.72) = 0.86 > 0.85 -> kappa 0.6; Blue -> 0.5
    p25 = M.ServoParams(tau_s=2.69, omega0=7.32)
    assert s["DS3225-1"]["gate"]["kappa"] == fit.gate(2.69, p25.omega_noload, 1.95)["kappa"] == 0.6
    assert s["DS3230-1"]["gate"]["kappa"] == 0.5
    md = r.read_text()
    assert "Karsilastirma" in md and "DS3225-1" in md


def test_rejections(tmp_path):
    """300 Hz / 3.3 V reddi ve rayli pot (ortak-mod kucuk) karar tablosuna dogru yansir; eksik testler None.
    Rayli pot tek uclu okumada ray cokmesini de gorur (x*dV); sert ray (r_src 10 mOhm) ile alt/ust uc dususleri
    x ~ 0.5'te sadelesir ve hata < 1 derece kalir."""
    tr = synth.Truth(accept_300=False, accept_3v3=False, pot_reg=0.0, r_src=0.01)
    b = synth.Bench(truth=tr, seed=11)
    for k, (prog, kw) in enumerate([("sweep", dict(step_us=40, dwell_ms=150)), ("potcal", dict(points="-60,-30,0,30,60")),
                                    ("track", dict(freq=50, tag="f50")), ("track", dict(freq=300, tag="f300")),
                                    ("track", dict(tag="v33")), ("stall", dict(deltas_deg="1,3,8", hold_ms=1500,
                                                                             cool_ms=500))]):
        rec = b.run(prog, **kw)
        rec.write(tmp_path / "S" / ("%02d_%s" % (k, prog)), dict(servo_id="S", test=prog,
                                                                    host_time="2026-10-23T10:%02d" % k))
    res = fit.fit_servo(tmp_path / "S", mass=1.85)
    assert res["pwm"]["accept_300hz"] is False and res["pwm"]["accept_3v3"] is False
    v = {x["item"]: x for x in res["verdicts"]}
    assert "50 Hz" in v["DS 300 Hz"]["action"] and "Seviye cevirici" in v["3.3 V darbe"]["action"]
    assert res["pot"]["sweep"]["pot_supply"] == "ray"
    cm = res["pot"]["common_mode"]
    assert cm is not None and abs(cm["err_at_ref_deg"]) < 1.0        # rayli pot: alt/ust uc dususleri sadelesir
    assert res["actuator"]["armature"] is None and res["thermal"] is None
    assert v["Isinma: T_diz"]["result"] == "veri yok"


def test_u_fit_matches_research():
    # review tur 1 (RL-1/SC-3): Blue 2.88 N.m, 5.72 rad/s, kappa 0.6 -> 0.867 (1.80 kg), 0.876 (1.90 kg);
    # review tur 2: 1.95 kg'da kappa 0.5 esigi w0 >= 4.98; discovery-critic R2 tablosu (yuvarlanmis) +-0.02
    assert abs(M.u_fit(2.88, 5.72, 1.80, 0.6) - 0.867) < 0.002
    assert abs(M.u_fit(2.88, 5.72, 1.90, 0.6) - 0.876) < 0.002
    assert abs(M.u_fit(2.88, 4.98, 1.95, 0.5) - 0.85) < 0.005
    for kap, u in ((0.8, 1.11), (0.72, 1.00), (0.6, 0.85), (0.5, 0.74)):
        assert abs(M.u_fit(2.88, 5.72, 1.80, kap) - u) < 0.02


def test_model_basics():
    p = M.ServoParams()
    t = np.arange(0.0, 0.6, 0.001)
    c = np.where(t > 0.05, math.radians(60), 0.0)
    s = M.simulate(p, t, c, t)
    m = (t > 0.12) & (t < 0.2)
    w = np.polyfit(t[m], s["q"][m], 1)[0]
    assert abs(w / p.omega_noload - 1) < 0.01                         # yuksuz plato = (tau_s - tf)/b
    # kalinti: play + surtunme (db/2 + tf/kp) ve hareket yokken akim ~ bosta
    assert abs(s["q"][-1] - math.radians(60)) < 0.5 * p.deadband + p.frictionloss / p.kp + 1e-4
    assert abs(s["i_avg"][-1] - p.i_idle) < 0.01
    y = M.play(np.array([0, 1, 2, 1.8, 1.0, 1.0]), 0.4, y0=0.0)
    assert np.allclose(y, [0, 0.8, 1.8, 1.8, 1.2, 1.2])
    qc = M.heat_center(1.0, 1.47, math.radians(15))
    q = qc + math.radians(15) * np.sin(np.linspace(0, 2 * np.pi, 20001))
    assert abs(math.sqrt(np.mean((1.47 * np.cos(q)) ** 2)) - 1.0) < 1e-3
    assert M.heat_center(2.0, 1.47, 0.2) is None


def test_analytic_model_matches_mujoco():
    """Analitik servo = MuJoCo position aktuatoru (kp, forcerange) + eklem damping/armature/frictionloss."""
    mujoco = pytest.importorskip("mujoco")
    p = M.ServoParams(deadband=0.0, latency=0.0, frictionloss=0.05)
    xml = """<mujoco><option timestep="0.00025" integrator="Euler"><flag gravity="disable" contact="disable"/></option>
      <worldbody><body><inertial pos="0 0 0" mass="1e-6" diaginertia="1e-9 1e-9 1e-9"/>
      <joint name="j" type="hinge" axis="0 0 1" damping="%g" armature="%g" frictionloss="%g"/></body></worldbody>
      <actuator><position joint="j" kp="%g" forcelimited="true" forcerange="%g %g"/></actuator></mujoco>""" % (
        p.damping, p.armature, p.frictionloss, p.kp, -p.tau_s, p.tau_s)
    mdl = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(mdl)
    for amp in (math.radians(2.0), math.radians(30.0)):
        mujoco.mj_resetData(mdl, d)
        d.ctrl[0] = amp
        n = int(0.3 / mdl.opt.timestep)
        qs = np.empty(n)
        for i in range(n):
            mujoco.mj_step(mdl, d)
            qs[i] = d.qpos[0]
        t = (np.arange(n) + 1) * mdl.opt.timestep
        s = M.simulate(p, [0.0], [amp], t, q0=0.0, ceff0=0.0, rc_tau=0.0)
        err = np.abs(s["q"] - qs).max()
        assert err < 0.03 * amp + math.radians(0.05), (amp, err)
