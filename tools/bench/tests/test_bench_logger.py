# -*- coding: utf-8 -*-
"""Kaydedici testi: sahte seri port, firmware satir akisini (synth.Bench) oynatir; istemler, jog, kayit, zip.

  python -m pytest tools/bench/tests -q -p no:cacheprovider
"""
import queue
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.bench import logger  # noqa: E402
from tools.sysid import fit, logio, synth  # noqa: E402


class FakeSerial:
    """write(bytes)/readline() arayuzu; 'run' komutunu synth ile uretip P satirlarinda yanit bekler."""

    def __init__(self, bench):
        self.b = bench
        self.out = queue.Queue()
        self.pending = []
        self.writes = []

    def write(self, data):
        for ln in data.decode().splitlines():
            self.writes.append(ln)
            if ln == "hb":
                continue
            if ln == "cfg":
                self.out.put(self.b.cfg_line())
            elif ln.startswith(("set", "cls")):
                self.out.put("OK " + ln)
            elif ln.startswith("run "):
                parts = ln.split()[1:]
                kw = {k: logio._num(v) for k, v in (p.split("=", 1) for p in parts[1:])}
                self.pending = list(self.b.run(parts[0], **kw).raw)
                self._flush()
            elif ln.startswith("ans"):
                self._flush()
            elif ln.startswith(("scan", "osc")):
                self.out.put("R,osc_hz,25012345")

    def _flush(self):
        while self.pending:
            s = self.pending.pop(0)
            if s.startswith("CFG"):
                continue
            self.out.put(s)
            if s.startswith("P,"):
                break

    def readline(self):
        try:
            return (self.out.get(timeout=0.02) + "\n").encode()
        except queue.Empty:
            return b""


def _answers(pid, text):
    if pid in ("horiz", "touch") or pid.startswith("pc"):
        return ["+5", "-2", "ok"]
    return {"obs": "e", "amb": "24"}.get(pid, "100")


def test_logger_runs_tests_and_saves(tmp_path):
    b = synth.Bench(seed=2, cfg=dict(mass_g=1500.0))
    fake = FakeSerial(b)
    link = logger.Link(fake, hb=False)
    quiet = dict(echo=lambda *a, **k: None, answer_fn=_answers, confirm=lambda *a: "")
    try:
        extra = dict(servo_model="DS3230", servo_class="ds")
        p_sw = logger.run_test(link, "sweep", "DS-L1", tmp_path, extra, args={"step_us": "40"}, **quiet)
        p_pc = logger.run_test(link, "potcal", "DS-L1", tmp_path, extra, **quiet)
        p_st = logger.run_test(link, "stall", "DS-L1", tmp_path, extra,
                               args={"deltas_deg": "1,3,8", "hold_ms": "1500", "cool_ms": "500"}, **quiet)
        p_tr = logger.run_test(link, "accept300", "DS-L1", tmp_path, extra, **quiet)
        p_os = logger.run_test(link, "overshoot", "DS-L1", tmp_path, extra, args={"reps": "2"}, **quiet)
        p_cmd = logger.run_test(link, "osc", "DS-L1", tmp_path, extra, **quiet)
    finally:
        link.close()
    assert len(p_tr) == 2 and len(p_os) == 2 and p_cmd[0].endswith(".cmd.json")
    r = logio.Run(p_st[0])
    assert r.meta["status"] == "ok" and r.meta["servo_id"] == "DS-L1" and r.meta["servo_model"] == "DS3230"
    assert r.meta["test"] == "stall" and r.program == "stall" and len(r.t) > 100
    assert "g_0" in r.meta["answers"] and r.cfg["lever_mm"] == 100.0
    assert [kv["kind"] for _, _, kv in r.segs()].count("stall") == 4          # 3 nokta + multimetre
    assert "+5" in fake.writes and "-2" in fake.writes and "ans ok" in fake.writes
    rt = [logio.Run(p) for p in p_tr]
    assert sorted(x.tag for x in rt) == ["f300", "f50"]
    assert logio.Run(p_os[0]).burst is not None
    # kaydedicinin yazdigi dosyalar sysID'ye girer
    res = fit.fit_servo(tmp_path / "DS-L1", mass=1.95)
    assert res["pwm"]["accept_300hz"] is True
    assert abs(res["actuator"]["tau_s"] / b.tr.p.tau_s - 1) < 0.05
    z = logger.pack(tmp_path / "DS-L1")
    with zipfile.ZipFile(z) as zf:
        assert any(n.endswith("_stall_stall.json") for n in zf.namelist())


def test_cli_list(capsys):
    assert logger.main(["--list"]) == 0
    out = capsys.readouterr().out
    assert "accept300" in out and "plan ds" in out
