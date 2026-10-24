# -*- coding: utf-8 -*-
"""
cad/check.py modellenen temas kurallari (check.contact_pairs, spec 3.2.2; FreeCAD'siz): openduck sahte modulle
yer degistirir, kurallar sahte model uzerinde denenir.

  pytest tests/cad -q -p no:cacheprovider
"""
import collections
import importlib
import json
import os
import sys
import types

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


@pytest.fixture
def check():
    fake = types.ModuleType("openduck")
    fake.joint_key = lambda n: n[n.index("_") + 1:] if n.startswith(("left_", "right_")) else n
    saved = {k: sys.modules.get(k) for k in ("openduck", "check")}
    sys.modules["openduck"] = fake
    sys.modules.pop("check", None)
    sys.path.insert(0, os.path.join(ROOT, "cad"))
    try:
        yield importlib.import_module("check")
    finally:
        sys.path.remove(os.path.join(ROOT, "cad"))
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


def _model(cfg_check):
    it = lambda name, kind: types.SimpleNamespace(name=name, kind=kind)
    links = {
        "left_foot": [it("left_foot_plate", "bracket"), it("left_sole", "sole")],
        "right_foot": [it("right_foot_plate", "bracket"), it("right_sole", "sole")],
        "left_thigh": [it("left_hip_pitch_servo", "servo"), it("left_knee_harness_in", "harness")],
        "right_thigh": [it("right_hip_pitch_servo", "servo")],
        "left_hip_cradle": [it("left_hip_pitch_horn", "horn")],
        "trunk": [it("pca9685_1", "pcb")],
    }
    return types.SimpleNamespace(cfg={"check": cfg_check},
                                 links={ln: types.SimpleNamespace(items=v) for ln, v in links.items()})


def _tr35_check():
    with open(os.path.join(ROOT, "configs", "tr35_b2.json")) as f:
        return json.load(f)["check"]


def test_no_rules_means_none(check):
    assert check.contact_rules(_model({}), "pairs") is None
    assert check.contact_rules(_model(_tr35_check()), "random") is None


def test_tr35_rules(check):
    m = _model(_tr35_check())
    ll, pr = check.contact_rules(m, "leg_leg"), check.contact_rules(m, "pairs")
    # iki ayak birbirine (capraz) yalniz bacak-bacak bolumunde
    assert ll("left_sole", "right_foot_plate")
    assert ll("left_sole", "left_foot_plate") is None            # ayni taraf: capraz kural degil
    assert pr("left_sole", "right_sole") is None
    # ayni bacak ayak <-> uyluk / kalca besigi yalniz cift izgarasinda; horn dahil
    assert pr("left_foot_plate", "left_hip_pitch_servo")
    assert pr("left_hip_pitch_horn", "left_sole")
    assert pr("left_foot_plate", "right_hip_pitch_servo") is None  # karsi bacak uyluk: izinli degil
    assert ll("left_foot_plate", "left_hip_pitch_servo") is None
    # kablo demeti her zaman sayilir; listede olmayan link ciftleri sayilir
    assert pr("left_sole", "left_knee_harness_in") is None
    assert pr("left_sole", "pca9685_1") is None


LEGS = {"left_thigh", "left_foot", "left_hip_cradle", "right_foot", "right_thigh"}


def test_pair_scope_still_counts_harness(check, monkeypatch):
    """Komsu cift izgarasi kapsam "module": govde <-> uzuv yapi temasi bilgi amacli (info), kablo demeti temasi kapsam
    disinda da sayilir (spec A1) ve yalniz kablo / kablo once diye isaretlenir; yol taramasi (first) verilmezse yapi
    temasli pozdaki kablo da sayilir (tutucu)."""
    m = _model(_tr35_check())
    hits = {"p1": [(5.0, "left_knee_harness_in", "pca9685_1")],
            "p2": [(9.0, "left_hip_pitch_servo", "pca9685_1"), (4.0, "left_knee_harness_in", "pca9685_1")],
            "p3": [(7.0, "left_hip_pitch_servo", "pca9685_1")]}
    monkeypatch.setattr(check, "check_pose", lambda model, pose, links=None: hits[pose])
    lines, info, ext = [], collections.Counter(), {}
    n = check._scan(m, [(k, k) for k in hits], None, lines, scope=lambda name: LEGS, info=info, ext=ext)
    assert n == 2
    assert info == {("left_hip_pitch_servo", "pca9685_1"): 2}
    assert ext == {"yalniz kablo": 1, "kablo once": 1}
    assert "yalniz kablo" in lines[0] and "kablo once" in lines[1]


# -- D2: "yapi once" kurali (spec A1 v3.2) -------------------------------------------------------------------------
def test_path_poses_one_degree_steps_of_largest_change(check):
    stand = {"a": 0.0, "b": 10.0, "c": 3.0}
    pose = {"a": 5.0, "b": -20.0, "c": 3.0}
    path = check.path_poses(stand, pose)
    assert len(path) == 30 and [k for k, _, _ in path] == list(range(1, 31))
    assert all(n == 30 for _, n, _ in path)
    k, n, p = path[0]
    assert p["b"] == pytest.approx(9.0) and p["a"] == pytest.approx(5.0 / 30) and p["c"] == 3.0
    assert path[-1][2] == pytest.approx(pose)
    assert len(check.path_poses(stand, dict(stand, a=0.4))) == 1          # en az bir adim (poz kendisi)
    assert len(check.path_poses(stand, dict(stand, a=-2.5))) == 3         # tavan


def test_first_contact_stops_at_first_step(check):
    stand, pose = {"q": 0.0}, {"q": 90.0}
    calls = []

    def contacts(p):
        calls.append(p["q"])
        return [(2.0, "s", "t")] if p["q"] >= 41.0 else []
    k, n, p, hits = check.first_contact(stand, pose, contacts)
    assert (k, n) == (41, 90) and p["q"] == pytest.approx(41.0) and hits == [(2.0, "s", "t")]
    assert len(calls) == 41                                               # ilk temasta durur
    assert check.first_contact(stand, pose, lambda p: []) is None


def test_harness_rule_classes(check):
    cab = lambda a: "harness" in a
    st, hz = (9.0, "left_shin_bracket", "torso_shell_left"), (4.0, "left_knee_harness_out", "pca9685_1")
    never = lambda: pytest.fail("yol taramasi gereksiz")
    assert check.harness_rule([st], cab, never) == ([st], [], None, None)
    assert check.harness_rule([hz], cab, never) == ([], [hz], "yalniz kablo", None)
    fc_struct = (87, 116, {}, [(1.5, "left_shin_bracket", "torso_shell_left")])
    assert check.harness_rule([st, hz], cab, lambda: fc_struct)[2:] == ("yapi once", fc_struct)
    fc_cable = (60, 116, {}, [(1.1, "left_knee_harness_in", "torso_shell_left")])
    assert check.harness_rule([st, hz], cab, lambda: fc_cable)[2] == "kablo once"
    # ilk temas adiminda yapi ve kablo birlikte: kablo once sayilir (yapi kablodan once degmedi)
    fc_both = (60, 116, {}, [(1.1, "left_shin_bracket", "x"), (1.2, "left_knee_harness_in", "y")])
    assert check.harness_rule([st, hz], cab, lambda: fc_both)[2] == "kablo once"
    assert check.harness_rule([st, hz], cab, lambda: None)[2] == "kablo once"


def test_scan_structure_first_excuses_harness(check, monkeypatch):
    """Yapi temasli pozda HOME -> poz yolunda ilk temas yapiysa kablo temaslari sayilmaz, excused'a (ilk temas adimi
    ve parcalari ile) gider; ilk temas kabloysa sayilir."""
    m = _model(_tr35_check())
    hits = {"pA": [(9.0, "left_hip_pitch_servo", "pca9685_1"), (4.0, "left_knee_harness_in", "pca9685_1")],
            "pB": [(9.0, "left_hip_pitch_servo", "pca9685_1"), (3.0, "left_knee_harness_in", "pca9685_1")],
            "pC": [(1.5, "left_knee_harness_in", "left_foot_plate")]}          # modul ici: her zaman sayilir
    monkeypatch.setattr(check, "check_pose", lambda model, pose, links=None: hits[pose])
    firsts = {"pA": (50, 80, {}, [(2.0, "left_hip_pitch_servo", "pca9685_1")]),
              "pB": (30, 80, {}, [(1.2, "left_knee_harness_in", "pca9685_1")])}
    lines, info, ext, exc = [], collections.Counter(), {}, []
    n = check._scan(m, [(k, k) for k in hits], None, lines, scope=lambda name: LEGS, info=info, ext=ext,
                    first=lambda name, pose: firsts[name], excused=exc)
    assert n == 2                                                          # pB kablo + pC modul ici
    assert ext == {"yapi once": 1, "kablo once": 1}
    assert exc == [("pA", firsts["pA"], [(4.0, "left_knee_harness_in", "pca9685_1")])]
    assert not any(l.split()[0] == "pA" for l in lines)
    assert any("kablo once, ilk temas adim 30/80" in l for l in lines)


# -- D4: cikis kodu ----------------------------------------------------------------------------------------------
@pytest.mark.parametrize("design,same,code", [(0, 0, 0), (3, 0, 1), (0, 2, 1)])
def test_main_exit_code(check, monkeypatch, tmp_path, design, same, code):
    out = tmp_path / "interference.txt"
    monkeypatch.setenv("OPENDUCK_CHECK_OUT", str(out))
    monkeypatch.setattr(check, "run", lambda n_random=40: (design, 40, 40, same, "# OpenDuck carpisma raporu"))
    assert check.main() == code
    assert out.read_text() == "# OpenDuck carpisma raporu\n"


# -- D5: ayni gecis istisnasi yalniz gecis noktasinin 2 d cevresinde --------------------------------------------
def test_crossing_sphere(check):
    hc = {"crossings": {"knee": {"d": 4.0, "cross": [-40.768, 114.5, -50.639]},
                        "neck_pitch": {"d": 5.0, "cross": [33.332, -34.4, 41.537]}}}
    assert check.crossing_sphere(hc, "left_knee") == ((-40.768, 114.5, -50.639), 8.0)
    assert check.crossing_sphere(hc, "right_knee") == ((-40.768, -114.5, -50.639), 8.0)
    assert check.crossing_sphere(hc, "neck_pitch") == ((33.332, -34.4, 41.537), 10.0)
    assert check.crossing_sphere(hc, "left_ankle") is None
    assert check.crossing_sphere(None, "left_knee") is None
    # kesisimin ayrik parcalari (hacim, kutle merkezi): merkezi 2 d icindekiler sayilmaz, disindakiler sayilir
    c = (0.0, 0.0, 0.0)
    assert check.far_overlap([(30.0, (5.0, 0.0, 0.0)), (12.0, (0.0, 7.9, 0.0))], c, 8.0) == 0.0
    assert check.far_overlap([(30.0, (5.0, 0.0, 0.0)), (12.0, (0.0, 8.1, 0.0))], c, 8.0) == 12.0
    assert check.far_overlap([], c, 8.0) == 0.0
    it = lambda name: types.SimpleNamespace(name=name, kind="harness")
    assert check.allowed_pair(it("left_knee_harness_in"), it("left_knee_harness_out"))
    assert not check.allowed_pair(it("left_knee_harness_in"), it("right_knee_harness_out"))
    assert not check.allowed_pair(it("left_knee_harness_out"), it("left_hip_pitch_harness_out"))
