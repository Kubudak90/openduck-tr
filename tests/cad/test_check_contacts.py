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


def test_pair_scope_still_counts_harness(check, monkeypatch):
    """Komsu cift izgarasi kapsam "module": govde <-> uzuv yapi temasi bilgi amacli (info), kablo demeti temasi kapsam
    disinda da sayilir (spec A1 "Kablo demeti temaslari her zaman sayilir") ve yalniz kablo / yapi temasli poz diye
    isaretlenir."""
    m = _model(_tr35_check())
    hits = {"p1": [(5.0, "left_knee_harness_in", "pca9685_1")],
            "p2": [(9.0, "left_hip_pitch_servo", "pca9685_1"), (4.0, "left_knee_harness_in", "pca9685_1")],
            "p3": [(7.0, "left_hip_pitch_servo", "pca9685_1")]}
    monkeypatch.setattr(check, "check_pose", lambda model, pose, links=None: hits[pose])
    legs = {"left_thigh", "left_foot", "left_hip_cradle", "right_foot", "right_thigh"}
    lines, info, ext = [], collections.Counter(), {}
    n = check._scan(m, [(k, k) for k in hits], None, lines, scope=lambda name: legs, info=info, ext=ext)
    assert n == 2
    assert info == {("left_hip_pitch_servo", "pca9685_1"): 2}
    assert ext == {"kablo": 1, "yapi": 1}
    assert "yalniz kablo" in lines[0] and "yapi temasli poz" in lines[1]
