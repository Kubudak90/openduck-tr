# -*- coding: utf-8 -*-
"""cad/build_config.py + configs/*.json (spec 3.1, 3.2.1): yukleme, dogrulama, servo haritasi, horn ofsetleri ve
servo hareket araligi. FreeCAD gerektirmez."""
import copy
import os

import pytest

import build_config as bc
import servos

SPEC_MAP = {"hip_roll": "DS32XX", "hip_pitch": "DS32XX", "knee": "DS32XX", "ankle": "DS32XX", "neck_pitch": "DS32XX",
            "hip_yaw": "ES3054", "head_pitch": "MG90S", "head_yaw": "MG90S", "head_roll": "MG90S", "mouth": "SG90"}


def test_default_is_xl330(monkeypatch):
    monkeypatch.delenv(bc.ENV, raising=False)
    assert bc.resolve_path() == os.path.join(bc.CONFIG_DIR, "xl330_27.json")
    monkeypatch.setenv(bc.ENV, "tr35_b2")
    assert bc.load()["name"] == "tr35_b2"
    monkeypatch.setenv(bc.ENV, "configs/xl330_27.json")       # repo kokune gore goreli yol
    assert bc.load()["name"] == "xl330_27"
    with pytest.raises(IOError, match="konfig bulunamadi"):
        bc.resolve_path("yok_boyle_konfig")


def test_xl330_27():
    c = bc.load("xl330_27")
    assert c["scale"] == 1.0 and c["params"] == {}
    assert c["servo_map"] == {k: "XL330" for k in servos.JOINT_KEYS}
    assert c["generators"] == {s: "xl330_27" for s in bc.STAGES}
    assert c["horn_offset_deg"] == {} and c["neutral_deg"] == {}
    assert c["robot"] == "OpenDuck (Microduck benzeri)"
    assert c["bom_buy"][0] == [15, "ROBOTIS DYNAMIXEL XL330-M288-T",
                               "bacaklar 2x5, boyun+kafa 4, gaga 1 (ID 10-14, 20-24, 30-34)"]
    assert c["path"].endswith(os.path.join("configs", "xl330_27.json"))


def test_tr35_b2_matches_spec():
    c = bc.load("tr35_b2")
    assert c["scale"] == 1.282
    assert c["servo_map"] == SPEC_MAP
    # spec 3.1: hip_pitch horn'u bir dis (25T) ofsetli, boyun notru -25
    assert c["horn_offset_deg"] == {"hip_pitch": 14.4}
    assert c["horn_offset_deg"]["hip_pitch"] == pytest.approx(servos.spline_step_deg(servos.get("DS32XX")))
    assert c["neutral_deg"] == {"neck_pitch": -25.0}
    assert bc.servo_center(c, "hip_pitch") == 14.4 and bc.servo_center(c, "neck_pitch") == -25.0
    assert c["joint_limits_deg"]["hip_pitch"] == [-70.0, 90.0] and c["joint_limits_deg"]["head_yaw"] == [-80.0, 80.0]
    assert sorted(c["required_deg"]) == sorted(servos.JOINT_KEYS)
    # spec 3.2.2 / A3: 4 aktuator sinifi
    classes = {servos.actuator(servos.get(n))["mjcf_class"] for n in c["servo_map"].values()}
    assert classes == {"ds32xx", "es3054", "mg90s", "sg90"}
    # et/dolgu parametreleri (mutlak, olceklenmez); aciklama ("_") anahtarlari atilmis
    p = c["params"]
    assert p["wall"] == 1.8 and p["plate_t"] == 4.0
    assert p["print"]["bracket"]["material"] == "PETG" and 0 < p["print"]["bracket"]["infill"] < 1
    assert p["print"]["shell"]["material"] == "PLA" and p["print"]["sole"]["material"] == "TPU 95A"
    assert all(p["print"][k]["density_eff"] > 0 for k in ("bracket", "shell", "trim", "face", "sole", "pad"))
    assert not [k for k in p if k.startswith("_")]
    assert c["targets"]["mass_max_kg"] == 1.95 and c["targets"]["bed_mm"] == [256.0, 256.0, 256.0]


def test_tr35_required_ranges_fit_servo_travel():
    """Gerekli araliklar (spec 3.1) servo hareketine (merkez +- hareket/2) sigar; hip_pitch +14.4 ve boyun -25
    merkezleriyle her eklemde >= 5 derece pay kalir (ofsetsiz hip_pitch +90'da ve boyun -90'da pay 0 olurdu)."""
    c = bc.load("tr35_b2")
    assert bc.travel_violations(c) == []
    assert bc.travel_violations(c, c["joint_limits_deg"]) == []
    for k, (lo, hi) in c["required_deg"].items():
        slo, shi = servos.travel_range(servos.get(c["servo_map"][k]), bc.servo_center(c, k))
        assert min(lo - slo, shi - hi) >= 5.0, k
    flat = copy.deepcopy(c)
    flat["horn_offset_deg"], flat["neutral_deg"] = {}, {}
    for k in ("hip_pitch", "neck_pitch"):
        lo, hi = c["required_deg"][k]
        slo, shi = servos.travel_range(servos.get(c["servo_map"][k]), bc.servo_center(flat, k))
        assert min(lo - slo, shi - hi) == 0.0


def _base():
    return bc.load("tr35_b2")


@pytest.mark.parametrize("mutate, err", [
    (lambda c: c["servo_map"].pop("mouth"), "servo_map"),
    (lambda c: c["servo_map"].update(mouth="MG996R"), "bilinmeyen servo"),
    (lambda c: c["horn_offset_deg"].update(hip_pitch=10.0), "spline adiminin"),
    (lambda c: c["horn_offset_deg"].update(head_pitch=17.0), "spline adiminin"),
    (lambda c: c["neutral_deg"].update(elbow=1.0), "bilinmeyen eklem"),
    (lambda c: c.update(scale=0), "scale"),
    (lambda c: c["generators"].pop("head"), "generators"),
    (lambda c: c.update(colour="red"), "bilinmeyen"),
    (lambda c: c["required_deg"].update(knee=[10.0, -10.0]), "alt, ust"),
    (lambda c: c.update(bom_buy=[[1, "x"]]), "bom_buy"),
])
def test_validate_rejects(mutate, err):
    c = _base()
    c.pop("path")
    mutate(c)
    with pytest.raises((ValueError, KeyError), match=err):
        bc.load(c)


def test_horn_offset_multiples_accepted():
    c = _base()
    c.pop("path")
    c["horn_offset_deg"] = {"hip_pitch": -28.8, "knee": 0.0}
    assert bc.load(c)["horn_offset_deg"]["hip_pitch"] == -28.8


def test_scaled_and_merge():
    v = (1.5, -2.0, 3.25)
    assert bc.scaled(v, 1.0) is v                         # s = 1: dokunulmaz (bayt-esitlik)
    assert bc.scaled(v, 2.0) == (3.0, -4.0, 6.5)
    assert bc.scaled({"a": (1.0, 2.0), "b": 3.0}, 0.5) == {"a": (0.5, 1.0), "b": 1.5}
    base = {"x": 1.0, "d": {"a": (1, 2), "b": 2}, "t": (1, 2)}
    out = bc.merge(base, {"d": {"b": 5}, "t": [3]})
    assert out == {"x": 1.0, "d": {"a": (1, 2), "b": 5}, "t": [3]}
    assert base["d"]["b"] == 2                             # kaynak degismez
    assert bc.strip_notes({"_n": 1, "a": {"_m": 2, "b": [{"_x": 0, "y": 1}]}}) == {"a": {"b": [{"y": 1}]}}
