# -*- coding: utf-8 -*-
"""cad/servos.py kayit defteri (spec A9 "servos"): kayitlarin tamligi, anahtar olculer (servo-mechanics.md),
ic tutarlilik ve aktuator varsayilanlari. FreeCAD gerektirmez."""
import math

import pytest

import servos

REQUIRED = ("name", "label", "models", "interface", "geometry", "body", "shaft", "boss", "spline", "horn", "tabs",
            "holes", "cable", "mass_g", "travel_deg", "pwm_us", "neutral_us", "deadband_us", "frame_hz", "electrical",
            "actuator", "estimated")
PWM = ("DS32XX", "ES3054", "MG90S", "SG90")
ACT = ("class", "mjcf_class", "kp", "forcerange", "damping", "frictionloss", "armature", "velocity", "pband_deg",
       "deadband_deg", "source")
ELEC = ("v_nom", "v_min", "v_max", "stall_torque", "omega0", "stall_current", "idle_current")


def _path(rec, dotted):
    v = rec
    for k in dotted.split("."):
        v = v[k]
    return v


def test_registry_names_and_lookup():
    assert list(servos.SERVO_TYPES) == ["XL330", "DS32XX", "ES3054", "MG90S", "SG90"]
    assert servos.get("ds32xx") is servos.SERVO_TYPES["DS32XX"]
    with pytest.raises(KeyError, match="bilinmeyen servo"):
        servos.get("MG996R")
    assert len(servos.JOINT_KEYS) == 10 and len(set(servos.JOINT_KEYS)) == 10


@pytest.mark.parametrize("name", list(servos.SERVO_TYPES))
def test_record_complete(name):
    t = servos.get(name)
    assert t["name"] == name
    assert all(k in t for k in REQUIRED), [k for k in REQUIRED if k not in t]
    assert all(k in t["actuator"] for k in ACT) and all(k in t["electrical"] for k in ELEC)
    for k in ("l", "w", "h", "corner_r"):
        assert t["body"][k] > 0
    assert 0 < t["shaft"]["near"] < t["body"]["l"] / 2
    assert t["shaft"]["x"] == 0.0            # mil genislikte ortada: sag taraf mirror_y ile gecerli
    assert t["cable"]["face"] in ("near_end", "far_end")
    assert t["mass_g"] > 0 and t["horn"]["mass_g"] > 0
    assert t["interface"] in ("ttl", "pwm")
    for dotted in t["estimated"]:
        _path(t, dotted)                      # tahmin isaretleri gercek alanlari gostermeli


@pytest.mark.parametrize("name", PWM)
def test_pwm_envelope_consistency(name):
    t = servos.get(name)
    b, sp, tb, ho = t["body"], t["spline"], t["tabs"], t["holes"]
    assert sp["top_h"] > b["h"] and sp["od"] > 0 and sp["teeth"] >= 20
    assert tb["span"] > b["l"] and tb["w"] == pytest.approx(b["w"])
    assert 0 < tb["under_h"] < b["h"]
    assert tb["top_h"] == pytest.approx(tb["under_h"] + tb["t"], abs=0.06)
    assert ho["pitch_l"] + 2 * ho["from_tab_end"] == pytest.approx(tb["span"], abs=0.11)   # DS: cizim 54.5, STEP 54.4
    assert b["l"] < ho["pitch_l"] < tb["span"]
    assert t["cable"]["face"] == "near_end"   # dort PWM turunde kablo mil tarafi uc yuzunden cikar
    lo, hi = t["pwm_us"]
    assert lo < t["neutral_us"] < hi and t["neutral_us"] == 1500
    assert t["geometry"] == "pwm"             # P2-0: kanonik zarf (servos.canon), orijin horn dis yuzunde


def test_xl330_matches_legacy_model():
    t = servos.get("XL330")
    assert (t["body"]["w"], t["body"]["l"], t["body"]["h"], t["shaft"]["near"]) == (20.0, 34.0, 26.0, 9.5)
    assert t["body"]["corner_r"] == 3.5
    hn = t["horn"]
    assert (hn["r"], hn["t"], hn["center_hole_r"], hn["bolt_n"], hn["bolt_r"], hn["bolt_a0_deg"], hn["bolt_hole_r"]) \
        == (6.5, 3.0, 1.2, 4, 4.2, 45.0, 0.8)
    assert t["cable"] == {"face": "far_end", "slot_h": (6.0, 7.0), "slot_depth": 6.0}
    assert servos.mass_kg(t) == 0.0175 and servos.horn_mass_kg(t) == 0.0005
    a = servos.actuator(t)
    assert (a["kp"], a["forcerange"], a["damping"], a["frictionloss"], a["armature"], a["velocity"]) \
        == (1.1, 0.96, 0.053, 0.0048, 0.0018, 10.0)
    assert a["class"] == "xl330" and a["mjcf_class"] == servos.ROOT_CLASS == "openduck"
    assert servos.horn_face_y(t) == 14.5
    assert t["travel_deg"] == 360.0 and t["spline"] is None and servos.spline_step_deg(t) is None
    # eski _servo_canon / horn_bolts deliklerinin acilari: pi/4 + k pi/2 (bit-duzeyinde)
    xz = servos.horn_bolt_xz(t)
    assert xz == [(4.2 * math.cos(math.pi / 4 + k * math.pi / 2), 4.2 * math.sin(math.pi / 4 + k * math.pi / 2))
                  for k in range(4)]


def test_ds32xx_key_dimensions():
    t = servos.get("DS32XX")
    assert (t["body"]["l"], t["body"]["w"], t["body"]["h"]) == (40.0, 20.0, 40.4)
    assert t["shaft"]["near"] == 10.0 and t["body"]["l"] - t["shaft"]["near"] == 30.0
    assert t["spline"]["teeth"] == 25 and t["spline"]["od"] == 6.0 and t["spline"]["top_h"] == 45.0
    assert servos.spline_step_deg(t) == pytest.approx(14.4)
    assert (t["tabs"]["span"], t["tabs"]["t"], t["tabs"]["under_h"]) == (54.5, 3.2, 27.7)
    ho = t["holes"]
    assert (ho["n"], ho["d"], ho["pitch_l"], ho["pitch_w"]) == (4, 4.5, 49.5, 10.0)
    # delik deseni govde ortasinda -> milden 10 mm uzak uca dogru
    assert t["body"]["l"] / 2 - t["shaft"]["near"] == pytest.approx(10.0)
    assert t["cable"]["h"] == 8.55 and t["cable"]["lead"] == 300.0
    assert t["horn"]["kind"] == "al_arm_25t" and t["horn"]["h"] == (42.2, 47.9)
    assert t["mass_g"] == 65.0
    assert t["travel_deg"] == 180.0 and t["pwm_us"] == (500, 2500) and t["deadband_us"] == 3.0
    assert set(t["models"]) == {"DS3225MG", "DS3230MG", "DS3235"}


def test_es3054_key_dimensions():
    t = servos.get("ES3054")
    assert (t["body"]["l"], t["body"]["w"], t["body"]["h"]) == (28.45, 13.0, 26.7)
    assert t["shaft"]["near"] == 6.0
    assert (t["spline"]["teeth"], t["spline"]["od"], t["spline"]["top_h"]) == (23, 5.9, 31.1)
    assert (t["tabs"]["span"], t["tabs"]["t"], t["tabs"]["under_h"]) == (38.35, 1.9, 17.8)
    assert (t["holes"]["n"], t["holes"]["d"], t["holes"]["pitch_l"]) == (2, 4.1, 35.0)
    assert t["travel_deg"] == 145.0 and t["pwm_us"] == (700, 2300) and t["mass_g"] == 20.0


def test_micro_servo_key_dimensions():
    mg, sg = servos.get("MG90S"), servos.get("SG90")
    assert (mg["body"]["l"], mg["body"]["w"], mg["body"]["h"]) == (22.8, 12.2, 28.4)
    assert (mg["spline"]["top_h"], mg["tabs"]["span"], mg["tabs"]["under_h"], mg["holes"]["pitch_l"]) == \
        (32.5, 32.1, 18.5, 27.8)
    assert mg["mass_g"] == 13.4 and mg["shaft"]["near"] == 5.9
    assert (sg["body"]["l"], sg["body"]["w"], sg["body"]["h"]) == (22.7, 12.2, 27.0)
    assert (sg["spline"]["top_h"], sg["tabs"]["span"], sg["tabs"]["under_h"], sg["holes"]["pitch_l"]) == \
        (30.3, 32.3, 17.0, 27.85)
    assert sg["mass_g"] == 9.0 and sg["shaft"]["near"] == 5.9
    assert mg["travel_deg"] == sg["travel_deg"] == 180.0


@pytest.mark.parametrize("name", PWM)
def test_pwm_actuator_defaults(name):
    """spec 3.2.3: kp = tau_s / P-bandi, forcerange = tau_s, b = tau_s / omega0; armature ve frictionloss XL330
    BAM degerlerinden tau_s / 0.96 ile (Adim 0 sysID'ye kadar)."""
    t = servos.get(name)
    a, e = servos.actuator(t), t["electrical"]
    assert a["mjcf_class"] == a["class"] == name.lower()
    assert a["forcerange"] == e["stall_torque"] and a["velocity"] == e["omega0"]
    assert a["kp"] == pytest.approx(e["stall_torque"] / math.radians(a["pband_deg"]), rel=5e-3)
    assert a["damping"] == pytest.approx(e["stall_torque"] / e["omega0"], rel=5e-3)
    assert a["armature"] == pytest.approx(0.0018 * a["forcerange"] / 0.96, rel=1e-2)
    assert a["frictionloss"] == pytest.approx(0.0048 * a["forcerange"] / 0.96, rel=1e-2)
    assert a["deadband_deg"] > 0


def test_ds_default_is_spec_value():
    """spec 3.2.3 varsayilani: DS3230MG Blue 2.88 N.m, 5.72 rad/s, P-bandi 3 derece; DS 6.5 V, digerleri 6.0 V."""
    t = servos.get("DS32XX")
    assert (t["electrical"]["stall_torque"], t["electrical"]["omega0"], t["actuator"]["pband_deg"]) == (2.88, 5.72, 3.0)
    assert t["electrical"]["v_nom"] == 6.5
    assert t["variants"]["DS3230MG_blue"][:2] == (2.88, 5.72)
    assert all(servos.get(n)["electrical"]["v_nom"] == 6.0 for n in ("ES3054", "MG90S", "SG90"))


def test_xl330_kp_rule_and_travel():
    t = servos.get("XL330")
    assert t["actuator"]["kp"] == pytest.approx(0.96 / math.radians(t["actuator"]["pband_deg"]), rel=1e-2)
    assert servos.travel_range(t) == (-180.0, 180.0)
    assert servos.travel_range(servos.get("DS32XX"), 14.4) == pytest.approx((-75.6, 104.4))


def test_mjcf_classes_unique():
    classes = [servos.actuator(t)["mjcf_class"] for t in servos.SERVO_TYPES.values()]
    assert len(classes) == len(set(classes))
    assert set(classes) - {servos.ROOT_CLASS} == {"ds32xx", "es3054", "mg90s", "sg90"}


def test_require_guard():
    class M(object):
        p = {"scale": 1.282}
    sv = {k: servos.get("XL330") for k in servos.JOINT_KEYS}
    with pytest.raises(NotImplementedError, match="olcekli"):
        servos.require(M, sv, ("knee",), "XL330", "legs.xl330_27", scale=1.0)
    sv["knee"] = servos.get("DS32XX")
    with pytest.raises(NotImplementedError, match="knee=DS32XX"):
        servos.require(M, sv, ("knee",), "XL330", "legs.xl330_27")


# ---------------------------------------------------------------------------
# P2-0 kanonik zarf sozlesmesi (FreeCAD'siz kisim: datumlar, idler, doner tabla, horn delikleri)
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("name", PWM)
def test_pwm_origin_convention(name):
    """Orijin horn dis yuzunde (horn_face_y = 0), taban (arka yuz) Y = -horn.h[1]; govde/kule/spline/horn sirasi
    tabandan yukari tutarli; kulak deseni govde ortasinda (milden uzak uca body_center_z)."""
    t = servos.get(name)
    d = servos.datums(t)
    assert servos.horn_face_y(t) == 0.0 == d["horn_face"]
    assert d["rear_face"] == pytest.approx(-t["horn"]["h"][1]) == pytest.approx(servos.rear_face_y(t))
    assert servos.face_h(t) == t["horn"]["h"][1]
    assert d["rear_face"] < d["case_top"] <= d["tower_top"] < d["spline_top"] < d["horn_face"]
    assert d["tower_top"] == pytest.approx(d["rear_face"] + t["body"]["h"])
    assert d["rear_face"] + t["horn"]["h"][0] > d["tower_top"] + 0.3     # horn gobegi govdeye degmez
    assert d["spline_top"] < d["horn_face"]
    assert d["body_center_z"] == pytest.approx(t["body"]["l"] / 2 - t["shaft"]["near"])
    z0, z1 = d["tab_z"]
    assert z1 - z0 == pytest.approx(t["tabs"]["span"]) and (z0 + z1) / 2 == pytest.approx(d["body_center_z"])
    assert d["tab"][0] == pytest.approx(d["rear_face"] + t["tabs"]["under_h"])
    hz = sorted({z for _, z in d["tab_holes"]})
    assert hz[1] - hz[0] == pytest.approx(t["holes"]["pitch_l"])
    # X = 0 simetrisi (mirror_y gecerli): delikler, kablo cikisi, horn delikleri ortada / cift
    assert sorted(x for x, _ in d["tab_holes"]) == sorted(-x for x, _ in d["tab_holes"])
    assert d["cable_exit"][0][0] == 0.0 and d["cable_exit"][1] == (0.0, 0.0, -1.0)
    assert all(x == 0.0 for x, _ in servos.horn_bolt_xz(t))
    assert len(servos.horn_bolt_xz(t)) == t["horn"]["bolt_n"]


def test_ds_idler_stack():
    """DS arka idler (spec 3.1, servo-mechanics D2/D5): F623ZZ, besik plakasi tabandan 0.4..4.4, rulman -1.4..-5.4
    (merkez -3.4, flans disarida), 0.5 calisma boslugu, U-link B kolu -5.9..-9.9; eksen mil ile es eksenli."""
    t = servos.get("DS32XX")
    st = servos.idler_stack(t)
    yr = servos.rear_face_y(t)
    assert yr == pytest.approx(-47.9)
    assert st["seat"] == pytest.approx((yr - 4.4, yr - 0.4))
    assert st["bearing"] == pytest.approx((yr - 5.4, yr - 1.4))
    assert st["axis"] == pytest.approx(yr - 3.4)
    assert st["leg"] == pytest.approx((yr - 9.9, yr - 5.9))
    assert st["bearing_rec"] is servos.BEARINGS["F623ZZ"]
    assert servos.datums(t)["idler_axis"] == (0.0, st["axis"], 0.0)
    for n in ("ES3054", "MG90S", "SG90", "XL330"):
        assert servos.idler_stack(servos.get(n)) is None


def test_turntable_stack():
    """hip_yaw doner tablasi: iki rulman, merkezler arasi >= 20 mm; ES3054 horn yuzu ust rulmanin ustunde; taban
    rulmani degisse de horn yuzu ayni (dondurulmus arayuz)."""
    tt = {"bearing": "6800ZZ", "spacing": 22.0, "z_mate": 0.0, "gap": 0.5, "nut_t": 2.5, "coupler_gap": 0.5,
          "housing_wall": 3.0}
    s = servos.turntable_stack(tt)
    b = servos.BEARINGS["6800ZZ"]
    assert s["lower"] == (0.0, b["B"]) and s["upper"] == (22.0, 22.0 + b["B"])
    assert s["spacing"] >= 20.0
    assert s["horn_face"] == pytest.approx(22.0 + b["B"] + 2.5 + 0.5)
    assert s["housing_r"] == pytest.approx(b["D"] / 2 + 3.0)
    s2 = servos.turntable_stack(tt, "608ZZ")
    assert s2["horn_face"] == s["horn_face"] and s2["upper"][1] < s["horn_face"]


def test_arm_angle():
    assert servos.arm_angle_deg((0, 0, -1), (-1, 0, 0), (-1, 0, 0)) == pytest.approx(0.0)
    assert abs(servos.arm_angle_deg((0, 0, -1), (-1, 0, 0), (1, 0, 0))) == pytest.approx(180.0)
    # horn -y, uzak uc +z: kolu -z'ye cevirmek 180, +x'e cevirmek: -y etrafinda +z -> +x = -90
    assert servos.arm_angle_deg((0, -1, 0), (0, 0, 1), (1, 0, 0)) == pytest.approx(-90.0)
    assert servos.arm_angle_deg((1, 0, 0), (0, 0, -1), (0, 1, 0)) == pytest.approx(90.0)
