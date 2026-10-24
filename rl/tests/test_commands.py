# -*- coding: utf-8 -*-
"""
rl/commands.py (spec eki K5) testleri: kimlik tablosu, turetilmis sabitler, K5 tablosu satir satir (spec tablosunun
bagimsiz kopyasina karsi), ic olaylar (dusme, tamamlanma, basarisiz gecis kurallari, devir, BEKLE), rastgele dizilerde
K5 degismezleri, jest fonksiyonu (task_math), gercek sensor modeliyle gyro bias, play.Supervisor = FallMonitor ve altin
izlerin yeniden oynatilmasi (tr35 alt sureci; ayrik birebir, surekli 1e-6).

  python3 -m pytest rl/tests/test_commands.py

Bu dosya surec profilinden bagimsizdir (xl330 surecinde de calisir); test_tr35_process ayni testleri tr35_b2 alt
surecinde (turetilmis dosya; sahne gerekmez) tekrar kosar.
"""
import copy
import json
import math
import os
import subprocess
import sys

import numpy as np
import pytest

from rl import commands as CM
from rl import config as C
from rl import sensors as SENS
from rl import task_math as T
from rl.tests import komut_plant as KP

ROOT = C.ROOT
DATA = os.path.join(ROOT, "rl", "tests", "data")
FS = CM.K["fall_steps"]

# spec eki K5 tablosunun BAGIMSIZ kopyasi (kelime satirlari): (yeni durum, isaret). "?" otur/A ve kalk/B kosula bagli.
SPEC = {
    "hareket":   {"A": ("Y", "apply"), "OE": ("Y", "apply"), "Y": ("Y", "apply"), "D": ("Y", "apply"),
                  "O": ("O", "queue"), "M": ("K", "queue"), "K": ("K", "queue"), "F": ("F", "red"), "B": ("B", "red")},
    "donus":     {"A": ("D", "apply"), "OE": ("D", "apply"), "Y": ("D", "apply"), "D": ("D", "apply"),
                  "O": ("O", "queue"), "M": ("K", "queue"), "K": ("K", "queue"), "F": ("F", "red"), "B": ("B", "red")},
    "dur":       {"A": ("A", "apply"), "OE": ("A", "apply"), "Y": ("A", "apply"), "D": ("A", "apply"),
                  "O": ("O", "apply"), "M": ("M", "apply"), "K": ("K", "apply"), "F": ("B", "apply"),
                  "B": ("B", "noop")},
    "otur":      {"A": ("?", "apply"), "OE": ("OE", "noop"), "Y": ("OE", "apply"), "D": ("OE", "apply"),
                  "O": ("O", "noop"), "M": ("M", "noop"), "K": ("K", "queue"), "F": ("F", "red"), "B": ("B", "red")},
    "kalk":      {"A": ("A", "noop"), "OE": ("A", "apply"), "Y": ("Y", "noop"), "D": ("D", "noop"),
                  "O": ("O", "queue"), "M": ("K", "apply"), "K": ("K", "noop"), "F": ("F", "red"), "B": ("?", "apply")},
    "yukseklik": {"A": ("A", "apply"), "OE": ("OE", "red"), "Y": ("Y", "apply"), "D": ("D", "red"), "O": ("O", "red"),
                  "M": ("M", "red"), "K": ("K", "red"), "F": ("F", "red"), "B": ("B", "red")},
    "bas_jest":  {"A": ("A", "apply"), "OE": ("OE", "red"), "Y": ("Y", "apply"), "D": ("D", "apply"),
                  "O": ("O", "red"), "M": ("M", "red"), "K": ("K", "red"), "F": ("F", "red"), "B": ("B", "red")},
    "vak":       {"A": ("A", "apply"), "OE": ("OE", "apply"), "Y": ("Y", "apply"), "D": ("D", "apply"),
                  "O": ("O", "red"), "M": ("M", "apply"), "K": ("K", "red"), "F": ("F", "red"), "B": ("B", "red")},
}
ROW_WORDS = {r: [w for w, rr in CM.WORD_ROW.items() if rr == r] for r in SPEC}


def kinds(out):
    return [s[0] for s in out["signals"]]


# --- kimlik tablosu ve sabitler ---------------------------------------------------------------------------------
def test_id_table_whitelist_many_to_one():
    """configs/komut_kimlik.json (ornek): beyaz liste, coktan bire (17 -> dur), uyanma 1/2, 200-208 uyari, digerleri yok
    sayilir; bozuk tablo ValueError."""
    t = CM.load_id_table()
    assert t.example and t.missing() == [] and t.wake == {1, 2}
    assert t.ids_of("dur") == [6, 17] and t.classify(17) == ("cmd", "dur") and t.classify(1) == ("wake", None)
    assert t.classify(99) == ("ignore", None) and t.classify(203) == ("warn", None) and t.classify(3) == ("ignore", None)
    for bad in ({"5": "kos"}, {"1": "gel"}):
        with pytest.raises(ValueError):
            CM.IdTable([1, 2], bad)
    with pytest.raises(ValueError):
        CM.IdTable([], {"5": "gel"})


def test_derived_constants():
    """K4/K5 sabitleri config'ten: 0.8 x COMMAND_RANGES, dH = (H_HOME - H_MIN_WALK)/2, T1 sert sinir
    STAND_RAMP_S + ready_hold_s + 0.5, T2 4.0 + ready_hold_s, geri 0.5 m'lik sure en cok 8 s, SUPERVISOR ve GETUP."""
    K, R = CM.K, C.COMMAND_RANGES
    assert K["vx_gel"] == 0.8 * R["vx"][1] and K["vx_geri"] == 0.8 * R["vx"][0] and K["wz_turn"] == 0.8 * R["wz"][1]
    assert K["dh"] == 0.5 * (C.H_HOME - C.H_MIN_WALK) and (K["h_lo"], K["h_hi"]) == (C.H_MIN_WALK, C.H_HOME)
    assert K["t1_hard_s"] == C.STAND_RAMP_S + C.SUPERVISOR["ready_hold_s"] + 0.5
    assert K["t2_limit_s"] == C.MAIN["sitdown_window_s"] + C.SUPERVISOR["ready_hold_s"]
    assert K["geri_s"] == min(0.5 / abs(K["vx_geri"]), 8.0) and K["getup_s"] == C.GETUP_EPISODE_S
    assert K["fall_steps"] == C.SUPERVISOR["fall_steps"] and K["seat_pitch"] == math.radians(C.SEAT_PITCH_DEG)
    if C.ROBOT_PROFILE == "tr35_b2":                     # K4: kappa 0.5'te vx -0.085..0.17 m/s, wz +-0.44 rad/s
        assert abs(R["vx"][0] + 0.085) < 1e-3 and abs(R["vx"][1] - 0.17) < 1e-3 and abs(R["wz"][1] - 0.44) < 2e-3
        assert abs(K["t1_hard_s"] - 3.93) < 0.01 and K["t1_first_s"] == 3.4


def test_table_is_complete_data():
    """TABLE: her olay satiri x her durum, her eylem bir yontem; kelime satirlari spec tablosunun hucre sayisi."""
    assert set(CM.TABLE) == set(CM.ROWS) and all(set(v) == set(CM.STATES) for v in CM.TABLE.values())
    acts = {a for v in CM.TABLE.values() for a in v.values()} | set(CM.HANDOFF.values()) | {CM.HANDOFF_DEFAULT}
    for a in acts:
        assert a == "-" or hasattr(CM.Supervisor, "_act_" + a), a
    assert set(SPEC) | {"dusme", "tamam"} == set(CM.ROWS) and len(CM.WORDS) == 12


# --- K5 tablosu satir satir --------------------------------------------------------------------------------------
@pytest.mark.parametrize("row", list(SPEC))
def test_table_row(row):
    """Her kelime satirinda her durum: kelime islenince yeni durum ve isaret spec tablosundaki gibi; red'de LED red
    kipine gecer, A/OE/Y/D'de cift gaga tiki baslar; kuyruk hucresinde bekleyen = kelime; tablodaki hucre olay listesinde."""
    for state in CM.STATES:
        for word in ROW_WORDS[row]:
            sup, pl = KP.to_state(state)
            if row == "kalk" and state == "B":
                pl.mode = "fallen"                         # dusmusse ->F (oturusta ->K: test_b_kalk_seated)
            out = KP.run(sup, pl, 1, {sup.n: [word]})[0]
            want, sig = SPEC[row][state]
            if want == "?":
                want = {"otur": "O", "kalk": "F"}[row]
            assert out["state"] == want, (row, state, word, out["state"], out["signals"])
            assert sig in kinds(out), (row, state, word, kinds(out))
            assert "%s/%s" % (row, state) in out["events"], (row, state, out["events"])
            if sig == "red":
                assert out["led_mode"] == "red" and out["led"] == 1
                assert (sup.g_word == "red" and sup.g_active) == (state in ("A", "OE", "Y", "D"))
            if sig == "queue":
                assert out["pending"] == word
            if state in ("O", "M", "K") and row == "dur":
                assert out["pending"] is None
            if row == "hareket" and want == "Y":
                v = CM.K["vx_gel"] if word == "gel" else CM.K["vx_geri"]
                assert np.allclose(out["command"], [v, 0, 0, sup.h_cmd, 0], atol=1e-7)
            if row == "donus" and want == "D":
                assert out["command"][2] == np.float32(CM.K["wz_turn"] * (1 if word == "sola" else -1))
            if row == "yukseklik" and sig == "apply":
                h = np.clip(C.H_HOME + (-1 if word == "egil" else 1) * CM.K["dh"], C.H_MIN_WALK, C.H_HOME)
                assert abs(float(out["command"][3]) - h) < 1e-6
            if row in ("bas_jest", "vak") and sig == "apply":
                assert sup.g_active and sup.g_word == word
                if state == "M":                             # gaga oturusta <= 15 derece
                    assert abs(float(sup.gst["amp"][0]) - math.radians(15.0) / C.GESTURES["beak"]["amp"]) < 1e-6


def test_otur_settle_rule_and_b_kalk_seated():
    """A'da otur: son hizdan 1.0 s gectiyse ->O, yoksa ->OE (OE 1.0 s dolunca ->O); B'de oturusta kalk ->K."""
    sup, pl = KP.to_state("Y")
    KP.run(sup, pl, 1, {sup.n: ["dur"]})
    out = KP.run(sup, pl, 10, {sup.n: ["otur"]})[0]
    assert out["state"] == "OE"
    outs = KP.run(sup, pl, CM.steps(CM.SETTLE_S))
    i = next(k for k, o in enumerate(outs) if o["state"] == "O")
    assert "tamam/OE" in outs[i]["events"] and i == CM.steps(CM.SETTLE_S) - 11      # dur'dan 1.0 s sonra
    sup, pl = KP.to_state("A")
    assert KP.run(sup, pl, 1, {sup.n: ["otur"]})[0]["state"] == "O"
    sup, pl = KP.to_state("O", sit_fail=True)
    outs = KP.run(sup, pl, 400)
    assert outs[-1]["state"] == "B"
    out = KP.run(sup, pl, 1, {sup.n: ["kalk"]})[0]
    assert out["state"] == "F"                                # oturustan 25 derece: oturusla uyumlu degil -> getup
    sup, pl = KP.to_state("K", stand_fail=2)
    KP.run(sup, pl, 700)
    assert sup.state == "B"
    pl.g_last = KP.g_of(math.radians(C.SEAT_PITCH_DEG))      # BEKLE'de oturus pozunda duruyor
    out = KP.run(sup, pl, 1, {sup.n: ["kalk"]})[0]
    assert out["state"] == "K" and out["main_alpha"] == 1.0 and out["policy"] == "main"


# --- ic olaylar: dusme ve tamamlanma -------------------------------------------------------------------------------
def test_fall_standard_and_posture_relative():
    """A, OE, Y, D: gz_obs > fall_gz art arda fall_steps adim (kesinti sayaci sifirlar). O, M, K: egim hem
    pitch_ref'ten hem oturus pozundan > 35 derece; oturusla uyumlu (<= 15 derece, gz < seat_gz) ya da yalniz birinden
    sapan durus dusme degil. Dusme: kuyruk temizlenir, jest kesilir, getup."""
    for state in ("A", "OE", "Y", "D"):
        sup, pl = KP.to_state(state)
        pl.fall_at = pl.k
        outs = KP.run(sup, pl, FS)
        assert [o["state"] for o in outs][:FS - 1] == [state] * (FS - 1) and outs[FS - 1]["state"] == "F", state
        assert "dusme/%s" % state in outs[FS - 1]["events"] and outs[FS - 1]["policy"] == "getup"
        assert outs[FS - 1]["getup_restart"] and outs[FS - 1]["command"] is None
    sup, pl = KP.to_state("A")
    pl.fall_at = pl.k
    KP.run(sup, pl, FS - 1)
    pl.mode = "ok"
    assert KP.run(sup, pl, 1)[0]["state"] == "A" and sup.fall_n == 0
    seat = math.radians(C.SEAT_PITCH_DEG)
    for state in ("M", "K", "O"):
        sup, _ = KP.to_state(state)
        for p, fell in ((seat + math.radians(10), False), (seat + math.radians(-30), False), (math.radians(60), True),
                        (seat - math.radians(40), True)):
            s2 = copy.deepcopy(sup)
            s2.state = state
            for _ in range(FS):
                o = s2.step(CM.Inputs(0, [], KP.g_of(p), np.zeros(3), np.zeros(14), np.zeros(14), 1.0, C.H_HOME))
                if o["state"] != state:
                    break
            assert (o["state"] == "F") == fell, (state, math.degrees(p), o["state"])
    assert CM.seat_compatible(KP.g_of(seat + math.radians(10))) and not CM.seat_compatible(KP.g_of(seat + math.radians(20)))


def test_completions_motion_turn_height_gesture():
    """Y: gel 12 s, geri 0.5 m'lik sure (en cok 8 s) -> A; D: bias duzeltilmis gyro integrali 90 derece (ya da 8 s)
    -> A; eğil / yüksel |h_ref - h_cmd| < 1e-6; jest bitince 'gesture' tamamlanmasi; hepsinde 'tamam/<durum>'."""
    for word, n in (("gel", CM.steps(12.0)), ("geri", CM.steps(CM.K["geri_s"]))):
        sup, pl = KP.to_state("A")
        outs = KP.run(sup, pl, n + 2, {sup.n: [word]})
        i = next(k for k, o in enumerate(outs) if o["state"] == "A" and k > 0)
        assert i == n and "tamam/Y" in outs[i]["events"] and ("motion", None) in outs[i]["done"], (word, i, n)
    sup, pl = KP.to_state("A")
    outs = KP.run(sup, pl, 500, {sup.n: ["sola"]})
    i = next(k for k, o in enumerate(outs) if o["state"] == "A" and k > 0)
    exp = math.radians(90.0) / CM.K["wz_turn"] / C.CTRL_DT
    assert abs(i - exp) <= 2 and ("turn", None) in outs[i]["done"] and "tamam/D" in outs[i]["events"]
    sup, pl = KP.to_state("A")
    pl.cmd[2] = 0.0
    sup.bias[:] = 0.0
    outs = [sup.step(CM.Inputs(0, [KP.wid("saga")] if k == 0 else [], [0, 0, -1], [0, 0, 0], np.zeros(14),
                               np.zeros(14), 0.0, C.H_HOME)) for k in range(CM.steps(8.0) + 2)]
    assert outs[CM.steps(8.0)]["state"] == "A" and outs[CM.steps(8.0) - 1]["state"] == "D"      # donmeyen robot: 8 s
    sup, pl = KP.to_state("A")
    outs = KP.run(sup, pl, 120, {sup.n: ["egil"]})
    i = next(k for k, o in enumerate(outs) if ("height", None) in o["done"])
    assert abs(i - CM.K["dh"] / C.H_RATE / C.CTRL_DT) <= 1 and "tamam/A" in outs[i]["events"]
    sup, pl = KP.to_state("A")
    outs = KP.run(sup, pl, 120, {sup.n: ["evet"]})
    i = next(k for k, o in enumerate(outs) if ("gesture", "evet") in o["done"])
    assert i == int(np.floor(T.GEST_TAB["dur"][1] / C.CTRL_DT + 1e-6)) + 1 and "tamam/A" in outs[i]["events"]


def test_transitions_t1_t2_queue_and_failed_rules():
    """T1 (alpha 0, gz < stand_gz, |gyro| < esik, ready_hold_s; ilk saglanma <= 3.4 s, sert sinir STAND_RAMP_S +
    ready_hold_s + 0.5) ->A ve bekleyen uygulanir; T2 ->M, bekleyen varsa kalk + bekleyen ->K. Basarisiz kalkis: oturusa
    don (O), T2, bir kez daha kalk; ikinci basarisizlikta red ->B (kuyruk temizlenir). Basarisiz oturus: red ->B. Gecis
    hatalari getup baslatmaz."""
    sup, pl = KP.to_state("M")
    outs = KP.run(sup, pl, 400, {sup.n: ["kalk"], sup.n + 20: ["gel"]})
    i = next(k for k, o in enumerate(outs) if o["state"] != "K" and k > 0)
    t1 = math.ceil(C.STAND_RAMP_S / C.CTRL_DT - 1e-6) + CM.steps(C.SUPERVISOR["ready_hold_s"]) - 1
    assert outs[i]["state"] == "Y" and ("T1", None) in outs[i]["done"] and abs(i - t1) <= 1, (i, t1)
    assert "hareket/A" in outs[i]["events"] and outs[i]["pending"] is None
    sup, pl = KP.to_state("A")
    outs = KP.run(sup, pl, 400, {sup.n: ["otur"], sup.n + 50: ["saga"]})
    i = next(k for k, o in enumerate(outs) if ("T2", None) in o["done"])
    assert outs[i]["state"] == "K" and outs[i]["pending"] == "saga" and "tamam/O" in outs[i]["events"]
    sup, pl = KP.to_state("A")
    outs = KP.run(sup, pl, 400, {sup.n: ["otur"]})
    i = next(k for k, o in enumerate(outs) if ("T2", None) in o["done"])
    assert outs[i]["state"] == "M" and outs[i + 1]["state"] == "M" and outs[i]["command"][4] == 1.0
    # basarisiz kalkis kurali
    sup, pl = KP.to_state("K", stand_fail=2)
    pend = KP.run(sup, pl, 1, {sup.n: ["gel"]})[0]["pending"]
    outs = KP.run(sup, pl, 700)
    seq = [o["state"] for o in outs]
    comp = [seq[0]] + [b for a, b in zip(seq, seq[1:]) if a != b]
    assert pend == "gel" and comp == ["K", "O", "K", "B"], comp            # O -> (T2: M) -> K ayni adimda
    k_b = seq.index("B")
    assert "red" in kinds(outs[k_b]) and outs[k_b]["pending"] is None and "F" not in seq
    assert all(o["pending"] == "gel" for o in outs[:k_b])                # bekleyen kurtarma boyunca korunur
    # basarisiz oturus
    sup, pl = KP.to_state("O", sit_fail=True)
    outs = KP.run(sup, pl, 300)
    k_b = [o["state"] for o in outs].index("B")
    assert "red" in kinds(outs[k_b]) and abs((k_b + 3) * C.CTRL_DT - CM.K["t2_limit_s"]) < 0.1


def test_getup_success_handover_and_failure():
    """F: getup basarisi (olculen gz, poz RMS, gyro; ready_hold_s) ->A devir: h_cmd = H_HOME, sit 0, handoff_zero_cmd_s
    sifir komut, bu surede kelimeler kuyruga (jestler red), sure bitince uygulanir. 12 s'de basarisiz: 2. deneme
    (getup_restart), sonra red ->B; F'de dur getup'i keser ->B; B'de dur etkisiz, ramp 0.5 s sonra PWM kapali."""
    sup, pl = KP.to_state("F", getup_after=5)
    outs = KP.run(sup, pl, 120, {sup.n + 40: ["gel"], sup.n + 42: ["evet"], sup.n + 44: ["egil"]})
    i = next(k for k, o in enumerate(outs) if o["handover"])
    assert outs[i]["state"] == "A" and outs[i]["policy"] == "main" and outs[i]["main_alpha"] == 0.0
    hz = CM.steps(C.SUPERVISOR["handoff_zero_cmd_s"])
    assert 0 < i < 40
    for o in outs[i:i + hz]:
        assert np.allclose(o["command"], [0, 0, 0, C.H_HOME, 0])
    q = [o["pending"] for o in outs[i:i + hz]]
    assert "egil" in q and any("red" in kinds(o) for o in outs[i:i + hz])
    j = i + hz
    assert outs[j]["pending"] is None and abs(float(outs[j]["command"][3]) - (C.H_HOME - CM.K["dh"])) < 1e-6
    sup, pl = KP.to_state("F", getup_after=None)
    ng = CM.steps(CM.K["getup_s"])                                       # tr35 12 s (GETUP_EPISODE_S)
    outs = KP.run(sup, pl, 2 * ng + 150, {sup.n + 2 * ng + 10: ["dur"]})
    rs = [k for k, o in enumerate(outs) if o["getup_restart"]]
    assert rs == [ng - 2] and outs[-1]["state"] == "B"                   # F'de zaten 2 adim (to_state)
    kb = [o["state"] for o in outs].index("B")
    assert kb == rs[0] + ng and "red" in kinds(outs[kb])
    assert outs[kb]["policy"] == "none" and outs[kb]["command"] is None and outs[kb]["led_mode"] == "red"
    nb = CM.steps(CM.B_RAMP_S)
    assert not outs[kb + nb - 1]["pwm_off"] and outs[kb + nb]["pwm_off"] and outs[kb + nb]["ramp"] == 1.0
    assert outs[kb + 1]["ramp"] == np.float32(1.0 / nb)
    k_dur = 2 * ng + 10
    assert outs[k_dur]["state"] == "B" and "noop" in kinds(outs[k_dur])
    assert {o["led_mode"] for o in outs[kb + 50:]} == {"bekle"} and {0, 1} <= {o["led"] for o in outs[kb + 50:]}
    sup, pl = KP.to_state("F", getup_after=None)
    out = KP.run(sup, pl, 1, {sup.n: ["dur"]})[0]
    assert out["state"] == "B" and "bekle" in kinds(out)


def test_ids_led_and_unknown_ineffective():
    """Uyanma kimligi LED'i 15 s yakar (yenilenmez degil: yeni uyanma yeniden baslatir); tabloda olmayan kimlik ve
    200-208 etkisiz (ciktilar ayni, yalniz ignore / warn isareti); red LED 3 kez yanip soner."""
    sup, pl = KP.to_state("A")
    outs = KP.run(sup, pl, CM.steps(15.0) + 3, {sup.n: ["wake"]})
    assert outs[0]["led"] == 1 and outs[CM.steps(15.0)]["led"] == 1 and outs[CM.steps(15.0) + 1]["led"] == 0
    for i in (99, 3, 205, 21):
        sup, pl = KP.to_state("Y")
        a, b = copy.deepcopy(sup), copy.deepcopy(sup)
        pa, pb = copy.deepcopy(pl), copy.deepcopy(pl)
        oa, ob = KP.run(a, pa, 1, {a.n: [i]})[0], KP.run(b, pb, 1)[0]
        assert kinds(oa) == (["warn"] if 200 <= i <= 208 else ["ignore"]) and kinds(ob) == []
        for k in ("state", "pending", "led", "events"):
            assert oa[k] == ob[k], (i, k)
        assert np.array_equal(oa["command"], ob["command"]) and np.array_equal(oa["gesture"], ob["gesture"])
    sup, pl = KP.to_state("OE")
    outs = KP.run(sup, pl, 40, {sup.n: ["evet"]})
    led = [o["led"] for o in outs[:2 * 3 * CM.steps(CM.LED_RED_HALF_S)]]
    assert sum(1 for a, b in zip([0] + led, led) if b and not a) == 3 and outs[0]["led_mode"] == "red"


# --- degismezler (rastgele diziler) --------------------------------------------------------------------------------
def _check_invariants(sup, out, inp):
    st, cmd = out["state"], out["command"]
    bad = []
    if st in ("F", "B") and cmd is not None:
        bad.append("F/B main komutu")
    if st in ("OE", "M", "O", "K") and cmd is not None and np.any(cmd[:3] != 0):
        bad.append("hiz komutu " + st)
    if cmd is not None and not (C.H_MIN_WALK - 1e-6 <= float(cmd[3]) <= C.H_HOME + 1e-6):
        bad.append("h araligi")
    if any(sup.tab.classify(i) == ("cmd", "dur") for i in inp.ids) and not any(e.startswith("dur/") for e in out["events"]):
        bad.append("dur dusuruldu")
    if out["pending"] is not None and not (st in ("O", "K") or (st == "A" and sup.handoff_n > 0)):
        bad.append("bekleyen %s / %s" % (out["pending"], st))
    return bad


def test_invariants_random_sequences():
    """200 rastgele dizi (kelimeler, tablo disi kimlikler, dusmeler, basarisiz kalkis / oturus, basarisiz getup): F ve
    B'de main komutu yok; OE, M, O, K'de hiz komutu yok; h [H_MIN_WALK, H_HOME] icinde; dur hicbir durumda dusmez; tablo
    disi kimlik etkisiz; bekleyen yalniz O / K'de (ya da devrin sifir komut suresinde A'da) ve tamamlanan gecis icinde
    uygulanir ya da temizlenir (gecis bitince bekleyen kalmaz). Butun hucreler en az bir kez tetiklenir (erisilemezler
    haric)."""
    rng = np.random.default_rng(5)
    ids = [KP.wid(w) for w in CM.WORDS] + [17, 99, 204, 1]
    cells, nviol = {}, []
    for ep in range(120):
        pl = KP.Plant(seated=bool(ep % 2), fall_at=int(rng.integers(100, 2500)) if rng.random() < 0.5 else None,
                      stand_fail=int(rng.integers(0, 3)), sit_fail=rng.random() < 0.2,
                      getup_after=None if rng.random() < 0.3 else int(rng.integers(5, 400)))
        sup = CM.Supervisor(KP.table(), KP.home_q(), "M" if ep % 2 else "A")
        for k in range(2500):
            evs = [int(rng.choice(ids))] if rng.random() < 0.02 else []
            if pl.fall_at is None and pl.mode == "ok" and rng.random() < 3e-4:
                pl.fall_at = pl.k
            inp = pl.inputs(k * C.CTRL_DT, evs)
            if evs and sup.tab.classify(evs[0])[0] in ("ignore", "warn"):
                ref = copy.deepcopy(sup).step(CM.Inputs(inp.t, [], inp.gravity, inp.gyro, inp.qd, inp.q, inp.alpha, inp.h_ref))
            else:
                ref = None
            out = sup.step(inp)
            pl.apply(out)
            if ref is not None:
                for key in ("state", "pending", "led", "events", "policy"):
                    if ref[key] != out[key]:
                        nviol.append((ep, k, "kimlik %s etkili (%s)" % (evs, key)))
            nviol += [(ep, k, b) for b in _check_invariants(sup, out, inp)]
            for c in out["events"]:
                cells[c] = cells.get(c, 0) + 1
    assert not nviol, nviol[:10]
    missing = [(r, s) for r in CM.ROWS for s in CM.STATES
               if cells.get("%s/%s" % (r, s), 0) == 0 and (r, s) not in CM.UNREACHABLE]
    assert not missing, missing


# --- jest fonksiyonu (task_math; env_cpu, env_mjx, commands ayni fonksiyon) ---------------------------------------
def _gesture_run(kind, alpha=0.0, cut_at=None, n=150):
    st = T.gesture_init(np, np.zeros(1))
    outs, act = [], []
    for k in range(n):
        st, o, a, _ = T.gesture_step(np, st, np.array([kind if k == 0 else 0.0]), np.array([alpha]),
                                     np.array([1.0 if k == cut_at else 0.0]))
        outs.append(o[0])
        act.append(a[0])
    return np.array(outs), np.array(act)


def test_gesture_envelope_fade_gate_slew():
    """Tukey zarfi: t = 0 ve sonda konum/hiz 0, ortadaki tam cevrimde tam genlik (tepeden tepeye 2A; gaga 0 <-> A); tik
    0 -> 12 derece -> 0, 0.3 s; kesme 0.25 s yukseltilmis kosinusle sonumlenir (basamak yok, sonra 0); kapi: bas
    jestleri alpha < ALPHA_STAND_MAX, gaga alpha = 1'de de (en cok 15 derece), kanal doluyken yeni jest baslamaz;
    hiz sinirlari (sayisal, sonumleme dahil) >= gercek hiz; check_head_slew tr35 servo sinirlarinda gecer."""
    dt = C.CTRL_DT
    for kind, name in ((T.GEST_NOD, "nod"), (T.GEST_SHAKE, "shake"), (T.GEST_BEAK, "beak")):
        g = C.GESTURES[name]
        ch = T.GEST_CHANNELS.index(g["joint"])
        o, a = _gesture_run(kind)
        x = o[:, ch]
        D, tt = g["cycles"] / g["freq"], 0.5 / g["freq"]
        t = np.arange(len(x)) * dt
        assert x[0] == 0.0 and abs(x[1]) < 0.01 * g["amp"] and np.all(o[:, [c for c in range(5) if c != ch]] == 0)
        mid = (t >= tt - 1e-9) & (t <= D - tt + 1e-9)
        p2p = x[mid].max() - x[mid].min()
        assert abs(p2p - (2.0 if name != "beak" else 1.0) * g["amp"]) < 0.02 * g["amp"], (name, p2p)
        n_on = int(a.sum())
        assert abs(n_on * dt - D) <= 2 * dt and np.all(x[n_on:] == 0) and abs(x[n_on - 1]) < 0.02 * g["amp"]
        rate = np.abs(np.diff(x)) / dt
        assert rate.max() <= T.gesture_rate_bounds()[ch] + 1e-6
    o, a = _gesture_run(T.GEST_CLICK)
    m = o[:, 4]
    assert math.radians(11.7) < m.max() <= math.radians(12.0) and m[0] == 0 and int(a.sum()) == 16 and m[15] < 1e-9
    tf = np.linspace(0.0, 0.3, 3001)
    pk = T.gesture_raw(np, np.full(len(tf), T.GEST_CLICK), tf, np.ones(len(tf)))[:, 4]
    assert abs(pk.max() - math.radians(12.0)) < 1e-9 and pk[0] == 0 and abs(pk[-1]) < 1e-12
    # kesme: 0.25 s sonumleme, sonra 0
    o, a = _gesture_run(T.GEST_SHAKE, cut_at=30)
    x = o[:, 2]
    nf = int(math.floor(C.GESTURE_FADE_S / dt + 1e-9))
    assert np.all(x[30 + nf + 1:] == 0.0) and int(a.sum()) == 30 + nf + 1
    assert np.max(np.abs(np.diff(x[25:40]))) / dt <= T.gesture_rate_bounds()[2] + 1e-6
    w = T.gesture_fade(np, np.arange(0, nf + 1, dtype=float))
    assert w[0] == 1.0 and np.all(np.diff(w) < 0) and w[-1] < 0.01
    # kapi
    for alpha in (0.02, 0.5, 1.0):
        o, a = _gesture_run(T.GEST_NOD, alpha=alpha)
        assert a.sum() == 0 and np.all(o == 0)
    o, a = _gesture_run(T.GEST_BEAK, alpha=1.0)
    assert math.radians(14.8) < o[:, 4].max() <= math.radians(15.0) + 1e-9 and a.sum() > 0
    assert _gesture_run(T.GEST_BEAK, alpha=0.5)[1].sum() == 0
    st = T.gesture_init(np, np.zeros(1))
    st, _, _, go = T.gesture_step(np, st, np.array([1.0]), np.zeros(1), np.zeros(1))
    st, _, _, go2 = T.gesture_step(np, st, np.array([2.0]), np.zeros(1), np.zeros(1))
    assert go[0] == 1 and go2[0] == 0 and st["kind"][0] == 1.0
    # numpy float32 korunur (commands.py float32)
    st32 = T.gesture_init(np, np.zeros(1, np.float32))
    st32, o32, _, _ = T.gesture_step(np, st32, np.array([2.0], np.float32), np.zeros(1, np.float32), np.zeros(1, np.float32))
    assert o32.dtype == np.float32 and all(v.dtype == np.float32 for v in st32.values())
    # sinirlar
    b, lim = T.check_head_slew(False)
    if C.ROBOT_PROFILE == "tr35_b2":
        b2, _ = T.check_head_slew(True)
        assert np.all(b2 >= b) and np.all(b2 <= lim)


def test_gesture_sample_probabilities():
    """Egitim orneklemesi (mevcut bas uniform'lari): 0.3 jest (4 tur esit), 0.35 rastgele, 0.35 sifir."""
    u = np.random.default_rng(0).random((200000, 2))
    req, rnd = T.gesture_sample(np, u[:, 0], u[:, 1])
    assert abs(np.mean(req > 0) - 0.3) < 0.005 and abs(np.mean(rnd > 0) - 0.35) < 0.005
    assert set(np.unique(req)) == {0.0} | set(T.GEST_TRAIN)
    for k in T.GEST_TRAIN:
        assert abs(np.mean(req == k) - 0.075) < 0.003


# --- gyro bias: gercek sensor modeli ---------------------------------------------------------------------------------
def test_bias_update_real_sensor_model():
    """sensors.imu_tick (200 Hz; montaj, olcek, beyaz gurultu) ile duran robot, gyro bias b = 0.04 rad/s (uc eksen): 2 s
    icinde guncellenir ve z ekseni hatasi < 0.005 rad/s (200 tohumun >= %97'si; ortanca << 0.005); hareket ya da bacak
    qd'si guncellemeyi durdurur; |b| <= 0.06 kirpilir; donus integrali bias'tan arindirilir."""
    P = SENS.pot_table(list(C.OBS_JOINTS), ["ds32xx"] * 14, ["leg"] * 14, [7.4] * 14)
    errs, first = [], []
    for seed in range(200):
        rng = np.random.default_rng(seed)
        ep = SENS.episode(np, rng.random(SENS.n_episode(14)), P, True)
        ep["gb"] = np.full(3, 0.04)
        imu = SENS.imu_reset(np, np.array([1.0, 0, 0, 0]), np.zeros(3), ep)
        sup = CM.Supervisor(KP.table(), KP.home_q(), "A")
        upd = None
        for k in range(CM.steps(2.0)):
            for _ in range(C.N_SUBSTEPS):
                imu = SENS.imu_tick(np, imu, np.zeros(3), np.array([0.0, 0.0, 9.81]), 0.0, rng.random(6), ep, True)
            sup.step(CM.Inputs(k * C.CTRL_DT, [], imu["g"], imu["w"], np.zeros(14), np.zeros(14), 0.0, C.H_HOME))
            if upd is None and sup.bias_updates:
                upd = k
        first.append(upd)
        errs.append(np.abs(sup.bias - 0.04))
    errs = np.array(errs)
    assert all(u is not None and u <= CM.steps(CM.BIAS_WIN_S) for u in first)
    assert np.mean(errs[:, 2] < 0.005) >= 0.97 and np.median(errs) < 0.002, (np.mean(errs[:, 2] < 0.005), np.median(errs))
    # hareket / bacak hizi / kirpma
    pl = KP.Plant(bias=(0, 0, 0.04), qd=0.3)
    sup = CM.Supervisor(KP.table(), KP.home_q(), "A")
    KP.run(sup, pl, 200)
    assert sup.bias_updates == 0
    pl = KP.Plant(bias=(0.1, -0.1, 0.0))
    sup = CM.Supervisor(KP.table(), KP.home_q(), "A")
    KP.run(sup, pl, 100)
    assert np.allclose(sup.bias, [0.06, -0.06, 0.0], atol=1e-7)
    pl = KP.Plant(bias=(0, 0, 0.04))
    sup = CM.Supervisor(KP.table(), KP.home_q(), "A")
    outs = KP.run(sup, pl, 600, {150: ["sola"]})
    i = next(k for k, o in enumerate(outs) if k > 150 and o["state"] == "A")
    assert abs((i - 150) - math.radians(90) / CM.K["wz_turn"] / C.CTRL_DT) <= 2     # bias'siz plan suresi
    sup = CM.Supervisor(KP.table(), KP.home_q(), "A")
    pl = KP.Plant(bias=(0, 0, 0.04))
    outs = KP.run(sup, pl, 400, {0: ["sola"]})                   # bias guncellenmeden: integral 0.04 rad/s hizli
    i = next(k for k, o in enumerate(outs) if k > 0 and o["state"] == "A")
    w = CM.K["wz_turn"]
    assert i < math.radians(90) / w / C.CTRL_DT * (1.0 - 0.5 * 0.04 / w)


# --- play.Supervisor = commands.FallMonitor ------------------------------------------------------------------------
def test_play_supervisor_uses_commands():
    """play.Supervisor mantigi commands.FallMonitor'da (play ve eval cagirir); olculen alanlar varsa onlari kullanir."""
    from rl import play
    assert issubclass(play.Supervisor, CM.FallMonitor) and play.Supervisor.update is CM.FallMonitor.update
    s = {"quat": np.array([[1.0, 0, 0, 0]]), "gyro": np.zeros((1, 3)), "q": np.zeros((1, 14)), "default_q": np.zeros(14),
         "trunk_z": np.array([0.05])}
    m = dict(s, gravity_obs=np.array([[0.0, 1.0, 0.0]]), gyro_obs=np.zeros((1, 3)), q_obs=np.zeros((1, 14)),
             qd_obs=np.zeros((1, 14)))
    a, b = CM.FallMonitor(), CM.FallMonitor()
    for _ in range(FS):
        ra, rb = a.update(s), b.update(m)
    assert ra is None and rb == "getup"


# --- altin izler (tr35 alt sureci) -----------------------------------------------------------------------------------
_REPLAY = r"""
import json, sys, os, hashlib
sys.path.insert(0, %(root)r)
import numpy as np
from rl import commands as CM, config as C
sys.path.insert(0, os.path.join(%(root)r, "tools"))
import commands_golden as G
meta = json.load(open(os.path.join(%(data)r, "komut_golden.json")))
d = meta["id_table"]
tab = CM.IdTable(d["uyanma"], d["komutlar"], d.get("ogrenme_kimlikleri", CM.LEARNING_IDS), d.get("ornek", False))
out = {"profile": C.ROBOT_PROFILE, "derived_ok": hashlib.sha256(open(C.DERIVED_PATH, "rb").read()).hexdigest() == meta["derived_sha256"],
       "traces": {}}
for name, tr in sorted(meta["traces"].items()):
    rows = G.read_trace(os.path.join(%(data)r, tr["file"]))
    sup = CM.Supervisor(tab, np.asarray(tr["home_q"], np.float32), tr["start"])
    bad, states, cmax = [], set(), 0.0
    for r in rows:
        inp = CM.inputs_from_row(r)
        o = CM.trace_row(inp, sup.step(inp))
        diff = CM.compare_rows(r, o, meta["atol"])
        if diff and len(bad) < 5:
            bad.append((r["t"], diff, [(k, r[k], o[k]) for k in diff][:3]))
        states.add(o["state"])
        cmax = max(cmax, max(abs(float(r[k]) - float(o[k])) for k in CM.TRACE_OUT_C))
    out["traces"][name] = {"rows": len(rows), "want": tr["rows"], "bad": bad, "states": sorted(states), "cmax": cmax}
print(json.dumps(out))
"""


def _tr35_env(**kw):
    env = dict(os.environ, OPENDUCK_ROBOT="tr35_b2", PYTHONDONTWRITEBYTECODE="1", MUJOCO_GL="disabled")
    env.pop("OPENDUCK_SCENE", None)
    env.update(kw)
    return env


def test_golden_traces_replay():
    """rl/tests/data/komut_*.csv.gz (tools/commands_golden.py; sentetik + A10 S1/S2 rollout'lari) tr35 surecinde yeniden
    oynatilir: her adimda ayrik ciktilar birebir, surekli ciktilar 1e-6 icinde. Turetilmis dosya izlerin uretildigi
    dosyayla ayni olmali (degistiyse izler yeniden uretilir)."""
    with open(os.path.join(DATA, "komut_golden.json")) as f:
        meta = json.load(f)
    assert meta["profile"] == "tr35_b2" and len(meta["traces"]) >= 10
    assert {"rollout_S1", "rollout_S2"} <= set(meta["traces"]) and set(KP.SCENARIOS) <= set(meta["traces"])
    r = subprocess.run([sys.executable, "-c", _REPLAY % {"root": ROOT, "data": DATA}], cwd=ROOT, env=_tr35_env(),
                       capture_output=True, text=True, timeout=1200)
    assert r.returncode == 0, r.stderr[-4000:]
    o = json.loads(r.stdout.strip().splitlines()[-1])
    assert o["profile"] == "tr35_b2"
    assert o["derived_ok"], "rl/derived/tr35_b2.json degisti: python tools/commands_golden.py ile izleri yeniden uretin"
    seen = set()
    for name, t in o["traces"].items():
        assert t["rows"] == t["want"] and not t["bad"], (name, t["bad"])
        assert t["cmax"] <= 1e-6, (name, t["cmax"])
        seen |= set(t["states"])
    assert seen == set(CM.STATES), seen


def test_tr35_process():
    """Bu dosyanin testleri tr35_b2 surecinde (tr35 sabitleriyle; sahne gerekmez)."""
    r = subprocess.run([sys.executable, "-m", "pytest", os.path.abspath(__file__), "-q", "-p", "no:cacheprovider",
                        "-k", "not tr35_process and not golden"], cwd=ROOT, env=_tr35_env(), capture_output=True,
                       text=True, timeout=1800)
    assert r.returncode == 0, r.stdout[-6000:] + r.stderr[-3000:]
    assert " passed" in r.stdout and " failed" not in r.stdout
