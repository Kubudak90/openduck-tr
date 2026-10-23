# -*- coding: utf-8 -*-
"""Spec 7 karar tablosu hukumleri ve kisa Turkce rapor (Markdown)."""
import math


def _g(d, *keys):
    for k in keys:
        if not isinstance(d, dict):
            return None
        d = d.get(k)
    return d


def _v(item, measured, threshold, ok, action_ok, action_fail, note=None):
    if ok is None:
        res, act = "veri yok", "Testi kosun"
    else:
        res, act = ("gecti" if ok else "kaldi"), (action_ok if ok else action_fail)
    return dict(item=item, measured=measured, threshold=threshold, ok=ok, result=res, action=act, note=note)


def verdicts(r):
    cls = _g(r, "servo", "cls")
    out = []
    g = r.get("gate") or {}
    u = g.get("u") or {}
    kap = g.get("kappa")
    out.append(_v("u(tau_s, omega0, kutle) <= 0.85", ", ".join("k%s: %.3f" % (k, v) for k, v in u.items()) or None,
                  "<= 0.85", (kap is not None) if u else None,
                  "kappa = %s (GAIT_FREQ %.2f Hz)" % (kap, 2.21 * kap) if kap else "", "Dur, spec'e don",
                  note="kutle %.2f kg" % g["mass_kg"] if g else None))
    sw = _g(r, "pot", "sweep") or {}
    pc = _g(r, "pot", "potcal") or {}
    present = sw.get("present")
    ok_pot = None if present is None else bool(present and sw.get("monotonic") and (sw.get("span_v") or 0) >= 1.0
                                               and (pc.get("monotonic", True)))
    out.append(_v("Pot (var, monoton, >= 1 V)", "var=%s monoton=%s aralik=%.2f V" % (present, sw.get("monotonic"),
                  sw.get("span_v") or 0) if sw else None, "monoton, >= 1 V", ok_pot, "Devam", "Dur, spec'e don",
                  note=("besleme %.2f V (%s)" % (sw["pot_supply_v"], sw.get("pot_supply"))) if sw.get("pot_supply_v")
                  else None))
    cm = _g(r, "pot", "common_mode")
    out.append(_v("Pot ortak-mod hata", "%.2f derece @ %.2f A (%.2f d/A)" % (cm["err_at_ref_deg"], cm["i_ref"],
                  cm["slope_deg_per_a"]) if cm else None, "<= 1 derece", (cm["err_at_ref_deg"] <= 1.0) if cm else None,
                  "Tek uclu okuma yeterli", "Oranli okuma (bos ADS kanali referans)"))
    a300 = _g(r, "pwm", "accept_300hz")
    if cls == "ds":
        out.append(_v("DS 300 Hz", a300, "kabul", a300, "Bacak karti 300 Hz (prescale 19/20)",
                      "Bacak karti 50 Hz, gecikme U{0..4}, cozunurluk 4.9 us; yeniden egit"))
    elif cls == "es":
        out.append(_v("ES3054 300 Hz", a300, "kabul", a300, "hip_yaw 0x40'a tasinir, gecikme U{0..2}",
                      "hip_yaw 0x41'de 50 Hz kalir"))
    a33 = _g(r, "pwm", "accept_3v3")
    out.append(_v("3.3 V darbe", a33, "kabul", a33, "Seviye cevirici gerekmez", "Seviye cevirici (BOM + CAD)"))
    ov = r.get("rail")
    if cls == "ds" or ov:
        out.append(_v("Ray asimi (yavaslamada)", "%.2f V" % ov["v_max"] if ov else None, "<= 6.8 V",
                      (ov["v_max"] <= 6.8) if ov else None, "Kapasitor yeterli",
                      "Kirpici ya da fren direnci; kapasitor artir"))
    sc = r.get("supply_current") or {}
    d2 = sc.get("stall_two_method_diff")
    out.append(_v("Stall akimi (iki yontem)", "INA %.2f A / olcu aleti %.2f A (%.1f%%)" % (
        sc["stall_a_ina"], sc["stall_a_meter"], 100 * d2) if d2 is not None else None, "fark <= %10",
        sc.get("stall_two_method_pass"), "Akim modeli gecerli", "Sont kalibrasyonunu kontrol et, tekrar olc"))
    th = r.get("thermal") or {}
    tk = th.get("t_knee")
    out.append(_v("Isinma: T_diz", ("%.3f N.m RMS%s" % (tk, " (ekstrapolasyon)" if th.get("extrapolated") else ""))
                  if tk else None, "kasa <= 60 C @ 25 C, 10 dk", (tk is not None) if th else None,
                  "A4 ve G-ISIL bu degerle", "Isinma testini tekrarlayin (seviye/kutle)"))
    kn = r.get("knee") or {}
    out.append(_v("Diz uygunlugu", ("T_diz %.3f vs tahmini diz RMS %.3f N.m" % (tk, kn["knee_rms_pred"])) if tk else None,
                  "T_diz >= diz RMS", kn.get("eligible"), "Diz adayi", "Dize uygun degil (diger DS eklemleri)",
                  note=kn.get("knee_rms_src")))
    return out


def _fmt(v, f="%.3f"):
    return "-" if v is None else (f % v if isinstance(v, (int, float)) else str(v))


def servo_section(r):
    a = r.get("actuator") or {}
    s = r.get("servo") or {}
    sc = r.get("supply_current") or {}
    pot = r.get("pot") or {}
    L = ["## %s (%s, sinif %s)" % (s.get("id"), s.get("model") or "?", s.get("mjcf_class"))]
    if r.get("error"):
        L.append("**HATA:** " + r["error"])
        return "\n".join(L)
    L.append("")
    L.append("| Parametre | Deger |")
    L.append("|---|---|")
    rows = [("tau_s (N.m)", a.get("tau_s")), ("omega0 = tau_s/b (rad/s)", a.get("omega0")),
            ("bosta plato hizi (rad/s)", a.get("omega_noload")), ("60 derece suresi (s)", a.get("t60_s")),
            ("P-bandi (derece)", a.get("p_band_deg")), ("kp (N.m/rad)", a.get("kp")),
            ("sonum b (N.m.s/rad)", a.get("damping")), ("armature (kg.m^2)", a.get("armature")),
            ("frictionloss (N.m)", a.get("frictionloss")), ("olu bant (derece)", a.get("deadband_deg")),
            ("olu bant (us)", a.get("deadband_us")), ("gecikme (s)", a.get("latency_s")),
            ("i_stall (A, model)", sc.get("i_stall")), ("i_idle (A)", sc.get("i_idle")),
            ("akim modeli R^2", sc.get("r2"))]
    m = pot.get("map") or {}
    pc = pot.get("potcal") or {}
    sw = pot.get("sweep") or {}
    rows += [("pot olcek kaynagi", m.get("src")), ("pot olcek (derece/V orta uc)", pc.get("scale_deg_per_v")),
             ("pot 0 derece gerilimi (V)", pc.get("offset_v")), ("pot dogrusalsizlik (derece)", pc.get("nonlin_max_deg")),
             ("komut olcegi (derece/us)", m.get("cmd_deg_per_us")), ("hareket araligi (derece)", sw.get("travel_deg")),
             ("orta uc araligi (V)", sw.get("span_v")), ("pot beslemesi (V)", sw.get("pot_supply_v"))]
    th = r.get("thermal") or {}
    rows += [("T_diz (N.m RMS)", th.get("t_knee")), ("isil zaman sabiti (s)", th.get("tau_th_s"))]
    for k, v in rows:
        L.append("| %s | %s |" % (k, _fmt(v, "%.5g") if isinstance(v, (int, float)) else _fmt(v)))
    L.append("")
    L.append("**Karar tablosu (spec 7)**")
    L.append("")
    L.append("| Olcum | Olculen | Esik | Sonuc | Eylem |")
    L.append("|---|---|---|---|---|")
    for v in r.get("verdicts") or []:
        L.append("| %s | %s | %s | %s | %s |" % (v["item"], v["measured"] if v["measured"] is not None else "-",
                                               v["threshold"], v["result"], v["action"] + (
                                                   " (%s)" % v["note"] if v.get("note") else "")))
    return "\n".join(L)


def write_report(results, mass):
    L = ["# Adim 0 sysID raporu", "", "G-CAD kutlesi: %.2f kg. u-fit: parts-research (kappa ile, rl-map D1); "
         "bosta hiz = olculen plato hizi." % mass, ""]
    if len(results) > 1:
        L += ["## Karsilastirma", "", "| Servo | Model | tau_s | w_bosta | u(0.72) | u(0.6) | u(0.5) | kappa | T_diz | "
              "Diz | Pot | 300 Hz |", "|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in results:
            a = r.get("actuator") or {}
            g = (r.get("gate") or {}).get("u") or {}
            vd = {v["item"]: v for v in r.get("verdicts") or []}
            pot = next((v["result"] for k, v in vd.items() if k.startswith("Pot (")), "-")
            h3 = next((v["result"] for k, v in vd.items() if "300 Hz" in k), "-")
            L.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                r["servo"]["id"], r["servo"].get("model") or "?", _fmt(a.get("tau_s")), _fmt(a.get("omega_noload")),
                _fmt(g.get("0.72")), _fmt(g.get("0.6")), _fmt(g.get("0.5")), _fmt((r.get("gate") or {}).get("kappa"), "%s"),
                _fmt((r.get("thermal") or {}).get("t_knee")), _fmt((r.get("knee") or {}).get("eligible"), "%s"), pot, h3))
        knee = [r for r in results if (r.get("knee") or {}).get("eligible")]
        knee.sort(key=lambda r: min(((r.get("gate") or {}).get("u") or {"x": 9}).values()))
        L += ["", "- Diz servosu adaylari (T_diz >= diz RMS; u'ya gore): " +
              (", ".join(r["servo"]["id"] for r in knee) or "yok -> G-ISIL / spec'e don"),
              "- Diger DS eklemleri: u ve stok dengesi (DS3225 ya da DS3230); stok bilgisini BOM'dan ekleyin.", ""]
    for r in results:
        L.append(servo_section(r))
        L.append("")
    L.append("Parametreler `rl/actuators.py`'e: kp = tau_s/P-bandi, forcerange = tau_s, damping = tau_s/omega0, "
             "armature, frictionloss, olu bant; DR kabulde olculen degerlerin +-%15'i (spec 4).")
    return "\n".join(L) + "\n"
