# -*- coding: utf-8 -*-
"""
Robot konfigurasyonu (spec 3.2.1): configs/<ad>.json okunur, varsayilanlarla tamamlanir ve dogrulanir.
FreeCAD gerektirmez (tests/cad/test_build_config.py).

  OPENDUCK_CONFIG=configs/tr35_b2.json    yol (goreli ise once cwd, sonra repo koku)
  OPENDUCK_CONFIG=tr35_b2                 ad -> configs/tr35_b2.json
  (bos)                                   configs/xl330_27.json

Alanlar ("_" ile baslayan anahtarlar aciklamadir, yok sayilir):
  name, scale, generators{trunk, legs, head}, servo_map{eklem anahtari: servo turu}   zorunlu
  description, robot                       metin (robot: summary/BOM'daki ad)
  horn_offset_deg{anahtar: derece}         spline dis kaydirmasi (sol taraf; turun dis adiminin kati)
  neutral_deg{anahtar: derece}             servo orta darbesinde eklem acisi, geometriden (sol taraf).
                                           Servo merkezi = neutral + horn_offset; sagda isaret ters.
  joint_limits_deg{anahtar: [alt, ust]}    openduck.JOINT_LIMITS ustune (sol taraf)
  home_deg{eklem adi: derece}              openduck.HOME_POSE ustune
  required_deg{anahtar: [alt, ust]}        spec 3.1 gerekli eklem araliklari (A1 kontrolu icin veri)
  params{...}                              openduck.PARAMS ustune MUTLAK (mm) degerler: once 27 cm yerlesim
                                           anahtarlari (openduck.LAYOUT_KEYS) scale ile carpilir, sonra bunlar
                                           yazilir (sozluk degerleri anahtar anahtar birlesir)
  targets{...}, bom_buy[[adet, parca, not]]
  check{...}                               cad/check.py genisletilmis taramalar (P2-0): pair_steps, pair_range
                                           ("required" | "limits"), pair_scope ("module" | "all"), leg_leg,
                                           leg_leg_range; bos -> yalniz 27 cm raporu (bayt-esit)
"""
import copy
import json
import os

import servos

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(ROOT, "configs")
DEFAULT = "xl330_27"
ENV = "OPENDUCK_CONFIG"
STAGES = ("trunk", "legs", "head")          # uretec sirasi: trunk -> legs -> head

REQUIRED = ("name", "scale", "generators", "servo_map")
OPTIONAL = {"description": "", "robot": "OpenDuck", "horn_offset_deg": {}, "neutral_deg": {},
            "joint_limits_deg": {}, "home_deg": {}, "required_deg": {}, "params": {}, "targets": {},
            "bom_buy": [], "check": {}}


def resolve_path(spec=None):
    """Konfig belirteci (yol ya da ad; None -> $OPENDUCK_CONFIG ya da varsayilan) -> mutlak dosya yolu."""
    spec = spec or os.environ.get(ENV) or DEFAULT
    cands = [spec, os.path.join(ROOT, spec)]
    if not spec.endswith(".json") and os.sep not in spec and "/" not in spec:
        cands.append(os.path.join(CONFIG_DIR, spec + ".json"))
    for c in cands:
        if os.path.isfile(c):
            return os.path.abspath(c)
    raise IOError("konfig bulunamadi: %r (denenen: %s)" % (spec, ", ".join(cands)))


def load(spec=None):
    """Konfig sozlugu: spec dict ise kopyasi, degilse resolve_path(spec) dosyasi; varsayilanlar eklenir,
    validate() edilir. "path" alanina kaynak yazilir (dict icin None)."""
    if isinstance(spec, dict):
        cfg, path = copy.deepcopy(spec), None
    else:
        path = resolve_path(spec)
        with open(path) as f:
            cfg = json.load(f)
    cfg = strip_notes(cfg)
    for k, v in OPTIONAL.items():
        cfg.setdefault(k, copy.deepcopy(v))
    cfg["path"] = path
    validate(cfg)
    return cfg


def strip_notes(obj):
    """Sozluklerden (ic ice) "_" ile baslayan aciklama anahtarlarini atar."""
    if isinstance(obj, dict):
        return {k: strip_notes(v) for k, v in obj.items() if not str(k).startswith("_")}
    if isinstance(obj, list):
        return [strip_notes(v) for v in obj]
    return obj


def _range(v, what):
    if not (isinstance(v, (list, tuple)) and len(v) == 2 and v[0] <= v[1]):
        raise ValueError("%s: [alt, ust] bekleniyordu, %r" % (what, v))


def validate(cfg):
    """Yapi ve deger kontrolu; hata -> ValueError (KeyError: bilinmeyen servo)."""
    unknown = set(cfg) - set(REQUIRED) - set(OPTIONAL) - {"path"}
    missing = [k for k in REQUIRED if k not in cfg]
    if unknown or missing:
        raise ValueError("konfig %r: eksik %s, bilinmeyen %s" % (cfg.get("name"), missing, sorted(unknown)))
    name = cfg["name"]
    if not (isinstance(cfg["scale"], (int, float)) and cfg["scale"] > 0):
        raise ValueError("%s: scale > 0 olmali (%r)" % (name, cfg["scale"]))
    g = cfg["generators"]
    if sorted(g) != sorted(STAGES) or not all(isinstance(v, str) for v in g.values()):
        raise ValueError("%s: generators %s anahtarlarini (ad) icermeli: %r" % (name, list(STAGES), g))
    sm = cfg["servo_map"]
    if sorted(sm) != sorted(servos.JOINT_KEYS):
        raise ValueError("%s: servo_map tam olarak %s olmali; eksik %s, fazla %s" % (
            name, servos.JOINT_KEYS, sorted(set(servos.JOINT_KEYS) - set(sm)),
            sorted(set(sm) - set(servos.JOINT_KEYS))))
    for k in sm:
        servos.get(sm[k])
    for field in ("horn_offset_deg", "neutral_deg", "joint_limits_deg", "required_deg"):
        bad = sorted(set(cfg[field]) - set(servos.JOINT_KEYS))
        if bad:
            raise ValueError("%s: %s bilinmeyen eklem anahtarlari %s" % (name, field, bad))
    for k, off in cfg["horn_offset_deg"].items():
        step = servos.spline_step_deg(servos.get(sm[k]))
        if step is None or abs(off / step - round(off / step)) > 1e-6:
            raise ValueError("%s: %s horn ofseti %g, %s spline adiminin (%s) kati degil" % (
                name, k, off, sm[k], step))
    for field in ("joint_limits_deg", "required_deg"):
        for k, v in cfg[field].items():
            _range(v, "%s %s.%s" % (name, field, k))
    if not isinstance(cfg["params"], dict):
        raise ValueError("%s: params sozluk olmali" % name)
    for row in cfg["bom_buy"]:
        if len(row) != 3:
            raise ValueError("%s: bom_buy satiri [adet, parca, not] olmali: %r" % (name, row))


def servo_center(cfg, key):
    """Sol taraf: servo orta darbesindeki eklem acisi (derece) = neutral + horn_offset."""
    return float(cfg["neutral_deg"].get(key, 0.0)) + float(cfg["horn_offset_deg"].get(key, 0.0))


def travel_violations(cfg, ranges=None):
    """`ranges` (varsayilan required_deg) araliklarinin servo hareket araligina (merkez +- hareket/2) sigmadigi
    eklemler: [(anahtar, (alt, ust), (servo_alt, servo_ust))]. Sag taraf aynali oldugu icin sol yeterli."""
    ranges = cfg["required_deg"] if ranges is None else ranges
    out = []
    for k, (lo, hi) in sorted(ranges.items()):
        slo, shi = servos.travel_range(servos.get(cfg["servo_map"][k]), servo_center(cfg, k))
        if lo < slo - 1e-9 or hi > shi + 1e-9:
            out.append((k, (lo, hi), (slo, shi)))
    return out


def scaled(v, s):
    """Sayi, demet/liste (ic ice) ya da sozluk degerlerini s ile carpar. s == 1 icin v aynen (bayt-esitlik)."""
    if s == 1.0:
        return v
    if isinstance(v, dict):
        return type(v)((k, scaled(x, s)) for k, x in v.items())
    if isinstance(v, (list, tuple)):
        return tuple(scaled(x, s) for x in v)
    return v * s


def merge(base, over):
    """base'in derin kopyasi uzerine over: sozluk degerleri anahtar anahtar birlesir, digerleri yer degistirir."""
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out
