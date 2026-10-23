# -*- coding: utf-8 -*-
# OpenDuck tam derleme betigi. FreeCAD icinden calistirilir (bkz. tools/build.sh).
# GUI varsa renkler ve gorseller de uretilir; yoksa yalnizca geometri ciktilari.
import os
import sys
import time
import traceback

import FreeCAD as App

ROOT = os.environ.get("OPENDUCK_ROOT") or os.getcwd()
OUT = os.environ.get("OPENDUCK_OUT") or os.path.join(ROOT, "build")
sys.path.insert(0, os.path.join(ROOT, "cad"))

import openduck  # noqa: E402
import export  # noqa: E402

LOG = open(os.path.join(OUT, "build.log"), "w") if os.path.isdir(OUT) else None


def log(msg):
    line = "[%6.1fs] %s\n" % (time.time() - T0, msg)
    sys.__stdout__.write(line)
    sys.__stdout__.flush()
    if LOG:
        LOG.write(line)
        LOG.flush()


T0 = time.time()
code = 0
try:
    os.makedirs(OUT, exist_ok=True)
    if LOG is None:
        LOG = open(os.path.join(OUT, "build.log"), "w")
    App.ParamGet("User parameter:BaseApp/Preferences/Document").SetBool("CreateBackupFiles", False)
    log("model kuruluyor (GUI=%s)" % App.GuiUp)
    model = openduck.OpenDuck()
    doc = openduck.build_document(model)
    if App.GuiUp:
        import FreeCADGui as Gui
        Gui.activeDocument().activeView().viewIsometric()
    log("%d link, %d eklem, %.3f kg" % (len(model.links), len(model.joints), model.total_mass()))

    summ = export.summary(model)
    export.write_json(os.path.join(OUT, "summary.json"), summ)
    export.write_bom(model, os.path.join(OUT, "BOM.md"), summ)
    log("ozet + BOM: %s" % dict(summ))

    export.save_fcstd(doc, os.path.join(OUT, "openduck.FCStd"))
    export.export_step(doc, os.path.join(OUT, "openduck.step"))
    log("FCStd + STEP yazildi")

    files = export.export_print_stls(model, os.path.join(OUT, "stl"))
    log("%d baski STL'i" % len(files))

    export.export_robot(model, os.path.join(OUT, "sim"))
    log("URDF + MJCF + meshes yazildi")

    render = App.GuiUp and os.environ.get("OPENDUCK_RENDER", "1") == "1"
    if render:
        imgs = export.render_views(doc, model, os.path.join(OUT, "img"))
        log("%d gorsel" % len(imgs))
    if os.environ.get("OPENDUCK_DOCS", "1") == "1":
        import docs
        res = docs.build_all(model, doc, OUT, summ, gui=render)
        log("baski seti: %d parca cesidi -> print/, openduck_print_parts.zip" % res["parts"])
        if render:
            log("dokumanlar: %d patlatilmis gorsel, katalog, %d teknik resim sayfasi, %s"
                % (len(res["exploded"]), res["sheets"], os.path.basename(res["pdf"])))
    if render:
        doc.save()
    log("bitti")
except Exception:
    log("HATA:\n" + traceback.format_exc())
    code = 1
finally:
    if LOG:
        LOG.close()
    if App.GuiUp:
        os._exit(code)
    elif code:
        sys.exit(code)
