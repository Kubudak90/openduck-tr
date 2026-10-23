# -*- coding: utf-8 -*-
"""
MuJoCo duman testi: build/sim/scene.xml'i yukler, robotu "stand" (HOME) pozunda birakir ve
pozisyon kontrolcusuyle sirasiyla ayakta durur, comelir, kafasini cevirir ve gagasini acip kapar.
Ayakta kalip kalmadigini ve servo torklarinin XL330 sinirina ne kadar yaklastigini raporlar.

  pip install mujoco
  python sim/stand_test.py [--scene build/sim/scene.xml] [--view] [--render build/img/mujoco.png]
"""
import math
import os
import sys

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def arg(name, default):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


def tilt_deg(quat):
    w, x, y, z = quat
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def crouch_targets(m, home, names, dhip_deg):
    """Kalca pitch'i dhip kadar degistirip ayak bilegini kalcanin altinda tutan diz/bilek acilari."""
    d = mujoco.MjData(m)
    q = home.copy()
    for side, sgn in (("left", 1.0), ("right", -1.0)):
        ih, ik, ia = (names.index(side + "_" + j) for j in ("hip_pitch", "knee", "ankle"))
        jid = [m.actuator_trnid[i, 0] for i in (ih, ik, ia)]
        adr = [m.jnt_qposadr[j] for j in jid]

        def ankle_x(qh, qk):
            d.qpos[:] = m.key_qpos[0]
            d.qpos[adr[0]], d.qpos[adr[1]] = qh, qk
            mujoco.mj_kinematics(m, d)
            return d.xanchor[jid[2]][0] - d.xanchor[jid[0]][0]
        target = ankle_x(home[ih], home[ik])
        qh = home[ih] + sgn * math.radians(dhip_deg)
        lo, hi = home[ik] - 2.0, home[ik] + 2.0
        f_lo = ankle_x(qh, lo) - target
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            f_mid = ankle_x(qh, mid) - target
            if (f_mid > 0) == (f_lo > 0):
                lo, f_lo = mid, f_mid
            else:
                hi = mid
        qk = 0.5 * (lo + hi)
        q[ih], q[ik] = qh, qk
        q[ia] = home[ia] - (qh - home[ih]) + (qk - home[ik])   # ayak tabani duz kalsin
    return q


def render(m, d, path, size=(900, 1200)):
    """Ekransiz goruntu (Linux'ta MUJOCO_GL=egl veya osmesa gerekir)."""
    from PIL import Image
    r = mujoco.Renderer(m, size[0], size[1])
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.0, 0.0, 0.13]
    cam.distance, cam.azimuth, cam.elevation = 0.62, 215, -14
    r.update_scene(d, cam)
    Image.fromarray(r.render()).save(path)
    r.close()
    print("goruntu: %s" % path)


def main():
    scene = arg("--scene", os.path.join(ROOT, "build", "sim", "scene.xml"))
    m = mujoco.MjModel.from_xml_path(scene)
    d = mujoco.MjData(m)
    key = m.key("stand").id
    mujoco.mj_resetDataKeyframe(m, d, key)
    mujoco.mj_forward(m, d)
    print("govde sayisi %d, eklem %d, aktuator %d, toplam kutle %.3f kg" % (m.nbody, m.njnt, m.nu, sum(m.body_mass)))
    trunk = m.body("trunk").id
    z0 = d.xpos[trunk][2]
    home = d.ctrl.copy()
    names = [m.actuator(i).name for i in range(m.nu)]
    fmax = m.actuator_forcerange[:, 1]

    def run(seconds, ctrl_fn, label):
        peak = np.zeros(m.nu)
        zmin, tmax = 1e9, 0.0
        for k in range(int(seconds / m.opt.timestep)):
            d.ctrl[:] = np.clip(ctrl_fn(k * m.opt.timestep), m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
            mujoco.mj_step(m, d)
            peak = np.maximum(peak, np.abs(d.actuator_force))
            zmin, tmax = min(zmin, d.xpos[trunk][2]), max(tmax, tilt_deg(d.xquat[trunk]))
        worst = np.argsort(-(peak / fmax))[:4]
        print("%-10s govde z min %.3f m (HOME %.3f), en fazla egim %.1f derece" % (label, zmin, z0, tmax))
        print("           en yuklu: " + ", ".join("%s %.2f N.m (%%%d)" % (names[i], peak[i], 100 * peak[i] / fmax[i])
                                                for i in worst))
        return zmin, tmax

    ok = True
    zmin, tmax = run(3.0, lambda t: home, "ayakta")
    ok &= zmin > 0.85 * z0 and tmax < 12
    if "--render" in sys.argv:
        render(m, d, arg("--render", ""))

    low = crouch_targets(m, home, names, 20.0)
    zmin, tmax = run(4.0, lambda t: home + (0.5 - 0.5 * math.cos(math.pi * t)) * (low - home), "comelme")
    ok &= zmin > 0.6 * z0 and tmax < 15

    ih = [names.index(n) for n in ("head_yaw", "neck_pitch", "head_pitch")]

    def head_fn(t):
        c = home.copy()
        # Kafa+boyun kutlenin ~%40'i: boyun pitch'i acik dongude buyuk oynatilirsa robot devrilir;
        # buyuk hareketleri dengeleyen yurume/durma politikasi yapar. Burada kucuk genlik.
        c[ih[0]] += math.radians(60) * math.sin(2 * math.pi * 0.4 * t)
        c[ih[1]] += math.radians(3) * math.sin(2 * math.pi * 0.4 * t)
        c[ih[2]] += math.radians(10) * math.sin(2 * math.pi * 0.4 * t)
        return c
    zmin, tmax = run(5.0, head_fn, "kafa")
    ok &= zmin > 0.85 * z0 and tmax < 15

    im = names.index("mouth")

    def beak_fn(t):
        c = home.copy()
        c[im] = math.radians(15) * (1 - math.cos(2 * math.pi * t))
        return c
    zmin, tmax = run(2.0, beak_fn, "gaga")
    ok &= zmin > 0.85 * z0 and tmax < 15
    print("SONUC: %s" % ("GECTI" if ok else "KALDI"))

    if "--view" in sys.argv:
        from mujoco import viewer
        mujoco.mj_resetDataKeyframe(m, d, key)
        viewer.launch(m, d)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
