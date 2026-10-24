# -*- coding: utf-8 -*-
"""
Kablo demeti guzergahi geometrisi (FreeCAD gerektirmez; cad/legs.py kullanir, tests/cad/test_route.py dener).

  fillet(noktalar, yaricaplar)   kose basina bukulme yaricapi (0: keskin kose) -> yaylari ayrik noktali polyline
  min_bend_radius(noktalar)      polyline'in fillet ile kabul ettigi en buyuk ortak yaricap (butun koselerin teget
                                 boylari kendi parcalarina sigar)
  crossing_bend(...)             eklem gecisinde dinamik bukulme (p2-interface.md 6: bend_r = 4 d, dinamik): gecis
                                 noktasinda in ve out tuplerinin duz kollari, eklem sinirlari boyunca en buyuk donus
                                 acisi ve kollarin tasiyabildigi yay yaricapi
"""
import math


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _turn(p0, p1, p2):
    """p1 kosesindeki donus acisi (radyan, 0 = duz)."""
    u, w = _unit(_sub(p1, p0)), _unit(_sub(p2, p1))
    return math.acos(max(-1.0, min(1.0, _dot(u, w))))


def fillet(pts, radii, step_deg=10.0):
    """Ic koselere (radii[i] > 0) kose duzleminde iki parcaya tegetli yay; yay en cok step_deg'lik esit dilimlerle
    yazilir ve dilim koseleri yaricap / cos(dilim / 2) uzakta durur: ayrik polyline'in min_bend_radius'u radii[i]'nin
    altina dusmez. Teget boylari parcalara sigmazsa ValueError. Uc noktalar ve radii[i] = 0 koseler oldugu gibi."""
    P = [tuple(float(c) for c in p) for p in pts]
    n = len(P)
    assert len(radii) == n, "her noktaya bir yaricap"
    th, t, m = [0.0] * n, [0.0] * n, [1] * n
    for i in range(1, n - 1):
        th[i] = _turn(P[i - 1], P[i], P[i + 1])
        if radii[i] > 0 and th[i] > 1e-6:
            m[i] = max(1, int(math.ceil(math.degrees(th[i]) / step_deg - 1e-9)))
            t[i] = radii[i] / math.cos(th[i] / m[i] / 2.0) * math.tan(th[i] / 2.0)
    for i in range(n - 1):
        L = math.dist(P[i], P[i + 1])
        if t[i] + t[i + 1] > L + 1e-9:
            raise ValueError("fillet: parca %d (%.2f mm) teget boylarina (%.2f mm) yetmiyor" % (i, L, t[i] + t[i + 1]))
    out = [P[0]]
    for i in range(1, n - 1):
        if t[i] <= 0.0:
            out.append(P[i])
            continue
        u, w = _unit(_sub(P[i], P[i - 1])), _unit(_sub(P[i + 1], P[i]))
        rho = radii[i] / math.cos(th[i] / m[i] / 2.0)
        bis = _unit(_sub(w, u))
        c = tuple(P[i][j] + bis[j] * rho / math.cos(th[i] / 2.0) for j in range(3))
        a = tuple(P[i][j] - u[j] * t[i] - c[j] for j in range(3))
        b = tuple(P[i][j] + w[j] * t[i] - c[j] for j in range(3))
        s = math.sin(th[i])
        for k in range(m[i] + 1):
            f1, f2 = math.sin((1.0 - k / m[i]) * th[i]) / s, math.sin(k / m[i] * th[i]) / s
            q = tuple(c[j] + f1 * a[j] + f2 * b[j] for j in range(3))
            if math.dist(q, out[-1]) > 1e-6:          # bitisik iki yayin ortak teget noktasi tek kez
                out.append(q)
    if math.dist(P[-1], out[-1]) > 1e-6:
        out.append(P[-1])
    return out


def min_bend_radius(pts):
    """Polyline koselerinin hepsi ayni yaricapla fillet edilebilecek en buyuk yaricap (mm; kose yoksa inf): her parca
    iki ucundaki teget boylarini (r tan(aci / 2)) tasir, uc parcalar tek ucunkini."""
    n = len(pts)
    tn = [0.0] * n
    for i in range(1, n - 1):
        tn[i] = math.tan(_turn(pts[i - 1], pts[i], pts[i + 1]) / 2.0)
    r = float("inf")
    for i in range(n - 1):
        s = tn[i] + tn[i + 1]
        if s > 1e-12:
            r = min(r, math.dist(pts[i], pts[i + 1]) / s)
    return r


def _straight(pts, tol_deg=0.5):
    """pts[0]'dan baslayan duz kolun boyu ve yonu (ardisik parcalar tol_deg icinde ayni dogrultuda)."""
    u = _unit(_sub(pts[1], pts[0]))
    L = math.dist(pts[0], pts[1])
    for a, b in zip(pts[1:], pts[2:]):
        w = _unit(_sub(b, a))
        if math.degrees(math.acos(max(-1.0, min(1.0, _dot(u, w))))) > tol_deg:
            break
        L += math.dist(a, b)
    return L, u


def _rot(v, axis, deg):
    a = _unit(axis)
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    d = _dot(a, v)
    x = (a[1] * v[2] - a[2] * v[1], a[2] * v[0] - a[0] * v[2], a[0] * v[1] - a[1] * v[0])
    return tuple(v[i] * c + x[i] * s + a[i] * d * (1.0 - c) for i in range(3))


def crossing_bend(in_pts, out_pts, axis, lo, hi, step=0.5):
    """Eklem gecisinde dinamik bukulme: in_pts parent link'te gecise biter, out_pts child link'te gecisten baslar
    (sifir poz, ayni nokta). Child eksen (axis, parent cercevesi) etrafinda q derece donunce out kolunun yonu
    R(q) u_out; gecisteki donus acisi beta(q) = aci(u_in, R(q) u_out). Kablo iki duz kol boyunca serbest: yay
    yaricapi R = min(L_in, L_out) / tan(beta_max / 2). Doner {"L_in", "L_out", "beta_max", "q0" (duz gecis acisi),
    "R"}."""
    assert math.dist(in_pts[-1], out_pts[0]) < 1e-6, "in ve out ayni gecis noktasinda bulusur"
    L_in, u = _straight(list(reversed(in_pts)))
    u_in = tuple(-c for c in u)
    L_out, u_out = _straight(out_pts)
    n = max(1, int(round((hi - lo) / step)))
    best, q0, bmax = 9.0, lo, 0.0
    for k in range(n + 1):
        q = lo + (hi - lo) * k / n
        b = math.acos(max(-1.0, min(1.0, _dot(u_in, _rot(u_out, axis, q)))))
        bmax = max(bmax, b)
        if b < best:
            best, q0 = b, q
    R = min(L_in, L_out) / math.tan(bmax / 2.0) if bmax > 1e-9 else float("inf")
    return {"L_in": L_in, "L_out": L_out, "beta_max": math.degrees(bmax), "q0": q0, "R": R}
