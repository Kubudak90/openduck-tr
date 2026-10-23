# -*- coding: utf-8 -*-
"""
OpenDuck geometri yardimcilari (cad/openduck.py'den ayrildi; davranis aynen). Butun olculer mm, kanonik/dunya
cerceveleri openduck.py'deki gibi. openduck, servos ve uretec modulleri (trunk, legs, head) bunlari kullanir.
"""
import math

import FreeCAD as App
import Part

V = App.Vector


def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def xcyl(r, y, z, x0, x1):
    return Part.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def ycyl(r, x, z, y0, y1):
    return Part.makeCylinder(r, y1 - y0, V(x, y0, z), V(0, 1, 0))


def zcyl(r, x, y, z0, z1):
    return Part.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def _parallel(edge, axis):
    if not isinstance(edge.Curve, Part.Line):
        return False
    d = edge.Vertexes[-1].Point - edge.Vertexes[0].Point
    return d.Length > 1e-9 and abs(abs(d.normalize().dot(V(*axis))) - 1) < 1e-6


def _at_z(edge, z):
    bb = edge.BoundBox
    return abs(bb.ZMin - z) < 1e-6 and abs(bb.ZMax - z) < 1e-6


def fillet(shape, r, edges):
    """Kenar yuvarlatma; OCC tumunu yapamazsa kenar kenar dener, olmayanlari atlar."""
    if r <= 0 or not edges:
        return shape
    try:
        return shape.makeFillet(r, edges)
    except Exception:
        pass
    out = shape
    for e in edges:
        try:
            match = [f for f in out.Edges if f.isSame(e) or (f.CenterOfMass - e.CenterOfMass).Length < 1e-6]
            if match:
                out = out.makeFillet(r, match[:1])
        except Exception:
            continue
    return out


def fillet_axis(shape, r, axis):
    return fillet(shape, r, [e for e in shape.Edges if _parallel(e, axis)])


def rbox(x0, x1, y0, y1, z0, z1, r_side=0.0, r_top=0.0, r_bot=0.0):
    """Dikey kenarlari r_side, ust kenarlari r_top, alt kenarlari r_bot ile yuvarlatilmis kutu."""
    b = box(x0, x1, y0, y1, z0, z1)
    b = fillet_axis(b, r_side, (0, 0, 1))
    b = fillet(b, r_top, [e for e in b.Edges if _at_z(e, z1)])
    b = fillet(b, r_bot, [e for e in b.Edges if _at_z(e, z0)])
    return b


def shell_rbox(x0, x1, y0, y1, z0, z1, wall, r_side, r_top, r_bot):
    outer = rbox(x0, x1, y0, y1, z0, z1, r_side, r_top, r_bot)
    inner = rbox(x0 + wall, x1 - wall, y0 + wall, y1 - wall, z0 + wall, z1 - wall,
                 max(r_side - wall, 0.5), max(r_top - wall, 0.0), max(r_bot - wall, 0.0))
    return outer.cut(inner)


def stadium_y(cx, cz, r, z_end, y0, y1, x_half=None):
    """XZ duzleminde (cx,cz) etrafinda r yaricapli disk + z_end'e uzanan dikdortgen, Y'de ekstrude."""
    lo, hi = sorted((cz, z_end))
    w = r if x_half is None else x_half
    if hi - lo < 1e-6:
        return ycyl(r, cx, cz, y0, y1)
    return ycyl(r, cx, cz, y0, y1).fuse(box(cx - w, cx + w, y0, y1, lo, hi)).removeSplitter()


def hull2d(points):
    pts = sorted(set(points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def prism_xz(pts, y0, y1, r_round=0.0):
    """XZ duzleminde (x, z) cokgen, Y yonunde y0..y1 ekstrude; kose yuvarlatma r_round."""
    poly = Part.makePolygon([V(x, y0, z) for x, z in pts] + [V(pts[0][0], y0, pts[0][1])])
    solid = Part.Face(poly).extrude(V(0, y1 - y0, 0))
    return fillet_axis(solid, r_round, (0, 1, 0)) if r_round > 0 else solid


def bar_y(p0, p1, r, y0, y1):
    """XZ duzleminde iki daire (p0, p1; yaricap r) arasi kapsul, Y'de ekstrude."""
    (x0, z0), (x1, z1) = p0, p1
    dx, dz = x1 - x0, z1 - z0
    n = math.hypot(dx, dz)
    ox, oz = -dz / n * r, dx / n * r
    pts = [(x0 + ox, z0 + oz), (x1 + ox, z1 + oz), (x1 - ox, z1 - oz), (x0 - ox, z0 - oz)]
    return prism_xz(pts, y0, y1).fuse([ycyl(r, x0, z0, y0, y1), ycyl(r, x1, z1, y0, y1)]).removeSplitter()


def frame(origin, horn, far):
    """Kanonik servo cercevesi -> dunya yerlesimi."""
    Y, Z = V(*horn), V(*far)
    X = Y.cross(Z)
    m = App.Matrix(X.x, Y.x, Z.x, origin[0],
                   X.y, Y.y, Z.y, origin[1],
                   X.z, Y.z, Z.z, origin[2],
                   0, 0, 0, 1)
    return App.Placement(m)


def moved(shape, placement):
    s = shape.copy()
    s.Placement = placement.multiply(s.Placement)
    return s


def rotated(shape, center, axis, angle_deg):
    s = shape.copy()
    s.rotate(V(*center), V(*axis), angle_deg)
    return s


_MY = App.Matrix(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)


def mirror_y(shape):
    """XZ duzlemine gore ayna. Shape.mirror() yerlesimli sekillerde yanlis sonuc
    verdigi icin yerlesim ayrica aynalanir: M*P*S = (M*P*M) * (M*S)."""
    s = shape.copy()
    pm = s.Placement.toMatrix()
    s.Placement = App.Placement()
    m = s.mirror(V(0, 0, 0), V(0, 1, 0))
    m.Placement = App.Placement(_MY.multiply(pm).multiply(_MY)).multiply(m.Placement)
    return m
