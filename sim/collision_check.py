# -*- coding: utf-8 -*-
"""
Gorsel mesh oz-carpisma denetimi (spec A8): phys-sit voxel aracinin (collide.py / traj_collide.py) repo'ya
tasinmis hali. Yalniz numpy + mujoco (trimesh/scipy gerekmez).

Yontem: gorsel modeldeki (rl.robot_model.build_model(visual=True)) her mesh geom, kendi cercevesinde PITCH
aralikli dolu bir voxel izgarasina cevrilir: yuzey voxelleri (ucgenler PITCH/2'den sik orneklenir) + ic voxeller
(z ekseni boyunca isin paritesi; kapali olmayan sutunlar ic sayilmaz). Bir pozda FARKLI govdelerdeki iki parcanin
girisim hacmi = kucuk parcanin dolu voxel merkezlerinden digerinin dolu voxellerine dusenlerin sayisi x PITCH^3
(mm^3; temas eden yuzeyler birkac mm^3 verir). HOME pozunda zaten girisen parca ciftleri (servo uzerindeki horn
gibi eklem arayuzleri) taban cizgisidir ve haric tutulur. Kabul (A8): bir yorungede taban disi girisim <= 1 mm^3.

Eklem toleransi: govdeleri dogrudan bir eklemle bagli (ebeveyn/cocuk) parca ciftlerinde <= JOINT_TOL_VOX (4) voxel
(4 * PITCH^3 = 2.048 mm^3, 0.8 mm'de) girisim yok sayilir. Neden: diz arayuzu (thigh_inner_plate ~ shin_bracket)
HOME yakinindaki diz acilarinda 0/1/2 voxel (0.512/1.024 mm^3) arasinda titrer (voxel orneklemesi; HOME'da 0 oldugu
icin taban cizgisinde degil); gercek girisim yokken ayakta duran robot A8'den kalirdi. Diger tum ciftlerde (or.
incik - govde) sinir 1 mm^3 olarak kalir. Yok sayilan eklem ciftleri sonucta "joint_tolerated" altinda raporlanir.

  python3 sim/collision_check.py traj.npy [--stride 5] [--pitch 0.0008] [--limit 1.0] [--json out.json]
  # yorunge: (T, nq) qpos dizisi (.npy, ya da "qpos" anahtarli .npz), or. rl.play --qpos-out traj.npy

API:
  ms = mesh_set(pitch=PITCH)            # surec ici onbellek (kurulum birkac saniye)
  ms.baseline()                         # {(parca_a, parca_b): mm3} HOME'da girisenler
  recs, minz = ms.pose(qpos)            # [(parca_a, parca_b, govde_a, govde_b, mm3)], {govde: (z, parca)}
  r = check_pose(qpos)                  # taban disi girisimler
  r = check_trajectory(qpos_traj, stride=5)
  ms.ignored(a, b, govde_a, govde_b, mm3)  # taban cizgisi ya da eklem toleransi icinde mi
"""
import argparse
import functools
import json
import os
import sys

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

PITCH = 0.0008          # m
AABB_PAD = 0.006        # m; bu kadar yakin olmayan parca ciftleri denetlenmez
LIMIT_MM3 = 1.0
JOINT_TOL_VOX = 4       # ebeveyn/cocuk (eklemli) govde ciftlerinde yok sayilan en fazla voxel sayisi
BLOCK_VOX = 8           # _pen hizlandirma: dolu voxel merkezleri bu kenarli (voxel) bloklarda gruplanir


def _surface_points(V, F, step):
    """Ucgenlerin kenar <= step olana dek ortadan 4'e bolunmesiyle olusan kose noktalari (trimesh
    subdivide_to_size ile ayni nokta kumesi: ucgen basina 2^k bolmeli barisentrik izgara)."""
    A, B, Cc = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    L = np.max(np.stack([np.linalg.norm(B - A, axis=1), np.linalg.norm(Cc - B, axis=1),
                         np.linalg.norm(A - Cc, axis=1)], 1), 1)
    n = 2 ** np.ceil(np.log2(np.maximum(L / step, 1.0))).astype(np.int64)
    out = [V]
    for k in np.unique(n):
        i, j = np.meshgrid(np.arange(k + 1), np.arange(k + 1), indexing="ij")
        m = (i + j) <= k
        w1, w2 = i[m] / k, j[m] / k
        w0 = 1.0 - w1 - w2
        t = n == k
        p = A[t, None] * w0[None, :, None] + B[t, None] * w1[None, :, None] + Cc[t, None] * w2[None, :, None]
        out.append(p.reshape(-1, 3))
    return np.concatenate(out)


def _interior(V, F, origin, shape, pitch):
    """(shape) bool: voxel merkezi mesh icinde (z ekseni isin paritesi; tek sayida kesisimli sutunlar atlanir)."""
    nx, ny, nz = shape
    jit = pitch * 1e-3 * np.array([0.318309886, 0.271828183])      # kenar/kose isabetlerini onler
    A, B, Cc = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    lo = np.minimum(np.minimum(A, B), Cc)[:, :2]
    hi = np.maximum(np.maximum(A, B), Cc)[:, :2]
    i0 = np.ceil((lo - origin[:2] - jit) / pitch - 0.5).astype(np.int64)
    i1 = np.floor((hi - origin[:2] - jit) / pitch - 0.5).astype(np.int64)
    cnt = np.maximum(i1 - i0 + 1, 0)
    ntri = cnt[:, 0] * cnt[:, 1]
    occ = np.zeros(shape, bool)
    if ntri.sum() == 0:
        return occ
    tri = np.repeat(np.arange(len(F)), ntri)
    k = np.arange(ntri.sum()) - np.repeat(np.cumsum(ntri) - ntri, ntri)
    ci = i0[tri, 0] + k // cnt[tri, 1]
    cj = i0[tri, 1] + k % cnt[tri, 1]
    px = origin[0] + (ci + 0.5) * pitch + jit[0]
    py = origin[1] + (cj + 0.5) * pitch + jit[1]
    a, b, c = A[tri], B[tri], Cc[tri]
    d = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    ok = np.abs(d) > 1e-18
    d = np.where(ok, d, 1.0)
    w0 = ((b[:, 0] - px) * (c[:, 1] - py) - (b[:, 1] - py) * (c[:, 0] - px)) / d
    w1 = ((c[:, 0] - px) * (a[:, 1] - py) - (c[:, 1] - py) * (a[:, 0] - px)) / d
    w2 = 1.0 - w0 - w1
    hit = ok & (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
    col = (ci * ny + cj)[hit]
    z = (w0 * a[:, 2] + w1 * b[:, 2] + w2 * c[:, 2])[hit]
    inside = (ci >= 0) & (ci < nx) & (cj >= 0) & (cj < ny)
    col, z = col[inside[hit]], z[inside[hit]]
    if not len(col):
        return occ
    zr = (z - origin[2]) / (nz * pitch + 1e-12)
    keys = np.sort(col + np.clip(zr, 0.0, 0.999999))
    cols = np.unique(col)
    nhit = np.searchsorted(keys, cols + 1.0) - np.searchsorted(keys, cols.astype(np.float64))
    cols = cols[nhit % 2 == 0]                                    # kapali (cift kesisimli) sutunlar
    zc = (np.arange(nz) + 0.5) / nz
    q = cols[:, None] + np.clip(zc[None, :], 0.0, 0.999999)
    below = np.searchsorted(keys, q.ravel()).reshape(q.shape) - np.searchsorted(keys, cols.astype(np.float64))[:, None]
    ins = below % 2 == 1
    ii, kk = np.nonzero(ins)
    occ[cols[ii] // ny, cols[ii] % ny, kk] = True
    return occ


class MeshSet(object):
    """Gorsel modelin mesh geomlarinin dolu voxel izgaralari (mesh = geom cercevesi)."""

    def __init__(self, model=None, pitch=PITCH):
        if model is None:
            from rl.robot_model import build_model
            model = build_model(visual=True)
        self.m = m = model
        self.d = mujoco.MjData(m)
        self.pitch = pitch
        self.geoms = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
        self.data = {}
        for g in self.geoms:
            mid = m.geom_dataid[g]
            va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
            fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
            V = m.mesh_vert[va:va + vn].astype(np.float64)
            F = m.mesh_face[fa:fa + fn].astype(np.int64)
            # voxel merkezleri mesh cercevesinde pitch'in tam katlari (trimesh voxelize ile ayni hizalama)
            oi = np.floor(V.min(0) / pitch).astype(np.int64) - 2
            shape = tuple(int(x) for x in np.ceil(V.max(0) / pitch).astype(np.int64) + 3 - oi)
            origin = (oi - 0.5) * pitch                                  # (0,0,0) voxelinin kosesi
            occ = _interior(V, F, origin, shape, pitch)
            ijk = np.round(_surface_points(V, F, 0.5 * pitch) / pitch).astype(np.int64) - oi
            ijk = np.clip(ijk, 0, np.array(shape) - 1)
            occ[ijk[:, 0], ijk[:, 1], ijk[:, 2]] = True
            ijk = np.argwhere(occ)
            # merkezler BLOCK_VOX bloklarina gore sirali: _pen once blok merkezlerini eler (sonuc ayni, daha hizli)
            blk = ijk // BLOCK_VOX
            bid = np.ravel_multi_index(blk.T, tuple(int(x) for x in blk.max(0) + 1)) if len(ijk) else blk[:, 0]
            order = np.argsort(bid, kind="stable")
            ijk, bid = ijk[order], bid[order]
            cnt = np.unique(bid, return_counts=True)[1]
            centers = origin + (ijk + 0.5) * pitch
            if len(cnt):        # blok merkezi ve Oklid yaricapi (donusumle korunur -> her eksende sinir)
                starts = np.concatenate([[0], np.cumsum(cnt)[:-1]])
                bc = np.add.reduceat(centers, starts, axis=0) / cnt[:, None]
                br = np.maximum.reduceat(np.linalg.norm(centers - np.repeat(bc, cnt, axis=0), axis=1), starts)
            else:
                bc, br = np.zeros((0, 3)), np.zeros(0)
            self.data[g] = dict(V=V, occ=occ, origin=origin, centers=centers, body=int(m.geom_bodyid[g]),
                                name=m.mesh(mid).name, blk_c=bc, blk_r=br + pitch, blk_n=cnt,
                                hi=origin + np.array(occ.shape) * pitch)
        key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "stand")
        self.home_qpos = m.key_qpos[key].copy() if key >= 0 else m.qpos0.copy()
        self._base = None
        # dogrudan eklemle bagli (ebeveyn/cocuk) govde adi ciftleri ve eklem toleransi (mm^3)
        self.joint_pairs = set()
        for b in range(1, m.nbody):
            p = int(m.body_parentid[b])
            if p > 0 and m.body_jntnum[b] > 0:
                self.joint_pairs.add(frozenset((m.body(p).name, m.body(b).name)))
        self.joint_tol_mm3 = JOINT_TOL_VOX * (pitch * 1000.0) ** 3

    def _pen(self, ga, gb, X):
        A, B = self.data[ga], self.data[gb]
        if len(A["centers"]) > len(B["centers"]):
            ga, gb, A, B = gb, ga, B, A
        pa, Ra = X[ga]
        pb, Rb = X[gb]
        # blok elemesi: blok merkezi B izgarasinin (blok yaricapi + PITCH kadar genisletilmis) disindaysa bloktaki
        # hicbir merkez B izgarasina dusmez (kati donusum mesafeyi korur) -> yalniz kalan bloklarin merkezleri
        # (sayim elemesiz hesapla ayni; tam donusum ifadesi degismedi)
        bl = ((A["blk_c"] @ Ra.T + pa) - pb) @ Rb
        r = A["blk_r"][:, None]
        keep = np.all((bl >= B["origin"] - r) & (bl <= B["hi"] + r), axis=1)
        if not keep.any():
            return 0.0
        loc = ((A["centers"][np.repeat(keep, A["blk_n"])] @ Ra.T + pa) - pb) @ Rb
        ijk = np.floor((loc - B["origin"]) / self.pitch).astype(np.int64)
        occ = B["occ"]
        ok = np.all((ijk >= 0) & (ijk < np.array(occ.shape)), axis=1)
        if not ok.any():
            return 0.0
        ijk = ijk[ok]
        return float(occ[ijk[:, 0], ijk[:, 1], ijk[:, 2]].sum()) * (self.pitch * 1000.0) ** 3

    def pose(self, qpos, aabb_pad=AABB_PAD):
        """qpos (nq,) -> ([(parca_a, parca_b, govde_a, govde_b, mm3)] girisen ciftler, {govde: (en alcak z, parca)})."""
        m, d = self.m, self.d
        d.qpos[:] = qpos
        mujoco.mj_kinematics(m, d)
        X, box = {}, {}
        G = self.geoms
        lo, hi = np.empty((len(G), 3)), np.empty((len(G), 3))
        for i, g in enumerate(G):
            p, R = d.geom_xpos[g].copy(), d.geom_xmat[g].reshape(3, 3).copy()
            X[g] = (p, R)
            w = self.data[g]["V"] @ R.T + p
            lo[i], hi[i] = w.min(0), w.max(0)
            box[g] = (lo[i], hi[i])
        # AABB (pad'li) ortusmesi ve farkli govde; i < j sirasi ic ice dongu ile ayni
        body = np.array([self.data[g]["body"] for g in G])
        ov = np.all(lo[:, None] - aabb_pad <= hi[None, :], -1) & np.all(lo[None, :] - aabb_pad <= hi[:, None], -1)
        ov &= body[:, None] != body[None, :]
        out = []
        for i, j in zip(*np.nonzero(np.triu(ov, 1))):
            ga, gb = G[i], G[j]
            A, B = self.data[ga], self.data[gb]
            pen = self._pen(ga, gb, X)
            if pen > 0.0:
                out.append((A["name"], B["name"], m.body(A["body"]).name, m.body(B["body"]).name, pen))
        minz = {}
        for g in self.geoms:
            b = m.body(self.data[g]["body"]).name
            z = float(box[g][0][2])
            if b not in minz or z < minz[b][0]:
                minz[b] = (z, self.data[g]["name"])
        return out, minz

    def baseline(self):
        """HOME pozunda girisen parca ciftleri {(a, b): mm3} (eklem arayuzleri; denetimde haric)."""
        if self._base is None:
            recs, _ = self.pose(self.home_qpos)
            self._base = {tuple(sorted((a, b))): pen for a, b, _, _, pen in recs}
        return self._base

    def joint_tolerated(self, body_a, body_b, mm3):
        """Ebeveyn/cocuk (eklemli) govdeler ve girisim <= JOINT_TOL_VOX voxel."""
        return frozenset((body_a, body_b)) in self.joint_pairs and mm3 <= self.joint_tol_mm3 + 1e-9

    def ignored(self, a, b, body_a, body_b, mm3):
        """None (denetlenir) | "baseline" (HOME'da girisen cift) | "joint" (eklem toleransi icinde)."""
        if tuple(sorted((a, b))) in self.baseline():
            return "baseline"
        return "joint" if self.joint_tolerated(body_a, body_b, mm3) else None

    def info(self):
        """Sonuclara eklenen yontem bilgisi."""
        return {"pitch_mm": 1000.0 * self.pitch, "baseline_pairs": len(self.baseline()),
                "joint_tol_voxels": JOINT_TOL_VOX, "joint_tol_mm3": self.joint_tol_mm3,
                "joint_tol_rule": "ebeveyn/cocuk (eklemli) govde ciftlerinde <= %d voxel yok sayilir" % JOINT_TOL_VOX}


@functools.lru_cache(maxsize=4)
def mesh_set(pitch=PITCH):
    return MeshSet(pitch=pitch)


def check_pose(qpos, ms=None):
    """Tek poz: taban disi ve eklem toleransi disi girisimler {"pen_mm3": en buyuk cift, "pairs": [...],
    "joint_tolerated": [...] (yok sayilan eklem ciftleri), "min_mesh_z_mm"}."""
    ms = ms or mesh_set()
    recs, minz = ms.pose(np.asarray(qpos, dtype=np.float64))
    pairs, tol = [], []
    for a, b, ba, bb, pen in recs:
        why = ms.ignored(a, b, ba, bb, pen)
        if why is None:
            pairs.append(dict(parts=[a, b], bodies=[ba, bb], mm3=pen))
        elif why == "joint":
            tol.append(dict(parts=[a, b], bodies=[ba, bb], mm3=pen))
    pairs.sort(key=lambda r: -r["mm3"])
    zs = [z for z, _ in minz.values()]
    return {"pen_mm3": pairs[0]["mm3"] if pairs else 0.0, "pairs": pairs, "joint_tolerated": tol,
            "min_mesh_z_mm": 1000.0 * min(zs) if zs else None}


def check_trajectory(qpos, stride=5, ms=None, limit=LIMIT_MM3):
    """qpos (T, nq) yorungesi: her stride'inci kare (+ son kare) denetlenir. Dondurur:
    {"frames", "max_pen_mm3", "worst": {frame, parts, bodies, mm3} | None, "pairs": {"a~b": en buyuk mm3},
     "joint_tolerated": {"a~b": en buyuk yok sayilan mm3}, "min_mesh_z_mm", "min_nonfoot_mesh_z_mm",
     "baseline_pairs", "joint_tol_voxels", "joint_tol_mm3", "pass"}."""
    ms = ms or mesh_set()
    q = np.asarray(qpos, dtype=np.float64)
    if q.ndim != 2 or q.shape[1] != ms.m.nq:
        raise ValueError("qpos (T, %d) olmali, verilen %s" % (ms.m.nq, q.shape))
    frames = list(range(0, len(q), max(1, int(stride))))
    if frames and frames[-1] != len(q) - 1:
        frames.append(len(q) - 1)
    pairs, tol, worst, zmin, znf = {}, {}, None, np.inf, np.inf
    for f in frames:
        recs, minz = ms.pose(q[f])
        for a, b, ba, bb, pen in recs:
            why = ms.ignored(a, b, ba, bb, pen)
            k = "%s~%s" % tuple(sorted((a, b)))
            if why == "joint":
                tol[k] = max(tol.get(k, 0.0), pen)
            if why is not None:
                continue
            pairs[k] = max(pairs.get(k, 0.0), pen)
            if worst is None or pen > worst["mm3"]:
                worst = {"frame": f, "parts": [a, b], "bodies": [ba, bb], "mm3": pen}
        for body, (z, _) in minz.items():
            zmin = min(zmin, z)
            if not body.endswith("foot"):
                znf = min(znf, z)
    mx = worst["mm3"] if worst else 0.0
    out = {"frames": len(frames), "max_pen_mm3": mx, "worst": worst,
           "pairs": dict(sorted(pairs.items(), key=lambda kv: -kv[1])),
           "joint_tolerated": dict(sorted(tol.items(), key=lambda kv: -kv[1])),
           "min_mesh_z_mm": 1000.0 * zmin if np.isfinite(zmin) else None,
           "min_nonfoot_mesh_z_mm": 1000.0 * znf if np.isfinite(znf) else None}
    out.update(ms.info())
    out.update(limit_mm3=limit, **{"pass": bool(mx <= limit)})
    return out


def _load_traj(path):
    if path.endswith(".npz"):
        with np.load(path) as z:
            return z["qpos"]
    return np.load(path)


def main(argv=None):
    p = argparse.ArgumentParser(description="Gorsel mesh oz-carpisma denetimi (qpos yorungesi)")
    p.add_argument("traj", help="(T, nq) qpos .npy ya da qpos anahtarli .npz")
    p.add_argument("--stride", type=int, default=5, help="her kacinci kare (varsayilan 5 = 0.1 s)")
    p.add_argument("--pitch", type=float, default=PITCH, help="voxel araligi (m)")
    p.add_argument("--limit", type=float, default=LIMIT_MM3, help="kabul siniri (mm^3)")
    p.add_argument("--json", default=None, help="sonucu JSON olarak yaz")
    a = p.parse_args(argv)
    r = check_trajectory(_load_traj(a.traj), a.stride, mesh_set(a.pitch), a.limit)
    print("%d kare, taban cizgisi %d cift (HOME), eklem toleransi <= %d voxel (%.3f mm^3), en buyuk girisim %.2f mm^3%s; "
          "en alcak mesh %.1f mm (ayak disi %.1f mm) -> %s" % (
              r["frames"], r["baseline_pairs"], r["joint_tol_voxels"], r["joint_tol_mm3"], r["max_pen_mm3"],
              (" (%s, kare %d)" % (" ~ ".join(r["worst"]["parts"]), r["worst"]["frame"])) if r["worst"] else "",
              r["min_mesh_z_mm"], r["min_nonfoot_mesh_z_mm"], "GECTI" if r["pass"] else "GECMEDI"))
    for k, v in list(r["pairs"].items())[:10]:
        print("  %-48s %.2f mm^3" % (k, v))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(r, f, indent=1)
    return 0 if r["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
