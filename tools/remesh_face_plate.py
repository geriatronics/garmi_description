#!/usr/bin/env python3
# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0
"""Re-triangulate the flat faces of the head's face plate in ``head.obj``.

Blender exports the face plate (the ``Face`` material group of ``head.obj``)
as two flat n-gons -- front and back -- each fan-triangulated from the rim into
~165 sliver triangles up to 20 cm long. Renderers that light per vertex
(Gouraud shading: RViz's fixed-function path, MuJoCo's classic renderer)
evaluate the specular highlight only at the rim vertices and interpolate it
across those slivers, which shows up as triangle-shaped stripes whenever a
light reflects off the plate.

This script replaces the front and back triangulations with a fine, regular
one (Delaunay over a ~6 mm hexagonal grid) that keeps the original outline
vertices and rim untouched, so the plate stays watertight and the texture
mapping (which is an affine projection onto the plate) is preserved exactly.
Everything else in the OBJ is copied through verbatim.

Build-time helper, not a runtime dependency: the result is committed. Re-run
it after re-exporting ``head.obj`` from Blender, then regenerate the MuJoCo
assets with ``garmi_description/mujoco/build/build_assets.py``.

Usage:
    python tools/remesh_face_plate.py [--edge 0.006] [--out other.obj]

Requires ``numpy`` and ``scipy`` (``pip install numpy scipy``).
"""

import argparse
import os
from collections import Counter, defaultdict

import numpy as np
from scipy.spatial import Delaunay

HERE = os.path.dirname(os.path.abspath(__file__))
HEAD_OBJ = os.path.normpath(os.path.join(HERE, "..", "garmi_description", "meshes", "head.obj"))
MATERIAL = "Face"
COPLANAR = 0.999  # cos of the angle below which a face counts as front/back


def parse_face(tokens):
    """Parse ``f`` tokens into (v, vt, vn) zero-based index triples (-1 if absent)."""
    out = []
    for tok in tokens:
        parts = [*tok.split("/"), "", ""][:3]
        out.append(tuple(int(p) - 1 if p else -1 for p in parts))
    return out


def parse_obj(lines):
    """Return (vertices, uvs, faces); faces are (line index, material, corners)."""
    verts, uvs, faces = [], [], []
    material = None
    for i, ln in enumerate(lines):
        p = ln.split()
        if not p:
            continue
        if p[0] == "v":
            verts.append([float(x) for x in p[1:4]])
        elif p[0] == "vt":
            uvs.append([float(x) for x in p[1:3]])
        elif p[0] == "usemtl":
            material = p[1]
        elif p[0] == "f":
            faces.append((i, material, parse_face(p[1:])))
    return np.array(verts), np.array(uvs), faces


def outline_loop(tris):
    """Return the ordered boundary vertex loop of a triangulated simple polygon."""
    edges = Counter()
    for t in tris:
        for i in range(3):
            edges[tuple(sorted((t[i], t[(i + 1) % 3])))] += 1
    adj = defaultdict(list)
    for (a, b), n in edges.items():
        if n == 1:
            adj[a].append(b)
            adj[b].append(a)
    if any(len(v) != 2 for v in adj.values()):
        raise RuntimeError("plate outline is not a single simple loop")
    start = next(iter(adj))
    loop, prev = [start], None
    while True:
        nxt = next(x for x in adj[loop[-1]] if x != prev)
        prev = loop[-1]
        if nxt == start:
            break
        loop.append(nxt)
    return loop


def point_in_polygon(points, poly):
    """Even-odd ray-casting test of ``points`` (n, 2) against ``poly`` (m, 2)."""
    x, y = points[:, 0], points[:, 1]
    inside = np.zeros(len(points), dtype=bool)
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        crosses = (yi > y) != (yj > y)
        xint = (xj - xi) * (y - yi) / (yj - yi + 1e-300) + xi
        inside ^= crosses & (x < xint)
        j = i
    return inside


def dist_to_polygon(points, poly):
    """Distance from each of ``points`` (n, 2) to the closest edge of ``poly``."""
    best = np.full(len(points), np.inf)
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        ab = b - a
        t = np.clip(((points - a) @ ab) / (ab @ ab), 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(points - (a + t[:, None] * ab), axis=1))
    return best


def hex_grid_inside(poly, spacing):
    """Hexagonal grid of points inside ``poly``, at least spacing/2 from its edge."""
    lo, hi = poly.min(axis=0), poly.max(axis=0)
    dy = spacing * np.sqrt(3) / 2
    rows = []
    for k, y in enumerate(np.arange(lo[1], hi[1] + dy, dy)):
        xs = np.arange(lo[0] + (spacing / 2 if k % 2 else 0.0), hi[0] + spacing, spacing)
        rows.append(np.c_[xs, np.full(len(xs), y)])
    pts = np.vstack(rows)
    keep = point_in_polygon(pts, poly) & (dist_to_polygon(pts, poly) >= 0.5 * spacing)
    return pts[keep]


def triangulate(xy, max_edge):
    """Triangulate the polygon ``xy`` keeping its edges; return (points, triangles).

    Fills the outline with a hexagonal grid of interior points (kept at least
    half a spacing from the rim so no slivers form there) and runs Delaunay on
    outline + interior. The outline is convex, so its edges are hull edges and
    survive; both properties are verified.
    """
    pts = np.vstack([xy, hex_grid_inside(xy, max_edge)])
    tris = Delaunay(pts).simplices
    tris = tris[point_in_polygon(pts[tris].mean(axis=1), xy)]
    edges = {tuple(sorted(e)) for t in tris for e in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0]))}
    n = len(xy)
    missing = [i for i in range(n) if tuple(sorted((i, (i + 1) % n))) not in edges]
    if missing:
        raise RuntimeError(f"{len(missing)} outline edges are not in the triangulation")
    return pts, tris


def geom_normal(verts, corners):
    """Compute the unit normal of a face from its first three corners."""
    a, b, c = (verts[k[0]] for k in corners[:3])
    n = np.cross(b - a, c - a)
    return n / np.linalg.norm(n)


def split_plate(verts, plate):
    """Split the plate faces into (normal, front, back, rim) by their orientation."""
    areas = [
        np.linalg.norm(
            np.cross(verts[f[2][1][0]] - verts[f[2][0][0]], verts[f[2][2][0]] - verts[f[2][0][0]])
        )
        for f in plate
    ]
    ref = geom_normal(verts, plate[int(np.argmax(areas))][2])
    front, back, rim = [], [], []
    for f in plate:
        d = geom_normal(verts, f[2]) @ ref
        (front if d > COPLANAR else back if d < -COPLANAR else rim).append(f)
    if any(len(f[2]) != 3 for f in front + back):
        raise RuntimeError("front/back face of the plate is not triangulated")
    return ref, front, back, rim


def plane_basis(ref):
    """Two orthonormal in-plane axes for the plane with normal ``ref``."""
    b1 = np.cross(ref, [1.0, 0.0, 0.0])
    if np.linalg.norm(b1) < 0.5:
        b1 = np.cross(ref, [0.0, 1.0, 0.0])
    b1 /= np.linalg.norm(b1)
    return b1, np.cross(ref, b1)


def match_outlines(verts, basis, loop, back_ids):
    """Map each front outline vertex to the back vertex at the same in-plane spot."""
    back_xy = verts[back_ids] @ basis
    mapping = {}
    for vid in loop:
        d = np.linalg.norm(back_xy - verts[vid] @ basis, axis=1)
        j = int(np.argmin(d))
        if d[j] > 1e-5:
            raise RuntimeError("front and back outlines do not match")
        mapping[vid] = back_ids[j]
    if len(set(mapping.values())) != len(loop):
        raise RuntimeError("front/back outline correspondence is not one-to-one")
    return mapping


def corner_maps(faces):
    """Per-vertex vt index and the (single) vn index used by ``faces``."""
    vt = {c[0]: c[1] for f in faces for c in f[2]}
    vn = Counter(c[2] for f in faces for c in f[2]).most_common(1)[0][0]
    return vt, vn


def fit_uv(verts, uvs, basis, vt_map):
    """Fit UV = affine(in-plane xy); return (3x2 matrix, max residual)."""
    ids = sorted(vt_map)
    xy1 = np.c_[verts[ids] @ basis, np.ones(len(ids))]
    uv = np.array([uvs[vt_map[i]] for i in ids])
    aff, *_ = np.linalg.lstsq(xy1, uv, rcond=None)
    return aff, float(np.abs(xy1 @ aff - uv).max())


def splice_points(lines, plate, rim, drop):
    """Return (line to insert new v/vt before, line after which new faces go)."""
    insert_at = min(f[0] for f in plate)
    while insert_at > 0 and lines[insert_at - 1].split()[:1] in (["s"], ["usemtl"]):
        insert_at -= 1
    last_rim_line = max(f[0] for f in rim) if rim else max(drop)
    return insert_at, last_rim_line


def assemble(lines, where, drop, new_lines):
    """Splice new (v, vt, f) lines into ``lines``, dropping the faces in ``drop``."""
    insert_at, last_rim_line = where
    new_v, new_vt, new_faces = new_lines
    out = []
    for i, ln in enumerate(lines):
        if i == insert_at:
            out.extend(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n" for p in new_v)
            out.extend(f"vt {t[0]:.6f} {t[1]:.6f}\n" for t in new_vt)
        if i in drop:
            continue
        out.append(ln)
        if i == last_rim_line:
            out.extend(f + "\n" for f in new_faces)
    return out


def remesh(lines, max_edge):
    """Return the rewritten OBJ lines and a dict of statistics."""
    verts, uvs, faces = parse_obj(lines)
    plate = [f for f in faces if f[1] == MATERIAL]
    if not plate:
        raise RuntimeError(f"no faces with material {MATERIAL!r}")
    ref, front, back, rim = split_plate(verts, plate)
    b1, b2 = plane_basis(ref)
    basis = np.c_[b1, b2]

    # Outlines of both flat faces, and their vertex-to-vertex correspondence.
    loop = outline_loop([[c[0] for c in f[2]] for f in front])
    back_ids = sorted(outline_loop([[c[0] for c in f[2]] for f in back]))
    z_front = float(np.median(verts[loop] @ ref))
    z_back = float(np.median(verts[back_ids] @ ref))
    to_back = match_outlines(verts, basis, loop, back_ids)

    # Texture coordinates and normals the new faces inherit.
    vt_front, vn_front = corner_maps(front)
    vt_back, vn_back = corner_maps(back)
    aff_front, r_front = fit_uv(verts, uvs, basis, vt_front)
    aff_back, r_back = fit_uv(verts, uvs, basis, vt_back)
    if max(r_front, r_back) > 1e-4:
        raise RuntimeError("plate UVs are not an affine projection; cannot extend them")

    pts, tris = triangulate(verts[loop] @ basis, max_edge)
    n_loop, n_new = len(loop), len(pts) - len(loop)

    # New interior vertices: front plane first, then back plane.
    nv0, nvt0 = len(verts), len(uvs)
    new_v, new_vt = [], []
    for z, aff in ((z_front, aff_front), (z_back, aff_back)):
        for p in pts[n_loop:]:
            new_v.append(b1 * p[0] + b2 * p[1] + ref * z)
            new_vt.append(np.r_[p, 1.0] @ aff)

    def indices(local, side):
        """OBJ (v, vt) indices of triangulation vertex ``local`` on ``side``."""
        if local < n_loop:
            fv = loop[local]
            if side == "front":
                return fv, vt_front[fv]
            return to_back[fv], vt_back[to_back[fv]]
        k = local - n_loop + (0 if side == "front" else n_new)
        return nv0 + k, nvt0 + k

    # Winding: CCW in (b1, b2) gives a normal along +ref, i.e. the front side.
    ccw = (np.cross(b1, b2) @ ref) > 0
    p = pts[tris]
    cross = (p[:, 1, 0] - p[:, 0, 0]) * (p[:, 2, 1] - p[:, 0, 1]) - (p[:, 1, 1] - p[:, 0, 1]) * (
        p[:, 2, 0] - p[:, 0, 0]
    )
    new_faces = []
    for side, vn, want in (("front", vn_front, ccw), ("back", vn_back, not ccw)):
        for t, c in zip(tris, cross, strict=True):
            order = t if (c > 0) == want else t[[0, 2, 1]]
            corners = (indices(int(i), side) for i in order)
            new_faces.append("f " + " ".join(f"{v + 1}/{vt + 1}/{vn + 1}" for v, vt in corners))

    drop = {f[0] for f in front + back}
    where = splice_points(lines, plate, rim, drop)
    out = assemble(lines, where, drop, (new_v, new_vt, new_faces))
    stats = {
        "plate_normal": ref.round(4).tolist(),
        "thickness_m": round(z_front - z_back, 6),
        "outline_vertices": n_loop,
        "old_triangles_per_side": len(front),
        "new_triangles_per_side": len(tris),
        "new_vertices": 2 * n_new,
        "uv_fit_residual": max(r_front, r_back),
    }
    return out, stats


def main():
    """Rewrite head.obj with a finely triangulated face plate."""
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--obj", default=HEAD_OBJ, help="OBJ to rewrite (default: meshes/head.obj)")
    ap.add_argument("--edge", type=float, default=0.006, help="target triangle edge [m]")
    ap.add_argument("--out", default=None, help="output path (default: overwrite --obj)")
    args = ap.parse_args()

    with open(args.obj) as fh:
        lines = fh.readlines()
    out, stats = remesh(lines, args.edge)
    dst = args.out or args.obj
    with open(dst, "w") as fh:
        fh.writelines(out)
    for k, v in stats.items():
        print(f"{k}: {v}")
    print(f"wrote {dst}")


if __name__ == "__main__":
    main()
