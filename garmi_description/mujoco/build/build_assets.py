#!/usr/bin/env python3
# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0
"""Regenerate the MuJoCo assets for the Garmi model.

This is a *build-time* helper, not a runtime dependency. The assets it produces
are committed to the repository, so end users only need the committed files.

It does three things:
  1. Downloads the Franka FR3 arm and Franka Hand meshes from mujoco_menagerie
     (pinned revision), which are reused verbatim for Garmi's two arms.
  2. Copies Garmi's own mobile-base / lift / wheel meshes (STL) from the URDF
     package's meshes/ directory.
  3. Runs obj2mjcf on the TUM-designed multi-material parts (torso, head, neck,
     covers) to split them by material and convert textures to PNG, which is
     what MuJoCo needs (one material per mesh, PNG textures only).

Usage:
    python build_assets.py            # writes ../assets
"""

import os
import shutil
import subprocess
import sys
import tempfile
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
MUJOCO_DIR = os.path.dirname(HERE)
ASSETS = os.path.join(MUJOCO_DIR, "assets")
URDF_MESHES = os.path.normpath(os.path.join(MUJOCO_DIR, "..", "meshes"))

# Pinned mujoco_menagerie revision for reproducibility.
MENAGERIE_SHA = "accb6df40a9a1d1e49eff88157f6818b63a49335"
RAW = f"https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/{MENAGERIE_SHA}"

# FR3 arm meshes (visual .obj split by obj2mjcf upstream, collision .stl).
# link0 (the base) is NOT taken from the FR3 -- its link0 has a brake-release
# tool and branded collar that the Garmi base cover hides. We use the cleaner
# Panda/FER link0 instead (see PANDA_FILES below), which matches the Gazebo mesh.
FR3_FILES = (
    ["link1.obj", "link2.obj"]
    + [f"link3_{i}.obj" for i in range(2)]
    + [f"link4_{i}.obj" for i in range(2)]
    + [f"link5_{i}.obj" for i in range(3)]
    + [f"link6_{i}.obj" for i in range(8)]
    + [f"link7_{i}.obj" for i in range(4)]
    + [f"link{i}.stl" for i in range(1, 8)]  # collision meshes for links 1-7
)
# Franka Hand meshes + the Panda/FER link0 base (both in the panda model dir).
PANDA_FILES = (
    [f"hand_{i}.obj" for i in range(5)] + ["finger_0.obj", "finger_1.obj", "hand.stl"]
    + [f"link0_{i}.obj" for i in (0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11)]
    + ["link0.stl"]
)

# Garmi's own meshes (mobile base, wheels, lift). STL loads into MuJoCo as-is.
BASE_STL = [
    "body.stl", "body-collision.stl", "side-cover.stl", "end-cover.stl",
    "lights.stl", "axle.stl", "rocker.stl", "mecanum.stl", "hokuyo_ust.stl",
    "plate.stl", "base.stl", "lower.stl", "upper.stl",
]
# TUM multi-material parts -> split by material with obj2mjcf.
TUM_OBJ = ["body.obj", "head.obj", "cover.obj", "mounting_plate.obj", "neck_1.obj", "neck_2.obj"]


def fetch(rel_url, dest):
    urllib.request.urlretrieve(f"{RAW}/{rel_url}", dest)


def main():
    os.makedirs(ASSETS, exist_ok=True)

    print("[1/3] Downloading menagerie FR3 arm + Panda (hand, link0) meshes ...")
    for f in FR3_FILES:
        fetch(f"franka_fr3/assets/{f}", os.path.join(ASSETS, f))
    for f in PANDA_FILES:
        fetch(f"franka_emika_panda/assets/{f}", os.path.join(ASSETS, f))

    print("[2/3] Copying Garmi base/lift/wheel STL meshes ...")
    for f in BASE_STL:
        shutil.copy2(os.path.join(URDF_MESHES, f), os.path.join(ASSETS, f))

    print("[3/3] Splitting TUM multi-material parts with obj2mjcf ...")
    with tempfile.TemporaryDirectory() as tmp:
        for obj in TUM_OBJ:
            shutil.copy2(os.path.join(URDF_MESHES, obj), os.path.join(tmp, obj))
            mtl = obj.replace(".obj", ".mtl")
            if os.path.exists(os.path.join(URDF_MESHES, mtl)):
                shutil.copy2(os.path.join(URDF_MESHES, mtl), os.path.join(tmp, mtl))
        # Textures referenced by the MTLs.
        for tex in ("fabric.jpg", "face.jpg"):
            src = os.path.join(URDF_MESHES, tex)
            if os.path.exists(src):
                shutil.copy2(src, os.path.join(tmp, tex))
        obj2mjcf = shutil.which("obj2mjcf") or os.path.join(os.path.dirname(sys.executable), "obj2mjcf")
        subprocess.run(
            [obj2mjcf, "--obj-dir", tmp, "--save-mjcf", "--overwrite"],
            check=True,
        )
        # obj2mjcf writes one folder per OBJ (split submeshes + PNG + an MJCF
        # snippet). Move those folders into assets/ for reference and use.
        for obj in TUM_OBJ:
            name = obj[:-4]
            out = os.path.join(tmp, name)
            if os.path.isdir(out):
                dst = os.path.join(ASSETS, name)
                if os.path.exists(dst):
                    shutil.rmtree(dst)
                shutil.move(out, dst)

    # Tidy the obj2mjcf output: drop the example MJCF snippets, the source
    # JPEGs (we keep the PNGs MuJoCo needs) and the MTLs, and strip the now
    # dangling material references from the split OBJs (materials live in
    # garmi.xml, not the OBJ files).
    for root, _, files in os.walk(ASSETS):
        for fn in files:
            if fn.endswith((".jpeg", ".jpg", ".xml")) or fn == "material.mtl":
                os.remove(os.path.join(root, fn))
            elif fn.endswith(".obj"):
                p = os.path.join(root, fn)
                lines = [ln for ln in open(p) if not ln.startswith(("mtllib ", "usemtl "))]
                open(p, "w").writelines(lines)

    print(f"Done. Assets written to {ASSETS}")


if __name__ == "__main__":
    main()
