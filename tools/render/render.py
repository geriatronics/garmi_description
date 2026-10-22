#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13,<3.14"
# dependencies = [
#     "bpy>=5.2,<5.3",
#     "numpy",
#     "pycollada>=0.8",
#     "trimesh>=4",
# ]
# ///
# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0
"""Render Garmi from its URDF with Blender's Cycles path tracer.

The script reads the plain URDF, poses it with its own forward kinematics, loads every visual
mesh (Collada, OBJ with its MTL and textures, STL), gives each surface a physically based
material derived from the model's own colours, and renders a studio shot. Blender comes in as
the `bpy` Python module, so `uv` is the only thing to install:

    uv run tools/render/render.py --view front-left --transparent -o garmi.png

See tools/render/README.md for the options, the presets and the material rules.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, replace
from pathlib import Path

import bpy
import collada
import numpy as np
import trimesh
from mathutils import Matrix, Vector

REPOSITORY = Path(__file__).resolve().parents[2]
DEFAULT_URDF = REPOSITORY / "garmi_description" / "urdf" / "garmi.urdf"
CACHE = Path.home() / ".cache" / "garmi_description" / "render"


class UsageError(Exception):
    """A request the model or the options cannot satisfy, reported as a command-line error."""


# =================================================================================================
# The model: URDF parsing and forward kinematics
# =================================================================================================


@dataclass(frozen=True)
class Visual:
    """One <visual> of a link: its geometry, pose in the link frame and URDF colour."""

    origin: np.ndarray
    kind: str  # mesh | box
    mesh: Path | None = None
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    size: tuple[float, ...] = ()
    material: str | None = None
    rgba: tuple[float, float, float, float] | None = None


@dataclass(frozen=True)
class Joint:
    """One <joint>: how its child link moves relative to its parent."""

    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float | None = None
    upper: float | None = None
    mimic: tuple[str, float, float] | None = None  # (joint, multiplier, offset)


@dataclass
class Robot:
    """The links' visuals and the joint tree of one URDF."""

    visuals: dict[str, list[Visual]]
    joints: dict[str, Joint]
    root: str = field(init=False)

    def __post_init__(self) -> None:
        """Find the root: the link that is no joint's child."""
        children = {j.child for j in self.joints.values()}
        self.root = next(link for link in self.visuals if link not in children)

    def movable(self) -> dict[str, Joint]:
        """Return the joints a pose may set: the moving ones that mimic no other joint."""
        return {
            n: j
            for n, j in self.joints.items()
            if j.kind in ("revolute", "continuous", "prismatic") and j.mimic is None
        }

    def forward_kinematics(self, positions: dict[str, float]) -> dict[str, np.ndarray]:
        """Return every link's 4x4 pose in the root frame for the given joint positions."""
        by_parent: dict[str, list[Joint]] = {}
        for joint in self.joints.values():
            by_parent.setdefault(joint.parent, []).append(joint)
        poses = {self.root: np.eye(4)}
        stack = [self.root]
        while stack:
            parent = stack.pop()
            for joint in by_parent.get(parent, []):
                poses[joint.child] = poses[parent] @ joint.origin @ joint_motion(joint, positions)
                stack.append(joint.child)
        return poses


def joint_motion(joint: Joint, positions: dict[str, float]) -> np.ndarray:
    """Return the transform a joint adds at its position; fixed and other kinds add none."""
    if joint.mimic is not None:
        source, multiplier, offset = joint.mimic
        value = multiplier * positions.get(source, 0.0) + offset
    else:
        value = positions.get(joint.name, 0.0)
    motion = np.eye(4)
    if joint.kind in ("revolute", "continuous"):
        motion[:3, :3] = axis_angle(joint.axis, value)
    elif joint.kind == "prismatic":
        motion[:3, 3] = joint.axis * value
    return motion


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Return the rotation matrix of `angle` radians about the unit vector `axis`."""
    x, y, z = axis
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def origin_matrix(element: ET.Element | None) -> np.ndarray:
    """Return the 4x4 transform of an <origin xyz rpy> element (identity when absent)."""
    matrix = np.eye(4)
    if element is None:
        return matrix
    roll, pitch, yaw = floats(element.get("rpy", "0 0 0"))
    matrix[:3, :3] = (
        axis_angle(np.array([0.0, 0.0, 1.0]), yaw)
        @ axis_angle(np.array([0.0, 1.0, 0.0]), pitch)
        @ axis_angle(np.array([1.0, 0.0, 0.0]), roll)
    )
    matrix[:3, 3] = floats(element.get("xyz", "0 0 0"))
    return matrix


def floats(text: str) -> tuple[float, ...]:
    """Parse a whitespace-separated list of numbers."""
    return tuple(float(v) for v in text.split())


def resolve_uri(uri: str, urdf: Path) -> Path:
    """Resolve a package:// mesh URI against the package.xml of that name above the URDF."""
    package, _, rest = uri.removeprefix("package://").partition("/")
    for directory in urdf.resolve().parents:
        manifest = directory / "package.xml"
        if manifest.is_file() and ET.parse(manifest).findtext("name") == package:
            return directory / rest
    raise UsageError(f"{uri}: no package.xml naming '{package}' above {urdf}")


def load_urdf(path: Path) -> Robot:
    """Read a plain (xacro-expanded) URDF into its visuals and joint tree."""
    root = ET.parse(path).getroot()
    colours = {
        m.get("name"): floats(m.find("color").get("rgba"))
        for m in root.findall("material")
        if m.find("color") is not None
    }
    visuals: dict[str, list[Visual]] = {}
    for link in root.findall("link"):
        visuals[link.get("name")] = [parse_visual(v, colours, path) for v in link.findall("visual")]
    joints = {}
    for j in root.findall("joint"):
        limit, mimic = j.find("limit"), j.find("mimic")
        axis = np.array(
            floats(j.find("axis").get("xyz")) if j.find("axis") is not None else (1, 0, 0)
        )
        joints[j.get("name")] = Joint(
            name=j.get("name"),
            kind=j.get("type"),
            parent=j.find("parent").get("link"),
            child=j.find("child").get("link"),
            origin=origin_matrix(j.find("origin")),
            axis=axis / np.linalg.norm(axis),
            lower=float(limit.get("lower"))
            if limit is not None and "lower" in limit.attrib
            else None,
            upper=float(limit.get("upper"))
            if limit is not None and "upper" in limit.attrib
            else None,
            mimic=(
                mimic.get("joint"),
                float(mimic.get("multiplier", 1.0)),
                float(mimic.get("offset", 0.0)),
            )
            if mimic is not None
            else None,
        )
    return Robot(visuals, joints)


def parse_visual(element: ET.Element, colours: dict, urdf: Path) -> Visual:
    """Read one <visual>; an inline <color> overrides the named material's."""
    geometry = element.find("geometry")[0]
    material = element.find("material")
    name = rgba = None
    if material is not None:
        name = material.get("name")
        inline = material.find("color")
        rgba = floats(inline.get("rgba")) if inline is not None else colours.get(name)
    visual = Visual(origin=origin_matrix(element.find("origin")), kind=geometry.tag)
    visual = replace(visual, material=name, rgba=rgba)
    if geometry.tag == "mesh":
        scale = floats(geometry.get("scale", "1 1 1"))
        return replace(visual, mesh=resolve_uri(geometry.get("filename"), urdf), scale=scale)
    if geometry.tag == "box":
        return replace(visual, size=floats(geometry.get("size")))
    raise UsageError(f"unsupported visual geometry <{geometry.tag}>: only mesh and box are read")


# =================================================================================================
# Joint poses
# =================================================================================================

# Franka's ready pose, joints 1-7: the arms' start pose and the MuJoCo model's `home` keyframe.
ARM_READY = (0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4)
# Garmi's driving posture, left arm: folded back over the shoulder, above the base.
ARM_TRAVEL = (-2.32, 1.17, 0.33, -0.68, 2.63, 3.49, -1.5)
# The README image's pose, seen from the front (--camera azimuth=0): the left hand raised and
# tilted 20 degrees down, its wrist camera looking at the viewer above the fingertips; the right
# gripper held out sideways, its length in view.
README_LEFT = (-1.229, -1.74, -0.699, -2.605, 1.461, 2.105, 0.727)
README_RIGHT = (-0.765, -1.401, -1.03, -2.55, -0.106, 4.348, -1.361)


def arm(side: str, values: tuple[float, ...]) -> dict[str, float]:
    """Name one arm's seven joint values."""
    return {f"{side}_fr3_joint{i}": v for i, v in enumerate(values, start=1)}


def mirrored(values: tuple[float, ...]) -> tuple[float, ...]:
    """Mirror a left-arm pose to the right arm: joints 1, 3, 5 and 7 change sign."""
    return tuple(-v if i % 2 == 0 else v for i, v in enumerate(values))


POSES = {
    "zero": {},
    "manipulation": {
        **arm("left", ARM_READY),
        **arm("right", ARM_READY),
        "left_fr3_finger_joint1": 0.04,
        "right_fr3_finger_joint1": 0.04,
    },
    "travel": {**arm("left", ARM_TRAVEL), **arm("right", mirrored(ARM_TRAVEL))},
    "readme": {
        **arm("left", README_LEFT),
        **arm("right", README_RIGHT),
        "left_fr3_finger_joint1": 0.04,
        "right_fr3_finger_joint1": 0.04,
    },
}


def joint_positions(robot: Robot, pose: str, overrides: list[str]) -> dict[str, float]:
    """Return the preset's joint values with `name=value` overrides applied and checked."""
    positions = dict(POSES[pose])
    movable = robot.movable()
    missing = sorted(set(positions) - set(movable))
    if missing:
        raise UsageError(f"--pose {pose}: the URDF lacks the movable joint(s) {', '.join(missing)}")
    for item in overrides:
        name, _, value = item.partition("=")
        if name not in movable:
            raise UsageError(f"--joint {item}: '{name}' is not a movable, non-mimic joint")
        try:
            positions[name] = float(value)
        except ValueError:
            raise UsageError(
                f"--joint {item}: '{value}' is not a number (radians or metres)"
            ) from None
    for name, value in positions.items():
        joint = movable[name]
        if joint.lower is not None and not joint.lower <= value <= joint.upper:
            print(
                f"warning: {name}={value:g} lies outside its limits [{joint.lower}, {joint.upper}]"
            )
    return positions


# =================================================================================================
# Meshes and their surfaces, read from the files
# =================================================================================================


@dataclass(frozen=True)
class Surface:
    """A surface as its source file describes it: linear colour, roughness if stated, texture."""

    name: str
    color: tuple[float, float, float]
    roughness: float | None = None
    texture: Path | None = None


@dataclass(frozen=True)
class Part:
    """A triangle mesh in the file's frame with one surface (None: the URDF colour applies)."""

    vertices: np.ndarray
    faces: np.ndarray
    surface: Surface | None
    uv: np.ndarray | None = None


def srgb_to_linear(value: float) -> float:
    """Convert one sRGB-encoded channel (as URDF and RViz colours are) to linear light."""
    return value / 12.92 if value <= 0.04045 else ((value + 0.055) / 1.055) ** 2.4


def box(size: tuple[float, ...]) -> Part:
    """Return a URDF <box> of the given edge lengths, centred on its origin."""
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) / 2.0
    quads = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
    faces = [tri for a, b, c, d in quads for tri in ((a, b, c), (a, c, d))]
    return Part(corners * np.array(size), np.array(faces), None)


def load_mesh(path: Path) -> list[Part]:
    """Read a mesh file into parts, one per surface."""
    suffix = path.suffix.lower()
    if suffix == ".dae":
        return load_collada(path)
    if suffix == ".obj":
        return load_obj(path)
    if suffix == ".stl":
        mesh = trimesh.load(path, force="mesh", process=False)
        return [Part(np.asarray(mesh.vertices), np.asarray(mesh.faces), None)]
    raise UsageError(f"{path}: unsupported mesh format '{suffix}'")


def load_collada(path: Path) -> list[Part]:
    """Read a Collada file; its effect colours are linear, as Blender writes and reads them."""
    parts = []
    for geometry in collada.Collada(str(path)).scene.objects("geometry"):
        for primitive in geometry.primitives():
            effect = primitive.material.effect
            # A Phong exponent maps to roughness as sqrt(2 / (n + 2)); zero or absent states none.
            shininess = effect.shininess or 0.0
            surface = Surface(
                name=primitive.material.name,
                color=tuple(effect.diffuse[:3]),
                roughness=math.sqrt(2.0 / (shininess + 2.0)) if shininess > 0 else None,
            )
            parts.append(Part(primitive.vertex, primitive.vertex_index, surface))
    return parts


def load_obj(path: Path) -> list[Part]:
    """Read an OBJ and its MTL; the MTL is Blender's, so Kd is linear and Ns encodes roughness."""
    surfaces = read_mtl(path)
    scene = trimesh.load(path, force="scene", process=False, group_material=True)
    parts = []
    for mesh in scene.geometry.values():
        material = getattr(mesh.visual, "material", None)
        surface = surfaces.get(getattr(material, "name", None))
        uv = getattr(mesh.visual, "uv", None)
        if surface is None or surface.texture is None:
            uv = None
        parts.append(Part(np.asarray(mesh.vertices), np.asarray(mesh.faces), surface, uv))
    return parts


def read_mtl(obj: Path) -> dict[str, Surface]:
    """Read the materials an OBJ's mtllib names; Blender writes Ns = 1000 (1 - roughness)^2."""
    surfaces: dict[str, Surface] = {}
    library = next(
        line.split(maxsplit=1)[1].strip() for line in obj.open() if line.startswith("mtllib")
    )
    current = None
    for line in (obj.parent / library).read_text().splitlines():
        key, _, value = line.strip().partition(" ")
        if key == "newmtl":
            current = value.strip()
            surfaces[current] = Surface(current, (0.8, 0.8, 0.8))
        elif current is None:
            continue
        elif key == "Kd":
            surfaces[current] = replace(surfaces[current], color=floats(value)[:3])
        elif key == "Ns":
            shininess = min(max(float(value), 0.0), 1000.0)
            roughness = 1.0 - math.sqrt(shininess / 1000.0)
            surfaces[current] = replace(surfaces[current], roughness=roughness)
        elif key == "map_Kd":
            texture = obj.parent / value.split()[-1]
            surfaces[current] = replace(surfaces[current], texture=texture)
    return surfaces


# =================================================================================================
# Physically based finishes per material class
# =================================================================================================


@dataclass(frozen=True)
class Finish:
    """How a class of surface reflects; a None roughness defers to the source file's."""

    roughness: float | None = None
    metallic: float = 0.0
    coat: float = 0.0
    sheen: float = 0.0
    specular: float = 0.5  # Principled's specular IOR level; 0.5 is plain dielectric
    emission: float = 0.0  # a display: its texture glows at this strength


WHITEST = 0.80  # linear albedo of bright white paint; a model's 1.0 renders as this

# The first pattern matching the whole lower-cased material name wins; colours stay the model's.
FINISHES = (
    (r"glass", Finish(roughness=0.04, coat=1.0)),  # the RealSense's black glass front
    (r"screen", Finish(roughness=0.12, specular=0.2)),  # the chest's display
    (r"face", Finish(roughness=0.12, specular=0.2, emission=1.6)),  # the head's face display
    (r"opening", Finish(roughness=0.08, coat=1.0)),  # the RealSense's dark lenses
    (r"aluminium|clearpath_aluminum", Finish(roughness=0.32, metallic=1.0)),  # metal housings
    (r"fabric", Finish(roughness=0.9, sheen=0.6)),  # the torso's knitted cover
    (r"fingertip_black", Finish(roughness=0.75)),  # rubber fingertip pads
    (r"(wrist_camera_mount|olive_camera)_white", Finish(roughness=0.65)),  # 3D-printed parts
    (r"olive_camera_dark", Finish(roughness=0.45)),  # the head camera's dark plastic
    (r"(white|grey)_plastic", Finish()),  # the design's plastics: their MTL states roughness
    (r"clearpath_.*|lightgrey", Finish(roughness=0.5)),  # the base's and lift's paint
    (r".*", Finish(roughness=0.38, coat=0.15)),  # Franka's painted shells and anything unnamed
)


def finish_for(name: str) -> Finish:
    """Return the finish of the first class whose pattern matches the material name."""
    return next(f for pattern, f in FINISHES if re.fullmatch(pattern, name.lower()))


# =================================================================================================
# The Blender scene
# =================================================================================================


@dataclass(frozen=True)
class Camera:
    """A viewpoint: angles in degrees from the direction the facing frame's axis points."""

    targets: tuple[str, ...]  # links to frame; empty frames the whole robot
    facing: tuple[str, int] | None  # (frame, axis index); None faces the robot's front, +x
    azimuth: float
    elevation: float
    lens: float  # focal length in mm on a 36 mm sensor
    resolution: tuple[int, int]
    margin: float = 1.06  # 1 frames the targets edge to edge


HEAD = ("head", "neck_2", "head_camera_mount", "olive_camera_link", "olive_camera_housing")
WRIST_CAMERA = ("left_fr3_wrist_camera_mount", "camera_d435i_left_link")

VIEWS = {
    "front-left": Camera((), None, 35.0, 12.0, 70.0, (1200, 1500)),
    "head": Camera(HEAD, ("head", 0), 30.0, 8.0, 100.0, (1600, 1200)),
    # The bracket and camera fill the middle; the margin leaves the hand around them as context.
    "wrist": Camera(
        WRIST_CAMERA,
        ("camera_d435i_left_color_optical_frame", 2),
        35.0,
        20.0,
        100.0,
        (1600, 1200),
        margin=1.4,
    ),
}


def camera_settings(view: str, overrides: list[str]) -> Camera:
    """Return a view preset with `key=value` overrides applied."""
    camera = VIEWS[view]
    for item in overrides:
        key, _, value = item.partition("=")
        if key == "target":
            camera = replace(camera, targets=tuple(v for v in value.split(",") if v))
        elif key in ("azimuth", "elevation", "lens", "margin"):
            try:
                camera = replace(camera, **{key: float(value)})
            except ValueError:
                raise UsageError(f"--camera {item}: '{value}' is not a number") from None
        else:
            raise UsageError(
                f"--camera {item}: unknown key '{key}' (azimuth, elevation, lens, margin, target)"
            )
    return camera


class Studio:
    """Builds the robot, its materials, lights, ground and camera in an empty Blender scene."""

    def __init__(self) -> None:
        """Start from Blender's empty factory scene with Cycles as the renderer."""
        bpy.ops.wm.read_factory_settings(use_empty=True)
        self.scene = bpy.context.scene
        self.scene.render.engine = "CYCLES"
        self.rig = bpy.data.objects.new("garmi", None)  # turntable pivot, holds every link
        self.scene.collection.objects.link(self.rig)
        self.meshes: dict[Path, list[tuple[object, Surface | None]]] = {}
        self.materials: dict[tuple, object] = {}
        self.objects: dict[str, list] = {}

    # -- robot --------------------------------------------------------------------------------

    def add_robot(self, robot: Robot, poses: dict[str, np.ndarray]) -> None:
        """Place every visual of every link at its forward-kinematics pose."""
        for link, visuals in robot.visuals.items():
            for v_index, visual in enumerate(visuals):
                scale = np.diag([*visual.scale, 1.0])
                world = poses[link] @ visual.origin @ scale
                for p_index, (mesh, surface) in enumerate(self.visual_meshes(visual)):
                    obj = bpy.data.objects.new(f"{link}.{v_index}.{p_index}", mesh)
                    obj.matrix_world = Matrix(world.tolist())
                    obj.parent = self.rig
                    slot = obj.material_slots[0]
                    slot.link = "OBJECT"
                    slot.material = self.material(surface, visual)
                    self.scene.collection.objects.link(obj)
                    self.objects.setdefault(link, []).append(obj)

    def visual_meshes(self, visual: Visual) -> list[tuple[object, Surface | None]]:
        """Return the Blender meshes of a visual, each file read and built once."""
        if visual.kind == "box":
            return [(self.build_mesh("box", box(visual.size)), None)]
        if visual.mesh not in self.meshes:
            self.meshes[visual.mesh] = [
                (self.build_mesh(visual.mesh.stem, part), part.surface)
                for part in load_mesh(visual.mesh)
            ]
        return self.meshes[visual.mesh]

    def build_mesh(self, name: str, part: Part) -> object:
        """Turn a triangle soup into a welded, smooth-shaded Blender mesh with sharp creases."""
        faces = np.asarray(part.faces, dtype=np.int64)
        # Files split vertices per face or per normal; weld them, then let the angle decide
        # which edges stay sharp. UVs live on face corners, so welding keeps the texture seams.
        grid = np.round(np.asarray(part.vertices, dtype=np.float64) / 1e-6).astype(np.int64)
        _, first, welded = np.unique(grid, axis=0, return_index=True, return_inverse=True)
        vertices = np.asarray(part.vertices, dtype=np.float32)[first]
        corners = welded.ravel()[faces].astype(np.int32)
        mesh = bpy.data.meshes.new(name)
        mesh.vertices.add(len(vertices))
        mesh.vertices.foreach_set("co", vertices.ravel())
        mesh.loops.add(corners.size)
        mesh.loops.foreach_set("vertex_index", corners.ravel())
        mesh.polygons.add(len(corners))
        mesh.polygons.foreach_set("loop_start", np.arange(0, corners.size, 3, dtype=np.int32))
        if part.uv is not None:
            layer = mesh.uv_layers.new(name="UVMap")
            layer.data.foreach_set("uv", np.asarray(part.uv, dtype=np.float32)[faces].ravel())
        mesh.update()
        mesh.validate()  # drops the triangles welding collapsed
        mesh.shade_smooth()
        mesh.set_sharp_from_angle(angle=math.radians(35.0))
        mesh.materials.append(None)
        return mesh

    def material(self, surface: Surface | None, visual: Visual) -> object:
        """Return the Principled BSDF material of a surface, or of the visual's URDF colour."""
        if surface is None:
            rgba = visual.rgba or (0.8, 0.8, 0.8, 1.0)
            name = visual.material or "default"
            surface = Surface(name, tuple(srgb_to_linear(c) for c in rgba[:3]))
        key = (surface.name, surface.color, surface.roughness, surface.texture)
        if key not in self.materials:
            self.materials[key] = self.build_material(surface, finish_for(surface.name))
        return self.materials[key]

    def build_material(self, surface: Surface, finish: Finish) -> object:
        """Build a Principled BSDF from the surface's colour and its class's finish."""
        material = bpy.data.materials.new(surface.name)
        nodes, links = material.node_tree.nodes, material.node_tree.links
        bsdf = nodes["Principled BSDF"]
        roughness = finish.roughness if finish.roughness is not None else surface.roughness
        # No paint or plastic reflects all light; files that say 1.0 mean "white", so cap it.
        albedo = tuple(min(c, WHITEST) for c in surface.color)
        bsdf.inputs["Base Color"].default_value = (*albedo, 1.0)
        bsdf.inputs["Roughness"].default_value = 0.4 if roughness is None else roughness
        bsdf.inputs["Metallic"].default_value = finish.metallic
        bsdf.inputs["Coat Weight"].default_value = finish.coat
        bsdf.inputs["Coat Roughness"].default_value = 0.03
        bsdf.inputs["Sheen Weight"].default_value = finish.sheen
        bsdf.inputs["Sheen Roughness"].default_value = 0.4
        bsdf.inputs["Specular IOR Level"].default_value = finish.specular
        if surface.texture is not None:
            texture = nodes.new("ShaderNodeTexImage")
            texture.image = bpy.data.images.load(str(surface.texture), check_existing=True)
            links.new(texture.outputs["Color"], bsdf.inputs["Base Color"])
            if finish.emission:  # a display shows its texture
                links.new(texture.outputs["Color"], bsdf.inputs["Emission Color"])
                bsdf.inputs["Emission Strength"].default_value = finish.emission
        return material

    def link_points(self, links: tuple[str, ...]) -> np.ndarray:
        """Return the world-space vertices of the given links' visuals (all links when empty)."""
        missing = [n for n in links if n not in self.objects]
        if missing:
            raise UsageError(f"camera target(s) without visuals: {', '.join(missing)}")
        objects = [o for n in (links or self.objects) for o in self.objects[n]]
        points = []
        for obj in objects:
            local = np.empty(len(obj.data.vertices) * 3, dtype=np.float32)
            obj.data.vertices.foreach_get("co", local)
            matrix = np.array(obj.matrix_world)
            points.append(local.reshape(-1, 3) @ matrix[:3, :3].T + matrix[:3, 3])
        return np.concatenate(points)

    # -- environment --------------------------------------------------------------------------

    def add_lights(self, azimuth: float, center: np.ndarray, radius: float) -> None:
        """Light the robot like a product shot: key, fill, rim and top softboxes.

        The fill and top lights stay well below the key, so white surfaces keep a visible
        gradient and their shape reads against a light background.

        The rig turns with the camera's azimuth, so every view gets the same light from the
        viewer's standpoint. Powers scale with the square of the distance to keep exposure.
        """
        distance = max(3.0 * radius, 2.0)
        softboxes = (
            # name, azimuth offset, elevation, size (fraction of distance), watts at 1 m
            ("key", 50.0, 35.0, 0.55, 14.0),
            ("fill", -65.0, 15.0, 0.8, 1.5),
            ("rim", 165.0, 40.0, 0.4, 20.0),
            ("top", 0.0, 85.0, 0.6, 2.0),
        )
        for name, offset, elevation, size, watts in softboxes:
            light = bpy.data.lights.new(name, type="AREA")
            light.shape = "DISK"
            light.size = size * distance
            light.energy = watts * distance**2
            obj = bpy.data.objects.new(name, light)
            direction = spherical(azimuth + offset, elevation)
            obj.location = Vector(center + distance * direction)
            obj.rotation_euler = Vector(-direction).to_track_quat("-Z", "Y").to_euler()
            self.scene.collection.objects.link(obj)

    def add_world(self, background: tuple[float, float, float], hdri: Path | None) -> None:
        """Light with the HDRI (or a dim even fill) while the camera sees the plain backdrop."""
        world = bpy.data.worlds.new("studio")
        self.scene.world = world
        nodes, links = world.node_tree.nodes, world.node_tree.links
        backdrop = nodes["Background"]
        backdrop.inputs["Color"].default_value = (*background, 1.0)
        environment = nodes.new("ShaderNodeBackground")
        if hdri is None:
            environment.inputs["Color"].default_value = (*background, 1.0)
            environment.inputs["Strength"].default_value = 0.2
        else:
            texture = nodes.new("ShaderNodeTexEnvironment")
            texture.image = bpy.data.images.load(str(hdri), check_existing=True)
            links.new(texture.outputs["Color"], environment.inputs["Color"])
            environment.inputs["Strength"].default_value = 0.25
        path = nodes.new("ShaderNodeLightPath")
        mix = nodes.new("ShaderNodeMixShader")
        links.new(path.outputs["Is Camera Ray"], mix.inputs["Fac"])
        links.new(environment.outputs["Background"], mix.inputs[1])
        links.new(backdrop.outputs["Background"], mix.inputs[2])
        links.new(mix.outputs["Shader"], nodes["World Output"].inputs["Surface"])

    def add_ground(self, height: float) -> None:
        """Add a floor that only catches shadows: the backdrop or transparency shows elsewhere."""
        mesh = bpy.data.meshes.new("ground")
        corners = [(50.0 * x, 50.0 * y, height) for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        mesh.from_pydata(corners, [], [(0, 1, 2, 3)])
        ground = bpy.data.objects.new("ground", mesh)
        ground.is_shadow_catcher = True
        self.scene.collection.objects.link(ground)

    def add_camera(self, camera: Camera, points: np.ndarray, resolution: tuple[int, int]) -> None:
        """Place the camera at the given angles so the target points just fill the frame."""
        data = bpy.data.cameras.new("camera")
        data.lens = camera.lens
        data.clip_start, data.clip_end = 0.01, 200.0
        obj = bpy.data.objects.new("camera", data)
        self.scene.collection.objects.link(obj)
        self.scene.camera = obj
        toward_camera = spherical(camera.azimuth, camera.elevation)
        rotation = np.array(Vector(toward_camera).to_track_quat("Z", "Y").to_matrix())
        tangents = half_angle_tangents(data.lens, data.sensor_width, resolution) / camera.margin
        location = frame_points(points, rotation, tangents)
        world = np.eye(4)
        world[:3, :3], world[:3, 3] = rotation, location
        obj.matrix_world = Matrix(world.tolist())


def spherical(azimuth: float, elevation: float) -> np.ndarray:
    """Return the unit vector at an azimuth (from +x toward +y) and elevation, in degrees."""
    a, e = math.radians(azimuth), math.radians(elevation)
    return np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])


def half_angle_tangents(lens: float, sensor: float, resolution: tuple[int, int]) -> np.ndarray:
    """Return tan of the horizontal and vertical half view angles; the sensor spans the long side."""
    width, height = resolution
    long_side = sensor / 2.0 / lens
    if width >= height:
        return np.array([long_side, long_side * height / width])
    return np.array([long_side * width / height, long_side])


def frame_points(points: np.ndarray, rotation: np.ndarray, tangents: np.ndarray) -> np.ndarray:
    """Return the camera position, at the given orientation, whose view just holds the points.

    In camera axes (x right, y up, z backward), a point stays in view while its lateral offset is
    at most tan(half angle) times its depth. Per axis, the two outermost points pin both the
    camera's offset and its distance; the larger distance of the two axes wins.
    """
    local = points @ rotation  # rows: points in camera axes
    offsets, distances = [], []
    for axis in (0, 1):
        upper = np.max(local[:, axis] + tangents[axis] * local[:, 2])
        lower = np.min(local[:, axis] - tangents[axis] * local[:, 2])
        offsets.append((upper + lower) / 2.0)
        distances.append((upper - lower) / (2.0 * tangents[axis]))
    return rotation @ np.array([offsets[0], offsets[1], max(distances)])


def facing_angles(
    poses: dict[str, np.ndarray], facing: tuple[str, int] | None
) -> tuple[float, float]:
    """Return azimuth and elevation, in degrees, of the direction a frame's axis points."""
    if facing is None:
        return 0.0, 0.0
    frame, axis = facing
    x, y, z = poses[frame][:3, axis]
    return math.degrees(math.atan2(y, x)), math.degrees(math.asin(max(-1.0, min(1.0, z))))


# =================================================================================================
# Render settings, the HDRI download, and the command line
# =================================================================================================


def use_gpu(allow: bool) -> str:
    """Enable the first GPU backend Cycles finds (OptiX before CUDA); fall back to the CPU."""
    scene = bpy.context.scene
    if allow:
        preferences = bpy.context.preferences.addons["cycles"].preferences
        for backend in ("OPTIX", "CUDA", "HIP", "METAL", "ONEAPI"):
            try:
                preferences.compute_device_type = backend
            except TypeError:
                continue
            preferences.get_devices()
            gpus = [d for d in preferences.devices if d.type == backend]
            if gpus:
                for device in preferences.devices:
                    device.use = device.type == backend
                scene.cycles.device = "GPU"
                return f"{backend} ({', '.join(d.name for d in gpus)})"
    scene.cycles.device = "CPU"
    return "CPU"


def download(url: str) -> bytes:
    """Fetch a URL; Poly Haven refuses Python's default user agent."""
    request = urllib.request.Request(url, headers={"User-Agent": "garmi_description-render"})
    with urllib.request.urlopen(request, timeout=60) as response:
        return response.read()


def fetch_hdri(name: str) -> tuple[Path | None, str]:
    """Return a Poly Haven HDRI (CC0, 2k) and its credit, downloading both on first use."""
    path = CACHE / "hdri" / f"{name}_2k.hdr"
    credit = path.with_suffix(".txt")
    if not path.is_file():
        try:
            info = json.loads(download(f"https://api.polyhaven.com/info/{name}"))
            data = download(f"https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/2k/{name}_2k.hdr")
        except (OSError, ValueError) as error:
            print(f"warning: HDRI '{name}' unavailable ({error}); lighting with the softboxes only")
            return None, ""
        path.parent.mkdir(parents=True, exist_ok=True)
        authors = ", ".join(info.get("authors", {}))
        credit.write_text(f"HDRI '{info.get('name', name)}' by {authors}, polyhaven.com, CC0\n")
        path.with_suffix(".part").write_bytes(data)
        path.with_suffix(".part").replace(path)
    return path, credit.read_text().strip()


def configure_render(args: argparse.Namespace, resolution: tuple[int, int]) -> str:
    """Set Cycles: device, samples, denoising, colour management and the output format."""
    scene = bpy.context.scene
    device = use_gpu(allow=not args.cpu)
    cycles = scene.cycles
    cycles.samples = args.samples
    cycles.use_adaptive_sampling = True
    cycles.use_denoising = True
    cycles.denoiser = "OPENIMAGEDENOISE"
    cycles.denoising_input_passes = "RGB_ALBEDO_NORMAL"
    cycles.denoising_prefilter = "ACCURATE"
    cycles.denoising_use_gpu = device != "CPU"
    cycles.caustics_reflective = cycles.caustics_refractive = False
    render = scene.render
    render.resolution_x, render.resolution_y = resolution
    render.resolution_percentage = 100
    render.film_transparent = args.transparent
    render.image_settings.file_format = "PNG"
    render.image_settings.color_mode = "RGBA" if args.transparent else "RGB"
    # A neutral tone map keeps the model's colours where they are, unlike AgX's desaturation.
    scene.view_settings.view_transform = "Khronos PBR Neutral"
    scene.view_settings.look = "None"
    scene.view_settings.exposure = args.exposure
    return device


def parse_resolution(text: str) -> tuple[int, int]:
    """Parse WIDTHxHEIGHT."""
    match = re.fullmatch(r"(\d+)x(\d+)", text)
    if not match:
        raise argparse.ArgumentTypeError(f"'{text}' is not WIDTHxHEIGHT, e.g. 1200x1500")
    return int(match[1]), int(match[2])


def parse_colour(text: str) -> tuple[float, float, float]:
    """Parse a #rrggbb sRGB colour into linear light."""
    match = re.fullmatch(r"#?([0-9a-fA-F]{6})", text)
    if not match:
        raise argparse.ArgumentTypeError(f"'{text}' is not a #rrggbb colour")
    return tuple(srgb_to_linear(int(match[1][i : i + 2], 16) / 255) for i in (0, 2, 4))


def arguments(argv: list[str]) -> argparse.Namespace:
    """Parse the command line."""
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="plain URDF to render")
    parser.add_argument("-o", "--output", type=Path, help="PNG to write [garmi_<view>_<pose>.png]")
    parser.add_argument("--pose", choices=POSES, default="manipulation", help="joint pose preset")
    parser.add_argument(
        "--joint",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="set one joint (radians or metres) on top of the pose; repeatable",
    )
    parser.add_argument("--view", choices=VIEWS, default="front-left", help="camera preset")
    parser.add_argument(
        "--camera",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="override the preset: azimuth, elevation (degrees from the direction the target "
        "faces), lens (mm), margin (1 = tight), target (comma-separated links); repeatable",
    )
    parser.add_argument("--resolution", type=parse_resolution, help="WIDTHxHEIGHT [per view]")
    parser.add_argument("--samples", type=int, default=256, help="path-tracing samples per pixel")
    parser.add_argument(
        "--exposure",
        type=float,
        default=-0.4,
        help="exposure in stops (default: %(default)s; lower keeps more shape in the whites)",
    )
    parser.add_argument(
        "--transparent", action="store_true", help="transparent background (RGBA PNG)"
    )
    parser.add_argument(
        "--no-ground", dest="ground", action="store_false", help="drop the shadow-catching floor"
    )
    parser.add_argument(
        "--background",
        type=parse_colour,
        default="#f2f3f5",
        help="backdrop colour (#rrggbb) when not transparent",
    )
    parser.add_argument(
        "--hdri",
        default="studio_small_08",
        help="Poly Haven HDRI for light and reflections, or 'none' for the softboxes alone",
    )
    parser.add_argument(
        "--turntable",
        type=int,
        default=0,
        metavar="FRAMES",
        help="render FRAMES PNGs of one robot turn (output_0000.png, ...) instead of one image",
    )
    parser.add_argument(
        "--fade-shadow",
        action="store_true",
        help="with --transparent: fade the floor shadow out around the base, inside the image",
    )
    parser.add_argument(
        "--crop", action="store_true", help="crop the image to the robot and its shadow"
    )
    parser.add_argument(
        "--width", type=int, help="scale the finished image to this width in pixels"
    )
    parser.add_argument("--cpu", action="store_true", help="render on the CPU even if a GPU exists")
    args = parser.parse_args(argv)
    if args.fade_shadow and not (args.transparent and args.ground):
        parser.error("--fade-shadow needs --transparent and the floor")
    return args


def render(args: argparse.Namespace) -> None:
    """Build the scene the arguments describe and render it."""
    started = time.monotonic()
    robot = load_urdf(args.urdf)
    positions = joint_positions(robot, args.pose, args.joint)
    poses = robot.forward_kinematics(positions)
    camera = camera_settings(args.view, args.camera)
    resolution = args.resolution or camera.resolution
    output = args.output or Path(f"garmi_{args.view}_{args.pose}.png")

    studio = Studio()
    studio.add_robot(robot, poses)
    hdri, credit = fetch_hdri(args.hdri) if args.hdri != "none" else (None, "")
    whole = studio.link_points(())
    center = (whole.min(axis=0) + whole.max(axis=0)) / 2.0
    radius = float(np.linalg.norm(whole.max(axis=0) - whole.min(axis=0))) / 2.0
    base_azimuth, base_elevation = facing_angles(poses, camera.facing)
    camera = replace(
        camera, azimuth=camera.azimuth + base_azimuth, elevation=camera.elevation + base_elevation
    )
    studio.add_lights(camera.azimuth, center, radius)
    studio.add_world(args.background, hdri)
    if args.ground:
        studio.add_ground(float(whole[:, 2].min()))
    points = studio.link_points(camera.targets)
    frames = max(args.turntable, 1)
    angles = [2.0 * math.pi * i / frames for i in range(frames)]
    if args.turntable:  # frame the robot in every orientation it turns through
        points = np.concatenate([points @ axis_angle(np.array([0, 0, 1.0]), a).T for a in angles])
    studio.add_camera(camera, points, resolution)
    device = configure_render(args, resolution)
    print(f"scene built in {time.monotonic() - started:.1f} s; rendering on {device}")
    if credit:
        print(credit)

    output.parent.mkdir(parents=True, exist_ok=True)
    for index, angle in enumerate(angles):
        studio.rig.rotation_euler = (0.0, 0.0, angle)
        target = output.with_name(f"{output.stem}_{index:04d}.png") if args.turntable else output
        studio.scene.render.filepath = str(target.resolve())
        frame_started = time.monotonic()
        bpy.ops.render.render(write_still=True)
        if args.fade_shadow or args.crop or args.width:
            finish_image(target, fade=args.fade_shadow, crop=args.crop, width=args.width)
        print(f"wrote {target} in {time.monotonic() - frame_started:.1f} s")
    print(f"total {time.monotonic() - started:.1f} s")


def area_weights(old: int, new: int) -> np.ndarray:
    """Return the (new, old) matrix averaging each new pixel over the old pixels it covers."""
    edges = np.linspace(0.0, old, new + 1)
    weights = np.zeros((new, old), dtype=np.float32)
    for i in range(new):
        start, end = edges[i], edges[i + 1]
        for j in range(int(start), min(int(np.ceil(end)), old)):
            weights[i, j] = min(end, j + 1) - max(start, j)
    return weights / weights.sum(axis=1, keepdims=True)


def area_resize(rgba: np.ndarray, width: int, height: int) -> np.ndarray:
    """Downscale an RGBA array by area averaging, in premultiplied alpha (clean cut-out edges)."""
    rows, columns = area_weights(rgba.shape[0], height), area_weights(rgba.shape[1], width)
    premultiplied = rgba.copy()
    premultiplied[..., :3] *= rgba[..., 3:4]
    # Rows, then columns: two matrix products instead of one four-index sum.
    scaled = np.tensordot(rows, premultiplied, axes=(1, 0))  # (height, old width, 4)
    scaled = np.tensordot(scaled, columns, axes=(1, 1)).transpose(0, 2, 1)  # (height, width, 4)
    alpha = scaled[..., 3:4]
    scaled[..., :3] = np.divide(
        scaled[..., :3], alpha, out=np.zeros_like(scaled[..., :3]), where=alpha > 0
    )
    return scaled


def finish_image(path: Path, fade: bool, crop: bool, width: int | None) -> None:
    """Post-process a rendered PNG in place: fade the floor shadow, crop, scale.

    The floor shadow of the large softboxes reaches far beyond the robot, and a transparent
    image would cut it off at its border. With `fade`, the shadow (every pixel not fully opaque)
    fades out smoothly in an ellipse around the bottom of the base, inside the image; the robot
    itself is never touched. `crop` keeps the robot and what is left of its shadow, with a small
    margin, and `width` scales the result down by area averaging.
    """
    image = bpy.data.images.load(str(path), check_existing=False)
    height, full_width = image.size[1], image.size[0]
    pixels = np.empty(full_width * height * 4, dtype=np.float32)
    image.pixels.foreach_get(pixels)
    rgba = np.flipud(pixels.reshape(height, full_width, 4)).copy()  # rows top to bottom
    alpha = rgba[..., 3]
    robot = alpha > 0.98
    if fade and robot.any():
        rows, columns = np.nonzero(robot)
        centre, bottom = (columns.min() + columns.max()) / 2.0, float(rows.max())
        half = max((columns.max() - columns.min()) / 2.0, 1.0)
        y, x = np.mgrid[0:height, 0:full_width].astype(np.float32)
        reach = np.sqrt(((x - centre) / (1.05 * half)) ** 2 + ((y - bottom) / (0.33 * half)) ** 2)
        weight = np.clip((1.45 - reach) / 0.7, 0.0, 1.0)
        weight = weight * weight * (3.0 - 2.0 * weight)
        alpha[~robot] *= weight[~robot]
    if crop:
        rows, columns = np.nonzero(alpha > 2.0 / 255.0)
        if rows.size:
            pad = int(0.02 * full_width)
            top, low = max(rows.min() - pad, 0), min(rows.max() + pad + 1, height)
            left, right = max(columns.min() - pad, 0), min(columns.max() + pad + 1, full_width)
            rgba = rgba[top:low, left:right]
    if width:
        rgba = area_resize(rgba, width, max(round(rgba.shape[0] * width / rgba.shape[1]), 1))
    finished = bpy.data.images.new(
        path.stem, width=rgba.shape[1], height=rgba.shape[0], alpha=True, float_buffer=False
    )  # an 8-bit buffer saves an 8-bit PNG, like the render itself
    # The values are the PNG's own (display-encoded); set the colour space before the pixels, as
    # setting it later regenerates the buffer and drops the pixels.
    finished.colorspace_settings.name = image.colorspace_settings.name
    finished.pixels.foreach_set(np.ascontiguousarray(np.flipud(rgba)).ravel())
    finished.filepath_raw = str(path)
    finished.file_format = "PNG"
    finished.save()
    bpy.data.images.remove(image)
    bpy.data.images.remove(finished)


def main() -> None:
    """Run the command line; model and option errors end as a usage message."""
    args = arguments(sys.argv[1:])
    try:
        render(args)
    except UsageError as error:
        sys.exit(f"render.py: error: {error}")


if __name__ == "__main__":
    main()
