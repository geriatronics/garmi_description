# Rendering Garmi

[`render.py`](render.py) renders the robot model photorealistically with Blender's Cycles path
tracer: for README images, slides, tickets and the website. It reads the plain URDF
([`garmi_description/urdf/garmi.urdf`](../../garmi_description/urdf/garmi.urdf)), poses it with its
own forward kinematics, loads every visual mesh with its colours and textures, gives each surface a
physically based material and lights it like a studio product shot.

The only thing to install is [uv](https://docs.astral.sh/uv/getting-started/installation/). The
script declares its dependencies inline, so `uv run` fetches Python 3.13 and Blender as the `bpy`
module (about 900 MB on the first run, cached afterwards) and nothing else lands on your system.
Blender is GPL-licensed and installed by uv at run time; it is not part of this repository.

```bash
# From the repository root. The whole robot, transparent, for a README:
uv run tools/render/render.py --transparent -o garmi.png

# On a floor with its shadow, in the arms' driving posture:
uv run tools/render/render.py --pose travel -o garmi_travel.png

# Close-ups of the head and of the left wrist camera:
uv run tools/render/render.py --view head -o garmi_head.png
uv run tools/render/render.py --view wrist -o garmi_wrist.png

# 36 frames of one turn (garmi_turn_0000.png ... garmi_turn_0035.png):
uv run tools/render/render.py --turntable 36 --samples 128 -o turn/garmi_turn.png
```

On a laptop RTX 5090 (OptiX) each image above takes 6 to 8 seconds at the default 256 samples,
plus about 1.5 seconds to build the scene. Without a GPU, Cycles renders on the CPU, which is many
times slower; lower `--samples` or `--resolution` for drafts. `--help` lists every option.

## Options

| Option | Default | What it does |
| --- | --- | --- |
| `--pose` | `manipulation` | Joint pose preset, see below. |
| `--joint NAME=VALUE` | | Sets one joint (radians, metres) on top of the pose; repeatable. Mimic joints follow their source; a value outside the joint's limits is rendered with a warning. |
| `--view` | `front-left` | Camera preset, see below. |
| `--camera KEY=VALUE` | | Overrides the preset: `azimuth`, `elevation` (degrees), `lens` (mm), `margin` (1 frames the targets edge to edge), `target` (comma-separated link names); repeatable. |
| `--resolution WxH` | per view | Image size in pixels. |
| `--samples` | 256 | Path-tracing samples per pixel; OpenImageDenoise cleans up the rest. |
| `--exposure` | -0.4 | Exposure in stops; lower keeps more shape in the white surfaces, higher brightens. |
| `--fade-shadow` | off | With `--transparent`: fades the floor shadow out around the base, so it ends inside the image instead of at its border. |
| `--crop` | off | Crops the image to the robot and its shadow, with a small margin. |
| `--width` | | Scales the finished image down to this width (area averaging). |
| `--transparent` | off | Transparent background: an RGBA PNG whose floor keeps only the robot's shadow. |
| `--no-ground` | | Drops the shadow-catching floor. |
| `--background` | `#f2f3f5` | Backdrop colour when not transparent. |
| `--hdri` | `studio_small_08` | Poly Haven HDRI for light and reflections, or `none`. |
| `--turntable N` | | Renders N frames of one turn of the robot instead of one image. |
| `--cpu` | | Renders on the CPU even when a GPU is there. |
| `--urdf`, `-o` | this repository's URDF, `garmi_<view>_<pose>.png` | Input model and output file. |

## Poses

- `zero`: every joint at zero, as the URDF defines the model; FR3 joints 4 and 6 lie outside their
  limits there.
- `manipulation`: Franka's ready pose on both arms, fingers open; the MuJoCo model's `home`
  keyframe.
- `travel`: both arms folded back over the shoulders, above the base, Garmi's driving posture
  (the right arm mirrors the left).
- `readme`: the left hand raised and tilted 20 degrees down, its wrist camera looking at the
  viewer above the fingertips; the right gripper held out sideways, its length in view. Seen from
  the front, it makes the repository's README image:

  ```bash
  uv run tools/render/render.py --pose readme --camera azimuth=0 --camera margin=1.6 \
      --resolution 1600x1600 --transparent --fade-shadow --crop --width 600 \
      -o garmi_description/model.png
  ```

## Views

Each view frames its target links so they just fill the image, from the given azimuth and
elevation. The angles count from the direction the target faces, so a preset follows the joints:

| View | Frames | Faces along | Azimuth, elevation | Lens | Size |
| --- | --- | --- | --- | --- | --- |
| `front-left` | the whole robot | the base's front (+x) | 35, 12 | 70 mm | 1200x1500 |
| `head` | head, neck and head camera | the head's +x | 30, 8 | 100 mm | 1600x1200 |
| `wrist` | left wrist camera and bracket, the hand around them | the D435's colour optical axis | 35, 20 | 100 mm | 1600x1200 |

`--camera target=right_fr3_wrist_camera_mount,camera_d435i_right_link` frames the right wrist
camera instead. A close-up from a steep angle can put the camera inside a neighbouring part; turn
the azimuth or elevation if the image comes out dark or white.

## Materials

Colours are the model's own; the script only decides how each kind of surface reflects.

- **Colour sources**: URDF `rgba` values are sRGB, as RViz shows them, and are converted to linear
  light. The Collada effect colours and the OBJ meshes' MTL `Kd` values are linear already, the
  way Blender writes them; textures (`fabric.jpg`, `face.jpg`) are sRGB images. A white of 1.0 is
  capped at an albedo of 0.80, as bright as real white paint gets.
- **Roughness**: where a file states it, it is used: the MTL's `Ns` (Blender writes
  `Ns = 1000 (1 - roughness)^2`) and a Collada Phong exponent `n` (as `sqrt(2 / (n + 2))`).
- **Finish by material name** ([`FINISHES`](render.py)): the RealSense's black glass front and
  lenses are glossy with a clear coat, its housing anodised aluminium, the torso's fabric a rough
  sheen, the chest and face displays dark glass (the face's eyes glow from its texture), the
  fingertips rubber, the camera mounts matte 3D print, the base and lift satin paint, and Franka's
  shells, like anything unnamed, satin paint with a thin clear coat.

## Light and output

Four disc softboxes (key, fill, rim, top) turn with the camera, so every view is lit alike; fill
and top stay well below the key, so white surfaces keep a gradient that shows their shape. An
HDRI from [Poly Haven](https://polyhaven.com/hdris) (CC0) adds soft light and reflections. The HDRI
downloads once into `~/.cache/garmi_description/render/hdri/`, and each run prints its credit;
without a connection the script warns and renders with the softboxes alone. The camera always sees
the plain `--background`, never the HDRI. The floor is a shadow catcher: it shows the robot's
shadow on the backdrop, or in the alpha channel with `--transparent`. Colours are tone-mapped with
Khronos PBR Neutral, which keeps them where the model puts them.

## Limitations

- Visual geometry only: URDF `<mesh>` (Collada, OBJ with MTL, STL) and `<box>`. Cylinders and
  spheres are refused; Collada meshes must be triangulated; mesh URIs must be `package://` with
  the package's `package.xml` above the URDF.
- Materials follow the model's colours, so they are as right as those: the Clearpath side covers
  render yellow and the base's lights as plain white and red plastic.
- No depth of field, motion blur or video encoding; turn turntable frames into a video with your
  own tool, e.g. `ffmpeg -framerate 24 -i turn/garmi_turn_%04d.png garmi_turn.mp4`.
