# Garmi — MuJoCo model

A [MuJoCo](https://mujoco.org/) (MJCF) version of the Garmi robot, kept
alongside the URDF so the model can be used directly in MuJoCo-based projects,
benchmarks and RL environments.

```
mujoco/
├── garmi.xml     # the robot (include this in your own scene)
├── scene.xml     # garmi.xml + floor + light  (viewer entry point)
├── assets/       # meshes, textures (committed; regenerate with build/)
└── build/        # scripts to regenerate assets/ (not needed at runtime)
```

## Quick start

From the repository root:

```bash
docker compose up mujoco          # interactive viewer (drag joints, apply forces)
docker compose up mujoco-teleop   # viewer + a twist teleop panel for the base
```

`mujoco` opens the interactive MuJoCo viewer (you can drag joints, apply
forces, and drive the actuators).

`mujoco-teleop` additionally opens a small control panel ([`teleop.py`](teleop.py))
to drive the mecanum base by **twist** instead of poking individual wheels: drag
the joystick for linear x/y, use the slider for turn rate (both spring back to
zero on release). The twist is mapped to the four wheel actuators with the
mecanum inverse kinematics.

The **"closed loop" checkbox** (on by default) runs a feedforward + proportional
controller on the base's measured twist. Open loop, the base drifts and
over-rotates (mecanum inverse kinematics assumes ideal no-slip rolling); closed
loop tracks the commanded twist closely and — like the real joystick-driven
robot — stops without overshooting (the feedforward keeps the controller from
winding up, so releasing the joystick doesn't cause a reverse lurch). This is
the role wheel-odometry/IMU feedback plays on the real robot (here using
MuJoCo's ground-truth base velocity as perfect odometry). Toggle it off to feel
the raw open-loop drift.

The joystick itself ([`../scripts/twist_joystick.py`](../scripts/twist_joystick.py))
is a dependency-free Tk widget. The Gazebo example offers the same control as an
rqt plugin (in the single rqt window of `docker compose up gazebo`), so driving
feels the same in both simulators. Locally, with the `mujoco` Python package installed:

```bash
cd garmi_description/mujoco
python view.py                              # opens in the home (ready) pose
# or, plain (starts at the zero configuration):
python -m mujoco.viewer --mjcf=scene.xml
```

`view.py` loads the `home` keyframe so the robot opens in the same ready pose
the Gazebo model starts in; the bare `mujoco.viewer` command starts at the
zero configuration (arms flat).

`garmi.xml` is self-contained (no floor/light) so you can `<include>` it in your
own scene or attach it to other models.

## What's in the model

| Part | DoF | Actuator |
| --- | --- | --- |
| Mobile base | free joint + 4 mecanum wheels | 4 wheel velocity servos |
| Lift / torso | 2 prismatic (coupled, telescoping) | 1 position servo |
| Head | pan + tilt | 2 position servos |
| Left / right arm | 2 × 7 (Franka FR3) | 2 × 7 position servos |
| Grippers | 2 × Franka Hand | 2 tendon servos |

A `home` keyframe places the arms in the same ready pose the Gazebo model
starts in.

**Collisions & limits.** The arm links and grippers have collision geometry and
also collide with coarse self-collision volumes around the torso, head and
mobile base, so the arms cannot pass through the body. Joint position limits and
the wheel speed limit mirror the URDF; the lift is heavily damped to reproduce
the slow, stiff, non-backdrivable lead-screw of the real robot (~0.088 m/s per
stage). Both telescoping stages move together via a stiff equality constraint
(the URDF `mimic`), giving ~0.8 m of total travel.

## The mobile base (mecanum)

The base is a **free-floating** rigid body — it has full dynamics and can be
pushed, lifted or toppled, just like the Gazebo model. Each wheel is a driven
hub carrying several **free-spinning rollers** mounted at 45°. Spinning a wheel
therefore grips perpendicular to the rollers and slips along them, which
produces real holonomic (omnidirectional) motion from physics alone.

This differs from Gazebo, which fakes the same effect with a fixed friction
direction (`mu`/`mu2`/`fdir1`) — a knob MuJoCo does not expose, hence the
explicit rollers here.

Notes:
- **Driving:** command the four `wheel_*` velocity actuators. All wheels the
  same sign → drive forward/back; diagonal patterns → strafe; opposing
  left/right → rotate. (Use the standard mecanum inverse kinematics to map a
  body twist to wheel speeds.)
- **Open-loop drift:** like a real mecanum base, open-loop strafing exhibits
  some yaw drift; close the loop (odometry/IMU) for accurate motion.
- **Tuning knobs:** the rollers and their contact are authored directly in
  `garmi.xml` — adjust the `roller` default class (`friction`/`solref`/`solimp`)
  and the per-wheel roller bodies.

## Regenerating the assets

The meshes in `assets/` are committed, so nothing below is needed to *use* the
model. To regenerate them (e.g. after updating a source mesh):

```bash
pip install -r build/requirements.txt
python build/build_assets.py
```

This downloads the FR3 + Franka Hand meshes from
[mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)
(pinned revision), copies Garmi's own base/lift meshes from `../meshes`, and
runs [`obj2mjcf`](https://github.com/kevinzakka/obj2mjcf) on the multi-material
torso/head/cover parts (splitting by material, converting textures to PNG).

## Maintaining two versions

The URDF ([`../urdf/garmi.urdf`](../urdf/garmi.urdf)) remains the source of
truth for kinematics. `garmi.xml` mirrors it: joint axes/limits, link offsets
and the arm mount transforms are taken directly from the URDF. If you change
those in the URDF, update the corresponding values in `garmi.xml`. The FR3 arm
and hand structure is reused from MuJoCo Menagerie and is unlikely to change.

## Licensing

Apache-2.0 (see the repository `LICENSE`). The FR3 arm and Franka Hand assets
are reused from MuJoCo Menagerie (Apache-2.0); see the repository `NOTICE` and
`THIRD_PARTY_LICENSES` for full attribution.
